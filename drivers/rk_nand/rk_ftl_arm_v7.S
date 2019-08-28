/*
 * Copyright (c) 2016-2018, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-06-03
 */
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
	.file	"rk_ftl_arm_v7.S"
	.text
	.align	2
	.global	FlashMemCmp8
	.type	FlashMemCmp8, %function
FlashMemCmp8:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L11
	str	lr, [sp, #-4]!
	.save {lr}
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L4
	ldrb	r3, [r1, #1]	@ zero_extendqisi2
	ldrb	ip, [r0, #1]	@ zero_extendqisi2
	cmp	ip, r3
	movne	r3, #0
	beq	.L8
.L4:
	cmp	r3, r2
	beq	.L8
	ldrb	lr, [r0, r3]	@ zero_extendqisi2
	ldrb	ip, [r1, r3]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	lr, ip
	beq	.L4
	mov	r0, r3
	ldr	pc, [sp], #4
.L8:
	mov	r0, #0
	ldr	pc, [sp], #4
.L12:
	.align	2
.L11:
	.word	.LANCHOR0
	.fnend
	.size	FlashMemCmp8, .-FlashMemCmp8
	.align	2
	.global	FlashRsvdBlkChk
	.type	FlashRsvdBlkChk, %function
FlashRsvdBlkChk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L14
	ldrb	r2, [r3, #1]	@ zero_extendqisi2
	ldr	r3, [r3, #4]
	mul	r3, r3, r2
	cmp	r1, r3
	movcs	r2, #0
	movcc	r2, #1
	cmp	r0, #0
	movne	r2, #0
	eor	r0, r2, #1
	bx	lr
.L15:
	.align	2
.L14:
	.word	.LANCHOR0
	.fnend
	.size	FlashRsvdBlkChk, .-FlashRsvdBlkChk
	.align	2
	.global	FlashGetRandomizer
	.type	FlashGetRandomizer, %function
FlashGetRandomizer:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	and	r3, r1, #127
	ldr	r2, .L25
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r3, r3, asl #1
	ldrh	r4, [r2, r3]
	ldr	r3, .L25+4
	ldrb	r3, [r3, #8]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L17
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r4, r4, #-1073741824
.L17:
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L26:
	.align	2
.L25:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashGetRandomizer, .-FlashGetRandomizer
	.align	2
	.global	FlashSetRandomizer
	.type	FlashSetRandomizer, %function
FlashSetRandomizer:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	and	r3, r1, #127
	ldr	r2, .L36
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r3, r3, asl #1
	mov	r6, r0
	ldrh	r5, [r2, r3]
	ldr	r3, .L36+4
	ldrb	r2, [r3, #8]	@ zero_extendqisi2
	mov	r4, r3
	cmp	r2, #0
	beq	.L28
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r5, r5, #-1073741824
.L28:
	add	r4, r4, r6, asl #3
	ldr	r3, [r4, #12]
	str	r5, [r3, #336]
	ldmfd	sp!, {r4, r5, r6, pc}
.L37:
	.align	2
.L36:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashSetRandomizer, .-FlashSetRandomizer
	.align	2
	.global	FlashBlockAlignInit
	.type	FlashBlockAlignInit, %function
FlashBlockAlignInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, #256
	ldr	r3, .L43
	movhi	r2, #512
	bhi	.L42
	cmp	r0, #128
	strls	r0, [r3, #4]
	bxls	lr
	mov	r2, #256
.L42:
	str	r2, [r3, #4]
	bx	lr
.L44:
	.align	2
.L43:
	.word	.LANCHOR0
	.fnend
	.size	FlashBlockAlignInit, .-FlashBlockAlignInit
	.align	2
	.global	FlashReadCmd
	.type	FlashReadCmd, %function
FlashReadCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	ip, .L46
	mov	r2, r1, lsr #16
	add	ip, ip, r0, asl #3
	ldr	r3, [ip, #12]
	ldrb	ip, [ip, #16]	@ zero_extendqisi2
	add	r3, r3, ip, asl #8
	mov	ip, #0
	str	ip, [r3, #2056]
	str	ip, [r3, #2052]
	str	ip, [r3, #2052]
	uxtb	ip, r1
	str	ip, [r3, #2052]
	mov	ip, r1, lsr #8
	str	ip, [r3, #2052]
	str	r2, [r3, #2052]
	mov	r2, #48
	str	r2, [r3, #2056]
	b	FlashSetRandomizer
.L47:
	.align	2
.L46:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadCmd, .-FlashReadCmd
	.align	2
	.global	FlashReadDpDataOutCmd
	.type	FlashReadDpDataOutCmd, %function
FlashReadDpDataOutCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L53
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	r3, ip, r0, asl #3
	ldrb	ip, [ip, #60]	@ zero_extendqisi2
	uxtb	r4, r1
	ldr	r2, [r3, #12]
	mov	lr, r1, lsr #8
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	cmp	ip, #1
	mov	ip, r1, lsr #16
	mov	r3, r3, asl #8
	add	r3, r2, r3
	bne	.L49
	mov	r2, #6
	str	r2, [r3, #2056]
	mov	r2, #0
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
	str	r4, [r3, #2052]
	str	lr, [r3, #2052]
	str	ip, [r3, #2052]
	b	.L52
.L49:
	mov	r2, #0
	str	r2, [r3, #2056]
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
	str	r4, [r3, #2052]
	str	lr, [r3, #2052]
	str	ip, [r3, #2052]
	mov	ip, #5
	str	ip, [r3, #2056]
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
.L52:
	mov	r2, #224
	str	r2, [r3, #2056]
	ldmfd	sp!, {r4, lr}
	b	FlashSetRandomizer
.L54:
	.align	2
.L53:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadDpDataOutCmd, .-FlashReadDpDataOutCmd
	.align	2
	.global	FlashProgFirstCmd
	.type	FlashProgFirstCmd, %function
FlashProgFirstCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	ip, .L56
	mov	r2, r1, lsr #16
	add	ip, ip, r0, asl #3
	ldr	r3, [ip, #12]
	ldrb	ip, [ip, #16]	@ zero_extendqisi2
	add	r3, r3, ip, asl #8
	mov	ip, #128
	str	ip, [r3, #2056]
	mov	ip, #0
	str	ip, [r3, #2052]
	str	ip, [r3, #2052]
	uxtb	ip, r1
	str	ip, [r3, #2052]
	mov	ip, r1, lsr #8
	str	ip, [r3, #2052]
	str	r2, [r3, #2052]
	b	FlashSetRandomizer
.L57:
	.align	2
.L56:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgFirstCmd, .-FlashProgFirstCmd
	.align	2
	.global	FlashEraseCmd
	.type	FlashEraseCmd, %function
FlashEraseCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!
	.save {lr}
	cmp	r2, #0
	ldr	lr, .L64
	add	r0, lr, r0, asl #3
	ldrb	r3, [r0, #16]	@ zero_extendqisi2
	ldr	ip, [r0, #12]
	mov	r3, r3, asl #8
	beq	.L59
	add	r2, ip, r3
	mov	r0, #96
	str	r0, [r2, #2056]
	uxtb	r0, r1
	str	r0, [r2, #2052]
	mov	r0, r1, lsr #8
	str	r0, [r2, #2052]
	mov	r0, r1, lsr #16
	str	r0, [r2, #2052]
	ldr	r2, [lr, #4]
	add	r1, r1, r2
.L59:
	add	r3, ip, r3
	mov	r2, #96
	str	r2, [r3, #2056]
	uxtb	r2, r1
	str	r2, [r3, #2052]
	mov	r2, r1, lsr #8
	mov	r1, r1, lsr #16
	str	r2, [r3, #2052]
	str	r1, [r3, #2052]
	mov	r2, #208
	str	r2, [r3, #2056]
	ldr	pc, [sp], #4
.L65:
	.align	2
.L64:
	.word	.LANCHOR0
	.fnend
	.size	FlashEraseCmd, .-FlashEraseCmd
	.align	2
	.global	FlashProgDpSecondCmd
	.type	FlashProgDpSecondCmd, %function
FlashProgDpSecondCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L68
	mov	r2, r1, lsr #16
	str	lr, [sp, #-4]!
	.save {lr}
	add	lr, ip, r0, asl #3
	ldrb	ip, [ip, #55]	@ zero_extendqisi2
	ldr	r3, [lr, #12]
	ldrb	lr, [lr, #16]	@ zero_extendqisi2
	add	r3, r3, lr, asl #8
	str	ip, [r3, #2056]
	mov	ip, #0
	str	ip, [r3, #2052]
	str	ip, [r3, #2052]
	uxtb	ip, r1
	str	ip, [r3, #2052]
	mov	ip, r1, lsr #8
	str	ip, [r3, #2052]
	str	r2, [r3, #2052]
	ldr	lr, [sp], #4
	b	FlashSetRandomizer
.L69:
	.align	2
.L68:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgDpSecondCmd, .-FlashProgDpSecondCmd
	.align	2
	.global	FlashProgSecondCmd
	.type	FlashProgSecondCmd, %function
FlashProgSecondCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L71
	add	r0, r3, r0, asl #3
	ldrb	r2, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r0, #12]
	add	r3, r3, r2, asl #8
	mov	r2, #16
	str	r2, [r3, #2056]
	bx	lr
.L72:
	.align	2
.L71:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgSecondCmd, .-FlashProgSecondCmd
	.align	2
	.global	FlashProgDpFirstCmd
	.type	FlashProgDpFirstCmd, %function
FlashProgDpFirstCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L74
	add	r0, r2, r0, asl #3
	ldrb	r2, [r2, #54]	@ zero_extendqisi2
	ldrb	r1, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r0, #12]
	add	r3, r3, r1, asl #8
	str	r2, [r3, #2056]
	bx	lr
.L75:
	.align	2
.L74:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgDpFirstCmd, .-FlashProgDpFirstCmd
	.align	2
	.global	js_hash
	.type	js_hash, %function
js_hash:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L80
	add	r1, r0, r1
.L77:
	cmp	r0, r1
	beq	.L79
	mov	r2, r3, asl #5
	ldrb	ip, [r0], #1	@ zero_extendqisi2
	add	r2, r2, r3, lsr #2
	add	r2, r2, ip
	eor	r3, r3, r2
	b	.L77
.L79:
	mov	r0, r3
	bx	lr
.L81:
	.align	2
.L80:
	.word	1204201446
	.fnend
	.size	js_hash, .-js_hash
	.align	2
	.global	FlashLoadIdbInfo
	.type	FlashLoadIdbInfo, %function
FlashLoadIdbInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r0, #0
	bx	lr
	.fnend
	.size	FlashLoadIdbInfo, .-FlashLoadIdbInfo
	.align	2
	.global	BuildFlashLsbPageTable
	.type	BuildFlashLsbPageTable, %function
BuildFlashLsbPageTable:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r1
	bne	.L84
	ldr	r3, .L114
.L85:
	mov	r2, r0, asl #1
	strh	r0, [r2, r3]	@ movhi
	add	r0, r0, #1
	cmp	r0, #256
	bne	.L85
.L89:
	mov	r2, #1024
	ldr	r0, .L114+4
	mov	r1, #255
	uxth	r4, r4
	bl	memset
	ldr	r2, .L114
	mov	r3, #0
	add	r0, r2, #512
	b	.L86
.L84:
	cmp	r0, #1
	bne	.L87
	ldr	ip, .L114
	mov	r3, #0
.L88:
	uxth	r2, r3
	mov	lr, r3, asl #1
	cmp	r2, #3
	movls	r0, #0
	movhi	r0, #1
	bics	r1, r0, r3
	add	r3, r3, #1
	movne	r1, #2
	moveq	r1, #3
	cmp	r0, #0
	rsb	r1, r1, r2, asl #1
	movne	r2, r1
	cmp	r3, #256
	strh	r2, [lr, ip]	@ movhi
	bne	.L88
	b	.L89
.L87:
	cmp	r0, #2
	bne	.L90
	ldr	r1, .L114
	mov	r3, #0
.L91:
	uxth	r2, r3
	mov	r0, r3, asl #1
	cmp	r2, #1
	add	r3, r3, #1
	mov	ip, r2, asl #1
	subhi	r2, ip, #1
	cmp	r3, #256
	strh	r2, [r0, r1]	@ movhi
	bne	.L91
	b	.L89
.L90:
	cmp	r0, #3
	bne	.L92
	ldr	ip, .L114
	mov	r3, #0
.L93:
	uxth	r2, r3
	mov	lr, r3, asl #1
	cmp	r2, #5
	movls	r0, #0
	movhi	r0, #1
	bics	r1, r0, r3
	add	r3, r3, #1
	movne	r1, #4
	moveq	r1, #5
	cmp	r0, #0
	rsb	r1, r1, r2, asl #1
	movne	r2, r1
	cmp	r3, #256
	strh	r2, [lr, ip]	@ movhi
	bne	.L93
	b	.L89
.L92:
	cmp	r0, #4
	mov	r3, #0
	bne	.L94
	ldr	r2, .L114+8
	strh	r3, [r2, #76]	@ movhi
	mov	r3, #1
	strh	r0, [r2, #84]	@ movhi
	strh	r3, [r2, #78]	@ movhi
	mov	r3, #2
	strh	r3, [r2, #80]	@ movhi
	mov	r3, #3
	strh	r3, [r2, #82]	@ movhi
	mov	r3, #5
	strh	r3, [r2, #86]	@ movhi
	mov	r3, #7
	strh	r3, [r2, #88]	@ movhi
	mov	r3, #8
	strh	r3, [r2, #90]!	@ movhi
.L95:
	tst	r3, #1
	movne	r1, #7
	moveq	r1, #6
	rsb	r1, r1, r3, asl #1
	add	r3, r3, #1
	strh	r1, [r2, #2]!	@ movhi
	uxth	r3, r3
	cmp	r3, #256
	bne	.L95
	b	.L89
.L94:
	cmp	r0, #5
	bne	.L96
	ldr	r2, .L114
.L97:
	mov	r1, r3, asl #1
	strh	r3, [r1, r2]	@ movhi
	add	r3, r3, #1
	cmp	r3, #16
	bne	.L97
	ldr	r2, .L114+12
.L98:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, #496
	bne	.L98
	b	.L89
.L96:
	cmp	r0, #6
	bne	.L99
	ldr	r1, .L114+16
.L100:
	cmp	r3, #5
	add	r2, r3, r3, asl #1
	movls	r0, #0
	movhi	r0, #1
	bics	ip, r0, r3
	movne	ip, #10
	moveq	ip, #12
	cmp	r0, #0
	subne	r2, r2, ip
	moveq	r2, r3
	add	r3, r3, #1
	strh	r2, [r1, #2]!	@ movhi
	uxth	r3, r3
	cmp	r3, #256
	bne	.L100
	b	.L89
.L99:
	cmp	r0, #9
	bne	.L89
	ldr	r2, .L114+8
	movw	r1, #509
	strh	r3, [r2, #76]	@ movhi
	mov	r3, #1
	strh	r3, [r2, #78]	@ movhi
	mov	r3, #2
	strh	r3, [r2, #80]!	@ movhi
	mov	r3, #3
.L101:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, r1
	bne	.L101
	b	.L89
.L86:
	uxth	r1, r3
	cmp	r1, r4
	bcs	.L113
	mov	r1, r3, asl #1
	add	r3, r3, #1
	ldrh	r1, [r1, r2]
	mov	ip, r1, asl #1
	strh	r1, [r0, ip]	@ movhi
	b	.L86
.L113:
	ldmfd	sp!, {r4, pc}
.L115:
	.align	2
.L114:
	.word	.LANCHOR0+76
	.word	.LANCHOR0+588
	.word	.LANCHOR0
	.word	.LANCHOR0+106
	.word	.LANCHOR0+74
	.fnend
	.size	BuildFlashLsbPageTable, .-BuildFlashLsbPageTable
	.align	2
	.global	FlashPrintInfo
	.type	FlashPrintInfo, %function
FlashPrintInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FlashPrintInfo, .-FlashPrintInfo
	.align	2
	.global	FlashDieInfoInit
	.type	FlashDieInfoInit, %function
FlashDieInfoInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L132
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r6, #0
	ldr	r4, .L132+4
	ldrh	r0, [r3, #10]
	ldr	r10, .L132+8
	strb	r6, [r4, #1612]
	mov	r8, r4
	strb	r6, [r4, #1613]
	bl	FlashBlockAlignInit
	add	r0, r4, #1616
	mov	r1, #8
	bl	__memzero
	ldr	r0, .L132+12
	mov	r1, #32
	bl	__memzero
	ldr	r0, .L132+16
	mov	r1, #128
	bl	__memzero
	ldr	r5, [r4, #1784]
	add	r7, r5, #1
.L119:
	mov	r0, r7
	add	r1, r10, r6, asl #3
	ldrb	r2, [r5]	@ zero_extendqisi2
	bl	FlashMemCmp8
	ldr	r9, .L132+8
	cmp	r0, #0
	bne	.L118
	ldrb	r3, [r8, #1612]	@ zero_extendqisi2
	add	r2, r8, r3, asl #2
	str	r0, [r2, #1624]
	add	r2, r3, #1
	add	r3, r8, r3
	strb	r2, [r8, #1612]
	strb	r6, [r3, #1616]
.L118:
	add	r6, r6, #1
	cmp	r6, #4
	bne	.L119
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	ldr	r8, .L132+4
	strb	r3, [r4, #1613]
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	cmp	r3, #2
	beq	.L120
.L124:
	ldrb	r3, [r5, #13]	@ zero_extendqisi2
	ldrb	r2, [r4, #1612]	@ zero_extendqisi2
	smulbb	r2, r2, r3
	ldrh	r3, [r5, #14]
	smulbb	r3, r2, r3
	movw	r2, #1820
	strh	r3, [r4, r2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L120:
	ldr	r10, [r8, #4]
	mov	r6, #0
.L123:
	mov	r0, r7
	add	r1, r9, r6, asl #3
	ldrb	r2, [r5]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L121
	ldrb	r1, [r5, #13]	@ zero_extendqisi2
	ldrh	r3, [r5, #14]
	ldrb	r2, [r8, #1612]	@ zero_extendqisi2
	mul	r1, r10, r1
	and	r3, r3, #65280
	add	r0, r8, r2, asl #2
	mul	r3, r3, r1
	str	r3, [r0, #1624]
	ldrb	r1, [r5, #23]	@ zero_extendqisi2
	cmp	r1, #0
	movne	r3, r3, asl #1
	strne	r3, [r0, #1624]
	add	r3, r2, #1
	add	r2, r4, r2
	strb	r3, [r4, #1612]
	strb	r6, [r2, #1616]
.L121:
	add	r6, r6, #1
	cmp	r6, #4
	bne	.L123
	b	.L124
.L133:
	.align	2
.L132:
	.word	.LANCHOR1+256
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.word	.LANCHOR0+1624
	.word	.LANCHOR0+1656
	.fnend
	.size	FlashDieInfoInit, .-FlashDieInfoInit
	.align	2
	.global	ftl_flash_suspend
	.type	ftl_flash_suspend, %function
ftl_flash_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L135
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
.L136:
	.align	2
.L135:
	.word	.LANCHOR0
	.fnend
	.size	ftl_flash_suspend, .-ftl_flash_suspend
	.global	__aeabi_uidiv
	.align	2
	.global	LogAddr2PhyAddr
	.type	LogAddr2PhyAddr, %function
LogAddr2PhyAddr:
	.fnstart
	@ args = 4, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r9, r3
	ldr	r3, .L143
	mov	r10, r1
	ldr	r7, .L143+4
	mov	r6, r0
	ldr	ip, [r0, #4]
	mov	r8, r2
	ldrh	r4, [r3, #12]
	ldrh	r3, [r3, #14]
	bic	ip, ip, #-2147483648
	ldrh	fp, [r7, #4]
	ubfx	r5, ip, #10, #16
	str	ip, [sp, #4]
	smulbb	r4, r4, r3
	ldrb	r3, [r7]	@ zero_extendqisi2
	mov	r0, r5
	cmp	r3, #1
	moveq	fp, fp, asl #1
	uxth	r4, r4
	uxtheq	fp, fp
	mov	r1, r4
	bl	__aeabi_uidiv
	cmp	r10, #1
	uxth	r0, r0
	ldr	ip, [sp, #4]
	smulbb	r4, r0, r4
	rsb	r5, r4, r5
	ubfx	r4, ip, #0, #10
	uxth	r5, r5
	bne	.L139
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L143+4
	addeq	r4, r3, r4, asl #1
	ldreqh	r4, [r4, #76]
.L139:
	add	ip, r7, r0, asl #2
	ldrb	r3, [sp, #48]	@ zero_extendqisi2
	ldr	r1, [ip, #1624]
	cmp	r3, #1
	mla	r1, fp, r5, r1
	add	r4, r1, r4
	str	r4, [r8]
	str	r0, [r9]
	movls	r0, #0
	ldrhi	r0, [r6, #4]
	ldrhi	r3, [r6, #40]
	addhi	r0, r0, #1024
	rsbhi	r0, r3, r0
	clzhi	r0, r0
	movhi	r0, r0, lsr #5
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L144:
	.align	2
.L143:
	.word	.LANCHOR0+1860
	.word	.LANCHOR0
	.fnend
	.size	LogAddr2PhyAddr, .-LogAddr2PhyAddr
	.align	2
	.global	ftl_read_flash_info
	.type	ftl_read_flash_info, %function
ftl_read_flash_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r1, #11
	mov	r4, r0
	bl	__memzero
	ldr	r3, .L150
	mov	ip, #1
	add	r3, r3, #1616
	ldr	r2, [r3, #168]
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	ldr	r2, [r3, #-1612]
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
	mov	r1, #32
	strb	r1, [r4, #8]
	ldrb	r2, [r2, #7]	@ zero_extendqisi2
	strb	r2, [r4, #9]
	mov	r2, #0
	strb	r2, [r4, #10]
.L146:
	uxtb	r1, r2
	cmp	r1, r0
	bcs	.L149
	ldrb	lr, [r2, r3]	@ zero_extendqisi2
	add	r2, r2, #1
	ldrb	r1, [r4, #10]	@ zero_extendqisi2
	orr	r1, r1, ip, asl lr
	strb	r1, [r4, #10]
	b	.L146
.L149:
	ldmfd	sp!, {r4, pc}
.L151:
	.align	2
.L150:
	.word	.LANCHOR0
	.fnend
	.size	ftl_read_flash_info, .-ftl_read_flash_info
	.align	2
	.global	FlashScheduleEnSet
	.type	FlashScheduleEnSet, %function
FlashScheduleEnSet:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L153
	ldr	r2, [r3, #1896]
	str	r0, [r3, #1896]
	mov	r0, r2
	bx	lr
.L154:
	.align	2
.L153:
	.word	.LANCHOR0
	.fnend
	.size	FlashScheduleEnSet, .-FlashScheduleEnSet
	.align	2
	.global	FlashGetPageSize
	.type	FlashGetPageSize, %function
FlashGetPageSize:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L156
	ldr	r3, [r3, #1784]
	ldrb	r0, [r3, #9]	@ zero_extendqisi2
	bx	lr
.L157:
	.align	2
.L156:
	.word	.LANCHOR0
	.fnend
	.size	FlashGetPageSize, .-FlashGetPageSize
	.align	2
	.global	NandcReadDontCaseBusyEn
	.type	NandcReadDontCaseBusyEn, %function
NandcReadDontCaseBusyEn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	NandcReadDontCaseBusyEn, .-NandcReadDontCaseBusyEn
	.align	2
	.global	NandcGetChipIf
	.type	NandcGetChipIf, %function
NandcGetChipIf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L160
	add	r0, r3, r0, asl #3
	ldrb	r2, [r0, #16]	@ zero_extendqisi2
	ldr	r0, [r0, #12]
	add	r2, r2, #8
	add	r0, r0, r2, asl #8
	bx	lr
.L161:
	.align	2
.L160:
	.word	.LANCHOR0
	.fnend
	.size	NandcGetChipIf, .-NandcGetChipIf
	.align	2
	.global	NandcSetDdrPara
	.type	NandcSetDdrPara, %function
NandcSetDdrPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L163
	ldr	r2, [r3, #1824]
	mov	r3, r0, asl #8
	orr	r0, r3, r0, asl #16
	orr	r3, r0, #1
	str	r3, [r2, #304]
	bx	lr
.L164:
	.align	2
.L163:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetDdrPara, .-NandcSetDdrPara
	.align	2
	.global	NandcSetDdrDiv
	.type	NandcSetDdrDiv, %function
NandcSetDdrDiv:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L166
	orr	r0, r0, #16640
	ldr	r3, [r3, #1824]
	str	r0, [r3, #344]
	bx	lr
.L167:
	.align	2
.L166:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetDdrDiv, .-NandcSetDdrDiv
	.align	2
	.global	NandcSetDdrMode
	.type	NandcSetDdrMode, %function
NandcSetDdrMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L171
	cmp	r0, #0
	ldr	r2, [r3, #1824]
	ldr	r3, [r2]
	bfieq	r3, r0, #13, #1
	orrne	r3, r3, #253952
	str	r3, [r2]
	bx	lr
.L172:
	.align	2
.L171:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetDdrMode, .-NandcSetDdrMode
	.align	2
	.global	NandcSetMode
	.type	NandcSetMode, %function
NandcSetMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L180
	ands	r1, r0, #6
	ldr	r2, [r3, #1824]
	ldr	r3, [r2]
	bfieq	r3, r1, #13, #1
	beq	.L176
	orr	r3, r3, #24576
	movw	r1, #8322
	bfc	r3, #15, #1
	str	r1, [r2, #344]
	orr	r3, r3, #196608
	ldr	r1, .L180+4
	tst	r0, #4
	orrne	r3, r3, #32768
	str	r1, [r2, #304]
	mov	r1, #38
	str	r1, [r2, #308]
	mov	r1, #39
	str	r1, [r2, #308]
.L176:
	str	r3, [r2]
	mov	r0, #0
	bx	lr
.L181:
	.align	2
.L180:
	.word	.LANCHOR0
	.word	1052675
	.fnend
	.size	NandcSetMode, .-NandcSetMode
	.align	2
	.global	NandcFlashCs
	.type	NandcFlashCs, %function
NandcFlashCs:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L183
	add	r0, r2, r0, asl #3
	mov	r2, #1
	ldr	r1, [r0, #12]
	ldrb	r0, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r1]
	mov	r2, r2, asl r0
	bfi	r3, r2, #0, #8
	str	r3, [r1]
	bx	lr
.L184:
	.align	2
.L183:
	.word	.LANCHOR0
	.fnend
	.size	NandcFlashCs, .-NandcFlashCs
	.align	2
	.global	NandcFlashDeCs
	.type	NandcFlashDeCs, %function
NandcFlashDeCs:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L186
	add	r0, r3, r0, asl #3
	ldr	r2, [r0, #12]
	ldr	r3, [r2]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2]
	bx	lr
.L187:
	.align	2
.L186:
	.word	.LANCHOR0
	.fnend
	.size	NandcFlashDeCs, .-NandcFlashDeCs
	.align	2
	.global	NandcDelayns
	.type	NandcDelayns, %function
NandcDelayns:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	add	r0, r0, #996
	ldr	r3, .L190
	add	r0, r0, #3
	umull	r0, r1, r0, r3
	ldr	r3, .L190+4
	ldr	r3, [r3, #8]
	mov	r0, r1, lsr #6
	blx	r3
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L191:
	.align	2
.L190:
	.word	274877907
	.word	arm_delay_ops
	.fnend
	.size	NandcDelayns, .-NandcDelayns
	.align	2
	.global	FlashReadStatus
	.type	FlashReadStatus, %function
FlashReadStatus:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r2, #112
	ldr	r3, .L194
	add	r0, r3, r0, asl #3
	ldrb	r4, [r0, #16]	@ zero_extendqisi2
	ldr	r5, [r0, #12]
	mov	r0, #80
	add	r3, r5, r4, asl #8
	add	r4, r4, #8
	str	r2, [r3, #2056]
	bl	NandcDelayns
	ldr	r0, [r5, r4, asl #8]
	ldmfd	sp!, {r3, r4, r5, pc}
.L195:
	.align	2
.L194:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadStatus, .-FlashReadStatus
	.align	2
	.global	ToshibaSetRRPara
	.type	ToshibaSetRRPara, %function
ToshibaSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r8, r1, r1, asl #2
	ldr	r9, .L206
	mov	r5, r0
	ldr	r7, .L206+4
	mov	r6, r1
	add	r10, r9, #288
	mov	r4, #0
.L197:
	ldrb	r3, [r7, #1901]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L205
	mov	r3, #85
	str	r3, [r5, #8]
	ldrsb	r3, [r4, r10]
	mov	r0, #200
	str	r3, [r5, #4]
	bl	NandcDelayns
	ldrb	r3, [r7, #1900]	@ zero_extendqisi2
	cmp	r3, #34
	addeq	r3, r4, r8
	addeq	r3, r10, r3
	beq	.L204
	cmp	r3, #35
	addne	r3, r9, r6
	addne	r3, r3, #432
	ldrnesb	r3, [r3]
	bne	.L203
	ldr	r3, .L206+8
	add	r2, r4, r8
	add	r3, r3, r2
.L204:
	ldrsb	r3, [r3, #5]
.L203:
	str	r3, [r5]
	add	r4, r4, #1
	b	.L197
.L205:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L207:
	.align	2
.L206:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR1+336
	.fnend
	.size	ToshibaSetRRPara, .-ToshibaSetRRPara
	.align	2
	.global	SamsungSetRRPara
	.type	SamsungSetRRPara, %function
SamsungSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L213
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r1, r3, r1, asl #2
	ldr	r8, .L213+4
	mov	r4, #0
	add	r5, r1, #3
	mov	r6, r0
	mov	r7, r3
	mov	r9, #161
	mov	r10, r4
.L209:
	ldrb	r3, [r8, #1901]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L212
	str	r9, [r6, #8]
	mov	r0, #300
	str	r10, [r6]
	ldrsb	r3, [r7, r4]
	add	r4, r4, #1
	str	r3, [r6]
	ldrsb	r3, [r5, #1]!
	str	r3, [r6]
	bl	NandcDelayns
	b	.L209
.L212:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L214:
	.align	2
.L213:
	.word	.LANCHOR1+440
	.word	.LANCHOR0
	.fnend
	.size	SamsungSetRRPara, .-SamsungSetRRPara
	.align	2
	.global	HynixSetRRPara
	.type	HynixSetRRPara, %function
HynixSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r6, r3
	ldr	r5, .L224
	mov	r9, r2
	ldr	r4, .L224+4
	mov	r7, r0
	mov	r8, r1
	ldr	r3, [r5, #1784]
	ldrb	r2, [r3, #19]	@ zero_extendqisi2
	mov	r3, r0, asl #3
	cmp	r2, #6
	addeq	r4, r4, r0, asl #6
	addeq	r4, r4, #20
	addeq	r4, r4, r6, asl #2
	beq	.L217
	cmp	r2, #7
	bne	.L218
	mov	r2, #160
	mla	r4, r2, r0, r4
	add	r2, r6, r6, asl #2
	add	r4, r4, #28
	add	r4, r4, r2, asl #1
	b	.L217
.L218:
	cmp	r2, #8
	addne	r2, r6, r3
	addeq	r2, r6, r6, asl #2
	ldreq	r4, .L224+8
	addne	r4, r4, r2, asl #3
	addeq	r4, r4, r2
	addne	r4, r4, #20
.L217:
	add	r3, r5, r3
	mov	r0, r7
	sub	r8, r8, #1
	sub	r4, r4, #1
	ldrb	fp, [r3, #16]	@ zero_extendqisi2
	ldr	r10, [r3, #12]
	bl	NandcFlashCs
	sub	ip, r9, #1
	add	r9, r9, r8
	mov	r3, fp, asl #8
	mov	r2, #54
	add	fp, r10, r3
	str	r2, [fp, #2056]
.L220:
	cmp	ip, r9
	beq	.L223
	ldrb	r2, [ip, #1]!	@ zero_extendqisi2
	mov	r0, #200
	str	r3, [sp, #4]
	str	r2, [fp, #2052]
	str	ip, [sp]
	bl	NandcDelayns
	ldrsb	r2, [r4, #1]!
	str	r2, [fp, #2048]
	ldr	r3, [sp, #4]
	ldr	ip, [sp]
	b	.L220
.L223:
	add	r10, r10, r3
	mov	r0, r7
	mov	r3, #22
	add	r5, r5, r7
	str	r3, [r10, #2056]
	bl	NandcFlashDeCs
	strb	r6, [r5, #2756]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L225:
	.align	2
.L224:
	.word	.LANCHOR0
	.word	.LANCHOR0+1902
	.word	.LANCHOR0+1930
	.fnend
	.size	HynixSetRRPara, .-HynixSetRRPara
	.align	2
	.global	FlashSetReadRetryDefault
	.type	FlashSetReadRetryDefault, %function
FlashSetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L234
	ldr	r2, [r3, #1784]
	ldrb	r2, [r2, #19]	@ zero_extendqisi2
	sub	r2, r2, #1
	cmp	r2, #7
	bxhi	lr
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, #0
	ldr	r6, .L234+4
	mov	r5, r3
.L227:
	ldrb	r3, [r6, r4, asl #3]	@ zero_extendqisi2
	uxtb	r0, r4
	cmp	r3, #173
	bne	.L228
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	mov	r3, #0
	ldr	r2, .L234+8
	bl	HynixSetRRPara
.L228:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L227
	ldmfd	sp!, {r4, r5, r6, pc}
.L235:
	.align	2
.L234:
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.word	.LANCHOR0+1906
	.fnend
	.size	FlashSetReadRetryDefault, .-FlashSetReadRetryDefault
	.align	2
	.global	FlashReadStatusEN
	.type	FlashReadStatusEN, %function
FlashReadStatusEN:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L247
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	add	r0, ip, r0, asl #3
	ldr	r3, [ip, #1784]
	ldrb	r5, [r0, #16]	@ zero_extendqisi2
	ldr	r4, [r0, #12]
	ldrb	r3, [r3, #8]	@ zero_extendqisi2
	cmp	r3, #2
	mov	r3, r5, asl #8
	addne	r3, r4, r3
	add	r5, r5, #8
	movne	r2, #112
	strne	r2, [r3, #2056]
	bne	.L241
	cmp	r2, #0
	add	r3, r4, r3
	ldrneb	r2, [ip, #58]	@ zero_extendqisi2
	ldreqb	r2, [ip, #57]	@ zero_extendqisi2
	str	r2, [r3, #2056]
	ldrb	r0, [ip, #59]	@ zero_extendqisi2
	cmp	r0, #0
	addne	ip, r4, r5, asl #8
	movne	r2, #0
	beq	.L241
.L240:
	cmp	r2, r0
	bcs	.L241
	mov	r3, r2, asl #3
	add	r2, r2, #1
	mov	r3, r1, lsr r3
	uxtb	r3, r3
	str	r3, [ip, #4]
	b	.L240
.L241:
	mov	r0, #80
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	uxtb	r0, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L248:
	.align	2
.L247:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadStatusEN, .-FlashReadStatusEN
	.align	2
	.global	FlashWaitReadyEN
	.type	FlashWaitReadyEN, %function
FlashWaitReadyEN:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	mov	r5, r1
	mov	r6, r2
.L253:
	mov	r0, r4
	mov	r1, r5
	mov	r2, r6
	bl	FlashReadStatusEN
	cmp	r0, #255
	beq	.L253
	tst	r0, #64
	beq	.L253
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	FlashWaitReadyEN, .-FlashWaitReadyEN
	.align	2
	.global	FlashWaitCmdDone
	.type	FlashWaitCmdDone, %function
FlashWaitCmdDone:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldr	r5, .L265
	add	r4, r5, r0, asl #4
	ldr	r3, [r4, #1664]
	ldrb	r7, [r4, #1656]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L259
	mov	r6, r0
	mov	r0, r7
	add	r5, r5, r6, asl #2
	bl	NandcFlashCs
	ldr	r1, [r4, #1660]
	mov	r0, r7
	ldr	r2, [r5, #1624]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r5, r0
	mov	r0, r7
	bl	NandcFlashDeCs
	ldr	r2, [r4, #1664]
	sbfx	r3, r5, #0, #1
	str	r3, [r2]
	mov	r2, #0
	ldr	r1, [r4, #1668]
	str	r2, [r4, #1664]
	cmp	r1, r2
	strne	r3, [r1]
	strne	r2, [r4, #1668]
.L259:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L266:
	.align	2
.L265:
	.word	.LANCHOR0
	.fnend
	.size	FlashWaitCmdDone, .-FlashWaitCmdDone
	.align	2
	.type	flash_read_ecc, %function
flash_read_ecc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L269
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	r0, r2, r0, asl #3
	ldrb	r4, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r0, #12]
	mov	r0, #80
	add	r4, r3, r4, asl #8
	mov	r3, #122
	str	r3, [r4, #2056]
	bl	NandcDelayns
	ldr	r3, [r4, #2048]
	ldr	r0, [r4, #2048]
	and	r3, r3, #15
	and	r0, r0, #15
	cmp	r0, r3
	movcc	r0, r3
	ldr	r3, [r4, #2048]
	and	r3, r3, #15
	cmp	r0, r3
	movcc	r0, r3
	ldr	r3, [r4, #2048]
	and	r3, r3, #15
	cmp	r0, r3
	movcc	r0, r3
	ldmfd	sp!, {r4, pc}
.L270:
	.align	2
.L269:
	.word	.LANCHOR0
	.fnend
	.size	flash_read_ecc, .-flash_read_ecc
	.align	2
	.global	NandcWaitFlashReadyNoDelay
	.type	NandcWaitFlashReadyNoDelay, %function
NandcWaitFlashReadyNoDelay:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L277
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	add	r0, r3, r0, asl #3
	ldr	r4, .L277+4
	ldr	r5, [r0, #12]
.L273:
	ldr	r3, [r5]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L274
	mov	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L273
	mvn	r0, #0
	b	.L272
.L274:
	mov	r0, #0
.L272:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L278:
	.align	2
.L277:
	.word	.LANCHOR0
	.word	100000
	.fnend
	.size	NandcWaitFlashReadyNoDelay, .-NandcWaitFlashReadyNoDelay
	.align	2
	.global	NandcWaitFlashReady
	.type	NandcWaitFlashReady, %function
NandcWaitFlashReady:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L285
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	add	r0, r3, r0, asl #3
	ldr	r4, .L285+4
	ldr	r5, [r0, #12]
	mov	r0, #150
	bl	NandcDelayns
.L281:
	ldr	r3, [r5]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L282
	mov	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L281
	mvn	r0, #0
	b	.L280
.L282:
	mov	r0, #0
.L280:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L286:
	.align	2
.L285:
	.word	.LANCHOR0
	.word	100000
	.fnend
	.size	NandcWaitFlashReady, .-NandcWaitFlashReady
	.align	2
	.global	FlashReset
	.type	FlashReset, %function
FlashReset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L289
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r3, r3, r0, asl #3
	mov	r4, r0
	ldrb	r6, [r3, #16]	@ zero_extendqisi2
	ldr	r5, [r3, #12]
	bl	NandcFlashCs
	mov	r3, #255
	mov	r0, r4
	add	r5, r5, r6, asl #8
	str	r3, [r5, #2056]
	bl	NandcWaitFlashReady
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, lr}
	b	NandcFlashDeCs
.L290:
	.align	2
.L289:
	.word	.LANCHOR0
	.fnend
	.size	FlashReset, .-FlashReset
	.align	2
	.global	flash_enter_slc_mode
	.type	flash_enter_slc_mode, %function
flash_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L294
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	mov	r5, r0
	bl	NandcFlashCs
	add	r3, r4, r5, asl #3
	mov	r0, #100
	ldr	r4, [r3, #12]
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	add	r4, r4, r3, asl #8
	mov	r3, #239
	str	r3, [r4, #2056]
	mov	r3, #145
	str	r3, [r4, #2052]
	bl	NandcDelayns
	mov	r3, #0
	mov	r2, #1
	str	r3, [r4, #2048]
	mov	r0, #100
	str	r2, [r4, #2048]
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	bl	NandcDelayns
	mov	r0, r5
	bl	NandcWaitFlashReadyNoDelay
	mov	r3, #218
	mov	r0, r5
	str	r3, [r4, #2056]
	ldmfd	sp!, {r3, r4, r5, lr}
	b	NandcWaitFlashReady
.L295:
	.align	2
.L294:
	.word	.LANCHOR0
	.fnend
	.size	flash_enter_slc_mode, .-flash_enter_slc_mode
	.align	2
	.global	flash_exit_slc_mode
	.type	flash_exit_slc_mode, %function
flash_exit_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L299
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	mov	r5, r0
	bl	NandcFlashCs
	add	r3, r4, r5, asl #3
	mov	r0, #100
	ldr	r4, [r3, #12]
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	add	r4, r4, r3, asl #8
	mov	r3, #239
	str	r3, [r4, #2056]
	mov	r3, #145
	str	r3, [r4, #2052]
	bl	NandcDelayns
	mov	r3, #2
	mov	r0, #100
	str	r3, [r4, #2048]
	mov	r3, #1
	str	r3, [r4, #2048]
	mov	r3, #0
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	bl	NandcDelayns
	mov	r0, r5
	bl	NandcWaitFlashReadyNoDelay
	mov	r3, #223
	mov	r0, r5
	str	r3, [r4, #2056]
	ldmfd	sp!, {r3, r4, r5, lr}
	b	NandcWaitFlashReady
.L300:
	.align	2
.L299:
	.word	.LANCHOR0
	.fnend
	.size	flash_exit_slc_mode, .-flash_exit_slc_mode
	.align	2
	.global	FlashEraseBlock
	.type	FlashEraseBlock, %function
FlashEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
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
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	FlashEraseBlock, .-FlashEraseBlock
	.align	2
	.global	FlashSetInterfaceMode
	.type	FlashSetInterfaceMode, %function
FlashSetInterfaceMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	lr, #0
	ldr	r4, .L326
	mov	r5, #239
	mov	r6, #128
	mov	r7, #1
	add	r4, r4, #12
	mov	r8, #35
	ldrb	r3, [r4, #2748]	@ zero_extendqisi2
	mov	r9, #32
	mov	r10, #5
	and	r2, r3, #1
	str	r2, [sp, #4]
	and	r2, r3, #4
	uxtb	r3, r2
	mov	r2, lr
	str	r3, [sp]
.L313:
	ldr	ip, .L326+4
	mov	r3, r4
	ldr	r1, [r3, lr]!
	ldrb	ip, [lr, ip]	@ zero_extendqisi2
	ldrb	r3, [r3, #4]	@ zero_extendqisi2
	cmp	ip, #69
	cmpne	ip, #152
	beq	.L304
	cmp	ip, #44
	cmpne	ip, #173
	bne	.L305
.L304:
	cmp	r0, #1
	bne	.L306
	ldr	fp, [sp, #4]
	cmp	fp, #0
	beq	.L305
	mov	r3, r3, asl #8
	cmp	ip, #173
	add	fp, r1, r3
	str	r5, [fp, #2056]
	streq	r0, [fp, #2052]
	beq	.L325
	cmp	ip, #44
	streq	r0, [fp, #2052]
	strne	r6, [fp, #2052]
	streq	r10, [fp, #2048]
	strne	r0, [fp, #2048]
	b	.L311
.L306:
	ldr	fp, [sp]
	cmp	fp, #0
	beq	.L305
	mov	r3, r3, asl #8
	cmp	ip, #173
	add	fp, r1, r3
	str	r5, [fp, #2056]
	streq	r7, [fp, #2052]
	streq	r9, [fp, #2048]
	beq	.L311
	cmp	ip, #44
	streq	r7, [fp, #2052]
	streq	r8, [fp, #2048]
	beq	.L311
	str	r6, [fp, #2052]
.L325:
	str	r2, [fp, #2048]
.L311:
	add	r3, r1, r3
	str	r2, [r3, #2048]
	str	r2, [r3, #2048]
	str	r2, [r3, #2048]
.L305:
	add	lr, lr, #8
	cmp	lr, #32
	bne	.L313
	mov	r0, #0
	bl	NandcWaitFlashReady
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L327:
	.align	2
.L326:
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.fnend
	.size	FlashSetInterfaceMode, .-FlashSetInterfaceMode
	.align	2
	.global	FlashReadSpare
	.type	FlashReadSpare, %function
FlashReadSpare:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L330
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	add	ip, ip, r0, asl #3
	ldr	r3, .L330+4
	mov	r5, r2
	ldrb	r2, [ip, #16]	@ zero_extendqisi2
	ldr	r4, [ip, #12]
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	add	r4, r4, r2, asl #8
	mov	r2, #0
	mov	r3, r3, asl #9
	str	r2, [r4, #2056]
	str	r3, [r4, #2052]
	mov	r3, r3, lsr #8
	str	r3, [r4, #2052]
	uxtb	r3, r1
	str	r3, [r4, #2052]
	mov	r3, r1, lsr #8
	mov	r1, r1, lsr #16
	str	r3, [r4, #2052]
	mov	r3, #48
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
	bl	NandcWaitFlashReady
	ldr	r3, [r4, #2048]
	strb	r3, [r5]
	ldmfd	sp!, {r3, r4, r5, pc}
.L331:
	.align	2
.L330:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.fnend
	.size	FlashReadSpare, .-FlashReadSpare
	.align	2
	.global	SandiskProgTestBadBlock
	.type	SandiskProgTestBadBlock, %function
SandiskProgTestBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L334
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	r2, r2, r0, asl #3
	ldrb	r4, [r2, #16]	@ zero_extendqisi2
	ldr	r3, [r2, #12]
	add	r4, r3, r4, asl #8
	mov	r3, #162
	str	r3, [r4, #2056]
	mov	r3, #128
	str	r3, [r4, #2056]
	mov	r3, #0
	str	r3, [r4, #2052]
	str	r3, [r4, #2052]
	uxtb	r3, r1
	str	r3, [r4, #2052]
	mov	r3, r1, lsr #8
	mov	r1, r1, lsr #16
	str	r3, [r4, #2052]
	str	r1, [r4, #2052]
	mov	r3, #16
	str	r3, [r4, #2056]
	bl	NandcWaitFlashReady
	mov	r3, #112
	mov	r0, #80
	str	r3, [r4, #2056]
	bl	NandcDelayns
	ldr	r0, [r4, #2048]
	and	r0, r0, #1
	ldmfd	sp!, {r4, pc}
.L335:
	.align	2
.L334:
	.word	.LANCHOR0
	.fnend
	.size	SandiskProgTestBadBlock, .-SandiskProgTestBadBlock
	.align	2
	.global	SandiskSetRRPara
	.type	SandiskSetRRPara, %function
SandiskSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r3, #239
	mov	r5, r0
	str	r3, [r0, #8]
	mov	r3, #17
	str	r3, [r0, #4]
	mov	r0, #200
	mov	r4, r1
	bl	NandcDelayns
	ldr	r0, .L344
	ldr	r1, .L344+4
	add	r4, r4, r4, asl #2
	sub	ip, r0, #48
	mov	r2, #0
.L337:
	ldrb	r3, [r1, #1901]	@ zero_extendqisi2
	cmp	r2, r3
	bcs	.L343
	ldrb	r3, [r1, #1900]	@ zero_extendqisi2
	cmp	r3, #67
	add	r3, r2, r4
	addeq	r3, ip, r3
	addne	r3, r0, r3
	add	r2, r2, #1
	ldrsb	r3, [r3, #5]
	str	r3, [r5]
	b	.L337
.L343:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, lr}
	b	NandcWaitFlashReady
.L345:
	.align	2
.L344:
	.word	.LANCHOR1+336
	.word	.LANCHOR0
	.fnend
	.size	SandiskSetRRPara, .-SandiskSetRRPara
	.align	2
	.global	micron_auto_read_calibration_config
	.type	micron_auto_read_calibration_config, %function
micron_auto_read_calibration_config:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	mov	r5, r1
	bl	NandcWaitFlashReady
	ldr	r3, .L348
	mov	r0, #200
	add	r2, r3, r4, asl #3
	ldrb	r4, [r2, #16]	@ zero_extendqisi2
	ldr	r3, [r2, #12]
	add	r4, r3, r4, asl #8
	mov	r3, #239
	str	r3, [r4, #2056]
	mov	r3, #150
	str	r3, [r4, #2052]
	bl	NandcDelayns
	str	r5, [r4, #2048]
	mov	r3, #0
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	ldmfd	sp!, {r3, r4, r5, pc}
.L349:
	.align	2
.L348:
	.word	.LANCHOR0
	.fnend
	.size	micron_auto_read_calibration_config, .-micron_auto_read_calibration_config
	.align	2
	.global	FlashEraseSLc2KBlocks
	.type	FlashEraseSLc2KBlocks, %function
FlashEraseSLc2KBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r5, #0
	ldr	r8, .L362
	.pad #20
	sub	sp, sp, #20
	mov	r6, r0
	mov	r9, r1
	mov	r7, r5
.L351:
	cmp	r7, r9
	beq	.L361
	rsb	r3, r7, r9
	add	r2, sp, #8
	add	r0, r6, r5
	mov	r1, #0
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #12
	bl	LogAddr2PhyAddr
	ldrb	r2, [r8, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #12]
	cmp	r3, r2
	mvncs	r3, #0
	strcs	r3, [r6, r5]
	bcs	.L353
	add	r2, r8, r3
	add	r3, r8, r3, asl #4
	ldrb	r4, [r2, #1616]	@ zero_extendqisi2
	strb	r4, [r3, #1656]
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	mov	r2, #0
	mov	r0, r4
	ldr	r1, [sp, #8]
	bl	FlashEraseCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	ldr	r1, [sp, #8]
	bl	FlashReadStatus
	mov	r2, #0
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
	tst	r0, #1
	mvnne	r3, #0
	strne	r3, [r6, r5]
	ldr	r3, [r6, r5]
	cmn	r3, #1
	bne	.L355
	ldr	r0, .L362+4
	ldr	r1, [sp, #8]
	bl	printk
.L355:
	mov	r0, r4
	bl	NandcFlashDeCs
.L353:
	add	r7, r7, #1
	add	r5, r5, #36
	b	.L351
.L361:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L363:
	.align	2
.L362:
	.word	.LANCHOR0
	.word	.LC1
	.fnend
	.size	FlashEraseSLc2KBlocks, .-FlashEraseSLc2KBlocks
	.align	2
	.global	FlashReadDpCmd
	.type	FlashReadDpCmd, %function
FlashReadDpCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r1
	ldr	r3, .L369
	mov	r7, r0
	uxtb	lr, r6
	mov	ip, r6, lsr #8
	add	r1, r3, r0, asl #3
	uxtb	r9, r2
	ldrb	r10, [r3, #52]	@ zero_extendqisi2
	mov	r8, r2, lsr #8
	ldrb	r4, [r1, #16]	@ zero_extendqisi2
	mov	r5, r2, lsr #16
	ldr	fp, [r1, #12]
	ldrb	r1, [r3, #60]	@ zero_extendqisi2
	mov	r4, r4, asl #8
	ldrb	r3, [r3, #53]	@ zero_extendqisi2
	cmp	r1, #1
	add	r4, fp, r4
	mov	r1, r6, lsr #16
	str	r10, [r4, #2056]
	bne	.L365
	mov	r10, #0
	str	r10, [r4, #2052]
	str	r10, [r4, #2052]
	str	lr, [r4, #2052]
	str	ip, [r4, #2052]
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
	bl	NandcWaitFlashReady
	str	r10, [r4, #2056]
	str	r10, [r4, #2052]
	str	r10, [r4, #2052]
	b	.L368
.L365:
	str	lr, [r4, #2052]
	str	ip, [r4, #2052]
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
.L368:
	str	r9, [r4, #2052]
	mov	r3, #48
	str	r8, [r4, #2052]
	mov	r0, r7
	str	r5, [r4, #2052]
	mov	r1, r6
	str	r3, [r4, #2056]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	FlashSetRandomizer
.L370:
	.align	2
.L369:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadDpCmd, .-FlashReadDpCmd
	.align	2
	.global	ftl_flash_de_init
	.type	ftl_flash_de_init, %function
ftl_flash_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r0, #0
	ldr	r4, .L382
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	ldr	r0, [r4, #2764]
	cmp	r0, #0
	beq	.L372
	mov	r0, #0
	bl	flash_enter_slc_mode
	b	.L373
.L372:
	bl	flash_exit_slc_mode
.L373:
	ldrb	r3, [r4, #2768]	@ zero_extendqisi2
	ldr	r5, .L382
	cmp	r3, #0
	beq	.L374
	ldrb	r3, [r5, #2760]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L374
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r3, #0
	strb	r3, [r5, #2768]
.L374:
	ldr	r3, [r4, #12]
	mov	r0, #0
	str	r0, [r3, #336]
	ldmfd	sp!, {r3, r4, r5, pc}
.L383:
	.align	2
.L382:
	.word	.LANCHOR0
	.fnend
	.size	ftl_flash_de_init, .-ftl_flash_de_init
	.align	2
	.global	NandcRandmzSel
	.type	NandcRandmzSel, %function
NandcRandmzSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L385
	add	r0, r3, r0, asl #3
	ldr	r3, [r0, #12]
	str	r1, [r3, #336]
	bx	lr
.L386:
	.align	2
.L385:
	.word	.LANCHOR0
	.fnend
	.size	NandcRandmzSel, .-NandcRandmzSel
	.global	__aeabi_idiv
	.align	2
	.global	NandcTimeCfg
	.type	NandcTimeCfg, %function
NandcTimeCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	mov	r0, #0
	bl	rknand_get_clk_rate
	ldr	r1, .L398
	bl	__aeabi_idiv
	ldr	r3, .L398+4
	ldr	r3, [r3, #1824]
	cmp	r0, #250
	movwgt	r2, #8354
	bgt	.L396
	cmp	r0, #220
	bgt	.L397
	cmp	r0, #185
	movwgt	r2, #4226
	bgt	.L396
	cmp	r0, #160
	movwgt	r2, #4194
	bgt	.L396
	cmp	r4, #35
	movwls	r2, #4193
	bls	.L396
	cmp	r4, #99
	movwls	r2, #4225
	bls	.L396
.L397:
	movw	r2, #8322
.L396:
	str	r2, [r3, #4]
	ldmfd	sp!, {r4, pc}
.L399:
	.align	2
.L398:
	.word	1000000
	.word	.LANCHOR0
	.fnend
	.size	NandcTimeCfg, .-NandcTimeCfg
	.align	2
	.global	FlashTimingCfg
	.type	FlashTimingCfg, %function
FlashTimingCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	sub	r3, r0, #4160
	sub	r3, r3, #33
	bic	r3, r3, #32
	cmp	r3, #1
	bls	.L401
	movw	r3, #8322
	cmp	r0, r3
	bne	.L402
.L401:
	ldr	r3, .L403
	ldr	r3, [r3, #1824]
	str	r0, [r3, #4]
.L402:
	ldr	r3, .L403+4
	ldrb	r0, [r3, #277]	@ zero_extendqisi2
	b	NandcTimeCfg
.L404:
	.align	2
.L403:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.fnend
	.size	FlashTimingCfg, .-FlashTimingCfg
	.align	2
	.global	NandcInit
	.type	NandcInit, %function
NandcInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r1, #0
	ldr	r3, .L408
	mov	r2, #1
	mov	r5, #0
	str	r1, [r3, #16]
	mov	r4, r3
	str	r0, [r3, #12]
	str	r2, [r3, #24]
	mov	r2, #2
	str	r0, [r3, #20]
	str	r2, [r3, #32]
	mov	r2, #3
	str	r0, [r3, #28]
	str	r0, [r3, #36]
	str	r0, [r3, #1824]
	str	r2, [r3, #40]
	ldr	r2, [r0]
	and	r2, r2, #253952
	ubfx	ip, r2, #13, #1
	bfi	r2, r1, #13, #1
	ldr	r1, [r0, #352]
	orr	r2, r2, #256
	str	ip, [r3, #2772]
	movw	ip, #2049
	ubfx	r1, r1, #16, #4
	str	r1, [r3, #2776]
	ldr	r1, [r0, #352]
	cmp	r1, ip
	str	r1, [r3, #2780]
	moveq	r3, #8
	streq	r3, [r4, #2776]
	str	r2, [r0]
	mov	r0, #40
	ldr	r3, [r4, #1824]
	str	r5, [r3, #336]
	bl	NandcTimeCfg
	ldr	r3, [r4, #1824]
	movw	r2, #8322
	mov	r0, #36864
	str	r2, [r3, #344]
	ldr	r2, .L408+4
	str	r2, [r3, #304]
	bl	ftl_malloc
	str	r5, [r4, #2812]
	str	r5, [r4, #2820]
	str	r0, [r4, #2784]
	str	r0, [r4, #2788]
	add	r0, r0, #32768
	str	r0, [r4, #2792]
	ldmfd	sp!, {r3, r4, r5, pc}
.L409:
	.align	2
.L408:
	.word	.LANCHOR0
	.word	1579009
	.fnend
	.size	NandcInit, .-NandcInit
	.align	2
	.global	NandcGetTimeCfg
	.type	NandcGetTimeCfg, %function
NandcGetTimeCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L412
	str	lr, [sp, #-4]!
	.save {lr}
	ldr	lr, [ip, #1824]
	ldr	lr, [lr, #4]
	str	lr, [r0]
	ldr	r0, [ip, #1824]
	ldr	r0, [r0]
	str	r0, [r1]
	ldr	r1, [ip, #1824]
	ldr	r1, [r1, #304]
	str	r1, [r2]
	ldr	r1, [ip, #1824]
	ldr	r2, [r1, #308]
	ldr	r1, [r1, #344]
	uxtb	r2, r2
	orr	r2, r2, r1, asl #16
	str	r2, [r3]
	ldr	pc, [sp], #4
.L413:
	.align	2
.L412:
	.word	.LANCHOR0
	.fnend
	.size	NandcGetTimeCfg, .-NandcGetTimeCfg
	.align	2
	.global	NandcBchSel
	.type	NandcBchSel, %function
NandcBchSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L422
	mov	r1, #1
	ldr	r2, [r3, #1824]
	str	r0, [r3, #2824]
	mov	r3, #0
	str	r1, [r2, #8]
	mov	r1, #16
	cmp	r0, r1
	bfi	r3, r1, #8, #8
	bfc	r3, #18, #1
	bne	.L415
.L418:
	bfc	r3, #4, #1
	b	.L416
.L415:
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L416
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	beq	.L418
.L416:
	orr	r3, r3, #1
	str	r3, [r2, #12]
	bx	lr
.L423:
	.align	2
.L422:
	.word	.LANCHOR0
	.fnend
	.size	NandcBchSel, .-NandcBchSel
	.align	2
	.global	FlashBchSel
	.type	FlashBchSel, %function
FlashBchSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L425
	strb	r0, [r3, #1889]
	b	NandcBchSel
.L426:
	.align	2
.L425:
	.word	.LANCHOR0
	.fnend
	.size	FlashBchSel, .-FlashBchSel
	.align	2
	.global	ftl_flash_resume
	.type	ftl_flash_resume, %function
ftl_flash_resume:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L436
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, #0
	ldr	r2, [r3, #1824]
	mov	r4, r3
	ldr	r1, [r3, #1828]
	ldr	r6, .L436+4
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
.L429:
	ldrb	r3, [r6, r5, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L428
	uxtb	r0, r5
	bl	FlashReset
.L428:
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L429
	ldrb	r3, [r4, #2768]	@ zero_extendqisi2
	ldr	r5, .L436
	cmp	r3, #0
	beq	.L430
	mov	r0, #1
	bl	NandcSetMode
	ldrb	r0, [r5, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r5, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
	ldrb	r0, [r5, #1845]	@ zero_extendqisi2
	bl	NandcSetDdrPara
.L430:
	ldr	r3, [r4, #1784]
	ldmfd	sp!, {r4, r5, r6, lr}
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	b	FlashBchSel
.L437:
	.align	2
.L436:
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.fnend
	.size	ftl_flash_resume, .-ftl_flash_resume
	.align	2
	.global	ftl_nandc_get_irq_status
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
	.align	2
	.global	rk_nandc_flash_ready
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
	.align	2
	.global	NandcIqrWaitFlashReady
	.type	NandcIqrWaitFlashReady, %function
NandcIqrWaitFlashReady:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
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
	tst	r3, #512
	bne	.L441
	mov	r0, r4
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L441:
	ldr	r3, [r4, #368]
	orr	r3, r3, #2
	str	r3, [r4, #368]
	ldr	r3, [r4, #364]
	bic	r3, r3, #2
	str	r3, [r4, #364]
	ldmfd	sp!, {r4, pc}
	.fnend
	.size	NandcIqrWaitFlashReady, .-NandcIqrWaitFlashReady
	.align	2
	.global	FlashEraseBlocks
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r2
	ldr	r4, .L479
	.pad #28
	sub	sp, sp, #28
	ldrb	r5, [r4]	@ zero_extendqisi2
	cmp	r5, #0
	moveq	r9, r0
	moveq	r10, r1
	moveq	fp, r4
	beq	.L444
	mov	r1, r2
	bl	FlashEraseSLc2KBlocks
	b	.L445
.L453:
	mov	r3, #36
	mov	r1, #0
	mul	r6, r3, r5
	add	r2, sp, #16
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
	mvncs	r3, #0
	strcs	r3, [r9, r6]
	bcc	.L476
.L447:
	add	r5, r5, #1
.L444:
	cmp	r5, r8
	bcc	.L453
	b	.L477
.L476:
	ldrb	r3, [fp, #2828]	@ zero_extendqisi2
	cmp	r3, #0
	add	r3, r4, r0, asl #4
	ldr	r3, [r3, #1664]
	moveq	r7, #0
	cmp	r3, #0
	beq	.L449
	uxtb	r0, r0
	bl	FlashWaitCmdDone
.L449:
	ldr	r2, [sp, #20]
	cmp	r7, #0
	ldr	r0, [sp, #12]
	addne	ip, r6, #36
	addne	ip, r9, ip
	mov	r3, r2, asl #4
	add	r2, r4, r2
	add	r1, r4, r3
	add	r3, r4, r3
	ldrb	r6, [r2, #1616]	@ zero_extendqisi2
	str	r0, [r1, #1664]
	mov	r0, #0
	str	r0, [r1, #1668]
	ldr	r0, [sp, #16]
	strne	ip, [r1, #1668]
	strb	r6, [r3, #1656]
	str	r0, [r1, #1660]
	mov	r0, r6
	bl	NandcFlashCs
	cmp	r10, #1
	mov	r0, r6
	bne	.L451
	ldrb	r3, [fp, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L451
	bl	flash_enter_slc_mode
	b	.L452
.L451:
	bl	flash_exit_slc_mode
.L452:
	ldr	r3, [sp, #20]
	mov	r0, r6
	ldr	r1, [sp, #16]
	add	r5, r5, r7
	add	r3, r4, r3, asl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r6
	mov	r2, r7
	ldr	r1, [sp, #16]
	bl	FlashEraseCmd
	mov	r0, r6
	bl	NandcFlashDeCs
	b	.L447
.L477:
	ldr	r0, [r4, #1824]
	mov	r5, #0
	bl	NandcIqrWaitFlashReady
	ldr	r6, .L479
	ldr	r7, .L479+4
.L454:
	ldrb	r2, [r4, #1612]	@ zero_extendqisi2
	ldr	r3, .L479
	cmp	r5, r2
	bcs	.L478
	uxtb	r0, r5
	bl	FlashWaitCmdDone
	cmp	r10, #1
	bne	.L455
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L455
	ldrb	r0, [r7, r5, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L455:
	add	r5, r5, #1
	b	.L454
.L478:
	ldr	r2, [r3, #2832]
	cmp	r2, #0
	bne	.L457
.L459:
	mov	r0, #0
	b	.L445
.L457:
	ldrb	r3, [r3, #1788]	@ zero_extendqisi2
	cmp	r3, #69
	bne	.L459
	mov	r3, #0
	mov	r2, #36
	mov	r1, r3
.L458:
	cmp	r3, r8
	beq	.L459
	mul	r0, r2, r3
	add	r3, r3, #1
	str	r1, [r9, r0]
	b	.L458
.L445:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L480:
	.align	2
.L479:
	.word	.LANCHOR0
	.word	.LANCHOR0+1656
	.fnend
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	2
	.global	rk_nandc_flash_xfer_completed
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
	.align	2
	.global	NandcSendDumpDataStart
	.type	NandcSendDumpDataStart, %function
NandcSendDumpDataStart:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, [r0, #16]
	.pad #8
	sub	sp, sp, #8
	ldr	r3, .L484
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
.L485:
	.align	2
.L484:
	.word	538969130
	.fnend
	.size	NandcSendDumpDataStart, .-NandcSendDumpDataStart
	.align	2
	.global	NandcSendDumpDataDone
	.type	NandcSendDumpDataDone, %function
NandcSendDumpDataDone:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #8
	sub	sp, sp, #8
.L487:
	ldr	r3, [r0, #8]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #1048576
	beq	.L487
	add	sp, sp, #8
	@ sp needed
	bx	lr
	.fnend
	.size	NandcSendDumpDataDone, .-NandcSendDumpDataDone
	.align	2
	.global	NandcXferStart
	.type	NandcXferStart, %function
NandcXferStart:
	.fnstart
	@ args = 8, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	lr, #16
	ldr	r5, .L509
	mov	r4, #0
	.pad #28
	sub	sp, sp, #28
	add	r0, r5, r0, asl #3
	ldr	r8, [sp, #64]
	ldr	r6, [r0, #12]
	ldrb	r0, [r0, #16]	@ zero_extendqisi2
	ldr	ip, [sp, #68]
	ldr	r7, [r6, #12]
	bfi	r7, lr, #8, #8
	bfi	r7, r4, #3, #1
	bfi	r4, r1, #1, #1
	orr	r4, r4, #8
	bfi	r7, r0, #5, #3
	mov	r0, #1
	bfi	r4, r0, #5, #2
	orr	r4, r4, #536870912
	mov	r3, r3, lsr r0
	orr	r4, r4, #1024
	bfi	r4, r3, #4, #1
	ldr	r3, [r5, #2776]
	cmp	r3, #3
	bls	.L492
	ldr	r3, [r6, #16]
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	bic	r3, r3, #4
	str	r3, [sp, #20]
	adds	r3, ip, #0
	movne	r3, #1
	cmp	ip, #0
	cmpeq	r8, #0
	str	r3, [sp, #8]
	beq	.L493
	cmp	r1, #0
	bne	.L494
.L502:
	mov	r0, r6
	mov	r10, r2
	mov	r9, r1
	bl	rk_nandc_xfer_irq_flag_init
	ldr	r3, [r6, #368]
	add	r10, r10, #1
	cmp	r8, #0
	orr	r3, r3, #1
	str	r3, [r6, #368]
	ldr	r3, [r6, #364]
	mov	r10, r10, asr #1
	bfi	r4, r10, #22, #6
	movne	r0, r8
	orr	r3, r3, #1
	str	r3, [r6, #364]
	ldreq	r0, [r5, #2788]
	b	.L496
.L494:
	ldr	r3, [r5, #2824]
	mov	r9, r5
	cmp	r3, #25
	movcc	r3, #64
	movcs	r3, #128
	str	r3, [sp, #4]
	mov	r3, r2, lsr #1
	str	r3, [sp, #12]
	mov	r3, #0
	mov	r0, r3
.L498:
	ldr	lr, [sp, #12]
	cmp	r0, lr
	bcs	.L502
	ldr	lr, [sp, #8]
	mov	r10, r3, lsr #2
	add	r0, r0, #1
	cmp	lr, #0
	ldrneh	fp, [ip, #2]
	mvneq	fp, #0
	ldrneh	lr, [ip], #4
	ldreq	lr, [r9, #2792]
	orrne	lr, lr, fp, asl #16
	ldrne	fp, [r9, #2792]
	streq	fp, [lr, r10, asl #2]
	strne	lr, [fp, r10, asl #2]
	ldr	lr, [sp, #4]
	add	r3, r3, lr
	b	.L498
.L496:
	ldr	r3, [r5, #2792]
	ubfx	r10, r4, #22, #5
	mov	r2, r9
	str	r0, [r5, #2796]
	mov	r1, r10, asl #10
	str	r3, [r5, #2800]
	bl	rknand_dma_map_single
	mov	r2, r9
	mov	r1, r10, asl #7
	clz	r9, r9
	mov	r9, r9, lsr #5
	str	r0, [r5, #2804]
	ldr	r0, [r5, #2800]
	bl	rknand_dma_map_single
	mov	r3, #1
	str	r3, [r5, #2812]
	tst	r8, #3
	ldr	r3, [r5, #2804]
	str	r0, [r5, #2808]
	str	r3, [r6, #20]
	ldr	r3, [r5, #2808]
	str	r3, [r6, #24]
	mov	r3, #0
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	bic	r3, r3, #15872
	orr	r3, r3, #8192
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	orr	r3, r3, #448
	str	r3, [sp, #20]
	ldreq	r3, [sp, #20]
	biceq	r3, r3, #56
	orreq	r3, r3, #16
	streq	r3, [sp, #20]
	ldr	r3, [sp, #20]
	orr	r3, r3, #4
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	bic	r3, r3, #2
	orr	r9, r3, r9, asl #1
	str	r9, [sp, #20]
	ldr	r3, [sp, #20]
	orr	r3, r3, #1
	str	r3, [sp, #20]
.L493:
	ldr	r3, [sp, #20]
	str	r3, [r6, #16]
.L492:
	str	r7, [r6, #12]
	str	r4, [r6, #8]
	orr	r4, r4, #4
	str	r4, [r6, #8]
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L510:
	.align	2
.L509:
	.word	.LANCHOR0
	.fnend
	.size	NandcXferStart, .-NandcXferStart
	.align	2
	.global	Ftl_log2
	.type	Ftl_log2, %function
Ftl_log2:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r1, #0
	mov	r2, #1
.L512:
	cmp	r2, r0
	uxth	r3, r1
	add	r1, r1, #1
	movls	r2, r2, asl #1
	bls	.L512
.L514:
	sub	r0, r3, #1
	uxth	r0, r0
	bx	lr
	.fnend
	.size	Ftl_log2, .-Ftl_log2
	.align	2
	.global	FtlPrintInfo
	.type	FtlPrintInfo, %function
FtlPrintInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FtlPrintInfo, .-FtlPrintInfo
	.align	2
	.global	FtlSysBlkNumInit
	.type	FtlSysBlkNumInit, %function
FtlSysBlkNumInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L518
	movw	r2, #2840
	cmp	r0, #23
	movw	r1, #2850
	add	ip, r3, #2848
	ldrh	r2, [r3, r2]
	movls	r0, #24
	ldrh	r1, [r3, r1]
	str	r0, [r3, #2836]
	mul	r2, r2, r0
	rsb	r0, r0, r1
	ldr	r1, [r3, #2856]
	strh	r0, [ip]	@ movhi
	mov	r0, #0
	str	r2, [r3, #2844]
	rsb	r2, r2, r1
	str	r2, [r3, #2852]
	bx	lr
.L519:
	.align	2
.L518:
	.word	.LANCHOR0
	.fnend
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.align	2
	.global	FtlConstantsInit
	.type	FtlConstantsInit, %function
FtlConstantsInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #2860
	ldr	r4, .L548
	mov	r5, r0
	ldrh	r6, [r0, #8]
	.pad #20
	sub	sp, sp, #20
	ldrh	r2, [r0, #10]
	add	r1, r4, #2864
	ldrh	lr, [r0, #14]
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #2862
	strh	r2, [r4, r3]	@ movhi
	ldrh	r3, [r0, #12]
	ldr	r0, .L548+4
	strh	r3, [r1]	@ movhi
	movw	r1, #2850
	strh	lr, [r4, r1]	@ movhi
	mov	r1, #0
.L521:
	strb	r1, [r1, r0]
	add	r1, r1, #1
	cmp	r1, #32
	bne	.L521
	ldrh	r0, [r5, #14]
	ldrh	r1, [r5, #20]
	cmp	r1, r0, lsr #8
	bcs	.L522
	uxtb	r10, r3
	ldr	r9, .L548+4
	mov	r1, r10, asl #1
	uxtb	r1, r1
	str	r1, [sp, #4]
	sub	r1, r2, #1
	mul	r1, r3, r1
	str	r1, [sp]
	mov	r1, #0
.L523:
	cmp	r1, r3
	bcs	.L525
	ldr	ip, [sp]
	uxtb	r0, r1
	rsb	r7, r3, r1
	add	ip, r1, ip
	add	r7, r9, r7
	add	ip, r9, ip
	str	ip, [sp, #8]
	mov	ip, #0
	str	r7, [sp, #12]
	mov	r8, ip
.L526:
	cmp	r8, r2
	add	ip, ip, r3
	bcs	.L547
	ldr	r7, [sp, #12]
	add	fp, r0, r10
	add	r8, r8, #1
	strb	r0, [r7, ip]
	ldr	r7, [sp, #8]
	strb	fp, [r7, ip]
	ldr	r7, [sp, #4]
	add	r0, r0, r7
	uxtb	r0, r0
	b	.L526
.L547:
	add	r1, r1, #1
	b	.L523
.L525:
	movw	r1, #2862
	mov	r2, r2, asl #1
	strh	r2, [r4, r1]	@ movhi
	movw	r2, #2850
	mov	lr, lr, lsr #1
	strh	lr, [r4, r2]	@ movhi
.L522:
	cmp	r6, #1
	movw	r2, #2900
	mov	r1, #5
	strh	r1, [r4, r2]	@ movhi
	movw	r1, #2902
	mov	r0, #0
	strh	r0, [r4, r1]	@ movhi
	ldreq	r1, .L548
	ldrb	r9, [r4]	@ zero_extendqisi2
	ldrh	r8, [r5, #16]
	streqh	r6, [r1, r2]	@ movhi
	cmp	r9, #0
	movw	r2, #2904
	mov	r1, #4352
	strh	r1, [r4, r2]	@ movhi
	movne	r0, #384
	ldrne	r1, .L548
	ldrh	fp, [r5, #20]
	strneh	r0, [r1, r2]	@ movhi
	movw	r1, #2862
	ldrh	r7, [r4, r1]
	movw	r1, #2840
	ldr	r2, .L548
	smulbb	r7, r7, r3
	str	r2, [sp]
	uxth	r7, r7
	strh	r7, [r4, r1]	@ movhi
	movw	r1, #2850
	ldrh	r6, [r4, r1]
	movw	r1, #2906
	smulbb	r0, r7, r8
	smulbb	r3, r6, r3
	strh	r3, [r4, r1]	@ movhi
	movw	r3, #2908
	ldrh	r1, [r5, #18]
	strh	r8, [r4, r3]	@ movhi
	movw	r3, #2910
	strh	r1, [r4, r3]	@ movhi
	add	r3, r2, #2912
	str	r1, [sp, #4]
	strh	r0, [r3]	@ movhi
	movw	r3, #2914
	mov	r0, fp
	strh	fp, [r4, r3]	@ movhi
	bl	Ftl_log2
	movw	r3, #2916
	cmp	r6, #1024
	strh	r0, [r4, r3]	@ movhi
	mov	r3, fp, asl #9
	mov	r10, r0
	movw	r0, #2918
	uxth	r3, r3
	strh	r3, [r4, r0]	@ movhi
	movw	r0, #2920
	ldr	r2, [sp]
	mov	r3, r3, lsr #8
	strh	r3, [r4, r0]	@ movhi
	ldrh	r0, [r5, #26]
	movw	r3, #2922
	ldr	r1, [sp, #4]
	ldr	r5, .L548
	strh	r0, [r4, r3]	@ movhi
	movwhi	r0, #2902
	mul	r3, r6, r7
	mul	r1, r1, fp
	str	r3, [r4, #2856]
	uxtbhi	r3, r6
	strhih	r3, [r2, r0]	@ movhi
	movw	r3, #2902
	ldrh	r3, [r4, r3]
	rsb	r3, r3, r6
	mov	r6, r6, asl #6
	mul	r3, r3, r7
	mul	r3, fp, r3
	mul	r8, r8, r3
	movw	r3, #2904
	ldrh	r0, [r4, r3]
	mov	r0, r0, asl #3
	mov	r8, r8, asr #11
	str	r8, [r4, #2924]
	bl	__aeabi_idiv
	ldr	r8, .L548+8
	mov	r1, r7
	uxth	r0, r0
	cmp	r0, #4
	strhih	r0, [r8]	@ movhi
	movls	r3, #4
	strlsh	r3, [r8]	@ movhi
	cmp	r9, #0
	movwne	r3, #2904
	movne	r2, #640
	strneh	r2, [r4, r3]	@ movhi
	movw	r3, #2904
	ldrh	r3, [r4, r3]
	movw	r2, #2930
	mov	r3, r3, asr r10
	add	r10, r10, #9
	mov	r0, r6, asr r10
	add	r3, r3, #2
	strh	r3, [r4, r2]	@ movhi
	movw	r3, #2932
	strh	r0, [r4, r3]	@ movhi
	uxth	r0, r0
	add	r6, r0, #8
	mul	r3, r7, r0
	ldrh	r0, [r8]
	str	r3, [r4, #2936]
	bl	__aeabi_uidiv
	cmp	r7, #1
	uxtah	r0, r6, r0
	ldr	r6, .L548
	addeq	r0, r0, #4
	str	r0, [r5, #2836]
	ldr	r3, [r4, #2836]
	uxth	r0, r3
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #2836]
	ldr	r2, [r4, #2852]
	mov	r0, #2048
	str	r3, [r4, #2940]
	movw	r3, #2908
	ldrh	r3, [r4, r3]
	mov	r2, r2, asl #2
	mul	r3, r3, r2
	movw	r2, #2916
	ldrh	r5, [r4, r2]
	add	r5, r5, #9
	mov	r5, r3, lsr r5
	add	r3, r6, #2944
	add	r5, r5, #2
	uxth	r5, r5
	strh	r5, [r3]	@ movhi
	movw	r3, #2914
	ldrh	r7, [r4, r3]
	mov	r1, r7
	bl	__aeabi_idiv
	movw	r3, #2946
	add	r1, r6, #2928
	strh	r0, [r4, r3]	@ movhi
	mov	r3, #0
	ldrb	r0, [r4, #1888]	@ zero_extendqisi2
	str	r3, [r4, #1892]
	ldrh	r3, [r8]
	cmp	r0, #0
	add	r2, r3, #3
	strh	r2, [r8]	@ movhi
	ldr	r8, [r4, #2936]
	add	r2, r8, #3
	str	r2, [r4, #2936]
	addne	r3, r3, #4
	addne	r8, r8, #5
	strneh	r3, [r1]	@ movhi
	strne	r8, [r6, #2936]
	bne	.L536
.L535:
	cmp	r2, #7
	movls	r3, #8
	strls	r3, [r6, #2936]
.L536:
	ldr	r1, .L548
	movw	r2, #2948
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r3, r1, #2848
	ldrh	r0, [r3]
	mov	r3, r0, lsr #3
	add	r3, r3, r0, asl #1
	mov	r0, #0
	add	r3, r3, #52
	add	r5, r3, r5, asl #2
	cmp	r5, r7, asl #9
	movcc	r3, #1
	strcch	r3, [r1, r2]	@ movhi
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L549:
	.align	2
.L548:
	.word	.LANCHOR0
	.word	.LANCHOR0+2868
	.word	.LANCHOR0+2928
	.fnend
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	2
	.global	IsBlkInVendorPart
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L557
	movw	r3, #2950
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L556
	ldr	r3, [r2, #2952]
	add	r2, r2, #2928
	ldrh	r2, [r2]
	add	r2, r3, r2, asl #1
.L552:
	cmp	r3, r2
	beq	.L556
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L552
	mov	r0, #1
	bx	lr
.L556:
	mov	r0, #0
	bx	lr
.L558:
	.align	2
.L557:
	.word	.LANCHOR0
	.fnend
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheMetchLpa
	.type	FtlCacheMetchLpa, %function
FtlCacheMetchLpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L566
	ldr	r3, [r2, #2956]
	cmp	r3, #0
	beq	.L562
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	mov	r5, #36
	ldr	r4, [r2, #2960]
	mov	r2, #0
.L561:
	mla	ip, r5, r2, r4
	ldr	lr, [ip, #16]
	cmp	lr, r0
	movcc	ip, #0
	movcs	ip, #1
	cmp	lr, r1
	movhi	ip, #0
	cmp	ip, #0
	bne	.L563
	add	r2, r2, #1
	cmp	r2, r3
	bne	.L561
	mov	r0, ip
	ldmfd	sp!, {r4, r5, pc}
.L562:
	mov	r0, r3
	bx	lr
.L563:
	mov	r0, #1
	ldmfd	sp!, {r4, r5, pc}
.L567:
	.align	2
.L566:
	.word	.LANCHOR0
	.fnend
	.size	FtlCacheMetchLpa, .-FtlCacheMetchLpa
	.align	2
	.global	FtlGetCap
	.type	FtlGetCap, %function
FtlGetCap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L569
	ldr	r0, [r3, #1892]
	bx	lr
.L570:
	.align	2
.L569:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetCap, .-FtlGetCap
	.align	2
	.global	ftl_get_density
	.type	ftl_get_density, %function
ftl_get_density:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L572
	ldr	r0, [r3, #1892]
	bx	lr
.L573:
	.align	2
.L572:
	.word	.LANCHOR0
	.fnend
	.size	ftl_get_density, .-ftl_get_density
	.align	2
	.global	FtlGetLpn
	.type	FtlGetLpn, %function
FtlGetLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L575
	ldr	r0, [r3, #2964]
	bx	lr
.L576:
	.align	2
.L575:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLpn, .-FtlGetLpn
	.align	2
	.global	FtlBbmMapBadBlock
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #8
	movw	r3, #2906
	ldr	r5, .L579
	mov	r6, r0
	ldrh	r4, [r5, r3]
	mov	r1, r4
	bl	__aeabi_uidiv
	uxth	r2, r0
	smulbb	r3, r2, r4
	add	r1, r5, r2, asl #2
	mov	r4, #1
	ldr	ip, [r1, #2996]
	rsb	r3, r3, r6
	uxth	r3, r3
	and	r1, r3, #31
	mov	lr, r3, lsr #5
	ldr	r0, [ip, lr, asl #2]
	orr	r1, r0, r4, asl r1
	ldr	r0, .L579+4
	str	r1, [ip, lr, asl #2]
	str	r1, [sp]
	mov	r1, r6
	bl	printk
	add	r2, r5, #2960
	mov	r0, #0
	ldrh	r3, [r2, #14]
	add	r3, r3, r4
	strh	r3, [r2, #14]	@ movhi
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L580:
	.align	2
.L579:
	.word	.LANCHOR0
	.word	.LC2
	.fnend
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.global	__aeabi_uidivmod
	.align	2
	.global	FtlBbmIsBadBlock
	.type	FtlBbmIsBadBlock, %function
FtlBbmIsBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r3, #2906
	ldr	r5, .L583
	mov	r7, r0
	ldrh	r6, [r5, r3]
	mov	r1, r6
	bl	__aeabi_uidivmod
	mov	r0, r7
	uxth	r4, r1
	mov	r1, r6
	bl	__aeabi_uidiv
	mov	r2, r4, lsr #5
	and	r4, r4, #31
	uxth	r0, r0
	add	r5, r5, r0, asl #2
	ldr	r3, [r5, #2996]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr r4
	and	r0, r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L584:
	.align	2
.L583:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbmIsBadBlock, .-FtlBbmIsBadBlock
	.align	2
	.global	FtlBbtInfoPrint
	.type	FtlBbtInfoPrint, %function
FtlBbtInfoPrint:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FtlBbtInfoPrint, .-FtlBbtInfoPrint
	.align	2
	.global	FtlBbtMemInit
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L587
	movw	r3, #2968
	mvn	r1, #0
	add	r0, r2, r3
	strh	r1, [r2, r3]	@ movhi
	add	r0, r0, #12
	mov	r3, #0
	mov	r1, #255
	strh	r3, [r0, #-6]	@ movhi
	mov	r2, #16
	b	memset
.L588:
	.align	2
.L587:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movw	r3, #2906
	ldr	r4, .L598
	mov	r5, #0
	ldrh	r6, [r4, r3]
	movw	r3, #2862
	ldrh	r3, [r4, r3]
	mov	r4, r5
	mul	r6, r3, r6
.L590:
	uxth	r0, r5
	cmp	r0, r6
	bge	.L597
	bl	FtlBbmIsBadBlock
	add	r5, r5, #1
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
	b	.L590
.L597:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L599:
	.align	2
.L598:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	2
	.global	V2P_block
	.type	V2P_block, %function
V2P_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r1
	ldr	r6, .L602
	mov	r7, r0
	add	r3, r6, #2864
	ldrh	r5, [r3]
	mov	r1, r5
	bl	__aeabi_uidivmod
	mov	r0, r7
	smlabb	r4, r4, r5, r1
	mov	r1, r5
	bl	__aeabi_uidiv
	movw	r3, #2906
	ldrh	r3, [r6, r3]
	smlabb	r0, r3, r0, r4
	uxth	r0, r0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L603:
	.align	2
.L602:
	.word	.LANCHOR0
	.fnend
	.size	V2P_block, .-V2P_block
	.align	2
	.global	P2V_plane
	.type	P2V_plane, %function
P2V_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L606
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r2, r3, #2864
	mov	r6, r0
	ldrh	r5, [r2]
	movw	r2, #2906
	ldrh	r1, [r3, r2]
	bl	__aeabi_uidiv
	mov	r1, r5
	smulbb	r4, r0, r5
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r4, r1
	uxth	r0, r1
	ldmfd	sp!, {r4, r5, r6, pc}
.L607:
	.align	2
.L606:
	.word	.LANCHOR0
	.fnend
	.size	P2V_plane, .-P2V_plane
	.align	2
	.global	P2V_block_in_plane
	.type	P2V_block_in_plane, %function
P2V_block_in_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	movw	r3, #2906
	ldr	r4, .L610
	ldrh	r1, [r4, r3]
	add	r4, r4, #2864
	bl	__aeabi_uidivmod
	uxth	r0, r1
	ldrh	r1, [r4]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L611:
	.align	2
.L610:
	.word	.LANCHOR0
	.fnend
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	2
	.global	ftl_cmp_data_ver
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, r1
	bls	.L613
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L613:
	rsb	r0, r0, r1
	cmp	r0, #-2147483648
	movls	r0, #0
	movhi	r0, #1
	bx	lr
	.fnend
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L617
	movw	r2, #3028
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	r3, r1, r2
	mov	r4, #0
	strh	r0, [r1, r2]	@ movhi
	add	r0, r3, #8
	mov	r1, #2048
	strh	r4, [r3, #2]	@ movhi
	strh	r4, [r3, #4]	@ movhi
	strh	r4, [r3, #6]	@ movhi
	bl	__memzero
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L618:
	.align	2
.L617:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	2
	.global	FtlFreeSysBlkQueueEmpty
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L620
	ldrh	r0, [r3, #6]
	clz	r0, r0
	mov	r0, r0, lsr #5
	bx	lr
.L621:
	.align	2
.L620:
	.word	.LANCHOR0+3028
	.fnend
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	2
	.global	FtlFreeSysBlkQueueFull
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L623
	ldrh	r0, [r3, #6]
	sub	r0, r0, #1024
	clz	r0, r0
	mov	r0, r0, lsr #5
	bx	lr
.L624:
	.align	2
.L623:
	.word	.LANCHOR0+3028
	.fnend
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	sub	r3, r0, #1
	movw	r2, #65533
	mov	r7, r0
	uxth	r3, r3
	cmp	r3, r2
	ldmhifd	sp!, {r3, r4, r5, r6, r7, pc}
	ldr	r4, .L635
	ldrh	r3, [r4, #6]
	cmp	r3, #1024
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r1, #0
	beq	.L627
	ldr	r5, .L635+4
	ldr	r3, [r5, #-3100]
	cmp	r3, #0
	bne	.L627
	bl	P2V_block_in_plane
	mov	r1, #1
	mov	r3, r7, asl #10
	mov	r2, r1
	mov	r6, r0
	ldr	r0, [r5, #-3096]
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r1, [r5, #-3092]
	mov	r3, r6, asl #1
	ldrh	r2, [r1, r3]
	add	r2, r2, #1
	strh	r2, [r1, r3]	@ movhi
	ldr	r3, [r5, #-3088]
	add	r3, r3, #1
	str	r3, [r5, #-3088]
.L627:
	ldrh	r3, [r4, #6]
	add	r3, r3, #1
	strh	r3, [r4, #6]	@ movhi
	ldrh	r3, [r4, #4]
	add	r2, r4, r3, asl #1
	add	r3, r3, #1
	ubfx	r3, r3, #0, #10
	strh	r3, [r4, #4]	@ movhi
	strh	r7, [r2, #8]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L636:
	.align	2
.L635:
	.word	.LANCHOR0+3028
	.word	.LANCHOR2
	.fnend
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L650
	ldrh	ip, [r3, #28]
	ldr	r3, .L650+4
	ldrh	r2, [r3, #6]
	cmp	r2, #0
	bxeq	lr
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r0, #0
	ldrh	r1, [r3, #2]
	and	ip, ip, #31
	ldrh	r2, [r3, #4]
	mov	r4, r0
.L639:
	uxth	lr, r0
	add	r0, r0, #1
	cmp	lr, ip
	bge	.L649
	add	lr, r3, r1, asl #1
	add	r1, r1, #1
	ubfx	r1, r1, #0, #10
	ldrh	r4, [lr, #8]
	add	lr, r3, r2, asl #1
	strh	r4, [lr, #8]	@ movhi
	mov	r4, #1
	add	r2, r2, r4
	ubfx	r2, r2, #0, #10
	b	.L639
.L649:
	cmp	r4, #0
	strneh	r1, [r3, #2]	@ movhi
	strneh	r2, [r3, #4]	@ movhi
	ldmfd	sp!, {r4, pc}
.L651:
	.align	2
.L650:
	.word	.LANCHOR2-3084
	.word	.LANCHOR0+3028
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	ldr	r4, .L663
	ldr	r8, .L663+4
	mov	r7, r4
.L653:
	ldrh	r1, [r4, #6]
	cmp	r1, #0
	beq	.L654
	ldr	r5, .L663+8
	sub	r1, r1, #1
	ldrh	r3, [r4, #2]
	strh	r1, [r4, #6]	@ movhi
	ldr	r10, [r5, #-3100]
	add	r2, r4, r3, asl #1
	add	r3, r3, #1
	cmp	r10, #0
	ubfx	r3, r3, #0, #10
	ldrh	r6, [r2, #8]
	strh	r3, [r4, #2]	@ movhi
	bne	.L655
	mov	r0, r6
	bl	P2V_block_in_plane
	mov	r3, r6, asl #10
	mov	r9, r0
	ldr	r0, [r5, #-3096]
	str	r3, [r0, #4]
	ldrb	r3, [r8, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L656
	mov	r1, r10
	mov	r2, #1
	bl	FlashEraseBlocks
.L656:
	mov	r1, #1
	ldr	r0, [r5, #-3096]
	mov	r2, r1
	bl	FlashEraseBlocks
	ldr	r1, [r5, #-3092]
	mov	r3, r9, asl #1
	ldrh	r2, [r1, r3]
	add	r2, r2, #1
	strh	r2, [r1, r3]	@ movhi
	ldr	r3, [r5, #-3088]
	add	r3, r3, #1
	str	r3, [r5, #-3088]
	b	.L655
.L654:
	ldr	r0, .L663+12
	bl	printk
.L657:
	b	.L657
.L655:
	sub	r3, r6, #1
	movw	r2, #65533
	uxth	r3, r3
	cmp	r3, r2
	bls	.L658
	mov	r1, r6
	ldrh	r2, [r7, #6]
	ldr	r0, .L663+16
	bl	printk
	b	.L653
.L658:
	mov	r0, r6
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L664:
	.align	2
.L663:
	.word	.LANCHOR0+3028
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC3
	.word	.LC4
	.fnend
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	test_node_in_list
	.type	test_node_in_list, %function
test_node_in_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L671
	str	lr, [sp, #-4]!
	.save {lr}
	movw	lr, #65535
	ldr	ip, [r3, #-3036]
	ldr	r2, [r0]
	ldr	r3, .L671+4
	rsb	r0, ip, r2
	mov	r0, r0, asr #1
	mul	r3, r3, r0
	mov	r0, #6
	uxth	r3, r3
.L667:
	cmp	r1, r3
	beq	.L668
	ldrh	r3, [r2]
	cmp	r3, lr
	beq	.L669
	mla	r2, r0, r3, ip
	b	.L667
.L668:
	mov	r0, #1
	ldr	pc, [sp], #4
.L669:
	mov	r0, #0
	ldr	pc, [sp], #4
.L672:
	.align	2
.L671:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	test_node_in_list, .-test_node_in_list
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r5, .L690
	ldrh	r3, [r5]
	cmp	r3, r0
	bls	.L675
	ldr	r2, .L690+4
	mov	lr, #6
	mul	lr, lr, r0
	mvn	ip, #0
	ldr	r6, [r2, #-3036]
	mov	r10, r2
	add	r1, r6, lr
	strh	ip, [r1, #2]	@ movhi
	strh	ip, [r6, lr]	@ movhi
	ldr	r3, [r2, #-3032]
	cmp	r3, #0
	beq	.L689
	ldr	r8, [r2, #-3028]
	mov	r4, r0, asl #1
	ldrh	r2, [r1, #4]
	ldrh	r5, [r5]
	ldrh	r7, [r8, r4]
	cmp	r2, #0
	str	r5, [sp]
	mulne	ip, r2, r7
	ldr	r7, [r10, #-3036]
	ldr	r2, .L690+8
	rsb	r9, r7, r3
	mov	r9, r9, asr #1
	mul	r2, r2, r9
	ldr	r9, [r10, #-3092]
	add	r4, r9, r4
	str	r4, [sp, #4]
	mov	r4, #0
	uxth	r2, r2
.L684:
	add	r4, r4, #1
	ldr	r5, [sp]
	uxth	r4, r4
	cmp	r4, r5
	movls	r5, #0
	movhi	r5, #1
	cmp	r0, r2
	orreq	r5, r5, #1
	cmp	r5, #0
	bne	.L675
	mov	r10, r2, asl #1
	ldrh	r5, [r3, #4]
	ldrh	fp, [r8, r10]
	cmp	r5, #0
	mvneq	r5, #0
	mulne	r5, r5, fp
	cmp	r5, ip
	bne	.L680
	ldr	r5, [sp, #4]
	ldrh	r10, [r9, r10]
	ldrh	r5, [r5]
	cmp	r10, r5
	bcc	.L682
	b	.L681
.L680:
	bhi	.L681
.L682:
	ldrh	r5, [r3]
	movw	r10, #65535
	cmp	r5, r10
	streqh	r2, [r1, #2]	@ movhi
	streqh	r0, [r3]	@ movhi
	ldreq	r3, .L690+4
	streq	r1, [r3, #-3024]
	beq	.L675
.L683:
	mov	r3, #6
	mov	r2, r5
	mla	r3, r3, r5, r7
	b	.L684
.L681:
	strh	r2, [r6, lr]	@ movhi
	ldrh	r2, [r3, #2]
	strh	r2, [r1, #2]	@ movhi
	ldr	r2, .L690+4
	ldr	ip, [r2, #-3032]
	cmp	r3, ip
	bne	.L685
	strh	r0, [r3, #2]	@ movhi
.L689:
	str	r1, [r2, #-3032]
	b	.L675
.L685:
	ldrh	ip, [r3, #2]
	ldr	r1, [r2, #-3036]
	mov	r2, #6
	mul	r2, r2, ip
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L675:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L691:
	.align	2
.L690:
	.word	.LANCHOR0+2848
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	insert_data_list, .-insert_data_list
	.align	2
	.global	INSERT_DATA_LIST
	.type	INSERT_DATA_LIST, %function
INSERT_DATA_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	insert_data_list
	ldr	r2, .L694
	ldrh	r3, [r2, #-12]
	add	r3, r3, #1
	strh	r3, [r2, #-12]	@ movhi
	ldmfd	sp!, {r3, pc}
.L695:
	.align	2
.L694:
	.word	.LANCHOR2-3008
	.fnend
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	2
	.global	insert_free_list
	.type	insert_free_list, %function
insert_free_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movw	r4, #65535
	cmp	r0, r4
	beq	.L697
	ldr	r2, .L705
	mov	lr, #6
	mul	r6, lr, r0
	mvn	r3, #0
	ldr	r7, [r2, #-3036]
	mov	r5, r2
	add	ip, r7, r6
	strh	r3, [ip, #2]	@ movhi
	strh	r3, [r7, r6]	@ movhi
	ldr	r3, [r2, #-3016]
	cmp	r3, #0
	beq	.L704
	ldr	r9, [r2, #-3092]
	mov	r2, r0, asl #1
	ldr	r8, [r5, #-3036]
	rsb	r1, r8, r3
	ldrh	r10, [r9, r2]
	ldr	r2, .L705+4
	mov	r1, r1, asr #1
	mul	r1, r2, r1
	uxth	r2, r1
.L701:
	mov	r1, r2, asl #1
	ldrh	r1, [r9, r1]
	cmp	r1, r10
	bcs	.L699
	ldrh	r1, [r3]
	cmp	r1, r4
	streqh	r2, [ip, #2]	@ movhi
	streqh	r0, [r3]	@ movhi
	beq	.L697
.L700:
	mla	r3, lr, r1, r8
	mov	r2, r1
	b	.L701
.L699:
	ldrh	r1, [r3, #2]
	strh	r1, [ip, #2]	@ movhi
	strh	r2, [r7, r6]	@ movhi
	ldr	r1, [r5, #-3016]
	ldr	r2, .L705
	cmp	r3, r1
	bne	.L702
	strh	r0, [r3, #2]	@ movhi
.L704:
	str	ip, [r2, #-3016]
	b	.L697
.L702:
	ldrh	ip, [r3, #2]
	ldr	r1, [r2, #-3036]
	mov	r2, #6
	mul	r2, r2, ip
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L697:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L706:
	.align	2
.L705:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	insert_free_list, .-insert_free_list
	.align	2
	.global	INSERT_FREE_LIST
	.type	INSERT_FREE_LIST, %function
INSERT_FREE_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	insert_free_list
	ldr	r2, .L709
	ldrh	r3, [r2, #-4]
	add	r3, r3, #1
	strh	r3, [r2, #-4]	@ movhi
	ldmfd	sp!, {r3, pc}
.L710:
	.align	2
.L709:
	.word	.LANCHOR2-3008
	.fnend
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.type	List_remove_node, %function
List_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	mov	ip, #6
	ldr	r4, .L717
	movw	r5, #65535
	mul	r1, ip, r1
	ldr	r3, [r0]
	ldr	r2, [r4, #-3036]
	add	lr, r2, r1
	cmp	lr, r3
	ldrh	r3, [r2, r1]
	bne	.L712
	cmp	r3, r5
	mlane	r3, ip, r3, r2
	moveq	r3, #0
	streq	r3, [r0]
	strne	r3, [r0]
	mvnne	r0, #0
	strneh	r0, [r3, #2]	@ movhi
	b	.L714
.L712:
	cmp	r3, r5
	ldrh	r0, [lr, #2]
	bne	.L715
	cmp	r0, r3
	mulne	r0, ip, r0
	mvnne	r3, #0
	strneh	r3, [r2, r0]	@ movhi
	b	.L714
.L715:
	mla	r3, ip, r3, r2
	strh	r0, [r3, #2]	@ movhi
	ldrh	r5, [lr, #2]
	ldrh	r0, [r2, r1]
	ldr	r3, [r4, #-3036]
	mul	ip, ip, r5
	strh	r0, [r3, ip]	@ movhi
.L714:
	mvn	r3, #0
	mov	r0, #0
	strh	r3, [r2, r1]	@ movhi
	strh	r3, [lr, #2]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L718:
	.align	2
.L717:
	.word	.LANCHOR2
	.fnend
	.size	List_remove_node, .-List_remove_node
	.align	2
	.global	List_pop_index_node
	.type	List_pop_index_node, %function
List_pop_index_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, [r0]
	cmp	r3, #0
	beq	.L725
	ldr	r2, .L728
	movw	ip, #65535
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	lr, #6
	ldr	r4, [r2, #-3036]
.L721:
	cmp	r1, #0
	bne	.L722
.L724:
	rsb	r4, r4, r3
	ldr	r3, .L728+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L722:
	ldrh	r2, [r3]
	cmp	r2, ip
	beq	.L724
	sub	r1, r1, #1
	mla	r3, lr, r2, r4
	uxth	r1, r1
	b	.L721
.L725:
	movw	r0, #65535
	bx	lr
.L729:
	.align	2
.L728:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	List_pop_index_node, .-List_pop_index_node
	.align	2
	.global	List_get_gc_head_node
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L737
	ldr	r3, [r2, #-3032]
	cmp	r3, #0
	ldrne	r1, [r2, #-3036]
	movne	ip, #6
	movwne	r2, #65535
	beq	.L735
.L732:
	cmp	r0, #0
	beq	.L733
	ldrh	r3, [r3]
	cmp	r3, r2
	subne	r0, r0, #1
	mlane	r3, ip, r3, r1
	uxthne	r0, r0
	bne	.L732
.L735:
	movw	r0, #65535
	bx	lr
.L733:
	rsb	r3, r1, r3
	ldr	r0, .L737+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	bx	lr
.L738:
	.align	2
.L737:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	2
	.global	List_update_data_list
	.type	List_update_data_list, %function
List_update_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L747
	sub	r5, r4, #3008
	ldrh	r3, [r5]
	cmp	r3, r0
	beq	.L740
	sub	r3, r4, #2960
	ldrh	r3, [r3]
	cmp	r3, r0
	beq	.L740
	sub	r3, r4, #2912
	ldrh	r3, [r3]
	cmp	r3, r0
	beq	.L740
	mov	r1, #6
	ldr	r3, [r4, #-3036]
	mul	r1, r1, r0
	ldr	r2, [r4, #-3032]
	add	ip, r3, r1
	cmp	ip, r2
	beq	.L740
	ldr	lr, [r4, #-3028]
	mov	r2, r0, asl #1
	ldrh	r4, [lr, r2]
	ldrh	r2, [ip, #4]
	ldrh	ip, [ip, #2]
	cmp	r2, #0
	mulne	r2, r2, r4
	mvneq	r2, #0
	movw	r4, #65535
	cmp	ip, r4
	bne	.L742
	ldrh	r1, [r3, r1]
	cmp	r1, ip
	beq	.L740
.L742:
	mov	r1, #6
	mul	ip, r1, ip
	ldr	r1, .L747+4
	add	r3, r3, ip
	mov	r4, ip, asr #1
	ldrh	r3, [r3, #4]
	mul	r1, r1, r4
	cmp	r3, #0
	mvneq	r3, #0
	mov	r1, r1, asl #1
	ldrh	r1, [lr, r1]
	mulne	r3, r3, r1
	cmp	r2, r3
	bcs	.L740
	mov	r4, r0
	ldr	r0, .L747+8
	mov	r1, r4
	bl	List_remove_node
	ldrh	r3, [r5, #-12]
	mov	r0, r4
	sub	r3, r3, #1
	strh	r3, [r5, #-12]	@ movhi
	bl	INSERT_DATA_LIST
.L740:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L748:
	.align	2
.L747:
	.word	.LANCHOR2
	.word	-1431655765
	.word	.LANCHOR2-3032
	.fnend
	.size	List_update_data_list, .-List_update_data_list
	.align	2
	.global	ftl_map_blk_alloc_new_blk
	.type	ftl_map_blk_alloc_new_blk, %function
ftl_map_blk_alloc_new_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r3, #0
	ldrh	r1, [r0, #10]
	ldr	r2, [r0, #12]
.L750:
	uxth	r5, r3
	cmp	r5, r1
	bcs	.L753
	mov	r7, r2
	add	r3, r3, #1
	ldrh	r6, [r7]
	add	r2, r2, #2
	cmp	r6, #0
	bne	.L750
	mov	r4, r0
	bl	FtlFreeSysBlkQueueOut
	movw	r2, #65533
	sub	r3, r0, #1
	mov	r1, r0
	strh	r0, [r7]	@ movhi
	uxth	r3, r3
	cmp	r3, r2
	bls	.L751
	ldr	r3, .L757
	ldr	r0, .L757+4
	ldrh	r2, [r3, #6]
	bl	printk
.L752:
	b	.L752
.L751:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L753:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L758:
	.align	2
.L757:
	.word	.LANCHOR0+3028
	.word	.LC5
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L775
	movw	r3, #2946
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r1, #0
	ldrh	r2, [r2, r3]
	mov	ip, #12
	ldr	r3, .L775+4
	movw	lr, #65535
	ldr	r3, [r3, #-2864]
.L760:
	uxth	r0, r1
	cmp	r0, r2
	bcs	.L772
	add	r1, r1, #1
	mla	r4, ip, r1, r3
	ldrh	r4, [r4, #-12]
	cmp	r4, lr
	bne	.L760
	ldmfd	sp!, {r4, r5, r6, pc}
.L772:
	mov	r0, r2
	mov	r1, #0
	mov	ip, #-2147483648
	mov	r5, #12
.L763:
	uxth	r4, r1
	cmp	r4, r2
	bcs	.L773
	mla	lr, r5, r1, r3
	add	r1, r1, #1
	ldr	lr, [lr, #4]
	cmp	lr, ip
	mvn	r6, lr
	mov	r6, r6, lsr #31
	movcs	r6, #0
	cmp	r6, #0
	movne	ip, lr
	movne	r0, r4
	b	.L763
.L773:
	cmp	r0, r2
	ldmccfd	sp!, {r4, r5, r6, pc}
	ldr	r1, .L775+8
	mov	r0, r2
	mvn	ip, #0
	ldrh	r5, [r1, #-12]
	mov	r1, #0
.L766:
	uxth	lr, r1
	cmp	lr, r2
	bcs	.L774
	ldr	r4, [r3, #4]
	cmp	r4, ip
	bcs	.L767
	ldrh	r6, [r3]
	cmp	r6, r5
	movne	ip, r4
	movne	r0, lr
.L767:
	add	r1, r1, #1
	add	r3, r3, #12
	b	.L766
.L774:
	ldmfd	sp!, {r4, r5, r6, pc}
.L776:
	.align	2
.L775:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2-2848
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L785
	sub	r1, r3, #2848
	ldrh	r2, [r1, #-10]
	cmp	r2, #4
	cmpls	r0, #0
	addeq	r2, r2, #1
	streqh	r2, [r1, #-10]	@ movhi
	bxeq	lr
	str	lr, [sp, #-4]!
	.save {lr}
	mov	r2, #0
	strh	r2, [r1, #-10]	@ movhi
	movw	ip, #65535
	ldr	r1, .L785+4
	str	r2, [r3, #-2856]
	ldr	r2, [r3, #-3028]
	ldrh	r0, [r1]
	add	r0, r2, r0, asl #1
.L779:
	cmp	r2, r0
	beq	.L784
	ldrh	r1, [r2], #2
	cmp	r1, ip
	ldrne	lr, [r3, #-2856]
	addne	r1, r1, lr
	strne	r1, [r3, #-2856]
	b	.L779
.L784:
	ldr	pc, [sp], #4
.L786:
	.align	2
.L785:
	.word	.LANCHOR2
	.word	.LANCHOR0+2848
	.fnend
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	ftl_set_blk_mode
	.type	ftl_set_blk_mode, %function
ftl_set_blk_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L791
	mov	r3, r0, lsr #5
	cmp	r1, #0
	and	r0, r0, #31
	uxth	r3, r3
	mov	ip, #1
	ldr	r1, [r2, #-2852]
	ldr	r2, [r1, r3, asl #2]
	orrne	r0, r2, ip, asl r0
	biceq	r0, r2, ip, asl r0
	str	r0, [r1, r3, asl #2]
	bx	lr
.L792:
	.align	2
.L791:
	.word	.LANCHOR2
	.fnend
	.size	ftl_set_blk_mode, .-ftl_set_blk_mode
	.align	2
	.global	ftl_get_blk_mode
	.type	ftl_get_blk_mode, %function
ftl_get_blk_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L794
	mov	r2, r0, lsr #5
	and	r0, r0, #31
	ldr	r3, [r3, #-2852]
	ldr	r3, [r3, r2, asl #2]
	mov	r0, r3, lsr r0
	and	r0, r0, #1
	bx	lr
.L795:
	.align	2
.L794:
	.word	.LANCHOR2
	.fnend
	.size	ftl_get_blk_mode, .-ftl_get_blk_mode
	.align	2
	.global	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	r3, #0
	strh	r3, [r0, #4]	@ movhi
	ldr	r3, .L806
	movw	ip, #2840
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	movw	r5, #65535
	ldrh	lr, [r3, ip]
	add	ip, r2, #7
	add	ip, r0, ip, asl #1
.L797:
	cmp	r2, lr
	bcs	.L804
	ldrh	r4, [ip, #2]!
	add	r2, r2, #1
	cmp	r4, r5
	uxth	r2, r2
	ldrneh	r4, [r0, #4]
	addne	r4, r4, #1
	strneh	r4, [r0, #4]	@ movhi
	b	.L797
.L804:
	movw	r2, #2908
	add	ip, r0, #14
	ldrh	r4, [r3, r2]
	movw	r5, #65535
	mov	r2, #0
.L800:
	uxth	r3, r2
	cmp	r3, lr
	bcs	.L805
	ldrh	r3, [ip, #2]!
	add	r2, r2, #1
	cmp	r3, r5
	ldrneh	r3, [r0, #4]
	addne	r3, r4, r3
	subne	r3, r3, #1
	rsbne	r3, r1, r3
	strneh	r3, [r0, #4]	@ movhi
	b	.L800
.L805:
	ldmfd	sp!, {r4, r5, pc}
.L807:
	.align	2
.L806:
	.word	.LANCHOR0
	.fnend
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.align	2
	.global	make_superblock
	.type	make_superblock, %function
make_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r2, #2840
	ldr	r6, .L822
	mov	r3, #0
	ldr	r10, .L822+4
	add	r7, r0, #16
	mov	r4, r0
	mov	r5, r3
	ldrh	r8, [r6, r2]
	mvn	r9, #0
	strh	r3, [r0, #4]	@ movhi
	strb	r3, [r0, #7]
.L809:
	uxth	r3, r5
	cmp	r3, r8
	bcs	.L821
	ldrb	r0, [r10, r5]	@ zero_extendqisi2
	add	r7, r7, #2
	ldrh	r1, [r4]
	add	r5, r5, #1
	bl	V2P_block
	strh	r9, [r7, #-2]	@ movhi
	mov	fp, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	streqh	fp, [r7, #-2]	@ movhi
	ldreqb	r3, [r4, #7]	@ zero_extendqisi2
	addeq	r3, r3, #1
	streqb	r3, [r4, #7]
	b	.L809
.L821:
	movw	r3, #2908
	ldrb	r2, [r4, #7]	@ zero_extendqisi2
	ldrh	r3, [r6, r3]
	smulbb	r3, r2, r3
	strh	r3, [r4, #4]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #9]
	ldr	r3, [r6, #2832]
	cmp	r3, #0
	beq	.L812
	ldr	r2, .L822+8
	ldrh	r3, [r4]
	ldr	r2, [r2, #-3092]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #79
	movls	r3, #1
	strlsb	r3, [r4, #9]
.L812:
	ldrb	r3, [r6]	@ zero_extendqisi2
	mov	r0, #0
	cmp	r3, #0
	movne	r3, #1
	strneb	r3, [r4, #9]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L823:
	.align	2
.L822:
	.word	.LANCHOR0
	.word	.LANCHOR0+2868
	.word	.LANCHOR2
	.fnend
	.size	make_superblock, .-make_superblock
	.align	2
	.global	update_multiplier_value
	.type	update_multiplier_value, %function
update_multiplier_value:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	movw	r2, #2840
	ldr	r3, .L834
	mov	r5, #0
	ldr	r9, .L834+4
	mov	r6, r0
	mov	r4, r5
	ldrh	r7, [r3, r2]
	movw	r2, #2908
	ldrh	r8, [r3, r2]
.L825:
	uxth	r3, r5
	cmp	r3, r7
	bcs	.L833
	ldrb	r0, [r9, r5]	@ zero_extendqisi2
	mov	r1, r6
	bl	V2P_block
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addeq	r4, r4, r8
	uxtheq	r4, r4
	b	.L825
.L833:
	cmp	r4, #0
	beq	.L828
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L828:
	ldr	r3, .L834+8
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #-3036]
	mla	r6, r2, r6, r3
	strh	r4, [r6, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L835:
	.align	2
.L834:
	.word	.LANCHOR0
	.word	.LANCHOR0+2868
	.word	.LANCHOR2
	.fnend
	.size	update_multiplier_value, .-update_multiplier_value
	.align	2
	.global	GetFreeBlockMinEraseCount
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L839
	ldr	r0, [r2, #-3016]
	cmp	r0, #0
	bxeq	lr
	ldr	r3, [r2, #-3036]
	rsb	r0, r3, r0
	ldr	r3, .L839+4
	mov	r0, r0, asr #1
	mul	r0, r3, r0
	ldr	r3, [r2, #-3092]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r0, [r3, r0]
	bx	lr
.L840:
	.align	2
.L839:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	2
	.global	GetFreeBlockMaxEraseCount
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L851
	ldr	r3, [r1, #-3016]
	cmp	r3, #0
	beq	.L847
	sub	r2, r1, #3008
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	mov	r4, #6
	ldrh	r2, [r2, #-4]
	movw	r5, #65535
	ldr	ip, [r1, #-3036]
	rsb	r2, r2, r2, asl #3
	rsb	r3, ip, r3
	mov	r2, r2, asr #3
	mov	r3, r3, asr #1
	cmp	r0, r2
	uxthgt	r0, r2
	ldr	r2, .L851+4
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
.L844:
	uxth	lr, r2
	cmp	lr, r0
	bcs	.L846
	mul	lr, r4, r3
	add	r2, r2, #1
	ldrh	lr, [ip, lr]
	cmp	lr, r5
	bne	.L848
.L846:
	ldr	r2, [r1, #-3092]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L848:
	mov	r3, lr
	b	.L844
.L847:
	mov	r0, r3
	bx	lr
.L852:
	.align	2
.L851:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	2
	.global	FtlPrintInfo2buf
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r7, r0
	ldr	r8, .L864
	add	r4, r7, #12
	ldr	r1, .L864+4
	.pad #32
	sub	sp, sp, #32
	bl	strcpy
	mov	r0, r4
	ldr	r1, .L864+8
	ldr	r2, [r8, #1860]
	bl	sprintf
	ldr	r1, .L864+12
	ldr	r2, [r8, #2924]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L864+16
	ldr	r3, [r3, #504]
	cmp	r3, #1
	add	r4, r4, r0
	bne	.L859
	add	r0, sp, #16
	add	r1, sp, #20
	add	r2, sp, #24
	add	r3, sp, #28
	bl	NandcGetTimeCfg
	mov	r0, r4
	ldr	r1, .L864+20
	ldr	r6, .L864+24
	sub	r10, r6, #3008
	sub	r9, r6, #2768
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #16]
	str	r3, [sp]
	ldr	r3, [sp, #28]
	str	r3, [sp, #4]
	ldr	r3, [sp, #20]
	bl	sprintf
	ldr	r1, .L864+28
	add	r4, r4, r0
	add	r5, r4, #10
	mov	r0, r4
	sub	r4, r6, #2784
	bl	strcpy
	mov	r0, r5
	ldr	r1, .L864+32
	ldr	r2, [r8, #2964]
	bl	sprintf
	ldr	r1, .L864+36
	ldr	r2, [r6, #-2856]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+40
	ldr	r2, [r6, #-2848]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+44
	ldr	r2, [r6, #-2844]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+48
	ldr	r2, [r6, #-2840]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+52
	ldr	r2, [r6, #-2836]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+56
	ldr	r2, [r6, #-2832]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+60
	ldr	r2, [r6, #-2828]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #-2824]
	ldr	r1, .L864+64
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #-2820]
	ldr	r1, .L864+68
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+72
	ldr	r2, [r6, #-2816]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+76
	ldr	r2, [r6, #-2812]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	FtlBbtCalcTotleCnt
	ldr	r2, .L864+80
	ldr	r1, .L864+84
	ldrh	r2, [r2, #6]
	mov	r3, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+88
	ldrh	r2, [r10, #-4]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+92
	ldr	r2, [r6, #-2808]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+96
	ldr	r2, [r6, #-2804]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+100
	ldr	r2, [r6, #-2800]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+104
	ldr	r2, [r6, #-3088]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+108
	ldr	r2, [r6, #-2796]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+112
	ldr	r2, [r6, #-2792]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+116
	ldrh	r2, [r10, #-46]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+120
	ldrh	r2, [r10, #-48]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+124
	ldr	r2, [r8, #1892]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+128
	ldr	r2, [r8, #2940]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+132
	ldr	r2, [r8, #2836]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L864+136
	ldr	r1, .L864+140
	ldrh	r2, [r3, #6]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r3, r8, #2848
	ldr	r1, .L864+144
	ldrh	r2, [r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, #-4]
	ldr	r1, .L864+148
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+152
	ldr	r2, [r8, #2852]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4]
	ldr	r1, .L864+156
	sub	r4, r6, #2960
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #2968
	ldrh	r2, [r8, r3]
	ldr	r1, .L864+160
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+164
	ldrh	r2, [r10, #2]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+168
	ldrb	r2, [r6, #-3002]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+172
	ldrh	r2, [r10]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+176
	ldrb	r2, [r6, #-3000]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+180
	ldrh	r2, [r10, #4]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r10]
	ldr	r2, [r6, #-3028]
	ldr	r1, .L864+184
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, #2]
	ldr	r1, .L864+188
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+192
	ldrb	r2, [r6, #-2954]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4]
	ldr	r1, .L864+196
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+200
	ldrb	r2, [r6, #-2952]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, #4]
	ldr	r1, .L864+204
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4]
	ldr	r2, [r6, #-3028]
	sub	r4, r6, #2912
	ldr	r1, .L864+208
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, #2]
	ldr	r1, .L864+212
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+216
	ldrb	r2, [r6, #-2906]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4]
	ldr	r1, .L864+220
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+224
	ldrb	r2, [r6, #-2904]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, #4]
	ldr	r1, .L864+228
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+232
	ldrh	r2, [r9, #2]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+236
	ldrb	r2, [r6, #-2762]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+240
	ldrh	r2, [r9]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+244
	ldrb	r2, [r6, #-2760]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L864+248
	ldrh	r2, [r9, #4]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, [r6, #-2632]
	ldr	r3, [r8, #2832]
	ldr	r2, [r6, #-2720]
	orr	r2, r3, r2, asl #8
	ldr	r3, [r6, #-2636]
	str	r1, [sp]
	add	r5, r5, r0
	ldr	r1, [r6, #-2640]
	mov	r0, r5
	str	r1, [sp, #4]
	ldr	r1, .L864+252
	bl	sprintf
	ldr	r1, .L864+256
	ldr	r2, [r6, #-2644]
	add	r4, r5, r0
	sub	r5, r6, #2192
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L864+260
	ldr	r2, [r6, #-2620]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L864+264
	ldrh	r2, [r5, #-12]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L864+268
	ldrh	r2, [r5, #-10]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L864+272
	ldr	r2, [r6, #-2200]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L864+276
	ldrh	r2, [r5, #-4]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L864+280
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	ldrh	r0, [r10, #-4]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L864+284
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r9]
	movw	r2, #65535
	cmp	r3, r2
	add	r4, r4, r0
	beq	.L856
	ldr	r2, [r6, #-3028]
	mov	r3, r3, asl #1
	mov	r0, r4
	ldr	r1, .L864+288
	ldrh	r2, [r2, r3]
	bl	sprintf
	add	r4, r4, r0
.L856:
	mov	r0, #0
	mov	r5, #0
	bl	List_get_gc_head_node
	movw	r10, #65535
	mov	r9, #6
	uxth	r3, r0
.L858:
	cmp	r3, r10
	beq	.L857
	ldr	r2, [r6, #-3028]
	mov	r1, r3, asl #1
	mul	r8, r9, r3
	mov	r0, r4
	ldrh	r2, [r2, r1]
	str	r2, [sp]
	ldr	r2, [r6, #-3036]
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r2, [sp, #4]
	ldr	r2, [r6, #-3092]
	ldrh	r2, [r2, r1]
	ldr	r1, .L864+292
	str	r2, [sp, #8]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-3036]
	cmp	r5, #16
	ldrh	r3, [r3, r8]
	add	r4, r4, r0
	bne	.L858
.L857:
	ldr	r2, [r6, #-3036]
	mov	r5, #0
	ldr	r3, [r6, #-3016]
	movw	r10, #65535
	mov	r9, #6
	rsb	r3, r2, r3
	ldr	r2, .L864+296
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L860:
	cmp	r3, r10
	beq	.L859
	mul	r8, r9, r3
	ldr	r2, [r6, #-3036]
	ldr	r1, [r6, #-3092]
	mov	r0, r4
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r2, [sp]
	mov	r2, r3, asl #1
	ldrh	r2, [r1, r2]
	ldr	r1, .L864+300
	str	r2, [sp, #4]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-3036]
	cmp	r5, #4
	ldrh	r3, [r3, r8]
	add	r4, r4, r0
	bne	.L860
.L859:
	rsb	r0, r7, r4
	add	sp, sp, #32
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L865:
	.align	2
.L864:
	.word	.LANCHOR0
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
	.word	.LANCHOR0+2968
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
	.word	.LANCHOR0+3028
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
	.align	2
	.global	ftl_proc_ftl_read
	.type	ftl_proc_ftl_read, %function
ftl_proc_ftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r1, .L868
	ldr	r2, .L868+4
	bl	sprintf
	add	r4, r5, r0
	mov	r0, r4
	bl	FtlPrintInfo2buf
	add	r0, r4, r0
	rsb	r0, r5, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L869:
	.align	2
.L868:
	.word	.LC76
	.word	.LC77
	.fnend
	.size	ftl_proc_ftl_read, .-ftl_proc_ftl_read
	.align	2
	.global	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L900
	ldr	r2, [r4, #-2800]
	ldr	r3, [r4, #-2792]
	cmp	r2, r3
	bcs	.L871
	ldr	r2, .L900+4
	mov	r3, #0
	str	r3, [r4, #-2808]
	ldrh	r1, [r2]
	ldr	r2, [r4, #-3092]
	sub	r2, r2, #2
.L872:
	cmp	r3, r1
	bcs	.L899
	ldrh	r0, [r2, #2]!
	add	r3, r3, #1
	ldr	ip, [r4, #-2808]
	add	r0, r0, ip
	str	r0, [r4, #-2808]
	b	.L872
.L899:
	ldr	r5, [r4, #-2808]
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r2, .L900+8
	movw	r3, #2900
	ldrh	r1, [r2, r3]
	str	r0, [r4, #-2800]
	ldr	r0, [r4, #-2804]
	rsb	r0, r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #-2808]
	b	.L874
.L871:
	ldr	r3, [r4, #-2796]
	cmp	r2, r3
	bls	.L874
	ldr	ip, .L900+4
	add	r3, r3, #1
	str	r3, [r4, #-2796]
	mov	r3, #0
.L876:
	ldrh	r2, [ip]
	cmp	r3, r2
	bcs	.L874
	ldr	r0, [r4, #-3092]
	mov	r1, r3, asl #1
	add	r3, r3, #1
	ldrh	r2, [r0, r1]
	add	r2, r2, #1
	strh	r2, [r0, r1]	@ movhi
	b	.L876
.L874:
	ldr	r6, [r4, #-2792]
	ldr	r8, [r4, #-2800]
	add	r3, r6, #256
	cmp	r3, r8
	bls	.L879
	ldr	r2, .L900
	add	r3, r6, #768
	ldr	r2, [r2, #-2796]
	cmp	r3, r2
	bls	.L879
	ldr	r3, .L900+8
	cmp	r6, #40
	ldr	r2, [r3, #2832]
	movls	r3, #0
	movhi	r3, #1
	cmp	r2, #0
	orreq	r3, r3, #1
	cmp	r3, #0
	beq	.L879
.L881:
	movw	r0, #65535
	b	.L880
.L879:
	ldr	r5, .L900
	sub	r3, r5, #3008
	ldrh	r0, [r3, #-4]
	add	r0, r0, r0, asl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r1, r6, #64
	cmp	r0, r1
	mov	r9, r0
	movcs	r1, #0
	movcc	r1, #1
	cmp	r6, #40
	movls	r1, #0
	cmp	r1, #0
	bne	.L881
	ldr	r3, [r5, #-3032]
	cmp	r3, #0
	beq	.L881
	ldr	r2, .L900+4
	movw	r7, #65535
	ldr	ip, .L900+12
	mov	lr, #6
	ldr	r0, [r5, #-3036]
	ldrh	r2, [r2]
	ldr	r10, [r5, #-3092]
	mov	r5, r7
	str	r2, [sp, #20]
.L882:
	ldrh	r2, [r3]
	movw	fp, #65535
	cmp	r2, fp
	beq	.L884
	add	r1, r1, #1
	ldr	fp, [sp, #20]
	uxth	r1, r1
	cmp	r1, fp
	bhi	.L881
	ldrh	fp, [r3, #4]
	cmp	fp, #0
	beq	.L883
	rsb	r3, r0, r3
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	fp, r3, asl #1
	ldrh	fp, [r10, fp]
	cmp	fp, r6
	bls	.L888
	cmp	fp, r7
	movcc	r7, fp
	movcc	r5, r3
.L883:
	mla	r3, lr, r2, r0
	b	.L882
.L888:
	mov	r5, r3
.L884:
	movw	r3, #65535
	cmp	r5, r3
	beq	.L881
	mov	r3, r5, asl #1
	ldrh	fp, [r10, r3]
	cmp	fp, r6
	bls	.L886
	str	r3, [sp, #20]
	bl	GetFreeBlockMinEraseCount
	ldr	r3, [sp, #20]
	cmp	r0, r6
	strhi	r7, [r4, #-2792]
.L886:
	cmp	fp, r8
	bcs	.L881
	add	r2, fp, #128
	cmp	r9, r2
	ble	.L881
	add	r2, fp, #256
	cmp	r2, r8
	bcc	.L887
	ldr	r2, [r4, #-2796]
	add	fp, fp, #768
	cmp	fp, r2
	bcs	.L881
.L887:
	ldr	r2, [r4, #-3028]
	mov	r1, r5
	ldr	r0, .L900+16
	ldrh	r2, [r2, r3]
	str	r2, [sp]
	mov	r2, r8
	ldrh	r3, [r10, r3]
	stmib	sp, {r3, r9}
	ldr	r3, [r4, #-2796]
	bl	printk
	mov	r0, r5
	mov	r3, #1
	str	r3, [r4, #-2192]
.L880:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L901:
	.align	2
.L900:
	.word	.LANCHOR2
	.word	.LANCHOR0+2848
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC78
	.fnend
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.align	2
	.global	free_data_superblock
	.type	free_data_superblock, %function
free_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r2, #65535
	cmp	r0, r2
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	beq	.L903
	ldr	r2, .L905
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #-3028]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L903:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L906:
	.align	2
.L905:
	.word	.LANCHOR2
	.fnend
	.size	free_data_superblock, .-free_data_superblock
	.align	2
	.global	FtlGcBufInit
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L915
	mov	r3, #0
	ldr	r1, .L915+4
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r7, ip, #78
	mov	r5, #12
	mov	r6, #1
	mov	r8, #36
	str	r3, [r1, #-2188]
.L908:
	ldrh	r2, [ip]
	add	r4, r3, #1
	uxth	r3, r3
	cmp	r3, r2
	bcs	.L913
	mul	r0, r5, r3
	ldr	lr, [r1, #-2184]
	add	r2, lr, r0
	str	r6, [r2, #8]
	ldrh	r2, [r7]
	mul	r2, r2, r3
	add	r9, r2, #3
	cmp	r2, #0
	movlt	r2, r9
	ldr	r9, [r1, #-2180]
	bic	r2, r2, #3
	add	r2, r9, r2
	str	r2, [lr, r0]
	ldr	r2, .L915+8
	ldr	r9, [r1, #-2184]
	ldrh	r2, [r2]
	add	lr, r9, r0
	mul	r2, r2, r3
	add	r10, r2, #3
	cmp	r2, #0
	movlt	r2, r10
	ldr	r10, [r1, #-2176]
	bic	r2, r2, #3
	add	r2, r10, r2
	str	r2, [lr, #4]
	ldr	r2, [r1, #-2172]
	mla	r3, r8, r3, r2
	ldr	r2, [r9, r0]
	str	r2, [r3, #8]
	ldr	r2, [lr, #4]
	str	r2, [r3, #12]
	mov	r3, r4
	b	.L908
.L913:
	ldr	r0, .L915+4
	mov	ip, #12
	ldr	r4, .L915+12
	mov	lr, #0
.L910:
	ldr	r3, [r1, #-2168]
	cmp	r2, r3
	bcs	.L914
	mul	r5, ip, r2
	ldr	r7, [r0, #-2184]
	add	r3, r7, r5
	str	lr, [r3, #8]
	ldrh	r3, [r4]
	mul	r3, r3, r2
	add	r6, r3, #3
	cmp	r3, #0
	movlt	r3, r6
	ldr	r6, [r0, #-2180]
	bic	r3, r3, #3
	add	r3, r6, r3
	str	r3, [r7, r5]
	ldr	r3, .L915+8
	ldr	r6, [r0, #-2184]
	ldrh	r3, [r3]
	add	r5, r6, r5
	mul	r3, r3, r2
	add	r2, r2, #1
	uxth	r2, r2
	add	r6, r3, #3
	cmp	r3, #0
	movlt	r3, r6
	ldr	r6, [r0, #-2176]
	bic	r3, r3, #3
	add	r3, r6, r3
	str	r3, [r5, #4]
	b	.L910
.L914:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L916:
	.align	2
.L915:
	.word	.LANCHOR0+2840
	.word	.LANCHOR2
	.word	.LANCHOR0+2920
	.word	.LANCHOR0+2918
	.fnend
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L925
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	lr, #0
	ldr	r6, [r3, #-2168]
	mov	r5, #36
	ldr	r4, [r3, #-2184]
	mov	r7, lr
	mov	r8, #12
.L918:
	uxth	ip, lr
	cmp	ip, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	mla	ip, r5, ip, r0
	mov	r2, #0
.L919:
	uxth	r3, r2
	cmp	r3, r6
	bcs	.L920
	mul	r3, r8, r3
	add	r2, r2, #1
	add	r9, r4, r3
	ldr	r10, [r4, r3]
	ldr	r3, [ip, #8]
	cmp	r10, r3
	bne	.L919
	str	r7, [r9, #8]
.L920:
	add	lr, lr, #1
	b	.L918
.L926:
	.align	2
.L925:
	.word	.LANCHOR2
	.fnend
	.size	FtlGcBufFree, .-FtlGcBufFree
	.align	2
	.global	FtlGcBufAlloc
	.type	FtlGcBufAlloc, %function
FtlGcBufAlloc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L936
	mov	ip, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r6, #12
	ldr	r4, [r3, #-2168]
	mov	r7, #1
	ldr	r5, [r3, #-2184]
	mov	r8, #36
.L928:
	uxth	r2, ip
	cmp	r2, r1
	bcs	.L935
	mov	lr, #0
.L929:
	uxth	r3, lr
	cmp	r3, r4
	bcs	.L930
	mla	r3, r6, r3, r5
	add	lr, lr, #1
	ldr	r9, [r3, #8]
	cmp	r9, #0
	bne	.L929
	mla	r2, r8, r2, r0
	ldr	lr, [r3]
	str	r7, [r3, #8]
	str	lr, [r2, #8]
	ldr	r3, [r3, #4]
	str	r3, [r2, #12]
.L930:
	add	ip, ip, #1
	b	.L928
.L935:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L937:
	.align	2
.L936:
	.word	.LANCHOR2
	.fnend
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	2
	.global	IsBlkInGcList
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L944
	sub	r2, r2, #2160
	ldr	r3, [r2, #-4]
	ldrh	r2, [r2]
	add	r2, r3, r2, asl #1
.L939:
	cmp	r3, r2
	beq	.L943
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L939
	mov	r0, #1
	bx	lr
.L943:
	mov	r0, #0
	bx	lr
.L945:
	.align	2
.L944:
	.word	.LANCHOR2
	.fnend
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	2
	.global	FtlGcUpdatePage
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r4, r0
	ubfx	r0, r0, #10, #16
	mov	r5, r1
	mov	r6, r2
	bl	P2V_block_in_plane
	ldr	r2, .L954
	mov	lr, #0
	sub	r3, r2, #2160
	ldr	r8, [r2, #-2164]
	ldrh	r1, [r3]
	sub	r7, r8, #2
.L947:
	uxth	ip, lr
	cmp	ip, r1
	bcs	.L951
	ldrh	r9, [r7, #2]!
	add	lr, lr, #1
	cmp	r9, r0
	bne	.L947
.L951:
	cmp	ip, r1
	moveq	ip, ip, asl #1
	streqh	r0, [r8, ip]	@ movhi
	ldreqh	ip, [r3]
	addeq	ip, ip, #1
	streqh	ip, [r3]	@ movhi
.L949:
	ldr	r3, .L954+4
	mov	ip, #12
	ldr	r2, [r2, #-2156]
	ldrh	r1, [r3, #-8]
	mul	ip, ip, r1
	add	r1, r2, ip
	stmib	r1, {r5, r6}
	str	r4, [r2, ip]
	ldrh	r2, [r3, #-8]
	add	r2, r2, #1
	strh	r2, [r3, #-8]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L955:
	.align	2
.L954:
	.word	.LANCHOR2
	.word	.LANCHOR2-2144
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshOpenBlock
	.type	FtlGcRefreshOpenBlock, %function
FtlGcRefreshOpenBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L964
	ldrh	r3, [r4, #-6]
	cmp	r3, r0
	beq	.L958
	ldrh	r3, [r4, #-4]
	cmp	r3, r0
	beq	.L958
	ldrh	r3, [r4, #-2]
	cmp	r3, r0
	beq	.L958
	ldrh	r3, [r4]
	cmp	r3, r0
	beq	.L958
	ldr	r0, .L964+4
	mov	r1, r5
	bl	printk
	ldrh	r2, [r4, #-6]
	movw	r3, #65535
	cmp	r2, r3
	streqh	r5, [r4, #-6]	@ movhi
	beq	.L958
	ldrh	r2, [r4, #-4]
	cmp	r2, r3
	streqh	r5, [r4, #-4]	@ movhi
	beq	.L958
	ldrh	r2, [r4, #-2]
	cmp	r2, r3
	streqh	r5, [r4, #-2]	@ movhi
	beq	.L958
	ldrh	r2, [r4]
	cmp	r2, r3
	streqh	r5, [r4]	@ movhi
.L958:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L965:
	.align	2
.L964:
	.word	.LANCHOR2-2144
	.word	.LC79
	.fnend
	.size	FtlGcRefreshOpenBlock, .-FtlGcRefreshOpenBlock
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L977
	ldrh	r3, [r4, #-6]
	cmp	r3, r0
	beq	.L974
	ldrh	r3, [r4, #-4]
	cmp	r3, r0
	beq	.L974
	ldrh	r3, [r4, #-2]
	cmp	r3, r0
	beq	.L974
	ldrh	r3, [r4]
	cmp	r3, r0
	beq	.L974
	ldr	r0, .L977+4
	mov	r1, r5
	bl	printk
	ldrh	r2, [r4, #-6]
	movw	r3, #65535
	cmp	r2, r3
	streqh	r5, [r4, #-6]	@ movhi
	beq	.L974
	ldrh	r2, [r4, #-4]
	cmp	r2, r3
	streqh	r5, [r4, #-4]	@ movhi
	beq	.L974
	ldrh	r2, [r4, #-2]
	cmp	r2, r3
	streqh	r5, [r4, #-2]	@ movhi
	beq	.L974
	ldrh	r2, [r4]
	cmp	r2, r3
	bne	.L975
	strh	r5, [r4]	@ movhi
.L974:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L975:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L978:
	.align	2
.L977:
	.word	.LANCHOR2-2144
	.word	.LC79
	.fnend
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.align	2
	.global	FtlGcMarkBadPhyBlk
	.type	FtlGcMarkBadPhyBlk, %function
FtlGcMarkBadPhyBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, r0
	bl	P2V_block_in_plane
	ldr	r6, .L989
	mov	r2, r5
	sub	r7, r6, #2128
	ldrh	r1, [r7, #-14]
	mov	r4, r0
	ldr	r0, .L989+4
	bl	printk
	mov	r0, r4
	bl	FtlGcRefreshBlock
	ldr	r3, .L989+8
	ldr	r3, [r3, #2832]
	cmp	r3, #0
	beq	.L980
	ldr	r2, [r6, #-3092]
	mov	r4, r4, asl #1
	ldrh	r3, [r2, r4]
	cmp	r3, #39
	subhi	r3, r3, #40
	strhih	r3, [r2, r4]	@ movhi
.L980:
	ldrh	r3, [r7, #-14]
	mov	r2, #0
	ldr	r0, .L989+12
.L981:
	uxth	r1, r2
	cmp	r1, r3
	bcs	.L988
	add	r2, r2, #1
	add	r1, r0, r2, asl #1
	ldrh	r1, [r1, #-2]
	cmp	r1, r5
	bne	.L981
	b	.L982
.L988:
	cmp	r3, #15
	addls	r2, r3, #1
	strlsh	r2, [r7, #-14]	@ movhi
	ldrls	r2, .L989+12
	movls	r3, r3, asl #1
	strlsh	r5, [r2, r3]	@ movhi
.L982:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L990:
	.align	2
.L989:
	.word	.LANCHOR2
	.word	.LC80
	.word	.LANCHOR0
	.word	.LANCHOR2-2140
	.fnend
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	2
	.global	FtlGcReFreshBadBlk
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, .L998
	ldrh	r3, [r4, #-14]
	cmp	r3, #0
	beq	.L992
	ldrh	r1, [r4, #-22]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L992
	add	r4, r4, #32
	ldrh	r2, [r4, #-10]
	cmp	r2, r3
	ldr	r2, .L998+4
	movcs	r3, #0
	strcsh	r3, [r4, #-10]	@ movhi
	ldrh	r3, [r4, #-10]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r4, #-10]
	add	r3, r3, #1
	strh	r3, [r4, #-10]	@ movhi
.L992:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L999:
	.align	2
.L998:
	.word	.LANCHOR2-2128
	.word	.LANCHOR2-2140
	.fnend
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	2
	.global	ftl_memset
	.type	ftl_memset, %function
ftl_memset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r2, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	beq	.L1001
	bl	memset
.L1001:
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
	.fnend
	.size	ftl_memset, .-ftl_memset
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	movw	r3, #65535
	ldr	r4, .L1111
	mvn	r2, #0
	mov	r6, #0
	ldr	r5, .L1111+4
	sub	r1, r4, #2096
	mov	r0, #1024
	str	r3, [r4, #-2092]
	sub	r3, r4, #2144
	strh	r6, [r1, #-8]	@ movhi
	mov	r9, #12
	strh	r2, [r3, #-4]	@ movhi
	movw	r8, #2840
	strh	r2, [r3]	@ movhi
	mov	fp, #36
	strh	r2, [r3, #-6]	@ movhi
	strh	r2, [r3, #-2]	@ movhi
	sub	r3, r4, #2192
	mov	r2, #32
	str	r6, [r4, #-2100]
	strh	r2, [r3, #-12]	@ movhi
	mov	r2, #128
	strh	r6, [r3, #-4]	@ movhi
	strh	r2, [r3, #-10]	@ movhi
	sub	r3, r4, #2128
	str	r6, [r4, #-2720]
	strh	r6, [r3, #-14]	@ movhi
	sub	r3, r4, #2080
	str	r6, [r4, #-2816]
	strh	r6, [r3]	@ movhi
	movw	r3, #2914
	str	r6, [r4, #-2812]
	str	r6, [r4, #-2828]
	str	r6, [r4, #-2840]
	str	r6, [r4, #-2844]
	str	r6, [r4, #-2836]
	str	r6, [r4, #-2832]
	str	r6, [r4, #-2848]
	str	r6, [r4, #-2808]
	str	r6, [r4, #-2804]
	str	r6, [r4, #-3088]
	str	r6, [r4, #-2796]
	str	r6, [r4, #-2792]
	str	r6, [r4, #-2096]
	str	r6, [r4, #-2192]
	str	r6, [r4, #-2088]
	str	r6, [r4, #-2200]
	str	r6, [r4, #-2084]
	strh	r6, [r1, #-10]	@ movhi
	ldrh	r1, [r5, r3]
	bl	__aeabi_idiv
	movw	r3, #2840
	ldrh	r3, [r5, r3]
	str	r6, [r5, #2956]
	ldr	r6, .L1111+8
	mov	r3, r3, asl #2
	mov	r10, r6
	cmp	r0, r3
	str	r0, [r4, #-2076]
	ldrh	r0, [r6]
	strhi	r3, [r4, #-2076]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-2164]
	ldrh	r0, [r10], #32
	mul	r0, r9, r0
	bl	ftl_malloc
	ldrh	r7, [r5, r8]
	mul	r7, fp, r7
	mov	r3, r7, asl #3
	str	r0, [r4, #-2156]
	mov	r0, r3
	str	r3, [sp]
	bl	ftl_malloc
	str	r0, [r4, #-2072]
	mov	r0, r7
	bl	ftl_malloc
	ldr	r3, [sp]
	str	r0, [r4, #-2068]
	mov	r0, r3
	bl	ftl_malloc
	str	r0, [r4, #-2064]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-3096]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2172]
	ldr	r0, [r4, #-2076]
	mul	r0, fp, r0
	bl	ftl_malloc
	movw	ip, #2918
	ldrh	r7, [r5, ip]
	movw	fp, #2920
	ldrh	r3, [r5, r8]
	mov	r3, r3, asl #1
	add	r3, r3, #1
	str	r3, [r4, #-2168]
	str	r0, [r5, #2960]
	mov	r0, r7
	str	ip, [sp, #4]
	bl	ftl_malloc
	str	r0, [r4, #-2060]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2056]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2052]
	ldr	r0, [r4, #-2168]
	mul	r0, r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2180]
	ldr	r0, [r4, #-2076]
	mul	r0, r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2048]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2044]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2040]
	ldr	r0, [r4, #-2168]
	mul	r0, r9, r0
	bl	ftl_malloc
	ldrh	r3, [r5, r8]
	ldrh	r7, [r5, fp]
	movw	r8, #2850
	mul	r7, r3, r7
	str	r0, [r4, #-2184]
	mov	r0, r7
	bl	ftl_malloc
	str	r0, [r4, #-2036]
	mov	r0, r7, asl #3
	bl	ftl_malloc
	ldrh	r3, [r5, fp]
	ldr	r7, .L1111+12
	str	r0, [r4, #-2032]
	ldr	r0, [r4, #-2168]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, fp]
	str	r0, [r4, #-2176]
	ldr	r0, [r4, #-2076]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #-2028]
	ldrh	r0, [r5, r8]
	mov	r0, r0, asl #1
	uxth	r0, r0
	strh	r0, [r7, #-8]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #-2020]
	ldrh	r0, [r7, #-8]
	add	r0, r0, #544
	add	r0, r0, #3
	mov	r0, r0, lsr #9
	strh	r0, [r7, #-8]	@ movhi
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldrh	r3, [r5, r8]
	mov	fp, r3, asl #1
	str	r0, [r4, #-2016]
	add	r0, r0, #32
	str	r0, [r4, #-3092]
	mov	r0, fp
	bl	ftl_malloc
	str	r0, [r4, #-2012]
	mov	r0, fp
	bl	ftl_malloc
	ldr	r3, [r5, #2936]
	mov	fp, r3, asl #1
	str	r0, [r4, #-3028]
	mov	r0, fp
	bl	ftl_malloc
	str	r0, [r4, #-2008]
	mov	r0, fp
	bl	ftl_malloc
	mov	fp, r6
	add	r6, r6, #84
	str	r0, [r4, #-2004]
	ldrh	r0, [r5, r8]
	mov	r0, r0, lsr #3
	add	r0, r0, #4
	bl	ftl_malloc
	str	r0, [r4, #-2852]
	ldrh	r0, [fp, #16]!
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r5, #2952]
	ldrh	r0, [fp]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-2000]
	ldrh	r0, [fp]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	movw	r3, #2930
	str	r0, [r4, #-1996]
	ldrh	r0, [r5, r3]
	str	r3, [sp]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	mov	r1, #0
	ldr	r3, [sp]
	str	r0, [r4, #-1992]
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r10, [r10]
	mov	r10, r10, asl #2
	mov	r0, r10
	bl	ftl_malloc
	str	r0, [r4, #-1988]
	mov	r0, r10
	bl	ftl_malloc
	movw	r10, #2946
	str	r0, [r4, #-1984]
	ldr	r0, [r5, #2936]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-1980]
	ldrh	r0, [r5, r10]
	mul	r0, r9, r0
	bl	ftl_malloc
	ldrh	r3, [r5, r10]
	ldr	ip, [sp, #4]
	str	r0, [r4, #-2864]
	ldrh	r0, [r5, ip]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, r8]
	movw	r8, #2862
	str	r0, [r4, #-1976]
	mov	r0, #6
	mul	r0, r0, r3
	bl	ftl_malloc
	movw	r3, #2906
	ldrh	r3, [r5, r3]
	add	r3, r3, #31
	mov	r3, r3, asr #5
	strh	r3, [r7, #44]	@ movhi
	str	r0, [r4, #-3036]
	ldrh	r0, [r5, r8]
	mul	r0, r0, r3
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r3, [r7, #44]
	ldrh	ip, [r5, r8]
	mov	r3, r3, asl #2
	mov	r2, r3
	str	r0, [r5, #2996]
	mov	r0, #1
.L1008:
	cmp	r0, ip
	bcs	.L1109
	ldr	r1, [r5, #2996]
	add	r0, r0, #1
	add	r1, r1, r2
	add	r2, r2, r3
	str	r1, [r6, #4]!
	b	.L1008
.L1109:
	ldr	r3, .L1111+16
	mov	r2, #0
.L1010:
	cmp	r0, #8
	addne	r1, r3, r0, asl #2
	addne	r0, r0, #1
	strne	r2, [r1, #28]
	bne	.L1010
.L1110:
	ldr	r2, [r4, #-2008]
	ldr	r3, .L1111
	cmp	r2, #0
	bne	.L1012
.L1014:
	ldr	r0, .L1111+20
	ldr	r1, .L1111+24
	bl	printk
	mvn	r0, #0
	b	.L1013
.L1012:
	ldr	r2, [r3, #-2004]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-1988]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-1980]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2864]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-1976]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-3036]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r5, #2996]
	cmp	r2, #0
	beq	.L1014
	ldr	r3, [r3, #-3028]
	cmp	r3, #0
	beq	.L1014
	ldr	r2, [r4, #-2164]
	ldr	r3, .L1111
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2156]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2072]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2064]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-3096]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2172]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2068]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2060]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2056]
	cmp	r2, #0
	beq	.L1014
	ldr	r3, [r3, #-2052]
	cmp	r3, #0
	beq	.L1014
	ldr	r2, [r4, #-2180]
	ldr	r3, .L1111
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2044]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2040]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2184]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2036]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2032]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-2176]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-3092]
	cmp	r2, #0
	beq	.L1014
	ldr	r3, [r3, #-2020]
	cmp	r3, #0
	beq	.L1014
	ldr	r3, .L1111+4
	ldr	r3, [r3, #2952]
	cmp	r3, #0
	beq	.L1014
	ldr	r3, .L1111
	ldr	r2, [r3, #-2000]
	cmp	r2, #0
	beq	.L1014
	ldr	r2, [r3, #-1996]
	cmp	r2, #0
	beq	.L1014
	ldr	r3, [r3, #-1992]
	cmp	r3, #0
	beq	.L1014
	mov	r0, #0
.L1013:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1112:
	.align	2
.L1111:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+2912
	.word	.LANCHOR2-2016
	.word	.LANCHOR0+2968
	.word	.LC81
	.word	.LANCHOR3
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, .L1119
	mov	r0, r1
	mov	r5, r1
	mov	r1, #0
	ldrh	r2, [r3, #-4]
	mov	r2, r2, asl #2
	bl	ftl_memset
	add	r3, r4, #1020
	add	r3, r3, #2
	sub	r1, r4, #2
	mov	lr, #1
	movw	r4, #65535
.L1115:
	ldrh	r2, [r1, #2]!
	cmp	r2, r4
	ldmeqfd	sp!, {r3, r4, r5, pc}
	mov	ip, r2, lsr #5
	cmp	r1, r3
	and	r2, r2, #31
	ldr	r0, [r5, ip, asl #2]
	orr	r2, r0, lr, asl r2
	str	r2, [r5, ip, asl #2]
	bne	.L1115
	ldmfd	sp!, {r3, r4, r5, pc}
.L1120:
	.align	2
.L1119:
	.word	.LANCHOR2-1968
	.fnend
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r1, #0
	ldrh	r2, [r0, #10]
	mov	r4, r0
	ldr	r5, [r0, #20]
	ldr	r7, [r0, #12]
	ldr	r6, [r0, #24]
	mov	r2, r2, asl #1
	mov	r0, r5
	bl	ftl_memset
	mov	r2, #0
.L1122:
	ldrh	r1, [r4, #6]
	uxth	r3, r2
	cmp	r1, r3
	bls	.L1142
	ldr	r0, [r6, r3, asl #2]
	mov	r1, #0
	ubfx	r0, r0, #10, #16
.L1123:
	ldrh	ip, [r4, #10]
	uxth	r3, r1
	cmp	ip, r3
	bls	.L1143
	mov	r3, r3, asl #1
	add	r1, r1, #1
	ldrh	ip, [r7, r3]
	rsb	lr, ip, r0
	cmp	ip, #0
	clz	lr, lr
	mov	lr, lr, lsr #5
	moveq	lr, #0
	cmp	lr, #0
	ldrneh	ip, [r5, r3]
	addne	ip, ip, #1
	strneh	ip, [r5, r3]	@ movhi
	b	.L1123
.L1143:
	add	r2, r2, #1
	b	.L1122
.L1142:
	ldr	r2, .L1145
	movw	r3, #2910
	mov	r8, #0
	mov	r1, r8
	mov	fp, r8
	ldrh	r2, [r2, r3]
	ldrh	r3, [r4]
	mov	r3, r3, asl #1
	strh	r2, [r5, r3]	@ movhi
	ldrh	r9, [r5]
.L1127:
	ldrh	r3, [r4, #10]
	uxth	r6, r8
	cmp	r3, r6
	bls	.L1144
	mov	r2, r6, asl #1
	ldrh	r3, [r5, r2]
	cmp	r9, r3
	bls	.L1128
	ldrh	r0, [r7, r2]
	add	r10, r7, r2
	cmp	r0, #0
	bne	.L1129
	b	.L1130
.L1128:
	cmp	r3, #0
	bne	.L1130
	ldrh	r0, [r7, r2]
	add	r10, r7, r2
	cmp	r0, #0
	movne	r6, r1
	beq	.L1130
	b	.L1132
.L1129:
	cmp	r3, #0
	movne	r1, r6
	movne	r9, r3
	bne	.L1130
	mov	r9, r3
.L1132:
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	strh	fp, [r10]	@ movhi
	ldrh	r3, [r4, #8]
	mov	r1, r6
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L1130:
	add	r8, r8, #1
	b	.L1127
.L1144:
	mov	r0, r1
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1146:
	.align	2
.L1145:
	.word	.LANCHOR0
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	FtlL2PDataInit
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r1, #0
	ldr	r5, .L1152
	mvn	r6, #0
	ldr	r4, .L1152+4
	ldr	r2, [r5, #2936]
	ldr	r0, [r4, #-2004]
	mov	r2, r2, asl #1
	bl	ftl_memset
	movw	r3, #2918
	movw	r2, #2946
	ldrh	r3, [r5, r3]
	ldrh	r2, [r5, r2]
	mov	r1, #255
	ldr	r0, [r4, #-1976]
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	ip, .L1152+8
	mov	r0, #0
	mov	r3, r4
	sub	r7, ip, #28
	mov	r4, #12
	mov	r5, r0
.L1148:
	ldrh	r2, [ip]
	add	lr, r0, #1
	uxth	r0, r0
	ldr	r1, .L1152
	cmp	r2, r0
	bls	.L1151
	mul	r1, r4, r0
	ldr	r2, [r3, #-2864]
	add	r8, r2, r1
	str	r5, [r8, #4]
	strh	r6, [r2, r1]	@ movhi
	ldr	r2, [r3, #-2864]
	add	r1, r2, r1
	ldrh	r2, [r7]
	mul	r2, r0, r2
	ldr	r0, [r3, #-1976]
	bic	r2, r2, #3
	add	r2, r0, r2
	mov	r0, lr
	str	r2, [r1, #8]
	b	.L1148
.L1151:
	ldr	r2, .L1152+12
	mvn	r0, #0
	add	r1, r1, #2944
	strh	r0, [r2, #2]	@ movhi
	strh	r0, [r2]	@ movhi
	ldr	r0, [r1, #-8]
	ldrh	r1, [r1]
	strh	r0, [r2, #10]	@ movhi
	ldr	r0, .L1152+16
	strh	r1, [r2, #6]	@ movhi
	strh	r0, [r2, #4]	@ movhi
	ldrh	r0, [r2, #44]
	strh	r0, [r2, #8]	@ movhi
	ldr	r2, [r3, #-2008]
	str	r2, [r3, #-1956]
	ldr	r2, [r3, #-1980]
	str	r2, [r3, #-1952]
	ldr	r2, [r3, #-2004]
	str	r2, [r3, #-1948]
	ldr	r2, [r3, #-1988]
	str	r2, [r3, #-1944]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1153:
	.align	2
.L1152:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2946
	.word	.LANCHOR2-1968
	.word	-3902
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	FtlVariablesInit
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mvn	r3, #0
	ldr	r5, .L1156
	mov	r4, #0
	ldr	r6, .L1156+4
	mov	r1, r4
	sub	r2, r5, #1904
	movw	r7, #2850
	str	r3, [r5, #-1904]
	strh	r3, [r2, #-12]	@ movhi
	movw	r3, #2950
	strh	r4, [r6, r3]	@ movhi
	add	r3, r6, #2928
	ldr	r0, [r6, #2952]
	ldrh	r2, [r3]
	str	r4, [r5, #-1920]
	str	r4, [r5, #-1912]
	mov	r2, r2, asl #1
	str	r4, [r5, #-1908]
	str	r4, [r6, #2832]
	bl	ftl_memset
	ldrh	r2, [r6, r7]
	mov	r1, r4
	ldr	r0, [r5, #-3092]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r6, r7]
	mov	r1, r4
	ldr	r0, [r5, #-2020]
	mov	r2, r2, asl #1
	bl	ftl_memset
	sub	r0, r5, #3072
	sub	r0, r0, #12
	mov	r1, #48
	bl	__memzero
	sub	r0, r5, #2704
	sub	r0, r0, #12
	mov	r1, #512
	bl	__memzero
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1157:
	.align	2
.L1156:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #2850
	ldr	r10, .L1170
	mov	r2, #6
	ldr	r6, .L1170+4
	.pad #28
	sub	sp, sp, #28
	mov	r1, #0
	mov	r4, #0
	ldrh	r3, [r10, r3]
	sub	r7, r6, #3008
	ldr	r0, [r6, #-3036]
	mov	r8, r4
	mov	r9, r4
	mov	r5, r6
	mul	r2, r2, r3
	bl	ftl_memset
	sub	r3, r6, #2096
	mov	fp, r6
	str	r4, [r6, #-3016]
	str	r4, [r6, #-3032]
	str	r4, [r6, #-3024]
	strh	r4, [r7, #-12]	@ movhi
	strh	r4, [r7, #-4]	@ movhi
	strh	r4, [r3, #-8]	@ movhi
	str	r10, [sp, #8]
.L1159:
	ldr	r2, .L1170+8
	uxth	r3, r4
	sxth	r10, r3
	ldrh	r1, [r2]
	cmp	r10, r1
	bge	.L1166
	sub	r2, r2, #8
	ldr	r1, .L1170+12
	mov	ip, #0
	str	r3, [sp, #12]
	ldrh	r2, [r2]
	mov	r6, ip
	str	r2, [sp]
	ldrh	r2, [r1]
	str	r2, [sp, #4]
.L1167:
	ldr	r2, [sp]
	sxth	r1, ip
	cmp	r1, r2
	bge	.L1169
	str	r3, [sp, #16]
	ldr	r3, [sp, #8]
	str	ip, [sp, #20]
	add	r1, r3, r1
	ldrb	r0, [r1, #2868]	@ zero_extendqisi2
	ldr	r1, [sp, #12]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldr	ip, [sp, #20]
	ldr	r3, [sp, #16]
	ldreq	r2, [sp, #4]
	add	ip, ip, #1
	addeq	r6, r2, r6
	uxtheq	r6, r6
	b	.L1167
.L1169:
	cmp	r6, #0
	beq	.L1162
	sxth	r1, r6
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r6, r0
	b	.L1163
.L1162:
	sxth	r3, r3
	ldr	r1, [r5, #-3028]
	mvn	r0, #0
	mov	r3, r3, asl #1
	strh	r0, [r1, r3]	@ movhi
.L1163:
	mov	r1, r10, asl #1
	ldr	r3, [r5, #-3036]
	add	r0, r1, r10
	add	r3, r3, r0, asl #1
	strh	r6, [r3, #4]	@ movhi
	ldrh	r3, [r7]
	cmp	r10, r3
	beq	.L1164
	ldr	r3, .L1170+16
	ldrh	r0, [r3]
	cmp	r10, r0
	beq	.L1164
	ldrh	r3, [r3, #48]
	cmp	r10, r3
	beq	.L1164
	ldr	r3, [fp, #-3028]
	uxth	r0, r4
	ldrh	r3, [r3, r1]
	cmp	r3, #0
	bne	.L1165
	add	r8, r8, #1
	uxth	r8, r8
	bl	INSERT_FREE_LIST
	b	.L1164
.L1165:
	add	r9, r9, #1
	uxth	r9, r9
	bl	INSERT_DATA_LIST
.L1164:
	add	r4, r4, #1
	b	.L1159
.L1166:
	mov	r0, #0
	strh	r9, [r7, #-12]	@ movhi
	strh	r8, [r7, #-4]	@ movhi
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1171:
	.align	2
.L1170:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2848
	.word	.LANCHOR0+2908
	.word	.LANCHOR2-2960
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r3, #0
	ldr	r4, .L1174
	mov	r1, #255
	ldr	r5, .L1174+4
	sub	r2, r4, #2160
	ldr	r0, [r4, #-2164]
	strh	r3, [r2]	@ movhi
	sub	r2, r4, #2144
	strh	r3, [r2, #-8]	@ movhi
	ldrh	r2, [r5]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r5]
	mov	r2, #12
	ldr	r0, [r4, #-2156]
	mov	r1, #255
	mul	r2, r2, r3
	bl	ftl_memset
	ldmfd	sp!, {r3, r4, r5, lr}
	b	FtlGcBufInit
.L1175:
	.align	2
.L1174:
	.word	.LANCHOR2
	.word	.LANCHOR0+2912
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	ftl_memcpy
	.type	ftl_memcpy, %function
ftl_memcpy:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	memcpy
	.fnend
	.size	ftl_memcpy, .-ftl_memcpy
	.align	2
	.global	FlashReadIdbData
	.type	FlashReadIdbData, %function
FlashReadIdbData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	mov	r2, #2048
	ldr	r1, .L1179
	bl	ftl_memcpy
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1180:
	.align	2
.L1179:
	.word	.LANCHOR2-1900
	.fnend
	.size	FlashReadIdbData, .-FlashReadIdbData
	.align	2
	.global	FlashLoadPhyInfoInRam
	.type	FlashLoadPhyInfoInRam, %function
FlashLoadPhyInfoInRam:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r4, #0
	ldr	r8, .L1194
.L1185:
	mov	r7, r4, asl #5
	ldr	r1, .L1194+4
	add	r0, r7, #1
	ldrb	r2, [r8, r4, asl #5]	@ zero_extendqisi2
	add	r0, r8, r0
	ldr	r6, .L1194+8
	bl	FlashMemCmp8
	add	r9, r6, #508
	subs	r5, r0, #0
	bne	.L1182
	adds	r9, r9, r7
	beq	.L1188
	add	r4, r6, r7
	mov	r3, r5
	ldr	r1, .L1194+12
	ldrb	r2, [r4, #530]	@ zero_extendqisi2
	b	.L1187
.L1182:
	add	r4, r4, #1
	cmp	r4, #81
	bne	.L1185
	b	.L1188
.L1193:
	add	r3, r3, #1
	cmp	r3, #4
	beq	.L1186
.L1187:
	ldrb	r0, [r1, r3, asl #5]	@ zero_extendqisi2
	cmp	r0, r2
	bne	.L1193
.L1186:
	ldr	r4, .L1194+16
	mov	r2, #32
	ldr	r1, .L1194+12
	ldr	r0, .L1194+20
	add	r1, r1, r3, asl #5
	bl	ftl_memcpy
	mov	r0, r4
	mov	r1, r9
	mov	r2, #32
	bl	ftl_memcpy
	ldrh	r0, [r4, #10]
	bl	FlashBlockAlignInit
	b	.L1183
.L1188:
	mvn	r5, #0
.L1183:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L1195:
	.align	2
.L1194:
	.word	.LANCHOR1+508
	.word	.LANCHOR0+1788
	.word	.LANCHOR1
	.word	.LANCHOR1+3100
	.word	.LANCHOR1+256
	.word	.LANCHOR0+44
	.fnend
	.size	FlashLoadPhyInfoInRam, .-FlashLoadPhyInfoInRam
	.align	2
	.global	NandcCopy1KB
	.type	NandcCopy1KB, %function
NandcCopy1KB:
	.fnstart
	@ args = 4, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #1
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r2
	add	r2, r0, #4096
	ldr	r5, [sp, #16]
	add	r6, r0, #512
	add	r2, r2, r4, asl #9
	bne	.L1197
	cmp	r3, #0
	beq	.L1198
	mov	r0, r2
	mov	r1, r3
	mov	r2, #1024
	bl	ftl_memcpy
.L1198:
	cmp	r5, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldrb	r3, [r5, #2]	@ zero_extendqisi2
	mov	r4, r4, lsr #1
	ldrb	r2, [r5, #1]	@ zero_extendqisi2
	add	r4, r4, r4, asl #1
	mov	r3, r3, asl #16
	orr	r2, r3, r2, asl #8
	ldrb	r3, [r5]	@ zero_extendqisi2
	orr	r3, r2, r3
	ldrb	r2, [r5, #3]	@ zero_extendqisi2
	orr	r3, r3, r2, asl #24
	str	r3, [r6, r4, asl #4]
	ldmfd	sp!, {r4, r5, r6, pc}
.L1197:
	cmp	r3, #0
	beq	.L1201
	mov	r1, r2
	mov	r0, r3
	mov	r2, #1024
	bl	ftl_memcpy
.L1201:
	cmp	r5, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	mov	r4, r4, lsr #1
	add	r4, r4, r4, asl #1
	ldr	r3, [r6, r4, asl #4]
	mov	r2, r3, lsr #8
	strb	r3, [r5]
	strb	r2, [r5, #1]
	mov	r2, r3, lsr #16
	mov	r3, r3, lsr #24
	strb	r2, [r5, #2]
	strb	r3, [r5, #3]
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	NandcCopy1KB, .-NandcCopy1KB
	.align	2
	.global	ftl_memcpy32
	.type	ftl_memcpy32, %function
ftl_memcpy32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, #0
.L1214:
	cmp	r3, r2
	ldrne	ip, [r1, r3, asl #2]
	strne	ip, [r0, r3, asl #2]
	addne	r3, r3, #1
	bne	.L1214
.L1216:
	bx	lr
	.fnend
	.size	ftl_memcpy32, .-ftl_memcpy32
	.align	2
	.global	ftl_memcmp
	.type	ftl_memcmp, %function
ftl_memcmp:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	memcmp
	.fnend
	.size	ftl_memcmp, .-ftl_memcmp
	.align	2
	.global	timer_get_time
	.type	timer_get_time, %function
timer_get_time:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1219
	ldr	r0, [r3]
	b	jiffies_to_msecs
.L1220:
	.align	2
.L1219:
	.word	jiffies
	.fnend
	.size	timer_get_time, .-timer_get_time
	.align	2
	.global	FlashSramLoadStore
	.type	FlashSramLoadStore, %function
FlashSramLoadStore:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L1225
	cmp	r2, #0
	moveq	r2, r3
	ldr	ip, [ip, #148]
	add	ip, ip, #4096
	add	r1, ip, r1
	beq	.L1224
	str	lr, [sp, #-4]!
	.save {lr}
	mov	lr, r0
	mov	r0, r1
	mov	r1, lr
	ldr	lr, [sp], #4
	mov	r2, r3
.L1224:
	b	ftl_memcpy
.L1226:
	.align	2
.L1225:
	.word	.LANCHOR2
	.fnend
	.size	FlashSramLoadStore, .-FlashSramLoadStore
	.align	2
	.global	FlashCs123Init
	.type	FlashCs123Init, %function
FlashCs123Init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	FlashCs123Init, .-FlashCs123Init
	.align	2
	.global	rk_nand_suspend
	.type	rk_nand_suspend, %function
rk_nand_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_flash_suspend
	.fnend
	.size	rk_nand_suspend, .-rk_nand_suspend
	.align	2
	.global	rk_nand_resume
	.type	rk_nand_resume, %function
rk_nand_resume:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_flash_resume
	.fnend
	.size	rk_nand_resume, .-rk_nand_resume
	.align	2
	.global	rk_ftl_get_capacity
	.type	rk_ftl_get_capacity, %function
rk_ftl_get_capacity:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1231
	ldr	r0, [r3, #1892]
	bx	lr
.L1232:
	.align	2
.L1231:
	.word	.LANCHOR0
	.fnend
	.size	rk_ftl_get_capacity, .-rk_ftl_get_capacity
	.align	2
	.global	rk_nandc_get_irq_status
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
	.align	2
	.global	rknand_proc_ftlread
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_proc_ftl_read
	.fnend
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	2
	.global	ReadFlashInfo
	.type	ReadFlashInfo, %function
ReadFlashInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_read_flash_info
	.fnend
	.size	ReadFlashInfo, .-ReadFlashInfo
	.align	2
	.global	rknand_print_hex
	.type	rknand_print_hex, %function
rknand_print_hex:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, #0
	mov	r9, r0
	mov	r6, r1
	mov	r7, r2
	mov	r8, r3
	mov	r4, r5
.L1237:
	cmp	r4, r8
	beq	.L1246
	cmp	r5, #0
	bne	.L1238
	ldr	r0, .L1247
	mov	r1, r9
	mov	r2, r6
	mov	r3, r4
	bl	printk
.L1238:
	cmp	r7, #4
	ldreq	r0, .L1247+4
	ldreq	r1, [r6, r4, asl #2]
	beq	.L1245
	cmp	r7, #2
	moveq	r3, r4, asl #1
	ldreq	r0, .L1247+4
	ldreqsh	r1, [r6, r3]
	ldrne	r0, .L1247+4
	ldrneb	r1, [r6, r4]	@ zero_extendqisi2
.L1245:
	add	r5, r5, #1
	bl	printk
	cmp	r5, #15
	bls	.L1242
	ldr	r0, .L1247+8
	mov	r5, #0
	ldr	r1, .L1247+12
	bl	printk
.L1242:
	add	r4, r4, #1
	b	.L1237
.L1246:
	ldr	r0, .L1247+8
	ldr	r1, .L1247+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L1248:
	.align	2
.L1247:
	.word	.LC82
	.word	.LC83
	.word	.LC76
	.word	.LC84
	.fnend
	.size	rknand_print_hex, .-rknand_print_hex
	.align	2
	.global	HynixGetReadRetryDefault
	.type	HynixGetReadRetryDefault, %function
HynixGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	cmp	r0, #2
	ldr	r7, .L1358
	mvn	r3, #83
	mvn	r1, #82
	mvn	r2, #81
	.pad #44
	sub	sp, sp, #44
	mov	r4, r0
	strb	r3, [r7, #1906]
	mvn	r3, #80
	strb	r0, [r7, #1902]
	strb	r1, [r7, #1907]
	strb	r2, [r7, #1908]
	strb	r3, [r7, #1909]
	bne	.L1250
	mvn	r3, #88
	strb	r3, [r7, #1906]
	ldr	r3, .L1358+4
	mvn	r2, #8
	mov	r5, #7
	strb	r2, [r3, #3245]
	b	.L1311
.L1250:
	cmp	r0, #3
	bne	.L1252
	mvn	r3, #79
	strb	r3, [r7, #1906]
	mvn	r3, #78
	strb	r3, [r7, #1907]
	mvn	r3, #77
	strb	r3, [r7, #1908]
	mvn	r3, #76
	strb	r3, [r7, #1909]
	mvn	r3, #75
	strb	r3, [r7, #1910]
	mvn	r3, #74
	strb	r3, [r7, #1911]
	mvn	r3, #73
	strb	r3, [r7, #1912]
	mvn	r3, #72
	b	.L1352
.L1252:
	cmp	r0, #4
	bne	.L1253
	mvn	r0, #51
	strb	r1, [r7, #1911]
	strb	r0, [r7, #1906]
	mvn	r0, #64
	strb	r2, [r7, #1912]
	strb	r0, [r7, #1907]
	mvn	r0, #85
	strb	r0, [r7, #1908]
	mvn	r0, #84
	strb	r0, [r7, #1909]
	mvn	r0, #50
	strb	r0, [r7, #1910]
.L1352:
	mov	r5, #8
	strb	r3, [r7, #1913]
	mov	r6, r5
	b	.L1251
.L1253:
	cmp	r0, #5
	bne	.L1254
	mov	r3, #56
	mov	r5, #8
	strb	r3, [r7, #1906]
	mov	r3, #57
	strb	r3, [r7, #1907]
	mov	r3, #58
	strb	r3, [r7, #1908]
	mov	r3, #59
	strb	r3, [r7, #1909]
	b	.L1311
.L1254:
	cmp	r0, #6
	bne	.L1255
	mov	r3, #14
	mov	r5, #12
	strb	r3, [r7, #1906]
	mov	r3, #15
	strb	r3, [r7, #1907]
	mov	r3, #16
	strb	r3, [r7, #1908]
	mov	r3, #17
	strb	r3, [r7, #1909]
	b	.L1311
.L1255:
	cmp	r0, #7
	bne	.L1256
	mvn	r3, #79
	mov	r5, #12
	strb	r3, [r7, #1906]
	mov	r6, #10
	mvn	r3, #78
	strb	r3, [r7, #1907]
	mvn	r3, #77
	strb	r3, [r7, #1908]
	mvn	r3, #76
	strb	r3, [r7, #1909]
	mvn	r3, #75
	strb	r3, [r7, #1910]
	mvn	r3, #74
	strb	r3, [r7, #1911]
	mvn	r3, #73
	strb	r3, [r7, #1912]
	mvn	r3, #72
	strb	r3, [r7, #1913]
	mvn	r3, #43
	strb	r3, [r7, #1914]
	mvn	r3, #42
	strb	r3, [r7, #1915]
	b	.L1251
.L1256:
	cmp	r0, #8
	mov	r5, #7
	bne	.L1311
	mov	r3, #6
	strb	r5, [r7, #1907]
	strb	r3, [r7, #1906]
	mov	r5, #50
	mov	r3, #9
	strb	r0, [r7, #1908]
	strb	r3, [r7, #1909]
	mov	r6, #5
	mov	r3, #10
	strb	r3, [r7, #1910]
	b	.L1251
.L1311:
	mov	r6, #4
.L1251:
	sub	r3, r4, #1
	cmp	r3, #1
	bhi	.L1348
	ldr	r10, .L1358+8
	mov	r9, #0
.L1257:
	ldrb	r1, [r7, #1612]	@ zero_extendqisi2
	uxtb	r2, r9
	ldr	r3, .L1358
	cmp	r1, r2
	bls	.L1264
	add	r2, r3, r2
	ldr	r4, .L1358+12
	ldr	r1, .L1358+16
	mov	ip, #55
	ldrb	r2, [r2, #1616]	@ zero_extendqisi2
	add	r3, r3, r2, asl #3
	add	r4, r4, r2, asl #6
	add	r4, r4, #20
	ldrb	r8, [r3, #16]	@ zero_extendqisi2
	ldr	r0, [r3, #12]
	mov	fp, r4
	mov	r2, r4
	add	r8, r0, r8, asl #8
.L1259:
	str	ip, [r8, #2056]
	mov	r0, #80
	ldrb	r3, [r1, #1]!	@ zero_extendqisi2
	str	r2, [sp, #12]
	str	ip, [sp, #8]
	str	r3, [r8, #2052]
	str	r1, [sp, #4]
	str	r1, [sp, #16]
	bl	NandcDelayns
	ldr	r3, [r8, #2048]
	ldr	r2, [sp, #12]
	ldmib	sp, {r1, ip}
	strb	r3, [r2], #1
	rsb	r3, r4, r2
	uxtb	r3, r3
	cmp	r3, r6
	bcc	.L1259
	mov	r2, #0
.L1260:
	add	r0, r10, r2
	mov	r3, #1
.L1261:
	ldrb	r1, [r0, r3, asl #2]	@ zero_extendqisi2
	ldrb	ip, [fp]	@ zero_extendqisi2
	add	r1, r1, ip
	strb	r1, [fp, r3, asl #3]
	add	r3, r3, #1
	cmp	r3, #7
	bne	.L1261
	add	r2, r2, #1
	add	fp, fp, #1
	cmp	r2, #4
	bne	.L1260
	mov	r3, #0
	add	r9, r9, #1
	strb	r3, [r4, #16]
	strb	r3, [r4, #24]
	strb	r3, [r4, #32]
	strb	r3, [r4, #40]
	strb	r3, [r4, #48]
	strb	r3, [r4, #41]
	strb	r3, [r4, #49]
	b	.L1257
.L1348:
	sub	r3, r4, #3
	cmp	r3, #5
	bhi	.L1264
	smulbb	r2, r6, r5
	mov	r3, r2, asl #4
	mov	r2, r2, asr #1
	str	r3, [sp, #36]
	mov	r3, r2, asl #1
	str	r3, [sp, #8]
	mov	r3, #0
.L1356:
	str	r3, [sp, #16]
	ldrb	r3, [sp, #16]	@ zero_extendqisi2
	str	r3, [sp, #12]
	ldr	r3, .L1358
	ldr	r2, [sp, #12]
	ldrb	r3, [r3, #1612]	@ zero_extendqisi2
	cmp	r3, r2
	bhi	.L1309
.L1264:
	ldr	r3, .L1358
	strb	r6, [r3, #1903]
	strb	r5, [r3, #1904]
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1309:
	ldr	r2, [sp, #12]
	ldr	r3, .L1358
	add	r3, r3, r2
	ldrb	r10, [r3, #1616]	@ zero_extendqisi2
	ldr	r3, .L1358
	add	r3, r3, r10, asl #3
	mov	r0, r10
	ldrb	fp, [r3, #16]	@ zero_extendqisi2
	ldr	r8, [r3, #12]
	mov	r3, #255
	mov	r9, fp, asl #8
	add	r7, r8, r9
	str	r3, [r7, #2056]
	bl	NandcWaitFlashReady
	cmp	r4, #7
	ldreq	r3, .L1358+12
	moveq	r7, #160
	mlaeq	r7, r7, r10, r3
	addeq	r3, r7, #28
	beq	.L1353
.L1266:
	cmp	r4, #8
	beq	.L1268
	ldr	r3, .L1358+12
	add	r7, r3, r10, asl #6
	add	r3, r7, #20
.L1353:
	str	r3, [sp, #20]
	cmp	r4, #4
	add	r3, r8, fp, asl #8
	mov	r2, #54
	str	r2, [r3, #2056]
	bne	.L1269
	add	r9, r8, r9
	mov	r3, #255
	str	r3, [r9, #2052]
	mov	r3, #64
	str	r3, [r9, #2048]
	mov	r3, #204
	b	.L1354
.L1269:
	sub	r2, r4, #5
	cmp	r2, #1
	bhi	.L1271
	ldr	r2, .L1358
	ldrb	r2, [r2, #1906]	@ zero_extendqisi2
	str	r2, [r3, #2052]
	mov	r2, #82
	str	r2, [r3, #2048]
	b	.L1270
.L1271:
	cmp	r4, #7
	bne	.L1270
	add	r9, r8, r9
	mov	r3, #174
	str	r3, [r9, #2052]
	mov	r3, #0
	str	r3, [r9, #2048]
	mov	r3, #176
.L1354:
	str	r3, [r9, #2052]
	mov	r3, #77
	str	r3, [r9, #2048]
.L1270:
	add	r3, r8, fp, asl #8
	cmp	r4, #6
	mov	r2, #22
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
	moveq	r2, #31
	str	r2, [r3, #2052]
	mov	r2, #2
	str	r2, [r3, #2052]
	mov	r2, #0
	str	r2, [r3, #2052]
.L1310:
	add	r3, r8, fp, asl #8
	mov	r2, #48
	mov	r0, r10
	str	r2, [r3, #2056]
	bl	NandcWaitFlashReady
	sub	r3, r4, #5
	cmp	r3, #1
	movhi	r3, #0
	movls	r3, #1
	str	r3, [sp, #24]
	sub	r3, r4, #8
	clz	r3, r3
	mov	r3, r3, lsr #5
	str	r3, [sp, #4]
	ldr	r2, [sp, #4]
	ldr	r3, [sp, #24]
	orrs	r3, r3, r2
	movne	ip, #16
	bne	.L1274
	cmp	r4, #7
	movne	ip, #2
	moveq	ip, #32
.L1274:
	ldr	r3, .L1358+20
	add	r0, r8, fp, asl #8
	ldr	r3, [r3, #152]
	mov	r1, r3
.L1275:
	ldr	r2, [r0, #2048]
	strb	r2, [r1], #1
	rsb	r2, r3, r1
	uxtb	r2, r2
	cmp	r2, ip
	bcc	.L1275
	ldr	r2, [sp, #4]
	cmp	r2, #0
	beq	.L1276
	mov	r2, #0
.L1278:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r1, r2
	cmp	ip, #50
	beq	.L1277
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #5
	beq	.L1277
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L1278
	b	.L1279
.L1277:
	cmp	r1, #6
	bls	.L1280
.L1279:
	ldr	r0, .L1358+24
	mov	r1, #0
	bl	printk
.L1281:
	b	.L1281
.L1276:
	cmp	r4, #7
	bne	.L1282
	ldr	r2, [sp, #4]
.L1284:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r1, r2
	cmp	ip, #12
	beq	.L1283
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #10
	beq	.L1283
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L1284
	b	.L1285
.L1283:
	cmp	r1, #7
	bne	.L1280
.L1285:
	ldr	r0, .L1358+24
	mov	r1, #0
	bl	printk
.L1286:
	b	.L1286
.L1282:
	cmp	r4, #6
	bne	.L1280
	sub	r2, r3, #1
	add	r3, r3, #7
.L1287:
	ldrb	r1, [r2, #1]!	@ zero_extendqisi2
	cmp	r1, #12
	beq	.L1280
	ldrb	r1, [r2, #8]	@ zero_extendqisi2
	cmp	r1, #4
	beq	.L1280
	cmp	r2, r3
	bne	.L1287
	ldr	r0, .L1358+24
	mov	r1, #0
	bl	printk
.L1289:
	b	.L1289
.L1280:
	ldr	r1, .L1358+20
	ldr	ip, [sp, #36]
	ldr	r2, [r1, #152]
	add	ip, r2, ip
	mov	r3, r2
.L1290:
	cmp	r3, ip
	ldrne	lr, [r0, #2048]
	strneb	lr, [r3], #1
	bne	.L1290
.L1357:
	ldr	r3, [r1, #152]
	mov	r0, #8
	ldr	r1, [sp, #8]
	add	r1, r3, r1
	str	r1, [sp, #28]
.L1293:
	ldr	ip, [sp, #8]
	add	lr, r1, ip
.L1292:
	ldrh	ip, [r1]
	mvn	ip, ip
	strh	ip, [r1], #2	@ movhi
	cmp	r1, lr
	bne	.L1292
	ldr	ip, [sp, #8]
	subs	r0, r0, #1
	add	r1, r1, ip
	bne	.L1293
.L1294:
	mov	ip, #0
	mov	r0, ip
.L1297:
	mov	r1, #1
	mov	lr, #0
	mov	r1, r1, asl r0
	mov	r7, #16
	str	r7, [sp, #32]
	mov	r7, lr
.L1295:
	ldrh	r9, [r3, lr]
	and	r9, r9, r1
	cmp	r9, r1
	addeq	r7, r7, #1
	ldr	r9, [sp, #8]
	add	lr, lr, r9
	ldr	r9, [sp, #32]
	subs	r9, r9, #1
	str	r9, [sp, #32]
	bne	.L1295
	cmp	r7, #8
	add	r0, r0, #1
	orrhi	ip, ip, r1
	uxthhi	ip, ip
	cmp	r0, #16
	bne	.L1297
	ldr	r1, [sp, #28]
	strh	ip, [r3], #2	@ movhi
	cmp	r3, r1
	bne	.L1294
	ldr	r3, .L1358+20
	ldr	r1, [r3, #152]
	mov	r3, #0
	sub	r0, r1, #4
	add	ip, r1, #28
.L1299:
	ldr	lr, [r0, #4]!
	cmp	lr, #0
	addeq	r3, r3, #1
	cmp	r0, ip
	bne	.L1299
	cmp	r3, #7
	ble	.L1300
	ldr	r0, .L1358+28
	mov	r2, #1
	mov	r3, #1024
	bl	rknand_print_hex
	ldr	r0, .L1358+24
	mov	r1, #0
	bl	printk
.L1301:
	b	.L1301
.L1300:
	cmp	r4, #6
	moveq	r0, #4
	beq	.L1302
	cmp	r4, #7
	moveq	r0, #10
	beq	.L1302
	ldr	r3, [sp, #4]
	cmp	r3, #0
	moveq	r0, #8
	movne	r0, #5
.L1302:
	sub	r9, r6, #1
	ldr	r1, [sp, #20]
	mov	ip, #0
	uxtb	r9, r9
	add	r9, r9, #1
.L1303:
	mov	r7, r1
	mov	r3, r2
.L1304:
	ldrb	lr, [r3], #1	@ zero_extendqisi2
	strb	lr, [r7], #1
	rsb	lr, r2, r3
	uxtb	lr, lr
	cmp	lr, r6
	bcc	.L1304
	add	ip, ip, #1
	add	r2, r2, r9
	cmp	ip, r5
	add	r1, r1, r0
	blt	.L1303
	add	r7, r8, fp, asl #8
	mov	r3, #255
	mov	r0, r10
	str	r3, [r7, #2056]
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #24]
	cmp	r3, #0
	beq	.L1306
	mov	r3, #54
	str	r3, [r7, #2056]
	ldr	r3, .L1358
	mvn	r1, #0
	ldr	r0, [sp, #12]
	ldrb	r3, [r3, #1906]	@ zero_extendqisi2
	str	r3, [r7, #2052]
	mov	r3, #0
	str	r3, [r7, #2048]
	mov	r3, #22
	str	r3, [r7, #2056]
	bl	FlashReadCmd
	b	.L1307
.L1306:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	movne	r3, #190
	moveq	r3, #56
	str	r3, [r7, #2056]
.L1307:
	mov	r0, r10
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #16]
	add	r3, r3, #1
	b	.L1356
.L1268:
	mov	r3, #120
	mov	r2, #23
	str	r3, [r7, #2056]
	mov	r1, #25
	mov	r3, #0
	str	r3, [r7, #2052]
	str	r3, [r7, #2052]
	str	r3, [r7, #2052]
	str	r2, [r7, #2056]
	mov	r2, #4
	str	r2, [r7, #2056]
	str	r1, [r7, #2056]
	mov	r1, #218
	str	r1, [r7, #2056]
	mov	r1, #21
	str	r3, [r7, #2056]
	str	r3, [r7, #2052]
	str	r3, [r7, #2052]
	str	r1, [r7, #2052]
	str	r2, [r7, #2052]
	str	r3, [r7, #2052]
	ldr	r3, .L1358+32
	str	r3, [sp, #20]
	b	.L1310
.L1359:
	.align	2
.L1358:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR1+3228
	.word	.LANCHOR0+1902
	.word	.LANCHOR0+1905
	.word	.LANCHOR2
	.word	.LC85
	.word	.LC86
	.word	.LANCHOR0+1930
	.fnend
	.size	HynixGetReadRetryDefault, .-HynixGetReadRetryDefault
	.align	2
	.global	FlashGetReadRetryDefault
	.type	FlashGetReadRetryDefault, %function
FlashGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	subs	r3, r0, #0
	bxeq	lr
	sub	r2, r3, #1
	cmp	r2, #7
	bhi	.L1362
	b	HynixGetReadRetryDefault
.L1362:
	cmp	r3, #49
	bne	.L1363
	ldr	r2, .L1374
	ldr	r0, .L1374+4
	ldr	r1, .L1374+8
	strb	r3, [r2, #1902]
	mov	r3, #4
	strb	r3, [r2, #1903]
	mov	r3, #15
	strb	r3, [r2, #1904]
	mov	r2, #64
	b	.L1372
.L1363:
	sub	r2, r3, #65
	cmp	r3, #33
	cmpne	r2, #1
	ldrls	r2, .L1374
	strlsb	r3, [r2, #1902]
	movls	r3, #4
	bls	.L1373
.L1364:
	cmp	r3, #34
	cmpne	r3, #67
	bne	.L1365
	ldr	r2, .L1374
	strb	r3, [r2, #1902]
	mov	r3, #5
.L1373:
	strb	r3, [r2, #1903]
	mov	r3, #7
	ldr	r0, .L1374+4
	strb	r3, [r2, #1904]
	mov	r2, #45
	ldr	r1, .L1374+12
	b	.L1372
.L1365:
	cmp	r3, #35
	cmpne	r3, #68
	bxne	lr
	ldr	r2, .L1374
	ldr	r0, .L1374+4
	ldr	r1, .L1374+16
	strb	r3, [r2, #1902]
	mov	r3, #5
	strb	r3, [r2, #1903]
	mov	r3, #17
	strb	r3, [r2, #1904]
	mov	r2, #95
.L1372:
	b	ftl_memcpy
.L1375:
	.align	2
.L1374:
	.word	.LANCHOR0
	.word	.LANCHOR0+1906
	.word	.LANCHOR1+440
	.word	.LANCHOR1+288
	.word	.LANCHOR1+336
	.fnend
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
	.align	2
	.global	NandcXferComp
	.type	NandcXferComp, %function
NandcXferComp:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #8
	ldr	r5, .L1416
	add	r0, r5, r0, asl #3
	ldr	r3, [r5, #2776]
	ldr	r4, [r0, #12]
	cmp	r3, #3
	bls	.L1407
	ldr	r3, [r4, #16]
	tst	r3, #4
	beq	.L1407
	mov	r0, r4
	bl	wait_for_nandc_xfer_completed
	ldr	r6, [r4, #16]
	ldr	r3, [r4, #8]
	ubfx	r6, r6, #1, #1
	cmp	r6, #0
	movne	r6, #0
	str	r3, [sp]
	beq	.L1389
.L1379:
	ldr	r2, [r4, #28]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	bge	.L1387
	ldr	r3, [r5, #2776]
	cmp	r3, #5
	bhi	.L1380
.L1383:
	add	r6, r6, #1
	bic	r3, r6, #-16777216
	cmp	r3, #0
	bne	.L1379
	ldr	r2, [r4, #28]
	mov	r1, r6
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L1416+4
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L1416+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
	b	.L1379
.L1380:
	ldr	r3, [r4]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L1383
	ldr	r3, [sp, #4]
	tst	r3, #131072
	beq	.L1383
.L1387:
	ldr	r3, [r5, #2812]
	ldr	r4, .L1416
	cmp	r3, #0
	beq	.L1388
	ldr	r1, [sp]
	mov	r2, #0
	ldr	r0, [r4, #2804]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r4, #2808]
	mov	r2, #0
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
	b	.L1388
.L1391:
	ldr	r3, [r4, #8]
	add	r6, r6, #1
	str	r3, [sp]
	bic	r3, r6, #-16777216
	cmp	r3, #0
	bne	.L1389
	ldr	r2, [sp]
	mov	r1, r6
	ldr	r3, [r4, #28]
	ldr	r0, .L1416+12
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L1416+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L1389:
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L1391
	ldr	r3, [r5, #2820]
	cmp	r3, #0
	beq	.L1392
	mov	r0, r4
	bl	NandcSendDumpDataStart
.L1392:
	ldr	r3, [r5, #2812]
	ldr	r6, .L1416
	cmp	r3, #0
	beq	.L1393
	ldr	r1, [sp]
	mov	r2, #1
	ldr	r0, [r6, #2804]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r6, #2808]
	mov	r2, #1
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
.L1393:
	ldr	r3, [r5, #2820]
	cmp	r3, #0
	beq	.L1388
	mov	r0, r4
	bl	NandcSendDumpDataDone
.L1388:
	mov	r3, #0
	str	r3, [r5, #2812]
	b	.L1376
.L1407:
	ldr	r3, [r4, #8]
	str	r3, [sp]
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L1407
.L1376:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L1417:
	.align	2
.L1416:
	.word	.LANCHOR0
	.word	.LC87
	.word	.LC88
	.word	.LC89
	.fnend
	.size	NandcXferComp, .-NandcXferComp
	.align	2
	.global	NandcXferData
	.type	NandcXferData, %function
NandcXferData:
	.fnstart
	@ args = 4, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r3
	ldr	r6, .L1466
	tst	r10, #63
	.pad #92
	sub	sp, sp, #92
	mov	r9, r0
	add	r3, r6, r0, asl #3
	mov	r5, r1
	mov	r7, r2
	ldr	r8, [r3, #12]
	bne	.L1419
	ldr	r3, [sp, #128]
	cmp	r3, #0
	bne	.L1420
	add	r0, sp, #24
	mov	r1, #255
	mov	r2, #64
	bl	memset
	add	r3, sp, #24
	str	r3, [sp, #128]
.L1420:
	ldr	r3, [sp, #128]
	mov	r0, r9
	mov	r1, r5
	mov	r2, r7
	str	r10, [sp]
	str	r3, [sp, #4]
	mov	r3, #0
	bl	NandcXferStart
	mov	r0, r9
	bl	NandcXferComp
	cmp	r5, #0
	movne	fp, #0
	bne	.L1421
	ldr	r3, [r6, #2824]
	mov	r0, r7, lsr #1
	mov	r2, r5
	mov	r1, r5
	cmp	r3, #25
	ldr	r3, [sp, #128]
	movcc	lr, #64
	movcs	lr, #128
.L1423:
	cmp	r1, r0
	add	r3, r3, #4
	add	ip, r2, lr
	bcs	.L1464
	ldr	r4, [r6, #2792]
	mov	r2, r2, lsr #2
	add	r1, r1, #1
	ldr	r2, [r4, r2, asl #2]
	mov	r4, r2, lsr #8
	strb	r2, [r3, #-4]
	strb	r4, [r3, #-3]
	mov	r4, r2, lsr #16
	mov	r2, r2, lsr #24
	strb	r4, [r3, #-2]
	strb	r2, [r3, #-1]
	mov	r2, ip
	b	.L1423
.L1464:
	ldr	r0, [r6, #2824]
	mov	r2, #0
	ldr	r1, [r6, #2776]
	mov	r7, r7, lsr #2
	mov	fp, r2
.L1425:
	cmp	r2, r7
	bcs	.L1421
	cmp	r0, #0
	beq	.L1421
	add	r3, r2, #8
	ldr	r3, [r8, r3, asl #2]
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	tst	r3, #4
	bne	.L1449
	ldr	r4, [sp, #20]
	ubfx	r4, r4, #15, #1
	cmp	r4, #0
	bne	.L1449
	cmp	r1, #5
	bls	.L1427
	ldr	ip, [sp, #20]
	ldr	r4, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	ip, ip, #3, #5
	ldr	lr, [sp, #20]
	ubfx	r4, r4, #27, #1
	ubfx	r3, r3, #16, #5
	ubfx	lr, lr, #29, #1
	orr	ip, ip, r4, asl #5
	orr	r3, r3, lr, asl #5
	cmp	ip, r3
	ldr	r3, [sp, #20]
	ldrhi	r4, [sp, #20]
	ubfxhi	r3, r3, #3, #5
	ldrls	r4, [sp, #20]
	ubfxls	r3, r3, #16, #5
	ubfxhi	r4, r4, #27, #1
	ubfxls	r4, r4, #29, #1
	b	.L1463
.L1427:
	cmp	r1, #3
	bls	.L1429
	ldr	ip, [sp, #20]
	ldr	r4, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	ip, ip, #3, #5
	ldr	lr, [sp, #20]
	ubfx	r4, r4, #28, #1
	ubfx	r3, r3, #16, #5
	ubfx	lr, lr, #30, #1
	orr	ip, ip, r4, asl #5
	orr	r3, r3, lr, asl #5
	cmp	ip, r3
	ldr	r3, [sp, #20]
	ldrhi	r4, [sp, #20]
	ubfxhi	r3, r3, #3, #5
	ldrls	r4, [sp, #20]
	ubfxls	r3, r3, #16, #5
	ubfxhi	r4, r4, #28, #1
	ubfxls	r4, r4, #30, #1
.L1463:
	orr	r4, r3, r4, asl #5
.L1429:
	cmp	fp, r4
	movcc	fp, r4
	b	.L1426
.L1449:
	mvn	fp, #0
.L1426:
	add	r2, r2, #1
	b	.L1425
.L1421:
	mov	r3, #0
	str	r3, [r8, #16]
	b	.L1432
.L1419:
	cmp	r1, #1
	mov	r4, #0
	bne	.L1461
	mov	fp, r4
.L1433:
	cmp	r4, r7
	bcs	.L1465
	ldr	r2, [sp, #128]
	cmp	r10, #0
	addne	r3, r10, r4, asl #9
	ldr	r1, [sp, #128]
	moveq	r3, r10
	cmp	r2, #0
	and	ip, r4, #3
	mov	r0, r8
	movne	r2, #2
	moveq	r2, #0
	mla	r2, r4, r2, r1
	mov	r1, #1
	str	ip, [sp, #8]
	add	r4, r4, #2
	str	r2, [sp]
	mov	r2, ip
	bl	NandcCopy1KB
	mov	r0, r9
	mov	r1, #1
	mov	r2, #2
	ldr	ip, [sp, #8]
	str	fp, [sp]
	str	fp, [sp, #4]
	mov	r3, ip
	bl	NandcXferStart
	mov	r0, r9
	bl	NandcXferComp
	b	.L1433
.L1465:
	mov	fp, #0
	b	.L1432
.L1461:
	str	r4, [sp]
	mov	r1, r4
	str	r4, [sp, #4]
	mov	r2, #2
	mov	r3, r4
	mov	fp, r4
	bl	NandcXferStart
	str	r10, [sp, #8]
.L1438:
	cmp	r4, r7
	bcs	.L1432
	mov	r0, r9
	bl	NandcXferComp
	ldr	r3, [r8, #32]
	add	ip, r4, #2
	cmp	ip, r7
	str	r3, [sp, #20]
	bcs	.L1439
	mov	r3, #0
	mov	r0, r9
	str	r3, [sp]
	mov	r1, r3
	str	r3, [sp, #4]
	mov	r2, #2
	and	r3, ip, #3
	str	ip, [sp, #12]
	bl	NandcXferStart
	ldr	ip, [sp, #12]
.L1439:
	ldr	r3, [sp, #20]
	tst	r3, #4
	mvnne	fp, #0
	bne	.L1440
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r2, r2, #3, #5
	ubfx	r3, r3, #27, #1
	orr	r3, r2, r3, asl #5
	cmp	fp, r3
	movcc	fp, r3
.L1440:
	ldr	r1, [sp, #128]
	cmp	r10, #0
	ldr	r3, [sp, #8]
	sub	r2, ip, #2
	moveq	r3, #0
	ldr	r0, [sp, #128]
	cmp	r1, #0
	and	r2, r2, #3
	str	ip, [sp, #12]
	movne	r1, #2
	moveq	r1, #0
	mla	r4, r4, r1, r0
	mov	r0, r8
	mov	r1, #0
	str	r4, [sp]
	bl	NandcCopy1KB
	ldr	ip, [sp, #12]
	ldr	r3, [sp, #8]
	mov	r4, ip
	add	r3, r3, #1024
	str	r3, [sp, #8]
	b	.L1438
.L1432:
	ldr	r3, [r6, #2776]
	clz	r5, r5
	cmp	r3, #5
	mov	r5, r5, lsr #5
	movls	r5, #0
	cmp	r5, #0
	beq	.L1444
	ldr	r3, [r8]
	and	r2, r3, #139264
	cmp	r2, #139264
	orreq	r3, r3, #131072
	streq	r3, [r8]
	mvneq	fp, #0
.L1444:
	mov	r0, fp
	add	sp, sp, #92
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1467:
	.align	2
.L1466:
	.word	.LANCHOR0
	.fnend
	.size	NandcXferData, .-NandcXferData
	.align	2
	.global	FlashReadRawPage
	.type	FlashReadRawPage, %function
FlashReadRawPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r8, r3
	ldr	r3, .L1471
	subs	r4, r0, #0
	mov	r6, r1
	mov	r7, r2
	ldrb	r5, [r3, #265]	@ zero_extendqisi2
	bne	.L1469
	ldr	r2, .L1471+4
	ldrb	r3, [r2, #1]	@ zero_extendqisi2
	ldr	r2, [r2, #4]
	mul	r2, r2, r3
	cmp	r1, r2
	movcc	r5, #4
.L1469:
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
	mov	r1, #0
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
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1472:
	.align	2
.L1471:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashReadRawPage, .-FlashReadRawPage
	.align	2
	.global	FlashDdrTunningRead
	.type	FlashDdrTunningRead, %function
FlashDdrTunningRead:
	.fnstart
	@ args = 4, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r7, r3
	ldr	r4, .L1500
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
	movcc	r9, #6
	movcs	r9, #12
	cmp	r3, #0
	moveq	r6, #1024
	beq	.L1475
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r0, r10
	bl	FlashReset
	mov	r2, fp
	mov	r3, r7
	mov	r0, r10
	ldr	r1, [sp]
	bl	FlashReadRawPage
	mov	r6, r0
	ldrb	r0, [r4, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
	cmn	r6, #1
	bne	.L1476
.L1485:
	mvn	r6, #0
	b	.L1477
.L1476:
	mov	r2, r6
	ldr	r0, .L1500+4
	ldr	r1, [sp]
	bl	printk
	cmp	r6, #9
	addls	r4, r4, r10, asl #3
	ldrls	r3, [r4, #12]
	ldr	r4, .L1500+8
	ldrls	r2, [r3, #3840]
	ldrls	r2, [r3]
	orrls	r2, r2, #131072
	strls	r2, [r3]
	ldr	r3, [r4, #156]
	add	r3, r3, #1
	str	r3, [r4, #156]
	cmp	r3, #2048
	movcs	r7, #0
	strcs	r7, [r4, #156]
	movcs	fp, r7
	bcc	.L1477
.L1475:
	mov	r4, #0
	mvn	r8, #0
	mov	ip, r4
	mov	r5, r4
	str	r4, [sp, #4]
.L1483:
	uxtb	r0, r9
	str	ip, [sp, #12]
	bl	NandcSetDdrPara
	mov	r3, r7
	mov	r0, r10
	mov	r2, fp
	ldr	r1, [sp]
	bl	FlashReadRawPage
	add	r3, r6, #1
	cmp	r0, r3
	ldr	ip, [sp, #12]
	bhi	.L1479
	cmp	r0, #2
	bhi	.L1489
	add	r5, r5, #1
	cmp	r5, #9
	bls	.L1489
	rsb	r4, r5, r9
	mov	r6, r0
	mov	r8, #0
	b	.L1481
.L1479:
	ldr	r3, [sp, #4]
	cmp	r3, r5
	bcs	.L1490
	cmp	r5, #7
	rsb	ip, r5, r4
	bhi	.L1491
	str	r5, [sp, #4]
	b	.L1490
.L1489:
	mov	r8, #0
	mov	r4, r9
	mov	r6, r0
	mov	r7, r8
	mov	fp, r8
	b	.L1480
.L1490:
	mov	r5, #0
.L1480:
	add	r9, r9, #2
	cmp	r9, #69
	bls	.L1483
.L1481:
	ldr	r3, [sp, #4]
	cmp	r3, r5
	movcs	r4, ip
	b	.L1482
.L1491:
	mov	r4, ip
.L1482:
	cmp	r4, #0
	beq	.L1484
	ldr	r0, .L1500+12
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	bl	NandcSetDdrPara
.L1484:
	cmn	r8, #1
	bne	.L1477
	ldr	r0, .L1500+16
	mov	r1, r10
	ldr	r2, [sp]
	bl	printk
	ldr	r3, [sp, #56]
	cmp	r3, #0
	beq	.L1485
	ldr	r3, [sp, #8]
	ubfx	r0, r3, #8, #8
	bl	NandcSetDdrPara
.L1477:
	mov	r0, r6
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1501:
	.align	2
.L1500:
	.word	.LANCHOR0
	.word	.LC90
	.word	.LANCHOR2
	.word	.LC91
	.word	.LC92
	.fnend
	.size	FlashDdrTunningRead, .-FlashDdrTunningRead
	.align	2
	.global	FlashReadPage
	.type	FlashReadPage, %function
FlashReadPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r5, r0
	mov	r6, r1
	mov	r8, r2
	mov	r7, r3
	bl	FlashReadRawPage
	cmn	r0, #1
	mov	r4, r0
	bne	.L1503
	ldr	r9, .L1523
	ldrb	fp, [r9, #8]	@ zero_extendqisi2
	mov	r10, r9
	cmp	fp, #0
	bne	.L1504
.L1506:
	ldrb	r3, [r10, #2768]	@ zero_extendqisi2
	ldr	r9, .L1523
	cmp	r3, #0
	beq	.L1503
	b	.L1522
.L1504:
	mov	r3, #0
	mov	r0, r5
	strb	r3, [r9, #8]
	mov	r1, r6
	mov	r2, r8
	mov	r3, r7
	bl	FlashReadRawPage
	strb	fp, [r9, #8]
	cmn	r0, #1
	movne	r4, r0
	beq	.L1506
	b	.L1503
.L1522:
	ldr	r3, [r9, #1824]
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	ldr	r10, [r3, #304]
	mov	r3, #1
	str	r3, [sp]
	mov	r3, r7
	bl	FlashDdrTunningRead
	cmn	r0, #1
	mov	r4, r0
	beq	.L1507
	ldrb	r3, [r9, #1889]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1503
.L1507:
	ubfx	r0, r10, #8, #8
	bl	NandcSetDdrPara
.L1503:
	ldr	r9, .L1523+4
	ldr	ip, [r9, #160]
	adds	r3, ip, #0
	movne	r3, #1
	cmn	r4, #1
	movne	r3, #0
	cmp	r3, #0
	beq	.L1508
	mov	r1, r6
	mov	r2, r8
	mov	r3, r7
	mov	r0, r5
	blx	ip
	mov	r2, r5
	mov	r3, r6
	mov	r4, r0
	ldr	r0, .L1523+8
	mov	r1, r4
	bl	printk
	cmn	r4, #1
	bne	.L1508
	ldr	r3, .L1523
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1508
	mov	r0, r5
	bl	flash_enter_slc_mode
	ldr	ip, [r9, #160]
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	mov	r3, r7
	blx	ip
	mov	r4, r0
	mov	r0, r5
	bl	flash_exit_slc_mode
.L1508:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1524:
	.align	2
.L1523:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC93
	.fnend
	.size	FlashReadPage, .-FlashReadPage
	.align	2
	.global	FlashDdrParaScan
	.type	FlashDdrParaScan, %function
FlashDdrParaScan:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r7, r0
	ldr	r5, .L1535
	mov	r6, r1
	mov	r4, #0
	ldrb	r0, [r5, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r5, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
	mov	r1, r6
	mov	r2, r4
	mov	r3, r4
	mov	r0, r7
	str	r4, [sp]
	bl	FlashDdrTunningRead
	mov	r3, r4
	mov	r1, r6
	mov	r2, r4
	mov	r8, r0
	mov	r0, r7
	bl	FlashReadRawPage
	mov	r3, r5
	cmn	r0, #1
	cmnne	r8, #1
	bne	.L1526
	ldrb	r2, [r5, #2760]	@ zero_extendqisi2
	tst	r2, #1
	beq	.L1526
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	strb	r4, [r5, #2768]
	b	.L1527
.L1526:
	mov	r2, #1
	strb	r2, [r3, #2768]
.L1527:
	mov	r0, #0
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1536:
	.align	2
.L1535:
	.word	.LANCHOR0
	.fnend
	.size	FlashDdrParaScan, .-FlashDdrParaScan
	.align	2
	.global	FlashLoadPhyInfo
	.type	FlashLoadPhyInfo, %function
FlashLoadPhyInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, .L1553
	mov	r4, #0
	ldr	r7, .L1553+4
	mov	r9, #4
	ldr	r5, .L1553+8
	mvn	r8, #0
	ldr	r0, [r3]	@ unaligned
	ldr	r3, [r7, #4]
	str	r4, [r5, #168]
	mov	r6, r5
	str	r0, [sp, #12]	@ unaligned
	mov	r0, r4
	str	r3, [sp, #4]
	ldr	r3, [r5, #152]
	str	r3, [r5, #164]
	bl	flash_enter_slc_mode
.L1538:
	add	fp, r4, #1
	mov	r10, #0
.L1540:
	add	r3, sp, #12
	ldrb	r0, [r3, r10]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	mov	r1, r4
	ldr	r2, [r5, #152]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1539
	mov	r0, #0
	mov	r1, fp
	ldr	r2, [r6, #152]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1539
	add	r10, r10, #1
	cmp	r10, #4
	beq	.L1541
	b	.L1540
.L1542:
	add	r0, fp, #12
	movw	r1, #2036
	bl	js_hash
	ldr	r3, [fp, #8]
	cmp	r3, r0
	mvnne	r8, #0
	bne	.L1541
	ldr	r8, .L1553+12
	add	r1, fp, #160
	mov	r2, #32
	mov	r0, r8
	bl	ftl_memcpy
	ldr	r1, [r6, #164]
	mov	r2, #32
	ldr	r0, .L1553+16
	add	r1, r1, #192
	bl	ftl_memcpy
	ldr	r1, [r6, #164]
	mov	r2, #852
	ldr	r0, .L1553+20
	add	r1, r1, #224
	bl	ftl_memcpy
	ldrh	r0, [r8, #10]
	bl	FlashBlockAlignInit
	ldr	r8, [r6, #164]
	str	r4, [r6, #168]
	mov	r0, r4
	ldr	r1, [r7, #4]
	ldr	r3, [r8, #1076]
	strb	r3, [r7, #2768]
	bl	__aeabi_uidiv
	add	r0, r0, #1
	cmp	r0, #1
	strhi	r0, [r6, #172]
	movls	r3, #2
	strls	r3, [r6, #172]
	ldrh	r3, [r8, #14]
	mov	r8, #0
	strb	r3, [r5, #176]
.L1541:
	ldr	r3, [sp, #4]
	subs	r9, r9, #1
	add	r4, r4, r3
	bne	.L1538
	mov	r0, r9
.L1552:
	bl	flash_exit_slc_mode
	mov	r0, r8
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1539:
	ldr	fp, [r5, #164]
	ldr	r2, .L1553+24
	ldr	r3, [fp]
	cmp	r3, r2
	bne	.L1541
	cmp	r8, #0
	bne	.L1542
	mov	r0, r4
	ldr	r1, [r7, #4]
	bl	__aeabi_uidiv
	ldr	r3, .L1553+8
	add	r0, r0, #1
	str	r0, [r3, #172]
	mov	r0, r8
	b	.L1552
.L1554:
	.align	2
.L1553:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR1+256
	.word	.LANCHOR0+44
	.word	.LANCHOR0+1902
	.word	1312902724
	.fnend
	.size	FlashLoadPhyInfo, .-FlashLoadPhyInfo
	.align	2
	.global	ToshibaReadRetrial
	.type	ToshibaReadRetrial, %function
ToshibaReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	mov	r9, r0
	mov	fp, r3
	str	r2, [sp, #8]
	str	r1, [sp, #16]
	bl	NandcWaitFlashReady
	ldr	r4, .L1584
	add	r3, r4, r9, asl #3
	ldrb	r2, [r3, #16]	@ zero_extendqisi2
	ldr	r6, [r3, #12]
	ldrb	r3, [r4, #1900]	@ zero_extendqisi2
	add	r7, r2, #8
	sub	r3, r3, #67
	add	r7, r6, r7, asl #8
	cmp	r3, #1
	mov	r3, r2, asl #8
	str	r3, [sp, #12]
	movls	r3, #0
	strls	r3, [sp, #4]
	bls	.L1556
	ldrb	r5, [r4, #2768]	@ zero_extendqisi2
	cmp	r5, #0
	streq	r5, [sp, #4]
	beq	.L1557
	mov	r0, #0
	str	r2, [sp, #20]
	bl	NandcSetDdrMode
	mov	r3, #1
	ldr	r2, [sp, #20]
	str	r3, [sp, #4]
.L1557:
	ldr	r3, [sp, #12]
	mov	r1, #92
	add	r3, r6, r3
	str	r1, [r3, #2056]
	mov	r1, #197
	str	r1, [r3, #2056]
.L1556:
	mov	r8, #1
	mvn	r10, #0
	mov	r3, r2, asl #8
	str	r3, [sp, #20]
.L1558:
	ldr	r3, .L1584+4
	ldrb	r3, [r3, #177]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r8, r3
	bcs	.L1583
	ldrb	r3, [r4, #1900]	@ zero_extendqisi2
	mov	r0, r7
	uxtb	r1, r8
	sub	r3, r3, #67
	cmp	r3, #1
	bhi	.L1559
	bl	SandiskSetRRPara
	b	.L1560
.L1559:
	bl	ToshibaSetRRPara
.L1560:
	ldrb	r3, [r4, #1900]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L1561
	ldr	r3, .L1584+4
	ldrb	r3, [r3, #177]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r8, r3
	ldreq	r3, [sp, #20]
	moveq	r2, #179
	addeq	r3, r6, r3
	streq	r2, [r3, #2056]
.L1561:
	ldr	r3, [sp, #12]
	mov	r2, #38
	add	r3, r6, r3
	str	r2, [r3, #2056]
	mov	r2, #93
	str	r2, [r3, #2056]
	ldr	r3, [sp, #4]
	cmp	r3, #0
	beq	.L1562
	mov	r0, #4
	bl	NandcSetDdrMode
	mov	r0, r9
	mov	r3, fp
	ldr	r1, [sp, #16]
	ldr	r2, [sp, #8]
	bl	FlashReadRawPage
	mov	r5, r0
	mov	r0, #0
	bl	NandcSetDdrMode
	b	.L1563
.L1562:
	mov	r0, r9
	ldr	r1, [sp, #16]
	ldr	r2, [sp, #8]
	mov	r3, fp
	bl	FlashReadRawPage
	mov	r5, r0
.L1563:
	cmn	r5, #1
	beq	.L1564
	ldrb	r2, [r4, #1889]	@ zero_extendqisi2
	cmn	r10, #1
	moveq	r10, r5
	add	r2, r2, r2, asl #1
	cmp	r5, r2, asr #2
	bcc	.L1566
	mov	fp, #0
	str	fp, [sp, #8]
.L1564:
	add	r8, r8, #1
	b	.L1558
.L1583:
	mov	r5, r10
.L1566:
	ldrb	r2, [r4, #1900]	@ zero_extendqisi2
	mov	r0, r7
	mov	r1, #0
	sub	r2, r2, #67
	cmp	r2, #1
	bhi	.L1568
	bl	SandiskSetRRPara
	b	.L1569
.L1568:
	bl	ToshibaSetRRPara
.L1569:
	ldr	r3, [sp, #12]
	mov	r2, #255
	add	r6, r6, r3
	str	r2, [r6, #2056]
	ldrb	r2, [r4, #1889]	@ zero_extendqisi2
	add	r2, r2, r2, asl #1
	cmp	r5, r2, asr #2
	bcc	.L1570
	cmn	r5, #1
	movne	r5, #256
.L1570:
	mov	r0, r9
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #4]
	cmp	r3, #0
	beq	.L1571
	mov	r0, #4
	bl	NandcSetDdrMode
.L1571:
	mov	r0, r5
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1585:
	.align	2
.L1584:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	ToshibaReadRetrial, .-ToshibaReadRetrial
	.align	2
	.global	SamsungReadRetrial
	.type	SamsungReadRetrial, %function
SamsungReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r9, r0
	ldr	r5, .L1600
	mov	r10, r2
	mov	r8, r3
	mov	fp, r1
	bl	NandcWaitFlashReady
	add	r2, r5, r9, asl #3
	ldr	ip, .L1600+4
	mov	r7, #1
	ldrb	r3, [r2, #16]	@ zero_extendqisi2
	mvn	r4, #0
	ldr	r6, [r2, #12]
	add	r3, r3, #8
	add	r6, r6, r3, asl #8
.L1587:
	ldrb	r3, [ip, #177]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r7, r3
	bcs	.L1590
	mov	r0, r6
	uxtb	r1, r7
	str	ip, [sp, #4]
	bl	SamsungSetRRPara
	mov	r0, r9
	mov	r1, fp
	mov	r2, r10
	mov	r3, r8
	bl	FlashReadRawPage
	cmn	r0, #1
	ldr	ip, [sp, #4]
	beq	.L1588
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L1593
	mov	r8, #0
	mov	r10, r8
.L1588:
	add	r7, r7, #1
	b	.L1587
.L1593:
	mov	r4, r0
.L1590:
	mov	r0, r6
	mov	r1, #0
	bl	SamsungSetRRPara
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r4, r3, asr #2
	bcc	.L1592
	cmn	r4, #1
	movne	r4, #256
.L1592:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1601:
	.align	2
.L1600:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	SamsungReadRetrial, .-SamsungReadRetrial
	.align	2
	.global	MicronReadRetrial
	.type	MicronReadRetrial, %function
MicronReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
.L1604:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r3
	ldr	r3, .L1628
	mov	fp, r2
	.pad #36
	sub	sp, sp, #36
	mov	r6, r0
	mov	r10, #0
	ldrb	r5, [r3, #1889]	@ zero_extendqisi2
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	str	r1, [sp, #20]
	cmp	r3, #0
	addeq	r5, r5, r5, asl #1
	ldrne	r2, .L1628+4
	ubfxeq	r5, r5, #2, #8
	smullne	r2, r3, r5, r2
	uxtbne	r5, r3
	ldr	r3, .L1628
	add	r3, r3, r0, asl #3
	str	r3, [sp, #24]
.L1614:
	mov	r0, r6
	mov	r9, #0
	bl	NandcWaitFlashReady
	mvn	r4, #0
	ldr	r3, [sp, #24]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [sp, #24]
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	str	r3, [sp, #16]
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #12]
	add	r7, r3, r2, asl #8
.L1605:
	ldr	r3, .L1628+8
	ldrb	r3, [r3, #177]	@ zero_extendqisi2
	cmp	r9, r3
	bcs	.L1608
	mov	r3, #239
	mov	r0, #200
	str	r3, [r7, #2056]
	mov	r3, #137
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
	cmn	r0, #1
	beq	.L1606
	cmn	r4, #1
	ldr	ip, [sp, #28]
	moveq	r4, r0
	cmp	r0, r5
	bcc	.L1616
	mov	r8, ip
	mov	fp, ip
.L1606:
	ldr	r9, [sp, #8]
	b	.L1605
.L1616:
	mov	r4, r0
	mov	r8, ip
	mov	fp, ip
.L1608:
	ldr	r2, [sp, #16]
	mov	r0, #200
	ldr	r3, [sp, #12]
	add	r7, r3, r2, asl #8
	mov	r3, #239
	str	r3, [r7, #2056]
	mov	r3, #137
	str	r3, [r7, #2052]
	bl	NandcDelayns
	cmp	r4, r5
	mov	r3, #0
	str	r3, [r7, #2048]
	str	r3, [r7, #2048]
	str	r3, [r7, #2048]
	str	r3, [r7, #2048]
	bcc	.L1610
	cmn	r4, #1
	movne	r4, #256
.L1610:
	cmn	r4, #1
	movne	r7, #0
	moveq	r7, #1
	cmp	r4, #256
	movne	r1, r7
	orreq	r1, r7, #1
	cmp	r1, #0
	beq	.L1611
	str	r4, [sp]
	mov	r1, r9
	ldr	r0, .L1628+12
	mov	r3, r9
	ldr	r2, [sp, #20]
	bl	printk
	cmp	r10, #0
	bne	.L1612
	ldr	r3, .L1628
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r7, #0
	andne	r7, r7, #1
	cmp	r7, #0
	beq	.L1621
	mov	r0, r6
	mov	r1, #3
	bl	micron_auto_read_calibration_config
	mov	r10, #1
	b	.L1614
.L1612:
	mov	r0, r6
	mov	r1, #0
	bl	micron_auto_read_calibration_config
	cmn	r4, #1
	movne	r4, #256
	b	.L1621
.L1611:
	cmp	r10, #0
	beq	.L1621
	mov	r0, r6
	mov	r4, #256
	bl	micron_auto_read_calibration_config
.L1621:
	mov	r0, r4
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1629:
	.align	2
.L1628:
	.word	.LANCHOR0
	.word	1431655766
	.word	.LANCHOR2
	.word	.LC94
	.fnend
	.size	MicronReadRetrial, .-MicronReadRetrial
	.align	2
	.global	HynixReadRetrial
	.type	HynixReadRetrial, %function
HynixReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r8, r3
	ldr	r5, .L1648
	mov	r10, r2
	mov	r7, r0
	mov	fp, r1
	add	r2, r5, r0
	mvn	r6, #0
	ldr	r3, [r5, #1784]
	ldrb	r4, [r2, #1914]	@ zero_extendqisi2
	ldrb	r9, [r5, #1904]	@ zero_extendqisi2
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #7
	cmp	r3, #1
	ldrlsb	r4, [r2, #1922]	@ zero_extendqisi2
	bl	NandcWaitFlashReady
	mov	ip, #0
.L1632:
	cmp	ip, r9
	bcs	.L1636
	add	r4, r4, #1
	mov	r0, r7
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	uxtb	r4, r4
	ldr	r2, .L1648+4
	cmp	r4, r9
	str	ip, [sp, #4]
	movcs	r4, #0
	mov	r3, r4
	bl	HynixSetRRPara
	mov	r0, r7
	mov	r1, fp
	mov	r2, r10
	mov	r3, r8
	bl	FlashReadRawPage
	cmn	r0, #1
	ldr	ip, [sp, #4]
	beq	.L1634
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	cmn	r6, #1
	moveq	r6, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L1641
	mov	r8, #0
	mov	r10, r8
.L1634:
	add	ip, ip, #1
	b	.L1632
.L1641:
	mov	r6, r0
.L1636:
	ldr	r3, [r5, #1784]
	add	r7, r5, r7
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #7
	cmp	r3, #1
	ldrb	r3, [r5, #1889]	@ zero_extendqisi2
	strlsb	r4, [r7, #1922]
	strhib	r4, [r7, #1914]
	add	r3, r3, r3, asl #1
	cmp	r6, r3, asr #2
	bcc	.L1640
	cmn	r6, #1
	movne	r6, #256
.L1640:
	mov	r0, r6
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1649:
	.align	2
.L1648:
	.word	.LANCHOR0
	.word	.LANCHOR0+1906
	.fnend
	.size	HynixReadRetrial, .-HynixReadRetrial
	.align	2
	.global	FlashProgPage
	.type	FlashProgPage, %function
FlashProgPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r8, r3
	ldr	r3, .L1654
	subs	r4, r0, #0
	mov	r5, r1
	mov	r7, r2
	ldrb	r6, [r3, #265]	@ zero_extendqisi2
	bne	.L1651
	ldr	r2, .L1654+4
	ldrb	r3, [r2, #1]	@ zero_extendqisi2
	ldr	r1, [r2, #4]
	mul	r1, r1, r3
	cmp	r5, r1
	bcs	.L1651
	ldrb	r3, [r2]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r6, r6, #2
	movne	r6, #4
.L1651:
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
	mov	r1, #1
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
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1655:
	.align	2
.L1654:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashProgPage, .-FlashProgPage
	.align	2
	.global	FlashSavePhyInfo
	.type	FlashSavePhyInfo, %function
FlashSavePhyInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r4, .L1666
	ldr	r5, .L1666+4
	ldr	r8, .L1666+8
	ldr	r3, [r4, #152]
	mov	r9, r4
	ldrb	r0, [r4, #178]	@ zero_extendqisi2
	mov	r10, r5
	str	r3, [r4, #164]
	bl	FlashBchSel
	ldr	r0, [r4, #152]
	mov	r1, #2048
	bl	__memzero
	ldr	r3, [r4, #164]
	ldr	r1, .L1666+12
	mov	r2, #32
	str	r8, [r3]
	ldr	r0, [r4, #164]
	ldrb	r3, [r5, #1612]	@ zero_extendqisi2
	add	r0, r0, #16
	strh	r3, [r0, #-4]	@ movhi
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	strh	r3, [r0, #-2]	@ movhi
	ldrb	r3, [r5, #2768]	@ zero_extendqisi2
	str	r3, [r0, #1060]
	bl	ftl_memcpy
	ldr	r0, [r4, #164]
	add	r1, r5, #1616
	mov	r2, #8
	add	r0, r0, #80
	bl	ftl_memcpy
	ldr	r0, [r4, #164]
	ldr	r1, .L1666+16
	mov	r2, #32
	add	r0, r0, #96
	bl	ftl_memcpy
	ldr	r0, [r4, #164]
	ldr	r1, .L1666+20
	mov	r2, #32
	add	r0, r0, #160
	bl	ftl_memcpy
	ldr	r0, [r4, #164]
	add	r1, r5, #44
	mov	r2, #32
	add	r0, r0, #192
	bl	ftl_memcpy
	ldr	r0, [r4, #164]
	mov	r2, #852
	ldr	r1, .L1666+24
	add	r0, r0, #224
	bl	ftl_memcpy
	ldr	r6, [r4, #164]
	movw	r1, #2036
	add	r0, r6, #12
	bl	js_hash
	movw	r3, #1592
	str	r3, [r6, #4]
	ldr	r3, [r4, #180]
	str	r3, [r4, #164]
	str	r0, [r6, #8]
	mov	r0, #0
	bl	flash_enter_slc_mode
	mov	r6, #0
	mov	r7, r6
.L1659:
	ldr	r1, [r5, #4]
	mov	r0, #0
	mov	r2, r0
	mul	r1, r1, r7
	bl	FlashEraseBlock
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r4, #152]
	mov	r3, r0
	mul	r1, r1, r7
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r4, #152]
	mov	r3, r0
	mul	r1, r1, r7
	add	r1, r1, #1
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r4, #180]
	mov	r3, r0
	mul	r1, r1, r7
	bl	FlashReadRawPage
	add	r2, r7, #1
	cmn	r0, #1
	beq	.L1657
	ldr	fp, [r9, #164]
	ldr	r3, [fp]
	cmp	r3, r8
	bne	.L1657
	add	r0, fp, #12
	movw	r1, #2036
	str	r2, [sp, #4]
	bl	js_hash
	ldr	r3, [fp, #8]
	cmp	r3, r0
	ldr	r2, [sp, #4]
	bne	.L1657
	ldr	r3, [r10, #4]
	cmp	r6, #1
	str	r2, [r9, #172]
	mul	r7, r3, r7
	str	r7, [r9, #168]
	beq	.L1660
	mov	r6, #1
.L1657:
	cmp	r2, #4
	mov	r7, r2
	bne	.L1659
	b	.L1658
.L1660:
	mov	r6, #2
.L1658:
	mov	r0, #0
	bl	flash_exit_slc_mode
	clz	r0, r6
	mov	r0, r0, lsr #5
	rsb	r0, r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1667:
	.align	2
.L1666:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	1312902724
	.word	.LANCHOR0+1788
	.word	.LANCHOR0+1624
	.word	.LANCHOR1+256
	.word	.LANCHOR0+1902
	.fnend
	.size	FlashSavePhyInfo, .-FlashSavePhyInfo
	.align	2
	.global	FlashReadIdbDataRaw
	.type	FlashReadIdbDataRaw, %function
FlashReadIdbDataRaw:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r0
	ldr	r3, .L1688
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L1688+4
	ldr	r0, [r3]	@ unaligned
	ldr	r3, [r4, #2764]
	ldrb	r10, [r4, #1889]	@ zero_extendqisi2
	cmp	r3, #0
	str	r0, [sp, #12]	@ unaligned
	beq	.L1669
	mov	r0, #0
	bl	flash_enter_slc_mode
.L1669:
	mov	r0, r9
	mov	r1, #2048
	bl	__memzero
	mvn	r8, #0
	mov	r5, #2
.L1670:
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	cmp	r5, r3
	bcs	.L1674
	ldr	fp, .L1688+8
	mov	r7, #0
.L1672:
	add	r3, sp, #12
	ldr	r6, .L1688+8
	ldrb	ip, [r7, r3]	@ zero_extendqisi2
	mov	r0, ip
	str	ip, [sp, #4]
	bl	FlashBchSel
	ldr	r1, [r4, #4]
	mov	r0, #0
	ldr	r2, [fp, #152]
	mov	r3, r0
	mul	r1, r1, r5
	bl	FlashReadRawPage
	cmn	r0, #1
	ldr	ip, [sp, #4]
	bne	.L1671
	add	r7, r7, #1
	cmp	r7, #4
	bne	.L1672
	b	.L1673
.L1671:
	ldr	r3, [r6, #152]
	ldr	r2, .L1688+12
	ldr	r3, [r3]
	cmp	r3, r2
	bne	.L1673
	mov	r1, ip
	ldr	r0, .L1688+16
	bl	printk
	mov	r2, #2048
	mov	r0, r9
	ldr	r1, [r6, #152]
	bl	ftl_memcpy
	ldr	r3, [r6, #152]
	ldr	r2, .L1688+4
	ldr	r3, [r3, #512]
	strb	r3, [r2, #1]
	ldr	r3, [r6, #172]
	cmp	r3, r5
	bls	.L1677
	str	r5, [r6, #172]
	bl	FlashSavePhyInfo
	mov	r8, #0
.L1673:
	add	r5, r5, #1
	b	.L1670
.L1677:
	mov	r8, #0
.L1674:
	mov	r0, r10
	bl	FlashBchSel
	ldr	r3, [r4, #2764]
	cmp	r3, #0
	beq	.L1681
	mov	r0, #0
	bl	flash_exit_slc_mode
.L1681:
	mov	r0, r8
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1689:
	.align	2
.L1688:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-52655045
	.word	.LC95
	.fnend
	.size	FlashReadIdbDataRaw, .-FlashReadIdbDataRaw
	.align	2
	.global	FlashInit
	.type	FlashInit, %function
FlashInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r0
	.pad #20
	sub	sp, sp, #20
	mov	r0, #32768
	bl	ftl_malloc
	ldr	r8, .L1785
	ldr	r4, .L1785+4
	mov	r7, #0
	add	r9, r4, #12
	str	r0, [r8, #152]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r8, #180]
	mov	r0, #4096
	bl	ftl_malloc
	str	r0, [r8, #184]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r8, #188]
	mov	r0, #4096
	bl	ftl_malloc
	mov	r3, #50
	str	r7, [r8, #172]
	strb	r3, [r4, #1]
	strb	r3, [r8, #176]
	mov	r3, #128
	strb	r7, [r4, #2768]
	str	r3, [r4, #4]
	mov	r3, #60
	str	r7, [r8, #156]
	strb	r7, [r4]
	strb	r7, [r8, #196]
	strb	r3, [r8, #178]
	str	r0, [r8, #192]
	mov	r0, r5
	bl	NandcInit
	ldr	r5, .L1785+8
.L1696:
	add	r3, r9, r7, asl #3
	uxtb	fp, r7
	ldr	r6, [r9, r7, asl #3]
	ldrb	r10, [r3, #4]	@ zero_extendqisi2
	mov	r0, fp
	bl	FlashReset
	mov	r0, fp
	bl	NandcFlashCs
	mov	r3, #144
	add	r6, r6, r10, asl #8
	mov	r0, #200
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
	bne	.L1691
	mov	r2, #239
	mov	r0, #200
	str	r2, [r6, #2056]
	mov	r2, #1
	str	r2, [r6, #2052]
	bl	NandcDelayns
	mov	r2, #4
	str	r2, [r6, #2048]
	str	r10, [r6, #2048]
	str	r10, [r6, #2048]
	str	r10, [r6, #2048]
.L1691:
	mov	r0, fp
	bl	NandcFlashDeCs
	ldrb	r2, [r5]	@ zero_extendqisi2
	sub	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L1692
	ldrb	r1, [r5, #2]	@ zero_extendqisi2
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	ldr	r0, .L1785+12
	str	r1, [sp]
	ldrb	r1, [r5, #3]	@ zero_extendqisi2
	str	r1, [sp, #4]
	ldrb	r1, [r5, #4]	@ zero_extendqisi2
	str	r1, [sp, #8]
	ldrb	r1, [r5, #5]	@ zero_extendqisi2
	str	r1, [sp, #12]
	add	r1, r7, #1
	bl	printk
.L1692:
	cmp	r7, #0
	bne	.L1693
	ldrb	r3, [r4, #1788]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L1742
	ldr	r3, .L1785+4
	ldrb	r3, [r3, #1789]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L1742
.L1693:
	ldrb	r3, [r5]	@ zero_extendqisi2
	add	r7, r7, #1
	add	r5, r5, #8
	cmp	r3, #181
	moveq	r3, #44
	streqb	r3, [r5, #-8]
	cmp	r7, #4
	bne	.L1696
	ldrb	r3, [r4, #1788]	@ zero_extendqisi2
	cmp	r3, #173
	beq	.L1697
	ldr	r3, .L1785+4
	ldr	r0, [r3, #2772]
	bl	NandcSetDdrMode
.L1697:
	ldr	r0, .L1785+16
	mov	r1, #852
	bl	__memzero
	ldr	r5, .L1785+20
	ldr	r6, [r4, #2780]
	ldr	r3, .L1785+24
	cmp	r6, r5
	ldrb	r0, [r4, #1789]	@ zero_extendqisi2
	str	r3, [r4, #1784]
	mov	r3, #0
	strb	r3, [r4, #8]
	sub	ip, r0, #218
	ldreq	r3, .L1785+4
	clz	ip, ip
	moveq	r2, #1
	mov	ip, ip, lsr #5
	streq	r2, [r3, #2764]
	cmp	r0, #161
	cmpne	r0, #241
	and	r2, r0, #253
	moveq	r3, #1
	movne	r3, #0
	orr	r3, ip, r3
	cmp	r2, #209
	orreq	r3, r3, #1
	cmp	r3, #0
	bne	.L1699
	cmp	r0, #220
	bne	.L1700
	ldr	r3, .L1785+4
	ldrb	r3, [r3, #1791]	@ zero_extendqisi2
	cmp	r3, #149
	bne	.L1700
.L1699:
	ldrb	lr, [r4, #1788]	@ zero_extendqisi2
	mov	r3, #16
	strb	r3, [r4, #1]
	mov	r1, #1
	strb	r3, [r8, #178]
	cmp	lr, #152
	ldr	r3, .L1785+28
	strb	r1, [r4]
	ldr	r2, .L1785
	strb	lr, [r3, #3257]
	strb	r0, [r3, #3258]
	bne	.L1702
	ldr	lr, .L1785+32
	ldrsb	lr, [lr]
	cmp	lr, #0
	strltb	r1, [r2, #196]
	movge	r1, #24
	strgeb	r1, [r2, #178]
.L1702:
	movw	r2, #2049
	cmp	r6, r5
	cmpne	r6, r2
	moveq	r2, #16
	streqb	r2, [r8, #178]
	cmp	ip, #0
	ldrne	r2, .L1785+36
	movne	r1, #2048
	strneh	r1, [r2, #14]	@ movhi
	mvnne	r2, #37
	bne	.L1779
.L1706:
	cmp	r0, #220
	bne	.L1708
	ldr	r2, .L1785+36
	mov	r1, #4096
	strh	r1, [r2, #14]	@ movhi
	mvn	r2, #35
.L1779:
	strb	r2, [r3, #3258]
	b	.L1707
.L1708:
	cmp	r0, #211
	ldreq	r2, .L1785+36
	moveq	r1, #4096
	streqh	r1, [r2, #14]	@ movhi
	moveq	r2, #2
	streqb	r2, [r3, #3269]
.L1707:
	ldr	r1, .L1785+40
	mov	r2, #32
	ldr	r0, .L1785+44
	bl	ftl_memcpy
	ldr	r0, .L1785+24
	ldr	r1, .L1785+36
	mov	r2, #32
	bl	ftl_memcpy
.L1700:
	ldrb	r3, [r4]	@ zero_extendqisi2
	ldr	r6, .L1785+4
	cmp	r3, #0
	bne	.L1709
	bl	FlashLoadPhyInfoInRam
	cmp	r0, #0
	bne	.L1711
	ldr	r3, [r6, #1784]
	ldrh	r3, [r3, #16]
	mov	r3, r3, lsr #8
	tst	r3, #1
	and	r0, r3, #7
	strb	r0, [r6, #2760]
	bne	.L1711
	mov	r3, #1
	strb	r3, [r6, #2768]
	bl	FlashSetInterfaceMode
	ldrb	r0, [r6, #2760]	@ zero_extendqisi2
	bl	NandcSetMode
.L1711:
	ldr	r3, [r4, #1784]
	ldr	r6, .L1785+4
	ldrb	r3, [r3, #26]	@ zero_extendqisi2
	strb	r3, [r4, #1888]
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1709
	ldr	r3, [r6, #2772]
	cmp	r3, #0
	beq	.L1714
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	b	.L1780
.L1714:
	ldrb	r0, [r6, #2760]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r6, #2760]	@ zero_extendqisi2
.L1780:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1709
	mov	r0, #1
	ldr	r6, .L1785+4
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r3, [r4, #1784]
	ldr	r0, .L1785+48
	ldrh	r1, [r3, #14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	cmn	r0, #1
	beq	.L1755
	bl	FlashDieInfoInit
	ldr	r3, [r6, #1784]
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	bl	FlashGetReadRetryDefault
	movw	r3, #1820
	ldr	r2, [r6, #1784]
	ldrh	r3, [r6, r3]
	add	r3, r3, #4080
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	add	r3, r3, #15
	cmp	r1, r3, asr #12
	ldrh	r3, [r2, #14]
	blt	.L1716
	add	r0, r3, #255
	cmp	r1, r0, asr #8
	bge	.L1717
.L1716:
	bic	r3, r3, #255
	strh	r3, [r2, #14]	@ movhi
.L1717:
	ldrb	r3, [r4, #2760]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1718
	bl	FlashSavePhyInfo
	mov	r0, #0
	ldr	r1, [r8, #168]
	bl	FlashDdrParaScan
.L1718:
	bl	FlashSavePhyInfo
.L1709:
	ldr	r2, [r4, #1784]
	ldr	r7, .L1785+4
	ldr	r6, .L1785
	ldrb	r3, [r2, #26]	@ zero_extendqisi2
	ldrh	r0, [r2, #10]
	ldrb	r9, [r2, #18]	@ zero_extendqisi2
	strb	r3, [r4, #1888]
	ldrh	r3, [r2, #16]
	ubfx	r1, r3, #7, #1
	strb	r1, [r4, #8]
	ubfx	r1, r3, #3, #1
	strb	r1, [r8, #197]
	ubfx	r1, r3, #4, #1
	ubfx	r3, r3, #8, #3
	strb	r1, [r4, #2828]
	strb	r3, [r4, #2760]
	mov	r3, #0
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	str	r3, [r8, #160]
	bl	__aeabi_idiv
	mov	r1, r0
	mov	r0, r9
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	ldr	r3, [r4, #1784]
	ldrh	r2, [r3, #16]
	tst	r2, #64
	beq	.L1720
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	ldrb	r3, [r7, #1903]	@ zero_extendqisi2
	strb	r0, [r7, #1900]
	strb	r3, [r7, #1901]
	ldrb	r3, [r7, #1904]	@ zero_extendqisi2
	strb	r3, [r6, #177]
	sub	r3, r0, #1
	cmp	r3, #7
	bhi	.L1721
	ldr	r3, .L1785+52
	sub	r2, r0, #8
	clz	r2, r2
	str	r3, [r6, #160]
	sub	r3, r0, #5
	cmp	r0, #8
	cmpne	r3, #1
	mov	r2, r2, lsr #5
	ldr	r6, .L1785+56
	movls	r3, #1
	strls	r3, [r7, #2820]
	cmp	r0, #7
	beq	.L1724
	sub	r3, r6, #8
	cmp	r2, #0
	moveq	r6, r3
.L1724:
	sub	r2, r6, #1
	add	r6, r6, #31
	mov	r3, #0
.L1725:
	ldrsb	r1, [r2, #1]!
	cmp	r1, #0
	addeq	r3, r3, #1
	cmp	r2, r6
	bne	.L1725
	cmp	r3, #27
	bls	.L1720
	bl	FlashGetReadRetryDefault
	bl	FlashSavePhyInfo
	b	.L1720
.L1721:
	sub	r3, r0, #17
	cmp	r3, #2
	bhi	.L1727
	ldr	r3, .L1785+60
	cmp	r0, #19
	str	r3, [r6, #160]
	moveq	r3, #15
	bne	.L1784
	b	.L1782
.L1727:
	sub	r3, r0, #65
	cmp	r0, #33
	cmpne	r3, #1
	bhi	.L1729
	ldr	r3, .L1785+64
	str	r3, [r6, #160]
	mov	r3, #4
	strb	r3, [r7, #1901]
.L1784:
	mov	r3, #7
.L1782:
	strb	r3, [r6, #177]
	b	.L1720
.L1729:
	sub	r3, r0, #67
	sub	r2, r0, #34
	cmp	r3, #1
	movhi	r1, #0
	movls	r1, #1
	cmp	r3, #1
	cmphi	r2, #1
	bhi	.L1730
	ldr	r3, .L1785+64
	cmp	r0, #68
	cmpne	r0, #35
	str	r3, [r6, #160]
	movne	r3, #7
	moveq	r3, #17
	cmp	r1, #0
	strb	r3, [r6, #177]
	movne	r3, #4
	moveq	r3, #5
	strb	r3, [r4, #1901]
	b	.L1720
.L1730:
	cmp	r0, #49
	ldreq	r3, .L1785+68
	streq	r3, [r6, #160]
.L1720:
	ldr	r2, [r4, #2780]
	ldr	r3, .L1785+4
	cmp	r2, r5
	bne	.L1734
	ldrb	r1, [r3, #1888]	@ zero_extendqisi2
	cmp	r1, #0
	ldrne	r3, [r3, #1784]
	movne	r1, #0
	strneb	r1, [r3, #18]
.L1734:
	ldrb	r1, [r4, #1788]	@ zero_extendqisi2
	ldr	r3, .L1785+4
	cmp	r1, #44
	bne	.L1735
	ldrb	r1, [r3, #2768]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L1735
	cmp	r2, r5
	bne	.L1736
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1735
.L1736:
	mov	r0, #1
	mov	r3, #0
	strb	r3, [r4, #2768]
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
.L1735:
	ldrb	r3, [r4, #2760]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1737
	ldr	r2, .L1785+4
	ldrb	r2, [r2, #2768]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1738
	tst	r3, #1
	bne	.L1737
.L1738:
	mov	r0, #0
	bl	flash_enter_slc_mode
	mov	r0, #0
	ldr	r1, [r8, #168]
	bl	FlashDdrParaScan
	mov	r0, #0
	bl	flash_exit_slc_mode
.L1737:
	ldr	r3, [r4, #1784]
	mov	r8, #16
	ldr	r5, .L1785+72
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	ldr	r0, .L1785+76
	bl	FlashReadIdbDataRaw
	ldr	r0, .L1785+80
	strb	r8, [r4, #1]
	bl	FlashTimingCfg
	ldr	r6, [r4, #1784]
	ldrb	r3, [r6, #12]	@ zero_extendqisi2
	ldrh	r7, [r6, #14]
	strh	r3, [r5, #8]	@ movhi
	ldrb	r3, [r6, #7]	@ zero_extendqisi2
	str	r3, [r4, #1864]
	ldr	r3, [r4, #1788]
	str	r3, [r4, #1860]
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	strh	r3, [r5, #10]	@ movhi
	ldrb	r3, [r6, #13]	@ zero_extendqisi2
	strh	r7, [r5, #14]	@ movhi
	strh	r3, [r5, #12]	@ movhi
	ldrh	r3, [r6, #10]
	strh	r3, [r5, #16]	@ movhi
	ldrb	r1, [r6, #12]	@ zero_extendqisi2
	ldrh	r0, [r6, #10]
	bl	__aeabi_idiv
	strh	r0, [r5, #18]	@ movhi
	ldrb	r2, [r6, #9]	@ zero_extendqisi2
	strh	r2, [r5, #20]	@ movhi
	ldrb	r1, [r6, #9]	@ zero_extendqisi2
	ldrh	r3, [r6, #10]
	smulbb	r3, r1, r3
	mov	r1, #512
	strh	r1, [r5, #24]	@ movhi
	ldrb	r1, [r4, #1]	@ zero_extendqisi2
	strh	r1, [r5, #26]	@ movhi
	uxth	r3, r3
	ldrb	r1, [r4]	@ zero_extendqisi2
	strh	r3, [r5, #22]	@ movhi
	cmp	r1, #1
	bne	.L1740
	ldr	r1, .L1785+4
	mov	r3, r3, asl #1
	mov	r2, r2, asl #1
	strh	r3, [r5, #22]	@ movhi
	strh	r2, [r5, #20]	@ movhi
	mov	r3, #8
	strb	r8, [r1, #1]
	mov	r1, r7, lsr #1
	strh	r3, [r5, #26]	@ movhi
	strh	r1, [r5, #14]	@ movhi
.L1740:
	ldrb	r0, [r6, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	bl	ftl_flash_suspend
	mov	r0, #0
	b	.L1755
.L1742:
	mvn	r0, #1
.L1755:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1786:
	.align	2
.L1785:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+1788
	.word	.LC96
	.word	.LANCHOR0+1902
	.word	1446522928
	.word	.LANCHOR1+256
	.word	.LANCHOR1
	.word	.LANCHOR0+1792
	.word	.LANCHOR1+3256
	.word	.LANCHOR1+3132
	.word	.LANCHOR0+44
	.word	.LC97
	.word	HynixReadRetrial
	.word	.LANCHOR0+1930
	.word	MicronReadRetrial
	.word	ToshibaReadRetrial
	.word	SamsungReadRetrial
	.word	.LANCHOR0+1860
	.word	.LANCHOR2-1900
	.word	150000
	.fnend
	.size	FlashInit, .-FlashInit
	.align	2
	.global	FlashPageProgMsbFFData
	.type	FlashPageProgMsbFFData, %function
FlashPageProgMsbFFData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r2
	ldr	r3, .L1803
	mov	r8, r0
	mov	r9, r1
	ldr	r2, [r3, #1784]
	mov	r6, r3
	ldrb	r7, [r2, #19]	@ zero_extendqisi2
	ldrb	r2, [r3, #1888]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1788
	ldr	r3, [r3, #2764]
	cmp	r3, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1788:
	sub	r3, r7, #5
	cmp	r7, #68
	cmpne	r3, #3
	bls	.L1789
	sub	r3, r7, #19
	tst	r3, #239
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1789:
	ldr	r10, .L1803+4
	movw	fp, #65535
	ldr	r5, .L1803+8
.L1791:
	ldr	r3, [r6, #1784]
	ldrh	r3, [r3, #10]
	cmp	r3, r4
	bls	.L1802
	mov	r3, r4, asl #1
	ldrh	r3, [r10, r3]
	cmp	r3, fp
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
	cmp	r7, #8
	ldr	r0, [r5, #180]
	bne	.L1792
	mov	r1, #32768
	bl	__memzero
	b	.L1793
.L1792:
	mov	r1, #255
	mov	r2, #32768
	bl	memset
.L1793:
	ldr	r2, [r5, #180]
	add	r1, r4, r9
	add	r4, r4, #1
	mov	r0, r8
	mov	r3, r2
	uxth	r4, r4
	bl	FlashProgPage
	b	.L1791
.L1802:
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1804:
	.align	2
.L1803:
	.word	.LANCHOR0
	.word	.LANCHOR0+588
	.word	.LANCHOR2
	.fnend
	.size	FlashPageProgMsbFFData, .-FlashPageProgMsbFFData
	.align	2
	.global	FlashReadSlc2KPages
	.type	FlashReadSlc2KPages, %function
FlashReadSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1856
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #36
	sub	sp, sp, #36
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	mov	r4, r0
	mov	r8, #0
	str	r1, [sp, #16]
	str	r2, [sp, #20]
	str	r3, [sp, #12]
.L1806:
	ldr	r3, [sp, #16]
	cmp	r8, r3
	beq	.L1855
	ldr	r3, [sp, #16]
	add	r2, sp, #28
	mov	r0, r4
	ldr	r1, [sp, #20]
	rsb	r3, r8, r3
	ldr	r9, .L1856+4
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #24
	mov	fp, r9
	bl	LogAddr2PhyAddr
	ldrb	r2, [r9, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #24]
	cmp	r3, r2
	mvncs	r3, #0
	strcs	r3, [r4]
	bcs	.L1808
	add	r3, r9, r3
	mov	r7, #0
	ldrb	r5, [r3, #1616]	@ zero_extendqisi2
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	bl	NandcFlashCs
.L1809:
	ldr	r1, [sp, #28]
	mov	r0, r5
	bl	FlashReadCmd
	mov	r0, r5
	bl	NandcWaitFlashReady
	ldr	r3, [r4, #12]
	mov	r0, r5
	mov	r1, #0
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [r4, #8]
	bl	NandcXferData
	ldr	r3, .L1856+8
	ldrb	r3, [r3, #196]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r6, r0
	beq	.L1810
	mov	r0, r5
	bl	flash_read_ecc
	cmp	r0, #5
	movhi	r6, #256
.L1810:
	cmp	r7, #9
	cmnls	r6, #1
	moveq	r3, #1
	movne	r3, #0
	addeq	r7, r7, #1
	beq	.L1809
.L1811:
	cmp	r7, #0
	mov	r7, r3
	movne	r6, #256
.L1813:
	ldr	r3, [sp, #28]
	mov	r0, r5
	ldr	r1, [fp, #4]
	add	r1, r1, r3
	bl	FlashReadCmd
	mov	r0, r5
	bl	NandcWaitFlashReady
	ldr	r3, [r4, #8]
	ldr	r2, [r4, #12]
	mov	r0, r5
	cmp	r3, #0
	mov	r1, #0
	addne	r3, r3, #2048
	cmp	r2, #0
	addne	r2, r2, #8
	str	r2, [sp]
	ldr	r2, [sp, #12]
	bl	NandcXferData
	ldr	r3, .L1856+8
	ldrb	r2, [r3, #196]	@ zero_extendqisi2
	cmp	r2, #0
	mov	r10, r0
	beq	.L1816
	mov	r0, r5
	bl	flash_read_ecc
	cmp	r0, #5
	movhi	r10, #256
.L1816:
	cmp	r7, #9
	cmnls	r10, #1
	addeq	r7, r7, #1
	beq	.L1813
.L1817:
	cmp	r7, #0
	mov	r0, r5
	movne	r10, #256
	bl	NandcFlashDeCs
	ldrb	r2, [r9, #1889]	@ zero_extendqisi2
	cmp	r10, r6
	movcs	r3, r10
	movcc	r3, r6
	add	r2, r2, r2, asl #1
	cmp	r3, r2, asr #2
	bls	.L1819
	cmn	r3, #1
	movne	r3, #256
.L1819:
	cmp	r3, #256
	cmnne	r3, #1
	movne	r3, #0
	str	r3, [r4]
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1822
	ldr	r2, [r3, #12]
	cmn	r2, #1
	bne	.L1822
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L1822
	ldr	r3, [r3]
	cmn	r3, #1
	strne	r2, [r4]
.L1822:
	ldr	r3, [r4]
	cmn	r3, #1
	bne	.L1808
	ldr	r1, [r4, #4]
	ldr	r0, .L1856+12
	ldrb	r2, [r9, #1889]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r4, #8]
	cmp	r1, #0
	beq	.L1824
	ldr	r0, .L1856+16
	mov	r2, #4
	mov	r3, #8
	bl	rknand_print_hex
.L1824:
	ldr	r1, [r4, #12]
	cmp	r1, #0
	beq	.L1808
	mov	r2, #4
	ldr	r0, .L1856+20
	mov	r3, r2
	bl	rknand_print_hex
.L1808:
	add	r8, r8, #1
	add	r4, r4, #36
	b	.L1806
.L1855:
	mov	r0, #0
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1857:
	.align	2
.L1856:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC98
	.word	.LC99
	.word	.LC100
	.fnend
	.size	FlashReadSlc2KPages, .-FlashReadSlc2KPages
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1933
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldr	r6, .L1933+4
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	str	r1, [sp, #20]
	ldrb	r8, [r6]	@ zero_extendqisi2
	str	r3, [sp, #16]
	ldrb	r3, [r6, #8]	@ zero_extendqisi2
	cmp	r8, #0
	str	r2, [sp, #24]
	str	r3, [sp, #28]
	beq	.L1895
	bl	FlashReadSlc2KPages
	b	.L1928
.L1895:
	ldr	r10, .L1933+8
	mov	r9, r0
	mov	fp, r8
.L1859:
	ldr	r3, [sp, #20]
	cmp	fp, r3
	bcs	.L1931
	mov	r3, #36
	add	r2, sp, #44
	mul	r3, r3, fp
	ldr	r1, [sp, #24]
	add	r7, r9, r3
	str	r3, [sp, #8]
	mov	r0, r7
	ldr	r3, [r7, #4]
	str	r3, [sp, #12]
	ldr	r3, [sp, #20]
	rsb	r3, fp, r3
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #40
	bl	LogAddr2PhyAddr
	ldrb	r2, [r6, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #40]
	mov	r5, r0
	cmp	r3, r2
	ldrcs	r2, [sp, #8]
	mvncs	r3, #0
	strcs	r3, [r9, r2]
	bcs	.L1862
	add	r3, r6, r3
	ldrb	r4, [r3, #1616]	@ zero_extendqisi2
	ldrb	r3, [r10, #197]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r0, r4
	moveq	r5, #0
	bl	NandcWaitFlashReady
	ldr	r3, .L1933+4
	ldr	r3, [r3, #1784]
	ldrb	r2, [r3, #19]	@ zero_extendqisi2
	sub	r3, r2, #1
	cmp	r3, #7
	bhi	.L1864
	sub	r2, r2, #7
	add	r1, r6, r4
	cmp	r2, #1
	ldr	r2, .L1933+4
	ldrb	r3, [r1, #1914]	@ zero_extendqisi2
	add	r2, r2, r4
	ldrlsb	r3, [r1, #1922]	@ zero_extendqisi2
	ldrb	r2, [r2, #2756]	@ zero_extendqisi2
	cmp	r2, r3
	beq	.L1864
	mov	r0, r4
	ldrb	r1, [r6, #1903]	@ zero_extendqisi2
	ldr	r2, .L1933+12
	bl	HynixSetRRPara
.L1864:
	mov	r0, r4
	bl	NandcFlashCs
	mov	r0, r4
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #24]
	mov	r3, r3, lsr #31
	cmp	r2, #1
	orreq	r3, r3, #1
	str	r3, [sp, #12]
	cmp	r3, #0
	beq	.L1866
	ldr	r3, .L1933+4
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1866
	bl	flash_enter_slc_mode
	b	.L1867
.L1866:
	bl	flash_exit_slc_mode
.L1867:
	ldr	r1, [sp, #44]
	cmn	r1, #1
	cmpeq	r4, #255
	moveq	r3, #0
	movne	r3, #1
	moveq	r5, r3
	beq	.L1869
	cmp	r5, #0
	beq	.L1870
	ldr	r2, [r6, #4]
	mov	r0, r4
	add	r2, r1, r2
	bl	FlashReadDpCmd
	b	.L1871
.L1870:
	mov	r0, r4
	bl	FlashReadCmd
.L1871:
	mov	r0, r4
	bl	NandcWaitFlashReady
	cmp	r5, #0
	beq	.L1869
	mov	r0, r4
	ldr	r1, [sp, #44]
	bl	FlashReadDpDataOutCmd
.L1869:
	ldr	r3, [r7, #12]
	mov	r0, r4
	ldr	r2, [sp, #16]
	mov	r1, #0
	str	r3, [sp]
	ldr	r3, [r7, #8]
	bl	NandcXferData
	ldrb	r2, [r6, #8]	@ zero_extendqisi2
	adds	r2, r2, #0
	movne	r2, #1
	cmn	r0, #1
	mov	ip, r0
	movne	r2, #0
	cmp	r2, #0
	movne	r3, #0
	strneb	r3, [r6, #8]
	movne	r5, r3
	bne	.L1867
.L1872:
	cmp	r5, #0
	beq	.L1873
	ldr	r3, .L1933+4
	str	r0, [sp, #32]
	mov	r0, r4
	str	r2, [sp, #36]
	ldr	r1, [r3, #4]
	ldr	r3, [sp, #44]
	add	r1, r1, r3
	bl	FlashReadDpDataOutCmd
	mov	r0, r4
	ldr	r3, [sp, #8]
	ldr	r2, [sp, #36]
	add	r3, r3, #36
	add	r3, r9, r3
	ldr	r1, [r3, #12]
	str	r1, [sp]
	mov	r1, r2
	ldr	r3, [r3, #8]
	ldr	r2, [sp, #16]
	bl	NandcXferData
	cmn	r0, #1
	ldr	ip, [sp, #32]
	mov	r8, r0
	moveq	r5, #0
.L1873:
	mov	r0, r4
	str	ip, [sp, #32]
	bl	NandcFlashDeCs
	ldr	ip, [sp, #32]
	ldrb	r3, [sp, #28]	@ zero_extendqisi2
	cmn	ip, #1
	strb	r3, [r6, #8]
	bne	.L1880
	ldrb	r3, [r6, #2768]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1875
.L1879:
	ldr	r5, [r10, #160]
	cmp	r5, #0
	bne	.L1876
	b	.L1932
.L1875:
	ldr	r3, [r6, #1824]
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r5, [r3, #304]
	mov	r3, #1
	str	r3, [sp]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashDdrTunningRead
	cmn	r0, #1
	mov	ip, r0
	beq	.L1878
	ldrb	r3, [r6, #1889]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1898
.L1878:
	ubfx	r0, r5, #8, #8
	str	ip, [sp, #32]
	bl	NandcSetDdrPara
	ldr	ip, [sp, #32]
	cmn	ip, #1
	beq	.L1879
	b	.L1898
.L1876:
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	blx	r5
	cmn	r0, #1
	mov	ip, r0
	bne	.L1900
	ldr	r3, [r6, #1784]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	cmp	r3, #7
	bhi	.L1882
	mov	r0, r4
	ldrb	r1, [r6, #1903]	@ zero_extendqisi2
	ldr	r2, .L1933+12
	mov	r3, #0
	bl	HynixSetRRPara
.L1882:
	ldr	r1, [sp, #44]
	mov	r0, r4
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashReadRawPage
	ldr	r1, [r7, #4]
	ldrb	r2, [r6, #1889]	@ zero_extendqisi2
	mov	ip, r0
	ldr	r0, .L1933+16
	mov	r3, ip
	str	ip, [sp, #32]
	bl	printk
	ldr	ip, [sp, #32]
	cmn	ip, #1
	bne	.L1900
	ldrb	r5, [r6, #1888]	@ zero_extendqisi2
	cmp	r5, #0
	beq	.L1881
	ldr	r3, [sp, #12]
	mov	r0, r4
	cmp	r3, #0
	beq	.L1883
	bl	flash_enter_slc_mode
	b	.L1884
.L1883:
	bl	flash_exit_slc_mode
.L1884:
	ldr	ip, [r10, #160]
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	blx	ip
	mov	ip, r0
	b	.L1900
.L1932:
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashReadRawPage
	mov	ip, r0
	b	.L1881
.L1898:
	mov	r5, #0
.L1880:
	ldrb	r3, [r6, #1889]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	ip, r3, asr #2
	bls	.L1881
	ldr	r3, [r10, #160]
	cmp	r3, #0
	moveq	ip, #256
	b	.L1881
.L1900:
	mov	r5, #0
.L1881:
	cmp	ip, #256
	cmnne	ip, #1
	ldreq	r3, [sp, #8]
	movne	r3, #0
	ldrne	r2, [sp, #8]
	streq	ip, [r9, r3]
	strne	r3, [r9, r2]
	ldr	r3, [sp, #8]
	ldr	r3, [r9, r3]
	cmn	r3, #1
	bne	.L1888
	ldr	r1, [r7, #4]
	ldr	r0, .L1933+20
	ldrb	r2, [r6, #1889]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r7, #12]
	cmp	r1, #0
	beq	.L1888
	mov	r2, #4
	ldr	r0, .L1933+24
	mov	r3, r2
	bl	rknand_print_hex
.L1888:
	cmp	r5, #0
	beq	.L1890
	ldrb	r3, [r6, #1889]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r8, r3, asr #2
	bls	.L1891
	ldr	r3, [r10, #160]
	cmp	r3, #0
	moveq	r8, #256
.L1891:
	ldr	r3, [sp, #8]
	cmp	r8, #256
	cmnne	r8, #1
	add	r3, r3, #36
	movne	r2, #0
	streq	r8, [r9, r3]
	strne	r2, [r9, r3]
.L1890:
	ldr	r3, [sp, #12]
	add	fp, fp, r5
	cmp	r3, #0
	beq	.L1862
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1862
	mov	r0, r4
	bl	flash_exit_slc_mode
.L1862:
	add	fp, fp, #1
	b	.L1859
.L1931:
	mov	r0, #0
.L1928:
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1934:
	.align	2
.L1933:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+1906
	.word	.LC101
	.word	.LC98
	.word	.LC100
	.fnend
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FlashLoadFactorBbt
	.type	FlashLoadFactorBbt, %function
FlashLoadFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r1, #16
	ldr	r4, .L1948
	.pad #52
	sub	sp, sp, #52
	ldr	r8, .L1948+4
	mov	r5, #0
	ldr	fp, .L1948+8
	mov	r7, r5
	ldrh	r3, [r4, #14]
	add	r0, r8, #200
	ldrh	r6, [r4, #12]
	mvn	r10, #0
	smulbb	r6, r6, r3
	bl	__memzero
	ldr	r3, [r8, #184]
	uxth	r6, r6
	str	r5, [sp, #20]
	str	r3, [sp, #24]
.L1936:
	ldrb	r3, [fp, #1612]	@ zero_extendqisi2
	uxtb	r9, r5
	cmp	r3, r9
	bls	.L1947
	mul	ip, r6, r9
	sub	r4, r6, #1
	sub	r3, r6, #12
	uxth	r4, r4
.L1937:
	cmp	r4, r3
	ble	.L1939
	add	r2, ip, r4
	mov	r1, #1
	add	r0, sp, #12
	str	r3, [sp, #4]
	mov	r2, r2, asl #10
	str	r2, [sp, #16]
	mov	r2, r1
	str	ip, [sp]
	bl	FlashReadPages
	ldr	r2, [sp, #12]
	ldr	ip, [sp]
	cmn	r2, #1
	ldr	r3, [sp, #4]
	beq	.L1938
	ldr	r2, [r8, #184]
	ldrh	r1, [r2]
	movw	r2, #61664
	cmp	r1, r2
	bne	.L1938
	mov	r1, r9
	ldr	r0, .L1948+12
	mov	r2, r4
	add	r7, r7, #1
	bl	printk
	ldr	r3, .L1948+4
	uxth	r7, r7
	add	r9, r3, r9, asl #1
	strh	r4, [r9, #200]	@ movhi
	b	.L1939
.L1938:
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1937
.L1939:
	ldr	r3, .L1948+8
	add	r5, r5, #1
	ldrb	r3, [r3, #1612]	@ zero_extendqisi2
	cmp	r3, r7
	moveq	r10, #0
	b	.L1936
.L1947:
	mov	r0, r10
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1949:
	.align	2
.L1948:
	.word	.LANCHOR0+1860
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC102
	.fnend
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.align	2
	.global	FlashProgSlc2KPages
	.type	FlashProgSlc2KPages, %function
FlashProgSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #68
	sub	sp, sp, #68
	ldr	r8, .L1981
	mov	r6, r1
	str	r3, [sp, #12]
	mov	r9, r2
	ldr	r3, .L1981+4
	mov	r4, r0
	mov	r10, r0
	mov	r7, #0
	mov	fp, r8
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	str	r3, [sp, #8]
.L1951:
	cmp	r7, r6
	beq	.L1980
	rsb	r3, r7, r6
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
	mvncs	r3, #0
	strcs	r3, [r10]
	bcs	.L1953
	add	r3, r8, r3
	ldrb	r5, [r3, #1616]	@ zero_extendqisi2
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	bl	NandcFlashCs
	mov	r0, r5
	ldr	r1, [sp, #20]
	bl	FlashProgFirstCmd
	ldr	r3, [r10, #12]
	mov	r1, #1
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
	ldr	r2, [r10, #12]
	mov	r1, #1
	cmp	r3, #0
	mov	r0, r5
	addne	r3, r3, #2048
	cmp	r2, #0
	addne	r2, r2, #8
	str	r2, [sp]
	ldr	r2, [sp, #8]
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
	tst	r0, #1
	mov	r0, r5
	mvnne	r3, #0
	strne	r3, [r10]
	bl	NandcFlashDeCs
.L1953:
	add	r7, r7, #1
	add	r10, r10, #36
	b	.L1951
.L1980:
	ldr	r3, [sp, #12]
	cmp	r3, #0
	beq	.L1978
	ldr	r5, .L1981+8
	mov	r7, #0
	mov	r8, r5
.L1958:
	cmp	r7, r6
	beq	.L1978
	ldr	r3, [r4]
	cmn	r3, #1
	bne	.L1959
	ldr	r1, [r4, #4]
	ldr	r0, .L1981+12
	bl	printk
	b	.L1960
.L1959:
	rsb	r3, r7, r6
	mov	r1, r9
	add	r2, sp, #20
	mov	r0, r4
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #24
	bl	LogAddr2PhyAddr
	ldr	r2, [r5, #188]
	mov	r3, #0
	mov	lr, r4
	str	r3, [r2]
	ldr	r2, [r5, #192]
	str	r3, [r2]
	ldmia	lr!, {r0, r1, r2, r3}
	add	ip, sp, #28
	stmia	ip!, {r0, r1, r2, r3}
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	add	r0, sp, #28
	ldr	r3, [lr]
	mov	r1, #1
	mov	r2, r9
	str	r3, [ip]
	ldr	r3, [r5, #188]
	str	r3, [sp, #36]
	ldr	r3, [r5, #192]
	str	r3, [sp, #40]
	bl	FlashReadPages
	ldr	r10, [sp, #28]
	cmn	r10, #1
	bne	.L1961
	ldr	r0, .L1981+16
	ldr	r1, [r4, #4]
	bl	printk
	str	r10, [r4]
.L1961:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1962
	ldr	r2, [r3]
	ldr	r3, [r8, #192]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1962
	ldr	r0, .L1981+20
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L1962:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L1960
	ldr	r2, [r3]
	ldr	r3, [r8, #188]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1960
	ldr	r0, .L1981+24
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L1960:
	add	r7, r7, #1
	add	r4, r4, #36
	b	.L1958
.L1978:
	mov	r0, #0
	add	sp, sp, #68
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1982:
	.align	2
.L1981:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LC103
	.word	.LC104
	.word	.LC105
	.word	.LC106
	.fnend
	.size	FlashProgSlc2KPages, .-FlashProgSlc2KPages
	.align	2
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r7, #0
	ldr	r5, .L1995
	ldr	r6, .L1995+4
	ldr	r3, [r5, #-2060]
	sub	r9, r6, #72
	ldr	r8, [r5, #-2036]
	sub	r10, r6, #116
	mov	fp, r9
	str	r3, [r5, #224]
	str	r8, [r5, #228]
.L1984:
	ldrh	r3, [r10]
	cmp	r7, r3
	bcs	.L1994
	ldrh	r4, [r9]
	mvn	r3, #0
	movw	ip, #61664
	strh	r3, [r6, #2]!	@ movhi
	add	r4, r4, r3
	uxth	r4, r4
.L1985:
	ldrh	r3, [fp]
	sub	r2, r3, #16
	cmp	r4, r2
	ble	.L1987
	mla	r3, r3, r7, r4
	mov	r1, #1
	ldr	r0, .L1995+8
	mov	r2, r1
	str	ip, [sp, #4]
	mov	r3, r3, asl #10
	str	r3, [r5, #220]
	bl	FlashReadPages
	ldr	r3, [r5, #216]
	cmn	r3, #1
	ldr	ip, [sp, #4]
	beq	.L1986
	ldrh	r3, [r8]
	cmp	r3, ip
	streqh	r4, [r6]	@ movhi
	beq	.L1987
.L1986:
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1985
.L1987:
	add	r7, r7, #1
	b	.L1984
.L1994:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1996:
	.align	2
.L1995:
	.word	.LANCHOR2
	.word	.LANCHOR0+2978
	.word	.LANCHOR2+216
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #1
	ldr	r3, .L2010
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movweq	r2, #2910
	movwne	r2, #2908
	.pad #104
	sub	sp, sp, #104
	ldrh	r4, [r3, r2]
	mov	r8, r1
	add	r3, sp, #40
	mov	r6, r0, asl #10
	sub	r5, r4, #1
	str	r3, [sp, #16]
	add	r0, sp, #4
	mov	r1, #1
	uxth	r5, r5
	mov	r2, r8
	mov	r7, #0
	str	r7, [sp, #12]
	sxth	r3, r5
	orr	r3, r3, r6
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	cmn	r3, #1
	bne	.L2000
.L2001:
	sxth	r4, r7
	sxth	r3, r5
	cmp	r4, r3
	bgt	.L2000
	add	r4, r4, r3
	add	r0, sp, #4
	mov	r1, #1
	mov	r2, r8
	add	r4, r4, r4, lsr #31
	mov	r4, r4, asr #1
	sxth	r3, r4
	orr	r3, r3, r6
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	cmn	r3, #1
	bne	.L2002
	ldr	r3, [sp, #44]
	cmn	r3, #1
	bne	.L2002
	ldr	r3, [sp, #4]
	cmn	r3, #1
	subne	r4, r4, #1
	uxthne	r5, r4
	bne	.L2001
.L2002:
	add	r3, r4, #1
	uxth	r7, r3
	b	.L2001
.L2000:
	sxth	r0, r5
	add	sp, sp, #104
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2011:
	.align	2
.L2010:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlLoadBbt
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	ldr	r4, .L2045
	ldr	r7, .L2045+4
	ldr	r8, .L2045+8
	ldr	r3, [r4, #-2060]
	mov	r9, r4
	ldr	r6, [r4, #-2036]
	str	r3, [r4, #224]
	str	r6, [r4, #228]
	bl	FtlBbtMemInit
	movw	r3, #2906
	ldrh	r5, [r7, r3]
	sub	r5, r5, #1
	uxth	r5, r5
.L2013:
	ldrh	r3, [r8]
	sub	r3, r3, #48
	cmp	r5, r3
	ble	.L2016
	mov	r1, #1
	ldr	r0, .L2045+12
	mov	r2, r1
	mov	r3, r5, asl #10
	str	r3, [r4, #220]
	bl	FlashReadPages
	ldr	r3, [r4, #216]
	cmn	r3, #1
	bne	.L2014
	ldr	r3, [r9, #220]
	mov	r1, #1
	ldr	r0, .L2045+12
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r9, #220]
	bl	FlashReadPages
.L2014:
	ldr	r3, [r4, #216]
	cmn	r3, #1
	beq	.L2015
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L2015
	movw	r3, #2968
	strh	r5, [r7, r3]	@ movhi
	ldr	r3, [r6, #4]
	str	r3, [r7, #2976]
	ldr	r3, .L2045+16
	ldrh	r2, [r6, #8]
	strh	r2, [r3, #4]	@ movhi
	b	.L2016
.L2015:
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L2013
.L2016:
	movw	r9, #2968
	movw	r2, #65535
	ldrh	r3, [r7, r9]
	ldr	r8, .L2045+4
	cmp	r3, r2
	ldr	r5, .L2045+16
	beq	.L2030
	ldrh	r3, [r5, #4]
	cmp	r3, r2
	beq	.L2020
	mov	r1, #1
	ldr	r0, .L2045+12
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [r4, #220]
	bl	FlashReadPages
	ldr	r3, [r4, #216]
	cmn	r3, #1
	beq	.L2020
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L2020
	ldr	r3, [r6, #4]
	ldr	r2, [r8, #2976]
	cmp	r3, r2
	strhi	r3, [r8, #2976]
	ldrhih	r2, [r5, #4]
	ldrhih	r3, [r6, #8]
	strhih	r2, [r8, r9]	@ movhi
	strhih	r3, [r5, #4]	@ movhi
.L2020:
	movw	r3, #2968
	mov	r1, #1
	ldrh	r0, [r7, r3]
	movw	r8, #61649
	bl	FtlGetLastWrittenPage
	uxth	r7, r0
	add	r0, r0, #1
	strh	r0, [r5, #2]	@ movhi
.L2022:
	sxth	r3, r7
	cmp	r3, #0
	blt	.L2027
	ldrh	r2, [r5]
	mov	r1, #1
	ldr	r0, .L2045+12
	orr	r3, r3, r2, asl #10
	str	r3, [r4, #220]
	ldr	r3, [r4, #-2060]
	mov	r2, r1
	str	r3, [r4, #224]
	bl	FlashReadPages
	ldr	r3, [r4, #216]
	cmn	r3, #1
	beq	.L2023
	ldrh	r3, [r6]
	cmp	r3, r8
	bne	.L2023
.L2027:
	ldrh	r2, [r6, #10]
	ldrh	r0, [r6, #12]
	ldr	r3, .L2045+4
	strh	r2, [r5, #6]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	bne	.L2024
	b	.L2025
.L2023:
	sub	r7, r7, #1
	uxth	r7, r7
	b	.L2022
.L2024:
	ldr	r2, [r3, #2836]
	cmp	r0, r2
	beq	.L2025
	movw	r1, #2850
	ldrh	r3, [r3, r1]
	mov	r3, r3, lsr #2
	cmp	r0, r3
	cmpcc	r2, r3
	bcs	.L2025
	bl	FtlSysBlkNumInit
.L2025:
	ldr	r6, .L2045+20
	mov	r5, #0
	ldr	r8, .L2045+24
	sub	r7, r6, #130
.L2028:
	ldrh	r3, [r7]
	cmp	r5, r3
	bcs	.L2044
	ldrh	r2, [r8]
	ldr	r1, [r4, #224]
	ldr	r0, [r6, #4]!
	mov	r2, r2, asl #2
	mla	r1, r5, r2, r1
	bl	ftl_memcpy
	add	r5, r5, #1
	b	.L2028
.L2044:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L2030:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L2046:
	.align	2
.L2045:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+2906
	.word	.LANCHOR2+216
	.word	.LANCHOR0+2968
	.word	.LANCHOR0+2992
	.word	.LANCHOR2-1972
	.fnend
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, #0
	ldr	r5, .L2133
	mov	r1, fp
	ldr	r10, .L2133+4
	.pad #36
	sub	sp, sp, #36
	add	r7, r5, #2928
	ldr	r2, [r5, #2936]
	sub	r3, r10, #1920
	ldr	r0, [r10, #-1980]
	mov	r4, r10
	strh	fp, [r3, #-4]	@ movhi
	movw	r3, #2950
	mov	r2, r2, asl #2
	strh	fp, [r5, r3]	@ movhi
	bl	ftl_memset
	ldr	r2, [r5, #2936]
	mov	r1, fp
	ldr	r0, [r10, #-2008]
	mov	r8, r10
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r7]
	mov	r1, fp
	ldr	r0, [r10, #-1996]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r7]
	mov	r1, fp
	ldr	r0, [r5, #2952]
	mov	r2, r2, asl #1
	bl	ftl_memset
	sub	r0, r10, #2784
	mov	r1, #255
	mov	r2, #16
	bl	memset
	add	r3, r5, #2848
	ldrh	r3, [r3]
	str	r5, [sp, #12]
	str	r3, [sp, #4]
	str	r7, [sp, #16]
.L2048:
	ldr	r3, .L2133+8
	ldr	r2, [sp, #4]
	ldr	r0, .L2133
	ldrh	r3, [r3]
	cmp	r3, r2
	bls	.L2089
	ldr	r3, .L2133+12
	mov	r5, #0
	ldr	r1, .L2133+16
	mov	r7, r5
	ldr	r2, [r4, #-2072]
	mov	r6, #36
	ldrh	ip, [r3]
	sub	r9, r1, #52
	ldr	r3, [r4, #-2180]
	ldr	fp, [r4, #-2176]
	ldrh	r10, [r1]
.L2090:
	uxth	r1, r5
	cmp	r1, ip
	bcs	.L2128
	ldr	r1, [sp, #4]
	ldrb	r0, [r9, r5]	@ zero_extendqisi2
	str	r3, [sp, #28]
	str	r2, [sp, #24]
	str	ip, [sp, #20]
	bl	V2P_block
	str	r0, [sp, #8]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldr	r1, [sp, #8]
	ldr	ip, [sp, #20]
	ldr	r2, [sp, #24]
	ldr	r3, [sp, #28]
	bne	.L2049
	mla	r0, r6, r7, r2
	mov	r1, r1, asl #10
	stmib	r0, {r1, r3}
	mul	r1, r10, r7
	add	r7, r7, #1
	uxth	r7, r7
	add	lr, r1, #3
	cmp	r1, #0
	movlt	r1, lr
	bic	r1, r1, #3
	add	r1, fp, r1
	str	r1, [r0, #12]
.L2049:
	add	r5, r5, #1
	b	.L2090
.L2128:
	cmp	r7, #0
	beq	.L2052
	ldr	r0, [r4, #-2072]
	mov	r1, r7
	mov	r2, #1
	bl	FlashReadPages
	mov	r3, #0
.L2126:
	str	r3, [sp, #8]
	ldrh	r3, [sp, #8]
	cmp	r3, r7
	bcs	.L2052
	ldr	r3, [sp, #8]
	mov	r9, #36
	mul	r9, r9, r3
	ldr	r3, [r4, #-2072]
	add	r2, r3, r9
	ldr	r3, [r3, r9]
	ldr	r5, [r2, #4]
	cmn	r3, #1
	ldr	r6, [r2, #12]
	ubfx	r5, r5, #10, #16
	bne	.L2055
	mov	r10, #16
	movw	fp, #65535
.L2054:
	ldr	r0, [r4, #-2072]
	mov	r1, #1
	mov	r2, r1
	add	r0, r0, r9
	ldr	r3, [r0, #4]
	add	r3, r3, #1
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldrh	r3, [r6]
	cmp	r3, fp
	ldreq	r3, .L2133+4
	mvneq	r2, #0
	ldreq	r3, [r3, #-2072]
	streq	r2, [r3, r9]
	beq	.L2055
.L2056:
	ldr	r3, [r8, #-2072]
	ldr	r3, [r3, r9]
	cmn	r3, #1
	bne	.L2055
	sub	r10, r10, #1
	uxth	r10, r10
	cmp	r10, #0
	bne	.L2054
.L2055:
	ldr	r3, [r8, #-2072]
	ldr	r3, [r3, r9]
	ldr	r9, .L2133+4
	cmn	r3, #1
	beq	.L2058
	ldr	r2, [r8, #-2816]
	ldr	r3, [r6, #4]
	cmn	r2, #1
	beq	.L2059
	cmp	r2, r3
	bhi	.L2060
.L2059:
	cmn	r3, #1
	addne	r2, r3, #1
	strne	r2, [r9, #-2816]
.L2060:
	ldrh	r2, [r6]
	movw	r1, #61604
	cmp	r2, r1
	beq	.L2062
	bhi	.L2063
	movw	r3, #61574
	cmp	r2, r3
	bne	.L2061
	ldr	r3, [sp, #16]
	ldr	r2, .L2133+20
	ldr	r9, [r9, #-1996]
	ldrh	ip, [r3]
	ldrh	r2, [r2]
	sub	r0, ip, #1
	uxth	r3, r0
	rsb	r0, r2, r0
	b	.L2076
.L2063:
	movw	r3, #61634
	cmp	r2, r3
	beq	.L2065
	movw	r3, #65535
	cmp	r2, r3
	moveq	r0, r5
	beq	.L2127
	b	.L2061
.L2065:
	ldr	r3, [sp, #12]
	ldr	fp, .L2133+24
	ldr	lr, [r4, #-1980]
	ldr	ip, [r3, #2936]
	ldrh	r2, [fp]
	uxth	r1, ip
	sub	r3, r1, #1
	rsb	r1, r2, r1
	sub	r1, r1, #1
	uxth	r3, r3
	sxth	r1, r1
	str	r1, [sp, #20]
.L2067:
	ldr	r1, [sp, #20]
	sxth	r0, r3
	cmp	r0, r1
	ble	.L2129
	ldr	r10, [lr, r0, asl #2]
	mov	r9, r0, asl #2
	ldr	r1, [r6, #4]
	cmp	r1, r10
	bls	.L2068
	ldr	r1, [lr]
	cmp	r1, #0
	bne	.L2069
	cmp	r2, ip
	addne	r2, r2, #1
	ldrne	r1, .L2133+24
	strneh	r2, [r1]	@ movhi
.L2069:
	uxth	lr, r3
	mov	r1, #0
.L2070:
	uxth	r2, r1
	cmp	r2, lr
	bcs	.L2130
	ldr	ip, [r8, #-1980]
	sxth	r2, r2
	add	r1, r1, #1
	add	r10, ip, r2, asl #2
	ldr	r10, [r10, #4]
	str	r10, [ip, r2, asl #2]
	mov	r2, r2, asl #1
	ldr	ip, [r8, #-2008]
	add	r10, ip, r2
	ldrh	r10, [r10, #2]
	strh	r10, [ip, r2]	@ movhi
	b	.L2070
.L2130:
	ldr	r2, [r4, #-1980]
	mov	r0, r0, asl #1
	ldr	r1, [r6, #4]
	str	r1, [r2, r9]
	ldr	r2, [r4, #-2008]
	strh	r5, [r2, r0]	@ movhi
	sxth	r0, r3
	cmp	r0, #0
	bge	.L2072
	b	.L2061
.L2068:
	sub	r3, r3, #1
	uxth	r3, r3
	b	.L2067
.L2129:
	cmp	r0, #0
	bge	.L2103
	b	.L2061
.L2072:
	ldr	r2, .L2133+24
	ldr	r1, .L2133
	ldrh	r2, [r2]
	ldr	r1, [r1, #2936]
	rsb	r1, r2, r1
	sub	r1, r1, #1
	sxth	r1, r1
	cmp	r0, r1
	bgt	.L2061
.L2103:
	add	r2, r2, #1
	ldr	r1, [r6, #4]
	strh	r2, [fp]	@ movhi
	sxth	r3, r3
	ldr	r2, [r4, #-1980]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #-2008]
	b	.L2124
.L2082:
	ldr	fp, [r6, #4]
	mov	lr, r1, asl #2
	ldr	r10, [r9, r1, asl #2]
	cmp	fp, r10
	bhi	.L2131
	sub	r3, r3, #1
	uxth	r3, r3
.L2076:
	sxth	r1, r3
	cmp	r1, r0
	bgt	.L2082
	b	.L2081
.L2131:
	ldr	r0, [r9]
	cmp	r0, #0
	bne	.L2078
	cmp	r2, ip
	addne	r2, r2, #1
	ldrne	r0, .L2133+20
	strneh	r2, [r0]	@ movhi
.L2078:
	ldr	r10, .L2133
	uxth	r9, r3
	mov	r0, #0
.L2079:
	uxth	r2, r0
	cmp	r2, r9
	bcs	.L2132
	ldr	ip, [r4, #-1996]
	sxth	r2, r2
	add	r0, r0, #1
	add	fp, ip, r2, asl #2
	ldr	fp, [fp, #4]
	str	fp, [ip, r2, asl #2]
	mov	r2, r2, asl #1
	ldr	ip, [r10, #2952]
	add	fp, ip, r2
	ldrh	fp, [fp, #2]
	strh	fp, [ip, r2]	@ movhi
	b	.L2079
.L2132:
	ldr	r2, [r8, #-1996]
	mov	r1, r1, asl #1
	ldr	r0, [r6, #4]
	str	r0, [r2, lr]
	ldr	r2, .L2133
	ldr	r2, [r2, #2952]
	strh	r5, [r2, r1]	@ movhi
.L2081:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L2061
	ldr	r0, .L2133
	ldr	ip, .L2133+20
	add	r2, r0, #2928
	ldrh	r2, [r2]
	ldrh	r1, [ip]
	sub	r2, r2, #1
	rsb	r2, r1, r2
	sxth	r2, r2
	cmp	r3, r2
	bgt	.L2061
	add	r1, r1, #1
	ldr	r2, [r8, #-1996]
	strh	r1, [ip]	@ movhi
	ldr	r1, [r6, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r0, #2952]
.L2124:
	strh	r5, [r2, r3]	@ movhi
	b	.L2061
.L2062:
	ldr	r2, .L2133+28
	ldrh	r1, [r2]
	movw	r2, #65535
	cmp	r1, r2
	ldreq	r2, .L2133+28
	streqh	r5, [r2]	@ movhi
	beq	.L2125
	ldr	r3, .L2133+28
	ldrh	r0, [r3, #4]
	cmp	r0, r2
	beq	.L2084
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L2084:
	ldr	r3, [r6, #4]
	ldr	r2, [r9, #-2776]
	cmp	r2, r3
	ldr	r3, .L2133+28
	strcsh	r5, [r3, #4]	@ movhi
	bcs	.L2061
	ldrh	r3, [r3]
	ldr	r2, .L2133+28
	strh	r3, [r2, #4]	@ movhi
	ldr	r3, [r6, #4]
	strh	r5, [r2]	@ movhi
.L2125:
	str	r3, [r8, #-2776]
	b	.L2061
.L2058:
	ldr	r3, .L2133
	mov	r0, r5
	ldrb	r1, [r3, #1888]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L2086
.L2127:
	mov	r1, #0
.L2086:
	bl	FtlFreeSysBlkQueueIn
.L2061:
	ldr	r3, [sp, #8]
	add	r3, r3, #1
	b	.L2126
.L2052:
	ldr	r3, [sp, #4]
	add	r7, r3, #1
	uxth	r3, r7
	str	r3, [sp, #4]
	b	.L2048
.L2089:
	ldr	r3, .L2133+4
	ldr	ip, [r3, #-2008]
	ldrh	r2, [ip]
	cmp	r2, #0
	beq	.L2091
.L2094:
	ldr	ip, [r0, #2952]
	ldr	r1, .L2133
	ldrh	r2, [ip]
	cmp	r2, #0
	beq	.L2092
	b	.L2115
.L2091:
	sub	r3, r3, #1920
	ldrh	r3, [r3, #-4]
	cmp	r3, #0
	ldrne	lr, [r0, #2936]
	beq	.L2094
.L2095:
	uxth	r3, r2
	sxth	r1, r3
	cmp	r1, lr
	bcs	.L2094
	mov	r5, r1, asl #1
	add	r2, r2, #1
	ldrh	r5, [ip, r5]
	cmp	r5, #0
	beq	.L2095
	mov	r6, #0
.L2096:
	ldr	ip, [r0, #2936]
	sxth	r2, r3
	cmp	r2, ip
	bcs	.L2094
	ldr	lr, [r4, #-2008]
	mov	ip, r2, asl #1
	rsb	r5, r1, r2
	add	r3, r3, #1
	ldrh	r8, [lr, ip]
	mov	r7, r5, asl #1
	uxth	r3, r3
	strh	r8, [lr, r7]	@ movhi
	ldr	lr, [r4, #-1980]
	ldr	r2, [lr, r2, asl #2]
	str	r2, [lr, r5, asl #2]
	ldr	r2, [r4, #-2008]
	strh	r6, [r2, ip]	@ movhi
	b	.L2096
.L2092:
	movw	r3, #2950
	ldrh	r3, [r1, r3]
	cmp	r3, #0
	addne	r1, r1, #2928
	ldrneh	lr, [r1]
	beq	.L2115
.L2099:
	uxth	r3, r2
	sxth	r0, r3
	cmp	r0, lr
	bge	.L2115
	mov	r5, r0, asl #1
	add	r2, r2, #1
	ldrh	r5, [ip, r5]
	cmp	r5, #0
	beq	.L2099
	ldr	ip, .L2133
	mov	r7, #0
.L2100:
	ldrh	lr, [r1]
	sxth	r2, r3
	cmp	r2, lr
	bge	.L2115
	ldr	r5, [ip, #2952]
	mov	lr, r2, asl #1
	rsb	r6, r0, r2
	add	r3, r3, #1
	ldrh	r9, [r5, lr]
	mov	r8, r6, asl #1
	uxth	r3, r3
	strh	r9, [r5, r8]	@ movhi
	ldr	r5, [r4, #-1996]
	ldr	r2, [r5, r2, asl #2]
	str	r2, [r5, r6, asl #2]
	ldr	r2, [ip, #2952]
	strh	r7, [r2, lr]	@ movhi
	b	.L2100
.L2115:
	mov	r0, #0
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2134:
	.align	2
.L2133:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2850
	.word	.LANCHOR0+2840
	.word	.LANCHOR0+2920
	.word	.LANCHOR0+2950
	.word	.LANCHOR2-1924
	.word	.LANCHOR2-2784
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	FtlLoadSysInfo
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r1, #0
	ldr	r4, .L2164
	ldr	r6, .L2164+4
	sub	r7, r4, #2784
	ldr	r3, [r4, #-2060]
	ldrh	r2, [r6]
	ldr	r0, [r4, #-3028]
	str	r3, [r4, #224]
	ldr	r3, [r4, #-2036]
	mov	r2, r2, asl #1
	str	r3, [r4, #228]
	bl	ftl_memset
	ldrh	r0, [r7]
	movw	r3, #65535
	cmp	r0, r3
	bne	.L2136
.L2147:
	mvn	r0, #0
	b	.L2137
.L2136:
	mov	r1, #1
	mov	r5, r4
	bl	FtlGetLastWrittenPage
	ldrh	r9, [r7]
	mov	fp, r5
	uxth	r4, r0
	add	r0, r0, #1
	strh	r0, [r7, #2]	@ movhi
.L2138:
	sxth	r3, r4
	ldr	r8, .L2164+8
	cmp	r3, #0
	blt	.L2146
	sxth	r10, r9
	mov	r1, #1
	ldr	r0, .L2164+12
	mov	r2, r1
	orr	r3, r3, r10, asl #10
	str	r3, [r5, #220]
	ldr	r3, [r5, #-2060]
	str	r3, [r5, #224]
	bl	FlashReadPages
	ldrb	r3, [r8]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2139
	ldr	r3, [fp, #228]
	ldr	r3, [r3, #12]
	cmp	r3, #0
	beq	.L2139
	ldr	r2, .L2164+16
	ldr	r0, [fp, #224]
	str	r3, [sp, #4]
	ldrh	r1, [r2]
	bl	js_hash
	ldr	r3, [sp, #4]
	cmp	r3, r0
	beq	.L2139
	cmp	r4, #0
	bne	.L2140
	ldrh	r3, [r7, #4]
	cmp	r10, r3
	uxthne	r9, r3
	ldrne	r3, .L2164+20
	ldrneh	r4, [r3]
	bne	.L2142
.L2140:
	mvn	r3, #0
	str	r3, [r5, #216]
.L2139:
	ldr	r3, [r5, #216]
	cmn	r3, #1
	beq	.L2142
	ldr	r3, [fp, #-2060]
	ldr	r2, .L2164+24
	ldr	r3, [r3]
	cmp	r3, r2
	bne	.L2142
	ldr	r3, [fp, #-2036]
	ldrh	r2, [r3]
	movw	r3, #61604
	cmp	r2, r3
	bne	.L2142
.L2146:
	ldr	r1, [r5, #224]
	mov	r2, #48
	ldr	r0, .L2164+28
	bl	ftl_memcpy
	ldrh	r2, [r6]
	ldr	r1, [r5, #224]
	ldr	r0, [r5, #-3028]
	add	r1, r1, #48
	mov	r2, r2, asl #1
	bl	ftl_memcpy
	ldrh	r2, [r6]
	ldr	r1, [r5, #224]
	add	r3, r2, #24
	ldr	r0, [r5, #-2852]
	ubfx	r3, r3, #1, #30
	mov	r2, r2, lsr #3
	add	r2, r2, #4
	ldr	r4, .L2164
	add	r1, r1, r3, asl #2
	bl	ftl_memcpy
	movw	r3, #2948
	ldrh	r3, [r8, r3]
	cmp	r3, #0
	beq	.L2144
	ldr	r2, .L2164+4
	ldr	r0, [r4, #-1984]
	ldrh	r1, [r2], #96
	mov	r3, r1, lsr #3
	ldrh	r2, [r2]
	add	r3, r3, r1, asl #1
	ldr	r1, [r4, #224]
	add	r3, r3, #52
	ubfx	r3, r3, #2, #14
	mov	r2, r2, asl #2
	add	r1, r1, r3, asl #2
	bl	ftl_memcpy
	b	.L2144
.L2142:
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L2138
.L2144:
	ldr	r2, [r5, #-3084]
	ldr	r3, .L2164+24
	ldr	r4, .L2164
	cmp	r2, r3
	bne	.L2147
	ldr	r6, .L2164+28
	movw	r1, #2862
	ldrb	r0, [r4, #-3074]	@ zero_extendqisi2
	sub	r9, r4, #2784
	ldrh	r1, [r8, r1]
	ldrh	r2, [r6, #8]
	cmp	r0, r1
	ldr	r3, .L2164+8
	strh	r2, [r7, #6]	@ movhi
	bne	.L2147
	movw	r1, #2908
	movw	r0, #2914
	ldrh	r1, [r3, r1]
	ldrh	r0, [r3, r0]
	str	r2, [r4, #252]
	mul	r1, r2, r1
	str	r1, [r3, #2964]
	mul	r1, r0, r1
	str	r1, [r3, #1892]
	ldr	r1, .L2164+32
	ldrh	r0, [r1, #6]
	ldr	r1, [r3, #2852]
	rsb	r0, r0, r1
	rsb	r0, r2, r0
	movw	r2, #2840
	ldrh	r1, [r3, r2]
	bl	__aeabi_uidiv
	ldrh	r3, [r6, #16]
	ldrh	ip, [r6, #14]
	sub	r2, r4, #3008
	sub	lr, r4, #2768
	mov	r1, r3, lsr #6
	strh	r1, [r2, #2]	@ movhi
	ldrh	r1, [r6, #18]
	and	r3, r3, #63
	strh	ip, [r2]	@ movhi
	sub	r2, r4, #2960
	strb	r3, [r4, #-3002]
	mov	r8, lr
	strh	r1, [r2]	@ movhi
	ldrh	r1, [r6, #20]
	ldrb	r3, [r4, #-3073]	@ zero_extendqisi2
	strb	r3, [r4, #-3000]
	mvn	r3, #0
	strh	r3, [lr]	@ movhi
	mov	r3, #0
	strh	r3, [lr, #2]	@ movhi
	strb	r3, [r4, #-2762]
	strb	r3, [r4, #-2760]
	str	r3, [r4, #-2828]
	strh	r0, [r9, #-4]	@ movhi
	mov	r0, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-2954]
	ldrb	r1, [r4, #-3072]	@ zero_extendqisi2
	strh	r0, [r2, #2]	@ movhi
	sub	r0, r4, #2912
	strb	r1, [r4, #-2952]
	mov	r7, r0
	ldrh	r1, [r6, #22]
	strh	r1, [r0]	@ movhi
	ldrh	r1, [r6, #24]
	mov	r6, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-2906]
	ldrb	r1, [r4, #-3071]	@ zero_extendqisi2
	strh	r6, [r0, #2]	@ movhi
	mov	r6, r2
	strb	r1, [r4, #-2904]
	str	r3, [r4, #-2840]
	ldr	r1, [r4, #-3052]
	str	r3, [r4, #-2848]
	str	r3, [r4, #-2832]
	str	r3, [r4, #-2804]
	str	r3, [r4, #-2796]
	str	r3, [r4, #-2836]
	ldr	r3, [r4, #-3044]
	str	r1, [r4, #-2808]
	ldr	r1, [r4, #-2816]
	ldr	r2, [r5, #-2812]
	cmp	r3, r1
	strhi	r3, [r4, #-2816]
	ldr	r3, [r5, #-3048]
	cmp	r3, r2
	ldrhi	r2, .L2164
	strhi	r3, [r2, #-2812]
	movw	r3, #65535
	cmp	ip, r3
	beq	.L2150
	ldr	r0, .L2164+36
	bl	make_superblock
.L2150:
	ldrh	r2, [r6]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2151
	ldr	r0, .L2164+40
	bl	make_superblock
.L2151:
	ldrh	r2, [r7]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2152
	ldr	r0, .L2164+44
	bl	make_superblock
.L2152:
	ldrh	r2, [r8]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2153
	ldr	r0, .L2164+48
	bl	make_superblock
.L2153:
	mov	r0, #0
.L2137:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2165:
	.align	2
.L2164:
	.word	.LANCHOR2
	.word	.LANCHOR0+2848
	.word	.LANCHOR0
	.word	.LANCHOR2+216
	.word	.LANCHOR0+2918
	.word	.LANCHOR0+2910
	.word	1179929683
	.word	.LANCHOR2-3084
	.word	.LANCHOR0+2968
	.word	.LANCHOR2-3008
	.word	.LANCHOR2-2960
	.word	.LANCHOR2-2912
	.word	.LANCHOR2-2768
	.fnend
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlDumpBlockInfo
	.type	FtlDumpBlockInfo, %function
FtlDumpBlockInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 72
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #2908
	ldr	r5, .L2180
	.pad #100
	sub	sp, sp, #100
	ubfx	r0, r0, #10, #16
	mov	r4, r1
	ldr	r7, .L2180+4
	ldrh	r9, [r5, r3]
	bl	P2V_block_in_plane
	ldr	r1, .L2180+8
	mov	r6, r0
	ldr	r0, .L2180+12
	bl	printk
	ldr	r2, [r7, #-3028]
	mov	r3, r6, asl #1
	mov	r1, r6
	ldr	r0, .L2180+16
	ldrh	r2, [r2, r3]
	bl	printk
	add	r0, sp, #96
	strh	r6, [r0, #-48]!	@ movhi
	bl	make_superblock
	ldrb	r2, [r5, #1888]	@ zero_extendqisi2
	clz	r3, r4
	cmp	r2, #0
	mov	r3, r3, lsr #5
	moveq	r3, #0
	cmp	r3, #0
	moveq	r4, r3
	str	r7, [sp, #44]
	beq	.L2167
	mov	r0, r6
	bl	ftl_get_blk_mode
	cmp	r0, #1
	mov	r4, r0
	movweq	r3, #2910
	ldreqh	r9, [r5, r3]
.L2167:
	movw	r3, #2908
	ldr	r0, .L2180+20
	mov	r1, r4
	mov	r2, r9
	ldrh	r3, [r5, r3]
	mov	r6, #0
	bl	printk
	ldr	r8, .L2180+4
.L2168:
	ldr	r3, .L2180+24
	add	r0, sp, #62
	movw	lr, #65535
	mov	r10, #36
	ldrh	r3, [r3]
	mov	r7, r3
	ldr	r3, [r8, #-2072]
	str	r3, [sp, #28]
	ldr	r3, [r8, #-2180]
	str	r3, [sp, #32]
	ldr	r3, .L2180+28
	ldrh	r2, [r3]
	ldrh	ip, [r3, #2]
	str	r2, [sp, #36]
	ldr	r2, [r8, #-2176]
	str	r2, [sp, #40]
	mov	r2, #0
	mov	r5, r2
.L2169:
	uxth	r3, r2
	cmp	r3, r7
	bcs	.L2178
	ldrh	r3, [r0, #2]!
	cmp	r3, lr
	beq	.L2170
	ldr	r1, [sp, #28]
	orr	r3, r6, r3, asl #10
	mla	r1, r10, r5, r1
	str	r3, [r1, #4]
	ldr	r3, [sp, #36]
	mul	r3, r3, r5
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	ldr	fp, [sp, #32]
	bic	r3, r3, #3
	add	r3, fp, r3
	str	r3, [r1, #8]
	mul	r3, ip, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	ldr	fp, [sp, #40]
	bic	r3, r3, #3
	add	r3, fp, r3
	str	r3, [r1, #12]
.L2170:
	add	r2, r2, #1
	b	.L2169
.L2178:
	ldr	r0, [r8, #-2072]
	mov	r1, r5
	mov	r2, r4
	mov	r10, #0
	bl	FlashReadPages
	mov	fp, #36
.L2172:
	uxth	r3, r10
	cmp	r3, r5
	bcs	.L2179
	ldr	r3, [sp, #44]
	mul	r2, fp, r10
	ldrh	r1, [sp, #48]
	ldr	lr, [r3, #-2072]
	add	r10, r10, #1
	add	ip, lr, r2
	ldr	r3, [ip, #12]
	ldr	r0, [ip, #8]
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
	ldr	r0, .L2180+32
	str	r3, [sp, #20]
	ldr	r2, [lr, r2]
	ldr	r3, [ip, #4]
	bl	printk
	b	.L2172
.L2179:
	add	r6, r6, #1
	uxth	r6, r6
	cmp	r6, r9
	bne	.L2168
.L2174:
	mov	r0, #0
	add	sp, sp, #100
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2181:
	.align	2
.L2180:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR3+16
	.word	.LC107
	.word	.LC108
	.word	.LC109
	.word	.LANCHOR0+2840
	.word	.LANCHOR0+2918
	.word	.LC110
	.fnend
	.size	FtlDumpBlockInfo, .-FtlDumpBlockInfo
	.align	2
	.global	FtlScanAllBlock
	.type	FtlScanAllBlock, %function
FtlScanAllBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #92
	sub	sp, sp, #92
	ldr	r0, .L2197
	mov	r7, #0
	ldr	r1, .L2197+4
	bl	printk
	ldr	r5, .L2197+8
	mov	r6, r5
.L2183:
	ldr	r3, .L2197+12
	uxth	r4, r7
	ldrh	r3, [r3]
	cmp	r3, r4
	bls	.L2193
	add	r8, sp, #88
	mov	r0, r4
	movw	r9, #65535
	mov	r10, #36
	strh	r4, [r8, #-48]!	@ movhi
	bl	ftl_get_blk_mode
	ldr	ip, [r5, #-3028]
	mov	r2, r4, asl #1
	mov	r1, r4
	ldrh	r2, [ip, r2]
	mov	r3, r0
	ldr	r0, .L2197+16
	bl	printk
	mov	r0, r8
	bl	make_superblock
	ldr	r3, .L2197+20
	ldr	lr, [r5, #-2176]
	ldrh	r2, [r3]
	ldrh	ip, [r3, #78]
	ldrh	r8, [r3, #80]
	str	r2, [sp, #28]
	add	r0, sp, #54
	ldr	r2, [r5, #-2072]
	str	r2, [sp, #32]
	ldr	r2, [r5, #-2180]
	str	r2, [sp, #36]
	mov	r2, #0
	mov	r4, r2
.L2184:
	ldr	r1, [sp, #28]
	uxth	r3, r2
	cmp	r3, r1
	bcs	.L2194
	ldrh	r3, [r0, #2]!
	cmp	r3, r9
	beq	.L2185
	ldr	r1, [sp, #32]
	mov	r3, r3, asl #10
	mla	r1, r10, r4, r1
	str	r3, [r1, #4]
	mul	r3, ip, r4
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	ldr	fp, [sp, #36]
	bic	r3, r3, #3
	add	r3, fp, r3
	str	r3, [r1, #8]
	mul	r3, r8, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	bic	r3, r3, #3
	add	r3, lr, r3
	str	r3, [r1, #12]
.L2185:
	add	r2, r2, #1
	b	.L2184
.L2194:
	ldr	r0, [r6, #-2072]
	mov	r1, r4
	mov	r2, #0
	mov	r8, #0
	bl	FlashReadPages
	mov	r9, #36
.L2187:
	uxth	r3, r8
	cmp	r3, r4
	bcs	.L2195
	mul	r2, r9, r8
	ldr	lr, [r6, #-2072]
	ldrh	r1, [sp, #40]
	add	r8, r8, #1
	add	ip, lr, r2
	ldr	r3, [ip, #12]
	ldr	r0, [ip, #8]
	ldr	r10, [r3]
	str	r10, [sp]
	ldr	r10, [r3, #4]
	str	r10, [sp, #4]
	ldr	r10, [r3, #8]
	str	r10, [sp, #8]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [r0]
	str	r3, [sp, #16]
	ldr	r3, [r0, #4]
	ldr	r0, .L2197+24
	str	r3, [sp, #20]
	ldr	r2, [lr, r2]
	ldr	r3, [ip, #4]
	bl	printk
	b	.L2187
.L2195:
	ldr	r0, [r6, #-2072]
	mov	r1, r4
	mov	r2, #1
	mov	r8, #0
	bl	FlashReadPages
	mov	r9, #36
.L2189:
	uxth	r3, r8
	cmp	r3, r4
	bcs	.L2196
	mul	r2, r9, r8
	ldr	lr, [r6, #-2072]
	ldrh	r1, [sp, #40]
	add	r8, r8, #1
	add	ip, lr, r2
	ldr	r3, [ip, #12]
	ldr	r0, [ip, #8]
	ldr	r10, [r3]
	str	r10, [sp]
	ldr	r10, [r3, #4]
	str	r10, [sp, #4]
	ldr	r10, [r3, #8]
	str	r10, [sp, #8]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [r0]
	str	r3, [sp, #16]
	ldr	r3, [r0, #4]
	ldr	r0, .L2197+28
	str	r3, [sp, #20]
	ldr	r2, [lr, r2]
	ldr	r3, [ip, #4]
	bl	printk
	b	.L2189
.L2196:
	add	r7, r7, #1
	b	.L2183
.L2193:
	mov	r0, #0
	add	sp, sp, #92
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2198:
	.align	2
.L2197:
	.word	.LC107
	.word	.LANCHOR3+36
	.word	.LANCHOR2
	.word	.LANCHOR0+2850
	.word	.LC111
	.word	.LANCHOR0+2840
	.word	.LC112
	.word	.LC113
	.fnend
	.size	FtlScanAllBlock, .-FtlScanAllBlock
	.align	2
	.global	ftl_scan_all_ppa
	.type	ftl_scan_all_ppa, %function
ftl_scan_all_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2206
	movw	r2, #2906
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #28
	sub	sp, sp, #28
	ldrh	r1, [r3, r2]
	movw	r2, #2908
	ldr	r0, .L2206+4
	mov	r6, #0
	ldrh	r2, [r3, r2]
	bl	printk
	ldr	r9, .L2206+8
	ldr	r4, .L2206+12
.L2200:
	ldrh	r3, [r9]
	cmp	r6, r3
	bcs	.L2202
	ldr	r7, .L2206+16
	mov	r8, r6, asl #10
	mov	r5, #0
.L2203:
	ldrh	r3, [r7]
	cmp	r5, r3
	bcs	.L2205
	add	r3, r5, r8
	str	r3, [r4, #220]
	ldr	r3, [r4, #-2060]
	mov	r2, #0
	mov	r1, #1
	ldr	r0, .L2206+20
	str	r2, [r4, #216]
	add	r5, r5, #1
	str	r3, [r4, #224]
	ldr	r3, [r4, #-2036]
	str	r3, [r4, #228]
	bl	FlashReadPages
	ldr	r3, [r4, #228]
	ldr	r2, [r4, #224]
	ldr	r0, .L2206+24
	ldr	r1, [r3, #4]
	str	r1, [sp]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2]
	str	r1, [sp, #12]
	ldr	r2, [r2, #4]
	ldr	r1, [r4, #220]
	str	r2, [sp, #16]
	ldr	r2, [r4, #216]
	ldr	r3, [r3]
	bl	printk
	b	.L2203
.L2205:
	add	r6, r6, #1
	b	.L2200
.L2202:
	ldr	r0, .L2206+28
	ldr	r1, .L2206+32
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L2207:
	.align	2
.L2206:
	.word	.LANCHOR0
	.word	.LC114
	.word	.LANCHOR0+2906
	.word	.LANCHOR2
	.word	.LANCHOR0+2908
	.word	.LANCHOR2+216
	.word	.LC115
	.word	.LC116
	.word	.LANCHOR3+52
	.fnend
	.size	ftl_scan_all_ppa, .-ftl_scan_all_ppa
	.align	2
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #76
	sub	sp, sp, #76
	ldr	r5, .L2265
	mov	r4, r0
	str	r1, [sp, #8]
	mov	r9, r2
	str	r3, [sp, #20]
	ldr	ip, [r5, #1784]
	ldrb	r7, [r5]	@ zero_extendqisi2
	ldrb	ip, [ip, #19]	@ zero_extendqisi2
	cmp	r7, #0
	str	ip, [sp, #16]
	ldr	ip, .L2265+4
	ldrb	ip, [ip, #265]	@ zero_extendqisi2
	str	ip, [sp, #12]
	beq	.L2209
	bl	FlashProgSlc2KPages
	b	.L2210
.L2223:
	mov	r3, #36
	add	r2, sp, #28
	mul	r8, r3, r7
	ldr	r3, [sp, #8]
	mov	r1, r9
	rsb	r3, r7, r3
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
	mvncs	r3, #0
	strcs	r3, [r4, r8]
	bcc	.L2262
.L2212:
	add	r7, r7, #1
.L2209:
	ldr	r3, [sp, #8]
	ldr	r6, .L2265
	cmp	r7, r3
	bcc	.L2223
	b	.L2263
.L2262:
	ldrb	r1, [r5, #2828]	@ zero_extendqisi2
	add	r3, r5, r3, asl #4
	cmp	r1, #0
	ldr	r3, [r3, #1664]
	moveq	r10, #0
	cmp	r3, #0
	beq	.L2214
	cmp	r2, #1
	bne	.L2215
	ldr	r0, [r5, #1824]
	bl	NandcIqrWaitFlashReady
.L2215:
	ldrb	r0, [sp, #32]	@ zero_extendqisi2
	bl	FlashWaitCmdDone
.L2214:
	ldr	r3, [sp, #32]
	mov	r1, #0
	cmp	r10, #0
	add	r2, r5, r3, asl #4
	str	r1, [r2, #1668]
	ldr	r1, [sp, #28]
	str	fp, [r2, #1664]
	str	r1, [r2, #1660]
	addne	r1, r8, #36
	addne	r1, r4, r1
	strne	r1, [r2, #1668]
	add	r2, r5, r3
	add	r3, r5, r3, asl #4
	ldrb	r6, [r2, #1616]	@ zero_extendqisi2
	strb	r6, [r3, #1656]
	mov	r0, r6
	ldrb	r3, [r5, #1612]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L2217
	bl	NandcWaitFlashReady
	b	.L2218
.L2217:
	bl	NandcFlashCs
	mov	r0, r6
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r6
	bl	NandcFlashDeCs
.L2218:
	ldr	r3, [sp, #16]
	sub	r3, r3, #1
	cmp	r3, #7
	bhi	.L2219
	add	r3, r5, r6
	ldrb	r3, [r3, #2756]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2219
	mov	r0, r6
	ldrb	r1, [r5, #1903]	@ zero_extendqisi2
	ldr	r2, .L2265+8
	mov	r3, #0
	bl	HynixSetRRPara
.L2219:
	mov	r0, r6
	bl	NandcFlashCs
	cmp	r9, #1
	mov	r0, r6
	bne	.L2220
	ldrb	r3, [r5, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2220
	bl	flash_enter_slc_mode
	b	.L2221
.L2220:
	bl	flash_exit_slc_mode
.L2221:
	mov	r0, r6
	ldr	r1, [sp, #28]
	bl	FlashProgFirstCmd
	ldr	r3, [fp, #12]
	mov	r0, r6
	mov	r1, #1
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [fp, #8]
	bl	NandcXferData
	cmp	r10, #0
	beq	.L2222
	mov	r0, r6
	ldr	r1, [sp, #28]
	bl	FlashProgDpFirstCmd
	mov	r0, r6
	add	r8, r8, #36
	add	r8, r4, r8
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	ldr	r1, [r5, #4]
	mov	r0, r6
	ldr	r3, [sp, #28]
	add	r1, r1, r3
	bl	FlashProgDpSecondCmd
	ldr	r3, [r8, #12]
	mov	r0, r6
	mov	r1, #1
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [r8, #8]
	bl	NandcXferData
.L2222:
	mov	r0, r6
	ldr	r1, [sp, #28]
	bl	FlashProgSecondCmd
	mov	r0, r6
	bl	NandcFlashDeCs
	add	r7, r7, r10
	b	.L2212
.L2263:
	ldr	r0, [r6, #1824]
	mov	r5, #0
	bl	NandcIqrWaitFlashReady
	ldr	r8, .L2265+12
	mov	r7, r6
.L2224:
	ldrb	r3, [r6, #1612]	@ zero_extendqisi2
	cmp	r5, r3
	bcs	.L2264
	uxtb	r0, r5
	bl	FlashWaitCmdDone
	cmp	r9, #1
	bne	.L2225
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2225
	ldrb	r0, [r8, r5, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L2225:
	add	r5, r5, #1
	b	.L2224
.L2264:
	ldr	r3, [sp, #20]
	cmp	r3, #0
	bne	.L2227
.L2235:
	mov	r0, #0
	b	.L2210
.L2227:
	ldr	r5, .L2265+16
	mov	r6, #0
	mov	r7, r5
.L2228:
	ldr	r3, [sp, #8]
	cmp	r6, r3
	beq	.L2235
	ldr	r3, [r4]
	cmn	r3, #1
	bne	.L2229
	ldr	r1, [r4, #4]
	ldr	r0, .L2265+20
	bl	printk
	b	.L2230
.L2229:
	ldr	r3, [sp, #8]
	mov	r1, r9
	add	r2, sp, #28
	mov	r0, r4
	rsb	r3, r6, r3
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldr	r2, [r5, #188]
	mov	r3, #0
	mov	lr, r4
	str	r3, [r2]
	ldr	r2, [r5, #192]
	str	r3, [r2]
	ldmia	lr!, {r0, r1, r2, r3}
	add	ip, sp, #36
	stmia	ip!, {r0, r1, r2, r3}
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	add	r0, sp, #36
	ldr	r3, [lr]
	mov	r1, #1
	mov	r2, r9
	str	r3, [ip]
	ldr	r3, [r5, #188]
	str	r3, [sp, #44]
	ldr	r3, [r5, #192]
	str	r3, [sp, #48]
	bl	FlashReadPages
	ldr	r8, [sp, #36]
	cmn	r8, #1
	bne	.L2231
	ldr	r0, .L2265+24
	ldr	r1, [r4, #4]
	bl	printk
	str	r8, [r4]
.L2231:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L2232
	ldr	r2, [r3]
	ldr	r3, [r7, #192]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L2232
	ldr	r0, .L2265+28
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L2232:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L2230
	ldr	r2, [r3]
	ldr	r3, [r7, #188]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L2230
	ldr	r0, .L2265+32
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L2230:
	add	r6, r6, #1
	add	r4, r4, #36
	b	.L2228
.L2210:
	add	sp, sp, #76
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2266:
	.align	2
.L2265:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+1906
	.word	.LANCHOR0+1656
	.word	.LANCHOR2
	.word	.LC103
	.word	.LC104
	.word	.LC105
	.word	.LC106
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2271
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #108
	sub	sp, sp, #108
	ldr	r2, [r3, #172]
	cmp	r0, r2
	movcc	r4, #0
	bcc	.L2268
	ldr	r5, [r3, #180]
	mov	r4, r0
	mov	r1, #165
	add	r0, sp, #40
	mov	r2, #32
	str	r0, [sp, #16]
	str	r5, [sp, #12]
	bl	memset
	mov	r1, #90
	mov	r2, #8
	mov	r0, r5
	mov	r4, r4, asl #10
	bl	memset
	mov	r1, #1
	mov	r2, r1
	add	r0, sp, #4
	str	r4, [sp, #8]
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r2, r1
	mov	r3, r1
	add	r0, sp, #4
	bl	FlashProgPages
	mov	r1, #0
	mov	r2, #1
	ldr	r4, [sp, #4]
	add	r0, sp, #4
	adds	r4, r4, #0
	movne	r4, #1
	rsb	r4, r4, #0
	bl	FlashEraseBlocks
.L2268:
	mov	r0, r4
	add	sp, sp, #108
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L2272:
	.align	2
.L2271:
	.word	.LANCHOR2
	.fnend
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FlashMakeFactorBbt
	.type	FlashMakeFactorBbt, %function
FlashMakeFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #84
	sub	sp, sp, #84
	ldr	r4, .L2328
	mov	r1, #1
	ldr	r6, .L2328+4
	ldr	r5, .L2328+8
	ldr	r3, [r4, #184]
	ldrh	r8, [r6, #12]
	ldr	r0, .L2328+12
	str	r3, [sp, #20]
	ldrh	r3, [r6, #14]
	ldr	r7, .L2328+8
	smulbb	r8, r8, r3
	ldr	r3, [r5, #1784]
	ldrb	r3, [r3, #24]	@ zero_extendqisi2
	uxth	r8, r8
	str	r3, [sp, #24]
	ldrh	r3, [r5, #4]
	str	r3, [sp, #16]
	ldrb	r3, [r5]	@ zero_extendqisi2
	cmp	r3, #1
	ldreq	r3, [sp, #16]
	moveq	r3, r3, asl #1
	uxtheq	r3, r3
	streq	r3, [sp, #16]
	bl	printk
	ldr	r0, [r4, #184]
	mov	r1, #4096
	ldr	r4, .L2328
	bl	__memzero
	mov	r3, r8, lsr #4
	str	r3, [sp, #28]
	mov	r3, #0
	str	r3, [sp, #8]
.L2275:
	ldrb	r6, [sp, #8]	@ zero_extendqisi2
	ldrb	r3, [r7, #1612]	@ zero_extendqisi2
	cmp	r3, r6
	bls	.L2324
	add	r3, r4, r6, asl #1
	ldrh	r5, [r3, #200]
	cmp	r5, #0
	bne	.L2304
	ldr	r3, .L2328+4
	mov	r1, r5
	ldr	r0, [r4, #152]
	mov	r9, r5
	ldrh	r2, [r3, #20]
	mov	r2, r2, asl #9
	bl	ftl_memset
	add	r3, r7, r6
	ldrb	r10, [r3, #1616]	@ zero_extendqisi2
	mov	r3, r6, asl #2
	add	fp, r7, r3
	str	r5, [sp, #4]
	str	r3, [sp, #32]
.L2277:
	ldrh	r3, [sp, #4]
	cmp	r3, r8
	str	r3, [sp, #12]
	bcs	.L2286
	mvn	r3, #0
	strb	r3, [sp, #42]
	strb	r3, [sp, #43]
	ldr	r3, [sp, #24]
	tst	r3, #1
	beq	.L2279
	ldr	r3, [fp, #1624]
	add	r2, sp, #42
	mov	r0, r10
	add	r3, r5, r3
	str	r3, [sp, #36]
	mov	r1, r3
	bl	FlashReadSpare
	ldrb	r2, [r7]	@ zero_extendqisi2
	cmp	r2, #1
	ldr	r3, [sp, #36]
	bne	.L2279
	ldr	r1, [r7, #4]
	add	r2, sp, #43
	mov	r0, r10
	add	r1, r3, r1
	bl	FlashReadSpare
	ldrb	r3, [sp, #42]	@ zero_extendqisi2
	ldrb	r2, [sp, #43]	@ zero_extendqisi2
	and	r3, r3, r2
	strb	r3, [sp, #42]
.L2279:
	ldr	r3, [sp, #24]
	tst	r3, #2
	beq	.L2281
	ldr	r3, [r7, #1784]
	mov	r0, r10
	add	r2, sp, #43
	ldrh	r1, [r3, #10]
	ldr	r3, [fp, #1624]
	sub	r1, r1, #1
	add	r1, r1, r3
	add	r1, r1, r5
	bl	FlashReadSpare
.L2281:
	ldr	r3, [r7, #1784]
	ldrb	r3, [r3, #7]	@ zero_extendqisi2
	cmp	r3, #1
	cmpne	r3, #8
	ldrb	r3, [sp, #42]	@ zero_extendqisi2
	bne	.L2282
	cmp	r3, #0
	ldrneb	r0, [sp, #43]	@ zero_extendqisi2
	clzne	r0, r0
	movne	r0, r0, lsr #5
	bne	.L2283
	b	.L2303
.L2282:
	cmp	r3, #255
	bne	.L2303
	ldrb	r0, [sp, #43]	@ zero_extendqisi2
	subs	r0, r0, #255
	movne	r0, #1
	b	.L2283
.L2303:
	mov	r0, #1
.L2283:
	ldr	r3, [sp, #24]
	tst	r3, #4
	beq	.L2284
	ldr	r3, .L2328+8
	mov	r0, r10
	ldr	r2, [sp, #32]
	add	r3, r3, r2
	ldr	r1, [r3, #1624]
	add	r1, r5, r1
	bl	SandiskProgTestBadBlock
.L2284:
	cmp	r0, #0
	beq	.L2285
	mov	r1, r6
	ldr	r2, [sp, #4]
	ldr	r0, .L2328+16
	add	r9, r9, #1
	bl	printk
	ldr	r1, [r4, #152]
	mov	ip, #1
	uxth	r9, r9
	ldr	r3, [sp, #12]
	mov	r0, r3, lsr #5
	and	r3, r3, #31
	ldr	r2, [r1, r0, asl #2]
	orr	r3, r2, ip, asl r3
	ldr	r2, [sp, #28]
	str	r3, [r1, r0, asl #2]
	ldrb	r3, [r7, #1612]	@ zero_extendqisi2
	mul	r3, r3, r2
	cmp	r9, r3
	bgt	.L2286
.L2285:
	ldr	r3, [sp, #4]
	add	r3, r3, #1
	str	r3, [sp, #4]
	ldr	r3, [sp, #16]
	add	r5, r5, r3
	b	.L2277
.L2286:
	mov	r2, r9
	ldr	r0, .L2328+20
	mov	r1, r6
	bl	printk
	ldrb	r3, [r7, #1612]	@ zero_extendqisi2
	ldr	r2, [sp, #28]
	mul	r3, r3, r2
	cmp	r9, r3
	blt	.L2288
	ldr	r3, .L2328+4
	mov	r1, #0
	ldr	r0, [r4, #152]
	ldrh	r2, [r3, #20]
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2288:
	cmp	r6, #0
	ldreqh	r10, [r4, #172]
	moveq	r5, r6
	moveq	r9, #1
	bne	.L2290
.L2291:
	ldr	r3, .L2328+8
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, r10
	bls	.L2325
	mov	r0, r10
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L2292
	mov	r1, r10
	ldr	r0, .L2328+24
	bl	printk
	ldr	r1, [r4, #152]
	mov	r0, r10, lsr #5
	add	r5, r5, #1
	and	r3, r10, #31
	ldr	r2, [r1, r0, asl #2]
	uxth	r5, r5
	orr	r3, r2, r9, asl r3
	str	r3, [r1, r0, asl #2]
.L2292:
	add	r10, r10, #1
	uxth	r10, r10
	b	.L2291
.L2325:
	sub	fp, r8, #1
	sub	r9, r8, #50
	mov	r10, #1
	uxth	fp, fp
.L2294:
	cmp	fp, r9
	ble	.L2326
	mov	r0, fp
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L2295
	mov	r1, fp
	ldr	r0, .L2328+24
	bl	printk
	ldr	r1, [r4, #152]
	mov	r0, fp, lsr #5
	and	r3, fp, #31
	ldr	r2, [r1, r0, asl #2]
	orr	r3, r2, r10, asl r3
	str	r3, [r1, r0, asl #2]
.L2295:
	sub	fp, fp, #1
	uxth	fp, fp
	b	.L2294
.L2326:
	ldr	r3, .L2328+8
	ldr	r2, [r4, #172]
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	rsb	r3, r2, r3
	cmp	r5, r3
	bcc	.L2290
	ldr	r3, .L2328+4
	mov	r1, #0
	ldr	r0, [r4, #152]
	ldrh	r2, [r3, #20]
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2290:
	ldrb	r5, [sp, #8]	@ zero_extendqisi2
	sub	r10, r8, #1
	add	r9, r4, r6, asl #1
	uxth	r10, r10
	mul	r5, r8, r5
.L2298:
	mov	r1, r6
	ldr	r0, .L2328+28
	mov	r2, r10
	bl	printk
	ldr	r1, [r4, #152]
.L2299:
	mov	r2, r10, lsr #5
	and	r3, r10, #31
	ldr	r2, [r1, r2, asl #2]
	mov	r3, r2, lsr r3
	ands	r3, r3, #1
	subne	r10, r10, #1
	uxthne	r10, r10
	bne	.L2299
.L2327:
	ldr	r1, [sp, #20]
	add	r0, sp, #44
	ldr	r2, .L2328+32
	strh	r10, [r9, #200]	@ movhi
	strh	r10, [r1, #2]	@ movhi
	strh	r2, [r1]	@ movhi
	strh	r3, [r1, #8]	@ movhi
	mov	r1, #1
	ldr	r3, [r4, #152]
	mov	r2, r1
	str	r3, [sp, #52]
	ldr	r3, [r4, #184]
	str	r3, [sp, #56]
	add	r3, r10, r5
	mov	r3, r3, asl #10
	str	r3, [sp, #48]
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	mov	r2, r1
	add	r0, sp, #44
	bl	FlashProgPages
	ldr	r3, [sp, #44]
	cmp	r3, #0
	subne	r10, r10, #1
	uxthne	r10, r10
	bne	.L2298
.L2304:
	ldr	r3, [sp, #8]
	add	r3, r3, #1
	str	r3, [sp, #8]
	b	.L2275
.L2324:
	add	sp, sp, #84
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2329:
	.align	2
.L2328:
	.word	.LANCHOR2
	.word	.LANCHOR0+1860
	.word	.LANCHOR0
	.word	.LC117
	.word	.LC118
	.word	.LC119
	.word	.LC120
	.word	.LC121
	.word	-3872
	.fnend
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r5, .L2380
	ldr	r6, .L2380+4
	ldr	fp, [r5, #-3100]
	ldrb	r3, [r6, #2828]	@ zero_extendqisi2
	cmp	fp, #0
	str	r3, [sp, #16]
	movne	r0, #0
	bne	.L2331
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	mov	r10, r5
	mov	r8, fp
	mov	r5, fp
	mov	r4, fp
	mov	r9, r6
	mov	fp, r6
	mov	r7, #36
	str	r1, [sp]
	str	r0, [sp, #4]
	str	r3, [sp, #8]
	str	r0, [r10, #-2084]
.L2332:
	ldr	r3, .L2380+8
	uxth	r1, r8
	ldr	ip, .L2380+4
	ldrh	r0, [r3]
	cmp	r0, r1
	bls	.L2375
	mul	r0, r7, r1
	ldr	ip, [r10, #-3096]
	add	r1, fp, r1
	mov	r3, #0
	str	r3, [ip, r0]
	ldrb	r0, [r1, #2868]	@ zero_extendqisi2
	ldr	r1, [sp, #4]
	bl	V2P_block
	ldr	r3, [sp]
	mov	r6, r0
	cmp	r3, #0
	beq	.L2333
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L2334
.L2333:
	mov	r0, r6
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
	bne	.L2334
	ldr	r3, .L2380+12
	mov	r6, r6, asl #10
	ldr	ip, [r10, #-3096]
	ldrh	r1, [r3]
	mla	ip, r7, r5, ip
	mul	r1, r1, r5
	add	r5, r5, #1
	uxth	r5, r5
	cmp	r1, #0
	str	r0, [ip, #8]
	add	r0, r1, #3
	str	r6, [ip, #4]
	movlt	r1, r0
	ldr	r0, [r10, #-2032]
	bic	r1, r1, #3
	add	r1, r0, r1
	str	r1, [ip, #12]
.L2334:
	add	r8, r8, #1
	b	.L2332
.L2375:
	cmp	r5, #0
	beq	.L2355
	ldr	r3, [sp, #8]
	mov	r2, r5
	ldr	r0, [r10, #-3096]
	mov	r7, #0
	adds	r6, r3, #0
	strb	r7, [ip, #2828]
	str	ip, [sp, #8]
	mov	r8, #36
	movne	r6, #1
	mov	r1, r6
	bl	FlashEraseBlocks
	ldrb	r3, [sp, #16]	@ zero_extendqisi2
	ldr	ip, [sp, #8]
	strb	r3, [ip, #2828]
.L2338:
	uxth	r2, r7
	cmp	r2, r5
	bcs	.L2376
	mul	r2, r8, r7
	ldr	r1, [r10, #-3096]
	add	r0, r1, r2
	ldr	r2, [r1, r2]
	cmn	r2, #1
	bne	.L2339
	ldr	r0, [r0, #4]
	add	r4, r4, #1
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
.L2339:
	add	r7, r7, #1
	b	.L2338
.L2376:
	ldr	r3, [sp]
	cmp	r3, #0
	bne	.L2341
	mov	r3, #6
	uxth	r6, r6
	str	r3, [sp, #12]
	mov	r3, #1
	str	r3, [sp, #8]
	b	.L2342
.L2341:
	movw	r3, #2910
	ldrh	r3, [r9, r3]
	str	r3, [sp, #8]
	ldrb	r3, [r9, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, [sp, #8]
	moveq	r6, #1
	movne	r6, #1
	strne	r6, [sp, #12]
	moveq	r3, r3, lsr #2
	streq	r3, [sp, #12]
.L2342:
	ldr	r10, .L2380
	mov	r7, #0
	mov	r8, r10
.L2351:
	mov	fp, #0
	mov	r5, fp
.L2343:
	ldr	r3, .L2380+8
	ldr	r9, .L2380+4
	ldrh	r1, [r3]
	uxth	r3, fp
	cmp	r1, r3
	bls	.L2377
	mov	r2, #36
	ldr	r0, [r10, #-3096]
	mul	r1, r2, r3
	mov	r2, #0
	str	r2, [r0, r1]
	ldr	r2, .L2380+4
	ldr	r1, [sp, #4]
	add	r3, r2, r3
	ldrb	r0, [r3, #2868]	@ zero_extendqisi2
	bl	V2P_block
	ldr	r3, [sp]
	mov	r9, r0
	cmp	r3, #0
	beq	.L2344
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L2345
.L2344:
	mov	r0, r9
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L2345
	ldr	r1, [r8, #-3096]
	mov	r3, #36
	add	r9, r7, r9, asl #10
	mla	r1, r3, r5, r1
	ldr	r3, [r8, #-2044]
	str	r3, [r1, #8]
	ldr	r3, .L2380+12
	str	r9, [r1, #4]
	ldrh	r3, [r3]
	mul	r3, r3, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	r0, r3, #3
	cmp	r3, #0
	movlt	r3, r0
	ldr	r0, [r8, #-2040]
	bic	r3, r3, #3
	add	r3, r0, r3
	str	r3, [r1, #12]
.L2345:
	add	fp, fp, #1
	b	.L2343
.L2377:
	cmp	r5, #0
	beq	.L2355
	mov	r2, r6
	mov	r3, #1
	ldr	r0, [r10, #-3096]
	mov	r1, r5
	mov	fp, #0
	strb	fp, [r9, #2828]
	bl	FlashProgPages
	mov	r2, #36
	ldrb	r3, [sp, #16]	@ zero_extendqisi2
	strb	r3, [r9, #2828]
	ldr	r9, .L2380
.L2348:
	uxth	r3, fp
	cmp	r3, r5
	bcs	.L2378
	mul	r3, r2, fp
	ldr	r1, [r8, #-3096]
	add	r0, r1, r3
	ldr	r3, [r1, r3]
	cmp	r3, #0
	beq	.L2349
	ldr	r0, [r0, #4]
	add	r4, r4, #1
	str	r2, [sp, #20]
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
	ldr	r2, [sp, #20]
.L2349:
	add	fp, fp, #1
	b	.L2348
.L2378:
	ldr	r3, [sp, #12]
	add	r7, r7, r3
	ldr	r3, [sp, #8]
	uxth	r7, r7
	cmp	r7, r3
	bcc	.L2351
	mov	r7, #0
	mov	r8, #36
.L2352:
	uxth	r3, r7
	cmp	r3, r5
	ldr	r3, [sp]
	bcs	.L2379
	cmp	r3, #0
	beq	.L2353
	mul	r3, r8, r7
	ldr	r2, [r9, #-3096]
	add	r1, r2, r3
	ldr	r3, [r2, r3]
	cmp	r3, #0
	bne	.L2353
	ldr	r0, [r1, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L2353:
	add	r7, r7, #1
	b	.L2352
.L2379:
	adds	r3, r3, #0
	ldr	r2, [sp, #4]
	movne	r3, #1
	cmp	r2, #63
	orrls	r3, r3, #1
	cmp	r3, #0
	beq	.L2355
	ldr	r0, [r9, #-3096]
	mov	r1, r6
	mov	r2, r5
	bl	FlashEraseBlocks
.L2355:
	mov	r0, r4
.L2331:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2381:
	.align	2
.L2380:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+2840
	.word	.LANCHOR0+2920
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.global	FtlBbmTblFlush
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L2400
	ldr	r5, [r4, #-3100]
	cmp	r5, #0
	bne	.L2384
	ldr	r3, [r4, #-2036]
	mov	r1, r5
	ldr	r6, .L2400+4
	ldr	r0, [r4, #-2060]
	str	r3, [r4, #228]
	movw	r3, #2918
	ldrh	r2, [r6, r3]
	add	r6, r6, #2992
	str	r0, [r4, #224]
	bl	ftl_memset
	ldr	r9, .L2400+8
	sub	r7, r6, #130
.L2385:
	ldrh	r3, [r7]
	ldr	r8, .L2400+4
	cmp	r5, r3
	bge	.L2399
	ldrh	r2, [r9]
	ldr	r3, [r4, #224]
	ldr	r1, [r6, #4]!
	mul	r0, r2, r5
	mov	r2, r2, asl #2
	add	r5, r5, #1
	add	r0, r3, r0, asl #2
	bl	ftl_memcpy
	b	.L2385
.L2399:
	ldr	r5, [r4, #228]
	mov	r1, #255
	mov	r2, #16
	ldr	r6, .L2400+12
	ldr	fp, .L2400
	mov	r7, #0
	mov	r0, r5
	mov	r10, r7
	bl	memset
	ldr	r3, .L2400+16
	strh	r3, [r5]	@ movhi
	ldr	r3, [r8, #2976]
	str	r3, [r5, #4]
	movw	r3, #2968
	ldrh	r3, [r8, r3]
	strh	r3, [r5, #2]	@ movhi
	ldrh	r3, [r6, #4]
	strh	r3, [r5, #8]	@ movhi
	ldrh	r3, [r6, #6]
	strh	r3, [r5, #10]	@ movhi
	ldr	r3, [r8, #2836]
	strh	r3, [r5, #12]	@ movhi
.L2387:
	ldr	r3, [r4, #-2060]
	mov	ip, #0
	ldrh	r1, [r6]
	ldrh	r2, [r6, #2]
	str	r3, [r4, #224]
	ldr	r3, [r4, #-2036]
	str	ip, [r4, #216]
	str	ip, [sp, #12]
	str	r3, [r4, #228]
	orr	r3, r2, r1, asl #10
	ldrh	r0, [r5, #10]
	str	r3, [r4, #220]
	ldrh	r3, [r6, #4]
	str	r0, [sp]
	ldr	r0, .L2400+20
	bl	printk
	ldr	r9, .L2400+12
	ldrh	r2, [r6, #2]
	ldrh	r3, [r9, #-58]
	sub	r3, r3, #1
	cmp	r2, r3
	ldr	ip, [sp, #12]
	blt	.L2388
	ldr	r3, [r8, #2976]
	mov	r1, #1
	ldrh	r2, [r9]
	add	r3, r3, #1
	ldr	r0, [fp, #-3096]
	str	r3, [r8, #2976]
	str	r3, [r5, #4]
	ldrh	r3, [r9, #4]
	strh	r2, [r5, #8]	@ movhi
	strh	r2, [r9, #4]	@ movhi
	mov	r2, r1
	strh	r3, [r9]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [fp, #220]
	str	r3, [r0, #4]
	strh	ip, [r9, #2]	@ movhi
	bl	FlashEraseBlocks
.L2388:
	ldr	r9, .L2400
	mov	r1, #1
	mov	r3, r1
	mov	r2, r1
	add	r0, r9, #216
	bl	FlashProgPages
	ldrh	r3, [r6, #2]
	add	r3, r3, #1
	strh	r3, [r6, #2]	@ movhi
	ldr	r3, [r4, #216]
	cmn	r3, #1
	bne	.L2389
	add	r7, r7, #1
	ldr	r0, .L2400+24
	ldr	r1, [fp, #220]
	uxth	r7, r7
	bl	printk
	cmp	r7, #3
	bls	.L2387
	ldr	r0, .L2400+28
	mov	r2, r7
	ldr	r1, [r9, #220]
	bl	printk
	mov	r3, #1
	str	r3, [r9, #-3100]
	b	.L2384
.L2389:
	add	r10, r10, #1
	cmp	r10, #1
	beq	.L2387
	cmp	r3, #256
	beq	.L2387
.L2384:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2401:
	.align	2
.L2400:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2-1972
	.word	.LANCHOR0+2968
	.word	-3887
	.word	.LC122
	.word	.LC123
	.word	.LC124
	.fnend
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	allocate_data_superblock
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L2457
	ldr	r3, [r4, #-3100]
	cmp	r3, #0
	bne	.L2403
	ldr	r10, .L2457+4
	mov	r5, r0
	mov	r8, r4
.L2404:
	ldr	r3, .L2457+8
	ldrb	r2, [r5, #8]	@ zero_extendqisi2
	cmp	r5, r3
	bne	.L2405
	sub	r3, r3, #100
	ldr	ip, [r8, #-2192]
	ldrh	r3, [r3]
	mul	lr, ip, r3
	mov	r0, r3, lsr #1
	add	r1, r0, #1
	add	r1, r1, lr, lsr #2
	ldr	lr, [r10, #2832]
	cmp	lr, #0
	uxth	r1, r1
	beq	.L2406
	ldr	lr, [r8, #-2792]
	cmp	lr, #39
	bhi	.L2406
	cmp	lr, #2
	bls	.L2431
	cmp	ip, #0
	movne	r3, #0
	andeq	r3, r3, #1
	cmp	r3, #0
	moveq	r1, r0
	beq	.L2406
	b	.L2431
.L2405:
	cmp	r2, #1
	bne	.L2431
	ldr	r3, .L2457+12
	ldrh	r3, [r3]
	cmp	r3, #1
	beq	.L2431
	ldrb	r3, [r10, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2431
	ldr	r3, .L2457+16
	ldr	r0, [r10, #2832]
	ldrh	r3, [r3]
	cmp	r0, #0
	mov	r1, r3, lsr #3
	beq	.L2406
	ldr	r0, [r8, #-2792]
	cmp	r0, #1
	rsbls	r3, r3, r3, asl #3
	ubfxls	r1, r3, #3, #16
.L2406:
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
	b	.L2407
.L2431:
	mov	r1, #0
.L2407:
	ldr	r0, .L2457+20
	bl	List_pop_index_node
	ldr	r2, .L2457+16
	ldr	fp, .L2457+24
	ldrh	r3, [r2]
	sub	r3, r3, #1
	strh	r3, [r2]	@ movhi
	ldrh	r3, [fp]
	uxth	r9, r0
	cmp	r3, r9
	bls	.L2404
	ldr	r3, [r8, #-3028]
	mov	r7, r9, asl #1
	ldrh	r6, [r3, r7]
	cmp	r6, #0
	bne	.L2404
	strh	r9, [r5]	@ movhi
	mov	r0, r5
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2409
	ldr	r1, [r8, #-3096]
	mov	lr, #36
	ldrh	ip, [fp, #-8]
	add	r0, r5, #14
	str	lr, [sp, #4]
	mov	r3, r1
	mla	r2, lr, ip, r1
	mov	ip, r6
	str	r2, [sp]
	b	.L2410
.L2409:
	ldr	r3, [r8, #-3028]
	b	.L2452
.L2410:
	ldr	r2, [sp]
	cmp	r3, r2
	beq	.L2454
	str	ip, [r3, #8]
	movw	lr, #65535
	str	ip, [r3, #12]
	add	r3, r3, #36
	ldrh	r2, [r0, #2]!
	cmp	r2, lr
	movne	r2, r2, asl #10
	ldrne	lr, [sp, #4]
	mlane	lr, lr, r6, r1
	addne	r6, r6, #1
	uxthne	r6, r6
	strne	r2, [lr, #4]
	b	.L2410
.L2454:
	ldr	r3, .L2457+28
	ldr	r2, [r10, #2832]
	rsb	r3, r3, r5
	clz	r3, r3
	cmp	r2, #0
	mov	r3, r3, lsr #5
	moveq	r3, #0
	cmp	r3, #0
	beq	.L2413
	ldr	r3, [r8, #-3092]
	ldrh	r3, [r3, r7]
	cmp	r3, #40
	movhi	r3, #0
	strhib	r3, [r8, #-3000]
.L2413:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r1, [r4, #-3092]
	cmp	r3, #0
	ldr	r2, .L2457+32
	ldrh	r3, [r1, r7]
	bne	.L2414
	cmp	r3, #0
	str	r2, [sp]
	ldrneh	r0, [r2]
	moveq	r3, #2
	addne	r3, r3, r0
	mov	r0, r9
	uxthne	r3, r3
	strh	r3, [r1, r7]	@ movhi
	ldr	r3, [r4, #-2808]
	mov	r1, #0
	add	r3, r3, #1
	str	r3, [r4, #-2808]
	bl	ftl_set_blk_mode
	ldr	r2, [sp]
	b	.L2416
.L2414:
	add	r3, r3, #1
	strh	r3, [r1, r7]	@ movhi
	ldr	r1, [r4, #-2852]
	mov	r0, r9, lsr #5
	ldr	r3, [r4, #-2804]
	mov	ip, #1
	add	r3, r3, #1
	str	r3, [r4, #-2804]
	ldr	lr, [r1, r0, asl #2]
	and	r3, r9, #31
	orr	r3, lr, ip, asl r3
	str	r3, [r1, r0, asl #2]
.L2416:
	ldr	r3, [r4, #-3092]
	ldr	r1, [r4, #-2796]
	ldrh	r2, [r2]
	ldrh	r3, [r3, r7]
	ldr	r0, [r4, #-2808]
	cmp	r3, r1
	ldrh	r1, [fp]
	strhi	r3, [r8, #-2796]
	ldr	r3, [r4, #-2804]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	ldr	r2, [r4, #-2016]
	ldr	r3, [r2, #16]
	add	r3, r3, #1
	str	r3, [r2, #16]
	ldr	r2, [r4, #-3096]
	mov	r3, #36
	add	r1, r2, #4
	mla	r3, r3, r6, r2
	add	r3, r3, #40
	str	r0, [r4, #-2800]
.L2418:
	add	r1, r1, #36
	cmp	r1, r3
	ldrne	r2, [r1, #-36]
	bicne	r2, r2, #1020
	bicne	r2, r2, #3
	strne	r2, [r1, #-36]
	bne	.L2418
.L2455:
	ldrb	r3, [r10, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2420
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	mov	r2, r6
	ldr	r0, [r4, #-3096]
	cmp	r3, #1
	moveq	r1, #0
	movne	r1, #1
	bl	FlashEraseBlocks
.L2420:
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	mov	r2, r6
	ldr	r0, [r4, #-3096]
	mov	fp, #0
	bl	FlashEraseBlocks
	add	r1, r5, #16
	mov	r2, fp
	mov	ip, #36
.L2422:
	uxth	r3, fp
	cmp	r3, r6
	bcs	.L2456
	mul	r3, ip, fp
	ldr	lr, [r4, #-3096]
	add	r0, lr, r3
	ldr	r3, [lr, r3]
	cmn	r3, #1
	bne	.L2423
	ldr	r0, [r0, #4]
	add	r2, r2, #1
	stmib	sp, {r1, r3, ip}
	ubfx	r0, r0, #10, #16
	str	r2, [sp]
	bl	FtlBbmMapBadBlock
	ldmib	sp, {r1, r3}
	ldr	ip, [sp, #12]
	ldr	r2, [sp]
	strh	r3, [r1]	@ movhi
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	sub	r3, r3, #1
	strb	r3, [r5, #7]
.L2423:
	add	fp, fp, #1
	add	r1, r1, #2
	b	.L2422
.L2456:
	cmp	r2, #0
	beq	.L2425
	mov	r0, r9
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L2425:
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2426
	ldr	r3, [r4, #-3028]
.L2452:
	mvn	r2, #0
	strh	r2, [r3, r7]	@ movhi
	b	.L2404
.L2426:
	ldr	r1, .L2457+4
	movw	r2, #2908
	ldrh	r2, [r1, r2]
	strh	r9, [r5]	@ movhi
	smulbb	r3, r2, r3
	mov	r2, #0
	strh	r2, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	ldr	r2, [r4, #-2816]
	ldr	r1, [r4, #-3028]
	uxth	r3, r3
	strh	r3, [r5, #4]	@ movhi
	str	r2, [r5, #12]
	add	r2, r2, #1
	str	r2, [r4, #-2816]
	ldrh	r2, [r5]
	mov	r2, r2, asl #1
	strh	r3, [r1, r2]	@ movhi
.L2403:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2458:
	.align	2
.L2457:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2-2912
	.word	.LANCHOR0+2860
	.word	.LANCHOR2-3012
	.word	.LANCHOR2-3016
	.word	.LANCHOR0+2848
	.word	.LANCHOR2-3008
	.word	.LANCHOR0+2900
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r5, .L2473
	ldrh	r3, [r5, #-14]
	cmp	r3, #0
	beq	.L2461
	sub	r5, r5, #14
	mov	r8, r0
	mov	r7, #0
	mov	r9, r5
.L2460:
	ldr	r3, .L2473+4
	uxth	r2, r7
	ldrh	r3, [r3]
	cmp	r3, r2
	bls	.L2470
	ldr	r3, .L2473+8
	mov	r1, r8
	mov	r10, #0
	add	r3, r3, r2
	ldrb	r0, [r3, #2868]	@ zero_extendqisi2
	bl	V2P_block
	ldr	ip, .L2473+12
	mov	fp, r0
.L2462:
	ldrh	r3, [r5]
	uxth	r4, r10
	cmp	r3, r4
	bls	.L2471
	mov	r3, r4, asl #1
	add	r6, ip, r3
	ldrh	r3, [ip, r3]
	cmp	r3, fp
	bne	.L2463
	mov	r1, fp
	ldr	r0, .L2473+16
	str	ip, [sp, #4]
	bl	printk
	mov	r0, fp
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r2, [r5]
	mov	r3, r6
	ldr	ip, [sp, #4]
.L2464:
	cmp	r4, r2
	ldrcch	r1, [r3, #2]
	addcc	r4, r4, #1
	uxthcc	r4, r4
	strcch	r1, [r3], #2	@ movhi
	bcc	.L2464
.L2472:
	sub	r2, r2, #1
	strh	r2, [r9]	@ movhi
.L2463:
	add	r10, r10, #1
	b	.L2462
.L2471:
	add	r7, r7, #1
	b	.L2460
.L2470:
	bl	FtlGcReFreshBadBlk
.L2461:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2474:
	.align	2
.L2473:
	.word	.LANCHOR2-2128
	.word	.LANCHOR0+2840
	.word	.LANCHOR0
	.word	.LANCHOR2-2140
	.word	.LC125
	.fnend
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L2485
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r3, r0, asl #1
	ldr	r1, [r2, #-3028]
	mov	r4, r0
	ldrh	r3, [r1, r3]
	cmp	r3, #0
	bne	.L2476
	sub	r1, r2, #2768
	sub	r5, r2, #3008
	ldrh	r0, [r1]
	cmp	r0, r4
	mvneq	r3, #0
	streqh	r3, [r1]	@ movhi
	beq	.L2478
	ldrh	r1, [r5]
	cmp	r1, r4
	beq	.L2484
	sub	r1, r2, #2960
	ldrh	r1, [r1]
	cmp	r1, r4
	beq	.L2484
	sub	r2, r2, #2912
	ldrh	r2, [r2]
	cmp	r2, r4
	beq	.L2484
.L2478:
	mov	r1, r4
	ldr	r0, .L2485+4
	bl	List_remove_node
	ldrh	r3, [r5, #-12]
	mov	r0, r4
	sub	r3, r3, #1
	strh	r3, [r5, #-12]	@ movhi
	bl	free_data_superblock
	mov	r0, r4
	bl	FtlGcFreeBadSuperBlk
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, pc}
.L2476:
	bl	List_update_data_list
.L2484:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L2486:
	.align	2
.L2485:
	.word	.LANCHOR2
	.word	.LANCHOR2-3032
	.fnend
	.size	update_vpc_list, .-update_vpc_list
	.align	2
	.global	decrement_vpc_count
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	beq	.L2488
	ldr	r6, .L2498
	mov	r5, r0, asl #1
	ldr	r3, [r6, #-3028]
	ldrh	r2, [r3, r5]
	cmp	r2, #0
	subne	r2, r2, #1
	strneh	r2, [r3, r5]	@ movhi
	bne	.L2488
	mov	r1, r4
	ldr	r0, .L2498+4
	bl	printk
	ldr	r3, [r6, #-3028]
	sub	r7, r6, #3008
	mov	r2, #32
	sub	r0, r7, #8
	mov	r1, r4
	strh	r2, [r3, r5]	@ movhi
	bl	test_node_in_list
	cmp	r0, #0
	beq	.L2490
	mov	r1, r4
	sub	r0, r7, #8
	bl	List_remove_node
	ldrh	r3, [r7, #-4]
	mov	r0, r4
	sub	r3, r3, #1
	strh	r3, [r7, #-4]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r6, #-3028]
	ldr	r0, .L2498+8
	mov	r1, r4
	ldrh	r2, [r3, r5]
	bl	printk
.L2490:
	mov	r0, r4
	bl	FtlGcRefreshBlock
	b	.L2493
.L2488:
	ldr	r5, .L2498+12
	movw	r3, #65535
	ldrh	r0, [r5, #-12]
	cmp	r0, r3
	streqh	r4, [r5, #-12]	@ movhi
	beq	.L2493
	cmp	r0, r4
	beq	.L2493
	bl	update_vpc_list
	strh	r4, [r5, #-12]	@ movhi
	adds	r0, r0, #0
	movne	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2493:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2499:
	.align	2
.L2498:
	.word	.LANCHOR2
	.word	.LC126
	.word	.LC127
	.word	.LANCHOR2-1904
	.fnend
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldrh	r3, [r0, #4]
	cmp	r3, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	ldrh	r2, [r0]
	movw	r3, #65535
	cmp	r2, r3
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	mov	r4, r0
	ldr	r5, .L2513
	ldr	r7, .L2513+4
	add	r3, r0, r3, asl #1
	add	r6, r5, #588
	ldrh	r3, [r3, #16]
.L2504:
	movw	r1, #65535
	cmp	r3, r1
	bne	.L2512
.L2506:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	ldrh	r2, [r7]
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r4, #6]
	cmp	r2, r3
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r4, #6]
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	add	r3, r4, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L2504
.L2512:
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L2507
	ldrb	r3, [r5, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2507
	ldrh	r3, [r4, #2]
	mov	r3, r3, asl #1
	ldrh	r3, [r6, r3]
	cmp	r3, r1
	bne	.L2507
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	ldrh	r2, [r4, #4]
	cmp	r2, #0
	bne	.L2506
	ldrh	r3, [r4, #2]
	strb	r2, [r4, #6]
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2507:
	ldr	r1, .L2513
	ldrb	r3, [r1, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r2, #1
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	movw	r3, #2910
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	ldmccfd	sp!, {r3, r4, r5, r6, r7, pc}
	ldr	r2, .L2513+8
	ldrh	r3, [r4]
	ldrh	ip, [r4, #4]
	ldr	r0, [r2, #-3028]
	mov	r3, r3, asl #1
	ldrh	r2, [r0, r3]
	rsb	r2, ip, r2
	strh	r2, [r0, r3]	@ movhi
	movw	r2, #2908
	mov	r3, #0
	ldrh	r2, [r1, r2]
	strh	r3, [r4, #4]	@ movhi
	strb	r3, [r4, #6]
	strh	r2, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2514:
	.align	2
.L2513:
	.word	.LANCHOR0
	.word	.LANCHOR0+2840
	.word	.LANCHOR2
	.fnend
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	2
	.global	get_new_active_ppa
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r3, #0
	strb	r3, [r0, #10]
	mov	r4, r0
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	ldr	r7, .L2533
	ldr	r8, .L2533+4
	add	r3, r0, r3, asl #1
	add	r9, r7, #588
	ldrh	r2, [r3, #16]
.L2516:
	movw	r1, #65535
	cmp	r2, r1
	ldr	r6, .L2533+4
	bne	.L2532
.L2517:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	ldrh	r2, [r8]
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r4, #6]
	cmp	r2, r3
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r4, #6]
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	add	r3, r4, r3, asl #1
	ldrh	r2, [r3, #16]
	b	.L2516
.L2532:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	ldrh	r5, [r4, #2]
	cmp	r3, #1
	ldrh	r3, [r4, #4]
	bne	.L2519
	ldrb	r0, [r7, #1888]	@ zero_extendqisi2
	cmp	r0, #0
	bne	.L2519
	mov	r0, r5, asl #1
	ldrh	r0, [r9, r0]
	cmp	r0, r1
	bne	.L2519
	sub	r3, r3, #1
	ldrh	r0, [r4]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L2517
.L2519:
	ldr	r7, .L2533
	orr	r5, r5, r2, asl #10
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
.L2520:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	r2, #65535
	ldrh	r0, [r6]
.L2522:
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, r0
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	add	r1, r4, r3, asl #1
	ldrh	r1, [r1, #16]
	cmp	r1, r2
	beq	.L2522
	strb	r3, [r4, #6]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L2527
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	ldr	r2, .L2533
	cmp	r3, #0
	bne	.L2524
	ldrh	r3, [r4, #2]
	add	r1, r2, #588
	mov	r3, r3, asl #1
	ldrh	r1, [r1, r3]
	movw	r3, #65535
	cmp	r1, r3
	bne	.L2524
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L2524
	sub	r3, r3, #1
	ldrh	r0, [r4]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L2520
.L2524:
	ldrb	r3, [r2, #1888]	@ zero_extendqisi2
	ldr	r1, .L2533
	cmp	r3, #0
	beq	.L2527
	movw	r3, #2910
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L2527
	ldr	r2, .L2533+8
	ldrh	r3, [r4]
	ldrh	ip, [r4, #4]
	ldr	r0, [r2, #-3028]
	mov	r3, r3, asl #1
	ldrh	r2, [r0, r3]
	rsb	r2, ip, r2
	strh	r2, [r0, r3]	@ movhi
	movw	r2, #2908
	mov	r3, #0
	ldrh	r2, [r1, r2]
	strh	r3, [r4, #4]	@ movhi
	strb	r3, [r4, #6]
	strh	r2, [r4, #2]	@ movhi
.L2527:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L2534:
	.align	2
.L2533:
	.word	.LANCHOR0
	.word	.LANCHOR0+2840
	.word	.LANCHOR2
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L2553
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r6, [ip, #-3100]
	cmp	r6, #0
	bne	.L2537
	ldr	r3, [ip, #-2060]
	sub	r5, ip, #2784
	ldr	r4, [ip, #-2036]
	sub	r7, r5, #300
	mov	r1, #255
	mov	fp, ip
	str	r3, [ip, #224]
	ldrh	r3, [r5]
	str	r4, [ip, #228]
	str	r6, [r4, #12]
	strh	r3, [r4, #2]	@ movhi
	ldr	r3, .L2553+4
	strh	r3, [r4]	@ movhi
	ldr	r3, [ip, #-2776]
	stmib	r4, {r3, r6}
	ldr	r3, .L2553+8
	ldr	r6, .L2553+12
	str	r3, [ip, #-3084]
	ldr	r3, .L2553+16
	str	r3, [ip, #-3080]
	ldrh	r3, [r5, #6]
	strh	r3, [r7, #8]	@ movhi
	movw	r3, #2862
	ldrh	r3, [r6, r3]
	strb	r3, [ip, #-3074]
	sub	r3, ip, #3008
	ldrh	r2, [r3]
	strh	r2, [r7, #14]	@ movhi
	ldrh	r2, [r3, #2]
	ldrb	r3, [ip, #-3002]	@ zero_extendqisi2
	orr	r3, r3, r2, asl #6
	strh	r3, [r7, #16]	@ movhi
	ldrb	r3, [ip, #-3000]	@ zero_extendqisi2
	strb	r3, [ip, #-3073]
	sub	r3, ip, #2960
	ldrh	r2, [r3]
	strh	r2, [r7, #18]	@ movhi
	ldrh	r2, [r3, #2]
	ldrb	r3, [ip, #-2954]	@ zero_extendqisi2
	orr	r3, r3, r2, asl #6
	strh	r3, [r7, #20]	@ movhi
	ldrb	r3, [ip, #-2952]	@ zero_extendqisi2
	strb	r3, [ip, #-3072]
	sub	r3, ip, #2912
	ldrh	r2, [r3]
	strh	r2, [r7, #22]	@ movhi
	ldrh	r2, [r3, #2]
	ldrb	r3, [ip, #-2906]	@ zero_extendqisi2
	ldr	r0, [ip, #224]
	orr	r3, r3, r2, asl #6
	strh	r3, [r7, #24]	@ movhi
	ldrb	r3, [ip, #-2904]	@ zero_extendqisi2
	strb	r3, [ip, #-3071]
	ldr	r3, [ip, #-2808]
	str	r3, [ip, #-3052]
	ldr	r3, [ip, #-2816]
	str	r3, [ip, #-3044]
	ldr	r3, [ip, #-2812]
	str	r3, [ip, #-3048]
	sub	r3, ip, #2144
	ldrh	r2, [r3, #-6]
	ldrh	r3, [r3, #-4]
	strh	r2, [r7, #44]	@ movhi
	strh	r3, [r5, #-254]	@ movhi
	movw	r3, #2918
	ldrh	r2, [r6, r3]
	bl	ftl_memset
	mov	r1, r7
	add	r7, r6, #2848
	mov	r2, #48
	ldr	r0, [fp, #224]
	bl	ftl_memcpy
	ldrh	r2, [r7]
	ldr	r0, [fp, #224]
	ldr	r1, [fp, #-3028]
	mov	r2, r2, asl #1
	add	r0, r0, #48
	bl	ftl_memcpy
	ldrh	r2, [r7]
	ldr	r0, [fp, #224]
	add	r3, r2, #24
	ldr	r1, [fp, #-2852]
	ubfx	r3, r3, #1, #30
	mov	r2, r2, lsr #3
	add	r2, r2, #4
	add	r0, r0, r3, asl #2
	bl	ftl_memcpy
	movw	r3, #2948
	ldrh	r3, [r6, r3]
	cmp	r3, #0
	beq	.L2538
	ldrh	r2, [r7]
	ldr	r0, [fp, #224]
	ldr	r1, [fp, #-1988]
	mov	r3, r2, lsr #3
	add	r3, r3, r2, asl #1
	add	r2, r6, #2944
	add	r3, r3, #52
	ldrh	r2, [r2]
	ubfx	r3, r3, #2, #14
	add	r0, r0, r3, asl #2
	mov	r2, r2, asl #2
	bl	ftl_memcpy
.L2538:
	mov	r0, #0
	ldr	r8, .L2553
	bl	FtlUpdateVaildLpn
	ldr	r10, .L2553+20
	mov	r7, #0
	movw	r9, #65535
.L2539:
	ldrh	r1, [r5]
	ldrh	r0, [r5, #2]
	ldr	r3, [fp, #-2060]
	orr	r2, r0, r1, asl #10
	str	r2, [fp, #220]
	ldrh	r2, [r10]
	str	r3, [fp, #224]
	sub	r2, r2, #1
	ldr	r3, [fp, #-2036]
	cmp	r0, r2
	str	r3, [fp, #228]
	ldr	r3, .L2553+24
	blt	.L2540
	ldrh	r9, [r3, #4]
	mov	r2, #0
	strh	r1, [r3, #4]	@ movhi
	strh	r2, [r3, #2]	@ movhi
	str	r3, [sp, #4]
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [sp, #4]
	strh	r0, [r3]	@ movhi
	ldr	r3, [r8, #-2816]
	add	r2, r3, #1
	str	r2, [r8, #-2816]
	str	r3, [r8, #-2776]
	mov	r2, r0, asl #10
	str	r2, [r8, #220]
	str	r3, [r4, #4]
	strh	r0, [r4, #2]	@ movhi
.L2540:
	ldrb	r3, [r6]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2541
	ldr	r3, .L2553+28
	ldr	r0, [fp, #-2060]
	ldrh	r1, [r3]
	bl	js_hash
	str	r0, [r4, #12]
.L2541:
	mov	r1, #1
	ldr	r0, .L2553+32
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r5, #2]
	ldr	r2, [fp, #216]
	add	r3, r3, #1
	cmn	r2, #1
	uxth	r3, r3
	strh	r3, [r5, #2]	@ movhi
	bne	.L2542
	cmp	r3, #1
	add	r7, r7, #1
	ldreq	r3, .L2553+20
	uxth	r7, r7
	ldreq	r2, .L2553+24
	ldreqh	r3, [r3]
	subeq	r3, r3, #1
	streqh	r3, [r2, #2]	@ movhi
	cmp	r7, #3
	bls	.L2539
	ldr	r0, .L2553+36
	mov	r2, r7
	ldr	r1, [fp, #220]
	bl	printk
	mov	r3, #1
	str	r3, [fp, #-3100]
	b	.L2537
.L2542:
	cmp	r2, #256
	cmpne	r3, #1
	beq	.L2539
	movw	r3, #65535
	cmp	r9, r3
	beq	.L2537
	mov	r0, r9
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L2537:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2554:
	.align	2
.L2553:
	.word	.LANCHOR2
	.word	-3932
	.word	1179929683
	.word	.LANCHOR0
	.word	1342177366
	.word	.LANCHOR0+2910
	.word	.LANCHOR2-2784
	.word	.LANCHOR0+2918
	.word	.LANCHOR2+216
	.word	.LC128
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r5, .L2572
	ldr	r3, [r5, #-3100]
	cmp	r3, #0
	bne	.L2555
	ldr	r8, .L2572+4
	ldrb	r6, [r8, #1888]	@ zero_extendqisi2
	cmp	r6, #0
	beq	.L2571
	ldrb	r6, [r0, #8]	@ zero_extendqisi2
	cmp	r6, #1
	ldreqh	fp, [r0, #4]
	beq	.L2557
	mov	r6, r3
.L2571:
	mov	fp, #12
.L2557:
	ldr	r7, [r5, #-2036]
	mvn	r3, #0
	str	r3, [sp, #20]
	mov	r9, #0
	ldr	r3, [r5, #-2060]
	movw	r2, #61589
	str	r7, [sp, #16]
	mov	r4, r0
	ldr	r10, .L2572
	str	r3, [sp, #12]
	mvn	r3, #2
	str	r3, [r7, #8]
	mvn	r3, #1
	str	r3, [r7, #12]
	ldrh	r3, [r0]
	strh	r9, [r7]	@ movhi
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, [r5, #-2060]
	str	r2, [r3]
	ldr	r2, .L2572+8
	ldr	r3, [r5, #-2060]
	str	r2, [r3, #4]
.L2558:
	subs	fp, fp, #1
	bcc	.L2561
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L2559
.L2561:
	ldrh	r3, [r4]
	ldr	r1, [r5, #-3028]
	ldrh	r0, [r4, #4]
	mov	r3, r3, asl #1
	ldrh	r2, [r1, r3]
	rsb	r2, r0, r2
	strh	r2, [r1, r3]	@ movhi
	movw	r3, #2908
	ldrh	r3, [r8, r3]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	b	.L2555
.L2559:
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [sp, #8]
	beq	.L2561
	ldr	r3, [r5, #-2812]
	mov	r1, #1
	mov	r2, r6
	add	r0, sp, #4
	str	r3, [r7, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, r9
	str	r3, [r10, #-2812]
	mov	r3, #0
	bl	FlashProgPages
	ldrh	r0, [r4]
	bl	decrement_vpc_count
	b	.L2558
.L2555:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2573:
	.align	2
.L2572:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	305419896
	.fnend
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	ftl_map_blk_gc
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r0
	ldr	r5, [r0, #12]
	ldr	r7, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	sub	r3, r3, #4
	ldr	r8, .L2588
	cmp	r2, r3
	blt	.L2575
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r10, [r5, r0]
	cmp	r10, #0
	beq	.L2575
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L2575
	mov	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	movw	r3, #2910
	ldrh	r3, [r8, r3]
	cmp	r2, r3
	bcc	.L2576
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L2576:
	ldr	r5, .L2588+4
	mov	fp, #0
.L2577:
	ldrh	r3, [r4, #6]
	uxth	r6, fp
	cmp	r3, r6
	bls	.L2587
	ldr	r3, [r7, r6, asl #2]
	add	ip, r7, r6, asl #2
	cmp	r10, r3, lsr #10
	bne	.L2578
	ldr	r3, [r5, #-2056]
	mov	r1, #1
	ldr	r9, [r5, #-2036]
	mov	r2, r1
	ldr	r0, .L2588+8
	str	r3, [r5, #224]
	str	r9, [r5, #228]
	ldr	r3, [r7, r6, asl #2]
	str	ip, [sp, #4]
	str	r3, [r5, #220]
	bl	FlashReadPages
	ldr	r3, [r5, #216]
	cmn	r3, #1
	ldr	r3, .L2588+4
	ldr	ip, [sp, #4]
	bne	.L2579
.L2581:
	mov	r2, #0
	ldr	r0, .L2588+12
	str	r2, [ip]
	ldr	r1, [r3, #220]
	ldrh	r2, [r9, #8]
	str	r3, [sp, #4]
	bl	printk
	mov	r2, #1
	ldr	r3, [sp, #4]
	str	r2, [r3, #-3100]
	b	.L2580
.L2579:
	ldrh	r1, [r9, #8]
	cmp	r1, r6
	bne	.L2581
	ldrh	r0, [r9]
	ldrh	r2, [r4, #4]
	cmp	r0, r2
	bne	.L2581
	mov	r0, r4
	ldr	r2, [r5, #224]
	bl	FtlMapWritePage
.L2578:
	add	fp, fp, #1
	b	.L2577
.L2587:
	mov	r0, r10
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L2575:
	movw	r3, #2910
	ldrh	r2, [r4, #2]
	ldrh	r3, [r8, r3]
	cmp	r2, r3
	bcc	.L2580
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L2580:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2589:
	.align	2
.L2588:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+216
	.word	.LC129
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	ldr	r5, .L2602
	ldr	r7, [r0, #12]
	ldr	r9, [r0, #24]
	ldr	r6, [r5, #-3100]
	cmp	r6, #0
	bne	.L2591
	ldrh	r3, [r0]
	movw	r2, #65535
	mov	r4, r0
	cmp	r3, r2
	bne	.L2592
	ldrh	r3, [r0, #8]
	add	r3, r3, #1
	strh	r3, [r0, #8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	strh	r0, [r7]	@ movhi
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	strh	r6, [r4]	@ movhi
	str	r3, [r4, #28]
	b	.L2591
.L2592:
	mov	r3, r3, asl #1
	ldr	r10, .L2602+4
	mov	r1, #255
	ldrh	r8, [r7, r3]
	ldrh	r3, [r0, #2]
	ldr	r7, [r5, #-2036]
	orr	r3, r3, r8, asl #10
	str	r3, [r5, #220]
	ldr	r3, [r5, #-2060]
	str	r7, [r5, #228]
	str	r3, [r5, #224]
	ldr	r3, [r0, #28]
	str	r3, [r7, #4]
	ldr	r3, .L2602+8
	strh	r3, [r7, #8]	@ movhi
	ldrh	r3, [r0, #4]
	strh	r8, [r7, #2]	@ movhi
	strh	r3, [r7]	@ movhi
	movw	r3, #2910
	ldrh	r2, [r10, r3]
	ldr	r0, [r5, #-2060]
	mov	r2, r2, asl #3
	bl	ftl_memset
	mov	r2, r6
.L2593:
	ldrh	r1, [r4, #6]
	uxth	r3, r2
	cmp	r1, r3
	bls	.L2601
	ldr	r1, [r9, r3, asl #2]
	cmp	r8, r1, lsr #10
	bne	.L2594
	add	r6, r6, #1
	ldr	r1, [r5, #-2060]
	uxth	r6, r6
	str	r3, [r1, r6, asl #3]
	ldr	r1, [r9, r3, asl #2]
	ldr	r3, [r5, #-2060]
	add	r3, r3, r6, asl #3
	str	r1, [r3, #4]
.L2594:
	add	r2, r2, #1
	b	.L2593
.L2601:
	ldrb	r3, [r10]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2596
	ldr	r2, .L2602+4
	movw	r3, #2918
	ldr	r0, [r5, #224]
	ldrh	r1, [r2, r3]
	bl	js_hash
	str	r0, [r7, #12]
.L2596:
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L2602+12
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L2591:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2603:
	.align	2
.L2602:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1291
	.word	.LANCHOR2+216
	.fnend
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	FtlMapWritePage
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r0
	ldr	r10, .L2623
	mov	r8, r1
	ldr	fp, .L2623+4
	mov	r5, #0
	str	r2, [sp]
	mov	r9, r10
.L2605:
	ldr	r3, [r10, #-2832]
	ldr	r6, .L2623
	add	r3, r3, #1
	str	r3, [r10, #-2832]
	ldrh	r3, [fp]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L2606
	ldrh	r2, [r4]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2607
.L2606:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L2607:
	ldr	r3, [r9, #-3100]
	cmp	r3, #0
	bne	.L2608
	ldrh	r3, [r4]
	mov	r1, #16
	ldr	r2, [r4, #12]
	ldr	r0, [r10, #-2036]
	mov	r3, r3, asl #1
	ldrh	r7, [r2, r3]
	ldrh	r3, [r4, #2]
	str	r0, [r10, #228]
	orr	r3, r3, r7, asl #10
	str	r3, [r10, #220]
	ldr	r3, [sp]
	str	r3, [r10, #224]
	bl	__memzero
	ldr	r2, [r4, #28]
	ldr	r3, [r10, #228]
	str	r2, [r3, #4]
	strh	r8, [r3, #8]	@ movhi
	ldrh	r2, [r4, #4]
	strh	r7, [r3, #2]	@ movhi
	strh	r2, [r3]	@ movhi
	ldr	r2, .L2623+8
	ldrb	r2, [r2]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2609
	ldr	r2, .L2623+12
	ldr	r0, [r10, #224]
	str	r3, [sp, #4]
	ldrh	r1, [r2]
	bl	js_hash
	ldr	r3, [sp, #4]
	str	r0, [r3, #12]
.L2609:
	mov	r1, #1
	ldr	r0, .L2623+16
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #2]	@ movhi
	ldr	r2, [r9, #216]
	cmn	r2, #1
	bne	.L2610
	ldr	r0, .L2623+20
	add	r5, r5, #1
	ldr	r1, [r10, #220]
	bl	printk
	ldrh	r3, [r4, #2]
	uxth	r5, r5
	cmp	r3, #2
	ldrls	r3, .L2623+4
	ldrlsh	r3, [r3]
	subls	r3, r3, #1
	strlsh	r3, [r4, #2]	@ movhi
	cmp	r5, #3
	bls	.L2605
	ldr	r0, .L2623+24
	mov	r2, r5
	ldr	r1, [r6, #220]
	bl	printk
	mov	r3, #1
	str	r3, [r6, #-3100]
	b	.L2608
.L2610:
	cmp	r2, #0
	strneh	r7, [r4, #40]	@ movhi
	cmp	r2, #256
	cmpne	r3, #1
	beq	.L2614
	ldr	r3, [r4, #36]
	cmp	r3, #0
	beq	.L2615
.L2614:
	mov	r3, #0
	str	r3, [r4, #36]
	b	.L2605
.L2615:
	ldr	r2, [r6, #220]
	ldr	r3, [r4, #24]
	str	r2, [r3, r8, asl #2]
.L2608:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2624:
	.align	2
.L2623:
	.word	.LANCHOR2
	.word	.LANCHOR0+2910
	.word	.LANCHOR0
	.word	.LANCHOR0+2918
	.word	.LANCHOR2+216
	.word	.LC130
	.word	.LC131
	.fnend
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	2
	.global	flush_l2p_region
	.type	flush_l2p_region, %function
flush_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, #12
	ldr	r5, .L2627
	mul	r4, r4, r0
	sub	r0, r5, #1968
	ldr	r3, [r5, #-2864]
	add	r2, r3, r4
	ldrh	r1, [r3, r4]
	ldr	r2, [r2, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #-2864]
	mov	r0, #0
	add	r4, r3, r4
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp!, {r3, r4, r5, pc}
.L2628:
	.align	2
.L2627:
	.word	.LANCHOR2
	.fnend
	.size	flush_l2p_region, .-flush_l2p_region
	.align	2
	.global	FtlMapTblRecovery
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r3, [r0, #16]
	mov	r4, r0
	ldrh	r10, [r0, #6]
	mov	r1, #0
	ldr	r8, [r0, #24]
	mov	r6, #0
	ldr	r5, .L2672
	str	r3, [sp, #4]
	mov	r2, r10, asl #2
	ldrh	r3, [r0, #8]
	ldr	r9, [r0, #12]
	mov	r0, r8
	ldr	fp, .L2672+4
	str	r3, [sp]
	bl	ftl_memset
	ldr	r3, [r5, #-2060]
	ldr	r7, [r5, #-2036]
	str	r6, [r4, #32]
	str	r3, [r5, #224]
	mvn	r3, #0
	str	r7, [r5, #228]
	str	r6, [r4, #28]
	strh	r3, [r4]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #1
	str	r3, [r4, #36]
.L2630:
	uxth	r2, r6
	ldr	r1, [sp]
	sxth	r3, r2
	cmp	r3, r1
	bge	.L2648
	ldr	r1, [sp]
	sub	r1, r1, #1
	cmp	r3, r1
	mov	r1, r3, asl #1
	bne	.L2631
	ldrh	r0, [r9, r1]
	mov	r6, #0
	str	r3, [sp, #12]
	add	r3, r9, r1
	mov	r1, #1
	str	r2, [sp, #8]
	str	r3, [sp]
	bl	FtlGetLastWrittenPage
	ldr	fp, .L2672
	ldr	r2, [sp, #8]
	add	r1, r0, #1
	ldr	r3, [sp, #12]
	sxth	r0, r0
	strh	r1, [r4, #2]	@ movhi
	add	r9, r0, #1
	strh	r2, [r4]	@ movhi
	ldr	r2, [sp, #4]
	ldr	r3, [r2, r3, asl #2]
	str	r3, [r4, #28]
.L2632:
	sxth	r3, r6
	cmp	r3, r9
	bge	.L2648
	ldr	r2, [sp]
	mov	r1, #1
	ldr	r0, .L2672+8
	ldrh	r2, [r2]
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #220]
	bl	FlashReadPages
	ldr	r3, .L2672+12
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2633
	ldr	r3, [fp, #228]
	ldr	r3, [r3, #12]
	cmp	r3, #0
	beq	.L2633
	ldr	r2, .L2672+16
	ldr	r0, [fp, #224]
	str	r3, [sp, #4]
	ldrh	r1, [r2]
	bl	js_hash
	ldr	r3, [sp, #4]
	cmp	r3, r0
	mvnne	r3, #0
	strne	r3, [fp, #216]
.L2633:
	ldr	r3, [fp, #216]
	cmn	r3, #1
	beq	.L2634
	ldrh	r3, [r7, #8]
	cmp	r3, r10
	bcs	.L2634
	ldrh	r2, [r4, #4]
	ldrh	r1, [r7]
	cmp	r1, r2
	ldreq	r2, [fp, #220]
	streq	r2, [r8, r3, asl #2]
.L2634:
	add	r6, r6, #1
	b	.L2632
.L2648:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r1, .L2672+12
	movw	r3, #2910
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bne	.L2637
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
	b	.L2637
.L2631:
	ldr	r3, [r5, #-2060]
	ldr	r0, .L2672+8
	str	r3, [r5, #224]
	add	r3, r9, r1
	ldrh	r2, [r9, r1]
	mov	r1, #1
	str	r3, [sp, #8]
	ldrh	r3, [fp]
	sub	r3, r3, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #220]
	bl	FlashReadPages
	ldr	r3, [r5, #216]
	cmn	r3, #1
	beq	.L2650
	ldrh	r2, [r7]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L2650
	ldrh	r2, [r7, #8]
	movw	r3, #64245
	cmp	r2, r3
	bne	.L2650
	mov	r1, #0
	mov	ip, #8
	mov	lr, #4
.L2639:
	ldr	r0, .L2672+4
	uxth	r3, r1
	sxth	r2, r3
	ldrh	r0, [r0]
	sub	r0, r0, #1
	cmp	r2, r0
	bge	.L2642
	ldr	r0, [r5, #-2060]
	add	r1, r1, #1
	ldr	r2, [r0, r2, asl #3]
	uxth	r2, r2
	cmp	r2, r10
	smlabbcc	r3, r3, ip, lr
	ldrcc	r3, [r0, r3]
	strcc	r3, [r8, r2, asl #2]
	b	.L2639
.L2650:
	ldr	ip, .L2672
	mov	r3, #0
.L2670:
	ldrh	r1, [fp]
	sxth	r2, r3
	cmp	r2, r1
	bge	.L2642
	str	r3, [sp, #16]
	ldr	r3, [sp, #8]
	ldr	r0, .L2672+8
	str	ip, [sp, #12]
	ldrh	r1, [r3]
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [ip, #220]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, .L2672+12
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #0
	ldr	ip, [sp, #12]
	ldr	r3, [sp, #16]
	beq	.L2643
	ldr	r2, [ip, #228]
	ldr	r2, [r2, #12]
	cmp	r2, #0
	beq	.L2643
	ldr	r1, .L2672+16
	ldr	r0, [ip, #224]
	str	r2, [sp, #20]
	ldrh	r1, [r1]
	bl	js_hash
	ldr	r2, [sp, #20]
	ldr	ip, [sp, #12]
	ldr	r3, [sp, #16]
	cmp	r2, r0
	mvnne	r2, #0
	strne	r2, [ip, #216]
.L2643:
	ldr	r2, .L2672
	ldr	r2, [r2, #216]
	cmn	r2, #1
	beq	.L2644
	ldrh	r2, [r7, #8]
	cmp	r2, r10
	bcs	.L2644
	ldrh	r1, [r4, #4]
	ldrh	r0, [r7]
	cmp	r0, r1
	ldreq	r1, [ip, #220]
	streq	r1, [r8, r2, asl #2]
.L2644:
	add	r3, r3, #1
	b	.L2670
.L2642:
	add	r6, r6, #1
	b	.L2630
.L2637:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2673:
	.align	2
.L2672:
	.word	.LANCHOR2
	.word	.LANCHOR0+2910
	.word	.LANCHOR2+216
	.word	.LANCHOR0
	.word	.LANCHOR0+2918
	.fnend
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	2
	.global	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L2676
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	add	r1, r2, #2928
	ldr	r3, .L2676+4
	ldrh	r1, [r1]
	add	r0, r3, #256
	strh	r1, [r0, #10]	@ movhi
	ldr	r1, .L2676+8
	strh	r1, [r0, #4]	@ movhi
	movw	r1, #2950
	ldrh	r1, [r2, r1]
	strh	r1, [r0, #8]	@ movhi
	movw	r1, #2930
	ldrh	r1, [r2, r1]
	ldr	r2, [r2, #2952]
	strh	r1, [r0, #6]	@ movhi
	str	r2, [r3, #268]
	ldr	r2, [r3, #-1996]
	str	r2, [r3, #272]
	ldr	r2, [r3, #-2000]
	str	r2, [r3, #276]
	ldr	r2, [r3, #-1992]
	str	r2, [r3, #280]
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2677:
	.align	2
.L2676:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-3962
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FtlLoadMapInfo
	.type	FtlLoadMapInfo, %function
FtlLoadMapInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	FtlL2PDataInit
	ldr	r0, .L2680
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2681:
	.align	2
.L2680:
	.word	.LANCHOR2-1968
	.fnend
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2693
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r2
	movw	r2, #2916
	mov	r4, r1
	ldrh	r5, [r3, r2]
	add	r2, r0, r1
	movw	r1, #2904
	mov	r6, r0
	ldrh	r3, [r3, r1]
	.pad #60
	sub	sp, sp, #60
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L2683
	ldr	fp, .L2693+4
	mov	r5, r6, lsr r5
	mov	r3, r5, asl #2
	str	r3, [sp, #4]
	mov	r7, fp
	mov	r3, #0
	str	r3, [sp]
.L2684:
	cmp	r4, #0
	beq	.L2692
	ldr	r2, [sp, #4]
	mov	r0, r6
	ldr	r3, [fp, #-1992]
	ldr	ip, [r3, r2]
	ldr	r3, .L2693+8
	str	ip, [sp, #12]
	ldrh	r2, [r3]
	mov	r1, r2
	str	r2, [sp, #8]
	bl	__aeabi_uidivmod
	ldr	r2, [sp, #8]
	mov	r8, r1
	ldr	ip, [sp, #12]
	rsb	r3, r1, r2
	uxth	r9, r3
	cmp	r9, r4
	uxthhi	r9, r4
	cmp	r9, r2
	cmpne	ip, #0
	movne	r1, #1
	moveq	r1, #0
	beq	.L2686
	ldr	r2, [r7, #-2052]
	mov	r1, #1
	add	r0, sp, #20
	str	ip, [sp, #24]
	str	r2, [sp, #28]
	mov	r2, #0
	str	r2, [sp, #32]
	mov	r2, r1
	bl	FlashReadPages
	b	.L2687
.L2686:
	ldr	r2, .L2693+12
	ldr	r0, [r7, #-2052]
	ldrh	r2, [r2]
	bl	ftl_memset
.L2687:
	mov	ip, r9, asl #9
	ldr	r0, [r7, #-2052]
	uxth	r8, r8
	mov	r1, r10
	mov	r2, ip
	str	ip, [sp, #8]
	add	r0, r0, r8, asl #9
	rsb	r4, r9, r4
	bl	ftl_memcpy
	mov	r1, r5
	ldr	r0, .L2693+16
	add	r5, r5, #1
	ldr	r2, [r7, #-2052]
	add	r6, r6, r9
	bl	FtlMapWritePage
	ldr	r3, [sp]
	cmn	r0, #1
	ldr	ip, [sp, #8]
	mvneq	r3, #0
	str	r3, [sp]
	add	r10, r10, ip
	ldr	r3, [sp, #4]
	add	r3, r3, #4
	str	r3, [sp, #4]
	b	.L2684
.L2692:
	ldr	r0, [sp]
.L2683:
	add	sp, sp, #60
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2694:
	.align	2
.L2693:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2914
	.word	.LANCHOR0+2918
	.word	.LANCHOR2+256
	.fnend
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	2
	.global	Ftl_save_ext_data
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L2697
	ldr	r2, .L2697+4
	ldr	r1, [r3, #-2716]
	cmp	r1, r2
	bxne	lr
	ldr	r2, .L2697+8
	mov	r0, #0
	mov	r1, #1
	str	r2, [r3, #-2712]
	ldr	r2, [r3, #-2824]
	str	r2, [r3, #-2628]
	ldr	r2, [r3, #-2820]
	str	r2, [r3, #-2624]
	ldr	r2, [r3, #-2828]
	str	r2, [r3, #-2708]
	ldr	r2, [r3, #-2840]
	str	r2, [r3, #-2704]
	ldr	r2, [r3, #-2848]
	str	r2, [r3, #-2700]
	ldr	r2, [r3, #-2832]
	str	r2, [r3, #-2696]
	ldr	r2, [r3, #-2804]
	str	r2, [r3, #-2688]
	ldr	r2, [r3, #-3088]
	str	r2, [r3, #-2684]
	ldr	r2, [r3, #-2844]
	str	r2, [r3, #-2680]
	ldr	r2, [r3, #-2836]
	str	r2, [r3, #-2676]
	ldr	r2, [r3, #-2796]
	str	r2, [r3, #-2672]
	ldr	r2, [r3, #-2792]
	str	r2, [r3, #-2668]
	ldr	r2, [r3, #-2720]
	str	r2, [r3, #-2656]
	ldr	r2, [r3, #-2100]
	str	r2, [r3, #-2652]
	ldr	r2, .L2697+12
	b	FtlVendorPartWrite
.L2698:
	.align	2
.L2697:
	.word	.LANCHOR2
	.word	1179929683
	.word	1342177366
	.word	.LANCHOR2-2716
	.fnend
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L2707
	ldr	r2, .L2707+4
	ldr	r3, [r3, #2832]
	cmp	r3, #0
	moveq	r3, #32
	beq	.L2700
	ldr	r3, [r2, #-2792]
	cmp	r3, #39
	movhi	r3, #32
	movls	r3, #4
.L2700:
	ldr	ip, .L2707+8
	ldrh	r1, [ip]
	cmp	r1, #31
	addls	r1, r1, #1
	strlsh	r1, [ip]	@ movhi
	movls	r3, #1
	cmp	r0, #0
	bne	.L2702
	ldr	r1, [r2, #-2016]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r3, r3, r0
	cmp	r1, r3
	bcc	.L2703
.L2702:
	ldr	r3, [r2, #-2016]
	mov	r0, #64
	ldr	r1, [r3, #16]
	str	r1, [r3, #20]
	ldr	r1, .L2707+12
	str	r1, [r3]
	ldr	r3, .L2707+16
	ldr	r2, [r2, #-2016]
	ldrh	r1, [r3, #-8]
	mov	r3, r1, asl #9
	str	r3, [r2, #12]
	ldr	r3, [r2, #8]
	add	r3, r3, #1
	str	r3, [r2, #8]
	mov	r3, #0
	str	r3, [r2, #4]
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L2703:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2708:
	.align	2
.L2707:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+300
	.word	1112818501
	.word	.LANCHOR2-2016
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #2916
	ldr	r4, .L2721
	mov	r8, r2
	add	r2, r0, r1
	mov	r7, r0
	.pad #60
	sub	sp, sp, #60
	mov	r6, r1
	ldrh	r5, [r4, r3]
	movw	r3, #2904
	ldrh	r3, [r4, r3]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L2710
	ldr	r9, .L2721+4
	mov	r5, r7, lsr r5
	mov	r3, r5, asl #2
	str	r3, [sp, #4]
	mov	r10, r9
	mov	r3, #0
	str	r3, [sp]
.L2711:
	cmp	r6, #0
	beq	.L2720
	ldr	r2, [sp, #4]
	mov	r0, r7
	ldr	r3, [r9, #-1992]
	ldr	r3, [r3, r2]
	str	r3, [sp, #12]
	ldr	r3, .L2721+8
	ldrh	r4, [r3]
	mov	r1, r4
	bl	__aeabi_uidivmod
	rsb	r4, r1, r4
	ldr	r3, [sp, #12]
	str	r1, [sp, #8]
	uxth	r4, r4
	cmp	r4, r6
	uxthhi	r4, r6
	cmp	r3, #0
	mov	fp, r4, asl #9
	beq	.L2713
	ldr	r2, [r10, #-2052]
	mov	r1, #1
	add	r0, sp, #20
	str	r3, [sp, #24]
	str	r3, [sp, #12]
	str	r2, [sp, #28]
	mov	r2, #0
	str	r2, [sp, #32]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [sp, #20]
	ldr	r3, [sp]
	cmn	r2, #1
	ldr	r2, [r10, #216]
	mvneq	r3, #0
	cmp	r2, #256
	str	r3, [sp]
	ldr	r3, [sp, #12]
	bne	.L2715
	mov	r1, r5
	mov	r2, r3
	ldr	r0, .L2721+12
	bl	printk
	ldr	r0, .L2721+16
	mov	r1, r5
	ldr	r2, [r9, #-2052]
	bl	FtlMapWritePage
.L2715:
	ldrh	r3, [sp, #8]
	mov	r0, r8
	ldr	r1, [r10, #-2052]
	mov	r2, fp
	add	r1, r1, r3, asl #9
	bl	ftl_memcpy
	b	.L2716
.L2713:
	mov	r0, r8
	mov	r1, r3
	mov	r2, fp
	bl	ftl_memset
.L2716:
	ldr	r3, [sp, #4]
	add	r5, r5, #1
	rsb	r6, r4, r6
	add	r7, r7, r4
	add	r3, r3, #4
	add	r8, r8, fp
	str	r3, [sp, #4]
	b	.L2711
.L2720:
	ldr	r0, [sp]
.L2710:
	add	sp, sp, #60
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2722:
	.align	2
.L2721:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2914
	.word	.LC132
	.word	.LANCHOR2+256
	.fnend
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	2
	.global	FtlLoadEctTbl
	.type	FtlLoadEctTbl, %function
FtlLoadEctTbl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r0, #64
	ldr	r4, .L2726
	sub	r5, r4, #2016
	ldr	r2, [r4, #-2016]
	ldrh	r1, [r5, #-8]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #-2016]
	ldr	r2, [r3]
	ldr	r3, .L2726+4
	cmp	r2, r3
	beq	.L2724
	ldr	r1, .L2726+8
	ldr	r0, .L2726+12
	bl	printk
	ldrh	r2, [r5, #-8]
	ldr	r0, [r4, #-2016]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2724:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L2727:
	.align	2
.L2726:
	.word	.LANCHOR2
	.word	1112818501
	.word	.LC133
	.word	.LC76
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	Ftl_load_ext_data
	.type	Ftl_load_ext_data, %function
Ftl_load_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r0, #0
	ldr	r4, .L2733
	mov	r1, #1
	ldr	r2, .L2733+4
	bl	FtlVendorPartRead
	ldr	r5, .L2733+8
	ldr	r3, [r4, #-2716]
	cmp	r3, r5
	beq	.L2729
	ldr	r0, .L2733+4
	mov	r1, #512
	bl	__memzero
	str	r5, [r4, #-2716]
.L2729:
	ldr	r2, [r4, #-2716]
	ldr	r3, .L2733
	cmp	r2, r5
	bne	.L2730
	ldr	r2, [r3, #-2628]
	str	r2, [r3, #-2824]
	ldr	r2, [r3, #-2624]
	str	r2, [r3, #-2820]
	ldr	r2, [r3, #-2708]
	str	r2, [r3, #-2828]
	ldr	r2, [r3, #-2704]
	str	r2, [r3, #-2840]
	ldr	r2, [r3, #-2700]
	str	r2, [r3, #-2848]
	ldr	r2, [r3, #-2696]
	str	r2, [r3, #-2832]
	ldr	r2, [r3, #-2688]
	str	r2, [r3, #-2804]
	ldr	r2, [r3, #-2684]
	str	r2, [r3, #-3088]
	ldr	r2, [r3, #-2680]
	str	r2, [r3, #-2844]
	ldr	r2, [r3, #-2676]
	str	r2, [r3, #-2836]
	ldr	r2, [r3, #-2672]
	str	r2, [r3, #-2796]
	ldr	r2, [r3, #-2668]
	str	r2, [r3, #-2792]
	ldr	r2, [r3, #-2656]
	str	r2, [r3, #-2720]
.L2730:
	mov	r3, #0
	ldr	r2, [r4, #-2648]
	str	r3, [r4, #-2100]
	ldr	r3, .L2733+12
	ldr	r5, .L2733+16
	cmp	r2, r3
	bne	.L2731
	ldr	r0, .L2733+20
	mov	r3, #1
	ldr	r1, .L2733+24
	str	r3, [r5, #2832]
	bl	printk
.L2731:
	movw	r3, #2900
	ldr	r0, [r4, #-2808]
	ldrh	r2, [r5, r3]
	ldr	r3, [r4, #-2804]
	mla	r0, r0, r2, r3
	ldr	r3, .L2733+28
	ldrh	r1, [r3]
	bl	__aeabi_uidiv
	str	r0, [r4, #-2800]
	ldmfd	sp!, {r3, r4, r5, pc}
.L2734:
	.align	2
.L2733:
	.word	.LANCHOR2
	.word	.LANCHOR2-2716
	.word	1179929683
	.word	305432421
	.word	.LANCHOR0
	.word	.LC76
	.word	.LC134
	.word	.LANCHOR0+2848
	.fnend
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	2
	.global	FtlMapBlkWriteDumpData
	.type	FtlMapBlkWriteDumpData, %function
FtlMapBlkWriteDumpData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r2, [r0, #36]
	ldrh	r5, [r0, #6]
	cmp	r2, #0
	ldr	r3, [r0, #24]
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldr	r4, .L2742
	mov	r2, #0
	str	r2, [r0, #36]
	ldr	r2, [r4, #-3100]
	cmp	r2, #0
	ldmnefd	sp!, {r4, r5, r6, pc}
	sub	r5, r5, #1
	mov	r6, r0
	ldr	r2, [r4, #-2036]
	ldr	r0, [r4, #-2056]
	uxth	r5, r5
	str	r2, [r4, #228]
	str	r0, [r4, #224]
	ldr	r3, [r3, r5, asl #2]
	cmp	r3, #0
	str	r3, [r4, #220]
	beq	.L2739
	mov	r1, #1
	add	r0, r4, #216
	mov	r2, r1
	bl	FlashReadPages
	b	.L2740
.L2739:
	ldr	r2, .L2742+4
	movw	r3, #2918
	mov	r1, #255
	ldrh	r2, [r2, r3]
	bl	ftl_memset
.L2740:
	mov	r0, r6
	mov	r1, r5
	ldr	r2, [r4, #224]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L2743:
	.align	2
.L2742:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.align	2
	.global	FlashReadFacBbtData
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2759
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r8, r2
	ldrh	r2, [r3, #12]
	.pad #40
	sub	sp, sp, #40
	ldrh	r3, [r3, #14]
	mov	r5, r0
	ldr	r9, .L2759+4
	mov	r7, r1
	smulbb	r3, r2, r3
	ldr	r2, [r9, #152]
	str	r2, [sp, #12]
	ldr	r2, [r9, #184]
	uxth	r3, r3
	sub	r6, r3, #1
	sub	r4, r3, #16
	mul	r10, r3, r1
	uxth	r6, r6
	str	r2, [sp, #16]
.L2745:
	cmp	r6, r4
	ble	.L2758
	mov	r1, #1
	add	r3, r6, r10
	add	r0, sp, #4
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L2746
	ldr	r3, [r9, #184]
	ldrh	r2, [r3]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L2746
	cmp	r5, #0
	moveq	r0, r5
	beq	.L2747
	cmp	r7, #0
	ldreq	ip, .L2759+4
	moveq	lr, #1
	beq	.L2748
.L2750:
	ldr	r1, [r9, #152]
	mov	r2, r8
	mov	r0, r5
	bl	ftl_memcpy
	mov	r2, #4
	ldr	r0, .L2759+8
	mov	r1, r5
	mov	r3, r2
	bl	rknand_print_hex
	mov	r0, #0
	b	.L2747
.L2748:
	ldr	r2, [r9, #172]
	uxth	r3, r7
	add	r7, r7, #1
	cmp	r3, r2
	bcs	.L2750
	ldr	r1, [ip, #152]
	mov	r0, r3, lsr #5
	and	r4, r3, #31
	ldr	r2, [r1, r0, asl #2]
	orr	r4, r2, lr, asl r4
	str	r4, [r1, r0, asl #2]
	b	.L2748
.L2746:
	sub	r6, r6, #1
	uxth	r6, r6
	b	.L2745
.L2758:
	mvn	r0, #0
.L2747:
	add	sp, sp, #40
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2760:
	.align	2
.L2759:
	.word	.LANCHOR0+1860
	.word	.LANCHOR2
	.word	.LC135
	.fnend
	.size	FlashReadFacBbtData, .-FlashReadFacBbtData
	.align	2
	.global	FlashGetBadBlockList
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2772
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r3, [r3, #1784]
	ldr	r6, .L2772+4
	ldrb	r4, [r3, #13]	@ zero_extendqisi2
	ldrh	r3, [r3, #14]
	ldr	r0, [r6, #180]
	smulbb	r4, r4, r3
	uxth	r4, r4
	add	r2, r4, #7
	mov	r2, r2, asr #3
	bl	FlashReadFacBbtData
	cmn	r0, #1
	bne	.L2762
.L2766:
	mov	r3, #0
	b	.L2763
.L2762:
	mov	lr, r4, lsr #4
	mov	ip, #0
	sub	r4, r4, #1
	mov	r3, ip
	mov	r7, #1
.L2764:
	uxth	r0, ip
	cmp	r0, r4
	bge	.L2763
	ldr	r8, [r6, #180]
	mov	r1, r0, lsr #5
	and	r2, r0, #31
	add	ip, ip, #1
	ldr	r1, [r8, r1, asl #2]
	ands	r2, r1, r7, asl r2
	addne	r2, r3, #1
	movne	r3, r3, asl #1
	strneh	r0, [r5, r3]	@ movhi
	uxthne	r3, r2
	cmp	r3, lr
	bcc	.L2764
	b	.L2766
.L2763:
	mov	r3, r3, asl #1
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r5, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2773:
	.align	2
.L2772:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	2
	.global	FtlMakeBbt
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	ldr	r4, .L2797
	ldr	r5, [r4, #-3100]
	cmp	r5, #0
	bne	.L2775
	bl	FtlBbtMemInit
	ldr	r7, .L2797+4
	bl	FtlLoadFactoryBbt
	sub	r8, r7, #18
	sub	r9, r7, #90
.L2776:
	ldr	r3, .L2797+8
	ldr	r6, .L2797+12
	ldrh	r3, [r3]
	cmp	r5, r3
	bcs	.L2795
	ldrh	r3, [r8, #2]!
	movw	r2, #65535
	ldr	r0, [r4, #-2060]
	ldr	r10, [r4, #-2036]
	cmp	r3, r2
	ldr	r6, .L2797
	str	r0, [r4, #224]
	str	r10, [r4, #228]
	beq	.L2777
	ldrh	ip, [r9]
	mov	r1, #1
	mov	r2, r1
	add	r0, r6, #216
	mla	fp, ip, r5, r3
	mov	r3, fp, asl #10
	str	r3, [r6, #220]
	bl	FlashReadPages
	ldrh	r2, [r9]
	ldr	r0, [r7]
	add	r2, r2, #7
	ldr	r1, [r6, #224]
	mov	r2, r2, asr #3
	bl	ftl_memcpy
	b	.L2778
.L2777:
	mov	r1, r5
	bl	FlashGetBadBlockList
	ldr	r0, [r6, #224]
	ldr	r1, [r7]
	bl	FtlBbt2Bitmap
	ldrh	r6, [r9]
.L2780:
	sub	r6, r6, #1
	uxth	r6, r6
.L2779:
	ldrh	r0, [r9]
	smlabb	r0, r0, r5, r6
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2780
	ldr	r0, [r4, #-2036]
	mov	r1, #16
	strh	r6, [r8]	@ movhi
	bl	__memzero
	ldr	r3, .L2797+16
	strh	r3, [r10]	@ movhi
	mov	r3, #0
	str	r3, [r10, #4]
	ldrh	r3, [r8]
	ldrh	ip, [r9]
	strh	r3, [r10, #2]	@ movhi
	ldrh	r3, [r8]
	ldr	r1, [r7]
	ldr	r0, [r4, #224]
	mla	fp, ip, r5, r3
	mov	r3, fp, asl #10
	str	r3, [r4, #220]
	ldr	r3, .L2797+20
	ldrh	r2, [r3]
	mov	r2, r2, asl #2
	bl	ftl_memcpy
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L2797+24
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L2797+24
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #216]
	cmn	r3, #1
	bne	.L2778
	uxth	r0, fp
	bl	FtlBbmMapBadBlock
	b	.L2779
.L2778:
	uxth	r0, fp
	add	r5, r5, #1
	bl	FtlBbmMapBadBlock
	add	r7, r7, #4
	b	.L2776
.L2795:
	ldr	r7, .L2797+28
	mov	r5, #0
.L2783:
	ldrh	r3, [r7]
	uxth	r0, r5
	add	r5, r5, #1
	cmp	r3, r0
	bls	.L2796
	bl	FtlBbmMapBadBlock
	b	.L2783
.L2796:
	ldr	r7, .L2797+32
	movw	r9, #65535
	ldrh	r5, [r7, #12]
	mov	r8, r7
	sub	r5, r5, #1
	uxth	r5, r5
.L2785:
	ldrh	r3, [r7, #12]
	sub	r3, r3, #48
	cmp	r5, r3
	ble	.L2789
	mov	r0, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2786
	mov	r0, r5
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L2787
	mov	r0, r5
	bl	FtlBbmMapBadBlock
	b	.L2786
.L2787:
	ldrh	r3, [r8]
	cmp	r3, r9
	streqh	r5, [r8]	@ movhi
.L2788:
	ldrne	r3, .L2797+32
	strneh	r5, [r3, #4]	@ movhi
	bne	.L2789
.L2786:
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L2785
.L2789:
	movw	r5, #2968
	ldr	r0, [r4, #-3096]
	ldrh	r3, [r6, r5]
	mov	r8, #0
	str	r8, [r6, #2976]
	mov	r2, #2
	mov	r1, #1
	strh	r8, [r7, #2]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r0, #4]
	ldrh	r3, [r7, #4]
	mov	r3, r3, asl #10
	str	r3, [r0, #40]
	bl	FlashEraseBlocks
	ldrh	r0, [r6, r5]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r7, #4]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r6, #2976]
	ldrh	r2, [r7, #4]
	add	r3, r3, #1
	str	r3, [r6, #2976]
	ldrh	r3, [r6, r5]
	strh	r8, [r7, #2]	@ movhi
	strh	r2, [r6, r5]	@ movhi
	strh	r3, [r7, #4]	@ movhi
	bl	FtlBbmTblFlush
.L2775:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2798:
	.align	2
.L2797:
	.word	.LANCHOR2
	.word	.LANCHOR0+2996
	.word	.LANCHOR0+2862
	.word	.LANCHOR0
	.word	-3872
	.word	.LANCHOR2-1972
	.word	.LANCHOR2+216
	.word	.LANCHOR0+2922
	.word	.LANCHOR0+2968
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #2916
	ldr	r6, .L2816
	.pad #20
	sub	sp, sp, #20
	ldrh	fp, [r6, r3]
	ldr	r3, [r6, #2964]
	str	r6, [sp, #4]
	cmp	r0, r3
	bcs	.L2800
	ldr	ip, .L2816+4
	add	fp, fp, #7
	str	r2, [sp, #12]
	mov	r6, r0, lsr fp
	ldr	r2, .L2816
	movw	r3, #2946
	mov	r10, r1
	str	r0, [sp, #8]
	uxth	r6, r6
	ldr	r8, [ip, #-2864]
	ldrh	r2, [r2, r3]
	mov	r4, #0
	mov	r9, ip
	mov	r1, #12
	b	.L2801
.L2800:
	cmp	r2, #0
	mvn	r0, #0
	streq	r0, [r1]
	b	.L2802
.L2806:
	add	r4, r4, #1
	mla	r0, r1, r4, r8
	ldrh	r0, [r0, #-12]
	cmp	r0, r6
	beq	.L2803
.L2801:
	uxth	r7, r4
	cmp	r7, r2
	bcc	.L2806
	bl	select_l2p_ram_region
	mov	r5, #12
	movw	r3, #65535
	mul	r5, r5, r0
	mov	r7, r0
	add	r2, r8, r5
	ldrh	r1, [r8, r5]
	cmp	r1, r3
	bne	.L2815
.L2807:
	ldr	r3, [r9, #-1988]
	ldr	r4, .L2816+4
	ldr	r8, [r3, r6, asl #2]
	cmp	r8, #0
	bne	.L2808
	ldr	r2, [r4, #-2864]
	mov	r1, #255
	ldr	r3, [sp, #4]
	add	r2, r2, r5
	ldr	r0, [r2, #8]
	movw	r2, #2918
	ldrh	r2, [r3, r2]
	bl	ftl_memset
	ldr	r2, [r4, #-2864]
	strh	r6, [r2, r5]	@ movhi
	ldr	r2, [r4, #-2864]
	add	r5, r2, r5
	str	r8, [r5, #4]
.L2803:
	ldr	r2, [sp, #8]
	mvn	r3, #0
	mov	r8, #12
	bic	fp, r2, r3, asl fp
	ldr	r3, [sp, #12]
	cmp	r3, #0
	uxth	fp, fp
	bne	.L2804
	ldr	r3, [r9, #-2864]
	mla	r8, r8, r7, r3
	ldr	r3, [r8, #8]
	ldr	r3, [r3, fp, asl #2]
	str	r3, [r10]
	b	.L2805
.L2804:
	mul	r8, r8, r7
	ldr	r3, [r9, #-2864]
	ldr	r2, [r10]
	add	r3, r3, r8
	ldr	r3, [r3, #8]
	str	r2, [r3, fp, asl #2]
	ldr	r3, [r9, #-2864]
	add	r8, r3, r8
	ldr	r3, [r8, #4]
	orr	r3, r3, #-2147483648
	str	r3, [r8, #4]
	ldr	r3, .L2816+8
	strh	r6, [r3, #-12]	@ movhi
.L2805:
	ldr	r2, [r9, #-2864]
	mov	r3, #12
	mov	r0, #0
	mla	r7, r3, r7, r2
	ldr	r3, [r7, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r7, #4]
	b	.L2802
.L2815:
	ldr	r3, [r2, #4]
	cmp	r3, #0
	bge	.L2807
	bl	flush_l2p_region
	b	.L2807
.L2808:
	ldr	r2, [r4, #-2864]
	mov	r1, #1
	add	r0, r4, #216
	str	r8, [r4, #220]
	add	r2, r2, r5
	ldr	r2, [r2, #8]
	str	r2, [r4, #224]
	ldr	r2, [r4, #-2036]
	str	r2, [r4, #228]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [r4, #228]
	ldrh	r2, [r2, #8]
	cmp	r2, r6
	beq	.L2809
	mov	r1, r6
	mov	r2, r8
	ldr	r0, .L2816+12
	bl	printk
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2816+16
	ldr	r1, [r4, #228]
	bl	rknand_print_hex
	ldr	r3, .L2816+20
	ldr	r0, .L2816+24
	mov	r2, #4
	ldr	r1, [r4, #-1988]
	ldrh	r3, [r3]
	bl	rknand_print_hex
	mov	r3, #1
	str	r3, [r4, #-3100]
	b	.L2810
.L2809:
	ldr	r2, [r4, #216]
	cmp	r2, #256
	bne	.L2810
	mov	r1, r6
	mov	r2, r8
	ldr	r0, .L2816+28
	bl	printk
	ldr	r3, [r4, #-2864]
	sub	r0, r4, #1968
	mov	r1, r6
	add	r3, r3, r5
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
.L2810:
	ldr	r3, [r9, #-2864]
	mov	r1, #0
	add	r2, r3, r5
	str	r1, [r2, #4]
	strh	r6, [r3, r5]	@ movhi
	b	.L2803
.L2802:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2817:
	.align	2
.L2816:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2-2848
	.word	.LC136
	.word	.LC100
	.word	.LANCHOR0+2944
	.word	.LC137
	.word	.LC138
	.fnend
	.size	log2phys, .-log2phys
	.align	2
	.global	FtlWriteDumpData
	.type	FtlWriteDumpData, %function
FtlWriteDumpData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r4, .L2837
	ldr	r3, [r4, #-3100]
	cmp	r3, #0
	bne	.L2818
	sub	r7, r4, #3008
	ldrh	r5, [r7, #4]
	cmp	r5, #0
	beq	.L2820
	ldrb	r3, [r4, #-3000]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2820
	ldr	r8, .L2837+4
	movw	r3, #2908
	ldrb	r2, [r4, #-3001]	@ zero_extendqisi2
	ldrh	r3, [r8, r3]
	mul	r3, r3, r2
	cmp	r5, r3
	beq	.L2820
	ldrb	r10, [r4, #-2998]	@ zero_extendqisi2
	movw	r3, #2840
	ldr	r6, [r8, #2964]
	cmp	r10, #0
	ldrh	r9, [r8, r3]
	bne	.L2818
	sub	r6, r6, #1
	mov	r1, sp
	mov	r2, r10
	mov	r0, r6
	bl	log2phys
	ldr	r5, [r4, #-2036]
	ldr	r0, [r4, #-2060]
	ldr	r3, [sp]
	str	r6, [sp, #20]
	cmn	r3, #1
	str	r0, [sp, #12]
	str	r3, [sp, #8]
	str	r5, [sp, #16]
	str	r10, [r5, #4]
	beq	.L2822
	mov	r1, #1
	mov	r2, r10
	add	r0, sp, #4
	bl	FlashReadPages
	b	.L2823
.L2822:
	movw	r3, #2918
	mov	r1, #255
	ldrh	r2, [r8, r3]
	bl	ftl_memset
.L2823:
	ldr	r3, .L2837+8
	mov	r8, #0
	ldr	fp, .L2837
	mov	r9, r9, asl #2
	mov	r10, r8
	strh	r3, [r5]	@ movhi
.L2824:
	cmp	r8, r9
	beq	.L2825
	ldrh	r3, [r7, #4]
	ldr	r0, .L2837+12
	cmp	r3, #0
	beq	.L2825
	ldr	r3, [sp, #8]
	add	r8, r8, #1
	str	r6, [r5, #8]
	str	r3, [r5, #12]
	ldrh	r3, [r0]
	strh	r3, [r5, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [fp, #-2812]
	mov	r2, #0
	mov	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r5, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, r10
	str	r3, [fp, #-2812]
	mov	r3, r2
	bl	FlashProgPages
	ldrh	r0, [r7]
	bl	decrement_vpc_count
	b	.L2824
.L2825:
	mov	r3, #1
	b	.L2836
.L2820:
	mov	r3, #0
.L2836:
	strb	r3, [r4, #-2998]
.L2818:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2838:
	.align	2
.L2837:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-3947
	.word	.LANCHOR2-3008
	.fnend
	.size	FtlWriteDumpData, .-FtlWriteDumpData
	.align	2
	.global	l2p_flush
	.type	l2p_flush, %function
l2p_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	bl	FtlWriteDumpData
	mov	r4, #0
	ldr	r5, .L2845
	mov	r7, #12
	ldr	r6, .L2845+4
.L2840:
	ldrh	r3, [r5]
	uxth	r0, r4
	cmp	r3, r0
	bls	.L2844
	ldr	r3, [r6, #-2864]
	mla	r3, r7, r0, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L2841
	bl	flush_l2p_region
.L2841:
	add	r4, r4, #1
	b	.L2840
.L2844:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2846:
	.align	2
.L2845:
	.word	.LANCHOR0+2946
	.word	.LANCHOR2
	.fnend
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	allocate_new_data_superblock
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r5, .L2874
	ldrh	r6, [r0]
	ldr	r3, [r5, #-3100]
	cmp	r3, #0
	bne	.L2848
	movw	r3, #65535
	cmp	r6, r3
	mov	r4, r0
	beq	.L2849
	ldr	r2, [r5, #-3028]
	mov	r3, r6, asl #1
	mov	r0, r6
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2850
	bl	INSERT_DATA_LIST
	b	.L2849
.L2850:
	bl	INSERT_FREE_LIST
.L2849:
	ldr	r2, .L2874
	mov	r3, #0
	strb	r3, [r4, #8]
	sub	r3, r2, #2960
	cmp	r4, r3
	beq	.L2851
	ldr	r3, .L2874+4
	movw	r1, #2860
	ldrh	r1, [r3, r1]
	cmp	r1, #1
	beq	.L2851
	ldrb	r0, [r3, #1888]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L2852
.L2851:
	mov	r3, #1
	strb	r3, [r4, #8]
	b	.L2853
.L2852:
	sub	r0, r2, #3008
	cmp	r4, r0
	bne	.L2853
	cmp	r1, #3
	beq	.L2855
	ldr	r2, [r2, #-2720]
	cmp	r2, #1
	bne	.L2856
.L2855:
	mov	r2, #1
	strb	r2, [r5, #-3000]
.L2856:
	ldr	r3, [r3, #2832]
	cmp	r3, #0
	beq	.L2853
	ldr	r3, [r5, #-2792]
	cmp	r3, #39
	ldrls	r3, .L2874
	movls	r2, #1
	strlsb	r2, [r3, #-3000]
.L2853:
	ldr	r2, .L2874
	movw	r1, #65535
	sub	r3, r2, #1904
	ldrh	r0, [r3, #-12]
	mov	r5, r3
	cmp	r0, r1
	beq	.L2858
	cmp	r6, r0
	bne	.L2859
	ldr	r2, [r2, #-3028]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2860
.L2859:
	bl	update_vpc_list
.L2860:
	mvn	r3, #0
	strh	r3, [r5, #-12]	@ movhi
.L2858:
	mov	r0, r4
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2848:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2875:
	.align	2
.L2874:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlCheckVpc
	.type	FtlCheckVpc, %function
FtlCheckVpc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	mov	r4, #0
	ldr	r1, .L2897
	ldr	r0, .L2897+4
	bl	printk
	ldr	r0, .L2897+8
	mov	r1, #8192
	ldr	r6, .L2897+12
	bl	__memzero
	ldr	r5, .L2897+8
.L2877:
	ldr	r3, [r6, #2964]
	cmp	r4, r3
	bcs	.L2895
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L2878
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	mov	r0, r0, asl #1
	ldrh	r3, [r5, r0]
	add	r3, r3, #1
	strh	r3, [r5, r0]	@ movhi
.L2878:
	add	r4, r4, #1
	b	.L2877
.L2895:
	ldr	r5, .L2897+16
	mov	r4, #0
	ldr	r7, .L2897+20
	add	r8, r5, #304
.L2880:
	ldrh	r3, [r7]
	uxth	r1, r4
	ldr	r6, .L2897+16
	cmp	r3, r1
	bls	.L2896
	ldr	r2, [r5, #-3028]
	mov	r3, r1, asl #1
	ldrh	r2, [r2, r3]
	ldrh	r3, [r8, r3]
	cmp	r2, r3
	beq	.L2881
	ldr	r0, .L2897+24
	bl	printk
.L2881:
	add	r4, r4, #1
	b	.L2880
.L2896:
	ldr	r3, [r6, #-3016]
	cmp	r3, #0
	beq	.L2876
	sub	r2, r6, #3008
	add	r8, r6, #304
	mov	r5, #0
	mov	r9, #6
	ldrh	r7, [r2, #-4]
	ldr	r2, [r6, #-3036]
	rsb	r3, r2, r3
	ldr	r2, .L2897+28
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r4, r3
.L2884:
	uxth	r3, r5
	cmp	r3, r7
	bcs	.L2876
	ldr	r2, [r6, #-3028]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	beq	.L2885
	ldr	r0, .L2897+32
	mov	r1, r4
	ldrh	r3, [r8, r3]
	bl	printk
.L2885:
	mul	r4, r9, r4
	ldr	r3, [r6, #-3036]
	add	r5, r5, #1
	ldrh	r4, [r3, r4]
	movw	r3, #65535
	cmp	r4, r3
	bne	.L2884
.L2876:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2898:
	.align	2
.L2897:
	.word	.LANCHOR3+72
	.word	.LC107
	.word	.LANCHOR2+304
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2848
	.word	.LC139
	.word	-1431655765
	.word	.LC140
	.fnend
	.size	FtlCheckVpc, .-FtlCheckVpc
	.align	2
	.global	Ftlscanalldata
	.type	Ftlscanalldata, %function
Ftlscanalldata:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	mov	r1, #0
	.pad #36
	sub	sp, sp, #36
	ldr	r0, .L2909
	bl	printk
	ldr	r7, .L2909+4
	ldr	r5, .L2909+8
	mov	r4, #0
.L2900:
	ldr	r3, [r7, #2964]
	cmp	r4, r3
	bcs	.L2908
	mov	r0, r4
	add	r1, sp, #28
	mov	r2, #0
	bl	log2phys
	ubfx	r3, r4, #0, #11
	cmp	r3, #0
	bne	.L2901
	ldr	r0, .L2909+12
	mov	r1, r4
	ldr	r2, [sp, #28]
	bl	printk
.L2901:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L2903
	str	r3, [r5, #220]
	mov	r2, #0
	ldr	r3, [r5, #-2060]
	mov	r1, #1
	ldr	r6, [r5, #-2036]
	ldr	r0, .L2909+16
	str	r3, [r5, #224]
	str	r2, [r5, #216]
	str	r4, [r5, #232]
	str	r6, [r5, #228]
	bl	FlashReadPages
	ldr	r3, [r5, #216]
	ldr	r2, .L2909+8
	cmn	r3, #1
	cmpne	r3, #256
	beq	.L2904
	ldr	r3, [r6, #8]
	cmp	r3, r4
	beq	.L2903
.L2904:
	ldr	r3, [r2, #228]
	ldr	r1, [r2, #224]
	ldr	r0, [r3, #4]
	str	r0, [sp]
	ldr	r0, [r3, #8]
	str	r0, [sp, #4]
	ldr	r0, [r3, #12]
	str	r0, [sp, #8]
	ldr	r0, [r1]
	str	r0, [sp, #12]
	ldr	r1, [r1, #4]
	ldr	r0, .L2909+20
	str	r1, [sp, #16]
	mov	r1, r4
	ldr	r2, [r2, #220]
	ldr	r3, [r3]
	bl	printk
.L2903:
	add	r4, r4, #1
	b	.L2900
.L2908:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2910:
	.align	2
.L2909:
	.word	.LC141
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC142
	.word	.LANCHOR2+216
	.word	.LC143
	.fnend
	.size	Ftlscanalldata, .-Ftlscanalldata
	.align	2
	.global	FtlReUsePrevPpa
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	mov	r5, r0
	ubfx	r0, r1, #10, #16
	str	r1, [sp, #4]
	bl	P2V_block_in_plane
	ldr	r6, .L2921
	ldr	r4, [r6, #-3028]
	mov	r7, r0, asl #1
	ldrh	r3, [r4, r7]
	cmp	r3, #0
	addne	r3, r3, #1
	strneh	r3, [r4, r7]	@ movhi
	bne	.L2913
	ldr	r4, [r6, #-3016]
	cmp	r4, #0
	beq	.L2913
	ldr	r1, [r6, #-3036]
	sub	r2, r6, #3008
	ldr	lr, .L2921+4
	movw	r9, #65535
	rsb	r4, r1, r4
	ldrh	ip, [r2, #-4]
	mov	r8, r2
	mov	r4, r4, asr #1
	mul	r4, lr, r4
	mov	lr, #6
	uxth	r4, r4
.L2914:
	uxth	r2, r3
	cmp	r2, ip
	bcs	.L2913
	cmp	r4, r0
	bne	.L2915
	mov	r1, r4
	ldr	r0, .L2921+8
	bl	List_remove_node
	ldrh	r3, [r8, #-4]
	mov	r0, r4
	sub	r3, r3, #1
	strh	r3, [r8, #-4]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r2, [r6, #-3028]
	ldrh	r3, [r2, r7]
	add	r3, r3, #1
	strh	r3, [r2, r7]	@ movhi
	b	.L2913
.L2915:
	mul	r4, lr, r4
	add	r3, r3, #1
	ldrh	r4, [r1, r4]
	cmp	r4, r9
	bne	.L2914
.L2913:
	mov	r0, r5
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2922:
	.align	2
.L2921:
	.word	.LANCHOR2
	.word	-1431655765
	.word	.LANCHOR2-3016
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r4, #65535
	ldrh	r3, [r0]
	.pad #68
	sub	sp, sp, #68
	mov	lr, r0
	cmp	r3, r4
	beq	.L3060
	ldrh	r3, [r0, #2]
	mov	ip, #0
	ldr	r5, .L3077
	str	r3, [sp, #8]
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	ldr	r2, [sp, #8]
	str	r3, [sp, #16]
	movw	r3, #2908
	ldrh	r3, [r5, r3]
	cmp	r3, r2
	streqh	ip, [r0, #4]	@ movhi
	streqb	ip, [r0, #6]
	ldrneh	r0, [r0, #16]
	beq	.L3060
.L2927:
	cmp	r0, r4
	add	ip, ip, #1
	uxtheq	r3, ip
	addeq	r3, lr, r3, asl #1
	ldreqh	r0, [r3, #16]
	beq	.L2927
.L3072:
	ldrb	r1, [lr, #8]	@ zero_extendqisi2
	mov	r9, lr
	cmp	r1, #1
	bne	.L2929
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r4, r0
	beq	.L2930
	ldrb	r3, [r5, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3063
	ldr	r3, .L3077
	add	r3, r3, r0, asl #1
	ldrh	r6, [r3, #76]
	b	.L3000
.L2929:
	mov	r1, #0
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r4, r0
	beq	.L2930
.L3063:
	mov	r6, r4
.L3000:
	movw	r3, #2840
	mov	r2, #0
	ldrh	r3, [r5, r3]
	movw	r8, #65535
	mov	r10, r2
	str	r3, [sp, #4]
	ldr	r3, .L3077+4
	ldr	ip, [r3, #-2072]
	ldr	lr, [r3, #-2176]
	movw	r3, #2920
	ldrh	r7, [r5, r3]
	add	r3, r9, #14
	mov	r5, r2
	str	r3, [sp, #20]
	mov	r0, r3
	b	.L2932
.L2930:
	mov	r3, #0
	strh	r3, [r9, #2]	@ movhi
	strb	r3, [r9, #6]
	b	.L3060
.L2934:
	ldrh	r3, [r0, #2]!
	cmp	r3, r8
	beq	.L2933
	mov	r1, #36
	orr	r3, r6, r3, asl #10
	mla	r1, r1, r5, ip
	stmib	r1, {r3, r10}
	mul	r3, r7, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	bic	r3, r3, #3
	add	r3, lr, r3
	str	r3, [r1, #12]
.L2933:
	add	r2, r2, #1
.L2932:
	ldr	r1, [sp, #4]
	uxth	r3, r2
	cmp	r3, r1
	bcc	.L2934
	ldrb	r3, [r9, #8]	@ zero_extendqisi2
	cmp	r3, #1
	movne	r3, #0
	bne	.L3064
	ldr	r3, .L3077
	ldrb	lr, [r3, #1888]	@ zero_extendqisi2
	adds	r3, lr, #0
	movne	r3, #1
.L3064:
	ldr	r7, .L3077+4
	mov	r1, r5
	str	r3, [sp, #24]
	mov	fp, #0
	ldr	r2, [sp, #24]
	movw	r10, #65535
	ldr	r0, [r7, #-2072]
	bl	FlashReadPages
	ldr	r3, [r7, #-2812]
	str	r7, [sp, #12]
	str	r3, [sp, #28]
.L2936:
	uxth	r8, fp
	cmp	r8, r5
	bcs	.L2943
	mov	r1, #36
	ldr	r0, [r7, #-2072]
	mul	r1, r1, fp
	add	ip, r0, r1
	ldr	r1, [r0, r1]
	cmp	r1, #0
	bne	.L2937
	ldr	ip, [ip, #12]
	ldr	r3, [ip, #4]
	cmn	r3, #1
	beq	.L2938
	mov	r0, r3
	ldr	r1, [r7, #-2812]
	str	ip, [sp, #32]
	str	r3, [sp, #4]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #4]
	cmp	r0, #0
	ldr	ip, [sp, #32]
	addne	r3, r3, #1
	strne	r3, [r7, #-2812]
.L2938:
	ldr	r1, [ip]
	cmn	r1, #1
	bne	.L2939
.L2943:
	cmp	r8, r5
	ldr	r5, .L3077+4
	bne	.L3061
	add	fp, r4, #1
	uxth	r3, fp
	str	r3, [sp, #4]
	ldr	r3, [r5, #-2072]
	ldr	r0, [r3, #4]
	b	.L3065
.L2937:
	ldr	r1, [ip, #4]
	uxth	r10, r6
	ldr	r0, .L3077+8
	bl	printk
	ldrh	r1, [r9]
	ldr	r3, .L3077+12
	strh	r1, [r3]	@ movhi
.L2939:
	add	fp, fp, #1
	b	.L2936
.L3061:
	uxth	r3, r4
	str	r3, [sp, #4]
	ldr	r3, [sp, #12]
	mov	r2, #36
	ldr	r3, [r3, #-2072]
	mla	r8, r2, r8, r3
	ldr	r0, [r8, #4]
.L3065:
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldrb	r2, [r9, #8]	@ zero_extendqisi2
	cmp	r2, #1
	str	r0, [sp, #12]
	ldr	r0, .L3077
	bne	.L2945
	ldrb	r3, [r0, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, [sp, #4]
	addeq	fp, r0, r3, asl #1
	ldreqh	r3, [fp, #76]
	streq	r3, [sp, #4]
.L2945:
	movw	r3, #2908
	ldr	r1, [sp, #4]
	ldrh	r3, [r0, r3]
	ldr	r0, [sp, #12]
	cmp	r3, r1
	ldr	r1, [sp, #8]
	ldreqh	r3, [sp, #4]
	streqh	r3, [r9, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r9, #6]
	streqh	r3, [r9, #4]	@ movhi
	ldrh	r3, [sp, #16]
	str	r3, [sp, #32]
	ldr	ip, [sp, #32]
	ldr	r3, [sp, #4]
	cmp	r3, r1
	cmpeq	r0, ip
	moveq	r0, r9
	moveq	r1, r3
	beq	.L3070
	ldr	r3, [sp, #28]
	sub	fp, r3, #1
	clz	r3, r2
	movw	r2, #65535
	mov	r3, r3, lsr #5
	cmp	r10, r2
	orrne	r3, r3, #1
	cmp	r3, #0
	beq	.L2949
	ldr	r3, [r5, #-1904]
	uxth	r10, r6
	ldr	r7, .L3077+4
	mvn	r8, #0
	cmn	r3, #1
	mov	r6, r8
	ldreq	r3, .L3077+4
	streq	fp, [r3, #-1904]
	ldr	r3, [r5, #-1904]
	str	r3, [sp, #16]
	ldr	r3, [sp, #8]
	add	r3, r3, #7
	cmp	r10, r3
	subgt	r4, r10, #7
	ldrle	r4, [sp, #8]
	uxthgt	r4, r4
.L2952:
	cmp	r4, r10
	bhi	.L2965
	ldr	r3, .L3077+16
	mov	r0, #36
	ldr	lr, [r7, #-2072]
	ldr	r1, [sp, #20]
	ldrh	r3, [r3]
	str	r3, [sp, #28]
	mov	r3, #0
	mov	r5, r3
.L2966:
	ldr	ip, [sp, #28]
	uxth	r2, r3
	cmp	r2, ip
	bcs	.L3073
	ldrh	r2, [r1, #2]!
	movw	ip, #65535
	add	r3, r3, #1
	cmp	r2, ip
	orrne	r2, r4, r2, asl #10
	mlane	ip, r0, r5, lr
	addne	r5, r5, #1
	uxthne	r5, r5
	strne	r2, [ip, #4]
	b	.L2966
.L3073:
	mov	r1, r5
	ldr	r2, [sp, #24]
	ldr	r0, [r7, #-2072]
	bl	FlashReadPages
	ldr	r2, .L3077
	ldr	r3, [r7, #-2072]
	mov	r1, #36
	add	r2, r2, #588
	ldrb	ip, [r2, #1300]	@ zero_extendqisi2
	add	r2, r2, r4, asl #1
	mla	r5, r1, r5, r3
	movw	r1, #65535
.L2955:
	cmp	r3, r5
	beq	.L3074
	ldr	r0, [r3]
	cmp	r0, #0
	bne	.L2956
	ldr	r0, [r3, #12]
	ldrh	lr, [r0]
	cmp	lr, r1
	beq	.L2957
	ldr	r0, [r0, #4]
	cmn	r0, #1
	beq	.L2957
	cmn	r8, #1
	ldr	r6, [r7, #-1904]
	str	r0, [r7, #-1904]
	bne	.L2957
	ldrh	r0, [r2]
	cmp	r0, r1
	bne	.L2958
	cmp	ip, #0
	beq	.L2957
.L2958:
	cmp	r6, fp
	mvneq	r8, #0
	movne	r8, r6
	b	.L2957
.L2956:
	ldr	r3, .L3077+12
	ldrh	r2, [r9]
	strh	r2, [r3]	@ movhi
	ldrb	r3, [r9, #8]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2949
	ldr	r3, .L3077+20
	mov	r4, r4, asl #1
	ldrh	r2, [r3, r4]
	movw	r3, #65535
	cmp	r2, r3
	ldr	r3, .L3077+4
	bne	.L2960
	cmn	r8, #1
	strne	r8, [r3, #-1904]
	bne	.L2949
	ldr	r2, [sp, #16]
	cmp	r2, fp
	bne	.L3066
	ldr	r2, [r3, #-1904]
	b	.L3071
.L2960:
	cmp	r6, fp
	beq	.L2963
	cmn	r6, #1
	strne	r6, [r3, #-1904]
	b	.L2949
.L2963:
	ldr	r2, [r3, #-1904]
	cmp	r2, fp
	beq	.L2949
.L3071:
	sub	r2, r2, #1
	b	.L3066
.L2957:
	add	r3, r3, #36
	b	.L2955
.L3074:
	add	r4, r4, #1
	uxth	r4, r4
	b	.L2952
.L2965:
	ldr	r3, .L3077+4
	mvn	r2, #0
.L3066:
	str	r2, [r3, #-1904]
.L2949:
	ldr	r2, .L3077+24
	movw	r3, #314
	ldr	r10, [sp, #8]
	mov	r1, #1
	ldr	r4, .L3077+4
	strh	r1, [r2, r3]	@ movhi
.L2967:
	ldr	r3, .L3077+16
	movw	r1, #65535
	ldr	r8, [r4, #-2072]
	mov	r0, #36
	ldr	r6, [sp, #20]
	mov	r5, #0
	ldrh	r7, [r3]
	ldr	r3, .L3077
	str	r5, [sp, #16]
	ldrb	lr, [r3, #1888]	@ zero_extendqisi2
.L2968:
	uxth	r3, r5
	cmp	r3, r7
	bcs	.L3075
	ldrh	r3, [r6, #2]!
	cmp	r3, r1
	beq	.L2969
	ldr	r2, [sp, #16]
	orr	r3, r10, r3, asl #10
	mla	r2, r0, r2, r8
	str	r3, [r2, #4]
	ldrb	ip, [r9, #8]	@ zero_extendqisi2
	cmp	ip, #1
	bne	.L2970
	cmp	lr, #0
	orrne	r3, r3, #-2147483648
	strne	r3, [r2, #4]
.L2970:
	ldr	r3, [sp, #16]
	add	r3, r3, #1
	uxth	r3, r3
	str	r3, [sp, #16]
.L2969:
	add	r5, r5, #1
	b	.L2968
.L3075:
	ldr	r0, [r4, #-2072]
	ldr	r1, [sp, #16]
	ldr	r2, [sp, #24]
	bl	FlashReadPages
	mov	r3, #0
.L3069:
	str	r3, [sp, #28]
	ldr	r2, [sp, #16]
	ldrh	r3, [sp, #28]
	cmp	r3, r2
	bcs	.L3076
	ldr	r3, [sp, #28]
	mov	r5, #36
	ldr	r8, [r4, #-2072]
	mul	r5, r5, r3
	add	r7, r8, r5
	ldr	r6, [r7, #4]
	ubfx	r0, r6, #10, #16
	str	r6, [sp, #60]
	bl	P2V_plane
	ldr	r3, [sp, #8]
	cmp	r10, r3
	bcc	.L2973
	ldr	r3, [sp, #32]
	ldr	r2, [sp, #8]
	cmp	r0, r3
	movcs	r3, #0
	movcc	r3, #1
	cmp	r10, r2
	movne	r3, #0
	cmp	r3, #0
	bne	.L2973
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #4]
	cmp	r0, r3
	cmpeq	r10, r2
	beq	.L2974
	ldr	r3, [r8, r5]
	cmn	r3, #1
	beq	.L2975
	ldr	r7, [r7, #12]
	movw	r3, #61589
	ldrh	r2, [r7]
	cmp	r2, r3
	ldrneh	r0, [r9]
	bne	.L3067
	ldr	fp, [r7, #4]
	cmn	fp, #1
	beq	.L2977
	ldr	r1, [r4, #-2812]
	mov	r0, fp
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	addne	r3, fp, #1
	strne	r3, [r4, #-2812]
.L2977:
	ldr	r6, [r7, #8]
	add	r1, sp, #56
	ldr	r3, [r7, #12]
	mov	r2, #0
	mov	r0, r6
	str	r3, [sp, #52]
	bl	log2phys
	ldr	r1, [r4, #-1904]
	cmn	r1, #1
	beq	.L2978
	mov	r0, fp
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L2978
	ldr	r3, [sp, #52]
	cmn	r3, #1
	beq	.L2979
	ldr	r0, [r4, #-2072]
	mov	r2, #0
	mov	r1, #1
	add	r0, r0, r5
	str	r3, [r0, #4]
	ldr	r8, [r0, #12]
	bl	FlashReadPages
	ldr	r2, [r4, #-2072]
	ldr	r3, [r8, #4]
	add	ip, r2, r5
	str	r3, [sp, #36]
	ldr	r3, [r2, r5]
	cmn	r3, #1
	bne	.L2980
	b	.L2981
.L2979:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	bne	.L2973
	mov	r0, r6
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
	b	.L2973
.L2980:
	ldr	r7, [r8, #8]
	cmp	r7, r6
	bne	.L2981
	ldr	r0, [r4, #-1904]
	ldr	r1, [sp, #36]
	str	r2, [sp, #44]
	str	ip, [sp, #40]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	ldr	ip, [sp, #40]
	ldr	r2, [sp, #44]
	beq	.L2981
	ldr	r3, [sp, #56]
	ldr	r1, [sp, #60]
	cmp	r3, r1
	beq	.L2986
	ldr	r1, [sp, #52]
	cmp	r3, r1
	beq	.L2981
	cmn	r3, #1
	streq	r3, [r2, r5]
	beq	.L2985
	str	r3, [ip, #4]
	mov	r0, ip
	mov	r1, #1
	mov	r2, #0
	ldr	r8, [ip, #12]
	bl	FlashReadPages
.L2985:
	ldr	r3, [r4, #-2072]
	ldr	r3, [r3, r5]
	cmn	r3, #1
	beq	.L2986
	ldr	r5, [r8, #4]
	ldr	r0, [r4, #-1904]
	mov	r1, r5
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L2986
	ldr	r0, [sp, #36]
	mov	r1, r5
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L2981
.L2986:
	mov	r0, r7
	ldr	r1, [sp, #52]
	bl	FtlReUsePrevPpa
.L2981:
	mvn	r3, #0
	str	r3, [sp, #52]
	b	.L2988
.L2978:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	beq	.L2988
	ldr	r3, [sp, #52]
	ldr	r2, .L3077
	mov	r1, r3, lsr #10
	adds	r3, r3, #1
	ldr	r2, [r2, #2856]
	movne	r3, #1
	cmp	r1, r2
	movcc	r3, #0
	cmp	r3, #0
	bne	.L2973
	mov	r0, r6
	add	r1, sp, #60
	mov	r2, #1
	bl	log2phys
	ldr	r8, [sp, #56]
	cmn	r8, #1
	beq	.L2988
	ldr	r3, [sp, #52]
	cmp	r8, r3
	beq	.L2988
	ubfx	r0, r8, #10, #16
	ldr	r5, .L3077+28
	bl	P2V_block_in_plane
	ldrh	r3, [r5]
	add	r5, r5, #3008
	cmp	r3, r0
	beq	.L2991
	sub	r3, r5, #2960
	ldrh	r3, [r3]
	cmp	r3, r0
	beq	.L2991
	sub	r3, r5, #2912
	ldrh	r3, [r3]
	cmp	r3, r0
	bne	.L2988
.L2991:
	ldr	r0, [r5, #-2072]
	mov	r1, #1
	mov	r2, #0
	str	r8, [r0, #4]
	ldr	r7, [r0, #12]
	bl	FlashReadPages
	ldr	r3, [r5, #-2072]
	ldr	r1, [r7, #4]
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L2988
	mov	r0, fp
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L2988
	mov	r0, r6
	add	r1, sp, #56
	mov	r2, #1
	bl	log2phys
.L2988:
	ldr	r0, [sp, #52]
	cmn	r0, #1
	beq	.L2973
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r4, #-3028]
	mov	r3, r0, asl #1
	mov	r1, r0
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2992
.L3067:
	bl	decrement_vpc_count
	b	.L2973
.L2992:
	ldr	r0, .L3077+32
	bl	printk
	b	.L2973
.L2975:
	ldrh	r2, [r9]
	mov	r1, r6
	ldr	r3, .L3077+12
	ldr	r0, .L3077+36
	strh	r2, [r3]	@ movhi
	mov	r2, fp
	bl	printk
	ldr	r3, .L3077+24
	ldr	r3, [r3, #316]
	cmp	r3, #31
	bhi	.L2993
	ldr	r2, .L3077+24
	ldr	r1, [sp, #60]
	add	r2, r2, r3, asl #2
	add	r3, r3, #1
	str	r1, [r2, #320]
	ldr	r2, .L3077+24
	str	r3, [r2, #316]
.L2993:
	ldrh	r0, [r9]
	bl	decrement_vpc_count
	ldr	r3, [r4, #-1904]
	cmn	r3, #1
	beq	.L3068
	cmp	r3, fp
	bls	.L2973
.L3068:
	str	fp, [r4, #-1904]
.L2973:
	ldr	r3, [sp, #28]
	add	r3, r3, #1
	b	.L3069
.L3076:
	ldrb	r3, [r9, #8]	@ zero_extendqisi2
	add	r10, r10, #1
	cmp	r3, #1
	uxth	r10, r10
	bne	.L2996
	ldr	r3, .L3077
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2996
	ldr	r3, .L3077+40
	ldr	r2, [sp, #4]
	ldrh	r3, [r3]
	cmp	r2, r10
	cmpeq	r3, r10
	beq	.L2974
.L2996:
	ldr	r3, .L3077+44
	ldrh	r3, [r3]
	cmp	r10, r3
	bne	.L2967
	ldr	r1, .L3077
	movw	r2, #2840
	movw	r0, #65535
	mov	r3, #0
	strh	r10, [r9, #2]	@ movhi
	ldrh	r1, [r1, r2]
	strh	r3, [r9, #4]	@ movhi
.L2997:
	uxth	r2, r3
	cmp	r2, r1
	bcs	.L3060
	ldr	lr, [sp, #20]
	add	r3, r3, #1
	ldrh	ip, [lr, #2]!
	cmp	ip, r0
	str	lr, [sp, #20]
	beq	.L2997
	strb	r2, [r9, #6]
	b	.L3060
.L2974:
	ldrb	r3, [sp, #12]	@ zero_extendqisi2
	mov	r0, r9
	ldr	r1, [sp, #4]
	strb	r3, [r9, #6]
	ldrh	r3, [sp, #4]
	strh	r3, [r9, #2]	@ movhi
.L3070:
	ldr	r2, [sp, #12]
	bl	ftl_sb_update_avl_pages
.L3060:
	mov	r0, #0
	add	sp, sp, #68
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3078:
	.align	2
.L3077:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC144
	.word	.LANCHOR4+312
	.word	.LANCHOR0+2840
	.word	.LANCHOR0+588
	.word	.LANCHOR4
	.word	.LANCHOR2-3008
	.word	.LC145
	.word	.LC146
	.word	.LANCHOR0+2910
	.word	.LANCHOR0+2908
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	mov	r4, #0
	ldr	r5, .L3095
	ldr	r1, .L3095+4
	ldr	r0, .L3095+8
	bl	printk
	movw	r3, #2850
	ldrh	r2, [r5, r3]
	mov	r1, #0
	ldr	r6, .L3095+12
	mov	r2, r2, asl #1
	ldr	r0, [r6, #-2012]
	bl	ftl_memset
.L3080:
	ldr	r3, [r5, #2964]
	cmp	r4, r3
	bcs	.L3093
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L3081
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r6, #-2012]
	mov	r0, r0, asl #1
	ldrh	r3, [r2, r0]
	add	r3, r3, #1
	strh	r3, [r2, r0]	@ movhi
.L3081:
	add	r4, r4, #1
	b	.L3080
.L3093:
	ldr	r9, .L3095+16
	mov	r7, #0
	ldr	r8, .L3095+12
.L3083:
	ldrh	r3, [r9]
	uxth	r4, r7
	cmp	r3, r4
	bls	.L3094
	ldr	r3, [r6, #-3028]
	mov	r5, r4, asl #1
	movw	r1, #65535
	ldrh	r2, [r3, r5]
	ldr	r3, [r6, #-2012]
	ldrh	r3, [r3, r5]
	cmp	r2, r3
	cmpne	r2, r1
	beq	.L3085
	ldr	r0, .L3095+20
	mov	r1, r4
	bl	printk
	ldr	r3, .L3095+24
	ldrh	r2, [r3]
	cmp	r2, r4
	beq	.L3085
	ldrh	r2, [r3, #96]
	cmp	r2, r4
	beq	.L3085
	ldrh	r3, [r3, #48]
	cmp	r3, r4
	beq	.L3085
	ldr	r3, [r8, #-3028]
	ldrh	r2, [r3, r5]
	cmp	r2, #0
	ldr	r2, [r8, #-2012]
	ldrh	r2, [r2, r5]
	strh	r2, [r3, r5]	@ movhi
	beq	.L3085
	mov	r0, r4
	bl	update_vpc_list
.L3085:
	add	r7, r7, #1
	b	.L3083
.L3094:
	bl	l2p_flush
	bl	FtlVpcTblFlush
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3096:
	.align	2
.L3095:
	.word	.LANCHOR0
	.word	.LANCHOR3+84
	.word	.LC107
	.word	.LANCHOR2
	.word	.LANCHOR0+2848
	.word	.LC147
	.word	.LANCHOR2-3008
	.fnend
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L3144
	movw	r3, #3288
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #68
	sub	sp, sp, #68
	ldrh	r5, [r2, r3]
	movw	r3, #65535
	mov	r4, r0
	str	r1, [sp, #12]
	cmp	r5, r3
	beq	.L3128
	cmp	r5, #0
	bne	.L3098
	b	.L3099
.L3128:
	mov	r5, #0
.L3098:
	ldr	r2, .L3144+4
	movw	r3, #2908
	ldrh	r3, [r2, r3]
	ldr	r2, [sp, #12]
	cmp	r2, r3
	bne	.L3100
.L3099:
	bl	FtlGcPageVarInit
.L3100:
	ldr	r6, .L3144+8
	mvn	r3, #0
	str	r3, [sp, #8]
	mov	r3, #0
	str	r3, [sp]
.L3101:
	ldrh	r2, [r4]
	mov	r3, #0
	strb	r3, [r4, #8]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L3102
.L3103:
.L3125:
	ldr	r3, .L3144+12
	mov	r2, #0
	ldr	lr, [r6, #-2176]
	add	r0, r4, #14
	mov	r7, r2
	movw	r9, #65535
	ldrh	r3, [r3]
	mov	r10, #36
	str	r3, [sp, #4]
	ldr	r3, [r6, #-2072]
	str	r3, [sp, #16]
	ldr	r3, [r6, #-2180]
	str	r3, [sp, #20]
	ldr	r3, .L3144+16
	ldrh	ip, [r3]
	ldrh	r8, [r3, #2]
.L3104:
	ldr	r1, [sp, #4]
	uxth	r3, r2
	cmp	r3, r1
	bcs	.L3142
	ldrh	r3, [r0, #2]!
	cmp	r3, r9
	beq	.L3105
	ldr	r1, [sp, #16]
	orr	r3, r5, r3, asl #10
	mla	r1, r10, r7, r1
	str	r3, [r1, #4]
	mul	r3, ip, r7
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	ldr	fp, [sp, #20]
	bic	r3, r3, #3
	add	r3, fp, r3
	str	r3, [r1, #8]
	mul	r3, r8, r7
	add	r7, r7, #1
	uxth	r7, r7
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	bic	r3, r3, #3
	add	r3, lr, r3
	str	r3, [r1, #12]
.L3105:
	add	r2, r2, #1
	b	.L3104
.L3142:
	ldr	r0, [r6, #-2072]
	mov	r1, r7
	mov	r2, #0
	ldr	fp, .L3144+4
	bl	FlashReadPages
	mov	r10, #0
.L3107:
	uxth	r3, r10
	cmp	r3, r7
	bcs	.L3143
	mov	r9, #36
	ldr	r2, [r6, #-2072]
	mul	r9, r9, r10
	ldr	ip, .L3144+8
	add	r3, r2, r9
	ldr	r8, [r2, r9]
	ldr	r1, [r3, #4]
	cmp	r8, #0
	ldr	r3, [r3, #12]
	str	r1, [sp, #4]
	bne	.L3108
	ldrh	r1, [r3]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L3109
.L3111:
	ldr	r2, .L3144+8
	ldrh	r3, [r4]
	ldr	r2, [r2, #-3028]
	b	.L3141
.L3109:
	ldr	r0, [r3, #8]
	ldr	r2, [fp, #2964]
	cmp	r0, r2
	bhi	.L3111
	ldrb	r2, [fp]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3113
	add	r1, sp, #24
	mov	r2, r8
	str	r3, [sp, #16]
	bl	log2phys
	ldr	r3, [sp, #16]
	ldr	r1, [sp, #24]
	ldr	r2, [r3, #12]
	cmn	r1, #1
	rsb	r0, r2, r1
	clz	r0, r0
	mov	r0, r0, lsr #5
	moveq	r0, #0
	cmp	r0, #0
	beq	.L3113
	str	r2, [sp, #32]
	mov	r1, #1
	ldr	r2, [r6, #-2040]
	add	r0, sp, #28
	str	r2, [sp, #36]
	ldr	r2, [r6, #-2032]
	str	r2, [sp, #40]
	mov	r2, r8
	bl	FlashReadPages
	ldr	r2, .L3144+20
	ldr	r1, [r6, #-2072]
	ldrh	r2, [r2]
	add	r9, r1, r9
	mov	r2, r2, asl #7
	ldr	r1, [sp, #36]
	ldr	r3, [sp, #16]
	b	.L3115
.L3116:
	add	r8, r8, #1
.L3115:
	cmp	r8, r2
	beq	.L3113
	ldr	r0, [r9, #8]
	ldr	ip, [r0, r8, asl #2]
	ldr	r0, [r1, r8, asl #2]
	cmp	ip, r0
	beq	.L3116
	ldr	r2, [sp, #32]
	ldr	r0, .L3144+24
	ldrh	r1, [r4]
	bl	printk
	ldrh	r3, [r4]
	ldr	r2, [r6, #-3028]
.L3141:
	mov	r3, r3, asl #1
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	ldr	r2, .L3144+28
	mvn	r3, #0
	strh	r3, [r4]	@ movhi
	strh	r3, [r2]	@ movhi
	b	.L3140
.L3113:
	ldr	r0, [r3, #12]
	add	r10, r10, #1
	ldr	r1, [sp, #4]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
	b	.L3107
.L3108:
	ldr	r2, [sp, #4]
	ldr	r0, .L3144+32
	ldrh	r1, [r4]
	str	ip, [sp, #16]
	bl	printk
	ldr	r3, .L3144+4
	ldrh	r5, [r4]
	ldr	r2, [r3, #2832]
	cmp	r2, #0
	ldr	ip, [sp, #16]
	bne	.L3119
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3120
.L3119:
	ldr	r2, [ip, #-3092]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #159
	bls	.L3121
.L3120:
	ldr	r3, [ip, #-2072]
	ldr	r3, [r3, r9]
	cmn	r3, #1
	bne	.L3122
.L3121:
	ldr	r3, [ip, #-2072]
	add	r9, r3, r9
	ldr	r3, [r9, #4]
	str	r3, [sp, #8]
.L3122:
	ldr	r3, .L3144+8
	mov	r5, r5, asl #1
	mov	r2, #0
	ldr	r3, [r3, #-3028]
	strh	r2, [r3, r5]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r4]	@ movhi
.L3140:
	bl	FtlGcPageVarInit
	mov	r5, #0
	b	.L3101
.L3143:
	ldr	r3, [sp]
	add	r5, r5, #1
	ldr	r2, [sp, #12]
	add	r3, r3, #1
	uxth	r5, r5
	cmp	r3, r2
	str	r3, [sp]
	ldr	r2, .L3144+36
	bcs	.L3124
.L3126:
	ldrh	r3, [r2]
	cmp	r3, r5
	bhi	.L3125
	b	.L3102
.L3124:
	ldr	r1, .L3144+40
	movw	r0, #65535
	ldrh	r3, [r1]
	cmp	r3, r0
	beq	.L3126
	ldr	r0, [sp]
	add	r3, r3, r0
	strh	r3, [r1]	@ movhi
	ldrh	r3, [r2]
	cmp	r3, r5
	bls	.L3126
	b	.L3127
.L3102:
	ldr	r2, .L3144
	movw	r3, #3288
	mvn	r1, #0
	strh	r5, [r4, #2]	@ movhi
	mov	r0, r4
	strh	r1, [r2, r3]	@ movhi
	mov	r2, #0
	mov	r1, r5
	strb	r2, [r4, #6]
	bl	ftl_sb_update_avl_pages
.L3127:
	ldr	r0, [sp, #8]
	add	sp, sp, #68
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3145:
	.align	2
.L3144:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2840
	.word	.LANCHOR0+2918
	.word	.LANCHOR0+2914
	.word	.LC148
	.word	.LANCHOR2-2768
	.word	.LC149
	.word	.LANCHOR0+2908
	.word	.LANCHOR1+3288
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r5, .L3163
	ldr	r4, .L3163+4
	ldr	r9, [r5, #-2636]
	mov	r6, r5
	cmp	r9, #0
	beq	.L3147
	ldr	r2, [r5, #-2632]
	ldr	r3, [r4, #2964]
	cmp	r2, r3
	bcs	.L3148
	mov	r5, #2048
	mov	r7, r6
.L3153:
	ldr	r0, [r6, #-2632]
	ldr	r3, [r4, #2964]
	cmp	r0, r3
	bcs	.L3152
	mov	r2, #0
	mov	r1, sp
	bl	log2phys
	ldr	r3, [r7, #-2632]
	add	r3, r3, #1
	str	r3, [r7, #-2632]
	ldr	r2, [sp]
	cmn	r2, #1
	beq	.L3151
	add	r0, sp, #40
	str	r2, [sp, #8]
	mov	r1, #1
	mov	r2, #0
	str	r2, [r0, #-36]!
	str	r3, [sp, #20]
	str	r2, [sp, #12]
	str	r2, [sp, #16]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmp	r3, #256
	bne	.L3152
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L3152:
	mvn	r0, #0
	b	.L3155
.L3151:
	subs	r5, r5, #1
	bne	.L3153
	b	.L3152
.L3148:
	ldr	r3, [r5, #-2848]
	mov	r0, #0
	str	r0, [r5, #-2636]
	str	r0, [r5, #-2632]
	str	r3, [r5, #-2640]
	b	.L3155
.L3147:
	ldr	r1, [r5, #-2796]
	ldr	r8, [r5, #-2848]
	ldr	r3, [r4, #2964]
	movw	r4, #10000
	ldr	r7, [r5, #-2640]
	cmp	r1, r4
	add	r2, r8, #1048576
	movhi	r4, #31
	movls	r4, #63
	cmp	r7, r2
	bhi	.L3157
	mov	r1, r1, lsr #10
	mov	r0, #1000
	mul	r0, r0, r3
	add	r1, r1, #1
	bl	__aeabi_uidiv
	add	r0, r0, r7
	cmp	r0, r8
	bcc	.L3157
	ldr	r3, .L3163+8
	ldrh	r3, [r3, #28]
	ands	r0, r4, r3
	movne	r0, r9
	bne	.L3155
	ldr	r2, [r5, #-2616]
	cmp	r2, r3
	beq	.L3155
.L3157:
	ldr	r3, .L3163+8
	mov	r0, #0
	str	r8, [r6, #-2640]
	str	r0, [r6, #-2632]
	ldrh	r3, [r3, #28]
	str	r3, [r6, #-2616]
	mov	r3, #1
	str	r3, [r6, #-2636]
.L3155:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3164:
	.align	2
.L3163:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2-3084
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #2908
	ldr	r5, .L3204
	.pad #28
	sub	sp, sp, #28
	ldr	r7, .L3204+4
	ldr	ip, [r5, #-3100]
	ldrh	r1, [r7, r3]
	cmp	ip, #0
	bne	.L3202
	sub	r4, r5, #2912
	movw	lr, #65535
	ldrh	r6, [r4]
	cmp	r6, lr
	bne	.L3168
.L3177:
	ldrh	r2, [r4]
	movw	r3, #65535
	ldr	r6, .L3204
	mov	r8, #0
	cmp	r2, r3
	str	r8, [r5, #-1908]
	sub	r9, r6, #2912
	beq	.L3202
	bl	FtlCacheWriteBack
	movw	r2, #2908
	ldrb	r0, [r6, #-2905]	@ zero_extendqisi2
	mov	fp, #12
	ldrh	r2, [r7, r2]
	ldrh	r3, [r9]
	ldr	r1, [r6, #-3028]
	smulbb	r2, r0, r2
	mov	r3, r3, asl #1
	strh	r2, [r1, r3]	@ movhi
	sub	r2, r6, #2144
	ldr	r3, [r6, #-2828]
	sub	r10, r2, #8
	ldrh	ip, [r2, #-8]
	add	r3, ip, r3
	str	r3, [r6, #-2828]
	stmib	sp, {r2, r6}
	b	.L3178
.L3168:
	cmp	r0, #0
	beq	.L3171
	ldr	r2, .L3204+8
	movw	r3, #3288
	ldrh	r0, [r2, r3]
	cmp	r0, lr
	beq	.L3172
.L3173:
	mov	r1, #2
	b	.L3171
.L3172:
	strh	ip, [r2, r3]	@ movhi
	sub	r3, r5, #3008
	ldrh	r3, [r3, #-4]
	cmp	r3, #17
	bhi	.L3173
.L3171:
	ldr	r8, .L3204
	sub	r0, r8, #2912
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #20]
	beq	.L3174
	ldr	r2, [r8, #-3092]
	mov	r6, r6, asl #1
	ldrh	r3, [r2, r6]
	cmp	r3, #4
	bls	.L3175
	sub	r3, r3, #5
	mov	r0, #1
	strh	r3, [r2, r6]	@ movhi
	bl	FtlEctTblFlush
.L3175:
	ldr	r3, [r5, #-1908]
	ldr	r2, .L3204
	cmp	r3, #0
	bne	.L3176
	ldr	r0, [sp, #20]
	ldr	r3, [r2, #-2620]
	ubfx	r0, r0, #10, #16
	add	r3, r3, #1
	str	r3, [r2, #-2620]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L3176:
	mov	r3, #0
	str	r3, [r5, #-1908]
	b	.L3188
.L3174:
	ldr	r2, .L3204+8
	movw	r3, #3288
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3188
	b	.L3177
.L3181:
	ldr	r3, [r6, #4]
	cmp	r0, r3
	bne	.L3200
.L3180:
	add	r8, r8, #1
.L3178:
	ldrh	r3, [r10]
	uxth	r9, r8
	cmp	r3, r9
	bls	.L3203
	ldr	r3, [sp, #8]
	mul	r9, fp, r9
	ldr	r2, [r7, #2964]
	ldr	ip, [r3, #-2156]
	add	r6, ip, r9
	ldr	r0, [r6, #8]
	cmp	r0, r2
	bcs	.L3200
	mov	r2, #0
	add	r1, sp, #20
	str	ip, [sp, #12]
	bl	log2phys
	ldr	ip, [sp, #12]
	ldr	r0, [sp, #20]
	ldr	r2, [ip, r9]
	cmp	r0, r2
	bne	.L3181
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r6, #4
	mov	r2, #1
	mov	r9, r0
	ldr	r0, [r6, #8]
	bl	log2phys
	mov	r0, r9
	b	.L3201
.L3200:
	ldrh	r0, [r4]
.L3201:
	bl	decrement_vpc_count
	b	.L3180
.L3203:
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrb	r3, [r7, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3183
	ldr	r0, .L3204+12
	ldrh	r1, [r4]
	bl	printk
.L3183:
	ldrh	r0, [r4]
	ldr	r2, [r5, #-3028]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3184
	bl	INSERT_DATA_LIST
	b	.L3185
.L3184:
	bl	INSERT_FREE_LIST
.L3185:
	ldr	r3, [sp, #4]
	mvn	r6, #0
	ldr	r5, .L3204
	strh	r6, [r4]	@ movhi
	mov	r4, #0
	strh	r4, [r3, #-8]	@ movhi
	sub	r3, r5, #2160
	strh	r4, [r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	sub	r3, r5, #2768
	sub	r2, r5, #3008
	strh	r6, [r3]	@ movhi
	ldr	r3, [r7, #2832]
	ldrh	r2, [r2, #-4]
	cmp	r3, r4
	sub	r3, r5, #2784
	ldrh	r3, [r3, #-4]
	beq	.L3186
	ldr	r1, [r5, #-2792]
	cmp	r1, #39
	bhi	.L3186
	cmp	r2, r3
	subcc	r5, r5, #2192
	movcc	r3, r3, asl #1
	strcch	r3, [r5, #-12]	@ movhi
	b	.L3202
.L3186:
	add	r1, r3, r3, asl #1
	cmp	r2, r1, asr #2
	ble	.L3202
	ldrb	r0, [r7, #1888]	@ zero_extendqisi2
	ldr	r2, .L3204+16
	cmp	r0, #0
	moveq	r3, #20
	streqh	r3, [r2, #-12]	@ movhi
	beq	.L3167
	sub	r3, r3, #2
	strh	r3, [r2, #-12]	@ movhi
.L3202:
	mov	r0, #0
	b	.L3167
.L3188:
	mov	r0, #1
.L3167:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3205:
	.align	2
.L3204:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC150
	.word	.LANCHOR2-2192
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r5, #2908
	ldr	r4, .L3209
	ldr	r6, .L3209+4
	sub	r7, r4, #2912
	ldrh	r1, [r6, r5]
	mov	r0, r7
	bl	FtlGcScanTempBlk
	ldrh	r2, [r7, #2]
	ldrh	r3, [r6, r5]
	cmp	r2, r3
	ldmccfd	sp!, {r3, r4, r5, r6, r7, pc}
	sub	r0, r4, #1968
	bl	FtlMapBlkWriteDumpData
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	mov	r3, #0
	str	r3, [r4, #-1908]
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3210:
	.align	2
.L3209:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, #0
	ldr	r4, .L3213
	ldr	r3, .L3213+4
	sub	r6, r4, #3008
	sub	r4, r4, #2960
	mov	r0, r6
	str	r5, [r3, #316]
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
	ldmfd	sp!, {r4, r5, r6, pc}
.L3214:
	.align	2
.L3213:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.fnend
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	movw	r3, #314
	ldr	r4, .L3233
	mov	r2, #0
	ldr	r6, .L3233+4
	ldr	r5, .L3233+8
	strh	r2, [r4, r3]	@ movhi
	add	r3, r4, #312
	mvn	r2, #0
	strh	r2, [r3]	@ movhi
	ldr	r3, [r6, #2844]
	uxth	r0, r3
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	sub	r3, r5, #2784
	ldrh	r2, [r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3216
.L3218:
	mvn	r9, #0
	b	.L3217
.L3216:
	bl	FtlLoadSysInfo
	subs	r9, r0, #0
	bne	.L3218
	bl	FtlLoadMapInfo
	mov	r8, r4
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	r0, #1
	bl	FtlUpdateVaildLpn
	ldr	r2, [r5, #-2864]
	movw	r3, #2946
	mov	r0, #12
	ldrh	r1, [r6, r3]
	mov	r3, r9
.L3219:
	cmp	r3, r1
	bge	.L3224
	mla	ip, r0, r3, r2
	ldr	ip, [ip, #4]
	cmp	ip, #0
	bge	.L3220
.L3224:
	ldr	r4, .L3233+12
	cmp	r3, r1
	add	r7, r4, #76
	ldrh	r2, [r4, #28]
	add	r2, r2, #1
	strh	r2, [r4, #28]	@ movhi
	bge	.L3231
	b	.L3221
.L3220:
	add	r3, r3, #1
	b	.L3219
.L3231:
	movw	r3, #314
	ldrh	r3, [r8, r3]
	cmp	r3, #0
	beq	.L3225
.L3221:
	ldrh	r3, [r7]
	movw	ip, #2908
	ldr	r1, [r5, #-3028]
	ldrh	r0, [r7, #4]
	mov	r3, r3, asl #1
	ldrh	r2, [r1, r3]
	rsb	r2, r0, r2
	strh	r2, [r1, r3]	@ movhi
	ldrh	r3, [r6, ip]
	mov	r2, #0
	ldr	lr, [r5, #-3028]
	strb	r2, [r5, #-3002]
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, .L3233+16
	strh	r2, [r7, #4]	@ movhi
	ldrh	r1, [r3]
	ldrh	r8, [r3, #4]
	mov	r1, r1, asl #1
	ldrh	r0, [lr, r1]
	rsb	r0, r8, r0
	strh	r0, [lr, r1]	@ movhi
	ldrh	r1, [r6, ip]
	strh	r2, [r3, #4]	@ movhi
	strb	r2, [r5, #-2954]
	strh	r1, [r3, #2]	@ movhi
	ldrh	r3, [r4, #30]
	add	r3, r3, #1
	strh	r3, [r4, #30]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L3225:
	ldrh	r0, [r7]
	movw	r3, #65535
	ldr	r8, .L3233+20
	cmp	r0, r3
	beq	.L3226
	ldrh	r3, [r8, #4]
	cmp	r3, #0
	bne	.L3226
	ldrh	r3, [r8, #52]
	add	r5, r8, #48
	cmp	r3, #0
	bne	.L3226
	bl	FtlGcRefreshOpenBlock
	ldrh	r0, [r8, #48]
	bl	FtlGcRefreshOpenBlock
	bl	FtlVpcTblFlush
	mov	r0, r8
	bl	allocate_new_data_superblock
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3226:
	ldrb	r3, [r6]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3227
	ldrh	r3, [r4, #28]
	tst	r3, #31
	bne	.L3217
.L3227:
	bl	FtlVpcCheckAndModify
.L3217:
	mov	r0, r9
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L3234:
	.align	2
.L3233:
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2-3084
	.word	.LANCHOR2-2960
	.word	.LANCHOR2-3008
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	FtlLowFormat
	.type	FtlLowFormat, %function
FtlLowFormat:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r4, .L3270
	ldr	r5, [r4, #-3100]
	cmp	r5, #0
	bne	.L3237
	ldr	r6, .L3270+4
	mov	r1, r5
	ldr	r0, [r4, #-1984]
	add	r7, r6, #2944
	ldrh	r2, [r7]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r7]
	mov	r1, r5
	ldr	r0, [r4, #-1988]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r3, [r6, #2844]
	str	r5, [r4, #-2816]
	str	r5, [r4, #-2812]
	uxth	r0, r3
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L3238
	bl	FtlMakeBbt
.L3238:
	ldr	r0, .L3270+8
	mov	r2, #0
	ldr	ip, .L3270+12
.L3239:
	ldrh	r1, [r0]
	uxth	r3, r2
	add	r2, r2, #1
	cmp	r3, r1, asl #7
	bge	.L3267
	ldr	lr, [r4, #-2044]
	mvn	r1, r3
	orr	r1, r3, r1, asl #16
	str	r1, [lr, r3, asl #2]
	ldr	r1, [r4, #-2040]
	str	ip, [r1, r3, asl #2]
	b	.L3239
.L3267:
	ldr	r9, .L3270+16
	mov	r7, #0
	add	fp, r9, #2
	ldrh	r5, [r9]
.L3241:
	ldrh	r3, [fp]
	ldr	r10, .L3270+4
	cmp	r3, r5
	ldr	r8, .L3270+20
	bls	.L3268
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
	add	r0, r7, r0
	uxth	r7, r0
	b	.L3241
.L3268:
	movw	r3, #2840
	ldrh	r1, [r10, r3]
	sub	r3, r7, #3
	cmp	r3, r1, asl #1
	bge	.L3243
.L3247:
	mov	r5, #0
	mov	r8, r5
	b	.L3244
.L3243:
	mov	r0, r7
	bl	__aeabi_uidiv
	ldr	r3, [r10, #2940]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r0, [r10, #2844]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	ldrh	r5, [r9]
.L3245:
	ldrh	r3, [r8]
	cmp	r3, r5
	bls	.L3247
	mov	r0, r5
	add	r5, r5, #1
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	uxth	r5, r5
	b	.L3245
.L3244:
	ldrh	r2, [r9]
	uxth	r0, r5
	ldr	r10, .L3270+4
	add	r5, r5, #1
	cmp	r2, r0
	bls	.L3269
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r0, r8, r0
	uxth	r8, r0
	b	.L3244
.L3269:
	movw	r2, #2850
	ldr	ip, [r10, #2852]
	ldrh	r2, [r10, r2]
	ldr	r5, .L3270+24
	mov	r0, ip
	str	ip, [sp, #4]
	str	r2, [r4, #-2084]
	movw	r2, #2840
	ldrh	r7, [r10, r2]
	mov	r1, r7
	bl	__aeabi_uidiv
	ubfx	r3, r0, #5, #16
	mov	fp, r0
	add	r2, r3, #36
	strh	r2, [r5, #-4]	@ movhi
	mov	r2, #24
	str	r0, [r10, #2964]
	mul	r2, r2, r7
	str	r3, [sp]
	ldr	ip, [sp, #4]
	cmp	r8, r2
	ble	.L3249
	rsb	r0, r8, ip
	mov	r1, r7
	bl	__aeabi_uidiv
	str	r0, [r10, #2964]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r5, #-4]	@ movhi
.L3249:
	ldr	r3, [r6, #2832]
	cmp	r3, #1
	bne	.L3250
	ldrh	r3, [r5, #-4]
	mov	r0, r8
	mov	r1, r7
	str	r3, [sp, #4]
	bl	__aeabi_uidiv
	ldr	r3, [sp, #4]
	uxtah	r0, r3, r0
	add	r3, r3, r0, asr #2
	strh	r3, [r5, #-4]	@ movhi
.L3250:
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3251
	ldrh	r3, [r5, #-4]
	mov	r0, r8
	mov	r1, r7
	str	r3, [sp, #4]
	bl	__aeabi_uidiv
	ldr	r3, [sp, #4]
	uxtah	r0, r3, r0
	add	r3, r3, r0, asr #2
	strh	r3, [r5, #-4]	@ movhi
.L3251:
	movw	r3, #2902
	ldrh	r3, [r6, r3]
	cmp	r3, #0
	beq	.L3253
	ldrh	r2, [r5, #-4]
	add	r2, r2, r3, lsr #1
	strh	r2, [r5, #-4]	@ movhi
	mul	r2, r7, r3
	cmp	r2, r8
	ble	.L3253
	ldr	r2, .L3270+4
	add	r3, r3, #32
	str	fp, [r2, #2964]
	ldr	r2, [sp]
	add	r3, r2, r3
	ldr	r2, .L3270+24
	strh	r3, [r2, #-4]	@ movhi
.L3253:
	ldr	r3, [r6, #2964]
	mvn	r10, #0
	ldrh	r2, [r5, #-4]
	rsb	r3, r2, r3
	mul	r7, r7, r3
	movw	r3, #2908
	ldrh	r3, [r6, r3]
	str	r7, [r4, #252]
	mul	r7, r3, r7
	movw	r3, #2914
	ldrh	r3, [r6, r3]
	str	r7, [r6, #2964]
	mul	r7, r3, r7
	str	r7, [r6, #1892]
	bl	FtlBbmTblFlush
	movw	r3, #2850
	mov	r1, #0
	ldrh	r2, [r6, r3]
	mov	r6, #0
	ldr	r0, [r4, #-3028]
	ldr	r7, .L3270+28
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r9]
	sub	r8, r7, #240
	strh	r10, [r7]	@ movhi
	strh	r6, [r7, #2]	@ movhi
	mov	r1, #255
	mov	r2, r2, lsr #3
	ldr	r0, [r4, #-2852]
	add	r7, r7, #2768
	mov	r3, #1
	str	r6, [r4, #-2856]
	strb	r6, [r4, #-2762]
	strb	r6, [r4, #-2760]
	strh	r6, [r8, #2]	@ movhi
	strb	r6, [r4, #-3002]
	strh	r6, [r8]	@ movhi
	strb	r3, [r4, #-3000]
	bl	ftl_memset
	mov	r2, r8
.L3255:
	ldr	r6, .L3270
	str	r2, [sp]
	sub	fp, r6, #3008
	mov	r0, fp
	bl	make_superblock
	ldrb	r3, [r4, #-3001]	@ zero_extendqisi2
	ldrh	r9, [r8]
	cmp	r3, #0
	bne	.L3256
	ldr	r3, [r7, #-3028]
	mov	r9, r9, asl #1
	ldr	r2, [sp]
	strh	r10, [r3, r9]	@ movhi
	ldrh	r3, [r2]
	add	r3, r3, #1
	strh	r3, [r2]	@ movhi
	b	.L3255
.L3256:
	ldr	r3, [r6, #-2816]
	mov	r9, r9, asl #1
	ldrh	r2, [fp, #4]
	str	r3, [r6, #-2996]
	add	r3, r3, #1
	str	r3, [r6, #-2816]
	ldr	r3, [r6, #-3028]
	strh	r2, [r3, r9]	@ movhi
	sub	r2, r6, #2960
	mov	r3, #0
	strb	r3, [r6, #-2954]
	strh	r3, [r2, #2]	@ movhi
	mov	r8, r2
	ldrh	r3, [fp]
	mov	r10, r2
	mvn	fp, #0
	add	r3, r3, #1
	strh	r3, [r2]	@ movhi
	mov	r3, #1
	strb	r3, [r6, #-2952]
.L3257:
	ldr	r7, .L3270
	sub	r9, r7, #2960
	mov	r0, r9
	bl	make_superblock
	ldrb	r3, [r4, #-2953]	@ zero_extendqisi2
	cmp	r3, #0
	ldrh	r3, [r8]
	bne	.L3258
	ldr	r2, [r6, #-3028]
	mov	r3, r3, asl #1
	strh	fp, [r2, r3]	@ movhi
	ldrh	r3, [r10]
	add	r3, r3, #1
	strh	r3, [r10]	@ movhi
	b	.L3257
.L3258:
	ldr	r2, [r7, #-2816]
	mov	r3, r3, asl #1
	ldrh	r1, [r9, #4]
	mvn	r4, #0
	str	r2, [r7, #-2948]
	add	r2, r2, #1
	str	r2, [r7, #-2816]
	ldr	r2, [r7, #-3028]
	strh	r1, [r2, r3]	@ movhi
	sub	r3, r7, #2912
	strh	r4, [r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	mov	r3, #0
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, [r7, #252]
	strh	r4, [r5, #4]	@ movhi
	strh	r3, [r5, #6]	@ movhi
	ldr	r3, [r7, #-2816]
	str	r3, [r7, #-2776]
	add	r3, r3, #1
	str	r3, [r7, #-2816]
	strh	r0, [r5]	@ movhi
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	ldreq	r3, .L3270+32
	moveq	r2, #1
	streq	r2, [r3, #504]
.L3237:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3271:
	.align	2
.L3270:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+2914
	.word	168778952
	.word	.LANCHOR0+2848
	.word	.LANCHOR0+2850
	.word	.LANCHOR2-2784
	.word	.LANCHOR2-2768
	.word	.LANCHOR1
	.fnend
	.size	FtlLowFormat, .-FtlLowFormat
	.align	2
	.global	FtlReInitForSDUpdata
	.type	FtlReInitForSDUpdata, %function
FtlReInitForSDUpdata:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L3301
	ldrb	r3, [r4, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3273
.L3275:
	mov	r0, #0
	b	.L3274
.L3273:
	ldr	r5, .L3301+4
	ldr	r0, [r5, #148]
	bl	FlashInit
	cmp	r0, #0
	bne	.L3275
	bl	FlashLoadFactorBbt
	cmp	r0, #0
	beq	.L3276
	bl	FlashMakeFactorBbt
.L3276:
	ldr	r0, [r5, #180]
	bl	FlashReadIdbDataRaw
	cmp	r0, #0
	beq	.L3277
	mov	r1, #0
	mov	r2, #16
	mov	r0, sp
	bl	FlashReadFacBbtData
	mov	r3, #0
	mov	r2, r3
	mov	ip, #1
	ldr	r1, [sp]
.L3278:
	ands	lr, r1, ip, asl r2
	add	r0, r3, #1
	add	r2, r2, #1
	movne	r3, r0
	cmp	r2, #16
	bne	.L3278
	cmp	r3, #6
	bls	.L3297
	mov	r2, #0
	mov	ip, #1
.L3281:
	ands	lr, r1, ip, asl r2
	add	r0, r3, #1
	add	r2, r2, #1
	movne	r3, r0
	cmp	r2, #24
	bne	.L3281
	cmp	r3, #17
	movhi	r3, #36
	strhib	r3, [r4, #1]
	bhi	.L3280
.L3297:
	strb	r2, [r4, #1]
.L3280:
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	ldr	r3, .L3301+8
	strh	r2, [r3, #26]	@ movhi
.L3277:
	ldr	r1, .L3301+12
	ldr	r0, .L3301+16
	bl	printk
	ldr	r0, .L3301+8
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldr	r0, [r4, #2844]
	mov	r4, #1
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
.L3283:
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L3284
.L3299:
	bl	FtlLowFormat
	cmp	r4, #3
	addls	r4, r4, #1
	bls	.L3283
.L3300:
	mvn	r0, #0
	b	.L3274
.L3284:
	bl	FtlSysBlkInit
	cmp	r0, #0
	bne	.L3299
	ldr	r3, .L3301+20
	mov	r2, #1
	str	r2, [r3, #504]
.L3274:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L3302:
	.align	2
.L3301:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+1860
	.word	.LC77
	.word	.LC76
	.word	.LANCHOR1
	.fnend
	.size	FtlReInitForSDUpdata, .-FtlReInitForSDUpdata
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	ldr	r5, .L3320
	ldr	r3, [r5, #-3100]
	cmp	r3, #0
	beq	.L3304
.L3307:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3304:
	ldr	r3, .L3320+4
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3306
	ldr	r3, [r5, #-2188]
	tst	r3, #1
	beq	.L3306
	sub	r3, r5, #2912
	ldrh	r3, [r3, #4]
	cmp	r3, #0
	bne	.L3307
.L3306:
	mov	r2, #0
	ldr	r0, [r5, #-2068]
	ldr	r1, [r5, #-2188]
	mov	r3, r2
	bl	FlashProgPages
	ldr	r7, .L3320
	mov	r6, #0
	mov	r8, #36
.L3308:
	ldr	r1, [r5, #-2188]
	uxth	r3, r6
	ldr	r4, .L3320
	cmp	r3, r1
	bcs	.L3319
	mul	r3, r8, r3
	ldr	r0, [r7, #-2068]
	add	r6, r6, #1
	add	r1, r0, r3
	ldr	r0, [r0, r3]
	ldr	r2, [r1, #12]
	cmn	r0, #1
	bne	.L3309
	sub	r1, r4, #2912
	ldr	ip, [r4, #-3028]
	mov	lr, #0
	ldrh	r2, [r1]
	mov	r2, r2, asl #1
	strh	lr, [ip, r2]	@ movhi
	ldr	r2, [r4, #-2620]
	strh	r0, [r1]	@ movhi
	add	r2, r2, #1
	str	r2, [r4, #-2620]
	ldr	r2, [r4, #-2068]
	add	r3, r2, r3
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	b	.L3318
.L3309:
	ldr	r0, [r2, #12]
	ldr	r1, [r1, #4]
	ldr	r2, [r2, #8]
	bl	FtlGcUpdatePage
	b	.L3308
.L3319:
	ldr	r0, [r4, #-2068]
	sub	r4, r4, #2912
	bl	FtlGcBufFree
	mov	r3, #0
	str	r3, [r4, #724]
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L3307
	mov	r0, #1
	bl	FtlGcFreeTempBlock
.L3318:
	mov	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3321:
	.align	2
.L3320:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r2, #65535
	ldr	r3, .L3326
	ldrh	r1, [r3]
	cmp	r1, r2
	beq	.L3323
	ldrh	r3, [r3, #4]
	cmp	r3, #0
	bne	.L3324
.L3323:
	ldr	r4, .L3326+4
	bl	FtlCacheWriteBack
	mov	r0, #0
	mov	r5, #0
	bl	FtlGcFreeTempBlock
	sub	r0, r4, #2912
	strb	r5, [r4, #-2904]
	bl	allocate_data_superblock
	sub	r3, r4, #2160
	sub	r4, r4, #2144
	strh	r5, [r3]	@ movhi
	strh	r5, [r4, #-8]	@ movhi
	bl	l2p_flush
	mov	r0, r5
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3324:
	ldr	r0, .L3326
	ldmfd	sp!, {r3, r4, r5, lr}
	b	get_new_active_ppa
.L3327:
	.align	2
.L3326:
	.word	.LANCHOR2-2912
	.word	.LANCHOR2
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.global	ftl_do_gc
	.type	ftl_do_gc, %function
ftl_do_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L3492
	ldr	ip, [r2, #-3100]
	cmp	ip, #0
	bne	.L3422
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	lr, .L3492+4
	ldr	r3, [lr, #504]
	cmn	r3, #1
	beq	.L3424
	ldr	r5, [r2, #-2096]
	cmp	r5, #0
	bne	.L3424
	sub	r8, r2, #3008
	ldrh	r3, [r8, #-12]
	mov	r7, r8
	cmp	r3, #47
	movls	r0, r5
	bls	.L3481
	movw	r3, #3288
	str	r1, [sp, #8]
	ldrh	r1, [lr, r3]
	movw	r3, #65535
	mov	r6, r2
	str	r0, [sp, #16]
	cmp	r1, r3
	bne	.L3330
.L3333:
	ldr	r5, .L3492+8
	movw	r2, #65535
	ldrh	r4, [r5, #-4]
	cmp	r4, r2
	bne	.L3331
	b	.L3332
.L3330:
	sub	r2, r2, #2912
	ldrh	r2, [r2]
	cmp	r2, r3
	beq	.L3333
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	beq	.L3333
	mov	r0, #1
	b	.L3481
.L3331:
	ldrh	r3, [r5, #-6]
	cmp	r3, r2
	bne	.L3332
	ldrh	r1, [r5, #-2]
	cmp	r1, r3
	beq	.L3332
	ldrh	r2, [r5]
	cmp	r2, r3
	strneh	r4, [r5, #-6]	@ movhi
	strneh	r1, [r5, #-4]	@ movhi
	mvnne	r3, #0
	strneh	r2, [r5, #-2]	@ movhi
	strneh	r3, [r5]	@ movhi
.L3332:
	ldr	r2, [sp, #16]
	ldr	r3, [r6, #-2200]
	cmp	r2, #1
	ldr	r8, .L3492+12
	add	r3, r3, #1
	add	r3, r3, r2, asl #7
	str	r3, [r6, #-2200]
	bne	.L3344
	ldr	r4, .L3492+16
	ldr	r2, [r4, #2832]
	cmp	r2, #0
	bne	.L3335
	ldrb	r2, [r4, #1888]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3344
.L3335:
	ldr	r2, [r6, #-2792]
	ldr	r10, .L3492
	cmp	r2, #39
	bhi	.L3344
	ldr	r9, .L3492+20
	ldrh	r2, [r9]
	add	r3, r3, r2
	str	r3, [r10, #-2200]
	bl	FtlGcReFreshBadBlk
	ldrh	r3, [r8]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L3344
	ldrh	r2, [r5, #-6]
	cmp	r2, r3
	bne	.L3344
	ldr	r3, [r10, #-2200]
	cmp	r3, #1024
	bhi	.L3336
	ldrh	r3, [r7, #-4]
	cmp	r3, #63
	bhi	.L3344
.L3336:
	ldr	r1, .L3492
	mov	r3, #0
	strh	r3, [r9]	@ movhi
	sub	fp, r1, #2192
	ldrh	r0, [r7, #-4]
	ldr	r2, .L3492+20
	ldrh	r9, [fp, #-10]
	add	ip, r9, #64
	cmp	r0, ip
	bgt	.L3344
	str	r3, [r1, #-2200]
	ldr	r3, [r1, #-2792]
	cmp	r3, #0
	moveq	r3, #6
	beq	.L3483
	cmp	r3, #5
	bhi	.L3338
	mov	r3, #18
.L3483:
	strh	r3, [r2]	@ movhi
.L3338:
	mov	r0, #32
	bl	List_get_gc_head_node
	movw	ip, #65535
	uxth	r2, r0
	cmp	r2, ip
	beq	.L3343
	ldrh	r0, [fp, #-4]
	ldr	r10, .L3492
	cmp	r0, #0
	sub	r9, r10, #2192
	beq	.L3340
	movw	lr, #2910
	movw	fp, #2840
	ldrh	lr, [r4, lr]
	mov	r2, r2, asl #1
	ldrh	r4, [r4, fp]
	ldr	r3, [r10, #-3028]
	mul	lr, r4, lr
	ldrh	r1, [r3, r2]
	add	lr, lr, #1
	cmp	r1, lr
	bgt	.L3343
	add	r1, r0, #1
	str	r2, [sp, #28]
	str	ip, [sp, #24]
	mov	fp, #0
	uxth	r1, r1
	str	r3, [sp, #20]
	strh	r1, [r9, #-4]	@ movhi
	str	r1, [sp, #12]
	str	fp, [r10, #-2192]
	bl	List_get_gc_head_node
	ldr	ip, [sp, #24]
	uxth	r4, r0
	ldr	r1, [sp, #12]
	cmp	r4, ip
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #28]
	beq	.L3343
	mov	ip, r4, asl #1
	ldr	r0, .L3492+24
	str	ip, [sp, #12]
	ldrh	lr, [r3, ip]
	ldrh	r3, [r3, r2]
	mov	r2, r4
	str	r3, [sp]
	mov	r3, lr
	bl	printk
	ldrh	r3, [r9, #-4]
	cmp	r3, #40
	ldr	ip, [sp, #12]
	bls	.L3341
	ldr	r3, [r10, #-3028]
	ldrh	r3, [r3, ip]
	cmp	r3, #32
	strhih	fp, [r9, #-4]	@ movhi
.L3341:
	ldr	r3, .L3492+20
	mov	r2, #6
	strh	r2, [r3]	@ movhi
	b	.L3345
.L3340:
	mov	r3, #1
	strh	r3, [r9, #-4]	@ movhi
.L3343:
	bl	GetSwlReplaceBlock
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	bne	.L3345
	ldr	r3, .L3492+20
	mov	r2, #0
	strh	r2, [r3]	@ movhi
.L3344:
	ldrh	r3, [r8]
	movw	r4, #65535
	ldr	r9, .L3492
	cmp	r3, r4
	bne	.L3345
	sub	r2, r9, #2912
	ldrh	r4, [r2]
	cmp	r4, r3
	movne	r4, r3
	beq	.L3489
.L3345:
	movw	r2, #65535
	rsb	r3, r2, r4
	clz	r3, r3
	ldr	r1, [sp, #16]
	mov	r3, r3, lsr #5
	cmp	r1, #0
	movne	ip, #0
	andeq	ip, r3, #1
	cmp	ip, #0
	beq	.L3358
	ldrh	r0, [r7, #-4]
	cmp	r0, #24
	movhi	fp, #1
	bhi	.L3359
	ldr	r2, .L3492+16
	movw	r3, #2908
	cmp	r0, #16
	ldrh	fp, [r2, r3]
	movhi	fp, fp, lsr #5
	bhi	.L3359
	cmp	r0, #12
	movhi	fp, fp, lsr #4
	bhi	.L3359
	cmp	r0, #8
	movhi	fp, fp, lsr #2
.L3359:
	ldr	r1, .L3492
	sub	r2, r1, #2192
	ldrh	r3, [r2, #-12]
	cmp	r3, r0
	bcs	.L3363
	sub	r3, r1, #2912
	movw	r0, #65535
	ldrh	r3, [r3]
	cmp	r3, r0
	bne	.L3364
	sub	r0, r1, #2144
	ldrh	r0, [r0, #-6]
	cmp	r0, r3
	bne	.L3364
	ldr	r3, .L3492+20
	ldrh	r0, [r3]
	cmp	r0, #0
	bne	.L3365
	ldr	r3, .L3492+16
	ldr	r1, [r1, #-2856]
	ldr	r3, [r3, #2964]
	add	r3, r3, r3, asl #1
	cmp	r1, r3, lsr #2
	movcs	r3, #18
	bcs	.L3366
.L3365:
	ldr	r3, .L3492+28
	ldrh	r3, [r3, #-4]
	add	r3, r3, r3, asl #1
	ubfx	r3, r3, #2, #16
.L3366:
	strh	r3, [r2, #-12]	@ movhi
	mov	r3, #0
	str	r3, [r6, #-2192]
	b	.L3481
.L3364:
	ldr	r3, .L3492+28
	ldrh	r3, [r3, #-4]
	add	r3, r3, r3, asl #1
	mov	r3, r3, asr #2
	strh	r3, [r2, #-12]	@ movhi
.L3363:
	ldr	r3, .L3492+16
	movw	r4, #65535
	ldr	r2, [r3, #2832]
	ldr	r3, [sp, #8]
	cmp	r3, #2
	movhi	r3, #0
	movls	r3, #1
	cmp	r2, #0
	moveq	r3, #0
	cmp	r3, #0
	addne	fp, fp, #1
	uxthne	fp, fp
	b	.L3368
.L3358:
	ldr	r1, .L3492
	sub	r0, r1, #2912
	ldrh	r0, [r0]
	cmp	r0, r2
	bne	.L3369
	ldrh	r2, [r5, #-6]
	cmp	r2, r0
	movne	r3, #0
	andeq	r3, r3, #1
	cmp	r3, #0
	beq	.L3369
	sub	r3, r1, #2768
	ldrh	r3, [r3]
	cmp	r3, r0
	beq	.L3370
.L3374:
	movw	r4, #65535
	b	.L3369
.L3370:
	mov	r2, r1
	ldrh	r0, [r7, #-4]
	str	ip, [r2, #-2192]!
	mov	r9, r2
	ldrh	r3, [r2, #-12]
	ldr	r4, .L3492+20
	cmp	r0, r3
	bls	.L3371
	ldrh	r3, [r4]
	cmp	r3, #0
	bne	.L3372
	ldr	r3, .L3492+16
	ldr	r2, [r1, #-2856]
	ldr	r3, [r3, #2964]
	add	r3, r3, r3, asl #1
	cmp	r2, r3, lsr #2
	movcs	r3, #18
	bcs	.L3373
.L3372:
	ldr	r3, .L3492+28
	ldrh	r3, [r3, #-4]
	add	r3, r3, r3, asl #1
	ubfx	r3, r3, #2, #16
.L3373:
	strh	r3, [r9, #-12]	@ movhi
	bl	FtlReadRefresh
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, [r6, #-3028]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	cmp	r3, #4
	ldrhih	r0, [r4]
	bhi	.L3481
.L3371:
	ldrh	r0, [r4]
	cmp	r0, #0
	bne	.L3374
	ldr	r3, .L3492+28
	ldrh	r4, [r3, #-4]
	add	r3, r4, r4, asl #1
	mov	r3, r3, asr #2
	strh	r3, [r9, #-12]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r6, #-3028]
	movw	r2, #2910
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r1, [r3, r0]
	movw	r0, #2840
	ldr	r3, .L3492+16
	ldrh	r2, [r3, r2]
	ldrh	r3, [r3, r0]
	mul	r3, r3, r2
	add	r3, r3, r3, lsr #31
	cmp	r1, r3, asr #1
	ble	.L3375
	ldrh	r3, [r7, #-4]
	sub	r2, r4, #1
	cmp	r3, r2
	blt	.L3375
	bl	FtlReadRefresh
	b	.L3487
.L3375:
	cmp	r1, #0
	bne	.L3374
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r0, [r7, #-4]
	add	r0, r0, #1
	b	.L3481
.L3369:
	ldr	r3, .L3492+16
	ldr	r3, [r3, #2832]
	cmp	r3, #0
	moveq	fp, #1
	movne	fp, #2
.L3368:
	ldrh	r3, [r8]
	movw	r0, #65535
	ldr	r2, .L3492
	cmp	r3, r0
	sub	r1, r2, #2768
	bne	.L3377
	cmp	r4, r3
	strneh	r4, [r1]	@ movhi
	bne	.L3379
	sub	r1, r2, #2144
	ldrh	r3, [r1, #-6]
	cmp	r3, r4
	beq	.L3379
	ldr	r2, [r2, #-3028]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	mvneq	r3, #0
	streqh	r3, [r1, #-6]	@ movhi
	ldrh	r3, [r5, #-6]
	strh	r3, [r8]	@ movhi
	mvn	r3, #0
	strh	r3, [r5, #-6]	@ movhi
.L3379:
	ldrh	r0, [r8]
	mov	r3, #0
	strb	r3, [r6, #-2760]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L3377
	bl	IsBlkInGcList
	cmp	r0, #0
	ldrne	r3, .L3492+12
	mvnne	r2, #0
	strneh	r2, [r3]	@ movhi
	ldr	r3, .L3492+16
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3383
	ldrh	r0, [r8]
	bl	ftl_get_blk_mode
	strb	r0, [r6, #-2760]
.L3383:
	ldrh	r2, [r8]
	movw	r3, #65535
	ldr	r6, .L3492
	cmp	r2, r3
	sub	r5, r6, #2768
	beq	.L3377
	mov	r0, r5
	bl	make_superblock
	ldr	r3, .L3492+32
	movw	r1, #450
	mov	r2, #0
	strh	r2, [r5, #2]	@ movhi
	strh	r2, [r3, r1]	@ movhi
	add	r3, r3, #452
	strb	r2, [r6, #-2762]
	ldrh	r2, [r5]
	ldr	r1, [r6, #-3028]
	mov	r2, r2, asl #1
	ldrh	r2, [r1, r2]
	strh	r2, [r3]	@ movhi
.L3377:
	ldrh	r3, [r8]
	ldrh	r2, [r7]
	cmp	r2, r3
	beq	.L3384
	ldr	r2, .L3492+36
	ldrh	r1, [r2]
	cmp	r1, r3
	beq	.L3384
	ldrh	r2, [r2, #48]
	cmp	r2, r3
	bne	.L3385
.L3384:
	ldr	r3, .L3492+12
	mvn	r2, #0
	strh	r2, [r3]	@ movhi
.L3385:
	ldr	r7, .L3492
	sub	r10, r7, #2768
.L3418:
	ldrh	r2, [r10]
	movw	r3, #65535
	ldr	r6, .L3492
	cmp	r2, r3
	bne	.L3386
	ldr	r9, .L3492+40
	mov	r3, #0
	str	r3, [r7, #-2192]
	mov	r8, r9
.L3387:
	ldrh	r5, [r9]
	mov	r0, r5
	bl	List_get_gc_head_node
	ldr	r2, .L3492+12
	uxth	r3, r0
	strh	r3, [r2]	@ movhi
	movw	r2, #65535
	cmp	r3, r2
	ldreq	r2, .L3492+40
	moveq	r3, #0
	moveq	r0, #8
	streqh	r3, [r2]	@ movhi
	beq	.L3481
.L3388:
	mov	r0, r3
	str	r3, [sp, #8]
	bl	IsBlkInGcList
	add	r5, r5, #1
	cmp	r0, #0
	ldr	r3, [sp, #8]
	strneh	r5, [r9]	@ movhi
	bne	.L3387
	ldr	r2, .L3492+44
	mov	r1, r3, asl #1
	ldr	r0, [r7, #-3028]
	uxth	r5, r5
	strh	r5, [r9]	@ movhi
	ldrh	lr, [r2]
	ldrh	r2, [r2, #-68]
	ldrh	ip, [r0, r1]
	mul	r2, r2, lr
	add	lr, r2, r2, lsr #31
	cmp	ip, lr, asr #1
	bgt	.L3391
	cmp	ip, #8
	cmphi	r5, #48
	bls	.L3392
	ldr	ip, .L3492+40
	ldrh	ip, [ip, #36]
	cmp	ip, #35
	bhi	.L3392
.L3391:
	mov	ip, #0
	strh	ip, [r8]	@ movhi
.L3392:
	ldrh	r1, [r0, r1]
	movw	r0, #65535
	cmp	r1, r2
	cmpge	r4, r0
	bne	.L3393
	ldrh	r2, [r8]
	cmp	r2, #3
	bhi	.L3393
	ldr	r2, .L3492+12
	mvn	r3, #0
	strh	r3, [r2]	@ movhi
	mov	r2, #0
	ldr	r3, .L3492+40
	strh	r2, [r3]	@ movhi
.L3487:
	ldr	r3, .L3492+20
	ldrh	r0, [r3]
	b	.L3481
.L3393:
	cmp	r1, #0
	bne	.L3394
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r8]
	add	r3, r3, #1
	strh	r3, [r8]	@ movhi
	b	.L3387
.L3394:
	mov	r2, #0
	strb	r2, [r6, #-2760]
	ldr	r2, .L3492+16
	ldrb	r2, [r2, #1888]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3395
	mov	r0, r3
	bl	ftl_get_blk_mode
	strb	r0, [r7, #-2760]
.L3395:
	ldr	r0, .L3492+12
	bl	make_superblock
	ldr	r2, .L3492+12
	ldr	r1, .L3492+48
	mov	r3, #0
	ldr	r0, [r6, #-3028]
	ldrh	r2, [r2]
	strh	r3, [r1]	@ movhi
	mov	r2, r2, asl #1
	ldrh	r2, [r0, r2]
	strb	r3, [r6, #-2762]
	strh	r2, [r1, #2]	@ movhi
	ldr	r2, .L3492+12
	strh	r3, [r2, #2]	@ movhi
.L3386:
	ldr	r3, [sp, #16]
	cmp	r3, #1
	bne	.L3396
	bl	FtlReadRefresh
.L3396:
	mov	r3, #1
	str	r3, [r6, #-2096]
	ldr	r3, .L3492+44
	ldrh	r3, [r3]
	str	r3, [sp, #8]
	ldr	r3, .L3492+16
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3397
	ldrb	r3, [r7, #-2760]	@ zero_extendqisi2
	cmp	r3, #1
	ldreq	r3, .L3492+52
	ldreqh	r3, [r3]
	streq	r3, [sp, #8]
.L3397:
	ldr	r3, .L3492+12
	ldr	r1, [sp, #8]
	ldrh	r3, [r3, #2]
	add	r2, r3, fp
	cmp	r2, r1
	movgt	r2, r1
	rsbgt	r3, r3, r2
	uxthgt	fp, r3
	mov	r3, #0
.L3486:
	str	r3, [sp, #12]
	ldrh	r3, [sp, #12]
	ldr	r5, .L3492
	cmp	r3, fp
	bcs	.L3406
	ldr	r3, .L3492+56
	movw	lr, #65535
	ldrh	r1, [r10, #2]
	mov	ip, #36
	ldr	r6, [r7, #-2172]
	ldrh	r8, [r3]
	ldr	r3, [sp, #12]
	ldr	r0, .L3492+60
	add	r1, r1, r3
	mov	r3, #0
	mov	r5, r3
.L3407:
	uxth	r2, r3
	cmp	r2, r8
	bcs	.L3490
	ldrh	r2, [r0, #2]!
	add	r3, r3, #1
	cmp	r2, lr
	orrne	r2, r1, r2, asl #10
	mlane	r9, ip, r5, r6
	addne	r5, r5, #1
	uxthne	r5, r5
	strne	r2, [r9, #4]
	b	.L3407
.L3490:
	ldr	r0, [r7, #-2172]
	mov	r1, r5
	ldrb	r2, [r7, #-2760]	@ zero_extendqisi2
	mov	r8, #0
	bl	FlashReadPages
.L3402:
	uxth	r3, r8
	cmp	r3, r5
	bcs	.L3491
	mov	r3, #36
	ldr	r2, [r7, #-2172]
	mul	r6, r3, r8
	add	r1, r2, r6
	ldr	r2, [r2, r6]
	ldr	r9, [r1, #12]
	cmn	r2, #1
	beq	.L3438
	ldrh	r1, [r9]
	movw	r2, #61589
	cmp	r1, r2
	bne	.L3438
	add	r1, sp, #32
	mov	r2, #0
	ldr	r0, [r9, #8]
	str	r3, [sp, #20]
	bl	log2phys
	ldr	r2, [r7, #-2172]
	add	r2, r2, r6
	ldr	r0, [r2, #4]
	ldr	r1, [sp, #32]
	ldr	r3, [sp, #20]
	bic	r1, r1, #-2147483648
	cmp	r1, r0
	bne	.L3438
	ldr	r1, .L3492+48
	ldr	r0, .L3492+48
	ldr	r2, [r2, #16]
	ldrh	r1, [r1]
	str	r3, [sp, #24]
	add	r1, r1, #1
	strh	r1, [r0]	@ movhi
	ldr	r0, [r7, #-2188]
	ldr	r1, [r7, #-2068]
	mla	r1, r3, r0, r1
	str	r2, [r1, #16]
	str	r1, [sp, #20]
	bl	Ftl_get_new_temp_ppa
	ldr	r2, [r7, #-2068]
	ldr	r1, [sp, #20]
	ldr	r3, [sp, #24]
	str	r0, [r1, #4]
	ldr	r1, [r7, #-2188]
	mla	r3, r3, r1, r2
	ldr	r2, [r7, #-2172]
	add	r2, r2, r6
	ldr	r1, [r2, #8]
	str	r1, [r3, #8]
	mov	r1, #1
	ldr	r2, [r2, #12]
	str	r2, [r3, #12]
	ldr	r3, [sp, #32]
	str	r3, [r9, #12]
	ldr	r3, .L3492+64
	ldrh	r3, [r3]
	strh	r3, [r9, #2]	@ movhi
	ldr	r3, [r7, #-2812]
	ldr	r0, [r7, #-2172]
	str	r3, [r9, #4]
	add	r0, r0, r6
	ldr	r3, [r7, #-2188]
	add	r3, r3, #1
	str	r3, [r7, #-2188]
	bl	FtlGcBufAlloc
	ldr	r3, .L3492+16
	ldrb	r3, [r3, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3404
	ldrb	r3, [r7, #-2905]	@ zero_extendqisi2
	ldr	r2, [r7, #-2188]
	cmp	r2, r3
	beq	.L3404
	ldr	r3, .L3492+64
	ldrh	r3, [r3, #4]
	cmp	r3, #0
	bne	.L3438
.L3404:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	beq	.L3438
	ldr	r3, .L3492
	mov	r2, #0
	mvn	r1, #0
	sub	r3, r3, #2768
	str	r2, [r3, #672]
	strh	r1, [r3]	@ movhi
	strh	r2, [r3, #2]	@ movhi
	b	.L3487
.L3438:
	add	r8, r8, #1
	b	.L3402
.L3491:
	ldr	r3, [sp, #12]
	add	r3, r3, #1
	b	.L3486
.L3406:
	ldrh	r3, [r10, #2]
	add	fp, fp, r3
	ldr	r3, [sp, #8]
	uxth	fp, fp
	strh	fp, [r10, #2]	@ movhi
	cmp	fp, r3
	bcc	.L3408
	ldr	r3, [r7, #-2188]
	cmp	r3, #0
	beq	.L3409
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	movne	r3, #0
	strne	r3, [r5, #-2096]
	bne	.L3487
.L3409:
	ldr	r3, .L3492+48
	ldrh	r6, [r3]
	cmp	r6, #0
	bne	.L3410
	ldrh	r3, [r10]
	ldr	r2, [r5, #-3028]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3410
	ldr	r8, .L3492+16
.L3411:
	ldr	r3, [r8, #2964]
	cmp	r6, r3
	bcs	.L3416
	mov	r0, r6
	add	r1, sp, #36
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #36]
	cmn	r0, #1
	beq	.L3412
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r10]
	cmp	r3, r0
	bne	.L3412
.L3416:
	ldr	r3, .L3492+16
	ldr	r3, [r3, #2964]
	cmp	r6, r3
	bcc	.L3410
	ldrh	r3, [r10]
	mov	r1, #0
	ldr	r2, [r5, #-3028]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r10]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L3410
.L3412:
	add	r6, r6, #1
	b	.L3411
.L3410:
	mvn	r3, #0
	strh	r3, [r10]	@ movhi
.L3408:
	ldr	r3, .L3492+68
	ldrh	r3, [r3]
	cmp	r3, #2
	ldrls	r3, .L3492+44
	ldrlsh	fp, [r3]
	bls	.L3418
.L3417:
	ldr	r2, .L3492
	mov	r1, #0
	str	r1, [r2, #-2096]
	ldr	r2, .L3492+20
	ldrh	r0, [r2]
	cmp	r0, r1
	addeq	r0, r3, #1
	b	.L3481
.L3422:
	mov	r0, #0
	bx	lr
.L3424:
	mov	r0, ip
	b	.L3481
.L3489:
	sub	r3, r9, #2144
	ldrh	r10, [r3, #-6]
	cmp	r10, r4
	bne	.L3345
	ldrh	r3, [r7, #-4]
	ldr	r2, [r9, #-2200]
	cmp	r3, #24
	movcc	r3, #5120
	movcs	r3, #1024
	cmp	r2, r3
	movls	r4, r10
	bls	.L3345
	ldr	r2, .L3492+20
	mov	r3, #0
	str	r3, [r6, #-2200]
	strh	r3, [r2]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r10
	mov	r4, r0
	sub	r10, r9, #2192
	bne	.L3347
	ldrh	r2, [r7, #-4]
	ldrh	r3, [r10, #-10]
	cmp	r2, r3
	bcs	.L3348
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r4
	beq	.L3357
	ldr	r2, [r9, #-2100]
	ldr	fp, .L3492+16
	cmp	r2, #0
	bne	.L3350
	movw	r1, #2860
	ldrh	r1, [fp, r1]
	cmp	r1, #3
	beq	.L3350
	ldr	r1, [r9, #-2720]
	cmp	r1, #0
	bne	.L3350
	ldr	r1, [fp, #2832]
	cmp	r1, #0
	bne	.L3350
	ldrb	r0, [fp, #1888]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L3351
.L3350:
	ldr	r2, [r6, #-3028]
	mov	r3, r3, asl #1
	movw	r0, #2860
	ldrh	r0, [fp, r0]
	ldrh	r1, [r2, r3]
	movw	r3, #2910
	movw	r2, #2840
	ldrh	r3, [fp, r3]
	ldrh	r2, [fp, r2]
	cmp	r0, #3
	mul	r2, r2, r3
	moveq	r3, r3, lsr #1
	movne	r3, #0
	add	r3, r2, r3
	cmp	r1, r3
	bgt	.L3353
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, [fp, #2964]
	ldr	r2, [r6, #-2856]
	add	r3, r3, r3, asl #1
	cmp	r2, r3, lsr #2
	movls	r3, #160
	uxth	r4, r0
	bls	.L3484
	b	.L3488
.L3351:
	ldr	r2, [r9, #-3028]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #7
	bhi	.L3356
	bl	List_get_gc_head_node
	uxth	r4, r0
.L3488:
	mov	r3, #128
.L3484:
	strh	r3, [r10, #-10]	@ movhi
	movw	r3, #65535
	cmp	r4, r3
	beq	.L3357
	b	.L3347
.L3353:
	mov	r3, #128
	b	.L3485
.L3356:
	mov	r3, #64
	b	.L3485
.L3348:
	mov	r3, #80
.L3485:
	strh	r3, [r10, #-10]	@ movhi
	b	.L3357
.L3347:
	ldr	r0, [r6, #-3092]
	mov	r1, r4, asl #1
	ldr	r3, [r6, #-3028]
	ldrh	r2, [r7, #-4]
	ldrh	r3, [r3, r1]
	ldrh	r1, [r0, r1]
	ldr	r0, .L3492+72
	str	r1, [sp]
	ldrh	r1, [r10, #-12]
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L3357:
	bl	FtlGcReFreshBadBlk
	b	.L3345
.L3481:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3493:
	.align	2
.L3492:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR2-2144
	.word	.LANCHOR2-2768
	.word	.LANCHOR0
	.word	.LANCHOR4+448
	.word	.LC151
	.word	.LANCHOR2-2784
	.word	.LANCHOR4
	.word	.LANCHOR2-2960
	.word	.LANCHOR2-2196
	.word	.LANCHOR0+2908
	.word	.LANCHOR4+450
	.word	.LANCHOR0+2910
	.word	.LANCHOR0+2840
	.word	.LANCHOR2-2754
	.word	.LANCHOR2-2912
	.word	.LANCHOR2-3012
	.word	.LC152
	.fnend
	.size	ftl_do_gc, .-ftl_do_gc
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r3, .L3538
	ldr	r7, .L3538+4
	ldr	r5, [r3, #456]
	ldr	r3, [r7, #-3100]
	cmp	r3, #0
	bne	.L3496
	ldr	r4, .L3538+8
	ldr	r1, [r4, #2956]
	cmp	r1, #0
	beq	.L3496
	ldrb	r6, [r4, #1888]	@ zero_extendqisi2
	mov	r8, #0
	ldr	r0, [r4, #2960]
	mov	r10, #36
	cmp	r6, #0
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	ldr	r9, .L3538+8
	ldrneb	r6, [r5, #8]	@ zero_extendqisi2
	subne	r6, r6, #1
	clzne	r6, r6
	movne	r6, r6, lsr #5
	mov	r2, r6
	bl	FlashProgPages
.L3499:
	ldr	r3, [r4, #2956]
	cmp	r8, r3
	bcs	.L3517
	mul	fp, r10, r8
	ldr	r3, [r9, #2960]
	add	r2, r3, fp
	ldr	r3, [r3, fp]
	cmn	r3, #1
	beq	.L3520
	ldr	r3, [r2, #4]
	cmp	r6, #0
	ldr	r0, [r2, #16]
	add	r1, sp, #4
	mov	r2, #1
	orrne	r3, r3, #-2147483648
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r4, #2960]
	add	r3, r3, fp
	ldr	r3, [r3, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L3503
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r7, #-3028]
	mov	r3, r0, asl #1
	mov	fp, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L3504
	ldr	r0, .L3538+12
	mov	r1, fp
	bl	printk
.L3504:
	mov	r0, fp
	bl	decrement_vpc_count
.L3503:
	add	r8, r8, #1
	b	.L3499
.L3536:
	ldr	r6, .L3538+16
	movw	r5, #16386
.L3516:
	ldrh	r3, [r6]
	cmp	r3, #0
	beq	.L3517
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r5, r5, #1
	bne	.L3516
.L3517:
	mov	r3, #0
	str	r3, [r4, #2956]
	b	.L3496
.L3520:
	ldr	fp, .L3538+8
	mov	r9, #0
.L3500:
	ldr	r3, [r4, #2956]
	cmp	r9, r3
	bcs	.L3536
	mov	r8, #36
	ldr	r3, [fp, #2960]
	mul	r8, r8, r9
	mov	r10, #0
	mvn	r2, #0
	str	r2, [r3, r8]
.L3506:
	ldr	r3, [r4, #2960]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L3537
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r5]
	cmp	r3, r0
	bne	.L3507
	ldr	r1, [r7, #-3028]
	mov	r3, r3, asl #1
	ldrh	r0, [r5, #4]
	ldrh	r2, [r1, r3]
	rsb	r2, r0, r2
	strh	r2, [r1, r3]	@ movhi
	ldr	r3, .L3538+20
	strb	r10, [r5, #6]
	strh	r10, [r5, #4]	@ movhi
	ldrh	r3, [r3]
	strh	r3, [r5, #2]	@ movhi
.L3507:
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	bne	.L3508
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3508:
	ldr	r3, [r7, #-2620]
	add	r3, r3, #1
	str	r3, [r7, #-2620]
	ldr	r3, [r4, #2960]
	add	r3, r3, r8
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlGcMarkBadPhyBlk
	mov	r0, r5
	bl	get_new_active_ppa
	ldr	r3, [r4, #2960]
	mov	r2, r6
	mov	r1, #1
	add	r3, r3, r8
	str	r0, [sp, #4]
	str	r0, [r3, #4]
	mov	r0, r3
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	bl	FlashProgPages
	ldr	r3, [r4, #2960]
	ldr	r3, [r3, r8]
	cmn	r3, #1
	ldreq	r2, .L3538+4
	moveq	r3, #1
	streq	r3, [r2, #-3100]
	ldr	r3, [r7, #-3100]
	cmp	r3, #0
	beq	.L3506
	b	.L3496
.L3537:
	ldr	r3, [r2, #4]
	cmp	r6, #0
	ldr	r0, [r2, #16]
	add	r1, sp, #4
	mov	r2, #1
	orrne	r3, r3, #-2147483648
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r4, #2960]
	add	r8, r3, r8
	ldr	r3, [r8, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L3513
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r7, #-3028]
	mov	r3, r0, asl #1
	mov	r8, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L3514
	ldr	r0, .L3538+12
	mov	r1, r8
	bl	printk
.L3514:
	mov	r0, r8
	bl	decrement_vpc_count
.L3513:
	add	r9, r9, #1
	b	.L3500
.L3496:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3539:
	.align	2
.L3538:
	.word	.LANCHOR4
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC153
	.word	.LANCHOR2-2142
	.word	.LANCHOR0+2908
	.fnend
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	2
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L3546
	ldr	r3, [r3, #-3100]
	cmp	r3, #0
	bne	.L3541
	ldr	r3, .L3546+4
	ldr	r3, [r3, #504]
	cmn	r3, #1
	beq	.L3541
	bl	FtlCacheWriteBack
	bl	l2p_flush
	mov	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3541:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L3547:
	.align	2
.L3546:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.fnend
	.size	FtlSysFlush, .-FtlSysFlush
	.align	2
	.global	FtlDeInit
	.type	FtlDeInit, %function
FtlDeInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L3551
	ldr	r3, [r3, #504]
	cmp	r3, #1
	bne	.L3549
	bl	FtlSysFlush
.L3549:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L3552:
	.align	2
.L3551:
	.word	.LANCHOR1
	.fnend
	.size	FtlDeInit, .-FtlDeInit
	.align	2
	.global	ftl_deinit
	.type	ftl_deinit, %function
ftl_deinit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	ftl_flash_de_init
	bl	FtlDeInit
	ldmfd	sp!, {r3, lr}
	b	ftl_flash_de_init
	.fnend
	.size	ftl_deinit, .-ftl_deinit
	.align	2
	.global	rk_ftl_de_init
	.type	rk_ftl_de_init, %function
rk_ftl_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	mov	r1, #0
	ldr	r0, .L3557
	bl	printk
	ldmfd	sp!, {r3, lr}
	b	ftl_deinit
.L3558:
	.align	2
.L3557:
	.word	.LC154
	.fnend
	.size	rk_ftl_de_init, .-rk_ftl_de_init
	.align	2
	.global	ftl_cache_flush
	.type	ftl_cache_flush, %function
ftl_cache_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	FtlCacheWriteBack
	.fnend
	.size	ftl_cache_flush, .-ftl_cache_flush
	.align	2
	.global	rk_ftl_cache_write_back
	.type	rk_ftl_cache_write_back, %function
rk_ftl_cache_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	FtlCacheWriteBack
	.fnend
	.size	rk_ftl_cache_write_back, .-rk_ftl_cache_write_back
	.align	2
	.global	ftl_discard
	.type	ftl_discard, %function
ftl_discard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	add	r2, r0, r1
	ldr	r4, .L3580
	mov	r6, r0
	mov	r5, r1
	ldr	r3, [r4, #1892]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L3562
	ldr	r3, .L3580+4
	ldr	r0, [r3, #504]
	cmn	r0, #1
	beq	.L3562
	cmp	r1, #31
	bls	.L3571
	ldr	r7, .L3580+8
	ldr	r3, [r7, #-3100]
	cmp	r3, #0
	bne	.L3571
	bl	FtlCacheWriteBack
	movw	r3, #2914
	ldrh	r4, [r4, r3]
	mov	r0, r6
	mov	r1, r4
	bl	__aeabi_uidiv
	smulbb	r3, r0, r4
	mov	r8, r0
	rsb	r6, r3, r6
	uxth	r6, r6
	cmp	r6, #0
	beq	.L3563
	rsb	r4, r6, r4
	add	r8, r0, #1
	cmp	r4, r5
	movcs	r4, r5
	uxth	r4, r4
	rsb	r5, r4, r5
.L3563:
	ldr	r4, .L3580+12
	mvn	r3, #0
	ldr	r9, .L3580+16
	str	r3, [sp, #4]
	mov	r6, r4
.L3564:
	ldrh	r3, [r4]
	cmp	r5, r3
	bcc	.L3579
	mov	r0, r8
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp]
	cmn	r3, #1
	beq	.L3565
	ldr	r3, [r9, #460]
	add	r1, sp, #4
	mov	r2, #1
	mov	r0, r8
	add	r3, r3, #1
	str	r3, [r9, #460]
	ldr	r3, [r7, #-2844]
	add	r3, r3, #1
	str	r3, [r7, #-2844]
	bl	log2phys
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L3565:
	ldrh	r3, [r6]
	add	r8, r8, #1
	rsb	r5, r3, r5
	b	.L3564
.L3579:
	ldr	r3, .L3580+16
	ldr	r2, [r3, #460]
	cmp	r2, #32
	bls	.L3571
	mov	r2, #0
	str	r2, [r3, #460]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L3571:
	mov	r0, #0
.L3562:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3581:
	.align	2
.L3580:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LANCHOR0+2914
	.word	.LANCHOR4
	.fnend
	.size	ftl_discard, .-ftl_discard
	.align	2
	.global	FtlDiscard
	.type	FtlDiscard, %function
FtlDiscard:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_discard
	.fnend
	.size	FtlDiscard, .-FtlDiscard
	.align	2
	.global	ftl_read
	.type	ftl_read, %function
ftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r1
	ldr	r1, .L3630
	.pad #84
	sub	sp, sp, #84
	ldr	r4, [r1, #504]
	cmn	r4, #1
	moveq	r0, r4
	beq	.L3584
	cmp	r0, #16
	mov	r8, r3
	str	r2, [sp, #36]
	bne	.L3585
	mov	r1, r2
	add	r0, r5, #256
	mov	r2, r3
	bl	FtlVendorPartRead
	b	.L3584
.L3585:
	ldr	r3, [sp, #36]
	add	r3, r5, r3
	str	r3, [sp, #40]
	ldr	r3, .L3630+4
	ldr	r1, [sp, #40]
	ldr	r2, [r3, #1892]
	cmp	r1, r2
	mvnhi	r0, #0
	bhi	.L3584
	movw	r2, #2914
	mov	r0, r5
	ldrh	r4, [r3, r2]
	mov	r1, r4
	bl	__aeabi_uidiv
	mov	r1, r4
	ldr	r3, [sp, #40]
	mov	r10, r0
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	rsb	r3, r10, #1
	add	r3, r3, r0
	str	r3, [sp, #28]
	ldr	r3, .L3630+8
	ldr	r1, [sp, #36]
	str	r0, [sp, #32]
	mov	r0, r10
	ldr	r2, [r3, #-2820]
	add	r2, r1, r2
	ldr	r1, [sp, #28]
	str	r2, [r3, #-2820]
	ldr	r2, [r3, #-2848]
	add	r2, r1, r2
	ldr	r1, [sp, #32]
	str	r2, [r3, #-2848]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L3586
	bl	FtlCacheWriteBack
.L3586:
	ldr	r4, .L3630+8
	mov	r9, #0
	mov	r6, r10
	mov	r7, r9
	str	r9, [sp, #48]
	str	r9, [sp, #52]
.L3587:
	ldr	r3, [sp, #28]
	cmp	r3, #0
	beq	.L3629
	mov	r0, r6
	add	r1, sp, #76
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #76]
	cmn	r3, #1
	bne	.L3625
	mov	fp, #0
.L3588:
	ldr	r3, .L3630+12
	ldrh	r0, [r3]
	cmp	fp, r0
	bcs	.L3592
	mla	r0, r0, r6, fp
	ldr	r2, [sp, #40]
	cmp	r0, r5
	movcs	r3, #1
	movcc	r3, #0
	cmp	r0, r2
	movcs	r3, #0
	cmp	r3, #0
	beq	.L3590
	rsb	r0, r5, r0
	mov	r1, #512
	add	r0, r8, r0, asl #9
	bl	__memzero
.L3590:
	add	fp, fp, #1
	b	.L3588
.L3625:
	ldr	r2, [r4, #-2072]
	mov	fp, #36
	cmp	r6, r10
	mla	fp, fp, r7, r2
	str	r3, [fp, #4]
	bne	.L3593
	ldr	r3, [r4, #-2044]
	mov	r0, r5
	str	r3, [fp, #8]
	ldr	r3, .L3630+12
	ldrh	ip, [r3]
	mov	r1, ip
	str	ip, [sp, #44]
	bl	__aeabi_uidivmod
	ldr	ip, [sp, #44]
	ldr	r2, [sp, #36]
	rsb	r3, r1, ip
	str	r1, [sp, #56]
	cmp	r3, r2
	movcs	r3, r2
	cmp	r3, ip
	str	r3, [sp, #48]
	streq	r8, [fp, #8]
	b	.L3594
.L3593:
	ldr	r3, [sp, #32]
	cmp	r6, r3
	bne	.L3595
	ldr	r3, [r4, #-2040]
	ldr	r1, [sp, #40]
	str	r3, [fp, #8]
	ldr	r3, .L3630+12
	ldrh	r2, [r3]
	mul	r3, r2, r6
	rsb	r9, r3, r1
	cmp	r9, r2
	bne	.L3594
	b	.L3627
.L3595:
	ldr	r3, .L3630+12
	ldrh	r3, [r3]
	mul	r3, r3, r6
.L3627:
	rsb	r3, r5, r3
	add	r3, r8, r3, asl #9
	str	r3, [fp, #8]
.L3594:
	ldr	r3, .L3630+16
	ldr	r2, [r4, #-2032]
	str	r6, [fp, #16]
	ldrh	r3, [r3]
	mul	r3, r3, r7
	add	r7, r7, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [fp, #12]
.L3592:
	ldr	r3, [sp, #28]
	add	r6, r6, #1
	subs	r3, r3, #1
	str	r3, [sp, #28]
	beq	.L3596
	ldr	r3, .L3630+20
	ldrh	r3, [r3]
	cmp	r7, r3, asl #3
	bne	.L3587
.L3596:
	cmp	r7, #0
	beq	.L3587
	ldr	r0, [r4, #-2072]
	mov	r1, r7
	mov	r2, #0
	bl	FlashReadPages
	ldr	r3, [sp, #56]
	mov	r3, r3, asl #9
	str	r3, [sp, #60]
	ldr	r3, [sp, #48]
	mov	r3, r3, asl #9
	str	r3, [sp, #64]
	mov	r3, r9, asl #9
	str	r3, [sp, #68]
	mov	r3, #0
	str	r3, [sp, #44]
.L3603:
	ldr	r3, [sp, #44]
	mov	ip, #36
	mul	fp, ip, r3
	ldr	r3, [r4, #-2072]
	add	r3, r3, fp
	ldr	r2, [r3, #16]
	cmp	r2, r10
	bne	.L3598
	ldr	r1, [r3, #8]
	ldr	r3, [r4, #-2044]
	cmp	r1, r3
	bne	.L3599
	ldr	r3, [sp, #60]
	mov	r0, r8
	ldr	r2, [sp, #64]
	add	r1, r1, r3
	b	.L3628
.L3598:
	ldr	r1, [sp, #32]
	cmp	r2, r1
	bne	.L3599
	ldr	r1, [r3, #8]
	ldr	r3, [r4, #-2040]
	cmp	r1, r3
	bne	.L3599
	ldr	r3, .L3630+12
	ldr	r2, [sp, #68]
	ldrh	r0, [r3]
	ldr	r3, [sp, #32]
	mul	r0, r0, r3
	rsb	r0, r5, r0
	add	r0, r8, r0, asl #9
.L3628:
	bl	ftl_memcpy
.L3599:
	ldr	r2, [r4, #-2072]
	add	r3, r2, fp
	ldr	r1, [r2, fp]
	cmn	r1, #1
	streq	r1, [sp, #52]
	ldreq	r2, [r4, #-2644]
	addeq	r2, r2, #1
	streq	r2, [r4, #-2644]
	ldr	r2, [r3, #12]
	ldr	r1, [r3, #16]
	ldr	r2, [r2, #8]
	cmp	r1, r2
	beq	.L3601
	ldr	r2, [r4, #-2644]
	ldr	r0, .L3630+24
	add	r2, r2, #1
	str	r2, [r4, #-2644]
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
.L3601:
	ldr	r3, [r4, #-2072]
	add	r2, r3, fp
	ldr	r3, [r3, fp]
	cmp	r3, #256
	bne	.L3602
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L3602:
	ldr	r3, [sp, #44]
	add	r3, r3, #1
	str	r3, [sp, #44]
	cmp	r3, r7
	bne	.L3603
	mov	r7, #0
	b	.L3587
.L3629:
	ldr	r3, .L3630+28
	ldrh	r3, [r3, #-14]
	cmp	r3, #0
	beq	.L3605
	ldr	r0, [sp, #28]
	mov	r1, #1
	bl	ftl_do_gc
.L3605:
	ldr	r0, [sp, #52]
.L3584:
	add	sp, sp, #84
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3631:
	.align	2
.L3630:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2914
	.word	.LANCHOR0+2920
	.word	.LANCHOR0+2840
	.word	.LC143
	.word	.LANCHOR2-2128
	.fnend
	.size	ftl_read, .-ftl_read
	.align	2
	.global	ftl_vendor_read
	.type	ftl_vendor_read, %function
ftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!
	.save {lr}
	mov	ip, r1
	mov	lr, r0
	mov	r3, r2
	mov	r1, lr
	mov	r0, #16
	mov	r2, ip
	ldr	lr, [sp], #4
	b	ftl_read
	.fnend
	.size	ftl_vendor_read, .-ftl_vendor_read
	.align	2
	.global	FlashBootVendorRead
	.type	FlashBootVendorRead, %function
FlashBootVendorRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r2
	mov	r6, r0
	mov	r5, r1
	bl	rknand_device_lock
	ldr	r3, .L3638
	ldr	r3, [r3, #504]
	cmp	r3, #1
	mvnne	r4, #0
	bne	.L3635
	mov	r2, r4
	mov	r0, r6
	mov	r1, r5
	bl	ftl_vendor_read
	mov	r4, r0
.L3635:
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L3639:
	.align	2
.L3638:
	.word	.LANCHOR1
	.fnend
	.size	FlashBootVendorRead, .-FlashBootVendorRead
	.align	2
	.global	ftl_sys_read
	.type	ftl_sys_read, %function
ftl_sys_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	ip, r1
	mov	r3, r2
	add	r1, r0, #256
	mov	r2, ip
	mov	r0, #16
	b	ftl_read
	.fnend
	.size	ftl_sys_read, .-ftl_sys_read
	.align	2
	.global	StorageSysDataLoad
	.type	StorageSysDataLoad, %function
StorageSysDataLoad:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	mov	r4, r1
	mov	r0, r1
	mov	r1, #512
	bl	__memzero
	bl	rknand_device_lock
	mov	r2, r4
	mov	r1, #1
	mov	r0, r5
	bl	ftl_sys_read
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
	.fnend
	.size	StorageSysDataLoad, .-StorageSysDataLoad
	.align	2
	.global	FtlRead
	.type	FtlRead, %function
FtlRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_read
	.fnend
	.size	FtlRead, .-FtlRead
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mvn	r3, #0
	ldr	r2, .L3661
	ldr	r7, .L3661+4
	ldr	r5, .L3661+8
	ldr	r1, .L3661+12
	ldr	r0, .L3661+16
	str	r3, [r7, #504]
	mov	r3, #0
	ldr	r6, .L3661+20
	str	r3, [r2, #464]
	str	r3, [r5, #-3100]
	bl	printk
	ldr	r0, .L3661+24
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldr	r3, [r6, #2844]
	uxth	r0, r3
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	ldrne	r0, .L3661+28
	bne	.L3660
	bl	FtlSysBlkInit
	subs	r4, r0, #0
	beq	.L3647
	ldr	r0, .L3661+32
.L3660:
	ldr	r1, .L3661+36
	bl	printk
	b	.L3646
.L3647:
	mov	r1, #1
	str	r1, [r7, #504]
	bl	ftl_do_gc
	sub	r3, r5, #3008
	ldrh	r7, [r3, #-4]
	cmp	r7, #15
	bhi	.L3648
	ldr	r6, .L3661+40
	sub	r5, r5, #2768
	sub	r8, r3, #4
.L3651:
	ldrh	r3, [r5]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L3649
	ldrh	r2, [r6]
	cmp	r2, r3
	bne	.L3649
	and	r0, r4, #7
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L3649:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	mov	r0, #0
	mov	r1, #1
	bl	ftl_do_gc
	ldrh	r2, [r8]
	add	r3, r7, #2
	cmp	r2, r3
	bhi	.L3646
	add	r4, r4, #1
	cmp	r4, #1024
	bne	.L3651
	b	.L3646
.L3648:
	ldrb	r3, [r6, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3646
	mov	r4, #128
.L3653:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	bne	.L3653
.L3646:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3662:
	.align	2
.L3661:
	.word	.LANCHOR4
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LC77
	.word	.LC76
	.word	.LANCHOR0
	.word	.LANCHOR0+1860
	.word	.LC155
	.word	.LC156
	.word	.LANCHOR3+108
	.word	.LANCHOR2-2150
	.fnend
	.size	FtlInit, .-FtlInit
	.align	2
	.global	rk_ftl_init
	.type	rk_ftl_init, %function
rk_ftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r0, #2048
	bl	ftl_malloc
	ldr	r6, .L3668
	ldr	r4, .L3668+4
	mov	r5, #0
	mov	r1, r6
	str	r5, [r1, #472]!
	str	r0, [r6, #468]
	mov	r0, r4
	str	r5, [r0, #148]!
	bl	rknand_get_reg_addr
	ldr	r3, [r4, #148]
	cmp	r3, r5
	beq	.L3666
	bl	rk_nandc_irq_init
	mov	r1, r5
	mov	r2, r5
	mov	r3, #2048
	ldr	r0, [r6, #468]
	bl	FlashSramLoadStore
	bl	rknand_flash_cs_init
	ldr	r0, [r4, #148]
	bl	FlashInit
	subs	r4, r0, #0
	bne	.L3665
	bl	FtlInit
.L3665:
	mov	r1, r4
	ldr	r0, .L3668+8
	bl	printk
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L3666:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L3669:
	.align	2
.L3668:
	.word	.LANCHOR4
	.word	.LANCHOR2
	.word	.LC157
	.fnend
	.size	rk_ftl_init, .-rk_ftl_init
	.align	2
	.global	ftl_fix_nand_power_lost_error
	.type	ftl_fix_nand_power_lost_error, %function
ftl_fix_nand_power_lost_error:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldr	r5, .L3686
	ldrb	r3, [r5, #1888]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3670
	ldr	r8, .L3686+4
	ldr	r4, .L3686+8
	ldr	r0, .L3686+12
	ldrh	r7, [r8]
	sub	r10, r4, #3008
	ldr	r3, [r4, #-3028]
	sub	r9, r4, #2960
	mov	r6, r7, asl #1
	mov	r1, r7
	ldrh	r2, [r3, r6]
	bl	printk
	ldrh	r0, [r10]
	bl	FtlGcRefreshOpenBlock
	ldrh	r0, [r9]
	bl	FtlGcRefreshOpenBlock
	mov	r0, r10
	bl	allocate_new_data_superblock
	mov	r0, r9
	bl	allocate_new_data_superblock
	movw	r9, #4097
.L3672:
	subs	r9, r9, #1
	beq	.L3676
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	ldr	r3, [r4, #-3028]
	ldrh	r3, [r3, r6]
	cmp	r3, #0
	bne	.L3672
.L3676:
	ldr	r3, [r4, #-3028]
	mov	r1, r7
	ldr	r0, .L3686+12
	ldrh	r2, [r3, r6]
	bl	printk
	ldr	r3, [r4, #-3028]
	ldrh	r10, [r3, r6]
	cmp	r10, #0
	bne	.L3674
	add	r0, sp, #48
	mov	fp, #36
	strh	r7, [r0, #-48]!	@ movhi
	bl	make_superblock
	movw	r3, #2840
	ldrh	lr, [r5, r3]
	mov	r5, r10
	ldr	r3, .L3686+8
	ldr	r9, [r3, #-3096]
	mov	r3, r10
	mov	ip, r3
	movw	r10, #65535
	add	r0, sp, #14
.L3677:
	uxth	r2, r3
	cmp	r2, lr
	bcs	.L3685
	ldrh	r2, [r0, #2]!
	add	r3, r3, #1
	cmp	r2, r10
	movne	r2, r2, asl #10
	mlane	r1, fp, r5, r9
	addne	r5, r5, #1
	uxthne	r5, r5
	stmneib	r1, {r2, ip}
	strne	ip, [r1, #12]
	b	.L3677
.L3685:
	ldr	r3, [r4, #-3028]
	mov	r1, r7
	ldr	r0, .L3686+16
	ldrh	r2, [r3, r6]
	bl	printk
	mov	r1, #0
	mov	r2, r5
	ldr	r0, [r4, #-3096]
	bl	FlashEraseBlocks
	ldr	r0, [r4, #-3096]
	mov	r1, #1
	mov	r2, r5
	bl	FlashEraseBlocks
.L3674:
	mvn	r3, #0
	strh	r3, [r8]	@ movhi
.L3670:
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3687:
	.align	2
.L3686:
	.word	.LANCHOR0
	.word	.LANCHOR4+312
	.word	.LANCHOR2
	.word	.LC158
	.word	.LC159
	.fnend
	.size	ftl_fix_nand_power_lost_error, .-ftl_fix_nand_power_lost_error
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_do_gc
	.fnend
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	2
	.global	ftl_write
	.type	ftl_write, %function
ftl_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #100
	sub	sp, sp, #100
	ldr	r10, .L3762
	str	r3, [sp, #8]
	ldr	r3, [r10, #-3100]
	cmp	r3, #0
	bne	.L3730
	mov	r8, r2
	ldr	r2, .L3762+4
	ldr	r2, [r2, #504]
	cmn	r2, #1
	moveq	r0, r3
	beq	.L3690
	cmp	r0, #16
	mov	r7, r1
	bne	.L3691
	add	r0, r1, #256
	ldr	r2, [sp, #8]
	mov	r1, r8
	bl	FtlVendorPartWrite
	b	.L3690
.L3691:
	ldr	fp, .L3762+8
	add	r5, r1, r8
	ldr	r3, [fp, #1892]
	cmp	r5, r3
	mvnhi	r0, #0
	bhi	.L3690
	ldr	r6, .L3762+12
	mov	r3, #2048
	mov	r0, r1
	str	r3, [r6, #476]
	movw	r3, #2914
	ldrh	r4, [fp, r3]
	mov	r1, r4
	bl	__aeabi_uidiv
	mov	r1, r4
	str	r0, [sp, #12]
	sub	r0, r5, #1
	bl	__aeabi_uidiv
	cmp	r8, r4, asl #1
	ldr	r2, [sp, #12]
	str	r0, [sp, #28]
	rsb	r5, r2, r0
	add	r3, r5, #1
	str	r3, [sp, #4]
	ldr	r2, [sp, #4]
	ldr	r3, [r10, #-2840]
	add	r3, r2, r3
	ldr	r2, [fp, #2956]
	str	r3, [r10, #-2840]
	ldr	r3, [r10, #-2824]
	add	r3, r8, r3
	str	r3, [r10, #-2824]
	movcs	r3, #1
	movcc	r3, #0
	cmp	r2, #0
	str	r3, [sp, #24]
	beq	.L3693
	mov	r3, #36
	ldr	r9, [fp, #2960]
	mul	r3, r3, r2
	ldr	r2, [sp, #12]
	sub	r3, r3, #36
	add	r9, r9, r3
	ldr	r3, [r9, #16]
	cmp	r2, r3
	bne	.L3694
	ldr	r3, [r10, #-2836]
	mov	r1, r4
	mov	r0, r7
	add	r3, r3, #1
	str	r3, [r10, #-2836]
	ldr	r3, [r6, #480]
	add	r3, r3, #1
	str	r3, [r6, #480]
	bl	__aeabi_uidivmod
	ldr	r0, [r9, #8]
	rsb	r4, r1, r4
	add	r0, r0, r1, asl #9
	cmp	r4, r8
	ldr	r1, [sp, #8]
	movcs	r4, r8
	mov	r10, r4, asl #9
	mov	r2, r10
	bl	ftl_memcpy
	cmp	r5, #0
	bne	.L3695
	ldr	r3, [r6, #480]
	cmp	r3, #2
	ble	.L3730
.L3695:
	ldr	r3, [sp, #8]
	rsb	r8, r4, r8
	add	r7, r7, r4
	str	r5, [sp, #4]
	add	r3, r3, r10
	str	r3, [sp, #8]
	ldr	r3, [sp, #12]
	add	r3, r3, #1
	str	r3, [sp, #12]
.L3694:
	mov	r3, #0
	str	r3, [r6, #480]
.L3693:
	ldr	r0, [sp, #12]
	ldr	r1, [sp, #28]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L3696
	bl	FtlCacheWriteBack
.L3696:
	ldr	r5, .L3762+16
	mov	r3, #0
	ldr	r4, .L3762+8
	str	r3, [sp, #16]
	str	r5, [r6, #456]
	ldr	r6, [sp, #12]
	str	r3, [sp, #32]
.L3697:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	beq	.L3761
	ldrh	r2, [r5, #4]
	ldr	r10, .L3762
	cmp	r2, #0
	bne	.L3698
	sub	r3, r10, #3008
	ldr	r9, .L3762+4
	cmp	r5, r3
	bne	.L3699
	sub	r0, r10, #2960
	ldrh	r5, [r0, #4]
	cmp	r5, #0
	bne	.L3700
	bl	allocate_new_data_superblock
	str	r5, [r9, #3292]
.L3700:
	ldr	r0, .L3762+16
	bl	allocate_new_data_superblock
	ldr	r3, [r9, #3292]
	cmp	r3, #0
	ldrne	r5, .L3762+20
	bne	.L3701
.L3702:
	ldr	r5, .L3762+16
	b	.L3701
.L3699:
	ldrh	r3, [r3, #4]
	str	r2, [r9, #3292]
	cmp	r3, #0
	bne	.L3702
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3701:
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	bne	.L3703
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3703:
	ldr	r3, .L3762+12
	str	r5, [r3, #456]
.L3698:
	ldr	r2, [r10, #-2076]
	ldr	r1, [r4, #2956]
	ldrh	r3, [r5, #4]
	rsb	r2, r1, r2
	cmp	r3, r2
	movcs	r3, r2
	ldr	r2, [sp, #4]
	cmp	r3, r2
	movcs	r3, r2
	str	r3, [sp, #44]
	mov	r3, #0
.L3759:
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #44]
	cmp	r3, r2
	beq	.L3705
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L3705
	ldr	r3, [sp, #28]
	ldr	r2, [sp, #20]
	rsb	ip, r3, r6
	ldr	r3, [sp, #24]
	clz	ip, ip
	mov	ip, ip, lsr #5
	and	r3, ip, r3
	cmp	r2, #0
	moveq	r3, #0
	andne	r3, r3, #1
	cmp	r3, #0
	beq	.L3706
	ldr	r3, .L3762+24
	ldrh	r2, [r3]
	add	r3, r8, r7
	mls	r3, r2, r6, r3
	cmp	r3, r2
	bne	.L3705
.L3706:
	add	r1, sp, #56
	mov	r2, #0
	mov	r0, r6
	str	ip, [sp, #52]
	bl	log2phys
	mov	r0, r5
	bl	get_new_active_ppa
	ldr	fp, .L3762+28
	ldr	r10, [r4, #2956]
	mov	r3, #36
	ldr	r1, [r4, #2960]
	ldrh	r2, [fp]
	mla	r1, r3, r10, r1
	mul	lr, r10, r2
	str	r6, [r1, #16]
	str	r3, [sp, #48]
	bic	r3, lr, #3
	ldrh	lr, [fp, #-2]
	str	r0, [r1, #4]
	ldr	r0, .L3762
	str	r3, [sp, #36]
	mul	lr, r10, lr
	ldr	r3, [r0, #-2028]
	ldr	ip, [sp, #36]
	ldr	r0, [r0, #-2048]
	add	r9, r3, ip
	bic	lr, lr, #3
	add	lr, r0, lr
	str	r9, [r1, #12]
	str	lr, [r1, #8]
	mov	r0, r9
	mov	r1, #0
	str	r3, [sp, #40]
	bl	ftl_memset
	ldr	r3, [sp, #12]
	ldr	ip, [sp, #52]
	rsb	r10, r3, r6
	clz	r10, r10
	mov	r10, r10, lsr #5
	orrs	r3, r10, ip
	ldr	r3, [sp, #48]
	beq	.L3707
	cmp	r10, #0
	beq	.L3708
	ldrh	fp, [fp, #-6]
	mov	r0, r7
	mov	r1, fp
	bl	__aeabi_uidivmod
	rsb	fp, r1, fp
	mov	r3, r1
	cmp	fp, r8
	str	r1, [sp, #32]
	movcc	r3, fp
	movcs	r3, r8
	str	r3, [sp, #16]
	b	.L3709
.L3708:
	cmp	ip, #0
	beq	.L3709
	ldr	r2, .L3762+24
	add	r3, r8, r7
	str	r10, [sp, #32]
	ldrh	r2, [r2]
	smulbb	r2, r2, r6
	rsb	r3, r2, r3
	uxth	r3, r3
	str	r3, [sp, #16]
.L3709:
	ldr	r3, .L3762+24
	ldr	r2, [sp, #16]
	ldrh	r3, [r3]
	cmp	r2, r3
	bne	.L3710
	cmp	r10, #0
	ldr	r0, .L3762+8
	moveq	r3, r2
	ldr	r2, .L3762+8
	muleq	r1, r6, r3
	ldreq	r3, [sp, #8]
	ldr	r2, [r2, #2956]
	ldr	r0, [r0, #2960]
	rsbeq	r1, r7, r1
	ldrne	r1, [sp, #8]
	addeq	r1, r3, r1, asl #9
	ldr	r3, [sp, #24]
	cmp	r3, #0
	mov	r3, #36
	mla	r3, r3, r2, r0
	strne	r1, [r3, #8]
	bne	.L3713
	ldr	r0, [r3, #8]
	ldr	r3, .L3762+32
	ldrh	r2, [r3]
	b	.L3757
.L3710:
	ldr	r2, [sp, #56]
	mov	r3, #36
	cmn	r2, #1
	beq	.L3714
	ldr	r1, [r4, #2960]
	add	r0, sp, #60
	str	r2, [sp, #64]
	ldr	r2, [r4, #2956]
	str	r6, [sp, #76]
	mla	r3, r3, r2, r1
	mov	r1, #1
	ldr	r2, [r3, #8]
	ldr	r3, [r3, #12]
	str	r2, [sp, #68]
	mov	r2, #0
	str	r3, [sp, #72]
	bl	FlashReadPages
	ldr	r3, [sp, #60]
	cmn	r3, #1
	ldr	r3, .L3762
	ldreq	r2, [r3, #-2644]
	addeq	r2, r2, #1
	streq	r2, [r3, #-2644]
	beq	.L3717
	ldr	r2, [r9, #8]
	cmp	r2, r6
	beq	.L3717
	ldr	r2, [r3, #-2644]
	ldr	r0, .L3762+36
	add	r2, r2, #1
	str	r2, [r3, #-2644]
	mov	r2, r6
	ldr	r1, [r9, #8]
	bl	printk
	b	.L3717
.L3714:
	ldr	r2, [r4, #2956]
	ldr	r1, [r4, #2960]
	mla	r3, r3, r2, r1
	mov	r1, #0
	ldr	r0, [r3, #8]
	ldr	r3, .L3762+32
	ldrh	r2, [r3]
	bl	ftl_memset
.L3717:
	cmp	r10, #0
	mov	r3, #36
	beq	.L3718
	ldr	r1, [r4, #2960]
	ldr	r2, [r4, #2956]
	mla	r3, r3, r2, r1
	ldr	r1, [sp, #8]
	ldr	r0, [r3, #8]
	ldr	r3, [sp, #32]
	add	r0, r0, r3, asl #9
	b	.L3760
.L3718:
	ldr	r1, [r4, #2956]
	ldr	r2, [r4, #2960]
	mla	r3, r3, r1, r2
	ldr	r2, .L3762+24
	ldrh	r1, [r2]
	ldr	r0, [r3, #8]
	mul	r1, r1, r6
	ldr	r3, [sp, #8]
	rsb	r1, r7, r1
	add	r1, r3, r1, asl #9
.L3760:
	ldr	r3, [sp, #16]
	mov	r2, r3, asl #9
	b	.L3757
.L3707:
	ldr	r2, [sp, #24]
	cmp	r2, #0
	ldr	r2, [r4, #2956]
	beq	.L3719
	ldr	r1, [r4, #2960]
	mla	r3, r3, r2, r1
	ldr	r2, .L3762+24
	ldrh	fp, [r2]
	ldr	r2, [sp, #8]
	mul	fp, fp, r6
	rsb	fp, r7, fp
	add	fp, r2, fp, asl #9
	str	fp, [r3, #8]
	b	.L3713
.L3719:
	ldr	r0, [r4, #2960]
	mla	r3, r3, r2, r0
	ldr	r2, .L3762+24
	ldrh	r1, [r2]
	ldrh	r2, [fp, #-2]
	ldr	r0, [r3, #8]
	mul	r1, r1, r6
	ldr	r3, [sp, #8]
	rsb	r1, r7, r1
	add	r1, r3, r1, asl #9
.L3757:
	bl	ftl_memcpy
.L3713:
	ldr	r3, .L3762+40
	ldr	r2, [sp, #40]
	ldr	r1, [sp, #36]
	strh	r3, [r2, r1]	@ movhi
	ldr	r2, .L3762
	str	r6, [r9, #8]
	add	r6, r6, #1
	ldr	r3, [r2, #-2812]
	str	r3, [r9, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, #0
	str	r3, [r2, #-2812]
	ldr	r3, [sp, #56]
	str	r3, [r9, #12]
	ldrh	r3, [r5]
	strh	r3, [r9, #2]	@ movhi
	ldr	r3, [r4, #2956]
	add	r3, r3, #1
	str	r3, [r4, #2956]
	ldr	r3, [sp, #20]
	add	r3, r3, #1
	b	.L3759
.L3705:
	ldr	r3, [sp, #4]
	ldr	r2, [sp, #20]
	ldr	r1, [sp, #24]
	rsb	r3, r2, r3
	ldr	r2, .L3762
	str	r3, [sp, #4]
	ldr	r3, [r4, #2956]
	ldr	r2, [r2, #-2076]
	cmp	r3, r2
	orrcs	r1, r1, #1
	cmp	r1, #0
	bne	.L3723
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L3723
.L3725:
	mov	r3, #0
	str	r3, [sp, #24]
	b	.L3697
.L3723:
	bl	FtlCacheWriteBack
	ldr	r2, .L3762+8
	mov	r3, #0
	str	r3, [r2, #2956]
	ldr	r3, [sp, #4]
	cmp	r3, #1
	bhi	.L3697
	b	.L3725
.L3761:
	mov	r0, r3
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #28]
	rsb	r1, r2, r3
	bl	ftl_do_gc
	ldr	r3, .L3762+16
	ldrh	r3, [r3, #-4]
	cmp	r3, #5
	bls	.L3735
	cmp	r3, #31
	bhi	.L3730
	ldr	r3, .L3762+8
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3730
.L3735:
	ldr	r5, .L3762+44
	ldr	r4, [sp, #4]
	ldr	r6, .L3762+48
	add	r7, r5, #620
.L3748:
	ldrh	r2, [r5]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3729
	ldrh	r3, [r6]
	cmp	r3, r2
	bne	.L3729
	ldrh	r2, [r7]
	cmp	r2, r3
	bne	.L3729
	and	r0, r4, #7
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L3729:
	ldr	r3, .L3762+52
	mov	r0, #1
	mov	r1, r0
	mov	r2, #128
	strh	r2, [r3]	@ movhi
	strh	r2, [r3, #-2]	@ movhi
	bl	ftl_do_gc
	mov	r0, #0
	mov	r1, #1
	bl	ftl_do_gc
	ldr	r3, .L3762+56
	ldr	r3, [r3, #-3100]
	cmp	r3, #0
	bne	.L3730
	ldr	r3, .L3762+60
	ldrh	r3, [r3]
	cmp	r3, #2
	bhi	.L3730
	add	r4, r4, #1
	cmp	r4, #256
	bne	.L3748
.L3730:
	mov	r0, #0
.L3690:
	add	sp, sp, #100
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3763:
	.align	2
.L3762:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR2-3008
	.word	.LANCHOR2-2960
	.word	.LANCHOR0+2914
	.word	.LANCHOR0+2920
	.word	.LANCHOR0+2918
	.word	.LC160
	.word	-3947
	.word	.LANCHOR2-2768
	.word	.LANCHOR2-2150
	.word	.LANCHOR2-2202
	.word	.LANCHOR2+0
	.word	.LANCHOR2-3012
	.fnend
	.size	ftl_write, .-ftl_write
	.align	2
	.global	ftl_vendor_write
	.type	ftl_vendor_write, %function
ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!
	.save {lr}
	mov	ip, r1
	mov	lr, r0
	mov	r3, r2
	mov	r1, lr
	mov	r0, #16
	mov	r2, ip
	ldr	lr, [sp], #4
	b	ftl_write
	.fnend
	.size	ftl_vendor_write, .-ftl_vendor_write
	.align	2
	.global	FlashBootVendorWrite
	.type	FlashBootVendorWrite, %function
FlashBootVendorWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r2
	mov	r6, r0
	mov	r5, r1
	bl	rknand_device_lock
	ldr	r3, .L3770
	ldr	r3, [r3, #504]
	cmp	r3, #1
	mvnne	r4, #0
	bne	.L3767
	mov	r2, r4
	mov	r0, r6
	mov	r1, r5
	bl	ftl_vendor_write
	mov	r4, r0
.L3767:
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L3771:
	.align	2
.L3770:
	.word	.LANCHOR1
	.fnend
	.size	FlashBootVendorWrite, .-FlashBootVendorWrite
	.align	2
	.global	ftl_sys_write
	.type	ftl_sys_write, %function
ftl_sys_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	ip, r1
	mov	r3, r2
	add	r1, r0, #256
	mov	r2, ip
	mov	r0, #16
	b	ftl_write
	.fnend
	.size	ftl_sys_write, .-ftl_sys_write
	.align	2
	.global	StorageSysDataStore
	.type	StorageSysDataStore, %function
StorageSysDataStore:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r1
	mov	r5, r0
	bl	rknand_device_lock
	mov	r2, r4
	mov	r1, #1
	mov	r0, r5
	bl	ftl_sys_write
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
	.fnend
	.size	StorageSysDataStore, .-StorageSysDataStore
	.align	2
	.global	FtlDumpSysBlock
	.type	FtlDumpSysBlock, %function
FtlDumpSysBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r8, r0, asl #10
	ldr	r4, .L3784
	.pad #28
	sub	sp, sp, #28
	ldr	r7, .L3784+4
	mov	r6, r0
	mov	r5, #0
	ldr	r3, [r4, #-2060]
	mov	r9, r4
	str	r3, [r4, #224]
	ldr	r3, [r4, #-2036]
	str	r3, [r4, #228]
.L3776:
	ldrh	r2, [r7]
	sxth	r3, r5
	cmp	r3, r2
	bge	.L3783
	mov	r1, #1
	ldr	r0, .L3784+8
	mov	r2, r1
	orr	r3, r3, r8
	str	r3, [r4, #220]
	bl	FlashReadPages
	ldr	r3, [r4, #228]
	mov	r1, r6
	ldr	r0, .L3784+12
	ldr	r2, [r3]
	str	r2, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	ldr	r2, [r3, #8]
	str	r2, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r2, [r4, #216]
	str	r3, [sp, #12]
	ldr	r3, [r4, #224]
	ldr	r3, [r3]
	str	r3, [sp, #16]
	ldr	r3, [r4, #220]
	bl	printk
	ldr	r3, [r4, #228]
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L3777
	ldr	r0, .L3784+16
	mov	r2, #4
	ldr	r1, [r9, #-2060]
	mov	r3, #768
	bl	rknand_print_hex
.L3777:
	add	r5, r5, #1
	b	.L3776
.L3783:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3785:
	.align	2
.L3784:
	.word	.LANCHOR2
	.word	.LANCHOR0+2910
	.word	.LANCHOR2+216
	.word	.LC161
	.word	.LC162
	.fnend
	.size	FtlDumpSysBlock, .-FtlDumpSysBlock
	.align	2
	.global	dump_map_info
	.type	dump_map_info, %function
dump_map_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r8, .L3804
	mov	r9, r8
	add	r8, r8, #20
	ldrh	r6, [r9], #2
.L3787:
	ldrh	r3, [r9]
	ldr	r4, .L3804+4
	cmp	r3, r6
	bls	.L3800
	ldr	r3, .L3804+8
	mov	r7, #0
	ldr	r1, .L3804+12
	mov	r5, r7
	ldr	r2, [r4, #-2072]
	ldrh	ip, [r3]
	ldr	fp, [r4, #-2176]
	ldr	r3, [r4, #-2180]
	ldrh	r10, [r1]
.L3796:
	uxth	r1, r7
	cmp	r1, ip
	bcs	.L3802
	mov	r1, r6
	ldrb	r0, [r8, r7]	@ zero_extendqisi2
	str	r3, [sp, #36]
	str	r2, [sp, #32]
	str	ip, [sp, #28]
	bl	V2P_block
	str	r0, [sp, #24]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldr	r1, [sp, #24]
	ldr	ip, [sp, #28]
	ldr	r2, [sp, #32]
	ldr	r3, [sp, #36]
	bne	.L3788
	mov	r0, #36
	mov	r1, r1, asl #10
	mla	r0, r0, r5, r2
	stmib	r0, {r1, r3}
	mul	r1, r10, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	lr, r1, #3
	cmp	r1, #0
	movlt	r1, lr
	bic	r1, r1, #3
	add	r1, fp, r1
	str	r1, [r0, #12]
.L3788:
	add	r7, r7, #1
	b	.L3796
.L3802:
	cmp	r5, #0
	beq	.L3791
	ldr	r0, [r4, #-2072]
	mov	r1, r5
	mov	r2, #1
	mov	r7, #0
	bl	FlashReadPages
	mov	r10, #36
.L3792:
	uxth	r3, r7
	cmp	r3, r5
	bcs	.L3791
	ldr	r3, [r4, #-2072]
	ldr	r0, .L3804+16
	mla	r3, r10, r7, r3
	add	r7, r7, #1
	ldmib	r3, {r2, r3, ip}
	ldr	r1, [ip, #4]
	str	r1, [sp]
	ldr	r1, [ip, #8]
	str	r1, [sp, #4]
	ldr	r1, [ip, #12]
	str	r1, [sp, #8]
	ldr	r1, [r3]
	str	r1, [sp, #12]
	ubfx	r1, r2, #10, #16
	ldr	r3, [r3, #4]
	str	r3, [sp, #16]
	ldr	r3, [ip]
	bl	printk
	b	.L3792
.L3791:
	add	r6, r6, #1
	uxth	r6, r6
	b	.L3787
.L3800:
	ldr	r8, .L3804+20
	mov	r7, #0
.L3795:
	ldr	r3, .L3804+24
	sxth	r5, r7
	ldr	r6, .L3804+4
	ldrh	r3, [r3]
	cmp	r5, r3
	bge	.L3798
	mov	r5, r5, asl #1
	mov	r6, #0
.L3799:
	ldrh	r2, [r8]
	sxth	r3, r6
	add	r6, r6, #1
	cmp	r3, r2
	bge	.L3803
	ldr	r2, [r4, #-2008]
	mov	r1, #1
	ldr	r0, .L3804+28
	ldrh	r2, [r2, r5]
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r4, #220]
	bl	FlashReadPages
	ldr	r3, [r4, #228]
	ldr	r1, [r4, #-2008]
	ldr	r2, [r4, #224]
	ldr	r0, [r3]
	ldrh	r1, [r1, r5]
	str	r0, [sp]
	ldr	r0, [r3, #4]
	str	r0, [sp, #4]
	ldr	r0, [r3, #8]
	str	r0, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r0, .L3804+32
	str	r3, [sp, #12]
	ldr	r3, [r2]
	str	r3, [sp, #16]
	ldr	r3, [r2, #4]
	str	r3, [sp, #20]
	ldr	r2, [r4, #216]
	ldr	r3, [r4, #220]
	bl	printk
	b	.L3799
.L3803:
	add	r7, r7, #1
	b	.L3795
.L3798:
	ldr	r3, .L3804+36
	mov	r2, #2
	ldr	r4, .L3804+40
	ldr	r1, [r6, #-2008]
	ldr	r0, .L3804+44
	ldr	r3, [r3, #2936]
	bl	rknand_print_hex
	ldr	r1, [r6, #-1988]
	ldrh	r3, [r4]
	mov	r2, #4
	ldr	r0, .L3804+48
	bl	rknand_print_hex
	ldr	r0, .L3804+52
	ldr	r1, [r6, #-1984]
	mov	r2, #4
	ldrh	r3, [r4]
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	rknand_print_hex
.L3805:
	.align	2
.L3804:
	.word	.LANCHOR0+2848
	.word	.LANCHOR2
	.word	.LANCHOR0+2840
	.word	.LANCHOR0+2920
	.word	.LC163
	.word	.LANCHOR0+2910
	.word	.LANCHOR2-1924
	.word	.LANCHOR2+216
	.word	.LC110
	.word	.LANCHOR0
	.word	.LANCHOR0+2944
	.word	.LC164
	.word	.LC165
	.word	.LC166
	.fnend
	.size	dump_map_info, .-dump_map_info
	.align	2
	.global	flash_boot_enter_slc_mode
	.type	flash_boot_enter_slc_mode, %function
flash_boot_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L3808
	ldr	r2, [r3, #2780]
	ldr	r3, .L3808+4
	cmp	r2, r3
	bxne	lr
	b	flash_enter_slc_mode
.L3809:
	.align	2
.L3808:
	.word	.LANCHOR0
	.word	1446522928
	.fnend
	.size	flash_boot_enter_slc_mode, .-flash_boot_enter_slc_mode
	.align	2
	.global	flash_boot_exit_slc_mode
	.type	flash_boot_exit_slc_mode, %function
flash_boot_exit_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L3812
	ldr	r2, [r3, #2780]
	ldr	r3, .L3812+4
	cmp	r2, r3
	bxne	lr
	b	flash_exit_slc_mode
.L3813:
	.align	2
.L3812:
	.word	.LANCHOR0
	.word	1446522928
	.fnend
	.size	flash_boot_exit_slc_mode, .-flash_boot_exit_slc_mode
	.align	2
	.type	IdBlockReadData, %function
IdBlockReadData:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r0
	ldr	r4, .L3823
	mov	r8, r1
	.pad #20
	sub	sp, sp, #20
	mov	r1, r6
	mov	fp, r2
	ldr	r0, .L3823+4
	ldr	r3, [r4, #1784]
	mov	r2, r8
	ldr	r5, [r4, #4]
	mov	r7, #0
	ldrb	r9, [r3, #9]	@ zero_extendqisi2
	bl	printk
	mov	r0, r6
	smulbb	r5, r5, r9
	uxth	r5, r5
	mov	r1, r5
	bl	__aeabi_uidivmod
	mul	ip, r9, r1
	mov	r10, r1
	rsb	r3, r1, r6
	str	r3, [sp]
	ubfx	ip, ip, #2, #2
.L3815:
	cmp	r7, r8
	bcs	.L3822
	add	r2, r7, r10
	ldrb	r1, [r4, #1888]	@ zero_extendqisi2
	ubfx	r2, r2, #2, #16
	rsb	r5, ip, #4
	cmp	r1, #0
	add	r3, r4, r2, asl #1
	uxth	r5, r5
	ldrh	r3, [r3, #76]
	beq	.L3816
	ldr	r1, .L3823
	ldr	r0, .L3823+8
	ldr	r1, [r1, #2780]
	cmp	r1, r0
	moveq	r3, r2
.L3816:
	ldr	r2, [sp]
	add	r7, r5, r7
	add	r2, ip, r2
	ldrb	ip, [r4, #1889]	@ zero_extendqisi2
	uxth	r7, r7
	mla	r3, r9, r3, r2
	ldr	r2, [r4, #1784]
	str	ip, [sp, #4]
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	str	r3, [sp, #12]
	ldr	r3, .L3823+12
	str	r1, [sp, #8]
	ldrb	r0, [r3, #178]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r3, [sp, #12]
	ldr	r1, [sp, #8]
	mov	r0, r3
	bl	__aeabi_uidiv
	mov	r2, fp
	add	fp, fp, r5, asl #9
	mov	r1, r0
	mov	r0, #0
	mov	r3, r0
	bl	FlashReadPage
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	ip, [sp, #4]
	mov	r0, ip
	bl	FlashBchSel
	mov	ip, #0
	b	.L3815
.L3822:
	mov	r1, r6
	mov	r2, r8
	mov	r3, #0
	ldr	r0, .L3823+16
	bl	printk
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3824:
	.align	2
.L3823:
	.word	.LANCHOR0
	.word	.LC167
	.word	1446522928
	.word	.LANCHOR2
	.word	.LC168
	.fnend
	.size	IdBlockReadData, .-IdBlockReadData
	.align	2
	.global	write_idblock
	.type	write_idblock, %function
write_idblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 120
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r0
	ldr	r7, .L3871
	.pad #132
	sub	sp, sp, #132
	mov	r0, #256000
	mov	r6, r1
	mov	r5, r2
	ldr	r3, [r7, #1784]
	ldr	r8, [r7, #4]
	ldrb	r9, [r3, #9]	@ zero_extendqisi2
	bl	ftl_malloc
	subs	r3, r0, #0
	str	r3, [sp, #12]
	beq	.L3849
	add	r4, r4, #508
	add	r4, r4, #3
	mov	r4, r4, lsr #9
	cmp	r4, #8
	bls	.L3847
	cmp	r4, #500
	bhi	.L3849
	b	.L3827
.L3847:
	mov	r4, #8
.L3827:
	ldr	r2, [r6]
	ldr	r3, .L3871+4
	cmp	r2, r3
	bne	.L3849
	smulbb	r3, r9, r8
	uxth	fp, r3
	sub	r0, fp, #1
	add	r0, r0, r4
	mov	r1, fp
	bl	__aeabi_uidiv
	mov	r3, #0
	movw	r2, #63871
	str	r0, [sp, #32]
	add	r0, r6, #254976
	add	r0, r0, #512
.L3831:
	ldr	r1, [r0, #-4]!
	cmp	r1, #0
	bne	.L3828
	ldr	r1, [r6, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, #4096
	sub	r2, r2, #1
	movhi	r3, #0
	cmp	r2, #4096
	str	r1, [r0, #512]
	bne	.L3831
	b	.L3830
.L3828:
	ldr	r0, .L3871+8
	bl	printk
.L3830:
	mov	r1, r5
	mov	r3, #5
	ldr	r0, .L3871+12
	mov	r2, #4
	bl	rknand_print_hex
	ldrb	r2, [r7, #1]	@ zero_extendqisi2
	ldr	r1, [r6, #512]
	sub	r5, r5, #4
	ldr	r0, .L3871+16
	bl	printk
	ldr	r2, .L3871+20
	ldr	r3, .L3871+24
	mov	r1, r4
	ldr	r0, .L3871+28
	ldr	r2, [r2, #172]
	ldrh	r3, [r3, #26]
	str	r2, [sp]
	mov	r2, r4
	bl	printk
	ldrb	r3, [r7, #1]	@ zero_extendqisi2
	ldr	r7, .L3871
	ldr	r2, [r6, #512]
	cmp	r2, r3
	strhi	r3, [r6, #512]
	mov	r3, r4, asl #7
	str	r3, [sp, #36]
	mov	r3, #0
	str	r3, [sp, #20]
	str	r3, [sp, #16]
.L3845:
	ldr	r2, [r5, #4]
	ldrb	r3, [r7, #1]	@ zero_extendqisi2
	ldr	r9, .L3871
	cmp	r2, r3
	bcs	.L3833
	ldr	r3, .L3871+20
	ldr	r3, [r3, #172]
	cmp	r2, r3
	bcc	.L3833
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #32]
	cmp	r3, #1
	movls	r3, #0
	movhi	r3, #1
	str	r3, [sp, #44]
	ldr	r3, [sp, #16]
	cmp	r3, #0
	cmpne	r1, #1
	bls	.L3834
	ldr	r3, [r5]
	add	r3, r3, #1
	cmp	r2, r3
	beq	.L3833
.L3834:
	ldr	r0, [sp, #12]
	mov	r1, #512
	bl	__memzero
	ldr	r8, [r5, #4]
	ldr	r3, [r9, #1784]
	mov	r2, r4
	ldr	r9, [r9, #4]
	mul	r8, r8, fp
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	ldr	r0, .L3871+32
	mov	r1, r8
	str	r3, [sp, #24]
	ldrh	r3, [sp, #24]
	smulbb	r9, r9, r3
	bl	printk
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	mov	r0, r8
	uxth	r9, r9
	ldr	r1, [sp, #24]
	bl	__aeabi_uidiv
	mov	r1, r0
	mov	r0, #0
	mov	r2, r0
	bl	FlashEraseBlock
	cmp	r4, r9
	movls	r10, #1
	bls	.L3835
	mov	r0, #0
	add	r1, r8, r9
	mov	r2, r0
	mov	r10, #2
	bl	FlashEraseBlock
.L3835:
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r3, [r7, #1784]
	ldrh	r0, [r3, #10]
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	mov	r0, r0, asl #2
	mul	r0, r10, r0
	bl	__aeabi_idiv
	mov	r1, r9
	mov	r9, #0
	str	r0, [sp, #52]
	mov	r0, r8
	bl	__aeabi_uidivmod
	mov	ip, r1
	rsb	r3, r1, r8
	str	r6, [sp, #28]
	str	r3, [sp, #40]
.L3836:
	ldr	r3, [sp, #52]
	cmp	r9, r3
	bcs	.L3870
	add	r3, r9, ip
	ubfx	r3, r3, #2, #16
	cmp	r3, #0
	beq	.L3837
	add	r2, r3, #1
	add	r1, r7, r2, asl #1
	ldrh	r10, [r1, #76]
	ldrb	r1, [r7, #1888]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L3838
	ldr	r1, [r7, #2780]
	ldr	r0, .L3871+36
	cmp	r1, r0
	moveq	r10, r2
.L3838:
	sub	r10, r10, #-1073741823
	mov	r10, r10, asl #2
	str	r10, [sp, #64]
.L3837:
	movw	r2, #61424
	str	r2, [sp, #68]
	add	r2, r7, r3, asl #1
	ldrh	r10, [r2, #76]
	ldrb	r2, [r7, #1888]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3839
	ldr	r2, [r7, #2780]
	ldr	r1, .L3871+36
	cmp	r2, r1
	moveq	r10, r3
.L3839:
	ldr	r3, [sp, #24]
	add	r9, r9, #4
	ldr	r2, [sp, #40]
	str	ip, [sp, #60]
	uxth	r9, r9
	mla	r2, r3, r10, r2
	ldr	r3, .L3871
	ldrb	r3, [r3, #1889]	@ zero_extendqisi2
	str	r2, [sp, #56]
	str	r3, [sp, #48]
	ldr	r3, .L3871+20
	ldrb	r0, [r3, #178]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r3, .L3871
	ldr	r3, [r3, #1784]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	ldr	r2, [sp, #56]
	mov	r0, r2
	bl	__aeabi_uidiv
	add	r3, sp, #64
	mov	r1, r0
	ldr	r2, [sp, #28]
	mov	r0, #0
	bl	FlashProgPage
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r0, [sp, #48]
	bl	FlashBchSel
	ldr	r1, [sp, #24]
	ldr	r0, [sp, #40]
	bl	__aeabi_uidiv
	add	r2, r10, #1
	uxth	r2, r2
	mov	r1, r0
	mov	r0, #0
	bl	FlashPageProgMsbFFData
	ldr	r3, [sp, #28]
	ldr	ip, [sp, #60]
	add	r3, r3, #2048
	str	r3, [sp, #28]
	b	.L3836
.L3870:
	mov	r1, r8
	mov	r3, #0
	mov	r2, r4
	ldr	r0, .L3871+40
	bl	printk
	ldr	r0, [r5, #4]
	mov	r1, r4
	mov	r8, #0
	mul	r0, r0, fp
	ldr	r2, [sp, #12]
	bl	IdBlockReadData
	mov	r3, r6
	ldr	r2, [sp, #12]
.L3844:
	mov	r10, r2
	mov	r9, r3
	ldr	r0, [r10]
	add	r2, r2, #4
	ldr	r1, [r9]
	add	r3, r3, #4
	cmp	r0, r1
	beq	.L3841
	ldr	r0, [sp, #12]
	mov	r1, #512
	bl	__memzero
	ldr	r3, [r10]
	ldr	r0, .L3871+44
	str	r3, [sp]
	ldr	r3, [r9]
	bic	r9, r8, #255
	ldr	r1, [sp, #16]
	mov	r9, r9, asl #2
	str	r3, [sp, #4]
	mov	r3, r8
	ldr	r2, [r5, #4]
	bl	printk
	ldr	r0, .L3871+48
	add	r1, r6, r9
	mov	r2, #4
	mov	r3, #256
	bl	rknand_print_hex
	mov	r2, #4
	ldr	r0, .L3871+52
	ldr	r3, [sp, #12]
	add	r1, r3, r9
	mov	r3, #256
	bl	rknand_print_hex
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r1, [r5, #4]
	mov	r0, #0
	mov	r2, r0
	mul	r1, r1, fp
	bl	FlashEraseBlock
	ldr	r3, [sp, #44]
	cmp	r3, #0
	beq	.L3842
	ldr	r1, [r5, #4]
	mov	r0, #0
	mov	r2, r0
	mla	r1, r1, fp, fp
	bl	FlashEraseBlock
.L3842:
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r0, .L3871+56
	ldr	r1, [r5, #4]
	bl	printk
	ldr	r3, [sp, #36]
	cmp	r8, r3
	bcc	.L3833
	b	.L3843
.L3841:
	ldr	r1, [sp, #36]
	add	r8, r8, #1
	cmp	r8, r1
	bne	.L3844
.L3843:
	ldr	r3, [sp, #20]
	add	r3, r3, #1
	str	r3, [sp, #20]
.L3833:
	ldr	r3, [sp, #16]
	add	r5, r5, #4
	add	r3, r3, #1
	str	r3, [sp, #16]
	cmp	r3, #5
	bne	.L3845
	ldr	r0, [sp, #12]
	bl	ftl_free
	ldr	r3, [sp, #20]
	clz	r0, r3
	mov	r0, r0, lsr #5
	rsb	r0, r0, #0
	b	.L3826
.L3849:
	mvn	r0, #0
.L3826:
	add	sp, sp, #132
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3872:
	.align	2
.L3871:
	.word	.LANCHOR0
	.word	-52655045
	.word	.LC169
	.word	.LC170
	.word	.LC171
	.word	.LANCHOR2
	.word	.LANCHOR0+1860
	.word	.LC172
	.word	.LC173
	.word	1446522928
	.word	.LC174
	.word	.LC175
	.word	.LC176
	.word	.LC177
	.word	.LC178
	.fnend
	.size	write_idblock, .-write_idblock
	.align	2
	.global	CRC_32
	.type	CRC_32, %function
CRC_32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	ip, .L3877
	add	r1, r0, r1
	mov	r2, #0
.L3874:
	cmp	r0, r1
	beq	.L3876
	ldrb	r3, [r0], #1	@ zero_extendqisi2
	eor	r3, r3, r2, lsr #24
	add	r3, ip, r3, asl #2
	ldr	r3, [r3, #3296]
	eor	r2, r3, r2, asl #8
	b	.L3874
.L3876:
	mov	r0, r2
	bx	lr
.L3878:
	.align	2
.L3877:
	.word	.LANCHOR1
	.fnend
	.size	CRC_32, .-CRC_32
	.align	2
	.global	write_loader_lba
	.type	write_loader_lba, %function
write_loader_lba:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #64
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r5, r0
	.pad #52
	sub	sp, sp, #52
	mov	r6, r1
	mov	r8, r2
	ldr	r4, .L3902
	bne	.L3880
	ldr	r2, [r2]
	ldr	r3, .L3902+4
	cmp	r2, r3
	bne	.L3880
	mov	r0, #256000
	mov	r3, #1
	strb	r3, [r4, #484]
	bl	ftl_malloc
	mov	r1, #256000
	str	r0, [r4, #488]
	bl	__memzero
	str	r5, [r4, #492]
.L3880:
	str	r6, [sp]
	mov	r3, r5
	ldr	r0, .L3902+8
	ldr	r1, [r4, #488]
	ldr	r2, [r8]
	bl	printk
	ldrb	r3, [r4, #484]	@ zero_extendqisi2
	ldr	r9, .L3902
	cmp	r3, #0
	beq	.L3879
	sub	r0, r5, #64
	ldr	r7, [r9, #488]
	cmp	r0, #500
	bcs	.L3882
	rsb	r2, r5, #564
	add	r0, r7, r0, asl #9
	cmp	r2, r6
	mov	r1, r8
	movcs	r2, r6
	mov	r2, r2, asl #9
	bl	ftl_memcpy
	b	.L3883
.L3882:
	cmp	r5, #564
	bcs	.L3891
.L3883:
	ldr	r3, [r4, #492]
	cmp	r3, r5
	beq	.L3889
	ldr	r2, .L3902
	mov	r3, #0
	cmp	r7, r3
	mov	r8, r3
	strb	r3, [r2, #484]
	beq	.L3890
	mov	r0, r7
	bl	ftl_free
.L3890:
	str	r8, [r4, #488]
.L3889:
	add	r5, r5, r6
	str	r5, [r4, #492]
	b	.L3879
.L3891:
	ldr	r3, .L3902+12
	ldr	r0, [r9, #492]
	ldr	r3, [r3, #1784]
	sub	r0, r0, #64
	cmp	r0, #500
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	movcs	r0, #500
	cmp	r3, #4
	beq	.L3892
	mov	r3, #2
	str	r3, [sp, #8]
	mov	r3, #3
	str	r3, [sp, #12]
	mov	r3, #4
	str	r3, [sp, #16]
	mov	r3, #5
	str	r3, [sp, #20]
	mov	r3, #6
	str	r3, [sp, #24]
	b	.L3885
.L3892:
	mov	r3, #0
.L3884:
	cmp	r0, #256
	add	r1, sp, #8
	mov	r2, r3, asl #1
	movls	r2, r3
	str	r2, [r1, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, #5
	bne	.L3884
.L3885:
	movw	r3, #63872
.L3888:
	ldr	r2, [r7, r3, asl #2]
	cmp	r2, #0
	addne	r3, r3, #128
	movne	r0, r3, asl #2
	bne	.L3887
.L3886:
	sub	r3, r3, #1
	cmp	r3, #4096
	bne	.L3888
	mov	r0, r0, asl #9
.L3887:
	mov	r1, r7
	add	r2, sp, #8
	bl	write_idblock
	ldr	r0, [r4, #488]
	mov	r7, #0
	strb	r7, [r4, #484]
	bl	ftl_free
	str	r7, [r4, #488]
	b	.L3889
.L3879:
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3903:
	.align	2
.L3902:
	.word	.LANCHOR4
	.word	-52655045
	.word	.LC179
	.word	.LANCHOR0
	.fnend
	.size	write_loader_lba, .-write_loader_lba
	.align	2
	.global	FtlWrite
	.type	FtlWrite, %function
FtlWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r6, r2
	sub	r2, r1, #64
	mov	r4, r1
	cmp	r2, #1984
	mov	r5, r3
	mov	r7, r0
	movcs	r2, #0
	movcc	r2, #1
	cmp	r0, #0
	movne	r2, #0
	cmp	r2, #0
	beq	.L3905
	mov	r0, r1
	mov	r2, r3
	mov	r1, r6
	bl	write_loader_lba
.L3905:
	mov	r0, r7
	mov	r1, r4
	mov	r2, r6
	mov	r3, r5
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	b	ftl_write
	.fnend
	.size	FtlWrite, .-FtlWrite
	.align	2
	.global	rknand_sys_storage_ioctl
	.type	rknand_sys_storage_ioctl, %function
rknand_sys_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 528
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L4016
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	cmp	r1, r3
	.pad #532
	sub	sp, sp, #532
	mov	r4, r1
	mov	r5, r2
	beq	.L3912
	bhi	.L3913
	sub	r3, r3, #3456
	sub	r3, r3, #5
	cmp	r1, r3
	beq	.L3914
	bhi	.L3915
	sub	r3, r3, #125
	cmp	r1, r3
	beq	.L3916
	bhi	.L3917
	sub	r3, r3, #237
	cmp	r1, r3
	bne	.L3973
	bl	rknand_dev_flush
	b	.L4004
.L3917:
	ldr	r3, .L4016+4
	cmp	r1, r3
	beq	.L3919
	add	r3, r3, #1
	cmp	r1, r3
	beq	.L3920
	b	.L3973
.L3915:
	ldr	r3, .L4016+8
	cmp	r1, r3
	beq	.L3921
	bhi	.L3922
	sub	r3, r3, #1952
	sub	r3, r3, #9
	cmp	r1, r3
	beq	.L3923
	b	.L3973
.L3922:
	ldr	r3, .L4016+12
	cmp	r1, r3
	beq	.L3921
	add	r3, r3, #10
	cmp	r1, r3
	beq	.L3921
	b	.L3973
.L3913:
	ldr	r3, .L4016+16
	cmp	r1, r3
	beq	.L3924
	bhi	.L3925
	sub	r3, r3, #78
	cmp	r1, r3
	beq	.L3926
	bcc	.L3927
	add	r3, r3, #21
	cmp	r1, r3
	beq	.L3928
	add	r3, r3, #56
	cmp	r1, r3
	beq	.L3929
	b	.L3973
.L3925:
	ldr	r3, .L4016+20
	cmp	r1, r3
	mov	r6, r3
	beq	.L3930
	bhi	.L3931
	sub	r3, r3, #956
	sub	r3, r3, #1
	cmp	r1, r3
	beq	.L3932
	add	r3, r3, #956
	cmp	r1, r3
	beq	.L3933
	b	.L3973
.L3931:
	ldr	r3, .L4016+24
	cmp	r1, r3
	beq	.L3933
	add	r3, r3, #1
	cmp	r1, r3
	beq	.L3930
	b	.L3973
.L3927:
	ldr	r0, .L4016+28
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r4, r0, #0
	bne	.L3934
.L3939:
	mvn	r0, #11
	b	.L3911
.L3934:
	mov	r1, r5
	mov	r2, #512
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4010
	ldr	r0, .L4016+32
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r4, #4]
	cmp	r3, #8
	str	r3, [sp, #4]
	bhi	.L4008
	bl	rknand_device_lock
	mov	r2, r4
	ldr	r0, [r4]
	ldr	r1, [sp, #4]
	bl	IdBlockReadData
	bl	rknand_device_unlock
	mov	r0, r5
	mov	r1, r4
	ldr	r2, [sp, #4]
	mov	r2, r2, asl #9
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L4003
	ldr	r0, .L4016+36
.L4006:
	bl	printk
.L4008:
	mov	r0, r4
.L4009:
	bl	ftl_free
.L4007:
	mvn	r0, #13
	b	.L3911
.L3926:
	ldr	r0, .L4016+40
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r4, r0, #0
	beq	.L3939
	mov	r1, r5
	mov	r2, #4096
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L3940
.L4010:
	ldr	r0, .L4016+44
	b	.L4006
.L3940:
	ldr	r5, .L4016+48
	ldr	r0, .L4016+52
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r5, #496]
	mov	r6, r5
	cmp	r3, #0
	bne	.L3941
	mov	r0, #260096
	bl	ftl_malloc
	cmp	r0, #0
	str	r0, [r5, #496]
	beq	.L4008
	mov	r1, #260096
	bl	__memzero
.L3941:
	ldr	r2, [r4, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L4008
	ldr	r3, [r4]
	cmp	r3, #251904
	bhi	.L4008
	ldr	r0, [r6, #496]
	add	r1, r4, #8
	add	r0, r0, r3
	bl	memcpy
.L4003:
	mov	r0, r4
	bl	ftl_free
.L4004:
	mov	r4, #0
	b	.L3938
.L3929:
	ldr	r0, .L4016+56
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L3939
	mov	r1, r5
	mov	r2, #28
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L3944
	ldr	r0, .L4016+44
	bl	printk
	b	.L3945
.L3944:
	ldmia	r6, {r1, r2}
	ldr	r0, .L4016+60
	bl	printk
	ldr	r1, [r6]
	cmp	r1, #256000
	bhi	.L3945
	ldr	r5, .L4016+48
	ldr	r0, [r5, #496]
	cmp	r0, #0
	beq	.L3945
	bl	CRC_32
	ldr	r3, [r6, #4]
	cmp	r3, r0
	beq	.L3947
	mov	r0, r6
	bl	ftl_free
	b	.L3974
.L3947:
	bl	rknand_device_lock
	ldr	r1, [r5, #496]
	add	r2, r6, #8
	ldr	r0, [r6]
	bl	write_idblock
	mov	r4, #0
	bl	rknand_device_unlock
	ldr	r0, [r5, #496]
	bl	ftl_free
	str	r4, [r5, #496]
	mov	r0, r6
	b	.L3951
.L3945:
	mov	r0, r6
	b	.L4009
.L3928:
	ldr	r0, .L4016+64
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L3939
	bl	ftl_read_flash_info
	mov	r0, r5
	mov	r1, r6
	mov	r2, #11
	b	.L4014
.L3912:
	ldr	r0, .L4016+68
	bl	printk
	bl	nand_blk_add_whole_disk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L3939
	mov	r1, #64
	bl	__memzero
	mov	r0, r5
	mov	r1, r6
	mov	r2, #64
	b	.L4014
.L3924:
	ldr	r0, .L4016+72
	bl	printk
	ldr	r3, .L4016+48
	mov	r0, r5
	mov	r2, #4
	ldr	r3, [r3, #500]
	ldr	r3, [r3, #20]
	add	r1, sp, #528
	str	r3, [r1, #-524]!
.L3998:
	bl	rk_copy_to_user
	cmp	r0, #0
	bne	.L4007
	b	.L4004
.L3932:
	ldr	r0, .L4016+76
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L3939
	bl	rknand_device_lock
	mov	r1, #264
	mov	r2, #2
	mov	r3, r6
	mov	r0, #16
	bl	ftl_read
	bl	rknand_device_unlock
	mov	r0, r5
	mov	r1, r6
	mov	r2, #1024
.L4014:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	mov	r0, r6
	bne	.L4009
.L3951:
	bl	ftl_free
	b	.L3938
.L3916:
	ldr	r0, .L4016+80
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	subs	r4, r0, #0
	beq	.L3952
.L3957:
	ldr	r0, .L4016+44
	bl	printk
	b	.L4007
.L3952:
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+84
	cmp	r2, r3
	beq	.L3953
.L3954:
	mvn	r4, #0
	b	.L3938
.L3953:
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3954
	ldr	r6, .L4016+48
	mov	r2, #512
	add	r0, sp, #8
	ldr	r1, [r6, #500]
	bl	memcpy
	ldr	r2, [r6, #504]
	ldr	r3, .L4016+88
	cmp	r2, r3
	beq	.L3955
	add	r0, sp, #72
	mov	r1, #128
	str	r4, [sp, #16]
	str	r4, [sp, #20]
	bl	__memzero
.L3955:
	add	r0, sp, #264
	mov	r1, #256
	mov	r3, #0
	str	r3, [sp, #24]
	bl	__memzero
	b	.L4012
.L3919:
	ldr	r0, .L4016+92
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3957
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+84
	cmp	r2, r3
	bne	.L3954
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3954
	ldr	r2, .L4016+48
	ldr	r3, .L4016+88
	ldr	r1, [r2, #504]
	cmp	r1, r3
	bne	.L3974
	ldr	r3, [sp, #20]
	sub	r1, r3, #1
	cmp	r1, #127
	mvnhi	r0, #2
	bhi	.L3911
	ldr	r4, [r2, #500]
	add	r1, sp, #72
	add	r0, r4, #64
	str	r3, [r4, #12]
	ldr	r2, [sp, #20]
	bl	memcpy
	mov	r0, #1
	mov	r1, r4
	b	.L4005
.L3923:
	ldr	r0, .L4016+96
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3957
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+100
	cmp	r2, r3
	bne	.L3954
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3954
	ldr	r5, .L4016+48
	ldr	r3, [r5, #508]
	cmp	r3, #0
	bne	.L3958
.L3961:
	mov	r0, #0
	b	.L3911
.L3958:
	ldr	r3, [r5, #512]
	ldr	r2, .L4016+104
	ldr	r1, [r3]
	cmp	r1, r2
	beq	.L3959
	str	r2, [r3]
	mov	r2, #504
	ldr	r3, [r5, #512]
	str	r2, [r3, #4]
	mov	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L3959:
	ldr	r1, [r5, #512]
	mov	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #500]
	ldr	r2, .L4016+84
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r0, [r5, #500]
	mov	r1, #128
	ldrne	r3, .L4016+48
	movne	r2, #504
	add	r0, r0, #64
	ldrne	r3, [r3, #500]
	stmneib	r3, {r2, r4}
	mov	r4, #0
	str	r4, [r0, #-52]
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #500]
	bl	StorageSysDataStore
	str	r4, [r5, #508]
	str	r4, [r5, #504]
	b	.L3938
.L3914:
	ldr	r0, .L4016+108
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3957
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+112
	cmp	r2, r3
	bne	.L3954
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L3954
	ldr	r5, .L4016+48
	ldr	r3, [r5, #508]
	cmp	r3, #1
	beq	.L3961
	ldr	r3, [r5, #512]
	mov	r4, #0
	ldr	r2, .L4016+104
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r1, [r5, #512]
	ldrne	r3, [r5, #512]
	movne	r2, #504
	strne	r0, [r3, #8]
	strne	r2, [r3, #4]
	strne	r0, [r3, #12]
	mov	r3, #1
	mov	r0, #0
	str	r3, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #500]
	ldr	r2, .L4016+84
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r0, [r5, #500]
	ldrne	r3, .L4016+48
	movne	r1, #504
	movne	r2, #0
	add	r0, r0, #64
	ldrne	r3, [r3, #500]
	stmneib	r3, {r1, r2}
	mov	r1, #128
	str	r4, [r0, #-52]
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #500]
	bl	StorageSysDataStore
	mov	r3, #1
	str	r3, [r5, #508]
	b	.L3938
.L3920:
	ldr	r0, .L4016+116
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3957
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+120
	cmp	r2, r3
	bne	.L3954
	ldr	r2, [sp, #12]
	cmp	r2, #512
	addls	r0, sp, #16
	ldrls	r1, .L4016+124
	bls	.L4011
	b	.L3954
.L3921:
	ldr	r0, .L4016+12
	cmp	r4, r0
	mov	r7, r0
	ldreq	r0, .L4016+128
	beq	.L4001
	ldr	r3, .L4016+132
	cmp	r4, r3
	ldreq	r0, .L4016+136
	ldrne	r0, .L4016+140
.L4001:
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3957
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+144
	cmp	r2, r3
	bne	.L4007
	ldr	r3, .L4016+132
	ldr	r6, .L4016+48
	cmp	r4, r3
	bne	.L3967
	ldr	r3, [r6, #500]
	mov	r0, r5
	add	r1, sp, #8
	mov	r2, #16
	ldr	r3, [r3, #20]
	str	r3, [sp, #12]
	strb	r3, [sp, #16]
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L3911
	b	.L4007
.L3967:
	ldr	r3, [r6, #1028]
	cmp	r3, #10
	bhi	.L4007
	ldr	r1, [r6, #500]
	ldr	r2, [sp, #12]
	ldr	r3, [r1, #24]
	cmp	r3, r2
	cmpne	r3, #0
	movne	r3, #1
	moveq	r3, #0
	beq	.L3968
	ldr	r0, .L4016+148
	mov	r1, r2
	bl	printk
	ldr	r3, [r6, #1028]
	add	r3, r3, #1
	str	r3, [r6, #1028]
	b	.L4007
.L3968:
	cmp	r4, r7
	str	r3, [r6, #1028]
	mov	r0, #1
	moveq	r2, r3
	movne	r3, #1
	moveq	r3, r2
	str	r2, [r1, #24]
	str	r3, [r1, #20]
	bl	StorageSysDataStore
	cmn	r0, #1
	bne	.L4004
	mvn	r4, #1
	b	.L3938
.L3933:
	ldr	r0, .L4016+152
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3957
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+156
	cmp	r2, r3
	bne	.L3954
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L3954
	ldr	r3, .L4016+160
	add	r0, sp, #16
	cmp	r4, r3
	ldr	r3, .L4016+48
	ldreq	r1, [r3, #1032]
	ldrne	r1, [r3, #1036]
	add	r1, r1, #8
.L4011:
	bl	memcpy
.L4012:
	mov	r0, r5
	add	r1, sp, #8
	mov	r2, #520
	b	.L3998
.L3930:
	ldr	r0, .L4016+164
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L3957
	ldr	r2, [sp, #8]
	ldr	r3, .L4016+156
	cmp	r2, r3
	bne	.L3954
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L3954
	cmp	r4, r6
	add	r2, r2, #8
	ldr	r4, .L4016+48
	bne	.L3972
	add	r1, sp, #8
	ldr	r0, [r4, #1032]
	bl	memcpy
	mov	r0, #2
	ldr	r1, [r4, #1032]
	b	.L4005
.L3972:
	add	r1, sp, #8
	ldr	r0, [r4, #1036]
	bl	memcpy
	ldr	r1, [r4, #1036]
	mov	r0, #3
.L4005:
	bl	StorageSysDataStore
	mov	r4, r0
.L3938:
	ldr	r0, .L4016+168
	mov	r1, r4
	bl	printk
	mov	r0, r4
	b	.L3911
.L3973:
	mvn	r0, #21
	b	.L3911
.L3974:
	mvn	r0, #1
.L3911:
	add	sp, sp, #532
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L4017:
	.align	2
.L4016:
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
	.word	.LANCHOR4
	.word	.LC185
	.word	.LC186
	.word	.LC187
	.word	.LC188
	.word	.LC189
	.word	.LC190
	.word	.LC191
	.word	.LC192
	.word	1263358532
	.word	-1067903959
	.word	.LC193
	.word	.LC194
	.word	1112753220
	.word	1146313043
	.word	.LC195
	.word	1112755781
	.word	.LC196
	.word	1094995539
	.word	.LANCHOR4+516
	.word	.LC197
	.word	1074031676
	.word	.LC198
	.word	.LC199
	.word	1280262987
	.word	.LC200
	.word	.LC201
	.word	1145980246
	.word	1074034192
	.word	.LC202
	.word	.LC203
	.fnend
	.size	rknand_sys_storage_ioctl, .-rknand_sys_storage_ioctl
	.align	2
	.global	rk_ftl_storage_sys_init
	.type	rk_ftl_storage_sys_init, %function
rk_ftl_storage_sys_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r2, #512
	ldr	r4, .L4028
	mvn	r3, #0
	mov	r5, #0
	add	r0, r4, #516
	ldr	r1, [r4, #468]
	str	r3, [r4, #492]
	add	r3, r1, #512
	str	r3, [r4, #500]
	str	r1, [r4, #512]
	add	r3, r1, #1024
	add	r1, r1, #1536
	str	r3, [r4, #1032]
	strb	r5, [r4, #484]
	str	r5, [r4, #488]
	str	r5, [r4, #496]
	str	r1, [r4, #1036]
	bl	ftl_memcpy
	ldr	r6, [r4, #512]
	str	r5, [r4, #504]
	str	r5, [r4, #1028]
	ldr	r7, [r6, #508]
	ldr	r3, [r6, #16]
	cmp	r7, r5
	str	r3, [r4, #508]
	beq	.L4019
	mov	r0, r6
	mov	r1, #508
	bl	js_hash
	cmp	r7, r0
	beq	.L4019
	str	r5, [r6, #16]
	ldr	r0, .L4028+4
	str	r5, [r4, #508]
	bl	printk
.L4019:
	ldr	r3, [r4, #508]
	mov	r0, #2
	ldr	r1, [r4, #1032]
	cmp	r3, #0
	ldrne	r3, .L4028
	ldrne	r2, .L4028+8
	strne	r2, [r3, #504]
	bl	StorageSysDataLoad
	ldr	r1, [r4, #1036]
	mov	r0, #3
	bl	StorageSysDataLoad
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	rknand_sys_storage_init
.L4029:
	.align	2
.L4028:
	.word	.LANCHOR4
	.word	.LC204
	.word	-1067903959
	.fnend
	.size	rk_ftl_storage_sys_init, .-rk_ftl_storage_sys_init
	.align	2
	.global	StorageSysDataDeInit
	.type	StorageSysDataDeInit, %function
StorageSysDataDeInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r0, #0
	bx	lr
	.fnend
	.size	StorageSysDataDeInit, .-StorageSysDataDeInit
	.align	2
	.global	rk_ftl_vendor_storage_init
	.type	rk_ftl_vendor_storage_init, %function
rk_ftl_vendor_storage_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r0, #65536
	bl	ftl_malloc
	ldr	r5, .L4042
	cmp	r0, #0
	str	r0, [r5, #1040]
	beq	.L4037
	ldr	r8, .L4042+4
	mov	r7, #0
	mov	r4, r7
	mov	r6, r7
	mov	r9, r5
.L4035:
	mov	r0, r6, asl #7
	mov	r1, #128
	ldr	r2, [r5, #1040]
	bl	FlashBootVendorRead
	cmp	r0, #0
	bne	.L4033
	ldr	r3, [r9, #1040]
	ldr	r0, .L4042+8
	add	r2, r3, #61440
	ldr	r1, [r3]
	ldr	r2, [r2, #4092]
	ldr	r3, [r3, #4]
	bl	printk
	ldr	r2, [r9, #1040]
	ldr	r3, [r2]
	cmp	r3, r8
	bne	.L4034
	add	r1, r2, #61440
	ldr	r3, [r2, #4]
	ldr	r0, [r1, #4092]
	cmp	r4, r3
	movcs	r1, #0
	movcc	r1, #1
	cmp	r0, r3
	movne	r1, #0
	cmp	r1, #0
	movne	r7, r6
	movne	r4, r3
.L4034:
	cmp	r6, #1
	movne	r6, #1
	bne	.L4035
.L4041:
	cmp	r4, #0
	beq	.L4036
	mov	r0, r7, asl #7
	mov	r1, #128
	bl	FlashBootVendorRead
	cmp	r0, #0
	bne	.L4033
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4036:
	mov	r0, r2
	mov	r1, #65536
	bl	__memzero
	ldr	r3, [r5, #1040]
	ldr	r2, .L4042+4
	mov	r0, r4
	str	r6, [r3, #4]
	str	r2, [r3]
	add	r2, r3, #61440
	str	r6, [r2, #4092]
	ldr	r2, .L4042+12
	strh	r4, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4033:
	ldr	r0, [r5, #1040]
	bl	kfree
	mov	r3, #0
	mvn	r0, #0
	str	r3, [r5, #1040]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4037:
	mvn	r0, #11
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4043:
	.align	2
.L4042:
	.word	.LANCHOR4
	.word	1380668996
	.word	.LC205
	.word	-1032
	.fnend
	.size	rk_ftl_vendor_storage_init, .-rk_ftl_vendor_storage_init
	.align	2
	.global	rk_ftl_vendor_read
	.type	rk_ftl_vendor_read, %function
rk_ftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L4051
	ldr	ip, [r3, #1040]
	cmp	ip, #0
	ldrneh	r4, [ip, #10]
	movne	r3, #0
	beq	.L4049
.L4046:
	cmp	r3, r4
	bcs	.L4049
	add	lr, ip, r3, asl #3
	ldrh	r5, [lr, #16]
	cmp	r5, r0
	bne	.L4047
	ldrh	r3, [lr, #20]
	mov	r0, r1
	ldrh	r1, [lr, #18]
	mov	r4, r2
	cmp	r2, r3
	add	r1, r1, #1024
	movcs	r4, r3
	add	r1, ip, r1
	mov	r2, r4
	bl	memcpy
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L4047:
	add	r3, r3, #1
	b	.L4046
.L4049:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L4052:
	.align	2
.L4051:
	.word	.LANCHOR4
	.fnend
	.size	rk_ftl_vendor_read, .-rk_ftl_vendor_read
	.align	2
	.global	rk_ftl_vendor_write
	.type	rk_ftl_vendor_write, %function
rk_ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r2
	ldr	r2, .L4076
	.pad #28
	sub	sp, sp, #28
	ldr	r4, [r2, #1040]
	cmp	r4, #0
	beq	.L4068
	mov	r3, r1
	add	r7, r9, #63
	ldrh	r1, [r4, #8]
	bic	r7, r7, #63
	ldrh	r2, [r4, #10]
	mov	ip, r0
	mov	r6, #0
	str	r1, [sp, #4]
.L4055:
	cmp	r6, r2
	bcs	.L4074
	add	r5, r4, r6, asl #3
	ldrh	r1, [r5, #16]
	cmp	r1, ip
	bne	.L4056
	ldrh	r1, [r5, #20]
	add	fp, r4, #1024
	add	r1, r1, #63
	bic	r1, r1, #63
	str	r1, [sp, #8]
	cmp	r9, r1
	bls	.L4057
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	subcs	r2, r2, #1
	strcs	r2, [sp, #12]
	ldrcsh	r8, [r5, #18]
	bcc	.L4068
.L4058:
	ldr	r2, [sp, #12]
	add	r5, r5, #8
	cmp	r6, r2
	bcs	.L4075
	ldrh	r10, [r5, #20]
	add	r0, fp, r8
	ldrh	r2, [r5, #16]
	add	r6, r6, #1
	ldrh	r1, [r5, #18]
	strh	r10, [r5, #12]	@ movhi
	add	r10, r10, #63
	bic	r10, r10, #63
	strh	r2, [r5, #8]	@ movhi
	strh	r8, [r5, #10]	@ movhi
	add	r1, fp, r1
	mov	r2, r10
	str	r3, [sp, #20]
	str	ip, [sp, #16]
	bl	memcpy
	add	r8, r8, r10
	ldr	r3, [sp, #20]
	ldr	ip, [sp, #16]
	b	.L4058
.L4075:
	add	r6, r4, r6, asl #3
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
	add	r5, r3, r5
	rsb	r7, r7, r5
	strh	r7, [r4, #14]	@ movhi
	b	.L4073
.L4057:
	ldrh	r0, [r5, #18]
	mov	r1, r3
	mov	r2, r9
	add	r0, fp, r0
	bl	memcpy
	strh	r9, [r5, #20]	@ movhi
	b	.L4073
.L4056:
	add	r6, r6, #1
	b	.L4055
.L4074:
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	bcc	.L4068
	add	r2, r4, r2, asl #3
	uxth	r7, r7
	rsb	r1, r7, r1
	strh	ip, [r2, #16]	@ movhi
	ldrh	r0, [r4, #12]
	strh	r9, [r2, #20]	@ movhi
	strh	r0, [r2, #18]	@ movhi
	add	r0, r7, r0
	strh	r1, [r4, #14]	@ movhi
	mov	r1, r3
	strh	r0, [r4, #12]	@ movhi
	ldrh	r0, [r2, #18]
	mov	r2, r9
	add	r0, r0, #1024
	add	r0, r4, r0
	bl	memcpy
	ldrh	r3, [r4, #10]
	add	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
.L4073:
	ldr	r3, [r4, #4]
	add	r2, r4, #61440
	mov	r1, #128
	add	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r2, #4092]
	mov	r2, r4
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #1
	movhi	r3, #0
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, [sp, #4]
	mov	r0, r3, asl #7
	bl	FlashBootVendorWrite
	mov	r0, #0
	b	.L4054
.L4068:
	mvn	r0, #0
.L4054:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4077:
	.align	2
.L4076:
	.word	.LANCHOR4
	.fnend
	.size	rk_ftl_vendor_write, .-rk_ftl_vendor_write
	.align	2
	.global	rk_ftl_vendor_storage_ioctl
	.type	rk_ftl_vendor_storage_ioctl, %function
rk_ftl_vendor_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r0, #4096
	mov	r5, r2
	mov	r6, r1
	bl	ftl_malloc
	subs	r4, r0, #0
	mvneq	r5, #0
	beq	.L4079
	ldr	r3, .L4095
	cmp	r6, r3
	beq	.L4081
	add	r3, r3, #1
	cmp	r6, r3
	beq	.L4082
	b	.L4093
.L4081:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4093
	ldr	r2, [r4]
	ldr	r3, .L4095+4
	cmp	r2, r3
	beq	.L4084
.L4085:
	mvn	r5, #0
	b	.L4080
.L4084:
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_read
	cmn	r0, #1
	beq	.L4085
	uxth	r2, r0
	strh	r0, [r4, #6]	@ movhi
	mov	r1, r4
	mov	r0, r5
	add	r2, r2, #8
	bl	rk_copy_to_user
	cmp	r0, #0
	moveq	r5, #0
	mvnne	r5, #13
	b	.L4080
.L4082:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4093
	ldr	r2, [r4]
	ldr	r3, .L4095+4
	cmp	r2, r3
	bne	.L4085
	ldrh	r2, [r4, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L4085
	mov	r0, r4
	mov	r1, r5
	add	r2, r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4093
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_write
	mov	r5, r0
	b	.L4080
.L4093:
	mvn	r5, #13
.L4080:
	mov	r0, r4
	bl	kfree
.L4079:
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L4096:
	.align	2
.L4095:
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
.LANCHOR2 = . + 8184
.LANCHOR4 = . + 16368
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
