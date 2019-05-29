/*
 * Copyright (c) 2016-2018, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-05-29
 * function: rk ftl v5 for rockchip soc base on arm v7 to support MLC NAND.
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
	.file	"rk_ftlv5_arm_v7.S"
#APP
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
	bl	ftl_memset
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
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, #0
	ldr	r3, .L132
	ldr	r4, .L132+4
	ldr	r9, .L132+8
	ldrh	r0, [r3, #10]
	strb	r5, [r4, #1612]
	strb	r5, [r4, #1613]
	bl	FlashBlockAlignInit
	mov	r1, r5
	mov	r2, #8
	add	r0, r4, #1616
	bl	ftl_memset
	mov	r1, r5
	mov	r2, #32
	ldr	r0, .L132+12
	bl	ftl_memset
	ldr	r0, .L132+16
	mov	r1, r5
	mov	r2, #128
	bl	ftl_memset
	ldr	r7, [r4, #1784]
	add	r6, r7, #1
.L119:
	mov	r0, r6
	add	r1, r9, r5, asl #3
	ldrb	r2, [r7]	@ zero_extendqisi2
	bl	FlashMemCmp8
	ldr	r8, .L132+8
	cmp	r0, #0
	bne	.L118
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	add	r2, r4, r3, asl #2
	str	r0, [r2, #1624]
	add	r2, r3, #1
	add	r3, r4, r3
	strb	r2, [r4, #1612]
	strb	r5, [r3, #1616]
.L118:
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L119
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	strb	r3, [r4, #1613]
	ldrb	r3, [r7, #8]	@ zero_extendqisi2
	cmp	r3, #2
	beq	.L120
.L124:
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	ldrb	r2, [r4, #1612]	@ zero_extendqisi2
	smulbb	r2, r2, r3
	ldrh	r3, [r7, #14]
	smulbb	r3, r2, r3
	movw	r2, #1788
	strh	r3, [r4, r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L120:
	ldr	r3, .L132+4
	mov	r5, #0
	ldr	r9, [r3, #4]
.L123:
	mov	r0, r6
	add	r1, r8, r5, asl #3
	ldrb	r2, [r7]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L121
	ldrb	r1, [r7, #13]	@ zero_extendqisi2
	ldrh	r3, [r7, #14]
	ldrb	r2, [r4, #1612]	@ zero_extendqisi2
	mul	r1, r9, r1
	and	r3, r3, #65280
	add	r0, r4, r2, asl #2
	mul	r3, r3, r1
	str	r3, [r0, #1624]
	ldrb	r1, [r7, #23]	@ zero_extendqisi2
	cmp	r1, #0
	movne	r3, r3, asl #1
	strne	r3, [r0, #1624]
	add	r3, r2, #1
	add	r2, r4, r2
	strb	r3, [r4, #1612]
	strb	r5, [r2, #1616]
.L121:
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L123
	b	.L124
.L133:
	.align	2
.L132:
	.word	.LANCHOR1+256
	.word	.LANCHOR0
	.word	IDByte
	.word	.LANCHOR0+1624
	.word	.LANCHOR0+1656
	.fnend
	.size	FlashDieInfoInit, .-FlashDieInfoInit
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
	ldr	r1, .L136
	bl	ftl_memcpy
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L137:
	.align	2
.L136:
	.word	.LANCHOR0+1792
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
	mov	r6, #0
	ldr	r8, .L151
.L142:
	mov	r4, r6, asl #5
	ldr	r1, .L151+4
	add	r0, r4, #1
	ldrb	r2, [r8, r6, asl #5]	@ zero_extendqisi2
	add	r0, r8, r0
	ldr	r5, .L151+8
	bl	FlashMemCmp8
	add	r9, r5, #288
	subs	r7, r0, #0
	bne	.L139
	adds	r9, r9, r4
	beq	.L145
	add	r4, r5, r4
	add	r5, r5, #2880
	ldrb	r3, [r4, #310]	@ zero_extendqisi2
	mov	r4, r7
	b	.L144
.L139:
	add	r6, r6, #1
	cmp	r6, #81
	bne	.L142
	b	.L145
.L150:
	add	r4, r4, #1
	cmp	r4, #4
	beq	.L143
.L144:
	ldrb	r2, [r5, r4, asl #5]	@ zero_extendqisi2
	cmp	r2, r3
	bne	.L150
.L143:
	ldr	r6, .L151+12
	mov	r2, #32
	ldr	r1, .L151+16
	ldr	r0, .L151+20
	add	r1, r1, r4, asl #5
	bl	ftl_memcpy
	mov	r0, r6
	mov	r1, r9
	mov	r2, #32
	bl	ftl_memcpy
	ldrh	r0, [r6, #10]
	bl	FlashBlockAlignInit
	b	.L140
.L145:
	mvn	r7, #0
.L140:
	mov	r0, r7
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L152:
	.align	2
.L151:
	.word	.LANCHOR1+288
	.word	IDByte
	.word	.LANCHOR1
	.word	.LANCHOR1+256
	.word	.LANCHOR1+2880
	.word	.LANCHOR0+44
	.fnend
	.size	FlashLoadPhyInfoInRam, .-FlashLoadPhyInfoInRam
	.align	2
	.global	ftl_flash_suspend
	.type	ftl_flash_suspend, %function
ftl_flash_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L154
	ldr	r2, [r3, #3840]
	ldr	r1, [r2]
	str	r1, [r3, #3844]
	ldr	r1, [r2, #4]
	str	r1, [r3, #3848]
	ldr	r1, [r2, #8]
	str	r1, [r3, #3852]
	ldr	r1, [r2, #12]
	str	r1, [r3, #3856]
	ldr	r1, [r2, #304]
	str	r1, [r3, #3860]
	ldr	r1, [r2, #308]
	str	r1, [r3, #3864]
	ldr	r1, [r2, #336]
	ldr	r2, [r2, #344]
	str	r1, [r3, #3868]
	str	r2, [r3, #3872]
	bx	lr
.L155:
	.align	2
.L154:
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
	ldr	r3, .L162
	mov	r10, r1
	ldr	r7, .L162+4
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
	bne	.L158
	ldrb	r3, [r7, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L162+4
	addeq	r4, r3, r4, asl #1
	ldreqh	r4, [r4, #76]
.L158:
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
.L163:
	.align	2
.L162:
	.word	.LANCHOR0+3876
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
	mov	r1, #0
	mov	r2, #11
	mov	r4, r0
	bl	ftl_memset
	ldr	r3, .L169
	mov	ip, #1
	add	r3, r3, #1616
	ldr	r2, [r3, #168]
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	ldr	r2, [r3, #-1612]
	smulbb	r2, r1, r2
	strh	r2, [r4, #4]	@ unaligned
	ldrb	r2, [r3, #2289]	@ zero_extendqisi2
	strb	r2, [r4, #7]
	ldr	r2, [r3, #2292]
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
.L165:
	uxtb	r1, r2
	cmp	r1, r0
	bcs	.L168
	ldrb	lr, [r2, r3]	@ zero_extendqisi2
	add	r2, r2, #1
	ldrb	r1, [r4, #10]	@ zero_extendqisi2
	orr	r1, r1, ip, asl lr
	strb	r1, [r4, #10]
	b	.L165
.L168:
	ldmfd	sp!, {r4, pc}
.L170:
	.align	2
.L169:
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
	ldr	r3, .L172
	ldr	r2, [r3, #3912]
	str	r0, [r3, #3912]
	mov	r0, r2
	bx	lr
.L173:
	.align	2
.L172:
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
	ldr	r3, .L175
	ldr	r3, [r3, #1784]
	ldrb	r0, [r3, #9]	@ zero_extendqisi2
	bx	lr
.L176:
	.align	2
.L175:
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
	ldr	r3, .L179
	add	r0, r3, r0, asl #3
	ldrb	r2, [r0, #16]	@ zero_extendqisi2
	ldr	r0, [r0, #12]
	add	r2, r2, #8
	add	r0, r0, r2, asl #8
	bx	lr
.L180:
	.align	2
.L179:
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
	ldr	r3, .L182
	ldr	r2, [r3, #3840]
	mov	r3, r0, asl #8
	orr	r0, r3, r0, asl #16
	orr	r3, r0, #1
	str	r3, [r2, #304]
	bx	lr
.L183:
	.align	2
.L182:
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
	ldr	r3, .L185
	orr	r0, r0, #16640
	ldr	r3, [r3, #3840]
	str	r0, [r3, #344]
	bx	lr
.L186:
	.align	2
.L185:
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
	ldr	r3, .L190
	cmp	r0, #0
	ldr	r2, [r3, #3840]
	ldr	r3, [r2]
	bfieq	r3, r0, #13, #1
	orrne	r3, r3, #253952
	str	r3, [r2]
	bx	lr
.L191:
	.align	2
.L190:
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
	ldr	r3, .L199
	ands	r1, r0, #6
	ldr	r2, [r3, #3840]
	ldr	r3, [r2]
	bfieq	r3, r1, #13, #1
	beq	.L195
	orr	r3, r3, #24576
	movw	r1, #8322
	bfc	r3, #15, #1
	str	r1, [r2, #344]
	orr	r3, r3, #196608
	ldr	r1, .L199+4
	tst	r0, #4
	orrne	r3, r3, #32768
	str	r1, [r2, #304]
	mov	r1, #38
	str	r1, [r2, #308]
	mov	r1, #39
	str	r1, [r2, #308]
.L195:
	str	r3, [r2]
	mov	r0, #0
	bx	lr
.L200:
	.align	2
.L199:
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
	ldr	r2, .L202
	add	r0, r2, r0, asl #3
	mov	r2, #1
	ldr	r1, [r0, #12]
	ldrb	r0, [r0, #16]	@ zero_extendqisi2
	ldr	r3, [r1]
	mov	r2, r2, asl r0
	bfi	r3, r2, #0, #8
	str	r3, [r1]
	bx	lr
.L203:
	.align	2
.L202:
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
	ldr	r3, .L205
	add	r0, r3, r0, asl #3
	ldr	r2, [r0, #12]
	ldr	r3, [r2]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2]
	bx	lr
.L206:
	.align	2
.L205:
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
	ldr	r3, .L209
	add	r0, r0, #3
	umull	r0, r1, r0, r3
	ldr	r3, .L209+4
	ldr	r3, [r3, #8]
	mov	r0, r1, lsr #6
	blx	r3
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L210:
	.align	2
.L209:
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
	ldr	r3, .L213
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
.L214:
	.align	2
.L213:
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
	add	r7, r1, r1, asl #2
	ldr	r8, .L225
	mov	r5, r0
	ldr	r10, .L225+4
	mov	r6, r1
	add	r9, r8, #3008
	mov	r4, #0
.L216:
	ldrb	r3, [r10]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L224
	mov	r3, #85
	str	r3, [r5, #8]
	ldrsb	r3, [r4, r9]
	mov	r0, #200
	str	r3, [r5, #4]
	bl	NandcDelayns
	ldr	r3, .L225+8
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #34
	addeq	r3, r4, r7
	addeq	r3, r9, r3
	beq	.L223
	cmp	r3, #35
	addne	r3, r8, r6
	addne	r3, r3, #3152
	ldrnesb	r3, [r3]
	bne	.L222
	ldr	r3, .L225+12
	add	r2, r4, r7
	add	r3, r3, r2
.L223:
	ldrsb	r3, [r3, #5]
.L222:
	str	r3, [r5]
	add	r4, r4, #1
	b	.L216
.L224:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L226:
	.align	2
.L225:
	.word	.LANCHOR1
	.word	g_maxRegNum
	.word	g_retryMode
	.word	.LANCHOR1+3056
	.fnend
	.size	ToshibaSetRRPara, .-ToshibaSetRRPara
	.align	2
	.global	SamsungSetRRPara
	.type	SamsungSetRRPara, %function
SamsungSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L232
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r1, r3, r1, asl #2
	ldr	r8, .L232+4
	mov	r4, #0
	add	r5, r1, #3
	mov	r6, r0
	mov	r7, r3
	mov	r9, #161
	mov	r10, r4
.L228:
	ldrb	r3, [r8]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L231
	str	r9, [r6, #8]
	mov	r0, #300
	str	r10, [r6]
	ldrsb	r3, [r4, r7]
	add	r4, r4, #1
	str	r3, [r6]
	ldrsb	r3, [r5, #1]!
	str	r3, [r6]
	bl	NandcDelayns
	b	.L228
.L231:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L233:
	.align	2
.L232:
	.word	.LANCHOR1+3160
	.word	g_maxRegNum
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
	mov	r5, r3
	ldr	r3, .L243
	mov	r10, r2
	ldr	r4, .L243+4
	mov	r6, r0
	mov	r9, r1
	ldr	r2, [r3, #1784]
	ldrb	r8, [r2, #19]	@ zero_extendqisi2
	mov	r2, r0, asl #3
	cmp	r8, #6
	addeq	r4, r4, r0, asl #6
	addeq	r4, r4, #20
	addeq	r4, r4, r5, asl #2
	beq	.L236
	cmp	r8, #7
	bne	.L237
	mov	r8, #160
	add	r1, r5, r5, asl #2
	mla	r4, r8, r0, r4
	add	r4, r4, #28
	add	r4, r4, r1, asl #1
	b	.L236
.L237:
	cmp	r8, #8
	addne	r1, r5, r2
	addeq	r1, r5, r5, asl #2
	ldreq	r4, .L243+8
	addne	r4, r4, r1, asl #3
	addeq	r4, r4, r1
	addne	r4, r4, #20
.L236:
	add	r3, r3, r2
	mov	r0, r6
	sub	r9, r9, #1
	sub	r4, r4, #1
	ldrb	fp, [r3, #16]	@ zero_extendqisi2
	ldr	r8, [r3, #12]
	bl	NandcFlashCs
	sub	r2, r10, #1
	add	r10, r10, r9
	mov	fp, fp, asl #8
	mov	r3, #54
	add	r7, r8, fp
	str	r3, [r7, #2056]
.L239:
	cmp	r2, r10
	beq	.L242
	ldrb	r3, [r2, #1]!	@ zero_extendqisi2
	mov	r0, #200
	str	r3, [r7, #2052]
	str	r2, [sp, #4]
	bl	NandcDelayns
	ldrsb	r3, [r4, #1]!
	str	r3, [r7, #2048]
	ldr	r2, [sp, #4]
	b	.L239
.L242:
	add	r8, r8, fp
	mov	r3, #22
	mov	r0, r6
	str	r3, [r8, #2056]
	bl	NandcFlashDeCs
	ldr	r3, .L243+12
	add	r6, r3, r6
	strb	r5, [r6, #-3416]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L244:
	.align	2
.L243:
	.word	.LANCHOR0
	.word	.LANCHOR0+3916
	.word	.LANCHOR0+3944
	.word	.LANCHOR2
	.fnend
	.size	HynixSetRRPara, .-HynixSetRRPara
	.align	2
	.global	FlashSetReadRetryDefault
	.type	FlashSetReadRetryDefault, %function
FlashSetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L253
	ldr	r2, [r3, #1784]
	ldrb	r2, [r2, #19]	@ zero_extendqisi2
	sub	r2, r2, #1
	cmp	r2, #7
	bxhi	lr
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r3
	ldr	r6, .L253+4
	mov	r4, #0
.L246:
	ldrb	r3, [r6, r4, asl #3]	@ zero_extendqisi2
	uxtb	r0, r4
	cmp	r3, #173
	bne	.L247
	ldrb	r1, [r5, #3917]	@ zero_extendqisi2
	mov	r3, #0
	ldr	r2, .L253+8
	bl	HynixSetRRPara
.L247:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L246
	ldmfd	sp!, {r4, r5, r6, pc}
.L254:
	.align	2
.L253:
	.word	.LANCHOR0
	.word	IDByte
	.word	.LANCHOR0+3920
	.fnend
	.size	FlashSetReadRetryDefault, .-FlashSetReadRetryDefault
	.align	2
	.global	FlashReadStatusEN
	.type	FlashReadStatusEN, %function
FlashReadStatusEN:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L266
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
	bne	.L260
	cmp	r2, #0
	add	r3, r4, r3
	ldrneb	r2, [ip, #58]	@ zero_extendqisi2
	ldreqb	r2, [ip, #57]	@ zero_extendqisi2
	str	r2, [r3, #2056]
	ldrb	r0, [ip, #59]	@ zero_extendqisi2
	cmp	r0, #0
	addne	ip, r4, r5, asl #8
	movne	r2, #0
	beq	.L260
.L259:
	cmp	r2, r0
	bcs	.L260
	mov	r3, r2, asl #3
	add	r2, r2, #1
	mov	r3, r1, lsr r3
	uxtb	r3, r3
	str	r3, [ip, #4]
	b	.L259
.L260:
	mov	r0, #80
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	uxtb	r0, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L267:
	.align	2
.L266:
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
.L272:
	mov	r0, r4
	mov	r1, r5
	mov	r2, r6
	bl	FlashReadStatusEN
	cmp	r0, #255
	beq	.L272
	tst	r0, #64
	beq	.L272
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
	ldr	r5, .L284
	add	r4, r5, r0, asl #4
	ldr	r3, [r4, #1664]
	ldrb	r7, [r4, #1656]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L278
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
.L278:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L285:
	.align	2
.L284:
	.word	.LANCHOR0
	.fnend
	.size	FlashWaitCmdDone, .-FlashWaitCmdDone
	.align	2
	.type	flash_read_ecc, %function
flash_read_ecc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L288
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
.L289:
	.align	2
.L288:
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
	ldr	r3, .L296
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	add	r0, r3, r0, asl #3
	ldr	r4, .L296+4
	ldr	r5, [r0, #12]
.L292:
	ldr	r3, [r5]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L293
	mov	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L292
	mvn	r0, #0
	b	.L291
.L293:
	mov	r0, #0
.L291:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L297:
	.align	2
.L296:
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
	ldr	r3, .L304
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	add	r0, r3, r0, asl #3
	ldr	r4, .L304+4
	ldr	r5, [r0, #12]
	mov	r0, #150
	bl	NandcDelayns
.L300:
	ldr	r3, [r5]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L301
	mov	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L300
	mvn	r0, #0
	b	.L299
.L301:
	mov	r0, #0
.L299:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L305:
	.align	2
.L304:
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
	ldr	r3, .L308
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
.L309:
	.align	2
.L308:
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
	ldr	r4, .L313
	ldrb	r3, [r4, #3904]	@ zero_extendqisi2
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
.L314:
	.align	2
.L313:
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
	ldr	r4, .L318
	ldrb	r3, [r4, #3904]	@ zero_extendqisi2
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
.L319:
	.align	2
.L318:
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
	ldr	r3, .L345
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	lr, #0
	ldrb	r2, [r3, #-3412]	@ zero_extendqisi2
	mov	r4, #239
	ldr	r7, .L345+4
	mov	r5, #128
	and	r3, r2, #1
	and	r2, r2, #4
	str	r3, [sp, #4]
	mov	r6, #1
	uxtb	r3, r2
	mov	r8, #35
	mov	r2, lr
	mov	r9, #32
	mov	r10, #5
	str	r3, [sp]
.L332:
	ldr	r3, .L345+8
	ldrb	ip, [lr, r7]	@ zero_extendqisi2
	ldr	r1, [r3, lr]!
	cmp	ip, #69
	cmpne	ip, #152
	ldrb	r3, [r3, #4]	@ zero_extendqisi2
	beq	.L323
	cmp	ip, #44
	cmpne	ip, #173
	bne	.L324
.L323:
	cmp	r0, #1
	bne	.L325
	ldr	fp, [sp, #4]
	cmp	fp, #0
	beq	.L324
	mov	r3, r3, asl #8
	cmp	ip, #173
	add	fp, r1, r3
	str	r4, [fp, #2056]
	streq	r0, [fp, #2052]
	beq	.L344
	cmp	ip, #44
	streq	r0, [fp, #2052]
	strne	r5, [fp, #2052]
	streq	r10, [fp, #2048]
	strne	r0, [fp, #2048]
	b	.L330
.L325:
	ldr	fp, [sp]
	cmp	fp, #0
	beq	.L324
	mov	r3, r3, asl #8
	cmp	ip, #173
	add	fp, r1, r3
	str	r4, [fp, #2056]
	streq	r6, [fp, #2052]
	streq	r9, [fp, #2048]
	beq	.L330
	cmp	ip, #44
	streq	r6, [fp, #2052]
	streq	r8, [fp, #2048]
	beq	.L330
	str	r5, [fp, #2052]
.L344:
	str	r2, [fp, #2048]
.L330:
	add	r3, r1, r3
	str	r2, [r3, #2048]
	str	r2, [r3, #2048]
	str	r2, [r3, #2048]
.L324:
	add	lr, lr, #8
	cmp	lr, #32
	bne	.L332
	mov	r0, #0
	bl	NandcWaitFlashReady
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L346:
	.align	2
.L345:
	.word	.LANCHOR2
	.word	IDByte
	.word	.LANCHOR0+12
	.fnend
	.size	FlashSetInterfaceMode, .-FlashSetInterfaceMode
	.align	2
	.global	FlashReadSpare
	.type	FlashReadSpare, %function
FlashReadSpare:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L349
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	add	ip, ip, r0, asl #3
	ldr	r3, .L349+4
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
.L350:
	.align	2
.L349:
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
	ldr	r2, .L353
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
.L354:
	.align	2
.L353:
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
	ldr	r1, .L363
	ldr	r0, .L363+4
	add	r4, r4, r4, asl #2
	ldr	ip, .L363+8
	sub	lr, r1, #48
	mov	r2, #0
.L356:
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r2, r3
	bcs	.L362
	ldrb	r3, [ip]	@ zero_extendqisi2
	cmp	r3, #67
	add	r3, r2, r4
	addeq	r3, lr, r3
	addne	r3, r1, r3
	add	r2, r2, #1
	ldrsb	r3, [r3, #5]
	str	r3, [r5]
	b	.L356
.L362:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, lr}
	b	NandcWaitFlashReady
.L364:
	.align	2
.L363:
	.word	.LANCHOR1+3056
	.word	g_maxRegNum
	.word	g_retryMode
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
	ldr	r3, .L367
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
.L368:
	.align	2
.L367:
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
	ldr	r8, .L381
	.pad #20
	sub	sp, sp, #20
	mov	r6, r0
	mov	r9, r1
	mov	r7, r5
.L370:
	cmp	r7, r9
	beq	.L380
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
	bcs	.L372
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
	bne	.L374
	ldr	r0, .L381+4
	ldr	r1, [sp, #8]
	bl	printk
.L374:
	mov	r0, r4
	bl	NandcFlashDeCs
.L372:
	add	r7, r7, #1
	add	r5, r5, #36
	b	.L370
.L380:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L382:
	.align	2
.L381:
	.word	.LANCHOR0
	.word	.LC1
	.fnend
	.size	FlashEraseSLc2KBlocks, .-FlashEraseSLc2KBlocks
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
	ldr	r7, .L492
	mvn	r3, #83
	mvn	r1, #82
	mvn	r2, #81
	.pad #44
	sub	sp, sp, #44
	mov	r4, r0
	strb	r3, [r7, #3920]
	mvn	r3, #80
	strb	r0, [r7, #3916]
	strb	r1, [r7, #3921]
	strb	r2, [r7, #3922]
	strb	r3, [r7, #3923]
	bne	.L384
	mvn	r3, #88
	strb	r3, [r7, #3920]
	ldr	r3, .L492+4
	mvn	r2, #8
	mov	r5, #7
	strb	r2, [r3, #3241]
	b	.L445
.L384:
	cmp	r0, #3
	bne	.L386
	mvn	r3, #79
	strb	r3, [r7, #3920]
	mvn	r3, #78
	strb	r3, [r7, #3921]
	mvn	r3, #77
	strb	r3, [r7, #3922]
	mvn	r3, #76
	strb	r3, [r7, #3923]
	mvn	r3, #75
	strb	r3, [r7, #3924]
	mvn	r3, #74
	strb	r3, [r7, #3925]
	mvn	r3, #73
	strb	r3, [r7, #3926]
	mvn	r3, #72
	b	.L486
.L386:
	cmp	r0, #4
	bne	.L387
	mvn	r0, #51
	strb	r1, [r7, #3925]
	strb	r0, [r7, #3920]
	mvn	r0, #64
	strb	r2, [r7, #3926]
	strb	r0, [r7, #3921]
	mvn	r0, #85
	strb	r0, [r7, #3922]
	mvn	r0, #84
	strb	r0, [r7, #3923]
	mvn	r0, #50
	strb	r0, [r7, #3924]
.L486:
	mov	r5, #8
	strb	r3, [r7, #3927]
	mov	r6, r5
	b	.L385
.L387:
	cmp	r0, #5
	bne	.L388
	mov	r3, #56
	mov	r5, #8
	strb	r3, [r7, #3920]
	mov	r3, #57
	strb	r3, [r7, #3921]
	mov	r3, #58
	strb	r3, [r7, #3922]
	mov	r3, #59
	strb	r3, [r7, #3923]
	b	.L445
.L388:
	cmp	r0, #6
	bne	.L389
	mov	r3, #14
	mov	r5, #12
	strb	r3, [r7, #3920]
	mov	r3, #15
	strb	r3, [r7, #3921]
	mov	r3, #16
	strb	r3, [r7, #3922]
	mov	r3, #17
	strb	r3, [r7, #3923]
	b	.L445
.L389:
	cmp	r0, #7
	bne	.L390
	mvn	r3, #79
	mov	r5, #12
	strb	r3, [r7, #3920]
	mov	r6, #10
	mvn	r3, #78
	strb	r3, [r7, #3921]
	mvn	r3, #77
	strb	r3, [r7, #3922]
	mvn	r3, #76
	strb	r3, [r7, #3923]
	mvn	r3, #75
	strb	r3, [r7, #3924]
	mvn	r3, #74
	strb	r3, [r7, #3925]
	mvn	r3, #73
	strb	r3, [r7, #3926]
	mvn	r3, #72
	strb	r3, [r7, #3927]
	mvn	r3, #43
	strb	r3, [r7, #3928]
	mvn	r3, #42
	strb	r3, [r7, #3929]
	b	.L385
.L390:
	cmp	r0, #8
	mov	r5, #7
	bne	.L445
	mov	r3, #6
	strb	r5, [r7, #3921]
	strb	r3, [r7, #3920]
	mov	r5, #50
	mov	r3, #9
	strb	r0, [r7, #3922]
	strb	r3, [r7, #3923]
	mov	r6, #5
	mov	r3, #10
	strb	r3, [r7, #3924]
	b	.L385
.L445:
	mov	r6, #4
.L385:
	sub	r3, r4, #1
	cmp	r3, #1
	bhi	.L482
	ldr	r10, .L492+8
	mov	r9, #0
.L391:
	ldrb	r1, [r7, #1612]	@ zero_extendqisi2
	uxtb	r2, r9
	ldr	r3, .L492
	cmp	r1, r2
	bls	.L398
	add	r2, r3, r2
	ldr	r4, .L492+12
	ldr	r1, .L492+16
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
.L393:
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
	bcc	.L393
	mov	r2, #0
.L394:
	add	r0, r10, r2
	mov	r3, #1
.L395:
	ldrb	r1, [r0, r3, asl #2]	@ zero_extendqisi2
	ldrb	ip, [fp]	@ zero_extendqisi2
	add	r1, r1, ip
	strb	r1, [fp, r3, asl #3]
	add	r3, r3, #1
	cmp	r3, #7
	bne	.L395
	add	r2, r2, #1
	add	fp, fp, #1
	cmp	r2, #4
	bne	.L394
	mov	r3, #0
	add	r9, r9, #1
	strb	r3, [r4, #16]
	strb	r3, [r4, #24]
	strb	r3, [r4, #32]
	strb	r3, [r4, #40]
	strb	r3, [r4, #48]
	strb	r3, [r4, #41]
	strb	r3, [r4, #49]
	b	.L391
.L482:
	sub	r3, r4, #3
	cmp	r3, #5
	bhi	.L398
	smulbb	r2, r6, r5
	mov	r3, r2, asl #4
	mov	r2, r2, asr #1
	str	r3, [sp, #36]
	mov	r3, r2, asl #1
	str	r3, [sp, #8]
	mov	r3, #0
.L490:
	str	r3, [sp, #16]
	ldrb	r3, [sp, #16]	@ zero_extendqisi2
	str	r3, [sp, #12]
	ldr	r3, .L492
	ldr	r2, [sp, #12]
	ldrb	r3, [r3, #1612]	@ zero_extendqisi2
	cmp	r3, r2
	bhi	.L443
.L398:
	ldr	r3, .L492
	strb	r6, [r3, #3917]
	strb	r5, [r3, #3918]
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L443:
	ldr	r2, [sp, #12]
	ldr	r3, .L492
	add	r3, r3, r2
	ldrb	r10, [r3, #1616]	@ zero_extendqisi2
	ldr	r3, .L492
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
	ldreq	r3, .L492+12
	moveq	r7, #160
	mlaeq	r7, r7, r10, r3
	addeq	r3, r7, #28
	beq	.L487
.L400:
	cmp	r4, #8
	beq	.L402
	ldr	r3, .L492+12
	add	r7, r3, r10, asl #6
	add	r3, r7, #20
.L487:
	str	r3, [sp, #20]
	cmp	r4, #4
	add	r3, r8, fp, asl #8
	mov	r2, #54
	str	r2, [r3, #2056]
	bne	.L403
	add	r9, r8, r9
	mov	r3, #255
	str	r3, [r9, #2052]
	mov	r3, #64
	str	r3, [r9, #2048]
	mov	r3, #204
	b	.L488
.L403:
	sub	r2, r4, #5
	cmp	r2, #1
	bhi	.L405
	ldr	r2, .L492
	ldrb	r2, [r2, #3920]	@ zero_extendqisi2
	str	r2, [r3, #2052]
	mov	r2, #82
	str	r2, [r3, #2048]
	b	.L404
.L405:
	cmp	r4, #7
	bne	.L404
	add	r9, r8, r9
	mov	r3, #174
	str	r3, [r9, #2052]
	mov	r3, #0
	str	r3, [r9, #2048]
	mov	r3, #176
.L488:
	str	r3, [r9, #2052]
	mov	r3, #77
	str	r3, [r9, #2048]
.L404:
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
.L444:
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
	bne	.L408
	cmp	r4, #7
	movne	ip, #2
	moveq	ip, #32
.L408:
	ldr	r3, .L492+20
	add	r0, r8, fp, asl #8
	ldr	r3, [r3, #-3408]
	mov	r1, r3
.L409:
	ldr	r2, [r0, #2048]
	strb	r2, [r1], #1
	rsb	r2, r3, r1
	uxtb	r2, r2
	cmp	r2, ip
	bcc	.L409
	ldr	r2, [sp, #4]
	cmp	r2, #0
	beq	.L410
	mov	r2, #0
.L412:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r1, r2
	cmp	ip, #50
	beq	.L411
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #5
	beq	.L411
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L412
	b	.L413
.L411:
	cmp	r1, #6
	bls	.L414
.L413:
	ldr	r0, .L492+24
	mov	r1, #0
	bl	printk
.L415:
	b	.L415
.L410:
	cmp	r4, #7
	bne	.L416
	ldr	r2, [sp, #4]
.L418:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r1, r2
	cmp	ip, #12
	beq	.L417
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #10
	beq	.L417
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L418
	b	.L419
.L417:
	cmp	r1, #7
	bne	.L414
.L419:
	ldr	r0, .L492+24
	mov	r1, #0
	bl	printk
.L420:
	b	.L420
.L416:
	cmp	r4, #6
	bne	.L414
	sub	r2, r3, #1
	add	r3, r3, #7
.L421:
	ldrb	r1, [r2, #1]!	@ zero_extendqisi2
	cmp	r1, #12
	beq	.L414
	ldrb	r1, [r2, #8]	@ zero_extendqisi2
	cmp	r1, #4
	beq	.L414
	cmp	r2, r3
	bne	.L421
	ldr	r0, .L492+24
	mov	r1, #0
	bl	printk
.L423:
	b	.L423
.L414:
	ldr	r1, .L492+20
	ldr	ip, [sp, #36]
	ldr	r2, [r1, #-3408]
	add	ip, r2, ip
	mov	r3, r2
.L424:
	cmp	r3, ip
	ldrne	lr, [r0, #2048]
	strneb	lr, [r3], #1
	bne	.L424
.L491:
	ldr	r3, [r1, #-3408]
	mov	r0, #8
	ldr	r1, [sp, #8]
	add	r1, r3, r1
	str	r1, [sp, #28]
.L427:
	ldr	ip, [sp, #8]
	add	lr, r1, ip
.L426:
	ldrh	ip, [r1]
	mvn	ip, ip
	strh	ip, [r1], #2	@ movhi
	cmp	r1, lr
	bne	.L426
	ldr	ip, [sp, #8]
	subs	r0, r0, #1
	add	r1, r1, ip
	bne	.L427
.L428:
	mov	ip, #0
	mov	r0, ip
.L431:
	mov	r1, #1
	mov	lr, #0
	mov	r1, r1, asl r0
	mov	r7, #16
	str	r7, [sp, #32]
	mov	r7, lr
.L429:
	ldrh	r9, [r3, lr]
	and	r9, r9, r1
	cmp	r9, r1
	addeq	r7, r7, #1
	ldr	r9, [sp, #8]
	add	lr, lr, r9
	ldr	r9, [sp, #32]
	subs	r9, r9, #1
	str	r9, [sp, #32]
	bne	.L429
	cmp	r7, #8
	add	r0, r0, #1
	orrhi	ip, ip, r1
	uxthhi	ip, ip
	cmp	r0, #16
	bne	.L431
	ldr	r1, [sp, #28]
	strh	ip, [r3], #2	@ movhi
	cmp	r3, r1
	bne	.L428
	ldr	r3, .L492+20
	ldr	r1, [r3, #-3408]
	mov	r3, #0
	sub	r0, r1, #4
	add	ip, r1, #28
.L433:
	ldr	lr, [r0, #4]!
	cmp	lr, #0
	addeq	r3, r3, #1
	cmp	r0, ip
	bne	.L433
	cmp	r3, #7
	ble	.L434
	ldr	r0, .L492+28
	mov	r2, #1
	mov	r3, #1024
	bl	rknand_print_hex
	ldr	r0, .L492+24
	mov	r1, #0
	bl	printk
.L435:
	b	.L435
.L434:
	cmp	r4, #6
	moveq	r0, #4
	beq	.L436
	cmp	r4, #7
	moveq	r0, #10
	beq	.L436
	ldr	r3, [sp, #4]
	cmp	r3, #0
	moveq	r0, #8
	movne	r0, #5
.L436:
	sub	r9, r6, #1
	ldr	r1, [sp, #20]
	mov	ip, #0
	uxtb	r9, r9
	add	r9, r9, #1
.L437:
	mov	r7, r1
	mov	r3, r2
.L438:
	ldrb	lr, [r3], #1	@ zero_extendqisi2
	strb	lr, [r7], #1
	rsb	lr, r2, r3
	uxtb	lr, lr
	cmp	lr, r6
	bcc	.L438
	add	ip, ip, #1
	add	r2, r2, r9
	cmp	ip, r5
	add	r1, r1, r0
	blt	.L437
	add	r7, r8, fp, asl #8
	mov	r3, #255
	mov	r0, r10
	str	r3, [r7, #2056]
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #24]
	cmp	r3, #0
	beq	.L440
	mov	r3, #54
	str	r3, [r7, #2056]
	ldr	r3, .L492
	mvn	r1, #0
	ldr	r0, [sp, #12]
	ldrb	r3, [r3, #3920]	@ zero_extendqisi2
	str	r3, [r7, #2052]
	mov	r3, #0
	str	r3, [r7, #2048]
	mov	r3, #22
	str	r3, [r7, #2056]
	bl	FlashReadCmd
	b	.L441
.L440:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	movne	r3, #190
	moveq	r3, #56
	str	r3, [r7, #2056]
.L441:
	mov	r0, r10
	bl	NandcWaitFlashReady
	ldr	r3, [sp, #16]
	add	r3, r3, #1
	b	.L490
.L402:
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
	ldr	r3, .L492+32
	str	r3, [sp, #20]
	b	.L444
.L493:
	.align	2
.L492:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR1+3224
	.word	.LANCHOR0+3916
	.word	.LANCHOR0+3919
	.word	.LANCHOR2
	.word	.LC2
	.word	.LC3
	.word	.LANCHOR0+3944
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
	bhi	.L496
	b	HynixGetReadRetryDefault
.L496:
	cmp	r3, #49
	bne	.L497
	ldr	r0, .L508
	mov	r2, #64
	ldr	r1, .L508+4
	add	r0, r0, #3920
	strb	r3, [r0, #-4]
	mov	r3, #4
	strb	r3, [r0, #-3]
	mov	r3, #15
	strb	r3, [r0, #-2]
	b	.L506
.L497:
	sub	r2, r3, #65
	cmp	r3, #33
	cmpne	r2, #1
	ldrls	r0, .L508
	strlsb	r3, [r0, #3916]
	movls	r3, #4
	bls	.L507
.L498:
	cmp	r3, #34
	cmpne	r3, #67
	bne	.L499
	ldr	r0, .L508
	strb	r3, [r0, #3916]
	mov	r3, #5
.L507:
	strb	r3, [r0, #3917]
	mov	r2, #45
	mov	r3, #7
	add	r0, r0, #3920
	strb	r3, [r0, #-2]
	ldr	r1, .L508+8
	b	.L506
.L499:
	cmp	r3, #35
	cmpne	r3, #68
	bxne	lr
	ldr	r0, .L508
	mov	r2, #95
	ldr	r1, .L508+12
	add	r0, r0, #3920
	strb	r3, [r0, #-4]
	mov	r3, #5
	strb	r3, [r0, #-3]
	mov	r3, #17
	strb	r3, [r0, #-2]
.L506:
	b	ftl_memcpy
.L509:
	.align	2
.L508:
	.word	.LANCHOR0
	.word	.LANCHOR1+3160
	.word	.LANCHOR1+3008
	.word	.LANCHOR1+3056
	.fnend
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
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
	ldr	r3, .L515
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
	bne	.L511
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
	b	.L514
.L511:
	str	lr, [r4, #2052]
	str	ip, [r4, #2052]
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
.L514:
	str	r9, [r4, #2052]
	mov	r3, #48
	str	r8, [r4, #2052]
	mov	r0, r7
	str	r5, [r4, #2052]
	mov	r1, r6
	str	r3, [r4, #2056]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	FlashSetRandomizer
.L516:
	.align	2
.L515:
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
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	ldr	r3, .L528
	ldr	r0, [r3, #-3404]
	mov	r4, r3
	cmp	r0, #0
	beq	.L518
	mov	r0, #0
	bl	flash_enter_slc_mode
	b	.L519
.L518:
	bl	flash_exit_slc_mode
.L519:
	ldrb	r3, [r4, #-3400]	@ zero_extendqisi2
	ldr	r5, .L528
	cmp	r3, #0
	beq	.L520
	ldrb	r3, [r5, #-3412]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L520
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r3, #0
	strb	r3, [r5, #-3400]
.L520:
	ldr	r3, .L528+4
	mov	r0, #0
	ldr	r3, [r3, #12]
	str	r0, [r3, #336]
	ldmfd	sp!, {r3, r4, r5, pc}
.L529:
	.align	2
.L528:
	.word	.LANCHOR2
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
	ldr	r3, .L531
	add	r0, r3, r0, asl #3
	ldr	r3, [r0, #12]
	str	r1, [r3, #336]
	bx	lr
.L532:
	.align	2
.L531:
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
	ldr	r1, .L544
	bl	__aeabi_idiv
	ldr	r3, .L544+4
	ldr	r3, [r3, #3840]
	cmp	r0, #250
	movwgt	r2, #8354
	bgt	.L542
	cmp	r0, #220
	bgt	.L543
	cmp	r0, #185
	movwgt	r2, #4226
	bgt	.L542
	cmp	r0, #160
	movwgt	r2, #4194
	bgt	.L542
	cmp	r4, #35
	movwls	r2, #4193
	bls	.L542
	cmp	r4, #99
	movwls	r2, #4225
	bls	.L542
.L543:
	movw	r2, #8322
.L542:
	str	r2, [r3, #4]
	ldmfd	sp!, {r4, pc}
.L545:
	.align	2
.L544:
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
	bls	.L547
	movw	r3, #8322
	cmp	r0, r3
	bne	.L548
.L547:
	ldr	r3, .L549
	ldr	r3, [r3, #3840]
	str	r0, [r3, #4]
.L548:
	ldr	r3, .L549+4
	ldrb	r0, [r3, #277]	@ zero_extendqisi2
	b	NandcTimeCfg
.L550:
	.align	2
.L549:
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
	ldr	r3, .L554
	mov	r1, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r2, #1
	str	r1, [r3, #16]
	mov	r4, r3
	str	r0, [r3, #12]
	mov	r6, #0
	str	r2, [r3, #24]
	mov	r2, #2
	str	r0, [r3, #20]
	str	r2, [r3, #32]
	mov	r2, #3
	str	r0, [r3, #28]
	str	r0, [r3, #36]
	str	r0, [r3, #3840]
	str	r2, [r3, #40]
	ldr	r2, [r0]
	ldr	r5, .L554+4
	and	r2, r2, #253952
	ubfx	ip, r2, #13, #1
	bfi	r2, r1, #13, #1
	ldr	r1, [r0, #352]
	orr	r2, r2, #256
	str	ip, [r5, #-3396]
	movw	ip, #2049
	ubfx	r1, r1, #16, #4
	str	r1, [r5, #-3392]
	ldr	r1, [r0, #352]
	cmp	r1, ip
	str	r1, [r5, #-3388]
	moveq	r3, #8
	streq	r3, [r5, #-3392]
	str	r2, [r0]
	mov	r0, #40
	ldr	r3, [r4, #3840]
	str	r6, [r3, #336]
	bl	NandcTimeCfg
	ldr	r3, [r4, #3840]
	movw	r2, #8322
	mov	r0, #36864
	str	r2, [r3, #344]
	ldr	r2, .L554+8
	str	r2, [r3, #304]
	bl	ftl_malloc
	str	r6, [r5, #-3356]
	str	r6, [r5, #-3348]
	str	r0, [r5, #-3384]
	str	r0, [r5, #-3380]
	add	r0, r0, #32768
	str	r0, [r5, #-3376]
	ldmfd	sp!, {r4, r5, r6, pc}
.L555:
	.align	2
.L554:
	.word	.LANCHOR0
	.word	.LANCHOR2
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
	ldr	ip, .L558
	str	lr, [sp, #-4]!
	.save {lr}
	ldr	lr, [ip, #3840]
	ldr	lr, [lr, #4]
	str	lr, [r0]
	ldr	r0, [ip, #3840]
	ldr	r0, [r0]
	str	r0, [r1]
	ldr	r1, [ip, #3840]
	ldr	r1, [r1, #304]
	str	r1, [r2]
	ldr	r1, [ip, #3840]
	ldr	r2, [r1, #308]
	ldr	r1, [r1, #344]
	uxtb	r2, r2
	orr	r2, r2, r1, asl #16
	str	r2, [r3]
	ldr	pc, [sp], #4
.L559:
	.align	2
.L558:
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
	ldr	r3, .L568
	mov	r1, #16
	cmp	r0, r1
	ldr	r2, [r3, #3840]
	mov	r3, #1
	str	r3, [r2, #8]
	ldr	r3, .L568+4
	str	r0, [r3, #-3344]
	mov	r3, #0
	bfi	r3, r1, #8, #8
	bfc	r3, #18, #1
	bne	.L561
.L564:
	bfc	r3, #4, #1
	b	.L562
.L561:
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L562
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	beq	.L564
.L562:
	orr	r3, r3, #1
	str	r3, [r2, #12]
	bx	lr
.L569:
	.align	2
.L568:
	.word	.LANCHOR0
	.word	.LANCHOR2
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
	ldr	r3, .L571
	strb	r0, [r3, #3905]
	b	NandcBchSel
.L572:
	.align	2
.L571:
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
	ldr	r3, .L582
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, #0
	ldr	r2, [r3, #3840]
	mov	r4, r3
	ldr	r1, [r3, #3844]
	ldr	r6, .L582+4
	str	r1, [r2]
	ldr	r1, [r3, #3848]
	ldr	r2, [r3, #3840]
	str	r1, [r2, #4]
	ldr	r1, [r3, #3852]
	str	r1, [r2, #8]
	ldr	r1, [r3, #3856]
	str	r1, [r2, #12]
	ldr	r1, [r3, #3860]
	str	r1, [r2, #304]
	ldr	r1, [r3, #3864]
	str	r1, [r2, #308]
	ldr	r1, [r3, #3868]
	str	r1, [r2, #336]
	ldr	r1, [r3, #3872]
	str	r1, [r2, #344]
.L575:
	ldrb	r3, [r6, r5, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L574
	uxtb	r0, r5
	bl	FlashReset
.L574:
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L575
	ldr	r5, .L582+8
	ldrb	r3, [r5, #-3400]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L576
	mov	r0, #1
	bl	NandcSetMode
	ldrb	r0, [r5, #-3412]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r5, #-3412]	@ zero_extendqisi2
	bl	NandcSetMode
	ldrb	r0, [r4, #3861]	@ zero_extendqisi2
	bl	NandcSetDdrPara
.L576:
	ldr	r3, [r4, #1784]
	ldmfd	sp!, {r4, r5, r6, lr}
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	b	FlashBchSel
.L583:
	.align	2
.L582:
	.word	.LANCHOR0
	.word	IDByte
	.word	.LANCHOR2
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
	bne	.L586
	mov	r0, r4
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L586:
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
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r0
	ldr	r4, .L624
	.pad #20
	sub	sp, sp, #20
	mov	r10, r1
	mov	r8, r2
	ldrb	r5, [r4]	@ zero_extendqisi2
	cmp	r5, #0
	beq	.L589
	mov	r1, r2
	bl	FlashEraseSLc2KBlocks
	b	.L590
.L598:
	mov	r3, #36
	mov	r1, #0
	mul	fp, r3, r5
	rsb	r3, r5, r8
	add	r2, sp, #8
	uxtb	r3, r3
	str	r3, [sp]
	add	r6, r9, fp
	add	r3, sp, #12
	mov	r0, r6
	bl	LogAddr2PhyAddr
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	mov	r7, r0
	ldr	r0, [sp, #12]
	cmp	r0, r3
	mvncs	r3, #0
	strcs	r3, [r9, fp]
	bcc	.L621
.L592:
	add	r5, r5, #1
.L589:
	cmp	r5, r8
	bcc	.L598
	b	.L622
.L621:
	ldr	r3, .L624+4
	ldrb	r3, [r3, #-3340]	@ zero_extendqisi2
	cmp	r3, #0
	add	r3, r4, r0, asl #4
	ldr	r3, [r3, #1664]
	moveq	r7, #0
	cmp	r3, #0
	beq	.L594
	uxtb	r0, r0
	bl	FlashWaitCmdDone
.L594:
	ldr	r2, [sp, #12]
	mov	r0, #0
	cmp	r7, #0
	addne	ip, fp, #36
	mov	r3, r2, asl #4
	add	r2, r4, r2
	add	r1, r4, r3
	add	r3, r4, r3
	addne	ip, r9, ip
	str	r0, [r1, #1668]
	str	r6, [r1, #1664]
	ldr	r0, [sp, #8]
	ldrb	r6, [r2, #1616]	@ zero_extendqisi2
	strne	ip, [r1, #1668]
	str	r0, [r1, #1660]
	strb	r6, [r3, #1656]
	mov	r0, r6
	bl	NandcFlashCs
	cmp	r10, #1
	mov	r0, r6
	bne	.L596
	ldr	r3, .L624
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L596
	bl	flash_enter_slc_mode
	b	.L597
.L596:
	bl	flash_exit_slc_mode
.L597:
	ldr	r3, [sp, #12]
	mov	r0, r6
	ldr	r1, [sp, #8]
	add	r5, r5, r7
	add	r3, r4, r3, asl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r6
	mov	r2, r7
	ldr	r1, [sp, #8]
	bl	FlashEraseCmd
	mov	r0, r6
	bl	NandcFlashDeCs
	b	.L592
.L622:
	ldr	r0, [r4, #3840]
	mov	r5, #0
	bl	NandcIqrWaitFlashReady
	ldr	r6, .L624
	ldr	r7, .L624+8
.L599:
	ldrb	r3, [r4, #1612]	@ zero_extendqisi2
	cmp	r5, r3
	bcs	.L623
	uxtb	r0, r5
	bl	FlashWaitCmdDone
	cmp	r10, #1
	bne	.L600
	ldrb	r3, [r6, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L600
	ldrb	r0, [r7, r5, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L600:
	add	r5, r5, #1
	b	.L599
.L623:
	ldr	r3, .L624+4
	ldr	r3, [r3, #-3336]
	cmp	r3, #0
	bne	.L602
.L604:
	mov	r0, #0
	b	.L590
.L602:
	ldr	r3, .L624+12
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #69
	bne	.L604
	mov	r3, #0
	mov	r2, #36
	mov	r1, r3
.L603:
	cmp	r3, r8
	beq	.L604
	mul	r0, r2, r3
	add	r3, r3, #1
	str	r1, [r9, r0]
	b	.L603
.L590:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L625:
	.align	2
.L624:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+1656
	.word	IDByte
	.fnend
	.size	FlashEraseBlocks, .-FlashEraseBlocks
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
	ldr	r3, .L628
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
.L629:
	.align	2
.L628:
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
.L631:
	ldr	r3, [r0, #8]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #1048576
	beq	.L631
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
	mov	r4, #0
	ldr	lr, .L653
	.pad #28
	sub	sp, sp, #28
	ldr	r5, .L653+4
	add	r0, lr, r0, asl #3
	mov	lr, #16
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
	ldr	r3, [r5, #-3392]
	cmp	r3, #3
	bls	.L636
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
	beq	.L637
	cmp	r1, #0
	bne	.L638
.L646:
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
	ldreq	r0, [r5, #-3380]
	b	.L640
.L638:
	ldr	r3, [r5, #-3344]
	mov	r9, r5
	cmp	r3, #25
	movcc	r3, #64
	movcs	r3, #128
	str	r3, [sp, #4]
	mov	r3, r2, lsr #1
	str	r3, [sp, #12]
	mov	r3, #0
	mov	r0, r3
.L642:
	ldr	lr, [sp, #12]
	cmp	r0, lr
	bcs	.L646
	ldr	lr, [sp, #8]
	mov	r10, r3, lsr #2
	add	r0, r0, #1
	cmp	lr, #0
	ldrneh	fp, [ip, #2]
	mvneq	fp, #0
	ldrneh	lr, [ip], #4
	ldreq	lr, [r9, #-3376]
	orrne	lr, lr, fp, asl #16
	ldrne	fp, [r9, #-3376]
	streq	fp, [lr, r10, asl #2]
	strne	lr, [fp, r10, asl #2]
	ldr	lr, [sp, #4]
	add	r3, r3, lr
	b	.L642
.L640:
	ldr	r3, [r5, #-3376]
	ubfx	r10, r4, #22, #5
	mov	r2, r9
	str	r0, [r5, #-3372]
	mov	r1, r10, asl #10
	str	r3, [r5, #-3368]
	bl	rknand_dma_map_single
	mov	r2, r9
	mov	r1, r10, asl #7
	clz	r9, r9
	mov	r9, r9, lsr #5
	str	r0, [r5, #-3364]
	ldr	r0, [r5, #-3368]
	bl	rknand_dma_map_single
	mov	r3, #1
	str	r3, [r5, #-3356]
	tst	r8, #3
	ldr	r3, [r5, #-3364]
	str	r0, [r5, #-3360]
	str	r3, [r6, #20]
	ldr	r3, [r5, #-3360]
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
.L637:
	ldr	r3, [sp, #20]
	str	r3, [r6, #16]
.L636:
	str	r7, [r6, #12]
	str	r4, [r6, #8]
	orr	r4, r4, #4
	str	r4, [r6, #8]
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L654:
	.align	2
.L653:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	NandcXferStart, .-NandcXferStart
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
	ldr	r5, .L695
	ldr	r3, .L695+4
	add	r0, r3, r0, asl #3
	ldr	r3, [r5, #-3392]
	cmp	r3, #3
	ldr	r4, [r0, #12]
	bls	.L686
	ldr	r3, [r4, #16]
	tst	r3, #4
	beq	.L686
	mov	r0, r4
	bl	wait_for_nandc_xfer_completed
	ldr	r6, [r4, #16]
	ldr	r3, [r4, #8]
	ubfx	r6, r6, #1, #1
	cmp	r6, #0
	movne	r6, #0
	str	r3, [sp]
	beq	.L668
.L658:
	ldr	r2, [r4, #28]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	bge	.L666
	ldr	r3, [r5, #-3392]
	cmp	r3, #5
	bhi	.L659
.L662:
	add	r6, r6, #1
	bic	r3, r6, #-16777216
	cmp	r3, #0
	bne	.L658
	ldr	r2, [r4, #28]
	mov	r1, r6
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L695+8
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L695+12
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
	b	.L658
.L659:
	ldr	r3, [r4]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L662
	ldr	r3, [sp, #4]
	tst	r3, #131072
	beq	.L662
.L666:
	ldr	r3, [r5, #-3356]
	ldr	r4, .L695
	cmp	r3, #0
	beq	.L667
	ldr	r1, [sp]
	mov	r2, #0
	ldr	r0, [r4, #-3364]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r4, #-3360]
	mov	r2, #0
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
	b	.L667
.L670:
	ldr	r3, [r4, #8]
	add	r6, r6, #1
	str	r3, [sp]
	bic	r3, r6, #-16777216
	cmp	r3, #0
	bne	.L668
	ldr	r2, [sp]
	mov	r1, r6
	ldr	r3, [r4, #28]
	ldr	r0, .L695+16
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L695+12
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L668:
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L670
	ldr	r3, [r5, #-3348]
	cmp	r3, #0
	beq	.L671
	mov	r0, r4
	bl	NandcSendDumpDataStart
.L671:
	ldr	r3, [r5, #-3356]
	ldr	r6, .L695
	cmp	r3, #0
	beq	.L672
	ldr	r1, [sp]
	mov	r2, #1
	ldr	r0, [r6, #-3364]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r6, #-3360]
	mov	r2, #1
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
.L672:
	ldr	r3, [r5, #-3348]
	cmp	r3, #0
	beq	.L667
	mov	r0, r4
	bl	NandcSendDumpDataDone
.L667:
	mov	r3, #0
	str	r3, [r5, #-3356]
	b	.L655
.L686:
	ldr	r3, [r4, #8]
	str	r3, [sp]
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L686
.L655:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L696:
	.align	2
.L695:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC4
	.word	.LC5
	.word	.LC6
	.fnend
	.size	NandcXferComp, .-NandcXferComp
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
	bne	.L698
	cmp	r3, #0
	beq	.L699
	mov	r0, r2
	mov	r1, r3
	mov	r2, #1024
	bl	ftl_memcpy
.L699:
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
.L698:
	cmp	r3, #0
	beq	.L702
	mov	r1, r2
	mov	r0, r3
	mov	r2, #1024
	bl	ftl_memcpy
.L702:
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
	.global	NandcXferData
	.type	NandcXferData, %function
NandcXferData:
	.fnstart
	@ args = 4, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r3
	ldr	r3, .L762
	.pad #92
	sub	sp, sp, #92
	tst	fp, #63
	mov	r10, r0
	add	r3, r3, r0, asl #3
	mov	r7, r1
	mov	r8, r2
	ldr	r5, [sp, #128]
	ldr	r9, [r3, #12]
	bne	.L715
	cmp	r5, #0
	bne	.L716
	add	r0, sp, #24
	mov	r1, #255
	mov	r2, #64
	bl	ftl_memset
	add	r5, sp, #24
.L716:
	mov	r0, r10
	mov	r1, r7
	mov	r2, r8
	mov	r3, #0
	str	fp, [sp]
	str	r5, [sp, #4]
	bl	NandcXferStart
	mov	r0, r10
	bl	NandcXferComp
	cmp	r7, #0
	movne	r6, #0
	bne	.L717
	ldr	r4, .L762+4
	mov	r1, r8, lsr #1
	mov	r2, r7
	ldr	r3, [r4, #-3344]
	cmp	r3, #25
	mov	r3, r7
	movcc	ip, #64
	movcs	ip, #128
.L719:
	cmp	r2, r1
	add	r5, r5, #4
	add	r0, r3, ip
	bcs	.L760
	ldr	lr, [r4, #-3376]
	mov	r3, r3, lsr #2
	add	r2, r2, #1
	ldr	r3, [lr, r3, asl #2]
	mov	lr, r3, lsr #8
	strb	r3, [r5, #-4]
	strb	lr, [r5, #-3]
	mov	lr, r3, lsr #16
	mov	r3, r3, lsr #24
	strb	lr, [r5, #-2]
	strb	r3, [r5, #-1]
	mov	r3, r0
	b	.L719
.L760:
	ldr	r0, [r4, #-3344]
	mov	r2, #0
	ldr	r1, [r4, #-3392]
	mov	r8, r8, lsr #2
	mov	r6, r2
.L721:
	cmp	r2, r8
	bcs	.L717
	cmp	r0, #0
	beq	.L717
	add	r3, r2, #8
	ldr	r3, [r9, r3, asl #2]
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	tst	r3, #4
	bne	.L745
	ldr	r4, [sp, #20]
	ubfx	r4, r4, #15, #1
	cmp	r4, #0
	bne	.L745
	cmp	r1, #5
	bls	.L723
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
	b	.L759
.L723:
	cmp	r1, #3
	bls	.L725
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
	bls	.L726
	ldr	r3, [sp, #20]
	ldr	r4, [sp, #20]
	ubfx	r3, r3, #3, #5
	ubfx	r4, r4, #28, #1
.L759:
	orr	r4, r3, r4, asl #5
	b	.L725
.L726:
	ldr	r5, [sp, #20]
	ldr	r4, [sp, #20]
	ubfx	r5, r5, #16, #5
	ubfx	r4, r4, #30, #1
	orr	r4, r5, r4, asl #5
.L725:
	cmp	r6, r4
	movcc	r6, r4
	b	.L722
.L745:
	mvn	r6, #0
.L722:
	add	r2, r2, #1
	b	.L721
.L717:
	mov	r3, #0
	str	r3, [r9, #16]
	b	.L728
.L715:
	cmp	r1, #1
	mov	r4, #0
	bne	.L757
	mov	r6, r4
.L729:
	cmp	r4, r8
	bcs	.L761
	cmp	fp, #0
	and	ip, r4, #3
	addne	r3, fp, r4, asl #9
	mov	r0, r9
	moveq	r3, fp
	cmp	r5, #0
	mov	r1, #1
	str	ip, [sp, #8]
	movne	r2, #2
	moveq	r2, #0
	mla	r2, r4, r2, r5
	add	r4, r4, #2
	str	r2, [sp]
	mov	r2, ip
	bl	NandcCopy1KB
	mov	r0, r10
	mov	r1, #1
	mov	r2, #2
	ldr	ip, [sp, #8]
	str	r6, [sp]
	str	r6, [sp, #4]
	mov	r3, ip
	bl	NandcXferStart
	mov	r0, r10
	bl	NandcXferComp
	b	.L729
.L761:
	mov	r6, #0
	b	.L728
.L757:
	str	r4, [sp]
	mov	r1, r4
	str	r4, [sp, #4]
	mov	r2, #2
	mov	r3, r4
	mov	r6, r4
	bl	NandcXferStart
	str	fp, [sp, #8]
.L734:
	cmp	r4, r8
	bcs	.L728
	mov	r0, r10
	bl	NandcXferComp
	ldr	r3, [r9, #32]
	add	ip, r4, #2
	cmp	ip, r8
	str	r3, [sp, #20]
	bcs	.L735
	mov	r3, #0
	mov	r0, r10
	str	r3, [sp]
	mov	r1, r3
	str	r3, [sp, #4]
	mov	r2, #2
	and	r3, ip, #3
	str	ip, [sp, #12]
	bl	NandcXferStart
	ldr	ip, [sp, #12]
.L735:
	ldr	r3, [sp, #20]
	tst	r3, #4
	mvnne	r6, #0
	bne	.L736
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r2, r2, #3, #5
	ubfx	r3, r3, #27, #1
	orr	r3, r2, r3, asl #5
	cmp	r6, r3
	movcc	r6, r3
.L736:
	cmp	fp, #0
	ldr	r3, [sp, #8]
	sub	r2, ip, #2
	mov	r0, r9
	moveq	r3, #0
	cmp	r5, #0
	and	r2, r2, #3
	str	ip, [sp, #12]
	movne	r1, #2
	moveq	r1, #0
	mla	r4, r4, r1, r5
	mov	r1, #0
	str	r4, [sp]
	bl	NandcCopy1KB
	ldr	ip, [sp, #12]
	ldr	r3, [sp, #8]
	mov	r4, ip
	add	r3, r3, #1024
	str	r3, [sp, #8]
	b	.L734
.L728:
	ldr	r3, .L762+4
	clz	r7, r7
	mov	r7, r7, lsr #5
	ldr	r3, [r3, #-3392]
	cmp	r3, #5
	movls	r7, #0
	cmp	r7, #0
	beq	.L740
	ldr	r3, [r9]
	and	r2, r3, #139264
	cmp	r2, #139264
	orreq	r3, r3, #131072
	streq	r3, [r9]
	mvneq	r6, #0
.L740:
	mov	r0, r6
	add	sp, sp, #92
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L763:
	.align	2
.L762:
	.word	.LANCHOR0
	.word	.LANCHOR2
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
	ldr	r3, .L767
	subs	r4, r0, #0
	mov	r6, r1
	mov	r7, r2
	ldrb	r5, [r3, #265]	@ zero_extendqisi2
	bne	.L765
	ldr	r2, .L767+4
	ldrb	r3, [r2, #1]	@ zero_extendqisi2
	ldr	r2, [r2, #4]
	mul	r2, r2, r3
	cmp	r1, r2
	movcc	r5, #4
.L765:
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
.L768:
	.align	2
.L767:
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
	ldr	r5, .L796
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L796+4
	mov	r10, r0
	str	r1, [sp]
	mov	fp, r2
	ldr	r3, [r5, #3840]
	ldr	r3, [r3, #304]
	str	r3, [sp, #8]
	ldr	r3, [r4, #-3392]
	cmp	r3, #8
	ldr	r3, [sp, #56]
	movcc	r9, #6
	movcs	r9, #12
	cmp	r3, #0
	moveq	r6, #1024
	beq	.L771
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
	ldrb	r0, [r4, #-3412]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #-3412]	@ zero_extendqisi2
	bl	NandcSetMode
	cmn	r6, #1
	bne	.L772
.L781:
	mvn	r6, #0
	b	.L773
.L772:
	mov	r2, r6
	ldr	r0, .L796+8
	ldr	r1, [sp]
	bl	printk
	cmp	r6, #9
	addls	r5, r5, r10, asl #3
	ldrls	r3, [r5, #12]
	ldrls	r2, [r3, #3840]
	ldrls	r2, [r3]
	orrls	r2, r2, #131072
	strls	r2, [r3]
	ldr	r3, [r4, #-3332]
	add	r3, r3, #1
	str	r3, [r4, #-3332]
	cmp	r3, #2048
	bcc	.L773
	ldr	r3, .L796+4
	mov	r7, #0
	mov	fp, r7
	str	r7, [r3, #-3332]
.L771:
	mov	r4, #0
	mvn	r8, #0
	mov	ip, r4
	mov	r5, r4
	str	r4, [sp, #4]
.L779:
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
	bhi	.L775
	cmp	r0, #2
	bhi	.L785
	add	r5, r5, #1
	cmp	r5, #9
	bls	.L785
	rsb	r4, r5, r9
	mov	r6, r0
	mov	r8, #0
	b	.L777
.L775:
	ldr	r3, [sp, #4]
	cmp	r3, r5
	bcs	.L786
	cmp	r5, #7
	rsb	ip, r5, r4
	bhi	.L787
	str	r5, [sp, #4]
	b	.L786
.L785:
	mov	r8, #0
	mov	r4, r9
	mov	r6, r0
	mov	r7, r8
	mov	fp, r8
	b	.L776
.L786:
	mov	r5, #0
.L776:
	add	r9, r9, #2
	cmp	r9, #69
	bls	.L779
.L777:
	ldr	r3, [sp, #4]
	cmp	r3, r5
	movcs	r4, ip
	b	.L778
.L787:
	mov	r4, ip
.L778:
	cmp	r4, #0
	beq	.L780
	ldr	r0, .L796+12
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	bl	NandcSetDdrPara
.L780:
	cmn	r8, #1
	bne	.L773
	ldr	r0, .L796+16
	mov	r1, r10
	ldr	r2, [sp]
	bl	printk
	ldr	r3, [sp, #56]
	cmp	r3, #0
	beq	.L781
	ldr	r3, [sp, #8]
	ubfx	r0, r3, #8, #8
	bl	NandcSetDdrPara
.L773:
	mov	r0, r6
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L797:
	.align	2
.L796:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC7
	.word	.LC8
	.word	.LC9
	.fnend
	.size	FlashDdrTunningRead, .-FlashDdrTunningRead
	.align	2
	.global	FlashReadPage
	.type	FlashReadPage, %function
FlashReadPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	mov	r5, r0
	mov	r6, r1
	mov	r8, r2
	mov	r7, r3
	bl	FlashReadRawPage
	ldr	r10, .L819
	cmn	r0, #1
	mov	r4, r0
	bne	.L799
	ldr	r9, .L819+4
	ldrb	fp, [r9, #8]	@ zero_extendqisi2
	mov	ip, r9
	cmp	fp, #0
	bne	.L800
.L802:
	ldrb	r3, [r10, #-3400]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L799
	b	.L818
.L800:
	mov	r3, #0
	mov	r0, r5
	strb	r3, [r9, #8]
	mov	r1, r6
	mov	r2, r8
	mov	r3, r7
	str	r9, [sp, #12]
	bl	FlashReadRawPage
	strb	fp, [r9, #8]
	cmn	r0, #1
	ldr	ip, [sp, #12]
	movne	r4, r0
	beq	.L802
	b	.L799
.L818:
	ldr	r3, [ip, #3840]
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	ldr	r9, [r3, #304]
	mov	r3, #1
	str	r3, [sp]
	mov	r3, r7
	bl	FlashDdrTunningRead
	cmn	r0, #1
	mov	r4, r0
	beq	.L803
	ldr	r3, .L819+4
	ldrb	r3, [r3, #3905]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L799
.L803:
	ubfx	r0, r9, #8, #8
	bl	NandcSetDdrPara
.L799:
	ldr	ip, [r10, #-3328]
	adds	r3, ip, #0
	movne	r3, #1
	cmn	r4, #1
	movne	r3, #0
	cmp	r3, #0
	beq	.L804
	mov	r1, r6
	mov	r2, r8
	mov	r3, r7
	mov	r0, r5
	blx	ip
	mov	r2, r5
	mov	r3, r6
	mov	r4, r0
	ldr	r0, .L819+8
	mov	r1, r4
	bl	printk
	cmn	r4, #1
	bne	.L804
	ldr	r3, .L819+4
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L804
	mov	r0, r5
	bl	flash_enter_slc_mode
	ldr	r3, .L819
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	ldr	ip, [r3, #-3328]
	mov	r3, r7
	blx	ip
	mov	r4, r0
	mov	r0, r5
	bl	flash_exit_slc_mode
.L804:
	mov	r0, r4
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L820:
	.align	2
.L819:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC10
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
	ldr	r5, .L831
	mov	r6, r1
	mov	r4, #0
	ldrb	r0, [r5, #-3412]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r5, #-3412]	@ zero_extendqisi2
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
	bne	.L822
	ldrb	r2, [r5, #-3412]	@ zero_extendqisi2
	tst	r2, #1
	beq	.L822
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	strb	r4, [r5, #-3400]
	b	.L823
.L822:
	mov	r2, #1
	strb	r2, [r3, #-3400]
.L823:
	mov	r0, #0
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L832:
	.align	2
.L831:
	.word	.LANCHOR2
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
	ldr	r3, .L849
	mov	r4, #0
	ldr	r8, .L849+4
	mov	r9, #4
	ldr	r5, .L849+8
	mvn	r7, #0
	ldr	r0, [r3]	@ unaligned
	ldr	r3, [r8, #4]
	str	r4, [r5, #-3320]
	mov	r6, r5
	str	r0, [sp, #12]	@ unaligned
	mov	r0, r4
	str	r3, [sp, #4]
	ldr	r3, [r5, #-3408]
	str	r3, [r5, #-3324]
	bl	flash_enter_slc_mode
.L834:
	add	fp, r4, #1
	mov	r10, #0
.L836:
	add	r3, sp, #12
	ldrb	r0, [r3, r10]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	mov	r1, r4
	ldr	r2, [r5, #-3408]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L835
	mov	r0, #0
	mov	r1, fp
	ldr	r2, [r6, #-3408]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L835
	add	r10, r10, #1
	cmp	r10, #4
	beq	.L837
	b	.L836
.L838:
	add	r0, fp, #12
	movw	r1, #2036
	bl	js_hash
	ldr	r3, [fp, #8]
	cmp	r3, r0
	mvnne	r7, #0
	bne	.L837
	ldr	r7, .L849+12
	add	r1, fp, #160
	mov	r2, #32
	mov	r0, r7
	bl	ftl_memcpy
	ldr	r1, [r6, #-3324]
	mov	r2, #32
	ldr	r0, .L849+16
	add	r1, r1, #192
	bl	ftl_memcpy
	ldr	r1, [r6, #-3324]
	mov	r2, #852
	ldr	r0, .L849+20
	add	r1, r1, #224
	bl	ftl_memcpy
	ldrh	r0, [r7, #10]
	bl	FlashBlockAlignInit
	ldr	r7, [r6, #-3324]
	str	r4, [r6, #-3320]
	mov	r0, r4
	ldr	r1, [r8, #4]
	ldr	r3, [r7, #1076]
	strb	r3, [r6, #-3400]
	bl	__aeabi_uidiv
	add	r0, r0, #1
	cmp	r0, #1
	strhi	r0, [r6, #-3316]
	movls	r3, #2
	strls	r3, [r6, #-3316]
	ldrh	r3, [r7, #14]
	mov	r7, #0
	strb	r3, [r5, #-3312]
.L837:
	ldr	r3, [sp, #4]
	subs	r9, r9, #1
	add	r4, r4, r3
	bne	.L834
	mov	r0, r9
.L848:
	bl	flash_exit_slc_mode
	mov	r0, r7
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L835:
	ldr	fp, [r5, #-3324]
	ldr	r2, .L849+24
	ldr	r3, [fp]
	cmp	r3, r2
	bne	.L837
	cmp	r7, #0
	bne	.L838
	mov	r0, r4
	ldr	r1, [r8, #4]
	bl	__aeabi_uidiv
	ldr	r3, .L849+8
	add	r0, r0, #1
	str	r0, [r3, #-3316]
	mov	r0, r7
	b	.L848
.L850:
	.align	2
.L849:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR1+256
	.word	.LANCHOR0+44
	.word	.LANCHOR0+3916
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
	mov	r8, r0
	mov	fp, r3
	str	r2, [sp, #4]
	str	r1, [sp, #12]
	bl	NandcWaitFlashReady
	ldr	r2, .L880
	add	r3, r2, r8, asl #3
	ldr	r5, [r3, #12]
	ldrb	r9, [r3, #16]	@ zero_extendqisi2
	ldr	r3, .L880+4
	add	r6, r9, #8
	ldrb	r7, [r3]	@ zero_extendqisi2
	add	r6, r5, r6, asl #8
	sub	r4, r7, #67
	cmp	r4, #1
	str	r3, [sp, #8]
	movls	r3, #0
	str	r2, [sp, #16]
	strls	r3, [sp]
	bls	.L852
	ldr	r3, .L880+8
	ldrb	r4, [r3, #-3400]	@ zero_extendqisi2
	cmp	r4, #0
	streq	r4, [sp]
	beq	.L853
	mov	r0, #0
	bl	NandcSetDdrMode
	mov	r3, #1
	str	r3, [sp]
.L853:
	add	r3, r5, r9, asl #8
	mov	r2, #92
	str	r2, [r3, #2056]
	mov	r2, #197
	str	r2, [r3, #2056]
.L852:
	mov	r7, #1
	mvn	r10, #0
	mov	r3, r9, asl #8
	str	r3, [sp, #20]
.L854:
	ldr	r3, .L880+12
	ldrb	r3, [r3]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r7, r3
	bcs	.L879
	ldr	r3, [sp, #8]
	mov	r0, r6
	uxtb	r1, r7
	ldrb	r3, [r3]	@ zero_extendqisi2
	sub	r3, r3, #67
	cmp	r3, #1
	bhi	.L855
	bl	SandiskSetRRPara
	b	.L856
.L855:
	bl	ToshibaSetRRPara
.L856:
	ldr	r3, [sp, #8]
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L857
	ldr	r3, .L880+12
	ldrb	r3, [r3]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r7, r3
	ldreq	r3, [sp, #20]
	moveq	r2, #179
	addeq	r3, r5, r3
	streq	r2, [r3, #2056]
.L857:
	ldr	r3, [sp, #20]
	mov	r2, #38
	add	r3, r5, r3
	str	r2, [r3, #2056]
	mov	r2, #93
	str	r2, [r3, #2056]
	ldr	r3, [sp]
	cmp	r3, #0
	beq	.L858
	mov	r0, #4
	bl	NandcSetDdrMode
	mov	r0, r8
	mov	r3, fp
	ldr	r1, [sp, #12]
	ldr	r2, [sp, #4]
	bl	FlashReadRawPage
	mov	r4, r0
	mov	r0, #0
	bl	NandcSetDdrMode
	b	.L859
.L858:
	mov	r0, r8
	ldr	r1, [sp, #12]
	ldr	r2, [sp, #4]
	mov	r3, fp
	bl	FlashReadRawPage
	mov	r4, r0
.L859:
	cmn	r4, #1
	beq	.L860
	ldr	r3, [sp, #16]
	cmn	r10, #1
	moveq	r10, r4
	ldrb	r2, [r3, #3905]	@ zero_extendqisi2
	add	r2, r2, r2, asl #1
	cmp	r4, r2, asr #2
	bcc	.L862
	mov	fp, #0
	str	fp, [sp, #4]
.L860:
	add	r7, r7, #1
	b	.L854
.L879:
	mov	r4, r10
.L862:
	ldr	r3, [sp, #8]
	mov	r0, r6
	mov	r1, #0
	ldrb	r2, [r3]	@ zero_extendqisi2
	sub	r2, r2, #67
	cmp	r2, #1
	bhi	.L864
	bl	SandiskSetRRPara
	b	.L865
.L864:
	bl	ToshibaSetRRPara
.L865:
	ldr	r3, [sp, #16]
	add	r5, r5, r9, asl #8
	mov	r2, #255
	str	r2, [r5, #2056]
	ldrb	r2, [r3, #3905]	@ zero_extendqisi2
	add	r2, r2, r2, asl #1
	cmp	r4, r2, asr #2
	bcc	.L866
	cmn	r4, #1
	movne	r4, #256
.L866:
	mov	r0, r8
	bl	NandcWaitFlashReady
	ldr	r3, [sp]
	cmp	r3, #0
	beq	.L867
	mov	r0, #4
	bl	NandcSetDdrMode
.L867:
	mov	r0, r4
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L881:
	.align	2
.L880:
	.word	.LANCHOR0
	.word	g_retryMode
	.word	.LANCHOR2
	.word	g_maxRetryCount
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
	ldr	r5, .L896
	mov	r10, r2
	mov	r8, r3
	mov	fp, r1
	bl	NandcWaitFlashReady
	add	r2, r5, r9, asl #3
	ldr	ip, .L896+4
	mov	r7, #1
	ldrb	r3, [r2, #16]	@ zero_extendqisi2
	mvn	r4, #0
	ldr	r6, [r2, #12]
	add	r3, r3, #8
	add	r6, r6, r3, asl #8
.L883:
	ldrb	r3, [ip]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r7, r3
	bcs	.L886
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
	beq	.L884
	ldrb	r3, [r5, #3905]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L889
	mov	r8, #0
	mov	r10, r8
.L884:
	add	r7, r7, #1
	b	.L883
.L889:
	mov	r4, r0
.L886:
	mov	r0, r6
	mov	r1, #0
	bl	SamsungSetRRPara
	ldrb	r3, [r5, #3905]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r4, r3, asr #2
	bcc	.L888
	cmn	r4, #1
	movne	r4, #256
.L888:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L897:
	.align	2
.L896:
	.word	.LANCHOR0
	.word	g_maxRetryCount
	.fnend
	.size	SamsungReadRetrial, .-SamsungReadRetrial
	.align	2
	.global	MicronReadRetrial
	.type	MicronReadRetrial, %function
MicronReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
.L900:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r3
	ldr	r3, .L924
	mov	fp, r2
	.pad #36
	sub	sp, sp, #36
	mov	r6, r0
	mov	r10, #0
	ldrb	r5, [r3, #3905]	@ zero_extendqisi2
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	str	r1, [sp, #20]
	cmp	r3, #0
	addeq	r5, r5, r5, asl #1
	ldrne	r2, .L924+4
	ubfxeq	r5, r5, #2, #8
	smullne	r2, r3, r5, r2
	uxtbne	r5, r3
	ldr	r3, .L924
	add	r3, r3, r0, asl #3
	str	r3, [sp, #24]
.L910:
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
.L901:
	ldr	r3, .L924+8
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r9, r3
	bcs	.L904
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
	beq	.L902
	cmn	r4, #1
	ldr	ip, [sp, #28]
	moveq	r4, r0
	cmp	r0, r5
	bcc	.L912
	mov	r8, ip
	mov	fp, ip
.L902:
	ldr	r9, [sp, #8]
	b	.L901
.L912:
	mov	r4, r0
	mov	r8, ip
	mov	fp, ip
.L904:
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
	bcc	.L906
	cmn	r4, #1
	movne	r4, #256
.L906:
	cmn	r4, #1
	movne	r7, #0
	moveq	r7, #1
	cmp	r4, #256
	movne	r1, r7
	orreq	r1, r7, #1
	cmp	r1, #0
	beq	.L907
	str	r4, [sp]
	mov	r1, r9
	ldr	r0, .L924+12
	mov	r3, r9
	ldr	r2, [sp, #20]
	bl	printk
	cmp	r10, #0
	bne	.L908
	ldr	r3, .L924
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r7, #0
	andne	r7, r7, #1
	cmp	r7, #0
	beq	.L917
	mov	r0, r6
	mov	r1, #3
	bl	micron_auto_read_calibration_config
	mov	r10, #1
	b	.L910
.L908:
	mov	r0, r6
	mov	r1, #0
	bl	micron_auto_read_calibration_config
	cmn	r4, #1
	movne	r4, #256
	b	.L917
.L907:
	cmp	r10, #0
	beq	.L917
	mov	r0, r6
	mov	r4, #256
	bl	micron_auto_read_calibration_config
.L917:
	mov	r0, r4
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L925:
	.align	2
.L924:
	.word	.LANCHOR0
	.word	1431655766
	.word	g_maxRetryCount
	.word	.LC11
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
	ldr	r5, .L944
	mov	r10, r2
	mov	r7, r0
	mov	fp, r1
	add	r2, r5, r0
	mvn	r6, #0
	ldr	r3, [r5, #1784]
	ldrb	r4, [r2, #3928]	@ zero_extendqisi2
	ldrb	r9, [r5, #3918]	@ zero_extendqisi2
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #7
	cmp	r3, #1
	ldrlsb	r4, [r2, #3936]	@ zero_extendqisi2
	bl	NandcWaitFlashReady
	mov	ip, #0
.L928:
	cmp	ip, r9
	bcs	.L932
	add	r4, r4, #1
	mov	r0, r7
	ldrb	r1, [r5, #3917]	@ zero_extendqisi2
	uxtb	r4, r4
	ldr	r2, .L944+4
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
	beq	.L930
	ldrb	r3, [r5, #3905]	@ zero_extendqisi2
	cmn	r6, #1
	moveq	r6, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L937
	mov	r8, #0
	mov	r10, r8
.L930:
	add	ip, ip, #1
	b	.L928
.L937:
	mov	r6, r0
.L932:
	ldr	r3, [r5, #1784]
	add	r7, r5, r7
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #7
	cmp	r3, #1
	ldrb	r3, [r5, #3905]	@ zero_extendqisi2
	strlsb	r4, [r7, #3936]
	strhib	r4, [r7, #3928]
	add	r3, r3, r3, asl #1
	cmp	r6, r3, asr #2
	bcc	.L936
	cmn	r6, #1
	movne	r6, #256
.L936:
	mov	r0, r6
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L945:
	.align	2
.L944:
	.word	.LANCHOR0
	.word	.LANCHOR0+3920
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
	ldr	r3, .L950
	subs	r4, r0, #0
	mov	r5, r1
	mov	r7, r2
	ldrb	r6, [r3, #265]	@ zero_extendqisi2
	bne	.L947
	ldr	r2, .L950+4
	ldrb	r3, [r2, #1]	@ zero_extendqisi2
	ldr	r1, [r2, #4]
	mul	r1, r1, r3
	cmp	r5, r1
	bcs	.L947
	ldrb	r3, [r2]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r6, r6, #2
	movne	r6, #4
.L947:
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
.L951:
	.align	2
.L950:
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
	ldr	r4, .L962
	ldr	r5, .L962+4
	ldr	r8, .L962+8
	ldr	r3, [r4, #-3408]
	mov	r9, r4
	ldrb	r0, [r4, #-3311]	@ zero_extendqisi2
	mov	r10, r5
	str	r3, [r4, #-3324]
	bl	FlashBchSel
	mov	r1, #0
	mov	r2, #2048
	ldr	r0, [r4, #-3408]
	bl	ftl_memset
	ldr	r3, [r4, #-3324]
	ldr	r1, .L962+12
	mov	r2, #32
	str	r8, [r3]
	ldr	r0, [r4, #-3324]
	ldrb	r3, [r5, #1612]	@ zero_extendqisi2
	add	r0, r0, #16
	strh	r3, [r0, #-4]	@ movhi
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	strh	r3, [r0, #-2]	@ movhi
	ldrb	r3, [r4, #-3400]	@ zero_extendqisi2
	str	r3, [r0, #1060]
	bl	ftl_memcpy
	ldr	r0, [r4, #-3324]
	add	r1, r5, #1616
	mov	r2, #8
	add	r0, r0, #80
	bl	ftl_memcpy
	ldr	r0, [r4, #-3324]
	ldr	r1, .L962+16
	mov	r2, #32
	add	r0, r0, #96
	bl	ftl_memcpy
	ldr	r0, [r4, #-3324]
	ldr	r1, .L962+20
	mov	r2, #32
	add	r0, r0, #160
	bl	ftl_memcpy
	ldr	r0, [r4, #-3324]
	add	r1, r5, #44
	mov	r2, #32
	add	r0, r0, #192
	bl	ftl_memcpy
	ldr	r0, [r4, #-3324]
	mov	r2, #852
	ldr	r1, .L962+24
	add	r0, r0, #224
	bl	ftl_memcpy
	ldr	r6, [r4, #-3324]
	movw	r1, #2036
	add	r0, r6, #12
	bl	js_hash
	movw	r3, #1592
	str	r3, [r6, #4]
	ldr	r3, [r4, #-3308]
	str	r3, [r4, #-3324]
	str	r0, [r6, #8]
	mov	r0, #0
	bl	flash_enter_slc_mode
	mov	r6, #0
	mov	r7, r6
.L955:
	ldr	r1, [r5, #4]
	mov	r0, #0
	mov	r2, r0
	mul	r1, r1, r7
	bl	FlashEraseBlock
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r4, #-3408]
	mov	r3, r0
	mul	r1, r1, r7
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r4, #-3408]
	mov	r3, r0
	mul	r1, r1, r7
	add	r1, r1, #1
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r4, #-3308]
	mov	r3, r0
	mul	r1, r1, r7
	bl	FlashReadRawPage
	add	r2, r7, #1
	cmn	r0, #1
	beq	.L953
	ldr	fp, [r9, #-3324]
	ldr	r3, [fp]
	cmp	r3, r8
	bne	.L953
	add	r0, fp, #12
	movw	r1, #2036
	str	r2, [sp, #4]
	bl	js_hash
	ldr	r3, [fp, #8]
	cmp	r3, r0
	ldr	r2, [sp, #4]
	bne	.L953
	ldr	r3, [r10, #4]
	cmp	r6, #1
	str	r2, [r9, #-3316]
	mul	r7, r3, r7
	str	r7, [r9, #-3320]
	beq	.L956
	mov	r6, #1
.L953:
	cmp	r2, #4
	mov	r7, r2
	bne	.L955
	b	.L954
.L956:
	mov	r6, #2
.L954:
	mov	r0, #0
	bl	flash_exit_slc_mode
	clz	r0, r6
	mov	r0, r0, lsr #5
	rsb	r0, r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L963:
	.align	2
.L962:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	1312902724
	.word	IDByte
	.word	.LANCHOR0+1624
	.word	.LANCHOR1+256
	.word	.LANCHOR0+3916
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
	mov	r10, r0
	ldr	r3, .L984
	.pad #20
	sub	sp, sp, #20
	ldr	r7, .L984+4
	ldr	r6, .L984+8
	ldr	r0, [r3]	@ unaligned
	ldr	r3, [r7, #-3404]
	ldrb	fp, [r6, #3905]	@ zero_extendqisi2
	cmp	r3, #0
	str	r0, [sp, #12]	@ unaligned
	beq	.L965
	mov	r0, #0
	bl	flash_enter_slc_mode
.L965:
	mov	r0, r10
	mov	r1, #0
	mov	r2, #2048
	mvn	r9, #0
	bl	ftl_memset
	mov	r4, #2
.L966:
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L970
	mov	r8, #0
.L968:
	add	r3, sp, #12
	ldr	r5, .L984+4
	ldrb	ip, [r8, r3]	@ zero_extendqisi2
	mov	r0, ip
	str	ip, [sp, #4]
	bl	FlashBchSel
	ldr	r1, [r6, #4]
	mov	r0, #0
	ldr	r2, [r7, #-3408]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	ldr	ip, [sp, #4]
	bne	.L967
	add	r8, r8, #1
	cmp	r8, #4
	bne	.L968
	b	.L969
.L967:
	ldr	r3, [r5, #-3408]
	ldr	r2, .L984+12
	ldr	r3, [r3]
	cmp	r3, r2
	bne	.L969
	mov	r1, ip
	ldr	r0, .L984+16
	bl	printk
	mov	r2, #2048
	mov	r0, r10
	ldr	r1, [r5, #-3408]
	bl	ftl_memcpy
	ldr	r3, [r5, #-3408]
	ldr	r2, .L984+8
	ldr	r3, [r3, #512]
	strb	r3, [r2, #1]
	ldr	r3, [r5, #-3316]
	cmp	r3, r4
	bls	.L973
	str	r4, [r5, #-3316]
	bl	FlashSavePhyInfo
	mov	r9, #0
.L969:
	add	r4, r4, #1
	b	.L966
.L973:
	mov	r9, #0
.L970:
	mov	r0, fp
	bl	FlashBchSel
	ldr	r3, [r7, #-3404]
	cmp	r3, #0
	beq	.L977
	mov	r0, #0
	bl	flash_exit_slc_mode
.L977:
	mov	r0, r9
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L985:
	.align	2
.L984:
	.word	.LANCHOR3
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-52655045
	.word	.LC12
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
	mov	r6, r0
	.pad #20
	sub	sp, sp, #20
	mov	r0, #32768
	bl	ftl_malloc
	ldr	r4, .L1081
	ldr	r5, .L1081+4
	mov	r7, #0
	str	r0, [r4, #-3408]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r4, #-3308]
	mov	r0, #4096
	bl	ftl_malloc
	str	r0, [r4, #-3304]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r4, #-3300]
	mov	r0, #4096
	bl	ftl_malloc
	mov	r3, #50
	str	r7, [r4, #-3316]
	strb	r3, [r5, #1]
	strb	r3, [r4, #-3312]
	mov	r3, #128
	strb	r7, [r4, #-3400]
	str	r3, [r5, #4]
	mov	r3, #60
	str	r7, [r4, #-3332]
	strb	r7, [r5]
	strb	r7, [r4, #-3292]
	strb	r3, [r4, #-3311]
	str	r0, [r4, #-3296]
	mov	r0, r6
	bl	NandcInit
	ldr	r6, .L1081+8
	mov	r8, r6
.L992:
	ldr	r3, .L1081+12
	uxtb	r9, r7
	add	r2, r3, r7, asl #3
	mov	r0, r9
	ldr	r10, [r3, r7, asl #3]
	ldrb	fp, [r2, #4]	@ zero_extendqisi2
	bl	FlashReset
	mov	r0, r9
	bl	NandcFlashCs
	mov	r2, #144
	add	fp, r10, fp, asl #8
	mov	r0, #200
	mov	r10, #0
	str	r2, [fp, #2056]
	str	r10, [fp, #2052]
	bl	NandcDelayns
	ldr	r1, [fp, #2048]
	uxtb	r1, r1
	strb	r1, [r6]
	ldr	r0, [fp, #2048]
	cmp	r1, #44
	strb	r0, [r6, #1]
	ldr	r0, [fp, #2048]
	strb	r0, [r6, #2]
	ldr	r0, [fp, #2048]
	strb	r0, [r6, #3]
	ldr	r0, [fp, #2048]
	strb	r0, [r6, #4]
	ldr	r0, [fp, #2048]
	strb	r0, [r6, #5]
	bne	.L987
	mov	r1, #239
	mov	r0, #200
	str	r1, [fp, #2056]
	mov	r1, #1
	str	r1, [fp, #2052]
	bl	NandcDelayns
	mov	r1, #4
	str	r1, [fp, #2048]
	str	r10, [fp, #2048]
	str	r10, [fp, #2048]
	str	r10, [fp, #2048]
.L987:
	mov	r0, r9
	bl	NandcFlashDeCs
	ldrb	r2, [r6]	@ zero_extendqisi2
	sub	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L988
	ldrb	r1, [r6, #2]	@ zero_extendqisi2
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	ldr	r0, .L1081+16
	str	r1, [sp]
	ldrb	r1, [r6, #3]	@ zero_extendqisi2
	str	r1, [sp, #4]
	ldrb	r1, [r6, #4]	@ zero_extendqisi2
	str	r1, [sp, #8]
	ldrb	r1, [r6, #5]	@ zero_extendqisi2
	str	r1, [sp, #12]
	add	r1, r7, #1
	bl	printk
.L988:
	cmp	r7, #0
	bne	.L989
	ldrb	r3, [r8]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L1038
	ldr	r3, .L1081+8
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L1038
	bl	FlashCs123Init
.L989:
	ldrb	r3, [r6]	@ zero_extendqisi2
	add	r7, r7, #1
	add	r6, r6, #8
	cmp	r3, #181
	moveq	r3, #44
	streqb	r3, [r6, #-8]
	cmp	r7, #4
	bne	.L992
	ldrb	r3, [r8]	@ zero_extendqisi2
	cmp	r3, #173
	beq	.L993
	ldr	r0, [r4, #-3396]
	bl	NandcSetDdrMode
.L993:
	mov	r1, #0
	mov	r2, #852
	ldr	r0, .L1081+20
	bl	ftl_memset
	ldr	r6, .L1081+24
	ldr	r2, [r4, #-3388]
	ldr	r3, .L1081+28
	cmp	r2, r6
	ldrb	r0, [r8, #1]	@ zero_extendqisi2
	str	r3, [r5, #1784]
	mov	r3, #0
	strb	r3, [r5, #8]
	sub	lr, r0, #218
	ldreq	r3, .L1081
	clz	lr, lr
	moveq	r1, #1
	mov	lr, lr, lsr #5
	streq	r1, [r3, #-3404]
	cmp	r0, #161
	cmpne	r0, #241
	and	r1, r0, #253
	moveq	r3, #1
	movne	r3, #0
	orr	r3, lr, r3
	cmp	r1, #209
	orreq	r3, r3, #1
	cmp	r3, #0
	bne	.L995
	cmp	r0, #220
	bne	.L996
	ldr	r3, .L1081+8
	ldrb	r3, [r3, #3]	@ zero_extendqisi2
	cmp	r3, #149
	bne	.L996
.L995:
	ldrb	r7, [r8]	@ zero_extendqisi2
	mov	r3, #16
	strb	r3, [r5, #1]
	mov	ip, #1
	strb	r3, [r4, #-3311]
	cmp	r7, #152
	ldr	r3, .L1081+32
	strb	ip, [r5]
	ldr	r1, .L1081
	strb	r7, [r3, #3253]
	strb	r0, [r3, #3254]
	bne	.L998
	ldr	r7, .L1081+8
	ldrsb	r7, [r7, #4]
	cmp	r7, #0
	strltb	ip, [r1, #-3292]
	movge	ip, #24
	strgeb	ip, [r1, #-3311]
.L998:
	movw	r1, #2049
	cmp	r2, r6
	cmpne	r2, r1
	moveq	r2, #16
	streqb	r2, [r4, #-3311]
	cmp	lr, #0
	ldrne	r2, .L1081+36
	movne	r1, #2048
	strneh	r1, [r2, #14]	@ movhi
	mvnne	r2, #37
	bne	.L1075
.L1002:
	cmp	r0, #220
	bne	.L1004
	ldr	r2, .L1081+36
	mov	r1, #4096
	strh	r1, [r2, #14]	@ movhi
	mvn	r2, #35
.L1075:
	strb	r2, [r3, #3254]
	b	.L1003
.L1004:
	cmp	r0, #211
	ldreq	r2, .L1081+36
	moveq	r1, #4096
	streqh	r1, [r2, #14]	@ movhi
	moveq	r2, #2
	streqb	r2, [r3, #3265]
.L1003:
	ldr	r1, .L1081+40
	mov	r2, #32
	ldr	r0, .L1081+44
	bl	ftl_memcpy
	ldr	r0, .L1081+28
	ldr	r1, .L1081+36
	mov	r2, #32
	bl	ftl_memcpy
.L996:
	ldrb	r3, [r5]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1005
	bl	FlashLoadPhyInfoInRam
	cmp	r0, #0
	bne	.L1007
	ldr	r3, .L1081+4
	ldr	r7, .L1081
	ldr	r3, [r3, #1784]
	ldrh	r3, [r3, #16]
	mov	r3, r3, lsr #8
	tst	r3, #1
	and	r0, r3, #7
	strb	r0, [r4, #-3412]
	bne	.L1007
	mov	r3, #1
	strb	r3, [r7, #-3400]
	bl	FlashSetInterfaceMode
	ldrb	r0, [r7, #-3412]	@ zero_extendqisi2
	bl	NandcSetMode
.L1007:
	ldr	r3, [r5, #1784]
	ldrb	r3, [r3, #26]	@ zero_extendqisi2
	strb	r3, [r5, #3904]
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1005
	ldr	r3, [r4, #-3396]
	ldr	r7, .L1081
	cmp	r3, #0
	beq	.L1010
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	b	.L1076
.L1010:
	ldrb	r0, [r7, #-3412]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r7, #-3412]	@ zero_extendqisi2
.L1076:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1005
	mov	r0, #1
	ldr	r7, .L1081+4
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r3, [r5, #1784]
	ldr	r0, .L1081+48
	ldrh	r1, [r3, #14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	cmn	r0, #1
	beq	.L1051
	bl	FlashDieInfoInit
	ldr	r3, [r7, #1784]
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	bl	FlashGetReadRetryDefault
	movw	r3, #1788
	ldr	r2, [r7, #1784]
	ldrh	r3, [r7, r3]
	add	r3, r3, #4080
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	add	r3, r3, #15
	cmp	r1, r3, asr #12
	ldrh	r3, [r2, #14]
	blt	.L1012
	add	r0, r3, #255
	cmp	r1, r0, asr #8
	bge	.L1013
.L1012:
	bic	r3, r3, #255
	strh	r3, [r2, #14]	@ movhi
.L1013:
	ldrb	r3, [r4, #-3412]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1014
	bl	FlashSavePhyInfo
	ldr	r3, .L1081
	mov	r0, #0
	ldr	r1, [r3, #-3320]
	bl	FlashDdrParaScan
.L1014:
	bl	FlashSavePhyInfo
.L1005:
	ldr	r2, [r5, #1784]
	ldr	r9, .L1081+4
	ldr	r7, .L1081
	ldrb	r3, [r2, #26]	@ zero_extendqisi2
	ldrh	r0, [r2, #10]
	ldrb	r10, [r2, #18]	@ zero_extendqisi2
	strb	r3, [r5, #3904]
	ldrh	r3, [r2, #16]
	ubfx	r1, r3, #7, #1
	strb	r1, [r5, #8]
	ubfx	r1, r3, #3, #1
	strb	r1, [r4, #-3291]
	ubfx	r1, r3, #4, #1
	ubfx	r3, r3, #8, #3
	strb	r1, [r4, #-3340]
	strb	r3, [r4, #-3412]
	mov	r3, #0
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	str	r3, [r4, #-3328]
	bl	__aeabi_idiv
	mov	r1, r0
	mov	r0, r10
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	ldr	r3, [r5, #1784]
	ldrh	r2, [r3, #16]
	tst	r2, #64
	beq	.L1016
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	ldr	r3, .L1081+52
	ldrb	r2, [r9, #3917]	@ zero_extendqisi2
	ldr	r1, .L1081+56
	strb	r0, [r3]
	ldr	r3, .L1081+60
	strb	r2, [r3]
	mov	ip, r3
	ldrb	r2, [r9, #3918]	@ zero_extendqisi2
	strb	r2, [r1]
	sub	r2, r0, #1
	cmp	r2, #7
	bhi	.L1017
	ldr	r3, .L1081+64
	sub	r2, r0, #5
	cmp	r0, #8
	cmpne	r2, #1
	sub	r1, r0, #8
	clz	r1, r1
	str	r3, [r7, #-3328]
	movls	r3, #1
	strls	r3, [r7, #-3348]
	cmp	r0, #7
	mov	r1, r1, lsr #5
	ldreq	r3, .L1081+68
	beq	.L1020
	ldr	r2, .L1081+68
	cmp	r1, #0
	sub	r3, r2, #8
	movne	r3, r2
.L1020:
	sub	r1, r3, #1
	add	r3, r3, #31
	mov	r2, #0
.L1021:
	ldrsb	ip, [r1, #1]!
	cmp	ip, #0
	addeq	r2, r2, #1
	cmp	r1, r3
	bne	.L1021
	cmp	r2, #27
	bls	.L1016
	bl	FlashGetReadRetryDefault
	bl	FlashSavePhyInfo
	b	.L1016
.L1017:
	sub	r2, r0, #17
	cmp	r2, #2
	bhi	.L1023
	ldr	r3, .L1081+72
	cmp	r0, #19
	str	r3, [r7, #-3328]
	moveq	r3, #15
	bne	.L1080
	b	.L1078
.L1023:
	sub	r2, r0, #65
	cmp	r0, #33
	cmpne	r2, #1
	bhi	.L1025
	ldr	r2, .L1081+76
	str	r2, [r7, #-3328]
	mov	r2, #4
	strb	r2, [r3]
.L1080:
	mov	r3, #7
.L1078:
	strb	r3, [r1]
	b	.L1016
.L1025:
	sub	r3, r0, #67
	sub	r2, r0, #34
	cmp	r3, #1
	movhi	lr, #0
	movls	lr, #1
	cmp	r3, #1
	cmphi	r2, #1
	bhi	.L1026
	ldr	r3, .L1081+76
	cmp	r0, #68
	cmpne	r0, #35
	str	r3, [r7, #-3328]
	movne	r3, #7
	moveq	r3, #17
	cmp	lr, #0
	strb	r3, [r1]
	movne	r3, #4
	moveq	r3, #5
	strb	r3, [ip]
	b	.L1016
.L1026:
	cmp	r0, #49
	ldreq	r3, .L1081+80
	streq	r3, [r7, #-3328]
.L1016:
	ldr	r3, [r4, #-3388]
	cmp	r3, r6
	bne	.L1030
	ldrb	r2, [r5, #3904]	@ zero_extendqisi2
	cmp	r2, #0
	ldrne	r2, .L1081+4
	movne	r1, #0
	ldrne	r2, [r2, #1784]
	strneb	r1, [r2, #18]
.L1030:
	ldrb	r2, [r8]	@ zero_extendqisi2
	cmp	r2, #44
	bne	.L1031
	ldrb	r2, [r4, #-3400]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1031
	cmp	r3, r6
	bne	.L1032
	ldrb	r3, [r5, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1031
.L1032:
	mov	r0, #1
	mov	r3, #0
	strb	r3, [r4, #-3400]
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
.L1031:
	ldrb	r3, [r4, #-3412]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1033
	ldr	r2, .L1081
	ldrb	r2, [r2, #-3400]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1034
	tst	r3, #1
	bne	.L1033
.L1034:
	mov	r0, #0
	bl	flash_enter_slc_mode
	mov	r0, #0
	ldr	r1, [r4, #-3320]
	bl	FlashDdrParaScan
	mov	r0, #0
	bl	flash_exit_slc_mode
.L1033:
	ldr	r3, [r5, #1784]
	mov	r9, #16
	ldr	r7, .L1081+4
	ldr	r4, .L1081+84
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	add	r0, r7, #1792
	bl	FlashReadIdbDataRaw
	ldr	r0, .L1081+88
	strb	r9, [r5, #1]
	bl	FlashTimingCfg
	ldr	r6, [r5, #1784]
	ldrb	r3, [r6, #12]	@ zero_extendqisi2
	strh	r3, [r4, #8]	@ movhi
	ldrb	r3, [r6, #7]	@ zero_extendqisi2
	str	r3, [r5, #3880]
	ldr	r3, [r8]	@ unaligned
	ldrh	r8, [r6, #14]
	str	r3, [r5, #3876]
	ldrb	r3, [r5, #1612]	@ zero_extendqisi2
	strh	r3, [r4, #10]	@ movhi
	ldrb	r3, [r6, #13]	@ zero_extendqisi2
	strh	r8, [r4, #14]	@ movhi
	strh	r3, [r4, #12]	@ movhi
	ldrh	r3, [r6, #10]
	strh	r3, [r4, #16]	@ movhi
	ldrb	r1, [r6, #12]	@ zero_extendqisi2
	ldrh	r0, [r6, #10]
	bl	__aeabi_idiv
	strh	r0, [r4, #18]	@ movhi
	ldrb	r2, [r6, #9]	@ zero_extendqisi2
	strh	r2, [r4, #20]	@ movhi
	ldrb	r1, [r6, #9]	@ zero_extendqisi2
	ldrh	r3, [r6, #10]
	smulbb	r3, r1, r3
	mov	r1, #512
	strh	r1, [r4, #24]	@ movhi
	ldrb	r1, [r5, #1]	@ zero_extendqisi2
	strh	r1, [r4, #26]	@ movhi
	uxth	r3, r3
	ldrb	r1, [r5]	@ zero_extendqisi2
	strh	r3, [r4, #22]	@ movhi
	cmp	r1, #1
	bne	.L1036
	mov	r3, r3, asl #1
	mov	r1, r8, lsr #1
	mov	r2, r2, asl #1
	strh	r3, [r4, #22]	@ movhi
	strb	r9, [r7, #1]
	mov	r3, #8
	strh	r1, [r4, #14]	@ movhi
	strh	r2, [r4, #20]	@ movhi
	strh	r3, [r4, #26]	@ movhi
.L1036:
	ldrb	r0, [r6, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	bl	ftl_flash_suspend
	mov	r0, #0
	b	.L1051
.L1038:
	mvn	r0, #1
.L1051:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1082:
	.align	2
.L1081:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	IDByte
	.word	.LANCHOR0+12
	.word	.LC13
	.word	.LANCHOR0+3916
	.word	1446522928
	.word	.LANCHOR1+256
	.word	.LANCHOR1
	.word	.LANCHOR1+3252
	.word	.LANCHOR1+2912
	.word	.LANCHOR0+44
	.word	.LC14
	.word	g_retryMode
	.word	g_maxRetryCount
	.word	g_maxRegNum
	.word	HynixReadRetrial
	.word	.LANCHOR0+3944
	.word	MicronReadRetrial
	.word	ToshibaReadRetrial
	.word	SamsungReadRetrial
	.word	.LANCHOR0+3876
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
	ldr	r3, .L1100
	mov	r8, r0
	mov	r9, r1
	ldr	r2, [r3, #1784]
	mov	r6, r3
	ldrb	r7, [r2, #19]	@ zero_extendqisi2
	ldrb	r2, [r3, #3904]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1084
	ldr	r3, .L1100+4
	ldr	r3, [r3, #-3404]
	cmp	r3, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1084:
	sub	r3, r7, #5
	cmp	r7, #68
	cmpne	r3, #3
	bls	.L1085
	sub	r3, r7, #19
	tst	r3, #239
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1085:
	ldr	r10, .L1100+8
	movw	fp, #65535
	ldr	r5, .L1100+4
.L1087:
	ldr	r3, [r6, #1784]
	ldrh	r3, [r3, #10]
	cmp	r3, r4
	bls	.L1099
	mov	r3, r4, asl #1
	ldrh	r3, [r10, r3]
	cmp	r3, fp
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
	cmp	r7, #8
	mov	r2, #32768
	ldr	r0, [r5, #-3308]
	moveq	r1, #0
	movne	r1, #255
	bl	ftl_memset
	ldr	r2, [r5, #-3308]
	add	r1, r4, r9
	add	r4, r4, #1
	mov	r0, r8
	mov	r3, r2
	uxth	r4, r4
	bl	FlashProgPage
	b	.L1087
.L1099:
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1101:
	.align	2
.L1100:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+588
	.fnend
	.size	FlashPageProgMsbFFData, .-FlashPageProgMsbFFData
	.align	2
	.global	FlashReadSlc2KPages
	.type	FlashReadSlc2KPages, %function
FlashReadSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1153
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
.L1103:
	ldr	r3, [sp, #16]
	cmp	r8, r3
	beq	.L1152
	ldr	r3, [sp, #16]
	add	r2, sp, #28
	mov	r0, r4
	ldr	r1, [sp, #20]
	rsb	r3, r8, r3
	ldr	r9, .L1153+4
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
	bcs	.L1105
	add	r3, r9, r3
	mov	r7, #0
	ldrb	r5, [r3, #1616]	@ zero_extendqisi2
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	bl	NandcFlashCs
.L1106:
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
	ldr	r3, .L1153+8
	ldrb	r3, [r3, #-3292]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r6, r0
	beq	.L1107
	mov	r0, r5
	bl	flash_read_ecc
	cmp	r0, #5
	movhi	r6, #256
.L1107:
	cmp	r7, #9
	cmnls	r6, #1
	moveq	r3, #1
	movne	r3, #0
	addeq	r7, r7, #1
	beq	.L1106
.L1108:
	cmp	r7, #0
	mov	r7, r3
	movne	r6, #256
.L1110:
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
	ldr	r3, .L1153+8
	ldrb	r2, [r3, #-3292]	@ zero_extendqisi2
	cmp	r2, #0
	mov	r10, r0
	beq	.L1113
	mov	r0, r5
	bl	flash_read_ecc
	cmp	r0, #5
	movhi	r10, #256
.L1113:
	cmp	r7, #9
	cmnls	r10, #1
	addeq	r7, r7, #1
	beq	.L1110
.L1114:
	cmp	r7, #0
	mov	r0, r5
	movne	r10, #256
	bl	NandcFlashDeCs
	ldrb	r2, [r9, #3905]	@ zero_extendqisi2
	cmp	r10, r6
	movcs	r3, r10
	movcc	r3, r6
	add	r2, r2, r2, asl #1
	cmp	r3, r2, asr #2
	bls	.L1116
	cmn	r3, #1
	movne	r3, #256
.L1116:
	cmp	r3, #256
	cmnne	r3, #1
	movne	r3, #0
	str	r3, [r4]
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1119
	ldr	r2, [r3, #12]
	cmn	r2, #1
	bne	.L1119
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L1119
	ldr	r3, [r3]
	cmn	r3, #1
	strne	r2, [r4]
.L1119:
	ldr	r3, [r4]
	cmn	r3, #1
	bne	.L1105
	ldr	r1, [r4, #4]
	ldr	r0, .L1153+12
	ldrb	r2, [r9, #3905]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r4, #8]
	cmp	r1, #0
	beq	.L1121
	ldr	r0, .L1153+16
	mov	r2, #4
	mov	r3, #8
	bl	rknand_print_hex
.L1121:
	ldr	r1, [r4, #12]
	cmp	r1, #0
	beq	.L1105
	mov	r2, #4
	ldr	r0, .L1153+20
	mov	r3, r2
	bl	rknand_print_hex
.L1105:
	add	r8, r8, #1
	add	r4, r4, #36
	b	.L1103
.L1152:
	mov	r0, #0
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1154:
	.align	2
.L1153:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC15
	.word	.LC16
	.word	.LC17
	.fnend
	.size	FlashReadSlc2KPages, .-FlashReadSlc2KPages
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1230
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldr	r6, .L1230+4
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	str	r1, [sp, #20]
	ldrb	r8, [r6]	@ zero_extendqisi2
	str	r3, [sp, #16]
	ldrb	r3, [r6, #8]	@ zero_extendqisi2
	cmp	r8, #0
	str	r2, [sp, #24]
	str	r3, [sp, #28]
	beq	.L1192
	bl	FlashReadSlc2KPages
	b	.L1225
.L1192:
	ldr	r10, .L1230+8
	mov	r9, r0
	mov	fp, r8
.L1156:
	ldr	r3, [sp, #20]
	cmp	fp, r3
	bcs	.L1228
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
	bcs	.L1159
	add	r3, r6, r3
	ldrb	r4, [r3, #1616]	@ zero_extendqisi2
	ldrb	r3, [r10, #-3291]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r0, r4
	moveq	r5, #0
	bl	NandcWaitFlashReady
	ldr	r3, .L1230+4
	ldr	r3, [r3, #1784]
	ldrb	r2, [r3, #19]	@ zero_extendqisi2
	sub	r3, r2, #1
	cmp	r3, #7
	bhi	.L1161
	sub	r2, r2, #7
	add	r1, r6, r4
	cmp	r2, #1
	add	r2, r10, r4
	ldrb	r3, [r1, #3928]	@ zero_extendqisi2
	ldrb	r2, [r2, #-3416]	@ zero_extendqisi2
	ldrlsb	r3, [r1, #3936]	@ zero_extendqisi2
	cmp	r2, r3
	beq	.L1161
	mov	r0, r4
	ldrb	r1, [r6, #3917]	@ zero_extendqisi2
	ldr	r2, .L1230+12
	bl	HynixSetRRPara
.L1161:
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
	beq	.L1163
	ldrb	r3, [r6, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1163
	bl	flash_enter_slc_mode
	b	.L1164
.L1163:
	bl	flash_exit_slc_mode
.L1164:
	ldr	r1, [sp, #44]
	cmn	r1, #1
	cmpeq	r4, #255
	moveq	r3, #0
	movne	r3, #1
	moveq	r5, r3
	beq	.L1166
	cmp	r5, #0
	beq	.L1167
	ldr	r2, [r6, #4]
	mov	r0, r4
	add	r2, r1, r2
	bl	FlashReadDpCmd
	b	.L1168
.L1167:
	mov	r0, r4
	bl	FlashReadCmd
.L1168:
	mov	r0, r4
	bl	NandcWaitFlashReady
	cmp	r5, #0
	beq	.L1166
	mov	r0, r4
	ldr	r1, [sp, #44]
	bl	FlashReadDpDataOutCmd
.L1166:
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
	bne	.L1164
.L1169:
	cmp	r5, #0
	beq	.L1170
	ldr	r3, .L1230+4
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
.L1170:
	mov	r0, r4
	str	ip, [sp, #32]
	bl	NandcFlashDeCs
	ldr	ip, [sp, #32]
	ldrb	r3, [sp, #28]	@ zero_extendqisi2
	cmn	ip, #1
	strb	r3, [r6, #8]
	bne	.L1177
	ldrb	r3, [r10, #-3400]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1172
.L1176:
	ldr	r5, [r10, #-3328]
	cmp	r5, #0
	bne	.L1173
	b	.L1229
.L1172:
	ldr	r3, [r6, #3840]
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
	beq	.L1175
	ldrb	r3, [r6, #3905]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1195
.L1175:
	ubfx	r0, r5, #8, #8
	str	ip, [sp, #32]
	bl	NandcSetDdrPara
	ldr	ip, [sp, #32]
	cmn	ip, #1
	beq	.L1176
	b	.L1195
.L1173:
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	blx	r5
	cmn	r0, #1
	mov	ip, r0
	bne	.L1197
	ldr	r3, [r6, #1784]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	cmp	r3, #7
	bhi	.L1179
	mov	r0, r4
	ldrb	r1, [r6, #3917]	@ zero_extendqisi2
	ldr	r2, .L1230+12
	mov	r3, #0
	bl	HynixSetRRPara
.L1179:
	ldr	r1, [sp, #44]
	mov	r0, r4
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashReadRawPage
	ldr	r1, [r7, #4]
	ldrb	r2, [r6, #3905]	@ zero_extendqisi2
	mov	ip, r0
	ldr	r0, .L1230+16
	mov	r3, ip
	str	ip, [sp, #32]
	bl	printk
	ldr	ip, [sp, #32]
	cmn	ip, #1
	bne	.L1197
	ldrb	r5, [r6, #3904]	@ zero_extendqisi2
	cmp	r5, #0
	beq	.L1178
	ldr	r3, [sp, #12]
	mov	r0, r4
	cmp	r3, #0
	beq	.L1180
	bl	flash_enter_slc_mode
	b	.L1181
.L1180:
	bl	flash_exit_slc_mode
.L1181:
	ldr	ip, [r10, #-3328]
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	blx	ip
	mov	ip, r0
	b	.L1197
.L1229:
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r7, #8]
	ldr	r3, [r7, #12]
	bl	FlashReadRawPage
	mov	ip, r0
	b	.L1178
.L1195:
	mov	r5, #0
.L1177:
	ldrb	r3, [r6, #3905]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	ip, r3, asr #2
	bls	.L1178
	ldr	r3, [r10, #-3328]
	cmp	r3, #0
	moveq	ip, #256
	b	.L1178
.L1197:
	mov	r5, #0
.L1178:
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
	bne	.L1185
	ldr	r1, [r7, #4]
	ldr	r0, .L1230+20
	ldrb	r2, [r6, #3905]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r7, #12]
	cmp	r1, #0
	beq	.L1185
	mov	r2, #4
	ldr	r0, .L1230+24
	mov	r3, r2
	bl	rknand_print_hex
.L1185:
	cmp	r5, #0
	beq	.L1187
	ldrb	r3, [r6, #3905]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r8, r3, asr #2
	bls	.L1188
	ldr	r3, [r10, #-3328]
	cmp	r3, #0
	moveq	r8, #256
.L1188:
	ldr	r3, [sp, #8]
	cmp	r8, #256
	cmnne	r8, #1
	add	r3, r3, #36
	movne	r2, #0
	streq	r8, [r9, r3]
	strne	r2, [r9, r3]
.L1187:
	ldr	r3, [sp, #12]
	add	fp, fp, r5
	cmp	r3, #0
	beq	.L1159
	ldrb	r3, [r6, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1159
	mov	r0, r4
	bl	flash_exit_slc_mode
.L1159:
	add	fp, fp, #1
	b	.L1156
.L1228:
	mov	r0, #0
.L1225:
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1231:
	.align	2
.L1230:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+3920
	.word	.LC18
	.word	.LC15
	.word	.LC17
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
	mov	r1, #0
	ldr	r4, .L1245
	.pad #52
	sub	sp, sp, #52
	ldr	r0, .L1245+4
	mov	r2, #16
	ldr	r9, .L1245+8
	mov	r5, #0
	ldrh	r3, [r4, #14]
	mov	r8, r5
	ldrh	r6, [r4, #12]
	mvn	r10, #0
	ldr	fp, .L1245+12
	smulbb	r6, r6, r3
	bl	ftl_memset
	ldr	r3, [r9, #-3304]
	uxth	r6, r6
	str	r5, [sp, #20]
	str	r3, [sp, #24]
.L1233:
	ldrb	r3, [fp, #1612]	@ zero_extendqisi2
	uxtb	r7, r5
	cmp	r3, r7
	bls	.L1244
	mul	ip, r6, r7
	sub	r4, r6, #1
	sub	r3, r6, #12
	uxth	r4, r4
.L1234:
	cmp	r4, r3
	ble	.L1236
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
	beq	.L1235
	ldr	r2, [r9, #-3304]
	ldrh	r1, [r2]
	movw	r2, #61664
	cmp	r1, r2
	bne	.L1235
	mov	r1, r7
	ldr	r0, .L1245+16
	mov	r2, r4
	mov	r7, r7, asl #1
	bl	printk
	ldr	r3, .L1245+4
	add	r8, r8, #1
	strh	r4, [r3, r7]	@ movhi
	uxth	r8, r8
	b	.L1236
.L1235:
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1234
.L1236:
	ldr	r3, .L1245+12
	add	r5, r5, #1
	ldrb	r3, [r3, #1612]	@ zero_extendqisi2
	cmp	r3, r8
	moveq	r10, #0
	b	.L1233
.L1244:
	mov	r0, r10
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1246:
	.align	2
.L1245:
	.word	.LANCHOR0+3876
	.word	.LANCHOR2-3288
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC19
	.fnend
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.align	2
	.global	FlashReadFacBbtData
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1262
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r8, r2
	ldrh	r2, [r3, #12]
	.pad #40
	sub	sp, sp, #40
	ldrh	r3, [r3, #14]
	mov	r5, r0
	ldr	r9, .L1262+4
	mov	r7, r1
	smulbb	r3, r2, r3
	ldr	r2, [r9, #-3408]
	str	r2, [sp, #12]
	ldr	r2, [r9, #-3304]
	uxth	r3, r3
	sub	r6, r3, #1
	sub	r4, r3, #16
	mul	r10, r3, r1
	uxth	r6, r6
	str	r2, [sp, #16]
.L1248:
	cmp	r6, r4
	ble	.L1261
	mov	r1, #1
	add	r3, r6, r10
	add	r0, sp, #4
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L1249
	ldr	r3, [r9, #-3304]
	ldrh	r2, [r3]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L1249
	cmp	r5, #0
	moveq	r0, r5
	beq	.L1250
	cmp	r7, #0
	ldreq	ip, .L1262+4
	moveq	lr, #1
	beq	.L1251
.L1253:
	ldr	r1, [r9, #-3408]
	mov	r2, r8
	mov	r0, r5
	bl	ftl_memcpy
	mov	r2, #4
	ldr	r0, .L1262+8
	mov	r1, r5
	mov	r3, r2
	bl	rknand_print_hex
	mov	r0, #0
	b	.L1250
.L1251:
	ldr	r2, [r9, #-3316]
	uxth	r3, r7
	add	r7, r7, #1
	cmp	r3, r2
	bcs	.L1253
	ldr	r1, [ip, #-3408]
	mov	r0, r3, lsr #5
	and	r4, r3, #31
	ldr	r2, [r1, r0, asl #2]
	orr	r4, r2, lr, asl r4
	str	r4, [r1, r0, asl #2]
	b	.L1251
.L1249:
	sub	r6, r6, #1
	uxth	r6, r6
	b	.L1248
.L1261:
	mvn	r0, #0
.L1250:
	add	sp, sp, #40
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L1263:
	.align	2
.L1262:
	.word	.LANCHOR0+3876
	.word	.LANCHOR2
	.word	.LC20
	.fnend
	.size	FlashReadFacBbtData, .-FlashReadFacBbtData
	.align	2
	.global	FlashGetBadBlockList
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1275
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r3, [r3, #1784]
	ldr	r6, .L1275+4
	ldrb	r4, [r3, #13]	@ zero_extendqisi2
	ldrh	r3, [r3, #14]
	ldr	r0, [r6, #-3308]
	smulbb	r4, r4, r3
	uxth	r4, r4
	add	r2, r4, #7
	mov	r2, r2, asr #3
	bl	FlashReadFacBbtData
	cmn	r0, #1
	bne	.L1265
.L1269:
	mov	r3, #0
	b	.L1266
.L1265:
	mov	lr, r4, lsr #4
	mov	ip, #0
	sub	r4, r4, #1
	mov	r3, ip
	mov	r7, #1
.L1267:
	uxth	r0, ip
	cmp	r0, r4
	bge	.L1266
	ldr	r8, [r6, #-3308]
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
	bcc	.L1267
	b	.L1269
.L1266:
	mov	r3, r3, asl #1
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r5, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1276:
	.align	2
.L1275:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
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
	ldr	r8, .L1308
	mov	r6, r1
	str	r3, [sp, #12]
	mov	r9, r2
	ldr	r3, .L1308+4
	mov	r4, r0
	mov	r10, r0
	mov	r7, #0
	mov	fp, r8
	ldrb	r3, [r3, #265]	@ zero_extendqisi2
	str	r3, [sp, #8]
.L1278:
	cmp	r7, r6
	beq	.L1307
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
	bcs	.L1280
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
.L1280:
	add	r7, r7, #1
	add	r10, r10, #36
	b	.L1278
.L1307:
	ldr	r3, [sp, #12]
	cmp	r3, #0
	beq	.L1305
	ldr	r5, .L1308+8
	mov	r7, #0
	mov	r8, r5
.L1285:
	cmp	r7, r6
	beq	.L1305
	ldr	r3, [r4]
	cmn	r3, #1
	bne	.L1286
	ldr	r1, [r4, #4]
	ldr	r0, .L1308+12
	bl	printk
	b	.L1287
.L1286:
	rsb	r3, r7, r6
	mov	r1, r9
	add	r2, sp, #20
	mov	r0, r4
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #24
	bl	LogAddr2PhyAddr
	ldr	r2, [r5, #-3300]
	mov	r3, #0
	mov	lr, r4
	str	r3, [r2]
	ldr	r2, [r5, #-3296]
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
	ldr	r3, [r5, #-3300]
	str	r3, [sp, #36]
	ldr	r3, [r5, #-3296]
	str	r3, [sp, #40]
	bl	FlashReadPages
	ldr	r10, [sp, #28]
	cmn	r10, #1
	bne	.L1288
	ldr	r0, .L1308+16
	ldr	r1, [r4, #4]
	bl	printk
	str	r10, [r4]
.L1288:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1289
	ldr	r2, [r3]
	ldr	r3, [r8, #-3296]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1289
	ldr	r0, .L1308+20
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L1289:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L1287
	ldr	r2, [r3]
	ldr	r3, [r8, #-3300]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1287
	ldr	r0, .L1308+24
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L1287:
	add	r7, r7, #1
	add	r4, r4, #36
	b	.L1285
.L1305:
	mov	r0, #0
	add	sp, sp, #68
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1309:
	.align	2
.L1308:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LC21
	.word	.LC22
	.word	.LC23
	.word	.LC24
	.fnend
	.size	FlashProgSlc2KPages, .-FlashProgSlc2KPages
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
	ldr	r6, .L1367
	mov	r4, r0
	str	r1, [sp, #8]
	mov	r9, r2
	str	r3, [sp, #20]
	ldr	ip, [r6, #1784]
	ldrb	r8, [r6]	@ zero_extendqisi2
	ldrb	ip, [ip, #19]	@ zero_extendqisi2
	cmp	r8, #0
	str	ip, [sp, #16]
	ldr	ip, .L1367+4
	ldrb	ip, [ip, #265]	@ zero_extendqisi2
	str	ip, [sp, #12]
	beq	.L1311
	bl	FlashProgSlc2KPages
	b	.L1312
.L1325:
	mov	r7, #36
	ldr	r3, [sp, #8]
	mul	r7, r7, r8
	rsb	r3, r8, r3
	add	r2, sp, #28
	mov	r1, r9
	uxtb	r3, r3
	str	r3, [sp]
	add	fp, r4, r7
	add	r3, sp, #32
	mov	r0, fp
	bl	LogAddr2PhyAddr
	ldrb	r2, [r6, #1612]	@ zero_extendqisi2
	ldr	r3, [sp, #32]
	mov	r10, r0
	cmp	r3, r2
	mvncs	r3, #0
	strcs	r3, [r4, r7]
	bcc	.L1364
.L1314:
	add	r8, r8, #1
.L1311:
	ldr	r3, [sp, #8]
	ldr	r5, .L1367
	cmp	r8, r3
	bcc	.L1325
	b	.L1365
.L1364:
	ldr	r1, .L1367+8
	add	r3, r6, r3, asl #4
	ldr	r3, [r3, #1664]
	ldrb	r1, [r1, #-3340]	@ zero_extendqisi2
	cmp	r1, #0
	moveq	r10, #0
	cmp	r3, #0
	beq	.L1316
	cmp	r2, #1
	bne	.L1317
	ldr	r0, [r6, #3840]
	bl	NandcIqrWaitFlashReady
.L1317:
	ldrb	r0, [sp, #32]	@ zero_extendqisi2
	bl	FlashWaitCmdDone
.L1316:
	ldr	r3, [sp, #32]
	mov	r1, #0
	cmp	r10, #0
	add	r2, r6, r3, asl #4
	str	r1, [r2, #1668]
	ldr	r1, [sp, #28]
	str	fp, [r2, #1664]
	str	r1, [r2, #1660]
	addne	r1, r7, #36
	addne	r1, r4, r1
	strne	r1, [r2, #1668]
	add	r2, r6, r3
	add	r3, r6, r3, asl #4
	ldrb	r5, [r2, #1616]	@ zero_extendqisi2
	strb	r5, [r3, #1656]
	mov	r0, r5
	ldrb	r3, [r6, #1612]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1319
	bl	NandcWaitFlashReady
	b	.L1320
.L1319:
	bl	NandcFlashCs
	mov	r0, r5
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	add	r3, r6, r3, asl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r5
	bl	NandcFlashDeCs
.L1320:
	ldr	r3, [sp, #16]
	sub	r3, r3, #1
	cmp	r3, #7
	bhi	.L1321
	ldr	r3, .L1367+8
	add	r3, r3, r5
	ldrb	r3, [r3, #-3416]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1321
	mov	r0, r5
	ldrb	r1, [r6, #3917]	@ zero_extendqisi2
	ldr	r2, .L1367+12
	mov	r3, #0
	bl	HynixSetRRPara
.L1321:
	mov	r0, r5
	bl	NandcFlashCs
	cmp	r9, #1
	mov	r0, r5
	bne	.L1322
	ldrb	r3, [r6, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1322
	bl	flash_enter_slc_mode
	b	.L1323
.L1322:
	bl	flash_exit_slc_mode
.L1323:
	mov	r0, r5
	ldr	r1, [sp, #28]
	bl	FlashProgFirstCmd
	ldr	r3, [fp, #12]
	mov	r0, r5
	mov	r1, #1
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [fp, #8]
	bl	NandcXferData
	cmp	r10, #0
	beq	.L1324
	mov	r0, r5
	ldr	r1, [sp, #28]
	bl	FlashProgDpFirstCmd
	mov	r0, r5
	add	r7, r7, #36
	add	r7, r4, r7
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	add	r3, r6, r3, asl #2
	ldr	r2, [r3, #1624]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	ldr	r1, [r6, #4]
	mov	r0, r5
	ldr	r3, [sp, #28]
	add	r1, r1, r3
	bl	FlashProgDpSecondCmd
	ldr	r3, [r7, #12]
	mov	r0, r5
	mov	r1, #1
	str	r3, [sp]
	ldr	r2, [sp, #12]
	ldr	r3, [r7, #8]
	bl	NandcXferData
.L1324:
	mov	r0, r5
	ldr	r1, [sp, #28]
	bl	FlashProgSecondCmd
	mov	r0, r5
	bl	NandcFlashDeCs
	add	r8, r8, r10
	b	.L1314
.L1365:
	ldr	r0, [r5, #3840]
	mov	r6, #0
	bl	NandcIqrWaitFlashReady
	ldr	r8, .L1367+16
	mov	r7, r5
.L1326:
	ldrb	r3, [r5, #1612]	@ zero_extendqisi2
	cmp	r6, r3
	bcs	.L1366
	uxtb	r0, r6
	bl	FlashWaitCmdDone
	cmp	r9, #1
	bne	.L1327
	ldrb	r3, [r7, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1327
	ldrb	r0, [r8, r6, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L1327:
	add	r6, r6, #1
	b	.L1326
.L1366:
	ldr	r3, [sp, #20]
	cmp	r3, #0
	bne	.L1329
.L1337:
	mov	r0, #0
	b	.L1312
.L1329:
	ldr	r5, .L1367+8
	mov	r6, #0
	mov	r7, r5
.L1330:
	ldr	r3, [sp, #8]
	cmp	r6, r3
	beq	.L1337
	ldr	r3, [r4]
	cmn	r3, #1
	bne	.L1331
	ldr	r1, [r4, #4]
	ldr	r0, .L1367+20
	bl	printk
	b	.L1332
.L1331:
	ldr	r3, [sp, #8]
	mov	r1, r9
	add	r2, sp, #28
	mov	r0, r4
	rsb	r3, r6, r3
	uxtb	r3, r3
	str	r3, [sp]
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldr	r2, [r5, #-3300]
	mov	r3, #0
	mov	lr, r4
	str	r3, [r2]
	ldr	r2, [r5, #-3296]
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
	ldr	r3, [r5, #-3300]
	str	r3, [sp, #44]
	ldr	r3, [r5, #-3296]
	str	r3, [sp, #48]
	bl	FlashReadPages
	ldr	r8, [sp, #36]
	cmn	r8, #1
	bne	.L1333
	ldr	r0, .L1367+24
	ldr	r1, [r4, #4]
	bl	printk
	str	r8, [r4]
.L1333:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1334
	ldr	r2, [r3]
	ldr	r3, [r7, #-3296]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1334
	ldr	r0, .L1367+28
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L1334:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L1332
	ldr	r2, [r3]
	ldr	r3, [r7, #-3300]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L1332
	ldr	r0, .L1367+32
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4]
.L1332:
	add	r6, r6, #1
	add	r4, r4, #36
	b	.L1330
.L1312:
	add	sp, sp, #76
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1368:
	.align	2
.L1367:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LANCHOR0+3920
	.word	.LANCHOR0+1656
	.word	.LC21
	.word	.LC22
	.word	.LC23
	.word	.LC24
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #108
	sub	sp, sp, #108
	ldr	r5, .L1373
	ldr	r3, [r5, #-3316]
	cmp	r0, r3
	movcc	r4, #0
	bcc	.L1370
	ldr	r3, [r5, #-3308]
	mov	r4, r0
	mov	r1, #165
	add	r0, sp, #40
	mov	r2, #32
	str	r0, [sp, #16]
	str	r3, [sp, #12]
	bl	ftl_memset
	mov	r1, #90
	mov	r2, #8
	ldr	r0, [r5, #-3308]
	mov	r4, r4, asl #10
	bl	ftl_memset
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
.L1370:
	mov	r0, r4
	add	sp, sp, #108
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L1374:
	.align	2
.L1373:
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
	ldr	r4, .L1430
	mov	r1, #1
	ldr	r6, .L1430+4
	ldr	r5, .L1430+8
	ldr	r3, [r4, #-3304]
	ldrh	r8, [r6, #12]
	ldr	r0, .L1430+12
	str	r3, [sp, #20]
	ldrh	r3, [r6, #14]
	ldr	r7, .L1430+8
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
	ldr	r0, [r4, #-3304]
	mov	r1, #0
	mov	r2, #4096
	bl	ftl_memset
	ldr	r4, .L1430
	mov	r3, r8, lsr #4
	str	r3, [sp, #28]
	mov	r3, #0
	str	r3, [sp, #8]
.L1377:
	ldrb	r6, [sp, #8]	@ zero_extendqisi2
	ldrb	r3, [r7, #1612]	@ zero_extendqisi2
	cmp	r3, r6
	bls	.L1426
	ldr	r2, .L1430+16
	mov	r3, r6, asl #1
	ldrh	r5, [r2, r3]
	cmp	r5, #0
	bne	.L1406
	ldr	r3, .L1430+4
	mov	r1, r5
	ldr	r0, [r4, #-3408]
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
.L1379:
	ldrh	r3, [sp, #4]
	cmp	r3, r8
	str	r3, [sp, #12]
	bcs	.L1388
	mvn	r3, #0
	strb	r3, [sp, #42]
	strb	r3, [sp, #43]
	ldr	r3, [sp, #24]
	tst	r3, #1
	beq	.L1381
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
	bne	.L1381
	ldr	r1, [r7, #4]
	add	r2, sp, #43
	mov	r0, r10
	add	r1, r3, r1
	bl	FlashReadSpare
	ldrb	r3, [sp, #42]	@ zero_extendqisi2
	ldrb	r2, [sp, #43]	@ zero_extendqisi2
	and	r3, r3, r2
	strb	r3, [sp, #42]
.L1381:
	ldr	r3, [sp, #24]
	tst	r3, #2
	beq	.L1383
	ldr	r3, [r7, #1784]
	mov	r0, r10
	add	r2, sp, #43
	ldrh	r1, [r3, #10]
	ldr	r3, [fp, #1624]
	sub	r1, r1, #1
	add	r1, r1, r3
	add	r1, r1, r5
	bl	FlashReadSpare
.L1383:
	ldr	r3, [r7, #1784]
	ldrb	r3, [r3, #7]	@ zero_extendqisi2
	cmp	r3, #1
	cmpne	r3, #8
	ldrb	r3, [sp, #42]	@ zero_extendqisi2
	bne	.L1384
	cmp	r3, #0
	ldrneb	r0, [sp, #43]	@ zero_extendqisi2
	clzne	r0, r0
	movne	r0, r0, lsr #5
	bne	.L1385
	b	.L1405
.L1384:
	cmp	r3, #255
	bne	.L1405
	ldrb	r0, [sp, #43]	@ zero_extendqisi2
	subs	r0, r0, #255
	movne	r0, #1
	b	.L1385
.L1405:
	mov	r0, #1
.L1385:
	ldr	r3, [sp, #24]
	tst	r3, #4
	beq	.L1386
	ldr	r3, .L1430+8
	mov	r0, r10
	ldr	r2, [sp, #32]
	add	r3, r3, r2
	ldr	r1, [r3, #1624]
	add	r1, r5, r1
	bl	SandiskProgTestBadBlock
.L1386:
	cmp	r0, #0
	beq	.L1387
	mov	r1, r6
	ldr	r2, [sp, #4]
	ldr	r0, .L1430+20
	add	r9, r9, #1
	bl	printk
	ldr	r1, [r4, #-3408]
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
	bgt	.L1388
.L1387:
	ldr	r3, [sp, #4]
	add	r3, r3, #1
	str	r3, [sp, #4]
	ldr	r3, [sp, #16]
	add	r5, r5, r3
	b	.L1379
.L1388:
	mov	r2, r9
	ldr	r0, .L1430+24
	mov	r1, r6
	bl	printk
	ldrb	r3, [r7, #1612]	@ zero_extendqisi2
	ldr	r2, [sp, #28]
	mul	r3, r3, r2
	cmp	r9, r3
	blt	.L1390
	ldr	r3, .L1430+4
	mov	r1, #0
	ldr	r0, [r4, #-3408]
	ldrh	r2, [r3, #20]
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1390:
	cmp	r6, #0
	bne	.L1392
	ldr	r3, [r4, #-3316]
	mov	r5, r6
	mov	r9, #1
	uxth	r10, r3
.L1393:
	ldr	r3, .L1430+8
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, r10
	bls	.L1427
	mov	r0, r10
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1394
	mov	r1, r10
	ldr	r0, .L1430+28
	bl	printk
	ldr	r1, [r4, #-3408]
	mov	r0, r10, lsr #5
	add	r5, r5, #1
	and	r3, r10, #31
	ldr	r2, [r1, r0, asl #2]
	uxth	r5, r5
	orr	r3, r2, r9, asl r3
	str	r3, [r1, r0, asl #2]
.L1394:
	add	r10, r10, #1
	uxth	r10, r10
	b	.L1393
.L1427:
	sub	fp, r8, #1
	sub	r9, r8, #50
	mov	r10, #1
	uxth	fp, fp
.L1396:
	cmp	fp, r9
	ble	.L1428
	mov	r0, fp
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1397
	mov	r1, fp
	ldr	r0, .L1430+28
	bl	printk
	ldr	r1, [r4, #-3408]
	mov	r0, fp, lsr #5
	and	r3, fp, #31
	ldr	r2, [r1, r0, asl #2]
	orr	r3, r2, r10, asl r3
	str	r3, [r1, r0, asl #2]
.L1397:
	sub	fp, fp, #1
	uxth	fp, fp
	b	.L1396
.L1428:
	ldr	r3, .L1430+8
	ldr	r2, [r4, #-3316]
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	rsb	r3, r2, r3
	cmp	r5, r3
	bcc	.L1392
	ldr	r3, .L1430+4
	mov	r1, #0
	ldr	r0, [r4, #-3408]
	ldrh	r2, [r3, #20]
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1392:
	ldrb	r5, [sp, #8]	@ zero_extendqisi2
	sub	r10, r8, #1
	ldr	r9, .L1430+16
	uxth	r10, r10
	mul	r5, r8, r5
	add	r9, r9, r6, asl #1
.L1400:
	mov	r1, r6
	ldr	r0, .L1430+32
	mov	r2, r10
	bl	printk
	ldr	r1, [r4, #-3408]
.L1401:
	mov	r2, r10, lsr #5
	and	r3, r10, #31
	ldr	r2, [r1, r2, asl #2]
	mov	r3, r2, lsr r3
	ands	r3, r3, #1
	subne	r10, r10, #1
	uxthne	r10, r10
	bne	.L1401
.L1429:
	ldr	r1, [sp, #20]
	add	r0, sp, #44
	ldr	r2, .L1430+36
	strh	r10, [r9]	@ movhi
	strh	r10, [r1, #2]	@ movhi
	strh	r2, [r1]	@ movhi
	strh	r3, [r1, #8]	@ movhi
	mov	r1, #1
	ldr	r3, [r4, #-3408]
	mov	r2, r1
	str	r3, [sp, #52]
	ldr	r3, [r4, #-3304]
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
	bne	.L1400
.L1406:
	ldr	r3, [sp, #8]
	add	r3, r3, #1
	str	r3, [sp, #8]
	b	.L1377
.L1426:
	add	sp, sp, #84
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1431:
	.align	2
.L1430:
	.word	.LANCHOR2
	.word	.LANCHOR0+3876
	.word	.LANCHOR0
	.word	.LC25
	.word	.LANCHOR2-3288
	.word	.LC26
	.word	.LC27
	.word	.LC28
	.word	.LC29
	.word	-3872
	.fnend
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
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
.L1433:
	cmp	r2, r0
	uxth	r3, r1
	add	r1, r1, #1
	movls	r2, r2, asl #1
	bls	.L1433
.L1435:
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
	ldr	r3, .L1439
	cmp	r0, #23
	sub	r2, r3, #3264
	movls	r0, #24
	sub	ip, r3, #3248
	str	r0, [r3, #-3272]
	ldrh	r2, [r2, #-4]
	ldrh	r1, [ip, #-10]
	mul	r2, r2, r0
	rsb	r0, r0, r1
	ldr	r1, [r3, #-3252]
	strh	r0, [ip, #-12]	@ movhi
	mov	r0, #0
	str	r2, [r3, #-3264]
	rsb	r2, r2, r1
	str	r2, [r3, #-3256]
	bx	lr
.L1440:
	.align	2
.L1439:
	.word	.LANCHOR2
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
	mov	r5, r0
	ldrh	r7, [r0, #8]
	.pad #20
	sub	sp, sp, #20
	ldr	r0, .L1469
	mov	r9, #0
	ldrh	ip, [r5, #10]
	ldrh	r6, [r5, #12]
	add	r3, r0, #8
	ldrh	r8, [r5, #14]
	mov	r4, r0
	strh	r7, [r4], #16	@ movhi
	str	r0, [sp]
	strh	ip, [r4, #-14]	@ movhi
	strh	r6, [r4, #-12]	@ movhi
	strh	r8, [r0, #-10]	@ movhi
.L1442:
	strb	r9, [r9, r3]
	add	r9, r9, #1
	cmp	r9, #32
	bne	.L1442
	ldrh	r2, [r5, #14]
	ldrh	r3, [r5, #20]
	cmp	r3, r2, lsr #8
	bcs	.L1443
	uxtb	r9, r6
	mov	r3, r9, asl #1
	uxtb	r3, r3
	str	r3, [sp, #8]
	sub	r3, ip, #1
	mul	r3, r6, r3
	str	r3, [sp, #4]
	mov	r3, #0
.L1444:
	cmp	r3, r6
	bcs	.L1446
	ldr	r1, [sp, #4]
	rsb	fp, r6, r3
	ldr	r0, .L1469+4
	uxtb	r2, r3
	ldr	lr, .L1469+4
	add	r1, r3, r1
	add	r1, r0, r1
	str	r1, [sp, #12]
	add	fp, lr, fp
	mov	r1, #0
	mov	r0, r1
.L1447:
	cmp	r0, ip
	add	r1, r1, r6
	bcs	.L1468
	ldr	lr, [sp, #12]
	add	r10, r2, r9
	strb	r2, [fp, r1]
	add	r0, r0, #1
	strb	r10, [lr, r1]
	ldr	lr, [sp, #8]
	add	r2, r2, lr
	uxtb	r2, r2
	b	.L1447
.L1468:
	add	r3, r3, #1
	b	.L1444
.L1446:
	ldr	r3, [sp]
	mov	ip, ip, asl #1
	mov	r8, r8, lsr #1
	strh	ip, [r4, #-14]	@ movhi
	strh	r8, [r3, #-10]	@ movhi
.L1443:
	ldr	r9, .L1469+8
	mov	r3, #5
	cmp	r7, #1
	ldr	ip, .L1469+12
	ldrh	fp, [r5, #16]
	strh	r3, [r9, #-8]	@ movhi
	mov	r3, #0
	strh	r3, [r9, #-6]	@ movhi
	mov	r3, #4352
	strh	r3, [r9, #-4]	@ movhi
	sub	r8, ip, #3200
	ldr	r3, .L1469+16
	streqh	r7, [r9, #-8]	@ movhi
	ldrh	r7, [r4, #-14]
	ldrb	r10, [r3]	@ zero_extendqisi2
	ldrh	r1, [r5, #18]
	cmp	r10, #0
	strh	fp, [r8]	@ movhi
	smulbb	r7, r7, r6
	ldrne	r3, .L1469+8
	movne	r2, #384
	str	ip, [sp, #12]
	str	r1, [sp, #8]
	strneh	r2, [r3, #-4]	@ movhi
	sub	r3, ip, #3264
	ldrh	r2, [r5, #20]
	uxth	r7, r7
	strh	r7, [r3, #-4]	@ movhi
	ldr	r3, [sp]
	mov	r0, r2
	str	r2, [sp, #4]
	ldrh	r4, [r3, #-10]
	smulbb	r3, r7, fp
	smulbb	r6, r4, r6
	strh	r6, [r8, #-2]	@ movhi
	sub	r6, ip, #3184
	strh	r1, [r6, #-14]	@ movhi
	strh	r3, [r6, #-12]	@ movhi
	strh	r2, [r6, #-10]	@ movhi
	bl	Ftl_log2
	cmp	r4, #1024
	ldr	r2, [sp, #4]
	mov	r3, r0
	strh	r0, [r6, #-8]	@ movhi
	ldr	ip, [sp, #12]
	mov	r0, r2, asl #9
	ldr	r1, [sp, #8]
	str	r3, [sp, #4]
	uxth	r0, r0
	strh	r0, [r6, #-6]	@ movhi
	mul	r1, r1, r2
	mov	r0, r0, lsr #8
	strh	r0, [r6, #-4]	@ movhi
	ldrh	r0, [r5, #26]
	mov	r5, ip
	strh	r0, [r6, #-2]	@ movhi
	mul	r0, r4, r7
	str	r0, [ip, #-3252]
	uxtbhi	r0, r4
	strhih	r0, [r8, #-6]	@ movhi
	ldrh	r0, [r8, #-6]
	rsb	r0, r0, r4
	mov	r4, r4, asl #6
	mul	r0, r0, r7
	mul	r0, r2, r0
	mul	fp, fp, r0
	ldrh	r0, [r8, #-4]
	mov	r0, r0, asl #3
	mov	fp, fp, asr #11
	str	fp, [ip, #-3184]
	bl	__aeabi_idiv
	ldr	fp, .L1469+20
	mov	r1, r7
	uxth	r0, r0
	ldr	r3, [sp, #4]
	cmp	r0, #4
	strhih	r0, [fp, #-12]	@ movhi
	add	r0, r3, #9
	movls	r2, #4
	strlsh	r2, [fp, #-12]	@ movhi
	cmp	r10, #0
	mov	r0, r4, asr r0
	strh	r0, [fp, #-8]	@ movhi
	movne	r2, #640
	strneh	r2, [r8, #-4]	@ movhi
	ldrh	r2, [r8, #-4]
	uxth	r0, r0
	add	r4, r0, #8
	ldr	r10, .L1469+12
	mov	r2, r2, asr r3
	mul	r3, r7, r0
	add	r2, r2, #2
	ldrh	r0, [fp, #-12]
	strh	r2, [fp, #-10]	@ movhi
	str	r3, [r5, #-3172]
	bl	__aeabi_uidiv
	cmp	r7, #1
	ldr	r7, .L1469+12
	sub	r8, r7, #3152
	uxtah	r0, r4, r0
	addeq	r0, r0, #4
	str	r0, [r10, #-3272]
	ldr	r3, [r5, #-3272]
	uxth	r0, r3
	bl	FtlSysBlkNumInit
	ldr	r3, [r5, #-3272]
	ldr	r2, [r5, #-3256]
	mov	r0, #2048
	ldrh	r4, [r6, #-8]
	str	r3, [r5, #-3168]
	ldrh	r3, [r9]
	mov	r2, r2, asl #2
	add	r4, r4, #9
	ldrh	r6, [r6, #-10]
	mul	r3, r3, r2
	mov	r1, r6
	mov	r4, r3, lsr r4
	add	r4, r4, #2
	uxth	r4, r4
	strh	r4, [r8, #-12]	@ movhi
	bl	__aeabi_idiv
	ldr	r1, .L1469+16
	mov	r3, #0
	str	r3, [r1, #3908]
	ldrh	r3, [fp, #-12]
	ldrb	r1, [r1, #3904]	@ zero_extendqisi2
	add	r2, r3, #3
	strh	r2, [fp, #-12]	@ movhi
	cmp	r1, #0
	ldr	fp, [r5, #-3172]
	add	r2, fp, #3
	str	r2, [r5, #-3172]
	strh	r0, [r8, #-10]	@ movhi
	sub	r0, r7, #3168
	addne	r3, r3, #4
	addne	fp, fp, #5
	strneh	r3, [r0, #-12]	@ movhi
	strne	fp, [r7, #-3172]
	bne	.L1457
.L1456:
	cmp	r2, #7
	movls	r3, #8
	strls	r3, [r7, #-3172]
.L1457:
	mov	r3, #0
	strh	r3, [r8, #-8]	@ movhi
	ldr	r3, [sp]
	mov	r0, #0
	ldrh	r2, [r3, #-12]
	mov	r3, r2, lsr #3
	add	r3, r3, r2, asl #1
	add	r3, r3, #52
	add	r4, r3, r4, asl #2
	cmp	r4, r6, asl #9
	ldrcc	r3, .L1469+24
	movcc	r2, #1
	strcch	r2, [r3, #-8]	@ movhi
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1470:
	.align	2
.L1469:
	.word	.LANCHOR2-3248
	.word	.LANCHOR2-3240
	.word	.LANCHOR2-3200
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2-3168
	.word	.LANCHOR2-3152
	.fnend
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, #0
	ldr	r4, .L1576
	movw	r3, #65535
	mov	r0, #1024
	mov	r10, #36
	sub	r2, r4, #3072
	sub	r6, r4, #3056
	sub	r7, r4, #3152
	str	r3, [r4, #-3088]
	strh	r5, [r6, #-4]	@ movhi
	mvn	r3, #0
	strh	r5, [r6, #-2]	@ movhi
	sub	r9, r4, #3264
	strh	r3, [r2]	@ movhi
	strh	r3, [r6, #-14]	@ movhi
	strh	r3, [r6, #-12]	@ movhi
	strh	r3, [r6, #-10]	@ movhi
	mov	r3, #32
	strh	r5, [r6]	@ movhi
	strh	r3, [r6, #-8]	@ movhi
	mov	r3, #128
	strh	r3, [r6, #-6]	@ movhi
	sub	r3, r4, #3040
	sub	r6, r4, #3184
	strh	r5, [r7, #-6]	@ movhi
	strh	r5, [r3, #-14]	@ movhi
	str	r5, [r4, #-3156]
	str	r5, [r4, #-3152]
	str	r5, [r4, #-3148]
	str	r5, [r4, #-3144]
	str	r5, [r4, #-3140]
	str	r5, [r4, #-3136]
	str	r5, [r4, #-3132]
	str	r5, [r4, #-3128]
	str	r5, [r4, #-3124]
	str	r5, [r4, #-3120]
	str	r5, [r4, #-3116]
	str	r5, [r4, #-3112]
	str	r5, [r4, #-3108]
	str	r5, [r4, #-3104]
	str	r5, [r4, #-3100]
	str	r5, [r4, #-3096]
	str	r5, [r4, #-3092]
	str	r5, [r4, #-3084]
	str	r5, [r4, #-3080]
	str	r5, [r4, #-3076]
	ldrh	r1, [r6, #-10]
	bl	__aeabi_idiv
	ldrh	r8, [r9, #-4]
	str	r5, [r4, #-3048]
	mov	r8, r8, asl #2
	cmp	r0, r8
	str	r0, [r4, #-3052]
	ldrh	r0, [r6, #-12]
	strhi	r8, [r4, #-3052]
	mov	r8, #12
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-3044]
	ldrh	r0, [r6, #-12]
	mul	r0, r8, r0
	bl	ftl_malloc
	ldrh	r5, [r9, #-4]
	mul	r5, r10, r5
	mov	fp, r5, asl #3
	str	r0, [r4, #-3040]
	mov	r0, fp
	bl	ftl_malloc
	str	r0, [r4, #-3036]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-3032]
	mov	r0, fp
	bl	ftl_malloc
	str	r0, [r4, #-3028]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-3024]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-3020]
	ldr	r0, [r4, #-3052]
	mul	r0, r10, r0
	bl	ftl_malloc
	ldrh	r5, [r6, #-6]
	ldrh	r3, [r9, #-4]
	mov	r3, r3, asl #1
	add	r3, r3, #1
	str	r3, [r4, #-3012]
	str	r0, [r4, #-3016]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-3008]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-3004]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-3000]
	ldr	r0, [r4, #-3012]
	mul	r0, r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-2996]
	ldr	r0, [r4, #-3052]
	mul	r0, r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-2992]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-2988]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-2984]
	ldr	r0, [r4, #-3012]
	mul	r0, r8, r0
	bl	ftl_malloc
	ldrh	r3, [r9, #-4]
	ldrh	r5, [r6, #-4]
	mul	r5, r3, r5
	str	r0, [r4, #-2980]
	mov	r0, r5
	bl	ftl_malloc
	str	r0, [r4, #-2976]
	mov	r0, r5, asl #3
	bl	ftl_malloc
	ldrh	r3, [r6, #-4]
	ldr	r5, .L1576+4
	add	r9, r5, #288
	str	r0, [r4, #-2972]
	ldr	r0, [r4, #-3012]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r6, #-4]
	str	r0, [r4, #-2968]
	ldr	r0, [r4, #-3052]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #-2964]
	ldrh	r0, [r5, #-10]
	mov	r0, r0, asl #1
	uxth	r0, r0
	strh	r0, [r9]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #-2956]
	ldrh	r0, [r9]
	add	r0, r0, #544
	add	r0, r0, #3
	mov	r0, r0, lsr #9
	strh	r0, [r9]	@ movhi
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldrh	r9, [r5, #-10]
	mov	r9, r9, asl #1
	str	r0, [r4, #-2952]
	add	r0, r0, #32
	str	r0, [r4, #-2948]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-2944]
	mov	r0, r9
	bl	ftl_malloc
	ldr	r9, [r4, #-3172]
	mov	r9, r9, asl #1
	str	r0, [r4, #-2940]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-2936]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-2932]
	ldrh	r0, [r5, #-10]
	mov	r0, r0, lsr #3
	add	r0, r0, #4
	bl	ftl_malloc
	str	r0, [r4, #-2928]
	ldrh	r0, [r5, #68]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-2924]
	ldrh	r0, [r5, #68]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-2920]
	ldrh	r0, [r5, #68]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-2916]
	ldrh	r0, [r5, #70]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r2, [r5, #70]
	mov	r1, #0
	mov	r2, r2, asl #2
	str	r0, [r4, #-2912]
	bl	ftl_memset
	ldrh	r9, [r7, #-12]
	mov	r9, r9, asl #2
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-2908]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-2904]
	ldr	r0, [r4, #-3172]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-2900]
	ldrh	r0, [r7, #-10]
	mul	r0, r8, r0
	bl	ftl_malloc
	ldrh	r3, [r7, #-10]
	add	r7, r5, #16
	str	r0, [r4, #-2896]
	ldrh	r0, [r6, #-6]
	add	r6, r5, #368
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, #-10]
	str	r0, [r4, #-2892]
	mov	r0, #6
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, #46]
	add	r3, r3, #31
	mov	r3, r3, asr #5
	strh	r3, [r6, #-4]	@ movhi
	str	r0, [r4, #-2888]
	ldrh	r0, [r5, #2]
	mul	r0, r0, r3
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r2, [r6, #-4]
	ldrh	ip, [r5, #2]
	add	r5, r5, #396
	mov	r3, #1
	mov	r2, r2, asl #2
	mov	r1, r2
	str	r0, [r4, #-2852]
.L1473:
	cmp	r3, ip
	bcs	.L1574
	ldr	r0, [r4, #-2852]
	add	r3, r3, #1
	add	r0, r0, r1
	add	r1, r1, r2
	str	r0, [r5, #4]!
	b	.L1473
.L1574:
	mov	r2, #0
.L1475:
	cmp	r3, #8
	addne	r1, r6, r3, asl #2
	addne	r3, r3, #1
	strne	r2, [r1, #28]
	bne	.L1475
.L1575:
	ldr	r2, [r4, #-2936]
	ldr	r3, .L1576
	cmp	r2, #0
	bne	.L1477
.L1479:
	ldr	r1, .L1576+8
	ldr	r0, .L1576+12
	bl	printk
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1477:
	ldr	r2, [r3, #-2932]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2908]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2900]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2896]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2892]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2888]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2852]
	cmp	r2, #0
	beq	.L1479
	ldr	r3, [r3, #-2940]
	cmp	r3, #0
	beq	.L1479
	ldr	r2, [r4, #-3044]
	ldr	r3, .L1576
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3040]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3036]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3028]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3024]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3020]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3032]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3008]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-3004]
	cmp	r2, #0
	beq	.L1479
	ldr	r3, [r3, #-3000]
	cmp	r3, #0
	beq	.L1479
	ldr	r2, [r4, #-2996]
	ldr	r3, .L1576
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2988]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2984]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2980]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2976]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2972]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2968]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2948]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2956]
	cmp	r2, #0
	beq	.L1479
	ldr	r3, [r3, #-2924]
	cmp	r3, #0
	beq	.L1479
	ldr	r3, .L1576
	ldr	r2, [r3, #-2920]
	cmp	r2, #0
	beq	.L1479
	ldr	r2, [r3, #-2916]
	cmp	r2, #0
	beq	.L1479
	ldr	r3, [r3, #-2912]
	cmp	r3, #0
	beq	.L1479
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1577:
	.align	2
.L1576:
	.word	.LANCHOR2
	.word	.LANCHOR2-3248
	.word	.LANCHOR3+4
	.word	.LC30
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	IsBlkInVendorPart
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L1585
	sub	r3, r2, #2816
	ldrh	r3, [r3, #-4]
	cmp	r3, #0
	beq	.L1584
	ldr	r3, [r2, #-2924]
	sub	r2, r2, #3168
	ldrh	r2, [r2, #-12]
	add	r2, r3, r2, asl #1
.L1580:
	cmp	r3, r2
	beq	.L1584
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L1580
	mov	r0, #1
	bx	lr
.L1584:
	mov	r0, #0
	bx	lr
.L1586:
	.align	2
.L1585:
	.word	.LANCHOR2
	.fnend
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheMetchLpa
	.type	FtlCacheMetchLpa, %function
FtlCacheMetchLpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1594
	ldr	r3, [r2, #-3048]
	cmp	r3, #0
	beq	.L1590
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	mov	r5, #36
	ldr	r4, [r2, #-3016]
	mov	r2, #0
.L1589:
	mla	ip, r5, r2, r4
	ldr	lr, [ip, #16]
	cmp	lr, r0
	movcc	ip, #0
	movcs	ip, #1
	cmp	lr, r1
	movhi	ip, #0
	cmp	ip, #0
	bne	.L1591
	add	r2, r2, #1
	cmp	r2, r3
	bne	.L1589
	mov	r0, ip
	ldmfd	sp!, {r4, r5, pc}
.L1590:
	mov	r0, r3
	bx	lr
.L1591:
	mov	r0, #1
	ldmfd	sp!, {r4, r5, pc}
.L1595:
	.align	2
.L1594:
	.word	.LANCHOR2
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
	ldr	r3, .L1597
	ldr	r0, [r3, #3908]
	bx	lr
.L1598:
	.align	2
.L1597:
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
	ldr	r3, .L1600
	ldr	r0, [r3, #3908]
	bx	lr
.L1601:
	.align	2
.L1600:
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
	ldr	r3, .L1603
	ldr	r0, [r3, #-2816]
	bx	lr
.L1604:
	.align	2
.L1603:
	.word	.LANCHOR2
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
	mov	r6, r0
	ldr	r5, .L1607
	sub	r3, r5, #3200
	ldrh	r4, [r3, #-2]
	mov	r1, r4
	bl	__aeabi_uidiv
	uxth	r2, r0
	smulbb	r3, r2, r4
	add	r1, r5, r2, asl #2
	mov	r4, #1
	sub	r5, r5, #2880
	ldr	ip, [r1, #-2852]
	rsb	r3, r3, r6
	uxth	r3, r3
	and	r1, r3, #31
	mov	lr, r3, lsr #5
	ldr	r0, [ip, lr, asl #2]
	orr	r1, r0, r4, asl r1
	ldr	r0, .L1607+4
	str	r1, [ip, lr, asl #2]
	str	r1, [sp]
	mov	r1, r6
	bl	printk
	ldrh	r3, [r5, #6]
	mov	r0, #0
	add	r3, r3, r4
	strh	r3, [r5, #6]	@ movhi
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L1608:
	.align	2
.L1607:
	.word	.LANCHOR2
	.word	.LC31
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
	mov	r7, r0
	ldr	r5, .L1611
	sub	r3, r5, #3200
	ldrh	r6, [r3, #-2]
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
	ldr	r3, [r5, #-2852]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr r4
	and	r0, r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1612:
	.align	2
.L1611:
	.word	.LANCHOR2
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
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, .L1620
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
.L1616:
	ldrh	r2, [r1, #2]!
	cmp	r2, r4
	ldmeqfd	sp!, {r3, r4, r5, pc}
	mov	ip, r2, lsr #5
	cmp	r1, r3
	and	r2, r2, #31
	ldr	r0, [r5, ip, asl #2]
	orr	r2, r0, lr, asl r2
	str	r2, [r5, ip, asl #2]
	bne	.L1616
	ldmfd	sp!, {r3, r4, r5, pc}
.L1621:
	.align	2
.L1620:
	.word	.LANCHOR2-2880
	.fnend
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
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
	ldr	r4, .L1640
	ldr	r5, [r4, #-2812]
	cmp	r5, #0
	bne	.L1624
	ldr	r3, [r4, #-2976]
	mov	r1, r5
	ldr	r0, [r4, #-3008]
	sub	r7, r4, #2880
	ldr	r8, .L1640+4
	sub	fp, r7, #4
	str	r3, [r4, #-2796]
	sub	r3, r4, #3184
	str	r0, [r4, #-2800]
	mov	r10, r4
	ldrh	r2, [r3, #-6]
	bl	ftl_memset
	ldr	r9, .L1640+8
.L1625:
	ldrh	r3, [r9]
	ldr	r6, .L1640
	cmp	r5, r3
	bge	.L1639
	ldrh	r2, [fp]
	ldr	r3, [r10, #-2800]
	ldr	r1, [r8, #4]!
	mul	r0, r2, r5
	mov	r2, r2, asl #2
	add	r5, r5, #1
	add	r0, r3, r0, asl #2
	bl	ftl_memcpy
	b	.L1625
.L1639:
	ldr	r5, [r6, #-2796]
	mov	r1, #255
	mov	r2, #16
	ldr	r10, .L1640+12
	mov	r8, #0
	mov	r0, r5
	mov	r9, r8
	bl	ftl_memset
	ldr	r3, .L1640+16
	strh	r3, [r5]	@ movhi
	ldr	r3, [r6, #-2872]
	str	r3, [r5, #4]
	ldrh	r3, [r7]
	strh	r3, [r5, #2]	@ movhi
	ldrh	r3, [r7, #4]
	strh	r3, [r5, #8]	@ movhi
	ldrh	r3, [r7, #6]
	strh	r3, [r5, #10]	@ movhi
	ldr	r3, [r6, #-3272]
	strh	r3, [r5, #12]	@ movhi
.L1627:
	ldr	r3, [r4, #-3008]
	mov	ip, #0
	ldrh	r1, [r7]
	ldrh	r2, [r7, #2]
	str	r3, [r4, #-2800]
	ldr	r3, [r4, #-2976]
	str	ip, [r4, #-2808]
	str	ip, [sp, #12]
	str	r3, [r4, #-2796]
	orr	r3, r2, r1, asl #10
	ldrh	r0, [r5, #10]
	str	r3, [r4, #-2804]
	ldrh	r3, [r7, #4]
	str	r0, [sp]
	ldr	r0, .L1640+20
	bl	printk
	ldrh	r3, [r10]
	ldrh	r2, [r7, #2]
	sub	r3, r3, #1
	ldr	fp, .L1640+24
	cmp	r2, r3
	ldr	ip, [sp, #12]
	blt	.L1628
	ldr	r3, [r6, #-2872]
	mov	r1, #1
	ldrh	r2, [fp]
	add	r3, r3, #1
	ldr	r0, [r6, #-3024]
	str	r3, [r6, #-2872]
	str	r3, [r5, #4]
	ldrh	r3, [fp, #4]
	strh	r2, [r5, #8]	@ movhi
	strh	r2, [fp, #4]	@ movhi
	mov	r2, r1
	strh	r3, [fp]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r6, #-2804]
	str	r3, [r0, #4]
	strh	ip, [fp, #2]	@ movhi
	bl	FlashEraseBlocks
.L1628:
	mov	r1, #1
	ldr	r0, .L1640+28
	mov	r3, r1
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r7, #2]
	ldr	fp, .L1640
	add	r3, r3, #1
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, [r4, #-2808]
	cmn	r3, #1
	bne	.L1629
	add	r8, r8, #1
	ldr	r0, .L1640+32
	ldr	r1, [r6, #-2804]
	uxth	r8, r8
	bl	printk
	cmp	r8, #3
	bls	.L1627
	ldr	r0, .L1640+36
	mov	r2, r8
	ldr	r1, [fp, #-2804]
	bl	printk
	mov	r3, #1
	str	r3, [fp, #-2812]
	b	.L1624
.L1629:
	add	r9, r9, #1
	cmp	r9, #1
	beq	.L1627
	cmp	r3, #256
	beq	.L1627
.L1624:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1641:
	.align	2
.L1640:
	.word	.LANCHOR2
	.word	.LANCHOR2-2856
	.word	.LANCHOR2-3246
	.word	.LANCHOR2-3198
	.word	-3887
	.word	.LC32
	.word	.LANCHOR2-2880
	.word	.LANCHOR2-2808
	.word	.LC33
	.word	.LC34
	.fnend
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1654
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r5, #0
	ldr	r6, .L1654+4
	mov	r9, r2
	ldr	r3, [r2, #-3008]
	ldr	r7, [r2, #-2976]
	sub	r8, r6, #376
	str	r3, [r2, #-2800]
	str	r7, [r2, #-2796]
.L1643:
	ldrh	r3, [r8]
	cmp	r5, r3
	bcs	.L1653
	ldr	r2, .L1654+8
	mvn	r3, #0
	strh	r3, [r6, #2]!	@ movhi
	ldrh	r3, [r2]
	mov	r10, r2
	sub	r3, r3, #1
	uxth	r4, r3
.L1644:
	ldrh	r3, [r10]
	sub	r2, r3, #16
	cmp	r4, r2
	ble	.L1646
	mla	r3, r3, r5, r4
	mov	r1, #1
	ldr	r0, .L1654+12
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [r9, #-2804]
	bl	FlashReadPages
	ldr	r3, [r9, #-2808]
	cmn	r3, #1
	beq	.L1645
	ldrh	r2, [r7]
	movw	r3, #61664
	cmp	r2, r3
	streqh	r4, [r6]	@ movhi
	beq	.L1646
.L1645:
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1644
.L1646:
	add	r5, r5, #1
	b	.L1643
.L1653:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L1655:
	.align	2
.L1654:
	.word	.LANCHOR2
	.word	.LANCHOR2-2870
	.word	.LANCHOR2-3202
	.word	.LANCHOR2-2808
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FtlBbtMemInit
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, .L1657
	mvn	r3, #0
	mov	r1, #255
	mov	r2, #16
	add	r0, r0, #12
	strh	r3, [r0, #-12]	@ movhi
	mov	r3, #0
	strh	r3, [r0, #-6]	@ movhi
	b	ftl_memset
.L1658:
	.align	2
.L1657:
	.word	.LANCHOR2-2880
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
	mov	r5, #0
	ldr	r4, .L1668
	ldrh	r6, [r4, #-2]
	ldrh	r3, [r4, #-46]
	mov	r4, r5
	mul	r6, r3, r6
.L1660:
	uxth	r0, r5
	cmp	r0, r6
	bge	.L1667
	bl	FtlBbmIsBadBlock
	add	r5, r5, #1
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
	b	.L1660
.L1667:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1669:
	.align	2
.L1668:
	.word	.LANCHOR2-3200
	.fnend
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	2
	.global	FtlMakeBbt
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r7, .L1695
	ldr	r6, [r7, #-2812]
	cmp	r6, #0
	bne	.L1671
	bl	FtlBbtMemInit
	ldr	r8, .L1695+4
	bl	FtlLoadFactoryBbt
	sub	r4, r7, #2880
	sub	r9, r8, #18
	mov	r5, r7
.L1672:
	ldr	r3, .L1695+8
	ldrh	r2, [r3]
	cmp	r6, r2
	bcs	.L1692
	ldrh	r2, [r9, #2]!
	movw	r1, #65535
	ldr	r0, [r5, #-3008]
	ldr	r10, [r5, #-2976]
	cmp	r2, r1
	str	r0, [r5, #-2800]
	str	r10, [r5, #-2796]
	beq	.L1673
	ldrh	fp, [r3, #44]
	mov	r1, #1
	ldr	r0, .L1695+12
	str	r3, [sp, #4]
	mla	fp, fp, r6, r2
	mov	r2, fp, asl #10
	str	r2, [r5, #-2804]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r0, [r8]
	ldr	r1, [r5, #-2800]
	ldr	r3, [sp, #4]
	ldrh	r2, [r3, #44]
	add	r2, r2, #7
	mov	r2, r2, asr #3
	bl	ftl_memcpy
	b	.L1674
.L1673:
	mov	r1, r6
	str	r3, [sp, #4]
	bl	FlashGetBadBlockList
	ldr	r0, [r5, #-2800]
	ldr	r1, [r8]
	bl	FtlBbt2Bitmap
	ldr	r3, [sp, #4]
	ldrh	r3, [r3, #44]
.L1691:
	sub	r3, r3, #1
	uxth	r3, r3
	str	r3, [sp, #4]
.L1675:
	ldr	fp, .L1695+16
	ldr	r3, [sp, #4]
	ldrh	r0, [fp]
	smlabb	r0, r0, r6, r3
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	ldreq	r3, [sp, #4]
	beq	.L1691
.L1693:
	ldrh	r3, [sp, #4]
	mov	r1, #0
	mov	r2, #16
	ldr	r0, [r5, #-2976]
	strh	r3, [r9]	@ movhi
	bl	ftl_memset
	ldr	r3, .L1695+20
	strh	r3, [r10]	@ movhi
	mov	r3, #0
	str	r3, [r10, #4]
	ldrh	r3, [r9]
	ldrh	fp, [fp]
	strh	r3, [r10, #2]	@ movhi
	ldrh	r3, [r9]
	ldr	r1, [r8]
	ldr	r0, [r5, #-2800]
	mla	fp, fp, r6, r3
	mov	r3, fp, asl #10
	str	r3, [r5, #-2804]
	ldr	r3, .L1695+24
	ldrh	r2, [r3]
	mov	r2, r2, asl #2
	bl	ftl_memcpy
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1695+12
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L1695+12
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r5, #-2808]
	cmn	r3, #1
	bne	.L1674
	uxth	r0, fp
	bl	FtlBbmMapBadBlock
	b	.L1675
.L1674:
	uxth	r0, fp
	add	r6, r6, #1
	bl	FtlBbmMapBadBlock
	add	r8, r8, #4
	b	.L1672
.L1692:
	add	r5, r3, #60
	mov	r6, #0
.L1679:
	ldrh	r3, [r5]
	uxth	r0, r6
	add	r6, r6, #1
	cmp	r3, r0
	bls	.L1694
	bl	FtlBbmMapBadBlock
	b	.L1679
.L1694:
	ldrh	r5, [r4, #12]
	movw	r8, #65535
	ldr	r6, .L1695+28
	sub	r5, r5, #1
	uxth	r5, r5
.L1681:
	ldrh	r3, [r4, #12]
	sub	r3, r3, #48
	cmp	r5, r3
	ble	.L1685
	mov	r0, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1682
	mov	r0, r5
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1683
	mov	r0, r5
	bl	FtlBbmMapBadBlock
	b	.L1682
.L1683:
	ldrh	r3, [r6]
	cmp	r3, r8
	streqh	r5, [r6]	@ movhi
.L1684:
	ldrne	r3, .L1695+28
	strneh	r5, [r3, #4]	@ movhi
	bne	.L1685
.L1682:
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L1681
.L1685:
	ldr	r0, [r7, #-3024]
	mov	r5, #0
	ldrh	r3, [r4]
	mov	r2, #2
	str	r5, [r7, #-2872]
	mov	r1, #1
	strh	r5, [r4, #2]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, #4]
	mov	r3, r3, asl #10
	str	r3, [r0, #40]
	bl	FlashEraseBlocks
	ldrh	r0, [r4]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, #4]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r7, #-2872]
	ldrh	r2, [r4, #4]
	add	r3, r3, #1
	str	r3, [r7, #-2872]
	ldrh	r3, [r4]
	strh	r5, [r4, #2]	@ movhi
	strh	r2, [r4]	@ movhi
	strh	r3, [r4, #4]	@ movhi
	bl	FtlBbmTblFlush
.L1671:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1696:
	.align	2
.L1695:
	.word	.LANCHOR2
	.word	.LANCHOR2-2852
	.word	.LANCHOR2-3246
	.word	.LANCHOR2-2808
	.word	.LANCHOR2-3202
	.word	-3872
	.word	.LANCHOR2-2884
	.word	.LANCHOR2-2880
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	V2P_block
	.type	V2P_block, %function
V2P_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, r1
	ldr	r4, .L1699
	mov	r7, r0
	sub	r3, r4, #3232
	sub	r4, r4, #3200
	ldrh	r6, [r3, #-12]
	mov	r1, r6
	bl	__aeabi_uidivmod
	mov	r0, r7
	smlabb	r5, r5, r6, r1
	mov	r1, r6
	bl	__aeabi_uidiv
	ldrh	r3, [r4, #-2]
	smlabb	r0, r3, r0, r5
	uxth	r0, r0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1700:
	.align	2
.L1699:
	.word	.LANCHOR2
	.fnend
	.size	V2P_block, .-V2P_block
	.align	2
	.global	P2V_plane
	.type	P2V_plane, %function
P2V_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1703
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	sub	r2, r3, #3232
	sub	r3, r3, #3200
	mov	r6, r0
	ldrh	r5, [r2, #-12]
	ldrh	r1, [r3, #-2]
	bl	__aeabi_uidiv
	mov	r1, r5
	smulbb	r4, r0, r5
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r4, r1
	uxth	r0, r1
	ldmfd	sp!, {r4, r5, r6, pc}
.L1704:
	.align	2
.L1703:
	.word	.LANCHOR2
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
	ldr	r4, .L1707
	sub	r3, r4, #3200
	sub	r4, r4, #3232
	ldrh	r1, [r3, #-2]
	bl	__aeabi_uidivmod
	uxth	r0, r1
	ldrh	r1, [r4, #-12]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L1708:
	.align	2
.L1707:
	.word	.LANCHOR2
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
	bls	.L1710
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L1710:
	rsb	r0, r0, r1
	cmp	r0, #-2147483648
	movls	r0, #0
	movhi	r0, #1
	bx	lr
	.fnend
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #1
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #104
	sub	sp, sp, #104
	ldreq	r3, .L1724
	mov	r8, r1
	ldrne	r3, .L1724+4
	mov	r6, r0, asl #10
	mov	r1, #1
	add	r0, sp, #4
	ldreqh	r4, [r3, #-14]
	mov	r2, r8
	ldrneh	r4, [r3]
	add	r3, sp, #40
	str	r3, [sp, #16]
	mov	r7, #0
	sub	r5, r4, #1
	str	r7, [sp, #12]
	uxth	r5, r5
	sxth	r3, r5
	orr	r3, r3, r6
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	cmn	r3, #1
	bne	.L1715
.L1716:
	sxth	r4, r7
	sxth	r3, r5
	cmp	r4, r3
	bgt	.L1715
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
	bne	.L1717
	ldr	r3, [sp, #44]
	cmn	r3, #1
	bne	.L1717
	ldr	r3, [sp, #4]
	cmn	r3, #1
	subne	r4, r4, #1
	uxthne	r5, r4
	bne	.L1716
.L1717:
	add	r3, r4, #1
	uxth	r7, r3
	b	.L1716
.L1715:
	sxth	r0, r5
	add	sp, sp, #104
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1725:
	.align	2
.L1724:
	.word	.LANCHOR2-3184
	.word	.LANCHOR2-3200
	.fnend
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlLoadBbt
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	ldr	r4, .L1759
	ldr	r3, [r4, #-3008]
	mov	r7, r4
	ldr	r6, [r4, #-2976]
	str	r3, [r4, #-2800]
	str	r6, [r4, #-2796]
	bl	FtlBbtMemInit
	sub	r3, r4, #3200
	sub	r8, r3, #2
	ldrh	r5, [r3, #-2]
	sub	r5, r5, #1
	uxth	r5, r5
.L1727:
	ldrh	r3, [r8]
	sub	r3, r3, #48
	cmp	r5, r3
	ble	.L1730
	mov	r1, #1
	ldr	r0, .L1759+4
	mov	r2, r1
	mov	r3, r5, asl #10
	str	r3, [r7, #-2804]
	bl	FlashReadPages
	ldr	r3, [r7, #-2808]
	cmn	r3, #1
	bne	.L1728
	ldr	r3, [r7, #-2804]
	mov	r1, #1
	ldr	r0, .L1759+4
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r7, #-2804]
	bl	FlashReadPages
.L1728:
	ldr	r3, [r4, #-2808]
	ldr	r2, .L1759
	cmn	r3, #1
	beq	.L1729
	ldrh	r1, [r6]
	movw	r3, #61649
	cmp	r1, r3
	bne	.L1729
	ldr	r1, [r6, #4]
	sub	r3, r2, #2880
	strh	r5, [r3]	@ movhi
	str	r1, [r2, #-2872]
	ldrh	r2, [r6, #8]
	strh	r2, [r3, #4]	@ movhi
	b	.L1730
.L1729:
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L1727
.L1730:
	ldr	r7, .L1759
	movw	r2, #65535
	sub	r5, r7, #2880
	ldrh	r3, [r5]
	cmp	r3, r2
	beq	.L1744
	ldrh	r3, [r5, #4]
	cmp	r3, r2
	beq	.L1734
	mov	r1, #1
	add	r0, r5, #72
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [r7, #-2804]
	bl	FlashReadPages
	ldr	r3, [r7, #-2808]
	cmn	r3, #1
	beq	.L1734
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1734
	ldr	r3, [r6, #4]
	ldr	r2, [r7, #-2872]
	cmp	r3, r2
	strhi	r3, [r7, #-2872]
	ldrhih	r2, [r5, #4]
	ldrhih	r3, [r6, #8]
	strhih	r2, [r5]	@ movhi
	strhih	r3, [r5, #4]	@ movhi
.L1734:
	ldrh	r0, [r5]
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	movw	r8, #61649
	uxth	r7, r0
	add	r0, r0, #1
	strh	r0, [r5, #2]	@ movhi
.L1736:
	sxth	r3, r7
	cmp	r3, #0
	blt	.L1741
	ldrh	r2, [r5]
	mov	r1, #1
	ldr	r0, .L1759+4
	orr	r3, r3, r2, asl #10
	str	r3, [r4, #-2804]
	ldr	r3, [r4, #-3008]
	mov	r2, r1
	str	r3, [r4, #-2800]
	bl	FlashReadPages
	ldr	r3, [r4, #-2808]
	cmn	r3, #1
	beq	.L1737
	ldrh	r3, [r6]
	cmp	r3, r8
	bne	.L1737
.L1741:
	ldrh	r2, [r6, #10]
	ldrh	r0, [r6, #12]
	ldr	r3, .L1759
	strh	r2, [r5, #6]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	bne	.L1738
	b	.L1739
.L1737:
	sub	r7, r7, #1
	uxth	r7, r7
	b	.L1736
.L1738:
	ldr	r2, [r3, #-3272]
	cmp	r0, r2
	beq	.L1739
	sub	r3, r3, #3248
	ldrh	r3, [r3, #-10]
	mov	r3, r3, lsr #2
	cmp	r0, r3
	cmpcc	r2, r3
	bcs	.L1739
	bl	FtlSysBlkNumInit
.L1739:
	ldr	r5, .L1759+8
	mov	r4, #0
	ldr	r7, .L1759
	ldr	r6, .L1759+12
	sub	r8, r5, #28
.L1742:
	ldrh	r3, [r6]
	cmp	r4, r3
	bcs	.L1758
	ldrh	r2, [r8]
	ldr	r1, [r7, #-2800]
	ldr	r0, [r5, #4]!
	mov	r2, r2, asl #2
	mla	r1, r4, r2, r1
	bl	ftl_memcpy
	add	r4, r4, #1
	b	.L1742
.L1758:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1744:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1760:
	.align	2
.L1759:
	.word	.LANCHOR2
	.word	.LANCHOR2-2808
	.word	.LANCHOR2-2856
	.word	.LANCHOR2-3246
	.fnend
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1763
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	sub	r2, r3, #4
	mov	r4, #0
	strh	r0, [r3, #-4]	@ movhi
	strh	r4, [r2, #4]	@ movhi
	add	r0, r3, #4
	mov	r1, r4
	mov	r2, #2048
	strh	r4, [r3, #-2]	@ movhi
	strh	r4, [r3, #2]	@ movhi
	bl	ftl_memset
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L1764:
	.align	2
.L1763:
	.word	.LANCHOR2-2768
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
	ldr	r3, .L1766
	ldrh	r0, [r3, #6]
	clz	r0, r0
	mov	r0, r0, lsr #5
	bx	lr
.L1767:
	.align	2
.L1766:
	.word	.LANCHOR2-2772
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
	ldr	r3, .L1769
	ldrh	r0, [r3, #6]
	sub	r0, r0, #1024
	clz	r0, r0
	mov	r0, r0, lsr #5
	bx	lr
.L1770:
	.align	2
.L1769:
	.word	.LANCHOR2-2772
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
	ldr	r4, .L1781
	ldr	r5, .L1781+4
	ldrh	r3, [r4, #6]
	cmp	r3, #1024
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r1, #0
	beq	.L1773
	ldr	r3, [r5, #-2812]
	cmp	r3, #0
	bne	.L1773
	bl	P2V_block_in_plane
	mov	r1, #1
	mov	r3, r7, asl #10
	mov	r2, r1
	mov	r6, r0
	ldr	r0, [r5, #-3024]
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r1, [r5, #-2948]
	mov	r3, r6, asl #1
	ldrh	r2, [r1, r3]
	add	r2, r2, #1
	strh	r2, [r1, r3]	@ movhi
	ldr	r3, [r5, #-3108]
	add	r3, r3, #1
	str	r3, [r5, #-3108]
.L1773:
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
.L1782:
	.align	2
.L1781:
	.word	.LANCHOR2-2772
	.word	.LANCHOR2
	.fnend
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
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
	ldr	r6, .L1833
	ldr	r5, [r6, #-2812]
	ldrb	r3, [r6, #-3340]	@ zero_extendqisi2
	cmp	r5, #0
	str	r3, [sp, #16]
	movne	r0, #0
	bne	.L1784
	ldr	r2, .L1833+4
	mov	r9, r6
	mov	fp, r1
	mov	r10, r5
	mov	r4, r5
	mov	r7, r6
	ldrb	r3, [r2, #3904]	@ zero_extendqisi2
	mov	r8, #36
	str	r0, [sp, #4]
	str	r0, [r9, #-3076]
	str	r3, [sp, #8]
	str	r2, [sp, #12]
.L1785:
	ldr	r3, .L1833+8
	uxth	r2, r10
	ldr	ip, .L1833
	ldrh	r1, [r3]
	cmp	r1, r2
	bls	.L1828
	mul	r1, r8, r2
	ldr	r0, [r7, #-3024]
	add	r2, r7, r2
	mov	r3, #0
	str	r3, [r0, r1]
	ldr	r1, [sp, #4]
	ldrb	r0, [r2, #-3240]	@ zero_extendqisi2
	bl	V2P_block
	cmp	fp, #0
	mov	r6, r0
	beq	.L1786
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1787
.L1786:
	mov	r0, r6
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
	bne	.L1787
	ldr	r3, .L1833+12
	mov	r6, r6, asl #10
	ldr	r1, [r9, #-3024]
	ldrh	r2, [r3]
	mla	r1, r8, r5, r1
	mul	r2, r2, r5
	add	r5, r5, #1
	uxth	r5, r5
	cmp	r2, #0
	str	r0, [r1, #8]
	add	r0, r2, #3
	str	r6, [r1, #4]
	movlt	r2, r0
	ldr	r0, [r9, #-2972]
	bic	r2, r2, #3
	add	r2, r0, r2
	str	r2, [r1, #12]
.L1787:
	add	r10, r10, #1
	b	.L1785
.L1828:
	cmp	r5, #0
	beq	.L1808
	ldr	r3, [sp, #8]
	mov	r2, r5
	ldr	r0, [ip, #-3024]
	mov	r7, #0
	adds	r6, r3, #0
	strb	r7, [ip, #-3340]
	str	ip, [sp, #8]
	mov	r8, #36
	movne	r6, #1
	mov	r1, r6
	bl	FlashEraseBlocks
	ldrb	r3, [sp, #16]	@ zero_extendqisi2
	ldr	ip, [sp, #8]
	strb	r3, [ip, #-3340]
.L1791:
	uxth	r3, r7
	cmp	r3, r5
	bcs	.L1829
	mul	r3, r8, r7
	ldr	r2, [r9, #-3024]
	add	r0, r2, r3
	ldr	r3, [r2, r3]
	cmn	r3, #1
	bne	.L1792
	ldr	r0, [r0, #4]
	add	r4, r4, #1
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
.L1792:
	add	r7, r7, #1
	b	.L1791
.L1829:
	cmp	fp, #0
	bne	.L1794
	mov	r3, #6
	uxth	r6, r6
	str	r3, [sp, #12]
	mov	r3, #1
	str	r3, [sp, #8]
	b	.L1795
.L1794:
	ldr	r3, .L1833+16
	ldrh	r3, [r3, #-14]
	str	r3, [sp, #8]
	ldr	r3, [sp, #12]
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, [sp, #8]
	moveq	r6, #1
	movne	r6, #1
	strne	r6, [sp, #12]
	moveq	r3, r3, lsr #2
	streq	r3, [sp, #12]
.L1795:
	ldr	r10, .L1833
	mov	r7, #0
.L1804:
	mov	r8, #0
	mov	r5, r8
.L1796:
	ldr	r3, .L1833+8
	ldr	r9, .L1833
	ldrh	r2, [r3]
	uxth	r3, r8
	cmp	r2, r3
	bls	.L1830
	mov	r2, #36
	ldr	r1, [r10, #-3024]
	mul	r2, r2, r3
	add	r3, r10, r3
	mov	r0, #0
	str	r0, [r1, r2]
	ldr	r1, [sp, #4]
	ldrb	r0, [r3, #-3240]	@ zero_extendqisi2
	bl	V2P_block
	cmp	fp, #0
	mov	r9, r0
	beq	.L1797
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1798
.L1797:
	mov	r0, r9
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L1798
	ldr	r2, [r10, #-3024]
	mov	r3, #36
	add	r9, r7, r9, asl #10
	mla	r2, r3, r5, r2
	ldr	r3, [r10, #-2988]
	str	r3, [r2, #8]
	ldr	r3, .L1833+12
	str	r9, [r2, #4]
	ldrh	r3, [r3]
	mul	r3, r3, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	r1, r3, #3
	cmp	r3, #0
	movlt	r3, r1
	ldr	r1, [r10, #-2984]
	bic	r3, r3, #3
	add	r3, r1, r3
	str	r3, [r2, #12]
.L1798:
	add	r8, r8, #1
	b	.L1796
.L1830:
	cmp	r5, #0
	beq	.L1808
	ldr	r2, .L1833
	mov	r3, #0
	mov	r1, r5
	mov	r8, #0
	strb	r3, [r2, #-3340]
	mov	r3, #1
	ldr	r0, [r2, #-3024]
	mov	r2, r6
	bl	FlashProgPages
	ldr	r3, .L1833
	mov	r1, #36
	ldrb	r2, [sp, #16]	@ zero_extendqisi2
	strb	r2, [r3, #-3340]
.L1801:
	uxth	r3, r8
	cmp	r3, r5
	bcs	.L1831
	mul	r3, r1, r8
	ldr	r2, .L1833
	ldr	r2, [r2, #-3024]
	add	r0, r2, r3
	ldr	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1802
	ldr	r0, [r0, #4]
	add	r4, r4, #1
	str	r1, [sp, #20]
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
	ldr	r1, [sp, #20]
.L1802:
	add	r8, r8, #1
	b	.L1801
.L1831:
	ldr	r3, [sp, #12]
	add	r7, r7, r3
	ldr	r3, [sp, #8]
	uxth	r7, r7
	cmp	r7, r3
	bcc	.L1804
	mov	r7, #0
	mov	r8, #36
.L1805:
	uxth	r3, r7
	cmp	r3, r5
	bcs	.L1832
	cmp	fp, #0
	beq	.L1806
	mul	r3, r8, r7
	ldr	r2, [r9, #-3024]
	add	r1, r2, r3
	ldr	r3, [r2, r3]
	cmp	r3, #0
	bne	.L1806
	ldr	r0, [r1, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L1806:
	add	r7, r7, #1
	b	.L1805
.L1832:
	adds	r3, fp, #0
	ldr	r2, [sp, #4]
	movne	r3, #1
	cmp	r2, #63
	orrls	r3, r3, #1
	cmp	r3, #0
	beq	.L1808
	ldr	r0, [r9, #-3024]
	mov	r1, r6
	mov	r2, r5
	bl	FlashEraseBlocks
.L1808:
	mov	r0, r4
.L1784:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1834:
	.align	2
.L1833:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2-3268
	.word	.LANCHOR2-3188
	.word	.LANCHOR2-3184
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1848
	ldrh	ip, [r3, #28]
	ldr	r3, .L1848+4
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
.L1837:
	uxth	lr, r0
	add	r0, r0, #1
	cmp	lr, ip
	bge	.L1847
	add	lr, r3, r1, asl #1
	add	r1, r1, #1
	ubfx	r1, r1, #0, #10
	ldrh	r4, [lr, #8]
	add	lr, r3, r2, asl #1
	strh	r4, [lr, #8]	@ movhi
	mov	r4, #1
	add	r2, r2, r4
	ubfx	r2, r2, #0, #10
	b	.L1837
.L1847:
	cmp	r4, #0
	strneh	r1, [r3, #2]	@ movhi
	strneh	r2, [r3, #4]	@ movhi
	ldmfd	sp!, {r4, pc}
.L1849:
	.align	2
.L1848:
	.word	.LANCHOR2-716
	.word	.LANCHOR2-2772
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	ldr	r7, .L1861
	ldr	r5, .L1861+4
	mov	r6, r7
.L1851:
	ldrh	r1, [r5, #6]
	cmp	r1, #0
	beq	.L1852
	ldrh	r3, [r5, #2]
	sub	r1, r1, #1
	ldr	r9, [r7, #-2812]
	strh	r1, [r5, #6]	@ movhi
	add	r2, r5, r3, asl #1
	cmp	r9, #0
	add	r3, r3, #1
	ubfx	r3, r3, #0, #10
	ldrh	r4, [r2, #8]
	strh	r3, [r5, #2]	@ movhi
	bne	.L1853
	mov	r0, r4
	bl	P2V_block_in_plane
	mov	r3, r4, asl #10
	mov	r8, r0
	ldr	r0, [r7, #-3024]
	str	r3, [r0, #4]
	ldr	r3, .L1861+8
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1854
	mov	r1, r9
	mov	r2, #1
	bl	FlashEraseBlocks
.L1854:
	mov	r1, #1
	ldr	r0, [r6, #-3024]
	mov	r2, r1
	bl	FlashEraseBlocks
	ldr	r1, [r6, #-2948]
	mov	r3, r8, asl #1
	ldrh	r2, [r1, r3]
	add	r2, r2, #1
	strh	r2, [r1, r3]	@ movhi
	ldr	r3, [r6, #-3108]
	add	r3, r3, #1
	str	r3, [r6, #-3108]
	b	.L1853
.L1852:
	ldr	r0, .L1861+12
	bl	printk
.L1855:
	b	.L1855
.L1853:
	sub	r3, r4, #1
	movw	r2, #65533
	uxth	r3, r3
	cmp	r3, r2
	bls	.L1856
	ldr	r3, .L1861+4
	mov	r1, r4
	ldr	r0, .L1861+16
	ldrh	r2, [r3, #6]
	bl	printk
	b	.L1851
.L1856:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L1862:
	.align	2
.L1861:
	.word	.LANCHOR2
	.word	.LANCHOR2-2772
	.word	.LANCHOR0
	.word	.LC35
	.word	.LC36
	.fnend
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	test_node_in_list
	.type	test_node_in_list, %function
test_node_in_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1869
	str	lr, [sp, #-4]!
	.save {lr}
	movw	lr, #65535
	ldr	ip, [r3, #-2888]
	ldr	r2, [r0]
	ldr	r3, .L1869+4
	rsb	r0, ip, r2
	mov	r0, r0, asr #1
	mul	r3, r3, r0
	mov	r0, #6
	uxth	r3, r3
.L1865:
	cmp	r1, r3
	beq	.L1866
	ldrh	r3, [r2]
	cmp	r3, lr
	beq	.L1867
	mla	r2, r0, r3, ip
	b	.L1865
.L1866:
	mov	r0, #1
	ldr	pc, [sp], #4
.L1867:
	mov	r0, #0
	ldr	pc, [sp], #4
.L1870:
	.align	2
.L1869:
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
	ldr	r2, .L1888
	sub	r5, r2, #3248
	ldrh	r3, [r5, #-12]
	cmp	r3, r0
	bls	.L1873
	mov	lr, #6
	ldr	r6, [r2, #-2888]
	mul	lr, lr, r0
	mvn	ip, #0
	add	r1, r6, lr
	strh	ip, [r1, #2]	@ movhi
	strh	ip, [r6, lr]	@ movhi
	ldr	r3, [r2, #-668]
	cmp	r3, #0
	beq	.L1887
	ldr	r8, [r2, #-2940]
	mov	r4, r0, asl #1
	mov	r10, r2
	ldrh	r2, [r1, #4]
	ldrh	r5, [r5, #-12]
	ldrh	r7, [r8, r4]
	cmp	r2, #0
	str	r5, [sp]
	mulne	ip, r2, r7
	ldr	r7, [r10, #-2888]
	ldr	r2, .L1888+4
	rsb	r9, r7, r3
	mov	r9, r9, asr #1
	mul	r2, r2, r9
	ldr	r9, [r10, #-2948]
	add	r4, r9, r4
	str	r4, [sp, #4]
	mov	r4, #0
	uxth	r2, r2
.L1882:
	add	r4, r4, #1
	ldr	r5, [sp]
	uxth	r4, r4
	cmp	r4, r5
	movls	r5, #0
	movhi	r5, #1
	cmp	r0, r2
	orreq	r5, r5, #1
	cmp	r5, #0
	bne	.L1873
	mov	r10, r2, asl #1
	ldrh	r5, [r3, #4]
	ldrh	fp, [r8, r10]
	cmp	r5, #0
	mvneq	r5, #0
	mulne	r5, r5, fp
	cmp	r5, ip
	bne	.L1878
	ldr	r5, [sp, #4]
	ldrh	r10, [r9, r10]
	ldrh	r5, [r5]
	cmp	r10, r5
	bcc	.L1880
	b	.L1879
.L1878:
	bhi	.L1879
.L1880:
	ldrh	r5, [r3]
	movw	r10, #65535
	cmp	r5, r10
	streqh	r2, [r1, #2]	@ movhi
	streqh	r0, [r3]	@ movhi
	ldreq	r3, .L1888
	streq	r1, [r3, #-664]
	beq	.L1873
.L1881:
	mov	r3, #6
	mov	r2, r5
	mla	r3, r3, r5, r7
	b	.L1882
.L1879:
	strh	r2, [r6, lr]	@ movhi
	ldrh	r2, [r3, #2]
	strh	r2, [r1, #2]	@ movhi
	ldr	r2, .L1888
	ldr	ip, [r2, #-668]
	cmp	r3, ip
	bne	.L1883
	strh	r0, [r3, #2]	@ movhi
.L1887:
	str	r1, [r2, #-668]
	b	.L1873
.L1883:
	ldrh	ip, [r3, #2]
	ldr	r1, [r2, #-2888]
	mov	r2, #6
	mul	r2, r2, ip
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L1873:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1889:
	.align	2
.L1888:
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
	ldr	r2, .L1892
	ldrh	r3, [r2]
	add	r3, r3, #1
	strh	r3, [r2]	@ movhi
	ldmfd	sp!, {r3, pc}
.L1893:
	.align	2
.L1892:
	.word	.LANCHOR2-660
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
	beq	.L1895
	ldr	r2, .L1903
	mov	lr, #6
	mul	r6, lr, r0
	mvn	r3, #0
	ldr	r7, [r2, #-2888]
	mov	r5, r2
	add	ip, r7, r6
	strh	r3, [ip, #2]	@ movhi
	strh	r3, [r7, r6]	@ movhi
	ldr	r3, [r2, #-656]
	cmp	r3, #0
	beq	.L1902
	ldr	r9, [r2, #-2948]
	mov	r2, r0, asl #1
	ldr	r8, [r5, #-2888]
	rsb	r1, r8, r3
	ldrh	r10, [r9, r2]
	ldr	r2, .L1903+4
	mov	r1, r1, asr #1
	mul	r1, r2, r1
	uxth	r2, r1
.L1899:
	mov	r1, r2, asl #1
	ldrh	r1, [r9, r1]
	cmp	r1, r10
	bcs	.L1897
	ldrh	r1, [r3]
	cmp	r1, r4
	streqh	r2, [ip, #2]	@ movhi
	streqh	r0, [r3]	@ movhi
	beq	.L1895
.L1898:
	mla	r3, lr, r1, r8
	mov	r2, r1
	b	.L1899
.L1897:
	ldrh	r1, [r3, #2]
	strh	r1, [ip, #2]	@ movhi
	strh	r2, [r7, r6]	@ movhi
	ldr	r1, [r5, #-656]
	ldr	r2, .L1903
	cmp	r3, r1
	bne	.L1900
	strh	r0, [r3, #2]	@ movhi
.L1902:
	str	ip, [r2, #-656]
	b	.L1895
.L1900:
	ldrh	ip, [r3, #2]
	ldr	r1, [r2, #-2888]
	mov	r2, #6
	mul	r2, r2, ip
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L1895:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L1904:
	.align	2
.L1903:
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
	ldr	r2, .L1907
	ldrh	r3, [r2]
	add	r3, r3, #1
	strh	r3, [r2]	@ movhi
	ldmfd	sp!, {r3, pc}
.L1908:
	.align	2
.L1907:
	.word	.LANCHOR2-652
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
	ldr	r4, .L1915
	movw	r5, #65535
	mul	r1, ip, r1
	ldr	r3, [r0]
	ldr	r2, [r4, #-2888]
	add	lr, r2, r1
	cmp	lr, r3
	ldrh	r3, [r2, r1]
	bne	.L1910
	cmp	r3, r5
	mlane	r3, ip, r3, r2
	moveq	r3, #0
	streq	r3, [r0]
	strne	r3, [r0]
	mvnne	r0, #0
	strneh	r0, [r3, #2]	@ movhi
	b	.L1912
.L1910:
	cmp	r3, r5
	ldrh	r0, [lr, #2]
	bne	.L1913
	cmp	r0, r3
	mulne	r0, ip, r0
	mvnne	r3, #0
	strneh	r3, [r2, r0]	@ movhi
	b	.L1912
.L1913:
	mla	r3, ip, r3, r2
	strh	r0, [r3, #2]	@ movhi
	ldrh	r5, [lr, #2]
	ldrh	r0, [r2, r1]
	ldr	r3, [r4, #-2888]
	mul	ip, ip, r5
	strh	r0, [r3, ip]	@ movhi
.L1912:
	mvn	r3, #0
	mov	r0, #0
	strh	r3, [r2, r1]	@ movhi
	strh	r3, [lr, #2]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L1916:
	.align	2
.L1915:
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
	beq	.L1923
	ldr	r2, .L1926
	movw	ip, #65535
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	lr, #6
	ldr	r4, [r2, #-2888]
.L1919:
	cmp	r1, #0
	bne	.L1920
.L1922:
	rsb	r4, r4, r3
	ldr	r3, .L1926+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L1920:
	ldrh	r2, [r3]
	cmp	r2, ip
	beq	.L1922
	sub	r1, r1, #1
	mla	r3, lr, r2, r4
	uxth	r1, r1
	b	.L1919
.L1923:
	movw	r0, #65535
	bx	lr
.L1927:
	.align	2
.L1926:
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
	ldr	r2, .L1935
	ldr	r3, [r2, #-668]
	cmp	r3, #0
	ldrne	r1, [r2, #-2888]
	movne	ip, #6
	movwne	r2, #65535
	beq	.L1933
.L1930:
	cmp	r0, #0
	beq	.L1931
	ldrh	r3, [r3]
	cmp	r3, r2
	subne	r0, r0, #1
	mlane	r3, ip, r3, r1
	uxthne	r0, r0
	bne	.L1930
.L1933:
	movw	r0, #65535
	bx	lr
.L1931:
	rsb	r3, r1, r3
	ldr	r0, .L1935+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	bx	lr
.L1936:
	.align	2
.L1935:
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
	ldr	r3, .L1946
	sub	r2, r3, #648
	ldrh	r2, [r2]
	cmp	r2, r0
	beq	.L1945
	sub	r2, r3, #600
	ldrh	r2, [r2]
	cmp	r2, r0
	beq	.L1945
	sub	r2, r3, #552
	ldrh	r2, [r2]
	cmp	r2, r0
	beq	.L1945
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	lr, #6
	mul	lr, lr, r0
	ldr	r1, [r3, #-2888]
	ldr	r2, [r3, #-668]
	add	ip, r1, lr
	cmp	ip, r2
	beq	.L1938
	ldr	r4, [r3, #-2940]
	mov	r3, r0, asl #1
	ldrh	r2, [r4, r3]
	ldrh	r3, [ip, #4]
	ldrh	ip, [ip, #2]
	cmp	r3, #0
	mulne	r2, r3, r2
	mvneq	r2, #0
	movw	r3, #65535
	cmp	ip, r3
	bne	.L1940
	ldrh	r3, [r1, lr]
	cmp	r3, ip
	beq	.L1938
.L1940:
	mov	r3, #6
	mul	ip, r3, ip
	ldr	r3, .L1946+4
	add	r1, r1, ip
	mov	lr, ip, asr #1
	mul	r3, r3, lr
	mov	r3, r3, asl #1
	ldrh	lr, [r4, r3]
	ldrh	r3, [r1, #4]
	cmp	r3, #0
	mulne	r3, r3, lr
	mvneq	r3, #0
	cmp	r2, r3
	bcs	.L1938
	mov	r4, r0
	ldr	r0, .L1946+8
	mov	r1, r4
	bl	List_remove_node
	ldr	r2, .L1946+12
	mov	r0, r4
	ldrh	r3, [r2]
	sub	r3, r3, #1
	strh	r3, [r2]	@ movhi
	bl	INSERT_DATA_LIST
.L1938:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L1945:
	mov	r0, #0
	bx	lr
.L1947:
	.align	2
.L1946:
	.word	.LANCHOR2
	.word	-1431655765
	.word	.LANCHOR2-668
	.word	.LANCHOR2-660
	.fnend
	.size	List_update_data_list, .-List_update_data_list
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
.L1949:
	ldrh	r1, [r4, #6]
	uxth	r3, r2
	cmp	r1, r3
	bls	.L1969
	ldr	r0, [r6, r3, asl #2]
	mov	r1, #0
	ubfx	r0, r0, #10, #16
.L1950:
	ldrh	ip, [r4, #10]
	uxth	r3, r1
	cmp	ip, r3
	bls	.L1970
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
	b	.L1950
.L1970:
	add	r2, r2, #1
	b	.L1949
.L1969:
	ldr	r3, .L1972
	mov	r8, #0
	mov	r1, r8
	mov	fp, r8
	ldrh	r2, [r3, #-14]
	ldrh	r3, [r4]
	mov	r3, r3, asl #1
	strh	r2, [r5, r3]	@ movhi
	ldrh	r9, [r5]
.L1954:
	ldrh	r3, [r4, #10]
	uxth	r6, r8
	cmp	r3, r6
	bls	.L1971
	mov	r2, r6, asl #1
	ldrh	r3, [r5, r2]
	cmp	r9, r3
	bls	.L1955
	ldrh	r0, [r7, r2]
	add	r10, r7, r2
	cmp	r0, #0
	bne	.L1956
	b	.L1957
.L1955:
	cmp	r3, #0
	bne	.L1957
	ldrh	r0, [r7, r2]
	add	r10, r7, r2
	cmp	r0, #0
	movne	r6, r1
	beq	.L1957
	b	.L1959
.L1956:
	cmp	r3, #0
	movne	r1, r6
	movne	r9, r3
	bne	.L1957
	mov	r9, r3
.L1959:
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	strh	fp, [r10]	@ movhi
	ldrh	r3, [r4, #8]
	mov	r1, r6
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L1957:
	add	r8, r8, #1
	b	.L1954
.L1971:
	mov	r0, r1
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1973:
	.align	2
.L1972:
	.word	.LANCHOR2-3184
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
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
.L1975:
	uxth	r5, r3
	cmp	r5, r1
	bcs	.L1978
	mov	r7, r2
	add	r3, r3, #1
	ldrh	r6, [r7]
	add	r2, r2, #2
	cmp	r6, #0
	bne	.L1975
	mov	r4, r0
	bl	FtlFreeSysBlkQueueOut
	movw	r2, #65533
	sub	r3, r0, #1
	mov	r1, r0
	strh	r0, [r7]	@ movhi
	uxth	r3, r3
	cmp	r3, r2
	bls	.L1976
	ldr	r3, .L1982
	ldr	r0, .L1982+4
	ldrh	r2, [r3, #6]
	bl	printk
.L1977:
	b	.L1977
.L1976:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L1978:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1983:
	.align	2
.L1982:
	.word	.LANCHOR2-2772
	.word	.LC37
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
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
	ldr	fp, .L2003
	mov	r8, r1
	ldr	r9, .L2003+4
	mov	r5, #0
	str	r2, [sp]
	mov	r10, fp
.L1985:
	ldr	r3, [fp, #-3124]
	ldr	r6, .L2003
	add	r3, r3, #1
	str	r3, [fp, #-3124]
	ldrh	r3, [r9]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L1986
	ldrh	r2, [r4]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1987
.L1986:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L1987:
	ldr	r1, [r10, #-2812]
	cmp	r1, #0
	bne	.L1988
	ldrh	r3, [r4]
	ldr	r2, [r4, #12]
	ldr	r0, [fp, #-2976]
	mov	r3, r3, asl #1
	ldrh	r7, [r2, r3]
	mov	r2, #16
	ldrh	r3, [r4, #2]
	str	r0, [fp, #-2796]
	orr	r3, r3, r7, asl #10
	str	r3, [fp, #-2804]
	ldr	r3, [sp]
	str	r3, [fp, #-2800]
	bl	ftl_memset
	ldr	r2, [r4, #28]
	ldr	r3, [fp, #-2796]
	str	r2, [r3, #4]
	strh	r8, [r3, #8]	@ movhi
	ldrh	r2, [r4, #4]
	strh	r7, [r3, #2]	@ movhi
	strh	r2, [r3]	@ movhi
	ldr	r2, .L2003+8
	ldrb	r2, [r2]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1989
	ldr	r2, .L2003+12
	ldr	r0, [fp, #-2800]
	str	r3, [sp, #4]
	ldrh	r1, [r2]
	bl	js_hash
	ldr	r3, [sp, #4]
	str	r0, [r3, #12]
.L1989:
	mov	r1, #1
	ldr	r0, .L2003+16
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #2]	@ movhi
	ldr	r2, [r10, #-2808]
	cmn	r2, #1
	bne	.L1990
	ldr	r0, .L2003+20
	add	r5, r5, #1
	ldr	r1, [fp, #-2804]
	bl	printk
	ldrh	r3, [r4, #2]
	uxth	r5, r5
	cmp	r3, #2
	ldrlsh	r3, [r9]
	subls	r3, r3, #1
	strlsh	r3, [r4, #2]	@ movhi
	cmp	r5, #3
	bls	.L1985
	ldr	r0, .L2003+24
	mov	r2, r5
	ldr	r1, [r6, #-2804]
	bl	printk
	mov	r3, #1
	str	r3, [r6, #-2812]
	b	.L1988
.L1990:
	cmp	r2, #0
	strneh	r7, [r4, #40]	@ movhi
	cmp	r2, #256
	cmpne	r3, #1
	beq	.L1994
	ldr	r3, [r4, #36]
	cmp	r3, #0
	beq	.L1995
.L1994:
	mov	r3, #0
	str	r3, [r4, #36]
	b	.L1985
.L1995:
	ldr	r2, [r6, #-2804]
	ldr	r3, [r4, #24]
	str	r2, [r3, r8, asl #2]
.L1988:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2004:
	.align	2
.L2003:
	.word	.LANCHOR2
	.word	.LANCHOR2-3198
	.word	.LANCHOR0
	.word	.LANCHOR2-3190
	.word	.LANCHOR2-2808
	.word	.LC38
	.word	.LC39
	.fnend
	.size	FtlMapWritePage, .-FtlMapWritePage
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
	ldr	r8, .L2019
	cmp	r2, r3
	blt	.L2006
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r10, [r5, r0]
	cmp	r10, #0
	beq	.L2006
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L2006
	mov	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	ldrh	r3, [r8, #-14]
	cmp	r2, r3
	bcc	.L2007
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L2007:
	ldr	r5, .L2019+4
	mov	fp, #0
.L2008:
	ldrh	r3, [r4, #6]
	uxth	r6, fp
	cmp	r3, r6
	bls	.L2018
	ldr	r3, [r7, r6, asl #2]
	add	ip, r7, r6, asl #2
	cmp	r10, r3, lsr #10
	bne	.L2009
	ldr	r3, [r5, #-3004]
	mov	r1, #1
	ldr	r9, [r5, #-2976]
	mov	r2, r1
	ldr	r0, .L2019+8
	str	r3, [r5, #-2800]
	str	r9, [r5, #-2796]
	ldr	r3, [r7, r6, asl #2]
	str	ip, [sp, #4]
	str	r3, [r5, #-2804]
	bl	FlashReadPages
	ldr	r3, [r5, #-2808]
	cmn	r3, #1
	ldr	r3, .L2019+4
	ldr	ip, [sp, #4]
	bne	.L2010
.L2012:
	mov	r2, #0
	ldr	r0, .L2019+12
	str	r2, [ip]
	ldr	r1, [r3, #-2804]
	ldrh	r2, [r9, #8]
	str	r3, [sp, #4]
	bl	printk
	mov	r2, #1
	ldr	r3, [sp, #4]
	str	r2, [r3, #-2812]
	b	.L2011
.L2010:
	ldrh	r1, [r9, #8]
	cmp	r1, r6
	bne	.L2012
	ldrh	r0, [r9]
	ldrh	r2, [r4, #4]
	cmp	r0, r2
	bne	.L2012
	mov	r0, r4
	ldr	r2, [r5, #-2800]
	bl	FtlMapWritePage
.L2009:
	add	fp, fp, #1
	b	.L2008
.L2018:
	mov	r0, r10
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L2006:
	ldrh	r2, [r4, #2]
	ldrh	r3, [r8, #-14]
	cmp	r2, r3
	bcc	.L2011
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L2011:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2020:
	.align	2
.L2019:
	.word	.LANCHOR2-3184
	.word	.LANCHOR2
	.word	.LANCHOR2-2808
	.word	.LC40
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
	ldr	r5, .L2033
	ldr	r7, [r0, #12]
	ldr	r8, [r0, #24]
	ldr	r6, [r5, #-2812]
	cmp	r6, #0
	bne	.L2022
	ldrh	r3, [r0]
	movw	r2, #65535
	mov	r4, r0
	cmp	r3, r2
	bne	.L2023
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
	b	.L2022
.L2023:
	mov	r3, r3, asl #1
	sub	r10, r5, #3184
	mov	r1, #255
	ldrh	r9, [r7, r3]
	ldrh	r3, [r0, #2]
	ldr	r7, [r5, #-2976]
	orr	r3, r3, r9, asl #10
	str	r3, [r5, #-2804]
	ldr	r3, [r5, #-3008]
	str	r7, [r5, #-2796]
	str	r3, [r5, #-2800]
	ldr	r3, [r0, #28]
	str	r3, [r7, #4]
	ldr	r3, .L2033+4
	strh	r3, [r7, #8]	@ movhi
	ldrh	r3, [r0, #4]
	strh	r9, [r7, #2]	@ movhi
	strh	r3, [r7]	@ movhi
	ldrh	r2, [r10, #-14]
	ldr	r0, [r5, #-3008]
	mov	r2, r2, asl #3
	bl	ftl_memset
	mov	r2, r6
.L2024:
	ldrh	r1, [r4, #6]
	uxth	r3, r2
	cmp	r1, r3
	bls	.L2032
	ldr	r1, [r8, r3, asl #2]
	cmp	r9, r1, lsr #10
	bne	.L2025
	add	r6, r6, #1
	ldr	r1, [r5, #-3008]
	uxth	r6, r6
	str	r3, [r1, r6, asl #3]
	ldr	r1, [r8, r3, asl #2]
	ldr	r3, [r5, #-3008]
	add	r3, r3, r6, asl #3
	str	r1, [r3, #4]
.L2025:
	add	r2, r2, #1
	b	.L2024
.L2032:
	ldr	r3, .L2033+8
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2027
	ldr	r0, [r5, #-2800]
	ldrh	r1, [r10, #-6]
	bl	js_hash
	str	r0, [r7, #12]
.L2027:
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L2033+12
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L2022:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2034:
	.align	2
.L2033:
	.word	.LANCHOR2
	.word	-1291
	.word	.LANCHOR0
	.word	.LANCHOR2-2808
	.fnend
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
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
	ldr	r5, .L2037
	mul	r4, r4, r0
	sub	r0, r5, #504
	ldr	r3, [r5, #-2896]
	add	r2, r3, r4
	ldrh	r1, [r3, r4]
	ldr	r2, [r2, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #-2896]
	mov	r0, #0
	add	r4, r3, r4
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp!, {r3, r4, r5, pc}
.L2038:
	.align	2
.L2037:
	.word	.LANCHOR2
	.fnend
	.size	flush_l2p_region, .-flush_l2p_region
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2055
	mov	r1, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	sub	r2, r3, #3152
	ldr	r3, [r3, #-2896]
	mov	ip, #12
	ldrh	r2, [r2, #-10]
	movw	lr, #65535
.L2040:
	uxth	r0, r1
	cmp	r0, r2
	bcs	.L2052
	add	r1, r1, #1
	mla	r4, ip, r1, r3
	ldrh	r4, [r4, #-12]
	cmp	r4, lr
	bne	.L2040
	ldmfd	sp!, {r4, r5, r6, pc}
.L2052:
	mov	r0, r2
	mov	r1, #0
	mov	ip, #-2147483648
	mov	r5, #12
.L2043:
	uxth	r4, r1
	cmp	r4, r2
	bcs	.L2053
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
	b	.L2043
.L2053:
	cmp	r0, r2
	ldmccfd	sp!, {r4, r5, r6, pc}
	ldr	r1, .L2055+4
	mov	r0, r2
	mvn	ip, #0
	ldrh	r5, [r1]
	mov	r1, #0
.L2046:
	uxth	lr, r1
	cmp	lr, r2
	bcs	.L2054
	ldr	r4, [r3, #4]
	cmp	r4, ip
	bcs	.L2047
	ldrh	r6, [r3]
	cmp	r6, r5
	movne	ip, r4
	movne	r0, lr
.L2047:
	add	r1, r1, #1
	add	r3, r3, #12
	b	.L2046
.L2054:
	ldmfd	sp!, {r4, r5, r6, pc}
.L2056:
	.align	2
.L2055:
	.word	.LANCHOR2
	.word	.LANCHOR2-460
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r6, .L2074
	sub	r3, r6, #3184
	str	r3, [sp, #8]
	ldr	ip, [r6, #-2816]
	ldrh	r10, [r3, #-8]
	cmp	r0, ip
	bcs	.L2058
	add	r10, r10, #7
	mov	fp, r6
	mov	r6, r0, lsr r10
	sub	r3, fp, #3152
	str	r2, [sp, #12]
	mov	r9, r1
	ldrh	r2, [r3, #-10]
	uxth	r6, r6
	str	r0, [sp, #4]
	mov	r3, #0
	ldr	r4, [fp, #-2896]
	mov	r1, #12
	b	.L2059
.L2058:
	cmp	r2, #0
	mvn	r0, #0
	streq	r0, [r1]
	b	.L2060
.L2064:
	add	r3, r3, #1
	mla	r0, r1, r3, r4
	ldrh	r0, [r0, #-12]
	cmp	r0, r6
	beq	.L2061
.L2059:
	uxth	r7, r3
	cmp	r7, r2
	bcc	.L2064
	bl	select_l2p_ram_region
	mov	r5, #12
	movw	r2, #65535
	mul	r5, r5, r0
	mov	r7, r0
	add	r3, r4, r5
	ldrh	r1, [r4, r5]
	cmp	r1, r2
	bne	.L2073
.L2065:
	ldr	r3, [fp, #-2908]
	ldr	r4, .L2074
	ldr	r8, [r3, r6, asl #2]
	cmp	r8, #0
	bne	.L2066
	ldr	r2, [r4, #-2896]
	mov	r1, #255
	ldr	r3, [sp, #8]
	add	r2, r2, r5
	ldr	r0, [r2, #8]
	ldrh	r2, [r3, #-6]
	bl	ftl_memset
	ldr	r2, [r4, #-2896]
	strh	r6, [r2, r5]	@ movhi
	ldr	r2, [r4, #-2896]
	add	r5, r2, r5
	str	r8, [r5, #4]
.L2061:
	ldr	r3, [sp, #4]
	mvn	r2, #0
	bic	r10, r3, r2, asl r10
	ldr	r3, [sp, #12]
	cmp	r3, #0
	uxth	r10, r10
	mov	r3, #12
	bne	.L2062
	ldr	r2, [fp, #-2896]
	mla	r3, r3, r7, r2
	ldr	r3, [r3, #8]
	ldr	r3, [r3, r10, asl #2]
	str	r3, [r9]
	b	.L2063
.L2062:
	mul	r3, r3, r7
	ldr	r2, [fp, #-2896]
	ldr	r1, [r9]
	add	r2, r2, r3
	ldr	r2, [r2, #8]
	str	r1, [r2, r10, asl #2]
	ldr	r2, [fp, #-2896]
	add	r3, r2, r3
	ldr	r2, [r3, #4]
	orr	r2, r2, #-2147483648
	str	r2, [r3, #4]
	ldr	r3, .L2074+4
	strh	r6, [r3]	@ movhi
.L2063:
	ldr	r2, [fp, #-2896]
	mov	r3, #12
	mov	r0, #0
	mla	r7, r3, r7, r2
	ldr	r3, [r7, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r7, #4]
	b	.L2060
.L2073:
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L2065
	bl	flush_l2p_region
	b	.L2065
.L2066:
	ldr	r2, [r4, #-2896]
	mov	r1, #1
	ldr	r0, .L2074+8
	add	r2, r2, r5
	str	r8, [r4, #-2804]
	ldr	r2, [r2, #8]
	str	r2, [r4, #-2800]
	ldr	r2, [r4, #-2976]
	str	r2, [r4, #-2796]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [r4, #-2796]
	ldrh	r2, [r2, #8]
	cmp	r2, r6
	beq	.L2067
	mov	r1, r6
	mov	r2, r8
	ldr	r0, .L2074+12
	bl	printk
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2074+16
	ldr	r1, [r4, #-2796]
	bl	rknand_print_hex
	sub	r3, r4, #3152
	ldr	r0, .L2074+20
	mov	r2, #4
	ldrh	r3, [r3, #-12]
	ldr	r1, [r4, #-2908]
	bl	rknand_print_hex
	mov	r3, #1
	str	r3, [r4, #-2812]
	b	.L2068
.L2067:
	ldr	r2, [r4, #-2808]
	cmp	r2, #256
	bne	.L2068
	mov	r1, r6
	mov	r2, r8
	ldr	r0, .L2074+24
	bl	printk
	ldr	r3, [r4, #-2896]
	sub	r0, r4, #504
	mov	r1, r6
	add	r3, r3, r5
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
.L2068:
	ldr	r3, [fp, #-2896]
	mov	r1, #0
	add	r2, r3, r5
	str	r1, [r2, #4]
	strh	r6, [r3, r5]	@ movhi
	b	.L2061
.L2060:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2075:
	.align	2
.L2074:
	.word	.LANCHOR2
	.word	.LANCHOR2-460
	.word	.LANCHOR2-2808
	.word	.LC41
	.word	.LC17
	.word	.LC42
	.word	.LC43
	.fnend
	.size	log2phys, .-log2phys
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r2
	ldr	r6, .L2087
	add	r2, r0, r1
	mov	r7, r0
	.pad #60
	sub	sp, sp, #60
	mov	r4, r1
	ldrh	r3, [r6, #-20]
	ldrh	r5, [r6, #-8]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L2077
	mov	r5, r7, lsr r5
	mov	r3, #0
	str	r3, [sp]
	add	r3, r6, #3184
	mov	r10, r5, asl #2
	str	r3, [sp, #4]
	mov	r8, r3
.L2078:
	cmp	r4, #0
	beq	.L2086
	ldr	r3, [sp, #4]
	mov	r0, r7
	ldr	r3, [r3, #-2912]
	ldr	ip, [r3, r10]
	ldr	r3, .L2087+4
	str	ip, [sp, #12]
	ldrh	r2, [r3]
	mov	r1, r2
	str	r2, [sp, #8]
	bl	__aeabi_uidivmod
	ldr	r2, [sp, #8]
	mov	r9, r1
	ldr	ip, [sp, #12]
	rsb	r3, r1, r2
	uxth	r6, r3
	cmp	r6, r4
	uxthhi	r6, r4
	cmp	r6, r2
	cmpne	ip, #0
	movne	r1, #1
	moveq	r1, #0
	beq	.L2080
	ldr	r2, [r8, #-3000]
	mov	r1, #1
	add	r0, sp, #20
	str	ip, [sp, #24]
	str	r2, [sp, #28]
	mov	r2, #0
	str	r2, [sp, #32]
	mov	r2, r1
	bl	FlashReadPages
	b	.L2081
.L2080:
	ldr	r2, .L2087+8
	ldr	r0, [r8, #-3000]
	ldrh	r2, [r2]
	bl	ftl_memset
.L2081:
	mov	ip, r6, asl #9
	ldr	r0, [r8, #-3000]
	uxth	r9, r9
	mov	r1, fp
	mov	r2, ip
	str	ip, [sp, #8]
	add	r0, r0, r9, asl #9
	rsb	r4, r6, r4
	bl	ftl_memcpy
	mov	r1, r5
	ldr	r0, .L2087+12
	add	r5, r5, #1
	ldr	r2, [r8, #-3000]
	add	r7, r7, r6
	bl	FtlMapWritePage
	add	r10, r10, #4
	ldr	r3, [sp]
	cmn	r0, #1
	ldr	ip, [sp, #8]
	mvneq	r3, #0
	add	fp, fp, ip
	str	r3, [sp]
	b	.L2078
.L2086:
	ldr	r0, [sp]
.L2077:
	add	sp, sp, #60
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2088:
	.align	2
.L2087:
	.word	.LANCHOR2-3184
	.word	.LANCHOR2-3194
	.word	.LANCHOR2-3190
	.word	.LANCHOR2-456
	.fnend
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r2
	ldr	r6, .L2101
	add	r2, r0, r1
	mov	r8, r0
	.pad #60
	sub	sp, sp, #60
	mov	r7, r1
	ldrh	r3, [r6, #-20]
	ldrh	r5, [r6, #-8]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L2090
	add	r6, r6, #3184
	mov	r5, r8, lsr r5
	mov	r10, r6
	mov	r3, r5, asl #2
	str	r3, [sp, #4]
	mov	r3, #0
	str	r3, [sp]
.L2091:
	cmp	r7, #0
	beq	.L2100
	ldr	r2, [sp, #4]
	mov	r0, r8
	ldr	r3, [r6, #-2912]
	ldr	r3, [r3, r2]
	str	r3, [sp, #12]
	ldr	r3, .L2101+4
	ldrh	r4, [r3]
	mov	r1, r4
	bl	__aeabi_uidivmod
	rsb	r4, r1, r4
	ldr	r3, [sp, #12]
	str	r1, [sp, #8]
	uxth	r4, r4
	cmp	r4, r7
	uxthhi	r4, r7
	cmp	r3, #0
	mov	fp, r4, asl #9
	beq	.L2093
	ldr	r2, [r10, #-3000]
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
	ldr	r2, [r10, #-2808]
	mvneq	r3, #0
	cmp	r2, #256
	str	r3, [sp]
	ldr	r3, [sp, #12]
	bne	.L2095
	mov	r1, r5
	mov	r2, r3
	ldr	r0, .L2101+8
	bl	printk
	ldr	r0, .L2101+12
	mov	r1, r5
	ldr	r2, [r6, #-3000]
	bl	FtlMapWritePage
.L2095:
	ldrh	r3, [sp, #8]
	mov	r0, r9
	ldr	r1, [r10, #-3000]
	mov	r2, fp
	add	r1, r1, r3, asl #9
	bl	ftl_memcpy
	b	.L2096
.L2093:
	mov	r0, r9
	mov	r1, r3
	mov	r2, fp
	bl	ftl_memset
.L2096:
	ldr	r3, [sp, #4]
	add	r5, r5, #1
	rsb	r7, r4, r7
	add	r8, r8, r4
	add	r3, r3, #4
	add	r9, r9, fp
	str	r3, [sp, #4]
	b	.L2091
.L2100:
	ldr	r0, [sp]
.L2090:
	add	sp, sp, #60
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2102:
	.align	2
.L2101:
	.word	.LANCHOR2-3184
	.word	.LANCHOR2-3194
	.word	.LC44
	.word	.LANCHOR2-456
	.fnend
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2111
	sub	r1, r3, #412
	ldrh	r2, [r1]
	cmp	r2, #4
	cmpls	r0, #0
	addeq	r2, r2, #1
	streqh	r2, [r1]	@ movhi
	bxeq	lr
	str	lr, [sp, #-4]!
	.save {lr}
	mov	r2, #0
	strh	r2, [r1]	@ movhi
	sub	r1, r3, #3248
	str	r2, [r3, #-408]
	movw	ip, #65535
	ldrh	r0, [r1, #-12]
	ldr	r2, [r3, #-2940]
	add	r0, r2, r0, asl #1
.L2105:
	cmp	r2, r0
	beq	.L2110
	ldrh	r1, [r2], #2
	cmp	r1, ip
	ldrne	lr, [r3, #-408]
	addne	r1, r1, lr
	strne	r1, [r3, #-408]
	b	.L2105
.L2110:
	ldr	pc, [sp], #4
.L2112:
	.align	2
.L2111:
	.word	.LANCHOR2
	.fnend
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
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
	ldr	r4, .L2120
	mov	r2, #0
	str	r2, [r0, #36]
	ldr	r2, [r4, #-2812]
	cmp	r2, #0
	ldmnefd	sp!, {r4, r5, r6, pc}
	sub	r5, r5, #1
	mov	r6, r0
	ldr	r2, [r4, #-2976]
	ldr	r0, [r4, #-3004]
	uxth	r5, r5
	str	r2, [r4, #-2796]
	str	r0, [r4, #-2800]
	ldr	r3, [r3, r5, asl #2]
	cmp	r3, #0
	str	r3, [r4, #-2804]
	beq	.L2117
	mov	r1, #1
	ldr	r0, .L2120+4
	mov	r2, r1
	bl	FlashReadPages
	b	.L2118
.L2117:
	sub	r3, r4, #3184
	mov	r1, #255
	ldrh	r2, [r3, #-6]
	bl	ftl_memset
.L2118:
	mov	r0, r6
	mov	r1, r5
	ldr	r2, [r4, #-2800]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L2121:
	.align	2
.L2120:
	.word	.LANCHOR2
	.word	.LANCHOR2-2808
	.fnend
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r4, .L2140
	ldr	r3, [r4, #-2812]
	cmp	r3, #0
	bne	.L2124
	ldr	r2, [r4, #-3008]
	sub	r7, r4, #404
	ldr	r6, [r4, #-2976]
	sub	r5, r4, #716
	mov	r1, #255
	str	r2, [r4, #-2800]
	ldrh	r2, [r7]
	str	r6, [r4, #-2796]
	str	r3, [r6, #12]
	strh	r2, [r6, #2]	@ movhi
	ldr	r2, .L2140+4
	strh	r2, [r6]	@ movhi
	ldr	r2, [r4, #-396]
	stmib	r6, {r2, r3}
	ldr	r3, .L2140+8
	str	r3, [r4, #-716]
	ldr	r3, .L2140+12
	str	r3, [r4, #-712]
	ldrh	r3, [r7, #6]
	strh	r3, [r5, #8]	@ movhi
	sub	r3, r4, #3232
	ldrh	r3, [r3, #-14]
	strb	r3, [r4, #-706]
	sub	r3, r4, #648
	ldrh	r2, [r3]
	strh	r2, [r5, #14]	@ movhi
	ldrh	r2, [r3, #2]
	ldrb	r3, [r4, #-642]	@ zero_extendqisi2
	orr	r3, r3, r2, asl #6
	strh	r3, [r5, #16]	@ movhi
	ldrb	r3, [r4, #-640]	@ zero_extendqisi2
	strb	r3, [r4, #-705]
	sub	r3, r4, #600
	ldrh	r2, [r3]
	strh	r2, [r5, #18]	@ movhi
	ldrh	r2, [r3, #2]
	ldrb	r3, [r4, #-594]	@ zero_extendqisi2
	orr	r3, r3, r2, asl #6
	strh	r3, [r5, #20]	@ movhi
	ldrb	r3, [r4, #-592]	@ zero_extendqisi2
	strb	r3, [r4, #-704]
	sub	r3, r4, #552
	ldrh	r2, [r3]
	strh	r2, [r5, #22]	@ movhi
	ldrh	r2, [r3, #2]
	ldrb	r3, [r4, #-546]	@ zero_extendqisi2
	ldr	r0, [r4, #-2800]
	orr	r3, r3, r2, asl #6
	strh	r3, [r5, #24]	@ movhi
	ldrb	r3, [r4, #-544]	@ zero_extendqisi2
	strb	r3, [r4, #-703]
	ldr	r3, [r4, #-3116]
	str	r3, [r4, #-684]
	ldr	r3, [r4, #-3148]
	str	r3, [r4, #-676]
	ldr	r3, [r4, #-3144]
	str	r3, [r4, #-680]
	sub	r3, r4, #3072
	ldrh	r3, [r3]
	strh	r3, [r5, #44]	@ movhi
	sub	r3, r4, #3056
	ldrh	r3, [r3, #-14]
	strh	r3, [r5, #46]	@ movhi
	sub	r3, r4, #3184
	ldrh	r2, [r3, #-6]
	bl	ftl_memset
	mov	r1, r5
	sub	r5, r4, #3248
	mov	r2, #48
	ldr	r0, [r4, #-2800]
	bl	ftl_memcpy
	ldrh	r2, [r5, #-12]
	ldr	r0, [r4, #-2800]
	ldr	r1, [r4, #-2940]
	mov	r2, r2, asl #1
	add	r0, r0, #48
	bl	ftl_memcpy
	ldrh	r2, [r5, #-12]
	ldr	r0, [r4, #-2800]
	add	r3, r2, #24
	ldr	r1, [r4, #-2928]
	ubfx	r3, r3, #1, #30
	mov	r2, r2, lsr #3
	add	r2, r2, #4
	add	r0, r0, r3, asl #2
	bl	ftl_memcpy
	sub	r2, r4, #3152
	ldrh	r3, [r2, #-8]
	cmp	r3, #0
	beq	.L2125
	ldrh	r1, [r5, #-12]
	ldrh	r2, [r2, #-12]
	ldr	r0, [r4, #-2800]
	mov	r3, r1, lsr #3
	add	r3, r3, r1, asl #1
	mov	r2, r2, asl #2
	add	r3, r3, #52
	ldr	r1, [r4, #-2908]
	ubfx	r3, r3, #2, #14
	add	r0, r0, r3, asl #2
	bl	ftl_memcpy
.L2125:
	mov	r0, #0
	ldr	r8, .L2140
	bl	FtlUpdateVaildLpn
	ldr	r10, .L2140+16
	ldr	fp, .L2140+20
	mov	r5, #0
	movw	r9, #65535
.L2126:
	ldrh	r1, [r7]
	ldrh	r0, [r7, #2]
	ldr	r3, [r4, #-3008]
	orr	r2, r0, r1, asl #10
	str	r2, [r4, #-2804]
	ldrh	r2, [r10]
	str	r3, [r4, #-2800]
	sub	r2, r2, #1
	ldr	r3, [r4, #-2976]
	cmp	r0, r2
	str	r3, [r4, #-2796]
	ldr	r3, .L2140+24
	blt	.L2127
	ldrh	r9, [r3, #4]
	mov	r2, #0
	strh	r1, [r3, #4]	@ movhi
	strh	r2, [r3, #2]	@ movhi
	str	r3, [sp, #4]
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [sp, #4]
	strh	r0, [r3]	@ movhi
	ldr	r3, [r8, #-3148]
	add	r2, r3, #1
	str	r2, [r8, #-3148]
	str	r3, [r8, #-396]
	mov	r2, r0, asl #10
	str	r2, [r8, #-2804]
	str	r3, [r6, #4]
	strh	r0, [r6, #2]	@ movhi
.L2127:
	ldrb	r3, [fp]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2128
	ldr	r3, .L2140+28
	ldr	r0, [r4, #-3008]
	ldrh	r1, [r3]
	bl	js_hash
	str	r0, [r6, #12]
.L2128:
	mov	r1, #1
	ldr	r0, .L2140+32
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r7, #2]
	ldr	r2, [r4, #-2808]
	add	r3, r3, #1
	cmn	r2, #1
	uxth	r3, r3
	strh	r3, [r7, #2]	@ movhi
	bne	.L2129
	cmp	r3, #1
	add	r5, r5, #1
	ldreqh	r3, [r10]
	uxth	r5, r5
	ldreq	r2, .L2140+24
	subeq	r3, r3, #1
	streqh	r3, [r2, #2]	@ movhi
	cmp	r5, #3
	bls	.L2126
	ldr	r0, .L2140+36
	mov	r2, r5
	ldr	r1, [r4, #-2804]
	bl	printk
	mov	r3, #1
	str	r3, [r4, #-2812]
	b	.L2124
.L2129:
	cmp	r2, #256
	cmpne	r3, #1
	beq	.L2126
	movw	r3, #65535
	cmp	r9, r3
	beq	.L2124
	mov	r0, r9
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L2124:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2141:
	.align	2
.L2140:
	.word	.LANCHOR2
	.word	-3932
	.word	1179929683
	.word	1342177366
	.word	.LANCHOR2-3198
	.word	.LANCHOR0
	.word	.LANCHOR2-404
	.word	.LANCHOR2-3190
	.word	.LANCHOR2-2808
	.word	.LC45
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, #0
	ldr	r10, .L2228
	mov	r1, fp
	.pad #28
	sub	sp, sp, #28
	sub	r3, r10, #2816
	sub	r5, r10, #388
	ldr	r2, [r10, #-3172]
	sub	r4, r10, #3168
	ldr	r0, [r10, #-2900]
	strh	fp, [r3, #-4]	@ movhi
	mov	r2, r2, asl #2
	strh	fp, [r5]	@ movhi
	bl	ftl_memset
	ldr	r2, [r10, #-3172]
	mov	r1, fp
	ldr	r0, [r10, #-2936]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r4, #-12]
	mov	r1, fp
	ldr	r0, [r10, #-2916]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r4, #-12]
	mov	r1, fp
	ldr	r0, [r10, #-2924]
	mov	r4, r10
	mov	r2, r2, asl #1
	bl	ftl_memset
	sub	r0, r10, #404
	mov	r1, #255
	mov	r2, #16
	bl	ftl_memset
	sub	r3, r10, #3248
	ldrh	fp, [r3, #-12]
	str	r5, [sp, #12]
	mov	r5, r10
.L2143:
	ldr	r3, .L2228+4
	ldr	r1, .L2228
	ldrh	r3, [r3]
	cmp	r3, fp
	bls	.L2184
	ldr	r1, [r5, #-2968]
	mov	r6, #0
	ldr	r3, .L2228+8
	mov	r8, r6
	ldr	r2, [r5, #-3036]
	mov	r7, #36
	str	r1, [sp]
	ldr	r1, .L2228+12
	ldrh	ip, [r3]
	ldr	r3, [r5, #-2996]
	sub	r9, r1, #52
	ldrh	r10, [r1]
.L2185:
	uxth	r1, r6
	cmp	r1, ip
	bcs	.L2223
	mov	r1, fp
	ldrb	r0, [r9, r6]	@ zero_extendqisi2
	str	r3, [sp, #20]
	str	r2, [sp, #16]
	str	ip, [sp, #8]
	bl	V2P_block
	str	r0, [sp, #4]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldmib	sp, {r1, ip}
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #20]
	bne	.L2144
	mla	r0, r7, r8, r2
	mov	r1, r1, asl #10
	stmib	r0, {r1, r3}
	mul	r1, r10, r8
	add	r8, r8, #1
	uxth	r8, r8
	add	lr, r1, #3
	cmp	r1, #0
	movlt	r1, lr
	ldr	lr, [sp]
	bic	r1, r1, #3
	add	r1, lr, r1
	str	r1, [r0, #12]
.L2144:
	add	r6, r6, #1
	b	.L2185
.L2223:
	cmp	r8, #0
	beq	.L2147
	ldr	r0, [r4, #-3036]
	mov	r1, r8
	mov	r2, #1
	bl	FlashReadPages
	mov	r3, #0
.L2221:
	str	r3, [sp]
	ldrh	r3, [sp]
	cmp	r3, r8
	bcs	.L2147
	ldr	r3, [sp]
	mov	r9, #36
	mul	r9, r9, r3
	ldr	r3, [r4, #-3036]
	add	r2, r3, r9
	ldr	r3, [r3, r9]
	ldr	r6, [r2, #4]
	cmn	r3, #1
	ldr	r7, [r2, #12]
	ubfx	r6, r6, #10, #16
	bne	.L2150
	mov	r10, #16
	movw	ip, #65535
.L2149:
	ldr	r0, [r4, #-3036]
	mov	r1, #1
	mov	r2, r1
	str	ip, [sp, #4]
	add	r0, r0, r9
	ldr	r3, [r0, #4]
	add	r3, r3, #1
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldrh	r3, [r7]
	ldr	ip, [sp, #4]
	cmp	r3, ip
	ldreq	r3, .L2228
	mvneq	r2, #0
	ldreq	r3, [r3, #-3036]
	streq	r2, [r3, r9]
	beq	.L2150
.L2151:
	ldr	r3, [r5, #-3036]
	ldr	r3, [r3, r9]
	cmn	r3, #1
	bne	.L2150
	sub	r10, r10, #1
	uxth	r10, r10
	cmp	r10, #0
	bne	.L2149
.L2150:
	ldr	r3, [r5, #-3036]
	ldr	r3, [r3, r9]
	ldr	r9, .L2228
	cmn	r3, #1
	beq	.L2153
	ldr	r2, [r5, #-3148]
	ldr	r3, [r7, #4]
	cmn	r2, #1
	beq	.L2154
	cmp	r2, r3
	bhi	.L2155
.L2154:
	cmn	r3, #1
	addne	r2, r3, #1
	strne	r2, [r9, #-3148]
.L2155:
	ldrh	r2, [r7]
	movw	r1, #61604
	cmp	r2, r1
	beq	.L2157
	bhi	.L2158
	movw	r3, #61574
	cmp	r2, r3
	bne	.L2156
	ldr	r2, .L2228+16
	ldr	lr, [r4, #-2916]
	ldrh	ip, [r2]
	add	r2, r2, #360
	sub	r0, ip, #1
	ldrh	r1, [r2]
	uxth	r3, r0
	rsb	r2, r1, r0
	str	r2, [sp, #4]
	b	.L2171
.L2158:
	movw	r3, #61634
	cmp	r2, r3
	beq	.L2160
	movw	r3, #65535
	cmp	r2, r3
	moveq	r0, r6
	beq	.L2222
	b	.L2156
.L2160:
	ldr	ip, [r4, #-3172]
	ldr	r2, [sp, #12]
	ldr	lr, [r4, #-2900]
	uxth	r1, ip
	ldrh	r2, [r2]
	sub	r3, r1, #1
	rsb	r1, r2, r1
	uxth	r3, r3
	sub	r1, r1, #1
	sxth	r1, r1
	str	r1, [sp, #8]
.L2162:
	ldr	r1, [sp, #8]
	sxth	r0, r3
	cmp	r0, r1
	ble	.L2224
	mov	r10, r0, asl #2
	ldr	r1, [r7, #4]
	str	r10, [sp, #4]
	ldr	r10, [lr, r0, asl #2]
	cmp	r1, r10
	bls	.L2163
	ldr	r1, [lr]
	cmp	r1, #0
	bne	.L2164
	cmp	r2, ip
	addne	r2, r2, #1
	ldrne	r1, .L2228+20
	strneh	r2, [r1]	@ movhi
.L2164:
	uxth	lr, r3
	mov	r1, #0
.L2165:
	uxth	r2, r1
	cmp	r2, lr
	bcs	.L2225
	ldr	ip, [r5, #-2900]
	sxth	r2, r2
	add	r1, r1, #1
	add	r10, ip, r2, asl #2
	ldr	r10, [r10, #4]
	str	r10, [ip, r2, asl #2]
	mov	r2, r2, asl #1
	ldr	ip, [r5, #-2936]
	add	r10, ip, r2
	ldrh	r10, [r10, #2]
	strh	r10, [ip, r2]	@ movhi
	b	.L2165
.L2225:
	ldr	r2, [r9, #-2900]
	mov	r0, r0, asl #1
	ldr	r1, [r7, #4]
	ldr	ip, [sp, #4]
	str	r1, [r2, ip]
	ldr	r2, [r9, #-2936]
	strh	r6, [r2, r0]	@ movhi
	sxth	r0, r3
	cmp	r0, #0
	bge	.L2167
	b	.L2156
.L2163:
	sub	r3, r3, #1
	uxth	r3, r3
	b	.L2162
.L2224:
	cmp	r0, #0
	bge	.L2198
	b	.L2156
.L2167:
	ldr	r2, .L2228+20
	ldr	r1, [r5, #-3172]
	ldrh	r2, [r2]
	rsb	r1, r2, r1
	sub	r1, r1, #1
	sxth	r1, r1
	cmp	r0, r1
	bgt	.L2156
.L2198:
	ldr	r1, .L2228+20
	add	r2, r2, #1
	sxth	r3, r3
	strh	r2, [r1]	@ movhi
	ldr	r2, [r4, #-2900]
	ldr	r1, [r7, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #-2936]
	b	.L2219
.L2177:
	mov	r10, r2, asl #2
	ldr	r0, [r7, #4]
	str	r10, [sp, #8]
	ldr	r10, [lr, r2, asl #2]
	cmp	r0, r10
	bhi	.L2226
	sub	r3, r3, #1
	uxth	r3, r3
.L2171:
	ldr	r0, [sp, #4]
	sxth	r2, r3
	cmp	r2, r0
	bgt	.L2177
	b	.L2176
.L2226:
	ldr	r0, [lr]
	cmp	r0, #0
	bne	.L2173
	cmp	r1, ip
	addne	r1, r1, #1
	ldrne	r0, .L2228+24
	strneh	r1, [r0]	@ movhi
.L2173:
	uxth	lr, r3
	mov	r0, #0
.L2174:
	uxth	r1, r0
	cmp	r1, lr
	bcs	.L2227
	ldr	ip, [r5, #-2916]
	sxth	r1, r1
	add	r0, r0, #1
	add	r10, ip, r1, asl #2
	ldr	r10, [r10, #4]
	str	r10, [ip, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	ip, [r5, #-2924]
	add	r10, ip, r1
	ldrh	r10, [r10, #2]
	strh	r10, [ip, r1]	@ movhi
	b	.L2174
.L2227:
	ldr	r1, [r9, #-2916]
	mov	r2, r2, asl #1
	ldr	r0, [r7, #4]
	ldr	ip, [sp, #8]
	str	r0, [r1, ip]
	ldr	r1, [r9, #-2924]
	strh	r6, [r1, r2]	@ movhi
.L2176:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L2156
	ldr	r1, .L2228+24
	sub	r0, r1, #360
	ldrh	r2, [r1]
	ldrh	r9, [r0]
	sub	r9, r9, #1
	rsb	r9, r2, r9
	sxth	r9, r9
	cmp	r3, r9
	bgt	.L2156
	add	r2, r2, #1
	strh	r2, [r1]	@ movhi
	ldr	r2, [r5, #-2916]
	ldr	r1, [r7, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r5, #-2924]
.L2219:
	strh	r6, [r2, r3]	@ movhi
	b	.L2156
.L2157:
	ldr	r2, .L2228+28
	ldrh	r1, [r2]
	movw	r2, #65535
	cmp	r1, r2
	ldreq	r2, .L2228+28
	streqh	r6, [r2]	@ movhi
	beq	.L2220
	ldr	r3, .L2228+28
	ldrh	r0, [r3, #4]
	cmp	r0, r2
	beq	.L2179
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L2179:
	ldr	r3, [r7, #4]
	ldr	r2, [r9, #-396]
	cmp	r2, r3
	ldr	r3, .L2228+28
	strcsh	r6, [r3, #4]	@ movhi
	bcs	.L2156
	ldrh	r3, [r3]
	ldr	r2, .L2228+28
	strh	r3, [r2, #4]	@ movhi
	ldr	r3, [r7, #4]
	strh	r6, [r2]	@ movhi
.L2220:
	str	r3, [r5, #-396]
	b	.L2156
.L2153:
	ldr	r3, .L2228+32
	mov	r0, r6
	ldrb	r1, [r3, #3904]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L2181
.L2222:
	mov	r1, #0
.L2181:
	bl	FtlFreeSysBlkQueueIn
.L2156:
	ldr	r3, [sp]
	add	r3, r3, #1
	b	.L2221
.L2147:
	add	r8, fp, #1
	uxth	fp, r8
	b	.L2143
.L2184:
	ldr	ip, [r1, #-2936]
	ldrh	r2, [ip]
	cmp	r2, #0
	beq	.L2186
.L2189:
	ldr	r0, [r1, #-2924]
	ldrh	r2, [r0]
	cmp	r2, #0
	beq	.L2187
	b	.L2210
.L2186:
	sub	r3, r1, #388
	ldrh	r3, [r3]
	cmp	r3, #0
	ldrne	lr, [r1, #-3172]
	beq	.L2189
.L2190:
	uxth	r3, r2
	sxth	r0, r3
	cmp	r0, lr
	bcs	.L2189
	mov	r4, r0, asl #1
	add	r2, r2, #1
	ldrh	r4, [ip, r4]
	cmp	r4, #0
	beq	.L2190
	ldr	ip, .L2228
	mov	r6, #0
.L2191:
	ldr	lr, [r1, #-3172]
	sxth	r2, r3
	cmp	r2, lr
	bcs	.L2189
	ldr	r4, [ip, #-2936]
	mov	lr, r2, asl #1
	rsb	r5, r0, r2
	add	r3, r3, #1
	ldrh	r8, [r4, lr]
	mov	r7, r5, asl #1
	uxth	r3, r3
	strh	r8, [r4, r7]	@ movhi
	ldr	r4, [ip, #-2900]
	ldr	r2, [r4, r2, asl #2]
	str	r2, [r4, r5, asl #2]
	ldr	r2, [ip, #-2936]
	strh	r6, [r2, lr]	@ movhi
	b	.L2191
.L2187:
	ldr	r3, .L2228+36
	ldrh	r1, [r3, #-4]
	cmp	r1, #0
	subne	r3, r3, #352
	ldrneh	ip, [r3, #-12]
	beq	.L2210
.L2194:
	uxth	r3, r2
	sxth	r1, r3
	cmp	r1, ip
	bge	.L2210
	mov	lr, r1, asl #1
	add	r2, r2, #1
	ldrh	lr, [r0, lr]
	cmp	lr, #0
	beq	.L2194
	ldr	r0, .L2228
	mov	r6, #0
	ldr	r5, .L2228+16
.L2195:
	ldrh	ip, [r5]
	sxth	r2, r3
	cmp	r2, ip
	bge	.L2210
	ldr	lr, [r0, #-2924]
	mov	ip, r2, asl #1
	rsb	r4, r1, r2
	add	r3, r3, #1
	ldrh	r8, [lr, ip]
	mov	r7, r4, asl #1
	uxth	r3, r3
	strh	r8, [lr, r7]	@ movhi
	ldr	lr, [r0, #-2916]
	ldr	r2, [lr, r2, asl #2]
	str	r2, [lr, r4, asl #2]
	ldr	r2, [r0, #-2924]
	strh	r6, [r2, ip]	@ movhi
	b	.L2195
.L2210:
	mov	r0, #0
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2229:
	.align	2
.L2228:
	.word	.LANCHOR2
	.word	.LANCHOR2-3258
	.word	.LANCHOR2-3268
	.word	.LANCHOR2-3188
	.word	.LANCHOR2-3180
	.word	.LANCHOR2-388
	.word	.LANCHOR2-2820
	.word	.LANCHOR2-404
	.word	.LANCHOR0
	.word	.LANCHOR2-2816
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
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
	ldr	r4, .L2233
	sub	r5, r4, #2960
	ldr	r2, [r4, #-2952]
	ldrh	r1, [r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #-2952]
	ldr	r2, [r3]
	ldr	r3, .L2233+4
	cmp	r2, r3
	beq	.L2231
	ldr	r1, .L2233+8
	ldr	r0, .L2233+12
	bl	printk
	ldrh	r2, [r5]
	ldr	r0, [r4, #-2952]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2231:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L2234:
	.align	2
.L2233:
	.word	.LANCHOR2
	.word	1112818501
	.word	.LC47
	.word	.LC46
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	ftl_set_blk_mode
	.type	ftl_set_blk_mode, %function
ftl_set_blk_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L2239
	mov	r3, r0, lsr #5
	cmp	r1, #0
	and	r0, r0, #31
	uxth	r3, r3
	mov	ip, #1
	ldr	r1, [r2, #-2928]
	ldr	r2, [r1, r3, asl #2]
	orrne	r0, r2, ip, asl r0
	biceq	r0, r2, ip, asl r0
	str	r0, [r1, r3, asl #2]
	bx	lr
.L2240:
	.align	2
.L2239:
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
	ldr	r3, .L2242
	mov	r2, r0, lsr #5
	and	r0, r0, #31
	ldr	r3, [r3, #-2928]
	ldr	r3, [r3, r2, asl #2]
	mov	r0, r3, lsr r0
	and	r0, r0, #1
	bx	lr
.L2243:
	.align	2
.L2242:
	.word	.LANCHOR2
	.fnend
	.size	ftl_get_blk_mode, .-ftl_get_blk_mode
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
	ldr	r1, .L2265
	ldr	r0, .L2265+4
	bl	printk
	ldr	r0, .L2265+8
	mov	r1, #8192
	ldr	r5, .L2265+12
	bl	__memzero
	sub	r7, r5, #384
.L2245:
	ldr	r3, [r5, #-2816]
	ldr	r6, .L2265+12
	cmp	r4, r3
	bcs	.L2263
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L2246
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	mov	r0, r0, asl #1
	ldrh	r3, [r7, r0]
	add	r3, r3, #1
	strh	r3, [r7, r0]	@ movhi
.L2246:
	add	r4, r4, #1
	b	.L2245
.L2263:
	ldr	r5, .L2265+16
	sub	r8, r6, #384
	mov	r4, #0
	mov	r7, r6
.L2248:
	ldrh	r3, [r5]
	uxth	r1, r4
	ldr	r0, .L2265+12
	cmp	r3, r1
	bls	.L2264
	ldr	r2, [r7, #-2940]
	mov	r3, r1, asl #1
	ldrh	r2, [r2, r3]
	ldrh	r3, [r8, r3]
	cmp	r2, r3
	beq	.L2249
	ldr	r0, .L2265+20
	bl	printk
.L2249:
	add	r4, r4, #1
	b	.L2248
.L2264:
	ldr	r3, [r0, #-656]
	cmp	r3, #0
	beq	.L2244
	sub	r2, r0, #652
	sub	r8, r0, #384
	mov	r5, #0
	mov	r9, #6
	ldrh	r7, [r2]
	ldr	r2, [r0, #-2888]
	rsb	r3, r2, r3
	ldr	r2, .L2265+24
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r4, r3
.L2252:
	uxth	r3, r5
	cmp	r3, r7
	bcs	.L2244
	ldr	r2, [r6, #-2940]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	beq	.L2253
	ldr	r0, .L2265+28
	mov	r1, r4
	ldrh	r3, [r8, r3]
	bl	printk
.L2253:
	mul	r4, r9, r4
	ldr	r3, [r6, #-2888]
	add	r5, r5, #1
	ldrh	r4, [r3, r4]
	movw	r3, #65535
	cmp	r4, r3
	bne	.L2252
.L2244:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2266:
	.align	2
.L2265:
	.word	.LANCHOR3+16
	.word	.LC48
	.word	.LANCHOR2-384
	.word	.LANCHOR2
	.word	.LANCHOR2-3260
	.word	.LC49
	.word	-1431655765
	.word	.LC50
	.fnend
	.size	FtlCheckVpc, .-FtlCheckVpc
	.align	2
	.global	FtlDumpSysBlock
	.type	FtlDumpSysBlock, %function
FtlDumpSysBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r0, asl #10
	ldr	r4, .L2276
	.pad #24
	sub	sp, sp, #24
	ldr	r8, .L2276+4
	mov	r6, r0
	mov	r5, #0
	ldr	r3, [r4, #-3008]
	str	r3, [r4, #-2800]
	ldr	r3, [r4, #-2976]
	str	r3, [r4, #-2796]
.L2268:
	ldrh	r2, [r8]
	sxth	r3, r5
	cmp	r3, r2
	bge	.L2275
	mov	r1, #1
	ldr	r0, .L2276+8
	mov	r2, r1
	orr	r3, r3, r7
	str	r3, [r4, #-2804]
	bl	FlashReadPages
	ldr	r3, [r4, #-2796]
	mov	r1, r6
	ldr	r0, .L2276+12
	ldr	r2, [r3]
	str	r2, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	ldr	r2, [r3, #8]
	str	r2, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r2, [r4, #-2808]
	str	r3, [sp, #12]
	ldr	r3, [r4, #-2800]
	ldr	r3, [r3]
	str	r3, [sp, #16]
	ldr	r3, [r4, #-2804]
	bl	printk
	ldr	r3, [r4, #-2796]
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L2269
	ldr	r0, .L2276+16
	mov	r2, #4
	ldr	r1, [r4, #-3008]
	mov	r3, #768
	bl	rknand_print_hex
.L2269:
	add	r5, r5, #1
	b	.L2268
.L2275:
	add	sp, sp, #24
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2277:
	.align	2
.L2276:
	.word	.LANCHOR2
	.word	.LANCHOR2-3198
	.word	.LANCHOR2-2808
	.word	.LC51
	.word	.LC52
	.fnend
	.size	FtlDumpSysBlock, .-FtlDumpSysBlock
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
	ldr	r0, .L2288
	bl	printk
	ldr	r6, .L2288+4
	mov	r5, #0
	mov	r4, r6
.L2279:
	ldr	r3, [r6, #-2816]
	cmp	r5, r3
	bcs	.L2287
	mov	r0, r5
	add	r1, sp, #28
	mov	r2, #0
	bl	log2phys
	ubfx	r3, r5, #0, #11
	cmp	r3, #0
	bne	.L2280
	ldr	r0, .L2288+8
	mov	r1, r5
	ldr	r2, [sp, #28]
	bl	printk
.L2280:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L2282
	str	r3, [r4, #-2804]
	mov	r2, #0
	ldr	r3, [r4, #-3008]
	mov	r1, #1
	ldr	r7, [r4, #-2976]
	ldr	r0, .L2288+12
	str	r3, [r4, #-2800]
	str	r5, [r4, #-2792]
	str	r7, [r4, #-2796]
	str	r2, [r4, #-2808]
	bl	FlashReadPages
	ldr	r3, [r4, #-2808]
	cmn	r3, #1
	cmpne	r3, #256
	beq	.L2283
	ldr	r3, [r7, #8]
	cmp	r3, r5
	beq	.L2282
.L2283:
	ldr	r3, [r4, #-2796]
	ldr	r2, [r4, #-2800]
	ldr	r0, .L2288+16
	ldr	r1, [r3, #4]
	str	r1, [sp]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2]
	str	r1, [sp, #12]
	mov	r1, r5
	ldr	r2, [r2, #4]
	str	r2, [sp, #16]
	ldr	r2, [r4, #-2804]
	ldr	r3, [r3]
	bl	printk
.L2282:
	add	r5, r5, #1
	b	.L2279
.L2287:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2289:
	.align	2
.L2288:
	.word	.LC53
	.word	.LANCHOR2
	.word	.LC54
	.word	.LANCHOR2-2808
	.word	.LC55
	.fnend
	.size	Ftlscanalldata, .-Ftlscanalldata
	.align	2
	.global	dump_map_info
	.type	dump_map_info, %function
dump_map_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldr	r5, .L2308
	ldr	fp, .L2308+4
	ldrh	r6, [r5, #-12]
	add	r5, r5, #3248
	mov	r8, r5
.L2291:
	ldrh	r3, [fp]
	cmp	r3, r6
	bls	.L2304
	ldr	r1, .L2308+8
	mov	r7, #0
	ldr	r2, [r5, #-3036]
	mov	r4, r7
	ldr	r3, [r5, #-2996]
	add	r1, r1, #28
	ldr	r10, [r5, #-2968]
	ldrh	ip, [r1, #-28]
	ldrh	r9, [r1, #52]
	str	r1, [sp, #28]
.L2300:
	uxth	r1, r7
	cmp	r1, ip
	bcs	.L2306
	str	r3, [sp, #44]
	mov	r1, r6
	ldr	r3, [sp, #28]
	str	r2, [sp, #40]
	str	ip, [sp, #36]
	ldrb	r0, [r3, r7]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #32]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldr	r1, [sp, #32]
	ldr	ip, [sp, #36]
	ldr	r2, [sp, #40]
	ldr	r3, [sp, #44]
	bne	.L2292
	mov	r0, #36
	mov	r1, r1, asl #10
	mla	r0, r0, r4, r2
	stmib	r0, {r1, r3}
	mul	r1, r9, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	lr, r1, #3
	cmp	r1, #0
	movlt	r1, lr
	bic	r1, r1, #3
	add	r1, r10, r1
	str	r1, [r0, #12]
.L2292:
	add	r7, r7, #1
	b	.L2300
.L2306:
	cmp	r4, #0
	beq	.L2295
	ldr	r0, [r8, #-3036]
	mov	r1, r4
	mov	r2, #1
	mov	r7, #0
	bl	FlashReadPages
	mov	r9, #36
.L2296:
	uxth	r3, r7
	cmp	r3, r4
	bcs	.L2295
	ldr	r3, [r8, #-3036]
	ldr	r0, .L2308+12
	mla	r3, r9, r7, r3
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
	b	.L2296
.L2295:
	add	r6, r6, #1
	uxth	r6, r6
	b	.L2291
.L2304:
	ldr	r4, .L2308+16
	mov	r7, #0
	ldr	r8, .L2308+20
.L2299:
	ldr	r5, .L2308+16
	sxth	r6, r7
	sub	r3, r5, #388
	ldrh	r3, [r3]
	cmp	r6, r3
	bge	.L2302
	mov	r6, r6, asl #1
	mov	r5, #0
.L2303:
	ldrh	r2, [r8]
	sxth	r3, r5
	add	r5, r5, #1
	cmp	r3, r2
	bge	.L2307
	ldr	r2, [r4, #-2936]
	mov	r1, #1
	ldr	r0, .L2308+24
	ldrh	r2, [r2, r6]
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r4, #-2804]
	bl	FlashReadPages
	ldr	r3, [r4, #-2796]
	ldr	r1, [r4, #-2936]
	ldr	r2, [r4, #-2800]
	ldr	r0, [r3]
	ldrh	r1, [r1, r6]
	str	r0, [sp]
	ldr	r0, [r3, #4]
	str	r0, [sp, #4]
	ldr	r0, [r3, #8]
	str	r0, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r0, .L2308+28
	str	r3, [sp, #12]
	ldr	r3, [r2]
	str	r3, [sp, #16]
	ldr	r3, [r2, #4]
	str	r3, [sp, #20]
	ldr	r2, [r4, #-2808]
	ldr	r3, [r4, #-2804]
	bl	printk
	b	.L2303
.L2307:
	add	r7, r7, #1
	b	.L2299
.L2302:
	sub	r4, r5, #3152
	ldr	r1, [r5, #-2936]
	ldr	r3, [r5, #-3172]
	mov	r2, #2
	ldr	r0, .L2308+32
	bl	rknand_print_hex
	ldr	r1, [r5, #-2908]
	ldrh	r3, [r4, #-12]
	mov	r2, #4
	ldr	r0, .L2308+36
	bl	rknand_print_hex
	ldr	r0, .L2308+40
	ldr	r1, [r5, #-2904]
	mov	r2, #4
	ldrh	r3, [r4, #-12]
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	rknand_print_hex
.L2309:
	.align	2
.L2308:
	.word	.LANCHOR2-3248
	.word	.LANCHOR2-3258
	.word	.LANCHOR2-3268
	.word	.LC56
	.word	.LANCHOR2
	.word	.LANCHOR2-3198
	.word	.LANCHOR2-2808
	.word	.LC57
	.word	.LC58
	.word	.LC59
	.word	.LC60
	.fnend
	.size	dump_map_info, .-dump_map_info
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
	ldrh	r9, [r0, #6]
	mov	r1, #0
	ldr	r7, [r0, #24]
	str	r3, [sp, #12]
	ldrh	r3, [r0, #8]
	mov	r2, r9, asl #2
	ldr	r8, [r0, #12]
	mov	r0, r7
	ldr	r5, .L2353
	str	r3, [sp, #8]
	bl	ftl_memset
	mov	r1, #0
	str	r1, [r4, #32]
	ldr	r3, [r5, #-3008]
	mov	r10, r5
	ldr	r6, [r5, #-2976]
	str	r1, [r4, #28]
	str	r3, [r5, #-2800]
	mvn	r3, #0
	str	r6, [r5, #-2796]
	strh	r3, [r4]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #1
	str	r3, [r4, #36]
	str	r1, [sp, #4]
.L2311:
	ldrh	r2, [sp, #4]
	ldr	r3, [sp, #8]
	sxth	fp, r2
	cmp	fp, r3
	bge	.L2329
	ldr	r3, [sp, #8]
	sub	r1, r3, #1
	cmp	fp, r1
	mov	r1, fp, asl #1
	bne	.L2312
	ldrh	r0, [r8, r1]
	add	r10, r8, r1
	mov	r1, #1
	str	r2, [sp, #4]
	bl	FtlGetLastWrittenPage
	mov	r8, #0
	ldr	r2, [sp, #4]
	add	r3, r0, #1
	strh	r3, [r4, #2]	@ movhi
	sxth	r0, r0
	ldr	r3, [sp, #12]
	strh	r2, [r4]	@ movhi
	ldr	r3, [r3, fp, asl #2]
	ldr	fp, .L2353
	str	r3, [r4, #28]
	add	r3, r0, #1
	str	r3, [sp, #4]
.L2313:
	ldr	r2, [sp, #4]
	sxth	r3, r8
	cmp	r3, r2
	bge	.L2329
	ldrh	r2, [r10]
	mov	r1, #1
	ldr	r0, .L2353+4
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #-2804]
	bl	FlashReadPages
	ldr	r3, .L2353+8
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2314
	ldr	r3, [fp, #-2796]
	ldr	r3, [r3, #12]
	cmp	r3, #0
	beq	.L2314
	ldr	r2, .L2353+12
	ldr	r0, [fp, #-2800]
	str	r3, [sp, #8]
	ldrh	r1, [r2]
	bl	js_hash
	ldr	r3, [sp, #8]
	cmp	r3, r0
	mvnne	r3, #0
	strne	r3, [fp, #-2808]
.L2314:
	ldr	r3, [fp, #-2808]
	cmn	r3, #1
	beq	.L2315
	ldrh	r3, [r6, #8]
	cmp	r3, r9
	bcs	.L2315
	ldrh	r2, [r4, #4]
	ldrh	r1, [r6]
	cmp	r1, r2
	ldreq	r2, [fp, #-2804]
	streq	r2, [r7, r3, asl #2]
.L2315:
	add	r8, r8, #1
	b	.L2313
.L2329:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r3, .L2353+16
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3, #-14]
	cmp	r2, r3
	bne	.L2318
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
	b	.L2318
.L2312:
	ldr	r2, [r5, #-3008]
	add	r3, r8, r1
	str	r3, [sp, #16]
	ldr	r3, .L2353+20
	str	r2, [r5, #-2800]
	ldrh	r1, [r8, r1]
	ldrh	r2, [r3]
	ldr	r0, .L2353+4
	sub	r2, r2, #1
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [r5, #-2804]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [r5, #-2808]
	cmn	r2, #1
	beq	.L2331
	ldrh	r1, [r6]
	ldrh	r2, [r4, #4]
	cmp	r1, r2
	bne	.L2331
	ldrh	r1, [r6, #8]
	movw	r2, #64245
	cmp	r1, r2
	bne	.L2331
	mov	r0, #0
	mov	fp, #8
	mov	lr, #4
.L2320:
	ldr	r3, .L2353+20
	uxth	r2, r0
	sxth	r1, r2
	ldrh	ip, [r3]
	sub	ip, ip, #1
	cmp	r1, ip
	bge	.L2323
	ldr	ip, [r10, #-3008]
	add	r0, r0, #1
	ldr	r1, [ip, r1, asl #3]
	uxth	r1, r1
	cmp	r1, r9
	smlabbcc	r2, r2, fp, lr
	ldrcc	r2, [ip, r2]
	strcc	r2, [r7, r1, asl #2]
	b	.L2320
.L2331:
	mov	fp, #0
.L2351:
	ldr	r3, .L2353+20
	sxth	r2, fp
	ldrh	r1, [r3]
	cmp	r2, r1
	bge	.L2323
	ldr	r3, [sp, #16]
	ldr	r0, .L2353+4
	ldrh	r1, [r3]
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [r10, #-2804]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, .L2353+8
	ldrb	r2, [r2]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2324
	ldr	r2, [r10, #-2796]
	ldr	r2, [r2, #12]
	cmp	r2, #0
	beq	.L2324
	ldr	r1, .L2353+12
	ldr	r0, [r10, #-2800]
	str	r2, [sp, #20]
	ldrh	r1, [r1]
	bl	js_hash
	ldr	r2, [sp, #20]
	cmp	r2, r0
	mvnne	r2, #0
	strne	r2, [r10, #-2808]
.L2324:
	ldr	r2, [r10, #-2808]
	cmn	r2, #1
	beq	.L2325
	ldrh	r2, [r6, #8]
	cmp	r2, r9
	bcs	.L2325
	ldrh	r1, [r4, #4]
	ldrh	r0, [r6]
	cmp	r0, r1
	ldreq	r1, [r10, #-2804]
	streq	r1, [r7, r2, asl #2]
.L2325:
	add	fp, fp, #1
	b	.L2351
.L2323:
	ldr	r3, [sp, #4]
	add	r3, r3, #1
	str	r3, [sp, #4]
	b	.L2311
.L2318:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2354:
	.align	2
.L2353:
	.word	.LANCHOR2
	.word	.LANCHOR2-2808
	.word	.LANCHOR0
	.word	.LANCHOR2-3190
	.word	.LANCHOR2-3184
	.word	.LANCHOR2-3198
	.fnend
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	2
	.global	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L2357
	sub	r2, r3, #3168
	sub	r0, r3, #456
	ldrh	r1, [r2, #-12]
	ldrh	r2, [r2, #-10]
	strh	r1, [r0, #10]	@ movhi
	strh	r2, [r0, #6]	@ movhi
	ldr	r2, [r3, #-2924]
	ldr	r1, .L2357+4
	str	r2, [r3, #-444]
	ldr	r2, [r3, #-2916]
	strh	r1, [r0, #4]	@ movhi
	sub	r1, r3, #2816
	str	r2, [r3, #-440]
	ldr	r2, [r3, #-2920]
	ldrh	r1, [r1, #-4]
	str	r2, [r3, #-436]
	ldr	r2, [r3, #-2912]
	strh	r1, [r0, #8]	@ movhi
	str	r2, [r3, #-432]
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2358:
	.align	2
.L2357:
	.word	.LANCHOR2
	.word	-3962
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
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
	ldr	r4, .L2364
	mvn	r6, #0
	ldr	r2, [r4, #-3172]
	ldr	r0, [r4, #-2932]
	mov	r2, r2, asl #1
	bl	ftl_memset
	sub	r3, r4, #3184
	sub	r2, r4, #3152
	mov	r1, #255
	ldrh	r3, [r3, #-6]
	ldrh	r2, [r2, #-10]
	ldr	r0, [r4, #-2892]
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r0, .L2364+4
	mov	r1, #0
	mov	lr, #12
	sub	r7, r0, #28
	mov	r5, r1
.L2360:
	ldrh	r2, [r0]
	add	ip, r1, #1
	uxth	r1, r1
	ldr	r3, .L2364
	cmp	r2, r1
	sub	r8, r3, #3152
	bls	.L2363
	mul	r8, lr, r1
	ldr	r3, [r4, #-2896]
	add	r2, r3, r8
	str	r5, [r2, #4]
	strh	r6, [r3, r8]	@ movhi
	ldrh	r2, [r7]
	ldr	r3, [r4, #-2896]
	mul	r2, r1, r2
	add	r3, r3, r8
	ldr	r1, [r4, #-2892]
	bic	r2, r2, #3
	add	r2, r1, r2
	mov	r1, ip
	str	r2, [r3, #8]
	b	.L2360
.L2363:
	sub	r2, r3, #504
	mvn	r1, #0
	strh	r1, [r2, #2]	@ movhi
	strh	r1, [r2]	@ movhi
	ldr	r1, [r3, #-3172]
	strh	r1, [r2, #10]	@ movhi
	ldr	r1, .L2364+8
	strh	r1, [r2, #4]	@ movhi
	sub	r1, r3, #388
	ldrh	r1, [r1]
	strh	r1, [r2, #8]	@ movhi
	ldrh	r1, [r8, #-12]
	strh	r1, [r2, #6]	@ movhi
	ldr	r2, [r3, #-2936]
	str	r2, [r3, #-492]
	ldr	r2, [r3, #-2900]
	str	r2, [r3, #-488]
	ldr	r2, [r3, #-2932]
	str	r2, [r3, #-484]
	ldr	r2, [r3, #-2908]
	str	r2, [r3, #-480]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2365:
	.align	2
.L2364:
	.word	.LANCHOR2
	.word	.LANCHOR2-3162
	.word	-3902
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
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
	ldr	r0, .L2368
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2369:
	.align	2
.L2368:
	.word	.LANCHOR2-504
	.fnend
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	r3, #0
	strh	r3, [r0, #4]	@ movhi
	ldr	r3, .L2380
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	movw	r4, #65535
	ldrh	lr, [r3, #-4]
	add	r3, r2, #7
	add	r3, r0, r3, asl #1
.L2371:
	cmp	r2, lr
	bcs	.L2378
	ldrh	ip, [r3, #2]!
	add	r2, r2, #1
	cmp	ip, r4
	uxth	r2, r2
	ldrneh	ip, [r0, #4]
	addne	ip, ip, #1
	strneh	ip, [r0, #4]	@ movhi
	b	.L2371
.L2378:
	ldr	r3, .L2380+4
	add	ip, r0, #14
	mov	r2, #0
	movw	r5, #65535
	ldrh	r4, [r3]
.L2374:
	uxth	r3, r2
	cmp	r3, lr
	bcs	.L2379
	ldrh	r3, [ip, #2]!
	add	r2, r2, #1
	cmp	r3, r5
	ldrneh	r3, [r0, #4]
	addne	r3, r4, r3
	subne	r3, r3, #1
	rsbne	r3, r1, r3
	strneh	r3, [r0, #4]	@ movhi
	b	.L2374
.L2379:
	ldmfd	sp!, {r4, r5, pc}
.L2381:
	.align	2
.L2380:
	.word	.LANCHOR2-3264
	.word	.LANCHOR2-3200
	.fnend
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
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
	ldr	r6, .L2392
	ldr	r4, [r6, #-2940]
	mov	r7, r0, asl #1
	ldrh	r3, [r4, r7]
	cmp	r3, #0
	addne	r3, r3, #1
	strneh	r3, [r4, r7]	@ movhi
	bne	.L2384
	ldr	r4, [r6, #-656]
	cmp	r4, #0
	beq	.L2384
	ldr	r1, [r6, #-2888]
	sub	r2, r6, #652
	ldr	lr, .L2392+4
	movw	r9, #65535
	rsb	r4, r1, r4
	ldrh	ip, [r2]
	mov	r8, r2
	mov	r4, r4, asr #1
	mul	r4, lr, r4
	mov	lr, #6
	uxth	r4, r4
.L2385:
	uxth	r2, r3
	cmp	r2, ip
	bcs	.L2384
	cmp	r4, r0
	bne	.L2386
	mov	r1, r4
	ldr	r0, .L2392+8
	bl	List_remove_node
	ldrh	r3, [r8]
	mov	r0, r4
	sub	r3, r3, #1
	strh	r3, [r8]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r2, [r6, #-2940]
	ldrh	r3, [r2, r7]
	add	r3, r3, #1
	strh	r3, [r2, r7]	@ movhi
	b	.L2384
.L2386:
	mul	r4, lr, r4
	add	r3, r3, #1
	ldrh	r4, [r1, r4]
	cmp	r4, r9
	bne	.L2385
.L2384:
	mov	r0, r5
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2393:
	.align	2
.L2392:
	.word	.LANCHOR2
	.word	-1431655765
	.word	.LANCHOR2-656
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	make_superblock
	.type	make_superblock, %function
make_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2408
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r6, r0, #16
	ldrh	r8, [r3, #-4]
	add	r7, r3, #24
	mov	r4, r0
	mvn	r9, #0
	mov	r5, #0
	strh	r5, [r0, #4]	@ movhi
	strb	r5, [r0, #7]
.L2395:
	uxth	r3, r5
	cmp	r3, r8
	bcs	.L2407
	ldrb	r0, [r7, r5]	@ zero_extendqisi2
	add	r6, r6, #2
	ldrh	r1, [r4]
	add	r5, r5, #1
	bl	V2P_block
	strh	r9, [r6, #-2]	@ movhi
	mov	r10, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	streqh	r10, [r6, #-2]	@ movhi
	ldreqb	r3, [r4, #7]	@ zero_extendqisi2
	addeq	r3, r3, #1
	streqb	r3, [r4, #7]
	b	.L2395
.L2407:
	ldr	r2, .L2408+4
	ldrb	r1, [r4, #7]	@ zero_extendqisi2
	sub	r3, r2, #3200
	ldrh	r3, [r3]
	smulbb	r3, r1, r3
	strh	r3, [r4, #4]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #9]
	ldr	r3, [r2, #-3336]
	cmp	r3, #0
	beq	.L2398
	ldrh	r3, [r4]
	ldr	r2, [r2, #-2948]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #79
	movls	r3, #1
	strlsb	r3, [r4, #9]
.L2398:
	ldr	r3, .L2408+8
	mov	r0, #0
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #1
	strneb	r3, [r4, #9]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2409:
	.align	2
.L2408:
	.word	.LANCHOR2-3264
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	make_superblock, .-make_superblock
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
	ldr	r4, .L2439
	sub	r6, r4, #3248
	sub	r7, r4, #404
	ldr	r3, [r4, #-3008]
	ldrh	r2, [r6, #-12]
	ldr	r0, [r4, #-2940]
	str	r3, [r4, #-2800]
	ldr	r3, [r4, #-2976]
	mov	r2, r2, asl #1
	str	r3, [r4, #-2796]
	bl	ftl_memset
	ldrh	r0, [r7]
	movw	r3, #65535
	cmp	r0, r3
	bne	.L2411
.L2422:
	mvn	r0, #0
	b	.L2412
.L2411:
	mov	r1, #1
	ldr	r10, .L2439+4
	bl	FtlGetLastWrittenPage
	ldrh	r8, [r7]
	mov	r5, r4
	mov	fp, r5
	uxth	r4, r0
	add	r0, r0, #1
	strh	r0, [r7, #2]	@ movhi
.L2413:
	sxth	r3, r4
	cmp	r3, #0
	blt	.L2421
	sxth	r9, r8
	mov	r1, #1
	ldr	r0, .L2439+8
	mov	r2, r1
	orr	r3, r3, r9, asl #10
	str	r3, [r5, #-2804]
	ldr	r3, [r5, #-3008]
	str	r3, [r5, #-2800]
	bl	FlashReadPages
	ldr	r3, .L2439+12
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2414
	ldr	r3, [fp, #-2796]
	ldr	r2, [r3, #12]
	cmp	r2, #0
	beq	.L2414
	ldr	r3, .L2439+16
	ldr	r0, [fp, #-2800]
	str	r2, [sp, #4]
	ldrh	r1, [r3]
	str	r3, [sp]
	bl	js_hash
	ldr	r2, [sp, #4]
	cmp	r2, r0
	beq	.L2414
	cmp	r4, #0
	ldr	r3, [sp]
	bne	.L2415
	ldrh	r2, [r7, #4]
	cmp	r9, r2
	uxthne	r8, r2
	ldrneh	r4, [r3, #-8]
	bne	.L2417
.L2415:
	mvn	r3, #0
	str	r3, [r5, #-2808]
.L2414:
	ldr	r3, [r5, #-2808]
	cmn	r3, #1
	beq	.L2417
	ldr	r3, [fp, #-3008]
	ldr	r3, [r3]
	cmp	r3, r10
	bne	.L2417
	ldr	r3, [fp, #-2976]
	ldrh	r2, [r3]
	movw	r3, #61604
	cmp	r2, r3
	bne	.L2417
.L2421:
	ldr	r4, .L2439
	mov	r2, #48
	ldr	r1, [r5, #-2800]
	sub	r0, r4, #716
	sub	r8, r4, #3248
	bl	ftl_memcpy
	ldrh	r2, [r6, #-12]
	ldr	r1, [r5, #-2800]
	ldr	r0, [r5, #-2940]
	add	r1, r1, #48
	mov	r2, r2, asl #1
	bl	ftl_memcpy
	ldrh	r2, [r6, #-12]
	ldr	r1, [r5, #-2800]
	add	r3, r2, #24
	ldr	r0, [r5, #-2928]
	ubfx	r3, r3, #1, #30
	mov	r2, r2, lsr #3
	add	r2, r2, #4
	add	r1, r1, r3, asl #2
	bl	ftl_memcpy
	sub	r2, r4, #3152
	ldrh	r3, [r2, #-8]
	cmp	r3, #0
	beq	.L2419
	ldrh	r1, [r8, #-12]
	ldrh	r2, [r2, #-12]
	ldr	r0, [r4, #-2904]
	mov	r3, r1, lsr #3
	add	r3, r3, r1, asl #1
	ldr	r1, [r4, #-2800]
	add	r3, r3, #52
	mov	r2, r2, asl #2
	ubfx	r3, r3, #2, #14
	add	r1, r1, r3, asl #2
	bl	ftl_memcpy
	b	.L2419
.L2417:
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L2413
.L2419:
	ldr	r2, [r5, #-716]
	ldr	r3, .L2439+4
	ldr	r4, .L2439
	cmp	r2, r3
	bne	.L2422
	sub	r2, r4, #3232
	sub	r6, r4, #716
	ldrb	r1, [r4, #-706]	@ zero_extendqisi2
	ldrh	r2, [r2, #-14]
	ldrh	r3, [r6, #8]
	cmp	r1, r2
	strh	r3, [r7, #6]	@ movhi
	bne	.L2422
	sub	r2, r4, #3200
	sub	r1, r4, #3184
	ldr	r7, .L2439+20
	ldrh	r2, [r2]
	ldrh	r1, [r1, #-10]
	sub	r8, r7, #372
	str	r3, [r7, #-376]
	mul	r2, r3, r2
	str	r2, [r4, #-2816]
	mul	r2, r1, r2
	ldr	r1, .L2439+12
	str	r2, [r1, #3908]
	sub	r2, r4, #2880
	ldrh	r0, [r2, #6]
	ldr	r2, [r4, #-3256]
	rsb	r0, r0, r2
	sub	r2, r4, #3264
	rsb	r0, r3, r0
	ldrh	r1, [r2, #-4]
	bl	__aeabi_uidiv
	ldrh	r3, [r6, #16]
	ldrh	ip, [r6, #14]
	sub	r2, r4, #648
	sub	lr, r7, #368
	mov	r1, r3, lsr #6
	strh	r1, [r2, #2]	@ movhi
	ldrh	r1, [r6, #18]
	and	r3, r3, #63
	strh	ip, [r2]	@ movhi
	sub	r2, r4, #600
	strb	r3, [r4, #-642]
	strh	r1, [r2]	@ movhi
	ldrh	r1, [r6, #20]
	ldrb	r3, [r4, #-705]	@ zero_extendqisi2
	strb	r3, [r4, #-640]
	mvn	r3, #0
	strh	r3, [lr]	@ movhi
	mov	r3, #0
	strb	r3, [r7, #-362]
	strb	r3, [r7, #-360]
	strh	r3, [lr, #2]	@ movhi
	str	r3, [r4, #-3140]
	strh	r0, [r8]	@ movhi
	mov	r0, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-594]
	ldrb	r1, [r4, #-704]	@ zero_extendqisi2
	mov	r8, lr
	strh	r0, [r2, #2]	@ movhi
	sub	r0, r4, #552
	strb	r1, [r4, #-592]
	mov	r7, r0
	ldrh	r1, [r6, #22]
	strh	r1, [r0]	@ movhi
	ldrh	r1, [r6, #24]
	mov	r6, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-546]
	ldrb	r1, [r4, #-703]	@ zero_extendqisi2
	strh	r6, [r0, #2]	@ movhi
	mov	r6, r2
	strb	r1, [r4, #-544]
	str	r3, [r4, #-3136]
	ldr	r1, [r4, #-684]
	str	r3, [r4, #-3120]
	str	r3, [r4, #-3124]
	str	r3, [r4, #-3112]
	str	r3, [r4, #-3104]
	str	r3, [r4, #-3128]
	ldr	r3, [r4, #-676]
	str	r1, [r4, #-3116]
	ldr	r1, [r4, #-3148]
	ldr	r2, [r5, #-3144]
	cmp	r3, r1
	strhi	r3, [r4, #-3148]
	ldr	r3, [r5, #-680]
	cmp	r3, r2
	ldrhi	r2, .L2439
	strhi	r3, [r2, #-3144]
	movw	r3, #65535
	cmp	ip, r3
	beq	.L2425
	ldr	r0, .L2439+24
	bl	make_superblock
.L2425:
	ldrh	r2, [r6]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2426
	ldr	r0, .L2439+28
	bl	make_superblock
.L2426:
	ldrh	r2, [r7]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2427
	ldr	r0, .L2439+32
	bl	make_superblock
.L2427:
	ldrh	r2, [r8]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2428
	ldr	r0, .L2439+36
	bl	make_superblock
.L2428:
	mov	r0, #0
.L2412:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2440:
	.align	2
.L2439:
	.word	.LANCHOR2
	.word	1179929683
	.word	.LANCHOR2-2808
	.word	.LANCHOR0
	.word	.LANCHOR2-3190
	.word	.LANCHOR4
	.word	.LANCHOR2-648
	.word	.LANCHOR2-600
	.word	.LANCHOR2-552
	.word	.LANCHOR4-368
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
	ubfx	r0, r0, #10, #16
	.pad #100
	sub	sp, sp, #100
	mov	r4, r1
	bl	P2V_block_in_plane
	ldr	r7, .L2455
	ldr	r1, .L2455+4
	sub	r5, r7, #3200
	ldrh	r9, [r5]
	mov	r6, r0
	ldr	r0, .L2455+8
	bl	printk
	ldr	r2, [r7, #-2940]
	mov	r3, r6, asl #1
	mov	r1, r6
	ldr	r0, .L2455+12
	ldrh	r2, [r2, r3]
	bl	printk
	add	r0, sp, #96
	strh	r6, [r0, #-48]!	@ movhi
	bl	make_superblock
	ldr	r3, .L2455+16
	ldrb	r2, [r3, #3904]	@ zero_extendqisi2
	clz	r3, r4
	cmp	r2, #0
	mov	r3, r3, lsr #5
	moveq	r3, #0
	cmp	r3, #0
	moveq	r4, r3
	beq	.L2442
	mov	r0, r6
	bl	ftl_get_blk_mode
	cmp	r0, #1
	mov	r4, r0
	subeq	r3, r7, #3184
	ldreqh	r9, [r3, #-14]
.L2442:
	ldr	r0, .L2455+20
	mov	r1, r4
	mov	r2, r9
	ldrh	r3, [r5]
	bl	printk
	ldr	r8, .L2455
	mov	r6, #0
.L2443:
	ldr	r3, .L2455+24
	mov	r2, #0
	add	r0, sp, #62
	mov	r5, r2
	movw	lr, #65535
	mov	r10, #36
	ldrh	r3, [r3]
	str	r3, [sp, #28]
	ldr	r3, [r8, #-3036]
	str	r3, [sp, #32]
	ldr	r3, [r8, #-2996]
	str	r3, [sp, #36]
	ldr	r3, .L2455+28
	ldrh	r3, [r3]
	str	r3, [sp, #40]
	ldr	r3, [r8, #-2968]
	str	r3, [sp, #44]
	ldr	r3, .L2455+32
	ldrh	ip, [r3]
.L2444:
	ldr	r1, [sp, #28]
	uxth	r3, r2
	cmp	r3, r1
	bcs	.L2453
	ldrh	r3, [r0, #2]!
	cmp	r3, lr
	beq	.L2445
	ldr	r1, [sp, #32]
	orr	r3, r6, r3, asl #10
	mla	r1, r10, r5, r1
	str	r3, [r1, #4]
	ldr	r3, [sp, #40]
	mul	r3, r3, r5
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	ldr	fp, [sp, #36]
	bic	r3, r3, #3
	add	r3, fp, r3
	str	r3, [r1, #8]
	mul	r3, ip, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	ldr	fp, [sp, #44]
	bic	r3, r3, #3
	add	r3, fp, r3
	str	r3, [r1, #12]
.L2445:
	add	r2, r2, #1
	b	.L2444
.L2453:
	ldr	r0, [r8, #-3036]
	mov	r1, r5
	mov	r2, r4
	mov	r10, #0
	bl	FlashReadPages
.L2447:
	uxth	r3, r10
	cmp	r3, r5
	bcs	.L2454
	mov	r3, #36
	ldr	lr, [r7, #-3036]
	mul	r2, r3, r10
	ldrh	r1, [sp, #48]
	add	r10, r10, #1
	add	ip, lr, r2
	ldr	r3, [ip, #12]
	ldr	r0, [ip, #8]
	ldr	fp, [r3]
	str	fp, [sp]
	ldr	fp, [r3, #4]
	str	fp, [sp, #4]
	ldr	fp, [r3, #8]
	str	fp, [sp, #8]
	ldr	r3, [r3, #12]
	str	r3, [sp, #12]
	ldr	r3, [r0]
	str	r3, [sp, #16]
	ldr	r3, [r0, #4]
	ldr	r0, .L2455+36
	str	r3, [sp, #20]
	ldr	r2, [lr, r2]
	ldr	r3, [ip, #4]
	bl	printk
	b	.L2447
.L2454:
	add	r6, r6, #1
	uxth	r6, r6
	cmp	r6, r9
	bne	.L2443
.L2449:
	mov	r0, #0
	add	sp, sp, #100
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2456:
	.align	2
.L2455:
	.word	.LANCHOR2
	.word	.LANCHOR3+28
	.word	.LC48
	.word	.LC61
	.word	.LANCHOR0
	.word	.LC62
	.word	.LANCHOR2-3268
	.word	.LANCHOR2-3190
	.word	.LANCHOR2-3188
	.word	.LC57
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
	ldr	r0, .L2472
	mov	r7, #0
	ldr	r1, .L2472+4
	bl	printk
	ldr	r5, .L2472+8
	mov	r6, r5
.L2458:
	ldr	r3, .L2472+12
	uxth	r4, r7
	ldrh	r3, [r3]
	cmp	r3, r4
	bls	.L2468
	add	r8, sp, #88
	mov	r0, r4
	movw	r9, #65535
	mov	r10, #36
	strh	r4, [r8, #-48]!	@ movhi
	bl	ftl_get_blk_mode
	ldr	ip, [r5, #-2940]
	mov	r2, r4, asl #1
	mov	r1, r4
	ldrh	r2, [ip, r2]
	mov	r3, r0
	ldr	r0, .L2472+16
	bl	printk
	mov	r0, r8
	bl	make_superblock
	ldr	r3, .L2472+20
	ldr	lr, [r5, #-2968]
	ldrh	r2, [r3]
	ldrh	ip, [r3, #78]
	ldrh	r8, [r3, #80]
	str	r2, [sp, #28]
	add	r0, sp, #54
	ldr	r2, [r5, #-3036]
	str	r2, [sp, #32]
	ldr	r2, [r5, #-2996]
	str	r2, [sp, #36]
	mov	r2, #0
	mov	r4, r2
.L2459:
	ldr	r1, [sp, #28]
	uxth	r3, r2
	cmp	r3, r1
	bcs	.L2469
	ldrh	r3, [r0, #2]!
	cmp	r3, r9
	beq	.L2460
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
.L2460:
	add	r2, r2, #1
	b	.L2459
.L2469:
	ldr	r0, [r6, #-3036]
	mov	r1, r4
	mov	r2, #0
	mov	r8, #0
	bl	FlashReadPages
	mov	r9, #36
.L2462:
	uxth	r3, r8
	cmp	r3, r4
	bcs	.L2470
	mul	r2, r9, r8
	ldr	lr, [r6, #-3036]
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
	ldr	r0, .L2472+24
	str	r3, [sp, #20]
	ldr	r2, [lr, r2]
	ldr	r3, [ip, #4]
	bl	printk
	b	.L2462
.L2470:
	ldr	r0, [r6, #-3036]
	mov	r1, r4
	mov	r2, #1
	mov	r8, #0
	bl	FlashReadPages
	mov	r9, #36
.L2464:
	uxth	r3, r8
	cmp	r3, r4
	bcs	.L2471
	mul	r2, r9, r8
	ldr	lr, [r6, #-3036]
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
	ldr	r0, .L2472+28
	str	r3, [sp, #20]
	ldr	r2, [lr, r2]
	ldr	r3, [ip, #4]
	bl	printk
	b	.L2464
.L2471:
	add	r7, r7, #1
	b	.L2458
.L2468:
	mov	r0, #0
	add	sp, sp, #92
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2473:
	.align	2
.L2472:
	.word	.LC48
	.word	.LANCHOR3+48
	.word	.LANCHOR2
	.word	.LANCHOR2-3258
	.word	.LC63
	.word	.LANCHOR2-3268
	.word	.LC64
	.word	.LC65
	.fnend
	.size	FtlScanAllBlock, .-FtlScanAllBlock
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r2, #6
	ldr	r6, .L2486
	mov	r1, #0
	.pad #28
	sub	sp, sp, #28
	sub	r3, r6, #3248
	ldr	r0, [r6, #-2888]
	mov	r5, r6
	ldrh	r3, [r3, #-10]
	mov	fp, r6
	mul	r2, r2, r3
	bl	ftl_memset
	mov	r3, #0
	sub	r1, r6, #660
	sub	r2, r6, #652
	sub	r0, r6, #3152
	mov	r4, r3
	mov	r8, r3
	mov	r9, r3
	str	r3, [r6, #-656]
	str	r3, [r6, #-668]
	str	r3, [r6, #-664]
	strh	r3, [r1]	@ movhi
	strh	r3, [r2]	@ movhi
	strh	r3, [r0, #-6]	@ movhi
	str	r1, [sp, #8]
	str	r2, [sp, #12]
.L2475:
	ldr	r3, .L2486+4
	uxth	r7, r4
	sxth	r10, r7
	ldrh	r2, [r3]
	cmp	r10, r2
	bge	.L2482
	sub	r3, r3, #8
	ldr	r2, .L2486+8
	mov	ip, r7
	ldrh	r3, [r3]
	str	r3, [sp]
	ldrh	r3, [r2]
	mov	r2, #0
	mov	r6, r2
	str	r3, [sp, #4]
.L2483:
	ldr	r3, [sp]
	sxth	r1, r2
	cmp	r1, r3
	bge	.L2485
	add	r1, r5, r1
	str	r2, [sp, #20]
	str	ip, [sp, #16]
	ldrb	r0, [r1, #-3240]	@ zero_extendqisi2
	mov	r1, ip
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldr	r2, [sp, #20]
	ldr	ip, [sp, #16]
	ldreq	r3, [sp, #4]
	add	r2, r2, #1
	addeq	r6, r3, r6
	uxtheq	r6, r6
	b	.L2483
.L2485:
	cmp	r6, #0
	beq	.L2478
	sxth	r1, r6
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r6, r0
	b	.L2479
.L2478:
	sxth	r7, r7
	ldr	r2, [r5, #-2940]
	mvn	r1, #0
	mov	r7, r7, asl #1
	strh	r1, [r2, r7]	@ movhi
.L2479:
	mov	r1, r10, asl #1
	ldr	r2, [r5, #-2888]
	add	r0, r1, r10
	add	r2, r2, r0, asl #1
	strh	r6, [r2, #4]	@ movhi
	ldr	r2, .L2486+12
	ldrh	r0, [r2]
	cmp	r10, r0
	beq	.L2480
	ldrh	r0, [r2, #48]
	cmp	r10, r0
	beq	.L2480
	ldrh	r2, [r2, #96]
	cmp	r10, r2
	beq	.L2480
	ldr	r3, [fp, #-2940]
	uxth	r0, r4
	ldrh	r3, [r3, r1]
	cmp	r3, #0
	bne	.L2481
	add	r8, r8, #1
	uxth	r8, r8
	bl	INSERT_FREE_LIST
	b	.L2480
.L2481:
	add	r9, r9, #1
	uxth	r9, r9
	bl	INSERT_DATA_LIST
.L2480:
	add	r4, r4, #1
	b	.L2475
.L2482:
	ldr	r3, [sp, #8]
	mov	r0, #0
	strh	r9, [r3]	@ movhi
	ldr	r3, [sp, #12]
	strh	r8, [r3]	@ movhi
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2487:
	.align	2
.L2486:
	.word	.LANCHOR2
	.word	.LANCHOR2-3260
	.word	.LANCHOR2-3200
	.word	.LANCHOR2-648
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	Ftl_save_ext_data
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L2490
	ldr	r3, .L2490+4
	ldr	r1, [r2, #-320]
	cmp	r1, r3
	bxne	lr
	ldr	r3, .L2490+8
	mov	r0, #0
	sub	r2, r2, #320
	str	r3, [r2, #4]
	ldr	r3, [r2, #512]
	str	r3, [r2, #88]
	ldr	r3, [r2, #516]
	str	r3, [r2, #92]
	ldr	r3, .L2490+12
	ldr	r1, [r3, #-3140]
	str	r1, [r2, #8]
	ldr	r1, [r3, #-3136]
	str	r1, [r2, #12]
	ldr	r1, [r3, #-3120]
	str	r1, [r2, #16]
	ldr	r1, [r3, #-3124]
	str	r1, [r2, #20]
	ldr	r1, [r3, #-3112]
	str	r1, [r2, #28]
	ldr	r1, [r3, #-3108]
	str	r1, [r2, #32]
	ldr	r1, [r3, #-3132]
	str	r1, [r2, #36]
	ldr	r1, [r3, #-3128]
	str	r1, [r2, #40]
	ldr	r1, [r3, #-3104]
	str	r1, [r2, #44]
	ldr	r1, [r3, #-3100]
	str	r1, [r2, #48]
	ldr	r1, [r3, #-3152]
	ldr	r3, [r3, #-3156]
	str	r1, [r2, #60]
	mov	r1, #1
	str	r3, [r2, #64]
	b	FtlVendorPartWrite
.L2491:
	.align	2
.L2490:
	.word	.LANCHOR4
	.word	1179929683
	.word	1342177366
	.word	.LANCHOR2
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
	ldr	r3, .L2500
	ldr	r2, [r3, #-3336]
	cmp	r2, #0
	moveq	r2, #32
	beq	.L2493
	ldr	r2, [r3, #-3100]
	cmp	r2, #39
	movhi	r2, #32
	movls	r2, #4
.L2493:
	ldr	ip, .L2500+4
	ldrh	r1, [ip, #200]
	cmp	r1, #31
	addls	r1, r1, #1
	strlsh	r1, [ip, #200]	@ movhi
	movls	r2, #1
	cmp	r0, #0
	bne	.L2495
	ldr	r1, [r3, #-2952]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r2, r2, r0
	cmp	r1, r2
	bcc	.L2496
.L2495:
	ldr	r2, [r3, #-2952]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L2500+8
	str	r1, [r2]
	ldr	r2, [r3, #-2952]
	ldr	r3, .L2500+12
	ldrh	r1, [r3]
	mov	r3, r1, asl #9
	str	r3, [r2, #12]
	ldr	r3, [r2, #8]
	add	r3, r3, #1
	str	r3, [r2, #8]
	mov	r3, #0
	str	r3, [r2, #4]
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L2496:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2501:
	.align	2
.L2500:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	1112818501
	.word	.LANCHOR2-2960
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
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
	ldr	r5, .L2507
	mov	r1, #1
	ldr	r4, .L2507+4
	sub	r2, r5, #320
	bl	FtlVendorPartRead
	ldr	r3, [r5, #-320]
	cmp	r3, r4
	beq	.L2503
	sub	r0, r5, #320
	mov	r1, #0
	mov	r2, #512
	bl	ftl_memset
	str	r4, [r5, #-320]
.L2503:
	ldr	r2, [r5, #-320]
	ldr	r3, .L2507
	cmp	r2, r4
	ldr	r4, .L2507+8
	bne	.L2504
	ldr	r2, [r3, #-232]
	str	r2, [r3, #192]
	ldr	r2, [r3, #-228]
	str	r2, [r3, #196]
	ldr	r2, [r3, #-312]
	str	r2, [r4, #-3140]
	ldr	r2, [r3, #-308]
	str	r2, [r4, #-3136]
	ldr	r2, [r3, #-304]
	str	r2, [r4, #-3120]
	ldr	r2, [r3, #-300]
	str	r2, [r4, #-3124]
	ldr	r2, [r3, #-292]
	str	r2, [r4, #-3112]
	ldr	r2, [r3, #-288]
	str	r2, [r4, #-3108]
	ldr	r2, [r3, #-284]
	str	r2, [r4, #-3132]
	ldr	r2, [r3, #-280]
	str	r2, [r4, #-3128]
	ldr	r2, [r3, #-276]
	str	r2, [r4, #-3104]
	ldr	r2, [r3, #-272]
	ldr	r3, [r3, #-260]
	str	r2, [r4, #-3100]
	str	r3, [r4, #-3152]
.L2504:
	mov	r3, #0
	ldr	r2, [r5, #-252]
	str	r3, [r4, #-3156]
	ldr	r3, .L2507+12
	cmp	r2, r3
	bne	.L2505
	ldr	r3, .L2507+8
	mov	r2, #1
	ldr	r0, .L2507+16
	ldr	r1, .L2507+20
	str	r2, [r3, #-3336]
	bl	printk
.L2505:
	ldr	r3, .L2507+24
	ldr	r2, [r4, #-3112]
	ldr	r0, [r4, #-3116]
	ldrh	r1, [r3, #-8]
	mla	r0, r0, r1, r2
	ldrh	r1, [r3, #-60]
	bl	__aeabi_uidiv
	str	r0, [r5, #204]
	ldmfd	sp!, {r3, r4, r5, pc}
.L2508:
	.align	2
.L2507:
	.word	.LANCHOR4
	.word	1179929683
	.word	.LANCHOR2
	.word	305432421
	.word	.LC46
	.word	.LC66
	.word	.LANCHOR2-3200
	.fnend
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	2
	.global	ftl_scan_all_ppa
	.type	ftl_scan_all_ppa, %function
ftl_scan_all_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r5, .L2516
	mov	r7, #0
	ldr	r0, .L2516+4
	sub	r9, r5, #2
	add	r4, r5, #3200
	ldrh	r1, [r5, #-2]
	ldrh	r2, [r5]
	bl	printk
.L2510:
	ldrh	r3, [r9]
	cmp	r7, r3
	bcs	.L2512
	mov	r8, r7, asl #10
	mov	r6, #0
.L2513:
	ldrh	r3, [r5]
	cmp	r6, r3
	bcs	.L2515
	add	r3, r6, r8
	str	r3, [r4, #-2804]
	ldr	r3, [r4, #-3008]
	mov	r2, #0
	mov	r1, #1
	ldr	r0, .L2516+8
	str	r2, [r4, #-2808]
	add	r6, r6, #1
	str	r3, [r4, #-2800]
	ldr	r3, [r4, #-2976]
	str	r3, [r4, #-2796]
	bl	FlashReadPages
	ldr	r3, [r4, #-2796]
	ldr	r2, [r4, #-2800]
	ldr	r0, .L2516+12
	ldr	r1, [r3, #4]
	str	r1, [sp]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2]
	str	r1, [sp, #12]
	ldr	r2, [r2, #4]
	str	r2, [sp, #16]
	ldr	r1, [r4, #-2804]
	ldr	r2, [r4, #-2808]
	ldr	r3, [r3]
	bl	printk
	b	.L2513
.L2515:
	add	r7, r7, #1
	b	.L2510
.L2512:
	ldr	r0, .L2516+16
	ldr	r1, .L2516+20
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L2517:
	.align	2
.L2516:
	.word	.LANCHOR2-3200
	.word	.LC67
	.word	.LANCHOR2-2808
	.word	.LC68
	.word	.LC69
	.word	.LANCHOR3+64
	.fnend
	.size	ftl_scan_all_ppa, .-ftl_scan_all_ppa
	.align	2
	.global	update_multiplier_value
	.type	update_multiplier_value, %function
update_multiplier_value:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, #0
	ldr	r3, .L2528
	mov	r7, r0
	mov	r4, r5
	add	r6, r3, #24
	ldrh	r8, [r3, #-4]
	ldrh	r9, [r3, #64]
.L2519:
	uxth	r3, r5
	cmp	r3, r8
	bcs	.L2527
	ldrb	r0, [r6, r5]	@ zero_extendqisi2
	mov	r1, r7
	bl	V2P_block
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addeq	r4, r4, r9
	uxtheq	r4, r4
	b	.L2519
.L2527:
	cmp	r4, #0
	beq	.L2522
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L2522:
	ldr	r3, .L2528+4
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #-2888]
	mla	r7, r2, r7, r3
	strh	r4, [r7, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L2529:
	.align	2
.L2528:
	.word	.LANCHOR2-3264
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
	ldr	r2, .L2533
	ldr	r0, [r2, #-656]
	cmp	r0, #0
	bxeq	lr
	ldr	r3, [r2, #-2888]
	rsb	r0, r3, r0
	ldr	r3, .L2533+4
	mov	r0, r0, asr #1
	mul	r0, r3, r0
	ldr	r3, [r2, #-2948]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r0, [r3, r0]
	bx	lr
.L2534:
	.align	2
.L2533:
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
	ldr	r1, .L2545
	ldr	r3, [r1, #-656]
	cmp	r3, #0
	beq	.L2541
	sub	r2, r1, #652
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	mov	r4, #6
	ldrh	r2, [r2]
	movw	r5, #65535
	ldr	ip, [r1, #-2888]
	rsb	r2, r2, r2, asl #3
	rsb	r3, ip, r3
	mov	r2, r2, asr #3
	mov	r3, r3, asr #1
	cmp	r0, r2
	uxthgt	r0, r2
	ldr	r2, .L2545+4
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
.L2538:
	uxth	lr, r2
	cmp	lr, r0
	bcs	.L2540
	mul	lr, r4, r3
	add	r2, r2, #1
	ldrh	lr, [ip, lr]
	cmp	lr, r5
	bne	.L2542
.L2540:
	ldr	r2, [r1, #-2948]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L2542:
	mov	r3, lr
	b	.L2538
.L2541:
	mov	r0, r3
	bx	lr
.L2546:
	.align	2
.L2545:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r7, r0
	ldr	r10, .L2558
	add	r5, r7, #12
	ldr	r1, .L2558+4
	.pad #36
	sub	sp, sp, #36
	bl	strcpy
	mov	r0, r5
	ldr	r1, .L2558+8
	ldr	r2, [r10, #3876]
	bl	sprintf
	ldr	r6, .L2558+12
	ldr	r1, .L2558+16
	ldr	r2, [r6, #-3184]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L2558+20
	ldr	r3, [r3, #3284]
	cmp	r3, #1
	add	r4, r5, r0
	bne	.L2553
	add	r0, sp, #16
	add	r1, sp, #20
	add	r2, sp, #24
	add	r3, sp, #28
	bl	NandcGetTimeCfg
	mov	r0, r4
	ldr	r1, .L2558+24
	sub	r8, r6, #2880
	sub	r9, r6, #652
	sub	fp, r6, #716
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #16]
	str	r3, [sp]
	ldr	r3, [sp, #28]
	str	r3, [sp, #4]
	ldr	r3, [sp, #20]
	bl	sprintf
	ldr	r1, .L2558+28
	add	r4, r4, r0
	add	r5, r4, #10
	mov	r0, r4
	ldr	r4, .L2558+32
	bl	strcpy
	mov	r0, r5
	ldr	r1, .L2558+36
	ldr	r2, [r6, #-2816]
	bl	sprintf
	ldr	r1, .L2558+40
	ldr	r2, [r6, #-408]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+44
	ldr	r2, [r6, #-3120]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+48
	ldr	r2, [r6, #-3132]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+52
	ldr	r2, [r6, #-3136]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+56
	ldr	r2, [r6, #-3128]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+60
	ldr	r2, [r6, #-3124]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+64
	ldr	r2, [r6, #-3140]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #192]
	ldr	r1, .L2558+68
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #196]
	ldr	r1, .L2558+72
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+76
	ldr	r2, [r6, #-3148]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+80
	ldr	r2, [r6, #-3144]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	FtlBbtCalcTotleCnt
	ldrh	r2, [r8, #6]
	ldr	r1, .L2558+84
	mov	r3, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+88
	ldrh	r2, [r9]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+92
	ldr	r2, [r6, #-3116]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+96
	ldr	r2, [r6, #-3112]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #204]
	ldr	r1, .L2558+100
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+104
	ldr	r2, [r6, #-3108]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+108
	ldr	r2, [r6, #-3104]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+112
	ldr	r2, [r6, #-3100]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+116
	ldrh	r2, [fp, #30]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+120
	ldrh	r2, [fp, #28]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r10, #3908]
	ldr	r1, .L2558+124
	sub	r10, r6, #3056
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+128
	ldr	r2, [r6, #-3168]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+132
	ldr	r2, [r6, #-3272]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8, #114]
	ldr	r1, .L2558+136
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	sub	r3, r6, #3248
	ldr	r1, .L2558+140
	ldrh	r2, [r3, #-12]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	sub	r3, r4, #372
	ldr	r1, .L2558+144
	ldrh	r2, [r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+148
	ldr	r2, [r6, #-3256]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	sub	r3, r6, #404
	ldr	r1, .L2558+152
	ldrh	r2, [r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8]
	ldr	r1, .L2558+156
	sub	r8, r6, #648
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8, #2]
	ldr	r1, .L2558+160
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+164
	ldrb	r2, [r6, #-642]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8]
	ldr	r1, .L2558+168
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+172
	ldrb	r2, [r6, #-640]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8, #4]
	ldr	r1, .L2558+176
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r8]
	ldr	r2, [r6, #-2940]
	sub	r8, r6, #600
	ldr	r1, .L2558+180
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8, #2]
	ldr	r1, .L2558+184
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+188
	ldrb	r2, [r6, #-594]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8]
	ldr	r1, .L2558+192
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+196
	ldrb	r2, [r6, #-592]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8, #4]
	ldr	r1, .L2558+200
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r8]
	ldr	r2, [r6, #-2940]
	sub	r8, r6, #552
	ldr	r1, .L2558+204
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8, #2]
	ldr	r1, .L2558+208
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+212
	ldrb	r2, [r6, #-546]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8]
	ldr	r1, .L2558+216
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+220
	ldrb	r2, [r6, #-544]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r8, #4]
	ldr	r1, .L2558+224
	sub	r8, r4, #368
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+228
	ldrh	r2, [r8, #2]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrb	r2, [r4, #-362]	@ zero_extendqisi2
	ldr	r1, .L2558+232
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+236
	ldrh	r2, [r8]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrb	r2, [r4, #-360]	@ zero_extendqisi2
	ldr	r1, .L2558+240
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+244
	ldrh	r2, [r8, #4]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, [r4, #-236]
	ldr	r2, [r6, #-3152]
	ldr	r3, [r6, #-3336]
	orr	r2, r3, r2, asl #8
	str	r1, [sp]
	add	r5, r5, r0
	ldr	r1, [r4, #-244]
	mov	r0, r5
	str	r1, [sp, #4]
	ldr	r1, .L2558+248
	ldr	r3, [r4, #-240]
	bl	sprintf
	ldr	r2, [r4, #-248]
	ldr	r1, .L2558+252
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #-224]
	ldr	r1, .L2558+256
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+260
	ldrh	r2, [r10, #-8]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L2558+264
	ldrh	r2, [r10, #-6]
	add	r4, r5, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L2558+268
	ldr	r2, [r6, #-3080]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L2558+272
	ldrh	r2, [r10, #-4]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L2558+276
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	ldrh	r0, [r9]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L2558+280
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r8]
	movw	r2, #65535
	cmp	r3, r2
	add	r4, r4, r0
	beq	.L2550
	ldr	r2, [r6, #-2940]
	mov	r3, r3, asl #1
	mov	r0, r4
	ldr	r1, .L2558+284
	ldrh	r2, [r2, r3]
	bl	sprintf
	add	r4, r4, r0
.L2550:
	mov	r0, #0
	mov	r5, #0
	bl	List_get_gc_head_node
	movw	r10, #65535
	mov	r9, #6
	uxth	r3, r0
.L2552:
	cmp	r3, r10
	beq	.L2551
	ldr	r2, [r6, #-2940]
	mov	r1, r3, asl #1
	mul	r8, r9, r3
	mov	r0, r4
	ldrh	r2, [r2, r1]
	str	r2, [sp]
	ldr	r2, [r6, #-2888]
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r2, [sp, #4]
	ldr	r2, [r6, #-2948]
	ldrh	r2, [r2, r1]
	ldr	r1, .L2558+288
	str	r2, [sp, #8]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-2888]
	cmp	r5, #16
	ldrh	r3, [r3, r8]
	add	r4, r4, r0
	bne	.L2552
.L2551:
	ldr	r2, [r6, #-2888]
	mov	r5, #0
	ldr	r3, [r6, #-656]
	movw	r10, #65535
	mov	r9, #6
	rsb	r3, r2, r3
	ldr	r2, .L2558+292
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L2554:
	cmp	r3, r10
	beq	.L2553
	mul	r8, r9, r3
	ldr	r2, [r6, #-2888]
	ldr	r1, [r6, #-2948]
	mov	r0, r4
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r2, [sp]
	mov	r2, r3, asl #1
	ldrh	r2, [r1, r2]
	ldr	r1, .L2558+296
	str	r2, [sp, #4]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-2888]
	cmp	r5, #4
	ldrh	r3, [r3, r8]
	add	r4, r4, r0
	bne	.L2554
.L2553:
	rsb	r0, r7, r4
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2559:
	.align	2
.L2558:
	.word	.LANCHOR0
	.word	.LC70
	.word	.LC71
	.word	.LANCHOR2
	.word	.LC72
	.word	.LANCHOR1
	.word	.LC73
	.word	.LC74
	.word	.LANCHOR4
	.word	.LC75
	.word	.LC76
	.word	.LC77
	.word	.LC78
	.word	.LC79
	.word	.LC80
	.word	.LC81
	.word	.LC82
	.word	.LC83
	.word	.LC84
	.word	.LC85
	.word	.LC86
	.word	.LC87
	.word	.LC88
	.word	.LC89
	.word	.LC90
	.word	.LC91
	.word	.LC92
	.word	.LC93
	.word	.LC94
	.word	.LC95
	.word	.LC96
	.word	.LC97
	.word	.LC98
	.word	.LC99
	.word	.LC100
	.word	.LC101
	.word	.LC102
	.word	.LC103
	.word	.LC104
	.word	.LC105
	.word	.LC106
	.word	.LC107
	.word	.LC108
	.word	.LC109
	.word	.LC110
	.word	.LC111
	.word	.LC112
	.word	.LC113
	.word	.LC114
	.word	.LC115
	.word	.LC116
	.word	.LC117
	.word	.LC118
	.word	.LC119
	.word	.LC120
	.word	.LC121
	.word	.LC122
	.word	.LC123
	.word	.LC124
	.word	.LC125
	.word	.LC126
	.word	.LC127
	.word	.LC128
	.word	.LC129
	.word	.LC130
	.word	.LC131
	.word	.LC132
	.word	.LC133
	.word	.LC134
	.word	.LC135
	.word	.LC136
	.word	.LC137
	.word	.LC138
	.word	-1431655765
	.word	.LC139
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
	ldr	r1, .L2562
	ldr	r2, .L2562+4
	bl	sprintf
	add	r4, r5, r0
	mov	r0, r4
	bl	FtlPrintInfo2buf
	add	r0, r4, r0
	rsb	r0, r5, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L2563:
	.align	2
.L2562:
	.word	.LC46
	.word	.LC140
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
	ldr	r5, .L2594
	ldr	r4, .L2594+4
	ldr	r6, [r5, #204]
	ldr	r3, [r4, #-3100]
	cmp	r6, r3
	bcs	.L2565
	sub	r3, r4, #3248
	mov	r6, #0
	str	r6, [r4, #-3116]
	ldrh	r1, [r3, #-12]
	ldr	r3, [r4, #-2948]
	sub	r3, r3, #2
.L2566:
	cmp	r6, r1
	bcs	.L2593
	ldrh	r2, [r3, #2]!
	add	r6, r6, #1
	ldr	r0, [r4, #-3116]
	add	r2, r2, r0
	str	r2, [r4, #-3116]
	b	.L2566
.L2593:
	ldr	r6, [r4, #-3116]
	mov	r0, r6
	bl	__aeabi_uidiv
	ldr	r3, .L2594+8
	ldrh	r1, [r3, #-8]
	str	r0, [r5, #204]
	ldr	r0, [r4, #-3112]
	rsb	r0, r0, r6
	bl	__aeabi_uidiv
	str	r0, [r4, #-3116]
	b	.L2568
.L2565:
	ldr	r3, [r4, #-3104]
	cmp	r6, r3
	bls	.L2568
	ldr	ip, .L2594+12
	add	r3, r3, #1
	mov	lr, r4
	str	r3, [r4, #-3104]
	mov	r3, #0
.L2570:
	ldrh	r2, [ip]
	cmp	r3, r2
	bcs	.L2568
	ldr	r0, [lr, #-2948]
	mov	r1, r3, asl #1
	add	r3, r3, #1
	ldrh	r2, [r0, r1]
	add	r2, r2, #1
	strh	r2, [r0, r1]	@ movhi
	b	.L2570
.L2568:
	ldr	r6, [r4, #-3100]
	ldr	r8, [r5, #204]
	add	r3, r6, #256
	ldr	r2, .L2594+4
	cmp	r3, r8
	bls	.L2573
	ldr	r1, [r2, #-3104]
	add	r3, r6, #768
	cmp	r3, r1
	bls	.L2573
	cmp	r6, #40
	ldr	r2, [r2, #-3336]
	movls	r3, #0
	movhi	r3, #1
	cmp	r2, #0
	orreq	r3, r3, #1
	cmp	r3, #0
	beq	.L2573
.L2575:
	movw	r0, #65535
	b	.L2574
.L2573:
	ldr	r10, .L2594+4
	sub	r3, r10, #652
	ldrh	r0, [r3]
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
	bne	.L2575
	ldr	r3, [r10, #-668]
	cmp	r3, #0
	beq	.L2575
	sub	r2, r10, #3248
	ldr	r0, [r10, #-2888]
	ldr	ip, .L2594+16
	movw	r7, #65535
	ldrh	r2, [r2, #-12]
	mov	r5, r7
	ldr	r10, [r10, #-2948]
	mov	lr, #6
	str	r2, [sp, #20]
.L2576:
	ldrh	r2, [r3]
	movw	fp, #65535
	cmp	r2, fp
	beq	.L2578
	add	r1, r1, #1
	ldr	fp, [sp, #20]
	uxth	r1, r1
	cmp	r1, fp
	bhi	.L2575
	ldrh	fp, [r3, #4]
	cmp	fp, #0
	beq	.L2577
	rsb	r3, r0, r3
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	fp, r3, asl #1
	ldrh	fp, [r10, fp]
	cmp	fp, r6
	bls	.L2582
	cmp	fp, r7
	movcc	r7, fp
	movcc	r5, r3
.L2577:
	mla	r3, lr, r2, r0
	b	.L2576
.L2582:
	mov	r5, r3
.L2578:
	movw	r3, #65535
	cmp	r5, r3
	beq	.L2575
	mov	r3, r5, asl #1
	ldrh	fp, [r10, r3]
	cmp	fp, r6
	bls	.L2580
	str	r3, [sp, #20]
	bl	GetFreeBlockMinEraseCount
	ldr	r3, [sp, #20]
	cmp	r0, r6
	strhi	r7, [r4, #-3100]
.L2580:
	cmp	fp, r8
	bcs	.L2575
	add	r2, fp, #128
	cmp	r9, r2
	ble	.L2575
	add	r2, fp, #256
	cmp	r2, r8
	bcc	.L2581
	ldr	r2, [r4, #-3104]
	add	fp, fp, #768
	cmp	fp, r2
	bcs	.L2575
.L2581:
	ldr	r2, [r4, #-2940]
	mov	r1, r5
	ldr	r0, .L2594+20
	ldrh	r2, [r2, r3]
	str	r2, [sp]
	mov	r2, r8
	ldrh	r3, [r10, r3]
	stmib	sp, {r3, r9}
	ldr	r3, [r4, #-3104]
	bl	printk
	mov	r0, r5
	mov	r3, #1
	str	r3, [r4, #-3092]
.L2574:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2595:
	.align	2
.L2594:
	.word	.LANCHOR4
	.word	.LANCHOR2
	.word	.LANCHOR2-3200
	.word	.LANCHOR2-3260
	.word	-1431655765
	.word	.LC141
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
	beq	.L2597
	ldr	r2, .L2599
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #-2940]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L2597:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2600:
	.align	2
.L2599:
	.word	.LANCHOR2
	.fnend
	.size	free_data_superblock, .-free_data_superblock
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
	ldr	r4, .L2656
	ldr	r3, [r4, #-2812]
	cmp	r3, #0
	bne	.L2602
	mov	r5, r0
	mov	r7, r4
.L2603:
	ldr	r3, .L2656+4
	ldrb	r2, [r5, #8]	@ zero_extendqisi2
	cmp	r5, r3
	sub	r6, r3, #100
	bne	.L2604
	ldrh	r3, [r6]
	ldr	ip, [r7, #-3092]
	mov	r0, r3, lsr #1
	mul	lr, ip, r3
	add	r1, r0, #1
	add	r1, r1, lr, lsr #2
	ldr	lr, [r7, #-3336]
	cmp	lr, #0
	uxth	r1, r1
	beq	.L2605
	ldr	lr, [r7, #-3100]
	cmp	lr, #39
	bhi	.L2605
	cmp	lr, #2
	bls	.L2630
	cmp	ip, #0
	movne	r3, #0
	andeq	r3, r3, #1
	cmp	r3, #0
	moveq	r1, r0
	beq	.L2605
	b	.L2630
.L2604:
	cmp	r2, #1
	bne	.L2630
	ldr	r3, .L2656+8
	ldrh	r3, [r3]
	cmp	r3, #1
	beq	.L2630
	ldr	r3, .L2656+12
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2630
	ldr	r0, [r7, #-3336]
	ldrh	r3, [r6]
	cmp	r0, #0
	mov	r1, r3, lsr #3
	beq	.L2605
	ldr	r0, [r7, #-3100]
	cmp	r0, #1
	rsbls	r3, r3, r3, asl #3
	ubfxls	r1, r3, #3, #16
.L2605:
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
	b	.L2606
.L2630:
	mov	r1, #0
.L2606:
	ldr	r0, .L2656+16
	bl	List_pop_index_node
	ldr	r10, .L2656+20
	ldrh	r3, [r6]
	sub	r3, r3, #1
	strh	r3, [r6]	@ movhi
	ldrh	r3, [r10]
	uxth	r9, r0
	cmp	r3, r9
	bls	.L2603
	ldr	r3, [r7, #-2940]
	mov	r8, r9, asl #1
	ldrh	r6, [r3, r8]
	cmp	r6, #0
	bne	.L2603
	strh	r9, [r5]	@ movhi
	mov	r0, r5
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2608
	ldr	r0, [r7, #-3024]
	mov	r2, #36
	ldrh	lr, [r10, #-8]
	add	ip, r5, #14
	mov	r3, r0
	mla	r1, r2, lr, r0
	mov	lr, r6
	str	r1, [sp, #4]
	b	.L2609
.L2608:
	ldr	r3, [r7, #-2940]
	b	.L2651
.L2609:
	ldr	r1, [sp, #4]
	cmp	r3, r1
	beq	.L2653
	str	lr, [r3, #8]
	movw	fp, #65535
	str	lr, [r3, #12]
	add	r3, r3, #36
	ldrh	r1, [ip, #2]!
	cmp	r1, fp
	movne	r1, r1, asl #10
	mlane	fp, r2, r6, r0
	addne	r6, r6, #1
	uxthne	r6, r6
	strne	r1, [fp, #4]
	b	.L2609
.L2653:
	ldr	r3, .L2656+24
	ldr	r2, [r4, #-3336]
	rsb	r3, r3, r5
	clz	r3, r3
	cmp	r2, #0
	mov	r3, r3, lsr #5
	moveq	r3, #0
	cmp	r3, #0
	beq	.L2612
	ldr	r3, [r7, #-2948]
	ldrh	r3, [r3, r8]
	cmp	r3, #40
	movhi	r3, #0
	strhib	r3, [r7, #-640]
.L2612:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r2, [r4, #-2948]
	cmp	r3, #0
	ldrh	r3, [r2, r8]
	bne	.L2613
	cmp	r3, #0
	mov	r0, r9
	ldrne	r1, .L2656+28
	moveq	r3, #2
	ldrneh	r1, [r1]
	addne	r3, r3, r1
	mov	r1, #0
	uxthne	r3, r3
	strh	r3, [r2, r8]	@ movhi
	ldr	r3, [r4, #-3116]
	add	r3, r3, #1
	str	r3, [r4, #-3116]
	bl	ftl_set_blk_mode
	b	.L2615
.L2613:
	add	r3, r3, #1
	strh	r3, [r2, r8]	@ movhi
	ldr	r1, [r4, #-2928]
	mov	r0, r9, lsr #5
	ldr	r3, [r4, #-3112]
	mov	ip, #1
	add	r3, r3, #1
	str	r3, [r4, #-3112]
	ldr	r2, [r1, r0, asl #2]
	and	r3, r9, #31
	orr	r3, r2, ip, asl r3
	str	r3, [r1, r0, asl #2]
.L2615:
	ldr	r3, [r4, #-2948]
	ldr	r2, [r4, #-3104]
	ldr	r0, [r4, #-3116]
	ldrh	r3, [r3, r8]
	ldrh	r1, [r10]
	cmp	r3, r2
	strhi	r3, [r7, #-3104]
	ldr	r3, .L2656+28
	ldrh	r2, [r3]
	ldr	r3, [r4, #-3112]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	ldr	r2, [r4, #-2952]
	ldr	r3, .L2656+32
	str	r0, [r3, #204]
	ldr	r3, [r2, #16]
	add	r3, r3, #1
	str	r3, [r2, #16]
	ldr	r2, [r4, #-3024]
	mov	r3, #36
	add	r1, r2, #4
	mla	r3, r3, r6, r2
	add	r3, r3, #40
.L2617:
	add	r1, r1, #36
	cmp	r1, r3
	ldrne	r2, [r1, #-36]
	bicne	r2, r2, #1020
	bicne	r2, r2, #3
	strne	r2, [r1, #-36]
	bne	.L2617
.L2654:
	ldr	r3, .L2656+12
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2619
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	mov	r2, r6
	ldr	r0, [r4, #-3024]
	cmp	r3, #1
	moveq	r1, #0
	movne	r1, #1
	bl	FlashEraseBlocks
.L2619:
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	mov	r2, r6
	ldr	r0, [r4, #-3024]
	mov	r10, #0
	bl	FlashEraseBlocks
	add	r2, r5, #16
	mov	fp, r10
	mov	r1, #36
.L2621:
	uxth	r3, r10
	cmp	r3, r6
	bcs	.L2655
	mul	r3, r1, r10
	ldr	ip, [r4, #-3024]
	add	r0, ip, r3
	ldr	r3, [ip, r3]
	cmn	r3, #1
	bne	.L2622
	ldr	r0, [r0, #4]
	add	fp, fp, #1
	str	r1, [sp, #12]
	ubfx	r0, r0, #10, #16
	stmib	sp, {r2, r3}
	bl	FtlBbmMapBadBlock
	ldmib	sp, {r2, r3}
	ldr	r1, [sp, #12]
	strh	r3, [r2]	@ movhi
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	sub	r3, r3, #1
	strb	r3, [r5, #7]
.L2622:
	add	r10, r10, #1
	add	r2, r2, #2
	b	.L2621
.L2655:
	cmp	fp, #0
	beq	.L2624
	mov	r0, r9
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L2624:
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2625
	ldr	r3, [r4, #-2940]
.L2651:
	mvn	r2, #0
	strh	r2, [r3, r8]	@ movhi
	b	.L2603
.L2625:
	ldr	r2, .L2656+36
	ldrh	r2, [r2]
	strh	r9, [r5]	@ movhi
	smulbb	r3, r2, r3
	mov	r2, #0
	strh	r2, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	ldr	r2, [r4, #-3148]
	ldr	r1, [r4, #-2940]
	uxth	r3, r3
	strh	r3, [r5, #4]	@ movhi
	str	r2, [r5, #12]
	add	r2, r2, #1
	str	r2, [r4, #-3148]
	ldrh	r2, [r5]
	mov	r2, r2, asl #1
	strh	r3, [r1, r2]	@ movhi
.L2602:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2657:
	.align	2
.L2656:
	.word	.LANCHOR2
	.word	.LANCHOR2-552
	.word	.LANCHOR2-3248
	.word	.LANCHOR0
	.word	.LANCHOR2-656
	.word	.LANCHOR2-3260
	.word	.LANCHOR2-648
	.word	.LANCHOR2-3208
	.word	.LANCHOR4
	.word	.LANCHOR2-3200
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlGcBufInit
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r4, #12
	ldr	lr, .L2666
	mov	r5, #1
	ldr	r2, .L2666+4
	mov	r7, #36
	ldr	r1, .L2666+8
	add	r6, lr, #78
	mov	r3, #0
	str	r3, [r2, #208]
.L2659:
	ldrh	r2, [lr]
	add	ip, r3, #1
	uxth	r3, r3
	ldr	r0, .L2666+8
	cmp	r3, r2
	bcs	.L2664
	mul	r0, r4, r3
	ldr	r8, [r1, #-2980]
	add	r2, r8, r0
	str	r5, [r2, #8]
	ldrh	r2, [r6]
	mul	r2, r2, r3
	add	r9, r2, #3
	cmp	r2, #0
	movlt	r2, r9
	ldr	r9, [r1, #-2996]
	bic	r2, r2, #3
	add	r2, r9, r2
	str	r2, [r8, r0]
	ldr	r2, .L2666+12
	ldr	r9, [r1, #-2980]
	ldrh	r2, [r2]
	add	r8, r9, r0
	mul	r2, r2, r3
	add	r10, r2, #3
	cmp	r2, #0
	movlt	r2, r10
	ldr	r10, [r1, #-2968]
	bic	r2, r2, #3
	add	r2, r10, r2
	str	r2, [r8, #4]
	ldr	r2, [r1, #-3020]
	mla	r3, r7, r3, r2
	ldr	r2, [r9, r0]
	str	r2, [r3, #8]
	ldr	r2, [r8, #4]
	str	r2, [r3, #12]
	mov	r3, ip
	b	.L2659
.L2664:
	ldr	r4, .L2666+16
	mov	r1, r0
	mov	ip, #12
	mov	lr, #0
.L2661:
	ldr	r3, [r0, #-3012]
	cmp	r2, r3
	bcs	.L2665
	mul	r5, ip, r2
	ldr	r7, [r1, #-2980]
	add	r3, r7, r5
	str	lr, [r3, #8]
	ldrh	r3, [r4]
	mul	r3, r3, r2
	add	r6, r3, #3
	cmp	r3, #0
	movlt	r3, r6
	ldr	r6, [r1, #-2996]
	bic	r3, r3, #3
	add	r3, r6, r3
	str	r3, [r7, r5]
	ldr	r3, .L2666+12
	ldr	r6, [r1, #-2980]
	ldrh	r3, [r3]
	add	r5, r6, r5
	mul	r3, r3, r2
	add	r2, r2, #1
	uxth	r2, r2
	add	r6, r3, #3
	cmp	r3, #0
	movlt	r3, r6
	ldr	r6, [r1, #-2968]
	bic	r3, r3, #3
	add	r3, r6, r3
	str	r3, [r5, #4]
	b	.L2661
.L2665:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2667:
	.align	2
.L2666:
	.word	.LANCHOR2-3268
	.word	.LANCHOR4
	.word	.LANCHOR2
	.word	.LANCHOR2-3188
	.word	.LANCHOR2-3190
	.fnend
	.size	FtlGcBufInit, .-FtlGcBufInit
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
	ldr	r6, .L2670
	mov	r4, #0
	ldr	r5, .L2670+4
	mov	r1, r4
	strh	r3, [r6, #216]	@ movhi
	sub	r7, r5, #3248
	str	r3, [r6, #228]
	sub	r3, r5, #2816
	ldr	r0, [r5, #-2924]
	strh	r4, [r3, #-4]	@ movhi
	sub	r3, r5, #3168
	str	r4, [r6, #212]
	ldrh	r2, [r3, #-12]
	str	r4, [r6, #220]
	str	r4, [r6, #224]
	mov	r2, r2, asl #1
	str	r4, [r5, #-3336]
	bl	ftl_memset
	ldrh	r2, [r7, #-10]
	mov	r1, r4
	ldr	r0, [r5, #-2948]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r7, #-10]
	mov	r1, r4
	ldr	r0, [r5, #-2956]
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r1, r4
	sub	r0, r5, #716
	mov	r2, #48
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #512
	sub	r0, r6, #320
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2671:
	.align	2
.L2670:
	.word	.LANCHOR4
	.word	.LANCHOR2
	.fnend
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2680
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	lr, #0
	ldr	r6, [r3, #-3012]
	mov	r5, #36
	ldr	r4, [r3, #-2980]
	mov	r7, lr
	mov	r8, #12
.L2673:
	uxth	ip, lr
	cmp	ip, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	mla	ip, r5, ip, r0
	mov	r2, #0
.L2674:
	uxth	r3, r2
	cmp	r3, r6
	bcs	.L2675
	mul	r3, r8, r3
	add	r2, r2, #1
	add	r9, r4, r3
	ldr	r10, [r4, r3]
	ldr	r3, [ip, #8]
	cmp	r10, r3
	bne	.L2674
	str	r7, [r9, #8]
.L2675:
	add	lr, lr, #1
	b	.L2673
.L2681:
	.align	2
.L2680:
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
	ldr	r3, .L2691
	mov	ip, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r6, #12
	ldr	r4, [r3, #-3012]
	mov	r7, #1
	ldr	r5, [r3, #-2980]
	mov	r8, #36
.L2683:
	uxth	r2, ip
	cmp	r2, r1
	bcs	.L2690
	mov	lr, #0
.L2684:
	uxth	r3, lr
	cmp	r3, r4
	bcs	.L2685
	mla	r3, r6, r3, r5
	add	lr, lr, #1
	ldr	r9, [r3, #8]
	cmp	r9, #0
	bne	.L2684
	mla	r2, r8, r2, r0
	ldr	lr, [r3]
	str	r7, [r3, #8]
	str	lr, [r2, #8]
	ldr	r3, [r3, #4]
	str	r3, [r2, #12]
.L2685:
	add	ip, ip, #1
	b	.L2683
.L2690:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2692:
	.align	2
.L2691:
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
	ldr	r2, .L2699
	ldr	r3, .L2699+4
	ldrh	r2, [r2, #232]
	ldr	r3, [r3, #-3044]
	add	r2, r3, r2, asl #1
.L2694:
	cmp	r3, r2
	beq	.L2698
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L2694
	mov	r0, #1
	bx	lr
.L2698:
	mov	r0, #0
	bx	lr
.L2700:
	.align	2
.L2699:
	.word	.LANCHOR4
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
	mov	r5, r0
	ubfx	r0, r0, #10, #16
	mov	r6, r1
	mov	r7, r2
	bl	P2V_block_in_plane
	ldr	lr, .L2709
	mov	r4, #0
	ldr	r3, .L2709+4
	ldr	r2, [lr, #-3044]
	ldrh	r1, [r3, #232]
	sub	r8, r2, #2
.L2702:
	uxth	ip, r4
	cmp	ip, r1
	bcs	.L2706
	ldrh	r9, [r8, #2]!
	add	r4, r4, #1
	cmp	r9, r0
	bne	.L2702
.L2706:
	cmp	ip, r1
	moveq	ip, ip, asl #1
	streqh	r0, [r2, ip]	@ movhi
	ldreqh	r0, [r3, #232]
	addeq	r0, r0, #1
	streqh	r0, [r3, #232]	@ movhi
.L2704:
	ldrh	r2, [r3, #234]
	mov	ip, #12
	mul	ip, ip, r2
	ldr	r2, [lr, #-3040]
	add	r1, r2, ip
	stmib	r1, {r6, r7}
	str	r5, [r2, ip]
	ldrh	r2, [r3, #234]
	add	r2, r2, #1
	strh	r2, [r3, #234]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L2710:
	.align	2
.L2709:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r2, #0
	ldr	r4, .L2713
	mov	r1, #255
	ldr	r3, .L2713+4
	sub	r5, r4, #3184
	ldr	r0, [r4, #-3044]
	strh	r2, [r3, #232]	@ movhi
	strh	r2, [r3, #234]	@ movhi
	ldrh	r2, [r5, #-12]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r5, #-12]
	mov	r2, #12
	ldr	r0, [r4, #-3040]
	mov	r1, #255
	mul	r2, r2, r3
	bl	ftl_memset
	ldmfd	sp!, {r3, r4, r5, lr}
	b	FtlGcBufInit
.L2714:
	.align	2
.L2713:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 72
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L2762
	movw	r3, #3288
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #76
	sub	sp, sp, #76
	ldrh	r5, [r2, r3]
	movw	r3, #65535
	mov	r4, r0
	str	r1, [sp, #16]
	cmp	r5, r3
	beq	.L2746
	cmp	r5, #0
	bne	.L2716
	b	.L2717
.L2746:
	mov	r5, #0
.L2716:
	ldr	r3, .L2762+4
	ldr	r2, [sp, #16]
	ldrh	r3, [r3]
	cmp	r2, r3
	bne	.L2718
.L2717:
	bl	FtlGcPageVarInit
.L2718:
	ldr	r6, .L2762+8
	mvn	r3, #0
	str	r3, [sp, #12]
	mov	r3, #0
	str	r3, [sp, #4]
	mov	r8, r6
.L2719:
	ldrh	r2, [r4]
	mov	r3, #0
	strb	r3, [r4, #8]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2720
.L2721:
.L2743:
	ldr	r3, .L2762+12
	add	r0, r4, #14
	ldr	ip, [r6, #-2968]
	movw	r9, #65535
	mov	r10, #36
	ldrh	r3, [r3]
	str	r3, [sp, #8]
	ldr	r3, [r6, #-3036]
	str	r3, [sp, #20]
	ldr	r3, [r6, #-2996]
	str	r3, [sp, #24]
	ldr	r3, .L2762+16
	ldrh	r2, [r3]
	ldrh	lr, [r3, #2]
	str	r2, [sp, #28]
	mov	r2, #0
	mov	r7, r2
.L2722:
	ldr	r1, [sp, #8]
	uxth	r3, r2
	cmp	r3, r1
	bcs	.L2760
	ldrh	r3, [r0, #2]!
	cmp	r3, r9
	beq	.L2723
	ldr	r1, [sp, #20]
	orr	r3, r5, r3, asl #10
	mla	r1, r10, r7, r1
	str	r3, [r1, #4]
	ldr	r3, [sp, #28]
	mul	r3, r3, r7
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	ldr	fp, [sp, #24]
	bic	r3, r3, #3
	add	r3, fp, r3
	str	r3, [r1, #8]
	mul	r3, lr, r7
	add	r7, r7, #1
	uxth	r7, r7
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	bic	r3, r3, #3
	add	r3, ip, r3
	str	r3, [r1, #12]
.L2723:
	add	r2, r2, #1
	b	.L2722
.L2760:
	ldr	r0, [r6, #-3036]
	mov	r1, r7
	mov	r2, #0
	mov	fp, #0
	bl	FlashReadPages
.L2725:
	uxth	r3, fp
	cmp	r3, r7
	bcs	.L2761
	mov	r10, #36
	ldr	r2, [r6, #-3036]
	mul	r10, r10, fp
	ldr	ip, .L2762+8
	add	r3, r2, r10
	ldr	r9, [r2, r10]
	ldr	r1, [r3, #4]
	cmp	r9, #0
	ldr	r3, [r3, #12]
	str	r1, [sp, #8]
	bne	.L2726
	ldrh	r1, [r3]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L2727
.L2729:
	ldrh	r3, [r4]
	ldr	r2, [r8, #-2940]
	b	.L2759
.L2727:
	ldr	r0, [r3, #8]
	ldr	r2, [r6, #-2816]
	cmp	r0, r2
	bhi	.L2729
	ldr	r2, .L2762+20
	ldrb	r2, [r2]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2731
	add	r1, sp, #32
	mov	r2, r9
	str	r3, [sp, #20]
	bl	log2phys
	ldr	r3, [sp, #20]
	ldr	r1, [sp, #32]
	ldr	r2, [r3, #12]
	cmn	r1, #1
	rsb	r0, r2, r1
	clz	r0, r0
	mov	r0, r0, lsr #5
	moveq	r0, #0
	cmp	r0, #0
	beq	.L2731
	str	r2, [sp, #40]
	mov	r1, #1
	ldr	r2, [r6, #-2984]
	add	r0, sp, #36
	str	r2, [sp, #44]
	ldr	r2, [r6, #-2972]
	str	r2, [sp, #48]
	mov	r2, r9
	bl	FlashReadPages
	ldr	r2, .L2762+24
	ldr	r1, [r6, #-3036]
	ldrh	r2, [r2]
	add	r10, r1, r10
	mov	r2, r2, asl #7
	ldr	r1, [sp, #44]
	ldr	r3, [sp, #20]
	b	.L2733
.L2734:
	add	r9, r9, #1
.L2733:
	cmp	r9, r2
	beq	.L2731
	ldr	r0, [r10, #8]
	ldr	ip, [r0, r9, asl #2]
	ldr	r0, [r1, r9, asl #2]
	cmp	ip, r0
	beq	.L2734
	ldr	r2, [sp, #40]
	ldr	r0, .L2762+28
	ldrh	r1, [r4]
	bl	printk
	ldrh	r3, [r4]
	ldr	r2, [r6, #-2940]
.L2759:
	mov	r3, r3, asl #1
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	ldr	r2, .L2762+32
	mvn	r3, #0
	strh	r3, [r4]	@ movhi
	strh	r3, [r2]	@ movhi
	b	.L2758
.L2731:
	ldr	r0, [r3, #12]
	add	fp, fp, #1
	ldr	r1, [sp, #8]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
	b	.L2725
.L2726:
	ldr	r0, .L2762+36
	ldrh	r1, [r4]
	ldr	r2, [sp, #8]
	str	ip, [sp, #20]
	bl	printk
	ldr	r3, [r8, #-3336]
	ldrh	r5, [r4]
	cmp	r3, #0
	ldr	ip, [sp, #20]
	bne	.L2737
	ldr	r3, .L2762+20
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2738
.L2737:
	ldr	r2, [ip, #-2948]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #159
	bls	.L2739
.L2738:
	ldr	r3, [ip, #-3036]
	ldr	r3, [r3, r10]
	cmn	r3, #1
	bne	.L2740
.L2739:
	ldr	r3, [ip, #-3036]
	add	r10, r3, r10
	ldr	r3, [r10, #4]
	str	r3, [sp, #12]
.L2740:
	ldr	r3, [r8, #-2940]
	mov	r5, r5, asl #1
	mov	r2, #0
	strh	r2, [r3, r5]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r4]	@ movhi
.L2758:
	bl	FtlGcPageVarInit
	mov	r5, #0
	b	.L2719
.L2761:
	ldr	r3, [sp, #4]
	add	r5, r5, #1
	ldr	r2, [sp, #16]
	add	r3, r3, #1
	uxth	r5, r5
	cmp	r3, r2
	str	r3, [sp, #4]
	ldr	r2, .L2762+4
	bcs	.L2742
.L2744:
	ldrh	r3, [r2]
	cmp	r3, r5
	bhi	.L2743
	b	.L2720
.L2742:
	ldr	r1, .L2762+40
	movw	r0, #65535
	ldrh	r3, [r1]
	cmp	r3, r0
	beq	.L2744
	ldr	r0, [sp, #4]
	add	r3, r3, r0
	strh	r3, [r1]	@ movhi
	ldrh	r3, [r2]
	cmp	r3, r5
	bls	.L2744
	b	.L2745
.L2720:
	ldr	r2, .L2762
	movw	r3, #3288
	mvn	r1, #0
	strh	r5, [r4, #2]	@ movhi
	mov	r0, r4
	strh	r1, [r2, r3]	@ movhi
	mov	r2, #0
	mov	r1, r5
	strb	r2, [r4, #6]
	bl	ftl_sb_update_avl_pages
.L2745:
	ldr	r0, [sp, #12]
	add	sp, sp, #76
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2763:
	.align	2
.L2762:
	.word	.LANCHOR1
	.word	.LANCHOR2-3200
	.word	.LANCHOR2
	.word	.LANCHOR2-3268
	.word	.LANCHOR2-3190
	.word	.LANCHOR0
	.word	.LANCHOR2-3194
	.word	.LC142
	.word	.LANCHOR4-368
	.word	.LC143
	.word	.LANCHOR1+3288
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	FtlGcRefreshOpenBlock
	.type	FtlGcRefreshOpenBlock, %function
FtlGcRefreshOpenBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r6, .L2772
	ldrh	r3, [r6]
	cmp	r3, r0
	beq	.L2766
	add	r4, r6, #16
	ldrh	r3, [r4, #-14]
	cmp	r3, r0
	beq	.L2766
	ldrh	r3, [r4, #-12]
	cmp	r3, r0
	beq	.L2766
	ldrh	r3, [r4, #-10]
	cmp	r3, r0
	beq	.L2766
	ldr	r0, .L2772+4
	mov	r1, r5
	bl	printk
	ldrh	r2, [r6]
	movw	r3, #65535
	cmp	r2, r3
	streqh	r5, [r6]	@ movhi
	beq	.L2766
	ldrh	r2, [r4, #-14]
	cmp	r2, r3
	streqh	r5, [r4, #-14]	@ movhi
	beq	.L2766
	ldrh	r2, [r4, #-12]
	cmp	r2, r3
	streqh	r5, [r4, #-12]	@ movhi
	beq	.L2766
	ldrh	r2, [r4, #-10]
	cmp	r2, r3
	streqh	r5, [r4, #-10]	@ movhi
.L2766:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2773:
	.align	2
.L2772:
	.word	.LANCHOR2-3072
	.word	.LC144
	.fnend
	.size	FtlGcRefreshOpenBlock, .-FtlGcRefreshOpenBlock
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r6, .L2785
	ldrh	r3, [r6]
	cmp	r3, r0
	beq	.L2782
	add	r4, r6, #16
	ldrh	r3, [r4, #-14]
	cmp	r3, r0
	beq	.L2782
	ldrh	r3, [r4, #-12]
	cmp	r3, r0
	beq	.L2782
	ldrh	r3, [r4, #-10]
	cmp	r3, r0
	beq	.L2782
	ldr	r0, .L2785+4
	mov	r1, r5
	bl	printk
	ldrh	r2, [r6]
	movw	r3, #65535
	cmp	r2, r3
	streqh	r5, [r6]	@ movhi
	beq	.L2782
	ldrh	r2, [r4, #-14]
	cmp	r2, r3
	streqh	r5, [r4, #-14]	@ movhi
	beq	.L2782
	ldrh	r2, [r4, #-12]
	cmp	r2, r3
	streqh	r5, [r4, #-12]	@ movhi
	beq	.L2782
	ldrh	r2, [r4, #-10]
	cmp	r2, r3
	bne	.L2783
	strh	r5, [r4, #-10]	@ movhi
.L2782:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2783:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2786:
	.align	2
.L2785:
	.word	.LANCHOR2-3072
	.word	.LC144
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
	ldr	r6, .L2797
	mov	r2, r5
	sub	r7, r6, #3056
	ldrh	r1, [r7, #-2]
	mov	r4, r0
	ldr	r0, .L2797+4
	bl	printk
	mov	r0, r4
	bl	FtlGcRefreshBlock
	ldr	r3, [r6, #-3336]
	cmp	r3, #0
	beq	.L2788
	ldr	r2, [r6, #-2948]
	mov	r4, r4, asl #1
	ldrh	r3, [r2, r4]
	cmp	r3, #39
	subhi	r3, r3, #40
	strhih	r3, [r2, r4]	@ movhi
.L2788:
	ldrh	r3, [r7, #-2]
	mov	r2, #0
	ldr	r0, .L2797+8
.L2789:
	uxth	r1, r2
	cmp	r1, r3
	bcs	.L2796
	add	r2, r2, #1
	add	r1, r0, r2, asl #1
	ldrh	r1, [r1, #-2]
	cmp	r1, r5
	bne	.L2789
	b	.L2790
.L2796:
	cmp	r3, #15
	addls	r2, r3, #1
	strlsh	r2, [r7, #-2]	@ movhi
	ldrls	r2, .L2797+12
	addls	r3, r2, r3, asl #1
	strlsh	r5, [r3, #236]	@ movhi
.L2790:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2798:
	.align	2
.L2797:
	.word	.LANCHOR2
	.word	.LC145
	.word	.LANCHOR4+236
	.word	.LANCHOR4
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
	ldr	r4, .L2806
	ldrh	r3, [r4, #-2]
	cmp	r3, #0
	beq	.L2800
	ldrh	r1, [r4, #-16]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L2800
	add	r4, r4, #16
	ldrh	r2, [r4, #-14]
	cmp	r2, r3
	movcs	r3, #0
	strcsh	r3, [r4, #-14]	@ movhi
	ldrh	r2, [r4, #-14]
	ldr	r3, .L2806+4
	add	r3, r3, r2, asl #1
	ldrh	r0, [r3, #236]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r4, #-14]
	add	r3, r3, #1
	strh	r3, [r4, #-14]	@ movhi
.L2800:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L2807:
	.align	2
.L2806:
	.word	.LANCHOR2-3056
	.word	.LANCHOR4
	.fnend
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2822
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldrh	r2, [r3, #-2]
	cmp	r2, #0
	beq	.L2810
	sub	r5, r3, #2
	mov	r7, r0
	mov	r6, #0
	mov	r8, r5
.L2809:
	ldr	r3, .L2822+4
	uxth	r2, r6
	ldrh	r3, [r3]
	cmp	r3, r2
	bls	.L2819
	ldr	r3, .L2822+8
	mov	r1, r7
	ldr	fp, .L2822+12
	mov	r9, #0
	add	r3, r3, r2
	ldrb	r0, [r3, #-3240]	@ zero_extendqisi2
	bl	V2P_block
	mov	r10, r0
.L2811:
	ldrh	r3, [r5]
	uxth	r4, r9
	cmp	r3, r4
	bls	.L2820
	mov	ip, r4, asl #1
	add	r3, fp, ip
	ldrh	r3, [r3, #236]
	cmp	r3, r10
	bne	.L2812
	mov	r1, r10
	ldr	r0, .L2822+16
	str	ip, [sp, #4]
	bl	printk
	mov	r0, r10
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, .L2822+20
	ldrh	r2, [r5]
	ldr	ip, [sp, #4]
	add	r3, r3, ip
.L2813:
	cmp	r4, r2
	ldrcch	r1, [r3, #2]
	addcc	r4, r4, #1
	uxthcc	r4, r4
	strcch	r1, [r3], #2	@ movhi
	bcc	.L2813
.L2821:
	sub	r2, r2, #1
	strh	r2, [r8]	@ movhi
.L2812:
	add	r9, r9, #1
	b	.L2811
.L2820:
	add	r6, r6, #1
	b	.L2809
.L2819:
	bl	FtlGcReFreshBadBlk
.L2810:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2823:
	.align	2
.L2822:
	.word	.LANCHOR2-3056
	.word	.LANCHOR2-3268
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	.LC146
	.word	.LANCHOR4+236
	.fnend
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L2834
	mov	r3, r0, asl #1
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	ldr	r1, [r2, #-2940]
	ldrh	r3, [r1, r3]
	cmp	r3, #0
	bne	.L2825
	ldr	r1, .L2834+4
	ldrh	r0, [r1]
	cmp	r0, r4
	mvneq	r3, #0
	streqh	r3, [r1]	@ movhi
	beq	.L2827
	sub	r1, r2, #648
	ldrh	r1, [r1]
	cmp	r1, r4
	beq	.L2833
	sub	r1, r2, #600
	ldrh	r1, [r1]
	cmp	r1, r4
	beq	.L2833
	sub	r2, r2, #552
	ldrh	r2, [r2]
	cmp	r2, r4
	beq	.L2833
.L2827:
	mov	r1, r4
	ldr	r0, .L2834+8
	bl	List_remove_node
	ldr	r2, .L2834+12
	mov	r0, r4
	ldrh	r3, [r2]
	sub	r3, r3, #1
	strh	r3, [r2]	@ movhi
	bl	free_data_superblock
	mov	r0, r4
	bl	FtlGcFreeBadSuperBlk
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L2825:
	bl	List_update_data_list
.L2833:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L2835:
	.align	2
.L2834:
	.word	.LANCHOR2
	.word	.LANCHOR4-368
	.word	.LANCHOR2-668
	.word	.LANCHOR2-660
	.fnend
	.size	update_vpc_list, .-update_vpc_list
	.align	2
	.global	decrement_vpc_count
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r3, #65535
	cmp	r0, r3
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	beq	.L2837
	ldr	r5, .L2847
	mov	r6, r0, asl #1
	ldr	r3, [r5, #-2940]
	ldrh	r2, [r3, r6]
	cmp	r2, #0
	subne	r2, r2, #1
	strneh	r2, [r3, r6]	@ movhi
	bne	.L2837
	mov	r1, r4
	ldr	r0, .L2847+4
	bl	printk
	ldr	r3, [r5, #-2940]
	mov	r2, #32
	sub	r0, r5, #656
	mov	r1, r4
	strh	r2, [r3, r6]	@ movhi
	bl	test_node_in_list
	cmp	r0, #0
	beq	.L2839
	mov	r1, r4
	sub	r0, r5, #656
	bl	List_remove_node
	sub	r2, r5, #652
	mov	r0, r4
	ldrh	r3, [r2]
	sub	r3, r3, #1
	strh	r3, [r2]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r5, #-2940]
	ldr	r0, .L2847+8
	mov	r1, r4
	ldrh	r2, [r3, r6]
	bl	printk
.L2839:
	mov	r0, r4
	bl	FtlGcRefreshBlock
	b	.L2842
.L2837:
	ldr	r5, .L2847+12
	movw	r3, #65535
	ldrh	r0, [r5, #216]
	cmp	r0, r3
	streqh	r4, [r5, #216]	@ movhi
	beq	.L2842
	cmp	r0, r4
	beq	.L2842
	bl	update_vpc_list
	strh	r4, [r5, #216]	@ movhi
	adds	r0, r0, #0
	movne	r0, #1
	ldmfd	sp!, {r4, r5, r6, pc}
.L2842:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2848:
	.align	2
.L2847:
	.word	.LANCHOR2
	.word	.LC147
	.word	.LC148
	.word	.LANCHOR4
	.fnend
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	ldrh	r3, [r0]
	movw	r2, #65535
	mov	r1, r0
	cmp	r3, r2
	beq	.L2989
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #68
	sub	sp, sp, #68
	ldrh	r3, [r0, #2]
	str	r3, [sp, #8]
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	ldr	r0, [sp, #8]
	str	r3, [sp, #16]
	ldr	r3, .L3004
	ldrh	r3, [r3]
	cmp	r3, r0
	mov	r3, #0
	streqh	r3, [r1, #4]	@ movhi
	streqb	r3, [r1, #6]
	ldrneh	r0, [r1, #16]
	beq	.L2986
.L2853:
	cmp	r0, r2
	add	r3, r3, #1
	uxtheq	r0, r3
	addeq	r0, r1, r0, asl #1
	ldreqh	r0, [r0, #16]
	beq	.L2853
.L2999:
	mov	r9, r1
	ldrb	r1, [r1, #8]	@ zero_extendqisi2
	cmp	r1, #1
	bne	.L2855
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r4, r0
	beq	.L2856
	ldr	r3, .L3004+4
	ldrb	r2, [r3, #3904]	@ zero_extendqisi2
	cmp	r2, #0
	addeq	r3, r3, r0, asl #1
	ldreqh	r5, [r3, #76]
	beq	.L2926
	b	.L2990
.L2855:
	mov	r1, #0
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r4, r0
	beq	.L2856
.L2990:
	mov	r5, r4
.L2926:
	ldr	r3, .L3004+8
	movw	r8, #65535
	sub	r2, r3, #3264
	sub	r3, r3, #3184
	ldr	ip, [r3, #148]
	ldr	lr, [r3, #216]
	ldrh	r2, [r2, #-4]
	ldrh	r7, [r3, #-4]
	add	r3, r9, #14
	str	r3, [sp, #20]
	str	r2, [sp]
	mov	r2, #0
	mov	r0, r3
	mov	r6, r2
	mov	r10, r2
	b	.L2858
.L2856:
	mov	r3, #0
	strh	r3, [r9, #2]	@ movhi
	strb	r3, [r9, #6]
	b	.L2986
.L2860:
	ldrh	r3, [r0, #2]!
	cmp	r3, r8
	beq	.L2859
	mov	r1, #36
	orr	r3, r5, r3, asl #10
	mla	r1, r1, r6, ip
	stmib	r1, {r3, r10}
	mul	r3, r7, r6
	add	r6, r6, #1
	uxth	r6, r6
	add	fp, r3, #3
	cmp	r3, #0
	movlt	r3, fp
	bic	r3, r3, #3
	add	r3, lr, r3
	str	r3, [r1, #12]
.L2859:
	add	r2, r2, #1
.L2858:
	ldr	r1, [sp]
	uxth	r3, r2
	cmp	r3, r1
	bcc	.L2860
	ldrb	r3, [r9, #8]	@ zero_extendqisi2
	cmp	r3, #1
	movne	r3, #0
	bne	.L2991
	ldr	r3, .L3004+4
	ldrb	lr, [r3, #3904]	@ zero_extendqisi2
	adds	r3, lr, #0
	movne	r3, #1
.L2991:
	ldr	r7, .L3004+8
	mov	r1, r6
	str	r3, [sp, #24]
	mov	fp, #0
	ldr	r2, [sp, #24]
	movw	r10, #65535
	ldr	r0, [r7, #-3036]
	bl	FlashReadPages
	ldr	r3, [r7, #-3144]
	str	r7, [sp]
	str	r3, [sp, #28]
.L2862:
	uxth	r8, fp
	cmp	r8, r6
	bcs	.L2869
	mov	r1, #36
	ldr	r0, [r7, #-3036]
	mul	r1, r1, fp
	add	ip, r0, r1
	ldr	r1, [r0, r1]
	cmp	r1, #0
	bne	.L2863
	ldr	ip, [ip, #12]
	ldr	r3, [ip, #4]
	cmn	r3, #1
	beq	.L2864
	mov	r0, r3
	ldr	r1, [r7, #-3144]
	str	ip, [sp, #12]
	str	r3, [sp, #4]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #4]
	cmp	r0, #0
	ldr	ip, [sp, #12]
	addne	r3, r3, #1
	strne	r3, [r7, #-3144]
.L2864:
	ldr	r1, [ip]
	cmn	r1, #1
	bne	.L2865
.L2869:
	cmp	r8, r6
	bne	.L2987
	add	fp, r4, #1
	uxth	r3, fp
	str	r3, [sp, #4]
	ldr	r3, .L3004+8
	ldr	r3, [r3, #-3036]
	ldr	r0, [r3, #4]
	b	.L2992
.L2863:
	ldr	r1, [ip, #4]
	uxth	r10, r5
	ldr	r0, .L3004+12
	bl	printk
	ldrh	r1, [r9]
	ldr	r3, .L3004+16
	strh	r1, [r3]	@ movhi
.L2865:
	add	fp, fp, #1
	b	.L2862
.L2987:
	uxth	r3, r4
	str	r3, [sp, #4]
	ldr	r3, [sp]
	mov	r2, #36
	ldr	r3, [r3, #-3036]
	mla	r8, r2, r8, r3
	ldr	r0, [r8, #4]
.L2992:
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldrb	r3, [r9, #8]	@ zero_extendqisi2
	cmp	r3, #1
	str	r0, [sp, #12]
	bne	.L2871
	ldr	r2, .L3004+4
	ldrb	r1, [r2, #3904]	@ zero_extendqisi2
	cmp	r1, #0
	ldreq	r1, [sp, #4]
	addeq	r2, r2, r1, asl #1
	ldreqh	r2, [r2, #76]
	streq	r2, [sp, #4]
.L2871:
	ldr	r2, .L3004
	ldr	r1, [sp, #4]
	ldr	r0, [sp, #12]
	ldrh	r2, [r2]
	cmp	r2, r1
	ldr	r1, [sp, #8]
	ldreqh	r2, [sp, #4]
	streqh	r2, [r9, #2]	@ movhi
	moveq	r2, #0
	streqb	r2, [r9, #6]
	streqh	r2, [r9, #4]	@ movhi
	ldrh	r2, [sp, #16]
	str	r2, [sp, #32]
	ldr	ip, [sp, #32]
	ldr	r2, [sp, #4]
	cmp	r2, r1
	cmpeq	r0, ip
	moveq	r0, r9
	moveq	r1, r2
	beq	.L2997
	clz	r3, r3
	ldr	r2, [sp, #28]
	mov	r3, r3, lsr #5
	sub	r2, r2, #1
	str	r2, [sp]
	movw	r2, #65535
	cmp	r10, r2
	orrne	r3, r3, #1
	cmp	r3, #0
	beq	.L2875
	ldr	r3, .L3004+20
	uxth	r5, r5
	ldr	fp, .L3004+8
	mvn	r8, #0
	mov	r7, r8
	ldr	r2, [r3, #228]
	cmn	r2, #1
	ldreq	r2, [sp]
	streq	r2, [r3, #228]
	ldr	r10, [r3, #228]
	ldr	r3, [sp, #8]
	add	r3, r3, #7
	cmp	r5, r3
	subgt	r4, r5, #7
	ldrle	r4, [sp, #8]
	uxthgt	r4, r4
.L2878:
	cmp	r4, r5
	bhi	.L2891
	ldr	r3, .L3004+24
	mov	r0, #36
	ldr	lr, [fp, #-3036]
	ldr	r1, [sp, #20]
	ldrh	r3, [r3]
	str	r3, [sp, #16]
	mov	r3, #0
	mov	r6, r3
.L2892:
	ldr	ip, [sp, #16]
	uxth	r2, r3
	cmp	r2, ip
	bcs	.L3000
	ldrh	r2, [r1, #2]!
	movw	ip, #65535
	add	r3, r3, #1
	cmp	r2, ip
	orrne	r2, r4, r2, asl #10
	mlane	ip, r0, r6, lr
	addne	r6, r6, #1
	uxthne	r6, r6
	strne	r2, [ip, #4]
	b	.L2892
.L3000:
	mov	r1, r6
	ldr	r2, [sp, #24]
	ldr	r0, [fp, #-3036]
	bl	FlashReadPages
	ldr	r2, .L3004+4
	ldr	r3, [fp, #-3036]
	mov	r1, #36
	add	r2, r2, #588
	ldrb	ip, [r2, #3316]	@ zero_extendqisi2
	add	r2, r2, r4, asl #1
	mla	r6, r1, r6, r3
	movw	r1, #65535
.L2881:
	cmp	r3, r6
	beq	.L3001
	ldr	r0, [r3]
	cmp	r0, #0
	bne	.L2882
	ldr	r0, [r3, #12]
	ldrh	lr, [r0]
	cmp	lr, r1
	beq	.L2883
	ldr	r0, [r0, #4]
	cmn	r0, #1
	beq	.L2883
	ldr	lr, .L3004+20
	cmn	r8, #1
	ldr	r7, [lr, #228]
	str	r0, [lr, #228]
	bne	.L2883
	ldrh	r0, [r2]
	cmp	r0, r1
	bne	.L2884
	cmp	ip, #0
	beq	.L2883
.L2884:
	ldr	r0, [sp]
	cmp	r7, r0
	mvneq	r8, #0
	movne	r8, r7
	b	.L2883
.L2882:
	ldrh	r1, [r9]
	movw	r2, #270
	ldr	r3, .L3004+20
	strh	r1, [r3, r2]	@ movhi
	ldrb	r2, [r9, #8]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L2875
	ldr	r2, .L3004+28
	mov	r4, r4, asl #1
	ldrh	r1, [r2, r4]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L2886
	cmn	r8, #1
	strne	r8, [r3, #228]
	bne	.L2875
	ldr	r2, [sp]
	cmp	r10, r2
	strne	r10, [r3, #228]
	ldreq	r2, [r3, #228]
	bne	.L2875
	b	.L2998
.L2886:
	ldr	r2, [sp]
	cmp	r7, r2
	beq	.L2889
	cmn	r7, #1
	strne	r7, [r3, #228]
	b	.L2875
.L2889:
	ldr	r2, [r3, #228]
	ldr	r1, [sp]
	cmp	r2, r1
	beq	.L2875
.L2998:
	sub	r2, r2, #1
	b	.L2993
.L2883:
	add	r3, r3, #36
	b	.L2881
.L3001:
	add	r4, r4, #1
	uxth	r4, r4
	b	.L2878
.L2891:
	ldr	r3, .L3004+20
	mvn	r2, #0
.L2993:
	str	r2, [r3, #228]
.L2875:
	ldr	fp, .L3004+32
	mov	r3, #1
	ldr	r10, [sp, #8]
	strh	r3, [fp]	@ movhi
	sub	fp, fp, #272
.L2893:
	ldr	r3, .L3004+24
	movw	lr, #65535
	ldr	r5, [sp, #20]
	mov	r1, #36
	mov	r4, #0
	str	r4, [sp, #16]
	ldrh	r6, [r3]
	ldr	r3, .L3004+8
	ldr	r7, [r3, #-3036]
	ldr	r3, .L3004+4
	ldrb	r8, [r3, #3904]	@ zero_extendqisi2
.L2894:
	uxth	r3, r4
	cmp	r3, r6
	bcs	.L3002
	ldrh	r3, [r5, #2]!
	cmp	r3, lr
	beq	.L2895
	ldr	r2, [sp, #16]
	orr	r3, r10, r3, asl #10
	mla	r2, r1, r2, r7
	str	r3, [r2, #4]
	ldrb	r0, [r9, #8]	@ zero_extendqisi2
	cmp	r0, #1
	bne	.L2896
	cmp	r8, #0
	orrne	r3, r3, #-2147483648
	strne	r3, [r2, #4]
.L2896:
	ldr	r3, [sp, #16]
	add	ip, r3, #1
	uxth	r3, ip
	str	r3, [sp, #16]
.L2895:
	add	r4, r4, #1
	b	.L2894
.L3002:
	ldr	r4, .L3004+8
	ldr	r1, [sp, #16]
	ldr	r2, [sp, #24]
	ldr	r0, [r4, #-3036]
	bl	FlashReadPages
	mov	r3, #0
.L2996:
	str	r3, [sp, #28]
	ldr	r2, [sp, #16]
	ldrh	r3, [sp, #28]
	cmp	r3, r2
	bcs	.L3003
	ldr	r3, [sp, #28]
	mov	r5, #36
	ldr	r8, [r4, #-3036]
	mul	r5, r5, r3
	add	r7, r8, r5
	ldr	r6, [r7, #4]
	ubfx	r0, r6, #10, #16
	str	r6, [sp, #60]
	bl	P2V_plane
	ldr	r3, [sp, #8]
	cmp	r10, r3
	bcc	.L2899
	ldr	r3, [sp, #32]
	ldr	r2, [sp, #8]
	cmp	r0, r3
	movcs	r3, #0
	movcc	r3, #1
	cmp	r10, r2
	movne	r3, #0
	cmp	r3, #0
	bne	.L2899
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #4]
	cmp	r0, r3
	cmpeq	r10, r2
	beq	.L2900
	ldr	r3, [r8, r5]
	cmn	r3, #1
	beq	.L2901
	ldr	r7, [r7, #12]
	movw	r3, #61589
	ldrh	r2, [r7]
	cmp	r2, r3
	ldrneh	r0, [r9]
	bne	.L2994
	ldr	r3, [r7, #4]
	cmn	r3, #1
	str	r3, [sp]
	beq	.L2903
	mov	r0, r3
	ldr	r1, [r4, #-3144]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	ldrne	r3, [sp]
	addne	r3, r3, #1
	strne	r3, [r4, #-3144]
.L2903:
	ldr	r6, [r7, #8]
	add	r1, sp, #56
	ldr	r3, [r7, #12]
	mov	r2, #0
	mov	r0, r6
	str	r3, [sp, #52]
	bl	log2phys
	ldr	r1, [fp, #228]
	cmn	r1, #1
	beq	.L2904
	ldr	r0, [sp]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L2904
	ldr	r3, [sp, #52]
	cmn	r3, #1
	beq	.L2905
	ldr	r0, [r4, #-3036]
	mov	r2, #0
	mov	r1, #1
	add	r0, r0, r5
	str	r3, [r0, #4]
	ldr	r8, [r0, #12]
	bl	FlashReadPages
	ldr	r2, [r4, #-3036]
	ldr	r3, [r8, #4]
	add	ip, r2, r5
	str	r3, [sp, #36]
	ldr	r3, [r2, r5]
	cmn	r3, #1
	bne	.L2906
	b	.L2907
.L2905:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	bne	.L2899
	mov	r0, r6
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
	b	.L2899
.L2906:
	ldr	r7, [r8, #8]
	cmp	r7, r6
	bne	.L2907
	ldr	r0, [fp, #228]
	ldr	r1, [sp, #36]
	str	r2, [sp, #44]
	str	ip, [sp, #40]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	ldr	ip, [sp, #40]
	ldr	r2, [sp, #44]
	beq	.L2907
	ldr	r3, [sp, #56]
	ldr	r1, [sp, #60]
	cmp	r3, r1
	beq	.L2912
	ldr	r1, [sp, #52]
	cmp	r3, r1
	beq	.L2907
	cmn	r3, #1
	streq	r3, [r2, r5]
	beq	.L2911
	str	r3, [ip, #4]
	mov	r0, ip
	mov	r1, #1
	mov	r2, #0
	ldr	r8, [ip, #12]
	bl	FlashReadPages
.L2911:
	ldr	r3, [r4, #-3036]
	ldr	r3, [r3, r5]
	cmn	r3, #1
	beq	.L2912
	ldr	r5, [r8, #4]
	ldr	r0, [fp, #228]
	mov	r1, r5
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L2912
	ldr	r0, [sp, #36]
	mov	r1, r5
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L2907
.L2912:
	mov	r0, r7
	ldr	r1, [sp, #52]
	bl	FtlReUsePrevPpa
.L2907:
	mvn	r3, #0
	str	r3, [sp, #52]
	b	.L2914
.L2904:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	beq	.L2914
	ldr	r3, [sp, #52]
	ldr	r2, [r4, #-3252]
	mov	r1, r3, lsr #10
	adds	r3, r3, #1
	movne	r3, #1
	cmp	r1, r2
	movcc	r3, #0
	cmp	r3, #0
	bne	.L2899
	mov	r0, r6
	add	r1, sp, #60
	mov	r2, #1
	bl	log2phys
	ldr	r5, [sp, #56]
	cmn	r5, #1
	beq	.L2914
	ldr	r3, [sp, #52]
	cmp	r5, r3
	beq	.L2914
	ubfx	r0, r5, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L3004+36
	ldrh	r2, [r3]
	cmp	r2, r0
	beq	.L2917
	ldrh	r2, [r3, #48]
	cmp	r2, r0
	beq	.L2917
	ldrh	r3, [r3, #96]
	cmp	r3, r0
	bne	.L2914
.L2917:
	ldr	r0, [r4, #-3036]
	mov	r1, #1
	mov	r2, #0
	str	r5, [r0, #4]
	ldr	r7, [r0, #12]
	bl	FlashReadPages
	ldr	r3, [r4, #-3036]
	ldr	r1, [r7, #4]
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L2914
	ldr	r0, [sp]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L2914
	mov	r0, r6
	add	r1, sp, #56
	mov	r2, #1
	bl	log2phys
.L2914:
	ldr	r0, [sp, #52]
	cmn	r0, #1
	beq	.L2899
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r4, #-2940]
	mov	r3, r0, asl #1
	mov	r1, r0
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2918
.L2994:
	bl	decrement_vpc_count
	b	.L2899
.L2918:
	ldr	r0, .L3004+40
	bl	printk
	b	.L2899
.L2901:
	ldrh	r2, [r9]
	mov	r1, r6
	ldr	r3, .L3004+16
	ldr	r0, .L3004+44
	strh	r2, [r3]	@ movhi
	ldr	r2, [sp]
	bl	printk
	ldr	r3, [fp, #276]
	cmp	r3, #31
	addls	r2, fp, r3, asl #2
	addls	r3, r3, #1
	strls	r3, [fp, #276]
	ldrls	r1, [sp, #60]
	strls	r1, [r2, #280]
	ldrh	r0, [r9]
	bl	decrement_vpc_count
	ldr	r3, .L3004+20
	ldr	r3, [r3, #228]
	cmn	r3, #1
	ldreq	r3, [sp]
	beq	.L2995
	ldr	r2, [sp]
	cmp	r3, r2
	bls	.L2899
	mov	r3, r2
.L2995:
	str	r3, [fp, #228]
.L2899:
	ldr	r3, [sp, #28]
	add	r3, r3, #1
	b	.L2996
.L3003:
	ldrb	r3, [r9, #8]	@ zero_extendqisi2
	add	r10, r10, #1
	cmp	r3, #1
	uxth	r10, r10
	bne	.L2922
	ldr	r3, .L3004+4
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2922
	ldr	r3, .L3004+48
	ldr	r2, [sp, #4]
	ldrh	r3, [r3]
	cmp	r2, r10
	cmpeq	r3, r10
	beq	.L2900
.L2922:
	ldr	r2, .L3004
	ldrh	r3, [r2]
	cmp	r10, r3
	bne	.L2893
	ldrh	r1, [r2, #-68]
	movw	r0, #65535
	mov	r3, #0
	strh	r10, [r9, #2]	@ movhi
	strh	r3, [r9, #4]	@ movhi
.L2923:
	uxth	r2, r3
	cmp	r2, r1
	bcs	.L2986
	ldr	lr, [sp, #20]
	add	r3, r3, #1
	ldrh	ip, [lr, #2]!
	cmp	ip, r0
	str	lr, [sp, #20]
	beq	.L2923
	strb	r2, [r9, #6]
	b	.L2986
.L2900:
	ldrb	r3, [sp, #12]	@ zero_extendqisi2
	mov	r0, r9
	ldr	r1, [sp, #4]
	strb	r3, [r9, #6]
	ldrh	r3, [sp, #4]
	strh	r3, [r9, #2]	@ movhi
.L2997:
	ldr	r2, [sp, #12]
	bl	ftl_sb_update_avl_pages
.L2986:
	mov	r0, #0
	add	sp, sp, #68
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2989:
	mov	r0, #0
	bx	lr
.L3005:
	.align	2
.L3004:
	.word	.LANCHOR2-3200
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC149
	.word	.LANCHOR4+270
	.word	.LANCHOR4
	.word	.LANCHOR2-3268
	.word	.LANCHOR0+588
	.word	.LANCHOR4+272
	.word	.LANCHOR2-648
	.word	.LC150
	.word	.LC151
	.word	.LANCHOR2-3198
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
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
	ldr	r5, .L3020
	ldr	r7, .L3020+4
	add	r3, r0, r3, asl #1
	add	r6, r5, #588
	ldrh	r3, [r3, #16]
.L3010:
	movw	r1, #65535
	cmp	r3, r1
	bne	.L3019
.L3012:
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
	b	.L3010
.L3019:
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L3013
	ldrb	r3, [r5, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3013
	ldrh	r3, [r4, #2]
	mov	r3, r3, asl #1
	ldrh	r3, [r6, r3]
	cmp	r3, r1
	bne	.L3013
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	ldrh	r2, [r4, #4]
	cmp	r2, #0
	bne	.L3012
	ldrh	r3, [r4, #2]
	add	r3, r3, #1
	b	.L3018
.L3013:
	ldr	r3, .L3020
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r2, #1
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	ldr	r3, .L3020+8
	ldrh	r1, [r4, #2]
	sub	r2, r3, #3184
	ldrh	r2, [r2, #-14]
	cmp	r1, r2
	ldmccfd	sp!, {r3, r4, r5, r6, r7, pc}
	ldrh	r2, [r4]
	sub	r3, r3, #3200
	ldr	r0, [r3, #260]
	ldrh	ip, [r4, #4]
	mov	r2, r2, asl #1
	ldrh	r1, [r0, r2]
	rsb	r1, ip, r1
	strh	r1, [r0, r2]	@ movhi
	ldrh	r3, [r3]
	mov	r2, #0
	strh	r2, [r4, #4]	@ movhi
.L3018:
	strh	r3, [r4, #2]	@ movhi
	strb	r2, [r4, #6]
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3021:
	.align	2
.L3020:
	.word	.LANCHOR0
	.word	.LANCHOR2-3268
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
	ldr	r7, .L3040
	ldr	r8, .L3040+4
	add	r3, r0, r3, asl #1
	add	r9, r7, #588
	ldrh	r2, [r3, #16]
.L3023:
	movw	r1, #65535
	cmp	r2, r1
	ldr	r6, .L3040+4
	bne	.L3039
.L3024:
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
	b	.L3023
.L3039:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	ldrh	r5, [r4, #2]
	cmp	r3, #1
	ldrh	r3, [r4, #4]
	bne	.L3026
	ldrb	r0, [r7, #3904]	@ zero_extendqisi2
	cmp	r0, #0
	bne	.L3026
	mov	r0, r5, asl #1
	ldrh	r0, [r9, r0]
	cmp	r0, r1
	bne	.L3026
	sub	r3, r3, #1
	ldrh	r0, [r4]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L3024
.L3026:
	ldr	r7, .L3040
	orr	r5, r5, r2, asl #10
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
.L3027:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	r2, #65535
	ldrh	r0, [r6]
.L3029:
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
	beq	.L3029
	strb	r3, [r4, #6]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L3034
	ldrb	r3, [r7, #3904]	@ zero_extendqisi2
	ldr	r2, .L3040
	cmp	r3, #0
	bne	.L3031
	ldrh	r3, [r4, #2]
	add	r1, r2, #588
	mov	r3, r3, asl #1
	ldrh	r1, [r1, r3]
	movw	r3, #65535
	cmp	r1, r3
	bne	.L3031
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L3031
	sub	r3, r3, #1
	ldrh	r0, [r4]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L3027
.L3031:
	ldrb	r3, [r2, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3034
	ldr	r3, .L3040+8
	ldrh	r1, [r4, #2]
	sub	r2, r3, #3184
	ldrh	r2, [r2, #-14]
	cmp	r1, r2
	bcc	.L3034
	ldrh	r2, [r4]
	sub	r3, r3, #3200
	ldr	r0, [r3, #260]
	ldrh	ip, [r4, #4]
	mov	r2, r2, asl #1
	ldrh	r1, [r0, r2]
	rsb	r1, ip, r1
	strh	r1, [r0, r2]	@ movhi
	ldrh	r3, [r3]
	mov	r2, #0
	strh	r2, [r4, #4]	@ movhi
	strb	r2, [r4, #6]
	strh	r3, [r4, #2]	@ movhi
.L3034:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L3041:
	.align	2
.L3040:
	.word	.LANCHOR0
	.word	.LANCHOR2-3268
	.word	.LANCHOR2
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
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
	ldr	r4, .L3061
	ldr	r3, [r4, #-2812]
	cmp	r3, #0
	bne	.L3042
	sub	r7, r4, #648
	ldrh	r5, [r7, #4]
	cmp	r5, #0
	beq	.L3044
	ldrb	r3, [r4, #-640]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3044
	sub	r3, r4, #3200
	ldrb	r2, [r4, #-641]	@ zero_extendqisi2
	ldrh	r3, [r3]
	mul	r3, r3, r2
	cmp	r5, r3
	beq	.L3044
	ldrb	r8, [r4, #-638]	@ zero_extendqisi2
	sub	r3, r4, #3264
	ldr	r6, [r4, #-2816]
	cmp	r8, #0
	ldrh	r9, [r3, #-4]
	bne	.L3042
	sub	r6, r6, #1
	mov	r1, sp
	mov	r2, r8
	mov	r0, r6
	bl	log2phys
	ldr	r5, [r4, #-2976]
	ldr	r0, [r4, #-3008]
	ldr	r3, [sp]
	str	r6, [sp, #20]
	cmn	r3, #1
	str	r0, [sp, #12]
	str	r3, [sp, #8]
	str	r5, [sp, #16]
	str	r8, [r5, #4]
	beq	.L3046
	mov	r1, #1
	mov	r2, r8
	add	r0, sp, #4
	bl	FlashReadPages
	b	.L3047
.L3046:
	sub	r3, r4, #3184
	mov	r1, #255
	ldrh	r2, [r3, #-6]
	bl	ftl_memset
.L3047:
	ldr	r3, .L3061+4
	mov	r8, #0
	ldr	fp, .L3061
	mov	r9, r9, asl #2
	mov	r10, r8
	strh	r3, [r5]	@ movhi
.L3048:
	cmp	r8, r9
	beq	.L3049
	ldrh	r3, [r7, #4]
	ldr	r0, .L3061+8
	cmp	r3, #0
	beq	.L3049
	ldr	r3, [sp, #8]
	add	r8, r8, #1
	str	r6, [r5, #8]
	str	r3, [r5, #12]
	ldrh	r3, [r0]
	strh	r3, [r5, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [fp, #-3144]
	mov	r2, #0
	mov	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r5, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, r10
	str	r3, [fp, #-3144]
	mov	r3, r2
	bl	FlashProgPages
	ldrh	r0, [r7]
	bl	decrement_vpc_count
	b	.L3048
.L3049:
	mov	r3, #1
	b	.L3060
.L3044:
	mov	r3, #0
.L3060:
	strb	r3, [r4, #-638]
.L3042:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3062:
	.align	2
.L3061:
	.word	.LANCHOR2
	.word	-3947
	.word	.LANCHOR2-648
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
	ldr	r6, .L3069
	mov	r7, #12
	ldr	r5, .L3069+4
.L3064:
	ldrh	r3, [r5]
	uxth	r0, r4
	cmp	r3, r0
	bls	.L3068
	ldr	r3, [r6, #-2896]
	mla	r3, r7, r0, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L3065
	bl	flush_l2p_region
.L3065:
	add	r4, r4, #1
	b	.L3064
.L3068:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3070:
	.align	2
.L3069:
	.word	.LANCHOR2
	.word	.LANCHOR2-3162
	.fnend
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #40
	sub	sp, sp, #40
	ldr	r5, .L3088
	ldr	r3, [r5, #-2812]
	cmp	r3, #0
	bne	.L3071
	ldr	r2, .L3088+4
	ldrb	r6, [r2, #3904]	@ zero_extendqisi2
	cmp	r6, #0
	beq	.L3087
	ldrb	r6, [r0, #8]	@ zero_extendqisi2
	cmp	r6, #1
	ldreqh	r10, [r0, #4]
	beq	.L3073
	mov	r6, r3
.L3087:
	mov	r10, #12
.L3073:
	ldr	r7, [r5, #-2976]
	mvn	r3, #0
	str	r3, [sp, #20]
	mov	r8, #0
	ldr	r3, [r5, #-3008]
	movw	r2, #61589
	str	r7, [sp, #16]
	mov	r4, r0
	ldr	r9, .L3088
	str	r3, [sp, #12]
	mvn	r3, #2
	str	r3, [r7, #8]
	mvn	r3, #1
	str	r3, [r7, #12]
	ldrh	r3, [r0]
	strh	r8, [r7]	@ movhi
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, [r5, #-3008]
	str	r2, [r3]
	ldr	r2, .L3088+8
	ldr	r3, [r5, #-3008]
	str	r2, [r3, #4]
.L3074:
	subs	r10, r10, #1
	bcc	.L3077
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L3075
.L3077:
	ldrh	r3, [r4]
	ldr	r1, [r5, #-2940]
	ldrh	r0, [r4, #4]
	mov	r3, r3, asl #1
	ldrh	r2, [r1, r3]
	rsb	r2, r0, r2
	strh	r2, [r1, r3]	@ movhi
	ldr	r3, .L3088+12
	ldrh	r3, [r3]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	b	.L3071
.L3075:
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [sp, #8]
	beq	.L3077
	ldr	r3, [r5, #-3144]
	mov	r1, #1
	mov	r2, r6
	add	r0, sp, #4
	str	r3, [r7, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, r8
	str	r3, [r9, #-3144]
	mov	r3, #0
	bl	FlashProgPages
	ldrh	r0, [r4]
	bl	decrement_vpc_count
	b	.L3074
.L3071:
	add	sp, sp, #40
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L3089:
	.align	2
.L3088:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	305419896
	.word	.LANCHOR2-3200
	.fnend
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r5, #0
	ldr	r4, .L3106
	ldr	r1, .L3106+4
	ldr	r0, .L3106+8
	bl	printk
	sub	r3, r4, #3248
	ldr	r0, [r4, #-2944]
	mov	r1, #0
	ldrh	r2, [r3, #-10]
	mov	r6, r4
	mov	r2, r2, asl #1
	bl	ftl_memset
.L3091:
	ldr	r3, [r4, #-2816]
	cmp	r5, r3
	bcs	.L3104
	mov	r0, r5
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L3092
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r6, #-2944]
	mov	r0, r0, asl #1
	ldrh	r3, [r2, r0]
	add	r3, r3, #1
	strh	r3, [r2, r0]	@ movhi
.L3092:
	add	r5, r5, #1
	b	.L3091
.L3104:
	ldr	r6, .L3106
	mov	r7, #0
	ldr	r8, .L3106+12
.L3094:
	ldrh	r3, [r8]
	uxth	r4, r7
	cmp	r3, r4
	bls	.L3105
	ldr	r3, [r6, #-2940]
	mov	r5, r4, asl #1
	movw	r1, #65535
	ldrh	r2, [r3, r5]
	ldr	r3, [r6, #-2944]
	ldrh	r3, [r3, r5]
	cmp	r2, r3
	cmpne	r2, r1
	beq	.L3096
	ldr	r0, .L3106+16
	mov	r1, r4
	bl	printk
	ldr	r3, .L3106+20
	ldrh	r2, [r3]
	cmp	r2, r4
	beq	.L3096
	ldrh	r2, [r3, #96]
	cmp	r2, r4
	beq	.L3096
	ldrh	r3, [r3, #48]
	cmp	r3, r4
	beq	.L3096
	ldr	r3, [r6, #-2940]
	ldrh	r2, [r3, r5]
	cmp	r2, #0
	ldr	r2, [r6, #-2944]
	ldrh	r2, [r2, r5]
	strh	r2, [r3, r5]	@ movhi
	beq	.L3096
	mov	r0, r4
	bl	update_vpc_list
.L3096:
	add	r7, r7, #1
	b	.L3094
.L3105:
	bl	l2p_flush
	bl	FtlVpcTblFlush
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3107:
	.align	2
.L3106:
	.word	.LANCHOR2
	.word	.LANCHOR3+84
	.word	.LC48
	.word	.LANCHOR2-3260
	.word	.LC152
	.word	.LANCHOR2-648
	.fnend
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	allocate_new_data_superblock
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldr	r5, .L3135
	ldrh	r7, [r0]
	ldr	r3, [r5, #-2812]
	cmp	r3, #0
	bne	.L3109
	movw	r3, #65535
	cmp	r7, r3
	mov	r4, r0
	beq	.L3110
	ldr	r2, [r5, #-2940]
	mov	r3, r7, asl #1
	mov	r0, r7
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3111
	bl	INSERT_DATA_LIST
	b	.L3110
.L3111:
	bl	INSERT_FREE_LIST
.L3110:
	mov	r3, #0
	strb	r3, [r4, #8]
	ldr	r3, .L3135
	sub	r2, r3, #600
	cmp	r4, r2
	beq	.L3112
	sub	r2, r3, #3248
	ldrh	r2, [r2]
	cmp	r2, #1
	beq	.L3112
	ldr	r1, .L3135+4
	ldrb	r1, [r1, #3904]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L3113
.L3112:
	mov	r3, #1
	strb	r3, [r4, #8]
	b	.L3114
.L3113:
	sub	r1, r3, #648
	cmp	r4, r1
	bne	.L3114
	cmp	r2, #3
	beq	.L3116
	ldr	r3, [r3, #-3152]
	cmp	r3, #1
	bne	.L3117
.L3116:
	mov	r3, #1
	strb	r3, [r5, #-640]
.L3117:
	ldr	r2, [r5, #-3336]
	ldr	r3, .L3135
	cmp	r2, #0
	beq	.L3114
	ldr	r2, [r3, #-3100]
	cmp	r2, #39
	movls	r2, #1
	strlsb	r2, [r3, #-640]
.L3114:
	ldr	r3, .L3135+8
	movw	r2, #65535
	ldrh	r0, [r3, #216]
	mov	r6, r3
	cmp	r0, r2
	beq	.L3119
	cmp	r7, r0
	bne	.L3120
	ldr	r2, [r5, #-2940]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3121
.L3120:
	bl	update_vpc_list
.L3121:
	mvn	r3, #0
	strh	r3, [r6, #216]	@ movhi
.L3119:
	mov	r0, r4
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3109:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3136:
	.align	2
.L3135:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR4
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #40
	sub	sp, sp, #40
	ldr	r7, .L3154
	ldr	r5, .L3154+4
	ldr	r10, [r7, #-240]
	mov	r6, r7
	cmp	r10, #0
	beq	.L3138
	ldr	r2, [r7, #-236]
	ldr	r3, [r5, #-2816]
	cmp	r2, r3
	bcs	.L3139
	mov	r4, #2048
.L3144:
	ldr	r0, [r6, #-236]
	ldr	r3, [r5, #-2816]
	cmp	r0, r3
	bcs	.L3143
	mov	r2, #0
	mov	r1, sp
	bl	log2phys
	ldr	r3, [r7, #-236]
	add	r3, r3, #1
	str	r3, [r7, #-236]
	ldr	r2, [sp]
	cmn	r2, #1
	beq	.L3142
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
	bne	.L3143
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L3143:
	mvn	r0, #0
	b	.L3146
.L3142:
	subs	r4, r4, #1
	bne	.L3144
	b	.L3143
.L3139:
	ldr	r3, [r5, #-3120]
	mov	r0, #0
	str	r0, [r7, #-240]
	str	r0, [r7, #-236]
	str	r3, [r7, #-244]
	b	.L3146
.L3138:
	ldr	r1, [r5, #-3104]
	movw	r4, #10000
	ldr	r9, [r5, #-3120]
	sub	r5, r5, #716
	cmp	r1, r4
	ldr	r8, [r7, #-244]
	add	r2, r9, #1048576
	ldr	r3, [r5, #-2100]
	movhi	r4, #31
	movls	r4, #63
	cmp	r8, r2
	bhi	.L3148
	mov	r1, r1, lsr #10
	mov	r0, #1000
	mul	r0, r0, r3
	add	r1, r1, #1
	bl	__aeabi_uidiv
	add	r0, r0, r8
	cmp	r0, r9
	bcc	.L3148
	ldrh	r3, [r5, #28]
	ands	r0, r4, r3
	movne	r0, r10
	bne	.L3146
	ldr	r2, [r7, #-220]
	cmp	r2, r3
	beq	.L3146
.L3148:
	ldrh	r3, [r5, #28]
	mov	r0, #0
	str	r9, [r6, #-244]
	str	r0, [r6, #-236]
	str	r3, [r6, #-220]
	mov	r3, #1
	str	r3, [r6, #-240]
.L3146:
	add	sp, sp, #40
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L3155:
	.align	2
.L3154:
	.word	.LANCHOR4
	.word	.LANCHOR2
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	ftl_do_gc
	.type	ftl_do_gc, %function
ftl_do_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r5, .L3320
	ldr	r6, [r5, #-2812]
	cmp	r6, #0
	movne	r0, #0
	bne	.L3309
	ldr	r3, .L3320+4
	ldr	r2, [r3, #3284]
	cmn	r2, #1
	beq	.L3252
	ldr	r8, [r5, #-3096]
	cmp	r8, #0
	bne	.L3252
	sub	r2, r5, #660
	ldrh	r2, [r2]
	cmp	r2, #47
	movls	r0, r8
	bls	.L3309
	movw	r2, #3288
	mov	r7, r5
	ldrh	r2, [r3, r2]
	movw	r3, #65535
	mov	r9, r1
	str	r0, [sp, #20]
	cmp	r2, r3
	bne	.L3158
.L3161:
	ldr	r5, .L3320+8
	movw	r1, #65535
	ldrh	r4, [r5, #-14]
	cmp	r4, r1
	bne	.L3159
	b	.L3160
.L3158:
	sub	r5, r5, #552
	ldrh	r2, [r5]
	cmp	r2, r3
	beq	.L3161
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	beq	.L3161
	mov	r0, #1
	b	.L3309
.L3159:
	mov	r3, r5
	ldrh	r2, [r3, #-16]!
	cmp	r2, r1
	bne	.L3160
	ldrh	r0, [r5, #-12]
	cmp	r0, r2
	beq	.L3160
	ldrh	r1, [r5, #-10]
	cmp	r1, r2
	strneh	r4, [r3]	@ movhi
	strneh	r0, [r5, #-14]	@ movhi
	mvnne	r3, #0
	strneh	r1, [r5, #-12]	@ movhi
	strneh	r3, [r5, #-10]	@ movhi
.L3160:
	ldr	r2, [sp, #20]
	ldr	r3, [r7, #-3080]
	cmp	r2, #1
	ldr	r6, .L3320+12
	add	r3, r3, #1
	add	r3, r3, r2, asl #7
	str	r3, [r7, #-3080]
	bne	.L3172
	ldr	r2, .L3320
	ldr	r2, [r2, #-3336]
	cmp	r2, #0
	bne	.L3163
	ldr	r2, .L3320+16
	ldrb	r2, [r2, #3904]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3172
.L3163:
	ldr	r2, [r7, #-3100]
	ldr	r4, .L3320
	cmp	r2, #39
	bhi	.L3172
	ldr	r8, .L3320+20
	ldrh	r2, [r8]
	add	r3, r3, r2
	str	r3, [r4, #-3080]
	bl	FtlGcReFreshBadBlk
	ldrh	r1, [r6]
	movw	r3, #65535
	cmp	r1, r3
	bne	.L3172
	sub	r3, r4, #3072
	ldrh	r3, [r3]
	cmp	r3, r1
	bne	.L3172
	ldr	r3, [r4, #-3080]
	cmp	r3, #1024
	sub	r3, r4, #652
	bhi	.L3164
	ldrh	r2, [r3]
	cmp	r2, #63
	bhi	.L3172
.L3164:
	ldrh	ip, [r3]
	mov	r1, #0
	ldrh	r3, [r5, #-6]
	ldr	r0, .L3320+20
	add	r3, r3, #64
	strh	r1, [r8]	@ movhi
	cmp	ip, r3
	ldr	r2, .L3320
	bgt	.L3172
	ldr	r3, [r2, #-3100]
	str	r1, [r2, #-3080]
	cmp	r3, r1
	moveq	r3, #6
	beq	.L3311
	cmp	r3, #5
	bhi	.L3166
	mov	r3, #18
.L3311:
	strh	r3, [r0]	@ movhi
.L3166:
	mov	r0, #32
	bl	List_get_gc_head_node
	movw	ip, #65535
	uxth	r2, r0
	cmp	r2, ip
	beq	.L3171
	ldrh	r0, [r5, #-4]
	ldr	r8, .L3320
	cmp	r0, #0
	sub	r10, r8, #3056
	beq	.L3168
	sub	r1, r8, #3184
	ldr	r3, [r8, #-2940]
	mov	r2, r2, asl #1
	ldrh	r4, [r1, #-14]
	sub	r1, r8, #3264
	ldrh	lr, [r3, r2]
	ldrh	r1, [r1, #-4]
	mul	r1, r1, r4
	add	r1, r1, #1
	cmp	lr, r1
	bgt	.L3171
	add	r1, r0, #1
	str	r2, [sp, #28]
	str	ip, [sp, #24]
	mov	fp, #0
	uxth	r1, r1
	str	r3, [sp, #16]
	strh	r1, [r10, #-4]	@ movhi
	str	r1, [sp, #12]
	str	fp, [r8, #-3092]
	bl	List_get_gc_head_node
	ldr	ip, [sp, #24]
	uxth	r4, r0
	ldr	r1, [sp, #12]
	cmp	r4, ip
	ldr	r3, [sp, #16]
	ldr	r2, [sp, #28]
	beq	.L3171
	mov	ip, r4, asl #1
	ldr	r0, .L3320+24
	str	ip, [sp, #12]
	ldrh	lr, [r3, ip]
	ldrh	r3, [r3, r2]
	mov	r2, r4
	str	r3, [sp]
	mov	r3, lr
	bl	printk
	ldrh	r3, [r10, #-4]
	cmp	r3, #40
	ldr	ip, [sp, #12]
	bls	.L3169
	ldr	r3, [r8, #-2940]
	ldrh	r3, [r3, ip]
	cmp	r3, #32
	strhih	fp, [r10, #-4]	@ movhi
.L3169:
	ldr	r3, .L3320+20
	mov	r2, #6
	strh	r2, [r3]	@ movhi
	b	.L3173
.L3168:
	mov	r3, #1
	strh	r3, [r10, #-4]	@ movhi
.L3171:
	bl	GetSwlReplaceBlock
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	bne	.L3173
	ldr	r3, .L3320+20
	mov	r2, #0
	strh	r2, [r3]	@ movhi
.L3172:
	ldrh	r3, [r6]
	movw	r4, #65535
	cmp	r3, r4
	bne	.L3173
	ldr	r8, .L3320
	sub	r2, r8, #552
	ldrh	r4, [r2]
	cmp	r4, r3
	movne	r4, r3
	beq	.L3316
.L3173:
	movw	r0, #65535
	rsb	r3, r0, r4
	clz	r3, r3
	ldr	r2, [sp, #20]
	mov	r3, r3, lsr #5
	cmp	r2, #0
	movne	ip, #0
	andeq	ip, r3, #1
	cmp	ip, #0
	beq	.L3186
	ldr	r3, .L3320+28
	ldrh	r2, [r3]
	cmp	r2, #24
	movhi	fp, #1
	bhi	.L3187
	ldr	r3, .L3320+32
	cmp	r2, #16
	ldrhih	r3, [r3]
	movhi	fp, r3, lsr #5
	bhi	.L3187
	cmp	r2, #12
	ldrhih	r3, [r3]
	movhi	fp, r3, lsr #4
	bhi	.L3187
	cmp	r2, #8
	ldrhih	r3, [r3]
	ldrlsh	fp, [r3]
	movhi	fp, r3, lsr #2
.L3187:
	ldrh	r3, [r5, #-8]
	ldr	r1, .L3320
	cmp	r3, r2
	bcs	.L3191
	sub	r3, r1, #552
	movw	r2, #65535
	ldrh	r3, [r3]
	cmp	r3, r2
	bne	.L3192
	sub	r2, r1, #3072
	ldrh	r2, [r2]
	cmp	r2, r3
	bne	.L3192
	ldr	r3, .L3320+20
	ldrh	r0, [r3]
	cmp	r0, #0
	bne	.L3193
	ldr	r3, [r1, #-2816]
	ldr	r2, [r1, #-408]
	add	r3, r3, r3, asl #1
	cmp	r2, r3, lsr #2
	movcs	r3, #18
	bcs	.L3194
.L3193:
	ldr	r3, .L3320+36
	ldrh	r3, [r3]
	add	r3, r3, r3, asl #1
	ubfx	r3, r3, #2, #16
.L3194:
	strh	r3, [r5, #-8]	@ movhi
	mov	r3, #0
	str	r3, [r7, #-3092]
	b	.L3309
.L3192:
	ldr	r3, .L3320+36
	ldrh	r3, [r3]
	add	r3, r3, r3, asl #1
	mov	r3, r3, asr #2
	strh	r3, [r5, #-8]	@ movhi
.L3191:
	cmp	r9, #2
	ldr	r3, [r7, #-3336]
	movw	r4, #65535
	movhi	r9, #0
	movls	r9, #1
	cmp	r3, #0
	moveq	r9, #0
	cmp	r9, #0
	addne	r3, fp, #1
	uxthne	fp, r3
	b	.L3196
.L3186:
	ldr	r1, .L3320
	sub	r2, r1, #552
	ldrh	r2, [r2]
	cmp	r2, r0
	bne	.L3197
	sub	r0, r1, #3072
	ldrh	r0, [r0]
	cmp	r0, r2
	movne	r3, #0
	andeq	r3, r3, #1
	cmp	r3, #0
	beq	.L3197
	ldrh	r3, [r6]
	cmp	r3, r2
	beq	.L3198
.L3202:
	movw	r4, #65535
	b	.L3197
.L3198:
	sub	r4, r1, #652
	ldrh	r3, [r5, #-8]
	str	ip, [r1, #-3092]
	ldrh	r2, [r4]
	ldr	r9, .L3320+20
	cmp	r2, r3
	bls	.L3199
	ldrh	r3, [r9]
	cmp	r3, #0
	bne	.L3200
	ldr	r3, [r1, #-2816]
	ldr	r2, [r1, #-408]
	add	r3, r3, r3, asl #1
	cmp	r2, r3, lsr #2
	movcs	r3, #18
	bcs	.L3201
.L3200:
	ldr	r3, .L3320+36
	ldrh	r3, [r3]
	add	r3, r3, r3, asl #1
	ubfx	r3, r3, #2, #16
.L3201:
	strh	r3, [r5, #-8]	@ movhi
	bl	FtlReadRefresh
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, [r7, #-2940]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	cmp	r3, #4
	ldrhih	r0, [r9]
	bhi	.L3309
.L3199:
	ldrh	r0, [r9]
	ldr	r8, .L3320+20
	cmp	r0, #0
	bne	.L3202
	sub	r3, r8, #780
	ldrh	r9, [r3]
	add	r3, r9, r9, asl #1
	mov	r3, r3, asr #2
	strh	r3, [r5, #-8]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r7, #-2940]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	ldr	r3, .L3320+40
	ldrh	r1, [r3, #-14]
	ldrh	r5, [r3, #-84]
	mul	r5, r5, r1
	add	r5, r5, r5, lsr #31
	cmp	r2, r5, asr #1
	ble	.L3203
	ldrh	r1, [r4]
	sub	r3, r9, #1
	cmp	r1, r3
	blt	.L3203
	bl	FtlReadRefresh
	ldrh	r0, [r8]
	b	.L3309
.L3203:
	cmp	r2, #0
	bne	.L3202
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r0, [r4]
	add	r0, r0, #1
	b	.L3309
.L3197:
	ldr	r3, [r7, #-3336]
	cmp	r3, #0
	moveq	fp, #1
	movne	fp, #2
.L3196:
	ldrh	r3, [r6]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L3205
	cmp	r4, r3
	ldrne	r3, .L3320+12
	strneh	r4, [r3]	@ movhi
	bne	.L3207
	ldr	r1, .L3320
	sub	r3, r1, #3072
	ldrh	r2, [r3]
	cmp	r2, r4
	beq	.L3207
	ldr	r1, [r1, #-2940]
	mov	r2, r2, asl #1
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	mvneq	r2, #0
	streqh	r2, [r3]	@ movhi
	ldrh	r2, [r3]
	strh	r2, [r6]	@ movhi
	mvn	r2, #0
	strh	r2, [r3]	@ movhi
.L3207:
	ldr	r3, .L3320+44
	mov	r2, #0
	ldrh	r0, [r6]
	sub	r8, r3, #368
	strb	r2, [r3, #-360]
	movw	r2, #65535
	cmp	r0, r2
	mov	r5, r3
	beq	.L3205
	bl	IsBlkInGcList
	cmp	r0, #0
	mvnne	r3, #0
	strneh	r3, [r8]	@ movhi
	ldr	r3, .L3320+16
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3211
	ldrh	r0, [r6]
	bl	ftl_get_blk_mode
	strb	r0, [r5, #-360]
.L3211:
	ldrh	r2, [r6]
	movw	r3, #65535
	ldr	r5, .L3320+44
	cmp	r2, r3
	sub	r8, r5, #368
	beq	.L3205
	mov	r0, r8
	bl	make_superblock
	movw	r2, #410
	mov	r3, #0
	strh	r3, [r5, r2]	@ movhi
	strh	r3, [r8, #2]	@ movhi
	strb	r3, [r5, #-362]
	ldrh	r3, [r8]
	ldr	r2, [r7, #-2940]
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r3, r5, #412
	strh	r2, [r3]	@ movhi
.L3205:
	ldr	r2, .L3320+48
	ldrh	r3, [r6]
	ldrh	r1, [r2]
	cmp	r1, r3
	beq	.L3212
	ldrh	r1, [r2, #48]
	cmp	r1, r3
	beq	.L3212
	ldrh	r2, [r2, #96]
	cmp	r2, r3
	bne	.L3213
.L3212:
	mvn	r3, #0
	strh	r3, [r6]	@ movhi
.L3213:
	ldr	r5, .L3320+44
.L3246:
	ldr	r8, .L3320+12
	movw	r3, #65535
	ldr	r9, .L3320
	ldrh	r2, [r8]
	mov	r10, r8
	cmp	r2, r3
	bne	.L3214
	mov	r3, #0
	str	r3, [r9, #-3092]
.L3215:
	ldr	r6, .L3320+52
	ldrh	r7, [r6]
	mov	r0, r7
	bl	List_get_gc_head_node
	movw	r2, #65535
	uxth	r3, r0
	strh	r3, [r10]	@ movhi
	cmp	r3, r2
	moveq	r3, #0
	moveq	r0, #8
	streqh	r3, [r6]	@ movhi
	beq	.L3309
.L3216:
	mov	r0, r3
	str	r3, [sp, #12]
	bl	IsBlkInGcList
	add	r7, r7, #1
	cmp	r0, #0
	ldr	r3, [sp, #12]
	strneh	r7, [r6]	@ movhi
	bne	.L3215
	ldr	r2, .L3320
	mov	r1, r3, asl #1
	ldrh	lr, [r6, #-208]
	uxth	r7, r7
	strh	r7, [r6]	@ movhi
	ldr	r0, [r2, #-2940]
	ldrh	r2, [r6, #-140]
	ldrh	ip, [r0, r1]
	mul	r2, lr, r2
	add	lr, r2, r2, lsr #31
	cmp	ip, lr, asr #1
	bgt	.L3219
	cmp	ip, #8
	cmphi	r7, #48
	bls	.L3220
	ldrh	ip, [r5, #232]
	cmp	ip, #35
	bhi	.L3220
.L3219:
	mov	ip, #0
	strh	ip, [r6]	@ movhi
.L3220:
	ldrh	r1, [r0, r1]
	movw	r0, #65535
	cmp	r1, r2
	cmpge	r4, r0
	bne	.L3221
	ldrh	r2, [r6]
	cmp	r2, #3
	bhi	.L3221
	mvn	r3, #0
	strh	r3, [r10]	@ movhi
	ldr	r3, .L3320+52
	mov	r2, #0
	strh	r2, [r3]	@ movhi
	ldr	r3, .L3320+20
	b	.L3314
.L3221:
	cmp	r1, #0
	bne	.L3222
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r6]
	add	r3, r3, #1
	strh	r3, [r6]	@ movhi
	b	.L3215
.L3222:
	ldr	r1, .L3320+44
	mov	r2, #0
	strb	r2, [r1, #-360]
	ldr	r2, .L3320+16
	ldrb	r2, [r2, #3904]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3223
	mov	r0, r3
	bl	ftl_get_blk_mode
	strb	r0, [r5, #-360]
.L3223:
	ldr	r0, .L3320+12
	bl	make_superblock
	ldrh	r2, [r8]
	ldr	r1, .L3320+56
	mov	r3, #0
	ldr	r0, [r9, #-2940]
	mov	r2, r2, asl #1
	strh	r3, [r1]	@ movhi
	ldrh	r2, [r0, r2]
	strh	r3, [r8, #2]	@ movhi
	strh	r2, [r1, #2]	@ movhi
	ldr	r2, .L3320+44
	strb	r3, [r2, #-362]
.L3214:
	ldr	r3, [sp, #20]
	cmp	r3, #1
	bne	.L3224
	bl	FtlReadRefresh
.L3224:
	mov	r3, #1
	str	r3, [r9, #-3096]
	ldr	r3, .L3320+32
	ldrh	r2, [r3]
	str	r2, [sp, #12]
	ldr	r2, .L3320+16
	ldrb	r2, [r2, #3904]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3225
	ldrb	r2, [r5, #-360]	@ zero_extendqisi2
	cmp	r2, #1
	ldreqh	r3, [r3, #2]
	streq	r3, [sp, #12]
.L3225:
	ldrh	r3, [r8, #2]
	ldr	r1, [sp, #12]
	add	r2, r3, fp
	ldr	r6, .L3320
	cmp	r2, r1
	movgt	r2, r1
	rsbgt	r3, r3, r2
	uxthgt	fp, r3
	mov	r3, #0
	str	r3, [sp, #16]
	b	.L3227
.L3319:
	ldr	r0, [r6, #-3020]
	mov	r1, r7
	ldrb	r2, [r5, #-360]	@ zero_extendqisi2
	mov	r9, #0
	bl	FlashReadPages
.L3230:
	uxth	r3, r9
	cmp	r3, r7
	bcs	.L3317
	mov	r3, #36
	ldr	r2, [r6, #-3020]
	mul	r8, r3, r9
	add	r1, r2, r8
	ldr	r2, [r2, r8]
	ldr	r10, [r1, #12]
	cmn	r2, #1
	beq	.L3266
	ldrh	r1, [r10]
	movw	r2, #61589
	cmp	r1, r2
	bne	.L3266
	add	r1, sp, #32
	mov	r2, #0
	ldr	r0, [r10, #8]
	str	r3, [sp, #24]
	bl	log2phys
	ldr	r2, [r6, #-3020]
	add	r2, r2, r8
	ldr	r0, [r2, #4]
	ldr	r1, [sp, #32]
	ldr	r3, [sp, #24]
	bic	r1, r1, #-2147483648
	cmp	r1, r0
	bne	.L3266
	ldr	r1, .L3320+56
	ldr	r0, .L3320+56
	ldr	r2, [r2, #16]
	ldrh	r1, [r1]
	str	r3, [sp, #28]
	add	r1, r1, #1
	strh	r1, [r0]	@ movhi
	ldr	r0, [r5, #208]
	ldr	r1, [r6, #-3032]
	mla	r1, r3, r0, r1
	str	r2, [r1, #16]
	str	r1, [sp, #24]
	bl	Ftl_get_new_temp_ppa
	ldr	r2, [r6, #-3032]
	ldr	r1, [sp, #24]
	ldr	r3, [sp, #28]
	str	r0, [r1, #4]
	ldr	r1, [r5, #208]
	mla	r3, r3, r1, r2
	ldr	r2, [r6, #-3020]
	add	r2, r2, r8
	ldr	r1, [r2, #8]
	str	r1, [r3, #8]
	mov	r1, #1
	ldr	r2, [r2, #12]
	str	r2, [r3, #12]
	ldr	r3, [sp, #32]
	str	r3, [r10, #12]
	ldr	r3, .L3320+60
	ldrh	r2, [r3]
	str	r3, [sp, #24]
	strh	r2, [r10, #2]	@ movhi
	ldr	r2, [r6, #-3144]
	ldr	r0, [r6, #-3020]
	str	r2, [r10, #4]
	add	r0, r0, r8
	ldr	r2, [r5, #208]
	add	r2, r2, #1
	str	r2, [r5, #208]
	bl	FtlGcBufAlloc
	ldr	r2, .L3320+16
	ldrb	r2, [r2, #3904]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3318
.L3232:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	beq	.L3266
	ldr	r3, .L3320
	mov	r2, #0
	mvn	r1, #0
	str	r2, [r3, #-3096]
	ldr	r3, .L3320+12
	strh	r1, [r3]	@ movhi
	strh	r2, [r3, #2]	@ movhi
	add	r3, r3, #776
.L3314:
	ldrh	r0, [r3]
	b	.L3309
.L3317:
	ldr	r3, [sp, #16]
	add	r3, r3, #1
	str	r3, [sp, #16]
.L3227:
	ldrh	r3, [sp, #16]
	ldr	r8, .L3320
	cmp	r3, fp
	ldr	r7, .L3320+12
	bcs	.L3234
	ldr	r3, .L3320+64
	mov	r2, #0
	ldrh	r0, [r7, #2]
	movw	lr, #65535
	ldr	r8, [r6, #-3020]
	mov	ip, #36
	ldrh	r9, [r3]
	ldr	r3, [sp, #16]
	add	r0, r0, r3
	add	r3, r7, #14
	mov	r7, r2
.L3235:
	uxth	r1, r2
	cmp	r1, r9
	bcs	.L3319
	ldrh	r1, [r3, #2]!
	add	r2, r2, #1
	cmp	r1, lr
	orrne	r1, r0, r1, asl #10
	mlane	r10, ip, r7, r8
	addne	r7, r7, #1
	uxthne	r7, r7
	strne	r1, [r10, #4]
	b	.L3235
.L3318:
	ldrb	r2, [r6, #-545]	@ zero_extendqisi2
	ldr	r1, [r5, #208]
	cmp	r1, r2
	beq	.L3232
	ldr	r3, [sp, #24]
	ldrh	r3, [r3, #4]
	cmp	r3, #0
	beq	.L3232
.L3266:
	add	r9, r9, #1
	b	.L3230
.L3234:
	ldrh	r3, [r7, #2]
	ldr	r2, [sp, #12]
	add	r3, fp, r3
	uxth	r3, r3
	strh	r3, [r7, #2]	@ movhi
	cmp	r3, r2
	bcc	.L3236
	ldr	r3, [r5, #208]
	cmp	r3, #0
	beq	.L3237
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	addne	r7, r7, #776
	movne	r3, #0
	strne	r3, [r8, #-3096]
	ldrneh	r0, [r7]
	bne	.L3309
.L3237:
	ldr	r3, .L3320+56
	ldrh	r6, [r3]
	cmp	r6, #0
	bne	.L3238
	ldrh	r3, [r7]
	ldr	r2, [r8, #-2940]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3238
.L3239:
	ldr	r3, [r8, #-2816]
	cmp	r6, r3
	bcs	.L3244
	mov	r0, r6
	add	r1, sp, #36
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #36]
	cmn	r0, #1
	beq	.L3240
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r7]
	cmp	r3, r0
	bne	.L3240
.L3244:
	ldr	r3, [r8, #-2816]
	cmp	r6, r3
	bcc	.L3238
	ldr	r2, .L3320
	mov	r1, #0
	ldrh	r3, [r7]
	ldr	r2, [r2, #-2940]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r7]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L3238
.L3240:
	add	r6, r6, #1
	b	.L3239
.L3238:
	mvn	r3, #0
	strh	r3, [r7]	@ movhi
.L3236:
	ldr	r7, .L3320
	sub	r3, r7, #652
	ldrh	r3, [r3]
	cmp	r3, #2
	subls	r7, r7, #3200
	ldrlsh	fp, [r7]
	bls	.L3246
.L3245:
	mov	r2, #0
	str	r2, [r7, #-3096]
	ldr	r2, .L3320+20
	ldrh	r0, [r2]
	cmp	r0, #0
	addeq	r0, r3, #1
	b	.L3309
.L3252:
	mov	r0, r6
	b	.L3309
.L3316:
	sub	r3, r8, #3072
	ldrh	fp, [r3]
	cmp	fp, r4
	bne	.L3173
	sub	r10, r8, #652
	ldr	r2, [r8, #-3080]
	ldrh	r3, [r10]
	cmp	r3, #24
	movcc	r3, #5120
	movcs	r3, #1024
	cmp	r2, r3
	movls	r4, fp
	bls	.L3173
	ldr	r2, .L3320+20
	mov	r3, #0
	str	r3, [r7, #-3080]
	strh	r3, [r2]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, fp
	mov	r4, r0
	bne	.L3175
	ldrh	r2, [r5, #-6]
	sub	r3, r8, #3056
	ldrh	r1, [r10]
	cmp	r1, r2
	movcs	r2, #80
	strcsh	r2, [r3, #-6]	@ movhi
	bcs	.L3185
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r4
	beq	.L3185
	ldr	r2, [r8, #-3156]
	sub	r1, r8, #3248
	cmp	r2, #0
	bne	.L3178
	ldrh	r2, [r1]
	cmp	r2, #3
	beq	.L3178
	ldr	r2, [r8, #-3152]
	cmp	r2, #0
	bne	.L3178
	ldr	r2, [r8, #-3336]
	cmp	r2, #0
	bne	.L3178
	ldr	r2, .L3320+16
	ldrb	r0, [r2, #3904]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L3179
.L3178:
	ldr	r2, [r7, #-2940]
	mov	r3, r3, asl #1
	ldrh	r1, [r1]
	ldrh	r0, [r2, r3]
	cmp	r1, #3
	ldr	r2, .L3320+40
	ldrh	r3, [r2, #-14]
	ldrh	r2, [r2, #-84]
	mul	r2, r2, r3
	moveq	r3, r3, lsr #1
	movne	r3, #0
	add	r3, r2, r3
	cmp	r0, r3
	bgt	.L3181
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, [r7, #-2816]
	ldr	r2, [r7, #-408]
	add	r3, r3, r3, asl #1
	cmp	r2, r3, lsr #2
	movls	r3, #160
	uxth	r4, r0
	bls	.L3312
	b	.L3315
.L3179:
	ldr	r2, [r8, #-2940]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #7
	bhi	.L3184
	bl	List_get_gc_head_node
	uxth	r4, r0
.L3315:
	mov	r3, #128
.L3312:
	strh	r3, [r5, #-6]	@ movhi
	movw	r3, #65535
	cmp	r4, r3
	beq	.L3185
	b	.L3175
.L3181:
	mov	r3, #128
	b	.L3313
.L3184:
	mov	r3, #64
.L3313:
	strh	r3, [r5, #-6]	@ movhi
	b	.L3185
.L3175:
	ldr	r0, [r7, #-2948]
	mov	r1, r4, asl #1
	ldr	r3, [r7, #-2940]
	ldrh	r2, [r10]
	ldrh	r3, [r3, r1]
	ldrh	r1, [r0, r1]
	ldr	r0, .L3320+68
	str	r1, [sp]
	ldrh	r1, [r5, #-8]
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L3185:
	bl	FtlGcReFreshBadBlk
	b	.L3173
.L3309:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3321:
	.align	2
.L3320:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR2-3056
	.word	.LANCHOR4-368
	.word	.LANCHOR0
	.word	.LANCHOR4+408
	.word	.LC153
	.word	.LANCHOR2-652
	.word	.LANCHOR2-3200
	.word	.LANCHOR4-372
	.word	.LANCHOR2-3184
	.word	.LANCHOR4
	.word	.LANCHOR2-648
	.word	.LANCHOR2-3060
	.word	.LANCHOR4+410
	.word	.LANCHOR2-552
	.word	.LANCHOR2-3268
	.word	.LC154
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
	ldr	r4, .L3366
	ldr	r8, .L3366+4
	ldr	r3, [r4, #-2812]
	ldr	r5, [r8, #416]
	cmp	r3, #0
	bne	.L3324
	ldr	r1, [r4, #-3048]
	cmp	r1, #0
	beq	.L3324
	ldr	r3, .L3366+8
	mov	r7, #0
	ldr	r0, [r4, #-3016]
	mov	r10, #36
	ldr	r9, .L3366
	ldrb	r6, [r3, #3904]	@ zero_extendqisi2
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	cmp	r6, #0
	ldrneb	r6, [r5, #8]	@ zero_extendqisi2
	subne	r6, r6, #1
	clzne	r6, r6
	movne	r6, r6, lsr #5
	mov	r2, r6
	bl	FlashProgPages
.L3327:
	ldr	r3, [r4, #-3048]
	cmp	r7, r3
	bcs	.L3345
	mul	fp, r10, r7
	ldr	r3, [r9, #-3016]
	add	r2, r3, fp
	ldr	r3, [r3, fp]
	cmn	r3, #1
	beq	.L3348
	ldr	r3, [r2, #4]
	cmp	r6, #0
	ldr	r0, [r2, #16]
	add	r1, sp, #4
	mov	r2, #1
	orrne	r3, r3, #-2147483648
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r4, #-3016]
	add	r3, r3, fp
	ldr	r3, [r3, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L3331
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r9, #-2940]
	mov	r3, r0, asl #1
	mov	fp, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L3332
	ldr	r0, .L3366+12
	mov	r1, fp
	bl	printk
.L3332:
	mov	r0, fp
	bl	decrement_vpc_count
.L3331:
	add	r7, r7, #1
	b	.L3327
.L3364:
	ldr	r6, .L3366+16
	movw	r5, #16386
.L3344:
	ldrh	r3, [r6]
	cmp	r3, #0
	beq	.L3345
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r5, r5, #1
	bne	.L3344
.L3345:
	mov	r3, #0
	str	r3, [r4, #-3048]
	b	.L3324
.L3348:
	ldr	r9, .L3366
	mov	r10, #0
.L3328:
	ldr	r3, [r4, #-3048]
	cmp	r10, r3
	bcs	.L3364
	mov	r7, #36
	ldr	r3, [r9, #-3016]
	mul	r7, r7, r10
	mov	fp, #0
	mvn	r2, #0
	str	r2, [r3, r7]
.L3334:
	ldr	r3, [r4, #-3016]
	add	r2, r3, r7
	ldr	r3, [r3, r7]
	cmn	r3, #1
	bne	.L3365
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r5]
	cmp	r3, r0
	bne	.L3335
	ldr	r1, [r9, #-2940]
	mov	r3, r3, asl #1
	ldrh	r0, [r5, #4]
	ldrh	r2, [r1, r3]
	rsb	r2, r0, r2
	strh	r2, [r1, r3]	@ movhi
	ldr	r3, .L3366+20
	strb	fp, [r5, #6]
	strh	fp, [r5, #4]	@ movhi
	ldrh	r3, [r3]
	strh	r3, [r5, #2]	@ movhi
.L3335:
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	bne	.L3336
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3336:
	ldr	r3, [r8, #-224]
	add	r3, r3, #1
	str	r3, [r8, #-224]
	ldr	r3, [r4, #-3016]
	add	r3, r3, r7
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlGcMarkBadPhyBlk
	mov	r0, r5
	bl	get_new_active_ppa
	ldr	r3, [r4, #-3016]
	mov	r1, #1
	mov	r2, r6
	add	r3, r3, r7
	str	r0, [sp, #4]
	str	r0, [r3, #4]
	mov	r0, r3
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	bl	FlashProgPages
	ldr	r3, [r4, #-3016]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	moveq	r3, #1
	streq	r3, [r9, #-2812]
	ldr	r3, [r4, #-2812]
	cmp	r3, #0
	beq	.L3334
	b	.L3324
.L3365:
	ldr	r3, [r2, #4]
	cmp	r6, #0
	ldr	r0, [r2, #16]
	add	r1, sp, #4
	mov	r2, #1
	orrne	r3, r3, #-2147483648
	str	r3, [sp, #4]
	bl	log2phys
	ldr	r3, [r4, #-3016]
	add	r7, r3, r7
	ldr	r3, [r7, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L3341
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r9, #-2940]
	mov	r3, r0, asl #1
	mov	r7, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L3342
	ldr	r0, .L3366+12
	mov	r1, r7
	bl	printk
.L3342:
	mov	r0, r7
	bl	decrement_vpc_count
.L3341:
	add	r10, r10, #1
	b	.L3328
.L3324:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3367:
	.align	2
.L3366:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LC155
	.word	.LANCHOR2-3058
	.word	.LANCHOR2-3200
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
	ldr	r3, .L3374
	ldr	r3, [r3, #-2812]
	cmp	r3, #0
	bne	.L3369
	ldr	r3, .L3374+4
	ldr	r3, [r3, #3284]
	cmn	r3, #1
	beq	.L3369
	bl	FtlCacheWriteBack
	bl	l2p_flush
	mov	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3369:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L3375:
	.align	2
.L3374:
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
	ldr	r3, .L3379
	ldr	r3, [r3, #3284]
	cmp	r3, #1
	bne	.L3377
	bl	FtlSysFlush
.L3377:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L3380:
	.align	2
.L3379:
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
	.global	ftl_discard
	.type	ftl_discard, %function
ftl_discard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3402
	add	r2, r0, r1
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r6, r0
	ldr	r3, [r3, #3908]
	mov	r5, r1
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L3385
	ldr	r3, .L3402+4
	ldr	r0, [r3, #3284]
	cmn	r0, #1
	beq	.L3385
	cmp	r1, #31
	bhi	.L3386
.L3387:
	mov	r0, #0
	b	.L3385
.L3386:
	ldr	r4, .L3402+8
	ldr	r3, [r4, #-2812]
	cmp	r3, #0
	bne	.L3387
	sub	r4, r4, #3184
	bl	FtlCacheWriteBack
	mov	r0, r6
	ldrh	r4, [r4, #-10]
	mov	r1, r4
	bl	__aeabi_uidiv
	smulbb	r3, r0, r4
	mov	r7, r0
	rsb	r6, r3, r6
	uxth	r6, r6
	cmp	r6, #0
	beq	.L3388
	rsb	r4, r6, r4
	add	r7, r0, #1
	cmp	r4, r5
	movcs	r4, r5
	uxth	r4, r4
	rsb	r5, r4, r5
.L3388:
	ldr	r4, .L3402+12
	mvn	r3, #0
	ldr	r8, .L3402+8
	str	r3, [sp, #4]
	mov	r6, r4
.L3389:
	ldrh	r3, [r4]
	cmp	r5, r3
	bcc	.L3401
	mov	r0, r7
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp]
	cmn	r3, #1
	beq	.L3390
	ldr	r2, .L3402+16
	add	r1, sp, #4
	mov	r0, r7
	ldr	r3, [r2, #420]
	add	r3, r3, #1
	str	r3, [r2, #420]
	ldr	r3, [r8, #-3132]
	mov	r2, #1
	add	r3, r3, #1
	str	r3, [r8, #-3132]
	bl	log2phys
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L3390:
	ldrh	r3, [r6]
	add	r7, r7, #1
	rsb	r5, r3, r5
	b	.L3389
.L3401:
	ldr	r3, .L3402+16
	ldr	r2, [r3, #420]
	cmp	r2, #32
	bls	.L3387
	mov	r4, #0
	str	r4, [r3, #420]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L3387
.L3385:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3403:
	.align	2
.L3402:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LANCHOR2-3194
	.word	.LANCHOR4
	.fnend
	.size	ftl_discard, .-ftl_discard
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r6, .L3443
	sub	r9, r6, #3200
	ldr	r8, [r6, #-2812]
	ldrh	r1, [r9]
	cmp	r8, #0
	bne	.L3441
	sub	r4, r6, #552
	mov	r5, r6
	movw	ip, #65535
	ldrh	r6, [r4]
	cmp	r6, ip
	bne	.L3407
.L3416:
	ldrh	r2, [r4]
	movw	r3, #65535
	ldr	r6, .L3443+4
	mov	r7, #0
	ldr	r8, .L3443
	cmp	r2, r3
	str	r7, [r6, #224]
	sub	r10, r8, #552
	beq	.L3441
	bl	FtlCacheWriteBack
	ldrb	r0, [r8, #-545]	@ zero_extendqisi2
	ldrh	r2, [r9]
	ldrh	r3, [r10]
	mov	r10, #12
	ldr	r1, [r8, #-2940]
	smulbb	r2, r0, r2
	mov	r3, r3, asl #1
	strh	r2, [r1, r3]	@ movhi
	ldr	r2, [r8, #-3140]
	ldrh	r3, [r6, #234]
	add	r3, r3, r2
	str	r3, [r8, #-3140]
	b	.L3417
.L3407:
	cmp	r0, #0
	beq	.L3410
	ldr	r2, .L3443+8
	movw	r3, #3288
	ldrh	r0, [r2, r3]
	cmp	r0, ip
	beq	.L3411
.L3412:
	mov	r1, #2
	b	.L3410
.L3411:
	strh	r8, [r2, r3]	@ movhi
	sub	r3, r5, #652
	ldrh	r3, [r3]
	cmp	r3, #17
	bhi	.L3412
.L3410:
	ldr	r7, .L3443
	sub	r0, r7, #552
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #4]
	beq	.L3413
	ldr	r2, [r7, #-2948]
	mov	r6, r6, asl #1
	ldrh	r3, [r2, r6]
	cmp	r3, #4
	bls	.L3414
	sub	r3, r3, #5
	mov	r0, #1
	strh	r3, [r2, r6]	@ movhi
	bl	FtlEctTblFlush
.L3414:
	ldr	r4, .L3443+4
	ldr	r3, [r4, #224]
	cmp	r3, #0
	bne	.L3415
	ldr	r0, [sp, #4]
	ldr	r3, [r4, #-224]
	ubfx	r0, r0, #10, #16
	add	r3, r3, #1
	str	r3, [r4, #-224]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L3415:
	mov	r3, #0
	str	r3, [r4, #224]
	b	.L3427
.L3413:
	ldr	r2, .L3443+8
	movw	r3, #3288
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3427
	b	.L3416
.L3420:
	ldr	r3, [r9, #4]
	cmp	r0, r3
	bne	.L3439
.L3419:
	add	r7, r7, #1
.L3417:
	ldrh	r3, [r6, #234]
	uxth	r8, r7
	cmp	r3, r8
	bls	.L3442
	mul	r8, r10, r8
	ldr	fp, [r5, #-3040]
	ldr	r3, [r5, #-2816]
	add	r9, fp, r8
	ldr	r0, [r9, #8]
	cmp	r0, r3
	bcs	.L3439
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r3, [fp, r8]
	ldr	r0, [sp, #4]
	cmp	r0, r3
	bne	.L3420
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r9, #4
	mov	r2, #1
	mov	r8, r0
	ldr	r0, [r9, #8]
	bl	log2phys
	mov	r0, r8
	b	.L3440
.L3439:
	ldrh	r0, [r4]
.L3440:
	bl	decrement_vpc_count
	b	.L3419
.L3442:
	ldr	r8, .L3443+12
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrb	r3, [r8, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3422
	ldr	r0, .L3443+16
	ldrh	r1, [r4]
	bl	printk
.L3422:
	ldrh	r0, [r4]
	ldr	r2, [r5, #-2940]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3423
	bl	INSERT_DATA_LIST
	b	.L3424
.L3423:
	bl	INSERT_FREE_LIST
.L3424:
	mvn	r9, #0
	strh	r9, [r4]	@ movhi
	mov	r4, #0
	strh	r4, [r6, #234]	@ movhi
	strh	r4, [r6, #232]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	r7, .L3443
	ldr	r2, [r5, #-3336]
	ldr	r3, .L3443+20
	cmp	r2, r4
	sub	r2, r7, #652
	strh	r9, [r3], #-4	@ movhi
	ldrh	r2, [r2]
	ldrh	r3, [r3]
	beq	.L3425
	ldr	r1, [r7, #-3100]
	cmp	r1, #39
	bhi	.L3425
	cmp	r2, r3
	subcc	r7, r7, #3056
	movcc	r3, r3, asl #1
	strcch	r3, [r7, #-8]	@ movhi
	b	.L3441
.L3425:
	add	r1, r3, r3, asl #1
	cmp	r2, r1, asr #2
	ble	.L3441
	ldrb	r0, [r8, #3904]	@ zero_extendqisi2
	ldr	r2, .L3443+24
	cmp	r0, #0
	moveq	r3, #20
	streqh	r3, [r2, #-8]	@ movhi
	beq	.L3406
	sub	r3, r3, #2
	strh	r3, [r2, #-8]	@ movhi
.L3441:
	mov	r0, #0
	b	.L3406
.L3427:
	mov	r0, #1
.L3406:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3444:
	.align	2
.L3443:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LC156
	.word	.LANCHOR4-368
	.word	.LANCHOR2-3056
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L3448
	ldr	r5, .L3448+4
	mov	r0, r4
	ldrh	r1, [r5]
	bl	FtlGcScanTempBlk
	ldrh	r2, [r4, #2]
	ldrh	r3, [r5]
	cmp	r2, r3
	ldmccfd	sp!, {r3, r4, r5, pc}
	add	r0, r4, #48
	bl	FtlMapBlkWriteDumpData
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r3, .L3448+8
	mov	r2, #0
	str	r2, [r3, #224]
	ldmfd	sp!, {r3, r4, r5, pc}
.L3449:
	.align	2
.L3448:
	.word	.LANCHOR2-552
	.word	.LANCHOR2-3200
	.word	.LANCHOR4
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
	ldr	r4, .L3452
	ldr	r3, .L3452+4
	sub	r6, r4, #648
	sub	r4, r4, #600
	mov	r0, r6
	str	r5, [r3, #276]
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
.L3453:
	.align	2
.L3452:
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
	ldr	r3, .L3472
	mov	r2, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	add	r8, r3, #272
	ldr	r4, .L3472+4
	mvn	r1, #0
	strh	r2, [r8]	@ movhi
	movw	r2, #270
	strh	r1, [r3, r2]	@ movhi
	ldr	r3, [r4, #-3264]
	uxth	r0, r3
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	sub	r3, r4, #404
	ldrh	r2, [r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3455
.L3457:
	mvn	r7, #0
	b	.L3456
.L3455:
	bl	FtlLoadSysInfo
	subs	r7, r0, #0
	bne	.L3457
	bl	FtlLoadMapInfo
	mov	r6, r7
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	r0, #1
	bl	FtlUpdateVaildLpn
	sub	r3, r4, #3152
	mov	r1, #12
	ldrh	r2, [r3, #-10]
	ldr	r3, [r4, #-2896]
.L3458:
	cmp	r6, r2
	bge	.L3463
	mla	r0, r1, r6, r3
	ldr	r0, [r0, #4]
	cmp	r0, #0
	bge	.L3459
.L3463:
	ldr	r5, .L3472+8
	cmp	r6, r2
	add	r6, r5, #68
	ldrh	r3, [r5, #28]
	add	r3, r3, #1
	strh	r3, [r5, #28]	@ movhi
	bge	.L3470
	b	.L3460
.L3459:
	add	r6, r6, #1
	b	.L3458
.L3470:
	ldrh	r3, [r8]
	cmp	r3, #0
	beq	.L3464
.L3460:
	ldrh	r3, [r6]
	ldr	r1, [r4, #-2940]
	ldr	ip, .L3472+12
	mov	r3, r3, asl #1
	ldrh	r0, [r6, #4]
	ldrh	r2, [r1, r3]
	rsb	r2, r0, r2
	strh	r2, [r1, r3]	@ movhi
	ldrh	r3, [ip]
	mov	r2, #0
	ldr	lr, [r4, #-2940]
	strb	r2, [r4, #-642]
	strh	r3, [r6, #2]	@ movhi
	ldr	r3, .L3472+16
	strh	r2, [r6, #4]	@ movhi
	ldrh	r1, [r3]
	ldrh	r8, [r3, #4]
	mov	r1, r1, asl #1
	ldrh	r0, [lr, r1]
	rsb	r0, r8, r0
	strh	r0, [lr, r1]	@ movhi
	ldrh	r1, [ip]
	strh	r2, [r3, #4]	@ movhi
	strb	r2, [r4, #-594]
	strh	r1, [r3, #2]	@ movhi
	ldrh	r3, [r5, #30]
	add	r3, r3, #1
	strh	r3, [r5, #30]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L3464:
	ldrh	r0, [r6]
	movw	r3, #65535
	ldr	r8, .L3472+20
	cmp	r0, r3
	beq	.L3465
	ldrh	r3, [r8, #4]
	cmp	r3, #0
	bne	.L3465
	ldrh	r3, [r8, #52]
	add	r4, r8, #48
	cmp	r3, #0
	bne	.L3465
	bl	FtlGcRefreshOpenBlock
	ldrh	r0, [r8, #48]
	bl	FtlGcRefreshOpenBlock
	bl	FtlVpcTblFlush
	mov	r0, r8
	bl	allocate_new_data_superblock
	mov	r0, r4
	bl	allocate_new_data_superblock
.L3465:
	ldr	r3, .L3472+24
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3466
	ldrh	r3, [r5, #28]
	tst	r3, #31
	bne	.L3456
.L3466:
	bl	FtlVpcCheckAndModify
.L3456:
	mov	r0, r7
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3473:
	.align	2
.L3472:
	.word	.LANCHOR4
	.word	.LANCHOR2
	.word	.LANCHOR2-716
	.word	.LANCHOR2-3200
	.word	.LANCHOR2-600
	.word	.LANCHOR2-648
	.word	.LANCHOR0
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
	ldr	r4, .L3509
	ldr	r5, [r4, #-2812]
	cmp	r5, #0
	bne	.L3476
	sub	r6, r4, #3152
	mov	r1, r5
	ldr	r0, [r4, #-2904]
	ldrh	r2, [r6, #-12]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r6, #-12]
	mov	r1, r5
	ldr	r0, [r4, #-2908]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r3, [r4, #-3264]
	str	r5, [r4, #-3148]
	str	r5, [r4, #-3144]
	uxth	r0, r3
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L3477
	bl	FtlMakeBbt
.L3477:
	ldr	r0, .L3509
	mov	r2, #0
	ldr	ip, .L3509+4
	ldr	lr, .L3509+8
.L3478:
	ldrh	r1, [ip]
	uxth	r3, r2
	add	r2, r2, #1
	cmp	r3, r1, asl #7
	bge	.L3506
	ldr	r5, [r0, #-2988]
	mvn	r1, r3
	orr	r1, r3, r1, asl #16
	str	r1, [r5, r3, asl #2]
	ldr	r1, [r0, #-2984]
	str	lr, [r1, r3, asl #2]
	b	.L3478
.L3506:
	ldr	r8, .L3509+12
	mov	r7, #0
	sub	r6, r8, #10
	ldrh	r5, [r8, #-12]
.L3480:
	ldrh	r3, [r6]
	ldr	r10, .L3509
	cmp	r3, r5
	sub	fp, r10, #3248
	sub	r9, fp, #10
	bls	.L3507
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
	add	r0, r7, r0
	uxth	r7, r0
	b	.L3480
.L3507:
	sub	r6, r10, #3264
	sub	r3, r7, #3
	ldrh	r1, [r6, #-4]
	cmp	r3, r1, asl #1
	bge	.L3482
.L3486:
	mov	r5, #0
	ldr	r9, .L3509+16
	mov	r7, r5
	b	.L3483
.L3482:
	mov	r0, r7
	bl	__aeabi_uidiv
	ldr	r3, [r10, #-3168]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r0, [r10, #-3264]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	ldrh	r7, [fp, #-12]
.L3484:
	ldrh	r3, [r9]
	cmp	r3, r7
	bls	.L3486
	mov	r0, r7
	add	r7, r7, #1
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	uxth	r7, r7
	b	.L3484
.L3483:
	ldrh	r2, [r9]
	uxth	r0, r5
	ldr	fp, .L3509
	add	r5, r5, #1
	cmp	r2, r0
	sub	r3, fp, #3248
	bls	.L3508
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r0, r7, r0
	uxth	r7, r0
	b	.L3483
.L3508:
	ldr	r2, [fp, #-3256]
	ldrh	r3, [r3, #-10]
	ldrh	r6, [r6, #-4]
	mov	r0, r2
	str	r2, [sp, #4]
	str	r3, [fp, #-3076]
	mov	r1, r6
	ldr	r5, .L3509+20
	bl	__aeabi_uidiv
	ubfx	r9, r0, #5, #16
	mov	r10, r0
	add	r3, r9, #36
	strh	r3, [r5]	@ movhi
	mov	r3, #24
	str	r0, [fp, #-2816]
	mul	r3, r3, r6
	ldr	r2, [sp, #4]
	cmp	r7, r3
	ble	.L3488
	rsb	r0, r7, r2
	mov	r1, r6
	bl	__aeabi_uidiv
	str	r0, [fp, #-2816]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r5]	@ movhi
.L3488:
	ldr	r3, [r4, #-3336]
	cmp	r3, #1
	bne	.L3489
	mov	r0, r7
	mov	r1, r6
	bl	__aeabi_uidiv
	ldrh	fp, [r5]
	uxtah	r0, fp, r0
	add	fp, fp, r0, asr #2
	strh	fp, [r5]	@ movhi
.L3489:
	ldr	fp, .L3509+24
	ldrb	r3, [fp, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3490
	ldrh	r3, [r5]
	mov	r0, r7
	mov	r1, r6
	str	r3, [sp, #4]
	bl	__aeabi_uidiv
	ldr	r3, [sp, #4]
	uxtah	r0, r3, r0
	add	r3, r3, r0, asr #2
	strh	r3, [r5]	@ movhi
.L3490:
	ldr	r0, .L3509
	sub	r2, r0, #3200
	ldrh	r3, [r2, #-6]
	cmp	r3, #0
	beq	.L3492
	ldrh	r1, [r5]
	add	r1, r1, r3, lsr #1
	strh	r1, [r5]	@ movhi
	mul	r1, r6, r3
	cmp	r1, r7
	strgt	r10, [r0, #-2816]
	addgt	r3, r3, #32
	ldrgt	r1, .L3509+20
	addgt	r3, r9, r3
	strgth	r3, [r1]	@ movhi
.L3492:
	ldrh	r1, [r5]
	mvn	r9, #0
	ldr	r3, [r4, #-2816]
	mov	r5, #0
	ldr	r7, .L3509+28
	rsb	r3, r1, r3
	mul	r6, r6, r3
	ldrh	r3, [r2]
	str	r6, [r7, #-376]
	mul	r3, r3, r6
	ldr	r6, .L3509+32
	ldrh	r2, [r6, #-10]
	str	r3, [r4, #-2816]
	mul	r3, r2, r3
	str	r3, [fp, #3908]
	bl	FtlBbmTblFlush
	ldrh	r2, [r8, #-10]
	mov	r1, #0
	ldr	r0, [r4, #-2940]
	ldr	fp, .L3509+36
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r8, #-12]
	sub	r3, r7, #368
	ldr	r0, [r4, #-2928]
	mov	r1, #255
	str	r5, [r4, #-408]
	mov	r2, r2, lsr #3
	strh	r9, [r3]	@ movhi
	strh	r5, [r3, #2]	@ movhi
	mov	r3, #1
	strb	r5, [r7, #-362]
	mov	r10, fp
	strb	r5, [r7, #-360]
	strh	r5, [fp, #2]	@ movhi
	strb	r5, [r4, #-642]
	strh	r5, [fp]	@ movhi
	strb	r3, [r4, #-640]
	bl	ftl_memset
	add	r2, r6, #3184
.L3494:
	ldr	r5, .L3509
	str	r2, [sp, #4]
	sub	r6, r5, #648
	mov	r0, r6
	bl	make_superblock
	ldrb	r3, [r4, #-641]	@ zero_extendqisi2
	cmp	r3, #0
	ldrh	r3, [fp]
	bne	.L3495
	ldr	r2, [sp, #4]
	mov	r3, r3, asl #1
	ldr	r1, [r2, #-2940]
	strh	r9, [r1, r3]	@ movhi
	ldrh	r3, [r10]
	add	r8, r3, #1
	strh	r8, [r10]	@ movhi
	b	.L3494
.L3495:
	ldr	r2, [r5, #-3148]
	mov	r3, r3, asl #1
	ldrh	r1, [r6, #4]
	mvn	fp, #0
	str	r2, [r5, #-636]
	add	r2, r2, #1
	str	r2, [r5, #-3148]
	ldr	r2, [r5, #-2940]
	strh	r1, [r2, r3]	@ movhi
	sub	r2, r5, #600
	mov	r3, #0
	strb	r3, [r5, #-594]
	strh	r3, [r2, #2]	@ movhi
	mov	r8, r2
	ldrh	r3, [r6]
	mov	r10, r2
	add	r3, r3, #1
	strh	r3, [r2]	@ movhi
	mov	r3, #1
	strb	r3, [r5, #-592]
.L3496:
	ldr	r6, .L3509
	sub	r9, r6, #600
	mov	r0, r9
	bl	make_superblock
	ldrb	r3, [r4, #-593]	@ zero_extendqisi2
	cmp	r3, #0
	ldrh	r3, [r8]
	bne	.L3497
	ldr	r2, [r5, #-2940]
	mov	r3, r3, asl #1
	strh	fp, [r2, r3]	@ movhi
	ldrh	r3, [r10]
	add	r3, r3, #1
	strh	r3, [r10]	@ movhi
	b	.L3496
.L3497:
	ldr	r2, [r6, #-3148]
	mov	r3, r3, asl #1
	ldrh	r1, [r9, #4]
	mvn	r4, #0
	str	r2, [r6, #-588]
	add	r2, r2, #1
	str	r2, [r6, #-3148]
	ldr	r2, [r6, #-2940]
	strh	r1, [r2, r3]	@ movhi
	sub	r3, r6, #552
	strh	r4, [r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	sub	r3, r6, #404
	mov	r2, #0
	strh	r2, [r3, #2]	@ movhi
	ldr	r2, [r7, #-376]
	strh	r4, [r3, #4]	@ movhi
	strh	r2, [r3, #6]	@ movhi
	strh	r0, [r3]	@ movhi
	ldr	r3, [r6, #-3148]
	str	r3, [r6, #-396]
	add	r3, r3, #1
	str	r3, [r6, #-3148]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	ldreq	r3, .L3509+40
	moveq	r2, #1
	streq	r2, [r3, #3284]
.L3476:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3510:
	.align	2
.L3509:
	.word	.LANCHOR2
	.word	.LANCHOR2-3194
	.word	168778952
	.word	.LANCHOR2-3248
	.word	.LANCHOR2-3260
	.word	.LANCHOR4-372
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR2-3184
	.word	.LANCHOR2-648
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
	ldr	r4, .L3540
	ldrb	r3, [r4, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3512
.L3514:
	mov	r0, #0
	b	.L3513
.L3512:
	ldr	r3, .L3540+4
	ldr	r0, [r3]
	bl	FlashInit
	cmp	r0, #0
	bne	.L3514
	bl	FlashLoadFactorBbt
	cmp	r0, #0
	beq	.L3515
	bl	FlashMakeFactorBbt
.L3515:
	ldr	r5, .L3540+8
	ldr	r0, [r5, #-3308]
	bl	FlashReadIdbDataRaw
	cmp	r0, #0
	beq	.L3516
	mov	r1, #0
	mov	r2, #16
	mov	r0, sp
	bl	FlashReadFacBbtData
	mov	r3, #0
	mov	r2, r3
	mov	ip, #1
	ldr	r1, [sp]
.L3517:
	ands	lr, r1, ip, asl r2
	add	r0, r3, #1
	add	r2, r2, #1
	movne	r3, r0
	cmp	r2, #16
	bne	.L3517
	cmp	r3, #6
	bls	.L3536
	mov	r2, #0
	mov	ip, #1
.L3520:
	ands	lr, r1, ip, asl r2
	add	r0, r3, #1
	add	r2, r2, #1
	movne	r3, r0
	cmp	r2, #24
	bne	.L3520
	cmp	r3, #17
	movhi	r3, #36
	strhib	r3, [r4, #1]
	bhi	.L3519
.L3536:
	strb	r2, [r4, #1]
.L3519:
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	ldr	r3, .L3540+12
	strh	r2, [r3, #26]	@ movhi
.L3516:
	ldr	r1, .L3540+16
	mov	r4, #1
	ldr	r0, .L3540+20
	bl	printk
	ldr	r0, .L3540+12
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldr	r0, [r5, #-3264]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
.L3522:
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L3523
.L3538:
	bl	FtlLowFormat
	cmp	r4, #3
	addls	r4, r4, #1
	bls	.L3522
.L3539:
	mvn	r0, #0
	b	.L3513
.L3523:
	bl	FtlSysBlkInit
	cmp	r0, #0
	bne	.L3538
	ldr	r3, .L3540+24
	mov	r2, #1
	str	r2, [r3, #3284]
.L3513:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L3541:
	.align	2
.L3540:
	.word	.LANCHOR0
	.word	RK29_NANDC_REG_BASE
	.word	.LANCHOR2
	.word	.LANCHOR0+3876
	.word	.LC140
	.word	.LC46
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
	ldr	r4, .L3559
	ldr	r3, [r4, #-2812]
	cmp	r3, #0
	beq	.L3543
.L3546:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3543:
	ldr	r3, .L3559+4
	ldr	r6, .L3559+8
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3545
	ldr	r3, [r6, #208]
	tst	r3, #1
	beq	.L3545
	sub	r3, r4, #552
	ldrh	r3, [r3, #4]
	cmp	r3, #0
	bne	.L3546
.L3545:
	mov	r2, #0
	ldr	r0, [r4, #-3032]
	ldr	r1, [r6, #208]
	mov	r3, r2
	bl	FlashProgPages
	mov	r7, #0
	mov	r8, #36
.L3547:
	ldr	r1, [r6, #208]
	uxth	r3, r7
	ldr	r5, .L3559+8
	cmp	r3, r1
	bcs	.L3558
	mul	r3, r8, r3
	ldr	r0, [r4, #-3032]
	ldr	r2, .L3559
	add	r7, r7, #1
	add	r1, r0, r3
	ldr	lr, [r0, r3]
	ldr	ip, [r1, #12]
	cmn	lr, #1
	bne	.L3548
	sub	r0, r2, #552
	ldr	ip, [r2, #-2940]
	mov	r4, #0
	ldrh	r1, [r0]
	mov	r1, r1, asl #1
	strh	r4, [ip, r1]	@ movhi
	ldr	r2, [r2, #-3032]
	ldr	r1, [r5, #-224]
	add	r3, r2, r3
	strh	lr, [r0]	@ movhi
	add	r1, r1, #1
	str	r1, [r5, #-224]
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	b	.L3557
.L3548:
	ldr	r0, [ip, #12]
	ldr	r1, [r1, #4]
	ldr	r2, [ip, #8]
	bl	FtlGcUpdatePage
	b	.L3547
.L3558:
	ldr	r0, [r4, #-3032]
	bl	FtlGcBufFree
	mov	r3, #0
	str	r3, [r5, #208]
	ldr	r3, .L3559+12
	ldrh	r3, [r3, #4]
	cmp	r3, #0
	bne	.L3546
	mov	r0, #1
	bl	FtlGcFreeTempBlock
.L3557:
	mov	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3560:
	.align	2
.L3559:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR2-552
	.fnend
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3565
	movw	r2, #65535
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrh	r1, [r3]
	cmp	r1, r2
	beq	.L3562
	ldrh	r3, [r3, #4]
	cmp	r3, #0
	bne	.L3563
.L3562:
	bl	FtlCacheWriteBack
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L3565+4
	mov	r4, #0
	sub	r0, r0, #552
	strb	r4, [r0, #8]
	bl	allocate_data_superblock
	ldr	r3, .L3565+8
	strh	r4, [r3, #232]	@ movhi
	strh	r4, [r3, #234]	@ movhi
	bl	l2p_flush
	mov	r0, r4
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3563:
	ldr	r0, .L3565
	ldmfd	sp!, {r4, lr}
	b	get_new_active_ppa
.L3566:
	.align	2
.L3565:
	.word	.LANCHOR2-552
	.word	.LANCHOR2
	.word	.LANCHOR4
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.global	ftl_read
	.type	ftl_read, %function
ftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r1
	ldr	r1, .L3614
	.pad #84
	sub	sp, sp, #84
	ldr	r5, [r1, #3284]
	cmn	r5, #1
	moveq	r0, r5
	beq	.L3568
	cmp	r0, #16
	mov	r8, r3
	str	r2, [sp, #36]
	bne	.L3569
	mov	r1, r2
	add	r0, r4, #256
	mov	r2, r3
	bl	FtlVendorPartRead
	b	.L3568
.L3569:
	ldr	r3, [sp, #36]
	add	r3, r4, r3
	str	r3, [sp, #40]
	ldr	r3, .L3614+4
	ldr	r2, [sp, #40]
	ldr	r3, [r3, #3908]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L3568
	ldr	r5, .L3614+8
	mov	r0, r4
	sub	r3, r5, #3184
	ldrh	r6, [r3, #-10]
	mov	r1, r6
	bl	__aeabi_uidiv
	mov	r1, r6
	ldr	r3, [sp, #40]
	str	r0, [sp, #28]
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	r2, .L3614+12
	ldr	r3, [sp, #28]
	ldr	r1, [sp, #36]
	rsb	r3, r3, #1
	str	r0, [sp, #32]
	add	r3, r3, r0
	str	r3, [sp, #24]
	ldr	r3, [r2, #196]
	ldr	r0, [sp, #28]
	add	r3, r1, r3
	str	r3, [r2, #196]
	ldr	r3, [r5, #-3120]
	ldr	r2, [sp, #24]
	ldr	r1, [sp, #32]
	add	r3, r2, r3
	str	r3, [r5, #-3120]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L3570
	bl	FtlCacheWriteBack
.L3570:
	ldr	r6, [sp, #28]
	mov	r9, #0
	ldr	r5, .L3614+8
	mov	r7, r9
	str	r9, [sp, #48]
	str	r9, [sp, #52]
.L3571:
	ldr	r3, [sp, #24]
	cmp	r3, #0
	beq	.L3613
	mov	r0, r6
	add	r1, sp, #76
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #76]
	cmn	r3, #1
	bne	.L3609
	mov	r10, #0
.L3572:
	ldr	r3, .L3614+16
	ldrh	r0, [r3]
	cmp	r10, r0
	bcs	.L3576
	mla	r0, r0, r6, r10
	ldr	r2, [sp, #40]
	cmp	r0, r4
	movcs	r3, #1
	movcc	r3, #0
	cmp	r0, r2
	movcs	r3, #0
	cmp	r3, #0
	beq	.L3574
	rsb	r0, r4, r0
	mov	r1, #0
	mov	r2, #512
	add	r0, r8, r0, asl #9
	bl	ftl_memset
.L3574:
	add	r10, r10, #1
	b	.L3572
.L3609:
	ldr	r2, [r5, #-3036]
	mov	r10, #36
	mla	r10, r10, r7, r2
	str	r3, [r10, #4]
	ldr	r3, [sp, #28]
	cmp	r6, r3
	bne	.L3577
	ldr	r3, [r5, #-2988]
	mov	r0, r4
	str	r3, [r10, #8]
	ldr	r3, .L3614+16
	ldrh	fp, [r3]
	mov	r1, fp
	bl	__aeabi_uidivmod
	ldr	r2, [sp, #36]
	rsb	r3, r1, fp
	str	r1, [sp, #56]
	cmp	r3, r2
	movcs	r3, r2
	cmp	r3, fp
	str	r3, [sp, #48]
	streq	r8, [r10, #8]
	b	.L3578
.L3577:
	ldr	r3, [sp, #32]
	cmp	r6, r3
	bne	.L3579
	ldr	r3, [r5, #-2984]
	ldr	r1, [sp, #40]
	str	r3, [r10, #8]
	ldr	r3, .L3614+16
	ldrh	r2, [r3]
	mul	r3, r2, r6
	rsb	r9, r3, r1
	cmp	r9, r2
	bne	.L3578
	b	.L3611
.L3579:
	ldr	r3, .L3614+16
	ldrh	r3, [r3]
	mul	r3, r3, r6
.L3611:
	rsb	r3, r4, r3
	add	r3, r8, r3, asl #9
	str	r3, [r10, #8]
.L3578:
	ldr	r3, .L3614+20
	ldr	r2, [r5, #-2972]
	str	r6, [r10, #16]
	ldrh	r3, [r3]
	mul	r3, r3, r7
	add	r7, r7, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [r10, #12]
.L3576:
	ldr	r3, [sp, #24]
	add	r6, r6, #1
	subs	r3, r3, #1
	str	r3, [sp, #24]
	beq	.L3580
	ldr	r3, .L3614+24
	ldrh	r3, [r3]
	cmp	r7, r3, asl #3
	bne	.L3571
.L3580:
	cmp	r7, #0
	beq	.L3571
	ldr	r0, [r5, #-3036]
	mov	r1, r7
	mov	r2, #0
	ldr	fp, .L3614+12
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
.L3587:
	ldr	r3, [sp, #44]
	mov	ip, #36
	ldr	r1, [sp, #28]
	mul	r10, ip, r3
	ldr	r3, [r5, #-3036]
	add	r3, r3, r10
	ldr	r2, [r3, #16]
	cmp	r2, r1
	bne	.L3582
	ldr	r1, [r3, #8]
	ldr	r3, [r5, #-2988]
	cmp	r1, r3
	bne	.L3583
	ldr	r3, [sp, #60]
	mov	r0, r8
	ldr	r2, [sp, #64]
	add	r1, r1, r3
	b	.L3612
.L3582:
	ldr	r1, [sp, #32]
	cmp	r2, r1
	bne	.L3583
	ldr	r1, [r3, #8]
	ldr	r3, [r5, #-2984]
	cmp	r1, r3
	bne	.L3583
	ldr	r3, .L3614+16
	ldr	r2, [sp, #68]
	ldrh	r0, [r3]
	ldr	r3, [sp, #32]
	mul	r0, r0, r3
	rsb	r0, r4, r0
	add	r0, r8, r0, asl #9
.L3612:
	bl	ftl_memcpy
.L3583:
	ldr	r2, [r5, #-3036]
	add	r3, r2, r10
	ldr	r1, [r2, r10]
	cmn	r1, #1
	streq	r1, [sp, #52]
	ldreq	r2, [fp, #-248]
	addeq	r2, r2, #1
	streq	r2, [fp, #-248]
	ldr	r2, [r3, #12]
	ldr	r1, [r3, #16]
	ldr	r2, [r2, #8]
	cmp	r1, r2
	beq	.L3585
	ldr	r2, [fp, #-248]
	ldr	r0, .L3614+28
	add	r2, r2, #1
	str	r2, [fp, #-248]
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
.L3585:
	ldr	r3, [r5, #-3036]
	add	r2, r3, r10
	ldr	r3, [r3, r10]
	cmp	r3, #256
	bne	.L3586
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L3586:
	ldr	r3, [sp, #44]
	add	r3, r3, #1
	str	r3, [sp, #44]
	cmp	r3, r7
	bne	.L3587
	mov	r7, #0
	b	.L3571
.L3613:
	ldr	r3, .L3614+32
	ldrh	r3, [r3, #-2]
	cmp	r3, #0
	beq	.L3589
	ldr	r0, [sp, #24]
	mov	r1, #1
	bl	ftl_do_gc
.L3589:
	ldr	r0, [sp, #52]
.L3568:
	add	sp, sp, #84
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3615:
	.align	2
.L3614:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	.LANCHOR2-3194
	.word	.LANCHOR2-3188
	.word	.LANCHOR2-3268
	.word	.LC55
	.word	.LANCHOR2-3056
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
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mvn	r3, #0
	ldr	r5, .L3636
	ldr	r6, .L3636+4
	ldr	r8, .L3636+8
	ldr	r1, .L3636+12
	ldr	r0, .L3636+16
	str	r3, [r6, #3284]
	mov	r3, #0
	str	r3, [r8, #424]
	str	r3, [r5, #-2812]
	bl	printk
	ldr	r0, .L3636+20
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldr	r3, [r5, #-3264]
	uxth	r0, r3
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	ldrne	r0, .L3636+24
	bne	.L3635
	bl	FtlSysBlkInit
	subs	r4, r0, #0
	beq	.L3622
	ldr	r0, .L3636+28
.L3635:
	ldr	r1, .L3636+32
	bl	printk
	b	.L3621
.L3622:
	mov	r1, #1
	str	r1, [r6, #3284]
	bl	ftl_do_gc
	sub	r3, r5, #652
	ldrh	r9, [r3]
	mov	r7, r3
	cmp	r9, #15
	bhi	.L3623
	sub	r8, r8, #368
	sub	r5, r5, #3072
	movw	r6, #65535
.L3626:
	ldrh	r3, [r8]
	cmp	r3, r6
	bne	.L3624
	ldrh	r3, [r5]
	cmp	r3, r6
	bne	.L3624
	and	r0, r4, #7
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L3624:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	mov	r0, #0
	mov	r1, #1
	bl	ftl_do_gc
	ldrh	r2, [r7]
	add	r3, r9, #2
	cmp	r2, r3
	bhi	.L3621
	add	r4, r4, #1
	cmp	r4, #1024
	bne	.L3626
	b	.L3621
.L3623:
	ldr	r3, .L3636+36
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3621
	mov	r4, #128
.L3628:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	bne	.L3628
.L3621:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L3637:
	.align	2
.L3636:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR4
	.word	.LC140
	.word	.LC46
	.word	.LANCHOR0+3876
	.word	.LC157
	.word	.LC158
	.word	.LANCHOR3+108
	.word	.LANCHOR0
	.fnend
	.size	FtlInit, .-FtlInit
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
	ldr	r10, .L3711
	str	r3, [sp, #8]
	ldr	r3, [r10, #-2812]
	cmp	r3, #0
	bne	.L3679
	mov	r8, r2
	ldr	r2, .L3711+4
	ldr	r2, [r2, #3284]
	cmn	r2, #1
	moveq	r0, r3
	beq	.L3639
	cmp	r0, #16
	mov	r7, r1
	bne	.L3640
	add	r0, r1, #256
	ldr	r2, [sp, #8]
	mov	r1, r8
	bl	FtlVendorPartWrite
	b	.L3639
.L3640:
	ldr	r3, .L3711+8
	add	r5, r1, r8
	ldr	r3, [r3, #3908]
	cmp	r5, r3
	mvnhi	r0, #0
	bhi	.L3639
	ldr	r6, .L3711+12
	mov	r3, #2048
	mov	r0, r1
	str	r3, [r6, #428]
	sub	r3, r10, #3184
	ldrh	r4, [r3, #-10]
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
	ldr	r3, [r10, #-3136]
	add	r3, r2, r3
	ldr	r2, [r10, #-3048]
	str	r3, [r10, #-3136]
	ldr	r3, [r6, #192]
	add	r3, r8, r3
	str	r3, [r6, #192]
	movcs	r3, #1
	movcc	r3, #0
	cmp	r2, #0
	str	r3, [sp, #24]
	beq	.L3642
	mov	r3, #36
	ldr	r9, [r10, #-3016]
	mul	r3, r3, r2
	ldr	r2, [sp, #12]
	sub	r3, r3, #36
	add	r9, r9, r3
	ldr	r3, [r9, #16]
	cmp	r2, r3
	bne	.L3643
	ldr	r3, [r10, #-3128]
	mov	r1, r4
	mov	r0, r7
	add	r3, r3, #1
	str	r3, [r10, #-3128]
	ldr	r3, [r6, #432]
	add	r3, r3, #1
	str	r3, [r6, #432]
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
	bne	.L3644
	ldr	r3, [r6, #432]
	cmp	r3, #2
	ble	.L3679
.L3644:
	ldr	r3, [sp, #8]
	rsb	r8, r4, r8
	add	r7, r7, r4
	str	r5, [sp, #4]
	add	r3, r3, r10
	str	r3, [sp, #8]
	ldr	r3, [sp, #12]
	add	r3, r3, #1
	str	r3, [sp, #12]
.L3643:
	mov	r3, #0
	str	r3, [r6, #432]
.L3642:
	ldr	r0, [sp, #12]
	ldr	r1, [sp, #28]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L3645
	bl	FtlCacheWriteBack
.L3645:
	ldr	r5, .L3711+16
	mov	r3, #0
	str	r3, [sp, #16]
	add	r4, r5, #648
	str	r3, [sp, #32]
	str	r5, [r6, #416]
	ldr	r6, [sp, #12]
.L3646:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	beq	.L3710
	ldrh	r2, [r5, #4]
	cmp	r2, #0
	bne	.L3647
	ldr	r3, .L3711+16
	ldr	r9, .L3711+4
	cmp	r5, r3
	bne	.L3648
	add	r0, r5, #48
	ldrh	r10, [r0, #4]
	cmp	r10, #0
	bne	.L3649
	bl	allocate_new_data_superblock
	str	r10, [r9, #3292]
.L3649:
	ldr	r0, .L3711+16
	bl	allocate_new_data_superblock
	ldr	r3, [r9, #3292]
	cmp	r3, #0
	ldrne	r5, .L3711+20
	bne	.L3650
.L3651:
	ldr	r5, .L3711+16
	b	.L3650
.L3648:
	ldrh	r3, [r3, #4]
	str	r2, [r9, #3292]
	cmp	r3, #0
	bne	.L3651
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3650:
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	bne	.L3652
	mov	r0, r5
	bl	allocate_new_data_superblock
.L3652:
	ldr	r3, .L3711+12
	str	r5, [r3, #416]
.L3647:
	ldr	r3, [r4, #-3052]
	ldr	r1, [r4, #-3048]
	ldrh	r2, [r5, #4]
	rsb	r3, r1, r3
	cmp	r2, r3
	movcs	r2, r3
	ldr	r3, [sp, #4]
	cmp	r2, r3
	movcc	r3, r2
	str	r3, [sp, #44]
	mov	r3, #0
.L3708:
	str	r3, [sp, #20]
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #44]
	cmp	r3, r2
	beq	.L3654
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L3654
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
	beq	.L3655
	ldr	r3, .L3711+24
	ldrh	r2, [r3]
	add	r3, r8, r7
	mls	r3, r2, r6, r3
	cmp	r3, r2
	bne	.L3654
.L3655:
	add	r1, sp, #56
	mov	r2, #0
	mov	r0, r6
	str	ip, [sp, #52]
	bl	log2phys
	mov	r0, r5
	bl	get_new_active_ppa
	ldr	r10, [r4, #-3048]
	ldr	r1, [r4, #-3016]
	mov	r3, #36
	ldr	fp, .L3711+28
	mla	r1, r3, r10, r1
	ldrh	r2, [fp]
	str	r6, [r1, #16]
	str	r0, [r1, #4]
	mul	r0, r10, r2
	str	r3, [sp, #48]
	bic	r3, r0, #3
	str	r3, [sp, #36]
	ldr	r0, [sp, #36]
	ldr	r3, [r4, #-2964]
	add	r9, r3, r0
	ldrh	r0, [fp, #-2]
	str	r9, [r1, #12]
	str	r3, [sp, #40]
	mul	r10, r10, r0
	ldr	r0, [r4, #-2992]
	bic	r10, r10, #3
	add	r10, r0, r10
	mov	r0, r9
	str	r10, [r1, #8]
	mov	r1, #0
	bl	ftl_memset
	ldr	r3, [sp, #12]
	ldr	ip, [sp, #52]
	rsb	r10, r3, r6
	clz	r10, r10
	mov	r10, r10, lsr #5
	orrs	r3, r10, ip
	ldr	r3, [sp, #48]
	beq	.L3656
	cmp	r10, #0
	beq	.L3657
	ldrh	fp, [fp, #-6]
	mov	r0, r7
	mov	r1, fp
	bl	__aeabi_uidivmod
	rsb	r2, r1, fp
	mov	r3, r1
	cmp	r2, r8
	str	r1, [sp, #32]
	movcc	r3, r2
	movcs	r3, r8
	str	r3, [sp, #16]
	b	.L3658
.L3657:
	cmp	ip, #0
	beq	.L3658
	ldr	r3, .L3711+24
	add	r2, r8, r7
	str	r10, [sp, #32]
	ldrh	r1, [r3]
	smulbb	r1, r1, r6
	rsb	r2, r1, r2
	uxth	r3, r2
	str	r3, [sp, #16]
.L3658:
	ldr	r3, .L3711+24
	ldr	r2, [sp, #16]
	ldrh	r3, [r3]
	cmp	r2, r3
	bne	.L3659
	cmp	r10, #0
	ldr	r0, .L3711
	moveq	r3, r2
	ldr	r2, .L3711
	muleq	r1, r6, r3
	ldreq	r3, [sp, #8]
	ldr	r2, [r2, #-3048]
	ldr	r0, [r0, #-3016]
	rsbeq	r1, r7, r1
	ldrne	r1, [sp, #8]
	addeq	r1, r3, r1, asl #9
	ldr	r3, [sp, #24]
	cmp	r3, #0
	mov	r3, #36
	mla	r3, r3, r2, r0
	strne	r1, [r3, #8]
	bne	.L3662
	ldr	r0, [r3, #8]
	ldr	r3, .L3711+32
	ldrh	r2, [r3]
	b	.L3706
.L3659:
	ldr	r2, [sp, #56]
	mov	r3, #36
	cmn	r2, #1
	beq	.L3663
	ldr	r1, [r4, #-3016]
	add	r0, sp, #60
	str	r2, [sp, #64]
	ldr	r2, [r4, #-3048]
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
	ldreq	r2, .L3711+12
	ldreq	r3, [r2, #-248]
	addeq	r3, r3, #1
	streq	r3, [r2, #-248]
	beq	.L3666
.L3664:
	ldr	r3, [r9, #8]
	cmp	r3, r6
	beq	.L3666
	ldr	r2, .L3711+12
	ldr	r0, .L3711+36
	ldr	r3, [r2, #-248]
	add	r3, r3, #1
	str	r3, [r2, #-248]
	mov	r2, r6
	ldr	r1, [r9, #8]
	bl	printk
	b	.L3666
.L3663:
	ldr	r2, [r4, #-3048]
	ldr	r1, [r4, #-3016]
	mla	r3, r3, r2, r1
	mov	r1, #0
	ldr	r0, [r3, #8]
	ldr	r3, .L3711+32
	ldrh	r2, [r3]
	bl	ftl_memset
.L3666:
	cmp	r10, #0
	mov	r3, #36
	beq	.L3667
	ldr	r1, [r4, #-3016]
	ldr	r2, [r4, #-3048]
	mla	r3, r3, r2, r1
	ldr	r1, [sp, #8]
	ldr	r0, [r3, #8]
	ldr	r3, [sp, #32]
	add	r0, r0, r3, asl #9
	b	.L3709
.L3667:
	ldr	r1, [r4, #-3048]
	ldr	r2, [r4, #-3016]
	mla	r3, r3, r1, r2
	ldr	r2, .L3711+24
	ldrh	r1, [r2]
	ldr	r0, [r3, #8]
	mul	r1, r1, r6
	ldr	r3, [sp, #8]
	rsb	r1, r7, r1
	add	r1, r3, r1, asl #9
.L3709:
	ldr	r3, [sp, #16]
	mov	r2, r3, asl #9
	b	.L3706
.L3656:
	ldr	r2, [sp, #24]
	cmp	r2, #0
	ldr	r2, [r4, #-3048]
	beq	.L3668
	ldr	r1, [r4, #-3016]
	mla	r3, r3, r2, r1
	ldr	r2, .L3711+24
	ldrh	fp, [r2]
	ldr	r2, [sp, #8]
	mul	fp, fp, r6
	rsb	fp, r7, fp
	add	fp, r2, fp, asl #9
	str	fp, [r3, #8]
	b	.L3662
.L3668:
	ldr	r0, [r4, #-3016]
	mla	r3, r3, r2, r0
	ldr	r2, .L3711+24
	ldrh	r1, [r2]
	ldrh	r2, [fp, #-2]
	ldr	r0, [r3, #8]
	mul	r1, r1, r6
	ldr	r3, [sp, #8]
	rsb	r1, r7, r1
	add	r1, r3, r1, asl #9
.L3706:
	bl	ftl_memcpy
.L3662:
	ldr	r3, .L3711+40
	ldr	r2, [sp, #40]
	ldr	r1, [sp, #36]
	strh	r3, [r2, r1]	@ movhi
	ldr	r3, [r4, #-3144]
	str	r6, [r9, #8]
	add	r6, r6, #1
	str	r3, [r9, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, #0
	str	r3, [r4, #-3144]
	ldr	r3, [sp, #56]
	str	r3, [r9, #12]
	ldrh	r3, [r5]
	strh	r3, [r9, #2]	@ movhi
	ldr	r3, [r4, #-3048]
	add	r3, r3, #1
	str	r3, [r4, #-3048]
	ldr	r3, [sp, #20]
	add	r3, r3, #1
	b	.L3708
.L3654:
	ldr	r3, [sp, #4]
	ldr	r2, [sp, #20]
	ldr	r1, [sp, #24]
	rsb	r3, r2, r3
	ldr	r2, [r4, #-3052]
	str	r3, [sp, #4]
	ldr	r3, [r4, #-3048]
	cmp	r3, r2
	orrcs	r1, r1, #1
	cmp	r1, #0
	bne	.L3672
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L3672
.L3674:
	mov	r3, #0
	str	r3, [sp, #24]
	b	.L3646
.L3672:
	bl	FtlCacheWriteBack
	ldr	r2, .L3711
	mov	r3, #0
	str	r3, [r2, #-3048]
	ldr	r3, [sp, #4]
	cmp	r3, #1
	bhi	.L3646
	b	.L3674
.L3710:
	mov	r0, r3
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #28]
	ldr	r5, .L3711+44
	rsb	r1, r2, r3
	bl	ftl_do_gc
	ldrh	r4, [r5]
	cmp	r4, #5
	bls	.L3684
	cmp	r4, #31
	bhi	.L3679
	ldr	r3, .L3711+8
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3679
.L3684:
	ldr	r6, .L3711
	ldr	r4, [sp, #4]
	ldr	r7, .L3711+48
	sub	r8, r6, #3072
.L3697:
	ldrh	r2, [r7]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3678
	ldrh	r3, [r8]
	cmp	r3, r2
	bne	.L3678
	ldr	r2, .L3711+52
	ldrh	r2, [r2]
	cmp	r2, r3
	bne	.L3678
	and	r0, r4, #7
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L3678:
	ldr	r3, .L3711+56
	mov	r0, #1
	mov	r1, r0
	mov	r2, #128
	strh	r2, [r3]	@ movhi
	strh	r2, [r3, #-2]	@ movhi
	bl	ftl_do_gc
	mov	r0, #0
	mov	r1, #1
	bl	ftl_do_gc
	ldr	r3, [r6, #-2812]
	cmp	r3, #0
	bne	.L3679
	ldrh	r3, [r5]
	cmp	r3, #2
	bhi	.L3679
	add	r4, r4, #1
	cmp	r4, #256
	bne	.L3697
.L3679:
	mov	r0, #0
.L3639:
	add	sp, sp, #100
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3712:
	.align	2
.L3711:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR2-648
	.word	.LANCHOR2-600
	.word	.LANCHOR2-3194
	.word	.LANCHOR2-3188
	.word	.LANCHOR2-3190
	.word	.LC159
	.word	-3947
	.word	.LANCHOR2-652
	.word	.LANCHOR4-368
	.word	.LANCHOR2-3070
	.word	.LANCHOR2-3062
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
	.global	ftl_fix_nand_power_lost_error
	.type	ftl_fix_nand_power_lost_error, %function
ftl_fix_nand_power_lost_error:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3733
	ldrb	r3, [r3, #3904]	@ zero_extendqisi2
	cmp	r3, #0
	bxeq	lr
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #270
	ldr	r8, .L3733+4
	.pad #52
	sub	sp, sp, #52
	ldr	r4, .L3733+8
	ldr	r0, .L3733+12
	ldrh	r7, [r8, r3]
	sub	r9, r4, #648
	ldr	r3, [r4, #-2940]
	sub	r5, r4, #600
	mov	r6, r7, asl #1
	mov	r1, r7
	ldrh	r2, [r3, r6]
	bl	printk
	ldrh	r0, [r9]
	bl	FtlGcRefreshOpenBlock
	ldrh	r0, [r5]
	bl	FtlGcRefreshOpenBlock
	mov	r0, r9
	bl	allocate_new_data_superblock
	mov	r0, r5
	bl	allocate_new_data_superblock
	movw	r5, #4097
.L3718:
	subs	r5, r5, #1
	beq	.L3722
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	ldr	r3, [r4, #-2940]
	ldrh	r3, [r3, r6]
	cmp	r3, #0
	bne	.L3718
.L3722:
	ldr	r3, [r4, #-2940]
	mov	r1, r7
	ldr	r0, .L3733+12
	ldr	r9, .L3733+8
	ldrh	r2, [r3, r6]
	bl	printk
	ldr	r3, [r4, #-2940]
	ldrh	r5, [r3, r6]
	cmp	r5, #0
	bne	.L3720
	add	r0, sp, #48
	movw	r10, #65535
	mov	fp, #36
	strh	r7, [r0, #-48]!	@ movhi
	bl	make_superblock
	sub	r3, r9, #3264
	ldr	r9, [r9, #-3024]
	ldrh	lr, [r3, #-4]
	mov	r3, r5
	mov	ip, r3
	add	r0, sp, #14
.L3723:
	uxth	r2, r3
	cmp	r2, lr
	bcs	.L3732
	ldrh	r2, [r0, #2]!
	add	r3, r3, #1
	cmp	r2, r10
	movne	r2, r2, asl #10
	mlane	r1, fp, r5, r9
	addne	r5, r5, #1
	uxthne	r5, r5
	stmneib	r1, {r2, ip}
	strne	ip, [r1, #12]
	b	.L3723
.L3732:
	ldr	r3, [r4, #-2940]
	mov	r1, r7
	ldr	r0, .L3733+16
	ldrh	r2, [r3, r6]
	bl	printk
	mov	r1, #0
	mov	r2, r5
	ldr	r0, [r4, #-3024]
	bl	FlashEraseBlocks
	ldr	r0, [r4, #-3024]
	mov	r1, #1
	mov	r2, r5
	bl	FlashEraseBlocks
.L3720:
	movw	r3, #270
	mvn	r2, #0
	strh	r2, [r8, r3]	@ movhi
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3734:
	.align	2
.L3733:
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR2
	.word	.LC160
	.word	.LC161
	.fnend
	.size	ftl_fix_nand_power_lost_error, .-ftl_fix_nand_power_lost_error
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
	.global	read_retry_cur_offset
	.section	.rodata
	.align	2
.LANCHOR3 = . + 0
.LC0:
	.byte	60
	.byte	40
	.byte	24
	.byte	16
	.type	__func__.21937, %object
	.size	__func__.21937, 11
__func__.21937:
	.ascii	"FtlMemInit\000"
	.space	1
	.type	__func__.22680, %object
	.size	__func__.22680, 12
__func__.22680:
	.ascii	"FtlCheckVpc\000"
	.type	__func__.22715, %object
	.size	__func__.22715, 17
__func__.22715:
	.ascii	"FtlDumpBlockInfo\000"
	.space	3
	.type	__func__.22734, %object
	.size	__func__.22734, 16
__func__.22734:
	.ascii	"FtlScanAllBlock\000"
	.type	__func__.23000, %object
	.size	__func__.23000, 17
__func__.23000:
	.ascii	"ftl_scan_all_ppa\000"
	.space	3
	.type	__func__.22982, %object
	.size	__func__.22982, 21
__func__.22982:
	.ascii	"FtlVpcCheckAndModify\000"
	.space	3
	.type	__func__.22010, %object
	.size	__func__.22010, 8
__func__.22010:
	.ascii	"FtlInit\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC1:
	.ascii	"FlashEraseBlocks pageAddr error %x\012\000"
.LC2:
	.ascii	"otp error! %d\000"
.LC3:
	.ascii	"rr\000"
.LC4:
	.ascii	"%d statReg->V6.mtrans_cnt=%d flReg.V6.page_num=%d\012"
	.ascii	"\000"
.LC5:
	.ascii	"nandc:\000"
.LC6:
	.ascii	"%d flReg.d32=%x %x\012\000"
.LC7:
	.ascii	"sdr read ok %x ecc=%d\012\000"
.LC8:
	.ascii	"sync para %d\012\000"
.LC9:
	.ascii	"TOG mode Read error %x %x\012\000"
.LC10:
	.ascii	"read retry status %x %x %x\012\000"
.LC11:
	.ascii	"micron RR %d row=%x,count %d,status=%d\012\000"
.LC12:
	.ascii	"ECC:%d\012\000"
.LC13:
	.ascii	"No.%d FLASH ID:%x %x %x %x %x %x\012\000"
.LC14:
	.ascii	"FlashLoadPhyInfo fail %x!!\012\000"
.LC15:
	.ascii	"Read pageadd=%x  ecc=%x err=%x\012\000"
.LC16:
	.ascii	"data:\000"
.LC17:
	.ascii	"spare:\000"
.LC18:
	.ascii	"ReadRetry pageadd=%x ecc=%x err=%x\012\000"
.LC19:
	.ascii	"FLFB:%d %d\012\000"
.LC20:
	.ascii	"BBT:\000"
.LC21:
	.ascii	"prog error: = %x\012\000"
.LC22:
	.ascii	"prog read error: = %x\012\000"
.LC23:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC24:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC25:
	.ascii	"FlashMakeFactorBbt %d\012\000"
.LC26:
	.ascii	"bad block:%d %d\012\000"
.LC27:
	.ascii	"FMFB:%d %d\012\000"
.LC28:
	.ascii	"E:bad block:%d\012\000"
.LC29:
	.ascii	"FMFB:Save %d %d\012\000"
.LC30:
	.ascii	"%s error allocating memory. return -1\012\000"
.LC31:
	.ascii	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\012"
	.ascii	"\000"
.LC32:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC33:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC34:
	.ascii	"FtlBbmTblFlush error = %x error count = %d\012\000"
.LC35:
	.ascii	"FtlFreeSysBlkQueueOut free count = %d\012\000"
.LC36:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\012"
	.ascii	"\000"
.LC37:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d\012\000"
.LC38:
	.ascii	"FtlMapWritePage error = %x\012\000"
.LC39:
	.ascii	"FtlMapWritePage error = %x error count = %d\012\000"
.LC40:
	.ascii	"page map lost: %x %x\012\000"
.LC41:
	.ascii	"region_id = %x phyAddr = %x\012\000"
.LC42:
	.ascii	"map_ppn:\000"
.LC43:
	.ascii	"load_l2p_region refresh = %x phyAddr = %x\012\000"
.LC44:
	.ascii	"FtlVendorPartRead refresh = %x phyAddr = %x\012\000"
.LC45:
	.ascii	"FtlVpcTblFlush error = %x error count = %d\012\000"
.LC46:
	.ascii	"%s\012\000"
.LC47:
	.ascii	"no ect\000"
.LC48:
	.ascii	"...%s enter...\012\000"
.LC49:
	.ascii	"FtlCheckVpc2 %x = %x  %x\012\000"
.LC50:
	.ascii	"free blk vpc error %x = %x  %x\012\000"
.LC51:
	.ascii	"id = %x,%x addr= %x,spare= %x %x %x %x data = %x\012"
	.ascii	"\000"
.LC52:
	.ascii	":\000"
.LC53:
	.ascii	"Ftlscanalldata = %x\012\000"
.LC54:
	.ascii	"scan lpa = %x ppa= %x\012\000"
.LC55:
	.ascii	"lba = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC56:
	.ascii	"phyBlk = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC57:
	.ascii	"id = %x,%x addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC58:
	.ascii	"Mblk:\000"
.LC59:
	.ascii	"L2P:\000"
.LC60:
	.ascii	"L2PC:\000"
.LC61:
	.ascii	"superBlkID = %x vpc=%x\012\000"
.LC62:
	.ascii	"flashmode = %x pagenum = %x %x\012\000"
.LC63:
	.ascii	"blk = %x vpc=%x mode = %x\012\000"
.LC64:
	.ascii	"mlc id = %x,%x addr= %x,spare= %x %x %x %x data=%x "
	.ascii	"%x\012\000"
.LC65:
	.ascii	"slc id = %x,%x addr= %x,spare= %x %x %x %x data=%x "
	.ascii	"%x\012\000"
.LC66:
	.ascii	"slc mode\000"
.LC67:
	.ascii	"ftl_scan_all_ppa blk %x page %x\012\000"
.LC68:
	.ascii	"addr= %x, status= %d,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC69:
	.ascii	"%s finished\012\000"
.LC70:
	.ascii	"FLASH INFO:\012\000"
.LC71:
	.ascii	"FLASH ID: %x\012\000"
.LC72:
	.ascii	"Device Capacity: %d MB\012\000"
.LC73:
	.ascii	"FMWAIT: %x %x %x %x\012\000"
.LC74:
	.ascii	"FTL INFO:\012\000"
.LC75:
	.ascii	"g_MaxLpn = 0x%x\012\000"
.LC76:
	.ascii	"g_VaildLpn = 0x%x\012\000"
.LC77:
	.ascii	"read_page_count = 0x%x\012\000"
.LC78:
	.ascii	"discard_page_count = 0x%x\012\000"
.LC79:
	.ascii	"write_page_count = 0x%x\012\000"
.LC80:
	.ascii	"cache_write_count = 0x%x\012\000"
.LC81:
	.ascii	"l2p_write_count = 0x%x\012\000"
.LC82:
	.ascii	"gc_page_count = 0x%x\012\000"
.LC83:
	.ascii	"totle_write = %d MB\012\000"
.LC84:
	.ascii	"totle_read = %d MB\012\000"
.LC85:
	.ascii	"GSV = 0x%x\012\000"
.LC86:
	.ascii	"GDV = 0x%x\012\000"
.LC87:
	.ascii	"bad blk num = %d %d\012\000"
.LC88:
	.ascii	"free_superblocks = 0x%x\012\000"
.LC89:
	.ascii	"mlc_EC = 0x%x\012\000"
.LC90:
	.ascii	"slc_EC = 0x%x\012\000"
.LC91:
	.ascii	"avg_EC = 0x%x\012\000"
.LC92:
	.ascii	"sys_EC = 0x%x\012\000"
.LC93:
	.ascii	"max_EC = 0x%x\012\000"
.LC94:
	.ascii	"min_EC = 0x%x\012\000"
.LC95:
	.ascii	"PLT = 0x%x\012\000"
.LC96:
	.ascii	"POT = 0x%x\012\000"
.LC97:
	.ascii	"MaxSector = 0x%x\012\000"
.LC98:
	.ascii	"init_sys_blks_pp = 0x%x\012\000"
.LC99:
	.ascii	"sys_blks_pp = 0x%x\012\000"
.LC100:
	.ascii	"free sysblock = 0x%x\012\000"
.LC101:
	.ascii	"data_blks_pp = 0x%x\012\000"
.LC102:
	.ascii	"data_op_blks_pp = 0x%x\012\000"
.LC103:
	.ascii	"max_data_blks = 0x%x\012\000"
.LC104:
	.ascii	"Sys.id = 0x%x\012\000"
.LC105:
	.ascii	"Bbt.id = 0x%x\012\000"
.LC106:
	.ascii	"ACT.page = 0x%x\012\000"
.LC107:
	.ascii	"ACT.plane = 0x%x\012\000"
.LC108:
	.ascii	"ACT.id = 0x%x\012\000"
.LC109:
	.ascii	"ACT.mode = 0x%x\012\000"
.LC110:
	.ascii	"ACT.a_pages = 0x%x\012\000"
.LC111:
	.ascii	"ACT VPC = 0x%x\012\000"
.LC112:
	.ascii	"BUF.page = 0x%x\012\000"
.LC113:
	.ascii	"BUF.plane = 0x%x\012\000"
.LC114:
	.ascii	"BUF.id = 0x%x\012\000"
.LC115:
	.ascii	"BUF.mode = 0x%x\012\000"
.LC116:
	.ascii	"BUF.a_pages = 0x%x\012\000"
.LC117:
	.ascii	"BUF VPC = 0x%x\012\000"
.LC118:
	.ascii	"TMP.page = 0x%x\012\000"
.LC119:
	.ascii	"TMP.plane = 0x%x\012\000"
.LC120:
	.ascii	"TMP.id = 0x%x\012\000"
.LC121:
	.ascii	"TMP.mode = 0x%x\012\000"
.LC122:
	.ascii	"TMP.a_pages = 0x%x\012\000"
.LC123:
	.ascii	"GC.page = 0x%x\012\000"
.LC124:
	.ascii	"GC.plane = 0x%x\012\000"
.LC125:
	.ascii	"GC.id = 0x%x\012\000"
.LC126:
	.ascii	"GC.mode = 0x%x\012\000"
.LC127:
	.ascii	"GC.a_pages = 0x%x\012\000"
.LC128:
	.ascii	"WR_CHK = 0x%x %x %x %x\012\000"
.LC129:
	.ascii	"Read Err = 0x%x\012\000"
.LC130:
	.ascii	"Prog Err = 0x%x\012\000"
.LC131:
	.ascii	"gc_free_blk_th= 0x%x\012\000"
.LC132:
	.ascii	"gc_merge_free_blk_th= 0x%x\012\000"
.LC133:
	.ascii	"gc_skip_write_count= 0x%x\012\000"
.LC134:
	.ascii	"gc_blk_index= 0x%x\012\000"
.LC135:
	.ascii	"free min EC= 0x%x\012\000"
.LC136:
	.ascii	"free max EC= 0x%x\012\000"
.LC137:
	.ascii	"GC__SB VPC = 0x%x\012\000"
.LC138:
	.ascii	"%d. [0x%x]=0x%x 0x%x  0x%x\012\000"
.LC139:
	.ascii	"free %d. [0x%x] 0x%x  0x%x\012\000"
.LC140:
	.ascii	"FTL version: 5.0.56 20190508\000"
.LC141:
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC142:
	.ascii	"FtlGcScanTempBlk Error ID %x %x!!!!!!! \012\000"
.LC143:
	.ascii	"FtlGcScanTempBlkError ID %x %x!!!!!!!\012\000"
.LC144:
	.ascii	"FtlGcRefreshBlock  0x%x\012\000"
.LC145:
	.ascii	"FtlGcMarkBadPhyBlk %d 0x%x\012\000"
.LC146:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
.LC147:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC148:
	.ascii	"decrement_vpc_count %x = %d in free list\012\000"
.LC149:
	.ascii	"RSB refresh addr %x\012\000"
.LC150:
	.ascii	"spuer block %x vpn is 0\012 \000"
.LC151:
	.ascii	"g_recovery_ppa %x ver %x\012 \000"
.LC152:
	.ascii	"FtlCheckVpc %x = %x  %x\012\000"
.LC153:
	.ascii	"%d GC datablk  = %x vpc %x %x\012\000"
.LC154:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC155:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC156:
	.ascii	"GC des block %x done\012\000"
.LC157:
	.ascii	"...%s: no bad block mapping table, format device\012"
	.ascii	"\000"
.LC158:
	.ascii	"...%s FtlSysBlkInit error ,format device!\012\000"
.LC159:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
.LC160:
	.ascii	"fix power lost blk = %x vpc=%x\012\000"
.LC161:
	.ascii	"erase power lost blk = %x vpc=%x\012\000"
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
	.type	gFtlInitStatus, %object
	.size	gFtlInitStatus, 4
gFtlInitStatus:
	.word	-1
	.type	ftl_gc_temp_block_bops_scan_page_addr, %object
	.size	ftl_gc_temp_block_bops_scan_page_addr, 2
ftl_gc_temp_block_bops_scan_page_addr:
	.short	-1
	.space	2
	.type	power_up_flag, %object
	.size	power_up_flag, 4
power_up_flag:
	.word	1
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
	.type	gTotleBlock, %object
	.size	gTotleBlock, 2
gTotleBlock:
	.space	2
	.space	2
	.type	gNandIDataBuf, %object
	.size	gNandIDataBuf, 2048
gNandIDataBuf:
	.space	2048
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
	.type	gReadRetryInfo, %object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.space	852
	.type	read_retry_cur_offset, %object
	.size	read_retry_cur_offset, 4
read_retry_cur_offset:
	.space	4
	.type	gFlashInterfaceMode, %object
	.size	gFlashInterfaceMode, 1
gFlashInterfaceMode:
	.space	1
	.space	3
	.type	gFlashPageBuffer0, %object
	.size	gFlashPageBuffer0, 4
gFlashPageBuffer0:
	.space	4
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
	.type	gNandFlashIDBEccBits, %object
	.size	gNandFlashIDBEccBits, 1
gNandFlashIDBEccBits:
	.space	1
	.space	2
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
	.type	g_free_slc_blk_num, %object
	.size	g_free_slc_blk_num, 2
g_free_slc_blk_num:
	.space	2
	.type	g_SlcPartLbaEndSector, %object
	.size	g_SlcPartLbaEndSector, 4
g_SlcPartLbaEndSector:
	.space	4
	.type	g_all_blk_used_slc_mode, %object
	.size	g_all_blk_used_slc_mode, 4
g_all_blk_used_slc_mode:
	.space	4
	.type	g_GlobalSysVersion, %object
	.size	g_GlobalSysVersion, 4
g_GlobalSysVersion:
	.space	4
	.type	g_GlobalDataVersion, %object
	.size	g_GlobalDataVersion, 4
g_GlobalDataVersion:
	.space	4
	.type	g_totle_gc_page_count, %object
	.size	g_totle_gc_page_count, 4
g_totle_gc_page_count:
	.space	4
	.type	g_totle_write_page_count, %object
	.size	g_totle_write_page_count, 4
g_totle_write_page_count:
	.space	4
	.type	g_totle_discard_page_count, %object
	.size	g_totle_discard_page_count, 4
g_totle_discard_page_count:
	.space	4
	.type	g_totle_cache_write_count, %object
	.size	g_totle_cache_write_count, 4
g_totle_cache_write_count:
	.space	4
	.type	g_totle_l2p_write_count, %object
	.size	g_totle_l2p_write_count, 4
g_totle_l2p_write_count:
	.space	4
	.type	g_totle_read_page_count, %object
	.size	g_totle_read_page_count, 4
g_totle_read_page_count:
	.space	4
	.type	g_totle_mlc_erase_count, %object
	.size	g_totle_mlc_erase_count, 4
g_totle_mlc_erase_count:
	.space	4
	.type	g_totle_slc_erase_count, %object
	.size	g_totle_slc_erase_count, 4
g_totle_slc_erase_count:
	.space	4
	.type	g_totle_sys_slc_erase_count, %object
	.size	g_totle_sys_slc_erase_count, 4
g_totle_sys_slc_erase_count:
	.space	4
	.type	g_max_erase_count, %object
	.size	g_max_erase_count, 4
g_max_erase_count:
	.space	4
	.type	g_min_erase_count, %object
	.size	g_min_erase_count, 4
g_min_erase_count:
	.space	4
	.type	g_in_gc_progress, %object
	.size	g_in_gc_progress, 4
g_in_gc_progress:
	.space	4
	.type	g_in_swl_replace, %object
	.size	g_in_swl_replace, 4
g_in_swl_replace:
	.space	4
	.type	g_gc_head_data_block, %object
	.size	g_gc_head_data_block, 4
g_gc_head_data_block:
	.space	4
	.type	g_gc_head_data_block_count, %object
	.size	g_gc_head_data_block_count, 4
g_gc_head_data_block_count:
	.space	4
	.type	g_gc_skip_write_count, %object
	.size	g_gc_skip_write_count, 4
g_gc_skip_write_count:
	.space	4
	.type	g_cur_erase_blk, %object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.space	4
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
	.type	g_gc_free_blk_threshold, %object
	.size	g_gc_free_blk_threshold, 2
g_gc_free_blk_threshold:
	.space	2
	.type	g_gc_merge_free_blk_threshold, %object
	.size	g_gc_merge_free_blk_threshold, 2
g_gc_merge_free_blk_threshold:
	.space	2
	.type	g_gc_blk_index, %object
	.size	g_gc_blk_index, 2
g_gc_blk_index:
	.space	2
	.type	g_gc_bad_block_temp_num, %object
	.size	g_gc_bad_block_temp_num, 2
g_gc_bad_block_temp_num:
	.space	2
	.type	g_gc_refresh_block_temp_num, %object
	.size	g_gc_refresh_block_temp_num, 2
g_gc_refresh_block_temp_num:
	.space	2
	.type	g_gc_bad_block_gc_index, %object
	.size	g_gc_bad_block_gc_index, 2
g_gc_bad_block_gc_index:
	.space	2
	.type	c_wr_page_buf_num, %object
	.size	c_wr_page_buf_num, 4
c_wr_page_buf_num:
	.space	4
	.type	g_wr_page_num, %object
	.size	g_wr_page_num, 4
g_wr_page_num:
	.space	4
	.type	p_gc_blk_tbl, %object
	.size	p_gc_blk_tbl, 4
p_gc_blk_tbl:
	.space	4
	.type	p_gc_page_info, %object
	.size	p_gc_page_info, 4
p_gc_page_info:
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
	.type	req_erase, %object
	.size	req_erase, 4
req_erase:
	.space	4
	.type	req_gc, %object
	.size	req_gc, 4
req_gc:
	.space	4
	.type	req_wr_io, %object
	.size	req_wr_io, 4
req_wr_io:
	.space	4
	.type	c_gc_page_buf_num, %object
	.size	c_gc_page_buf_num, 4
c_gc_page_buf_num:
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
	.type	p_gc_data_buf, %object
	.size	p_gc_data_buf, 4
p_gc_data_buf:
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
	.type	gp_gc_page_buf_info, %object
	.size	gp_gc_page_buf_info, 4
gp_gc_page_buf_info:
	.space	4
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 4
p_sys_spare_buf:
	.space	4
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 4
p_io_spare_buf:
	.space	4
	.type	p_gc_spare_buf, %object
	.size	p_gc_spare_buf, 4
p_gc_spare_buf:
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
	.type	p_erase_count_table, %object
	.size	p_erase_count_table, 4
p_erase_count_table:
	.space	4
	.type	p_valid_page_count_check_table, %object
	.size	p_valid_page_count_check_table, 4
p_valid_page_count_check_table:
	.space	4
	.type	p_valid_page_count_table, %object
	.size	p_valid_page_count_table, 4
p_valid_page_count_table:
	.space	4
	.type	p_map_block_table, %object
	.size	p_map_block_table, 4
p_map_block_table:
	.space	4
	.type	p_map_block_valid_page_count, %object
	.size	p_map_block_valid_page_count, 4
p_map_block_valid_page_count:
	.space	4
	.type	p_blk_mode_table, %object
	.size	p_blk_mode_table, 4
p_blk_mode_table:
	.space	4
	.type	p_vendor_block_table, %object
	.size	p_vendor_block_table, 4
p_vendor_block_table:
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
	.type	p_l2p_ram_map, %object
	.size	p_l2p_ram_map, 4
p_l2p_ram_map:
	.space	4
	.type	p_l2p_map_buf, %object
	.size	p_l2p_map_buf, 4
p_l2p_map_buf:
	.space	4
	.type	p_data_block_list_table, %object
	.size	p_data_block_list_table, 4
p_data_block_list_table:
	.space	4
	.type	c_ftl_nand_bbm_buf_size, %object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
	.space	2
	.space	2
	.type	gBbtInfo, %object
	.size	gBbtInfo, 60
gBbtInfo:
	.space	60
	.type	g_totle_vendor_block, %object
	.size	g_totle_vendor_block, 2
g_totle_vendor_block:
	.space	2
	.space	2
	.type	g_MaxLpn, %object
	.size	g_MaxLpn, 4
g_MaxLpn:
	.space	4
	.type	g_flash_read_only_en, %object
	.size	g_flash_read_only_en, 4
g_flash_read_only_en:
	.space	4
	.type	req_sys, %object
	.size	req_sys, 36
req_sys:
	.space	36
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.space	2056
	.type	g_sys_save_data, %object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.space	48
	.type	p_data_block_list_head, %object
	.size	p_data_block_list_head, 4
p_data_block_list_head:
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
	.type	gL2pMapInfo, %object
	.size	gL2pMapInfo, 44
gL2pMapInfo:
	.space	44
	.type	g_l2p_last_update_region_id, %object
	.size	g_l2p_last_update_region_id, 2
g_l2p_last_update_region_id:
	.space	2
	.space	2
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 44
gVendorBlkInfo:
	.space	44
	.type	FtlUpdateVaildLpnCount, %object
	.size	FtlUpdateVaildLpnCount, 2
FtlUpdateVaildLpnCount:
	.space	2
	.space	2
	.type	g_VaildLpn, %object
	.size	g_VaildLpn, 4
g_VaildLpn:
	.space	4
	.type	gSysInfo, %object
	.size	gSysInfo, 16
gSysInfo:
	.space	16
	.type	g_totle_map_block, %object
	.size	g_totle_map_block, 2
g_totle_map_block:
	.space	2
	.space	2
	.type	check_valid_page_count_table, %object
	.size	check_valid_page_count_table, 8192
check_valid_page_count_table:
	.space	8192
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.space	4
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.space	2
	.space	2
	.type	g_gc_superblock, %object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.space	48
	.type	g_sys_ext_data, %object
	.size	g_sys_ext_data, 512
g_sys_ext_data:
	.space	512
	.type	g_totle_write_sector, %object
	.size	g_totle_write_sector, 4
g_totle_write_sector:
	.space	4
	.type	g_totle_read_sector, %object
	.size	g_totle_read_sector, 4
g_totle_read_sector:
	.space	4
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.space	2
	.space	2
	.type	g_totle_avg_erase_count, %object
	.size	g_totle_avg_erase_count, 4
g_totle_avg_erase_count:
	.space	4
	.type	g_gc_num_req, %object
	.size	g_gc_num_req, 4
g_gc_num_req:
	.space	4
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
	.type	g_gc_blk_num, %object
	.size	g_gc_blk_num, 2
g_gc_blk_num:
	.space	2
	.type	g_gc_page_offset, %object
	.size	g_gc_page_offset, 2
g_gc_page_offset:
	.space	2
	.type	g_gc_bad_block_temp_tbl, %object
	.size	g_gc_bad_block_temp_tbl, 34
g_gc_bad_block_temp_tbl:
	.space	34
	.type	g_power_lost_ecc_error_blk, %object
	.size	g_power_lost_ecc_error_blk, 2
g_power_lost_ecc_error_blk:
	.space	2
	.type	g_power_lost_recovery_flag, %object
	.size	g_power_lost_recovery_flag, 2
g_power_lost_recovery_flag:
	.space	2
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
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.space	4
	.type	last_cache_match_count, %object
	.size	last_cache_match_count, 4
last_cache_match_count:
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
