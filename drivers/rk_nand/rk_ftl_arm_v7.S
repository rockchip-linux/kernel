/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2017-08-16
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
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L9
	ldrb	r3, [r1, #1]	@ zero_extendqisi2
	ldrb	ip, [r0, #1]	@ zero_extendqisi2
	cmp	ip, r3
	movne	r3, #0
	bne	.L9
	b	.L10
.L6:
	ldrb	r4, [r0, r3]	@ zero_extendqisi2
	ldrb	ip, [r1, r3]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r4, ip
	beq	.L9
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L9:
	cmp	r3, r2
	bne	.L6
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L10:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
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
	ldr	r2, .L18
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r3, r3, asl #1
	ldrh	r4, [r2, r3]
	ldr	r3, .L18+4
	ldrb	r3, [r3, #8]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L17
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r4, r4, #-1073741824
.L17:
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L19:
	.align	2
.L18:
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
	ldr	r2, .L22
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r3, r3, asl #1
	mov	r6, r0
	ldrh	r4, [r2, r3]
	ldr	r3, .L22+4
	ldrb	r2, [r3, #8]	@ zero_extendqisi2
	mov	r5, r3
	cmp	r2, #0
	beq	.L21
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r4, r4, #-1073741824
.L21:
	add	r5, r5, r6, asl #3
	ldr	r3, [r5, #12]
	str	r4, [r3, #336]
	ldmfd	sp!, {r4, r5, r6, pc}
.L23:
	.align	2
.L22:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashSetRandomizer, .-FlashSetRandomizer
	.align	2
	.global	FlashReadCmd
	.type	FlashReadCmd, %function
FlashReadCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L25
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, ip, r0, asl #3
	ldrb	r3, [ip, #16]	@ zero_extendqisi2
	ldr	r4, [ip, #12]
	mov	ip, #0
	add	r3, r3, #8
	add	r3, r4, r3, asl #8
	str	ip, [r3, #8]
	str	ip, [r3, #4]
	str	ip, [r3, #4]
	uxtb	ip, r1
	str	ip, [r3, #4]
	mov	ip, r1, lsr #8
	str	ip, [r3, #4]
	str	r2, [r3, #4]
	mov	r2, #48
	str	r2, [r3, #8]
	ldmfd	sp!, {r4, lr}
	b	FlashSetRandomizer
.L26:
	.align	2
.L25:
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
	ldr	r2, .L31
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, r2, r0, asl #3
	ldrb	r2, [r2, #60]	@ zero_extendqisi2
	ldrb	r3, [ip, #16]	@ zero_extendqisi2
	ldr	r4, [ip, #12]
	cmp	r2, #1
	add	r3, r3, #8
	mov	r2, #0
	add	r3, r4, r3, asl #8
	bne	.L28
	mov	ip, #6
	str	ip, [r3, #8]
	str	r2, [r3, #4]
	str	r2, [r3, #4]
	uxtb	r2, r1
	str	r2, [r3, #4]
	mov	r2, r1, lsr #8
	str	r2, [r3, #4]
	mov	r2, r1, lsr #16
	b	.L30
.L28:
	str	r2, [r3, #8]
	uxtb	ip, r1
	str	r2, [r3, #4]
	str	r2, [r3, #4]
	str	ip, [r3, #4]
	mov	ip, r1, lsr #8
	str	ip, [r3, #4]
	mov	ip, r1, lsr #16
	str	ip, [r3, #4]
	mov	ip, #5
	str	ip, [r3, #8]
	str	r2, [r3, #4]
.L30:
	str	r2, [r3, #4]
	mov	r2, #224
	str	r2, [r3, #8]
	ldmfd	sp!, {r4, lr}
	b	FlashSetRandomizer
.L32:
	.align	2
.L31:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadDpDataOutCmd, .-FlashReadDpDataOutCmd
	.align	2
	.global	flash_enter_slc_mode
	.type	flash_enter_slc_mode, %function
flash_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L35
	ldrb	r2, [r3, #76]	@ zero_extendqisi2
	cmp	r2, #0
	bxeq	lr
	add	r0, r3, r0, asl #3
	ldrb	r3, [r0, #16]	@ zero_extendqisi2
	ldr	r2, [r0, #12]
	add	r3, r3, #8
	add	r3, r2, r3, asl #8
	mov	r2, #218
	str	r2, [r3, #8]
	bx	lr
.L36:
	.align	2
.L35:
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
	@ link register save eliminated.
	ldr	r3, .L39
	ldrb	r2, [r3, #76]	@ zero_extendqisi2
	cmp	r2, #0
	bxeq	lr
	add	r0, r3, r0, asl #3
	ldrb	r3, [r0, #16]	@ zero_extendqisi2
	ldr	r2, [r0, #12]
	add	r3, r3, #8
	add	r3, r2, r3, asl #8
	mov	r2, #223
	str	r2, [r3, #8]
	bx	lr
.L40:
	.align	2
.L39:
	.word	.LANCHOR0
	.fnend
	.size	flash_exit_slc_mode, .-flash_exit_slc_mode
	.align	2
	.global	FlashProgFirstCmd
	.type	FlashProgFirstCmd, %function
FlashProgFirstCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L42
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, ip, r0, asl #3
	ldrb	r3, [ip, #16]	@ zero_extendqisi2
	ldr	r4, [ip, #12]
	mov	ip, #128
	add	r3, r3, #8
	add	r3, r4, r3, asl #8
	str	ip, [r3, #8]
	mov	ip, #0
	str	ip, [r3, #4]
	str	ip, [r3, #4]
	uxtb	ip, r1
	str	ip, [r3, #4]
	mov	ip, r1, lsr #8
	str	ip, [r3, #4]
	str	r2, [r3, #4]
	ldmfd	sp!, {r4, lr}
	b	FlashSetRandomizer
.L43:
	.align	2
.L42:
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
	@ link register save eliminated.
	ldr	r3, .L46
	cmp	r2, #0
	add	r0, r3, r0, asl #3
	ldr	ip, [r0, #12]
	ldrb	r0, [r0, #16]	@ zero_extendqisi2
	add	r0, r0, #8
	add	r0, ip, r0, asl #8
	beq	.L45
	ldr	r3, [r3, #4]
	mov	r2, #96
	str	r2, [r0, #8]
	uxtb	r2, r1
	str	r2, [r0, #4]
	mov	r2, r1, lsr #8
	str	r2, [r0, #4]
	mov	r2, r1, lsr #16
	add	r1, r1, r3
	str	r2, [r0, #4]
.L45:
	mov	r3, #96
	str	r3, [r0, #8]
	uxtb	r3, r1
	str	r3, [r0, #4]
	mov	r3, r1, lsr #8
	mov	r1, r1, lsr #16
	str	r3, [r0, #4]
	mov	r3, #208
	str	r1, [r0, #4]
	str	r3, [r0, #8]
	bx	lr
.L47:
	.align	2
.L46:
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
	ldr	ip, .L49
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	add	r4, ip, r0, asl #3
	ldrb	ip, [ip, #55]	@ zero_extendqisi2
	ldrb	r3, [r4, #16]	@ zero_extendqisi2
	ldr	r5, [r4, #12]
	add	r3, r3, #8
	add	r3, r5, r3, asl #8
	str	ip, [r3, #8]
	mov	ip, #0
	str	ip, [r3, #4]
	str	ip, [r3, #4]
	uxtb	ip, r1
	str	ip, [r3, #4]
	mov	ip, r1, lsr #8
	str	ip, [r3, #4]
	str	r2, [r3, #4]
	ldmfd	sp!, {r4, r5, lr}
	b	FlashSetRandomizer
.L50:
	.align	2
.L49:
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
	ldr	r3, .L52
	add	r3, r3, r0, asl #3
	ldr	r2, [r3, #12]
	ldrb	r3, [r3, #16]	@ zero_extendqisi2
	add	r3, r3, #8
	add	r3, r2, r3, asl #8
	mov	r2, #16
	str	r2, [r3, #8]
	bx	lr
.L53:
	.align	2
.L52:
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
	ldr	r3, .L55
	add	r0, r3, r0, asl #3
	ldrb	r3, [r3, #54]	@ zero_extendqisi2
	ldrb	r2, [r0, #16]	@ zero_extendqisi2
	ldr	r1, [r0, #12]
	add	r2, r2, #8
	add	r2, r1, r2, asl #8
	str	r3, [r2, #8]
	bx	lr
.L56:
	.align	2
.L55:
	.word	.LANCHOR0
	.fnend
	.size	FlashProgDpFirstCmd, .-FlashProgDpFirstCmd
	.align	2
	.global	JSHash
	.type	JSHash, %function
JSHash:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	r3, r0
	mov	r2, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r0, .L60
	b	.L58
.L59:
	mov	ip, r0, asl #5
	ldrb	r4, [r3, r2]	@ zero_extendqisi2
	add	ip, ip, r0, lsr #2
	add	r2, r2, #1
	add	ip, ip, r4
	eor	r0, r0, ip
.L58:
	cmp	r2, r1
	bne	.L59
	ldmfd	sp!, {r4, pc}
.L61:
	.align	2
.L60:
	.word	1204201446
	.fnend
	.size	JSHash, .-JSHash
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
	.global	FlashReadIdbData
	.type	FlashReadIdbData, %function
FlashReadIdbData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	mov	r2, #2048
	ldr	r1, .L65
	bl	memcpy
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L66:
	.align	2
.L65:
	.word	.LANCHOR0+80
	.fnend
	.size	FlashReadIdbData, .-FlashReadIdbData
	.align	2
	.global	FlashLoadPhyInfoInRam
	.type	FlashLoadPhyInfoInRam, %function
FlashLoadPhyInfoInRam:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, #0
	ldr	r6, .L79
.L71:
	add	r5, r6, r4, asl #5
	ldr	r1, .L79+4
	add	r0, r5, #1
	ldrb	r2, [r6, r4, asl #5]	@ zero_extendqisi2
	bl	FlashMemCmp8
	subs	r1, r0, #0
	bne	.L68
	cmp	r5, #0
	ldrneb	r2, [r5, #22]	@ zero_extendqisi2
	ldrne	r3, .L79+8
	bne	.L73
	b	.L78
.L68:
	add	r4, r4, #1
	cmp	r4, #72
	bne	.L71
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L73:
	ldrb	r0, [r3, r1, asl #5]	@ zero_extendqisi2
	cmp	r0, r2
	beq	.L72
	add	r1, r1, #1
	cmp	r1, #4
	bne	.L73
.L72:
	ldr	r3, .L79+8
	mov	r2, #32
	ldr	r0, .L79+12
	add	r1, r3, r1, asl #5
	bl	memcpy
	ldr	r0, .L79+16
	mov	r1, r5
	mov	r2, #32
	bl	memcpy
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L78:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L80:
	.align	2
.L79:
	.word	.LANCHOR1+256
	.word	.LANCHOR0+2128
	.word	.LANCHOR1+2560
	.word	.LANCHOR0+44
	.word	.LANCHOR1+2688
	.fnend
	.size	FlashLoadPhyInfoInRam, .-FlashLoadPhyInfoInRam
	.align	2
	.global	FlashSuspend
	.type	FlashSuspend, %function
FlashSuspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L82
	mov	r0, #0
	ldr	r2, [r3, #2160]
	ldr	r1, [r2, #0]
	str	r1, [r3, #2164]
	ldr	r1, [r2, #4]
	str	r1, [r3, #2168]
	ldr	r1, [r2, #8]
	str	r1, [r3, #2172]
	ldr	r1, [r2, #12]
	str	r1, [r3, #2176]
	ldr	r1, [r2, #304]
	str	r1, [r3, #2180]
	ldr	r1, [r2, #308]
	str	r1, [r3, #2184]
	ldr	r1, [r2, #336]
	ldr	r2, [r2, #344]
	str	r1, [r3, #2188]
	str	r2, [r3, #2192]
	bx	lr
.L83:
	.align	2
.L82:
	.word	.LANCHOR0
	.fnend
	.size	FlashSuspend, .-FlashSuspend
	.global	__aeabi_uidiv
	.align	2
	.global	LogAddr2PhyAddr
	.type	LogAddr2PhyAddr, %function
LogAddr2PhyAddr:
	.fnstart
	@ args = 4, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	fp, r3
	ldr	r3, .L89
	mov	r5, r1
	mov	r6, r2
	movw	r1, #2210
	mov	r2, #2208
	ldr	r8, [r0, #4]
	ldrh	r2, [r3, r2]
	mov	r4, r0
	ldrh	sl, [r3, r1]
	bic	r8, r8, #-2147483648
	ldrh	r7, [r3, #4]
	ubfx	r9, r8, #10, #16
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	mov	r8, r8, asl #22
	mul	sl, sl, r2
	cmp	r3, #1
	moveq	r7, r7, asl #1
	mov	r0, r9
	mov	r8, r8, lsr #22
	uxtheq	r7, r7
	uxth	sl, sl
	mov	r1, sl
	bl	__aeabi_uidiv
	cmp	r5, #1
	ldr	r3, .L89
	uxth	r0, r0
	mls	sl, sl, r0, r9
	uxth	sl, sl
	bne	.L86
	ldrb	r2, [r3, #76]	@ zero_extendqisi2
	cmp	r2, #0
	addeq	r8, r3, r8, asl #1
	addeq	r8, r8, #2224
	ldreqh	r8, [r8, #0]
.L86:
	add	r3, r3, r0, asl #2
	ldr	r3, [r3, #2736]
	mla	r7, r7, sl, r3
	ldrb	r3, [sp, #40]	@ zero_extendqisi2
	cmp	r3, #1
	add	r8, r7, r8
	str	r8, [r6, #0]
	str	r0, [fp, #0]
	bls	.L88
	ldr	r0, [r4, #4]
	ldr	r3, [r4, #40]
	add	r0, r0, #1024
	rsb	r3, r3, r0
	rsbs	r0, r3, #0
	adc	r0, r0, r3
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L88:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L90:
	.align	2
.L89:
	.word	.LANCHOR0
	.fnend
	.size	LogAddr2PhyAddr, .-LogAddr2PhyAddr
	.align	2
	.global	FlashScheduleEnSet
	.type	FlashScheduleEnSet, %function
FlashScheduleEnSet:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L92
	ldr	r2, [r3, #2768]
	str	r0, [r3, #2768]
	mov	r0, r2
	bx	lr
.L93:
	.align	2
.L92:
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
	ldr	r3, .L95
	ldr	r3, [r3, #2772]
	ldrb	r0, [r3, #9]	@ zero_extendqisi2
	bx	lr
.L96:
	.align	2
.L95:
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
	ldr	r3, .L99
	add	r3, r3, r0, asl #3
	ldrb	r0, [r3, #16]	@ zero_extendqisi2
	ldr	r2, [r3, #12]
	add	r0, r0, #8
	add	r0, r2, r0, asl #8
	bx	lr
.L100:
	.align	2
.L99:
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
	ldr	r3, .L102
	mov	r2, r0, asl #8
	orr	r2, r2, r0, asl #16
	orr	r2, r2, #1
	ldr	r3, [r3, #2160]
	str	r2, [r3, #304]
	bx	lr
.L103:
	.align	2
.L102:
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
	ldr	r3, .L105
	orr	r0, r0, #16640
	ldr	r3, [r3, #2160]
	str	r0, [r3, #344]
	bx	lr
.L106:
	.align	2
.L105:
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
	ldr	r3, .L109
	cmp	r0, #0
	ldr	r2, [r3, #2160]
	ldr	r3, [r2, #0]
	bfc	r3, #13, #1
	orrne	r3, r3, #253952
	str	r3, [r2, #0]
	bx	lr
.L110:
	.align	2
.L109:
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
	ldr	r3, .L115
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r3, #2160]
	ldr	r4, [r3, #0]
	ands	r3, r0, #6
	bfieq	r4, r3, #13, #1
	beq	.L114
	orr	r4, r4, #24576
	tst	r0, #4
	bfc	r4, #15, #1
	mov	r0, #0
	orr	r4, r4, #196608
	orrne	r4, r4, #32768
	bl	rknand_get_clk_rate
	ldr	r3, .L115
	movw	r2, #16641
	ldr	r3, [r3, #2160]
	str	r2, [r3, #344]
	ldr	r2, .L115+4
	str	r2, [r3, #304]
	mov	r2, #38
	str	r2, [r3, #308]
	mov	r2, #39
	str	r2, [r3, #308]
.L114:
	ldr	r3, .L115
	mov	r0, #0
	ldr	r3, [r3, #2160]
	str	r4, [r3, #0]
	ldmfd	sp!, {r4, pc}
.L116:
	.align	2
.L115:
	.word	.LANCHOR0
	.word	1710595
	.fnend
	.size	NandcSetMode, .-NandcSetMode
	.align	2
	.global	NandcFlashCsDebug
	.type	NandcFlashCsDebug, %function
NandcFlashCsDebug:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L118
	ldr	r3, [r3, #12]
	ldr	r2, [r3, #0]
	bfi	r2, r0, #0, #8
	str	r2, [r3, #0]
	bx	lr
.L119:
	.align	2
.L118:
	.word	.LANCHOR0
	.fnend
	.size	NandcFlashCsDebug, .-NandcFlashCsDebug
	.align	2
	.global	NandcFlashCs
	.type	NandcFlashCs, %function
NandcFlashCs:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L121
	add	r3, r3, r0, asl #3
	mov	r0, #1
	ldr	r2, [r3, #12]
	ldrb	r1, [r3, #16]	@ zero_extendqisi2
	ldr	r3, [r2, #0]
	mov	r1, r0, asl r1
	bfi	r3, r1, #0, #8
	str	r3, [r2, #0]
	bx	lr
.L122:
	.align	2
.L121:
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
	ldr	r3, .L124
	add	r3, r3, r0, asl #3
	ldr	r2, [r3, #12]
	ldr	r3, [r2, #0]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2, #0]
	bx	lr
.L125:
	.align	2
.L124:
	.word	.LANCHOR0
	.fnend
	.size	NandcFlashDeCs, .-NandcFlashDeCs
	.align	2
	.global	NandcDelayns
	.type	NandcDelayns, %function
NandcDelayns:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #8
	sub	sp, sp, #8
	mov	r0, r0, lsr #4
	str	r0, [sp, #4]
.L127:
	ldr	r0, [sp, #4]
	cmp	r0, #0
	sub	r3, r0, #1
	str	r3, [sp, #4]
	bne	.L127
	add	sp, sp, #8
	bx	lr
	.fnend
	.size	NandcDelayns, .-NandcDelayns
	.align	2
	.global	FlashReadStatusEN
	.type	FlashReadStatusEN, %function
FlashReadStatusEN:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L138
	add	r0, r3, r0, asl #3
	ldr	ip, [r3, #2772]
	ldrb	r5, [r0, #16]	@ zero_extendqisi2
	ldr	r4, [r0, #12]
	ldrb	ip, [ip, #8]	@ zero_extendqisi2
	add	r5, r5, #8
	cmp	ip, #2
	add	r0, r4, r5, asl #8
	movne	r3, #112
	strne	r3, [r0, #8]
	bne	.L134
	cmp	r2, #0
	ldreqb	r2, [r3, #57]	@ zero_extendqisi2
	ldrneb	r2, [r3, #58]	@ zero_extendqisi2
	str	r2, [r0, #8]
	ldrb	r3, [r3, #59]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #0
	ldrne	r2, .L138
	bne	.L133
	b	.L134
.L135:
	mov	ip, r3, asl #3
	add	r3, r3, #1
	mov	ip, r1, lsr ip
	uxtb	ip, ip
	str	ip, [r0, #4]
.L133:
	ldrb	ip, [r2, #59]	@ zero_extendqisi2
	cmp	r3, ip
	bcc	.L135
.L134:
	mov	r0, #80
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	uxtb	r0, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L139:
	.align	2
.L138:
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
	mov	r6, r0
	mov	r5, r1
	mov	r4, r2
.L144:
	mov	r0, r6
	mov	r1, r5
	mov	r2, r4
	bl	FlashReadStatusEN
	cmp	r0, #255
	beq	.L144
	tst	r0, #64
	beq	.L144
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
	ldr	r3, .L148
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r4, r3, r0, asl #4
	ldrb	r5, [r3, r0, asl #4]	@ zero_extendqisi2
	mov	r6, r0
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L146
	mov	r0, r5
	bl	NandcFlashCs
	ldr	r3, .L148+4
	ldr	r1, [r4, #4]
	mov	r0, r5
	add	r6, r3, r6, asl #2
	ldr	r2, [r6, #2736]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r6, r0
	mov	r0, r5
	bl	NandcFlashDeCs
	ldr	r3, [r4, #8]
	ands	r6, r6, #1
	mvnne	r6, #0
	str	r6, [r3, #0]
	ldr	r2, [r4, #12]
	mov	r3, #0
	str	r3, [r4, #8]
	cmp	r2, r3
	strne	r6, [r2, #0]
	strne	r3, [r4, #12]
.L146:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L149:
	.align	2
.L148:
	.word	.LANCHOR0+2776
	.word	.LANCHOR0
	.fnend
	.size	FlashWaitCmdDone, .-FlashWaitCmdDone
	.align	2
	.global	HynixSetRRPara
	.type	HynixSetRRPara, %function
HynixSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	sl, r3
	ldr	r3, .L155
	mov	r6, r2
	mov	r4, r0
	mov	r5, r1
	mov	r7, #0
	ldr	r2, [r3, #2772]
	add	r3, r3, r0, asl #3
	ldrb	r8, [r3, #16]	@ zero_extendqisi2
	ldrb	r2, [r2, #19]	@ zero_extendqisi2
	add	r8, r8, #8
	cmp	r2, #6
	addne	r9, sl, r0, asl #3
	ldrne	r2, .L155+4
	ldreq	r9, .L155+4
	addne	r9, r2, r9, asl #3
	ldr	r2, [r3, #12]
	addeq	r9, r9, r0, asl #6
	add	r8, r2, r8, asl #8
	addeq	r9, r9, sl, asl #2
	bl	NandcFlashCs
	mov	r3, #54
	str	r3, [r8, #8]
	b	.L153
.L154:
	ldrb	r3, [r6, r7]	@ zero_extendqisi2
	mov	r0, #200
	str	r3, [r8, #4]
	bl	NandcDelayns
	ldrsb	r3, [r9, r7]
	add	r7, r7, #1
	str	r3, [r8, #0]
.L153:
	uxtb	r3, r7
	cmp	r3, r5
	bcc	.L154
	mov	r3, #22
	mov	r0, r4
	str	r3, [r8, #8]
	bl	NandcFlashDeCs
	ldr	r3, .L155
	add	r4, r3, r4
	strb	sl, [r4, #3756]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L156:
	.align	2
.L155:
	.word	.LANCHOR0
	.word	.LANCHOR0+2924
	.fnend
	.size	HynixSetRRPara, .-HynixSetRRPara
	.align	2
	.global	FlashSetReadRetryDefault
	.type	FlashSetReadRetryDefault, %function
FlashSetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r5, .L162
	ldr	r3, [r5, #2772]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #5
	ldmhifd	sp!, {r3, r4, r5, pc}
	mov	r4, #0
.L160:
	ldr	r2, .L162+4
	uxtb	r0, r4
	ldrb	r3, [r2, r4, asl #3]	@ zero_extendqisi2
	cmp	r3, #173
	bne	.L159
	ldrb	r1, [r5, #2905]	@ zero_extendqisi2
	add	r2, r2, #780
	mov	r3, #0
	bl	HynixSetRRPara
.L159:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L160
	ldmfd	sp!, {r3, r4, r5, pc}
.L163:
	.align	2
.L162:
	.word	.LANCHOR0
	.word	.LANCHOR0+2128
	.fnend
	.size	FlashSetReadRetryDefault, .-FlashSetReadRetryDefault
	.align	2
	.global	SamsungSetRRPara
	.type	SamsungSetRRPara, %function
SamsungSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	ldr	r3, .L167
	mov	r5, #0
	ldr	sl, .L167+4
	mov	r8, #161
	add	r1, r3, r1, asl #2
	add	r6, r1, #3
	mov	r7, r3
	b	.L165
.L166:
	str	r8, [r4, #8]
	mov	r3, #0
	str	r3, [r4, #0]
	mov	r0, #300
	ldrsb	r3, [r5, r7]
	add	r5, r5, #1
	str	r3, [r4, #0]
	ldrsb	r3, [r6, #1]!
	str	r3, [r4, #0]
	bl	NandcDelayns
.L165:
	ldrb	r3, [sl, #3760]	@ zero_extendqisi2
	cmp	r5, r3
	bcc	.L166
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L168:
	.align	2
.L167:
	.word	.LANCHOR1+2720
	.word	.LANCHOR0
	.fnend
	.size	SamsungSetRRPara, .-SamsungSetRRPara
	.align	2
	.global	ToshibaSetRRPara
	.type	ToshibaSetRRPara, %function
ToshibaSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	add	r6, r1, #1
	ldr	r3, .L176
	mov	r4, r0
	add	r6, r6, r6, asl #2
	mov	r5, #0
	add	r2, r3, #45
	sub	sl, r3, #2784
	add	r7, r3, r6
	ldr	r8, .L176+4
	add	r6, r2, r6
	add	sl, sl, r1
	b	.L170
.L174:
	mov	r3, #85
	str	r3, [r4, #8]
	ldr	r3, .L176
	mov	r0, #200
	ldrsb	r3, [r5, r3]
	str	r3, [r4, #4]
	bl	NandcDelayns
	ldrb	r3, [r8, #3761]	@ zero_extendqisi2
	cmp	r3, #34
	ldreqsb	r3, [r7, #0]
	beq	.L175
	cmp	r3, #35
	addne	r3, sl, #2912
	addne	r3, r3, #12
	ldreqsb	r3, [r6, #0]
	ldrnesb	r3, [r3, #0]
.L175:
	add	r5, r5, #1
	add	r7, r7, #1
	add	r6, r6, #1
	str	r3, [r4, #0]
.L170:
	ldrb	r3, [r8, #3760]	@ zero_extendqisi2
	cmp	r5, r3
	bcc	.L174
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L177:
	.align	2
.L176:
	.word	.LANCHOR1+2784
	.word	.LANCHOR0
	.fnend
	.size	ToshibaSetRRPara, .-ToshibaSetRRPara
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
	ldr	r3, .L179
	add	r3, r3, r0, asl #3
	mov	r0, #80
	ldrb	r5, [r3, #16]	@ zero_extendqisi2
	ldr	r4, [r3, #12]
	add	r5, r5, #8
	add	r3, r4, r5, asl #8
	str	r2, [r3, #8]
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	ldmfd	sp!, {r3, r4, r5, pc}
.L180:
	.align	2
.L179:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadStatus, .-FlashReadStatus
	.align	2
	.global	NandcWaitFlashReady
	.type	NandcWaitFlashReady, %function
NandcWaitFlashReady:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L186
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r0, r1, r2, r4, r5, lr}
	add	r3, r3, r0, asl #3
	ldr	r4, .L186+4
	ldr	r5, [r3, #12]
.L183:
	mov	r0, #100
	bl	NandcDelayns
	ldr	r3, [r5, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L184
	subs	r4, r4, #1
	bne	.L183
	mvn	r0, #0
	b	.L182
.L184:
	mov	r0, #0
.L182:
	ldmfd	sp!, {r1, r2, r3, r4, r5, pc}
.L187:
	.align	2
.L186:
	.word	.LANCHOR0
	.word	100000
	.fnend
	.size	NandcWaitFlashReady, .-NandcWaitFlashReady
	.align	2
	.global	FlashEraseSLc2KBlocks
	.type	FlashEraseSLc2KBlocks, %function
FlashEraseSLc2KBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #0
	.pad #20
	sub	sp, sp, #20
	mov	r8, r1
	mov	r4, r0
	mov	r7, r6
	ldr	r5, .L196
	b	.L189
.L195:
	add	r3, r6, r8
	add	r2, sp, #8
	mov	r0, r4
	mov	r1, #0
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #12
	bl	LogAddr2PhyAddr
	ldr	r3, [sp, #12]
	ldrb	r2, [r5, #3762]	@ zero_extendqisi2
	cmp	r3, r2
	mvncs	r3, #0
	strcs	r3, [r4, #0]
	bcs	.L191
	add	r2, r5, r3
	add	r3, r5, r3, asl #4
	ldrb	sl, [r2, #3764]	@ zero_extendqisi2
	strb	sl, [r3, #2776]
	mov	r0, sl
	bl	NandcWaitFlashReady
	mov	r0, sl
	bl	NandcFlashCs
	mov	r2, #0
	ldr	r1, [sp, #8]
	mov	r0, sl
	bl	FlashEraseCmd
	mov	r0, sl
	bl	NandcWaitFlashReady
	ldr	r1, [sp, #8]
	mov	r0, sl
	bl	FlashReadStatus
	ldr	r3, [sp, #8]
	mov	r2, #0
	ands	r0, r0, #1
	mvnne	r0, #0
	str	r0, [r4, #0]
	ldr	r1, [r5, #4]
	mov	r0, sl
	add	r1, r1, r3
	bl	FlashEraseCmd
	mov	r0, sl
	bl	NandcWaitFlashReady
	mov	r0, sl
	ldr	r1, [sp, #8]
	bl	FlashReadStatus
	tst	r0, #1
	mvnne	r3, #0
	strne	r3, [r4, #0]
	ldr	r3, [r4, #0]
	cmn	r3, #1
	bne	.L194
	ldr	r0, .L196+4
	ldr	r1, [sp, #8]
	bl	printk
.L194:
	mov	r0, sl
	bl	NandcFlashDeCs
.L191:
	sub	r6, r6, #1
	add	r7, r7, #1
	add	r4, r4, #36
	uxtb	r6, r6
.L189:
	cmp	r7, r8
	bne	.L195
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L197:
	.align	2
.L196:
	.word	.LANCHOR0
	.word	.LC1
	.fnend
	.size	FlashEraseSLc2KBlocks, .-FlashEraseSLc2KBlocks
	.align	2
	.global	SandiskSetRRPara
	.type	SandiskSetRRPara, %function
SandiskSetRRPara:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r3, #239
	mov	r4, r0
	str	r3, [r0, #8]
	mov	r3, #17
	str	r3, [r0, #4]
	mov	r0, #200
	mov	r5, r1
	bl	NandcDelayns
	ldr	r1, .L204
	add	r3, r5, r5, asl #2
	mvn	r6, #4
	add	r0, r1, r3
	add	r1, r1, #45
	add	r1, r1, r3
	mov	r2, #0
	ldr	ip, .L204+4
	mul	r5, r6, r5
	b	.L199
.L202:
	ldrb	r6, [ip, #3761]	@ zero_extendqisi2
	add	r2, r2, #1
	cmp	r6, #67
	addeq	r6, r0, r5
	addne	r6, r1, r5
	add	r6, r6, r3
	add	r0, r0, #1
	add	r1, r1, #1
	ldrsb	r6, [r6, #5]
	str	r6, [r4, #0]
.L199:
	ldrb	r6, [ip, #3760]	@ zero_extendqisi2
	cmp	r2, r6
	bcc	.L202
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, lr}
	b	NandcWaitFlashReady
.L205:
	.align	2
.L204:
	.word	.LANCHOR1+2784
	.word	.LANCHOR0
	.fnend
	.size	SandiskSetRRPara, .-SandiskSetRRPara
	.align	2
	.global	SandiskProgTestBadBlock
	.type	SandiskProgTestBadBlock, %function
SandiskProgTestBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L207
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r3, r3, r0, asl #3
	ldrb	r6, [r3, #16]	@ zero_extendqisi2
	ldr	r5, [r3, #12]
	mov	r3, #162
	add	r6, r6, #8
	add	r4, r5, r6, asl #8
	str	r3, [r4, #8]
	mov	r3, #128
	str	r3, [r4, #8]
	mov	r3, #0
	str	r3, [r4, #4]
	str	r3, [r4, #4]
	uxtb	r3, r1
	str	r3, [r4, #4]
	mov	r3, r1, lsr #8
	mov	r1, r1, lsr #16
	str	r3, [r4, #4]
	str	r1, [r4, #4]
	mov	r3, #16
	str	r3, [r4, #8]
	bl	NandcWaitFlashReady
	mov	r3, #112
	mov	r0, #80
	str	r3, [r4, #8]
	bl	NandcDelayns
	ldr	r0, [r5, r6, asl #8]
	and	r0, r0, #1
	ldmfd	sp!, {r4, r5, r6, pc}
.L208:
	.align	2
.L207:
	.word	.LANCHOR0
	.fnend
	.size	SandiskProgTestBadBlock, .-SandiskProgTestBadBlock
	.align	2
	.global	FlashReadSpare
	.type	FlashReadSpare, %function
FlashReadSpare:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L210
	mov	ip, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r2
	ldrb	r2, [r3, #2697]	@ zero_extendqisi2
	ldr	r3, .L210+4
	add	r3, r3, r0, asl #3
	mov	r2, r2, asl #9
	ldrb	r5, [r3, #16]	@ zero_extendqisi2
	ldr	r4, [r3, #12]
	add	r5, r5, #8
	add	r3, r4, r5, asl #8
	str	ip, [r3, #8]
	str	r2, [r3, #4]
	mov	r2, r2, lsr #8
	str	r2, [r3, #4]
	uxtb	r2, r1
	str	r2, [r3, #4]
	mov	r2, r1, lsr #8
	mov	r1, r1, lsr #16
	str	r2, [r3, #4]
	str	r1, [r3, #4]
	mov	r2, #48
	str	r2, [r3, #8]
	bl	NandcWaitFlashReady
	ldr	r3, [r4, r5, asl #8]
	strb	r3, [r6, #0]
	ldmfd	sp!, {r4, r5, r6, pc}
.L211:
	.align	2
.L210:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashReadSpare, .-FlashReadSpare
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
	.global	FlashReset
	.type	FlashReset, %function
FlashReset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, .L214
	add	r3, r3, r0, asl #3
	ldrb	r5, [r3, #16]	@ zero_extendqisi2
	ldr	r2, [r3, #12]
	add	r5, r5, #8
	add	r5, r2, r5, asl #8
	bl	NandcFlashCs
	mov	r3, #255
	mov	r0, r4
	str	r3, [r5, #8]
	bl	NandcWaitFlashReady
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, lr}
	b	NandcFlashDeCs
.L215:
	.align	2
.L214:
	.word	.LANCHOR0
	.fnend
	.size	FlashReset, .-FlashReset
	.align	2
	.type	FlashReadIDRaw, %function
FlashReadIDRaw:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r2, r3, r4, r5, r6, r7, r8, lr}
	mov	r5, r1
	mov	r4, r0
	bl	FlashReset
	ldr	r3, .L218
	mov	r0, r4
	add	r3, r3, r4, asl #3
	ldrb	r7, [r3, #16]	@ zero_extendqisi2
	ldr	r6, [r3, #12]
	bl	NandcFlashCs
	add	r7, r7, #8
	mov	r3, #144
	mov	r0, #200
	add	r8, r6, r7, asl #8
	str	r3, [r8, #8]
	mov	r3, #0
	str	r3, [r8, #4]
	bl	NandcDelayns
	ldr	r3, [r6, r7, asl #8]
	mov	r0, r4
	strb	r3, [r5, #0]
	ldr	r3, [r6, r7, asl #8]
	strb	r3, [r5, #1]
	ldr	r3, [r6, r7, asl #8]
	strb	r3, [r5, #2]
	ldr	r3, [r6, r7, asl #8]
	strb	r3, [r5, #3]
	ldr	r3, [r6, r7, asl #8]
	strb	r3, [r5, #4]
	ldr	r3, [r6, r7, asl #8]
	strb	r3, [r5, #5]
	bl	NandcFlashDeCs
	ldrb	r2, [r5, #0]	@ zero_extendqisi2
	sub	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L216
	ldrb	r1, [r5, #2]	@ zero_extendqisi2
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	ldr	r0, .L218+4
	str	r1, [sp, #0]
	ldrb	r1, [r5, #3]	@ zero_extendqisi2
	str	r1, [sp, #4]
	ldrb	r1, [r5, #4]	@ zero_extendqisi2
	str	r1, [sp, #8]
	ldrb	r1, [r5, #5]	@ zero_extendqisi2
	str	r1, [sp, #12]
	add	r1, r4, #1
	bl	printk
.L216:
	ldmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, pc}
.L219:
	.align	2
.L218:
	.word	.LANCHOR0
	.word	.LC2
	.fnend
	.size	FlashReadIDRaw, .-FlashReadIDRaw
	.align	2
	.global	FlashSetInterfaceMode
	.type	FlashSetInterfaceMode, %function
FlashSetInterfaceMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, #0
	ldr	sl, .L234
	mov	r7, #239
	ldr	r9, .L234+4
	mov	r1, r4
	sub	r8, sl, #2128
	mov	r6, #128
	mov	r5, #1
.L230:
	mov	r2, r9
	ldrb	lr, [r4, sl]	@ zero_extendqisi2
	ldr	r3, [r2, r4]!
	cmp	lr, #152
	cmpne	lr, #69
	ldrb	r2, [r2, #4]	@ zero_extendqisi2
	add	r2, r2, #8
	add	ip, r3, r2, asl #8
	beq	.L221
	cmp	lr, #173
	cmpne	lr, #44
	bne	.L222
.L221:
	cmp	r0, #1
	ldrb	fp, [r8, #3772]	@ zero_extendqisi2
	bne	.L223
	tst	fp, #1
	beq	.L222
	cmp	lr, #173
	str	r7, [ip, #8]
	streq	r0, [ip, #4]
	beq	.L233
	cmp	lr, #44
	streq	r0, [ip, #4]
	strne	r6, [ip, #4]
	moveq	ip, #5
	strne	r0, [r3, r2, asl #8]
	bne	.L228
	b	.L232
.L223:
	tst	fp, #4
	beq	.L222
	cmp	lr, #173
	str	r7, [ip, #8]
	streq	r5, [ip, #4]
	moveq	ip, #32
	beq	.L232
	cmp	lr, #44
	strne	r6, [ip, #4]
	bne	.L233
	str	r5, [ip, #4]
	mov	ip, #35
.L232:
	str	ip, [r3, r2, asl #8]
	b	.L228
.L233:
	str	r1, [r3, r2, asl #8]
.L228:
	str	r1, [r3, r2, asl #8]
	str	r1, [r3, r2, asl #8]
	str	r1, [r3, r2, asl #8]
.L222:
	add	r4, r4, #8
	cmp	r4, #32
	bne	.L230
	mov	r0, #0
	bl	NandcWaitFlashReady
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L235:
	.align	2
.L234:
	.word	.LANCHOR0+2128
	.word	.LANCHOR0+12
	.fnend
	.size	FlashSetInterfaceMode, .-FlashSetInterfaceMode
	.align	2
	.global	FlashDeInit
	.type	FlashDeInit, %function
FlashDeInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r0, #0
	ldr	r4, .L238
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	ldrb	r3, [r4, #3773]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L237
	ldrb	r3, [r4, #3772]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L237
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r3, #0
	strb	r3, [r4, #3773]
.L237:
	ldr	r3, [r4, #12]
	mov	r0, #0
	str	r0, [r3, #336]
	ldmfd	sp!, {r4, pc}
.L239:
	.align	2
.L238:
	.word	.LANCHOR0
	.fnend
	.size	FlashDeInit, .-FlashDeInit
	.align	2
	.global	FlashReadDpCmd
	.type	FlashReadDpCmd, %function
FlashReadDpCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L244
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r2
	add	r2, r3, r0, asl #3
	mov	r5, r1
	mov	r4, r0
	ldrb	r6, [r2, #16]	@ zero_extendqisi2
	ldr	r1, [r2, #12]
	ldrb	r2, [r3, #60]	@ zero_extendqisi2
	add	r6, r6, #8
	cmp	r2, #1
	ldrb	r2, [r3, #52]	@ zero_extendqisi2
	add	r6, r1, r6, asl #8
	str	r2, [r6, #8]
	bne	.L241
	mov	r8, #0
	uxtb	r2, r5
	str	r8, [r6, #4]
	str	r8, [r6, #4]
	str	r2, [r6, #4]
	mov	r2, r5, lsr #8
	str	r2, [r6, #4]
	mov	r2, r5, lsr #16
	str	r2, [r6, #4]
	ldrb	r3, [r3, #53]	@ zero_extendqisi2
	str	r3, [r6, #8]
	bl	NandcWaitFlashReady
	str	r8, [r6, #8]
	str	r8, [r6, #4]
	str	r8, [r6, #4]
	b	.L243
.L241:
	uxtb	r2, r5
	str	r2, [r6, #4]
	mov	r2, r5, lsr #8
	str	r2, [r6, #4]
	mov	r2, r5, lsr #16
	str	r2, [r6, #4]
	ldrb	r3, [r3, #53]	@ zero_extendqisi2
	str	r3, [r6, #8]
.L243:
	uxtb	r3, r7
	mov	r0, r4
	str	r3, [r6, #4]
	mov	r1, r5
	mov	r3, r7, lsr #8
	mov	r7, r7, lsr #16
	str	r3, [r6, #4]
	mov	r3, #48
	str	r7, [r6, #4]
	str	r3, [r6, #8]
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	b	FlashSetRandomizer
.L245:
	.align	2
.L244:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadDpCmd, .-FlashReadDpCmd
	.align	2
	.global	HynixGetReadRetryDefault
	.type	HynixGetReadRetryDefault, %function
HynixGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L305
	mvn	r2, #83
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	cmp	r4, #2
	strb	r0, [r3, #2904]
	strb	r2, [r3, #2908]
	mvn	r0, #82
	mvn	r1, #81
	mvn	r2, #80
	.pad #68
	sub	sp, sp, #68
	strb	r0, [r3, #2909]
	strb	r1, [r3, #2910]
	strb	r2, [r3, #2911]
	bne	.L247
	mvn	r2, #88
	strb	r2, [r3, #2908]
	ldr	r3, .L305+4
	mvn	r2, #8
	strb	r2, [r3, #2949]
	b	.L286
.L247:
	cmp	r4, #3
	bne	.L249
	mvn	r2, #79
	strb	r2, [r3, #2908]
	mvn	r2, #78
	strb	r2, [r3, #2909]
	mvn	r2, #77
	strb	r2, [r3, #2910]
	mvn	r2, #76
	strb	r2, [r3, #2911]
	mvn	r2, #75
	strb	r2, [r3, #2912]
	mvn	r2, #74
	strb	r2, [r3, #2913]
	mvn	r2, #73
	strb	r2, [r3, #2914]
	mvn	r2, #72
	b	.L301
.L249:
	cmp	r4, #4
	bne	.L250
	mvn	ip, #51
	strb	r0, [r3, #2913]
	strb	ip, [r3, #2908]
	mvn	ip, #64
	strb	r1, [r3, #2914]
	strb	ip, [r3, #2909]
	mvn	ip, #85
	strb	ip, [r3, #2910]
	mvn	ip, #84
	strb	ip, [r3, #2911]
	mvn	ip, #50
	strb	ip, [r3, #2912]
.L301:
	mov	r6, #8
	strb	r2, [r3, #2915]
	mov	r5, r6
	b	.L248
.L250:
	cmp	r4, #5
	bne	.L251
	mov	r2, #56
	mov	r6, #8
	strb	r2, [r3, #2908]
	mov	r2, #57
	strb	r2, [r3, #2909]
	mov	r2, #58
	strb	r2, [r3, #2910]
	mov	r2, #59
	strb	r2, [r3, #2911]
	b	.L300
.L251:
	cmp	r4, #6
	bne	.L286
	mov	r2, #14
	mov	r6, #12
	strb	r2, [r3, #2908]
	mov	r2, #15
	strb	r2, [r3, #2909]
	mov	r2, #16
	strb	r2, [r3, #2910]
	mov	r2, #17
	strb	r2, [r3, #2911]
	b	.L300
.L286:
	mov	r6, #7
.L300:
	mov	r5, #4
.L248:
	sub	r3, r4, #1
	cmp	r3, #1
	bhi	.L299
	b	.L304
.L258:
	add	r1, r2, r8
	ldr	r4, .L305+8
	ldr	fp, .L305+12
	mov	r7, #0
	ldrb	r1, [r1, #3764]	@ zero_extendqisi2
	mov	r5, #55
	add	r2, r2, r1, asl #3
	add	r4, r4, r1, asl #6
	ldrb	sl, [r2, #16]	@ zero_extendqisi2
	ldr	r1, [r2, #12]
	add	sl, sl, #8
	add	sl, r1, sl, asl #8
.L254:
	add	r2, fp, r7
	str	r5, [sl, #8]
	mov	r0, #80
	ldrb	r2, [r2, #4]	@ zero_extendqisi2
	str	r2, [sl, #4]
	str	r3, [sp, #12]
	bl	NandcDelayns
	ldr	r2, [sl, #0]
	strb	r2, [r4, r7]
	add	r7, r7, #1
	ldr	r3, [sp, #12]
	uxtb	r2, r7
	cmp	r2, r6
	bcc	.L254
	mov	r2, #0
	b	.L255
.L256:
	ldrb	r7, [r0, r1, asl #2]	@ zero_extendqisi2
	ldrb	r5, [r4, r2]	@ zero_extendqisi2
	add	r5, r7, r5
	strb	r5, [ip, r1, asl #3]
	add	r1, r1, #1
	cmp	r1, #7
	bne	.L256
	add	r2, r2, #1
	cmp	r2, #4
	beq	.L257
.L255:
	mov	r1, #1
	add	ip, r4, r2
	add	r0, r2, r9
	b	.L256
.L257:
	add	r8, r8, #1
	mov	r2, #0
	strb	r2, [r4, #16]
	strb	r2, [r4, #24]
	uxtb	r8, r8
	strb	r2, [r4, #32]
	strb	r2, [r4, #40]
	strb	r2, [r4, #48]
	strb	r2, [r4, #41]
	strb	r2, [r4, #49]
	b	.L252
.L304:
	ldr	r9, .L305+16
	mov	r3, r6
	mov	r8, #0
	mov	r6, r5
.L252:
	ldr	r2, .L305
	ldrb	r1, [r2, #3762]	@ zero_extendqisi2
	cmp	r1, r8
	bhi	.L258
	mov	r5, r6
	mov	r6, r3
	b	.L259
.L299:
	sub	r3, r4, #3
	cmp	r3, #3
	bhi	.L259
	mul	r3, r6, r5
	mov	r8, #0
	mov	r7, r8
	str	r6, [sp, #44]
	mov	r2, r3, asr #2
	mov	r3, r3, asl #4
	str	r2, [sp, #24]
	str	r3, [sp, #52]
	b	.L260
.L285:
	ldr	ip, .L305
	ldr	r1, .L305+8
	add	r3, ip, r8
	ldrb	r6, [r3, #3764]	@ zero_extendqisi2
	add	r3, ip, r6, asl #3
	mov	r0, r6
	add	r1, r1, r6, asl #6
	str	r1, [sp, #48]
	ldrb	fp, [r3, #16]	@ zero_extendqisi2
	ldr	r9, [r3, #12]
	mov	r3, #255
	add	fp, fp, #8
	add	sl, r9, fp, asl #8
	str	r3, [sl, #8]
	str	r3, [sp, #12]
	bl	NandcWaitFlashReady
	cmp	r4, #4
	mov	r2, #54
	str	r2, [sl, #8]
	ldr	r3, [sp, #12]
	streq	r3, [sl, #4]
	moveq	r3, #64
	streq	r3, [r9, fp, asl #8]
	moveq	r3, #204
	beq	.L302
.L261:
	sub	r3, r4, #5
	cmp	r3, #1
	ldrls	r2, .L305+12
	ldrlsb	r3, [r2, #4]	@ zero_extendqisi2
	strls	r3, [sl, #4]
	movls	r3, #82
	bls	.L303
.L263:
	mov	r3, #174
	str	r3, [sl, #4]
	mov	r3, #176
	str	r7, [r9, fp, asl #8]
.L302:
	str	r3, [sl, #4]
	mov	r3, #77
.L303:
	cmp	r4, #6
	str	r3, [r9, fp, asl #8]
	mov	r0, r6
	mov	r3, #22
	str	r3, [sl, #8]
	mov	r3, #23
	str	r3, [sl, #8]
	mov	r3, #4
	str	r3, [sl, #8]
	mov	r3, #25
	str	r3, [sl, #8]
	moveq	r3, #31
	str	r7, [sl, #8]
	str	r7, [sl, #4]
	str	r7, [sl, #4]
	streq	r3, [sl, #4]
	mov	r3, #2
	strne	r7, [sl, #4]
	str	r3, [sl, #4]
	mov	r3, #48
	str	r7, [sl, #4]
	str	r3, [sl, #8]
	bl	NandcWaitFlashReady
	sub	r3, r4, #5
	cmp	r3, #1
	str	r3, [sp, #28]
	mov	r3, #0
	movls	r2, #16
	movhi	r2, #2
.L267:
	add	r3, r3, #1
	ldr	r1, [sl, #0]
	uxtb	r3, r3
	cmp	r3, r2
	bcc	.L267
	ldr	ip, .L305
	mov	r3, #0
	ldr	ip, [ip, #3776]
	str	ip, [sp, #32]
.L268:
	ldr	r2, [sl, #0]
	ldr	lr, [sp, #32]
	strb	r2, [lr, r3]
	add	r3, r3, #1
	ldr	r2, [sp, #52]
	cmp	r3, r2
	blt	.L268
	ldr	r3, [sp, #24]
	mov	ip, r3, asl #2
	mov	lr, r3, asl #3
	ldr	r3, .L305
	str	ip, [sp, #40]
	ldr	r1, [r3, #3776]
	mov	r3, #8
	add	r1, r1, ip
.L270:
	mov	r2, #0
	mov	r0, r2
.L269:
	ldr	ip, [r1, r2]
	add	r0, r0, #1
	mvn	ip, ip
	str	ip, [r1, r2]
	ldr	ip, [sp, #24]
	add	r2, r2, #4
	cmp	r0, ip
	blt	.L269
	subs	r3, r3, #1
	add	r1, r1, lr
	bne	.L270
	str	r3, [sp, #20]
	str	r8, [sp, #60]
	b	.L271
.L275:
	mov	lr, #1
	mov	r0, #0
	mov	ip, lr, asl r2
	mov	lr, r0
	mov	r8, #16
	str	r3, [sp, #4]
	str	r8, [sp, #16]
.L273:
	ldr	r3, [sp, #56]
	ldr	r3, [r3, r0]
	and	r8, ip, r3
	ldr	r3, [sp, #16]
	cmp	r8, ip
	ldr	r8, [sp, #40]
	addeq	lr, lr, #1
	subs	r3, r3, #1
	add	r0, r0, r8
	str	r3, [sp, #16]
	bne	.L273
	cmp	lr, #8
	add	r2, r2, #1
	orrhi	r1, r1, ip
	cmp	r2, #32
	ldr	r3, [sp, #4]
	bne	.L275
	ldr	ip, [sp, #20]
	ldr	r2, [sp, #24]
	add	ip, ip, #1
	ldr	r8, [sp, #36]
	cmp	ip, r2
	str	ip, [sp, #20]
	str	r1, [r8, r3]
	add	r3, r3, #4
	bge	.L276
.L271:
	ldr	r8, .L305
	mov	r1, #0
	mov	r2, r1
	ldr	r8, [r8, #3776]
	add	ip, r8, r3
	str	ip, [sp, #56]
	str	r8, [sp, #36]
	b	.L275
.L276:
	cmp	r4, #6
	ldr	r8, [sp, #60]
	ldr	r1, [sp, #48]
	mov	r2, #0
	moveq	lr, #4
	movne	lr, #8
	cmp	r5, #0
	str	lr, [sp, #20]
	subne	r3, r5, #1
	moveq	r0, #1
	streq	r0, [sp, #16]
	uxtbne	r3, r3
	ldr	r0, [sp, #32]
	addne	r3, r3, #1
	strne	r3, [sp, #16]
	b	.L280
.L281:
	ldrb	lr, [ip], #1	@ zero_extendqisi2
	strb	lr, [r1, r3]
	add	r3, r3, #1
	uxtb	lr, r3
	cmp	lr, r5
	bcc	.L281
	ldr	r3, [sp, #16]
	add	r2, r2, #1
	ldr	ip, [sp, #20]
	add	r0, r0, r3
	ldr	r3, [sp, #44]
	add	r1, r1, ip
	cmp	r2, r3
	bge	.L282
.L280:
	mov	ip, r0
	mov	r3, #0
	b	.L281
.L282:
	mov	r3, #255
	mov	r0, r6
	str	r3, [sl, #8]
	bl	NandcWaitFlashReady
	ldr	ip, [sp, #28]
	cmp	ip, #1
	movhi	r3, #56
	strhi	r3, [sl, #8]
	bhi	.L284
	ldr	lr, .L305+12
	mov	r3, #54
	str	r3, [sl, #8]
	mov	r0, r8
	mvn	r1, #0
	ldrb	r3, [lr, #4]	@ zero_extendqisi2
	str	r3, [sl, #4]
	mov	r3, #22
	str	r7, [r9, fp, asl #8]
	str	r3, [sl, #8]
	bl	FlashReadCmd
.L284:
	mov	r0, r6
	add	r8, r8, #1
	bl	NandcWaitFlashReady
	uxtb	r8, r8
.L260:
	ldr	r0, .L305
	ldrb	r3, [r0, #3762]	@ zero_extendqisi2
	cmp	r3, r8
	bhi	.L285
	ldr	r6, [sp, #44]
.L259:
	ldr	r3, .L305
	strb	r5, [r3, #2905]
	strb	r6, [r3, #2906]
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L306:
	.align	2
.L305:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+2924
	.word	.LANCHOR0+2904
	.word	.LANCHOR1+2932
	.fnend
	.size	HynixGetReadRetryDefault, .-HynixGetReadRetryDefault
	.align	2
	.global	FlashGetReadRetryDefault
	.type	FlashGetReadRetryDefault, %function
FlashGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	subs	r3, r0, #0
	ldmeqfd	sp!, {r3, pc}
	sub	r2, r3, #1
	cmp	r2, #5
	bhi	.L309
	ldmfd	sp!, {r3, lr}
	b	HynixGetReadRetryDefault
.L309:
	cmp	r3, #49
	bne	.L310
	ldr	r2, .L316
	ldr	r0, .L316+4
	ldr	r1, .L316+8
	strb	r3, [r2, #2904]
	mov	r3, #4
	strb	r3, [r2, #2905]
	mov	r3, #15
	strb	r3, [r2, #2906]
	mov	r2, #64
	b	.L314
.L310:
	cmp	r3, #33
	cmpne	r3, #65
	beq	.L311
	cmp	r3, #66
	bne	.L312
.L311:
	ldr	r2, .L316
	strb	r3, [r2, #2904]
	mov	r3, #4
	b	.L315
.L312:
	cmp	r3, #34
	cmpne	r3, #67
	bne	.L313
	ldr	r2, .L316
	strb	r3, [r2, #2904]
	mov	r3, #5
.L315:
	strb	r3, [r2, #2905]
	mov	r3, #7
	ldr	r0, .L316+4
	strb	r3, [r2, #2906]
	mov	r2, #45
	ldr	r1, .L316+12
	b	.L314
.L313:
	cmp	r3, #35
	cmpne	r3, #68
	ldmnefd	sp!, {r3, pc}
	ldr	r2, .L316
	ldr	r0, .L316+4
	ldr	r1, .L316+16
	strb	r3, [r2, #2904]
	mov	r3, #5
	strb	r3, [r2, #2905]
	mov	r3, #17
	strb	r3, [r2, #2906]
	mov	r2, #95
.L314:
	bl	memcpy
	ldmfd	sp!, {r3, pc}
.L317:
	.align	2
.L316:
	.word	.LANCHOR0
	.word	.LANCHOR0+2908
	.word	.LANCHOR1+2720
	.word	.LANCHOR1+2784
	.word	.LANCHOR1+2829
	.fnend
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
	.align	2
	.global	NandcRandmzSel
	.type	NandcRandmzSel, %function
NandcRandmzSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L319
	add	r3, r3, r0, asl #3
	ldr	r3, [r3, #12]
	str	r1, [r3, #336]
	bx	lr
.L320:
	.align	2
.L319:
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
	ldr	r1, .L331
	bl	__aeabi_idiv
	ldr	r3, .L331+4
	ldr	r3, [r3, #2160]
	cmp	r0, #250
	movwgt	r2, #8354
	bgt	.L329
	cmp	r0, #220
	bgt	.L330
.L324:
	cmp	r0, #185
	movwgt	r2, #4226
	bgt	.L329
	cmp	r0, #160
	movwgt	r2, #4194
	bgt	.L329
	cmp	r4, #35
	movwls	r2, #4193
	bls	.L329
	cmp	r4, #99
	movwls	r2, #4225
	bls	.L329
.L330:
	movw	r2, #8322
.L329:
	str	r2, [r3, #4]
	ldmfd	sp!, {r4, pc}
.L332:
	.align	2
.L331:
	.word	1000000
	.word	.LANCHOR0
	.fnend
	.size	NandcTimeCfg, .-NandcTimeCfg
	.align	2
	.global	NandcInit
	.type	NandcInit, %function
NandcInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, #0
	ldr	r4, .L334
	mov	r3, #1
	str	r0, [r4, #2160]
	str	r0, [r4, #12]
	str	r3, [r4, #24]
	mov	r3, #2
	str	r0, [r4, #20]
	str	r3, [r4, #32]
	mov	r3, #3
	str	r0, [r4, #28]
	str	r0, [r4, #36]
	str	r5, [r4, #16]
	str	r3, [r4, #40]
	ldr	r3, [r0, #0]
	and	r3, r3, #253952
	ubfx	r2, r3, #13, #1
	str	r2, [r4, #3780]
	ldr	r2, [r0, #352]
	and	r3, r3, #245760
	orr	r3, r3, #256
	ubfx	r2, r2, #16, #4
	str	r2, [r4, #3784]
	ldr	r2, [r0, #352]
	str	r2, [r4, #3788]
	str	r3, [r0, #0]
	mov	r0, #40
	ldr	r3, [r4, #2160]
	str	r5, [r3, #336]
	bl	NandcTimeCfg
	ldr	r3, [r4, #2160]
	movw	r2, #8322
	mov	r0, #36864
	str	r2, [r3, #344]
	ldr	r2, .L334+4
	str	r2, [r3, #304]
	bl	ftl_malloc
	str	r5, [r4, #3820]
	str	r5, [r4, #3828]
	str	r0, [r4, #3792]
	str	r0, [r4, #3796]
	add	r0, r0, #32768
	str	r0, [r4, #3800]
	ldmfd	sp!, {r3, r4, r5, pc}
.L335:
	.align	2
.L334:
	.word	.LANCHOR0
	.word	1710593
	.fnend
	.size	NandcInit, .-NandcInit
	.align	2
	.global	FlashTimingCfg
	.type	FlashTimingCfg, %function
FlashTimingCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	sub	r3, r0, #4160
	movw	r2, #4225
	sub	r3, r3, #33
	cmp	r0, r2
	cmpne	r3, #1
	bls	.L337
	movw	r3, #4226
	movw	r2, #8322
	cmp	r0, r3
	cmpne	r0, r2
	bne	.L338
.L337:
	ldr	r3, .L339
	ldr	r3, [r3, #2160]
	str	r0, [r3, #4]
.L338:
	ldr	r3, .L339+4
	ldrb	r0, [r3, #2709]	@ zero_extendqisi2
	b	NandcTimeCfg
.L340:
	.align	2
.L339:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.fnend
	.size	FlashTimingCfg, .-FlashTimingCfg
	.align	2
	.global	NandcGetTimeCfg
	.type	NandcGetTimeCfg, %function
NandcGetTimeCfg:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L342
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, [ip, #2160]
	ldr	r4, [r4, #4]
	str	r4, [r0, #0]
	ldr	r0, [ip, #2160]
	ldr	r0, [r0, #0]
	str	r0, [r1, #0]
	ldr	r1, [ip, #2160]
	ldr	r1, [r1, #304]
	str	r1, [r2, #0]
	ldr	r2, [ip, #2160]
	ldr	r0, [r2, #308]
	ldr	r1, [r2, #344]
	uxtb	r2, r0
	orr	r2, r2, r1, asl #16
	str	r2, [r3, #0]
	ldmfd	sp!, {r4, pc}
.L343:
	.align	2
.L342:
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
	ldr	r3, .L349
	mov	r1, #1
	ldr	r2, [r3, #2160]
	str	r0, [r3, #3832]
	mov	r3, #0
	str	r1, [r2, #8]
	mov	r1, #16
	cmp	r0, r1
	bfi	r3, r1, #8, #8
	bfc	r3, #18, #1
	beq	.L348
.L345:
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L346
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	bne	.L346
.L348:
	bfc	r3, #4, #1
.L346:
	orr	r3, r3, #1
	str	r3, [r2, #12]
	bx	lr
.L350:
	.align	2
.L349:
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
	ldr	r3, .L352
	strb	r0, [r3, #3836]
	b	NandcBchSel
.L353:
	.align	2
.L352:
	.word	.LANCHOR0
	.fnend
	.size	FlashBchSel, .-FlashBchSel
	.align	2
	.global	FlashResume
	.type	FlashResume, %function
FlashResume:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, #0
	ldr	r5, .L359
	ldr	r3, [r5, #2160]
	ldr	r2, [r5, #2164]
	str	r2, [r3, #0]
	ldr	r2, [r5, #2168]
	ldr	r3, [r5, #2160]
	str	r2, [r3, #4]
	ldr	r2, [r5, #2172]
	str	r2, [r3, #8]
	ldr	r2, [r5, #2176]
	str	r2, [r3, #12]
	ldr	r2, [r5, #2180]
	str	r2, [r3, #304]
	ldr	r2, [r5, #2184]
	str	r2, [r3, #308]
	ldr	r2, [r5, #2188]
	str	r2, [r3, #336]
	ldr	r2, [r5, #2192]
	add	r5, r5, #2128
	str	r2, [r3, #344]
.L356:
	ldrb	r3, [r5, r4, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L355
	uxtb	r0, r4
	bl	FlashReset
.L355:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L356
	ldr	r4, .L359
	ldrb	r3, [r4, #3773]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L357
	mov	r0, #1
	bl	NandcSetMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
	ldrb	r0, [r4, #2181]	@ zero_extendqisi2
	bl	NandcSetDdrPara
.L357:
	ldr	r3, .L359
	ldr	r3, [r3, #2772]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L360:
	.align	2
.L359:
	.word	.LANCHOR0
	.fnend
	.size	FlashResume, .-FlashResume
	.align	2
	.global	NandCIrqEnable
	.type	NandCIrqEnable, %function
NandCIrqEnable:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #368]
	mov	r2, #1
	mov	r2, r2, asl r1
	orr	r3, r2, r3
	str	r3, [r0, #368]
	ldr	r3, [r0, #364]
	orr	r2, r2, r3
	str	r2, [r0, #364]
	bx	lr
	.fnend
	.size	NandCIrqEnable, .-NandCIrqEnable
	.align	2
	.global	NandCIrqDisable
	.type	NandCIrqDisable, %function
NandCIrqDisable:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, [r0, #368]
	mov	r2, #1
	mov	r2, r2, asl r1
	orr	r3, r2, r3
	str	r3, [r0, #368]
	ldr	r3, [r0, #364]
	bic	r2, r3, r2
	str	r2, [r0, #364]
	bx	lr
	.fnend
	.size	NandCIrqDisable, .-NandCIrqDisable
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
	.global	rk_nandc_flash_ready
	.type	rk_nandc_flash_ready, %function
rk_nandc_flash_ready:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r1, #1
	b	NandCIrqDisable
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
	mov	r0, r4
	mov	r1, #1
	bl	NandCIrqEnable
	ldr	r3, [r4, #0]
	mov	r0, r4
	ubfx	r3, r3, #8, #8
	and	r3, r3, #2
	uxtb	r3, r3
	cmp	r3, #0
	bne	.L366
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L366:
	mov	r1, #1
	ldmfd	sp!, {r4, lr}
	b	NandCIrqDisable
	.fnend
	.size	NandcIqrWaitFlashReady, .-NandcIqrWaitFlashReady
	.align	2
	.global	FlashEraseBlocks
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r5, .L383
	mov	r6, r0
	str	r1, [sp, #12]
	mov	r7, r2
	ldrb	r4, [r5, #0]	@ zero_extendqisi2
	cmp	r4, #0
	moveq	r9, r5
	beq	.L368
	mov	r1, r2
	bl	FlashEraseSLc2KBlocks
	b	.L369
.L377:
	mov	r8, #36
	rsb	r3, r4, r7
	mul	r8, r8, r4
	uxtb	r3, r3
	mov	r1, #0
	str	r3, [sp, #0]
	add	r2, sp, #16
	add	r3, sp, #20
	add	fp, r6, r8
	mov	r0, fp
	bl	LogAddr2PhyAddr
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	mov	sl, r0
	ldr	r0, [sp, #20]
	cmp	r0, r3
	mvncs	r3, #0
	strcs	r3, [r6, r8]
	bcs	.L371
	ldrb	r3, [r5, #3837]	@ zero_extendqisi2
	cmp	r3, #0
	add	r3, r9, r0, asl #4
	ldr	r3, [r3, #2784]
	moveq	sl, #0
	cmp	r3, #0
	beq	.L373
	uxtb	r0, r0
	bl	FlashWaitCmdDone
.L373:
	ldr	r2, [sp, #20]
	mov	r1, #0
	cmp	sl, #0
	add	r3, r5, r2, asl #4
	movne	r0, #36
	str	r1, [r3, #2788]
	ldr	r1, [sp, #16]
	str	fp, [r3, #2784]
	str	r1, [r3, #2780]
	addne	r1, r4, #1
	mlane	r1, r0, r1, r6
	strne	r1, [r3, #2788]
	add	r3, r9, r2
	add	r2, r9, r2, asl #4
	ldrb	r8, [r3, #3764]	@ zero_extendqisi2
	mov	r0, r8
	strb	r8, [r2, #2776]
	bl	NandcFlashCs
	ldr	r3, [sp, #12]
	mov	r0, r8
	cmp	r3, #1
	bne	.L375
	ldrb	r3, [r5, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L375
	bl	flash_enter_slc_mode
	b	.L376
.L375:
	bl	flash_exit_slc_mode
.L376:
	ldr	r3, [sp, #20]
	mov	r0, r8
	ldr	r1, [sp, #16]
	add	r4, r4, sl
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #2736]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r8
	ldr	r1, [sp, #16]
	mov	r2, sl
	bl	FlashEraseCmd
	mov	r0, r8
	bl	NandcFlashDeCs
.L371:
	add	r4, r4, #1
.L368:
	cmp	r4, r7
	bcc	.L377
	ldr	r5, .L383
	mov	r4, #0
	ldr	r6, .L383+4
	ldr	r0, [r5, #2160]
	bl	NandcIqrWaitFlashReady
	b	.L378
.L380:
	uxtb	r0, r4
	bl	FlashWaitCmdDone
	ldr	r3, [sp, #12]
	cmp	r3, #1
	bne	.L379
	ldrb	r3, [r5, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L379
	ldrb	r0, [r6, r4, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L379:
	add	r4, r4, #1
.L378:
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L380
	mov	r0, #0
.L369:
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L384:
	.align	2
.L383:
	.word	.LANCHOR0
	.word	.LANCHOR0+2776
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
	mov	r1, #0
	b	NandCIrqDisable
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
	ldr	r3, .L387
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	bfc	r2, #2, #1
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	str	r2, [r0, #16]
	str	r3, [r0, #8]
	orr	r3, r3, #4
	str	r3, [r0, #8]
	add	sp, sp, #8
	bx	lr
.L388:
	.align	2
.L387:
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
.L390:
	ldr	r3, [r0, #8]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #1048576
	beq	.L390
	add	sp, sp, #8
	bx	lr
	.fnend
	.size	NandcSendDumpDataDone, .-NandcSendDumpDataDone
	.align	2
	.global	NandcXferStart
	.type	NandcXferStart, %function
NandcXferStart:
	.fnstart
	@ args = 8, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r2
	ldr	r2, .L405
	mov	ip, #16
	mov	r6, r1
	mov	r4, #0
	add	r0, r2, r0, asl #3
	.pad #20
	sub	sp, sp, #20
	ldr	r5, [r0, #12]
	ldrb	r0, [r0, #16]	@ zero_extendqisi2
	ldr	r9, [sp, #56]
	ldr	r7, [r5, #12]
	ldr	r1, [sp, #60]
	bfi	r7, ip, #8, #8
	bfi	r7, r4, #3, #1
	bfi	r4, r6, #1, #1
	orr	r4, r4, #8
	bfi	r7, r0, #5, #3
	mov	r0, #1
	bfi	r4, r0, #5, #2
	orr	r4, r4, #536870912
	mov	r3, r3, lsr r0
	orr	r4, r4, #1024
	bfi	r4, r3, #4, #1
	ldr	r3, [r2, #3784]
	cmp	r3, #3
	bls	.L393
	ldr	r3, [r5, #16]
	cmp	r9, #0
	cmpeq	r1, #0
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bfc	r3, #2, #1
	str	r3, [sp, #12]
	beq	.L394
	cmp	r6, #0
	beq	.L395
	ldr	r3, [r2, #3832]
	mov	ip, r1
	cmp	r3, #24
	movhi	r3, #128
	movls	r3, #64
	str	r3, [sp, #0]
	mov	r3, r8, lsr r0
	str	r3, [sp, #4]
	mov	r3, #0
	mov	r0, r3
	b	.L397
.L400:
	cmp	r1, #0
	mov	lr, r3, lsr #2
	add	r0, r0, #1
	ldrneh	fp, [ip, #2]
	mvneq	fp, #0
	ldrneh	sl, [ip], #4
	ldreq	sl, [r2, #3800]
	orrne	sl, sl, fp, asl #16
	ldrne	fp, [r2, #3800]
	streq	fp, [sl, lr, asl #2]
	strne	sl, [fp, lr, asl #2]
	ldr	lr, [sp, #0]
	add	r3, r3, lr
.L397:
	ldr	sl, [sp, #4]
	cmp	r0, sl
	bcc	.L400
.L395:
	mov	r0, r5
	add	r8, r8, #1
	bl	rk_nandc_xfer_irq_flag_init
	mov	r0, r5
	mov	r1, #0
	mov	r8, r8, asr #1
	bl	NandCIrqEnable
	bfi	r4, r8, #22, #6
	ldr	r8, .L405
	cmp	r9, #0
	ubfx	sl, r4, #22, #5
	mov	r2, r6
	movne	r0, r9
	ldreq	r0, [r8, #3796]
	mov	r1, sl, asl #10
	ldr	r3, [r8, #3800]
	str	r3, [r8, #3808]
	str	r0, [r8, #3804]
	bl	rknand_dma_map_single
	mov	r2, r6
	mov	r1, sl, asl #7
	str	r0, [r8, #3812]
	ldr	r0, [r8, #3808]
	bl	rknand_dma_map_single
	mov	r3, #1
	str	r3, [r8, #3820]
	mov	r2, #16
	ldr	r3, [r8, #3812]
	tst	r9, #3
	str	r0, [r8, #3816]
	str	r3, [r5, #20]
	ldr	r3, [r8, #3816]
	str	r3, [r5, #24]
	mov	r3, #0
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bfi	r3, r2, #9, #5
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	moveq	r2, #2
	orr	r3, r3, #448
	str	r3, [sp, #12]
	ldreq	r3, [sp, #12]
	bfieq	r3, r2, #3, #3
	streq	r3, [sp, #12]
	rsbs	r6, r6, #1
	ldr	r3, [sp, #12]
	movcc	r6, #0
	orr	r3, r3, #4
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bfi	r3, r6, #1, #1
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	orr	r3, r3, #1
	str	r3, [sp, #12]
.L394:
	ldr	r3, [sp, #12]
	str	r3, [r5, #16]
.L393:
	str	r7, [r5, #12]
	str	r4, [r5, #8]
	orr	r4, r4, #4
	str	r4, [r5, #8]
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L406:
	.align	2
.L405:
	.word	.LANCHOR0
	.fnend
	.size	NandcXferStart, .-NandcXferStart
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
	mov	r5, r2
	add	r2, r0, #4096
	mov	r4, r0
	ldr	r6, [sp, #16]
	add	r2, r2, r5, asl #9
	bne	.L408
	cmp	r3, #0
	beq	.L409
	mov	r0, r2
	mov	r1, r3
	mov	r2, #1024
	bl	memcpy
.L409:
	cmp	r6, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	mov	r5, r5, lsr #1
	ldrb	r2, [r6, #1]	@ zero_extendqisi2
	mov	r3, r3, asl #16
	orr	r3, r3, r2, asl #8
	ldrb	r2, [r6, #0]	@ zero_extendqisi2
	orr	r3, r3, r2
	ldrb	r2, [r6, #3]	@ zero_extendqisi2
	orr	r3, r3, r2, asl #24
	mov	r2, #48
	mul	r5, r2, r5
	mov	r5, r5, asr #2
	add	r5, r5, #128
	str	r3, [r4, r5, asl #2]
	ldmfd	sp!, {r4, r5, r6, pc}
.L408:
	cmp	r3, #0
	beq	.L411
	mov	r1, r2
	mov	r0, r3
	mov	r2, #1024
	bl	memcpy
.L411:
	cmp	r6, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	mov	r5, r5, lsr #1
	mov	r3, #48
	mul	r5, r3, r5
	mov	r5, r5, asr #2
	add	r5, r5, #128
	ldr	r3, [r4, r5, asl #2]
	mov	r2, r3, lsr #8
	strb	r3, [r6, #0]
	strb	r2, [r6, #1]
	mov	r2, r3, lsr #16
	mov	r3, r3, lsr #24
	strb	r2, [r6, #2]
	strb	r3, [r6, #3]
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	NandcCopy1KB, .-NandcCopy1KB
	.align	2
	.global	Ftl_log2
	.type	Ftl_log2, %function
Ftl_log2:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r2, #1
	mov	r3, #0
	b	.L413
.L414:
	add	r3, r3, #1
	mov	r2, r2, asl #1
	uxth	r3, r3
.L413:
	cmp	r2, r0
	bls	.L414
	sub	r3, r3, #1
	uxth	r0, r3
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
	ldr	r3, .L419
	movw	r2, #3844
	cmp	r0, #23
	movw	r1, #3854
	ldrh	r2, [r3, r2]
	movls	r0, #24
	ldrh	r1, [r3, r1]
	str	r0, [r3, #3840]
	mul	r2, r2, r0
	rsb	r0, r0, r1
	movw	r1, #3852
	strh	r0, [r3, r1]	@ movhi
	mov	r0, #0
	ldr	r1, [r3, #3860]
	str	r2, [r3, #3848]
	rsb	r2, r2, r1
	str	r2, [r3, #3856]
	bx	lr
.L420:
	.align	2
.L419:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r2, #3864
	ldr	r3, .L435
	mov	r4, r0
	ldrh	lr, [r0, #8]
	.pad #20
	sub	sp, sp, #20
	ldrh	r1, [r0, #10]
	ldrh	r0, [r0, #12]
	strh	lr, [r3, r2]	@ movhi
	add	r2, r2, #2
	ldrh	r6, [r4, #14]
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	strh	r0, [r3, r2]	@ movhi
	movw	r2, #3854
	strh	r6, [r3, r2]	@ movhi
	mov	r3, #0
	ldr	r2, .L435+4
.L422:
	strb	r3, [r3, r2]
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L422
	ldrh	r2, [r4, #14]
	ldrh	r3, [r4, #20]
	cmp	r3, r2, lsr #8
	bcs	.L423
	ldr	r2, .L435+4
	sub	ip, r1, #1
	uxtb	r7, r0
	mov	r3, #0
	rsb	sl, r0, r2
	str	sl, [sp, #8]
	mla	ip, r0, ip, r2
	mov	r8, r7, asl #1
	uxtb	r8, r8
	str	ip, [sp, #12]
	b	.L424
.L425:
	strb	ip, [r9, r2]
	add	fp, ip, r7
	add	ip, ip, r8
	add	r5, r5, #1
	strb	fp, [sl, r2]
	uxtb	ip, ip
.L427:
	cmp	r5, r1
	add	r2, r2, r0
	bcc	.L425
	add	r3, r3, #1
.L424:
	cmp	r3, r0
	bcs	.L426
	ldr	fp, [sp, #8]
	mov	r2, #0
	uxtb	ip, r3
	mov	r5, r2
	add	r9, fp, r3
	ldr	fp, [sp, #12]
	add	sl, fp, r3
	b	.L427
.L426:
	ldr	r3, .L435
	movw	r2, #3866
	mov	r1, r1, asl #1
	mov	r6, r6, lsr #1
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #3854
	strh	r6, [r3, r2]	@ movhi
.L423:
	ldr	r3, .L435
	movw	r2, #3902
	cmp	lr, #1
	mov	r1, #5
	mov	ip, #0
	ldr	sl, .L435
	ldrb	r8, [r3, #0]	@ zero_extendqisi2
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #3904
	streqh	lr, [r3, r2]	@ movhi
	cmp	r8, #0
	movw	r2, #3906
	strh	ip, [r3, r1]	@ movhi
	mov	r1, #4352
	strh	r1, [r3, r2]	@ movhi
	ldrne	r3, .L435
	movne	r1, #384
	strneh	r1, [r3, r2]	@ movhi
	movw	r3, #3866
	ldrh	r5, [sl, r3]
	movw	r3, #3844
	mul	r5, r5, r0
	uxth	r5, r5
	strh	r5, [sl, r3]	@ movhi
	add	r3, r3, #10
	ldrh	r6, [sl, r3]
	add	r3, r3, #54
	mul	r0, r0, r6
	uxth	r0, r0
	strh	r0, [sl, r3]	@ movhi
	bl	Ftl_log2
	movw	r3, #3910
	ldrh	fp, [r4, #18]
	movw	r2, #3912
	ldrh	r9, [r4, #20]
	strh	r0, [sl, r3]	@ movhi
	mov	r0, r9
	ldrh	r3, [r4, #16]
	mul	r1, r3, r5
	strh	r3, [sl, r2]	@ movhi
	add	r2, r2, #2
	strh	fp, [sl, r2]	@ movhi
	add	r2, r2, #2
	strh	r1, [sl, r2]	@ movhi
	add	r2, r2, #2
	strh	r9, [sl, r2]	@ movhi
	str	r3, [sp, #4]
	bl	Ftl_log2
	mov	r2, #3920
	movw	r1, #3922
	cmp	r6, #1024
	strh	r0, [sl, r2]	@ movhi
	mov	r2, r9, asl #9
	mov	r7, r0
	uxth	r2, r2
	strh	r2, [sl, r1]	@ movhi
	add	r1, r1, #2
	mov	r2, r2, lsr #8
	strh	r2, [sl, r1]	@ movhi
	movw	r2, #3926
	ldrh	r1, [r4, #26]
	mov	r4, sl
	strh	r1, [sl, r2]	@ movhi
	uxtbhi	r1, r6
	mul	r2, r6, r5
	str	r2, [sl, #3860]
	movhi	r2, #3904
	ldr	r3, [sp, #4]
	strhih	r1, [sl, r2]	@ movhi
	mov	r2, #3904
	ldrh	r2, [sl, r2]
	mul	r1, fp, r9
	rsb	r2, r2, r6
	mov	r6, r6, asl #6
	mul	r2, r2, r5
	mul	r2, r9, r2
	mul	r3, r3, r2
	mov	r3, r3, asr #11
	str	r3, [sl, #3928]
	movw	sl, #3906
	ldrh	r0, [r4, sl]
	mov	r0, r0, asl #3
	bl	__aeabi_idiv
	movw	r3, #3932
	uxth	r0, r0
	strh	r0, [r4, r3]	@ movhi
	cmp	r0, #4
	ldr	r4, .L435
	ldrls	r2, .L435
	movls	r1, #4
	strlsh	r1, [r2, r3]	@ movhi
	cmp	r8, #0
	mov	r1, r5
	movne	r3, #640
	strneh	r3, [r4, sl]	@ movhi
	ldrh	r2, [r4, sl]
	movw	r3, #3934
	mov	r2, r2, asr r7
	add	r7, r7, #9
	mov	r6, r6, asr r7
	add	r2, r2, #2
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #3936
	uxth	r6, r6
	strh	r6, [r4, r3]	@ movhi
	mul	r3, r5, r6
	add	r6, r6, #8
	str	r3, [r4, #3940]
	movw	r3, #3932
	ldrh	r0, [r4, r3]
	bl	__aeabi_uidiv
	cmp	r5, #1
	ldreq	r3, .L435
	uxtah	r6, r6, r0
	str	r6, [r4, #3840]
	ldr	r4, .L435
	addeq	r6, r6, #4
	streq	r6, [r3, #3840]
	ldr	r0, [r4, #3840]
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #3840]
	ldr	r2, [r4, #3856]
	mov	r0, #2048
	str	r3, [r4, #3944]
	movw	r3, #3912
	ldrh	r3, [r4, r3]
	mov	r2, r2, asl #2
	mul	r2, r3, r2
	mov	r3, #3920
	ldrh	r3, [r4, r3]
	add	r3, r3, #9
	mov	r2, r2, lsr r3
	movw	r3, #3948
	add	r2, r2, #2
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #3918
	ldrh	r1, [r4, r3]
	bl	__aeabi_idiv
	movw	r3, #3950
	strh	r0, [r4, r3]	@ movhi
	movw	r3, #3932
	ldrh	r2, [r4, r3]
	mov	r0, #0
	str	r0, [r4, #3952]
	add	r2, r2, #3
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #3940]
	add	r3, r3, #3
	str	r3, [r4, #3940]
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L436:
	.align	2
.L435:
	.word	.LANCHOR0
	.word	.LANCHOR0+3870
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
	ldr	r3, .L444
	movw	r2, #3956
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L442
	movw	r2, #3932
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #3960]
	mov	r3, #0
	b	.L439
.L441:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L443
	add	r3, r3, #1
	uxth	r3, r3
.L439:
	cmp	r3, r1
	bne	.L441
	mov	r0, #0
	bx	lr
.L443:
	mov	r0, #1
	bx	lr
.L442:
	mov	r0, r2
	bx	lr
.L445:
	.align	2
.L444:
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
	ldr	r2, .L454
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #3964]
	cmp	r3, #0
	beq	.L451
	ldr	ip, [r2, #3968]
	mov	r2, #0
.L450:
	ldr	r4, [ip, #16]
	cmp	r4, r0
	bcc	.L448
	cmp	r4, r1
	bls	.L452
.L448:
	add	r2, r2, #1
	add	ip, ip, #36
	cmp	r2, r3
	bne	.L450
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L452:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L451:
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L455:
	.align	2
.L454:
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
	ldr	r3, .L457
	ldr	r0, [r3, #3952]
	bx	lr
.L458:
	.align	2
.L457:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetCap, .-FtlGetCap
	.align	2
	.global	FtlGetCapacity
	.type	FtlGetCapacity, %function
FtlGetCapacity:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L460
	ldr	r0, [r3, #3952]
	bx	lr
.L461:
	.align	2
.L460:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetCapacity, .-FtlGetCapacity
	.align	2
	.global	FtlGetLpn
	.type	FtlGetLpn, %function
FtlGetLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L463
	ldr	r0, [r3, #3972]
	bx	lr
.L464:
	.align	2
.L463:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLpn, .-FtlGetLpn
	.align	2
	.global	FtlGetCurEraseBlock
	.type	FtlGetCurEraseBlock, %function
FtlGetCurEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L466
	movw	r2, #3844
	ldrh	r2, [r3, r2]
	ldr	r0, [r3, #3976]
	mul	r0, r0, r2
	bx	lr
.L467:
	.align	2
.L466:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetCurEraseBlock, .-FtlGetCurEraseBlock
	.align	2
	.global	FtlGetAllBlockNum
	.type	FtlGetAllBlockNum, %function
FtlGetAllBlockNum:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L469
	movw	r1, #3844
	movw	r2, #3854
	ldrh	r2, [r3, r2]
	ldrh	r0, [r3, r1]
	mul	r0, r0, r2
	bx	lr
.L470:
	.align	2
.L469:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetAllBlockNum, .-FtlGetAllBlockNum
	.align	2
	.global	FtlBbmMapBadBlock
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r0, r1, r4, r5, r6, lr}
	movw	r3, #3908
	ldr	r4, .L472
	mov	r5, r0
	ldrh	r6, [r4, r3]
	mov	r1, r6
	bl	__aeabi_uidiv
	uxth	r2, r0
	mls	r3, r6, r2, r5
	add	r1, r4, r2, asl #2
	mov	r6, #1
	ldr	r0, [r1, #4008]
	uxth	r3, r3
	and	r1, r3, #31
	mov	ip, r3, lsr #5
	ldr	lr, [r0, ip, asl #2]
	orr	r1, lr, r6, asl r1
	str	r1, [r0, ip, asl #2]
	str	r1, [sp, #0]
	mov	r1, r5
	ldr	r0, .L472+4
	bl	printk
	movw	r3, #3986
	ldrh	r2, [r4, r3]
	mov	r0, #0
	add	r2, r2, r6
	strh	r2, [r4, r3]	@ movhi
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L473:
	.align	2
.L472:
	.word	.LANCHOR0
	.word	.LC3
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
	movw	r3, #3908
	ldr	r5, .L475
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
	ldr	r3, [r5, #4008]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr r4
	and	r0, r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L476:
	.align	2
.L475:
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
	.global	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r4, #0
	mov	r5, r4
	ldr	r6, .L482
	movw	r8, #3908
	movw	r7, #3866
	b	.L479
.L481:
	mov	r0, r5
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	uxth	r5, r5
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
.L479:
	ldrh	r3, [r6, r8]
	ldrh	r2, [r6, r7]
	mul	r3, r2, r3
	cmp	r5, r3
	blt	.L481
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L483:
	.align	2
.L482:
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
	movw	r3, #3868
	ldr	r5, .L485
	mov	r7, r1
	mov	r6, r0
	ldrh	r4, [r5, r3]
	mov	r1, r4
	bl	__aeabi_uidiv
	movw	r3, #3908
	ldrh	r5, [r5, r3]
	mov	r1, r4
	mul	r7, r4, r7
	mla	r5, r5, r0, r7
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L486:
	.align	2
.L485:
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
	ldr	r3, .L488
	movw	r2, #3868
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r4, [r3, r2]
	add	r2, r2, #40
	ldrh	r1, [r3, r2]
	bl	__aeabi_uidiv
	mov	r1, r4
	mul	r5, r0, r4
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r4, r5, r6, pc}
.L489:
	.align	2
.L488:
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
	movw	r3, #3908
	ldr	r4, .L491
	ldrh	r1, [r4, r3]
	bl	__aeabi_uidivmod
	movw	r3, #3868
	uxth	r0, r1
	ldrh	r1, [r4, r3]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L492:
	.align	2
.L491:
	.word	.LANCHOR0
	.fnend
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	2
	.type	FtlFreeSysBlkQueueOut.part.5, %function
FtlFreeSysBlkQueueOut.part.5:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L495
	movw	r2, #4042
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movw	r0, #4040
	ldrh	r1, [r3, r2]
	add	ip, r1, #4
	add	r1, r1, #1
	bic	r1, r1, #64512
	add	ip, r3, ip, asl #1
	ldrh	r5, [ip, r0]
	add	r0, r0, #6
	strh	r1, [r3, r2]	@ movhi
	ldrh	ip, [r3, r0]
	sub	ip, ip, #1
	strh	ip, [r3, r0]	@ movhi
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L494
	mov	r0, r5
	ldr	r4, .L495+4
	bl	P2V_block_in_plane
	mov	r1, #1
	mov	r3, r5, asl #10
	mov	r2, r1
	mov	r6, r0
	ldr	r0, [r4, #-2088]
	mov	r6, r6, asl #1
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #-2084]
	ldrh	r2, [r3, r6]
	add	r2, r2, #1
	strh	r2, [r3, r6]	@ movhi
	ldr	r3, [r4, #-2080]
	add	r3, r3, #1
	str	r3, [r4, #-2080]
.L494:
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L496:
	.align	2
.L495:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlFreeSysBlkQueueOut.part.5, .-FtlFreeSysBlkQueueOut.part.5
	.align	2
	.global	ftl_cmp_data_ver
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, r1
	bls	.L498
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L498:
	rsb	r0, r0, r1
	cmp	r0, #-2147483648
	movls	r0, #0
	movhi	r0, #1
	bx	lr
	.fnend
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlFreeSysBlkQueueEmpty
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L501
	movw	r3, #4046
	ldrh	r0, [r2, r3]
	rsbs	r0, r0, #1
	movcc	r0, #0
	bx	lr
.L502:
	.align	2
.L501:
	.word	.LANCHOR0
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
	ldr	r2, .L504
	movw	r3, #4046
	ldrh	r0, [r2, r3]
	sub	r2, r0, #1024
	rsbs	r0, r2, #0
	adc	r0, r0, r2
	bx	lr
.L505:
	.align	2
.L504:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L509
	movw	r3, #4046
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldrh	r3, [r2, r3]
	cmp	r3, #1024
	ldmeqfd	sp!, {r4, r5, r6, pc}
	cmp	r1, #0
	beq	.L508
	bl	P2V_block_in_plane
	ldr	r4, .L509+4
	mov	r1, #1
	mov	r3, r5, asl #10
	mov	r2, r1
	mov	r6, r0
	ldr	r0, [r4, #-2088]
	mov	r6, r6, asl #1
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #-2084]
	ldrh	r2, [r3, r6]
	add	r2, r2, #1
	strh	r2, [r3, r6]	@ movhi
	ldr	r3, [r4, #-2080]
	add	r3, r3, #1
	str	r3, [r4, #-2080]
.L508:
	ldr	r3, .L509
	movw	r2, #4046
	movw	r0, #4040
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #4044
	ldrh	r1, [r3, r2]
	add	ip, r1, #4
	add	r1, r1, #1
	bic	r1, r1, #64512
	add	ip, r3, ip, asl #1
	strh	r5, [ip, r0]	@ movhi
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L510:
	.align	2
.L509:
	.word	.LANCHOR0
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r4, #0
	ldr	r5, .L521
	movw	r8, #4046
	movw	r7, #4042
	ldr	r6, .L521+4
	b	.L512
.L513:
	ldrh	r3, [r5, r7]
	add	r3, r4, r3
	add	r3, r5, r3, asl #1
	add	r3, r3, #4048
	ldrh	r0, [r3, #0]
	bl	P2V_block_in_plane
	ldr	r2, [r6, #-2084]
	ldr	r3, [r6, #-2076]
	mov	r0, r0, asl #1
	ldrh	r2, [r2, r0]
	str	r2, [r3, r4, asl #2]
	add	r4, r4, #1
	uxth	r4, r4
.L512:
	ldrh	r3, [r5, r8]
	cmp	r3, r4
	bhi	.L513
	mov	r3, #0
	ldr	ip, .L521
	movw	r8, #4046
	ldr	r4, .L521+4
	movw	r7, #4042
	b	.L514
.L519:
	add	r6, r3, #1
	ldr	r1, [r4, #-2076]
	mov	r2, r3
	uxth	r6, r6
	mov	r0, r6
	b	.L515
.L517:
	ldr	r9, [r1, r2, asl #2]
	ldr	sl, [r1, r0, asl #2]
	cmp	r9, sl
	movhi	r2, r0
	add	r0, r0, #1
	uxth	r0, r0
.L515:
	cmp	r0, r5
	bcc	.L517
	cmp	r3, r2
	beq	.L518
	ldr	r5, [r1, r3, asl #2]
	ldr	r0, [r1, r2, asl #2]
	str	r5, [r1, r2, asl #2]
	ldr	r1, [r4, #-2076]
	str	r0, [r1, r3, asl #2]
	ldrh	r1, [ip, r7]
	add	r2, r1, r2
	add	r3, r1, r3
	add	r2, ip, r2, asl #1
	add	r3, ip, r3, asl #1
	add	r2, r2, #4048
	add	r3, r3, #4048
	ldrh	r0, [r2, #0]
	ldrh	r1, [r3, #0]
	strh	r1, [r2, #0]	@ movhi
	strh	r0, [r3, #0]	@ movhi
.L518:
	mov	r3, r6
.L514:
	ldrh	r5, [ip, r8]
	sub	r2, r5, #1
	cmp	r3, r2
	blt	.L519
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L522:
	.align	2
.L521:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	IsInFreeQueue
	.type	IsInFreeQueue, %function
IsInFreeQueue:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L529
	movw	r3, #4046
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrh	r1, [r2, r3]
	cmp	r1, #1024
	beq	.L527
	movw	r3, #4042
	ldrh	ip, [r2, r3]
	mov	r3, #0
	b	.L525
.L526:
	add	r4, r3, ip
	mov	r4, r4, asl #22
	add	r4, r2, r4, lsr #21
	add	r4, r4, #4048
	ldrh	r4, [r4, #0]
	cmp	r4, r0
	beq	.L528
	add	r3, r3, #1
.L525:
	cmp	r3, r1
	bcc	.L526
.L527:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L528:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L530:
	.align	2
.L529:
	.word	.LANCHOR0
	.fnend
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L533
	movw	r3, #4046
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L532
	b	FtlFreeSysBlkQueueOut.part.5
.L532:
	movw	r0, #65535
	bx	lr
.L534:
	.align	2
.L533:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r3, #65535
	cmp	r0, r3
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	beq	.L536
	ldr	r1, .L550
	mov	ip, #6
	mul	ip, ip, r0
	mvn	r6, #0
	ldr	r4, [r1, #-2072]
	mov	r8, r1
	add	r2, r4, ip
	strh	r6, [r2, #2]	@ movhi
	strh	r6, [r4, ip]	@ movhi
	ldr	r3, [r1, #-2068]
	cmp	r3, #0
	beq	.L549
.L537:
	ldr	r7, [r1, #-2064]
	mov	sl, r0, asl #1
	ldrh	r1, [r2, #4]
	ldr	r9, .L550+4
	ldrh	r5, [r7, sl]
	cmp	r1, #0
	ldr	fp, [r8, #-2084]
	mulne	r6, r1, r5
	ldr	r5, [r8, #-2072]
	add	sl, fp, sl
	str	sl, [sp, #4]
	rsb	r1, r5, r3
	mov	r1, r1, asr #1
	mul	r1, r9, r1
	uxth	r1, r1
.L544:
	mov	r9, r1, asl #1
	ldrh	r8, [r3, #4]
	ldrh	sl, [r7, r9]
	cmp	r8, #0
	mvneq	r8, #0
	mulne	r8, r8, sl
	cmp	r8, r6
	bne	.L540
	ldrh	sl, [fp, r9]
	ldr	r9, [sp, #4]
	ldrh	r8, [r9, #0]
	cmp	sl, r8
	bcc	.L542
	b	.L541
.L540:
	bhi	.L541
.L542:
	ldrh	r8, [r3, #0]
	movw	sl, #65535
	cmp	r8, sl
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	ldreq	r3, .L550
	streq	r2, [r3, #-2060]
	beq	.L536
.L543:
	mov	r1, #6
	mla	r3, r1, r8, r5
	mov	r1, r8
	b	.L544
.L541:
	strh	r1, [r4, ip]	@ movhi
	ldrh	r1, [r3, #2]
	strh	r1, [r2, #2]	@ movhi
	ldr	r1, .L550
	ldr	ip, [r1, #-2068]
	cmp	r3, ip
	bne	.L545
	strh	r0, [r3, #2]	@ movhi
.L549:
	str	r2, [r1, #-2068]
	b	.L536
.L545:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-2072]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L536:
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L551:
	.align	2
.L550:
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
	ldr	r2, .L553
	ldr	r3, .L553+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L554:
	.align	2
.L553:
	.word	.LANCHOR2
	.word	-2056
	.fnend
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	2
	.global	insert_free_list
	.type	insert_free_list, %function
insert_free_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	movw	r6, #65535
	cmp	r0, r6
	beq	.L556
	ldr	r1, .L563
	mov	r5, #6
	mul	ip, r5, r0
	mvn	r3, #0
	ldr	r4, [r1, #-2072]
	add	r2, r4, ip
	strh	r3, [r2, #2]	@ movhi
	strh	r3, [r4, ip]	@ movhi
	ldr	r3, [r1, #-2052]
	cmp	r3, #0
	beq	.L562
.L557:
	ldr	sl, [r1, #-2072]
	mov	r8, r0, asl #1
	ldr	r7, [r1, #-2084]
	rsb	r1, sl, r3
	ldr	r9, .L563+4
	mov	r1, r1, asr #1
	ldrh	r8, [r7, r8]
	mul	r1, r9, r1
	mov	r9, r5
	uxth	r1, r1
.L560:
	mov	r5, r1, asl #1
	ldrh	r5, [r7, r5]
	cmp	r5, r8
	bcs	.L558
	ldrh	r5, [r3, #0]
	cmp	r5, r6
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	beq	.L556
.L559:
	mla	r3, r9, r5, sl
	mov	r1, r5
	b	.L560
.L558:
	ldrh	r5, [r3, #2]
	strh	r5, [r2, #2]	@ movhi
	strh	r1, [r4, ip]	@ movhi
	ldr	r1, .L563
	ldr	ip, [r1, #-2052]
	cmp	r3, ip
	bne	.L561
	strh	r0, [r3, #2]	@ movhi
.L562:
	str	r2, [r1, #-2052]
	b	.L556
.L561:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-2072]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L556:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L564:
	.align	2
.L563:
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
	ldr	r2, .L566
	ldr	r3, .L566+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L567:
	.align	2
.L566:
	.word	.LANCHOR2
	.word	-2048
	.fnend
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.type	List_remove_node, %function
List_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L573
	mov	r2, #6
	mul	r1, r2, r1
	ldr	r3, [ip, #-2072]
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r4, r3, r1
	ldr	r5, [r0, #0]
	movw	r6, #65535
	cmp	r4, r5
	ldrh	r5, [r3, r1]
	bne	.L569
	cmp	r5, r6
	mlane	r2, r2, r5, r3
	moveq	r2, #0
	streq	r2, [r0, #0]
	strne	r2, [r0, #0]
	mvnne	r0, #0
	strneh	r0, [r2, #2]	@ movhi
	b	.L571
.L569:
	cmp	r5, r6
	ldrh	r0, [r4, #2]
	mlane	r5, r2, r5, r3
	muleq	r2, r2, r0
	mvneq	r0, #0
	strneh	r0, [r5, #2]	@ movhi
	ldrneh	r6, [r4, #2]
	ldrneh	r5, [r3, r1]
	ldrne	r0, [ip, #-2072]
	mulne	r2, r2, r6
	streqh	r0, [r3, r2]	@ movhi
	strneh	r5, [r0, r2]	@ movhi
.L571:
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r3, r1]	@ movhi
	strh	r2, [r4, #2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L574:
	.align	2
.L573:
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
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r0, #0]
	cmp	r3, #0
	beq	.L580
	ldr	r2, .L581
	movw	lr, #65535
	mov	ip, #6
	ldr	r4, [r2, #-2072]
	b	.L577
.L579:
	mla	r3, ip, r2, r4
	sub	r1, r1, #1
	uxth	r1, r1
.L577:
	cmp	r1, #0
	beq	.L578
	ldrh	r2, [r3, #0]
	cmp	r2, lr
	bne	.L579
.L578:
	rsb	r4, r4, r3
	ldr	r3, .L581+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L580:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L582:
	.align	2
.L581:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	List_pop_index_node, .-List_pop_index_node
	.align	2
	.global	List_pop_head_node
	.type	List_pop_head_node, %function
List_pop_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r1, #0
	b	List_pop_index_node
	.fnend
	.size	List_pop_head_node, .-List_pop_head_node
	.align	2
	.global	List_get_gc_head_node
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L590
	ldr	r3, [r2, #-2068]
	cmp	r3, #0
	beq	.L589
	ldr	r2, [r2, #-2072]
	movw	r1, #65535
	mov	ip, #6
	b	.L586
.L588:
	mla	r3, ip, r3, r2
	sub	r0, r0, #1
	uxth	r0, r0
.L586:
	cmp	r0, #0
	beq	.L587
	ldrh	r3, [r3, #0]
	cmp	r3, r1
	bne	.L588
	mov	r0, r1
	bx	lr
.L587:
	rsb	r3, r2, r3
	ldr	r0, .L590+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	bx	lr
.L589:
	movw	r0, #65535
	bx	lr
.L591:
	.align	2
.L590:
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
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r4, .L598
	ldr	r3, .L598+4
	ldrh	r3, [r4, r3]
	cmp	r3, r0
	beq	.L593
	ldr	r3, .L598+8
	ldrh	r3, [r4, r3]
	cmp	r3, r0
	beq	.L593
	ldr	r3, .L598+12
	ldrh	r3, [r4, r3]
	cmp	r3, r0
	beq	.L593
	ldr	r3, [r4, #-2072]
	mov	ip, #6
	ldr	r2, [r4, #-2068]
	mla	r1, ip, r0, r3
	cmp	r1, r2
	beq	.L593
	ldrh	r6, [r1, #4]
	mov	r2, r5, asl #1
	ldrh	r1, [r1, #2]
	ldr	r0, [r4, #-2064]
	mla	r1, ip, r1, r3
	ldr	ip, .L598+16
	ldrh	r2, [r0, r2]
	muls	r2, r6, r2
	mvneq	r2, #0
	rsb	r3, r3, r1
	mov	r3, r3, asr #1
	ldrh	r1, [r1, #4]
	mul	r3, ip, r3
	uxth	r3, r3
	mov	r3, r3, asl #1
	ldrh	r3, [r0, r3]
	muls	r3, r1, r3
	mvneq	r3, #0
	cmp	r2, r3
	bcs	.L593
	ldr	r0, .L598+20
	mov	r1, r5
	bl	List_remove_node
	ldr	r3, .L598+24
	mov	r0, r5
	ldrh	r2, [r4, r3]
	sub	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	bl	INSERT_DATA_LIST
.L593:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L599:
	.align	2
.L598:
	.word	.LANCHOR2
	.word	-2044
	.word	-1996
	.word	-1948
	.word	-1431655765
	.word	.LANCHOR2-2068
	.word	-2056
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
	mov	r4, r0
	ldrh	r2, [r0, #10]
	mov	r5, #0
	ldr	r3, [r0, #12]
	b	.L601
.L604:
	mov	r7, r3
	add	r3, r3, #2
	ldrh	r6, [r7, #0]
	cmp	r6, #0
	bne	.L602
	bl	FtlFreeSysBlkQueueOut
	cmp	r0, #0
	strh	r0, [r7, #0]	@ movhi
	beq	.L603
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4, #0]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L603
.L602:
	add	r5, r5, #1
	uxth	r5, r5
.L601:
	cmp	r5, r2
	bne	.L604
.L603:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L617
	movw	r3, #3950
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	movw	r1, #65535
	ldrh	r3, [r2, r3]
	ldr	r2, .L617+4
	ldr	ip, [r2, #-1900]
	mov	r2, #0
	mov	r0, r2
	b	.L606
.L608:
	add	r2, r2, #12
	add	r4, ip, r2
	ldrh	r4, [r4, #-12]
	cmp	r4, r1
	ldmeqfd	sp!, {r4, r5, r6, r7, pc}
	add	r0, r0, #1
	uxth	r0, r0
.L606:
	cmp	r0, r3
	bne	.L608
	mov	r1, #0
	mov	r0, r3
	mov	r5, #-2147483648
	mov	r2, r1
	b	.L609
.L611:
	add	r4, ip, r1
	ldr	r4, [r4, #4]
	cmp	r4, #0
	blt	.L610
	cmp	r4, r5
	movcc	r5, r4
	movcc	r0, r2
.L610:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L609:
	cmp	r2, r3
	bne	.L611
	cmp	r0, r3
	ldmccfd	sp!, {r4, r5, r6, r7, pc}
	ldr	r2, .L617+8
	mov	r0, r3
	ldr	r1, .L617+4
	mvn	r4, #0
	ldrh	r6, [r1, r2]
	mov	r2, #0
	mov	r1, r2
	b	.L612
.L614:
	add	r5, ip, r2
	ldr	r5, [r5, #4]
	cmp	r5, r4
	bcs	.L613
	ldrh	r7, [ip, r2]
	cmp	r7, r6
	movne	r4, r5
	movne	r0, r1
.L613:
	add	r1, r1, #1
	add	r2, r2, #12
	uxth	r1, r1
.L612:
	cmp	r1, r3
	bne	.L614
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L618:
	.align	2
.L617:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1896
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L625
	ldr	r2, .L625+4
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r1, [r2, r3]
	cmp	r1, #4
	add	ip, r1, #1
	mov	r1, r3
	strh	ip, [r2, r3]	@ movhi
	bhi	.L620
	cmp	r0, #0
	ldmeqfd	sp!, {r4, r5, pc}
.L620:
	ldr	r0, .L625+8
	mov	r3, #0
	strh	r3, [r2, r1]	@ movhi
	movw	r1, #3852
	str	r3, [r2, #-1892]
	movw	ip, #65535
	ldrh	r4, [r0, r1]
	ldr	r1, [r2, #-2064]
	ldr	r2, .L625+4
	b	.L622
.L624:
	ldrh	r0, [r1], #2
	add	r3, r3, #1
	cmp	r0, ip
	uxth	r3, r3
	ldrne	r5, [r2, #-1892]
	addne	r0, r0, r5
	strne	r0, [r2, #-1892]
.L622:
	cmp	r3, r4
	bne	.L624
	ldmfd	sp!, {r4, r5, pc}
.L626:
	.align	2
.L625:
	.word	-1894
	.word	.LANCHOR2
	.word	.LANCHOR0
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
	ldr	r3, .L631
	mov	r2, r0, lsr #5
	cmp	r1, #0
	and	r0, r0, #31
	uxth	r2, r2
	mov	ip, #1
	ldr	r3, [r3, #-1888]
	ldr	r1, [r3, r2, asl #2]
	orrne	r0, r1, ip, asl r0
	biceq	r0, r1, ip, asl r0
	str	r0, [r3, r2, asl #2]
	bx	lr
.L632:
	.align	2
.L631:
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
	ldr	r3, .L634
	mov	r2, r0, lsr #5
	and	r0, r0, #31
	ldr	r3, [r3, #-1888]
	ldr	r3, [r3, r2, asl #2]
	mov	r0, r3, lsr r0
	and	r0, r0, #1
	bx	lr
.L635:
	.align	2
.L634:
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
	ldr	ip, .L643
	mov	r3, #0
	strh	r3, [r0, #4]	@ movhi
	movw	r3, #3844
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r3, [ip, r3]
	movw	ip, #65535
	b	.L637
.L639:
	add	r4, r0, r2, asl #1
	add	r2, r2, #1
	ldrh	r4, [r4, #16]
	uxth	r2, r2
	cmp	r4, ip
	ldrneh	r4, [r0, #4]
	addne	r4, r4, #1
	strneh	r4, [r0, #4]	@ movhi
.L637:
	cmp	r2, r3
	bcc	.L639
	ldr	ip, .L643
	movw	r2, #3912
	movw	r4, #65535
	mvn	r1, r1
	ldrh	r5, [ip, r2]
	mov	ip, r0
	mov	r2, #0
	b	.L640
.L642:
	ldrh	r6, [ip, #16]
	add	r2, r2, #1
	add	ip, ip, #2
	cmp	r6, r4
	uxth	r2, r2
	ldrneh	r6, [r0, #4]
	addne	r6, r5, r6
	addne	r6, r6, r1
	strneh	r6, [r0, #4]	@ movhi
.L640:
	cmp	r2, r3
	bne	.L642
	ldmfd	sp!, {r4, r5, r6, pc}
.L644:
	.align	2
.L643:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r4, r0
	mov	r5, #0
	ldr	r6, .L651
	strh	r5, [r0, #4]	@ movhi
	movw	r8, #3844
	strb	r5, [r4, #7]
	mvn	r7, #0
	b	.L646
.L648:
	add	r3, r6, r5
	ldrh	r1, [r4, #0]
	add	sl, r5, #8
	add	r5, r5, #1
	ldrb	r0, [r3, #3870]	@ zero_extendqisi2
	bl	V2P_block
	mov	sl, sl, asl #1
	uxth	r5, r5
	strh	r7, [r4, sl]	@ movhi
	mov	r9, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	streqh	r9, [r4, sl]	@ movhi
	ldreqb	r3, [r4, #7]	@ zero_extendqisi2
	addeq	r3, r3, #1
	streqb	r3, [r4, #7]
.L646:
	ldrh	r3, [r6, r8]
	cmp	r3, r5
	bhi	.L648
	ldr	r1, .L651
	movw	r2, #3912
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	ldrh	r2, [r1, r2]
	mul	r3, r2, r3
	strh	r3, [r4, #4]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #9]
	ldr	r3, .L651+4
	ldr	r2, [r3, #-1884]
	cmp	r2, #0
	beq	.L649
	ldrh	r1, [r4, #0]
	ldr	r2, [r3, #-2084]
	mov	r3, r1, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #59
	movls	r3, #1
	strlsb	r3, [r4, #9]
.L649:
	ldr	r3, .L651
	mov	r0, #0
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #1
	strneb	r3, [r4, #9]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L652:
	.align	2
.L651:
	.word	.LANCHOR0
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
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, #0
	mov	r7, r0
	mov	r5, r4
	ldr	r6, .L658
	movw	sl, #3844
	movw	r8, #3912
	b	.L654
.L656:
	add	r3, r6, r5
	mov	r1, r7
	add	r5, r5, #1
	ldrb	r0, [r3, #3870]	@ zero_extendqisi2
	bl	V2P_block
	uxth	r5, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [r6, r8]
	addeq	r4, r4, r3
	uxtheq	r4, r4
.L654:
	ldrh	r3, [r6, sl]
	cmp	r3, r5
	bhi	.L656
	cmp	r4, #0
	beq	.L657
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L657:
	ldr	r3, .L658+4
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #-2072]
	mla	r7, r2, r7, r3
	strh	r4, [r7, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L659:
	.align	2
.L658:
	.word	.LANCHOR0
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
	ldr	r3, .L663
	ldr	r0, [r3, #-2052]
	cmp	r0, #0
	bxeq	lr
	ldr	r2, [r3, #-2072]
	rsb	r0, r2, r0
	ldr	r2, .L663+4
	mov	r0, r0, asr #1
	mul	r0, r2, r0
	ldr	r2, [r3, #-2084]
	uxth	r0, r0
	mov	r3, r0, asl #1
	ldrh	r0, [r2, r3]
	bx	lr
.L664:
	.align	2
.L663:
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
	ldr	r2, .L672
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r3, [r2, #-2052]
	cmp	r3, #0
	beq	.L671
	ldr	r1, .L672+4
	mov	ip, #7
	mov	r5, #6
	movw	r4, #65535
	ldrh	r1, [r2, r1]
	mul	r1, ip, r1
	mov	r1, r1, asr #3
	cmp	r0, r1
	uxthgt	r0, r1
	ldr	r1, [r2, #-2072]
	ldr	r2, .L672+8
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
	b	.L668
.L670:
	mul	ip, r5, r3
	ldrh	ip, [r1, ip]
	cmp	ip, r4
	beq	.L669
	add	r2, r2, #1
	mov	r3, ip
	uxth	r2, r2
.L668:
	cmp	r2, r0
	bne	.L670
.L669:
	ldr	r2, .L672
	mov	r3, r3, asl #1
	ldr	r2, [r2, #-2084]
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L671:
	mov	r0, r3
	ldmfd	sp!, {r4, r5, pc}
.L673:
	.align	2
.L672:
	.word	.LANCHOR2
	.word	-2048
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r8, r0
	ldr	r6, .L684
	add	r5, r8, #12
	.pad #32
	sub	sp, sp, #32
	ldr	r1, .L684+4
	bl	strcpy
	mov	r0, r5
	ldr	r1, .L684+8
	ldr	r2, [r6, #2196]
	bl	sprintf
	ldr	r1, .L684+12
	ldr	r2, [r6, #3928]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+16
	ldr	r3, [r3, #2960]
	cmp	r3, #1
	add	r5, r5, r0
	bne	.L680
.L675:
	add	r0, sp, #16
	add	r1, sp, #20
	add	r2, sp, #24
	add	r3, sp, #28
	bl	NandcGetTimeCfg
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #16]
	mov	r0, r5
	ldr	r1, .L684+20
	str	r3, [sp, #0]
	ldr	r3, [sp, #28]
	ldr	r4, .L684+24
	ldr	r7, .L684+28
	str	r3, [sp, #4]
	ldr	r3, [sp, #20]
	bl	sprintf
	ldr	r1, .L684+32
	add	r5, r5, r0
	mov	r0, r5
	add	r5, r5, #10
	bl	strcpy
	ldr	r2, [r6, #3972]
	mov	r0, r5
	ldr	r1, .L684+36
	bl	sprintf
	ldr	r1, .L684+40
	ldr	r2, [r4, #-1892]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+44
	ldr	r2, [r4, #-1880]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+48
	ldr	r2, [r4, #-1876]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+52
	ldr	r2, [r4, #-1872]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+56
	ldr	r2, [r4, #-1868]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+60
	ldr	r2, [r4, #-1864]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+64
	ldr	r2, [r4, #-1860]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #-1856]
	ldr	r1, .L684+68
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #-1852]
	ldr	r1, .L684+72
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+76
	ldr	r2, [r4, #-1848]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+80
	ldr	r2, [r4, #-1844]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	FtlBbtCalcTotleCnt
	movw	r2, #3986
	ldr	r1, .L684+84
	ldrh	r2, [r6, r2]
	mov	r3, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+88
	ldrh	r2, [r4, r7]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+92
	ldr	r2, [r4, #-1840]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+96
	ldr	r2, [r4, #-1836]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+100
	ldr	r2, [r4, #-1832]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+104
	ldr	r2, [r4, #-2080]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+108
	ldr	r2, [r4, #-1828]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+112
	ldr	r2, [r4, #-1824]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+116
	ldr	r1, .L684+120
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+124
	ldr	r1, .L684+128
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #3952]
	ldr	r1, .L684+132
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #3944]
	ldr	r1, .L684+136
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #3840]
	ldr	r1, .L684+140
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #4046
	ldrh	r2, [r6, r3]
	ldr	r1, .L684+144
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #3852
	ldrh	r2, [r6, r3]
	ldr	r1, .L684+148
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+152
	ldr	r1, .L684+156
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #3856]
	ldr	r1, .L684+160
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+164
	ldr	r1, .L684+168
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #3980
	ldrh	r2, [r6, r3]
	ldr	r1, .L684+172
	ldr	r6, .L684+176
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+180
	ldr	r1, .L684+184
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+188
	ldrb	r2, [r4, #-2038]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, r6]
	ldr	r1, .L684+192
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+196
	ldrb	r2, [r4, #-2036]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+200
	ldr	r1, .L684+204
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #-2064]
	add	r6, r6, #48
	ldr	r1, .L684+208
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+212
	ldr	r1, .L684+216
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+220
	ldrb	r2, [r4, #-1990]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, r6]
	ldr	r1, .L684+224
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+228
	ldrb	r2, [r4, #-1988]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+232
	ldr	r1, .L684+236
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #-2064]
	add	r6, r6, #240
	ldr	r1, .L684+240
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+244
	ldr	r1, .L684+248
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+252
	ldrb	r2, [r4, #-1942]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+256
	ldr	r1, .L684+260
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+264
	ldrb	r2, [r4, #-1940]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+268
	ldr	r1, .L684+272
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+276
	ldr	r1, .L684+280
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+284
	ldrb	r2, [r4, #-1750]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+288
	ldrh	r2, [r4, r6]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+292
	ldrb	r2, [r4, #-1748]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+296
	ldr	r1, .L684+300
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, [r4, #-1620]
	ldr	r3, [r4, #-1884]
	ldr	r2, [r4, #-1708]
	str	r1, [sp, #0]
	ldr	r1, [r4, #-1628]
	orr	r2, r3, r2, asl #8
	str	r1, [sp, #4]
	ldr	r1, .L684+304
	ldr	r3, [r4, #-1624]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+308
	ldr	r2, [r4, #-1632]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+312
	ldr	r2, [r4, #-1608]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+316
	ldr	r1, .L684+320
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+324
	ldr	r1, .L684+328
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L684+332
	ldr	r2, [r4, #-1188]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L684+336
	ldr	r1, .L684+340
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L684+344
	mov	r2, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	ldrh	r0, [r4, r7]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L684+348
	mov	r2, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	movw	r2, #65535
	cmp	r3, r2
	add	r5, r5, r0
	beq	.L677
	ldr	r2, [r4, #-2064]
	mov	r3, r3, asl #1
	mov	r0, r5
	ldr	r1, .L684+352
	ldrh	r2, [r2, r3]
	bl	sprintf
	add	r5, r5, r0
.L677:
	mov	r0, #0
	ldr	r4, .L684+24
	bl	List_get_gc_head_node
	mov	r6, #0
	movw	sl, #65535
	mov	r9, #6
	uxth	r3, r0
.L679:
	cmp	r3, sl
	beq	.L678
	ldr	r1, [r4, #-2064]
	mov	r2, r3, asl #1
	mul	r7, r9, r3
	mov	r0, r5
	ldrh	r1, [r1, r2]
	str	r1, [sp, #0]
	ldr	r1, [r4, #-2072]
	add	r1, r1, r7
	ldrh	r1, [r1, #4]
	str	r1, [sp, #4]
	ldr	r1, [r4, #-2084]
	ldrh	r2, [r1, r2]
	ldr	r1, .L684+356
	str	r2, [sp, #8]
	mov	r2, r6
	bl	sprintf
	add	r6, r6, #1
	ldr	r3, [r4, #-2072]
	cmp	r6, #16
	ldrh	r3, [r3, r7]
	add	r5, r5, r0
	bne	.L679
.L678:
	ldr	r6, .L684+24
	mov	r4, #0
	movw	sl, #65535
	mov	r9, #6
	ldr	r2, [r6, #-2052]
	ldr	r3, [r6, #-2072]
	rsb	r3, r3, r2
	ldr	r2, .L684+360
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L681:
	cmp	r3, sl
	beq	.L680
	mul	r7, r9, r3
	ldr	r2, [r6, #-2072]
	mov	r0, r5
	add	r2, r2, r7
	ldrh	r2, [r2, #4]
	str	r2, [sp, #0]
	mov	r2, r3, asl #1
	ldr	r1, [r6, #-2084]
	ldrh	r2, [r1, r2]
	ldr	r1, .L684+364
	str	r2, [sp, #4]
	mov	r2, r4
	bl	sprintf
	add	r4, r4, #1
	ldr	r3, [r6, #-2072]
	cmp	r4, #4
	ldrh	r3, [r3, r7]
	add	r5, r5, r0
	bne	.L681
.L680:
	rsb	r0, r8, r5
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L685:
	.align	2
.L684:
	.word	.LANCHOR0
	.word	.LC4
	.word	.LC5
	.word	.LC6
	.word	.LANCHOR1
	.word	.LC7
	.word	.LANCHOR2
	.word	-2048
	.word	.LC8
	.word	.LC9
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
	.word	-1790
	.word	.LC29
	.word	-1792
	.word	.LC30
	.word	.LC31
	.word	.LC32
	.word	.LC33
	.word	.LC34
	.word	.LC35
	.word	-1772
	.word	.LC36
	.word	.LC37
	.word	-1768
	.word	.LC38
	.word	.LC39
	.word	-2044
	.word	-2042
	.word	.LC40
	.word	.LC41
	.word	.LC42
	.word	.LC43
	.word	-2040
	.word	.LC44
	.word	.LC45
	.word	-1994
	.word	.LC46
	.word	.LC47
	.word	.LC48
	.word	.LC49
	.word	-1992
	.word	.LC50
	.word	.LC51
	.word	-1946
	.word	.LC52
	.word	.LC53
	.word	-1948
	.word	.LC54
	.word	.LC55
	.word	-1944
	.word	.LC56
	.word	-1754
	.word	.LC57
	.word	.LC58
	.word	.LC59
	.word	.LC60
	.word	-1752
	.word	.LC61
	.word	.LC62
	.word	.LC63
	.word	.LC64
	.word	-1192
	.word	.LC65
	.word	-1190
	.word	.LC66
	.word	.LC67
	.word	-1184
	.word	.LC68
	.word	.LC69
	.word	.LC70
	.word	.LC71
	.word	.LC72
	.word	-1431655765
	.word	.LC73
	.fnend
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	2
	.global	rknand_proc_ftlread
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r1, .L687
	ldr	r2, .L687+4
	bl	sprintf
	add	r5, r4, r0
	mov	r0, r5
	bl	FtlPrintInfo2buf
	add	r0, r5, r0
	rsb	r0, r4, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L688:
	.align	2
.L687:
	.word	.LC74
	.word	.LC75
	.fnend
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	2
	.global	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L716
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r1, [r3, #-1832]
	ldr	r2, [r3, #-1824]
	cmp	r1, r2
	bcs	.L690
	ldr	r0, .L716+4
	movw	r1, #3852
	mov	r2, #0
	str	r2, [r3, #-1840]
	ldrh	r1, [r0, r1]
	ldr	r0, [r3, #-2084]
	b	.L691
.L692:
	ldrh	lr, [r0], #2
	add	r2, r2, #1
	ldr	ip, [r3, #-1840]
	add	ip, lr, ip
	str	ip, [r3, #-1840]
.L691:
	cmp	r2, r1
	ldr	r4, .L716
	bcc	.L692
	ldr	r5, [r4, #-1840]
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r2, .L716+4
	movw	r3, #3902
	ldrh	r1, [r2, r3]
	str	r0, [r4, #-1832]
	ldr	r0, [r4, #-1836]
	rsb	r0, r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #-1840]
	b	.L693
.L690:
	ldr	r2, [r3, #-1828]
	cmp	r1, r2
	bls	.L693
	add	r2, r2, #1
	ldr	r4, .L716+4
	str	r2, [r3, #-1828]
	movw	ip, #3852
	mov	r2, #0
	b	.L694
.L695:
	ldr	r0, [r3, #-2084]
	mov	r1, r2, asl #1
	add	r2, r2, #1
	ldrh	r5, [r0, r1]
	add	r5, r5, #1
	strh	r5, [r0, r1]	@ movhi
.L694:
	ldrh	r1, [r4, ip]
	cmp	r2, r1
	bcc	.L695
.L693:
	ldr	r3, .L716
	ldr	r6, [r3, #-1824]
	ldr	r5, [r3, #-1832]
	add	r2, r6, #256
	cmp	r2, r5
	mov	r2, r3
	bls	.L696
	ldr	r1, [r3, #-1828]
	add	r0, r6, #768
	cmp	r0, r1
	bls	.L696
	ldr	r3, [r3, #-1884]
	cmp	r3, #0
	beq	.L714
	cmp	r6, #30
	bhi	.L714
.L696:
	ldr	r3, .L716+8
	ldrh	r0, [r2, r3]
	add	r0, r0, r0, asl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	cmp	r0, r3
	mov	r8, r0
	bcs	.L698
	cmp	r6, #30
	bhi	.L714
.L698:
	ldr	r2, .L716
	ldr	r3, [r2, #-2068]
	cmp	r3, #0
	beq	.L714
	movw	r7, #65535
	ldr	r1, [r2, #-2072]
	mov	r4, r7
	ldr	r2, [r2, #-2084]
	mov	sl, r7
	ldr	ip, .L716+12
	mov	fp, #6
	b	.L699
.L702:
	ldrh	r9, [r3, #4]
	cmp	r9, #0
	beq	.L700
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	r9, r3, asl #1
	ldrh	r9, [r2, r9]
	cmp	r9, r6
	bls	.L711
	cmp	r9, r7
	movcc	r7, r9
	movcc	r4, r3
.L700:
	mla	r3, fp, r0, r1
.L699:
	ldrh	r0, [r3, #0]
	cmp	r0, sl
	bne	.L702
	b	.L701
.L711:
	mov	r4, r3
.L701:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L697
	mov	sl, r4, asl #1
	ldr	fp, .L716
	ldrh	r9, [r2, sl]
	cmp	r9, r6
	bls	.L703
	bl	GetFreeBlockMinEraseCount
	cmp	r0, r6
	strhi	r7, [fp, #-1824]
.L703:
	cmp	r9, #29
	ldr	r2, [fp, #-1884]
	movhi	r3, #0
	movls	r3, #1
	cmp	r2, #0
	moveq	r3, #0
	cmp	r3, #0
	beq	.L704
	add	r3, r9, #10
	cmp	r3, r5
	bls	.L705
	ldr	r2, .L716+4
	movw	r3, #3902
	ldrh	r3, [r2, r3]
	cmp	r3, r9
	bls	.L704
.L705:
	ldr	r6, .L716
	ldr	r3, .L716+16
	ldrh	r3, [r6, r3]
	cmp	r3, #64
	bls	.L704
	ldr	r3, [r6, #-2084]
	mov	r1, r4
	ldr	r0, .L716+20
	ldrh	r2, [r3, sl]
	bl	printk
	b	.L715
.L704:
	cmp	r9, r5
	bcs	.L714
	add	r3, r9, #128
	cmp	r8, r3
	ble	.L714
	add	r3, r9, #256
	ldr	r6, .L716
	cmp	r3, r5
	bcc	.L706
	ldr	r3, [r6, #-1828]
	add	r9, r9, #768
	cmp	r9, r3
	bcs	.L714
.L706:
	ldr	r3, [r6, #-2064]
	mov	r1, r4
	ldr	r0, .L716+24
	mov	r2, r5
	ldrh	r3, [r3, sl]
	str	r3, [sp, #0]
	ldr	r3, [r6, #-2084]
	ldrh	r3, [r3, sl]
	stmib	sp, {r3, r8}
	ldr	r3, [r6, #-1828]
	bl	printk
.L715:
	mov	r3, #1
	str	r3, [r6, #-1180]
	b	.L697
.L714:
	movw	r4, #65535
.L697:
	mov	r0, r4
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L717:
	.align	2
.L716:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-2048
	.word	-1431655765
	.word	-2056
	.word	.LC76
	.word	.LC77
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
	beq	.L719
	ldr	r2, .L720
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #-2064]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L719:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L721:
	.align	2
.L720:
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
	ldr	r2, .L730
	mov	r3, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	sl, #3844
	str	r3, [r2, #-1176]
	mov	r8, #12
	ldr	r0, .L730+4
	mov	r7, #1
	movw	r6, #3922
	movw	fp, #3924
	b	.L723
.L724:
	mul	r1, r8, r3
	ldr	r4, [r2, #-1172]
	add	ip, r4, r1
	str	r7, [ip, #8]
	ldrh	ip, [r0, r6]
	mul	ip, ip, r3
	add	r5, ip, #3
	cmp	ip, #0
	movlt	ip, r5
	ldr	r5, [r2, #-1168]
	bic	ip, ip, #3
	add	ip, r5, ip
	str	ip, [r4, r1]
	ldrh	ip, [r0, fp]
	ldr	r9, [r2, #-1172]
	mul	ip, ip, r3
	add	r4, r9, r1
	add	r5, ip, #3
	cmp	ip, #0
	movlt	ip, r5
	ldr	r5, [r2, #-1164]
	bic	ip, ip, #3
	add	ip, r5, ip
	str	ip, [r4, #4]
	ldr	ip, [r2, #-1160]
	mov	r5, #36
	ldr	r1, [r9, r1]
	mla	ip, r5, r3, ip
	add	r3, r3, #1
	uxth	r3, r3
	str	r1, [ip, #8]
	ldr	r1, [r4, #4]
	str	r1, [ip, #12]
.L723:
	ldrh	r1, [r0, sl]
	cmp	r3, r1
	bcc	.L724
	b	.L729
.L726:
	mul	r2, r8, r1
	ldr	r4, [r3, #-1172]
	add	r0, r4, r2
	str	r7, [r0, #8]
	ldrh	r0, [ip, r6]
	mul	r0, r0, r1
	add	sl, r0, #3
	cmp	r0, #0
	movlt	r0, sl
	ldr	sl, [r3, #-1168]
	bic	r0, r0, #3
	add	r0, sl, r0
	str	r0, [r4, r2]
	ldr	r0, [r3, #-1172]
	add	r0, r0, r2
	ldrh	r2, [ip, r5]
	mul	r2, r2, r1
	add	r1, r1, #1
	uxth	r1, r1
	add	r4, r2, #3
	cmp	r2, #0
	movlt	r2, r4
	ldr	r4, [r3, #-1164]
	bic	r2, r2, #3
	add	r2, r4, r2
	str	r2, [r0, #4]
	b	.L728
.L729:
	ldr	r3, .L730
	mov	r8, #12
	ldr	ip, .L730+4
	mov	r7, #0
	movw	r6, #3922
	movw	r5, #3924
.L728:
	ldr	r2, [r3, #-1156]
	cmp	r1, r2
	bcc	.L726
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L731:
	.align	2
.L730:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L739
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, #36
	ldr	r6, [r3, #-1156]
	mov	fp, #12
	ldr	ip, [r3, #-1172]
	mov	r3, #0
	mov	r4, r3
	b	.L733
.L736:
	mul	sl, fp, r2
	add	r8, ip, sl
	ldr	r9, [ip, sl]
	ldr	sl, [r7, #8]
	cmp	r9, sl
	streq	r4, [r8, #8]
	beq	.L735
.L734:
	add	r2, r2, #1
	uxth	r2, r2
.L738:
	cmp	r2, r6
	bcc	.L736
.L735:
	add	r3, r3, #1
	uxth	r3, r3
.L733:
	cmp	r3, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
	mla	r7, r5, r3, r0
	mov	r2, #0
	b	.L738
.L740:
	.align	2
.L739:
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
	ldr	r3, .L748
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #12
	ldr	r8, [r3, #-1156]
	mov	r5, #1
	ldr	r7, [r3, #-1172]
	mov	r4, #36
	mov	r3, #0
	b	.L742
.L745:
	mla	ip, r6, r2, r7
	ldr	sl, [ip, #8]
	cmp	sl, #0
	bne	.L743
	mla	r2, r4, r3, r0
	ldr	sl, [ip, #0]
	str	r5, [ip, #8]
	str	sl, [r2, #8]
	ldr	ip, [ip, #4]
	str	ip, [r2, #12]
	b	.L744
.L743:
	add	r2, r2, #1
	uxth	r2, r2
	b	.L746
.L747:
	mov	r2, #0
.L746:
	cmp	r2, r8
	bcc	.L745
.L744:
	add	r3, r3, #1
	uxth	r3, r3
.L742:
	cmp	r3, r1
	bcc	.L747
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L749:
	.align	2
.L748:
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
	ldr	r3, .L755
	ldr	r2, .L755+4
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #-1148]
	mov	r3, #0
	b	.L751
.L753:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L754
	add	r3, r3, #1
	uxth	r3, r3
.L751:
	cmp	r3, r1
	bne	.L753
	mov	r0, #0
	bx	lr
.L754:
	mov	r0, #1
	bx	lr
.L756:
	.align	2
.L755:
	.word	.LANCHOR2
	.word	-1152
	.fnend
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	2
	.global	FtlGcUpdatePage
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ubfx	r0, r0, #10, #16
	mov	r5, r1
	mov	r6, r2
	bl	P2V_block_in_plane
	ldr	r3, .L761
	ldr	r2, .L761+4
	ldrh	ip, [r3, r2]
	ldr	r2, [r3, #-1148]
	mov	r3, #0
	mov	r1, r2
	b	.L758
.L760:
	ldrh	r7, [r1], #2
	cmp	r7, r0
	beq	.L759
	add	r3, r3, #1
	uxth	r3, r3
.L758:
	cmp	r3, ip
	bne	.L760
	mov	r3, r3, asl #1
	strh	r0, [r2, r3]	@ movhi
	ldr	r2, .L761
	ldr	r3, .L761+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L759:
	ldr	r3, .L761
	mov	r0, #12
	ldr	r2, .L761+8
	ldrh	r1, [r3, r2]
	mul	r1, r0, r1
	ldr	r0, [r3, #-1144]
	add	ip, r0, r1
	stmib	ip, {r5, r6}
	str	r4, [r0, r1]
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L762:
	.align	2
.L761:
	.word	.LANCHOR2
	.word	-1152
	.word	-1140
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	mov	r1, r4
	ldr	r0, .L766
	bl	printk
	ldr	r3, .L766+4
	ldr	ip, .L766+8
	ldrh	r5, [r3, ip]
	cmp	r5, r4
	beq	.L764
	ldr	r2, .L766+12
	ldrh	r0, [r3, r2]
	cmp	r0, r4
	beq	.L764
	movw	r1, #65535
	cmp	r5, r1
	streqh	r4, [r3, ip]	@ movhi
	beq	.L764
	cmp	r0, r1
	streqh	r4, [r3, r2]	@ movhi
.L764:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L767:
	.align	2
.L766:
	.word	.LC78
	.word	.LANCHOR2
	.word	-1138
	.word	-1136
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
	mov	r4, r0
	bl	P2V_block_in_plane
	ldr	r5, .L773
	mov	r2, r4
	ldr	r7, .L773+4
	ldrh	r1, [r5, r7]
	mov	r6, r0
	ldr	r0, .L773+8
	bl	printk
	mov	r0, r6
	bl	FtlGcRefreshBlock
	ldr	r3, [r5, #-1884]
	cmp	r3, #0
	beq	.L769
	ldr	r3, [r5, #-2084]
	mov	r6, r6, asl #1
	ldrh	r2, [r3, r6]
	cmp	r2, #29
	subhi	r2, r2, #30
	strhih	r2, [r3, r6]	@ movhi
.L769:
	ldrh	r1, [r5, r7]
	mov	r3, #0
	ldr	r2, .L773+12
	b	.L770
.L772:
	ldrh	r0, [r2, #2]!
	cmp	r0, r4
	beq	.L771
	add	r3, r3, #1
	uxth	r3, r3
.L770:
	cmp	r3, r1
	bne	.L772
	cmp	r3, #15
	bhi	.L771
	ldr	r2, .L773
	ldr	r1, .L773+16
	add	r0, r2, r3, asl #1
	add	r3, r3, #1
	strh	r4, [r0, r1]	@ movhi
	sub	r1, r1, #2
	strh	r3, [r2, r1]	@ movhi
.L771:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L774:
	.align	2
.L773:
	.word	.LANCHOR2
	.word	-1134
	.word	.LC79
	.word	.LANCHOR2-1134
	.word	-1132
	.fnend
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	2
	.global	FtlGcReFreshBadBlk
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r2, .L778
	ldr	r3, .L778+4
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L776
	ldr	r1, .L778+8
	ldrh	r0, [r3, r1]
	movw	r1, #65535
	cmp	r0, r1
	bne	.L776
	ldr	r4, .L778+12
	ldr	r5, .L778+4
	ldrh	r1, [r3, r4]
	cmp	r1, r2
	movcs	r2, #0
	strcsh	r2, [r3, r4]	@ movhi
	ldrh	r2, [r5, r4]
	ldr	r3, .L778+16
	add	r2, r5, r2, asl #1
	ldrh	r0, [r2, r3]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r5, r4]
	add	r3, r3, #1
	strh	r3, [r5, r4]	@ movhi
.L776:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L779:
	.align	2
.L778:
	.word	-1134
	.word	.LANCHOR2
	.word	-1138
	.word	-1098
	.word	-1132
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
	beq	.L781
	bl	memset
.L781:
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
	.fnend
	.size	ftl_memset, .-ftl_memset
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r3, #0
	ldr	r4, .L783
	movw	r5, #3916
	ldr	r2, .L783+4
	mov	r1, #255
	ldr	r6, .L783+8
	ldr	r0, [r4, #-1148]
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #12
	strh	r3, [r4, r2]	@ movhi
	ldrh	r2, [r6, r5]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r6, r5]
	mov	r2, #12
	ldr	r0, [r4, #-1144]
	mov	r1, #255
	mul	r2, r2, r3
	bl	ftl_memset
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlGcBufInit
.L784:
	.align	2
.L783:
	.word	.LANCHOR2
	.word	-1152
	.word	.LANCHOR0
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r3, #3854
	ldr	r7, .L794
	mov	r2, #6
	ldr	r4, .L794+4
	mov	r1, #0
	mov	r6, #0
	ldrh	r3, [r7, r3]
	mov	sl, r6
	ldr	r0, [r4, #-2072]
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r3, .L794+8
	str	r6, [r4, #-2052]
	str	r6, [r4, #-2068]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #8
	str	r6, [r4, #-2060]
	strh	r6, [r4, r3]	@ movhi
	mov	r4, r6
	b	.L786
.L788:
	add	r3, r7, r8
	mov	r1, r4
	add	r8, r8, #1
	ldrb	r0, [r3, #3870]	@ zero_extendqisi2
	bl	V2P_block
	uxth	r8, r8
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [r7, r9]
	addeq	r5, r5, r3
	uxtheq	r5, r5
	b	.L792
.L793:
	mov	r5, #0
	movw	fp, #3844
	mov	r8, r5
	movw	r9, #3912
.L792:
	ldrh	r3, [r7, fp]
	cmp	r3, r8
	bhi	.L788
	cmp	r5, #0
	beq	.L789
	mov	r1, r5
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r5, r0
.L789:
	ldr	r3, .L794+4
	mov	r1, #6
	ldr	r2, [r3, #-2072]
	mla	r2, r1, r4, r2
	strh	r5, [r2, #4]	@ movhi
	ldr	r2, .L794+12
	ldrh	r2, [r3, r2]
	cmp	r2, r4
	beq	.L790
	ldr	r2, .L794+16
	ldrh	r2, [r3, r2]
	cmp	r2, r4
	beq	.L790
	ldr	r2, .L794+20
	ldrh	r2, [r3, r2]
	cmp	r2, r4
	beq	.L790
	ldr	r2, [r3, #-2064]
	mov	r3, r4, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L791
	add	r6, r6, #1
	mov	r0, r4
	uxth	r6, r6
	bl	INSERT_FREE_LIST
	b	.L790
.L791:
	add	sl, sl, #1
	mov	r0, r4
	uxth	sl, sl
	bl	INSERT_DATA_LIST
.L790:
	add	r4, r4, #1
	uxth	r4, r4
.L786:
	movw	r2, #3852
	ldrh	r3, [r7, r2]
	cmp	r3, r4
	bhi	.L793
	ldr	r3, .L794+4
	mov	r0, #0
	ldr	r2, .L794+8
	strh	sl, [r3, r2]	@ movhi
	add	r2, r2, #8
	strh	r6, [r3, r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L795:
	.align	2
.L794:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-2056
	.word	-2044
	.word	-1996
	.word	-1948
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	FtlL2PDataInit
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r1, #0
	ldr	r4, .L799
	movw	r6, #3922
	ldr	r5, .L799+4
	movw	r7, #3950
	mov	r8, #12
	ldr	r2, [r4, #3940]
	ldr	r0, [r5, #-1096]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, r6]
	ldrh	r2, [r4, r7]
	mov	r1, #255
	ldr	r0, [r5, #-1092]
	mul	r2, r2, r3
	bl	ftl_memset
	mov	r2, #0
	mov	ip, r2
	mvn	r0, #0
	b	.L797
.L798:
	mul	r3, r8, r2
	ldr	r1, [r5, #-1900]
	add	sl, r1, r3
	str	ip, [sl, #4]
	strh	r0, [r1, r3]	@ movhi
	ldr	r1, [r5, #-1900]
	ldr	sl, [r5, #-1092]
	add	r3, r1, r3
	ldrh	r1, [r4, r6]
	mul	r1, r2, r1
	add	r2, r2, #1
	uxth	r2, r2
	bic	r1, r1, #3
	add	r1, sl, r1
	str	r1, [r3, #8]
.L797:
	ldrh	r3, [r4, r7]
	ldr	r1, .L799
	cmp	r3, r2
	ldr	r3, .L799+4
	bhi	.L798
	ldr	r0, .L799+8
	mvn	r2, #0
	strh	r2, [r3, r0]	@ movhi
	ldr	r0, .L799+12
	strh	r2, [r3, r0]	@ movhi
	ldr	r0, [r1, #3940]
	ldr	r2, .L799+16
	strh	r0, [r3, r2]	@ movhi
	sub	r2, r2, #6
	ldr	r0, .L799+20
	strh	r0, [r3, r2]	@ movhi
	add	r2, r2, #36
	ldrh	r0, [r3, r2]
	sub	r2, r2, #32
	strh	r0, [r3, r2]	@ movhi
	movw	r2, #3948
	ldrh	r1, [r1, r2]
	ldr	r2, .L799+24
	strh	r1, [r3, r2]	@ movhi
	ldr	r2, [r3, #-1044]
	str	r2, [r3, #-1076]
	ldr	r2, [r3, #-1040]
	str	r2, [r3, #-1072]
	ldr	r2, [r3, #-1096]
	str	r2, [r3, #-1068]
	ldr	r2, [r3, #-1036]
	str	r2, [r3, #-1064]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L800:
	.align	2
.L799:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1086
	.word	-1088
	.word	-1078
	.word	-3902
	.word	-1082
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	ldrh	r2, [r0, #10]
	mov	r1, #0
	ldr	r5, [r0, #20]
	ldr	r6, [r0, #12]
	ldr	r7, [r0, #24]
	mov	r2, r2, asl #1
	mov	r0, r5
	bl	ftl_memset
	mov	r3, #0
	b	.L802
.L806:
	ldr	r0, [r7, r3, asl #2]
	mov	r2, #0
	ubfx	r0, r0, #10, #16
	b	.L803
.L805:
	mov	r1, r2, asl #1
	add	r2, r2, #1
	ldrh	ip, [r6, r1]
	uxth	r2, r2
	cmp	ip, r0
	ldreqh	ip, [r5, r1]
	addeq	ip, ip, #1
	streqh	ip, [r5, r1]	@ movhi
.L803:
	ldrh	r1, [r4, #10]
	cmp	r1, r2
	bhi	.L805
	add	r3, r3, #1
	uxth	r3, r3
.L802:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L806
	mov	sl, #0
	ldrh	fp, [r5, #0]
	mov	r7, sl
	ldr	r3, .L812
	movw	r2, #3914
	b	.L807
.L811:
	ldrh	r1, [r4, #0]
	cmp	r1, r7
	bne	.L808
	ldrh	r0, [r4, #2]
	ldrh	r1, [r3, r2]
	cmp	r0, r1
	movcc	r0, r7, asl #1
	strcch	r1, [r5, r0]	@ movhi
.L808:
	mov	r9, r7, asl #1
	ldrh	r8, [r5, r9]
	cmp	fp, r8
	movhi	sl, r7
	movhi	fp, r8
	cmp	r8, #0
	bne	.L810
	ldrh	r0, [r6, r9]
	cmp	r0, #0
	beq	.L810
	mov	r1, #1
	stmia	sp, {r2, r3}
	bl	FtlFreeSysBlkQueueIn
	strh	r8, [r6, r9]	@ movhi
	ldmia	sp, {r2, r3}
	ldrh	r1, [r4, #8]
	sub	r1, r1, #1
	strh	r1, [r4, #8]	@ movhi
.L810:
	add	r7, r7, #1
	uxth	r7, r7
.L807:
	ldrh	r1, [r4, #10]
	cmp	r1, r7
	bhi	.L811
	mov	r0, sl
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L813:
	.align	2
.L812:
	.word	.LANCHOR0
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L815
	movw	r2, #4042
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, #0
	strh	r4, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, r4
	strh	r4, [r3, r2]	@ movhi
	add	r2, r2, #2
	strh	r4, [r3, r2]	@ movhi
	movw	r2, #4040
	strh	r0, [r3, r2]	@ movhi
	add	r0, r3, #4048
	mov	r2, #2048
	bl	ftl_memset
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L816:
	.align	2
.L815:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	2
	.global	FtlBbtMemInit
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L818
	movw	r2, #3980
	mvn	r1, #0
	ldr	r0, .L818+4
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #6
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #255
	mov	r2, #16
	b	ftl_memset
.L819:
	.align	2
.L818:
	.word	.LANCHOR0
	.word	.LANCHOR0+3992
	.fnend
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L824
	ldr	r2, .L824+4
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldrh	r2, [r2, r3]
	mov	r0, r1
	mov	r4, r1
	mov	r1, #0
	mov	r2, r2, asl #2
	bl	ftl_memset
	mov	r3, #0
	movw	ip, #65535
	mov	r0, #1
.L822:
	ldrh	r2, [r5, r3]
	cmp	r2, ip
	ldmeqfd	sp!, {r4, r5, r6, pc}
	mov	r1, r2, lsr #5
	add	r3, r3, #2
	cmp	r3, #1024
	and	r2, r2, #31
	ldr	r6, [r4, r1, asl #2]
	orr	r2, r6, r0, asl r2
	str	r2, [r4, r1, asl #2]
	bne	.L822
	ldmfd	sp!, {r4, r5, r6, pc}
.L825:
	.align	2
.L824:
	.word	-1032
	.word	.LANCHOR2
	.fnend
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
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
	ldr	r5, .L827
	mov	r4, #0
	ldr	r2, .L827+4
	mov	r1, r4
	ldr	r6, .L827+8
	movw	r7, #3854
	str	r3, [r5, #-1012]
	strh	r3, [r5, r2]	@ movhi
	movw	r3, #3956
	strh	r4, [r6, r3]	@ movhi
	movw	r3, #3932
	ldrh	r2, [r6, r3]
	ldr	r0, [r6, #3960]
	str	r4, [r5, #-1028]
	mov	r2, r2, asl #1
	str	r4, [r5, #-1020]
	str	r4, [r5, #-1016]
	str	r4, [r5, #-1884]
	bl	ftl_memset
	ldrh	r2, [r6, r7]
	mov	r1, r4
	ldr	r0, [r5, #-2084]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r6, r7]
	mov	r1, r4
	ldr	r0, [r5, #-1008]
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #48
	ldr	r0, .L827+12
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #512
	ldr	r0, .L827+16
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L828:
	.align	2
.L827:
	.word	.LANCHOR2
	.word	-1024
	.word	.LANCHOR0
	.word	.LANCHOR2-1820
	.word	.LANCHOR2-1704
	.fnend
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	movw	r3, #65535
	ldr	r4, .L860
	mov	r6, #0
	ldr	r2, .L860+4
	mov	r0, #1024
	ldr	r5, .L860+8
	mov	r7, #12
	str	r3, [r4, #-996]
	mvn	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #2
	str	r6, [r4, #-1004]
	movw	sl, #3844
	strh	r3, [r4, r2]	@ movhi
	mov	r2, #32
	ldr	r3, .L860+12
	mov	r8, #36
	str	r6, [r4, #-1708]
	str	r6, [r4, #-1848]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	mov	r2, #128
	str	r6, [r4, #-1844]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #6
	str	r6, [r4, #-1860]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #50
	str	r6, [r4, #-1872]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #36
	str	r6, [r4, #-1876]
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #3918
	ldrh	r1, [r5, r3]
	str	r6, [r4, #-1868]
	str	r6, [r4, #-1864]
	str	r6, [r4, #-1880]
	str	r6, [r4, #-1840]
	str	r6, [r4, #-1836]
	str	r6, [r4, #-2080]
	str	r6, [r4, #-1828]
	str	r6, [r4, #-1824]
	str	r6, [r4, #-1000]
	str	r6, [r4, #-1180]
	str	r6, [r4, #-992]
	str	r6, [r4, #-1188]
	str	r6, [r5, #3976]
	bl	__aeabi_idiv
	movw	r3, #3844
	ldrh	r3, [r5, r3]
	str	r6, [r5, #3964]
	movw	r6, #3916
	mov	r3, r3, asl #2
	cmp	r0, r3
	str	r0, [r4, #-988]
	ldrh	r0, [r5, r6]
	strhi	r3, [r4, #-988]
	ldr	r4, .L860
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-1148]
	ldrh	r0, [r5, r6]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r6, [r5, sl]
	mul	r6, r8, r6
	mov	r9, r6, asl #3
	str	r0, [r4, #-1144]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-984]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-980]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-976]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-2088]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-1160]
	ldr	r0, [r4, #-988]
	mul	r0, r8, r0
	bl	ftl_malloc
	movw	r8, #3922
	ldrh	r3, [r5, sl]
	ldrh	r6, [r5, r8]
	mov	r3, r3, asl #1
	add	r3, r3, #1
	str	r3, [r4, #-1156]
	str	r0, [r5, #3968]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-2076]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-972]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-968]
	ldr	r0, [r4, #-1156]
	mul	r0, r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-1168]
	ldr	r0, [r4, #-988]
	mul	r0, r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-964]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-960]
	mov	r0, r6
	bl	ftl_malloc
	movw	r6, #3924
	str	r0, [r4, #-956]
	ldr	r0, [r4, #-1156]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	ldrh	sl, [r5, sl]
	mul	sl, sl, r3
	str	r0, [r4, #-1172]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-952]
	mov	r0, sl, asl #3
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	ldr	sl, .L860+16
	str	r0, [r4, #-948]
	ldr	r0, [r4, #-1156]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	movw	r6, #3854
	str	r0, [r4, #-1164]
	ldr	r0, [r4, #-988]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #-944]
	ldrh	r0, [r5, r6]
	mov	r0, r0, asl #1
	uxth	r0, r0
	strh	r0, [r4, sl]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #-1008]
	ldrh	r0, [r4, sl]
	add	r0, r0, #544
	add	r0, r0, #3
	mov	r0, r0, lsr #9
	strh	r0, [r4, sl]	@ movhi
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldrh	sl, [r5, r6]
	mov	sl, sl, asl #1
	str	r0, [r4, #-936]
	add	r0, r0, #32
	str	r0, [r4, #-2084]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-932]
	mov	r0, sl
	bl	ftl_malloc
	ldr	sl, [r5, #3940]
	mov	sl, sl, asl #1
	str	r0, [r4, #-2064]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-1044]
	mov	r0, sl
	bl	ftl_malloc
	movw	sl, #3932
	str	r0, [r4, #-1096]
	ldrh	r0, [r5, r6]
	mov	r0, r0, lsr #3
	add	r0, r0, #4
	bl	ftl_malloc
	str	r0, [r4, #-1888]
	ldrh	r0, [r5, sl]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r5, #3960]
	ldrh	r0, [r5, sl]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-928]
	ldrh	r0, [r5, sl]
	add	sl, sl, #2
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-924]
	ldrh	r0, [r5, sl]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r2, [r5, sl]
	mov	r1, #0
	add	sl, sl, #16
	mov	r2, r2, asl #2
	str	r0, [r4, #-920]
	bl	ftl_memset
	movw	r3, #3948
	ldrh	r0, [r5, r3]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-1036]
	ldr	r0, [r5, #3940]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-1040]
	ldrh	r0, [r5, sl]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r5, sl]
	movw	r7, #3866
	str	r0, [r4, #-1900]
	ldrh	r0, [r5, r8]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	ldr	r6, .L860+20
	str	r0, [r4, #-1092]
	mov	r0, #6
	mul	r0, r0, r3
	bl	ftl_malloc
	movw	r3, #3908
	ldrh	r3, [r5, r3]
	ldrh	r2, [r5, r7]
	add	r3, r3, #31
	mov	r3, r3, lsr #5
	strh	r3, [r4, r6]	@ movhi
	mul	r3, r2, r3
	str	r0, [r4, #-2072]
	mov	r0, r3, asl #2
	bl	ftl_malloc
	ldrh	r1, [r4, r6]
	mov	r3, #1
	ldr	ip, .L860+8
	mov	r1, r1, asl #2
	mov	r2, r1
	str	r0, [r5, #4008]
	ldrh	r5, [r5, r7]
	ldr	r0, .L860+24
	b	.L831
.L832:
	ldr	r4, [ip, #4008]
	add	r3, r3, #1
	add	r4, r4, r2
	add	r2, r2, r1
	str	r4, [r0, #4]!
.L831:
	cmp	r3, r5
	bcc	.L832
	ldr	r0, .L860+28
	mov	r2, #0
	mov	r1, r2
	add	r0, r0, r3, asl #2
	b	.L833
.L834:
	add	ip, r0, r2
	add	r3, r3, #1
	add	r2, r2, #4
	str	r1, [ip, #28]
.L833:
	cmp	r3, #7
	bls	.L834
	ldr	r3, .L860
	ldr	r2, [r3, #-1044]
	cmp	r2, #0
	beq	.L859
.L835:
	ldr	r2, [r3, #-1096]
	cmp	r2, #0
	beq	.L859
.L837:
	ldr	r2, [r3, #-1036]
	cmp	r2, #0
	beq	.L859
.L838:
	ldr	r2, [r3, #-1040]
	cmp	r2, #0
	beq	.L859
.L839:
	ldr	r2, [r3, #-1900]
	cmp	r2, #0
	beq	.L859
.L840:
	ldr	r2, [r3, #-1092]
	cmp	r2, #0
	beq	.L859
.L841:
	ldr	r2, [r3, #-2072]
	cmp	r2, #0
	beq	.L859
.L842:
	ldr	r2, .L860+8
	ldr	r2, [r2, #4008]
	cmp	r2, #0
	beq	.L859
.L843:
	ldr	r3, [r3, #-2064]
	cmp	r3, #0
	beq	.L859
.L844:
	ldr	r3, .L860
	ldr	r2, [r3, #-1148]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-1144]
	cmp	r2, #0
	beq	.L859
.L846:
	ldr	r2, [r3, #-984]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-976]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-2088]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-1160]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-980]
	cmp	r2, #0
	beq	.L859
.L848:
	ldr	r2, [r3, #-2076]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-972]
	cmp	r2, #0
	beq	.L859
	ldr	r3, [r3, #-968]
	cmp	r3, #0
	beq	.L859
	ldr	r3, .L860
	ldr	r2, [r3, #-1168]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-960]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-956]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-1172]
	cmp	r2, #0
	beq	.L859
.L850:
	ldr	r2, [r3, #-952]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-948]
	cmp	r2, #0
	beq	.L859
	ldr	r2, [r3, #-1164]
	cmp	r2, #0
	beq	.L859
.L852:
	ldr	r2, [r3, #-2084]
	cmp	r2, #0
	beq	.L859
	ldr	r3, [r3, #-1008]
	cmp	r3, #0
	beq	.L859
.L854:
	ldr	r3, .L860+8
	ldr	r3, [r3, #3960]
	cmp	r3, #0
	beq	.L859
.L855:
	ldr	r3, .L860
	ldr	r2, [r3, #-928]
	cmp	r2, #0
	beq	.L859
.L856:
	ldr	r2, [r3, #-924]
	cmp	r2, #0
	beq	.L859
.L857:
	ldr	r3, [r3, #-920]
	cmp	r3, #0
	bne	.L858
.L859:
	ldr	r0, .L860+32
	ldr	r1, .L860+36
	bl	printk
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L858:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L861:
	.align	2
.L860:
	.word	.LANCHOR2
	.word	-1138
	.word	.LANCHOR0
	.word	-1192
	.word	-940
	.word	-1032
	.word	.LANCHOR0+4008
	.word	.LANCHOR0+3980
	.word	.LC80
	.word	.LANCHOR3
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	ReadFlashInfo
	.type	ReadFlashInfo, %function
ReadFlashInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r1, #0
	mov	r2, #11
	mov	r4, r0
	bl	ftl_memset
	ldr	r3, .L865
	ldr	r1, [r3, #2772]
	ldr	r0, [r3, #4]
	ldrb	r2, [r1, #9]	@ zero_extendqisi2
	mul	r2, r0, r2
	mov	r0, #0
	uxth	r2, r2
	strb	r2, [r4, #4]
	mov	ip, r2, lsr #8
	strb	ip, [r4, #5]
	ldrb	ip, [r3, #3836]	@ zero_extendqisi2
	strb	ip, [r4, #7]
	ldrb	ip, [r1, #13]	@ zero_extendqisi2
	mul	r2, r2, ip
	ldrh	ip, [r1, #14]
	ldrb	r1, [r1, #8]	@ zero_extendqisi2
	mul	r2, ip, r2
	mul	r2, r1, r2
	ubfx	r1, r2, #8, #8
	strb	r1, [r4, #1]
	ubfx	r1, r2, #16, #8
	strb	r1, [r4, #2]
	strb	r2, [r4, #0]
	mov	r2, r2, lsr #24
	strb	r2, [r4, #3]
	ldr	r2, [r3, #2772]
	ldrb	ip, [r3, #3762]	@ zero_extendqisi2
	mov	r3, r0
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	strb	r1, [r4, #6]
	mov	r1, #32
	strb	r1, [r4, #8]
	ldrb	r2, [r2, #7]	@ zero_extendqisi2
	ldr	r1, .L865+4
	strb	r0, [r4, #10]
	strb	r2, [r4, #9]
	mov	r2, #1
	b	.L863
.L864:
	ldrb	r5, [r3, r1]	@ zero_extendqisi2
	add	r3, r3, #1
	ldrb	r0, [r4, #10]	@ zero_extendqisi2
	orr	r0, r0, r2, asl r5
	strb	r0, [r4, #10]
.L863:
	uxtb	r0, r3
	cmp	r0, ip
	bcc	.L864
	ldmfd	sp!, {r3, r4, r5, pc}
.L866:
	.align	2
.L865:
	.word	.LANCHOR0
	.word	.LANCHOR0+3764
	.fnend
	.size	ReadFlashInfo, .-ReadFlashInfo
	.align	2
	.global	FlashDieInfoInit
	.type	FlashDieInfoInit, %function
FlashDieInfoInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L879
	mov	r2, #0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	ldr	r3, .L879+4
	strb	r2, [r1, #-916]
	ldr	r1, .L879+8
	strb	r2, [r3, #3762]
	movw	r2, #2698
	ldrh	r2, [r1, r2]
	cmp	r2, #256
	str	r2, [r3, #4]
	movhi	r2, #512
	bhi	.L878
	cmp	r2, #128
	bls	.L869
	mov	r2, #256
.L878:
	str	r2, [r3, #4]
.L869:
	mov	r1, #0
	mov	r2, #8
	ldr	r0, .L879+12
	mov	r7, #0
	bl	ftl_memset
	mov	r1, #0
	mov	r2, #32
	ldr	r0, .L879+16
	bl	ftl_memset
	ldr	r0, .L879+20
	mov	r1, #0
	mov	r2, #128
	bl	ftl_memset
	ldr	r5, .L879+24
	sub	r6, r5, #2128
	mov	r8, r5
.L871:
	ldr	r4, [r6, #2772]
	mov	r1, r8
	add	r0, r4, #1
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L870
	ldrb	r3, [r6, #3762]	@ zero_extendqisi2
	add	r2, r6, r3, asl #2
	str	r0, [r2, #2736]
	add	r2, r6, r3
	add	r3, r3, #1
	strb	r3, [r6, #3762]
	strb	r7, [r2, #3764]
.L870:
	add	r7, r7, #1
	add	r8, r8, #8
	cmp	r7, #4
	bne	.L871
	ldr	r6, .L879+4
	ldr	r3, .L879
	ldrb	r2, [r6, #3762]	@ zero_extendqisi2
	strb	r2, [r3, #-916]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #2
	bne	.L872
	add	sl, r4, #1
	mov	r7, #0
	mov	r8, r6
.L875:
	mov	r0, sl
	mov	r1, r5
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L873
	ldrb	r1, [r4, #13]	@ zero_extendqisi2
	ldr	r0, [r6, #4]
	ldrb	r3, [r6, #3762]	@ zero_extendqisi2
	mul	r0, r0, r1
	ldrh	r1, [r4, #14]
	add	r2, r6, r3, asl #2
	and	r1, r1, #65280
	mul	r1, r1, r0
	str	r1, [r2, #2736]
	ldrb	r0, [r4, #23]	@ zero_extendqisi2
	cmp	r0, #0
	movne	r1, r1, asl #1
	strne	r1, [r2, #2736]
	add	r2, r8, r3
	add	r3, r3, #1
	strb	r3, [r8, #3762]
	strb	r7, [r2, #3764]
.L873:
	add	r7, r7, #1
	add	r5, r5, #8
	cmp	r7, #4
	bne	.L875
.L872:
	ldr	r3, .L879+4
	ldrb	r1, [r4, #13]	@ zero_extendqisi2
	ldr	r2, .L879
	ldrb	r3, [r3, #3762]	@ zero_extendqisi2
	mul	r1, r1, r3
	ldrh	r3, [r4, #14]
	mul	r1, r3, r1
	ldr	r3, .L879+28
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L880:
	.align	2
.L879:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+3764
	.word	.LANCHOR0+2736
	.word	.LANCHOR0+2776
	.word	.LANCHOR0+2128
	.word	-914
	.fnend
	.size	FlashDieInfoInit, .-FlashDieInfoInit
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
	bne	.L882
	ldr	r3, .L921
.L883:
	strh	r0, [r3, #2]!	@ movhi
	add	r0, r0, #1
	uxth	r0, r0
	cmp	r0, #256
	bne	.L883
	b	.L884
.L882:
	cmp	r0, #1
	bne	.L885
	ldr	r0, .L921+4
	mov	r2, #0
	mov	r3, r2
.L888:
	uxth	r1, r3
	cmp	r1, #3
	bls	.L886
	tst	r3, #1
	moveq	r1, #2
	movne	r1, #3
	rsb	r1, r1, r2
	uxth	r1, r1
.L886:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L888
	b	.L884
.L885:
	cmp	r0, #2
	bne	.L889
	mov	r3, #0
	ldr	r0, .L921
	movw	r2, #65535
	mov	r1, r3
	b	.L920
.L892:
	cmp	r3, #1
	movls	r1, r3
	movhi	r1, r2
.L920:
	add	r3, r3, #1
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r3, r3
	uxth	r2, r2
	cmp	r3, #256
	bne	.L892
	b	.L884
.L889:
	cmp	r0, #3
	bne	.L893
	ldr	r0, .L921+4
	mov	r2, #0
	mov	r3, r2
.L896:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L894
	tst	r3, #1
	moveq	r1, #4
	movne	r1, #5
	rsb	r1, r1, r2
	uxth	r1, r1
.L894:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L896
	b	.L884
.L893:
	cmp	r0, #4
	mov	r2, #0
	bne	.L897
	ldr	r3, .L921+8
	mov	r1, #2224
	strh	r2, [r3, r1]	@ movhi
	movw	r2, #2226
	mov	r1, #1
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #2
	add	r2, r2, r1
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #3
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #5
	strh	r0, [r3, r2]	@ movhi
	add	r2, r2, #2
	ldr	r0, .L921+12
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #7
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #8
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #16
	mov	r3, r1
.L899:
	tst	r3, #1
	add	r3, r3, #1
	moveq	r1, #6
	movne	r1, #7
	cmp	r3, #256
	rsb	r1, r1, r2
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L899
	b	.L884
.L897:
	cmp	r0, #5
	bne	.L900
	ldr	r1, .L921
	mov	r3, r2
.L901:
	strh	r3, [r1, #2]!	@ movhi
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #16
	bne	.L901
	ldr	r2, .L921+16
.L902:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, #496
	bne	.L902
	b	.L884
.L900:
	cmp	r0, #6
	bne	.L884
	ldr	r0, .L921
	mov	r3, r2
.L905:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L903
	tst	r3, #1
	moveq	r1, #10
	movne	r1, #12
	rsb	r1, r1, r2
	uxth	r1, r1
.L903:
	add	r3, r3, #1
	add	r2, r2, #3
	cmp	r3, #256
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L905
.L884:
	mov	r2, #1024
	ldr	r0, .L921+20
	mov	r1, #255
	uxth	r4, r4
	bl	ftl_memset
	ldr	r2, .L921
	mov	r3, #0
	ldr	r0, .L921+24
	b	.L906
.L907:
	ldrh	r1, [r2, #2]!
	add	r3, r3, #1
	uxth	r3, r3
	add	ip, r0, r1, asl #1
	sub	ip, ip, #912
	strh	r1, [ip, #0]	@ movhi
.L906:
	cmp	r3, r4
	bcc	.L907
	ldmfd	sp!, {r4, pc}
.L922:
	.align	2
.L921:
	.word	.LANCHOR0+2222
	.word	.LANCHOR0+2224
	.word	.LANCHOR0
	.word	.LANCHOR0+2238
	.word	.LANCHOR0+2254
	.word	.LANCHOR2-912
	.word	.LANCHOR2
	.fnend
	.size	BuildFlashLsbPageTable, .-BuildFlashLsbPageTable
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
	.global	FlashSramLoadStore
	.type	FlashSramLoadStore, %function
FlashSramLoadStore:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	cmp	r2, #0
	ldr	r4, .L928
	mov	ip, r0
	mov	r2, r3
	ldr	r4, [r4, #112]
	add	r4, r4, #4096
	add	r1, r4, r1
	movne	r0, r1
	movne	r1, ip
	ldmfd	sp!, {r4, lr}
	b	memcpy
.L929:
	.align	2
.L928:
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
	.global	rk_nand_de_init
	.type	rk_nand_de_init, %function
rk_nand_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	FlashDeInit
	.fnend
	.size	rk_nand_de_init, .-rk_nand_de_init
	.align	2
	.global	rk_nand_suspend
	.type	rk_nand_suspend, %function
rk_nand_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	FlashSuspend
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
	b	FlashResume
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
	ldr	r3, .L935
	ldr	r0, [r3, #3952]
	bx	lr
.L936:
	.align	2
.L935:
	.word	.LANCHOR0
	.fnend
	.size	rk_ftl_get_capacity, .-rk_ftl_get_capacity
	.align	2
	.global	rknand_print_hex
	.type	rknand_print_hex, %function
rknand_print_hex:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r8, #0
	mov	r6, r0
	mov	r4, r1
	mov	r5, r2
	mov	sl, r3
	mov	r7, r8
	b	.L938
.L944:
	cmp	r8, #0
	bne	.L939
	ldr	r0, .L946
	mov	r1, r6
	mov	r2, r7
	bl	printk
.L939:
	cmp	r5, #4
	ldreq	r0, .L946+4
	ldreq	r1, [r4, r7, asl #2]
	beq	.L945
	cmp	r5, #2
	moveq	r3, r7, asl #1
	ldreq	r0, .L946+4
	ldreqsh	r1, [r4, r3]
	ldrne	r0, .L946+4
	ldrneb	r1, [r4, r7]	@ zero_extendqisi2
.L945:
	add	r8, r8, #1
	bl	printk
	cmp	r8, #15
	bls	.L943
	ldr	r0, .L946+8
	mov	r8, #0
	ldr	r1, .L946+12
	bl	printk
.L943:
	add	r7, r7, #1
.L938:
	cmp	r7, sl
	bne	.L944
	ldr	r0, .L946+8
	ldr	r1, .L946+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	b	printk
.L947:
	.align	2
.L946:
	.word	.LC81
	.word	.LC82
	.word	.LC74
	.word	.LC83
	.fnend
	.size	rknand_print_hex, .-rknand_print_hex
	.align	2
	.global	NandcXferComp
	.type	NandcXferComp, %function
NandcXferComp:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r0, r1, r4, r5, r6, lr}
	ldr	r6, .L971
	add	r0, r6, r0, asl #3
	ldr	r3, [r6, #3784]
	ldr	r4, [r0, #12]
	cmp	r3, #3
	bls	.L965
	ldr	r3, [r4, #16]
	tst	r3, #4
	beq	.L965
	mov	r0, r4
	bl	wait_for_nandc_xfer_completed
	ldr	r5, [r4, #16]
	ldr	r3, [r4, #8]
	ands	r5, r5, #2
	str	r3, [sp, #0]
	movne	r5, #0
	bne	.L969
	b	.L970
.L955:
	ldr	r3, [r6, #3784]
	cmp	r3, #5
	bls	.L952
	ldr	r3, [r4, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L952
	ldr	r3, [sp, #4]
	tst	r3, #131072
	bne	.L953
.L952:
	add	r5, r5, #1
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L969
	ldr	r2, [r4, #28]
	mov	r1, r5
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L971+4
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L971+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L969:
	ldr	r2, [r4, #28]
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	blt	.L955
.L953:
	ldr	r4, .L971
	ldr	r3, [r4, #3820]
	cmp	r3, #0
	beq	.L956
	ldr	r1, [sp, #0]
	mov	r2, #0
	ldr	r0, [r4, #3812]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r1, [sp, #0]
	ldr	r0, [r4, #3816]
	mov	r2, #0
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
	b	.L956
.L959:
	ldr	r3, [r4, #8]
	add	r5, r5, #1
	str	r3, [sp, #0]
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L970
	ldr	r2, [sp, #0]
	mov	r1, r5
	ldr	r3, [r4, #28]
	ldr	r0, .L971+12
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L971+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L970:
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L959
	ldr	r6, .L971
	ldr	r3, [r6, #3828]
	cmp	r3, #0
	beq	.L960
	mov	r0, r4
	bl	NandcSendDumpDataStart
.L960:
	ldr	r3, [r6, #3820]
	ldr	r5, .L971
	cmp	r3, #0
	beq	.L961
	ldr	r1, [sp, #0]
	mov	r2, #1
	ldr	r0, [r5, #3812]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r1, [sp, #0]
	ldr	r0, [r5, #3816]
	mov	r2, #1
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
.L961:
	ldr	r3, .L971
	ldr	r3, [r3, #3828]
	cmp	r3, #0
	beq	.L956
	mov	r0, r4
	bl	NandcSendDumpDataDone
.L956:
	ldr	r3, .L971
	mov	r2, #0
	str	r2, [r3, #3820]
	b	.L948
.L965:
	ldr	r3, [r4, #8]
	str	r3, [sp, #0]
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L965
.L948:
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L972:
	.align	2
.L971:
	.word	.LANCHOR0
	.word	.LC84
	.word	.LC85
	.word	.LC86
	.fnend
	.size	NandcXferComp, .-NandcXferComp
	.align	2
	.global	NandcXferData
	.type	NandcXferData, %function
NandcXferData:
	.fnstart
	@ args = 4, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r3
	mov	r9, r3
	ldr	r3, .L1011
	.pad #92
	sub	sp, sp, #92
	tst	r4, #63
	add	r3, r3, r0, asl #3
	mov	r7, r0
	mov	fp, r1
	mov	sl, r2
	ldr	r5, [sp, #128]
	ldr	r6, [r3, #12]
	bne	.L974
	cmp	r5, #0
	bne	.L975
	add	r0, sp, #24
	mov	r1, #255
	mov	r2, #64
	add	r5, sp, #24
	bl	ftl_memset
.L975:
	mov	r0, r7
	mov	r1, fp
	mov	r2, sl
	mov	r3, #0
	stmia	sp, {r4, r5}
	bl	NandcXferStart
	mov	r0, r7
	bl	NandcXferComp
	cmp	fp, #0
	movne	r4, #0
	bne	.L976
	ldr	r1, .L1011
	mov	r0, sl, lsr #1
	mov	r3, fp
	mov	r2, fp
	ldr	ip, [r1, #3832]
	mov	r4, r1
	cmp	ip, #24
	movhi	ip, #128
	movls	ip, #64
	b	.L978
.L979:
	ldr	r1, [r4, #3800]
	mov	r3, r3, lsr #2
	add	r2, r2, #1
	ldr	r3, [r1, r3, asl #2]
	mov	r1, r3, lsr #8
	strb	r3, [r5, #0]
	strb	r1, [r5, #1]
	mov	r1, r3, lsr #16
	mov	r3, r3, lsr #24
	strb	r1, [r5, #2]
	strb	r3, [r5, #3]
	add	r5, r5, #4
	mov	r3, r7
.L978:
	cmp	r2, r0
	add	r7, r3, ip
	ldr	r1, .L1011
	bcc	.L979
	mov	r3, #0
	ldr	r0, [r1, #3832]
	mov	sl, sl, lsr #2
	ldr	r1, [r1, #3784]
	mov	r4, r3
	b	.L980
.L986:
	add	r2, r3, #8
	ldr	r2, [r6, r2, asl #2]
	str	r2, [sp, #20]
	ldr	r2, [sp, #20]
	tst	r2, #4
	bne	.L1001
	ldr	r2, [sp, #20]
	ands	r2, r2, #32768
	bne	.L1001
	cmp	r1, #5
	bls	.L982
	ldr	r7, [sp, #20]
	ldr	ip, [sp, #20]
	ldr	r5, [sp, #20]
	ubfx	r7, r7, #3, #5
	ldr	r2, [sp, #20]
	ubfx	ip, ip, #27, #1
	ubfx	r5, r5, #16, #5
	ubfx	r2, r2, #29, #1
	orr	ip, r7, ip, asl #5
	orr	r2, r5, r2, asl #5
	cmp	ip, r2
	ldr	ip, [sp, #20]
	ldrhi	r2, [sp, #20]
	ubfxhi	ip, ip, #3, #5
	ldrls	r2, [sp, #20]
	ubfxls	ip, ip, #16, #5
	ubfxhi	r2, r2, #27, #1
	ubfxls	r2, r2, #29, #1
	b	.L1009
.L982:
	cmp	r1, #3
	bls	.L984
	ldr	r7, [sp, #20]
	ldr	ip, [sp, #20]
	ldr	r5, [sp, #20]
	ubfx	r7, r7, #3, #5
	ldr	r2, [sp, #20]
	ubfx	ip, ip, #28, #1
	ubfx	r5, r5, #16, #5
	ubfx	r2, r2, #30, #1
	orr	ip, r7, ip, asl #5
	orr	r2, r5, r2, asl #5
	cmp	ip, r2
	ldr	ip, [sp, #20]
	ldrhi	r2, [sp, #20]
	ubfxhi	ip, ip, #3, #5
	ldrls	r2, [sp, #20]
	ubfxls	ip, ip, #16, #5
	ubfxhi	r2, r2, #28, #1
	ubfxls	r2, r2, #30, #1
.L1009:
	orr	r2, ip, r2, asl #5
.L984:
	cmp	r4, r2
	movcc	r4, r2
	b	.L981
.L1001:
	mvn	r4, #0
.L981:
	add	r3, r3, #1
.L980:
	cmp	r3, sl
	bcs	.L976
	cmp	r0, #0
	bne	.L986
.L976:
	mov	r3, #0
	str	r3, [r6, #16]
	b	.L987
.L974:
	cmp	r1, #1
	mov	r8, #0
	bne	.L1007
	b	.L988
.L991:
	cmp	r5, #0
	and	r4, r8, #3
	mov	r0, r6
	mov	r1, #1
	movne	r3, #2
	moveq	r3, #0
	mla	r3, r8, r3, r5
	mov	r2, r4
	add	r8, r8, #2
	str	r3, [sp, #0]
	mov	r3, r9
	bl	NandcCopy1KB
	mov	r3, #0
	mov	r0, r7
	str	r3, [sp, #0]
	str	r3, [sp, #4]
	mov	r1, #1
	mov	r2, #2
	mov	r3, r4
	bl	NandcXferStart
	mov	r0, r7
	bl	NandcXferComp
	add	r9, r9, #1024
.L988:
	cmp	r8, sl
	bcc	.L991
	mov	r4, #0
	b	.L987
.L1007:
	mov	r1, r8
	mov	r2, #2
	mov	r3, r8
	str	r8, [sp, #0]
	str	r8, [sp, #4]
	mov	r4, r8
	bl	NandcXferStart
	str	fp, [sp, #12]
	mov	ip, r8
	mov	fp, r6
	mov	r6, r5
	b	.L1010
.L996:
	mov	r0, r7
	bl	NandcXferComp
	ldr	r3, [fp, #32]
	add	ip, r5, #2
	cmp	ip, sl
	str	r3, [sp, #20]
	bcs	.L993
	mov	r3, #0
	mov	r0, r7
	str	r3, [sp, #0]
	mov	r1, r3
	str	r3, [sp, #4]
	mov	r2, #2
	and	r3, ip, #3
	str	ip, [sp, #8]
	bl	NandcXferStart
	ldr	ip, [sp, #8]
.L993:
	ldr	r3, [sp, #20]
	tst	r3, #4
	mvnne	r4, #0
	bne	.L994
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r2, r2, #3, #5
	ubfx	r3, r3, #27, #1
	orr	r3, r2, r3, asl #5
	cmp	r4, r3
	movcc	r4, r3
.L994:
	cmp	r6, #0
	and	r2, r8, #3
	mov	r0, fp
	mov	r1, #0
	movne	r3, #2
	moveq	r3, #0
	mla	r5, r5, r3, r6
	mov	r3, r9
	str	ip, [sp, #8]
	add	r8, r8, #2
	add	r9, r9, #1024
	str	r5, [sp, #0]
	bl	NandcCopy1KB
	ldr	ip, [sp, #8]
.L1010:
	cmp	ip, sl
	mov	r5, ip
	bcc	.L996
	mov	r6, fp
	ldr	fp, [sp, #12]
.L987:
	ldr	r3, .L1011
	rsbs	fp, fp, #1
	movcc	fp, #0
	ldr	r3, [r3, #3784]
	cmp	r3, #5
	movls	fp, #0
	cmp	fp, #0
	beq	.L997
	ldr	r3, [r6, #0]
	and	r2, r3, #139264
	cmp	r2, #139264
	orreq	r3, r3, #131072
	streq	r3, [r6, #0]
	mvneq	r4, #0
.L997:
	mov	r0, r4
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1012:
	.align	2
.L1011:
	.word	.LANCHOR0
	.fnend
	.size	NandcXferData, .-NandcXferData
	.align	2
	.global	FlashProgPage
	.type	FlashProgPage, %function
FlashProgPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r8, r3
	ldr	r3, .L1016
	subs	r4, r0, #0
	mov	r5, r1
	mov	r6, r2
	ldrb	r7, [r3, #2697]	@ zero_extendqisi2
	bne	.L1014
	ldr	r3, .L1016+4
	ldrb	r2, [r3, #1]	@ zero_extendqisi2
	ldr	r1, [r3, #4]
	mul	r2, r1, r2
	cmp	r5, r2
	bcs	.L1014
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r7, r7, #2
	movne	r7, #4
.L1014:
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	mov	r0, r4
	mov	r1, r5
	bl	FlashProgFirstCmd
	uxtb	r2, r7
	mov	r3, r6
	mov	r0, r4
	mov	r1, #1
	str	r8, [sp, #0]
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
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1017:
	.align	2
.L1016:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashProgPage, .-FlashProgPage
	.align	2
	.global	FlashPageProgMsbFFData
	.type	FlashPageProgMsbFFData, %function
FlashPageProgMsbFFData:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r2
	ldr	r2, .L1029
	.pad #28
	sub	sp, sp, #28
	mov	r6, r0
	str	r1, [sp, #12]
	ldrb	r1, [r2, #76]	@ zero_extendqisi2
	ldr	r3, [r2, #2772]
	cmp	r1, #0
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	beq	.L1019
	ldr	r1, [r2, #3788]
	ldr	r2, .L1029+4
	cmp	r1, r2
	beq	.L1018
.L1019:
	sub	r2, r3, #5
	uxtb	r2, r2
	cmp	r3, #68
	cmpne	r2, #1
	bls	.L1022
	cmp	r3, #35
	cmpne	r3, #17
	beq	.L1022
	cmp	r3, #18
	cmpne	r3, #19
	beq	.L1022
	b	.L1018
.L1027:
	add	r3, sl, r8, asl #1
	sub	r3, r3, #912
	ldrh	r2, [r3, #0]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1018
	mov	r1, #255
	ldr	r0, [sl, #116]
	mov	r2, #32768
	bl	ftl_memset
	ldr	r1, [sp, #20]
	cmp	r1, #1
	bhi	.L1023
	ldr	r3, .L1029+8
	ldr	r2, [r3, #2964]
	cmp	r2, #0
	beq	.L1024
	ldr	r1, [sp, #16]
	mov	r0, r6
	mov	fp, #0
	ldrb	r4, [r1, #16]	@ zero_extendqisi2
	ldr	r5, [r1, #12]
	add	r4, r4, #8
	str	r2, [sp, #8]
	str	r3, [sp, #4]
	bl	NandcFlashCs
	add	r7, r5, r4, asl #8
	mov	r1, #239
	mov	r0, r6
	str	r1, [r7, #8]
	mov	r1, #146
	str	r1, [r7, #4]
	str	fp, [r5, r4, asl #8]
	str	fp, [r5, r4, asl #8]
	str	fp, [r5, r4, asl #8]
	str	fp, [r5, r4, asl #8]
	bl	NandcFlashDeCs
	ldr	r2, [sp, #8]
	cmp	r2, #1
	bne	.L1025
	mov	r0, r6
	bl	NandcFlashCs
	mov	r2, #238
	mov	r0, #2000
	str	r2, [r7, #8]
	mov	r2, #146
	str	r2, [r7, #4]
	bl	NandcDelayns
	ldr	r2, [r5, r4, asl #8]
	mov	r0, r6
	str	r2, [sp, #8]
	bl	NandcFlashDeCs
	ldr	r2, [sp, #8]
	ldr	r3, [sp, #4]
	tst	r2, #255
	moveq	r2, #2
	streq	r2, [r3, #2964]
.L1026:
	strne	fp, [r3, #2964]
	bne	.L1018
.L1025:
	ldr	r3, [sp, #12]
	mov	r0, r6
	ldr	r2, [sl, #116]
	add	r1, r8, r3
	mov	r3, #0
	bl	FlashProgPage
	mov	r0, r6
	bl	NandcFlashCs
	mov	r3, #239
	mov	r0, r6
	str	r3, [r7, #8]
	mov	r3, #146
	str	r3, [r7, #4]
	mov	r3, #1
	str	r3, [r5, r4, asl #8]
	mov	r3, #0
	str	r3, [r5, r4, asl #8]
	str	r3, [r5, r4, asl #8]
	str	r3, [r5, r4, asl #8]
	bl	NandcFlashDeCs
	b	.L1024
.L1023:
	ldr	r2, [sp, #12]
	mov	r0, r6
	mov	r3, #0
	add	r1, r8, r2
	ldr	r2, [sl, #116]
	bl	FlashProgPage
.L1024:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L1028
.L1022:
	ldr	sl, .L1029+12
	sub	r3, r3, #17
	ldr	r9, .L1029
	uxtb	r3, r3
	str	r3, [sp, #20]
	add	r3, r9, r6, asl #3
	str	r3, [sp, #16]
.L1028:
	ldr	r3, [r9, #2772]
	ldrh	r3, [r3, #10]
	cmp	r3, r8
	bhi	.L1027
.L1018:
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1030:
	.align	2
.L1029:
	.word	.LANCHOR0
	.word	1446522928
	.word	.LANCHOR1
	.word	.LANCHOR2
	.fnend
	.size	FlashPageProgMsbFFData, .-FlashPageProgMsbFFData
	.align	2
	.global	FlashReadRawPage
	.type	FlashReadRawPage, %function
FlashReadRawPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r8, r3
	ldr	r3, .L1034
	subs	r4, r0, #0
	mov	r6, r1
	mov	r5, r2
	ldrb	r7, [r3, #2697]	@ zero_extendqisi2
	bne	.L1032
	ldr	r3, .L1034+4
	ldrb	r2, [r3, #1]	@ zero_extendqisi2
	ldr	r3, [r3, #4]
	mul	r3, r3, r2
	cmp	r1, r3
	movcc	r7, #4
.L1032:
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	mov	r1, r6
	mov	r0, r4
	bl	FlashReadCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	mov	r3, r5
	mov	r1, #0
	mov	r2, r7
	mov	r0, r4
	str	r8, [sp, #0]
	bl	NandcXferData
	mov	r5, r0
	mov	r0, r4
	bl	NandcFlashDeCs
	mov	r0, r5
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1035:
	.align	2
.L1034:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashReadRawPage, .-FlashReadRawPage
	.align	2
	.global	HynixReadRetrial
	.type	HynixReadRetrial, %function
HynixReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r7, r3
	ldr	fp, .L1047
	mov	r6, r0
	str	r1, [sp, #4]
	mov	sl, r2
	add	r3, fp, r0
	mov	r8, #0
	ldrb	r9, [fp, #2906]	@ zero_extendqisi2
	mvn	r4, #0
	ldrb	r5, [r3, #2916]	@ zero_extendqisi2
	bl	NandcWaitFlashReady
	mov	ip, fp
	b	.L1037
.L1042:
	add	r5, r5, #1
	ldrb	r1, [fp, #2905]	@ zero_extendqisi2
	mov	r0, r6
	ldr	r2, .L1047+4
	uxtb	r5, r5
	str	ip, [sp, #0]
	cmp	r5, r9
	movcs	r5, #0
	mov	r3, r5
	bl	HynixSetRRPara
	mov	r0, r6
	ldr	r1, [sp, #4]
	mov	r2, sl
	mov	r3, r7
	bl	FlashReadRawPage
	ldr	ip, [sp, #0]
	cmn	r0, #1
	beq	.L1039
	ldrb	r3, [ip, #3836]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1046
	mov	r7, #0
	mov	sl, r7
.L1039:
	add	r8, r8, #1
.L1037:
	cmp	r8, r9
	bcc	.L1042
	b	.L1041
.L1046:
	mov	r4, r0
.L1041:
	ldr	r3, .L1047
	adds	r0, r4, #1
	add	r6, r3, r6
	movne	r0, #1
	ldrb	r3, [r3, #3836]	@ zero_extendqisi2
	strb	r5, [r6, #2916]
	add	r3, r3, r3, asl #1
	cmp	r4, r3, lsr #2
	movcc	r0, #0
	andcs	r0, r0, #1
	cmp	r0, #0
	moveq	r0, r4
	movne	r0, #256
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1048:
	.align	2
.L1047:
	.word	.LANCHOR0
	.word	.LANCHOR0+2908
	.fnend
	.size	HynixReadRetrial, .-HynixReadRetrial
	.align	2
	.global	MicronReadRetrial
	.type	MicronReadRetrial, %function
MicronReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L1062
	mov	sl, r3
	str	r2, [sp, #20]
	mov	r9, r0
	str	r1, [sp, #16]
	mov	r8, #0
	ldrb	r3, [r4, #3836]	@ zero_extendqisi2
	mov	fp, r8
	ldrb	r2, [r4, #76]	@ zero_extendqisi2
	add	r4, r4, r9, asl #3
	cmp	r2, #0
	addeq	r3, r3, r3, asl #1
	movne	r3, r3, lsr #2
	ubfxeq	r3, r3, #2, #8
	str	r3, [sp, #12]
	bl	NandcWaitFlashReady
	ldrb	r7, [r4, #16]	@ zero_extendqisi2
	ldr	r6, [r4, #12]
	mvn	r4, #0
	add	r7, r7, #8
	add	r5, r6, r7, asl #8
	b	.L1052
.L1056:
	mov	r3, #239
	mov	r0, #200
	str	r3, [r5, #8]
	mov	r3, #137
	str	r3, [r5, #4]
	bl	NandcDelayns
	add	ip, r8, #1
	mov	r0, r9
	str	ip, [r5, #0]
	ldr	r1, [sp, #16]
	mov	r3, sl
	str	fp, [r5, #0]
	ldr	r2, [sp, #20]
	str	fp, [r5, #0]
	str	fp, [r5, #0]
	str	ip, [sp, #8]
	bl	FlashReadRawPage
	ldr	ip, [sp, #8]
	cmn	r0, #1
	beq	.L1053
	cmn	r4, #1
	ldr	r2, [sp, #12]
	moveq	r4, r0
	cmp	r0, r2
	bcc	.L1060
	mov	sl, #0
	str	sl, [sp, #20]
.L1053:
	mov	r8, ip
.L1052:
	ldr	r2, .L1062+4
	ldrb	r3, [r2, #120]	@ zero_extendqisi2
	cmp	r8, r3
	bcc	.L1056
	b	.L1055
.L1060:
	mov	r4, r0
.L1055:
	mov	r3, #239
	mov	r0, #200
	str	r3, [r5, #8]
	mov	r3, #137
	str	r3, [r5, #4]
	bl	NandcDelayns
	ldr	r2, [sp, #12]
	mov	r3, #0
	str	r3, [r6, r7, asl #8]
	cmp	r4, r2
	str	r3, [r6, r7, asl #8]
	str	r3, [r6, r7, asl #8]
	str	r3, [r6, r7, asl #8]
	movcc	r3, #0
	movcs	r3, #1
	cmn	r4, #1
	moveq	r3, #0
	cmp	r3, #0
	movne	r4, #256
	cmn	r4, #1
	cmpne	r4, #256
	bne	.L1058
	ldr	r0, .L1062+8
	mov	r1, r8
	ldr	r2, [sp, #16]
	mov	r3, r8
	str	r4, [sp, #0]
	bl	printk
.L1058:
	mov	r0, r4
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1063:
	.align	2
.L1062:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC87
	.fnend
	.size	MicronReadRetrial, .-MicronReadRetrial
	.align	2
	.global	SamsungReadRetrial
	.type	SamsungReadRetrial, %function
SamsungReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r0
	mov	r7, r2
	mov	r6, r3
	mov	r9, r1
	bl	NandcWaitFlashReady
	ldr	ip, .L1073
	mov	r5, #1
	mvn	r4, #0
	ldr	fp, .L1073+4
	add	r3, ip, r8, asl #3
	ldrb	sl, [r3, #16]	@ zero_extendqisi2
	ldr	r2, [r3, #12]
	add	sl, sl, #8
	add	sl, r2, sl, asl #8
	b	.L1065
.L1069:
	mov	r0, sl
	uxtb	r1, r5
	str	ip, [sp, #4]
	bl	SamsungSetRRPara
	mov	r0, r8
	mov	r1, r9
	mov	r2, r7
	mov	r3, r6
	bl	FlashReadRawPage
	ldr	ip, [sp, #4]
	cmn	r0, #1
	beq	.L1066
	ldrb	r3, [ip, #3836]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1072
	mov	r6, #0
	mov	r7, r6
.L1066:
	add	r5, r5, #1
.L1065:
	ldrb	r3, [fp, #120]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r5, r3
	bcc	.L1069
	b	.L1068
.L1072:
	mov	r4, r0
.L1068:
	mov	r0, sl
	mov	r1, #0
	bl	SamsungSetRRPara
	ldr	r3, .L1073
	adds	r0, r4, #1
	ldrb	r3, [r3, #3836]	@ zero_extendqisi2
	movne	r0, #1
	add	r3, r3, r3, asl #1
	cmp	r4, r3, lsr #2
	movcc	r0, #0
	andcs	r0, r0, #1
	cmp	r0, #0
	moveq	r0, r4
	movne	r0, #256
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1074:
	.align	2
.L1073:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	SamsungReadRetrial, .-SamsungReadRetrial
	.align	2
	.global	ToshibaReadRetrial
	.type	ToshibaReadRetrial, %function
ToshibaReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r0
	str	r1, [sp, #4]
	mov	r9, r2
	mov	r7, r3
	bl	NandcWaitFlashReady
	ldr	r3, .L1096
	add	r2, r3, r8, asl #3
	ldr	r1, [r2, #12]
	ldrb	r4, [r2, #16]	@ zero_extendqisi2
	ldrb	r2, [r3, #3761]	@ zero_extendqisi2
	add	r4, r4, #8
	sub	r2, r2, #67
	add	r4, r1, r4, asl #8
	uxtb	r2, r2
	cmp	r2, #1
	movls	sl, #0
	bls	.L1076
	ldrb	sl, [r3, #3773]	@ zero_extendqisi2
	cmp	sl, #0
	beq	.L1077
	mov	r0, #0
	mov	sl, #1
	bl	NandcSetDdrMode
.L1077:
	mov	r3, #92
	str	r3, [r4, #8]
	mov	r3, #197
	str	r3, [r4, #8]
.L1076:
	mov	r6, #1
	mvn	r5, #0
	ldr	fp, .L1096
	b	.L1078
.L1087:
	ldrb	r3, [fp, #3761]	@ zero_extendqisi2
	mov	r0, r4
	uxtb	r1, r6
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1079
	bl	SandiskSetRRPara
	b	.L1080
.L1079:
	bl	ToshibaSetRRPara
.L1080:
	ldr	r2, .L1096
	ldrb	r3, [r2, #3761]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L1081
	ldr	r2, .L1096+4
	ldrb	r3, [r2, #120]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r6, r3
	moveq	r3, #179
	streq	r3, [r4, #8]
.L1081:
	cmp	sl, #0
	mov	r3, #38
	str	r3, [r4, #8]
	mov	r3, #93
	str	r3, [r4, #8]
	beq	.L1082
	mov	r0, #4
	bl	NandcSetDdrMode
	ldr	r1, [sp, #4]
	mov	r2, r9
	mov	r3, r7
	mov	r0, r8
	bl	FlashReadRawPage
	mov	r3, r0
	mov	r0, #0
	str	r3, [sp, #0]
	bl	NandcSetDdrMode
	ldr	r3, [sp, #0]
	b	.L1083
.L1082:
	mov	r3, r7
	mov	r0, r8
	ldr	r1, [sp, #4]
	mov	r2, r9
	bl	FlashReadRawPage
	mov	r3, r0
.L1083:
	cmn	r3, #1
	beq	.L1084
	ldrb	r2, [fp, #3836]	@ zero_extendqisi2
	cmn	r5, #1
	moveq	r5, r3
	add	r2, r2, r2, asl #1
	cmp	r3, r2, lsr #2
	bcc	.L1095
	mov	r7, #0
	mov	r9, r7
.L1084:
	add	r6, r6, #1
.L1078:
	ldr	r2, .L1096+4
	ldrb	r3, [r2, #120]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r6, r3
	bcc	.L1087
	b	.L1086
.L1095:
	mov	r5, r3
.L1086:
	ldr	r6, .L1096
	mov	r0, r4
	mov	r1, #0
	ldrb	r3, [r6, #3761]	@ zero_extendqisi2
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1088
	bl	SandiskSetRRPara
	b	.L1089
.L1088:
	bl	ToshibaSetRRPara
.L1089:
	mov	r3, #255
	str	r3, [r4, #8]
	ldrb	r3, [r6, #3836]	@ zero_extendqisi2
	adds	r2, r5, #1
	mov	r0, r8
	movne	r2, #1
	add	r3, r3, r3, asl #1
	cmp	r5, r3, lsr #2
	movcc	r3, #0
	andcs	r3, r2, #1
	cmp	r3, #0
	movne	r5, #256
	bl	NandcWaitFlashReady
	cmp	sl, #0
	beq	.L1091
	mov	r0, #4
	bl	NandcSetDdrMode
.L1091:
	mov	r0, r5
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1097:
	.align	2
.L1096:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	ToshibaReadRetrial, .-ToshibaReadRetrial
	.align	2
	.global	FlashSavePhyInfo
	.type	FlashSavePhyInfo, %function
FlashSavePhyInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #0
	ldr	r5, .L1105
	ldr	r7, .L1105+4
	ldr	r8, .L1105+8
	ldr	r3, [r5, #3776]
	ldrb	r0, [r7, #128]	@ zero_extendqisi2
	str	r3, [r7, #124]
	bl	FlashBchSel
	mov	r1, #0
	mov	r2, #2048
	ldr	r0, [r5, #3776]
	bl	ftl_memset
	ldr	r3, [r7, #124]
	add	r1, r5, #2128
	mov	r2, #32
	str	r8, [r3, #0]
	ldr	r4, [r7, #124]
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	add	r0, r4, #16
	strh	r3, [r4, #12]	@ movhi
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	strh	r3, [r4, #14]	@ movhi
	ldrb	r3, [r5, #3773]	@ zero_extendqisi2
	str	r3, [r4, #1076]
	bl	memcpy
	add	r0, r4, #80
	ldr	r1, .L1105+12
	mov	r2, #8
	bl	memcpy
	add	r0, r4, #96
	add	r1, r5, #2736
	mov	r2, #32
	bl	memcpy
	ldr	r0, [r7, #124]
	ldr	r1, .L1105+16
	mov	r2, #32
	add	r0, r0, #160
	bl	memcpy
	ldr	r4, [r7, #124]
	add	r1, r5, #44
	mov	r2, #32
	add	r0, r4, #192
	bl	memcpy
	mov	r2, #852
	add	r0, r4, #224
	ldr	r1, .L1105+20
	bl	memcpy
	add	r0, r4, #12
	movw	r1, #2036
	bl	JSHash
	movw	r3, #1592
	str	r3, [r4, #4]
	ldr	r3, [r7, #116]
	str	r3, [r7, #124]
	str	r0, [r4, #8]
	mov	r0, #0
	bl	flash_enter_slc_mode
	mov	r4, r6
.L1101:
	ldr	r1, [r5, #4]
	mov	r0, #0
	mov	r2, r0
	mul	r1, r1, r4
	bl	FlashEraseBlock
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r5, #3776]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r5, #3776]
	mov	r3, r0
	mul	r1, r1, r4
	add	r1, r1, #1
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r7, #116]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	beq	.L1099
	ldr	sl, [r7, #124]
	ldr	r3, [sl, #0]
	cmp	r3, r8
	bne	.L1099
	add	r0, sl, #12
	movw	r1, #2036
	bl	JSHash
	ldr	r3, [sl, #8]
	cmp	r3, r0
	bne	.L1099
	add	r3, r4, #1
	str	r3, [r7, #132]
	ldr	r3, [r5, #4]
	add	r6, r6, #1
	cmp	r6, #1
	mul	r3, r3, r4
	str	r3, [r7, #136]
	bhi	.L1100
.L1099:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L1101
.L1100:
	mov	r0, #0
	bl	flash_exit_slc_mode
	cmp	r6, #0
	mvneq	r0, #0
	movne	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1106:
	.align	2
.L1105:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	1312902724
	.word	.LANCHOR0+3764
	.word	.LANCHOR1+2688
	.word	.LANCHOR0+2904
	.fnend
	.size	FlashSavePhyInfo, .-FlashSavePhyInfo
	.align	2
	.global	FlashReadIdbDataRaw
	.type	FlashReadIdbDataRaw, %function
FlashReadIdbDataRaw:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r2, #4
	mov	sl, r0
	ldr	r1, .L1118
	add	r0, sp, #4
	bl	memcpy
	ldr	r3, .L1118+4
	ldr	r2, [r3, #3788]
	ldrb	fp, [r3, #3836]	@ zero_extendqisi2
	ldr	r3, .L1118+8
	cmp	r2, r3
	bne	.L1108
	mov	r0, #0
	bl	flash_enter_slc_mode
.L1108:
	ldr	r7, .L1118+4
	mov	r0, sl
	mov	r1, #0
	mov	r2, #2048
	bl	ftl_memset
	mvn	r8, #0
	mov	r4, #2
	mov	r6, r7
	ldr	r9, .L1118+12
	b	.L1109
.L1116:
	mov	r5, #0
.L1111:
	add	r3, sp, #4
	ldrb	r0, [r3, r5]	@ zero_extendqisi2
	bl	FlashBchSel
	ldr	r1, [r7, #4]
	mov	r0, #0
	ldr	r2, [r7, #3776]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1110
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L1111
	b	.L1112
.L1110:
	ldr	r3, [r6, #3776]
	ldr	r3, [r3, #0]
	cmp	r3, r9
	bne	.L1112
	add	r3, sp, #8
	ldr	r0, .L1118+16
	add	r5, r3, r5
	ldrb	r1, [r5, #-4]	@ zero_extendqisi2
	bl	printk
	mov	r2, #2048
	mov	r0, sl
	ldr	r1, [r6, #3776]
	bl	memcpy
	ldr	r3, [r6, #3776]
	ldr	r3, [r3, #512]
	strb	r3, [r6, #1]
	ldr	r3, .L1118+20
	ldr	r2, [r3, #132]
	cmp	r2, r4
	bls	.L1115
	str	r4, [r3, #132]
	mov	r8, #0
	bl	FlashSavePhyInfo
.L1112:
	add	r4, r4, #1
.L1109:
	ldrb	r3, [r7, #1]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1116
	b	.L1113
.L1115:
	mov	r8, #0
.L1113:
	mov	r0, fp
	bl	FlashBchSel
	ldr	r3, .L1118+4
	ldr	r2, [r3, #3788]
	ldr	r3, .L1118+8
	cmp	r2, r3
	bne	.L1114
	mov	r0, #0
	bl	flash_exit_slc_mode
.L1114:
	mov	r0, r8
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1119:
	.align	2
.L1118:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	1446522928
	.word	-52655045
	.word	.LC88
	.word	.LANCHOR2
	.fnend
	.size	FlashReadIdbDataRaw, .-FlashReadIdbDataRaw
	.align	2
	.global	FlashLoadPhyInfo
	.type	FlashLoadPhyInfo, %function
FlashLoadPhyInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r2, #4
	ldr	r1, .L1131
	add	r0, sp, #4
	bl	memcpy
	ldr	r8, .L1131+4
	ldr	r2, .L1131+8
	movw	r3, #2698
	ldr	r5, .L1131+12
	mov	r4, #0
	mov	r0, r4
	mov	sl, #4
	ldrh	fp, [r2, r3]
	mvn	r6, #0
	ldr	r3, [r8, #3776]
	str	r4, [r5, #136]
	str	r3, [r5, #124]
	bl	flash_enter_slc_mode
	b	.L1121
.L1123:
	add	r3, sp, #4
	ldrb	r0, [r3, r7]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	mov	r1, r4
	ldr	r2, [r8, #3776]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1122
	mov	r0, #0
	mov	r1, r9
	ldr	r2, [r8, #3776]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1122
	add	r7, r7, #1
	cmp	r7, #4
	beq	.L1124
	b	.L1123
.L1122:
	ldr	r7, [r5, #124]
	ldr	r3, .L1131+16
	ldr	r2, [r7, #0]
	cmp	r2, r3
	bne	.L1124
	cmp	r6, #0
	bne	.L1125
	ldr	r2, .L1131+8
	movw	r3, #2698
	mov	r0, r4
	ldrh	r1, [r2, r3]
	bl	__aeabi_uidiv
	ldr	r3, .L1131+12
	add	r0, r0, #1
	str	r0, [r3, #132]
	mov	r0, r6
	b	.L1130
.L1125:
	add	r0, r7, #12
	movw	r1, #2036
	bl	JSHash
	ldr	r3, [r7, #8]
	cmp	r3, r0
	bne	.L1124
	ldr	r6, .L1131+8
	add	r1, r7, #160
	mov	r2, #32
	add	r0, r6, #2688
	bl	memcpy
	add	r1, r7, #192
	mov	r2, #32
	ldr	r0, .L1131+20
	bl	memcpy
	add	r1, r7, #224
	mov	r2, #852
	ldr	r0, .L1131+24
	bl	memcpy
	ldr	r3, [r7, #1076]
	mov	r0, r4
	str	r4, [r5, #136]
	strb	r3, [r8, #3773]
	movw	r3, #2698
	ldrh	r1, [r6, r3]
	mov	r6, #0
	bl	__aeabi_uidiv
	add	r0, r0, #1
	str	r0, [r5, #132]
	cmp	r0, #1
	moveq	r3, #2
	streq	r3, [r5, #132]
	ldrh	r3, [r7, #14]
	strb	r3, [r5, #140]
.L1124:
	subs	sl, sl, #1
	add	r4, r4, fp
	beq	.L1128
.L1121:
	add	r9, r4, #1
	mov	r7, #0
	b	.L1123
.L1128:
	mov	r0, sl
.L1130:
	bl	flash_exit_slc_mode
	mov	r0, r6
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1132:
	.align	2
.L1131:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	1312902724
	.word	.LANCHOR0+44
	.word	.LANCHOR0+2904
	.fnend
	.size	FlashLoadPhyInfo, .-FlashLoadPhyInfo
	.align	2
	.global	FlashDdrTunningRead
	.type	FlashDdrTunningRead, %function
FlashDdrTunningRead:
	.fnstart
	@ args = 4, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r3
	ldr	r4, .L1154
	.pad #20
	sub	sp, sp, #20
	mov	fp, r0
	mov	r9, r2
	str	r1, [sp, #0]
	ldr	r3, [r4, #2160]
	ldr	r7, [r4, #3784]
	ldr	r3, [r3, #304]
	cmp	r7, #7
	movhi	r7, #12
	movls	r7, #6
	str	r3, [sp, #12]
	ldr	r3, [sp, #56]
	cmp	r3, #0
	moveq	r8, #1024
	beq	.L1135
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r0, fp
	bl	FlashReset
	ldr	r1, [sp, #0]
	mov	r2, r9
	mov	r3, sl
	mov	r0, fp
	bl	FlashReadRawPage
	mov	r8, r0
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
	cmn	r8, #1
	beq	.L1136
	ldr	r0, .L1154+4
	mov	r2, r8
	ldr	r1, [sp, #0]
	bl	printk
	cmp	r8, #9
	bhi	.L1137
	add	r4, r4, fp, asl #3
	ldr	r3, [r4, #12]
	ldr	r2, [r3, #3840]
	ldr	r2, [r3, #0]
	orr	r2, r2, #131072
	str	r2, [r3, #0]
.L1137:
	ldr	r3, .L1154+8
	ldr	r2, [r3, #144]
	add	r2, r2, #1
	str	r2, [r3, #144]
	cmp	r2, #2048
	movcs	sl, #0
	strcs	sl, [r3, #144]
	movcs	r9, sl
	bcc	.L1136
.L1135:
	mov	r4, #0
	str	fp, [sp, #4]
	mov	r5, r4
	mvn	r6, #0
	mov	fp, r4
	str	r4, [sp, #8]
.L1142:
	uxtb	r0, r7
	bl	NandcSetDdrPara
	mov	r3, sl
	ldr	r0, [sp, #4]
	mov	r2, r9
	ldr	r1, [sp, #0]
	bl	FlashReadRawPage
	add	r3, r8, #1
	cmp	r0, r3
	bhi	.L1138
	cmp	r0, #2
	bhi	.L1148
	add	r5, r5, #1
	cmp	r5, #9
	bls	.L1148
	mov	ip, fp
	rsb	r4, r5, r7
	ldr	fp, [sp, #4]
	mov	r8, r0
	mov	r6, #0
	b	.L1140
.L1138:
	cmp	fp, r5
	bcs	.L1149
	cmp	r5, #7
	rsb	r3, r5, r4
	str	r3, [sp, #8]
	bhi	.L1153
	mov	fp, r5
	b	.L1149
.L1148:
	mov	r6, #0
	mov	r4, r7
	mov	r8, r0
	mov	sl, r6
	mov	r9, r6
	b	.L1139
.L1149:
	mov	r5, #0
.L1139:
	add	r7, r7, #2
	cmp	r7, #69
	bls	.L1142
	mov	ip, fp
	ldr	fp, [sp, #4]
.L1140:
	cmp	ip, r5
	bcc	.L1143
	b	.L1141
.L1153:
	ldr	fp, [sp, #4]
.L1141:
	ldr	r4, [sp, #8]
.L1143:
	cmp	r4, #0
	beq	.L1144
	ldr	r0, .L1154+12
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	bl	NandcSetDdrPara
.L1144:
	cmn	r6, #1
	bne	.L1136
	ldr	r0, .L1154+16
	mov	r1, fp
	ldr	r2, [sp, #0]
	bl	printk
	ldr	r3, [sp, #56]
	cmp	r3, #0
	moveq	r8, r6
	beq	.L1136
	ldr	r3, [sp, #12]
	ubfx	r0, r3, #8, #8
	bl	NandcSetDdrPara
.L1136:
	mov	r0, r8
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1155:
	.align	2
.L1154:
	.word	.LANCHOR0
	.word	.LC89
	.word	.LANCHOR2
	.word	.LC90
	.word	.LC91
	.fnend
	.size	FlashDdrTunningRead, .-FlashDdrTunningRead
	.align	2
	.global	FlashDdrParaScan
	.type	FlashDdrParaScan, %function
FlashDdrParaScan:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r7, r0
	ldr	r5, .L1160
	mov	r4, #0
	mov	r6, r1
	ldrb	r0, [r5, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r5, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
	mov	r1, r6
	mov	r2, r4
	mov	r3, r4
	mov	r0, r7
	str	r4, [sp, #0]
	bl	FlashDdrTunningRead
	mov	r1, r6
	mov	r2, r4
	mov	r3, r4
	mov	r8, r0
	mov	r0, r7
	bl	FlashReadRawPage
	cmn	r0, #1
	beq	.L1157
	cmn	r8, #1
	bne	.L1158
.L1157:
	ldrb	r3, [r5, #3772]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L1158
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r3, .L1160
	mov	r2, #0
	strb	r2, [r3, #3773]
	b	.L1159
.L1158:
	mov	r3, #1
	strb	r3, [r5, #3773]
.L1159:
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1161:
	.align	2
.L1160:
	.word	.LANCHOR0
	.fnend
	.size	FlashDdrParaScan, .-FlashDdrParaScan
	.align	2
	.global	FlashInit
	.type	FlashInit, %function
FlashInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r8, r0
	mov	r0, #32768
	ldr	r6, .L1205
	bl	ftl_malloc
	ldr	r7, .L1205+4
	mov	r4, #0
	mov	r5, r6
	str	r0, [r6, #3776]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r7, #116]
	mov	r0, #4096
	bl	ftl_malloc
	str	r0, [r7, #148]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r7, #152]
	mov	r0, #4096
	bl	ftl_malloc
	mov	r3, #50
	str	r4, [r7, #132]
	strb	r3, [r6, #1]
	strb	r3, [r7, #140]
	mov	r3, #128
	strb	r4, [r6, #3773]
	str	r3, [r6, #4]
	mov	r3, #60
	str	r4, [r7, #144]
	strb	r4, [r5], #2128
	strb	r3, [r7, #128]
	str	r0, [r7, #156]
	mov	r0, r8
	bl	NandcInit
.L1165:
	uxtb	r0, r4
	mov	r1, r5
	bl	FlashReadIDRaw
	cmp	r4, #0
	bne	.L1163
	ldrb	r3, [r6, #2128]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L1199
	ldrb	r3, [r6, #2129]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L1200
.L1163:
	add	r4, r4, #1
	add	r5, r5, #8
	cmp	r4, #4
	bne	.L1165
	ldr	r4, .L1205
	ldrb	r3, [r4, #2128]	@ zero_extendqisi2
	cmp	r3, #173
	beq	.L1166
	ldr	r0, [r4, #3780]
	bl	NandcSetDdrMode
.L1166:
	mov	r1, #0
	ldr	r0, .L1205+8
	mov	r2, #852
	bl	ftl_memset
	ldr	r3, .L1205+12
	str	r3, [r4, #2772]
	mov	r3, #0
	strb	r3, [r4, #8]
	ldrb	r3, [r4, #2129]	@ zero_extendqisi2
	sub	r1, r3, #218
	rsbs	lr, r1, #0
	adc	lr, lr, r1
	cmp	r3, #218
	cmpne	r3, #241
	beq	.L1167
	cmp	r3, #220
	bne	.L1168
	ldr	r2, .L1205
	ldrb	r2, [r2, #2131]	@ zero_extendqisi2
	cmp	r2, #149
	bne	.L1168
.L1167:
	ldr	r2, .L1205
	mov	r1, #1
	ldr	r0, .L1205+4
	ldrb	ip, [r2, #2128]	@ zero_extendqisi2
	strb	r1, [r2, #0]
	mov	r1, #16
	strb	r1, [r2, #1]
	cmp	ip, #152
	strb	r1, [r0, #128]
	ldr	r1, .L1205+16
	strb	ip, [r1, #2969]
	bne	.L1169
	ldrb	r2, [r2, #2132]	@ zero_extendqisi2
	sxtb	r2, r2
	cmp	r2, #0
	movge	r2, #24
	strgeb	r2, [r0, #128]
.L1169:
	cmp	lr, #0
	beq	.L1170
	movw	r3, #2982
	mov	r2, #2048
	strh	r2, [r1, r3]	@ movhi
	mvn	r3, #37
	strb	r3, [r1, #2970]
	b	.L1171
.L1170:
	cmp	r3, #220
	bne	.L1171
	ldr	r3, .L1205+16
	movw	r2, #2982
	mov	r1, #4096
	strh	r1, [r3, r2]	@ movhi
	mvn	r2, #35
	strb	r2, [r3, #2970]
.L1171:
	ldr	r1, .L1205+20
	mov	r2, #32
	ldr	r0, .L1205+24
	bl	memcpy
	ldr	r0, .L1205+12
	mov	r2, #32
	add	r1, r0, #280
	bl	memcpy
.L1168:
	ldr	r4, .L1205
	ldrb	r3, [r4, #0]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1172
	bl	FlashLoadPhyInfoInRam
	cmp	r0, #0
	bne	.L1173
	ldr	r3, [r4, #2772]
	ldrh	r0, [r3, #16]
	ubfx	r0, r0, #8, #3
	strb	r0, [r4, #3772]
	tst	r0, #1
	bne	.L1173
	mov	r3, #1
	strb	r3, [r4, #3773]
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
.L1173:
	ldr	r4, .L1205
	ldr	r3, [r4, #2772]
	ldrb	r3, [r3, #26]	@ zero_extendqisi2
	strb	r3, [r4, #76]
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1172
	ldr	r3, [r4, #3780]
	cmp	r3, #0
	beq	.L1175
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	b	.L1203
.L1175:
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
.L1203:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1172
	ldr	r4, .L1205
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r0, .L1205+28
	ldr	r3, [r4, #2772]
	ldrh	r1, [r3, #14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	cmn	r0, #1
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, pc}
	bl	FlashDieInfoInit
	ldr	r3, [r4, #2772]
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	bl	FlashGetReadRetryDefault
	ldr	r1, .L1205+32
	ldr	r0, .L1205+4
	ldr	r3, [r4, #2772]
	ldrh	r1, [r0, r1]
	ldrb	r2, [r3, #9]	@ zero_extendqisi2
	add	r1, r1, #4080
	add	r1, r1, #15
	cmp	r2, r1, lsr #12
	blt	.L1178
	ldrh	r1, [r3, #14]
	add	r1, r1, #255
	cmp	r2, r1, lsr #8
	bge	.L1179
.L1178:
	ldrh	r2, [r3, #14]
	bic	r2, r2, #255
	strh	r2, [r3, #14]	@ movhi
.L1179:
	ldr	r3, .L1205
	ldrb	r3, [r3, #3772]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1180
	bl	FlashSavePhyInfo
	ldr	r3, .L1205+4
	mov	r0, #0
	ldr	r1, [r3, #136]
	bl	FlashDdrParaScan
.L1180:
	bl	FlashSavePhyInfo
.L1172:
	ldr	r2, .L1205
	ldr	r0, [r2, #2772]
	ldrb	r3, [r0, #26]	@ zero_extendqisi2
	strb	r3, [r2, #76]
	ldrh	r3, [r0, #16]
	ubfx	r1, r3, #7, #1
	strb	r1, [r2, #8]
	ldr	r1, .L1205+4
	tst	r3, #64
	ubfx	ip, r3, #3, #1
	strb	ip, [r1, #160]
	ubfx	ip, r3, #4, #1
	strb	ip, [r2, #3837]
	ubfx	ip, r3, #8, #3
	strb	ip, [r2, #3772]
	mov	ip, #0
	str	ip, [r1, #164]
	beq	.L1181
	ldrb	ip, [r2, #2905]	@ zero_extendqisi2
	ldrb	r3, [r0, #19]	@ zero_extendqisi2
	strb	ip, [r2, #3760]
	ldrb	ip, [r2, #2906]	@ zero_extendqisi2
	strb	r3, [r2, #3761]
	strb	ip, [r1, #120]
	sub	ip, r3, #1
	uxtb	ip, ip
	cmp	ip, #5
	bhi	.L1182
	sub	r3, r3, #5
	ldr	ip, .L1205+36
	uxtb	r3, r3
	cmp	r3, #1
	str	ip, [r1, #164]
	movls	r3, #1
	strls	r3, [r2, #3828]
	b	.L1181
.L1182:
	sub	r2, r3, #17
	uxtb	r2, r2
	cmp	r2, #2
	bhi	.L1183
	ldr	r2, .L1205+40
	cmp	r3, #19
	moveq	r3, #15
	str	r2, [r1, #164]
	mov	r2, #7
	strb	r2, [r1, #120]
	streqb	r3, [r1, #120]
	b	.L1181
.L1183:
	cmp	r3, #33
	cmpne	r3, #65
	beq	.L1184
	cmp	r3, #66
	bne	.L1185
.L1184:
	ldr	r3, .L1205+4
	mov	r1, #4
	ldr	r2, .L1205+44
	str	r2, [r3, #164]
	ldr	r2, .L1205
	strb	r1, [r2, #3760]
	mov	r2, #7
	strb	r2, [r3, #120]
	b	.L1181
.L1185:
	cmp	r3, #67
	cmpne	r3, #34
	beq	.L1186
	cmp	r3, #35
	beq	.L1186
	cmp	r3, #68
	bne	.L1187
.L1186:
	ldr	r2, .L1205+4
	cmp	r3, #35
	cmpne	r3, #68
	ldr	r1, .L1205+44
	sub	r3, r3, #67
	uxtb	r3, r3
	str	r1, [r2, #164]
	mov	r1, #7
	strb	r1, [r2, #120]
	moveq	r1, #17
	streqb	r1, [r2, #120]
	cmp	r3, #1
	ldr	r3, .L1205
	movls	r2, #4
	movhi	r2, #5
	strb	r2, [r3, #3760]
	b	.L1181
.L1187:
	cmp	r3, #49
	ldreq	r3, .L1205+48
	streq	r3, [r1, #164]
.L1181:
	ldr	r5, .L1205
	ldr	r3, .L1205+52
	ldr	r2, [r5, #3788]
	cmp	r2, r3
	bne	.L1190
	ldrb	r3, [r5, #76]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #0
	strneb	r3, [r0, #18]
.L1190:
	ldr	r3, [r5, #2772]
	ldr	r4, .L1205
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	ldrh	r0, [r3, #10]
	ldrb	r6, [r3, #18]	@ zero_extendqisi2
	bl	__aeabi_idiv
	mov	r1, r0
	mov	r0, r6
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	ldrb	r3, [r5, #2128]	@ zero_extendqisi2
	cmp	r3, #44
	bne	.L1191
	ldrb	r3, [r4, #3773]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1191
	ldr	r2, [r4, #3788]
	ldr	r3, .L1205+52
	cmp	r2, r3
	bne	.L1192
	ldrb	r3, [r4, #76]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1191
.L1192:
	ldr	r3, .L1205
	mov	r2, #0
	mov	r0, #1
	strb	r2, [r3, #3773]
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
.L1191:
	mov	r0, #0
	bl	flash_enter_slc_mode
	ldr	r2, .L1205
	ldrb	r3, [r2, #3772]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1193
	ldrb	r2, [r2, #3773]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1194
	tst	r3, #1
	bne	.L1193
.L1194:
	ldr	r3, .L1205+4
	mov	r0, #0
	ldr	r1, [r3, #136]
	bl	FlashDdrParaScan
.L1193:
	ldr	r4, .L1205
	mov	r0, #0
	bl	flash_exit_slc_mode
	ldr	r3, [r4, #2772]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	add	r0, r4, #80
	bl	FlashReadIdbDataRaw
	mov	r3, r4
	cmp	r0, #0
	movne	r3, #16
	strneb	r3, [r4, #1]
	bne	.L1197
	ldr	r2, .L1205+4
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	ldrb	r2, [r2, #140]	@ zero_extendqisi2
	cmp	r1, r2
	strhib	r2, [r3, #1]
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, #15
	ldrls	r3, .L1205
	movls	r2, #16
	strlsb	r2, [r3, #1]
.L1197:
	ldr	r4, .L1205
	movw	r6, #2210
	ldr	r0, .L1205+56
	bl	FlashTimingCfg
	movw	r3, #2204
	ldr	r5, [r4, #2772]
	ldrb	r2, [r5, #12]	@ zero_extendqisi2
	ldrh	r7, [r5, #14]
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	ldrb	r2, [r4, #3762]	@ zero_extendqisi2
	str	r3, [r4, #2200]
	ldr	r3, [r4, #2128]
	str	r3, [r4, #2196]
	movw	r3, #2206
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #2208
	ldrb	r2, [r5, #13]	@ zero_extendqisi2
	strh	r7, [r4, r6]	@ movhi
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #4
	ldrh	r2, [r5, #10]
	strh	r2, [r4, r3]	@ movhi
	ldrb	r1, [r5, #12]	@ zero_extendqisi2
	ldrh	r0, [r5, #10]
	bl	__aeabi_idiv
	movw	r3, #2214
	mov	lr, #512
	movw	r2, #2218
	strh	r0, [r4, r3]	@ movhi
	movw	r0, #2216
	ldrb	ip, [r5, #9]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	ldrh	r1, [r5, #10]
	mul	r1, r1, r3
	movw	r3, #2220
	strh	lr, [r4, r3]	@ movhi
	add	r3, r3, #2
	ldrb	lr, [r4, #1]	@ zero_extendqisi2
	strh	lr, [r4, r3]	@ movhi
	uxth	r1, r1
	ldrb	lr, [r4, #0]	@ zero_extendqisi2
	strh	r1, [r4, r2]	@ movhi
	cmp	lr, #1
	bne	.L1198
	mov	lr, #16
	mov	r7, r7, lsr #1
	mov	ip, ip, asl #1
	mov	r1, r1, asl #1
	strb	lr, [r4, #1]
	strh	r1, [r4, r2]	@ movhi
	mov	r2, #8
	strh	r7, [r4, r6]	@ movhi
	strh	ip, [r4, r0]	@ movhi
	strh	r2, [r4, r3]	@ movhi
.L1198:
	ldrb	r0, [r5, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	bl	FlashSuspend
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1199:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1200:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1206:
	.align	2
.L1205:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2904
	.word	.LANCHOR1+2688
	.word	.LANCHOR1
	.word	.LANCHOR1+2592
	.word	.LANCHOR0+44
	.word	.LC92
	.word	-914
	.word	HynixReadRetrial
	.word	MicronReadRetrial
	.word	ToshibaReadRetrial
	.word	SamsungReadRetrial
	.word	1446522928
	.word	150000
	.fnend
	.size	FlashInit, .-FlashInit
	.align	2
	.global	FlashReadPage
	.type	FlashReadPage, %function
FlashReadPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r5, r0
	mov	r6, r1
	mov	r8, r2
	mov	sl, r3
	bl	FlashReadRawPage
	cmn	r0, #1
	mov	r4, r0
	bne	.L1208
	ldr	r7, .L1213
	ldrb	r9, [r7, #8]	@ zero_extendqisi2
	cmp	r9, #0
	beq	.L1209
	mov	r3, #0
	mov	r0, r5
	strb	r3, [r7, #8]
	mov	r1, r6
	mov	r2, r8
	mov	r3, sl
	bl	FlashReadRawPage
	strb	r9, [r7, #8]
	cmn	r0, #1
	movne	r4, r0
	bne	.L1208
.L1209:
	ldr	r7, .L1213
	ldrb	r3, [r7, #3773]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1208
	ldr	r3, [r7, #2160]
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	ldr	r9, [r3, #304]
	mov	r3, #1
	str	r3, [sp, #0]
	mov	r3, sl
	bl	FlashDdrTunningRead
	cmn	r0, #1
	mov	r4, r0
	beq	.L1210
	ldrb	r3, [r7, #3836]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1208
.L1210:
	ubfx	r0, r9, #8, #8
	bl	NandcSetDdrPara
.L1208:
	ldr	r7, .L1213+4
	ldr	ip, [r7, #164]
	adds	r3, ip, #0
	movne	r3, #1
	cmn	r4, #1
	movne	r3, #0
	cmp	r3, #0
	beq	.L1211
	mov	r1, r6
	mov	r2, r8
	mov	r3, sl
	mov	r0, r5
	blx	ip
	mov	r2, r5
	mov	r3, r6
	mov	r4, r0
	ldr	r0, .L1213+8
	mov	r1, r4
	bl	printk
	cmn	r4, #1
	bne	.L1211
	ldr	r3, .L1213
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1211
	mov	r0, r5
	bl	flash_enter_slc_mode
	ldr	ip, [r7, #164]
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	mov	r3, sl
	blx	ip
	mov	r4, r0
.L1211:
	mov	r0, r4
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L1214:
	.align	2
.L1213:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC93
	.fnend
	.size	FlashReadPage, .-FlashReadPage
	.align	2
	.global	FlashReadSlc2KPages
	.type	FlashReadSlc2KPages, %function
FlashReadSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1228
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r1
	.pad #28
	sub	sp, sp, #28
	mov	fp, r2
	ldrb	sl, [r3, #2697]	@ zero_extendqisi2
	mov	r4, r0
	mov	r7, #0
	ldr	r6, .L1228+4
	b	.L1216
.L1225:
	rsb	r3, r7, r8
	add	r2, sp, #20
	mov	r0, r4
	mov	r1, fp
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #16
	bl	LogAddr2PhyAddr
	ldr	r3, [sp, #16]
	ldrb	r2, [r6, #3762]	@ zero_extendqisi2
	cmp	r3, r2
	mvncs	r3, #0
	strcs	r3, [r4, #0]
	bcs	.L1218
	add	r3, r6, r3
	ldrb	r5, [r3, #3764]	@ zero_extendqisi2
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	bl	NandcFlashCs
	ldr	r1, [sp, #20]
	mov	r0, r5
	bl	FlashReadCmd
	mov	r0, r5
	bl	NandcWaitFlashReady
	ldr	r3, [r4, #12]
	mov	r2, sl
	mov	r1, #0
	mov	r0, r5
	str	r3, [sp, #0]
	ldr	r3, [r4, #8]
	bl	NandcXferData
	ldr	r3, [sp, #20]
	ldr	r1, [r6, #4]
	add	r1, r1, r3
	mov	r9, r0
	mov	r0, r5
	bl	FlashReadCmd
	ldr	r3, [r4, #8]
	ldr	r2, [r4, #12]
	mov	r1, #0
	cmp	r3, #0
	mov	r0, r5
	addne	r3, r3, #2048
	cmp	r2, #0
	addne	r2, r2, #8
	str	r2, [sp, #0]
	mov	r2, sl
	bl	NandcXferData
	mov	r3, r0
	mov	r0, r5
	str	r3, [sp, #12]
	bl	NandcFlashDeCs
	ldr	r3, [sp, #12]
	cmp	r3, r9
	movcs	r9, r3
	ldrb	r3, [r6, #3836]	@ zero_extendqisi2
	adds	r2, r9, #1
	add	r3, r3, r3, asl #1
	movne	r2, #1
	cmp	r9, r3, lsr #2
	movls	r3, #0
	andhi	r3, r2, #1
	cmp	r3, #0
	movne	r9, #256
	cmp	r9, #256
	cmnne	r9, #1
	str	r9, [r4, #0]
	movne	r3, #0
	strne	r3, [r4, #0]
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1223
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L1223
	ldr	r3, [r3, #0]
	cmn	r3, #1
	strne	r2, [r4, #0]
.L1223:
	ldr	r3, [r4, #0]
	cmn	r3, #1
	bne	.L1218
	ldr	r1, [r4, #4]
	ldr	r0, .L1228+8
	ldrb	r2, [r6, #3836]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r4, #8]
	cmp	r1, #0
	beq	.L1224
	ldr	r0, .L1228+12
	mov	r2, #4
	mov	r3, #8
	bl	rknand_print_hex
.L1224:
	ldr	r1, [r4, #12]
	cmp	r1, #0
	beq	.L1218
	mov	r2, #4
	ldr	r0, .L1228+16
	mov	r3, r2
	bl	rknand_print_hex
.L1218:
	add	r7, r7, #1
	add	r4, r4, #36
.L1216:
	cmp	r7, r8
	bne	.L1225
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1229:
	.align	2
.L1228:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LC94
	.word	.LC95
	.word	.LC96
	.fnend
	.size	FlashReadSlc2KPages, .-FlashReadSlc2KPages
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1271
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldrb	r3, [r3, #2697]	@ zero_extendqisi2
	mov	r9, r0
	str	r1, [sp, #28]
	str	r2, [sp, #32]
	str	r3, [sp, #24]
	ldr	r3, .L1271+4
	ldrb	r8, [r3, #0]	@ zero_extendqisi2
	ldrb	ip, [r3, #8]	@ zero_extendqisi2
	cmp	r8, #0
	str	ip, [sp, #36]
	moveq	sl, r8
	beq	.L1231
	bl	FlashReadSlc2KPages
	b	.L1232
.L1258:
	mov	r3, #36
	ldr	ip, [sp, #28]
	mul	r3, r3, sl
	add	r2, sp, #44
	ldr	r1, [sp, #32]
	ldr	fp, .L1271+4
	add	r6, r9, r3
	str	r3, [sp, #16]
	rsb	r3, sl, ip
	ldr	r5, [r6, #4]
	mov	r0, r6
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #40
	bl	LogAddr2PhyAddr
	ldr	r3, [sp, #40]
	ldrb	r2, [fp, #3762]	@ zero_extendqisi2
	cmp	r3, r2
	ldrcs	ip, [sp, #16]
	mvncs	r3, #0
	mov	r7, r0
	strcs	r3, [r9, ip]
	bcs	.L1234
	add	r3, fp, r3
	ldrb	r4, [r3, #3764]	@ zero_extendqisi2
	ldr	r3, .L1271+8
	mov	r0, r4
	ldrb	r3, [r3, #160]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r7, #0
	bl	NandcWaitFlashReady
	ldr	r3, [fp, #2772]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #5
	bhi	.L1236
	add	r2, fp, r4
	ldrb	r3, [r2, #2916]	@ zero_extendqisi2
	ldrb	r2, [r2, #3756]	@ zero_extendqisi2
	cmp	r2, r3
	beq	.L1236
	mov	r0, r4
	ldrb	r1, [fp, #2905]	@ zero_extendqisi2
	ldr	r2, .L1271+12
	bl	HynixSetRRPara
.L1236:
	mov	r0, r4
	mov	r5, r5, lsr #31
	bl	NandcFlashCs
	ldr	ip, [sp, #32]
	mov	r0, r4
	cmp	ip, #1
	orreq	r5, r5, #1
	str	r5, [sp, #20]
	cmp	r5, #0
	beq	.L1237
	ldr	r3, .L1271+4
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1237
	bl	flash_enter_slc_mode
	b	.L1238
.L1237:
	bl	flash_exit_slc_mode
.L1238:
	ldr	fp, .L1271+4
.L1270:
	ldr	r1, [sp, #44]
	cmn	r1, #1
	cmpeq	r4, #255
	moveq	r3, #0
	movne	r3, #1
	moveq	r7, r3
	beq	.L1240
	cmp	r7, #0
	beq	.L1241
	ldr	r2, [fp, #4]
	mov	r0, r4
	add	r2, r1, r2
	bl	FlashReadDpCmd
	b	.L1242
.L1241:
	mov	r0, r4
	bl	FlashReadCmd
.L1242:
	mov	r0, r4
	bl	NandcWaitFlashReady
	cmp	r7, #0
	beq	.L1240
	mov	r0, r4
	ldr	r1, [sp, #44]
	bl	FlashReadDpDataOutCmd
.L1240:
	ldr	r3, [r6, #12]
	mov	r0, r4
	ldr	r2, [sp, #24]
	mov	r1, #0
	str	r3, [sp, #0]
	ldr	r3, [r6, #8]
	bl	NandcXferData
	ldrb	r2, [fp, #8]	@ zero_extendqisi2
	adds	r2, r2, #0
	movne	r2, #1
	cmn	r0, #1
	movne	r2, #0
	cmp	r2, #0
	movne	r2, #0
	movne	r7, #0
	strneb	r2, [fp, #8]
	bne	.L1270
.L1243:
	cmp	r7, #0
	mov	r5, r0
	beq	.L1244
	ldr	r3, .L1271+4
	mov	r0, r4
	ldr	r1, [r3, #4]
	ldr	r3, [sp, #44]
	str	r2, [sp, #12]
	add	r1, r1, r3
	bl	FlashReadDpDataOutCmd
	add	r3, sl, #1
	mov	r1, #36
	ldr	r2, [sp, #12]
	mla	r3, r1, r3, r9
	mov	r0, r4
	ldr	r1, [r3, #12]
	str	r1, [sp, #0]
	mov	r1, r2
	ldr	r3, [r3, #8]
	ldr	r2, [sp, #24]
	bl	NandcXferData
	cmn	r0, #1
	mov	r8, r0
	moveq	r7, #0
.L1244:
	mov	r0, r4
	ldr	fp, .L1271+4
	bl	NandcFlashDeCs
	ldr	ip, [sp, #36]
	cmn	r5, #1
	strb	ip, [fp, #8]
	bne	.L1248
	ldrb	r3, [fp, #3773]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1246
	ldr	r3, [fp, #2160]
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r7, [r3, #304]
	mov	r3, #1
	str	r3, [sp, #0]
	ldr	r2, [r6, #8]
	ldr	r3, [r6, #12]
	bl	FlashDdrTunningRead
	cmn	r0, #1
	mov	r5, r0
	beq	.L1247
	ldrb	r3, [fp, #3836]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1264
.L1247:
	ubfx	r0, r7, #8, #8
	bl	NandcSetDdrPara
	cmn	r5, #1
	bne	.L1264
.L1246:
	ldr	r3, .L1271+8
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r6, #8]
	ldr	r7, [r3, #164]
	ldr	r3, [r6, #12]
	cmp	r7, #0
	beq	.L1249
	blx	r7
	cmn	r0, #1
	mov	r5, r0
	bne	.L1266
	ldr	r3, .L1271+4
	ldr	r2, [r3, #2772]
	ldrb	r2, [r2, #19]	@ zero_extendqisi2
	sub	r2, r2, #1
	uxtb	r2, r2
	cmp	r2, #5
	bhi	.L1251
	ldrb	r1, [r3, #2905]	@ zero_extendqisi2
	mov	r0, r4
	ldr	r2, .L1271+12
	mov	r3, #0
	bl	HynixSetRRPara
.L1251:
	ldr	r1, [sp, #44]
	mov	r0, r4
	ldr	r2, [r6, #8]
	ldr	r3, [r6, #12]
	bl	FlashReadRawPage
	ldr	r7, .L1271+4
	ldr	r1, [r6, #4]
	ldrb	r2, [r7, #3836]	@ zero_extendqisi2
	mov	r5, r0
	ldr	r0, .L1271+16
	mov	r3, r5
	bl	printk
	cmn	r5, #1
	bne	.L1266
	ldrb	r7, [r7, #76]	@ zero_extendqisi2
	cmp	r7, #0
	beq	.L1250
	ldr	ip, [sp, #20]
	mov	r0, r4
	cmp	ip, #0
	beq	.L1252
	bl	flash_enter_slc_mode
	b	.L1253
.L1252:
	bl	flash_exit_slc_mode
.L1253:
	ldr	r3, .L1271+8
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r6, #8]
	ldr	ip, [r3, #164]
	ldr	r3, [r6, #12]
	blx	ip
	mov	r5, r0
	b	.L1266
.L1249:
	bl	FlashReadRawPage
	mov	r5, r0
	b	.L1250
.L1264:
	mov	r7, #0
.L1248:
	ldr	r3, .L1271+4
	ldrb	r3, [r3, #3836]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r5, r3, lsr #2
	bls	.L1250
	ldr	r3, .L1271+8
	ldr	r3, [r3, #164]
	cmp	r3, #0
	moveq	r5, #256
	b	.L1250
.L1266:
	mov	r7, #0
.L1250:
	ldr	ip, [sp, #16]
	cmp	r5, #256
	cmnne	r5, #1
	movne	r2, #0
	str	r5, [r9, ip]
	strne	r2, [r9, ip]
	ldr	ip, [sp, #16]
	ldr	r3, [r9, ip]
	cmn	r3, #1
	bne	.L1255
	ldr	r2, .L1271+4
	ldr	r1, [r6, #4]
	ldr	r0, .L1271+20
	ldrb	r2, [r2, #3836]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r6, #12]
	cmp	r1, #0
	beq	.L1255
	mov	r2, #4
	ldr	r0, .L1271+24
	mov	r3, r2
	bl	rknand_print_hex
.L1255:
	cmp	r7, #0
	beq	.L1256
	ldr	r3, .L1271+4
	ldrb	r3, [r3, #3836]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r8, r3, lsr #2
	bls	.L1257
	ldr	r3, .L1271+8
	ldr	r3, [r3, #164]
	cmp	r3, #0
	moveq	r8, #256
.L1257:
	add	r3, sl, #1
	mov	r2, #36
	cmp	r8, #256
	cmnne	r8, #1
	mul	r3, r2, r3
	movne	r2, #0
	str	r8, [r9, r3]
	strne	r2, [r9, r3]
.L1256:
	ldr	ip, [sp, #20]
	add	sl, sl, r7
	cmp	ip, #0
	beq	.L1234
	ldr	r3, .L1271+4
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1234
	mov	r0, r4
	bl	flash_exit_slc_mode
.L1234:
	add	sl, sl, #1
.L1231:
	ldr	ip, [sp, #28]
	cmp	sl, ip
	bcc	.L1258
	mov	r0, #0
.L1232:
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1272:
	.align	2
.L1271:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+2908
	.word	.LC97
	.word	.LC94
	.word	.LC96
	.fnend
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1300
	movw	r3, #3000
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldrh	r4, [r2, r3]
	movw	r3, #65535
	mov	r5, r0
	str	r1, [sp, #16]
	cmp	r4, r3
	beq	.L1293
	cmp	r4, #0
	bne	.L1274
	b	.L1275
.L1293:
	mov	r4, #0
.L1274:
	ldr	r2, .L1300+4
	movw	r3, #3912
	ldr	fp, [sp, #16]
	ldrh	r3, [r2, r3]
	cmp	fp, r3
	bne	.L1276
.L1275:
	bl	FtlGcPageVarInit
.L1276:
	ldr	r6, .L1300+8
	mov	r8, #0
	mvn	sl, #0
	mov	r9, r8
.L1288:
	ldrh	r2, [r5, #0]
	movw	r3, #65535
	strb	r9, [r5, #8]
	cmp	r2, r3
	beq	.L1294
.L1278:
	mov	r7, r5
.L1297:
	ldr	r3, .L1300+4
	movw	r2, #3844
	ldr	r0, [r6, #-1164]
	ldr	ip, [r6, #-984]
	ldrh	lr, [r3, r2]
	add	r2, r2, #80
	str	r0, [sp, #12]
	ldrh	r2, [r3, r2]
	mov	r3, #0
	mov	r5, r3
	str	r2, [sp, #20]
	mov	r2, r7
	b	.L1279
.L1281:
	ldrh	r0, [r2, #16]
	movw	r1, #65535
	cmp	r0, r1
	beq	.L1280
	mov	fp, #36
	orr	r0, r4, r0, asl #10
	mla	r1, fp, r5, ip
	ldr	fp, [sp, #20]
	stmib	r1, {r0, r9}
	mul	r0, fp, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	ldr	fp, [sp, #12]
	bic	r0, r0, #3
	add	r0, fp, r0
	str	r0, [r1, #12]
.L1280:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1279:
	cmp	r3, lr
	bne	.L1281
	mov	r1, r5
	ldr	r0, [r6, #-984]
	mov	r2, #0
	bl	FlashReadPages
	mov	r3, #36
	mul	r3, r3, r5
	mov	r5, #0
	str	r3, [sp, #12]
	b	.L1282
.L1289:
	ldr	r3, [r6, #-984]
	ldr	ip, .L1300+8
	add	r1, r3, r5
	ldr	fp, [r1, #4]
	stmia	sp, {r3, ip}
	ubfx	r0, fp, #10, #16
	str	r1, [sp, #8]
	bl	P2V_plane
	ldr	r3, [sp, #0]
	ldr	ip, [sp, #4]
	ldr	r1, [sp, #8]
	ldr	r3, [r3, r5]
	cmp	r3, #0
	mov	r2, r0
	mov	r0, ip
	bne	.L1283
	ldr	r3, [r1, #12]
	movw	r0, #65535
	add	r5, r5, #36
	ldrh	r1, [r3, #0]
	cmp	r1, r0
	moveq	r3, #1
	moveq	r5, r7
	streq	r3, [ip, #-1016]
	beq	.L1277
.L1284:
	ldr	r0, [r3, #12]
	mov	r1, fp
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
	b	.L1282
.L1283:
	ldr	r1, .L1300+8
	mov	r5, r7
	ldr	r2, [r1, #-1884]
	cmp	r2, #0
	bne	.L1285
	ldr	r2, .L1300+4
	ldrb	r2, [r2, #76]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1286
.L1285:
	ldrh	r2, [r5, #0]
	ldr	r1, [r0, #-2084]
	mov	r2, r2, asl #1
	ldrh	r2, [r1, r2]
	cmp	r2, #119
	movls	sl, fp
	bls	.L1287
.L1286:
	cmn	r3, #1
	moveq	sl, fp
.L1287:
	ldrh	r3, [r5, #0]
	mov	r4, #0
	ldr	r2, [r6, #-2064]
	mov	r3, r3, asl #1
	strh	r9, [r2, r3]	@ movhi
	ldrh	r0, [r5, #0]
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r5, #0]	@ movhi
	bl	FtlGcPageVarInit
	b	.L1288
.L1282:
	ldr	r3, [sp, #12]
	cmp	r5, r3
	bne	.L1289
	ldr	fp, [sp, #16]
	add	r8, r8, #1
	add	r4, r4, #1
	cmp	r8, fp
	uxth	r4, r4
	bcc	.L1290
	ldr	r2, .L1300
	movw	r3, #3000
	movw	r0, #65535
	ldrh	r1, [r2, r3]
	cmp	r1, r0
	beq	.L1290
	add	r1, r1, r8
	strh	r1, [r2, r3]	@ movhi
	ldr	r2, .L1300+4
	add	r3, r3, #912
	ldrh	r3, [r2, r3]
	cmp	r3, r4
	bhi	.L1291
.L1290:
	ldr	r2, .L1300+4
	movw	r3, #3912
	ldrh	r3, [r2, r3]
	cmp	r3, r4
	bhi	.L1297
	mov	r5, r7
.L1294:
	mov	r2, #0
.L1277:
	ldr	r1, .L1300
	movw	r3, #3000
	mvn	r0, #0
	strh	r4, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	strh	r0, [r1, r3]	@ movhi
	mov	r0, r5
	mov	r1, r4
	bl	ftl_sb_update_avl_pages
.L1291:
	mov	r0, sl
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1301:
	.align	2
.L1300:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r6, #0
	ldr	r5, .L1357
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L1357+4
	mov	r1, r6
	ldr	r3, .L1357+8
	movw	r7, #3932
	ldr	r2, [r5, #3940]
	ldr	r0, [r4, #-1040]
	mov	sl, r4
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #3956
	strh	r6, [r5, r3]	@ movhi
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r2, [r5, #3940]
	mov	r1, r6
	ldr	r0, [r4, #-1044]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r5, r7]
	mov	r1, r6
	ldr	r0, [r4, #-924]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r5, r7]
	mov	r1, r6
	ldr	r0, [r5, #3960]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r0, .L1357+12
	mov	r1, #255
	mov	r2, #12
	bl	ftl_memset
	movw	r3, #3852
	ldrh	r3, [r5, r3]
	str	r3, [sp, #8]
	b	.L1303
.L1305:
	add	r3, r6, r5
	ldr	r1, [sp, #8]
	ldrb	r0, [r3, #3870]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #0]
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #0]
	cmp	r0, #0
	bne	.L1304
	ldr	r2, [r4, #-984]
	mov	r3, r3, asl #10
	mla	r2, r9, r7, r2
	str	r3, [r2, #4]
	ldr	r3, [r4, #-1168]
	str	r3, [r2, #8]
	ldrh	r3, [r6, r8]
	mul	r3, r3, r7
	add	r7, r7, #1
	uxth	r7, r7
	add	r1, r3, #3
	cmp	r3, #0
	movlt	r3, r1
	ldr	r1, [r4, #-1164]
	bic	r3, r3, #3
	add	r3, r1, r3
	str	r3, [r2, #12]
.L1304:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L1336
.L1349:
	mov	r7, #0
	movw	fp, #3844
	mov	r5, r7
	mov	r9, #36
	movw	r8, #3924
.L1336:
	ldrh	r3, [r6, fp]
	cmp	r3, r5
	bhi	.L1305
	cmp	r7, #0
	beq	.L1306
	ldr	r0, [r4, #-984]
	mov	r1, r7
	mov	r2, #1
	mov	fp, r7
	bl	FlashReadPages
	mov	r8, #0
	str	r8, [sp, #12]
.L1335:
	ldr	r2, [r4, #-984]
	add	r0, r2, r8
	ldr	r7, [r2, r8]
	ldr	r3, [r0, #4]
	cmn	r7, #1
	ldr	r5, [r0, #12]
	ubfx	r6, r3, #10, #16
	bne	.L1307
	mov	r1, #1
	add	r3, r3, #1
	mov	r2, r1
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldrh	r3, [r5, #0]
	movw	r1, #65535
	cmp	r3, r1
	ldreq	r3, [r4, #-984]
	streq	r7, [r3, r8]
.L1307:
	ldr	r3, [r4, #-984]
	ldr	r3, [r3, r8]
	cmn	r3, #1
	beq	.L1308
	ldr	r2, [r4, #-1848]
	ldr	r3, [r5, #4]
	cmn	r2, #1
	beq	.L1309
	cmp	r2, r3
	bhi	.L1310
.L1309:
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [sl, #-1848]
.L1310:
	ldrh	r3, [r5, #0]
	movw	r2, #61604
	cmp	r3, r2
	beq	.L1313
	bhi	.L1316
	movw	r2, #61574
	cmp	r3, r2
	bne	.L1311
	b	.L1355
.L1316:
	movw	r2, #61634
	cmp	r3, r2
	beq	.L1314
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1311
	b	.L1356
.L1314:
	ldr	r3, .L1357
	mov	r9, r6
	ldr	r1, .L1357+8
	ldr	ip, [r4, #-1040]
	ldr	r0, [r3, #3940]
	ldrh	r1, [r4, r1]
	uxth	r2, r0
	sub	r3, r2, #1
	rsb	r2, r1, r2
	sub	r2, r2, #1
	uxth	r3, r3
	sxth	r2, r2
	str	r2, [sp, #4]
	b	.L1317
.L1323:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L1318
	ldr	ip, [ip, #0]
	mov	r6, r9
	cmp	ip, #0
	bne	.L1319
	cmp	r1, r0
	addne	r1, r1, #1
	ldrne	r0, .L1357+8
	strneh	r1, [r4, r0]	@ movhi
.L1319:
	mov	r0, #0
	sxth	r9, r3
	str	r3, [sp, #4]
	b	.L1320
.L1321:
	ldr	r7, [r4, #-1040]
	add	ip, r1, #1
	add	r0, r0, #1
	ldr	r3, [r7, ip, asl #2]
	mov	ip, ip, asl #1
	uxth	r0, r0
	str	r3, [r7, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	r7, [r4, #-1044]
	ldrh	ip, [r7, ip]
	strh	ip, [r7, r1]	@ movhi
.L1320:
	sxth	r1, r0
	cmp	r1, r9
	bne	.L1321
	ldr	r1, [sl, #-1040]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	ldr	r3, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, [sl, #-1044]
	strh	r6, [r1, r2]	@ movhi
	b	.L1322
.L1318:
	sub	r3, r3, #1
	uxth	r3, r3
.L1317:
	ldr	r6, [sp, #4]
	sxth	r2, r3
	cmp	r2, r6
	bgt	.L1323
	mov	r6, r9
.L1322:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L1311
	ldr	r2, .L1357+8
	ldr	r0, .L1357
	ldrh	r1, [r4, r2]
	ldr	r0, [r0, #3940]
	rsb	r0, r1, r0
	sub	r0, r0, #1
	sxth	r0, r0
	cmp	r3, r0
	bgt	.L1311
	add	r1, r1, #1
	strh	r1, [r4, r2]	@ movhi
	ldr	r2, [r4, #-1040]
	ldr	r1, [r5, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #-1044]
	b	.L1352
.L1355:
	ldr	r1, .L1357
	movw	r3, #3932
	movw	ip, #3956
	mov	r9, r6
	ldrh	r0, [r1, r3]
	ldrh	r1, [r1, ip]
	sub	r2, r0, #1
	ldr	ip, [r4, #-924]
	uxth	r3, r2
	rsb	r2, r1, r2
	str	r2, [sp, #4]
	b	.L1324
.L1330:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L1325
	ldr	ip, [ip, #0]
	mov	r6, r9
	cmp	ip, #0
	bne	.L1326
	cmp	r1, r0
	addne	r1, r1, #1
	ldrne	ip, .L1357
	movwne	r0, #3956
	strneh	r1, [ip, r0]	@ movhi
.L1326:
	sxth	ip, r3
	mov	r0, #0
	str	ip, [sp, #20]
	ldr	r9, .L1357
	str	r3, [sp, #4]
	b	.L1327
.L1328:
	ldr	r7, [r4, #-924]
	add	ip, r1, #1
	add	r0, r0, #1
	ldr	r3, [r7, ip, asl #2]
	mov	ip, ip, asl #1
	uxth	r0, r0
	str	r3, [r7, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	r7, [r9, #3960]
	ldrh	ip, [r7, ip]
	strh	ip, [r7, r1]	@ movhi
.L1327:
	ldr	ip, [sp, #20]
	sxth	r1, r0
	cmp	r1, ip
	bne	.L1328
	ldr	r1, [sl, #-924]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	ldr	r3, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, .L1357
	ldr	r1, [r1, #3960]
	strh	r6, [r1, r2]	@ movhi
	b	.L1329
.L1325:
	sub	r3, r3, #1
	uxth	r3, r3
.L1324:
	ldr	r6, [sp, #4]
	sxth	r2, r3
	cmp	r2, r6
	bgt	.L1330
	mov	r6, r9
.L1329:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L1311
	ldr	r2, .L1357
	movw	ip, #3932
	movw	r1, #3956
	ldrh	ip, [r2, ip]
	ldrh	r0, [r2, r1]
	sub	ip, ip, #1
	rsb	ip, r0, ip
	sxth	ip, ip
	cmp	r3, ip
	bgt	.L1311
	add	r0, r0, #1
	strh	r0, [r2, r1]	@ movhi
	ldr	r0, [r5, #4]
	ldr	r1, [r4, #-924]
	str	r0, [r1, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r2, #3960]
.L1352:
	strh	r6, [r2, r3]	@ movhi
	b	.L1311
.L1313:
	ldr	r3, .L1357+16
	movw	ip, #65535
	ldrh	r2, [r4, r3]
	cmp	r2, ip
	streqh	r6, [r4, r3]	@ movhi
	beq	.L1353
	ldr	r3, .L1357+20
	movw	r1, #65535
	ldrh	r0, [r4, r3]
	cmp	r0, r1
	beq	.L1332
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L1332:
	ldr	r3, [r5, #4]
	ldr	r2, [sl, #-1760]
	cmp	r2, r3
	ldr	r3, .L1357+20
	strcsh	r6, [r4, r3]	@ movhi
	bcs	.L1311
	ldr	r2, .L1357+16
	ldrh	r1, [r4, r2]
	strh	r6, [r4, r2]	@ movhi
	strh	r1, [r4, r3]	@ movhi
.L1353:
	ldr	r3, [r5, #4]
	str	r3, [r4, #-1760]
	b	.L1311
.L1356:
	mov	r0, r6
	b	.L1354
.L1308:
	ldr	r3, .L1357
	mov	r0, r6
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r1, #1
	beq	.L1351
.L1354:
	mov	r1, #0
.L1351:
	bl	FtlFreeSysBlkQueueIn
.L1311:
	ldr	r6, [sp, #12]
	add	r8, r8, #36
	add	r3, r6, #1
	uxth	r3, r3
	str	r3, [sp, #12]
	cmp	r3, fp
	bne	.L1335
.L1306:
	ldr	ip, [sp, #8]
	add	r3, ip, #1
	uxth	r3, r3
	str	r3, [sp, #8]
.L1303:
	ldr	r6, .L1357
	movw	r3, #3854
	ldr	ip, [sp, #8]
	ldrh	r3, [r6, r3]
	cmp	r3, ip
	bhi	.L1349
	ldr	r1, .L1357+4
	ldr	r2, [r1, #-1044]
	ldrh	r3, [r2, #0]
	cmp	r3, #0
	bne	.L1337
	ldr	r0, .L1357+8
	ldrh	r1, [r1, r0]
	cmp	r1, #0
	ldrne	r0, [r6, #3940]
	bne	.L1338
	b	.L1337
.L1342:
	mov	r1, r1, asl #1
	ldrh	r1, [r2, r1]
	cmp	r1, #0
	beq	.L1339
	ldr	r7, .L1357
	sxth	r6, r3
	ldr	r1, .L1357+4
	mov	r5, #0
	b	.L1340
.L1341:
	ldr	ip, [r1, #-1044]
	mov	r0, r2, asl #1
	rsb	r4, r6, r2
	add	r3, r3, #1
	ldrh	sl, [ip, r0]
	mov	r8, r4, asl #1
	uxth	r3, r3
	strh	sl, [ip, r8]	@ movhi
	ldr	ip, [r1, #-1040]
	ldr	r2, [ip, r2, asl #2]
	str	r2, [ip, r4, asl #2]
	ldr	r2, [r1, #-1044]
	strh	r5, [r2, r0]	@ movhi
.L1340:
	ldr	r0, [r7, #3940]
	sxth	r2, r3
	cmp	r2, r0
	bcc	.L1341
	b	.L1337
.L1339:
	add	r3, r3, #1
	uxth	r3, r3
.L1338:
	sxth	r1, r3
	cmp	r1, r0
	bcc	.L1342
.L1337:
	ldr	r2, .L1357
	ldr	r1, [r2, #3960]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L1343
	movw	r0, #3956
	ldrh	r0, [r2, r0]
	cmp	r0, #0
	movwne	r0, #3932
	ldrneh	r2, [r2, r0]
	bne	.L1344
	b	.L1343
.L1348:
	mov	ip, r0, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, #0
	beq	.L1345
	ldr	r1, .L1357
	movw	r8, #3932
	ldr	r7, .L1357+4
	mov	r6, #0
	b	.L1346
.L1347:
	ldr	r4, [r1, #3960]
	mov	ip, r2, asl #1
	rsb	r5, r0, r2
	add	r3, r3, #1
	ldrh	r9, [r4, ip]
	mov	sl, r5, asl #1
	uxth	r3, r3
	strh	r9, [r4, sl]	@ movhi
	ldr	r4, [r7, #-924]
	ldr	r2, [r4, r2, asl #2]
	str	r2, [r4, r5, asl #2]
	ldr	r2, [r1, #3960]
	strh	r6, [r2, ip]	@ movhi
.L1346:
	ldrh	ip, [r1, r8]
	sxth	r2, r3
	cmp	r2, ip
	blt	.L1347
	b	.L1343
.L1345:
	add	r3, r3, #1
	uxth	r3, r3
.L1344:
	sxth	r0, r3
	cmp	r0, r2
	blt	.L1348
.L1343:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1358:
	.align	2
.L1357:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1048
	.word	.LANCHOR2-1768
	.word	-1768
	.word	-1764
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1369
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r2
	mov	r2, #3920
	mov	r5, r1
	ldrh	r9, [r3, r2]
	movw	r2, #3906
	ldrh	r2, [r3, r2]
	add	r1, r1, r0
	.pad #52
	sub	sp, sp, #52
	mov	r6, r0
	cmp	r1, r2
	mvnhi	r7, #0
	bhi	.L1360
	mov	r9, r0, lsr r9
	mov	r7, #0
	ldr	sl, .L1369+4
	mov	r9, r9, asl #2
	b	.L1361
.L1366:
	ldr	r3, [sl, #-920]
	mov	r0, r6
	ldr	r1, .L1369
	ldr	fp, [r3, r9]
	movw	r3, #3918
	ldrh	r4, [r1, r3]
	mov	r1, r4
	bl	__aeabi_uidivmod
	uxth	r3, r1
	rsb	r4, r3, r4
	uxth	r4, r4
	cmp	r4, r5
	uxthhi	r4, r5
	cmp	fp, #0
	beq	.L1363
	ldr	r1, .L1369+4
	add	r0, sp, #12
	str	fp, [sp, #16]
	ldr	r2, [r1, #-968]
	mov	r1, #1
	str	r3, [sp, #4]
	str	r2, [sp, #20]
	mov	r2, #0
	str	r2, [sp, #24]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #4]
	mov	r0, r8
	cmn	r2, #1
	ldr	r1, [sl, #-968]
	moveq	r7, r2
	add	r1, r1, r3, asl #9
	mov	r2, r4, asl #9
	bl	memcpy
	b	.L1365
.L1363:
	mov	r0, r8
	mov	r1, fp
	mov	r2, r4, asl #9
	bl	ftl_memset
.L1365:
	rsb	r5, r4, r5
	add	r6, r6, r4
	add	r8, r8, r4, asl #9
	add	r9, r9, #4
.L1361:
	cmp	r5, #0
	bne	.L1366
.L1360:
	mov	r0, r7
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1370:
	.align	2
.L1369:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlVendorPartRead, .-FtlVendorPartRead
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
	ldr	r5, .L1375
	mov	r1, #1
	ldr	r2, .L1375+4
	bl	FtlVendorPartRead
	ldr	r4, .L1375+8
	ldr	r3, [r5, #-1704]
	cmp	r3, r4
	beq	.L1372
	ldr	r0, .L1375+4
	mov	r1, #0
	mov	r2, #512
	bl	ftl_memset
	str	r4, [r5, #-1704]
.L1372:
	ldr	r2, [r5, #-1704]
	ldr	r3, .L1375
	cmp	r2, r4
	bne	.L1373
	ldr	r2, [r3, #-1616]
	str	r2, [r3, #-1856]
	ldr	r2, [r3, #-1612]
	str	r2, [r3, #-1852]
	ldr	r2, [r3, #-1696]
	str	r2, [r3, #-1860]
	ldr	r2, [r3, #-1692]
	str	r2, [r3, #-1872]
	ldr	r2, [r3, #-1688]
	str	r2, [r3, #-1880]
	ldr	r2, [r3, #-1684]
	str	r2, [r3, #-1864]
	ldr	r2, [r3, #-1676]
	str	r2, [r3, #-1836]
	ldr	r2, [r3, #-1672]
	str	r2, [r3, #-2080]
	ldr	r2, [r3, #-1668]
	str	r2, [r3, #-1876]
	ldr	r2, [r3, #-1664]
	str	r2, [r3, #-1868]
	ldr	r2, [r3, #-1660]
	str	r2, [r3, #-1828]
	ldr	r2, [r3, #-1656]
	str	r2, [r3, #-1824]
	ldr	r2, [r3, #-1644]
	str	r2, [r3, #-1708]
.L1373:
	ldr	r4, .L1375
	mov	r3, #0
	ldr	r2, [r4, #-1636]
	str	r3, [r4, #-1004]
	ldr	r3, .L1375+12
	cmp	r2, r3
	bne	.L1374
	mov	r3, #1
	ldr	r0, .L1375+16
	str	r3, [r4, #-1884]
	ldr	r1, .L1375+20
	bl	printk
.L1374:
	ldr	r3, .L1375+24
	movw	r2, #3902
	ldr	r0, [r4, #-1840]
	ldrh	r1, [r3, r2]
	ldr	r2, [r4, #-1836]
	mla	r0, r0, r1, r2
	movw	r2, #3852
	ldrh	r1, [r3, r2]
	bl	__aeabi_uidiv
	str	r0, [r4, #-1832]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1376:
	.align	2
.L1375:
	.word	.LANCHOR2
	.word	.LANCHOR2-1704
	.word	1179929683
	.word	305432421
	.word	.LC74
	.word	.LC98
	.word	.LANCHOR0
	.fnend
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
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
	ldr	r4, .L1379
	ldr	r5, .L1379+4
	ldr	r2, [r4, #-936]
	ldrh	r1, [r4, r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #-936]
	ldr	r2, [r3, #0]
	ldr	r3, .L1379+8
	cmp	r2, r3
	beq	.L1378
	ldr	r1, .L1379+12
	ldr	r0, .L1379+16
	bl	printk
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #-936]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1378:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1380:
	.align	2
.L1379:
	.word	.LANCHOR2
	.word	-940
	.word	1112818501
	.word	.LC99
	.word	.LC74
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	load_l2p_region
	.type	load_l2p_region, %function
load_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r6, r0
	ldr	r4, .L1384
	mov	r5, #12
	ldr	r3, [r4, #-1036]
	ldr	r7, [r3, r0, asl #2]
	cmp	r7, #0
	bne	.L1382
	mul	r5, r5, r1
	ldr	r3, [r4, #-1900]
	ldr	r2, .L1384+4
	mov	r1, #255
	add	r3, r3, r5
	ldr	r0, [r3, #8]
	movw	r3, #3922
	ldrh	r2, [r2, r3]
	bl	ftl_memset
	ldr	r3, [r4, #-1900]
	strh	r6, [r3, r5]	@ movhi
	ldr	r3, [r4, #-1900]
	add	r5, r3, r5
	str	r7, [r5, #4]
	b	.L1383
.L1382:
	mul	r5, r5, r1
	ldr	r3, [r4, #-1900]
	mov	r1, #1
	add	r0, r4, #168
	mov	r2, r1
	str	r7, [r4, #172]
	add	r3, r3, r5
	ldr	r3, [r3, #8]
	str	r3, [r4, #176]
	ldr	r3, [r4, #-952]
	str	r3, [r4, #180]
	bl	FlashReadPages
	ldr	r3, [r4, #-1900]
	mov	r1, #0
	add	r2, r3, r5
	str	r1, [r2, #4]
	strh	r6, [r3, r5]	@ movhi
.L1383:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1385:
	.align	2
.L1384:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	load_l2p_region, .-load_l2p_region
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #1
	ldr	r3, .L1397
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	movweq	r2, #3914
	movwne	r2, #3912
	.pad #108
	sub	sp, sp, #108
	ldrh	r4, [r3, r2]
	add	r7, sp, #4
	mov	r5, r1
	add	r3, sp, #40
	sub	r4, r4, #1
	str	r3, [sp, #16]
	mov	sl, r0, asl #10
	mov	r1, #1
	uxth	r4, r4
	mov	r0, r7
	mov	r2, r5
	mov	r8, #0
	sxth	r3, r4
	str	r8, [sp, #12]
	orr	r3, r3, sl
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	cmn	r3, #1
	bne	.L1390
	b	.L1395
.L1393:
	add	r6, r6, r3
	mov	r0, r7
	mov	r1, #1
	mov	r2, r5
	add	r6, r6, r6, lsr #31
	ubfx	r6, r6, #1, #16
	sxth	r3, r6
	orr	r3, r3, sl
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	cmn	r3, #1
	bne	.L1391
	ldr	r3, [sp, #44]
	cmn	r3, #1
	bne	.L1391
	ldr	r3, [sp, #4]
	cmn	r3, #1
	subne	r4, r6, #1
	uxthne	r4, r4
	bne	.L1395
.L1391:
	add	r6, r6, #1
	uxth	r8, r6
.L1395:
	sxth	r6, r8
	sxth	r3, r4
	cmp	r6, r3
	ble	.L1393
.L1390:
	sxth	r0, r4
	add	sp, sp, #108
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1398:
	.align	2
.L1397:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlLoadSysInfo
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, #0
	ldr	r4, .L1414
	mov	r1, r5
	ldr	r2, .L1414+4
	ldr	r7, .L1414+8
	ldr	r3, [r4, #-952]
	str	r5, [r4, #176]
	ldr	r0, [r4, #-2064]
	str	r3, [r4, #180]
	movw	r3, #3852
	ldrh	r2, [r2, r3]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r0, [r4, r7]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1412
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	ldr	r3, .L1414+12
	add	r2, r0, #1
	mov	r6, r0
	strh	r2, [r4, r3]	@ movhi
	b	.L1401
.L1404:
	ldrh	r2, [r4, r7]
	mov	r1, #1
	ldr	r0, .L1414+16
	orr	r3, r3, r2, asl #10
	str	r3, [r4, #172]
	ldr	r3, [r4, #-2076]
	mov	r2, r1
	str	r3, [r4, #176]
	bl	FlashReadPages
	ldr	r3, [r4, #168]
	cmn	r3, #1
	beq	.L1402
	ldr	r3, [r4, #-2076]
	ldr	r2, [r3, #0]
	ldr	r3, .L1414+20
	cmp	r2, r3
	bne	.L1402
	ldr	r3, [r4, #-952]
	ldrh	r2, [r3, #0]
	movw	r3, #61604
	cmp	r2, r3
	beq	.L1403
.L1402:
	sub	r5, r5, #1
	uxth	r5, r5
.L1401:
	add	r3, r5, r6
	sxth	r3, r3
	cmp	r3, #0
	bge	.L1404
.L1403:
	ldr	r4, .L1414
	mov	r2, #48
	ldr	r5, .L1414+4
	movw	r6, #3852
	ldr	r0, .L1414+24
	ldr	r7, [r4, #176]
	mov	r1, r7
	bl	memcpy
	ldrh	r2, [r5, r6]
	add	r1, r7, #48
	ldr	r0, [r4, #-2064]
	mov	r2, r2, asl #1
	bl	memcpy
	ldrh	r1, [r5, r6]
	ldr	r3, [r4, #176]
	ldr	r0, [r4, #-1888]
	mov	r2, r1, lsr #3
	add	r1, r1, #24
	add	r2, r2, #4
	mov	r1, r1, lsr #1
	add	r1, r3, r1, asl #2
	bl	memcpy
	ldr	r2, [r4, #-1820]
	ldr	r3, .L1414+20
	cmp	r2, r3
	bne	.L1412
	ldr	r3, .L1414+28
	ldr	r2, .L1414+32
	ldrb	r1, [r4, #-1810]	@ zero_extendqisi2
	ldrh	r3, [r4, r3]
	strh	r3, [r4, r2]	@ movhi
	movw	r2, #3866
	ldrh	r2, [r5, r2]
	cmp	r1, r2
	bne	.L1412
	movw	r2, #3912
	movw	r1, #3918
	ldrh	r2, [r5, r2]
	ldrh	r1, [r5, r1]
	ldr	r0, [r5, #3856]
	mul	r2, r3, r2
	str	r3, [r4, #204]
	str	r2, [r5, #3972]
	mul	r2, r1, r2
	str	r2, [r5, #3952]
	movw	r2, #3986
	ldrh	r2, [r5, r2]
	rsb	r0, r2, r0
	rsb	r0, r3, r0
	movw	r3, #3844
	ldrh	r1, [r5, r3]
	movw	r5, #65535
	bl	__aeabi_uidiv
	ldr	r3, .L1414+36
	ldr	r1, .L1414+40
	strh	r0, [r4, r3]	@ movhi
	sub	r3, r3, #34
	ldrh	r2, [r4, r3]
	sub	r3, r3, #238
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #240
	ldrh	r3, [r4, r3]
	mov	r0, r3, lsr #6
	and	r3, r3, #63
	strb	r3, [r4, #-2038]
	ldrb	r3, [r4, #-1809]	@ zero_extendqisi2
	strh	r0, [r4, r1]	@ movhi
	mvn	r1, #0
	ldr	r0, .L1414+44
	strb	r3, [r4, #-2036]
	ldr	r3, .L1414+48
	strh	r1, [r4, r3]	@ movhi
	mov	r3, #0
	ldr	r1, .L1414+52
	strh	r3, [r4, r0]	@ movhi
	strb	r3, [r4, #-1750]
	ldrh	r0, [r4, r1]
	sub	r1, r1, #194
	strb	r3, [r4, #-1748]
	str	r3, [r4, #-1860]
	strh	r0, [r4, r1]	@ movhi
	add	r1, r1, #196
	ldr	r0, .L1414+56
	ldrh	r1, [r4, r1]
	mov	ip, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-1990]
	ldrb	r1, [r4, #-1808]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	strb	r1, [r4, #-1988]
	ldr	r1, .L1414+60
	ldrh	r0, [r4, r1]
	sub	r1, r1, #150
	strh	r0, [r4, r1]	@ movhi
	add	r1, r1, #152
	ldr	r0, .L1414+64
	ldrh	r1, [r4, r1]
	mov	ip, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-1942]
	ldrb	r1, [r4, #-1807]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	strb	r1, [r4, #-1940]
	str	r3, [r4, #-1872]
	ldr	r1, [r4, #-1788]
	str	r3, [r4, #-1880]
	str	r3, [r4, #-1864]
	str	r3, [r4, #-1836]
	str	r3, [r4, #-1828]
	str	r3, [r4, #-1868]
	ldr	r3, [r4, #-1780]
	str	r1, [r4, #-1840]
	ldr	r1, [r4, #-1848]
	cmp	r3, r1
	strhi	r3, [r4, #-1848]
	ldr	r4, .L1414
	ldr	r3, [r4, #-1784]
	ldr	r1, [r4, #-1844]
	cmp	r3, r1
	strhi	r3, [r4, #-1844]
	cmp	r2, r5
	beq	.L1407
	ldr	r0, .L1414+68
	bl	make_superblock
.L1407:
	ldr	r3, .L1414+72
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1408
	ldr	r0, .L1414+76
	bl	make_superblock
.L1408:
	ldr	r5, .L1414
	movw	r4, #65535
	ldr	r3, .L1414+80
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1409
	ldr	r0, .L1414+84
	bl	make_superblock
.L1409:
	ldr	r3, .L1414+48
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1413
	ldr	r0, .L1414+88
	bl	make_superblock
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1412:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1413:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1415:
	.align	2
.L1414:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1768
	.word	-1766
	.word	.LANCHOR2+168
	.word	1179929683
	.word	.LANCHOR2-1820
	.word	-1812
	.word	-1762
	.word	-1772
	.word	-2042
	.word	-1754
	.word	-1756
	.word	-1802
	.word	-1994
	.word	-1798
	.word	-1946
	.word	.LANCHOR2-2044
	.word	-1996
	.word	.LANCHOR2-1996
	.word	-1948
	.word	.LANCHOR2-1948
	.word	.LANCHOR2-1756
	.fnend
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlLoadBbt
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r3, #0
	ldr	r6, .L1432
	ldr	r8, .L1432+4
	ldr	r4, [r6, #-952]
	mov	r7, r6
	str	r3, [r6, #176]
	str	r4, [r6, #180]
	bl	FtlBbtMemInit
	movw	r3, #3908
	ldrh	r5, [r8, r3]
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L1417
.L1421:
	mov	r1, #1
	mov	r3, r5, asl #10
	ldr	r0, .L1432+8
	mov	r2, r1
	str	r3, [r6, #172]
	bl	FlashReadPages
	ldr	r3, [r6, #168]
	cmn	r3, #1
	bne	.L1418
	ldr	r3, [r6, #172]
	mov	r1, #1
	ldr	r0, .L1432+8
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r6, #172]
	bl	FlashReadPages
.L1418:
	ldr	r3, [r7, #168]
	cmn	r3, #1
	beq	.L1419
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1419
	ldr	r3, .L1432+4
	movw	r2, #3980
	strh	r5, [r3, r2]	@ movhi
	ldr	r2, [r4, #4]
	str	r2, [r3, #3988]
	mov	r2, #3984
	ldrh	r1, [r4, #8]
	strh	r1, [r3, r2]	@ movhi
	b	.L1420
.L1419:
	sub	r5, r5, #1
	uxth	r5, r5
.L1417:
	movw	r3, #3908
	ldrh	r3, [r8, r3]
	sub	r3, r3, #48
	cmp	r5, r3
	bgt	.L1421
.L1420:
	ldr	r5, .L1432+4
	movw	r8, #3980
	movw	r3, #65535
	ldrh	r2, [r5, r8]
	cmp	r2, r3
	beq	.L1431
	mov	r6, #3984
	ldrh	r2, [r5, r6]
	cmp	r2, r3
	beq	.L1423
	ldr	r7, .L1432
	mov	r1, #1
	mov	r2, r2, asl #10
	add	r0, r7, #168
	str	r2, [r7, #172]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r7, #168]
	cmn	r3, #1
	beq	.L1423
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1423
	ldr	r3, [r4, #4]
	ldr	r2, [r5, #3988]
	cmp	r3, r2
	strhi	r3, [r5, #3988]
	ldrhih	r2, [r5, r6]
	ldrhih	r3, [r4, #8]
	strhih	r2, [r5, r8]	@ movhi
	strhih	r3, [r5, r6]	@ movhi
.L1423:
	ldr	r8, .L1432+4
	movw	r3, #3980
	mov	r1, #1
	mov	r6, #0
	ldr	r5, .L1432
	ldrh	r0, [r8, r3]
	bl	FtlGetLastWrittenPage
	movw	r3, #3982
	add	r2, r0, #1
	mov	r7, r0
	strh	r2, [r8, r3]	@ movhi
	b	.L1424
.L1427:
	movw	r2, #3980
	mov	r1, #1
	ldrh	r2, [r8, r2]
	ldr	r0, .L1432+8
	orr	r3, r3, r2, asl #10
	str	r3, [r5, #172]
	ldr	r3, [r5, #-2076]
	mov	r2, r1
	str	r3, [r5, #176]
	bl	FlashReadPages
	ldr	r3, [r5, #168]
	cmn	r3, #1
	beq	.L1425
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	beq	.L1426
.L1425:
	sub	r6, r6, #1
	uxth	r6, r6
.L1424:
	add	r3, r6, r7
	sxth	r3, r3
	cmp	r3, #0
	bge	.L1427
.L1426:
	ldr	r3, .L1432+4
	movw	r2, #3986
	ldrh	r1, [r4, #10]
	ldrh	r0, [r4, #12]
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	beq	.L1428
	ldr	r2, [r3, #3840]
	cmp	r0, r2
	beq	.L1428
	movw	r1, #3854
	ldrh	r3, [r3, r1]
	mov	r3, r3, lsr #2
	cmp	r2, r3
	bcs	.L1428
	cmp	r0, r3
	bcs	.L1428
	bl	FtlSysBlkNumInit
.L1428:
	ldr	r6, .L1432+12
	mov	r4, #0
	ldr	r8, .L1432+4
	movw	r7, #3866
	ldr	r5, .L1432
	b	.L1429
.L1430:
	ldr	r3, .L1432+16
	ldr	r1, [r5, #176]
	ldr	r0, [r6, #4]!
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #2
	mla	r1, r4, r2, r1
	bl	memcpy
	add	r4, r4, #1
.L1429:
	ldrh	r3, [r8, r7]
	cmp	r4, r3
	bcc	.L1430
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1431:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1433:
	.align	2
.L1432:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+168
	.word	.LANCHOR0+4004
	.word	-1032
	.fnend
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, #0
	ldr	r3, .L1441
	movw	fp, #3866
	ldr	r7, .L1441+4
	ldr	r6, .L1441+8
	ldr	r2, [r3, #-2076]
	ldr	r8, [r3, #-952]
	str	r2, [r3, #176]
	str	r8, [r3, #180]
	b	.L1435
.L1440:
	movw	r9, #3908
	mvn	r3, #0
	ldrh	r4, [r6, r9]
	strh	r3, [r7], #2	@ movhi
	add	r4, r4, r3
	ldr	sl, .L1441
	uxth	r4, r4
	b	.L1436
.L1439:
	mla	r3, r3, r5, r4
	mov	r1, #1
	ldr	r0, .L1441+12
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sl, #172]
	bl	FlashReadPages
	ldr	r3, [sl, #168]
	cmn	r3, #1
	beq	.L1437
	ldrh	r2, [r8, #0]
	movw	r3, #61664
	cmp	r2, r3
	streqh	r4, [r7, #-2]	@ movhi
	beq	.L1438
.L1437:
	sub	r4, r4, #1
	uxth	r4, r4
.L1436:
	ldrh	r3, [r6, r9]
	sub	r2, r3, #16
	cmp	r4, r2
	bgt	.L1439
.L1438:
	add	r5, r5, #1
.L1435:
	ldrh	r3, [r6, fp]
	cmp	r5, r3
	bcc	.L1440
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1442:
	.align	2
.L1441:
	.word	.LANCHOR2
	.word	.LANCHOR0+3992
	.word	.LANCHOR0
	.word	.LANCHOR2+168
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FlashProgSlc2KPages
	.type	FlashProgSlc2KPages, %function
FlashProgSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #68
	sub	sp, sp, #68
	mov	sl, r1
	mov	r9, r2
	str	r3, [sp, #12]
	mov	r4, r0
	ldr	r3, .L1462
	mov	r6, r0
	mov	r8, #0
	ldr	r7, .L1462+4
	ldrb	fp, [r3, #2697]	@ zero_extendqisi2
	b	.L1444
.L1451:
	rsb	r3, r8, sl
	add	r2, sp, #20
	mov	r0, r6
	mov	r1, r9
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #24
	bl	LogAddr2PhyAddr
	ldr	r3, [sp, #24]
	ldrb	r2, [r7, #3762]	@ zero_extendqisi2
	cmp	r3, r2
	mvncs	r3, #0
	strcs	r3, [r6, #0]
	bcs	.L1446
	add	r3, r7, r3
	ldrb	r5, [r3, #3764]	@ zero_extendqisi2
	mov	r0, r5
	bl	NandcWaitFlashReady
	mov	r0, r5
	bl	NandcFlashCs
	mov	r0, r5
	ldr	r1, [sp, #20]
	bl	FlashProgFirstCmd
	ldr	r3, [r6, #12]
	mov	r2, fp
	mov	r1, #1
	mov	r0, r5
	str	r3, [sp, #0]
	ldr	r3, [r6, #8]
	bl	NandcXferData
	ldr	r1, [sp, #20]
	mov	r0, r5
	bl	FlashProgSecondCmd
	mov	r0, r5
	bl	NandcWaitFlashReady
	ldr	r1, [sp, #20]
	mov	r0, r5
	bl	FlashReadStatus
	ldr	r3, [sp, #20]
	ands	r0, r0, #1
	mvnne	r0, #0
	str	r0, [r6, #0]
	ldr	r1, [r7, #4]
	mov	r0, r5
	add	r1, r1, r3
	bl	FlashProgFirstCmd
	ldr	r3, [r6, #8]
	ldr	r2, [r6, #12]
	mov	r1, #1
	cmp	r3, #0
	mov	r0, r5
	addne	r3, r3, #2048
	cmp	r2, #0
	addne	r2, r2, #8
	str	r2, [sp, #0]
	mov	r2, fp
	bl	NandcXferData
	ldr	r3, [sp, #20]
	ldr	r1, [r7, #4]
	mov	r0, r5
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
	strne	r3, [r6, #0]
	bl	NandcFlashDeCs
.L1446:
	add	r8, r8, #1
	add	r6, r6, #36
.L1444:
	cmp	r8, sl
	bne	.L1451
	ldr	r3, [sp, #12]
	cmp	r3, #0
	movne	r6, #0
	ldrne	r5, .L1462+8
	bne	.L1452
	b	.L1453
.L1458:
	ldr	r3, [r4, #0]
	cmn	r3, #1
	bne	.L1454
	ldr	r1, [r4, #4]
	ldr	r0, .L1462+12
	bl	printk
	b	.L1455
.L1454:
	rsb	r3, r6, sl
	mov	r1, r9
	add	r2, sp, #20
	mov	r0, r4
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #24
	bl	LogAddr2PhyAddr
	ldr	r2, [r5, #152]
	mov	r3, #0
	mov	lr, r4
	add	ip, sp, #28
	str	r3, [r2, #0]
	ldr	r2, [r5, #156]
	str	r3, [r2, #0]
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	add	r0, sp, #28
	ldr	r3, [lr, #0]
	mov	r1, #1
	mov	r2, r9
	str	r3, [ip, #0]
	ldr	r3, [r5, #152]
	str	r3, [sp, #36]
	ldr	r3, [r5, #156]
	str	r3, [sp, #40]
	bl	FlashReadPages
	ldr	r7, [sp, #28]
	cmn	r7, #1
	bne	.L1456
	ldr	r0, .L1462+16
	ldr	r1, [r4, #4]
	bl	printk
	str	r7, [r4, #0]
.L1456:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1457
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #156]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1457
	ldr	r0, .L1462+20
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L1457:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L1455
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #152]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1455
	ldr	r0, .L1462+24
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L1455:
	add	r6, r6, #1
	add	r4, r4, #36
.L1452:
	cmp	r6, sl
	bne	.L1458
.L1453:
	mov	r0, #0
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1463:
	.align	2
.L1462:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC100
	.word	.LC101
	.word	.LC102
	.word	.LC103
	.fnend
	.size	FlashProgSlc2KPages, .-FlashProgSlc2KPages
	.align	2
	.global	FlashLoadFactorBbt
	.type	FlashLoadFactorBbt, %function
FlashLoadFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r2, #2210
	ldr	r9, .L1473
	mov	r3, #2208
	ldr	r4, .L1473+4
	.pad #52
	sub	sp, sp, #52
	mov	r1, #0
	mvn	r7, #0
	ldrh	r3, [r9, r3]
	add	r0, r4, #208
	ldrh	r8, [r9, r2]
	mov	r2, #16
	mov	r6, #0
	mul	r8, r8, r3
	bl	ftl_memset
	ldr	r3, [r4, #148]
	str	r6, [sp, #20]
	mov	r4, r6
	str	r3, [sp, #24]
	uxth	r8, r8
	add	r3, r8, r7
	uxth	r3, r3
	str	r3, [sp, #4]
	b	.L1465
.L1471:
	ldr	r5, [sp, #4]
	sub	r3, r8, #12
	mul	fp, r8, r4
	ldr	sl, .L1473+4
	b	.L1466
.L1469:
	add	r2, fp, r5
	mov	r1, #1
	add	r0, sp, #12
	str	r3, [sp, #0]
	mov	r2, r2, asl #10
	str	r2, [sp, #16]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #0]
	cmn	r2, #1
	beq	.L1467
	ldr	r2, [sl, #148]
	ldrh	r1, [r2, #0]
	movw	r2, #61664
	cmp	r1, r2
	bne	.L1467
	ldr	r0, .L1473+8
	mov	r1, r4
	mov	r2, r5
	add	r6, r6, #1
	bl	printk
	ldr	r3, .L1473+4
	uxth	r6, r6
	add	r3, r3, r4, asl #1
	strh	r5, [r3, #208]	@ movhi
	b	.L1468
.L1467:
	sub	r5, r5, #1
	uxth	r5, r5
.L1466:
	cmp	r5, r3
	bgt	.L1469
.L1468:
	ldrb	r3, [r9, #3762]	@ zero_extendqisi2
	add	r4, r4, #1
	cmp	r3, r6
	uxtb	r4, r4
	moveq	r7, #0
.L1465:
	ldrb	r3, [r9, #3762]	@ zero_extendqisi2
	cmp	r3, r4
	bhi	.L1471
	mov	r0, r7
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1474:
	.align	2
.L1473:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC104
	.fnend
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.align	2
	.global	FlashReadFacBbtData
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1485
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r6, r1
	mov	r9, r2
	movw	r1, #2210
	mov	r2, #2208
	ldrh	r8, [r3, r1]
	ldrh	r2, [r3, r2]
	.pad #40
	sub	sp, sp, #40
	ldr	r3, [r3, #3776]
	mov	r4, r0
	ldr	r7, .L1485+4
	mul	r8, r8, r2
	str	r3, [sp, #12]
	ldr	r3, [r7, #148]
	str	r3, [sp, #16]
	uxth	r8, r8
	sub	r5, r8, #1
	mul	sl, r8, r6
	uxth	r5, r5
	sub	r8, r8, #16
	b	.L1476
.L1482:
	mov	r1, #1
	add	r3, r5, sl
	add	r0, sp, #4
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L1477
	ldr	r3, [r7, #148]
	ldrh	r2, [r3, #0]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L1477
	cmp	r4, #0
	moveq	r0, r4
	beq	.L1478
	cmp	r6, #0
	beq	.L1484
	b	.L1480
.L1481:
	ldr	r3, [r0, #3776]
	ubfx	r2, r6, #5, #16
	and	r5, r6, #31
	add	r6, r6, #1
	ldr	lr, [r3, r2, asl #2]
	uxth	r6, r6
	orr	lr, lr, r1, asl r5
	str	lr, [r3, r2, asl #2]
	b	.L1479
.L1484:
	ldr	ip, .L1485+4
	mov	r1, #1
	ldr	r0, .L1485
.L1479:
	ldr	r3, [ip, #132]
	cmp	r6, r3
	bcc	.L1481
.L1480:
	ldr	r3, .L1485
	mov	r2, r9
	mov	r0, r4
	ldr	r1, [r3, #3776]
	bl	memcpy
	mov	r2, #4
	ldr	r0, .L1485+8
	mov	r1, r4
	mov	r3, r2
	bl	rknand_print_hex
	mov	r0, #0
	b	.L1478
.L1477:
	sub	r5, r5, #1
	uxth	r5, r5
.L1476:
	cmp	r5, r8
	bgt	.L1482
	mvn	r0, #0
.L1478:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1486:
	.align	2
.L1485:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC105
	.fnend
	.size	FlashReadFacBbtData, .-FlashReadFacBbtData
	.align	2
	.global	FlashGetBadBlockList
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1494
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r3, [r3, #2772]
	ldr	r6, .L1494+4
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	ldrh	r4, [r3, #14]
	ldr	r0, [r6, #116]
	mul	r4, r4, r2
	uxth	r4, r4
	add	r2, r4, #7
	mov	r2, r2, lsr #3
	bl	FlashReadFacBbtData
	cmn	r0, #1
	beq	.L1493
	mov	r2, #0
	mov	r0, r4, lsr #4
	mov	r3, r2
	sub	r4, r4, #1
	mov	r1, #1
	b	.L1489
.L1491:
	ldr	ip, [r6, #116]
	mov	r8, r2, lsr #5
	and	r7, r2, #31
	ldr	ip, [ip, r8, asl #2]
	ands	ip, ip, r1, asl r7
	movne	ip, r3, asl #1
	addne	r3, r3, #1
	uxthne	r3, r3
	strneh	r2, [r5, ip]	@ movhi
	cmp	r3, r0
	bcs	.L1493
	add	r2, r2, #1
	uxth	r2, r2
.L1489:
	cmp	r2, r4
	blt	.L1491
	b	.L1488
.L1493:
	mov	r3, #0
.L1488:
	mov	r3, r3, asl #1
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r5, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1495:
	.align	2
.L1494:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	2
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #76
	sub	sp, sp, #76
	ldr	r5, .L1524
	mov	r9, r0
	str	r1, [sp, #8]
	mov	sl, r2
	str	r3, [sp, #16]
	ldr	ip, [r5, #2772]
	ldrb	r6, [r5, #0]	@ zero_extendqisi2
	ldrb	ip, [ip, #19]	@ zero_extendqisi2
	cmp	r6, #0
	moveq	fp, r5
	str	ip, [sp, #20]
	ldr	ip, .L1524+4
	ldrb	ip, [ip, #2697]	@ zero_extendqisi2
	str	ip, [sp, #12]
	beq	.L1497
	bl	FlashProgSlc2KPages
	b	.L1498
.L1511:
	mov	r4, #36
	ldr	r2, [sp, #8]
	mul	r4, r4, r6
	rsb	r3, r6, r2
	mov	r1, sl
	add	r2, sp, #28
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r7, r9, r4
	add	r3, sp, #32
	mov	r0, r7
	bl	LogAddr2PhyAddr
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	ldr	r2, [sp, #32]
	cmp	r2, r3
	mvncs	r3, #0
	strcs	r3, [r9, r4]
	mov	r8, r0
	bcs	.L1500
	ldrb	r1, [r5, #3837]	@ zero_extendqisi2
	add	r2, fp, r2, asl #4
	cmp	r1, #0
	ldr	r2, [r2, #2784]
	moveq	r8, #0
	cmp	r2, #0
	beq	.L1502
	cmp	r3, #1
	bne	.L1503
	ldr	r0, [r5, #2160]
	bl	NandcIqrWaitFlashReady
.L1503:
	ldrb	r0, [sp, #32]	@ zero_extendqisi2
	bl	FlashWaitCmdDone
.L1502:
	ldr	r2, [sp, #32]
	mov	r1, #0
	cmp	r8, #0
	add	r3, r5, r2, asl #4
	movne	r0, #36
	str	r1, [r3, #2788]
	ldr	r1, [sp, #28]
	str	r7, [r3, #2784]
	str	r1, [r3, #2780]
	addne	r1, r6, #1
	mlane	r1, r0, r1, r9
	strne	r1, [r3, #2788]
	add	r3, fp, r2
	add	r2, fp, r2, asl #4
	ldrb	r4, [r3, #3764]	@ zero_extendqisi2
	ldrb	r3, [fp, #3762]	@ zero_extendqisi2
	cmp	r3, #1
	strb	r4, [r2, #2776]
	mov	r0, r4
	bne	.L1505
	bl	NandcWaitFlashReady
	b	.L1506
.L1505:
	bl	NandcFlashCs
	ldr	r3, [sp, #32]
	mov	r0, r4
	ldr	r1, [sp, #28]
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #2736]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r4
	bl	NandcFlashDeCs
.L1506:
	ldr	r2, [sp, #20]
	sub	r3, r2, #1
	cmp	r3, #5
	bhi	.L1507
	add	r3, r5, r4
	ldrb	r3, [r3, #3756]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1507
	mov	r0, r4
	ldrb	r1, [r5, #2905]	@ zero_extendqisi2
	ldr	r2, .L1524+8
	mov	r3, #0
	bl	HynixSetRRPara
.L1507:
	mov	r0, r4
	bl	NandcFlashCs
	cmp	sl, #1
	mov	r0, r4
	bne	.L1508
	ldrb	r3, [r5, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1508
	bl	flash_enter_slc_mode
	b	.L1509
.L1508:
	bl	flash_exit_slc_mode
.L1509:
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgFirstCmd
	ldr	r3, [r7, #12]
	mov	r0, r4
	mov	r1, #1
	ldr	r2, [sp, #12]
	str	r3, [sp, #0]
	ldr	r3, [r7, #8]
	bl	NandcXferData
	cmp	r8, #0
	beq	.L1510
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgDpFirstCmd
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	mov	r0, r4
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #2736]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	ldr	r3, [sp, #28]
	ldr	r1, [r5, #4]
	mov	r0, r4
	add	r1, r1, r3
	bl	FlashProgDpSecondCmd
	add	r3, r6, #1
	mov	r2, #36
	mov	r0, r4
	mla	r3, r2, r3, r9
	mov	r1, #1
	ldr	r2, [r3, #12]
	str	r2, [sp, #0]
	ldr	r2, [sp, #12]
	ldr	r3, [r3, #8]
	bl	NandcXferData
.L1510:
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgSecondCmd
	mov	r0, r4
	bl	NandcFlashDeCs
	add	r6, r6, r8
.L1500:
	add	r6, r6, #1
.L1497:
	ldr	r3, [sp, #8]
	cmp	r6, r3
	bcc	.L1511
	ldr	r5, .L1524
	mov	r4, #0
	ldr	r6, .L1524+12
	ldr	r0, [r5, #2160]
	bl	NandcIqrWaitFlashReady
	b	.L1512
.L1514:
	uxtb	r0, r4
	bl	FlashWaitCmdDone
	cmp	sl, #1
	bne	.L1513
	ldrb	r3, [r5, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1513
	ldrb	r0, [r6, r4, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L1513:
	add	r4, r4, #1
.L1512:
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1514
	ldr	r2, [sp, #16]
	cmp	r2, #0
	ldreq	r0, [sp, #16]
	beq	.L1498
	mov	r5, #0
	ldr	r4, .L1524+16
	ldr	r6, [sp, #8]
	b	.L1515
.L1520:
	ldr	r3, [r9, #0]
	cmn	r3, #1
	bne	.L1516
	ldr	r1, [r9, #4]
	ldr	r0, .L1524+20
	bl	printk
	b	.L1517
.L1516:
	rsb	r3, r5, r6
	mov	r1, sl
	add	r2, sp, #28
	mov	r0, r9
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldr	r2, [r4, #152]
	mov	r3, #0
	mov	lr, r9
	add	ip, sp, #36
	str	r3, [r2, #0]
	ldr	r2, [r4, #156]
	str	r3, [r2, #0]
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	add	r0, sp, #36
	ldr	r3, [lr, #0]
	mov	r1, #1
	mov	r2, sl
	str	r3, [ip, #0]
	ldr	r3, [r4, #152]
	str	r3, [sp, #44]
	ldr	r3, [r4, #156]
	str	r3, [sp, #48]
	bl	FlashReadPages
	ldr	r7, [sp, #36]
	cmn	r7, #1
	bne	.L1518
	ldr	r0, .L1524+24
	ldr	r1, [r9, #4]
	bl	printk
	str	r7, [r9, #0]
.L1518:
	ldr	r3, [r9, #12]
	cmp	r3, #0
	beq	.L1519
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #156]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1519
	ldr	r1, [r9, #4]
	ldr	r0, .L1524+28
	bl	printk
	ldr	r0, .L1524+32
	ldr	r1, [r9, #12]
	mov	r2, #4
	mov	r3, #16
	bl	rknand_print_hex
	mov	r3, #16
	ldr	r0, .L1524+36
	mov	r2, #4
	ldr	r1, [r4, #156]
	bl	rknand_print_hex
	mvn	r3, #0
	str	r3, [r9, #0]
.L1519:
	ldr	r3, [r9, #8]
	cmp	r3, #0
	beq	.L1517
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #152]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1517
	ldr	r1, [r9, #4]
	ldr	r0, .L1524+40
	bl	printk
	mvn	r3, #0
	ldr	r0, .L1524+44
	mov	r2, #4
	str	r3, [r9, #0]
	mov	r3, #16
	ldr	r1, [r9, #8]
	bl	rknand_print_hex
	ldr	r0, .L1524+48
	ldr	r1, [r4, #152]
	mov	r2, #4
	mov	r3, #16
	bl	rknand_print_hex
.L1517:
	add	r5, r5, #1
	add	r9, r9, #36
.L1515:
	cmp	r5, r6
	bne	.L1520
	mov	r0, #0
.L1498:
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1525:
	.align	2
.L1524:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+2908
	.word	.LANCHOR0+2776
	.word	.LANCHOR2
	.word	.LC100
	.word	.LC101
	.word	.LC102
	.word	.LC106
	.word	.LC107
	.word	.LC103
	.word	.LC108
	.word	.LC109
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #0
	ldr	r4, .L1537
	mov	r1, #255
	ldr	r8, .L1537+4
	movw	sl, #3852
	ldr	r7, .L1537+8
	ldr	r3, [r4, #-2076]
	ldr	r5, [r4, #-952]
	str	r3, [r4, #176]
	ldrh	r3, [r4, r8]
	str	r5, [r4, #180]
	str	r6, [r5, #12]
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, .L1537+12
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #-1760]
	str	r6, [r5, #8]
	str	r3, [r5, #4]
	ldr	r3, .L1537+16
	str	r3, [r4, #-1820]
	ldr	r3, .L1537+20
	str	r3, [r4, #-1816]
	ldr	r3, .L1537+24
	ldrh	r2, [r4, r3]
	sub	r3, r3, #50
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #3866
	ldrh	r3, [r7, r3]
	strb	r3, [r4, #-1810]
	ldr	r3, .L1537+28
	ldrh	r2, [r4, r3]
	add	r3, r3, #238
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #236
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-2038]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1537+32
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-2036]	@ zero_extendqisi2
	strb	r3, [r4, #-1809]
	ldr	r3, .L1537+36
	ldrh	r2, [r4, r3]
	add	r3, r3, #194
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #192
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1990]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1537+40
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1988]	@ zero_extendqisi2
	strb	r3, [r4, #-1808]
	ldr	r3, .L1537+44
	ldrh	r2, [r4, r3]
	add	r3, r3, #150
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #148
	ldr	r0, [r4, #176]
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1942]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1537+48
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1940]	@ zero_extendqisi2
	strb	r3, [r4, #-1807]
	ldr	r3, [r4, #-1840]
	str	r3, [r4, #-1788]
	ldr	r3, [r4, #-1848]
	str	r3, [r4, #-1780]
	ldr	r3, [r4, #-1844]
	str	r3, [r4, #-1784]
	movw	r3, #3922
	ldrh	r2, [r7, r3]
	bl	ftl_memset
	ldr	r1, .L1537+52
	mov	r2, #48
	ldr	r0, [r4, #176]
	bl	memcpy
	ldrh	r2, [r7, sl]
	ldr	r0, [r4, #176]
	ldr	r1, [r4, #-2064]
	mov	r2, r2, asl #1
	add	r0, r0, #48
	bl	memcpy
	ldrh	r2, [r7, sl]
	ldr	r3, [r4, #176]
	movw	sl, #65535
	add	r0, r2, #24
	ldr	r1, [r4, #-1888]
	mov	r2, r2, lsr #3
	mov	r0, r0, lsr #1
	add	r2, r2, #4
	add	r0, r3, r0, asl #2
	bl	memcpy
	mov	r0, r6
	bl	FtlUpdateVaildLpn
.L1536:
	ldr	r3, [r4, #-2076]
	mov	r1, #1
	ldr	r7, .L1537+56
	ldrh	r2, [r4, r8]
	str	r3, [r4, #176]
	ldr	r3, [r4, #-952]
	ldr	r0, .L1537+60
	str	r3, [r4, #180]
	ldrh	r3, [r4, r7]
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r4, #172]
	mov	r3, r1
	bl	FlashProgPages
	ldr	r1, .L1537+8
	movw	r3, #3914
	ldrh	r2, [r4, r7]
	ldrh	r3, [r1, r3]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L1528
	ldr	r3, .L1537+64
	ldrh	r2, [r4, r8]
	ldrh	sl, [r4, r3]
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r7]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r4, #-1848]
	mov	r1, #1
	add	r2, r3, #1
	str	r2, [r4, #-1848]
	str	r3, [r4, #-1760]
	mov	r2, r0, asl #10
	strh	r0, [r4, r8]	@ movhi
	str	r2, [r4, #172]
	mov	r2, r1
	str	r3, [r5, #4]
	mov	r3, r1
	strh	r0, [r5, #2]	@ movhi
	ldr	r0, .L1537+60
	bl	FlashProgPages
.L1528:
	ldr	r2, .L1537+56
	ldr	r1, [r4, #168]
	ldrh	r3, [r4, r2]
	cmn	r1, #1
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, r2]	@ movhi
	bne	.L1529
	cmp	r3, #1
	add	r6, r6, #1
	ldreq	r1, .L1537+8
	movweq	r3, #3914
	uxth	r6, r6
	ldreqh	r3, [r1, r3]
	subeq	r3, r3, #1
	streqh	r3, [r4, r2]	@ movhi
	cmp	r6, #3
	bls	.L1536
	ldr	r3, .L1537
	mov	r2, r6
	ldr	r0, .L1537+68
	ldr	r1, [r3, #172]
	bl	printk
.L1532:
	b	.L1532
.L1529:
	cmp	r3, #1
	beq	.L1536
	movw	r3, #65535
	cmp	sl, r3
	beq	.L1533
	mov	r0, sl
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L1533:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1538:
	.align	2
.L1537:
	.word	.LANCHOR2
	.word	-1768
	.word	.LANCHOR0
	.word	-3932
	.word	1179929683
	.word	1342177344
	.word	-1762
	.word	-2044
	.word	-1804
	.word	-1996
	.word	-1800
	.word	-1948
	.word	-1796
	.word	.LANCHOR2-1820
	.word	-1766
	.word	.LANCHOR2+168
	.word	-1764
	.word	.LC110
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r2, #65535
	ldrh	r3, [r0, #0]
	mov	r4, r0
	ldr	r5, [r0, #12]
	cmp	r3, r2
	ldr	r7, [r0, #24]
	bne	.L1540
	ldrh	r3, [r0, #8]
	add	r3, r3, #1
	strh	r3, [r0, #8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	mov	r3, #0
	strh	r0, [r5, #0]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	strh	r3, [r4, #0]	@ movhi
	ldr	r3, [r4, #28]
	add	r3, r3, #1
	str	r3, [r4, #28]
	b	.L1541
.L1540:
	mov	r3, r3, asl #1
	ldr	r2, [r0, #28]
	mov	r1, #255
	ldrh	r6, [r5, r3]
	ldr	r5, .L1545
	ldrh	r3, [r0, #2]
	orr	r3, r3, r6, asl #10
	str	r3, [r5, #172]
	ldr	r3, [r5, #-2076]
	str	r3, [r5, #176]
	ldr	r3, [r5, #-952]
	str	r3, [r5, #180]
	str	r2, [r3, #4]
	ldr	r2, .L1545+4
	strh	r2, [r3, #8]	@ movhi
	ldrh	r2, [r0, #4]
	strh	r6, [r3, #2]	@ movhi
	strh	r2, [r3, #0]	@ movhi
	movw	r3, #3914
	ldr	r2, .L1545+8
	ldr	r0, [r5, #-2076]
	ldrh	r2, [r2, r3]
	mov	r2, r2, asl #3
	bl	ftl_memset
	mov	r2, #0
	mov	r3, r2
	b	.L1542
.L1544:
	ldr	r1, [r7, r3, asl #2]
	cmp	r6, r1, lsr #10
	bne	.L1543
	add	r2, r2, #1
	ldr	r1, [r5, #-2076]
	uxth	r2, r2
	str	r3, [r1, r2, asl #3]
	ldr	r1, [r5, #-2076]
	ldr	r0, [r7, r3, asl #2]
	add	r1, r1, r2, asl #3
	str	r0, [r1, #4]
.L1543:
	add	r3, r3, #1
	uxth	r3, r3
.L1542:
	ldrh	r1, [r4, #6]
	cmp	r1, r3
	bhi	.L1544
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L1545+12
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L1541:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1546:
	.align	2
.L1545:
	.word	.LANCHOR2
	.word	-1291
	.word	.LANCHOR0
	.word	.LANCHOR2+168
	.fnend
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	FtlMapWritePage
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	ldr	r5, .L1557
	mov	r8, r1
	ldr	fp, .L1557+4
	mov	r9, r2
	mov	r7, #4
.L1548:
.L1556:
	ldr	r3, [r5, #-1864]
	movw	r1, #3914
	add	r3, r3, #1
	str	r3, [r5, #-1864]
	ldrh	r3, [fp, r1]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L1549
	ldrh	r2, [r4, #0]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1550
.L1549:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L1550:
	ldrh	r3, [r4, #0]
	mov	r1, #0
	ldr	r2, [r4, #12]
	ldr	r0, [r5, #-952]
	mov	r3, r3, asl #1
	ldr	r6, .L1557
	ldrh	sl, [r2, r3]
	mov	r2, #16
	ldrh	r3, [r4, #2]
	str	r9, [r5, #176]
	orr	r3, r3, sl, asl #10
	str	r0, [r5, #180]
	str	r3, [r5, #172]
	bl	ftl_memset
	ldr	r3, [r5, #180]
	mov	r1, #1
	ldr	r2, [r4, #28]
	add	r0, r6, #168
	strh	r8, [r3, #8]	@ movhi
	str	r2, [r3, #4]
	ldrh	r2, [r4, #4]
	strh	sl, [r3, #2]	@ movhi
	strh	r2, [r3, #0]	@ movhi
	mov	r3, r1
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #2]	@ movhi
	cmp	r3, #1
	beq	.L1556
	ldr	r3, [r5, #168]
	cmn	r3, #1
	bne	.L1552
	sub	r7, r7, #1
	ldr	r0, .L1557+8
	ldr	r1, [r5, #172]
	uxth	r7, r7
	bl	printk
	cmp	r7, #0
	bne	.L1556
	ldr	r0, .L1557+12
	mov	r2, #4
	ldr	r1, [r6, #172]
	bl	printk
.L1553:
	b	.L1553
.L1552:
	ldr	r2, [r6, #172]
	mov	r0, #0
	ldr	r3, [r4, #24]
	str	r2, [r3, r8, asl #2]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1558:
	.align	2
.L1557:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC111
	.word	.LC112
	.fnend
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	2
	.global	FtlMapBlkWriteDumpData
	.type	FtlMapBlkWriteDumpData, %function
FtlMapBlkWriteDumpData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r2, [r0, #36]
	ldrh	r6, [r0, #6]
	cmp	r2, #0
	ldr	r3, [r0, #24]
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldr	r5, .L1563
	sub	r6, r6, #1
	mov	r2, #0
	str	r2, [r0, #36]
	uxth	r6, r6
	ldr	r0, [r5, #-972]
	ldr	r2, [r5, #-952]
	str	r0, [r5, #176]
	str	r2, [r5, #180]
	ldr	r3, [r3, r6, asl #2]
	cmp	r3, #0
	str	r3, [r5, #172]
	beq	.L1561
	mov	r1, #1
	add	r0, r5, #168
	mov	r2, r1
	bl	FlashReadPages
	b	.L1562
.L1561:
	ldr	r2, .L1563+4
	movw	r3, #3922
	mov	r1, #255
	ldrh	r2, [r2, r3]
	bl	ftl_memset
.L1562:
	mov	r0, r4
	mov	r1, r6
	ldr	r2, [r5, #176]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L1564:
	.align	2
.L1563:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1575
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r9, r2
	mov	r2, #3920
	mov	r4, r1
	ldrh	r8, [r3, r2]
	movw	r2, #3906
	ldrh	r2, [r3, r2]
	add	r1, r1, r0
	.pad #60
	sub	sp, sp, #60
	mov	r6, r0
	cmp	r1, r2
	mvnhi	r7, #0
	bhi	.L1566
	mov	r8, r0, lsr r8
	mov	r7, #0
	ldr	sl, .L1575+4
	mov	ip, r8, asl #2
	str	ip, [sp, #8]
	b	.L1567
.L1572:
	ldr	r1, .L1575
	movw	r2, #3918
	ldr	ip, [sp, #8]
	mov	r0, r6
	ldr	r3, [sl, #-920]
	ldrh	fp, [r1, r2]
	ldr	r3, [r3, ip]
	mov	r1, fp
	str	r3, [sp, #4]
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #4]
	ldr	r2, .L1575+4
	uxth	r1, r1
	str	r1, [sp, #12]
	rsb	r5, r1, fp
	uxth	r5, r5
	cmp	r5, r4
	uxthhi	r5, r4
	cmp	r3, #0
	beq	.L1569
	cmp	r5, fp
	beq	.L1569
	str	r3, [sp, #24]
	mov	r1, #1
	ldr	r3, [r2, #-968]
	add	r0, sp, #20
	mov	r2, r1
	str	r3, [sp, #28]
	mov	r3, #0
	str	r3, [sp, #32]
	bl	FlashReadPages
	b	.L1570
.L1569:
	ldr	ip, .L1575
	movw	r3, #3922
	ldr	r0, [r2, #-968]
	mov	r1, #0
	ldrh	r2, [ip, r3]
	bl	ftl_memset
.L1570:
	ldr	r3, [sp, #12]
	mov	fp, r5, asl #9
	ldr	r0, [sl, #-968]
	mov	r1, r9
	mov	r2, fp
	rsb	r4, r5, r4
	add	r0, r0, r3, asl #9
	add	r6, r6, r5
	bl	memcpy
	mov	r1, r8
	ldr	r0, .L1575+8
	add	r8, r8, #1
	ldr	r2, [sl, #-968]
	add	r9, r9, fp
	bl	FtlMapWritePage
	ldr	ip, [sp, #8]
	add	ip, ip, #4
	str	ip, [sp, #8]
	cmn	r0, #1
	moveq	r7, r0
.L1567:
	cmp	r4, #0
	bne	.L1572
.L1566:
	mov	r0, r7
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1576:
	.align	2
.L1575:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+224
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
	ldr	r3, .L1579
	ldr	r2, .L1579+4
	ldr	r1, [r3, #-1704]
	cmp	r1, r2
	bxne	lr
	ldr	r2, .L1579+8
	mov	r0, #0
	mov	r1, #1
	str	r2, [r3, #-1700]
	ldr	r2, [r3, #-1856]
	str	r2, [r3, #-1616]
	ldr	r2, [r3, #-1852]
	str	r2, [r3, #-1612]
	ldr	r2, [r3, #-1860]
	str	r2, [r3, #-1696]
	ldr	r2, [r3, #-1872]
	str	r2, [r3, #-1692]
	ldr	r2, [r3, #-1880]
	str	r2, [r3, #-1688]
	ldr	r2, [r3, #-1864]
	str	r2, [r3, #-1684]
	ldr	r2, [r3, #-1836]
	str	r2, [r3, #-1676]
	ldr	r2, [r3, #-2080]
	str	r2, [r3, #-1672]
	ldr	r2, [r3, #-1876]
	str	r2, [r3, #-1668]
	ldr	r2, [r3, #-1868]
	str	r2, [r3, #-1664]
	ldr	r2, [r3, #-1828]
	str	r2, [r3, #-1660]
	ldr	r2, [r3, #-1824]
	str	r2, [r3, #-1656]
	ldr	r2, [r3, #-1708]
	str	r2, [r3, #-1644]
	ldr	r2, [r3, #-1004]
	str	r2, [r3, #-1640]
	ldr	r2, .L1579+12
	b	FtlVendorPartWrite
.L1580:
	.align	2
.L1579:
	.word	.LANCHOR2
	.word	1179929683
	.word	1342177344
	.word	.LANCHOR2-1704
	.fnend
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L1588
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, [r1, #-1884]
	cmp	r3, #0
	moveq	r2, #32
	beq	.L1582
	ldr	r2, [r1, #-1824]
	cmp	r2, #29
	movls	r2, #4
	movhi	r2, #32
.L1582:
	mov	r3, #264
	ldrh	r1, [r1, r3]
	cmp	r1, #31
	addls	r1, r1, #1
	ldrls	r2, .L1588
	strlsh	r1, [r2, r3]	@ movhi
	movls	r2, #1
	cmp	r0, #0
	ldr	r3, .L1588
	bne	.L1584
	ldr	r1, [r3, #-936]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r2, r2, r0
	cmp	r1, r2
	bcc	.L1585
.L1584:
	ldr	r2, [r3, #-936]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L1588+4
	str	r1, [r2, #0]
	ldr	r1, .L1588+8
	ldr	r2, [r3, #-936]
	ldrh	r1, [r3, r1]
	mov	r3, r1, asl #9
	str	r3, [r2, #12]
	ldr	r3, [r2, #8]
	add	r3, r3, #1
	str	r3, [r2, #8]
	mov	r3, #0
	str	r3, [r2, #4]
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L1585:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1589:
	.align	2
.L1588:
	.word	.LANCHOR2
	.word	1112818501
	.word	-940
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
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
	ldr	r5, .L1591
	mul	r4, r4, r0
	sub	r0, r5, #1088
	ldr	r2, [r5, #-1900]
	add	r3, r2, r4
	ldrh	r1, [r2, r4]
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #-1900]
	mov	r0, #0
	add	r4, r3, r4
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1592:
	.align	2
.L1591:
	.word	.LANCHOR2
	.fnend
	.size	flush_l2p_region, .-flush_l2p_region
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r8, r2
	ldr	r3, .L1602
	mov	r2, #3920
	mov	r4, r1
	ldrh	r7, [r3, r2]
	mvn	r2, #0
	add	r7, r7, #7
	mov	r6, r0, lsr r7
	bic	r7, r0, r2, asl r7
	movw	r2, #3950
	ldrh	r2, [r3, r2]
	uxth	r6, r6
	ldr	r3, .L1602+4
	uxth	r7, r7
	ldr	sl, [r3, #-1900]
	mov	r3, #0
	mov	r5, r3
	b	.L1594
.L1600:
	add	r3, r3, #12
	add	r1, sl, r3
	ldrh	r1, [r1, #-12]
	cmp	r1, r6
	bne	.L1595
.L1596:
	cmp	r8, #0
	ldr	r3, .L1602+4
	mov	r2, #12
	bne	.L1597
	ldr	r3, [r3, #-1900]
	mla	r2, r2, r5, r3
	ldr	r3, [r2, #8]
	ldr	r3, [r3, r7, asl #2]
	str	r3, [r4, #0]
	b	.L1598
.L1597:
	mul	r2, r2, r5
	ldr	r1, [r3, #-1900]
	ldr	r0, [r4, #0]
	add	r1, r1, r2
	ldr	r1, [r1, #8]
	str	r0, [r1, r7, asl #2]
	ldr	r1, [r3, #-1900]
	add	r2, r1, r2
	ldr	r1, [r2, #4]
	orr	r1, r1, #-2147483648
	str	r1, [r2, #4]
	ldr	r2, .L1602+8
	strh	r6, [r3, r2]	@ movhi
.L1598:
	ldr	r3, .L1602+4
	mov	r2, #12
	mov	r0, #0
	ldr	r3, [r3, #-1900]
	mla	r5, r2, r5, r3
	ldr	r3, [r5, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r5, #4]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1595:
	add	r5, r5, #1
	uxth	r5, r5
.L1594:
	cmp	r5, r2
	bne	.L1600
	bl	select_l2p_ram_region
	mov	r3, #12
	mul	r3, r3, r0
	mov	r5, r0
	add	r2, sl, r3
	ldrh	r1, [sl, r3]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1601
	ldr	r3, [r2, #4]
	cmp	r3, #0
	bge	.L1601
	bl	flush_l2p_region
.L1601:
	mov	r0, r6
	mov	r1, r5
	bl	load_l2p_region
	b	.L1596
.L1603:
	.align	2
.L1602:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1896
	.fnend
	.size	log2phys, .-log2phys
	.align	2
	.type	FtlReadRefresh.part.13, %function
FtlReadRefresh.part.13:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #40
	sub	sp, sp, #40
	ldr	r4, .L1612
	ldr	r6, .L1612+4
	ldr	r2, [r4, #-1620]
	ldr	r3, [r6, #3972]
	cmp	r2, r3
	bcs	.L1605
	mov	r5, #2048
.L1608:
	ldr	r0, [r4, #-1620]
	ldr	r3, [r6, #3972]
	cmp	r0, r3
	bcs	.L1610
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #0]
	ldr	r3, [r4, #-1620]
	cmn	r0, #1
	add	r3, r3, #1
	str	r3, [r4, #-1620]
	beq	.L1607
	str	r0, [sp, #8]
	add	r0, sp, #40
	mov	r2, #0
	mov	r1, #1
	str	r2, [r0, #-36]!
	str	r3, [sp, #20]
	str	r2, [sp, #12]
	str	r2, [sp, #16]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmp	r3, #256
	bne	.L1610
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L1610
.L1607:
	subs	r5, r5, #1
	bne	.L1608
	b	.L1606
.L1605:
	ldr	r3, [r4, #-1880]
	mov	r0, #0
	str	r0, [r4, #-1624]
	str	r0, [r4, #-1620]
	str	r3, [r4, #-1628]
	b	.L1606
.L1610:
	mvn	r0, #0
.L1606:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, pc}
.L1613:
	.align	2
.L1612:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh.part.13, .-FtlReadRefresh.part.13
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1622
	ldr	r2, [r3, #-1624]
	cmp	r2, #0
	beq	.L1615
	b	FtlReadRefresh.part.13
.L1615:
	ldr	r1, .L1622+4
	ldr	ip, [r3, #-1828]
	ldr	r2, [r3, #-1884]
	ldrb	r0, [r1, #76]	@ zero_extendqisi2
	cmp	r0, #0
	addeq	ip, r2, ip, lsr #10
	ldr	r2, [r3, #-1880]
	moveq	r1, #33554432
	movne	ip, #1048576
	moveq	ip, r1, asr ip
	ldr	r1, [r3, #-1628]
	add	r3, r2, #1048576
	cmp	r1, r3
	bhi	.L1617
	add	r1, ip, r1
	cmp	r1, r2
	bcc	.L1617
	ldr	r3, .L1622+8
	ldr	r1, .L1622
	ldrh	r3, [r1, r3]
	tst	r3, #63
	bne	.L1620
	cmp	r0, #0
	bxeq	lr
.L1617:
	ldr	r3, .L1622
	mov	r1, #1
	mov	r0, #0
	str	r1, [r3, #-1624]
	str	r0, [r3, #-1620]
	str	r2, [r3, #-1628]
	bx	lr
.L1620:
	mov	r0, #0
	bx	lr
.L1623:
	.align	2
.L1622:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1792
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	FtlReUsePrevPpa
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, lr}
	mov	r7, r0
	ubfx	r0, r1, #10, #16
	str	r1, [sp, #4]
	bl	P2V_block_in_plane
	ldr	r3, .L1630
	ldr	r1, [r3, #-2064]
	mov	r5, r0, asl #1
	ldrh	r2, [r1, r5]
	cmp	r2, #0
	addne	r2, r2, #1
	strneh	r2, [r1, r5]	@ movhi
	bne	.L1626
	ldr	r4, [r3, #-2052]
	cmp	r4, #0
	beq	.L1626
	ldr	r1, .L1630+4
	mov	ip, #6
	ldrh	lr, [r3, r1]
	ldr	r3, [r3, #-2072]
	ldr	r1, .L1630+8
	rsb	r4, r3, r4
	mov	r4, r4, asr #1
	mul	r4, r1, r4
	movw	r1, #65535
	uxth	r4, r4
	b	.L1627
.L1629:
	cmp	r4, r0
	bne	.L1628
	mov	r1, r4
	ldr	r0, .L1630+12
	bl	List_remove_node
	ldr	r6, .L1630
	ldr	r3, .L1630+4
	mov	r0, r4
	ldrh	r2, [r6, r3]
	sub	r2, r2, #1
	strh	r2, [r6, r3]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r6, #-2064]
	ldrh	r2, [r3, r5]
	add	r2, r2, #1
	strh	r2, [r3, r5]	@ movhi
	b	.L1626
.L1628:
	mul	r4, ip, r4
	ldrh	r4, [r3, r4]
	cmp	r4, r1
	beq	.L1626
	add	r2, r2, #1
	uxth	r2, r2
.L1627:
	cmp	r2, lr
	bne	.L1629
.L1626:
	mov	r0, r7
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L1631:
	.align	2
.L1630:
	.word	.LANCHOR2
	.word	-2048
	.word	-1431655765
	.word	.LANCHOR2-2052
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	ftl_map_blk_gc
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r4, r0
	ldr	r5, [r0, #12]
	ldr	r8, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	sub	r3, r3, #4
	cmp	r2, r3
	blt	.L1633
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r7, [r5, r0]
	cmp	r7, #0
	beq	.L1633
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L1633
	mov	r2, #1
	ldr	r1, .L1640
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	movw	r3, #3914
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1634
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1634:
	mov	r5, #0
	ldr	r6, .L1640+4
	mov	r9, r5
	b	.L1635
.L1638:
	ldr	r3, [r8, r5, asl #2]
	mov	sl, r5, asl #2
	cmp	r7, r3, lsr #10
	bne	.L1636
	ldr	r3, [r6, #-972]
	mov	r1, #1
	ldr	r0, .L1640+8
	mov	r2, r1
	str	r3, [r6, #176]
	ldr	r3, [r6, #-952]
	str	r3, [r6, #180]
	ldr	r3, [r8, r5, asl #2]
	str	r3, [r6, #172]
	bl	FlashReadPages
	ldr	r3, [r6, #168]
	cmn	r3, #1
	streq	r9, [r8, sl]
	beq	.L1636
	mov	r0, r4
	mov	r1, r5
	ldr	r2, [r6, #176]
	bl	FtlMapWritePage
.L1636:
	add	r5, r5, #1
	uxth	r5, r5
.L1635:
	ldrh	r3, [r4, #6]
	cmp	r3, r5
	bhi	.L1638
	mov	r0, r7
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L1633:
	ldr	r1, .L1640
	movw	r3, #3914
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1639
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1639:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1641:
	.align	2
.L1640:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+168
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	FtlMapTblRecovery
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r3, [r0, #16]
	mov	r4, r0
	ldrh	sl, [r0, #6]
	mov	r1, #0
	ldr	r8, [r0, #24]
	mov	r9, #0
	ldr	r5, .L1661
	str	r3, [sp, #16]
	mov	r2, sl, asl #2
	ldrh	r3, [r0, #8]
	ldr	r7, [r0, #12]
	mov	r0, r8
	str	r3, [sp, #8]
	bl	ftl_memset
	ldr	r3, [r5, #-2076]
	ldr	r6, [r5, #-952]
	str	r9, [r4, #32]
	str	r3, [r5, #176]
	mvn	r3, #0
	str	r6, [r5, #180]
	strh	r3, [r4, #0]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #1
	str	r3, [r4, #36]
	ldr	r3, [sp, #8]
	str	r9, [r4, #28]
	sub	r3, r3, #1
	str	r3, [sp, #12]
	b	.L1643
.L1657:
	ldr	r0, [sp, #12]
	cmp	r3, r0
	mov	r3, r3, asl #1
	bne	.L1644
	ldrh	r0, [r7, r3]
	mov	r1, #1
	add	fp, r7, r3
	bl	FtlGetLastWrittenPage
	ldr	r1, [sp, #12]
	mov	r7, #0
	ldr	r2, [sp, #16]
	strh	r9, [r4, #0]	@ movhi
	ldr	r5, .L1661
	uxth	r0, r0
	add	r3, r0, #1
	strh	r3, [r4, #2]	@ movhi
	ldr	r3, [r2, r1, asl #2]
	sxth	r9, r0
	str	r7, [r5, #176]
	add	r9, r9, #1
	str	r3, [r4, #28]
	b	.L1645
.L1647:
	ldrh	r2, [fp, #0]
	mov	r1, #1
	ldr	r0, .L1661+4
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #172]
	bl	FlashReadPages
	ldr	r3, [r5, #168]
	cmn	r3, #1
	beq	.L1646
	ldrh	r3, [r6, #8]
	cmp	r3, sl
	bcs	.L1646
	ldrh	r2, [r4, #4]
	ldrh	r1, [r6, #0]
	cmp	r1, r2
	ldreq	r2, [r5, #172]
	streq	r2, [r8, r3, asl #2]
.L1646:
	add	r7, r7, #1
	uxth	r7, r7
.L1645:
	sxth	r3, r7
	cmp	r3, r9
	blt	.L1647
	b	.L1648
.L1644:
	ldr	r2, [r5, #-2076]
	add	r0, r7, r3
	ldr	fp, .L1661+8
	str	r0, [sp, #20]
	str	r2, [r5, #176]
	ldrh	r1, [r7, r3]
	movw	r3, #3914
	ldrh	r2, [fp, r3]
	ldr	r0, .L1661+4
	sub	r2, r2, #1
	str	r3, [sp, #0]
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [r5, #172]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [r5, #168]
	ldr	r3, [sp, #0]
	cmn	r2, #1
	beq	.L1649
	ldrh	r1, [r6, #0]
	ldrh	r2, [r4, #4]
	cmp	r1, r2
	bne	.L1649
	ldrh	r1, [r6, #8]
	movw	r2, #64245
	cmp	r1, r2
	bne	.L1649
	b	.L1660
.L1652:
	ldr	r0, [r5, #-2076]
	mov	ip, r1, asl #3
	add	r2, r2, #1
	ldr	r1, [r0, r1, asl #3]
	uxth	r2, r2
	uxth	r1, r1
	cmp	r1, sl
	addcc	r0, r0, ip
	ldrcc	r0, [r0, #4]
	strcc	r0, [r8, r1, asl #2]
	b	.L1650
.L1660:
	mov	r2, #0
.L1650:
	ldrh	r0, [fp, r3]
	sxth	r1, r2
	sub	r0, r0, #1
	cmp	r1, r0
	blt	.L1652
	b	.L1653
.L1649:
	ldr	r1, .L1661
	mov	fp, #0
	ldr	r3, .L1661+8
	add	ip, r1, #168
	str	fp, [r1, #176]
	b	.L1654
.L1656:
	ldr	r0, [sp, #20]
	ldrh	r1, [r0, #0]
	mov	r0, ip
	stmia	sp, {r3, ip}
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [r5, #172]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [r5, #168]
	ldmia	sp, {r3, ip}
	cmn	r2, #1
	beq	.L1655
	ldrh	r2, [r6, #8]
	cmp	r2, sl
	bcs	.L1655
	ldrh	r1, [r4, #4]
	ldrh	r0, [r6, #0]
	cmp	r0, r1
	ldreq	r1, [r5, #172]
	streq	r1, [r8, r2, asl #2]
.L1655:
	add	fp, fp, #1
	uxth	fp, fp
.L1654:
	movw	r1, #3914
	sxth	r2, fp
	ldrh	r1, [r3, r1]
	cmp	r2, r1
	blt	.L1656
.L1653:
	add	r9, r9, #1
	uxth	r9, r9
.L1643:
	ldr	r1, [sp, #8]
	sxth	r3, r9
	cmp	r3, r1
	blt	.L1657
.L1648:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r1, .L1661+8
	movw	r3, #3914
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bne	.L1658
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1658:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1662:
	.align	2
.L1661:
	.word	.LANCHOR2
	.word	.LANCHOR2+168
	.word	.LANCHOR0
	.fnend
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
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
	ldr	r0, .L1664
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1665:
	.align	2
.L1664:
	.word	.LANCHOR2-1088
	.fnend
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	movw	r2, #3932
	ldr	r3, .L1667
	ldr	r0, .L1667+4
	ldrh	r2, [r3, r2]
	strh	r2, [r0, #234]	@ movhi
	ldr	r2, .L1667+8
	strh	r2, [r0, #228]	@ movhi
	movw	r2, #3956
	ldrh	r2, [r3, r2]
	strh	r2, [r0, #232]	@ movhi
	movw	r2, #3934
	ldrh	r2, [r3, r2]
	ldr	r3, [r3, #3960]
	strh	r2, [r0, #230]	@ movhi
	str	r3, [r0, #236]
	ldr	r3, [r0, #-924]
	str	r3, [r0, #240]
	ldr	r3, [r0, #-928]
	str	r3, [r0, #244]
	ldr	r3, [r0, #-920]
	str	r3, [r0, #248]
	add	r0, r0, #224
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1668:
	.align	2
.L1667:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-3962
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FtlBbmTblFlush
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r1, #0
	ldr	r7, .L1681
	mov	r6, #0
	ldr	sl, .L1681+4
	movw	r9, #3866
	ldr	r8, .L1681+8
	ldr	r3, [r7, #-952]
	ldr	r0, [r7, #-2076]
	str	r3, [r7, #180]
	movw	r3, #3922
	str	r0, [r7, #176]
	ldrh	r2, [sl, r3]
	bl	ftl_memset
	b	.L1670
.L1671:
	ldr	r3, .L1681+12
	ldr	r1, [r8, #4]!
	ldrh	r2, [r7, r3]
	ldr	r3, [r7, #176]
	mul	r0, r2, r6
	mov	r2, r2, asl #2
	add	r6, r6, #1
	add	r0, r3, r0, asl #2
	bl	memcpy
.L1670:
	ldrh	r3, [sl, r9]
	ldr	r4, .L1681+4
	cmp	r6, r3
	ldr	r5, .L1681
	blt	.L1671
	ldr	r6, [r5, #180]
	mov	r1, #255
	mov	r2, #16
	mov	r7, #0
	mov	fp, r7
	mov	r0, r6
	bl	ftl_memset
	ldr	r3, .L1681+16
	strh	r3, [r6, #0]	@ movhi
	ldr	r3, [r4, #3988]
	str	r3, [r6, #4]
	movw	r3, #3980
	ldrh	r3, [r4, r3]
	strh	r3, [r6, #2]	@ movhi
	mov	r3, #3984
	ldrh	r3, [r4, r3]
	strh	r3, [r6, #8]	@ movhi
	movw	r3, #3986
	ldrh	r3, [r4, r3]
	strh	r3, [r6, #10]	@ movhi
	ldr	r3, [r4, #3840]
	strh	r3, [r6, #12]	@ movhi
	b	.L1680
.L1677:
	mov	fp, #1
.L1680:
	ldr	r3, [r5, #-2076]
	movw	r8, #3980
	ldr	r4, .L1681+4
	mov	r1, #0
	movw	sl, #3982
	str	r1, [r5, #168]
	str	r3, [r5, #176]
	mov	r9, #3984
	ldr	r3, [r5, #-952]
	ldrh	r1, [r4, r8]
	ldrh	r2, [r4, sl]
	str	r3, [r5, #180]
	ldrh	r0, [r6, #10]
	orr	r3, r2, r1, asl #10
	str	r3, [r5, #172]
	ldrh	r3, [r4, r9]
	str	r0, [sp, #0]
	ldr	r0, .L1681+20
	bl	printk
	mov	r1, #1
	mov	r2, r1
	mov	r3, r1
	ldr	r0, .L1681+24
	bl	FlashProgPages
	movw	r3, #3914
	ldrh	r3, [r4, r3]
	ldrh	r2, [r4, sl]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L1673
	ldr	r3, [r4, #3988]
	mov	r1, #0	@ movhi
	ldrh	r2, [r4, r8]
	add	r3, r3, #1
	ldr	r0, [r5, #-2088]
	str	r3, [r4, #3988]
	str	r3, [r6, #4]
	ldrh	r3, [r4, r9]
	strh	r1, [r4, sl]	@ movhi
	mov	r1, #1
	strh	r2, [r6, #8]	@ movhi
	strh	r2, [r4, r9]	@ movhi
	mov	r2, r1
	strh	r3, [r4, r8]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r5, #172]
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	mov	r1, #1
	ldr	r0, .L1681+24
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
.L1673:
	ldr	r2, .L1681+4
	movw	r3, #3982
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r5, #168]
	cmn	r3, #1
	bne	.L1674
	add	r7, r7, #1
	ldr	r0, .L1681+28
	ldr	r1, [r5, #172]
	uxth	r7, r7
	bl	printk
	cmp	r7, #3
	bls	.L1680
	ldr	r3, .L1681
	mov	r2, r7
	ldr	r0, .L1681+32
	ldr	r1, [r3, #172]
	bl	printk
.L1676:
	b	.L1676
.L1674:
	cmp	fp, #0
	beq	.L1677
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1682:
	.align	2
.L1681:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+4004
	.word	-1032
	.word	-3887
	.word	.LC113
	.word	.LANCHOR2+168
	.word	.LC114
	.word	.LC115
	.fnend
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r9, r0
	ldr	r4, .L1693
	ldr	r3, .L1693+4
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	movne	r6, #0
	movne	sl, r4
	bne	.L1684
	b	.L1685
.L1691:
	add	r3, r3, r6
	mov	r1, r9
	mov	r5, #0
	ldr	r7, .L1693+4
	ldrb	r0, [r3, #3870]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	b	.L1686
.L1690:
	add	r3, r4, r5, asl #1
	sub	r3, r3, #1120
	sub	r3, r3, #12
	ldrh	r3, [r3, #0]
	cmp	r3, r8
	bne	.L1687
	mov	r1, r8
	ldr	r0, .L1693+8
	bl	printk
	mov	r0, r8
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r1, [r4, r7]
	mov	r3, r5
	b	.L1688
.L1689:
	add	r0, r3, #1
	add	r3, r4, r3, asl #1
	sub	r3, r3, #1120
	add	r2, r4, r0, asl #1
	sub	r2, r2, #1120
	sub	r2, r2, #12
	ldrh	r2, [r2, #0]
	strh	r2, [r3, #-12]	@ movhi
	uxth	r3, r0
.L1688:
	cmp	r3, r1
	bcc	.L1689
	sub	r1, r1, #1
	strh	r1, [sl, r7]	@ movhi
.L1687:
	add	r5, r5, #1
	uxth	r5, r5
.L1686:
	ldrh	r3, [r4, r7]
	cmp	r3, r5
	bhi	.L1690
	add	r6, r6, #1
	uxth	r6, r6
.L1684:
	ldr	r3, .L1693+12
	movw	r2, #3844
	ldrh	r2, [r3, r2]
	cmp	r2, r6
	bhi	.L1691
	bl	FtlGcReFreshBadBlk
.L1685:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1694:
	.align	2
.L1693:
	.word	.LANCHOR2
	.word	-1134
	.word	.LC116
	.word	.LANCHOR0
	.fnend
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r2, r0, asl #1
	ldr	r3, .L1703
	mov	r4, r0
	ldr	r1, [r3, #-2064]
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	bne	.L1696
	ldr	r1, .L1703+4
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	mvneq	r2, #0
	streqh	r2, [r3, r1]	@ movhi
	beq	.L1698
	ldr	r1, .L1703+8
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1699
	ldr	r1, .L1703+12
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1699
	ldr	r1, .L1703+16
	ldrh	r3, [r3, r1]
	cmp	r3, r4
	beq	.L1699
.L1698:
	mov	r1, r4
	ldr	r0, .L1703+20
	bl	List_remove_node
	ldr	r5, .L1703
	ldr	r3, .L1703+24
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	free_data_superblock
	mov	r0, r4
	bl	FtlGcFreeBadSuperBlk
	mov	r2, #1
	b	.L1699
.L1696:
	bl	List_update_data_list
	mov	r2, #0
.L1699:
	mov	r0, r2
	ldmfd	sp!, {r3, r4, r5, pc}
.L1704:
	.align	2
.L1703:
	.word	.LANCHOR2
	.word	-1756
	.word	-2044
	.word	-1996
	.word	-1948
	.word	.LANCHOR2-2068
	.word	-2056
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
	mov	r5, r0
	beq	.L1706
	ldr	r2, .L1711
	mov	r3, r0, asl #1
	ldr	r2, [r2, #-2064]
	ldrh	r4, [r2, r3]
	cmp	r4, #0
	subne	r4, r4, #1
	strneh	r4, [r2, r3]	@ movhi
	bne	.L1706
	ldr	r0, .L1711+4
	mov	r1, r5
	mov	r2, r4
	bl	printk
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1706:
	ldr	r7, .L1711
	movw	r3, #65535
	ldr	r6, .L1711+8
	ldrh	r0, [r7, r6]
	cmp	r0, r3
	streqh	r5, [r7, r6]	@ movhi
	moveq	r0, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r0, r5
	beq	.L1710
	bl	update_vpc_list
	strh	r5, [r7, r6]	@ movhi
	adds	r0, r0, #0
	movne	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1710:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1712:
	.align	2
.L1711:
	.word	.LANCHOR2
	.word	.LC117
	.word	-1024
	.fnend
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	get_new_active_ppa
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r3, #0
	strb	r3, [r0, #10]
	mov	r4, r0
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	movw	r6, #65535
	ldr	r5, .L1725
	ldr	r7, .L1725+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1714
.L1715:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	r2, #3844
	ldrh	r2, [r5, r2]
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
.L1714:
	cmp	r3, r6
	beq	.L1715
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L1717
	ldrb	r2, [r5, #76]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1717
	ldrh	r2, [r4, #2]
	add	r2, r7, r2, asl #1
	sub	r2, r2, #912
	ldrh	r2, [r2, #0]
	cmp	r2, r6
	bne	.L1717
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1715
.L1717:
	ldrh	r6, [r4, #2]
	movw	r5, #65535
	mov	r7, r5
	orr	r6, r6, r3, asl #10
	ldrh	r3, [r4, #4]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
.L1722:
	ldr	r2, .L1725
	movw	r3, #3844
	ldrh	r2, [r2, r3]
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
.L1719:
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, r2
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	add	r1, r4, r3, asl #1
	ldrh	r1, [r1, #16]
	cmp	r1, r5
	beq	.L1719
	strb	r3, [r4, #6]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1720
	ldr	r3, .L1725
	ldrb	r2, [r3, #76]	@ zero_extendqisi2
	cmp	r2, #0
	ldrh	r2, [r4, #2]
	bne	.L1723
	ldr	r3, .L1725+4
	add	r2, r3, r2, asl #1
	sub	r2, r2, #912
	ldrh	r3, [r2, #0]
	cmp	r3, r7
	bne	.L1720
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1720
	sub	r3, r3, #1
	ldrh	r0, [r4, #0]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1722
.L1723:
	movw	r1, #3914
	ldrh	r1, [r3, r1]
	cmp	r2, r1
	bcc	.L1720
	ldr	r1, .L1725+4
	ldrh	r2, [r4, #0]
	ldrh	r0, [r4, #4]
	ldr	r1, [r1, #-2064]
	mov	r2, r2, asl #1
	ldrh	ip, [r1, r2]
	rsb	r0, r0, ip
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	strh	r2, [r4, #4]	@ movhi
	mov	r1, r2	@ movhi
	movw	r2, #3912
	ldrh	r3, [r3, r2]
	strb	r1, [r4, #6]
	strh	r3, [r4, #2]	@ movhi
.L1720:
	mov	r0, r6
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1726:
	.align	2
.L1725:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.type	FtlSlcSuperblockCheck.part.16, %function
FtlSlcSuperblockCheck.part.16:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	movw	r6, #65535
	ldr	r5, .L1733
	ldr	r7, .L1733+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1728
.L1729:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	r2, #3844
	ldrh	r2, [r5, r2]
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
.L1728:
	cmp	r3, r6
	beq	.L1729
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L1731
	ldrb	r3, [r5, #76]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1731
	ldrh	r3, [r4, #2]
	add	r3, r7, r3, asl #1
	sub	r3, r3, #912
	ldrh	r3, [r3, #0]
	cmp	r3, r6
	bne	.L1731
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1729
	ldrh	r2, [r4, #2]
	strb	r3, [r4, #6]
	add	r2, r2, #1
	strh	r2, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1731:
	ldr	r3, .L1733
	ldrb	r1, [r3, #76]	@ zero_extendqisi2
	cmp	r1, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r2, #1
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	movw	r2, #3914
	ldrh	r1, [r4, #2]
	ldrh	r2, [r3, r2]
	cmp	r1, r2
	ldmccfd	sp!, {r3, r4, r5, r6, r7, pc}
	ldr	r1, .L1733+4
	ldrh	r2, [r4, #0]
	ldrh	r0, [r4, #4]
	ldr	r1, [r1, #-2064]
	mov	r2, r2, asl #1
	ldrh	ip, [r1, r2]
	rsb	r0, r0, ip
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	strh	r2, [r4, #4]	@ movhi
	mov	r1, r2	@ movhi
	movw	r2, #3912
	ldrh	r3, [r3, r2]
	strb	r1, [r4, #6]
	strh	r3, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1734:
	.align	2
.L1733:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlSlcSuperblockCheck.part.16, .-FtlSlcSuperblockCheck.part.16
	.align	2
	.global	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrh	r2, [r0, #4]
	cmp	r2, #0
	bxeq	lr
	ldrh	r2, [r0, #0]
	movw	r3, #65535
	cmp	r2, r3
	bxeq	lr
	b	FtlSlcSuperblockCheck.part.16
	.fnend
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	2
	.global	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r2, #65535
	ldrh	r3, [r0, #0]
	.pad #68
	sub	sp, sp, #68
	mov	r4, r0
	cmp	r3, r2
	beq	.L1738
	ldrh	r3, [r0, #2]
	ldr	r1, .L1825
	ldrb	fp, [r0, #6]	@ zero_extendqisi2
	str	r3, [sp, #20]
	movw	r3, #3912
	ldrh	r3, [r1, r3]
	ldr	ip, [sp, #20]
	str	fp, [sp, #32]
	cmp	r3, ip
	mov	r3, #0
	streqh	r3, [r0, #4]	@ movhi
	moveq	r2, r3	@ movhi
	ldrneh	r0, [r0, #16]
	bne	.L1740
	b	.L1822
.L1741:
	add	r3, r3, #1
	uxth	r3, r3
	add	r1, r4, r3, asl #1
	ldrh	r0, [r1, #16]
.L1740:
	cmp	r0, r2
	beq	.L1741
	ldrb	r1, [r4, #8]	@ zero_extendqisi2
	cmp	r1, #1
	bne	.L1742
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1743
	ldr	r3, .L1825
	ldrb	r2, [r3, #76]	@ zero_extendqisi2
	cmp	r2, #0
	addeq	r3, r3, r0, asl #1
	addeq	r3, r3, #2224
	ldreqh	r5, [r3, #0]
	beq	.L1807
	b	.L1815
.L1742:
	mov	r1, #0
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1743
.L1815:
	mov	r5, r7
.L1807:
	ldr	r3, .L1825
	movw	r2, #3844
	mov	r6, #0
	str	r4, [sp, #40]
	movw	ip, #65535
	ldrh	r9, [r3, r2]
	ldr	r2, .L1825+4
	ldr	sl, [r2, #-984]
	ldr	r8, [r2, #-1164]
	movw	r2, #3924
	ldrh	lr, [r3, r2]
	mov	r2, r4
	mov	r3, r6
	b	.L1745
.L1743:
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	mov	r2, r3	@ movhi
.L1822:
	strb	r2, [r4, #6]
	b	.L1738
.L1747:
	ldrh	r0, [r2, #16]
	cmp	r0, ip
	beq	.L1746
	mov	fp, #36
	orr	r0, r5, r0, asl #10
	mla	r1, fp, r6, sl
	str	r0, [r1, #4]
	mov	r0, #0
	str	r0, [r1, #8]
	mul	r0, lr, r6
	add	r6, r6, #1
	uxth	r6, r6
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	bic	r0, r0, #3
	add	r0, r8, r0
	str	r0, [r1, #12]
.L1746:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1745:
	cmp	r3, r9
	bne	.L1747
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	movne	r3, #0
	bne	.L1816
	ldr	r3, .L1825
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	adds	r3, r3, #0
	movne	r3, #1
.L1816:
	ldr	r8, .L1825+4
	mov	r1, r6
	str	r3, [sp, #28]
	mov	sl, #0
	ldr	r2, [sp, #28]
	ldr	r0, [r8, #-984]
	bl	FlashReadPages
	ldr	fp, [r8, #-1844]
	uxth	r2, r5
	ldr	r9, [r8, #-984]
	movw	ip, #65535
	mov	r3, r8
	str	fp, [sp, #16]
	mov	r8, r2
	str	ip, [sp, #12]
	mov	fp, r7
	b	.L1749
.L1754:
	ldr	r2, [r9, #0]
	cmp	r2, #0
	strne	r8, [sp, #12]
	bne	.L1752
	ldr	r2, [r9, #12]
	ldr	r7, [r2, #4]
	cmn	r7, #1
	beq	.L1751
	ldr	r1, [r3, #-1844]
	mov	r0, r7
	str	r2, [sp, #8]
	str	r3, [sp, #4]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #4]
	ldr	r2, [sp, #8]
	cmp	r0, #0
	addne	r7, r7, #1
	strne	r7, [r3, #-1844]
.L1751:
	ldr	r2, [r2, #0]
	cmn	r2, #1
	bne	.L1752
	ldr	r3, .L1825+4
	mov	r2, #36
	uxth	r7, fp
	ldr	r3, [r3, #-984]
	mla	sl, r2, sl, r3
	ldr	r0, [sl, #4]
	b	.L1817
.L1752:
	add	sl, sl, #1
	add	r9, r9, #36
	uxth	sl, sl
.L1749:
	cmp	sl, r6
	bne	.L1754
	ldr	r3, .L1825+4
	add	r7, fp, #1
	uxth	r7, r7
	ldr	r3, [r3, #-984]
	ldr	r0, [r3, #4]
.L1817:
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	ldr	r3, .L1825
	cmp	r2, #1
	str	r0, [sp, #24]
	bne	.L1755
	ldrb	r1, [r3, #76]	@ zero_extendqisi2
	cmp	r1, #0
	addeq	r7, r3, r7, asl #1
	addeq	r7, r7, #2224
	ldreqh	r7, [r7, #0]
.L1755:
	movw	r1, #3912
	ldr	fp, [sp, #24]
	ldrh	r3, [r3, r1]
	ldr	ip, [sp, #32]
	cmp	r3, r7
	streqh	r7, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r4, #6]
	streqh	r3, [r4, #4]	@ movhi
	ldr	r3, [sp, #20]
	cmp	r7, r3
	cmpeq	fp, ip
	moveq	r0, r4
	moveq	r1, r7
	moveq	r2, fp
	beq	.L1823
	ldr	r3, [sp, #16]
	ldr	fp, [sp, #12]
	sub	r6, r3, #1
	movw	r3, #65535
	cmp	fp, r3
	bne	.L1758
	cmp	r2, #0
	bne	.L1759
.L1758:
	ldr	r2, .L1825+4
	uxth	r3, r5
	ldr	ip, [sp, #20]
	mvn	fp, #0
	mov	sl, r3
	mov	r9, fp
	ldr	r1, [r2, #-1012]
	ldr	r8, .L1825+4
	cmn	r1, #1
	streq	r6, [r2, #-1012]
	ldr	r2, [r2, #-1012]
	str	r2, [sp, #12]
	add	r2, ip, #7
	cmp	r3, r2
	subgt	r5, r3, #7
	mov	r3, r7
	ldrle	r5, [sp, #20]
	mov	r7, r6
	uxthgt	r5, r5
	b	.L1762
.L1764:
	ldrh	r0, [r1, #16]
	movw	lr, #65535
	cmp	r0, lr
	beq	.L1763
	ldr	ip, [sp, #16]
	mov	r3, #36
	orr	r0, r5, r0, asl #10
	mla	lr, r3, r6, ip
	add	r6, r6, #1
	uxth	r6, r6
	str	r0, [lr, #4]
.L1763:
	add	r2, r2, #1
	add	r1, r1, #2
	uxth	r2, r2
.L1775:
	ldr	lr, [sp, #44]
	cmp	r2, lr
	bne	.L1764
	ldr	r3, [sp, #36]
	mov	r1, r6
	ldr	r0, [r8, #-984]
	ldr	r2, [sp, #28]
	str	r3, [sp, #4]
	bl	FlashReadPages
	ldr	r2, .L1825
	add	r0, r8, r5, asl #1
	mov	r1, #0
	str	r0, [sp, #16]
	ldr	r3, [sp, #4]
	ldrb	ip, [r2, #76]	@ zero_extendqisi2
	ldr	r2, [r8, #-984]
	str	ip, [sp, #36]
	b	.L1765
.L1773:
	ldr	r0, [r2, #0]
	cmp	r0, #0
	bne	.L1766
	ldr	r0, [r2, #12]
	movw	ip, #65535
	ldrh	lr, [r0, #0]
	cmp	lr, ip
	beq	.L1767
	ldr	r0, [r0, #4]
	cmn	r0, #1
	beq	.L1767
	cmn	fp, #1
	ldr	r9, [r8, #-1012]
	str	r0, [r8, #-1012]
	bne	.L1767
	ldr	lr, [sp, #16]
	sub	r0, lr, #912
	ldrh	r0, [r0, #0]
	cmp	r0, ip
	bne	.L1768
	ldr	r0, [sp, #36]
	cmp	r0, #0
	beq	.L1767
.L1768:
	cmp	r9, r7
	mvneq	fp, #0
	movne	fp, r9
	b	.L1767
.L1766:
	mov	r6, r7
	mov	r7, r3
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1759
	ldr	r3, .L1825+4
	movw	r2, #65535
	add	r5, r3, r5, asl #1
	sub	r5, r5, #912
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	bne	.L1769
	cmn	fp, #1
	bne	.L1818
.L1770:
	ldr	fp, [sp, #12]
	cmp	fp, r6
	beq	.L1771
.L1818:
	str	fp, [r3, #-1012]
	b	.L1759
.L1771:
	ldr	r2, [r3, #-1012]
	b	.L1824
.L1769:
	cmp	r9, r6
	beq	.L1772
	cmn	r9, #1
	strne	r9, [r3, #-1012]
	b	.L1759
.L1772:
	ldr	r2, [r3, #-1012]
	cmp	r2, r6
	beq	.L1759
.L1824:
	sub	r2, r2, #1
	b	.L1819
.L1767:
	add	r1, r1, #1
	add	r2, r2, #36
	uxth	r1, r1
.L1765:
	cmp	r1, r6
	bne	.L1773
	add	r5, r5, #1
	uxth	r5, r5
.L1762:
	cmp	r5, sl
	bhi	.L1774
	ldr	r1, .L1825
	movw	r2, #3844
	ldr	lr, [r8, #-984]
	mov	r6, #0
	str	r3, [sp, #36]
	ldrh	ip, [r1, r2]
	mov	r1, r4
	str	lr, [sp, #16]
	mov	r2, r6
	str	ip, [sp, #44]
	b	.L1775
.L1774:
	mov	r6, r7
	mov	r7, r3
	ldr	r3, .L1825+4
	mvn	r2, #0
.L1819:
	str	r2, [r3, #-1012]
.L1759:
	ldr	r5, .L1825+4
	movw	r3, #266
	mov	r2, #1
	sub	r0, r5, #1088
	strh	r2, [r5, r3]	@ movhi
	bl	FtlMapBlkWriteDumpData
	ldr	r8, [sp, #20]
	str	r7, [sp, #16]
.L1776:
	ldr	r3, .L1825
	movw	r2, #3844
	mov	r7, #0
	ldr	lr, [r5, #-984]
	mov	fp, #36
	ldrh	sl, [r3, r2]
	mov	r2, r4
	ldrb	ip, [r3, #76]	@ zero_extendqisi2
	mov	r3, r7
	b	.L1777
.L1780:
	ldrh	r0, [r2, #16]
	movw	r1, #65535
	cmp	r0, r1
	beq	.L1778
	mla	r1, fp, r7, lr
	orr	r0, r8, r0, asl #10
	str	r0, [r1, #4]
	ldrb	r9, [r4, #8]	@ zero_extendqisi2
	cmp	r9, #1
	bne	.L1779
	cmp	ip, #0
	orrne	r0, r0, #-2147483648
	strne	r0, [r1, #4]
.L1779:
	add	r7, r7, #1
	uxth	r7, r7
.L1778:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1777:
	cmp	r3, sl
	bne	.L1780
	mov	r1, r7
	ldr	r0, [r5, #-984]
	ldr	r2, [sp, #28]
	bl	FlashReadPages
	mov	r3, #36
	mul	r3, r3, r7
	mov	r7, #0
	str	r3, [sp, #44]
	b	.L1781
.L1802:
	ldr	r9, [r5, #-984]
	add	r9, r9, r7
	ldr	sl, [r9, #4]
	ubfx	r0, sl, #10, #16
	str	sl, [sp, #60]
	bl	P2V_plane
	ldr	r3, [sp, #20]
	cmp	r8, r3
	bcc	.L1782
	ldr	fp, [sp, #32]
	mov	ip, r3
	cmp	r0, fp
	movcs	r3, #0
	movcc	r3, #1
	cmp	r8, ip
	movne	r3, #0
	cmp	r3, #0
	bne	.L1782
	ldr	r3, [sp, #24]
	ldr	ip, [sp, #16]
	cmp	r0, r3
	cmpeq	r8, ip
	beq	.L1814
	ldr	r3, [r9, #0]
	cmn	r3, #1
	beq	.L1784
	ldr	sl, [r9, #12]
	movw	lr, #61589
	ldrh	r3, [sl, #0]
	cmp	r3, lr
	bne	.L1791
.L1785:
	ldr	r6, [sl, #4]
	cmn	r6, #1
	beq	.L1786
	ldr	r1, [r5, #-1844]
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	addne	r3, r6, #1
	strne	r3, [r5, #-1844]
.L1786:
	ldr	r9, [sl, #8]
	add	r1, sp, #56
	ldr	r3, [sl, #12]
	mov	r2, #0
	mov	r0, r9
	str	r3, [sp, #52]
	bl	log2phys
	ldr	r1, [r5, #-1012]
	cmn	r1, #1
	beq	.L1787
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1787
	ldr	r3, [sp, #52]
	cmn	r3, #1
	beq	.L1788
	ldr	r0, [r5, #-984]
	mov	r1, #1
	mov	r2, #0
	add	r0, r0, r7
	ldr	fp, [r0, #12]
	str	r3, [r0, #4]
	str	fp, [sp, #12]
	bl	FlashReadPages
	ldr	r3, [r5, #-984]
	ldr	ip, [fp, #4]
	add	fp, r3, r7
	ldr	r3, [r3, r7]
	str	ip, [sp, #36]
	cmn	r3, #1
	bne	.L1789
	b	.L1790
.L1788:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	bne	.L1791
	mov	r0, r9
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
.L1791:
	ldrh	r0, [r4, #0]
	b	.L1821
.L1789:
	ldr	r3, [sp, #12]
	ldr	sl, [r3, #8]
	cmp	sl, r9
	bne	.L1790
	ldr	r0, [r5, #-1012]
	ldr	r1, [sp, #36]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1790
	ldr	r3, [sp, #56]
	ldr	r2, [sp, #60]
	cmp	r3, r2
	beq	.L1795
.L1792:
	ldr	r2, [sp, #52]
	cmp	r3, r2
	beq	.L1790
	cmn	r3, #1
	streq	r3, [fp, #0]
	beq	.L1794
	ldr	ip, [fp, #12]
	mov	r0, fp
	str	r3, [fp, #4]
	mov	r1, #1
	mov	r2, #0
	str	ip, [sp, #12]
	bl	FlashReadPages
.L1794:
	ldr	r3, [r5, #-984]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	beq	.L1795
	ldr	r3, [sp, #12]
	ldr	r0, [r5, #-1012]
	ldr	r9, [r3, #4]
	mov	r1, r9
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1795
	ldr	r0, [sp, #36]
	mov	r1, r9
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1790
.L1795:
	mov	r0, sl
	ldr	r1, [sp, #52]
	bl	FtlReUsePrevPpa
.L1790:
	ldrh	r0, [r4, #0]
	mvn	r3, #0
	str	r3, [sp, #52]
	bl	decrement_vpc_count
	b	.L1797
.L1787:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	beq	.L1797
	mov	r0, r9
	add	r1, sp, #60
	mov	r2, #1
	bl	log2phys
	ldr	fp, [sp, #56]
	cmn	fp, #1
	beq	.L1797
	ldr	r3, [sp, #52]
	cmp	fp, r3
	beq	.L1797
	ubfx	r0, fp, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L1825+8
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1798
	ldr	r3, .L1825+12
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1798
	ldr	r3, .L1825+16
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	bne	.L1797
.L1798:
	ldr	ip, .L1825+4
	mov	r1, #1
	mov	r2, #0
	ldr	r0, [ip, #-984]
	str	fp, [r0, #4]
	ldr	sl, [r0, #12]
	bl	FlashReadPages
	ldr	r0, .L1825+4
	ldr	r1, [sl, #4]
	ldr	r3, [r0, #-984]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L1797
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L1797
	mov	r0, r9
	add	r1, sp, #56
	mov	r2, #1
	bl	log2phys
.L1797:
	ldr	r0, [sp, #52]
	cmn	r0, #1
	beq	.L1782
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r5, #-2064]
	mov	r3, r0, asl #1
	mov	r1, r0
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1799
.L1821:
	bl	decrement_vpc_count
	b	.L1782
.L1799:
	ldr	r0, .L1825+20
	bl	printk
	b	.L1782
.L1784:
	ldr	r3, [r5, #268]
	cmp	r3, #31
	addls	r2, r5, r3, asl #2
	addls	r3, r3, #1
	strls	r3, [r5, #268]
	strls	sl, [r2, #272]
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
	ldr	r3, [r5, #-1012]
	cmn	r3, #1
	beq	.L1820
.L1801:
	cmp	r3, r6
	bls	.L1782
.L1820:
	str	r6, [r5, #-1012]
.L1782:
	add	r7, r7, #36
.L1781:
	ldr	r3, [sp, #44]
	cmp	r7, r3
	bne	.L1802
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	add	r8, r8, #1
	cmp	r3, #1
	uxth	r8, r8
	bne	.L1803
	ldr	r3, .L1825
	ldrb	r2, [r3, #76]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1803
	movw	r2, #3914
	ldr	ip, [sp, #16]
	ldrh	r3, [r3, r2]
	cmp	ip, r8
	cmpeq	r3, r8
	beq	.L1814
.L1803:
	ldr	r3, .L1825
	movw	r2, #3912
	ldrh	r2, [r3, r2]
	cmp	r8, r2
	bne	.L1776
	mov	r2, #0
	movw	r1, #3844
	strh	r2, [r4, #4]	@ movhi
	ldrh	r1, [r3, r1]
	mov	r3, r2
	strh	r8, [r4, #2]	@ movhi
	movw	r2, #65535
	ldr	r0, [sp, #40]
	b	.L1804
.L1806:
	add	r0, r0, #2
	ldrh	ip, [r0, #14]
	cmp	ip, r2
	strneb	r3, [r4, #6]
	bne	.L1738
.L1805:
	add	r3, r3, #1
	uxth	r3, r3
.L1804:
	cmp	r3, r1
	bne	.L1806
	b	.L1738
.L1814:
.L1783:
	ldr	r7, [sp, #16]
	mov	r0, r4
	ldr	r3, [sp, #24]
	mov	r1, r7
	strh	r7, [r4, #2]	@ movhi
	mov	r2, r3
	strb	r3, [r4, #6]
.L1823:
	bl	ftl_sb_update_avl_pages
.L1738:
	mov	r0, #0
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1826:
	.align	2
.L1825:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-2044
	.word	-1996
	.word	-1948
	.word	.LC118
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	2
	.global	allocate_data_superblock
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L1861
	mov	r5, r0
	mov	r7, r4
.L1857:
	ldr	r3, .L1861+4
	cmp	r5, r3
	bne	.L1829
	ldr	r3, .L1861+8
	ldrh	r2, [r4, r3]
	ldr	r3, [r4, #-1180]
	mov	r1, r2, lsr #1
	mul	ip, r3, r2
	add	r0, r1, #1
	add	r0, r0, ip, lsr #2
	ldr	ip, [r4, #-1884]
	cmp	ip, #0
	uxth	r0, r0
	beq	.L1851
	ldr	ip, [r4, #-1824]
	cmp	ip, #29
	bhi	.L1851
	cmp	ip, #2
	bls	.L1856
	tst	r2, #1
	beq	.L1830
	cmp	r3, #0
	moveq	r1, r3
	beq	.L1831
	b	.L1830
.L1829:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1856
	ldr	r3, .L1861+12
	movw	r2, #3864
	ldrh	r2, [r3, r2]
	cmp	r2, #1
	beq	.L1856
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1856
	sub	r3, r3, #2048
	ldr	r2, [r4, #-1884]
	ldrh	r3, [r4, r3]
	cmp	r2, #0
	mov	r1, r3, lsr #3
	beq	.L1830
	ldr	r2, [r4, #-1824]
	cmp	r2, #1
	movls	r1, #7
	mulls	r1, r1, r3
	movls	r1, r1, lsr #3
	b	.L1830
.L1851:
	mov	r1, r0
.L1830:
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
	b	.L1831
.L1856:
	mov	r1, #0
.L1831:
	ldr	r0, .L1861+16
	bl	List_pop_index_node
	ldr	r3, .L1861+8
	ldrh	r2, [r4, r3]
	sub	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	uxth	r6, r0
	mov	r0, r5
	strh	r6, [r5, #0]	@ movhi
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1832
	ldr	r2, .L1861+12
	movw	r3, #3844
	ldr	ip, [r4, #-2088]
	mov	fp, #0
	mov	sl, r5
	mov	r1, r5
	ldrh	r8, [r2, r3]
	mov	r0, fp
	mov	r3, ip
	mov	r2, fp
	stmia	sp, {r5, ip}
	b	.L1833
.L1832:
	ldr	r2, [r4, #-2064]
	mov	r3, r6, asl #1
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
	b	.L1858
.L1836:
	str	r0, [r3, #8]
	movw	r5, #65535
	str	r0, [r3, #12]
	ldrh	lr, [r1, #16]
	cmp	lr, r5
	beq	.L1835
	ldr	r5, [sp, #4]
	mov	ip, #36
	mov	lr, lr, asl #10
	mla	r9, ip, fp, r5
	add	fp, fp, #1
	uxth	fp, fp
	str	lr, [r9, #4]
.L1835:
	add	r2, r2, #1
	add	r3, r3, #36
	add	r1, r1, #2
	uxth	r2, r2
.L1833:
	cmp	r2, r8
	bne	.L1836
	ldr	r3, [r4, #-1884]
	ldr	r5, [sp, #0]
	cmp	r3, #0
	beq	.L1837
	ldr	r3, .L1861+20
	cmp	r5, r3
	bne	.L1837
	ldr	r2, [r4, #-2084]
	mov	r3, r6, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #30
	movhi	r3, #0
	strhib	r3, [r4, #-2036]
.L1837:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r2, [r4, #-2084]
	cmp	r3, #0
	mov	r3, r6, asl #1
	ldrh	r1, [r2, r3]
	bne	.L1838
	cmp	r1, #0
	ldrne	ip, .L1861+12
	movwne	r0, #3902
	moveq	r1, #2
	ldrneh	r0, [ip, r0]
	addne	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r4, #-1840]
	mov	r0, r6
	mov	r1, #0
	add	r3, r3, #1
	str	r3, [r4, #-1840]
	b	.L1860
.L1838:
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r4, #-1836]
	mov	r0, r6
	mov	r1, #1
	add	r3, r3, #1
	str	r3, [r4, #-1836]
.L1860:
	bl	ftl_set_blk_mode
	ldr	r3, [r4, #-2084]
	mov	r9, r6, asl #1
	ldr	r2, [r4, #-1828]
	ldr	r0, [r7, #-1840]
	mov	r8, #0
	ldrh	r3, [r3, r9]
	cmp	r3, r2
	movw	r2, #3902
	strhi	r3, [r4, #-1828]
	ldr	r3, .L1861+12
	ldrh	r1, [r3, r2]
	ldr	r2, [r7, #-1836]
	mla	r0, r0, r1, r2
	movw	r2, #3852
	ldrh	r1, [r3, r2]
	bl	__aeabi_uidiv
	ldr	r3, [r7, #-936]
	ldr	r2, [r3, #16]
	add	r2, r2, #1
	str	r2, [r3, #16]
	ldr	r3, [r7, #-2088]
	str	r0, [r7, #-1832]
	b	.L1843
.L1844:
	add	r8, r8, #1
	ldr	r2, [r3, #-32]
	uxth	r8, r8
	bic	r2, r2, #1020
	bic	r2, r2, #3
	str	r2, [r3, #-32]
.L1843:
	cmp	r8, fp
	add	r3, r3, #36
	bne	.L1844
	mov	r2, r8
	ldr	r0, [r4, #-2088]
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	mov	fp, #0
	bl	FlashEraseBlocks
	mov	r2, #36
	mul	r2, r2, r8
	mov	r3, r6
	mov	ip, r9
	mov	r8, fp
	mov	r6, r5
	mov	r9, r2
	b	.L1845
.L1847:
	ldr	r1, [r4, #-2088]
	add	r0, r1, fp
	ldr	r5, [r1, fp]
	cmn	r5, #1
	bne	.L1846
	ldr	r0, [r0, #4]
	add	r8, r8, #1
	str	r3, [sp, #12]
	ubfx	r0, r0, #10, #16
	str	ip, [sp, #8]
	bl	FtlBbmMapBadBlock
	strh	r5, [sl, #16]	@ movhi
	ldrb	r1, [r6, #7]	@ zero_extendqisi2
	sub	r1, r1, #1
	strb	r1, [r6, #7]
	ldr	ip, [sp, #8]
	ldr	r3, [sp, #12]
.L1846:
	add	fp, fp, #36
	add	sl, sl, #2
.L1845:
	cmp	fp, r9
	bne	.L1847
	cmp	r8, #0
	mov	r5, r6
	mov	r9, ip
	mov	r6, r3
	beq	.L1848
	mov	r0, r3
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1848:
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1849
	ldr	r3, [r7, #-2064]
	mvn	r2, #0
	strh	r2, [r3, r9]	@ movhi
.L1858:
	mov	r0, r6
	bl	INSERT_DATA_LIST
	b	.L1857
.L1849:
	ldr	r1, .L1861+12
	movw	r2, #3912
	mov	r0, #0
	strh	r0, [r5, #2]	@ movhi
	strb	r0, [r5, #6]
	ldrh	r2, [r1, r2]
	strh	r6, [r5, #0]	@ movhi
	mul	r2, r2, r3
	ldr	r3, .L1861
	ldr	r1, [r3, #-1848]
	uxth	r2, r2
	strh	r2, [r5, #4]	@ movhi
	str	r1, [r5, #12]
	add	r1, r1, #1
	str	r1, [r3, #-1848]
	ldrh	ip, [r5, #0]
	ldr	r1, [r3, #-2064]
	mov	r3, ip, asl #1
	strh	r2, [r1, r3]	@ movhi
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1862:
	.align	2
.L1861:
	.word	.LANCHOR2
	.word	.LANCHOR2-1948
	.word	-2048
	.word	.LANCHOR0
	.word	.LANCHOR2-2052
	.word	.LANCHOR2-2044
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1870
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r4, r0
	ldrb	r7, [r3, #76]	@ zero_extendqisi2
	.pad #40
	sub	sp, sp, #40
	cmp	r7, #0
	beq	.L1864
	ldrb	r7, [r0, #8]	@ zero_extendqisi2
	sub	r0, r7, #1
	rsbs	r7, r0, #0
	adc	r7, r7, r0
.L1864:
	mov	r8, #7
	mvn	r6, #0
	ldr	r5, .L1870+4
	b	.L1865
.L1868:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1866
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [sp, #8]
	beq	.L1866
	ldr	r2, [r5, #-952]
	add	r0, sp, #4
	ldr	r3, [r5, #-2076]
	str	r6, [sp, #20]
	str	r2, [sp, #16]
	str	r3, [sp, #12]
	str	r6, [r2, #12]
	str	r6, [r2, #8]
	ldrh	r3, [r4, #0]
	strh	r3, [r2, #2]	@ movhi
	mov	r3, #0
	strh	r3, [r2, #0]	@ movhi
	ldr	r1, [r5, #-1844]
	cmn	r1, #2
	str	r1, [r2, #4]
	addne	r1, r1, #1
	moveq	r1, #0
	mov	r2, r7
	str	r1, [r5, #-1844]
	mov	r1, #1
	bl	FlashProgPages
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
.L1865:
	subs	r8, r8, #1
	bne	.L1868
.L1866:
	ldr	r2, .L1870+4
	ldrh	r3, [r4, #0]
	ldrh	r1, [r4, #4]
	ldr	r2, [r2, #-2064]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldr	r2, .L1870
	movw	r3, #3912
	ldrh	r3, [r2, r3]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1871:
	.align	2
.L1870:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlWriteDumpData
	.type	FtlWriteDumpData, %function
FtlWriteDumpData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r4, .L1881
	ldr	r3, .L1881+4
	mov	r2, r4
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	beq	.L1873
	ldrb	r1, [r4, #-2036]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L1873
	ldr	r5, .L1881+8
	movw	r0, #3912
	ldrb	r1, [r4, #-2037]	@ zero_extendqisi2
	ldrh	r0, [r5, r0]
	mul	r1, r0, r1
	cmp	r3, r1
	beq	.L1873
	ldrb	r8, [r4, #-2034]	@ zero_extendqisi2
	movw	r3, #3844
	ldr	r7, [r5, #3972]
	cmp	r8, #0
	ldrh	r6, [r5, r3]
	bne	.L1872
	sub	r7, r7, #1
	mov	r1, sp
	mov	r2, r8
	mov	r0, r7
	bl	log2phys
	ldr	r3, [sp, #0]
	ldr	r0, [r4, #-2076]
	ldr	r4, [r4, #-952]
	cmn	r3, #1
	str	r3, [sp, #8]
	str	r7, [sp, #20]
	str	r0, [sp, #12]
	str	r4, [sp, #16]
	str	r8, [r4, #4]
	beq	.L1875
	add	r0, sp, #4
	mov	r1, #1
	mov	r2, r8
	bl	FlashReadPages
	b	.L1876
.L1875:
	movw	r3, #3922
	mov	r1, #255
	ldrh	r2, [r5, r3]
	bl	ftl_memset
.L1876:
	ldr	r3, .L1881+12
	mov	r6, r6, asl #2
	ldr	r5, .L1881
	ldr	sl, .L1881+4
	strh	r3, [r4, #0]	@ movhi
	ldr	r8, .L1881+16
	b	.L1877
.L1880:
	ldrh	r3, [r5, sl]
	cmp	r3, #0
	beq	.L1878
	ldr	r3, [sp, #8]
	sub	r6, r6, #1
	str	r7, [r4, #8]
	ldr	r0, .L1881+20
	str	r3, [r4, #12]
	ldrh	r3, [r5, r8]
	strh	r3, [r4, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r5, #-1844]
	mov	r2, #0
	mov	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r4, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r5, #-1844]
	moveq	r3, #0
	streq	r3, [r5, #-1844]
	mov	r3, r2
	bl	FlashProgPages
	ldrh	r0, [r5, r8]
	bl	decrement_vpc_count
.L1877:
	cmp	r6, #0
	bne	.L1880
.L1878:
	ldr	r3, .L1881
	mov	r2, #1
	strb	r2, [r3, #-2034]
	b	.L1872
.L1873:
	mov	r3, #0
	strb	r3, [r2, #-2034]
.L1872:
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1882:
	.align	2
.L1881:
	.word	.LANCHOR2
	.word	-2040
	.word	.LANCHOR0
	.word	-3947
	.word	-2044
	.word	.LANCHOR2-2044
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
	ldr	r7, .L1887
	movw	r6, #3950
	ldr	r5, .L1887+4
	b	.L1884
.L1886:
	ldr	r3, [r5, #-1900]
	mov	r2, #12
	mla	r3, r2, r4, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L1885
	mov	r0, r4
	bl	flush_l2p_region
.L1885:
	add	r4, r4, #1
	uxth	r4, r4
.L1884:
	ldrh	r3, [r7, r6]
	cmp	r3, r4
	bhi	.L1886
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1888:
	.align	2
.L1887:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r4, #0
	ldr	r6, .L1896
	ldr	r1, .L1896+4
	ldr	r0, .L1896+8
	bl	printk
	movw	r3, #3854
	ldrh	r2, [r6, r3]
	mov	r1, #0
	ldr	r5, .L1896+12
	mov	r2, r2, asl #1
	ldr	r0, [r5, #-932]
	bl	ftl_memset
	b	.L1890
.L1892:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L1891
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r5, #-932]
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	add	r2, r2, #1
	strh	r2, [r3, r0]	@ movhi
.L1891:
	add	r4, r4, #1
.L1890:
	ldr	r3, [r6, #3972]
	cmp	r4, r3
	bcc	.L1892
	mov	r4, #0
	ldr	r7, .L1896
	movw	r8, #3852
	ldr	r5, .L1896+12
	b	.L1893
.L1895:
	ldr	r3, [r5, #-2064]
	mov	r6, r4, asl #1
	ldrh	r2, [r3, r6]
	ldr	r3, [r5, #-932]
	ldrh	r3, [r3, r6]
	cmp	r2, r3
	beq	.L1894
	movw	r1, #65535
	cmp	r2, r1
	beq	.L1894
	ldr	r0, .L1896+16
	mov	r1, r4
	bl	printk
	ldr	r3, .L1896+20
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1894
	ldr	r3, .L1896+24
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1894
	ldr	r3, .L1896+28
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1894
	ldr	r3, [r5, #-932]
	mov	r0, r4
	ldrh	r2, [r3, r6]
	ldr	r3, [r5, #-2064]
	strh	r2, [r3, r6]	@ movhi
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1894:
	add	r4, r4, #1
	uxth	r4, r4
.L1893:
	ldrh	r3, [r7, r8]
	cmp	r3, r4
	bhi	.L1895
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1897:
	.align	2
.L1896:
	.word	.LANCHOR0
	.word	.LANCHOR3+15
	.word	.LC119
	.word	.LANCHOR2
	.word	.LC120
	.word	-2044
	.word	-1948
	.word	-1996
	.fnend
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	allocate_new_data_superblock
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movw	r3, #65535
	ldrh	r5, [r0, #0]
	mov	r4, r0
	cmp	r5, r3
	beq	.L1899
	ldr	r3, .L1909
	mov	r0, r5
	ldr	r2, [r3, #-2064]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1900
	bl	INSERT_DATA_LIST
	b	.L1899
.L1900:
	bl	INSERT_FREE_LIST
.L1899:
	mov	r3, #0
	strb	r3, [r4, #8]
	ldr	r3, .L1909+4
	cmp	r4, r3
	beq	.L1901
	ldr	r2, .L1909+8
	movw	r3, #3864
	ldrh	r3, [r2, r3]
	cmp	r3, #1
	beq	.L1901
	ldrb	r2, [r2, #76]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1902
.L1901:
	mov	r3, #1
	strb	r3, [r4, #8]
	b	.L1903
.L1902:
	ldr	r2, .L1909+12
	cmp	r4, r2
	bne	.L1903
	cmp	r3, #3
	ldr	r3, .L1909
	beq	.L1904
	ldr	r2, [r3, #-1708]
	cmp	r2, #1
	bne	.L1905
.L1904:
	mov	r2, #1
	strb	r2, [r3, #-2036]
.L1905:
	ldr	r3, [r3, #-1884]
	ldr	r2, .L1909
	cmp	r3, #0
	beq	.L1903
	ldr	r3, [r2, #-1824]
	cmp	r3, #29
	movls	r3, #1
	strlsb	r3, [r2, #-2036]
.L1903:
	ldr	r3, .L1909+16
	ldr	r6, .L1909
	ldrh	r0, [r6, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1906
	cmp	r5, r0
	bne	.L1907
	ldr	r2, [r6, #-2064]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1908
.L1907:
	bl	update_vpc_list
.L1908:
	ldr	r3, .L1909+16
	mvn	r2, #0
	strh	r2, [r6, r3]	@ movhi
.L1906:
	mov	r0, r4
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1910:
	.align	2
.L1909:
	.word	.LANCHOR2
	.word	.LANCHOR2-1996
	.word	.LANCHOR0
	.word	.LANCHOR2-2044
	.word	-1024
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1933
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	ldr	r4, [r3, #400]
	ldr	r3, .L1933+4
	ldr	r1, [r3, #3964]
	cmp	r1, #0
	beq	.L1912
	ldrb	r8, [r3, #76]	@ zero_extendqisi2
	cmp	r8, #0
	beq	.L1913
	ldrb	r8, [r4, #8]	@ zero_extendqisi2
	sub	ip, r8, #1
	rsbs	r8, ip, #0
	adc	r8, r8, ip
.L1913:
	ldr	r0, [r3, #3968]
	mov	r2, r8
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	mov	r5, #0
	bl	FlashProgPages
	mov	r7, r5
	ldr	r6, .L1933+4
	ldr	r9, .L1933
	b	.L1914
.L1919:
	ldr	r2, [r6, #3968]
	add	r3, r2, r5
	ldr	r2, [r2, r5]
	cmn	r2, #1
	beq	.L1932
	ldr	r2, [r3, #4]
	cmp	r8, #0
	ldr	r0, [r3, #16]
	add	r1, sp, #4
	str	r2, [sp, #4]
	orrne	r2, r2, #-2147483648
	strne	r2, [sp, #4]
	mov	r2, #1
	bl	log2phys
	ldr	r3, [r6, #3968]
	add	r3, r3, r5
	ldr	r3, [r3, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L1917
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r9, #-2064]
	mov	r3, r0, asl #1
	mov	sl, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L1918
	ldr	r0, .L1933+8
	mov	r1, sl
	bl	printk
.L1918:
	mov	r0, sl
	bl	decrement_vpc_count
.L1917:
	add	r7, r7, #1
	add	r5, r5, #36
.L1914:
	ldr	r3, [r6, #3964]
	cmp	r7, r3
	bcc	.L1919
	b	.L1920
.L1928:
	ldr	r3, [r6, #3968]
	mvn	r2, #0
	movw	sl, #3912
	str	r2, [r3, r5]
	b	.L1921
.L1924:
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #0]
	cmp	r3, r0
	bne	.L1922
	ldr	r2, [r7, #-2064]
	mov	r3, r3, asl #1
	ldrh	r1, [r4, #4]
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r6, sl]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
.L1922:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1923
	mov	r0, r4
	bl	allocate_new_data_superblock
.L1923:
	ldr	r3, [r7, #-1608]
	add	r3, r3, #1
	str	r3, [r7, #-1608]
	ldr	r3, [r6, #3968]
	add	r3, r3, r5
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlGcMarkBadPhyBlk
	mov	r0, r4
	bl	get_new_active_ppa
	mov	r1, #1
	mov	r2, r8
	mov	r3, r0
	str	r0, [sp, #4]
	ldr	r0, [r6, #3968]
	add	r0, r0, r5
	str	r3, [r0, #4]
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	bl	FlashProgPages
.L1921:
	ldr	r2, [r6, #3968]
	add	r3, r2, r5
	ldr	r2, [r2, r5]
	cmn	r2, #1
	beq	.L1924
	ldr	r2, [r3, #4]
	cmp	r8, #0
	ldr	r0, [r3, #16]
	add	r1, sp, #4
	str	r2, [sp, #4]
	orrne	r2, r2, #-2147483648
	strne	r2, [sp, #4]
	mov	r2, #1
	bl	log2phys
	ldr	r3, [r6, #3968]
	add	r3, r3, r5
	ldr	r3, [r3, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L1926
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r7, #-2064]
	mov	r3, r0, asl #1
	mov	sl, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L1927
	ldr	r0, .L1933+8
	mov	r1, sl
	bl	printk
.L1927:
	mov	r0, sl
	bl	decrement_vpc_count
.L1926:
	add	r9, r9, #1
	add	r5, r5, #36
	b	.L1915
.L1932:
	ldr	r6, .L1933+4
	mov	r5, #0
	ldr	r7, .L1933
	mov	r9, r5
.L1915:
	ldr	r3, [r6, #3964]
	cmp	r9, r3
	bcc	.L1928
	movw	r4, #16386
	ldr	r6, .L1933
	ldr	r5, .L1933+12
	b	.L1929
.L1930:
	mov	r0, #1
	mov	r1, r0
	bl	rk_ftl_garbage_collect
	subs	r4, r4, #1
	beq	.L1920
.L1929:
	ldrh	r3, [r6, r5]
	cmp	r3, #0
	bne	.L1930
.L1920:
	ldr	r3, .L1933+4
	mov	r2, #0
	str	r2, [r3, #3964]
.L1912:
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L1934:
	.align	2
.L1933:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC121
	.word	-1134
	.fnend
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
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
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	movw	r3, #3912
	ldr	r2, .L1958
	ldrh	r1, [r2, r3]
	ldr	r2, .L1958+4
	ldr	r3, .L1958+8
	ldrh	r4, [r3, r2]
	movw	r2, #65535
	cmp	r4, r2
	beq	.L1937
	cmp	r0, #0
	beq	.L1938
	ldr	ip, .L1958+12
	movw	r0, #3000
	ldrh	lr, [ip, r0]
	cmp	lr, r2
	movne	r1, #2
	bne	.L1938
	mov	r2, #0
	strh	r2, [ip, r0]	@ movhi
	sub	r2, r2, #2048
	ldrh	r3, [r3, r2]
	cmp	r3, #17
	movhi	r1, #2
.L1938:
	ldr	r0, .L1958+16
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #4]
	beq	.L1939
	ldr	r3, .L1958+8
	mov	r4, r4, asl #1
	ldr	r3, [r3, #-2084]
	ldrh	r2, [r3, r4]
	cmp	r2, #4
	bls	.L1940
	sub	r2, r2, #5
	mov	r0, #1
	strh	r2, [r3, r4]	@ movhi
	bl	FtlEctTblFlush
.L1940:
	ldr	r3, .L1958+8
	ldr	r2, [r3, #-1016]
	cmp	r2, #0
	bne	.L1941
	ldr	r2, [r3, #-1608]
	ldr	r0, [sp, #4]
	add	r2, r2, #1
	str	r2, [r3, #-1608]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L1941:
	ldr	r3, .L1958+8
	mov	r2, #0
	str	r2, [r3, #-1016]
	b	.L1954
.L1939:
	ldr	r2, .L1958+12
	movw	r3, #3000
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1954
.L1937:
	ldr	r6, .L1958+8
	movw	r3, #65535
	ldr	r5, .L1958+4
	mov	r4, #0
	str	r4, [r6, #-1016]
	ldrh	r2, [r6, r5]
	cmp	r2, r3
	moveq	r0, r4
	beq	.L1942
	bl	FtlCacheWriteBack
	ldr	ip, .L1958
	movw	r0, #3912
	ldrb	r1, [r6, #-1941]	@ zero_extendqisi2
	ldrh	r3, [r6, r5]
	ldrh	r0, [ip, r0]
	ldr	r2, [r6, #-2064]
	ldr	sl, .L1958+20
	mov	r3, r3, asl #1
	mul	r1, r0, r1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r6, #-1860]
	ldrh	r2, [r6, sl]
	add	r3, r2, r3
	str	r3, [r6, #-1860]
	b	.L1943
.L1946:
	mov	r7, #12
	ldr	r8, [r6, #-1144]
	mul	r7, r7, r4
	add	r1, sp, #4
	mov	r2, #0
	add	r5, r8, r7
	ldr	r0, [r5, #8]
	bl	log2phys
	ldr	r0, [sp, #4]
	ldr	r3, [r8, r7]
	cmp	r0, r3
	bne	.L1944
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r5, #4
	mov	r2, #1
	mov	r7, r0
	ldr	r0, [r5, #8]
	bl	log2phys
	mov	r0, r7
	b	.L1957
.L1944:
	ldr	r3, [r5, #4]
	cmp	r0, r3
	beq	.L1945
	ldr	r3, .L1958+4
	ldrh	r0, [r6, r3]
.L1957:
	bl	decrement_vpc_count
.L1945:
	add	r4, r4, #1
	uxth	r4, r4
.L1943:
	ldrh	r3, [r6, sl]
	ldr	r5, .L1958+8
	cmp	r3, r4
	bhi	.L1946
	movw	r0, #65535
	ldr	r4, .L1958+4
	bl	decrement_vpc_count
	ldr	r3, .L1958
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1947
	ldr	r0, .L1958+24
	ldrh	r1, [r5, r4]
	bl	printk
.L1947:
	ldrh	r0, [r5, r4]
	ldr	r2, [r5, #-2064]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1948
	bl	INSERT_DATA_LIST
	b	.L1949
.L1948:
	bl	INSERT_FREE_LIST
.L1949:
	ldr	r4, .L1958+8
	mvn	r2, #0
	ldr	r3, .L1958+4
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #0
	ldr	r2, .L1958+20
	strh	r3, [r4, r2]	@ movhi
	ldr	r2, .L1958+28
	strh	r3, [r4, r2]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	r3, [r4, #-1884]
	mov	r1, r4
	ldr	r2, .L1958+32
	cmp	r3, #0
	beq	.L1950
	ldr	r3, [r4, #-1824]
	cmp	r3, #29
	bhi	.L1950
	ldrh	r3, [r4, r2]
	sub	r2, r2, #276
	mvn	r1, #0
	ldrh	r2, [r4, r2]
	cmp	r2, r3
	movcc	r3, r3, asl #1
	ldrcc	r2, .L1958+36
	strcch	r3, [r4, r2]	@ movhi
	ldr	r3, .L1958+40
	ldr	r2, .L1958+8
	strh	r1, [r2, r3]	@ movhi
	b	.L1956
.L1950:
	ldrh	r2, [r1, r2]
	ldr	r0, .L1958+44
	ldr	r3, .L1958+8
	ldrh	r1, [r1, r0]
	add	r0, r2, r2, asl #1
	cmp	r1, r0, lsr #2
	ble	.L1956
	ldr	r1, .L1958+40
	mvn	r0, #0
	strh	r0, [r3, r1]	@ movhi
	ldr	r1, .L1958
	ldrb	r0, [r1, #76]	@ zero_extendqisi2
	ldr	r1, .L1958+36
	cmp	r0, #0
	subne	r2, r2, #2
	moveq	r2, #20
	strneh	r2, [r3, r1]	@ movhi
	streqh	r2, [r3, r1]	@ movhi
	beq	.L1942
	b	.L1956
.L1954:
	mov	r0, #1
	b	.L1942
.L1956:
	mov	r0, #0
.L1942:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L1959:
	.align	2
.L1958:
	.word	.LANCHOR0
	.word	-1948
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR2-1948
	.word	-1140
	.word	.LC122
	.word	-1152
	.word	-1772
	.word	-1192
	.word	-1756
	.word	-2048
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldr	r3, .L1969
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	ldr	r3, .L1969+4
	beq	.L1961
	ldr	r2, [r3, #-1176]
	tst	r2, #1
	beq	.L1961
	ldr	r2, .L1969+8
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	bne	.L1966
.L1961:
	mov	r2, #0
	ldr	r0, [r3, #-980]
	ldr	r1, [r3, #-1176]
	mov	r3, r2
	bl	FlashProgPages
	mov	r4, #0
	ldr	r5, .L1969+4
	mov	r6, #36
	b	.L1963
.L1965:
	mul	r3, r6, r4
	ldr	r1, [r5, #-980]
	add	r2, r1, r3
	ldr	r1, [r1, r3]
	cmn	r1, #1
	bne	.L1964
	ldr	r2, .L1969+12
	mov	lr, #0
	ldr	ip, [r7, #-2064]
	ldrh	r0, [r7, r2]
	mov	r0, r0, asl #1
	strh	lr, [ip, r0]	@ movhi
	strh	r1, [r7, r2]	@ movhi
	ldr	r2, [r7, #-1608]
	add	r2, r2, #1
	str	r2, [r7, #-1608]
	ldr	r2, [r7, #-980]
	add	r3, r2, r3
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	b	.L1968
.L1964:
	ldr	r3, [r2, #12]
	add	r4, r4, #1
	ldr	r1, [r2, #4]
	uxth	r4, r4
	ldr	r0, [r3, #12]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
.L1963:
	ldr	r1, [r5, #-1176]
	ldr	r7, .L1969+4
	cmp	r4, r1
	bcc	.L1965
	ldr	r0, [r7, #-980]
	bl	FtlGcBufFree
	ldr	r3, .L1969+8
	mov	r0, #0
	str	r0, [r7, #-1176]
	ldrh	r3, [r7, r3]
	cmp	r3, r0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r0, #1
	bl	FtlGcFreeTempBlock
.L1968:
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1966:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1970:
	.align	2
.L1969:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1944
	.word	-1948
	.fnend
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movw	r5, #3912
	ldr	r6, .L1973
	ldr	r0, .L1973+4
	ldr	r4, .L1973+8
	ldrh	r1, [r6, r5]
	bl	FtlGcScanTempBlk
	ldr	r3, .L1973+12
	ldrh	r2, [r4, r3]
	ldrh	r3, [r6, r5]
	cmp	r2, r3
	ldmccfd	sp!, {r4, r5, r6, pc}
	sub	r0, r4, #1088
	bl	FtlMapBlkWriteDumpData
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	mov	r3, #0
	str	r3, [r4, #-1016]
	ldmfd	sp!, {r4, r5, r6, pc}
.L1974:
	.align	2
.L1973:
	.word	.LANCHOR0
	.word	.LANCHOR2-1948
	.word	.LANCHOR2
	.word	-1946
	.fnend
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, #0
	ldr	r3, .L1976
	ldr	r5, .L1976+4
	str	r4, [r3, #268]
	mov	r0, r5
	bl	FtlRecoverySuperblock
	mov	r0, r5
	add	r5, r5, #48
	bl	FtlSlcSuperblockCheck
	mov	r0, r5
	bl	FtlRecoverySuperblock
	mov	r0, r5
	bl	FtlSlcSuperblockCheck
	bl	FtlGcPageRecovery
	movw	r0, #65535
	bl	decrement_vpc_count
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1977:
	.align	2
.L1976:
	.word	.LANCHOR2
	.word	.LANCHOR2-2044
	.fnend
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movw	r3, #266
	ldr	r5, .L1989
	mov	r2, #0
	ldr	r4, .L1989+4
	ldr	r0, [r5, #3848]
	strh	r2, [r4, r3]	@ movhi
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldr	r3, .L1989+8
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1987
	bl	FtlLoadSysInfo
	subs	r6, r0, #0
	bne	.L1987
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	r0, #1
	bl	FtlUpdateVaildLpn
	movw	r3, #3950
	ldrh	r2, [r5, r3]
	mov	r3, r6
	ldr	r1, [r4, #-1900]
	b	.L1980
.L1982:
	add	r0, r1, r6
	add	r6, r6, #12
	ldr	r0, [r0, #4]
	cmp	r0, #0
	blt	.L1981
	add	r3, r3, #1
.L1980:
	cmp	r3, r2
	blt	.L1982
.L1981:
	ldr	r4, .L1989+4
	cmp	r3, r2
	ldr	r1, .L1989+12
	ldrh	r0, [r4, r1]
	add	r0, r0, #1
	strh	r0, [r4, r1]	@ movhi
	blt	.L1983
	movw	r3, #266
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	beq	.L1984
.L1983:
	ldr	r0, .L1989+16
	bl	FtlSuperblockPowerLostFix
	ldr	r0, .L1989+20
	bl	FtlSuperblockPowerLostFix
	ldr	r3, .L1989+24
	ldr	r1, [r4, #-2064]
	ldr	r2, .L1989+28
	ldrh	r3, [r4, r3]
	ldrh	r0, [r4, r2]
	mov	r3, r3, asl #1
	ldrh	ip, [r1, r3]
	rsb	r0, r0, ip
	strh	r0, [r1, r3]	@ movhi
	ldr	r0, .L1989
	movw	r1, #3912
	ldr	r3, .L1989+32
	ldr	lr, [r4, #-2064]
	ldrh	ip, [r0, r1]
	strh	ip, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #44
	strb	r3, [r4, #-2038]
	ldrh	ip, [r4, r2]
	add	r2, r2, #4
	ldrh	r5, [r4, r2]
	mov	ip, ip, asl #1
	ldrh	r6, [lr, ip]
	rsb	r5, r5, r6
	strh	r5, [lr, ip]	@ movhi
	ldrh	r0, [r0, r1]
	ldr	r1, .L1989+36
	strh	r3, [r4, r2]	@ movhi
	strb	r3, [r4, #-1990]
	strh	r0, [r4, r1]	@ movhi
	ldr	r0, .L1989+40
	bl	FtlMapBlkWriteDumpData
	ldr	r0, .L1989+44
	bl	FtlMapBlkWriteDumpData
	ldr	r3, .L1989+48
	ldrh	r2, [r4, r3]
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L1984:
	ldr	r3, .L1989+24
	ldr	r4, .L1989+4
	ldrh	r0, [r4, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1985
	ldr	r3, .L1989+28
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1985
	ldr	r3, .L1989+52
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1985
	bl	FtlGcRefreshBlock
	ldr	r3, .L1989+56
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshBlock
	ldr	r0, .L1989+16
	bl	allocate_new_data_superblock
	ldr	r0, .L1989+20
	bl	allocate_new_data_superblock
	add	r0, r4, #224
	bl	FtlMapBlkWriteDumpData
.L1985:
	ldr	r3, .L1989+12
	ldr	r2, .L1989+4
	ldrh	r0, [r2, r3]
	ands	r4, r0, #31
	bne	.L1988
	bl	FtlVpcCheckAndModify
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1987:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1988:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1990:
	.align	2
.L1989:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1768
	.word	-1792
	.word	.LANCHOR2-2044
	.word	.LANCHOR2-1996
	.word	-2044
	.word	-2040
	.word	-2042
	.word	-1994
	.word	.LANCHOR2-1088
	.word	.LANCHOR2+224
	.word	-1790
	.word	-1992
	.word	-1996
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L1994
	ldr	r5, .L1994+4
	ldrh	r2, [r5, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1992
	ldr	r3, .L1994+8
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L1993
.L1992:
	bl	FtlCacheWriteBack
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L1994+12
	mov	r4, #0
	strb	r4, [r5, #-1940]
	bl	allocate_data_superblock
	ldr	r3, .L1994+16
	strh	r4, [r5, r3]	@ movhi
	add	r3, r3, #12
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	mov	r0, r4
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L1993:
	ldr	r0, .L1994+12
	ldmfd	sp!, {r3, r4, r5, lr}
	b	get_new_active_ppa
.L1995:
	.align	2
.L1994:
	.word	-1948
	.word	.LANCHOR2
	.word	-1944
	.word	.LANCHOR2-1948
	.word	-1152
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.type	rk_ftl_garbage_collect.part.17, %function
rk_ftl_garbage_collect.part.17:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #36
	sub	sp, sp, #36
	ldr	r4, .L2093
	mov	r7, r0
	ldr	r3, .L2093+4
	movw	r0, #65535
	str	r1, [sp, #20]
	ldrh	r1, [r4, r3]
	cmp	r1, r0
	beq	.L1997
	ldr	r2, .L2093+8
	ldrh	ip, [r4, r2]
	cmp	ip, r0
	streqh	r1, [r4, r2]	@ movhi
	mvneq	r2, #0
	streqh	r2, [r4, r3]	@ movhi
.L1997:
	bl	FtlReadRefresh
	ldr	r3, [r4, #-1188]
	cmp	r7, #1
	add	r3, r3, #1
	add	r3, r3, r7, asl #7
	str	r3, [r4, #-1188]
	bne	.L1998
	ldr	r2, .L2093
	ldr	r2, [r2, #-1884]
	cmp	r2, #0
	bne	.L1999
	ldr	r2, .L2093+12
	ldrb	r2, [r2, #76]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1998
.L1999:
	ldr	r4, .L2093
	ldr	r2, [r4, #-1824]
	cmp	r2, #29
	bhi	.L1998
	mov	r2, #404
	ldrh	r2, [r4, r2]
	add	r3, r3, r2
	str	r3, [r4, #-1188]
	bl	FtlGcReFreshBadBlk
	ldr	r3, .L2093+16
	movw	r2, #65535
	ldrh	r3, [r4, r3]
	cmp	r3, r2
	bne	.L1998
	ldr	r2, .L2093+8
	ldrh	r2, [r4, r2]
	cmp	r2, r3
	bne	.L1998
	ldr	r3, [r4, #-1188]
	cmp	r3, #1024
	bhi	.L2000
	ldr	r3, .L2093+20
	ldrh	r3, [r4, r3]
	cmp	r3, #63
	bhi	.L1998
.L2000:
	ldr	r3, .L2093
	mov	r2, #404
	mov	r1, #0
	str	r1, [r3, #-1188]
	strh	r1, [r3, r2]	@ movhi
	ldr	r1, [r3, #-1824]
	cmp	r1, #0
	moveq	r1, #6
	beq	.L2089
	cmp	r1, #5
	bhi	.L2002
	mov	r1, #18
.L2089:
	strh	r1, [r3, r2]	@ movhi
.L2002:
	ldr	r2, .L2093+12
	ldrb	r2, [r2, #76]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2003
	ldr	r2, .L2093+20
	ldr	r1, .L2093+24
	ldrh	r2, [r3, r2]
	ldrh	r3, [r3, r1]
	add	r3, r3, #64
	cmp	r2, r3
	bgt	.L1998
.L2003:
	mov	r0, #32
	movw	r8, #65535
	bl	List_get_gc_head_node
	uxth	r2, r0
	cmp	r2, r8
	beq	.L2004
	ldr	r5, .L2093
	ldr	r6, .L2093+28
	ldrh	r0, [r5, r6]
	cmp	r0, #0
	moveq	r3, #1
	streqh	r3, [r5, r6]	@ movhi
	beq	.L2004
	ldr	r3, .L2093+12
	movw	ip, #3914
	movw	lr, #3844
	ldr	r9, [r5, #-2064]
	mov	r2, r2, asl #1
	ldrh	ip, [r3, ip]
	ldrh	r3, [r3, lr]
	ldrh	r1, [r9, r2]
	mul	r3, r3, ip
	add	r3, r3, #1
	cmp	r1, r3
	bgt	.L2004
	add	fp, r0, #1
	mov	sl, #0
	str	sl, [r5, #-1180]
	uxth	fp, fp
	strh	fp, [r5, r6]	@ movhi
	str	r2, [sp, #12]
	bl	List_get_gc_head_node
	ldr	r2, [sp, #12]
	uxth	r4, r0
	cmp	r4, r8
	beq	.L2004
	ldrh	r2, [r9, r2]
	mov	r8, r4, asl #1
	ldr	r0, .L2093+32
	mov	r1, fp
	ldrh	r3, [r9, r8]
	str	r2, [sp, #0]
	mov	r2, r4
	bl	printk
	ldrh	r3, [r5, r6]
	cmp	r3, #40
	bls	.L2006
	ldr	r3, [r5, #-2064]
	ldrh	r3, [r3, r8]
	cmp	r3, #32
	strhih	sl, [r5, r6]	@ movhi
.L2006:
	ldr	r2, .L2093
	mov	r3, #404
	mov	r1, #6
	strh	r1, [r2, r3]	@ movhi
	b	.L2008
.L2004:
	bl	GetSwlReplaceBlock
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	bne	.L2008
	ldr	r2, .L2093
	mov	r3, #404
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L1998:
	ldr	r6, .L2093
	movw	r4, #65535
	ldr	r3, .L2093+16
	ldrh	r2, [r6, r3]
	cmp	r2, r4
	bne	.L2008
	sub	r3, r3, #192
	ldrh	r3, [r6, r3]
	cmp	r3, r2
	movne	r4, r2
	bne	.L2008
	ldr	r2, .L2093+8
	ldrh	r8, [r6, r2]
	cmp	r8, r3
	movne	r4, r3
	bne	.L2008
	ldr	sl, .L2093+20
	ldr	r2, [r6, #-1188]
	ldrh	r3, [r6, sl]
	cmp	r3, #23
	movhi	r3, #1024
	movls	r3, #5120
	cmp	r2, r3
	movls	r4, r8
	bls	.L2008
	mov	r2, #404
	mov	r3, #0
	str	r3, [r6, #-1188]
	strh	r3, [r6, r2]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r8
	mov	r4, r0
	bne	.L2010
	ldr	r3, .L2093+24
	ldrh	r1, [r6, sl]
	ldrh	r2, [r6, r3]
	cmp	r1, r2
	movcs	r2, #80
	strcsh	r2, [r6, r3]	@ movhi
	bcs	.L2020
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r4
	beq	.L2020
	ldr	r2, [r6, #-1004]
	cmp	r2, #0
	bne	.L2013
	ldr	r2, .L2093+12
	movw	r1, #3864
	ldrh	r1, [r2, r1]
	cmp	r1, #3
	beq	.L2013
	ldr	r1, [r6, #-1708]
	cmp	r1, #0
	bne	.L2013
	ldr	r1, [r6, #-1884]
	cmp	r1, #0
	bne	.L2013
	ldrb	r0, [r2, #76]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L2014
.L2013:
	ldr	r1, .L2093
	mov	r3, r3, asl #1
	movw	ip, #3844
	movw	lr, #3864
	ldr	r2, [r1, #-2064]
	ldrh	r0, [r2, r3]
	movw	r2, #3914
	ldr	r3, .L2093+12
	ldrh	r2, [r3, r2]
	ldrh	ip, [r3, ip]
	ldrh	r3, [r3, lr]
	mul	ip, ip, r2
	cmp	r3, #3
	moveq	r3, r2, lsr #1
	movne	r3, #0
	add	r3, ip, r3
	cmp	r0, r3
	bgt	.L2016
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r2, .L2093+12
	ldr	r3, .L2093
	ldr	r2, [r2, #3972]
	ldr	r1, [r3, #-1892]
	add	r2, r2, r2, asl #1
	cmp	r1, r2, lsr #2
	ldr	r2, .L2093+24
	movhi	r1, #128
	movls	r1, #160
	strh	r1, [r3, r2]	@ movhi
	uxth	r4, r0
	b	.L2018
.L2016:
	ldr	r3, .L2093+24
	mov	r2, #128
	strh	r2, [r1, r3]	@ movhi
	b	.L2020
.L2014:
	ldr	r2, [r6, #-2064]
	mov	r3, r3, asl #1
	ldr	r6, .L2093
	ldr	r5, .L2093+24
	ldrh	r3, [r2, r3]
	cmp	r3, #7
	movhi	r3, #64
	strhih	r3, [r6, r5]	@ movhi
	bhi	.L2020
	bl	List_get_gc_head_node
	mov	r3, #128
	strh	r3, [r6, r5]	@ movhi
	uxth	r4, r0
.L2018:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L2020
.L2010:
	ldr	r1, .L2093
	mov	r0, r4, asl #1
	ldr	r3, .L2093+20
	ldr	ip, [r1, #-2084]
	ldrh	r2, [r1, r3]
	ldr	r3, [r1, #-2064]
	ldrh	r3, [r3, r0]
	ldrh	r0, [ip, r0]
	str	r0, [sp, #0]
	ldr	r0, .L2093+36
	ldrh	r1, [r1, r0]
	ldr	r0, .L2093+40
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L2020:
	bl	FtlGcReFreshBadBlk
.L2008:
	movw	r1, #65535
	rsb	r5, r1, r4
	rsbs	r2, r5, #0
	adc	r2, r2, r5
	ldr	r5, .L2093
	cmp	r7, #0
	movne	r7, #0
	andeq	r7, r2, #1
	cmp	r7, #0
	beq	.L2021
	ldr	r3, .L2093+20
	ldrh	r2, [r5, r3]
	cmp	r2, #24
	movhi	r6, #1
	bhi	.L2022
	ldr	r1, .L2093+12
	movw	r3, #3912
	cmp	r2, #16
	ldrh	r6, [r1, r3]
	movhi	r6, r6, lsr #5
	bhi	.L2022
	cmp	r2, #12
	movhi	r6, r6, lsr #4
	bhi	.L2022
	cmp	r2, #8
	movhi	r6, r6, lsr #2
.L2022:
	ldr	r1, .L2093+36
	ldr	r3, .L2093
	ldrh	r0, [r5, r1]
	cmp	r0, r2
	mov	r0, r1
	bcs	.L2026
	ldr	r2, .L2093+44
	movw	ip, #65535
	ldrh	r2, [r3, r2]
	cmp	r2, ip
	bne	.L2027
	ldr	ip, .L2093+8
	ldrh	ip, [r3, ip]
	cmp	ip, r2
	bne	.L2027
	mov	r2, #404
	ldrh	r0, [r3, r2]
	cmp	r0, #0
	bne	.L2028
	ldr	r2, .L2093+12
	ldr	ip, [r3, #-1892]
	ldr	r2, [r2, #3972]
	add	r2, r2, r2, asl #1
	cmp	ip, r2, lsr #2
	movcs	r2, #18
	bcs	.L2091
.L2028:
	ldr	r3, .L2093
	ldr	r2, .L2093+48
	ldr	r1, .L2093+36
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
.L2091:
	strh	r2, [r3, r1]	@ movhi
	mov	r2, #0
	ldr	r3, .L2093
	str	r2, [r3, #-1180]
	b	.L2031
.L2027:
	ldr	r3, .L2093
	ldr	r2, .L2093+48
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r0]	@ movhi
.L2026:
	ldr	r3, .L2093
	movw	r4, #65535
	ldr	r8, [sp, #20]
	ldr	r2, [r3, #-1884]
	cmp	r8, #2
	movhi	r3, #0
	movls	r3, #1
	cmp	r2, #0
	moveq	r3, #0
	cmp	r3, #0
	addne	r6, r6, #1
	uxthne	r6, r6
	b	.L2032
.L2021:
	ldr	r3, .L2093+44
	ldrh	r3, [r5, r3]
	cmp	r3, r1
	bne	.L2033
	ldr	r1, .L2093+8
	ldrh	r1, [r5, r1]
	cmp	r1, r3
	movne	r2, #0
	andeq	r2, r2, #1
	cmp	r2, #0
	beq	.L2033
	ldr	r2, .L2093+16
	ldrh	r4, [r5, r2]
	cmp	r4, r3
	movne	r4, r3
	bne	.L2033
	ldr	r3, .L2093+20
	mov	r6, #404
	str	r7, [r5, #-1180]
	ldrh	r7, [r5, r3]
	add	r3, r3, #856
	ldrh	r2, [r5, r3]
	cmp	r2, r7
	bcs	.L2034
	ldrh	r2, [r5, r6]
	cmp	r2, #0
	bne	.L2035
	ldr	r2, .L2093+12
	ldr	r1, [r5, #-1892]
	ldr	r2, [r2, #3972]
	add	r2, r2, r2, asl #1
	cmp	r1, r2, lsr #2
	movcs	r2, #18
	strcsh	r2, [r5, r3]	@ movhi
	bcs	.L2037
.L2035:
	ldr	r3, .L2093
	ldr	r2, .L2093+48
	ldr	r1, .L2093+36
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L2037:
	bl	FtlReadRefresh
	mov	r3, #404
	ldr	r2, .L2093
	b	.L2092
.L2034:
	ldrh	r0, [r5, r6]
	cmp	r0, #0
	bne	.L2033
	ldr	r2, .L2093+48
	ldrh	r4, [r5, r2]
	add	r2, r4, r4, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r5, r3]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r5, #-2064]
	ldr	r1, .L2093+12
	movw	r2, #3914
	ldrh	r2, [r1, r2]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	movw	r0, #3844
	ldrh	r1, [r1, r0]
	mul	r2, r1, r2
	add	r2, r2, r2, lsr #31
	cmp	r3, r2, asr #1
	ble	.L2038
	sub	r4, r4, #1
	cmp	r7, r4
	blt	.L2038
	bl	FtlReadRefresh
	ldrh	r0, [r5, r6]
	b	.L2031
.L2038:
	cmp	r3, #0
	movwne	r4, #65535
	bne	.L2033
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r3, .L2093+20
	ldr	r2, .L2093
	ldrh	r0, [r2, r3]
	add	r0, r0, #1
	b	.L2031
.L2033:
	ldr	r3, .L2093
	ldr	r6, [r3, #-1884]
	cmp	r6, #0
	movne	r6, #2
	moveq	r6, #1
.L2032:
	ldr	r3, .L2093
	movw	r0, #65535
	ldr	r2, .L2093+16
	ldrh	r1, [r3, r2]
	cmp	r1, r0
	bne	.L2039
	cmp	r4, r1
	strneh	r4, [r3, r2]	@ movhi
	bne	.L2041
	ldr	r2, .L2093+8
	ldrh	r1, [r3, r2]
	cmp	r1, r4
	beq	.L2041
	ldr	r0, [r3, #-2064]
	mov	r1, r1, asl #1
	ldrh	r1, [r0, r1]
	cmp	r1, #0
	mvneq	r1, #0
	streqh	r1, [r3, r2]	@ movhi
	ldr	r3, .L2093
	ldr	r1, .L2093+16
	ldrh	r0, [r3, r2]
	strh	r0, [r3, r1]	@ movhi
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
.L2041:
	ldr	r5, .L2093
	mov	r3, #0
	ldr	r7, .L2093+16
	strb	r3, [r5, #-1748]
	movw	r3, #65535
	ldrh	r0, [r5, r7]
	cmp	r0, r3
	beq	.L2039
	bl	IsBlkInGcList
	cmp	r0, #0
	mvnne	r3, #0
	strneh	r3, [r5, r7]	@ movhi
	ldr	r3, .L2093+12
	ldr	r7, .L2093
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2044
	ldr	r3, .L2093+16
	ldrh	r0, [r7, r3]
	bl	ftl_get_blk_mode
	strb	r0, [r7, #-1748]
.L2044:
	ldr	r8, .L2093+16
	movw	r3, #65535
	ldr	r5, .L2093
	ldrh	r2, [r7, r8]
	cmp	r2, r3
	beq	.L2039
	ldr	r0, .L2093+52
	bl	make_superblock
	ldr	r1, .L2093+56
	movw	r2, #406
	mov	r3, #0
	strh	r3, [r5, r2]	@ movhi
	strh	r3, [r5, r1]	@ movhi
	strb	r3, [r5, #-1750]
	ldrh	r3, [r5, r8]
	ldr	r2, [r5, #-2064]
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	mov	r3, #408
	strh	r2, [r5, r3]	@ movhi
.L2039:
	ldr	r3, .L2093
	ldr	r1, .L2093+16
	ldr	r0, .L2093+60
	ldrh	r2, [r3, r1]
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	beq	.L2045
	ldr	r0, .L2093+64
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	bne	.L2073
.L2045:
	mvn	r2, #0
	strh	r2, [r3, r1]	@ movhi
.L2073:
	str	r4, [sp, #20]
.L2088:
	ldr	r4, .L2093
	movw	r0, #65535
	ldr	r9, .L2093+16
	ldrh	r3, [r4, r9]
	cmp	r3, r0
	bne	.L2047
	mov	r3, #0
	str	r3, [r4, #-1180]
.L2087:
	ldr	r8, .L2093+28
	ldrh	r7, [r4, r8]
	mov	r0, r7
	bl	List_get_gc_head_node
	movw	r1, #65535
	uxth	r5, r0
	strh	r5, [r4, r9]	@ movhi
	cmp	r5, r1
	ldreq	r3, .L2093
	moveq	r2, #0
	moveq	r0, #8
	streqh	r2, [r3, r8]	@ movhi
	beq	.L2031
.L2049:
	mov	r0, r5
	add	r7, r7, #1
	bl	IsBlkInGcList
	cmp	r0, #0
	strneh	r7, [r4, r8]	@ movhi
	bne	.L2087
	ldr	ip, .L2093+12
	uxth	r7, r7
	movw	r3, #3912
	strh	r7, [r4, r8]	@ movhi
	movw	r8, #3844
	ldr	r1, [r4, #-2064]
	ldrh	r3, [ip, r3]
	mov	r2, r5, asl #1
	ldrh	ip, [ip, r8]
	ldrh	r0, [r1, r2]
	mul	r3, ip, r3
	add	ip, r3, r3, lsr #31
	cmp	r0, ip, asr #1
	bgt	.L2052
	cmp	r7, #48
	bls	.L2053
	cmp	r0, #8
	bls	.L2053
	ldr	r0, .L2093+68
	ldrh	r0, [r4, r0]
	cmp	r0, #35
	bhi	.L2053
.L2052:
	ldr	r0, .L2093+28
	mov	ip, #0
	strh	ip, [r4, r0]	@ movhi
.L2053:
	ldrh	r2, [r1, r2]
	movw	r0, #65535
	ldr	r8, [sp, #20]
	ldr	sl, .L2093
	cmp	r2, r3
	cmpge	r8, r0
	movne	r3, #0
	moveq	r3, #1
	bne	.L2054
	ldr	r3, .L2093+16
	mvn	r2, #0
	strh	r2, [sl, r3]	@ movhi
	add	r3, r3, #572
	mov	r2, #0
	strh	r2, [sl, r3]	@ movhi
	mov	r3, #404
	ldrh	r0, [sl, r3]
	b	.L2031
.L2054:
	cmp	r2, #0
	bne	.L2055
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r3, .L2093+28
	ldrh	r2, [r4, r3]
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	b	.L2087
.L2055:
	strb	r3, [sl, #-1748]
	ldr	r3, .L2093+12
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2056
	mov	r0, r5
	bl	ftl_get_blk_mode
	strb	r0, [sl, #-1748]
.L2056:
	ldr	r4, .L2093
	ldr	r0, .L2093+52
	bl	make_superblock
	movw	r1, #406
	mov	r3, #0
	strh	r3, [r4, r1]	@ movhi
	ldr	r1, .L2093+16
	ldr	r0, [r4, #-2064]
	ldrh	r1, [r4, r1]
	mov	r1, r1, asl #1
	ldrh	r0, [r0, r1]
	mov	r1, #408
	strb	r3, [r4, #-1750]
	strh	r0, [r4, r1]	@ movhi
	ldr	r1, .L2093+56
	strh	r3, [r4, r1]	@ movhi
.L2047:
	ldr	r3, .L2093
	mov	r2, #1
	movw	r1, #3912
	str	r2, [r3, #-1000]
	ldr	r2, .L2093+12
	ldrh	r9, [r2, r1]
	ldrb	r1, [r2, #76]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L2057
	ldrb	r1, [r3, #-1748]	@ zero_extendqisi2
	cmp	r1, #1
	movweq	r1, #3914
	ldreqh	r9, [r2, r1]
.L2057:
	ldr	r2, .L2093+56
	mov	sl, #0
	ldr	r4, .L2093
	mov	fp, r9
	ldrh	r3, [r3, r2]
	add	r2, r3, r6
	cmp	r2, r9
	rsbgt	r6, r3, r9
	uxthgt	r6, r6
	b	.L2059
.L2061:
	ldrh	r1, [r2, #2]!
	movw	r8, #65535
	add	r3, r3, #1
	cmp	r1, r8
	uxth	r3, r3
	orrne	r1, ip, r1, asl #10
	mlane	r8, r0, r7, lr
	addne	r7, r7, #1
	uxthne	r7, r7
	strne	r1, [r8, #4]
.L2067:
	cmp	r3, r5
	bne	.L2061
	ldr	r0, [r4, #-1160]
	mov	r1, r7
	ldrb	r2, [r4, #-1748]	@ zero_extendqisi2
	mov	r5, #0
	bl	FlashReadPages
	mov	r9, r5
	mov	r8, r6
	b	.L2062
.L2065:
	ldr	r3, [r4, #-1160]
	add	r2, r3, r5
	ldr	r3, [r3, r5]
	ldr	r6, [r2, #12]
	cmn	r3, #1
	beq	.L2063
	ldrh	r3, [r6, #0]
	movw	r0, #61589
	cmp	r3, r0
	bne	.L2063
	add	r1, sp, #28
	mov	r2, #0
	ldr	r0, [r6, #8]
	bl	log2phys
	ldr	r1, [r4, #-1160]
	ldr	r2, [sp, #28]
	add	r1, r1, r5
	bic	r2, r2, #-2147483648
	ldr	r3, [r1, #4]
	cmp	r2, r3
	bne	.L2063
	movw	r3, #406
	ldr	r0, [r4, #-1176]
	ldrh	r2, [r4, r3]
	ldr	r1, [r1, #16]
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	ldr	r2, [r4, #-980]
	mov	r3, #36
	mla	r2, r3, r0, r2
	str	r1, [r2, #16]
	str	r2, [sp, #12]
	str	r3, [sp, #16]
	bl	Ftl_get_new_temp_ppa
	ldr	r2, [sp, #12]
	ldr	r1, [r4, #-1176]
	str	r0, [r2, #4]
	ldr	r3, [sp, #16]
	ldr	r2, [r4, #-980]
	mla	r3, r3, r1, r2
	ldr	r2, [r4, #-1160]
	add	r2, r2, r5
	ldr	r1, [r2, #8]
	str	r1, [r3, #8]
	mov	r1, #1
	ldr	r2, [r2, #12]
	str	r2, [r3, #12]
	ldr	r3, [sp, #28]
	str	r3, [r6, #12]
	ldr	r3, .L2093+44
	ldrh	r3, [r4, r3]
	strh	r3, [r6, #2]	@ movhi
	ldr	r3, [r4, #-1844]
	ldr	r0, [r4, #-1160]
	str	r3, [r6, #4]
	add	r0, r0, r5
	ldr	r3, [r4, #-1176]
	add	r3, r3, #1
	str	r3, [r4, #-1176]
	bl	FtlGcBufAlloc
	ldr	r3, .L2093+12
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2064
	ldrb	r3, [r4, #-1941]	@ zero_extendqisi2
	ldr	r2, [r4, #-1176]
	cmp	r2, r3
	beq	.L2064
	ldr	r3, .L2093+72
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L2063
.L2064:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	beq	.L2063
	ldr	r3, .L2093
	mov	r2, #0
	str	r2, [r3, #-1000]
	mov	r2, #404
.L2092:
	ldrh	r0, [r3, r2]
	b	.L2031
.L2063:
	add	r9, r9, #1
	add	r5, r5, #36
	uxth	r9, r9
.L2062:
	cmp	r9, r7
	bne	.L2065
	add	sl, sl, #1
	mov	r6, r8
.L2059:
	uxth	r3, sl
	ldr	r2, .L2093+56
	cmp	r3, r6
	ldr	r3, .L2093
	bcs	.L2066
	ldr	r1, .L2093+12
	movw	r3, #3844
	ldrh	ip, [r4, r2]
	mov	r7, #0
	ldr	lr, [r4, #-1160]
	mov	r0, #36
	ldrh	r5, [r1, r3]
	add	ip, ip, sl
	ldr	r2, .L2093+76
	mov	r3, r7
	b	.L2067
.L2066:
	ldrh	r1, [r3, r2]
	mov	r9, fp
	add	r6, r6, r1
	uxth	r6, r6
	strh	r6, [r3, r2]	@ movhi
	cmp	r6, fp
	bcs	.L2068
	movw	r2, #406
	mov	r1, #408
	ldrh	r2, [r3, r2]
	ldrh	r3, [r3, r1]
	cmp	r2, r3
	bne	.L2069
.L2068:
	ldr	r5, .L2093
	ldr	r3, [r5, #-1176]
	cmp	r3, #0
	beq	.L2070
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	movne	r3, #0
	strne	r3, [r5, #-1000]
	movne	r3, #404
	ldrneh	r0, [r5, r3]
	bne	.L2031
.L2070:
	movw	r2, #406
	ldr	r3, .L2093
	ldrh	ip, [r5, r2]
	cmp	ip, #0
	bne	.L2071
	ldr	r2, .L2093+16
	ldr	r0, [r3, #-2064]
	ldrh	r1, [r3, r2]
	mov	r1, r1, asl #1
	ldrh	r4, [r0, r1]
	cmp	r4, #0
	beq	.L2071
	strh	ip, [r0, r1]	@ movhi
	ldrh	r0, [r3, r2]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2071:
	ldr	r3, .L2093+16
	mvn	r1, #0
	ldr	r2, .L2093
	strh	r1, [r2, r3]	@ movhi
.L2069:
	ldr	r3, .L2093
	mov	r2, #0
	str	r2, [r3, #-1000]
	sub	r2, r2, #2048
	ldrh	r2, [r3, r2]
	cmp	r2, #2
	ldrls	r2, .L2093+12
	movwls	r3, #3912
	ldrlsh	r6, [r2, r3]
	bls	.L2088
.L2072:
	mov	r1, #404
	ldrh	r0, [r3, r1]
	cmp	r0, #0
	addeq	r0, r2, #1
.L2031:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2094:
	.align	2
.L2093:
	.word	.LANCHOR2
	.word	-1136
	.word	-1138
	.word	.LANCHOR0
	.word	-1756
	.word	-2048
	.word	-1190
	.word	-1184
	.word	.LC123
	.word	-1192
	.word	.LC124
	.word	-1948
	.word	-1772
	.word	.LANCHOR2-1756
	.word	-1754
	.word	-2044
	.word	-1996
	.word	-1152
	.word	-1944
	.word	.LANCHOR2-1742
	.fnend
	.size	rk_ftl_garbage_collect.part.17, .-rk_ftl_garbage_collect.part.17
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, .L2101
	mov	r5, r1
	ldr	r0, [r3, #-1000]
	cmp	r0, #0
	movne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, pc}
	ldr	r2, .L2101+4
	ldrh	r2, [r3, r2]
	cmp	r2, #47
	ldmlsfd	sp!, {r3, r4, r5, pc}
	ldr	r1, .L2101+8
	movw	r2, #3000
	ldrh	r1, [r1, r2]
	movw	r2, #65535
	cmp	r1, r2
	beq	.L2097
	ldr	r1, .L2101+12
	ldrh	r3, [r3, r1]
	cmp	r3, r2
	beq	.L2097
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	bne	.L2100
.L2097:
	mov	r0, r4
	mov	r1, r5
	ldmfd	sp!, {r3, r4, r5, lr}
	b	rk_ftl_garbage_collect.part.17
.L2100:
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, pc}
.L2102:
	.align	2
.L2101:
	.word	.LANCHOR2
	.word	-2056
	.word	.LANCHOR1
	.word	-1948
	.fnend
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.L2104:
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mvn	r3, #0
	ldr	r5, .L2108
	mov	r6, r0
	ldr	r4, .L2108+4
	ldr	r1, .L2108+8
	str	r3, [r5, #2960]
	mov	r3, #0
	ldr	r0, .L2108+12
	str	r3, [r4, #412]
	bl	printk
	mov	r0, r6
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldr	r3, .L2108+16
	ldr	r0, [r3, #3848]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L2105
	bl	FtlSysBlkInit
	cmp	r0, #0
	bne	.L2105
	mov	r3, #1
	str	r3, [r5, #2960]
	ldr	r3, .L2108+20
	ldrh	r3, [r4, r3]
	cmp	r3, #15
	bhi	.L2105
	movw	r4, #8129
.L2106:
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
	subs	r4, r4, #1
	bne	.L2106
.L2105:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2109:
	.align	2
.L2108:
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LC75
	.word	.LC74
	.word	.LANCHOR0
	.word	-2048
	.fnend
	.size	FtlInit, .-FtlInit
	.align	2
	.global	rk_ftl_init
	.type	rk_ftl_init, %function
rk_ftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r0, #2048
	bl	ftl_malloc
	ldr	r4, .L2114
	mov	r5, #0
	add	r1, r4, #420
	str	r5, [r4, #112]
	str	r5, [r4, #420]
	str	r0, [r4, #416]
	add	r0, r4, #112
	bl	rknand_get_reg_addr
	ldr	r3, [r4, #112]
	cmp	r3, r5
	mvneq	r4, #0
	beq	.L2111
	bl	rk_nandc_irq_init
	ldr	r0, [r4, #416]
	mov	r1, r5
	mov	r2, r5
	mov	r3, #2048
	bl	FlashSramLoadStore
	bl	rknand_flash_cs_init
	ldr	r0, [r4, #112]
	bl	FlashInit
	subs	r4, r0, #0
	bne	.L2112
	ldr	r0, .L2114+4
	bl	FtlInit
.L2112:
	ldr	r0, .L2114+8
	mov	r1, r4
	bl	printk
.L2111:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L2115:
	.align	2
.L2114:
	.word	.LANCHOR2
	.word	.LANCHOR0+2196
	.word	.LC125
	.fnend
	.size	rk_ftl_init, .-rk_ftl_init
	.align	2
	.global	FtlDiscard
	.type	FtlDiscard, %function
FtlDiscard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, lr}
	add	r2, r1, r0
	ldr	r5, .L2125
	mov	r7, r0
	mov	r4, r1
	ldr	r3, [r5, #3952]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L2117
	cmp	r1, #31
	movls	r0, #0
	bls	.L2117
	bl	FtlCacheWriteBack
	movw	r3, #3918
	ldrh	r6, [r5, r3]
	mov	r0, r7
	mov	r1, r6
	bl	__aeabi_uidiv
	mls	r7, r0, r6, r7
	mov	r5, r0
	uxth	r7, r7
	cmp	r7, #0
	beq	.L2118
	rsb	r6, r7, r6
	add	r5, r0, #1
	cmp	r6, r4
	movcs	r6, r4
	uxth	r6, r6
	rsb	r4, r6, r4
.L2118:
	mvn	r3, #0
	ldr	r7, .L2125
	str	r3, [sp, #4]
	movw	r6, #3918
	b	.L2119
.L2121:
	mov	r0, r5
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #0]
	cmn	r3, #1
	beq	.L2120
	ldr	r3, .L2125+4
	add	r1, sp, #4
	mov	r0, r5
	ldr	r2, [r3, #424]
	add	r2, r2, #1
	str	r2, [r3, #424]
	ldr	r2, [r3, #-1876]
	add	r2, r2, #1
	str	r2, [r3, #-1876]
	mov	r2, #1
	bl	log2phys
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L2120:
	ldrh	r3, [r7, r6]
	add	r5, r5, #1
	rsb	r4, r3, r4
.L2119:
	ldrh	r3, [r7, r6]
	cmp	r4, r3
	bcs	.L2121
	ldr	r3, .L2125+4
	mov	r4, #0
	ldr	r2, [r3, #424]
	cmp	r2, #32
	bls	.L2124
	str	r4, [r3, #424]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2124:
	mov	r0, r4
.L2117:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L2126:
	.align	2
.L2125:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlDiscard, .-FtlDiscard
	.align	2
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	FtlCacheWriteBack
	bl	l2p_flush
	mov	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
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
	ldr	r3, .L2130
	ldr	r3, [r3, #2960]
	cmp	r3, #1
	bne	.L2129
	bl	FtlSysFlush
.L2129:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2131:
	.align	2
.L2130:
	.word	.LANCHOR1
	.fnend
	.size	FtlDeInit, .-FtlDeInit
	.align	2
	.global	rk_ftl_de_init
	.type	rk_ftl_de_init, %function
rk_ftl_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	rk_nand_de_init
	bl	FtlDeInit
	ldmfd	sp!, {r3, lr}
	b	rk_nand_de_init
	.fnend
	.size	rk_ftl_de_init, .-rk_ftl_de_init
	.align	2
	.global	FtlWrite
	.type	FtlWrite, %function
FtlWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #16
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r1
	.pad #100
	sub	sp, sp, #100
	mov	r5, r2
	mov	r8, r3
	bne	.L2134
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartWrite
	b	.L2135
.L2134:
	ldr	r9, .L2188
	add	r7, r2, r1
	ldr	r3, [r9, #3952]
	cmp	r7, r3
	mvnhi	r0, #0
	bhi	.L2135
	ldr	r6, .L2188+4
	mov	r3, #2048
	mov	r0, r1
	str	r3, [r6, #428]
	movw	r3, #3918
	ldrh	sl, [r9, r3]
	mov	r1, sl
	bl	__aeabi_uidiv
	mov	r1, sl
	str	r0, [sp, #16]
	sub	r0, r7, #1
	bl	__aeabi_uidiv
	ldr	r3, [sp, #16]
	cmp	r5, #8
	rsb	fp, r3, r0
	ldr	r3, [r6, #-1872]
	add	r7, fp, #1
	str	r7, [sp, #12]
	add	r3, r7, r3
	str	r3, [r6, #-1872]
	ldr	r3, [r6, #-1856]
	str	r0, [sp, #44]
	add	r3, r5, r3
	str	r3, [r6, #-1856]
	ldr	r3, .L2188+8
	add	r7, r3, #48
	movhi	r7, r3
	ldr	r3, [r9, #3964]
	cmp	r3, #0
	beq	.L2171
	ldr	r2, [r9, #3968]
	sub	r3, r3, #1
	mov	r7, #36
	ldr	lr, [sp, #16]
	mla	r7, r7, r3, r2
	ldr	r3, [r7, #16]
	cmp	lr, r3
	strne	r5, [sp, #32]
	bne	.L2138
	ldr	r3, [r6, #-1868]
	mov	r0, r4
	mov	r1, sl
	add	r3, r3, #1
	str	r3, [r6, #-1868]
	ldr	r3, [r6, #432]
	add	r3, r3, #1
	str	r3, [r6, #432]
	bl	__aeabi_uidivmod
	ldr	r0, [r7, #8]
	rsb	r9, r1, sl
	add	r0, r0, r1, asl #9
	cmp	r9, r5
	movcs	r9, r5
	mov	r1, r8
	mov	r3, r9, asl #9
	str	r3, [sp, #0]
	mov	r2, r3
	bl	memcpy
	cmp	fp, #0
	ldr	r3, [sp, #0]
	bne	.L2139
	ldr	r2, [r6, #432]
	cmp	r2, #2
	movle	r0, fp
	ble	.L2135
.L2139:
	add	r8, r8, r3
	add	r4, r4, r9
	ldr	r3, [sp, #16]
	rsb	r7, r9, r5
	str	fp, [sp, #12]
	add	r3, r3, #1
	str	r7, [sp, #32]
	str	r3, [sp, #16]
.L2138:
	ldr	r3, .L2188+4
	mov	r2, #0
	ldr	r7, [r3, #400]
	str	r2, [r3, #432]
	b	.L2137
.L2171:
	str	r5, [sp, #32]
.L2137:
	ldr	r0, [sp, #16]
	ldr	r1, [sp, #44]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L2140
	bl	FtlCacheWriteBack
.L2140:
	cmp	r5, sl, asl #1
	ldr	r5, .L2188
	mov	fp, #0
	ldr	r6, [sp, #16]
	movcc	r3, #0
	movcs	r3, #1
	str	r3, [sp, #20]
	ldr	r3, .L2188+4
	mov	sl, r5
	str	fp, [sp, #40]
	str	r8, [sp, #28]
	str	r7, [r3, #400]
	b	.L2185
.L2166:
	ldrh	r2, [r7, #4]
	cmp	r2, #0
	bne	.L2142
	ldr	r3, .L2188+8
	ldr	r8, .L2188+12
	cmp	r7, r3
	bne	.L2143
	ldr	r3, .L2188+16
	ldrh	r9, [r9, r3]
	cmp	r9, #0
	bne	.L2144
	ldr	r0, .L2188+20
	bl	allocate_new_data_superblock
	str	r9, [r8, #3004]
.L2144:
	ldr	r0, .L2188+8
	bl	allocate_new_data_superblock
	ldr	r2, [r8, #3004]
	ldr	r3, .L2188+20
	cmp	r2, #0
	movne	r7, r3
	b	.L2145
.L2143:
	str	r2, [r8, #3004]
	ldr	r2, .L2188+24
	ldrh	r2, [r9, r2]
	cmp	r2, #0
	movne	r7, r3
	bne	.L2145
	mov	r0, r7
	bl	allocate_new_data_superblock
.L2145:
	ldrh	r3, [r7, #4]
	cmp	r3, #0
	bne	.L2146
	mov	r0, r7
	bl	allocate_new_data_superblock
.L2146:
	ldr	r3, .L2188+4
	str	r7, [r3, #400]
.L2142:
	ldr	r2, .L2188+4
	movw	r9, #3918
	ldrh	r3, [r7, #4]
	mov	r8, r7
	ldr	r1, [r2, #-988]
	ldr	r2, [r5, #3964]
	rsb	r2, r2, r1
	cmp	r3, r2
	movcc	r2, r3
	ldr	r3, [sp, #12]
	cmp	r2, r3
	movcc	r3, r2
	str	r3, [sp, #36]
	mov	r3, #0
	str	r3, [sp, #24]
	b	.L2147
.L2163:
	ldrh	r3, [r8, #4]
	cmp	r3, #0
	beq	.L2177
	ldr	r7, [sp, #44]
	rsb	r2, r7, r6
	ldr	r7, [sp, #20]
	rsbs	r3, r2, #0
	adc	r3, r3, r2
	tst	r3, r7
	beq	.L2149
	ldr	r7, [sp, #24]
	cmp	r7, #0
	beq	.L2149
	ldr	r7, [sp, #32]
	ldrh	r2, [r5, r9]
	add	r1, r7, r4
	mls	r1, r2, r6, r1
	cmp	r1, r2
	bne	.L2177
.L2149:
	add	r1, sp, #56
	mov	r2, #0
	mov	r0, r6
	str	r3, [sp, #0]
	bl	log2phys
	mov	r0, r8
	bl	get_new_active_ppa
	ldr	lr, [r5, #3964]
	ldr	r1, [r5, #3968]
	movw	r2, #3924
	mov	ip, #36
	ldrh	r2, [r5, r2]
	ldr	r7, .L2188+4
	mla	r1, ip, lr, r1
	mul	lr, lr, r2
	ldr	r7, [r7, #-944]
	bic	lr, lr, #3
	str	r7, [sp, #52]
	str	lr, [sp, #48]
	add	r7, r7, lr
	str	r6, [r1, #16]
	str	r7, [r1, #12]
	str	r0, [r1, #4]
	movw	r0, #3922
	ldrh	lr, [r5, r0]
	ldr	r0, [r5, #3964]
	mul	lr, r0, lr
	ldr	r0, .L2188+4
	bic	lr, lr, #3
	str	lr, [sp, #8]
	ldr	lr, [r0, #-964]
	ldr	r0, [sp, #8]
	add	lr, lr, r0
	mov	r0, r7
	str	lr, [r1, #8]
	mov	r1, #0
	str	ip, [sp, #4]
	bl	ftl_memset
	ldr	r1, [sp, #16]
	ldr	ip, [sp, #4]
	rsb	r3, r1, r6
	rsbs	r1, r3, #0
	adc	r1, r1, r3
	ldr	r3, [sp, #0]
	str	r1, [sp, #8]
	orrs	r1, r1, r3
	beq	.L2150
	ldr	r2, [sp, #8]
	cmp	r2, #0
	beq	.L2151
	ldrh	fp, [r5, r9]
	mov	r0, r4
	mov	r1, fp
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #32]
	rsb	fp, r1, fp
	str	r1, [sp, #40]
	cmp	fp, r3
	movcs	fp, r3
	b	.L2152
.L2151:
	cmp	r3, #0
	beq	.L2152
	ldr	r3, [sp, #32]
	add	fp, r3, r4
	ldrh	r3, [r5, r9]
	mls	fp, r3, r6, fp
	ldr	r3, [sp, #8]
	str	r3, [sp, #40]
	uxth	fp, fp
.L2152:
	ldrh	r3, [r5, r9]
	cmp	fp, r3
	bne	.L2153
	ldr	r3, [sp, #8]
	ldr	r0, [sl, #3964]
	cmp	r3, #0
	ldr	r2, [sl, #3968]
	muleq	r1, r6, fp
	ldreq	r3, [sp, #28]
	ldrne	r1, [sp, #28]
	rsbeq	r1, r4, r1
	addeq	r1, r3, r1, asl #9
	ldr	r3, [sp, #20]
	cmp	r3, #0
	mov	r3, #36
	mla	r3, r3, r0, r2
	strne	r1, [r3, #8]
	bne	.L2156
	ldr	r0, [r3, #8]
	movw	r3, #3922
	ldrh	r2, [sl, r3]
	b	.L2186
.L2153:
	ldr	r2, [sp, #56]
	mov	r3, #36
	cmn	r2, #1
	beq	.L2157
	ldr	r1, [r5, #3964]
	add	r0, sp, #60
	str	r2, [sp, #64]
	ldr	r2, [r5, #3968]
	str	r6, [sp, #76]
	mla	r3, r3, r1, r2
	mov	r1, #1
	ldr	r2, [r3, #8]
	ldr	r3, [r3, #12]
	str	r2, [sp, #68]
	mov	r2, #0
	str	r3, [sp, #72]
	bl	FlashReadPages
	ldr	r3, [sp, #60]
	cmn	r3, #1
	ldr	r3, .L2188+4
	ldreq	r2, [r3, #-1632]
	addeq	r2, r2, #1
	streq	r2, [r3, #-1632]
	beq	.L2159
	ldr	r2, [r7, #8]
	cmp	r2, r6
	beq	.L2159
	ldr	r2, [r3, #-1632]
	ldr	r0, .L2188+28
	add	r2, r2, #1
	str	r2, [r3, #-1632]
	mov	r2, r6
	ldr	r1, [r7, #8]
	bl	printk
	b	.L2159
.L2157:
	ldr	r1, [r5, #3964]
	ldr	r2, [r5, #3968]
	mla	r3, r3, r1, r2
	mov	r1, #0
	ldr	r0, [r3, #8]
	movw	r3, #3922
	ldrh	r2, [r5, r3]
	bl	ftl_memset
.L2159:
	ldr	r3, [sp, #8]
	ldr	r1, [r5, #3964]
	ldr	r2, [r5, #3968]
	cmp	r3, #0
	mov	r3, #36
	mla	r3, r3, r1, r2
	ldreqh	r1, [r5, r9]
	ldrne	r1, [sp, #28]
	mov	r2, fp, asl #9
	muleq	r1, r1, r6
	ldrne	r0, [r3, #8]
	ldrne	r3, [sp, #40]
	rsbeq	r1, r4, r1
	ldreq	r0, [r3, #8]
	ldreq	r3, [sp, #28]
	addne	r0, r0, r3, asl #9
	addeq	r1, r3, r1, asl #9
	b	.L2186
.L2150:
	ldr	r3, [sp, #20]
	ldr	r2, [r5, #3964]
	cmp	r3, #0
	ldr	r3, [r5, #3968]
	mla	ip, ip, r2, r3
	beq	.L2161
	ldrh	r3, [r5, r9]
	ldr	lr, [sp, #28]
	mul	r3, r3, r6
	rsb	r3, r4, r3
	add	r3, lr, r3, asl #9
	str	r3, [ip, #8]
	b	.L2156
.L2161:
	ldrh	r1, [r5, r9]
	movw	lr, #3922
	ldr	r3, [sp, #28]
	ldr	r0, [ip, #8]
	mul	r1, r1, r6
	ldrh	r2, [r5, lr]
	rsb	r1, r4, r1
	add	r1, r3, r1, asl #9
.L2186:
	bl	memcpy
.L2156:
	ldr	r3, .L2188+32
	ldr	r1, [sp, #52]
	ldr	r0, [sp, #48]
	strh	r3, [r1, r0]	@ movhi
	ldr	r3, .L2188+4
	str	r6, [r7, #8]
	add	r6, r6, #1
	ldr	r2, [r3, #-1844]
	str	r2, [r7, #4]
	add	r2, r2, #1
	cmn	r2, #1
	str	r2, [r3, #-1844]
	moveq	r2, #0
	streq	r2, [r3, #-1844]
	ldr	r3, [sp, #56]
	str	r3, [r7, #12]
	ldrh	r3, [r8, #0]
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, [r5, #3964]
	add	r3, r3, #1
	str	r3, [r5, #3964]
	ldr	r3, [sp, #24]
	add	r3, r3, #1
	str	r3, [sp, #24]
.L2147:
	ldr	r7, [sp, #24]
	ldr	r3, [sp, #36]
	cmp	r7, r3
	bne	.L2163
	mov	r7, r8
	b	.L2148
.L2177:
	ldr	r3, [sp, #24]
	mov	r7, r8
	str	r3, [sp, #36]
.L2148:
	ldr	r3, [sp, #12]
	ldr	lr, [sp, #36]
	ldr	r2, [r5, #3964]
	rsb	r3, lr, r3
	str	r3, [sp, #12]
	ldr	r3, .L2188+4
	ldr	r0, [sp, #20]
	ldr	r3, [r3, #-988]
	cmp	r2, r3
	orrcs	r0, r0, #1
	uxtb	r3, r0
	cmp	r3, #0
	bne	.L2164
	ldrh	r2, [r7, #4]
	cmp	r2, #0
	bne	.L2180
.L2164:
	bl	FtlCacheWriteBack
	ldr	lr, [sp, #12]
	mov	r3, #0
	str	r3, [sl, #3964]
	cmp	lr, #3
	ldr	r3, [sp, #20]
	movls	r3, #0
.L2180:
	str	r3, [sp, #20]
.L2185:
	ldr	r3, [sp, #12]
	ldr	r9, .L2188+4
	cmp	r3, #0
	bne	.L2166
	mov	r0, r3
	ldr	r7, [sp, #44]
	ldr	r3, [sp, #16]
	rsb	r1, r3, r7
	bl	rk_ftl_garbage_collect
	ldr	r3, .L2188+36
	ldrh	r3, [r9, r3]
	cmp	r3, #15
	ldrhi	r0, [sp, #12]
	bhi	.L2135
	ldr	r5, .L2188+40
	mov	r4, r9
.L2183:
	ldrh	r3, [r9, r5]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L2167
	ldr	r2, .L2188+44
	ldrh	r2, [r9, r2]
	cmp	r2, r3
	bne	.L2167
	mov	r0, #0
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L2167:
	ldr	r2, .L2188+48
	mov	r3, #128
	mov	r0, #1
	mov	r1, r0
	strh	r3, [r4, r2]	@ movhi
	sub	r2, r2, #2
	strh	r3, [r4, r2]	@ movhi
	bl	rk_ftl_garbage_collect
	mov	r0, #0
	mov	r1, #1
	bl	rk_ftl_garbage_collect
	ldr	r3, .L2188+36
	ldrh	r3, [r4, r3]
	cmp	r3, #8
	bls	.L2183
	mov	r0, #0
.L2135:
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2189:
	.align	2
.L2188:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2-2044
	.word	.LANCHOR1
	.word	-1992
	.word	.LANCHOR2-1996
	.word	-2040
	.word	.LC126
	.word	-3947
	.word	-2048
	.word	-1756
	.word	-1138
	.word	-1190
	.fnend
	.size	FtlWrite, .-FtlWrite
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
	mov	r3, r4
	add	r1, r5, #256
	mov	r2, #1
	mov	r0, #16
	bl	FtlWrite
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
	.fnend
	.size	StorageSysDataStore, .-StorageSysDataStore
	.align	2
	.global	FtlRead
	.type	FtlRead, %function
FtlRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	cmp	r0, #16
	.pad #68
	sub	sp, sp, #68
	mov	r4, r1
	mov	r8, r3
	str	r2, [sp, #28]
	bne	.L2192
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartRead
	str	r0, [sp, #16]
	b	.L2193
.L2192:
	ldr	r3, [sp, #28]
	add	r3, r3, r1
	str	r3, [sp, #20]
	ldr	r3, .L2222
	ldr	r1, [sp, #20]
	ldr	r2, [r3, #3952]
	cmp	r1, r2
	mvnhi	r3, #0
	strhi	r3, [sp, #16]
	bhi	.L2193
	movw	r2, #3918
	mov	r0, r4
	ldrh	r5, [r3, r2]
	mov	r1, r5
	bl	__aeabi_uidiv
	ldr	r3, [sp, #20]
	mov	r1, r5
	str	r0, [sp, #12]
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	r1, [sp, #12]
	rsb	r3, r1, #1
	ldr	r1, [sp, #28]
	add	r3, r3, r0
	str	r3, [sp, #8]
	ldr	r3, .L2222+4
	mov	fp, r0
	ldr	r0, [sp, #12]
	ldr	r2, [r3, #-1852]
	add	r2, r1, r2
	ldr	r1, [sp, #8]
	str	r2, [r3, #-1852]
	ldr	r2, [r3, #-1880]
	add	r2, r1, r2
	mov	r1, fp
	str	r2, [r3, #-1880]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L2194
	bl	FtlCacheWriteBack
.L2194:
	mov	r9, #0
	ldr	r6, [sp, #12]
	str	r9, [sp, #32]
	mov	r5, r9
	str	r9, [sp, #16]
	ldr	r7, .L2222
	b	.L2217
.L2211:
	mov	r2, #0
	mov	r0, r6
	add	r1, sp, #60
	bl	log2phys
	ldr	r2, [sp, #60]
	cmn	r2, #1
	bne	.L2218
	b	.L2221
.L2199:
	mla	r0, r0, r6, sl
	cmp	r0, r4
	bcc	.L2198
	ldr	r2, [sp, #20]
	cmp	r0, r2
	bcs	.L2198
	rsb	r0, r4, r0
	mov	r1, #0
	mov	r2, #512
	str	r3, [sp, #4]
	add	r0, r8, r0, asl #9
	bl	ftl_memset
	ldr	r3, [sp, #4]
.L2198:
	add	sl, sl, #1
	b	.L2196
.L2221:
	mov	sl, #0
	movw	r3, #3918
.L2196:
	ldrh	r0, [r7, r3]
	cmp	sl, r0
	bcc	.L2199
	b	.L2200
.L2218:
	ldr	r3, .L2222+4
	mov	sl, #36
	ldr	r1, [r3, #-984]
	mla	sl, sl, r5, r1
	ldr	r1, [sp, #12]
	cmp	r6, r1
	str	r2, [sl, #4]
	movw	r2, #3918
	bne	.L2201
	ldr	r3, [r3, #-960]
	mov	r0, r4
	str	r3, [sl, #8]
	ldrh	r3, [r7, r2]
	mov	r1, r3
	str	r3, [sp, #4]
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #4]
	str	r1, [sp, #36]
	rsb	r2, r1, r3
	ldr	r1, [sp, #28]
	cmp	r2, r1
	movcc	r1, r2
	str	r1, [sp, #32]
	cmp	r1, r3
	streq	r8, [sl, #8]
	b	.L2202
.L2201:
	cmp	r6, fp
	ldrneh	r3, [r7, r2]
	mulne	r3, r3, r6
	bne	.L2219
	ldr	r3, [r3, #-956]
	ldrh	r2, [r7, r2]
	ldr	r1, [sp, #20]
	str	r3, [sl, #8]
	mul	r3, r2, r6
	rsb	r9, r3, r1
	cmp	r9, r2
	bne	.L2202
.L2219:
	rsb	r3, r4, r3
	add	r3, r8, r3, asl #9
	str	r3, [sl, #8]
.L2202:
	movw	r3, #3924
	ldr	r2, .L2222+4
	ldrh	r3, [r7, r3]
	str	r6, [sl, #16]
	ldr	r2, [r2, #-948]
	mul	r3, r3, r5
	add	r5, r5, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [sl, #12]
.L2200:
	ldr	r3, [sp, #8]
	add	r6, r6, #1
	subs	r3, r3, #1
	str	r3, [sp, #8]
	beq	.L2204
	movw	r3, #3844
	ldrh	r3, [r7, r3]
	cmp	r5, r3, asl #3
	bne	.L2217
.L2204:
	cmp	r5, #0
	beq	.L2217
	ldr	sl, .L2222+4
	mov	r1, r5
	mov	r2, #0
	ldr	r0, [sl, #-984]
	bl	FlashReadPages
	ldr	r3, [sp, #36]
	str	r6, [sp, #52]
	mov	r6, r5
	mov	r3, r3, asl #9
	str	r3, [sp, #40]
	ldr	r3, [sp, #32]
	mov	r3, r3, asl #9
	str	r3, [sp, #44]
	mov	r3, r9, asl #9
	str	r3, [sp, #48]
	mov	r3, #0
	str	r3, [sp, #24]
.L2210:
	ldr	r3, [sp, #24]
	mov	r5, #36
	ldr	r1, [sp, #12]
	mul	r5, r5, r3
	ldr	r3, [sl, #-984]
	add	r3, r3, r5
	ldr	r2, [r3, #16]
	cmp	r2, r1
	bne	.L2206
	ldr	r1, [r3, #8]
	ldr	r3, [sl, #-960]
	cmp	r1, r3
	bne	.L2207
	ldr	r3, [sp, #40]
	mov	r0, r8
	ldr	r2, [sp, #44]
	add	r1, r1, r3
	b	.L2220
.L2206:
	cmp	r2, fp
	bne	.L2207
	ldr	r1, [r3, #8]
	ldr	r3, [sl, #-956]
	cmp	r1, r3
	bne	.L2207
	movw	r2, #3918
	ldrh	r0, [r7, r2]
	ldr	r2, [sp, #48]
	mul	r0, r0, fp
	rsb	r0, r4, r0
	add	r0, r8, r0, asl #9
.L2220:
	bl	memcpy
.L2207:
	ldr	r3, [sl, #-984]
	add	r0, r3, r5
	ldr	r2, [r3, r5]
	cmn	r2, #1
	streq	r2, [sp, #16]
	ldreq	r1, [sl, #-1632]
	addeq	r1, r1, #1
	streq	r1, [sl, #-1632]
	ldr	r3, [r3, r5]
	cmp	r3, #256
	bne	.L2209
	ldr	r0, [r0, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L2209:
	ldr	r3, [sp, #24]
	add	r3, r3, #1
	str	r3, [sp, #24]
	cmp	r3, r6
	bne	.L2210
	ldr	r6, [sp, #52]
	mov	r5, #0
.L2217:
	ldr	r3, [sp, #8]
	cmp	r3, #0
	bne	.L2211
	ldr	r3, .L2222+8
	ldr	r2, .L2222+4
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2193
	ldr	r0, [sp, #8]
	mov	r1, #1
	bl	rk_ftl_garbage_collect
.L2193:
	ldr	r0, [sp, #16]
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2223:
	.align	2
.L2222:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1134
	.fnend
	.size	FtlRead, .-FtlRead
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
	mov	r3, r4
	add	r1, r5, #256
	mov	r2, #1
	mov	r0, #16
	bl	FtlRead
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
	.fnend
	.size	StorageSysDataLoad, .-StorageSysDataLoad
	.align	2
	.type	rk_ftl_vendor_ops.constprop.19, %function
rk_ftl_vendor_ops.constprop.19:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	mov	r6, r2
	mov	r5, r1
	bl	rknand_device_lock
	cmp	r6, #0
	mov	r0, #16
	mov	r1, r5
	mov	r2, #128
	mov	r3, r4
	beq	.L2226
	bl	FtlWrite
	b	.L2228
.L2226:
	bl	FtlRead
.L2228:
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	rk_ftl_vendor_ops.constprop.19, .-rk_ftl_vendor_ops.constprop.19
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r8, .L2259
	mov	r6, #0
	mov	r9, r0
	mov	sl, r1
	mov	r4, r6
	mov	r5, r6
	ldrb	r2, [r8, #76]	@ zero_extendqisi2
	mov	fp, #36
	str	r0, [r8, #3976]
	ldr	r7, .L2259+4
	str	r2, [sp, #8]
	b	.L2230
.L2234:
	mul	r3, fp, r6
	ldr	r2, [r7, #-2088]
	mov	r0, #0
	mov	r1, r9
	str	r0, [r2, r3]
	add	r3, r8, r6
	ldrb	r0, [r3, #3870]	@ zero_extendqisi2
	bl	V2P_block
	cmp	sl, #0
	str	r0, [sp, #4]
	beq	.L2231
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L2232
.L2231:
	ldr	r0, [sp, #4]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r5, r5, #1
	uxthne	r5, r5
	bne	.L2232
	ldr	r3, [r7, #-2088]
	ldr	r1, [sp, #4]
	mla	r3, fp, r4, r3
	mov	r2, r1, asl #10
	str	r0, [r3, #8]
	movw	r0, #3924
	str	r2, [r3, #4]
	ldrh	r2, [r8, r0]
	mul	r2, r2, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [r7, #-948]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L2232:
	add	r6, r6, #1
	uxth	r6, r6
.L2230:
	movw	r1, #3844
	ldrh	r3, [r8, r1]
	cmp	r3, r6
	bhi	.L2234
	cmp	r4, #0
	beq	.L2236
	ldr	r2, [sp, #8]
	mov	r8, #0
	ldr	r7, .L2259+4
	mov	fp, r8
	adds	r6, r2, #0
	mov	r2, r4
	movne	r6, #1
	ldr	r0, [r7, #-2088]
	mov	r1, r6
	bl	FlashEraseBlocks
.L2238:
	ldr	r3, [r7, #-2088]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L2237
	ldr	r0, [r2, #4]
	add	r5, r5, #1
	ubfx	r0, r0, #10, #16
	uxth	r5, r5
	bl	FtlBbmMapBadBlock
.L2237:
	add	fp, fp, #1
	add	r8, r8, #36
	uxth	fp, fp
	cmp	fp, r4
	bne	.L2238
	cmp	sl, #0
	beq	.L2251
	ldr	r3, .L2259
	movw	r2, #3914
	ldrh	r2, [r3, r2]
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	str	r2, [sp, #8]
	moveq	r3, r2, lsr #2
	streq	r3, [sp, #4]
	moveq	r6, #1
	beq	.L2239
	b	.L2258
.L2251:
	mov	r0, #6
	mov	r1, #1
	stmib	sp, {r0, r1}
	b	.L2239
.L2258:
	mov	r6, #1
	str	r6, [sp, #4]
.L2239:
	mov	r8, #0
	mov	r7, r6
.L2247:
	mov	r6, #0
	ldr	fp, .L2259+4
	mov	r4, r6
	b	.L2240
.L2243:
	mov	r0, #36
	ldr	r2, [fp, #-2088]
	mul	r3, r0, r6
	mov	r1, #0
	str	r1, [r2, r3]
	mov	r1, r9
	ldr	r2, .L2259
	add	r3, r2, r6
	ldrb	r0, [r3, #3870]	@ zero_extendqisi2
	bl	V2P_block
	cmp	sl, #0
	str	r0, [sp, #12]
	beq	.L2241
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L2242
.L2241:
	ldr	r0, [sp, #12]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L2242
	ldr	r3, [fp, #-2088]
	mov	r0, #36
	ldr	r1, [sp, #12]
	mla	r3, r0, r4, r3
	add	r2, r8, r1, asl #10
	ldr	r1, .L2259
	add	r0, r0, #3888
	str	r2, [r3, #4]
	ldr	r2, [fp, #-960]
	str	r2, [r3, #8]
	ldrh	r2, [r1, r0]
	mul	r2, r2, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [fp, #-956]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L2242:
	add	r6, r6, #1
	uxth	r6, r6
.L2240:
	ldr	r0, .L2259
	movw	r2, #3844
	ldrh	r3, [r0, r2]
	cmp	r3, r6
	bhi	.L2243
	cmp	r4, #0
	beq	.L2236
	ldr	r6, .L2259+4
	mov	r1, r4
	mov	r2, r7
	mov	r3, #1
	mov	fp, #0
	ldr	r0, [r6, #-2088]
	bl	FlashProgPages
	mov	ip, sl
	mov	sl, r7
	mov	r7, r4
	mov	r4, fp
.L2246:
	ldr	r2, [r6, #-2088]
	add	r1, r2, fp
	ldr	r2, [r2, fp]
	cmp	r2, #0
	beq	.L2245
	ldr	r0, [r1, #4]
	add	r5, r5, #1
	str	ip, [sp, #0]
	ubfx	r0, r0, #10, #16
	uxth	r5, r5
	bl	FtlBbmMapBadBlock
	ldr	ip, [sp, #0]
.L2245:
	add	r4, r4, #1
	add	fp, fp, #36
	uxth	r4, r4
	cmp	r4, r7
	bne	.L2246
	ldr	r1, [sp, #4]
	mov	r4, r7
	ldr	r2, [sp, #8]
	mov	r7, sl
	add	r8, r8, r1
	mov	sl, ip
	uxth	r8, r8
	cmp	r8, r2
	bcc	.L2247
	ldr	fp, .L2259+4
	mov	r6, r7
	mov	r7, #0
	mov	r8, r7
.L2249:
	cmp	sl, #0
	beq	.L2248
	ldr	r3, [fp, #-2088]
	add	r2, r3, r7
	ldr	r3, [r3, r7]
	cmp	r3, #0
	bne	.L2248
	ldr	r0, [r2, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L2248:
	add	r8, r8, #1
	add	r7, r7, #36
	uxth	r8, r8
	cmp	r8, r4
	bne	.L2249
	cmp	r9, #63
	movhi	r9, #0
	movls	r9, #1
	cmp	sl, #0
	moveq	sl, r9
	orrne	sl, r9, #1
	cmp	sl, #0
	beq	.L2236
	ldr	r3, .L2259+4
	mov	r1, r6
	mov	r2, r8
	ldr	r0, [r3, #-2088]
	bl	FlashEraseBlocks
.L2236:
	mov	r0, r5
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2260:
	.align	2
.L2259:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.type	FlashTestBlk.part.18, %function
FlashTestBlk.part.18:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
.L2262:
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #108
	sub	sp, sp, #108
	ldr	r4, .L2264
	mov	r5, r0
	mov	r1, #165
	add	r0, sp, #40
	mov	r2, #32
	str	r0, [sp, #16]
	ldr	r3, [r4, #116]
	mov	r5, r5, asl #10
	str	r3, [sp, #12]
	bl	ftl_memset
	ldr	r0, [r4, #116]
	mov	r1, #90
	mov	r2, #8
	bl	ftl_memset
	mov	r1, #1
	mov	r2, r1
	add	r0, sp, #4
	str	r5, [sp, #8]
	bl	FlashEraseBlocks
	mov	r1, #1
	add	r0, sp, #4
	mov	r3, r1
	mov	r2, r1
	bl	FlashProgPages
	ldr	r4, [sp, #4]
	add	r0, sp, #4
	mov	r1, #0
	cmp	r4, #0
	mov	r2, #1
	mvnne	r4, #0
	bl	FlashEraseBlocks
	mov	r0, r4
	add	sp, sp, #108
	ldmfd	sp!, {r4, r5, pc}
.L2265:
	.align	2
.L2264:
	.word	.LANCHOR2
	.fnend
	.size	FlashTestBlk.part.18, .-FlashTestBlk.part.18
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L2268
	ldr	r3, [r3, #132]
	cmp	r0, r3
	bcc	.L2267
	b	FlashTestBlk.part.18
.L2267:
	mov	r0, #0
	bx	lr
.L2269:
	.align	2
.L2268:
	.word	.LANCHOR2
	.fnend
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FtlMakeBbt
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	bl	FtlBbtMemInit
	mov	r5, #0
	ldr	r7, .L2287
	bl	FtlLoadFactoryBbt
	ldr	r6, .L2287+4
	add	sl, r7, #12
	ldr	r4, .L2287+8
	b	.L2271
.L2277:
	ldrh	r3, [sl], #2
	movw	r2, #65535
	ldr	r0, [r4, #-2076]
	movw	fp, #3908
	ldr	r9, [r4, #-952]
	cmp	r3, r2
	str	r0, [r4, #176]
	str	r9, [r4, #180]
	beq	.L2272
	ldrh	r8, [r6, fp]
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L2287+12
	mla	r8, r8, r5, r3
	mov	r3, r8, asl #10
	str	r3, [r4, #172]
	bl	FlashReadPages
	ldrh	r2, [r6, fp]
	ldr	r0, [r7, #28]
	add	r2, r2, #7
	ldr	r1, [r4, #176]
	mov	r2, r2, lsr #3
	bl	memcpy
	b	.L2273
.L2272:
	mov	r1, r5
	bl	FlashGetBadBlockList
	ldr	r0, [r4, #176]
	ldr	r1, [r7, #28]
	bl	FtlBbt2Bitmap
	ldrh	fp, [r6, fp]
.L2275:
	sub	fp, fp, #1
	uxth	fp, fp
.L2286:
	movw	r8, #3908
	ldrh	r0, [r6, r8]
	mla	r0, r0, r5, fp
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2275
	mov	r1, #0
	mov	r2, #16
	strh	fp, [sl, #-2]	@ movhi
	ldr	r0, [r4, #-952]
	bl	ftl_memset
	ldr	r3, .L2287+16
	strh	r3, [r9, #0]	@ movhi
	mov	r3, #0
	str	r3, [r9, #4]
	ldrh	r3, [sl, #-2]
	ldrh	r8, [r6, r8]
	strh	r3, [r9, #2]	@ movhi
	ldrh	r3, [sl, #-2]
	ldr	r1, [r7, #28]
	ldr	r0, [r4, #176]
	mla	r8, r8, r5, r3
	mov	r3, r8, asl #10
	str	r3, [r4, #172]
	ldr	r3, .L2287+20
	ldrh	r2, [r4, r3]
	mov	r2, r2, asl #2
	bl	memcpy
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L2287+12
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L2287+12
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #168]
	cmn	r3, #1
	bne	.L2273
	uxth	r0, r8
	bl	FtlBbmMapBadBlock
	b	.L2286
.L2273:
	uxth	r0, r8
	add	r5, r5, #1
	bl	FtlBbmMapBadBlock
	add	r7, r7, #4
.L2271:
	movw	r3, #3866
	ldrh	r3, [r6, r3]
	cmp	r5, r3
	bcc	.L2277
	mov	r4, #0
	ldr	r7, .L2287+4
	movw	r6, #3926
	b	.L2278
.L2279:
	mov	r0, r4
	add	r4, r4, #1
	bl	FtlBbmMapBadBlock
	uxth	r4, r4
.L2278:
	ldrh	r3, [r7, r6]
	ldr	r5, .L2287+4
	cmp	r3, r4
	bhi	.L2279
	movw	r7, #3992
	movw	r6, #3980
	ldrh	r4, [r5, r7]
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L2280
.L2285:
	mov	r0, r4
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2281
	mov	r0, r4
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L2282
	mov	r0, r4
	bl	FtlBbmMapBadBlock
	b	.L2281
.L2282:
	ldrh	r2, [r5, r6]
	movw	r3, #65535
	cmp	r2, r3
	streqh	r4, [r5, r6]	@ movhi
.L2283:
	ldrne	r2, .L2287+4
	movne	r3, #3984
	strneh	r4, [r2, r3]	@ movhi
	bne	.L2284
.L2281:
	sub	r4, r4, #1
	uxth	r4, r4
.L2280:
	ldrh	r3, [r5, r7]
	sub	r3, r3, #48
	cmp	r4, r3
	bgt	.L2285
.L2284:
	ldr	r4, .L2287+4
	movw	r7, #3980
	ldr	r3, .L2287+8
	mov	r6, #3984
	mov	r5, #0
	movw	r8, #3982
	str	r5, [r4, #3988]
	mov	r1, #1
	ldr	r0, [r3, #-2088]
	mov	r2, #2
	ldrh	r3, [r4, r7]
	strh	r5, [r4, r8]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, r6]
	mov	r3, r3, asl #10
	str	r3, [r0, #40]
	bl	FlashEraseBlocks
	ldrh	r0, [r4, r7]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, r6]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r4, #3988]
	ldrh	r2, [r4, r6]
	add	r3, r3, #1
	str	r3, [r4, #3988]
	ldrh	r3, [r4, r7]
	strh	r5, [r4, r8]	@ movhi
	strh	r2, [r4, r7]	@ movhi
	strh	r3, [r4, r6]	@ movhi
	bl	FtlBbmTblFlush
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2288:
	.align	2
.L2287:
	.word	.LANCHOR0+3980
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+168
	.word	-3872
	.word	-1032
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	FtlLowFormat
	.type	FtlLowFormat, %function
FtlLowFormat:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r2, #0
	ldr	r3, .L2310
	str	r2, [r3, #-1848]
	str	r2, [r3, #-1844]
	ldr	r3, .L2310+4
	ldr	r0, [r3, #3848]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L2290
	bl	FtlMakeBbt
.L2290:
	mov	r3, #0
	ldr	ip, .L2310+4
	movw	r0, #3918
	ldr	r2, .L2310
	ldr	r1, .L2310+8
	b	.L2291
.L2292:
	ldr	lr, [r2, #-960]
	mvn	r4, r3
	orr	r4, r3, r4, asl #16
	str	r4, [lr, r3, asl #2]
	ldr	lr, [r2, #-956]
	str	r1, [lr, r3, asl #2]
	add	r3, r3, #1
	uxth	r3, r3
.L2291:
	ldrh	lr, [ip, r0]
	ldr	r8, .L2310+4
	cmp	r3, lr, asl #7
	blt	.L2292
	movw	r3, #3852
	mov	r5, #0
	ldrh	r6, [r8, r3]
	movw	r7, #3854
	b	.L2293
.L2294:
	mov	r0, r6
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L2293:
	ldrh	r3, [r8, r7]
	ldr	r4, .L2310+4
	cmp	r3, r6
	bhi	.L2294
	movw	r3, #3844
	ldrh	r1, [r4, r3]
	sub	r3, r5, #3
	cmp	r3, r1, asl #1
	bge	.L2295
.L2299:
	mov	r4, #0
	ldr	r8, .L2310+4
	mov	r6, r4
	movw	r7, #3852
	b	.L2296
.L2295:
	mov	r0, r5
	movw	r6, #3854
	bl	__aeabi_uidiv
	ldr	r3, [r4, #3944]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r0, [r4, #3848]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	movw	r3, #3852
	ldrh	r5, [r4, r3]
	b	.L2297
.L2298:
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
.L2297:
	ldrh	r3, [r4, r6]
	cmp	r3, r5
	bhi	.L2298
	b	.L2299
.L2300:
	mov	r0, r6
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r4, r4, r0
	uxth	r4, r4
.L2296:
	ldrh	r3, [r8, r7]
	ldr	r5, .L2310+4
	cmp	r3, r6
	bhi	.L2300
	movw	r3, #3854
	ldr	fp, [r5, #3856]
	ldrh	r3, [r5, r3]
	ldr	r9, .L2310
	mov	r0, fp
	ldr	sl, .L2310+12
	str	r3, [r5, #3976]
	movw	r3, #3844
	ldrh	r6, [r5, r3]
	mov	r1, r6
	bl	__aeabi_uidiv
	ubfx	r8, r0, #5, #16
	mov	r7, r0
	add	r3, r8, #36
	strh	r3, [r9, sl]	@ movhi
	mov	r3, #24
	str	r0, [r5, #3972]
	mul	r3, r3, r6
	cmp	r4, r3
	ble	.L2301
	rsb	r0, r4, fp
	mov	r1, r6
	bl	__aeabi_uidiv
	str	r0, [r5, #3972]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r9, sl]	@ movhi
.L2301:
	ldr	r5, .L2310
	ldr	r3, [r5, #-1884]
	cmp	r3, #1
	bne	.L2302
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	sl, .L2310+12
	ldrh	r9, [r5, sl]
	uxtah	r0, r9, r0
	add	r9, r9, r0, lsr #2
	strh	r9, [r5, sl]	@ movhi
.L2302:
	ldr	r3, .L2310+4
	ldrb	r3, [r3, #76]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2303
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	sl, .L2310
	ldr	r5, .L2310+12
	ldrh	r9, [sl, r5]
	uxtah	r0, r9, r0
	add	r9, r9, r0, lsr #2
	strh	r9, [sl, r5]	@ movhi
.L2303:
	ldr	r0, .L2310+4
	mov	r3, #3904
	ldrh	r3, [r0, r3]
	cmp	r3, #0
	beq	.L2304
	ldr	r1, .L2310
	ldr	r2, .L2310+12
	ldrh	ip, [r1, r2]
	add	ip, ip, r3, lsr #1
	strh	ip, [r1, r2]	@ movhi
	mul	ip, r6, r3
	cmp	ip, r4
	strgt	r7, [r0, #3972]
	addgt	r3, r3, #32
	addgt	r8, r8, r3
	strgth	r8, [r1, r2]	@ movhi
.L2304:
	ldr	r4, .L2310
	ldr	r5, .L2310+4
	ldr	r3, .L2310+12
	ldr	r7, .L2310+16
	ldr	r2, [r5, #3972]
	ldrh	r3, [r4, r3]
	rsb	r3, r3, r2
	mul	r6, r6, r3
	movw	r3, #3912
	ldrh	r3, [r5, r3]
	str	r6, [r4, #204]
	mul	r6, r3, r6
	movw	r3, #3918
	ldrh	r3, [r5, r3]
	str	r6, [r5, #3972]
	mul	r6, r3, r6
	str	r6, [r5, #3952]
	mvn	r6, #0
	bl	FtlBbmTblFlush
	movw	r3, #3854
	ldrh	r2, [r5, r3]
	mov	r1, #0
	ldr	r0, [r4, #-2064]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r1, .L2310+20
	mov	r3, #0
	ldr	r2, .L2310+24
	str	r3, [r4, #-1892]
	strh	r3, [r4, r1]	@ movhi
	sub	r1, r1, #288
	strb	r3, [r4, #-1750]
	strh	r3, [r4, r1]	@ movhi
	mov	r1, #255
	strb	r3, [r4, #-1748]
	strb	r3, [r4, #-2038]
	strh	r3, [r4, r7]	@ movhi
	mov	r3, #1
	strb	r3, [r4, #-2036]
	movw	r3, #3852
	strh	r6, [r4, r2]	@ movhi
	ldrh	r2, [r5, r3]
	ldr	r0, [r4, #-1888]
	mov	r2, r2, lsr #3
	bl	ftl_memset
.L2305:
	ldr	r0, .L2310+28
	bl	make_superblock
	ldrb	r3, [r4, #-2037]	@ zero_extendqisi2
	ldr	r5, .L2310
	cmp	r3, #0
	ldr	r3, .L2310+16
	bne	.L2306
	ldrh	r3, [r4, r7]
	ldr	r2, [r4, #-2064]
	mov	r3, r3, asl #1
	strh	r6, [r2, r3]	@ movhi
	ldrh	r3, [r4, r7]
	add	r3, r3, #1
	strh	r3, [r4, r7]	@ movhi
	b	.L2305
.L2306:
	ldr	r2, [r5, #-1848]
	mvn	r7, #0
	ldr	r1, [r5, #-2064]
	ldr	r6, .L2310+32
	str	r2, [r5, #-2032]
	add	r2, r2, #1
	str	r2, [r5, #-1848]
	ldr	r2, .L2310+36
	ldrh	r0, [r5, r2]
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #1
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	ldr	r0, .L2310+40
	ldrh	r3, [r5, r3]
	strb	r2, [r5, #-1990]
	add	r3, r3, #1
	strh	r2, [r5, r0]	@ movhi
	strh	r3, [r5, r6]	@ movhi
	mov	r3, #1
	strb	r3, [r5, #-1988]
.L2307:
	ldr	r0, .L2310+44
	bl	make_superblock
	ldrb	r3, [r5, #-1989]	@ zero_extendqisi2
	ldr	r4, .L2310
	cmp	r3, #0
	bne	.L2308
	ldrh	r3, [r5, r6]
	ldr	r2, [r5, #-2064]
	mov	r3, r3, asl #1
	strh	r7, [r2, r3]	@ movhi
	ldrh	r3, [r5, r6]
	add	r3, r3, #1
	strh	r3, [r5, r6]	@ movhi
	b	.L2307
.L2308:
	ldr	r3, [r4, #-1848]
	mvn	r5, #0
	ldr	r2, [r4, #-2064]
	str	r3, [r4, #-1984]
	add	r3, r3, #1
	str	r3, [r4, #-1848]
	ldr	r3, .L2310+48
	ldrh	r1, [r4, r3]
	sub	r3, r3, #4
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, .L2310+52
	strh	r5, [r4, r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, .L2310+56
	mov	r2, #0
	strh	r0, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	ldr	r2, [r4, #204]
	strh	r5, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #-1848]
	str	r3, [r4, #-1760]
	add	r3, r3, #1
	str	r3, [r4, #-1848]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	mov	r0, #0
	ldreq	r3, .L2310+60
	moveq	r2, #1
	streq	r2, [r3, #2960]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2311:
	.align	2
.L2310:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	168778952
	.word	-1772
	.word	-2044
	.word	-1754
	.word	-1756
	.word	.LANCHOR2-2044
	.word	-1996
	.word	-2040
	.word	-1994
	.word	.LANCHOR2-1996
	.word	-1992
	.word	-1948
	.word	-1768
	.word	.LANCHOR1
	.fnend
	.size	FtlLowFormat, .-FtlLowFormat
	.align	2
	.global	FlashMakeFactorBbt
	.type	FlashMakeFactorBbt, %function
FlashMakeFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 72
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #76
	sub	sp, sp, #76
	ldr	r4, .L2340
	movw	r1, #2210
	ldr	r3, .L2340+4
	mov	r5, #0
	ldr	r0, .L2340+8
	ldr	r2, [r4, #148]
	ldrh	r9, [r3, r1]
	mov	r1, #1
	str	r2, [sp, #28]
	mov	r2, #2208
	ldrh	r2, [r3, r2]
	mul	r9, r9, r2
	ldr	r2, [r3, #2772]
	ldrb	r2, [r2, #24]	@ zero_extendqisi2
	uxth	r9, r9
	str	r2, [sp, #12]
	ldrh	r2, [r3, #4]
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	cmp	r3, #1
	str	r2, [sp, #8]
	moveq	r3, r2, asl #1
	uxtheq	r3, r3
	streq	r3, [sp, #8]
	bl	printk
	ldr	r0, [r4, #148]
	mov	r1, #0
	mov	r2, #4096
	bl	ftl_memset
	mov	r3, r9, lsr #4
	str	r3, [sp, #20]
	sub	r3, r9, #1
	ldr	r4, .L2340+4
	uxth	r3, r3
	str	r3, [sp, #24]
	b	.L2314
.L2336:
	ldr	r3, .L2340
	add	r3, r3, r5, asl #1
	ldrh	r6, [r3, #208]
	cmp	r6, #0
	bne	.L2315
	movw	r3, #2216
	ldr	r0, [r4, #3776]
	ldrh	r2, [r4, r3]
	mov	r1, r6
	mov	r7, r6
	mov	sl, r6
	add	r8, r4, r5, asl #2
	mov	r2, r2, asl #9
	bl	ftl_memset
	add	r3, r4, r5
	ldrb	fp, [r3, #3764]	@ zero_extendqisi2
	b	.L2316
.L2324:
	ldr	r2, [sp, #12]
	mvn	r3, #0
	strb	r3, [sp, #34]
	tst	r2, #1
	strb	r3, [sp, #35]
	beq	.L2317
	ldr	r3, [r8, #2736]
	add	r2, sp, #34
	mov	r0, fp
	add	r3, r6, r3
	str	r3, [sp, #4]
	mov	r1, r3
	bl	FlashReadSpare
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	ldr	r3, [sp, #4]
	cmp	r2, #1
	bne	.L2317
	ldr	r1, [r4, #4]
	add	r2, sp, #35
	mov	r0, fp
	add	r1, r3, r1
	bl	FlashReadSpare
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	ldrb	r2, [sp, #35]	@ zero_extendqisi2
	and	r3, r2, r3
	strb	r3, [sp, #34]
.L2317:
	ldr	r3, [sp, #12]
	tst	r3, #2
	beq	.L2318
	ldr	r3, [r4, #2772]
	mov	r0, fp
	add	r2, sp, #35
	ldrh	r1, [r3, #10]
	ldr	r3, [r8, #2736]
	sub	r1, r1, #1
	add	r1, r1, r3
	add	r1, r1, r6
	bl	FlashReadSpare
.L2318:
	ldr	r3, [r4, #2772]
	ldrb	r3, [r3, #7]	@ zero_extendqisi2
	cmp	r3, #1
	cmpne	r3, #8
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	bne	.L2319
	cmp	r3, #0
	beq	.L2338
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	rsbs	r0, r0, #1
	movcc	r0, #0
	b	.L2320
.L2319:
	cmp	r3, #255
	bne	.L2338
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	subs	r0, r0, #255
	movne	r0, #1
	b	.L2320
.L2338:
	mov	r0, #1
.L2320:
	ldr	r2, [sp, #12]
	tst	r2, #4
	beq	.L2321
	ldr	r1, [r8, #2736]
	mov	r0, fp
	add	r1, r6, r1
	bl	SandiskProgTestBadBlock
.L2321:
	cmp	r0, #0
	beq	.L2322
	mov	r1, r5
	mov	r2, r7
	ldr	r0, .L2340+12
	add	sl, sl, #1
	bl	printk
	ldr	r3, [sp, #16]
	and	r0, r7, #31
	mov	ip, #1
	uxth	sl, sl
	mov	r2, r3, lsr #5
	ldr	r3, [r4, #3776]
	ldr	r1, [r3, r2, asl #2]
	orr	r1, r1, ip, asl r0
	str	r1, [r3, r2, asl #2]
	ldr	r2, [sp, #20]
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	mul	r3, r3, r2
	cmp	sl, r3
	bgt	.L2323
.L2322:
	ldr	r3, [sp, #8]
	add	r7, r7, #1
	add	r6, r6, r3
.L2316:
	uxth	r3, r7
	str	r3, [sp, #16]
	cmp	r3, r9
	bcc	.L2324
.L2323:
	mov	r2, sl
	ldr	r0, .L2340+16
	mov	r1, r5
	bl	printk
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	ldr	r2, [sp, #20]
	mul	r3, r3, r2
	cmp	sl, r3
	blt	.L2325
	movw	r3, #2216
	ldr	r0, [r4, #3776]
	ldrh	r2, [r4, r3]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2325:
	cmp	r5, #0
	bne	.L2326
	ldr	r3, .L2340
	mov	r8, r5
	mov	r7, r5
	mov	sl, #1
	ldrh	fp, [r3, #132]
	b	.L2327
.L2329:
	mov	r0, r6
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L2328
	mov	r1, r6
	ldr	r0, .L2340+20
	bl	printk
	ldr	r3, [r4, #3776]
	mov	r2, r6, lsr #5
	add	r7, r7, #1
	and	r6, r6, #31
	ldr	r1, [r3, r2, asl #2]
	uxth	r7, r7
	orr	r6, r1, sl, asl r6
	str	r6, [r3, r2, asl #2]
.L2328:
	add	r8, r8, #1
	uxth	r8, r8
.L2327:
	add	r6, r8, fp
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	uxth	r6, r6
	cmp	r3, r6
	bhi	.L2329
	ldr	r6, [sp, #24]
	sub	sl, r9, #50
	mov	r8, #1
	b	.L2330
.L2332:
	mov	r0, r6
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L2331
	mov	r1, r6
	ldr	r0, .L2340+20
	bl	printk
	ldr	r3, [r4, #3776]
	mov	r2, r6, lsr #5
	and	r0, r6, #31
	ldr	r1, [r3, r2, asl #2]
	orr	r1, r1, r8, asl r0
	str	r1, [r3, r2, asl #2]
.L2331:
	sub	r6, r6, #1
	uxth	r6, r6
.L2330:
	cmp	r6, sl
	bgt	.L2332
	ldr	r3, .L2340
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	ldr	r3, [r3, #132]
	rsb	r3, r3, r2
	cmp	r7, r3
	bcc	.L2326
	movw	r3, #2216
	ldr	r0, [r4, #3776]
	ldrh	r2, [r4, r3]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2326:
	mul	r8, r9, r5
	ldr	r7, .L2340
	ldr	r6, [sp, #24]
	ldr	fp, [sp, #28]
	add	sl, r7, r5, asl #1
.L2333:
	ldr	r0, .L2340+24
	mov	r1, r5
	mov	r2, r6
	bl	printk
	ldr	r3, [r4, #3776]
	b	.L2334
.L2335:
	sub	r6, r6, #1
	uxth	r6, r6
.L2334:
	mov	r1, r6, lsr #5
	and	r2, r6, #31
	ldr	r1, [r3, r1, asl #2]
	mov	r2, r1, lsr r2
	ands	r2, r2, #1
	bne	.L2335
	ldr	r3, .L2340+28
	mov	r1, #1
	strh	r6, [sl, #208]	@ movhi
	add	r0, sp, #36
	strh	r6, [fp, #2]	@ movhi
	strh	r3, [fp, #0]	@ movhi
	ldr	r3, [r4, #3776]
	strh	r2, [fp, #8]	@ movhi
	mov	r2, r1
	str	r3, [sp, #44]
	ldr	r3, [r7, #148]
	str	r3, [sp, #48]
	add	r3, r6, r8
	mov	r3, r3, asl #10
	str	r3, [sp, #40]
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	add	r0, sp, #36
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [sp, #36]
	cmp	r3, #0
	subne	r6, r6, #1
	uxthne	r6, r6
	bne	.L2333
.L2315:
	add	r5, r5, #1
	uxtb	r5, r5
.L2314:
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	cmp	r3, r5
	bhi	.L2336
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2341:
	.align	2
.L2340:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC127
	.word	.LC128
	.word	.LC129
	.word	.LC130
	.word	.LC131
	.word	-3872
	.fnend
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
	.align	2
	.global	FtlReInitForSDUpdata
	.type	FtlReInitForSDUpdata, %function
FtlReInitForSDUpdata:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2367
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #20
	sub	sp, sp, #20
	ldrb	r4, [r3, #76]	@ zero_extendqisi2
	cmp	r4, #0
	movne	r0, #0
	bne	.L2343
	ldr	r5, .L2367+4
	ldr	r0, [r5, #112]
	bl	FlashInit
	cmp	r0, #0
	movne	r0, r4
	bne	.L2343
	bl	FlashLoadFactorBbt
	cmp	r0, #0
	beq	.L2344
	bl	FlashMakeFactorBbt
.L2344:
	ldr	r0, [r5, #116]
	bl	FlashReadIdbDataRaw
	cmp	r0, #0
	beq	.L2345
	mov	r1, #0
	mov	r2, #16
	mov	r0, sp
	bl	FlashReadFacBbtData
	ldr	r1, [sp, #0]
	mov	r3, #0
	mov	r0, #1
	mov	r2, r3
.L2347:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #16
	bne	.L2347
	cmp	r3, #6
	ldrls	r3, .L2367
	bls	.L2363
	mov	r2, #0
	mov	r0, #1
.L2348:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #24
	bne	.L2348
	cmp	r3, #17
	ldr	r3, .L2367
	movhi	r2, #36
.L2363:
	strb	r2, [r3, #1]
	movw	r2, #2222
	ldr	r3, .L2367
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	strh	r1, [r3, r2]	@ movhi
.L2345:
	ldr	r1, .L2367+8
	mov	r4, #1
	ldr	r0, .L2367+12
	bl	printk
	ldr	r0, .L2367+16
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldr	r3, .L2367
	ldr	r0, [r3, #3848]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	b	.L2352
.L2354:
	add	r4, r4, #1
.L2352:
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L2365
.L2353:
	bl	FtlSysBlkInit
	cmp	r0, #0
	beq	.L2355
.L2365:
	bl	FtlLowFormat
	cmp	r4, #3
	bls	.L2354
	b	.L2366
.L2355:
	ldr	r3, .L2367+20
	mov	r2, #1
	str	r2, [r3, #2960]
	b	.L2343
.L2366:
	mvn	r0, #0
.L2343:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, pc}
.L2368:
	.align	2
.L2367:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC75
	.word	.LC74
	.word	.LANCHOR0+2196
	.word	.LANCHOR1
	.fnend
	.size	FtlReInitForSDUpdata, .-FtlReInitForSDUpdata
	.align	2
	.global	flash_boot_enter_slc_mode
	.type	flash_boot_enter_slc_mode, %function
flash_boot_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L2371
	ldr	r2, [r3, #3788]
	ldr	r3, .L2371+4
	cmp	r2, r3
	bxne	lr
	b	flash_enter_slc_mode
.L2372:
	.align	2
.L2371:
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
	ldr	r3, .L2375
	ldr	r2, [r3, #3788]
	ldr	r3, .L2375+4
	cmp	r2, r3
	bxne	lr
	b	flash_exit_slc_mode
.L2376:
	.align	2
.L2375:
	.word	.LANCHOR0
	.word	1446522928
	.fnend
	.size	flash_boot_exit_slc_mode, .-flash_boot_exit_slc_mode
	.align	2
	.global	IdBlockReadData
	.type	IdBlockReadData, %function
IdBlockReadData:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L2382
	mov	r6, r0
	str	r2, [sp, #20]
	mov	sl, r1
	ldr	r0, .L2382+4
	mov	r1, r6
	ldr	r2, [r4, #2772]
	ldr	r3, [r4, #4]
	ldr	r8, .L2382+8
	ldrb	r5, [r2, #9]	@ zero_extendqisi2
	mov	r2, sl
	mul	r5, r5, r3
	bl	printk
	ldr	r3, [r4, #2772]
	mov	r0, r6
	ldrb	r9, [r3, #9]	@ zero_extendqisi2
	uxth	r5, r5
	mov	r1, r5
	mov	r5, #0
	bl	__aeabi_uidivmod
	mul	r2, r9, r1
	rsb	ip, r1, r6
	mov	r7, r1
	str	ip, [sp, #12]
	ubfx	r2, r2, #2, #2
	b	.L2378
.L2380:
	add	r1, r5, r7
	rsb	r3, r2, #4
	ubfx	r1, r1, #2, #16
	ldrb	r0, [r4, #76]	@ zero_extendqisi2
	uxth	r3, r3
	str	r3, [sp, #16]
	add	r3, r4, r1, asl #1
	cmp	r0, #0
	add	r3, r3, #2224
	ldrh	r3, [r3, #0]
	beq	.L2379
	ldr	r0, [r4, #3788]
	ldr	ip, .L2382+12
	cmp	r0, ip
	moveq	r3, r1
.L2379:
	ldr	ip, [sp, #12]
	ldrb	r0, [r8, #128]	@ zero_extendqisi2
	add	r2, r2, ip
	ldrb	ip, [r4, #3836]	@ zero_extendqisi2
	ldr	fp, [r8, #116]
	mla	r2, r9, r3, r2
	ldr	r3, [r4, #2772]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	str	ip, [sp, #0]
	stmib	sp, {r1, r2}
	bl	FlashBchSel
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r2, [sp, #8]
	ldr	r1, [sp, #4]
	mov	r0, r2
	bl	__aeabi_uidiv
	mov	r2, fp
	mov	r1, r0
	mov	r0, #0
	mov	r3, r0
	bl	FlashReadPage
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	ip, [sp, #0]
	mov	r0, ip
	bl	FlashBchSel
	ldr	ip, [sp, #20]
	mov	r2, #2048
	ldr	r1, [r8, #116]
	add	r0, ip, r5, asl #9
	bl	memcpy
	ldr	ip, [sp, #16]
	mov	r2, #0
	add	r5, ip, r5
	uxth	r5, r5
.L2378:
	cmp	r5, sl
	bcc	.L2380
	mov	r1, r6
	mov	r2, sl
	mov	r3, #0
	ldr	r0, .L2382+16
	bl	printk
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2383:
	.align	2
.L2382:
	.word	.LANCHOR0
	.word	.LC132
	.word	.LANCHOR2
	.word	1446522928
	.word	.LC133
	.fnend
	.size	IdBlockReadData, .-IdBlockReadData
	.align	2
	.global	IDBlockWriteData
	.type	IDBlockWriteData, %function
IDBlockWriteData:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #92
	sub	sp, sp, #92
	ldr	r4, .L2391
	mov	r5, r0
	str	r2, [sp, #20]
	mov	r8, r1
	ldr	r0, .L2391+4
	mov	r1, r5
	ldr	r2, [r4, #2772]
	mov	r7, r5
	ldr	r3, [r4, #4]
	ldr	fp, .L2391+8
	ldrb	r6, [r2, #9]	@ zero_extendqisi2
	mov	r2, r8
	mul	r6, r6, r3
	bl	printk
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	movw	r3, #2216
	ldrh	r1, [r4, r3]
	mov	r0, r5
	bl	__aeabi_uidiv
	uxth	r6, r6
	mov	r1, r0
	mov	r0, #0
	mov	r2, r0
	bl	FlashEraseBlock
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r3, [r4, #2772]
	mov	r1, r6
	mov	r0, r5
	mov	r6, #0
	ldrb	sl, [r3, #9]	@ zero_extendqisi2
	bl	__aeabi_uidivmod
	mov	r9, r1
	rsb	r1, r1, r5
	str	r1, [sp, #16]
	b	.L2385
.L2389:
	add	r3, r6, r9
	ubfx	r3, r3, #2, #16
	cmp	r3, #0
	beq	.L2386
	add	r1, r3, #1
	ldrb	r0, [r4, #76]	@ zero_extendqisi2
	add	r2, r4, r1, asl #1
	cmp	r0, #0
	add	r2, r2, #2224
	ldrh	r2, [r2, #0]
	beq	.L2387
	ldr	r0, [r4, #3788]
	cmp	r0, fp
	uxtheq	r2, r1
.L2387:
	sub	r2, r2, #1
	mov	r2, r2, asl #2
	str	r2, [sp, #24]
	mov	r2, #0
	str	r2, [sp, #28]
.L2386:
	add	r2, r4, r3, asl #1
	add	r2, r2, #2224
	ldrh	r5, [r2, #0]
	ldrb	r2, [r4, #76]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2388
	ldr	r2, [r4, #3788]
	cmp	r2, fp
	moveq	r5, r3
.L2388:
	ldr	r2, [sp, #16]
	ldr	r1, [sp, #20]
	ldrb	ip, [r4, #3836]	@ zero_extendqisi2
	mla	r3, sl, r5, r2
	add	r2, r1, r6, asl #9
	ldr	r1, .L2391+12
	add	r6, r6, #4
	uxth	r6, r6
	ldrb	r0, [r1, #128]	@ zero_extendqisi2
	str	ip, [sp, #4]
	str	r2, [sp, #8]
	str	r3, [sp, #12]
	bl	FlashBchSel
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r3, [sp, #12]
	ldr	r1, [r4, #2772]
	mov	r0, r3
	ldrb	r1, [r1, #9]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r2, [sp, #8]
	add	r3, sp, #24
	mov	r1, r0
	mov	r0, #0
	bl	FlashProgPage
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	ip, [sp, #4]
	mov	r0, ip
	bl	FlashBchSel
	ldr	r0, [sp, #16]
	mov	r1, sl
	bl	__aeabi_uidiv
	add	r2, r5, #1
	uxth	r2, r2
	mov	r1, r0
	mov	r0, #0
	bl	FlashPageProgMsbFFData
.L2385:
	cmp	r6, r8
	bcc	.L2389
	mov	r1, r7
	mov	r2, r8
	mov	r3, #0
	ldr	r0, .L2391+16
	bl	printk
	mov	r5, r7
	mov	r0, #0
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2392:
	.align	2
.L2391:
	.word	.LANCHOR0
	.word	.LC134
	.word	1446522928
	.word	.LANCHOR2
	.word	.LC135
	.fnend
	.size	IDBlockWriteData, .-IDBlockWriteData
	.align	2
	.global	write_idblock
	.type	write_idblock, %function
write_idblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2406
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r7, r2
	ldr	r2, [r3, #4]
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r3, #2772]
	mov	r6, r0
	mov	r0, #256000
	mov	r4, r1
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	mul	r3, r3, r2
	uxth	r3, r3
	str	r3, [sp, #8]
	bl	ftl_malloc
	subs	r5, r0, #0
	mvneq	r0, #0
	beq	.L2394
	add	sl, r6, #508
	add	sl, sl, #3
	mov	sl, sl, lsr #9
	cmp	sl, #255
	bhi	.L2395
	add	r0, r4, sl, asl #9
	mov	r1, r4
	rsb	r2, sl, #256
	bl	memcpy
.L2395:
	ldr	r8, .L2406
	mov	r3, #5
	ldr	r0, .L2406+4
	mov	r1, r7
	mov	r2, #4
	add	sl, sl, #128
	bl	rknand_print_hex
	ldrb	r2, [r8, #1]	@ zero_extendqisi2
	ldr	r1, [r4, #512]
	cmp	sl, #256
	movcs	sl, #256
	ldr	r0, .L2406+8
	mov	fp, #0
	bl	printk
	ldrb	r3, [r8, #1]	@ zero_extendqisi2
	ldr	r2, [r4, #512]
	mov	r1, sl
	ldr	r0, .L2406+12
	mov	r9, fp
	cmp	r2, r3
	mov	r8, r4
	mov	r2, r6
	strhi	r3, [r4, #512]
	mov	r6, r7
	bl	printk
	mov	r1, sl, asl #7
	str	r1, [sp, #12]
.L2401:
	ldr	r1, .L2406
	movw	r2, #2222
	ldr	r3, [r6], #4
	ldrh	r2, [r1, r2]
	cmp	r3, r2
	bcs	.L2397
	ldr	r2, .L2406+16
	ldr	r2, [r2, #132]
	cmp	r3, r2
	bcc	.L2397
	mov	r0, r5
	mov	r1, #512
	bl	__memzero
	ldr	r2, [sp, #8]
	ldr	r0, [r6, #-4]
	mov	r1, sl
	mul	r0, r0, r2
	mov	r2, r8
	bl	IDBlockWriteData
	ldr	r3, [sp, #8]
	ldr	r0, [r6, #-4]
	mov	r1, sl
	mov	r2, r5
	mul	r0, r0, r3
	bl	IdBlockReadData
	mov	r3, #0
	mov	r7, r3
.L2400:
	ldr	r1, [r5, r3]
	add	r3, r3, #4
	add	r2, r8, r3
	ldr	r2, [r2, #-4]
	cmp	r1, r2
	beq	.L2398
	bic	r4, r7, #255
	stmia	sp, {r1, r2}
	mov	r3, r7
	mov	r1, r9
	ldr	r2, [r6, #-4]
	mov	r4, r4, asl #2
	ldr	r0, .L2406+20
	bl	printk
	ldr	r0, .L2406+24
	add	r1, r8, r4
	mov	r2, #4
	mov	r3, #256
	bl	rknand_print_hex
	mov	r3, #256
	mov	r2, #4
	ldr	r0, .L2406+28
	add	r1, r5, r4
	bl	rknand_print_hex
	mov	r0, r5
	mov	r1, #512
	bl	__memzero
	ldr	r1, [sp, #8]
	ldr	r0, [r6, #-4]
	mov	r2, r5
	mul	r0, r0, r1
	mov	r1, #4
	bl	IDBlockWriteData
	ldr	r0, .L2406+32
	bl	printk
	ldr	r2, [sp, #12]
	cmp	r7, r2
	bcc	.L2397
	b	.L2399
.L2398:
	ldr	r1, [sp, #12]
	add	r7, r7, #1
	cmp	r7, r1
	bne	.L2400
.L2399:
	add	fp, fp, #1
.L2397:
	add	r9, r9, #1
	cmp	r9, #5
	bne	.L2401
	mov	r0, r5
	bl	ftl_free
	cmp	fp, #0
	mvneq	r0, #0
	movne	r0, #0
.L2394:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2407:
	.align	2
.L2406:
	.word	.LANCHOR0
	.word	.LC136
	.word	.LC137
	.word	.LC138
	.word	.LANCHOR2
	.word	.LC139
	.word	.LC140
	.word	.LC141
	.word	.LC142
	.fnend
	.size	write_idblock, .-write_idblock
	.align	2
	.global	CRC_32
	.type	CRC_32, %function
CRC_32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	r3, r0
	mov	r0, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r2, r0
	ldr	ip, .L2411
	b	.L2409
.L2410:
	ldrb	r4, [r3, r2]	@ zero_extendqisi2
	add	r2, r2, #1
	eor	r4, r4, r0, lsr #24
	add	r4, ip, r4, asl #2
	ldr	r4, [r4, #3008]
	eor	r0, r4, r0, asl #8
.L2409:
	cmp	r2, r1
	bne	.L2410
	ldmfd	sp!, {r4, pc}
.L2412:
	.align	2
.L2411:
	.word	.LANCHOR1
	.fnend
	.size	CRC_32, .-CRC_32
	.align	2
	.global	rknand_sys_storage_ioctl
	.type	rknand_sys_storage_ioctl, %function
rknand_sys_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 528
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2529
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	cmp	r1, r3
	.pad #528
	sub	sp, sp, #528
	mov	r5, r1
	mov	r4, r2
	beq	.L2422
	bhi	.L2431
	ldr	r3, .L2529+4
	cmp	r1, r3
	beq	.L2419
	bhi	.L2432
	sub	r3, r3, #125
	cmp	r1, r3
	beq	.L2416
	bhi	.L2433
	sub	r3, r3, #237
	cmp	r1, r3
	bne	.L2474
	b	.L2525
.L2433:
	ldr	r3, .L2529+8
	cmp	r1, r3
	beq	.L2417
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2474
	b	.L2526
.L2432:
	ldr	r3, .L2529+12
	cmp	r1, r3
	beq	.L2421
	bhi	.L2434
	ldr	r3, .L2529+16
	cmp	r1, r3
	bne	.L2474
	b	.L2527
.L2434:
	ldr	r3, .L2529+20
	cmp	r1, r3
	beq	.L2421
	add	r3, r3, #10
	cmp	r1, r3
	bne	.L2474
	b	.L2421
.L2431:
	ldr	r3, .L2529+24
	cmp	r1, r3
	beq	.L2427
	bhi	.L2435
	sub	r3, r3, #78
	cmp	r1, r3
	beq	.L2424
	bcc	.L2423
	add	r3, r3, #21
	cmp	r1, r3
	beq	.L2425
	add	r3, r3, #56
	cmp	r1, r3
	bne	.L2474
	b	.L2528
.L2435:
	ldr	r3, .L2529+28
	cmp	r1, r3
	beq	.L2430
	bhi	.L2436
	sub	r3, r3, #956
	sub	r3, r3, #1
	cmp	r1, r3
	beq	.L2428
	add	r3, r3, #956
	cmp	r1, r3
	bne	.L2474
	b	.L2429
.L2436:
	ldr	r3, .L2529+32
	cmp	r1, r3
	beq	.L2429
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2474
	b	.L2430
.L2423:
	ldr	r0, .L2529+36
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2482
	mov	r1, r4
	mov	r2, #512
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2524
.L2437:
	ldr	r0, .L2529+40
	ldmia	r5, {r1, r2}
	bl	printk
	ldr	r3, [r5, #4]
	cmp	r3, #8
	str	r3, [sp, #4]
	bhi	.L2445
	bl	rknand_device_lock
	ldr	r1, [sp, #4]
	mov	r2, r5
	ldr	r0, [r5, #0]
	bl	IdBlockReadData
	bl	rknand_device_unlock
	ldr	r2, [sp, #4]
	mov	r0, r4
	mov	r1, r5
	mov	r2, r2, asl #9
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L2510
	ldr	r0, .L2529+44
.L2523:
	bl	printk
	b	.L2445
.L2424:
	ldr	r0, .L2529+48
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2482
	mov	r1, r4
	mov	r2, #4096
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2524
.L2441:
	ldr	r4, .L2529+52
	ldr	r0, .L2529+56
	ldmia	r5, {r1, r2}
	bl	printk
	ldr	r3, [r4, #436]
	cmp	r3, #0
	bne	.L2442
	mov	r0, #260096
	bl	ftl_malloc
	cmp	r0, #0
	str	r0, [r4, #436]
	beq	.L2445
.L2442:
	ldr	r2, [r5, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L2445
	ldr	r3, [r5, #0]
	cmp	r3, #251904
	bhi	.L2445
	ldr	r1, .L2529+52
	ldr	r0, [r1, #436]
	add	r1, r5, #8
	add	r0, r0, r3
	bl	memcpy
.L2510:
	mov	r0, r5
	bl	ftl_free
.L2511:
	mov	r4, #0
	b	.L2440
.L2528:
	ldr	r0, .L2529+60
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2482
	mov	r1, r4
	mov	r2, #28
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L2444
.L2524:
	ldr	r0, .L2529+64
	b	.L2523
.L2444:
	ldmia	r5, {r1, r2}
	ldr	r0, .L2529+68
	bl	printk
	ldr	r1, [r5, #0]
	cmp	r1, #256000
	bhi	.L2445
	ldr	r4, .L2529+52
	ldr	r0, [r4, #436]
	cmp	r0, #0
	beq	.L2445
.L2446:
	bl	CRC_32
	ldr	r3, [r5, #4]
	cmp	r3, r0
	beq	.L2447
	mov	r0, r5
	bl	ftl_free
	b	.L2488
.L2447:
	bl	rknand_device_lock
	ldr	r1, [r4, #436]
	add	r2, r5, #8
	ldr	r0, [r5, #0]
	bl	write_idblock
	mov	r6, #0
	bl	rknand_device_unlock
	ldr	r0, [r4, #436]
	bl	ftl_free
	str	r6, [r4, #436]
	mov	r0, r5
	mov	r4, r6
	bl	ftl_free
	b	.L2440
.L2445:
	mov	r0, r5
	b	.L2514
.L2425:
	ldr	r0, .L2529+72
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2482
	bl	ReadFlashInfo
	mov	r0, r4
	mov	r1, r5
	mov	r2, #11
	b	.L2521
.L2422:
	ldr	r0, .L2529+76
	bl	printk
	bl	rknand_device_lock
	bl	FtlReInitForSDUpdata
	mov	r6, r0
	bl	rknand_device_unlock
	cmp	r6, #0
	bne	.L2505
	bl	nand_blk_add_whole_disk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2482
	bl	rknand_device_lock
	mov	r1, r6
	mov	r2, #64
	mov	r0, r5
	bl	FlashReadFacBbtData
	bl	rknand_device_unlock
	ldr	r0, .L2529+80
	mov	r1, r5
	mov	r2, #4
	mov	r3, #8
	bl	rknand_print_hex
	mov	r0, r4
	mov	r1, r5
	mov	r2, #64
	b	.L2521
.L2427:
	ldr	r0, .L2529+84
	bl	printk
	ldr	r3, .L2529+52
	add	r1, sp, #528
	mov	r0, r4
	mov	r2, #4
	ldr	r3, [r3, #440]
	ldr	r3, [r3, #20]
	str	r3, [r1, #-524]!
	b	.L2518
.L2428:
	ldr	r0, .L2529+88
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2482
	bl	rknand_device_lock
	mov	r1, #264
	mov	r2, #2
	mov	r3, r5
	mov	r0, #16
	bl	FtlRead
	bl	rknand_device_unlock
	mov	r0, r4
	mov	r1, r5
	mov	r2, #1024
.L2521:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	mov	r0, r5
	beq	.L2450
.L2514:
	bl	ftl_free
	b	.L2505
.L2450:
	bl	ftl_free
	b	.L2440
.L2416:
	ldr	r0, .L2529+92
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r5, r0, #0
	bne	.L2515
.L2451:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+96
	cmp	r2, r3
	bne	.L2507
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2507
	ldr	r6, .L2529+52
	mov	r2, #512
	add	r0, sp, #8
	ldr	r1, [r6, #440]
	bl	memcpy
	ldr	r2, [r6, #444]
	ldr	r3, .L2529+100
	cmp	r2, r3
	beq	.L2452
	add	r0, sp, #72
	mov	r1, #128
	str	r5, [sp, #16]
	str	r5, [sp, #20]
	bl	__memzero
.L2452:
	add	r0, sp, #264
	mov	r1, #256
	mov	r3, #0
	str	r3, [sp, #24]
	bl	__memzero
	b	.L2517
.L2417:
	ldr	r0, .L2529+104
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2515
.L2453:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+96
	cmp	r2, r3
	bne	.L2507
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2507
	ldr	r4, .L2529+52
	ldr	r3, .L2529+100
	ldr	r2, [r4, #444]
	cmp	r2, r3
	bne	.L2488
	ldr	r3, [sp, #20]
	sub	r2, r3, #1
	cmp	r2, #127
	mvnhi	r4, #2
	bhi	.L2414
	ldr	r0, [r4, #440]
	add	r1, sp, #72
	str	r3, [r0, #12]
	add	r0, r0, #64
	ldr	r2, [sp, #20]
	bl	memcpy
	mov	r0, #1
	ldr	r1, [r4, #440]
	b	.L2512
.L2527:
	ldr	r0, .L2529+108
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2515
.L2454:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+112
	cmp	r2, r3
	bne	.L2507
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2507
	ldr	r5, .L2529+52
	ldr	r4, [r5, #448]
	cmp	r4, #0
	beq	.L2414
	ldr	r3, [r5, #452]
	ldr	r2, .L2529+116
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2455
	str	r2, [r3, #0]
	mov	r2, #504
	ldr	r3, [r5, #452]
	str	r2, [r3, #4]
	mov	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L2455:
	ldr	r1, [r5, #452]
	mov	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #440]
	ldr	r2, .L2529+96
	ldr	r5, .L2529+52
	ldr	r1, [r3, #0]
	cmp	r1, r2
	strne	r2, [r3, #0]
	ldr	r0, [r5, #440]
	mov	r1, #128
	ldrne	r3, .L2529+52
	movne	r2, #504
	ldrne	r3, [r3, #440]
	stmneib	r3, {r2, r4}
	mov	r4, #0
	str	r4, [r0, #12]
	add	r0, r0, #64
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #440]
	bl	StorageSysDataStore
	str	r4, [r5, #448]
	str	r4, [r5, #444]
	b	.L2440
.L2419:
	ldr	r0, .L2529+120
	bl	printk
	mov	r1, r4
	add	r0, sp, #8
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r4, r0, #0
	bne	.L2515
.L2457:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+124
	cmp	r2, r3
	bne	.L2507
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2507
	ldr	r5, .L2529+52
	ldr	r3, [r5, #448]
	cmp	r3, #1
	beq	.L2414
	ldr	r3, [r5, #452]
	mov	r0, #0
	ldr	r2, .L2529+116
	ldr	r1, [r3, #0]
	cmp	r1, r2
	strne	r2, [r3, #0]
	ldr	r1, [r5, #452]
	ldrne	r3, [r5, #452]
	movne	r2, #504
	strne	r4, [r3, #12]
	stmneib	r3, {r2, r4}
	mov	r3, #1
	str	r3, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #440]
	ldr	r2, .L2529+96
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2459
	str	r2, [r3, #0]
	mov	r1, #504
	ldr	r3, .L2529+52
	mov	r2, #0
	ldr	r3, [r3, #440]
	stmib	r3, {r1, r2}
.L2459:
	ldr	r5, .L2529+52
	mov	r1, #128
	mov	r4, #0
	ldr	r0, [r5, #440]
	str	r4, [r0, #12]
	add	r0, r0, #64
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #440]
	bl	StorageSysDataStore
	mov	r3, #1
	str	r3, [r5, #448]
	b	.L2440
.L2526:
	ldr	r0, .L2529+128
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2515
.L2460:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+132
	cmp	r2, r3
	bne	.L2507
	ldr	r2, [sp, #12]
	cmp	r2, #512
	addls	r0, sp, #16
	ldrls	r1, .L2529+136
	bls	.L2516
	b	.L2507
.L2421:
	ldr	r3, .L2529+20
	cmp	r5, r3
	ldreq	r0, .L2529+140
	beq	.L2508
	ldr	r3, .L2529+144
	cmp	r5, r3
	ldreq	r0, .L2529+148
	ldrne	r0, .L2529+152
.L2508:
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2515
.L2464:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+156
	cmp	r2, r3
	bne	.L2505
	ldr	r3, .L2529+144
	ldr	r6, .L2529+52
	cmp	r5, r3
	bne	.L2465
	ldr	r3, [r6, #440]
	mov	r0, r4
	add	r1, sp, #8
	mov	r2, #16
	ldr	r3, [r3, #20]
	str	r3, [sp, #12]
	strb	r3, [sp, #16]
	bl	rk_copy_to_user
	cmp	r0, #0
	moveq	r4, r0
	mvnne	r4, #13
	b	.L2414
.L2465:
	ldr	r3, [r6, #968]
	cmp	r3, #10
	bhi	.L2505
	ldr	r3, [r6, #440]
	ldr	r1, [sp, #12]
	ldr	r2, [r3, #24]
	cmp	r2, r1
	beq	.L2466
	cmp	r2, #0
	beq	.L2466
	ldr	r0, .L2529+160
	bl	printk
	ldr	r3, [r6, #968]
	add	r3, r3, #1
	str	r3, [r6, #968]
	b	.L2505
.L2466:
	ldr	r0, .L2529+52
	mov	r2, #0
	str	r2, [r0, #968]
	ldr	r0, .L2529+20
	cmp	r5, r0
	mov	r0, #1
	strne	r1, [r3, #24]
	mov	r1, r3
	movne	r2, #1
	streq	r2, [r3, #20]
	streq	r2, [r3, #24]
	strne	r2, [r3, #20]
	bl	StorageSysDataStore
	cmn	r0, #1
	mvneq	r4, #1
	movne	r4, #0
	b	.L2440
.L2429:
	ldr	r0, .L2529+164
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2515
.L2469:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+168
	cmp	r2, r3
	bne	.L2507
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L2507
	ldr	r3, .L2529+172
	add	r0, sp, #16
	cmp	r5, r3
	ldr	r3, .L2529+52
	ldreq	r1, [r3, #972]
	ldrne	r1, [r3, #976]
	add	r1, r1, #8
.L2516:
	bl	memcpy
.L2517:
	add	r1, sp, #8
	mov	r0, r4
	mov	r2, #520
.L2518:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	bne	.L2505
	b	.L2440
.L2430:
	ldr	r0, .L2529+176
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L2472
.L2515:
	ldr	r0, .L2529+64
	bl	printk
	b	.L2505
.L2472:
	ldr	r2, [sp, #8]
	ldr	r3, .L2529+168
	cmp	r2, r3
	bne	.L2507
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L2507
	ldr	r3, .L2529+28
	add	r2, r2, #8
	ldr	r4, .L2529+52
	cmp	r5, r3
	bne	.L2473
	add	r1, sp, #8
	ldr	r0, [r4, #972]
	bl	memcpy
	ldr	r1, [r4, #972]
	mov	r0, #2
.L2512:
	bl	StorageSysDataStore
	mov	r4, r0
	b	.L2440
.L2473:
	add	r1, sp, #8
	ldr	r0, [r4, #976]
	bl	memcpy
	mov	r0, #3
	ldr	r1, [r4, #976]
	b	.L2512
.L2525:
	bl	rknand_dev_flush
	b	.L2511
.L2507:
	mvn	r4, #0
.L2440:
	mov	r1, r4
	ldr	r0, .L2529+180
	bl	printk
	b	.L2414
.L2474:
	mvn	r4, #21
	b	.L2414
.L2482:
	mvn	r4, #11
	b	.L2414
.L2488:
	mvn	r4, #1
	b	.L2414
.L2505:
	mvn	r4, #13
.L2414:
	mov	r0, r4
	add	sp, sp, #528
	ldmfd	sp!, {r4, r5, r6, pc}
.L2530:
	.align	2
.L2529:
	.word	1074033155
	.word	1074029694
	.word	1074029570
	.word	1074031656
	.word	1074029695
	.word	1074031666
	.word	1074033235
	.word	1074034193
	.word	1074034194
	.word	.LC143
	.word	.LC145
	.word	.LC146
	.word	.LC147
	.word	.LANCHOR2
	.word	.LC148
	.word	.LC149
	.word	.LC144
	.word	.LC150
	.word	.LC151
	.word	.LC152
	.word	.LC153
	.word	.LC154
	.word	.LC155
	.word	.LC156
	.word	1263358532
	.word	-1067903959
	.word	.LC157
	.word	.LC158
	.word	1112753220
	.word	1146313043
	.word	.LC159
	.word	1112755781
	.word	.LC160
	.word	1094995539
	.word	.LANCHOR2+456
	.word	.LC161
	.word	1074031676
	.word	.LC162
	.word	.LC163
	.word	1280262987
	.word	.LC164
	.word	.LC165
	.word	1145980246
	.word	1074034192
	.word	.LC166
	.word	.LC167
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
	ldr	r4, .L2534
	mov	r6, #0
	add	r0, r4, #456
	ldr	r5, [r4, #416]
	str	r6, [r4, #436]
	add	r1, r5, #1536
	add	r3, r5, #512
	str	r5, [r4, #452]
	str	r3, [r4, #440]
	add	r3, r5, #1024
	str	r1, [r4, #976]
	str	r3, [r4, #972]
	bl	memcpy
	ldr	r7, [r5, #508]
	ldr	r3, [r5, #16]
	cmp	r7, r6
	str	r6, [r4, #444]
	str	r6, [r4, #968]
	str	r3, [r4, #448]
	beq	.L2532
	mov	r0, r5
	mov	r1, #508
	bl	JSHash
	cmp	r7, r0
	beq	.L2532
	str	r6, [r5, #16]
	ldr	r0, .L2534+4
	str	r6, [r4, #448]
	bl	printk
.L2532:
	ldr	r3, [r4, #448]
	mov	r0, #2
	ldr	r4, .L2534
	cmp	r3, #0
	ldrne	r2, .L2534+8
	ldrne	r3, .L2534
	ldr	r1, [r4, #972]
	strne	r2, [r3, #444]
	bl	StorageSysDataLoad
	ldr	r1, [r4, #976]
	mov	r0, #3
	bl	StorageSysDataLoad
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	rknand_sys_storage_init
.L2535:
	.align	2
.L2534:
	.word	.LANCHOR2
	.word	.LC168
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
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r0, #65536
	mov	r1, #209
	mov	r2, #4
	bl	kmalloc_order_trace
	ldr	r7, .L2545
	cmp	r0, #0
	str	r0, [r7, #980]
	beq	.L2543
	ldr	sl, .L2545+4
	mov	r6, #0
	mov	r4, r6
	mov	r5, r6
	movw	r8, #65532
.L2541:
	ldr	r0, [r7, #980]
	mov	r1, r5, asl #7
	mov	r2, #0
	bl	rk_ftl_vendor_ops.constprop.19
	cmp	r0, #0
	bne	.L2539
	ldr	r3, [r7, #980]
	ldr	r0, .L2545+8
	ldr	r1, [r3, #0]
	ldr	r2, [r3, r8]
	ldr	r3, [r3, #4]
	bl	printk
	ldr	r0, [r7, #980]
	ldr	r3, [r0, #0]
	cmp	r3, sl
	bne	.L2540
	ldr	r3, [r0, r8]
	ldr	r2, [r0, #4]
	cmp	r3, r2
	bne	.L2540
	cmp	r4, r3
	movcc	r6, r5
	movcc	r4, r3
.L2540:
	add	r5, r5, #1
	cmp	r5, #2
	bne	.L2541
	cmp	r4, #0
	beq	.L2542
	mov	r1, r6, asl #7
	mov	r2, #0
	bl	rk_ftl_vendor_ops.constprop.19
	cmp	r0, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	b	.L2539
.L2542:
	mov	r1, #65536
	bl	__memzero
	ldr	r3, .L2545
	ldr	r1, .L2545+4
	mov	r2, #1
	mov	r0, r4
	ldr	r3, [r3, #980]
	stmia	r3, {r1, r2}
	movw	r1, #65532
	str	r2, [r3, r1]
	ldr	r2, .L2545+12
	strh	r4, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2539:
	ldr	r4, .L2545
	ldr	r0, [r4, #980]
	bl	kfree
	mov	r3, #0
	mvn	r0, #0
	str	r3, [r4, #980]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2543:
	mvn	r0, #11
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2546:
	.align	2
.L2545:
	.word	.LANCHOR2
	.word	1380668996
	.word	.LC169
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
	mov	r3, r0
	mov	r0, r1
	ldr	r1, .L2554
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	ip, [r1, #980]
	cmp	ip, #0
	ldrneh	r4, [ip, #10]
	movne	r5, ip
	movne	r1, #0
	bne	.L2549
	b	.L2553
.L2551:
	ldrh	r6, [r5, #16]
	add	r5, r5, #8
	cmp	r6, r3
	bne	.L2550
	add	r1, r1, #2
	add	r1, ip, r1, asl #3
	ldrh	r4, [r1, #4]
	ldrh	r1, [r1, #2]
	cmp	r2, r4
	movcc	r4, r2
	add	r1, r1, #1024
	add	r1, ip, r1
	mov	r2, r4
	bl	memcpy
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L2550:
	add	r1, r1, #1
.L2549:
	cmp	r1, r4
	bcc	.L2551
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2553:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2555:
	.align	2
.L2554:
	.word	.LANCHOR2
	.fnend
	.size	rk_ftl_vendor_read, .-rk_ftl_vendor_read
	.align	2
	.global	rk_ftl_vendor_write
	.type	rk_ftl_vendor_write, %function
rk_ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2573
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r0
	ldr	r4, [r3, #980]
	.pad #20
	sub	sp, sp, #20
	mov	r9, r1
	mov	r5, r2
	cmp	r4, #0
	beq	.L2569
	add	r8, r2, #63
	ldrh	r2, [r4, #8]
	ldrh	r3, [r4, #10]
	bic	r8, r8, #63
	mov	r7, #0
	str	r2, [sp, #8]
	b	.L2558
.L2565:
	add	r2, r7, #2
	mov	r2, r2, asl #3
	add	r6, r4, r2
	ldrh	r2, [r4, r2]
	cmp	r2, sl
	bne	.L2559
	ldrh	r2, [r6, #4]
	add	r2, r2, #63
	bic	r2, r2, #63
	str	r2, [sp, #12]
	cmp	r5, r2
	bls	.L2560
	ldrh	r2, [r4, #14]
	cmp	r2, r8
	bcc	.L2569
	sub	r3, r3, #1
	mov	fp, r8
	ldrh	r6, [r6, #2]
	mov	r8, r3
	b	.L2561
.L2562:
	add	r7, r7, #1
	mov	r2, r2, asl #3
	add	r0, r7, #2
	add	r3, r4, r2
	mov	r0, r0, asl #3
	add	r1, r4, r0
	ldrh	r0, [r4, r0]
	strh	r0, [r4, r2]	@ movhi
	add	r0, r6, #1024
	ldrh	r2, [r1, #4]
	add	r0, r4, r0
	strh	r6, [r3, #2]	@ movhi
	strh	r2, [r3, #4]	@ movhi
	ldrh	r3, [r1, #4]
	ldrh	r1, [r1, #2]
	add	r3, r3, #63
	bic	r3, r3, #63
	add	r1, r1, #1024
	add	r1, r4, r1
	str	r3, [sp, #4]
	mov	r2, r3
	bl	memcpy
	ldr	r3, [sp, #4]
	add	r6, r6, r3
.L2561:
	cmp	r7, r8
	add	r2, r7, #2
	bcc	.L2562
	mov	r2, r2, asl #3
	uxth	r6, r6
	add	r3, r4, r2
	add	r0, r6, #1024
	strh	sl, [r4, r2]	@ movhi
	add	r0, r4, r0
	strh	r6, [r3, #2]	@ movhi
	mov	r2, r5
	strh	r5, [r3, #4]	@ movhi
	mov	r1, r9
	bl	memcpy
	ldrh	r3, [r4, #14]
	ldr	r2, [sp, #12]
	uxth	r8, fp
	add	r6, r6, r8
	strh	r6, [r4, #12]	@ movhi
	add	r3, r2, r3
	rsb	r8, r8, r3
	strh	r8, [r4, #14]	@ movhi
	b	.L2572
.L2560:
	ldrh	r0, [r6, #2]
	mov	r1, r9
	mov	r2, r5
	add	r0, r0, #1024
	add	r0, r4, r0
	bl	memcpy
	strh	r5, [r6, #4]	@ movhi
	b	.L2572
.L2559:
	add	r7, r7, #1
.L2558:
	cmp	r7, r3
	bcc	.L2565
	ldrh	r2, [r4, #14]
	cmp	r2, r8
	bcc	.L2569
	add	r3, r3, #2
	uxth	r8, r8
	mov	r1, r9
	mov	r3, r3, asl #3
	add	r2, r4, r3
	strh	sl, [r4, r3]	@ movhi
	ldrh	r3, [r4, #12]
	strh	r5, [r2, #4]	@ movhi
	strh	r3, [r2, #2]	@ movhi
	ldrh	r3, [r4, #12]
	add	r3, r8, r3
	strh	r3, [r4, #12]	@ movhi
	ldrh	r3, [r4, #14]
	rsb	r8, r8, r3
	strh	r8, [r4, #14]	@ movhi
	ldrh	r0, [r2, #2]
	mov	r2, r5
	add	r0, r0, #1024
	add	r0, r4, r0
	bl	memcpy
	ldrh	r3, [r4, #10]
	add	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
.L2572:
	ldr	r3, [r4, #4]
	movw	r2, #65532
	mov	r0, r4
	add	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r4, r2]
	mov	r2, #1
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #8]	@ movhi
	cmp	r3, #1
	movhi	r3, #0
	strhih	r3, [r4, #8]	@ movhi
	ldr	r3, [sp, #8]
	mov	r1, r3, asl #7
	bl	rk_ftl_vendor_ops.constprop.19
	mov	r0, #0
	b	.L2557
.L2569:
	mvn	r0, #0
.L2557:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2574:
	.align	2
.L2573:
	.word	.LANCHOR2
	.fnend
	.size	rk_ftl_vendor_write, .-rk_ftl_vendor_write
	.align	2
	.global	rk_ftl_vendor_storage_ioctl
	.type	rk_ftl_vendor_storage_ioctl, %function
rk_ftl_vendor_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2604
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r2
	mov	r6, r1
	ldr	r0, [r3, #48]
	mov	r1, #208
	mov	r2, #4096
	bl	kmem_cache_alloc_trace
	subs	r4, r0, #0
	mvneq	r5, #0
	beq	.L2576
	ldr	r3, .L2604+4
	cmp	r6, r3
	beq	.L2578
	add	r3, r3, #1
	cmp	r6, r3
	bne	.L2596
	b	.L2602
.L2578:
	mov	r3, sp
	bic	r6, r3, #8128
	bic	r6, r6, #63
	ldr	r3, [r6, #8]
@ 435 "/home/zyf/rk30/rk3288_android4.4/kernel/arch/arm/include/asm/uaccess.h" 1
	adds r2, r5, #8; sbcccs r2, r2, r3; movcc r3, #0
@ 0 "" 2
	cmp	r3, #0
	bne	.L2583
	mov	r1, r5
	mov	r2, #8
	bl	__copy_from_user
	cmp	r0, #0
	bne	.L2596
.L2598:
	ldr	r2, [r4, #0]
	ldr	r3, .L2604+8
	cmp	r2, r3
	bne	.L2595
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_read
	cmn	r0, #1
	beq	.L2591
	uxth	r2, r0
	strh	r2, [r4, #6]	@ movhi
	add	r2, r2, #8
	ldr	r3, [r6, #8]
@ 444 "/home/zyf/rk30/rk3288_android4.4/kernel/arch/arm/include/asm/uaccess.h" 1
	adds r1, r5, r2; sbcccs r1, r1, r3; movcc r3, #0
@ 0 "" 2
	cmp	r3, #0
	bne	.L2596
	mov	r0, r5
	mov	r1, r4
	bl	__copy_to_user
	subs	r5, r0, #0
	beq	.L2577
	b	.L2596
.L2602:
	mov	r3, sp
	bic	r6, r3, #8128
	bic	r6, r6, #63
	ldr	r3, [r6, #8]
@ 435 "/home/zyf/rk30/rk3288_android4.4/kernel/arch/arm/include/asm/uaccess.h" 1
	adds r2, r5, #8; sbcccs r2, r2, r3; movcc r3, #0
@ 0 "" 2
	cmp	r3, #0
	bne	.L2583
	mov	r1, r5
	mov	r2, #8
	bl	__copy_from_user
	cmp	r0, #0
	beq	.L2599
	b	.L2596
.L2583:
	mov	r1, #8
	b	.L2601
.L2599:
	ldr	r2, [r4, #0]
	ldr	r3, .L2604+8
	cmp	r2, r3
	bne	.L2595
	ldrh	r2, [r4, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L2595
	add	r2, r2, #8
	ldr	r3, [r6, #8]
@ 435 "/home/zyf/rk30/rk3288_android4.4/kernel/arch/arm/include/asm/uaccess.h" 1
	adds r1, r5, r2; sbcccs r1, r1, r3; movcc r3, #0
@ 0 "" 2
	cmp	r3, #0
	mov	r0, r4
	movne	r1, r2
	bne	.L2601
	mov	r1, r5
	bl	__copy_from_user
	cmp	r0, #0
	bne	.L2596
	b	.L2603
.L2601:
	bl	__memzero
	b	.L2596
.L2603:
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_write
.L2591:
	mov	r5, r0
	b	.L2577
.L2595:
	mvn	r5, #0
	b	.L2577
.L2596:
	mvn	r5, #13
.L2577:
	mov	r0, r4
	bl	kfree
.L2576:
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L2605:
	.align	2
.L2604:
	.word	kmalloc_caches
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
	.global	NandFlashParaTbl
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
	.global	g_gc_bad_block_temp_tbl
	.global	g_gc_bad_block_gc_index
	.global	g_gc_bad_block_temp_num
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
	.global	c_ftl_nand_blks_per_die_shift
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
	.set	.LANCHOR3,. + 0
	.type	__func__.14257, %object
	.size	__func__.14257, 11
__func__.14257:
	.ascii	"FtlMemInit\000"
.LC0:
	.byte	60
	.byte	40
	.byte	24
	.byte	16
	.type	__func__.15150, %object
	.size	__func__.15150, 21
__func__.15150:
	.ascii	"FtlVpcCheckAndModify\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC1:
	.ascii	"FlashEraseBlocks pageAddr error %x\012\000"
.LC2:
	.ascii	"No.%d FLASH ID:%x %x %x %x %x %x\012\000"
.LC3:
	.ascii	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\012"
	.ascii	"\000"
.LC4:
	.ascii	"FLASH INFO:\012\000"
.LC5:
	.ascii	"FLASH ID: %x\012\000"
.LC6:
	.ascii	"Device Capacity: %d MB\012\000"
.LC7:
	.ascii	"FMWAIT: %x %x %x %x\012\000"
.LC8:
	.ascii	"FTL INFO:\012\000"
.LC9:
	.ascii	"g_MaxLpn = 0x%x\012\000"
.LC10:
	.ascii	"g_VaildLpn = 0x%x\012\000"
.LC11:
	.ascii	"read_page_count = 0x%x\012\000"
.LC12:
	.ascii	"discard_page_count = 0x%x\012\000"
.LC13:
	.ascii	"write_page_count = 0x%x\012\000"
.LC14:
	.ascii	"cache_write_count = 0x%x\012\000"
.LC15:
	.ascii	"l2p_write_count = 0x%x\012\000"
.LC16:
	.ascii	"gc_page_count = 0x%x\012\000"
.LC17:
	.ascii	"totle_write = %d MB\012\000"
.LC18:
	.ascii	"totle_read = %d MB\012\000"
.LC19:
	.ascii	"GSV = 0x%x\012\000"
.LC20:
	.ascii	"GDV = 0x%x\012\000"
.LC21:
	.ascii	"bad blk num = %d %d\012\000"
.LC22:
	.ascii	"free_superblocks = 0x%x\012\000"
.LC23:
	.ascii	"mlc_EC = 0x%x\012\000"
.LC24:
	.ascii	"slc_EC = 0x%x\012\000"
.LC25:
	.ascii	"avg_EC = 0x%x\012\000"
.LC26:
	.ascii	"sys_EC = 0x%x\012\000"
.LC27:
	.ascii	"max_EC = 0x%x\012\000"
.LC28:
	.ascii	"min_EC = 0x%x\012\000"
.LC29:
	.ascii	"PLT = 0x%x\012\000"
.LC30:
	.ascii	"POT = 0x%x\012\000"
.LC31:
	.ascii	"MaxSector = 0x%x\012\000"
.LC32:
	.ascii	"init_sys_blks_pp = 0x%x\012\000"
.LC33:
	.ascii	"sys_blks_pp = 0x%x\012\000"
.LC34:
	.ascii	"free sysblock = 0x%x\012\000"
.LC35:
	.ascii	"data_blks_pp = 0x%x\012\000"
.LC36:
	.ascii	"data_op_blks_pp = 0x%x\012\000"
.LC37:
	.ascii	"max_data_blks = 0x%x\012\000"
.LC38:
	.ascii	"Sys.id = 0x%x\012\000"
.LC39:
	.ascii	"Bbt.id = 0x%x\012\000"
.LC40:
	.ascii	"ACT.page = 0x%x\012\000"
.LC41:
	.ascii	"ACT.plane = 0x%x\012\000"
.LC42:
	.ascii	"ACT.id = 0x%x\012\000"
.LC43:
	.ascii	"ACT.mode = 0x%x\012\000"
.LC44:
	.ascii	"ACT.a_pages = 0x%x\012\000"
.LC45:
	.ascii	"ACT VPC = 0x%x\012\000"
.LC46:
	.ascii	"BUF.page = 0x%x\012\000"
.LC47:
	.ascii	"BUF.plane = 0x%x\012\000"
.LC48:
	.ascii	"BUF.id = 0x%x\012\000"
.LC49:
	.ascii	"BUF.mode = 0x%x\012\000"
.LC50:
	.ascii	"BUF.a_pages = 0x%x\012\000"
.LC51:
	.ascii	"BUF VPC = 0x%x\012\000"
.LC52:
	.ascii	"TMP.page = 0x%x\012\000"
.LC53:
	.ascii	"TMP.plane = 0x%x\012\000"
.LC54:
	.ascii	"TMP.id = 0x%x\012\000"
.LC55:
	.ascii	"TMP.mode = 0x%x\012\000"
.LC56:
	.ascii	"TMP.a_pages = 0x%x\012\000"
.LC57:
	.ascii	"GC.page = 0x%x\012\000"
.LC58:
	.ascii	"GC.plane = 0x%x\012\000"
.LC59:
	.ascii	"GC.id = 0x%x\012\000"
.LC60:
	.ascii	"GC.mode = 0x%x\012\000"
.LC61:
	.ascii	"GC.a_pages = 0x%x\012\000"
.LC62:
	.ascii	"WR_CHK = 0x%x %x %x %x\012\000"
.LC63:
	.ascii	"Read Err Cnt = 0x%x\012\000"
.LC64:
	.ascii	"Prog Err Cnt = 0x%x\012\000"
.LC65:
	.ascii	"gc_free_blk_th= 0x%x\012\000"
.LC66:
	.ascii	"gc_merge_free_blk_th= 0x%x\012\000"
.LC67:
	.ascii	"gc_skip_write_count= 0x%x\012\000"
.LC68:
	.ascii	"gc_blk_index= 0x%x\012\000"
.LC69:
	.ascii	"free min EC= 0x%x\012\000"
.LC70:
	.ascii	"free max EC= 0x%x\012\000"
.LC71:
	.ascii	"GC__SB VPC = 0x%x\012\000"
.LC72:
	.ascii	"%d. [0x%x]=0x%x 0x%x  0x%x\012\000"
.LC73:
	.ascii	"free %d. [0x%x] 0x%x  0x%x\012\000"
.LC74:
	.ascii	"%s\012\000"
.LC75:
	.ascii	"FTL version: 5.0.40 20170816\000"
.LC76:
	.ascii	"GetSwlReplaceBlock min_ec_id =%x %x\012\000"
.LC77:
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC78:
	.ascii	"FtlGcRefreshBlock  0x%x\012\000"
.LC79:
	.ascii	"FtlGcMarkBadPhyBlk %d 0x%x\012\000"
.LC80:
	.ascii	"%s error allocating memory. return -1\012\000"
.LC81:
	.ascii	"%s 0x%x:\000"
.LC82:
	.ascii	"%x \000"
.LC83:
	.ascii	"\000"
.LC84:
	.ascii	"%d statReg->V6.mtrans_cnt=%d flReg.V6.page_num=%d\012"
	.ascii	"\000"
.LC85:
	.ascii	"nandc:\000"
.LC86:
	.ascii	"%d flReg.d32=%x %x\012\000"
.LC87:
	.ascii	"micron RR %d row=%x,count %d,status=%d\012\000"
.LC88:
	.ascii	"ECC:%d\012\000"
.LC89:
	.ascii	"sdr read ok %x ecc=%d\012\000"
.LC90:
	.ascii	"sync para %d\012\000"
.LC91:
	.ascii	"TOG mode Read error %x %x\012\000"
.LC92:
	.ascii	"FlashLoadPhyInfo fail %x!!\012\000"
.LC93:
	.ascii	"read retry status %x %x %x\012\000"
.LC94:
	.ascii	"Read pageadd=%x  ecc=%x err=%x\012\000"
.LC95:
	.ascii	"data:\000"
.LC96:
	.ascii	"spare:\000"
.LC97:
	.ascii	"ReadRetry pageadd=%x ecc=%x err=%x\012\000"
.LC98:
	.ascii	"slc mode\000"
.LC99:
	.ascii	"no ect\000"
.LC100:
	.ascii	"prog error: = %x\012\000"
.LC101:
	.ascii	"prog read error: = %x\012\000"
.LC102:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC103:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC104:
	.ascii	"FLFB:%d %d\012\000"
.LC105:
	.ascii	"BBT:\000"
.LC106:
	.ascii	"s raw s:\000"
.LC107:
	.ascii	"s read s:\000"
.LC108:
	.ascii	"d raw d:\000"
.LC109:
	.ascii	"d read d:\000"
.LC110:
	.ascii	"FtlVpcTblFlush error = %x error count = %d\012\000"
.LC111:
	.ascii	"FtlMapWritePage error = %x\012\000"
.LC112:
	.ascii	"FtlMapWritePage error = %x error count = %d\012\000"
.LC113:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC114:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC115:
	.ascii	"FtlBbmTblFlush error = %x error count = %d\012\000"
.LC116:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
.LC117:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC118:
	.ascii	"spuer block %x vpn is 0\012 \000"
.LC119:
	.ascii	"...%s enter...\012\000"
.LC120:
	.ascii	"FtlCheckVpc %x = %x  %x\012\000"
.LC121:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC122:
	.ascii	"GC des block %x done\012\000"
.LC123:
	.ascii	"%d GC datablk  = %x vpc %x %x\012\000"
.LC124:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC125:
	.ascii	"FtlInit %x\012\000"
.LC126:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
.LC127:
	.ascii	"FlashMakeFactorBbt %d\012\000"
.LC128:
	.ascii	"bad block:%d %d\012\000"
.LC129:
	.ascii	"FMFB:%d %d\012\000"
.LC130:
	.ascii	"E:bad block:%d\012\000"
.LC131:
	.ascii	"FMFB:Save %d %d\012\000"
.LC132:
	.ascii	"IdBlockReadData %x %x\012\000"
.LC133:
	.ascii	"IdBlockReadData %x %x ret= %x\012\000"
.LC134:
	.ascii	"IDBlockWriteData %x %x\012\000"
.LC135:
	.ascii	"IDBlockWriteData %x %x ret= %x\012\000"
.LC136:
	.ascii	"idblk:\000"
.LC137:
	.ascii	"idb reverse %x %x\012\000"
.LC138:
	.ascii	"write_idblock totle_sec %x %x\012\000"
.LC139:
	.ascii	"write and check error:%d idb=%x,offset=%x,r=%x,w=%x"
	.ascii	"\012\000"
.LC140:
	.ascii	"write\000"
.LC141:
	.ascii	"read\000"
.LC142:
	.ascii	"write_idblock error\012\000"
.LC143:
	.ascii	"READ_SECTOR_IO\012\000"
.LC144:
	.ascii	"rk_copy_from_user error\012\000"
.LC145:
	.ascii	"READ_SECTOR_IO %x %x\012\000"
.LC146:
	.ascii	"rk_copy_to_user error\012\000"
.LC147:
	.ascii	"WRITE_SECTOR_IO\012\000"
.LC148:
	.ascii	"WRITE_SECTOR_IO %x %x\012\000"
.LC149:
	.ascii	"END_WRITE_SECTOR_IO\012\000"
.LC150:
	.ascii	"END_WRITE_SECTOR_IO %x %x\012\000"
.LC151:
	.ascii	"GET_FLASH_INFO_IO\012\000"
.LC152:
	.ascii	"GET_BAD_BLOCK_IO\012\000"
.LC153:
	.ascii	"bbt:\000"
.LC154:
	.ascii	"GET_LOCK_FLAG_IO\012\000"
.LC155:
	.ascii	"GET_PUBLIC_KEY_IO\012\000"
.LC156:
	.ascii	"RKNAND_GET_DRM_KEY\012\000"
.LC157:
	.ascii	"RKNAND_STORE_DRM_KEY\012\000"
.LC158:
	.ascii	"RKNAND_DIASBLE_SECURE_BOOT\012\000"
.LC159:
	.ascii	"RKNAND_ENASBLE_SECURE_BOOT\012\000"
.LC160:
	.ascii	"RKNAND_GET_SN_SECTOR\012\000"
.LC161:
	.ascii	"RKNAND_LOADER_UNLOCK\012\000"
.LC162:
	.ascii	"RKNAND_LOADER_STATUS\012\000"
.LC163:
	.ascii	"RKNAND_LOADER_LOCK\012\000"
.LC164:
	.ascii	"LockKey not match %d\012\000"
.LC165:
	.ascii	"RKNAND_GET_VENDOR_SECTOR\012\000"
.LC166:
	.ascii	"RKNAND_STORE_VENDOR_SECTOR\012\000"
.LC167:
	.ascii	"return ret = %lx\012\000"
.LC168:
	.ascii	"secureBootEn check error\012\000"
.LC169:
	.ascii	"\0013vendor storage %x,%x,%x\012\000"
	.data
	.align	2
	.set	.LANCHOR1,. + 0
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
	.type	NandFlashParaTbl, %object
	.size	NandFlashParaTbl, 2304
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
	.byte	6
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
	.byte	-75
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
	.short	1503
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
	.short	1497
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
	.short	1497
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
	.type	gFtlInitStatus, %object
	.size	gFtlInitStatus, 4
gFtlInitStatus:
	.word	-1
	.type	micron_flash_randomizer_flag, %object
	.size	micron_flash_randomizer_flag, 4
micron_flash_randomizer_flag:
	.word	1
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
	.set	.LANCHOR0,. + 0
	.set	.LANCHOR2,. + 8184
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
	.type	gFlashSlcMode, %object
	.size	gFlashSlcMode, 1
gFlashSlcMode:
	.space	1
	.space	3
	.type	gNandIDataBuf, %object
	.size	gNandIDataBuf, 2048
gNandIDataBuf:
	.space	2048
	.type	IDByte, %object
	.size	IDByte, 32
IDByte:
	.space	32
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
	.type	slcPageToMlcPageTbl, %object
	.size	slcPageToMlcPageTbl, 512
slcPageToMlcPageTbl:
	.space	512
	.type	DieAddrs, %object
	.size	DieAddrs, 32
DieAddrs:
	.space	32
	.type	FlashWaitBusyScheduleEn, %object
	.size	FlashWaitBusyScheduleEn, 4
FlashWaitBusyScheduleEn:
	.space	4
	.type	gpNandParaInfo, %object
	.size	gpNandParaInfo, 4
gpNandParaInfo:
	.space	4
	.type	gDieOp, %object
	.size	gDieOp, 128
gDieOp:
	.space	128
	.type	gReadRetryInfo, %object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.space	852
	.type	read_retry_cur_offset, %object
	.size	read_retry_cur_offset, 4
read_retry_cur_offset:
	.space	4
	.type	g_maxRegNum, %object
	.size	g_maxRegNum, 1
g_maxRegNum:
	.space	1
	.type	g_retryMode, %object
	.size	g_retryMode, 1
g_retryMode:
	.space	1
	.type	gNandMaxDie, %object
	.size	gNandMaxDie, 1
gNandMaxDie:
	.space	1
	.space	1
	.type	DieCsIndex, %object
	.size	DieCsIndex, 8
DieCsIndex:
	.space	8
	.type	gFlashInterfaceMode, %object
	.size	gFlashInterfaceMode, 1
gFlashInterfaceMode:
	.space	1
	.type	gFlashToggleModeEn, %object
	.size	gFlashToggleModeEn, 1
gFlashToggleModeEn:
	.space	1
	.space	2
	.type	gFlashPageBuffer0, %object
	.size	gFlashPageBuffer0, 4
gFlashPageBuffer0:
	.space	4
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
	.type	gNandFlashEccBits, %object
	.size	gNandFlashEccBits, 1
gNandFlashEccBits:
	.space	1
	.type	gMultiPageProgEn, %object
	.size	gMultiPageProgEn, 1
gMultiPageProgEn:
	.space	1
	.space	2
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
	.type	c_ftl_nand_blks_per_die_shift, %object
	.size	c_ftl_nand_blks_per_die_shift, 2
c_ftl_nand_blks_per_die_shift:
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
	.type	g_MaxLbaSector, %object
	.size	g_MaxLbaSector, 4
g_MaxLbaSector:
	.space	4
	.type	g_totle_vendor_block, %object
	.size	g_totle_vendor_block, 2
g_totle_vendor_block:
	.space	2
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
	.type	g_cur_erase_blk, %object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.space	4
	.type	gBbtInfo, %object
	.size	gBbtInfo, 60
gBbtInfo:
	.space	60
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.space	2056
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
	.type	p_sys_data_buf, %object
	.size	p_sys_data_buf, 4
p_sys_data_buf:
	.space	4
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
	.type	g_inkDie_check_enable, %object
	.size	g_inkDie_check_enable, 4
g_inkDie_check_enable:
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
	.type	g_sys_save_data, %object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.space	48
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.space	2
	.space	2
	.type	gSysInfo, %object
	.size	gSysInfo, 12
gSysInfo:
	.space	12
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
	.type	g_gc_blk_num, %object
	.size	g_gc_blk_num, 2
g_gc_blk_num:
	.space	2
	.space	2
	.type	p_gc_blk_tbl, %object
	.size	p_gc_blk_tbl, 4
p_gc_blk_tbl:
	.space	4
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
	.type	p_map_block_valid_page_count, %object
	.size	p_map_block_valid_page_count, 4
p_map_block_valid_page_count:
	.space	4
	.type	p_l2p_map_buf, %object
	.size	p_l2p_map_buf, 4
p_l2p_map_buf:
	.space	4
	.type	gL2pMapInfo, %object
	.size	gL2pMapInfo, 40
gL2pMapInfo:
	.space	40
	.type	g_totle_map_block, %object
	.size	g_totle_map_block, 2
g_totle_map_block:
	.space	2
	.space	2
	.type	p_map_block_table, %object
	.size	p_map_block_table, 4
p_map_block_table:
	.space	4
	.type	p_map_block_ver_table, %object
	.size	p_map_block_ver_table, 4
p_map_block_ver_table:
	.space	4
	.type	p_map_region_ppn_table, %object
	.size	p_map_region_ppn_table, 4
p_map_region_ppn_table:
	.space	4
	.type	c_ftl_nand_bbm_buf_size, %object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
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
	.type	p_swl_mul_table, %object
	.size	p_swl_mul_table, 4
p_swl_mul_table:
	.space	4
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
	.type	gp_ect_tbl_info, %object
	.size	gp_ect_tbl_info, 4
gp_ect_tbl_info:
	.space	4
	.type	p_valid_page_count_check_table, %object
	.size	p_valid_page_count_check_table, 4
p_valid_page_count_check_table:
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
	.type	gNandMaxChip, %object
	.size	gNandMaxChip, 1
gNandMaxChip:
	.space	1
	.space	1
	.type	gTotleBlock, %object
	.size	gTotleBlock, 2
gTotleBlock:
	.space	2
	.type	mlcPageToSlcPageTbl, %object
	.size	mlcPageToSlcPageTbl, 1024
mlcPageToSlcPageTbl:
	.space	1024
	.type	RK29_NANDC_REG_BASE, %object
	.size	RK29_NANDC_REG_BASE, 4
RK29_NANDC_REG_BASE:
	.space	4
	.type	gFlashPageBuffer1, %object
	.size	gFlashPageBuffer1, 4
gFlashPageBuffer1:
	.space	4
	.type	g_maxRetryCount, %object
	.size	g_maxRetryCount, 1
g_maxRetryCount:
	.space	1
	.space	3
	.type	gpFlashSaveInfo, %object
	.size	gpFlashSaveInfo, 4
gpFlashSaveInfo:
	.space	4
	.type	gNandFlashIDBEccBits, %object
	.size	gNandFlashIDBEccBits, 1
gNandFlashIDBEccBits:
	.space	1
	.space	3
	.type	gNandFlashIdbBlockAddr, %object
	.size	gNandFlashIdbBlockAddr, 4
gNandFlashIdbBlockAddr:
	.space	4
	.type	gNandFlashInfoBlockAddr, %object
	.size	gNandFlashInfoBlockAddr, 4
gNandFlashInfoBlockAddr:
	.space	4
	.type	gNandIDBResBlkNumSaveInFlash, %object
	.size	gNandIDBResBlkNumSaveInFlash, 1
gNandIDBResBlkNumSaveInFlash:
	.space	1
	.space	3
	.type	FlashDdrTunningReadCount, %object
	.size	FlashDdrTunningReadCount, 4
FlashDdrTunningReadCount:
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
	.type	gMultiPageReadEn, %object
	.size	gMultiPageReadEn, 1
gMultiPageReadEn:
	.space	1
	.space	3
	.type	gpReadRetrial, %object
	.size	gpReadRetrial, 4
gpReadRetrial:
	.space	4
	.type	req_sys, %object
	.size	req_sys, 36
req_sys:
	.space	36
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.space	4
	.type	FbbtBlk, %object
	.size	FbbtBlk, 16
FbbtBlk:
	.space	16
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 40
gVendorBlkInfo:
	.space	40
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
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
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 4
gp_last_act_superblock:
	.space	4
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
	.type	gc_discard_updated, %object
	.size	gc_discard_updated, 4
gc_discard_updated:
	.space	4
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.space	4
	.type	last_cache_match_count, %object
	.size	last_cache_match_count, 4
last_cache_match_count:
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
