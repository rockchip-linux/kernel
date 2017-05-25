/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
	.type	rknand_sys_storage_release, %function
rknand_sys_storage_release:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r0, [r1, #124]
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	cmp	r0, #0
	beq	.L2
	mov	r3, #0
	str	r3, [r1, #124]
	bl	kfree
.L2:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
	.fnend
	.size	rknand_sys_storage_release, .-rknand_sys_storage_release
	.align	2
	.type	rknand_sys_storage_open, %function
rknand_sys_storage_open:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L6
	mov	r2, #4096
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r1
	ldr	r0, [r3, #48]
	mov	r1, #208
	bl	kmem_cache_alloc_trace
	mov	r3, #0
	str	r3, [r4, #124]
	cmp	r0, #0
	bne	.L4
	ldr	r0, .L6+4
	bl	printk
	mvn	r0, #11
	ldmfd	sp!, {r4, pc}
.L4:
	str	r0, [r4, #124]
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L7:
	.align	2
.L6:
	.word	kmalloc_caches
	.word	.LC1
	.fnend
	.size	rknand_sys_storage_open, .-rknand_sys_storage_open
	.align	2
	.global	FlashMemCmp8
	.type	FlashMemCmp8, %function
FlashMemCmp8:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L18
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L16
	ldrb	r3, [r1, #1]	@ zero_extendqisi2
	ldrb	ip, [r0, #1]	@ zero_extendqisi2
	cmp	ip, r3
	movne	r3, #0
	bne	.L16
	b	.L17
.L13:
	ldrb	r4, [r0, r3]	@ zero_extendqisi2
	ldrb	ip, [r1, r3]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r4, ip
	beq	.L16
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L16:
	cmp	r3, r2
	bne	.L13
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L17:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L19:
	.align	2
.L18:
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
	ldr	r3, .L21
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
.L22:
	.align	2
.L21:
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
	beq	.L24
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r4, r4, #-1073741824
.L24:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r0
	ldr	r6, .L33
	mov	r5, r1
	ldr	r3, [r6, #12]
	mov	r8, r6
	cmp	r3, #5
	bls	.L28
	and	r5, r1, #127
	ldr	r3, .L33+4
	mov	r5, r5, asl #1
	ldrh	r4, [r3, r5]
	ldrb	r3, [r6, #8]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L29
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r4, r4, #-1073741824
.L29:
	add	r7, r8, r7, asl #3
	b	.L32
.L28:
	cmp	r3, #4
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	and	r3, r1, #127
	ldr	r2, .L33+4
	mov	r3, r3, asl #1
	ldrh	r4, [r2, r3]
	bl	FlashRsvdBlkChk
	mov	r4, r4, asl #8
	cmp	r0, #0
	beq	.L31
	ldrb	r3, [r6, #8]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r5, r5, asl #1
	uxtbne	r5, r5
	orrne	r5, r5, #1
	orrne	r4, r4, r5
.L31:
	ldr	r3, .L33
	add	r7, r3, r7, asl #3
.L32:
	ldr	r3, [r7, #16]
	str	r4, [r3, #336]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L34:
	.align	2
.L33:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.fnend
	.size	FlashSetRandomizer, .-FlashSetRandomizer
	.align	2
	.global	FlashReadCmd
	.type	FlashReadCmd, %function
FlashReadCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L36
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, ip, r0, asl #3
	ldrb	r3, [ip, #20]	@ zero_extendqisi2
	ldr	r4, [ip, #16]
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
.L37:
	.align	2
.L36:
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
	ldr	r2, .L42
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, r2, r0, asl #3
	ldrb	r2, [r2, #96]	@ zero_extendqisi2
	ldrb	r3, [ip, #20]	@ zero_extendqisi2
	ldr	r4, [ip, #16]
	cmp	r2, #1
	add	r3, r3, #8
	mov	r2, #0
	add	r3, r4, r3, asl #8
	bne	.L39
	mov	ip, #6
	str	ip, [r3, #8]
	str	r2, [r3, #4]
	str	r2, [r3, #4]
	uxtb	r2, r1
	str	r2, [r3, #4]
	mov	r2, r1, lsr #8
	str	r2, [r3, #4]
	mov	r2, r1, lsr #16
	b	.L41
.L39:
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
.L41:
	str	r2, [r3, #4]
	mov	r2, #224
	str	r2, [r3, #8]
	ldmfd	sp!, {r4, lr}
	b	FlashSetRandomizer
.L43:
	.align	2
.L42:
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
	ldr	ip, .L45
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, ip, r0, asl #3
	ldrb	r3, [ip, #20]	@ zero_extendqisi2
	ldr	r4, [ip, #16]
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
.L46:
	.align	2
.L45:
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
	ldr	r3, .L49
	cmp	r2, #0
	add	r0, r3, r0, asl #3
	ldr	ip, [r0, #16]
	ldrb	r0, [r0, #20]	@ zero_extendqisi2
	add	r0, r0, #8
	add	r0, ip, r0, asl #8
	beq	.L48
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
.L48:
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
.L50:
	.align	2
.L49:
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
	ldr	ip, .L52
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	add	r4, ip, r0, asl #3
	ldrb	ip, [ip, #91]	@ zero_extendqisi2
	ldrb	r3, [r4, #20]	@ zero_extendqisi2
	ldr	r5, [r4, #16]
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
.L53:
	.align	2
.L52:
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
	ldr	r3, .L55
	add	r3, r3, r0, asl #3
	ldr	r2, [r3, #16]
	ldrb	r3, [r3, #20]	@ zero_extendqisi2
	add	r3, r3, #8
	add	r3, r2, r3, asl #8
	mov	r2, #16
	str	r2, [r3, #8]
	bx	lr
.L56:
	.align	2
.L55:
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
	ldr	r3, .L58
	add	r0, r3, r0, asl #3
	ldrb	r3, [r3, #90]	@ zero_extendqisi2
	ldrb	r2, [r0, #20]	@ zero_extendqisi2
	ldr	r1, [r0, #16]
	add	r2, r2, #8
	add	r2, r1, r2, asl #8
	str	r3, [r2, #8]
	bx	lr
.L59:
	.align	2
.L58:
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
	ldr	r0, .L63
	b	.L61
.L62:
	mov	ip, r0, asl #5
	ldrb	r4, [r3, r2]	@ zero_extendqisi2
	add	ip, ip, r0, lsr #2
	add	r2, r2, #1
	add	ip, ip, r4
	eor	r0, r0, ip
.L61:
	cmp	r2, r1
	bne	.L62
	ldmfd	sp!, {r4, pc}
.L64:
	.align	2
.L63:
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
	ldr	r1, .L68
	bl	memcpy
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L69:
	.align	2
.L68:
	.word	.LANCHOR0+112
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
	ldr	r6, .L82
.L74:
	add	r5, r6, r4, asl #5
	ldr	r1, .L82+4
	add	r0, r5, #1
	ldrb	r2, [r6, r4, asl #5]	@ zero_extendqisi2
	bl	FlashMemCmp8
	subs	r1, r0, #0
	bne	.L71
	cmp	r5, #0
	ldrneb	r2, [r5, #22]	@ zero_extendqisi2
	ldrne	r3, .L82+8
	bne	.L76
	b	.L81
.L71:
	add	r4, r4, #1
	cmp	r4, #68
	bne	.L74
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L76:
	ldrb	r0, [r3, r1, asl #5]	@ zero_extendqisi2
	cmp	r0, r2
	beq	.L75
	add	r1, r1, #1
	cmp	r1, #4
	bne	.L76
.L75:
	ldr	r3, .L82+8
	mov	r2, #32
	ldr	r0, .L82+12
	add	r1, r3, r1, asl #5
	bl	memcpy
	ldr	r0, .L82+16
	mov	r1, r5
	mov	r2, #32
	bl	memcpy
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L81:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L83:
	.align	2
.L82:
	.word	.LANCHOR1+256
	.word	.LANCHOR0+2160
	.word	.LANCHOR1+2432
	.word	.LANCHOR0+80
	.word	.LANCHOR1+2560
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
	ldr	r3, .L85
	mov	r0, #0
	ldr	r2, [r3, #2224]
	ldr	r1, [r2, #0]
	str	r1, [r3, #2228]
	ldr	r1, [r2, #4]
	str	r1, [r3, #2232]
	ldr	r1, [r2, #8]
	str	r1, [r3, #2236]
	ldr	r1, [r2, #12]
	str	r1, [r3, #2240]
	ldr	r1, [r2, #304]
	str	r1, [r3, #2244]
	ldr	r1, [r2, #308]
	str	r1, [r3, #2248]
	ldr	r1, [r2, #336]
	ldr	r2, [r2, #344]
	str	r1, [r3, #2252]
	str	r2, [r3, #2256]
	bx	lr
.L86:
	.align	2
.L85:
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
	ldr	r3, .L92
	mov	r5, r1
	mov	r6, r2
	movw	r1, #2274
	mov	r2, #2272
	ldr	r8, [r0, #4]
	ldrh	r2, [r3, r2]
	mov	r4, r0
	ldrh	sl, [r3, r1]
	ubfx	r9, r8, #10, #16
	ldrh	r7, [r3, #4]
	mov	r8, r8, asl #22
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	mov	r0, r9
	mul	sl, sl, r2
	cmp	r3, #1
	moveq	r7, r7, asl #1
	mov	r8, r8, lsr #22
	uxtheq	r7, r7
	uxth	sl, sl
	mov	r1, sl
	bl	__aeabi_uidiv
	ldr	r3, .L92
	cmp	r5, #1
	addeq	r8, r3, r8, asl #1
	addeq	r8, r8, #2288
	ldreqh	r8, [r8, #0]
	uxth	r0, r0
	mls	sl, sl, r0, r9
	add	r3, r3, r0, asl #2
	ldr	r3, [r3, #2800]
	uxth	sl, sl
	mla	r7, r7, sl, r3
	ldrb	r3, [sp, #40]	@ zero_extendqisi2
	cmp	r3, #1
	add	r8, r7, r8
	str	r8, [r6, #0]
	str	r0, [fp, #0]
	bls	.L91
	ldr	r0, [r4, #4]
	ldr	r3, [r4, #40]
	add	r0, r0, #1024
	rsb	r3, r3, r0
	rsbs	r0, r3, #0
	adc	r0, r0, r3
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L91:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L93:
	.align	2
.L92:
	.word	.LANCHOR0
	.fnend
	.size	LogAddr2PhyAddr, .-LogAddr2PhyAddr
	.align	2
	.global	FlashGetPageSize
	.type	FlashGetPageSize, %function
FlashGetPageSize:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L95
	ldr	r3, [r3, #2864]
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
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	ldr	r2, [r3, #16]
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
	ldr	r2, .L103
	mov	r1, r0, asl #8
	orr	r1, r1, r0, asl #16
	orr	r1, r1, #1
	ldr	r3, [r2, #2224]
	ldr	r2, [r2, #2868]
	cmp	r2, #0
	str	r1, [r3, #304]
	ldrne	r3, [r3, #304]
	strne	r3, [r2, #304]
	bx	lr
.L104:
	.align	2
.L103:
	.word	.LANCHOR0
	.fnend
	.size	NandcSetDdrPara, .-NandcSetDdrPara
	.align	2
	.global	NandcSetDdrMode
	.type	NandcSetDdrMode, %function
NandcSetDdrMode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r1, .L108
	cmp	r0, #0
	ldr	r2, [r1, #2224]
	ldr	r3, [r2, #0]
	bfc	r3, #13, #1
	orrne	r3, r3, #253952
	str	r3, [r2, #0]
	ldr	r3, [r1, #2868]
	cmp	r3, #0
	ldrne	r2, .L108
	ldrne	r2, [r2, #2224]
	ldrne	r2, [r2, #0]
	strne	r2, [r3, #0]
	bx	lr
.L109:
	.align	2
.L108:
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
	ldr	r3, .L117
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r3, #2224]
	ldr	r4, [r3, #0]
	ands	r3, r0, #6
	bfieq	r4, r3, #13, #1
	beq	.L115
	orr	r4, r4, #24576
	tst	r0, #4
	bfc	r4, #15, #1
	mov	r0, #0
	orr	r4, r4, #196608
	orrne	r4, r4, #32768
	bl	rknand_get_clk_rate
	ldr	r3, .L117+4
	cmp	r0, r3
	ldr	r3, .L117
	mov	r0, #38
	ldr	r1, [r3, #2872]
	ldr	r2, [r3, #2224]
	addgt	r1, r1, #1
	orr	r1, r1, #8320
	str	r1, [r2, #344]
	ldr	r2, [r3, #2224]
	ldr	r3, [r3, #2868]
	ldr	r1, .L117+8
	cmp	r3, #0
	str	r1, [r2, #304]
	mov	r1, #39
	str	r0, [r2, #308]
	str	r1, [r2, #308]
	beq	.L115
	ldr	ip, [r2, #344]
	str	ip, [r3, #344]
	ldr	r2, [r2, #304]
	str	r2, [r3, #304]
	str	r0, [r3, #308]
	str	r1, [r3, #308]
.L115:
	ldr	r3, .L117
	mov	r0, #0
	ldr	r2, [r3, #2224]
	str	r4, [r2, #0]
	ldr	r2, [r3, #2868]
	cmp	r2, #0
	ldrne	r3, [r3, #2224]
	ldrne	r3, [r3, #0]
	strne	r3, [r2, #0]
	ldmfd	sp!, {r4, pc}
.L118:
	.align	2
.L117:
	.word	.LANCHOR0
	.word	150999999
	.word	1052675
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
	ldr	r3, .L120
	ldr	r3, [r3, #16]
	ldr	r2, [r3, #0]
	bfi	r2, r0, #0, #8
	str	r2, [r3, #0]
	bx	lr
.L121:
	.align	2
.L120:
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
	ldr	r3, .L123
	add	r3, r3, r0, asl #3
	mov	r0, #1
	ldr	r2, [r3, #16]
	ldrb	r1, [r3, #20]	@ zero_extendqisi2
	ldr	r3, [r2, #0]
	mov	r1, r0, asl r1
	bfi	r3, r1, #0, #8
	str	r3, [r2, #0]
	bx	lr
.L124:
	.align	2
.L123:
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
	ldr	r3, .L126
	add	r3, r3, r0, asl #3
	ldr	r2, [r3, #16]
	ldr	r3, [r2, #0]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2, #0]
	bx	lr
.L127:
	.align	2
.L126:
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
.L129:
	ldr	r0, [sp, #4]
	cmp	r0, #0
	sub	r3, r0, #1
	str	r3, [sp, #4]
	bne	.L129
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
	ldr	r3, .L140
	add	r0, r3, r0, asl #3
	ldr	ip, [r3, #2864]
	ldrb	r5, [r0, #20]	@ zero_extendqisi2
	ldr	r4, [r0, #16]
	ldrb	ip, [ip, #8]	@ zero_extendqisi2
	add	r5, r5, #8
	cmp	ip, #2
	add	r0, r4, r5, asl #8
	movne	r3, #112
	strne	r3, [r0, #8]
	bne	.L136
	cmp	r2, #0
	ldreqb	r2, [r3, #93]	@ zero_extendqisi2
	ldrneb	r2, [r3, #94]	@ zero_extendqisi2
	str	r2, [r0, #8]
	ldrb	r3, [r3, #95]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #0
	ldrne	r2, .L140
	bne	.L135
	b	.L136
.L137:
	mov	ip, r3, asl #3
	add	r3, r3, #1
	mov	ip, r1, lsr ip
	uxtb	ip, ip
	str	ip, [r0, #4]
.L135:
	ldrb	ip, [r2, #95]	@ zero_extendqisi2
	cmp	r3, ip
	bcc	.L137
.L136:
	mov	r0, #80
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	uxtb	r0, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L141:
	.align	2
.L140:
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
.L146:
	mov	r0, r6
	mov	r1, r5
	mov	r2, r4
	bl	FlashReadStatusEN
	cmp	r0, #255
	beq	.L146
	tst	r0, #64
	beq	.L146
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
	ldr	r3, .L150
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r4, r3, r0, asl #4
	ldrb	r5, [r3, r0, asl #4]	@ zero_extendqisi2
	mov	r6, r0
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L148
	mov	r0, r5
	bl	NandcFlashCs
	ldr	r3, .L150+4
	ldr	r1, [r4, #4]
	mov	r0, r5
	add	r6, r3, r6, asl #2
	ldr	r2, [r6, #2800]
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
.L148:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L151:
	.align	2
.L150:
	.word	.LANCHOR0+2876
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
	ldr	r3, .L157
	mov	r6, r2
	mov	r4, r0
	mov	r5, r1
	mov	r7, #0
	ldr	r2, [r3, #2864]
	ldrb	r2, [r2, #19]	@ zero_extendqisi2
	cmp	r2, #6
	addeq	r9, r3, #3152
	addne	r2, r3, #3152
	add	r3, r3, r0, asl #3
	addne	r9, sl, r0, asl #3
	addeq	r9, r9, r0, asl #6
	ldrb	r8, [r3, #20]	@ zero_extendqisi2
	addne	r9, r2, r9, asl #3
	ldr	r2, [r3, #16]
	addeq	r9, r9, sl, asl #2
	add	r8, r8, #8
	add	r8, r2, r8, asl #8
	bl	NandcFlashCs
	mov	r3, #54
	str	r3, [r8, #8]
	b	.L155
.L156:
	ldrb	r3, [r6, r7]	@ zero_extendqisi2
	mov	r0, #200
	str	r3, [r8, #4]
	bl	NandcDelayns
	ldrb	r3, [r9, r7]	@ zero_extendqisi2
	add	r7, r7, #1
	str	r3, [r8, #0]
.L155:
	uxtb	r3, r7
	cmp	r3, r5
	bcc	.L156
	mov	r3, #22
	mov	r0, r4
	str	r3, [r8, #8]
	bl	NandcFlashDeCs
	ldr	r3, .L157
	add	r4, r3, r4
	strb	sl, [r4, #3984]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L158:
	.align	2
.L157:
	.word	.LANCHOR0
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
	ldr	r5, .L164
	ldr	r3, [r5, #2864]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #5
	ldmhifd	sp!, {r3, r4, r5, pc}
	mov	r4, #0
.L162:
	ldr	r2, .L164+4
	uxtb	r0, r4
	ldrb	r3, [r2, r4, asl #3]	@ zero_extendqisi2
	cmp	r3, #173
	bne	.L161
	ldrb	r1, [r5, #3133]	@ zero_extendqisi2
	add	r2, r2, #976
	mov	r3, #0
	bl	HynixSetRRPara
.L161:
	add	r4, r4, #1
	cmp	r4, #8
	bne	.L162
	ldmfd	sp!, {r3, r4, r5, pc}
.L165:
	.align	2
.L164:
	.word	.LANCHOR0
	.word	.LANCHOR0+2160
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
	ldr	r3, .L169
	mov	r5, #0
	ldr	sl, .L169+4
	mov	r8, #161
	add	r1, r3, r1, asl #2
	add	r6, r1, #3
	mov	r7, r3
	b	.L167
.L168:
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
.L167:
	ldrb	r3, [sl, #3992]	@ zero_extendqisi2
	cmp	r5, r3
	bcc	.L168
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L170:
	.align	2
.L169:
	.word	.LANCHOR1+2592
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
	ldr	r3, .L178
	mov	r4, r0
	add	r6, r6, r6, asl #2
	mov	r5, #0
	add	r2, r3, #45
	sub	sl, r3, #2656
	add	r7, r3, r6
	ldr	r8, .L178+4
	add	r6, r2, r6
	add	sl, sl, r1
	b	.L172
.L176:
	mov	r3, #85
	str	r3, [r4, #8]
	ldr	r3, .L178
	mov	r0, #200
	ldrsb	r3, [r5, r3]
	str	r3, [r4, #4]
	bl	NandcDelayns
	ldrb	r3, [r8, #3993]	@ zero_extendqisi2
	cmp	r3, #34
	ldreqsb	r3, [r7, #0]
	beq	.L177
	cmp	r3, #35
	addne	r3, sl, #2784
	addne	r3, r3, #12
	ldreqsb	r3, [r6, #0]
	ldrnesb	r3, [r3, #0]
.L177:
	add	r5, r5, #1
	add	r7, r7, #1
	add	r6, r6, #1
	str	r3, [r4, #0]
.L172:
	ldrb	r3, [r8, #3992]	@ zero_extendqisi2
	cmp	r5, r3
	bcc	.L176
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L179:
	.align	2
.L178:
	.word	.LANCHOR1+2656
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
	ldr	r3, .L181
	add	r3, r3, r0, asl #3
	mov	r0, #80
	ldrb	r5, [r3, #20]	@ zero_extendqisi2
	ldr	r4, [r3, #16]
	add	r5, r5, #8
	add	r3, r4, r5, asl #8
	str	r2, [r3, #8]
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	ldmfd	sp!, {r3, r4, r5, pc}
.L182:
	.align	2
.L181:
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
	ldr	r3, .L188
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r0, r1, r2, r4, r5, lr}
	add	r3, r3, r0, asl #3
	ldr	r4, .L188+4
	ldr	r5, [r3, #16]
.L185:
	mov	r0, #100
	bl	NandcDelayns
	ldr	r3, [r5, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L186
	subs	r4, r4, #1
	bne	.L185
	mvn	r0, #0
	b	.L184
.L186:
	mov	r0, #0
.L184:
	ldmfd	sp!, {r1, r2, r3, r4, r5, pc}
.L189:
	.align	2
.L188:
	.word	.LANCHOR0
	.word	100000
	.fnend
	.size	NandcWaitFlashReady, .-NandcWaitFlashReady
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
	ldr	r1, .L196
	add	r3, r5, r5, asl #2
	mvn	r6, #4
	add	r0, r1, r3
	add	r1, r1, #45
	add	r1, r1, r3
	mov	r2, #0
	ldr	ip, .L196+4
	mul	r5, r6, r5
	b	.L191
.L194:
	ldrb	r6, [ip, #3993]	@ zero_extendqisi2
	add	r2, r2, #1
	cmp	r6, #67
	addeq	r6, r0, r5
	addne	r6, r1, r5
	add	r6, r6, r3
	add	r0, r0, #1
	add	r1, r1, #1
	ldrsb	r6, [r6, #5]
	str	r6, [r4, #0]
.L191:
	ldrb	r6, [ip, #3992]	@ zero_extendqisi2
	cmp	r2, r6
	bcc	.L194
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, lr}
	b	NandcWaitFlashReady
.L197:
	.align	2
.L196:
	.word	.LANCHOR1+2656
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
	ldr	r3, .L199
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r3, r3, r0, asl #3
	ldrb	r6, [r3, #20]	@ zero_extendqisi2
	ldr	r5, [r3, #16]
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
.L200:
	.align	2
.L199:
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
	ldr	r3, .L202
	mov	ip, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r2
	ldrb	r2, [r3, #2569]	@ zero_extendqisi2
	ldr	r3, .L202+4
	add	r3, r3, r0, asl #3
	mov	r2, r2, asl #9
	ldrb	r5, [r3, #20]	@ zero_extendqisi2
	ldr	r4, [r3, #16]
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
.L203:
	.align	2
.L202:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashReadSpare, .-FlashReadSpare
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
	ldr	r9, .L218
	mov	r7, #239
	mov	r1, r4
	mov	r6, #128
	sub	r8, r9, #16
	mov	r5, #1
	mov	sl, #35
.L214:
	mov	r2, r9
	ldr	lr, .L218+4
	ldr	r3, [r2, r4]!
	ldrb	lr, [r4, lr]	@ zero_extendqisi2
	ldrb	r2, [r2, #4]	@ zero_extendqisi2
	cmp	lr, #152
	cmpne	lr, #69
	add	r2, r2, #8
	add	ip, r3, r2, asl #8
	beq	.L205
	cmp	lr, #173
	cmpne	lr, #44
	bne	.L206
.L205:
	cmp	r0, #1
	ldrb	fp, [r8, #3994]	@ zero_extendqisi2
	bne	.L207
	tst	fp, #1
	beq	.L206
	cmp	lr, #173
	str	r7, [ip, #8]
	streq	r0, [ip, #4]
	beq	.L216
	cmp	lr, #44
	streq	r0, [ip, #4]
	strne	r6, [ip, #4]
	moveq	ip, #5
	strne	r0, [r3, r2, asl #8]
	bne	.L212
	b	.L217
.L207:
	tst	fp, #4
	beq	.L206
	cmp	lr, #173
	str	r7, [ip, #8]
	bne	.L211
	str	r5, [ip, #4]
	mov	ip, #32
.L217:
	str	ip, [r3, r2, asl #8]
	b	.L212
.L211:
	cmp	lr, #44
	streq	r5, [ip, #4]
	streq	sl, [r3, r2, asl #8]
	beq	.L212
	str	r6, [ip, #4]
.L216:
	str	r1, [r3, r2, asl #8]
.L212:
	str	r1, [r3, r2, asl #8]
	str	r1, [r3, r2, asl #8]
	str	r1, [r3, r2, asl #8]
.L206:
	add	r4, r4, #8
	cmp	r4, #64
	bne	.L214
	mov	r0, #0
	bl	NandcWaitFlashReady
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L219:
	.align	2
.L218:
	.word	.LANCHOR0+16
	.word	.LANCHOR0+2160
	.fnend
	.size	FlashSetInterfaceMode, .-FlashSetInterfaceMode
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
	ldr	r3, .L222
	add	r3, r3, r0, asl #3
	ldrb	r5, [r3, #20]	@ zero_extendqisi2
	ldr	r2, [r3, #16]
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
.L223:
	.align	2
.L222:
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
	ldr	r3, .L226
	mov	r0, r4
	add	r3, r3, r4, asl #3
	ldrb	r7, [r3, #20]	@ zero_extendqisi2
	ldr	r6, [r3, #16]
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
	ldr	r3, [r6, r7, asl #8]
	strb	r3, [r5, #6]
	ldr	r3, [r6, r7, asl #8]
	strb	r3, [r5, #7]
	bl	NandcFlashDeCs
	ldrb	r2, [r5, #0]	@ zero_extendqisi2
	sub	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L224
	ldrb	r1, [r5, #2]	@ zero_extendqisi2
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	ldr	r0, .L226+4
	str	r1, [sp, #0]
	ldrb	r1, [r5, #3]	@ zero_extendqisi2
	str	r1, [sp, #4]
	ldrb	r1, [r5, #4]	@ zero_extendqisi2
	str	r1, [sp, #8]
	ldrb	r1, [r5, #5]	@ zero_extendqisi2
	str	r1, [sp, #12]
	add	r1, r4, #1
	bl	printk
.L224:
	ldmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, pc}
.L227:
	.align	2
.L226:
	.word	.LANCHOR0
	.word	.LC2
	.fnend
	.size	FlashReadIDRaw, .-FlashReadIDRaw
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
	ldr	r4, .L230
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	ldrb	r3, [r4, #3995]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L229
	ldrb	r3, [r4, #3994]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L229
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r3, #0
	strb	r3, [r4, #3995]
.L229:
	ldr	r3, [r4, #16]
	mov	r0, #0
	str	r0, [r3, #336]
	ldmfd	sp!, {r4, pc}
.L231:
	.align	2
.L230:
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
	ldr	r3, .L236
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r2
	add	r2, r3, r0, asl #3
	mov	r5, r1
	mov	r4, r0
	ldrb	r6, [r2, #20]	@ zero_extendqisi2
	ldr	r1, [r2, #16]
	ldrb	r2, [r3, #96]	@ zero_extendqisi2
	add	r6, r6, #8
	cmp	r2, #1
	ldrb	r2, [r3, #88]	@ zero_extendqisi2
	add	r6, r1, r6, asl #8
	str	r2, [r6, #8]
	bne	.L233
	mov	r8, #0
	uxtb	r2, r5
	str	r8, [r6, #4]
	str	r8, [r6, #4]
	str	r2, [r6, #4]
	mov	r2, r5, lsr #8
	str	r2, [r6, #4]
	mov	r2, r5, lsr #16
	str	r2, [r6, #4]
	ldrb	r3, [r3, #89]	@ zero_extendqisi2
	str	r3, [r6, #8]
	bl	NandcWaitFlashReady
	str	r8, [r6, #8]
	str	r8, [r6, #4]
	str	r8, [r6, #4]
	b	.L235
.L233:
	uxtb	r2, r5
	str	r2, [r6, #4]
	mov	r2, r5, lsr #8
	str	r2, [r6, #4]
	mov	r2, r5, lsr #16
	str	r2, [r6, #4]
	ldrb	r3, [r3, #89]	@ zero_extendqisi2
	str	r3, [r6, #8]
.L235:
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
.L237:
	.align	2
.L236:
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
	ldr	r3, .L297
	mvn	r2, #83
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	cmp	r4, #2
	strb	r0, [r3, #3132]
	strb	r2, [r3, #3136]
	mvn	r0, #82
	mvn	r1, #81
	mvn	r2, #80
	.pad #68
	sub	sp, sp, #68
	strb	r0, [r3, #3137]
	strb	r1, [r3, #3138]
	strb	r2, [r3, #3139]
	bne	.L239
	mvn	r2, #88
	strb	r2, [r3, #3136]
	ldr	r3, .L297+4
	mvn	r2, #8
	strb	r2, [r3, #2821]
	b	.L278
.L239:
	cmp	r4, #3
	bne	.L241
	mvn	r2, #79
	strb	r2, [r3, #3136]
	mvn	r2, #78
	strb	r2, [r3, #3137]
	mvn	r2, #77
	strb	r2, [r3, #3138]
	mvn	r2, #76
	strb	r2, [r3, #3139]
	mvn	r2, #75
	strb	r2, [r3, #3140]
	mvn	r2, #74
	strb	r2, [r3, #3141]
	mvn	r2, #73
	strb	r2, [r3, #3142]
	mvn	r2, #72
	b	.L293
.L241:
	cmp	r4, #4
	bne	.L242
	mvn	ip, #51
	strb	r0, [r3, #3141]
	strb	ip, [r3, #3136]
	mvn	ip, #64
	strb	r1, [r3, #3142]
	strb	ip, [r3, #3137]
	mvn	ip, #85
	strb	ip, [r3, #3138]
	mvn	ip, #84
	strb	ip, [r3, #3139]
	mvn	ip, #50
	strb	ip, [r3, #3140]
.L293:
	mov	r6, #8
	strb	r2, [r3, #3143]
	mov	r5, r6
	b	.L240
.L242:
	cmp	r4, #5
	bne	.L243
	mov	r2, #56
	mov	r6, #8
	strb	r2, [r3, #3136]
	mov	r2, #57
	strb	r2, [r3, #3137]
	mov	r2, #58
	strb	r2, [r3, #3138]
	mov	r2, #59
	strb	r2, [r3, #3139]
	b	.L292
.L243:
	cmp	r4, #6
	bne	.L278
	mov	r2, #14
	mov	r6, #12
	strb	r2, [r3, #3136]
	mov	r2, #15
	strb	r2, [r3, #3137]
	mov	r2, #16
	strb	r2, [r3, #3138]
	mov	r2, #17
	strb	r2, [r3, #3139]
	b	.L292
.L278:
	mov	r6, #7
.L292:
	mov	r5, #4
.L240:
	sub	r3, r4, #1
	cmp	r3, #1
	bhi	.L291
	b	.L296
.L250:
	add	r1, r2, r8
	add	r4, r2, #3152
	ldr	fp, .L297+8
	mov	r7, #0
	ldrb	r1, [r1, #3996]	@ zero_extendqisi2
	mov	r5, #55
	add	r2, r2, r1, asl #3
	add	r4, r4, r1, asl #6
	ldrb	sl, [r2, #20]	@ zero_extendqisi2
	ldr	r1, [r2, #16]
	add	sl, sl, #8
	add	sl, r1, sl, asl #8
.L246:
	add	r2, fp, r7
	str	r5, [sl, #8]
	mov	r0, #80
	ldrb	r2, [r2, #4]	@ zero_extendqisi2
	str	r2, [sl, #4]
	str	r3, [sp, #8]
	bl	NandcDelayns
	ldr	r2, [sl, #0]
	strb	r2, [r4, r7]
	add	r7, r7, #1
	ldr	r3, [sp, #8]
	uxtb	r2, r7
	cmp	r2, r6
	bcc	.L246
	mov	r2, #0
	b	.L247
.L248:
	ldrb	r7, [r0, r1, asl #2]	@ zero_extendqisi2
	ldrb	r5, [r4, r2]	@ zero_extendqisi2
	add	r5, r7, r5
	strb	r5, [ip, r1, asl #3]
	add	r1, r1, #1
	cmp	r1, #7
	bne	.L248
	add	r2, r2, #1
	cmp	r2, #4
	beq	.L249
.L247:
	mov	r1, #1
	add	ip, r4, r2
	add	r0, r2, r9
	b	.L248
.L249:
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
	b	.L244
.L296:
	ldr	r9, .L297+12
	mov	r3, r6
	mov	r8, #0
	mov	r6, r5
.L244:
	ldr	r2, .L297
	ldrb	r1, [r2, #4012]	@ zero_extendqisi2
	cmp	r1, r8
	bhi	.L250
	mov	r5, r6
	mov	r6, r3
	b	.L251
.L291:
	sub	r3, r4, #3
	cmp	r3, #3
	bhi	.L251
	mul	r3, r6, r5
	ldr	r8, .L297
	str	r6, [sp, #44]
	mov	r2, r3, asr #2
	mov	r3, r3, asl #4
	str	r3, [sp, #52]
	mov	r3, #0
	str	r2, [sp, #24]
	mov	r7, r3
	str	r3, [sp, #12]
	b	.L252
.L277:
	ldr	ip, [sp, #12]
	ldr	r1, .L297+16
	add	r3, r8, ip
	ldrb	r6, [r3, #3996]	@ zero_extendqisi2
	add	r3, r8, r6, asl #3
	mov	r0, r6
	add	r1, r1, r6, asl #6
	str	r1, [sp, #48]
	ldrb	fp, [r3, #20]	@ zero_extendqisi2
	ldr	r9, [r3, #16]
	mov	r3, #255
	add	fp, fp, #8
	add	sl, r9, fp, asl #8
	str	r3, [sl, #8]
	str	r3, [sp, #8]
	bl	NandcWaitFlashReady
	cmp	r4, #4
	mov	r2, #54
	str	r2, [sl, #8]
	ldr	r3, [sp, #8]
	streq	r3, [sl, #4]
	moveq	r3, #64
	streq	r3, [r9, fp, asl #8]
	moveq	r3, #204
	beq	.L294
.L253:
	sub	r3, r4, #5
	cmp	r3, #1
	ldrls	r2, .L297+8
	ldrlsb	r3, [r2, #4]	@ zero_extendqisi2
	strls	r3, [sl, #4]
	movls	r3, #82
	bls	.L295
.L255:
	mov	r3, #174
	str	r3, [sl, #4]
	mov	r3, #176
	str	r7, [r9, fp, asl #8]
.L294:
	str	r3, [sl, #4]
	mov	r3, #77
.L295:
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
.L259:
	add	r3, r3, #1
	ldr	r1, [sl, #0]
	uxtb	r3, r3
	cmp	r3, r2
	bcc	.L259
	ldr	ip, [r8, #4016]
	mov	r3, #0
	str	ip, [sp, #32]
.L260:
	ldr	r2, [sl, #0]
	ldr	lr, [sp, #32]
	strb	r2, [lr, r3]
	add	r3, r3, #1
	ldr	r2, [sp, #52]
	cmp	r3, r2
	blt	.L260
	ldr	r3, [sp, #24]
	mov	ip, r3, asl #2
	mov	lr, r3, asl #3
	ldr	r3, .L297
	str	ip, [sp, #40]
	ldr	r1, [r3, #4016]
	mov	r3, #8
	add	r1, r1, ip
.L262:
	mov	r2, #0
	mov	r0, r2
.L261:
	ldr	ip, [r1, r2]
	add	r0, r0, #1
	mvn	ip, ip
	str	ip, [r1, r2]
	ldr	ip, [sp, #24]
	add	r2, r2, #4
	cmp	r0, ip
	blt	.L261
	subs	r3, r3, #1
	add	r1, r1, lr
	bne	.L262
	str	r3, [sp, #20]
	str	r5, [sp, #60]
	b	.L263
.L267:
	mov	lr, #1
	mov	r0, #0
	mov	ip, lr, asl r2
	mov	lr, r0
	mov	r5, #16
	str	r3, [sp, #0]
	str	r5, [sp, #16]
.L265:
	ldr	r3, [sp, #56]
	ldr	r3, [r3, r0]
	and	r5, ip, r3
	ldr	r3, [sp, #16]
	cmp	r5, ip
	ldr	r5, [sp, #40]
	addeq	lr, lr, #1
	subs	r3, r3, #1
	add	r0, r0, r5
	str	r3, [sp, #16]
	bne	.L265
	cmp	lr, #8
	add	r2, r2, #1
	orrhi	r1, r1, ip
	cmp	r2, #32
	ldr	r3, [sp, #0]
	bne	.L267
	ldr	ip, [sp, #20]
	ldr	r2, [sp, #24]
	add	ip, ip, #1
	ldr	r5, [sp, #36]
	cmp	ip, r2
	str	ip, [sp, #20]
	str	r1, [r5, r3]
	add	r3, r3, #4
	bge	.L268
.L263:
	ldr	r5, [r8, #4016]
	mov	r1, #0
	mov	r2, r1
	add	ip, r5, r3
	str	ip, [sp, #56]
	str	r5, [sp, #36]
	b	.L267
.L268:
	cmp	r4, #6
	ldr	r5, [sp, #60]
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
	b	.L272
.L273:
	ldrb	lr, [ip], #1	@ zero_extendqisi2
	strb	lr, [r1, r3]
	add	r3, r3, #1
	uxtb	lr, r3
	cmp	lr, r5
	bcc	.L273
	ldr	r3, [sp, #16]
	add	r2, r2, #1
	ldr	ip, [sp, #20]
	add	r0, r0, r3
	ldr	r3, [sp, #44]
	add	r1, r1, ip
	cmp	r2, r3
	bge	.L274
.L272:
	mov	ip, r0
	mov	r3, #0
	b	.L273
.L274:
	mov	r3, #255
	mov	r0, r6
	str	r3, [sl, #8]
	bl	NandcWaitFlashReady
	ldr	ip, [sp, #28]
	cmp	ip, #1
	movhi	r3, #56
	strhi	r3, [sl, #8]
	bhi	.L276
	ldr	lr, .L297+8
	mov	r3, #54
	str	r3, [sl, #8]
	mvn	r1, #0
	ldr	r0, [sp, #12]
	ldrb	r3, [lr, #4]	@ zero_extendqisi2
	str	r3, [sl, #4]
	mov	r3, #22
	str	r7, [r9, fp, asl #8]
	str	r3, [sl, #8]
	bl	FlashReadCmd
.L276:
	mov	r0, r6
	bl	NandcWaitFlashReady
	ldr	r2, [sp, #12]
	add	r3, r2, #1
	uxtb	r3, r3
	str	r3, [sp, #12]
.L252:
	ldrb	r3, [r8, #4012]	@ zero_extendqisi2
	ldr	ip, [sp, #12]
	cmp	r3, ip
	bhi	.L277
	ldr	r6, [sp, #44]
.L251:
	ldr	r3, .L297
	strb	r5, [r3, #3133]
	strb	r6, [r3, #3134]
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L298:
	.align	2
.L297:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+3132
	.word	.LANCHOR1+2804
	.word	.LANCHOR0+3152
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
	bhi	.L301
	ldmfd	sp!, {r3, lr}
	b	HynixGetReadRetryDefault
.L301:
	cmp	r3, #49
	bne	.L302
	ldr	r0, .L308
	mov	r2, #64
	ldr	r1, .L308+4
	strb	r3, [r0, #3132]
	mov	r3, #4
	strb	r3, [r0, #3133]
	mov	r3, #15
	strb	r3, [r0, #3134]
	add	r0, r0, #3136
	b	.L306
.L302:
	cmp	r3, #33
	cmpne	r3, #65
	beq	.L303
	cmp	r3, #66
	bne	.L304
.L303:
	ldr	r0, .L308
	strb	r3, [r0, #3132]
	mov	r3, #4
	b	.L307
.L304:
	cmp	r3, #34
	cmpne	r3, #67
	bne	.L305
	ldr	r0, .L308
	strb	r3, [r0, #3132]
	mov	r3, #5
.L307:
	strb	r3, [r0, #3133]
	mov	r2, #45
	mov	r3, #7
	ldr	r1, .L308+8
	strb	r3, [r0, #3134]
	add	r0, r0, #3136
	b	.L306
.L305:
	cmp	r3, #35
	cmpne	r3, #68
	ldmnefd	sp!, {r3, pc}
	ldr	r0, .L308
	mov	r2, #95
	ldr	r1, .L308+12
	strb	r3, [r0, #3132]
	mov	r3, #5
	strb	r3, [r0, #3133]
	mov	r3, #17
	strb	r3, [r0, #3134]
	add	r0, r0, #3136
.L306:
	bl	memcpy
	ldmfd	sp!, {r3, pc}
.L309:
	.align	2
.L308:
	.word	.LANCHOR0
	.word	.LANCHOR1+2592
	.word	.LANCHOR1+2656
	.word	.LANCHOR1+2701
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
	ldr	r3, .L311
	add	r3, r3, r0, asl #3
	ldr	r3, [r3, #16]
	str	r1, [r3, #336]
	bx	lr
.L312:
	.align	2
.L311:
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
	ldr	r1, .L324
	bl	__aeabi_idiv
	ldr	r3, .L324+4
	ldr	r2, [r3, #2224]
	cmp	r0, #250
	movwgt	r1, #8354
	bgt	.L322
	cmp	r0, #220
	bgt	.L323
.L316:
	cmp	r0, #185
	movwgt	r1, #4226
	bgt	.L322
	cmp	r0, #160
	movwgt	r1, #4194
	bgt	.L322
	cmp	r4, #35
	movwls	r1, #4193
	bls	.L322
	cmp	r4, #99
	movwls	r1, #4225
	bls	.L322
.L323:
	movw	r1, #8322
.L322:
	str	r1, [r2, #4]
	ldr	r3, [r3, #2868]
	cmp	r3, #0
	ldrne	r2, .L324+4
	ldrne	r2, [r2, #2224]
	ldrne	r2, [r2, #4]
	strne	r2, [r3, #4]
	ldmfd	sp!, {r4, pc}
.L325:
	.align	2
.L324:
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
	movw	r2, #4225
	sub	r3, r3, #33
	cmp	r0, r2
	cmpne	r3, #1
	bls	.L327
	movw	r3, #4226
	movw	r2, #8322
	cmp	r0, r3
	cmpne	r0, r2
	bne	.L328
.L327:
	ldr	r3, .L329
	ldr	r2, [r3, #2224]
	ldr	r3, [r3, #2868]
	cmp	r3, #0
	str	r0, [r2, #4]
	ldrne	r2, [r2, #4]
	strne	r2, [r3, #4]
.L328:
	ldr	r3, .L329+4
	ldrb	r0, [r3, #2581]	@ zero_extendqisi2
	b	NandcTimeCfg
.L330:
	.align	2
.L329:
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
	ldr	ip, .L332
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, [ip, #2224]
	ldr	r4, [r4, #4]
	str	r4, [r0, #0]
	ldr	r0, [ip, #2224]
	ldr	r0, [r0, #0]
	str	r0, [r1, #0]
	ldr	r1, [ip, #2224]
	ldr	r1, [r1, #304]
	str	r1, [r2, #0]
	ldr	r2, [ip, #2224]
	ldr	r0, [r2, #308]
	ldr	r1, [r2, #344]
	uxtb	r2, r0
	orr	r2, r2, r1, asl #16
	str	r2, [r3, #0]
	ldmfd	sp!, {r4, pc}
.L333:
	.align	2
.L332:
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
	ldr	r2, .L340
	mov	r3, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, #16
	ldr	ip, [r2, #2224]
	cmp	r0, r4
	bfi	r3, r4, #8, #8
	mov	r1, #1
	str	r0, [r2, #4020]
	bfc	r3, #18, #1
	str	r1, [ip, #8]
	beq	.L339
.L335:
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L336
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	bne	.L336
.L339:
	bfc	r3, #4, #1
.L336:
	ldr	r2, [r2, #2868]
	orr	r3, r3, #1
	str	r3, [ip, #12]
	cmp	r2, #0
	strne	r1, [r2, #8]
	strne	r3, [r2, #12]
	ldmfd	sp!, {r4, pc}
.L341:
	.align	2
.L340:
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
	ldr	r3, .L343
	strb	r0, [r3, #4024]
	b	NandcBchSel
.L344:
	.align	2
.L343:
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
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	ldr	r3, .L351
	ldr	r2, [r3, #2224]
	ldr	r7, [r3, #2228]
	ldr	r6, [r3, #2232]
	ldr	r5, [r3, #2236]
	str	r7, [r2, #0]
	ldr	r4, [r3, #2240]
	ldr	r2, [r3, #2224]
	ldr	lr, [r3, #2244]
	ldr	ip, [r3, #2248]
	ldr	r0, [r3, #2252]
	ldr	r1, [r3, #2256]
	str	r6, [r2, #4]
	str	r5, [r2, #8]
	str	r4, [r2, #12]
	str	lr, [r2, #304]
	str	ip, [r2, #308]
	str	r0, [r2, #336]
	str	r1, [r2, #344]
	ldr	r2, [r3, #2868]
	cmp	r2, #0
	beq	.L346
	str	r7, [r2, #0]
	ldr	r3, [r3, #2868]
	str	r6, [r3, #4]
	str	r5, [r3, #8]
	str	r4, [r3, #12]
	str	lr, [r3, #304]
	str	ip, [r3, #308]
	str	r0, [r3, #336]
	str	r1, [r3, #344]
.L346:
	ldr	r5, .L351+4
	mov	r4, #0
.L348:
	ldrb	r3, [r5, r4, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L347
	uxtb	r0, r4
	bl	FlashReset
.L347:
	add	r4, r4, #1
	cmp	r4, #8
	bne	.L348
	ldr	r4, .L351
	ldrb	r3, [r4, #3995]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L349
	mov	r0, #1
	bl	NandcSetMode
	ldrb	r0, [r4, #3994]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3994]	@ zero_extendqisi2
	bl	NandcSetMode
	ldrb	r0, [r4, #2245]	@ zero_extendqisi2
	bl	NandcSetDdrPara
.L349:
	ldr	r3, .L351
	ldr	r3, [r3, #2864]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L352:
	.align	2
.L351:
	.word	.LANCHOR0
	.word	.LANCHOR0+2160
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
	bne	.L358
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L358:
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
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r7, r0
	ldr	r6, .L371
	.pad #20
	sub	sp, sp, #20
	mov	fp, r1
	mov	r4, #0
	mov	r5, r6
	b	.L360
.L366:
	mov	r8, #36
	rsb	r3, r4, fp
	mul	r8, r8, r4
	uxtb	r3, r3
	mov	r1, #0
	str	r3, [sp, #0]
	add	r2, sp, #8
	add	r3, sp, #12
	add	r9, r7, r8
	mov	r0, r9
	bl	LogAddr2PhyAddr
	ldrb	r3, [r6, #4012]	@ zero_extendqisi2
	mov	sl, r0
	ldr	r0, [sp, #12]
	cmp	r0, r3
	mvncs	r3, #0
	strcs	r3, [r7, r8]
	bcs	.L362
	ldrb	r3, [r6, #4025]	@ zero_extendqisi2
	cmp	r3, #0
	add	r3, r5, r0, asl #4
	ldr	r3, [r3, #2884]
	moveq	sl, #0
	cmp	r3, #0
	beq	.L364
	uxtb	r0, r0
	bl	FlashWaitCmdDone
.L364:
	ldr	r2, [sp, #12]
	mov	r1, #0
	cmp	sl, #0
	add	r3, r6, r2, asl #4
	movne	r0, #36
	str	r1, [r3, #2888]
	ldr	r1, [sp, #8]
	str	r9, [r3, #2884]
	str	r1, [r3, #2880]
	addne	r1, r4, #1
	add	r4, r4, sl
	mlane	r1, r0, r1, r7
	strne	r1, [r3, #2888]
	add	r3, r5, r2
	add	r2, r5, r2, asl #4
	ldrb	r8, [r3, #3996]	@ zero_extendqisi2
	strb	r8, [r2, #2876]
	mov	r0, r8
	bl	NandcFlashCs
	ldr	r3, [sp, #12]
	ldr	r1, [sp, #8]
	mov	r0, r8
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #2800]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r8
	ldr	r1, [sp, #8]
	mov	r2, sl
	bl	FlashEraseCmd
	mov	r0, r8
	bl	NandcFlashDeCs
.L362:
	add	r4, r4, #1
.L360:
	cmp	r4, fp
	bcc	.L366
	ldr	r4, .L371
	ldr	r0, [r4, #2224]
	bl	NandcIqrWaitFlashReady
	ldr	r0, [r4, #2868]
	cmp	r0, #0
	beq	.L367
	ldrb	r3, [r4, #4026]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L367
	bl	NandcIqrWaitFlashReady
.L367:
	mov	r4, #0
	ldr	r5, .L371
	b	.L368
.L369:
	uxtb	r0, r4
	add	r4, r4, #1
	bl	FlashWaitCmdDone
.L368:
	ldrb	r3, [r5, #4012]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L369
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L372:
	.align	2
.L371:
	.word	.LANCHOR0
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
	ldr	r3, .L375
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
.L376:
	.align	2
.L375:
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
.L378:
	ldr	r3, [r0, #8]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #1048576
	beq	.L378
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
	ldr	r2, .L393
	mov	ip, #16
	mov	r6, r1
	mov	r4, #0
	add	r0, r2, r0, asl #3
	.pad #20
	sub	sp, sp, #20
	ldr	r5, [r0, #16]
	ldrb	r0, [r0, #20]	@ zero_extendqisi2
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
	ldr	r3, [r2, #12]
	cmp	r3, #3
	bls	.L381
	ldr	r3, [r5, #16]
	cmp	r9, #0
	cmpeq	r1, #0
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bfc	r3, #2, #1
	str	r3, [sp, #12]
	beq	.L382
	cmp	r6, #0
	beq	.L383
	ldr	r3, [r2, #4020]
	mov	ip, r1
	cmp	r3, #24
	movhi	r3, #128
	movls	r3, #64
	str	r3, [sp, #0]
	mov	r3, r8, lsr r0
	str	r3, [sp, #4]
	mov	r3, #0
	mov	r0, r3
	b	.L385
.L388:
	cmp	r1, #0
	mov	lr, r3, lsr #2
	add	r0, r0, #1
	ldrneh	fp, [ip, #2]
	mvneq	fp, #0
	ldrneh	sl, [ip], #4
	ldreq	sl, [r2, #4032]
	orrne	sl, sl, fp, asl #16
	ldrne	fp, [r2, #4032]
	streq	fp, [sl, lr, asl #2]
	strne	sl, [fp, lr, asl #2]
	ldr	lr, [sp, #0]
	add	r3, r3, lr
.L385:
	ldr	sl, [sp, #4]
	cmp	r0, sl
	bcc	.L388
.L383:
	mov	r0, r5
	add	r8, r8, #1
	bl	rk_nandc_xfer_irq_flag_init
	mov	r0, r5
	mov	r1, #0
	mov	r8, r8, asr #1
	bl	NandCIrqEnable
	bfi	r4, r8, #22, #6
	ldr	r8, .L393
	cmp	r9, #0
	ubfx	sl, r4, #22, #5
	mov	r2, r6
	movne	r0, r9
	ldreq	r0, [r8, #4028]
	mov	r1, sl, asl #10
	ldr	r3, [r8, #4032]
	str	r3, [r8, #4040]
	str	r0, [r8, #4036]
	bl	rknand_dma_map_single
	mov	r2, r6
	mov	r1, sl, asl #7
	str	r0, [r8, #4044]
	ldr	r0, [r8, #4040]
	bl	rknand_dma_map_single
	mov	r3, #1
	str	r3, [r8, #4052]
	mov	r2, #16
	ldr	r3, [r8, #4044]
	tst	r9, #3
	str	r0, [r8, #4048]
	str	r3, [r5, #20]
	ldr	r3, [r8, #4048]
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
.L382:
	ldr	r3, [sp, #12]
	str	r3, [r5, #16]
.L381:
	str	r7, [r5, #12]
	str	r4, [r5, #8]
	orr	r4, r4, #4
	str	r4, [r5, #8]
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L394:
	.align	2
.L393:
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
	bne	.L396
	cmp	r3, #0
	beq	.L397
	mov	r0, r2
	mov	r1, r3
	mov	r2, #1024
	bl	memcpy
.L397:
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
.L396:
	cmp	r3, #0
	beq	.L399
	mov	r1, r2
	mov	r0, r3
	mov	r2, #1024
	bl	memcpy
.L399:
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
	b	.L401
.L402:
	add	r3, r3, #1
	mov	r2, r2, asl #1
	uxth	r3, r3
.L401:
	cmp	r2, r0
	bls	.L402
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
	ldr	r3, .L407
	mov	r2, #4064
	cmp	r0, #23
	movw	r1, #4074
	ldrh	r2, [r3, r2]
	movls	r0, #24
	ldrh	r1, [r3, r1]
	str	r0, [r3, #4060]
	mul	r2, r2, r0
	rsb	r0, r0, r1
	movw	r1, #4072
	strh	r0, [r3, r1]	@ movhi
	mov	r0, #0
	ldr	r1, [r3, #4080]
	str	r2, [r3, #4068]
	rsb	r2, r2, r1
	str	r2, [r3, #4076]
	bx	lr
.L408:
	.align	2
.L407:
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
	movw	r2, #4084
	ldr	r3, .L422
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
	movw	r2, #4074
	strh	r6, [r3, r2]	@ movhi
	mov	r3, #0
	ldr	r2, .L422+4
.L410:
	strb	r3, [r3, r2]
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L410
	ldrh	r2, [r4, #14]
	ldrh	r3, [r4, #20]
	cmp	r3, r2, lsr #8
	bcs	.L411
	ldr	r2, .L422+4
	sub	ip, r1, #1
	uxtb	r7, r0
	mov	r3, #0
	rsb	sl, r0, r2
	str	sl, [sp, #8]
	mla	ip, r0, ip, r2
	mov	r8, r7, asl #1
	uxtb	r8, r8
	str	ip, [sp, #12]
	b	.L412
.L413:
	strb	ip, [r9, r2]
	add	fp, ip, r7
	add	ip, ip, r8
	add	r5, r5, #1
	strb	fp, [sl, r2]
	uxtb	ip, ip
.L415:
	cmp	r5, r1
	add	r2, r2, r0
	bcc	.L413
	add	r3, r3, #1
.L412:
	cmp	r3, r0
	bcs	.L414
	ldr	fp, [sp, #8]
	mov	r2, #0
	uxtb	ip, r3
	mov	r5, r2
	add	r9, fp, r3
	ldr	fp, [sp, #12]
	add	sl, fp, r3
	b	.L415
.L414:
	ldr	r3, .L422
	movw	r2, #4086
	mov	r1, r1, asl #1
	mov	r6, r6, lsr #1
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #4074
	strh	r6, [r3, r2]	@ movhi
.L411:
	ldr	r3, .L422+8
	cmp	lr, #1
	ldr	r2, .L422+12
	mov	r1, #5
	ldr	r9, .L422
	mov	ip, #0
	ldr	r8, .L422+8
	strh	r1, [r3, r2]	@ movhi
	streqh	lr, [r3, r2]	@ movhi
	ldr	r1, .L422+16
	ldr	r2, .L422+20
	strh	ip, [r3, r1]	@ movhi
	mov	r1, #4352
	strh	r1, [r3, r2]	@ movhi
	ldrb	r3, [r9, #0]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, .L422+8
	movne	r1, #384
	strneh	r1, [r3, r2]	@ movhi
	movw	r3, #4086
	ldrh	r5, [r9, r3]
	mov	r3, #4064
	mul	r5, r5, r0
	uxth	r5, r5
	strh	r5, [r9, r3]	@ movhi
	add	r3, r3, #10
	ldrh	r6, [r9, r3]
	ldr	r3, .L422+24
	mul	r0, r0, r6
	uxth	r0, r0
	strh	r0, [r8, r3]	@ movhi
	bl	Ftl_log2
	ldr	r3, .L422+28
	ldr	r2, .L422+32
	ldrh	fp, [r4, #18]
	ldrh	sl, [r4, #20]
	strh	r0, [r8, r3]	@ movhi
	mov	r0, sl
	ldrh	r3, [r4, #16]
	mul	r1, r3, r5
	strh	r3, [r8, r2]	@ movhi
	add	r2, r2, #2
	strh	fp, [r8, r2]	@ movhi
	add	r2, r2, #2
	strh	r1, [r8, r2]	@ movhi
	add	r2, r2, #2
	strh	sl, [r8, r2]	@ movhi
	str	r3, [sp, #4]
	bl	Ftl_log2
	ldr	r2, .L422+36
	cmp	r6, #1024
	ldr	r1, .L422+40
	strh	r0, [r8, r2]	@ movhi
	mov	r2, sl, asl #9
	mov	r7, r0
	uxth	r2, r2
	strh	r2, [r8, r1]	@ movhi
	add	r1, r1, #2
	mov	r2, r2, lsr #8
	strh	r2, [r8, r1]	@ movhi
	ldrh	r1, [r4, #26]
	mov	r4, r8
	ldr	r2, .L422+44
	strh	r1, [r8, r2]	@ movhi
	uxtbhi	r1, r6
	mul	r2, r6, r5
	str	r2, [r9, #4080]
	ldr	r2, .L422+16
	ldr	r3, [sp, #4]
	strhih	r1, [r8, r2]	@ movhi
	ldrh	r2, [r8, r2]
	mul	r1, fp, sl
	rsb	r2, r2, r6
	mov	r6, r6, asl #6
	mul	r2, r2, r5
	mul	r2, sl, r2
	mul	r3, r3, r2
	mov	r3, r3, asr #11
	str	r3, [r8, #-4036]
	ldr	r3, .L422+20
	ldrh	r8, [r8, r3]
	mov	r0, r8, asl #3
	mov	r8, r8, asr r7
	bl	__aeabi_idiv
	ldr	r3, .L422+48
	add	r7, r7, #9
	add	r8, r8, #2
	mov	r6, r6, asr r7
	uxth	r6, r6
	uxth	r0, r0
	strh	r0, [r4, r3]	@ movhi
	cmp	r0, #4
	ldr	r4, .L422+8
	ldrls	r2, .L422+8
	movls	r1, #4
	strlsh	r1, [r2, r3]	@ movhi
	mov	r1, r5
	ldr	r3, .L422+52
	strh	r8, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r6, [r4, r3]	@ movhi
	mul	r3, r5, r6
	add	r6, r6, #8
	str	r3, [r4, #-4024]
	ldr	r3, .L422+48
	ldrh	r0, [r4, r3]
	bl	__aeabi_uidiv
	ldr	r3, .L422
	cmp	r5, #1
	mov	r5, r3
	uxtah	r6, r6, r0
	str	r6, [r3, #4060]
	addeq	r6, r6, #4
	streq	r6, [r3, #4060]
	ldr	r0, [r3, #4060]
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r3, [r5, #4060]
	ldr	r2, [r5, #4076]
	mov	r0, #2048
	str	r3, [r4, #-4020]
	ldr	r3, .L422+32
	mov	r2, r2, asl #2
	ldrh	r3, [r4, r3]
	mul	r2, r3, r2
	ldr	r3, .L422+36
	ldrh	r3, [r4, r3]
	add	r3, r3, #9
	mov	r2, r2, lsr r3
	ldr	r3, .L422+56
	add	r2, r2, #2
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #30
	ldrh	r1, [r4, r3]
	bl	__aeabi_idiv
	ldr	r3, .L422+60
	strh	r0, [r4, r3]	@ movhi
	mov	r0, #0
	str	r0, [r4, #-4012]
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L423:
	.align	2
.L422:
	.word	.LANCHOR0
	.word	.LANCHOR0+4090
	.word	.LANCHOR2
	.word	-4062
	.word	-4060
	.word	-4058
	.word	-4056
	.word	-4054
	.word	-4052
	.word	-4044
	.word	-4042
	.word	-4038
	.word	-4032
	.word	-4030
	.word	-4016
	.word	-4014
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
	ldr	r3, .L431
	ldr	r2, .L431+4
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L429
	ldr	r2, .L431+8
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #-4004]
	mov	r3, #0
	b	.L426
.L428:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L430
	add	r3, r3, #1
	uxth	r3, r3
.L426:
	cmp	r3, r1
	bne	.L428
	mov	r0, #0
	bx	lr
.L430:
	mov	r0, #1
	bx	lr
.L429:
	mov	r0, r2
	bx	lr
.L432:
	.align	2
.L431:
	.word	.LANCHOR2
	.word	-4008
	.word	-4032
	.fnend
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheMetchLpa
	.type	FtlCacheMetchLpa, %function
FtlCacheMetchLpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L441
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #-4000]
	cmp	r3, #0
	beq	.L438
	ldr	ip, [r2, #-3996]
	mov	r2, #0
.L437:
	ldr	r4, [ip, #16]
	cmp	r4, r0
	bcc	.L435
	cmp	r4, r1
	bls	.L439
.L435:
	add	r2, r2, #1
	add	ip, ip, #36
	cmp	r2, r3
	bne	.L437
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L439:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L438:
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L442:
	.align	2
.L441:
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
	ldr	r3, .L444
	ldr	r0, [r3, #-4012]
	bx	lr
.L445:
	.align	2
.L444:
	.word	.LANCHOR2
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
	ldr	r3, .L447
	ldr	r0, [r3, #-4012]
	bx	lr
.L448:
	.align	2
.L447:
	.word	.LANCHOR2
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
	ldr	r3, .L450
	ldr	r0, [r3, #-3992]
	bx	lr
.L451:
	.align	2
.L450:
	.word	.LANCHOR2
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
	ldr	r2, .L453
	mov	r3, #4064
	ldrh	r3, [r2, r3]
	ldr	r2, .L453+4
	ldr	r0, [r2, #-3988]
	mul	r0, r0, r3
	bx	lr
.L454:
	.align	2
.L453:
	.word	.LANCHOR0
	.word	.LANCHOR2
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
	ldr	r3, .L456
	mov	r1, #4064
	movw	r2, #4074
	ldrh	r2, [r3, r2]
	ldrh	r0, [r3, r1]
	mul	r0, r0, r2
	bx	lr
.L457:
	.align	2
.L456:
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
	mov	r5, r0
	ldr	r4, .L459
	ldr	r3, .L459+4
	ldrh	r6, [r4, r3]
	mov	r1, r6
	bl	__aeabi_uidiv
	uxth	r2, r0
	mls	r3, r6, r2, r5
	add	r1, r4, r2, asl #2
	mov	r6, #1
	ldr	r0, [r1, #-3956]
	uxth	r3, r3
	and	r1, r3, #31
	mov	ip, r3, lsr #5
	ldr	lr, [r0, ip, asl #2]
	orr	r1, lr, r6, asl r1
	str	r1, [r0, ip, asl #2]
	str	r1, [sp, #0]
	mov	r1, r5
	ldr	r0, .L459+8
	bl	printk
	ldr	r3, .L459+12
	mov	r0, #0
	ldrh	r2, [r4, r3]
	add	r2, r2, r6
	strh	r2, [r4, r3]	@ movhi
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L460:
	.align	2
.L459:
	.word	.LANCHOR2
	.word	-4056
	.word	.LC3
	.word	-3978
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
	ldr	r3, .L462
	ldr	r5, .L462+4
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
	ldr	r3, [r5, #-3956]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr r4
	and	r0, r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L463:
	.align	2
.L462:
	.word	-4056
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
	.global	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, #0
	mov	r5, r4
	ldr	sl, .L469
	ldr	r8, .L469+4
	movw	r6, #4086
	ldr	r7, .L469+8
	b	.L466
.L468:
	mov	r0, r5
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	uxth	r5, r5
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
.L466:
	ldrh	r3, [sl, r8]
	ldrh	r2, [r7, r6]
	mul	r3, r2, r3
	cmp	r5, r3
	blt	.L468
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L470:
	.align	2
.L469:
	.word	.LANCHOR2
	.word	-4056
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
	ldr	r2, .L472
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r3, #4088
	ldrh	r4, [r2, r3]
	mov	r7, r1
	mov	r6, r0
	mov	r1, r4
	bl	__aeabi_uidiv
	ldr	r3, .L472+4
	ldr	r2, .L472+8
	mov	r1, r4
	mul	r7, r4, r7
	ldrh	r5, [r2, r3]
	mla	r5, r5, r0, r7
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L473:
	.align	2
.L472:
	.word	.LANCHOR0
	.word	-4056
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
	ldr	r2, .L475
	movw	r3, #4088
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r4, [r2, r3]
	ldr	r3, .L475+4
	ldr	r2, .L475+8
	ldrh	r1, [r2, r3]
	bl	__aeabi_uidiv
	mov	r1, r4
	mul	r5, r0, r4
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r4, r5, r6, pc}
.L476:
	.align	2
.L475:
	.word	.LANCHOR0
	.word	-4056
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
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r2, .L478
	ldr	r3, .L478+4
	ldrh	r1, [r2, r3]
	bl	__aeabi_uidivmod
	ldr	r2, .L478+8
	movw	r3, #4088
	uxth	r0, r1
	ldrh	r1, [r2, r3]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r3, pc}
.L479:
	.align	2
.L478:
	.word	.LANCHOR2
	.word	-4056
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
	bls	.L481
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L481:
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
	ldr	r3, .L484
	ldr	r2, .L484+4
	ldrh	r0, [r2, r3]
	rsbs	r0, r0, #1
	movcc	r0, #0
	bx	lr
.L485:
	.align	2
.L484:
	.word	-3918
	.word	.LANCHOR2
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
	ldr	r3, .L487
	ldr	r2, .L487+4
	ldrh	r0, [r2, r3]
	sub	r2, r0, #1024
	rsbs	r0, r2, #0
	adc	r0, r0, r2
	bx	lr
.L488:
	.align	2
.L487:
	.word	-3918
	.word	.LANCHOR2
	.fnend
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r4, .L492
	ldr	r3, .L492+4
	ldrh	r3, [r4, r3]
	cmp	r3, #1024
	ldmeqfd	sp!, {r4, r5, r6, pc}
	cmp	r1, #0
	beq	.L491
	bl	P2V_block_in_plane
	mov	r3, r5, asl #10
	mov	r1, #1
	mov	r6, r0
	ldr	r0, [r4, #-1868]
	mov	r6, r6, asl #1
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #-1864]
	ldrh	r2, [r3, r6]
	add	r2, r2, #1
	strh	r2, [r3, r6]	@ movhi
	ldr	r3, [r4, #-1860]
	add	r3, r3, #1
	str	r3, [r4, #-1860]
.L491:
	ldr	r3, .L492
	ldr	r2, .L492+4
	ldr	r0, .L492+8
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	ldr	r2, .L492+12
	ldrh	r1, [r3, r2]
	add	ip, r1, #4
	add	r1, r1, #1
	bic	r1, r1, #64512
	add	ip, r3, ip, asl #1
	strh	r5, [ip, r0]	@ movhi
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L493:
	.align	2
.L492:
	.word	.LANCHOR2
	.word	-3918
	.word	-3924
	.word	-3920
	.fnend
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r5, #0
	ldr	r4, .L504
	ldr	r7, .L504+4
	ldr	r6, .L504+8
	b	.L495
.L496:
	ldrh	r3, [r4, r6]
	add	r3, r5, r3
	add	r3, r4, r3, asl #1
	sub	r3, r3, #3904
	ldrh	r0, [r3, #-12]
	bl	P2V_block_in_plane
	ldr	r2, [r4, #-1864]
	ldr	r3, [r4, #-1856]
	mov	r0, r0, asl #1
	ldrh	r2, [r2, r0]
	str	r2, [r3, r5, asl #2]
	add	r5, r5, #1
	uxth	r5, r5
.L495:
	ldrh	r3, [r4, r7]
	cmp	r3, r5
	bhi	.L496
	mov	r3, #0
	ldr	r1, .L504
	ldr	r7, .L504+4
	ldr	r6, .L504+8
	b	.L497
.L502:
	add	r5, r3, #1
	ldr	r0, [r1, #-1856]
	mov	r2, r3
	uxth	r5, r5
	mov	ip, r5
	b	.L498
.L500:
	ldr	sl, [r0, r2, asl #2]
	ldr	r8, [r0, ip, asl #2]
	cmp	sl, r8
	movhi	r2, ip
	add	ip, ip, #1
	uxth	ip, ip
.L498:
	cmp	ip, r4
	bcc	.L500
	cmp	r3, r2
	beq	.L501
	ldr	r4, [r0, r3, asl #2]
	ldr	ip, [r0, r2, asl #2]
	str	r4, [r0, r2, asl #2]
	ldr	r0, [r1, #-1856]
	str	ip, [r0, r3, asl #2]
	ldrh	r0, [r1, r6]
	add	r2, r0, r2
	add	r3, r0, r3
	add	r2, r1, r2, asl #1
	add	r3, r1, r3, asl #1
	sub	r2, r2, #3904
	sub	r3, r3, #3904
	ldrh	ip, [r2, #-12]
	ldrh	r0, [r3, #-12]
	strh	r0, [r2, #-12]	@ movhi
	strh	ip, [r3, #-12]	@ movhi
.L501:
	mov	r3, r5
.L497:
	ldrh	r4, [r1, r7]
	sub	r2, r4, #1
	cmp	r3, r2
	blt	.L502
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L505:
	.align	2
.L504:
	.word	.LANCHOR2
	.word	-3918
	.word	-3922
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	IsInFreeQueue
	.type	IsInFreeQueue, %function
IsInFreeQueue:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L512
	ldr	r3, .L512+4
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrh	ip, [r2, r3]
	cmp	ip, #1024
	beq	.L510
	sub	r3, r3, #4
	ldrh	r4, [r2, r3]
	mov	r3, #0
	b	.L508
.L509:
	add	r1, r3, r4
	mov	r1, r1, asl #22
	add	r1, r2, r1, lsr #21
	sub	r1, r1, #3904
	sub	r1, r1, #12
	ldrh	r1, [r1, #0]
	cmp	r1, r0
	beq	.L511
	add	r3, r3, #1
.L508:
	cmp	r3, ip
	bcc	.L509
.L510:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L511:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L513:
	.align	2
.L512:
	.word	.LANCHOR2
	.word	-3918
	.fnend
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r3, .L517
	ldr	r4, .L517+4
	ldrh	r2, [r3, r4]
	cmp	r2, #0
	beq	.L516
	ldr	r1, .L517+8
	sub	r2, r2, #1
	ldr	r0, .L517+12
	ldrh	ip, [r3, r1]
	add	r5, ip, #4
	add	ip, ip, #1
	bic	ip, ip, #64512
	add	r5, r3, r5, asl #1
	ldrh	r0, [r5, r0]
	strh	r2, [r3, r4]	@ movhi
	strh	ip, [r3, r1]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L516:
	mov	r0, r2
	ldmfd	sp!, {r4, r5, pc}
.L518:
	.align	2
.L517:
	.word	.LANCHOR2
	.word	-3918
	.word	-3922
	.word	-3924
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
	beq	.L520
	ldr	r1, .L534
	mov	ip, #6
	mul	ip, ip, r0
	mvn	r6, #0
	ldr	r4, [r1, #-1852]
	mov	r8, r1
	add	r2, r4, ip
	strh	r6, [r2, #2]	@ movhi
	strh	r6, [r4, ip]	@ movhi
	ldr	r3, [r1, #-1848]
	cmp	r3, #0
	beq	.L533
.L521:
	ldr	r7, [r1, #-1844]
	mov	sl, r0, asl #1
	ldrh	r1, [r2, #4]
	ldr	r9, .L534+4
	ldrh	r5, [r7, sl]
	cmp	r1, #0
	ldr	fp, [r8, #-1864]
	mulne	r6, r1, r5
	ldr	r5, [r8, #-1852]
	add	sl, fp, sl
	str	sl, [sp, #4]
	rsb	r1, r5, r3
	mov	r1, r1, asr #1
	mul	r1, r9, r1
	uxth	r1, r1
.L528:
	mov	r9, r1, asl #1
	ldrh	r8, [r3, #4]
	ldrh	sl, [r7, r9]
	cmp	r8, #0
	mvneq	r8, #0
	mulne	r8, r8, sl
	cmp	r8, r6
	bne	.L524
	ldrh	sl, [fp, r9]
	ldr	r9, [sp, #4]
	ldrh	r8, [r9, #0]
	cmp	sl, r8
	bcc	.L526
	b	.L525
.L524:
	bhi	.L525
.L526:
	ldrh	r8, [r3, #0]
	movw	sl, #65535
	cmp	r8, sl
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	ldreq	r3, .L534
	streq	r2, [r3, #-1840]
	beq	.L520
.L527:
	mov	r1, #6
	mla	r3, r1, r8, r5
	mov	r1, r8
	b	.L528
.L525:
	strh	r1, [r4, ip]	@ movhi
	ldrh	r1, [r3, #2]
	strh	r1, [r2, #2]	@ movhi
	ldr	r1, .L534
	ldr	ip, [r1, #-1848]
	cmp	r3, ip
	bne	.L529
	strh	r0, [r3, #2]	@ movhi
.L533:
	str	r2, [r1, #-1848]
	b	.L520
.L529:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-1852]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L520:
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L535:
	.align	2
.L534:
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
	ldr	r2, .L537
	ldr	r3, .L537+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L538:
	.align	2
.L537:
	.word	.LANCHOR2
	.word	-1836
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
	beq	.L540
	ldr	r1, .L547
	mov	r5, #6
	mul	ip, r5, r0
	mvn	r3, #0
	ldr	r4, [r1, #-1852]
	add	r2, r4, ip
	strh	r3, [r2, #2]	@ movhi
	strh	r3, [r4, ip]	@ movhi
	ldr	r3, [r1, #-1832]
	cmp	r3, #0
	beq	.L546
.L541:
	ldr	sl, [r1, #-1852]
	mov	r8, r0, asl #1
	ldr	r7, [r1, #-1864]
	rsb	r1, sl, r3
	ldr	r9, .L547+4
	mov	r1, r1, asr #1
	ldrh	r8, [r7, r8]
	mul	r1, r9, r1
	mov	r9, r5
	uxth	r1, r1
.L544:
	mov	r5, r1, asl #1
	ldrh	r5, [r7, r5]
	cmp	r5, r8
	bcs	.L542
	ldrh	r5, [r3, #0]
	cmp	r5, r6
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	beq	.L540
.L543:
	mla	r3, r9, r5, sl
	mov	r1, r5
	b	.L544
.L542:
	ldrh	r5, [r3, #2]
	strh	r5, [r2, #2]	@ movhi
	strh	r1, [r4, ip]	@ movhi
	ldr	r1, .L547
	ldr	ip, [r1, #-1832]
	cmp	r3, ip
	bne	.L545
	strh	r0, [r3, #2]	@ movhi
.L546:
	str	r2, [r1, #-1832]
	b	.L540
.L545:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-1852]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L540:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L548:
	.align	2
.L547:
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
	ldr	r2, .L550
	ldr	r3, .L550+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L551:
	.align	2
.L550:
	.word	.LANCHOR2
	.word	-1828
	.fnend
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.type	List_remove_node, %function
List_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L557
	mov	r2, #6
	mul	r1, r2, r1
	ldr	r3, [ip, #-1852]
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r4, r3, r1
	ldr	r5, [r0, #0]
	movw	r6, #65535
	cmp	r4, r5
	ldrh	r5, [r3, r1]
	bne	.L553
	cmp	r5, r6
	mlane	r2, r2, r5, r3
	moveq	r2, #0
	streq	r2, [r0, #0]
	strne	r2, [r0, #0]
	mvnne	r0, #0
	strneh	r0, [r2, #2]	@ movhi
	b	.L555
.L553:
	cmp	r5, r6
	ldrh	r0, [r4, #2]
	mlane	r5, r2, r5, r3
	muleq	r2, r2, r0
	mvneq	r0, #0
	strneh	r0, [r5, #2]	@ movhi
	ldrneh	r6, [r4, #2]
	ldrneh	r5, [r3, r1]
	ldrne	r0, [ip, #-1852]
	mulne	r2, r2, r6
	streqh	r0, [r3, r2]	@ movhi
	strneh	r5, [r0, r2]	@ movhi
.L555:
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r3, r1]	@ movhi
	strh	r2, [r4, #2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L558:
	.align	2
.L557:
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
	beq	.L564
	ldr	r2, .L565
	movw	lr, #65535
	mov	ip, #6
	ldr	r4, [r2, #-1852]
	b	.L561
.L563:
	mla	r3, ip, r2, r4
	sub	r1, r1, #1
	uxth	r1, r1
.L561:
	cmp	r1, #0
	beq	.L562
	ldrh	r2, [r3, #0]
	cmp	r2, lr
	bne	.L563
.L562:
	rsb	r4, r4, r3
	ldr	r3, .L565+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L564:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L566:
	.align	2
.L565:
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
	ldr	r2, .L574
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #-1848]
	cmp	r3, #0
	beq	.L573
	ldr	r2, [r2, #-1852]
	movw	r4, #65535
	mov	ip, #6
	b	.L570
.L572:
	mla	r3, ip, r1, r2
	sub	r0, r0, #1
	uxth	r0, r0
.L570:
	cmp	r0, #0
	beq	.L571
	ldrh	r1, [r3, #0]
	cmp	r1, r4
	bne	.L572
.L571:
	rsb	r3, r2, r3
	ldr	r0, .L574+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L573:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L575:
	.align	2
.L574:
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
	ldr	r4, .L582
	ldr	r3, .L582+4
	ldrh	r3, [r4, r3]
	cmp	r3, r0
	beq	.L577
	ldr	r3, .L582+8
	ldrh	r3, [r4, r3]
	cmp	r3, r0
	beq	.L577
	ldr	r3, .L582+12
	ldrh	r3, [r4, r3]
	cmp	r3, r0
	beq	.L577
	ldr	r3, [r4, #-1852]
	mov	ip, #6
	ldr	r2, [r4, #-1848]
	mla	r1, ip, r0, r3
	cmp	r1, r2
	beq	.L577
	ldrh	r6, [r1, #4]
	mov	r2, r5, asl #1
	ldrh	r1, [r1, #2]
	ldr	r0, [r4, #-1844]
	mla	r1, ip, r1, r3
	ldr	ip, .L582+16
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
	bcs	.L577
	ldr	r0, .L582+20
	mov	r1, r5
	bl	List_remove_node
	ldr	r3, .L582+24
	mov	r0, r5
	ldrh	r2, [r4, r3]
	sub	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	bl	INSERT_DATA_LIST
.L577:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L583:
	.align	2
.L582:
	.word	.LANCHOR2
	.word	-1824
	.word	-1776
	.word	-1728
	.word	-1431655765
	.word	.LANCHOR2-1848
	.word	-1836
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
	b	.L585
.L588:
	mov	r7, r3
	add	r3, r3, #2
	ldrh	r6, [r7, #0]
	cmp	r6, #0
	bne	.L586
	bl	FtlFreeSysBlkQueueOut
	cmp	r0, #0
	strh	r0, [r7, #0]	@ movhi
	beq	.L587
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4, #0]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L587
.L586:
	add	r5, r5, #1
	uxth	r5, r5
.L585:
	cmp	r5, r2
	bne	.L588
.L587:
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
	ldr	r2, .L601
	movw	r1, #65535
	ldr	r3, .L601+4
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	ldrh	r3, [r2, r3]
	ldr	ip, [r2, #-1680]
	mov	r2, #0
	mov	r0, r2
	b	.L590
.L592:
	add	r2, r2, #12
	add	r4, ip, r2
	ldrh	r4, [r4, #-12]
	cmp	r4, r1
	ldmeqfd	sp!, {r4, r5, r6, r7, pc}
	add	r0, r0, #1
	uxth	r0, r0
.L590:
	cmp	r0, r3
	bne	.L592
	mov	r1, #0
	mov	r0, r3
	mov	r5, #-2147483648
	mov	r2, r1
	b	.L593
.L595:
	add	r4, ip, r1
	ldr	r4, [r4, #4]
	cmp	r4, #0
	blt	.L594
	cmp	r4, r5
	movcc	r5, r4
	movcc	r0, r2
.L594:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L593:
	cmp	r2, r3
	bne	.L595
	cmp	r0, r3
	ldmccfd	sp!, {r4, r5, r6, r7, pc}
	ldr	r2, .L601+8
	mov	r0, r3
	ldr	r1, .L601
	mvn	r4, #0
	ldrh	r6, [r1, r2]
	mov	r2, #0
	mov	r1, r2
	b	.L596
.L598:
	add	r5, ip, r2
	ldr	r5, [r5, #4]
	cmp	r5, r4
	bcs	.L597
	ldrh	r7, [ip, r2]
	cmp	r7, r6
	movne	r4, r5
	movne	r0, r1
.L597:
	add	r1, r1, #1
	add	r2, r2, #12
	uxth	r1, r1
.L596:
	cmp	r1, r3
	bne	.L598
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L602:
	.align	2
.L601:
	.word	.LANCHOR2
	.word	-4014
	.word	-1676
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L609
	ldr	r2, .L609+4
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r1, [r2, r3]
	cmp	r1, #4
	add	ip, r1, #1
	mov	r1, r3
	strh	ip, [r2, r3]	@ movhi
	bhi	.L604
	cmp	r0, #0
	ldmeqfd	sp!, {r4, r5, pc}
.L604:
	ldr	r0, .L609+8
	mov	r3, #0
	strh	r3, [r2, r1]	@ movhi
	movw	r1, #4072
	str	r3, [r2, #-1672]
	movw	ip, #65535
	ldrh	r4, [r0, r1]
	ldr	r1, [r2, #-1844]
	ldr	r2, .L609+4
	b	.L606
.L608:
	ldrh	r0, [r1], #2
	add	r3, r3, #1
	cmp	r0, ip
	uxth	r3, r3
	ldrne	r5, [r2, #-1672]
	addne	r0, r0, r5
	strne	r0, [r2, #-1672]
.L606:
	cmp	r3, r4
	bne	.L608
	ldmfd	sp!, {r4, r5, pc}
.L610:
	.align	2
.L609:
	.word	-1674
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L618
	mov	r3, #0
	strh	r3, [r0, #4]	@ movhi
	mov	r3, #4064
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r3, [ip, r3]
	movw	ip, #65535
	b	.L612
.L614:
	add	r4, r0, r2, asl #1
	add	r2, r2, #1
	ldrh	r4, [r4, #16]
	uxth	r2, r2
	cmp	r4, ip
	ldrneh	r4, [r0, #4]
	addne	r4, r4, #1
	strneh	r4, [r0, #4]	@ movhi
.L612:
	cmp	r2, r3
	bcc	.L614
	ldr	r2, .L618+4
	movw	r4, #65535
	ldr	ip, .L618+8
	mvn	r1, r1
	ldrh	r5, [ip, r2]
	mov	ip, r0
	mov	r2, #0
	b	.L615
.L617:
	ldrh	r6, [ip, #16]
	add	r2, r2, #1
	add	ip, ip, #2
	cmp	r6, r4
	uxth	r2, r2
	ldrneh	r6, [r0, #4]
	addne	r6, r5, r6
	addne	r6, r6, r1
	strneh	r6, [r0, #4]	@ movhi
.L615:
	cmp	r2, r3
	bne	.L617
	ldmfd	sp!, {r4, r5, r6, pc}
.L619:
	.align	2
.L618:
	.word	.LANCHOR0
	.word	-4052
	.word	.LANCHOR2
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
	ldr	r6, .L625
	strh	r5, [r0, #4]	@ movhi
	mov	r8, #4064
	strb	r5, [r4, #7]
	mvn	r7, #0
	b	.L621
.L623:
	add	r3, r6, r5
	ldrh	r1, [r4, #0]
	add	sl, r5, #8
	add	r5, r5, #1
	ldrb	r0, [r3, #4090]	@ zero_extendqisi2
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
.L621:
	ldrh	r3, [r6, r8]
	cmp	r3, r5
	bhi	.L623
	ldr	r3, .L625+4
	ldr	r1, .L625+8
	ldrb	r2, [r4, #7]	@ zero_extendqisi2
	ldrh	r1, [r3, r1]
	mul	r2, r1, r2
	strh	r2, [r4, #4]	@ movhi
	mov	r2, #0
	strb	r2, [r4, #9]
	ldr	r2, [r3, #-1668]
	cmp	r2, #0
	beq	.L624
	ldrh	r1, [r4, #0]
	ldr	r2, [r3, #-1864]
	mov	r3, r1, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #59
	movls	r3, #1
	strlsb	r3, [r4, #9]
.L624:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L626:
	.align	2
.L625:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-4052
	.fnend
	.size	make_superblock, .-make_superblock
	.align	2
	.global	update_multiplier_value
	.type	update_multiplier_value, %function
update_multiplier_value:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r4, #0
	mov	r6, r0
	mov	r5, r4
	ldr	r7, .L632
	mov	r9, #4064
	ldr	sl, .L632+4
	ldr	r8, .L632+8
	b	.L628
.L630:
	add	r3, r7, r5
	mov	r1, r6
	add	r5, r5, #1
	ldrb	r0, [r3, #4090]	@ zero_extendqisi2
	bl	V2P_block
	uxth	r5, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [sl, r8]
	addeq	r4, r4, r3
	uxtheq	r4, r4
.L628:
	ldrh	r3, [r7, r9]
	cmp	r3, r5
	bhi	.L630
	cmp	r4, #0
	beq	.L631
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L631:
	ldr	r3, .L632+4
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #-1852]
	mla	r6, r2, r6, r3
	strh	r4, [r6, #4]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L633:
	.align	2
.L632:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-4052
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
	ldr	r3, .L637
	ldr	r0, [r3, #-1832]
	cmp	r0, #0
	bxeq	lr
	ldr	r2, [r3, #-1852]
	rsb	r0, r2, r0
	ldr	r2, .L637+4
	mov	r0, r0, asr #1
	mul	r0, r2, r0
	ldr	r2, [r3, #-1864]
	uxth	r0, r0
	mov	r3, r0, asl #1
	ldrh	r0, [r2, r3]
	bx	lr
.L638:
	.align	2
.L637:
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
	ldr	r2, .L646
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r3, [r2, #-1832]
	cmp	r3, #0
	beq	.L645
	ldr	r1, .L646+4
	mov	ip, #7
	mov	r5, #6
	movw	r4, #65535
	ldrh	r1, [r2, r1]
	mul	r1, ip, r1
	mov	r1, r1, asr #3
	cmp	r0, r1
	uxthgt	r0, r1
	ldr	r1, [r2, #-1852]
	ldr	r2, .L646+8
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
	b	.L642
.L644:
	mul	ip, r5, r3
	ldrh	ip, [r1, ip]
	cmp	ip, r4
	beq	.L643
	add	r2, r2, #1
	mov	r3, ip
	uxth	r2, r2
.L642:
	cmp	r2, r0
	bne	.L644
.L643:
	ldr	r2, .L646
	mov	r3, r3, asl #1
	ldr	r2, [r2, #-1864]
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L645:
	mov	r0, r3
	ldmfd	sp!, {r4, r5, pc}
.L647:
	.align	2
.L646:
	.word	.LANCHOR2
	.word	-1828
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
	ldr	r6, .L658
	add	r5, r8, #12
	.pad #32
	sub	sp, sp, #32
	ldr	r1, .L658+4
	bl	strcpy
	mov	r0, r5
	ldr	r1, .L658+8
	ldr	r2, [r6, #2260]
	bl	sprintf
	ldr	r4, .L658+12
	ldr	r1, .L658+16
	ldr	r2, [r4, #-4036]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+20
	ldr	r3, [r3, #2832]
	cmp	r3, #1
	add	r5, r5, r0
	bne	.L654
.L649:
	add	r0, sp, #16
	add	r1, sp, #20
	add	r2, sp, #24
	add	r3, sp, #28
	bl	NandcGetTimeCfg
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #16]
	mov	r0, r5
	ldr	r1, .L658+24
	str	r3, [sp, #0]
	ldr	r3, [sp, #28]
	ldr	r7, .L658+28
	str	r3, [sp, #4]
	ldr	r3, [sp, #20]
	bl	sprintf
	ldr	r1, .L658+32
	add	r5, r5, r0
	mov	r0, r5
	add	r5, r5, #10
	bl	strcpy
	ldr	r1, .L658+36
	mov	r0, r5
	ldr	r2, [r4, #-3992]
	bl	sprintf
	ldr	r1, .L658+40
	ldr	r2, [r4, #-1672]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+44
	ldr	r2, [r4, #-1664]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+48
	ldr	r2, [r4, #-1660]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+52
	ldr	r2, [r4, #-1656]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+56
	ldr	r2, [r4, #-1652]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+60
	ldr	r2, [r4, #-1648]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+64
	ldr	r2, [r4, #-1644]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #-1640]
	ldr	r1, .L658+68
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r4, #-1636]
	ldr	r1, .L658+72
	mov	r2, r2, lsr #11
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+76
	ldr	r2, [r4, #-1632]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+80
	ldr	r2, [r4, #-1628]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	FtlBbtCalcTotleCnt
	ldr	r2, .L658+84
	ldr	r1, .L658+88
	ldrh	r2, [r4, r2]
	mov	r3, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+92
	ldrh	r2, [r4, r7]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+96
	ldr	r2, [r4, #-1624]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+100
	ldr	r2, [r4, #-1620]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+104
	ldr	r2, [r4, #-1616]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+108
	ldr	r2, [r4, #-1860]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+112
	ldr	r2, [r4, #-1612]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+116
	ldr	r2, [r4, #-1608]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+120
	ldr	r1, .L658+124
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+128
	ldr	r1, .L658+132
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+136
	ldr	r2, [r4, #-4012]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+140
	ldr	r2, [r4, #-4020]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #4060]
	ldr	r1, .L658+144
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+148
	ldr	r1, .L658+152
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	movw	r3, #4072
	ldrh	r2, [r6, r3]
	ldr	r1, .L658+156
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+160
	ldr	r1, .L658+164
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r2, [r6, #4076]
	ldr	r1, .L658+168
	ldr	r6, .L658+172
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+176
	ldr	r1, .L658+180
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+184
	ldr	r1, .L658+188
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+192
	ldr	r1, .L658+196
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+200
	ldrb	r2, [r4, #-1818]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, r6]
	ldr	r1, .L658+204
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+208
	ldrb	r2, [r4, #-1816]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+212
	ldr	r1, .L658+216
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #-1844]
	add	r6, r6, #48
	ldr	r1, .L658+220
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+224
	ldr	r1, .L658+228
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+232
	ldrb	r2, [r4, #-1770]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r2, [r4, r6]
	ldr	r1, .L658+236
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+240
	ldrb	r2, [r4, #-1768]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+244
	ldr	r1, .L658+248
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #-1844]
	add	r6, r6, #236
	ldr	r1, .L658+252
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+256
	ldr	r1, .L658+260
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+264
	ldrb	r2, [r4, #-1722]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+268
	ldr	r1, .L658+272
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+276
	ldrb	r2, [r4, #-1720]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+280
	ldr	r1, .L658+284
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+288
	ldr	r1, .L658+292
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+296
	ldrb	r2, [r4, #-1534]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+300
	ldrh	r2, [r4, r6]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+304
	ldrb	r2, [r4, #-1532]	@ zero_extendqisi2
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+308
	ldr	r1, .L658+312
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, [r4, #-1404]
	ldr	r3, [r4, #-1668]
	ldr	r2, [r4, #-1492]
	str	r1, [sp, #0]
	ldr	r1, [r4, #-1412]
	orr	r2, r3, r2, asl #8
	str	r1, [sp, #4]
	ldr	r1, .L658+316
	ldr	r3, [r4, #-1408]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+320
	ldr	r2, [r4, #-1416]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+324
	ldr	r2, [r4, #-1392]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+328
	ldr	r1, .L658+332
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+336
	ldr	r1, .L658+340
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r1, .L658+344
	ldr	r2, [r4, #-972]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	ldr	r3, .L658+348
	ldr	r1, .L658+352
	ldrh	r2, [r4, r3]
	add	r5, r5, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L658+356
	mov	r2, r0
	mov	r0, r5
	bl	sprintf
	add	r5, r5, r0
	ldrh	r0, [r4, r7]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L658+360
	mov	r2, r0
	mov	r0, r5
	bl	sprintf
	ldrh	r3, [r4, r6]
	movw	r2, #65535
	cmp	r3, r2
	add	r5, r5, r0
	beq	.L651
	ldr	r2, [r4, #-1844]
	mov	r3, r3, asl #1
	mov	r0, r5
	ldr	r1, .L658+364
	ldrh	r2, [r2, r3]
	bl	sprintf
	add	r5, r5, r0
.L651:
	mov	r0, #0
	ldr	r4, .L658+12
	bl	List_get_gc_head_node
	mov	r6, #0
	movw	sl, #65535
	mov	r9, #6
	uxth	r3, r0
.L653:
	cmp	r3, sl
	beq	.L652
	ldr	r1, [r4, #-1844]
	mov	r2, r3, asl #1
	mul	r7, r9, r3
	mov	r0, r5
	ldrh	r1, [r1, r2]
	str	r1, [sp, #0]
	ldr	r1, [r4, #-1852]
	add	r1, r1, r7
	ldrh	r1, [r1, #4]
	str	r1, [sp, #4]
	ldr	r1, [r4, #-1864]
	ldrh	r2, [r1, r2]
	ldr	r1, .L658+368
	str	r2, [sp, #8]
	mov	r2, r6
	bl	sprintf
	add	r6, r6, #1
	ldr	r3, [r4, #-1852]
	cmp	r6, #16
	ldrh	r3, [r3, r7]
	add	r5, r5, r0
	bne	.L653
.L652:
	ldr	r6, .L658+12
	mov	r4, #0
	movw	sl, #65535
	mov	r9, #6
	ldr	r2, [r6, #-1832]
	ldr	r3, [r6, #-1852]
	rsb	r3, r3, r2
	ldr	r2, .L658+372
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L655:
	cmp	r3, sl
	beq	.L654
	mul	r7, r9, r3
	ldr	r2, [r6, #-1852]
	mov	r0, r5
	add	r2, r2, r7
	ldrh	r2, [r2, #4]
	str	r2, [sp, #0]
	mov	r2, r3, asl #1
	ldr	r1, [r6, #-1864]
	ldrh	r2, [r1, r2]
	ldr	r1, .L658+376
	str	r2, [sp, #4]
	mov	r2, r4
	bl	sprintf
	add	r4, r4, #1
	ldr	r3, [r6, #-1852]
	cmp	r4, #4
	ldrh	r3, [r3, r7]
	add	r5, r5, r0
	bne	.L655
.L654:
	rsb	r0, r8, r5
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L659:
	.align	2
.L658:
	.word	.LANCHOR0
	.word	.LC4
	.word	.LC5
	.word	.LANCHOR2
	.word	.LC6
	.word	.LANCHOR1
	.word	.LC7
	.word	-1828
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
	.word	-3978
	.word	.LC21
	.word	.LC22
	.word	.LC23
	.word	.LC24
	.word	.LC25
	.word	.LC26
	.word	.LC27
	.word	.LC28
	.word	-1574
	.word	.LC29
	.word	-1576
	.word	.LC30
	.word	.LC31
	.word	.LC32
	.word	.LC33
	.word	-3918
	.word	.LC34
	.word	.LC35
	.word	-1556
	.word	.LC36
	.word	.LC37
	.word	-1824
	.word	-1552
	.word	.LC38
	.word	-3984
	.word	.LC39
	.word	-1822
	.word	.LC40
	.word	.LC41
	.word	.LC42
	.word	.LC43
	.word	-1820
	.word	.LC44
	.word	.LC45
	.word	-1774
	.word	.LC46
	.word	.LC47
	.word	.LC48
	.word	.LC49
	.word	-1772
	.word	.LC50
	.word	.LC51
	.word	-1726
	.word	.LC52
	.word	.LC53
	.word	-1728
	.word	.LC54
	.word	.LC55
	.word	-1724
	.word	.LC56
	.word	-1538
	.word	.LC57
	.word	.LC58
	.word	.LC59
	.word	.LC60
	.word	-1536
	.word	.LC61
	.word	.LC62
	.word	.LC63
	.word	.LC64
	.word	-976
	.word	.LC65
	.word	-974
	.word	.LC66
	.word	.LC67
	.word	-968
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
	ldr	r1, .L661
	ldr	r2, .L661+4
	bl	sprintf
	add	r5, r4, r0
	mov	r0, r5
	bl	FtlPrintInfo2buf
	add	r0, r5, r0
	rsb	r0, r4, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L662:
	.align	2
.L661:
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
	ldr	r3, .L690
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r1, [r3, #-1616]
	ldr	r2, [r3, #-1608]
	cmp	r1, r2
	bcs	.L664
	ldr	r0, .L690+4
	movw	r1, #4072
	mov	r2, #0
	str	r2, [r3, #-1624]
	ldrh	r1, [r0, r1]
	ldr	r0, [r3, #-1864]
	b	.L665
.L666:
	ldrh	lr, [r0], #2
	add	r2, r2, #1
	ldr	ip, [r3, #-1624]
	add	ip, lr, ip
	str	ip, [r3, #-1624]
.L665:
	cmp	r2, r1
	ldr	r4, .L690
	bcc	.L666
	ldr	r5, [r4, #-1624]
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r3, .L690+8
	ldrh	r1, [r4, r3]
	str	r0, [r4, #-1616]
	ldr	r0, [r4, #-1620]
	rsb	r0, r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #-1624]
	b	.L667
.L664:
	ldr	r2, [r3, #-1612]
	cmp	r1, r2
	bls	.L667
	add	r2, r2, #1
	ldr	r4, .L690+4
	str	r2, [r3, #-1612]
	movw	ip, #4072
	mov	r2, #0
	b	.L668
.L669:
	ldr	r0, [r3, #-1864]
	mov	r1, r2, asl #1
	add	r2, r2, #1
	ldrh	r5, [r0, r1]
	add	r5, r5, #1
	strh	r5, [r0, r1]	@ movhi
.L668:
	ldrh	r1, [r4, ip]
	cmp	r2, r1
	bcc	.L669
.L667:
	ldr	r3, .L690
	ldr	r6, [r3, #-1608]
	ldr	r5, [r3, #-1616]
	add	r2, r6, #340
	add	r2, r2, #1
	cmp	r2, r5
	mov	r2, r3
	bls	.L670
	ldr	r1, [r3, #-1612]
	add	r0, r6, #1024
	cmp	r0, r1
	bls	.L670
	ldr	r3, [r3, #-1668]
	cmp	r3, #0
	beq	.L688
	cmp	r6, #30
	bhi	.L688
.L670:
	ldr	r3, .L690+12
	ldrh	r0, [r2, r3]
	add	r0, r0, r0, asl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	cmp	r0, r3
	mov	r8, r0
	bcs	.L672
	cmp	r6, #30
	bhi	.L688
.L672:
	ldr	r2, .L690
	ldr	r3, [r2, #-1848]
	cmp	r3, #0
	beq	.L688
	movw	r7, #65535
	ldr	r1, [r2, #-1852]
	mov	r4, r7
	ldr	r2, [r2, #-1864]
	mov	sl, r7
	ldr	ip, .L690+16
	mov	fp, #6
	b	.L673
.L676:
	ldrh	r9, [r3, #4]
	cmp	r9, #0
	beq	.L674
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	r9, r3, asl #1
	ldrh	r9, [r2, r9]
	cmp	r9, r6
	bls	.L685
	cmp	r9, r7
	movcc	r7, r9
	movcc	r4, r3
.L674:
	mla	r3, fp, r0, r1
.L673:
	ldrh	r0, [r3, #0]
	cmp	r0, sl
	bne	.L676
	b	.L675
.L685:
	mov	r4, r3
.L675:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L671
	mov	sl, r4, asl #1
	ldr	fp, .L690
	ldrh	r9, [r2, sl]
	cmp	r9, r6
	bls	.L677
	bl	GetFreeBlockMinEraseCount
	cmp	r0, r6
	strhi	r7, [fp, #-1608]
.L677:
	cmp	r9, #29
	ldr	r2, [fp, #-1668]
	movhi	r3, #0
	movls	r3, #1
	cmp	r2, #0
	moveq	r3, #0
	cmp	r3, #0
	beq	.L678
	add	r3, r9, #10
	cmp	r3, r5
	bls	.L679
	ldr	r3, .L690+8
	ldr	r2, .L690
	ldrh	r3, [r2, r3]
	cmp	r3, r9
	bls	.L678
.L679:
	ldr	r6, .L690
	ldr	r3, .L690+20
	ldrh	r3, [r6, r3]
	cmp	r3, #64
	bls	.L678
	ldr	r3, [r6, #-1864]
	mov	r1, r4
	ldr	r0, .L690+24
	ldrh	r2, [r3, sl]
	bl	printk
	b	.L689
.L678:
	cmp	r9, r5
	bcs	.L688
	add	r3, r9, #128
	cmp	r8, r3
	ble	.L688
	add	r3, r9, #340
	ldr	r6, .L690
	add	r3, r3, #1
	cmp	r3, r5
	bcc	.L680
	ldr	r3, [r6, #-1612]
	add	r9, r9, #1024
	cmp	r9, r3
	bcs	.L688
.L680:
	ldr	r3, [r6, #-1844]
	mov	r1, r4
	ldr	r0, .L690+28
	mov	r2, r5
	ldrh	r3, [r3, sl]
	str	r3, [sp, #0]
	ldr	r3, [r6, #-1864]
	ldrh	r3, [r3, sl]
	stmib	sp, {r3, r8}
	ldr	r3, [r6, #-1612]
	bl	printk
.L689:
	mov	r3, #1
	str	r3, [r6, #-964]
	b	.L671
.L688:
	movw	r4, #65535
.L671:
	mov	r0, r4
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L691:
	.align	2
.L690:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-4062
	.word	-1828
	.word	-1431655765
	.word	-1836
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
	beq	.L693
	ldr	r2, .L694
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #-1844]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L693:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L695:
	.align	2
.L694:
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
	ldr	r3, .L704
	mov	r2, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, #4064
	str	r2, [r3, #-960]
	mov	r7, #12
	ldr	sl, .L704+4
	mov	r6, #1
	ldr	r5, .L704+8
	ldr	fp, .L704+12
	b	.L697
.L698:
	mul	r1, r7, r2
	ldr	ip, [r3, #-956]
	add	r0, ip, r1
	str	r6, [r0, #8]
	ldrh	r0, [r3, r5]
	mul	r0, r0, r2
	add	r4, r0, #3
	cmp	r0, #0
	movlt	r0, r4
	ldr	r4, [r3, #-952]
	bic	r0, r0, #3
	add	r0, r4, r0
	str	r0, [ip, r1]
	ldrh	r0, [r3, fp]
	ldr	r9, [r3, #-956]
	mul	r0, r0, r2
	add	ip, r9, r1
	add	r4, r0, #3
	cmp	r0, #0
	movlt	r0, r4
	ldr	r4, [r3, #-948]
	bic	r0, r0, #3
	add	r0, r4, r0
	str	r0, [ip, #4]
	ldr	r0, [r3, #-944]
	mov	r4, #36
	ldr	r1, [r9, r1]
	mla	r0, r4, r2, r0
	add	r2, r2, #1
	uxth	r2, r2
	str	r1, [r0, #8]
	ldr	r1, [ip, #4]
	str	r1, [r0, #12]
.L697:
	ldrh	r1, [sl, r8]
	cmp	r2, r1
	bcc	.L698
	b	.L703
.L700:
	mul	r2, r7, r1
	ldr	ip, [r3, #-956]
	add	r0, ip, r2
	str	r6, [r0, #8]
	ldrh	r0, [r3, r5]
	mul	r0, r0, r1
	add	r8, r0, #3
	cmp	r0, #0
	movlt	r0, r8
	ldr	r8, [r3, #-952]
	bic	r0, r0, #3
	add	r0, r8, r0
	str	r0, [ip, r2]
	ldr	r0, [r3, #-956]
	add	r0, r0, r2
	ldrh	r2, [r3, r4]
	mul	r2, r2, r1
	add	r1, r1, #1
	uxth	r1, r1
	add	ip, r2, #3
	cmp	r2, #0
	movlt	r2, ip
	ldr	ip, [r3, #-948]
	bic	r2, r2, #3
	add	r2, ip, r2
	str	r2, [r0, #4]
	b	.L702
.L703:
	ldr	r3, .L704
	mov	r7, #12
	ldr	r5, .L704+8
	mov	r6, #0
	ldr	r4, .L704+12
.L702:
	ldr	r2, [r3, #-940]
	cmp	r1, r2
	bcc	.L700
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L705:
	.align	2
.L704:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-4042
	.word	-4040
	.fnend
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L713
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, #36
	ldr	r6, [r3, #-940]
	mov	fp, #12
	ldr	ip, [r3, #-956]
	mov	r3, #0
	mov	r4, r3
	b	.L707
.L710:
	mul	sl, fp, r2
	add	r8, ip, sl
	ldr	r9, [ip, sl]
	ldr	sl, [r7, #8]
	cmp	r9, sl
	streq	r4, [r8, #8]
	beq	.L709
.L708:
	add	r2, r2, #1
	uxth	r2, r2
.L712:
	cmp	r2, r6
	bcc	.L710
.L709:
	add	r3, r3, #1
	uxth	r3, r3
.L707:
	cmp	r3, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
	mla	r7, r5, r3, r0
	mov	r2, #0
	b	.L712
.L714:
	.align	2
.L713:
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
	ldr	r3, .L722
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #12
	ldr	r8, [r3, #-940]
	mov	r5, #1
	ldr	r7, [r3, #-956]
	mov	r4, #36
	mov	r3, #0
	b	.L716
.L719:
	mla	ip, r6, r2, r7
	ldr	sl, [ip, #8]
	cmp	sl, #0
	bne	.L717
	mla	r2, r4, r3, r0
	ldr	sl, [ip, #0]
	str	r5, [ip, #8]
	str	sl, [r2, #8]
	ldr	ip, [ip, #4]
	str	ip, [r2, #12]
	b	.L718
.L717:
	add	r2, r2, #1
	uxth	r2, r2
	b	.L720
.L721:
	mov	r2, #0
.L720:
	cmp	r2, r8
	bcc	.L719
.L718:
	add	r3, r3, #1
	uxth	r3, r3
.L716:
	cmp	r3, r1
	bcc	.L721
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L723:
	.align	2
.L722:
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
	ldr	r3, .L729
	ldr	r2, .L729+4
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #-932]
	mov	r3, #0
	b	.L725
.L727:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L728
	add	r3, r3, #1
	uxth	r3, r3
.L725:
	cmp	r3, r1
	bne	.L727
	mov	r0, #0
	bx	lr
.L728:
	mov	r0, #1
	bx	lr
.L730:
	.align	2
.L729:
	.word	.LANCHOR2
	.word	-936
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
	ldr	r3, .L735
	ldr	r2, .L735+4
	ldrh	ip, [r3, r2]
	ldr	r2, [r3, #-932]
	mov	r3, #0
	mov	r1, r2
	b	.L732
.L734:
	ldrh	r7, [r1], #2
	cmp	r7, r0
	beq	.L733
	add	r3, r3, #1
	uxth	r3, r3
.L732:
	cmp	r3, ip
	bne	.L734
	mov	r3, r3, asl #1
	strh	r0, [r2, r3]	@ movhi
	ldr	r2, .L735
	ldr	r3, .L735+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L733:
	ldr	r3, .L735
	mov	r0, #12
	ldr	r2, .L735+8
	ldrh	r1, [r3, r2]
	mul	r1, r0, r1
	ldr	r0, [r3, #-928]
	add	ip, r0, r1
	stmib	ip, {r5, r6}
	str	r4, [r0, r1]
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L736:
	.align	2
.L735:
	.word	.LANCHOR2
	.word	-936
	.word	-924
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	mov	r1, r4
	ldr	r0, .L741
	bl	printk
	ldr	r3, .L741+4
	ldr	r2, .L741+8
	movw	r1, #65535
	ldrh	r0, [r3, r2]
	cmp	r0, r1
	beq	.L740
.L738:
	ldr	r2, .L741+12
	ldrh	r0, [r3, r2]
	cmp	r0, r1
	bne	.L739
.L740:
	strh	r4, [r3, r2]	@ movhi
.L739:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L742:
	.align	2
.L741:
	.word	.LC78
	.word	.LANCHOR2
	.word	-922
	.word	-920
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
	ldr	r5, .L748
	mov	r2, r4
	ldr	r7, .L748+4
	ldrh	r1, [r5, r7]
	mov	r6, r0
	ldr	r0, .L748+8
	bl	printk
	mov	r0, r6
	bl	FtlGcRefreshBlock
	ldr	r3, [r5, #-1668]
	cmp	r3, #0
	beq	.L744
	ldr	r3, [r5, #-1864]
	mov	r6, r6, asl #1
	ldrh	r2, [r3, r6]
	cmp	r2, #29
	subhi	r2, r2, #30
	strhih	r2, [r3, r6]	@ movhi
.L744:
	ldrh	r1, [r5, r7]
	mov	r3, #0
	ldr	r2, .L748+12
	b	.L745
.L747:
	ldrh	r0, [r2, #2]!
	cmp	r0, r4
	beq	.L746
	add	r3, r3, #1
	uxth	r3, r3
.L745:
	cmp	r3, r1
	bne	.L747
	cmp	r3, #15
	bhi	.L746
	ldr	r2, .L748
	add	r1, r2, r3, asl #1
	add	r3, r3, #1
	sub	r1, r1, #916
	strh	r4, [r1, #0]	@ movhi
	ldr	r1, .L748+4
	strh	r3, [r2, r1]	@ movhi
.L746:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L749:
	.align	2
.L748:
	.word	.LANCHOR2
	.word	-918
	.word	.LC79
	.word	.LANCHOR2-918
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
	ldr	r2, .L753
	ldr	r3, .L753+4
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L751
	ldr	r1, .L753+8
	ldrh	r0, [r3, r1]
	movw	r1, #65535
	cmp	r0, r1
	bne	.L751
	ldr	r4, .L753+12
	ldr	r5, .L753+4
	ldrh	r1, [r3, r4]
	cmp	r1, r2
	movcs	r2, #0
	strcsh	r2, [r3, r4]	@ movhi
	ldrh	r3, [r5, r4]
	add	r3, r5, r3, asl #1
	sub	r3, r3, #916
	ldrh	r0, [r3, #0]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r5, r4]
	add	r3, r3, #1
	strh	r3, [r5, r4]	@ movhi
.L751:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L754:
	.align	2
.L753:
	.word	-918
	.word	.LANCHOR2
	.word	-922
	.word	-882
	.fnend
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	2
	.global	ftl_malloc
	.type	ftl_malloc, %function
ftl_malloc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r1, #209
	b	__kmalloc
	.fnend
	.size	ftl_malloc, .-ftl_malloc
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
	ldr	r3, .L768
	ldr	ip, .L768+4
	ldr	r3, [r3, #-880]
	str	r1, [ip, #2868]
	cmp	r3, r1
	beq	.L764
	str	r3, [ip, #2868]
	add	ip, ip, #16
	mov	r2, r1
	mov	lr, ip
.L759:
	add	r4, ip, r1
	tst	r2, #1
	mov	r5, r2, lsr #1
	add	r2, r2, #1
	str	r5, [r4, #4]
	moveq	r4, r0
	movne	r4, r3
	cmp	r2, #8
	str	r4, [lr, r1]
	add	r1, r1, #8
	bne	.L759
	b	.L760
.L764:
	add	ip, ip, #16
	mov	r2, r3
.L757:
	add	r1, ip, r3
	str	r2, [r1, #4]
	add	r2, r2, #1
	cmp	r2, #8
	str	r0, [r3, ip]
	add	r3, r3, #8
	bne	.L757
.L760:
	ldr	r2, .L768+4
	ldr	r4, .L768+4
	str	r0, [r2, #2224]
	ldr	r3, [r0, #0]
	ldr	r1, [r0, #344]
	and	r3, r3, #253952
	and	r1, r1, #7
	str	r1, [r2, #2872]
	cmp	r1, #2
	movhi	r1, #2
	strhi	r1, [r2, #2872]
	ldr	r2, [r2, #2872]
	cmp	r2, #0
	ldreq	r2, .L768+4
	moveq	r1, #1
	streq	r1, [r2, #2872]
	ubfx	r1, r3, #13, #1
	ldr	r2, .L768
	bfc	r3, #13, #1
	orr	r3, r3, #256
	str	r1, [r2, #-876]
	ldr	r2, [r0, #352]
	ubfx	r2, r2, #16, #4
	str	r2, [r4, #12]
	str	r3, [r0, #0]
	mov	r2, #0
	ldr	r3, [r4, #2224]
	mov	r0, #40
	str	r2, [r3, #336]
	bl	NandcTimeCfg
	ldr	r3, [r4, #2224]
	movw	r2, #8322
	str	r2, [r3, #344]
	ldr	r2, .L768+8
	str	r2, [r3, #304]
	mov	r2, #39
	str	r2, [r3, #308]
	ldr	r2, [r4, #2868]
	cmp	r2, #0
	beq	.L763
	ldr	r3, [r3, #0]
	str	r3, [r2, #0]
	ldr	r2, [r4, #2224]
	ldr	r3, [r4, #2868]
	ldr	r1, [r2, #336]
	str	r1, [r3, #336]
	ldr	r1, [r2, #4]
	str	r1, [r3, #4]
	ldr	r1, [r2, #304]
	str	r1, [r3, #304]
	ldr	r1, [r2, #308]
	str	r1, [r3, #308]
	ldr	r2, [r2, #344]
	str	r2, [r3, #344]
.L763:
	mov	r0, #36864
	bl	ftl_malloc
	ldr	r3, .L768+4
	ldr	r2, .L768
	mov	r1, #0
	str	r1, [r3, #4052]
	str	r1, [r2, #-868]
	str	r0, [r2, #-872]
	str	r0, [r3, #4028]
	add	r0, r0, #32768
	str	r0, [r3, #4032]
	ldmfd	sp!, {r3, r4, r5, pc}
.L769:
	.align	2
.L768:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	1579009
	.fnend
	.size	NandcInit, .-NandcInit
	.align	2
	.global	ftl_free
	.type	ftl_free, %function
ftl_free:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	kfree
	.fnend
	.size	ftl_free, .-ftl_free
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
	beq	.L772
	bl	memset
.L772:
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
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r3, #0
	ldr	r4, .L774
	mov	r1, #255
	ldr	r2, .L774+4
	ldr	r5, .L774+8
	ldr	r0, [r4, #-932]
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #12
	strh	r3, [r4, r2]	@ movhi
	ldrh	r2, [r4, r5]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, r5]
	mov	r2, #12
	ldr	r0, [r4, #-928]
	mov	r1, #255
	mul	r2, r2, r3
	bl	ftl_memset
	ldmfd	sp!, {r3, r4, r5, lr}
	b	FtlGcBufInit
.L775:
	.align	2
.L774:
	.word	.LANCHOR2
	.word	-936
	.word	-4048
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r3, #4074
	ldr	sl, .L785
	mov	r2, #6
	ldr	r4, .L785+4
	mov	r1, #0
	mov	r6, #0
	ldrh	r3, [sl, r3]
	mov	r8, r6
	ldr	r0, [r4, #-1852]
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r3, .L785+8
	str	r6, [r4, #-1832]
	str	r6, [r4, #-1848]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #8
	str	r6, [r4, #-1840]
	strh	r6, [r4, r3]	@ movhi
	mov	r4, r6
	b	.L777
.L779:
	add	r2, sl, r7
	mov	r1, r4
	add	r7, r7, #1
	ldrb	r0, [r2, #4090]	@ zero_extendqisi2
	str	r3, [sp, #4]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #4]
	uxth	r7, r7
	cmp	r0, #0
	ldreqh	r2, [r9, r3]
	addeq	r5, r5, r2
	uxtheq	r5, r5
	b	.L783
.L784:
	ldr	r9, .L785+4
	mov	r5, #0
	ldr	r3, .L785+12
	mov	r7, r5
	mov	fp, #4064
.L783:
	ldrh	r2, [sl, fp]
	cmp	r2, r7
	bhi	.L779
	cmp	r5, #0
	beq	.L780
	mov	r1, r5
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r5, r0
.L780:
	ldr	r3, .L785+4
	mov	r1, #6
	ldr	r2, [r3, #-1852]
	mla	r2, r1, r4, r2
	strh	r5, [r2, #4]	@ movhi
	ldr	r2, .L785+16
	ldrh	r2, [r3, r2]
	cmp	r2, r4
	beq	.L781
	ldr	r2, .L785+20
	ldrh	r2, [r3, r2]
	cmp	r2, r4
	beq	.L781
	ldr	r2, .L785+24
	ldrh	r2, [r3, r2]
	cmp	r2, r4
	beq	.L781
	ldr	r2, [r3, #-1844]
	mov	r3, r4, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L782
	add	r6, r6, #1
	mov	r0, r4
	uxth	r6, r6
	bl	INSERT_FREE_LIST
	b	.L781
.L782:
	add	r8, r8, #1
	mov	r0, r4
	uxth	r8, r8
	bl	INSERT_DATA_LIST
.L781:
	add	r4, r4, #1
	uxth	r4, r4
.L777:
	movw	r2, #4072
	ldrh	r3, [sl, r2]
	cmp	r3, r4
	bhi	.L784
	ldr	r3, .L785+4
	mov	r0, #0
	ldr	r2, .L785+8
	strh	r8, [r3, r2]	@ movhi
	add	r2, r2, #8
	strh	r6, [r3, r2]	@ movhi
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L786:
	.align	2
.L785:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1836
	.word	-4052
	.word	-1824
	.word	-1776
	.word	-1728
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
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
	ldr	r4, .L790
	mov	r7, #12
	ldr	r5, .L790+4
	ldr	r6, .L790+8
	ldr	r2, [r4, #-4024]
	ldr	r0, [r4, #-864]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, r5]
	ldrh	r2, [r4, r6]
	mov	r1, #255
	ldr	r0, [r4, #-860]
	mul	r2, r2, r3
	bl	ftl_memset
	mov	r2, #0
	mov	ip, r2
	mvn	r0, #0
	b	.L788
.L789:
	mul	r3, r7, r2
	ldr	r1, [r4, #-1680]
	add	r8, r1, r3
	str	ip, [r8, #4]
	strh	r0, [r1, r3]	@ movhi
	ldr	r1, [r4, #-1680]
	ldr	r8, [r4, #-860]
	add	r3, r1, r3
	ldrh	r1, [r4, r5]
	mul	r1, r2, r1
	add	r2, r2, #1
	uxth	r2, r2
	bic	r1, r1, #3
	add	r1, r8, r1
	str	r1, [r3, #8]
.L788:
	ldrh	r1, [r4, r6]
	ldr	r3, .L790
	cmp	r1, r2
	bhi	.L789
	ldr	r1, .L790+12
	mvn	r2, #0
	strh	r2, [r3, r1]	@ movhi
	ldr	r1, .L790+16
	strh	r2, [r3, r1]	@ movhi
	ldr	r1, [r3, #-4024]
	ldr	r2, .L790+20
	strh	r1, [r3, r2]	@ movhi
	ldr	r1, .L790+24
	ldr	r2, .L790+28
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #36
	ldrh	r1, [r3, r2]
	ldr	r2, .L790+32
	strh	r1, [r3, r2]	@ movhi
	sub	r2, r2, #3168
	ldrh	r1, [r3, r2]
	ldr	r2, .L790+36
	strh	r1, [r3, r2]	@ movhi
	ldr	r2, [r3, #-812]
	str	r2, [r3, #-844]
	ldr	r2, [r3, #-808]
	str	r2, [r3, #-840]
	ldr	r2, [r3, #-864]
	str	r2, [r3, #-836]
	ldr	r2, [r3, #-804]
	str	r2, [r3, #-832]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L791:
	.align	2
.L790:
	.word	.LANCHOR2
	.word	-4042
	.word	-4014
	.word	-854
	.word	-856
	.word	-846
	.word	-3902
	.word	-852
	.word	-848
	.word	-850
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
	b	.L793
.L797:
	ldr	r0, [r7, r3, asl #2]
	mov	r2, #0
	ubfx	r0, r0, #10, #16
	b	.L794
.L796:
	mov	r1, r2, asl #1
	add	r2, r2, #1
	ldrh	ip, [r6, r1]
	uxth	r2, r2
	cmp	ip, r0
	ldreqh	ip, [r5, r1]
	addeq	ip, ip, #1
	streqh	ip, [r5, r1]	@ movhi
.L794:
	ldrh	r1, [r4, #10]
	cmp	r1, r2
	bhi	.L796
	add	r3, r3, #1
	uxth	r3, r3
.L793:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L797
	mov	sl, #0
	ldrh	fp, [r5, #0]
	mov	r7, sl
	ldr	r3, .L803
	ldr	r2, .L803+4
	b	.L798
.L802:
	ldrh	r1, [r4, #0]
	cmp	r1, r7
	bne	.L799
	ldrh	r0, [r4, #2]
	ldrh	r1, [r3, r2]
	cmp	r0, r1
	movcc	r0, r7, asl #1
	strcch	r1, [r5, r0]	@ movhi
.L799:
	mov	r9, r7, asl #1
	ldrh	r8, [r5, r9]
	cmp	fp, r8
	movhi	sl, r7
	movhi	fp, r8
	cmp	r8, #0
	bne	.L801
	ldrh	r0, [r6, r9]
	cmp	r0, #0
	beq	.L801
	mov	r1, #1
	stmia	sp, {r2, r3}
	bl	FtlFreeSysBlkQueueIn
	strh	r8, [r6, r9]	@ movhi
	ldmia	sp, {r2, r3}
	ldrh	r1, [r4, #8]
	sub	r1, r1, #1
	strh	r1, [r4, #8]	@ movhi
.L801:
	add	r7, r7, #1
	uxth	r7, r7
.L798:
	ldrh	r1, [r4, #10]
	cmp	r1, r7
	bhi	.L802
	mov	r0, sl
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L804:
	.align	2
.L803:
	.word	.LANCHOR2
	.word	-4050
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L806
	ldr	r2, .L806+4
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, #0
	strh	r4, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, r4
	strh	r4, [r3, r2]	@ movhi
	add	r2, r2, #2
	strh	r4, [r3, r2]	@ movhi
	sub	r2, r2, #6
	strh	r0, [r3, r2]	@ movhi
	mov	r2, #2048
	ldr	r0, .L806+8
	bl	ftl_memset
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L807:
	.align	2
.L806:
	.word	.LANCHOR2
	.word	-3922
	.word	.LANCHOR2-3916
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
	ldr	r3, .L809
	mvn	r1, #0
	ldr	r2, .L809+4
	ldr	r0, .L809+8
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #6
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #255
	mov	r2, #16
	b	ftl_memset
.L810:
	.align	2
.L809:
	.word	.LANCHOR2
	.word	-3984
	.word	.LANCHOR2-3972
	.fnend
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L815
	ldr	r2, .L815+4
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
.L813:
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
	bne	.L813
	ldmfd	sp!, {r4, r5, r6, pc}
.L816:
	.align	2
.L815:
	.word	-800
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
	ldr	r4, .L818
	mov	r5, #0
	ldr	r2, .L818+4
	mov	r1, r5
	ldr	r7, .L818+8
	movw	r6, #4074
	str	r3, [r4, #-780]
	strh	r3, [r4, r2]	@ movhi
	ldr	r3, .L818+12
	ldr	r0, [r4, #-4004]
	str	r5, [r4, #-796]
	strh	r5, [r4, r3]	@ movhi
	ldr	r3, .L818+16
	str	r5, [r4, #-788]
	str	r5, [r4, #-784]
	ldrh	r2, [r4, r3]
	str	r5, [r4, #-1668]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r7, r6]
	mov	r1, r5
	ldr	r0, [r4, #-1864]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r7, r6]
	mov	r1, r5
	ldr	r0, [r4, #-776]
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r1, r5
	mov	r2, #48
	ldr	r0, .L818+20
	bl	ftl_memset
	mov	r1, r5
	mov	r2, #512
	sub	r0, r4, #1488
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L819:
	.align	2
.L818:
	.word	.LANCHOR2
	.word	-792
	.word	.LANCHOR0
	.word	-4008
	.word	-4032
	.word	.LANCHOR2-1604
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
	ldr	r4, .L851
	mov	r5, #0
	ldr	r2, .L851+4
	mov	r0, #1024
	mov	r7, #12
	mov	sl, #4064
	str	r3, [r4, #-764]
	mvn	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #2
	str	r5, [r4, #-772]
	mov	r8, #36
	strh	r3, [r4, r2]	@ movhi
	mov	r2, #32
	ldr	r3, .L851+8
	str	r5, [r4, #-1492]
	str	r5, [r4, #-1632]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	mov	r2, #128
	str	r5, [r4, #-1628]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #6
	str	r5, [r4, #-1644]
	strh	r5, [r4, r3]	@ movhi
	add	r3, r3, #50
	str	r5, [r4, #-1656]
	strh	r5, [r4, r3]	@ movhi
	add	r3, r3, #36
	str	r5, [r4, #-1660]
	strh	r5, [r4, r3]	@ movhi
	ldr	r3, .L851+12
	str	r5, [r4, #-1652]
	str	r5, [r4, #-1648]
	ldrh	r1, [r4, r3]
	str	r5, [r4, #-1664]
	str	r5, [r4, #-1624]
	str	r5, [r4, #-1620]
	str	r5, [r4, #-1860]
	str	r5, [r4, #-1612]
	str	r5, [r4, #-1608]
	str	r5, [r4, #-768]
	str	r5, [r4, #-964]
	str	r5, [r4, #-760]
	str	r5, [r4, #-972]
	str	r5, [r4, #-3988]
	bl	__aeabi_idiv
	ldr	r2, .L851+16
	mov	r3, #4064
	str	r5, [r4, #-4000]
	ldr	r5, .L851+20
	ldrh	r3, [r2, r3]
	mov	r3, r3, asl #2
	cmp	r0, r3
	str	r0, [r4, #-756]
	ldrh	r0, [r4, r5]
	strhi	r3, [r4, #-756]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-932]
	ldrh	r0, [r4, r5]
	ldr	r5, .L851+16
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r6, [r5, sl]
	mul	r6, r8, r6
	mov	r9, r6, asl #3
	str	r0, [r4, #-928]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-752]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-748]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-744]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-1868]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-944]
	ldr	r0, [r4, #-756]
	mul	r0, r8, r0
	bl	ftl_malloc
	ldr	r8, .L851+24
	ldrh	r3, [r5, sl]
	ldrh	r6, [r4, r8]
	mov	r3, r3, asl #1
	add	r3, r3, #1
	str	r3, [r4, #-940]
	str	r0, [r4, #-3996]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-1856]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-740]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-736]
	ldr	r0, [r4, #-940]
	mul	r0, r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-952]
	ldr	r0, [r4, #-756]
	mul	r0, r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-732]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-728]
	mov	r0, r6
	bl	ftl_malloc
	ldr	r6, .L851+28
	str	r0, [r4, #-724]
	ldr	r0, [r4, #-940]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, r6]
	ldrh	sl, [r5, sl]
	mul	sl, sl, r3
	str	r0, [r4, #-956]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-720]
	mov	r0, sl, asl #3
	bl	ftl_malloc
	ldrh	r3, [r4, r6]
	ldr	sl, .L851+32
	str	r0, [r4, #-716]
	ldr	r0, [r4, #-940]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r4, r6]
	movw	r6, #4074
	str	r0, [r4, #-948]
	ldr	r0, [r4, #-756]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #-712]
	ldrh	r0, [r5, r6]
	mov	r0, r0, asl #1
	uxth	r0, r0
	strh	r0, [r4, sl]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #-776]
	ldrh	r0, [r4, sl]
	add	r0, r0, #544
	add	r0, r0, #3
	mov	r0, r0, lsr #9
	strh	r0, [r4, sl]	@ movhi
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldrh	sl, [r5, r6]
	mov	sl, sl, asl #1
	str	r0, [r4, #-704]
	add	r0, r0, #32
	str	r0, [r4, #-1864]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-700]
	mov	r0, sl
	bl	ftl_malloc
	ldr	sl, [r4, #-4024]
	mov	sl, sl, asl #1
	str	r0, [r4, #-1844]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-812]
	mov	r0, sl
	bl	ftl_malloc
	ldr	sl, .L851+36
	str	r0, [r4, #-864]
	ldrh	r0, [r4, sl]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-4004]
	ldrh	r0, [r4, sl]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-696]
	ldrh	r0, [r4, sl]
	add	sl, sl, #2
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-692]
	ldrh	r0, [r4, sl]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r2, [r4, sl]
	mov	r1, #0
	add	sl, sl, #16
	mov	r2, r2, asl #2
	str	r0, [r4, #-688]
	bl	ftl_memset
	ldr	r3, .L851+40
	ldrh	r0, [r4, r3]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-804]
	ldr	r0, [r4, #-4024]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-808]
	ldrh	r0, [r4, sl]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, sl]
	movw	r7, #4086
	str	r0, [r4, #-1680]
	ldrh	r0, [r4, r8]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	ldr	r6, .L851+44
	str	r0, [r4, #-860]
	mov	r0, #6
	mul	r0, r0, r3
	bl	ftl_malloc
	ldr	r3, .L851+48
	ldrh	r2, [r5, r7]
	ldrh	r3, [r4, r3]
	add	r3, r3, #31
	mov	r3, r3, lsr #5
	strh	r3, [r4, r6]	@ movhi
	mul	r3, r2, r3
	str	r0, [r4, #-1852]
	mov	r0, r3, asl #2
	bl	ftl_malloc
	ldrh	r1, [r4, r6]
	ldrh	r5, [r5, r7]
	mov	r3, #1
	ldr	ip, .L851
	mov	r1, r1, asl #2
	mov	r2, r1
	str	r0, [r4, #-3956]
	ldr	r0, .L851+52
	b	.L822
.L823:
	ldr	r4, [ip, #-3956]
	add	r3, r3, #1
	add	r4, r4, r2
	add	r2, r2, r1
	str	r4, [r0, #4]!
.L822:
	cmp	r3, r5
	bcc	.L823
	ldr	r0, .L851+56
	mov	r2, #0
	mov	r1, r2
	add	r0, r0, r3, asl #2
	b	.L824
.L825:
	add	ip, r0, r2
	add	r3, r3, #1
	add	r2, r2, #4
	str	r1, [ip, #28]
.L824:
	cmp	r3, #7
	bls	.L825
	ldr	r3, .L851
	ldr	r2, [r3, #-812]
	cmp	r2, #0
	beq	.L850
.L826:
	ldr	r2, [r3, #-864]
	cmp	r2, #0
	beq	.L850
.L828:
	ldr	r2, [r3, #-804]
	cmp	r2, #0
	beq	.L850
.L829:
	ldr	r2, [r3, #-808]
	cmp	r2, #0
	beq	.L850
.L830:
	ldr	r2, [r3, #-1680]
	cmp	r2, #0
	beq	.L850
.L831:
	ldr	r2, [r3, #-860]
	cmp	r2, #0
	beq	.L850
.L832:
	ldr	r2, [r3, #-1852]
	cmp	r2, #0
	beq	.L850
.L833:
	ldr	r2, [r3, #-3956]
	cmp	r2, #0
	beq	.L850
.L834:
	ldr	r3, [r3, #-1844]
	cmp	r3, #0
	beq	.L850
.L835:
	ldr	r3, .L851
	ldr	r2, [r3, #-932]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-928]
	cmp	r2, #0
	beq	.L850
.L837:
	ldr	r2, [r3, #-752]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-744]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-1868]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-944]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-748]
	cmp	r2, #0
	beq	.L850
.L839:
	ldr	r2, [r3, #-1856]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-740]
	cmp	r2, #0
	beq	.L850
	ldr	r3, [r3, #-736]
	cmp	r3, #0
	beq	.L850
	ldr	r3, .L851
	ldr	r2, [r3, #-952]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-728]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-724]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-956]
	cmp	r2, #0
	beq	.L850
.L841:
	ldr	r2, [r3, #-720]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-716]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-948]
	cmp	r2, #0
	beq	.L850
.L843:
	ldr	r2, [r3, #-1864]
	cmp	r2, #0
	beq	.L850
	ldr	r2, [r3, #-776]
	cmp	r2, #0
	beq	.L850
.L845:
	ldr	r3, [r3, #-4004]
	cmp	r3, #0
	beq	.L850
.L846:
	ldr	r3, .L851
	ldr	r2, [r3, #-696]
	cmp	r2, #0
	beq	.L850
.L847:
	ldr	r2, [r3, #-692]
	cmp	r2, #0
	beq	.L850
.L848:
	ldr	r3, [r3, #-688]
	cmp	r3, #0
	bne	.L849
.L850:
	ldr	r0, .L851+60
	ldr	r1, .L851+64
	bl	printk
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L849:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L852:
	.align	2
.L851:
	.word	.LANCHOR2
	.word	-922
	.word	-976
	.word	-4046
	.word	.LANCHOR0
	.word	-4048
	.word	-4042
	.word	-4040
	.word	-708
	.word	-4032
	.word	-4016
	.word	-800
	.word	-4056
	.word	.LANCHOR2-3956
	.word	.LANCHOR2-3984
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
	ldr	r3, .L856
	ldr	r1, [r3, #2864]
	ldr	r0, [r3, #4]
	ldrb	r2, [r1, #9]	@ zero_extendqisi2
	mul	r2, r0, r2
	mov	r0, #0
	uxth	r2, r2
	strb	r2, [r4, #4]
	mov	ip, r2, lsr #8
	strb	ip, [r4, #5]
	ldrb	ip, [r3, #4024]	@ zero_extendqisi2
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
	ldr	r2, [r3, #2864]
	ldrb	ip, [r3, #4012]	@ zero_extendqisi2
	mov	r3, r0
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	strb	r1, [r4, #6]
	mov	r1, #32
	strb	r1, [r4, #8]
	ldrb	r2, [r2, #7]	@ zero_extendqisi2
	ldr	r1, .L856+4
	strb	r0, [r4, #10]
	strb	r2, [r4, #9]
	mov	r2, #1
	b	.L854
.L855:
	ldrb	r5, [r3, r1]	@ zero_extendqisi2
	add	r3, r3, #1
	ldrb	r0, [r4, #10]	@ zero_extendqisi2
	orr	r0, r0, r2, asl r5
	strb	r0, [r4, #10]
.L854:
	uxtb	r0, r3
	cmp	r0, ip
	bcc	.L855
	ldmfd	sp!, {r3, r4, r5, pc}
.L857:
	.align	2
.L856:
	.word	.LANCHOR0
	.word	.LANCHOR0+3996
	.fnend
	.size	ReadFlashInfo, .-ReadFlashInfo
	.align	2
	.global	FlashDieInfoInit
	.type	FlashDieInfoInit, %function
FlashDieInfoInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L870
	mov	r3, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	ldr	r4, .L870+4
	strb	r3, [r2, #-684]
	ldr	r2, .L870+8
	strb	r3, [r4, #4012]
	movw	r3, #2570
	ldrh	r3, [r2, r3]
	cmp	r3, #256
	str	r3, [r4, #4]
	movhi	r3, #512
	bhi	.L869
	cmp	r3, #128
	bls	.L860
	mov	r3, #256
.L869:
	str	r3, [r4, #4]
.L860:
	ldr	r6, .L870+12
	mov	r1, #0
	mov	r2, #16
	ldr	r0, .L870+16
	sub	r5, r6, #2160
	bl	ftl_memset
	add	sl, r5, #16
	mov	r1, #0
	mov	r2, #64
	ldr	r0, .L870+20
	bl	ftl_memset
	mov	r8, r6
	ldr	r0, .L870+24
	mov	r1, #0
	mov	r2, #256
	mov	r9, #1
	bl	ftl_memset
	mov	r7, #0
	strb	r7, [r4, #4026]
.L862:
	ldr	r4, [r5, #2864]
	mov	r1, r8
	add	r0, r4, #1
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L861
	ldrb	r3, [r5, #4012]	@ zero_extendqisi2
	add	r2, r5, r3, asl #2
	str	r0, [r2, #2800]
	add	r2, r5, r3
	add	r3, r3, #1
	strb	r3, [r5, #4012]
	ldr	r3, [r5, #2868]
	strb	r7, [r2, #3996]
	cmp	r3, #0
	beq	.L861
	ldr	r2, [sl, r7, asl #3]
	cmp	r2, r3
	streqb	r9, [r5, #4026]
.L861:
	add	r7, r7, #1
	add	r8, r8, #8
	cmp	r7, #8
	bne	.L862
	ldr	r5, .L870+4
	ldr	r3, .L870
	ldrb	r2, [r5, #4012]	@ zero_extendqisi2
	strb	r2, [r3, #-684]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #2
	bne	.L863
	add	sl, r4, #1
	mov	r7, #0
	mov	r8, r5
.L866:
	mov	r0, sl
	mov	r1, r6
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L864
	ldrb	r1, [r4, #13]	@ zero_extendqisi2
	ldr	r0, [r5, #4]
	ldrb	r3, [r5, #4012]	@ zero_extendqisi2
	mul	r0, r0, r1
	ldrh	r1, [r4, #14]
	add	r2, r5, r3, asl #2
	and	r1, r1, #65280
	mul	r1, r1, r0
	str	r1, [r2, #2800]
	ldrb	r0, [r4, #23]	@ zero_extendqisi2
	cmp	r0, #0
	movne	r1, r1, asl #1
	strne	r1, [r2, #2800]
	add	r2, r8, r3
	add	r3, r3, #1
	strb	r3, [r8, #4012]
	strb	r7, [r2, #3996]
.L864:
	add	r7, r7, #1
	add	r6, r6, #8
	cmp	r7, #8
	bne	.L866
.L863:
	ldr	r3, .L870+4
	ldrb	r1, [r4, #13]	@ zero_extendqisi2
	ldr	r2, .L870
	ldrb	r3, [r3, #4012]	@ zero_extendqisi2
	mul	r1, r1, r3
	ldrh	r3, [r4, #14]
	mul	r1, r3, r1
	ldr	r3, .L870+28
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L871:
	.align	2
.L870:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+2160
	.word	.LANCHOR0+3996
	.word	.LANCHOR0+2800
	.word	.LANCHOR0+2876
	.word	-682
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
	bne	.L873
	ldr	r3, .L907
.L874:
	strh	r0, [r3, #2]!	@ movhi
	add	r0, r0, #1
	uxth	r0, r0
	cmp	r0, #256
	bne	.L874
	b	.L875
.L873:
	cmp	r0, #1
	bne	.L876
	ldr	r0, .L907+4
	mov	r2, #0
	mov	r3, r2
.L879:
	uxth	r1, r3
	cmp	r1, #3
	bls	.L877
	tst	r3, #1
	moveq	r1, #2
	movne	r1, #3
	rsb	r1, r1, r2
	uxth	r1, r1
.L877:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L879
	b	.L875
.L876:
	cmp	r0, #2
	bne	.L880
	mov	r3, #0
	ldr	r0, .L907
	movw	r2, #65535
	mov	r1, r3
	b	.L906
.L883:
	cmp	r3, #1
	movls	r1, r3
	movhi	r1, r2
.L906:
	add	r3, r3, #1
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r3, r3
	uxth	r2, r2
	cmp	r3, #256
	bne	.L883
	b	.L875
.L880:
	cmp	r0, #3
	bne	.L884
	ldr	r0, .L907+4
	mov	r2, #0
	mov	r3, r2
.L887:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L885
	tst	r3, #1
	moveq	r1, #4
	movne	r1, #5
	rsb	r1, r1, r2
	uxth	r1, r1
.L885:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L887
	b	.L875
.L884:
	cmp	r0, #4
	mov	r2, #0
	bne	.L888
	ldr	r3, .L907+8
	mov	r1, #2288
	strh	r2, [r3, r1]	@ movhi
	movw	r2, #2290
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
	ldr	r0, .L907+12
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #7
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #8
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #16
	mov	r3, r1
.L890:
	tst	r3, #1
	add	r3, r3, #1
	moveq	r1, #6
	movne	r1, #7
	cmp	r3, #256
	rsb	r1, r1, r2
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L890
	b	.L875
.L888:
	cmp	r0, #6
	bne	.L875
	ldr	r0, .L907
	mov	r3, r2
.L893:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L891
	tst	r3, #1
	moveq	r1, #10
	movne	r1, #12
	rsb	r1, r1, r2
	uxth	r1, r1
.L891:
	add	r3, r3, #1
	add	r2, r2, #3
	cmp	r3, #256
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L893
.L875:
	mov	r2, #1024
	ldr	r0, .L907+16
	mov	r1, #255
	uxth	r4, r4
	bl	ftl_memset
	ldr	r2, .L907
	mov	r3, #0
	ldr	r0, .L907+20
	b	.L894
.L895:
	ldrh	r1, [r2, #2]!
	add	r3, r3, #1
	uxth	r3, r3
	add	ip, r0, r1, asl #1
	sub	ip, ip, #680
	strh	r1, [ip, #0]	@ movhi
.L894:
	cmp	r3, r4
	bcc	.L895
	ldmfd	sp!, {r4, pc}
.L908:
	.align	2
.L907:
	.word	.LANCHOR0+2286
	.word	.LANCHOR0+2288
	.word	.LANCHOR0
	.word	.LANCHOR0+2302
	.word	.LANCHOR2-680
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
	ldr	r4, .L914
	mov	ip, r0
	mov	r2, r3
	ldr	r4, [r4, #344]
	add	r4, r4, #4096
	add	r1, r4, r1
	movne	r0, r1
	movne	r1, ip
	ldmfd	sp!, {r4, lr}
	b	memcpy
.L915:
	.align	2
.L914:
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
	ldr	r3, .L921
	ldr	r0, [r3, #-4012]
	bx	lr
.L922:
	.align	2
.L921:
	.word	.LANCHOR2
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
	b	.L924
.L930:
	cmp	r8, #0
	bne	.L925
	ldr	r0, .L932
	mov	r1, r6
	mov	r2, r7
	bl	printk
.L925:
	cmp	r5, #4
	ldreq	r0, .L932+4
	ldreq	r1, [r4, r7, asl #2]
	beq	.L931
	cmp	r5, #2
	moveq	r3, r7, asl #1
	ldreq	r0, .L932+4
	ldreqsh	r1, [r4, r3]
	ldrne	r0, .L932+4
	ldrneb	r1, [r4, r7]	@ zero_extendqisi2
.L931:
	add	r8, r8, #1
	bl	printk
	cmp	r8, #15
	bls	.L929
	ldr	r0, .L932+8
	mov	r8, #0
	ldr	r1, .L932+12
	bl	printk
.L929:
	add	r7, r7, #1
.L924:
	cmp	r7, sl
	bne	.L930
	ldr	r0, .L932+8
	ldr	r1, .L932+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	b	printk
.L933:
	.align	2
.L932:
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
	ldr	r6, .L957
	add	r0, r6, r0, asl #3
	ldr	r3, [r6, #12]
	ldr	r4, [r0, #16]
	cmp	r3, #3
	bls	.L951
	ldr	r3, [r4, #16]
	tst	r3, #4
	beq	.L951
	mov	r0, r4
	bl	wait_for_nandc_xfer_completed
	ldr	r5, [r4, #16]
	ldr	r3, [r4, #8]
	ands	r5, r5, #2
	str	r3, [sp, #0]
	movne	r5, #0
	bne	.L955
	b	.L956
.L941:
	ldr	r3, [r6, #12]
	cmp	r3, #5
	bls	.L938
	ldr	r3, [r4, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L938
	ldr	r3, [sp, #4]
	tst	r3, #131072
	bne	.L939
.L938:
	add	r5, r5, #1
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L955
	ldr	r2, [r4, #28]
	mov	r1, r5
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L957+4
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L957+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L955:
	ldr	r2, [r4, #28]
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	blt	.L941
.L939:
	ldr	r4, .L957
	ldr	r3, [r4, #4052]
	cmp	r3, #0
	beq	.L942
	ldr	r1, [sp, #0]
	mov	r2, #0
	ldr	r0, [r4, #4044]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r1, [sp, #0]
	ldr	r0, [r4, #4048]
	mov	r2, #0
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
	b	.L942
.L945:
	ldr	r3, [r4, #8]
	add	r5, r5, #1
	str	r3, [sp, #0]
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L956
	ldr	r2, [sp, #0]
	mov	r1, r5
	ldr	r3, [r4, #28]
	ldr	r0, .L957+12
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L957+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L956:
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L945
	ldr	r3, .L957+16
	ldr	r3, [r3, #-868]
	cmp	r3, #0
	beq	.L946
	mov	r0, r4
	bl	NandcSendDumpDataStart
.L946:
	ldr	r5, .L957
	ldr	r3, [r5, #4052]
	cmp	r3, #0
	beq	.L947
	ldr	r1, [sp, #0]
	mov	r2, #1
	ldr	r0, [r5, #4044]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r1, [sp, #0]
	ldr	r0, [r5, #4048]
	mov	r2, #1
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
.L947:
	ldr	r3, .L957+16
	ldr	r3, [r3, #-868]
	cmp	r3, #0
	beq	.L942
	mov	r0, r4
	bl	NandcSendDumpDataDone
.L942:
	ldr	r3, .L957
	mov	r2, #0
	str	r2, [r3, #4052]
	b	.L934
.L951:
	ldr	r3, [r4, #8]
	str	r3, [sp, #0]
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L951
.L934:
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L958:
	.align	2
.L957:
	.word	.LANCHOR0
	.word	.LC84
	.word	.LC85
	.word	.LC86
	.word	.LANCHOR2
	.fnend
	.size	NandcXferComp, .-NandcXferComp
	.align	2
	.global	NandcXferData
	.type	NandcXferData, %function
NandcXferData:
	.fnstart
	@ args = 4, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L997
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r3
	.pad #100
	sub	sp, sp, #100
	mov	r9, r3
	tst	r4, #63
	add	r3, ip, r0, asl #3
	mov	r6, r0
	mov	fp, r1
	mov	sl, r2
	ldr	r7, [sp, #136]
	ldr	r5, [r3, #16]
	bne	.L960
	cmp	r7, #0
	bne	.L961
	add	r0, sp, #32
	mov	r1, #255
	mov	r2, #64
	add	r7, sp, #32
	bl	ftl_memset
.L961:
	mov	r0, r6
	mov	r1, fp
	mov	r2, sl
	mov	r3, #0
	stmia	sp, {r4, r7}
	bl	NandcXferStart
	mov	r0, r6
	bl	NandcXferComp
	cmp	fp, #0
	movne	r4, #0
	bne	.L962
	ldr	r1, .L997
	mov	r0, sl, lsr #1
	mov	r3, fp
	mov	r2, fp
	ldr	ip, [r1, #4020]
	mov	r4, r1
	cmp	ip, #24
	movhi	ip, #128
	movls	ip, #64
	b	.L964
.L965:
	ldr	r1, [r4, #4032]
	mov	r3, r3, lsr #2
	add	r2, r2, #1
	ldr	r3, [r1, r3, asl #2]
	mov	r1, r3, lsr #8
	strb	r3, [r7, #0]
	strb	r1, [r7, #1]
	mov	r1, r3, lsr #16
	mov	r3, r3, lsr #24
	strb	r1, [r7, #2]
	strb	r3, [r7, #3]
	add	r7, r7, #4
	mov	r3, r6
.L964:
	cmp	r2, r0
	add	r6, r3, ip
	ldr	r1, .L997
	bcc	.L965
	mov	r3, #0
	ldr	r0, [r1, #4020]
	mov	sl, sl, lsr #2
	ldr	r1, [r1, #12]
	mov	r4, r3
	b	.L966
.L972:
	add	r2, r3, #8
	ldr	r2, [r5, r2, asl #2]
	str	r2, [sp, #28]
	ldr	r2, [sp, #28]
	tst	r2, #4
	bne	.L988
	ldr	r2, [sp, #28]
	ands	r2, r2, #32768
	bne	.L988
	cmp	r1, #5
	bls	.L968
	ldr	r7, [sp, #28]
	ldr	ip, [sp, #28]
	ldr	r6, [sp, #28]
	ubfx	r7, r7, #3, #5
	ldr	r2, [sp, #28]
	ubfx	ip, ip, #27, #1
	ubfx	r6, r6, #16, #5
	ubfx	r2, r2, #29, #1
	orr	ip, r7, ip, asl #5
	orr	r2, r6, r2, asl #5
	cmp	ip, r2
	ldr	ip, [sp, #28]
	ldrhi	r2, [sp, #28]
	ubfxhi	ip, ip, #3, #5
	ldrls	r2, [sp, #28]
	ubfxls	ip, ip, #16, #5
	ubfxhi	r2, r2, #27, #1
	ubfxls	r2, r2, #29, #1
	b	.L996
.L968:
	cmp	r1, #3
	bls	.L970
	ldr	r7, [sp, #28]
	ldr	ip, [sp, #28]
	ldr	r6, [sp, #28]
	ubfx	r7, r7, #3, #5
	ldr	r2, [sp, #28]
	ubfx	ip, ip, #28, #1
	ubfx	r6, r6, #16, #5
	ubfx	r2, r2, #30, #1
	orr	ip, r7, ip, asl #5
	orr	r2, r6, r2, asl #5
	cmp	ip, r2
	ldr	ip, [sp, #28]
	ldrhi	r2, [sp, #28]
	ubfxhi	ip, ip, #3, #5
	ldrls	r2, [sp, #28]
	ubfxls	ip, ip, #16, #5
	ubfxhi	r2, r2, #28, #1
	ubfxls	r2, r2, #30, #1
.L996:
	orr	r2, ip, r2, asl #5
.L970:
	cmp	r4, r2
	movcc	r4, r2
	b	.L967
.L988:
	mvn	r4, #0
.L967:
	add	r3, r3, #1
.L966:
	cmp	r3, sl
	bcs	.L962
	cmp	r0, #0
	bne	.L972
.L962:
	mov	r3, #0
	str	r3, [r5, #16]
	b	.L973
.L960:
	cmp	r1, #1
	mov	r8, #0
	bne	.L994
	b	.L974
.L977:
	cmp	r7, #0
	and	r4, r8, #3
	mov	r0, r5
	mov	r1, #1
	movne	r3, #2
	moveq	r3, #0
	mla	r3, r8, r3, r7
	mov	r2, r4
	add	r8, r8, #2
	str	r3, [sp, #0]
	mov	r3, r9
	bl	NandcCopy1KB
	mov	r2, #0
	mov	r0, r6
	str	r2, [sp, #0]
	str	r2, [sp, #4]
	mov	r1, #1
	mov	r2, #2
	mov	r3, r4
	bl	NandcXferStart
	mov	r0, r6
	bl	NandcXferComp
	add	r9, r9, #1024
.L974:
	cmp	r8, sl
	bcc	.L977
	mov	r4, #0
	b	.L973
.L994:
	mov	r1, r8
	mov	r2, #2
	mov	r3, r8
	str	r8, [sp, #0]
	str	r8, [sp, #4]
	mov	r4, r8
	str	ip, [sp, #12]
	bl	NandcXferStart
	str	r8, [sp, #16]
	str	fp, [sp, #20]
	b	.L978
.L983:
	mov	r0, r6
	bl	NandcXferComp
	ldr	r3, [r5, #32]
	str	r3, [sp, #28]
	ldr	r3, [sp, #16]
	add	fp, r3, #2
	cmp	fp, sl
	bcs	.L979
	mov	r2, #0
	mov	r0, r6
	str	r2, [sp, #0]
	mov	r1, r2
	str	r2, [sp, #4]
	and	r3, fp, #3
	mov	r2, #2
	bl	NandcXferStart
.L979:
	ldr	r3, [sp, #28]
	tst	r3, #4
	mvnne	r4, #0
	bne	.L980
	ldr	r2, [sp, #28]
	ldr	r3, [sp, #28]
	ubfx	r2, r2, #3, #5
	ubfx	r3, r3, #27, #1
	orr	r3, r2, r3, asl #5
	cmp	r4, r3
	movcc	r4, r3
.L980:
	ldr	r2, .L997
	ldr	r3, [r2, #2868]
	cmp	r5, r3
	bne	.L981
	mov	r0, r6
	bl	NandcXferComp
.L981:
	cmp	r7, #0
	ldr	r2, [sp, #16]
	mov	r0, r5
	mov	r1, #0
	movne	r3, #2
	moveq	r3, #0
	mla	r3, r2, r3, r7
	and	r2, r8, #3
	add	r8, r8, #2
	str	r3, [sp, #0]
	mov	r3, r9
	add	r9, r9, #1024
	bl	NandcCopy1KB
	str	fp, [sp, #16]
.L978:
	ldr	r3, [sp, #16]
	cmp	r3, sl
	bcc	.L983
	ldr	fp, [sp, #20]
.L973:
	ldr	r3, .L997
	rsbs	fp, fp, #1
	movcc	fp, #0
	ldr	r3, [r3, #12]
	cmp	r3, #5
	movls	fp, #0
	cmp	fp, #0
	beq	.L984
	ldr	r3, [r5, #0]
	and	r2, r3, #139264
	cmp	r2, #139264
	orreq	r3, r3, #131072
	streq	r3, [r5, #0]
	mvneq	r4, #0
.L984:
	mov	r0, r4
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L998:
	.align	2
.L997:
	.word	.LANCHOR0
	.fnend
	.size	NandcXferData, .-NandcXferData
	.align	2
	.global	FlashProgPageDp
	.type	FlashProgPageDp, %function
FlashProgPageDp:
	.fnstart
	@ args = 8, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	mov	r8, r3
	ldr	r3, .L1000
	mov	r4, r0
	mov	r5, r1
	mov	r6, r2
	ldrb	r7, [r3, #2569]	@ zero_extendqisi2
	bl	NandcWaitFlashReady
	mov	r0, r4
	bl	NandcFlashCs
	mov	r0, r4
	mov	r1, r5
	bl	FlashProgFirstCmd
	mov	r2, r7
	mov	r3, r6
	mov	r0, r4
	ldr	r6, .L1000+4
	mov	r1, #1
	str	r8, [sp, #0]
	bl	NandcXferData
	mov	r1, r5
	mov	r0, r4
	bl	FlashProgDpFirstCmd
	mov	r0, r4
	bl	NandcWaitFlashReady
	ldr	r1, [r6, #4]
	mov	r0, r4
	add	r1, r5, r1
	bl	FlashProgDpSecondCmd
	ldr	r3, [sp, #36]
	mov	r2, r7
	mov	r0, r4
	mov	r1, #1
	str	r3, [sp, #0]
	ldr	r3, [sp, #32]
	bl	NandcXferData
	ldr	r1, [r6, #4]
	mov	r0, r4
	add	r1, r5, r1
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
.L1001:
	.align	2
.L1000:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashProgPageDp, .-FlashProgPageDp
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
	ldr	r3, .L1005
	subs	r4, r0, #0
	mov	r5, r1
	mov	r6, r2
	ldrb	r7, [r3, #2569]	@ zero_extendqisi2
	bne	.L1003
	ldr	r3, .L1005+4
	ldrb	r2, [r3, #1]	@ zero_extendqisi2
	ldr	r1, [r3, #4]
	mul	r2, r1, r2
	cmp	r5, r2
	bcs	.L1003
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r7, r7, #2
	movne	r7, #4
.L1003:
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
.L1006:
	.align	2
.L1005:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.fnend
	.size	FlashProgPage, .-FlashProgPage
	.align	2
	.global	FlashPageProgMsbFFData
	.type	FlashPageProgMsbFFData, %function
FlashPageProgMsbFFData:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r2
	ldr	r3, .L1013
	mov	r8, r0
	mov	r7, r1
	ldr	r3, [r3, #2864]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r2, r3, #5
	uxtb	r2, r2
	cmp	r3, #68
	cmpne	r2, #1
	bls	.L1010
	cmp	r3, #35
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	b	.L1010
.L1011:
	add	r3, r5, r4, asl #1
	sub	r3, r3, #680
	ldrh	r3, [r3, #0]
	cmp	r3, sl
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	mov	r1, #255
	mov	r2, #32768
	ldr	r0, [r5, #348]
	bl	ftl_memset
	add	r1, r4, r7
	add	r4, r4, #1
	mov	r0, r8
	ldr	r2, [r5, #348]
	mov	r3, #0
	bl	FlashProgPage
	uxtb	r4, r4
	b	.L1012
.L1010:
	ldr	r6, .L1013
	movw	sl, #65535
	ldr	r5, .L1013+4
.L1012:
	ldr	r3, [r6, #2864]
	ldrh	r3, [r3, #10]
	cmp	r3, r4
	bhi	.L1011
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1014:
	.align	2
.L1013:
	.word	.LANCHOR0
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
	ldr	r3, .L1018
	subs	r4, r0, #0
	mov	r6, r1
	mov	r5, r2
	ldrb	r7, [r3, #2569]	@ zero_extendqisi2
	bne	.L1016
	ldr	r3, .L1018+4
	ldrb	r2, [r3, #1]	@ zero_extendqisi2
	ldr	r3, [r3, #4]
	mul	r3, r3, r2
	cmp	r1, r3
	movcc	r7, #4
.L1016:
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
.L1019:
	.align	2
.L1018:
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
	ldr	fp, .L1031
	mov	r6, r0
	str	r1, [sp, #4]
	mov	sl, r2
	add	r3, fp, r0
	mov	r8, #0
	ldrb	r9, [fp, #3134]	@ zero_extendqisi2
	mvn	r4, #0
	ldrb	r5, [r3, #3144]	@ zero_extendqisi2
	bl	NandcWaitFlashReady
	mov	ip, fp
	b	.L1021
.L1026:
	add	r5, r5, #1
	ldrb	r1, [fp, #3133]	@ zero_extendqisi2
	mov	r0, r6
	ldr	r2, .L1031+4
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
	beq	.L1023
	ldrb	r3, [ip, #4024]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1030
	mov	r7, #0
	mov	sl, r7
.L1023:
	add	r8, r8, #1
.L1021:
	cmp	r8, r9
	bcc	.L1026
	b	.L1025
.L1030:
	mov	r4, r0
.L1025:
	ldr	r3, .L1031
	adds	r0, r4, #1
	add	r6, r3, r6
	movne	r0, #1
	ldrb	r3, [r3, #4024]	@ zero_extendqisi2
	strb	r5, [r6, #3144]
	add	r3, r3, r3, asl #1
	cmp	r4, r3, lsr #2
	movcc	r0, #0
	andcs	r0, r0, #1
	cmp	r0, #0
	moveq	r0, r4
	movne	r0, #256
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1032:
	.align	2
.L1031:
	.word	.LANCHOR0
	.word	.LANCHOR0+3136
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
	.pad #20
	sub	sp, sp, #20
	mov	fp, r2
	mov	r9, r3
	str	r1, [sp, #12]
	mov	sl, #0
	str	r0, [sp, #8]
	bl	NandcWaitFlashReady
	ldr	r1, [sp, #8]
	mvn	r5, #0
	ldr	r3, .L1042
	mov	ip, #239
	add	r2, r3, r1, asl #3
	ldrb	r7, [r2, #20]	@ zero_extendqisi2
	ldr	r6, [r2, #16]
	add	r7, r7, #8
	add	r4, r6, r7, asl #8
	mov	r8, r6
.L1037:
	str	ip, [r4, #8]
	mov	r2, #137
	mov	r0, #200
	str	r2, [r4, #4]
	mov	r6, #0
	str	ip, [sp, #4]
	add	sl, sl, #1
	bl	NandcDelayns
	str	sl, [r4, #0]
	mov	r2, fp
	str	r6, [r4, #0]
	mov	r3, r9
	str	r6, [r4, #0]
	ldr	r0, [sp, #8]
	str	r6, [r4, #0]
	ldr	r1, [sp, #12]
	bl	FlashReadRawPage
	ldr	ip, [sp, #4]
	cmn	r0, #1
	beq	.L1034
	ldr	r1, .L1042
	cmn	r5, #1
	moveq	r5, r0
	ldrb	r3, [r1, #4024]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1040
	mov	r9, r6
	mov	fp, r6
.L1034:
	cmp	sl, #7
	bne	.L1037
	mov	r6, r8
	b	.L1036
.L1040:
	mov	r6, r8
	mov	r5, r0
.L1036:
	mov	r3, #239
	mov	r0, #200
	str	r3, [r4, #8]
	mov	r3, #137
	str	r3, [r4, #4]
	bl	NandcDelayns
	mov	r3, #0
	str	r3, [r6, r7, asl #8]
	adds	r0, r5, #1
	str	r3, [r6, r7, asl #8]
	str	r3, [r6, r7, asl #8]
	movne	r0, #1
	str	r3, [r6, r7, asl #8]
	ldr	r3, .L1042
	ldrb	r3, [r3, #4024]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r5, r3, lsr #2
	movcc	r0, #0
	andcs	r0, r0, #1
	cmp	r0, #0
	moveq	r0, r5
	movne	r0, #256
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1043:
	.align	2
.L1042:
	.word	.LANCHOR0
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
	ldr	ip, .L1053
	mov	r5, #1
	mvn	r4, #0
	ldr	fp, .L1053+4
	add	r3, ip, r8, asl #3
	ldrb	sl, [r3, #20]	@ zero_extendqisi2
	ldr	r2, [r3, #16]
	add	sl, sl, #8
	add	sl, r2, sl, asl #8
	b	.L1045
.L1049:
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
	beq	.L1046
	ldrb	r3, [ip, #4024]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1052
	mov	r6, #0
	mov	r7, r6
.L1046:
	add	r5, r5, #1
.L1045:
	ldrb	r3, [fp, #352]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r5, r3
	bcc	.L1049
	b	.L1048
.L1052:
	mov	r4, r0
.L1048:
	mov	r0, sl
	mov	r1, #0
	bl	SamsungSetRRPara
	ldr	r3, .L1053
	adds	r0, r4, #1
	ldrb	r3, [r3, #4024]	@ zero_extendqisi2
	movne	r0, #1
	add	r3, r3, r3, asl #1
	cmp	r4, r3, lsr #2
	movcc	r0, #0
	andcs	r0, r0, #1
	cmp	r0, #0
	moveq	r0, r4
	movne	r0, #256
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1054:
	.align	2
.L1053:
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
	ldr	r3, .L1076
	add	r2, r3, r8, asl #3
	ldr	r1, [r2, #16]
	ldrb	r4, [r2, #20]	@ zero_extendqisi2
	ldrb	r2, [r3, #3993]	@ zero_extendqisi2
	add	r4, r4, #8
	sub	r2, r2, #67
	add	r4, r1, r4, asl #8
	uxtb	r2, r2
	cmp	r2, #1
	movls	sl, #0
	bls	.L1056
	ldrb	sl, [r3, #3995]	@ zero_extendqisi2
	cmp	sl, #0
	beq	.L1057
	mov	r0, #0
	mov	sl, #1
	bl	NandcSetDdrMode
.L1057:
	mov	r3, #92
	str	r3, [r4, #8]
	mov	r3, #197
	str	r3, [r4, #8]
.L1056:
	mov	r6, #1
	mvn	r5, #0
	ldr	fp, .L1076
	b	.L1058
.L1067:
	ldrb	r3, [fp, #3993]	@ zero_extendqisi2
	mov	r0, r4
	uxtb	r1, r6
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1059
	bl	SandiskSetRRPara
	b	.L1060
.L1059:
	bl	ToshibaSetRRPara
.L1060:
	ldr	r2, .L1076
	ldrb	r3, [r2, #3993]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L1061
	ldr	r2, .L1076+4
	ldrb	r3, [r2, #352]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r6, r3
	moveq	r3, #179
	streq	r3, [r4, #8]
.L1061:
	cmp	sl, #0
	mov	r3, #38
	str	r3, [r4, #8]
	mov	r3, #93
	str	r3, [r4, #8]
	beq	.L1062
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
	b	.L1063
.L1062:
	mov	r3, r7
	mov	r0, r8
	ldr	r1, [sp, #4]
	mov	r2, r9
	bl	FlashReadRawPage
	mov	r3, r0
.L1063:
	cmn	r3, #1
	beq	.L1064
	ldrb	r2, [fp, #4024]	@ zero_extendqisi2
	cmn	r5, #1
	moveq	r5, r3
	add	r2, r2, r2, asl #1
	cmp	r3, r2, lsr #2
	bcc	.L1075
	mov	r7, #0
	mov	r9, r7
.L1064:
	add	r6, r6, #1
.L1058:
	ldr	r2, .L1076+4
	ldrb	r3, [r2, #352]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r6, r3
	bcc	.L1067
	b	.L1066
.L1075:
	mov	r5, r3
.L1066:
	ldr	r6, .L1076
	mov	r0, r4
	mov	r1, #0
	ldrb	r3, [r6, #3993]	@ zero_extendqisi2
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1068
	bl	SandiskSetRRPara
	b	.L1069
.L1068:
	bl	ToshibaSetRRPara
.L1069:
	mov	r3, #255
	str	r3, [r4, #8]
	ldrb	r3, [r6, #4024]	@ zero_extendqisi2
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
	beq	.L1071
	mov	r0, #4
	bl	NandcSetDdrMode
.L1071:
	mov	r0, r5
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1077:
	.align	2
.L1076:
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
	ldr	r5, .L1085
	ldr	r7, .L1085+4
	ldr	r8, .L1085+8
	ldr	r3, [r5, #4016]
	ldrb	r0, [r7, #360]	@ zero_extendqisi2
	str	r3, [r7, #356]
	bl	FlashBchSel
	mov	r1, #0
	mov	r2, #2048
	ldr	r0, [r5, #4016]
	bl	ftl_memset
	ldr	r3, [r7, #356]
	add	r1, r5, #2160
	mov	r2, #64
	str	r8, [r3, #0]
	ldr	r4, [r7, #356]
	ldrb	r3, [r5, #4012]	@ zero_extendqisi2
	add	r0, r4, #16
	strh	r3, [r4, #12]	@ movhi
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	strh	r3, [r4, #14]	@ movhi
	ldrb	r3, [r5, #3995]	@ zero_extendqisi2
	str	r3, [r4, #1076]
	bl	memcpy
	add	r0, r4, #80
	ldr	r1, .L1085+12
	mov	r2, #16
	bl	memcpy
	add	r0, r4, #96
	add	r1, r5, #2800
	mov	r2, #64
	bl	memcpy
	ldr	r0, [r7, #356]
	ldr	r1, .L1085+16
	mov	r2, #32
	add	r0, r0, #160
	bl	memcpy
	ldr	r4, [r7, #356]
	add	r1, r5, #80
	mov	r2, #32
	add	r0, r4, #192
	bl	memcpy
	add	r0, r4, #224
	ldr	r1, .L1085+20
	mov	r2, #852
	bl	memcpy
	add	r0, r4, #12
	movw	r1, #2036
	bl	JSHash
	movw	r3, #1592
	str	r3, [r4, #4]
	ldr	r3, [r7, #348]
	str	r3, [r7, #356]
	str	r0, [r4, #8]
	mov	r4, r6
.L1081:
	ldr	r1, [r5, #4]
	mov	r0, #0
	mov	r2, r0
	mul	r1, r1, r4
	bl	FlashEraseBlock
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r5, #4016]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r5, #4016]
	mov	r3, r0
	mul	r1, r1, r4
	add	r1, r1, #1
	bl	FlashProgPage
	ldr	r1, [r5, #4]
	mov	r0, #0
	ldr	r2, [r7, #348]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	beq	.L1079
	ldr	sl, [r7, #356]
	ldr	r3, [sl, #0]
	cmp	r3, r8
	bne	.L1079
	add	r0, sl, #12
	movw	r1, #2036
	bl	JSHash
	ldr	r3, [sl, #8]
	cmp	r3, r0
	bne	.L1079
	add	r3, r4, #1
	str	r3, [r7, #364]
	ldr	r3, [r5, #4]
	add	r6, r6, #1
	cmp	r6, #1
	mul	r3, r3, r4
	str	r3, [r7, #368]
	bhi	.L1082
.L1079:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L1081
	cmp	r6, #0
	mvneq	r0, #0
	movne	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1082:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1086:
	.align	2
.L1085:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	1312902724
	.word	.LANCHOR0+3996
	.word	.LANCHOR1+2560
	.word	.LANCHOR0+3132
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
	mov	sl, r0
	ldr	r7, .L1096
	mov	r2, #4
	ldr	r1, .L1096+4
	add	r0, sp, #4
	bl	memcpy
	mov	r0, sl
	mov	r1, #0
	mov	r2, #2048
	ldrb	fp, [r7, #4024]	@ zero_extendqisi2
	mvn	r8, #0
	bl	ftl_memset
	mov	r4, #2
	mov	r6, r7
	ldr	r9, .L1096+8
	b	.L1088
.L1094:
	mov	r5, #0
.L1090:
	add	r3, sp, #4
	ldrb	r0, [r3, r5]	@ zero_extendqisi2
	bl	FlashBchSel
	ldr	r1, [r7, #4]
	mov	r0, #0
	ldr	r2, [r7, #4016]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1089
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L1090
	b	.L1091
.L1089:
	ldr	r3, [r6, #4016]
	ldr	r3, [r3, #0]
	cmp	r3, r9
	bne	.L1091
	add	r3, sp, #8
	ldr	r0, .L1096+12
	add	r5, r3, r5
	ldrb	r1, [r5, #-4]	@ zero_extendqisi2
	bl	printk
	mov	r2, #2048
	mov	r0, sl
	ldr	r1, [r6, #4016]
	bl	memcpy
	ldr	r3, [r6, #4016]
	ldr	r3, [r3, #512]
	strb	r3, [r6, #1]
	ldr	r3, .L1096+16
	ldr	r2, [r3, #364]
	cmp	r2, r4
	bls	.L1093
	str	r4, [r3, #364]
	mov	r8, #0
	bl	FlashSavePhyInfo
.L1091:
	add	r4, r4, #1
.L1088:
	ldrb	r3, [r7, #1]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1094
	b	.L1092
.L1093:
	mov	r8, #0
.L1092:
	mov	r0, fp
	bl	FlashBchSel
	mov	r0, r8
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1097:
	.align	2
.L1096:
	.word	.LANCHOR0
	.word	.LANCHOR3+11
	.word	-52655045
	.word	.LC87
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
	add	r0, sp, #4
	ldr	r1, .L1107
	bl	memcpy
	ldr	r7, .L1107+4
	ldr	r2, .L1107+8
	movw	r3, #2570
	ldr	r5, .L1107+12
	mov	r4, #0
	mov	sl, #4
	mvn	r8, #0
	ldrh	fp, [r2, r3]
	ldr	r3, [r7, #4016]
	str	r4, [r5, #368]
	str	r3, [r5, #356]
	b	.L1099
.L1101:
	add	r3, sp, #4
	ldrb	r0, [r3, r6]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	mov	r1, r4
	ldr	r2, [r7, #4016]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1100
	mov	r0, #0
	mov	r1, r9
	ldr	r2, [r7, #4016]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1100
	add	r6, r6, #1
	cmp	r6, #4
	beq	.L1102
	b	.L1101
.L1100:
	ldr	r6, [r5, #356]
	ldr	r3, .L1107+16
	ldr	r2, [r6, #0]
	cmp	r2, r3
	bne	.L1102
	cmp	r8, #0
	bne	.L1103
	ldr	r2, .L1107+8
	movw	r3, #2570
	mov	r0, r4
	ldrh	r1, [r2, r3]
	bl	__aeabi_uidiv
	ldr	r3, .L1107+12
	add	r0, r0, #1
	str	r0, [r3, #364]
	b	.L1104
.L1103:
	add	r0, r6, #12
	movw	r1, #2036
	bl	JSHash
	ldr	r3, [r6, #8]
	cmp	r3, r0
	bne	.L1102
	ldr	r8, .L1107+8
	add	r1, r6, #160
	mov	r2, #32
	add	r0, r8, #2560
	bl	memcpy
	add	r1, r6, #192
	mov	r2, #32
	ldr	r0, .L1107+20
	bl	memcpy
	add	r1, r6, #224
	mov	r2, #852
	ldr	r0, .L1107+24
	bl	memcpy
	ldr	r3, [r6, #1076]
	mov	r0, r4
	str	r4, [r5, #368]
	strb	r3, [r7, #3995]
	movw	r3, #2570
	ldrh	r1, [r8, r3]
	mov	r8, #0
	bl	__aeabi_uidiv
	add	r0, r0, #1
	str	r0, [r5, #364]
	cmp	r0, #1
	moveq	r3, #2
	streq	r3, [r5, #364]
	ldrh	r3, [r6, #14]
	strb	r3, [r5, #372]
.L1102:
	subs	sl, sl, #1
	add	r4, r4, fp
	beq	.L1104
.L1099:
	add	r9, r4, #1
	mov	r6, #0
	b	.L1101
.L1104:
	mov	r0, r8
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1108:
	.align	2
.L1107:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	1312902724
	.word	.LANCHOR0+80
	.word	.LANCHOR0+3132
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
	ldr	r4, .L1130
	.pad #20
	sub	sp, sp, #20
	mov	fp, r0
	mov	r9, r2
	str	r1, [sp, #0]
	ldr	r3, [r4, #2224]
	ldr	r7, [r4, #12]
	ldr	r3, [r3, #304]
	cmp	r7, #7
	movhi	r7, #12
	movls	r7, #6
	str	r3, [sp, #12]
	ldr	r3, [sp, #56]
	cmp	r3, #0
	moveq	r8, #1024
	beq	.L1111
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
	ldrb	r0, [r4, #3994]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3994]	@ zero_extendqisi2
	bl	NandcSetMode
	cmn	r8, #1
	beq	.L1112
	mov	r2, r8
	ldr	r0, .L1130+4
	ldr	r1, [sp, #0]
	add	r4, r4, fp, asl #3
	bl	printk
	ldrb	r3, [r4, #2160]	@ zero_extendqisi2
	cmp	r8, #9
	cmpls	r3, #173
	ldreq	r3, [r4, #16]
	ldreq	r2, [r3, #3840]
	ldreq	r2, [r3, #0]
	orreq	r2, r2, #131072
	streq	r2, [r3, #0]
	ldr	r3, .L1130+8
	ldr	r2, [r3, #376]
	add	r2, r2, #1
	str	r2, [r3, #376]
	cmp	r2, #2048
	movcs	sl, #0
	strcs	sl, [r3, #376]
	movcs	r9, sl
	bcc	.L1112
.L1111:
	mov	r4, #0
	str	fp, [sp, #4]
	mov	r5, r4
	mvn	r6, #0
	mov	fp, r4
	str	r4, [sp, #8]
.L1118:
	uxtb	r0, r7
	bl	NandcSetDdrPara
	mov	r3, sl
	ldr	r0, [sp, #4]
	mov	r2, r9
	ldr	r1, [sp, #0]
	bl	FlashReadRawPage
	add	r3, r8, #1
	cmp	r0, r3
	bhi	.L1114
	cmp	r0, #2
	bhi	.L1124
	add	r5, r5, #1
	cmp	r5, #9
	bls	.L1124
	mov	ip, fp
	rsb	r4, r5, r7
	ldr	fp, [sp, #4]
	mov	r8, r0
	mov	r6, #0
	b	.L1116
.L1114:
	cmp	fp, r5
	bcs	.L1125
	cmp	r5, #7
	rsb	r3, r5, r4
	str	r3, [sp, #8]
	bhi	.L1129
	mov	fp, r5
	b	.L1125
.L1124:
	mov	r6, #0
	mov	r4, r7
	mov	r8, r0
	mov	sl, r6
	mov	r9, r6
	b	.L1115
.L1125:
	mov	r5, #0
.L1115:
	add	r7, r7, #2
	cmp	r7, #69
	bls	.L1118
	mov	ip, fp
	ldr	fp, [sp, #4]
.L1116:
	cmp	ip, r5
	bcc	.L1119
	b	.L1117
.L1129:
	ldr	fp, [sp, #4]
.L1117:
	ldr	r4, [sp, #8]
.L1119:
	cmp	r4, #0
	beq	.L1120
	ldr	r0, .L1130+12
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	bl	NandcSetDdrPara
.L1120:
	cmn	r6, #1
	bne	.L1112
	ldr	r0, .L1130+16
	mov	r1, fp
	ldr	r2, [sp, #0]
	bl	printk
	ldr	r3, [sp, #56]
	cmp	r3, #0
	moveq	r8, r6
	beq	.L1112
	ldr	r3, [sp, #12]
	ubfx	r0, r3, #8, #8
	bl	NandcSetDdrPara
.L1112:
	mov	r0, r8
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1131:
	.align	2
.L1130:
	.word	.LANCHOR0
	.word	.LC88
	.word	.LANCHOR2
	.word	.LC89
	.word	.LC90
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
	ldr	r5, .L1136
	mov	r4, #0
	mov	r6, r1
	ldrb	r0, [r5, #3994]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r5, #3994]	@ zero_extendqisi2
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
	beq	.L1133
	cmn	r8, #1
	bne	.L1134
.L1133:
	ldrb	r3, [r5, #3994]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L1134
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r3, .L1136
	mov	r2, #0
	strb	r2, [r3, #3995]
	b	.L1135
.L1134:
	mov	r3, #1
	strb	r3, [r5, #3995]
.L1135:
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1137:
	.align	2
.L1136:
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
	ldr	r6, .L1182
	bl	ftl_malloc
	ldr	r7, .L1182+4
	mov	r4, #0
	mov	r5, r6
	str	r0, [r6, #4016]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r7, #348]
	mov	r0, #4096
	bl	ftl_malloc
	str	r0, [r7, #380]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r7, #384]
	mov	r0, #4096
	bl	ftl_malloc
	mov	r3, #50
	str	r4, [r7, #364]
	strb	r3, [r6, #1]
	strb	r3, [r7, #372]
	mov	r3, #128
	strb	r4, [r6, #3995]
	str	r3, [r6, #4]
	mov	r3, #60
	str	r4, [r7, #376]
	strb	r4, [r5], #2160
	strb	r3, [r7, #360]
	str	r0, [r7, #388]
	mov	r0, r8
	bl	NandcInit
.L1141:
	uxtb	r0, r4
	mov	r1, r5
	bl	FlashReadIDRaw
	cmp	r4, #0
	bne	.L1139
	ldrb	r3, [r6, #2160]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L1174
	ldrb	r3, [r6, #2161]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L1175
.L1139:
	add	r4, r4, #1
	add	r5, r5, #8
	cmp	r4, #8
	bne	.L1141
	ldr	r4, .L1182
	ldrb	r3, [r4, #2160]	@ zero_extendqisi2
	cmp	r3, #173
	beq	.L1142
	ldr	r3, .L1182+4
	ldr	r0, [r3, #-876]
	bl	NandcSetDdrMode
.L1142:
	mov	r1, #0
	ldr	r0, .L1182+8
	mov	r2, #852
	bl	ftl_memset
	ldr	r3, .L1182+12
	str	r3, [r4, #2864]
	mov	r3, #0
	strb	r3, [r4, #8]
	ldrb	r3, [r4, #2161]	@ zero_extendqisi2
	sub	r1, r3, #218
	rsbs	r0, r1, #0
	adc	r0, r0, r1
	cmp	r3, #218
	cmpne	r3, #241
	beq	.L1143
	cmp	r3, #220
	bne	.L1144
	ldr	r3, .L1182
	ldrb	r3, [r3, #2163]	@ zero_extendqisi2
	cmp	r3, #149
	bne	.L1144
.L1143:
	ldr	r3, .L1182
	mov	r1, #16
	mov	r2, #1
	strb	r1, [r3, #1]
	strb	r2, [r3, #0]
	ldrb	r3, [r3, #2160]	@ zero_extendqisi2
	ldr	r2, .L1182+4
	cmp	r3, #152
	strb	r1, [r2, #360]
	moveq	r3, #24
	streqb	r3, [r2, #360]
	cmp	r0, #0
	ldr	r3, .L1182+16
	movw	r2, #2850
	movne	r1, #2048
	moveq	r1, #4096
	ldr	r0, .L1182+20
	strneh	r1, [r3, r2]	@ movhi
	mvnne	r2, #37
	streqh	r1, [r3, r2]	@ movhi
	mvneq	r2, #35
	ldr	r1, .L1182+24
	strb	r2, [r3, #2838]
	mov	r2, #32
	bl	memcpy
	ldr	r0, .L1182+12
	mov	r2, #32
	add	r1, r0, #276
	bl	memcpy
.L1144:
	ldr	r4, .L1182
	ldrb	r3, [r4, #0]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1148
	bl	FlashLoadPhyInfoInRam
	cmp	r0, #0
	bne	.L1149
	ldr	r3, [r4, #2864]
	ldrh	r0, [r3, #16]
	ubfx	r0, r0, #8, #3
	strb	r0, [r4, #3994]
	tst	r0, #1
	bne	.L1149
	mov	r3, #1
	strb	r3, [r4, #3995]
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3994]	@ zero_extendqisi2
	bl	NandcSetMode
.L1149:
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1148
	ldr	r3, .L1182+4
	ldr	r3, [r3, #-876]
	cmp	r3, #0
	beq	.L1151
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	b	.L1179
.L1151:
	ldr	r4, .L1182
	ldrb	r0, [r4, #3994]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3994]	@ zero_extendqisi2
.L1179:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L1148
	ldr	r4, .L1182
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r0, .L1182+28
	ldr	r3, [r4, #2864]
	ldrh	r1, [r3, #14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	cmn	r0, #1
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, pc}
	bl	FlashDieInfoInit
	ldr	r3, [r4, #2864]
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	bl	FlashGetReadRetryDefault
	ldr	r1, .L1182+32
	ldr	r0, .L1182+4
	ldr	r3, [r4, #2864]
	ldrh	r1, [r0, r1]
	ldrb	r2, [r3, #9]	@ zero_extendqisi2
	add	r1, r1, #4080
	add	r1, r1, #15
	cmp	r2, r1, lsr #12
	blt	.L1154
	ldrh	r1, [r3, #14]
	add	r1, r1, #255
	cmp	r2, r1, lsr #8
	bge	.L1155
.L1154:
	ldrh	r2, [r3, #14]
	bic	r2, r2, #255
	strh	r2, [r3, #14]	@ movhi
.L1155:
	ldr	r3, .L1182
	ldrb	r2, [r3, #3994]	@ zero_extendqisi2
	tst	r2, #6
	beq	.L1156
	ldr	r3, [r3, #12]
	ldr	r4, .L1182+4
	cmp	r3, #7
	movls	r3, #40
	strlsb	r3, [r4, #360]
	bl	FlashSavePhyInfo
	mov	r0, #0
	ldr	r1, [r4, #368]
	bl	FlashDdrParaScan
	ldr	r3, .L1182
	ldrb	r2, [r3, #3995]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1156
	ldr	r3, [r3, #12]
	cmp	r3, #7
	ldrls	r3, .L1182+4
	movls	r2, #60
	strlsb	r2, [r3, #360]
.L1156:
	bl	FlashSavePhyInfo
.L1148:
	ldr	r1, .L1182
	ldr	r3, [r1, #2864]
	ldrh	r2, [r3, #16]
	ubfx	r0, r2, #7, #1
	strb	r0, [r1, #8]
	ldr	r0, .L1182+4
	tst	r2, #64
	ubfx	ip, r2, #3, #1
	strb	ip, [r0, #392]
	ubfx	ip, r2, #4, #1
	strb	ip, [r1, #4025]
	ubfx	ip, r2, #8, #3
	strb	ip, [r1, #3994]
	mov	ip, #0
	str	ip, [r0, #396]
	beq	.L1158
	ldrb	r2, [r3, #19]	@ zero_extendqisi2
	ldrb	ip, [r1, #3133]	@ zero_extendqisi2
	strb	r2, [r1, #3993]
	strb	ip, [r1, #3992]
	ldrb	r1, [r1, #3134]	@ zero_extendqisi2
	strb	r1, [r0, #352]
	sub	r1, r2, #1
	uxtb	r1, r1
	cmp	r1, #5
	bhi	.L1159
	sub	r2, r2, #5
	ldr	r1, .L1182+36
	uxtb	r2, r2
	cmp	r2, #1
	str	r1, [r0, #396]
	movls	r2, #1
	strls	r2, [r0, #-868]
	b	.L1158
.L1159:
	sub	r1, r2, #17
	uxtb	r1, r1
	cmp	r1, #1
	ldrls	r2, .L1182+40
	bls	.L1180
	cmp	r2, #33
	cmpne	r2, #65
	beq	.L1161
	cmp	r2, #66
	bne	.L1162
.L1161:
	ldr	r2, .L1182+4
	mov	r0, #4
	ldr	r1, .L1182+44
	str	r1, [r2, #396]
	ldr	r1, .L1182
	strb	r0, [r1, #3992]
	mov	r1, #7
	strb	r1, [r2, #352]
	b	.L1158
.L1162:
	cmp	r2, #67
	cmpne	r2, #34
	beq	.L1163
	cmp	r2, #35
	beq	.L1163
	cmp	r2, #68
	bne	.L1164
.L1163:
	ldr	r1, .L1182+4
	cmp	r2, #35
	cmpne	r2, #68
	ldr	r0, .L1182+44
	sub	r2, r2, #67
	uxtb	r2, r2
	str	r0, [r1, #396]
	mov	r0, #7
	strb	r0, [r1, #352]
	moveq	r0, #17
	streqb	r0, [r1, #352]
	cmp	r2, #1
	ldr	r2, .L1182
	movls	r1, #4
	movhi	r1, #5
	strb	r1, [r2, #3992]
	b	.L1158
.L1164:
	cmp	r2, #49
	bne	.L1158
	ldr	r2, .L1182+48
.L1180:
	str	r2, [r0, #396]
.L1158:
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	ldrh	r0, [r3, #10]
	ldrb	r4, [r3, #18]	@ zero_extendqisi2
	bl	__aeabi_idiv
	mov	r1, r0
	mov	r0, r4
	ldr	r4, .L1182
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	ldrb	r3, [r4, #2160]	@ zero_extendqisi2
	cmp	r3, #44
	bne	.L1167
	ldrb	r3, [r4, #3995]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1167
	mov	r3, #0
	mov	r0, #1
	strb	r3, [r4, #3995]
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
.L1167:
	ldrb	r3, [r4, #3994]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L1168
	ldr	r2, .L1182
	ldrb	r2, [r2, #3995]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1169
	tst	r3, #1
	bne	.L1168
.L1169:
	ldr	r4, .L1182+4
	mov	r0, #0
	ldr	r1, [r4, #368]
	bl	FlashDdrParaScan
	ldr	r3, .L1182
	ldr	r3, [r3, #12]
	cmp	r3, #7
	movls	r3, #40
	strlsb	r3, [r4, #360]
.L1168:
	ldr	r4, .L1182
	ldr	r3, [r4, #2864]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	add	r0, r4, #112
	bl	FlashReadIdbDataRaw
	mov	r3, r4
	cmp	r0, #0
	movne	r3, #16
	strneb	r3, [r4, #1]
	bne	.L1172
	ldr	r2, .L1182+4
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	ldrb	r2, [r2, #372]	@ zero_extendqisi2
	cmp	r1, r2
	strhib	r2, [r3, #1]
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, #15
	ldrls	r3, .L1182
	movls	r2, #16
	strlsb	r2, [r3, #1]
.L1172:
	ldr	r4, .L1182
	movw	r6, #2274
	ldr	r0, .L1182+52
	bl	FlashTimingCfg
	movw	r3, #2268
	ldr	r5, [r4, #2864]
	ldrb	r2, [r5, #12]	@ zero_extendqisi2
	ldrh	r7, [r5, #14]
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	ldrb	r2, [r4, #4012]	@ zero_extendqisi2
	str	r3, [r4, #2264]
	ldr	r3, [r4, #2160]
	str	r3, [r4, #2260]
	movw	r3, #2270
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #2272
	ldrb	r2, [r5, #13]	@ zero_extendqisi2
	strh	r7, [r4, r6]	@ movhi
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #4
	ldrh	r2, [r5, #10]
	strh	r2, [r4, r3]	@ movhi
	ldrb	r1, [r5, #12]	@ zero_extendqisi2
	ldrh	r0, [r5, #10]
	bl	__aeabi_idiv
	movw	r3, #2278
	mov	lr, #512
	movw	r2, #2282
	strh	r0, [r4, r3]	@ movhi
	movw	r0, #2280
	ldrb	ip, [r5, #9]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	ldrh	r1, [r5, #10]
	mul	r1, r1, r3
	movw	r3, #2284
	strh	lr, [r4, r3]	@ movhi
	add	r3, r3, #2
	ldrb	lr, [r4, #1]	@ zero_extendqisi2
	strh	lr, [r4, r3]	@ movhi
	uxth	r1, r1
	ldrb	lr, [r4, #0]	@ zero_extendqisi2
	strh	r1, [r4, r2]	@ movhi
	cmp	lr, #1
	bne	.L1173
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
.L1173:
	ldrb	r0, [r5, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	bl	FlashSuspend
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1174:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1175:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1183:
	.align	2
.L1182:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+3132
	.word	.LANCHOR1+2560
	.word	.LANCHOR1
	.word	.LANCHOR0+80
	.word	.LANCHOR1+2464
	.word	.LC91
	.word	-682
	.word	HynixReadRetrial
	.word	MicronReadRetrial
	.word	ToshibaReadRetrial
	.word	SamsungReadRetrial
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
	mov	r6, r0
	mov	r5, r1
	mov	r8, r2
	mov	sl, r3
	bl	FlashReadRawPage
	cmn	r0, #1
	mov	r4, r0
	bne	.L1185
	ldr	r7, .L1190
	ldrb	r9, [r7, #8]	@ zero_extendqisi2
	cmp	r9, #0
	beq	.L1186
	mov	r3, #0
	mov	r0, r6
	strb	r3, [r7, #8]
	mov	r1, r5
	mov	r2, r8
	mov	r3, sl
	bl	FlashReadRawPage
	strb	r9, [r7, #8]
	cmn	r0, #1
	movne	r4, r0
	bne	.L1185
.L1186:
	ldr	r7, .L1190
	ldrb	r3, [r7, #3995]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1185
	ldr	r3, [r7, #2224]
	mov	r0, r6
	mov	r1, r5
	mov	r2, r8
	ldr	r9, [r3, #304]
	mov	r3, #1
	str	r3, [sp, #0]
	mov	r3, sl
	bl	FlashDdrTunningRead
	cmn	r0, #1
	mov	r4, r0
	beq	.L1187
	ldrb	r3, [r7, #4024]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1185
.L1187:
	ubfx	r0, r9, #8, #8
	bl	NandcSetDdrPara
.L1185:
	ldr	r3, .L1190+4
	ldr	ip, [r3, #396]
	adds	r3, ip, #0
	movne	r3, #1
	cmn	r4, #1
	movne	r3, #0
	cmp	r3, #0
	beq	.L1188
	mov	r1, r5
	mov	r2, r8
	mov	r3, sl
	mov	r0, r6
	blx	ip
	mov	r2, r6
	mov	r3, r5
	mov	r4, r0
	ldr	r0, .L1190+8
	mov	r1, r4
	bl	printk
.L1188:
	mov	r0, r4
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L1191:
	.align	2
.L1190:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC92
	.fnend
	.size	FlashReadPage, .-FlashReadPage
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1225
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldrb	r3, [r3, #2569]	@ zero_extendqisi2
	mov	r7, #0
	mov	r9, r0
	str	r1, [sp, #20]
	str	r2, [sp, #28]
	mov	sl, r7
	str	r3, [sp, #16]
	ldr	r3, .L1225+4
	ldrb	r3, [r3, #8]	@ zero_extendqisi2
	str	r3, [sp, #24]
	b	.L1193
.L1215:
	mov	r3, #36
	ldr	r2, [sp, #20]
	mul	r3, r3, sl
	ldr	r1, [sp, #28]
	ldr	r5, .L1225+4
	add	r6, r9, r3
	str	r3, [sp, #12]
	rsb	r3, sl, r2
	mov	r0, r6
	add	r2, sp, #36
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldr	r3, [sp, #32]
	ldrb	r2, [r5, #4012]	@ zero_extendqisi2
	cmp	r3, r2
	ldrcs	r2, [sp, #12]
	mvncs	r3, #0
	strcs	r3, [r9, r2]
	mov	r8, r0
	bcs	.L1195
	add	r3, r5, r3
	ldrb	r4, [r3, #3996]	@ zero_extendqisi2
	ldr	r3, .L1225+8
	mov	r0, r4
	ldrb	r3, [r3, #392]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r8, #0
	bl	NandcWaitFlashReady
	ldr	r3, [r5, #2864]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #5
	bhi	.L1197
	add	r2, r5, r4
	ldrb	r3, [r2, #3144]	@ zero_extendqisi2
	ldrb	r2, [r2, #3984]	@ zero_extendqisi2
	cmp	r2, r3
	beq	.L1197
	mov	r0, r4
	ldrb	r1, [r5, #3133]	@ zero_extendqisi2
	add	r2, r5, #3136
	bl	HynixSetRRPara
.L1197:
	mov	r0, r4
	ldr	fp, .L1225+4
	bl	NandcFlashCs
.L1198:
	ldr	r1, [sp, #36]
	cmn	r1, #1
	cmpeq	r4, #255
	moveq	r3, #0
	movne	r3, #1
	moveq	r8, r3
	beq	.L1199
	cmp	r8, #0
	beq	.L1200
	ldr	r2, [fp, #4]
	mov	r0, r4
	add	r2, r1, r2
	bl	FlashReadDpCmd
	b	.L1201
.L1200:
	mov	r0, r4
	bl	FlashReadCmd
	b	.L1201
.L1216:
	mov	r0, r4
	ldr	r1, [sp, #36]
	bl	FlashReadDpDataOutCmd
.L1199:
	ldr	r3, [r6, #12]
	mov	r0, r4
	ldr	r2, [sp, #16]
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
	movne	r3, #0
	movne	r8, #0
	strneb	r3, [fp, #8]
	bne	.L1198
.L1202:
	cmp	r8, #0
	mov	r5, r0
	beq	.L1203
	ldr	r3, .L1225+4
	mov	r0, r4
	ldr	r1, [r3, #4]
	ldr	r3, [sp, #36]
	str	r2, [sp, #8]
	add	r1, r1, r3
	bl	FlashReadDpDataOutCmd
	add	r3, sl, #1
	mov	r1, #36
	ldr	r2, [sp, #8]
	mla	r3, r1, r3, r9
	mov	r0, r4
	ldr	r1, [r3, #12]
	str	r1, [sp, #0]
	mov	r1, r2
	ldr	r3, [r3, #8]
	ldr	r2, [sp, #16]
	bl	NandcXferData
	cmn	r0, #1
	mov	r7, r0
	moveq	r8, #0
.L1203:
	mov	r0, r4
	ldr	fp, .L1225+4
	bl	NandcFlashDeCs
	ldr	r2, [sp, #24]
	cmn	r5, #1
	strb	r2, [fp, #8]
	bne	.L1207
	ldrb	r3, [fp, #3995]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1205
	ldr	r3, [fp, #2224]
	mov	r0, r4
	ldr	r1, [sp, #36]
	ldr	r8, [r3, #304]
	mov	r3, #1
	str	r3, [sp, #0]
	ldr	r2, [r6, #8]
	ldr	r3, [r6, #12]
	bl	FlashDdrTunningRead
	cmn	r0, #1
	mov	r5, r0
	beq	.L1206
	ldrb	r3, [fp, #4024]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1221
.L1206:
	ubfx	r0, r8, #8, #8
	bl	NandcSetDdrPara
	cmn	r5, #1
	bne	.L1221
.L1205:
	ldr	r3, .L1225+8
	mov	r0, r4
	ldr	r1, [sp, #36]
	ldr	r2, [r6, #8]
	ldr	r8, [r3, #396]
	ldr	r3, [r6, #12]
	cmp	r8, #0
	beq	.L1208
	blx	r8
	cmn	r0, #1
	mov	r5, r0
	bne	.L1222
	ldr	r2, .L1225+4
	ldr	r3, [r2, #2864]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #5
	bhi	.L1210
	ldrb	r1, [r2, #3133]	@ zero_extendqisi2
	mov	r0, r4
	add	r2, r2, #3136
	mov	r3, #0
	bl	HynixSetRRPara
.L1210:
	ldr	r1, [sp, #36]
	mov	r0, r4
	ldr	r2, [r6, #8]
	ldr	r3, [r6, #12]
	bl	FlashReadRawPage
	ldr	r3, .L1225+4
	ldr	r1, [r6, #4]
	ldrb	r2, [r3, #4024]	@ zero_extendqisi2
	mov	r5, r0
	ldr	r0, .L1225+12
	mov	r3, r5
	bl	printk
	b	.L1222
.L1208:
	bl	FlashReadRawPage
	mov	r5, r0
	b	.L1209
.L1221:
	mov	r8, #0
.L1207:
	ldr	r3, .L1225+4
	ldrb	r3, [r3, #4024]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r5, r3, lsr #2
	bls	.L1209
	ldr	r3, .L1225+8
	ldr	r3, [r3, #396]
	cmp	r3, #0
	moveq	r5, #256
	b	.L1209
.L1222:
	mov	r8, #0
.L1209:
	ldr	r3, [sp, #12]
	cmp	r5, #256
	cmnne	r5, #1
	movne	r2, #0
	str	r5, [r9, r3]
	strne	r2, [r9, r3]
	ldr	r2, [sp, #12]
	ldr	r3, [r9, r2]
	cmn	r3, #1
	bne	.L1212
	ldr	r2, .L1225+4
	ldr	r1, [r6, #4]
	ldr	r0, .L1225+16
	ldrb	r2, [r2, #4024]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r6, #12]
	cmp	r1, #0
	beq	.L1212
	mov	r2, #4
	ldr	r0, .L1225+20
	mov	r3, r2
	bl	rknand_print_hex
.L1212:
	cmp	r8, #0
	beq	.L1213
	ldr	r3, .L1225+4
	ldrb	r3, [r3, #4024]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r7, r3, lsr #2
	bls	.L1214
	ldr	r3, .L1225+8
	ldr	r3, [r3, #396]
	cmp	r3, #0
	moveq	r7, #256
.L1214:
	add	r3, sl, #1
	mov	r2, #36
	cmp	r7, #256
	cmnne	r7, #1
	mul	r3, r2, r3
	movne	r2, #0
	str	r7, [r9, r3]
	strne	r2, [r9, r3]
.L1213:
	add	sl, sl, r8
.L1195:
	add	sl, sl, #1
.L1193:
	ldr	r3, [sp, #20]
	cmp	sl, r3
	bcc	.L1215
	mov	r0, #0
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1201:
	mov	r0, r4
	bl	NandcWaitFlashReady
	cmp	r8, #0
	bne	.L1216
	b	.L1199
.L1226:
	.align	2
.L1225:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC93
	.word	.LC94
	.word	.LC95
	.fnend
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1253
	movw	r3, #2868
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r4, [r2, r3]
	movw	r3, #65535
	mov	r5, r0
	str	r1, [sp, #12]
	cmp	r4, r3
	beq	.L1246
	cmp	r4, #0
	bne	.L1228
	b	.L1229
.L1246:
	mov	r4, #0
.L1228:
	ldr	r3, .L1253+4
	ldr	r2, .L1253+8
	ldr	fp, [sp, #12]
	ldrh	r3, [r2, r3]
	cmp	fp, r3
	bne	.L1230
.L1229:
	bl	FtlGcPageVarInit
.L1230:
	mvn	r8, #0
	mov	r7, #0
	movw	sl, #65535
.L1241:
	ldrh	r3, [r5, #0]
	mov	r0, #0
	strb	r0, [r5, #8]
	cmp	r3, sl
	beq	.L1247
.L1232:
.L1250:
	ldr	r2, .L1253+12
	mov	r3, #4064
	ldrh	r9, [r2, r3]
	ldr	r3, .L1253+8
	ldr	r2, .L1253+16
	ldr	lr, [r3, #-752]
	ldrh	r2, [r3, r2]
	ldr	ip, [r3, #-948]
	mov	r3, #0
	mov	r6, r3
	str	r2, [sp, #8]
	mov	r2, r5
	b	.L1233
.L1235:
	ldrh	r0, [r2, #16]
	cmp	r0, sl
	beq	.L1234
	mov	fp, #36
	orr	r0, r4, r0, asl #10
	mla	r1, fp, r6, lr
	ldr	fp, [sp, #8]
	str	r0, [r1, #4]
	mov	r0, #0
	str	r0, [r1, #8]
	mul	r0, fp, r6
	add	r6, r6, #1
	uxth	r6, r6
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	bic	r0, r0, #3
	add	r0, ip, r0
	str	r0, [r1, #12]
.L1234:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1233:
	cmp	r3, r9
	bne	.L1235
	ldr	r9, .L1253+8
	mov	r1, r6
	mov	r2, #0
	ldr	r0, [r9, #-752]
	bl	FlashReadPages
	mov	r3, #36
	mul	r3, r3, r6
	mov	r6, #0
	str	r3, [sp, #8]
	b	.L1236
.L1242:
	ldr	r0, .L1253+8
	ldr	r1, [r0, #-752]
	add	r3, r1, r6
	ldr	fp, [r3, #4]
	stmia	sp, {r1, r3}
	ubfx	r0, fp, #10, #16
	bl	P2V_plane
	ldr	r1, [sp, #0]
	ldr	r3, [sp, #4]
	ldr	r1, [r1, r6]
	cmp	r1, #0
	mov	r2, r0
	bne	.L1237
	ldr	r3, [r3, #12]
	add	r6, r6, #36
	ldrh	r1, [r3, #0]
	cmp	r1, sl
	moveq	r3, #1
	streq	r3, [r9, #-784]
	beq	.L1231
.L1238:
	ldr	r0, [r3, #12]
	mov	r1, fp
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
	b	.L1236
.L1237:
	ldr	r3, [r9, #-1668]
	cmp	r3, #0
	beq	.L1239
	ldrh	r3, [r5, #0]
	ldr	r2, [r9, #-1864]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #119
	movls	r8, fp
	bls	.L1240
.L1239:
	cmn	r1, #1
	moveq	r8, fp
.L1240:
	ldr	r2, .L1253+8
	mov	r1, #0	@ movhi
	ldrh	r3, [r5, #0]
	mov	r4, #0
	ldr	r2, [r2, #-1844]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r5, #0]
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r5, #0]	@ movhi
	bl	FtlGcPageVarInit
	b	.L1241
.L1236:
	ldr	r3, [sp, #8]
	ldr	r9, .L1253+8
	cmp	r6, r3
	bne	.L1242
	ldr	fp, [sp, #12]
	add	r7, r7, #1
	add	r4, r4, #1
	cmp	r7, fp
	uxth	r4, r4
	bcc	.L1243
	ldr	r2, .L1253
	movw	r3, #2868
	ldrh	r1, [r2, r3]
	cmp	r1, sl
	beq	.L1243
	add	r1, r1, r7
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, .L1253+4
	ldrh	r3, [r9, r3]
	cmp	r3, r4
	bhi	.L1244
.L1243:
	ldr	r3, .L1253+4
	ldrh	r3, [r9, r3]
	cmp	r3, r4
	bhi	.L1250
.L1247:
	mov	r2, #0
.L1231:
	ldr	r1, .L1253
	movw	r3, #2868
	mvn	r0, #0
	strh	r4, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	strh	r0, [r1, r3]	@ movhi
	mov	r0, r5
	mov	r1, r4
	bl	ftl_sb_update_avl_pages
.L1244:
	mov	r0, r8
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1254:
	.align	2
.L1253:
	.word	.LANCHOR1
	.word	-4052
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-4040
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
	mov	r5, #0
	ldr	r4, .L1313
	.pad #28
	sub	sp, sp, #28
	ldr	r3, .L1313+4
	mov	r1, r5
	ldr	r6, .L1313+8
	ldr	r2, [r4, #-4024]
	mov	sl, r4
	strh	r5, [r4, r3]	@ movhi
	ldr	r3, .L1313+12
	mov	r2, r2, asl #2
	ldr	r0, [r4, #-808]
	strh	r5, [r4, r3]	@ movhi
	bl	ftl_memset
	ldr	r2, [r4, #-4024]
	mov	r1, r5
	ldr	r0, [r4, #-812]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r4, r6]
	mov	r1, r5
	ldr	r0, [r4, #-692]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r4, r6]
	mov	r1, r5
	ldr	r0, [r4, #-4004]
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r2, #12
	sub	r0, r4, #1552
	mov	r1, #255
	bl	ftl_memset
	ldr	r2, .L1313+16
	movw	r3, #4072
	ldrh	r3, [r2, r3]
	str	r3, [sp, #12]
	b	.L1256
.L1258:
	add	r3, r6, r5
	ldr	r1, [sp, #12]
	ldrb	r0, [r3, #4090]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #4]
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #4]
	cmp	r0, #0
	bne	.L1257
	ldr	r2, [r4, #-752]
	mov	r3, r3, asl #10
	mla	r2, r9, r8, r2
	str	r3, [r2, #4]
	ldrh	r3, [r4, r7]
	str	r0, [r2, #8]
	mul	r3, r3, r8
	add	r8, r8, #1
	uxth	r8, r8
	add	r1, r3, #3
	cmp	r3, #0
	movlt	r3, r1
	ldr	r1, [r4, #-948]
	bic	r3, r3, #3
	add	r3, r1, r3
	str	r3, [r2, #12]
.L1257:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L1294
.L1307:
	ldr	r7, .L1313+20
	mov	r8, #0
	mov	r5, r8
	mov	fp, #4064
	mov	r9, #36
.L1294:
	ldrh	r3, [r6, fp]
	cmp	r3, r5
	bhi	.L1258
	ldr	r0, [r4, #-752]
	mov	r1, r8
	mov	r2, #1
	mov	r7, #0
	bl	FlashReadPages
	mov	fp, r8
	str	r7, [sp, #16]
	b	.L1259
.L1293:
	ldr	r2, [r4, #-752]
	add	r0, r2, r7
	ldr	r8, [r2, r7]
	ldr	r3, [r0, #4]
	cmn	r8, #1
	ldr	r5, [r0, #12]
	ubfx	r6, r3, #10, #16
	bne	.L1260
	mov	r1, #1
	add	r3, r3, #1
	mov	r2, r1
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldrh	r3, [r5, #0]
	movw	r0, #65535
	cmp	r3, r0
	ldreq	r3, [r4, #-752]
	streq	r8, [r3, r7]
.L1260:
	ldr	r3, [r4, #-752]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	beq	.L1261
	ldr	r2, [r4, #-1632]
	ldr	r3, [r5, #4]
	cmn	r2, #1
	beq	.L1262
	cmp	r2, r3
	bhi	.L1263
.L1262:
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [sl, #-1632]
.L1263:
	ldrh	r3, [r5, #0]
	movw	r2, #61604
	cmp	r3, r2
	beq	.L1266
	bhi	.L1269
	movw	r2, #61574
	cmp	r3, r2
	bne	.L1264
	b	.L1311
.L1269:
	movw	r2, #61634
	cmp	r3, r2
	beq	.L1267
	movw	r1, #65535
	cmp	r3, r1
	bne	.L1264
	b	.L1312
.L1267:
	ldr	r3, .L1313+4
	ldr	r2, [r4, #-4024]
	ldr	r1, [r4, #-808]
	ldrh	r3, [r4, r3]
	uxth	r0, r2
	sub	r8, r0, #1
	rsb	r0, r3, r0
	sub	r0, r0, #1
	uxth	r8, r8
	sxth	r0, r0
	str	r0, [sp, #8]
	b	.L1270
.L1278:
	mov	ip, r9, asl #2
	ldr	r0, [r1, r9, asl #2]
	str	ip, [sp, #20]
	ldr	ip, [r5, #4]
	cmp	ip, r0
	bls	.L1271
	ldr	r1, [r1, #0]
	cmp	r1, #0
	bne	.L1272
	cmp	r3, r2
	addne	r3, r3, #1
	ldrne	r2, .L1313+4
	strneh	r3, [r4, r2]	@ movhi
	bne	.L1274
.L1272:
	ldr	r3, [r4, #-812]
	mov	r1, #1
	ldrh	r0, [r3, #0]
	bl	FtlFreeSysBlkQueueIn
.L1274:
	sxth	ip, r8
	mov	r2, #0
	str	ip, [sp, #8]
	b	.L1275
.L1276:
	ldr	r0, [r4, #-808]
	add	r1, r3, #1
	add	r2, r2, #1
	ldr	ip, [r0, r1, asl #2]
	mov	r1, r1, asl #1
	uxth	r2, r2
	str	ip, [r0, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r0, [r4, #-812]
	ldrh	r1, [r0, r1]
	strh	r1, [r0, r3]	@ movhi
.L1275:
	ldr	r0, [sp, #8]
	sxth	r3, r2
	cmp	r3, r0
	bne	.L1276
	ldr	r3, [sl, #-808]
	mov	r9, r9, asl #1
	ldr	r2, [r5, #4]
	ldr	ip, [sp, #20]
	str	r2, [r3, ip]
	ldr	r3, [sl, #-812]
	strh	r6, [r3, r9]	@ movhi
	b	.L1277
.L1271:
	sub	r8, r8, #1
	uxth	r8, r8
.L1270:
	ldr	r0, [sp, #8]
	sxth	r9, r8
	cmp	r9, r0
	bgt	.L1278
.L1277:
	sxth	r8, r8
	cmp	r8, #0
	blt	.L1261
.L1279:
	ldr	r3, .L1313+4
	ldr	r1, [r4, #-4024]
	ldrh	r2, [r4, r3]
	rsb	r1, r2, r1
	sub	r1, r1, #1
	sxth	r1, r1
	cmp	r8, r1
	bgt	.L1264
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #-808]
	ldr	r2, [r5, #4]
	str	r2, [r3, r8, asl #2]
	mov	r8, r8, asl #1
	ldr	r3, [r4, #-812]
	b	.L1309
.L1311:
	ldr	r3, .L1313+8
	ldrh	r2, [r4, r3]
	add	r3, r3, #24
	sub	r1, r2, #1
	ldrh	r3, [r4, r3]
	uxth	r8, r1
	rsb	r1, r3, r1
	str	r1, [sp, #8]
	ldr	r1, [r4, #-692]
	b	.L1280
.L1288:
	mov	ip, r9, asl #2
	ldr	r0, [r1, r9, asl #2]
	str	ip, [sp, #20]
	ldr	ip, [r5, #4]
	cmp	ip, r0
	bls	.L1281
	ldr	r1, [r1, #0]
	cmp	r1, #0
	bne	.L1282
	cmp	r3, r2
	bne	.L1283
.L1282:
	ldr	r3, [r4, #-4004]
	mov	r1, #1
	ldrh	r0, [r3, #0]
	bl	FtlFreeSysBlkQueueIn
	b	.L1284
.L1283:
	ldr	r2, .L1313+12
	add	r3, r3, #1
	strh	r3, [r4, r2]	@ movhi
.L1284:
	sxth	ip, r8
	mov	r2, #0
	str	ip, [sp, #8]
	b	.L1285
.L1286:
	ldr	r0, [r4, #-692]
	add	r1, r3, #1
	add	r2, r2, #1
	ldr	ip, [r0, r1, asl #2]
	mov	r1, r1, asl #1
	uxth	r2, r2
	str	ip, [r0, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r0, [r4, #-4004]
	ldrh	r1, [r0, r1]
	strh	r1, [r0, r3]	@ movhi
.L1285:
	ldr	r0, [sp, #8]
	sxth	r3, r2
	cmp	r3, r0
	bne	.L1286
	ldr	r3, [sl, #-692]
	mov	r9, r9, asl #1
	ldr	r2, [r5, #4]
	ldr	ip, [sp, #20]
	str	r2, [r3, ip]
	ldr	r3, [sl, #-4004]
	strh	r6, [r3, r9]	@ movhi
	b	.L1287
.L1281:
	sub	r8, r8, #1
	uxth	r8, r8
.L1280:
	ldr	r0, [sp, #8]
	sxth	r9, r8
	cmp	r9, r0
	bgt	.L1288
.L1287:
	sxth	r8, r8
	cmp	r8, #0
	blt	.L1261
.L1289:
	ldr	r1, .L1313+8
	ldr	r3, .L1313+12
	ldrh	r1, [r4, r1]
	ldrh	r2, [r4, r3]
	sub	r1, r1, #1
	rsb	r1, r2, r1
	sxth	r1, r1
	cmp	r8, r1
	bgt	.L1264
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #-692]
	ldr	r2, [r5, #4]
	str	r2, [r3, r8, asl #2]
	mov	r8, r8, asl #1
	ldr	r3, [r4, #-4004]
.L1309:
	strh	r6, [r3, r8]	@ movhi
	b	.L1264
.L1266:
	ldr	r3, .L1313+24
	movw	r1, #65535
	ldrh	r2, [r4, r3]
	cmp	r2, r1
	streqh	r6, [r4, r3]	@ movhi
	beq	.L1310
	ldr	r3, .L1313+28
	movw	r2, #65535
	ldrh	r0, [r4, r3]
	cmp	r0, r2
	beq	.L1291
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L1291:
	ldr	r3, [r5, #4]
	ldr	r2, [sl, #-1544]
	cmp	r2, r3
	ldr	r3, .L1313+28
	strcsh	r6, [r4, r3]	@ movhi
	bcs	.L1264
	ldr	r2, .L1313+24
	ldrh	r1, [r4, r2]
	strh	r6, [r4, r2]	@ movhi
	strh	r1, [r4, r3]	@ movhi
.L1310:
	ldr	r3, [r5, #4]
	str	r3, [r4, #-1544]
	b	.L1264
.L1312:
	mov	r0, r6
	mov	r1, #0
	b	.L1308
.L1261:
	mov	r0, r6
	mov	r1, #1
.L1308:
	bl	FtlFreeSysBlkQueueIn
.L1264:
	ldr	ip, [sp, #16]
	add	r7, r7, #36
	add	r3, ip, #1
	uxth	r3, r3
	str	r3, [sp, #16]
.L1259:
	ldr	r3, [sp, #16]
	cmp	r3, fp
	bne	.L1293
	ldr	ip, [sp, #12]
	add	r3, ip, #1
	uxth	r3, r3
	str	r3, [sp, #12]
.L1256:
	ldr	r6, .L1313+16
	movw	r3, #4074
	ldr	ip, [sp, #12]
	ldrh	r3, [r6, r3]
	cmp	r3, ip
	bhi	.L1307
	ldr	r2, .L1313
	ldr	r1, [r2, #-812]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L1295
	ldr	r0, .L1313+4
	ldrh	r0, [r2, r0]
	cmp	r0, #0
	ldrne	r0, [r2, #-4024]
	bne	.L1296
	b	.L1295
.L1300:
	mov	r2, r2, asl #1
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	beq	.L1297
	ldr	r2, .L1313
	sxth	r6, r3
	mov	r5, #0
	b	.L1298
.L1299:
	ldr	ip, [r2, #-812]
	mov	r0, r1, asl #1
	rsb	r4, r6, r1
	add	r3, r3, #1
	ldrh	r8, [ip, r0]
	mov	r7, r4, asl #1
	uxth	r3, r3
	strh	r8, [ip, r7]	@ movhi
	ldr	ip, [r2, #-808]
	ldr	r1, [ip, r1, asl #2]
	str	r1, [ip, r4, asl #2]
	ldr	r1, [r2, #-812]
	strh	r5, [r1, r0]	@ movhi
.L1298:
	ldr	r0, [r2, #-4024]
	sxth	r1, r3
	cmp	r1, r0
	bcc	.L1299
	b	.L1295
.L1297:
	add	r3, r3, #1
	uxth	r3, r3
.L1296:
	sxth	r2, r3
	cmp	r2, r0
	bcc	.L1300
.L1295:
	ldr	r2, .L1313
	ldr	r1, [r2, #-4004]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L1301
	ldr	r0, .L1313+12
	ldrh	r0, [r2, r0]
	cmp	r0, #0
	ldrne	r0, .L1313+8
	ldrneh	r2, [r2, r0]
	bne	.L1302
	b	.L1301
.L1306:
	mov	ip, r0, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, #0
	beq	.L1303
	mov	r1, r3
	ldr	r7, .L1313+8
	ldr	r3, .L1313
	mov	r6, #0
	b	.L1304
.L1305:
	ldr	r4, [r3, #-4004]
	mov	ip, r2, asl #1
	rsb	r5, r0, r2
	add	r1, r1, #1
	ldrh	sl, [r4, ip]
	mov	r8, r5, asl #1
	uxth	r1, r1
	strh	sl, [r4, r8]	@ movhi
	ldr	r4, [r3, #-692]
	ldr	r2, [r4, r2, asl #2]
	str	r2, [r4, r5, asl #2]
	ldr	r2, [r3, #-4004]
	strh	r6, [r2, ip]	@ movhi
.L1304:
	ldrh	ip, [r3, r7]
	sxth	r2, r1
	cmp	r2, ip
	blt	.L1305
	b	.L1301
.L1303:
	add	r3, r3, #1
	uxth	r3, r3
.L1302:
	sxth	r0, r3
	cmp	r0, r2
	blt	.L1306
.L1301:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1314:
	.align	2
.L1313:
	.word	.LANCHOR2
	.word	-816
	.word	-4032
	.word	-4008
	.word	.LANCHOR0
	.word	-4040
	.word	-1552
	.word	-1548
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r2
	ldr	r8, .L1326
	add	r2, r1, r0
	ldr	r3, .L1326+4
	.pad #52
	sub	sp, sp, #52
	mov	r6, r0
	mov	r5, r1
	ldrh	r9, [r8, r3]
	sub	r3, r3, #14
	ldrh	r3, [r8, r3]
	cmp	r2, r3
	movls	r9, r0, lsr r9
	movls	r7, #0
	movls	r9, r9, asl #2
	bhi	.L1325
	b	.L1317
.L1322:
	ldr	r1, .L1326+8
	mov	r0, r6
	ldr	r3, [r8, #-688]
	ldrh	r4, [r8, r1]
	ldr	fp, [r3, r9]
	mov	r1, r4
	bl	__aeabi_uidivmod
	uxth	r3, r1
	rsb	r4, r3, r4
	uxth	r4, r4
	cmp	r4, r5
	uxthhi	r4, r5
	cmp	fp, #0
	beq	.L1319
	ldr	r1, .L1326
	add	r0, sp, #12
	str	fp, [sp, #16]
	ldr	r2, [r1, #-736]
	mov	r1, #1
	str	r3, [sp, #4]
	str	r2, [sp, #20]
	mov	r2, #0
	str	r2, [sp, #24]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #4]
	mov	r0, sl
	cmn	r2, #1
	ldr	r1, [r8, #-736]
	moveq	r7, r2
	add	r1, r1, r3, asl #9
	mov	r2, r4, asl #9
	bl	memcpy
	b	.L1321
.L1319:
	mov	r0, sl
	mov	r1, fp
	mov	r2, r4, asl #9
	bl	ftl_memset
.L1321:
	rsb	r5, r4, r5
	add	r6, r6, r4
	add	sl, sl, r4, asl #9
	add	r9, r9, #4
.L1317:
	cmp	r5, #0
	bne	.L1322
	b	.L1316
.L1325:
	mvn	r7, #0
.L1316:
	mov	r0, r7
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1327:
	.align	2
.L1326:
	.word	.LANCHOR2
	.word	-4044
	.word	-4046
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
	ldr	r4, .L1332
	mov	r1, #1
	ldr	r5, .L1332+4
	sub	r2, r4, #1488
	bl	FtlVendorPartRead
	ldr	r3, [r4, #-1488]
	cmp	r3, r5
	beq	.L1329
	sub	r0, r4, #1488
	mov	r1, #0
	mov	r2, #512
	bl	ftl_memset
	str	r5, [r4, #-1488]
.L1329:
	ldr	r2, [r4, #-1488]
	ldr	r3, .L1332
	cmp	r2, r5
	bne	.L1330
	ldr	r2, [r3, #-1400]
	str	r2, [r3, #-1640]
	ldr	r2, [r3, #-1396]
	str	r2, [r3, #-1636]
	ldr	r2, [r3, #-1480]
	str	r2, [r3, #-1644]
	ldr	r2, [r3, #-1476]
	str	r2, [r3, #-1656]
	ldr	r2, [r3, #-1472]
	str	r2, [r3, #-1664]
	ldr	r2, [r3, #-1468]
	str	r2, [r3, #-1648]
	ldr	r2, [r3, #-1460]
	str	r2, [r3, #-1620]
	ldr	r2, [r3, #-1456]
	str	r2, [r3, #-1860]
	ldr	r2, [r3, #-1452]
	str	r2, [r3, #-1660]
	ldr	r2, [r3, #-1448]
	str	r2, [r3, #-1652]
	ldr	r2, [r3, #-1444]
	str	r2, [r3, #-1612]
	ldr	r2, [r3, #-1440]
	str	r2, [r3, #-1608]
	ldr	r2, [r3, #-1428]
	str	r2, [r3, #-1492]
.L1330:
	ldr	r4, .L1332
	mov	r3, #0
	ldr	r2, [r4, #-1420]
	str	r3, [r4, #-772]
	ldr	r3, .L1332+8
	cmp	r2, r3
	bne	.L1331
	ldr	r0, .L1332+12
	mov	r3, #1
	ldr	r1, .L1332+16
	str	r3, [r4, #-1668]
	bl	printk
.L1331:
	ldr	r3, .L1332+20
	ldr	r0, [r4, #-1624]
	ldrh	r2, [r4, r3]
	ldr	r3, [r4, #-1620]
	mla	r0, r0, r2, r3
	ldr	r2, .L1332+24
	movw	r3, #4072
	ldrh	r1, [r2, r3]
	bl	__aeabi_uidiv
	str	r0, [r4, #-1616]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1333:
	.align	2
.L1332:
	.word	.LANCHOR2
	.word	1179929683
	.word	305432421
	.word	.LC74
	.word	.LC96
	.word	-4062
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
	ldr	r4, .L1336
	ldr	r5, .L1336+4
	ldr	r2, [r4, #-704]
	ldrh	r1, [r4, r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #-704]
	ldr	r2, [r3, #0]
	ldr	r3, .L1336+8
	cmp	r2, r3
	beq	.L1335
	ldr	r1, .L1336+12
	ldr	r0, .L1336+16
	bl	printk
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #-704]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1335:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1337:
	.align	2
.L1336:
	.word	.LANCHOR2
	.word	-708
	.word	1112818501
	.word	.LC97
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
	ldr	r4, .L1341
	mov	r5, #12
	ldr	r3, [r4, #-804]
	ldr	r7, [r3, r0, asl #2]
	cmp	r7, #0
	bne	.L1339
	mul	r5, r5, r1
	ldr	r3, [r4, #-1680]
	mov	r1, #255
	add	r3, r3, r5
	ldr	r0, [r3, #8]
	ldr	r3, .L1341+4
	ldrh	r2, [r4, r3]
	bl	ftl_memset
	ldr	r3, [r4, #-1680]
	strh	r6, [r3, r5]	@ movhi
	ldr	r3, [r4, #-1680]
	add	r5, r3, r5
	str	r7, [r5, #4]
	b	.L1340
.L1339:
	mul	r5, r5, r1
	ldr	r3, [r4, #-1680]
	mov	r1, #1
	add	r0, r4, #400
	mov	r2, r1
	str	r7, [r4, #404]
	add	r3, r3, r5
	ldr	r3, [r3, #8]
	str	r3, [r4, #408]
	ldr	r3, [r4, #-720]
	str	r3, [r4, #412]
	bl	FlashReadPages
	ldr	r3, [r4, #-1680]
	mov	r1, #0
	add	r2, r3, r5
	str	r1, [r2, #4]
	strh	r6, [r3, r5]	@ movhi
.L1340:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1342:
	.align	2
.L1341:
	.word	.LANCHOR2
	.word	-4042
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
	ldr	r3, .L1354
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	.pad #108
	sub	sp, sp, #108
	ldreq	r2, .L1354+4
	add	r7, sp, #4
	ldrne	r2, .L1354+8
	mov	r5, r1
	mov	sl, r0, asl #10
	mov	r1, #1
	mov	r0, r7
	mov	r8, #0
	ldrh	r4, [r3, r2]
	add	r3, sp, #40
	mov	r2, r5
	str	r3, [sp, #16]
	sub	r4, r4, #1
	str	r8, [sp, #12]
	uxth	r4, r4
	sxth	r3, r4
	orr	r3, r3, sl
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #40]
	cmn	r3, #1
	bne	.L1347
	b	.L1352
.L1350:
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
	bne	.L1348
	ldr	r3, [sp, #44]
	cmn	r3, #1
	bne	.L1348
	ldr	r3, [sp, #4]
	cmn	r3, #1
	subne	r4, r6, #1
	uxthne	r4, r4
	bne	.L1352
.L1348:
	add	r6, r6, #1
	uxth	r8, r6
.L1352:
	sxth	r6, r8
	sxth	r3, r4
	cmp	r6, r3
	ble	.L1350
.L1347:
	sxth	r0, r4
	add	sp, sp, #108
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1355:
	.align	2
.L1354:
	.word	.LANCHOR2
	.word	-4050
	.word	-4052
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
	ldr	r4, .L1370
	mov	r1, r5
	ldr	r2, .L1370+4
	ldr	r7, .L1370+8
	ldr	r3, [r4, #-720]
	ldr	r0, [r4, #-1844]
	str	r5, [r4, #408]
	str	r3, [r4, #412]
	movw	r3, #4072
	ldrh	r2, [r2, r3]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r0, [r4, r7]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1368
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	ldr	r3, .L1370+12
	add	r2, r0, #1
	mov	r6, r0
	strh	r2, [r4, r3]	@ movhi
	b	.L1358
.L1360:
	ldrh	r2, [r4, r7]
	mov	r1, #1
	ldr	r0, .L1370+16
	sub	r5, r5, #1
	orr	r3, r3, r2, asl #10
	str	r3, [r4, #404]
	ldr	r3, [r4, #-1856]
	mov	r2, r1
	uxth	r5, r5
	str	r3, [r4, #408]
	bl	FlashReadPages
	ldr	r3, [r4, #400]
	cmn	r3, #1
	bne	.L1359
.L1358:
	add	r3, r5, r6
	sxth	r3, r3
	cmp	r3, #0
	bge	.L1360
.L1359:
	ldr	r4, .L1370
	mov	r2, #48
	ldr	r5, .L1370+4
	ldr	r0, .L1370+20
	ldr	r6, [r4, #408]
	mov	r1, r6
	bl	memcpy
	movw	r3, #4072
	ldrh	r2, [r5, r3]
	add	r1, r6, #48
	ldr	r0, [r4, #-1844]
	mov	r2, r2, asl #1
	bl	memcpy
	ldr	r2, [r4, #-1604]
	ldr	r3, .L1370+24
	cmp	r2, r3
	bne	.L1368
	ldr	r3, .L1370+28
	ldr	r2, .L1370+32
	ldrb	r1, [r4, #-1594]	@ zero_extendqisi2
	ldrh	r3, [r4, r3]
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #5632
	ldrh	r2, [r5, r2]
	cmp	r1, r2
	bne	.L1368
	ldr	r2, .L1370+36
	ldr	r1, .L1370+40
	ldr	r0, [r5, #4076]
	ldrh	r2, [r4, r2]
	ldrh	r1, [r4, r1]
	str	r3, [r4, #436]
	mul	r2, r3, r2
	str	r2, [r4, #-3992]
	mul	r2, r1, r2
	str	r2, [r4, #-4012]
	ldr	r2, .L1370+44
	ldrh	r2, [r4, r2]
	rsb	r0, r2, r0
	rsb	r0, r3, r0
	mov	r3, #4064
	ldrh	r1, [r5, r3]
	movw	r5, #65535
	bl	__aeabi_uidiv
	ldr	r3, .L1370+48
	ldr	r1, .L1370+52
	strh	r0, [r4, r3]	@ movhi
	sub	r3, r3, #34
	ldrh	r2, [r4, r3]
	ldr	r3, .L1370+56
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #236
	ldrh	r3, [r4, r3]
	mov	r0, r3, lsr #6
	and	r3, r3, #63
	strb	r3, [r4, #-1818]
	ldrb	r3, [r4, #-1593]	@ zero_extendqisi2
	strh	r0, [r4, r1]	@ movhi
	mvn	r1, #0
	ldr	r0, .L1370+60
	strb	r3, [r4, #-1816]
	ldr	r3, .L1370+64
	strh	r1, [r4, r3]	@ movhi
	mov	r3, #0
	ldr	r1, .L1370+68
	strh	r3, [r4, r0]	@ movhi
	strb	r3, [r4, #-1534]
	ldrh	r0, [r4, r1]
	ldr	r1, .L1370+72
	strb	r3, [r4, #-1532]
	str	r3, [r4, #-1644]
	strh	r0, [r4, r1]	@ movhi
	add	r1, r1, #192
	ldr	r0, .L1370+76
	ldrh	r1, [r4, r1]
	mov	ip, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-1770]
	ldrb	r1, [r4, #-1592]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	strb	r1, [r4, #-1768]
	ldr	r1, .L1370+80
	ldrh	r0, [r4, r1]
	ldr	r1, .L1370+84
	strh	r0, [r4, r1]	@ movhi
	add	r1, r1, #148
	ldr	r0, .L1370+88
	ldrh	r1, [r4, r1]
	mov	ip, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-1722]
	ldrb	r1, [r4, #-1591]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	strb	r1, [r4, #-1720]
	str	r3, [r4, #-1656]
	ldr	r1, [r4, #-1572]
	str	r3, [r4, #-1664]
	str	r3, [r4, #-1648]
	str	r3, [r4, #-1620]
	str	r3, [r4, #-1612]
	str	r3, [r4, #-1652]
	ldr	r3, [r4, #-1564]
	str	r1, [r4, #-1624]
	ldr	r1, [r4, #-1632]
	cmp	r3, r1
	strhi	r3, [r4, #-1632]
	ldr	r4, .L1370
	ldr	r3, [r4, #-1568]
	ldr	r1, [r4, #-1628]
	cmp	r3, r1
	strhi	r3, [r4, #-1628]
	cmp	r2, r5
	beq	.L1363
	ldr	r0, .L1370+92
	bl	make_superblock
.L1363:
	ldr	r3, .L1370+72
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1364
	ldr	r0, .L1370+96
	bl	make_superblock
.L1364:
	ldr	r4, .L1370
	movw	r5, #65535
	ldr	r3, .L1370+84
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1365
	sub	r0, r4, #1728
	bl	make_superblock
.L1365:
	ldr	r3, .L1370+64
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1369
	ldr	r0, .L1370+100
	bl	make_superblock
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1368:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1369:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1371:
	.align	2
.L1370:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1552
	.word	-1550
	.word	.LANCHOR2+400
	.word	.LANCHOR2-1604
	.word	1179929683
	.word	-1596
	.word	-1546
	.word	-4052
	.word	-4046
	.word	-3978
	.word	-1556
	.word	-1822
	.word	-1824
	.word	-1538
	.word	-1540
	.word	-1586
	.word	-1776
	.word	-1774
	.word	-1582
	.word	-1728
	.word	-1726
	.word	.LANCHOR2-1824
	.word	.LANCHOR2-1776
	.word	.LANCHOR2-1540
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
	ldr	r4, .L1387
	movw	r7, #61649
	ldr	r8, .L1387+4
	ldr	r5, [r4, #-720]
	str	r3, [r4, #408]
	str	r5, [r4, #412]
	bl	FtlBbtMemInit
	ldrh	r6, [r4, r8]
	sub	r6, r6, #1
	uxth	r6, r6
	b	.L1373
.L1377:
	mov	r1, #1
	mov	r3, r6, asl #10
	ldr	r0, .L1387+8
	mov	r2, r1
	str	r3, [r4, #404]
	bl	FlashReadPages
	ldr	r3, [r4, #400]
	cmn	r3, #1
	bne	.L1374
	ldr	r3, [r4, #404]
	mov	r1, #1
	ldr	r0, .L1387+8
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r4, #404]
	bl	FlashReadPages
.L1374:
	ldr	r2, [r4, #400]
	ldr	r3, .L1387
	cmn	r2, #1
	beq	.L1375
	ldrh	r2, [r5, #0]
	cmp	r2, r7
	bne	.L1375
	ldr	r2, .L1387+12
	strh	r6, [r3, r2]	@ movhi
	ldr	r2, [r5, #4]
	str	r2, [r3, #-3976]
	ldr	r2, .L1387+16
	ldrh	r1, [r5, #8]
	strh	r1, [r3, r2]	@ movhi
	b	.L1376
.L1375:
	sub	r6, r6, #1
	uxth	r6, r6
.L1373:
	ldrh	r3, [r4, r8]
	sub	r3, r3, #48
	cmp	r6, r3
	bgt	.L1377
.L1376:
	ldr	r4, .L1387
	movw	r3, #65535
	ldr	r7, .L1387+12
	ldrh	r2, [r4, r7]
	cmp	r2, r3
	beq	.L1386
	ldr	r6, .L1387+16
	ldrh	r2, [r4, r6]
	cmp	r2, r3
	beq	.L1379
	mov	r1, #1
	mov	r2, r2, asl #10
	add	r0, r4, #400
	str	r2, [r4, #404]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r4, #400]
	cmn	r3, #1
	beq	.L1379
	ldrh	r2, [r5, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1379
	ldr	r3, [r5, #4]
	ldr	r2, [r4, #-3976]
	cmp	r3, r2
	strhi	r3, [r4, #-3976]
	ldrhih	r2, [r4, r6]
	ldrhih	r3, [r5, #8]
	strhih	r2, [r4, r7]	@ movhi
	strhih	r3, [r4, r6]	@ movhi
.L1379:
	ldr	r4, .L1387
	mov	r1, #1
	ldr	r8, .L1387+12
	mov	r6, #0
	ldrh	r0, [r4, r8]
	bl	FtlGetLastWrittenPage
	ldr	r3, .L1387+20
	add	r2, r0, #1
	mov	r7, r0
	strh	r2, [r4, r3]	@ movhi
	b	.L1380
.L1382:
	ldrh	r2, [r4, r8]
	mov	r1, #1
	ldr	r0, .L1387+8
	sub	r6, r6, #1
	orr	r3, r3, r2, asl #10
	str	r3, [r4, #404]
	ldr	r3, [r4, #-1856]
	mov	r2, r1
	uxth	r6, r6
	str	r3, [r4, #408]
	bl	FlashReadPages
	ldr	r3, [r4, #400]
	cmn	r3, #1
	bne	.L1381
.L1380:
	add	r3, r6, r7
	sxth	r3, r3
	cmp	r3, #0
	bge	.L1382
.L1381:
	ldr	r3, .L1387+24
	ldrh	r1, [r5, #10]
	ldr	r2, .L1387
	ldrh	r0, [r5, #12]
	strh	r1, [r2, r3]	@ movhi
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1383
	ldr	r3, .L1387+28
	ldr	r2, [r3, #4060]
	cmp	r0, r2
	beq	.L1383
	movw	r1, #4074
	ldrh	r3, [r3, r1]
	mov	r3, r3, lsr #2
	cmp	r2, r3
	bcs	.L1383
	cmp	r0, r3
	bcs	.L1383
	bl	FtlSysBlkNumInit
.L1383:
	ldr	r6, .L1387+32
	mov	r4, #0
	ldr	r8, .L1387+28
	movw	r7, #4086
	ldr	r5, .L1387
	b	.L1384
.L1385:
	ldr	r3, .L1387+36
	ldr	r1, [r5, #408]
	ldr	r0, [r6, #4]!
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #2
	mla	r1, r4, r2, r1
	bl	memcpy
	add	r4, r4, #1
.L1384:
	ldrh	r3, [r8, r7]
	cmp	r4, r3
	bcc	.L1385
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1386:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1388:
	.align	2
.L1387:
	.word	.LANCHOR2
	.word	-4056
	.word	.LANCHOR2+400
	.word	-3984
	.word	-3980
	.word	-3982
	.word	-3978
	.word	.LANCHOR0
	.word	.LANCHOR2-3960
	.word	-800
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
	mov	r6, #0
	ldr	r5, .L1396
	ldr	r7, .L1396+4
	ldr	fp, .L1396+8
	ldr	r3, [r5, #-1856]
	ldr	r8, [r5, #-720]
	str	r3, [r5, #408]
	str	r8, [r5, #412]
	b	.L1390
.L1395:
	ldr	sl, .L1396+12
	mvn	r3, #0
	movw	r9, #61664
	strh	r3, [r7], #2	@ movhi
	ldrh	r4, [r5, sl]
	add	r4, r4, r3
	uxth	r4, r4
	b	.L1391
.L1394:
	mla	r3, r3, r6, r4
	mov	r1, #1
	ldr	r0, .L1396+16
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [r5, #404]
	bl	FlashReadPages
	ldr	r3, [r5, #400]
	cmn	r3, #1
	beq	.L1392
	ldrh	r3, [r8, #0]
	cmp	r3, r9
	streqh	r4, [r7, #-2]	@ movhi
	beq	.L1393
.L1392:
	sub	r4, r4, #1
	uxth	r4, r4
.L1391:
	ldrh	r3, [r5, sl]
	sub	r2, r3, #16
	cmp	r4, r2
	bgt	.L1394
.L1393:
	add	r6, r6, #1
.L1390:
	movw	r3, #4086
	ldrh	r3, [fp, r3]
	cmp	r6, r3
	bcc	.L1395
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1397:
	.align	2
.L1396:
	.word	.LANCHOR2
	.word	.LANCHOR2-3972
	.word	.LANCHOR0
	.word	-4056
	.word	.LANCHOR2+400
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
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
	ldr	r5, .L1422
	mov	sl, r0
	str	r3, [sp, #20]
	mov	fp, r1
	str	r2, [sp, #12]
	mov	r6, #0
	ldr	r3, [r5, #2864]
	mov	r9, r5
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	str	r3, [sp, #16]
	ldr	r3, .L1422+4
	ldrb	r3, [r3, #2569]	@ zero_extendqisi2
	str	r3, [sp, #8]
	b	.L1399
.L1410:
	mov	r4, #36
	rsb	r3, r6, fp
	mul	r4, r4, r6
	uxtb	r3, r3
	add	r2, sp, #28
	str	r3, [sp, #0]
	ldr	r1, [sp, #12]
	add	r3, sp, #32
	add	r7, sl, r4
	mov	r0, r7
	bl	LogAddr2PhyAddr
	ldrb	r3, [r5, #4012]	@ zero_extendqisi2
	ldr	r2, [sp, #32]
	cmp	r2, r3
	mvncs	r3, #0
	strcs	r3, [sl, r4]
	mov	r8, r0
	bcs	.L1401
	ldrb	r1, [r5, #4025]	@ zero_extendqisi2
	add	r2, r9, r2, asl #4
	cmp	r1, #0
	ldr	r2, [r2, #2884]
	moveq	r8, #0
	cmp	r2, #0
	beq	.L1403
	cmp	r3, #1
	bne	.L1404
	ldr	r0, [r5, #2224]
	bl	NandcIqrWaitFlashReady
.L1404:
	ldrb	r0, [sp, #32]	@ zero_extendqisi2
	bl	FlashWaitCmdDone
.L1403:
	ldr	r2, [sp, #32]
	mov	r1, #0
	cmp	r8, #0
	add	r3, r5, r2, asl #4
	movne	r0, #36
	str	r1, [r3, #2888]
	ldr	r1, [sp, #28]
	str	r7, [r3, #2884]
	str	r1, [r3, #2880]
	addne	r1, r6, #1
	mlane	r1, r0, r1, sl
	strne	r1, [r3, #2888]
	add	r3, r9, r2
	add	r2, r9, r2, asl #4
	ldrb	r4, [r3, #3996]	@ zero_extendqisi2
	ldrb	r3, [r9, #4012]	@ zero_extendqisi2
	cmp	r3, #1
	strb	r4, [r2, #2876]
	mov	r0, r4
	bne	.L1406
	bl	NandcWaitFlashReady
	b	.L1407
.L1406:
	bl	NandcFlashCs
	ldr	r3, [sp, #32]
	mov	r0, r4
	ldr	r1, [sp, #28]
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #2800]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r4
	bl	NandcFlashDeCs
.L1407:
	ldr	r2, [sp, #16]
	sub	r3, r2, #1
	cmp	r3, #5
	bhi	.L1408
	add	r3, r5, r4
	ldrb	r3, [r3, #3984]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1408
	mov	r0, r4
	ldrb	r1, [r5, #3133]	@ zero_extendqisi2
	ldr	r2, .L1422+8
	mov	r3, #0
	bl	HynixSetRRPara
.L1408:
	mov	r0, r4
	bl	NandcFlashCs
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgFirstCmd
	ldr	r3, [r7, #12]
	mov	r0, r4
	mov	r1, #1
	ldr	r2, [sp, #8]
	str	r3, [sp, #0]
	ldr	r3, [r7, #8]
	bl	NandcXferData
	cmp	r8, #0
	beq	.L1409
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgDpFirstCmd
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	mov	r0, r4
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #2800]
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
	mla	r3, r2, r3, sl
	mov	r1, #1
	ldr	r2, [r3, #12]
	str	r2, [sp, #0]
	ldr	r2, [sp, #8]
	ldr	r3, [r3, #8]
	bl	NandcXferData
.L1409:
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgSecondCmd
	mov	r0, r4
	bl	NandcFlashDeCs
	add	r6, r6, r8
.L1401:
	add	r6, r6, #1
.L1399:
	cmp	r6, fp
	bcc	.L1410
	ldr	r4, .L1422
	ldr	r0, [r4, #2224]
	bl	NandcIqrWaitFlashReady
	ldr	r0, [r4, #2868]
	cmp	r0, #0
	beq	.L1411
	ldrb	r3, [r4, #4026]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1411
	bl	NandcIqrWaitFlashReady
.L1411:
	mov	r4, #0
	ldr	r5, .L1422
	b	.L1412
.L1413:
	uxtb	r0, r4
	add	r4, r4, #1
	bl	FlashWaitCmdDone
.L1412:
	ldrb	r3, [r5, #4012]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1413
	ldr	r3, [sp, #20]
	cmp	r3, #0
	beq	.L1414
	mov	r5, #0
	ldr	r4, .L1422+12
	ldr	r6, [sp, #12]
	b	.L1415
.L1420:
	ldr	r3, [sl, #0]
	cmn	r3, #1
	bne	.L1416
	ldr	r1, [sl, #4]
	ldr	r0, .L1422+16
	bl	printk
	b	.L1417
.L1416:
	rsb	r3, r5, fp
	mov	r1, r6
	add	r2, sp, #28
	mov	r0, sl
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldr	r2, [r4, #384]
	mov	r3, #0
	mov	lr, sl
	add	ip, sp, #36
	str	r3, [r2, #0]
	ldr	r2, [r4, #388]
	str	r3, [r2, #0]
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	ldmia	lr!, {r0, r1, r2, r3}
	stmia	ip!, {r0, r1, r2, r3}
	add	r0, sp, #36
	ldr	r3, [lr, #0]
	mov	r1, #1
	mov	r2, r6
	str	r3, [ip, #0]
	ldr	r3, [r4, #384]
	str	r3, [sp, #44]
	ldr	r3, [r4, #388]
	str	r3, [sp, #48]
	bl	FlashReadPages
	ldr	r7, [sp, #36]
	cmn	r7, #1
	bne	.L1418
	ldr	r0, .L1422+20
	ldr	r1, [sl, #4]
	bl	printk
	str	r7, [sl, #0]
.L1418:
	ldr	r3, [sl, #12]
	cmp	r3, #0
	beq	.L1419
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #388]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1419
	ldr	r0, .L1422+24
	ldr	r1, [sl, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [sl, #0]
.L1419:
	ldr	r3, [sl, #8]
	cmp	r3, #0
	beq	.L1417
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #384]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1417
	ldr	r0, .L1422+28
	ldr	r1, [sl, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [sl, #0]
.L1417:
	add	r5, r5, #1
	add	sl, sl, #36
.L1415:
	cmp	r5, fp
	bne	.L1420
.L1414:
	mov	r0, #0
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1423:
	.align	2
.L1422:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+3136
	.word	.LANCHOR2
	.word	.LC98
	.word	.LC99
	.word	.LC100
	.word	.LC101
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r6, #0
	ldr	r4, .L1433
	mov	r1, #255
	ldr	r7, .L1433+4
	movw	r8, #65535
	ldr	r3, [r4, #-1856]
	ldr	r5, [r4, #-720]
	str	r3, [r4, #408]
	ldr	r3, .L1433+8
	str	r5, [r4, #412]
	str	r6, [r5, #12]
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, .L1433+12
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #-1544]
	str	r6, [r5, #8]
	str	r3, [r5, #4]
	ldr	r3, .L1433+16
	str	r3, [r4, #-1604]
	ldr	r3, .L1433+20
	str	r3, [r4, #-1600]
	ldr	r3, .L1433+24
	ldrh	r2, [r4, r3]
	sub	r3, r3, #50
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #4086
	ldrh	r3, [r7, r3]
	strb	r3, [r4, #-1594]
	ldr	r3, .L1433+28
	ldrh	r2, [r4, r3]
	add	r3, r3, #234
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #232
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1818]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1433+32
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1816]	@ zero_extendqisi2
	strb	r3, [r4, #-1593]
	ldr	r3, .L1433+36
	ldrh	r2, [r4, r3]
	add	r3, r3, #190
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #188
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1770]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1433+40
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1768]	@ zero_extendqisi2
	strb	r3, [r4, #-1592]
	ldr	r3, .L1433+44
	ldrh	r2, [r4, r3]
	add	r3, r3, #146
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #144
	ldr	r0, [r4, #408]
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1722]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1433+48
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1720]	@ zero_extendqisi2
	strb	r3, [r4, #-1591]
	ldr	r3, [r4, #-1624]
	str	r3, [r4, #-1572]
	ldr	r3, [r4, #-1632]
	str	r3, [r4, #-1564]
	ldr	r3, [r4, #-1628]
	str	r3, [r4, #-1568]
	ldr	r3, .L1433+52
	ldrh	r2, [r4, r3]
	bl	ftl_memset
	ldr	r1, .L1433+56
	mov	r2, #48
	ldr	r0, [r4, #408]
	bl	memcpy
	movw	r3, #4072
	ldrh	r2, [r7, r3]
	ldr	r0, [r4, #408]
	ldr	r1, [r4, #-1844]
	mov	r2, r2, asl #1
	add	r0, r0, #48
	bl	memcpy
	mov	r0, r6
	bl	FtlUpdateVaildLpn
	ldr	r6, .L1433+60
.L1432:
	ldr	r3, [r4, #-1856]
	mov	r1, #1
	ldr	r7, .L1433+8
	ldr	r0, .L1433+64
	str	r3, [r4, #408]
	ldr	r3, [r4, #-720]
	ldrh	r2, [r4, r7]
	str	r3, [r4, #412]
	ldrh	r3, [r4, r6]
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r4, #404]
	mov	r3, r1
	bl	FlashProgPages
	ldr	r3, .L1433+68
	ldrh	r2, [r4, r6]
	ldrh	r3, [r4, r3]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L1426
	ldr	r3, .L1433+72
	ldrh	r2, [r4, r7]
	ldrh	r8, [r4, r3]
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r6]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r4, #-1632]
	mov	r1, #1
	add	r2, r3, #1
	str	r2, [r4, #-1632]
	str	r3, [r4, #-1544]
	mov	r2, r0, asl #10
	strh	r0, [r4, r7]	@ movhi
	str	r2, [r4, #404]
	mov	r2, r1
	str	r3, [r5, #4]
	mov	r3, r1
	strh	r0, [r5, #2]	@ movhi
	ldr	r0, .L1433+64
	bl	FlashProgPages
.L1426:
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #400]
	add	r3, r3, #1
	cmn	r2, #1
	uxth	r3, r3
	strh	r3, [r4, r6]	@ movhi
	bne	.L1427
	cmp	r3, #1
	ldreq	r3, .L1433+68
	ldreqh	r3, [r4, r3]
	subeq	r3, r3, #1
	streqh	r3, [r4, r6]	@ movhi
	b	.L1432
.L1427:
	cmp	r3, #1
	beq	.L1432
	movw	r3, #65535
	cmp	r8, r3
	beq	.L1429
	mov	r0, r8
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L1429:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1434:
	.align	2
.L1433:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1552
	.word	-3932
	.word	1179929683
	.word	1342177319
	.word	-1546
	.word	-1824
	.word	-1588
	.word	-1776
	.word	-1584
	.word	-1728
	.word	-1580
	.word	-4042
	.word	.LANCHOR2-1604
	.word	-1550
	.word	.LANCHOR2+400
	.word	-4050
	.word	-1548
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
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
	ldr	r4, .L1444
	mov	r5, #0
	ldr	r6, .L1444+4
	movw	r7, #4086
	ldr	r8, .L1444+8
	ldr	r3, [r4, #-720]
	mov	sl, r4
	ldr	r0, [r4, #-1856]
	str	r3, [r4, #412]
	ldr	r3, .L1444+12
	str	r0, [r4, #408]
	ldrh	r2, [r4, r3]
	bl	ftl_memset
	b	.L1436
.L1437:
	ldr	r3, .L1444+16
	ldr	r1, [r6, #4]!
	ldrh	r2, [sl, r3]
	ldr	r3, [sl, #408]
	mul	r0, r2, r5
	mov	r2, r2, asl #2
	add	r5, r5, #1
	add	r0, r3, r0, asl #2
	bl	memcpy
.L1436:
	ldrh	r3, [r8, r7]
	ldr	r4, .L1444
	cmp	r5, r3
	blt	.L1437
	ldr	r5, [r4, #412]
	mov	r1, #255
	mov	r2, #16
	mov	r6, #0
	mov	r9, r6
	mov	fp, r6
	mov	r0, r5
	bl	ftl_memset
	ldr	r3, .L1444+20
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #-3976]
	str	r3, [r5, #4]
	ldr	r3, .L1444+24
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, .L1444+28
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #8]	@ movhi
	ldr	r3, .L1444+32
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #10]	@ movhi
	ldr	r3, .L1444+8
	ldr	r3, [r3, #4060]
	strh	r3, [r5, #12]	@ movhi
	b	.L1443
.L1442:
	mov	r6, #1
.L1443:
	ldr	r3, [r4, #-1856]
	ldr	r7, .L1444+24
	ldr	r8, .L1444+36
	str	r3, [r4, #408]
	ldr	r3, [r4, #-720]
	ldrh	r1, [r4, r7]
	ldr	sl, .L1444+28
	str	r3, [r4, #412]
	ldrh	r0, [r5, #10]
	ldrh	r2, [r4, r8]
	str	fp, [r4, #400]
	orr	r3, r2, r1, asl #10
	str	r0, [sp, #0]
	str	r3, [r4, #404]
	ldr	r0, .L1444+40
	ldrh	r3, [r4, sl]
	bl	printk
	mov	r1, #1
	mov	r2, r1
	mov	r3, r1
	ldr	r0, .L1444+44
	bl	FlashProgPages
	ldr	r3, .L1444+48
	ldrh	r2, [r4, r8]
	ldrh	r3, [r4, r3]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L1439
	ldr	r3, [r4, #-3976]
	mov	r1, #1
	ldrh	r2, [r4, r7]
	add	r3, r3, #1
	ldr	r0, [r4, #-1868]
	str	r3, [r4, #-3976]
	str	r3, [r5, #4]
	ldrh	r3, [r4, sl]
	strh	r2, [r5, #8]	@ movhi
	strh	r2, [r4, sl]	@ movhi
	strh	r3, [r4, r7]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r4, #404]
	str	r3, [r0, #4]
	strh	r9, [r4, r8]	@ movhi
	bl	FlashEraseBlocks
	mov	r1, #1
	ldr	r0, .L1444+44
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
.L1439:
	ldr	r3, .L1444+36
	ldrh	r2, [r4, r3]
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #400]
	cmn	r3, #1
	bne	.L1440
	ldr	r1, [r4, #404]
	ldr	r0, .L1444+52
	bl	printk
	b	.L1443
.L1440:
	cmp	r6, #0
	beq	.L1442
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1445:
	.align	2
.L1444:
	.word	.LANCHOR2
	.word	.LANCHOR2-3960
	.word	.LANCHOR0
	.word	-4042
	.word	-800
	.word	-3887
	.word	-3984
	.word	-3980
	.word	-3978
	.word	-3982
	.word	.LC102
	.word	.LANCHOR2+400
	.word	-4050
	.word	.LC103
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
	ldr	r4, .L1456
	ldr	r3, .L1456+4
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	movne	r6, #0
	movne	sl, r4
	bne	.L1447
	b	.L1448
.L1454:
	add	r3, r3, r6
	mov	r1, r9
	mov	r5, #0
	ldr	r7, .L1456+4
	ldrb	r0, [r3, #4090]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	b	.L1449
.L1453:
	add	r3, r4, r5, asl #1
	sub	r3, r3, #916
	ldrh	r3, [r3, #0]
	cmp	r3, r8
	bne	.L1450
	mov	r1, r8
	ldr	r0, .L1456+8
	bl	printk
	mov	r0, r8
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r2, [r4, r7]
	mov	r3, r5
	b	.L1451
.L1452:
	add	r1, r3, #1
	add	r3, r4, r3, asl #1
	sub	r3, r3, #916
	add	r0, r4, r1, asl #1
	sub	r0, r0, #916
	ldrh	r0, [r0, #0]
	strh	r0, [r3, #0]	@ movhi
	uxth	r3, r1
.L1451:
	cmp	r3, r2
	bcc	.L1452
	sub	r2, r2, #1
	strh	r2, [sl, r7]	@ movhi
.L1450:
	add	r5, r5, #1
	uxth	r5, r5
.L1449:
	ldrh	r3, [r4, r7]
	cmp	r3, r5
	bhi	.L1453
	add	r6, r6, #1
	uxth	r6, r6
.L1447:
	ldr	r3, .L1456+12
	mov	r2, #4064
	ldrh	r2, [r3, r2]
	cmp	r2, r6
	bhi	.L1454
	bl	FtlGcReFreshBadBlk
.L1448:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1457:
	.align	2
.L1456:
	.word	.LANCHOR2
	.word	-918
	.word	.LC104
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
	ldr	r3, .L1466
	mov	r4, r0
	ldr	r1, [r3, #-1844]
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	bne	.L1459
	ldr	r1, .L1466+4
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	mvneq	r2, #0
	streqh	r2, [r3, r1]	@ movhi
	beq	.L1461
	ldr	r1, .L1466+8
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1462
	ldr	r1, .L1466+12
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1462
	ldr	r1, .L1466+16
	ldrh	r3, [r3, r1]
	cmp	r3, r4
	beq	.L1462
.L1461:
	mov	r1, r4
	ldr	r0, .L1466+20
	bl	List_remove_node
	ldr	r5, .L1466
	ldr	r3, .L1466+24
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	free_data_superblock
	mov	r0, r4
	bl	FtlGcFreeBadSuperBlk
	mov	r2, #1
	b	.L1462
.L1459:
	bl	List_update_data_list
	mov	r2, #0
.L1462:
	mov	r0, r2
	ldmfd	sp!, {r3, r4, r5, pc}
.L1467:
	.align	2
.L1466:
	.word	.LANCHOR2
	.word	-1540
	.word	-1824
	.word	-1776
	.word	-1728
	.word	.LANCHOR2-1848
	.word	-1836
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
	beq	.L1469
	ldr	r2, .L1474
	mov	r3, r0, asl #1
	ldr	r2, [r2, #-1844]
	ldrh	r4, [r2, r3]
	cmp	r4, #0
	subne	r4, r4, #1
	strneh	r4, [r2, r3]	@ movhi
	bne	.L1469
	ldr	r0, .L1474+4
	mov	r1, r5
	mov	r2, r4
	bl	printk
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1469:
	ldr	r7, .L1474
	movw	r3, #65535
	ldr	r6, .L1474+8
	ldrh	r0, [r7, r6]
	cmp	r0, r3
	streqh	r5, [r7, r6]	@ movhi
	moveq	r0, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r0, r5
	beq	.L1473
	bl	update_vpc_list
	strh	r5, [r7, r6]	@ movhi
	adds	r0, r0, #0
	movne	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1473:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1475:
	.align	2
.L1474:
	.word	.LANCHOR2
	.word	.LC105
	.word	-792
	.fnend
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	get_new_active_ppa
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r3, #0
	strb	r3, [r0, #10]
	mov	r4, r0
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	movw	r5, #65535
	ldr	r8, .L1486
	ldr	r7, .L1486+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1477
.L1478:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	mov	r2, #4064
	ldrh	r2, [r7, r2]
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
.L1477:
	cmp	r3, r5
	beq	.L1478
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L1480
	ldrh	r2, [r4, #2]
	add	r2, r8, r2, asl #1
	sub	r2, r2, #680
	ldrh	r2, [r2, #0]
	cmp	r2, r5
	bne	.L1480
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1478
.L1480:
	ldrh	r6, [r4, #2]
	movw	r5, #65535
	mov	r7, r5
	orr	r6, r6, r3, asl #10
	ldrh	r3, [r4, #4]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
.L1484:
	ldr	r2, .L1486+4
	mov	r3, #4064
	ldrh	r2, [r2, r3]
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
.L1482:
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
	beq	.L1482
	strb	r3, [r4, #6]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1483
	ldrh	r2, [r4, #2]
	ldr	r3, .L1486
	add	r3, r3, r2, asl #1
	sub	r3, r3, #680
	ldrh	r3, [r3, #0]
	cmp	r3, r7
	bne	.L1483
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1483
	sub	r3, r3, #1
	ldrh	r0, [r4, #0]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1484
.L1483:
	mov	r0, r6
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1487:
	.align	2
.L1486:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.type	FtlSlcSuperblockCheck.part.14, %function
FtlSlcSuperblockCheck.part.14:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	movw	r5, #65535
	ldr	r7, .L1493
	ldr	r6, .L1493+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1489
.L1490:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	mov	r2, #4064
	ldrh	r2, [r6, r2]
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
.L1489:
	cmp	r3, r5
	beq	.L1490
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	ldrh	r3, [r4, #2]
	add	r3, r7, r3, asl #1
	sub	r3, r3, #680
	ldrh	r3, [r3, #0]
	cmp	r3, r5
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1490
	ldrh	r2, [r4, #2]
	strb	r3, [r4, #6]
	add	r2, r2, #1
	strh	r2, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1494:
	.align	2
.L1493:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlSlcSuperblockCheck.part.14, .-FtlSlcSuperblockCheck.part.14
	.align	2
	.global	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrh	r3, [r0, #4]
	cmp	r3, #0
	bxeq	lr
	b	FtlSlcSuperblockCheck.part.14
	.fnend
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
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
	ldr	r4, .L1529
	mov	r5, r0
	ldr	r8, .L1529+4
.L1526:
	ldr	r3, .L1529+8
	cmp	r5, r3
	bne	.L1499
	ldrh	r2, [r4, r8]
	ldr	r3, [r4, #-964]
	mov	r1, r2, lsr #1
	mul	ip, r3, r2
	add	r0, r1, #1
	add	r0, r0, ip, lsr #2
	ldr	ip, [r4, #-1668]
	cmp	ip, #0
	uxth	r0, r0
	beq	.L1521
	ldr	ip, [r4, #-1608]
	cmp	ip, #29
	bhi	.L1521
	cmp	ip, #2
	bls	.L1525
	tst	r2, #1
	beq	.L1500
	cmp	r3, #0
	moveq	r1, r3
	beq	.L1501
	b	.L1500
.L1499:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1525
	ldr	r2, .L1529+12
	movw	r3, #4084
	ldrh	r3, [r2, r3]
	cmp	r3, #1
	beq	.L1525
	ldr	r2, [r4, #-1668]
	ldrh	r3, [r4, r8]
	cmp	r2, #0
	mov	r1, r3, lsr #3
	beq	.L1500
	ldr	r2, [r4, #-1608]
	cmp	r2, #1
	movls	r1, #7
	mulls	r1, r1, r3
	movls	r1, r1, lsr #3
	b	.L1500
.L1521:
	mov	r1, r0
.L1500:
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
	b	.L1501
.L1525:
	mov	r1, #0
.L1501:
	ldr	r0, .L1529+16
	bl	List_pop_index_node
	ldrh	r3, [r4, r8]
	sub	r3, r3, #1
	strh	r3, [r4, r8]	@ movhi
	uxth	r6, r0
	mov	r0, r5
	strh	r6, [r5, #0]	@ movhi
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1502
	ldr	r2, .L1529+12
	mov	r3, #4064
	ldr	ip, [r4, #-1868]
	mov	r7, #0
	mov	sl, r5
	mov	r1, r5
	ldrh	r9, [r2, r3]
	mov	r0, r7
	mov	r3, ip
	mov	r2, r7
	str	ip, [sp, #12]
	str	r5, [sp, #4]
	b	.L1503
.L1502:
	ldr	r2, [r4, #-1844]
	mov	r3, r6, asl #1
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
	b	.L1527
.L1506:
	str	r0, [r3, #8]
	movw	r5, #65535
	str	r0, [r3, #12]
	ldrh	lr, [r1, #16]
	cmp	lr, r5
	beq	.L1505
	ldr	r5, [sp, #12]
	mov	ip, #36
	mov	lr, lr, asl #10
	mla	fp, ip, r7, r5
	add	r7, r7, #1
	uxth	r7, r7
	str	lr, [fp, #4]
.L1505:
	add	r2, r2, #1
	add	r3, r3, #36
	add	r1, r1, #2
	uxth	r2, r2
.L1503:
	cmp	r2, r9
	bne	.L1506
	ldr	r3, [r4, #-1668]
	ldr	r5, [sp, #4]
	cmp	r3, #0
	beq	.L1507
	ldr	r3, .L1529+20
	cmp	r5, r3
	bne	.L1507
	ldr	r2, [r4, #-1864]
	mov	r3, r6, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #30
	movhi	r3, #0
	strhib	r3, [r4, #-1816]
.L1507:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r2, [r4, #-1864]
	cmp	r3, #0
	mov	r3, r6, asl #1
	ldrh	r1, [r2, r3]
	bne	.L1508
	cmp	r1, #0
	ldrne	r0, .L1529+24
	moveq	r1, #2
	ldrneh	r0, [r4, r0]
	addne	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r4, #-1624]
	add	r3, r3, #1
	str	r3, [r4, #-1624]
	b	.L1511
.L1508:
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r4, #-1620]
	add	r3, r3, #1
	str	r3, [r4, #-1620]
.L1511:
	ldr	r3, [r4, #-1864]
	mov	fp, r6, asl #1
	ldr	r2, [r4, #-1612]
	ldr	r9, .L1529
	ldrh	r3, [r3, fp]
	cmp	r3, r2
	ldr	r0, [r9, #-1624]
	strhi	r3, [r4, #-1612]
	ldr	r3, .L1529+24
	ldrh	r2, [r9, r3]
	ldr	r3, [r9, #-1620]
	mla	r0, r0, r2, r3
	ldr	r2, .L1529+12
	movw	r3, #4072
	ldrh	r1, [r2, r3]
	bl	__aeabi_uidiv
	ldr	r3, [r9, #-704]
	ldr	r2, [r3, #16]
	add	r2, r2, #1
	str	r2, [r3, #16]
	ldr	r3, [r9, #-1868]
	str	r0, [r9, #-1616]
	mov	r9, #0
	b	.L1513
.L1514:
	add	r9, r9, #1
	ldr	r2, [r3, #-32]
	uxth	r9, r9
	bic	r2, r2, #1020
	bic	r2, r2, #3
	str	r2, [r3, #-32]
.L1513:
	cmp	r9, r7
	add	r3, r3, #36
	bne	.L1514
	ldr	r0, [r4, #-1868]
	mov	r1, r9
	bl	FlashEraseBlocks
	mov	r2, #36
	mul	r2, r2, r9
	mov	r7, #0
	mov	ip, r6
	str	fp, [sp, #12]
	mov	r6, r5
	mov	r5, r7
	mov	fp, r2
	b	.L1515
.L1517:
	ldr	r1, [r4, #-1868]
	add	r0, r1, r7
	ldr	r9, [r1, r7]
	cmn	r9, #1
	bne	.L1516
	ldr	r0, [r0, #4]
	add	r5, r5, #1
	str	ip, [sp, #8]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	strh	r9, [sl, #16]	@ movhi
	ldrb	r1, [r6, #7]	@ zero_extendqisi2
	sub	r1, r1, #1
	strb	r1, [r6, #7]
	ldr	ip, [sp, #8]
.L1516:
	add	r7, r7, #36
	add	sl, sl, #2
.L1515:
	cmp	r7, fp
	ldr	r9, .L1529
	bne	.L1517
	mov	r3, r5
	cmp	r3, #0
	mov	r5, r6
	ldr	fp, [sp, #12]
	mov	r6, ip
	beq	.L1518
	mov	r0, ip
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1518:
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1519
	ldr	r3, [r9, #-1844]
	mvn	r2, #0
	strh	r2, [r3, fp]	@ movhi
.L1527:
	mov	r0, r6
	bl	INSERT_DATA_LIST
	b	.L1526
.L1519:
	ldr	r2, .L1529+28
	mov	r0, #0
	strh	r0, [r5, #2]	@ movhi
	strb	r0, [r5, #6]
	ldrh	r2, [r9, r2]
	strh	r6, [r5, #0]	@ movhi
	ldr	r1, [r9, #-1844]
	mul	r3, r2, r3
	ldr	r2, [r9, #-1632]
	str	r2, [r5, #12]
	add	r2, r2, #1
	str	r2, [r9, #-1632]
	ldrh	r2, [r5, #0]
	uxth	r3, r3
	strh	r3, [r5, #4]	@ movhi
	mov	r2, r2, asl #1
	strh	r3, [r1, r2]	@ movhi
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1530:
	.align	2
.L1529:
	.word	.LANCHOR2
	.word	-1828
	.word	.LANCHOR2-1728
	.word	.LANCHOR0
	.word	.LANCHOR2-1832
	.word	.LANCHOR2-1824
	.word	-4062
	.word	-4052
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, #0
	ldr	r8, .L1559
	.pad #20
	sub	sp, sp, #20
	mov	sl, r0
	mov	r7, r1
	mov	r6, r4
	mov	r5, r4
	str	r0, [r8, #-3988]
	mov	r9, #36
	ldr	fp, .L1559+4
	b	.L1532
.L1536:
	mul	r3, r9, r4
	ldr	r2, [r8, #-1868]
	mov	r1, #0
	str	r1, [r2, r3]
	add	r3, fp, r4
	mov	r1, sl
	ldrb	r0, [r3, #4090]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r7, #0
	str	r0, [sp, #4]
	beq	.L1533
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1534
.L1533:
	ldr	r0, [sp, #4]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r5, r5, #1
	uxthne	r5, r5
	bne	.L1534
	ldr	r3, [r8, #-1868]
	ldr	r1, [sp, #4]
	mla	r3, r9, r6, r3
	mov	r2, r1, asl #10
	ldr	r1, .L1559+8
	str	r2, [r3, #4]
	ldrh	r2, [r8, r1]
	str	r0, [r3, #8]
	mul	r2, r2, r6
	add	r6, r6, #1
	uxth	r6, r6
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [r8, #-716]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L1534:
	add	r4, r4, #1
	uxth	r4, r4
.L1532:
	mov	r2, #4064
	ldrh	r3, [fp, r2]
	cmp	r3, r4
	bhi	.L1536
	cmp	r6, #0
	beq	.L1538
	ldr	r4, .L1559
	mov	r1, r6
	mov	r8, #0
	mov	r9, r8
	ldr	r0, [r4, #-1868]
	bl	FlashEraseBlocks
.L1540:
	ldr	r3, [r4, #-1868]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L1539
	ldr	r0, [r2, #4]
	add	r5, r5, #1
	ubfx	r0, r0, #10, #16
	uxth	r5, r5
	bl	FtlBbmMapBadBlock
.L1539:
	add	r9, r9, #1
	add	r8, r8, #36
	uxth	r9, r9
	cmp	r9, r6
	bne	.L1540
	ldr	r2, .L1559
	ldr	r3, [r2, #-1668]
	cmp	r3, #0
	streq	r3, [sp, #4]
	moveq	r1, #6
	streq	r1, [sp, #12]
	beq	.L1558
	cmp	r7, #0
	streq	r7, [sp, #4]
	moveq	r2, #6
	streq	r2, [sp, #12]
	beq	.L1558
	ldr	r3, .L1559+12
	mov	r1, #5
	ldrh	r8, [r2, r3]
	mov	r0, r8
	bl	__aeabi_uidiv
	mov	r3, #1
	str	r3, [sp, #4]
	sub	r0, r0, #1
	uxth	r0, r0
	str	r0, [sp, #12]
	b	.L1541
.L1558:
	mov	r8, #1
.L1541:
	mov	r6, #0
.L1551:
	mov	r9, #0
	ldr	fp, .L1559
	mov	r4, r9
	b	.L1542
.L1545:
	mov	r1, #36
	ldr	r2, [fp, #-1868]
	mul	r3, r1, r9
	mov	r1, #0
	str	r1, [r2, r3]
	mov	r1, sl
	ldr	r2, .L1559+4
	add	r3, r2, r9
	ldrb	r0, [r3, #4090]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r7, #0
	str	r0, [sp, #8]
	beq	.L1543
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1544
.L1543:
	ldr	r0, [sp, #8]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L1544
	ldr	r3, [fp, #-1868]
	mov	r1, #36
	mla	r3, r1, r4, r3
	ldr	r1, [sp, #8]
	add	r2, r6, r1, asl #10
	ldr	r1, .L1559+8
	str	r2, [r3, #4]
	ldr	r2, [fp, #-728]
	str	r2, [r3, #8]
	ldrh	r2, [fp, r1]
	mul	r2, r2, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [fp, #-724]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L1544:
	add	r9, r9, #1
	uxth	r9, r9
.L1542:
	ldr	r1, .L1559+4
	mov	r2, #4064
	ldrh	r3, [r1, r2]
	cmp	r3, r9
	bhi	.L1545
	cmp	r4, #0
	beq	.L1538
	ldr	r9, .L1559
	mov	r3, #1
	mov	r1, r4
	ldr	r2, [sp, #4]
	mov	fp, #0
	ldr	r0, [r9, #-1868]
	bl	FlashProgPages
	mov	r3, #36
	mul	r3, r3, r4
	str	r3, [sp, #8]
	b	.L1547
.L1550:
	ldr	r2, [r9, #-1868]
	add	r3, r2, fp
	ldr	r2, [r2, fp]
	cmn	r2, #1
	bne	.L1548
	ldr	r0, [r3, #4]
	add	r5, r5, #1
	ubfx	r0, r0, #10, #16
	uxth	r5, r5
	bl	FtlBbmMapBadBlock
	b	.L1549
.L1548:
	cmp	r7, #0
	beq	.L1549
	ldr	r0, [r3, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L1549:
	add	fp, fp, #36
.L1547:
	ldr	r2, [sp, #8]
	cmp	fp, r2
	bne	.L1550
	ldr	r3, [sp, #12]
	add	r6, r6, r3
	uxth	r6, r6
	cmp	r6, r8
	bcc	.L1551
	cmp	sl, #63
	bhi	.L1538
	ldr	r3, .L1559
	mov	r1, r4
	ldr	r0, [r3, #-1868]
	bl	FlashEraseBlocks
.L1538:
	mov	r0, r5
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1560:
	.align	2
.L1559:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-4040
	.word	-4050
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.type	FlashTestBlk.part.16, %function
FlashTestBlk.part.16:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
.L1562:
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #108
	sub	sp, sp, #108
	ldr	r4, .L1564
	mov	r5, r0
	mov	r1, #165
	add	r0, sp, #40
	mov	r2, #32
	str	r0, [sp, #16]
	ldr	r3, [r4, #348]
	mov	r5, r5, asl #10
	str	r3, [sp, #12]
	bl	ftl_memset
	mov	r2, #8
	ldr	r0, [r4, #348]
	mov	r1, #90
	bl	ftl_memset
	add	r0, sp, #4
	mov	r1, #1
	str	r5, [sp, #8]
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r2, r1
	mov	r3, r1
	add	r0, sp, #4
	bl	FlashProgPages
	ldr	r4, [sp, #4]
	add	r0, sp, #4
	mov	r1, #1
	cmn	r4, #1
	movne	r4, #0
	bl	FlashEraseBlocks
	mov	r0, r4
	add	sp, sp, #108
	ldmfd	sp!, {r4, r5, pc}
.L1565:
	.align	2
.L1564:
	.word	.LANCHOR2
	.fnend
	.size	FlashTestBlk.part.16, .-FlashTestBlk.part.16
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1568
	ldr	r3, [r3, #364]
	cmp	r0, r3
	bcc	.L1567
	b	FlashTestBlk.part.16
.L1567:
	mov	r0, #0
	bx	lr
.L1569:
	.align	2
.L1568:
	.word	.LANCHOR2
	.fnend
	.size	FlashTestBlk, .-FlashTestBlk
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
	ldr	r4, .L1595
	movw	r1, #2274
	ldr	r3, .L1595+4
	mov	r5, #0
	ldr	r0, .L1595+8
	ldr	r2, [r4, #380]
	ldrh	fp, [r3, r1]
	mov	r1, #1
	str	r2, [sp, #24]
	mov	r2, #2272
	ldrh	r2, [r3, r2]
	mul	fp, fp, r2
	ldr	r2, [r3, #2864]
	ldrb	r2, [r2, #24]	@ zero_extendqisi2
	uxth	fp, fp
	str	r2, [sp, #12]
	ldrh	r2, [r3, #4]
	ldrb	r3, [r3, #0]	@ zero_extendqisi2
	cmp	r3, #1
	str	r2, [sp, #8]
	moveq	r3, r2, asl #1
	uxtheq	r3, r3
	streq	r3, [sp, #8]
	bl	printk
	ldr	r0, [r4, #380]
	mov	r1, #0
	mov	r2, #4096
	bl	ftl_memset
	mov	r3, fp, lsr #4
	str	r3, [sp, #20]
	sub	r3, fp, #1
	ldr	r4, .L1595+4
	uxth	r3, r3
	str	r3, [sp, #28]
	b	.L1572
.L1591:
	ldr	r3, .L1595
	add	r3, r3, r5, asl #1
	add	r3, r3, #440
	ldrh	r6, [r3, #0]
	cmp	r6, #0
	bne	.L1573
	movw	r3, #2280
	ldr	r0, [r4, #4016]
	ldrh	r2, [r4, r3]
	mov	r1, r6
	mov	r8, r6
	mov	r7, r6
	add	sl, r4, r5, asl #2
	mov	r2, r2, asl #9
	bl	ftl_memset
	add	r3, r4, r5
	ldrb	r9, [r3, #3996]	@ zero_extendqisi2
	b	.L1574
.L1582:
	ldr	r2, [sp, #12]
	mvn	r3, #0
	strb	r3, [sp, #34]
	tst	r2, #1
	strb	r3, [sp, #35]
	beq	.L1575
	ldr	r3, [sl, #2800]
	add	r2, sp, #34
	mov	r0, r9
	add	r3, r6, r3
	str	r3, [sp, #4]
	mov	r1, r3
	bl	FlashReadSpare
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	ldr	r3, [sp, #4]
	cmp	r2, #1
	bne	.L1575
	ldr	r1, [r4, #4]
	add	r2, sp, #35
	mov	r0, r9
	add	r1, r3, r1
	bl	FlashReadSpare
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	ldrb	r2, [sp, #35]	@ zero_extendqisi2
	and	r3, r2, r3
	strb	r3, [sp, #34]
.L1575:
	ldr	r3, [sp, #12]
	tst	r3, #2
	beq	.L1576
	ldr	r3, [r4, #2864]
	mov	r0, r9
	add	r2, sp, #35
	ldrh	r1, [r3, #10]
	ldr	r3, [sl, #2800]
	sub	r1, r1, #1
	add	r1, r1, r3
	add	r1, r1, r6
	bl	FlashReadSpare
.L1576:
	ldr	r3, [r4, #2864]
	ldrb	r3, [r3, #7]	@ zero_extendqisi2
	cmp	r3, #1
	cmpne	r3, #8
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	bne	.L1577
	cmp	r3, #0
	beq	.L1593
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	rsbs	r0, r0, #1
	movcc	r0, #0
	b	.L1578
.L1577:
	cmp	r3, #255
	bne	.L1593
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	subs	r0, r0, #255
	movne	r0, #1
	b	.L1578
.L1593:
	mov	r0, #1
.L1578:
	ldr	r2, [sp, #12]
	tst	r2, #4
	beq	.L1579
	ldr	r1, [sl, #2800]
	mov	r0, r9
	add	r1, r6, r1
	bl	SandiskProgTestBadBlock
.L1579:
	cmp	r0, #0
	beq	.L1580
	mov	r1, r5
	mov	r2, r8
	ldr	r0, .L1595+12
	add	r7, r7, #1
	bl	printk
	ldr	r3, [sp, #16]
	and	r0, r8, #31
	mov	ip, #1
	uxth	r7, r7
	mov	r2, r3, lsr #5
	ldr	r3, [r4, #4016]
	ldr	r1, [r3, r2, asl #2]
	orr	r1, r1, ip, asl r0
	str	r1, [r3, r2, asl #2]
	ldr	r2, [sp, #20]
	ldrb	r3, [r4, #4012]	@ zero_extendqisi2
	mul	r3, r3, r2
	cmp	r7, r3
	bgt	.L1581
.L1580:
	ldr	r3, [sp, #8]
	add	r8, r8, #1
	add	r6, r6, r3
.L1574:
	uxth	r3, r8
	str	r3, [sp, #16]
	cmp	r3, fp
	bcc	.L1582
.L1581:
	mov	r2, r7
	ldr	r0, .L1595+16
	mov	r1, r5
	bl	printk
	ldrb	r3, [r4, #4012]	@ zero_extendqisi2
	ldr	r2, [sp, #20]
	mul	r3, r3, r2
	cmp	r7, r3
	blt	.L1583
	movw	r3, #2280
	ldr	r0, [r4, #4016]
	ldrh	r2, [r4, r3]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1583:
	cmp	r5, #0
	bne	.L1584
	ldr	r3, .L1595
	mov	r8, r5
	mov	r7, r5
	mov	sl, #1
	mov	r6, r5
	ldr	r9, [r3, #364]
	uxth	r9, r9
	b	.L1585
.L1587:
	mov	r0, r5
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1586
	mov	r1, r5
	ldr	r0, .L1595+20
	bl	printk
	ldr	r3, [r4, #4016]
	mov	r2, r5, lsr #5
	add	r7, r7, #1
	and	r5, r5, #31
	ldr	r1, [r3, r2, asl #2]
	uxth	r7, r7
	orr	r5, r1, sl, asl r5
	str	r5, [r3, r2, asl #2]
.L1586:
	add	r8, r8, #1
	uxth	r8, r8
.L1585:
	add	r5, r8, r9
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	ldr	r3, .L1595+4
	uxth	r5, r5
	cmp	r5, r2
	bcc	.L1587
	ldr	r1, .L1595
	mov	r5, r6
	ldr	r1, [r1, #364]
	rsb	r2, r1, r2
	cmp	r7, r2
	bcc	.L1584
	movw	r2, #2280
	ldr	r0, [r3, #4016]
	ldrh	r2, [r3, r2]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1584:
	mul	r8, fp, r5
	ldr	r7, .L1595
	ldr	r6, [sp, #28]
	ldr	r9, [sp, #24]
	add	sl, r7, r5, asl #1
.L1588:
	ldr	r0, .L1595+24
	mov	r1, r5
	mov	r2, r6
	bl	printk
	ldr	r3, [r4, #4016]
	b	.L1589
.L1590:
	sub	r6, r6, #1
	uxth	r6, r6
.L1589:
	mov	r1, r6, lsr #5
	and	r2, r6, #31
	ldr	r1, [r3, r1, asl #2]
	mov	r2, r1, lsr r2
	ands	r2, r2, #1
	bne	.L1590
	add	r3, sl, #440
	mov	r1, #1
	add	r0, sp, #36
	strh	r6, [r3, #0]	@ movhi
	ldr	r3, .L1595+28
	strh	r6, [r9, #2]	@ movhi
	strh	r2, [r9, #8]	@ movhi
	strh	r3, [r9, #0]	@ movhi
	ldr	r3, [r4, #4016]
	str	r3, [sp, #44]
	ldr	r3, [r7, #380]
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
	bne	.L1588
.L1573:
	add	r5, r5, #1
	uxtb	r5, r5
.L1572:
	ldrb	r3, [r4, #4012]	@ zero_extendqisi2
	cmp	r3, r5
	bhi	.L1591
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1596:
	.align	2
.L1595:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC106
	.word	.LC107
	.word	.LC108
	.word	.LC109
	.word	.LC110
	.word	-3872
	.fnend
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
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
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L1598
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r7, [r5, r0]
	cmp	r7, #0
	beq	.L1598
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L1598
	mov	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldr	r1, .L1605
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, .L1605+4
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1599
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1599:
	mov	r5, #0
	ldr	r6, .L1605
	mov	r9, r5
	b	.L1600
.L1603:
	ldr	r3, [r8, r5, asl #2]
	mov	sl, r5, asl #2
	cmp	r7, r3, lsr #10
	bne	.L1601
	ldr	r3, [r6, #-740]
	mov	r1, #1
	ldr	r0, .L1605+8
	mov	r2, r1
	str	r3, [r6, #408]
	ldr	r3, [r6, #-720]
	str	r3, [r6, #412]
	ldr	r3, [r8, r5, asl #2]
	str	r3, [r6, #404]
	bl	FlashReadPages
	ldr	r3, [r6, #400]
	cmn	r3, #1
	streq	r9, [r8, sl]
	beq	.L1601
	mov	r0, r4
	mov	r1, r5
	ldr	r2, [r6, #408]
	bl	FtlMapWritePage
.L1601:
	add	r5, r5, #1
	uxth	r5, r5
.L1600:
	ldrh	r3, [r4, #6]
	cmp	r3, r5
	bhi	.L1603
	mov	r0, r7
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L1598:
	ldr	r3, .L1605+4
	ldr	r1, .L1605
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1604
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1604:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1606:
	.align	2
.L1605:
	.word	.LANCHOR2
	.word	-4050
	.word	.LANCHOR2+400
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
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
	bne	.L1608
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
	b	.L1609
.L1608:
	mov	r3, r3, asl #1
	ldr	r2, [r0, #28]
	mov	r1, #255
	ldrh	r6, [r5, r3]
	ldr	r5, .L1613
	ldrh	r3, [r0, #2]
	orr	r3, r3, r6, asl #10
	str	r3, [r5, #404]
	ldr	r3, [r5, #-1856]
	str	r3, [r5, #408]
	ldr	r3, [r5, #-720]
	str	r3, [r5, #412]
	str	r2, [r3, #4]
	ldr	r2, .L1613+4
	strh	r2, [r3, #8]	@ movhi
	ldrh	r2, [r0, #4]
	strh	r6, [r3, #2]	@ movhi
	strh	r2, [r3, #0]	@ movhi
	ldr	r3, .L1613+8
	ldr	r0, [r5, #-1856]
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #3
	bl	ftl_memset
	mov	r2, #0
	mov	r3, r2
	b	.L1610
.L1612:
	ldr	r1, [r7, r3, asl #2]
	cmp	r6, r1, lsr #10
	bne	.L1611
	add	r2, r2, #1
	ldr	r1, [r5, #-1856]
	uxth	r2, r2
	str	r3, [r1, r2, asl #3]
	ldr	r1, [r5, #-1856]
	ldr	r0, [r7, r3, asl #2]
	add	r1, r1, r2, asl #3
	str	r0, [r1, #4]
.L1611:
	add	r3, r3, #1
	uxth	r3, r3
.L1610:
	ldrh	r1, [r4, #6]
	cmp	r1, r3
	bhi	.L1612
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L1613+12
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L1609:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1614:
	.align	2
.L1613:
	.word	.LANCHOR2
	.word	-1291
	.word	-4050
	.word	.LANCHOR2+400
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
	ldr	r5, .L1622
	mov	r6, r1
	ldr	r9, .L1622+4
	mov	r8, r2
	movw	fp, #65535
.L1621:
	ldr	r3, [r5, #-1648]
	add	r3, r3, #1
	str	r3, [r5, #-1648]
	ldrh	r3, [r5, r9]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L1616
	ldrh	r3, [r4, #0]
	cmp	r3, fp
	bne	.L1617
.L1616:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L1617:
	ldrh	r3, [r4, #0]
	mov	r1, #0
	ldr	r2, [r4, #12]
	ldr	r0, [r5, #-720]
	mov	r3, r3, asl #1
	ldr	r7, .L1622
	ldrh	sl, [r2, r3]
	mov	r2, #16
	ldrh	r3, [r4, #2]
	str	r8, [r5, #408]
	orr	r3, r3, sl, asl #10
	str	r0, [r5, #412]
	str	r3, [r5, #404]
	bl	ftl_memset
	ldr	r3, [r5, #412]
	mov	r1, #1
	ldr	r2, [r4, #28]
	add	r0, r7, #400
	strh	r6, [r3, #8]	@ movhi
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
	beq	.L1621
	ldr	r3, [r5, #400]
	cmn	r3, #1
	beq	.L1621
	ldr	r2, [r7, #404]
	mov	r0, #0
	ldr	r3, [r4, #24]
	str	r2, [r3, r6, asl #2]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1623:
	.align	2
.L1622:
	.word	.LANCHOR2
	.word	-4050
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
	ldr	r5, .L1625
	mul	r4, r4, r0
	sub	r0, r5, #856
	ldr	r2, [r5, #-1680]
	add	r3, r2, r4
	ldrh	r1, [r2, r4]
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #-1680]
	mov	r0, #0
	add	r4, r3, r4
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1626:
	.align	2
.L1625:
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
	ldr	r3, .L1636
	mov	r4, r1
	ldr	r2, .L1636+4
	ldr	sl, [r3, #-1680]
	ldrh	r7, [r3, r2]
	mvn	r2, #0
	add	r7, r7, #7
	mov	r6, r0, lsr r7
	bic	r7, r0, r2, asl r7
	ldr	r2, .L1636+8
	uxth	r6, r6
	uxth	r7, r7
	ldrh	r2, [r3, r2]
	mov	r3, #0
	mov	r5, r3
	b	.L1628
.L1634:
	add	r3, r3, #12
	add	r1, sl, r3
	ldrh	r1, [r1, #-12]
	cmp	r1, r6
	bne	.L1629
.L1630:
	cmp	r8, #0
	ldr	r3, .L1636
	mov	r2, #12
	bne	.L1631
	ldr	r3, [r3, #-1680]
	mla	r2, r2, r5, r3
	ldr	r3, [r2, #8]
	ldr	r3, [r3, r7, asl #2]
	str	r3, [r4, #0]
	b	.L1632
.L1631:
	mul	r2, r2, r5
	ldr	r1, [r3, #-1680]
	ldr	r0, [r4, #0]
	add	r1, r1, r2
	ldr	r1, [r1, #8]
	str	r0, [r1, r7, asl #2]
	ldr	r1, [r3, #-1680]
	add	r2, r1, r2
	ldr	r1, [r2, #4]
	orr	r1, r1, #-2147483648
	str	r1, [r2, #4]
	ldr	r2, .L1636+12
	strh	r6, [r3, r2]	@ movhi
.L1632:
	ldr	r3, .L1636
	mov	r2, #12
	mov	r0, #0
	ldr	r3, [r3, #-1680]
	mla	r5, r2, r5, r3
	ldr	r3, [r5, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r5, #4]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1629:
	add	r5, r5, #1
	uxth	r5, r5
.L1628:
	cmp	r5, r2
	bne	.L1634
	bl	select_l2p_ram_region
	mov	r3, #12
	mul	r3, r3, r0
	mov	r5, r0
	add	r2, sl, r3
	ldrh	r1, [sl, r3]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1635
	ldr	r3, [r2, #4]
	cmp	r3, #0
	bge	.L1635
	bl	flush_l2p_region
.L1635:
	mov	r0, r6
	mov	r1, r5
	bl	load_l2p_region
	b	.L1630
.L1637:
	.align	2
.L1636:
	.word	.LANCHOR2
	.word	-4044
	.word	-4014
	.word	-1676
	.fnend
	.size	log2phys, .-log2phys
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
	ldr	r3, .L1644
	ldr	r1, [r3, #-1844]
	mov	r5, r0, asl #1
	ldrh	r2, [r1, r5]
	cmp	r2, #0
	addne	r2, r2, #1
	strneh	r2, [r1, r5]	@ movhi
	bne	.L1640
	ldr	r4, [r3, #-1832]
	cmp	r4, #0
	beq	.L1640
	ldr	r1, .L1644+4
	mov	ip, #6
	ldrh	lr, [r3, r1]
	ldr	r3, [r3, #-1852]
	ldr	r1, .L1644+8
	rsb	r4, r3, r4
	mov	r4, r4, asr #1
	mul	r4, r1, r4
	movw	r1, #65535
	uxth	r4, r4
	b	.L1641
.L1643:
	cmp	r4, r0
	bne	.L1642
	mov	r1, r4
	ldr	r0, .L1644+12
	bl	List_remove_node
	ldr	r6, .L1644
	ldr	r3, .L1644+4
	mov	r0, r4
	ldrh	r2, [r6, r3]
	sub	r2, r2, #1
	strh	r2, [r6, r3]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r6, #-1844]
	ldrh	r2, [r3, r5]
	add	r2, r2, #1
	strh	r2, [r3, r5]	@ movhi
	b	.L1640
.L1642:
	mul	r4, ip, r4
	ldrh	r4, [r3, r4]
	cmp	r4, r1
	beq	.L1640
	add	r2, r2, #1
	uxth	r2, r2
.L1641:
	cmp	r2, lr
	bne	.L1643
.L1640:
	mov	r0, r7
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L1645:
	.align	2
.L1644:
	.word	.LANCHOR2
	.word	-1828
	.word	-1431655765
	.word	.LANCHOR2-1832
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
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
	ldr	r5, .L1650
	sub	r6, r6, #1
	mov	r2, #0
	str	r2, [r0, #36]
	uxth	r6, r6
	ldr	r0, [r5, #-740]
	ldr	r2, [r5, #-720]
	str	r0, [r5, #408]
	str	r2, [r5, #412]
	ldr	r3, [r3, r6, asl #2]
	cmp	r3, #0
	str	r3, [r5, #404]
	beq	.L1648
	mov	r1, #1
	add	r0, r5, #400
	mov	r2, r1
	bl	FlashReadPages
	b	.L1649
.L1648:
	ldr	r3, .L1650+4
	mov	r1, #255
	ldrh	r2, [r5, r3]
	bl	ftl_memset
.L1649:
	mov	r0, r4
	mov	r1, r6
	ldr	r2, [r5, #408]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L1651:
	.align	2
.L1650:
	.word	.LANCHOR2
	.word	-4042
	.fnend
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #60
	sub	sp, sp, #60
	ldr	r6, .L1662
	mov	r7, r0
	ldr	r3, .L1662+4
	mov	r4, r1
	str	r2, [sp, #8]
	add	r2, r1, r0
	ldrh	r8, [r6, r3]
	sub	r3, r3, #14
	ldrh	r3, [r6, r3]
	cmp	r2, r3
	mvnhi	r9, #0
	bhi	.L1653
	mov	r8, r0, lsr r8
	mov	r9, #0
	mov	sl, r6
	mov	r3, r8, asl #2
	str	r3, [sp, #4]
	b	.L1654
.L1659:
	ldr	r2, [sp, #4]
	mov	r0, r7
	ldr	r3, [r6, #-688]
	ldr	r3, [r3, r2]
	ldr	r2, .L1662+8
	str	r3, [sp, #0]
	ldrh	fp, [r6, r2]
	mov	r1, fp
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #0]
	uxth	r1, r1
	str	r1, [sp, #12]
	rsb	r5, r1, fp
	uxth	r5, r5
	cmp	r5, r4
	uxthhi	r5, r4
	cmp	r3, #0
	beq	.L1656
	cmp	r5, fp
	beq	.L1656
	str	r3, [sp, #24]
	mov	r1, #1
	ldr	r3, [sl, #-736]
	add	r0, sp, #20
	mov	r2, r1
	str	r3, [sp, #28]
	mov	r3, #0
	str	r3, [sp, #32]
	bl	FlashReadPages
	b	.L1657
.L1656:
	ldr	r3, .L1662+12
	mov	r1, #0
	ldr	r0, [sl, #-736]
	ldrh	r2, [sl, r3]
	bl	ftl_memset
.L1657:
	ldr	r3, [sp, #12]
	mov	fp, r5, asl #9
	ldr	r0, [r6, #-736]
	rsb	r4, r5, r4
	ldr	r1, [sp, #8]
	mov	r2, fp
	add	r0, r0, r3, asl #9
	add	r7, r7, r5
	bl	memcpy
	mov	r1, r8
	ldr	r0, .L1662+16
	add	r8, r8, #1
	ldr	r2, [r6, #-736]
	bl	FtlMapWritePage
	ldr	r3, [sp, #8]
	add	r3, r3, fp
	str	r3, [sp, #8]
	ldr	r3, [sp, #4]
	add	r3, r3, #4
	str	r3, [sp, #4]
	cmn	r0, #1
	moveq	r9, r0
.L1654:
	cmp	r4, #0
	bne	.L1659
.L1653:
	mov	r0, r9
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1663:
	.align	2
.L1662:
	.word	.LANCHOR2
	.word	-4044
	.word	-4046
	.word	-4042
	.word	.LANCHOR2+472
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
	ldr	r2, .L1666
	ldr	r3, .L1666+4
	ldr	r1, [r2, #-1488]
	cmp	r1, r3
	bxne	lr
	ldr	r3, .L1666+8
	mov	r0, #0
	mov	r1, #1
	str	r3, [r2, #-1484]
	ldr	r3, [r2, #-1640]
	str	r3, [r2, #-1400]
	ldr	r3, [r2, #-1636]
	str	r3, [r2, #-1396]
	ldr	r3, [r2, #-1644]
	str	r3, [r2, #-1480]
	ldr	r3, [r2, #-1656]
	str	r3, [r2, #-1476]
	ldr	r3, [r2, #-1664]
	str	r3, [r2, #-1472]
	ldr	r3, [r2, #-1648]
	str	r3, [r2, #-1468]
	ldr	r3, [r2, #-1620]
	str	r3, [r2, #-1460]
	ldr	r3, [r2, #-1860]
	str	r3, [r2, #-1456]
	ldr	r3, [r2, #-1660]
	str	r3, [r2, #-1452]
	ldr	r3, [r2, #-1652]
	str	r3, [r2, #-1448]
	ldr	r3, [r2, #-1612]
	str	r3, [r2, #-1444]
	ldr	r3, [r2, #-1608]
	str	r3, [r2, #-1440]
	ldr	r3, [r2, #-1492]
	str	r3, [r2, #-1428]
	ldr	r3, [r2, #-772]
	str	r3, [r2, #-1424]
	sub	r2, r2, #1488
	b	FtlVendorPartWrite
.L1667:
	.align	2
.L1666:
	.word	.LANCHOR2
	.word	1179929683
	.word	1342177319
	.fnend
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L1675
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, [r1, #-1668]
	cmp	r3, #0
	moveq	r2, #32
	beq	.L1669
	ldr	r2, [r1, #-1608]
	cmp	r2, #29
	movls	r2, #4
	movhi	r2, #32
.L1669:
	mov	r3, #512
	ldrh	r1, [r1, r3]
	cmp	r1, #31
	addls	r1, r1, #1
	ldrls	r2, .L1675
	strlsh	r1, [r2, r3]	@ movhi
	movls	r2, #1
	cmp	r0, #0
	ldr	r3, .L1675
	bne	.L1671
	ldr	r1, [r3, #-704]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r2, r2, r0
	cmp	r1, r2
	bcc	.L1672
.L1671:
	ldr	r2, [r3, #-704]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L1675+4
	str	r1, [r2, #0]
	ldr	r1, .L1675+8
	ldr	r2, [r3, #-704]
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
.L1672:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1676:
	.align	2
.L1675:
	.word	.LANCHOR2
	.word	1112818501
	.word	-708
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	2
	.type	FtlReadRefresh.part.11, %function
FtlReadRefresh.part.11:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r4, .L1682
	mov	r5, #2048
.L1680:
	ldr	r0, [r4, #-1404]
	ldr	r3, [r4, #-3992]
	cmp	r0, r3
	bcs	.L1678
	mov	r2, #0
	mov	r1, sp
	bl	log2phys
	ldr	r2, [sp, #0]
	ldr	r3, [r4, #-1404]
	cmn	r2, #1
	add	r3, r3, #1
	str	r3, [r4, #-1404]
	beq	.L1679
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
	bne	.L1678
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L1678
.L1679:
	subs	r5, r5, #1
	bne	.L1680
.L1678:
	mvn	r0, #0
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, pc}
.L1683:
	.align	2
.L1682:
	.word	.LANCHOR2
	.fnend
	.size	FtlReadRefresh.part.11, .-FtlReadRefresh.part.11
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1690
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r2, [r3, #-1408]
	cmp	r2, #0
	mov	r2, r3
	beq	.L1685
	ldr	r1, [r3, #-1404]
	ldr	r2, [r3, #-3992]
	cmp	r1, r2
	bcs	.L1686
	ldmfd	sp!, {r4, lr}
	b	FtlReadRefresh.part.11
.L1686:
	mov	r2, #0
	str	r2, [r3, #-1408]
	str	r2, [r3, #-1404]
	ldr	r2, [r3, #-1664]
	str	r2, [r3, #-1412]
	b	.L1687
.L1685:
	ldr	r1, [r3, #-1412]
	ldr	ip, [r3, #-1612]
	ldr	r0, [r3, #-1668]
	ldr	r3, [r3, #-1664]
	add	r4, r3, #1048576
	cmp	r1, r4
	bhi	.L1688
	add	r0, r0, ip, lsr #10
	mov	ip, #33554432
	add	r1, r1, ip, asr r0
	cmp	r1, r3
	bcs	.L1687
.L1688:
	mov	r1, #1
	str	r3, [r2, #-1412]
	str	r1, [r2, #-1408]
	mov	r1, #0
	str	r1, [r2, #-1404]
.L1687:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L1691:
	.align	2
.L1690:
	.word	.LANCHOR2
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
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
	ldr	r5, .L1712
	str	r3, [sp, #16]
	mov	r2, sl, asl #2
	ldrh	r3, [r0, #8]
	ldr	r7, [r0, #12]
	mov	r0, r8
	str	r3, [sp, #8]
	bl	ftl_memset
	ldr	r3, [r5, #-1856]
	ldr	r6, [r5, #-720]
	str	r9, [r4, #32]
	str	r3, [r5, #408]
	mvn	r3, #0
	str	r6, [r5, #412]
	strh	r3, [r4, #0]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #1
	str	r3, [r4, #36]
	ldr	r3, [sp, #8]
	str	r9, [r4, #28]
	sub	r3, r3, #1
	str	r3, [sp, #12]
	b	.L1693
.L1707:
	ldr	r0, [sp, #12]
	cmp	r3, r0
	mov	r3, r3, asl #1
	bne	.L1694
	ldrh	r0, [r7, r3]
	mov	r1, #1
	add	fp, r7, r3
	bl	FtlGetLastWrittenPage
	ldr	r1, [sp, #12]
	mov	r7, #0
	ldr	r2, [sp, #16]
	strh	r9, [r4, #0]	@ movhi
	ldr	r5, .L1712
	uxth	r0, r0
	add	r3, r0, #1
	strh	r3, [r4, #2]	@ movhi
	ldr	r3, [r2, r1, asl #2]
	sxth	r9, r0
	str	r7, [r5, #408]
	add	r9, r9, #1
	str	r3, [r4, #28]
	b	.L1695
.L1697:
	ldrh	r2, [fp, #0]
	mov	r1, #1
	ldr	r0, .L1712+4
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #404]
	bl	FlashReadPages
	ldr	r3, [r5, #400]
	cmn	r3, #1
	beq	.L1696
	ldrh	r3, [r6, #8]
	cmp	r3, sl
	bcs	.L1696
	ldrh	r2, [r4, #4]
	ldrh	r1, [r6, #0]
	cmp	r1, r2
	ldreq	r2, [r5, #404]
	streq	r2, [r8, r3, asl #2]
.L1696:
	add	r7, r7, #1
	uxth	r7, r7
.L1695:
	sxth	r3, r7
	cmp	r3, r9
	blt	.L1697
	b	.L1698
.L1694:
	ldr	r2, [r5, #-1856]
	add	r0, r7, r3
	ldr	fp, .L1712+8
	mov	r1, #1
	str	r0, [sp, #20]
	str	r2, [r5, #408]
	ldrh	r2, [r7, r3]
	ldrh	r3, [r5, fp]
	ldr	r0, .L1712+4
	sub	r3, r3, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r5, #404]
	bl	FlashReadPages
	ldr	r3, [r5, #400]
	cmn	r3, #1
	beq	.L1699
	ldrh	r2, [r6, #0]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L1699
	ldrh	r2, [r6, #8]
	movw	r3, #64245
	cmp	r2, r3
	bne	.L1699
	b	.L1711
.L1702:
	ldr	r1, [r5, #-1856]
	mov	r0, r2, asl #3
	add	r3, r3, #1
	ldr	r2, [r1, r2, asl #3]
	uxth	r3, r3
	uxth	r2, r2
	cmp	r2, sl
	addcc	r1, r1, r0
	ldrcc	r1, [r1, #4]
	strcc	r1, [r8, r2, asl #2]
	b	.L1700
.L1711:
	mov	r3, #0
.L1700:
	ldrh	r1, [r5, fp]
	sxth	r2, r3
	sub	r1, r1, #1
	cmp	r2, r1
	blt	.L1702
	b	.L1703
.L1699:
	ldr	r1, .L1712
	mov	fp, #0
	ldr	r3, .L1712+8
	str	fp, [r1, #408]
	b	.L1704
.L1706:
	ldr	r0, [sp, #20]
	ldrh	r1, [r0, #0]
	ldr	r0, .L1712+4
	str	r3, [sp, #4]
	orr	r2, r2, r1, asl #10
	mov	r1, #1
	str	r2, [r5, #404]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r2, [r5, #400]
	ldr	r3, [sp, #4]
	cmn	r2, #1
	beq	.L1705
	ldrh	r2, [r6, #8]
	cmp	r2, sl
	bcs	.L1705
	ldrh	r1, [r4, #4]
	ldrh	r0, [r6, #0]
	cmp	r0, r1
	ldreq	r1, [r5, #404]
	streq	r1, [r8, r2, asl #2]
.L1705:
	add	fp, fp, #1
	uxth	fp, fp
.L1704:
	ldrh	r1, [r5, r3]
	sxth	r2, fp
	cmp	r2, r1
	blt	.L1706
.L1703:
	add	r9, r9, #1
	uxth	r9, r9
.L1693:
	ldr	r1, [sp, #8]
	sxth	r3, r9
	cmp	r3, r1
	blt	.L1707
.L1698:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r3, .L1712+8
	ldr	r1, .L1712
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bne	.L1708
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1708:
	ldrh	r2, [r4, #8]
	ldrh	r3, [r4, #10]
	cmp	r2, r3
	bcc	.L1709
	mov	r0, r4
	bl	ftl_map_blk_gc
.L1709:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1713:
	.align	2
.L1712:
	.word	.LANCHOR2
	.word	.LANCHOR2+400
	.word	-4050
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
	ldr	r0, .L1715
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1716:
	.align	2
.L1715:
	.word	.LANCHOR2-856
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
	ldr	r0, .L1718
	ldr	r3, .L1718+4
	ldrh	r2, [r0, r3]
	movw	r3, #482
	strh	r2, [r0, r3]	@ movhi
	mov	r3, #476
	ldr	r2, .L1718+8
	strh	r2, [r0, r3]	@ movhi
	ldr	r3, .L1718+12
	ldrh	r2, [r0, r3]
	mov	r3, #480
	strh	r2, [r0, r3]	@ movhi
	ldr	r3, .L1718+16
	ldrh	r2, [r0, r3]
	movw	r3, #478
	strh	r2, [r0, r3]	@ movhi
	ldr	r3, [r0, #-4004]
	str	r3, [r0, #484]
	ldr	r3, [r0, #-692]
	str	r3, [r0, #488]
	ldr	r3, [r0, #-696]
	str	r3, [r0, #492]
	ldr	r3, [r0, #-688]
	str	r3, [r0, #496]
	add	r0, r0, #472
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1719:
	.align	2
.L1718:
	.word	.LANCHOR2
	.word	-4032
	.word	-3962
	.word	-4008
	.word	-4030
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #60
	sub	sp, sp, #60
	ldrh	r3, [r0, #2]
	mov	r4, r0
	ldr	r2, .L1796
	ldrb	r8, [r0, #6]	@ zero_extendqisi2
	str	r3, [sp, #12]
	ldr	r3, .L1796+4
	ldr	fp, [sp, #12]
	str	r8, [sp, #24]
	ldrh	r3, [r2, r3]
	cmp	r3, fp
	mov	r3, #0
	streqh	r3, [r0, #4]	@ movhi
	moveq	r2, r3	@ movhi
	ldrneh	r0, [r0, #16]
	movwne	r2, #65535
	bne	.L1723
	b	.L1793
.L1724:
	add	r3, r3, #1
	uxth	r3, r3
	add	r1, r4, r3, asl #1
	ldrh	r0, [r1, #16]
.L1723:
	cmp	r0, r2
	beq	.L1724
	ldrb	r1, [r4, #8]	@ zero_extendqisi2
	cmp	r1, #1
	bne	.L1725
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1726
	ldr	r3, .L1796+8
	add	r3, r3, r0, asl #1
	add	r3, r3, #2288
	ldrh	r8, [r3, #0]
	b	.L1727
.L1725:
	mov	r1, #0
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1726
	mov	r8, r0
.L1727:
	ldr	r2, .L1796+8
	mov	r3, #4064
	mov	r5, #0
	str	r4, [sp, #32]
	movw	ip, #65535
	ldrh	r9, [r2, r3]
	ldr	r3, .L1796
	ldr	r2, .L1796+12
	ldr	sl, [r3, #-752]
	ldrh	lr, [r3, r2]
	mov	r2, r4
	ldr	r6, [r3, #-948]
	mov	r3, r5
	b	.L1728
.L1726:
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	mov	r2, r3	@ movhi
.L1793:
	strb	r2, [r4, #6]
	b	.L1722
.L1730:
	ldrh	r0, [r2, #16]
	cmp	r0, ip
	beq	.L1729
	mov	fp, #36
	orr	r0, r8, r0, asl #10
	mla	r1, fp, r5, sl
	str	r0, [r1, #4]
	mov	r0, #0
	str	r0, [r1, #8]
	mul	r0, lr, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	fp, r0, #3
	cmp	r0, #0
	movlt	r0, fp
	bic	r0, r0, #3
	add	r0, r6, r0
	str	r0, [r1, #12]
.L1729:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1728:
	cmp	r3, r9
	bne	.L1730
	ldr	r6, .L1796
	mov	r2, #0
	mov	r1, r5
	mov	fp, r7
	mov	sl, #0
	ldr	r0, [r6, #-752]
	bl	FlashReadPages
	ldr	r2, [r6, #-1628]
	movw	r3, #65535
	ldr	r9, [r6, #-752]
	str	r3, [sp, #8]
	mov	r3, r6
	str	r2, [sp, #20]
	uxth	r2, r8
	mov	r7, r2
	b	.L1731
.L1736:
	ldr	r2, [r9, #0]
	cmp	r2, #0
	strne	r7, [sp, #8]
	bne	.L1734
	ldr	r2, [r9, #12]
	ldr	r6, [r2, #4]
	cmn	r6, #1
	beq	.L1733
	ldr	r1, [r3, #-1628]
	mov	r0, r6
	str	r2, [sp, #4]
	str	r3, [sp, #0]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #0]
	ldr	r2, [sp, #4]
	cmp	r0, #0
	addne	r6, r6, #1
	strne	r6, [r3, #-1628]
.L1733:
	ldr	r2, [r2, #0]
	cmn	r2, #1
	bne	.L1734
	ldr	r3, .L1796
	mov	r2, #36
	uxth	r7, fp
	ldr	r3, [r3, #-752]
	mla	sl, r2, sl, r3
	ldr	r0, [sl, #4]
	b	.L1789
.L1734:
	add	sl, sl, #1
	add	r9, r9, #36
	uxth	sl, sl
.L1731:
	cmp	sl, r5
	bne	.L1736
	ldr	r3, .L1796
	add	r7, fp, #1
	uxth	r7, r7
	ldr	r3, [r3, #-752]
	ldr	r0, [r3, #4]
.L1789:
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	ldr	r1, .L1796
	cmp	r3, #1
	ldr	fp, [sp, #12]
	ldr	lr, [sp, #24]
	ldreq	r2, .L1796+8
	addeq	r7, r2, r7, asl #1
	ldr	r2, .L1796+4
	addeq	r7, r7, #2288
	str	r0, [sp, #16]
	ldr	ip, [sp, #16]
	ldreqh	r7, [r7, #0]
	ldrh	r2, [r1, r2]
	cmp	r2, r7
	streqh	r7, [r4, #2]	@ movhi
	moveq	r2, #0
	streqb	r2, [r4, #6]
	streqh	r2, [r4, #4]	@ movhi
	cmp	r7, fp
	cmpeq	ip, lr
	moveq	r0, r4
	moveq	r1, r7
	moveq	r2, ip
	beq	.L1794
	ldr	r2, [sp, #20]
	ldr	fp, [sp, #8]
	sub	r6, r2, #1
	movw	r2, #65535
	cmp	fp, r2
	bne	.L1740
	cmp	r3, #0
	bne	.L1741
.L1740:
	ldr	r2, .L1796
	uxth	r3, r8
	ldr	ip, [sp, #12]
	mvn	fp, #0
	mov	sl, r3
	mov	r9, fp
	ldr	r1, [r2, #-780]
	ldr	r8, .L1796
	cmn	r1, #1
	streq	r6, [r2, #-780]
	ldr	r2, [r2, #-780]
	str	r2, [sp, #8]
	add	r2, ip, #7
	cmp	r3, r2
	subgt	r5, r3, #7
	mov	r3, r7
	ldrle	r5, [sp, #12]
	mov	r7, r6
	uxthgt	r5, r5
	b	.L1744
.L1746:
	ldrh	r0, [r1, #16]
	movw	lr, #65535
	cmp	r0, lr
	beq	.L1745
	ldr	ip, [sp, #20]
	mov	r3, #36
	orr	r0, r5, r0, asl #10
	mla	lr, r3, r6, ip
	add	r6, r6, #1
	uxth	r6, r6
	str	r0, [lr, #4]
.L1745:
	add	r2, r2, #1
	add	r1, r1, #2
	uxth	r2, r2
.L1756:
	ldr	lr, [sp, #36]
	cmp	r2, lr
	bne	.L1746
	ldr	r3, [sp, #28]
	mov	r1, r6
	ldr	r0, [r8, #-752]
	mov	r2, #0
	str	r3, [sp, #0]
	bl	FlashReadPages
	add	ip, r8, r5, asl #1
	ldr	r2, [r8, #-752]
	mov	r1, #0
	ldr	r3, [sp, #0]
	str	ip, [sp, #20]
	b	.L1747
.L1754:
	ldr	r0, [r2, #0]
	cmp	r0, #0
	bne	.L1748
	ldr	r0, [r2, #12]
	movw	ip, #65535
	ldrh	lr, [r0, #0]
	cmp	lr, ip
	beq	.L1749
	ldr	r0, [r0, #4]
	cmn	r0, #1
	beq	.L1749
	cmn	fp, #1
	ldr	r9, [r8, #-780]
	str	r0, [r8, #-780]
	bne	.L1749
	ldr	lr, [sp, #20]
	sub	r0, lr, #680
	ldrh	r0, [r0, #0]
	cmp	r0, ip
	beq	.L1749
	cmp	r9, r7
	mvneq	fp, #0
	movne	fp, r9
	b	.L1749
.L1748:
	mov	r6, r7
	mov	r7, r3
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1741
	ldr	r3, .L1796
	movw	r2, #65535
	add	r5, r3, r5, asl #1
	sub	r5, r5, #680
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	bne	.L1750
	cmn	fp, #1
	strne	fp, [r3, #-780]
	bne	.L1741
	ldr	r8, [sp, #8]
	cmp	r8, r6
	strne	r8, [r3, #-780]
	ldreq	r2, [r3, #-780]
	bne	.L1741
	b	.L1795
.L1750:
	cmp	r9, r6
	beq	.L1753
	cmn	r9, #1
	strne	r9, [r3, #-780]
	b	.L1741
.L1753:
	ldr	r2, [r3, #-780]
	cmp	r2, r6
	beq	.L1741
.L1795:
	sub	r2, r2, #1
	b	.L1790
.L1749:
	add	r1, r1, #1
	add	r2, r2, #36
	uxth	r1, r1
.L1747:
	cmp	r1, r6
	bne	.L1754
	add	r5, r5, #1
	uxth	r5, r5
.L1744:
	cmp	r5, sl
	bhi	.L1755
	ldr	r1, .L1796+8
	mov	r2, #4064
	ldr	lr, [r8, #-752]
	mov	r6, #0
	str	r3, [sp, #28]
	ldrh	ip, [r1, r2]
	mov	r1, r4
	str	lr, [sp, #20]
	mov	r2, r6
	str	ip, [sp, #36]
	b	.L1756
.L1755:
	mov	r6, r7
	mov	r7, r3
	ldr	r3, .L1796
	mvn	r2, #0
.L1790:
	str	r2, [r3, #-780]
.L1741:
	ldr	r5, .L1796
	movw	r3, #514
	mov	r2, #1
	sub	r0, r5, #856
	strh	r2, [r5, r3]	@ movhi
	bl	FtlMapBlkWriteDumpData
	ldr	r9, [sp, #12]
	str	r7, [sp, #28]
.L1757:
	ldr	r2, .L1796+8
	mov	r3, #4064
	mov	r7, #0
	ldr	ip, [r5, #-752]
	mov	r0, #36
	ldrh	lr, [r2, r3]
	mov	r2, r4
	mov	r3, r7
	b	.L1758
.L1760:
	ldrh	r1, [r2, #16]
	movw	r8, #65535
	add	r3, r3, #1
	add	r2, r2, #2
	cmp	r1, r8
	uxth	r3, r3
	orrne	r1, r9, r1, asl #10
	mlane	r8, r0, r7, ip
	addne	r7, r7, #1
	uxthne	r7, r7
	strne	r1, [r8, #4]
.L1758:
	cmp	r3, lr
	bne	.L1760
	mov	r1, r7
	ldr	r0, [r5, #-752]
	mov	r2, #0
	bl	FlashReadPages
	mov	r3, #36
	mul	r3, r3, r7
	mov	r7, #0
	str	r3, [sp, #36]
	b	.L1761
.L1782:
	ldr	sl, [r5, #-752]
	add	sl, sl, r7
	ldr	r8, [sl, #4]
	ubfx	r0, r8, #10, #16
	str	r8, [sp, #52]
	bl	P2V_plane
	ldr	fp, [sp, #12]
	cmp	r9, fp
	bcc	.L1762
	ldr	ip, [sp, #24]
	cmp	r0, ip
	movcs	r3, #0
	movcc	r3, #1
	cmp	r9, fp
	movne	r3, #0
	cmp	r3, #0
	bne	.L1762
	ldr	r3, [sp, #16]
	ldr	ip, [sp, #28]
	cmp	r0, r3
	cmpeq	r9, ip
	beq	.L1763
	ldr	r3, [sl, #0]
	cmn	r3, #1
	beq	.L1764
	ldr	r8, [sl, #12]
	movw	lr, #61589
	ldrh	r3, [r8, #0]
	cmp	r3, lr
	bne	.L1771
.L1765:
	ldr	r6, [r8, #4]
	cmn	r6, #1
	beq	.L1766
	ldr	r1, [r5, #-1628]
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	addne	r3, r6, #1
	strne	r3, [r5, #-1628]
.L1766:
	ldr	sl, [r8, #8]
	add	r1, sp, #48
	ldr	r3, [r8, #12]
	mov	r2, #0
	mov	r0, sl
	str	r3, [sp, #44]
	bl	log2phys
	ldr	r1, [r5, #-780]
	cmn	r1, #1
	beq	.L1767
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1767
	ldr	r3, [sp, #44]
	cmn	r3, #1
	beq	.L1768
	ldr	r0, [r5, #-752]
	mov	r1, #1
	mov	r2, #0
	add	r0, r0, r7
	ldr	r8, [r0, #12]
	str	r3, [r0, #4]
	str	r8, [sp, #8]
	bl	FlashReadPages
	ldr	fp, [r8, #4]
	ldr	r3, [r5, #-752]
	str	fp, [sp, #20]
	add	fp, r3, r7
	ldr	r3, [r3, r7]
	cmn	r3, #1
	bne	.L1769
	b	.L1770
.L1768:
	ldr	r3, [sp, #52]
	ldr	r2, [sp, #48]
	cmp	r2, r3
	bne	.L1771
	mov	r0, sl
	add	r1, sp, #44
	mov	r2, #1
	bl	log2phys
.L1771:
	ldrh	r0, [r4, #0]
	b	.L1792
.L1769:
	ldr	ip, [sp, #8]
	ldr	r8, [ip, #8]
	cmp	r8, sl
	bne	.L1770
	ldr	r0, [r5, #-780]
	ldr	r1, [sp, #20]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1770
	ldr	r3, [sp, #48]
	ldr	r2, [sp, #52]
	cmp	r3, r2
	beq	.L1775
.L1772:
	ldr	r2, [sp, #44]
	cmp	r3, r2
	beq	.L1770
	cmn	r3, #1
	streq	r3, [fp, #0]
	beq	.L1774
	ldr	ip, [fp, #12]
	mov	r0, fp
	str	r3, [fp, #4]
	mov	r1, #1
	mov	r2, #0
	str	ip, [sp, #8]
	bl	FlashReadPages
.L1774:
	ldr	r3, [r5, #-752]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	beq	.L1775
	ldr	r3, [sp, #8]
	ldr	r0, [r5, #-780]
	ldr	sl, [r3, #4]
	mov	r1, sl
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1775
	ldr	r0, [sp, #20]
	mov	r1, sl
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1770
.L1775:
	mov	r0, r8
	ldr	r1, [sp, #44]
	bl	FtlReUsePrevPpa
.L1770:
	ldrh	r0, [r4, #0]
	mvn	r3, #0
	str	r3, [sp, #44]
	bl	decrement_vpc_count
	b	.L1777
.L1767:
	ldr	r3, [sp, #52]
	ldr	r2, [sp, #48]
	cmp	r2, r3
	beq	.L1777
	mov	r0, sl
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
	ldr	fp, [sp, #48]
	cmn	fp, #1
	beq	.L1777
	ldr	r3, [sp, #44]
	cmp	fp, r3
	beq	.L1777
	ubfx	r0, fp, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L1796+16
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1778
	ldr	r3, .L1796+20
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1778
	ldr	r3, .L1796+24
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	bne	.L1777
.L1778:
	ldr	r8, .L1796
	mov	r1, #1
	mov	r2, #0
	ldr	r0, [r8, #-752]
	str	fp, [r0, #4]
	ldr	fp, .L1796
	ldr	r8, [r0, #12]
	bl	FlashReadPages
	ldr	r3, [fp, #-752]
	ldr	r1, [r8, #4]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L1777
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L1777
	mov	r0, sl
	add	r1, sp, #48
	mov	r2, #1
	bl	log2phys
.L1777:
	ldr	r0, [sp, #44]
	cmn	r0, #1
	beq	.L1762
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r5, #-1844]
	mov	r3, r0, asl #1
	mov	r1, r0
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1779
.L1792:
	bl	decrement_vpc_count
	b	.L1762
.L1779:
	ldr	r0, .L1796+28
	bl	printk
	b	.L1762
.L1764:
	ldr	r3, [r5, #516]
	cmp	r3, #31
	addls	r2, r5, r3, asl #2
	addls	r3, r3, #1
	strls	r3, [r5, #516]
	strls	r8, [r2, #520]
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
	ldr	r3, [r5, #-780]
	cmn	r3, #1
	beq	.L1791
.L1781:
	cmp	r3, r6
	bls	.L1762
.L1791:
	str	r6, [r5, #-780]
.L1762:
	add	r7, r7, #36
.L1761:
	ldr	ip, [sp, #36]
	cmp	r7, ip
	bne	.L1782
	ldr	r3, .L1796+4
	add	r9, r9, #1
	ldr	r0, .L1796
	uxth	r9, r9
	ldrh	r3, [r0, r3]
	cmp	r9, r3
	bne	.L1757
	ldr	r1, .L1796+8
	mov	r2, #4064
	mov	r3, #0
	strh	r9, [r4, #2]	@ movhi
	strh	r3, [r4, #4]	@ movhi
	ldrh	r1, [r1, r2]
	movw	r2, #65535
	ldr	r0, [sp, #32]
	b	.L1783
.L1785:
	add	r0, r0, #2
	ldrh	ip, [r0, #14]
	cmp	ip, r2
	strneb	r3, [r4, #6]
	bne	.L1722
.L1784:
	add	r3, r3, #1
	uxth	r3, r3
.L1783:
	cmp	r3, r1
	bne	.L1785
	b	.L1722
.L1763:
	ldr	r7, [sp, #28]
	mov	r0, r4
	ldr	r3, [sp, #16]
	mov	r1, r7
	strh	r7, [r4, #2]	@ movhi
	mov	r2, r3
	strb	r3, [r4, #6]
.L1794:
	bl	ftl_sb_update_avl_pages
.L1722:
	mov	r0, #0
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1797:
	.align	2
.L1796:
	.word	.LANCHOR2
	.word	-4052
	.word	.LANCHOR0
	.word	-4040
	.word	-1824
	.word	-1776
	.word	-1728
	.word	.LC111
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	2
	.global	FtlWriteDumpData
	.type	FtlWriteDumpData, %function
FtlWriteDumpData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #40
	sub	sp, sp, #40
	ldr	r5, .L1807
	ldr	r3, .L1807+4
	mov	r2, r5
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	beq	.L1799
	ldrb	r1, [r5, #-1816]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L1799
	ldr	r0, .L1807+8
	ldrb	r1, [r5, #-1817]	@ zero_extendqisi2
	ldrh	r0, [r5, r0]
	mul	r1, r0, r1
	cmp	r3, r1
	beq	.L1799
	ldrb	r8, [r5, #-1814]	@ zero_extendqisi2
	mov	r3, #4064
	ldr	r2, .L1807+12
	cmp	r8, #0
	ldr	r7, [r5, #-3992]
	ldrh	r6, [r2, r3]
	bne	.L1798
	sub	r7, r7, #1
	mov	r1, sp
	mov	r2, r8
	mov	r0, r7
	bl	log2phys
	ldr	r3, [sp, #0]
	ldr	r4, [r5, #-720]
	ldr	r0, [r5, #-1856]
	cmn	r3, #1
	str	r3, [sp, #8]
	str	r7, [sp, #20]
	str	r0, [sp, #12]
	str	r4, [sp, #16]
	str	r8, [r4, #4]
	beq	.L1801
	add	r0, sp, #4
	mov	r1, #1
	mov	r2, r8
	bl	FlashReadPages
	b	.L1802
.L1801:
	ldr	r3, .L1807+16
	mov	r1, #255
	ldrh	r2, [r5, r3]
	bl	ftl_memset
.L1802:
	mov	r3, #6
	ldr	r5, .L1807
	mul	r6, r3, r6
	ldr	r3, .L1807+20
	ldr	r8, .L1807+4
	strh	r3, [r4, #0]	@ movhi
	b	.L1803
.L1806:
	ldrh	r3, [r5, r8]
	cmp	r3, #0
	beq	.L1804
	ldr	r3, [sp, #8]
	sub	r0, r0, #1824
	str	r7, [r4, #8]
	sub	r6, r6, #1
	str	r3, [r4, #12]
	ldr	r3, .L1807+24
	ldrh	r3, [r5, r3]
	strh	r3, [r4, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r5, #-1628]
	mov	r2, #0
	mov	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r4, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r5, #-1628]
	moveq	r3, #0
	streq	r3, [r5, #-1628]
	mov	r3, r2
	bl	FlashProgPages
	ldr	r3, .L1807+24
	ldrh	r0, [r5, r3]
	bl	decrement_vpc_count
.L1803:
	cmp	r6, #0
	ldr	r0, .L1807
	bne	.L1806
.L1804:
	mov	r3, #1
	strb	r3, [r0, #-1814]
	b	.L1798
.L1799:
	mov	r3, #0
	strb	r3, [r2, #-1814]
.L1798:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1808:
	.align	2
.L1807:
	.word	.LANCHOR2
	.word	-1820
	.word	-4052
	.word	.LANCHOR0
	.word	-4042
	.word	-3947
	.word	-1824
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
	ldr	r5, .L1813
	mov	r6, #12
	ldr	r7, .L1813+4
	b	.L1810
.L1812:
	ldr	r3, [r5, #-1680]
	mla	r3, r6, r4, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L1811
	mov	r0, r4
	bl	flush_l2p_region
.L1811:
	add	r4, r4, #1
	uxth	r4, r4
.L1810:
	ldrh	r3, [r5, r7]
	cmp	r3, r4
	bhi	.L1812
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1814:
	.align	2
.L1813:
	.word	.LANCHOR2
	.word	-4014
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
	ldr	r1, .L1822
	ldr	r0, .L1822+4
	bl	printk
	ldr	r2, .L1822+8
	movw	r3, #4074
	ldr	r5, .L1822+12
	mov	r1, #0
	ldrh	r2, [r2, r3]
	ldr	r0, [r5, #-700]
	mov	r2, r2, asl #1
	bl	ftl_memset
	b	.L1816
.L1818:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L1817
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r5, #-700]
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	add	r2, r2, #1
	strh	r2, [r3, r0]	@ movhi
.L1817:
	add	r4, r4, #1
.L1816:
	ldr	r3, [r5, #-3992]
	cmp	r4, r3
	bcc	.L1818
	mov	r4, #0
	ldr	r7, .L1822+8
	movw	r8, #4072
	ldr	r5, .L1822+12
	b	.L1819
.L1821:
	ldr	r3, [r5, #-1844]
	mov	r6, r4, asl #1
	ldrh	r2, [r3, r6]
	ldr	r3, [r5, #-700]
	ldrh	r3, [r3, r6]
	cmp	r2, r3
	beq	.L1820
	movw	r1, #65535
	cmp	r2, r1
	beq	.L1820
	ldr	r0, .L1822+16
	mov	r1, r4
	bl	printk
	ldr	r3, .L1822+20
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1820
	ldr	r3, .L1822+24
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1820
	ldr	r3, .L1822+28
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1820
	ldr	r3, [r5, #-700]
	mov	r0, r4
	ldrh	r2, [r3, r6]
	ldr	r3, [r5, #-1844]
	strh	r2, [r3, r6]	@ movhi
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1820:
	add	r4, r4, #1
	uxth	r4, r4
.L1819:
	ldrh	r3, [r7, r8]
	cmp	r3, r4
	bhi	.L1821
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1823:
	.align	2
.L1822:
	.word	.LANCHOR3+15
	.word	.LC112
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC113
	.word	-1824
	.word	-1728
	.word	-1776
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
	beq	.L1825
	ldr	r3, .L1835
	mov	r0, r5
	ldr	r2, [r3, #-1844]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1826
	bl	INSERT_DATA_LIST
	b	.L1825
.L1826:
	bl	INSERT_FREE_LIST
.L1825:
	mov	r3, #0
	strb	r3, [r4, #8]
	ldr	r3, .L1835
	sub	r2, r3, #1776
	cmp	r4, r2
	beq	.L1827
	ldr	r1, .L1835+4
	movw	r2, #4084
	ldrh	r2, [r1, r2]
	cmp	r2, #1
	bne	.L1828
.L1827:
	mov	r3, #1
	strb	r3, [r4, #8]
	b	.L1829
.L1828:
	sub	r1, r3, #1824
	cmp	r4, r1
	bne	.L1829
	cmp	r2, #3
	beq	.L1830
	ldr	r2, [r3, #-1492]
	cmp	r2, #1
	bne	.L1831
.L1830:
	mov	r2, #1
	strb	r2, [r3, #-1816]
.L1831:
	ldr	r3, [r3, #-1668]
	ldr	r2, .L1835
	cmp	r3, #0
	beq	.L1829
	ldr	r3, [r2, #-1608]
	cmp	r3, #29
	movls	r3, #1
	strlsb	r3, [r2, #-1816]
.L1829:
	ldr	r3, .L1835+8
	ldr	r6, .L1835
	ldrh	r0, [r6, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1832
	cmp	r5, r0
	bne	.L1833
	ldr	r2, [r6, #-1844]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1834
.L1833:
	bl	update_vpc_list
.L1834:
	ldr	r3, .L1835+8
	mvn	r2, #0
	strh	r2, [r6, r3]	@ movhi
.L1832:
	mov	r0, r4
	bl	allocate_data_superblock
	bl	l2p_flush
	bl	FtlVpcTblFlush
	mov	r0, #0
	bl	FtlEctTblFlush
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1836:
	.align	2
.L1835:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-792
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	ldr	r4, .L1849
	ldr	r1, [r4, #-4000]
	ldr	r6, [r4, #648]
	cmp	r1, #0
	beq	.L1838
	mov	r5, #0
	ldr	r0, [r4, #-3996]
	mov	r2, #0
	ldrb	r3, [r6, #9]	@ zero_extendqisi2
	bl	FlashProgPages
	mov	r9, r5
	mov	r8, r5
	ldr	fp, .L1849+4
	mov	r7, r4
	b	.L1839
.L1847:
	cmp	r9, #0
	mov	sl, #0
	ldrne	r3, [r4, #-3996]
	mvnne	r2, #0
	strne	r2, [r3, r5]
	b	.L1848
.L1844:
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r6, #0]
	cmp	r3, r0
	bne	.L1842
	ldr	r2, [r4, #-1844]
	mov	r3, r3, asl #1
	ldrh	r1, [r6, #4]
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r4, fp]
	strb	sl, [r6, #6]
	strh	sl, [r6, #4]	@ movhi
	strh	r3, [r6, #2]	@ movhi
.L1842:
	ldrh	r3, [r6, #4]
	cmp	r3, #0
	bne	.L1843
	mov	r0, r6
	bl	allocate_new_data_superblock
.L1843:
	ldr	r3, [r4, #-1392]
	mov	r9, #1
	add	r3, r3, #1
	str	r3, [r4, #-1392]
	ldr	r3, [r4, #-3996]
	add	r3, r3, r5
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlGcMarkBadPhyBlk
	mov	r0, r6
	bl	get_new_active_ppa
	mov	r1, #1
	mov	r2, #0
	mov	r3, r0
	str	r0, [sp, #4]
	ldr	r0, [r4, #-3996]
	add	r0, r0, r5
	str	r3, [r0, #4]
	ldrb	r3, [r6, #9]	@ zero_extendqisi2
	bl	FlashProgPages
.L1848:
	ldr	r2, [r4, #-3996]
	add	r3, r2, r5
	ldr	r2, [r2, r5]
	cmn	r2, #1
	beq	.L1844
	ldr	r2, [r3, #4]
	add	r1, sp, #8
	ldr	r0, [r3, #16]
	str	r2, [r1, #-4]!
	mov	r2, #1
	bl	log2phys
	ldr	r3, [r7, #-3996]
	add	r3, r3, r5
	ldr	r3, [r3, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L1845
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r7, #-1844]
	mov	r3, r0, asl #1
	mov	sl, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L1846
	ldr	r0, .L1849+8
	mov	r1, sl
	bl	printk
.L1846:
	mov	r0, sl
	bl	decrement_vpc_count
.L1845:
	add	r8, r8, #1
	add	r5, r5, #36
.L1839:
	ldr	r3, [r4, #-4000]
	cmp	r8, r3
	bcc	.L1847
	ldr	r3, .L1849
	mov	r2, #0
	str	r2, [r3, #-4000]
.L1838:
	mov	r0, #0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1850:
	.align	2
.L1849:
	.word	.LANCHOR2
	.word	-4052
	.word	.LC114
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
	ldr	r3, .L1873
	ldr	r2, .L1873+4
	ldrh	r1, [r3, r2]
	ldr	r2, .L1873+8
	ldrh	r5, [r3, r2]
	movw	r2, #65535
	cmp	r5, r2
	beq	.L1853
	cmp	r0, #0
	beq	.L1854
	ldr	ip, .L1873+12
	movw	r0, #2868
	ldrh	lr, [ip, r0]
	cmp	lr, r2
	movne	r1, #2
	bne	.L1854
	mov	r2, #0
	strh	r2, [ip, r0]	@ movhi
	ldr	r2, .L1873+16
	ldrh	r3, [r3, r2]
	cmp	r3, #17
	movhi	r1, #2
.L1854:
	ldr	r4, .L1873
	sub	r0, r4, #1728
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #4]
	beq	.L1855
	ldr	r3, [r4, #-1392]
	mov	r5, r5, asl #1
	add	r3, r3, #1
	str	r3, [r4, #-1392]
	ldr	r3, [r4, #-1864]
	ldrh	r2, [r3, r5]
	cmp	r2, #4
	bls	.L1856
	sub	r2, r2, #5
	mov	r0, #1
	strh	r2, [r3, r5]	@ movhi
	bl	FtlEctTblFlush
.L1856:
	ldr	r4, .L1873
	ldr	r3, [r4, #-784]
	cmp	r3, #0
	bne	.L1857
	ldr	r0, [sp, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L1857:
	mov	r3, #0
	str	r3, [r4, #-784]
	b	.L1868
.L1855:
	ldr	r2, .L1873+12
	movw	r3, #2868
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1868
.L1853:
	ldr	r6, .L1873
	movw	r3, #65535
	ldr	r5, .L1873+8
	mov	r4, #0
	str	r4, [r6, #-784]
	ldrh	r2, [r6, r5]
	cmp	r2, r3
	moveq	r0, r4
	beq	.L1858
	bl	FtlCacheWriteBack
	ldr	r0, .L1873+4
	ldrb	r1, [r6, #-1721]	@ zero_extendqisi2
	ldrh	r3, [r6, r5]
	ldrh	r0, [r6, r0]
	ldr	r2, [r6, #-1844]
	ldr	sl, .L1873+20
	mov	r3, r3, asl #1
	mul	r1, r0, r1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, [r6, #-1644]
	ldrh	r2, [r6, sl]
	add	r3, r2, r3
	str	r3, [r6, #-1644]
	b	.L1859
.L1862:
	mov	r7, #12
	ldr	r8, [r6, #-928]
	mul	r7, r7, r4
	add	r1, sp, #4
	mov	r2, #0
	add	r5, r8, r7
	ldr	r0, [r5, #8]
	bl	log2phys
	ldr	r0, [sp, #4]
	ldr	r3, [r8, r7]
	cmp	r0, r3
	bne	.L1860
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r5, #4
	mov	r2, #1
	mov	r7, r0
	ldr	r0, [r5, #8]
	bl	log2phys
	mov	r0, r7
	b	.L1871
.L1860:
	ldr	r3, [r5, #4]
	cmp	r0, r3
	beq	.L1861
	ldr	r3, .L1873+8
	ldrh	r0, [r6, r3]
.L1871:
	bl	decrement_vpc_count
.L1861:
	add	r4, r4, #1
	uxth	r4, r4
.L1859:
	ldrh	r3, [r6, sl]
	ldr	r5, .L1873
	cmp	r3, r4
	bhi	.L1862
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r3, .L1873+8
	ldr	r1, [r5, #-1844]
	ldrh	r0, [r5, r3]
	mov	r5, r3
	mov	r2, r0, asl #1
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	beq	.L1863
	bl	INSERT_DATA_LIST
	b	.L1864
.L1863:
	bl	INSERT_FREE_LIST
.L1864:
	ldr	r4, .L1873
	mvn	r3, #0
	ldr	r2, .L1873+20
	strh	r3, [r4, r5]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	ldr	r2, .L1873+24
	strh	r3, [r4, r2]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	r3, [r4, #-1668]
	mov	r1, r4
	ldr	r0, .L1873+16
	cmp	r3, #0
	ldr	r2, .L1873+28
	beq	.L1865
	ldr	r3, [r4, #-1608]
	cmp	r3, #29
	bhi	.L1865
	ldrh	r3, [r4, r2]
	mvn	r1, #0
	ldrh	r2, [r4, r0]
	cmp	r2, r3
	movcc	r3, r3, asl #1
	ldrcc	r2, .L1873+32
	strcch	r3, [r4, r2]	@ movhi
	ldr	r3, .L1873+36
	ldr	r2, .L1873
	b	.L1872
.L1865:
	ldrh	r2, [r1, r2]
	ldrh	r0, [r1, r0]
	ldr	r3, .L1873
	add	r2, r2, r2, asl #1
	cmp	r0, r2, lsr #2
	ble	.L1870
	ldr	r2, .L1873+36
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #564
	mov	r1, #20
.L1872:
	strh	r1, [r3, r2]	@ movhi
	b	.L1870
.L1868:
	mov	r0, #1
	b	.L1858
.L1870:
	mov	r0, #0
.L1858:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L1874:
	.align	2
.L1873:
	.word	.LANCHOR2
	.word	-4052
	.word	-1728
	.word	.LANCHOR1
	.word	-1828
	.word	-924
	.word	-936
	.word	-1556
	.word	-976
	.word	-1540
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
	mov	r2, #0
	ldr	r4, .L1881
	mov	r3, r2
	mov	r5, #0
	mov	r6, #36
	ldr	r0, [r4, #-748]
	ldr	r1, [r4, #-960]
	bl	FlashProgPages
	b	.L1876
.L1879:
	mul	r3, r6, r5
	ldr	r1, [r4, #-748]
	add	r2, r1, r3
	ldr	r1, [r1, r3]
	cmn	r1, #1
	bne	.L1877
	ldr	r2, .L1881+4
	mov	lr, #0
	ldr	ip, [r7, #-1844]
	ldrh	r0, [r7, r2]
	mov	r0, r0, asl #1
	strh	lr, [ip, r0]	@ movhi
	strh	r1, [r7, r2]	@ movhi
	ldr	r2, [r7, #-1392]
	add	r2, r2, #1
	str	r2, [r7, #-1392]
	ldr	r2, [r7, #-748]
	add	r3, r2, r3
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1877:
	ldr	r3, [r2, #12]
	add	r5, r5, #1
	ldr	r1, [r2, #4]
	uxth	r5, r5
	ldr	r0, [r3, #12]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
.L1876:
	ldr	r1, [r4, #-960]
	ldr	r7, .L1881
	cmp	r5, r1
	bcc	.L1879
	ldr	r0, [r7, #-748]
	bl	FtlGcBufFree
	ldr	r3, .L1881+8
	mov	r0, #0
	str	r0, [r7, #-960]
	ldrh	r3, [r7, r3]
	cmp	r3, r0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1882:
	.align	2
.L1881:
	.word	.LANCHOR2
	.word	-1728
	.word	-1724
	.fnend
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L1885
	ldr	r5, .L1885+4
	sub	r0, r4, #1728
	ldrh	r1, [r4, r5]
	bl	FtlGcScanTempBlk
	ldr	r3, .L1885+8
	ldrh	r2, [r4, r3]
	ldrh	r3, [r4, r5]
	cmp	r2, r3
	ldmccfd	sp!, {r3, r4, r5, pc}
	sub	r0, r4, #856
	bl	FtlMapBlkWriteDumpData
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	mov	r3, #0
	str	r3, [r4, #-784]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1886:
	.align	2
.L1885:
	.word	.LANCHOR2
	.word	-4052
	.word	-1726
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
	movw	r3, #514
	ldr	r4, .L1888
	mov	r5, #0
	sub	r6, r4, #1824
	strh	r5, [r4, r3]	@ movhi
	str	r5, [r4, #516]
	mov	r0, r6
	sub	r4, r4, #1776
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
.L1889:
	.align	2
.L1888:
	.word	.LANCHOR2
	.fnend
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L1893
	ldr	r5, .L1893+4
	ldrh	r2, [r5, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1891
	ldr	r3, .L1893+8
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L1892
.L1891:
	bl	FtlCacheWriteBack
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L1893+12
	mov	r4, #0
	strb	r4, [r5, #-1720]
	bl	allocate_data_superblock
	ldr	r3, .L1893+16
	strh	r4, [r5, r3]	@ movhi
	add	r3, r3, #12
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	mov	r0, r4
	bl	FtlEctTblFlush
.L1892:
	ldr	r0, .L1893+12
	ldmfd	sp!, {r3, r4, r5, lr}
	b	get_new_active_ppa
.L1894:
	.align	2
.L1893:
	.word	-1728
	.word	.LANCHOR2
	.word	-1724
	.word	.LANCHOR2-1728
	.word	-936
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L1906
	ldr	r4, .L1906+4
	ldr	r0, [r3, #4068]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldr	r3, .L1906+8
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1904
	bl	FtlLoadSysInfo
	subs	r5, r0, #0
	bne	.L1904
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	ldr	r3, .L1906+12
	ldr	r1, [r4, #-1680]
	ldrh	r2, [r4, r3]
	mov	r3, r5
	b	.L1897
.L1899:
	add	r0, r1, r5
	add	r5, r5, #12
	ldr	r0, [r0, #4]
	cmp	r0, #0
	blt	.L1898
	add	r3, r3, #1
.L1897:
	cmp	r3, r2
	blt	.L1899
.L1898:
	ldr	r4, .L1906+4
	cmp	r3, r2
	ldr	r1, .L1906+16
	ldrh	r0, [r4, r1]
	add	r0, r0, #1
	strh	r0, [r4, r1]	@ movhi
	blt	.L1900
	movw	r3, #514
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	beq	.L1901
.L1900:
	ldr	r3, .L1906+20
	ldr	r1, [r4, #-1844]
	ldr	r2, .L1906+24
	ldrh	r3, [r4, r3]
	ldrh	r0, [r4, r2]
	mov	r3, r3, asl #1
	ldrh	ip, [r1, r3]
	rsb	r0, r0, ip
	strh	r0, [r1, r3]	@ movhi
	ldr	r1, .L1906+28
	ldr	r3, .L1906+32
	ldr	ip, [r4, #-1844]
	ldrh	r0, [r4, r1]
	strh	r0, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #44
	strb	r3, [r4, #-1818]
	ldrh	r0, [r4, r2]
	add	r2, r2, #4
	ldrh	lr, [r4, r2]
	mov	r0, r0, asl #1
	ldrh	r5, [ip, r0]
	rsb	lr, lr, r5
	strh	lr, [ip, r0]	@ movhi
	ldrh	r0, [r4, r1]
	ldr	r1, .L1906+36
	strh	r3, [r4, r2]	@ movhi
	strb	r3, [r4, #-1770]
	strh	r0, [r4, r1]	@ movhi
	ldr	r0, .L1906+40
	bl	FtlMapBlkWriteDumpData
	ldr	r0, .L1906+44
	bl	FtlMapBlkWriteDumpData
	ldr	r3, .L1906+48
	ldrh	r2, [r4, r3]
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1901:
	mov	r0, #1
	ldr	r4, .L1906+4
	bl	FtlUpdateVaildLpn
	ldr	r3, .L1906+20
	ldrh	r0, [r4, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1902
	ldr	r3, .L1906+24
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1902
	ldr	r3, .L1906+52
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1902
	bl	FtlGcRefreshBlock
	ldr	r3, .L1906+56
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshBlock
	sub	r0, r4, #1824
	bl	allocate_new_data_superblock
	sub	r0, r4, #1776
	bl	allocate_new_data_superblock
	add	r0, r4, #472
	bl	FtlMapBlkWriteDumpData
.L1902:
	ldr	r3, .L1906+16
	ldr	r2, .L1906+4
	ldrh	r0, [r2, r3]
	ands	r4, r0, #31
	bne	.L1905
	bl	FtlVpcCheckAndModify
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1904:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1905:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1907:
	.align	2
.L1906:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1552
	.word	-4014
	.word	-1576
	.word	-1824
	.word	-1820
	.word	-4052
	.word	-1822
	.word	-1774
	.word	.LANCHOR2-856
	.word	.LANCHOR2+472
	.word	-1574
	.word	-1772
	.word	-1776
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.L1909:
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mvn	r3, #0
	ldr	r4, .L1911
	mov	r2, #0
	ldr	r1, .L1911+4
	mov	r5, r0
	ldr	r0, .L1911+8
	str	r3, [r4, #2832]
	ldr	r3, .L1911+12
	str	r2, [r3, #652]
	bl	printk
	mov	r0, r5
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldr	r3, .L1911+16
	ldr	r0, [r3, #4068]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L1910
	bl	FtlSysBlkInit
	cmp	r0, #0
	moveq	r3, #1
	streq	r3, [r4, #2832]
.L1910:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1912:
	.align	2
.L1911:
	.word	.LANCHOR1
	.word	.LC75
	.word	.LC74
	.word	.LANCHOR2
	.word	.LANCHOR0
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
	ldr	r4, .L1917
	mov	r5, #0
	sub	r1, r4, #880
	str	r5, [r4, #344]
	str	r5, [r4, #-880]
	str	r0, [r4, #656]
	add	r0, r4, #344
	bl	rknand_get_reg_addr
	ldr	r3, [r4, #344]
	cmp	r3, r5
	mvneq	r4, #0
	beq	.L1914
	bl	rk_nandc_irq_init
	ldr	r0, [r4, #656]
	mov	r1, r5
	mov	r2, r5
	mov	r3, #2048
	bl	FlashSramLoadStore
	bl	rknand_flash_cs_init
	ldr	r0, [r4, #344]
	bl	FlashInit
	subs	r4, r0, #0
	bne	.L1915
	ldr	r0, .L1917+4
	bl	FtlInit
.L1915:
	ldr	r0, .L1917+8
	mov	r1, r4
	bl	printk
.L1914:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1918:
	.align	2
.L1917:
	.word	.LANCHOR2
	.word	.LANCHOR0+2260
	.word	.LC115
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
	ldr	r5, .L1928
	mov	r7, r0
	mov	r4, r1
	ldr	r3, [r5, #-4012]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L1920
	cmp	r1, #31
	movls	r0, #0
	bls	.L1920
	bl	FtlCacheWriteBack
	ldr	r3, .L1928+4
	mov	r0, r7
	ldrh	r6, [r5, r3]
	mov	r1, r6
	bl	__aeabi_uidiv
	mls	r7, r0, r6, r7
	mov	r5, r0
	uxth	r7, r7
	cmp	r7, #0
	beq	.L1921
	rsb	r6, r7, r6
	add	r5, r0, #1
	cmp	r6, r4
	movcs	r6, r4
	uxth	r6, r6
	rsb	r4, r6, r4
.L1921:
	mvn	r3, #0
	ldr	r6, .L1928
	str	r3, [sp, #4]
	ldr	r7, .L1928+4
	b	.L1922
.L1924:
	mov	r0, r5
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #0]
	cmn	r3, #1
	beq	.L1923
	ldr	r3, [r6, #660]
	add	r1, sp, #4
	mov	r2, #1
	mov	r0, r5
	add	r3, r3, #1
	str	r3, [r6, #660]
	ldr	r3, [r6, #-1660]
	add	r3, r3, #1
	str	r3, [r6, #-1660]
	bl	log2phys
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L1923:
	ldrh	r3, [r6, r7]
	add	r5, r5, #1
	rsb	r4, r3, r4
.L1922:
	ldrh	r2, [r6, r7]
	ldr	r3, .L1928
	cmp	r4, r2
	bcs	.L1924
	ldr	r2, [r3, #660]
	mov	r4, #0
	cmp	r2, #32
	bls	.L1927
	str	r4, [r3, #660]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1927:
	mov	r0, r4
.L1920:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L1929:
	.align	2
.L1928:
	.word	.LANCHOR2
	.word	-4046
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
	bl	FtlVpcTblFlush
	mov	r0, #1
	bl	FtlEctTblFlush
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
	ldr	r3, .L1933
	ldr	r3, [r3, #2832]
	cmp	r3, #1
	bne	.L1932
	bl	FtlSysFlush
.L1932:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1934:
	.align	2
.L1933:
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
	.type	rk_ftl_garbage_collect.part.15, %function
rk_ftl_garbage_collect.part.15:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2030
	movw	ip, #65535
	ldr	r2, .L2030+4
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r7, r0
	ldrh	r0, [r3, r2]
	.pad #36
	sub	sp, sp, #36
	cmp	r0, ip
	str	r1, [sp, #20]
	beq	.L1937
	ldr	r1, .L2030+8
	ldrh	r4, [r3, r1]
	cmp	r4, ip
	streqh	r0, [r3, r1]	@ movhi
	mvneq	r1, #0
	streqh	r1, [r3, r2]	@ movhi
.L1937:
	ldr	r2, [r3, #-972]
	cmp	r7, #1
	ldr	r5, .L2030
	add	r2, r2, #1
	add	r2, r2, r7, asl #7
	str	r2, [r3, #-972]
	bne	.L2009
	ldr	r3, [r5, #-1668]
	cmp	r3, #0
	beq	.L2009
	ldr	r3, [r5, #-1608]
	cmp	r3, #29
	bhi	.L2009
	mov	r6, #664
	movw	r4, #65535
	ldrh	r3, [r5, r6]
	add	r2, r2, r3
	str	r2, [r5, #-972]
	bl	FtlGcReFreshBadBlk
	ldr	r3, .L2030+12
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	bne	.L1938
	ldr	r2, .L2030+8
	ldrh	r4, [r5, r2]
	cmp	r4, r3
	movne	r4, r3
	bne	.L1938
	ldr	r3, [r5, #-972]
	cmp	r3, #1024
	bls	.L1938
	mov	r3, #0
	str	r3, [r5, #-972]
	strh	r3, [r5, r6]	@ movhi
	ldr	r3, [r5, #-1608]
	cmp	r3, #0
	moveq	r3, #6
	beq	.L2024
	cmp	r3, #5
	bhi	.L1940
	mov	r3, #18
.L2024:
	strh	r3, [r5, r6]	@ movhi
.L1940:
	mov	r0, #32
	movw	r8, #65535
	bl	List_get_gc_head_node
	uxth	r2, r0
	cmp	r2, r8
	beq	.L1941
	ldr	r5, .L2030
	ldr	r6, .L2030+16
	ldrh	r0, [r5, r6]
	cmp	r0, #0
	moveq	r3, #1
	streqh	r3, [r5, r6]	@ movhi
	beq	.L1941
	ldr	r3, .L2030+20
	mov	ip, #4064
	ldr	lr, .L2030+24
	mov	r2, r2, asl #1
	ldr	r9, [r5, #-1844]
	ldrh	r3, [r5, r3]
	ldrh	ip, [lr, ip]
	ldrh	r1, [r9, r2]
	mul	r3, ip, r3
	add	r3, r3, #1
	cmp	r1, r3
	bgt	.L1941
	add	fp, r0, #1
	mov	sl, #0
	str	sl, [r5, #-964]
	uxth	fp, fp
	strh	fp, [r5, r6]	@ movhi
	str	r2, [sp, #12]
	bl	List_get_gc_head_node
	ldr	r2, [sp, #12]
	uxth	r4, r0
	cmp	r4, r8
	beq	.L1941
	ldrh	r2, [r9, r2]
	mov	r8, r4, asl #1
	ldr	r0, .L2030+28
	mov	r1, fp
	ldrh	r3, [r9, r8]
	str	r2, [sp, #0]
	mov	r2, r4
	bl	printk
	ldrh	r3, [r5, r6]
	cmp	r3, #40
	bls	.L1943
	ldr	r3, [r5, #-1844]
	ldrh	r3, [r3, r8]
	cmp	r3, #32
	strhih	sl, [r5, r6]	@ movhi
.L1943:
	mov	r1, #6
	b	.L2025
.L1941:
	bl	GetSwlReplaceBlock
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	bne	.L1938
	mov	r1, #0
.L2025:
	ldr	r2, .L2030
	mov	r3, #664
	strh	r1, [r2, r3]	@ movhi
	b	.L1938
.L2009:
	movw	r4, #65535
.L1938:
	ldr	r3, .L2030+12
	ldr	r6, .L2030
	ldrh	r2, [r6, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1944
	ldr	r3, .L2030+32
	ldrh	r3, [r6, r3]
	cmp	r3, r2
	bne	.L1944
	ldr	r2, .L2030+8
	ldrh	r8, [r6, r2]
	cmp	r8, r3
	bne	.L1944
	ldr	sl, .L2030+36
	ldr	r2, [r6, #-972]
	ldrh	r3, [r6, sl]
	cmp	r3, #23
	movhi	r3, #1024
	movls	r3, #5120
	cmp	r2, r3
	bls	.L1944
	mov	r2, #664
	mov	r3, #0
	str	r3, [r6, #-972]
	strh	r3, [r6, r2]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r8
	mov	r4, r0
	bne	.L1946
	ldr	r8, .L2030+40
	ldrh	r2, [r6, sl]
	ldrh	r3, [r6, r8]
	cmp	r2, r3
	movcs	r3, #80
	strcsh	r3, [r6, r8]	@ movhi
	bcs	.L1956
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r4
	beq	.L1956
	ldr	r2, [r6, #-772]
	cmp	r2, #0
	bne	.L1949
	ldr	r1, .L2030+24
	movw	r2, #4084
	ldrh	r2, [r1, r2]
	cmp	r2, #3
	beq	.L1949
	ldr	r2, [r6, #-1492]
	cmp	r2, #0
	bne	.L1949
	ldr	r0, [r6, #-1668]
	cmp	r0, #0
	beq	.L1950
.L1949:
	ldr	r2, .L2030
	mov	r3, r3, asl #1
	mov	ip, #4064
	movw	lr, #4084
	ldr	r1, [r2, #-1844]
	ldrh	r0, [r1, r3]
	ldr	r1, .L2030+24
	ldr	r3, .L2030+20
	ldrh	ip, [r1, ip]
	ldrh	r3, [r2, r3]
	ldrh	r1, [r1, lr]
	mul	ip, ip, r3
	cmp	r1, #3
	moveq	r3, r3, lsr #1
	movne	r3, #0
	add	r3, ip, r3
	cmp	r0, r3
	bgt	.L1952
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, .L2030
	ldr	r2, [r3, #-3992]
	ldr	r1, [r3, #-1672]
	add	r2, r2, r2, asl #1
	cmp	r1, r2, lsr #2
	ldr	r2, .L2030+40
	movhi	r1, #128
	movls	r1, #160
	strh	r1, [r3, r2]	@ movhi
	uxth	r4, r0
	b	.L1954
.L1952:
	ldr	r3, .L2030+40
	mov	r1, #128
	strh	r1, [r2, r3]	@ movhi
	b	.L1956
.L1950:
	ldr	r2, [r6, #-1844]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #7
	movhi	r3, #64
	strhih	r3, [r6, r8]	@ movhi
	bhi	.L1956
	bl	List_get_gc_head_node
	mov	r3, #128
	strh	r3, [r6, r8]	@ movhi
	uxth	r4, r0
.L1954:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L1956
.L1946:
	ldr	r1, .L2030
	mov	r0, r4, asl #1
	ldr	r3, .L2030+36
	ldr	ip, [r1, #-1864]
	ldrh	r2, [r1, r3]
	ldr	r3, [r1, #-1844]
	ldrh	r3, [r3, r0]
	ldrh	r0, [ip, r0]
	str	r0, [sp, #0]
	ldr	r0, .L2030+44
	ldrh	r1, [r1, r0]
	ldr	r0, .L2030+48
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L1956:
	bl	FtlGcReFreshBadBlk
.L1944:
	movw	r1, #65535
	rsb	r0, r1, r4
	rsbs	r2, r0, #0
	ldr	r5, .L2030
	adc	r2, r2, r0
	cmp	r7, #0
	movne	r7, #0
	andeq	r7, r2, #1
	cmp	r7, #0
	beq	.L1957
	ldr	r3, .L2030+36
	ldrh	r2, [r5, r3]
	cmp	r2, #24
	movhi	r6, #1
	bhi	.L1958
	sub	r3, r3, #2224
	cmp	r2, #16
	ldrh	r6, [r5, r3]
	movhi	r6, r6, lsr #5
	bhi	.L1958
	cmp	r2, #12
	movhi	r6, r6, lsr #4
	bhi	.L1958
	cmp	r2, #8
	movhi	r6, r6, lsr #2
.L1958:
	ldr	r1, .L2030+44
	ldr	r3, .L2030
	ldrh	r0, [r5, r1]
	cmp	r0, r2
	bcs	.L1962
	ldr	r2, .L2030+32
	movw	r0, #65535
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	bne	.L1963
	ldr	r0, .L2030+8
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	bne	.L1963
	mov	r2, #664
	ldrh	r0, [r3, r2]
	cmp	r0, #0
	bne	.L1964
	ldr	r2, [r3, #-3992]
	ldr	ip, [r3, #-1672]
	add	r2, r2, r2, asl #1
	cmp	ip, r2, lsr #2
	movcs	r2, #18
	bcs	.L2027
.L1964:
	ldr	r3, .L2030
	ldr	r2, .L2030+52
	ldr	r1, .L2030+44
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
.L2027:
	strh	r2, [r3, r1]	@ movhi
	mov	r2, #0
	ldr	r3, .L2030
	str	r2, [r3, #-964]
	b	.L1967
.L1963:
	ldr	r3, .L2030
	ldr	r2, .L2030+52
	ldr	r1, .L2030+44
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L1962:
	ldr	r3, .L2030
	movw	r4, #65535
	ldr	r1, [sp, #20]
	ldr	r2, [r3, #-1668]
	cmp	r1, #2
	movhi	r3, #0
	movls	r3, #1
	cmp	r2, #0
	moveq	r3, #0
	cmp	r3, #0
	addne	r6, r6, #1
	uxthne	r6, r6
	b	.L1968
.L1957:
	ldr	r3, .L2030+32
	ldrh	r3, [r5, r3]
	cmp	r3, r1
	bne	.L1969
	ldr	r1, .L2030+8
	ldrh	r1, [r5, r1]
	cmp	r1, r3
	movne	r2, #0
	andeq	r2, r2, #1
	cmp	r2, #0
	beq	.L1969
	ldr	r2, .L2030+12
	ldrh	r4, [r5, r2]
	cmp	r4, r3
	movne	r4, r3
	bne	.L1969
	ldr	r3, .L2030+36
	mov	r6, #664
	str	r7, [r5, #-964]
	ldrh	r7, [r5, r3]
	add	r3, r3, #852
	ldrh	r2, [r5, r3]
	cmp	r2, r7
	bcs	.L1970
	ldrh	r2, [r5, r6]
	cmp	r2, #0
	bne	.L1971
	ldr	r2, [r5, #-3992]
	ldr	r1, [r5, #-1672]
	add	r2, r2, r2, asl #1
	cmp	r1, r2, lsr #2
	movcs	r2, #18
	strcsh	r2, [r5, r3]	@ movhi
	bcs	.L1973
.L1971:
	ldr	r3, .L2030
	ldr	r2, .L2030+52
	ldr	r1, .L2030+44
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L1973:
	bl	FtlReadRefresh
	mov	r3, #664
	ldr	r2, .L2030
	b	.L2028
.L1970:
	ldrh	r0, [r5, r6]
	cmp	r0, #0
	bne	.L1969
	ldr	r2, .L2030+52
	ldrh	r4, [r5, r2]
	add	r2, r4, r4, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r5, r3]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r5, #-1844]
	ldr	r2, .L2030+20
	mov	r1, #4064
	ldrh	r2, [r5, r2]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	ldr	r0, .L2030+24
	ldrh	r1, [r0, r1]
	mul	r2, r1, r2
	add	r2, r2, r2, lsr #31
	cmp	r3, r2, asr #1
	ble	.L1974
	sub	r4, r4, #1
	cmp	r7, r4
	blt	.L1974
	bl	FtlReadRefresh
	ldrh	r0, [r5, r6]
	b	.L1967
.L1974:
	cmp	r3, #0
	movwne	r4, #65535
	bne	.L1969
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r3, .L2030+36
	ldr	r2, .L2030
	ldrh	r0, [r2, r3]
	add	r0, r0, #1
	b	.L1967
.L1969:
	ldr	r3, .L2030
	ldr	r6, [r3, #-1668]
	cmp	r6, #0
	movne	r6, #2
	moveq	r6, #1
.L1968:
	ldr	r3, .L2030
	movw	r0, #65535
	ldr	r2, .L2030+12
	ldrh	r1, [r3, r2]
	cmp	r1, r0
	bne	.L1975
	cmp	r4, r1
	strneh	r4, [r3, r2]	@ movhi
	bne	.L1977
	ldr	r1, .L2030+8
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	strneh	r0, [r3, r2]	@ movhi
	mvnne	r0, #0
	strneh	r0, [r3, r1]	@ movhi
.L1977:
	ldrh	r0, [r3, r2]
	bl	IsBlkInGcList
	ldr	r5, .L2030
	ldr	r7, .L2030+12
	cmp	r0, #0
	ldrne	r3, .L2030+12
	mvnne	r1, #0
	ldrne	r2, .L2030
	strneh	r1, [r2, r3]	@ movhi
	movw	r3, #65535
	ldrh	r2, [r5, r7]
	cmp	r2, r3
	beq	.L1975
	ldr	r0, .L2030+56
	bl	make_superblock
	ldr	r1, .L2030+60
	movw	r2, #666
	mov	r3, #0
	strh	r3, [r5, r2]	@ movhi
	strh	r3, [r5, r1]	@ movhi
	strb	r3, [r5, #-1534]
	ldrh	r3, [r5, r7]
	ldr	r2, [r5, #-1844]
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	mov	r3, #668
	strh	r2, [r5, r3]	@ movhi
.L1975:
	ldr	r3, .L2030
	ldr	r0, .L2030+12
	ldr	r2, .L2030+64
	ldrh	r1, [r3, r0]
	ldrh	r2, [r3, r2]
	cmp	r2, r1
	mov	r2, r3
	beq	.L1979
	ldr	ip, .L2030+68
	ldrh	r3, [r3, ip]
	cmp	r3, r1
	movwne	fp, #65535
	bne	.L2023
.L1980:
.L1979:
	mvn	r3, #0
	strh	r3, [r2, r0]	@ movhi
	mov	r3, #664
.L2028:
	ldrh	r0, [r2, r3]
	b	.L1967
.L2023:
	ldr	r7, .L2030
	ldr	r9, .L2030+12
	ldrh	r3, [r7, r9]
	cmp	r3, fp
	bne	.L1982
	mov	r3, #0
	str	r3, [r7, #-964]
.L2022:
	ldr	r5, .L2030+16
	ldrh	sl, [r7, r5]
	mov	r0, sl
	bl	List_get_gc_head_node
	uxth	r8, r0
	strh	r8, [r7, r9]	@ movhi
	cmp	r8, fp
	ldreq	r3, .L2030
	moveq	r2, #0
	moveq	r0, #8
	streqh	r2, [r3, r5]	@ movhi
	beq	.L1967
.L1984:
	mov	r0, r8
	add	sl, sl, #1
	bl	IsBlkInGcList
	cmp	r0, #0
	strneh	sl, [r7, r5]	@ movhi
	bne	.L2022
	uxth	sl, sl
	ldr	r0, .L2030+72
	strh	sl, [r7, r5]	@ movhi
	mov	ip, #4064
	ldr	r5, .L2030+24
	mov	r3, r8, asl #1
	ldrh	r0, [r7, r0]
	ldr	r2, [r7, #-1844]
	ldrh	r8, [r5, ip]
	ldrh	r1, [r2, r3]
	mul	r8, r8, r0
	add	r0, r8, r8, lsr #31
	cmp	r1, r0, asr #1
	bgt	.L1987
	cmp	sl, #48
	bls	.L1988
	cmp	r1, #8
	bls	.L1988
	ldr	r1, .L2030+76
	ldrh	r1, [r7, r1]
	cmp	r1, #35
	bhi	.L1988
.L1987:
	ldr	r1, .L2030+16
	mov	r0, #0
	strh	r0, [r7, r1]	@ movhi
.L1988:
	ldrh	r3, [r2, r3]
	ldr	r5, .L2030
	cmp	r3, r8
	cmpge	r4, fp
	movne	r8, #0
	moveq	r8, #1
	bne	.L1989
	ldr	r3, .L2030+12
	mvn	r2, #0
	strh	r2, [r5, r3]	@ movhi
	add	r3, r3, #572
	mov	r2, #0
	strh	r2, [r5, r3]	@ movhi
	b	.L2029
.L1989:
	cmp	r3, #0
	bne	.L1990
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r3, .L2030+16
	ldrh	r2, [r7, r3]
	add	r2, r2, #1
	strh	r2, [r7, r3]	@ movhi
	b	.L2022
.L1990:
	strb	r8, [r5, #-1532]
	ldr	r0, .L2030+56
	bl	make_superblock
	movw	r3, #666
	strh	r8, [r5, r3]	@ movhi
	ldr	r3, .L2030+12
	ldr	r2, [r5, #-1844]
	ldrh	r3, [r5, r3]
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	mov	r3, #668
	strb	r8, [r5, #-1534]
	strh	r2, [r5, r3]	@ movhi
	ldr	r3, .L2030+60
	strh	r8, [r5, r3]	@ movhi
.L1982:
	bl	FtlReadRefresh
	ldr	r3, .L2030
	mov	r2, #1
	ldr	r1, .L2030+72
	mov	r9, #0
	ldr	r5, .L2030
	str	r2, [r3, #-768]
	ldr	r2, .L2030+60
	ldrh	r2, [r3, r2]
	ldrh	r3, [r3, r1]
	add	r1, r2, r6
	cmp	r1, r3
	rsbgt	r6, r2, r3
	uxthgt	r6, r6
	b	.L1992
.L1994:
	ldrh	r1, [r2, #2]!
	add	r3, r3, #1
	cmp	r1, fp
	uxth	r3, r3
	orrne	r1, ip, r1, asl #10
	mlane	sl, r0, r8, lr
	addne	r8, r8, #1
	uxthne	r8, r8
	strne	r1, [sl, #4]
.L2000:
	cmp	r3, r7
	bne	.L1994
	ldr	r0, [r5, #-944]
	mov	r1, r8
	mov	r2, #0
	mov	r7, #0
	bl	FlashReadPages
	str	r7, [sp, #20]
	b	.L1995
.L1998:
	ldr	r3, [r5, #-944]
	add	r2, r3, r7
	ldr	r3, [r3, r7]
	ldr	sl, [r2, #12]
	cmn	r3, #1
	beq	.L1996
	ldrh	r3, [sl, #0]
	movw	r2, #61589
	cmp	r3, r2
	bne	.L1996
	add	r1, sp, #28
	mov	r2, #0
	ldr	r0, [sl, #8]
	bl	log2phys
	ldr	r1, [r5, #-944]
	ldr	r2, [sp, #28]
	add	r1, r1, r7
	ldr	r3, [r1, #4]
	cmp	r2, r3
	bne	.L1996
	movw	r3, #666
	ldr	r0, [r5, #-960]
	ldrh	r2, [r5, r3]
	ldr	r1, [r1, #16]
	add	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	ldr	r3, [r5, #-748]
	mov	r2, #36
	mla	r3, r2, r0, r3
	str	r1, [r3, #16]
	str	r2, [sp, #12]
	str	r3, [sp, #16]
	bl	Ftl_get_new_temp_ppa
	ldr	r3, [sp, #16]
	ldr	r1, [r5, #-960]
	str	r0, [r3, #4]
	ldr	r2, [sp, #12]
	ldr	r3, [r5, #-748]
	mla	r2, r2, r1, r3
	ldr	r3, [r5, #-944]
	add	r3, r3, r7
	ldr	r1, [r3, #8]
	str	r1, [r2, #8]
	mov	r1, #1
	ldr	r3, [r3, #12]
	str	r3, [r2, #12]
	ldr	r3, [sp, #28]
	str	r3, [sl, #12]
	ldr	r3, .L2030+32
	ldrh	r3, [r5, r3]
	strh	r3, [sl, #2]	@ movhi
	ldr	r3, [r5, #-1628]
	ldr	r0, [r5, #-944]
	str	r3, [sl, #4]
	add	r0, r0, r7
	ldr	r3, [r5, #-960]
	add	r3, r3, #1
	str	r3, [r5, #-960]
	bl	FtlGcBufAlloc
	ldrb	r3, [r5, #-1721]	@ zero_extendqisi2
	ldr	r2, [r5, #-960]
	cmp	r2, r3
	beq	.L1997
	ldr	r3, .L2030+80
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L1996
.L1997:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	ldrne	r3, .L2030
	movne	r2, #0
	strne	r2, [r3, #-768]
	movne	r2, #664
	bne	.L2028
.L1996:
	ldr	r1, [sp, #20]
	add	r7, r7, #36
	add	r3, r1, #1
	uxth	r3, r3
	str	r3, [sp, #20]
.L1995:
	ldr	r3, [sp, #20]
	cmp	r3, r8
	bne	.L1998
	add	r9, r9, #1
.L1992:
	uxth	r3, r9
	ldr	r2, .L2030+60
	cmp	r3, r6
	ldr	r3, .L2030
	bcs	.L1999
	ldr	r1, .L2030+24
	mov	r3, #4064
	ldrh	ip, [r5, r2]
	mov	r8, #0
	ldr	lr, [r5, #-944]
	mov	r0, #36
	ldrh	r7, [r1, r3]
	add	ip, ip, r9
	ldr	r2, .L2030+84
	mov	r3, r8
	b	.L2000
.L1999:
	ldrh	r1, [r3, r2]
	add	r6, r6, r1
	uxth	r6, r6
	strh	r6, [r3, r2]	@ movhi
	ldr	r2, .L2030+72
	ldrh	r2, [r3, r2]
	cmp	r2, r6
	bls	.L2001
	movw	r2, #666
	mov	r1, #668
	ldrh	r2, [r3, r2]
	ldrh	r3, [r3, r1]
	cmp	r2, r3
	bne	.L2002
.L2001:
	ldr	r5, .L2030
	ldr	r3, [r5, #-960]
	cmp	r3, #0
	beq	.L2003
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	beq	.L2003
	mov	r3, #0
	str	r3, [r5, #-768]
.L2029:
	mov	r3, #664
	ldrh	r0, [r5, r3]
	b	.L1967
.L2003:
	movw	r2, #666
	ldr	r3, .L2030
	ldrh	ip, [r5, r2]
	cmp	ip, #0
	bne	.L2004
	ldr	r2, .L2030+12
	ldr	r0, [r3, #-1844]
	ldrh	r1, [r3, r2]
	mov	r1, r1, asl #1
	ldrh	r5, [r0, r1]
	cmp	r5, #0
	beq	.L2004
	strh	ip, [r0, r1]	@ movhi
	ldrh	r0, [r3, r2]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2004:
	ldr	r3, .L2030+12
	mvn	r1, #0
	ldr	r2, .L2030
	strh	r1, [r2, r3]	@ movhi
.L2002:
	ldr	r3, .L2030
	ldr	r2, .L2030+36
	ldrh	r2, [r3, r2]
	cmp	r2, #2
	ldrls	r2, .L2030+72
	ldrlsh	r6, [r3, r2]
	bls	.L2023
.L2005:
	mov	r1, #0
	str	r1, [r3, #-768]
	mov	r1, #664
	ldrh	r0, [r3, r1]
	cmp	r0, #0
	addeq	r0, r2, #1
.L1967:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2031:
	.align	2
.L2030:
	.word	.LANCHOR2
	.word	-920
	.word	-922
	.word	-1540
	.word	-968
	.word	-4050
	.word	.LANCHOR0
	.word	.LC116
	.word	-1728
	.word	-1828
	.word	-974
	.word	-976
	.word	.LC117
	.word	-1556
	.word	.LANCHOR2-1540
	.word	-1538
	.word	-1824
	.word	-1776
	.word	-4052
	.word	-936
	.word	-1724
	.word	.LANCHOR2-1526
	.fnend
	.size	rk_ftl_garbage_collect.part.15, .-rk_ftl_garbage_collect.part.15
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L2037
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, [r2, #-768]
	mov	r5, r1
	cmp	r3, #0
	movne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, pc}
	ldr	r1, .L2037+4
	movw	r3, #2868
	ldrh	r1, [r1, r3]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L2034
	ldr	r1, .L2037+8
	ldrh	r2, [r2, r1]
	cmp	r2, r3
	beq	.L2034
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	bne	.L2036
.L2034:
	mov	r0, r4
	mov	r1, r5
	ldmfd	sp!, {r3, r4, r5, lr}
	b	rk_ftl_garbage_collect.part.15
.L2036:
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, pc}
.L2038:
	.align	2
.L2037:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	-1728
	.fnend
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
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
	bne	.L2040
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartWrite
	b	.L2041
.L2040:
	ldr	r7, .L2088
	add	r6, r2, r1
	ldr	r3, [r7, #-4012]
	cmp	r6, r3
	mvnhi	r0, #0
	bhi	.L2041
	mov	r3, #2048
	str	r3, [r7, #672]
	ldr	r3, .L2088+4
	mov	r0, r1
	ldrh	sl, [r7, r3]
	mov	r1, sl
	bl	__aeabi_uidiv
	mov	r1, sl
	str	r0, [sp, #16]
	sub	r0, r6, #1
	bl	__aeabi_uidiv
	ldr	r3, [sp, #16]
	cmp	r5, #8
	sub	r6, r7, #1824
	subls	r6, r7, #1776
	rsb	r9, r3, r0
	str	r0, [sp, #44]
	add	r3, r9, #1
	str	r3, [sp, #12]
	ldr	lr, [sp, #12]
	ldr	r3, [r7, #-1656]
	add	r3, lr, r3
	str	r3, [r7, #-1656]
	ldr	r3, [r7, #-1640]
	add	r3, r5, r3
	str	r3, [r7, #-1640]
	ldr	r3, [r7, #-4000]
	cmp	r3, #0
	beq	.L2075
	ldr	r2, [r7, #-3996]
	sub	r3, r3, #1
	mov	r6, #36
	ldr	r0, [sp, #16]
	mla	r6, r6, r3, r2
	ldr	r3, [r6, #16]
	cmp	r0, r3
	strne	r5, [sp, #28]
	bne	.L2044
	ldr	r3, [r7, #-1652]
	mov	r0, r4
	mov	r1, sl
	add	r3, r3, #1
	str	r3, [r7, #-1652]
	ldr	r3, [r7, #676]
	add	r3, r3, #1
	str	r3, [r7, #676]
	bl	__aeabi_uidivmod
	ldr	r0, [r6, #8]
	rsb	fp, r1, sl
	add	r0, r0, r1, asl #9
	cmp	fp, r5
	movcs	fp, r5
	mov	r1, r8
	mov	r3, fp, asl #9
	str	r3, [sp, #4]
	mov	r2, r3
	bl	memcpy
	cmp	r9, #0
	ldr	r3, [sp, #4]
	bne	.L2045
	ldr	r2, [r7, #676]
	cmp	r2, #2
	movle	r0, r9
	ble	.L2041
.L2045:
	add	r8, r8, r3
	add	r4, r4, fp
	ldr	r3, [sp, #16]
	rsb	r1, fp, r5
	str	r9, [sp, #12]
	add	r3, r3, #1
	str	r1, [sp, #28]
	str	r3, [sp, #16]
.L2044:
	ldr	r3, .L2088
	mov	r2, #0
	ldr	r6, [r3, #648]
	str	r2, [r3, #676]
	b	.L2043
.L2075:
	str	r5, [sp, #28]
.L2043:
	ldr	r0, [sp, #16]
	ldr	r1, [sp, #44]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L2046
	bl	FtlCacheWriteBack
.L2046:
	cmp	r5, sl, asl #1
	ldr	r5, .L2088
	mov	fp, #0
	ldr	r7, [sp, #16]
	movcc	r3, #0
	movcs	r3, #1
	str	fp, [sp, #40]
	str	r3, [sp, #20]
	mov	sl, r5
	str	r6, [r5, #648]
	mov	r9, r8
	b	.L2047
.L2072:
	ldrh	r2, [r6, #4]
	cmp	r2, #0
	bne	.L2048
	ldr	r3, .L2088+8
	ldr	r8, .L2088+12
	cmp	r6, r3
	bne	.L2049
	ldr	r3, .L2088+16
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L2050
	add	r0, r6, #48
	str	r3, [sp, #4]
	bl	allocate_new_data_superblock
	ldr	r3, [sp, #4]
	str	r3, [r8, #2872]
.L2050:
	ldr	r0, .L2088+8
	bl	allocate_new_data_superblock
	ldr	r3, .L2088+12
	ldr	r2, [r3, #2872]
	ldr	r3, .L2088+20
	cmp	r2, #0
	movne	r6, r3
	b	.L2051
.L2049:
	str	r2, [r8, #2872]
	ldr	r2, .L2088+24
	ldrh	r2, [r5, r2]
	cmp	r2, #0
	movne	r6, r3
	bne	.L2051
	mov	r0, r6
	bl	allocate_new_data_superblock
.L2051:
	ldrh	r3, [r6, #4]
	cmp	r3, #0
	bne	.L2052
	mov	r0, r6
	bl	allocate_new_data_superblock
.L2052:
	str	r6, [r5, #648]
.L2048:
	ldr	r1, [r5, #-756]
	mov	r8, r6
	ldr	r2, [r5, #-4000]
	ldrh	r3, [r6, #4]
	rsb	r2, r2, r1
	cmp	r3, r2
	movcc	r2, r3
	ldr	r3, [sp, #12]
	cmp	r2, r3
	movcc	r3, r2
	str	r3, [sp, #36]
	mov	r3, #0
	str	r3, [sp, #24]
	b	.L2053
.L2069:
	ldrh	r3, [r8, #4]
	cmp	r3, #0
	beq	.L2081
	ldr	lr, [sp, #44]
	ldr	r0, [sp, #20]
	rsb	lr, lr, r7
	rsbs	r3, lr, #0
	adc	r3, r3, lr
	tst	r3, r0
	beq	.L2055
	ldr	r1, [sp, #24]
	cmp	r1, #0
	beq	.L2055
	ldr	lr, .L2088+4
	ldr	r0, [sp, #28]
	ldrh	r2, [r5, lr]
	add	r1, r0, r4
	mls	r1, r2, r7, r1
	cmp	r1, r2
	bne	.L2081
.L2055:
	add	r1, sp, #56
	mov	r2, #0
	mov	r0, r7
	str	r3, [sp, #4]
	bl	log2phys
	mov	r0, r8
	bl	get_new_active_ppa
	ldr	r2, [r5, #-4000]
	ldr	r1, [r5, #-3996]
	mov	ip, #36
	ldr	lr, [r5, #-4000]
	mla	r1, ip, r2, r1
	ldr	r2, .L2088+28
	ldrh	r2, [r5, r2]
	str	r7, [r1, #16]
	str	r0, [r1, #4]
	mul	r0, lr, r2
	bic	r0, r0, #3
	str	r0, [sp, #48]
	ldr	r0, [r5, #-712]
	ldr	lr, [sp, #48]
	str	r0, [sp, #52]
	add	r6, r0, lr
	ldr	r0, .L2088+32
	ldr	lr, [r5, #-4000]
	str	r6, [r1, #12]
	ldrh	r0, [r5, r0]
	mul	lr, lr, r0
	ldr	r0, [r5, #-732]
	str	lr, [sp, #32]
	bic	lr, lr, #3
	add	lr, r0, lr
	mov	r0, r6
	str	lr, [r1, #8]
	mov	r1, #0
	str	ip, [sp, #8]
	bl	ftl_memset
	ldr	r0, [sp, #16]
	ldr	r3, [sp, #4]
	rsb	ip, r0, r7
	rsbs	r0, ip, #0
	adc	r0, r0, ip
	str	r0, [sp, #32]
	orrs	r0, r0, r3
	ldr	ip, [sp, #8]
	beq	.L2056
	ldr	r1, [sp, #32]
	cmp	r1, #0
	beq	.L2057
	ldr	r2, .L2088+4
	mov	r0, r4
	ldrh	fp, [r5, r2]
	mov	r1, fp
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #28]
	rsb	fp, r1, fp
	str	r1, [sp, #40]
	cmp	fp, r3
	movcs	fp, r3
	b	.L2058
.L2057:
	cmp	r3, #0
	beq	.L2058
	ldr	lr, .L2088+4
	ldr	r3, [sp, #28]
	add	fp, r3, r4
	ldrh	r3, [r5, lr]
	mls	fp, r3, r7, fp
	ldr	r3, [sp, #32]
	str	r3, [sp, #40]
	uxth	fp, fp
.L2058:
	ldr	lr, .L2088+4
	ldrh	r3, [r5, lr]
	cmp	fp, r3
	bne	.L2059
	ldr	r3, [sp, #32]
	ldr	r0, [sl, #-4000]
	cmp	r3, #0
	ldr	r2, [sl, #-3996]
	ldr	r3, [sp, #20]
	muleq	r1, r7, fp
	movne	r1, r9
	rsbeq	r1, r4, r1
	addeq	r1, r9, r1, asl #9
	cmp	r3, #0
	mov	r3, #36
	mla	r3, r3, r0, r2
	strne	r1, [r3, #8]
	bne	.L2062
	ldr	r0, [r3, #8]
	ldr	r3, .L2088+32
	ldrh	r2, [sl, r3]
	b	.L2086
.L2059:
	ldr	r2, [sp, #56]
	mov	r3, #36
	cmn	r2, #1
	beq	.L2063
	ldr	r1, [r5, #-4000]
	add	r0, sp, #60
	str	r2, [sp, #64]
	ldr	r2, [r5, #-3996]
	str	r7, [sp, #76]
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
	ldreq	r3, [r5, #-1416]
	addeq	r3, r3, #1
	streq	r3, [r5, #-1416]
	beq	.L2065
	ldr	r3, [r6, #8]
	cmp	r3, r7
	beq	.L2065
	ldr	r3, [r5, #-1416]
	mov	r2, r7
	ldr	r0, .L2088+36
	add	r3, r3, #1
	str	r3, [r5, #-1416]
	ldr	r1, [r6, #8]
	bl	printk
	b	.L2065
.L2063:
	ldr	r1, [r5, #-4000]
	ldr	r2, [r5, #-3996]
	mla	r3, r3, r1, r2
	mov	r1, #0
	ldr	r0, [r3, #8]
	ldr	r3, .L2088+32
	ldrh	r2, [r5, r3]
	bl	ftl_memset
.L2065:
	ldr	r3, [sp, #32]
	ldr	r1, [r5, #-4000]
	cmp	r3, #0
	ldr	r2, [r5, #-3996]
	mov	r3, #36
	ldreq	lr, .L2088+4
	mla	r3, r3, r1, r2
	movne	r1, r9
	ldreqh	r1, [r5, lr]
	mov	r2, fp, asl #9
	muleq	r1, r1, r7
	ldrne	r0, [r3, #8]
	ldrne	r3, [sp, #40]
	ldreq	r0, [r3, #8]
	rsbeq	r1, r4, r1
	addne	r0, r0, r3, asl #9
	addeq	r1, r9, r1, asl #9
	b	.L2086
.L2056:
	ldr	r3, [sp, #20]
	ldr	r2, [r5, #-4000]
	cmp	r3, #0
	ldr	r3, [r5, #-3996]
	mla	ip, ip, r2, r3
	beq	.L2067
	ldr	lr, .L2088+4
	ldrh	r3, [r5, lr]
	mul	r3, r3, r7
	rsb	r3, r4, r3
	add	r3, r9, r3, asl #9
	str	r3, [ip, #8]
	b	.L2062
.L2067:
	ldr	r0, .L2088+4
	ldr	r3, .L2088+32
	ldrh	r1, [r5, r0]
	ldrh	r2, [r5, r3]
	ldr	r0, [ip, #8]
	mul	r1, r1, r7
	rsb	r1, r4, r1
	add	r1, r9, r1, asl #9
.L2086:
	bl	memcpy
.L2062:
	ldr	r3, .L2088+40
	ldr	r0, [sp, #52]
	ldr	lr, [sp, #48]
	strh	r3, [r0, lr]	@ movhi
	ldr	r3, [r5, #-1628]
	str	r7, [r6, #8]
	add	r7, r7, #1
	str	r3, [r6, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r5, #-1628]
	moveq	r3, #0
	streq	r3, [r5, #-1628]
	ldr	r3, [sp, #56]
	str	r3, [r6, #12]
	ldrh	r3, [r8, #0]
	strh	r3, [r6, #2]	@ movhi
	ldr	r3, [r5, #-4000]
	add	r3, r3, #1
	str	r3, [r5, #-4000]
	ldr	r3, [sp, #24]
	add	r3, r3, #1
	str	r3, [sp, #24]
.L2053:
	ldr	r3, [sp, #24]
	ldr	lr, [sp, #36]
	cmp	r3, lr
	bne	.L2069
	mov	r6, r8
	b	.L2054
.L2081:
	ldr	r3, [sp, #24]
	mov	r6, r8
	str	r3, [sp, #36]
.L2054:
	ldr	r3, [sp, #12]
	ldr	lr, [sp, #36]
	ldr	r2, [r5, #-4000]
	rsb	r3, lr, r3
	ldr	r0, [sp, #20]
	str	r3, [sp, #12]
	ldr	r3, [r5, #-756]
	cmp	r2, r3
	orrcs	r0, r0, #1
	uxtb	r3, r0
	cmp	r3, #0
	bne	.L2070
	ldrh	r2, [r6, #4]
	cmp	r2, #0
	bne	.L2084
.L2070:
	bl	FtlCacheWriteBack
	ldr	lr, [sp, #12]
	mov	r3, #0
	str	r3, [sl, #-4000]
	cmp	lr, #3
	ldr	r3, [sp, #20]
	movls	r3, #0
.L2084:
	str	r3, [sp, #20]
	bl	rknand_queue_cond_resched
.L2047:
	ldr	r3, [sp, #12]
	cmp	r3, #0
	bne	.L2072
	mov	r0, r3
	ldr	lr, [sp, #16]
	ldr	r3, [sp, #44]
	rsb	r1, lr, r3
	bl	rk_ftl_garbage_collect
	ldr	r0, [sp, #12]
.L2041:
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2089:
	.align	2
.L2088:
	.word	.LANCHOR2
	.word	-4046
	.word	.LANCHOR2-1824
	.word	.LANCHOR1
	.word	-1772
	.word	.LANCHOR2-1776
	.word	-1820
	.word	-4040
	.word	-4042
	.word	.LC118
	.word	-3947
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
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	cmp	r0, #16
	.pad #60
	sub	sp, sp, #60
	mov	r4, r1
	mov	r8, r3
	str	r2, [sp, #24]
	bne	.L2092
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartRead
	str	r0, [sp, #12]
	b	.L2093
.L2092:
	ldr	r3, [sp, #24]
	ldr	r5, .L2122
	add	r3, r3, r1
	str	r3, [sp, #16]
	ldr	r1, [sp, #16]
	ldr	r3, [r5, #-4012]
	cmp	r1, r3
	mvnhi	r3, #0
	strhi	r3, [sp, #12]
	bhi	.L2093
	ldr	r3, .L2122+4
	mov	r0, r4
	ldrh	r6, [r5, r3]
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r3, [sp, #16]
	mov	r1, r6
	str	r0, [sp, #8]
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	r1, [sp, #8]
	ldr	r2, [sp, #24]
	rsb	r3, r1, #1
	add	r3, r3, r0
	str	r3, [sp, #4]
	ldr	r3, [r5, #-1636]
	mov	r9, r0
	ldr	r1, [sp, #4]
	add	r3, r2, r3
	str	r3, [r5, #-1636]
	ldr	r3, [r5, #-1664]
	ldr	r0, [sp, #8]
	add	r3, r1, r3
	mov	r1, r9
	str	r3, [r5, #-1664]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L2094
	bl	FtlCacheWriteBack
.L2094:
	mov	sl, #0
	ldr	r7, [sp, #8]
	str	sl, [sp, #28]
	mov	r6, sl
	str	sl, [sp, #12]
	ldr	r5, .L2122
	b	.L2117
.L2111:
	mov	r0, r7
	add	r1, sp, #52
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #52]
	cmn	r3, #1
	bne	.L2118
	b	.L2121
.L2099:
	mla	r0, r0, r7, fp
	cmp	r0, r4
	bcc	.L2098
	ldr	r2, [sp, #16]
	cmp	r0, r2
	bcs	.L2098
	rsb	r0, r4, r0
	mov	r1, #0
	mov	r2, #512
	str	r3, [sp, #0]
	add	r0, r8, r0, asl #9
	bl	ftl_memset
	ldr	r3, [sp, #0]
.L2098:
	add	fp, fp, #1
	b	.L2096
.L2121:
	ldr	r3, .L2122+4
	mov	fp, #0
.L2096:
	ldrh	r0, [r5, r3]
	cmp	fp, r0
	bcc	.L2099
	b	.L2100
.L2118:
	ldr	r2, [r5, #-752]
	mov	fp, #36
	mla	fp, fp, r6, r2
	str	r3, [fp, #4]
	ldr	r3, [sp, #8]
	cmp	r7, r3
	ldr	r3, .L2122+4
	bne	.L2101
	ldrh	r3, [r5, r3]
	mov	r0, r4
	ldr	r2, [r5, #-728]
	mov	r1, r3
	str	r2, [fp, #8]
	str	r3, [sp, #0]
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #0]
	str	r1, [sp, #32]
	rsb	r2, r1, r3
	ldr	r1, [sp, #24]
	cmp	r2, r1
	movcc	r1, r2
	str	r1, [sp, #28]
	cmp	r1, r3
	streq	r8, [fp, #8]
	b	.L2102
.L2101:
	cmp	r7, r9
	ldrneh	r3, [r5, r3]
	mulne	r3, r3, r7
	bne	.L2119
	ldr	r2, [r5, #-724]
	ldr	r1, [sp, #16]
	str	r2, [fp, #8]
	ldrh	r2, [r5, r3]
	mul	r3, r2, r7
	rsb	sl, r3, r1
	cmp	sl, r2
	bne	.L2102
.L2119:
	rsb	r3, r4, r3
	add	r3, r8, r3, asl #9
	str	r3, [fp, #8]
.L2102:
	ldr	r3, .L2122+8
	ldr	r2, [r5, #-716]
	str	r7, [fp, #16]
	ldrh	r3, [r5, r3]
	mul	r3, r3, r6
	add	r6, r6, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [fp, #12]
.L2100:
	ldr	r3, [sp, #4]
	add	r7, r7, #1
	subs	r3, r3, #1
	str	r3, [sp, #4]
	beq	.L2104
	ldr	r2, .L2122+12
	mov	r3, #4064
	ldrh	r3, [r2, r3]
	cmp	r6, r3, asl #3
	bne	.L2117
.L2104:
	cmp	r6, #0
	beq	.L2117
	ldr	r0, [r5, #-752]
	mov	r1, r6
	mov	r2, #0
	mov	fp, r7
	bl	FlashReadPages
	ldr	r3, [sp, #32]
	mov	r3, r3, asl #9
	str	r3, [sp, #36]
	ldr	r3, [sp, #28]
	mov	r3, r3, asl #9
	str	r3, [sp, #40]
	mov	r3, sl, asl #9
	str	r3, [sp, #44]
	mov	r3, #0
	str	r3, [sp, #20]
.L2110:
	ldr	r3, [sp, #20]
	mov	r1, #36
	mul	r7, r1, r3
	ldr	r3, [r5, #-752]
	ldr	r1, [sp, #8]
	add	r3, r3, r7
	ldr	r2, [r3, #16]
	cmp	r2, r1
	bne	.L2106
	ldr	r1, [r3, #8]
	ldr	r3, [r5, #-728]
	cmp	r1, r3
	bne	.L2107
	ldr	r3, [sp, #36]
	mov	r0, r8
	ldr	r2, [sp, #40]
	add	r1, r1, r3
	b	.L2120
.L2106:
	cmp	r2, r9
	bne	.L2107
	ldr	r1, [r3, #8]
	ldr	r3, [r5, #-724]
	cmp	r1, r3
	bne	.L2107
	ldr	r2, .L2122+4
	ldrh	r0, [r5, r2]
	ldr	r2, [sp, #44]
	mul	r0, r0, r9
	rsb	r0, r4, r0
	add	r0, r8, r0, asl #9
.L2120:
	bl	memcpy
.L2107:
	ldr	r3, [r5, #-752]
	add	r1, r3, r7
	ldr	r2, [r3, r7]
	cmn	r2, #1
	streq	r2, [sp, #12]
	ldreq	r0, [r5, #-1416]
	addeq	r0, r0, #1
	streq	r0, [r5, #-1416]
	ldr	r3, [r3, r7]
	cmp	r3, #256
	bne	.L2109
	ldr	r0, [r1, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L2109:
	ldr	r3, [sp, #20]
	add	r3, r3, #1
	str	r3, [sp, #20]
	cmp	r3, r6
	bne	.L2110
	mov	r7, fp
	mov	r6, #0
.L2117:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	bne	.L2111
	ldr	r3, .L2122+16
	ldr	r2, .L2122
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2093
	ldr	r0, [sp, #4]
	mov	r1, #1
	bl	rk_ftl_garbage_collect
.L2093:
	ldr	r0, [sp, #12]
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2123:
	.align	2
.L2122:
	.word	.LANCHOR2
	.word	-4046
	.word	-4040
	.word	.LANCHOR0
	.word	-918
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
	.global	FlashLoadFactorBbt
	.type	FlashLoadFactorBbt, %function
FlashLoadFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r2, #2274
	ldr	r9, .L2134
	mov	r3, #2272
	ldr	r4, .L2134+4
	.pad #52
	sub	sp, sp, #52
	mov	r1, #0
	mvn	r7, #0
	ldrh	r3, [r9, r3]
	add	r0, r4, #440
	ldrh	r8, [r9, r2]
	mov	r2, #32
	mov	r6, #0
	mul	r8, r8, r3
	bl	ftl_memset
	ldr	r3, [r4, #380]
	str	r6, [sp, #20]
	mov	r4, r6
	str	r3, [sp, #24]
	uxth	r8, r8
	add	r3, r8, r7
	uxth	r3, r3
	str	r3, [sp, #4]
	b	.L2126
.L2132:
	ldr	r5, [sp, #4]
	sub	r3, r8, #16
	mul	fp, r8, r4
	ldr	sl, .L2134+4
	b	.L2127
.L2130:
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
	beq	.L2128
	ldr	r2, [sl, #380]
	ldrh	r1, [r2, #0]
	movw	r2, #61664
	cmp	r1, r2
	bne	.L2128
	ldr	r0, .L2134+8
	mov	r1, r4
	mov	r2, r5
	add	r6, r6, #1
	bl	printk
	ldr	r3, .L2134+4
	uxth	r6, r6
	add	r3, r3, r4, asl #1
	add	r3, r3, #440
	strh	r5, [r3, #0]	@ movhi
	b	.L2129
.L2128:
	sub	r5, r5, #1
	uxth	r5, r5
.L2127:
	cmp	r5, r3
	bgt	.L2130
.L2129:
	ldrb	r3, [r9, #4012]	@ zero_extendqisi2
	add	r4, r4, #1
	cmp	r3, r6
	uxtb	r4, r4
	moveq	r7, #0
.L2126:
	ldrb	r3, [r9, #4012]	@ zero_extendqisi2
	cmp	r3, r4
	bhi	.L2132
	mov	r0, r7
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2135:
	.align	2
.L2134:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC119
	.fnend
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.align	2
	.global	FlashReadFacBbtData
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2146
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r6, r1
	mov	r9, r2
	movw	r1, #2274
	mov	r2, #2272
	ldrh	r8, [r3, r1]
	ldrh	r2, [r3, r2]
	.pad #40
	sub	sp, sp, #40
	ldr	r3, [r3, #4016]
	mov	r4, r0
	ldr	r7, .L2146+4
	mul	r8, r8, r2
	str	r3, [sp, #12]
	ldr	r3, [r7, #380]
	str	r3, [sp, #16]
	uxth	r8, r8
	sub	r5, r8, #1
	mul	sl, r8, r6
	uxth	r5, r5
	sub	r8, r8, #16
	b	.L2137
.L2143:
	mov	r1, #1
	add	r3, r5, sl
	add	r0, sp, #4
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L2138
	ldr	r3, [r7, #380]
	ldrh	r2, [r3, #0]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L2138
	cmp	r4, #0
	moveq	r0, r4
	beq	.L2139
	cmp	r6, #0
	beq	.L2145
	b	.L2141
.L2142:
	ldr	r3, [r0, #4016]
	ubfx	r2, r6, #5, #16
	and	r5, r6, #31
	add	r6, r6, #1
	ldr	lr, [r3, r2, asl #2]
	uxth	r6, r6
	orr	lr, lr, r1, asl r5
	str	lr, [r3, r2, asl #2]
	b	.L2140
.L2145:
	ldr	ip, .L2146+4
	mov	r1, #1
	ldr	r0, .L2146
.L2140:
	ldr	r3, [ip, #364]
	cmp	r6, r3
	bcc	.L2142
.L2141:
	ldr	r3, .L2146
	mov	r2, r9
	mov	r0, r4
	ldr	r1, [r3, #4016]
	bl	memcpy
	mov	r2, #4
	ldr	r0, .L2146+8
	mov	r1, r4
	mov	r3, r2
	bl	rknand_print_hex
	mov	r0, #0
	b	.L2139
.L2138:
	sub	r5, r5, #1
	uxth	r5, r5
.L2137:
	cmp	r5, r8
	bgt	.L2143
	mvn	r0, #0
.L2139:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L2147:
	.align	2
.L2146:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC120
	.fnend
	.size	FlashReadFacBbtData, .-FlashReadFacBbtData
	.align	2
	.global	FlashGetBadBlockList
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2155
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r3, [r3, #2864]
	ldr	r6, .L2155+4
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	ldrh	r4, [r3, #14]
	ldr	r0, [r6, #348]
	mul	r4, r4, r2
	uxth	r4, r4
	add	r2, r4, #7
	mov	r2, r2, lsr #3
	bl	FlashReadFacBbtData
	cmn	r0, #1
	beq	.L2154
	mov	r2, #0
	mov	r0, r4, lsr #4
	mov	r3, r2
	sub	r4, r4, #1
	mov	r1, #1
	b	.L2150
.L2152:
	ldr	ip, [r6, #348]
	mov	r8, r2, lsr #5
	and	r7, r2, #31
	ldr	ip, [ip, r8, asl #2]
	ands	ip, ip, r1, asl r7
	movne	ip, r3, asl #1
	addne	r3, r3, #1
	uxthne	r3, r3
	strneh	r2, [r5, ip]	@ movhi
	cmp	r3, r0
	bcs	.L2154
	add	r2, r2, #1
	uxth	r2, r2
.L2150:
	cmp	r2, r4
	blt	.L2152
	b	.L2149
.L2154:
	mov	r3, #0
.L2149:
	mov	r3, r3, asl #1
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r5, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2156:
	.align	2
.L2155:
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
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	bl	FtlBbtMemInit
	mov	r6, #0
	ldr	r5, .L2175
	bl	FtlLoadFactoryBbt
	add	r8, r5, #12
	add	r4, r5, #3984
	b	.L2158
.L2164:
	ldrh	r3, [r8], #2
	movw	r2, #65535
	ldr	r0, [r4, #-1856]
	ldr	r9, [r4, #-720]
	cmp	r3, r2
	ldr	sl, .L2175+4
	str	r0, [r4, #408]
	str	r9, [r4, #412]
	beq	.L2159
	ldrh	r7, [r4, sl]
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L2175+8
	mla	r7, r7, r6, r3
	mov	r3, r7, asl #10
	str	r3, [r4, #404]
	bl	FlashReadPages
	ldrh	r2, [r4, sl]
	ldr	r0, [r5, #28]
	add	r2, r2, #7
	ldr	r1, [r4, #408]
	mov	r2, r2, lsr #3
	bl	memcpy
	b	.L2160
.L2159:
	mov	r1, r6
	bl	FlashGetBadBlockList
	ldr	r0, [r4, #408]
	ldr	r1, [r5, #28]
	bl	FtlBbt2Bitmap
	ldrh	fp, [r4, sl]
.L2162:
	sub	fp, fp, #1
	uxth	fp, fp
.L2173:
	ldrh	r0, [r4, sl]
	mla	r0, r0, r6, fp
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2162
	mov	r1, #0
	mov	r2, #16
	strh	fp, [r8, #-2]	@ movhi
	ldr	r0, [r4, #-720]
	bl	ftl_memset
	ldr	r2, .L2175+12
	mov	r3, #0
	str	r3, [r9, #4]
	strh	r2, [r9, #0]	@ movhi
	ldrh	r3, [r8, #-2]
	ldrh	r7, [r4, sl]
	strh	r3, [r9, #2]	@ movhi
	ldrh	r3, [r8, #-2]
	ldr	r1, [r5, #28]
	ldr	r0, [r4, #408]
	mla	r7, r7, r6, r3
	mov	r3, r7, asl #10
	str	r3, [r4, #404]
	ldr	r3, .L2175+16
	ldrh	r2, [r4, r3]
	mov	r2, r2, asl #2
	bl	memcpy
	mov	r1, #1
	ldr	r0, .L2175+8
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L2175+8
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #400]
	cmn	r3, #1
	bne	.L2160
	uxth	r0, r7
	bl	FtlBbmMapBadBlock
	b	.L2173
.L2160:
	uxth	r0, r7
	add	r6, r6, #1
	bl	FtlBbmMapBadBlock
	add	r5, r5, #4
.L2158:
	ldr	r2, .L2175+20
	movw	r3, #4086
	ldrh	r3, [r2, r3]
	cmp	r6, r3
	bcc	.L2164
	mov	r4, #0
	ldr	r7, .L2175+24
	ldr	r6, .L2175+28
	b	.L2165
.L2166:
	mov	r0, r4
	add	r4, r4, #1
	bl	FtlBbmMapBadBlock
	uxth	r4, r4
.L2165:
	ldrh	r3, [r7, r6]
	ldr	r5, .L2175+24
	cmp	r3, r4
	bhi	.L2166
	ldr	r7, .L2175+32
	movw	r6, #65535
	ldrh	r4, [r5, r7]
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L2167
.L2172:
	mov	r0, r4
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L2168
	mov	r0, r4
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L2169
	mov	r0, r4
	bl	FtlBbmMapBadBlock
	b	.L2168
.L2169:
	ldr	r3, .L2175+36
	ldrh	r2, [r5, r3]
	cmp	r2, r6
	streqh	r4, [r5, r3]	@ movhi
.L2170:
	ldrne	r2, .L2175+24
	ldrne	r3, .L2175+40
	strneh	r4, [r2, r3]	@ movhi
	bne	.L2171
.L2168:
	sub	r4, r4, #1
	uxth	r4, r4
.L2167:
	ldrh	r3, [r5, r7]
	sub	r3, r3, #48
	cmp	r4, r3
	bgt	.L2172
.L2171:
	ldr	r4, .L2175+24
	mov	r5, #0
	ldr	r7, .L2175+36
	mov	r1, #2
	ldr	r6, .L2175+40
	ldr	r0, [r4, #-1868]
	ldrh	r3, [r4, r7]
	ldr	r8, .L2175+44
	str	r5, [r4, #-3976]
	mov	r3, r3, asl #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, r6]
	strh	r5, [r4, r8]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r0, #40]
	bl	FlashEraseBlocks
	ldrh	r0, [r4, r7]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, r6]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r4, #-3976]
	ldrh	r2, [r4, r6]
	add	r3, r3, #1
	str	r3, [r4, #-3976]
	ldrh	r3, [r4, r7]
	strh	r5, [r4, r8]	@ movhi
	strh	r2, [r4, r7]	@ movhi
	strh	r3, [r4, r6]	@ movhi
	bl	FtlBbmTblFlush
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2176:
	.align	2
.L2175:
	.word	.LANCHOR2-3984
	.word	-4056
	.word	.LANCHOR2+400
	.word	-3872
	.word	-800
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-4038
	.word	-3972
	.word	-3984
	.word	-3980
	.word	-3982
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	FtlLowFormat
	.type	FtlLowFormat, %function
FtlLowFormat:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2197
	mov	r2, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	str	r2, [r3, #-1632]
	str	r2, [r3, #-1628]
	ldr	r3, .L2197+4
	ldr	r0, [r3, #4068]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L2178
	bl	FtlMakeBbt
.L2178:
	mov	r3, #0
	ldr	r2, .L2197
	ldr	r0, .L2197+8
	ldr	r1, .L2197+12
	b	.L2179
.L2180:
	ldr	ip, [r2, #-728]
	mvn	lr, r3
	orr	lr, r3, lr, asl #16
	str	lr, [ip, r3, asl #2]
	ldr	ip, [r2, #-724]
	str	r1, [ip, r3, asl #2]
	add	r3, r3, #1
	uxth	r3, r3
.L2179:
	ldrh	ip, [r2, r0]
	cmp	r3, ip, asl #7
	blt	.L2180
	ldr	r8, .L2197+4
	movw	r3, #4072
	mov	r5, #0
	movw	r7, #4074
	ldrh	r6, [r8, r3]
	b	.L2181
.L2182:
	mov	r0, r6
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L2181:
	ldrh	r3, [r8, r7]
	ldr	r4, .L2197+4
	cmp	r3, r6
	bhi	.L2182
	mov	r3, #4064
	ldrh	r1, [r4, r3]
	sub	r3, r5, #3
	cmp	r3, r1, asl #1
	bge	.L2183
.L2187:
	mov	r4, #0
	ldr	r7, .L2197+4
	mov	r5, r4
	movw	r6, #4072
	b	.L2184
.L2183:
	mov	r0, r5
	movw	r6, #4074
	bl	__aeabi_uidiv
	ldr	r3, .L2197
	ldr	r3, [r3, #-4020]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r0, [r4, #4068]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	movw	r3, #4072
	ldrh	r5, [r4, r3]
	b	.L2185
.L2186:
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
.L2185:
	ldrh	r3, [r4, r6]
	cmp	r3, r5
	bhi	.L2186
	b	.L2187
.L2188:
	mov	r0, r5
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
	add	r4, r4, r0
	uxth	r4, r4
.L2184:
	ldrh	r2, [r7, r6]
	ldr	r3, .L2197+4
	cmp	r2, r5
	bhi	.L2188
	movw	r2, #4074
	ldr	r6, .L2197
	ldrh	r2, [r3, r2]
	ldr	r9, [r3, #4076]
	ldr	sl, .L2197+16
	str	r2, [r6, #-3988]
	mov	r2, #4064
	ldrh	r5, [r3, r2]
	mov	r0, r9
	mov	r1, r5
	bl	__aeabi_uidiv
	ubfx	r8, r0, #5, #16
	mov	r7, r0
	add	r3, r8, #36
	strh	r3, [r6, sl]	@ movhi
	mov	r3, #24
	str	r0, [r6, #-3992]
	mul	r3, r3, r5
	cmp	r4, r3
	ble	.L2189
	rsb	r0, r4, r9
	mov	r1, r5
	bl	__aeabi_uidiv
	str	r0, [r6, #-3992]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r6, sl]	@ movhi
.L2189:
	ldr	r6, .L2197
	ldr	r3, [r6, #-1668]
	cmp	r3, #1
	bne	.L2190
	mov	r0, r4
	mov	r1, r5
	bl	__aeabi_uidiv
	ldr	sl, .L2197+16
	ldrh	r9, [r6, sl]
	uxtah	r0, r9, r0
	add	r9, r9, r0, lsr #2
	strh	r9, [r6, sl]	@ movhi
.L2190:
	ldr	r3, .L2197
	ldr	r2, .L2197+20
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L2191
	ldr	r1, .L2197+16
	ldrh	r0, [r3, r1]
	add	r0, r0, r2, lsr #1
	strh	r0, [r3, r1]	@ movhi
	mul	r0, r5, r2
	cmp	r0, r4
	strgt	r7, [r3, #-3992]
	addgt	r2, r2, #32
	addgt	r8, r8, r2
	strgth	r8, [r3, r1]	@ movhi
.L2191:
	ldr	r4, .L2197
	ldr	r3, .L2197+16
	ldr	r6, .L2197+24
	ldr	r2, [r4, #-3992]
	ldrh	r3, [r4, r3]
	rsb	r3, r3, r2
	mul	r5, r5, r3
	ldr	r3, .L2197+28
	ldrh	r3, [r4, r3]
	str	r5, [r4, #436]
	mul	r5, r3, r5
	ldr	r3, .L2197+8
	ldrh	r3, [r4, r3]
	str	r5, [r4, #-3992]
	mul	r5, r3, r5
	str	r5, [r4, #-4012]
	bl	FtlBbmTblFlush
	ldr	r2, .L2197+4
	movw	r3, #4074
	mov	r1, #0
	ldr	r0, [r4, #-1844]
	ldrh	r2, [r2, r3]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r2, .L2197+32
	mvn	r1, #0
	mov	r3, #0
	str	r3, [r4, #-1672]
	strh	r1, [r4, r2]	@ movhi
	ldr	r1, .L2197+36
	strb	r3, [r4, #-1534]
	strb	r3, [r4, #-1532]
	strh	r3, [r4, r1]	@ movhi
	sub	r1, r1, #284
	strb	r3, [r4, #-1818]
	strh	r3, [r4, r1]	@ movhi
	strh	r3, [r4, r6]	@ movhi
	strb	r3, [r4, #-1816]
.L2192:
	ldr	r5, .L2197
	sub	r0, r5, #1824
	bl	make_superblock
	ldrb	r3, [r4, #-1817]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2193
	ldrh	r3, [r4, r6]
	mvn	r1, #0
	ldr	r2, [r4, #-1844]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r4, r6]
	add	r3, r3, #1
	strh	r3, [r4, r6]	@ movhi
	b	.L2192
.L2193:
	ldr	r3, [r5, #-1632]
	ldr	r1, [r5, #-1844]
	ldr	r6, .L2197+40
	str	r3, [r5, #-1812]
	add	r3, r3, #1
	str	r3, [r5, #-1632]
	ldr	r3, .L2197+44
	ldrh	r0, [r5, r3]
	ldr	r3, .L2197+24
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #1
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	ldr	r0, .L2197+48
	ldrh	r3, [r5, r3]
	strb	r2, [r5, #-1770]
	add	r3, r3, #1
	strh	r2, [r5, r0]	@ movhi
	strh	r3, [r5, r6]	@ movhi
	mov	r3, #1
	strb	r3, [r5, #-1768]
.L2194:
	ldr	r4, .L2197
	sub	r0, r4, #1776
	bl	make_superblock
	ldrb	r3, [r5, #-1769]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2195
	ldrh	r3, [r5, r6]
	mvn	r1, #0
	ldr	r2, [r5, #-1844]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r3, [r5, r6]
	add	r3, r3, #1
	strh	r3, [r5, r6]	@ movhi
	b	.L2194
.L2195:
	ldr	r3, [r4, #-1632]
	mvn	r5, #0
	ldr	r2, [r4, #-1844]
	str	r3, [r4, #-1764]
	add	r3, r3, #1
	str	r3, [r4, #-1632]
	ldr	r3, .L2197+52
	ldrh	r1, [r4, r3]
	ldr	r3, .L2197+40
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, .L2197+56
	strh	r5, [r4, r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, .L2197+60
	mov	r2, #0
	strh	r0, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	ldr	r2, [r4, #436]
	strh	r5, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #-1632]
	str	r3, [r4, #-1544]
	add	r3, r3, #1
	str	r3, [r4, #-1632]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	mov	r0, #0
	ldreq	r3, .L2197+64
	moveq	r2, #1
	streq	r2, [r3, #2832]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L2198:
	.align	2
.L2197:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-4046
	.word	168778952
	.word	-1556
	.word	-4060
	.word	-1824
	.word	-4052
	.word	-1540
	.word	-1538
	.word	-1776
	.word	-1820
	.word	-1774
	.word	-1772
	.word	-1728
	.word	-1552
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
	stmfd	sp!, {r0, r1, r2, r3, r4, lr}
	.save {r0, r1, r2, r3, r4, lr}
	ldr	r4, .L2223
	ldr	r0, [r4, #344]
	bl	FlashInit
	cmp	r0, #0
	movne	r0, #0
	bne	.L2200
	bl	FlashLoadFactorBbt
	cmp	r0, #0
	beq	.L2201
	bl	FlashMakeFactorBbt
.L2201:
	ldr	r0, [r4, #348]
	bl	FlashReadIdbDataRaw
	cmp	r0, #0
	beq	.L2202
	mov	r1, #0
	mov	r2, #16
	mov	r0, sp
	bl	FlashReadFacBbtData
	ldr	r1, [sp, #0]
	mov	r3, #0
	mov	r0, #1
	mov	r2, r3
.L2204:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #16
	bne	.L2204
	cmp	r3, #6
	ldrls	r3, .L2223+4
	bls	.L2219
	mov	r2, #0
	mov	r0, #1
.L2205:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #24
	bne	.L2205
	cmp	r3, #17
	ldr	r3, .L2223+4
	movhi	r2, #36
.L2219:
	strb	r2, [r3, #1]
	movw	r2, #2286
	ldr	r3, .L2223+4
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	strh	r1, [r3, r2]	@ movhi
.L2202:
	ldr	r1, .L2223+8
	mov	r4, #1
	ldr	r0, .L2223+12
	bl	printk
	ldr	r0, .L2223+16
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldr	r3, .L2223+4
	ldr	r0, [r3, #4068]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	b	.L2209
.L2211:
	add	r4, r4, #1
.L2209:
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L2221
.L2210:
	bl	FtlSysBlkInit
	cmp	r0, #0
	beq	.L2212
.L2221:
	bl	FtlLowFormat
	cmp	r4, #3
	bls	.L2211
	b	.L2222
.L2212:
	ldr	r3, .L2223+20
	mov	r2, #1
	str	r2, [r3, #2832]
	b	.L2200
.L2222:
	mvn	r0, #0
.L2200:
	add	sp, sp, #16
	ldmfd	sp!, {r4, pc}
.L2224:
	.align	2
.L2223:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC75
	.word	.LC74
	.word	.LANCHOR0+2260
	.word	.LANCHOR1
	.fnend
	.size	FtlReInitForSDUpdata, .-FtlReInitForSDUpdata
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
	ldr	r6, .L2228
	mov	r5, r0
	str	r2, [sp, #20]
	mov	r9, r1
	ldr	r0, .L2228+4
	mov	r1, r5
	ldr	r2, [r6, #2864]
	ldr	r3, [r6, #4]
	ldr	sl, .L2228+8
	ldrb	r4, [r2, #9]	@ zero_extendqisi2
	mov	r2, r9
	mul	r4, r4, r3
	bl	printk
	ldr	r3, [r6, #2864]
	mov	r0, r5
	ldrb	fp, [r3, #9]	@ zero_extendqisi2
	uxth	r4, r4
	mov	r1, r4
	mov	r4, #0
	bl	__aeabi_uidivmod
	mul	r8, fp, r1
	rsb	r2, r1, r5
	mov	r7, r1
	str	r2, [sp, #16]
	ubfx	r8, r8, #2, #2
	b	.L2226
.L2227:
	ldr	r2, [sp, #16]
	ldr	r1, [r6, #2864]
	add	r3, r8, r2
	add	r2, r4, r7
	ubfx	r2, r2, #2, #16
	ldrb	ip, [r6, #4024]	@ zero_extendqisi2
	ldrb	r1, [r1, #9]	@ zero_extendqisi2
	rsb	r8, r8, #4
	add	r2, r6, r2, asl #1
	ldrb	r0, [sl, #360]	@ zero_extendqisi2
	add	r2, r2, #2288
	ldrh	r2, [r2, #0]
	str	ip, [sp, #0]
	str	r1, [sp, #8]
	mla	r3, fp, r2, r3
	ldr	r2, [sl, #348]
	str	r2, [sp, #4]
	str	r3, [sp, #12]
	bl	FlashBchSel
	ldr	r3, [sp, #12]
	ldr	r1, [sp, #8]
	mov	r0, r3
	bl	__aeabi_uidiv
	ldr	r2, [sp, #4]
	mov	r1, r0
	mov	r0, #0
	mov	r3, r0
	bl	FlashReadPage
	ldr	ip, [sp, #0]
	mov	r0, ip
	bl	FlashBchSel
	ldr	r3, [sp, #20]
	ldr	r1, [sl, #348]
	mov	r2, #2048
	add	r0, r3, r4, asl #9
	add	r4, r4, r8
	bl	memcpy
	mov	r8, #0
	uxth	r4, r4
.L2226:
	cmp	r4, r9
	bcc	.L2227
	mov	r1, r5
	mov	r2, r9
	mov	r3, #0
	ldr	r0, .L2228+12
	bl	printk
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2229:
	.align	2
.L2228:
	.word	.LANCHOR0
	.word	.LC121
	.word	.LANCHOR2
	.word	.LC122
	.fnend
	.size	IdBlockReadData, .-IdBlockReadData
	.align	2
	.global	IDBlockWriteData
	.type	IDBlockWriteData, %function
IDBlockWriteData:
	.fnstart
	@ args = 0, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #84
	sub	sp, sp, #84
	ldr	r6, .L2234
	mov	r4, r0
	str	r2, [sp, #12]
	mov	r7, r1
	ldr	r0, .L2234+4
	mov	r1, r4
	ldr	r2, [r6, #2864]
	ldr	r3, [r6, #4]
	ldrb	r5, [r2, #9]	@ zero_extendqisi2
	mov	r2, r7
	mul	r5, r5, r3
	bl	printk
	movw	r3, #2280
	ldrh	r1, [r6, r3]
	mov	r0, r4
	bl	__aeabi_uidiv
	uxth	r5, r5
	mov	r1, r0
	mov	r0, #0
	mov	r2, r0
	bl	FlashEraseBlock
	ldr	r3, [r6, #2864]
	mov	r1, r5
	mov	r0, r4
	mov	r5, #0
	ldrb	sl, [r3, #9]	@ zero_extendqisi2
	bl	__aeabi_uidivmod
	mov	r9, r1
	rsb	fp, r1, r4
	b	.L2231
.L2233:
	add	r8, r5, r9
	ubfx	r8, r8, #2, #16
	cmp	r8, #0
	beq	.L2232
	add	r3, r8, #1
	add	r3, r6, r3, asl #1
	add	r3, r3, #2288
	ldrh	r3, [r3, #0]
	sub	r3, r3, #1
	mov	r3, r3, asl #2
	str	r3, [sp, #16]
	mov	r3, #0
	str	r3, [sp, #20]
.L2232:
	add	r8, r6, r8, asl #1
	ldr	r1, [sp, #12]
	add	r8, r8, #2288
	ldrb	ip, [r6, #4024]	@ zero_extendqisi2
	add	r2, r1, r5, asl #9
	ldr	r1, .L2234+8
	ldrh	r3, [r8, #0]
	add	r5, r5, #4
	ldrb	r0, [r1, #360]	@ zero_extendqisi2
	uxth	r5, r5
	mla	r3, sl, r3, fp
	str	ip, [sp, #0]
	stmib	sp, {r2, r3}
	bl	FlashBchSel
	ldr	r3, [sp, #8]
	ldr	r1, [r6, #2864]
	mov	r0, r3
	ldrb	r1, [r1, #9]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r2, [sp, #4]
	add	r3, sp, #16
	mov	r1, r0
	mov	r0, #0
	bl	FlashProgPage
	ldr	ip, [sp, #0]
	mov	r0, ip
	bl	FlashBchSel
	mov	r1, sl
	mov	r0, fp
	bl	__aeabi_uidiv
	ldrb	r2, [r8, #0]	@ zero_extendqisi2
	add	r2, r2, #1
	uxtb	r2, r2
	mov	r1, r0
	mov	r0, #0
	bl	FlashPageProgMsbFFData
.L2231:
	cmp	r5, r7
	bcc	.L2233
	mov	r1, r4
	mov	r2, r7
	mov	r3, #0
	ldr	r0, .L2234+12
	bl	printk
	mov	r0, #0
	add	sp, sp, #84
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2235:
	.align	2
.L2234:
	.word	.LANCHOR0
	.word	.LC123
	.word	.LANCHOR2
	.word	.LC124
	.fnend
	.size	IDBlockWriteData, .-IDBlockWriteData
	.align	2
	.global	write_idblock
	.type	write_idblock, %function
write_idblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2249
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r7, r2
	ldr	r2, [r3, #4]
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r3, #2864]
	mov	r6, r0
	mov	r4, r1
	mov	r0, #256000
	mov	r1, #208
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	mul	r3, r3, r2
	mov	r2, #6
	uxth	r3, r3
	str	r3, [sp, #8]
	bl	kmalloc_order_trace
	subs	r5, r0, #0
	mvneq	r0, #0
	beq	.L2237
	add	sl, r6, #508
	add	sl, sl, #3
	mov	sl, sl, lsr #9
	cmp	sl, #255
	bhi	.L2238
	add	r0, r4, sl, asl #9
	mov	r1, r4
	rsb	r2, sl, #256
	bl	memcpy
.L2238:
	ldr	r8, .L2249
	mov	r3, #5
	ldr	r0, .L2249+4
	mov	r1, r7
	mov	r2, #4
	add	sl, sl, #128
	bl	rknand_print_hex
	ldrb	r2, [r8, #1]	@ zero_extendqisi2
	ldr	r1, [r4, #512]
	cmp	sl, #256
	movcs	sl, #256
	ldr	r0, .L2249+8
	mov	fp, #0
	bl	printk
	ldrb	r3, [r8, #1]	@ zero_extendqisi2
	ldr	r2, [r4, #512]
	mov	r1, sl
	ldr	r0, .L2249+12
	mov	r9, fp
	cmp	r2, r3
	mov	r8, r4
	mov	r2, r6
	strhi	r3, [r4, #512]
	mov	r6, r7
	bl	printk
	mov	r1, sl, asl #7
	str	r1, [sp, #12]
.L2244:
	ldr	r1, .L2249
	movw	r2, #2286
	ldr	r3, [r6], #4
	ldrh	r2, [r1, r2]
	cmp	r3, r2
	bcs	.L2240
	ldr	r2, .L2249+16
	ldr	r2, [r2, #364]
	cmp	r3, r2
	bcc	.L2240
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
.L2243:
	ldr	r1, [r5, r3]
	add	r3, r3, #4
	add	r2, r8, r3
	ldr	r2, [r2, #-4]
	cmp	r1, r2
	beq	.L2241
	bic	r4, r7, #255
	stmia	sp, {r1, r2}
	mov	r3, r7
	mov	r1, r9
	ldr	r2, [r6, #-4]
	mov	r4, r4, asl #2
	ldr	r0, .L2249+20
	bl	printk
	ldr	r0, .L2249+24
	add	r1, r8, r4
	mov	r2, #4
	mov	r3, #256
	bl	rknand_print_hex
	mov	r3, #256
	mov	r2, #4
	ldr	r0, .L2249+28
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
	ldr	r0, .L2249+32
	bl	printk
	ldr	r2, [sp, #12]
	cmp	r7, r2
	bcc	.L2240
	b	.L2242
.L2241:
	ldr	r1, [sp, #12]
	add	r7, r7, #1
	cmp	r7, r1
	bne	.L2243
.L2242:
	add	fp, fp, #1
.L2240:
	add	r9, r9, #1
	cmp	r9, #5
	bne	.L2244
	mov	r0, r5
	bl	kfree
	cmp	fp, #0
	mvneq	r0, #0
	movne	r0, #0
.L2237:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2250:
	.align	2
.L2249:
	.word	.LANCHOR0
	.word	.LC125
	.word	.LC126
	.word	.LC127
	.word	.LANCHOR2
	.word	.LC128
	.word	.LC129
	.word	.LC130
	.word	.LC131
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
	ldr	ip, .L2254
	b	.L2252
.L2253:
	ldrb	r4, [r3, r2]	@ zero_extendqisi2
	add	r2, r2, #1
	eor	r4, r4, r0, lsr #24
	add	r4, ip, r4, asl #2
	ldr	r4, [r4, #2876]
	eor	r0, r4, r0, asl #8
.L2252:
	cmp	r2, r1
	bne	.L2253
	ldmfd	sp!, {r4, pc}
.L2255:
	.align	2
.L2254:
	.word	.LANCHOR1
	.fnend
	.size	CRC_32, .-CRC_32
	.align	2
	.type	rknand_sys_storage_ioctl, %function
rknand_sys_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 520
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2360
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	cmp	r1, r3
	.pad #520
	sub	sp, sp, #520
	mov	r6, r1
	mov	r5, r2
	ldr	r4, [r0, #124]
	beq	.L2265
	bhi	.L2274
	ldr	r3, .L2360+4
	cmp	r1, r3
	beq	.L2262
	bhi	.L2275
	sub	r3, r3, #125
	cmp	r1, r3
	beq	.L2259
	bhi	.L2276
	sub	r3, r3, #237
	cmp	r1, r3
	bne	.L2308
	b	.L2356
.L2276:
	ldr	r3, .L2360+8
	cmp	r1, r3
	beq	.L2260
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2308
	b	.L2357
.L2275:
	ldr	r3, .L2360+12
	cmp	r1, r3
	beq	.L2264
	bhi	.L2277
	ldr	r3, .L2360+16
	cmp	r1, r3
	bne	.L2308
	b	.L2358
.L2277:
	ldr	r3, .L2360+20
	cmp	r1, r3
	beq	.L2264
	add	r3, r3, #10
	cmp	r1, r3
	bne	.L2308
	b	.L2264
.L2274:
	ldr	r3, .L2360+24
	cmp	r1, r3
	beq	.L2270
	bhi	.L2278
	sub	r3, r3, #78
	cmp	r1, r3
	beq	.L2267
	bcc	.L2266
	add	r3, r3, #21
	cmp	r1, r3
	beq	.L2268
	add	r3, r3, #56
	cmp	r1, r3
	bne	.L2308
	b	.L2359
.L2278:
	ldr	r3, .L2360+28
	cmp	r1, r3
	beq	.L2273
	bhi	.L2279
	sub	r3, r3, #956
	sub	r3, r3, #1
	cmp	r1, r3
	beq	.L2271
	add	r3, r3, #956
	cmp	r1, r3
	bne	.L2308
	b	.L2272
.L2279:
	ldr	r3, .L2360+32
	cmp	r1, r3
	beq	.L2272
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2308
	b	.L2273
.L2266:
	ldr	r0, .L2360+36
	bl	printk
	mov	r0, r4
	mov	r1, r5
	mov	r2, #512
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2280:
	ldr	r0, .L2360+40
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r6, [r4, #4]
	cmp	r6, #8
	bhi	.L2344
	bl	rknand_device_unlock
	mov	r2, r4
	mov	r1, r6
	ldr	r0, [r4, #0]
	bl	IdBlockReadData
	bl	rknand_device_unlock
	mov	r1, r4
	mov	r0, r5
	mov	r2, r6, asl #9
	bl	rk_copy_to_user
	subs	r4, r0, #0
	ldrne	r0, .L2360+44
	beq	.L2281
	b	.L2354
.L2267:
	ldr	r0, .L2360+48
	bl	printk
	mov	r0, r4
	mov	r1, r5
	mov	r2, #4096
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2282:
	ldr	r5, .L2360+52
	ldr	r0, .L2360+56
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r5, #680]
	cmp	r3, #0
	bne	.L2283
	mov	r0, #260096
	mov	r1, #208
	mov	r2, #6
	bl	kmalloc_order_trace
	cmp	r0, #0
	str	r0, [r5, #680]
	beq	.L2344
.L2283:
	ldr	r2, [r4, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L2344
	ldr	r3, [r4, #0]
	cmp	r3, #251904
	bhi	.L2344
	ldr	r1, .L2360+52
	ldr	r0, [r1, #680]
	add	r1, r4, #8
	add	r0, r0, r3
	bl	memcpy
	b	.L2349
.L2359:
	ldr	r0, .L2360+60
	bl	printk
	mov	r0, r4
	mov	r1, r5
	mov	r2, #28
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2284:
	ldmia	r4, {r1, r2}
	ldr	r0, .L2360+64
	bl	printk
	ldr	r1, [r4, #0]
	cmp	r1, #256000
	bhi	.L2344
	ldr	r5, .L2360+52
	ldr	r0, [r5, #680]
	cmp	r0, #0
	beq	.L2344
	bl	CRC_32
	ldr	r3, [r4, #4]
	cmp	r3, r0
	bne	.L2327
	bl	rknand_device_unlock
	add	r2, r4, #8
	ldr	r1, [r5, #680]
	ldr	r0, [r4, #0]
	mov	r4, #0
	bl	write_idblock
	bl	rknand_device_unlock
	ldr	r0, [r5, #680]
	bl	kfree
	str	r4, [r5, #680]
	b	.L2281
.L2268:
	ldr	r0, .L2360+68
	bl	printk
	mov	r0, r4
	bl	ReadFlashInfo
	mov	r0, r5
	mov	r1, r4
	mov	r2, #11
	b	.L2353
.L2265:
	ldr	r0, .L2360+72
	bl	printk
	bl	rknand_device_unlock
	bl	FtlReInitForSDUpdata
	mov	r6, r0
	bl	rknand_device_unlock
	cmp	r6, #0
	bne	.L2344
	bl	nand_blk_add_whole_disk
	bl	rknand_device_unlock
	mov	r1, r6
	mov	r2, #64
	mov	r0, r4
	bl	FlashReadFacBbtData
	bl	rknand_device_unlock
	ldr	r0, .L2360+76
	mov	r1, r4
	mov	r2, #4
	mov	r3, #8
	bl	rknand_print_hex
	mov	r0, r5
	mov	r1, r4
	mov	r2, #64
	b	.L2353
.L2270:
	ldr	r0, .L2360+80
	bl	printk
	ldr	r3, .L2360+52
	mov	r0, r5
	mov	r1, r4
	mov	r2, #4
	ldr	r3, [r3, #684]
	ldr	r3, [r3, #20]
	str	r3, [r4, #0]
	b	.L2353
.L2271:
	ldr	r0, .L2360+84
	bl	printk
	bl	rknand_device_unlock
	mov	r1, #264
	mov	r2, #2
	mov	r3, r4
	mov	r0, #16
	bl	FtlRead
	bl	rknand_device_unlock
	mov	r0, r5
	mov	r1, r4
	mov	r2, #1024
	b	.L2353
.L2259:
	ldr	r0, .L2360+88
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r4, r0, #0
	bne	.L2351
.L2285:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+92
	cmp	r2, r3
	bne	.L2346
	ldr	r3, [sp, #4]
	cmp	r3, #512
	bhi	.L2346
	ldr	r6, .L2360+52
	mov	r2, #512
	mov	r0, sp
	ldr	r1, [r6, #684]
	bl	memcpy
	ldr	r2, [r6, #688]
	ldr	r3, .L2360+96
	cmp	r2, r3
	beq	.L2286
	add	r0, sp, #64
	mov	r1, #128
	str	r4, [sp, #8]
	str	r4, [sp, #12]
	bl	__memzero
.L2286:
	add	r0, sp, #256
	mov	r1, #256
	mov	r3, #0
	str	r3, [sp, #16]
	bl	__memzero
	b	.L2355
.L2260:
	ldr	r0, .L2360+100
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2287:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+92
	cmp	r2, r3
	bne	.L2346
	ldr	r3, [sp, #4]
	cmp	r3, #512
	bhi	.L2346
	ldr	r4, .L2360+52
	ldr	r3, .L2360+96
	ldr	r2, [r4, #688]
	cmp	r2, r3
	bne	.L2327
	ldr	r3, [sp, #12]
	sub	r2, r3, #1
	cmp	r2, #127
	mvnhi	r4, #2
	bhi	.L2257
	ldr	r0, [r4, #684]
	add	r1, sp, #64
	str	r3, [r0, #12]
	add	r0, r0, #64
	ldr	r2, [sp, #12]
	bl	memcpy
	mov	r0, #1
	ldr	r1, [r4, #684]
	b	.L2350
.L2358:
	ldr	r0, .L2360+104
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2288:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+108
	cmp	r2, r3
	bne	.L2346
	ldr	r3, [sp, #4]
	cmp	r3, #512
	bhi	.L2346
	ldr	r5, .L2360+52
	ldr	r4, [r5, #692]
	cmp	r4, #0
	beq	.L2257
	ldr	r3, [r5, #696]
	ldr	r2, .L2360+112
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2289
	str	r2, [r3, #0]
	mov	r2, #504
	ldr	r3, [r5, #696]
	str	r2, [r3, #4]
	mov	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L2289:
	ldr	r1, [r5, #696]
	mov	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #684]
	ldr	r2, .L2360+92
	ldr	r5, .L2360+52
	ldr	r1, [r3, #0]
	cmp	r1, r2
	strne	r2, [r3, #0]
	ldr	r0, [r5, #684]
	mov	r1, #128
	ldrne	r3, .L2360+52
	movne	r2, #504
	ldrne	r3, [r3, #684]
	stmneib	r3, {r2, r4}
	mov	r4, #0
	str	r4, [r0, #12]
	add	r0, r0, #64
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #684]
	bl	StorageSysDataStore
	str	r4, [r5, #692]
	str	r4, [r5, #688]
	b	.L2281
.L2262:
	ldr	r0, .L2360+116
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r4, r0, #0
	bne	.L2351
.L2291:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+120
	cmp	r2, r3
	bne	.L2346
	ldr	r3, [sp, #4]
	cmp	r3, #512
	bhi	.L2346
	ldr	r5, .L2360+52
	ldr	r3, [r5, #692]
	cmp	r3, #1
	beq	.L2257
	ldr	r3, [r5, #696]
	mov	r0, #0
	ldr	r2, .L2360+112
	ldr	r1, [r3, #0]
	cmp	r1, r2
	strne	r2, [r3, #0]
	ldr	r1, [r5, #696]
	ldrne	r3, [r5, #696]
	movne	r2, #504
	strne	r4, [r3, #12]
	stmneib	r3, {r2, r4}
	mov	r3, #1
	str	r3, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #684]
	ldr	r2, .L2360+92
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2293
	str	r2, [r3, #0]
	mov	r4, #0
	ldr	r3, .L2360+52
	mov	r2, #504
	ldr	r3, [r3, #684]
	stmib	r3, {r2, r4}
.L2293:
	ldr	r5, .L2360+52
	mov	r1, #128
	mov	r4, #0
	ldr	r0, [r5, #684]
	str	r4, [r0, #12]
	add	r0, r0, #64
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #684]
	bl	StorageSysDataStore
	mov	r3, #1
	str	r3, [r5, #692]
	b	.L2281
.L2357:
	ldr	r0, .L2360+124
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2294:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+128
	cmp	r2, r3
	bne	.L2346
	ldr	r2, [sp, #4]
	cmp	r2, #512
	addls	r0, sp, #8
	ldrls	r1, .L2360+132
	bls	.L2352
	b	.L2346
.L2264:
	ldr	r3, .L2360+20
	cmp	r6, r3
	ldreq	r0, .L2360+136
	beq	.L2347
	ldr	r3, .L2360+140
	cmp	r6, r3
	ldreq	r0, .L2360+144
	ldrne	r0, .L2360+148
.L2347:
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2298:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+152
	cmp	r2, r3
	bne	.L2344
	ldr	r3, .L2360+140
	ldr	r4, .L2360+52
	cmp	r6, r3
	bne	.L2299
	ldr	r3, [r4, #684]
	mov	r0, r5
	mov	r1, sp
	mov	r2, #16
	ldr	r3, [r3, #20]
	str	r3, [sp, #4]
	strb	r3, [sp, #8]
	bl	rk_copy_to_user
	cmp	r0, #0
	moveq	r4, r0
	mvnne	r4, #13
	b	.L2257
.L2299:
	ldr	r3, [r4, #1212]
	cmp	r3, #10
	bhi	.L2344
	ldr	r3, [r4, #684]
	ldr	r1, [sp, #4]
	ldr	r2, [r3, #24]
	cmp	r2, r1
	beq	.L2300
	cmp	r2, #0
	beq	.L2300
	ldr	r0, .L2360+156
	bl	printk
	ldr	r3, [r4, #1212]
	add	r3, r3, #1
	str	r3, [r4, #1212]
	b	.L2344
.L2300:
	ldr	r0, .L2360+52
	mov	r2, #0
	str	r2, [r0, #1212]
	ldr	r0, .L2360+20
	cmp	r6, r0
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
	b	.L2281
.L2272:
	ldr	r0, .L2360+160
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2351
.L2303:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+164
	cmp	r2, r3
	bne	.L2346
	ldr	r2, [sp, #4]
	cmp	r2, #504
	bhi	.L2346
	ldr	r3, .L2360+168
	add	r0, sp, #8
	cmp	r6, r3
	ldr	r3, .L2360+52
	ldreq	r1, [r3, #1216]
	ldrne	r1, [r3, #1220]
	add	r1, r1, #8
.L2352:
	bl	memcpy
.L2355:
	mov	r0, r5
	mov	r1, sp
	mov	r2, #520
.L2353:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	bne	.L2344
	b	.L2281
.L2273:
	ldr	r0, .L2360+172
	bl	printk
	mov	r0, sp
	mov	r1, r5
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L2306
.L2351:
	ldr	r0, .L2360+176
.L2354:
	bl	printk
	b	.L2344
.L2306:
	ldr	r2, [sp, #0]
	ldr	r3, .L2360+164
	cmp	r2, r3
	bne	.L2346
	ldr	r2, [sp, #4]
	cmp	r2, #504
	bhi	.L2346
	ldr	r3, .L2360+28
	add	r2, r2, #8
	ldr	r4, .L2360+52
	cmp	r6, r3
	bne	.L2307
	mov	r1, sp
	ldr	r0, [r4, #1216]
	bl	memcpy
	mov	r0, #2
	ldr	r1, [r4, #1216]
	b	.L2350
.L2307:
	mov	r1, sp
	ldr	r0, [r4, #1220]
	bl	memcpy
	ldr	r1, [r4, #1220]
	mov	r0, #3
.L2350:
	bl	StorageSysDataStore
	mov	r4, r0
	b	.L2281
.L2356:
	bl	rknand_dev_flush
.L2349:
	mov	r4, #0
	b	.L2281
.L2346:
	mvn	r4, #0
.L2281:
	mov	r1, r4
	ldr	r0, .L2360+180
	bl	printk
	b	.L2257
.L2308:
	mvn	r4, #21
	b	.L2257
.L2327:
	mvn	r4, #1
	b	.L2257
.L2344:
	mvn	r4, #13
.L2257:
	mov	r0, r4
	add	sp, sp, #520
	ldmfd	sp!, {r4, r5, r6, pc}
.L2361:
	.align	2
.L2360:
	.word	1074033155
	.word	1074029694
	.word	1074029570
	.word	1074031656
	.word	1074029695
	.word	1074031666
	.word	1074033235
	.word	1074034193
	.word	1074034194
	.word	.LC132
	.word	.LC134
	.word	.LC135
	.word	.LC136
	.word	.LANCHOR2
	.word	.LC137
	.word	.LC138
	.word	.LC139
	.word	.LC140
	.word	.LC141
	.word	.LC142
	.word	.LC143
	.word	.LC144
	.word	.LC145
	.word	1263358532
	.word	-1067903959
	.word	.LC146
	.word	.LC147
	.word	1112753220
	.word	1146313043
	.word	.LC148
	.word	1112755781
	.word	.LC149
	.word	1094995539
	.word	.LANCHOR2+700
	.word	.LC150
	.word	1074031676
	.word	.LC151
	.word	.LC152
	.word	1280262987
	.word	.LC153
	.word	.LC154
	.word	1145980246
	.word	1074034192
	.word	.LC155
	.word	.LC133
	.word	.LC156
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
	ldr	r4, .L2365
	mov	r6, #0
	add	r0, r4, #700
	ldr	r5, [r4, #656]
	str	r6, [r4, #680]
	add	r1, r5, #1536
	add	r3, r5, #512
	str	r5, [r4, #696]
	str	r3, [r4, #684]
	add	r3, r5, #1024
	str	r1, [r4, #1220]
	str	r3, [r4, #1216]
	bl	memcpy
	ldr	r7, [r5, #508]
	ldr	r3, [r5, #16]
	cmp	r7, r6
	str	r6, [r4, #688]
	str	r6, [r4, #1212]
	str	r3, [r4, #692]
	beq	.L2363
	mov	r0, r5
	mov	r1, #508
	bl	JSHash
	cmp	r7, r0
	beq	.L2363
	str	r6, [r5, #16]
	ldr	r0, .L2365+4
	str	r6, [r4, #692]
	bl	printk
.L2363:
	ldr	r3, [r4, #692]
	mov	r0, #2
	ldr	r4, .L2365
	cmp	r3, #0
	ldrne	r2, .L2365+8
	ldrne	r3, .L2365
	ldr	r1, [r4, #1216]
	strne	r2, [r3, #688]
	bl	StorageSysDataLoad
	ldr	r1, [r4, #1220]
	mov	r0, #3
	bl	StorageSysDataLoad
	ldr	r0, .L2365+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	misc_register
.L2366:
	.align	2
.L2365:
	.word	.LANCHOR2
	.word	.LC157
	.word	-1067903959
	.word	.LANCHOR1+3900
	.fnend
	.size	rk_ftl_storage_sys_init, .-rk_ftl_storage_sys_init
	.align	2
	.global	StorageSysDataDeInit
	.type	StorageSysDataDeInit, %function
StorageSysDataDeInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r0, .L2368
	bl	misc_deregister
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2369:
	.align	2
.L2368:
	.word	.LANCHOR1+3900
	.fnend
	.size	StorageSysDataDeInit, .-StorageSysDataDeInit
	.global	rknand_sys_storage_fops
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
	.global	gNandcVer
	.global	gNandChipMap
	.global	gNandIDataBuf
	.global	FlashDdrTunningReadCount
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
	.global	gNandc1_enable
	.section	.rodata
	.set	.LANCHOR3,. + 0
	.type	__func__.14331, %object
	.size	__func__.14331, 11
__func__.14331:
	.ascii	"FtlMemInit\000"
.LC0:
	.byte	40
	.byte	60
	.byte	24
	.byte	16
	.type	__func__.15185, %object
	.size	__func__.15185, 21
__func__.15185:
	.ascii	"FtlVpcCheckAndModify\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC1:
	.ascii	"Context allocation failed\012\000"
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
	.ascii	"FTL version: 5.0.31 20160315\000"
.LC76:
	.ascii	"GetSwlReplaceBlock min_ec_id =%x %x\012\000"
.LC77:
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC78:
	.ascii	"FtlGcRefreshBlock  0x%x \012\000"
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
	.ascii	"ECC:%d\012\000"
.LC88:
	.ascii	"sdr read ok %x ecc=%d\012\000"
.LC89:
	.ascii	"sync para %d\012\000"
.LC90:
	.ascii	"TOG mode Read error %x %x\012\000"
.LC91:
	.ascii	"FlashLoadPhyInfo fail %x!!\012\000"
.LC92:
	.ascii	"read retry status %x %x %x\012\000"
.LC93:
	.ascii	"ReadRetry pageadd=%x  ecc=%x err=%x\012\000"
.LC94:
	.ascii	"Read pageadd=%x  ecc=%x err=%x\012\000"
.LC95:
	.ascii	"spare:\000"
.LC96:
	.ascii	"slc mode\000"
.LC97:
	.ascii	"no ect\000"
.LC98:
	.ascii	"prog error: = %x\012\000"
.LC99:
	.ascii	"prog read error: = %x\012\000"
.LC100:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC101:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC102:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC103:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC104:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
.LC105:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC106:
	.ascii	"FlashMakeFactorBbt %d\012\000"
.LC107:
	.ascii	"bad block:%d %d\012\000"
.LC108:
	.ascii	"FMFB:%d %d\012\000"
.LC109:
	.ascii	"E:bad block:%d\012\000"
.LC110:
	.ascii	"FMFB:Save %d %d\012\000"
.LC111:
	.ascii	"spuer block %x vpn is 0\012 \000"
.LC112:
	.ascii	"...%s enter...\012\000"
.LC113:
	.ascii	"FtlCheckVpc %x = %x  %x\012\000"
.LC114:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC115:
	.ascii	"FtlInit %x\012\000"
.LC116:
	.ascii	"%d GC datablk  = %x vpc %x %x\012\000"
.LC117:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC118:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
.LC119:
	.ascii	"FLFB:%d %d\012\000"
.LC120:
	.ascii	"BBT:\000"
.LC121:
	.ascii	"IdBlockReadData %x %x\012\000"
.LC122:
	.ascii	"IdBlockReadData %x %x ret= %x\012\000"
.LC123:
	.ascii	"IDBlockWriteData %x %x\012\000"
.LC124:
	.ascii	"IDBlockWriteData %x %x ret= %x\012\000"
.LC125:
	.ascii	"idblk:\000"
.LC126:
	.ascii	"idb reverse %x %x\012\000"
.LC127:
	.ascii	"write_idblock totle_sec %x %x\012\000"
.LC128:
	.ascii	"write and check error:%d idb=%x,offset=%x,r=%x,w=%x"
	.ascii	" \012\000"
.LC129:
	.ascii	"write\000"
.LC130:
	.ascii	"read\000"
.LC131:
	.ascii	"write_idblock error\012\000"
.LC132:
	.ascii	"READ_SECTOR_IO\012\000"
.LC133:
	.ascii	"rk_copy_from_user error \012\000"
.LC134:
	.ascii	"READ_SECTOR_IO %x %x\012\000"
.LC135:
	.ascii	"rk_copy_to_user error\012\000"
.LC136:
	.ascii	"WRITE_SECTOR_IO\012\000"
.LC137:
	.ascii	"WRITE_SECTOR_IO %x %x\012\000"
.LC138:
	.ascii	"END_WRITE_SECTOR_IO\012\000"
.LC139:
	.ascii	"END_WRITE_SECTOR_IO %x %x\012\000"
.LC140:
	.ascii	"GET_FLASH_INFO_IO\012\000"
.LC141:
	.ascii	"GET_BAD_BLOCK_IO\012\000"
.LC142:
	.ascii	"bbt:\000"
.LC143:
	.ascii	"GET_LOCK_FLAG_IO\012\000"
.LC144:
	.ascii	"GET_PUBLIC_KEY_IO\012\000"
.LC145:
	.ascii	"RKNAND_GET_DRM_KEY\012\000"
.LC146:
	.ascii	"RKNAND_STORE_DRM_KEY\012\000"
.LC147:
	.ascii	"RKNAND_DIASBLE_SECURE_BOOT\012\000"
.LC148:
	.ascii	"RKNAND_ENASBLE_SECURE_BOOT\012\000"
.LC149:
	.ascii	"RKNAND_GET_SN_SECTOR\012\000"
.LC150:
	.ascii	"RKNAND_LOADER_UNLOCK\012\000"
.LC151:
	.ascii	"RKNAND_LOADER_STATUS\012\000"
.LC152:
	.ascii	"RKNAND_LOADER_LOCK\012\000"
.LC153:
	.ascii	"LockKey not match %d\012\000"
.LC154:
	.ascii	"RKNAND_GET_VENDOR_SECTOR\012\000"
.LC155:
	.ascii	"RKNAND_STORE_VENDOR_SECTOR\012\000"
.LC156:
	.ascii	"return ret = %lx\012\000"
.LC157:
	.ascii	"secureBootEn check error\012\000"
.LC158:
	.ascii	"rknand_sys_storage\000"
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
	.size	NandFlashParaTbl, 2176
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.short	1497
	.byte	2
	.byte	6
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.space	5
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
	.short	1497
	.byte	2
	.byte	5
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	3
	.byte	0
	.byte	0
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.space	5
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
	.type	rknand_sys_storage_dev, %object
	.size	rknand_sys_storage_dev, 36
rknand_sys_storage_dev:
	.word	255
	.word	.LC158
	.word	rknand_sys_storage_fops
	.space	24
	.type	rknand_sys_storage_fops, %object
	.size	rknand_sys_storage_fops, 112
rknand_sys_storage_fops:
	.space	36
	.word	rknand_sys_storage_ioctl
	.word	rknand_sys_storage_ioctl
	.space	4
	.word	rknand_sys_storage_open
	.space	4
	.word	rknand_sys_storage_release
	.space	52
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
	.type	gNandcVer, %object
	.size	gNandcVer, 4
gNandcVer:
	.space	4
	.type	gNandChipMap, %object
	.size	gNandChipMap, 64
gNandChipMap:
	.space	64
	.type	gNandOptPara, %object
	.size	gNandOptPara, 32
gNandOptPara:
	.space	32
	.type	gNandIDataBuf, %object
	.size	gNandIDataBuf, 2048
gNandIDataBuf:
	.space	2048
	.type	IDByte, %object
	.size	IDByte, 64
IDByte:
	.space	64
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
	.size	DieAddrs, 64
DieAddrs:
	.space	64
	.type	gpNandParaInfo, %object
	.size	gpNandParaInfo, 4
gpNandParaInfo:
	.space	4
	.type	gpNandc1, %object
	.size	gpNandc1, 4
gpNandc1:
	.space	4
	.type	gToggleModeClkDiv, %object
	.size	gToggleModeClkDiv, 4
gToggleModeClkDiv:
	.space	4
	.type	gDieOp, %object
	.size	gDieOp, 256
gDieOp:
	.space	256
	.type	gReadRetryInfo, %object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.space	852
	.type	read_retry_cur_offset, %object
	.size	read_retry_cur_offset, 8
read_retry_cur_offset:
	.space	8
	.type	g_maxRegNum, %object
	.size	g_maxRegNum, 1
g_maxRegNum:
	.space	1
	.type	g_retryMode, %object
	.size	g_retryMode, 1
g_retryMode:
	.space	1
	.type	gFlashInterfaceMode, %object
	.size	gFlashInterfaceMode, 1
gFlashInterfaceMode:
	.space	1
	.type	gFlashToggleModeEn, %object
	.size	gFlashToggleModeEn, 1
gFlashToggleModeEn:
	.space	1
	.type	DieCsIndex, %object
	.size	DieCsIndex, 16
DieCsIndex:
	.space	16
	.type	gNandMaxDie, %object
	.size	gNandMaxDie, 1
gNandMaxDie:
	.space	1
	.space	3
	.type	gFlashPageBuffer0, %object
	.size	gFlashPageBuffer0, 4
gFlashPageBuffer0:
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
	.type	gNandc1_enable, %object
	.size	gNandc1_enable, 1
gNandc1_enable:
	.space	1
	.space	1
	.type	gMasterInfo, %object
	.size	gMasterInfo, 32
gMasterInfo:
	.space	32
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
	.type	RK29_NANDC1_REG_BASE, %object
	.size	RK29_NANDC1_REG_BASE, 4
RK29_NANDC1_REG_BASE:
	.space	4
	.type	gBootDdrMode, %object
	.size	gBootDdrMode, 4
gBootDdrMode:
	.space	4
	.type	gMasterTempBuf, %object
	.size	gMasterTempBuf, 4
gMasterTempBuf:
	.space	4
	.type	gNandcDumpWriteEn, %object
	.size	gNandcDumpWriteEn, 4
gNandcDumpWriteEn:
	.space	4
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
	.size	FbbtBlk, 32
FbbtBlk:
	.space	32
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
	.type	g_LowFormat, %object
	.size	g_LowFormat, 4
g_LowFormat:
	.space	4
	.type	gLoaderBootInfo, %object
	.size	gLoaderBootInfo, 4
gLoaderBootInfo:
	.space	4
	.type	gc_discard_updated, %object
	.size	gc_discard_updated, 4
gc_discard_updated:
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
