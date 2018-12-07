/*
 * Copyright (c) 2016-2018, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-01-07
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
	.type	FlashGetReadRetryDefault.part.28, %function
FlashGetReadRetryDefault.part.28:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #33
	cmpne	r0, #65
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	beq	.L2
	cmp	r0, #66
	bne	.L3
.L2:
	ldr	r3, .L8
	mov	r2, #4
	strb	r0, [r3, #0]
	b	.L7
.L3:
	cmp	r0, #34
	cmpne	r0, #67
	bne	.L5
	ldr	r3, .L8
	mov	r2, #5
	strb	r0, [r3, #0]
.L7:
	strb	r2, [r3, #1]
	add	r0, r3, #4
	mov	r2, #7
	ldr	r1, .L8+4
	strb	r2, [r3, #2]
	mov	r2, #45
	b	.L6
.L5:
	cmp	r0, #35
	cmpne	r0, #68
	ldmnefd	sp!, {r3, pc}
	ldr	r3, .L8
	mov	r2, #5
	ldr	r1, .L8+8
	strb	r0, [r3, #0]
	add	r0, r3, #4
	strb	r2, [r3, #1]
	mov	r2, #17
	strb	r2, [r3, #2]
	mov	r2, #95
.L6:
	bl	memcpy
	ldmfd	sp!, {r3, pc}
.L9:
	.align	2
.L8:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR1+45
	.fnend
	.size	FlashGetReadRetryDefault.part.28, .-FlashGetReadRetryDefault.part.28
	.align	2
	.global	FlashMemCmp8
	.type	FlashMemCmp8, %function
FlashMemCmp8:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L20
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrb	r3, [r3, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L18
	ldrb	r3, [r1, #1]	@ zero_extendqisi2
	ldrb	ip, [r0, #1]	@ zero_extendqisi2
	cmp	ip, r3
	movne	r3, #0
	bne	.L18
	b	.L19
.L15:
	ldrb	r4, [r0, r3]	@ zero_extendqisi2
	ldrb	ip, [r1, r3]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r4, ip
	beq	.L18
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L18:
	cmp	r3, r2
	bne	.L15
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L19:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L21:
	.align	2
.L20:
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
	ldr	r3, .L23
	ldrb	r2, [r3, #853]	@ zero_extendqisi2
	ldr	r3, [r3, #856]
	mul	r3, r3, r2
	cmp	r1, r3
	movcs	r2, #0
	movcc	r2, #1
	cmp	r0, #0
	movne	r2, #0
	eor	r0, r2, #1
	bx	lr
.L24:
	.align	2
.L23:
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
	and	r2, r1, #127
	ldr	r3, .L27
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	r3, r3, r2, asl #1
	ldrh	r4, [r3, #140]
	ldr	r3, .L27+4
	ldrb	r3, [r3, #860]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L26
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r4, r4, #-1073741824
.L26:
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L28:
	.align	2
.L27:
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
	and	r2, r1, #127
	ldr	r3, .L31
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r3, r3, r2, asl #1
	mov	r6, r0
	ldrh	r4, [r3, #140]
	ldr	r3, .L31+4
	ldrb	r2, [r3, #860]	@ zero_extendqisi2
	mov	r5, r3
	cmp	r2, #0
	beq	.L30
	bl	FlashRsvdBlkChk
	cmp	r0, #0
	orrne	r4, r4, #-1073741824
.L30:
	add	r5, r5, r6, asl #3
	ldr	r3, [r5, #864]
	str	r4, [r3, #336]
	ldmfd	sp!, {r4, r5, r6, pc}
.L32:
	.align	2
.L31:
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
	ldr	r3, .L37
	cmp	r0, #256
	movhi	r2, #512
	str	r0, [r3, #856]
	bhi	.L36
	cmp	r0, #128
	bxls	lr
	mov	r2, #256
.L36:
	str	r2, [r3, #856]
	bx	lr
.L38:
	.align	2
.L37:
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
	ldr	ip, .L40
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, ip, r0, asl #3
	ldrb	r3, [ip, #868]	@ zero_extendqisi2
	ldr	r4, [ip, #864]
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
.L41:
	.align	2
.L40:
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
	ldr	r2, .L46
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, r2, r0, asl #3
	ldrb	r2, [r2, #912]	@ zero_extendqisi2
	ldrb	r3, [ip, #868]	@ zero_extendqisi2
	ldr	r4, [ip, #864]
	cmp	r2, #1
	add	r3, r3, #8
	mov	r2, #0
	add	r3, r4, r3, asl #8
	bne	.L43
	mov	ip, #6
	str	ip, [r3, #8]
	str	r2, [r3, #4]
	str	r2, [r3, #4]
	uxtb	r2, r1
	str	r2, [r3, #4]
	mov	r2, r1, lsr #8
	str	r2, [r3, #4]
	mov	r2, r1, lsr #16
	b	.L45
.L43:
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
.L45:
	str	r2, [r3, #4]
	mov	r2, #224
	str	r2, [r3, #8]
	ldmfd	sp!, {r4, lr}
	b	FlashSetRandomizer
.L47:
	.align	2
.L46:
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
	ldr	ip, .L49
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	add	ip, ip, r0, asl #3
	ldrb	r3, [ip, #868]	@ zero_extendqisi2
	ldr	r4, [ip, #864]
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
.L50:
	.align	2
.L49:
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
	ldr	r3, .L53
	cmp	r2, #0
	add	r0, r3, r0, asl #3
	ldr	ip, [r0, #864]
	ldrb	r0, [r0, #868]	@ zero_extendqisi2
	add	r0, r0, #8
	add	r0, ip, r0, asl #8
	beq	.L52
	ldr	r3, [r3, #856]
	mov	r2, #96
	str	r2, [r0, #8]
	uxtb	r2, r1
	str	r2, [r0, #4]
	mov	r2, r1, lsr #8
	str	r2, [r0, #4]
	mov	r2, r1, lsr #16
	add	r1, r1, r3
	str	r2, [r0, #4]
.L52:
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
.L54:
	.align	2
.L53:
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
	ldr	ip, .L56
	mov	r2, r1, lsr #16
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	add	r4, ip, r0, asl #3
	ldrb	ip, [ip, #907]	@ zero_extendqisi2
	ldrb	r3, [r4, #868]	@ zero_extendqisi2
	ldr	r5, [r4, #864]
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
.L57:
	.align	2
.L56:
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
	ldr	r3, .L59
	add	r3, r3, r0, asl #3
	ldr	r2, [r3, #864]
	ldrb	r3, [r3, #868]	@ zero_extendqisi2
	add	r3, r3, #8
	add	r3, r2, r3, asl #8
	mov	r2, #16
	str	r2, [r3, #8]
	bx	lr
.L60:
	.align	2
.L59:
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
	ldr	r3, .L62
	add	r0, r3, r0, asl #3
	ldrb	r3, [r3, #906]	@ zero_extendqisi2
	ldrb	r2, [r0, #868]	@ zero_extendqisi2
	ldr	r1, [r0, #864]
	add	r2, r2, #8
	add	r2, r1, r2, asl #8
	str	r3, [r2, #8]
	bx	lr
.L63:
	.align	2
.L62:
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
	mov	r3, r0
	mov	r2, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r0, .L67
	b	.L65
.L66:
	mov	ip, r0, asl #5
	ldrb	r4, [r3, r2]	@ zero_extendqisi2
	add	ip, ip, r0, lsr #2
	add	r2, r2, #1
	add	ip, ip, r4
	eor	r0, r0, ip
.L65:
	cmp	r2, r1
	bne	.L66
	ldmfd	sp!, {r4, pc}
.L68:
	.align	2
.L67:
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
	ldr	r1, .L72
	bl	memcpy
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L73:
	.align	2
.L72:
	.word	.LANCHOR0+928
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
	ldr	r6, .L86
.L78:
	add	r5, r6, r4, asl #5
	ldr	r1, .L86+4
	add	r0, r5, #1
	ldrb	r2, [r6, r4, asl #5]	@ zero_extendqisi2
	bl	FlashMemCmp8
	subs	r1, r0, #0
	bne	.L75
	cmp	r5, #0
	ldrneb	r2, [r5, #22]	@ zero_extendqisi2
	ldrne	r3, .L86+8
	bne	.L80
	b	.L85
.L75:
	add	r4, r4, #1
	cmp	r4, #81
	bne	.L78
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L80:
	ldrb	r0, [r3, r1, asl #5]	@ zero_extendqisi2
	cmp	r0, r2
	beq	.L79
	add	r1, r1, #1
	cmp	r1, #4
	bne	.L80
.L79:
	ldr	r3, .L86+8
	mov	r2, #32
	ldr	r0, .L86+12
	add	r1, r3, r1, asl #5
	bl	memcpy
	mov	r1, r5
	mov	r2, #32
	ldr	r0, .L86+16
	bl	memcpy
	ldr	r2, .L86+20
	movw	r3, #3126
	ldrh	r0, [r2, r3]
	bl	FlashBlockAlignInit
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L85:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L87:
	.align	2
.L86:
	.word	.LANCHOR1+396
	.word	.LANCHOR0+2976
	.word	.LANCHOR1+2988
	.word	.LANCHOR0+896
	.word	.LANCHOR1+3116
	.word	.LANCHOR1
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
	ldr	r3, .L89
	ldr	r2, [r3, #3008]
	ldr	r1, [r2, #0]
	str	r1, [r3, #3012]
	ldr	r1, [r2, #4]
	str	r1, [r3, #3016]
	ldr	r1, [r2, #8]
	str	r1, [r3, #3020]
	ldr	r1, [r2, #12]
	str	r1, [r3, #3024]
	ldr	r1, [r2, #304]
	str	r1, [r3, #3028]
	ldr	r1, [r2, #308]
	str	r1, [r3, #3032]
	ldr	r1, [r2, #336]
	ldr	r2, [r2, #344]
	str	r1, [r3, #3036]
	str	r2, [r3, #3040]
	bx	lr
.L90:
	.align	2
.L89:
	.word	.LANCHOR0
	.fnend
	.size	ftl_flash_suspend, .-ftl_flash_suspend
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
	ldr	r3, .L96
	mov	r5, r1
	mov	r6, r2
	movw	r1, #3058
	mov	r2, #3056
	ldr	r8, [r0, #4]
	ldrh	r2, [r3, r2]
	mov	r4, r0
	ldrh	sl, [r3, r1]
	bic	r8, r8, #-2147483648
	ldr	r7, [r3, #856]
	ubfx	r9, r8, #10, #16
	ldrb	r3, [r3, #852]	@ zero_extendqisi2
	mov	r8, r8, asl #22
	mul	sl, sl, r2
	uxth	r7, r7
	cmp	r3, #1
	mov	r0, r9
	moveq	r7, r7, asl #1
	mov	r8, r8, lsr #22
	uxtheq	r7, r7
	uxth	sl, sl
	mov	r1, sl
	bl	__aeabi_uidiv
	cmp	r5, #1
	ldr	r3, .L96
	uxth	r0, r0
	mls	sl, sl, r0, r9
	uxth	sl, sl
	bne	.L93
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	addeq	r8, r3, r8, asl #1
	movweq	r2, #3076
	ldreqh	r8, [r8, r2]
.L93:
	add	r3, r3, r0, asl #2
	ldr	r3, [r3, #3588]
	mla	r7, r7, sl, r3
	ldrb	r3, [sp, #40]	@ zero_extendqisi2
	cmp	r3, #1
	add	r8, r7, r8
	str	r8, [r6, #0]
	str	r0, [fp, #0]
	bls	.L95
	ldr	r0, [r4, #4]
	ldr	r3, [r4, #40]
	add	r0, r0, #1024
	rsb	r3, r3, r0
	rsbs	r0, r3, #0
	adc	r0, r0, r3
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L95:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L97:
	.align	2
.L96:
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
	ldr	r3, .L99
	ldr	r2, [r3, #3620]
	str	r0, [r3, #3620]
	mov	r0, r2
	bx	lr
.L100:
	.align	2
.L99:
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
	ldr	r3, .L102
	ldr	r3, [r3, #3624]
	ldrb	r0, [r3, #9]	@ zero_extendqisi2
	bx	lr
.L103:
	.align	2
.L102:
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
	ldr	r3, .L106
	add	r3, r3, r0, asl #3
	ldrb	r0, [r3, #868]	@ zero_extendqisi2
	ldr	r2, [r3, #864]
	add	r0, r0, #8
	add	r0, r2, r0, asl #8
	bx	lr
.L107:
	.align	2
.L106:
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
	ldr	r3, .L109
	mov	r2, r0, asl #8
	orr	r2, r2, r0, asl #16
	orr	r2, r2, #1
	ldr	r3, [r3, #3008]
	str	r2, [r3, #304]
	bx	lr
.L110:
	.align	2
.L109:
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
	ldr	r3, .L112
	orr	r0, r0, #16640
	ldr	r3, [r3, #3008]
	str	r0, [r3, #344]
	bx	lr
.L113:
	.align	2
.L112:
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
	ldr	r3, .L116
	cmp	r0, #0
	ldr	r2, [r3, #3008]
	ldr	r3, [r2, #0]
	bfc	r3, #13, #1
	orrne	r3, r3, #253952
	str	r3, [r2, #0]
	bx	lr
.L117:
	.align	2
.L116:
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
	ldr	r3, .L122
	ands	r1, r0, #6
	ldr	r2, [r3, #3008]
	ldr	r3, [r2, #0]
	bfieq	r3, r1, #13, #1
	beq	.L121
	orr	r3, r3, #24576
	movw	r1, #16641
	bfc	r3, #15, #1
	str	r1, [r2, #344]
	orr	r3, r3, #196608
	ldr	r1, .L122+4
	tst	r0, #4
	orrne	r3, r3, #32768
	str	r1, [r2, #304]
	mov	r1, #38
	str	r1, [r2, #308]
	mov	r1, #39
	str	r1, [r2, #308]
.L121:
	str	r3, [r2, #0]
	mov	r0, #0
	bx	lr
.L123:
	.align	2
.L122:
	.word	.LANCHOR0
	.word	1710595
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
	ldr	r3, .L125
	add	r3, r3, r0, asl #3
	mov	r0, #1
	ldr	r2, [r3, #864]
	ldrb	r1, [r3, #868]	@ zero_extendqisi2
	ldr	r3, [r2, #0]
	mov	r1, r0, asl r1
	bfi	r3, r1, #0, #8
	str	r3, [r2, #0]
	bx	lr
.L126:
	.align	2
.L125:
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
	ldr	r3, .L128
	add	r3, r3, r0, asl #3
	ldr	r2, [r3, #864]
	ldr	r3, [r2, #0]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2, #0]
	bx	lr
.L129:
	.align	2
.L128:
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
	add	r0, r0, #996
	mov	r1, #1000
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	add	r0, r0, #3
	bl	__aeabi_uidiv
	ldr	r3, .L131
	ldr	r3, [r3, #8]
	blx	r3
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L132:
	.align	2
.L131:
	.word	arm_delay_ops
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
	ldr	r3, .L142
	add	r0, r3, r0, asl #3
	ldr	ip, [r3, #3624]
	ldrb	r5, [r0, #868]	@ zero_extendqisi2
	ldr	r4, [r0, #864]
	ldrb	ip, [ip, #8]	@ zero_extendqisi2
	add	r5, r5, #8
	cmp	ip, #2
	add	r0, r4, r5, asl #8
	movne	r3, #112
	strne	r3, [r0, #8]
	bne	.L138
	cmp	r2, #0
	ldreqb	r2, [r3, #909]	@ zero_extendqisi2
	ldrneb	r2, [r3, #910]	@ zero_extendqisi2
	str	r2, [r0, #8]
	ldrb	r3, [r3, #911]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #0
	ldrne	r2, .L142
	bne	.L137
	b	.L138
.L139:
	mov	ip, r3, asl #3
	add	r3, r3, #1
	mov	ip, r1, lsr ip
	uxtb	ip, ip
	str	ip, [r0, #4]
.L137:
	ldrb	ip, [r2, #911]	@ zero_extendqisi2
	cmp	r3, ip
	bcc	.L139
.L138:
	mov	r0, #80
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	uxtb	r0, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L143:
	.align	2
.L142:
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
.L148:
	mov	r0, r6
	mov	r1, r5
	mov	r2, r4
	bl	FlashReadStatusEN
	cmp	r0, #255
	beq	.L148
	tst	r0, #64
	beq	.L148
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
	ldr	r3, .L152
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r4, r3, r0, asl #4
	ldrb	r5, [r3, r0, asl #4]	@ zero_extendqisi2
	mov	r6, r0
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L150
	mov	r0, r5
	bl	NandcFlashCs
	ldr	r3, .L152+4
	ldr	r1, [r4, #4]
	mov	r0, r5
	add	r6, r3, r6, asl #2
	ldr	r2, [r6, #3588]
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
.L150:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L153:
	.align	2
.L152:
	.word	.LANCHOR0+3628
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
	mov	r7, r3
	ldr	r9, .L161
	mov	r4, r0
	mov	r5, r1
	mov	r6, r2
	ldr	r3, [r9, #3624]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	cmp	r3, #6
	addeq	r9, r9, #20
	addeq	r9, r9, r0, asl #6
	addeq	r9, r9, r7, asl #2
	beq	.L156
	cmp	r3, #7
	bne	.L157
	add	r9, r9, #28
	mov	r3, #160
	mla	r9, r3, r0, r9
	mov	r3, #10
	mla	r9, r3, r7, r9
	b	.L156
.L157:
	cmp	r3, #8
	addne	r3, r7, r0, asl #3
	addeq	r9, r9, #28
	addeq	r3, r7, r7, asl #2
	addne	r9, r9, #20
	addeq	r9, r9, r3
	addne	r9, r9, r3, asl #3
.L156:
	ldr	r3, .L161
	mov	r0, r4
	mov	r8, #0
	add	r3, r3, r4, asl #3
	ldrb	sl, [r3, #868]	@ zero_extendqisi2
	ldr	r2, [r3, #864]
	add	sl, sl, #8
	add	sl, r2, sl, asl #8
	bl	NandcFlashCs
	mov	r3, #54
	str	r3, [sl, #8]
	b	.L159
.L160:
	ldrb	r3, [r6, r8]	@ zero_extendqisi2
	mov	r0, #200
	str	r3, [sl, #4]
	bl	NandcDelayns
	ldrsb	r3, [r9, r8]
	add	r8, r8, #1
	str	r3, [sl, #0]
.L159:
	uxtb	r3, r8
	cmp	r3, r5
	bcc	.L160
	mov	r3, #22
	mov	r0, r4
	str	r3, [sl, #8]
	bl	NandcFlashDeCs
	ldr	r3, .L161
	add	r4, r3, r4
	strb	r7, [r4, #3756]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L162:
	.align	2
.L161:
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
	ldr	r5, .L168
	ldr	r3, [r5, #3624]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #7
	ldmhifd	sp!, {r3, r4, r5, pc}
	mov	r4, #0
.L166:
	ldr	r3, .L168+4
	uxtb	r0, r4
	ldrb	r3, [r3, r4, asl #3]	@ zero_extendqisi2
	cmp	r3, #173
	bne	.L165
	ldrb	r1, [r5, #1]	@ zero_extendqisi2
	mov	r3, #0
	ldr	r2, .L168+8
	bl	HynixSetRRPara
.L165:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L166
	ldmfd	sp!, {r3, r4, r5, pc}
.L169:
	.align	2
.L168:
	.word	.LANCHOR0
	.word	.LANCHOR0+2976
	.word	.LANCHOR0+4
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
	ldr	r3, .L173
	mov	r5, #0
	ldr	sl, .L173+4
	mov	r8, #161
	add	r1, r3, r1, asl #2
	add	r6, r1, #3
	mov	r7, r3
	b	.L171
.L172:
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
.L171:
	ldrb	r3, [sl, #3760]	@ zero_extendqisi2
	cmp	r5, r3
	bcc	.L172
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L174:
	.align	2
.L173:
	.word	.LANCHOR1+3148
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	add	r6, r1, #1
	ldr	r8, .L182
	mov	r4, r0
	add	r6, r6, r6, asl #2
	mov	r5, #0
	add	r3, r8, #45
	add	r7, r8, r6
	ldr	sl, .L182+4
	add	r6, r3, r6
	add	r9, r8, r1
	b	.L176
.L180:
	mov	r3, #85
	str	r3, [r4, #8]
	ldrsb	r3, [r5, r8]
	mov	r0, #200
	str	r3, [r4, #4]
	bl	NandcDelayns
	ldrb	r3, [sl, #3761]	@ zero_extendqisi2
	cmp	r3, #34
	ldreqsb	r3, [r7, #0]
	beq	.L181
	cmp	r3, #35
	addne	r3, r9, #3200
	addne	r3, r3, #12
	ldreqsb	r3, [r6, #0]
	ldrnesb	r3, [r3, #0]
.L181:
	add	r5, r5, #1
	add	r7, r7, #1
	add	r6, r6, #1
	str	r3, [r4, #0]
.L176:
	ldrb	r3, [sl, #3760]	@ zero_extendqisi2
	cmp	r5, r3
	bcc	.L180
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L183:
	.align	2
.L182:
	.word	.LANCHOR1
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
	ldr	r3, .L185
	add	r3, r3, r0, asl #3
	mov	r0, #80
	ldrb	r5, [r3, #868]	@ zero_extendqisi2
	ldr	r4, [r3, #864]
	add	r5, r5, #8
	add	r3, r4, r5, asl #8
	str	r2, [r3, #8]
	bl	NandcDelayns
	ldr	r0, [r4, r5, asl #8]
	ldmfd	sp!, {r3, r4, r5, pc}
.L186:
	.align	2
.L185:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadStatus, .-FlashReadStatus
	.align	2
	.global	NandcWaitFlashReadyNoDelay
	.type	NandcWaitFlashReadyNoDelay, %function
NandcWaitFlashReadyNoDelay:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L192
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r0, r1, r2, r4, r5, lr}
	add	r3, r3, r0, asl #3
	ldr	r4, .L192+4
	ldr	r5, [r3, #864]
.L189:
	ldr	r3, [r5, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L190
	mov	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L189
	mvn	r0, #0
	b	.L188
.L190:
	mov	r0, #0
.L188:
	ldmfd	sp!, {r1, r2, r3, r4, r5, pc}
.L193:
	.align	2
.L192:
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
	ldr	r3, .L199
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r0, r1, r2, r4, r5, lr}
	add	r3, r3, r0, asl #3
	mov	r0, #150
	ldr	r4, .L199+4
	ldr	r5, [r3, #864]
	bl	NandcDelayns
.L196:
	ldr	r3, [r5, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L197
	mov	r0, #10
	bl	NandcDelayns
	subs	r4, r4, #1
	bne	.L196
	mvn	r0, #0
	b	.L195
.L197:
	mov	r0, #0
.L195:
	ldmfd	sp!, {r1, r2, r3, r4, r5, pc}
.L200:
	.align	2
.L199:
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
	ldr	r5, .L209
	b	.L202
.L208:
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
	bcs	.L204
	add	r2, r5, r3
	add	r3, r5, r3, asl #4
	ldrb	sl, [r2, #3764]	@ zero_extendqisi2
	strb	sl, [r3, #3628]
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
	ldr	r1, [r5, #856]
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
	bne	.L207
	ldr	r0, .L209+4
	ldr	r1, [sp, #8]
	bl	printk
.L207:
	mov	r0, sl
	bl	NandcFlashDeCs
.L204:
	sub	r6, r6, #1
	add	r7, r7, #1
	add	r4, r4, #36
	uxtb	r6, r6
.L202:
	cmp	r7, r8
	bne	.L208
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L210:
	.align	2
.L209:
	.word	.LANCHOR0
	.word	.LC1
	.fnend
	.size	FlashEraseSLc2KBlocks, .-FlashEraseSLc2KBlocks
	.align	2
	.global	micron_auto_read_calibration_config
	.type	micron_auto_read_calibration_config, %function
micron_auto_read_calibration_config:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r1
	mov	r4, r0
	bl	NandcWaitFlashReady
	ldr	r3, .L212
	mov	r2, #239
	mov	r0, #200
	add	r4, r3, r4, asl #3
	ldr	r5, [r4, #864]
	ldrb	r4, [r4, #868]	@ zero_extendqisi2
	add	r4, r4, #8
	add	r3, r5, r4, asl #8
	str	r2, [r3, #8]
	mov	r2, #150
	str	r2, [r3, #4]
	bl	NandcDelayns
	str	r6, [r5, r4, asl #8]
	mov	r3, #0
	str	r3, [r5, r4, asl #8]
	str	r3, [r5, r4, asl #8]
	str	r3, [r5, r4, asl #8]
	ldmfd	sp!, {r4, r5, r6, pc}
.L213:
	.align	2
.L212:
	.word	.LANCHOR0
	.fnend
	.size	micron_auto_read_calibration_config, .-micron_auto_read_calibration_config
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
	ldr	r1, .L220
	add	r3, r5, r5, asl #2
	mvn	r6, #4
	add	r0, r1, r3
	add	r1, r1, #45
	add	r1, r1, r3
	mov	r2, #0
	ldr	ip, .L220+4
	mul	r5, r6, r5
	b	.L215
.L218:
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
.L215:
	ldrb	r6, [ip, #3760]	@ zero_extendqisi2
	cmp	r2, r6
	bcc	.L218
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, lr}
	b	NandcWaitFlashReady
.L221:
	.align	2
.L220:
	.word	.LANCHOR1
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
	ldr	r3, .L223
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r3, r3, r0, asl #3
	ldrb	r6, [r3, #868]	@ zero_extendqisi2
	ldr	r5, [r3, #864]
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
.L224:
	.align	2
.L223:
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
	ldr	r3, .L226
	mov	ip, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r2
	ldrb	r2, [r3, #3125]	@ zero_extendqisi2
	ldr	r3, .L226+4
	add	r3, r3, r0, asl #3
	mov	r2, r2, asl #9
	ldrb	r5, [r3, #868]	@ zero_extendqisi2
	ldr	r4, [r3, #864]
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
.L227:
	.align	2
.L226:
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
	ldr	sl, .L242
	mov	r7, #239
	mov	r1, r4
	mov	r6, #128
	sub	r8, sl, #864
	mov	r5, #1
	mov	r9, #35
.L238:
	mov	r2, sl
	ldr	lr, .L242+4
	ldr	r3, [r2, r4]!
	ldrb	lr, [r4, lr]	@ zero_extendqisi2
	ldrb	r2, [r2, #4]	@ zero_extendqisi2
	cmp	lr, #152
	cmpne	lr, #69
	add	r2, r2, #8
	add	ip, r3, r2, asl #8
	beq	.L229
	cmp	lr, #173
	cmpne	lr, #44
	bne	.L230
.L229:
	cmp	r0, #1
	ldrb	fp, [r8, #3772]	@ zero_extendqisi2
	bne	.L231
	tst	fp, #1
	beq	.L230
	cmp	lr, #173
	str	r7, [ip, #8]
	streq	r0, [ip, #4]
	beq	.L240
	cmp	lr, #44
	streq	r0, [ip, #4]
	strne	r6, [ip, #4]
	moveq	ip, #5
	strne	r0, [r3, r2, asl #8]
	bne	.L236
	b	.L241
.L231:
	tst	fp, #4
	beq	.L230
	cmp	lr, #173
	str	r7, [ip, #8]
	bne	.L235
	str	r5, [ip, #4]
	mov	ip, #32
.L241:
	str	ip, [r3, r2, asl #8]
	b	.L236
.L235:
	cmp	lr, #44
	streq	r5, [ip, #4]
	streq	r9, [r3, r2, asl #8]
	beq	.L236
	str	r6, [ip, #4]
.L240:
	str	r1, [r3, r2, asl #8]
.L236:
	str	r1, [r3, r2, asl #8]
	str	r1, [r3, r2, asl #8]
	str	r1, [r3, r2, asl #8]
.L230:
	add	r4, r4, #8
	cmp	r4, #32
	bne	.L238
	mov	r0, #0
	bl	NandcWaitFlashReady
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L243:
	.align	2
.L242:
	.word	.LANCHOR0+864
	.word	.LANCHOR0+2976
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
	.global	flash_exit_slc_mode
	.type	flash_exit_slc_mode, %function
flash_exit_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r6, r0
	ldr	r4, .L247
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	bl	NandcFlashCs
	add	r3, r4, r6, asl #3
	mov	r0, #100
	ldrb	r5, [r3, #868]	@ zero_extendqisi2
	ldr	r4, [r3, #864]
	mov	r3, #239
	add	r5, r5, #8
	add	r7, r4, r5, asl #8
	str	r3, [r7, #8]
	mov	r3, #145
	str	r3, [r7, #4]
	bl	NandcDelayns
	mov	r3, #2
	mov	r0, #100
	str	r3, [r4, r5, asl #8]
	mov	r3, #1
	str	r3, [r4, r5, asl #8]
	mov	r3, #0
	str	r3, [r4, r5, asl #8]
	str	r3, [r4, r5, asl #8]
	bl	NandcDelayns
	mov	r0, r6
	bl	NandcWaitFlashReadyNoDelay
	mov	r3, #223
	mov	r0, r6
	str	r3, [r7, #8]
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	NandcWaitFlashReady
.L248:
	.align	2
.L247:
	.word	.LANCHOR0
	.fnend
	.size	flash_exit_slc_mode, .-flash_exit_slc_mode
	.align	2
	.global	flash_enter_slc_mode
	.type	flash_enter_slc_mode, %function
flash_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r6, r0
	ldr	r4, .L251
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	bl	NandcFlashCs
	add	r3, r4, r6, asl #3
	mov	r0, #100
	ldrb	r5, [r3, #868]	@ zero_extendqisi2
	ldr	r4, [r3, #864]
	mov	r3, #239
	add	r5, r5, #8
	add	r7, r4, r5, asl #8
	str	r3, [r7, #8]
	mov	r3, #145
	str	r3, [r7, #4]
	bl	NandcDelayns
	mov	r3, #0
	mov	r2, #1
	str	r3, [r4, r5, asl #8]
	mov	r0, #100
	str	r2, [r4, r5, asl #8]
	str	r3, [r4, r5, asl #8]
	str	r3, [r4, r5, asl #8]
	bl	NandcDelayns
	mov	r0, r6
	bl	NandcWaitFlashReadyNoDelay
	mov	r3, #218
	mov	r0, r6
	str	r3, [r7, #8]
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	NandcWaitFlashReady
.L252:
	.align	2
.L251:
	.word	.LANCHOR0
	.fnend
	.size	flash_enter_slc_mode, .-flash_enter_slc_mode
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
	ldr	r3, .L254
	add	r3, r3, r0, asl #3
	ldrb	r5, [r3, #868]	@ zero_extendqisi2
	ldr	r2, [r3, #864]
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
.L255:
	.align	2
.L254:
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
	mov	r4, r0
	ldr	r3, .L258
	mov	r5, r1
	add	r3, r3, r0, asl #3
	ldrb	r7, [r3, #868]	@ zero_extendqisi2
	ldr	r6, [r3, #864]
	bl	FlashReset
	add	r7, r7, #8
	mov	r0, r4
	bl	NandcFlashCs
	mov	r3, #144
	add	r8, r6, r7, asl #8
	mov	r0, #200
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
	bhi	.L256
	ldrb	r1, [r5, #2]	@ zero_extendqisi2
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	ldr	r0, .L258+4
	str	r1, [sp, #0]
	ldrb	r1, [r5, #3]	@ zero_extendqisi2
	str	r1, [sp, #4]
	ldrb	r1, [r5, #4]	@ zero_extendqisi2
	str	r1, [sp, #8]
	ldrb	r1, [r5, #5]	@ zero_extendqisi2
	str	r1, [sp, #12]
	add	r1, r4, #1
	bl	printk
.L256:
	ldmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, pc}
.L259:
	.align	2
.L258:
	.word	.LANCHOR0
	.word	.LC2
	.fnend
	.size	FlashReadIDRaw, .-FlashReadIDRaw
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
	ldr	r4, .L264
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	ldr	r0, [r4, #3776]
	cmp	r0, #0
	beq	.L261
	mov	r0, #0
	bl	flash_enter_slc_mode
	b	.L262
.L261:
	bl	flash_exit_slc_mode
.L262:
	ldrb	r3, [r4, #3780]	@ zero_extendqisi2
	ldr	r5, .L264
	cmp	r3, #0
	beq	.L263
	ldrb	r3, [r5, #3772]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L263
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r3, #0
	strb	r3, [r5, #3780]
.L263:
	ldr	r3, [r4, #864]
	mov	r0, #0
	str	r0, [r3, #336]
	ldmfd	sp!, {r3, r4, r5, pc}
.L265:
	.align	2
.L264:
	.word	.LANCHOR0
	.fnend
	.size	ftl_flash_de_init, .-ftl_flash_de_init
	.align	2
	.global	FlashReadDpCmd
	.type	FlashReadDpCmd, %function
FlashReadDpCmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L270
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r2
	add	r2, r3, r0, asl #3
	mov	r5, r1
	mov	r4, r0
	ldrb	r6, [r2, #868]	@ zero_extendqisi2
	ldr	r1, [r2, #864]
	ldrb	r2, [r3, #912]	@ zero_extendqisi2
	add	r6, r6, #8
	cmp	r2, #1
	ldrb	r2, [r3, #904]	@ zero_extendqisi2
	add	r6, r1, r6, asl #8
	str	r2, [r6, #8]
	bne	.L267
	mov	r8, #0
	uxtb	r2, r5
	str	r8, [r6, #4]
	str	r8, [r6, #4]
	str	r2, [r6, #4]
	mov	r2, r5, lsr #8
	str	r2, [r6, #4]
	mov	r2, r5, lsr #16
	str	r2, [r6, #4]
	ldrb	r3, [r3, #905]	@ zero_extendqisi2
	str	r3, [r6, #8]
	bl	NandcWaitFlashReady
	str	r8, [r6, #8]
	str	r8, [r6, #4]
	str	r8, [r6, #4]
	b	.L269
.L267:
	uxtb	r2, r5
	str	r2, [r6, #4]
	mov	r2, r5, lsr #8
	str	r2, [r6, #4]
	mov	r2, r5, lsr #16
	str	r2, [r6, #4]
	ldrb	r3, [r3, #905]	@ zero_extendqisi2
	str	r3, [r6, #8]
.L269:
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
.L271:
	.align	2
.L270:
	.word	.LANCHOR0
	.fnend
	.size	FlashReadDpCmd, .-FlashReadDpCmd
	.align	2
	.global	NandcRandmzSel
	.type	NandcRandmzSel, %function
NandcRandmzSel:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L273
	add	r3, r3, r0, asl #3
	ldr	r3, [r3, #864]
	str	r1, [r3, #336]
	bx	lr
.L274:
	.align	2
.L273:
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
	ldr	r1, .L285
	bl	__aeabi_idiv
	ldr	r3, .L285+4
	ldr	r3, [r3, #3008]
	cmp	r0, #250
	movwgt	r2, #8354
	bgt	.L283
	cmp	r0, #220
	bgt	.L284
.L278:
	cmp	r0, #185
	movwgt	r2, #4226
	bgt	.L283
	cmp	r0, #160
	movwgt	r2, #4194
	bgt	.L283
	cmp	r4, #35
	movwls	r2, #4193
	bls	.L283
	cmp	r4, #99
	movwls	r2, #4225
	bls	.L283
.L284:
	movw	r2, #8322
.L283:
	str	r2, [r3, #4]
	ldmfd	sp!, {r4, pc}
.L286:
	.align	2
.L285:
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
	mov	r2, #0
	ldr	r4, .L289
	mov	r3, #1
	mov	r5, #0
	str	r0, [r4, #3008]
	str	r2, [r4, #868]
	str	r0, [r4, #864]
	str	r3, [r4, #876]
	mov	r3, #2
	str	r0, [r4, #872]
	str	r3, [r4, #884]
	mov	r3, #3
	str	r0, [r4, #880]
	str	r0, [r4, #888]
	str	r3, [r4, #892]
	ldr	r3, [r0, #0]
	and	r3, r3, #253952
	ubfx	r1, r3, #13, #1
	bfi	r3, r2, #13, #1
	ldr	r2, [r0, #352]
	orr	r3, r3, #256
	str	r1, [r4, #3784]
	movw	r1, #2049
	ubfx	r2, r2, #16, #4
	str	r2, [r4, #3788]
	ldr	r2, [r0, #352]
	cmp	r2, r1
	str	r2, [r4, #3792]
	moveq	r2, #8
	streq	r2, [r4, #3788]
	str	r3, [r0, #0]
	mov	r0, #40
	ldr	r3, [r4, #3008]
	str	r5, [r3, #336]
	bl	NandcTimeCfg
	ldr	r3, [r4, #3008]
	movw	r2, #8322
	mov	r0, #36864
	str	r2, [r3, #344]
	ldr	r2, .L289+4
	str	r2, [r3, #304]
	bl	ftl_malloc
	str	r5, [r4, #3824]
	str	r5, [r4, #3832]
	str	r0, [r4, #3796]
	str	r0, [r4, #3800]
	add	r0, r0, #32768
	str	r0, [r4, #3804]
	ldmfd	sp!, {r3, r4, r5, pc}
.L290:
	.align	2
.L289:
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
	bls	.L292
	movw	r3, #4226
	movw	r2, #8322
	cmp	r0, r3
	cmpne	r0, r2
	bne	.L293
.L292:
	ldr	r3, .L294
	ldr	r3, [r3, #3008]
	str	r0, [r3, #4]
.L293:
	ldr	r3, .L294+4
	ldrb	r0, [r3, #3137]	@ zero_extendqisi2
	b	NandcTimeCfg
.L295:
	.align	2
.L294:
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
	ldr	ip, .L297
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, [ip, #3008]
	ldr	r4, [r4, #4]
	str	r4, [r0, #0]
	ldr	r0, [ip, #3008]
	ldr	r0, [r0, #0]
	str	r0, [r1, #0]
	ldr	r1, [ip, #3008]
	ldr	r1, [r1, #304]
	str	r1, [r2, #0]
	ldr	r2, [ip, #3008]
	ldr	r0, [r2, #308]
	ldr	r1, [r2, #344]
	uxtb	r2, r0
	orr	r2, r2, r1, asl #16
	str	r2, [r3, #0]
	ldmfd	sp!, {r4, pc}
.L298:
	.align	2
.L297:
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
	ldr	r3, .L304
	mov	r1, #1
	ldr	r2, [r3, #3008]
	str	r0, [r3, #3836]
	mov	r3, #0
	str	r1, [r2, #8]
	mov	r1, #16
	cmp	r0, r1
	bfi	r3, r1, #8, #8
	bfc	r3, #18, #1
	beq	.L303
.L300:
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L301
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	bne	.L301
.L303:
	bfc	r3, #4, #1
.L301:
	orr	r3, r3, #1
	str	r3, [r2, #12]
	bx	lr
.L305:
	.align	2
.L304:
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
	ldr	r3, .L307
	strb	r0, [r3, #3840]
	b	NandcBchSel
.L308:
	.align	2
.L307:
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
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, #0
	ldr	r5, .L314
	ldr	r3, [r5, #3008]
	ldr	r2, [r5, #3012]
	str	r2, [r3, #0]
	ldr	r2, [r5, #3016]
	ldr	r3, [r5, #3008]
	str	r2, [r3, #4]
	ldr	r2, [r5, #3020]
	str	r2, [r3, #8]
	ldr	r2, [r5, #3024]
	str	r2, [r3, #12]
	ldr	r2, [r5, #3028]
	str	r2, [r3, #304]
	ldr	r2, [r5, #3032]
	str	r2, [r3, #308]
	ldr	r2, [r5, #3036]
	str	r2, [r3, #336]
	ldr	r2, [r5, #3040]
	add	r5, r5, #2976
	str	r2, [r3, #344]
.L311:
	ldrb	r3, [r5, r4, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L310
	uxtb	r0, r4
	bl	FlashReset
.L310:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L311
	ldr	r4, .L314
	ldrb	r3, [r4, #3780]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L312
	mov	r0, #1
	bl	NandcSetMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
	ldrb	r0, [r4, #3029]	@ zero_extendqisi2
	bl	NandcSetDdrPara
.L312:
	ldr	r3, .L314
	ldr	r3, [r3, #3624]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	ldmfd	sp!, {r3, r4, r5, lr}
	b	FlashBchSel
.L315:
	.align	2
.L314:
	.word	.LANCHOR0
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
	ldr	r3, [r4, #0]
	ubfx	r3, r3, #8, #8
	and	r3, r3, #2
	uxtb	r3, r3
	cmp	r3, #0
	bne	.L319
	mov	r0, r4
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L319:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r5, .L340
	mov	r6, r0
	str	r1, [sp, #12]
	mov	r7, r2
	ldrb	r4, [r5, #852]	@ zero_extendqisi2
	cmp	r4, #0
	moveq	r9, r5
	beq	.L321
	mov	r1, r2
	bl	FlashEraseSLc2KBlocks
	b	.L322
.L330:
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
	bcs	.L324
	ldrb	r3, [r5, #3841]	@ zero_extendqisi2
	cmp	r3, #0
	add	r3, r9, r0, asl #4
	ldr	r3, [r3, #3636]
	moveq	sl, #0
	cmp	r3, #0
	beq	.L326
	uxtb	r0, r0
	bl	FlashWaitCmdDone
.L326:
	ldr	r2, [sp, #20]
	mov	r1, #0
	cmp	sl, #0
	add	r3, r5, r2, asl #4
	movne	r0, #36
	str	r1, [r3, #3640]
	ldr	r1, [sp, #16]
	str	fp, [r3, #3636]
	str	r1, [r3, #3632]
	addne	r1, r4, #1
	mlane	r1, r0, r1, r6
	strne	r1, [r3, #3640]
	add	r3, r9, r2
	add	r2, r9, r2, asl #4
	ldrb	r8, [r3, #3764]	@ zero_extendqisi2
	mov	r0, r8
	strb	r8, [r2, #3628]
	bl	NandcFlashCs
	ldr	r3, [sp, #12]
	mov	r0, r8
	cmp	r3, #1
	bne	.L328
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L328
	bl	flash_enter_slc_mode
	b	.L329
.L328:
	bl	flash_exit_slc_mode
.L329:
	ldr	r3, [sp, #20]
	mov	r0, r8
	ldr	r1, [sp, #16]
	add	r4, r4, sl
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #3588]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r8
	ldr	r1, [sp, #16]
	mov	r2, sl
	bl	FlashEraseCmd
	mov	r0, r8
	bl	NandcFlashDeCs
.L324:
	add	r4, r4, #1
.L321:
	cmp	r4, r7
	bcc	.L330
	ldr	r5, .L340
	mov	r4, #0
	ldr	r8, .L340+4
	ldr	r0, [r5, #3008]
	bl	NandcIqrWaitFlashReady
	b	.L331
.L333:
	uxtb	r0, r4
	bl	FlashWaitCmdDone
	ldr	r3, [sp, #12]
	cmp	r3, #1
	bne	.L332
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L332
	ldrb	r0, [r8, r4, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L332:
	add	r4, r4, #1
.L331:
	ldrb	r2, [r5, #3762]	@ zero_extendqisi2
	ldr	r3, .L340
	cmp	r4, r2
	bcc	.L333
	ldr	r0, [r3, #3844]
	cmp	r0, #0
	beq	.L322
	ldrb	r3, [r3, #2976]	@ zero_extendqisi2
	cmp	r3, #69
	moveq	r3, #0
	moveq	r2, r3
	bne	.L339
	b	.L334
.L335:
	add	r3, r3, #1
	str	r2, [r6, #-36]
.L334:
	cmp	r3, r7
	add	r6, r6, #36
	bne	.L335
.L339:
	mov	r0, #0
.L322:
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L341:
	.align	2
.L340:
	.word	.LANCHOR0
	.word	.LANCHOR0+3628
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
	ldr	r3, .L344
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
.L345:
	.align	2
.L344:
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
.L347:
	ldr	r3, [r0, #8]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #1048576
	beq	.L347
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
	mov	r6, r1
	ldr	r1, .L362
	mov	r8, r2
	mov	r4, #0
	.pad #20
	sub	sp, sp, #20
	add	r0, r1, r0, asl #3
	ldr	r9, [sp, #56]
	ldr	r5, [r0, #864]
	ldrb	r2, [r0, #868]	@ zero_extendqisi2
	mov	r0, #16
	ldr	ip, [sp, #60]
	ldr	r7, [r5, #12]
	bfi	r7, r0, #8, #8
	bfi	r7, r4, #3, #1
	bfi	r4, r6, #1, #1
	orr	r4, r4, #8
	bfi	r7, r2, #5, #3
	mov	r2, #1
	bfi	r4, r2, #5, #2
	orr	r4, r4, #536870912
	mov	r3, r3, lsr r2
	orr	r4, r4, #1024
	bfi	r4, r3, #4, #1
	ldr	r3, [r1, #3788]
	cmp	r3, #3
	bls	.L350
	ldr	r3, [r5, #16]
	cmp	r9, #0
	cmpeq	ip, #0
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bfc	r3, #2, #1
	str	r3, [sp, #12]
	beq	.L351
	cmp	r6, #0
	beq	.L352
	ldr	r3, [r1, #3836]
	mov	r0, ip
	cmp	r3, #24
	movhi	r3, #128
	movls	r3, #64
	str	r3, [sp, #0]
	mov	r3, r8, lsr r2
	str	r3, [sp, #4]
	mov	r3, #0
	mov	r2, r3
	b	.L354
.L357:
	cmp	ip, #0
	mov	lr, r3, lsr #2
	add	r2, r2, #1
	ldrneh	fp, [r0, #2]
	mvneq	fp, #0
	ldrneh	sl, [r0], #4
	ldreq	sl, [r1, #3804]
	orrne	sl, sl, fp, asl #16
	ldrne	fp, [r1, #3804]
	streq	fp, [sl, lr, asl #2]
	strne	sl, [fp, lr, asl #2]
	ldr	lr, [sp, #0]
	add	r3, r3, lr
.L354:
	ldr	sl, [sp, #4]
	cmp	r2, sl
	bcc	.L357
.L352:
	mov	r0, r5
	add	r8, r8, #1
	bl	rk_nandc_xfer_irq_flag_init
	ldr	r3, [r5, #368]
	mov	r8, r8, asr #1
	cmp	r9, #0
	bfi	r4, r8, #22, #6
	ldr	r8, .L362
	orr	r3, r3, #1
	str	r3, [r5, #368]
	ldr	r3, [r5, #364]
	ubfx	sl, r4, #22, #5
	ldreq	r0, [r8, #3800]
	movne	r0, r9
	orr	r3, r3, #1
	str	r3, [r5, #364]
	ldr	r3, [r8, #3804]
	mov	r2, r6
	mov	r1, sl, asl #10
	str	r3, [r8, #3812]
	str	r0, [r8, #3808]
	bl	rknand_dma_map_single
	mov	r2, r6
	mov	r1, sl, asl #7
	str	r0, [r8, #3816]
	ldr	r0, [r8, #3812]
	bl	rknand_dma_map_single
	mov	r3, #1
	str	r3, [r8, #3824]
	mov	r2, #16
	ldr	r3, [r8, #3816]
	tst	r9, #3
	str	r0, [r8, #3820]
	str	r3, [r5, #20]
	ldr	r3, [r8, #3820]
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
.L351:
	ldr	r3, [sp, #12]
	str	r3, [r5, #16]
.L350:
	str	r7, [r5, #12]
	str	r4, [r5, #8]
	orr	r4, r4, #4
	str	r4, [r5, #8]
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L363:
	.align	2
.L362:
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
	bne	.L365
	cmp	r3, #0
	beq	.L366
	mov	r0, r2
	mov	r1, r3
	mov	r2, #1024
	bl	memcpy
.L366:
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
.L365:
	cmp	r3, #0
	beq	.L368
	mov	r1, r2
	mov	r0, r3
	mov	r2, #1024
	bl	memcpy
.L368:
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
	b	.L370
.L371:
	add	r3, r3, #1
	mov	r2, r2, asl #1
	uxth	r3, r3
.L370:
	cmp	r2, r0
	bls	.L371
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
	ldr	r3, .L376
	movw	r2, #3852
	cmp	r0, #23
	movw	r1, #3862
	ldrh	r2, [r3, r2]
	movls	r0, #24
	ldrh	r1, [r3, r1]
	str	r0, [r3, #3848]
	mul	r2, r2, r0
	rsb	r0, r0, r1
	movw	r1, #3860
	strh	r0, [r3, r1]	@ movhi
	mov	r0, #0
	ldr	r1, [r3, #3868]
	str	r2, [r3, #3856]
	rsb	r2, r2, r1
	str	r2, [r3, #3864]
	bx	lr
.L377:
	.align	2
.L376:
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
	mov	r3, #3872
	ldr	r2, .L395
	mov	r4, r0
	ldrh	lr, [r0, #8]
	movw	r1, #3876
	ldrh	r0, [r0, #10]
	.pad #20
	sub	sp, sp, #20
	ldrh	r6, [r4, #14]
	strh	lr, [r2, r3]	@ movhi
	add	r3, r3, #2
	strh	r0, [r2, r3]	@ movhi
	ldrh	r3, [r4, #12]
	strh	r3, [r2, r1]	@ movhi
	movw	r1, #3862
	strh	r6, [r2, r1]	@ movhi
	mov	r2, #0
	ldr	r1, .L395+4
.L379:
	strb	r2, [r2, r1]
	add	r2, r2, #1
	cmp	r2, #32
	bne	.L379
	ldrh	r1, [r4, #14]
	ldrh	r2, [r4, #20]
	cmp	r2, r1, lsr #8
	bcs	.L380
	ldr	r1, .L395+4
	sub	ip, r0, #1
	uxtb	r7, r3
	mov	r2, #0
	rsb	sl, r3, r1
	str	sl, [sp, #8]
	mla	ip, r3, ip, r1
	mov	r8, r7, asl #1
	uxtb	r8, r8
	str	ip, [sp, #12]
	b	.L381
.L382:
	strb	ip, [r9, r1]
	add	fp, ip, r7
	add	ip, ip, r8
	add	r5, r5, #1
	strb	fp, [sl, r1]
	uxtb	ip, ip
.L384:
	cmp	r5, r0
	add	r1, r1, r3
	bcc	.L382
	add	r2, r2, #1
.L381:
	cmp	r2, r3
	bcs	.L383
	ldr	fp, [sp, #8]
	mov	r1, #0
	uxtb	ip, r2
	mov	r5, r1
	add	r9, fp, r2
	ldr	fp, [sp, #12]
	add	sl, fp, r2
	b	.L384
.L383:
	ldr	r2, .L395
	movw	r1, #3874
	mov	r0, r0, asl #1
	mov	r6, r6, lsr #1
	strh	r0, [r2, r1]	@ movhi
	movw	r1, #3862
	strh	r6, [r2, r1]	@ movhi
.L380:
	ldr	r2, .L395
	movw	r1, #3910
	cmp	lr, #1
	mov	r0, #5
	mov	ip, #0
	ldr	sl, .L395
	ldrb	r8, [r2, #852]	@ zero_extendqisi2
	strh	r0, [r2, r1]	@ movhi
	movw	r0, #3912
	streqh	lr, [r2, r1]	@ movhi
	cmp	r8, #0
	movw	r1, #3914
	strh	ip, [r2, r0]	@ movhi
	mov	r0, #4352
	strh	r0, [r2, r1]	@ movhi
	ldrne	r2, .L395
	movne	r0, #384
	ldrh	fp, [r4, #18]
	ldrh	r9, [r4, #20]
	strneh	r0, [r2, r1]	@ movhi
	movw	r2, #3874
	ldrh	r5, [sl, r2]
	movw	r2, #3852
	mov	r0, r9
	mul	r5, r5, r3
	uxth	r5, r5
	strh	r5, [sl, r2]	@ movhi
	add	r2, r2, #10
	ldrh	r6, [sl, r2]
	add	r2, r2, #54
	mul	r3, r3, r6
	strh	r3, [sl, r2]	@ movhi
	add	r2, r2, #2
	ldrh	r3, [r4, #16]
	mul	r1, r3, r5
	strh	r3, [sl, r2]	@ movhi
	mov	r2, #3920
	strh	fp, [sl, r2]	@ movhi
	add	r2, r2, #2
	strh	r1, [sl, r2]	@ movhi
	add	r2, r2, #2
	strh	r9, [sl, r2]	@ movhi
	str	r3, [sp, #4]
	bl	Ftl_log2
	movw	r2, #3926
	movw	r1, #3928
	cmp	r6, #1024
	strh	r0, [sl, r2]	@ movhi
	mov	r2, r9, asl #9
	mov	r7, r0
	uxth	r2, r2
	strh	r2, [sl, r1]	@ movhi
	add	r1, r1, #2
	mov	r2, r2, lsr #8
	strh	r2, [sl, r1]	@ movhi
	movw	r2, #3932
	ldrh	r1, [r4, #26]
	mov	r4, sl
	strh	r1, [sl, r2]	@ movhi
	uxtbhi	r1, r6
	mul	r2, r6, r5
	str	r2, [sl, #3868]
	movw	r2, #3912
	ldr	r3, [sp, #4]
	strhih	r1, [sl, r2]	@ movhi
	ldrh	r2, [sl, r2]
	mul	r1, fp, r9
	rsb	r2, r2, r6
	mov	r6, r6, asl #6
	mul	r2, r2, r5
	mul	r2, r9, r2
	mul	r3, r3, r2
	mov	r3, r3, asr #11
	str	r3, [sl, #3936]
	movw	sl, #3914
	ldrh	r0, [r4, sl]
	mov	r0, r0, asl #3
	bl	__aeabi_idiv
	movw	r3, #3940
	uxth	r0, r0
	strh	r0, [r4, r3]	@ movhi
	cmp	r0, #4
	ldr	r4, .L395
	ldrls	r2, .L395
	movls	r1, #4
	strlsh	r1, [r2, r3]	@ movhi
	cmp	r8, #0
	mov	r1, r5
	movne	r3, #640
	strneh	r3, [r4, sl]	@ movhi
	ldrh	r2, [r4, sl]
	movw	r3, #3942
	mov	r2, r2, asr r7
	add	r7, r7, #9
	mov	r6, r6, asr r7
	add	r2, r2, #2
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	uxth	r6, r6
	strh	r6, [r4, r3]	@ movhi
	mul	r3, r5, r6
	add	r6, r6, #8
	str	r3, [r4, #3948]
	movw	r3, #3940
	ldrh	r0, [r4, r3]
	bl	__aeabi_uidiv
	cmp	r5, #1
	ldreq	r3, .L395
	uxtah	r6, r6, r0
	str	r6, [r4, #3848]
	ldr	r4, .L395
	addeq	r6, r6, #4
	streq	r6, [r3, #3848]
	ldr	r0, [r4, #3848]
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #3848]
	ldr	r6, [r4, #3864]
	mov	r0, #2048
	str	r3, [r4, #3952]
	movw	r3, #3918
	ldrh	r3, [r4, r3]
	mov	r6, r6, asl #2
	mul	r6, r3, r6
	movw	r3, #3926
	ldrh	r3, [r4, r3]
	add	r3, r3, #9
	mov	r6, r6, lsr r3
	movw	r3, #3956
	add	r6, r6, #2
	uxth	r6, r6
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #3924
	ldrh	r5, [r4, r3]
	mov	r1, r5
	bl	__aeabi_idiv
	movw	r3, #3958
	ldrb	ip, [r4, #3072]	@ zero_extendqisi2
	ldr	r1, [r4, #3948]
	cmp	ip, #0
	strh	r0, [r4, r3]	@ movhi
	mov	r3, #0
	str	r3, [r4, #3960]
	movw	r3, #3940
	ldrh	r0, [r4, r3]
	add	r2, r0, #3
	strh	r2, [r4, r3]	@ movhi
	add	r2, r1, #3
	str	r2, [r4, #3948]
	addne	r0, r0, #4
	addne	r1, r1, #5
	strneh	r0, [r4, r3]	@ movhi
	strne	r1, [r4, #3948]
	bne	.L392
.L391:
	cmp	r2, #7
	movls	r3, #8
	strls	r3, [r4, #3948]
.L392:
	movw	r3, #3964
	mov	r2, #0
	strh	r2, [r4, r3]	@ movhi
	movw	r2, #3860
	ldrh	r2, [r4, r2]
	mov	r0, #0
	mov	r1, r2, lsr #3
	add	r2, r1, r2, asl #1
	add	r2, r2, #52
	add	r6, r2, r6, asl #2
	cmp	r6, r5, asl #9
	ldrcc	r2, .L395
	movcc	r1, #1
	strcch	r1, [r2, r3]	@ movhi
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L396:
	.align	2
.L395:
	.word	.LANCHOR0
	.word	.LANCHOR0+3878
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
	ldr	r3, .L404
	movw	r2, #3966
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L402
	movw	r2, #3940
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #3968]
	mov	r3, #0
	b	.L399
.L401:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L403
	add	r3, r3, #1
	uxth	r3, r3
.L399:
	cmp	r3, r1
	bne	.L401
	mov	r0, #0
	bx	lr
.L403:
	mov	r0, #1
	bx	lr
.L402:
	mov	r0, r2
	bx	lr
.L405:
	.align	2
.L404:
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
	ldr	r2, .L414
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #3972]
	cmp	r3, #0
	beq	.L411
	ldr	ip, [r2, #3976]
	mov	r2, #0
.L410:
	ldr	r4, [ip, #16]
	cmp	r4, r0
	bcc	.L408
	cmp	r4, r1
	bls	.L412
.L408:
	add	r2, r2, #1
	add	ip, ip, #36
	cmp	r2, r3
	bne	.L410
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L412:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L411:
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L415:
	.align	2
.L414:
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
	ldr	r3, .L417
	ldr	r0, [r3, #3960]
	bx	lr
.L418:
	.align	2
.L417:
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
	ldr	r3, .L420
	ldr	r0, [r3, #3960]
	bx	lr
.L421:
	.align	2
.L420:
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
	ldr	r3, .L423
	ldr	r0, [r3, #3980]
	bx	lr
.L424:
	.align	2
.L423:
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
	.save {r0, r1, r4, r5, r6, lr}
	movw	r3, #3916
	ldr	r4, .L426
	mov	r5, r0
	ldrh	r6, [r4, r3]
	mov	r1, r6
	bl	__aeabi_uidiv
	uxth	r2, r0
	mls	r3, r6, r2, r5
	add	r1, r4, r2, asl #2
	mov	r6, #1
	ldr	r0, [r1, #4012]
	uxth	r3, r3
	and	r1, r3, #31
	mov	ip, r3, lsr #5
	ldr	lr, [r0, ip, asl #2]
	orr	r1, lr, r6, asl r1
	str	r1, [r0, ip, asl #2]
	str	r1, [sp, #0]
	mov	r1, r5
	ldr	r0, .L426+4
	bl	printk
	movw	r3, #3990
	ldrh	r2, [r4, r3]
	mov	r0, #0
	add	r2, r2, r6
	strh	r2, [r4, r3]	@ movhi
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L427:
	.align	2
.L426:
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
	movw	r3, #3916
	ldr	r5, .L429
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
	ldr	r3, [r5, #4012]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr r4
	and	r0, r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L430:
	.align	2
.L429:
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
	ldr	r6, .L436
	movw	r8, #3916
	movw	r7, #3874
	b	.L433
.L435:
	mov	r0, r5
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	uxth	r5, r5
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
.L433:
	ldrh	r3, [r6, r8]
	ldrh	r2, [r6, r7]
	mul	r3, r2, r3
	cmp	r5, r3
	blt	.L435
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L437:
	.align	2
.L436:
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
	movw	r3, #3876
	ldr	r5, .L439
	mov	r7, r1
	mov	r6, r0
	ldrh	r4, [r5, r3]
	mov	r1, r4
	bl	__aeabi_uidiv
	movw	r3, #3916
	ldrh	r5, [r5, r3]
	mov	r1, r4
	mul	r7, r4, r7
	mla	r5, r5, r0, r7
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r5, r1
	uxth	r0, r1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L440:
	.align	2
.L439:
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
	ldr	r3, .L442
	movw	r2, #3876
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
.L443:
	.align	2
.L442:
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
	movw	r3, #3916
	ldr	r4, .L445
	ldrh	r1, [r4, r3]
	bl	__aeabi_uidivmod
	movw	r3, #3876
	uxth	r0, r1
	ldrh	r1, [r4, r3]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L446:
	.align	2
.L445:
	.word	.LANCHOR0
	.fnend
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	2
	.type	FtlFreeSysBlkQueueIn.part.7, %function
FtlFreeSysBlkQueueIn.part.7:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	beq	.L448
	ldr	r4, .L449
	ldr	r3, [r4, #4044]
	cmp	r3, #0
	bne	.L448
	bl	P2V_block_in_plane
	mov	r1, #1
	mov	r3, r5, asl #10
	mov	r2, r1
	mov	r6, r0
	ldr	r0, [r4, #4048]
	mov	r6, r6, asl #1
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #4052]
	ldrh	r2, [r3, r6]
	add	r2, r2, #1
	strh	r2, [r3, r6]	@ movhi
	ldr	r3, [r4, #4056]
	add	r3, r3, #1
	str	r3, [r4, #4056]
.L448:
	ldr	r3, .L449
	movw	r2, #4066
	movw	r0, #4060
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #4064
	ldrh	r1, [r3, r2]
	add	ip, r1, #4
	add	r1, r1, #1
	bic	r1, r1, #64512
	add	ip, r3, ip, asl #1
	strh	r5, [ip, r0]	@ movhi
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L450:
	.align	2
.L449:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueIn.part.7, .-FtlFreeSysBlkQueueIn.part.7
	.align	2
	.global	ftl_cmp_data_ver
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, r1
	bls	.L452
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L452:
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
	ldr	r2, .L455
	movw	r3, #4066
	ldrh	r0, [r2, r3]
	rsbs	r0, r0, #1
	movcc	r0, #0
	bx	lr
.L456:
	.align	2
.L455:
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
	ldr	r2, .L458
	movw	r3, #4066
	ldrh	r0, [r2, r3]
	sub	r2, r0, #1024
	rsbs	r0, r2, #0
	adc	r0, r0, r2
	bx	lr
.L459:
	.align	2
.L458:
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
	@ link register save eliminated.
	sub	r2, r0, #1
	movw	r3, #65533
	uxth	r2, r2
	cmp	r2, r3
	bxhi	lr
	ldr	r2, .L462
	movw	r3, #4066
	ldrh	r3, [r2, r3]
	cmp	r3, #1024
	bxeq	lr
	b	FtlFreeSysBlkQueueIn.part.7
.L463:
	.align	2
.L462:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L468
	ldr	r2, .L468+4
	ldr	r1, .L468+8
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r4, [r2, r3]
	movw	r3, #4066
	ldrh	r3, [r1, r3]
	and	r4, r4, #31
	cmp	r3, #0
	ldmeqfd	sp!, {r4, r5, pc}
	movw	r3, #4062
	mov	r0, #0
	ldrh	r2, [r1, r3]
	mov	r3, #4064
	ldrh	r3, [r1, r3]
	b	.L466
.L467:
	add	ip, r1, r2, asl #1
	add	r2, r2, #1
	add	ip, ip, #4064
	add	r0, r0, #1
	mov	r2, r2, asl #22
	ldrh	r5, [ip, #4]
	add	ip, r1, r3, asl #1
	add	r3, r3, #1
	add	ip, ip, #4064
	mov	r2, r2, lsr #22
	uxth	r0, r0
	mov	r3, r3, asl #22
	strh	r5, [ip, #4]	@ movhi
	mov	r3, r3, lsr #22
.L466:
	cmp	r0, r4
	ldr	ip, .L468+8
	blt	.L467
	movw	r1, #4062
	strh	r2, [ip, r1]	@ movhi
	mov	r2, #4064
	strh	r3, [ip, r2]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L469:
	.align	2
.L468:
	.word	-2040
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	movw	r6, #4066
	ldr	r4, .L477
	movw	r7, #4062
.L471:
	ldrh	r1, [r4, r6]
	cmp	r1, #0
	beq	.L472
	ldrh	r2, [r4, r7]
	sub	r1, r1, #1
	ldr	sl, [r4, #4044]
	add	r3, r4, r2, asl #1
	cmp	sl, #0
	add	r3, r3, #4064
	add	r2, r2, #1
	bic	r2, r2, #64512
	ldrh	r5, [r3, #4]
	strh	r1, [r4, r6]	@ movhi
	strh	r2, [r4, r7]	@ movhi
	bne	.L473
	mov	r0, r5
	bl	P2V_block_in_plane
	mov	r3, r5, asl #10
	mov	r8, r0
	ldr	r0, [r4, #4048]
	str	r3, [r0, #4]
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L474
	mov	r1, sl
	mov	r2, #1
	bl	FlashEraseBlocks
.L474:
	mov	r1, #1
	ldr	r0, [r4, #4048]
	mov	r2, r1
	mov	r8, r8, asl #1
	bl	FlashEraseBlocks
	ldr	r3, [r4, #4052]
	ldrh	r2, [r3, r8]
	add	r2, r2, #1
	strh	r2, [r3, r8]	@ movhi
	ldr	r3, [r4, #4056]
	add	r3, r3, #1
	str	r3, [r4, #4056]
	b	.L473
.L472:
	ldr	r0, .L477+4
	bl	printk
.L475:
	b	.L475
.L473:
	sub	r2, r5, #1
	movw	r3, #65533
	uxth	r2, r2
	cmp	r2, r3
	bls	.L476
	mov	r1, r5
	ldrh	r2, [r4, r6]
	ldr	r0, .L477+8
	bl	printk
	b	.L471
.L476:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L478:
	.align	2
.L477:
	.word	.LANCHOR0
	.word	.LC4
	.word	.LC5
	.fnend
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	test_node_in_list
	.type	test_node_in_list, %function
test_node_in_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L484
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	movw	r4, #65535
	ldr	r2, [r0, #0]
	ldr	r0, [r3, #-2020]
	ldr	ip, .L484+4
	rsb	r3, r0, r2
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	mov	ip, #6
	uxth	r3, r3
.L481:
	cmp	r1, r3
	beq	.L482
	ldrh	r3, [r2, #0]
	cmp	r3, r4
	beq	.L483
	mla	r2, ip, r3, r0
	b	.L481
.L482:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L483:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L485:
	.align	2
.L484:
	.word	.LANCHOR2
	.word	-1431655765
	.fnend
	.size	test_node_in_list, .-test_node_in_list
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L501
	movw	r3, #3860
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r3, [r2, r3]
	cmp	r3, r0
	bls	.L487
	ldr	r1, .L501+4
	mov	ip, #6
	mul	ip, ip, r0
	mvn	r6, #0
	ldr	r4, [r1, #-2020]
	mov	r8, r1
	add	r2, r4, ip
	strh	r6, [r2, #2]	@ movhi
	strh	r6, [r4, ip]	@ movhi
	ldr	r3, [r1, #-2016]
	cmp	r3, #0
	beq	.L500
.L488:
	ldr	r7, [r1, #-2012]
	mov	r5, r0, asl #1
	ldrh	r1, [r2, #4]
	ldr	fp, [r8, #-2020]
	ldrh	sl, [r7, r5]
	cmp	r1, #0
	ldr	r8, .L501+8
	mulne	r6, r1, sl
	rsb	r1, fp, r3
	movw	sl, #3860
	mov	r1, r1, asr #1
	mul	r1, r8, r1
	ldr	r8, .L501
	ldrh	sl, [r8, sl]
	ldr	r8, [r8, #4052]
	uxth	r1, r1
	add	r5, r8, r5
	str	r5, [sp, #12]
	mov	r5, #0
	str	sl, [sp, #8]
	str	r8, [sp, #4]
.L495:
	add	r5, r5, #1
	ldr	r8, [sp, #8]
	uxth	r5, r5
	cmp	r5, r8
	bhi	.L487
	cmp	r0, r1
	beq	.L487
	mov	r9, r1, asl #1
	ldrh	r8, [r3, #4]
	ldrh	sl, [r7, r9]
	cmp	r8, #0
	mvneq	r8, #0
	mulne	r8, r8, sl
	cmp	r8, r6
	bne	.L491
	ldr	r8, [sp, #4]
	ldrh	sl, [r8, r9]
	ldr	r9, [sp, #12]
	ldrh	r8, [r9, #0]
	cmp	sl, r8
	bcc	.L493
	b	.L492
.L491:
	bhi	.L492
.L493:
	ldrh	r8, [r3, #0]
	movw	sl, #65535
	cmp	r8, sl
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	ldreq	r3, .L501+4
	streq	r2, [r3, #-2008]
	beq	.L487
.L494:
	mov	r1, #6
	mla	r3, r1, r8, fp
	mov	r1, r8
	b	.L495
.L492:
	strh	r1, [r4, ip]	@ movhi
	ldrh	r1, [r3, #2]
	strh	r1, [r2, #2]	@ movhi
	ldr	r1, .L501+4
	ldr	ip, [r1, #-2016]
	cmp	r3, ip
	bne	.L496
	strh	r0, [r3, #2]	@ movhi
.L500:
	str	r2, [r1, #-2016]
	b	.L487
.L496:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-2020]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L487:
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L502:
	.align	2
.L501:
	.word	.LANCHOR0
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
	ldr	r2, .L504
	ldr	r3, .L504+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L505:
	.align	2
.L504:
	.word	.LANCHOR2
	.word	-2004
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
	beq	.L507
	ldr	r1, .L514
	mov	r5, #6
	mul	ip, r5, r0
	mvn	r3, #0
	ldr	r4, [r1, #-2020]
	add	r2, r4, ip
	strh	r3, [r2, #2]	@ movhi
	strh	r3, [r4, ip]	@ movhi
	ldr	r3, [r1, #-2000]
	cmp	r3, #0
	beq	.L513
.L508:
	ldr	sl, [r1, #-2020]
	mov	r8, r0, asl #1
	ldr	r9, .L514+4
	rsb	r1, sl, r3
	ldr	r7, .L514+8
	mov	r1, r1, asr #1
	ldr	r7, [r7, #4052]
	mul	r1, r9, r1
	mov	r9, r5
	ldrh	r8, [r7, r8]
	uxth	r1, r1
.L511:
	mov	r5, r1, asl #1
	ldrh	r5, [r7, r5]
	cmp	r5, r8
	bcs	.L509
	ldrh	r5, [r3, #0]
	cmp	r5, r6
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	beq	.L507
.L510:
	mla	r3, r9, r5, sl
	mov	r1, r5
	b	.L511
.L509:
	ldrh	r5, [r3, #2]
	strh	r5, [r2, #2]	@ movhi
	strh	r1, [r4, ip]	@ movhi
	ldr	r1, .L514
	ldr	ip, [r1, #-2000]
	cmp	r3, ip
	bne	.L512
	strh	r0, [r3, #2]	@ movhi
.L513:
	str	r2, [r1, #-2000]
	b	.L507
.L512:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-2020]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L507:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L515:
	.align	2
.L514:
	.word	.LANCHOR2
	.word	-1431655765
	.word	.LANCHOR0
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
	ldr	r2, .L517
	ldr	r3, .L517+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L518:
	.align	2
.L517:
	.word	.LANCHOR2
	.word	-1996
	.fnend
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.type	List_remove_node, %function
List_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L524
	mov	r2, #6
	mul	r1, r2, r1
	ldr	r3, [ip, #-2020]
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	r5, r3, r1
	ldr	r4, [r0, #0]
	movw	r6, #65535
	cmp	r5, r4
	ldrh	r4, [r3, r1]
	bne	.L520
	cmp	r4, r6
	mlane	r2, r2, r4, r3
	moveq	r2, #0
	streq	r2, [r0, #0]
	strne	r2, [r0, #0]
	mvnne	r0, #0
	strneh	r0, [r2, #2]	@ movhi
	b	.L522
.L520:
	cmp	r4, r6
	ldrh	r0, [r5, #2]
	bne	.L523
	cmp	r0, r4
	mulne	r2, r2, r0
	mvnne	r0, #0
	strneh	r0, [r3, r2]	@ movhi
	b	.L522
.L523:
	mla	r4, r2, r4, r3
	strh	r0, [r4, #2]	@ movhi
	ldrh	r6, [r5, #2]
	ldrh	r4, [r3, r1]
	ldr	r0, [ip, #-2020]
	mul	r2, r2, r6
	strh	r4, [r0, r2]	@ movhi
.L522:
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r3, r1]	@ movhi
	strh	r2, [r5, #2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L525:
	.align	2
.L524:
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
	beq	.L531
	ldr	r2, .L532
	movw	lr, #65535
	mov	ip, #6
	ldr	r4, [r2, #-2020]
	b	.L528
.L530:
	mla	r3, ip, r2, r4
	sub	r1, r1, #1
	uxth	r1, r1
.L528:
	cmp	r1, #0
	beq	.L529
	ldrh	r2, [r3, #0]
	cmp	r2, lr
	bne	.L530
.L529:
	rsb	r4, r4, r3
	ldr	r3, .L532+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L531:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L533:
	.align	2
.L532:
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
	ldr	r2, .L540
	ldr	r3, [r2, #-2016]
	cmp	r3, #0
	beq	.L539
	ldr	r2, [r2, #-2020]
	movw	r1, #65535
	mov	ip, #6
	b	.L536
.L538:
	mla	r3, ip, r3, r2
	sub	r0, r0, #1
	uxth	r0, r0
.L536:
	cmp	r0, #0
	beq	.L537
	ldrh	r3, [r3, #0]
	cmp	r3, r1
	bne	.L538
	mov	r0, r1
	bx	lr
.L537:
	rsb	r3, r2, r3
	ldr	r0, .L540+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	bx	lr
.L539:
	movw	r0, #65535
	bx	lr
.L541:
	.align	2
.L540:
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
	mov	r4, r0
	ldr	r3, .L549
	ldr	r2, .L549+4
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L543
	ldr	r2, .L549+8
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L543
	ldr	r2, .L549+12
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L543
	mov	r5, #6
	ldr	r2, [r3, #-2020]
	mul	r5, r5, r0
	ldr	r0, [r3, #-2016]
	add	r1, r2, r5
	cmp	r1, r0
	beq	.L543
	ldr	ip, [r3, #-2012]
	mov	r3, r4, asl #1
	ldrh	r0, [ip, r3]
	ldrh	r3, [r1, #4]
	cmp	r3, #0
	mulne	r0, r3, r0
	ldrh	r3, [r1, #2]
	mvneq	r0, #0
	movw	r1, #65535
	cmp	r3, r1
	bne	.L545
	ldrh	r1, [r2, r5]
	cmp	r1, r3
	beq	.L543
.L545:
	mov	r1, #6
	mla	r3, r1, r3, r2
	ldr	r1, .L549+16
	rsb	r2, r2, r3
	mov	r2, r2, asr #1
	ldrh	r3, [r3, #4]
	mul	r2, r1, r2
	cmp	r3, #0
	mvneq	r3, #0
	uxth	r2, r2
	mov	r2, r2, asl #1
	ldrh	r2, [ip, r2]
	mulne	r3, r3, r2
	cmp	r0, r3
	bcs	.L543
	ldr	r5, .L549
	mov	r1, r4
	sub	r0, r5, #2016
	bl	List_remove_node
	ldr	r3, .L549+20
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	INSERT_DATA_LIST
.L543:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L550:
	.align	2
.L549:
	.word	.LANCHOR2
	.word	-1992
	.word	-1944
	.word	-1896
	.word	-1431655765
	.word	-2004
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
	b	.L552
.L557:
	mov	r7, r3
	add	r3, r3, #2
	ldrh	r6, [r7, #0]
	cmp	r6, #0
	bne	.L553
	bl	FtlFreeSysBlkQueueOut
	movw	r3, #65533
	sub	r2, r0, #1
	mov	r1, r0
	strh	r0, [r7, #0]	@ movhi
	uxth	r2, r2
	cmp	r2, r3
	bls	.L554
	ldr	r2, .L558
	movw	r3, #4066
	ldr	r0, .L558+4
	ldrh	r2, [r2, r3]
	bl	printk
.L555:
	b	.L555
.L554:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4, #0]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L556
.L553:
	add	r5, r5, #1
	uxth	r5, r5
.L552:
	cmp	r5, r2
	bne	.L557
.L556:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L559:
	.align	2
.L558:
	.word	.LANCHOR0
	.word	.LC6
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L572
	movw	r3, #3958
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	movw	r1, #65535
	ldrh	r3, [r2, r3]
	ldr	r2, .L572+4
	ldr	ip, [r2, #-1848]
	mov	r2, #0
	mov	r0, r2
	b	.L561
.L563:
	add	r2, r2, #12
	add	r4, ip, r2
	ldrh	r4, [r4, #-12]
	cmp	r4, r1
	ldmeqfd	sp!, {r4, r5, r6, r7, pc}
	add	r0, r0, #1
	uxth	r0, r0
.L561:
	cmp	r0, r3
	bne	.L563
	mov	r1, #0
	mov	r0, r3
	mov	r5, #-2147483648
	mov	r2, r1
	b	.L564
.L566:
	add	r4, ip, r1
	ldr	r4, [r4, #4]
	cmp	r4, #0
	blt	.L565
	cmp	r4, r5
	movcc	r5, r4
	movcc	r0, r2
.L565:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L564:
	cmp	r2, r3
	bne	.L566
	cmp	r0, r3
	ldmccfd	sp!, {r4, r5, r6, r7, pc}
	ldr	r2, .L572+8
	mov	r0, r3
	ldr	r1, .L572+4
	mvn	r4, #0
	ldrh	r6, [r1, r2]
	mov	r2, #0
	mov	r1, r2
	b	.L567
.L569:
	add	r5, ip, r2
	ldr	r5, [r5, #4]
	cmp	r5, r4
	bcs	.L568
	ldrh	r7, [ip, r2]
	cmp	r7, r6
	movne	r4, r5
	movne	r0, r1
.L568:
	add	r1, r1, #1
	add	r2, r2, #12
	uxth	r1, r1
.L567:
	cmp	r1, r3
	bne	.L569
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L573:
	.align	2
.L572:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1844
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L580
	ldr	r2, .L580+4
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r1, [r2, r3]
	cmp	r1, #4
	add	ip, r1, #1
	mov	r1, r3
	strh	ip, [r2, r3]	@ movhi
	bhi	.L575
	cmp	r0, #0
	ldmeqfd	sp!, {r4, r5, pc}
.L575:
	ldr	r0, .L580+8
	mov	r3, #0
	strh	r3, [r2, r1]	@ movhi
	movw	r1, #3860
	str	r3, [r2, #-1840]
	movw	ip, #65535
	ldrh	r4, [r0, r1]
	ldr	r1, [r2, #-2012]
	ldr	r2, .L580+4
	b	.L577
.L579:
	ldrh	r0, [r1], #2
	add	r3, r3, #1
	cmp	r0, ip
	uxth	r3, r3
	ldrne	r5, [r2, #-1840]
	addne	r0, r0, r5
	strne	r0, [r2, #-1840]
.L577:
	cmp	r3, r4
	bne	.L579
	ldmfd	sp!, {r4, r5, pc}
.L581:
	.align	2
.L580:
	.word	-1842
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
	ldr	r3, .L586
	mov	r2, r0, lsr #5
	cmp	r1, #0
	and	r0, r0, #31
	uxth	r2, r2
	mov	ip, #1
	ldr	r3, [r3, #-1836]
	ldr	r1, [r3, r2, asl #2]
	orrne	r0, r1, ip, asl r0
	biceq	r0, r1, ip, asl r0
	str	r0, [r3, r2, asl #2]
	bx	lr
.L587:
	.align	2
.L586:
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
	ldr	r3, .L589
	mov	r2, r0, lsr #5
	and	r0, r0, #31
	ldr	r3, [r3, #-1836]
	ldr	r3, [r3, r2, asl #2]
	mov	r0, r3, lsr r0
	and	r0, r0, #1
	bx	lr
.L590:
	.align	2
.L589:
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
	ldr	ip, .L598
	mov	r3, #0
	strh	r3, [r0, #4]	@ movhi
	movw	r3, #3852
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r3, [ip, r3]
	movw	ip, #65535
	b	.L592
.L594:
	add	r4, r0, r2, asl #1
	add	r2, r2, #1
	ldrh	r4, [r4, #16]
	uxth	r2, r2
	cmp	r4, ip
	ldrneh	r4, [r0, #4]
	addne	r4, r4, #1
	strneh	r4, [r0, #4]	@ movhi
.L592:
	cmp	r2, r3
	bcc	.L594
	ldr	ip, .L598
	movw	r2, #3918
	movw	r4, #65535
	mvn	r1, r1
	ldrh	r5, [ip, r2]
	mov	ip, r0
	mov	r2, #0
	b	.L595
.L597:
	ldrh	r6, [ip, #16]
	add	r2, r2, #1
	add	ip, ip, #2
	cmp	r6, r4
	uxth	r2, r2
	ldrneh	r6, [r0, #4]
	addne	r6, r5, r6
	addne	r6, r6, r1
	strneh	r6, [r0, #4]	@ movhi
.L595:
	cmp	r2, r3
	bne	.L597
	ldmfd	sp!, {r4, r5, r6, pc}
.L599:
	.align	2
.L598:
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
	ldr	r6, .L606
	strh	r5, [r0, #4]	@ movhi
	movw	r8, #3852
	strb	r5, [r4, #7]
	mvn	r7, #0
	b	.L601
.L603:
	add	r3, r6, r5
	ldrh	r1, [r4, #0]
	add	sl, r5, #8
	add	r5, r5, #1
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
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
.L601:
	ldrh	r2, [r6, r8]
	ldr	r3, .L606
	cmp	r2, r5
	bhi	.L603
	movw	r1, #3918
	ldrb	r2, [r4, #7]	@ zero_extendqisi2
	ldrh	r1, [r3, r1]
	mul	r2, r1, r2
	strh	r2, [r4, #4]	@ movhi
	mov	r2, #0
	strb	r2, [r4, #9]
	ldr	r2, [r3, #3844]
	cmp	r2, #0
	beq	.L604
	ldrh	r1, [r4, #0]
	ldr	r2, [r3, #4052]
	mov	r3, r1, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #79
	movls	r3, #1
	strlsb	r3, [r4, #9]
.L604:
	ldr	r3, .L606
	mov	r0, #0
	ldrb	r3, [r3, #852]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #1
	strneb	r3, [r4, #9]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L607:
	.align	2
.L606:
	.word	.LANCHOR0
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
	ldr	r6, .L613
	movw	sl, #3852
	movw	r8, #3918
	b	.L609
.L611:
	add	r3, r6, r5
	mov	r1, r7
	add	r5, r5, #1
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	uxth	r5, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreqh	r3, [r6, r8]
	addeq	r4, r4, r3
	uxtheq	r4, r4
.L609:
	ldrh	r3, [r6, sl]
	cmp	r3, r5
	bhi	.L611
	cmp	r4, #0
	beq	.L612
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L612:
	ldr	r3, .L613+4
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #-2020]
	mla	r7, r2, r7, r3
	strh	r4, [r7, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L614:
	.align	2
.L613:
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
	ldr	r3, .L618
	ldr	r0, [r3, #-2000]
	cmp	r0, #0
	bxeq	lr
	ldr	r3, [r3, #-2020]
	ldr	r2, .L618+4
	rsb	r0, r3, r0
	ldr	r3, .L618+8
	mov	r0, r0, asr #1
	ldr	r2, [r2, #4052]
	mul	r3, r3, r0
	uxth	r3, r3
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	bx	lr
.L619:
	.align	2
.L618:
	.word	.LANCHOR2
	.word	.LANCHOR0
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
	ldr	r2, .L627
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r3, [r2, #-2000]
	cmp	r3, #0
	beq	.L626
	ldr	r1, .L627+4
	mov	ip, #7
	mov	r5, #6
	movw	r4, #65535
	ldrh	r1, [r2, r1]
	mul	r1, ip, r1
	mov	r1, r1, asr #3
	cmp	r0, r1
	uxthgt	r0, r1
	ldr	r1, [r2, #-2020]
	ldr	r2, .L627+8
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
	b	.L623
.L625:
	mul	ip, r5, r3
	ldrh	ip, [r1, ip]
	cmp	ip, r4
	beq	.L624
	add	r2, r2, #1
	mov	r3, ip
	uxth	r2, r2
.L623:
	cmp	r2, r0
	bne	.L625
.L624:
	ldr	r2, .L627+12
	mov	r3, r3, asl #1
	ldr	r2, [r2, #4052]
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L626:
	mov	r0, r3
	ldmfd	sp!, {r4, r5, pc}
.L628:
	.align	2
.L627:
	.word	.LANCHOR2
	.word	-1996
	.word	-1431655765
	.word	.LANCHOR0
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
	mov	sl, r0
	ldr	r6, .L639
	add	r4, sl, #12
	.pad #32
	sub	sp, sp, #32
	ldr	r1, .L639+4
	bl	strcpy
	mov	r0, r4
	ldr	r1, .L639+8
	ldr	r2, [r6, #3044]
	bl	sprintf
	ldr	r1, .L639+12
	ldr	r2, [r6, #3936]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+16
	ldr	r3, [r3, #3220]
	cmp	r3, #1
	add	r4, r4, r0
	bne	.L635
.L630:
	add	r0, sp, #16
	add	r1, sp, #20
	add	r2, sp, #24
	add	r3, sp, #28
	bl	NandcGetTimeCfg
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #16]
	mov	r0, r4
	ldr	r1, .L639+20
	str	r3, [sp, #0]
	ldr	r3, [sp, #28]
	ldr	r5, .L639+24
	ldr	r7, .L639+28
	str	r3, [sp, #4]
	ldr	r3, [sp, #20]
	bl	sprintf
	ldr	r1, .L639+32
	ldr	r8, .L639+36
	add	r4, r4, r0
	mov	r0, r4
	add	r4, r4, #10
	bl	strcpy
	ldr	r1, .L639+40
	mov	r0, r4
	ldr	r2, [r6, #3980]
	bl	sprintf
	ldr	r1, .L639+44
	ldr	r2, [r5, #-1840]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+48
	ldr	r2, [r5, #-1832]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+52
	ldr	r2, [r5, #-1828]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+56
	ldr	r2, [r5, #-1824]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+60
	ldr	r2, [r5, #-1820]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+64
	ldr	r2, [r5, #-1816]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+68
	ldr	r2, [r5, #-1812]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r2, [r5, #-1808]
	ldr	r1, .L639+72
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r2, [r5, #-1804]
	ldr	r1, .L639+76
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+80
	ldr	r2, [r5, #-1800]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+84
	ldr	r2, [r5, #-1796]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	bl	FtlBbtCalcTotleCnt
	movw	r2, #3990
	ldr	r1, .L639+88
	ldrh	r2, [r6, r2]
	mov	r3, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+92
	ldrh	r2, [r5, r7]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+96
	ldr	r2, [r5, #-1792]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+100
	ldr	r2, [r5, #-1788]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+104
	ldr	r2, [r5, #-1784]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+108
	ldr	r2, [r6, #4056]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+112
	ldr	r2, [r5, #-1780]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+116
	ldr	r2, [r5, #-1776]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+120
	ldr	r1, .L639+124
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+128
	ldr	r1, .L639+132
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+136
	ldr	r2, [r6, #3960]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+140
	ldr	r2, [r6, #3952]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+144
	ldr	r2, [r6, #3848]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #4066
	ldrh	r2, [r6, r3]
	ldr	r1, .L639+148
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3860
	ldrh	r2, [r6, r3]
	ldr	r1, .L639+152
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+156
	ldr	r1, .L639+160
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+164
	ldr	r2, [r6, #3864]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+168
	ldr	r1, .L639+172
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	mov	r3, #3984
	ldrh	r2, [r6, r3]
	ldr	r1, .L639+176
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+180
	ldr	r1, .L639+184
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+188
	ldrb	r2, [r5, #-1986]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r2, [r5, r8]
	ldr	r1, .L639+192
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+196
	ldrb	r2, [r5, #-1984]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+200
	ldr	r1, .L639+204
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r5, r8]
	ldr	r2, [r5, #-2012]
	add	r8, r8, #48
	ldr	r1, .L639+208
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+212
	ldr	r1, .L639+216
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+220
	ldrb	r2, [r5, #-1938]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r2, [r5, r8]
	ldr	r1, .L639+224
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+228
	ldrb	r2, [r5, #-1936]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+232
	ldr	r1, .L639+236
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r5, r8]
	ldr	r2, [r5, #-2012]
	add	r8, r8, #192
	ldr	r1, .L639+240
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+244
	ldr	r1, .L639+248
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+252
	ldrb	r2, [r5, #-1890]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+256
	ldr	r1, .L639+260
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+264
	ldrb	r2, [r5, #-1888]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+268
	ldr	r1, .L639+272
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+276
	ldr	r1, .L639+280
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+284
	ldrb	r2, [r5, #-1746]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+288
	ldrh	r2, [r5, r8]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+292
	ldrb	r2, [r5, #-1744]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+296
	ldr	r1, .L639+300
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, [r5, #-1616]
	ldr	r3, [r6, #3844]
	ldr	r2, [r5, #-1704]
	str	r1, [sp, #0]
	ldr	r1, [r5, #-1624]
	orr	r2, r3, r2, asl #8
	str	r1, [sp, #4]
	ldr	r1, .L639+304
	ldr	r3, [r5, #-1620]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+308
	ldr	r2, [r5, #-1628]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+312
	ldr	r2, [r5, #-1604]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+316
	ldr	r1, .L639+320
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+324
	ldr	r1, .L639+328
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L639+332
	ldr	r2, [r5, #-1184]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L639+336
	ldr	r1, .L639+340
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L639+344
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	ldrh	r0, [r5, r7]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L639+348
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r5, r8]
	movw	r2, #65535
	cmp	r3, r2
	add	r4, r4, r0
	beq	.L632
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	mov	r0, r4
	ldr	r1, .L639+352
	ldrh	r2, [r2, r3]
	bl	sprintf
	add	r4, r4, r0
.L632:
	mov	r0, #0
	ldr	r6, .L639+24
	bl	List_get_gc_head_node
	ldr	r8, .L639
	mov	r5, #0
	movw	r9, #65535
	uxth	r3, r0
.L634:
	cmp	r3, r9
	beq	.L633
	ldr	r1, [r6, #-2012]
	mov	r2, r3, asl #1
	mov	r7, #6
	mov	r0, r4
	mul	r7, r7, r3
	ldrh	r1, [r1, r2]
	str	r1, [sp, #0]
	ldr	r1, [r6, #-2020]
	add	r1, r1, r7
	ldrh	r1, [r1, #4]
	str	r1, [sp, #4]
	ldr	r1, [r8, #4052]
	ldrh	r2, [r1, r2]
	ldr	r1, .L639+356
	str	r2, [sp, #8]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-2020]
	cmp	r5, #16
	ldrh	r3, [r3, r7]
	add	r4, r4, r0
	bne	.L634
.L633:
	ldr	r6, .L639+24
	mov	r5, #0
	ldr	r8, .L639
	movw	r9, #65535
	ldr	r2, [r6, #-2000]
	ldr	r3, [r6, #-2020]
	rsb	r3, r3, r2
	ldr	r2, .L639+360
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L636:
	cmp	r3, r9
	beq	.L635
	mov	r7, #6
	ldr	r2, [r6, #-2020]
	mul	r7, r7, r3
	mov	r0, r4
	add	r2, r2, r7
	ldrh	r2, [r2, #4]
	str	r2, [sp, #0]
	mov	r2, r3, asl #1
	ldr	r1, [r8, #4052]
	ldrh	r2, [r1, r2]
	ldr	r1, .L639+364
	str	r2, [sp, #4]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-2020]
	cmp	r5, #4
	ldrh	r3, [r3, r7]
	add	r4, r4, r0
	bne	.L636
.L635:
	rsb	r0, sl, r4
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L640:
	.align	2
.L639:
	.word	.LANCHOR0
	.word	.LC7
	.word	.LC8
	.word	.LC9
	.word	.LANCHOR1
	.word	.LC10
	.word	.LANCHOR2
	.word	-1996
	.word	.LC11
	.word	-1992
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
	.word	-2038
	.word	.LC32
	.word	-2040
	.word	.LC33
	.word	.LC34
	.word	.LC35
	.word	.LC36
	.word	.LC37
	.word	.LC38
	.word	-1772
	.word	.LC39
	.word	.LC40
	.word	-1768
	.word	.LC41
	.word	.LC42
	.word	-1990
	.word	.LC43
	.word	.LC44
	.word	.LC45
	.word	.LC46
	.word	-1988
	.word	.LC47
	.word	.LC48
	.word	-1942
	.word	.LC49
	.word	.LC50
	.word	.LC51
	.word	.LC52
	.word	-1940
	.word	.LC53
	.word	.LC54
	.word	-1894
	.word	.LC55
	.word	.LC56
	.word	-1896
	.word	.LC57
	.word	.LC58
	.word	-1892
	.word	.LC59
	.word	-1750
	.word	.LC60
	.word	.LC61
	.word	.LC62
	.word	.LC63
	.word	-1748
	.word	.LC64
	.word	.LC65
	.word	.LC66
	.word	.LC67
	.word	-1188
	.word	.LC68
	.word	-1186
	.word	.LC69
	.word	.LC70
	.word	-1180
	.word	.LC71
	.word	.LC72
	.word	.LC73
	.word	.LC74
	.word	.LC75
	.word	-1431655765
	.word	.LC76
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
	mov	r4, r0
	ldr	r1, .L642
	ldr	r2, .L642+4
	bl	sprintf
	add	r5, r4, r0
	mov	r0, r5
	bl	FtlPrintInfo2buf
	add	r0, r5, r0
	rsb	r0, r4, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L643:
	.align	2
.L642:
	.word	.LC77
	.word	.LC78
	.fnend
	.size	ftl_proc_ftl_read, .-ftl_proc_ftl_read
	.align	2
	.global	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L669
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r1, [r3, #-1784]
	ldr	r2, [r3, #-1776]
	cmp	r1, r2
	bcs	.L645
	ldr	r0, .L669+4
	movw	r1, #3860
	mov	r2, #0
	str	r2, [r3, #-1792]
	ldrh	r1, [r0, r1]
	ldr	r0, [r0, #4052]
	b	.L646
.L647:
	ldrh	lr, [r0], #2
	add	r2, r2, #1
	ldr	ip, [r3, #-1792]
	add	ip, lr, ip
	str	ip, [r3, #-1792]
.L646:
	cmp	r2, r1
	ldr	r4, .L669
	bcc	.L647
	ldr	r5, [r4, #-1792]
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r2, .L669+4
	movw	r3, #3910
	ldrh	r1, [r2, r3]
	str	r0, [r4, #-1784]
	ldr	r0, [r4, #-1788]
	rsb	r0, r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #-1792]
	b	.L648
.L645:
	ldr	r2, [r3, #-1780]
	cmp	r1, r2
	bls	.L648
	add	r2, r2, #1
	movw	ip, #3860
	str	r2, [r3, #-1780]
	mov	r3, #0
	ldr	r2, .L669+4
	b	.L649
.L650:
	ldr	r0, [r2, #4052]
	mov	r1, r3, asl #1
	add	r3, r3, #1
	ldrh	r4, [r0, r1]
	add	r4, r4, #1
	strh	r4, [r0, r1]	@ movhi
.L649:
	ldrh	r1, [r2, ip]
	cmp	r3, r1
	bcc	.L650
.L648:
	ldr	r3, .L669
	ldr	r6, [r3, #-1776]
	ldr	r5, [r3, #-1784]
	add	r2, r6, #256
	cmp	r2, r5
	mov	r2, r3
	bls	.L651
	ldr	r3, [r3, #-1780]
	add	r1, r6, #768
	cmp	r1, r3
	bls	.L651
	ldr	r3, .L669+4
	ldr	r3, [r3, #3844]
	cmp	r3, #0
	beq	.L668
	cmp	r6, #40
	bhi	.L668
.L651:
	ldr	r3, .L669+8
	ldrh	r0, [r2, r3]
	add	r0, r0, r0, asl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	cmp	r0, r3
	mov	r8, r0
	bcs	.L653
	cmp	r6, #40
	bhi	.L668
.L653:
	ldr	r1, .L669
	ldr	r3, [r1, #-2016]
	cmp	r3, #0
	beq	.L668
	ldr	r2, .L669+4
	movw	r0, #3860
	movw	r7, #65535
	mov	r4, r7
	mov	fp, r7
	ldrh	sl, [r2, r0]
	ldr	r0, [r1, #-2020]
	ldr	r1, [r2, #4052]
	mov	r2, #0
	b	.L654
.L657:
	add	r2, r2, #1
	uxth	r2, r2
	cmp	r2, sl
	bhi	.L668
	ldrh	ip, [r3, #4]
	cmp	ip, #0
	beq	.L655
	rsb	r3, r0, r3
	ldr	ip, .L669+12
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	ip, r3, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, r6
	bls	.L665
	cmp	ip, r7
	movcc	r7, ip
	movcc	r4, r3
.L655:
	mov	ip, #6
	mla	r3, ip, r9, r0
.L654:
	ldrh	r9, [r3, #0]
	cmp	r9, fp
	bne	.L657
	b	.L656
.L665:
	mov	r4, r3
.L656:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L652
	mov	r9, r4, asl #1
	ldrh	sl, [r1, r9]
	cmp	sl, r6
	bls	.L658
	bl	GetFreeBlockMinEraseCount
	cmp	r0, r6
	ldrhi	r3, .L669
	strhi	r7, [r3, #-1776]
.L658:
	cmp	sl, r5
	bcs	.L668
	add	r3, sl, #128
	cmp	r8, r3
	ble	.L668
	add	r3, sl, #256
	ldr	r6, .L669
	cmp	r3, r5
	bcc	.L659
	ldr	r3, [r6, #-1780]
	add	sl, sl, #768
	cmp	sl, r3
	bcs	.L668
.L659:
	ldr	r3, [r6, #-2012]
	mov	r1, r4
	ldr	r0, .L669+16
	mov	r2, r5
	ldrh	r3, [r3, r9]
	str	r3, [sp, #0]
	ldr	r3, .L669+4
	ldr	r3, [r3, #4052]
	ldrh	r3, [r3, r9]
	stmib	sp, {r3, r8}
	ldr	r3, [r6, #-1780]
	bl	printk
	mov	r3, #1
	str	r3, [r6, #-1176]
	b	.L652
.L668:
	movw	r4, #65535
.L652:
	mov	r0, r4
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L670:
	.align	2
.L669:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1996
	.word	-1431655765
	.word	.LC79
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
	beq	.L672
	ldr	r2, .L673
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #-2012]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L672:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L674:
	.align	2
.L673:
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
	ldr	r2, .L683
	mov	r3, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	sl, #3852
	str	r3, [r2, #-1172]
	mov	r8, #12
	ldr	r0, .L683+4
	mov	r7, #1
	movw	r6, #3928
	movw	fp, #3930
	b	.L676
.L677:
	mul	r1, r8, r3
	ldr	r4, [r2, #-1168]
	add	ip, r4, r1
	str	r7, [ip, #8]
	ldrh	ip, [r0, r6]
	mul	ip, ip, r3
	add	r5, ip, #3
	cmp	ip, #0
	movlt	ip, r5
	ldr	r5, [r2, #-1164]
	bic	ip, ip, #3
	add	ip, r5, ip
	str	ip, [r4, r1]
	ldrh	ip, [r0, fp]
	ldr	r9, [r2, #-1168]
	mul	ip, ip, r3
	add	r4, r9, r1
	add	r5, ip, #3
	cmp	ip, #0
	movlt	ip, r5
	ldr	r5, [r2, #-1160]
	bic	ip, ip, #3
	add	ip, r5, ip
	str	ip, [r4, #4]
	ldr	ip, [r2, #-1156]
	mov	r5, #36
	ldr	r1, [r9, r1]
	mla	ip, r5, r3, ip
	add	r3, r3, #1
	uxth	r3, r3
	str	r1, [ip, #8]
	ldr	r1, [r4, #4]
	str	r1, [ip, #12]
.L676:
	ldrh	r1, [r0, sl]
	cmp	r3, r1
	bcc	.L677
	b	.L682
.L679:
	mul	r2, r8, r1
	ldr	r4, [r3, #-1168]
	add	r0, r4, r2
	str	r7, [r0, #8]
	ldrh	r0, [ip, r6]
	mul	r0, r0, r1
	add	sl, r0, #3
	cmp	r0, #0
	movlt	r0, sl
	ldr	sl, [r3, #-1164]
	bic	r0, r0, #3
	add	r0, sl, r0
	str	r0, [r4, r2]
	ldr	r0, [r3, #-1168]
	add	r0, r0, r2
	ldrh	r2, [ip, r5]
	mul	r2, r2, r1
	add	r1, r1, #1
	uxth	r1, r1
	add	r4, r2, #3
	cmp	r2, #0
	movlt	r2, r4
	ldr	r4, [r3, #-1160]
	bic	r2, r2, #3
	add	r2, r4, r2
	str	r2, [r0, #4]
	b	.L681
.L682:
	ldr	r3, .L683
	mov	r8, #12
	ldr	ip, .L683+4
	mov	r7, #0
	movw	r6, #3928
	movw	r5, #3930
.L681:
	ldr	r2, [r3, #-1152]
	cmp	r1, r2
	bcc	.L679
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L684:
	.align	2
.L683:
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
	ldr	r3, .L692
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, #36
	ldr	r6, [r3, #-1152]
	mov	fp, #12
	ldr	ip, [r3, #-1168]
	mov	r3, #0
	mov	r4, r3
	b	.L686
.L689:
	mul	sl, fp, r2
	add	r8, ip, sl
	ldr	r9, [ip, sl]
	ldr	sl, [r7, #8]
	cmp	r9, sl
	streq	r4, [r8, #8]
	beq	.L688
.L687:
	add	r2, r2, #1
	uxth	r2, r2
.L691:
	cmp	r2, r6
	bcc	.L689
.L688:
	add	r3, r3, #1
	uxth	r3, r3
.L686:
	cmp	r3, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
	mla	r7, r5, r3, r0
	mov	r2, #0
	b	.L691
.L693:
	.align	2
.L692:
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
	ldr	r3, .L701
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #12
	ldr	r8, [r3, #-1152]
	mov	r5, #1
	ldr	r7, [r3, #-1168]
	mov	r4, #36
	mov	r3, #0
	b	.L695
.L698:
	mla	ip, r6, r2, r7
	ldr	sl, [ip, #8]
	cmp	sl, #0
	bne	.L696
	mla	r2, r4, r3, r0
	ldr	sl, [ip, #0]
	str	r5, [ip, #8]
	str	sl, [r2, #8]
	ldr	ip, [ip, #4]
	str	ip, [r2, #12]
	b	.L697
.L696:
	add	r2, r2, #1
	uxth	r2, r2
	b	.L699
.L700:
	mov	r2, #0
.L699:
	cmp	r2, r8
	bcc	.L698
.L697:
	add	r3, r3, #1
	uxth	r3, r3
.L695:
	cmp	r3, r1
	bcc	.L700
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L702:
	.align	2
.L701:
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
	ldr	r3, .L708
	ldr	r2, .L708+4
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #-1144]
	mov	r3, #0
	b	.L704
.L706:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L707
	add	r3, r3, #1
	uxth	r3, r3
.L704:
	cmp	r3, r1
	bne	.L706
	mov	r0, #0
	bx	lr
.L707:
	mov	r0, #1
	bx	lr
.L709:
	.align	2
.L708:
	.word	.LANCHOR2
	.word	-1148
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
	ldr	r3, .L714
	ldr	r2, .L714+4
	ldrh	ip, [r3, r2]
	ldr	r2, [r3, #-1144]
	mov	r3, #0
	mov	r1, r2
	b	.L711
.L713:
	ldrh	r7, [r1], #2
	cmp	r7, r0
	beq	.L712
	add	r3, r3, #1
	uxth	r3, r3
.L711:
	cmp	r3, ip
	bne	.L713
	mov	r3, r3, asl #1
	strh	r0, [r2, r3]	@ movhi
	ldr	r2, .L714
	ldr	r3, .L714+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L712:
	ldr	r3, .L714
	mov	r0, #12
	ldr	r2, .L714+8
	ldrh	r1, [r3, r2]
	mul	r1, r0, r1
	ldr	r0, [r3, #-1140]
	add	ip, r0, r1
	stmib	ip, {r5, r6}
	str	r4, [r0, r1]
	ldrh	r1, [r3, r2]
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L715:
	.align	2
.L714:
	.word	.LANCHOR2
	.word	-1148
	.word	-1136
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshOpenBlock
	.type	FtlGcRefreshOpenBlock, %function
FtlGcRefreshOpenBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	mov	r1, r4
	ldr	r0, .L721
	bl	printk
	ldr	r8, .L721+4
	ldr	r3, .L721+8
	ldrh	sl, [r3, r8]
	cmp	sl, r4
	beq	.L717
	ldr	r6, .L721+12
	ldrh	r7, [r3, r6]
	cmp	r7, r4
	beq	.L717
	ldr	ip, .L721+16
	ldrh	r5, [r3, ip]
	cmp	r5, r4
	beq	.L717
	ldr	r1, .L721+20
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	beq	.L717
	movw	r2, #65535
	cmp	sl, r2
	streqh	r4, [r3, r8]	@ movhi
	beq	.L717
	cmp	r7, r2
	streqh	r4, [r3, r6]	@ movhi
	beq	.L717
	cmp	r5, r2
	streqh	r4, [r3, ip]	@ movhi
	beq	.L717
	cmp	r0, r2
	streqh	r4, [r3, r1]	@ movhi
.L717:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L722:
	.align	2
.L721:
	.word	.LC80
	.word	-1134
	.word	.LANCHOR2
	.word	-1132
	.word	-1130
	.word	-1128
	.fnend
	.size	FtlGcRefreshOpenBlock, .-FtlGcRefreshOpenBlock
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	mov	r1, r4
	ldr	r0, .L734
	bl	printk
	ldr	r8, .L734+4
	ldr	r3, .L734+8
	ldrh	sl, [r3, r8]
	cmp	sl, r4
	beq	.L731
	ldr	r6, .L734+12
	ldrh	r7, [r3, r6]
	cmp	r7, r4
	beq	.L731
	ldr	ip, .L734+16
	ldrh	r5, [r3, ip]
	cmp	r5, r4
	beq	.L731
	ldr	r1, .L734+20
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	beq	.L732
	movw	r2, #65535
	cmp	sl, r2
	streqh	r4, [r3, r8]	@ movhi
	beq	.L731
	cmp	r7, r2
	streqh	r4, [r3, r6]	@ movhi
	beq	.L731
	cmp	r5, r2
	streqh	r4, [r3, ip]	@ movhi
	beq	.L731
	cmp	r0, r2
	streqh	r4, [r3, r1]	@ movhi
	mvnne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L731:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L732:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L735:
	.align	2
.L734:
	.word	.LC80
	.word	-1134
	.word	.LANCHOR2
	.word	-1132
	.word	-1130
	.word	-1128
	.fnend
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.section	.text.unlikely,"ax",%progbits
	.align	2
	.type	decrement_vpc_count.part.20, %function
decrement_vpc_count.part.20:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0, asl #1
	ldr	r5, .L738
	mov	r4, r0
	mov	r1, r4
	ldr	r0, .L738+4
	ldr	r3, [r5, #-2012]
	ldrh	r2, [r3, r6]
	bl	printk
	ldr	r3, [r5, #-2012]
	mov	r2, #32
	sub	r0, r5, #2000
	mov	r1, r4
	strh	r2, [r3, r6]	@ movhi
	bl	test_node_in_list
	cmp	r0, #0
	beq	.L737
	mov	r1, r4
	sub	r0, r5, #2000
	bl	List_remove_node
	ldr	r3, .L738+8
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r5, #-2012]
	ldr	r0, .L738+12
	mov	r1, r4
	ldrh	r2, [r3, r6]
	bl	printk
.L737:
	mov	r0, r4
	bl	FtlGcRefreshBlock
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L739:
	.align	2
.L738:
	.word	.LANCHOR2
	.word	.LC81
	.word	-1996
	.word	.LC82
	.fnend
	.size	decrement_vpc_count.part.20, .-decrement_vpc_count.part.20
	.text
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
	ldr	r7, .L745
	mov	r2, r4
	ldr	r6, .L745+4
	ldrh	r1, [r7, r6]
	mov	r5, r0
	ldr	r0, .L745+8
	bl	printk
	mov	r0, r5
	bl	FtlGcRefreshBlock
	ldr	r3, .L745+12
	ldr	r2, [r3, #3844]
	cmp	r2, #0
	beq	.L741
	ldr	r3, [r3, #4052]
	mov	r5, r5, asl #1
	ldrh	r2, [r3, r5]
	cmp	r2, #39
	subhi	r2, r2, #40
	strhih	r2, [r3, r5]	@ movhi
.L741:
	ldrh	r1, [r7, r6]
	mov	r3, #0
	ldr	r2, .L745+16
	b	.L742
.L744:
	ldrh	r0, [r2, #2]!
	cmp	r0, r4
	beq	.L743
	add	r3, r3, #1
	uxth	r3, r3
.L742:
	cmp	r3, r1
	bne	.L744
	cmp	r3, #15
	bhi	.L743
	ldr	r2, .L745
	ldr	r1, .L745+20
	add	r0, r2, r3, asl #1
	add	r3, r3, #1
	strh	r4, [r0, r1]	@ movhi
	sub	r1, r1, #2
	strh	r3, [r2, r1]	@ movhi
.L743:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L746:
	.align	2
.L745:
	.word	.LANCHOR2
	.word	-1126
	.word	.LC83
	.word	.LANCHOR0
	.word	.LANCHOR2-1126
	.word	-1124
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
	ldr	r2, .L750
	ldr	r3, .L750+4
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L748
	ldr	r1, .L750+8
	ldrh	r0, [r3, r1]
	movw	r1, #65535
	cmp	r0, r1
	bne	.L748
	ldr	r4, .L750+12
	ldr	r5, .L750+4
	ldrh	r1, [r3, r4]
	cmp	r1, r2
	movcs	r2, #0
	strcsh	r2, [r3, r4]	@ movhi
	ldrh	r2, [r5, r4]
	ldr	r3, .L750+16
	add	r2, r5, r2, asl #1
	ldrh	r0, [r2, r3]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r5, r4]
	add	r3, r3, #1
	strh	r3, [r5, r4]	@ movhi
.L748:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L751:
	.align	2
.L750:
	.word	-1126
	.word	.LANCHOR2
	.word	-1134
	.word	-1090
	.word	-1124
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
	beq	.L753
	bl	memset
.L753:
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
	ldr	r4, .L755
	movw	r5, #3922
	ldr	r2, .L755+4
	mov	r1, #255
	ldr	r6, .L755+8
	ldr	r0, [r4, #-1144]
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #12
	strh	r3, [r4, r2]	@ movhi
	ldrh	r2, [r6, r5]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r6, r5]
	mov	r2, #12
	ldr	r0, [r4, #-1140]
	mov	r1, #255
	mul	r2, r2, r3
	bl	ftl_memset
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlGcBufInit
.L756:
	.align	2
.L755:
	.word	.LANCHOR2
	.word	-1148
	.word	.LANCHOR0
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
	movw	r3, #3862
	ldr	r7, .L767
	mov	r2, #6
	ldr	r4, .L767+4
	mov	r1, #0
	mov	r6, #0
	ldrh	r3, [r7, r3]
	mov	sl, r6
	ldr	r0, [r4, #-2020]
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r3, .L767+8
	str	r6, [r4, #-2000]
	str	r6, [r4, #-2016]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #8
	str	r6, [r4, #-2008]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #908
	strh	r6, [r4, r3]	@ movhi
	mov	r4, r6
	b	.L758
.L760:
	add	r1, r7, r1
	add	r9, r9, #1
	ldrb	r0, [r1, #3878]	@ zero_extendqisi2
	mov	r1, r3
	stmia	sp, {r2, r3}
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	ldmia	sp, {r2, r3}
	uxth	r9, r9
	cmp	r0, #0
	ldreqh	r1, [r7, r2]
	addeq	r5, r5, r1
	uxtheq	r5, r5
	b	.L765
.L766:
	mov	r5, #0
	uxth	r3, r4
	mov	r9, r5
	movw	fp, #3852
	movw	r2, #3918
.L765:
	ldrh	r0, [r7, fp]
	sxth	r1, r9
	cmp	r1, r0
	blt	.L760
	cmp	r5, #0
	ldr	r9, .L767+4
	beq	.L761
	sxth	r1, r5
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r5, r0
	b	.L762
.L761:
	sxth	r3, r4
	ldr	r2, [r9, #-2012]
	mvn	r1, #0
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
.L762:
	sxth	r1, r4
	ldr	r0, [r9, #-2020]
	ldr	r3, .L767+4
	mov	r2, r1, asl #1
	add	r1, r2, r1
	add	r1, r0, r1, asl #1
	strh	r5, [r1, #4]	@ movhi
	ldr	r1, .L767+12
	ldrh	r1, [r9, r1]
	cmp	r8, r1
	beq	.L763
	ldr	r1, .L767+16
	ldrh	r1, [r3, r1]
	cmp	r8, r1
	beq	.L763
	ldr	r1, .L767+20
	ldrh	r1, [r3, r1]
	cmp	r8, r1
	beq	.L763
	ldr	r3, [r3, #-2012]
	uxth	r0, r4
	ldrh	r3, [r3, r2]
	cmp	r3, #0
	bne	.L764
	add	r6, r6, #1
	uxth	r6, r6
	bl	INSERT_FREE_LIST
	b	.L763
.L764:
	add	sl, sl, #1
	uxth	sl, sl
	bl	INSERT_DATA_LIST
.L763:
	add	r4, r4, #1
	uxth	r4, r4
.L758:
	movw	r2, #3860
	sxth	r8, r4
	ldrh	r3, [r7, r2]
	cmp	r8, r3
	blt	.L766
	ldr	r3, .L767+4
	mov	r0, #0
	ldr	r2, .L767+8
	strh	sl, [r3, r2]	@ movhi
	add	r2, r2, #8
	strh	r6, [r3, r2]	@ movhi
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L768:
	.align	2
.L767:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-2004
	.word	-1992
	.word	-1944
	.word	-1896
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
	ldr	r4, .L772
	movw	r6, #3928
	ldr	r5, .L772+4
	movw	r7, #3958
	mov	r8, #12
	ldr	r2, [r4, #3948]
	ldr	r0, [r5, #-1084]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r3, [r4, r6]
	ldrh	r2, [r4, r7]
	mov	r1, #255
	ldr	r0, [r5, #-1080]
	mul	r2, r2, r3
	bl	ftl_memset
	mov	r2, #0
	mov	ip, r2
	mvn	r0, #0
	b	.L770
.L771:
	mul	r3, r8, r2
	ldr	r1, [r5, #-1848]
	add	sl, r1, r3
	str	ip, [sl, #4]
	strh	r0, [r1, r3]	@ movhi
	ldr	r1, [r5, #-1848]
	ldr	sl, [r5, #-1080]
	add	r3, r1, r3
	ldrh	r1, [r4, r6]
	mul	r1, r2, r1
	add	r2, r2, #1
	uxth	r2, r2
	bic	r1, r1, #3
	add	r1, sl, r1
	str	r1, [r3, #8]
.L770:
	ldrh	r3, [r4, r7]
	ldr	r1, .L772
	cmp	r3, r2
	ldr	r3, .L772+4
	bhi	.L771
	ldr	r0, .L772+8
	mvn	r2, #0
	strh	r2, [r3, r0]	@ movhi
	sub	r0, r0, #2
	strh	r2, [r3, r0]	@ movhi
	ldr	r0, [r1, #3948]
	ldr	r2, .L772+12
	strh	r0, [r3, r2]	@ movhi
	ldr	r0, .L772+16
	ldr	r2, .L772+20
	strh	r0, [r3, r2]	@ movhi
	add	r2, r2, #40
	ldrh	r0, [r3, r2]
	sub	r2, r2, #36
	strh	r0, [r3, r2]	@ movhi
	movw	r2, #3956
	ldrh	r1, [r1, r2]
	ldr	r2, .L772+24
	strh	r1, [r3, r2]	@ movhi
	ldr	r2, [r3, #-1028]
	str	r2, [r3, #-1064]
	ldr	r2, [r3, #-1024]
	str	r2, [r3, #-1060]
	ldr	r2, [r3, #-1084]
	str	r2, [r3, #-1056]
	ldr	r2, [r3, #-1020]
	str	r2, [r3, #-1052]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L773:
	.align	2
.L772:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1074
	.word	-1066
	.word	-3902
	.word	-1072
	.word	-1070
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r1, #0
	ldrh	r2, [r0, #10]
	mov	r4, r0
	ldr	r5, [r0, #20]
	ldr	r6, [r0, #12]
	ldr	r7, [r0, #24]
	mov	r2, r2, asl #1
	mov	r0, r5
	bl	ftl_memset
	mov	r3, #0
	b	.L775
.L779:
	ldr	r0, [r7, r3, asl #2]
	mov	r2, #0
	ubfx	r0, r0, #10, #16
	b	.L776
.L778:
	mov	r1, r2, asl #1
	ldrh	ip, [r6, r1]
	cmp	ip, r0
	bne	.L777
	cmp	r0, #0
	ldrneh	ip, [r5, r1]
	addne	ip, ip, #1
	strneh	ip, [r5, r1]	@ movhi
.L777:
	add	r2, r2, #1
	uxth	r2, r2
.L776:
	ldrh	r1, [r4, #10]
	cmp	r1, r2
	bhi	.L778
	add	r3, r3, #1
	uxth	r3, r3
.L775:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L779
	ldr	r2, .L785
	mov	r3, #3920
	mov	r8, #0
	mov	r7, r8
	ldrh	r2, [r2, r3]
	ldrh	r3, [r4, #0]
	mov	r3, r3, asl #1
	strh	r2, [r5, r3]	@ movhi
	ldrh	fp, [r5, #0]
	b	.L780
.L783:
	mov	sl, r7, asl #1
	ldrh	r9, [r5, sl]
	cmp	fp, r9
	bls	.L781
	ldrh	r3, [r6, sl]
	cmp	r3, #0
	movne	r8, r7
	movne	fp, r9
.L781:
	cmp	r9, #0
	bne	.L782
	ldrh	r0, [r6, sl]
	cmp	r0, #0
	beq	.L782
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	strh	r9, [r6, sl]	@ movhi
	ldrh	r3, [r4, #8]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L782:
	add	r7, r7, #1
	uxth	r7, r7
.L780:
	ldrh	r3, [r4, #10]
	cmp	r3, r7
	bhi	.L783
	mov	r0, r8
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L786:
	.align	2
.L785:
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
	ldr	r3, .L788
	movw	r2, #4062
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, #0
	strh	r4, [r3, r2]	@ movhi
	mov	r2, #4064
	strh	r4, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, r4
	strh	r4, [r3, r2]	@ movhi
	movw	r2, #4060
	strh	r0, [r3, r2]	@ movhi
	mov	r2, #2048
	ldr	r0, .L788+4
	bl	ftl_memset
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L789:
	.align	2
.L788:
	.word	.LANCHOR0
	.word	.LANCHOR0+4068
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
	ldr	r3, .L791
	mov	r2, #3984
	mvn	r1, #0
	ldr	r0, .L791+4
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #6
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #255
	mov	r2, #16
	b	ftl_memset
.L792:
	.align	2
.L791:
	.word	.LANCHOR0
	.word	.LANCHOR0+3996
	.fnend
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L797
	ldr	r2, .L797+4
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
.L795:
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
	bne	.L795
	ldmfd	sp!, {r4, r5, r6, pc}
.L798:
	.align	2
.L797:
	.word	-1016
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
	ldr	r6, .L800
	mov	r4, #0
	ldr	r2, .L800+4
	mov	r1, r4
	ldr	r5, .L800+8
	movw	r7, #3862
	str	r3, [r6, #-996]
	strh	r3, [r6, r2]	@ movhi
	movw	r3, #3966
	strh	r4, [r5, r3]	@ movhi
	movw	r3, #3940
	ldrh	r2, [r5, r3]
	ldr	r0, [r5, #3968]
	str	r4, [r6, #-1012]
	mov	r2, r2, asl #1
	str	r4, [r6, #-1004]
	str	r4, [r6, #-1000]
	str	r4, [r5, #3844]
	bl	ftl_memset
	ldrh	r2, [r5, r7]
	mov	r1, r4
	ldr	r0, [r5, #4052]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r5, r7]
	mov	r1, r4
	ldr	r0, [r6, #-992]
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #48
	ldr	r0, .L800+12
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #512
	ldr	r0, .L800+16
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L801:
	.align	2
.L800:
	.word	.LANCHOR2
	.word	-1008
	.word	.LANCHOR0
	.word	.LANCHOR2-2068
	.word	.LANCHOR2-1700
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
	mov	r6, #0
	ldr	r4, .L833
	mov	r0, #1024
	ldr	r3, .L833+4
	mov	r7, #12
	ldr	r2, .L833+8
	movw	sl, #3852
	ldr	r5, .L833+12
	mov	r8, #36
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #65535
	str	r3, [r4, #-980]
	mvn	r3, #0
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #2
	str	r6, [r4, #-988]
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #2
	str	r6, [r4, #-1704]
	strh	r3, [r4, r2]	@ movhi
	add	r2, r2, #2
	str	r6, [r4, #-1800]
	strh	r3, [r4, r2]	@ movhi
	mov	r2, #32
	ldr	r3, .L833+16
	str	r6, [r4, #-1796]
	str	r6, [r4, #-1812]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	mov	r2, #128
	str	r6, [r4, #-1824]
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #6
	str	r6, [r4, #-1828]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #54
	str	r6, [r4, #-1820]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #158
	str	r6, [r4, #-1816]
	strh	r6, [r4, r3]	@ movhi
	sub	r3, r3, #122
	str	r6, [r4, #-1832]
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #3924
	ldrh	r1, [r5, r3]
	str	r6, [r4, #-1792]
	str	r6, [r4, #-1788]
	str	r6, [r5, #4056]
	str	r6, [r4, #-1780]
	str	r6, [r4, #-1776]
	str	r6, [r4, #-984]
	str	r6, [r4, #-1176]
	str	r6, [r4, #-976]
	str	r6, [r4, #-1184]
	str	r6, [r4, #-972]
	bl	__aeabi_idiv
	movw	r3, #3852
	ldrh	r3, [r5, r3]
	str	r6, [r5, #3972]
	movw	r6, #3922
	mov	r3, r3, asl #2
	cmp	r0, r3
	str	r0, [r4, #-964]
	ldrh	r0, [r5, r6]
	strhi	r3, [r4, #-964]
	ldr	r4, .L833
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-1144]
	ldrh	r0, [r5, r6]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r6, [r5, sl]
	mul	r6, r8, r6
	mov	r9, r6, asl #3
	str	r0, [r4, #-1140]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-960]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-956]
	mov	r0, r9
	bl	ftl_malloc
	str	r0, [r4, #-952]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r5, #4048]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-1156]
	ldr	r0, [r4, #-964]
	mul	r0, r8, r0
	bl	ftl_malloc
	movw	r8, #3928
	ldrh	r3, [r5, sl]
	ldrh	r6, [r5, r8]
	mov	r3, r3, asl #1
	add	r3, r3, #1
	str	r3, [r4, #-1152]
	str	r0, [r5, #3976]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-948]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-944]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-940]
	ldr	r0, [r4, #-1152]
	mul	r0, r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-1164]
	ldr	r0, [r4, #-964]
	mul	r0, r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-936]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #-932]
	mov	r0, r6
	bl	ftl_malloc
	movw	r6, #3930
	str	r0, [r4, #-928]
	ldr	r0, [r4, #-1152]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	ldrh	sl, [r5, sl]
	mul	sl, sl, r3
	str	r0, [r4, #-1168]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-924]
	mov	r0, sl, asl #3
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	ldr	sl, .L833+20
	str	r0, [r4, #-920]
	ldr	r0, [r4, #-1152]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	movw	r6, #3862
	str	r0, [r4, #-1160]
	ldr	r0, [r4, #-964]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #-916]
	ldrh	r0, [r5, r6]
	mov	r0, r0, asl #1
	uxth	r0, r0
	strh	r0, [r4, sl]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #-992]
	ldrh	r0, [r4, sl]
	add	r0, r0, #544
	add	r0, r0, #3
	mov	r0, r0, lsr #9
	strh	r0, [r4, sl]	@ movhi
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldrh	sl, [r5, r6]
	mov	sl, sl, asl #1
	str	r0, [r4, #-908]
	add	r0, r0, #32
	str	r0, [r5, #4052]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-904]
	mov	r0, sl
	bl	ftl_malloc
	ldr	sl, [r5, #3948]
	mov	sl, sl, asl #1
	str	r0, [r4, #-2012]
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-1028]
	mov	r0, sl
	bl	ftl_malloc
	movw	sl, #3940
	str	r0, [r4, #-1084]
	ldrh	r0, [r5, r6]
	mov	r0, r0, lsr #3
	add	r0, r0, #4
	bl	ftl_malloc
	str	r0, [r4, #-1836]
	ldrh	r0, [r5, sl]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r5, #3968]
	ldrh	r0, [r5, sl]
	mov	r0, r0, asl #1
	bl	ftl_malloc
	str	r0, [r4, #-900]
	ldrh	r0, [r5, sl]
	add	sl, sl, #2
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-896]
	ldrh	r0, [r5, sl]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r2, [r5, sl]
	mov	r1, #0
	mov	r2, r2, asl #2
	str	r0, [r4, #-892]
	bl	ftl_memset
	movw	r3, #3956
	ldrh	sl, [r5, r3]
	mov	sl, sl, asl #2
	mov	r0, sl
	bl	ftl_malloc
	str	r0, [r4, #-1020]
	mov	r0, sl
	bl	ftl_malloc
	movw	sl, #3958
	str	r0, [r4, #-888]
	ldr	r0, [r5, #3948]
	mov	r0, r0, asl #2
	bl	ftl_malloc
	str	r0, [r4, #-1024]
	ldrh	r0, [r5, sl]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r5, sl]
	movw	r7, #3874
	str	r0, [r4, #-1848]
	ldrh	r0, [r5, r8]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrh	r3, [r5, r6]
	ldr	r6, .L833+24
	str	r0, [r4, #-1080]
	mov	r0, #6
	mul	r0, r0, r3
	bl	ftl_malloc
	movw	r3, #3916
	ldrh	r3, [r5, r3]
	ldrh	r2, [r5, r7]
	add	r3, r3, #31
	mov	r3, r3, lsr #5
	strh	r3, [r4, r6]	@ movhi
	mul	r3, r2, r3
	str	r0, [r4, #-2020]
	mov	r0, r3, asl #2
	bl	ftl_malloc
	ldrh	r1, [r4, r6]
	mov	r3, #1
	ldr	ip, .L833+12
	mov	r1, r1, asl #2
	mov	r2, r1
	str	r0, [r5, #4012]
	ldrh	r5, [r5, r7]
	ldr	r0, .L833+28
	b	.L804
.L805:
	ldr	r4, [ip, #4012]
	add	r3, r3, #1
	add	r4, r4, r2
	add	r2, r2, r1
	str	r4, [r0, #4]!
.L804:
	cmp	r3, r5
	bcc	.L805
	ldr	r0, .L833+32
	mov	r2, #0
	mov	r1, r2
	add	r0, r0, r3, asl #2
	b	.L806
.L807:
	add	ip, r0, r2
	add	r3, r3, #1
	add	r2, r2, #4
	str	r1, [ip, #28]
.L806:
	cmp	r3, #7
	bls	.L807
	ldr	r3, .L833
	ldr	r2, [r3, #-1028]
	cmp	r2, #0
	beq	.L832
.L808:
	ldr	r2, [r3, #-1084]
	cmp	r2, #0
	beq	.L832
.L810:
	ldr	r2, [r3, #-1020]
	cmp	r2, #0
	beq	.L832
.L811:
	ldr	r2, [r3, #-1024]
	cmp	r2, #0
	beq	.L832
.L812:
	ldr	r2, [r3, #-1848]
	cmp	r2, #0
	beq	.L832
.L813:
	ldr	r2, [r3, #-1080]
	cmp	r2, #0
	beq	.L832
.L814:
	ldr	r2, [r3, #-2020]
	cmp	r2, #0
	beq	.L832
.L815:
	ldr	r2, .L833+12
	ldr	r2, [r2, #4012]
	cmp	r2, #0
	beq	.L832
.L816:
	ldr	r3, [r3, #-2012]
	cmp	r3, #0
	beq	.L832
.L817:
	ldr	r3, .L833
	ldr	r2, [r3, #-1144]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-1140]
	cmp	r2, #0
	beq	.L832
.L819:
	ldr	r2, [r3, #-960]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-952]
	cmp	r2, #0
	beq	.L832
	ldr	r2, .L833+12
	ldr	r2, [r2, #4048]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-1156]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-956]
	cmp	r2, #0
	beq	.L832
.L821:
	ldr	r2, [r3, #-948]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-944]
	cmp	r2, #0
	beq	.L832
	ldr	r3, [r3, #-940]
	cmp	r3, #0
	beq	.L832
	ldr	r3, .L833
	ldr	r2, [r3, #-1164]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-932]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-928]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-1168]
	cmp	r2, #0
	beq	.L832
.L823:
	ldr	r2, [r3, #-924]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-920]
	cmp	r2, #0
	beq	.L832
	ldr	r2, [r3, #-1160]
	cmp	r2, #0
	beq	.L832
.L825:
	ldr	r2, .L833+12
	ldr	r1, [r2, #4052]
	cmp	r1, #0
	beq	.L832
	ldr	r3, [r3, #-992]
	cmp	r3, #0
	beq	.L832
.L827:
	ldr	r3, [r2, #3968]
	cmp	r3, #0
	beq	.L832
.L828:
	ldr	r3, .L833
	ldr	r2, [r3, #-900]
	cmp	r2, #0
	beq	.L832
.L829:
	ldr	r2, [r3, #-896]
	cmp	r2, #0
	beq	.L832
.L830:
	ldr	r3, [r3, #-892]
	cmp	r3, #0
	bne	.L831
.L832:
	ldr	r0, .L833+36
	ldr	r1, .L833+40
	bl	printk
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L831:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L834:
	.align	2
.L833:
	.word	.LANCHOR2
	.word	-1088
	.word	-1134
	.word	.LANCHOR0
	.word	-1188
	.word	-912
	.word	-1016
	.word	.LANCHOR0+4012
	.word	.LANCHOR0+3984
	.word	.LC84
	.word	.LANCHOR3
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	ftl_read_flash_info
	.type	ftl_read_flash_info, %function
ftl_read_flash_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r1, #0
	mov	r2, #11
	mov	r4, r0
	bl	ftl_memset
	ldr	r3, .L838
	ldr	r2, [r3, #3624]
	ldr	r1, [r3, #856]
	ldrb	r2, [r2, #9]	@ zero_extendqisi2
	mul	r2, r1, r2
	mov	r1, #0
	uxth	r2, r2
	strb	r2, [r4, #4]
	mov	r2, r2, lsr #8
	strb	r2, [r4, #5]
	ldrb	r2, [r3, #3840]	@ zero_extendqisi2
	strb	r2, [r4, #7]
	ldr	r2, [r3, #3960]
	ubfx	r0, r2, #8, #8
	strb	r0, [r4, #1]
	strb	r2, [r4, #0]
	ubfx	r0, r2, #16, #8
	mov	r2, r2, lsr #24
	strb	r0, [r4, #2]
	strb	r2, [r4, #3]
	ldr	r2, [r3, #3624]
	ldrb	r0, [r2, #9]	@ zero_extendqisi2
	strb	r0, [r4, #6]
	mov	r0, #32
	strb	r0, [r4, #8]
	ldrb	r2, [r2, #7]	@ zero_extendqisi2
	ldrb	r0, [r3, #3762]	@ zero_extendqisi2
	mov	r3, r1
	strb	r1, [r4, #10]
	strb	r2, [r4, #9]
	mov	r2, #1
	ldr	r1, .L838+4
	b	.L836
.L837:
	ldrb	r5, [r3, r1]	@ zero_extendqisi2
	add	r3, r3, #1
	ldrb	ip, [r4, #10]	@ zero_extendqisi2
	orr	ip, ip, r2, asl r5
	strb	ip, [r4, #10]
.L836:
	uxtb	ip, r3
	cmp	ip, r0
	bcc	.L837
	ldmfd	sp!, {r3, r4, r5, pc}
.L839:
	.align	2
.L838:
	.word	.LANCHOR0
	.word	.LANCHOR0+3764
	.fnend
	.size	ftl_read_flash_info, .-ftl_read_flash_info
	.align	2
	.global	FlashDieInfoInit
	.type	FlashDieInfoInit, %function
FlashDieInfoInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, #0
	ldr	r3, .L849
	mov	r6, r4
	ldr	r2, .L849+4
	ldr	r5, .L849+8
	strb	r4, [r3, #-884]
	movw	r3, #3126
	ldrh	r0, [r2, r3]
	add	r7, r5, #2976
	strb	r4, [r5, #3762]
	bl	FlashBlockAlignInit
	mov	r1, r4
	mov	r2, #8
	ldr	r0, .L849+12
	mov	r8, r7
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #32
	ldr	r0, .L849+16
	bl	ftl_memset
	ldr	r0, .L849+20
	mov	r1, r4
	mov	r2, #128
	bl	ftl_memset
.L842:
	ldr	r4, [r5, #3624]
	mov	r1, r8
	add	r0, r4, #1
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L841
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	add	r2, r5, r3, asl #2
	str	r0, [r2, #3588]
	add	r2, r5, r3
	add	r3, r3, #1
	strb	r3, [r5, #3762]
	strb	r6, [r2, #3764]
.L841:
	add	r6, r6, #1
	add	r8, r8, #8
	cmp	r6, #4
	bne	.L842
	ldr	r5, .L849+8
	ldr	r3, .L849
	ldrb	r2, [r5, #3762]	@ zero_extendqisi2
	strb	r2, [r3, #-884]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #2
	bne	.L843
	add	sl, r4, #1
	mov	r6, #0
	mov	r8, r5
.L846:
	mov	r0, sl
	mov	r1, r7
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L844
	ldrb	r1, [r4, #13]	@ zero_extendqisi2
	ldr	r0, [r5, #856]
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	mul	r0, r0, r1
	ldrh	r1, [r4, #14]
	add	r2, r5, r3, asl #2
	and	r1, r1, #65280
	mul	r1, r1, r0
	str	r1, [r2, #3588]
	ldrb	r0, [r4, #23]	@ zero_extendqisi2
	cmp	r0, #0
	movne	r1, r1, asl #1
	strne	r1, [r2, #3588]
	add	r2, r8, r3
	add	r3, r3, #1
	strb	r3, [r8, #3762]
	strb	r6, [r2, #3764]
.L844:
	add	r6, r6, #1
	add	r7, r7, #8
	cmp	r6, #4
	bne	.L846
.L843:
	ldr	r3, .L849+8
	ldrb	r1, [r4, #13]	@ zero_extendqisi2
	ldr	r2, .L849
	ldrb	r3, [r3, #3762]	@ zero_extendqisi2
	mul	r1, r1, r3
	ldrh	r3, [r4, #14]
	mul	r1, r3, r1
	ldr	r3, .L849+24
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L850:
	.align	2
.L849:
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR0+3764
	.word	.LANCHOR0+3588
	.word	.LANCHOR0+3628
	.word	-882
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
	bne	.L852
	ldr	r3, .L894
.L853:
	strh	r0, [r3, #2]!	@ movhi
	add	r0, r0, #1
	uxth	r0, r0
	cmp	r0, #256
	bne	.L853
	b	.L854
.L852:
	cmp	r0, #1
	bne	.L855
	ldr	r0, .L894+4
	mov	r2, #0
	mov	r3, r2
.L858:
	uxth	r1, r3
	cmp	r1, #3
	bls	.L856
	tst	r3, #1
	moveq	r1, #2
	movne	r1, #3
	rsb	r1, r1, r2
	uxth	r1, r1
.L856:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L858
	b	.L854
.L855:
	cmp	r0, #2
	bne	.L859
	mov	r3, #0
	ldr	r0, .L894
	movw	r2, #65535
	mov	r1, r3
	b	.L893
.L862:
	cmp	r3, #1
	movls	r1, r3
	movhi	r1, r2
.L893:
	add	r3, r3, #1
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r3, r3
	uxth	r2, r2
	cmp	r3, #256
	bne	.L862
	b	.L854
.L859:
	cmp	r0, #3
	bne	.L863
	ldr	r0, .L894+4
	mov	r2, #0
	mov	r3, r2
.L866:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L864
	tst	r3, #1
	moveq	r1, #4
	movne	r1, #5
	rsb	r1, r1, r2
	uxth	r1, r1
.L864:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L866
	b	.L854
.L863:
	cmp	r0, #4
	mov	r2, #0
	bne	.L867
	ldr	r3, .L894+8
	movw	r1, #3076
	strh	r2, [r3, r1]	@ movhi
	movw	r2, #3078
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
	ldr	r0, .L894+12
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #3088
	mov	r1, #7
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #8
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #16
	mov	r3, r1
.L869:
	tst	r3, #1
	add	r3, r3, #1
	moveq	r1, #6
	movne	r1, #7
	cmp	r3, #256
	rsb	r1, r1, r2
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L869
	b	.L854
.L867:
	cmp	r0, #5
	bne	.L870
	ldr	r1, .L894
	mov	r3, r2
.L871:
	strh	r3, [r1, #2]!	@ movhi
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #16
	bne	.L871
	ldr	r2, .L894+16
.L872:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, #496
	bne	.L872
	b	.L854
.L870:
	cmp	r0, #6
	bne	.L873
	ldr	r0, .L894
	mov	r3, r2
.L876:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L874
	tst	r3, #1
	moveq	r1, #10
	movne	r1, #12
	rsb	r1, r1, r2
	uxth	r1, r1
.L874:
	add	r3, r3, #1
	add	r2, r2, #3
	cmp	r3, #256
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L876
	b	.L854
.L873:
	cmp	r0, #9
	bne	.L854
	ldr	r3, .L894+8
	movw	r1, #3076
	strh	r2, [r3, r1]	@ movhi
	movw	r2, #3078
	mov	r1, #1
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #2
	add	r2, r2, r1
	strh	r1, [r3, r2]	@ movhi
	mov	r3, #3
	ldr	r2, .L894+20
	movw	r1, #509
.L877:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, r1
	bne	.L877
.L854:
	mov	r2, #1024
	ldr	r0, .L894+24
	mov	r1, #255
	uxth	r4, r4
	bl	ftl_memset
	ldr	r2, .L894
	mov	r3, #0
	ldr	r0, .L894+28
	b	.L878
.L879:
	ldrh	r1, [r2, #2]!
	add	r3, r3, #1
	uxth	r3, r3
	add	ip, r0, r1, asl #1
	sub	ip, ip, #880
	strh	r1, [ip, #0]	@ movhi
.L878:
	cmp	r3, r4
	bcc	.L879
	ldmfd	sp!, {r4, pc}
.L895:
	.align	2
.L894:
	.word	.LANCHOR0+3074
	.word	.LANCHOR0+3076
	.word	.LANCHOR0
	.word	.LANCHOR0+3090
	.word	.LANCHOR0+3106
	.word	.LANCHOR0+3080
	.word	.LANCHOR2-880
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
	.global	ftl_memcpy32
	.type	ftl_memcpy32, %function
ftl_memcpy32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	r3, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	ip, r3
	b	.L898
.L899:
	ldr	r4, [r1, r3]
	add	ip, ip, #1
	str	r4, [r0, r3]
	add	r3, r3, #4
.L898:
	cmp	ip, r2
	bne	.L899
	ldmfd	sp!, {r4, pc}
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
	ldr	r3, .L902
	ldr	r0, [r3, #0]
	b	jiffies_to_msecs
.L903:
	.align	2
.L902:
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
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	cmp	r2, #0
	ldr	r4, .L907
	mov	ip, r0
	mov	r2, r3
	ldr	r4, [r4, #144]
	add	r4, r4, #4096
	add	r1, r4, r1
	movne	r0, r1
	movne	r1, ip
	ldmfd	sp!, {r4, lr}
	b	memcpy
.L908:
	.align	2
.L907:
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
	ldr	r3, .L913
	ldr	r0, [r3, #3960]
	bx	lr
.L914:
	.align	2
.L913:
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
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r5, #0
	mov	r8, r0
	mov	r6, r1
	mov	r7, r2
	mov	sl, r3
	mov	r4, r5
	b	.L919
.L925:
	cmp	r5, #0
	bne	.L920
	ldr	r0, .L927
	mov	r1, r8
	mov	r2, r6
	mov	r3, r4
	bl	printk
.L920:
	cmp	r7, #4
	ldreq	r0, .L927+4
	ldreq	r1, [r6, r4, asl #2]
	beq	.L926
	cmp	r7, #2
	moveq	r3, r4, asl #1
	ldreq	r0, .L927+4
	ldreqsh	r1, [r6, r3]
	ldrne	r0, .L927+4
	ldrneb	r1, [r6, r4]	@ zero_extendqisi2
.L926:
	add	r5, r5, #1
	bl	printk
	cmp	r5, #15
	bls	.L924
	ldr	r0, .L927+8
	mov	r5, #0
	ldr	r1, .L927+12
	bl	printk
.L924:
	add	r4, r4, #1
.L919:
	cmp	r4, sl
	bne	.L925
	ldr	r0, .L927+8
	ldr	r1, .L927+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	b	printk
.L928:
	.align	2
.L927:
	.word	.LC85
	.word	.LC86
	.word	.LC77
	.word	.LC87
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
	ldr	r6, .L952
	add	r0, r6, r0, asl #3
	ldr	r3, [r6, #3788]
	ldr	r4, [r0, #864]
	cmp	r3, #3
	bls	.L946
	ldr	r3, [r4, #16]
	tst	r3, #4
	beq	.L946
	mov	r0, r4
	bl	wait_for_nandc_xfer_completed
	ldr	r5, [r4, #16]
	ldr	r3, [r4, #8]
	ands	r5, r5, #2
	str	r3, [sp, #0]
	movne	r5, #0
	bne	.L950
	b	.L951
.L936:
	ldr	r3, [r6, #3788]
	cmp	r3, #5
	bls	.L933
	ldr	r3, [r4, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L933
	ldr	r3, [sp, #4]
	tst	r3, #131072
	bne	.L934
.L933:
	add	r5, r5, #1
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L950
	ldr	r2, [r4, #28]
	mov	r1, r5
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L952+4
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L952+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L950:
	ldr	r2, [r4, #28]
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	blt	.L936
.L934:
	ldr	r4, .L952
	ldr	r3, [r4, #3824]
	cmp	r3, #0
	beq	.L937
	ldr	r1, [sp, #0]
	mov	r2, #0
	ldr	r0, [r4, #3816]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r1, [sp, #0]
	ldr	r0, [r4, #3820]
	mov	r2, #0
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
	b	.L937
.L940:
	ldr	r3, [r4, #8]
	add	r5, r5, #1
	str	r3, [sp, #0]
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L951
	ldr	r2, [sp, #0]
	mov	r1, r5
	ldr	r3, [r4, #28]
	ldr	r0, .L952+12
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L952+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L951:
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L940
	ldr	r6, .L952
	ldr	r3, [r6, #3832]
	cmp	r3, #0
	beq	.L941
	mov	r0, r4
	bl	NandcSendDumpDataStart
.L941:
	ldr	r3, [r6, #3824]
	ldr	r5, .L952
	cmp	r3, #0
	beq	.L942
	ldr	r1, [sp, #0]
	mov	r2, #1
	ldr	r0, [r5, #3816]
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r1, [sp, #0]
	ldr	r0, [r5, #3820]
	mov	r2, #1
	ubfx	r1, r1, #22, #5
	mov	r1, r1, asl #7
	bl	rknand_dma_unmap_single
.L942:
	ldr	r3, .L952
	ldr	r3, [r3, #3832]
	cmp	r3, #0
	beq	.L937
	mov	r0, r4
	bl	NandcSendDumpDataDone
.L937:
	ldr	r3, .L952
	mov	r2, #0
	str	r2, [r3, #3824]
	b	.L929
.L946:
	ldr	r3, [r4, #8]
	str	r3, [sp, #0]
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L946
.L929:
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L953:
	.align	2
.L952:
	.word	.LANCHOR0
	.word	.LC88
	.word	.LC89
	.word	.LC90
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
	ldr	r3, .L992
	.pad #92
	sub	sp, sp, #92
	tst	r4, #63
	add	r3, r3, r0, asl #3
	mov	r7, r0
	mov	fp, r1
	mov	sl, r2
	ldr	r5, [sp, #128]
	ldr	r6, [r3, #864]
	bne	.L955
	cmp	r5, #0
	bne	.L956
	add	r0, sp, #24
	mov	r1, #255
	mov	r2, #64
	add	r5, sp, #24
	bl	ftl_memset
.L956:
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
	bne	.L957
	ldr	r1, .L992
	mov	r0, sl, lsr #1
	mov	r3, fp
	mov	r2, fp
	ldr	ip, [r1, #3836]
	mov	r4, r1
	cmp	ip, #24
	movhi	ip, #128
	movls	ip, #64
	b	.L959
.L960:
	ldr	r1, [r4, #3804]
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
.L959:
	cmp	r2, r0
	add	r7, r3, ip
	ldr	r1, .L992
	bcc	.L960
	mov	r3, #0
	ldr	r0, [r1, #3836]
	mov	sl, sl, lsr #2
	ldr	r1, [r1, #3788]
	mov	r4, r3
	b	.L961
.L967:
	add	r2, r3, #8
	ldr	r2, [r6, r2, asl #2]
	str	r2, [sp, #20]
	ldr	r2, [sp, #20]
	tst	r2, #4
	bne	.L982
	ldr	r2, [sp, #20]
	ands	r2, r2, #32768
	bne	.L982
	cmp	r1, #5
	bls	.L963
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
	b	.L990
.L963:
	cmp	r1, #3
	bls	.L965
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
.L990:
	orr	r2, ip, r2, asl #5
.L965:
	cmp	r4, r2
	movcc	r4, r2
	b	.L962
.L982:
	mvn	r4, #0
.L962:
	add	r3, r3, #1
.L961:
	cmp	r3, sl
	bcs	.L957
	cmp	r0, #0
	bne	.L967
.L957:
	mov	r3, #0
	str	r3, [r6, #16]
	b	.L968
.L955:
	cmp	r1, #1
	mov	r8, #0
	bne	.L988
	b	.L969
.L972:
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
.L969:
	cmp	r8, sl
	bcc	.L972
	mov	r4, #0
	b	.L968
.L988:
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
	b	.L991
.L977:
	mov	r0, r7
	bl	NandcXferComp
	ldr	r3, [fp, #32]
	add	ip, r5, #2
	cmp	ip, sl
	str	r3, [sp, #20]
	bcs	.L974
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
.L974:
	ldr	r3, [sp, #20]
	tst	r3, #4
	mvnne	r4, #0
	bne	.L975
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r2, r2, #3, #5
	ubfx	r3, r3, #27, #1
	orr	r3, r2, r3, asl #5
	cmp	r4, r3
	movcc	r4, r3
.L975:
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
.L991:
	cmp	ip, sl
	mov	r5, ip
	bcc	.L977
	mov	r6, fp
	ldr	fp, [sp, #12]
.L968:
	ldr	r3, .L992
	rsbs	fp, fp, #1
	movcc	fp, #0
	ldr	r3, [r3, #3788]
	cmp	r3, #5
	movls	fp, #0
	cmp	fp, #0
	beq	.L978
	ldr	r3, [r6, #0]
	and	r2, r3, #139264
	cmp	r2, #139264
	orreq	r3, r3, #131072
	streq	r3, [r6, #0]
	mvneq	r4, #0
.L978:
	mov	r0, r4
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L993:
	.align	2
.L992:
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
	ldr	r3, .L997
	subs	r4, r0, #0
	mov	r5, r1
	mov	r6, r2
	ldrb	r7, [r3, #3125]	@ zero_extendqisi2
	bne	.L995
	ldr	r3, .L997+4
	ldrb	r2, [r3, #853]	@ zero_extendqisi2
	ldr	r1, [r3, #856]
	mul	r2, r1, r2
	cmp	r5, r2
	bcs	.L995
	ldrb	r3, [r3, #852]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r7, r7, #2
	movne	r7, #4
.L995:
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
.L998:
	.align	2
.L997:
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
	ldr	r3, .L1009
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r5, r2
	ldr	r2, [r3, #3624]
	mov	r9, r0
	mov	sl, r1
	ldrb	r4, [r2, #19]	@ zero_extendqisi2
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1000
	ldr	r3, [r3, #3776]
	cmp	r3, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1000:
	sub	r3, r4, #5
	uxtb	r3, r3
	cmp	r4, #68
	cmpne	r3, #3
	bls	.L1003
	cmp	r4, #35
	cmpne	r4, #19
	beq	.L1003
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1006:
	add	r3, r6, r5, asl #1
	sub	r3, r3, #880
	ldrh	r2, [r3, #0]
	movw	r3, #65535
	cmp	r2, r3
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
	cmp	r4, #8
	mov	r2, #32768
	ldr	r0, [r6, #148]
	moveq	r1, #0
	movne	r1, #255
	bl	ftl_memset
	ldr	r2, [r8, #148]
	add	r1, r5, sl
	add	r5, r5, #1
	mov	r0, r9
	mov	r3, r2
	uxth	r5, r5
	bl	FlashProgPage
	b	.L1007
.L1003:
	ldr	r6, .L1009+4
	ldr	r7, .L1009
	mov	r8, r6
.L1007:
	ldr	r3, [r7, #3624]
	ldrh	r3, [r3, #10]
	cmp	r3, r5
	bhi	.L1006
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1010:
	.align	2
.L1009:
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
	ldr	r3, .L1014
	subs	r4, r0, #0
	mov	r6, r1
	mov	r5, r2
	ldrb	r7, [r3, #3125]	@ zero_extendqisi2
	bne	.L1012
	ldr	r3, .L1014+4
	ldrb	r2, [r3, #853]	@ zero_extendqisi2
	ldr	r3, [r3, #856]
	mul	r3, r3, r2
	cmp	r1, r3
	movcc	r7, #4
.L1012:
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
.L1015:
	.align	2
.L1014:
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
	ldr	r3, .L1030
	mov	sl, r2
	ldr	fp, .L1030
	mov	r6, r0
	add	r2, r3, r0
	str	r1, [sp, #4]
	ldrb	r9, [r3, #2]	@ zero_extendqisi2
	mov	r8, #0
	ldr	r3, [r3, #3624]
	mvn	r5, #0
	ldrb	r4, [r2, #12]	@ zero_extendqisi2
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #7
	uxtb	r3, r3
	cmp	r3, #1
	ldrlsb	r4, [r2, #20]	@ zero_extendqisi2
	bl	NandcWaitFlashReady
	mov	ip, fp
	b	.L1018
.L1023:
	add	r4, r4, #1
	ldrb	r1, [fp, #1]	@ zero_extendqisi2
	mov	r0, r6
	ldr	r2, .L1030+4
	uxtb	r4, r4
	str	ip, [sp, #0]
	cmp	r4, r9
	movcs	r4, #0
	mov	r3, r4
	bl	HynixSetRRPara
	mov	r0, r6
	ldr	r1, [sp, #4]
	mov	r2, sl
	mov	r3, r7
	bl	FlashReadRawPage
	ldr	ip, [sp, #0]
	cmn	r0, #1
	beq	.L1020
	ldrb	r3, [ip, #3840]	@ zero_extendqisi2
	cmn	r5, #1
	moveq	r5, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1029
	mov	r7, #0
	mov	sl, r7
.L1020:
	add	r8, r8, #1
.L1018:
	cmp	r8, r9
	bcc	.L1023
	b	.L1022
.L1029:
	mov	r5, r0
.L1022:
	ldr	r3, .L1030
	add	r6, r3, r6
	ldr	r2, [r3, #3624]
	ldrb	r3, [r3, #3840]	@ zero_extendqisi2
	ldrb	r2, [r2, #19]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	sub	r2, r2, #7
	uxtb	r2, r2
	cmp	r2, #1
	strlsb	r4, [r6, #20]
	strhib	r4, [r6, #12]
	adds	r0, r5, #1
	movne	r0, #1
	cmp	r5, r3, lsr #2
	movcc	r0, #0
	andcs	r0, r0, #1
	cmp	r0, #0
	moveq	r0, r5
	movne	r0, #256
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1031:
	.align	2
.L1030:
	.word	.LANCHOR0
	.word	.LANCHOR0+4
	.fnend
	.size	HynixReadRetrial, .-HynixReadRetrial
	.align	2
	.global	MicronReadRetrial
	.type	MicronReadRetrial, %function
MicronReadRetrial:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r3
	ldr	r3, .L1050
	mov	r9, r0
	.pad #36
	sub	sp, sp, #36
	ldrb	r0, [r3, #3840]	@ zero_extendqisi2
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	str	r1, [sp, #24]
	cmp	r3, #0
	str	r2, [sp, #20]
	addeq	r0, r0, r0, asl #1
	ubfxeq	r0, r0, #2, #8
	beq	.L1049
	mov	r1, #3
	bl	__aeabi_idiv
	uxtb	r0, r0
.L1049:
.L1034:
	mov	r2, #0
	ldr	r3, .L1050
	mvn	r4, #0
	mov	r6, r2
	add	r3, r3, r9, asl #3
	str	r0, [sp, #16]
	str	r2, [sp, #12]
	str	r3, [sp, #28]
.L1044:
	mov	r0, r9
	mov	fp, #0
	bl	NandcWaitFlashReady
	ldr	r2, [sp, #28]
	ldrb	r7, [r2, #868]	@ zero_extendqisi2
	ldr	r8, [r2, #864]
	add	r7, r7, #8
	add	r5, r8, r7, asl #8
	b	.L1035
.L1039:
	mov	r3, #239
	mov	r0, #200
	str	r3, [r5, #8]
	mov	r3, #137
	str	r3, [r5, #4]
	bl	NandcDelayns
	add	ip, fp, #1
	mov	r0, r9
	str	ip, [r5, #0]
	ldr	r1, [sp, #24]
	mov	r3, sl
	str	r6, [r5, #0]
	ldr	r2, [sp, #20]
	str	r6, [r5, #0]
	str	r6, [r5, #0]
	str	ip, [sp, #8]
	bl	FlashReadRawPage
	ldr	ip, [sp, #8]
	cmn	r0, #1
	beq	.L1036
	cmn	r4, #1
	ldr	r2, [sp, #16]
	moveq	r4, r0
	cmp	r0, r2
	bcc	.L1046
	mov	sl, #0
	str	sl, [sp, #20]
.L1036:
	mov	fp, ip
.L1035:
	ldr	r2, .L1050+4
	ldrb	r3, [r2, #152]	@ zero_extendqisi2
	cmp	fp, r3
	bcc	.L1039
	b	.L1038
.L1046:
	mov	r4, r0
	mov	sl, #0
	str	sl, [sp, #20]
.L1038:
	mov	r3, #239
	mov	r0, #200
	str	r3, [r5, #8]
	mov	r3, #137
	str	r3, [r5, #4]
	bl	NandcDelayns
	ldr	r2, [sp, #16]
	str	r6, [r8, r7, asl #8]
	cmp	r4, r2
	str	r6, [r8, r7, asl #8]
	str	r6, [r8, r7, asl #8]
	movcc	r3, #0
	movcs	r3, #1
	cmn	r4, #1
	str	r6, [r8, r7, asl #8]
	moveq	r3, #0
	cmp	r3, #0
	movne	r4, #256
	cmn	r4, #1
	movne	r5, #0
	moveq	r5, #1
	cmn	r4, #1
	cmpne	r4, #256
	bne	.L1041
	mov	r3, fp
	ldr	r0, .L1050+8
	mov	r1, fp
	ldr	r2, [sp, #24]
	str	r4, [sp, #0]
	bl	printk
	ldr	r3, [sp, #12]
	cmp	r3, #0
	bne	.L1042
	ldr	r2, .L1050
	ldrb	r3, [r2, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r5, #0
	andne	r5, r5, #1
	cmp	r5, #0
	beq	.L1043
	mov	r0, r9
	mov	r1, #3
	bl	micron_auto_read_calibration_config
	mov	r3, #1
	str	r3, [sp, #12]
	b	.L1044
.L1041:
	ldr	r2, [sp, #12]
	cmp	r2, #0
	beq	.L1043
.L1042:
	mov	r0, r9
	mov	r1, #0
	bl	micron_auto_read_calibration_config
	cmn	r4, #1
	movne	r4, #256
.L1043:
	mov	r0, r4
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1051:
	.align	2
.L1050:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC91
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
	ldr	ip, .L1061
	mov	r5, #1
	mvn	r4, #0
	ldr	fp, .L1061+4
	add	r3, ip, r8, asl #3
	ldrb	sl, [r3, #868]	@ zero_extendqisi2
	ldr	r2, [r3, #864]
	add	sl, sl, #8
	add	sl, r2, sl, asl #8
	b	.L1053
.L1057:
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
	beq	.L1054
	ldrb	r3, [ip, #3840]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1060
	mov	r6, #0
	mov	r7, r6
.L1054:
	add	r5, r5, #1
.L1053:
	ldrb	r3, [fp, #152]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r5, r3
	bcc	.L1057
	b	.L1056
.L1060:
	mov	r4, r0
.L1056:
	mov	r0, sl
	mov	r1, #0
	bl	SamsungSetRRPara
	ldr	r3, .L1061
	adds	r0, r4, #1
	ldrb	r3, [r3, #3840]	@ zero_extendqisi2
	movne	r0, #1
	add	r3, r3, r3, asl #1
	cmp	r4, r3, lsr #2
	movcc	r0, #0
	andcs	r0, r0, #1
	cmp	r0, #0
	moveq	r0, r4
	movne	r0, #256
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1062:
	.align	2
.L1061:
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
	ldr	r3, .L1084
	add	r2, r3, r8, asl #3
	ldr	r1, [r2, #864]
	ldrb	r4, [r2, #868]	@ zero_extendqisi2
	ldrb	r2, [r3, #3761]	@ zero_extendqisi2
	add	r4, r4, #8
	sub	r2, r2, #67
	add	r4, r1, r4, asl #8
	uxtb	r2, r2
	cmp	r2, #1
	movls	sl, #0
	bls	.L1064
	ldrb	sl, [r3, #3780]	@ zero_extendqisi2
	cmp	sl, #0
	beq	.L1065
	mov	r0, #0
	mov	sl, #1
	bl	NandcSetDdrMode
.L1065:
	mov	r3, #92
	str	r3, [r4, #8]
	mov	r3, #197
	str	r3, [r4, #8]
.L1064:
	mov	r6, #1
	mvn	r5, #0
	ldr	fp, .L1084
	b	.L1066
.L1075:
	ldrb	r3, [fp, #3761]	@ zero_extendqisi2
	mov	r0, r4
	uxtb	r1, r6
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1067
	bl	SandiskSetRRPara
	b	.L1068
.L1067:
	bl	ToshibaSetRRPara
.L1068:
	ldr	r2, .L1084
	ldrb	r3, [r2, #3761]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L1069
	ldr	r2, .L1084+4
	ldrb	r3, [r2, #152]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r6, r3
	moveq	r3, #179
	streq	r3, [r4, #8]
.L1069:
	cmp	sl, #0
	mov	r3, #38
	str	r3, [r4, #8]
	mov	r3, #93
	str	r3, [r4, #8]
	beq	.L1070
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
	b	.L1071
.L1070:
	mov	r3, r7
	mov	r0, r8
	ldr	r1, [sp, #4]
	mov	r2, r9
	bl	FlashReadRawPage
	mov	r3, r0
.L1071:
	cmn	r3, #1
	beq	.L1072
	ldrb	r2, [fp, #3840]	@ zero_extendqisi2
	cmn	r5, #1
	moveq	r5, r3
	add	r2, r2, r2, asl #1
	cmp	r3, r2, lsr #2
	bcc	.L1083
	mov	r7, #0
	mov	r9, r7
.L1072:
	add	r6, r6, #1
.L1066:
	ldr	r2, .L1084+4
	ldrb	r3, [r2, #152]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r6, r3
	bcc	.L1075
	b	.L1074
.L1083:
	mov	r5, r3
.L1074:
	ldr	r6, .L1084
	mov	r0, r4
	mov	r1, #0
	ldrb	r3, [r6, #3761]	@ zero_extendqisi2
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1076
	bl	SandiskSetRRPara
	b	.L1077
.L1076:
	bl	ToshibaSetRRPara
.L1077:
	mov	r3, #255
	str	r3, [r4, #8]
	ldrb	r3, [r6, #3840]	@ zero_extendqisi2
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
	beq	.L1079
	mov	r0, #4
	bl	NandcSetDdrMode
.L1079:
	mov	r0, r5
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1085:
	.align	2
.L1084:
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
	ldr	r5, .L1093
	ldr	r7, .L1093+4
	ldr	r8, .L1093+8
	ldr	r3, [r5, #160]
	ldrb	r0, [r5, #164]	@ zero_extendqisi2
	str	r3, [r5, #156]
	bl	FlashBchSel
	mov	r1, #0
	mov	r2, #2048
	ldr	r0, [r5, #160]
	bl	ftl_memset
	ldr	r3, [r5, #156]
	add	r1, r7, #2976
	mov	r2, #32
	str	r8, [r3, #0]
	ldr	r4, [r5, #156]
	ldrb	r3, [r7, #3762]	@ zero_extendqisi2
	add	r0, r4, #16
	strh	r3, [r4, #12]	@ movhi
	ldrb	r3, [r7, #853]	@ zero_extendqisi2
	strh	r3, [r4, #14]	@ movhi
	ldrb	r3, [r7, #3780]	@ zero_extendqisi2
	str	r3, [r4, #1076]
	bl	memcpy
	add	r0, r4, #80
	ldr	r1, .L1093+12
	mov	r2, #8
	bl	memcpy
	add	r0, r4, #96
	ldr	r1, .L1093+16
	mov	r2, #32
	bl	memcpy
	ldr	r0, [r5, #156]
	ldr	r1, .L1093+20
	mov	r2, #32
	add	r0, r0, #160
	bl	memcpy
	ldr	r4, [r5, #156]
	add	r1, r7, #896
	mov	r2, #32
	add	r0, r4, #192
	bl	memcpy
	mov	r2, #852
	add	r0, r4, #224
	mov	r1, r7
	bl	memcpy
	add	r0, r4, #12
	movw	r1, #2036
	bl	js_hash
	movw	r3, #1592
	str	r3, [r4, #4]
	ldr	r3, [r5, #148]
	str	r3, [r5, #156]
	str	r0, [r4, #8]
	mov	r0, #0
	bl	flash_enter_slc_mode
	mov	r4, r6
.L1089:
	ldr	r1, [r7, #856]
	mov	r0, #0
	mov	r2, r0
	mul	r1, r1, r4
	bl	FlashEraseBlock
	ldr	r1, [r7, #856]
	mov	r0, #0
	ldr	r2, [r5, #160]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashProgPage
	ldr	r1, [r7, #856]
	mov	r0, #0
	ldr	r2, [r5, #160]
	mov	r3, r0
	mul	r1, r1, r4
	add	r1, r1, #1
	bl	FlashProgPage
	ldr	r1, [r7, #856]
	mov	r0, #0
	ldr	r2, [r5, #148]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	beq	.L1087
	ldr	sl, [r5, #156]
	ldr	r3, [sl, #0]
	cmp	r3, r8
	bne	.L1087
	add	r0, sl, #12
	movw	r1, #2036
	bl	js_hash
	ldr	r3, [sl, #8]
	cmp	r3, r0
	bne	.L1087
	add	r3, r4, #1
	str	r3, [r5, #168]
	ldr	r3, [r7, #856]
	add	r6, r6, #1
	cmp	r6, #1
	mul	r3, r3, r4
	str	r3, [r5, #172]
	bhi	.L1088
.L1087:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L1089
.L1088:
	mov	r0, #0
	bl	flash_exit_slc_mode
	cmp	r6, #0
	mvneq	r0, #0
	movne	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1094:
	.align	2
.L1093:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	1312902724
	.word	.LANCHOR0+3764
	.word	.LANCHOR0+3588
	.word	.LANCHOR1+3116
	.fnend
	.size	FlashSavePhyInfo, .-FlashSavePhyInfo
	.align	2
	.global	FlashReadIdbDataRaw
	.type	FlashReadIdbDataRaw, %function
FlashReadIdbDataRaw:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	mov	r2, #4
	mov	sl, r0
	ldr	r1, .L1106
	add	r0, sp, #12
	bl	memcpy
	ldr	r3, .L1106+4
	ldrb	r2, [r3, #3840]	@ zero_extendqisi2
	ldr	r3, [r3, #3776]
	cmp	r3, #0
	str	r2, [sp, #4]
	beq	.L1096
	mov	r0, #0
	bl	flash_enter_slc_mode
.L1096:
	ldr	r7, .L1106+4
	mov	r0, sl
	mov	r1, #0
	mov	r2, #2048
	bl	ftl_memset
	mvn	r8, #0
	mov	r4, #2
	ldr	r9, .L1106+8
	mov	fp, r7
	b	.L1097
.L1104:
	mov	r6, #0
.L1099:
	add	r3, sp, #12
	ldr	r5, .L1106+8
	ldrb	r0, [r3, r6]	@ zero_extendqisi2
	bl	FlashBchSel
	ldr	r1, [r7, #856]
	mov	r0, #0
	ldr	r2, [r9, #160]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1098
	add	r6, r6, #1
	cmp	r6, #4
	bne	.L1099
	b	.L1100
.L1098:
	ldr	r3, [r5, #160]
	ldr	r2, [r3, #0]
	ldr	r3, .L1106+12
	cmp	r2, r3
	bne	.L1100
	add	r3, sp, #16
	ldr	r0, .L1106+16
	add	r6, r3, r6
	ldrb	r1, [r6, #-4]	@ zero_extendqisi2
	bl	printk
	mov	r0, sl
	ldr	r1, [r5, #160]
	mov	r2, #2048
	bl	memcpy
	ldr	r3, [r5, #160]
	ldr	r3, [r3, #512]
	strb	r3, [fp, #853]
	ldr	r3, [r5, #168]
	cmp	r3, r4
	bls	.L1103
	str	r4, [r5, #168]
	bl	FlashSavePhyInfo
	mov	r8, #0
.L1100:
	add	r4, r4, #1
.L1097:
	ldrb	r3, [r7, #853]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1104
	b	.L1101
.L1103:
	mov	r8, #0
.L1101:
	ldr	r0, [sp, #4]
	bl	FlashBchSel
	ldr	r3, .L1106+4
	ldr	r3, [r3, #3776]
	cmp	r3, #0
	beq	.L1102
	mov	r0, #0
	bl	flash_exit_slc_mode
.L1102:
	mov	r0, r8
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1107:
	.align	2
.L1106:
	.word	.LANCHOR3+11
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-52655045
	.word	.LC92
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
	ldr	r1, .L1119
	add	r0, sp, #4
	bl	memcpy
	ldr	r4, .L1119+4
	ldr	r3, .L1119+8
	mov	r5, #0
	mov	r0, r5
	mov	r8, #4
	str	r5, [r4, #172]
	mvn	r6, #0
	ldr	r9, [r3, #856]
	ldr	r3, [r4, #160]
	ldr	fp, .L1119+12
	str	r3, [r4, #156]
	bl	flash_enter_slc_mode
	b	.L1109
.L1111:
	add	r3, sp, #4
	ldrb	r0, [r3, r7]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	mov	r1, r5
	ldr	r2, [r4, #160]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1110
	mov	r0, #0
	mov	r1, sl
	ldr	r2, [r4, #160]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1110
	add	r7, r7, #1
	cmp	r7, #4
	beq	.L1112
	b	.L1111
.L1110:
	ldr	r7, [r4, #156]
	ldr	r3, [r7, #0]
	cmp	r3, fp
	bne	.L1112
	cmp	r6, #0
	bne	.L1113
	ldr	r3, .L1119+8
	mov	r0, r5
	ldr	r1, [r3, #856]
	bl	__aeabi_uidiv
	ldr	r3, .L1119+4
	add	r0, r0, #1
	str	r0, [r3, #168]
	mov	r0, r6
	b	.L1118
.L1113:
	add	r0, r7, #12
	movw	r1, #2036
	bl	js_hash
	ldr	r3, [r7, #8]
	cmp	r3, r0
	bne	.L1112
	ldr	r6, .L1119+8
	add	r1, r7, #160
	mov	r2, #32
	ldr	r0, .L1119+16
	bl	memcpy
	add	r1, r7, #192
	add	r0, r6, #896
	mov	r2, #32
	bl	memcpy
	add	r1, r7, #224
	mov	r0, r6
	mov	r2, #852
	bl	memcpy
	ldr	r2, .L1119+20
	movw	r3, #3126
	ldrh	r0, [r2, r3]
	bl	FlashBlockAlignInit
	ldr	r7, [r4, #156]
	ldr	r1, [r6, #856]
	mov	r0, r5
	str	r5, [r4, #172]
	ldr	r3, [r7, #1076]
	strb	r3, [r6, #3780]
	mov	r6, #0
	bl	__aeabi_uidiv
	add	r0, r0, #1
	str	r0, [r4, #168]
	cmp	r0, #1
	movls	r3, #2
	strls	r3, [r4, #168]
	ldrh	r3, [r7, #14]
	strb	r3, [r4, #176]
.L1112:
	subs	r8, r8, #1
	add	r5, r5, r9
	beq	.L1116
.L1109:
	add	sl, r5, #1
	mov	r7, #0
	b	.L1111
.L1116:
	mov	r0, r8
.L1118:
	bl	flash_exit_slc_mode
	mov	r0, r6
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1120:
	.align	2
.L1119:
	.word	.LANCHOR3+11
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	1312902724
	.word	.LANCHOR1+3116
	.word	.LANCHOR1
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
	ldr	r4, .L1142
	.pad #20
	sub	sp, sp, #20
	mov	fp, r0
	mov	r9, r2
	str	r1, [sp, #0]
	ldr	r3, [r4, #3008]
	ldr	r7, [r4, #3788]
	ldr	r3, [r3, #304]
	cmp	r7, #7
	movhi	r7, #12
	movls	r7, #6
	str	r3, [sp, #12]
	ldr	r3, [sp, #56]
	cmp	r3, #0
	moveq	r8, #1024
	beq	.L1123
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
	beq	.L1124
	ldr	r0, .L1142+4
	mov	r2, r8
	ldr	r1, [sp, #0]
	bl	printk
	cmp	r8, #9
	bhi	.L1125
	add	r4, r4, fp, asl #3
	ldr	r3, [r4, #864]
	ldr	r2, [r3, #3840]
	ldr	r2, [r3, #0]
	orr	r2, r2, #131072
	str	r2, [r3, #0]
.L1125:
	ldr	r3, .L1142+8
	ldr	r2, [r3, #180]
	add	r2, r2, #1
	str	r2, [r3, #180]
	cmp	r2, #2048
	movcs	sl, #0
	strcs	sl, [r3, #180]
	movcs	r9, sl
	bcc	.L1124
.L1123:
	mov	r4, #0
	str	fp, [sp, #4]
	mov	r5, r4
	mvn	r6, #0
	mov	fp, r4
	str	r4, [sp, #8]
.L1130:
	uxtb	r0, r7
	bl	NandcSetDdrPara
	mov	r3, sl
	ldr	r0, [sp, #4]
	mov	r2, r9
	ldr	r1, [sp, #0]
	bl	FlashReadRawPage
	add	r3, r8, #1
	cmp	r0, r3
	bhi	.L1126
	cmp	r0, #2
	bhi	.L1136
	add	r5, r5, #1
	cmp	r5, #9
	bls	.L1136
	mov	ip, fp
	rsb	r4, r5, r7
	ldr	fp, [sp, #4]
	mov	r8, r0
	mov	r6, #0
	b	.L1128
.L1126:
	cmp	fp, r5
	bcs	.L1137
	cmp	r5, #7
	rsb	r3, r5, r4
	str	r3, [sp, #8]
	bhi	.L1141
	mov	fp, r5
	b	.L1137
.L1136:
	mov	r6, #0
	mov	r4, r7
	mov	r8, r0
	mov	sl, r6
	mov	r9, r6
	b	.L1127
.L1137:
	mov	r5, #0
.L1127:
	add	r7, r7, #2
	cmp	r7, #69
	bls	.L1130
	mov	ip, fp
	ldr	fp, [sp, #4]
.L1128:
	cmp	ip, r5
	bcc	.L1131
	b	.L1129
.L1141:
	ldr	fp, [sp, #4]
.L1129:
	ldr	r4, [sp, #8]
.L1131:
	cmp	r4, #0
	beq	.L1132
	ldr	r0, .L1142+12
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	bl	NandcSetDdrPara
.L1132:
	cmn	r6, #1
	bne	.L1124
	ldr	r0, .L1142+16
	mov	r1, fp
	ldr	r2, [sp, #0]
	bl	printk
	ldr	r3, [sp, #56]
	cmp	r3, #0
	moveq	r8, r6
	beq	.L1124
	ldr	r3, [sp, #12]
	ubfx	r0, r3, #8, #8
	bl	NandcSetDdrPara
.L1124:
	mov	r0, r8
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1143:
	.align	2
.L1142:
	.word	.LANCHOR0
	.word	.LC93
	.word	.LANCHOR2
	.word	.LC94
	.word	.LC95
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
	ldr	r5, .L1148
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
	beq	.L1145
	cmn	r8, #1
	bne	.L1146
.L1145:
	ldrb	r3, [r5, #3772]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L1146
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r3, .L1148
	mov	r2, #0
	strb	r2, [r3, #3780]
	b	.L1147
.L1146:
	mov	r3, #1
	strb	r3, [r5, #3780]
.L1147:
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1149:
	.align	2
.L1148:
	.word	.LANCHOR0
	.fnend
	.size	FlashDdrParaScan, .-FlashDdrParaScan
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
	bne	.L1151
	ldr	r7, .L1156
	ldrb	r9, [r7, #860]	@ zero_extendqisi2
	cmp	r9, #0
	beq	.L1152
	mov	r3, #0
	mov	r0, r5
	strb	r3, [r7, #860]
	mov	r1, r6
	mov	r2, r8
	mov	r3, sl
	bl	FlashReadRawPage
	strb	r9, [r7, #860]
	cmn	r0, #1
	movne	r4, r0
	bne	.L1151
.L1152:
	ldr	r7, .L1156
	ldrb	r3, [r7, #3780]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1151
	ldr	r3, [r7, #3008]
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
	beq	.L1153
	ldrb	r3, [r7, #3840]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1151
.L1153:
	ubfx	r0, r9, #8, #8
	bl	NandcSetDdrPara
.L1151:
	ldr	r7, .L1156+4
	ldr	ip, [r7, #184]
	adds	r3, ip, #0
	movne	r3, #1
	cmn	r4, #1
	movne	r3, #0
	cmp	r3, #0
	beq	.L1154
	mov	r1, r6
	mov	r2, r8
	mov	r3, sl
	mov	r0, r5
	blx	ip
	mov	r2, r5
	mov	r3, r6
	mov	r4, r0
	ldr	r0, .L1156+8
	mov	r1, r4
	bl	printk
	cmn	r4, #1
	bne	.L1154
	ldr	r3, .L1156
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1154
	mov	r0, r5
	bl	flash_enter_slc_mode
	ldr	ip, [r7, #184]
	mov	r0, r5
	mov	r1, r6
	mov	r2, r8
	mov	r3, sl
	blx	ip
	mov	r4, r0
	mov	r0, r5
	bl	flash_exit_slc_mode
.L1154:
	mov	r0, r4
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L1157:
	.align	2
.L1156:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC96
	.fnend
	.size	FlashReadPage, .-FlashReadPage
	.align	2
	.global	FlashReadSlc2KPages
	.type	FlashReadSlc2KPages, %function
FlashReadSlc2KPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1171
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r1
	.pad #28
	sub	sp, sp, #28
	mov	fp, r2
	ldrb	sl, [r3, #3125]	@ zero_extendqisi2
	mov	r4, r0
	mov	r7, #0
	ldr	r6, .L1171+4
	b	.L1159
.L1168:
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
	bcs	.L1161
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
	ldr	r1, [r6, #856]
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
	ldrb	r3, [r6, #3840]	@ zero_extendqisi2
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
	beq	.L1166
	ldr	r2, [r3, #12]
	cmn	r2, #1
	bne	.L1166
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L1166
	ldr	r3, [r3, #0]
	cmn	r3, #1
	strne	r2, [r4, #0]
.L1166:
	ldr	r3, [r4, #0]
	cmn	r3, #1
	bne	.L1161
	ldr	r1, [r4, #4]
	ldr	r0, .L1171+8
	ldrb	r2, [r6, #3840]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r4, #8]
	cmp	r1, #0
	beq	.L1167
	ldr	r0, .L1171+12
	mov	r2, #4
	mov	r3, #8
	bl	rknand_print_hex
.L1167:
	ldr	r1, [r4, #12]
	cmp	r1, #0
	beq	.L1161
	mov	r2, #4
	ldr	r0, .L1171+16
	mov	r3, r2
	bl	rknand_print_hex
.L1161:
	add	r7, r7, #1
	add	r4, r4, #36
.L1159:
	cmp	r7, r8
	bne	.L1168
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1172:
	.align	2
.L1171:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LC97
	.word	.LC98
	.word	.LC99
	.fnend
	.size	FlashReadSlc2KPages, .-FlashReadSlc2KPages
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1215
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldrb	r3, [r3, #3125]	@ zero_extendqisi2
	mov	r9, r0
	str	r1, [sp, #28]
	str	r2, [sp, #32]
	str	r3, [sp, #24]
	ldr	r3, .L1215+4
	ldrb	r8, [r3, #852]	@ zero_extendqisi2
	ldrb	ip, [r3, #860]	@ zero_extendqisi2
	cmp	r8, #0
	str	ip, [sp, #36]
	moveq	sl, r8
	beq	.L1174
	bl	FlashReadSlc2KPages
	b	.L1175
.L1202:
	mov	r3, #36
	ldr	ip, [sp, #28]
	mul	r3, r3, sl
	add	r2, sp, #44
	ldr	r1, [sp, #32]
	ldr	fp, .L1215+4
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
	bcs	.L1177
	add	r3, fp, r3
	ldrb	r4, [r3, #3764]	@ zero_extendqisi2
	ldr	r3, .L1215+8
	mov	r0, r4
	ldrb	r3, [r3, #188]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r7, #0
	bl	NandcWaitFlashReady
	ldr	r3, [fp, #3624]
	ldrb	r2, [r3, #19]	@ zero_extendqisi2
	sub	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #7
	bhi	.L1179
	sub	r2, r2, #7
	add	r1, fp, r4
	add	fp, fp, r4
	uxtb	r2, r2
	ldrb	r3, [r1, #12]	@ zero_extendqisi2
	cmp	r2, #1
	ldr	r2, .L1215+4
	ldrlsb	r3, [r1, #20]	@ zero_extendqisi2
	ldrb	r1, [fp, #3756]	@ zero_extendqisi2
	cmp	r1, r3
	beq	.L1179
	ldrb	r1, [r2, #1]	@ zero_extendqisi2
	mov	r0, r4
	add	r2, r2, #4
	bl	HynixSetRRPara
.L1179:
	mov	r0, r4
	mov	r5, r5, lsr #31
	bl	NandcFlashCs
	ldr	ip, [sp, #32]
	mov	r0, r4
	cmp	ip, #1
	orreq	r5, r5, #1
	str	r5, [sp, #20]
	cmp	r5, #0
	beq	.L1181
	ldr	r3, .L1215+4
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1181
	bl	flash_enter_slc_mode
	b	.L1182
.L1181:
	bl	flash_exit_slc_mode
.L1182:
	ldr	fp, .L1215+4
.L1214:
	ldr	r1, [sp, #44]
	cmn	r1, #1
	cmpeq	r4, #255
	moveq	r3, #0
	movne	r3, #1
	moveq	r7, r3
	beq	.L1184
	cmp	r7, #0
	beq	.L1185
	ldr	r2, [fp, #856]
	mov	r0, r4
	add	r2, r1, r2
	bl	FlashReadDpCmd
	b	.L1186
.L1185:
	mov	r0, r4
	bl	FlashReadCmd
.L1186:
	mov	r0, r4
	bl	NandcWaitFlashReady
	cmp	r7, #0
	beq	.L1184
	mov	r0, r4
	ldr	r1, [sp, #44]
	bl	FlashReadDpDataOutCmd
.L1184:
	ldr	r3, [r6, #12]
	mov	r0, r4
	ldr	r2, [sp, #24]
	mov	r1, #0
	str	r3, [sp, #0]
	ldr	r3, [r6, #8]
	bl	NandcXferData
	ldrb	r2, [fp, #860]	@ zero_extendqisi2
	adds	r2, r2, #0
	movne	r2, #1
	cmn	r0, #1
	movne	r2, #0
	cmp	r2, #0
	movne	r2, #0
	movne	r7, #0
	strneb	r2, [fp, #860]
	bne	.L1214
.L1187:
	cmp	r7, #0
	mov	r5, r0
	beq	.L1188
	ldr	r3, .L1215+4
	mov	r0, r4
	ldr	r1, [r3, #856]
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
.L1188:
	mov	r0, r4
	ldr	fp, .L1215+4
	bl	NandcFlashDeCs
	ldr	ip, [sp, #36]
	cmn	r5, #1
	strb	ip, [fp, #860]
	bne	.L1192
	ldrb	r3, [fp, #3780]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1190
	ldr	r3, [fp, #3008]
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
	beq	.L1191
	ldrb	r3, [fp, #3840]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1208
.L1191:
	ubfx	r0, r7, #8, #8
	bl	NandcSetDdrPara
	cmn	r5, #1
	bne	.L1208
.L1190:
	ldr	r3, .L1215+8
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r6, #8]
	ldr	r7, [r3, #184]
	ldr	r3, [r6, #12]
	cmp	r7, #0
	beq	.L1193
	blx	r7
	cmn	r0, #1
	mov	r5, r0
	bne	.L1210
	ldr	r2, .L1215+4
	ldr	r3, [r2, #3624]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #7
	bhi	.L1195
	ldrb	r1, [r2, #1]	@ zero_extendqisi2
	mov	r0, r4
	add	r2, r2, #4
	mov	r3, #0
	bl	HynixSetRRPara
.L1195:
	ldr	r1, [sp, #44]
	mov	r0, r4
	ldr	r2, [r6, #8]
	ldr	r3, [r6, #12]
	bl	FlashReadRawPage
	ldr	r7, .L1215+4
	ldr	r1, [r6, #4]
	ldrb	r2, [r7, #3840]	@ zero_extendqisi2
	mov	r5, r0
	ldr	r0, .L1215+12
	mov	r3, r5
	bl	printk
	cmn	r5, #1
	bne	.L1210
	ldrb	r7, [r7, #3072]	@ zero_extendqisi2
	cmp	r7, #0
	beq	.L1194
	ldr	ip, [sp, #20]
	mov	r0, r4
	cmp	ip, #0
	beq	.L1196
	bl	flash_enter_slc_mode
	b	.L1197
.L1196:
	bl	flash_exit_slc_mode
.L1197:
	ldr	r3, .L1215+8
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r6, #8]
	ldr	ip, [r3, #184]
	ldr	r3, [r6, #12]
	blx	ip
	mov	r5, r0
	b	.L1210
.L1193:
	bl	FlashReadRawPage
	mov	r5, r0
	b	.L1194
.L1208:
	mov	r7, #0
.L1192:
	ldr	r3, .L1215+4
	ldrb	r3, [r3, #3840]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r5, r3, lsr #2
	bls	.L1194
	ldr	r3, .L1215+8
	ldr	r3, [r3, #184]
	cmp	r3, #0
	moveq	r5, #256
	b	.L1194
.L1210:
	mov	r7, #0
.L1194:
	ldr	ip, [sp, #16]
	cmp	r5, #256
	cmnne	r5, #1
	movne	r2, #0
	str	r5, [r9, ip]
	strne	r2, [r9, ip]
	ldr	ip, [sp, #16]
	ldr	r3, [r9, ip]
	cmn	r3, #1
	bne	.L1199
	ldr	r2, .L1215+4
	ldr	r1, [r6, #4]
	ldr	r0, .L1215+16
	ldrb	r2, [r2, #3840]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r6, #12]
	cmp	r1, #0
	beq	.L1199
	mov	r2, #4
	ldr	r0, .L1215+20
	mov	r3, r2
	bl	rknand_print_hex
.L1199:
	cmp	r7, #0
	beq	.L1200
	ldr	r3, .L1215+4
	ldrb	r3, [r3, #3840]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r8, r3, lsr #2
	bls	.L1201
	ldr	r3, .L1215+8
	ldr	r3, [r3, #184]
	cmp	r3, #0
	moveq	r8, #256
.L1201:
	add	r3, sl, #1
	mov	r2, #36
	cmp	r8, #256
	cmnne	r8, #1
	mul	r3, r2, r3
	movne	r2, #0
	str	r8, [r9, r3]
	strne	r2, [r9, r3]
.L1200:
	ldr	ip, [sp, #20]
	add	sl, sl, r7
	cmp	ip, #0
	beq	.L1177
	ldr	r3, .L1215+4
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1177
	mov	r0, r4
	bl	flash_exit_slc_mode
.L1177:
	add	sl, sl, #1
.L1174:
	ldr	ip, [sp, #28]
	cmp	sl, ip
	bcc	.L1202
	mov	r0, #0
.L1175:
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1216:
	.align	2
.L1215:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC100
	.word	.LC97
	.word	.LC99
	.fnend
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FtlScanAllBlock
	.type	FtlScanAllBlock, %function
FtlScanAllBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #84
	sub	sp, sp, #84
	ldr	r0, .L1227
	mov	r5, #0
	ldr	r1, .L1227+4
	bl	printk
	ldr	r7, .L1227+8
	b	.L1218
.L1226:
	add	r4, sp, #80
	mov	r0, r5
	ldr	r6, .L1227+12
	mov	r9, #36
	strh	r5, [r4, #-48]!	@ movhi
	bl	ftl_get_blk_mode
	mov	r2, r5, asl #1
	ldr	ip, [r6, #-2012]
	mov	r1, r5
	ldrh	r2, [ip, r2]
	mov	r3, r0
	ldr	r0, .L1227+16
	bl	printk
	mov	r0, r4
	bl	make_superblock
	movw	r3, #3852
	ldrh	sl, [r7, r3]
	add	r3, r3, #76
	add	r0, r4, #14
	mov	r4, #0
	ldrh	r3, [r7, r3]
	ldr	r8, [r6, #-960]
	ldr	lr, [r6, #-1164]
	str	r3, [sp, #28]
	movw	r3, #3930
	ldrh	ip, [r7, r3]
	mov	r3, r4
	ldr	r6, [r6, #-1160]
	b	.L1219
.L1221:
	ldrh	r1, [r0, #2]!
	movw	r2, #65535
	cmp	r1, r2
	beq	.L1220
	mla	r2, r9, r4, r8
	ldr	fp, [sp, #28]
	mov	r1, r1, asl #10
	str	r1, [r2, #4]
	mul	r1, fp, r4
	add	fp, r1, #3
	cmp	r1, #0
	movlt	r1, fp
	bic	r1, r1, #3
	add	r1, lr, r1
	str	r1, [r2, #8]
	mul	r1, ip, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	fp, r1, #3
	cmp	r1, #0
	movlt	r1, fp
	bic	r1, r1, #3
	add	r1, r6, r1
	str	r1, [r2, #12]
.L1220:
	add	r3, r3, #1
	uxth	r3, r3
.L1219:
	cmp	r3, sl
	bne	.L1221
	ldr	sl, .L1227+12
	mov	r1, r4
	mov	r2, #0
	mov	r6, #0
	mov	r8, r6
	ldr	r0, [sl, #-960]
	bl	FlashReadPages
	b	.L1222
.L1223:
	ldr	r2, [sl, #-960]
	add	r8, r8, #1
	ldrh	r1, [sp, #32]
	add	r3, r2, r6
	uxth	r8, r8
	ldr	r0, [r3, #12]
	ldr	ip, [r3, #8]
	ldr	lr, [r0, #0]
	str	lr, [sp, #0]
	ldr	lr, [r0, #4]
	str	lr, [sp, #4]
	ldr	lr, [r0, #8]
	str	lr, [sp, #8]
	ldr	r0, [r0, #12]
	str	r0, [sp, #12]
	ldr	r0, [ip, #0]
	str	r0, [sp, #16]
	ldr	r0, [ip, #4]
	str	r0, [sp, #20]
	ldr	r0, .L1227+20
	ldr	r2, [r2, r6]
	add	r6, r6, #36
	ldr	r3, [r3, #4]
	bl	printk
.L1222:
	cmp	r8, r4
	ldr	r9, .L1227+12
	bne	.L1223
	ldr	r0, [r9, #-960]
	mov	r1, r4
	mov	r2, #1
	mov	r6, #0
	bl	FlashReadPages
	mov	r8, r6
	mov	sl, r9
	b	.L1224
.L1225:
	ldr	r2, [sl, #-960]
	add	r8, r8, #1
	ldrh	r1, [sp, #32]
	add	r3, r2, r6
	uxth	r8, r8
	ldr	r0, [r3, #12]
	ldr	ip, [r3, #8]
	ldr	lr, [r0, #0]
	str	lr, [sp, #0]
	ldr	lr, [r0, #4]
	str	lr, [sp, #4]
	ldr	lr, [r0, #8]
	str	lr, [sp, #8]
	ldr	r0, [r0, #12]
	str	r0, [sp, #12]
	ldr	r0, [ip, #0]
	str	r0, [sp, #16]
	ldr	r0, [ip, #4]
	str	r0, [sp, #20]
	ldr	r0, .L1227+24
	ldr	r2, [r2, r6]
	add	r6, r6, #36
	ldr	r3, [r3, #4]
	bl	printk
.L1224:
	cmp	r8, r4
	bne	.L1225
	add	r5, r5, #1
	uxth	r5, r5
.L1218:
	movw	r3, #3862
	ldrh	r3, [r7, r3]
	cmp	r3, r5
	bhi	.L1226
	mov	r0, #0
	add	sp, sp, #84
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1228:
	.align	2
.L1227:
	.word	.LC101
	.word	.LANCHOR3+15
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC102
	.word	.LC103
	.word	.LC104
	.fnend
	.size	FtlScanAllBlock, .-FtlScanAllBlock
	.align	2
	.global	FtlDumpBlockInfo
	.type	FtlDumpBlockInfo, %function
FtlDumpBlockInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	ubfx	r0, r0, #10, #16
	.pad #84
	sub	sp, sp, #84
	mov	r7, r1
	bl	P2V_block_in_plane
	ldr	r5, .L1240
	ldr	r1, .L1240+4
	movw	r6, #3918
	ldrh	r9, [r5, r6]
	mov	r4, r0
	ldr	r0, .L1240+8
	bl	printk
	ldr	r3, .L1240+12
	mov	r1, r4
	ldr	r0, .L1240+16
	ldr	r2, [r3, #-2012]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	bl	printk
	add	r0, sp, #80
	strh	r4, [r0, #-48]!	@ movhi
	bl	make_superblock
	rsbs	r7, r7, #1
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	movcc	r7, #0
	cmp	r3, #0
	moveq	r7, #0
	cmp	r7, #0
	beq	.L1230
	mov	r0, r4
	bl	ftl_get_blk_mode
	cmp	r0, #1
	mov	r7, r0
	moveq	r3, #3920
	ldreqh	r9, [r5, r3]
.L1230:
	ldrh	r3, [r5, r6]
	mov	r1, r7
	ldr	r0, .L1240+20
	mov	r2, r9
	bl	printk
	mov	r6, #0
.L1231:
	ldr	r3, .L1240
	movw	r2, #3852
	movw	r1, #3928
	mov	r4, #0
	add	r0, sp, #46
	ldrh	sl, [r3, r2]
	ldr	r2, .L1240+12
	ldrh	lr, [r3, r1]
	ldr	r8, [r2, #-960]
	ldr	r5, [r2, #-1164]
	ldr	ip, [r2, #-1160]
	movw	r2, #3930
	ldrh	r2, [r3, r2]
	mov	r3, r4
	str	r2, [sp, #28]
	b	.L1232
.L1234:
	ldrh	r1, [r0, #2]!
	movw	r2, #65535
	cmp	r1, r2
	beq	.L1233
	mov	fp, #36
	orr	r1, r6, r1, asl #10
	mla	r2, fp, r4, r8
	str	r1, [r2, #4]
	mul	r1, lr, r4
	add	fp, r1, #3
	cmp	r1, #0
	movlt	r1, fp
	ldr	fp, [sp, #28]
	bic	r1, r1, #3
	add	r1, r5, r1
	str	r1, [r2, #8]
	mul	r1, fp, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	fp, r1, #3
	cmp	r1, #0
	movlt	r1, fp
	bic	r1, r1, #3
	add	r1, ip, r1
	str	r1, [r2, #12]
.L1233:
	add	r3, r3, #1
	uxth	r3, r3
.L1232:
	cmp	r3, sl
	bne	.L1234
	ldr	r8, .L1240+12
	mov	r1, r4
	mov	r2, r7
	mov	r5, #0
	mov	sl, r5
	ldr	r0, [r8, #-960]
	bl	FlashReadPages
	b	.L1235
.L1236:
	ldr	r2, [r8, #-960]
	add	sl, sl, #1
	ldrh	r1, [sp, #32]
	add	r3, r2, r5
	uxth	sl, sl
	ldr	r0, [r3, #12]
	ldr	ip, [r3, #8]
	ldr	lr, [r0, #0]
	str	lr, [sp, #0]
	ldr	lr, [r0, #4]
	str	lr, [sp, #4]
	ldr	lr, [r0, #8]
	str	lr, [sp, #8]
	ldr	r0, [r0, #12]
	str	r0, [sp, #12]
	ldr	r0, [ip, #0]
	str	r0, [sp, #16]
	ldr	r0, [ip, #4]
	str	r0, [sp, #20]
	ldr	r0, .L1240+24
	ldr	r2, [r2, r5]
	add	r5, r5, #36
	ldr	r3, [r3, #4]
	bl	printk
.L1235:
	cmp	sl, r4
	bne	.L1236
	add	r6, r6, #1
	uxth	r6, r6
	cmp	r6, r9
	bne	.L1231
.L1237:
	mov	r0, #0
	add	sp, sp, #84
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1241:
	.align	2
.L1240:
	.word	.LANCHOR0
	.word	.LANCHOR3+31
	.word	.LC101
	.word	.LANCHOR2
	.word	.LC105
	.word	.LC106
	.word	.LC107
	.fnend
	.size	FtlDumpBlockInfo, .-FtlDumpBlockInfo
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
	ldr	r5, .L1300
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L1300+4
	mov	r1, r6
	ldr	r3, .L1300+8
	movw	r7, #3940
	ldr	r2, [r5, #3948]
	ldr	r0, [r4, #-1024]
	mov	sl, r4
	strh	r6, [r4, r3]	@ movhi
	movw	r3, #3966
	strh	r6, [r5, r3]	@ movhi
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r2, [r5, #3948]
	mov	r1, r6
	ldr	r0, [r4, #-1028]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r2, [r5, r7]
	mov	r1, r6
	ldr	r0, [r4, #-896]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r5, r7]
	mov	r1, r6
	ldr	r0, [r5, #3968]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r0, .L1300+12
	mov	r1, #255
	mov	r2, #16
	bl	ftl_memset
	movw	r3, #3860
	ldrh	r3, [r5, r3]
	str	r3, [sp, #8]
	b	.L1243
.L1245:
	add	r3, r6, r5
	ldr	r1, [sp, #8]
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #0]
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #0]
	cmp	r0, #0
	bne	.L1244
	ldr	r2, [r4, #-960]
	mov	r3, r3, asl #10
	mla	r2, r9, r7, r2
	str	r3, [r2, #4]
	ldr	r3, [r4, #-1164]
	str	r3, [r2, #8]
	ldrh	r3, [r6, r8]
	mul	r3, r3, r7
	add	r7, r7, #1
	uxth	r7, r7
	add	r1, r3, #3
	cmp	r3, #0
	movlt	r3, r1
	ldr	r1, [r4, #-1160]
	bic	r3, r3, #3
	add	r3, r1, r3
	str	r3, [r2, #12]
.L1244:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L1278
.L1291:
	mov	r7, #0
	movw	fp, #3852
	mov	r5, r7
	mov	r9, #36
	movw	r8, #3930
.L1278:
	ldrh	r3, [r6, fp]
	cmp	r3, r5
	bhi	.L1245
	cmp	r7, #0
	beq	.L1246
	ldr	r0, [r4, #-960]
	mov	r1, r7
	mov	r2, #1
	mov	fp, r7
	bl	FlashReadPages
	mov	r8, #0
	str	r8, [sp, #12]
.L1277:
	ldr	r3, [r4, #-960]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	ldr	r6, [r2, #4]
	cmn	r3, #1
	ldr	r5, [r2, #12]
	ubfx	r6, r6, #10, #16
	bne	.L1247
	mov	r7, #16
.L1249:
	ldr	r0, [r4, #-960]
	mov	r1, #1
	mov	r2, r1
	add	r0, r0, r8
	ldr	r3, [r0, #4]
	add	r3, r3, #1
	str	r3, [r0, #4]
	bl	FlashReadPages
	ldrh	r3, [r5, #0]
	movw	r1, #65535
	cmp	r3, r1
	ldreq	r3, [sl, #-960]
	mvneq	r2, #0
	streq	r2, [r3, r8]
	beq	.L1247
.L1248:
	ldr	r3, [r4, #-960]
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L1247
	sub	r7, r7, #1
	uxth	r7, r7
	cmp	r7, #0
	bne	.L1249
.L1247:
	ldr	r3, [r4, #-960]
	ldr	r3, [r3, r8]
	cmn	r3, #1
	beq	.L1250
	ldr	r2, [r4, #-1800]
	ldr	r3, [r5, #4]
	cmn	r2, #1
	beq	.L1251
	cmp	r2, r3
	bhi	.L1252
.L1251:
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [sl, #-1800]
.L1252:
	ldrh	r3, [r5, #0]
	movw	r2, #61604
	cmp	r3, r2
	beq	.L1255
	bhi	.L1258
	movw	r2, #61574
	cmp	r3, r2
	bne	.L1253
	b	.L1298
.L1258:
	movw	r2, #61634
	cmp	r3, r2
	beq	.L1256
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1253
	b	.L1299
.L1256:
	ldr	r3, .L1300
	mov	r9, r6
	ldr	r1, .L1300+8
	ldr	ip, [r4, #-1024]
	ldr	r0, [r3, #3948]
	ldrh	r1, [r4, r1]
	uxth	r2, r0
	sub	r3, r2, #1
	rsb	r2, r1, r2
	sub	r2, r2, #1
	uxth	r3, r3
	sxth	r2, r2
	str	r2, [sp, #4]
	b	.L1259
.L1265:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L1260
	ldr	ip, [ip, #0]
	mov	r6, r9
	cmp	ip, #0
	bne	.L1261
	cmp	r1, r0
	addne	r1, r1, #1
	ldrne	r0, .L1300+8
	strneh	r1, [r4, r0]	@ movhi
.L1261:
	mov	r0, #0
	sxth	r9, r3
	str	r3, [sp, #4]
	b	.L1262
.L1263:
	ldr	r7, [r4, #-1024]
	add	ip, r1, #1
	add	r0, r0, #1
	ldr	r3, [r7, ip, asl #2]
	mov	ip, ip, asl #1
	uxth	r0, r0
	str	r3, [r7, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	r7, [r4, #-1028]
	ldrh	ip, [r7, ip]
	strh	ip, [r7, r1]	@ movhi
.L1262:
	sxth	r1, r0
	cmp	r1, r9
	bne	.L1263
	ldr	r1, [sl, #-1024]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	ldr	r3, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, [sl, #-1028]
	strh	r6, [r1, r2]	@ movhi
	b	.L1264
.L1260:
	sub	r3, r3, #1
	uxth	r3, r3
.L1259:
	ldr	r6, [sp, #4]
	sxth	r2, r3
	cmp	r2, r6
	bgt	.L1265
	mov	r6, r9
.L1264:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L1253
	ldr	r2, .L1300+8
	ldr	r0, .L1300
	ldrh	r1, [r4, r2]
	ldr	r0, [r0, #3948]
	rsb	r0, r1, r0
	sub	r0, r0, #1
	sxth	r0, r0
	cmp	r3, r0
	bgt	.L1253
	add	r1, r1, #1
	strh	r1, [r4, r2]	@ movhi
	ldr	r2, [r4, #-1024]
	ldr	r1, [r5, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #-1028]
	b	.L1295
.L1298:
	ldr	r1, .L1300
	movw	r3, #3940
	movw	ip, #3966
	mov	r9, r6
	ldrh	r0, [r1, r3]
	ldrh	r1, [r1, ip]
	sub	r2, r0, #1
	ldr	ip, [r4, #-896]
	uxth	r3, r2
	rsb	r2, r1, r2
	str	r2, [sp, #4]
	b	.L1266
.L1272:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L1267
	ldr	ip, [ip, #0]
	mov	r6, r9
	cmp	ip, #0
	bne	.L1268
	cmp	r1, r0
	addne	r1, r1, #1
	ldrne	ip, .L1300
	movwne	r0, #3966
	strneh	r1, [ip, r0]	@ movhi
.L1268:
	sxth	ip, r3
	mov	r0, #0
	str	ip, [sp, #20]
	ldr	r9, .L1300
	str	r3, [sp, #4]
	b	.L1269
.L1270:
	ldr	r7, [r4, #-896]
	add	ip, r1, #1
	add	r0, r0, #1
	ldr	r3, [r7, ip, asl #2]
	mov	ip, ip, asl #1
	uxth	r0, r0
	str	r3, [r7, r1, asl #2]
	mov	r1, r1, asl #1
	ldr	r7, [r9, #3968]
	ldrh	ip, [r7, ip]
	strh	ip, [r7, r1]	@ movhi
.L1269:
	ldr	ip, [sp, #20]
	sxth	r1, r0
	cmp	r1, ip
	bne	.L1270
	ldr	r1, [sl, #-896]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	ldr	r3, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, .L1300
	ldr	r1, [r1, #3968]
	strh	r6, [r1, r2]	@ movhi
	b	.L1271
.L1267:
	sub	r3, r3, #1
	uxth	r3, r3
.L1266:
	ldr	r6, [sp, #4]
	sxth	r2, r3
	cmp	r2, r6
	bgt	.L1272
	mov	r6, r9
.L1271:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L1253
	ldr	r2, .L1300
	movw	ip, #3940
	movw	r1, #3966
	ldrh	ip, [r2, ip]
	ldrh	r0, [r2, r1]
	sub	ip, ip, #1
	rsb	ip, r0, ip
	sxth	ip, ip
	cmp	r3, ip
	bgt	.L1253
	add	r0, r0, #1
	strh	r0, [r2, r1]	@ movhi
	ldr	r0, [r5, #4]
	ldr	r1, [r4, #-896]
	str	r0, [r1, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r2, #3968]
.L1295:
	strh	r6, [r2, r3]	@ movhi
	b	.L1253
.L1255:
	ldr	r3, .L1300+16
	movw	ip, #65535
	ldrh	r2, [r4, r3]
	cmp	r2, ip
	streqh	r6, [r4, r3]	@ movhi
	beq	.L1296
	ldr	r3, .L1300+20
	movw	r1, #65535
	ldrh	r0, [r4, r3]
	cmp	r0, r1
	beq	.L1274
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L1274:
	ldr	r3, [r5, #4]
	ldr	r2, [sl, #-1760]
	cmp	r2, r3
	ldr	r3, .L1300+20
	strcsh	r6, [r4, r3]	@ movhi
	bcs	.L1253
	ldr	r2, .L1300+16
	ldrh	r1, [r4, r2]
	strh	r6, [r4, r2]	@ movhi
	strh	r1, [r4, r3]	@ movhi
.L1296:
	ldr	r3, [r5, #4]
	str	r3, [r4, #-1760]
	b	.L1253
.L1299:
	mov	r0, r6
	b	.L1297
.L1250:
	ldr	r3, .L1300
	mov	r0, r6
	ldrb	r1, [r3, #3072]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L1276
.L1297:
	mov	r1, #0
.L1276:
	bl	FtlFreeSysBlkQueueIn
.L1253:
	ldr	r6, [sp, #12]
	add	r8, r8, #36
	add	r3, r6, #1
	uxth	r3, r3
	str	r3, [sp, #12]
	cmp	r3, fp
	bne	.L1277
.L1246:
	ldr	ip, [sp, #8]
	add	r3, ip, #1
	uxth	r3, r3
	str	r3, [sp, #8]
.L1243:
	ldr	r6, .L1300
	movw	r3, #3862
	ldr	ip, [sp, #8]
	ldrh	r3, [r6, r3]
	cmp	r3, ip
	bhi	.L1291
	ldr	r1, .L1300+4
	ldr	r2, [r1, #-1028]
	ldrh	r3, [r2, #0]
	cmp	r3, #0
	bne	.L1279
	ldr	r0, .L1300+8
	ldrh	r1, [r1, r0]
	cmp	r1, #0
	ldrne	r0, [r6, #3948]
	bne	.L1280
	b	.L1279
.L1284:
	mov	r1, r1, asl #1
	ldrh	r1, [r2, r1]
	cmp	r1, #0
	beq	.L1281
	ldr	r7, .L1300
	sxth	r6, r3
	ldr	r1, .L1300+4
	mov	r5, #0
	b	.L1282
.L1283:
	ldr	ip, [r1, #-1028]
	mov	r0, r2, asl #1
	rsb	r4, r6, r2
	add	r3, r3, #1
	ldrh	sl, [ip, r0]
	mov	r8, r4, asl #1
	uxth	r3, r3
	strh	sl, [ip, r8]	@ movhi
	ldr	ip, [r1, #-1024]
	ldr	r2, [ip, r2, asl #2]
	str	r2, [ip, r4, asl #2]
	ldr	r2, [r1, #-1028]
	strh	r5, [r2, r0]	@ movhi
.L1282:
	ldr	r0, [r7, #3948]
	sxth	r2, r3
	cmp	r2, r0
	bcc	.L1283
	b	.L1279
.L1281:
	add	r3, r3, #1
	uxth	r3, r3
.L1280:
	sxth	r1, r3
	cmp	r1, r0
	bcc	.L1284
.L1279:
	ldr	r2, .L1300
	ldr	r1, [r2, #3968]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L1285
	movw	r0, #3966
	ldrh	r0, [r2, r0]
	cmp	r0, #0
	movwne	r0, #3940
	ldrneh	r2, [r2, r0]
	bne	.L1286
	b	.L1285
.L1290:
	mov	ip, r0, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, #0
	beq	.L1287
	ldr	r1, .L1300
	movw	r8, #3940
	ldr	r7, .L1300+4
	mov	r6, #0
	b	.L1288
.L1289:
	ldr	r4, [r1, #3968]
	mov	ip, r2, asl #1
	rsb	r5, r0, r2
	add	r3, r3, #1
	ldrh	r9, [r4, ip]
	mov	sl, r5, asl #1
	uxth	r3, r3
	strh	r9, [r4, sl]	@ movhi
	ldr	r4, [r7, #-896]
	ldr	r2, [r4, r2, asl #2]
	str	r2, [r4, r5, asl #2]
	ldr	r2, [r1, #3968]
	strh	r6, [r2, ip]	@ movhi
.L1288:
	ldrh	ip, [r1, r8]
	sxth	r2, r3
	cmp	r2, ip
	blt	.L1289
	b	.L1285
.L1287:
	add	r3, r3, #1
	uxth	r3, r3
.L1286:
	sxth	r0, r3
	cmp	r0, r2
	blt	.L1290
.L1285:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1301:
	.align	2
.L1300:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1032
	.word	.LANCHOR2-1768
	.word	-1768
	.word	-1764
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #1
	ldr	r3, .L1313
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	moveq	r2, #3920
	movwne	r2, #3918
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
	bne	.L1306
	b	.L1311
.L1309:
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
	bne	.L1307
	ldr	r3, [sp, #44]
	cmn	r3, #1
	bne	.L1307
	ldr	r3, [sp, #4]
	cmn	r3, #1
	subne	r4, r6, #1
	uxthne	r4, r4
	bne	.L1311
.L1307:
	add	r6, r6, #1
	uxth	r8, r6
.L1311:
	sxth	r6, r8
	sxth	r3, r4
	cmp	r6, r3
	ble	.L1309
.L1306:
	sxth	r0, r4
	add	sp, sp, #108
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1314:
	.align	2
.L1313:
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
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r1, #0
	ldr	r4, .L1335
	ldr	r6, .L1335+4
	ldr	r7, .L1335+8
	ldr	r3, [r4, #-948]
	ldr	r0, [r4, #-2012]
	str	r3, [r4, #200]
	ldr	r3, [r4, #-924]
	str	r3, [r4, #204]
	movw	r3, #3860
	ldrh	r2, [r6, r3]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldrh	r0, [r4, r7]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1332
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	ldr	r3, .L1335+12
	ldrh	r7, [r4, r7]
	uxth	r5, r0
	add	r2, r5, #1
	strh	r2, [r4, r3]	@ movhi
	b	.L1334
.L1323:
	sxth	r8, r7
	mov	r1, #1
	ldr	r0, .L1335+16
	mov	r2, r1
	orr	r3, r3, r8, asl #10
	str	r3, [r4, #196]
	ldr	r3, [r4, #-948]
	str	r3, [r4, #200]
	bl	FlashReadPages
	ldrb	r3, [r6, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1318
	ldr	r3, [r4, #204]
	ldr	sl, [r3, #12]
	cmp	sl, #0
	beq	.L1318
	movw	r3, #3928
	ldr	r0, [r4, #200]
	ldrh	r1, [r6, r3]
	bl	js_hash
	cmp	sl, r0
	beq	.L1318
	cmp	r5, #0
	bne	.L1319
	ldr	r3, .L1335+20
	ldrh	r3, [r4, r3]
	cmp	r8, r3
	uxthne	r7, r3
	movne	r3, #3920
	ldrneh	r5, [r6, r3]
	bne	.L1321
.L1319:
	mvn	r3, #0
	str	r3, [r4, #192]
.L1318:
	ldr	r3, [r4, #192]
	cmn	r3, #1
	beq	.L1321
	ldr	r3, [r4, #-948]
	ldr	r2, [r3, #0]
	ldr	r3, .L1335+24
	cmp	r2, r3
	bne	.L1321
	ldr	r3, [r4, #-924]
	ldrh	r2, [r3, #0]
	movw	r3, #61604
	cmp	r2, r3
	beq	.L1322
.L1321:
	sub	r5, r5, #1
	uxth	r5, r5
.L1334:
	sxth	r3, r5
	cmp	r3, #0
	bge	.L1323
.L1322:
	ldr	r4, .L1335
	mov	r2, #48
	ldr	r5, .L1335+4
	movw	r6, #3860
	ldr	r0, .L1335+28
	ldr	r7, [r4, #200]
	mov	r1, r7
	bl	memcpy
	ldrh	r2, [r5, r6]
	add	r1, r7, #48
	ldr	r0, [r4, #-2012]
	mov	r2, r2, asl #1
	bl	memcpy
	ldrh	r1, [r5, r6]
	ldr	r3, [r4, #200]
	ldr	r0, [r4, #-1836]
	mov	r2, r1, lsr #3
	add	r1, r1, #24
	add	r2, r2, #4
	mov	r1, r1, lsr #1
	add	r1, r3, r1, asl #2
	bl	memcpy
	movw	r3, #3964
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	beq	.L1324
	ldrh	r3, [r5, r6]
	movw	r2, #3956
	ldrh	r2, [r5, r2]
	ldr	r0, [r4, #-888]
	mov	r1, r3, lsr #3
	add	r1, r1, r3, asl #1
	ldr	r3, [r4, #200]
	add	r1, r1, #52
	mov	r2, r2, asl #2
	ubfx	r1, r1, #2, #14
	add	r1, r3, r1, asl #2
	bl	memcpy
.L1324:
	ldr	r4, .L1335
	ldr	r3, .L1335+24
	ldr	r2, [r4, #-2068]
	cmp	r2, r3
	bne	.L1332
	ldr	r3, .L1335+32
	movw	r1, #3874
	ldrb	r0, [r4, #-2058]	@ zero_extendqisi2
	ldrh	r2, [r4, r3]
	ldr	r3, .L1335+36
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, .L1335+4
	ldrh	r1, [r3, r1]
	cmp	r0, r1
	bne	.L1332
	movw	r1, #3918
	movw	r0, #3924
	ldrh	r1, [r3, r1]
	movw	r5, #65535
	ldrh	r0, [r3, r0]
	str	r2, [r4, #228]
	mul	r1, r2, r1
	str	r1, [r3, #3980]
	mul	r1, r0, r1
	ldr	r0, [r3, #3864]
	str	r1, [r3, #3960]
	movw	r1, #3990
	ldrh	r1, [r3, r1]
	rsb	r0, r1, r0
	rsb	r0, r2, r0
	movw	r2, #3852
	ldrh	r1, [r3, r2]
	bl	__aeabi_uidiv
	ldr	r3, .L1335+40
	ldr	r1, .L1335+44
	strh	r0, [r4, r3]	@ movhi
	ldr	r3, .L1335+48
	ldrh	r2, [r4, r3]
	add	r3, r3, #62
	strh	r2, [r4, r3]	@ movhi
	sub	r3, r3, #60
	ldrh	r3, [r4, r3]
	mov	r0, r3, lsr #6
	and	r3, r3, #63
	strb	r3, [r4, #-1986]
	ldrb	r3, [r4, #-2057]	@ zero_extendqisi2
	strh	r0, [r4, r1]	@ movhi
	mvn	r1, #0
	ldr	r0, .L1335+52
	strb	r3, [r4, #-1984]
	ldr	r3, .L1335+56
	strh	r1, [r4, r3]	@ movhi
	mov	r3, #0
	ldr	r1, .L1335+60
	strh	r3, [r4, r0]	@ movhi
	strb	r3, [r4, #-1746]
	ldrh	r0, [r4, r1]
	add	r1, r1, #106
	strb	r3, [r4, #-1744]
	str	r3, [r4, #-1812]
	strh	r0, [r4, r1]	@ movhi
	ldr	r1, .L1335+64
	ldr	r0, .L1335+68
	ldrh	r1, [r4, r1]
	mov	ip, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-1938]
	ldrb	r1, [r4, #-2056]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	strb	r1, [r4, #-1936]
	ldr	r1, .L1335+72
	ldrh	r0, [r4, r1]
	add	r1, r1, #150
	strh	r0, [r4, r1]	@ movhi
	sub	r1, r1, #148
	ldr	r0, .L1335+76
	ldrh	r1, [r4, r1]
	mov	ip, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-1890]
	ldrb	r1, [r4, #-2055]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	strb	r1, [r4, #-1888]
	str	r3, [r4, #-1824]
	ldr	r1, [r4, #-2036]
	str	r3, [r4, #-1832]
	str	r3, [r4, #-1816]
	str	r3, [r4, #-1788]
	str	r3, [r4, #-1780]
	str	r3, [r4, #-1820]
	ldr	r3, [r4, #-2028]
	str	r1, [r4, #-1792]
	ldr	r1, [r4, #-1800]
	cmp	r3, r1
	strhi	r3, [r4, #-1800]
	ldr	r4, .L1335
	ldr	r3, [r4, #-2032]
	ldr	r1, [r4, #-1796]
	cmp	r3, r1
	strhi	r3, [r4, #-1796]
	cmp	r2, r5
	beq	.L1327
	ldr	r0, .L1335+80
	bl	make_superblock
.L1327:
	ldr	r3, .L1335+84
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1328
	ldr	r0, .L1335+88
	bl	make_superblock
.L1328:
	ldr	r5, .L1335
	movw	r4, #65535
	ldr	r3, .L1335+92
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1329
	ldr	r0, .L1335+96
	bl	make_superblock
.L1329:
	ldr	r3, .L1335+56
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1333
	ldr	r0, .L1335+100
	bl	make_superblock
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1332:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1333:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1336:
	.align	2
.L1335:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1768
	.word	-1766
	.word	.LANCHOR2+192
	.word	-1764
	.word	1179929683
	.word	.LANCHOR2-2068
	.word	-2060
	.word	-1762
	.word	-1772
	.word	-1990
	.word	-2054
	.word	-1750
	.word	-1752
	.word	-2050
	.word	-2048
	.word	-1942
	.word	-2046
	.word	-1894
	.word	.LANCHOR2-1992
	.word	-1944
	.word	.LANCHOR2-1944
	.word	-1896
	.word	.LANCHOR2-1896
	.word	.LANCHOR2-1752
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
	ldr	r6, .L1353
	ldr	r8, .L1353+4
	ldr	r3, [r6, #-948]
	mov	r7, r6
	ldr	r4, [r6, #-924]
	str	r3, [r6, #200]
	str	r4, [r6, #204]
	bl	FtlBbtMemInit
	movw	r3, #3916
	ldrh	r5, [r8, r3]
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L1338
.L1342:
	mov	r1, #1
	mov	r3, r5, asl #10
	ldr	r0, .L1353+8
	mov	r2, r1
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldr	r3, [r6, #192]
	cmn	r3, #1
	bne	.L1339
	ldr	r3, [r6, #196]
	mov	r1, #1
	ldr	r0, .L1353+8
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r6, #196]
	bl	FlashReadPages
.L1339:
	ldr	r3, [r7, #192]
	cmn	r3, #1
	beq	.L1340
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1340
	ldr	r3, .L1353+4
	mov	r2, #3984
	strh	r5, [r3, r2]	@ movhi
	ldr	r2, [r4, #4]
	str	r2, [r3, #3992]
	movw	r2, #3988
	ldrh	r1, [r4, #8]
	strh	r1, [r3, r2]	@ movhi
	b	.L1341
.L1340:
	sub	r5, r5, #1
	uxth	r5, r5
.L1338:
	movw	r3, #3916
	ldrh	r3, [r8, r3]
	sub	r3, r3, #48
	cmp	r5, r3
	bgt	.L1342
.L1341:
	ldr	r5, .L1353+4
	mov	r8, #3984
	movw	r3, #65535
	ldrh	r2, [r5, r8]
	cmp	r2, r3
	beq	.L1352
	movw	r6, #3988
	ldrh	r2, [r5, r6]
	cmp	r2, r3
	beq	.L1344
	ldr	r7, .L1353
	mov	r1, #1
	mov	r2, r2, asl #10
	add	r0, r7, #192
	str	r2, [r7, #196]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r7, #192]
	cmn	r3, #1
	beq	.L1344
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1344
	ldr	r3, [r4, #4]
	ldr	r2, [r5, #3992]
	cmp	r3, r2
	strhi	r3, [r5, #3992]
	ldrhih	r2, [r5, r6]
	ldrhih	r3, [r4, #8]
	strhih	r2, [r5, r8]	@ movhi
	strhih	r3, [r5, r6]	@ movhi
.L1344:
	ldr	r8, .L1353+4
	mov	r3, #3984
	mov	r1, #1
	mov	r6, #0
	ldr	r5, .L1353
	ldrh	r0, [r8, r3]
	bl	FtlGetLastWrittenPage
	movw	r3, #3986
	add	r2, r0, #1
	mov	r7, r0
	strh	r2, [r8, r3]	@ movhi
	b	.L1345
.L1348:
	mov	r2, #3984
	mov	r1, #1
	ldrh	r2, [r8, r2]
	ldr	r0, .L1353+8
	orr	r3, r3, r2, asl #10
	str	r3, [r5, #196]
	ldr	r3, [r5, #-948]
	mov	r2, r1
	str	r3, [r5, #200]
	bl	FlashReadPages
	ldr	r3, [r5, #192]
	cmn	r3, #1
	beq	.L1346
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	beq	.L1347
.L1346:
	sub	r6, r6, #1
	uxth	r6, r6
.L1345:
	add	r3, r6, r7
	sxth	r3, r3
	cmp	r3, #0
	bge	.L1348
.L1347:
	ldr	r3, .L1353+4
	movw	r2, #3990
	ldrh	r1, [r4, #10]
	ldrh	r0, [r4, #12]
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	beq	.L1349
	ldr	r2, [r3, #3848]
	cmp	r0, r2
	beq	.L1349
	movw	r1, #3862
	ldrh	r3, [r3, r1]
	mov	r3, r3, lsr #2
	cmp	r2, r3
	bcs	.L1349
	cmp	r0, r3
	bcs	.L1349
	bl	FtlSysBlkNumInit
.L1349:
	ldr	r6, .L1353+12
	mov	r4, #0
	ldr	r8, .L1353+4
	movw	r7, #3874
	ldr	r5, .L1353
	b	.L1350
.L1351:
	ldr	r3, .L1353+16
	ldr	r1, [r5, #200]
	ldr	r0, [r6, #4]!
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #2
	mla	r1, r4, r2, r1
	bl	memcpy
	add	r4, r4, #1
.L1350:
	ldrh	r3, [r8, r7]
	cmp	r4, r3
	bcc	.L1351
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1352:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1354:
	.align	2
.L1353:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+192
	.word	.LANCHOR0+4008
	.word	-1016
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
	ldr	r3, .L1362
	movw	fp, #3874
	ldr	r7, .L1362+4
	ldr	r6, .L1362+8
	ldr	r2, [r3, #-948]
	ldr	r8, [r3, #-924]
	str	r2, [r3, #200]
	str	r8, [r3, #204]
	b	.L1356
.L1361:
	movw	r9, #3916
	mvn	r3, #0
	ldrh	r4, [r6, r9]
	strh	r3, [r7], #2	@ movhi
	add	r4, r4, r3
	ldr	sl, .L1362
	uxth	r4, r4
	b	.L1357
.L1360:
	mla	r3, r3, r5, r4
	mov	r1, #1
	ldr	r0, .L1362+12
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sl, #196]
	bl	FlashReadPages
	ldr	r3, [sl, #192]
	cmn	r3, #1
	beq	.L1358
	ldrh	r2, [r8, #0]
	movw	r3, #61664
	cmp	r2, r3
	streqh	r4, [r7, #-2]	@ movhi
	beq	.L1359
.L1358:
	sub	r4, r4, #1
	uxth	r4, r4
.L1357:
	ldrh	r3, [r6, r9]
	sub	r2, r3, #16
	cmp	r4, r2
	bgt	.L1360
.L1359:
	add	r5, r5, #1
.L1356:
	ldrh	r3, [r6, fp]
	cmp	r5, r3
	bcc	.L1361
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1363:
	.align	2
.L1362:
	.word	.LANCHOR2
	.word	.LANCHOR0+3996
	.word	.LANCHOR0
	.word	.LANCHOR2+192
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
	ldr	r3, .L1383
	mov	r6, r0
	mov	r8, #0
	ldr	r7, .L1383+4
	ldrb	fp, [r3, #3125]	@ zero_extendqisi2
	b	.L1365
.L1372:
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
	bcs	.L1367
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
	ldr	r1, [r7, #856]
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
	ldr	r1, [r7, #856]
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
.L1367:
	add	r8, r8, #1
	add	r6, r6, #36
.L1365:
	cmp	r8, sl
	bne	.L1372
	ldr	r3, [sp, #12]
	cmp	r3, #0
	movne	r6, #0
	ldrne	r5, .L1383+8
	bne	.L1373
	b	.L1374
.L1379:
	ldr	r3, [r4, #0]
	cmn	r3, #1
	bne	.L1375
	ldr	r1, [r4, #4]
	ldr	r0, .L1383+12
	bl	printk
	b	.L1376
.L1375:
	rsb	r3, r6, sl
	mov	r1, r9
	add	r2, sp, #20
	mov	r0, r4
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #24
	bl	LogAddr2PhyAddr
	ldr	r2, [r5, #232]
	mov	r3, #0
	mov	lr, r4
	add	ip, sp, #28
	str	r3, [r2, #0]
	ldr	r2, [r5, #236]
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
	ldr	r3, [r5, #232]
	str	r3, [sp, #36]
	ldr	r3, [r5, #236]
	str	r3, [sp, #40]
	bl	FlashReadPages
	ldr	r7, [sp, #28]
	cmn	r7, #1
	bne	.L1377
	ldr	r0, .L1383+16
	ldr	r1, [r4, #4]
	bl	printk
	str	r7, [r4, #0]
.L1377:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1378
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #236]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1378
	ldr	r0, .L1383+20
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L1378:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L1376
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #232]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1376
	ldr	r0, .L1383+24
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L1376:
	add	r6, r6, #1
	add	r4, r4, #36
.L1373:
	cmp	r6, sl
	bne	.L1379
.L1374:
	mov	r0, #0
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1384:
	.align	2
.L1383:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC108
	.word	.LC109
	.word	.LC110
	.word	.LC111
	.fnend
	.size	FlashProgSlc2KPages, .-FlashProgSlc2KPages
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
	ldr	r5, .L1413
	mov	r9, r0
	str	r1, [sp, #8]
	mov	sl, r2
	str	r3, [sp, #16]
	ldr	ip, [r5, #3624]
	ldrb	r6, [r5, #852]	@ zero_extendqisi2
	ldrb	ip, [ip, #19]	@ zero_extendqisi2
	cmp	r6, #0
	moveq	fp, r5
	str	ip, [sp, #20]
	ldr	ip, .L1413+4
	ldrb	ip, [ip, #3125]	@ zero_extendqisi2
	str	ip, [sp, #12]
	beq	.L1386
	bl	FlashProgSlc2KPages
	b	.L1387
.L1400:
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
	bcs	.L1389
	ldrb	r1, [r5, #3841]	@ zero_extendqisi2
	add	r2, fp, r2, asl #4
	cmp	r1, #0
	ldr	r2, [r2, #3636]
	moveq	r8, #0
	cmp	r2, #0
	beq	.L1391
	cmp	r3, #1
	bne	.L1392
	ldr	r0, [r5, #3008]
	bl	NandcIqrWaitFlashReady
.L1392:
	ldrb	r0, [sp, #32]	@ zero_extendqisi2
	bl	FlashWaitCmdDone
.L1391:
	ldr	r2, [sp, #32]
	mov	r1, #0
	cmp	r8, #0
	add	r3, r5, r2, asl #4
	movne	r0, #36
	str	r1, [r3, #3640]
	ldr	r1, [sp, #28]
	str	r7, [r3, #3636]
	str	r1, [r3, #3632]
	addne	r1, r6, #1
	mlane	r1, r0, r1, r9
	strne	r1, [r3, #3640]
	add	r3, fp, r2
	add	r2, fp, r2, asl #4
	ldrb	r4, [r3, #3764]	@ zero_extendqisi2
	ldrb	r3, [fp, #3762]	@ zero_extendqisi2
	cmp	r3, #1
	strb	r4, [r2, #3628]
	mov	r0, r4
	bne	.L1394
	bl	NandcWaitFlashReady
	b	.L1395
.L1394:
	bl	NandcFlashCs
	ldr	r3, [sp, #32]
	mov	r0, r4
	ldr	r1, [sp, #28]
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #3588]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	mov	r0, r4
	bl	NandcFlashDeCs
.L1395:
	ldr	r2, [sp, #20]
	sub	r3, r2, #1
	cmp	r3, #7
	bhi	.L1396
	add	r3, r5, r4
	ldrb	r3, [r3, #3756]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1396
	mov	r0, r4
	ldrb	r1, [r5, #1]	@ zero_extendqisi2
	ldr	r2, .L1413+8
	mov	r3, #0
	bl	HynixSetRRPara
.L1396:
	mov	r0, r4
	bl	NandcFlashCs
	cmp	sl, #1
	mov	r0, r4
	bne	.L1397
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1397
	bl	flash_enter_slc_mode
	b	.L1398
.L1397:
	bl	flash_exit_slc_mode
.L1398:
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
	beq	.L1399
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgDpFirstCmd
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #28]
	mov	r0, r4
	add	r3, r5, r3, asl #2
	ldr	r2, [r3, #3588]
	adds	r2, r2, #0
	movne	r2, #1
	bl	FlashWaitReadyEN
	ldr	r3, [sp, #28]
	ldr	r1, [r5, #856]
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
.L1399:
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgSecondCmd
	mov	r0, r4
	bl	NandcFlashDeCs
	add	r6, r6, r8
.L1389:
	add	r6, r6, #1
.L1386:
	ldr	r3, [sp, #8]
	cmp	r6, r3
	bcc	.L1400
	ldr	r5, .L1413
	mov	r4, #0
	ldr	r6, .L1413+12
	ldr	r0, [r5, #3008]
	bl	NandcIqrWaitFlashReady
	b	.L1401
.L1403:
	uxtb	r0, r4
	bl	FlashWaitCmdDone
	cmp	sl, #1
	bne	.L1402
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1402
	ldrb	r0, [r6, r4, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L1402:
	add	r4, r4, #1
.L1401:
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1403
	ldr	r2, [sp, #16]
	cmp	r2, #0
	ldreq	r0, [sp, #16]
	beq	.L1387
	mov	r5, #0
	ldr	r4, .L1413+16
	ldr	r6, [sp, #8]
	b	.L1404
.L1409:
	ldr	r3, [r9, #0]
	cmn	r3, #1
	bne	.L1405
	ldr	r1, [r9, #4]
	ldr	r0, .L1413+20
	bl	printk
	b	.L1406
.L1405:
	rsb	r3, r5, r6
	mov	r1, sl
	add	r2, sp, #28
	mov	r0, r9
	uxtb	r3, r3
	str	r3, [sp, #0]
	add	r3, sp, #32
	bl	LogAddr2PhyAddr
	ldr	r2, [r4, #232]
	mov	r3, #0
	mov	lr, r9
	add	ip, sp, #36
	str	r3, [r2, #0]
	ldr	r2, [r4, #236]
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
	ldr	r3, [r4, #232]
	str	r3, [sp, #44]
	ldr	r3, [r4, #236]
	str	r3, [sp, #48]
	bl	FlashReadPages
	ldr	r7, [sp, #36]
	cmn	r7, #1
	bne	.L1407
	ldr	r0, .L1413+24
	ldr	r1, [r9, #4]
	bl	printk
	str	r7, [r9, #0]
.L1407:
	ldr	r3, [r9, #12]
	cmp	r3, #0
	beq	.L1408
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #236]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1408
	ldr	r0, .L1413+28
	ldr	r1, [r9, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r9, #0]
.L1408:
	ldr	r3, [r9, #8]
	cmp	r3, #0
	beq	.L1406
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #232]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1406
	ldr	r0, .L1413+32
	ldr	r1, [r9, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r9, #0]
.L1406:
	add	r5, r5, #1
	add	r9, r9, #36
.L1404:
	cmp	r5, r6
	bne	.L1409
	mov	r0, #0
.L1387:
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1414:
	.align	2
.L1413:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+4
	.word	.LANCHOR0+3628
	.word	.LANCHOR2
	.word	.LC108
	.word	.LC109
	.word	.LC110
	.word	.LC111
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.type	FtlVpcTblFlush.part.16, %function
FtlVpcTblFlush.part.16:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r1, #255
	ldr	r4, .L1427
	movw	r7, #3860
	ldr	r6, .L1427+4
	ldr	r3, [r4, #-948]
	ldr	r5, [r4, #-924]
	str	r3, [r4, #200]
	ldr	r3, .L1427+8
	str	r5, [r4, #204]
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, .L1427+12
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #-1760]
	str	r3, [r5, #4]
	mov	r3, #0
	str	r3, [r5, #8]
	str	r3, [r5, #12]
	ldr	r3, .L1427+16
	str	r3, [r4, #-2068]
	ldr	r3, .L1427+20
	str	r3, [r4, #-2064]
	ldr	r3, .L1427+24
	ldrh	r2, [r4, r3]
	ldr	r3, .L1427+28
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #3874
	ldrh	r3, [r6, r3]
	strb	r3, [r4, #-2058]
	ldr	r3, .L1427+32
	ldrh	r2, [r4, r3]
	sub	r3, r3, #62
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #64
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1986]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1427+36
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1984]	@ zero_extendqisi2
	strb	r3, [r4, #-2057]
	ldr	r3, .L1427+40
	ldrh	r2, [r4, r3]
	sub	r3, r3, #106
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #108
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1938]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1427+44
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1936]	@ zero_extendqisi2
	strb	r3, [r4, #-2056]
	ldr	r3, .L1427+48
	ldrh	r2, [r4, r3]
	sub	r3, r3, #150
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #152
	ldr	r0, [r4, #200]
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1890]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1427+52
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1888]	@ zero_extendqisi2
	strb	r3, [r4, #-2055]
	ldr	r3, [r4, #-1792]
	str	r3, [r4, #-2036]
	ldr	r3, [r4, #-1800]
	str	r3, [r4, #-2028]
	ldr	r3, [r4, #-1796]
	str	r3, [r4, #-2032]
	ldr	r3, .L1427+56
	ldrh	r2, [r4, r3]
	ldr	r3, .L1427+60
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #892
	ldrh	r2, [r4, r3]
	ldr	r3, .L1427+64
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #3928
	ldrh	r2, [r6, r3]
	bl	ftl_memset
	ldr	r1, .L1427+68
	mov	r2, #48
	ldr	r0, [r4, #200]
	bl	memcpy
	ldrh	r2, [r6, r7]
	ldr	r0, [r4, #200]
	ldr	r1, [r4, #-2012]
	mov	r2, r2, asl #1
	add	r0, r0, #48
	bl	memcpy
	ldrh	r2, [r6, r7]
	ldr	r3, [r4, #200]
	add	r0, r2, #24
	ldr	r1, [r4, #-1836]
	mov	r2, r2, lsr #3
	mov	r0, r0, lsr #1
	add	r2, r2, #4
	add	r0, r3, r0, asl #2
	bl	memcpy
	movw	r3, #3964
	ldrh	r3, [r6, r3]
	cmp	r3, #0
	beq	.L1416
	ldrh	r3, [r6, r7]
	ldr	r1, [r4, #200]
	mov	r0, r3, lsr #3
	add	r0, r0, r3, asl #1
	movw	r3, #3956
	add	r0, r0, #52
	ldrh	r2, [r6, r3]
	ubfx	r0, r0, #2, #14
	mov	r2, r2, asl #2
	add	r0, r1, r0, asl #2
	ldr	r1, [r4, #-1020]
	bl	memcpy
.L1416:
	mov	r0, #0
	ldr	r4, .L1427
	bl	FtlUpdateVaildLpn
	ldr	sl, .L1427+8
	ldr	r6, .L1427+72
	mov	r7, #0
	movw	r8, #65535
.L1426:
	ldr	r3, [r4, #-948]
	ldrh	r2, [r4, sl]
	ldr	r0, .L1427+4
	str	r3, [r4, #200]
	ldr	r3, [r4, #-924]
	str	r3, [r4, #204]
	ldrh	r3, [r4, r6]
	orr	r1, r3, r2, asl #10
	str	r1, [r4, #196]
	mov	r1, #3920
	ldrh	r1, [r0, r1]
	sub	r1, r1, #1
	cmp	r3, r1
	blt	.L1418
	ldr	r3, .L1427+76
	ldrh	r8, [r4, r3]
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, r6]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r4, #-1800]
	add	r2, r3, #1
	str	r2, [r4, #-1800]
	str	r3, [r4, #-1760]
	mov	r2, r0, asl #10
	strh	r0, [r4, sl]	@ movhi
	str	r2, [r4, #196]
	str	r3, [r5, #4]
	strh	r0, [r5, #2]	@ movhi
.L1418:
	mov	r1, #1
	ldr	r0, .L1427+80
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #192]
	add	r3, r3, #1
	cmn	r2, #1
	uxth	r3, r3
	strh	r3, [r4, r6]	@ movhi
	bne	.L1419
	cmp	r3, #1
	add	r7, r7, #1
	ldreq	r2, .L1427+4
	moveq	r3, #3920
	uxth	r7, r7
	ldreqh	r3, [r2, r3]
	subeq	r3, r3, #1
	streqh	r3, [r4, r6]	@ movhi
	cmp	r7, #3
	bls	.L1426
	ldr	r3, .L1427
	mov	r2, r7
	ldr	r0, .L1427+84
	ldr	r1, [r3, #196]
	bl	printk
	ldr	r3, .L1427+4
	mov	r2, #1
	str	r2, [r3, #4044]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1419:
	cmp	r3, #1
	beq	.L1426
	cmp	r2, #256
	beq	.L1426
	movw	r3, #65535
	cmp	r8, r3
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	mov	r0, r8
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1428:
	.align	2
.L1427:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1768
	.word	-3932
	.word	1179929683
	.word	1342177363
	.word	-1762
	.word	-2060
	.word	-1992
	.word	-2052
	.word	-1944
	.word	-2048
	.word	-1896
	.word	-2044
	.word	-1134
	.word	-2024
	.word	-2022
	.word	.LANCHOR2-2068
	.word	-1766
	.word	-1764
	.word	.LANCHOR2+192
	.word	.LC112
	.fnend
	.size	FtlVpcTblFlush.part.16, .-FtlVpcTblFlush.part.16
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L1431
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L1430
	bl	FtlVpcTblFlush.part.16
.L1430:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1432:
	.align	2
.L1431:
	.word	.LANCHOR0
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.section	.text.unlikely
	.align	2
	.type	FtlBbmTblFlush.part.18, %function
FtlBbmTblFlush.part.18:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r1, #0
	ldr	r7, .L1446
	mov	r5, #0
	ldr	sl, .L1446+4
	movw	r9, #3874
	ldr	r8, .L1446+8
	ldr	r3, [r7, #-924]
	ldr	r0, [r7, #-948]
	str	r3, [r7, #204]
	movw	r3, #3928
	str	r0, [r7, #200]
	ldrh	r2, [sl, r3]
	bl	ftl_memset
	b	.L1434
.L1435:
	ldr	r3, .L1446+12
	ldr	r1, [r8, #4]!
	ldrh	r2, [r7, r3]
	ldr	r3, [r7, #200]
	mul	r0, r2, r5
	mov	r2, r2, asl #2
	add	r5, r5, #1
	add	r0, r3, r0, asl #2
	bl	memcpy
.L1434:
	ldrh	r3, [sl, r9]
	ldr	r4, .L1446+4
	cmp	r5, r3
	ldr	r6, .L1446
	blt	.L1435
	ldr	r5, [r6, #204]
	mov	r1, #255
	mov	r2, #16
	mov	r7, #0
	mov	fp, r7
	mov	r0, r5
	bl	ftl_memset
	ldr	r3, .L1446+16
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #3992]
	str	r3, [r5, #4]
	mov	r3, #3984
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #2]	@ movhi
	movw	r3, #3988
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #8]	@ movhi
	movw	r3, #3990
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #10]	@ movhi
	ldr	r3, [r4, #3848]
	strh	r3, [r5, #12]	@ movhi
	b	.L1444
.L1441:
	mov	fp, #1
.L1444:
	ldr	r3, [r6, #-948]
	mov	r8, #3984
	ldr	r4, .L1446+4
	mov	r1, #0
	movw	sl, #3986
	str	r1, [r6, #192]
	str	r3, [r6, #200]
	movw	r9, #3988
	ldr	r3, [r6, #-924]
	ldrh	r1, [r4, r8]
	ldrh	r2, [r4, sl]
	str	r3, [r6, #204]
	ldrh	r0, [r5, #10]
	orr	r3, r2, r1, asl #10
	str	r3, [r6, #196]
	ldrh	r3, [r4, r9]
	str	r0, [sp, #0]
	ldr	r0, .L1446+20
	bl	printk
	mov	r3, #3920
	ldrh	r3, [r4, r3]
	ldrh	r2, [r4, sl]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L1437
	ldr	r3, [r4, #3992]
	mov	r1, #0	@ movhi
	ldrh	r2, [r4, r8]
	add	r3, r3, #1
	ldr	r0, [r4, #4048]
	str	r3, [r4, #3992]
	str	r3, [r5, #4]
	ldrh	r3, [r4, r9]
	strh	r1, [r4, sl]	@ movhi
	mov	r1, #1
	strh	r2, [r5, #8]	@ movhi
	strh	r2, [r4, r9]	@ movhi
	mov	r2, r1
	strh	r3, [r4, r8]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r6, #196]
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
.L1437:
	ldr	r4, .L1446
	mov	r1, #1
	ldr	r8, .L1446+4
	mov	r2, r1
	mov	r3, r1
	add	r0, r4, #192
	bl	FlashProgPages
	movw	r3, #3986
	ldrh	r2, [r8, r3]
	add	r2, r2, #1
	strh	r2, [r8, r3]	@ movhi
	ldr	r3, [r6, #192]
	cmn	r3, #1
	bne	.L1438
	add	r7, r7, #1
	ldr	r0, .L1446+24
	ldr	r1, [r6, #196]
	uxth	r7, r7
	bl	printk
	cmp	r7, #3
	bls	.L1444
	ldr	r0, .L1446+28
	mov	r2, r7
	ldr	r1, [r4, #196]
	bl	printk
	mov	r3, #1
	str	r3, [r8, #4044]
	b	.L1445
.L1438:
	cmp	fp, #0
	beq	.L1441
.L1445:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1447:
	.align	2
.L1446:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+4008
	.word	-1016
	.word	-3887
	.word	.LC113
	.word	.LC114
	.word	.LC115
	.fnend
	.size	FtlBbmTblFlush.part.18, .-FtlBbmTblFlush.part.18
	.text
	.align	2
	.global	FtlBbmTblFlush
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L1450
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L1449
	bl	FtlBbmTblFlush.part.18
.L1449:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1451:
	.align	2
.L1450:
	.word	.LANCHOR0
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
	ldr	r4, .L1462
	ldr	r3, .L1462+4
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	movne	r6, #0
	movne	sl, r4
	bne	.L1453
	b	.L1454
.L1460:
	add	r3, r3, r6
	mov	r1, r9
	mov	r5, #0
	ldr	r7, .L1462+4
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	b	.L1455
.L1459:
	add	r3, r4, r5, asl #1
	sub	r3, r3, #1120
	sub	r3, r3, #4
	ldrh	r3, [r3, #0]
	cmp	r3, r8
	bne	.L1456
	mov	r1, r8
	ldr	r0, .L1462+8
	bl	printk
	mov	r0, r8
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r1, [r4, r7]
	mov	r3, r5
	b	.L1457
.L1458:
	add	r0, r3, #1
	add	r3, r4, r3, asl #1
	sub	r3, r3, #1120
	add	r2, r4, r0, asl #1
	sub	r2, r2, #1120
	sub	r2, r2, #4
	ldrh	r2, [r2, #0]
	strh	r2, [r3, #-4]	@ movhi
	uxth	r3, r0
.L1457:
	cmp	r3, r1
	bcc	.L1458
	sub	r1, r1, #1
	strh	r1, [sl, r7]	@ movhi
.L1456:
	add	r5, r5, #1
	uxth	r5, r5
.L1455:
	ldrh	r3, [r4, r7]
	cmp	r3, r5
	bhi	.L1459
	add	r6, r6, #1
	uxth	r6, r6
.L1453:
	ldr	r3, .L1462+12
	movw	r2, #3852
	ldrh	r2, [r3, r2]
	cmp	r2, r6
	bhi	.L1460
	bl	FtlGcReFreshBadBlk
.L1454:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1463:
	.align	2
.L1462:
	.word	.LANCHOR2
	.word	-1126
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
	ldr	r3, .L1472
	mov	r4, r0
	ldr	r1, [r3, #-2012]
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	bne	.L1465
	ldr	r1, .L1472+4
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	mvneq	r2, #0
	streqh	r2, [r3, r1]	@ movhi
	beq	.L1467
	ldr	r1, .L1472+8
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1468
	ldr	r1, .L1472+12
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1468
	ldr	r1, .L1472+16
	ldrh	r3, [r3, r1]
	cmp	r3, r4
	beq	.L1468
.L1467:
	ldr	r5, .L1472
	mov	r1, r4
	sub	r0, r5, #2016
	bl	List_remove_node
	ldr	r3, .L1472+20
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	free_data_superblock
	mov	r0, r4
	bl	FtlGcFreeBadSuperBlk
	mov	r2, #1
	b	.L1468
.L1465:
	bl	List_update_data_list
	mov	r2, #0
.L1468:
	mov	r0, r2
	ldmfd	sp!, {r3, r4, r5, pc}
.L1473:
	.align	2
.L1472:
	.word	.LANCHOR2
	.word	-1752
	.word	-1992
	.word	-1944
	.word	-1896
	.word	-2004
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
	beq	.L1475
	ldr	r2, .L1480
	mov	r3, r0, asl #1
	ldr	r2, [r2, #-2012]
	ldrh	r1, [r2, r3]
	cmp	r1, #0
	subne	r1, r1, #1
	strneh	r1, [r2, r3]	@ movhi
	bne	.L1475
	ldmfd	sp!, {r4, r5, r6, lr}
	b	decrement_vpc_count.part.20
.L1475:
	ldr	r6, .L1480
	movw	r3, #65535
	ldr	r5, .L1480+4
	ldrh	r0, [r6, r5]
	cmp	r0, r3
	streqh	r4, [r6, r5]	@ movhi
	moveq	r0, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	cmp	r0, r4
	beq	.L1479
	bl	update_vpc_list
	strh	r4, [r6, r5]	@ movhi
	adds	r0, r0, #0
	movne	r0, #1
	ldmfd	sp!, {r4, r5, r6, pc}
.L1479:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1481:
	.align	2
.L1480:
	.word	.LANCHOR2
	.word	-1008
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
	ldr	r5, .L1494
	ldr	r7, .L1494+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1483
.L1484:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	r2, #3852
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
.L1483:
	cmp	r3, r6
	beq	.L1484
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L1486
	ldrb	r2, [r5, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1486
	ldrh	r2, [r4, #2]
	add	r2, r7, r2, asl #1
	sub	r2, r2, #880
	ldrh	r2, [r2, #0]
	cmp	r2, r6
	bne	.L1486
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1484
.L1486:
	ldrh	r6, [r4, #2]
	movw	r5, #65535
	mov	r7, r5
	orr	r6, r6, r3, asl #10
	ldrh	r3, [r4, #4]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
.L1491:
	ldr	r2, .L1494
	movw	r3, #3852
	ldrh	r2, [r2, r3]
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
.L1488:
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
	beq	.L1488
	strb	r3, [r4, #6]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1489
	ldr	r3, .L1494
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	ldrh	r2, [r4, #2]
	bne	.L1492
	ldr	r3, .L1494+4
	add	r2, r3, r2, asl #1
	sub	r2, r2, #880
	ldrh	r3, [r2, #0]
	cmp	r3, r7
	bne	.L1489
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1489
	sub	r3, r3, #1
	ldrh	r0, [r4, #0]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1491
.L1492:
	mov	r1, #3920
	ldrh	r1, [r3, r1]
	cmp	r2, r1
	bcc	.L1489
	ldr	r1, .L1494+4
	ldrh	r2, [r4, #0]
	ldrh	r0, [r4, #4]
	ldr	r1, [r1, #-2012]
	mov	r2, r2, asl #1
	ldrh	ip, [r1, r2]
	rsb	r0, r0, ip
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	strh	r2, [r4, #4]	@ movhi
	mov	r1, r2	@ movhi
	movw	r2, #3918
	ldrh	r3, [r3, r2]
	strb	r1, [r4, #6]
	strh	r3, [r4, #2]	@ movhi
.L1489:
	mov	r0, r6
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1495:
	.align	2
.L1494:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.type	FtlSlcSuperblockCheck.part.21, %function
FtlSlcSuperblockCheck.part.21:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	movw	r6, #65535
	ldr	r5, .L1502
	ldr	r7, .L1502+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1497
.L1498:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	r2, #3852
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
.L1497:
	cmp	r3, r6
	beq	.L1498
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L1500
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1500
	ldrh	r3, [r4, #2]
	add	r3, r7, r3, asl #1
	sub	r3, r3, #880
	ldrh	r3, [r3, #0]
	cmp	r3, r6
	bne	.L1500
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1498
	ldrh	r2, [r4, #2]
	strb	r3, [r4, #6]
	add	r2, r2, #1
	strh	r2, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1500:
	ldr	r3, .L1502
	ldrb	r1, [r3, #3072]	@ zero_extendqisi2
	cmp	r1, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	cmp	r2, #1
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r2, #3920
	ldrh	r1, [r4, #2]
	ldrh	r2, [r3, r2]
	cmp	r1, r2
	ldmccfd	sp!, {r3, r4, r5, r6, r7, pc}
	ldr	r1, .L1502+4
	ldrh	r2, [r4, #0]
	ldrh	r0, [r4, #4]
	ldr	r1, [r1, #-2012]
	mov	r2, r2, asl #1
	ldrh	ip, [r1, r2]
	rsb	r0, r0, ip
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	strh	r2, [r4, #4]	@ movhi
	mov	r1, r2	@ movhi
	movw	r2, #3918
	ldrh	r3, [r3, r2]
	strb	r1, [r4, #6]
	strh	r3, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1503:
	.align	2
.L1502:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlSlcSuperblockCheck.part.21, .-FtlSlcSuperblockCheck.part.21
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
	b	FtlSlcSuperblockCheck.part.21
	.fnend
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	2
	.type	allocate_data_superblock.part.22, %function
allocate_data_superblock.part.22:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r6, .L1545
	mov	r4, r0
	ldr	r5, .L1545+4
	mov	sl, r6
.L1540:
	ldr	r3, .L1545+8
	cmp	r4, r3
	bne	.L1508
	ldr	r3, .L1545+12
	ldrh	r2, [r5, r3]
	ldr	r3, [r5, #-1176]
	mov	r1, r2, lsr #1
	mul	ip, r3, r2
	add	r0, r1, #1
	add	r0, r0, ip, lsr #2
	ldr	ip, [r6, #3844]
	cmp	ip, #0
	uxth	r0, r0
	beq	.L1532
	ldr	ip, [r5, #-1776]
	cmp	ip, #39
	bhi	.L1532
	cmp	ip, #2
	bls	.L1537
	tst	r2, #1
	beq	.L1509
	cmp	r3, #0
	moveq	r1, r3
	beq	.L1510
	b	.L1509
.L1508:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1537
	mov	r3, #3872
	ldrh	r3, [r6, r3]
	cmp	r3, #1
	beq	.L1537
	ldrb	r3, [r6, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1537
	ldr	r3, .L1545+12
	ldr	r2, [r6, #3844]
	ldrh	r3, [r5, r3]
	cmp	r2, #0
	mov	r1, r3, lsr #3
	beq	.L1509
	ldr	r2, [r5, #-1776]
	cmp	r2, #1
	movls	r1, #7
	mulls	r1, r1, r3
	movls	r1, r1, lsr #3
	b	.L1509
.L1532:
	mov	r1, r0
.L1509:
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
	b	.L1510
.L1537:
	mov	r1, #0
.L1510:
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	ldr	r0, .L1545+16
	bl	List_pop_index_node
	ldr	r3, .L1545+12
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	movw	r3, #3860
	ldrh	r3, [r6, r3]
	uxth	r9, r0
	cmp	r3, r9
	bls	.L1540
	ldr	r3, [r5, #-2012]
	mov	r7, r9, asl #1
	ldrh	r8, [r3, r7]
	cmp	r8, #0
	bne	.L1540
	strh	r9, [r4, #0]	@ movhi
	mov	r0, r4
	bl	make_superblock
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1541
	movw	r3, #3852
	ldr	ip, [r6, #4048]
	ldrh	r3, [r6, r3]
	mov	r1, r4
	str	r4, [sp, #16]
	mov	r2, r8
	mov	r0, r8
	stmia	sp, {r4, ip}
	str	r3, [sp, #20]
	mov	r3, ip
	b	.L1513
.L1515:
	str	r0, [r3, #8]
	movw	r4, #65535
	str	r0, [r3, #12]
	ldrh	lr, [r1, #16]
	cmp	lr, r4
	beq	.L1514
	ldr	r4, [sp, #4]
	mov	ip, #36
	mov	lr, lr, asl #10
	mla	fp, ip, r8, r4
	add	r8, r8, #1
	uxth	r8, r8
	str	lr, [fp, #4]
.L1514:
	add	r2, r2, #1
	add	r3, r3, #36
	add	r1, r1, #2
	uxth	r2, r2
.L1513:
	ldr	fp, [sp, #20]
	cmp	r2, fp
	bne	.L1515
	ldr	r3, [r6, #3844]
	ldr	r4, [sp, #0]
	cmp	r3, #0
	beq	.L1516
	ldr	r3, .L1545+20
	cmp	r4, r3
	bne	.L1516
	ldr	r3, [r6, #4052]
	ldrh	r3, [r3, r7]
	cmp	r3, #40
	movhi	r3, #0
	strhib	r3, [r5, #-1984]
.L1516:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #0
	ldr	r3, [sl, #4052]
	ldrh	r2, [r3, r7]
	bne	.L1517
	cmp	r2, #0
	mov	r0, r9
	movwne	r1, #3910
	moveq	r2, #2
	ldrneh	r1, [r6, r1]
	addne	r2, r2, r1
	strh	r2, [r3, r7]	@ movhi
	ldr	r3, [r5, #-1792]
	mov	r1, #0
	add	r3, r3, #1
	str	r3, [r5, #-1792]
	b	.L1543
.L1517:
	add	r2, r2, #1
	strh	r2, [r3, r7]	@ movhi
	ldr	r3, [r5, #-1788]
	mov	r0, r9
	mov	r1, #1
	add	r3, r3, #1
	str	r3, [r5, #-1788]
.L1543:
	bl	ftl_set_blk_mode
	ldr	r3, [r6, #4052]
	ldr	fp, .L1545+4
	ldr	r2, [r5, #-1780]
	ldrh	r3, [r3, r7]
	ldr	r0, [fp, #-1792]
	cmp	r3, r2
	strhi	r3, [r5, #-1780]
	movw	r3, #3910
	ldrh	r2, [sl, r3]
	ldr	r3, [fp, #-1788]
	mla	r0, r0, r2, r3
	movw	r3, #3860
	ldrh	r1, [sl, r3]
	bl	__aeabi_uidiv
	ldr	r3, [fp, #-908]
	ldr	r2, [r3, #16]
	add	r2, r2, #1
	str	r2, [r3, #16]
	ldr	r3, [sl, #4048]
	mov	r2, #0
	str	r0, [fp, #-1784]
	b	.L1522
.L1523:
	add	r2, r2, #1
	ldr	r1, [r3, #-32]
	uxth	r2, r2
	bic	r1, r1, #1020
	bic	r1, r1, #3
	str	r1, [r3, #-32]
.L1522:
	cmp	r2, r8
	add	r3, r3, #36
	bne	.L1523
	ldrb	r3, [r6, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1524
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	mov	r2, r8
	ldr	r0, [r6, #4048]
	cmp	r3, #1
	moveq	r1, #0
	movne	r1, #1
	bl	FlashEraseBlocks
.L1524:
	mov	r2, r8
	ldr	r0, [sl, #4048]
	ldrb	r1, [r4, #8]	@ zero_extendqisi2
	bl	FlashEraseBlocks
	mov	r2, #36
	mul	r2, r2, r8
	mov	r8, #0
	mov	r3, r9
	mov	fp, r8
	mov	r9, r2
	mov	r2, r7
	mov	r7, r4
	b	.L1526
.L1528:
	ldr	r0, [r6, #4048]
	add	r1, r0, r8
	ldr	r4, [r0, r8]
	cmn	r4, #1
	bne	.L1527
	ldr	r0, [r1, #4]
	add	fp, fp, #1
	str	r2, [sp, #8]
	ubfx	r0, r0, #10, #16
	str	r3, [sp, #12]
	bl	FtlBbmMapBadBlock
	ldr	ip, [sp, #16]
	strh	r4, [ip, #16]	@ movhi
	ldrb	r1, [r7, #7]	@ zero_extendqisi2
	sub	r1, r1, #1
	strb	r1, [r7, #7]
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #8]
.L1527:
	ldr	r4, [sp, #16]
	add	r8, r8, #36
	add	r4, r4, #2
	str	r4, [sp, #16]
.L1526:
	cmp	r8, r9
	bne	.L1528
	cmp	fp, #0
	mov	r4, r7
	mov	r9, r3
	mov	r7, r2
	beq	.L1529
	mov	r0, r3
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1529:
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1530
.L1541:
	ldr	r3, [r5, #-2012]
	mvn	r2, #0
	strh	r2, [r3, r7]	@ movhi
	b	.L1540
.L1530:
	ldr	r1, .L1545
	movw	r2, #3918
	ldrh	r2, [r1, r2]
	strh	r9, [r4, #0]	@ movhi
	mul	r2, r2, r3
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	strb	r3, [r4, #6]
	ldr	r3, .L1545+4
	uxth	r2, r2
	strh	r2, [r4, #4]	@ movhi
	ldr	r1, [r3, #-1800]
	str	r1, [r4, #12]
	add	r1, r1, #1
	str	r1, [r3, #-1800]
	ldrh	r0, [r4, #0]
	ldr	r1, [r3, #-2012]
	mov	r3, r0, asl #1
	strh	r2, [r1, r3]	@ movhi
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1546:
	.align	2
.L1545:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2-1896
	.word	-1996
	.word	.LANCHOR2-2000
	.word	.LANCHOR2-1992
	.fnend
	.size	allocate_data_superblock.part.22, .-allocate_data_superblock.part.22
	.align	2
	.global	allocate_data_superblock
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L1549
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L1548
	bl	allocate_data_superblock.part.22
.L1548:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1550:
	.align	2
.L1549:
	.word	.LANCHOR0
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1561
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	ldr	r3, [r2, #4044]
	.pad #44
	sub	sp, sp, #44
	cmp	r3, #0
	bne	.L1551
	ldrb	r7, [r2, #3072]	@ zero_extendqisi2
	cmp	r7, #0
	beq	.L1560
	ldrb	r7, [r0, #8]	@ zero_extendqisi2
	cmp	r7, #1
	ldreqh	r8, [r0, #4]
	beq	.L1553
	mov	r7, r3
.L1560:
	mov	r8, #12
.L1553:
	ldr	r6, .L1561+4
	mvn	r3, #0
	str	r3, [sp, #20]
	mov	sl, #0
	movw	r2, #61589
	ldr	r5, [r6, #-924]
	ldr	r3, [r6, #-948]
	str	r5, [sp, #16]
	str	r3, [sp, #12]
	mvn	r3, #2
	str	r3, [r5, #8]
	mvn	r3, #1
	str	r3, [r5, #12]
	ldrh	r3, [r4, #0]
	strh	sl, [r5, #0]	@ movhi
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, [r6, #-948]
	str	r2, [r3, #0]
	ldr	r2, .L1561+8
	ldr	r3, [r6, #-948]
	str	r2, [r3, #4]
	b	.L1554
.L1557:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1555
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [sp, #8]
	beq	.L1555
	ldr	r3, [r6, #-1796]
	mov	r1, #1
	mov	r2, r7
	add	r0, sp, #4
	sub	r8, r8, #1
	str	r3, [r5, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, sl
	str	r3, [r6, #-1796]
	mov	r3, #0
	bl	FlashProgPages
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
.L1554:
	cmp	r8, #0
	bne	.L1557
.L1555:
	ldr	r2, .L1561+4
	ldrh	r3, [r4, #0]
	ldrh	r1, [r4, #4]
	ldr	r2, [r2, #-2012]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldr	r2, .L1561
	movw	r3, #3918
	ldrh	r3, [r2, r3]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
.L1551:
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1562:
	.align	2
.L1561:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	305419896
	.fnend
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r1
	ldr	r8, .L1593
	.pad #28
	sub	sp, sp, #28
	str	r0, [sp, #8]
	ldr	r6, [r8, #4044]
	ldrb	r1, [r8, #3841]	@ zero_extendqisi2
	cmp	r6, #0
	str	r1, [sp, #16]
	movne	r4, #0
	bne	.L1564
	ldrb	r2, [r8, #3072]	@ zero_extendqisi2
	mov	r5, r6
	ldr	fp, .L1593+4
	mov	r4, r6
	mov	r9, #36
	str	r2, [sp, #4]
	str	r0, [fp, #-972]
	b	.L1565
.L1569:
	mul	r3, r9, r6
	ldr	r2, [r8, #4048]
	mov	r1, #0
	str	r1, [r2, r3]
	add	r3, r8, r6
	ldr	r1, [sp, #8]
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	cmp	sl, #0
	mov	r7, r0
	beq	.L1566
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1567
.L1566:
	mov	r0, r7
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
	bne	.L1567
	movw	r1, #3930
	ldr	r3, [r8, #4048]
	ldrh	r2, [r8, r1]
	mov	r7, r7, asl #10
	mla	r3, r9, r5, r3
	mul	r2, r2, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	r1, r2, #3
	cmp	r2, #0
	str	r7, [r3, #4]
	movlt	r2, r1
	ldr	r1, [fp, #-920]
	bic	r2, r2, #3
	str	r0, [r3, #8]
	add	r2, r1, r2
	str	r2, [r3, #12]
.L1567:
	add	r6, r6, #1
	uxth	r6, r6
.L1565:
	movw	r2, #3852
	ldr	r7, .L1593
	ldrh	r3, [r8, r2]
	cmp	r3, r6
	bhi	.L1569
	cmp	r5, #0
	beq	.L1564
	ldr	r3, [sp, #4]
	mov	r8, #0
	ldr	r0, [r7, #4048]
	mov	r2, r5
	adds	r6, r3, #0
	strb	r8, [r7, #3841]
	mov	r9, r8
	movne	r6, #1
	mov	r1, r6
	bl	FlashEraseBlocks
	ldr	r1, [sp, #16]
	strb	r1, [r7, #3841]
.L1572:
	ldr	r3, [r7, #4048]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L1571
	ldr	r0, [r2, #4]
	add	r4, r4, #1
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
.L1571:
	add	r9, r9, #1
	add	r8, r8, #36
	uxth	r9, r9
	cmp	r9, r5
	bne	.L1572
	cmp	sl, #0
	beq	.L1586
	ldr	r3, .L1593
	mov	r2, #3920
	ldrh	r2, [r3, r2]
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	str	r2, [sp, #12]
	moveq	r2, r2, lsr #2
	streq	r2, [sp, #4]
	moveq	r6, #1
	beq	.L1573
	b	.L1592
.L1586:
	mov	r3, #6
	mov	r1, #1
	str	r3, [sp, #4]
	str	r1, [sp, #12]
	b	.L1573
.L1592:
	mov	r6, #1
	str	r6, [sp, #4]
.L1573:
	ldr	r7, .L1593
	mov	r8, #0
.L1581:
	mov	r9, #0
	ldr	fp, .L1593+4
	mov	r5, r9
	b	.L1574
.L1577:
	mov	r1, #36
	ldr	r2, [r7, #4048]
	mul	r3, r1, r9
	mov	r1, #0
	str	r1, [r2, r3]
	add	r3, r7, r9
	ldr	r1, [sp, #8]
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	cmp	sl, #0
	str	r0, [sp, #20]
	beq	.L1575
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1576
.L1575:
	ldr	r0, [sp, #20]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L1576
	ldr	r3, [r7, #4048]
	mov	r2, #36
	ldr	r1, [sp, #20]
	mla	r3, r2, r5, r3
	add	r2, r8, r1, asl #10
	movw	r1, #3930
	str	r2, [r3, #4]
	ldr	r2, [fp, #-932]
	str	r2, [r3, #8]
	ldrh	r2, [r7, r1]
	mul	r2, r2, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	r1, r2, #3
	cmp	r2, #0
	movlt	r2, r1
	ldr	r1, [fp, #-928]
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r3, #12]
.L1576:
	add	r9, r9, #1
	uxth	r9, r9
.L1574:
	movw	r2, #3852
	ldr	ip, .L1593
	ldrh	r3, [r7, r2]
	cmp	r3, r9
	bhi	.L1577
	cmp	r5, #0
	beq	.L1564
	ldr	r0, [ip, #4048]
	mov	r3, #0
	mov	r1, r5
	strb	r3, [ip, #3841]
	mov	r2, r6
	mov	r3, #1
	str	ip, [sp, #0]
	bl	FlashProgPages
	ldr	ip, [sp, #0]
	mov	r9, #0
	ldr	r1, [sp, #16]
	mov	fp, r9
	strb	r1, [ip, #3841]
.L1580:
	ldr	r3, [r7, #4048]
	add	r2, r3, r9
	ldr	r3, [r3, r9]
	cmp	r3, #0
	beq	.L1579
	ldr	r0, [r2, #4]
	add	r4, r4, #1
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
.L1579:
	add	fp, fp, #1
	add	r9, r9, #36
	uxth	fp, fp
	cmp	fp, r5
	bne	.L1580
	ldr	r2, [sp, #4]
	ldr	r3, [sp, #12]
	add	r8, r8, r2
	uxth	r8, r8
	cmp	r8, r3
	bcc	.L1581
	ldr	r9, .L1593
	mov	r7, #0
	mov	r8, r7
.L1583:
	cmp	sl, #0
	beq	.L1582
	ldr	r3, [r9, #4048]
	add	r2, r3, r7
	ldr	r3, [r3, r7]
	cmp	r3, #0
	bne	.L1582
	ldr	r0, [r2, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L1582:
	add	r8, r8, #1
	add	r7, r7, #36
	uxth	r8, r8
	cmp	r8, r5
	bne	.L1583
	ldr	r1, [sp, #8]
	cmp	r1, #63
	movhi	r3, #0
	movls	r3, #1
	cmp	sl, #0
	moveq	sl, r3
	orrne	sl, r3, #1
	cmp	sl, #0
	beq	.L1564
	ldr	r3, .L1593
	mov	r1, r6
	mov	r2, r8
	ldr	r0, [r3, #4048]
	bl	FlashEraseBlocks
.L1564:
	mov	r0, r4
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1594:
	.align	2
.L1593:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.type	FlashTestBlk.part.26, %function
FlashTestBlk.part.26:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
.L1596:
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #108
	sub	sp, sp, #108
	ldr	r4, .L1598
	mov	r5, r0
	mov	r1, #165
	add	r0, sp, #40
	mov	r2, #32
	str	r0, [sp, #16]
	ldr	r3, [r4, #148]
	mov	r5, r5, asl #10
	str	r3, [sp, #12]
	bl	ftl_memset
	ldr	r0, [r4, #148]
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
.L1599:
	.align	2
.L1598:
	.word	.LANCHOR2
	.fnend
	.size	FlashTestBlk.part.26, .-FlashTestBlk.part.26
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1602
	ldr	r3, [r3, #168]
	cmp	r0, r3
	bcc	.L1601
	b	FlashTestBlk.part.26
.L1601:
	mov	r0, #0
	bx	lr
.L1603:
	.align	2
.L1602:
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
	ldr	r3, .L1632
	movw	r1, #3058
	mov	r2, #3056
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #76
	sub	sp, sp, #76
	ldrh	r2, [r3, r2]
	mov	r5, #0
	ldrh	fp, [r3, r1]
	mov	r1, #1
	ldr	r4, .L1632+4
	mul	fp, fp, r2
	ldr	r2, [r3, #3624]
	ldr	r0, [r4, #240]
	ldrb	r2, [r2, #24]	@ zero_extendqisi2
	str	r0, [sp, #28]
	ldr	r0, .L1632+8
	uxth	fp, fp
	str	r2, [sp, #12]
	ldr	r2, [r3, #856]
	ldrb	r3, [r3, #852]	@ zero_extendqisi2
	uxth	r2, r2
	cmp	r3, #1
	str	r2, [sp, #8]
	moveq	r3, r2, asl #1
	uxtheq	r3, r3
	streq	r3, [sp, #8]
	bl	printk
	ldr	r0, [r4, #240]
	mov	r2, #4096
	mov	r1, #0
	bl	ftl_memset
	sub	r3, fp, #1
	mov	r2, fp, lsr #4
	ldr	r4, .L1632
	uxth	r3, r3
	str	r2, [sp, #20]
	str	r3, [sp, #24]
	b	.L1606
.L1628:
	ldr	r3, .L1632+4
	add	r2, r3, r5, asl #1
	ldrh	r6, [r2, #244]
	cmp	r6, #0
	bne	.L1607
	movw	r2, #3064
	ldr	r0, [r3, #160]
	ldrh	r2, [r4, r2]
	mov	r1, r6
	mov	r8, r6
	mov	r7, r6
	add	sl, r4, r5, asl #2
	mov	r2, r2, asl #9
	bl	ftl_memset
	add	r3, r4, r5
	ldrb	r9, [r3, #3764]	@ zero_extendqisi2
	b	.L1608
.L1616:
	mvn	r3, #0
	strb	r3, [sp, #34]
	strb	r3, [sp, #35]
	ldr	r3, [sp, #12]
	tst	r3, #1
	beq	.L1609
	ldr	r3, [sl, #3588]
	add	r2, sp, #34
	mov	r0, r9
	add	r3, r6, r3
	str	r3, [sp, #4]
	mov	r1, r3
	bl	FlashReadSpare
	ldrb	r2, [r4, #852]	@ zero_extendqisi2
	ldr	r3, [sp, #4]
	cmp	r2, #1
	bne	.L1609
	ldr	r1, [r4, #856]
	add	r2, sp, #35
	mov	r0, r9
	add	r1, r3, r1
	bl	FlashReadSpare
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	ldrb	r2, [sp, #35]	@ zero_extendqisi2
	and	r3, r2, r3
	strb	r3, [sp, #34]
.L1609:
	ldr	r0, [sp, #12]
	tst	r0, #2
	beq	.L1610
	ldr	r3, [r4, #3624]
	mov	r0, r9
	add	r2, sp, #35
	ldrh	r1, [r3, #10]
	ldr	r3, [sl, #3588]
	sub	r1, r1, #1
	add	r1, r1, r3
	add	r1, r1, r6
	bl	FlashReadSpare
.L1610:
	ldr	r3, [r4, #3624]
	ldrb	r3, [r3, #7]	@ zero_extendqisi2
	cmp	r3, #1
	cmpne	r3, #8
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	bne	.L1611
	cmp	r3, #0
	beq	.L1630
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	rsbs	r0, r0, #1
	movcc	r0, #0
	b	.L1612
.L1611:
	cmp	r3, #255
	bne	.L1630
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	subs	r0, r0, #255
	movne	r0, #1
	b	.L1612
.L1630:
	mov	r0, #1
.L1612:
	ldr	r2, [sp, #12]
	tst	r2, #4
	beq	.L1613
	ldr	r1, [sl, #3588]
	mov	r0, r9
	add	r1, r6, r1
	bl	SandiskProgTestBadBlock
.L1613:
	cmp	r0, #0
	beq	.L1614
	mov	r1, r5
	mov	r2, r8
	ldr	r0, .L1632+12
	add	r7, r7, #1
	bl	printk
	ldr	r3, [sp, #16]
	and	r0, r8, #31
	mov	ip, #1
	uxth	r7, r7
	mov	r2, r3, lsr #5
	ldr	r3, .L1632+4
	ldr	r3, [r3, #160]
	ldr	r1, [r3, r2, asl #2]
	orr	r1, r1, ip, asl r0
	ldr	r0, [sp, #20]
	str	r1, [r3, r2, asl #2]
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	mul	r3, r3, r0
	cmp	r7, r3
	bgt	.L1615
.L1614:
	ldr	r2, [sp, #8]
	add	r8, r8, #1
	add	r6, r6, r2
.L1608:
	uxth	r3, r8
	str	r3, [sp, #16]
	cmp	r3, fp
	bcc	.L1616
.L1615:
	ldr	r0, .L1632+16
	mov	r1, r5
	mov	r2, r7
	bl	printk
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	ldr	r0, [sp, #20]
	mul	r3, r3, r0
	cmp	r7, r3
	blt	.L1617
	movw	r3, #3064
	mov	r1, #0
	ldrh	r2, [r4, r3]
	ldr	r3, .L1632+4
	mov	r2, r2, asl #9
	ldr	r0, [r3, #160]
	bl	ftl_memset
.L1617:
	cmp	r5, #0
	bne	.L1618
	ldr	sl, .L1632+4
	mov	r8, r5
	mov	r7, r5
	ldrh	r9, [sl, #168]
	b	.L1619
.L1621:
	mov	r0, r6
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1620
	mov	r1, r6
	ldr	r0, .L1632+20
	bl	printk
	ldr	r3, [sl, #160]
	mov	r2, r6, lsr #5
	mov	r0, #1
	add	r7, r7, r0
	and	r6, r6, #31
	ldr	r1, [r3, r2, asl #2]
	uxth	r7, r7
	orr	r6, r1, r0, asl r6
	str	r6, [r3, r2, asl #2]
.L1620:
	add	r8, r8, #1
	uxth	r8, r8
.L1619:
	add	r6, r8, r9
	ldrb	r3, [r4, #853]	@ zero_extendqisi2
	uxth	r6, r6
	cmp	r3, r6
	bhi	.L1621
	ldr	r6, [sp, #24]
	sub	r9, fp, #50
	ldr	sl, .L1632+4
	mov	r8, #1
	b	.L1622
.L1624:
	mov	r0, r6
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1623
	mov	r1, r6
	ldr	r0, .L1632+20
	bl	printk
	ldr	r3, [sl, #160]
	mov	r2, r6, lsr #5
	and	r0, r6, #31
	ldr	r1, [r3, r2, asl #2]
	orr	r1, r1, r8, asl r0
	str	r1, [r3, r2, asl #2]
.L1623:
	sub	r6, r6, #1
	uxth	r6, r6
.L1622:
	cmp	r6, r9
	bgt	.L1624
	ldr	r3, .L1632+4
	ldrb	r1, [r4, #853]	@ zero_extendqisi2
	ldr	r2, [r3, #168]
	rsb	r2, r2, r1
	cmp	r7, r2
	bcc	.L1618
	movw	r2, #3064
	ldr	r0, [r3, #160]
	ldrh	r2, [r4, r2]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1618:
	mul	r8, fp, r5
	ldr	r7, .L1632+4
	ldr	r6, [sp, #24]
	ldr	r9, [sp, #28]
	add	sl, r7, r5, asl #1
.L1625:
	ldr	r0, .L1632+24
	mov	r1, r5
	mov	r2, r6
	bl	printk
	ldr	r3, [r7, #160]
	b	.L1626
.L1627:
	sub	r6, r6, #1
	uxth	r6, r6
.L1626:
	mov	r1, r6, lsr #5
	and	r2, r6, #31
	ldr	r1, [r3, r1, asl #2]
	mov	r2, r1, lsr r2
	ands	r2, r2, #1
	bne	.L1627
	ldr	r3, .L1632+28
	mov	r1, #1
	strh	r6, [sl, #244]	@ movhi
	add	r0, sp, #36
	strh	r6, [r9, #2]	@ movhi
	strh	r3, [r9, #0]	@ movhi
	ldr	r3, [r7, #160]
	strh	r2, [r9, #8]	@ movhi
	mov	r2, r1
	str	r3, [sp, #44]
	ldr	r3, [r7, #240]
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
	bne	.L1625
.L1607:
	add	r5, r5, #1
	uxtb	r5, r5
.L1606:
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	cmp	r3, r5
	bhi	.L1628
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1633:
	.align	2
.L1632:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC117
	.word	.LC118
	.word	.LC119
	.word	.LC120
	.word	.LC121
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
	ldr	sl, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	sub	r3, r3, #4
	cmp	r2, r3
	blt	.L1635
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r8, [r5, r0]
	cmp	r8, #0
	beq	.L1635
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L1635
	mov	r2, #1
	ldr	r1, .L1643
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	mov	r3, #3920
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1636
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1636:
	mov	r5, #0
	ldr	r6, .L1643+4
	b	.L1637
.L1642:
	ldr	r3, [sl, r5, asl #2]
	add	r9, sl, r5, asl #2
	cmp	r8, r3, lsr #10
	bne	.L1638
	ldr	r3, [r6, #-944]
	mov	r1, #1
	ldr	r7, [r6, #-924]
	mov	r2, r1
	ldr	r0, .L1643+8
	str	r3, [r6, #200]
	str	r7, [r6, #204]
	ldr	r3, [sl, r5, asl #2]
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldr	r3, [r6, #192]
	cmn	r3, #1
	beq	.L1639
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	bne	.L1639
	ldrh	r2, [r7, #0]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	beq	.L1640
.L1639:
	mov	r3, #0
	str	r3, [r9, #0]
	ldr	r3, .L1643+4
	ldrh	r2, [r7, #8]
	ldr	r0, .L1643+12
	ldr	r1, [r3, #196]
	bl	printk
	ldr	r3, .L1643
	mov	r2, #1
	str	r2, [r3, #4044]
	b	.L1641
.L1640:
	mov	r0, r4
	mov	r1, r5
	ldr	r2, [r6, #200]
	bl	FtlMapWritePage
.L1638:
	add	r5, r5, #1
	uxth	r5, r5
.L1637:
	ldrh	r3, [r4, #6]
	cmp	r3, r5
	bhi	.L1642
	mov	r0, r8
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L1635:
	ldr	r1, .L1643
	mov	r3, #3920
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1641
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1641:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1644:
	.align	2
.L1643:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+192
	.word	.LC122
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	ldr	r3, .L1652
	ldr	r5, [r0, #12]
	ldr	sl, [r0, #24]
	ldr	r7, [r3, #4044]
	cmp	r7, #0
	bne	.L1646
	ldrh	r2, [r0, #0]
	movw	r1, #65535
	cmp	r2, r1
	bne	.L1647
	ldrh	r3, [r0, #8]
	add	r3, r3, #1
	strh	r3, [r0, #8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	strh	r0, [r5, #0]	@ movhi
	ldr	r3, [r4, #28]
	strh	r7, [r4, #2]	@ movhi
	add	r3, r3, #1
	strh	r7, [r4, #0]	@ movhi
	str	r3, [r4, #28]
	b	.L1646
.L1647:
	mov	r2, r2, asl #1
	mov	r1, #255
	ldrh	r8, [r5, r2]
	ldr	r5, .L1652+4
	ldrh	r2, [r0, #2]
	orr	r2, r2, r8, asl #10
	str	r2, [r5, #196]
	ldr	r2, [r5, #-948]
	ldr	r6, [r5, #-924]
	str	r2, [r5, #200]
	ldr	r2, [r0, #28]
	str	r6, [r5, #204]
	str	r2, [r6, #4]
	ldr	r2, .L1652+8
	strh	r2, [r6, #8]	@ movhi
	ldrh	r2, [r0, #4]
	strh	r8, [r6, #2]	@ movhi
	strh	r2, [r6, #0]	@ movhi
	mov	r2, #3920
	ldrh	r2, [r3, r2]
	ldr	r0, [r5, #-948]
	mov	r2, r2, asl #3
	bl	ftl_memset
	mov	r3, r7
	b	.L1648
.L1650:
	ldr	r2, [sl, r3, asl #2]
	cmp	r8, r2, lsr #10
	bne	.L1649
	add	r7, r7, #1
	ldr	r2, [r5, #-948]
	uxth	r7, r7
	str	r3, [r2, r7, asl #3]
	ldr	r2, [r5, #-948]
	ldr	r1, [sl, r3, asl #2]
	add	r2, r2, r7, asl #3
	str	r1, [r2, #4]
.L1649:
	add	r3, r3, #1
	uxth	r3, r3
.L1648:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L1650
	ldr	r3, .L1652
	ldrb	r2, [r3, #852]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1651
	ldr	r2, .L1652+4
	ldr	r0, [r2, #200]
	movw	r2, #3928
	ldrh	r1, [r3, r2]
	bl	js_hash
	str	r0, [r6, #12]
.L1651:
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L1652+12
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L1646:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1653:
	.align	2
.L1652:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1291
	.word	.LANCHOR2+192
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
	ldr	r5, .L1669
	mov	sl, r1
	ldr	r6, .L1669+4
	mov	r9, r2
	mov	r7, #0
.L1668:
	ldr	r3, [r5, #-1816]
	add	r3, r3, #1
	str	r3, [r5, #-1816]
	mov	r3, #3920
	ldrh	r2, [r4, #2]
	ldrh	r3, [r6, r3]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L1656
	ldrh	r2, [r4, #0]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1657
.L1656:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L1657:
	ldr	r3, .L1669+4
	ldr	r1, [r3, #4044]
	cmp	r1, #0
	bne	.L1658
	ldrh	r3, [r4, #0]
	ldr	r2, [r4, #12]
	ldr	r0, [r5, #-924]
	mov	r3, r3, asl #1
	ldrh	r8, [r2, r3]
	mov	r2, #16
	ldrh	r3, [r4, #2]
	str	r9, [r5, #200]
	orr	r3, r3, r8, asl #10
	str	r0, [r5, #204]
	str	r3, [r5, #196]
	bl	ftl_memset
	ldr	r3, [r4, #28]
	ldr	fp, [r5, #204]
	str	r3, [fp, #4]
	strh	sl, [fp, #8]	@ movhi
	ldrh	r3, [r4, #4]
	strh	r8, [fp, #2]	@ movhi
	strh	r3, [fp, #0]	@ movhi
	ldrb	r3, [r6, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1659
	movw	r3, #3928
	ldr	r0, [r5, #200]
	ldrh	r1, [r6, r3]
	bl	js_hash
	str	r0, [fp, #12]
.L1659:
	mov	r1, #1
	ldr	r0, .L1669+8
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r2, [r4, #2]
	ldr	r3, .L1669
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r4, #2]	@ movhi
	ldr	r3, [r3, #192]
	cmn	r3, #1
	bne	.L1660
	ldr	r0, .L1669+12
	add	r7, r7, #1
	ldr	r1, [r5, #196]
	bl	printk
	ldrh	r3, [r4, #2]
	uxth	r7, r7
	cmp	r3, #2
	movls	r3, #3920
	ldrlsh	r3, [r6, r3]
	subls	r3, r3, #1
	strlsh	r3, [r4, #2]	@ movhi
	cmp	r7, #3
	bls	.L1668
	ldr	r3, .L1669
	mov	r2, r7
	ldr	r0, .L1669+16
	ldr	r1, [r3, #196]
	bl	printk
	ldr	r3, .L1669+4
	mov	r2, #1
	str	r2, [r3, #4044]
	b	.L1658
.L1660:
	cmp	r3, #0
	strneh	r8, [r4, #40]	@ movhi
	cmp	r2, #1
	beq	.L1664
	cmp	r3, #256
	beq	.L1664
	ldr	r3, [r4, #36]
	cmp	r3, #0
	beq	.L1665
.L1664:
	mov	r3, #0
	str	r3, [r4, #36]
	b	.L1668
.L1665:
	ldr	r3, .L1669
	ldr	r2, [r3, #196]
	ldr	r3, [r4, #24]
	str	r2, [r3, sl, asl #2]
.L1658:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1670:
	.align	2
.L1669:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+192
	.word	.LC123
	.word	.LC124
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
	ldr	r5, .L1672
	mul	r4, r4, r0
	ldr	r0, .L1672+4
	ldr	r2, [r5, #-1848]
	add	r3, r2, r4
	ldrh	r1, [r2, r4]
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #-1848]
	mov	r0, #0
	add	r4, r3, r4
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1673:
	.align	2
.L1672:
	.word	.LANCHOR2
	.word	.LANCHOR2-1076
	.fnend
	.size	flush_l2p_region, .-flush_l2p_region
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
	ldr	r3, [r0, #36]
	ldrh	r6, [r0, #6]
	cmp	r3, #0
	ldr	r2, [r0, #24]
	ldmeqfd	sp!, {r4, r5, r6, pc}
	mov	r3, #0
	str	r3, [r0, #36]
	ldr	r3, .L1678
	ldr	r1, [r3, #4044]
	cmp	r1, #0
	ldmnefd	sp!, {r4, r5, r6, pc}
	ldr	r5, .L1678+4
	sub	r6, r6, #1
	uxth	r6, r6
	ldr	r0, [r5, #-944]
	ldr	r1, [r5, #-924]
	str	r0, [r5, #200]
	str	r1, [r5, #204]
	ldr	r2, [r2, r6, asl #2]
	cmp	r2, #0
	str	r2, [r5, #196]
	beq	.L1676
	mov	r1, #1
	add	r0, r5, #192
	mov	r2, r1
	bl	FlashReadPages
	b	.L1677
.L1676:
	movw	r2, #3928
	mov	r1, #255
	ldrh	r2, [r3, r2]
	bl	ftl_memset
.L1677:
	mov	r0, r4
	mov	r1, r6
	ldr	r2, [r5, #200]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L1679:
	.align	2
.L1678:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1691
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r9, r2
	movw	r2, #3926
	mov	r5, r1
	ldrh	r7, [r3, r2]
	movw	r2, #3914
	ldrh	r2, [r3, r2]
	add	r1, r1, r0
	.pad #52
	sub	sp, sp, #52
	mov	r8, r0
	cmp	r1, r2
	mvnhi	fp, #0
	bhi	.L1681
	mov	r7, r0, lsr r7
	mov	fp, #0
	ldr	r6, .L1691+4
	mov	r2, r7, asl #2
	str	r2, [sp, #0]
	b	.L1682
.L1688:
	ldr	r3, [r6, #-892]
	mov	r0, r8
	ldr	r2, [sp, #0]
	ldr	sl, [r3, r2]
	movw	r3, #3924
	ldr	r2, .L1691
	ldrh	r4, [r2, r3]
	mov	r1, r4
	bl	__aeabi_uidivmod
	uxth	r1, r1
	str	r1, [sp, #4]
	rsb	r4, r1, r4
	uxth	r4, r4
	cmp	r4, r5
	uxthhi	r4, r5
	cmp	sl, #0
	beq	.L1684
	ldr	r2, .L1691+4
	mov	r1, #1
	add	r0, sp, #12
	str	sl, [sp, #16]
	ldr	r3, [r2, #-940]
	mov	r2, r1
	str	r3, [sp, #20]
	mov	r3, #0
	str	r3, [sp, #24]
	bl	FlashReadPages
	ldr	r3, [sp, #12]
	cmn	r3, #1
	moveq	fp, r3
	ldr	r3, [r6, #192]
	cmp	r3, #256
	bne	.L1686
	mov	r1, r7
	mov	r2, sl
	ldr	r0, .L1691+8
	bl	printk
	ldr	r0, .L1691+12
	mov	r1, r7
	ldr	r2, [r6, #-940]
	bl	FtlMapWritePage
.L1686:
	ldr	r3, [sp, #4]
	mov	r0, r9
	ldr	r1, [r6, #-940]
	mov	r2, r4, asl #9
	add	r1, r1, r3, asl #9
	bl	memcpy
	b	.L1687
.L1684:
	mov	r0, r9
	mov	r1, sl
	mov	r2, r4, asl #9
	bl	ftl_memset
.L1687:
	ldr	r3, [sp, #0]
	add	r7, r7, #1
	rsb	r5, r4, r5
	add	r8, r8, r4
	add	r9, r9, r4, asl #9
	add	r3, r3, #4
	str	r3, [sp, #0]
.L1682:
	cmp	r5, #0
	bne	.L1688
.L1681:
	mov	r0, fp
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1692:
	.align	2
.L1691:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC125
	.word	.LANCHOR2+260
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
	ldr	r5, .L1697
	mov	r1, #1
	ldr	r2, .L1697+4
	bl	FtlVendorPartRead
	ldr	r4, .L1697+8
	ldr	r3, [r5, #-1700]
	cmp	r3, r4
	beq	.L1694
	ldr	r0, .L1697+4
	mov	r1, #0
	mov	r2, #512
	bl	ftl_memset
	str	r4, [r5, #-1700]
.L1694:
	ldr	r2, [r5, #-1700]
	ldr	r3, .L1697
	cmp	r2, r4
	bne	.L1695
	ldr	r2, [r3, #-1612]
	ldr	r1, [r3, #-1668]
	str	r2, [r3, #-1808]
	ldr	r2, [r3, #-1608]
	str	r2, [r3, #-1804]
	ldr	r2, [r3, #-1692]
	str	r2, [r3, #-1812]
	ldr	r2, [r3, #-1688]
	str	r2, [r3, #-1824]
	ldr	r2, [r3, #-1684]
	str	r2, [r3, #-1832]
	ldr	r2, [r3, #-1680]
	str	r2, [r3, #-1816]
	ldr	r2, [r3, #-1672]
	str	r2, [r3, #-1788]
	ldr	r2, .L1697+12
	str	r1, [r2, #4056]
	ldr	r2, [r3, #-1664]
	str	r2, [r3, #-1828]
	ldr	r2, [r3, #-1660]
	str	r2, [r3, #-1820]
	ldr	r2, [r3, #-1656]
	str	r2, [r3, #-1780]
	ldr	r2, [r3, #-1652]
	str	r2, [r3, #-1776]
	ldr	r2, [r3, #-1640]
	str	r2, [r3, #-1704]
.L1695:
	ldr	r4, .L1697
	mov	r3, #0
	ldr	r5, .L1697+12
	str	r3, [r4, #-988]
	ldr	r2, [r4, #-1632]
	ldr	r3, .L1697+16
	cmp	r2, r3
	bne	.L1696
	mov	r3, #1
	ldr	r0, .L1697+20
	str	r3, [r5, #3844]
	ldr	r1, .L1697+24
	bl	printk
.L1696:
	movw	r3, #3910
	ldr	r0, [r4, #-1792]
	ldrh	r2, [r5, r3]
	ldr	r3, [r4, #-1788]
	mla	r0, r0, r2, r3
	movw	r3, #3860
	ldrh	r1, [r5, r3]
	bl	__aeabi_uidiv
	str	r0, [r4, #-1784]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1698:
	.align	2
.L1697:
	.word	.LANCHOR2
	.word	.LANCHOR2-1700
	.word	1179929683
	.word	.LANCHOR0
	.word	305432421
	.word	.LC77
	.word	.LC126
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
	ldr	r4, .L1701
	ldr	r5, .L1701+4
	ldr	r2, [r4, #-908]
	ldrh	r1, [r4, r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #-908]
	ldr	r2, [r3, #0]
	ldr	r3, .L1701+8
	cmp	r2, r3
	beq	.L1700
	ldr	r1, .L1701+12
	ldr	r0, .L1701+16
	bl	printk
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #-908]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1700:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1702:
	.align	2
.L1701:
	.word	.LANCHOR2
	.word	-912
	.word	1112818501
	.word	.LC127
	.word	.LC77
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1713
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r9, r2
	movw	r2, #3926
	mov	r4, r1
	ldrh	r8, [r3, r2]
	movw	r2, #3914
	ldrh	r2, [r3, r2]
	add	r1, r1, r0
	.pad #60
	sub	sp, sp, #60
	mov	r6, r0
	cmp	r1, r2
	mvnhi	r7, #0
	bhi	.L1704
	mov	r8, r0, lsr r8
	mov	r7, #0
	ldr	sl, .L1713+4
	mov	ip, r8, asl #2
	str	ip, [sp, #8]
	b	.L1705
.L1710:
	ldr	r1, .L1713
	movw	r2, #3924
	ldr	ip, [sp, #8]
	mov	r0, r6
	ldr	r3, [sl, #-892]
	ldrh	fp, [r1, r2]
	ldr	r3, [r3, ip]
	mov	r1, fp
	str	r3, [sp, #4]
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #4]
	ldr	r2, .L1713+4
	uxth	r1, r1
	str	r1, [sp, #12]
	rsb	r5, r1, fp
	uxth	r5, r5
	cmp	r5, r4
	uxthhi	r5, r4
	cmp	r3, #0
	beq	.L1707
	cmp	r5, fp
	beq	.L1707
	str	r3, [sp, #24]
	mov	r1, #1
	ldr	r3, [r2, #-940]
	add	r0, sp, #20
	mov	r2, r1
	str	r3, [sp, #28]
	mov	r3, #0
	str	r3, [sp, #32]
	bl	FlashReadPages
	b	.L1708
.L1707:
	ldr	ip, .L1713
	movw	r3, #3928
	ldr	r0, [r2, #-940]
	mov	r1, #0
	ldrh	r2, [ip, r3]
	bl	ftl_memset
.L1708:
	ldr	r3, [sp, #12]
	mov	fp, r5, asl #9
	ldr	r0, [sl, #-940]
	mov	r1, r9
	mov	r2, fp
	rsb	r4, r5, r4
	add	r0, r0, r3, asl #9
	add	r6, r6, r5
	bl	memcpy
	mov	r1, r8
	ldr	r0, .L1713+8
	add	r8, r8, #1
	ldr	r2, [sl, #-940]
	add	r9, r9, fp
	bl	FtlMapWritePage
	ldr	ip, [sp, #8]
	add	ip, ip, #4
	str	ip, [sp, #8]
	cmn	r0, #1
	moveq	r7, r0
.L1705:
	cmp	r4, #0
	bne	.L1710
.L1704:
	mov	r0, r7
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1714:
	.align	2
.L1713:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+260
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
	ldr	r3, .L1717
	ldr	r2, .L1717+4
	ldr	r1, [r3, #-1700]
	cmp	r1, r2
	bxne	lr
	ldr	r2, .L1717+8
	mov	r0, #0
	mov	r1, #1
	str	r2, [r3, #-1696]
	ldr	r2, [r3, #-1808]
	str	r2, [r3, #-1612]
	ldr	r2, [r3, #-1804]
	str	r2, [r3, #-1608]
	ldr	r2, [r3, #-1812]
	str	r2, [r3, #-1692]
	ldr	r2, [r3, #-1824]
	str	r2, [r3, #-1688]
	ldr	r2, [r3, #-1832]
	str	r2, [r3, #-1684]
	ldr	r2, [r3, #-1816]
	str	r2, [r3, #-1680]
	ldr	r2, [r3, #-1788]
	str	r2, [r3, #-1672]
	ldr	r2, .L1717+12
	ldr	r2, [r2, #4056]
	str	r2, [r3, #-1668]
	ldr	r2, [r3, #-1828]
	str	r2, [r3, #-1664]
	ldr	r2, [r3, #-1820]
	str	r2, [r3, #-1660]
	ldr	r2, [r3, #-1780]
	str	r2, [r3, #-1656]
	ldr	r2, [r3, #-1776]
	str	r2, [r3, #-1652]
	ldr	r2, [r3, #-1704]
	str	r2, [r3, #-1640]
	ldr	r2, [r3, #-988]
	str	r2, [r3, #-1636]
	ldr	r2, .L1717+16
	b	FtlVendorPartWrite
.L1718:
	.align	2
.L1717:
	.word	.LANCHOR2
	.word	1179929683
	.word	1342177363
	.word	.LANCHOR0
	.word	.LANCHOR2-1700
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
	ldr	r3, .L1726
	ldr	r1, .L1726+4
	ldr	r3, [r3, #3844]
	cmp	r3, #0
	moveq	r2, #32
	beq	.L1720
	ldr	r2, [r1, #-1776]
	cmp	r2, #39
	movls	r2, #4
	movhi	r2, #32
.L1720:
	mov	r3, #304
	ldrh	r1, [r1, r3]
	cmp	r1, #31
	addls	r1, r1, #1
	ldrls	r2, .L1726+4
	strlsh	r1, [r2, r3]	@ movhi
	movls	r2, #1
	cmp	r0, #0
	ldr	r3, .L1726+4
	bne	.L1722
	ldr	r1, [r3, #-908]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r2, r2, r0
	cmp	r1, r2
	bcc	.L1723
.L1722:
	ldr	r2, [r3, #-908]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L1726+8
	str	r1, [r2, #0]
	ldr	r1, .L1726+12
	ldr	r2, [r3, #-908]
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
.L1723:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1727:
	.align	2
.L1726:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	1112818501
	.word	-912
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
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
	ldr	r3, [r0, #12]
	mov	r4, r0
	ldrh	sl, [r0, #6]
	mov	r1, #0
	ldr	r9, [r0, #24]
	str	r3, [sp, #4]
	ldr	r3, [r0, #16]
	mov	r2, sl, asl #2
	ldr	r7, .L1750
	str	r3, [sp, #16]
	ldrh	r3, [r0, #8]
	mov	r0, r9
	str	r3, [sp, #8]
	bl	ftl_memset
	ldr	r3, .L1750+4
	ldr	r2, [r3, #-948]
	ldr	r5, [r3, #-924]
	str	r2, [r3, #200]
	mov	r2, #1
	str	r5, [r3, #204]
	mvn	r3, #0
	strh	r3, [r4, #0]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	str	r3, [r4, #32]
	str	r3, [r4, #28]
	str	r3, [sp, #0]
	ldr	r3, [sp, #8]
	str	r2, [r4, #36]
	sub	r3, r3, #1
	str	r3, [sp, #12]
	b	.L1729
.L1744:
	ldr	r1, [sp, #12]
	cmp	r3, r1
	mov	r3, r3, asl #1
	bne	.L1730
	ldr	r1, [sp, #4]
	mov	r7, #0
	ldr	r2, [sp, #4]
	ldr	r6, .L1750+4
	ldrh	r0, [r1, r3]
	mov	r1, #1
	add	r2, r2, r3
	str	r2, [sp, #8]
	bl	FtlGetLastWrittenPage
	ldr	r1, [sp, #12]
	ldr	r2, [sp, #16]
	uxth	r0, r0
	add	r3, r0, #1
	strh	r3, [r4, #2]	@ movhi
	ldr	r3, [sp, #0]
	sxth	r8, r0
	add	r8, r8, #1
	strh	r3, [r4, #0]	@ movhi
	ldr	r3, [r2, r1, asl #2]
	str	r3, [r4, #28]
	b	.L1731
.L1734:
	ldr	r1, [sp, #8]
	ldr	r0, .L1750+8
	ldrh	r2, [r1, #0]
	mov	r1, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldr	r3, .L1750
	ldrb	r2, [r3, #852]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1732
	ldr	r2, [r6, #204]
	ldr	fp, [r2, #12]
	cmp	fp, #0
	beq	.L1732
	movw	r2, #3928
	ldr	r0, [r6, #200]
	ldrh	r1, [r3, r2]
	bl	js_hash
	cmp	fp, r0
	mvnne	r3, #0
	strne	r3, [r6, #192]
.L1732:
	ldr	r3, [r6, #192]
	cmn	r3, #1
	beq	.L1733
	ldrh	r3, [r5, #8]
	cmp	r3, sl
	bcs	.L1733
	ldrh	r2, [r4, #4]
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	ldreq	r2, [r6, #196]
	streq	r2, [r9, r3, asl #2]
.L1733:
	add	r7, r7, #1
	uxth	r7, r7
.L1731:
	sxth	r3, r7
	cmp	r3, r8
	blt	.L1734
	b	.L1735
.L1730:
	ldr	r6, .L1750+4
	mov	r8, #3920
	ldr	r1, [sp, #4]
	add	r0, r6, #192
	ldr	r2, [r6, #-948]
	str	r2, [r6, #200]
	ldr	r2, [sp, #4]
	add	r2, r2, r3
	str	r2, [sp, #20]
	ldrh	r2, [r1, r3]
	mov	r1, #1
	ldrh	r3, [r7, r8]
	sub	r3, r3, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldr	r3, [r6, #192]
	cmn	r3, #1
	beq	.L1747
	ldrh	r2, [r5, #0]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L1747
	ldrh	r2, [r5, #8]
	movw	r3, #64245
	cmp	r2, r3
	bne	.L1747
	b	.L1748
.L1739:
	ldr	r1, [r6, #-948]
	mov	r0, r2, asl #3
	add	r3, r3, #1
	ldr	r2, [r1, r2, asl #3]
	uxth	r3, r3
	uxth	r2, r2
	cmp	r2, sl
	addcc	r1, r1, r0
	ldrcc	r1, [r1, #4]
	strcc	r1, [r9, r2, asl #2]
	b	.L1737
.L1748:
	mov	r3, #0
.L1737:
	ldrh	r1, [r7, r8]
	sxth	r2, r3
	sub	r1, r1, #1
	cmp	r2, r1
	blt	.L1739
	b	.L1740
.L1743:
	ldr	r1, [sp, #20]
	ldr	r0, .L1750+8
	ldrh	r2, [r1, #0]
	mov	r1, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldrb	r3, [r7, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1741
	ldr	r3, [r6, #204]
	ldr	fp, [r3, #12]
	cmp	fp, #0
	beq	.L1741
	movw	r3, #3928
	ldr	r0, [r6, #200]
	ldrh	r1, [r7, r3]
	bl	js_hash
	cmp	fp, r0
	mvnne	r3, #0
	strne	r3, [r6, #192]
.L1741:
	ldr	r3, [r6, #192]
	cmn	r3, #1
	beq	.L1742
	ldrh	r3, [r5, #8]
	cmp	r3, sl
	bcs	.L1742
	ldrh	r2, [r4, #4]
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	ldreq	r2, [r6, #196]
	streq	r2, [r9, r3, asl #2]
.L1742:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L1749
.L1747:
	ldr	r6, .L1750+4
	mov	r8, #0
.L1749:
	mov	r2, #3920
	sxth	r3, r8
	ldrh	r2, [r7, r2]
	cmp	r3, r2
	blt	.L1743
.L1740:
	ldr	r2, [sp, #0]
	add	r3, r2, #1
	uxth	r3, r3
	str	r3, [sp, #0]
.L1729:
	ldr	r1, [sp, #0]
	ldr	r2, [sp, #8]
	sxth	r3, r1
	cmp	r3, r2
	blt	.L1744
.L1735:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r1, .L1750
	mov	r3, #3920
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bne	.L1745
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1745:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1751:
	.align	2
.L1750:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+192
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
	ldr	r0, .L1753
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1754:
	.align	2
.L1753:
	.word	.LANCHOR2-1076
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
	movw	r2, #3940
	ldr	r3, .L1756
	ldr	r0, .L1756+4
	ldrh	r1, [r3, r2]
	movw	r2, #270
	strh	r1, [r0, r2]	@ movhi
	mov	r2, #264
	ldr	r1, .L1756+8
	strh	r1, [r0, r2]	@ movhi
	movw	r2, #3966
	ldrh	r1, [r3, r2]
	mov	r2, #268
	strh	r1, [r0, r2]	@ movhi
	movw	r2, #3942
	ldrh	r1, [r3, r2]
	movw	r2, #266
	ldr	r3, [r3, #3968]
	strh	r1, [r0, r2]	@ movhi
	str	r3, [r0, #272]
	ldr	r3, [r0, #-896]
	str	r3, [r0, #276]
	ldr	r3, [r0, #-900]
	str	r3, [r0, #280]
	ldr	r3, [r0, #-892]
	str	r3, [r0, #284]
	add	r0, r0, #260
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1757:
	.align	2
.L1756:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-3962
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FlashLoadFactorBbt
	.type	FlashLoadFactorBbt, %function
FlashLoadFactorBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r2, #3058
	ldr	r9, .L1767
	mov	r3, #3056
	ldr	r4, .L1767+4
	.pad #52
	sub	sp, sp, #52
	mov	r1, #0
	mvn	r7, #0
	ldrh	r3, [r9, r3]
	add	r0, r4, #244
	ldrh	r8, [r9, r2]
	mov	r2, #16
	mov	r6, #0
	mul	r8, r8, r3
	bl	ftl_memset
	ldr	r3, [r4, #240]
	str	r6, [sp, #20]
	mov	r4, r6
	str	r3, [sp, #24]
	uxth	r8, r8
	add	r3, r8, r7
	uxth	r3, r3
	str	r3, [sp, #4]
	b	.L1759
.L1765:
	ldr	r5, [sp, #4]
	sub	r3, r8, #12
	mul	fp, r8, r4
	ldr	sl, .L1767+4
	b	.L1760
.L1763:
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
	beq	.L1761
	ldr	r2, [sl, #240]
	ldrh	r1, [r2, #0]
	movw	r2, #61664
	cmp	r1, r2
	bne	.L1761
	ldr	r0, .L1767+8
	mov	r1, r4
	mov	r2, r5
	add	r6, r6, #1
	bl	printk
	ldr	r3, .L1767+4
	uxth	r6, r6
	add	r3, r3, r4, asl #1
	strh	r5, [r3, #244]	@ movhi
	b	.L1762
.L1761:
	sub	r5, r5, #1
	uxth	r5, r5
.L1760:
	cmp	r5, r3
	bgt	.L1763
.L1762:
	ldrb	r3, [r9, #3762]	@ zero_extendqisi2
	add	r4, r4, #1
	cmp	r3, r6
	uxtb	r4, r4
	moveq	r7, #0
.L1759:
	ldrb	r3, [r9, #3762]	@ zero_extendqisi2
	cmp	r3, r4
	bhi	.L1765
	mov	r0, r7
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1768:
	.align	2
.L1767:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC128
	.fnend
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.align	2
	.global	dump_map_info
	.type	dump_map_info, %function
dump_map_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r3, #3860
	ldr	r5, .L1782
	.pad #36
	sub	sp, sp, #36
	ldrh	r6, [r5, r3]
	b	.L1770
.L1772:
	add	r3, r5, r7
	mov	r1, r6
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #28]
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #28]
	cmp	r0, #0
	bne	.L1771
	ldr	r2, [r8, #-960]
	mov	r3, r3, asl #10
	mla	r2, r9, r4, r2
	str	r3, [r2, #4]
	ldr	r3, [r8, #-1164]
	str	r3, [r2, #8]
	ldrh	r3, [r5, sl]
	mul	r3, r3, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	r1, r3, #3
	cmp	r3, #0
	movlt	r3, r1
	ldr	r1, [r8, #-1160]
	bic	r3, r3, #3
	add	r3, r1, r3
	str	r3, [r2, #12]
.L1771:
	add	r7, r7, #1
	uxth	r7, r7
	b	.L1775
.L1780:
	ldr	r8, .L1782+4
	mov	r4, #0
	mov	r7, r4
	movw	fp, #3852
	mov	r9, #36
	movw	sl, #3930
.L1775:
	ldrh	r3, [r5, fp]
	cmp	r3, r7
	bhi	.L1772
	cmp	r4, #0
	beq	.L1773
	ldr	sl, .L1782+4
	mov	r1, r4
	mov	r2, #1
	mov	r7, #0
	mov	r8, r7
	ldr	r0, [sl, #-960]
	bl	FlashReadPages
.L1774:
	ldr	r1, [sl, #-960]
	add	r8, r8, #1
	add	r1, r1, r7
	uxth	r8, r8
	add	r7, r7, #36
	ldr	r3, [r1, #12]
	ldr	r2, [r1, #4]
	ldr	r1, [r1, #8]
	ldr	r0, [r3, #4]
	str	r0, [sp, #0]
	ldr	r0, [r3, #8]
	str	r0, [sp, #4]
	ldr	r0, [r3, #12]
	str	r0, [sp, #8]
	ldr	r0, [r1, #0]
	str	r0, [sp, #12]
	ldr	r1, [r1, #4]
	ldr	r0, .L1782+8
	str	r1, [sp, #16]
	ubfx	r1, r2, #10, #16
	ldr	r3, [r3, #0]
	bl	printk
	cmp	r8, r4
	bne	.L1774
.L1773:
	add	r6, r6, #1
	uxth	r6, r6
.L1770:
	movw	r2, #3862
	ldrh	r3, [r5, r2]
	cmp	r3, r6
	bhi	.L1780
	mov	r5, #0
	ldr	r4, .L1782+4
	ldr	sl, .L1782+12
	b	.L1776
.L1777:
	ldr	r2, [r4, #-1028]
	mov	r1, #1
	ldr	r0, .L1782+16
	add	r6, r6, #1
	ldrh	r2, [r2, r7]
	uxth	r6, r6
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r4, #196]
	bl	FlashReadPages
	ldr	r3, [r4, #204]
	ldr	r1, [r4, #-1028]
	ldr	r2, [r4, #200]
	ldr	r0, [r3, #0]
	ldrh	r1, [r1, r7]
	str	r0, [sp, #0]
	ldr	r0, [r3, #4]
	str	r0, [sp, #4]
	ldr	r0, [r3, #8]
	str	r0, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r0, .L1782+20
	str	r3, [sp, #12]
	ldr	r3, [r2, #0]
	str	r3, [sp, #16]
	ldr	r3, [r2, #4]
	str	r3, [sp, #20]
	ldr	r2, [r4, #192]
	ldr	r3, [r4, #196]
	bl	printk
.L1779:
	mov	r2, #3920
	sxth	r3, r6
	ldrh	r2, [r8, r2]
	cmp	r3, r2
	blt	.L1777
	add	r5, r5, #1
	uxth	r5, r5
.L1776:
	ldrh	r3, [r4, sl]
	sxth	r7, r5
	ldr	r6, .L1782+4
	cmp	r7, r3
	movlt	r7, r7, asl #1
	movlt	r6, #0
	ldrlt	r8, .L1782
	blt	.L1779
.L1778:
	ldr	r4, .L1782
	movw	r5, #3956
	ldr	r1, [r6, #-1028]
	mov	r2, #2
	ldr	r0, .L1782+24
	ldr	r3, [r4, #3948]
	bl	rknand_print_hex
	ldr	r1, [r6, #-1020]
	ldrh	r3, [r4, r5]
	mov	r2, #4
	ldr	r0, .L1782+28
	bl	rknand_print_hex
	ldr	r0, .L1782+32
	ldr	r1, [r6, #-888]
	mov	r2, #4
	ldrh	r3, [r4, r5]
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	b	rknand_print_hex
.L1783:
	.align	2
.L1782:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC129
	.word	-1032
	.word	.LANCHOR2+192
	.word	.LC107
	.word	.LC130
	.word	.LC131
	.word	.LC132
	.fnend
	.size	dump_map_info, .-dump_map_info
	.align	2
	.global	FtlDumpSysBlock
	.type	FtlDumpSysBlock, %function
FtlDumpSysBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r6, r0
	ldr	r4, .L1788
	.pad #24
	sub	sp, sp, #24
	mov	r7, r0, asl #10
	mov	r5, #0
	ldr	r8, .L1788+4
	ldr	r3, [r4, #-948]
	str	r3, [r4, #200]
	ldr	r3, [r4, #-924]
	str	r3, [r4, #204]
	b	.L1785
.L1787:
	mov	r1, #1
	ldr	r0, .L1788+8
	mov	r2, r1
	orr	r3, r3, r7
	str	r3, [r4, #196]
	bl	FlashReadPages
	ldr	r3, [r4, #204]
	mov	r1, r6
	ldr	r0, .L1788+12
	ldr	r2, [r3, #0]
	str	r2, [sp, #0]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	ldr	r2, [r3, #8]
	str	r2, [sp, #8]
	ldr	r3, [r3, #12]
	ldr	r2, [r4, #192]
	str	r3, [sp, #12]
	ldr	r3, [r4, #200]
	ldr	r3, [r3, #0]
	str	r3, [sp, #16]
	ldr	r3, [r4, #196]
	bl	printk
	ldr	r3, [r4, #204]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L1786
	ldr	r0, .L1788+16
	mov	r2, #4
	ldr	r1, [r4, #-948]
	mov	r3, #768
	bl	rknand_print_hex
.L1786:
	add	r5, r5, #1
	uxth	r5, r5
.L1785:
	mov	r2, #3920
	sxth	r3, r5
	ldrh	r2, [r8, r2]
	cmp	r3, r2
	blt	.L1787
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1789:
	.align	2
.L1788:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2+192
	.word	.LC133
	.word	.LC134
	.fnend
	.size	FtlDumpSysBlock, .-FtlDumpSysBlock
	.align	2
	.global	FlashReadFacBbtData
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1800
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r6, r1
	mov	r9, r2
	movw	r1, #3058
	mov	r2, #3056
	ldrh	r8, [r3, r1]
	ldrh	r2, [r3, r2]
	.pad #40
	sub	sp, sp, #40
	ldr	r7, .L1800+4
	mov	r4, r0
	mul	r8, r8, r2
	ldr	r3, [r7, #160]
	str	r3, [sp, #12]
	ldr	r3, [r7, #240]
	uxth	r8, r8
	sub	r5, r8, #1
	mul	sl, r8, r6
	str	r3, [sp, #16]
	uxth	r5, r5
	sub	r8, r8, #16
	b	.L1791
.L1797:
	mov	r1, #1
	add	r3, r5, sl
	add	r0, sp, #4
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L1792
	ldr	r3, [r7, #240]
	ldrh	r2, [r3, #0]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L1792
	cmp	r4, #0
	moveq	r0, r4
	beq	.L1793
	cmp	r6, #0
	ldreq	r3, .L1800+4
	moveq	r0, #1
	beq	.L1794
	b	.L1795
.L1796:
	ldr	r2, [r3, #160]
	ubfx	r1, r6, #5, #16
	and	lr, r6, #31
	add	r6, r6, #1
	ldr	ip, [r2, r1, asl #2]
	uxth	r6, r6
	orr	ip, ip, r0, asl lr
	str	ip, [r2, r1, asl #2]
.L1794:
	ldr	r2, [r3, #168]
	cmp	r6, r2
	bcc	.L1796
.L1795:
	ldr	r3, .L1800+4
	mov	r2, r9
	mov	r0, r4
	ldr	r1, [r3, #160]
	bl	memcpy
	mov	r2, #4
	ldr	r0, .L1800+8
	mov	r1, r4
	mov	r3, r2
	bl	rknand_print_hex
	mov	r0, #0
	b	.L1793
.L1792:
	sub	r5, r5, #1
	uxth	r5, r5
.L1791:
	cmp	r5, r8
	bgt	.L1797
	mvn	r0, #0
.L1793:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1801:
	.align	2
.L1800:
	.word	.LANCHOR0
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
	ldr	r3, .L1809
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r3, [r3, #3624]
	ldr	r6, .L1809+4
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	ldrh	r4, [r3, #14]
	ldr	r0, [r6, #148]
	mul	r4, r4, r2
	uxth	r4, r4
	add	r2, r4, #7
	mov	r2, r2, lsr #3
	bl	FlashReadFacBbtData
	cmn	r0, #1
	beq	.L1808
	mov	r2, #0
	mov	r0, r4, lsr #4
	mov	r3, r2
	sub	r4, r4, #1
	mov	r1, #1
	b	.L1804
.L1806:
	ldr	ip, [r6, #148]
	mov	r8, r2, lsr #5
	and	r7, r2, #31
	ldr	ip, [ip, r8, asl #2]
	ands	ip, ip, r1, asl r7
	movne	ip, r3, asl #1
	addne	r3, r3, #1
	uxthne	r3, r3
	strneh	r2, [r5, ip]	@ movhi
	cmp	r3, r0
	bcs	.L1808
	add	r2, r2, #1
	uxth	r2, r2
.L1804:
	cmp	r2, r4
	blt	.L1806
	b	.L1803
.L1808:
	mov	r3, #0
.L1803:
	mov	r3, r3, asl #1
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r5, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1810:
	.align	2
.L1809:
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
	ldr	r6, .L1829
	ldr	r5, [r6, #4044]
	cmp	r5, #0
	bne	.L1812
	add	r8, r6, #3984
	bl	FtlBbtMemInit
	add	sl, r8, #12
	bl	FtlLoadFactoryBbt
	ldr	r4, .L1829+4
	b	.L1813
.L1819:
	ldrh	r3, [sl], #2
	movw	r2, #65535
	ldr	r0, [r4, #-948]
	movw	fp, #3916
	ldr	r9, [r4, #-924]
	cmp	r3, r2
	str	r0, [r4, #200]
	str	r9, [r4, #204]
	beq	.L1814
	ldrh	r7, [r6, fp]
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1829+8
	mla	r7, r7, r5, r3
	mov	r3, r7, asl #10
	str	r3, [r4, #196]
	bl	FlashReadPages
	ldrh	r2, [r6, fp]
	ldr	r0, [r8, #28]
	add	r2, r2, #7
	ldr	r1, [r4, #200]
	mov	r2, r2, lsr #3
	bl	memcpy
	b	.L1815
.L1814:
	mov	r1, r5
	bl	FlashGetBadBlockList
	ldr	r0, [r4, #200]
	ldr	r1, [r8, #28]
	bl	FtlBbt2Bitmap
	ldrh	fp, [r6, fp]
.L1817:
	sub	fp, fp, #1
	uxth	fp, fp
.L1828:
	movw	r7, #3916
	ldrh	r0, [r6, r7]
	mla	r0, r0, r5, fp
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1817
	mov	r1, #0
	mov	r2, #16
	strh	fp, [sl, #-2]	@ movhi
	ldr	r0, [r4, #-924]
	bl	ftl_memset
	ldr	r3, .L1829+12
	strh	r3, [r9, #0]	@ movhi
	mov	r3, #0
	str	r3, [r9, #4]
	ldrh	r3, [sl, #-2]
	ldrh	r7, [r6, r7]
	strh	r3, [r9, #2]	@ movhi
	ldrh	r3, [sl, #-2]
	ldr	r1, [r8, #28]
	ldr	r0, [r4, #200]
	mla	r7, r7, r5, r3
	mov	r3, r7, asl #10
	str	r3, [r4, #196]
	ldr	r3, .L1829+16
	ldrh	r2, [r4, r3]
	mov	r2, r2, asl #2
	bl	memcpy
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1829+8
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L1829+8
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #192]
	cmn	r3, #1
	bne	.L1815
	uxth	r0, r7
	bl	FtlBbmMapBadBlock
	b	.L1828
.L1815:
	uxth	r0, r7
	add	r5, r5, #1
	bl	FtlBbmMapBadBlock
	add	r8, r8, #4
.L1813:
	movw	r3, #3874
	ldrh	r3, [r6, r3]
	cmp	r5, r3
	bcc	.L1819
	mov	r4, #0
	ldr	r7, .L1829
	movw	r6, #3932
	b	.L1820
.L1821:
	mov	r0, r4
	add	r4, r4, #1
	bl	FtlBbmMapBadBlock
	uxth	r4, r4
.L1820:
	ldrh	r3, [r7, r6]
	ldr	r5, .L1829
	cmp	r3, r4
	bhi	.L1821
	movw	r7, #3996
	movw	r6, #65535
	ldrh	r4, [r5, r7]
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1822
.L1827:
	mov	r0, r4
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1823
	mov	r0, r4
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1824
	mov	r0, r4
	bl	FtlBbmMapBadBlock
	b	.L1823
.L1824:
	mov	r3, #3984
	ldrh	r2, [r5, r3]
	cmp	r2, r6
	streqh	r4, [r5, r3]	@ movhi
.L1825:
	ldrne	r2, .L1829
	movwne	r3, #3988
	strneh	r4, [r2, r3]	@ movhi
	bne	.L1826
.L1823:
	sub	r4, r4, #1
	uxth	r4, r4
.L1822:
	ldrh	r3, [r5, r7]
	sub	r3, r3, #48
	cmp	r4, r3
	bgt	.L1827
.L1826:
	ldr	r4, .L1829
	mov	r6, #3984
	movw	r5, #3988
	mov	r7, #0
	movw	r8, #3986
	mov	r2, #2
	ldr	r0, [r4, #4048]
	mov	r1, #1
	ldrh	r3, [r4, r6]
	str	r7, [r4, #3992]
	strh	r7, [r4, r8]	@ movhi
	mov	r3, r3, asl #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, r5]
	mov	r3, r3, asl #10
	str	r3, [r0, #40]
	bl	FlashEraseBlocks
	ldrh	r0, [r4, r6]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, r5]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r4, #3992]
	ldrh	r2, [r4, r5]
	add	r3, r3, #1
	str	r3, [r4, #3992]
	ldrh	r3, [r4, r6]
	strh	r7, [r4, r8]	@ movhi
	strh	r2, [r4, r6]	@ movhi
	strh	r3, [r4, r5]	@ movhi
	bl	FtlBbmTblFlush
.L1812:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1830:
	.align	2
.L1829:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+192
	.word	-3872
	.word	-1016
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.type	ftl_load_l2p_region, %function
ftl_load_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, r0
	ldr	r4, .L1836
	mov	r6, #12
	ldr	r3, [r4, #-1020]
	ldr	r7, [r3, r0, asl #2]
	cmp	r7, #0
	bne	.L1832
	mul	r6, r6, r1
	ldr	r3, [r4, #-1848]
	ldr	r2, .L1836+4
	mov	r1, #255
	add	r3, r3, r6
	ldr	r0, [r3, #8]
	movw	r3, #3928
	ldrh	r2, [r2, r3]
	bl	ftl_memset
	ldr	r3, [r4, #-1848]
	strh	r5, [r3, r6]	@ movhi
	ldr	r3, [r4, #-1848]
	add	r6, r3, r6
	str	r7, [r6, #4]
	b	.L1833
.L1832:
	mul	r6, r6, r1
	ldr	r3, [r4, #-1848]
	mov	r1, #1
	add	r0, r4, #192
	mov	r2, r1
	str	r7, [r4, #196]
	add	r3, r3, r6
	ldr	r3, [r3, #8]
	str	r3, [r4, #200]
	ldr	r3, [r4, #-924]
	str	r3, [r4, #204]
	bl	FlashReadPages
	ldr	r3, [r4, #204]
	ldrh	r3, [r3, #8]
	cmp	r3, r5
	beq	.L1834
	mov	r2, r7
	mov	r1, r5
	ldr	r0, .L1836+8
	bl	printk
	ldr	r7, .L1836+4
	mov	r2, #4
	ldr	r0, .L1836+12
	mov	r3, r2
	ldr	r1, [r4, #204]
	bl	rknand_print_hex
	movw	r3, #3956
	ldrh	r3, [r7, r3]
	mov	r2, #4
	ldr	r0, .L1836+16
	ldr	r1, [r4, #-1020]
	bl	rknand_print_hex
	mov	r3, #1
	str	r3, [r7, #4044]
	b	.L1835
.L1834:
	ldr	r3, [r4, #192]
	cmp	r3, #256
	bne	.L1835
	mov	r1, r5
	mov	r2, r7
	ldr	r0, .L1836+20
	bl	printk
	ldr	r3, [r4, #-1848]
	ldr	r0, .L1836+24
	mov	r1, r5
	add	r3, r3, r6
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
.L1835:
	ldr	r3, .L1836
	mov	r1, #0
	ldr	r3, [r3, #-1848]
	add	r2, r3, r6
	str	r1, [r2, #4]
	strh	r5, [r3, r6]	@ movhi
.L1833:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1837:
	.align	2
.L1836:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC136
	.word	.LC99
	.word	.LC137
	.word	.LC138
	.word	.LANCHOR2-1076
	.fnend
	.size	ftl_load_l2p_region, .-ftl_load_l2p_region
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1851
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	sl, r2
	movw	r2, #3926
	mov	r4, r0
	ldrh	r8, [r3, r2]
	mov	r5, r1
	ldr	r2, [r3, #3980]
	add	r8, r8, #7
	mov	r7, r0, lsr r8
	cmp	r0, r2
	uxth	r7, r7
	bcs	.L1839
	movw	r2, #3958
	ldrh	r2, [r3, r2]
	ldr	r3, .L1851+4
	ldr	r9, [r3, #-1848]
	mov	r3, #0
	mov	r6, r3
	b	.L1840
.L1839:
	cmp	sl, #0
	mvn	r0, #0
	streq	r0, [r1, #0]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1846:
	add	r3, r3, #12
	add	r1, r9, r3
	ldrh	r1, [r1, #-12]
	cmp	r1, r7
	bne	.L1842
.L1843:
	mvn	r3, #0
	cmp	sl, #0
	bic	r4, r4, r3, asl r8
	mov	r2, #12
	ldr	r3, .L1851+4
	uxth	r4, r4
	bne	.L1844
	ldr	r3, [r3, #-1848]
	mla	r2, r2, r6, r3
	ldr	r3, [r2, #8]
	ldr	r3, [r3, r4, asl #2]
	str	r3, [r5, #0]
	b	.L1845
.L1844:
	mul	r2, r2, r6
	ldr	r1, [r3, #-1848]
	ldr	r0, [r5, #0]
	add	r1, r1, r2
	ldr	r1, [r1, #8]
	str	r0, [r1, r4, asl #2]
	ldr	r1, [r3, #-1848]
	add	r2, r1, r2
	ldr	r1, [r2, #4]
	orr	r1, r1, #-2147483648
	str	r1, [r2, #4]
	ldr	r2, .L1851+8
	strh	r7, [r3, r2]	@ movhi
.L1845:
	ldr	r3, .L1851+4
	mov	r2, #12
	ldr	r3, [r3, #-1848]
	mla	r6, r2, r6, r3
	ldr	r3, [r6, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r6, #4]
	movne	r0, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
	b	.L1850
.L1842:
	add	r6, r6, #1
	uxth	r6, r6
.L1840:
	cmp	r6, r2
	bne	.L1846
	bl	select_l2p_ram_region
	mov	r3, #12
	mul	r3, r3, r0
	mov	r6, r0
	add	r2, r9, r3
	ldrh	r1, [r9, r3]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1847
	ldr	r3, [r2, #4]
	cmp	r3, #0
	bge	.L1847
	bl	flush_l2p_region
.L1847:
	mov	r0, r7
	mov	r1, r6
	bl	ftl_load_l2p_region
	b	.L1843
.L1850:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1852:
	.align	2
.L1851:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1844
	.fnend
	.size	log2phys, .-log2phys
	.align	2
	.type	FtlReadRefresh.part.17, %function
FtlReadRefresh.part.17:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #40
	sub	sp, sp, #40
	ldr	r4, .L1861
	ldr	r6, .L1861+4
	ldr	r2, [r4, #-1616]
	ldr	r3, [r6, #3980]
	cmp	r2, r3
	bcs	.L1854
	mov	r5, #2048
.L1857:
	ldr	r0, [r4, #-1616]
	ldr	r3, [r6, #3980]
	cmp	r0, r3
	bcs	.L1859
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #0]
	ldr	r3, [r4, #-1616]
	cmn	r0, #1
	add	r3, r3, #1
	str	r3, [r4, #-1616]
	beq	.L1856
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
	bne	.L1859
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L1859
.L1856:
	subs	r5, r5, #1
	bne	.L1857
	b	.L1855
.L1854:
	ldr	r3, [r4, #-1832]
	mov	r0, #0
	str	r0, [r4, #-1620]
	str	r0, [r4, #-1616]
	str	r3, [r4, #-1624]
	b	.L1855
.L1859:
	mvn	r0, #0
.L1855:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, pc}
.L1862:
	.align	2
.L1861:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh.part.17, .-FtlReadRefresh.part.17
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1870
	ldr	r2, [r3, #-1620]
	cmp	r2, #0
	beq	.L1864
	b	FtlReadRefresh.part.17
.L1864:
	ldr	r2, .L1870+4
	ldr	r0, [r3, #-1780]
	ldr	r1, [r2, #3844]
	ldrb	r2, [r2, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	ldr	r2, [r3, #-1832]
	addeq	r0, r1, r0, lsr #10
	moveq	r1, #33554432
	movne	r0, #4194304
	moveq	r0, r1, asr r0
	ldr	r1, [r3, #-1624]
	add	r3, r2, #1048576
	cmp	r1, r3
	bhi	.L1866
	add	r1, r0, r1
	cmp	r1, r2
	bcc	.L1866
	ldr	r3, .L1870
	ldrb	r3, [r3, #-2040]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1867
.L1866:
	ldr	r3, .L1870
	mov	r1, #1
	str	r2, [r3, #-1624]
	str	r1, [r3, #-1620]
	mov	r1, #0
	str	r1, [r3, #-1616]
.L1867:
	mov	r0, #0
	bx	lr
.L1871:
	.align	2
.L1870:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1904
	movw	r3, #3224
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #68
	sub	sp, sp, #68
	ldrh	r6, [r2, r3]
	movw	r3, #65535
	mov	r4, r0
	str	r1, [sp, #20]
	cmp	r6, r3
	beq	.L1899
	cmp	r6, #0
	bne	.L1873
	b	.L1874
.L1899:
	mov	r6, #0
.L1873:
	ldr	r2, .L1904+4
	movw	r3, #3918
	ldr	sl, [sp, #20]
	ldrh	r3, [r2, r3]
	cmp	sl, r3
	bne	.L1875
.L1874:
	bl	FtlGcPageVarInit
.L1875:
	ldr	r5, .L1904+8
	mov	r9, #0
	mvn	r3, #0
	str	r3, [sp, #12]
.L1876:
	ldrh	r2, [r4, #0]
	movw	r3, #65535
	mov	sl, #0
	strb	sl, [r4, #8]
	cmp	r2, r3
	beq	.L1877
.L1878:
	str	r4, [sp, #4]
.L1900:
	ldr	r3, .L1904+4
	movw	r2, #3852
	ldr	r7, [r5, #-960]
	ldr	lr, [r5, #-1164]
	ldrh	r8, [r3, r2]
	add	r2, r2, #76
	ldr	fp, [r5, #-1160]
	ldrh	ip, [r3, r2]
	add	r2, r2, #2
	ldrh	r2, [r3, r2]
	mov	r3, #0
	mov	r4, r3
	str	r2, [sp, #8]
	ldr	r2, [sp, #4]
	b	.L1879
.L1881:
	ldrh	r0, [r2, #16]
	movw	r1, #65535
	cmp	r0, r1
	beq	.L1880
	mov	sl, #36
	orr	r0, r6, r0, asl #10
	mla	r1, sl, r4, r7
	str	r0, [r1, #4]
	mul	r0, ip, r4
	add	sl, r0, #3
	cmp	r0, #0
	movlt	r0, sl
	ldr	sl, [sp, #8]
	bic	r0, r0, #3
	add	r0, lr, r0
	str	r0, [r1, #8]
	mul	r0, sl, r4
	add	r4, r4, #1
	uxth	r4, r4
	add	sl, r0, #3
	cmp	r0, #0
	movlt	r0, sl
	bic	r0, r0, #3
	add	r0, fp, r0
	str	r0, [r1, #12]
.L1880:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1879:
	cmp	r3, r8
	bne	.L1881
	ldr	r0, [r5, #-960]
	mov	r1, r4
	mov	r2, #0
	mov	r7, #0
	bl	FlashReadPages
	mov	fp, r6
	str	r7, [sp, #8]
	b	.L1882
.L1895:
	ldr	r2, [r5, #-960]
	add	r3, r2, r7
	ldr	r8, [r2, r7]
	ldr	sl, [r3, #4]
	cmp	r8, #0
	str	sl, [sp, #16]
	bne	.L1883
	ldr	r6, [r3, #12]
	movw	r1, #65535
	ldrh	r3, [r6, #0]
	cmp	r3, r1
	beq	.L1884
	ldr	sl, .L1904+4
	ldr	r0, [r6, #8]
	ldr	r3, [sl, #3980]
	cmp	r0, r3
	bls	.L1885
.L1884:
	ldr	r4, [sp, #4]
	mov	sl, #0	@ movhi
	ldr	r2, .L1904+8
	ldrh	r3, [r4, #0]
	ldr	r2, [r2, #-2012]
	mov	r3, r3, asl #1
	strh	sl, [r2, r3]	@ movhi
	b	.L1903
.L1885:
	ldrb	r3, [sl, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1887
	mov	r2, r8
	add	r1, sp, #24
	bl	log2phys
	ldr	r3, [sp, #24]
	ldr	r2, [r6, #12]
	cmp	r3, r2
	bne	.L1887
	cmn	r3, #1
	beq	.L1887
	str	r3, [sp, #32]
	mov	r1, #1
	ldr	r3, [r5, #-928]
	mov	r2, r8
	add	r0, sp, #28
	str	r3, [sp, #36]
	ldr	r3, [r5, #-920]
	str	r3, [sp, #40]
	bl	FlashReadPages
	movw	r3, #3924
	ldr	r1, [r5, #-960]
	ldrh	r0, [sl, r3]
	mov	r3, r8
	add	r1, r1, r7
	ldr	r2, [sp, #36]
	mov	r0, r0, asl #7
	b	.L1888
.L1890:
	ldr	ip, [r1, #8]
	ldr	lr, [ip, r8]
	add	r8, r8, #4
	add	ip, r2, r8
	ldr	ip, [ip, #-4]
	cmp	lr, ip
	beq	.L1889
	ldr	r4, [sp, #4]
	ldr	r2, [sp, #32]
	ldr	r0, .L1904+12
	ldrh	r1, [r4, #0]
	bl	printk
	b	.L1894
.L1889:
	add	r3, r3, #1
.L1888:
	cmp	r3, r0
	bne	.L1890
.L1887:
	ldr	r0, [r6, #12]
	add	r7, r7, #36
	ldr	r1, [sp, #16]
	ldr	r2, [r6, #8]
	bl	FtlGcUpdatePage
	ldr	sl, [sp, #8]
	add	r3, sl, #1
	uxth	r3, r3
	str	r3, [sp, #8]
	b	.L1882
.L1883:
	ldr	r4, [sp, #4]
	ldr	r2, [sp, #16]
	ldr	r0, .L1904+16
	ldrh	r1, [r4, #0]
	bl	printk
	ldr	r3, .L1904+4
	ldr	r2, [r3, #3844]
	cmp	r2, #0
	bne	.L1891
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1892
.L1891:
	ldrh	r1, [r4, #0]
	ldr	r2, [r3, #4052]
	mov	r3, r1, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #159
	bls	.L1893
.L1892:
	ldr	r3, [r5, #-960]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	bne	.L1894
.L1893:
	ldr	r3, [r5, #-960]
	add	r7, r3, r7
	ldr	r7, [r7, #4]
	str	r7, [sp, #12]
.L1894:
	ldrh	r3, [r4, #0]
	mov	r1, #0	@ movhi
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
.L1903:
	ldrh	r0, [r4, #0]
	mov	r6, #0
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r4, #0]	@ movhi
	bl	FtlGcPageVarInit
	b	.L1876
.L1882:
	ldr	r3, [sp, #8]
	cmp	r3, r4
	bne	.L1895
	ldr	sl, [sp, #20]
	add	r9, r9, #1
	add	r6, fp, #1
	cmp	r9, sl
	uxth	r6, r6
	bcc	.L1896
	ldr	r2, .L1904
	movw	r3, #3224
	movw	r0, #65535
	ldrh	r1, [r2, r3]
	cmp	r1, r0
	beq	.L1896
	add	r1, r1, r9
	strh	r1, [r2, r3]	@ movhi
	ldr	r2, .L1904+4
	movw	r3, #3918
	ldrh	r3, [r2, r3]
	cmp	r3, r6
	bhi	.L1897
.L1896:
	ldr	r2, .L1904+4
	movw	r3, #3918
	ldrh	r3, [r2, r3]
	cmp	r3, r6
	bhi	.L1900
	ldr	r4, [sp, #4]
.L1877:
	ldr	r2, .L1904
	movw	r3, #3224
	mvn	r1, #0
	strh	r6, [r4, #2]	@ movhi
	mov	r0, r4
	strh	r1, [r2, r3]	@ movhi
	mov	r2, #0
	mov	r1, r6
	strb	r2, [r4, #6]
	bl	ftl_sb_update_avl_pages
.L1897:
	ldr	r0, [sp, #12]
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1905:
	.align	2
.L1904:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC139
	.word	.LC140
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
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
	ldr	r3, .L1912
	ldr	r1, [r3, #-2012]
	mov	r5, r0, asl #1
	ldrh	r2, [r1, r5]
	cmp	r2, #0
	addne	r2, r2, #1
	strneh	r2, [r1, r5]	@ movhi
	bne	.L1908
	ldr	r4, [r3, #-2000]
	cmp	r4, #0
	beq	.L1908
	ldr	r1, .L1912+4
	mov	ip, #6
	ldrh	lr, [r3, r1]
	ldr	r3, [r3, #-2020]
	ldr	r1, .L1912+8
	rsb	r4, r3, r4
	mov	r4, r4, asr #1
	mul	r4, r1, r4
	movw	r1, #65535
	uxth	r4, r4
	b	.L1909
.L1911:
	cmp	r4, r0
	bne	.L1910
	ldr	r6, .L1912
	mov	r1, r4
	sub	r0, r6, #2000
	bl	List_remove_node
	ldr	r3, .L1912+4
	mov	r0, r4
	ldrh	r2, [r6, r3]
	sub	r2, r2, #1
	strh	r2, [r6, r3]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r6, #-2012]
	ldrh	r2, [r3, r5]
	add	r2, r2, #1
	strh	r2, [r3, r5]	@ movhi
	b	.L1908
.L1910:
	mul	r4, ip, r4
	ldrh	r4, [r3, r4]
	cmp	r4, r1
	beq	.L1908
	add	r2, r2, #1
	uxth	r2, r2
.L1909:
	cmp	r2, lr
	bne	.L1911
.L1908:
	mov	r0, r7
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L1913:
	.align	2
.L1912:
	.word	.LANCHOR2
	.word	-1996
	.word	-1431655765
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	Ftlscanalldata
	.type	Ftlscanalldata, %function
Ftlscanalldata:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r1, #0
	ldr	r4, .L1920
	.pad #32
	sub	sp, sp, #32
	ldr	r0, .L1920+4
	mov	r5, #0
	bl	printk
	ldr	r7, .L1920+8
	mov	r6, r4
	b	.L1915
.L1919:
	mov	r0, r5
	add	r1, sp, #28
	mov	r2, #0
	bl	log2phys
	movs	r3, r5, asl #21
	bne	.L1916
	ldr	r0, .L1920+12
	mov	r1, r5
	ldr	r2, [sp, #28]
	bl	printk
.L1916:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L1917
	str	r3, [r4, #196]
	mov	r2, #0
	ldr	r3, [r4, #-948]
	mov	r1, #1
	ldr	r8, [r4, #-924]
	ldr	r0, .L1920+16
	str	r3, [r4, #200]
	str	r5, [r4, #208]
	str	r8, [r4, #204]
	str	r2, [r4, #192]
	bl	FlashReadPages
	ldr	r3, [r4, #192]
	cmn	r3, #1
	cmpne	r3, #256
	beq	.L1918
	ldr	r3, [r8, #8]
	cmp	r3, r5
	beq	.L1917
.L1918:
	ldr	r3, [r6, #204]
	ldr	r2, [r6, #200]
	ldr	r0, .L1920+20
	ldr	r1, [r3, #4]
	str	r1, [sp, #0]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2, #0]
	str	r1, [sp, #12]
	mov	r1, r5
	ldr	r2, [r2, #4]
	str	r2, [sp, #16]
	ldr	r2, [r6, #196]
	ldr	r3, [r3, #0]
	bl	printk
.L1917:
	add	r5, r5, #1
.L1915:
	ldr	r3, [r7, #3980]
	cmp	r5, r3
	bcc	.L1919
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1921:
	.align	2
.L1920:
	.word	.LANCHOR2
	.word	.LC141
	.word	.LANCHOR0
	.word	.LC142
	.word	.LANCHOR2+192
	.word	.LC143
	.fnend
	.size	Ftlscanalldata, .-Ftlscanalldata
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
	beq	.L1923
	ldrh	r3, [r0, #2]
	ldr	r1, .L2012
	ldrb	r6, [r0, #6]	@ zero_extendqisi2
	str	r3, [sp, #20]
	movw	r3, #3918
	ldrh	r3, [r1, r3]
	ldr	fp, [sp, #20]
	str	r6, [sp, #36]
	cmp	r3, fp
	mov	r3, #0
	streqh	r3, [r0, #4]	@ movhi
	moveq	r2, r3	@ movhi
	ldrneh	r0, [r0, #16]
	bne	.L1925
	b	.L2009
.L1926:
	add	r3, r3, #1
	uxth	r3, r3
	add	r1, r4, r3, asl #1
	ldrh	r0, [r1, #16]
.L1925:
	cmp	r0, r2
	beq	.L1926
	ldrb	r1, [r4, #8]	@ zero_extendqisi2
	cmp	r1, #1
	bne	.L1927
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1928
	ldr	r3, .L2012
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L2002
	add	r3, r3, r0, asl #1
	movw	r2, #3076
	ldrh	r8, [r3, r2]
	b	.L1994
.L1927:
	mov	r1, #0
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1928
.L2002:
	mov	r8, r7
.L1994:
	ldr	r3, .L2012
	movw	r2, #3852
	mov	r5, #0
	str	r4, [sp, #44]
	movw	ip, #65535
	ldrh	r9, [r3, r2]
	ldr	r2, .L2012+4
	ldr	sl, [r2, #-960]
	ldr	r6, [r2, #-1160]
	movw	r2, #3930
	ldrh	lr, [r3, r2]
	mov	r2, r4
	mov	r3, r5
	b	.L1930
.L1928:
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	mov	r2, r3	@ movhi
.L2009:
	strb	r2, [r4, #6]
	b	.L1923
.L1932:
	ldrh	r0, [r2, #16]
	cmp	r0, ip
	beq	.L1931
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
.L1931:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1930:
	cmp	r3, r9
	bne	.L1932
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	movne	r3, #0
	bne	.L2003
	ldr	r3, .L2012
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	adds	r3, r3, #0
	movne	r3, #1
.L2003:
	ldr	r6, .L2012+4
	mov	r1, r5
	str	r3, [sp, #32]
	mov	sl, #0
	ldr	r2, [sp, #32]
	ldr	r0, [r6, #-960]
	bl	FlashReadPages
	ldr	fp, [r6, #-1796]
	movw	r2, #65535
	str	r2, [sp, #12]
	str	fp, [sp, #16]
	b	.L1934
.L1939:
	mov	r3, #36
	ldr	r1, [r6, #-960]
	mul	r2, r3, sl
	add	r3, r1, r2
	ldr	r2, [r1, r2]
	cmp	r2, #0
	bne	.L1935
	ldr	r3, [r3, #12]
	ldr	fp, [r3, #4]
	cmn	fp, #1
	beq	.L1936
	ldr	r1, [r6, #-1796]
	mov	r0, fp
	str	r3, [sp, #4]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #4]
	cmp	r0, #0
	addne	fp, fp, #1
	strne	fp, [r6, #-1796]
.L1936:
	ldr	r3, [r3, #0]
	cmn	r3, #1
	bne	.L1938
	b	.L1937
.L1935:
	ldr	r1, [r3, #4]
	uxth	fp, r8
	ldr	r0, .L2012+8
	bl	printk
	ldrh	r3, [r4, #0]
	movw	lr, #306
	str	fp, [sp, #12]
	strh	r3, [r6, lr]	@ movhi
.L1938:
	add	sl, sl, #1
.L1934:
	uxth	r9, sl
	cmp	r9, r5
	bcc	.L1939
.L1937:
	ldr	r3, .L2012+4
	cmp	r9, r5
	addeq	r7, r7, #1
	uxthne	r7, r7
	movne	r2, #36
	ldrne	r3, [r3, #-960]
	uxtheq	r7, r7
	ldreq	r3, [r3, #-960]
	mlane	r9, r2, r9, r3
	ldreq	r0, [r3, #4]
	ldrne	r0, [r9, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	ldr	r3, .L2012
	cmp	r2, #1
	str	r0, [sp, #24]
	bne	.L1942
	ldrb	r1, [r3, #3072]	@ zero_extendqisi2
	cmp	r1, #0
	addeq	r7, r3, r7, asl #1
	movweq	r1, #3076
	ldreqh	r7, [r7, r1]
.L1942:
	movw	r1, #3918
	ldr	r6, [sp, #24]
	ldrh	r3, [r3, r1]
	ldr	fp, [sp, #36]
	cmp	r3, r7
	streqh	r7, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r4, #6]
	streqh	r3, [r4, #4]	@ movhi
	ldr	r3, [sp, #20]
	cmp	r7, r3
	cmpeq	r6, fp
	moveq	r0, r4
	moveq	r1, r7
	moveq	r2, r6
	beq	.L2010
	ldr	r3, [sp, #16]
	ldr	fp, [sp, #12]
	sub	r6, r3, #1
	movw	r3, #65535
	cmp	fp, r3
	bne	.L1945
	cmp	r2, #0
	bne	.L1946
.L1945:
	ldr	r2, .L2012+4
	uxth	r3, r8
	ldr	fp, [sp, #20]
	movw	ip, #65535
	mov	sl, r3
	ldr	r8, .L2012+4
	ldr	r1, [r2, #-996]
	cmn	r1, #1
	streq	r6, [r2, #-996]
	ldr	r2, [r2, #-996]
	str	r2, [sp, #12]
	add	r2, fp, #7
	cmp	r3, r2
	mvn	fp, #0
	subgt	r5, r3, #7
	mov	r9, fp
	mov	r3, r7
	ldrle	r5, [sp, #20]
	mov	r7, r6
	uxthgt	r5, r5
	mov	r6, r4
	b	.L1949
.L1951:
	ldrh	r0, [r1, #16]
	cmp	r0, ip
	beq	.L1950
	ldr	r6, [sp, #28]
	mov	r3, #36
	orr	r0, r5, r0, asl #10
	mla	lr, r3, r4, r6
	add	r4, r4, #1
	uxth	r4, r4
	str	r0, [lr, #4]
.L1950:
	add	r2, r2, #1
	add	r1, r1, #2
	uxth	r2, r2
.L1962:
	ldr	lr, [sp, #16]
	cmp	r2, lr
	bne	.L1951
	ldr	r3, [sp, #40]
	mov	r1, r4
	ldr	r0, [r8, #-960]
	ldr	r2, [sp, #32]
	stmib	sp, {r3, ip}
	ldr	r6, [sp, #0]
	bl	FlashReadPages
	ldr	r2, .L2012
	add	r0, r8, r5, asl #1
	mov	r1, #0
	str	r0, [sp, #28]
	ldmib	sp, {r3, ip}
	ldrb	r2, [r2, #3072]	@ zero_extendqisi2
	str	r2, [sp, #16]
	ldr	r2, [r8, #-960]
	b	.L1952
.L1960:
	ldr	r0, [r2, #0]
	cmp	r0, #0
	bne	.L1953
	ldr	r0, [r2, #12]
	ldrh	lr, [r0, #0]
	cmp	lr, ip
	beq	.L1954
	ldr	r0, [r0, #4]
	cmn	r0, #1
	beq	.L1954
	cmn	fp, #1
	ldr	r9, [r8, #-996]
	str	r0, [r8, #-996]
	bne	.L1954
	ldr	lr, [sp, #28]
	sub	r0, lr, #880
	ldrh	r0, [r0, #0]
	cmp	r0, ip
	bne	.L1955
	ldr	r0, [sp, #16]
	cmp	r0, #0
	beq	.L1954
.L1955:
	cmp	r9, r7
	mvneq	fp, #0
	movne	fp, r9
	b	.L1954
.L1953:
	mov	r4, r6
	mov	r6, r7
	ldrh	r1, [r4, #0]
	mov	r7, r3
	ldr	r3, .L2012+4
	movw	r2, #306
	strh	r1, [r3, r2]	@ movhi
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1946
	add	r5, r3, r5, asl #1
	movw	r2, #65535
	sub	r5, r5, #880
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	bne	.L1956
	cmn	fp, #1
	bne	.L2005
.L1957:
	ldr	fp, [sp, #12]
	cmp	fp, r6
	beq	.L1958
.L2005:
	str	fp, [r3, #-996]
	b	.L1946
.L1958:
	ldr	r2, [r3, #-996]
	b	.L2011
.L1956:
	cmp	r9, r6
	beq	.L1959
	cmn	r9, #1
	strne	r9, [r3, #-996]
	b	.L1946
.L1959:
	ldr	r2, [r3, #-996]
	cmp	r2, r6
	beq	.L1946
.L2011:
	sub	r2, r2, #1
	b	.L2006
.L1954:
	add	r1, r1, #1
	add	r2, r2, #36
	uxth	r1, r1
.L1952:
	cmp	r1, r4
	bne	.L1960
	add	r5, r5, #1
	uxth	r5, r5
.L1949:
	cmp	r5, sl
	bhi	.L1961
	ldr	r1, .L2012
	movw	r2, #3852
	ldr	lr, [r8, #-960]
	mov	r4, #0
	str	r3, [sp, #40]
	ldrh	r2, [r1, r2]
	mov	r1, r6
	str	lr, [sp, #28]
	str	r6, [sp, #0]
	str	r2, [sp, #16]
	mov	r2, r4
	b	.L1962
.L1961:
	mov	r4, r6
	mov	r6, r7
	mov	r7, r3
	ldr	r3, .L2012+4
	mvn	r2, #0
.L2006:
	str	r2, [r3, #-996]
.L1946:
	ldr	r5, .L2012+4
	mov	r3, #308
	ldr	r8, [sp, #20]
	mov	r2, #1
	str	r7, [sp, #16]
	strh	r2, [r5, r3]	@ movhi
.L1963:
	ldr	r3, .L2012
	movw	r2, #3852
	mov	r7, #0
	ldr	lr, [r5, #-960]
	mov	fp, #36
	ldrh	sl, [r3, r2]
	mov	r2, r4
	ldrb	ip, [r3, #3072]	@ zero_extendqisi2
	mov	r3, r7
	b	.L1964
.L1967:
	ldrh	r0, [r2, #16]
	movw	r1, #65535
	cmp	r0, r1
	beq	.L1965
	mla	r1, fp, r7, lr
	orr	r0, r8, r0, asl #10
	str	r0, [r1, #4]
	ldrb	r9, [r4, #8]	@ zero_extendqisi2
	cmp	r9, #1
	bne	.L1966
	cmp	ip, #0
	orrne	r0, r0, #-2147483648
	strne	r0, [r1, #4]
.L1966:
	add	r7, r7, #1
	uxth	r7, r7
.L1965:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1964:
	cmp	r3, sl
	bne	.L1967
	mov	r1, r7
	ldr	r0, [r5, #-960]
	ldr	r2, [sp, #32]
	bl	FlashReadPages
	mov	r3, #36
	mul	r3, r3, r7
	mov	r7, #0
	str	r3, [sp, #40]
	b	.L1968
.L1989:
	ldr	r9, [r5, #-960]
	add	r9, r9, r7
	ldr	sl, [r9, #4]
	ubfx	r0, sl, #10, #16
	str	sl, [sp, #60]
	bl	P2V_plane
	ldr	r3, [sp, #20]
	cmp	r8, r3
	bcc	.L1969
	ldr	fp, [sp, #36]
	mov	r1, r3
	cmp	r0, fp
	movcs	r3, #0
	movcc	r3, #1
	cmp	r8, r1
	movne	r3, #0
	cmp	r3, #0
	bne	.L1969
	ldr	r3, [sp, #24]
	ldr	fp, [sp, #16]
	cmp	r0, r3
	cmpeq	r8, fp
	beq	.L2001
	ldr	r3, [r9, #0]
	cmn	r3, #1
	beq	.L1971
	ldr	sl, [r9, #12]
	movw	r3, #61589
	ldrh	r2, [sl, #0]
	cmp	r2, r3
	ldrneh	r0, [r4, #0]
	bne	.L2007
	ldr	r6, [sl, #4]
	cmn	r6, #1
	beq	.L1973
	ldr	r1, [r5, #-1796]
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	addne	r3, r6, #1
	strne	r3, [r5, #-1796]
.L1973:
	ldr	r9, [sl, #8]
	add	r1, sp, #56
	ldr	r3, [sl, #12]
	mov	r2, #0
	mov	r0, r9
	str	r3, [sp, #52]
	bl	log2phys
	ldr	r1, [r5, #-996]
	cmn	r1, #1
	beq	.L1974
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1974
	ldr	r3, [sp, #52]
	cmn	r3, #1
	beq	.L1975
	ldr	r0, [r5, #-960]
	mov	r1, #1
	mov	r2, #0
	add	r0, r0, r7
	ldr	fp, [r0, #12]
	str	r3, [r0, #4]
	str	fp, [sp, #12]
	bl	FlashReadPages
	ldr	r3, [fp, #4]
	str	r3, [sp, #28]
	ldr	r3, [r5, #-960]
	add	fp, r3, r7
	ldr	r3, [r3, r7]
	cmn	r3, #1
	bne	.L1976
	b	.L1977
.L1975:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	bne	.L1969
	mov	r0, r9
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
	b	.L1969
.L1976:
	ldr	r3, [sp, #12]
	ldr	sl, [r3, #8]
	cmp	sl, r9
	bne	.L1977
	ldr	r0, [r5, #-996]
	ldr	r1, [sp, #28]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1977
	ldr	r3, [sp, #56]
	ldr	r2, [sp, #60]
	cmp	r3, r2
	beq	.L1981
.L1978:
	ldr	r2, [sp, #52]
	cmp	r3, r2
	beq	.L1977
	cmn	r3, #1
	streq	r3, [fp, #0]
	beq	.L1980
	ldr	lr, [fp, #12]
	mov	r0, fp
	str	r3, [fp, #4]
	mov	r1, #1
	mov	r2, #0
	str	lr, [sp, #12]
	bl	FlashReadPages
.L1980:
	ldr	r3, [r5, #-960]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	beq	.L1981
	ldr	r3, [sp, #12]
	ldr	r0, [r5, #-996]
	ldr	r9, [r3, #4]
	mov	r1, r9
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1981
	ldr	r0, [sp, #28]
	mov	r1, r9
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1977
.L1981:
	mov	r0, sl
	ldr	r1, [sp, #52]
	bl	FtlReUsePrevPpa
.L1977:
	mvn	r3, #0
	str	r3, [sp, #52]
	b	.L1983
.L1974:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	beq	.L1983
	ldr	r2, .L2012
	ldr	r3, [sp, #52]
	ldr	r2, [r2, #3868]
	cmp	r2, r3, lsr #10
	bhi	.L1984
	cmn	r3, #1
	bne	.L1969
.L1984:
	mov	r0, r9
	add	r1, sp, #60
	mov	r2, #1
	bl	log2phys
	ldr	fp, [sp, #56]
	cmn	fp, #1
	beq	.L1983
	ldr	r3, [sp, #52]
	cmp	fp, r3
	beq	.L1983
	ubfx	r0, fp, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L2012+12
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1985
	ldr	r3, .L2012+16
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1985
	ldr	r3, .L2012+20
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	bne	.L1983
.L1985:
	ldr	lr, .L2012+4
	mov	r1, #1
	mov	r2, #0
	ldr	r0, [lr, #-960]
	str	fp, [r0, #4]
	ldr	sl, [r0, #12]
	bl	FlashReadPages
	ldr	r0, .L2012+4
	ldr	r1, [sl, #4]
	ldr	r3, [r0, #-960]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L1983
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L1983
	mov	r0, r9
	add	r1, sp, #56
	mov	r2, #1
	bl	log2phys
.L1983:
	ldr	r0, [sp, #52]
	cmn	r0, #1
	beq	.L1969
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r5, #-2012]
	mov	r3, r0, asl #1
	mov	r1, r0
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1986
.L2007:
	bl	decrement_vpc_count
	b	.L1969
.L1986:
	ldr	r0, .L2012+24
	bl	printk
	b	.L1969
.L1971:
	ldrh	r3, [r4, #0]
	movw	r1, #306
	mov	r2, r6
	ldr	r0, .L2012+28
	strh	r3, [r5, r1]	@ movhi
	mov	r1, sl
	bl	printk
	ldr	r3, [r5, #312]
	cmp	r3, #31
	addls	r2, r5, r3, asl #2
	addls	r3, r3, #1
	ldrls	r1, [sp, #60]
	strls	r3, [r5, #312]
	strls	r1, [r2, #316]
	ldrh	r0, [r4, #0]
	bl	decrement_vpc_count
	ldr	r3, [r5, #-996]
	cmn	r3, #1
	beq	.L2008
.L1988:
	cmp	r3, r6
	bls	.L1969
.L2008:
	str	r6, [r5, #-996]
.L1969:
	add	r7, r7, #36
.L1968:
	ldr	r3, [sp, #40]
	cmp	r7, r3
	bne	.L1989
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	add	r8, r8, #1
	cmp	r3, #1
	uxth	r8, r8
	bne	.L1990
	ldr	r3, .L2012
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1990
	mov	r2, #3920
	ldr	fp, [sp, #16]
	ldrh	r3, [r3, r2]
	cmp	fp, r8
	cmpeq	r3, r8
	beq	.L2001
.L1990:
	ldr	r3, .L2012
	movw	r2, #3918
	ldrh	r2, [r3, r2]
	cmp	r8, r2
	bne	.L1963
	mov	r2, #0
	movw	r1, #3852
	strh	r2, [r4, #4]	@ movhi
	ldrh	r1, [r3, r1]
	mov	r3, r2
	strh	r8, [r4, #2]	@ movhi
	movw	r2, #65535
	ldr	r0, [sp, #44]
	b	.L1991
.L1993:
	add	r0, r0, #2
	ldrh	ip, [r0, #14]
	cmp	ip, r2
	strneb	r3, [r4, #6]
	bne	.L1923
.L1992:
	add	r3, r3, #1
	uxth	r3, r3
.L1991:
	cmp	r3, r1
	bne	.L1993
	b	.L1923
.L2001:
.L1970:
	ldr	r7, [sp, #16]
	mov	r0, r4
	ldr	r3, [sp, #24]
	mov	r1, r7
	strh	r7, [r4, #2]	@ movhi
	mov	r2, r3
	strb	r3, [r4, #6]
.L2010:
	bl	ftl_sb_update_avl_pages
.L1923:
	mov	r0, #0
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2013:
	.align	2
.L2012:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC144
	.word	-1992
	.word	-1944
	.word	-1896
	.word	.LC145
	.word	.LC146
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
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
	ldr	r5, .L2023
	ldr	r3, [r5, #4044]
	cmp	r3, #0
	bne	.L2014
	ldr	r4, .L2023+4
	ldr	r3, .L2023+8
	mov	r2, r4
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	beq	.L2016
	ldrb	r1, [r4, #-1984]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L2016
	movw	r0, #3918
	ldrb	r1, [r4, #-1985]	@ zero_extendqisi2
	ldrh	r0, [r5, r0]
	mul	r1, r0, r1
	cmp	r3, r1
	beq	.L2016
	ldrb	r8, [r4, #-1982]	@ zero_extendqisi2
	movw	r3, #3852
	ldr	r7, [r5, #3980]
	cmp	r8, #0
	ldrh	r6, [r5, r3]
	bne	.L2014
	sub	r7, r7, #1
	mov	r1, sp
	mov	r2, r8
	mov	r0, r7
	bl	log2phys
	ldr	r3, [sp, #0]
	ldr	r0, [r4, #-948]
	ldr	r4, [r4, #-924]
	cmn	r3, #1
	str	r3, [sp, #8]
	str	r7, [sp, #20]
	str	r0, [sp, #12]
	str	r4, [sp, #16]
	str	r8, [r4, #4]
	beq	.L2017
	add	r0, sp, #4
	mov	r1, #1
	mov	r2, r8
	bl	FlashReadPages
	b	.L2018
.L2017:
	movw	r3, #3928
	mov	r1, #255
	ldrh	r2, [r5, r3]
	bl	ftl_memset
.L2018:
	ldr	r3, .L2023+12
	mov	r6, r6, asl #2
	ldr	r5, .L2023+4
	ldr	sl, .L2023+8
	strh	r3, [r4, #0]	@ movhi
	ldr	r8, .L2023+16
	b	.L2019
.L2022:
	ldrh	r3, [r5, sl]
	cmp	r3, #0
	beq	.L2020
	ldr	r3, [sp, #8]
	sub	r6, r6, #1
	str	r7, [r4, #8]
	ldr	r0, .L2023+20
	str	r3, [r4, #12]
	ldrh	r3, [r5, r8]
	strh	r3, [r4, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r5, #-1796]
	mov	r2, #0
	mov	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r4, #4]
	add	r3, r3, #1
	cmn	r3, #1
	str	r3, [r5, #-1796]
	moveq	r3, #0
	streq	r3, [r5, #-1796]
	mov	r3, r2
	bl	FlashProgPages
	ldrh	r0, [r5, r8]
	bl	decrement_vpc_count
.L2019:
	cmp	r6, #0
	bne	.L2022
.L2020:
	ldr	r3, .L2023+4
	mov	r2, #1
	strb	r2, [r3, #-1982]
	b	.L2014
.L2016:
	mov	r3, #0
	strb	r3, [r2, #-1982]
.L2014:
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L2024:
	.align	2
.L2023:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1988
	.word	-3947
	.word	-1992
	.word	.LANCHOR2-1992
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
	ldr	r7, .L2029
	movw	r6, #3958
	ldr	r5, .L2029+4
	b	.L2026
.L2028:
	ldr	r3, [r5, #-1848]
	mov	r2, #12
	mla	r3, r2, r4, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L2027
	mov	r0, r4
	bl	flush_l2p_region
.L2027:
	add	r4, r4, #1
	uxth	r4, r4
.L2026:
	ldrh	r3, [r7, r6]
	cmp	r3, r4
	bhi	.L2028
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2030:
	.align	2
.L2029:
	.word	.LANCHOR0
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
	ldr	r3, .L2043
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r3, [r3, #4044]
	ldrh	r5, [r0, #0]
	cmp	r3, #0
	bne	.L2032
	movw	r3, #65535
	cmp	r5, r3
	beq	.L2033
	ldr	r3, .L2043+4
	mov	r0, r5
	ldr	r2, [r3, #-2012]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2034
	bl	INSERT_DATA_LIST
	b	.L2033
.L2034:
	bl	INSERT_FREE_LIST
.L2033:
	mov	r3, #0
	strb	r3, [r4, #8]
	ldr	r3, .L2043+8
	cmp	r4, r3
	beq	.L2035
	ldr	r2, .L2043
	mov	r3, #3872
	ldrh	r3, [r2, r3]
	cmp	r3, #1
	beq	.L2035
	ldrb	r2, [r2, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2036
.L2035:
	mov	r3, #1
	strb	r3, [r4, #8]
	b	.L2037
.L2036:
	ldr	r2, .L2043+12
	cmp	r4, r2
	bne	.L2037
	cmp	r3, #3
	ldr	r3, .L2043+4
	beq	.L2038
	ldr	r2, [r3, #-1704]
	cmp	r2, #1
	bne	.L2039
.L2038:
	mov	r2, #1
	strb	r2, [r3, #-1984]
.L2039:
	ldr	r2, .L2043
	ldr	r2, [r2, #3844]
	cmp	r2, #0
	beq	.L2037
	ldr	r3, [r3, #-1776]
	cmp	r3, #39
	ldrls	r3, .L2043+4
	movls	r2, #1
	strlsb	r2, [r3, #-1984]
.L2037:
	ldr	r3, .L2043+16
	ldr	r6, .L2043+4
	ldrh	r0, [r6, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L2040
	cmp	r5, r0
	bne	.L2041
	ldr	r2, [r6, #-2012]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2042
.L2041:
	bl	update_vpc_list
.L2042:
	ldr	r3, .L2043+16
	mvn	r2, #0
	strh	r2, [r6, r3]	@ movhi
.L2040:
	mov	r0, r4
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2032:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2044:
	.align	2
.L2043:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2-1944
	.word	.LANCHOR2-1992
	.word	-1008
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
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
	ldr	r6, .L2053
	ldr	r1, .L2053+4
	ldr	r0, .L2053+8
	bl	printk
	movw	r3, #3862
	ldrh	r2, [r6, r3]
	mov	r1, #0
	ldr	r5, .L2053+12
	mov	r2, r2, asl #1
	ldr	r0, [r5, #-904]
	bl	ftl_memset
	b	.L2046
.L2048:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L2047
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r5, #-904]
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	add	r2, r2, #1
	strh	r2, [r3, r0]	@ movhi
.L2047:
	add	r4, r4, #1
.L2046:
	ldr	r3, [r6, #3980]
	cmp	r4, r3
	bcc	.L2048
	mov	r4, #0
	ldr	r7, .L2053
	movw	r8, #3860
	ldr	r5, .L2053+12
	b	.L2049
.L2052:
	ldr	r3, [r5, #-2012]
	mov	r6, r4, asl #1
	ldrh	r2, [r3, r6]
	ldr	r3, [r5, #-904]
	ldrh	r3, [r3, r6]
	cmp	r2, r3
	beq	.L2050
	movw	r1, #65535
	cmp	r2, r1
	beq	.L2050
	ldr	r0, .L2053+16
	mov	r1, r4
	bl	printk
	ldr	r3, .L2053+20
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L2050
	ldr	r3, .L2053+24
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L2050
	ldr	r3, .L2053+28
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L2050
	ldr	r3, [r5, #-2012]
	ldrh	r2, [r3, r6]
	cmp	r2, #0
	ldr	r2, [r5, #-904]
	ldrh	r2, [r2, r6]
	strh	r2, [r3, r6]	@ movhi
	beq	.L2050
.L2051:
	mov	r0, r4
	bl	update_vpc_list
.L2050:
	add	r4, r4, #1
	uxth	r4, r4
.L2049:
	ldrh	r3, [r7, r8]
	cmp	r3, r4
	bhi	.L2052
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L2054:
	.align	2
.L2053:
	.word	.LANCHOR0
	.word	.LANCHOR3+48
	.word	.LC101
	.word	.LANCHOR2
	.word	.LC147
	.word	-1992
	.word	-1896
	.word	-1944
	.fnend
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2078
	movw	r2, #3918
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #4044]
	cmp	r2, #0
	bne	.L2076
	ldr	ip, .L2078+4
	ldr	r3, .L2078+8
	ldrh	r4, [r3, ip]
	movw	ip, #65535
	cmp	r4, ip
	beq	.L2057
	cmp	r0, #0
	beq	.L2058
	ldr	lr, .L2078+12
	movw	r0, #3224
	ldrh	r5, [lr, r0]
	cmp	r5, ip
	movne	r1, #2
	bne	.L2058
	strh	r2, [lr, r0]	@ movhi
	ldr	r2, .L2078+16
	ldrh	r3, [r3, r2]
	cmp	r3, #17
	movhi	r1, #2
.L2058:
	ldr	r0, .L2078+20
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #4]
	beq	.L2059
	ldr	r3, .L2078
	mov	r4, r4, asl #1
	ldr	r3, [r3, #4052]
	ldrh	r2, [r3, r4]
	cmp	r2, #4
	bls	.L2060
	sub	r2, r2, #5
	mov	r0, #1
	strh	r2, [r3, r4]	@ movhi
	bl	FtlEctTblFlush
.L2060:
	ldr	r3, .L2078+8
	ldr	r2, [r3, #-1000]
	cmp	r2, #0
	bne	.L2061
	ldr	r2, [r3, #-1604]
	ldr	r0, [sp, #4]
	add	r2, r2, #1
	str	r2, [r3, #-1604]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L2061:
	ldr	r3, .L2078+8
	mov	r2, #0
	str	r2, [r3, #-1000]
	b	.L2073
.L2059:
	ldr	r2, .L2078+12
	movw	r3, #3224
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2073
.L2057:
	ldr	r6, .L2078+8
	movw	r3, #65535
	ldr	r5, .L2078+4
	mov	r4, #0
	str	r4, [r6, #-1000]
	ldrh	r2, [r6, r5]
	cmp	r2, r3
	beq	.L2056
	bl	FtlCacheWriteBack
	ldr	ip, .L2078
	movw	r0, #3918
	ldrb	r1, [r6, #-1889]	@ zero_extendqisi2
	ldrh	r3, [r6, r5]
	ldrh	r0, [ip, r0]
	ldr	r2, [r6, #-2012]
	mov	r3, r3, asl #1
	mul	r1, r0, r1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, .L2078+24
	ldrh	r2, [r6, r3]
	ldr	r3, [r6, #-1812]
	add	r3, r2, r3
	str	r3, [r6, #-1812]
	b	.L2062
.L2065:
	mov	r7, #12
	ldr	r8, [r6, #-1140]
	mul	r7, r7, r4
	add	r1, sp, #4
	mov	r2, #0
	add	r5, r8, r7
	ldr	r0, [r5, #8]
	bl	log2phys
	ldr	r0, [sp, #4]
	ldr	r3, [r8, r7]
	cmp	r0, r3
	bne	.L2063
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r5, #4
	mov	r2, #1
	mov	r7, r0
	ldr	r0, [r5, #8]
	bl	log2phys
	mov	r0, r7
	b	.L2077
.L2063:
	ldr	r3, [r5, #4]
	cmp	r0, r3
	beq	.L2064
	ldr	r3, .L2078+4
	ldrh	r0, [r6, r3]
.L2077:
	bl	decrement_vpc_count
.L2064:
	add	r4, r4, #1
	uxth	r4, r4
.L2062:
	ldr	r3, .L2078+24
	ldr	r5, .L2078+8
	ldrh	r3, [r6, r3]
	cmp	r3, r4
	bhi	.L2065
	movw	r0, #65535
	ldr	r4, .L2078+4
	bl	decrement_vpc_count
	ldr	r3, .L2078
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2066
	ldr	r0, .L2078+28
	ldrh	r1, [r5, r4]
	bl	printk
.L2066:
	ldrh	r0, [r5, r4]
	ldr	r2, [r5, #-2012]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2067
	bl	INSERT_DATA_LIST
	b	.L2068
.L2067:
	bl	INSERT_FREE_LIST
.L2068:
	ldr	r5, .L2078+8
	mvn	r6, #0
	ldr	r3, .L2078+4
	mov	r4, #0
	strh	r6, [r5, r3]	@ movhi
	add	r3, r3, #760
	strh	r4, [r5, r3]	@ movhi
	sub	r3, r3, #12
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	r3, .L2078+32
	mov	r0, r5
	ldr	r2, .L2078+16
	strh	r6, [r5, r3]	@ movhi
	ldr	r3, .L2078
	ldr	r3, [r3, #3844]
	cmp	r3, r4
	ldr	r3, .L2078+36
	beq	.L2069
	ldr	r1, [r5, #-1776]
	cmp	r1, #39
	bhi	.L2069
	ldrh	r3, [r5, r3]
	ldrh	r2, [r5, r2]
	cmp	r2, r3
	ldrcc	r2, .L2078+40
	movcc	r3, r3, asl #1
	strcch	r3, [r5, r2]	@ movhi
	b	.L2056
.L2069:
	ldrh	r3, [r0, r3]
	ldrh	r2, [r0, r2]
	ldr	r1, .L2078+8
	add	r0, r3, r3, asl #1
	cmp	r2, r0, lsr #2
	ble	.L2076
	ldr	r2, .L2078
	ldrb	r4, [r2, #3072]	@ zero_extendqisi2
	ldr	r2, .L2078+40
	cmp	r4, #0
	subne	r3, r3, #2
	moveq	r3, #20
	strneh	r3, [r1, r2]	@ movhi
	streqh	r3, [r1, r2]	@ movhi
	beq	.L2056
	b	.L2076
.L2073:
	mov	r4, #1
	b	.L2056
.L2076:
	mov	r4, #0
.L2056:
	mov	r0, r4
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L2079:
	.align	2
.L2078:
	.word	.LANCHOR0
	.word	-1896
	.word	.LANCHOR2
	.word	.LANCHOR1
	.word	-1996
	.word	.LANCHOR2-1896
	.word	-1136
	.word	.LC148
	.word	-1752
	.word	-1772
	.word	-1188
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.type	Ftl_gc_temp_data_write_back.part.23, %function
Ftl_gc_temp_data_write_back.part.23:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r2, #0
	ldr	r4, .L2086
	mov	r3, r2
	mov	r5, #0
	mov	r6, #36
	ldr	r0, [r4, #-956]
	ldr	r1, [r4, #-1172]
	bl	FlashProgPages
	b	.L2081
.L2084:
	mul	r3, r6, r5
	ldr	r1, [r4, #-956]
	add	r2, r1, r3
	ldr	r1, [r1, r3]
	cmn	r1, #1
	bne	.L2082
	ldr	r2, .L2086+4
	mov	lr, #0
	ldr	ip, [r7, #-2012]
	ldrh	r0, [r7, r2]
	mov	r0, r0, asl #1
	strh	lr, [ip, r0]	@ movhi
	strh	r1, [r7, r2]	@ movhi
	ldr	r2, [r7, #-1604]
	add	r2, r2, #1
	str	r2, [r7, #-1604]
	ldr	r2, [r7, #-956]
	add	r3, r2, r3
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2082:
	ldr	r3, [r2, #12]
	add	r5, r5, #1
	ldr	r1, [r2, #4]
	uxth	r5, r5
	ldr	r0, [r3, #12]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
.L2081:
	ldr	r1, [r4, #-1172]
	ldr	r7, .L2086
	cmp	r5, r1
	bcc	.L2084
	ldr	r0, [r7, #-956]
	bl	FtlGcBufFree
	ldr	r3, .L2086+8
	mov	r0, #0
	str	r0, [r7, #-1172]
	ldrh	r3, [r7, r3]
	cmp	r3, r0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2087:
	.align	2
.L2086:
	.word	.LANCHOR2
	.word	-1896
	.word	-1892
	.fnend
	.size	Ftl_gc_temp_data_write_back.part.23, .-Ftl_gc_temp_data_write_back.part.23
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L2093
	ldr	r2, [r3, #4044]
	cmp	r2, #0
	bne	.L2089
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2090
	ldr	r3, .L2093+4
	ldr	r2, [r3, #-1172]
	tst	r2, #1
	beq	.L2090
	ldr	r2, .L2093+8
	ldrh	r3, [r3, r2]
	cmp	r3, #0
	bne	.L2089
.L2090:
	b	Ftl_gc_temp_data_write_back.part.23
.L2089:
	mov	r0, #0
	bx	lr
.L2094:
	.align	2
.L2093:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1892
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
	movw	r5, #3918
	ldr	r6, .L2097
	ldr	r0, .L2097+4
	ldr	r4, .L2097+8
	ldrh	r1, [r6, r5]
	bl	FtlGcScanTempBlk
	ldr	r3, .L2097+12
	ldrh	r2, [r4, r3]
	ldrh	r3, [r6, r5]
	cmp	r2, r3
	ldmccfd	sp!, {r4, r5, r6, pc}
	ldr	r0, .L2097+16
	bl	FtlMapBlkWriteDumpData
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	mov	r3, #0
	str	r3, [r4, #-1000]
	ldmfd	sp!, {r4, r5, r6, pc}
.L2098:
	.align	2
.L2097:
	.word	.LANCHOR0
	.word	.LANCHOR2-1896
	.word	.LANCHOR2
	.word	-1894
	.word	.LANCHOR2-1076
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
	ldr	r3, .L2100
	ldr	r5, .L2100+4
	str	r4, [r3, #312]
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
.L2101:
	.align	2
.L2100:
	.word	.LANCHOR2
	.word	.LANCHOR2-1992
	.fnend
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r3, #308
	ldr	r6, .L2115
	mov	r2, #0
	ldr	r4, .L2115+4
	mvn	r5, #0
	ldr	r0, [r6, #3856]
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #306
	strh	r5, [r4, r3]	@ movhi
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldr	r3, .L2115+8
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2103
	bl	FtlLoadSysInfo
	subs	r7, r0, #0
	bne	.L2103
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	r0, #1
	bl	FtlUpdateVaildLpn
	movw	r3, #3958
	ldrh	r1, [r6, r3]
	mov	r2, r7
	ldr	r3, [r4, #-1848]
	b	.L2104
.L2106:
	add	r0, r3, r7
	add	r7, r7, #12
	ldr	r0, [r0, #4]
	cmp	r0, #0
	blt	.L2105
	add	r2, r2, #1
.L2104:
	cmp	r2, r1
	blt	.L2106
.L2105:
	ldr	r3, .L2115+4
	cmp	r2, r1
	ldr	r0, .L2115+12
	ldrh	ip, [r3, r0]
	add	ip, ip, #1
	strh	ip, [r3, r0]	@ movhi
	blt	.L2107
	mov	r2, #308
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L2108
.L2107:
	ldr	r2, .L2115+16
	ldr	r0, [r3, #-2012]
	ldr	r1, .L2115+20
	ldrh	r2, [r3, r2]
	ldrh	ip, [r3, r1]
	mov	r2, r2, asl #1
	ldrh	lr, [r0, r2]
	rsb	ip, ip, lr
	strh	ip, [r0, r2]	@ movhi
	ldr	ip, .L2115
	movw	r0, #3918
	ldr	r2, .L2115+24
	ldr	r4, [r3, #-2012]
	ldrh	lr, [ip, r0]
	strh	lr, [r3, r2]	@ movhi
	mov	r2, #0
	strh	r2, [r3, r1]	@ movhi
	add	r1, r1, #44
	strb	r2, [r3, #-1986]
	ldrh	lr, [r3, r1]
	add	r1, r1, #4
	ldrh	r5, [r3, r1]
	mov	lr, lr, asl #1
	ldrh	r6, [r4, lr]
	rsb	r5, r5, r6
	strh	r5, [r4, lr]	@ movhi
	strh	r2, [r3, r1]	@ movhi
	strb	r2, [r3, #-1938]
	ldr	r2, .L2115+28
	ldrh	ip, [ip, r0]
	ldr	r0, .L2115+32
	ldrh	r1, [r3, r2]
	strh	ip, [r3, r0]	@ movhi
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L2108:
	ldr	r3, .L2115+4
	ldr	r2, .L2115+12
	mov	r5, r3
	ldrh	r2, [r3, r2]
	tst	r2, #31
	bne	.L2109
	bl	FtlVpcCheckAndModify
.L2109:
	ldr	r2, .L2115+16
	movw	r3, #65535
	ldr	r4, .L2115+4
	ldrh	r0, [r5, r2]
	cmp	r0, r3
	beq	.L2113
	ldr	r3, .L2115+20
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L2113
	add	r2, r2, #52
	ldrh	r5, [r4, r2]
	cmp	r5, #0
	movne	r5, r3
	bne	.L2103
	bl	FtlGcRefreshOpenBlock
	ldr	r3, .L2115+36
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshOpenBlock
	bl	FtlVpcTblFlush
	ldr	r0, .L2115+40
	bl	allocate_new_data_superblock
	ldr	r0, .L2115+44
	bl	allocate_new_data_superblock
	b	.L2103
.L2113:
	mov	r5, #0
.L2103:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2116:
	.align	2
.L2115:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1768
	.word	-2040
	.word	-1992
	.word	-1988
	.word	-1990
	.word	-2038
	.word	-1942
	.word	-1944
	.word	.LANCHOR2-1992
	.word	.LANCHOR2-1944
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.type	FtlLowFormat.part.27, %function
FtlLowFormat.part.27:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	r6, #3956
	ldr	r5, .L2138
	mov	r1, #0
	ldr	r4, .L2138+4
	ldrh	r2, [r5, r6]
	ldr	r0, [r4, #-888]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r5, r6]
	mov	r1, #0
	ldr	r0, [r4, #-1020]
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r0, [r5, #3856]
	mov	r3, #0
	str	r3, [r4, #-1800]
	str	r3, [r4, #-1796]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L2118
	bl	FtlMakeBbt
.L2118:
	mov	r3, #0
	ldr	ip, .L2138
	movw	r0, #3924
	ldr	r2, .L2138+4
	ldr	r1, .L2138+8
	b	.L2119
.L2120:
	ldr	lr, [r2, #-932]
	mvn	r4, r3
	orr	r4, r3, r4, asl #16
	str	r4, [lr, r3, asl #2]
	ldr	lr, [r2, #-928]
	str	r1, [lr, r3, asl #2]
	add	r3, r3, #1
	uxth	r3, r3
.L2119:
	ldrh	lr, [ip, r0]
	ldr	r8, .L2138
	cmp	r3, lr, asl #7
	blt	.L2120
	movw	r3, #3860
	mov	r5, #0
	ldrh	r6, [r8, r3]
	movw	r7, #3862
	b	.L2121
.L2122:
	mov	r0, r6
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L2121:
	ldrh	r3, [r8, r7]
	ldr	r4, .L2138
	cmp	r3, r6
	bhi	.L2122
	movw	r3, #3852
	ldrh	r1, [r4, r3]
	sub	r3, r5, #3
	cmp	r3, r1, asl #1
	blt	.L2123
	mov	r0, r5
	movw	r6, #3862
	bl	__aeabi_uidiv
	ldr	r3, [r4, #3952]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldr	r0, [r4, #3856]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	movw	r3, #3860
	ldrh	r5, [r4, r3]
	b	.L2124
.L2125:
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
.L2124:
	ldrh	r3, [r4, r6]
	cmp	r3, r5
	bhi	.L2125
.L2123:
	mov	r4, #0
	ldr	r8, .L2138
	mov	r6, r4
	movw	r7, #3860
	b	.L2126
.L2127:
	mov	r0, r6
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r4, r4, r0
	uxth	r4, r4
.L2126:
	ldrh	r3, [r8, r7]
	ldr	r5, .L2138
	cmp	r3, r6
	bhi	.L2127
	movw	r3, #3862
	ldr	r8, .L2138+4
	ldrh	r3, [r5, r3]
	ldr	fp, [r5, #3864]
	ldr	r9, .L2138+12
	str	r3, [r8, #-972]
	movw	r3, #3852
	ldrh	r6, [r5, r3]
	mov	r0, fp
	mov	r1, r6
	bl	__aeabi_uidiv
	ubfx	sl, r0, #5, #16
	mov	r7, r0
	add	r3, sl, #36
	strh	r3, [r8, r9]	@ movhi
	mov	r3, #24
	str	r0, [r5, #3980]
	mul	r3, r3, r6
	cmp	r4, r3
	ble	.L2128
	rsb	r0, r4, fp
	mov	r1, r6
	bl	__aeabi_uidiv
	str	r0, [r5, #3980]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r8, r9]	@ movhi
.L2128:
	ldr	r3, .L2138
	ldr	r3, [r3, #3844]
	cmp	r3, #1
	bne	.L2129
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r8, .L2138+4
	ldr	r5, .L2138+12
	ldrh	r9, [r8, r5]
	uxtah	r0, r9, r0
	add	r9, r9, r0, lsr #2
	strh	r9, [r8, r5]	@ movhi
.L2129:
	ldr	r3, .L2138
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2130
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r8, .L2138+4
	ldr	r5, .L2138+12
	ldrh	r9, [r8, r5]
	uxtah	r0, r9, r0
	add	r9, r9, r0, lsr #2
	strh	r9, [r8, r5]	@ movhi
.L2130:
	ldr	r0, .L2138
	movw	r3, #3912
	ldrh	r3, [r0, r3]
	cmp	r3, #0
	beq	.L2131
	ldr	r1, .L2138+4
	ldr	r2, .L2138+12
	ldrh	ip, [r1, r2]
	add	ip, ip, r3, lsr #1
	strh	ip, [r1, r2]	@ movhi
	mul	ip, r6, r3
	cmp	ip, r4
	strgt	r7, [r0, #3980]
	addgt	r3, r3, #32
	addgt	sl, sl, r3
	strgth	sl, [r1, r2]	@ movhi
.L2131:
	ldr	r4, .L2138+4
	ldr	r5, .L2138
	ldr	r3, .L2138+12
	ldr	r7, .L2138+16
	ldr	r2, [r5, #3980]
	ldrh	r3, [r4, r3]
	rsb	r3, r3, r2
	mul	r6, r6, r3
	movw	r3, #3918
	ldrh	r3, [r5, r3]
	str	r6, [r4, #228]
	mul	r6, r3, r6
	movw	r3, #3924
	ldrh	r3, [r5, r3]
	str	r6, [r5, #3980]
	mul	r6, r3, r6
	str	r6, [r5, #3960]
	mvn	r6, #0
	bl	FtlBbmTblFlush
	movw	r3, #3862
	ldrh	r2, [r5, r3]
	mov	r1, #0
	ldr	r0, [r4, #-2012]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r1, .L2138+20
	mov	r3, #0
	ldr	r2, .L2138+24
	str	r3, [r4, #-1840]
	strh	r3, [r4, r1]	@ movhi
	sub	r1, r1, #240
	strb	r3, [r4, #-1746]
	strh	r3, [r4, r1]	@ movhi
	mov	r1, #255
	strb	r3, [r4, #-1744]
	strb	r3, [r4, #-1986]
	strh	r3, [r4, r7]	@ movhi
	mov	r3, #1
	strb	r3, [r4, #-1984]
	movw	r3, #3860
	strh	r6, [r4, r2]	@ movhi
	ldrh	r2, [r5, r3]
	ldr	r0, [r4, #-1836]
	mov	r2, r2, lsr #3
	bl	ftl_memset
.L2132:
	ldr	r0, .L2138+28
	bl	make_superblock
	ldrb	r3, [r4, #-1985]	@ zero_extendqisi2
	ldr	r5, .L2138+4
	cmp	r3, #0
	ldr	r3, .L2138+16
	bne	.L2133
	ldrh	r3, [r4, r7]
	ldr	r2, [r4, #-2012]
	mov	r3, r3, asl #1
	strh	r6, [r2, r3]	@ movhi
	ldrh	r3, [r4, r7]
	add	r3, r3, #1
	strh	r3, [r4, r7]	@ movhi
	b	.L2132
.L2133:
	ldr	r2, [r5, #-1800]
	mvn	r7, #0
	ldr	r1, [r5, #-2012]
	ldr	r6, .L2138+32
	str	r2, [r5, #-1980]
	add	r2, r2, #1
	str	r2, [r5, #-1800]
	ldr	r2, .L2138+36
	ldrh	r0, [r5, r2]
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #1
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	ldr	r0, .L2138+40
	ldrh	r3, [r5, r3]
	strb	r2, [r5, #-1938]
	add	r3, r3, #1
	strh	r2, [r5, r0]	@ movhi
	strh	r3, [r5, r6]	@ movhi
	mov	r3, #1
	strb	r3, [r5, #-1936]
.L2134:
	ldr	r0, .L2138+44
	bl	make_superblock
	ldrb	r3, [r5, #-1937]	@ zero_extendqisi2
	ldr	r4, .L2138+4
	cmp	r3, #0
	bne	.L2135
	ldrh	r3, [r5, r6]
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	strh	r7, [r2, r3]	@ movhi
	ldrh	r3, [r5, r6]
	add	r3, r3, #1
	strh	r3, [r5, r6]	@ movhi
	b	.L2134
.L2135:
	ldr	r3, [r4, #-1800]
	mvn	r5, #0
	ldr	r2, [r4, #-2012]
	str	r3, [r4, #-1932]
	add	r3, r3, #1
	str	r3, [r4, #-1800]
	ldr	r3, .L2138+48
	ldrh	r1, [r4, r3]
	sub	r3, r3, #4
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, .L2138+52
	strh	r5, [r4, r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, .L2138+56
	mov	r2, #0
	strh	r0, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #2
	ldr	r2, [r4, #228]
	strh	r5, [r4, r3]	@ movhi
	add	r3, r3, #2
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, [r4, #-1800]
	str	r3, [r4, #-1760]
	add	r3, r3, #1
	str	r3, [r4, #-1800]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
	ldr	r3, .L2138+60
	mov	r2, #1
	str	r2, [r3, #3220]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2139:
	.align	2
.L2138:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	168778952
	.word	-1772
	.word	-1992
	.word	-1750
	.word	-1752
	.word	.LANCHOR2-1992
	.word	-1944
	.word	-1988
	.word	-1942
	.word	.LANCHOR2-1944
	.word	-1940
	.word	-1896
	.word	-1768
	.word	.LANCHOR1
	.fnend
	.size	FtlLowFormat.part.27, .-FtlLowFormat.part.27
	.align	2
	.global	FtlLowFormat
	.type	FtlLowFormat, %function
FtlLowFormat:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L2142
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L2141
	bl	FtlLowFormat.part.27
.L2141:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2143:
	.align	2
.L2142:
	.word	.LANCHOR0
	.fnend
	.size	FtlLowFormat, .-FtlLowFormat
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L2147
	ldr	r5, .L2147+4
	ldrh	r2, [r5, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2145
	ldr	r3, .L2147+8
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L2146
.L2145:
	bl	FtlCacheWriteBack
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L2147+12
	mov	r4, #0
	strb	r4, [r5, #-1888]
	bl	allocate_data_superblock
	ldr	r3, .L2147+16
	strh	r4, [r5, r3]	@ movhi
	add	r3, r3, #12
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	mov	r0, r4
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2146:
	ldr	r0, .L2147+12
	ldmfd	sp!, {r3, r4, r5, lr}
	b	get_new_active_ppa
.L2148:
	.align	2
.L2147:
	.word	-1896
	.word	.LANCHOR2
	.word	-1892
	.word	.LANCHOR2-1896
	.word	-1148
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.type	ftl_do_gc.part.24, %function
ftl_do_gc.part.24:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r1
	ldr	r3, .L2246
	.pad #36
	sub	sp, sp, #36
	ldr	r4, .L2246+4
	movw	r1, #65535
	str	r0, [sp, #16]
	ldrh	r7, [r3, r4]
	cmp	r7, r1
	beq	.L2150
	ldr	r6, .L2246+8
	ldrh	r2, [r3, r6]
	cmp	r2, r1
	bne	.L2150
	ldr	r0, .L2246+12
	ldrh	r5, [r3, r0]
	cmp	r5, r2
	beq	.L2150
	ldr	r1, .L2246+16
	ldrh	ip, [r3, r1]
	cmp	ip, r2
	strneh	r7, [r3, r6]	@ movhi
	strneh	r5, [r3, r4]	@ movhi
	mvnne	r2, #0
	strneh	ip, [r3, r0]	@ movhi
	strneh	r2, [r3, r1]	@ movhi
.L2150:
	ldr	r1, [sp, #16]
	ldr	r2, [r3, #-1184]
	cmp	r1, #1
	add	r2, r2, #1
	add	r2, r2, r1, asl #7
	str	r2, [r3, #-1184]
	bne	.L2151
	ldr	r3, .L2246+20
	ldr	r1, [r3, #3844]
	cmp	r1, #0
	bne	.L2152
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2151
.L2152:
	ldr	r4, .L2246
	ldr	r3, [r4, #-1776]
	cmp	r3, #39
	bhi	.L2151
	mov	r3, #444
	ldrh	r3, [r4, r3]
	add	r2, r2, r3
	str	r2, [r4, #-1184]
	bl	FtlGcReFreshBadBlk
	ldr	r3, .L2246+24
	movw	r2, #65535
	ldrh	r3, [r4, r3]
	cmp	r3, r2
	bne	.L2151
	ldr	r2, .L2246+8
	ldrh	r2, [r4, r2]
	cmp	r2, r3
	bne	.L2151
	ldr	r3, [r4, #-1184]
	cmp	r3, #1024
	bhi	.L2153
	ldr	r3, .L2246+28
	ldrh	r3, [r4, r3]
	cmp	r3, #63
	bhi	.L2151
.L2153:
	ldr	r3, .L2246
	mov	r2, #444
	ldr	r0, .L2246+28
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	ldrh	ip, [r3, r0]
	ldr	r0, .L2246+32
	ldrh	r0, [r3, r0]
	add	r0, r0, #64
	cmp	ip, r0
	bgt	.L2151
	str	r1, [r3, #-1184]
	ldr	r1, [r3, #-1776]
	cmp	r1, #0
	moveq	r1, #6
	beq	.L2242
	cmp	r1, #5
	bhi	.L2155
	mov	r1, #18
.L2242:
	strh	r1, [r3, r2]	@ movhi
.L2155:
	mov	r0, #32
	movw	r7, #65535
	bl	List_get_gc_head_node
	uxth	r2, r0
	cmp	r2, r7
	beq	.L2156
	ldr	r5, .L2246
	ldr	r6, .L2246+36
	ldrh	r0, [r5, r6]
	cmp	r0, #0
	moveq	r3, #1
	streqh	r3, [r5, r6]	@ movhi
	beq	.L2156
	ldr	r3, .L2246+20
	mov	ip, #3920
	movw	lr, #3852
	ldr	r9, [r5, #-2012]
	mov	r2, r2, asl #1
	ldrh	ip, [r3, ip]
	ldrh	r3, [r3, lr]
	ldrh	r1, [r9, r2]
	mul	r3, r3, ip
	add	r3, r3, #1
	cmp	r1, r3
	bgt	.L2156
	add	fp, r0, #1
	mov	r8, #0
	str	r8, [r5, #-1176]
	uxth	fp, fp
	strh	fp, [r5, r6]	@ movhi
	str	r2, [sp, #8]
	bl	List_get_gc_head_node
	ldr	r2, [sp, #8]
	uxth	r4, r0
	cmp	r4, r7
	beq	.L2156
	ldrh	r2, [r9, r2]
	mov	r7, r4, asl #1
	ldr	r0, .L2246+40
	mov	r1, fp
	ldrh	r3, [r9, r7]
	str	r2, [sp, #0]
	mov	r2, r4
	bl	printk
	ldrh	r3, [r5, r6]
	cmp	r3, #40
	bls	.L2158
	ldr	r3, [r5, #-2012]
	ldrh	r3, [r3, r7]
	cmp	r3, #32
	strhih	r8, [r5, r6]	@ movhi
.L2158:
	ldr	r2, .L2246
	mov	r3, #444
	mov	r1, #6
	strh	r1, [r2, r3]	@ movhi
	b	.L2160
.L2156:
	bl	GetSwlReplaceBlock
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	bne	.L2160
	ldr	r2, .L2246
	mov	r3, #444
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L2151:
	ldr	r5, .L2246
	movw	r4, #65535
	ldr	r3, .L2246+24
	ldrh	r2, [r5, r3]
	cmp	r2, r4
	bne	.L2160
	sub	r3, r3, #144
	ldrh	r3, [r5, r3]
	cmp	r3, r2
	movne	r4, r2
	bne	.L2160
	ldr	r2, .L2246+8
	ldrh	r7, [r5, r2]
	cmp	r7, r3
	movne	r4, r3
	bne	.L2160
	ldr	r8, .L2246+28
	ldr	r2, [r5, #-1184]
	ldrh	r3, [r5, r8]
	cmp	r3, #23
	movhi	r3, #1024
	movls	r3, #5120
	cmp	r2, r3
	movls	r4, r7
	bls	.L2160
	mov	r2, #444
	mov	r3, #0
	str	r3, [r5, #-1184]
	mov	r6, r5
	strh	r3, [r5, r2]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r7
	mov	r4, r0
	bne	.L2162
	ldr	r3, .L2246+32
	ldrh	r1, [r5, r8]
	ldrh	r2, [r5, r3]
	cmp	r1, r2
	movcs	r2, #80
	strcsh	r2, [r5, r3]	@ movhi
	bcs	.L2172
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r4
	beq	.L2172
	ldr	r2, [r5, #-988]
	cmp	r2, #0
	bne	.L2165
	ldr	r2, .L2246+20
	mov	r1, #3872
	ldrh	r1, [r2, r1]
	cmp	r1, #3
	beq	.L2165
	ldr	r1, [r5, #-1704]
	cmp	r1, #0
	bne	.L2165
	ldr	r1, [r2, #3844]
	cmp	r1, #0
	bne	.L2165
	ldrb	r0, [r2, #3072]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L2166
.L2165:
	ldr	r1, .L2246
	mov	r3, r3, asl #1
	movw	ip, #3852
	mov	lr, #3872
	ldr	r2, [r1, #-2012]
	ldrh	r0, [r2, r3]
	mov	r2, #3920
	ldr	r3, .L2246+20
	ldrh	r2, [r3, r2]
	ldrh	ip, [r3, ip]
	ldrh	r3, [r3, lr]
	mul	ip, ip, r2
	cmp	r3, #3
	moveq	r3, r2, lsr #1
	movne	r3, #0
	add	r3, ip, r3
	cmp	r0, r3
	bgt	.L2168
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r2, .L2246+20
	ldr	r3, .L2246
	ldr	r2, [r2, #3980]
	ldr	r1, [r3, #-1840]
	add	r2, r2, r2, asl #1
	cmp	r1, r2, lsr #2
	ldr	r2, .L2246+32
	movhi	r1, #128
	movls	r1, #160
	strh	r1, [r3, r2]	@ movhi
	uxth	r4, r0
	b	.L2170
.L2168:
	ldr	r3, .L2246+32
	mov	r2, #128
	strh	r2, [r1, r3]	@ movhi
	b	.L2172
.L2166:
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	ldr	r6, .L2246
	ldr	r5, .L2246+32
	ldrh	r3, [r2, r3]
	cmp	r3, #7
	movhi	r3, #64
	strhih	r3, [r6, r5]	@ movhi
	bhi	.L2172
	bl	List_get_gc_head_node
	mov	r3, #128
	strh	r3, [r6, r5]	@ movhi
	uxth	r4, r0
.L2170:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L2172
.L2162:
	ldr	r1, .L2246
	mov	r0, r4, asl #1
	ldr	r3, .L2246+28
	ldr	ip, .L2246+20
	ldrh	r2, [r1, r3]
	ldr	ip, [ip, #4052]
	ldr	r3, [r1, #-2012]
	ldrh	r3, [r3, r0]
	ldrh	r0, [ip, r0]
	str	r0, [sp, #0]
	ldr	r0, .L2246+44
	ldrh	r1, [r1, r0]
	ldr	r0, .L2246+48
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L2172:
	bl	FtlGcReFreshBadBlk
.L2160:
	movw	ip, #65535
	rsb	r1, ip, r4
	rsbs	r0, r1, #0
	ldr	r2, [sp, #16]
	adc	r0, r0, r1
	ldr	r3, .L2246
	cmp	r2, #0
	movne	r1, #0
	andeq	r1, r0, #1
	cmp	r1, #0
	beq	.L2173
	ldr	r2, .L2246+28
	ldrh	r1, [r3, r2]
	cmp	r1, #24
	movhi	r6, #1
	bhi	.L2174
	ldr	r0, .L2246+20
	movw	r2, #3918
	cmp	r1, #16
	ldrh	r6, [r0, r2]
	movhi	r6, r6, lsr #5
	bhi	.L2174
	cmp	r1, #12
	movhi	r6, r6, lsr #4
	bhi	.L2174
	cmp	r1, #8
	movhi	r6, r6, lsr #2
.L2174:
	ldr	ip, .L2246+44
	ldr	r2, .L2246
	ldrh	r3, [r3, ip]
	cmp	r3, r1
	mov	r1, ip
	bcs	.L2178
	ldr	r3, .L2246+52
	movw	r0, #65535
	ldrh	r3, [r2, r3]
	cmp	r3, r0
	bne	.L2179
	ldr	r0, .L2246+8
	ldrh	r0, [r2, r0]
	cmp	r0, r3
	bne	.L2179
	mov	r3, #444
	ldrh	r0, [r2, r3]
	cmp	r0, #0
	bne	.L2180
	ldr	r3, .L2246+20
	ldr	r1, [r2, #-1840]
	ldr	r3, [r3, #3980]
	add	r3, r3, r3, asl #1
	cmp	r1, r3, lsr #2
	movcs	r3, #18
	strcsh	r3, [r2, ip]	@ movhi
	bcs	.L2182
.L2180:
	ldr	r3, .L2246
	ldr	r2, .L2246+56
	ldr	r1, .L2246+44
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L2182:
	ldr	r3, .L2246
	mov	r2, #0
	str	r2, [r3, #-1176]
	b	.L2183
.L2179:
	ldr	r3, .L2246
	ldr	r2, .L2246+56
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L2178:
	ldr	r3, .L2246+20
	cmp	sl, #2
	movw	r4, #65535
	movhi	sl, #0
	movls	sl, #1
	ldr	r3, [r3, #3844]
	cmp	r3, #0
	moveq	sl, #0
	cmp	sl, #0
	addne	r6, r6, #1
	uxthne	r6, r6
	b	.L2184
.L2173:
	ldr	r2, .L2246+52
	ldrh	r2, [r3, r2]
	cmp	r2, ip
	bne	.L2185
	ldr	ip, .L2246+8
	ldrh	ip, [r3, ip]
	cmp	ip, r2
	movne	r0, #0
	andeq	r0, r0, #1
	cmp	r0, #0
	beq	.L2185
	ldr	r0, .L2246+24
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	movne	r4, r2
	bne	.L2185
	str	r1, [r3, #-1176]
	ldr	r2, .L2246+44
	ldr	r1, .L2246+28
	ldrh	r0, [r3, r1]
	ldrh	r1, [r3, r2]
	cmp	r0, r1
	bls	.L2186
	mov	r1, #444
	ldrh	r1, [r3, r1]
	cmp	r1, #0
	bne	.L2187
	ldr	r1, .L2246+20
	ldr	r0, [r3, #-1840]
	ldr	r1, [r1, #3980]
	add	r1, r1, r1, asl #1
	cmp	r0, r1, lsr #2
	movcs	r1, #18
	strcsh	r1, [r3, r2]	@ movhi
	bcs	.L2189
.L2187:
	ldr	r3, .L2246
	ldr	r2, .L2246+56
	ldr	r1, .L2246+44
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L2189:
	bl	FtlReadRefresh
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, .L2246
	ldr	r2, [r3, #-2012]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r2, [r2, r0]
	cmp	r2, #4
	bhi	.L2245
.L2186:
	ldr	r4, .L2246
	mov	r6, #444
	ldrh	r0, [r4, r6]
	cmp	r0, #0
	bne	.L2237
	ldr	r3, .L2246+56
	ldrh	r5, [r4, r3]
	add	r3, r3, #584
	add	r2, r5, r5, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r4, r3]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r4, #-2012]
	ldr	r1, .L2246+20
	mov	r2, #3920
	ldrh	r2, [r1, r2]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r3, [r3, r0]
	movw	r0, #3852
	ldrh	r1, [r1, r0]
	mul	r2, r1, r2
	add	r2, r2, r2, lsr #31
	cmp	r3, r2, asr #1
	ble	.L2190
	ldr	r2, .L2246+28
	sub	r5, r5, #1
	ldrh	r2, [r4, r2]
	cmp	r2, r5
	blt	.L2190
	bl	FtlReadRefresh
	ldrh	r0, [r4, r6]
	b	.L2183
.L2190:
	cmp	r3, #0
	bne	.L2237
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r3, .L2246+28
	ldr	r2, .L2246
	ldrh	r0, [r2, r3]
	add	r0, r0, #1
	b	.L2183
.L2237:
	movw	r4, #65535
.L2185:
	ldr	r3, .L2246+20
	ldr	r6, [r3, #3844]
	cmp	r6, #0
	movne	r6, #2
	moveq	r6, #1
.L2184:
	ldr	r3, .L2246
	movw	r0, #65535
	ldr	r2, .L2246+24
	ldrh	r1, [r3, r2]
	cmp	r1, r0
	bne	.L2191
	cmp	r4, r1
	strneh	r4, [r3, r2]	@ movhi
	bne	.L2193
	ldr	r2, .L2246+8
	ldrh	r1, [r3, r2]
	cmp	r1, r4
	beq	.L2193
	ldr	r0, [r3, #-2012]
	mov	r1, r1, asl #1
	ldrh	r1, [r0, r1]
	cmp	r1, #0
	mvneq	r1, #0
	streqh	r1, [r3, r2]	@ movhi
	ldr	r3, .L2246
	ldr	r1, .L2246+24
	ldrh	r0, [r3, r2]
	strh	r0, [r3, r1]	@ movhi
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
.L2193:
	ldr	r5, .L2246
	mov	r3, #0
	ldr	r7, .L2246+24
	strb	r3, [r5, #-1744]
	movw	r3, #65535
	ldrh	r0, [r5, r7]
	cmp	r0, r3
	beq	.L2191
	bl	IsBlkInGcList
	cmp	r0, #0
	mvnne	r3, #0
	strneh	r3, [r5, r7]	@ movhi
	ldr	r3, .L2246+20
	ldr	r7, .L2246
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2196
	ldr	r3, .L2246+24
	ldrh	r0, [r7, r3]
	bl	ftl_get_blk_mode
	strb	r0, [r7, #-1744]
.L2196:
	ldr	r8, .L2246+24
	movw	r3, #65535
	ldr	r5, .L2246
	ldrh	r2, [r7, r8]
	cmp	r2, r3
	beq	.L2191
	ldr	r0, .L2246+60
	bl	make_superblock
	ldr	r1, .L2246+64
	movw	r2, #446
	mov	r3, #0
	strh	r3, [r5, r2]	@ movhi
	strh	r3, [r5, r1]	@ movhi
	strb	r3, [r5, #-1746]
	ldrh	r3, [r5, r8]
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	mov	r3, #448
	strh	r2, [r5, r3]	@ movhi
.L2191:
	ldr	r3, .L2246
	ldr	r0, .L2246+24
	ldr	r2, .L2246+68
	ldrh	r1, [r3, r0]
	ldrh	r2, [r3, r2]
	cmp	r2, r1
	mov	r2, r3
	beq	.L2197
	ldr	ip, .L2246+72
	ldrh	r3, [r3, ip]
	cmp	r3, r1
	movne	fp, r4
	bne	.L2241
.L2198:
.L2197:
	mvn	r3, #0
	strh	r3, [r2, r0]	@ movhi
	mov	r3, #444
.L2244:
	ldrh	r0, [r2, r3]
	b	.L2183
.L2241:
	ldr	r4, .L2246
	movw	sl, #65535
	ldr	r9, .L2246+24
	ldrh	r3, [r4, r9]
	cmp	r3, sl
	bne	.L2200
	ldr	r5, .L2246+36
	mov	r3, #0
	str	r3, [r4, #-1176]
.L2240:
	ldrh	r8, [r4, r5]
	mov	r0, r8
	bl	List_get_gc_head_node
	movw	r1, #65535
	uxth	r7, r0
	strh	r7, [r4, r9]	@ movhi
	cmp	r7, r1
	bne	.L2202
	ldr	r3, .L2246+36
	mov	r1, #0
	ldr	r2, .L2246
	mov	r0, #8
	strh	r1, [r2, r3]	@ movhi
	b	.L2183
.L2202:
	mov	r0, r7
	bl	IsBlkInGcList
	cmp	r0, #0
	add	r0, r8, #1
	strneh	r0, [r4, r5]	@ movhi
	bne	.L2240
	ldr	ip, .L2246+20
	movw	r8, #3918
	ldr	r2, [r4, #-2012]
	mov	r3, r7, asl #1
	uxth	r0, r0
	strh	r0, [r4, r5]	@ movhi
	ldrh	sl, [ip, r8]
	movw	r8, #3852
	ldrh	ip, [ip, r8]
	ldrh	r1, [r2, r3]
	mul	sl, ip, sl
	add	ip, sl, sl, lsr #31
	cmp	r1, ip, asr #1
	bgt	.L2205
	cmp	r0, #48
	bls	.L2206
	cmp	r1, #8
	bls	.L2206
	ldr	r1, .L2246+76
	ldrh	r1, [r4, r1]
	cmp	r1, #35
	bhi	.L2206
.L2205:
	mov	r1, #0
	strh	r1, [r4, r5]	@ movhi
.L2206:
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, sl
	cmpge	fp, r3
	ldr	r3, .L2246
	bne	.L2207
	ldrh	r1, [r4, r5]
	cmp	r1, #3
	bhi	.L2207
	ldr	r2, .L2246+24
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #572
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
.L2245:
	mov	r2, #444
	b	.L2244
.L2207:
	cmp	r2, #0
	ldr	r8, .L2246
	bne	.L2208
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r4, r5]
	add	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	b	.L2240
.L2208:
	mov	r2, #0
	strb	r2, [r3, #-1744]
	ldr	r3, .L2246+20
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, r2
	beq	.L2209
	mov	r0, r7
	bl	ftl_get_blk_mode
	strb	r0, [r8, #-1744]
.L2209:
	ldr	r0, .L2246+60
	bl	make_superblock
	movw	r1, #446
	mov	r3, #0
	strh	r3, [r8, r1]	@ movhi
	ldr	r1, .L2246+24
	ldr	r0, [r8, #-2012]
	ldrh	r1, [r8, r1]
	mov	r1, r1, asl #1
	ldrh	r0, [r0, r1]
	mov	r1, #448
	strb	r3, [r8, #-1746]
	strh	r0, [r8, r1]	@ movhi
	ldr	r1, .L2246+64
	strh	r3, [r8, r1]	@ movhi
.L2200:
	ldr	sl, [sp, #16]
	cmp	sl, #1
	bne	.L2210
	bl	FtlReadRefresh
.L2210:
	ldr	r3, .L2246
	mov	r2, #1
	movw	r1, #3918
	str	r2, [r3, #-984]
	ldr	r2, .L2246+20
	ldrh	r9, [r2, r1]
	ldrb	r1, [r2, #3072]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L2211
	ldrb	r1, [r3, #-1744]	@ zero_extendqisi2
	cmp	r1, #1
	moveq	r1, #3920
	ldreqh	r9, [r2, r1]
.L2211:
	ldr	r2, .L2246+64
	mov	r8, #0
	str	fp, [sp, #20]
	mov	fp, r9
	ldr	r4, .L2246
	ldrh	r3, [r3, r2]
	add	r2, r3, r6
	cmp	r2, r9
	rsbgt	r6, r3, r9
	uxthgt	r6, r6
	b	.L2213
.L2215:
	ldrh	r1, [r2, #2]!
	movw	sl, #65535
	add	r3, r3, #1
	cmp	r1, sl
	uxth	r3, r3
	orrne	r1, ip, r1, asl #10
	mlane	sl, r0, r7, lr
	addne	r7, r7, #1
	uxthne	r7, r7
	strne	r1, [sl, #4]
.L2221:
	cmp	r3, r5
	bne	.L2215
	ldr	r0, [r4, #-1156]
	mov	r1, r7
	ldrb	r2, [r4, #-1744]	@ zero_extendqisi2
	mov	r5, #0
	bl	FlashReadPages
	mov	r9, r5
	mov	sl, r6
	b	.L2216
.L2219:
	ldr	r3, [r4, #-1156]
	add	r2, r3, r5
	ldr	r3, [r3, r5]
	ldr	r6, [r2, #12]
	cmn	r3, #1
	beq	.L2217
	ldrh	r3, [r6, #0]
	movw	r1, #61589
	cmp	r3, r1
	bne	.L2217
	add	r1, sp, #28
	mov	r2, #0
	ldr	r0, [r6, #8]
	bl	log2phys
	ldr	r1, [r4, #-1156]
	ldr	r2, [sp, #28]
	add	r1, r1, r5
	bic	r2, r2, #-2147483648
	ldr	r3, [r1, #4]
	cmp	r2, r3
	bne	.L2217
	movw	r3, #446
	ldr	r0, [r4, #-1172]
	ldrh	r2, [r4, r3]
	ldr	r1, [r1, #16]
	add	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	ldr	r2, [r4, #-956]
	mov	r3, #36
	mla	r2, r3, r0, r2
	str	r1, [r2, #16]
	str	r2, [sp, #8]
	str	r3, [sp, #12]
	bl	Ftl_get_new_temp_ppa
	ldr	r2, [sp, #8]
	ldr	r1, [r4, #-1172]
	str	r0, [r2, #4]
	ldr	r3, [sp, #12]
	ldr	r2, [r4, #-956]
	mla	r3, r3, r1, r2
	ldr	r2, [r4, #-1156]
	add	r2, r2, r5
	ldr	r1, [r2, #8]
	str	r1, [r3, #8]
	mov	r1, #1
	ldr	r2, [r2, #12]
	str	r2, [r3, #12]
	ldr	r3, [sp, #28]
	str	r3, [r6, #12]
	ldr	r3, .L2246+52
	ldrh	r3, [r4, r3]
	strh	r3, [r6, #2]	@ movhi
	ldr	r3, [r4, #-1796]
	ldr	r0, [r4, #-1156]
	str	r3, [r6, #4]
	add	r0, r0, r5
	ldr	r3, [r4, #-1172]
	add	r3, r3, #1
	str	r3, [r4, #-1172]
	bl	FtlGcBufAlloc
	ldr	r3, .L2246+20
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2218
	ldrb	r3, [r4, #-1889]	@ zero_extendqisi2
	ldr	r2, [r4, #-1172]
	cmp	r2, r3
	beq	.L2218
	ldr	r3, .L2246+80
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L2217
.L2218:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	ldrne	r3, .L2246
	movne	r2, #0
	strne	r2, [r3, #-984]
	bne	.L2245
.L2217:
	add	r9, r9, #1
	add	r5, r5, #36
	uxth	r9, r9
.L2216:
	cmp	r9, r7
	bne	.L2219
	add	r8, r8, #1
	mov	r6, sl
.L2213:
	uxth	r3, r8
	ldr	r7, .L2246
	cmp	r3, r6
	ldr	r3, .L2246+64
	bcs	.L2220
	ldr	r1, .L2246+20
	movw	r2, #3852
	ldrh	ip, [r4, r3]
	mov	r7, #0
	ldr	lr, [r4, #-1156]
	mov	r3, r7
	ldrh	r5, [r1, r2]
	add	ip, ip, r8
	ldr	r2, .L2246+84
	mov	r0, #36
	b	.L2221
.L2220:
	ldrh	r2, [r7, r3]
	mov	r9, fp
	ldr	fp, [sp, #20]
	add	r6, r6, r2
	uxth	r6, r6
	strh	r6, [r7, r3]	@ movhi
	cmp	r6, r9
	bcc	.L2222
	ldr	r3, [r7, #-1172]
	cmp	r3, #0
	beq	.L2223
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	movne	r3, #0
	strne	r3, [r7, #-984]
	movne	r3, #444
	ldrneh	r0, [r7, r3]
	bne	.L2183
.L2223:
	ldr	r4, .L2246
	movw	r3, #446
	ldrh	r6, [r4, r3]
	cmp	r6, #0
	bne	.L2224
	ldr	r5, .L2246+24
	ldr	r2, [r4, #-2012]
	ldrh	r1, [r4, r5]
	mov	r3, r1, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2224
	ldr	r2, .L2246+64
	ldr	r0, .L2246+88
	ldrh	r2, [r4, r2]
	str	r3, [sp, #0]
	mov	r3, #448
	ldrh	r3, [r4, r3]
	str	r3, [sp, #4]
	mov	r3, r6
	bl	printk
	ldrh	r3, [r4, r5]
	ldr	r2, [r4, #-2012]
	mov	r3, r3, asl #1
	strh	r6, [r2, r3]	@ movhi
	ldrh	r0, [r4, r5]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2224:
	ldr	r3, .L2246+24
	mvn	r1, #0
	ldr	r2, .L2246
	strh	r1, [r2, r3]	@ movhi
.L2222:
	ldr	r3, .L2246
	ldr	r2, .L2246+28
	ldrh	r2, [r3, r2]
	cmp	r2, #2
	ldrls	r2, .L2246+20
	movwls	r3, #3918
	ldrlsh	r6, [r2, r3]
	bls	.L2241
.L2225:
	mov	r1, #0
	str	r1, [r3, #-984]
	mov	r1, #444
	ldrh	r0, [r3, r1]
	cmp	r0, #0
	addeq	r0, r2, #1
.L2183:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2247:
	.align	2
.L2246:
	.word	.LANCHOR2
	.word	-1132
	.word	-1134
	.word	-1130
	.word	-1128
	.word	.LANCHOR0
	.word	-1752
	.word	-1996
	.word	-1186
	.word	-1180
	.word	.LC149
	.word	-1188
	.word	.LC150
	.word	-1896
	.word	-1772
	.word	.LANCHOR2-1752
	.word	-1750
	.word	-1992
	.word	-1944
	.word	-1148
	.word	-1892
	.word	.LANCHOR2-1738
	.word	.LC151
	.fnend
	.size	ftl_do_gc.part.24, .-ftl_do_gc.part.24
	.align	2
	.global	ftl_do_gc
	.type	ftl_do_gc, %function
ftl_do_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, .L2255
	mov	r5, r1
	ldr	r0, [r3, #4044]
	cmp	r0, #0
	movne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, pc}
	ldr	r3, .L2255+4
	ldr	r2, [r3, #-984]
	cmp	r2, #0
	ldmnefd	sp!, {r3, r4, r5, pc}
	ldr	r1, .L2255+8
	ldrh	r1, [r3, r1]
	cmp	r1, #47
	movls	r0, r2
	ldmlsfd	sp!, {r3, r4, r5, pc}
	ldr	r1, .L2255+12
	movw	r2, #3224
	ldrh	r1, [r1, r2]
	movw	r2, #65535
	cmp	r1, r2
	beq	.L2250
	ldr	r1, .L2255+16
	ldrh	r3, [r3, r1]
	cmp	r3, r2
	beq	.L2250
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	bne	.L2254
.L2250:
	mov	r0, r4
	mov	r1, r5
	ldmfd	sp!, {r3, r4, r5, lr}
	b	ftl_do_gc.part.24
.L2254:
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, pc}
.L2256:
	.align	2
.L2255:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-2004
	.word	.LANCHOR1
	.word	-1896
	.fnend
	.size	ftl_do_gc, .-ftl_do_gc
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2280
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	ldr	r4, [r3, #452]
	ldr	r3, .L2280+4
	ldr	r2, [r3, #4044]
	cmp	r2, #0
	bne	.L2258
	ldr	r1, [r3, #3972]
	cmp	r1, #0
	beq	.L2258
	ldrb	r8, [r3, #3072]	@ zero_extendqisi2
	cmp	r8, #0
	beq	.L2259
	ldrb	r8, [r4, #8]	@ zero_extendqisi2
	sub	r0, r8, #1
	rsbs	r8, r0, #0
	adc	r8, r8, r0
.L2259:
	ldr	r5, .L2280+4
	mov	r2, r8
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	mov	r6, #0
	mov	r7, r6
	ldr	r9, .L2280
	ldr	r0, [r5, #3976]
	bl	FlashProgPages
	b	.L2260
.L2265:
	ldr	r2, [r5, #3976]
	add	r3, r2, r6
	ldr	r2, [r2, r6]
	cmn	r2, #1
	beq	.L2279
	ldr	r2, [r3, #4]
	cmp	r8, #0
	ldr	r0, [r3, #16]
	add	r1, sp, #4
	str	r2, [sp, #4]
	orrne	r2, r2, #-2147483648
	strne	r2, [sp, #4]
	mov	r2, #1
	bl	log2phys
	ldr	r3, [r5, #3976]
	add	r3, r3, r6
	ldr	r3, [r3, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L2263
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r9, #-2012]
	mov	r3, r0, asl #1
	mov	sl, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L2264
	ldr	r0, .L2280+8
	mov	r1, sl
	bl	printk
.L2264:
	mov	r0, sl
	bl	decrement_vpc_count
.L2263:
	add	r7, r7, #1
	add	r6, r6, #36
.L2260:
	ldr	r3, [r5, #3972]
	cmp	r7, r3
	bcc	.L2265
	b	.L2266
.L2275:
	ldr	r3, [r5, #3976]
	mvn	r2, #0
	ldr	r7, .L2280
	str	r2, [r3, r6]
	b	.L2267
.L2271:
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #0]
	cmp	r3, r0
	bne	.L2268
	ldr	r2, [r7, #-2012]
	mov	r3, r3, asl #1
	ldrh	r1, [r4, #4]
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	movw	r3, #3918
	ldrh	r3, [r5, r3]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
.L2268:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L2269
	mov	r0, r4
	bl	allocate_new_data_superblock
.L2269:
	ldr	r3, [r7, #-1604]
	add	r3, r3, #1
	str	r3, [r7, #-1604]
	ldr	r3, [r5, #3976]
	add	r3, r3, r6
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	FtlGcMarkBadPhyBlk
	mov	r0, r4
	bl	get_new_active_ppa
	mov	r1, #1
	mov	r2, r8
	mov	r3, r0
	str	r0, [sp, #4]
	ldr	r0, [r5, #3976]
	add	r0, r0, r6
	str	r3, [r0, #4]
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	bl	FlashProgPages
	ldr	r3, [r5, #3976]
	ldr	r3, [r3, r6]
	cmn	r3, #1
	moveq	r3, #1
	streq	r3, [r5, #4044]
	ldr	r3, [r9, #4044]
	cmp	r3, #0
	bne	.L2258
.L2267:
	ldr	r2, [r5, #3976]
	add	r3, r2, r6
	ldr	r2, [r2, r6]
	cmn	r2, #1
	beq	.L2271
	ldr	r2, [r3, #4]
	cmp	r8, #0
	ldr	r0, [r3, #16]
	add	r1, sp, #4
	str	r2, [sp, #4]
	orrne	r2, r2, #-2147483648
	strne	r2, [sp, #4]
	mov	r2, #1
	bl	log2phys
	ldr	r3, [r5, #3976]
	add	r3, r3, r6
	ldr	r3, [r3, #12]
	ldr	r0, [r3, #12]
	cmn	r0, #1
	beq	.L2273
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L2280
	ldr	r2, [r3, #-2012]
	mov	r3, r0, asl #1
	mov	r7, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L2274
	ldr	r0, .L2280+8
	mov	r1, r7
	bl	printk
.L2274:
	mov	r0, r7
	bl	decrement_vpc_count
.L2273:
	add	sl, sl, #1
	add	r6, r6, #36
	b	.L2261
.L2279:
	ldr	r5, .L2280+4
	mov	r6, #0
	mov	sl, r6
	mov	r9, r5
.L2261:
	ldr	r3, [r5, #3972]
	cmp	sl, r3
	bcc	.L2275
	movw	r4, #16386
	ldr	r6, .L2280
	ldr	r5, .L2280+12
	b	.L2276
.L2277:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	beq	.L2266
.L2276:
	ldrh	r3, [r6, r5]
	cmp	r3, #0
	bne	.L2277
.L2266:
	ldr	r3, .L2280+4
	mov	r2, #0
	str	r2, [r3, #3972]
.L2258:
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L2281:
	.align	2
.L2280:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC152
	.word	-1126
	.fnend
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
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
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L2286
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L2285
	bl	FtlCacheWriteBack
	bl	l2p_flush
	mov	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2285:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2287:
	.align	2
.L2286:
	.word	.LANCHOR0
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
	ldr	r3, .L2290
	ldr	r3, [r3, #3220]
	cmp	r3, #1
	bne	.L2289
	bl	FtlSysFlush
.L2289:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2291:
	.align	2
.L2290:
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
	ldr	r0, .L2294
	bl	printk
	ldmfd	sp!, {r3, lr}
	b	ftl_deinit
.L2295:
	.align	2
.L2294:
	.word	.LC153
	.fnend
	.size	rk_ftl_de_init, .-rk_ftl_de_init
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
	.global	ftl_fix_nand_power_lost_error
	.type	ftl_fix_nand_power_lost_error, %function
ftl_fix_nand_power_lost_error:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2306
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	.pad #52
	sub	sp, sp, #52
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2297
	ldr	r4, .L2306+4
	movw	r3, #306
	ldr	r0, .L2306+8
	movw	r7, #4097
	ldrh	r6, [r4, r3]
	ldr	r3, [r4, #-2012]
	mov	r5, r6, asl #1
	mov	r1, r6
	ldrh	r2, [r3, r5]
	bl	printk
	ldr	r3, .L2306+12
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshOpenBlock
	ldr	r3, .L2306+16
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshOpenBlock
	ldr	r0, .L2306+20
	bl	allocate_new_data_superblock
	ldr	r0, .L2306+24
	bl	allocate_new_data_superblock
	b	.L2299
.L2301:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	ldr	r3, [r4, #-2012]
	ldrh	r3, [r3, r5]
	cmp	r3, #0
	beq	.L2300
.L2299:
	subs	r7, r7, #1
	bne	.L2301
.L2300:
	ldr	r4, .L2306+4
	mov	r1, r6
	ldr	r0, .L2306+8
	ldr	r3, [r4, #-2012]
	ldrh	r2, [r3, r5]
	bl	printk
	ldr	r3, [r4, #-2012]
	ldrh	r4, [r3, r5]
	cmp	r4, #0
	bne	.L2302
	add	r7, sp, #48
	mov	r0, sp
	strh	r6, [r7, #-48]!	@ movhi
	add	r7, r7, #14
	bl	make_superblock
	ldr	r3, .L2306
	movw	r2, #3852
	movw	lr, #65535
	mov	ip, #36
	mov	r1, r4
	ldrh	r8, [r3, r2]
	ldr	sl, [r3, #4048]
	mov	r3, r4
	b	.L2303
.L2305:
	ldrh	r0, [r7, #2]!
	cmp	r0, lr
	beq	.L2304
	mla	r2, ip, r4, sl
	add	r4, r4, #1
	mov	r0, r0, asl #10
	uxth	r4, r4
	stmib	r2, {r0, r1}
	str	r1, [r2, #12]
.L2304:
	add	r3, r3, #1
	uxth	r3, r3
.L2303:
	cmp	r3, r8
	bne	.L2305
	ldr	r3, .L2306+4
	mov	r1, r6
	ldr	r0, .L2306+28
	ldr	r3, [r3, #-2012]
	ldrh	r2, [r3, r5]
	ldr	r5, .L2306
	bl	printk
	mov	r1, #0
	mov	r2, r4
	ldr	r0, [r5, #4048]
	bl	FlashEraseBlocks
	ldr	r0, [r5, #4048]
	mov	r1, #1
	mov	r2, r4
	bl	FlashEraseBlocks
.L2302:
	ldr	r2, .L2306+4
	movw	r3, #306
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L2297:
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L2307:
	.align	2
.L2306:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC154
	.word	-1992
	.word	-1944
	.word	.LANCHOR2-1992
	.word	.LANCHOR2-1944
	.word	.LC155
	.fnend
	.size	ftl_fix_nand_power_lost_error, .-ftl_fix_nand_power_lost_error
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.L2309:
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mvn	r3, #0
	ldr	r4, .L2319
	ldr	r6, .L2319+4
	ldr	r5, .L2319+8
	ldr	r1, .L2319+12
	str	r3, [r6, #3220]
	mov	r3, #0
	ldr	r0, .L2319+16
	str	r3, [r4, #4044]
	str	r3, [r5, #456]
	bl	printk
	ldr	r0, .L2319+20
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldr	r0, [r4, #3856]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	ldrne	r0, .L2319+24
	bne	.L2318
	bl	FtlSysBlkInit
	cmp	r0, #0
	beq	.L2312
	ldr	r0, .L2319+28
.L2318:
	ldr	r1, .L2319+32
	bl	printk
	b	.L2311
.L2312:
	mov	r1, #1
	str	r1, [r6, #3220]
	bl	ftl_do_gc
	ldr	r3, .L2319+36
	ldrh	r3, [r5, r3]
	cmp	r3, #15
	bhi	.L2313
	mov	r4, #1024
.L2314:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	bne	.L2314
	b	.L2311
.L2313:
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2311
	mov	r4, #128
.L2315:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	bne	.L2315
.L2311:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2320:
	.align	2
.L2319:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LC78
	.word	.LC77
	.word	.LANCHOR0+3044
	.word	.LC156
	.word	.LC157
	.word	.LANCHOR3+69
	.word	-1996
	.fnend
	.size	FtlInit, .-FtlInit
	.align	2
	.global	ftl_discard
	.type	ftl_discard, %function
ftl_discard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, lr}
	add	r2, r1, r0
	ldr	r5, .L2331
	mov	r7, r0
	mov	r4, r1
	ldr	r3, [r5, #3960]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L2322
	cmp	r1, #31
	bls	.L2329
	ldr	r3, [r5, #4044]
	cmp	r3, #0
	bne	.L2329
	bl	FtlCacheWriteBack
	movw	r3, #3924
	ldrh	r6, [r5, r3]
	mov	r0, r7
	mov	r1, r6
	bl	__aeabi_uidiv
	mls	r7, r0, r6, r7
	mov	r5, r0
	uxth	r7, r7
	cmp	r7, #0
	beq	.L2323
	rsb	r6, r7, r6
	add	r5, r0, #1
	cmp	r6, r4
	movcs	r6, r4
	uxth	r6, r6
	rsb	r4, r6, r4
.L2323:
	mvn	r3, #0
	ldr	r7, .L2331
	str	r3, [sp, #4]
	movw	r6, #3924
	b	.L2324
.L2326:
	mov	r0, r5
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #0]
	cmn	r3, #1
	beq	.L2325
	ldr	r3, .L2331+4
	add	r1, sp, #4
	mov	r0, r5
	ldr	r2, [r3, #460]
	add	r2, r2, #1
	str	r2, [r3, #460]
	ldr	r2, [r3, #-1828]
	add	r2, r2, #1
	str	r2, [r3, #-1828]
	mov	r2, #1
	bl	log2phys
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L2325:
	ldrh	r3, [r7, r6]
	add	r5, r5, #1
	rsb	r4, r3, r4
.L2324:
	ldrh	r3, [r7, r6]
	cmp	r4, r3
	bcs	.L2326
	ldr	r3, .L2331+4
	mov	r4, #0
	ldr	r2, [r3, #460]
	cmp	r2, #32
	bls	.L2330
	str	r4, [r3, #460]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L2330
.L2329:
	mov	r0, #0
	b	.L2322
.L2330:
	mov	r0, r4
.L2322:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L2332:
	.align	2
.L2331:
	.word	.LANCHOR0
	.word	.LANCHOR2
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
	.type	ftl_write.part.25, %function
ftl_write.part.25:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r3, #2048
	ldr	r4, .L2379
	.pad #100
	sub	sp, sp, #100
	ldr	r8, .L2379+4
	mov	r5, r1
	mov	r7, r0
	mov	r9, r2
	str	r3, [r4, #464]
	movw	r3, #3924
	ldrh	r6, [r8, r3]
	mov	r1, r6
	bl	__aeabi_uidiv
	mov	r1, r6
	str	r0, [sp, #16]
	sub	r0, r7, #1
	add	r0, r0, r5
	bl	__aeabi_uidiv
	ldr	r3, [sp, #16]
	ldr	sl, [r8, #3972]
	cmp	sl, #0
	rsb	fp, r3, r0
	str	r0, [sp, #36]
	add	r3, fp, #1
	str	r3, [sp, #20]
	ldr	lr, [sp, #20]
	ldr	r3, [r4, #-1824]
	add	r3, lr, r3
	str	r3, [r4, #-1824]
	ldr	r3, [r4, #-1808]
	add	r3, r5, r3
	str	r3, [r4, #-1808]
	beq	.L2367
	ldr	r3, [r8, #3976]
	sub	sl, sl, #1
	mov	r2, #36
	ldr	r0, [sp, #16]
	mla	sl, r2, sl, r3
	ldr	r3, [sl, #16]
	cmp	r0, r3
	strne	r5, [sp, #32]
	bne	.L2336
	ldr	r3, [r4, #-1820]
	mov	r0, r7
	mov	r1, r6
	add	r3, r3, #1
	str	r3, [r4, #-1820]
	ldr	r3, [r4, #468]
	add	r3, r3, #1
	str	r3, [r4, #468]
	bl	__aeabi_uidivmod
	ldr	r0, [sl, #8]
	rsb	r8, r1, r6
	add	r0, r0, r1, asl #9
	cmp	r8, r5
	movcs	r8, r5
	mov	r1, r9
	mov	r3, r8, asl #9
	str	r3, [sp, #4]
	mov	r2, r3
	bl	memcpy
	cmp	fp, #0
	ldr	r3, [sp, #4]
	bne	.L2337
	ldr	r2, [r4, #468]
	cmp	r2, #2
	ble	.L2338
.L2337:
	add	r9, r9, r3
	add	r7, r7, r8
	ldr	r3, [sp, #16]
	rsb	r1, r8, r5
	str	fp, [sp, #20]
	add	r3, r3, #1
	str	r1, [sp, #32]
	str	r3, [sp, #16]
.L2336:
	ldr	r3, .L2379
	mov	r2, #0
	str	r2, [r3, #468]
	b	.L2335
.L2367:
	str	r5, [sp, #32]
.L2335:
	ldr	r0, [sp, #16]
	ldr	r1, [sp, #36]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L2339
	bl	FtlCacheWriteBack
.L2339:
	cmp	r5, r6, asl #1
	ldr	r3, .L2379
	ldr	r4, .L2379+4
	mov	fp, #0
	movcc	r6, #0
	movcs	r6, #1
	str	r6, [sp, #24]
	ldr	r6, .L2379+8
	ldr	r5, [sp, #16]
	mov	sl, r4
	str	fp, [sp, #40]
	str	r6, [r3, #452]
	b	.L2376
.L2365:
	ldrh	r1, [r6, #4]
	cmp	r1, #0
	bne	.L2341
	ldr	r2, .L2379+8
	ldr	r3, .L2379+12
	cmp	r6, r2
	bne	.L2342
	ldr	r2, .L2379+16
	ldrh	r8, [r8, r2]
	cmp	r8, #0
	bne	.L2343
	ldr	r0, .L2379+20
	str	r3, [sp, #4]
	bl	allocate_new_data_superblock
	ldr	r3, [sp, #4]
	str	r8, [r3, #3228]
.L2343:
	ldr	r0, .L2379+8
	str	r3, [sp, #4]
	bl	allocate_new_data_superblock
	ldr	r3, [sp, #4]
	ldr	r2, [r3, #3228]
	ldr	r3, .L2379+20
	cmp	r2, #0
	movne	r6, r3
	b	.L2344
.L2342:
	str	r1, [r3, #3228]
	ldr	r3, .L2379+24
	ldrh	r3, [r8, r3]
	cmp	r3, #0
	movne	r6, r2
	bne	.L2344
	mov	r0, r6
	bl	allocate_new_data_superblock
.L2344:
	ldrh	r3, [r6, #4]
	cmp	r3, #0
	bne	.L2345
	mov	r0, r6
	bl	allocate_new_data_superblock
.L2345:
	ldr	r3, .L2379
	str	r6, [r3, #452]
.L2341:
	ldr	r2, .L2379
	mov	r8, r6
	ldrh	r3, [r6, #4]
	ldr	r1, [r2, #-964]
	ldr	r2, [r4, #3972]
	rsb	r2, r2, r1
	cmp	r3, r2
	movcc	r2, r3
	ldr	r3, [sp, #20]
	cmp	r2, r3
	movcc	r3, r2
	str	r3, [sp, #52]
	mov	r3, #0
	str	r3, [sp, #28]
	b	.L2346
.L2362:
	ldrh	r3, [r8, #4]
	cmp	r3, #0
	beq	.L2347
	ldr	r6, [sp, #36]
	rsb	lr, r6, r5
	ldr	r6, [sp, #24]
	rsbs	r3, lr, #0
	adc	r3, r3, lr
	tst	r3, r6
	beq	.L2348
	ldr	r6, [sp, #28]
	cmp	r6, #0
	beq	.L2348
	movw	lr, #3924
	ldr	r6, [sp, #32]
	ldrh	r2, [r4, lr]
	add	r1, r6, r7
	mls	r1, r2, r5, r1
	cmp	r1, r2
	bne	.L2347
.L2348:
	add	r1, sp, #56
	mov	r2, #0
	mov	r0, r5
	str	r3, [sp, #4]
	bl	log2phys
	mov	r0, r8
	bl	get_new_active_ppa
	ldr	lr, [r4, #3972]
	ldr	r1, [r4, #3976]
	movw	r2, #3930
	mov	ip, #36
	ldrh	r2, [r4, r2]
	ldr	r6, .L2379
	mla	r1, ip, lr, r1
	mul	lr, lr, r2
	ldr	r6, [r6, #-916]
	bic	lr, lr, #3
	str	r6, [sp, #48]
	str	lr, [sp, #44]
	add	r6, r6, lr
	str	r5, [r1, #16]
	str	r6, [r1, #12]
	str	r0, [r1, #4]
	movw	r0, #3928
	ldrh	lr, [r4, r0]
	ldr	r0, [r4, #3972]
	mul	lr, r0, lr
	ldr	r0, .L2379
	bic	lr, lr, #3
	str	lr, [sp, #12]
	ldr	lr, [r0, #-936]
	ldr	r0, [sp, #12]
	add	lr, lr, r0
	mov	r0, r6
	str	lr, [r1, #8]
	mov	r1, #0
	str	ip, [sp, #8]
	bl	ftl_memset
	ldr	r1, [sp, #16]
	ldr	r3, [sp, #4]
	rsb	ip, r1, r5
	rsbs	r1, ip, #0
	adc	r1, r1, ip
	str	r1, [sp, #12]
	orrs	r1, r1, r3
	ldr	ip, [sp, #8]
	beq	.L2349
	ldr	r2, [sp, #12]
	cmp	r2, #0
	beq	.L2350
	movw	r3, #3924
	mov	r0, r7
	ldrh	fp, [r4, r3]
	mov	r1, fp
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #32]
	rsb	fp, r1, fp
	str	r1, [sp, #40]
	cmp	fp, r3
	movcs	fp, r3
	b	.L2351
.L2350:
	cmp	r3, #0
	beq	.L2351
	ldr	r3, [sp, #32]
	movw	lr, #3924
	add	fp, r3, r7
	ldrh	r3, [r4, lr]
	mls	fp, r3, r5, fp
	ldr	r3, [sp, #12]
	str	r3, [sp, #40]
	uxth	fp, fp
.L2351:
	movw	lr, #3924
	ldrh	r3, [r4, lr]
	cmp	fp, r3
	bne	.L2352
	ldr	r3, [sp, #12]
	ldr	r0, [sl, #3972]
	cmp	r3, #0
	ldr	r2, [sl, #3976]
	ldr	r3, [sp, #24]
	muleq	r1, r5, fp
	movne	r1, r9
	rsbeq	r1, r7, r1
	addeq	r1, r9, r1, asl #9
	cmp	r3, #0
	mov	r3, #36
	mla	r3, r3, r0, r2
	strne	r1, [r3, #8]
	bne	.L2355
	ldr	r0, [r3, #8]
	movw	r3, #3928
	ldrh	r2, [sl, r3]
	b	.L2377
.L2352:
	ldr	r2, [sp, #56]
	mov	r3, #36
	cmn	r2, #1
	beq	.L2356
	ldr	r1, [r4, #3972]
	add	r0, sp, #60
	str	r2, [sp, #64]
	ldr	r2, [r4, #3976]
	str	r5, [sp, #76]
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
	ldr	r3, .L2379
	ldreq	r2, [r3, #-1628]
	addeq	r2, r2, #1
	streq	r2, [r3, #-1628]
	beq	.L2358
	ldr	r2, [r6, #8]
	cmp	r2, r5
	beq	.L2358
	ldr	r2, [r3, #-1628]
	ldr	r0, .L2379+28
	add	r2, r2, #1
	str	r2, [r3, #-1628]
	mov	r2, r5
	ldr	r1, [r6, #8]
	bl	printk
	b	.L2358
.L2356:
	ldr	r1, [r4, #3972]
	ldr	r2, [r4, #3976]
	mla	r3, r3, r1, r2
	mov	r1, #0
	ldr	r0, [r3, #8]
	movw	r3, #3928
	ldrh	r2, [r4, r3]
	bl	ftl_memset
.L2358:
	ldr	r3, [sp, #12]
	ldr	r1, [r4, #3972]
	cmp	r3, #0
	ldr	r2, [r4, #3976]
	mov	r3, #36
	movweq	lr, #3924
	mla	r3, r3, r1, r2
	ldreqh	r1, [r4, lr]
	movne	r1, r9
	mov	r2, fp, asl #9
	muleq	r1, r1, r5
	ldrne	r0, [r3, #8]
	ldrne	r3, [sp, #40]
	rsbeq	r1, r7, r1
	ldreq	r0, [r3, #8]
	addne	r0, r0, r3, asl #9
	addeq	r1, r9, r1, asl #9
	b	.L2377
.L2349:
	ldr	r3, [sp, #24]
	ldr	r2, [r4, #3972]
	cmp	r3, #0
	ldr	r3, [r4, #3976]
	mla	ip, ip, r2, r3
	beq	.L2360
	movw	lr, #3924
	ldrh	r3, [r4, lr]
	mul	r3, r3, r5
	rsb	r3, r7, r3
	add	r3, r9, r3, asl #9
	str	r3, [ip, #8]
	b	.L2355
.L2360:
	movw	r0, #3924
	movw	r3, #3928
	ldrh	r1, [r4, r0]
	ldrh	r2, [r4, r3]
	ldr	r0, [ip, #8]
	mul	r1, r1, r5
	rsb	r1, r7, r1
	add	r1, r9, r1, asl #9
.L2377:
	bl	memcpy
.L2355:
	ldr	r3, .L2379+32
	ldr	r0, [sp, #48]
	ldr	lr, [sp, #44]
	strh	r3, [r0, lr]	@ movhi
	ldr	r3, .L2379
	str	r5, [r6, #8]
	add	r5, r5, #1
	ldr	r2, [r3, #-1796]
	str	r2, [r6, #4]
	add	r2, r2, #1
	cmn	r2, #1
	str	r2, [r3, #-1796]
	moveq	r2, #0
	streq	r2, [r3, #-1796]
	ldr	r3, [sp, #56]
	str	r3, [r6, #12]
	ldrh	r3, [r8, #0]
	strh	r3, [r6, #2]	@ movhi
	ldr	r3, [r4, #3972]
	add	r3, r3, #1
	str	r3, [r4, #3972]
	ldr	r3, [sp, #28]
	add	r3, r3, #1
	str	r3, [sp, #28]
.L2346:
	ldr	r6, [sp, #28]
	ldr	r3, [sp, #52]
	cmp	r6, r3
	bne	.L2362
.L2347:
	ldr	r3, [sp, #20]
	mov	r6, r8
	ldr	lr, [sp, #28]
	ldr	r2, [r4, #3972]
	rsb	r3, lr, r3
	str	r3, [sp, #20]
	ldr	r3, .L2379
	ldr	r0, [sp, #24]
	ldr	r3, [r3, #-964]
	cmp	r2, r3
	orrcs	r0, r0, #1
	uxtb	r3, r0
	cmp	r3, #0
	bne	.L2363
	ldrh	r2, [r8, #4]
	cmp	r2, #0
	bne	.L2373
.L2363:
	bl	FtlCacheWriteBack
	ldr	lr, [sp, #20]
	mov	r3, #0
	str	r3, [sl, #3972]
	cmp	lr, #1
	ldr	r3, [sp, #24]
	movls	r3, #0
.L2373:
	str	r3, [sp, #24]
.L2376:
	ldr	r3, [sp, #20]
	ldr	r8, .L2379
	cmp	r3, #0
	bne	.L2365
	mov	r0, r3
	ldr	r6, [sp, #36]
	ldr	r3, [sp, #16]
	rsb	r1, r3, r6
	bl	ftl_do_gc
	ldr	r3, .L2379+36
	ldrh	r3, [r8, r3]
	cmp	r3, #31
	bhi	.L2338
	ldr	r2, .L2379+40
	mov	r3, #128
	ldr	r5, .L2379+4
	mov	r4, #16
	strh	r3, [r8, r2]	@ movhi
	sub	r2, r2, #2
	strh	r3, [r8, r2]	@ movhi
.L2366:
	mov	r0, #0
	mov	r1, #1
	bl	ftl_do_gc
	ldr	r3, [r5, #4044]
	cmp	r3, #0
	bne	.L2338
	subs	r4, r4, #1
	bne	.L2366
.L2338:
	mov	r0, #0
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2380:
	.align	2
.L2379:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2-1992
	.word	.LANCHOR1
	.word	-1940
	.word	.LANCHOR2-1944
	.word	-1988
	.word	.LC158
	.word	-3947
	.word	-1996
	.word	-1186
	.fnend
	.size	ftl_write.part.25, .-ftl_write.part.25
	.align	2
	.global	ftl_write
	.type	ftl_write, %function
ftl_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	ip, r0
	mov	r0, r1
	mov	r1, r2
	mov	r2, r3
	ldr	r3, .L2386
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, [r3, #4044]
	cmp	r4, #0
	movne	r0, #0
	ldmnefd	sp!, {r4, pc}
	cmp	ip, #16
	bne	.L2383
	add	r0, r0, #256
	ldmfd	sp!, {r4, lr}
	b	FtlVendorPartWrite
.L2383:
	ldr	r3, [r3, #3960]
	add	ip, r1, r0
	cmp	ip, r3
	bhi	.L2385
	ldmfd	sp!, {r4, lr}
	b	ftl_write.part.25
.L2385:
	mvn	r0, #0
	ldmfd	sp!, {r4, pc}
.L2387:
	.align	2
.L2386:
	.word	.LANCHOR0
	.fnend
	.size	ftl_write, .-ftl_write
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
	.global	ftl_vendor_write
	.type	ftl_vendor_write, %function
ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	ip, r1
	mov	r3, r2
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	mov	r1, r4
	mov	r0, #16
	mov	r2, ip
	ldmfd	sp!, {r4, lr}
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
	mov	r6, r0
	mov	r5, r1
	mov	r4, r2
	bl	rknand_device_lock
	mov	r2, r4
	mov	r1, r5
	mov	r0, r6
	bl	ftl_vendor_write
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	FlashBootVendorWrite, .-FlashBootVendorWrite
	.align	2
	.global	ftl_read
	.type	ftl_read, %function
ftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	cmp	r0, #16
	.pad #84
	sub	sp, sp, #84
	mov	r6, r1
	mov	r8, r3
	str	r2, [sp, #48]
	bne	.L2393
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartRead
	str	r0, [sp, #36]
	b	.L2394
.L2393:
	ldr	r3, .L2424
	ldr	r1, [sp, #48]
	ldr	r2, [r3, #3960]
	add	r1, r1, r6
	str	r1, [sp, #40]
	cmp	r1, r2
	mvnhi	r3, #0
	strhi	r3, [sp, #36]
	bhi	.L2394
	movw	r2, #3924
	mov	r0, r6
	ldrh	r4, [r3, r2]
	mov	r1, r4
	bl	__aeabi_uidiv
	ldr	r2, [sp, #40]
	mov	r1, r4
	str	r0, [sp, #28]
	sub	r0, r2, #1
	bl	__aeabi_uidiv
	ldr	r1, [sp, #28]
	rsb	r3, r1, #1
	ldr	r1, [sp, #48]
	add	r3, r3, r0
	str	r3, [sp, #24]
	ldr	r3, .L2424+4
	str	r0, [sp, #32]
	ldr	r0, [sp, #28]
	ldr	r2, [r3, #-1804]
	add	r2, r1, r2
	ldr	r1, [sp, #24]
	str	r2, [r3, #-1804]
	ldr	r2, [r3, #-1832]
	add	r2, r1, r2
	ldr	r1, [sp, #32]
	str	r2, [r3, #-1832]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L2395
	bl	FtlCacheWriteBack
.L2395:
	mov	r9, #0
	ldr	r5, [sp, #28]
	str	r9, [sp, #52]
	mov	r4, r9
	str	r9, [sp, #36]
	mov	r7, r6
	ldr	sl, .L2424
	b	.L2419
.L2413:
	mov	r2, #0
	mov	r0, r5
	add	r1, sp, #76
	bl	log2phys
	ldr	r2, [sp, #76]
	cmn	r2, #1
	bne	.L2420
	b	.L2423
.L2400:
	mla	r0, r0, r5, r6
	cmp	r0, r7
	bcc	.L2399
	ldr	r2, [sp, #40]
	cmp	r0, r2
	bcs	.L2399
	rsb	r0, r7, r0
	mov	r1, #0
	mov	r2, #512
	add	r0, r8, r0, asl #9
	bl	ftl_memset
.L2399:
	add	r6, r6, #1
	b	.L2397
.L2423:
	mov	r6, #0
	movw	fp, #3924
.L2397:
	ldrh	r0, [sl, fp]
	cmp	r6, r0
	bcc	.L2400
	b	.L2401
.L2420:
	ldr	r3, .L2424+4
	mov	r6, #36
	ldr	r1, [r3, #-960]
	mla	r6, r6, r4, r1
	ldr	r1, [sp, #28]
	cmp	r5, r1
	str	r2, [r6, #4]
	movw	r2, #3924
	bne	.L2402
	ldrh	fp, [sl, r2]
	mov	r0, r7
	ldr	r3, [r3, #-932]
	mov	r1, fp
	str	r3, [r6, #8]
	bl	__aeabi_uidivmod
	ldr	r2, [sp, #48]
	rsb	r3, r1, fp
	str	r1, [sp, #56]
	cmp	r3, r2
	movcc	r2, r3
	str	r2, [sp, #52]
	cmp	r2, fp
	streq	r8, [r6, #8]
	b	.L2403
.L2402:
	ldr	r1, [sp, #32]
	cmp	r5, r1
	ldrneh	r3, [sl, r2]
	mulne	r3, r3, r5
	bne	.L2421
	ldr	r3, [r3, #-928]
	ldrh	r2, [sl, r2]
	ldr	r1, [sp, #40]
	str	r3, [r6, #8]
	mul	r3, r2, r5
	rsb	r9, r3, r1
	cmp	r9, r2
	bne	.L2403
.L2421:
	rsb	r3, r7, r3
	add	r3, r8, r3, asl #9
	str	r3, [r6, #8]
.L2403:
	movw	r3, #3930
	ldr	r2, .L2424+4
	ldrh	r3, [sl, r3]
	str	r5, [r6, #16]
	ldr	r2, [r2, #-920]
	mul	r3, r3, r4
	add	r4, r4, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [r6, #12]
.L2401:
	ldr	r2, [sp, #24]
	add	r5, r5, #1
	subs	r2, r2, #1
	str	r2, [sp, #24]
	beq	.L2405
	movw	r3, #3852
	ldrh	r3, [sl, r3]
	cmp	r4, r3, asl #3
	bne	.L2419
.L2405:
	cmp	r4, #0
	beq	.L2419
	ldr	r6, .L2424+4
	mov	r1, r4
	mov	r2, #0
	mov	fp, r5
	mov	r5, r4
	ldr	r0, [r6, #-960]
	bl	FlashReadPages
	ldr	r3, [sp, #56]
	ldr	r1, [sp, #52]
	mov	r2, r9, asl #9
	str	r2, [sp, #68]
	mov	r3, r3, asl #9
	str	r3, [sp, #60]
	mov	r1, r1, asl #9
	mov	r3, #0
	str	r1, [sp, #64]
	str	r3, [sp, #44]
.L2412:
	ldr	r1, [sp, #44]
	mov	r4, #36
	ldr	r3, [r6, #-960]
	mul	r4, r4, r1
	ldr	r1, [sp, #28]
	add	r3, r3, r4
	ldr	r2, [r3, #16]
	cmp	r2, r1
	bne	.L2407
	ldr	r1, [r3, #8]
	ldr	r3, [r6, #-932]
	cmp	r1, r3
	bne	.L2408
	ldr	r2, [sp, #60]
	mov	r0, r8
	add	r1, r1, r2
	ldr	r2, [sp, #64]
	b	.L2422
.L2407:
	ldr	r1, [sp, #32]
	cmp	r2, r1
	bne	.L2408
	ldr	r1, [r3, #8]
	ldr	r3, [r6, #-928]
	cmp	r1, r3
	bne	.L2408
	movw	r2, #3924
	ldr	r3, [sp, #32]
	ldrh	r0, [sl, r2]
	ldr	r2, [sp, #68]
	mul	r0, r0, r3
	rsb	r0, r7, r0
	add	r0, r8, r0, asl #9
.L2422:
	bl	memcpy
.L2408:
	ldr	r2, [r6, #-960]
	add	r3, r2, r4
	ldr	r2, [r2, r4]
	cmn	r2, #1
	streq	r2, [sp, #36]
	ldreq	r1, [r6, #-1628]
	addeq	r1, r1, #1
	streq	r1, [r6, #-1628]
	ldr	r2, [r3, #12]
	ldr	r1, [r3, #16]
	ldr	r2, [r2, #8]
	cmp	r1, r2
	beq	.L2410
	ldr	r2, [r6, #-1628]
	ldr	r0, .L2424+8
	add	r2, r2, #1
	str	r2, [r6, #-1628]
	ldr	ip, [r3, #12]
	ldr	r2, [r3, #8]
	ldr	r1, [ip, #4]
	str	r1, [sp, #0]
	ldr	r1, [ip, #8]
	str	r1, [sp, #4]
	ldr	r1, [ip, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2, #0]
	str	r1, [sp, #12]
	ldr	r2, [r2, #4]
	str	r2, [sp, #16]
	ldr	r1, [r3, #16]
	ldr	r2, [r3, #4]
	ldr	r3, [ip, #0]
	bl	printk
.L2410:
	ldr	r3, [r6, #-960]
	add	r2, r3, r4
	ldr	r3, [r3, r4]
	cmp	r3, #256
	bne	.L2411
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L2411:
	ldr	r1, [sp, #44]
	add	r1, r1, #1
	str	r1, [sp, #44]
	cmp	r1, r5
	bne	.L2412
	mov	r5, fp
	mov	r4, #0
.L2419:
	ldr	r2, [sp, #24]
	cmp	r2, #0
	bne	.L2413
	ldr	r3, .L2424+12
	ldr	r2, .L2424+4
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2394
	ldr	r0, [sp, #24]
	mov	r1, #1
	bl	ftl_do_gc
.L2394:
	ldr	r0, [sp, #36]
	add	sp, sp, #84
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2425:
	.align	2
.L2424:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC143
	.word	-1126
	.fnend
	.size	ftl_read, .-ftl_read
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
	mov	r2, #512
	mov	r5, r0
	mov	r4, r1
	mov	r0, r1
	mov	r1, #0
	bl	ftl_memset
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
	.global	ftl_vendor_read
	.type	ftl_vendor_read, %function
ftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	mov	ip, r1
	mov	r3, r2
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	mov	r1, r4
	mov	r0, #16
	mov	r2, ip
	ldmfd	sp!, {r4, lr}
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
	mov	r6, r0
	mov	r5, r1
	mov	r4, r2
	bl	rknand_device_lock
	mov	r2, r4
	mov	r1, r5
	mov	r0, r6
	bl	ftl_vendor_read
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	FlashBootVendorRead, .-FlashBootVendorRead
	.align	2
	.type	rk_ftl_vendor_ops.constprop.29, %function
rk_ftl_vendor_ops.constprop.29:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r2, #0
	mov	r3, r0
	mov	r2, r3
	mov	r0, r1
	mov	r1, #128
	beq	.L2432
	b	FlashBootVendorWrite
.L2432:
	b	FlashBootVendorRead
	.fnend
	.size	rk_ftl_vendor_ops.constprop.29, .-rk_ftl_vendor_ops.constprop.29
	.align	2
	.global	FtlCheckVpc
	.type	FtlCheckVpc, %function
FtlCheckVpc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, sl, lr}
	mov	r4, #0
	ldr	r1, .L2444
	ldr	r0, .L2444+4
	bl	printk
	ldr	r0, .L2444+8
	mov	r1, #8192
	ldr	r6, .L2444+12
	bl	__memzero
	ldr	r5, .L2444+16
	b	.L2434
.L2436:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L2435
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r0, r5, r0, asl #1
	add	r0, r0, #472
	ldrh	r3, [r0, #0]
	add	r3, r3, #1
	strh	r3, [r0, #0]	@ movhi
.L2435:
	add	r4, r4, #1
.L2434:
	ldr	r3, [r6, #3980]
	ldr	r7, .L2444+12
	cmp	r4, r3
	bcc	.L2436
	ldr	r5, .L2444+16
	movw	r6, #3860
	ldr	r0, .L2444+20
	mov	r2, #2
	ldrh	r3, [r7, r6]
	mov	r4, #0
	ldr	r1, [r5, #-2012]
	bl	rknand_print_hex
	ldr	r0, .L2444+24
	add	r1, r5, #472
	mov	r2, #2
	ldrh	r3, [r7, r6]
	bl	rknand_print_hex
	b	.L2437
.L2439:
	ldr	r2, [r5, #-2012]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	add	r3, r5, r3
	add	r3, r3, #472
	ldrh	r3, [r3, #0]
	cmp	r2, r3
	beq	.L2438
	ldr	r0, .L2444+28
	mov	r1, r4
	bl	printk
.L2438:
	add	r4, r4, #1
	uxth	r4, r4
.L2437:
	ldrh	r3, [r7, r6]
	cmp	r3, r4
	ldr	r3, .L2444+16
	bhi	.L2439
	ldr	r2, [r3, #-2000]
	cmp	r2, #0
	beq	.L2433
	ldr	r4, [r3, #-2020]
	mov	r6, #0
	ldr	r1, .L2444+32
	mov	r5, r3
	rsb	r2, r4, r2
	ldr	r4, .L2444+36
	mov	r8, #6
	movw	sl, #65535
	mov	r2, r2, asr #1
	ldrh	r7, [r3, r1]
	mul	r4, r4, r2
	uxth	r4, r4
	b	.L2441
.L2443:
	ldr	r2, [r5, #-2012]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	beq	.L2442
	add	r3, r5, r3
	ldr	r0, .L2444+40
	add	r3, r3, #472
	mov	r1, r4
	ldrh	r3, [r3, #0]
	bl	printk
.L2442:
	mul	r4, r8, r4
	ldr	r3, [r5, #-2020]
	ldrh	r4, [r3, r4]
	cmp	r4, sl
	beq	.L2433
	add	r6, r6, #1
	uxth	r6, r6
.L2441:
	cmp	r6, r7
	bne	.L2443
.L2433:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L2445:
	.align	2
.L2444:
	.word	.LANCHOR3+77
	.word	.LC101
	.word	.LANCHOR2+472
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC159
	.word	.LC160
	.word	.LC161
	.word	-1996
	.word	-1431655765
	.word	.LC162
	.fnend
	.size	FtlCheckVpc, .-FtlCheckVpc
	.align	2
	.global	HynixGetReadRetryDefault
	.type	HynixGetReadRetryDefault, %function
HynixGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2541
	mvn	r2, #83
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	cmp	r4, #2
	strb	r0, [r3, #0]
	strb	r2, [r3, #4]
	mvn	r0, #82
	mvn	r1, #81
	mvn	r2, #80
	.pad #60
	sub	sp, sp, #60
	strb	r0, [r3, #5]
	strb	r1, [r3, #6]
	strb	r2, [r3, #7]
	bne	.L2447
	mvn	r2, #88
	strb	r2, [r3, #4]
	ldr	r3, .L2541+4
	mvn	r2, #8
	mov	r7, #7
	strb	r2, [r3, #3249]
	b	.L2511
.L2447:
	cmp	r4, #3
	bne	.L2449
	mvn	r2, #79
	strb	r2, [r3, #4]
	mvn	r2, #78
	strb	r2, [r3, #5]
	mvn	r2, #77
	strb	r2, [r3, #6]
	mvn	r2, #76
	strb	r2, [r3, #7]
	mvn	r2, #75
	strb	r2, [r3, #8]
	mvn	r2, #74
	strb	r2, [r3, #9]
	mvn	r2, #73
	strb	r2, [r3, #10]
	mvn	r2, #72
	b	.L2535
.L2449:
	cmp	r4, #4
	bne	.L2450
	mvn	ip, #51
	strb	r0, [r3, #9]
	strb	ip, [r3, #4]
	mvn	ip, #64
	strb	r1, [r3, #10]
	strb	ip, [r3, #5]
	mvn	ip, #85
	strb	ip, [r3, #6]
	mvn	ip, #84
	strb	ip, [r3, #7]
	mvn	ip, #50
	strb	ip, [r3, #8]
.L2535:
	mov	r7, #8
	strb	r2, [r3, #11]
	mov	r6, r7
	b	.L2448
.L2450:
	cmp	r4, #5
	bne	.L2451
	mov	r2, #56
	mov	r7, #8
	strb	r2, [r3, #4]
	mov	r2, #57
	strb	r2, [r3, #5]
	mov	r2, #58
	strb	r2, [r3, #6]
	mov	r2, #59
	strb	r2, [r3, #7]
	b	.L2511
.L2451:
	cmp	r4, #6
	bne	.L2452
	mov	r2, #14
	mov	r7, #12
	strb	r2, [r3, #4]
	mov	r2, #15
	strb	r2, [r3, #5]
	mov	r2, #16
	strb	r2, [r3, #6]
	mov	r2, #17
	strb	r2, [r3, #7]
	b	.L2511
.L2452:
	cmp	r4, #7
	bne	.L2453
	mvn	r2, #79
	mov	r7, #12
	strb	r2, [r3, #4]
	mov	r6, #10
	mvn	r2, #78
	strb	r2, [r3, #5]
	mvn	r2, #77
	strb	r2, [r3, #6]
	mvn	r2, #76
	strb	r2, [r3, #7]
	mvn	r2, #75
	strb	r2, [r3, #8]
	mvn	r2, #74
	strb	r2, [r3, #9]
	mvn	r2, #73
	strb	r2, [r3, #10]
	mvn	r2, #72
	strb	r2, [r3, #11]
	mvn	r2, #43
	strb	r2, [r3, #12]
	mvn	r2, #42
	strb	r2, [r3, #13]
	b	.L2448
.L2453:
	cmp	r4, #8
	mov	r7, #7
	bne	.L2511
	mov	r2, #6
	strb	r7, [r3, #5]
	strb	r2, [r3, #4]
	mov	r7, #50
	mov	r2, #9
	strb	r4, [r3, #6]
	strb	r2, [r3, #7]
	mov	r6, #5
	mov	r2, #10
	strb	r2, [r3, #8]
	b	.L2448
.L2511:
	mov	r6, #4
.L2448:
	sub	r3, r4, #1
	cmp	r3, #1
	bhi	.L2531
	b	.L2539
.L2460:
	add	r2, fp, r8
	add	r4, fp, #20
	mov	r5, #0
	mov	r6, #55
	ldrb	r2, [r2, #3764]	@ zero_extendqisi2
	add	r4, r4, r2, asl #6
	add	r2, fp, r2, asl #3
	ldrb	sl, [r2, #868]	@ zero_extendqisi2
	ldr	r1, [r2, #864]
	add	sl, sl, #8
	add	sl, r1, sl, asl #8
.L2456:
	add	r2, fp, r5
	str	r6, [sl, #8]
	mov	r0, #80
	ldrb	r2, [r2, #4]	@ zero_extendqisi2
	str	r2, [sl, #4]
	str	r3, [sp, #8]
	bl	NandcDelayns
	ldr	r2, [sl, #0]
	strb	r2, [r4, r5]
	add	r5, r5, #1
	ldr	r3, [sp, #8]
	uxtb	r2, r5
	cmp	r2, r7
	bcc	.L2456
	mov	r2, #0
	b	.L2457
.L2458:
	ldrb	r6, [r0, r1, asl #2]	@ zero_extendqisi2
	ldrb	r5, [r4, r2]	@ zero_extendqisi2
	add	r5, r6, r5
	strb	r5, [ip, r1, asl #3]
	add	r1, r1, #1
	cmp	r1, #7
	bne	.L2458
	add	r2, r2, #1
	cmp	r2, #4
	beq	.L2459
.L2457:
	mov	r1, #1
	add	ip, r4, r2
	add	r0, r2, r9
	b	.L2458
.L2459:
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
	b	.L2454
.L2539:
	ldr	r9, .L2541+8
	mov	r3, r7
	mov	r8, #0
	mov	r7, r6
.L2454:
	ldr	fp, .L2541
	ldrb	r2, [fp, #3762]	@ zero_extendqisi2
	cmp	r2, r8
	bhi	.L2460
	mov	r6, r7
	mov	r7, r3
	b	.L2461
.L2531:
	sub	r3, r4, #3
	cmp	r3, #5
	bhi	.L2461
	mul	r3, r7, r6
	mov	sl, #0
	mov	r5, sl
	str	r7, [sp, #48]
	mov	r2, r3, asr #1
	mov	r3, r3, asl #4
	str	r2, [sp, #24]
	str	r3, [sp, #52]
	b	.L2462
.L2510:
	add	r3, r1, sl
	ldrb	r7, [r3, #3764]	@ zero_extendqisi2
	add	r3, r1, r7, asl #3
	mov	r0, r7
	ldrb	fp, [r3, #868]	@ zero_extendqisi2
	ldr	r9, [r3, #864]
	mov	r3, #255
	add	fp, fp, #8
	add	r8, r9, fp, asl #8
	str	r3, [r8, #8]
	str	r1, [sp, #8]
	bl	NandcWaitFlashReady
	cmp	r4, #7
	ldr	r1, [sp, #8]
	addeq	r1, r1, #28
	moveq	r3, #160
	mlaeq	r3, r3, r7, r1
	streq	r3, [sp, #32]
	beq	.L2464
.L2463:
	cmp	r4, #8
	addne	r1, r1, #20
	addne	r1, r1, r7, asl #6
	strne	r1, [sp, #32]
	bne	.L2464
	mov	r3, #120
	mov	r2, #25
	str	r3, [r8, #8]
	add	r1, r1, #28
	str	r5, [r8, #4]
	mov	r3, #23
	str	r5, [r8, #4]
	str	r1, [sp, #32]
	str	r5, [r8, #4]
	str	r3, [r8, #8]
	mov	r3, #4
	str	r3, [r8, #8]
	str	r2, [r8, #8]
	mov	r2, #218
	str	r2, [r8, #8]
	mov	r2, #21
	str	r5, [r8, #8]
	str	r5, [r8, #4]
	str	r5, [r8, #4]
	str	r2, [r8, #4]
	str	r3, [r8, #4]
	str	r5, [r8, #4]
	b	.L2466
.L2464:
	cmp	r4, #4
	mov	r3, #54
	str	r3, [r8, #8]
	bne	.L2467
	mov	r3, #255
	str	r3, [r8, #4]
	mov	r3, #64
	str	r3, [r9, fp, asl #8]
	mov	r3, #204
	b	.L2536
.L2467:
	sub	r3, r4, #5
	cmp	r3, #1
	ldrls	r3, .L2541
	ldrlsb	r3, [r3, #4]	@ zero_extendqisi2
	strls	r3, [r8, #4]
	movls	r3, #82
	bls	.L2537
.L2469:
	cmp	r4, #7
	bne	.L2468
	mov	r3, #174
	str	r3, [r8, #4]
	mov	r3, #176
	str	r5, [r9, fp, asl #8]
.L2536:
	str	r3, [r8, #4]
	mov	r3, #77
.L2537:
	str	r3, [r9, fp, asl #8]
.L2468:
	cmp	r4, #6
	mov	r3, #22
	str	r3, [r8, #8]
	mov	r3, #23
	str	r3, [r8, #8]
	mov	r3, #4
	str	r3, [r8, #8]
	mov	r3, #25
	str	r3, [r8, #8]
	moveq	r3, #31
	str	r5, [r8, #8]
	str	r5, [r8, #4]
	str	r5, [r8, #4]
	streq	r3, [r8, #4]
	mov	r3, #2
	strne	r5, [r8, #4]
	str	r3, [r8, #4]
	str	r5, [r8, #4]
.L2466:
	mov	r3, #48
	mov	r0, r7
	str	r3, [r8, #8]
	bl	NandcWaitFlashReady
	sub	r3, r4, #5
	cmp	r3, #1
	sub	r2, r4, #8
	movhi	r3, #0
	movls	r3, #1
	str	r3, [sp, #36]
	rsbs	r3, r2, #0
	ldr	ip, [sp, #36]
	adc	r3, r3, r2
	str	r3, [sp, #20]
	orrs	ip, ip, r3
	movne	r1, #16
	bne	.L2472
	cmp	r4, #7
	moveq	r1, #32
	movne	r1, #2
.L2472:
	ldr	r3, .L2541+12
	mov	r2, #0
	ldr	r3, [r3, #160]
.L2473:
	ldr	r0, [r8, #0]
	strb	r0, [r3, r2]
	add	r2, r2, #1
	uxtb	r0, r2
	cmp	r0, r1
	bcc	.L2473
	ldr	r1, [sp, #20]
	cmp	r1, #0
	beq	.L2474
	mov	r2, #0
.L2476:
	ldrb	r1, [r3, #0]	@ zero_extendqisi2
	cmp	r1, #50
	beq	.L2475
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	cmp	r1, #5
	beq	.L2475
	add	r2, r2, #1
	add	r3, r3, #4
	uxtb	r2, r2
	cmp	r2, #8
	bne	.L2476
	b	.L2477
.L2475:
	cmp	r2, #6
	bls	.L2478
.L2477:
	ldr	r0, .L2541+16
	mov	r1, #0
	bl	printk
.L2479:
	b	.L2479
.L2474:
	cmp	r4, #7
	bne	.L2480
	ldr	r2, [sp, #20]
.L2482:
	ldrb	r1, [r3, #0]	@ zero_extendqisi2
	cmp	r1, #12
	beq	.L2481
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	cmp	r1, #10
	beq	.L2481
	add	r2, r2, #1
	add	r3, r3, #4
	uxtb	r2, r2
	cmp	r2, #8
	bne	.L2482
	b	.L2483
.L2481:
	cmp	r2, #6
	bls	.L2478
.L2483:
	ldr	r0, .L2541+16
	mov	r1, #0
	bl	printk
.L2484:
	b	.L2484
.L2480:
	cmp	r4, #6
	bne	.L2478
	mov	r1, #8
.L2485:
	ldrb	r2, [r3], #1	@ zero_extendqisi2
	cmp	r2, #12
	beq	.L2478
	ldrb	r2, [r3, #7]	@ zero_extendqisi2
	cmp	r2, #4
	beq	.L2478
	sub	r1, r1, #1
	uxtb	r1, r1
	cmp	r1, #0
	bne	.L2485
	b	.L2540
.L2487:
	b	.L2487
.L2478:
	ldr	r2, .L2541+12
	ldr	r3, [r2, #160]
	str	r3, [sp, #40]
	mov	r3, #0
.L2488:
	ldr	ip, [sp, #40]
	ldr	r1, [r8, #0]
	strb	r1, [ip, r3]
	add	r3, r3, #1
	ldr	ip, [sp, #52]
	cmp	r3, ip
	blt	.L2488
	ldr	r0, [sp, #24]
	ldr	ip, [r2, #160]
	mov	r2, #8
	ldr	lr, [sp, #24]
	mov	r0, r0, asl #1
	str	r0, [sp, #44]
	add	r1, ip, r0
	mov	lr, lr, asl #2
	str	lr, [sp, #12]
.L2490:
	mov	r3, #0
	mov	r0, r3
.L2489:
	ldrh	lr, [r1, r3]
	add	r0, r0, #1
	mvn	lr, lr
	strh	lr, [r1, r3]	@ movhi
	ldr	lr, [sp, #24]
	add	r3, r3, #2
	cmp	r0, lr
	blt	.L2489
	ldr	r0, [sp, #12]
	subs	r2, r2, #1
	add	r1, r1, r0
	bne	.L2490
	str	ip, [sp, #28]
	str	r2, [sp, #16]
	b	.L2491
.L2495:
	mov	r1, #1
	mov	ip, #16
	mov	r0, r1, asl r3
	mov	r1, #0
	str	ip, [sp, #12]
	mov	ip, r1
	str	r3, [sp, #4]
.L2493:
	ldr	r3, [sp, #28]
	ldrh	lr, [r3, r1]
	ldr	r3, [sp, #12]
	and	lr, lr, r0
	cmp	lr, r0
	ldr	lr, [sp, #44]
	addeq	ip, ip, #1
	subs	r3, r3, #1
	add	r1, r1, lr
	str	r3, [sp, #12]
	bne	.L2493
	cmp	ip, #8
	ldr	r3, [sp, #4]
	orrhi	r2, r2, r0
	add	r3, r3, #1
	uxthhi	r2, r2
	cmp	r3, #16
	bne	.L2495
	ldr	ip, [sp, #28]
	ldr	lr, [sp, #16]
	strh	r2, [ip], #2	@ movhi
	add	lr, lr, #1
	ldr	r2, [sp, #24]
	str	ip, [sp, #28]
	cmp	lr, r2
	str	lr, [sp, #16]
	bge	.L2496
.L2491:
	mov	r2, #0
	mov	r3, r2
	b	.L2495
.L2496:
	ldr	r3, .L2541+12
	ldr	r1, [r3, #160]
	mov	r3, #0
	mov	r2, r3
.L2498:
	ldr	r0, [r1, r3]
	add	r3, r3, #4
	cmp	r0, #0
	addeq	r2, r2, #1
	cmp	r3, #32
	bne	.L2498
	cmp	r2, #7
	ble	.L2499
	ldr	r0, .L2541+20
	mov	r2, #1
	mov	r3, #1024
	bl	rknand_print_hex
	ldr	r0, .L2541+16
	mov	r1, #0
	bl	printk
.L2500:
	b	.L2500
.L2499:
	cmp	r4, #6
	moveq	ip, #4
	beq	.L2501
	cmp	r4, #7
	moveq	ip, #10
	beq	.L2501
	ldr	r3, [sp, #20]
	cmp	r3, #0
	movne	ip, #5
	moveq	ip, #8
.L2501:
	cmp	r6, #0
	ldr	r0, [sp, #40]
	subne	r3, r6, #1
	ldr	r1, [sp, #32]
	moveq	lr, #1
	mov	r2, #0
	uxtbne	r3, r3
	streq	lr, [sp, #16]
	addne	r3, r3, #1
	strne	r3, [sp, #16]
	b	.L2504
.L2505:
	ldr	r2, [sp, #12]
	ldrb	lr, [r2], #1	@ zero_extendqisi2
	strb	lr, [r1, r3]
	add	r3, r3, #1
	str	r2, [sp, #12]
	uxtb	lr, r3
	cmp	lr, r6
	bcc	.L2505
	ldr	r3, [sp, #16]
	add	r1, r1, ip
	ldr	r2, [sp, #28]
	add	r0, r0, r3
	ldr	r3, [sp, #48]
	add	r2, r2, #1
	cmp	r2, r3
	bge	.L2506
.L2504:
	str	r0, [sp, #12]
	mov	r3, #0
	str	r2, [sp, #28]
	b	.L2505
.L2506:
	mov	r3, #255
	mov	r0, r7
	str	r3, [r8, #8]
	bl	NandcWaitFlashReady
	ldr	ip, [sp, #36]
	cmp	ip, #0
	beq	.L2507
	mov	r3, #54
	str	r3, [r8, #8]
	ldr	r3, .L2541
	mov	r0, sl
	mvn	r1, #0
	ldrb	r3, [r3, #4]	@ zero_extendqisi2
	str	r3, [r8, #4]
	mov	r3, #22
	str	r5, [r9, fp, asl #8]
	str	r3, [r8, #8]
	bl	FlashReadCmd
	b	.L2508
.L2507:
	ldr	r1, [sp, #20]
	cmp	r1, #0
	movne	r3, #190
	moveq	r3, #56
	str	r3, [r8, #8]
.L2508:
	mov	r0, r7
	add	sl, sl, #1
	bl	NandcWaitFlashReady
	uxtb	sl, sl
.L2462:
	ldr	r1, .L2541
	ldrb	r3, [r1, #3762]	@ zero_extendqisi2
	cmp	r3, sl
	bhi	.L2510
	ldr	r7, [sp, #48]
.L2461:
	ldr	r3, .L2541
	strb	r6, [r3, #1]
	strb	r7, [r3, #2]
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2540:
	ldr	r0, .L2541+16
	bl	printk
	b	.L2487
.L2542:
	.align	2
.L2541:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR1+3232
	.word	.LANCHOR2
	.word	.LC163
	.word	.LC164
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
	cmp	r2, #7
	bhi	.L2545
	ldmfd	sp!, {r3, lr}
	b	HynixGetReadRetryDefault
.L2545:
	cmp	r3, #49
	bne	.L2546
	ldr	r0, .L2547
	mov	r2, #64
	ldr	r1, .L2547+4
	strb	r3, [r0, #0]
	mov	r3, #4
	strb	r3, [r0, #1]
	mov	r3, #15
	strb	r3, [r0, #2]
	add	r0, r0, #4
	bl	memcpy
	ldmfd	sp!, {r3, pc}
.L2546:
	ldmfd	sp!, {r3, lr}
	b	FlashGetReadRetryDefault.part.28
.L2548:
	.align	2
.L2547:
	.word	.LANCHOR0
	.word	.LANCHOR1+3148
	.fnend
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
	.align	2
	.global	FlashInit
	.type	FlashInit, %function
FlashInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r0
	mov	r0, #32768
	ldr	r5, .L2603
	bl	ftl_malloc
	ldr	r6, .L2603+4
	mov	r4, #0
	str	r0, [r5, #160]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r5, #148]
	mov	r0, #4096
	bl	ftl_malloc
	str	r0, [r5, #240]
	mov	r0, #32768
	bl	ftl_malloc
	str	r0, [r5, #232]
	mov	r0, #4096
	bl	ftl_malloc
	mov	r3, #50
	str	r4, [r5, #168]
	strb	r3, [r5, #176]
	strb	r3, [r6, #853]
	mov	r3, #128
	str	r4, [r5, #180]
	str	r3, [r6, #856]
	mov	r3, #60
	strb	r4, [r6, #3780]
	strb	r3, [r5, #164]
	strb	r4, [r6, #852]
	str	r0, [r5, #236]
	mov	r0, r7
	bl	NandcInit
	add	r5, r6, #2976
	mov	r7, #44
.L2553:
	uxtb	r0, r4
	mov	r1, r5
	bl	FlashReadIDRaw
	cmp	r4, #0
	bne	.L2550
	ldrb	r3, [r6, #2976]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L2593
	ldrb	r3, [r6, #2977]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2594
.L2550:
	ldrb	r3, [r5, #0]	@ zero_extendqisi2
	add	r4, r4, #1
	cmp	r3, #181
	streqb	r7, [r5, #0]
	cmp	r4, #4
	add	r5, r5, #8
	bne	.L2553
	ldr	r4, .L2603+4
	ldrb	r3, [r4, #2976]	@ zero_extendqisi2
	cmp	r3, #173
	beq	.L2554
	ldr	r0, [r4, #3784]
	bl	NandcSetDdrMode
.L2554:
	ldr	r5, .L2603+4
	mov	r1, #0
	mov	r2, #852
	mov	r0, r5
	bl	ftl_memset
	ldr	r3, .L2603+8
	ldr	r1, [r4, #3792]
	ldr	r2, .L2603+4
	str	r3, [r4, #3624]
	mov	r3, #0
	strb	r3, [r4, #860]
	ldr	r3, .L2603+12
	cmp	r1, r3
	moveq	r3, #1
	streq	r3, [r5, #3776]
	ldrb	r3, [r2, #2977]	@ zero_extendqisi2
	cmp	r3, #161
	cmpne	r3, #241
	beq	.L2556
	cmp	r3, #218
	beq	.L2556
	cmp	r3, #209
	beq	.L2556
	cmp	r3, #211
	beq	.L2556
	cmp	r3, #220
	bne	.L2557
	ldrb	r0, [r2, #2979]	@ zero_extendqisi2
	cmp	r0, #149
	bne	.L2557
.L2556:
	mov	r0, #1
	strb	r0, [r2, #852]
	ldr	r0, .L2603
	mov	ip, #16
	strb	ip, [r2, #853]
	ldrb	r2, [r2, #2976]	@ zero_extendqisi2
	strb	ip, [r0, #164]
	ldr	ip, .L2603+16
	cmp	r2, #152
	strb	r2, [ip, #3261]
	strb	r3, [ip, #3262]
	bne	.L2558
	ldr	r2, .L2603+4
	ldrb	r2, [r2, #2980]	@ zero_extendqisi2
	sxtb	r2, r2
	cmp	r2, #0
	movge	r2, #24
	strgeb	r2, [r0, #164]
.L2558:
	ldr	r2, .L2603+12
	movw	r0, #2049
	cmp	r1, r2
	cmpne	r1, r0
	ldreq	r2, .L2603
	moveq	r1, #16
	streqb	r1, [r2, #164]
	cmp	r3, #218
	bne	.L2560
	ldr	r3, .L2603+16
	movw	r2, #3274
	mov	r1, #2048
	strh	r1, [r3, r2]	@ movhi
	mvn	r2, #37
	b	.L2600
.L2560:
	cmp	r3, #220
	bne	.L2562
	ldr	r3, .L2603+16
	movw	r2, #3274
	mov	r1, #4096
	strh	r1, [r3, r2]	@ movhi
	mvn	r2, #35
.L2600:
	strb	r2, [r3, #3262]
	b	.L2561
.L2562:
	cmp	r3, #211
	bne	.L2561
	ldr	r3, .L2603+16
	movw	r2, #3274
	mov	r1, #4096
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #2
	strb	r2, [r3, #3273]
.L2561:
	ldr	r1, .L2603+20
	mov	r2, #32
	ldr	r0, .L2603+24
	bl	memcpy
	ldr	r0, .L2603+8
	mov	r2, #32
	add	r1, r0, #144
	bl	memcpy
.L2557:
	ldr	r4, .L2603+4
	ldrb	r3, [r4, #852]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2563
	bl	FlashLoadPhyInfoInRam
	cmp	r0, #0
	bne	.L2564
	ldr	r3, [r4, #3624]
	ldrh	r0, [r3, #16]
	ubfx	r0, r0, #8, #3
	strb	r0, [r4, #3772]
	tst	r0, #1
	bne	.L2564
	mov	r3, #1
	strb	r3, [r4, #3780]
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
.L2564:
	ldr	r4, .L2603+4
	ldr	r3, [r4, #3624]
	ldrb	r3, [r3, #26]	@ zero_extendqisi2
	strb	r3, [r4, #3072]
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L2563
	ldr	r3, [r4, #3784]
	cmp	r3, #0
	beq	.L2566
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	b	.L2601
.L2566:
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
.L2601:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L2563
	ldr	r4, .L2603+4
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r0, .L2603+28
	ldr	r3, [r4, #3624]
	ldrh	r1, [r3, #14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	cmn	r0, #1
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, pc}
	bl	FlashDieInfoInit
	ldr	r3, [r4, #3624]
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	bl	FlashGetReadRetryDefault
	ldr	r1, .L2603+32
	ldr	r0, .L2603
	ldr	r3, [r4, #3624]
	ldrh	r1, [r0, r1]
	ldrb	r2, [r3, #9]	@ zero_extendqisi2
	add	r1, r1, #4080
	add	r1, r1, #15
	cmp	r2, r1, lsr #12
	blt	.L2569
	ldrh	r1, [r3, #14]
	add	r1, r1, #255
	cmp	r2, r1, lsr #8
	bge	.L2570
.L2569:
	ldrh	r2, [r3, #14]
	bic	r2, r2, #255
	strh	r2, [r3, #14]	@ movhi
.L2570:
	ldr	r3, .L2603+4
	ldrb	r3, [r3, #3772]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L2571
	bl	FlashSavePhyInfo
	ldr	r3, .L2603
	mov	r0, #0
	ldr	r1, [r3, #172]
	bl	FlashDdrParaScan
.L2571:
	bl	FlashSavePhyInfo
.L2563:
	ldr	r4, .L2603+4
	ldr	r5, .L2603
	ldr	r3, [r4, #3624]
	ldrb	r2, [r3, #26]	@ zero_extendqisi2
	ldrh	r0, [r3, #10]
	ldrb	r6, [r3, #18]	@ zero_extendqisi2
	strb	r2, [r4, #3072]
	ldrh	r2, [r3, #16]
	ubfx	r1, r2, #7, #1
	strb	r1, [r4, #860]
	ubfx	r1, r2, #3, #1
	strb	r1, [r5, #188]
	ubfx	r1, r2, #4, #1
	ubfx	r2, r2, #8, #3
	strb	r1, [r4, #3841]
	strb	r2, [r4, #3772]
	mov	r2, #0
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	str	r2, [r5, #184]
	bl	__aeabi_idiv
	mov	r1, r0
	mov	r0, r6
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	ldr	r3, [r4, #3624]
	ldrh	r2, [r3, #16]
	tst	r2, #64
	beq	.L2572
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	strb	r0, [r4, #3761]
	strb	r3, [r4, #3760]
	ldrb	r3, [r4, #2]	@ zero_extendqisi2
	strb	r3, [r5, #152]
	sub	r3, r0, #1
	uxtb	r3, r3
	cmp	r3, #7
	bhi	.L2573
	ldr	r3, .L2603+36
	sub	ip, r0, #8
	rsbs	r1, ip, #0
	adc	r1, r1, ip
	str	r3, [r5, #184]
	sub	r3, r0, #5
	uxtb	r3, r3
	cmp	r0, #8
	cmpne	r3, #1
	movls	r3, #1
	strls	r3, [r4, #3832]
	cmp	r0, #7
	ldreq	r1, .L2603+40
	beq	.L2576
	ldr	r3, .L2603+44
	cmp	r1, #0
	add	r2, r3, #8
	moveq	r1, r3
	movne	r1, r2
.L2576:
	mov	r3, #0
	mov	r2, r3
.L2578:
	ldrsb	ip, [r1, r2]
	add	r2, r2, #1
	cmp	ip, #0
	addeq	r3, r3, #1
	cmp	r2, #32
	bne	.L2578
	cmp	r3, #27
	bls	.L2572
	bl	FlashGetReadRetryDefault
	bl	FlashSavePhyInfo
	b	.L2572
.L2573:
	sub	r3, r0, #17
	uxtb	r3, r3
	cmp	r3, #2
	bhi	.L2579
	ldr	r3, .L2603+48
	cmp	r0, #19
	str	r3, [r5, #184]
	mov	r3, #7
	strb	r3, [r5, #152]
	moveq	r3, #15
	streqb	r3, [r5, #152]
	b	.L2572
.L2579:
	cmp	r0, #33
	cmpne	r0, #65
	beq	.L2580
	cmp	r0, #66
	bne	.L2581
.L2580:
	ldr	r3, .L2603
	mov	r1, #4
	ldr	r2, .L2603+52
	str	r2, [r3, #184]
	ldr	r2, .L2603+4
	strb	r1, [r2, #3760]
	mov	r2, #7
	strb	r2, [r3, #152]
	b	.L2572
.L2581:
	cmp	r0, #67
	cmpne	r0, #34
	beq	.L2582
	cmp	r0, #35
	beq	.L2582
	cmp	r0, #68
	bne	.L2583
.L2582:
	ldr	r3, .L2603
	cmp	r0, #35
	cmpne	r0, #68
	ldr	r2, .L2603+52
	sub	r0, r0, #67
	uxtb	r0, r0
	str	r2, [r3, #184]
	mov	r2, #7
	strb	r2, [r3, #152]
	moveq	r2, #17
	streqb	r2, [r3, #152]
	cmp	r0, #1
	ldr	r3, .L2603+4
	movls	r2, #4
	movhi	r2, #5
	strb	r2, [r3, #3760]
	b	.L2572
.L2583:
	cmp	r0, #49
	ldreq	r3, .L2603+56
	streq	r3, [r5, #184]
.L2572:
	ldr	r4, .L2603+4
	ldr	r3, .L2603+12
	ldr	r2, [r4, #3792]
	cmp	r2, r3
	bne	.L2586
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, [r4, #3624]
	movne	r2, #0
	strneb	r2, [r3, #18]
.L2586:
	ldrb	r2, [r4, #2976]	@ zero_extendqisi2
	ldr	r3, .L2603+4
	cmp	r2, #44
	bne	.L2587
	ldrb	r2, [r3, #3780]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2587
	ldr	r1, [r3, #3792]
	ldr	r2, .L2603+12
	cmp	r1, r2
	bne	.L2588
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2587
.L2588:
	ldr	r3, .L2603+4
	mov	r2, #0
	mov	r0, #1
	strb	r2, [r3, #3780]
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
.L2587:
	ldrb	r3, [r4, #3772]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L2589
	ldr	r2, .L2603+4
	ldrb	r2, [r2, #3780]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L2590
	tst	r3, #1
	bne	.L2589
.L2590:
	mov	r0, #0
	bl	flash_enter_slc_mode
	ldr	r3, .L2603
	mov	r0, #0
	ldr	r1, [r3, #172]
	bl	FlashDdrParaScan
	mov	r0, #0
	bl	flash_exit_slc_mode
.L2589:
	ldr	r4, .L2603+4
	mov	r8, #16
	movw	r6, #3058
	ldr	r3, [r4, #3624]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	add	r0, r4, #928
	bl	FlashReadIdbDataRaw
	ldr	r0, .L2603+60
	strb	r8, [r4, #853]
	bl	FlashTimingCfg
	ldr	r5, [r4, #3624]
	movw	r3, #3052
	ldrb	r2, [r5, #12]	@ zero_extendqisi2
	ldrh	r7, [r5, #14]
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	ldrb	r2, [r4, #3762]	@ zero_extendqisi2
	str	r3, [r4, #3048]
	ldr	r3, [r4, #2976]
	str	r3, [r4, #3044]
	movw	r3, #3054
	strh	r2, [r4, r3]	@ movhi
	mov	r3, #3056
	ldrb	r2, [r5, #13]	@ zero_extendqisi2
	strh	r7, [r4, r6]	@ movhi
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #4
	ldrh	r2, [r5, #10]
	strh	r2, [r4, r3]	@ movhi
	ldrb	r1, [r5, #12]	@ zero_extendqisi2
	ldrh	r0, [r5, #10]
	bl	__aeabi_idiv
	movw	r3, #3062
	mov	lr, #512
	movw	r2, #3066
	strh	r0, [r4, r3]	@ movhi
	movw	r0, #3064
	ldrb	ip, [r5, #9]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	ldrh	r1, [r5, #10]
	mul	r1, r1, r3
	movw	r3, #3068
	strh	lr, [r4, r3]	@ movhi
	add	r3, r3, #2
	ldrb	lr, [r4, #853]	@ zero_extendqisi2
	strh	lr, [r4, r3]	@ movhi
	uxth	r1, r1
	ldrb	lr, [r4, #852]	@ zero_extendqisi2
	strh	r1, [r4, r2]	@ movhi
	cmp	lr, #1
	bne	.L2592
	mov	r7, r7, lsr #1
	mov	ip, ip, asl #1
	mov	r1, r1, asl #1
	strb	r8, [r4, #853]
	strh	r1, [r4, r2]	@ movhi
	mov	r2, #8
	strh	r7, [r4, r6]	@ movhi
	strh	ip, [r4, r0]	@ movhi
	strh	r2, [r4, r3]	@ movhi
.L2592:
	ldrb	r0, [r5, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	bl	ftl_flash_suspend
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2593:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2594:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2604:
	.align	2
.L2603:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1+3116
	.word	1446522928
	.word	.LANCHOR1
	.word	.LANCHOR1+3020
	.word	.LANCHOR0+896
	.word	.LC165
	.word	-882
	.word	HynixReadRetrial
	.word	.LANCHOR0+28
	.word	.LANCHOR0+20
	.word	MicronReadRetrial
	.word	ToshibaReadRetrial
	.word	SamsungReadRetrial
	.word	150000
	.fnend
	.size	FlashInit, .-FlashInit
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
	ldr	r4, .L2609
	ldr	r6, .L2609+4
	mov	r5, #0
	mov	r1, r6
	str	r5, [r1, #484]!
	str	r0, [r6, #480]
	mov	r0, r4
	str	r5, [r0, #144]!
	bl	rknand_get_reg_addr
	ldr	r3, [r4, #144]
	cmp	r3, r5
	mvneq	r4, #0
	beq	.L2606
	bl	rk_nandc_irq_init
	mov	r1, r5
	mov	r2, r5
	mov	r3, #2048
	ldr	r0, [r6, #480]
	bl	FlashSramLoadStore
	bl	rknand_flash_cs_init
	ldr	r0, [r4, #144]
	bl	FlashInit
	subs	r4, r0, #0
	bne	.L2607
	bl	FtlInit
.L2607:
	ldr	r0, .L2609+8
	mov	r1, r4
	bl	printk
.L2606:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L2610:
	.align	2
.L2609:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	.LC166
	.fnend
	.size	rk_ftl_init, .-rk_ftl_init
	.align	2
	.global	FtlReInitForSDUpdata
	.type	FtlReInitForSDUpdata, %function
FtlReInitForSDUpdata:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2636
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #20
	sub	sp, sp, #20
	ldrb	r4, [r3, #3072]	@ zero_extendqisi2
	cmp	r4, #0
	movne	r0, #0
	bne	.L2612
	ldr	r5, .L2636+4
	ldr	r0, [r5, #144]
	bl	FlashInit
	cmp	r0, #0
	movne	r0, r4
	bne	.L2612
	bl	FlashLoadFactorBbt
	cmp	r0, #0
	beq	.L2613
	bl	FlashMakeFactorBbt
.L2613:
	ldr	r0, [r5, #148]
	bl	FlashReadIdbDataRaw
	cmp	r0, #0
	beq	.L2614
	mov	r1, #0
	mov	r2, #16
	mov	r0, sp
	bl	FlashReadFacBbtData
	ldr	r1, [sp, #0]
	mov	r3, #0
	mov	r0, #1
	mov	r2, r3
.L2616:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #16
	bne	.L2616
	cmp	r3, #6
	ldrls	r3, .L2636
	bls	.L2632
	mov	r2, #0
	mov	r0, #1
.L2617:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #24
	bne	.L2617
	cmp	r3, #17
	ldr	r3, .L2636
	movhi	r2, #36
.L2632:
	strb	r2, [r3, #853]
	movw	r2, #3070
	ldr	r3, .L2636
	ldrb	r1, [r3, #853]	@ zero_extendqisi2
	strh	r1, [r3, r2]	@ movhi
.L2614:
	ldr	r1, .L2636+8
	mov	r4, #1
	ldr	r0, .L2636+12
	bl	printk
	ldr	r0, .L2636+16
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldr	r3, .L2636
	ldr	r0, [r3, #3856]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	b	.L2621
.L2623:
	add	r4, r4, #1
.L2621:
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L2634
.L2622:
	bl	FtlSysBlkInit
	cmp	r0, #0
	beq	.L2624
.L2634:
	bl	FtlLowFormat
	cmp	r4, #3
	bls	.L2623
	b	.L2635
.L2624:
	ldr	r3, .L2636+20
	mov	r2, #1
	str	r2, [r3, #3220]
	b	.L2612
.L2635:
	mvn	r0, #0
.L2612:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, pc}
.L2637:
	.align	2
.L2636:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC78
	.word	.LC77
	.word	.LANCHOR0+3044
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
	ldr	r3, .L2640
	ldr	r2, [r3, #3792]
	ldr	r3, .L2640+4
	cmp	r2, r3
	bxne	lr
	b	flash_enter_slc_mode
.L2641:
	.align	2
.L2640:
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
	ldr	r3, .L2644
	ldr	r2, [r3, #3792]
	ldr	r3, .L2644+4
	cmp	r2, r3
	bxne	lr
	b	flash_exit_slc_mode
.L2645:
	.align	2
.L2644:
	.word	.LANCHOR0
	.word	1446522928
	.fnend
	.size	flash_boot_exit_slc_mode, .-flash_boot_exit_slc_mode
	.align	2
	.type	IdBlockReadData, %function
IdBlockReadData:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r4, r0
	ldr	r5, .L2650
	mov	fp, r1
	.pad #28
	sub	sp, sp, #28
	mov	r1, r4
	mov	sl, r2
	ldr	r0, .L2650+4
	ldr	r3, [r5, #3624]
	mov	r2, fp
	ldr	r6, [r5, #856]
	mov	r9, r5
	ldrb	r7, [r3, #9]	@ zero_extendqisi2
	bl	printk
	mov	r0, r4
	mul	r6, r6, r7
	uxth	r6, r6
	mov	r1, r6
	mov	r6, #0
	bl	__aeabi_uidivmod
	mov	r8, r1
	rsb	r1, r1, r4
	mul	r2, r7, r8
	str	r1, [sp, #16]
	ubfx	r2, r2, #2, #2
	b	.L2647
.L2649:
	add	r1, r6, r8
	ldrb	r0, [r5, #3072]	@ zero_extendqisi2
	ubfx	r1, r1, #2, #16
	rsb	ip, r2, #4
	cmp	r0, #0
	add	r3, r5, r1, asl #1
	uxth	ip, ip
	add	r3, r3, #3072
	add	r3, r3, #4
	ldrh	r3, [r3, #0]
	beq	.L2648
	ldr	r0, [r5, #3792]
	ldr	lr, .L2650+8
	cmp	r0, lr
	moveq	r3, r1
.L2648:
	ldr	r1, [sp, #16]
	add	r2, r2, r1
	mla	r2, r7, r3, r2
	ldr	r3, [r9, #3624]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	ldrb	r3, [r9, #3840]	@ zero_extendqisi2
	str	r3, [sp, #20]
	ldr	r3, .L2650+12
	ldrb	r0, [r3, #164]	@ zero_extendqisi2
	str	ip, [sp, #4]
	str	r1, [sp, #8]
	str	r2, [sp, #12]
	bl	FlashBchSel
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r2, [sp, #12]
	ldr	r1, [sp, #8]
	mov	r0, r2
	bl	__aeabi_uidiv
	mov	r2, sl
	mov	r1, r0
	mov	r0, #0
	mov	r3, r0
	bl	FlashReadPage
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r0, [sp, #20]
	bl	FlashBchSel
	ldr	ip, [sp, #4]
	mov	r2, #0
	add	r6, ip, r6
	add	sl, sl, ip, asl #9
	uxth	r6, r6
.L2647:
	cmp	r6, fp
	bcc	.L2649
	mov	r1, r4
	mov	r2, fp
	mov	r3, #0
	ldr	r0, .L2650+16
	bl	printk
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2651:
	.align	2
.L2650:
	.word	.LANCHOR0
	.word	.LC167
	.word	1446522928
	.word	.LANCHOR2
	.word	.LC168
	.fnend
	.size	IdBlockReadData, .-IdBlockReadData
	.section	.text.unlikely
	.align	2
	.type	IDBlockWriteData, %function
IDBlockWriteData:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2660
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r2
	ldr	r2, [r3, #3624]
	mov	r5, r0
	mov	fp, r1
	.pad #92
	sub	sp, sp, #92
	mov	r1, r5
	ldr	r0, .L2660+4
	ldrb	r6, [r2, #9]	@ zero_extendqisi2
	mov	r2, fp
	ldr	r7, [r3, #856]
	bl	printk
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	mov	r1, r6
	mov	r0, r5
	bl	__aeabi_uidiv
	mul	r7, r7, r6
	uxth	r7, r7
	mov	r1, r0
	mov	r0, #0
	mov	r2, r0
	bl	FlashEraseBlock
	cmp	fp, r7
	movls	r8, #1
	bls	.L2653
	mov	r0, #0
	add	r1, r7, r5
	mov	r2, r0
	mov	r8, #2
	bl	FlashEraseBlock
.L2653:
	ldr	r4, .L2660
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r3, [r4, #3624]
	mov	r9, r4
	ldrh	r0, [r3, #10]
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	mov	r0, r0, asl #2
	mul	r0, r8, r0
	bl	__aeabi_idiv
	mov	r1, r7
	mov	r8, #0
	str	r0, [sp, #16]
	mov	r0, r5
	bl	__aeabi_uidivmod
	str	r5, [sp, #20]
	mov	ip, r1
	rsb	r1, r1, r5
	mov	r7, ip
	str	r1, [sp, #12]
	b	.L2654
.L2658:
	add	r3, r8, r7
	ubfx	r3, r3, #2, #16
	cmp	r3, #0
	beq	.L2655
	add	r1, r3, #1
	ldrb	r0, [r4, #3072]	@ zero_extendqisi2
	add	r2, r4, r1, asl #1
	cmp	r0, #0
	add	r2, r2, #3072
	add	r2, r2, #4
	ldrh	r2, [r2, #0]
	beq	.L2656
	ldr	r0, [r4, #3792]
	ldr	ip, .L2660+8
	cmp	r0, ip
	moveq	r2, r1
.L2656:
	sub	r2, r2, #-1073741823
	mov	r2, r2, asl #2
	str	r2, [sp, #24]
.L2655:
	movw	r2, #61424
	str	r2, [sp, #28]
	add	r2, r4, r3, asl #1
	add	r2, r2, #3072
	ldrh	r5, [r2, #4]
	ldrb	r2, [r4, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2657
	ldr	r2, [r4, #3792]
	ldr	r1, .L2660+8
	cmp	r2, r1
	moveq	r5, r3
.L2657:
	ldr	ip, [sp, #12]
	add	r8, r8, #4
	ldr	r2, .L2660+12
	uxth	r8, r8
	mla	r3, r6, r5, ip
	ldrb	ip, [r9, #3840]	@ zero_extendqisi2
	ldrb	r0, [r2, #164]	@ zero_extendqisi2
	str	ip, [sp, #4]
	str	r3, [sp, #8]
	bl	FlashBchSel
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r2, [r9, #3624]
	ldr	r3, [sp, #8]
	ldrb	r1, [r2, #9]	@ zero_extendqisi2
	mov	r0, r3
	bl	__aeabi_uidiv
	mov	r2, sl
	add	r3, sp, #24
	add	sl, sl, #2048
	mov	r1, r0
	mov	r0, #0
	bl	FlashProgPage
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	ip, [sp, #4]
	mov	r0, ip
	bl	FlashBchSel
	ldr	r0, [sp, #12]
	mov	r1, r6
	bl	__aeabi_uidiv
	add	r2, r5, #1
	uxth	r2, r2
	mov	r1, r0
	mov	r0, #0
	bl	FlashPageProgMsbFFData
.L2654:
	ldr	ip, [sp, #16]
	cmp	r8, ip
	bcc	.L2658
	ldr	r5, [sp, #20]
	mov	r2, fp
	mov	r3, #0
	ldr	r0, .L2660+16
	mov	r1, r5
	bl	printk
	mov	r0, #0
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2661:
	.align	2
.L2660:
	.word	.LANCHOR0
	.word	.LC169
	.word	1446522928
	.word	.LANCHOR2
	.word	.LC170
	.fnend
	.size	IDBlockWriteData, .-IDBlockWriteData
	.text
	.align	2
	.global	write_idblock
	.type	write_idblock, %function
write_idblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2687
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r7, r2
	ldr	r2, [r3, #3624]
	.pad #36
	sub	sp, sp, #36
	mov	r5, r0
	mov	r0, #256000
	mov	r6, r1
	ldr	sl, [r3, #856]
	ldrb	r4, [r2, #9]	@ zero_extendqisi2
	bl	ftl_malloc
	subs	r9, r0, #0
	beq	.L2681
	add	r5, r5, #508
	add	r5, r5, #3
	mov	r5, r5, lsr #9
	cmp	r5, #8
	movls	r5, #8
	bls	.L2664
	cmp	r5, #500
	bhi	.L2681
.L2664:
	ldr	r2, [r6, #0]
	ldr	r3, .L2687+4
	cmp	r2, r3
	bne	.L2681
	mul	sl, sl, r4
	uxth	sl, sl
	sub	r0, sl, #1
	add	r0, r0, r5
	mov	r1, sl
	bl	__aeabi_uidiv
	mov	r3, #0
	movw	r2, #63999
	movw	ip, #4097
	str	r0, [sp, #20]
	add	r0, r6, #254976
	add	r0, r0, #1020
.L2669:
	ldr	r1, [r0], #-4
	cmp	r1, #0
	bne	.L2665
	ldr	r1, [r6, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, ip
	sub	r2, r2, #1
	movcs	r3, #0
	cmp	r2, #4096
	str	r1, [r0, #4]
	bne	.L2669
	b	.L2668
.L2665:
	ldr	r0, .L2687+8
	bl	printk
.L2668:
	ldr	r4, .L2687
	mov	r1, r7
	mov	r3, #5
	ldr	r0, .L2687+12
	mov	r2, #4
	mov	r8, r5
	bl	rknand_print_hex
	ldrb	r2, [r4, #853]	@ zero_extendqisi2
	ldr	r1, [r6, #512]
	ldr	r0, .L2687+16
	bl	printk
	ldr	r2, .L2687+20
	movw	r3, #3070
	mov	r1, r5
	ldrh	r3, [r4, r3]
	ldr	r2, [r2, #168]
	ldr	r0, .L2687+24
	str	r2, [sp, #0]
	mov	r2, r5
	bl	printk
	ldrb	r3, [r4, #853]	@ zero_extendqisi2
	ldr	r2, [r6, #512]
	mov	r4, r7
	mov	r7, r6
	mov	r1, r5, asl #7
	cmp	r2, r3
	str	r1, [sp, #24]
	mov	r2, #0
	strhi	r3, [r6, #512]
	str	r2, [sp, #16]
	str	r2, [sp, #12]
.L2677:
	ldr	r1, .L2687
	ldr	r3, [r4], #4
	ldrb	r2, [r1, #853]	@ zero_extendqisi2
	cmp	r3, r2
	bcs	.L2671
	ldr	r1, .L2687+20
	ldr	r2, [r1, #168]
	cmp	r3, r2
	bcc	.L2671
	ldr	r2, [sp, #20]
	ldr	r1, [sp, #12]
	cmp	r2, #1
	movls	r2, #0
	movhi	r2, #1
	str	r2, [sp, #28]
	ldr	r2, [sp, #20]
	cmp	r1, #0
	cmpne	r2, #1
	bls	.L2672
	ldr	r2, [r4, #-8]
	add	r2, r2, #1
	cmp	r3, r2
	beq	.L2671
.L2672:
	mov	r0, r9
	mov	r1, #512
	bl	__memzero
	ldr	r0, [r4, #-4]
	mov	r1, r8
	mov	r2, r7
	mov	r6, #0
	mul	r0, r0, sl
	bl	IDBlockWriteData
	ldr	r0, [r4, #-4]
	mov	r2, r9
	mov	r1, r8
	mul	r0, r0, sl
	bl	IdBlockReadData
	mov	r2, r9
	mov	r3, r7
.L2676:
	mov	fp, r2
	mov	r5, r3
	ldr	r0, [fp, #0]
	add	r2, r2, #4
	ldr	r1, [r5, #0]
	add	r3, r3, #4
	cmp	r0, r1
	beq	.L2673
	mov	r0, r9
	mov	r1, #512
	bl	__memzero
	ldr	r3, [fp, #0]
	ldr	r1, [sp, #12]
	ldr	r0, .L2687+28
	str	r3, [sp, #0]
	ldr	r3, [r5, #0]
	bic	r5, r6, #255
	mov	r5, r5, asl #2
	str	r3, [sp, #4]
	mov	r3, r6
	ldr	r2, [r4, #-4]
	bl	printk
	ldr	r0, .L2687+32
	add	r1, r7, r5
	mov	r2, #4
	mov	r3, #256
	bl	rknand_print_hex
	mov	r3, #256
	add	r1, r9, r5
	mov	r2, #4
	ldr	r0, .L2687+36
	bl	rknand_print_hex
	mov	r0, #0
	bl	flash_boot_enter_slc_mode
	ldr	r1, [r4, #-4]
	mov	r0, #0
	mov	r2, r0
	mul	r1, r1, sl
	bl	FlashEraseBlock
	ldr	r3, [sp, #28]
	cmp	r3, #0
	beq	.L2674
	ldr	r1, [r4, #-4]
	mov	r0, #0
	mov	r2, r0
	add	r1, r1, #1
	mul	r1, sl, r1
	bl	FlashEraseBlock
.L2674:
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r1, [r4, #-4]
	ldr	r0, .L2687+40
	bl	printk
	ldr	r1, [sp, #24]
	cmp	r6, r1
	bcc	.L2671
	b	.L2675
.L2673:
	ldr	r1, [sp, #24]
	add	r6, r6, #1
	cmp	r6, r1
	bne	.L2676
.L2675:
	ldr	r2, [sp, #16]
	add	r2, r2, #1
	str	r2, [sp, #16]
.L2671:
	ldr	r3, [sp, #12]
	add	r3, r3, #1
	str	r3, [sp, #12]
	cmp	r3, #5
	bne	.L2677
	mov	r0, r9
	bl	ftl_free
	ldr	r1, [sp, #16]
	cmp	r1, #0
	mvneq	r0, #0
	movne	r0, #0
	b	.L2663
.L2681:
	mvn	r0, #0
.L2663:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2688:
	.align	2
.L2687:
	.word	.LANCHOR0
	.word	-52655045
	.word	.LC171
	.word	.LC172
	.word	.LC173
	.word	.LANCHOR2
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
	mov	r3, r0
	mov	r0, #0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r2, r0
	ldr	ip, .L2692
	b	.L2690
.L2691:
	ldrb	r4, [r3, r2]	@ zero_extendqisi2
	add	r2, r2, #1
	eor	r4, r4, r0, lsr #24
	add	r4, ip, r4, asl #2
	ldr	r4, [r4, #3292]
	eor	r0, r4, r0, asl #8
.L2690:
	cmp	r2, r1
	bne	.L2691
	ldmfd	sp!, {r4, pc}
.L2693:
	.align	2
.L2692:
	.word	.LANCHOR1
	.fnend
	.size	CRC_32, .-CRC_32
	.align	2
	.global	write_loader_lba
	.type	write_loader_lba, %function
write_loader_lba:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #64
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r4, r0
	.pad #32
	sub	sp, sp, #32
	mov	r6, r1
	mov	r7, r2
	bne	.L2695
	ldr	r2, [r2, #0]
	ldr	r3, .L2711
	cmp	r2, r3
	bne	.L2695
	ldr	r5, .L2711+4
	mov	r3, #1
	mov	r0, #256000
	strb	r3, [r5, #488]
	bl	ftl_malloc
	mov	r1, #0
	mov	r2, #256000
	str	r0, [r5, #492]
	bl	ftl_memset
	str	r4, [r5, #496]
.L2695:
	ldr	r3, .L2711+4
	ldrb	r2, [r3, #488]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2694
	sub	r0, r4, #64
	ldr	r5, [r3, #492]
	cmp	r0, #500
	bcs	.L2697
	rsb	r2, r4, #564
	add	r0, r5, r0, asl #9
	cmp	r6, r2
	movcc	r2, r6
	mov	r1, r7
	mov	r2, r2, asl #9
	bl	memcpy
	b	.L2698
.L2697:
	cmp	r4, #564
	bcc	.L2698
	ldr	r0, [r3, #496]
	ldr	r3, .L2711+8
	sub	r0, r0, #64
	cmp	r0, #500
	movcs	r0, #500
	ldr	r3, [r3, #3624]
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	cmp	r3, #4
	beq	.L2708
	mov	r3, #3
	mov	r2, #2
	mov	r7, #4
	mov	lr, #5
	stmia	sp, {r2, r3, r7, lr}
	mov	r3, #6
	str	r3, [sp, #16]
	b	.L2700
.L2708:
	mov	r1, #0
	mov	r2, r1
.L2699:
	cmp	r0, #256
	strls	r2, [sp, r1]
	movhi	r3, r2, asl #1
	add	r2, r2, #1
	strhi	r3, [sp, r1]
	cmp	r2, #5
	add	r1, r1, #4
	bne	.L2699
.L2700:
	ldr	r3, .L2711+4
	ldr	r2, [r3, #500]
	movw	r3, #63999
.L2705:
	ldrb	r1, [r2, r3]	@ zero_extendqisi2
	cmp	r1, #0
	addne	r3, r3, #1
	movne	r0, r3, asl #2
	bne	.L2704
.L2703:
	sub	r3, r3, #1
	cmp	r3, #4096
	bne	.L2705
	mov	r0, r0, asl #9
.L2704:
	mov	r1, r5
	ldr	r5, .L2711+4
	mov	r2, sp
	mov	r7, #0
	bl	write_idblock
	strb	r7, [r5, #488]
	ldr	r0, [r5, #492]
	bl	ftl_free
	str	r7, [r5, #492]
	b	.L2706
.L2698:
	ldr	r7, .L2711+4
	ldr	r3, [r7, #496]
	cmp	r3, r4
	beq	.L2706
	mov	r8, #0
	cmp	r5, r8
	strb	r8, [r7, #488]
	beq	.L2707
	mov	r0, r5
	bl	ftl_free
.L2707:
	str	r8, [r7, #492]
.L2706:
	ldr	r3, .L2711+4
	add	r4, r6, r4
	str	r4, [r3, #496]
.L2694:
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2712:
	.align	2
.L2711:
	.word	-52655045
	.word	.LANCHOR4
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
	mov	r6, r3
	sub	r3, r1, #64
	mov	r4, r1
	cmp	r3, #1984
	mov	r5, r2
	mov	r7, r0
	movcs	r3, #0
	movcc	r3, #1
	cmp	r0, #0
	movne	r3, #0
	cmp	r3, #0
	beq	.L2714
	mov	r0, r1
	mov	r1, r2
	mov	r2, r6
	bl	write_loader_lba
.L2714:
	mov	r0, r7
	mov	r1, r4
	mov	r2, r5
	mov	r3, r6
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
	ldr	r3, .L2831
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	cmp	r1, r3
	.pad #528
	sub	sp, sp, #528
	mov	r5, r1
	mov	r4, r2
	beq	.L2724
	bhi	.L2733
	ldr	r3, .L2831+4
	cmp	r1, r3
	beq	.L2721
	bhi	.L2734
	sub	r3, r3, #125
	cmp	r1, r3
	beq	.L2718
	bhi	.L2735
	sub	r3, r3, #237
	cmp	r1, r3
	bne	.L2777
	b	.L2827
.L2735:
	ldr	r3, .L2831+8
	cmp	r1, r3
	beq	.L2719
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2777
	b	.L2828
.L2734:
	ldr	r3, .L2831+12
	cmp	r1, r3
	beq	.L2723
	bhi	.L2736
	ldr	r3, .L2831+16
	cmp	r1, r3
	bne	.L2777
	b	.L2829
.L2736:
	ldr	r3, .L2831+20
	cmp	r1, r3
	beq	.L2723
	add	r3, r3, #10
	cmp	r1, r3
	bne	.L2777
	b	.L2723
.L2733:
	ldr	r3, .L2831+24
	cmp	r1, r3
	beq	.L2729
	bhi	.L2737
	sub	r3, r3, #78
	cmp	r1, r3
	beq	.L2726
	bcc	.L2725
	add	r3, r3, #21
	cmp	r1, r3
	beq	.L2727
	add	r3, r3, #56
	cmp	r1, r3
	bne	.L2777
	b	.L2830
.L2737:
	ldr	r3, .L2831+28
	cmp	r1, r3
	beq	.L2732
	bhi	.L2738
	sub	r3, r3, #956
	sub	r3, r3, #1
	cmp	r1, r3
	beq	.L2730
	add	r3, r3, #956
	cmp	r1, r3
	bne	.L2777
	b	.L2731
.L2738:
	ldr	r3, .L2831+32
	cmp	r1, r3
	beq	.L2731
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2777
	b	.L2732
.L2725:
	ldr	r0, .L2831+36
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2784
	mov	r1, r4
	mov	r2, #512
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2826
.L2739:
	ldr	r0, .L2831+40
	ldmia	r5, {r1, r2}
	bl	printk
	ldr	r3, [r5, #4]
	cmp	r3, #8
	str	r3, [sp, #4]
	bhi	.L2748
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
	beq	.L2812
	ldr	r0, .L2831+44
.L2825:
	bl	printk
	b	.L2748
.L2726:
	ldr	r0, .L2831+48
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2784
	mov	r1, r4
	mov	r2, #4096
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2826
.L2743:
	ldr	r4, .L2831+52
	ldr	r0, .L2831+56
	ldmia	r5, {r1, r2}
	bl	printk
	ldr	r3, [r4, #500]
	cmp	r3, #0
	bne	.L2744
	mov	r0, #260096
	bl	ftl_malloc
	cmp	r0, #0
	str	r0, [r4, #500]
	beq	.L2748
.L2745:
	mov	r1, #260096
	bl	__memzero
.L2744:
	ldr	r2, [r5, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L2748
	ldr	r3, [r5, #0]
	cmp	r3, #251904
	bhi	.L2748
	ldr	r1, .L2831+52
	ldr	r0, [r1, #500]
	add	r1, r5, #8
	add	r0, r0, r3
	bl	memcpy
.L2812:
	mov	r0, r5
	bl	ftl_free
.L2813:
	mov	r4, #0
	b	.L2742
.L2830:
	ldr	r0, .L2831+60
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2784
	mov	r1, r4
	mov	r2, #28
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L2747
.L2826:
	ldr	r0, .L2831+64
	b	.L2825
.L2747:
	ldmia	r5, {r1, r2}
	ldr	r0, .L2831+68
	bl	printk
	ldr	r1, [r5, #0]
	cmp	r1, #256000
	bhi	.L2748
	ldr	r4, .L2831+52
	ldr	r0, [r4, #500]
	cmp	r0, #0
	beq	.L2748
.L2749:
	bl	CRC_32
	ldr	r3, [r5, #4]
	cmp	r3, r0
	beq	.L2750
	mov	r0, r5
	bl	ftl_free
	b	.L2790
.L2750:
	bl	rknand_device_lock
	ldr	r1, [r4, #500]
	add	r2, r5, #8
	ldr	r0, [r5, #0]
	bl	write_idblock
	mov	r6, #0
	bl	rknand_device_unlock
	ldr	r0, [r4, #500]
	bl	ftl_free
	str	r6, [r4, #500]
	mov	r0, r5
	mov	r4, r6
	bl	ftl_free
	b	.L2742
.L2748:
	mov	r0, r5
	b	.L2816
.L2727:
	ldr	r0, .L2831+72
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2784
	bl	ftl_read_flash_info
	mov	r0, r4
	mov	r1, r5
	mov	r2, #11
	b	.L2821
.L2724:
	ldr	r0, .L2831+76
	bl	printk
	bl	nand_blk_add_whole_disk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2784
	mov	r1, #0
	mov	r2, #64
	bl	ftl_memset
	mov	r0, r4
	mov	r1, r5
	mov	r2, #64
	b	.L2821
.L2729:
	ldr	r0, .L2831+80
	bl	printk
	ldr	r3, .L2831+52
	add	r1, sp, #528
	mov	r0, r4
	mov	r2, #4
	ldr	r3, [r3, #504]
	ldr	r3, [r3, #20]
	str	r3, [r1, #-524]!
	b	.L2817
.L2730:
	ldr	r0, .L2831+84
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2784
	bl	rknand_device_lock
	mov	r1, #264
	mov	r2, #2
	mov	r3, r5
	mov	r0, #16
	bl	ftl_read
	bl	rknand_device_unlock
	mov	r0, r4
	mov	r1, r5
	mov	r2, #1024
.L2821:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	mov	r0, r5
	beq	.L2753
.L2816:
	bl	ftl_free
	b	.L2807
.L2753:
	bl	ftl_free
	b	.L2742
.L2718:
	ldr	r0, .L2831+88
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r5, r0, #0
	bne	.L2818
.L2754:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+92
	cmp	r2, r3
	bne	.L2809
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2809
	ldr	r6, .L2831+52
	mov	r2, #512
	add	r0, sp, #8
	ldr	r1, [r6, #504]
	bl	memcpy
	ldr	r2, [r6, #508]
	ldr	r3, .L2831+96
	cmp	r2, r3
	beq	.L2755
	add	r0, sp, #72
	mov	r1, #128
	str	r5, [sp, #16]
	str	r5, [sp, #20]
	bl	__memzero
.L2755:
	add	r0, sp, #264
	mov	r1, #256
	mov	r3, #0
	str	r3, [sp, #24]
	bl	__memzero
	b	.L2824
.L2719:
	ldr	r0, .L2831+100
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2818
.L2756:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+92
	cmp	r2, r3
	bne	.L2809
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2809
	ldr	r4, .L2831+52
	ldr	r3, .L2831+96
	ldr	r2, [r4, #508]
	cmp	r2, r3
	bne	.L2790
	ldr	r3, [sp, #20]
	sub	r2, r3, #1
	cmp	r2, #127
	mvnhi	r4, #2
	bhi	.L2716
	ldr	r0, [r4, #504]
	add	r1, sp, #72
	str	r3, [r0, #12]
	add	r0, r0, #64
	ldr	r2, [sp, #20]
	bl	memcpy
	mov	r0, #1
	ldr	r1, [r4, #504]
	b	.L2814
.L2829:
	ldr	r0, .L2831+104
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2818
.L2757:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+108
	cmp	r2, r3
	bne	.L2809
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2809
	ldr	r5, .L2831+52
	ldr	r4, [r5, #512]
	cmp	r4, #0
	beq	.L2716
	ldr	r3, [r5, #516]
	ldr	r2, .L2831+112
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2758
	str	r2, [r3, #0]
	mov	r2, #504
	ldr	r3, [r5, #516]
	str	r2, [r3, #4]
	mov	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L2758:
	ldr	r1, [r5, #516]
	mov	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #504]
	ldr	r2, .L2831+92
	ldr	r5, .L2831+52
	ldr	r1, [r3, #0]
	cmp	r1, r2
	strne	r2, [r3, #0]
	ldr	r0, [r5, #504]
	mov	r1, #128
	ldrne	r3, .L2831+52
	movne	r2, #504
	ldrne	r3, [r3, #504]
	stmneib	r3, {r2, r4}
	mov	r4, #0
	str	r4, [r0, #12]
	add	r0, r0, #64
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #504]
	bl	StorageSysDataStore
	str	r4, [r5, #512]
	str	r4, [r5, #508]
	b	.L2742
.L2721:
	ldr	r0, .L2831+116
	bl	printk
	mov	r1, r4
	add	r0, sp, #8
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r4, r0, #0
	bne	.L2818
.L2760:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+120
	cmp	r2, r3
	bne	.L2809
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2809
	ldr	r5, .L2831+52
	ldr	r3, [r5, #512]
	cmp	r3, #1
	beq	.L2716
	ldr	r3, [r5, #516]
	mov	r0, #0
	ldr	r2, .L2831+112
	ldr	r1, [r3, #0]
	cmp	r1, r2
	strne	r2, [r3, #0]
	ldr	r1, [r5, #516]
	ldrne	r3, [r5, #516]
	movne	r2, #504
	strne	r4, [r3, #12]
	stmneib	r3, {r2, r4}
	mov	r3, #1
	str	r3, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #504]
	ldr	r2, .L2831+92
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2762
	str	r2, [r3, #0]
	mov	r1, #504
	ldr	r3, .L2831+52
	mov	r2, #0
	ldr	r3, [r3, #504]
	stmib	r3, {r1, r2}
.L2762:
	ldr	r5, .L2831+52
	mov	r1, #128
	mov	r4, #0
	ldr	r0, [r5, #504]
	str	r4, [r0, #12]
	add	r0, r0, #64
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #504]
	bl	StorageSysDataStore
	mov	r3, #1
	str	r3, [r5, #512]
	b	.L2742
.L2828:
	ldr	r0, .L2831+124
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2818
.L2763:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+128
	cmp	r2, r3
	bne	.L2809
	ldr	r2, [sp, #12]
	cmp	r2, #512
	addls	r0, sp, #16
	ldrls	r1, .L2831+132
	bls	.L2823
	b	.L2809
.L2723:
	ldr	r3, .L2831+20
	cmp	r5, r3
	ldreq	r0, .L2831+136
	beq	.L2810
	ldr	r3, .L2831+140
	cmp	r5, r3
	ldreq	r0, .L2831+144
	ldrne	r0, .L2831+148
.L2810:
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2818
.L2767:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+152
	cmp	r2, r3
	bne	.L2807
	ldr	r3, .L2831+140
	ldr	r6, .L2831+52
	cmp	r5, r3
	bne	.L2768
	ldr	r3, [r6, #504]
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
	b	.L2716
.L2768:
	ldr	r3, [r6, #1032]
	cmp	r3, #10
	bhi	.L2807
	ldr	r3, [r6, #504]
	ldr	r1, [sp, #12]
	ldr	r2, [r3, #24]
	cmp	r2, r1
	beq	.L2769
	cmp	r2, #0
	beq	.L2769
	ldr	r0, .L2831+156
	bl	printk
	ldr	r3, [r6, #1032]
	add	r3, r3, #1
	str	r3, [r6, #1032]
	b	.L2807
.L2769:
	ldr	r0, .L2831+52
	mov	r2, #0
	str	r2, [r0, #1032]
	ldr	r0, .L2831+20
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
	b	.L2742
.L2731:
	ldr	r0, .L2831+160
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2818
.L2772:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+164
	cmp	r2, r3
	bne	.L2809
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L2809
	ldr	r3, .L2831+168
	add	r0, sp, #16
	cmp	r5, r3
	ldr	r3, .L2831+52
	ldreq	r1, [r3, #1036]
	ldrne	r1, [r3, #1040]
	add	r1, r1, #8
.L2823:
	bl	memcpy
.L2824:
	add	r1, sp, #8
	mov	r0, r4
	mov	r2, #520
.L2817:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	bne	.L2807
	b	.L2742
.L2732:
	ldr	r0, .L2831+172
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L2775
.L2818:
	ldr	r0, .L2831+64
	bl	printk
	b	.L2807
.L2775:
	ldr	r2, [sp, #8]
	ldr	r3, .L2831+164
	cmp	r2, r3
	bne	.L2809
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L2809
	ldr	r3, .L2831+28
	add	r2, r2, #8
	ldr	r4, .L2831+52
	cmp	r5, r3
	bne	.L2776
	add	r1, sp, #8
	ldr	r0, [r4, #1036]
	bl	memcpy
	ldr	r1, [r4, #1036]
	mov	r0, #2
.L2814:
	bl	StorageSysDataStore
	mov	r4, r0
	b	.L2742
.L2776:
	add	r1, sp, #8
	ldr	r0, [r4, #1040]
	bl	memcpy
	mov	r0, #3
	ldr	r1, [r4, #1040]
	b	.L2814
.L2827:
	bl	rknand_dev_flush
	b	.L2813
.L2809:
	mvn	r4, #0
.L2742:
	mov	r1, r4
	ldr	r0, .L2831+176
	bl	printk
	b	.L2716
.L2777:
	mvn	r4, #21
	b	.L2716
.L2784:
	mvn	r4, #11
	b	.L2716
.L2790:
	mvn	r4, #1
	b	.L2716
.L2807:
	mvn	r4, #13
.L2716:
	mov	r0, r4
	add	sp, sp, #528
	ldmfd	sp!, {r4, r5, r6, pc}
.L2832:
	.align	2
.L2831:
	.word	1074033155
	.word	1074029694
	.word	1074029570
	.word	1074031656
	.word	1074029695
	.word	1074031666
	.word	1074033235
	.word	1074034193
	.word	1074034194
	.word	.LC179
	.word	.LC181
	.word	.LC182
	.word	.LC183
	.word	.LANCHOR4
	.word	.LC184
	.word	.LC185
	.word	.LC180
	.word	.LC186
	.word	.LC187
	.word	.LC188
	.word	.LC189
	.word	.LC190
	.word	.LC191
	.word	1263358532
	.word	-1067903959
	.word	.LC192
	.word	.LC193
	.word	1112753220
	.word	1146313043
	.word	.LC194
	.word	1112755781
	.word	.LC195
	.word	1094995539
	.word	.LANCHOR4+520
	.word	.LC196
	.word	1074031676
	.word	.LC197
	.word	.LC198
	.word	1280262987
	.word	.LC199
	.word	.LC200
	.word	1145980246
	.word	1074034192
	.word	.LC201
	.word	.LC202
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
	ldr	r4, .L2836
	mov	r6, #0
	mvn	r3, #0
	add	r0, r4, #520
	ldr	r5, [r4, #480]
	str	r3, [r4, #496]
	add	r1, r5, #1536
	add	r3, r5, #512
	strb	r6, [r4, #488]
	str	r3, [r4, #504]
	add	r3, r5, #1024
	str	r6, [r4, #492]
	str	r3, [r4, #1036]
	str	r6, [r4, #500]
	str	r5, [r4, #516]
	str	r1, [r4, #1040]
	bl	memcpy
	ldr	r7, [r5, #508]
	ldr	r3, [r5, #16]
	cmp	r7, r6
	str	r6, [r4, #508]
	str	r6, [r4, #1032]
	str	r3, [r4, #512]
	beq	.L2834
	mov	r0, r5
	mov	r1, #508
	bl	js_hash
	cmp	r7, r0
	beq	.L2834
	str	r6, [r5, #16]
	ldr	r0, .L2836+4
	str	r6, [r4, #512]
	bl	printk
.L2834:
	ldr	r3, [r4, #512]
	mov	r0, #2
	ldr	r4, .L2836
	cmp	r3, #0
	ldrne	r2, .L2836+8
	ldrne	r3, .L2836
	ldr	r1, [r4, #1036]
	strne	r2, [r3, #508]
	bl	StorageSysDataLoad
	ldr	r1, [r4, #1040]
	mov	r0, #3
	bl	StorageSysDataLoad
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	rknand_sys_storage_init
.L2837:
	.align	2
.L2836:
	.word	.LANCHOR4
	.word	.LC203
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
	bl	ftl_malloc
	ldr	r7, .L2847
	cmp	r0, #0
	str	r0, [r7, #1044]
	beq	.L2845
	ldr	sl, .L2847+4
	mov	r6, #0
	mov	r4, r6
	mov	r5, r6
	movw	r8, #65532
.L2843:
	ldr	r0, [r7, #1044]
	mov	r1, r5, asl #7
	mov	r2, #0
	bl	rk_ftl_vendor_ops.constprop.29
	cmp	r0, #0
	bne	.L2841
	ldr	r3, [r7, #1044]
	ldr	r0, .L2847+8
	ldr	r1, [r3, #0]
	ldr	r2, [r3, r8]
	ldr	r3, [r3, #4]
	bl	printk
	ldr	r0, [r7, #1044]
	ldr	r3, [r0, #0]
	cmp	r3, sl
	bne	.L2842
	ldr	r3, [r0, r8]
	ldr	r2, [r0, #4]
	cmp	r3, r2
	bne	.L2842
	cmp	r4, r3
	movcc	r6, r5
	movcc	r4, r3
.L2842:
	add	r5, r5, #1
	cmp	r5, #2
	bne	.L2843
	cmp	r4, #0
	beq	.L2844
	mov	r1, r6, asl #7
	mov	r2, #0
	bl	rk_ftl_vendor_ops.constprop.29
	cmp	r0, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	b	.L2841
.L2844:
	mov	r1, #65536
	bl	__memzero
	ldr	r3, .L2847
	ldr	r1, .L2847+4
	mov	r2, #1
	mov	r0, r4
	ldr	r3, [r3, #1044]
	stmia	r3, {r1, r2}
	movw	r1, #65532
	str	r2, [r3, r1]
	ldr	r2, .L2847+12
	strh	r4, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2841:
	ldr	r4, .L2847
	ldr	r0, [r4, #1044]
	bl	kfree
	mov	r3, #0
	mvn	r0, #0
	str	r3, [r4, #1044]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2845:
	mvn	r0, #11
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2848:
	.align	2
.L2847:
	.word	.LANCHOR4
	.word	1380668996
	.word	.LC204
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
	ldr	r1, .L2856
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	ip, [r1, #1044]
	cmp	ip, #0
	ldrneh	r4, [ip, #10]
	movne	r5, ip
	movne	r1, #0
	bne	.L2851
	b	.L2855
.L2853:
	ldrh	r6, [r5, #16]
	add	r5, r5, #8
	cmp	r6, r3
	bne	.L2852
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
.L2852:
	add	r1, r1, #1
.L2851:
	cmp	r1, r4
	bcc	.L2853
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2855:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2857:
	.align	2
.L2856:
	.word	.LANCHOR4
	.fnend
	.size	rk_ftl_vendor_read, .-rk_ftl_vendor_read
	.align	2
	.global	rk_ftl_vendor_write
	.type	rk_ftl_vendor_write, %function
rk_ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2875
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r0
	ldr	r4, [r3, #1044]
	.pad #20
	sub	sp, sp, #20
	mov	r9, r1
	mov	r5, r2
	cmp	r4, #0
	beq	.L2871
	add	r8, r2, #63
	ldrh	r2, [r4, #8]
	ldrh	r3, [r4, #10]
	bic	r8, r8, #63
	mov	r7, #0
	str	r2, [sp, #8]
	b	.L2860
.L2867:
	add	r2, r7, #2
	mov	r2, r2, asl #3
	add	r6, r4, r2
	ldrh	r2, [r4, r2]
	cmp	r2, sl
	bne	.L2861
	ldrh	r2, [r6, #4]
	add	r2, r2, #63
	bic	r2, r2, #63
	str	r2, [sp, #12]
	cmp	r5, r2
	bls	.L2862
	ldrh	r2, [r4, #14]
	cmp	r2, r8
	bcc	.L2871
	sub	r3, r3, #1
	mov	fp, r8
	ldrh	r6, [r6, #2]
	mov	r8, r3
	b	.L2863
.L2864:
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
.L2863:
	cmp	r7, r8
	add	r2, r7, #2
	bcc	.L2864
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
	b	.L2874
.L2862:
	ldrh	r0, [r6, #2]
	mov	r1, r9
	mov	r2, r5
	add	r0, r0, #1024
	add	r0, r4, r0
	bl	memcpy
	strh	r5, [r6, #4]	@ movhi
	b	.L2874
.L2861:
	add	r7, r7, #1
.L2860:
	cmp	r7, r3
	bcc	.L2867
	ldrh	r2, [r4, #14]
	cmp	r2, r8
	bcc	.L2871
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
.L2874:
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
	bl	rk_ftl_vendor_ops.constprop.29
	mov	r0, #0
	b	.L2859
.L2871:
	mvn	r0, #0
.L2859:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2876:
	.align	2
.L2875:
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
	beq	.L2878
	ldr	r3, .L2894
	cmp	r6, r3
	beq	.L2880
	add	r3, r3, #1
	cmp	r6, r3
	bne	.L2891
	b	.L2893
.L2880:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2891
	ldr	r2, [r4, #0]
	ldr	r3, .L2894+4
	cmp	r2, r3
	bne	.L2890
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_read
	cmn	r0, #1
	beq	.L2886
	uxth	r2, r0
	mov	r1, r4
	strh	r2, [r4, #6]	@ movhi
	mov	r0, r5
	add	r2, r2, #8
	bl	rk_copy_to_user
	cmp	r0, #0
	moveq	r5, r0
	mvnne	r5, #13
	b	.L2879
.L2893:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2891
	ldr	r2, [r4, #0]
	ldr	r3, .L2894+4
	cmp	r2, r3
	bne	.L2890
	ldrh	r2, [r4, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L2890
	mov	r0, r4
	mov	r1, r5
	add	r2, r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2891
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_write
.L2886:
	mov	r5, r0
	b	.L2879
.L2890:
	mvn	r5, #0
	b	.L2879
.L2891:
	mvn	r5, #13
.L2879:
	mov	r0, r4
	bl	kfree
.L2878:
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L2895:
	.align	2
.L2894:
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
	.type	__func__.14500, %object
	.size	__func__.14500, 11
__func__.14500:
	.ascii	"FtlMemInit\000"
.LC0:
	.byte	60
	.byte	40
	.byte	24
	.byte	16
	.type	__func__.15296, %object
	.size	__func__.15296, 16
__func__.15296:
	.ascii	"FtlScanAllBlock\000"
	.type	__func__.15277, %object
	.size	__func__.15277, 17
__func__.15277:
	.ascii	"FtlDumpBlockInfo\000"
	.type	__func__.15544, %object
	.size	__func__.15544, 21
__func__.15544:
	.ascii	"FtlVpcCheckAndModify\000"
	.type	__func__.14573, %object
	.size	__func__.14573, 8
__func__.14573:
	.ascii	"FtlInit\000"
	.type	__func__.15242, %object
	.size	__func__.15242, 12
__func__.15242:
	.ascii	"FtlCheckVpc\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC1:
	.ascii	"FlashEraseBlocks pageAddr error %x\012\000"
.LC2:
	.ascii	"No.%d FLASH ID:%x %x %x %x %x %x\012\000"
.LC3:
	.ascii	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\012"
	.ascii	"\000"
.LC4:
	.ascii	"FtlFreeSysBlkQueueOut free count = %d\012\000"
.LC5:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\012"
	.ascii	"\000"
.LC6:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d\012\000"
.LC7:
	.ascii	"FLASH INFO:\012\000"
.LC8:
	.ascii	"FLASH ID: %x\012\000"
.LC9:
	.ascii	"Device Capacity: %d MB\012\000"
.LC10:
	.ascii	"FMWAIT: %x %x %x %x\012\000"
.LC11:
	.ascii	"FTL INFO:\012\000"
.LC12:
	.ascii	"g_MaxLpn = 0x%x\012\000"
.LC13:
	.ascii	"g_VaildLpn = 0x%x\012\000"
.LC14:
	.ascii	"read_page_count = 0x%x\012\000"
.LC15:
	.ascii	"discard_page_count = 0x%x\012\000"
.LC16:
	.ascii	"write_page_count = 0x%x\012\000"
.LC17:
	.ascii	"cache_write_count = 0x%x\012\000"
.LC18:
	.ascii	"l2p_write_count = 0x%x\012\000"
.LC19:
	.ascii	"gc_page_count = 0x%x\012\000"
.LC20:
	.ascii	"totle_write = %d MB\012\000"
.LC21:
	.ascii	"totle_read = %d MB\012\000"
.LC22:
	.ascii	"GSV = 0x%x\012\000"
.LC23:
	.ascii	"GDV = 0x%x\012\000"
.LC24:
	.ascii	"bad blk num = %d %d\012\000"
.LC25:
	.ascii	"free_superblocks = 0x%x\012\000"
.LC26:
	.ascii	"mlc_EC = 0x%x\012\000"
.LC27:
	.ascii	"slc_EC = 0x%x\012\000"
.LC28:
	.ascii	"avg_EC = 0x%x\012\000"
.LC29:
	.ascii	"sys_EC = 0x%x\012\000"
.LC30:
	.ascii	"max_EC = 0x%x\012\000"
.LC31:
	.ascii	"min_EC = 0x%x\012\000"
.LC32:
	.ascii	"PLT = 0x%x\012\000"
.LC33:
	.ascii	"POT = 0x%x\012\000"
.LC34:
	.ascii	"MaxSector = 0x%x\012\000"
.LC35:
	.ascii	"init_sys_blks_pp = 0x%x\012\000"
.LC36:
	.ascii	"sys_blks_pp = 0x%x\012\000"
.LC37:
	.ascii	"free sysblock = 0x%x\012\000"
.LC38:
	.ascii	"data_blks_pp = 0x%x\012\000"
.LC39:
	.ascii	"data_op_blks_pp = 0x%x\012\000"
.LC40:
	.ascii	"max_data_blks = 0x%x\012\000"
.LC41:
	.ascii	"Sys.id = 0x%x\012\000"
.LC42:
	.ascii	"Bbt.id = 0x%x\012\000"
.LC43:
	.ascii	"ACT.page = 0x%x\012\000"
.LC44:
	.ascii	"ACT.plane = 0x%x\012\000"
.LC45:
	.ascii	"ACT.id = 0x%x\012\000"
.LC46:
	.ascii	"ACT.mode = 0x%x\012\000"
.LC47:
	.ascii	"ACT.a_pages = 0x%x\012\000"
.LC48:
	.ascii	"ACT VPC = 0x%x\012\000"
.LC49:
	.ascii	"BUF.page = 0x%x\012\000"
.LC50:
	.ascii	"BUF.plane = 0x%x\012\000"
.LC51:
	.ascii	"BUF.id = 0x%x\012\000"
.LC52:
	.ascii	"BUF.mode = 0x%x\012\000"
.LC53:
	.ascii	"BUF.a_pages = 0x%x\012\000"
.LC54:
	.ascii	"BUF VPC = 0x%x\012\000"
.LC55:
	.ascii	"TMP.page = 0x%x\012\000"
.LC56:
	.ascii	"TMP.plane = 0x%x\012\000"
.LC57:
	.ascii	"TMP.id = 0x%x\012\000"
.LC58:
	.ascii	"TMP.mode = 0x%x\012\000"
.LC59:
	.ascii	"TMP.a_pages = 0x%x\012\000"
.LC60:
	.ascii	"GC.page = 0x%x\012\000"
.LC61:
	.ascii	"GC.plane = 0x%x\012\000"
.LC62:
	.ascii	"GC.id = 0x%x\012\000"
.LC63:
	.ascii	"GC.mode = 0x%x\012\000"
.LC64:
	.ascii	"GC.a_pages = 0x%x\012\000"
.LC65:
	.ascii	"WR_CHK = 0x%x %x %x %x\012\000"
.LC66:
	.ascii	"Read Err = 0x%x\012\000"
.LC67:
	.ascii	"Prog Err = 0x%x\012\000"
.LC68:
	.ascii	"gc_free_blk_th= 0x%x\012\000"
.LC69:
	.ascii	"gc_merge_free_blk_th= 0x%x\012\000"
.LC70:
	.ascii	"gc_skip_write_count= 0x%x\012\000"
.LC71:
	.ascii	"gc_blk_index= 0x%x\012\000"
.LC72:
	.ascii	"free min EC= 0x%x\012\000"
.LC73:
	.ascii	"free max EC= 0x%x\012\000"
.LC74:
	.ascii	"GC__SB VPC = 0x%x\012\000"
.LC75:
	.ascii	"%d. [0x%x]=0x%x 0x%x  0x%x\012\000"
.LC76:
	.ascii	"free %d. [0x%x] 0x%x  0x%x\012\000"
.LC77:
	.ascii	"%s\012\000"
.LC78:
	.ascii	"FTL version: 5.0.53 20190107\000"
.LC79:
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC80:
	.ascii	"FtlGcRefreshBlock  0x%x\012\000"
.LC81:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC82:
	.ascii	"decrement_vpc_count %x = %d in free list\012\000"
.LC83:
	.ascii	"FtlGcMarkBadPhyBlk %d 0x%x\012\000"
.LC84:
	.ascii	"%s error allocating memory. return -1\012\000"
.LC85:
	.ascii	"%s %p:0x%x:\000"
.LC86:
	.ascii	"%x \000"
.LC87:
	.ascii	"\000"
.LC88:
	.ascii	"%d statReg->V6.mtrans_cnt=%d flReg.V6.page_num=%d\012"
	.ascii	"\000"
.LC89:
	.ascii	"nandc:\000"
.LC90:
	.ascii	"%d flReg.d32=%x %x\012\000"
.LC91:
	.ascii	"micron RR %d row=%x,count %d,status=%d\012\000"
.LC92:
	.ascii	"ECC:%d\012\000"
.LC93:
	.ascii	"sdr read ok %x ecc=%d\012\000"
.LC94:
	.ascii	"sync para %d\012\000"
.LC95:
	.ascii	"TOG mode Read error %x %x\012\000"
.LC96:
	.ascii	"read retry status %x %x %x\012\000"
.LC97:
	.ascii	"Read pageadd=%x  ecc=%x err=%x\012\000"
.LC98:
	.ascii	"data:\000"
.LC99:
	.ascii	"spare:\000"
.LC100:
	.ascii	"ReadRetry pageadd=%x ecc=%x err=%x\012\000"
.LC101:
	.ascii	"...%s enter...\012\000"
.LC102:
	.ascii	"blk = %x vpc=%x mode = %x\012\000"
.LC103:
	.ascii	"mlc id = %x,%x addr= %x,spare= %x %x %x %x data=%x "
	.ascii	"%x\012\000"
.LC104:
	.ascii	"slc id = %x,%x addr= %x,spare= %x %x %x %x data=%x "
	.ascii	"%x\012\000"
.LC105:
	.ascii	"superBlkID = %x vpc=%x\012\000"
.LC106:
	.ascii	"flashmode = %x pagenum = %x %x\012\000"
.LC107:
	.ascii	"id = %x,%x addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC108:
	.ascii	"prog error: = %x\012\000"
.LC109:
	.ascii	"prog read error: = %x\012\000"
.LC110:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC111:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC112:
	.ascii	"FtlVpcTblFlush error = %x error count = %d\012\000"
.LC113:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC114:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC115:
	.ascii	"FtlBbmTblFlush error = %x error count = %d\012\000"
.LC116:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
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
	.ascii	"page map lost: %x %x\012\000"
.LC123:
	.ascii	"FtlMapWritePage error = %x\012\000"
.LC124:
	.ascii	"FtlMapWritePage error = %x error count = %d\012\000"
.LC125:
	.ascii	"FtlVendorPartRead refresh = %x phyAddr = %x\012\000"
.LC126:
	.ascii	"slc mode\000"
.LC127:
	.ascii	"no ect\000"
.LC128:
	.ascii	"FLFB:%d %d\012\000"
.LC129:
	.ascii	"phyBlk = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC130:
	.ascii	"Mblk:\000"
.LC131:
	.ascii	"L2P:\000"
.LC132:
	.ascii	"L2PC:\000"
.LC133:
	.ascii	"id = %x,%x addr= %x,spare= %x %x %x %x data = %x\012"
	.ascii	"\000"
.LC134:
	.ascii	":\000"
.LC135:
	.ascii	"BBT:\000"
.LC136:
	.ascii	"region_id = %x phyAddr = %x\012\000"
.LC137:
	.ascii	"map_ppn:\000"
.LC138:
	.ascii	"load_l2p_region refresh = %x phyAddr = %x\012\000"
.LC139:
	.ascii	"FtlGcScanTempBlk Error ID %x %x!!!!!!! \012\000"
.LC140:
	.ascii	"FtlGcScanTempBlkError ID %x %x!!!!!!!\012\000"
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
	.ascii	"GC des block %x done\012\000"
.LC149:
	.ascii	"%d GC datablk  = %x vpc %x %x\012\000"
.LC150:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC151:
	.ascii	"g_gc_superblock_free %x %x %x %x %x\012\000"
.LC152:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC153:
	.ascii	"rk_ftl_de_init %x\012\000"
.LC154:
	.ascii	"fix power lost blk = %x vpc=%x\012\000"
.LC155:
	.ascii	"erase power lost blk = %x vpc=%x\012\000"
.LC156:
	.ascii	"...%s: no bad block mapping table, format device\012"
	.ascii	"\000"
.LC157:
	.ascii	"...%s FtlSysBlkInit error ,format device!\012\000"
.LC158:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
.LC159:
	.ascii	"vpc1\000"
.LC160:
	.ascii	"vpc2\000"
.LC161:
	.ascii	"FtlCheckVpc2 %x = %x  %x\012\000"
.LC162:
	.ascii	"free blk vpc error %x = %x  %x\012\000"
.LC163:
	.ascii	"otp error! %d\000"
.LC164:
	.ascii	"rr\000"
.LC165:
	.ascii	"FlashLoadPhyInfo fail %x!!\012\000"
.LC166:
	.ascii	"FtlInit %x\012\000"
.LC167:
	.ascii	"IdBlockReadData %x %x\012\000"
.LC168:
	.ascii	"IdBlockReadData %x %x ret= %x\012\000"
.LC169:
	.ascii	"IDBlockWriteData %x %x\012\000"
.LC170:
	.ascii	"IDBlockWriteData %x %x ret= %x\012\000"
.LC171:
	.ascii	"write_idblock fix data %x %x\012\000"
.LC172:
	.ascii	"idblk:\000"
.LC173:
	.ascii	"idb reverse %x %x\012\000"
.LC174:
	.ascii	"write_idblock totle_sec %x %x %x %x\012\000"
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
	.ascii	"READ_SECTOR_IO\012\000"
.LC180:
	.ascii	"rk_copy_from_user error\012\000"
.LC181:
	.ascii	"READ_SECTOR_IO %x %x\012\000"
.LC182:
	.ascii	"rk_copy_to_user error\012\000"
.LC183:
	.ascii	"WRITE_SECTOR_IO\012\000"
.LC184:
	.ascii	"WRITE_SECTOR_IO %x %x\012\000"
.LC185:
	.ascii	"END_WRITE_SECTOR_IO\012\000"
.LC186:
	.ascii	"END_WRITE_SECTOR_IO %x %x\012\000"
.LC187:
	.ascii	"GET_FLASH_INFO_IO\012\000"
.LC188:
	.ascii	"GET_BAD_BLOCK_IO\012\000"
.LC189:
	.ascii	"GET_LOCK_FLAG_IO\012\000"
.LC190:
	.ascii	"GET_PUBLIC_KEY_IO\012\000"
.LC191:
	.ascii	"RKNAND_GET_DRM_KEY\012\000"
.LC192:
	.ascii	"RKNAND_STORE_DRM_KEY\012\000"
.LC193:
	.ascii	"RKNAND_DIASBLE_SECURE_BOOT\012\000"
.LC194:
	.ascii	"RKNAND_ENASBLE_SECURE_BOOT\012\000"
.LC195:
	.ascii	"RKNAND_GET_SN_SECTOR\012\000"
.LC196:
	.ascii	"RKNAND_LOADER_UNLOCK\012\000"
.LC197:
	.ascii	"RKNAND_LOADER_STATUS\012\000"
.LC198:
	.ascii	"RKNAND_LOADER_LOCK\012\000"
.LC199:
	.ascii	"LockKey not match %d\012\000"
.LC200:
	.ascii	"RKNAND_GET_VENDOR_SECTOR\012\000"
.LC201:
	.ascii	"RKNAND_STORE_VENDOR_SECTOR\012\000"
.LC202:
	.ascii	"return ret = %lx\012\000"
.LC203:
	.ascii	"secureBootEn check error\012\000"
.LC204:
	.ascii	"\0013vendor storage %x,%x,%x\012\000"
	.data
	.align	2
	.set	.LANCHOR1,. + 0
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
	.set	.LANCHOR4,. + 16368
	.type	gReadRetryInfo, %object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.space	852
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
	.type	gFlashSlcMode, %object
	.size	gFlashSlcMode, 1
gFlashSlcMode:
	.space	1
	.space	3
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
	.type	gNandFlashEccBits, %object
	.size	gNandFlashEccBits, 1
gNandFlashEccBits:
	.space	1
	.type	gMultiPageProgEn, %object
	.size	gMultiPageProgEn, 1
gMultiPageProgEn:
	.space	1
	.space	2
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
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.space	2056
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
	.type	p_map_block_valid_page_count, %object
	.size	p_map_block_valid_page_count, 4
p_map_block_valid_page_count:
	.space	4
	.type	p_l2p_map_buf, %object
	.size	p_l2p_map_buf, 4
p_l2p_map_buf:
	.space	4
	.type	gL2pMapInfo, %object
	.size	gL2pMapInfo, 44
gL2pMapInfo:
	.space	44
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
	.type	p_map_region_ppn_check_table, %object
	.size	p_map_region_ppn_check_table, 4
p_map_region_ppn_check_table:
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
	.type	gFlashPageBuffer0, %object
	.size	gFlashPageBuffer0, 4
gFlashPageBuffer0:
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
	.type	gpReadRetrial, %object
	.size	gpReadRetrial, 4
gpReadRetrial:
	.space	4
	.type	gMultiPageReadEn, %object
	.size	gMultiPageReadEn, 1
gMultiPageReadEn:
	.space	1
	.space	3
	.type	req_sys, %object
	.size	req_sys, 36
req_sys:
	.space	36
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.space	4
	.type	gFlashProgCheckBuffer, %object
	.size	gFlashProgCheckBuffer, 4
gFlashProgCheckBuffer:
	.space	4
	.type	gFlashProgCheckSpareBuffer, %object
	.size	gFlashProgCheckSpareBuffer, 4
gFlashProgCheckSpareBuffer:
	.space	4
	.type	gFlashSpareBuffer, %object
	.size	gFlashSpareBuffer, 4
gFlashSpareBuffer:
	.space	4
	.type	FbbtBlk, %object
	.size	FbbtBlk, 16
FbbtBlk:
	.space	16
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 44
gVendorBlkInfo:
	.space	44
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.space	2
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
	.type	g_LowFormat, %object
	.size	g_LowFormat, 4
g_LowFormat:
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
	.type	check_valid_page_count_table, %object
	.size	check_valid_page_count_table, 8192
check_valid_page_count_table:
	.space	8192
	.type	gLoaderBootInfo, %object
	.size	gLoaderBootInfo, 4
gLoaderBootInfo:
	.space	4
	.type	RK29_NANDC1_REG_BASE, %object
	.size	RK29_NANDC1_REG_BASE, 4
RK29_NANDC1_REG_BASE:
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
