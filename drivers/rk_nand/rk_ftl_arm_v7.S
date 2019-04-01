/*
 * Copyright (c) 2016-2018, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-04-02
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
	ldr	r3, .L259
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	add	r3, r3, r0, asl #3
	.pad #20
	sub	sp, sp, #20
	mov	r4, r0
	ldrb	r7, [r3, #868]	@ zero_extendqisi2
	mov	r5, r1
	ldr	r6, [r3, #864]
	bl	FlashReset
	add	r7, r7, #8
	mov	r0, r4
	bl	NandcFlashCs
	mov	r8, #0
	add	sl, r6, r7, asl #8
	mov	r3, #144
	mov	r0, #200
	str	r3, [sl, #8]
	str	r8, [sl, #4]
	bl	NandcDelayns
	ldr	r3, [r6, r7, asl #8]
	uxtb	r3, r3
	strb	r3, [r5, #0]
	ldr	r2, [r6, r7, asl #8]
	cmp	r3, #44
	strb	r2, [r5, #1]
	ldr	r2, [r6, r7, asl #8]
	strb	r2, [r5, #2]
	ldr	r2, [r6, r7, asl #8]
	strb	r2, [r5, #3]
	ldr	r2, [r6, r7, asl #8]
	strb	r2, [r5, #4]
	ldr	r2, [r6, r7, asl #8]
	strb	r2, [r5, #5]
	bne	.L257
	mov	r3, #239
	mov	r0, #200
	str	r3, [sl, #8]
	mov	r3, #1
	str	r3, [sl, #4]
	bl	NandcDelayns
	mov	r3, #4
	str	r3, [r6, r7, asl #8]
	str	r8, [r6, r7, asl #8]
	str	r8, [r6, r7, asl #8]
	str	r8, [r6, r7, asl #8]
.L257:
	mov	r0, r4
	bl	NandcFlashDeCs
	ldrb	r2, [r5, #0]	@ zero_extendqisi2
	sub	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L256
	ldrb	r1, [r5, #2]	@ zero_extendqisi2
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	ldr	r0, .L259+4
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
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L260:
	.align	2
.L259:
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
	ldr	r4, .L265
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	ldr	r0, [r4, #3776]
	cmp	r0, #0
	beq	.L262
	mov	r0, #0
	bl	flash_enter_slc_mode
	b	.L263
.L262:
	bl	flash_exit_slc_mode
.L263:
	ldrb	r3, [r4, #3780]	@ zero_extendqisi2
	ldr	r5, .L265
	cmp	r3, #0
	beq	.L264
	ldrb	r3, [r5, #3772]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L264
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	mov	r3, #0
	strb	r3, [r5, #3780]
.L264:
	ldr	r3, [r4, #864]
	mov	r0, #0
	str	r0, [r3, #336]
	ldmfd	sp!, {r3, r4, r5, pc}
.L266:
	.align	2
.L265:
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
	ldr	r3, .L271
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
	bne	.L268
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
	b	.L270
.L268:
	uxtb	r2, r5
	str	r2, [r6, #4]
	mov	r2, r5, lsr #8
	str	r2, [r6, #4]
	mov	r2, r5, lsr #16
	str	r2, [r6, #4]
	ldrb	r3, [r3, #905]	@ zero_extendqisi2
	str	r3, [r6, #8]
.L270:
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
.L272:
	.align	2
.L271:
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
	ldr	r3, .L274
	add	r3, r3, r0, asl #3
	ldr	r3, [r3, #864]
	str	r1, [r3, #336]
	bx	lr
.L275:
	.align	2
.L274:
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
	ldr	r1, .L286
	bl	__aeabi_idiv
	ldr	r3, .L286+4
	ldr	r3, [r3, #3008]
	cmp	r0, #250
	movwgt	r2, #8354
	bgt	.L284
	cmp	r0, #220
	bgt	.L285
.L279:
	cmp	r0, #185
	movwgt	r2, #4226
	bgt	.L284
	cmp	r0, #160
	movwgt	r2, #4194
	bgt	.L284
	cmp	r4, #35
	movwls	r2, #4193
	bls	.L284
	cmp	r4, #99
	movwls	r2, #4225
	bls	.L284
.L285:
	movw	r2, #8322
.L284:
	str	r2, [r3, #4]
	ldmfd	sp!, {r4, pc}
.L287:
	.align	2
.L286:
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
	ldr	r4, .L290
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
	ldr	r2, .L290+4
	str	r2, [r3, #304]
	bl	ftl_malloc
	str	r5, [r4, #3824]
	str	r5, [r4, #3832]
	str	r0, [r4, #3796]
	str	r0, [r4, #3800]
	add	r0, r0, #32768
	str	r0, [r4, #3804]
	ldmfd	sp!, {r3, r4, r5, pc}
.L291:
	.align	2
.L290:
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
	bls	.L293
	movw	r3, #4226
	movw	r2, #8322
	cmp	r0, r3
	cmpne	r0, r2
	bne	.L294
.L293:
	ldr	r3, .L295
	ldr	r3, [r3, #3008]
	str	r0, [r3, #4]
.L294:
	ldr	r3, .L295+4
	ldrb	r0, [r3, #3137]	@ zero_extendqisi2
	b	NandcTimeCfg
.L296:
	.align	2
.L295:
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
	ldr	ip, .L298
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
.L299:
	.align	2
.L298:
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
	ldr	r3, .L305
	mov	r1, #1
	ldr	r2, [r3, #3008]
	str	r0, [r3, #3836]
	mov	r3, #0
	str	r1, [r2, #8]
	mov	r1, #16
	cmp	r0, r1
	bfi	r3, r1, #8, #8
	bfc	r3, #18, #1
	beq	.L304
.L301:
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L302
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	bne	.L302
.L304:
	bfc	r3, #4, #1
.L302:
	orr	r3, r3, #1
	str	r3, [r2, #12]
	bx	lr
.L306:
	.align	2
.L305:
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
	ldr	r3, .L308
	strb	r0, [r3, #3840]
	b	NandcBchSel
.L309:
	.align	2
.L308:
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
	ldr	r5, .L315
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
.L312:
	ldrb	r3, [r5, r4, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L311
	uxtb	r0, r4
	bl	FlashReset
.L311:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L312
	ldr	r4, .L315
	ldrb	r3, [r4, #3780]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L313
	mov	r0, #1
	bl	NandcSetMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
	ldrb	r0, [r4, #3029]	@ zero_extendqisi2
	bl	NandcSetDdrPara
.L313:
	ldr	r3, .L315
	ldr	r3, [r3, #3624]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	ldmfd	sp!, {r3, r4, r5, lr}
	b	FlashBchSel
.L316:
	.align	2
.L315:
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
	bne	.L320
	mov	r0, r4
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L320:
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
	ldr	r5, .L341
	mov	r6, r0
	str	r1, [sp, #12]
	mov	r7, r2
	ldrb	r4, [r5, #852]	@ zero_extendqisi2
	cmp	r4, #0
	moveq	r9, r5
	beq	.L322
	mov	r1, r2
	bl	FlashEraseSLc2KBlocks
	b	.L323
.L331:
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
	bcs	.L325
	ldrb	r3, [r5, #3841]	@ zero_extendqisi2
	cmp	r3, #0
	add	r3, r9, r0, asl #4
	ldr	r3, [r3, #3636]
	moveq	sl, #0
	cmp	r3, #0
	beq	.L327
	uxtb	r0, r0
	bl	FlashWaitCmdDone
.L327:
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
	bne	.L329
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L329
	bl	flash_enter_slc_mode
	b	.L330
.L329:
	bl	flash_exit_slc_mode
.L330:
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
.L325:
	add	r4, r4, #1
.L322:
	cmp	r4, r7
	bcc	.L331
	ldr	r5, .L341
	mov	r4, #0
	ldr	r8, .L341+4
	ldr	r0, [r5, #3008]
	bl	NandcIqrWaitFlashReady
	b	.L332
.L334:
	uxtb	r0, r4
	bl	FlashWaitCmdDone
	ldr	r3, [sp, #12]
	cmp	r3, #1
	bne	.L333
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L333
	ldrb	r0, [r8, r4, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L333:
	add	r4, r4, #1
.L332:
	ldrb	r2, [r5, #3762]	@ zero_extendqisi2
	ldr	r3, .L341
	cmp	r4, r2
	bcc	.L334
	ldr	r0, [r3, #3844]
	cmp	r0, #0
	beq	.L323
	ldrb	r3, [r3, #2976]	@ zero_extendqisi2
	cmp	r3, #69
	moveq	r3, #0
	moveq	r2, r3
	bne	.L340
	b	.L335
.L336:
	add	r3, r3, #1
	str	r2, [r6, #-36]
.L335:
	cmp	r3, r7
	add	r6, r6, #36
	bne	.L336
.L340:
	mov	r0, #0
.L323:
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L342:
	.align	2
.L341:
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
	ldr	r3, .L345
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
.L346:
	.align	2
.L345:
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
.L348:
	ldr	r3, [r0, #8]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #1048576
	beq	.L348
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
	ldr	r1, .L363
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
	bls	.L351
	ldr	r3, [r5, #16]
	cmp	r9, #0
	cmpeq	ip, #0
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bfc	r3, #2, #1
	str	r3, [sp, #12]
	beq	.L352
	cmp	r6, #0
	beq	.L353
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
	b	.L355
.L358:
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
.L355:
	ldr	sl, [sp, #4]
	cmp	r2, sl
	bcc	.L358
.L353:
	mov	r0, r5
	add	r8, r8, #1
	bl	rk_nandc_xfer_irq_flag_init
	ldr	r3, [r5, #368]
	mov	r8, r8, asr #1
	cmp	r9, #0
	bfi	r4, r8, #22, #6
	ldr	r8, .L363
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
.L352:
	ldr	r3, [sp, #12]
	str	r3, [r5, #16]
.L351:
	str	r7, [r5, #12]
	str	r4, [r5, #8]
	orr	r4, r4, #4
	str	r4, [r5, #8]
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L364:
	.align	2
.L363:
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
	bne	.L366
	cmp	r3, #0
	beq	.L367
	mov	r0, r2
	mov	r1, r3
	mov	r2, #1024
	bl	memcpy
.L367:
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
.L366:
	cmp	r3, #0
	beq	.L369
	mov	r1, r2
	mov	r0, r3
	mov	r2, #1024
	bl	memcpy
.L369:
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
	b	.L371
.L372:
	add	r3, r3, #1
	mov	r2, r2, asl #1
	uxth	r3, r3
.L371:
	cmp	r2, r0
	bls	.L372
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
	ldr	r3, .L377
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
.L378:
	.align	2
.L377:
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
	ldr	r2, .L396
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
	ldr	r1, .L396+4
.L380:
	strb	r2, [r2, r1]
	add	r2, r2, #1
	cmp	r2, #32
	bne	.L380
	ldrh	r1, [r4, #14]
	ldrh	r2, [r4, #20]
	cmp	r2, r1, lsr #8
	bcs	.L381
	ldr	r1, .L396+4
	sub	ip, r0, #1
	uxtb	r7, r3
	mov	r2, #0
	rsb	sl, r3, r1
	str	sl, [sp, #8]
	mla	ip, r3, ip, r1
	mov	r8, r7, asl #1
	uxtb	r8, r8
	str	ip, [sp, #12]
	b	.L382
.L383:
	strb	ip, [r9, r1]
	add	fp, ip, r7
	add	ip, ip, r8
	add	r5, r5, #1
	strb	fp, [sl, r1]
	uxtb	ip, ip
.L385:
	cmp	r5, r0
	add	r1, r1, r3
	bcc	.L383
	add	r2, r2, #1
.L382:
	cmp	r2, r3
	bcs	.L384
	ldr	fp, [sp, #8]
	mov	r1, #0
	uxtb	ip, r2
	mov	r5, r1
	add	r9, fp, r2
	ldr	fp, [sp, #12]
	add	sl, fp, r2
	b	.L385
.L384:
	ldr	r2, .L396
	movw	r1, #3874
	mov	r0, r0, asl #1
	mov	r6, r6, lsr #1
	strh	r0, [r2, r1]	@ movhi
	movw	r1, #3862
	strh	r6, [r2, r1]	@ movhi
.L381:
	ldr	r2, .L396
	movw	r1, #3910
	cmp	lr, #1
	mov	r0, #5
	mov	ip, #0
	ldr	sl, .L396
	ldrb	r8, [r2, #852]	@ zero_extendqisi2
	strh	r0, [r2, r1]	@ movhi
	movw	r0, #3912
	streqh	lr, [r2, r1]	@ movhi
	cmp	r8, #0
	movw	r1, #3914
	strh	ip, [r2, r0]	@ movhi
	mov	r0, #4352
	strh	r0, [r2, r1]	@ movhi
	ldrne	r2, .L396
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
	ldr	r4, .L396
	ldrls	r2, .L396
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
	ldreq	r3, .L396
	uxtah	r6, r6, r0
	str	r6, [r4, #3848]
	ldr	r4, .L396
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
	bne	.L393
.L392:
	cmp	r2, #7
	movls	r3, #8
	strls	r3, [r4, #3948]
.L393:
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
	ldrcc	r2, .L396
	movcc	r1, #1
	strcch	r1, [r2, r3]	@ movhi
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L397:
	.align	2
.L396:
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
	ldr	r3, .L405
	movw	r2, #3966
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L403
	movw	r2, #3940
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #3968]
	mov	r3, #0
	b	.L400
.L402:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L404
	add	r3, r3, #1
	uxth	r3, r3
.L400:
	cmp	r3, r1
	bne	.L402
	mov	r0, #0
	bx	lr
.L404:
	mov	r0, #1
	bx	lr
.L403:
	mov	r0, r2
	bx	lr
.L406:
	.align	2
.L405:
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
	ldr	r2, .L415
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #3972]
	cmp	r3, #0
	beq	.L412
	ldr	ip, [r2, #3976]
	mov	r2, #0
.L411:
	ldr	r4, [ip, #16]
	cmp	r4, r0
	bcc	.L409
	cmp	r4, r1
	bls	.L413
.L409:
	add	r2, r2, #1
	add	ip, ip, #36
	cmp	r2, r3
	bne	.L411
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L413:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L412:
	mov	r0, r3
	ldmfd	sp!, {r4, pc}
.L416:
	.align	2
.L415:
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
	ldr	r3, .L418
	ldr	r0, [r3, #3960]
	bx	lr
.L419:
	.align	2
.L418:
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
	ldr	r3, .L421
	ldr	r0, [r3, #3960]
	bx	lr
.L422:
	.align	2
.L421:
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
	ldr	r3, .L424
	ldr	r0, [r3, #3980]
	bx	lr
.L425:
	.align	2
.L424:
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
	ldr	r4, .L427
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
	ldr	r0, .L427+4
	bl	printk
	movw	r3, #3990
	ldrh	r2, [r4, r3]
	mov	r0, #0
	add	r2, r2, r6
	strh	r2, [r4, r3]	@ movhi
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L428:
	.align	2
.L427:
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
	ldr	r5, .L430
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
.L431:
	.align	2
.L430:
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
	ldr	r6, .L437
	movw	r8, #3916
	movw	r7, #3874
	b	.L434
.L436:
	mov	r0, r5
	add	r5, r5, #1
	bl	FtlBbmIsBadBlock
	uxth	r5, r5
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
.L434:
	ldrh	r3, [r6, r8]
	ldrh	r2, [r6, r7]
	mul	r3, r2, r3
	cmp	r5, r3
	blt	.L436
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L438:
	.align	2
.L437:
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
	ldr	r5, .L440
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
.L441:
	.align	2
.L440:
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
	ldr	r3, .L443
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
.L444:
	.align	2
.L443:
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
	ldr	r4, .L446
	ldrh	r1, [r4, r3]
	bl	__aeabi_uidivmod
	movw	r3, #3876
	uxth	r0, r1
	ldrh	r1, [r4, r3]
	bl	__aeabi_uidiv
	uxth	r0, r0
	ldmfd	sp!, {r4, pc}
.L447:
	.align	2
.L446:
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
	beq	.L449
	ldr	r4, .L450
	ldr	r3, [r4, #4044]
	cmp	r3, #0
	bne	.L449
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
.L449:
	ldr	r3, .L450
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
.L451:
	.align	2
.L450:
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
	bls	.L453
	rsb	r0, r1, r0
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L453:
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
	ldr	r2, .L456
	movw	r3, #4066
	ldrh	r0, [r2, r3]
	rsbs	r0, r0, #1
	movcc	r0, #0
	bx	lr
.L457:
	.align	2
.L456:
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
	ldr	r2, .L459
	movw	r3, #4066
	ldrh	r0, [r2, r3]
	sub	r2, r0, #1024
	rsbs	r0, r2, #0
	adc	r0, r0, r2
	bx	lr
.L460:
	.align	2
.L459:
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
	ldr	r2, .L463
	movw	r3, #4066
	ldrh	r3, [r2, r3]
	cmp	r3, #1024
	bxeq	lr
	b	FtlFreeSysBlkQueueIn.part.7
.L464:
	.align	2
.L463:
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
	ldr	r3, .L469
	ldr	r2, .L469+4
	ldr	r1, .L469+8
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
	b	.L467
.L468:
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
.L467:
	cmp	r0, r4
	ldr	ip, .L469+8
	blt	.L468
	movw	r1, #4062
	strh	r2, [ip, r1]	@ movhi
	mov	r2, #4064
	strh	r3, [ip, r2]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L470:
	.align	2
.L469:
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
	ldr	r4, .L478
	movw	r7, #4062
.L472:
	ldrh	r1, [r4, r6]
	cmp	r1, #0
	beq	.L473
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
	bne	.L474
	mov	r0, r5
	bl	P2V_block_in_plane
	mov	r3, r5, asl #10
	mov	r8, r0
	ldr	r0, [r4, #4048]
	str	r3, [r0, #4]
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L475
	mov	r1, sl
	mov	r2, #1
	bl	FlashEraseBlocks
.L475:
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
	b	.L474
.L473:
	ldr	r0, .L478+4
	bl	printk
.L476:
	b	.L476
.L474:
	sub	r2, r5, #1
	movw	r3, #65533
	uxth	r2, r2
	cmp	r2, r3
	bls	.L477
	mov	r1, r5
	ldrh	r2, [r4, r6]
	ldr	r0, .L478+8
	bl	printk
	b	.L472
.L477:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L479:
	.align	2
.L478:
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
	ldr	r3, .L485
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	movw	r4, #65535
	ldr	r2, [r0, #0]
	ldr	r0, [r3, #-2020]
	ldr	ip, .L485+4
	rsb	r3, r0, r2
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	mov	ip, #6
	uxth	r3, r3
.L482:
	cmp	r1, r3
	beq	.L483
	ldrh	r3, [r2, #0]
	cmp	r3, r4
	beq	.L484
	mla	r2, ip, r3, r0
	b	.L482
.L483:
	mov	r0, #1
	ldmfd	sp!, {r4, pc}
.L484:
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L486:
	.align	2
.L485:
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
	ldr	r2, .L502
	movw	r3, #3860
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r3, [r2, r3]
	cmp	r3, r0
	bls	.L488
	ldr	r1, .L502+4
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
	beq	.L501
.L489:
	ldr	r7, [r1, #-2012]
	mov	r5, r0, asl #1
	ldrh	r1, [r2, #4]
	ldr	fp, [r8, #-2020]
	ldrh	sl, [r7, r5]
	cmp	r1, #0
	ldr	r8, .L502+8
	mulne	r6, r1, sl
	rsb	r1, fp, r3
	movw	sl, #3860
	mov	r1, r1, asr #1
	mul	r1, r8, r1
	ldr	r8, .L502
	ldrh	sl, [r8, sl]
	ldr	r8, [r8, #4052]
	uxth	r1, r1
	add	r5, r8, r5
	str	r5, [sp, #12]
	mov	r5, #0
	str	sl, [sp, #8]
	str	r8, [sp, #4]
.L496:
	add	r5, r5, #1
	ldr	r8, [sp, #8]
	uxth	r5, r5
	cmp	r5, r8
	bhi	.L488
	cmp	r0, r1
	beq	.L488
	mov	r9, r1, asl #1
	ldrh	r8, [r3, #4]
	ldrh	sl, [r7, r9]
	cmp	r8, #0
	mvneq	r8, #0
	mulne	r8, r8, sl
	cmp	r8, r6
	bne	.L492
	ldr	r8, [sp, #4]
	ldrh	sl, [r8, r9]
	ldr	r9, [sp, #12]
	ldrh	r8, [r9, #0]
	cmp	sl, r8
	bcc	.L494
	b	.L493
.L492:
	bhi	.L493
.L494:
	ldrh	r8, [r3, #0]
	movw	sl, #65535
	cmp	r8, sl
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	ldreq	r3, .L502+4
	streq	r2, [r3, #-2008]
	beq	.L488
.L495:
	mov	r1, #6
	mla	r3, r1, r8, fp
	mov	r1, r8
	b	.L496
.L493:
	strh	r1, [r4, ip]	@ movhi
	ldrh	r1, [r3, #2]
	strh	r1, [r2, #2]	@ movhi
	ldr	r1, .L502+4
	ldr	ip, [r1, #-2016]
	cmp	r3, ip
	bne	.L497
	strh	r0, [r3, #2]	@ movhi
.L501:
	str	r2, [r1, #-2016]
	b	.L488
.L497:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-2020]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L488:
	mov	r0, #0
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L503:
	.align	2
.L502:
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
	ldr	r2, .L505
	ldr	r3, .L505+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L506:
	.align	2
.L505:
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
	beq	.L508
	ldr	r1, .L515
	mov	r5, #6
	mul	ip, r5, r0
	mvn	r3, #0
	ldr	r4, [r1, #-2020]
	add	r2, r4, ip
	strh	r3, [r2, #2]	@ movhi
	strh	r3, [r4, ip]	@ movhi
	ldr	r3, [r1, #-2000]
	cmp	r3, #0
	beq	.L514
.L509:
	ldr	sl, [r1, #-2020]
	mov	r8, r0, asl #1
	ldr	r9, .L515+4
	rsb	r1, sl, r3
	ldr	r7, .L515+8
	mov	r1, r1, asr #1
	ldr	r7, [r7, #4052]
	mul	r1, r9, r1
	mov	r9, r5
	ldrh	r8, [r7, r8]
	uxth	r1, r1
.L512:
	mov	r5, r1, asl #1
	ldrh	r5, [r7, r5]
	cmp	r5, r8
	bcs	.L510
	ldrh	r5, [r3, #0]
	cmp	r5, r6
	streqh	r1, [r2, #2]	@ movhi
	streqh	r0, [r3, #0]	@ movhi
	beq	.L508
.L511:
	mla	r3, r9, r5, sl
	mov	r1, r5
	b	.L512
.L510:
	ldrh	r5, [r3, #2]
	strh	r5, [r2, #2]	@ movhi
	strh	r1, [r4, ip]	@ movhi
	ldr	r1, .L515
	ldr	ip, [r1, #-2000]
	cmp	r3, ip
	bne	.L513
	strh	r0, [r3, #2]	@ movhi
.L514:
	str	r2, [r1, #-2000]
	b	.L508
.L513:
	ldrh	r2, [r3, #2]
	mov	ip, #6
	ldr	r1, [r1, #-2020]
	mul	r2, ip, r2
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
.L508:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L516:
	.align	2
.L515:
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
	ldr	r2, .L518
	ldr	r3, .L518+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, pc}
.L519:
	.align	2
.L518:
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
	ldr	ip, .L525
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
	bne	.L521
	cmp	r4, r6
	mlane	r2, r2, r4, r3
	moveq	r2, #0
	streq	r2, [r0, #0]
	strne	r2, [r0, #0]
	mvnne	r0, #0
	strneh	r0, [r2, #2]	@ movhi
	b	.L523
.L521:
	cmp	r4, r6
	ldrh	r0, [r5, #2]
	bne	.L524
	cmp	r0, r4
	mulne	r2, r2, r0
	mvnne	r0, #0
	strneh	r0, [r3, r2]	@ movhi
	b	.L523
.L524:
	mla	r4, r2, r4, r3
	strh	r0, [r4, #2]	@ movhi
	ldrh	r6, [r5, #2]
	ldrh	r4, [r3, r1]
	ldr	r0, [ip, #-2020]
	mul	r2, r2, r6
	strh	r4, [r0, r2]	@ movhi
.L523:
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r3, r1]	@ movhi
	strh	r2, [r5, #2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L526:
	.align	2
.L525:
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
	beq	.L532
	ldr	r2, .L533
	movw	lr, #65535
	mov	ip, #6
	ldr	r4, [r2, #-2020]
	b	.L529
.L531:
	mla	r3, ip, r2, r4
	sub	r1, r1, #1
	uxth	r1, r1
.L529:
	cmp	r1, #0
	beq	.L530
	ldrh	r2, [r3, #0]
	cmp	r2, lr
	bne	.L531
.L530:
	rsb	r4, r4, r3
	ldr	r3, .L533+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	List_remove_node
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L532:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L534:
	.align	2
.L533:
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
	ldr	r2, .L541
	ldr	r3, [r2, #-2016]
	cmp	r3, #0
	beq	.L540
	ldr	r2, [r2, #-2020]
	movw	r1, #65535
	mov	ip, #6
	b	.L537
.L539:
	mla	r3, ip, r3, r2
	sub	r0, r0, #1
	uxth	r0, r0
.L537:
	cmp	r0, #0
	beq	.L538
	ldrh	r3, [r3, #0]
	cmp	r3, r1
	bne	.L539
	mov	r0, r1
	bx	lr
.L538:
	rsb	r3, r2, r3
	ldr	r0, .L541+4
	mov	r3, r3, asr #1
	mul	r0, r0, r3
	uxth	r0, r0
	bx	lr
.L540:
	movw	r0, #65535
	bx	lr
.L542:
	.align	2
.L541:
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
	ldr	r3, .L550
	ldr	r2, .L550+4
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L544
	ldr	r2, .L550+8
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L544
	ldr	r2, .L550+12
	ldrh	r2, [r3, r2]
	cmp	r2, r0
	beq	.L544
	mov	r5, #6
	ldr	r2, [r3, #-2020]
	mul	r5, r5, r0
	ldr	r0, [r3, #-2016]
	add	r1, r2, r5
	cmp	r1, r0
	beq	.L544
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
	bne	.L546
	ldrh	r1, [r2, r5]
	cmp	r1, r3
	beq	.L544
.L546:
	mov	r1, #6
	mla	r3, r1, r3, r2
	ldr	r1, .L550+16
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
	bcs	.L544
	ldr	r5, .L550
	mov	r1, r4
	sub	r0, r5, #2016
	bl	List_remove_node
	ldr	r3, .L550+20
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	INSERT_DATA_LIST
.L544:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L551:
	.align	2
.L550:
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
	b	.L553
.L558:
	mov	r7, r3
	add	r3, r3, #2
	ldrh	r6, [r7, #0]
	cmp	r6, #0
	bne	.L554
	bl	FtlFreeSysBlkQueueOut
	movw	r3, #65533
	sub	r2, r0, #1
	mov	r1, r0
	strh	r0, [r7, #0]	@ movhi
	uxth	r2, r2
	cmp	r2, r3
	bls	.L555
	ldr	r2, .L559
	movw	r3, #4066
	ldr	r0, .L559+4
	ldrh	r2, [r2, r3]
	bl	printk
.L556:
	b	.L556
.L555:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	strh	r5, [r4, #0]	@ movhi
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L557
.L554:
	add	r5, r5, #1
	uxth	r5, r5
.L553:
	cmp	r5, r2
	bne	.L558
.L557:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L560:
	.align	2
.L559:
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
	ldr	r2, .L573
	movw	r3, #3958
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	movw	r1, #65535
	ldrh	r3, [r2, r3]
	ldr	r2, .L573+4
	ldr	ip, [r2, #-1848]
	mov	r2, #0
	mov	r0, r2
	b	.L562
.L564:
	add	r2, r2, #12
	add	r4, ip, r2
	ldrh	r4, [r4, #-12]
	cmp	r4, r1
	ldmeqfd	sp!, {r4, r5, r6, r7, pc}
	add	r0, r0, #1
	uxth	r0, r0
.L562:
	cmp	r0, r3
	bne	.L564
	mov	r1, #0
	mov	r0, r3
	mov	r5, #-2147483648
	mov	r2, r1
	b	.L565
.L567:
	add	r4, ip, r1
	ldr	r4, [r4, #4]
	cmp	r4, #0
	blt	.L566
	cmp	r4, r5
	movcc	r5, r4
	movcc	r0, r2
.L566:
	add	r2, r2, #1
	add	r1, r1, #12
	uxth	r2, r2
.L565:
	cmp	r2, r3
	bne	.L567
	cmp	r0, r3
	ldmccfd	sp!, {r4, r5, r6, r7, pc}
	ldr	r2, .L573+8
	mov	r0, r3
	ldr	r1, .L573+4
	mvn	r4, #0
	ldrh	r6, [r1, r2]
	mov	r2, #0
	mov	r1, r2
	b	.L568
.L570:
	add	r5, ip, r2
	ldr	r5, [r5, #4]
	cmp	r5, r4
	bcs	.L569
	ldrh	r7, [ip, r2]
	cmp	r7, r6
	movne	r4, r5
	movne	r0, r1
.L569:
	add	r1, r1, #1
	add	r2, r2, #12
	uxth	r1, r1
.L568:
	cmp	r1, r3
	bne	.L570
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L574:
	.align	2
.L573:
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
	ldr	r3, .L581
	ldr	r2, .L581+4
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r1, [r2, r3]
	cmp	r1, #4
	add	ip, r1, #1
	mov	r1, r3
	strh	ip, [r2, r3]	@ movhi
	bhi	.L576
	cmp	r0, #0
	ldmeqfd	sp!, {r4, r5, pc}
.L576:
	ldr	r0, .L581+8
	mov	r3, #0
	strh	r3, [r2, r1]	@ movhi
	movw	r1, #3860
	str	r3, [r2, #-1840]
	movw	ip, #65535
	ldrh	r4, [r0, r1]
	ldr	r1, [r2, #-2012]
	ldr	r2, .L581+4
	b	.L578
.L580:
	ldrh	r0, [r1], #2
	add	r3, r3, #1
	cmp	r0, ip
	uxth	r3, r3
	ldrne	r5, [r2, #-1840]
	addne	r0, r0, r5
	strne	r0, [r2, #-1840]
.L578:
	cmp	r3, r4
	bne	.L580
	ldmfd	sp!, {r4, r5, pc}
.L582:
	.align	2
.L581:
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
	ldr	r3, .L587
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
.L588:
	.align	2
.L587:
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
	ldr	r3, .L590
	mov	r2, r0, lsr #5
	and	r0, r0, #31
	ldr	r3, [r3, #-1836]
	ldr	r3, [r3, r2, asl #2]
	mov	r0, r3, lsr r0
	and	r0, r0, #1
	bx	lr
.L591:
	.align	2
.L590:
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
	ldr	ip, .L599
	mov	r3, #0
	strh	r3, [r0, #4]	@ movhi
	movw	r3, #3852
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r3, [ip, r3]
	movw	ip, #65535
	b	.L593
.L595:
	add	r4, r0, r2, asl #1
	add	r2, r2, #1
	ldrh	r4, [r4, #16]
	uxth	r2, r2
	cmp	r4, ip
	ldrneh	r4, [r0, #4]
	addne	r4, r4, #1
	strneh	r4, [r0, #4]	@ movhi
.L593:
	cmp	r2, r3
	bcc	.L595
	ldr	ip, .L599
	movw	r2, #3918
	movw	r4, #65535
	mvn	r1, r1
	ldrh	r5, [ip, r2]
	mov	ip, r0
	mov	r2, #0
	b	.L596
.L598:
	ldrh	r6, [ip, #16]
	add	r2, r2, #1
	add	ip, ip, #2
	cmp	r6, r4
	uxth	r2, r2
	ldrneh	r6, [r0, #4]
	addne	r6, r5, r6
	addne	r6, r6, r1
	strneh	r6, [r0, #4]	@ movhi
.L596:
	cmp	r2, r3
	bne	.L598
	ldmfd	sp!, {r4, r5, r6, pc}
.L600:
	.align	2
.L599:
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
	ldr	r6, .L607
	strh	r5, [r0, #4]	@ movhi
	movw	r8, #3852
	strb	r5, [r4, #7]
	mvn	r7, #0
	b	.L602
.L604:
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
.L602:
	ldrh	r2, [r6, r8]
	ldr	r3, .L607
	cmp	r2, r5
	bhi	.L604
	movw	r1, #3918
	ldrb	r2, [r4, #7]	@ zero_extendqisi2
	ldrh	r1, [r3, r1]
	mul	r2, r1, r2
	strh	r2, [r4, #4]	@ movhi
	mov	r2, #0
	strb	r2, [r4, #9]
	ldr	r2, [r3, #3844]
	cmp	r2, #0
	beq	.L605
	ldrh	r1, [r4, #0]
	ldr	r2, [r3, #4052]
	mov	r3, r1, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #79
	movls	r3, #1
	strlsb	r3, [r4, #9]
.L605:
	ldr	r3, .L607
	mov	r0, #0
	ldrb	r3, [r3, #852]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #1
	strneb	r3, [r4, #9]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L608:
	.align	2
.L607:
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
	ldr	r6, .L614
	movw	sl, #3852
	movw	r8, #3918
	b	.L610
.L612:
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
.L610:
	ldrh	r3, [r6, sl]
	cmp	r3, r5
	bhi	.L612
	cmp	r4, #0
	beq	.L613
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r4, r0
.L613:
	ldr	r3, .L614+4
	mov	r2, #6
	mov	r0, #0
	ldr	r3, [r3, #-2020]
	mla	r7, r2, r7, r3
	strh	r4, [r7, #4]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L615:
	.align	2
.L614:
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
	ldr	r3, .L619
	ldr	r0, [r3, #-2000]
	cmp	r0, #0
	bxeq	lr
	ldr	r3, [r3, #-2020]
	ldr	r2, .L619+4
	rsb	r0, r3, r0
	ldr	r3, .L619+8
	mov	r0, r0, asr #1
	ldr	r2, [r2, #4052]
	mul	r3, r3, r0
	uxth	r3, r3
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	bx	lr
.L620:
	.align	2
.L619:
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
	ldr	r2, .L628
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	ldr	r3, [r2, #-2000]
	cmp	r3, #0
	beq	.L627
	ldr	r1, .L628+4
	mov	ip, #7
	mov	r5, #6
	movw	r4, #65535
	ldrh	r1, [r2, r1]
	mul	r1, ip, r1
	mov	r1, r1, asr #3
	cmp	r0, r1
	uxthgt	r0, r1
	ldr	r1, [r2, #-2020]
	ldr	r2, .L628+8
	rsb	r3, r1, r3
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	mov	r2, #0
	uxth	r3, r3
	b	.L624
.L626:
	mul	ip, r5, r3
	ldrh	ip, [r1, ip]
	cmp	ip, r4
	beq	.L625
	add	r2, r2, #1
	mov	r3, ip
	uxth	r2, r2
.L624:
	cmp	r2, r0
	bne	.L626
.L625:
	ldr	r2, .L628+12
	mov	r3, r3, asl #1
	ldr	r2, [r2, #4052]
	ldrh	r0, [r2, r3]
	ldmfd	sp!, {r4, r5, pc}
.L627:
	mov	r0, r3
	ldmfd	sp!, {r4, r5, pc}
.L629:
	.align	2
.L628:
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
	ldr	r6, .L640
	add	r4, sl, #12
	.pad #32
	sub	sp, sp, #32
	ldr	r1, .L640+4
	bl	strcpy
	mov	r0, r4
	ldr	r1, .L640+8
	ldr	r2, [r6, #3044]
	bl	sprintf
	ldr	r1, .L640+12
	ldr	r2, [r6, #3936]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+16
	ldr	r3, [r3, #3220]
	cmp	r3, #1
	add	r4, r4, r0
	bne	.L636
.L631:
	add	r0, sp, #16
	add	r1, sp, #20
	add	r2, sp, #24
	add	r3, sp, #28
	bl	NandcGetTimeCfg
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #16]
	mov	r0, r4
	ldr	r1, .L640+20
	str	r3, [sp, #0]
	ldr	r3, [sp, #28]
	ldr	r5, .L640+24
	ldr	r7, .L640+28
	str	r3, [sp, #4]
	ldr	r3, [sp, #20]
	bl	sprintf
	ldr	r1, .L640+32
	ldr	r8, .L640+36
	add	r4, r4, r0
	mov	r0, r4
	add	r4, r4, #10
	bl	strcpy
	ldr	r1, .L640+40
	mov	r0, r4
	ldr	r2, [r6, #3980]
	bl	sprintf
	ldr	r1, .L640+44
	ldr	r2, [r5, #-1840]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+48
	ldr	r2, [r5, #-1832]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+52
	ldr	r2, [r5, #-1828]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+56
	ldr	r2, [r5, #-1824]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+60
	ldr	r2, [r5, #-1820]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+64
	ldr	r2, [r5, #-1816]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+68
	ldr	r2, [r5, #-1812]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r2, [r5, #-1808]
	ldr	r1, .L640+72
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r2, [r5, #-1804]
	ldr	r1, .L640+76
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+80
	ldr	r2, [r5, #-1800]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+84
	ldr	r2, [r5, #-1796]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	bl	FtlBbtCalcTotleCnt
	movw	r2, #3990
	ldr	r1, .L640+88
	ldrh	r2, [r6, r2]
	mov	r3, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+92
	ldrh	r2, [r5, r7]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+96
	ldr	r2, [r5, #-1792]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+100
	ldr	r2, [r5, #-1788]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+104
	ldr	r2, [r5, #-1784]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+108
	ldr	r2, [r6, #4056]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+112
	ldr	r2, [r5, #-1780]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+116
	ldr	r2, [r5, #-1776]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+120
	ldr	r1, .L640+124
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+128
	ldr	r1, .L640+132
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+136
	ldr	r2, [r6, #3960]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+140
	ldr	r2, [r6, #3952]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+144
	ldr	r2, [r6, #3848]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #4066
	ldrh	r2, [r6, r3]
	ldr	r1, .L640+148
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3860
	ldrh	r2, [r6, r3]
	ldr	r1, .L640+152
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+156
	ldr	r1, .L640+160
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+164
	ldr	r2, [r6, #3864]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+168
	ldr	r1, .L640+172
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	mov	r3, #3984
	ldrh	r2, [r6, r3]
	ldr	r1, .L640+176
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+180
	ldr	r1, .L640+184
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+188
	ldrb	r2, [r5, #-1986]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r2, [r5, r8]
	ldr	r1, .L640+192
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+196
	ldrb	r2, [r5, #-1984]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+200
	ldr	r1, .L640+204
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r5, r8]
	ldr	r2, [r5, #-2012]
	add	r8, r8, #48
	ldr	r1, .L640+208
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+212
	ldr	r1, .L640+216
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+220
	ldrb	r2, [r5, #-1938]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r2, [r5, r8]
	ldr	r1, .L640+224
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+228
	ldrb	r2, [r5, #-1936]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+232
	ldr	r1, .L640+236
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r5, r8]
	ldr	r2, [r5, #-2012]
	add	r8, r8, #192
	ldr	r1, .L640+240
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+244
	ldr	r1, .L640+248
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+252
	ldrb	r2, [r5, #-1890]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+256
	ldr	r1, .L640+260
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+264
	ldrb	r2, [r5, #-1888]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+268
	ldr	r1, .L640+272
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+276
	ldr	r1, .L640+280
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+284
	ldrb	r2, [r5, #-1746]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+288
	ldrh	r2, [r5, r8]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+292
	ldrb	r2, [r5, #-1744]	@ zero_extendqisi2
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+296
	ldr	r1, .L640+300
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
	ldr	r1, .L640+304
	ldr	r3, [r5, #-1620]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+308
	ldr	r2, [r5, #-1628]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+312
	ldr	r2, [r5, #-1604]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+316
	ldr	r1, .L640+320
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+324
	ldr	r1, .L640+328
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L640+332
	ldr	r2, [r5, #-1184]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, .L640+336
	ldr	r1, .L640+340
	ldrh	r2, [r5, r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r1, .L640+344
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	add	r4, r4, r0
	ldrh	r0, [r5, r7]
	bl	GetFreeBlockMaxEraseCount
	ldr	r1, .L640+348
	mov	r2, r0
	mov	r0, r4
	bl	sprintf
	ldrh	r3, [r5, r8]
	movw	r2, #65535
	cmp	r3, r2
	add	r4, r4, r0
	beq	.L633
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	mov	r0, r4
	ldr	r1, .L640+352
	ldrh	r2, [r2, r3]
	bl	sprintf
	add	r4, r4, r0
.L633:
	mov	r0, #0
	ldr	r6, .L640+24
	bl	List_get_gc_head_node
	ldr	r8, .L640
	mov	r5, #0
	movw	r9, #65535
	uxth	r3, r0
.L635:
	cmp	r3, r9
	beq	.L634
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
	ldr	r1, .L640+356
	str	r2, [sp, #8]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-2020]
	cmp	r5, #16
	ldrh	r3, [r3, r7]
	add	r4, r4, r0
	bne	.L635
.L634:
	ldr	r6, .L640+24
	mov	r5, #0
	ldr	r8, .L640
	movw	r9, #65535
	ldr	r2, [r6, #-2000]
	ldr	r3, [r6, #-2020]
	rsb	r3, r3, r2
	ldr	r2, .L640+360
	mov	r3, r3, asr #1
	mul	r3, r2, r3
	uxth	r3, r3
.L637:
	cmp	r3, r9
	beq	.L636
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
	ldr	r1, .L640+364
	str	r2, [sp, #4]
	mov	r2, r5
	bl	sprintf
	add	r5, r5, #1
	ldr	r3, [r6, #-2020]
	cmp	r5, #4
	ldrh	r3, [r3, r7]
	add	r4, r4, r0
	bne	.L637
.L636:
	rsb	r0, sl, r4
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L641:
	.align	2
.L640:
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
	ldr	r1, .L643
	ldr	r2, .L643+4
	bl	sprintf
	add	r5, r4, r0
	mov	r0, r5
	bl	FtlPrintInfo2buf
	add	r0, r5, r0
	rsb	r0, r4, r0
	ldmfd	sp!, {r3, r4, r5, pc}
.L644:
	.align	2
.L643:
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
	ldr	r3, .L670
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r1, [r3, #-1784]
	ldr	r2, [r3, #-1776]
	cmp	r1, r2
	bcs	.L646
	ldr	r0, .L670+4
	movw	r1, #3860
	mov	r2, #0
	str	r2, [r3, #-1792]
	ldrh	r1, [r0, r1]
	ldr	r0, [r0, #4052]
	b	.L647
.L648:
	ldrh	lr, [r0], #2
	add	r2, r2, #1
	ldr	ip, [r3, #-1792]
	add	ip, lr, ip
	str	ip, [r3, #-1792]
.L647:
	cmp	r2, r1
	ldr	r4, .L670
	bcc	.L648
	ldr	r5, [r4, #-1792]
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r2, .L670+4
	movw	r3, #3910
	ldrh	r1, [r2, r3]
	str	r0, [r4, #-1784]
	ldr	r0, [r4, #-1788]
	rsb	r0, r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #-1792]
	b	.L649
.L646:
	ldr	r2, [r3, #-1780]
	cmp	r1, r2
	bls	.L649
	add	r2, r2, #1
	movw	ip, #3860
	str	r2, [r3, #-1780]
	mov	r3, #0
	ldr	r2, .L670+4
	b	.L650
.L651:
	ldr	r0, [r2, #4052]
	mov	r1, r3, asl #1
	add	r3, r3, #1
	ldrh	r4, [r0, r1]
	add	r4, r4, #1
	strh	r4, [r0, r1]	@ movhi
.L650:
	ldrh	r1, [r2, ip]
	cmp	r3, r1
	bcc	.L651
.L649:
	ldr	r3, .L670
	ldr	r6, [r3, #-1776]
	ldr	r5, [r3, #-1784]
	add	r2, r6, #256
	cmp	r2, r5
	mov	r2, r3
	bls	.L652
	ldr	r3, [r3, #-1780]
	add	r1, r6, #768
	cmp	r1, r3
	bls	.L652
	ldr	r3, .L670+4
	ldr	r3, [r3, #3844]
	cmp	r3, #0
	beq	.L669
	cmp	r6, #40
	bhi	.L669
.L652:
	ldr	r3, .L670+8
	ldrh	r0, [r2, r3]
	add	r0, r0, r0, asl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	cmp	r0, r3
	mov	r8, r0
	bcs	.L654
	cmp	r6, #40
	bhi	.L669
.L654:
	ldr	r1, .L670
	ldr	r3, [r1, #-2016]
	cmp	r3, #0
	beq	.L669
	ldr	r2, .L670+4
	movw	r0, #3860
	movw	r7, #65535
	mov	r4, r7
	mov	fp, r7
	ldrh	sl, [r2, r0]
	ldr	r0, [r1, #-2020]
	ldr	r1, [r2, #4052]
	mov	r2, #0
	b	.L655
.L658:
	add	r2, r2, #1
	uxth	r2, r2
	cmp	r2, sl
	bhi	.L669
	ldrh	ip, [r3, #4]
	cmp	ip, #0
	beq	.L656
	rsb	r3, r0, r3
	ldr	ip, .L670+12
	mov	r3, r3, asr #1
	mul	r3, ip, r3
	uxth	r3, r3
	mov	ip, r3, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, r6
	bls	.L666
	cmp	ip, r7
	movcc	r7, ip
	movcc	r4, r3
.L656:
	mov	ip, #6
	mla	r3, ip, r9, r0
.L655:
	ldrh	r9, [r3, #0]
	cmp	r9, fp
	bne	.L658
	b	.L657
.L666:
	mov	r4, r3
.L657:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L653
	mov	r9, r4, asl #1
	ldrh	sl, [r1, r9]
	cmp	sl, r6
	bls	.L659
	bl	GetFreeBlockMinEraseCount
	cmp	r0, r6
	ldrhi	r3, .L670
	strhi	r7, [r3, #-1776]
.L659:
	cmp	sl, r5
	bcs	.L669
	add	r3, sl, #128
	cmp	r8, r3
	ble	.L669
	add	r3, sl, #256
	ldr	r6, .L670
	cmp	r3, r5
	bcc	.L660
	ldr	r3, [r6, #-1780]
	add	sl, sl, #768
	cmp	sl, r3
	bcs	.L669
.L660:
	ldr	r3, [r6, #-2012]
	mov	r1, r4
	ldr	r0, .L670+16
	mov	r2, r5
	ldrh	r3, [r3, r9]
	str	r3, [sp, #0]
	ldr	r3, .L670+4
	ldr	r3, [r3, #4052]
	ldrh	r3, [r3, r9]
	stmib	sp, {r3, r8}
	ldr	r3, [r6, #-1780]
	bl	printk
	mov	r3, #1
	str	r3, [r6, #-1176]
	b	.L653
.L669:
	movw	r4, #65535
.L653:
	mov	r0, r4
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L671:
	.align	2
.L670:
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
	beq	.L673
	ldr	r2, .L674
	mov	r3, r0, asl #1
	mov	r1, #0
	ldr	r2, [r2, #-2012]
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L673:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L675:
	.align	2
.L674:
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
	ldr	r2, .L684
	mov	r3, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	movw	sl, #3852
	str	r3, [r2, #-1172]
	mov	r8, #12
	ldr	r0, .L684+4
	mov	r7, #1
	movw	r6, #3928
	movw	fp, #3930
	b	.L677
.L678:
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
.L677:
	ldrh	r1, [r0, sl]
	cmp	r3, r1
	bcc	.L678
	b	.L683
.L680:
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
	b	.L682
.L683:
	ldr	r3, .L684
	mov	r8, #12
	ldr	ip, .L684+4
	mov	r7, #0
	movw	r6, #3928
	movw	r5, #3930
.L682:
	ldr	r2, [r3, #-1152]
	cmp	r1, r2
	bcc	.L680
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L685:
	.align	2
.L684:
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
	ldr	r3, .L693
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r5, #36
	ldr	r6, [r3, #-1152]
	mov	fp, #12
	ldr	ip, [r3, #-1168]
	mov	r3, #0
	mov	r4, r3
	b	.L687
.L690:
	mul	sl, fp, r2
	add	r8, ip, sl
	ldr	r9, [ip, sl]
	ldr	sl, [r7, #8]
	cmp	r9, sl
	streq	r4, [r8, #8]
	beq	.L689
.L688:
	add	r2, r2, #1
	uxth	r2, r2
.L692:
	cmp	r2, r6
	bcc	.L690
.L689:
	add	r3, r3, #1
	uxth	r3, r3
.L687:
	cmp	r3, r1
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
	mla	r7, r5, r3, r0
	mov	r2, #0
	b	.L692
.L694:
	.align	2
.L693:
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
	ldr	r3, .L702
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r6, #12
	ldr	r8, [r3, #-1152]
	mov	r5, #1
	ldr	r7, [r3, #-1168]
	mov	r4, #36
	mov	r3, #0
	b	.L696
.L699:
	mla	ip, r6, r2, r7
	ldr	sl, [ip, #8]
	cmp	sl, #0
	bne	.L697
	mla	r2, r4, r3, r0
	ldr	sl, [ip, #0]
	str	r5, [ip, #8]
	str	sl, [r2, #8]
	ldr	ip, [ip, #4]
	str	ip, [r2, #12]
	b	.L698
.L697:
	add	r2, r2, #1
	uxth	r2, r2
	b	.L700
.L701:
	mov	r2, #0
.L700:
	cmp	r2, r8
	bcc	.L699
.L698:
	add	r3, r3, #1
	uxth	r3, r3
.L696:
	cmp	r3, r1
	bcc	.L701
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L703:
	.align	2
.L702:
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
	ldr	r3, .L709
	ldr	r2, .L709+4
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #-1144]
	mov	r3, #0
	b	.L705
.L707:
	ldrh	ip, [r2], #2
	cmp	ip, r0
	beq	.L708
	add	r3, r3, #1
	uxth	r3, r3
.L705:
	cmp	r3, r1
	bne	.L707
	mov	r0, #0
	bx	lr
.L708:
	mov	r0, #1
	bx	lr
.L710:
	.align	2
.L709:
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
	ldr	r3, .L715
	ldr	r2, .L715+4
	ldrh	ip, [r3, r2]
	ldr	r2, [r3, #-1144]
	mov	r3, #0
	mov	r1, r2
	b	.L712
.L714:
	ldrh	r7, [r1], #2
	cmp	r7, r0
	beq	.L713
	add	r3, r3, #1
	uxth	r3, r3
.L712:
	cmp	r3, ip
	bne	.L714
	mov	r3, r3, asl #1
	strh	r0, [r2, r3]	@ movhi
	ldr	r2, .L715
	ldr	r3, .L715+4
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L713:
	ldr	r3, .L715
	mov	r0, #12
	ldr	r2, .L715+8
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
.L716:
	.align	2
.L715:
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
	ldr	r0, .L722
	bl	printk
	ldr	r8, .L722+4
	ldr	r3, .L722+8
	ldrh	sl, [r3, r8]
	cmp	sl, r4
	beq	.L718
	ldr	r6, .L722+12
	ldrh	r7, [r3, r6]
	cmp	r7, r4
	beq	.L718
	ldr	ip, .L722+16
	ldrh	r5, [r3, ip]
	cmp	r5, r4
	beq	.L718
	ldr	r1, .L722+20
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	beq	.L718
	movw	r2, #65535
	cmp	sl, r2
	streqh	r4, [r3, r8]	@ movhi
	beq	.L718
	cmp	r7, r2
	streqh	r4, [r3, r6]	@ movhi
	beq	.L718
	cmp	r5, r2
	streqh	r4, [r3, ip]	@ movhi
	beq	.L718
	cmp	r0, r2
	streqh	r4, [r3, r1]	@ movhi
.L718:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L723:
	.align	2
.L722:
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
	ldr	r0, .L735
	bl	printk
	ldr	r8, .L735+4
	ldr	r3, .L735+8
	ldrh	sl, [r3, r8]
	cmp	sl, r4
	beq	.L732
	ldr	r6, .L735+12
	ldrh	r7, [r3, r6]
	cmp	r7, r4
	beq	.L732
	ldr	ip, .L735+16
	ldrh	r5, [r3, ip]
	cmp	r5, r4
	beq	.L732
	ldr	r1, .L735+20
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	beq	.L733
	movw	r2, #65535
	cmp	sl, r2
	streqh	r4, [r3, r8]	@ movhi
	beq	.L732
	cmp	r7, r2
	streqh	r4, [r3, r6]	@ movhi
	beq	.L732
	cmp	r5, r2
	streqh	r4, [r3, ip]	@ movhi
	beq	.L732
	cmp	r0, r2
	streqh	r4, [r3, r1]	@ movhi
	mvnne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L732:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L733:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L736:
	.align	2
.L735:
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
	ldr	r5, .L739
	mov	r4, r0
	mov	r1, r4
	ldr	r0, .L739+4
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
	beq	.L738
	mov	r1, r4
	sub	r0, r5, #2000
	bl	List_remove_node
	ldr	r3, .L739+8
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r5, #-2012]
	ldr	r0, .L739+12
	mov	r1, r4
	ldrh	r2, [r3, r6]
	bl	printk
.L738:
	mov	r0, r4
	bl	FtlGcRefreshBlock
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L740:
	.align	2
.L739:
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
	ldr	r7, .L746
	mov	r2, r4
	ldr	r6, .L746+4
	ldrh	r1, [r7, r6]
	mov	r5, r0
	ldr	r0, .L746+8
	bl	printk
	mov	r0, r5
	bl	FtlGcRefreshBlock
	ldr	r3, .L746+12
	ldr	r2, [r3, #3844]
	cmp	r2, #0
	beq	.L742
	ldr	r3, [r3, #4052]
	mov	r5, r5, asl #1
	ldrh	r2, [r3, r5]
	cmp	r2, #39
	subhi	r2, r2, #40
	strhih	r2, [r3, r5]	@ movhi
.L742:
	ldrh	r1, [r7, r6]
	mov	r3, #0
	ldr	r2, .L746+16
	b	.L743
.L745:
	ldrh	r0, [r2, #2]!
	cmp	r0, r4
	beq	.L744
	add	r3, r3, #1
	uxth	r3, r3
.L743:
	cmp	r3, r1
	bne	.L745
	cmp	r3, #15
	bhi	.L744
	ldr	r2, .L746
	ldr	r1, .L746+20
	add	r0, r2, r3, asl #1
	add	r3, r3, #1
	strh	r4, [r0, r1]	@ movhi
	sub	r1, r1, #2
	strh	r3, [r2, r1]	@ movhi
.L744:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L747:
	.align	2
.L746:
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
	ldr	r2, .L751
	ldr	r3, .L751+4
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L749
	ldr	r1, .L751+8
	ldrh	r0, [r3, r1]
	movw	r1, #65535
	cmp	r0, r1
	bne	.L749
	ldr	r4, .L751+12
	ldr	r5, .L751+4
	ldrh	r1, [r3, r4]
	cmp	r1, r2
	movcs	r2, #0
	strcsh	r2, [r3, r4]	@ movhi
	ldrh	r2, [r5, r4]
	ldr	r3, .L751+16
	add	r2, r5, r2, asl #1
	ldrh	r0, [r2, r3]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r5, r4]
	add	r3, r3, #1
	strh	r3, [r5, r4]	@ movhi
.L749:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L752:
	.align	2
.L751:
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
	beq	.L754
	bl	memset
.L754:
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
	ldr	r4, .L756
	movw	r5, #3922
	ldr	r2, .L756+4
	mov	r1, #255
	ldr	r6, .L756+8
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
.L757:
	.align	2
.L756:
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
	ldr	r7, .L768
	mov	r2, #6
	ldr	r4, .L768+4
	mov	r1, #0
	mov	r6, #0
	ldrh	r3, [r7, r3]
	mov	sl, r6
	ldr	r0, [r4, #-2020]
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r3, .L768+8
	str	r6, [r4, #-2000]
	str	r6, [r4, #-2016]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #8
	str	r6, [r4, #-2008]
	strh	r6, [r4, r3]	@ movhi
	add	r3, r3, #908
	strh	r6, [r4, r3]	@ movhi
	mov	r4, r6
	b	.L759
.L761:
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
	b	.L766
.L767:
	mov	r5, #0
	uxth	r3, r4
	mov	r9, r5
	movw	fp, #3852
	movw	r2, #3918
.L766:
	ldrh	r0, [r7, fp]
	sxth	r1, r9
	cmp	r1, r0
	blt	.L761
	cmp	r5, #0
	ldr	r9, .L768+4
	beq	.L762
	sxth	r1, r5
	mov	r0, #32768
	bl	__aeabi_idiv
	uxth	r5, r0
	b	.L763
.L762:
	sxth	r3, r4
	ldr	r2, [r9, #-2012]
	mvn	r1, #0
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
.L763:
	sxth	r1, r4
	ldr	r0, [r9, #-2020]
	ldr	r3, .L768+4
	mov	r2, r1, asl #1
	add	r1, r2, r1
	add	r1, r0, r1, asl #1
	strh	r5, [r1, #4]	@ movhi
	ldr	r1, .L768+12
	ldrh	r1, [r9, r1]
	cmp	r8, r1
	beq	.L764
	ldr	r1, .L768+16
	ldrh	r1, [r3, r1]
	cmp	r8, r1
	beq	.L764
	ldr	r1, .L768+20
	ldrh	r1, [r3, r1]
	cmp	r8, r1
	beq	.L764
	ldr	r3, [r3, #-2012]
	uxth	r0, r4
	ldrh	r3, [r3, r2]
	cmp	r3, #0
	bne	.L765
	add	r6, r6, #1
	uxth	r6, r6
	bl	INSERT_FREE_LIST
	b	.L764
.L765:
	add	sl, sl, #1
	uxth	sl, sl
	bl	INSERT_DATA_LIST
.L764:
	add	r4, r4, #1
	uxth	r4, r4
.L759:
	movw	r2, #3860
	sxth	r8, r4
	ldrh	r3, [r7, r2]
	cmp	r8, r3
	blt	.L767
	ldr	r3, .L768+4
	mov	r0, #0
	ldr	r2, .L768+8
	strh	sl, [r3, r2]	@ movhi
	add	r2, r2, #8
	strh	r6, [r3, r2]	@ movhi
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L769:
	.align	2
.L768:
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
	ldr	r4, .L773
	movw	r6, #3928
	ldr	r5, .L773+4
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
	b	.L771
.L772:
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
.L771:
	ldrh	r3, [r4, r7]
	ldr	r1, .L773
	cmp	r3, r2
	ldr	r3, .L773+4
	bhi	.L772
	ldr	r0, .L773+8
	mvn	r2, #0
	strh	r2, [r3, r0]	@ movhi
	sub	r0, r0, #2
	strh	r2, [r3, r0]	@ movhi
	ldr	r0, [r1, #3948]
	ldr	r2, .L773+12
	strh	r0, [r3, r2]	@ movhi
	ldr	r0, .L773+16
	ldr	r2, .L773+20
	strh	r0, [r3, r2]	@ movhi
	add	r2, r2, #40
	ldrh	r0, [r3, r2]
	sub	r2, r2, #36
	strh	r0, [r3, r2]	@ movhi
	movw	r2, #3956
	ldrh	r1, [r1, r2]
	ldr	r2, .L773+24
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
.L774:
	.align	2
.L773:
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
	b	.L776
.L780:
	ldr	r0, [r7, r3, asl #2]
	mov	r2, #0
	ubfx	r0, r0, #10, #16
	b	.L777
.L779:
	mov	r1, r2, asl #1
	ldrh	ip, [r6, r1]
	cmp	ip, r0
	bne	.L778
	cmp	r0, #0
	ldrneh	ip, [r5, r1]
	addne	ip, ip, #1
	strneh	ip, [r5, r1]	@ movhi
.L778:
	add	r2, r2, #1
	uxth	r2, r2
.L777:
	ldrh	r1, [r4, #10]
	cmp	r1, r2
	bhi	.L779
	add	r3, r3, #1
	uxth	r3, r3
.L776:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L780
	ldr	r2, .L786
	mov	r3, #3920
	mov	r8, #0
	mov	r7, r8
	ldrh	r2, [r2, r3]
	ldrh	r3, [r4, #0]
	mov	r3, r3, asl #1
	strh	r2, [r5, r3]	@ movhi
	ldrh	fp, [r5, #0]
	b	.L781
.L784:
	mov	sl, r7, asl #1
	ldrh	r9, [r5, sl]
	cmp	fp, r9
	bls	.L782
	ldrh	r3, [r6, sl]
	cmp	r3, #0
	movne	r8, r7
	movne	fp, r9
.L782:
	cmp	r9, #0
	bne	.L783
	ldrh	r0, [r6, sl]
	cmp	r0, #0
	beq	.L783
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	strh	r9, [r6, sl]	@ movhi
	ldrh	r3, [r4, #8]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L783:
	add	r7, r7, #1
	uxth	r7, r7
.L781:
	ldrh	r3, [r4, #10]
	cmp	r3, r7
	bhi	.L784
	mov	r0, r8
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L787:
	.align	2
.L786:
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
	ldr	r3, .L789
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
	ldr	r0, .L789+4
	bl	ftl_memset
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L790:
	.align	2
.L789:
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
	ldr	r3, .L792
	mov	r2, #3984
	mvn	r1, #0
	ldr	r0, .L792+4
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #6
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #255
	mov	r2, #16
	b	ftl_memset
.L793:
	.align	2
.L792:
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
	ldr	r3, .L798
	ldr	r2, .L798+4
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
.L796:
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
	bne	.L796
	ldmfd	sp!, {r4, r5, r6, pc}
.L799:
	.align	2
.L798:
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
	ldr	r6, .L801
	mov	r4, #0
	ldr	r2, .L801+4
	mov	r1, r4
	ldr	r5, .L801+8
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
	ldr	r0, .L801+12
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #512
	ldr	r0, .L801+16
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L802:
	.align	2
.L801:
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
	ldr	r4, .L834
	mov	r0, #1024
	ldr	r3, .L834+4
	mov	r7, #12
	ldr	r2, .L834+8
	movw	sl, #3852
	ldr	r5, .L834+12
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
	ldr	r3, .L834+16
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
	ldr	r4, .L834
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
	ldr	sl, .L834+20
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
	ldr	r6, .L834+24
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
	ldr	ip, .L834+12
	mov	r1, r1, asl #2
	mov	r2, r1
	str	r0, [r5, #4012]
	ldrh	r5, [r5, r7]
	ldr	r0, .L834+28
	b	.L805
.L806:
	ldr	r4, [ip, #4012]
	add	r3, r3, #1
	add	r4, r4, r2
	add	r2, r2, r1
	str	r4, [r0, #4]!
.L805:
	cmp	r3, r5
	bcc	.L806
	ldr	r0, .L834+32
	mov	r2, #0
	mov	r1, r2
	add	r0, r0, r3, asl #2
	b	.L807
.L808:
	add	ip, r0, r2
	add	r3, r3, #1
	add	r2, r2, #4
	str	r1, [ip, #28]
.L807:
	cmp	r3, #7
	bls	.L808
	ldr	r3, .L834
	ldr	r2, [r3, #-1028]
	cmp	r2, #0
	beq	.L833
.L809:
	ldr	r2, [r3, #-1084]
	cmp	r2, #0
	beq	.L833
.L811:
	ldr	r2, [r3, #-1020]
	cmp	r2, #0
	beq	.L833
.L812:
	ldr	r2, [r3, #-1024]
	cmp	r2, #0
	beq	.L833
.L813:
	ldr	r2, [r3, #-1848]
	cmp	r2, #0
	beq	.L833
.L814:
	ldr	r2, [r3, #-1080]
	cmp	r2, #0
	beq	.L833
.L815:
	ldr	r2, [r3, #-2020]
	cmp	r2, #0
	beq	.L833
.L816:
	ldr	r2, .L834+12
	ldr	r2, [r2, #4012]
	cmp	r2, #0
	beq	.L833
.L817:
	ldr	r3, [r3, #-2012]
	cmp	r3, #0
	beq	.L833
.L818:
	ldr	r3, .L834
	ldr	r2, [r3, #-1144]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-1140]
	cmp	r2, #0
	beq	.L833
.L820:
	ldr	r2, [r3, #-960]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-952]
	cmp	r2, #0
	beq	.L833
	ldr	r2, .L834+12
	ldr	r2, [r2, #4048]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-1156]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-956]
	cmp	r2, #0
	beq	.L833
.L822:
	ldr	r2, [r3, #-948]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-944]
	cmp	r2, #0
	beq	.L833
	ldr	r3, [r3, #-940]
	cmp	r3, #0
	beq	.L833
	ldr	r3, .L834
	ldr	r2, [r3, #-1164]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-932]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-928]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-1168]
	cmp	r2, #0
	beq	.L833
.L824:
	ldr	r2, [r3, #-924]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-920]
	cmp	r2, #0
	beq	.L833
	ldr	r2, [r3, #-1160]
	cmp	r2, #0
	beq	.L833
.L826:
	ldr	r2, .L834+12
	ldr	r1, [r2, #4052]
	cmp	r1, #0
	beq	.L833
	ldr	r3, [r3, #-992]
	cmp	r3, #0
	beq	.L833
.L828:
	ldr	r3, [r2, #3968]
	cmp	r3, #0
	beq	.L833
.L829:
	ldr	r3, .L834
	ldr	r2, [r3, #-900]
	cmp	r2, #0
	beq	.L833
.L830:
	ldr	r2, [r3, #-896]
	cmp	r2, #0
	beq	.L833
.L831:
	ldr	r3, [r3, #-892]
	cmp	r3, #0
	bne	.L832
.L833:
	ldr	r0, .L834+36
	ldr	r1, .L834+40
	bl	printk
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L832:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L835:
	.align	2
.L834:
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
	ldr	r3, .L839
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
	ldr	r1, .L839+4
	b	.L837
.L838:
	ldrb	r5, [r3, r1]	@ zero_extendqisi2
	add	r3, r3, #1
	ldrb	ip, [r4, #10]	@ zero_extendqisi2
	orr	ip, ip, r2, asl r5
	strb	ip, [r4, #10]
.L837:
	uxtb	ip, r3
	cmp	ip, r0
	bcc	.L838
	ldmfd	sp!, {r3, r4, r5, pc}
.L840:
	.align	2
.L839:
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
	ldr	r3, .L850
	mov	r6, r4
	ldr	r2, .L850+4
	ldr	r5, .L850+8
	strb	r4, [r3, #-884]
	movw	r3, #3126
	ldrh	r0, [r2, r3]
	add	r7, r5, #2976
	strb	r4, [r5, #3762]
	bl	FlashBlockAlignInit
	mov	r1, r4
	mov	r2, #8
	ldr	r0, .L850+12
	mov	r8, r7
	bl	ftl_memset
	mov	r1, r4
	mov	r2, #32
	ldr	r0, .L850+16
	bl	ftl_memset
	ldr	r0, .L850+20
	mov	r1, r4
	mov	r2, #128
	bl	ftl_memset
.L843:
	ldr	r4, [r5, #3624]
	mov	r1, r8
	add	r0, r4, #1
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L842
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	add	r2, r5, r3, asl #2
	str	r0, [r2, #3588]
	add	r2, r5, r3
	add	r3, r3, #1
	strb	r3, [r5, #3762]
	strb	r6, [r2, #3764]
.L842:
	add	r6, r6, #1
	add	r8, r8, #8
	cmp	r6, #4
	bne	.L843
	ldr	r5, .L850+8
	ldr	r3, .L850
	ldrb	r2, [r5, #3762]	@ zero_extendqisi2
	strb	r2, [r3, #-884]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #2
	bne	.L844
	add	sl, r4, #1
	mov	r6, #0
	mov	r8, r5
.L847:
	mov	r0, sl
	mov	r1, r7
	ldrb	r2, [r4, #0]	@ zero_extendqisi2
	bl	FlashMemCmp8
	cmp	r0, #0
	bne	.L845
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
.L845:
	add	r6, r6, #1
	add	r7, r7, #8
	cmp	r6, #4
	bne	.L847
.L844:
	ldr	r3, .L850+8
	ldrb	r1, [r4, #13]	@ zero_extendqisi2
	ldr	r2, .L850
	ldrb	r3, [r3, #3762]	@ zero_extendqisi2
	mul	r1, r1, r3
	ldrh	r3, [r4, #14]
	mul	r1, r3, r1
	ldr	r3, .L850+24
	strh	r1, [r2, r3]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L851:
	.align	2
.L850:
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
	bne	.L853
	ldr	r3, .L895
.L854:
	strh	r0, [r3, #2]!	@ movhi
	add	r0, r0, #1
	uxth	r0, r0
	cmp	r0, #256
	bne	.L854
	b	.L855
.L853:
	cmp	r0, #1
	bne	.L856
	ldr	r0, .L895+4
	mov	r2, #0
	mov	r3, r2
.L859:
	uxth	r1, r3
	cmp	r1, #3
	bls	.L857
	tst	r3, #1
	moveq	r1, #2
	movne	r1, #3
	rsb	r1, r1, r2
	uxth	r1, r1
.L857:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L859
	b	.L855
.L856:
	cmp	r0, #2
	bne	.L860
	mov	r3, #0
	ldr	r0, .L895
	movw	r2, #65535
	mov	r1, r3
	b	.L894
.L863:
	cmp	r3, #1
	movls	r1, r3
	movhi	r1, r2
.L894:
	add	r3, r3, #1
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r3, r3
	uxth	r2, r2
	cmp	r3, #256
	bne	.L863
	b	.L855
.L860:
	cmp	r0, #3
	bne	.L864
	ldr	r0, .L895+4
	mov	r2, #0
	mov	r3, r2
.L867:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L865
	tst	r3, #1
	moveq	r1, #4
	movne	r1, #5
	rsb	r1, r1, r2
	uxth	r1, r1
.L865:
	add	r3, r3, #1
	strh	r1, [r2, r0]	@ movhi
	cmp	r3, #256
	add	r2, r2, #2
	bne	.L867
	b	.L855
.L864:
	cmp	r0, #4
	mov	r2, #0
	bne	.L868
	ldr	r3, .L895+8
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
	ldr	r0, .L895+12
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #3088
	mov	r1, #7
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #2
	mov	r1, #8
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #16
	mov	r3, r1
.L870:
	tst	r3, #1
	add	r3, r3, #1
	moveq	r1, #6
	movne	r1, #7
	cmp	r3, #256
	rsb	r1, r1, r2
	add	r2, r2, #2
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L870
	b	.L855
.L868:
	cmp	r0, #5
	bne	.L871
	ldr	r1, .L895
	mov	r3, r2
.L872:
	strh	r3, [r1, #2]!	@ movhi
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #16
	bne	.L872
	ldr	r2, .L895+16
.L873:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, #496
	bne	.L873
	b	.L855
.L871:
	cmp	r0, #6
	bne	.L874
	ldr	r0, .L895
	mov	r3, r2
.L877:
	uxth	r1, r3
	cmp	r1, #5
	bls	.L875
	tst	r3, #1
	moveq	r1, #10
	movne	r1, #12
	rsb	r1, r1, r2
	uxth	r1, r1
.L875:
	add	r3, r3, #1
	add	r2, r2, #3
	cmp	r3, #256
	strh	r1, [r0, #2]!	@ movhi
	uxth	r2, r2
	bne	.L877
	b	.L855
.L874:
	cmp	r0, #9
	bne	.L855
	ldr	r3, .L895+8
	movw	r1, #3076
	strh	r2, [r3, r1]	@ movhi
	movw	r2, #3078
	mov	r1, #1
	strh	r1, [r3, r2]	@ movhi
	mov	r1, #2
	add	r2, r2, r1
	strh	r1, [r3, r2]	@ movhi
	mov	r3, #3
	ldr	r2, .L895+20
	movw	r1, #509
.L878:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, r1
	bne	.L878
.L855:
	mov	r2, #1024
	ldr	r0, .L895+24
	mov	r1, #255
	uxth	r4, r4
	bl	ftl_memset
	ldr	r2, .L895
	mov	r3, #0
	ldr	r0, .L895+28
	b	.L879
.L880:
	ldrh	r1, [r2, #2]!
	add	r3, r3, #1
	uxth	r3, r3
	add	ip, r0, r1, asl #1
	sub	ip, ip, #880
	strh	r1, [ip, #0]	@ movhi
.L879:
	cmp	r3, r4
	bcc	.L880
	ldmfd	sp!, {r4, pc}
.L896:
	.align	2
.L895:
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
	b	.L899
.L900:
	ldr	r4, [r1, r3]
	add	ip, ip, #1
	str	r4, [r0, r3]
	add	r3, r3, #4
.L899:
	cmp	ip, r2
	bne	.L900
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
	ldr	r3, .L903
	ldr	r0, [r3, #0]
	b	jiffies_to_msecs
.L904:
	.align	2
.L903:
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
	ldr	r4, .L908
	mov	ip, r0
	mov	r2, r3
	ldr	r4, [r4, #144]
	add	r4, r4, #4096
	add	r1, r4, r1
	movne	r0, r1
	movne	r1, ip
	ldmfd	sp!, {r4, lr}
	b	memcpy
.L909:
	.align	2
.L908:
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
	ldr	r3, .L914
	ldr	r0, [r3, #3960]
	bx	lr
.L915:
	.align	2
.L914:
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
	b	.L920
.L926:
	cmp	r5, #0
	bne	.L921
	ldr	r0, .L928
	mov	r1, r8
	mov	r2, r6
	mov	r3, r4
	bl	printk
.L921:
	cmp	r7, #4
	ldreq	r0, .L928+4
	ldreq	r1, [r6, r4, asl #2]
	beq	.L927
	cmp	r7, #2
	moveq	r3, r4, asl #1
	ldreq	r0, .L928+4
	ldreqsh	r1, [r6, r3]
	ldrne	r0, .L928+4
	ldrneb	r1, [r6, r4]	@ zero_extendqisi2
.L927:
	add	r5, r5, #1
	bl	printk
	cmp	r5, #15
	bls	.L925
	ldr	r0, .L928+8
	mov	r5, #0
	ldr	r1, .L928+12
	bl	printk
.L925:
	add	r4, r4, #1
.L920:
	cmp	r4, sl
	bne	.L926
	ldr	r0, .L928+8
	ldr	r1, .L928+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	b	printk
.L929:
	.align	2
.L928:
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
	ldr	r6, .L953
	add	r0, r6, r0, asl #3
	ldr	r3, [r6, #3788]
	ldr	r4, [r0, #864]
	cmp	r3, #3
	bls	.L947
	ldr	r3, [r4, #16]
	tst	r3, #4
	beq	.L947
	mov	r0, r4
	bl	wait_for_nandc_xfer_completed
	ldr	r5, [r4, #16]
	ldr	r3, [r4, #8]
	ands	r5, r5, #2
	str	r3, [sp, #0]
	movne	r5, #0
	bne	.L951
	b	.L952
.L937:
	ldr	r3, [r6, #3788]
	cmp	r3, #5
	bls	.L934
	ldr	r3, [r4, #0]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L934
	ldr	r3, [sp, #4]
	tst	r3, #131072
	bne	.L935
.L934:
	add	r5, r5, #1
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L951
	ldr	r2, [r4, #28]
	mov	r1, r5
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L953+4
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L953+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L951:
	ldr	r2, [r4, #28]
	ldr	r3, [sp, #0]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	blt	.L937
.L935:
	ldr	r4, .L953
	ldr	r3, [r4, #3824]
	cmp	r3, #0
	beq	.L938
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
	b	.L938
.L941:
	ldr	r3, [r4, #8]
	add	r5, r5, #1
	str	r3, [sp, #0]
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L952
	ldr	r2, [sp, #0]
	mov	r1, r5
	ldr	r3, [r4, #28]
	ldr	r0, .L953+12
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L953+8
	mov	r1, r4
	mov	r2, #4
	mov	r3, #512
	bl	rknand_print_hex
.L952:
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L941
	ldr	r6, .L953
	ldr	r3, [r6, #3832]
	cmp	r3, #0
	beq	.L942
	mov	r0, r4
	bl	NandcSendDumpDataStart
.L942:
	ldr	r3, [r6, #3824]
	ldr	r5, .L953
	cmp	r3, #0
	beq	.L943
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
.L943:
	ldr	r3, .L953
	ldr	r3, [r3, #3832]
	cmp	r3, #0
	beq	.L938
	mov	r0, r4
	bl	NandcSendDumpDataDone
.L938:
	ldr	r3, .L953
	mov	r2, #0
	str	r2, [r3, #3824]
	b	.L930
.L947:
	ldr	r3, [r4, #8]
	str	r3, [sp, #0]
	ldr	r3, [sp, #0]
	tst	r3, #1048576
	beq	.L947
.L930:
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L954:
	.align	2
.L953:
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
	ldr	r3, .L993
	.pad #92
	sub	sp, sp, #92
	tst	r4, #63
	add	r3, r3, r0, asl #3
	mov	r7, r0
	mov	fp, r1
	mov	sl, r2
	ldr	r5, [sp, #128]
	ldr	r6, [r3, #864]
	bne	.L956
	cmp	r5, #0
	bne	.L957
	add	r0, sp, #24
	mov	r1, #255
	mov	r2, #64
	add	r5, sp, #24
	bl	ftl_memset
.L957:
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
	bne	.L958
	ldr	r1, .L993
	mov	r0, sl, lsr #1
	mov	r3, fp
	mov	r2, fp
	ldr	ip, [r1, #3836]
	mov	r4, r1
	cmp	ip, #24
	movhi	ip, #128
	movls	ip, #64
	b	.L960
.L961:
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
.L960:
	cmp	r2, r0
	add	r7, r3, ip
	ldr	r1, .L993
	bcc	.L961
	mov	r3, #0
	ldr	r0, [r1, #3836]
	mov	sl, sl, lsr #2
	ldr	r1, [r1, #3788]
	mov	r4, r3
	b	.L962
.L968:
	add	r2, r3, #8
	ldr	r2, [r6, r2, asl #2]
	str	r2, [sp, #20]
	ldr	r2, [sp, #20]
	tst	r2, #4
	bne	.L983
	ldr	r2, [sp, #20]
	ands	r2, r2, #32768
	bne	.L983
	cmp	r1, #5
	bls	.L964
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
	b	.L991
.L964:
	cmp	r1, #3
	bls	.L966
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
.L991:
	orr	r2, ip, r2, asl #5
.L966:
	cmp	r4, r2
	movcc	r4, r2
	b	.L963
.L983:
	mvn	r4, #0
.L963:
	add	r3, r3, #1
.L962:
	cmp	r3, sl
	bcs	.L958
	cmp	r0, #0
	bne	.L968
.L958:
	mov	r3, #0
	str	r3, [r6, #16]
	b	.L969
.L956:
	cmp	r1, #1
	mov	r8, #0
	bne	.L989
	b	.L970
.L973:
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
.L970:
	cmp	r8, sl
	bcc	.L973
	mov	r4, #0
	b	.L969
.L989:
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
	b	.L992
.L978:
	mov	r0, r7
	bl	NandcXferComp
	ldr	r3, [fp, #32]
	add	ip, r5, #2
	cmp	ip, sl
	str	r3, [sp, #20]
	bcs	.L975
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
.L975:
	ldr	r3, [sp, #20]
	tst	r3, #4
	mvnne	r4, #0
	bne	.L976
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #20]
	ubfx	r2, r2, #3, #5
	ubfx	r3, r3, #27, #1
	orr	r3, r2, r3, asl #5
	cmp	r4, r3
	movcc	r4, r3
.L976:
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
.L992:
	cmp	ip, sl
	mov	r5, ip
	bcc	.L978
	mov	r6, fp
	ldr	fp, [sp, #12]
.L969:
	ldr	r3, .L993
	rsbs	fp, fp, #1
	movcc	fp, #0
	ldr	r3, [r3, #3788]
	cmp	r3, #5
	movls	fp, #0
	cmp	fp, #0
	beq	.L979
	ldr	r3, [r6, #0]
	and	r2, r3, #139264
	cmp	r2, #139264
	orreq	r3, r3, #131072
	streq	r3, [r6, #0]
	mvneq	r4, #0
.L979:
	mov	r0, r4
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L994:
	.align	2
.L993:
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
	ldr	r3, .L998
	subs	r4, r0, #0
	mov	r5, r1
	mov	r6, r2
	ldrb	r7, [r3, #3125]	@ zero_extendqisi2
	bne	.L996
	ldr	r3, .L998+4
	ldrb	r2, [r3, #853]	@ zero_extendqisi2
	ldr	r1, [r3, #856]
	mul	r2, r1, r2
	cmp	r5, r2
	bcs	.L996
	ldrb	r3, [r3, #852]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r7, r7, #2
	movne	r7, #4
.L996:
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
.L999:
	.align	2
.L998:
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
	ldr	r3, .L1010
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
	mov	r5, r2
	ldr	r2, [r3, #3624]
	mov	r9, r0
	mov	sl, r1
	ldrb	r4, [r2, #19]	@ zero_extendqisi2
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1001
	ldr	r3, [r3, #3776]
	cmp	r3, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1001:
	sub	r3, r4, #5
	uxtb	r3, r3
	cmp	r4, #68
	cmpne	r3, #3
	bls	.L1004
	cmp	r4, #35
	cmpne	r4, #19
	beq	.L1004
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1007:
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
	b	.L1008
.L1004:
	ldr	r6, .L1010+4
	ldr	r7, .L1010
	mov	r8, r6
.L1008:
	ldr	r3, [r7, #3624]
	ldrh	r3, [r3, #10]
	cmp	r3, r5
	bhi	.L1007
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1011:
	.align	2
.L1010:
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
	ldr	r3, .L1015
	subs	r4, r0, #0
	mov	r6, r1
	mov	r5, r2
	ldrb	r7, [r3, #3125]	@ zero_extendqisi2
	bne	.L1013
	ldr	r3, .L1015+4
	ldrb	r2, [r3, #853]	@ zero_extendqisi2
	ldr	r3, [r3, #856]
	mul	r3, r3, r2
	cmp	r1, r3
	movcc	r7, #4
.L1013:
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
.L1016:
	.align	2
.L1015:
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
	ldr	r3, .L1031
	mov	sl, r2
	ldr	fp, .L1031
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
	b	.L1019
.L1024:
	add	r4, r4, #1
	ldrb	r1, [fp, #1]	@ zero_extendqisi2
	mov	r0, r6
	ldr	r2, .L1031+4
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
	beq	.L1021
	ldrb	r3, [ip, #3840]	@ zero_extendqisi2
	cmn	r5, #1
	moveq	r5, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1030
	mov	r7, #0
	mov	sl, r7
.L1021:
	add	r8, r8, #1
.L1019:
	cmp	r8, r9
	bcc	.L1024
	b	.L1023
.L1030:
	mov	r5, r0
.L1023:
	ldr	r3, .L1031
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
.L1032:
	.align	2
.L1031:
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
	ldr	r3, .L1051
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
	beq	.L1050
	mov	r1, #3
	bl	__aeabi_idiv
	uxtb	r0, r0
.L1050:
.L1035:
	mov	r2, #0
	ldr	r3, .L1051
	mvn	r4, #0
	mov	r6, r2
	add	r3, r3, r9, asl #3
	str	r0, [sp, #16]
	str	r2, [sp, #12]
	str	r3, [sp, #28]
.L1045:
	mov	r0, r9
	mov	fp, #0
	bl	NandcWaitFlashReady
	ldr	r2, [sp, #28]
	ldrb	r7, [r2, #868]	@ zero_extendqisi2
	ldr	r8, [r2, #864]
	add	r7, r7, #8
	add	r5, r8, r7, asl #8
	b	.L1036
.L1040:
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
	beq	.L1037
	cmn	r4, #1
	ldr	r2, [sp, #16]
	moveq	r4, r0
	cmp	r0, r2
	bcc	.L1047
	mov	sl, #0
	str	sl, [sp, #20]
.L1037:
	mov	fp, ip
.L1036:
	ldr	r2, .L1051+4
	ldrb	r3, [r2, #152]	@ zero_extendqisi2
	cmp	fp, r3
	bcc	.L1040
	b	.L1039
.L1047:
	mov	r4, r0
	mov	sl, #0
	str	sl, [sp, #20]
.L1039:
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
	bne	.L1042
	mov	r3, fp
	ldr	r0, .L1051+8
	mov	r1, fp
	ldr	r2, [sp, #24]
	str	r4, [sp, #0]
	bl	printk
	ldr	r3, [sp, #12]
	cmp	r3, #0
	bne	.L1043
	ldr	r2, .L1051
	ldrb	r3, [r2, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r5, #0
	andne	r5, r5, #1
	cmp	r5, #0
	beq	.L1044
	mov	r0, r9
	mov	r1, #3
	bl	micron_auto_read_calibration_config
	mov	r3, #1
	str	r3, [sp, #12]
	b	.L1045
.L1042:
	ldr	r2, [sp, #12]
	cmp	r2, #0
	beq	.L1044
.L1043:
	mov	r0, r9
	mov	r1, #0
	bl	micron_auto_read_calibration_config
	cmn	r4, #1
	movne	r4, #256
.L1044:
	mov	r0, r4
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1052:
	.align	2
.L1051:
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
	ldr	ip, .L1062
	mov	r5, #1
	mvn	r4, #0
	ldr	fp, .L1062+4
	add	r3, ip, r8, asl #3
	ldrb	sl, [r3, #868]	@ zero_extendqisi2
	ldr	r2, [r3, #864]
	add	sl, sl, #8
	add	sl, r2, sl, asl #8
	b	.L1054
.L1058:
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
	beq	.L1055
	ldrb	r3, [ip, #3840]	@ zero_extendqisi2
	cmn	r4, #1
	moveq	r4, r0
	add	r3, r3, r3, asl #1
	cmp	r0, r3, lsr #2
	bcc	.L1061
	mov	r6, #0
	mov	r7, r6
.L1055:
	add	r5, r5, #1
.L1054:
	ldrb	r3, [fp, #152]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r5, r3
	bcc	.L1058
	b	.L1057
.L1061:
	mov	r4, r0
.L1057:
	mov	r0, sl
	mov	r1, #0
	bl	SamsungSetRRPara
	ldr	r3, .L1062
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
.L1063:
	.align	2
.L1062:
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
	ldr	r3, .L1085
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
	bls	.L1065
	ldrb	sl, [r3, #3780]	@ zero_extendqisi2
	cmp	sl, #0
	beq	.L1066
	mov	r0, #0
	mov	sl, #1
	bl	NandcSetDdrMode
.L1066:
	mov	r3, #92
	str	r3, [r4, #8]
	mov	r3, #197
	str	r3, [r4, #8]
.L1065:
	mov	r6, #1
	mvn	r5, #0
	ldr	fp, .L1085
	b	.L1067
.L1076:
	ldrb	r3, [fp, #3761]	@ zero_extendqisi2
	mov	r0, r4
	uxtb	r1, r6
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1068
	bl	SandiskSetRRPara
	b	.L1069
.L1068:
	bl	ToshibaSetRRPara
.L1069:
	ldr	r2, .L1085
	ldrb	r3, [r2, #3761]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L1070
	ldr	r2, .L1085+4
	ldrb	r3, [r2, #152]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r6, r3
	moveq	r3, #179
	streq	r3, [r4, #8]
.L1070:
	cmp	sl, #0
	mov	r3, #38
	str	r3, [r4, #8]
	mov	r3, #93
	str	r3, [r4, #8]
	beq	.L1071
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
	b	.L1072
.L1071:
	mov	r3, r7
	mov	r0, r8
	ldr	r1, [sp, #4]
	mov	r2, r9
	bl	FlashReadRawPage
	mov	r3, r0
.L1072:
	cmn	r3, #1
	beq	.L1073
	ldrb	r2, [fp, #3840]	@ zero_extendqisi2
	cmn	r5, #1
	moveq	r5, r3
	add	r2, r2, r2, asl #1
	cmp	r3, r2, lsr #2
	bcc	.L1084
	mov	r7, #0
	mov	r9, r7
.L1073:
	add	r6, r6, #1
.L1067:
	ldr	r2, .L1085+4
	ldrb	r3, [r2, #152]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r6, r3
	bcc	.L1076
	b	.L1075
.L1084:
	mov	r5, r3
.L1075:
	ldr	r6, .L1085
	mov	r0, r4
	mov	r1, #0
	ldrb	r3, [r6, #3761]	@ zero_extendqisi2
	sub	r3, r3, #67
	uxtb	r3, r3
	cmp	r3, #1
	bhi	.L1077
	bl	SandiskSetRRPara
	b	.L1078
.L1077:
	bl	ToshibaSetRRPara
.L1078:
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
	beq	.L1080
	mov	r0, #4
	bl	NandcSetDdrMode
.L1080:
	mov	r0, r5
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1086:
	.align	2
.L1085:
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
	ldr	r5, .L1094
	ldr	r7, .L1094+4
	ldr	r8, .L1094+8
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
	ldr	r1, .L1094+12
	mov	r2, #8
	bl	memcpy
	add	r0, r4, #96
	ldr	r1, .L1094+16
	mov	r2, #32
	bl	memcpy
	ldr	r0, [r5, #156]
	ldr	r1, .L1094+20
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
.L1090:
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
	beq	.L1088
	ldr	sl, [r5, #156]
	ldr	r3, [sl, #0]
	cmp	r3, r8
	bne	.L1088
	add	r0, sl, #12
	movw	r1, #2036
	bl	js_hash
	ldr	r3, [sl, #8]
	cmp	r3, r0
	bne	.L1088
	add	r3, r4, #1
	str	r3, [r5, #168]
	ldr	r3, [r7, #856]
	add	r6, r6, #1
	cmp	r6, #1
	mul	r3, r3, r4
	str	r3, [r5, #172]
	bhi	.L1089
.L1088:
	add	r4, r4, #1
	cmp	r4, #4
	bne	.L1090
.L1089:
	mov	r0, #0
	bl	flash_exit_slc_mode
	cmp	r6, #0
	mvneq	r0, #0
	movne	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1095:
	.align	2
.L1094:
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
	ldr	r1, .L1107
	add	r0, sp, #12
	bl	memcpy
	ldr	r3, .L1107+4
	ldrb	r2, [r3, #3840]	@ zero_extendqisi2
	ldr	r3, [r3, #3776]
	cmp	r3, #0
	str	r2, [sp, #4]
	beq	.L1097
	mov	r0, #0
	bl	flash_enter_slc_mode
.L1097:
	ldr	r7, .L1107+4
	mov	r0, sl
	mov	r1, #0
	mov	r2, #2048
	bl	ftl_memset
	mvn	r8, #0
	mov	r4, #2
	ldr	r9, .L1107+8
	mov	fp, r7
	b	.L1098
.L1105:
	mov	r6, #0
.L1100:
	add	r3, sp, #12
	ldr	r5, .L1107+8
	ldrb	r0, [r3, r6]	@ zero_extendqisi2
	bl	FlashBchSel
	ldr	r1, [r7, #856]
	mov	r0, #0
	ldr	r2, [r9, #160]
	mov	r3, r0
	mul	r1, r1, r4
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1099
	add	r6, r6, #1
	cmp	r6, #4
	bne	.L1100
	b	.L1101
.L1099:
	ldr	r3, [r5, #160]
	ldr	r2, [r3, #0]
	ldr	r3, .L1107+12
	cmp	r2, r3
	bne	.L1101
	add	r3, sp, #16
	ldr	r0, .L1107+16
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
	bls	.L1104
	str	r4, [r5, #168]
	bl	FlashSavePhyInfo
	mov	r8, #0
.L1101:
	add	r4, r4, #1
.L1098:
	ldrb	r3, [r7, #853]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1105
	b	.L1102
.L1104:
	mov	r8, #0
.L1102:
	ldr	r0, [sp, #4]
	bl	FlashBchSel
	ldr	r3, .L1107+4
	ldr	r3, [r3, #3776]
	cmp	r3, #0
	beq	.L1103
	mov	r0, #0
	bl	flash_exit_slc_mode
.L1103:
	mov	r0, r8
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1108:
	.align	2
.L1107:
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
	ldr	r1, .L1120
	add	r0, sp, #4
	bl	memcpy
	ldr	r4, .L1120+4
	ldr	r3, .L1120+8
	mov	r5, #0
	mov	r0, r5
	mov	r8, #4
	str	r5, [r4, #172]
	mvn	r6, #0
	ldr	r9, [r3, #856]
	ldr	r3, [r4, #160]
	ldr	fp, .L1120+12
	str	r3, [r4, #156]
	bl	flash_enter_slc_mode
	b	.L1110
.L1112:
	add	r3, sp, #4
	ldrb	r0, [r3, r7]	@ zero_extendqisi2
	bl	FlashBchSel
	mov	r0, #0
	mov	r1, r5
	ldr	r2, [r4, #160]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1111
	mov	r0, #0
	mov	r1, sl
	ldr	r2, [r4, #160]
	mov	r3, r0
	bl	FlashReadRawPage
	cmn	r0, #1
	bne	.L1111
	add	r7, r7, #1
	cmp	r7, #4
	beq	.L1113
	b	.L1112
.L1111:
	ldr	r7, [r4, #156]
	ldr	r3, [r7, #0]
	cmp	r3, fp
	bne	.L1113
	cmp	r6, #0
	bne	.L1114
	ldr	r3, .L1120+8
	mov	r0, r5
	ldr	r1, [r3, #856]
	bl	__aeabi_uidiv
	ldr	r3, .L1120+4
	add	r0, r0, #1
	str	r0, [r3, #168]
	mov	r0, r6
	b	.L1119
.L1114:
	add	r0, r7, #12
	movw	r1, #2036
	bl	js_hash
	ldr	r3, [r7, #8]
	cmp	r3, r0
	bne	.L1113
	ldr	r6, .L1120+8
	add	r1, r7, #160
	mov	r2, #32
	ldr	r0, .L1120+16
	bl	memcpy
	add	r1, r7, #192
	add	r0, r6, #896
	mov	r2, #32
	bl	memcpy
	add	r1, r7, #224
	mov	r0, r6
	mov	r2, #852
	bl	memcpy
	ldr	r2, .L1120+20
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
.L1113:
	subs	r8, r8, #1
	add	r5, r5, r9
	beq	.L1117
.L1110:
	add	sl, r5, #1
	mov	r7, #0
	b	.L1112
.L1117:
	mov	r0, r8
.L1119:
	bl	flash_exit_slc_mode
	mov	r0, r6
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1121:
	.align	2
.L1120:
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
	ldr	r4, .L1143
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
	beq	.L1124
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
	beq	.L1125
	ldr	r0, .L1143+4
	mov	r2, r8
	ldr	r1, [sp, #0]
	bl	printk
	cmp	r8, #9
	bhi	.L1126
	add	r4, r4, fp, asl #3
	ldr	r3, [r4, #864]
	ldr	r2, [r3, #3840]
	ldr	r2, [r3, #0]
	orr	r2, r2, #131072
	str	r2, [r3, #0]
.L1126:
	ldr	r3, .L1143+8
	ldr	r2, [r3, #180]
	add	r2, r2, #1
	str	r2, [r3, #180]
	cmp	r2, #2048
	movcs	sl, #0
	strcs	sl, [r3, #180]
	movcs	r9, sl
	bcc	.L1125
.L1124:
	mov	r4, #0
	str	fp, [sp, #4]
	mov	r5, r4
	mvn	r6, #0
	mov	fp, r4
	str	r4, [sp, #8]
.L1131:
	uxtb	r0, r7
	bl	NandcSetDdrPara
	mov	r3, sl
	ldr	r0, [sp, #4]
	mov	r2, r9
	ldr	r1, [sp, #0]
	bl	FlashReadRawPage
	add	r3, r8, #1
	cmp	r0, r3
	bhi	.L1127
	cmp	r0, #2
	bhi	.L1137
	add	r5, r5, #1
	cmp	r5, #9
	bls	.L1137
	mov	ip, fp
	rsb	r4, r5, r7
	ldr	fp, [sp, #4]
	mov	r8, r0
	mov	r6, #0
	b	.L1129
.L1127:
	cmp	fp, r5
	bcs	.L1138
	cmp	r5, #7
	rsb	r3, r5, r4
	str	r3, [sp, #8]
	bhi	.L1142
	mov	fp, r5
	b	.L1138
.L1137:
	mov	r6, #0
	mov	r4, r7
	mov	r8, r0
	mov	sl, r6
	mov	r9, r6
	b	.L1128
.L1138:
	mov	r5, #0
.L1128:
	add	r7, r7, #2
	cmp	r7, #69
	bls	.L1131
	mov	ip, fp
	ldr	fp, [sp, #4]
.L1129:
	cmp	ip, r5
	bcc	.L1132
	b	.L1130
.L1142:
	ldr	fp, [sp, #4]
.L1130:
	ldr	r4, [sp, #8]
.L1132:
	cmp	r4, #0
	beq	.L1133
	ldr	r0, .L1143+12
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	bl	NandcSetDdrPara
.L1133:
	cmn	r6, #1
	bne	.L1125
	ldr	r0, .L1143+16
	mov	r1, fp
	ldr	r2, [sp, #0]
	bl	printk
	ldr	r3, [sp, #56]
	cmp	r3, #0
	moveq	r8, r6
	beq	.L1125
	ldr	r3, [sp, #12]
	ubfx	r0, r3, #8, #8
	bl	NandcSetDdrPara
.L1125:
	mov	r0, r8
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1144:
	.align	2
.L1143:
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
	ldr	r5, .L1149
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
	beq	.L1146
	cmn	r8, #1
	bne	.L1147
.L1146:
	ldrb	r3, [r5, #3772]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L1147
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r3, .L1149
	mov	r2, #0
	strb	r2, [r3, #3780]
	b	.L1148
.L1147:
	mov	r3, #1
	strb	r3, [r5, #3780]
.L1148:
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L1150:
	.align	2
.L1149:
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
	bne	.L1152
	ldr	r7, .L1157
	ldrb	r9, [r7, #860]	@ zero_extendqisi2
	cmp	r9, #0
	beq	.L1153
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
	bne	.L1152
.L1153:
	ldr	r7, .L1157
	ldrb	r3, [r7, #3780]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1152
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
	beq	.L1154
	ldrb	r3, [r7, #3840]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1152
.L1154:
	ubfx	r0, r9, #8, #8
	bl	NandcSetDdrPara
.L1152:
	ldr	r7, .L1157+4
	ldr	ip, [r7, #184]
	adds	r3, ip, #0
	movne	r3, #1
	cmn	r4, #1
	movne	r3, #0
	cmp	r3, #0
	beq	.L1155
	mov	r1, r6
	mov	r2, r8
	mov	r3, sl
	mov	r0, r5
	blx	ip
	mov	r2, r5
	mov	r3, r6
	mov	r4, r0
	ldr	r0, .L1157+8
	mov	r1, r4
	bl	printk
	cmn	r4, #1
	bne	.L1155
	ldr	r3, .L1157
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1155
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
.L1155:
	mov	r0, r4
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L1158:
	.align	2
.L1157:
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
	ldr	r3, .L1172
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	r8, r1
	.pad #28
	sub	sp, sp, #28
	mov	fp, r2
	ldrb	sl, [r3, #3125]	@ zero_extendqisi2
	mov	r4, r0
	mov	r7, #0
	ldr	r6, .L1172+4
	b	.L1160
.L1169:
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
	bcs	.L1162
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
	beq	.L1167
	ldr	r2, [r3, #12]
	cmn	r2, #1
	bne	.L1167
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L1167
	ldr	r3, [r3, #0]
	cmn	r3, #1
	strne	r2, [r4, #0]
.L1167:
	ldr	r3, [r4, #0]
	cmn	r3, #1
	bne	.L1162
	ldr	r1, [r4, #4]
	ldr	r0, .L1172+8
	ldrb	r2, [r6, #3840]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r4, #8]
	cmp	r1, #0
	beq	.L1168
	ldr	r0, .L1172+12
	mov	r2, #4
	mov	r3, #8
	bl	rknand_print_hex
.L1168:
	ldr	r1, [r4, #12]
	cmp	r1, #0
	beq	.L1162
	mov	r2, #4
	ldr	r0, .L1172+16
	mov	r3, r2
	bl	rknand_print_hex
.L1162:
	add	r7, r7, #1
	add	r4, r4, #36
.L1160:
	cmp	r7, r8
	bne	.L1169
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1173:
	.align	2
.L1172:
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
	ldr	r3, .L1216
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.pad #52
	sub	sp, sp, #52
	ldrb	r3, [r3, #3125]	@ zero_extendqisi2
	mov	r9, r0
	str	r1, [sp, #28]
	str	r2, [sp, #32]
	str	r3, [sp, #24]
	ldr	r3, .L1216+4
	ldrb	r8, [r3, #852]	@ zero_extendqisi2
	ldrb	ip, [r3, #860]	@ zero_extendqisi2
	cmp	r8, #0
	str	ip, [sp, #36]
	moveq	sl, r8
	beq	.L1175
	bl	FlashReadSlc2KPages
	b	.L1176
.L1203:
	mov	r3, #36
	ldr	ip, [sp, #28]
	mul	r3, r3, sl
	add	r2, sp, #44
	ldr	r1, [sp, #32]
	ldr	fp, .L1216+4
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
	bcs	.L1178
	add	r3, fp, r3
	ldrb	r4, [r3, #3764]	@ zero_extendqisi2
	ldr	r3, .L1216+8
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
	bhi	.L1180
	sub	r2, r2, #7
	add	r1, fp, r4
	add	fp, fp, r4
	uxtb	r2, r2
	ldrb	r3, [r1, #12]	@ zero_extendqisi2
	cmp	r2, #1
	ldr	r2, .L1216+4
	ldrlsb	r3, [r1, #20]	@ zero_extendqisi2
	ldrb	r1, [fp, #3756]	@ zero_extendqisi2
	cmp	r1, r3
	beq	.L1180
	ldrb	r1, [r2, #1]	@ zero_extendqisi2
	mov	r0, r4
	add	r2, r2, #4
	bl	HynixSetRRPara
.L1180:
	mov	r0, r4
	mov	r5, r5, lsr #31
	bl	NandcFlashCs
	ldr	ip, [sp, #32]
	mov	r0, r4
	cmp	ip, #1
	orreq	r5, r5, #1
	str	r5, [sp, #20]
	cmp	r5, #0
	beq	.L1182
	ldr	r3, .L1216+4
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1182
	bl	flash_enter_slc_mode
	b	.L1183
.L1182:
	bl	flash_exit_slc_mode
.L1183:
	ldr	fp, .L1216+4
.L1215:
	ldr	r1, [sp, #44]
	cmn	r1, #1
	cmpeq	r4, #255
	moveq	r3, #0
	movne	r3, #1
	moveq	r7, r3
	beq	.L1185
	cmp	r7, #0
	beq	.L1186
	ldr	r2, [fp, #856]
	mov	r0, r4
	add	r2, r1, r2
	bl	FlashReadDpCmd
	b	.L1187
.L1186:
	mov	r0, r4
	bl	FlashReadCmd
.L1187:
	mov	r0, r4
	bl	NandcWaitFlashReady
	cmp	r7, #0
	beq	.L1185
	mov	r0, r4
	ldr	r1, [sp, #44]
	bl	FlashReadDpDataOutCmd
.L1185:
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
	bne	.L1215
.L1188:
	cmp	r7, #0
	mov	r5, r0
	beq	.L1189
	ldr	r3, .L1216+4
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
.L1189:
	mov	r0, r4
	ldr	fp, .L1216+4
	bl	NandcFlashDeCs
	ldr	ip, [sp, #36]
	cmn	r5, #1
	strb	ip, [fp, #860]
	bne	.L1193
	ldrb	r3, [fp, #3780]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1191
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
	beq	.L1192
	ldrb	r3, [fp, #3840]	@ zero_extendqisi2
	cmp	r0, r3, lsr #1
	bls	.L1209
.L1192:
	ubfx	r0, r7, #8, #8
	bl	NandcSetDdrPara
	cmn	r5, #1
	bne	.L1209
.L1191:
	ldr	r3, .L1216+8
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r6, #8]
	ldr	r7, [r3, #184]
	ldr	r3, [r6, #12]
	cmp	r7, #0
	beq	.L1194
	blx	r7
	cmn	r0, #1
	mov	r5, r0
	bne	.L1211
	ldr	r2, .L1216+4
	ldr	r3, [r2, #3624]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #7
	bhi	.L1196
	ldrb	r1, [r2, #1]	@ zero_extendqisi2
	mov	r0, r4
	add	r2, r2, #4
	mov	r3, #0
	bl	HynixSetRRPara
.L1196:
	ldr	r1, [sp, #44]
	mov	r0, r4
	ldr	r2, [r6, #8]
	ldr	r3, [r6, #12]
	bl	FlashReadRawPage
	ldr	r7, .L1216+4
	ldr	r1, [r6, #4]
	ldrb	r2, [r7, #3840]	@ zero_extendqisi2
	mov	r5, r0
	ldr	r0, .L1216+12
	mov	r3, r5
	bl	printk
	cmn	r5, #1
	bne	.L1211
	ldrb	r7, [r7, #3072]	@ zero_extendqisi2
	cmp	r7, #0
	beq	.L1195
	ldr	ip, [sp, #20]
	mov	r0, r4
	cmp	ip, #0
	beq	.L1197
	bl	flash_enter_slc_mode
	b	.L1198
.L1197:
	bl	flash_exit_slc_mode
.L1198:
	ldr	r3, .L1216+8
	mov	r0, r4
	ldr	r1, [sp, #44]
	ldr	r2, [r6, #8]
	ldr	ip, [r3, #184]
	ldr	r3, [r6, #12]
	blx	ip
	mov	r5, r0
	b	.L1211
.L1194:
	bl	FlashReadRawPage
	mov	r5, r0
	b	.L1195
.L1209:
	mov	r7, #0
.L1193:
	ldr	r3, .L1216+4
	ldrb	r3, [r3, #3840]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r5, r3, lsr #2
	bls	.L1195
	ldr	r3, .L1216+8
	ldr	r3, [r3, #184]
	cmp	r3, #0
	moveq	r5, #256
	b	.L1195
.L1211:
	mov	r7, #0
.L1195:
	ldr	ip, [sp, #16]
	cmp	r5, #256
	cmnne	r5, #1
	movne	r2, #0
	str	r5, [r9, ip]
	strne	r2, [r9, ip]
	ldr	ip, [sp, #16]
	ldr	r3, [r9, ip]
	cmn	r3, #1
	bne	.L1200
	ldr	r2, .L1216+4
	ldr	r1, [r6, #4]
	ldr	r0, .L1216+16
	ldrb	r2, [r2, #3840]	@ zero_extendqisi2
	bl	printk
	ldr	r1, [r6, #12]
	cmp	r1, #0
	beq	.L1200
	mov	r2, #4
	ldr	r0, .L1216+20
	mov	r3, r2
	bl	rknand_print_hex
.L1200:
	cmp	r7, #0
	beq	.L1201
	ldr	r3, .L1216+4
	ldrb	r3, [r3, #3840]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r8, r3, lsr #2
	bls	.L1202
	ldr	r3, .L1216+8
	ldr	r3, [r3, #184]
	cmp	r3, #0
	moveq	r8, #256
.L1202:
	add	r3, sl, #1
	mov	r2, #36
	cmp	r8, #256
	cmnne	r8, #1
	mul	r3, r2, r3
	movne	r2, #0
	str	r8, [r9, r3]
	strne	r2, [r9, r3]
.L1201:
	ldr	ip, [sp, #20]
	add	sl, sl, r7
	cmp	ip, #0
	beq	.L1178
	ldr	r3, .L1216+4
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1178
	mov	r0, r4
	bl	flash_exit_slc_mode
.L1178:
	add	sl, sl, #1
.L1175:
	ldr	ip, [sp, #28]
	cmp	sl, ip
	bcc	.L1203
	mov	r0, #0
.L1176:
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1217:
	.align	2
.L1216:
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
	ldr	r0, .L1228
	mov	r5, #0
	ldr	r1, .L1228+4
	bl	printk
	ldr	r7, .L1228+8
	b	.L1219
.L1227:
	add	r4, sp, #80
	mov	r0, r5
	ldr	r6, .L1228+12
	mov	r9, #36
	strh	r5, [r4, #-48]!	@ movhi
	bl	ftl_get_blk_mode
	mov	r2, r5, asl #1
	ldr	ip, [r6, #-2012]
	mov	r1, r5
	ldrh	r2, [ip, r2]
	mov	r3, r0
	ldr	r0, .L1228+16
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
	b	.L1220
.L1222:
	ldrh	r1, [r0, #2]!
	movw	r2, #65535
	cmp	r1, r2
	beq	.L1221
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
.L1221:
	add	r3, r3, #1
	uxth	r3, r3
.L1220:
	cmp	r3, sl
	bne	.L1222
	ldr	sl, .L1228+12
	mov	r1, r4
	mov	r2, #0
	mov	r6, #0
	mov	r8, r6
	ldr	r0, [sl, #-960]
	bl	FlashReadPages
	b	.L1223
.L1224:
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
	ldr	r0, .L1228+20
	ldr	r2, [r2, r6]
	add	r6, r6, #36
	ldr	r3, [r3, #4]
	bl	printk
.L1223:
	cmp	r8, r4
	ldr	r9, .L1228+12
	bne	.L1224
	ldr	r0, [r9, #-960]
	mov	r1, r4
	mov	r2, #1
	mov	r6, #0
	bl	FlashReadPages
	mov	r8, r6
	mov	sl, r9
	b	.L1225
.L1226:
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
	ldr	r0, .L1228+24
	ldr	r2, [r2, r6]
	add	r6, r6, #36
	ldr	r3, [r3, #4]
	bl	printk
.L1225:
	cmp	r8, r4
	bne	.L1226
	add	r5, r5, #1
	uxth	r5, r5
.L1219:
	movw	r3, #3862
	ldrh	r3, [r7, r3]
	cmp	r3, r5
	bhi	.L1227
	mov	r0, #0
	add	sp, sp, #84
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1229:
	.align	2
.L1228:
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
	ldr	r5, .L1241
	ldr	r1, .L1241+4
	movw	r6, #3918
	ldrh	r9, [r5, r6]
	mov	r4, r0
	ldr	r0, .L1241+8
	bl	printk
	ldr	r3, .L1241+12
	mov	r1, r4
	ldr	r0, .L1241+16
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
	beq	.L1231
	mov	r0, r4
	bl	ftl_get_blk_mode
	cmp	r0, #1
	mov	r7, r0
	moveq	r3, #3920
	ldreqh	r9, [r5, r3]
.L1231:
	ldrh	r3, [r5, r6]
	mov	r1, r7
	ldr	r0, .L1241+20
	mov	r2, r9
	bl	printk
	mov	r6, #0
.L1232:
	ldr	r3, .L1241
	movw	r2, #3852
	movw	r1, #3928
	mov	r4, #0
	add	r0, sp, #46
	ldrh	sl, [r3, r2]
	ldr	r2, .L1241+12
	ldrh	lr, [r3, r1]
	ldr	r8, [r2, #-960]
	ldr	r5, [r2, #-1164]
	ldr	ip, [r2, #-1160]
	movw	r2, #3930
	ldrh	r2, [r3, r2]
	mov	r3, r4
	str	r2, [sp, #28]
	b	.L1233
.L1235:
	ldrh	r1, [r0, #2]!
	movw	r2, #65535
	cmp	r1, r2
	beq	.L1234
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
.L1234:
	add	r3, r3, #1
	uxth	r3, r3
.L1233:
	cmp	r3, sl
	bne	.L1235
	ldr	r8, .L1241+12
	mov	r1, r4
	mov	r2, r7
	mov	r5, #0
	mov	sl, r5
	ldr	r0, [r8, #-960]
	bl	FlashReadPages
	b	.L1236
.L1237:
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
	ldr	r0, .L1241+24
	ldr	r2, [r2, r5]
	add	r5, r5, #36
	ldr	r3, [r3, #4]
	bl	printk
.L1236:
	cmp	sl, r4
	bne	.L1237
	add	r6, r6, #1
	uxth	r6, r6
	cmp	r6, r9
	bne	.L1232
.L1238:
	mov	r0, #0
	add	sp, sp, #84
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1242:
	.align	2
.L1241:
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
	ldr	r5, .L1301
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L1301+4
	mov	r1, r6
	ldr	r3, .L1301+8
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
	ldr	r0, .L1301+12
	mov	r1, #255
	mov	r2, #16
	bl	ftl_memset
	movw	r3, #3860
	ldrh	r3, [r5, r3]
	str	r3, [sp, #8]
	b	.L1244
.L1246:
	add	r3, r6, r5
	ldr	r1, [sp, #8]
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #0]
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #0]
	cmp	r0, #0
	bne	.L1245
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
.L1245:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L1279
.L1292:
	mov	r7, #0
	movw	fp, #3852
	mov	r5, r7
	mov	r9, #36
	movw	r8, #3930
.L1279:
	ldrh	r3, [r6, fp]
	cmp	r3, r5
	bhi	.L1246
	cmp	r7, #0
	beq	.L1247
	ldr	r0, [r4, #-960]
	mov	r1, r7
	mov	r2, #1
	mov	fp, r7
	bl	FlashReadPages
	mov	r8, #0
	str	r8, [sp, #12]
.L1278:
	ldr	r3, [r4, #-960]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	ldr	r6, [r2, #4]
	cmn	r3, #1
	ldr	r5, [r2, #12]
	ubfx	r6, r6, #10, #16
	bne	.L1248
	mov	r7, #16
.L1250:
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
	beq	.L1248
.L1249:
	ldr	r3, [r4, #-960]
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L1248
	sub	r7, r7, #1
	uxth	r7, r7
	cmp	r7, #0
	bne	.L1250
.L1248:
	ldr	r3, [r4, #-960]
	ldr	r3, [r3, r8]
	cmn	r3, #1
	beq	.L1251
	ldr	r2, [r4, #-1800]
	ldr	r3, [r5, #4]
	cmn	r2, #1
	beq	.L1252
	cmp	r2, r3
	bhi	.L1253
.L1252:
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [sl, #-1800]
.L1253:
	ldrh	r3, [r5, #0]
	movw	r2, #61604
	cmp	r3, r2
	beq	.L1256
	bhi	.L1259
	movw	r2, #61574
	cmp	r3, r2
	bne	.L1254
	b	.L1299
.L1259:
	movw	r2, #61634
	cmp	r3, r2
	beq	.L1257
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1254
	b	.L1300
.L1257:
	ldr	r3, .L1301
	mov	r9, r6
	ldr	r1, .L1301+8
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
	b	.L1260
.L1266:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L1261
	ldr	ip, [ip, #0]
	mov	r6, r9
	cmp	ip, #0
	bne	.L1262
	cmp	r1, r0
	addne	r1, r1, #1
	ldrne	r0, .L1301+8
	strneh	r1, [r4, r0]	@ movhi
.L1262:
	mov	r0, #0
	sxth	r9, r3
	str	r3, [sp, #4]
	b	.L1263
.L1264:
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
.L1263:
	sxth	r1, r0
	cmp	r1, r9
	bne	.L1264
	ldr	r1, [sl, #-1024]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	ldr	r3, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, [sl, #-1028]
	strh	r6, [r1, r2]	@ movhi
	b	.L1265
.L1261:
	sub	r3, r3, #1
	uxth	r3, r3
.L1260:
	ldr	r6, [sp, #4]
	sxth	r2, r3
	cmp	r2, r6
	bgt	.L1266
	mov	r6, r9
.L1265:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L1254
	ldr	r2, .L1301+8
	ldr	r0, .L1301
	ldrh	r1, [r4, r2]
	ldr	r0, [r0, #3948]
	rsb	r0, r1, r0
	sub	r0, r0, #1
	sxth	r0, r0
	cmp	r3, r0
	bgt	.L1254
	add	r1, r1, #1
	strh	r1, [r4, r2]	@ movhi
	ldr	r2, [r4, #-1024]
	ldr	r1, [r5, #4]
	str	r1, [r2, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r4, #-1028]
	b	.L1296
.L1299:
	ldr	r1, .L1301
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
	b	.L1267
.L1273:
	mov	r6, r2, asl #2
	ldr	r7, [r5, #4]
	str	r6, [sp, #16]
	ldr	r6, [ip, r2, asl #2]
	cmp	r7, r6
	bls	.L1268
	ldr	ip, [ip, #0]
	mov	r6, r9
	cmp	ip, #0
	bne	.L1269
	cmp	r1, r0
	addne	r1, r1, #1
	ldrne	ip, .L1301
	movwne	r0, #3966
	strneh	r1, [ip, r0]	@ movhi
.L1269:
	sxth	ip, r3
	mov	r0, #0
	str	ip, [sp, #20]
	ldr	r9, .L1301
	str	r3, [sp, #4]
	b	.L1270
.L1271:
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
.L1270:
	ldr	ip, [sp, #20]
	sxth	r1, r0
	cmp	r1, ip
	bne	.L1271
	ldr	r1, [sl, #-896]
	mov	r2, r2, asl #1
	ldr	r0, [r5, #4]
	ldr	ip, [sp, #16]
	ldr	r3, [sp, #4]
	str	r0, [r1, ip]
	ldr	r1, .L1301
	ldr	r1, [r1, #3968]
	strh	r6, [r1, r2]	@ movhi
	b	.L1272
.L1268:
	sub	r3, r3, #1
	uxth	r3, r3
.L1267:
	ldr	r6, [sp, #4]
	sxth	r2, r3
	cmp	r2, r6
	bgt	.L1273
	mov	r6, r9
.L1272:
	sxth	r3, r3
	cmp	r3, #0
	blt	.L1254
	ldr	r2, .L1301
	movw	ip, #3940
	movw	r1, #3966
	ldrh	ip, [r2, ip]
	ldrh	r0, [r2, r1]
	sub	ip, ip, #1
	rsb	ip, r0, ip
	sxth	ip, ip
	cmp	r3, ip
	bgt	.L1254
	add	r0, r0, #1
	strh	r0, [r2, r1]	@ movhi
	ldr	r0, [r5, #4]
	ldr	r1, [r4, #-896]
	str	r0, [r1, r3, asl #2]
	mov	r3, r3, asl #1
	ldr	r2, [r2, #3968]
.L1296:
	strh	r6, [r2, r3]	@ movhi
	b	.L1254
.L1256:
	ldr	r3, .L1301+16
	movw	ip, #65535
	ldrh	r2, [r4, r3]
	cmp	r2, ip
	streqh	r6, [r4, r3]	@ movhi
	beq	.L1297
	ldr	r3, .L1301+20
	movw	r1, #65535
	ldrh	r0, [r4, r3]
	cmp	r0, r1
	beq	.L1275
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L1275:
	ldr	r3, [r5, #4]
	ldr	r2, [sl, #-1760]
	cmp	r2, r3
	ldr	r3, .L1301+20
	strcsh	r6, [r4, r3]	@ movhi
	bcs	.L1254
	ldr	r2, .L1301+16
	ldrh	r1, [r4, r2]
	strh	r6, [r4, r2]	@ movhi
	strh	r1, [r4, r3]	@ movhi
.L1297:
	ldr	r3, [r5, #4]
	str	r3, [r4, #-1760]
	b	.L1254
.L1300:
	mov	r0, r6
	b	.L1298
.L1251:
	ldr	r3, .L1301
	mov	r0, r6
	ldrb	r1, [r3, #3072]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L1277
.L1298:
	mov	r1, #0
.L1277:
	bl	FtlFreeSysBlkQueueIn
.L1254:
	ldr	r6, [sp, #12]
	add	r8, r8, #36
	add	r3, r6, #1
	uxth	r3, r3
	str	r3, [sp, #12]
	cmp	r3, fp
	bne	.L1278
.L1247:
	ldr	ip, [sp, #8]
	add	r3, ip, #1
	uxth	r3, r3
	str	r3, [sp, #8]
.L1244:
	ldr	r6, .L1301
	movw	r3, #3862
	ldr	ip, [sp, #8]
	ldrh	r3, [r6, r3]
	cmp	r3, ip
	bhi	.L1292
	ldr	r1, .L1301+4
	ldr	r2, [r1, #-1028]
	ldrh	r3, [r2, #0]
	cmp	r3, #0
	bne	.L1280
	ldr	r0, .L1301+8
	ldrh	r1, [r1, r0]
	cmp	r1, #0
	ldrne	r0, [r6, #3948]
	bne	.L1281
	b	.L1280
.L1285:
	mov	r1, r1, asl #1
	ldrh	r1, [r2, r1]
	cmp	r1, #0
	beq	.L1282
	ldr	r7, .L1301
	sxth	r6, r3
	ldr	r1, .L1301+4
	mov	r5, #0
	b	.L1283
.L1284:
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
.L1283:
	ldr	r0, [r7, #3948]
	sxth	r2, r3
	cmp	r2, r0
	bcc	.L1284
	b	.L1280
.L1282:
	add	r3, r3, #1
	uxth	r3, r3
.L1281:
	sxth	r1, r3
	cmp	r1, r0
	bcc	.L1285
.L1280:
	ldr	r2, .L1301
	ldr	r1, [r2, #3968]
	ldrh	r3, [r1, #0]
	cmp	r3, #0
	bne	.L1286
	movw	r0, #3966
	ldrh	r0, [r2, r0]
	cmp	r0, #0
	movwne	r0, #3940
	ldrneh	r2, [r2, r0]
	bne	.L1287
	b	.L1286
.L1291:
	mov	ip, r0, asl #1
	ldrh	ip, [r1, ip]
	cmp	ip, #0
	beq	.L1288
	ldr	r1, .L1301
	movw	r8, #3940
	ldr	r7, .L1301+4
	mov	r6, #0
	b	.L1289
.L1290:
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
.L1289:
	ldrh	ip, [r1, r8]
	sxth	r2, r3
	cmp	r2, ip
	blt	.L1290
	b	.L1286
.L1288:
	add	r3, r3, #1
	uxth	r3, r3
.L1287:
	sxth	r0, r3
	cmp	r0, r2
	blt	.L1291
.L1286:
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1302:
	.align	2
.L1301:
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
	ldr	r3, .L1314
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
	bne	.L1307
	b	.L1312
.L1310:
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
	bne	.L1308
	ldr	r3, [sp, #44]
	cmn	r3, #1
	bne	.L1308
	ldr	r3, [sp, #4]
	cmn	r3, #1
	subne	r4, r6, #1
	uxthne	r4, r4
	bne	.L1312
.L1308:
	add	r6, r6, #1
	uxth	r8, r6
.L1312:
	sxth	r6, r8
	sxth	r3, r4
	cmp	r6, r3
	ble	.L1310
.L1307:
	sxth	r0, r4
	add	sp, sp, #108
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1315:
	.align	2
.L1314:
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
	ldr	r4, .L1336
	ldr	r6, .L1336+4
	ldr	r7, .L1336+8
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
	beq	.L1333
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	ldr	r3, .L1336+12
	ldrh	r7, [r4, r7]
	uxth	r5, r0
	add	r2, r5, #1
	strh	r2, [r4, r3]	@ movhi
	b	.L1335
.L1324:
	sxth	r8, r7
	mov	r1, #1
	ldr	r0, .L1336+16
	mov	r2, r1
	orr	r3, r3, r8, asl #10
	str	r3, [r4, #196]
	ldr	r3, [r4, #-948]
	str	r3, [r4, #200]
	bl	FlashReadPages
	ldrb	r3, [r6, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1319
	ldr	r3, [r4, #204]
	ldr	sl, [r3, #12]
	cmp	sl, #0
	beq	.L1319
	movw	r3, #3928
	ldr	r0, [r4, #200]
	ldrh	r1, [r6, r3]
	bl	js_hash
	cmp	sl, r0
	beq	.L1319
	cmp	r5, #0
	bne	.L1320
	ldr	r3, .L1336+20
	ldrh	r3, [r4, r3]
	cmp	r8, r3
	uxthne	r7, r3
	movne	r3, #3920
	ldrneh	r5, [r6, r3]
	bne	.L1322
.L1320:
	mvn	r3, #0
	str	r3, [r4, #192]
.L1319:
	ldr	r3, [r4, #192]
	cmn	r3, #1
	beq	.L1322
	ldr	r3, [r4, #-948]
	ldr	r2, [r3, #0]
	ldr	r3, .L1336+24
	cmp	r2, r3
	bne	.L1322
	ldr	r3, [r4, #-924]
	ldrh	r2, [r3, #0]
	movw	r3, #61604
	cmp	r2, r3
	beq	.L1323
.L1322:
	sub	r5, r5, #1
	uxth	r5, r5
.L1335:
	sxth	r3, r5
	cmp	r3, #0
	bge	.L1324
.L1323:
	ldr	r4, .L1336
	mov	r2, #48
	ldr	r5, .L1336+4
	movw	r6, #3860
	ldr	r0, .L1336+28
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
	beq	.L1325
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
.L1325:
	ldr	r4, .L1336
	ldr	r3, .L1336+24
	ldr	r2, [r4, #-2068]
	cmp	r2, r3
	bne	.L1333
	ldr	r3, .L1336+32
	movw	r1, #3874
	ldrb	r0, [r4, #-2058]	@ zero_extendqisi2
	ldrh	r2, [r4, r3]
	ldr	r3, .L1336+36
	strh	r2, [r4, r3]	@ movhi
	ldr	r3, .L1336+4
	ldrh	r1, [r3, r1]
	cmp	r0, r1
	bne	.L1333
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
	ldr	r3, .L1336+40
	ldr	r1, .L1336+44
	strh	r0, [r4, r3]	@ movhi
	ldr	r3, .L1336+48
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
	ldr	r0, .L1336+52
	strb	r3, [r4, #-1984]
	ldr	r3, .L1336+56
	strh	r1, [r4, r3]	@ movhi
	mov	r3, #0
	ldr	r1, .L1336+60
	strh	r3, [r4, r0]	@ movhi
	strb	r3, [r4, #-1746]
	ldrh	r0, [r4, r1]
	add	r1, r1, #106
	strb	r3, [r4, #-1744]
	str	r3, [r4, #-1812]
	strh	r0, [r4, r1]	@ movhi
	ldr	r1, .L1336+64
	ldr	r0, .L1336+68
	ldrh	r1, [r4, r1]
	mov	ip, r1, lsr #6
	and	r1, r1, #63
	strb	r1, [r4, #-1938]
	ldrb	r1, [r4, #-2056]	@ zero_extendqisi2
	strh	ip, [r4, r0]	@ movhi
	strb	r1, [r4, #-1936]
	ldr	r1, .L1336+72
	ldrh	r0, [r4, r1]
	add	r1, r1, #150
	strh	r0, [r4, r1]	@ movhi
	sub	r1, r1, #148
	ldr	r0, .L1336+76
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
	ldr	r4, .L1336
	ldr	r3, [r4, #-2032]
	ldr	r1, [r4, #-1796]
	cmp	r3, r1
	strhi	r3, [r4, #-1796]
	cmp	r2, r5
	beq	.L1328
	ldr	r0, .L1336+80
	bl	make_superblock
.L1328:
	ldr	r3, .L1336+84
	ldrh	r3, [r4, r3]
	cmp	r3, r5
	beq	.L1329
	ldr	r0, .L1336+88
	bl	make_superblock
.L1329:
	ldr	r5, .L1336
	movw	r4, #65535
	ldr	r3, .L1336+92
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1330
	ldr	r0, .L1336+96
	bl	make_superblock
.L1330:
	ldr	r3, .L1336+56
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L1334
	ldr	r0, .L1336+100
	bl	make_superblock
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1333:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1334:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1337:
	.align	2
.L1336:
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
	ldr	r6, .L1354
	ldr	r8, .L1354+4
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
	b	.L1339
.L1343:
	mov	r1, #1
	mov	r3, r5, asl #10
	ldr	r0, .L1354+8
	mov	r2, r1
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldr	r3, [r6, #192]
	cmn	r3, #1
	bne	.L1340
	ldr	r3, [r6, #196]
	mov	r1, #1
	ldr	r0, .L1354+8
	mov	r2, r1
	add	r3, r3, #1
	str	r3, [r6, #196]
	bl	FlashReadPages
.L1340:
	ldr	r3, [r7, #192]
	cmn	r3, #1
	beq	.L1341
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1341
	ldr	r3, .L1354+4
	mov	r2, #3984
	strh	r5, [r3, r2]	@ movhi
	ldr	r2, [r4, #4]
	str	r2, [r3, #3992]
	movw	r2, #3988
	ldrh	r1, [r4, #8]
	strh	r1, [r3, r2]	@ movhi
	b	.L1342
.L1341:
	sub	r5, r5, #1
	uxth	r5, r5
.L1339:
	movw	r3, #3916
	ldrh	r3, [r8, r3]
	sub	r3, r3, #48
	cmp	r5, r3
	bgt	.L1343
.L1342:
	ldr	r5, .L1354+4
	mov	r8, #3984
	movw	r3, #65535
	ldrh	r2, [r5, r8]
	cmp	r2, r3
	beq	.L1353
	movw	r6, #3988
	ldrh	r2, [r5, r6]
	cmp	r2, r3
	beq	.L1345
	ldr	r7, .L1354
	mov	r1, #1
	mov	r2, r2, asl #10
	add	r0, r7, #192
	str	r2, [r7, #196]
	mov	r2, r1
	bl	FlashReadPages
	ldr	r3, [r7, #192]
	cmn	r3, #1
	beq	.L1345
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L1345
	ldr	r3, [r4, #4]
	ldr	r2, [r5, #3992]
	cmp	r3, r2
	strhi	r3, [r5, #3992]
	ldrhih	r2, [r5, r6]
	ldrhih	r3, [r4, #8]
	strhih	r2, [r5, r8]	@ movhi
	strhih	r3, [r5, r6]	@ movhi
.L1345:
	ldr	r8, .L1354+4
	mov	r3, #3984
	mov	r1, #1
	mov	r6, #0
	ldr	r5, .L1354
	ldrh	r0, [r8, r3]
	bl	FtlGetLastWrittenPage
	movw	r3, #3986
	add	r2, r0, #1
	mov	r7, r0
	strh	r2, [r8, r3]	@ movhi
	b	.L1346
.L1349:
	mov	r2, #3984
	mov	r1, #1
	ldrh	r2, [r8, r2]
	ldr	r0, .L1354+8
	orr	r3, r3, r2, asl #10
	str	r3, [r5, #196]
	ldr	r3, [r5, #-948]
	mov	r2, r1
	str	r3, [r5, #200]
	bl	FlashReadPages
	ldr	r3, [r5, #192]
	cmn	r3, #1
	beq	.L1347
	ldrh	r2, [r4, #0]
	movw	r3, #61649
	cmp	r2, r3
	beq	.L1348
.L1347:
	sub	r6, r6, #1
	uxth	r6, r6
.L1346:
	add	r3, r6, r7
	sxth	r3, r3
	cmp	r3, #0
	bge	.L1349
.L1348:
	ldr	r3, .L1354+4
	movw	r2, #3990
	ldrh	r1, [r4, #10]
	ldrh	r0, [r4, #12]
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	beq	.L1350
	ldr	r2, [r3, #3848]
	cmp	r0, r2
	beq	.L1350
	movw	r1, #3862
	ldrh	r3, [r3, r1]
	mov	r3, r3, lsr #2
	cmp	r2, r3
	bcs	.L1350
	cmp	r0, r3
	bcs	.L1350
	bl	FtlSysBlkNumInit
.L1350:
	ldr	r6, .L1354+12
	mov	r4, #0
	ldr	r8, .L1354+4
	movw	r7, #3874
	ldr	r5, .L1354
	b	.L1351
.L1352:
	ldr	r3, .L1354+16
	ldr	r1, [r5, #200]
	ldr	r0, [r6, #4]!
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #2
	mla	r1, r4, r2, r1
	bl	memcpy
	add	r4, r4, #1
.L1351:
	ldrh	r3, [r8, r7]
	cmp	r4, r3
	bcc	.L1352
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1353:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1355:
	.align	2
.L1354:
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
	ldr	r3, .L1363
	movw	fp, #3874
	ldr	r7, .L1363+4
	ldr	r6, .L1363+8
	ldr	r2, [r3, #-948]
	ldr	r8, [r3, #-924]
	str	r2, [r3, #200]
	str	r8, [r3, #204]
	b	.L1357
.L1362:
	movw	r9, #3916
	mvn	r3, #0
	ldrh	r4, [r6, r9]
	strh	r3, [r7], #2	@ movhi
	add	r4, r4, r3
	ldr	sl, .L1363
	uxth	r4, r4
	b	.L1358
.L1361:
	mla	r3, r3, r5, r4
	mov	r1, #1
	ldr	r0, .L1363+12
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sl, #196]
	bl	FlashReadPages
	ldr	r3, [sl, #192]
	cmn	r3, #1
	beq	.L1359
	ldrh	r2, [r8, #0]
	movw	r3, #61664
	cmp	r2, r3
	streqh	r4, [r7, #-2]	@ movhi
	beq	.L1360
.L1359:
	sub	r4, r4, #1
	uxth	r4, r4
.L1358:
	ldrh	r3, [r6, r9]
	sub	r2, r3, #16
	cmp	r4, r2
	bgt	.L1361
.L1360:
	add	r5, r5, #1
.L1357:
	ldrh	r3, [r6, fp]
	cmp	r5, r3
	bcc	.L1362
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1364:
	.align	2
.L1363:
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
	ldr	r3, .L1384
	mov	r6, r0
	mov	r8, #0
	ldr	r7, .L1384+4
	ldrb	fp, [r3, #3125]	@ zero_extendqisi2
	b	.L1366
.L1373:
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
	bcs	.L1368
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
.L1368:
	add	r8, r8, #1
	add	r6, r6, #36
.L1366:
	cmp	r8, sl
	bne	.L1373
	ldr	r3, [sp, #12]
	cmp	r3, #0
	movne	r6, #0
	ldrne	r5, .L1384+8
	bne	.L1374
	b	.L1375
.L1380:
	ldr	r3, [r4, #0]
	cmn	r3, #1
	bne	.L1376
	ldr	r1, [r4, #4]
	ldr	r0, .L1384+12
	bl	printk
	b	.L1377
.L1376:
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
	bne	.L1378
	ldr	r0, .L1384+16
	ldr	r1, [r4, #4]
	bl	printk
	str	r7, [r4, #0]
.L1378:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L1379
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #236]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1379
	ldr	r0, .L1384+20
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L1379:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L1377
	ldr	r2, [r3, #0]
	ldr	r3, [r5, #232]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1377
	ldr	r0, .L1384+24
	ldr	r1, [r4, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #0]
.L1377:
	add	r6, r6, #1
	add	r4, r4, #36
.L1374:
	cmp	r6, sl
	bne	.L1380
.L1375:
	mov	r0, #0
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1385:
	.align	2
.L1384:
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
	ldr	r5, .L1414
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
	ldr	ip, .L1414+4
	ldrb	ip, [ip, #3125]	@ zero_extendqisi2
	str	ip, [sp, #12]
	beq	.L1387
	bl	FlashProgSlc2KPages
	b	.L1388
.L1401:
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
	bcs	.L1390
	ldrb	r1, [r5, #3841]	@ zero_extendqisi2
	add	r2, fp, r2, asl #4
	cmp	r1, #0
	ldr	r2, [r2, #3636]
	moveq	r8, #0
	cmp	r2, #0
	beq	.L1392
	cmp	r3, #1
	bne	.L1393
	ldr	r0, [r5, #3008]
	bl	NandcIqrWaitFlashReady
.L1393:
	ldrb	r0, [sp, #32]	@ zero_extendqisi2
	bl	FlashWaitCmdDone
.L1392:
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
	bne	.L1395
	bl	NandcWaitFlashReady
	b	.L1396
.L1395:
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
.L1396:
	ldr	r2, [sp, #20]
	sub	r3, r2, #1
	cmp	r3, #7
	bhi	.L1397
	add	r3, r5, r4
	ldrb	r3, [r3, #3756]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1397
	mov	r0, r4
	ldrb	r1, [r5, #1]	@ zero_extendqisi2
	ldr	r2, .L1414+8
	mov	r3, #0
	bl	HynixSetRRPara
.L1397:
	mov	r0, r4
	bl	NandcFlashCs
	cmp	sl, #1
	mov	r0, r4
	bne	.L1398
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1398
	bl	flash_enter_slc_mode
	b	.L1399
.L1398:
	bl	flash_exit_slc_mode
.L1399:
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
	beq	.L1400
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
.L1400:
	mov	r0, r4
	ldr	r1, [sp, #28]
	bl	FlashProgSecondCmd
	mov	r0, r4
	bl	NandcFlashDeCs
	add	r6, r6, r8
.L1390:
	add	r6, r6, #1
.L1387:
	ldr	r3, [sp, #8]
	cmp	r6, r3
	bcc	.L1401
	ldr	r5, .L1414
	mov	r4, #0
	ldr	r6, .L1414+12
	ldr	r0, [r5, #3008]
	bl	NandcIqrWaitFlashReady
	b	.L1402
.L1404:
	uxtb	r0, r4
	bl	FlashWaitCmdDone
	cmp	sl, #1
	bne	.L1403
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1403
	ldrb	r0, [r6, r4, asl #4]	@ zero_extendqisi2
	bl	flash_exit_slc_mode
.L1403:
	add	r4, r4, #1
.L1402:
	ldrb	r3, [r5, #3762]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L1404
	ldr	r2, [sp, #16]
	cmp	r2, #0
	ldreq	r0, [sp, #16]
	beq	.L1388
	mov	r5, #0
	ldr	r4, .L1414+16
	ldr	r6, [sp, #8]
	b	.L1405
.L1410:
	ldr	r3, [r9, #0]
	cmn	r3, #1
	bne	.L1406
	ldr	r1, [r9, #4]
	ldr	r0, .L1414+20
	bl	printk
	b	.L1407
.L1406:
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
	bne	.L1408
	ldr	r0, .L1414+24
	ldr	r1, [r9, #4]
	bl	printk
	str	r7, [r9, #0]
.L1408:
	ldr	r3, [r9, #12]
	cmp	r3, #0
	beq	.L1409
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #236]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1409
	ldr	r0, .L1414+28
	ldr	r1, [r9, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r9, #0]
.L1409:
	ldr	r3, [r9, #8]
	cmp	r3, #0
	beq	.L1407
	ldr	r2, [r3, #0]
	ldr	r3, [r4, #232]
	ldr	r3, [r3, #0]
	cmp	r2, r3
	beq	.L1407
	ldr	r0, .L1414+32
	ldr	r1, [r9, #4]
	bl	printk
	mvn	r3, #0
	str	r3, [r9, #0]
.L1407:
	add	r5, r5, #1
	add	r9, r9, #36
.L1405:
	cmp	r5, r6
	bne	.L1410
	mov	r0, #0
.L1388:
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1415:
	.align	2
.L1414:
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
	ldr	r4, .L1428
	movw	r7, #3860
	ldr	r6, .L1428+4
	ldr	r3, [r4, #-948]
	ldr	r5, [r4, #-924]
	str	r3, [r4, #200]
	ldr	r3, .L1428+8
	str	r5, [r4, #204]
	ldrh	r3, [r4, r3]
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, .L1428+12
	strh	r3, [r5, #0]	@ movhi
	ldr	r3, [r4, #-1760]
	str	r3, [r5, #4]
	mov	r3, #0
	str	r3, [r5, #8]
	str	r3, [r5, #12]
	ldr	r3, .L1428+16
	str	r3, [r4, #-2068]
	ldr	r3, .L1428+20
	str	r3, [r4, #-2064]
	ldr	r3, .L1428+24
	ldrh	r2, [r4, r3]
	ldr	r3, .L1428+28
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #3874
	ldrh	r3, [r6, r3]
	strb	r3, [r4, #-2058]
	ldr	r3, .L1428+32
	ldrh	r2, [r4, r3]
	sub	r3, r3, #62
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #64
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1986]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1428+36
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1984]	@ zero_extendqisi2
	strb	r3, [r4, #-2057]
	ldr	r3, .L1428+40
	ldrh	r2, [r4, r3]
	sub	r3, r3, #106
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #108
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1938]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1428+44
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1936]	@ zero_extendqisi2
	strb	r3, [r4, #-2056]
	ldr	r3, .L1428+48
	ldrh	r2, [r4, r3]
	sub	r3, r3, #150
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #152
	ldr	r0, [r4, #200]
	ldrh	r2, [r4, r3]
	ldrb	r3, [r4, #-1890]	@ zero_extendqisi2
	orr	r2, r3, r2, asl #6
	ldr	r3, .L1428+52
	strh	r2, [r4, r3]	@ movhi
	ldrb	r3, [r4, #-1888]	@ zero_extendqisi2
	strb	r3, [r4, #-2055]
	ldr	r3, [r4, #-1792]
	str	r3, [r4, #-2036]
	ldr	r3, [r4, #-1800]
	str	r3, [r4, #-2028]
	ldr	r3, [r4, #-1796]
	str	r3, [r4, #-2032]
	ldr	r3, .L1428+56
	ldrh	r2, [r4, r3]
	ldr	r3, .L1428+60
	strh	r2, [r4, r3]	@ movhi
	add	r3, r3, #892
	ldrh	r2, [r4, r3]
	ldr	r3, .L1428+64
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #3928
	ldrh	r2, [r6, r3]
	bl	ftl_memset
	ldr	r1, .L1428+68
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
	beq	.L1417
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
.L1417:
	mov	r0, #0
	ldr	r4, .L1428
	bl	FtlUpdateVaildLpn
	ldr	sl, .L1428+8
	ldr	r6, .L1428+72
	mov	r7, #0
	movw	r8, #65535
.L1427:
	ldr	r3, [r4, #-948]
	ldrh	r2, [r4, sl]
	ldr	r0, .L1428+4
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
	blt	.L1419
	ldr	r3, .L1428+76
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
.L1419:
	mov	r1, #1
	ldr	r0, .L1428+80
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r3, [r4, r6]
	ldr	r2, [r4, #192]
	add	r3, r3, #1
	cmn	r2, #1
	uxth	r3, r3
	strh	r3, [r4, r6]	@ movhi
	bne	.L1420
	cmp	r3, #1
	add	r7, r7, #1
	ldreq	r2, .L1428+4
	moveq	r3, #3920
	uxth	r7, r7
	ldreqh	r3, [r2, r3]
	subeq	r3, r3, #1
	streqh	r3, [r4, r6]	@ movhi
	cmp	r7, #3
	bls	.L1427
	ldr	r3, .L1428
	mov	r2, r7
	ldr	r0, .L1428+84
	ldr	r1, [r3, #196]
	bl	printk
	ldr	r3, .L1428+4
	mov	r2, #1
	str	r2, [r3, #4044]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1420:
	cmp	r3, #1
	beq	.L1427
	cmp	r2, #256
	beq	.L1427
	movw	r3, #65535
	cmp	r8, r3
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	mov	r0, r8
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1429:
	.align	2
.L1428:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	-1768
	.word	-3932
	.word	1179929683
	.word	1342177364
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
	ldr	r3, .L1432
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L1431
	bl	FtlVpcTblFlush.part.16
.L1431:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1433:
	.align	2
.L1432:
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
	ldr	r7, .L1447
	mov	r5, #0
	ldr	sl, .L1447+4
	movw	r9, #3874
	ldr	r8, .L1447+8
	ldr	r3, [r7, #-924]
	ldr	r0, [r7, #-948]
	str	r3, [r7, #204]
	movw	r3, #3928
	str	r0, [r7, #200]
	ldrh	r2, [sl, r3]
	bl	ftl_memset
	b	.L1435
.L1436:
	ldr	r3, .L1447+12
	ldr	r1, [r8, #4]!
	ldrh	r2, [r7, r3]
	ldr	r3, [r7, #200]
	mul	r0, r2, r5
	mov	r2, r2, asl #2
	add	r5, r5, #1
	add	r0, r3, r0, asl #2
	bl	memcpy
.L1435:
	ldrh	r3, [sl, r9]
	ldr	r4, .L1447+4
	cmp	r5, r3
	ldr	r6, .L1447
	blt	.L1436
	ldr	r5, [r6, #204]
	mov	r1, #255
	mov	r2, #16
	mov	r7, #0
	mov	fp, r7
	mov	r0, r5
	bl	ftl_memset
	ldr	r3, .L1447+16
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
	b	.L1445
.L1442:
	mov	fp, #1
.L1445:
	ldr	r3, [r6, #-948]
	mov	r8, #3984
	ldr	r4, .L1447+4
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
	ldr	r0, .L1447+20
	bl	printk
	mov	r3, #3920
	ldrh	r3, [r4, r3]
	ldrh	r2, [r4, sl]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L1438
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
.L1438:
	ldr	r4, .L1447
	mov	r1, #1
	ldr	r8, .L1447+4
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
	bne	.L1439
	add	r7, r7, #1
	ldr	r0, .L1447+24
	ldr	r1, [r6, #196]
	uxth	r7, r7
	bl	printk
	cmp	r7, #3
	bls	.L1445
	ldr	r0, .L1447+28
	mov	r2, r7
	ldr	r1, [r4, #196]
	bl	printk
	mov	r3, #1
	str	r3, [r8, #4044]
	b	.L1446
.L1439:
	cmp	fp, #0
	beq	.L1442
.L1446:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1448:
	.align	2
.L1447:
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
	ldr	r3, .L1451
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L1450
	bl	FtlBbmTblFlush.part.18
.L1450:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1452:
	.align	2
.L1451:
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
	ldr	r4, .L1463
	ldr	r3, .L1463+4
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	movne	r6, #0
	movne	sl, r4
	bne	.L1454
	b	.L1455
.L1461:
	add	r3, r3, r6
	mov	r1, r9
	mov	r5, #0
	ldr	r7, .L1463+4
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	b	.L1456
.L1460:
	add	r3, r4, r5, asl #1
	sub	r3, r3, #1120
	sub	r3, r3, #4
	ldrh	r3, [r3, #0]
	cmp	r3, r8
	bne	.L1457
	mov	r1, r8
	ldr	r0, .L1463+8
	bl	printk
	mov	r0, r8
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r1, [r4, r7]
	mov	r3, r5
	b	.L1458
.L1459:
	add	r0, r3, #1
	add	r3, r4, r3, asl #1
	sub	r3, r3, #1120
	add	r2, r4, r0, asl #1
	sub	r2, r2, #1120
	sub	r2, r2, #4
	ldrh	r2, [r2, #0]
	strh	r2, [r3, #-4]	@ movhi
	uxth	r3, r0
.L1458:
	cmp	r3, r1
	bcc	.L1459
	sub	r1, r1, #1
	strh	r1, [sl, r7]	@ movhi
.L1457:
	add	r5, r5, #1
	uxth	r5, r5
.L1456:
	ldrh	r3, [r4, r7]
	cmp	r3, r5
	bhi	.L1460
	add	r6, r6, #1
	uxth	r6, r6
.L1454:
	ldr	r3, .L1463+12
	movw	r2, #3852
	ldrh	r2, [r3, r2]
	cmp	r2, r6
	bhi	.L1461
	bl	FtlGcReFreshBadBlk
.L1455:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1464:
	.align	2
.L1463:
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
	ldr	r3, .L1473
	mov	r4, r0
	ldr	r1, [r3, #-2012]
	ldrh	r2, [r1, r2]
	cmp	r2, #0
	bne	.L1466
	ldr	r1, .L1473+4
	ldrh	r0, [r3, r1]
	cmp	r0, r4
	mvneq	r2, #0
	streqh	r2, [r3, r1]	@ movhi
	beq	.L1468
	ldr	r1, .L1473+8
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1469
	ldr	r1, .L1473+12
	ldrh	r1, [r3, r1]
	cmp	r1, r4
	beq	.L1469
	ldr	r1, .L1473+16
	ldrh	r3, [r3, r1]
	cmp	r3, r4
	beq	.L1469
.L1468:
	ldr	r5, .L1473
	mov	r1, r4
	sub	r0, r5, #2016
	bl	List_remove_node
	ldr	r3, .L1473+20
	mov	r0, r4
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	bl	free_data_superblock
	mov	r0, r4
	bl	FtlGcFreeBadSuperBlk
	mov	r2, #1
	b	.L1469
.L1466:
	bl	List_update_data_list
	mov	r2, #0
.L1469:
	mov	r0, r2
	ldmfd	sp!, {r3, r4, r5, pc}
.L1474:
	.align	2
.L1473:
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
	beq	.L1476
	ldr	r2, .L1481
	mov	r3, r0, asl #1
	ldr	r2, [r2, #-2012]
	ldrh	r1, [r2, r3]
	cmp	r1, #0
	subne	r1, r1, #1
	strneh	r1, [r2, r3]	@ movhi
	bne	.L1476
	ldmfd	sp!, {r4, r5, r6, lr}
	b	decrement_vpc_count.part.20
.L1476:
	ldr	r6, .L1481
	movw	r3, #65535
	ldr	r5, .L1481+4
	ldrh	r0, [r6, r5]
	cmp	r0, r3
	streqh	r4, [r6, r5]	@ movhi
	moveq	r0, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	cmp	r0, r4
	beq	.L1480
	bl	update_vpc_list
	strh	r4, [r6, r5]	@ movhi
	adds	r0, r0, #0
	movne	r0, #1
	ldmfd	sp!, {r4, r5, r6, pc}
.L1480:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1482:
	.align	2
.L1481:
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
	ldr	r5, .L1495
	ldr	r7, .L1495+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1484
.L1485:
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
.L1484:
	cmp	r3, r6
	beq	.L1485
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L1487
	ldrb	r2, [r5, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1487
	ldrh	r2, [r4, #2]
	add	r2, r7, r2, asl #1
	sub	r2, r2, #880
	ldrh	r2, [r2, #0]
	cmp	r2, r6
	bne	.L1487
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1485
.L1487:
	ldrh	r6, [r4, #2]
	movw	r5, #65535
	mov	r7, r5
	orr	r6, r6, r3, asl #10
	ldrh	r3, [r4, #4]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
.L1492:
	ldr	r2, .L1495
	movw	r3, #3852
	ldrh	r2, [r2, r3]
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
.L1489:
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
	beq	.L1489
	strb	r3, [r4, #6]
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1490
	ldr	r3, .L1495
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	ldrh	r2, [r4, #2]
	bne	.L1493
	ldr	r3, .L1495+4
	add	r2, r3, r2, asl #1
	sub	r2, r2, #880
	ldrh	r3, [r2, #0]
	cmp	r3, r7
	bne	.L1490
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1490
	sub	r3, r3, #1
	ldrh	r0, [r4, #0]
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	b	.L1492
.L1493:
	mov	r1, #3920
	ldrh	r1, [r3, r1]
	cmp	r2, r1
	bcc	.L1490
	ldr	r1, .L1495+4
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
.L1490:
	mov	r0, r6
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1496:
	.align	2
.L1495:
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
	ldr	r5, .L1503
	ldr	r7, .L1503+4
	add	r3, r0, r3, asl #1
	ldrh	r3, [r3, #16]
	b	.L1498
.L1499:
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
.L1498:
	cmp	r3, r6
	beq	.L1499
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L1501
	ldrb	r3, [r5, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1501
	ldrh	r3, [r4, #2]
	add	r3, r7, r3, asl #1
	sub	r3, r3, #880
	ldrh	r3, [r3, #0]
	cmp	r3, r6
	bne	.L1501
	ldrh	r3, [r4, #4]
	ldrh	r0, [r4, #0]
	sub	r3, r3, #1
	strh	r3, [r4, #4]	@ movhi
	bl	decrement_vpc_count
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1499
	ldrh	r2, [r4, #2]
	strb	r3, [r4, #6]
	add	r2, r2, #1
	strh	r2, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1501:
	ldr	r3, .L1503
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
	ldr	r1, .L1503+4
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
.L1504:
	.align	2
.L1503:
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
	ldr	r6, .L1546
	mov	r4, r0
	ldr	r5, .L1546+4
	mov	sl, r6
.L1541:
	ldr	r3, .L1546+8
	cmp	r4, r3
	bne	.L1509
	ldr	r3, .L1546+12
	ldrh	r2, [r5, r3]
	ldr	r3, [r5, #-1176]
	mov	r1, r2, lsr #1
	mul	ip, r3, r2
	add	r0, r1, #1
	add	r0, r0, ip, lsr #2
	ldr	ip, [r6, #3844]
	cmp	ip, #0
	uxth	r0, r0
	beq	.L1533
	ldr	ip, [r5, #-1776]
	cmp	ip, #39
	bhi	.L1533
	cmp	ip, #2
	bls	.L1538
	tst	r2, #1
	beq	.L1510
	cmp	r3, #0
	moveq	r1, r3
	beq	.L1511
	b	.L1510
.L1509:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L1538
	mov	r3, #3872
	ldrh	r3, [r6, r3]
	cmp	r3, #1
	beq	.L1538
	ldrb	r3, [r6, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1538
	ldr	r3, .L1546+12
	ldr	r2, [r6, #3844]
	ldrh	r3, [r5, r3]
	cmp	r2, #0
	mov	r1, r3, lsr #3
	beq	.L1510
	ldr	r2, [r5, #-1776]
	cmp	r2, #1
	movls	r1, #7
	mulls	r1, r1, r3
	movls	r1, r1, lsr #3
	b	.L1510
.L1533:
	mov	r1, r0
.L1510:
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
	b	.L1511
.L1538:
	mov	r1, #0
.L1511:
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	ldr	r0, .L1546+16
	bl	List_pop_index_node
	ldr	r3, .L1546+12
	ldrh	r2, [r5, r3]
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	movw	r3, #3860
	ldrh	r3, [r6, r3]
	uxth	r9, r0
	cmp	r3, r9
	bls	.L1541
	ldr	r3, [r5, #-2012]
	mov	r7, r9, asl #1
	ldrh	r8, [r3, r7]
	cmp	r8, #0
	bne	.L1541
	strh	r9, [r4, #0]	@ movhi
	mov	r0, r4
	bl	make_superblock
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1542
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
	b	.L1514
.L1516:
	str	r0, [r3, #8]
	movw	r4, #65535
	str	r0, [r3, #12]
	ldrh	lr, [r1, #16]
	cmp	lr, r4
	beq	.L1515
	ldr	r4, [sp, #4]
	mov	ip, #36
	mov	lr, lr, asl #10
	mla	fp, ip, r8, r4
	add	r8, r8, #1
	uxth	r8, r8
	str	lr, [fp, #4]
.L1515:
	add	r2, r2, #1
	add	r3, r3, #36
	add	r1, r1, #2
	uxth	r2, r2
.L1514:
	ldr	fp, [sp, #20]
	cmp	r2, fp
	bne	.L1516
	ldr	r3, [r6, #3844]
	ldr	r4, [sp, #0]
	cmp	r3, #0
	beq	.L1517
	ldr	r3, .L1546+20
	cmp	r4, r3
	bne	.L1517
	ldr	r3, [r6, #4052]
	ldrh	r3, [r3, r7]
	cmp	r3, #40
	movhi	r3, #0
	strhib	r3, [r5, #-1984]
.L1517:
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #0
	ldr	r3, [sl, #4052]
	ldrh	r2, [r3, r7]
	bne	.L1518
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
	b	.L1544
.L1518:
	add	r2, r2, #1
	strh	r2, [r3, r7]	@ movhi
	ldr	r3, [r5, #-1788]
	mov	r0, r9
	mov	r1, #1
	add	r3, r3, #1
	str	r3, [r5, #-1788]
.L1544:
	bl	ftl_set_blk_mode
	ldr	r3, [r6, #4052]
	ldr	fp, .L1546+4
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
	b	.L1523
.L1524:
	add	r2, r2, #1
	ldr	r1, [r3, #-32]
	uxth	r2, r2
	bic	r1, r1, #1020
	bic	r1, r1, #3
	str	r1, [r3, #-32]
.L1523:
	cmp	r2, r8
	add	r3, r3, #36
	bne	.L1524
	ldrb	r3, [r6, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1525
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	mov	r2, r8
	ldr	r0, [r6, #4048]
	cmp	r3, #1
	moveq	r1, #0
	movne	r1, #1
	bl	FlashEraseBlocks
.L1525:
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
	b	.L1527
.L1529:
	ldr	r0, [r6, #4048]
	add	r1, r0, r8
	ldr	r4, [r0, r8]
	cmn	r4, #1
	bne	.L1528
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
.L1528:
	ldr	r4, [sp, #16]
	add	r8, r8, #36
	add	r4, r4, #2
	str	r4, [sp, #16]
.L1527:
	cmp	r8, r9
	bne	.L1529
	cmp	fp, #0
	mov	r4, r7
	mov	r9, r3
	mov	r7, r2
	beq	.L1530
	mov	r0, r3
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1530:
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1531
.L1542:
	ldr	r3, [r5, #-2012]
	mvn	r2, #0
	strh	r2, [r3, r7]	@ movhi
	b	.L1541
.L1531:
	ldr	r1, .L1546
	movw	r2, #3918
	ldrh	r2, [r1, r2]
	strh	r9, [r4, #0]	@ movhi
	mul	r2, r2, r3
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	strb	r3, [r4, #6]
	ldr	r3, .L1546+4
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
.L1547:
	.align	2
.L1546:
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
	ldr	r3, .L1550
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L1549
	bl	allocate_data_superblock.part.22
.L1549:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1551:
	.align	2
.L1550:
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
	ldr	r2, .L1562
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	mov	r4, r0
	ldr	r3, [r2, #4044]
	.pad #44
	sub	sp, sp, #44
	cmp	r3, #0
	bne	.L1552
	ldrb	r7, [r2, #3072]	@ zero_extendqisi2
	cmp	r7, #0
	beq	.L1561
	ldrb	r7, [r0, #8]	@ zero_extendqisi2
	cmp	r7, #1
	ldreqh	r8, [r0, #4]
	beq	.L1554
	mov	r7, r3
.L1561:
	mov	r8, #12
.L1554:
	ldr	r6, .L1562+4
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
	ldr	r2, .L1562+8
	ldr	r3, [r6, #-948]
	str	r2, [r3, #4]
	b	.L1555
.L1558:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1556
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [sp, #8]
	beq	.L1556
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
.L1555:
	cmp	r8, #0
	bne	.L1558
.L1556:
	ldr	r2, .L1562+4
	ldrh	r3, [r4, #0]
	ldrh	r1, [r4, #4]
	ldr	r2, [r2, #-2012]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	rsb	r1, r1, r0
	strh	r1, [r2, r3]	@ movhi
	ldr	r2, .L1562
	movw	r3, #3918
	ldrh	r3, [r2, r3]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
.L1552:
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L1563:
	.align	2
.L1562:
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
	ldr	r8, .L1594
	.pad #28
	sub	sp, sp, #28
	str	r0, [sp, #8]
	ldr	r6, [r8, #4044]
	ldrb	r1, [r8, #3841]	@ zero_extendqisi2
	cmp	r6, #0
	str	r1, [sp, #16]
	movne	r4, #0
	bne	.L1565
	ldrb	r2, [r8, #3072]	@ zero_extendqisi2
	mov	r5, r6
	ldr	fp, .L1594+4
	mov	r4, r6
	mov	r9, #36
	str	r2, [sp, #4]
	str	r0, [fp, #-972]
	b	.L1566
.L1570:
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
	beq	.L1567
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1568
.L1567:
	mov	r0, r7
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
	bne	.L1568
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
.L1568:
	add	r6, r6, #1
	uxth	r6, r6
.L1566:
	movw	r2, #3852
	ldr	r7, .L1594
	ldrh	r3, [r8, r2]
	cmp	r3, r6
	bhi	.L1570
	cmp	r5, #0
	beq	.L1565
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
.L1573:
	ldr	r3, [r7, #4048]
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	cmn	r3, #1
	bne	.L1572
	ldr	r0, [r2, #4]
	add	r4, r4, #1
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
.L1572:
	add	r9, r9, #1
	add	r8, r8, #36
	uxth	r9, r9
	cmp	r9, r5
	bne	.L1573
	cmp	sl, #0
	beq	.L1587
	ldr	r3, .L1594
	mov	r2, #3920
	ldrh	r2, [r3, r2]
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	str	r2, [sp, #12]
	moveq	r2, r2, lsr #2
	streq	r2, [sp, #4]
	moveq	r6, #1
	beq	.L1574
	b	.L1593
.L1587:
	mov	r3, #6
	mov	r1, #1
	str	r3, [sp, #4]
	str	r1, [sp, #12]
	b	.L1574
.L1593:
	mov	r6, #1
	str	r6, [sp, #4]
.L1574:
	ldr	r7, .L1594
	mov	r8, #0
.L1582:
	mov	r9, #0
	ldr	fp, .L1594+4
	mov	r5, r9
	b	.L1575
.L1578:
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
	beq	.L1576
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L1577
.L1576:
	ldr	r0, [sp, #20]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L1577
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
.L1577:
	add	r9, r9, #1
	uxth	r9, r9
.L1575:
	movw	r2, #3852
	ldr	ip, .L1594
	ldrh	r3, [r7, r2]
	cmp	r3, r9
	bhi	.L1578
	cmp	r5, #0
	beq	.L1565
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
.L1581:
	ldr	r3, [r7, #4048]
	add	r2, r3, r9
	ldr	r3, [r3, r9]
	cmp	r3, #0
	beq	.L1580
	ldr	r0, [r2, #4]
	add	r4, r4, #1
	ubfx	r0, r0, #10, #16
	uxth	r4, r4
	bl	FtlBbmMapBadBlock
.L1580:
	add	fp, fp, #1
	add	r9, r9, #36
	uxth	fp, fp
	cmp	fp, r5
	bne	.L1581
	ldr	r2, [sp, #4]
	ldr	r3, [sp, #12]
	add	r8, r8, r2
	uxth	r8, r8
	cmp	r8, r3
	bcc	.L1582
	ldr	r9, .L1594
	mov	r7, #0
	mov	r8, r7
.L1584:
	cmp	sl, #0
	beq	.L1583
	ldr	r3, [r9, #4048]
	add	r2, r3, r7
	ldr	r3, [r3, r7]
	cmp	r3, #0
	bne	.L1583
	ldr	r0, [r2, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L1583:
	add	r8, r8, #1
	add	r7, r7, #36
	uxth	r8, r8
	cmp	r8, r5
	bne	.L1584
	ldr	r1, [sp, #8]
	cmp	r1, #63
	movhi	r3, #0
	movls	r3, #1
	cmp	sl, #0
	moveq	sl, r3
	orrne	sl, r3, #1
	cmp	sl, #0
	beq	.L1565
	ldr	r3, .L1594
	mov	r1, r6
	mov	r2, r8
	ldr	r0, [r3, #4048]
	bl	FlashEraseBlocks
.L1565:
	mov	r0, r4
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1595:
	.align	2
.L1594:
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
.L1597:
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #108
	sub	sp, sp, #108
	ldr	r4, .L1599
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
.L1600:
	.align	2
.L1599:
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
	ldr	r3, .L1603
	ldr	r3, [r3, #168]
	cmp	r0, r3
	bcc	.L1602
	b	FlashTestBlk.part.26
.L1602:
	mov	r0, #0
	bx	lr
.L1604:
	.align	2
.L1603:
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
	ldr	r3, .L1633
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
	ldr	r4, .L1633+4
	mul	fp, fp, r2
	ldr	r2, [r3, #3624]
	ldr	r0, [r4, #240]
	ldrb	r2, [r2, #24]	@ zero_extendqisi2
	str	r0, [sp, #28]
	ldr	r0, .L1633+8
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
	ldr	r4, .L1633
	uxth	r3, r3
	str	r2, [sp, #20]
	str	r3, [sp, #24]
	b	.L1607
.L1629:
	ldr	r3, .L1633+4
	add	r2, r3, r5, asl #1
	ldrh	r6, [r2, #244]
	cmp	r6, #0
	bne	.L1608
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
	b	.L1609
.L1617:
	mvn	r3, #0
	strb	r3, [sp, #34]
	strb	r3, [sp, #35]
	ldr	r3, [sp, #12]
	tst	r3, #1
	beq	.L1610
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
	bne	.L1610
	ldr	r1, [r4, #856]
	add	r2, sp, #35
	mov	r0, r9
	add	r1, r3, r1
	bl	FlashReadSpare
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	ldrb	r2, [sp, #35]	@ zero_extendqisi2
	and	r3, r2, r3
	strb	r3, [sp, #34]
.L1610:
	ldr	r0, [sp, #12]
	tst	r0, #2
	beq	.L1611
	ldr	r3, [r4, #3624]
	mov	r0, r9
	add	r2, sp, #35
	ldrh	r1, [r3, #10]
	ldr	r3, [sl, #3588]
	sub	r1, r1, #1
	add	r1, r1, r3
	add	r1, r1, r6
	bl	FlashReadSpare
.L1611:
	ldr	r3, [r4, #3624]
	ldrb	r3, [r3, #7]	@ zero_extendqisi2
	cmp	r3, #1
	cmpne	r3, #8
	ldrb	r3, [sp, #34]	@ zero_extendqisi2
	bne	.L1612
	cmp	r3, #0
	beq	.L1631
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	rsbs	r0, r0, #1
	movcc	r0, #0
	b	.L1613
.L1612:
	cmp	r3, #255
	bne	.L1631
	ldrb	r0, [sp, #35]	@ zero_extendqisi2
	subs	r0, r0, #255
	movne	r0, #1
	b	.L1613
.L1631:
	mov	r0, #1
.L1613:
	ldr	r2, [sp, #12]
	tst	r2, #4
	beq	.L1614
	ldr	r1, [sl, #3588]
	mov	r0, r9
	add	r1, r6, r1
	bl	SandiskProgTestBadBlock
.L1614:
	cmp	r0, #0
	beq	.L1615
	mov	r1, r5
	mov	r2, r8
	ldr	r0, .L1633+12
	add	r7, r7, #1
	bl	printk
	ldr	r3, [sp, #16]
	and	r0, r8, #31
	mov	ip, #1
	uxth	r7, r7
	mov	r2, r3, lsr #5
	ldr	r3, .L1633+4
	ldr	r3, [r3, #160]
	ldr	r1, [r3, r2, asl #2]
	orr	r1, r1, ip, asl r0
	ldr	r0, [sp, #20]
	str	r1, [r3, r2, asl #2]
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	mul	r3, r3, r0
	cmp	r7, r3
	bgt	.L1616
.L1615:
	ldr	r2, [sp, #8]
	add	r8, r8, #1
	add	r6, r6, r2
.L1609:
	uxth	r3, r8
	str	r3, [sp, #16]
	cmp	r3, fp
	bcc	.L1617
.L1616:
	ldr	r0, .L1633+16
	mov	r1, r5
	mov	r2, r7
	bl	printk
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	ldr	r0, [sp, #20]
	mul	r3, r3, r0
	cmp	r7, r3
	blt	.L1618
	movw	r3, #3064
	mov	r1, #0
	ldrh	r2, [r4, r3]
	ldr	r3, .L1633+4
	mov	r2, r2, asl #9
	ldr	r0, [r3, #160]
	bl	ftl_memset
.L1618:
	cmp	r5, #0
	bne	.L1619
	ldr	sl, .L1633+4
	mov	r8, r5
	mov	r7, r5
	ldrh	r9, [sl, #168]
	b	.L1620
.L1622:
	mov	r0, r6
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1621
	mov	r1, r6
	ldr	r0, .L1633+20
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
.L1621:
	add	r8, r8, #1
	uxth	r8, r8
.L1620:
	add	r6, r8, r9
	ldrb	r3, [r4, #853]	@ zero_extendqisi2
	uxth	r6, r6
	cmp	r3, r6
	bhi	.L1622
	ldr	r6, [sp, #24]
	sub	r9, fp, #50
	ldr	sl, .L1633+4
	mov	r8, #1
	b	.L1623
.L1625:
	mov	r0, r6
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1624
	mov	r1, r6
	ldr	r0, .L1633+20
	bl	printk
	ldr	r3, [sl, #160]
	mov	r2, r6, lsr #5
	and	r0, r6, #31
	ldr	r1, [r3, r2, asl #2]
	orr	r1, r1, r8, asl r0
	str	r1, [r3, r2, asl #2]
.L1624:
	sub	r6, r6, #1
	uxth	r6, r6
.L1623:
	cmp	r6, r9
	bgt	.L1625
	ldr	r3, .L1633+4
	ldrb	r1, [r4, #853]	@ zero_extendqisi2
	ldr	r2, [r3, #168]
	rsb	r2, r2, r1
	cmp	r7, r2
	bcc	.L1619
	movw	r2, #3064
	ldr	r0, [r3, #160]
	ldrh	r2, [r4, r2]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1619:
	mul	r8, fp, r5
	ldr	r7, .L1633+4
	ldr	r6, [sp, #24]
	ldr	r9, [sp, #28]
	add	sl, r7, r5, asl #1
.L1626:
	ldr	r0, .L1633+24
	mov	r1, r5
	mov	r2, r6
	bl	printk
	ldr	r3, [r7, #160]
	b	.L1627
.L1628:
	sub	r6, r6, #1
	uxth	r6, r6
.L1627:
	mov	r1, r6, lsr #5
	and	r2, r6, #31
	ldr	r1, [r3, r1, asl #2]
	mov	r2, r1, lsr r2
	ands	r2, r2, #1
	bne	.L1628
	ldr	r3, .L1633+28
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
	bne	.L1626
.L1608:
	add	r5, r5, #1
	uxtb	r5, r5
.L1607:
	ldrb	r3, [r4, #3762]	@ zero_extendqisi2
	cmp	r3, r5
	bhi	.L1629
	add	sp, sp, #76
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1634:
	.align	2
.L1633:
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
	blt	.L1636
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r8, [r5, r0]
	cmp	r8, #0
	beq	.L1636
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L1636
	mov	r2, #1
	ldr	r1, .L1644
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	mov	r3, #3920
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1637
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1637:
	mov	r5, #0
	ldr	r6, .L1644+4
	b	.L1638
.L1643:
	ldr	r3, [sl, r5, asl #2]
	add	r9, sl, r5, asl #2
	cmp	r8, r3, lsr #10
	bne	.L1639
	ldr	r3, [r6, #-944]
	mov	r1, #1
	ldr	r7, [r6, #-924]
	mov	r2, r1
	ldr	r0, .L1644+8
	str	r3, [r6, #200]
	str	r7, [r6, #204]
	ldr	r3, [sl, r5, asl #2]
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldr	r3, [r6, #192]
	cmn	r3, #1
	beq	.L1640
	ldrh	r3, [r7, #8]
	cmp	r3, r5
	bne	.L1640
	ldrh	r2, [r7, #0]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	beq	.L1641
.L1640:
	mov	r3, #0
	str	r3, [r9, #0]
	ldr	r3, .L1644+4
	ldrh	r2, [r7, #8]
	ldr	r0, .L1644+12
	ldr	r1, [r3, #196]
	bl	printk
	ldr	r3, .L1644
	mov	r2, #1
	str	r2, [r3, #4044]
	b	.L1642
.L1641:
	mov	r0, r4
	mov	r1, r5
	ldr	r2, [r6, #200]
	bl	FtlMapWritePage
.L1639:
	add	r5, r5, #1
	uxth	r5, r5
.L1638:
	ldrh	r3, [r4, #6]
	cmp	r3, r5
	bhi	.L1643
	mov	r0, r8
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L1636:
	ldr	r1, .L1644
	mov	r3, #3920
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bcc	.L1642
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1642:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1645:
	.align	2
.L1644:
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
	ldr	r3, .L1653
	ldr	r5, [r0, #12]
	ldr	sl, [r0, #24]
	ldr	r7, [r3, #4044]
	cmp	r7, #0
	bne	.L1647
	ldrh	r2, [r0, #0]
	movw	r1, #65535
	cmp	r2, r1
	bne	.L1648
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
	b	.L1647
.L1648:
	mov	r2, r2, asl #1
	mov	r1, #255
	ldrh	r8, [r5, r2]
	ldr	r5, .L1653+4
	ldrh	r2, [r0, #2]
	orr	r2, r2, r8, asl #10
	str	r2, [r5, #196]
	ldr	r2, [r5, #-948]
	ldr	r6, [r5, #-924]
	str	r2, [r5, #200]
	ldr	r2, [r0, #28]
	str	r6, [r5, #204]
	str	r2, [r6, #4]
	ldr	r2, .L1653+8
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
	b	.L1649
.L1651:
	ldr	r2, [sl, r3, asl #2]
	cmp	r8, r2, lsr #10
	bne	.L1650
	add	r7, r7, #1
	ldr	r2, [r5, #-948]
	uxth	r7, r7
	str	r3, [r2, r7, asl #3]
	ldr	r2, [r5, #-948]
	ldr	r1, [sl, r3, asl #2]
	add	r2, r2, r7, asl #3
	str	r1, [r2, #4]
.L1650:
	add	r3, r3, #1
	uxth	r3, r3
.L1649:
	ldrh	r2, [r4, #6]
	cmp	r2, r3
	bhi	.L1651
	ldr	r3, .L1653
	ldrb	r2, [r3, #852]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1652
	ldr	r2, .L1653+4
	ldr	r0, [r2, #200]
	movw	r2, #3928
	ldrh	r1, [r3, r2]
	bl	js_hash
	str	r0, [r6, #12]
.L1652:
	mov	r1, #1
	mov	r3, #0
	ldr	r0, .L1653+12
	mov	r2, r1
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
.L1647:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L1654:
	.align	2
.L1653:
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
	ldr	r5, .L1670
	mov	sl, r1
	ldr	r6, .L1670+4
	mov	r9, r2
	mov	r7, #0
.L1669:
	ldr	r3, [r5, #-1816]
	add	r3, r3, #1
	str	r3, [r5, #-1816]
	mov	r3, #3920
	ldrh	r2, [r4, #2]
	ldrh	r3, [r6, r3]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L1657
	ldrh	r2, [r4, #0]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1658
.L1657:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L1658:
	ldr	r3, .L1670+4
	ldr	r1, [r3, #4044]
	cmp	r1, #0
	bne	.L1659
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
	beq	.L1660
	movw	r3, #3928
	ldr	r0, [r5, #200]
	ldrh	r1, [r6, r3]
	bl	js_hash
	str	r0, [fp, #12]
.L1660:
	mov	r1, #1
	ldr	r0, .L1670+8
	mov	r2, r1
	mov	r3, r1
	bl	FlashProgPages
	ldrh	r2, [r4, #2]
	ldr	r3, .L1670
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r4, #2]	@ movhi
	ldr	r3, [r3, #192]
	cmn	r3, #1
	bne	.L1661
	ldr	r0, .L1670+12
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
	bls	.L1669
	ldr	r3, .L1670
	mov	r2, r7
	ldr	r0, .L1670+16
	ldr	r1, [r3, #196]
	bl	printk
	ldr	r3, .L1670+4
	mov	r2, #1
	str	r2, [r3, #4044]
	b	.L1659
.L1661:
	cmp	r3, #0
	strneh	r8, [r4, #40]	@ movhi
	cmp	r2, #1
	beq	.L1665
	cmp	r3, #256
	beq	.L1665
	ldr	r3, [r4, #36]
	cmp	r3, #0
	beq	.L1666
.L1665:
	mov	r3, #0
	str	r3, [r4, #36]
	b	.L1669
.L1666:
	ldr	r3, .L1670
	ldr	r2, [r3, #196]
	ldr	r3, [r4, #24]
	str	r2, [r3, sl, asl #2]
.L1659:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1671:
	.align	2
.L1670:
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
	ldr	r5, .L1673
	mul	r4, r4, r0
	ldr	r0, .L1673+4
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
.L1674:
	.align	2
.L1673:
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
	ldr	r3, .L1679
	ldr	r1, [r3, #4044]
	cmp	r1, #0
	ldmnefd	sp!, {r4, r5, r6, pc}
	ldr	r5, .L1679+4
	sub	r6, r6, #1
	uxth	r6, r6
	ldr	r0, [r5, #-944]
	ldr	r1, [r5, #-924]
	str	r0, [r5, #200]
	str	r1, [r5, #204]
	ldr	r2, [r2, r6, asl #2]
	cmp	r2, #0
	str	r2, [r5, #196]
	beq	.L1677
	mov	r1, #1
	add	r0, r5, #192
	mov	r2, r1
	bl	FlashReadPages
	b	.L1678
.L1677:
	movw	r2, #3928
	mov	r1, #255
	ldrh	r2, [r3, r2]
	bl	ftl_memset
.L1678:
	mov	r0, r4
	mov	r1, r6
	ldr	r2, [r5, #200]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	FtlMapWritePage
.L1680:
	.align	2
.L1679:
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
	ldr	r3, .L1692
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
	bhi	.L1682
	mov	r7, r0, lsr r7
	mov	fp, #0
	ldr	r6, .L1692+4
	mov	r2, r7, asl #2
	str	r2, [sp, #0]
	b	.L1683
.L1689:
	ldr	r3, [r6, #-892]
	mov	r0, r8
	ldr	r2, [sp, #0]
	ldr	sl, [r3, r2]
	movw	r3, #3924
	ldr	r2, .L1692
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
	beq	.L1685
	ldr	r2, .L1692+4
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
	bne	.L1687
	mov	r1, r7
	mov	r2, sl
	ldr	r0, .L1692+8
	bl	printk
	ldr	r0, .L1692+12
	mov	r1, r7
	ldr	r2, [r6, #-940]
	bl	FtlMapWritePage
.L1687:
	ldr	r3, [sp, #4]
	mov	r0, r9
	ldr	r1, [r6, #-940]
	mov	r2, r4, asl #9
	add	r1, r1, r3, asl #9
	bl	memcpy
	b	.L1688
.L1685:
	mov	r0, r9
	mov	r1, sl
	mov	r2, r4, asl #9
	bl	ftl_memset
.L1688:
	ldr	r3, [sp, #0]
	add	r7, r7, #1
	rsb	r5, r4, r5
	add	r8, r8, r4
	add	r9, r9, r4, asl #9
	add	r3, r3, #4
	str	r3, [sp, #0]
.L1683:
	cmp	r5, #0
	bne	.L1689
.L1682:
	mov	r0, fp
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1693:
	.align	2
.L1692:
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
	ldr	r5, .L1698
	mov	r1, #1
	ldr	r2, .L1698+4
	bl	FtlVendorPartRead
	ldr	r4, .L1698+8
	ldr	r3, [r5, #-1700]
	cmp	r3, r4
	beq	.L1695
	ldr	r0, .L1698+4
	mov	r1, #0
	mov	r2, #512
	bl	ftl_memset
	str	r4, [r5, #-1700]
.L1695:
	ldr	r2, [r5, #-1700]
	ldr	r3, .L1698
	cmp	r2, r4
	bne	.L1696
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
	ldr	r2, .L1698+12
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
.L1696:
	ldr	r4, .L1698
	mov	r3, #0
	ldr	r5, .L1698+12
	str	r3, [r4, #-988]
	ldr	r2, [r4, #-1632]
	ldr	r3, .L1698+16
	cmp	r2, r3
	bne	.L1697
	mov	r3, #1
	ldr	r0, .L1698+20
	str	r3, [r5, #3844]
	ldr	r1, .L1698+24
	bl	printk
.L1697:
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
.L1699:
	.align	2
.L1698:
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
	ldr	r4, .L1702
	ldr	r5, .L1702+4
	ldr	r2, [r4, #-908]
	ldrh	r1, [r4, r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #-908]
	ldr	r2, [r3, #0]
	ldr	r3, .L1702+8
	cmp	r2, r3
	beq	.L1701
	ldr	r1, .L1702+12
	ldr	r0, .L1702+16
	bl	printk
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #-908]
	mov	r1, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L1701:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1703:
	.align	2
.L1702:
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
	ldr	r3, .L1714
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
	bhi	.L1705
	mov	r8, r0, lsr r8
	mov	r7, #0
	ldr	sl, .L1714+4
	mov	ip, r8, asl #2
	str	ip, [sp, #8]
	b	.L1706
.L1711:
	ldr	r1, .L1714
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
	ldr	r2, .L1714+4
	uxth	r1, r1
	str	r1, [sp, #12]
	rsb	r5, r1, fp
	uxth	r5, r5
	cmp	r5, r4
	uxthhi	r5, r4
	cmp	r3, #0
	beq	.L1708
	cmp	r5, fp
	beq	.L1708
	str	r3, [sp, #24]
	mov	r1, #1
	ldr	r3, [r2, #-940]
	add	r0, sp, #20
	mov	r2, r1
	str	r3, [sp, #28]
	mov	r3, #0
	str	r3, [sp, #32]
	bl	FlashReadPages
	b	.L1709
.L1708:
	ldr	ip, .L1714
	movw	r3, #3928
	ldr	r0, [r2, #-940]
	mov	r1, #0
	ldrh	r2, [ip, r3]
	bl	ftl_memset
.L1709:
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
	ldr	r0, .L1714+8
	add	r8, r8, #1
	ldr	r2, [sl, #-940]
	add	r9, r9, fp
	bl	FtlMapWritePage
	ldr	ip, [sp, #8]
	add	ip, ip, #4
	str	ip, [sp, #8]
	cmn	r0, #1
	moveq	r7, r0
.L1706:
	cmp	r4, #0
	bne	.L1711
.L1705:
	mov	r0, r7
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1715:
	.align	2
.L1714:
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
	ldr	r3, .L1718
	ldr	r2, .L1718+4
	ldr	r1, [r3, #-1700]
	cmp	r1, r2
	bxne	lr
	ldr	r2, .L1718+8
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
	ldr	r2, .L1718+12
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
	ldr	r2, .L1718+16
	b	FtlVendorPartWrite
.L1719:
	.align	2
.L1718:
	.word	.LANCHOR2
	.word	1179929683
	.word	1342177364
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
	ldr	r3, .L1727
	ldr	r1, .L1727+4
	ldr	r3, [r3, #3844]
	cmp	r3, #0
	moveq	r2, #32
	beq	.L1721
	ldr	r2, [r1, #-1776]
	cmp	r2, #39
	movls	r2, #4
	movhi	r2, #32
.L1721:
	mov	r3, #304
	ldrh	r1, [r1, r3]
	cmp	r1, #31
	addls	r1, r1, #1
	ldrls	r2, .L1727+4
	strlsh	r1, [r2, r3]	@ movhi
	movls	r2, #1
	cmp	r0, #0
	ldr	r3, .L1727+4
	bne	.L1723
	ldr	r1, [r3, #-908]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r2, r2, r0
	cmp	r1, r2
	bcc	.L1724
.L1723:
	ldr	r2, [r3, #-908]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L1727+8
	str	r1, [r2, #0]
	ldr	r1, .L1727+12
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
.L1724:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1728:
	.align	2
.L1727:
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
	ldr	r7, .L1751
	str	r3, [sp, #16]
	ldrh	r3, [r0, #8]
	mov	r0, r9
	str	r3, [sp, #8]
	bl	ftl_memset
	ldr	r3, .L1751+4
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
	b	.L1730
.L1745:
	ldr	r1, [sp, #12]
	cmp	r3, r1
	mov	r3, r3, asl #1
	bne	.L1731
	ldr	r1, [sp, #4]
	mov	r7, #0
	ldr	r2, [sp, #4]
	ldr	r6, .L1751+4
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
	b	.L1732
.L1735:
	ldr	r1, [sp, #8]
	ldr	r0, .L1751+8
	ldrh	r2, [r1, #0]
	mov	r1, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldr	r3, .L1751
	ldrb	r2, [r3, #852]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1733
	ldr	r2, [r6, #204]
	ldr	fp, [r2, #12]
	cmp	fp, #0
	beq	.L1733
	movw	r2, #3928
	ldr	r0, [r6, #200]
	ldrh	r1, [r3, r2]
	bl	js_hash
	cmp	fp, r0
	mvnne	r3, #0
	strne	r3, [r6, #192]
.L1733:
	ldr	r3, [r6, #192]
	cmn	r3, #1
	beq	.L1734
	ldrh	r3, [r5, #8]
	cmp	r3, sl
	bcs	.L1734
	ldrh	r2, [r4, #4]
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	ldreq	r2, [r6, #196]
	streq	r2, [r9, r3, asl #2]
.L1734:
	add	r7, r7, #1
	uxth	r7, r7
.L1732:
	sxth	r3, r7
	cmp	r3, r8
	blt	.L1735
	b	.L1736
.L1731:
	ldr	r6, .L1751+4
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
	beq	.L1748
	ldrh	r2, [r5, #0]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L1748
	ldrh	r2, [r5, #8]
	movw	r3, #64245
	cmp	r2, r3
	bne	.L1748
	b	.L1749
.L1740:
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
	b	.L1738
.L1749:
	mov	r3, #0
.L1738:
	ldrh	r1, [r7, r8]
	sxth	r2, r3
	sub	r1, r1, #1
	cmp	r2, r1
	blt	.L1740
	b	.L1741
.L1744:
	ldr	r1, [sp, #20]
	ldr	r0, .L1751+8
	ldrh	r2, [r1, #0]
	mov	r1, #1
	orr	r3, r3, r2, asl #10
	mov	r2, r1
	str	r3, [r6, #196]
	bl	FlashReadPages
	ldrb	r3, [r7, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1742
	ldr	r3, [r6, #204]
	ldr	fp, [r3, #12]
	cmp	fp, #0
	beq	.L1742
	movw	r3, #3928
	ldr	r0, [r6, #200]
	ldrh	r1, [r7, r3]
	bl	js_hash
	cmp	fp, r0
	mvnne	r3, #0
	strne	r3, [r6, #192]
.L1742:
	ldr	r3, [r6, #192]
	cmn	r3, #1
	beq	.L1743
	ldrh	r3, [r5, #8]
	cmp	r3, sl
	bcs	.L1743
	ldrh	r2, [r4, #4]
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	ldreq	r2, [r6, #196]
	streq	r2, [r9, r3, asl #2]
.L1743:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L1750
.L1748:
	ldr	r6, .L1751+4
	mov	r8, #0
.L1750:
	mov	r2, #3920
	sxth	r3, r8
	ldrh	r2, [r7, r2]
	cmp	r3, r2
	blt	.L1744
.L1741:
	ldr	r2, [sp, #0]
	add	r3, r2, #1
	uxth	r3, r3
	str	r3, [sp, #0]
.L1730:
	ldr	r1, [sp, #0]
	ldr	r2, [sp, #8]
	sxth	r3, r1
	cmp	r3, r2
	blt	.L1745
.L1736:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r1, .L1751
	mov	r3, #3920
	ldrh	r2, [r4, #2]
	ldrh	r3, [r1, r3]
	cmp	r2, r3
	bne	.L1746
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1746:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1752:
	.align	2
.L1751:
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
	ldr	r0, .L1754
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L1755:
	.align	2
.L1754:
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
	ldr	r3, .L1757
	ldr	r0, .L1757+4
	ldrh	r1, [r3, r2]
	movw	r2, #270
	strh	r1, [r0, r2]	@ movhi
	mov	r2, #264
	ldr	r1, .L1757+8
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
.L1758:
	.align	2
.L1757:
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
	ldr	r9, .L1768
	mov	r3, #3056
	ldr	r4, .L1768+4
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
	b	.L1760
.L1766:
	ldr	r5, [sp, #4]
	sub	r3, r8, #12
	mul	fp, r8, r4
	ldr	sl, .L1768+4
	b	.L1761
.L1764:
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
	beq	.L1762
	ldr	r2, [sl, #240]
	ldrh	r1, [r2, #0]
	movw	r2, #61664
	cmp	r1, r2
	bne	.L1762
	ldr	r0, .L1768+8
	mov	r1, r4
	mov	r2, r5
	add	r6, r6, #1
	bl	printk
	ldr	r3, .L1768+4
	uxth	r6, r6
	add	r3, r3, r4, asl #1
	strh	r5, [r3, #244]	@ movhi
	b	.L1763
.L1762:
	sub	r5, r5, #1
	uxth	r5, r5
.L1761:
	cmp	r5, r3
	bgt	.L1764
.L1763:
	ldrb	r3, [r9, #3762]	@ zero_extendqisi2
	add	r4, r4, #1
	cmp	r3, r6
	uxtb	r4, r4
	moveq	r7, #0
.L1760:
	ldrb	r3, [r9, #3762]	@ zero_extendqisi2
	cmp	r3, r4
	bhi	.L1766
	mov	r0, r7
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1769:
	.align	2
.L1768:
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
	ldr	r5, .L1783
	.pad #36
	sub	sp, sp, #36
	ldrh	r6, [r5, r3]
	b	.L1771
.L1773:
	add	r3, r5, r7
	mov	r1, r6
	ldrb	r0, [r3, #3878]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #28]
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #28]
	cmp	r0, #0
	bne	.L1772
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
.L1772:
	add	r7, r7, #1
	uxth	r7, r7
	b	.L1776
.L1781:
	ldr	r8, .L1783+4
	mov	r4, #0
	mov	r7, r4
	movw	fp, #3852
	mov	r9, #36
	movw	sl, #3930
.L1776:
	ldrh	r3, [r5, fp]
	cmp	r3, r7
	bhi	.L1773
	cmp	r4, #0
	beq	.L1774
	ldr	sl, .L1783+4
	mov	r1, r4
	mov	r2, #1
	mov	r7, #0
	mov	r8, r7
	ldr	r0, [sl, #-960]
	bl	FlashReadPages
.L1775:
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
	ldr	r0, .L1783+8
	str	r1, [sp, #16]
	ubfx	r1, r2, #10, #16
	ldr	r3, [r3, #0]
	bl	printk
	cmp	r8, r4
	bne	.L1775
.L1774:
	add	r6, r6, #1
	uxth	r6, r6
.L1771:
	movw	r2, #3862
	ldrh	r3, [r5, r2]
	cmp	r3, r6
	bhi	.L1781
	mov	r5, #0
	ldr	r4, .L1783+4
	ldr	sl, .L1783+12
	b	.L1777
.L1778:
	ldr	r2, [r4, #-1028]
	mov	r1, #1
	ldr	r0, .L1783+16
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
	ldr	r0, .L1783+20
	str	r3, [sp, #12]
	ldr	r3, [r2, #0]
	str	r3, [sp, #16]
	ldr	r3, [r2, #4]
	str	r3, [sp, #20]
	ldr	r2, [r4, #192]
	ldr	r3, [r4, #196]
	bl	printk
.L1780:
	mov	r2, #3920
	sxth	r3, r6
	ldrh	r2, [r8, r2]
	cmp	r3, r2
	blt	.L1778
	add	r5, r5, #1
	uxth	r5, r5
.L1777:
	ldrh	r3, [r4, sl]
	sxth	r7, r5
	ldr	r6, .L1783+4
	cmp	r7, r3
	movlt	r7, r7, asl #1
	movlt	r6, #0
	ldrlt	r8, .L1783
	blt	.L1780
.L1779:
	ldr	r4, .L1783
	movw	r5, #3956
	ldr	r1, [r6, #-1028]
	mov	r2, #2
	ldr	r0, .L1783+24
	ldr	r3, [r4, #3948]
	bl	rknand_print_hex
	ldr	r1, [r6, #-1020]
	ldrh	r3, [r4, r5]
	mov	r2, #4
	ldr	r0, .L1783+28
	bl	rknand_print_hex
	ldr	r0, .L1783+32
	ldr	r1, [r6, #-888]
	mov	r2, #4
	ldrh	r3, [r4, r5]
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	b	rknand_print_hex
.L1784:
	.align	2
.L1783:
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
	ldr	r4, .L1789
	.pad #24
	sub	sp, sp, #24
	mov	r7, r0, asl #10
	mov	r5, #0
	ldr	r8, .L1789+4
	ldr	r3, [r4, #-948]
	str	r3, [r4, #200]
	ldr	r3, [r4, #-924]
	str	r3, [r4, #204]
	b	.L1786
.L1788:
	mov	r1, #1
	ldr	r0, .L1789+8
	mov	r2, r1
	orr	r3, r3, r7
	str	r3, [r4, #196]
	bl	FlashReadPages
	ldr	r3, [r4, #204]
	mov	r1, r6
	ldr	r0, .L1789+12
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
	beq	.L1787
	ldr	r0, .L1789+16
	mov	r2, #4
	ldr	r1, [r4, #-948]
	mov	r3, #768
	bl	rknand_print_hex
.L1787:
	add	r5, r5, #1
	uxth	r5, r5
.L1786:
	mov	r2, #3920
	sxth	r3, r5
	ldrh	r2, [r8, r2]
	cmp	r3, r2
	blt	.L1788
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1790:
	.align	2
.L1789:
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
	ldr	r3, .L1801
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
	ldr	r7, .L1801+4
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
	b	.L1792
.L1798:
	mov	r1, #1
	add	r3, r5, sl
	add	r0, sp, #4
	mov	r2, r1
	mov	r3, r3, asl #10
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L1793
	ldr	r3, [r7, #240]
	ldrh	r2, [r3, #0]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L1793
	cmp	r4, #0
	moveq	r0, r4
	beq	.L1794
	cmp	r6, #0
	ldreq	r3, .L1801+4
	moveq	r0, #1
	beq	.L1795
	b	.L1796
.L1797:
	ldr	r2, [r3, #160]
	ubfx	r1, r6, #5, #16
	and	lr, r6, #31
	add	r6, r6, #1
	ldr	ip, [r2, r1, asl #2]
	uxth	r6, r6
	orr	ip, ip, r0, asl lr
	str	ip, [r2, r1, asl #2]
.L1795:
	ldr	r2, [r3, #168]
	cmp	r6, r2
	bcc	.L1797
.L1796:
	ldr	r3, .L1801+4
	mov	r2, r9
	mov	r0, r4
	ldr	r1, [r3, #160]
	bl	memcpy
	mov	r2, #4
	ldr	r0, .L1801+8
	mov	r1, r4
	mov	r3, r2
	bl	rknand_print_hex
	mov	r0, #0
	b	.L1794
.L1793:
	sub	r5, r5, #1
	uxth	r5, r5
.L1792:
	cmp	r5, r8
	bgt	.L1798
	mvn	r0, #0
.L1794:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1802:
	.align	2
.L1801:
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
	ldr	r3, .L1810
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, r0
	ldr	r3, [r3, #3624]
	ldr	r6, .L1810+4
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	ldrh	r4, [r3, #14]
	ldr	r0, [r6, #148]
	mul	r4, r4, r2
	uxth	r4, r4
	add	r2, r4, #7
	mov	r2, r2, lsr #3
	bl	FlashReadFacBbtData
	cmn	r0, #1
	beq	.L1809
	mov	r2, #0
	mov	r0, r4, lsr #4
	mov	r3, r2
	sub	r4, r4, #1
	mov	r1, #1
	b	.L1805
.L1807:
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
	bcs	.L1809
	add	r2, r2, #1
	uxth	r2, r2
.L1805:
	cmp	r2, r4
	blt	.L1807
	b	.L1804
.L1809:
	mov	r3, #0
.L1804:
	mov	r3, r3, asl #1
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r5, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1811:
	.align	2
.L1810:
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
	ldr	r6, .L1830
	ldr	r5, [r6, #4044]
	cmp	r5, #0
	bne	.L1813
	add	r8, r6, #3984
	bl	FtlBbtMemInit
	add	sl, r8, #12
	bl	FtlLoadFactoryBbt
	ldr	r4, .L1830+4
	b	.L1814
.L1820:
	ldrh	r3, [sl], #2
	movw	r2, #65535
	ldr	r0, [r4, #-948]
	movw	fp, #3916
	ldr	r9, [r4, #-924]
	cmp	r3, r2
	str	r0, [r4, #200]
	str	r9, [r4, #204]
	beq	.L1815
	ldrh	r7, [r6, fp]
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1830+8
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
	b	.L1816
.L1815:
	mov	r1, r5
	bl	FlashGetBadBlockList
	ldr	r0, [r4, #200]
	ldr	r1, [r8, #28]
	bl	FtlBbt2Bitmap
	ldrh	fp, [r6, fp]
.L1818:
	sub	fp, fp, #1
	uxth	fp, fp
.L1829:
	movw	r7, #3916
	ldrh	r0, [r6, r7]
	mla	r0, r0, r5, fp
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1818
	mov	r1, #0
	mov	r2, #16
	strh	fp, [sl, #-2]	@ movhi
	ldr	r0, [r4, #-924]
	bl	ftl_memset
	ldr	r3, .L1830+12
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
	ldr	r3, .L1830+16
	ldrh	r2, [r4, r3]
	mov	r2, r2, asl #2
	bl	memcpy
	mov	r1, #1
	mov	r2, r1
	ldr	r0, .L1830+8
	bl	FlashEraseBlocks
	mov	r1, #1
	mov	r3, r1
	ldr	r0, .L1830+8
	mov	r2, r1
	bl	FlashProgPages
	ldr	r3, [r4, #192]
	cmn	r3, #1
	bne	.L1816
	uxth	r0, r7
	bl	FtlBbmMapBadBlock
	b	.L1829
.L1816:
	uxth	r0, r7
	add	r5, r5, #1
	bl	FtlBbmMapBadBlock
	add	r8, r8, #4
.L1814:
	movw	r3, #3874
	ldrh	r3, [r6, r3]
	cmp	r5, r3
	bcc	.L1820
	mov	r4, #0
	ldr	r7, .L1830
	movw	r6, #3932
	b	.L1821
.L1822:
	mov	r0, r4
	add	r4, r4, #1
	bl	FtlBbmMapBadBlock
	uxth	r4, r4
.L1821:
	ldrh	r3, [r7, r6]
	ldr	r5, .L1830
	cmp	r3, r4
	bhi	.L1822
	movw	r7, #3996
	movw	r6, #65535
	ldrh	r4, [r5, r7]
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L1823
.L1828:
	mov	r0, r4
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1824
	mov	r0, r4
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1825
	mov	r0, r4
	bl	FtlBbmMapBadBlock
	b	.L1824
.L1825:
	mov	r3, #3984
	ldrh	r2, [r5, r3]
	cmp	r2, r6
	streqh	r4, [r5, r3]	@ movhi
.L1826:
	ldrne	r2, .L1830
	movwne	r3, #3988
	strneh	r4, [r2, r3]	@ movhi
	bne	.L1827
.L1824:
	sub	r4, r4, #1
	uxth	r4, r4
.L1823:
	ldrh	r3, [r5, r7]
	sub	r3, r3, #48
	cmp	r4, r3
	bgt	.L1828
.L1827:
	ldr	r4, .L1830
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
.L1813:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1831:
	.align	2
.L1830:
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
	ldr	r4, .L1837
	mov	r6, #12
	ldr	r3, [r4, #-1020]
	ldr	r7, [r3, r0, asl #2]
	cmp	r7, #0
	bne	.L1833
	mul	r6, r6, r1
	ldr	r3, [r4, #-1848]
	ldr	r2, .L1837+4
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
	b	.L1834
.L1833:
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
	beq	.L1835
	mov	r2, r7
	mov	r1, r5
	ldr	r0, .L1837+8
	bl	printk
	ldr	r7, .L1837+4
	mov	r2, #4
	ldr	r0, .L1837+12
	mov	r3, r2
	ldr	r1, [r4, #204]
	bl	rknand_print_hex
	movw	r3, #3956
	ldrh	r3, [r7, r3]
	mov	r2, #4
	ldr	r0, .L1837+16
	ldr	r1, [r4, #-1020]
	bl	rknand_print_hex
	mov	r3, #1
	str	r3, [r7, #4044]
	b	.L1836
.L1835:
	ldr	r3, [r4, #192]
	cmp	r3, #256
	bne	.L1836
	mov	r1, r5
	mov	r2, r7
	ldr	r0, .L1837+20
	bl	printk
	ldr	r3, [r4, #-1848]
	ldr	r0, .L1837+24
	mov	r1, r5
	add	r3, r3, r6
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
.L1836:
	ldr	r3, .L1837
	mov	r1, #0
	ldr	r3, [r3, #-1848]
	add	r2, r3, r6
	str	r1, [r2, #4]
	strh	r5, [r3, r6]	@ movhi
.L1834:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1838:
	.align	2
.L1837:
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
	ldr	r3, .L1852
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
	bcs	.L1840
	movw	r2, #3958
	ldrh	r2, [r3, r2]
	ldr	r3, .L1852+4
	ldr	r9, [r3, #-1848]
	mov	r3, #0
	mov	r6, r3
	b	.L1841
.L1840:
	cmp	sl, #0
	mvn	r0, #0
	streq	r0, [r1, #0]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1847:
	add	r3, r3, #12
	add	r1, r9, r3
	ldrh	r1, [r1, #-12]
	cmp	r1, r7
	bne	.L1843
.L1844:
	mvn	r3, #0
	cmp	sl, #0
	bic	r4, r4, r3, asl r8
	mov	r2, #12
	ldr	r3, .L1852+4
	uxth	r4, r4
	bne	.L1845
	ldr	r3, [r3, #-1848]
	mla	r2, r2, r6, r3
	ldr	r3, [r2, #8]
	ldr	r3, [r3, r4, asl #2]
	str	r3, [r5, #0]
	b	.L1846
.L1845:
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
	ldr	r2, .L1852+8
	strh	r7, [r3, r2]	@ movhi
.L1846:
	ldr	r3, .L1852+4
	mov	r2, #12
	ldr	r3, [r3, #-1848]
	mla	r6, r2, r6, r3
	ldr	r3, [r6, #4]
	cmn	r3, #1
	addne	r3, r3, #1
	strne	r3, [r6, #4]
	movne	r0, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
	b	.L1851
.L1843:
	add	r6, r6, #1
	uxth	r6, r6
.L1841:
	cmp	r6, r2
	bne	.L1847
	bl	select_l2p_ram_region
	mov	r3, #12
	mul	r3, r3, r0
	mov	r6, r0
	add	r2, r9, r3
	ldrh	r1, [r9, r3]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1848
	ldr	r3, [r2, #4]
	cmp	r3, #0
	bge	.L1848
	bl	flush_l2p_region
.L1848:
	mov	r0, r7
	mov	r1, r6
	bl	ftl_load_l2p_region
	b	.L1844
.L1851:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L1853:
	.align	2
.L1852:
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
	ldr	r4, .L1862
	ldr	r6, .L1862+4
	ldr	r2, [r4, #-1616]
	ldr	r3, [r6, #3980]
	cmp	r2, r3
	bcs	.L1855
	mov	r5, #2048
.L1858:
	ldr	r0, [r4, #-1616]
	ldr	r3, [r6, #3980]
	cmp	r0, r3
	bcs	.L1860
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #0]
	ldr	r3, [r4, #-1616]
	cmn	r0, #1
	add	r3, r3, #1
	str	r3, [r4, #-1616]
	beq	.L1857
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
	bne	.L1860
	ldr	r0, [sp, #0]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L1860
.L1857:
	subs	r5, r5, #1
	bne	.L1858
	b	.L1856
.L1855:
	ldr	r3, [r4, #-1832]
	mov	r0, #0
	str	r0, [r4, #-1620]
	str	r0, [r4, #-1616]
	str	r3, [r4, #-1624]
	b	.L1856
.L1860:
	mvn	r0, #0
.L1856:
	add	sp, sp, #40
	ldmfd	sp!, {r4, r5, r6, pc}
.L1863:
	.align	2
.L1862:
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
	ldr	r3, .L1871
	ldr	r2, [r3, #-1620]
	cmp	r2, #0
	beq	.L1865
	b	FtlReadRefresh.part.17
.L1865:
	ldr	r2, .L1871+4
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
	bhi	.L1867
	add	r1, r0, r1
	cmp	r1, r2
	bcc	.L1867
	ldr	r3, .L1871
	ldrb	r3, [r3, #-2040]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1868
.L1867:
	ldr	r3, .L1871
	mov	r1, #1
	str	r2, [r3, #-1624]
	str	r1, [r3, #-1620]
	mov	r1, #0
	str	r1, [r3, #-1616]
.L1868:
	mov	r0, #0
	bx	lr
.L1872:
	.align	2
.L1871:
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
	ldr	r2, .L1905
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
	beq	.L1900
	cmp	r6, #0
	bne	.L1874
	b	.L1875
.L1900:
	mov	r6, #0
.L1874:
	ldr	r2, .L1905+4
	movw	r3, #3918
	ldr	sl, [sp, #20]
	ldrh	r3, [r2, r3]
	cmp	sl, r3
	bne	.L1876
.L1875:
	bl	FtlGcPageVarInit
.L1876:
	ldr	r5, .L1905+8
	mov	r9, #0
	mvn	r3, #0
	str	r3, [sp, #12]
.L1877:
	ldrh	r2, [r4, #0]
	movw	r3, #65535
	mov	sl, #0
	strb	sl, [r4, #8]
	cmp	r2, r3
	beq	.L1878
.L1879:
	str	r4, [sp, #4]
.L1901:
	ldr	r3, .L1905+4
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
	b	.L1880
.L1882:
	ldrh	r0, [r2, #16]
	movw	r1, #65535
	cmp	r0, r1
	beq	.L1881
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
.L1881:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1880:
	cmp	r3, r8
	bne	.L1882
	ldr	r0, [r5, #-960]
	mov	r1, r4
	mov	r2, #0
	mov	r7, #0
	bl	FlashReadPages
	mov	fp, r6
	str	r7, [sp, #8]
	b	.L1883
.L1896:
	ldr	r2, [r5, #-960]
	add	r3, r2, r7
	ldr	r8, [r2, r7]
	ldr	sl, [r3, #4]
	cmp	r8, #0
	str	sl, [sp, #16]
	bne	.L1884
	ldr	r6, [r3, #12]
	movw	r1, #65535
	ldrh	r3, [r6, #0]
	cmp	r3, r1
	beq	.L1885
	ldr	sl, .L1905+4
	ldr	r0, [r6, #8]
	ldr	r3, [sl, #3980]
	cmp	r0, r3
	bls	.L1886
.L1885:
	ldr	r4, [sp, #4]
	mov	sl, #0	@ movhi
	ldr	r2, .L1905+8
	ldrh	r3, [r4, #0]
	ldr	r2, [r2, #-2012]
	mov	r3, r3, asl #1
	strh	sl, [r2, r3]	@ movhi
	b	.L1904
.L1886:
	ldrb	r3, [sl, #852]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1888
	mov	r2, r8
	add	r1, sp, #24
	bl	log2phys
	ldr	r3, [sp, #24]
	ldr	r2, [r6, #12]
	cmp	r3, r2
	bne	.L1888
	cmn	r3, #1
	beq	.L1888
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
	b	.L1889
.L1891:
	ldr	ip, [r1, #8]
	ldr	lr, [ip, r8]
	add	r8, r8, #4
	add	ip, r2, r8
	ldr	ip, [ip, #-4]
	cmp	lr, ip
	beq	.L1890
	ldr	r4, [sp, #4]
	ldr	r2, [sp, #32]
	ldr	r0, .L1905+12
	ldrh	r1, [r4, #0]
	bl	printk
	b	.L1895
.L1890:
	add	r3, r3, #1
.L1889:
	cmp	r3, r0
	bne	.L1891
.L1888:
	ldr	r0, [r6, #12]
	add	r7, r7, #36
	ldr	r1, [sp, #16]
	ldr	r2, [r6, #8]
	bl	FtlGcUpdatePage
	ldr	sl, [sp, #8]
	add	r3, sl, #1
	uxth	r3, r3
	str	r3, [sp, #8]
	b	.L1883
.L1884:
	ldr	r4, [sp, #4]
	ldr	r2, [sp, #16]
	ldr	r0, .L1905+16
	ldrh	r1, [r4, #0]
	bl	printk
	ldr	r3, .L1905+4
	ldr	r2, [r3, #3844]
	cmp	r2, #0
	bne	.L1892
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1893
.L1892:
	ldrh	r1, [r4, #0]
	ldr	r2, [r3, #4052]
	mov	r3, r1, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #159
	bls	.L1894
.L1893:
	ldr	r3, [r5, #-960]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	bne	.L1895
.L1894:
	ldr	r3, [r5, #-960]
	add	r7, r3, r7
	ldr	r7, [r7, #4]
	str	r7, [sp, #12]
.L1895:
	ldrh	r3, [r4, #0]
	mov	r1, #0	@ movhi
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
.L1904:
	ldrh	r0, [r4, #0]
	mov	r6, #0
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r4, #0]	@ movhi
	bl	FtlGcPageVarInit
	b	.L1877
.L1883:
	ldr	r3, [sp, #8]
	cmp	r3, r4
	bne	.L1896
	ldr	sl, [sp, #20]
	add	r9, r9, #1
	add	r6, fp, #1
	cmp	r9, sl
	uxth	r6, r6
	bcc	.L1897
	ldr	r2, .L1905
	movw	r3, #3224
	movw	r0, #65535
	ldrh	r1, [r2, r3]
	cmp	r1, r0
	beq	.L1897
	add	r1, r1, r9
	strh	r1, [r2, r3]	@ movhi
	ldr	r2, .L1905+4
	movw	r3, #3918
	ldrh	r3, [r2, r3]
	cmp	r3, r6
	bhi	.L1898
.L1897:
	ldr	r2, .L1905+4
	movw	r3, #3918
	ldrh	r3, [r2, r3]
	cmp	r3, r6
	bhi	.L1901
	ldr	r4, [sp, #4]
.L1878:
	ldr	r2, .L1905
	movw	r3, #3224
	mvn	r1, #0
	strh	r6, [r4, #2]	@ movhi
	mov	r0, r4
	strh	r1, [r2, r3]	@ movhi
	mov	r2, #0
	mov	r1, r6
	strb	r2, [r4, #6]
	bl	ftl_sb_update_avl_pages
.L1898:
	ldr	r0, [sp, #12]
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L1906:
	.align	2
.L1905:
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
	ldr	r3, .L1913
	ldr	r1, [r3, #-2012]
	mov	r5, r0, asl #1
	ldrh	r2, [r1, r5]
	cmp	r2, #0
	addne	r2, r2, #1
	strneh	r2, [r1, r5]	@ movhi
	bne	.L1909
	ldr	r4, [r3, #-2000]
	cmp	r4, #0
	beq	.L1909
	ldr	r1, .L1913+4
	mov	ip, #6
	ldrh	lr, [r3, r1]
	ldr	r3, [r3, #-2020]
	ldr	r1, .L1913+8
	rsb	r4, r3, r4
	mov	r4, r4, asr #1
	mul	r4, r1, r4
	movw	r1, #65535
	uxth	r4, r4
	b	.L1910
.L1912:
	cmp	r4, r0
	bne	.L1911
	ldr	r6, .L1913
	mov	r1, r4
	sub	r0, r6, #2000
	bl	List_remove_node
	ldr	r3, .L1913+4
	mov	r0, r4
	ldrh	r2, [r6, r3]
	sub	r2, r2, #1
	strh	r2, [r6, r3]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r3, [r6, #-2012]
	ldrh	r2, [r3, r5]
	add	r2, r2, #1
	strh	r2, [r3, r5]	@ movhi
	b	.L1909
.L1911:
	mul	r4, ip, r4
	ldrh	r4, [r3, r4]
	cmp	r4, r1
	beq	.L1909
	add	r2, r2, #1
	uxth	r2, r2
.L1910:
	cmp	r2, lr
	bne	.L1912
.L1909:
	mov	r0, r7
	add	r1, sp, #4
	mov	r2, #1
	bl	log2phys
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L1914:
	.align	2
.L1913:
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
	ldr	r4, .L1921
	.pad #32
	sub	sp, sp, #32
	ldr	r0, .L1921+4
	mov	r5, #0
	bl	printk
	ldr	r7, .L1921+8
	mov	r6, r4
	b	.L1916
.L1920:
	mov	r0, r5
	add	r1, sp, #28
	mov	r2, #0
	bl	log2phys
	movs	r3, r5, asl #21
	bne	.L1917
	ldr	r0, .L1921+12
	mov	r1, r5
	ldr	r2, [sp, #28]
	bl	printk
.L1917:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L1918
	str	r3, [r4, #196]
	mov	r2, #0
	ldr	r3, [r4, #-948]
	mov	r1, #1
	ldr	r8, [r4, #-924]
	ldr	r0, .L1921+16
	str	r3, [r4, #200]
	str	r5, [r4, #208]
	str	r8, [r4, #204]
	str	r2, [r4, #192]
	bl	FlashReadPages
	ldr	r3, [r4, #192]
	cmn	r3, #1
	cmpne	r3, #256
	beq	.L1919
	ldr	r3, [r8, #8]
	cmp	r3, r5
	beq	.L1918
.L1919:
	ldr	r3, [r6, #204]
	ldr	r2, [r6, #200]
	ldr	r0, .L1921+20
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
.L1918:
	add	r5, r5, #1
.L1916:
	ldr	r3, [r7, #3980]
	cmp	r5, r3
	bcc	.L1920
	add	sp, sp, #32
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1922:
	.align	2
.L1921:
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
	beq	.L1924
	ldrh	r3, [r0, #2]
	ldr	r1, .L2013
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
	bne	.L1926
	b	.L2010
.L1927:
	add	r3, r3, #1
	uxth	r3, r3
	add	r1, r4, r3, asl #1
	ldrh	r0, [r1, #16]
.L1926:
	cmp	r0, r2
	beq	.L1927
	ldrb	r1, [r4, #8]	@ zero_extendqisi2
	cmp	r1, #1
	bne	.L1928
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1929
	ldr	r3, .L2013
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L2003
	add	r3, r3, r0, asl #1
	movw	r2, #3076
	ldrh	r8, [r3, r2]
	b	.L1995
.L1928:
	mov	r1, #0
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r7, r0
	beq	.L1929
.L2003:
	mov	r8, r7
.L1995:
	ldr	r3, .L2013
	movw	r2, #3852
	mov	r5, #0
	str	r4, [sp, #44]
	movw	ip, #65535
	ldrh	r9, [r3, r2]
	ldr	r2, .L2013+4
	ldr	sl, [r2, #-960]
	ldr	r6, [r2, #-1160]
	movw	r2, #3930
	ldrh	lr, [r3, r2]
	mov	r2, r4
	mov	r3, r5
	b	.L1931
.L1929:
	mov	r3, #0
	strh	r3, [r4, #2]	@ movhi
	mov	r2, r3	@ movhi
.L2010:
	strb	r2, [r4, #6]
	b	.L1924
.L1933:
	ldrh	r0, [r2, #16]
	cmp	r0, ip
	beq	.L1932
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
.L1932:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1931:
	cmp	r3, r9
	bne	.L1933
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	cmp	r3, #1
	movne	r3, #0
	bne	.L2004
	ldr	r3, .L2013
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	adds	r3, r3, #0
	movne	r3, #1
.L2004:
	ldr	r6, .L2013+4
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
	b	.L1935
.L1940:
	mov	r3, #36
	ldr	r1, [r6, #-960]
	mul	r2, r3, sl
	add	r3, r1, r2
	ldr	r2, [r1, r2]
	cmp	r2, #0
	bne	.L1936
	ldr	r3, [r3, #12]
	ldr	fp, [r3, #4]
	cmn	fp, #1
	beq	.L1937
	ldr	r1, [r6, #-1796]
	mov	r0, fp
	str	r3, [sp, #4]
	bl	ftl_cmp_data_ver
	ldr	r3, [sp, #4]
	cmp	r0, #0
	addne	fp, fp, #1
	strne	fp, [r6, #-1796]
.L1937:
	ldr	r3, [r3, #0]
	cmn	r3, #1
	bne	.L1939
	b	.L1938
.L1936:
	ldr	r1, [r3, #4]
	uxth	fp, r8
	ldr	r0, .L2013+8
	bl	printk
	ldrh	r3, [r4, #0]
	movw	lr, #306
	str	fp, [sp, #12]
	strh	r3, [r6, lr]	@ movhi
.L1939:
	add	sl, sl, #1
.L1935:
	uxth	r9, sl
	cmp	r9, r5
	bcc	.L1940
.L1938:
	ldr	r3, .L2013+4
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
	ldr	r3, .L2013
	cmp	r2, #1
	str	r0, [sp, #24]
	bne	.L1943
	ldrb	r1, [r3, #3072]	@ zero_extendqisi2
	cmp	r1, #0
	addeq	r7, r3, r7, asl #1
	movweq	r1, #3076
	ldreqh	r7, [r7, r1]
.L1943:
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
	beq	.L2011
	ldr	r3, [sp, #16]
	ldr	fp, [sp, #12]
	sub	r6, r3, #1
	movw	r3, #65535
	cmp	fp, r3
	bne	.L1946
	cmp	r2, #0
	bne	.L1947
.L1946:
	ldr	r2, .L2013+4
	uxth	r3, r8
	ldr	fp, [sp, #20]
	movw	ip, #65535
	mov	sl, r3
	ldr	r8, .L2013+4
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
	b	.L1950
.L1952:
	ldrh	r0, [r1, #16]
	cmp	r0, ip
	beq	.L1951
	ldr	r6, [sp, #28]
	mov	r3, #36
	orr	r0, r5, r0, asl #10
	mla	lr, r3, r4, r6
	add	r4, r4, #1
	uxth	r4, r4
	str	r0, [lr, #4]
.L1951:
	add	r2, r2, #1
	add	r1, r1, #2
	uxth	r2, r2
.L1963:
	ldr	lr, [sp, #16]
	cmp	r2, lr
	bne	.L1952
	ldr	r3, [sp, #40]
	mov	r1, r4
	ldr	r0, [r8, #-960]
	ldr	r2, [sp, #32]
	stmib	sp, {r3, ip}
	ldr	r6, [sp, #0]
	bl	FlashReadPages
	ldr	r2, .L2013
	add	r0, r8, r5, asl #1
	mov	r1, #0
	str	r0, [sp, #28]
	ldmib	sp, {r3, ip}
	ldrb	r2, [r2, #3072]	@ zero_extendqisi2
	str	r2, [sp, #16]
	ldr	r2, [r8, #-960]
	b	.L1953
.L1961:
	ldr	r0, [r2, #0]
	cmp	r0, #0
	bne	.L1954
	ldr	r0, [r2, #12]
	ldrh	lr, [r0, #0]
	cmp	lr, ip
	beq	.L1955
	ldr	r0, [r0, #4]
	cmn	r0, #1
	beq	.L1955
	cmn	fp, #1
	ldr	r9, [r8, #-996]
	str	r0, [r8, #-996]
	bne	.L1955
	ldr	lr, [sp, #28]
	sub	r0, lr, #880
	ldrh	r0, [r0, #0]
	cmp	r0, ip
	bne	.L1956
	ldr	r0, [sp, #16]
	cmp	r0, #0
	beq	.L1955
.L1956:
	cmp	r9, r7
	mvneq	fp, #0
	movne	fp, r9
	b	.L1955
.L1954:
	mov	r4, r6
	mov	r6, r7
	ldrh	r1, [r4, #0]
	mov	r7, r3
	ldr	r3, .L2013+4
	movw	r2, #306
	strh	r1, [r3, r2]	@ movhi
	ldrb	r2, [r4, #8]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L1947
	add	r5, r3, r5, asl #1
	movw	r2, #65535
	sub	r5, r5, #880
	ldrh	r1, [r5, #0]
	cmp	r1, r2
	bne	.L1957
	cmn	fp, #1
	bne	.L2006
.L1958:
	ldr	fp, [sp, #12]
	cmp	fp, r6
	beq	.L1959
.L2006:
	str	fp, [r3, #-996]
	b	.L1947
.L1959:
	ldr	r2, [r3, #-996]
	b	.L2012
.L1957:
	cmp	r9, r6
	beq	.L1960
	cmn	r9, #1
	strne	r9, [r3, #-996]
	b	.L1947
.L1960:
	ldr	r2, [r3, #-996]
	cmp	r2, r6
	beq	.L1947
.L2012:
	sub	r2, r2, #1
	b	.L2007
.L1955:
	add	r1, r1, #1
	add	r2, r2, #36
	uxth	r1, r1
.L1953:
	cmp	r1, r4
	bne	.L1961
	add	r5, r5, #1
	uxth	r5, r5
.L1950:
	cmp	r5, sl
	bhi	.L1962
	ldr	r1, .L2013
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
	b	.L1963
.L1962:
	mov	r4, r6
	mov	r6, r7
	mov	r7, r3
	ldr	r3, .L2013+4
	mvn	r2, #0
.L2007:
	str	r2, [r3, #-996]
.L1947:
	ldr	r5, .L2013+4
	mov	r3, #308
	ldr	r8, [sp, #20]
	mov	r2, #1
	str	r7, [sp, #16]
	strh	r2, [r5, r3]	@ movhi
.L1964:
	ldr	r3, .L2013
	movw	r2, #3852
	mov	r7, #0
	ldr	lr, [r5, #-960]
	mov	fp, #36
	ldrh	sl, [r3, r2]
	mov	r2, r4
	ldrb	ip, [r3, #3072]	@ zero_extendqisi2
	mov	r3, r7
	b	.L1965
.L1968:
	ldrh	r0, [r2, #16]
	movw	r1, #65535
	cmp	r0, r1
	beq	.L1966
	mla	r1, fp, r7, lr
	orr	r0, r8, r0, asl #10
	str	r0, [r1, #4]
	ldrb	r9, [r4, #8]	@ zero_extendqisi2
	cmp	r9, #1
	bne	.L1967
	cmp	ip, #0
	orrne	r0, r0, #-2147483648
	strne	r0, [r1, #4]
.L1967:
	add	r7, r7, #1
	uxth	r7, r7
.L1966:
	add	r3, r3, #1
	add	r2, r2, #2
	uxth	r3, r3
.L1965:
	cmp	r3, sl
	bne	.L1968
	mov	r1, r7
	ldr	r0, [r5, #-960]
	ldr	r2, [sp, #32]
	bl	FlashReadPages
	mov	r3, #36
	mul	r3, r3, r7
	mov	r7, #0
	str	r3, [sp, #40]
	b	.L1969
.L1990:
	ldr	r9, [r5, #-960]
	add	r9, r9, r7
	ldr	sl, [r9, #4]
	ubfx	r0, sl, #10, #16
	str	sl, [sp, #60]
	bl	P2V_plane
	ldr	r3, [sp, #20]
	cmp	r8, r3
	bcc	.L1970
	ldr	fp, [sp, #36]
	mov	r1, r3
	cmp	r0, fp
	movcs	r3, #0
	movcc	r3, #1
	cmp	r8, r1
	movne	r3, #0
	cmp	r3, #0
	bne	.L1970
	ldr	r3, [sp, #24]
	ldr	fp, [sp, #16]
	cmp	r0, r3
	cmpeq	r8, fp
	beq	.L2002
	ldr	r3, [r9, #0]
	cmn	r3, #1
	beq	.L1972
	ldr	sl, [r9, #12]
	movw	r3, #61589
	ldrh	r2, [sl, #0]
	cmp	r2, r3
	ldrneh	r0, [r4, #0]
	bne	.L2008
	ldr	r6, [sl, #4]
	cmn	r6, #1
	beq	.L1974
	ldr	r1, [r5, #-1796]
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	addne	r3, r6, #1
	strne	r3, [r5, #-1796]
.L1974:
	ldr	r9, [sl, #8]
	add	r1, sp, #56
	ldr	r3, [sl, #12]
	mov	r2, #0
	mov	r0, r9
	str	r3, [sp, #52]
	bl	log2phys
	ldr	r1, [r5, #-996]
	cmn	r1, #1
	beq	.L1975
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1975
	ldr	r3, [sp, #52]
	cmn	r3, #1
	beq	.L1976
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
	bne	.L1977
	b	.L1978
.L1976:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	bne	.L1970
	mov	r0, r9
	add	r1, sp, #52
	mov	r2, #1
	bl	log2phys
	b	.L1970
.L1977:
	ldr	r3, [sp, #12]
	ldr	sl, [r3, #8]
	cmp	sl, r9
	bne	.L1978
	ldr	r0, [r5, #-996]
	ldr	r1, [sp, #28]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1978
	ldr	r3, [sp, #56]
	ldr	r2, [sp, #60]
	cmp	r3, r2
	beq	.L1982
.L1979:
	ldr	r2, [sp, #52]
	cmp	r3, r2
	beq	.L1978
	cmn	r3, #1
	streq	r3, [fp, #0]
	beq	.L1981
	ldr	lr, [fp, #12]
	mov	r0, fp
	str	r3, [fp, #4]
	mov	r1, #1
	mov	r2, #0
	str	lr, [sp, #12]
	bl	FlashReadPages
.L1981:
	ldr	r3, [r5, #-960]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	beq	.L1982
	ldr	r3, [sp, #12]
	ldr	r0, [r5, #-996]
	ldr	r9, [r3, #4]
	mov	r1, r9
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1982
	ldr	r0, [sp, #28]
	mov	r1, r9
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1978
.L1982:
	mov	r0, sl
	ldr	r1, [sp, #52]
	bl	FtlReUsePrevPpa
.L1978:
	mvn	r3, #0
	str	r3, [sp, #52]
	b	.L1984
.L1975:
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #56]
	cmp	r2, r3
	beq	.L1984
	ldr	r2, .L2013
	ldr	r3, [sp, #52]
	ldr	r2, [r2, #3868]
	cmp	r2, r3, lsr #10
	bhi	.L1985
	cmn	r3, #1
	bne	.L1970
.L1985:
	mov	r0, r9
	add	r1, sp, #60
	mov	r2, #1
	bl	log2phys
	ldr	fp, [sp, #56]
	cmn	fp, #1
	beq	.L1984
	ldr	r3, [sp, #52]
	cmp	fp, r3
	beq	.L1984
	ubfx	r0, fp, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L2013+12
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1986
	ldr	r3, .L2013+16
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L1986
	ldr	r3, .L2013+20
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	bne	.L1984
.L1986:
	ldr	lr, .L2013+4
	mov	r1, #1
	mov	r2, #0
	ldr	r0, [lr, #-960]
	str	fp, [r0, #4]
	ldr	sl, [r0, #12]
	bl	FlashReadPages
	ldr	r0, .L2013+4
	ldr	r1, [sl, #4]
	ldr	r3, [r0, #-960]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	beq	.L1984
	mov	r0, r6
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L1984
	mov	r0, r9
	add	r1, sp, #56
	mov	r2, #1
	bl	log2phys
.L1984:
	ldr	r0, [sp, #52]
	cmn	r0, #1
	beq	.L1970
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r5, #-2012]
	mov	r3, r0, asl #1
	mov	r1, r0
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1987
.L2008:
	bl	decrement_vpc_count
	b	.L1970
.L1987:
	ldr	r0, .L2013+24
	bl	printk
	b	.L1970
.L1972:
	ldrh	r3, [r4, #0]
	movw	r1, #306
	mov	r2, r6
	ldr	r0, .L2013+28
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
	beq	.L2009
.L1989:
	cmp	r3, r6
	bls	.L1970
.L2009:
	str	r6, [r5, #-996]
.L1970:
	add	r7, r7, #36
.L1969:
	ldr	r3, [sp, #40]
	cmp	r7, r3
	bne	.L1990
	ldrb	r3, [r4, #8]	@ zero_extendqisi2
	add	r8, r8, #1
	cmp	r3, #1
	uxth	r8, r8
	bne	.L1991
	ldr	r3, .L2013
	ldrb	r2, [r3, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L1991
	mov	r2, #3920
	ldr	fp, [sp, #16]
	ldrh	r3, [r3, r2]
	cmp	fp, r8
	cmpeq	r3, r8
	beq	.L2002
.L1991:
	ldr	r3, .L2013
	movw	r2, #3918
	ldrh	r2, [r3, r2]
	cmp	r8, r2
	bne	.L1964
	mov	r2, #0
	movw	r1, #3852
	strh	r2, [r4, #4]	@ movhi
	ldrh	r1, [r3, r1]
	mov	r3, r2
	strh	r8, [r4, #2]	@ movhi
	movw	r2, #65535
	ldr	r0, [sp, #44]
	b	.L1992
.L1994:
	add	r0, r0, #2
	ldrh	ip, [r0, #14]
	cmp	ip, r2
	strneb	r3, [r4, #6]
	bne	.L1924
.L1993:
	add	r3, r3, #1
	uxth	r3, r3
.L1992:
	cmp	r3, r1
	bne	.L1994
	b	.L1924
.L2002:
.L1971:
	ldr	r7, [sp, #16]
	mov	r0, r4
	ldr	r3, [sp, #24]
	mov	r1, r7
	strh	r7, [r4, #2]	@ movhi
	mov	r2, r3
	strb	r3, [r4, #6]
.L2011:
	bl	ftl_sb_update_avl_pages
.L1924:
	mov	r0, #0
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2014:
	.align	2
.L2013:
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
	ldr	r5, .L2024
	ldr	r3, [r5, #4044]
	cmp	r3, #0
	bne	.L2015
	ldr	r4, .L2024+4
	ldr	r3, .L2024+8
	mov	r2, r4
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	beq	.L2017
	ldrb	r1, [r4, #-1984]	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L2017
	movw	r0, #3918
	ldrb	r1, [r4, #-1985]	@ zero_extendqisi2
	ldrh	r0, [r5, r0]
	mul	r1, r0, r1
	cmp	r3, r1
	beq	.L2017
	ldrb	r8, [r4, #-1982]	@ zero_extendqisi2
	movw	r3, #3852
	ldr	r7, [r5, #3980]
	cmp	r8, #0
	ldrh	r6, [r5, r3]
	bne	.L2015
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
	beq	.L2018
	add	r0, sp, #4
	mov	r1, #1
	mov	r2, r8
	bl	FlashReadPages
	b	.L2019
.L2018:
	movw	r3, #3928
	mov	r1, #255
	ldrh	r2, [r5, r3]
	bl	ftl_memset
.L2019:
	ldr	r3, .L2024+12
	mov	r6, r6, asl #2
	ldr	r5, .L2024+4
	ldr	sl, .L2024+8
	strh	r3, [r4, #0]	@ movhi
	ldr	r8, .L2024+16
	b	.L2020
.L2023:
	ldrh	r3, [r5, sl]
	cmp	r3, #0
	beq	.L2021
	ldr	r3, [sp, #8]
	sub	r6, r6, #1
	str	r7, [r4, #8]
	ldr	r0, .L2024+20
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
.L2020:
	cmp	r6, #0
	bne	.L2023
.L2021:
	ldr	r3, .L2024+4
	mov	r2, #1
	strb	r2, [r3, #-1982]
	b	.L2015
.L2017:
	mov	r3, #0
	strb	r3, [r2, #-1982]
.L2015:
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L2025:
	.align	2
.L2024:
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
	ldr	r7, .L2030
	movw	r6, #3958
	ldr	r5, .L2030+4
	b	.L2027
.L2029:
	ldr	r3, [r5, #-1848]
	mov	r2, #12
	mla	r3, r2, r4, r3
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L2028
	mov	r0, r4
	bl	flush_l2p_region
.L2028:
	add	r4, r4, #1
	uxth	r4, r4
.L2027:
	ldrh	r3, [r7, r6]
	cmp	r3, r4
	bhi	.L2029
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2031:
	.align	2
.L2030:
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
	ldr	r3, .L2044
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r3, [r3, #4044]
	ldrh	r5, [r0, #0]
	cmp	r3, #0
	bne	.L2033
	movw	r3, #65535
	cmp	r5, r3
	beq	.L2034
	ldr	r3, .L2044+4
	mov	r0, r5
	ldr	r2, [r3, #-2012]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2035
	bl	INSERT_DATA_LIST
	b	.L2034
.L2035:
	bl	INSERT_FREE_LIST
.L2034:
	mov	r3, #0
	strb	r3, [r4, #8]
	ldr	r3, .L2044+8
	cmp	r4, r3
	beq	.L2036
	ldr	r2, .L2044
	mov	r3, #3872
	ldrh	r3, [r2, r3]
	cmp	r3, #1
	beq	.L2036
	ldrb	r2, [r2, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2037
.L2036:
	mov	r3, #1
	strb	r3, [r4, #8]
	b	.L2038
.L2037:
	ldr	r2, .L2044+12
	cmp	r4, r2
	bne	.L2038
	cmp	r3, #3
	ldr	r3, .L2044+4
	beq	.L2039
	ldr	r2, [r3, #-1704]
	cmp	r2, #1
	bne	.L2040
.L2039:
	mov	r2, #1
	strb	r2, [r3, #-1984]
.L2040:
	ldr	r2, .L2044
	ldr	r2, [r2, #3844]
	cmp	r2, #0
	beq	.L2038
	ldr	r3, [r3, #-1776]
	cmp	r3, #39
	ldrls	r3, .L2044+4
	movls	r2, #1
	strlsb	r2, [r3, #-1984]
.L2038:
	ldr	r3, .L2044+16
	ldr	r6, .L2044+4
	ldrh	r0, [r6, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L2041
	cmp	r5, r0
	bne	.L2042
	ldr	r2, [r6, #-2012]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2043
.L2042:
	bl	update_vpc_list
.L2043:
	ldr	r3, .L2044+16
	mvn	r2, #0
	strh	r2, [r6, r3]	@ movhi
.L2041:
	mov	r0, r4
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2033:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2045:
	.align	2
.L2044:
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
	ldr	r6, .L2054
	ldr	r1, .L2054+4
	ldr	r0, .L2054+8
	bl	printk
	movw	r3, #3862
	ldrh	r2, [r6, r3]
	mov	r1, #0
	ldr	r5, .L2054+12
	mov	r2, r2, asl #1
	ldr	r0, [r5, #-904]
	bl	ftl_memset
	b	.L2047
.L2049:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L2048
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, [r5, #-904]
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	add	r2, r2, #1
	strh	r2, [r3, r0]	@ movhi
.L2048:
	add	r4, r4, #1
.L2047:
	ldr	r3, [r6, #3980]
	cmp	r4, r3
	bcc	.L2049
	mov	r4, #0
	ldr	r7, .L2054
	movw	r8, #3860
	ldr	r5, .L2054+12
	b	.L2050
.L2053:
	ldr	r3, [r5, #-2012]
	mov	r6, r4, asl #1
	ldrh	r2, [r3, r6]
	ldr	r3, [r5, #-904]
	ldrh	r3, [r3, r6]
	cmp	r2, r3
	beq	.L2051
	movw	r1, #65535
	cmp	r2, r1
	beq	.L2051
	ldr	r0, .L2054+16
	mov	r1, r4
	bl	printk
	ldr	r3, .L2054+20
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L2051
	ldr	r3, .L2054+24
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L2051
	ldr	r3, .L2054+28
	ldrh	r3, [r5, r3]
	cmp	r3, r4
	beq	.L2051
	ldr	r3, [r5, #-2012]
	ldrh	r2, [r3, r6]
	cmp	r2, #0
	ldr	r2, [r5, #-904]
	ldrh	r2, [r2, r6]
	strh	r2, [r3, r6]	@ movhi
	beq	.L2051
.L2052:
	mov	r0, r4
	bl	update_vpc_list
.L2051:
	add	r4, r4, #1
	uxth	r4, r4
.L2050:
	ldrh	r3, [r7, r8]
	cmp	r3, r4
	bhi	.L2053
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L2055:
	.align	2
.L2054:
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
	ldr	r3, .L2079
	movw	r2, #3918
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, lr}
	ldrh	r1, [r3, r2]
	ldr	r2, [r3, #4044]
	cmp	r2, #0
	bne	.L2077
	ldr	ip, .L2079+4
	ldr	r3, .L2079+8
	ldrh	r4, [r3, ip]
	movw	ip, #65535
	cmp	r4, ip
	beq	.L2058
	cmp	r0, #0
	beq	.L2059
	ldr	lr, .L2079+12
	movw	r0, #3224
	ldrh	r5, [lr, r0]
	cmp	r5, ip
	movne	r1, #2
	bne	.L2059
	strh	r2, [lr, r0]	@ movhi
	ldr	r2, .L2079+16
	ldrh	r3, [r3, r2]
	cmp	r3, #17
	movhi	r1, #2
.L2059:
	ldr	r0, .L2079+20
	bl	FtlGcScanTempBlk
	cmn	r0, #1
	str	r0, [sp, #4]
	beq	.L2060
	ldr	r3, .L2079
	mov	r4, r4, asl #1
	ldr	r3, [r3, #4052]
	ldrh	r2, [r3, r4]
	cmp	r2, #4
	bls	.L2061
	sub	r2, r2, #5
	mov	r0, #1
	strh	r2, [r3, r4]	@ movhi
	bl	FtlEctTblFlush
.L2061:
	ldr	r3, .L2079+8
	ldr	r2, [r3, #-1000]
	cmp	r2, #0
	bne	.L2062
	ldr	r2, [r3, #-1604]
	ldr	r0, [sp, #4]
	add	r2, r2, #1
	str	r2, [r3, #-1604]
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L2062:
	ldr	r3, .L2079+8
	mov	r2, #0
	str	r2, [r3, #-1000]
	b	.L2074
.L2060:
	ldr	r2, .L2079+12
	movw	r3, #3224
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2074
.L2058:
	ldr	r6, .L2079+8
	movw	r3, #65535
	ldr	r5, .L2079+4
	mov	r4, #0
	str	r4, [r6, #-1000]
	ldrh	r2, [r6, r5]
	cmp	r2, r3
	beq	.L2057
	bl	FtlCacheWriteBack
	ldr	ip, .L2079
	movw	r0, #3918
	ldrb	r1, [r6, #-1889]	@ zero_extendqisi2
	ldrh	r3, [r6, r5]
	ldrh	r0, [ip, r0]
	ldr	r2, [r6, #-2012]
	mov	r3, r3, asl #1
	mul	r1, r0, r1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, .L2079+24
	ldrh	r2, [r6, r3]
	ldr	r3, [r6, #-1812]
	add	r3, r2, r3
	str	r3, [r6, #-1812]
	b	.L2063
.L2066:
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
	bne	.L2064
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r1, r5, #4
	mov	r2, #1
	mov	r7, r0
	ldr	r0, [r5, #8]
	bl	log2phys
	mov	r0, r7
	b	.L2078
.L2064:
	ldr	r3, [r5, #4]
	cmp	r0, r3
	beq	.L2065
	ldr	r3, .L2079+4
	ldrh	r0, [r6, r3]
.L2078:
	bl	decrement_vpc_count
.L2065:
	add	r4, r4, #1
	uxth	r4, r4
.L2063:
	ldr	r3, .L2079+24
	ldr	r5, .L2079+8
	ldrh	r3, [r6, r3]
	cmp	r3, r4
	bhi	.L2066
	movw	r0, #65535
	ldr	r4, .L2079+4
	bl	decrement_vpc_count
	ldr	r3, .L2079
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2067
	ldr	r0, .L2079+28
	ldrh	r1, [r5, r4]
	bl	printk
.L2067:
	ldrh	r0, [r5, r4]
	ldr	r2, [r5, #-2012]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2068
	bl	INSERT_DATA_LIST
	b	.L2069
.L2068:
	bl	INSERT_FREE_LIST
.L2069:
	ldr	r5, .L2079+8
	mvn	r6, #0
	ldr	r3, .L2079+4
	mov	r4, #0
	strh	r6, [r5, r3]	@ movhi
	add	r3, r3, #760
	strh	r4, [r5, r3]	@ movhi
	sub	r3, r3, #12
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	r3, .L2079+32
	mov	r0, r5
	ldr	r2, .L2079+16
	strh	r6, [r5, r3]	@ movhi
	ldr	r3, .L2079
	ldr	r3, [r3, #3844]
	cmp	r3, r4
	ldr	r3, .L2079+36
	beq	.L2070
	ldr	r1, [r5, #-1776]
	cmp	r1, #39
	bhi	.L2070
	ldrh	r3, [r5, r3]
	ldrh	r2, [r5, r2]
	cmp	r2, r3
	ldrcc	r2, .L2079+40
	movcc	r3, r3, asl #1
	strcch	r3, [r5, r2]	@ movhi
	b	.L2057
.L2070:
	ldrh	r3, [r0, r3]
	ldrh	r2, [r0, r2]
	ldr	r1, .L2079+8
	add	r0, r3, r3, asl #1
	cmp	r2, r0, lsr #2
	ble	.L2077
	ldr	r2, .L2079
	ldrb	r4, [r2, #3072]	@ zero_extendqisi2
	ldr	r2, .L2079+40
	cmp	r4, #0
	subne	r3, r3, #2
	moveq	r3, #20
	strneh	r3, [r1, r2]	@ movhi
	streqh	r3, [r1, r2]	@ movhi
	beq	.L2057
	b	.L2077
.L2074:
	mov	r4, #1
	b	.L2057
.L2077:
	mov	r4, #0
.L2057:
	mov	r0, r4
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, pc}
.L2080:
	.align	2
.L2079:
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
	ldr	r4, .L2087
	mov	r3, r2
	mov	r5, #0
	mov	r6, #36
	ldr	r0, [r4, #-956]
	ldr	r1, [r4, #-1172]
	bl	FlashProgPages
	b	.L2082
.L2085:
	mul	r3, r6, r5
	ldr	r1, [r4, #-956]
	add	r2, r1, r3
	ldr	r1, [r1, r3]
	cmn	r1, #1
	bne	.L2083
	ldr	r2, .L2087+4
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
.L2083:
	ldr	r3, [r2, #12]
	add	r5, r5, #1
	ldr	r1, [r2, #4]
	uxth	r5, r5
	ldr	r0, [r3, #12]
	ldr	r2, [r3, #8]
	bl	FtlGcUpdatePage
.L2082:
	ldr	r1, [r4, #-1172]
	ldr	r7, .L2087
	cmp	r5, r1
	bcc	.L2085
	ldr	r0, [r7, #-956]
	bl	FtlGcBufFree
	ldr	r3, .L2087+8
	mov	r0, #0
	str	r0, [r7, #-1172]
	ldrh	r3, [r7, r3]
	cmp	r3, r0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2088:
	.align	2
.L2087:
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
	ldr	r3, .L2094
	ldr	r2, [r3, #4044]
	cmp	r2, #0
	bne	.L2090
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2091
	ldr	r3, .L2094+4
	ldr	r2, [r3, #-1172]
	tst	r2, #1
	beq	.L2091
	ldr	r2, .L2094+8
	ldrh	r3, [r3, r2]
	cmp	r3, #0
	bne	.L2090
.L2091:
	b	Ftl_gc_temp_data_write_back.part.23
.L2090:
	mov	r0, #0
	bx	lr
.L2095:
	.align	2
.L2094:
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
	ldr	r6, .L2098
	ldr	r0, .L2098+4
	ldr	r4, .L2098+8
	ldrh	r1, [r6, r5]
	bl	FtlGcScanTempBlk
	ldr	r3, .L2098+12
	ldrh	r2, [r4, r3]
	ldrh	r3, [r6, r5]
	cmp	r2, r3
	ldmccfd	sp!, {r4, r5, r6, pc}
	ldr	r0, .L2098+16
	bl	FtlMapBlkWriteDumpData
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	mov	r3, #0
	str	r3, [r4, #-1000]
	ldmfd	sp!, {r4, r5, r6, pc}
.L2099:
	.align	2
.L2098:
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
	ldr	r3, .L2101
	ldr	r5, .L2101+4
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
.L2102:
	.align	2
.L2101:
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
	ldr	r6, .L2116
	mov	r2, #0
	ldr	r4, .L2116+4
	mvn	r5, #0
	ldr	r0, [r6, #3856]
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #306
	strh	r5, [r4, r3]	@ movhi
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldr	r3, .L2116+8
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2104
	bl	FtlLoadSysInfo
	subs	r7, r0, #0
	bne	.L2104
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
	b	.L2105
.L2107:
	add	r0, r3, r7
	add	r7, r7, #12
	ldr	r0, [r0, #4]
	cmp	r0, #0
	blt	.L2106
	add	r2, r2, #1
.L2105:
	cmp	r2, r1
	blt	.L2107
.L2106:
	ldr	r3, .L2116+4
	cmp	r2, r1
	ldr	r0, .L2116+12
	ldrh	ip, [r3, r0]
	add	ip, ip, #1
	strh	ip, [r3, r0]	@ movhi
	blt	.L2108
	mov	r2, #308
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L2109
.L2108:
	ldr	r2, .L2116+16
	ldr	r0, [r3, #-2012]
	ldr	r1, .L2116+20
	ldrh	r2, [r3, r2]
	ldrh	ip, [r3, r1]
	mov	r2, r2, asl #1
	ldrh	lr, [r0, r2]
	rsb	ip, ip, lr
	strh	ip, [r0, r2]	@ movhi
	ldr	ip, .L2116
	movw	r0, #3918
	ldr	r2, .L2116+24
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
	ldr	r2, .L2116+28
	ldrh	ip, [ip, r0]
	ldr	r0, .L2116+32
	ldrh	r1, [r3, r2]
	strh	ip, [r3, r0]	@ movhi
	add	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L2109:
	ldr	r3, .L2116+4
	ldr	r2, .L2116+12
	mov	r5, r3
	ldrh	r2, [r3, r2]
	tst	r2, #31
	bne	.L2110
	bl	FtlVpcCheckAndModify
.L2110:
	ldr	r2, .L2116+16
	movw	r3, #65535
	ldr	r4, .L2116+4
	ldrh	r0, [r5, r2]
	cmp	r0, r3
	beq	.L2114
	ldr	r3, .L2116+20
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L2114
	add	r2, r2, #52
	ldrh	r5, [r4, r2]
	cmp	r5, #0
	movne	r5, r3
	bne	.L2104
	bl	FtlGcRefreshOpenBlock
	ldr	r3, .L2116+36
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshOpenBlock
	bl	FtlVpcTblFlush
	ldr	r0, .L2116+40
	bl	allocate_new_data_superblock
	ldr	r0, .L2116+44
	bl	allocate_new_data_superblock
	b	.L2104
.L2114:
	mov	r5, #0
.L2104:
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L2117:
	.align	2
.L2116:
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
	ldr	r5, .L2139
	mov	r1, #0
	ldr	r4, .L2139+4
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
	beq	.L2119
	bl	FtlMakeBbt
.L2119:
	mov	r3, #0
	ldr	ip, .L2139
	movw	r0, #3924
	ldr	r2, .L2139+4
	ldr	r1, .L2139+8
	b	.L2120
.L2121:
	ldr	lr, [r2, #-932]
	mvn	r4, r3
	orr	r4, r3, r4, asl #16
	str	r4, [lr, r3, asl #2]
	ldr	lr, [r2, #-928]
	str	r1, [lr, r3, asl #2]
	add	r3, r3, #1
	uxth	r3, r3
.L2120:
	ldrh	lr, [ip, r0]
	ldr	r8, .L2139
	cmp	r3, lr, asl #7
	blt	.L2121
	movw	r3, #3860
	mov	r5, #0
	ldrh	r6, [r8, r3]
	movw	r7, #3862
	b	.L2122
.L2123:
	mov	r0, r6
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r5, r5, r0
	uxth	r5, r5
.L2122:
	ldrh	r3, [r8, r7]
	ldr	r4, .L2139
	cmp	r3, r6
	bhi	.L2123
	movw	r3, #3852
	ldrh	r1, [r4, r3]
	sub	r3, r5, #3
	cmp	r3, r1, asl #1
	blt	.L2124
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
	b	.L2125
.L2126:
	mov	r0, r5
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r5, r5, #1
	uxth	r5, r5
.L2125:
	ldrh	r3, [r4, r6]
	cmp	r3, r5
	bhi	.L2126
.L2124:
	mov	r4, #0
	ldr	r8, .L2139
	mov	r6, r4
	movw	r7, #3860
	b	.L2127
.L2128:
	mov	r0, r6
	mov	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	uxth	r6, r6
	add	r4, r4, r0
	uxth	r4, r4
.L2127:
	ldrh	r3, [r8, r7]
	ldr	r5, .L2139
	cmp	r3, r6
	bhi	.L2128
	movw	r3, #3862
	ldr	r8, .L2139+4
	ldrh	r3, [r5, r3]
	ldr	fp, [r5, #3864]
	ldr	r9, .L2139+12
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
	ble	.L2129
	rsb	r0, r4, fp
	mov	r1, r6
	bl	__aeabi_uidiv
	str	r0, [r5, #3980]
	mov	r0, r0, lsr #5
	add	r0, r0, #24
	strh	r0, [r8, r9]	@ movhi
.L2129:
	ldr	r3, .L2139
	ldr	r3, [r3, #3844]
	cmp	r3, #1
	bne	.L2130
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r8, .L2139+4
	ldr	r5, .L2139+12
	ldrh	r9, [r8, r5]
	uxtah	r0, r9, r0
	add	r9, r9, r0, lsr #2
	strh	r9, [r8, r5]	@ movhi
.L2130:
	ldr	r3, .L2139
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2131
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidiv
	ldr	r8, .L2139+4
	ldr	r5, .L2139+12
	ldrh	r9, [r8, r5]
	uxtah	r0, r9, r0
	add	r9, r9, r0, lsr #2
	strh	r9, [r8, r5]	@ movhi
.L2131:
	ldr	r0, .L2139
	movw	r3, #3912
	ldrh	r3, [r0, r3]
	cmp	r3, #0
	beq	.L2132
	ldr	r1, .L2139+4
	ldr	r2, .L2139+12
	ldrh	ip, [r1, r2]
	add	ip, ip, r3, lsr #1
	strh	ip, [r1, r2]	@ movhi
	mul	ip, r6, r3
	cmp	ip, r4
	strgt	r7, [r0, #3980]
	addgt	r3, r3, #32
	addgt	sl, sl, r3
	strgth	sl, [r1, r2]	@ movhi
.L2132:
	ldr	r4, .L2139+4
	ldr	r5, .L2139
	ldr	r3, .L2139+12
	ldr	r7, .L2139+16
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
	ldr	r1, .L2139+20
	mov	r3, #0
	ldr	r2, .L2139+24
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
.L2133:
	ldr	r0, .L2139+28
	bl	make_superblock
	ldrb	r3, [r4, #-1985]	@ zero_extendqisi2
	ldr	r5, .L2139+4
	cmp	r3, #0
	ldr	r3, .L2139+16
	bne	.L2134
	ldrh	r3, [r4, r7]
	ldr	r2, [r4, #-2012]
	mov	r3, r3, asl #1
	strh	r6, [r2, r3]	@ movhi
	ldrh	r3, [r4, r7]
	add	r3, r3, #1
	strh	r3, [r4, r7]	@ movhi
	b	.L2133
.L2134:
	ldr	r2, [r5, #-1800]
	mvn	r7, #0
	ldr	r1, [r5, #-2012]
	ldr	r6, .L2139+32
	str	r2, [r5, #-1980]
	add	r2, r2, #1
	str	r2, [r5, #-1800]
	ldr	r2, .L2139+36
	ldrh	r0, [r5, r2]
	ldrh	r2, [r5, r3]
	mov	r2, r2, asl #1
	strh	r0, [r1, r2]	@ movhi
	mov	r2, #0
	ldr	r0, .L2139+40
	ldrh	r3, [r5, r3]
	strb	r2, [r5, #-1938]
	add	r3, r3, #1
	strh	r2, [r5, r0]	@ movhi
	strh	r3, [r5, r6]	@ movhi
	mov	r3, #1
	strb	r3, [r5, #-1936]
.L2135:
	ldr	r0, .L2139+44
	bl	make_superblock
	ldrb	r3, [r5, #-1937]	@ zero_extendqisi2
	ldr	r4, .L2139+4
	cmp	r3, #0
	bne	.L2136
	ldrh	r3, [r5, r6]
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	strh	r7, [r2, r3]	@ movhi
	ldrh	r3, [r5, r6]
	add	r3, r3, #1
	strh	r3, [r5, r6]	@ movhi
	b	.L2135
.L2136:
	ldr	r3, [r4, #-1800]
	mvn	r5, #0
	ldr	r2, [r4, #-2012]
	str	r3, [r4, #-1932]
	add	r3, r3, #1
	str	r3, [r4, #-1800]
	ldr	r3, .L2139+48
	ldrh	r1, [r4, r3]
	sub	r3, r3, #4
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r3, .L2139+52
	strh	r5, [r4, r3]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, .L2139+56
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
	ldr	r3, .L2139+60
	mov	r2, #1
	str	r2, [r3, #3220]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2140:
	.align	2
.L2139:
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
	ldr	r3, .L2143
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L2142
	bl	FtlLowFormat.part.27
.L2142:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2144:
	.align	2
.L2143:
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
	ldr	r3, .L2148
	ldr	r5, .L2148+4
	ldrh	r2, [r5, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L2146
	ldr	r3, .L2148+8
	ldrh	r3, [r5, r3]
	cmp	r3, #0
	bne	.L2147
.L2146:
	bl	FtlCacheWriteBack
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L2148+12
	mov	r4, #0
	strb	r4, [r5, #-1888]
	bl	allocate_data_superblock
	ldr	r3, .L2148+16
	strh	r4, [r5, r3]	@ movhi
	add	r3, r3, #12
	strh	r4, [r5, r3]	@ movhi
	bl	l2p_flush
	mov	r0, r4
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2147:
	ldr	r0, .L2148+12
	ldmfd	sp!, {r3, r4, r5, lr}
	b	get_new_active_ppa
.L2149:
	.align	2
.L2148:
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
	ldr	r3, .L2252
	.pad #36
	sub	sp, sp, #36
	ldr	r4, .L2252+4
	movw	r1, #65535
	str	r0, [sp, #16]
	ldrh	r7, [r3, r4]
	cmp	r7, r1
	beq	.L2151
	ldr	r6, .L2252+8
	ldrh	r2, [r3, r6]
	cmp	r2, r1
	bne	.L2151
	ldr	r0, .L2252+12
	ldrh	r5, [r3, r0]
	cmp	r5, r2
	beq	.L2151
	ldr	r1, .L2252+16
	ldrh	ip, [r3, r1]
	cmp	ip, r2
	strneh	r7, [r3, r6]	@ movhi
	strneh	r5, [r3, r4]	@ movhi
	mvnne	r2, #0
	strneh	ip, [r3, r0]	@ movhi
	strneh	r2, [r3, r1]	@ movhi
.L2151:
	ldr	r1, [sp, #16]
	ldr	r2, [r3, #-1184]
	cmp	r1, #1
	add	r2, r2, #1
	add	r2, r2, r1, asl #7
	str	r2, [r3, #-1184]
	bne	.L2152
	ldr	r3, .L2252+20
	ldr	r1, [r3, #3844]
	cmp	r1, #0
	bne	.L2153
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2152
.L2153:
	ldr	r4, .L2252
	ldr	r3, [r4, #-1776]
	cmp	r3, #39
	bhi	.L2152
	mov	r3, #444
	ldrh	r3, [r4, r3]
	add	r2, r2, r3
	str	r2, [r4, #-1184]
	bl	FtlGcReFreshBadBlk
	ldr	r3, .L2252+24
	movw	r2, #65535
	ldrh	r3, [r4, r3]
	cmp	r3, r2
	bne	.L2152
	ldr	r2, .L2252+8
	ldrh	r2, [r4, r2]
	cmp	r2, r3
	bne	.L2152
	ldr	r3, [r4, #-1184]
	cmp	r3, #1024
	bhi	.L2154
	ldr	r3, .L2252+28
	ldrh	r3, [r4, r3]
	cmp	r3, #63
	bhi	.L2152
.L2154:
	ldr	r3, .L2252
	mov	r2, #444
	ldr	r0, .L2252+28
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	ldrh	ip, [r3, r0]
	ldr	r0, .L2252+32
	ldrh	r0, [r3, r0]
	add	r0, r0, #64
	cmp	ip, r0
	bgt	.L2152
	str	r1, [r3, #-1184]
	ldr	r1, [r3, #-1776]
	cmp	r1, #0
	moveq	r1, #6
	beq	.L2248
	cmp	r1, #5
	bhi	.L2156
	mov	r1, #18
.L2248:
	strh	r1, [r3, r2]	@ movhi
.L2156:
	mov	r0, #32
	movw	r7, #65535
	bl	List_get_gc_head_node
	uxth	r2, r0
	cmp	r2, r7
	beq	.L2157
	ldr	r5, .L2252
	ldr	r6, .L2252+36
	ldrh	r0, [r5, r6]
	cmp	r0, #0
	moveq	r3, #1
	streqh	r3, [r5, r6]	@ movhi
	beq	.L2157
	ldr	r3, .L2252+20
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
	bgt	.L2157
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
	beq	.L2157
	ldrh	r2, [r9, r2]
	mov	r7, r4, asl #1
	ldr	r0, .L2252+40
	mov	r1, fp
	ldrh	r3, [r9, r7]
	str	r2, [sp, #0]
	mov	r2, r4
	bl	printk
	ldrh	r3, [r5, r6]
	cmp	r3, #40
	bls	.L2159
	ldr	r3, [r5, #-2012]
	ldrh	r3, [r3, r7]
	cmp	r3, #32
	strhih	r8, [r5, r6]	@ movhi
.L2159:
	ldr	r2, .L2252
	mov	r3, #444
	mov	r1, #6
	strh	r1, [r2, r3]	@ movhi
	b	.L2161
.L2157:
	bl	GetSwlReplaceBlock
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	bne	.L2161
	ldr	r2, .L2252
	mov	r3, #444
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L2152:
	ldr	r5, .L2252
	movw	r4, #65535
	ldr	r3, .L2252+24
	ldrh	r2, [r5, r3]
	cmp	r2, r4
	bne	.L2161
	sub	r3, r3, #144
	ldrh	r3, [r5, r3]
	cmp	r3, r2
	movne	r4, r2
	bne	.L2161
	ldr	r2, .L2252+8
	ldrh	r7, [r5, r2]
	cmp	r7, r3
	movne	r4, r3
	bne	.L2161
	ldr	r8, .L2252+28
	ldr	r2, [r5, #-1184]
	ldrh	r3, [r5, r8]
	cmp	r3, #23
	movhi	r3, #1024
	movls	r3, #5120
	cmp	r2, r3
	movls	r4, r7
	bls	.L2161
	mov	r2, #444
	mov	r3, #0
	str	r3, [r5, #-1184]
	mov	r6, r5
	strh	r3, [r5, r2]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r7
	mov	r4, r0
	bne	.L2163
	ldr	r3, .L2252+32
	ldrh	r1, [r5, r8]
	ldrh	r2, [r5, r3]
	cmp	r1, r2
	movcs	r2, #80
	strcsh	r2, [r5, r3]	@ movhi
	bcs	.L2173
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r4
	beq	.L2173
	ldr	r2, [r5, #-988]
	cmp	r2, #0
	bne	.L2166
	ldr	r2, .L2252+20
	mov	r1, #3872
	ldrh	r1, [r2, r1]
	cmp	r1, #3
	beq	.L2166
	ldr	r1, [r5, #-1704]
	cmp	r1, #0
	bne	.L2166
	ldr	r1, [r2, #3844]
	cmp	r1, #0
	bne	.L2166
	ldrb	r0, [r2, #3072]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L2167
.L2166:
	ldr	r1, .L2252
	mov	r3, r3, asl #1
	movw	ip, #3852
	mov	lr, #3872
	ldr	r2, [r1, #-2012]
	ldrh	r0, [r2, r3]
	mov	r2, #3920
	ldr	r3, .L2252+20
	ldrh	r2, [r3, r2]
	ldrh	ip, [r3, ip]
	ldrh	r3, [r3, lr]
	mul	ip, ip, r2
	cmp	r3, #3
	moveq	r3, r2, lsr #1
	movne	r3, #0
	add	r3, ip, r3
	cmp	r0, r3
	bgt	.L2169
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r2, .L2252+20
	ldr	r3, .L2252
	ldr	r2, [r2, #3980]
	ldr	r1, [r3, #-1840]
	add	r2, r2, r2, asl #1
	cmp	r1, r2, lsr #2
	ldr	r2, .L2252+32
	movhi	r1, #128
	movls	r1, #160
	strh	r1, [r3, r2]	@ movhi
	uxth	r4, r0
	b	.L2171
.L2169:
	ldr	r3, .L2252+32
	mov	r2, #128
	strh	r2, [r1, r3]	@ movhi
	b	.L2173
.L2167:
	ldr	r2, [r5, #-2012]
	mov	r3, r3, asl #1
	ldr	r6, .L2252
	ldr	r5, .L2252+32
	ldrh	r3, [r2, r3]
	cmp	r3, #7
	movhi	r3, #64
	strhih	r3, [r6, r5]	@ movhi
	bhi	.L2173
	bl	List_get_gc_head_node
	mov	r3, #128
	strh	r3, [r6, r5]	@ movhi
	uxth	r4, r0
.L2171:
	movw	r3, #65535
	cmp	r4, r3
	beq	.L2173
.L2163:
	ldr	r1, .L2252
	mov	r0, r4, asl #1
	ldr	r3, .L2252+28
	ldr	ip, .L2252+20
	ldrh	r2, [r1, r3]
	ldr	ip, [ip, #4052]
	ldr	r3, [r1, #-2012]
	ldrh	r3, [r3, r0]
	ldrh	r0, [ip, r0]
	str	r0, [sp, #0]
	ldr	r0, .L2252+44
	ldrh	r1, [r1, r0]
	ldr	r0, .L2252+48
	str	r1, [sp, #4]
	mov	r1, r4
	bl	printk
.L2173:
	bl	FtlGcReFreshBadBlk
.L2161:
	movw	ip, #65535
	rsb	r1, ip, r4
	rsbs	r0, r1, #0
	ldr	r2, [sp, #16]
	adc	r0, r0, r1
	ldr	r3, .L2252
	cmp	r2, #0
	movne	r1, #0
	andeq	r1, r0, #1
	cmp	r1, #0
	beq	.L2174
	ldr	r2, .L2252+28
	ldrh	r1, [r3, r2]
	cmp	r1, #24
	movhi	r6, #1
	bhi	.L2175
	ldr	r0, .L2252+20
	movw	r2, #3918
	cmp	r1, #16
	ldrh	r6, [r0, r2]
	movhi	r6, r6, lsr #5
	bhi	.L2175
	cmp	r1, #12
	movhi	r6, r6, lsr #4
	bhi	.L2175
	cmp	r1, #8
	movhi	r6, r6, lsr #2
.L2175:
	ldr	ip, .L2252+44
	ldr	r2, .L2252
	ldrh	r3, [r3, ip]
	cmp	r3, r1
	mov	r1, ip
	bcs	.L2179
	ldr	r3, .L2252+52
	movw	r0, #65535
	ldrh	r3, [r2, r3]
	cmp	r3, r0
	bne	.L2180
	ldr	r0, .L2252+8
	ldrh	r0, [r2, r0]
	cmp	r0, r3
	bne	.L2180
	mov	r3, #444
	ldrh	r0, [r2, r3]
	cmp	r0, #0
	bne	.L2181
	ldr	r3, .L2252+20
	ldr	r1, [r2, #-1840]
	ldr	r3, [r3, #3980]
	add	r3, r3, r3, asl #1
	cmp	r1, r3, lsr #2
	movcs	r3, #18
	strcsh	r3, [r2, ip]	@ movhi
	bcs	.L2183
.L2181:
	ldr	r3, .L2252
	ldr	r2, .L2252+56
	ldr	r1, .L2252+44
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L2183:
	ldr	r3, .L2252
	mov	r2, #0
	str	r2, [r3, #-1176]
	b	.L2184
.L2180:
	ldr	r3, .L2252
	ldr	r2, .L2252+56
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L2179:
	ldr	r3, .L2252+20
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
	b	.L2185
.L2174:
	ldr	r2, .L2252+52
	ldrh	r2, [r3, r2]
	cmp	r2, ip
	bne	.L2186
	ldr	ip, .L2252+8
	ldrh	ip, [r3, ip]
	cmp	ip, r2
	movne	r0, #0
	andeq	r0, r0, #1
	cmp	r0, #0
	beq	.L2186
	ldr	r0, .L2252+24
	ldrh	r0, [r3, r0]
	cmp	r0, r2
	movne	r4, r2
	bne	.L2186
	str	r1, [r3, #-1176]
	ldr	r2, .L2252+44
	ldr	r1, .L2252+28
	ldrh	r0, [r3, r1]
	ldrh	r1, [r3, r2]
	cmp	r0, r1
	bls	.L2187
	mov	r1, #444
	ldrh	r1, [r3, r1]
	cmp	r1, #0
	bne	.L2188
	ldr	r1, .L2252+20
	ldr	r0, [r3, #-1840]
	ldr	r1, [r1, #3980]
	add	r1, r1, r1, asl #1
	cmp	r0, r1, lsr #2
	movcs	r1, #18
	strcsh	r1, [r3, r2]	@ movhi
	bcs	.L2190
.L2188:
	ldr	r3, .L2252
	ldr	r2, .L2252+56
	ldr	r1, .L2252+44
	ldrh	r2, [r3, r2]
	add	r2, r2, r2, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r3, r1]	@ movhi
.L2190:
	bl	FtlReadRefresh
	mov	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, .L2252
	ldr	r2, [r3, #-2012]
	uxth	r0, r0
	mov	r0, r0, asl #1
	ldrh	r2, [r2, r0]
	cmp	r2, #4
	bhi	.L2251
.L2187:
	ldr	r4, .L2252
	mov	r6, #444
	ldrh	r0, [r4, r6]
	cmp	r0, #0
	bne	.L2242
	ldr	r3, .L2252+56
	ldrh	r5, [r4, r3]
	add	r3, r3, #584
	add	r2, r5, r5, asl #1
	mov	r2, r2, asr #2
	strh	r2, [r4, r3]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r4, #-2012]
	ldr	r1, .L2252+20
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
	ble	.L2191
	ldr	r2, .L2252+28
	sub	r5, r5, #1
	ldrh	r2, [r4, r2]
	cmp	r2, r5
	blt	.L2191
	bl	FtlReadRefresh
	ldrh	r0, [r4, r6]
	b	.L2184
.L2191:
	cmp	r3, #0
	bne	.L2242
	movw	r0, #65535
	bl	decrement_vpc_count
	ldr	r3, .L2252+28
	ldr	r2, .L2252
	ldrh	r0, [r2, r3]
	add	r0, r0, #1
	b	.L2184
.L2242:
	movw	r4, #65535
.L2186:
	ldr	r3, .L2252+20
	ldr	r6, [r3, #3844]
	cmp	r6, #0
	movne	r6, #2
	moveq	r6, #1
.L2185:
	ldr	r3, .L2252
	movw	r0, #65535
	ldr	r2, .L2252+24
	ldrh	r1, [r3, r2]
	cmp	r1, r0
	bne	.L2192
	cmp	r4, r1
	strneh	r4, [r3, r2]	@ movhi
	bne	.L2194
	ldr	r2, .L2252+8
	ldrh	r1, [r3, r2]
	cmp	r1, r4
	beq	.L2194
	ldr	r0, [r3, #-2012]
	mov	r1, r1, asl #1
	ldrh	r1, [r0, r1]
	cmp	r1, #0
	mvneq	r1, #0
	streqh	r1, [r3, r2]	@ movhi
	ldr	r3, .L2252
	ldr	r1, .L2252+24
	ldrh	r0, [r3, r2]
	strh	r0, [r3, r1]	@ movhi
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
.L2194:
	ldr	r5, .L2252
	mov	r3, #0
	ldr	r7, .L2252+24
	strb	r3, [r5, #-1744]
	movw	r3, #65535
	ldrh	r0, [r5, r7]
	cmp	r0, r3
	beq	.L2192
	bl	IsBlkInGcList
	cmp	r0, #0
	mvnne	r3, #0
	strneh	r3, [r5, r7]	@ movhi
	ldr	r3, .L2252+20
	ldr	r7, .L2252
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2197
	ldr	r3, .L2252+24
	ldrh	r0, [r7, r3]
	bl	ftl_get_blk_mode
	strb	r0, [r7, #-1744]
.L2197:
	ldr	r8, .L2252+24
	movw	r3, #65535
	ldr	r5, .L2252
	ldrh	r2, [r7, r8]
	cmp	r2, r3
	beq	.L2192
	ldr	r0, .L2252+60
	bl	make_superblock
	ldr	r1, .L2252+64
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
.L2192:
	ldr	r3, .L2252
	ldr	r0, .L2252+24
	ldr	r2, .L2252+68
	ldrh	r1, [r3, r0]
	ldrh	r2, [r3, r2]
	cmp	r2, r1
	mov	r2, r3
	beq	.L2198
	ldr	ip, .L2252+72
	ldrh	r3, [r3, ip]
	cmp	r3, r1
	movne	fp, r4
	bne	.L2247
.L2199:
.L2198:
	mvn	r3, #0
	strh	r3, [r2, r0]	@ movhi
	mov	r3, #444
.L2250:
	ldrh	r0, [r2, r3]
	b	.L2184
.L2247:
	ldr	r4, .L2252
	movw	sl, #65535
	ldr	r9, .L2252+24
	ldrh	r3, [r4, r9]
	cmp	r3, sl
	bne	.L2201
	ldr	r5, .L2252+36
	mov	r3, #0
	str	r3, [r4, #-1176]
.L2246:
	ldrh	r8, [r4, r5]
	mov	r0, r8
	bl	List_get_gc_head_node
	movw	r1, #65535
	uxth	r7, r0
	strh	r7, [r4, r9]	@ movhi
	cmp	r7, r1
	bne	.L2203
	ldr	r3, .L2252+36
	mov	r1, #0
	ldr	r2, .L2252
	mov	r0, #8
	strh	r1, [r2, r3]	@ movhi
	b	.L2184
.L2203:
	mov	r0, r7
	bl	IsBlkInGcList
	cmp	r0, #0
	add	r0, r8, #1
	strneh	r0, [r4, r5]	@ movhi
	bne	.L2246
	ldr	ip, .L2252+20
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
	bgt	.L2206
	cmp	r0, #48
	bls	.L2207
	cmp	r1, #8
	bls	.L2207
	ldr	r1, .L2252+76
	ldrh	r1, [r4, r1]
	cmp	r1, #35
	bhi	.L2207
.L2206:
	mov	r1, #0
	strh	r1, [r4, r5]	@ movhi
.L2207:
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, sl
	cmpge	fp, r3
	ldr	r3, .L2252
	bne	.L2208
	ldrh	r1, [r4, r5]
	cmp	r1, #3
	bhi	.L2208
	ldr	r2, .L2252+24
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
	add	r2, r2, #572
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
.L2251:
	mov	r2, #444
	b	.L2250
.L2208:
	cmp	r2, #0
	ldr	r8, .L2252
	bne	.L2209
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r4, r5]
	add	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	b	.L2246
.L2209:
	mov	r2, #0
	strb	r2, [r3, #-1744]
	ldr	r3, .L2252+20
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, r2
	beq	.L2210
	mov	r0, r7
	bl	ftl_get_blk_mode
	strb	r0, [r8, #-1744]
.L2210:
	ldr	r0, .L2252+60
	bl	make_superblock
	movw	r1, #446
	mov	r3, #0
	strh	r3, [r8, r1]	@ movhi
	ldr	r1, .L2252+24
	ldr	r0, [r8, #-2012]
	ldrh	r1, [r8, r1]
	mov	r1, r1, asl #1
	ldrh	r0, [r0, r1]
	mov	r1, #448
	strb	r3, [r8, #-1746]
	strh	r0, [r8, r1]	@ movhi
	ldr	r1, .L2252+64
	strh	r3, [r8, r1]	@ movhi
.L2201:
	ldr	sl, [sp, #16]
	cmp	sl, #1
	bne	.L2211
	bl	FtlReadRefresh
.L2211:
	ldr	r3, .L2252
	mov	r2, #1
	movw	r1, #3918
	str	r2, [r3, #-984]
	ldr	r2, .L2252+20
	ldrh	r9, [r2, r1]
	ldrb	r1, [r2, #3072]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L2212
	ldrb	r1, [r3, #-1744]	@ zero_extendqisi2
	cmp	r1, #1
	moveq	r1, #3920
	ldreqh	r9, [r2, r1]
.L2212:
	ldr	r2, .L2252+64
	mov	r8, #0
	str	fp, [sp, #20]
	mov	fp, r9
	ldr	r4, .L2252
	ldrh	r3, [r3, r2]
	add	r2, r3, r6
	cmp	r2, r9
	rsbgt	r6, r3, r9
	uxthgt	r6, r6
	b	.L2214
.L2216:
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
.L2222:
	cmp	r3, r5
	bne	.L2216
	ldr	r0, [r4, #-1156]
	mov	r1, r7
	ldrb	r2, [r4, #-1744]	@ zero_extendqisi2
	mov	r5, #0
	bl	FlashReadPages
	mov	r9, r5
	mov	sl, r6
	b	.L2217
.L2220:
	ldr	r3, [r4, #-1156]
	add	r2, r3, r5
	ldr	r3, [r3, r5]
	ldr	r6, [r2, #12]
	cmn	r3, #1
	beq	.L2218
	ldrh	r3, [r6, #0]
	movw	r1, #61589
	cmp	r3, r1
	bne	.L2218
	add	r1, sp, #24
	mov	r2, #0
	ldr	r0, [r6, #8]
	bl	log2phys
	ldr	r1, [r4, #-1156]
	ldr	r2, [sp, #24]
	add	r1, r1, r5
	bic	r2, r2, #-2147483648
	ldr	r3, [r1, #4]
	cmp	r2, r3
	bne	.L2218
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
	ldr	r3, [sp, #24]
	str	r3, [r6, #12]
	ldr	r3, .L2252+52
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
	ldr	r3, .L2252+20
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2219
	ldrb	r3, [r4, #-1889]	@ zero_extendqisi2
	ldr	r2, [r4, #-1172]
	cmp	r2, r3
	beq	.L2219
	ldr	r3, .L2252+80
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L2218
.L2219:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	ldrne	r3, .L2252
	movne	r2, #0
	strne	r2, [r3, #-984]
	bne	.L2251
.L2218:
	add	r9, r9, #1
	add	r5, r5, #36
	uxth	r9, r9
.L2217:
	cmp	r9, r7
	bne	.L2220
	add	r8, r8, #1
	mov	r6, sl
.L2214:
	uxth	r3, r8
	ldr	r7, .L2252
	cmp	r3, r6
	ldr	r3, .L2252+64
	bcs	.L2221
	ldr	r1, .L2252+20
	movw	r2, #3852
	ldrh	ip, [r4, r3]
	mov	r7, #0
	ldr	lr, [r4, #-1156]
	mov	r3, r7
	ldrh	r5, [r1, r2]
	add	ip, ip, r8
	ldr	r2, .L2252+84
	mov	r0, #36
	b	.L2222
.L2221:
	ldrh	r2, [r7, r3]
	mov	r9, fp
	ldr	fp, [sp, #20]
	add	r6, r6, r2
	uxth	r6, r6
	strh	r6, [r7, r3]	@ movhi
	cmp	r6, r9
	bcc	.L2223
	ldr	r3, [r7, #-1172]
	cmp	r3, #0
	beq	.L2224
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	movne	r3, #0
	strne	r3, [r7, #-984]
	movne	r3, #444
	ldrneh	r0, [r7, r3]
	bne	.L2184
.L2224:
	ldr	r5, .L2252
	movw	r3, #446
	ldrh	r4, [r5, r3]
	cmp	r4, #0
	bne	.L2225
	ldr	r3, .L2252+24
	ldr	r2, [r5, #-2012]
	ldrh	r3, [r5, r3]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	ldrne	r6, .L2252+20
	bne	.L2226
	b	.L2225
.L2229:
	mov	r0, r4
	add	r1, sp, #28
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #28]
	cmn	r0, #1
	beq	.L2227
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L2252+24
	ldrh	r3, [r5, r3]
	cmp	r3, r0
	beq	.L2228
.L2227:
	add	r4, r4, #1
.L2226:
	ldr	r3, [r6, #3980]
	cmp	r4, r3
	bcc	.L2229
.L2228:
	ldr	r3, .L2252+20
	ldr	r3, [r3, #3980]
	cmp	r4, r3
	bcc	.L2225
	ldr	r3, .L2252
	mov	ip, #0
	ldr	r2, .L2252+24
	ldr	r0, [r3, #-2012]
	ldrh	r1, [r3, r2]
	mov	r1, r1, asl #1
	strh	ip, [r0, r1]	@ movhi
	ldrh	r0, [r3, r2]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2225:
	ldr	r3, .L2252+24
	mvn	r1, #0
	ldr	r2, .L2252
	strh	r1, [r2, r3]	@ movhi
.L2223:
	ldr	r3, .L2252
	ldr	r2, .L2252+28
	ldrh	r2, [r3, r2]
	cmp	r2, #2
	ldrls	r2, .L2252+20
	movwls	r3, #3918
	ldrlsh	r6, [r2, r3]
	bls	.L2247
.L2230:
	mov	r1, #0
	str	r1, [r3, #-984]
	mov	r1, #444
	ldrh	r0, [r3, r1]
	cmp	r0, #0
	addeq	r0, r2, #1
.L2184:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2253:
	.align	2
.L2252:
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
	ldr	r3, .L2261
	mov	r5, r1
	ldr	r0, [r3, #4044]
	cmp	r0, #0
	movne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, pc}
	ldr	r3, .L2261+4
	ldr	r2, [r3, #-984]
	cmp	r2, #0
	ldmnefd	sp!, {r3, r4, r5, pc}
	ldr	r1, .L2261+8
	ldrh	r1, [r3, r1]
	cmp	r1, #47
	movls	r0, r2
	ldmlsfd	sp!, {r3, r4, r5, pc}
	ldr	r1, .L2261+12
	movw	r2, #3224
	ldrh	r1, [r1, r2]
	movw	r2, #65535
	cmp	r1, r2
	beq	.L2256
	ldr	r1, .L2261+16
	ldrh	r3, [r3, r1]
	cmp	r3, r2
	beq	.L2256
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	bne	.L2260
.L2256:
	mov	r0, r4
	mov	r1, r5
	ldmfd	sp!, {r3, r4, r5, lr}
	b	ftl_do_gc.part.24
.L2260:
	mov	r0, #1
	ldmfd	sp!, {r3, r4, r5, pc}
.L2262:
	.align	2
.L2261:
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
	ldr	r3, .L2286
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r0, r1, r4, r5, r6, r7, r8, r9, sl, lr}
	ldr	r4, [r3, #452]
	ldr	r3, .L2286+4
	ldr	r2, [r3, #4044]
	cmp	r2, #0
	bne	.L2264
	ldr	r1, [r3, #3972]
	cmp	r1, #0
	beq	.L2264
	ldrb	r8, [r3, #3072]	@ zero_extendqisi2
	cmp	r8, #0
	beq	.L2265
	ldrb	r8, [r4, #8]	@ zero_extendqisi2
	sub	r0, r8, #1
	rsbs	r8, r0, #0
	adc	r8, r8, r0
.L2265:
	ldr	r5, .L2286+4
	mov	r2, r8
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	mov	r6, #0
	mov	r7, r6
	ldr	r9, .L2286
	ldr	r0, [r5, #3976]
	bl	FlashProgPages
	b	.L2266
.L2271:
	ldr	r2, [r5, #3976]
	add	r3, r2, r6
	ldr	r2, [r2, r6]
	cmn	r2, #1
	beq	.L2285
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
	beq	.L2269
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r9, #-2012]
	mov	r3, r0, asl #1
	mov	sl, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L2270
	ldr	r0, .L2286+8
	mov	r1, sl
	bl	printk
.L2270:
	mov	r0, sl
	bl	decrement_vpc_count
.L2269:
	add	r7, r7, #1
	add	r6, r6, #36
.L2266:
	ldr	r3, [r5, #3972]
	cmp	r7, r3
	bcc	.L2271
	b	.L2272
.L2281:
	ldr	r3, [r5, #3976]
	mvn	r2, #0
	ldr	r7, .L2286
	str	r2, [r3, r6]
	b	.L2273
.L2277:
	ldr	r0, [r3, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #0]
	cmp	r3, r0
	bne	.L2274
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
.L2274:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L2275
	mov	r0, r4
	bl	allocate_new_data_superblock
.L2275:
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
	bne	.L2264
.L2273:
	ldr	r2, [r5, #3976]
	add	r3, r2, r6
	ldr	r2, [r2, r6]
	cmn	r2, #1
	beq	.L2277
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
	beq	.L2279
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r3, .L2286
	ldr	r2, [r3, #-2012]
	mov	r3, r0, asl #1
	mov	r7, r0
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L2280
	ldr	r0, .L2286+8
	mov	r1, r7
	bl	printk
.L2280:
	mov	r0, r7
	bl	decrement_vpc_count
.L2279:
	add	sl, sl, #1
	add	r6, r6, #36
	b	.L2267
.L2285:
	ldr	r5, .L2286+4
	mov	r6, #0
	mov	sl, r6
	mov	r9, r5
.L2267:
	ldr	r3, [r5, #3972]
	cmp	sl, r3
	bcc	.L2281
	movw	r4, #16386
	ldr	r6, .L2286
	ldr	r5, .L2286+12
	b	.L2282
.L2283:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	beq	.L2272
.L2282:
	ldrh	r3, [r6, r5]
	cmp	r3, #0
	bne	.L2283
.L2272:
	ldr	r3, .L2286+4
	mov	r2, #0
	str	r2, [r3, #3972]
.L2264:
	mov	r0, #0
	ldmfd	sp!, {r2, r3, r4, r5, r6, r7, r8, r9, sl, pc}
.L2287:
	.align	2
.L2286:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC151
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
	ldr	r3, .L2292
	ldr	r3, [r3, #4044]
	cmp	r3, #0
	bne	.L2291
	bl	FtlCacheWriteBack
	bl	l2p_flush
	mov	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2291:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2293:
	.align	2
.L2292:
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
	ldr	r3, .L2296
	ldr	r3, [r3, #3220]
	cmp	r3, #1
	bne	.L2295
	bl	FtlSysFlush
.L2295:
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L2297:
	.align	2
.L2296:
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
	ldr	r0, .L2300
	bl	printk
	ldmfd	sp!, {r3, lr}
	b	ftl_deinit
.L2301:
	.align	2
.L2300:
	.word	.LC152
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
	ldr	r3, .L2312
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
	.pad #52
	sub	sp, sp, #52
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2303
	ldr	r4, .L2312+4
	movw	r3, #306
	ldr	r0, .L2312+8
	movw	r7, #4097
	ldrh	r6, [r4, r3]
	ldr	r3, [r4, #-2012]
	mov	r5, r6, asl #1
	mov	r1, r6
	ldrh	r2, [r3, r5]
	bl	printk
	ldr	r3, .L2312+12
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshOpenBlock
	ldr	r3, .L2312+16
	ldrh	r0, [r4, r3]
	bl	FtlGcRefreshOpenBlock
	ldr	r0, .L2312+20
	bl	allocate_new_data_superblock
	ldr	r0, .L2312+24
	bl	allocate_new_data_superblock
	b	.L2305
.L2307:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	ldr	r3, [r4, #-2012]
	ldrh	r3, [r3, r5]
	cmp	r3, #0
	beq	.L2306
.L2305:
	subs	r7, r7, #1
	bne	.L2307
.L2306:
	ldr	r4, .L2312+4
	mov	r1, r6
	ldr	r0, .L2312+8
	ldr	r3, [r4, #-2012]
	ldrh	r2, [r3, r5]
	bl	printk
	ldr	r3, [r4, #-2012]
	ldrh	r4, [r3, r5]
	cmp	r4, #0
	bne	.L2308
	add	r7, sp, #48
	mov	r0, sp
	strh	r6, [r7, #-48]!	@ movhi
	add	r7, r7, #14
	bl	make_superblock
	ldr	r3, .L2312
	movw	r2, #3852
	movw	lr, #65535
	mov	ip, #36
	mov	r1, r4
	ldrh	r8, [r3, r2]
	ldr	sl, [r3, #4048]
	mov	r3, r4
	b	.L2309
.L2311:
	ldrh	r0, [r7, #2]!
	cmp	r0, lr
	beq	.L2310
	mla	r2, ip, r4, sl
	add	r4, r4, #1
	mov	r0, r0, asl #10
	uxth	r4, r4
	stmib	r2, {r0, r1}
	str	r1, [r2, #12]
.L2310:
	add	r3, r3, #1
	uxth	r3, r3
.L2309:
	cmp	r3, r8
	bne	.L2311
	ldr	r3, .L2312+4
	mov	r1, r6
	ldr	r0, .L2312+28
	ldr	r3, [r3, #-2012]
	ldrh	r2, [r3, r5]
	ldr	r5, .L2312
	bl	printk
	mov	r1, #0
	mov	r2, r4
	ldr	r0, [r5, #4048]
	bl	FlashEraseBlocks
	ldr	r0, [r5, #4048]
	mov	r1, #1
	mov	r2, r4
	bl	FlashEraseBlocks
.L2308:
	ldr	r2, .L2312+4
	movw	r3, #306
	mvn	r1, #0
	strh	r1, [r2, r3]	@ movhi
.L2303:
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L2313:
	.align	2
.L2312:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC153
	.word	-1992
	.word	-1944
	.word	.LANCHOR2-1992
	.word	.LANCHOR2-1944
	.word	.LC154
	.fnend
	.size	ftl_fix_nand_power_lost_error, .-ftl_fix_nand_power_lost_error
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.L2315:
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mvn	r3, #0
	ldr	r4, .L2325
	ldr	r6, .L2325+4
	ldr	r5, .L2325+8
	ldr	r1, .L2325+12
	str	r3, [r6, #3220]
	mov	r3, #0
	ldr	r0, .L2325+16
	str	r3, [r4, #4044]
	str	r3, [r5, #456]
	bl	printk
	ldr	r0, .L2325+20
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldr	r0, [r4, #3856]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	ldrne	r0, .L2325+24
	bne	.L2324
	bl	FtlSysBlkInit
	cmp	r0, #0
	beq	.L2318
	ldr	r0, .L2325+28
.L2324:
	ldr	r1, .L2325+32
	bl	printk
	b	.L2317
.L2318:
	mov	r1, #1
	str	r1, [r6, #3220]
	bl	ftl_do_gc
	ldr	r3, .L2325+36
	ldrh	r3, [r5, r3]
	cmp	r3, #15
	bhi	.L2319
	mov	r4, #1024
.L2320:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	bne	.L2320
	b	.L2317
.L2319:
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2317
	mov	r4, #128
.L2321:
	mov	r0, #1
	mov	r1, r0
	bl	ftl_do_gc
	subs	r4, r4, #1
	bne	.L2321
.L2317:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2326:
	.align	2
.L2325:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LC78
	.word	.LC77
	.word	.LANCHOR0+3044
	.word	.LC155
	.word	.LC156
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
	ldr	r5, .L2337
	mov	r7, r0
	mov	r4, r1
	ldr	r3, [r5, #3960]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L2328
	cmp	r1, #31
	bls	.L2335
	ldr	r3, [r5, #4044]
	cmp	r3, #0
	bne	.L2335
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
	beq	.L2329
	rsb	r6, r7, r6
	add	r5, r0, #1
	cmp	r6, r4
	movcs	r6, r4
	uxth	r6, r6
	rsb	r4, r6, r4
.L2329:
	mvn	r3, #0
	ldr	r7, .L2337
	str	r3, [sp, #4]
	movw	r6, #3924
	b	.L2330
.L2332:
	mov	r0, r5
	mov	r1, sp
	mov	r2, #0
	bl	log2phys
	ldr	r3, [sp, #0]
	cmn	r3, #1
	beq	.L2331
	ldr	r3, .L2337+4
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
.L2331:
	ldrh	r3, [r7, r6]
	add	r5, r5, #1
	rsb	r4, r3, r4
.L2330:
	ldrh	r3, [r7, r6]
	cmp	r4, r3
	bcs	.L2332
	ldr	r3, .L2337+4
	mov	r4, #0
	ldr	r2, [r3, #460]
	cmp	r2, #32
	bls	.L2336
	str	r4, [r3, #460]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L2336
.L2335:
	mov	r0, #0
	b	.L2328
.L2336:
	mov	r0, r4
.L2328:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, pc}
.L2338:
	.align	2
.L2337:
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
	ldr	r4, .L2385
	.pad #100
	sub	sp, sp, #100
	ldr	r8, .L2385+4
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
	beq	.L2373
	ldr	r3, [r8, #3976]
	sub	sl, sl, #1
	mov	r2, #36
	ldr	r0, [sp, #16]
	mla	sl, r2, sl, r3
	ldr	r3, [sl, #16]
	cmp	r0, r3
	strne	r5, [sp, #32]
	bne	.L2342
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
	bne	.L2343
	ldr	r2, [r4, #468]
	cmp	r2, #2
	ble	.L2344
.L2343:
	add	r9, r9, r3
	add	r7, r7, r8
	ldr	r3, [sp, #16]
	rsb	r1, r8, r5
	str	fp, [sp, #20]
	add	r3, r3, #1
	str	r1, [sp, #32]
	str	r3, [sp, #16]
.L2342:
	ldr	r3, .L2385
	mov	r2, #0
	str	r2, [r3, #468]
	b	.L2341
.L2373:
	str	r5, [sp, #32]
.L2341:
	ldr	r0, [sp, #16]
	ldr	r1, [sp, #36]
	bl	FtlCacheMetchLpa
	cmp	r0, #0
	beq	.L2345
	bl	FtlCacheWriteBack
.L2345:
	cmp	r5, r6, asl #1
	ldr	r3, .L2385
	ldr	r4, .L2385+4
	mov	fp, #0
	movcc	r6, #0
	movcs	r6, #1
	str	r6, [sp, #24]
	ldr	r6, .L2385+8
	ldr	r5, [sp, #16]
	mov	sl, r4
	str	fp, [sp, #40]
	str	r6, [r3, #452]
	b	.L2382
.L2371:
	ldrh	r1, [r6, #4]
	cmp	r1, #0
	bne	.L2347
	ldr	r2, .L2385+8
	ldr	r3, .L2385+12
	cmp	r6, r2
	bne	.L2348
	ldr	r2, .L2385+16
	ldrh	r8, [r8, r2]
	cmp	r8, #0
	bne	.L2349
	ldr	r0, .L2385+20
	str	r3, [sp, #4]
	bl	allocate_new_data_superblock
	ldr	r3, [sp, #4]
	str	r8, [r3, #3228]
.L2349:
	ldr	r0, .L2385+8
	str	r3, [sp, #4]
	bl	allocate_new_data_superblock
	ldr	r3, [sp, #4]
	ldr	r2, [r3, #3228]
	ldr	r3, .L2385+20
	cmp	r2, #0
	movne	r6, r3
	b	.L2350
.L2348:
	str	r1, [r3, #3228]
	ldr	r3, .L2385+24
	ldrh	r3, [r8, r3]
	cmp	r3, #0
	movne	r6, r2
	bne	.L2350
	mov	r0, r6
	bl	allocate_new_data_superblock
.L2350:
	ldrh	r3, [r6, #4]
	cmp	r3, #0
	bne	.L2351
	mov	r0, r6
	bl	allocate_new_data_superblock
.L2351:
	ldr	r3, .L2385
	str	r6, [r3, #452]
.L2347:
	ldr	r2, .L2385
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
	b	.L2352
.L2368:
	ldrh	r3, [r8, #4]
	cmp	r3, #0
	beq	.L2353
	ldr	r6, [sp, #36]
	rsb	lr, r6, r5
	ldr	r6, [sp, #24]
	rsbs	r3, lr, #0
	adc	r3, r3, lr
	tst	r3, r6
	beq	.L2354
	ldr	r6, [sp, #28]
	cmp	r6, #0
	beq	.L2354
	movw	lr, #3924
	ldr	r6, [sp, #32]
	ldrh	r2, [r4, lr]
	add	r1, r6, r7
	mls	r1, r2, r5, r1
	cmp	r1, r2
	bne	.L2353
.L2354:
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
	ldr	r6, .L2385
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
	ldr	r0, .L2385
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
	beq	.L2355
	ldr	r2, [sp, #12]
	cmp	r2, #0
	beq	.L2356
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
	b	.L2357
.L2356:
	cmp	r3, #0
	beq	.L2357
	ldr	r3, [sp, #32]
	movw	lr, #3924
	add	fp, r3, r7
	ldrh	r3, [r4, lr]
	mls	fp, r3, r5, fp
	ldr	r3, [sp, #12]
	str	r3, [sp, #40]
	uxth	fp, fp
.L2357:
	movw	lr, #3924
	ldrh	r3, [r4, lr]
	cmp	fp, r3
	bne	.L2358
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
	bne	.L2361
	ldr	r0, [r3, #8]
	movw	r3, #3928
	ldrh	r2, [sl, r3]
	b	.L2383
.L2358:
	ldr	r2, [sp, #56]
	mov	r3, #36
	cmn	r2, #1
	beq	.L2362
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
	ldr	r3, .L2385
	ldreq	r2, [r3, #-1628]
	addeq	r2, r2, #1
	streq	r2, [r3, #-1628]
	beq	.L2364
	ldr	r2, [r6, #8]
	cmp	r2, r5
	beq	.L2364
	ldr	r2, [r3, #-1628]
	ldr	r0, .L2385+28
	add	r2, r2, #1
	str	r2, [r3, #-1628]
	mov	r2, r5
	ldr	r1, [r6, #8]
	bl	printk
	b	.L2364
.L2362:
	ldr	r1, [r4, #3972]
	ldr	r2, [r4, #3976]
	mla	r3, r3, r1, r2
	mov	r1, #0
	ldr	r0, [r3, #8]
	movw	r3, #3928
	ldrh	r2, [r4, r3]
	bl	ftl_memset
.L2364:
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
	b	.L2383
.L2355:
	ldr	r3, [sp, #24]
	ldr	r2, [r4, #3972]
	cmp	r3, #0
	ldr	r3, [r4, #3976]
	mla	ip, ip, r2, r3
	beq	.L2366
	movw	lr, #3924
	ldrh	r3, [r4, lr]
	mul	r3, r3, r5
	rsb	r3, r7, r3
	add	r3, r9, r3, asl #9
	str	r3, [ip, #8]
	b	.L2361
.L2366:
	movw	r0, #3924
	movw	r3, #3928
	ldrh	r1, [r4, r0]
	ldrh	r2, [r4, r3]
	ldr	r0, [ip, #8]
	mul	r1, r1, r5
	rsb	r1, r7, r1
	add	r1, r9, r1, asl #9
.L2383:
	bl	memcpy
.L2361:
	ldr	r3, .L2385+32
	ldr	r0, [sp, #48]
	ldr	lr, [sp, #44]
	strh	r3, [r0, lr]	@ movhi
	ldr	r3, .L2385
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
.L2352:
	ldr	r6, [sp, #28]
	ldr	r3, [sp, #52]
	cmp	r6, r3
	bne	.L2368
.L2353:
	ldr	r3, [sp, #20]
	mov	r6, r8
	ldr	lr, [sp, #28]
	ldr	r2, [r4, #3972]
	rsb	r3, lr, r3
	str	r3, [sp, #20]
	ldr	r3, .L2385
	ldr	r0, [sp, #24]
	ldr	r3, [r3, #-964]
	cmp	r2, r3
	orrcs	r0, r0, #1
	uxtb	r3, r0
	cmp	r3, #0
	bne	.L2369
	ldrh	r2, [r8, #4]
	cmp	r2, #0
	bne	.L2379
.L2369:
	bl	FtlCacheWriteBack
	ldr	lr, [sp, #20]
	mov	r3, #0
	str	r3, [sl, #3972]
	cmp	lr, #1
	ldr	r3, [sp, #24]
	movls	r3, #0
.L2379:
	str	r3, [sp, #24]
.L2382:
	ldr	r3, [sp, #20]
	ldr	r8, .L2385
	cmp	r3, #0
	bne	.L2371
	mov	r0, r3
	ldr	r6, [sp, #36]
	ldr	r3, [sp, #16]
	rsb	r1, r3, r6
	bl	ftl_do_gc
	ldr	r3, .L2385+36
	ldrh	r3, [r8, r3]
	cmp	r3, #31
	bhi	.L2344
	ldr	r2, .L2385+40
	mov	r3, #128
	ldr	r5, .L2385+4
	mov	r4, #16
	strh	r3, [r8, r2]	@ movhi
	sub	r2, r2, #2
	strh	r3, [r8, r2]	@ movhi
.L2372:
	mov	r0, #0
	mov	r1, #1
	bl	ftl_do_gc
	ldr	r3, [r5, #4044]
	cmp	r3, #0
	bne	.L2344
	subs	r4, r4, #1
	bne	.L2372
.L2344:
	mov	r0, #0
	add	sp, sp, #100
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2386:
	.align	2
.L2385:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR2-1992
	.word	.LANCHOR1
	.word	-1940
	.word	.LANCHOR2-1944
	.word	-1988
	.word	.LC157
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
	ldr	r3, .L2392
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, [r3, #4044]
	cmp	r4, #0
	movne	r0, #0
	ldmnefd	sp!, {r4, pc}
	cmp	ip, #16
	bne	.L2389
	add	r0, r0, #256
	ldmfd	sp!, {r4, lr}
	b	FtlVendorPartWrite
.L2389:
	ldr	r3, [r3, #3960]
	add	ip, r1, r0
	cmp	ip, r3
	bhi	.L2391
	ldmfd	sp!, {r4, lr}
	b	ftl_write.part.25
.L2391:
	mvn	r0, #0
	ldmfd	sp!, {r4, pc}
.L2393:
	.align	2
.L2392:
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
	bne	.L2399
	add	r0, r1, #256
	mov	r1, r2
	mov	r2, r3
	bl	FtlVendorPartRead
	str	r0, [sp, #36]
	b	.L2400
.L2399:
	ldr	r3, .L2430
	ldr	r1, [sp, #48]
	ldr	r2, [r3, #3960]
	add	r1, r1, r6
	str	r1, [sp, #40]
	cmp	r1, r2
	mvnhi	r3, #0
	strhi	r3, [sp, #36]
	bhi	.L2400
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
	ldr	r3, .L2430+4
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
	beq	.L2401
	bl	FtlCacheWriteBack
.L2401:
	mov	r9, #0
	ldr	r5, [sp, #28]
	str	r9, [sp, #52]
	mov	r4, r9
	str	r9, [sp, #36]
	mov	r7, r6
	ldr	sl, .L2430
	b	.L2425
.L2419:
	mov	r2, #0
	mov	r0, r5
	add	r1, sp, #76
	bl	log2phys
	ldr	r2, [sp, #76]
	cmn	r2, #1
	bne	.L2426
	b	.L2429
.L2406:
	mla	r0, r0, r5, r6
	cmp	r0, r7
	bcc	.L2405
	ldr	r2, [sp, #40]
	cmp	r0, r2
	bcs	.L2405
	rsb	r0, r7, r0
	mov	r1, #0
	mov	r2, #512
	add	r0, r8, r0, asl #9
	bl	ftl_memset
.L2405:
	add	r6, r6, #1
	b	.L2403
.L2429:
	mov	r6, #0
	movw	fp, #3924
.L2403:
	ldrh	r0, [sl, fp]
	cmp	r6, r0
	bcc	.L2406
	b	.L2407
.L2426:
	ldr	r3, .L2430+4
	mov	r6, #36
	ldr	r1, [r3, #-960]
	mla	r6, r6, r4, r1
	ldr	r1, [sp, #28]
	cmp	r5, r1
	str	r2, [r6, #4]
	movw	r2, #3924
	bne	.L2408
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
	b	.L2409
.L2408:
	ldr	r1, [sp, #32]
	cmp	r5, r1
	ldrneh	r3, [sl, r2]
	mulne	r3, r3, r5
	bne	.L2427
	ldr	r3, [r3, #-928]
	ldrh	r2, [sl, r2]
	ldr	r1, [sp, #40]
	str	r3, [r6, #8]
	mul	r3, r2, r5
	rsb	r9, r3, r1
	cmp	r9, r2
	bne	.L2409
.L2427:
	rsb	r3, r7, r3
	add	r3, r8, r3, asl #9
	str	r3, [r6, #8]
.L2409:
	movw	r3, #3930
	ldr	r2, .L2430+4
	ldrh	r3, [sl, r3]
	str	r5, [r6, #16]
	ldr	r2, [r2, #-920]
	mul	r3, r3, r4
	add	r4, r4, #1
	bic	r3, r3, #3
	add	r3, r2, r3
	str	r3, [r6, #12]
.L2407:
	ldr	r2, [sp, #24]
	add	r5, r5, #1
	subs	r2, r2, #1
	str	r2, [sp, #24]
	beq	.L2411
	movw	r3, #3852
	ldrh	r3, [sl, r3]
	cmp	r4, r3, asl #3
	bne	.L2425
.L2411:
	cmp	r4, #0
	beq	.L2425
	ldr	r6, .L2430+4
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
.L2418:
	ldr	r1, [sp, #44]
	mov	r4, #36
	ldr	r3, [r6, #-960]
	mul	r4, r4, r1
	ldr	r1, [sp, #28]
	add	r3, r3, r4
	ldr	r2, [r3, #16]
	cmp	r2, r1
	bne	.L2413
	ldr	r1, [r3, #8]
	ldr	r3, [r6, #-932]
	cmp	r1, r3
	bne	.L2414
	ldr	r2, [sp, #60]
	mov	r0, r8
	add	r1, r1, r2
	ldr	r2, [sp, #64]
	b	.L2428
.L2413:
	ldr	r1, [sp, #32]
	cmp	r2, r1
	bne	.L2414
	ldr	r1, [r3, #8]
	ldr	r3, [r6, #-928]
	cmp	r1, r3
	bne	.L2414
	movw	r2, #3924
	ldr	r3, [sp, #32]
	ldrh	r0, [sl, r2]
	ldr	r2, [sp, #68]
	mul	r0, r0, r3
	rsb	r0, r7, r0
	add	r0, r8, r0, asl #9
.L2428:
	bl	memcpy
.L2414:
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
	beq	.L2416
	ldr	r2, [r6, #-1628]
	ldr	r0, .L2430+8
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
.L2416:
	ldr	r3, [r6, #-960]
	add	r2, r3, r4
	ldr	r3, [r3, r4]
	cmp	r3, #256
	bne	.L2417
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L2417:
	ldr	r1, [sp, #44]
	add	r1, r1, #1
	str	r1, [sp, #44]
	cmp	r1, r5
	bne	.L2418
	mov	r5, fp
	mov	r4, #0
.L2425:
	ldr	r2, [sp, #24]
	cmp	r2, #0
	bne	.L2419
	ldr	r3, .L2430+12
	ldr	r2, .L2430+4
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L2400
	ldr	r0, [sp, #24]
	mov	r1, #1
	bl	ftl_do_gc
.L2400:
	ldr	r0, [sp, #36]
	add	sp, sp, #84
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2431:
	.align	2
.L2430:
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
	beq	.L2438
	b	FlashBootVendorWrite
.L2438:
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
	ldr	r1, .L2450
	ldr	r0, .L2450+4
	bl	printk
	ldr	r0, .L2450+8
	mov	r1, #8192
	ldr	r6, .L2450+12
	bl	__memzero
	ldr	r5, .L2450+16
	b	.L2440
.L2442:
	mov	r0, r4
	add	r1, sp, #4
	mov	r2, #0
	bl	log2phys
	ldr	r0, [sp, #4]
	cmn	r0, #1
	beq	.L2441
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	add	r0, r5, r0, asl #1
	add	r0, r0, #472
	ldrh	r3, [r0, #0]
	add	r3, r3, #1
	strh	r3, [r0, #0]	@ movhi
.L2441:
	add	r4, r4, #1
.L2440:
	ldr	r3, [r6, #3980]
	ldr	r7, .L2450+12
	cmp	r4, r3
	bcc	.L2442
	ldr	r5, .L2450+16
	movw	r6, #3860
	ldr	r0, .L2450+20
	mov	r2, #2
	ldrh	r3, [r7, r6]
	mov	r4, #0
	ldr	r1, [r5, #-2012]
	bl	rknand_print_hex
	ldr	r0, .L2450+24
	add	r1, r5, #472
	mov	r2, #2
	ldrh	r3, [r7, r6]
	bl	rknand_print_hex
	b	.L2443
.L2445:
	ldr	r2, [r5, #-2012]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	add	r3, r5, r3
	add	r3, r3, #472
	ldrh	r3, [r3, #0]
	cmp	r2, r3
	beq	.L2444
	ldr	r0, .L2450+28
	mov	r1, r4
	bl	printk
.L2444:
	add	r4, r4, #1
	uxth	r4, r4
.L2443:
	ldrh	r3, [r7, r6]
	cmp	r3, r4
	ldr	r3, .L2450+16
	bhi	.L2445
	ldr	r2, [r3, #-2000]
	cmp	r2, #0
	beq	.L2439
	ldr	r4, [r3, #-2020]
	mov	r6, #0
	ldr	r1, .L2450+32
	mov	r5, r3
	rsb	r2, r4, r2
	ldr	r4, .L2450+36
	mov	r8, #6
	movw	sl, #65535
	mov	r2, r2, asr #1
	ldrh	r7, [r3, r1]
	mul	r4, r4, r2
	uxth	r4, r4
	b	.L2447
.L2449:
	ldr	r2, [r5, #-2012]
	mov	r3, r4, asl #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	beq	.L2448
	add	r3, r5, r3
	ldr	r0, .L2450+40
	add	r3, r3, #472
	mov	r1, r4
	ldrh	r3, [r3, #0]
	bl	printk
.L2448:
	mul	r4, r8, r4
	ldr	r3, [r5, #-2020]
	ldrh	r4, [r3, r4]
	cmp	r4, sl
	beq	.L2439
	add	r6, r6, #1
	uxth	r6, r6
.L2447:
	cmp	r6, r7
	bne	.L2449
.L2439:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, sl, pc}
.L2451:
	.align	2
.L2450:
	.word	.LANCHOR3+77
	.word	.LC101
	.word	.LANCHOR2+472
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC158
	.word	.LC159
	.word	.LC160
	.word	-1996
	.word	-1431655765
	.word	.LC161
	.fnend
	.size	FtlCheckVpc, .-FtlCheckVpc
	.align	2
	.global	HynixGetReadRetryDefault
	.type	HynixGetReadRetryDefault, %function
HynixGetReadRetryDefault:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2547
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
	bne	.L2453
	mvn	r2, #88
	strb	r2, [r3, #4]
	ldr	r3, .L2547+4
	mvn	r2, #8
	mov	r7, #7
	strb	r2, [r3, #3249]
	b	.L2517
.L2453:
	cmp	r4, #3
	bne	.L2455
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
	b	.L2541
.L2455:
	cmp	r4, #4
	bne	.L2456
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
.L2541:
	mov	r7, #8
	strb	r2, [r3, #11]
	mov	r6, r7
	b	.L2454
.L2456:
	cmp	r4, #5
	bne	.L2457
	mov	r2, #56
	mov	r7, #8
	strb	r2, [r3, #4]
	mov	r2, #57
	strb	r2, [r3, #5]
	mov	r2, #58
	strb	r2, [r3, #6]
	mov	r2, #59
	strb	r2, [r3, #7]
	b	.L2517
.L2457:
	cmp	r4, #6
	bne	.L2458
	mov	r2, #14
	mov	r7, #12
	strb	r2, [r3, #4]
	mov	r2, #15
	strb	r2, [r3, #5]
	mov	r2, #16
	strb	r2, [r3, #6]
	mov	r2, #17
	strb	r2, [r3, #7]
	b	.L2517
.L2458:
	cmp	r4, #7
	bne	.L2459
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
	b	.L2454
.L2459:
	cmp	r4, #8
	mov	r7, #7
	bne	.L2517
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
	b	.L2454
.L2517:
	mov	r6, #4
.L2454:
	sub	r3, r4, #1
	cmp	r3, #1
	bhi	.L2537
	b	.L2545
.L2466:
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
.L2462:
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
	bcc	.L2462
	mov	r2, #0
	b	.L2463
.L2464:
	ldrb	r6, [r0, r1, asl #2]	@ zero_extendqisi2
	ldrb	r5, [r4, r2]	@ zero_extendqisi2
	add	r5, r6, r5
	strb	r5, [ip, r1, asl #3]
	add	r1, r1, #1
	cmp	r1, #7
	bne	.L2464
	add	r2, r2, #1
	cmp	r2, #4
	beq	.L2465
.L2463:
	mov	r1, #1
	add	ip, r4, r2
	add	r0, r2, r9
	b	.L2464
.L2465:
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
	b	.L2460
.L2545:
	ldr	r9, .L2547+8
	mov	r3, r7
	mov	r8, #0
	mov	r7, r6
.L2460:
	ldr	fp, .L2547
	ldrb	r2, [fp, #3762]	@ zero_extendqisi2
	cmp	r2, r8
	bhi	.L2466
	mov	r6, r7
	mov	r7, r3
	b	.L2467
.L2537:
	sub	r3, r4, #3
	cmp	r3, #5
	bhi	.L2467
	mul	r3, r7, r6
	mov	sl, #0
	mov	r5, sl
	str	r7, [sp, #48]
	mov	r2, r3, asr #1
	mov	r3, r3, asl #4
	str	r2, [sp, #24]
	str	r3, [sp, #52]
	b	.L2468
.L2516:
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
	beq	.L2470
.L2469:
	cmp	r4, #8
	addne	r1, r1, #20
	addne	r1, r1, r7, asl #6
	strne	r1, [sp, #32]
	bne	.L2470
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
	b	.L2472
.L2470:
	cmp	r4, #4
	mov	r3, #54
	str	r3, [r8, #8]
	bne	.L2473
	mov	r3, #255
	str	r3, [r8, #4]
	mov	r3, #64
	str	r3, [r9, fp, asl #8]
	mov	r3, #204
	b	.L2542
.L2473:
	sub	r3, r4, #5
	cmp	r3, #1
	ldrls	r3, .L2547
	ldrlsb	r3, [r3, #4]	@ zero_extendqisi2
	strls	r3, [r8, #4]
	movls	r3, #82
	bls	.L2543
.L2475:
	cmp	r4, #7
	bne	.L2474
	mov	r3, #174
	str	r3, [r8, #4]
	mov	r3, #176
	str	r5, [r9, fp, asl #8]
.L2542:
	str	r3, [r8, #4]
	mov	r3, #77
.L2543:
	str	r3, [r9, fp, asl #8]
.L2474:
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
.L2472:
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
	bne	.L2478
	cmp	r4, #7
	moveq	r1, #32
	movne	r1, #2
.L2478:
	ldr	r3, .L2547+12
	mov	r2, #0
	ldr	r3, [r3, #160]
.L2479:
	ldr	r0, [r8, #0]
	strb	r0, [r3, r2]
	add	r2, r2, #1
	uxtb	r0, r2
	cmp	r0, r1
	bcc	.L2479
	ldr	r1, [sp, #20]
	cmp	r1, #0
	beq	.L2480
	mov	r2, #0
.L2482:
	ldrb	r1, [r3, #0]	@ zero_extendqisi2
	cmp	r1, #50
	beq	.L2481
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	cmp	r1, #5
	beq	.L2481
	add	r2, r2, #1
	add	r3, r3, #4
	uxtb	r2, r2
	cmp	r2, #8
	bne	.L2482
	b	.L2483
.L2481:
	cmp	r2, #6
	bls	.L2484
.L2483:
	ldr	r0, .L2547+16
	mov	r1, #0
	bl	printk
.L2485:
	b	.L2485
.L2480:
	cmp	r4, #7
	bne	.L2486
	ldr	r2, [sp, #20]
.L2488:
	ldrb	r1, [r3, #0]	@ zero_extendqisi2
	cmp	r1, #12
	beq	.L2487
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	cmp	r1, #10
	beq	.L2487
	add	r2, r2, #1
	add	r3, r3, #4
	uxtb	r2, r2
	cmp	r2, #8
	bne	.L2488
	b	.L2489
.L2487:
	cmp	r2, #6
	bls	.L2484
.L2489:
	ldr	r0, .L2547+16
	mov	r1, #0
	bl	printk
.L2490:
	b	.L2490
.L2486:
	cmp	r4, #6
	bne	.L2484
	mov	r1, #8
.L2491:
	ldrb	r2, [r3], #1	@ zero_extendqisi2
	cmp	r2, #12
	beq	.L2484
	ldrb	r2, [r3, #7]	@ zero_extendqisi2
	cmp	r2, #4
	beq	.L2484
	sub	r1, r1, #1
	uxtb	r1, r1
	cmp	r1, #0
	bne	.L2491
	b	.L2546
.L2493:
	b	.L2493
.L2484:
	ldr	r2, .L2547+12
	ldr	r3, [r2, #160]
	str	r3, [sp, #40]
	mov	r3, #0
.L2494:
	ldr	ip, [sp, #40]
	ldr	r1, [r8, #0]
	strb	r1, [ip, r3]
	add	r3, r3, #1
	ldr	ip, [sp, #52]
	cmp	r3, ip
	blt	.L2494
	ldr	r0, [sp, #24]
	ldr	ip, [r2, #160]
	mov	r2, #8
	ldr	lr, [sp, #24]
	mov	r0, r0, asl #1
	str	r0, [sp, #44]
	add	r1, ip, r0
	mov	lr, lr, asl #2
	str	lr, [sp, #12]
.L2496:
	mov	r3, #0
	mov	r0, r3
.L2495:
	ldrh	lr, [r1, r3]
	add	r0, r0, #1
	mvn	lr, lr
	strh	lr, [r1, r3]	@ movhi
	ldr	lr, [sp, #24]
	add	r3, r3, #2
	cmp	r0, lr
	blt	.L2495
	ldr	r0, [sp, #12]
	subs	r2, r2, #1
	add	r1, r1, r0
	bne	.L2496
	str	ip, [sp, #28]
	str	r2, [sp, #16]
	b	.L2497
.L2501:
	mov	r1, #1
	mov	ip, #16
	mov	r0, r1, asl r3
	mov	r1, #0
	str	ip, [sp, #12]
	mov	ip, r1
	str	r3, [sp, #4]
.L2499:
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
	bne	.L2499
	cmp	ip, #8
	ldr	r3, [sp, #4]
	orrhi	r2, r2, r0
	add	r3, r3, #1
	uxthhi	r2, r2
	cmp	r3, #16
	bne	.L2501
	ldr	ip, [sp, #28]
	ldr	lr, [sp, #16]
	strh	r2, [ip], #2	@ movhi
	add	lr, lr, #1
	ldr	r2, [sp, #24]
	str	ip, [sp, #28]
	cmp	lr, r2
	str	lr, [sp, #16]
	bge	.L2502
.L2497:
	mov	r2, #0
	mov	r3, r2
	b	.L2501
.L2502:
	ldr	r3, .L2547+12
	ldr	r1, [r3, #160]
	mov	r3, #0
	mov	r2, r3
.L2504:
	ldr	r0, [r1, r3]
	add	r3, r3, #4
	cmp	r0, #0
	addeq	r2, r2, #1
	cmp	r3, #32
	bne	.L2504
	cmp	r2, #7
	ble	.L2505
	ldr	r0, .L2547+20
	mov	r2, #1
	mov	r3, #1024
	bl	rknand_print_hex
	ldr	r0, .L2547+16
	mov	r1, #0
	bl	printk
.L2506:
	b	.L2506
.L2505:
	cmp	r4, #6
	moveq	ip, #4
	beq	.L2507
	cmp	r4, #7
	moveq	ip, #10
	beq	.L2507
	ldr	r3, [sp, #20]
	cmp	r3, #0
	movne	ip, #5
	moveq	ip, #8
.L2507:
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
	b	.L2510
.L2511:
	ldr	r2, [sp, #12]
	ldrb	lr, [r2], #1	@ zero_extendqisi2
	strb	lr, [r1, r3]
	add	r3, r3, #1
	str	r2, [sp, #12]
	uxtb	lr, r3
	cmp	lr, r6
	bcc	.L2511
	ldr	r3, [sp, #16]
	add	r1, r1, ip
	ldr	r2, [sp, #28]
	add	r0, r0, r3
	ldr	r3, [sp, #48]
	add	r2, r2, #1
	cmp	r2, r3
	bge	.L2512
.L2510:
	str	r0, [sp, #12]
	mov	r3, #0
	str	r2, [sp, #28]
	b	.L2511
.L2512:
	mov	r3, #255
	mov	r0, r7
	str	r3, [r8, #8]
	bl	NandcWaitFlashReady
	ldr	ip, [sp, #36]
	cmp	ip, #0
	beq	.L2513
	mov	r3, #54
	str	r3, [r8, #8]
	ldr	r3, .L2547
	mov	r0, sl
	mvn	r1, #0
	ldrb	r3, [r3, #4]	@ zero_extendqisi2
	str	r3, [r8, #4]
	mov	r3, #22
	str	r5, [r9, fp, asl #8]
	str	r3, [r8, #8]
	bl	FlashReadCmd
	b	.L2514
.L2513:
	ldr	r1, [sp, #20]
	cmp	r1, #0
	movne	r3, #190
	moveq	r3, #56
	str	r3, [r8, #8]
.L2514:
	mov	r0, r7
	add	sl, sl, #1
	bl	NandcWaitFlashReady
	uxtb	sl, sl
.L2468:
	ldr	r1, .L2547
	ldrb	r3, [r1, #3762]	@ zero_extendqisi2
	cmp	r3, sl
	bhi	.L2516
	ldr	r7, [sp, #48]
.L2467:
	ldr	r3, .L2547
	strb	r6, [r3, #1]
	strb	r7, [r3, #2]
	add	sp, sp, #60
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2546:
	ldr	r0, .L2547+16
	bl	printk
	b	.L2493
.L2548:
	.align	2
.L2547:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR1+3232
	.word	.LANCHOR2
	.word	.LC162
	.word	.LC163
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
	bhi	.L2551
	ldmfd	sp!, {r3, lr}
	b	HynixGetReadRetryDefault
.L2551:
	cmp	r3, #49
	bne	.L2552
	ldr	r0, .L2553
	mov	r2, #64
	ldr	r1, .L2553+4
	strb	r3, [r0, #0]
	mov	r3, #4
	strb	r3, [r0, #1]
	mov	r3, #15
	strb	r3, [r0, #2]
	add	r0, r0, #4
	bl	memcpy
	ldmfd	sp!, {r3, pc}
.L2552:
	ldmfd	sp!, {r3, lr}
	b	FlashGetReadRetryDefault.part.28
.L2554:
	.align	2
.L2553:
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
	ldr	r5, .L2609
	bl	ftl_malloc
	ldr	r6, .L2609+4
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
.L2559:
	uxtb	r0, r4
	mov	r1, r5
	bl	FlashReadIDRaw
	cmp	r4, #0
	bne	.L2556
	ldrb	r3, [r6, #2976]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L2599
	ldrb	r3, [r6, #2977]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2600
.L2556:
	ldrb	r3, [r5, #0]	@ zero_extendqisi2
	add	r4, r4, #1
	cmp	r3, #181
	streqb	r7, [r5, #0]
	cmp	r4, #4
	add	r5, r5, #8
	bne	.L2559
	ldr	r4, .L2609+4
	ldrb	r3, [r4, #2976]	@ zero_extendqisi2
	cmp	r3, #173
	beq	.L2560
	ldr	r0, [r4, #3784]
	bl	NandcSetDdrMode
.L2560:
	ldr	r5, .L2609+4
	mov	r1, #0
	mov	r2, #852
	mov	r0, r5
	bl	ftl_memset
	ldr	r3, .L2609+8
	ldr	r1, [r4, #3792]
	ldr	r2, .L2609+4
	str	r3, [r4, #3624]
	mov	r3, #0
	strb	r3, [r4, #860]
	ldr	r3, .L2609+12
	cmp	r1, r3
	moveq	r3, #1
	streq	r3, [r5, #3776]
	ldrb	r3, [r2, #2977]	@ zero_extendqisi2
	cmp	r3, #161
	cmpne	r3, #241
	beq	.L2562
	cmp	r3, #218
	beq	.L2562
	cmp	r3, #209
	beq	.L2562
	cmp	r3, #211
	beq	.L2562
	cmp	r3, #220
	bne	.L2563
	ldrb	r0, [r2, #2979]	@ zero_extendqisi2
	cmp	r0, #149
	bne	.L2563
.L2562:
	mov	r0, #1
	strb	r0, [r2, #852]
	ldr	r0, .L2609
	mov	ip, #16
	strb	ip, [r2, #853]
	ldrb	r2, [r2, #2976]	@ zero_extendqisi2
	strb	ip, [r0, #164]
	ldr	ip, .L2609+16
	cmp	r2, #152
	strb	r2, [ip, #3261]
	strb	r3, [ip, #3262]
	bne	.L2564
	ldr	r2, .L2609+4
	ldrb	r2, [r2, #2980]	@ zero_extendqisi2
	sxtb	r2, r2
	cmp	r2, #0
	movge	r2, #24
	strgeb	r2, [r0, #164]
.L2564:
	ldr	r2, .L2609+12
	movw	r0, #2049
	cmp	r1, r2
	cmpne	r1, r0
	ldreq	r2, .L2609
	moveq	r1, #16
	streqb	r1, [r2, #164]
	cmp	r3, #218
	bne	.L2566
	ldr	r3, .L2609+16
	movw	r2, #3274
	mov	r1, #2048
	strh	r1, [r3, r2]	@ movhi
	mvn	r2, #37
	b	.L2606
.L2566:
	cmp	r3, #220
	bne	.L2568
	ldr	r3, .L2609+16
	movw	r2, #3274
	mov	r1, #4096
	strh	r1, [r3, r2]	@ movhi
	mvn	r2, #35
.L2606:
	strb	r2, [r3, #3262]
	b	.L2567
.L2568:
	cmp	r3, #211
	bne	.L2567
	ldr	r3, .L2609+16
	movw	r2, #3274
	mov	r1, #4096
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #2
	strb	r2, [r3, #3273]
.L2567:
	ldr	r1, .L2609+20
	mov	r2, #32
	ldr	r0, .L2609+24
	bl	memcpy
	ldr	r0, .L2609+8
	mov	r2, #32
	add	r1, r0, #144
	bl	memcpy
.L2563:
	ldr	r4, .L2609+4
	ldrb	r3, [r4, #852]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2569
	bl	FlashLoadPhyInfoInRam
	cmp	r0, #0
	bne	.L2570
	ldr	r3, [r4, #3624]
	ldrh	r0, [r3, #16]
	ubfx	r0, r0, #8, #3
	strb	r0, [r4, #3772]
	tst	r0, #1
	bne	.L2570
	mov	r3, #1
	strb	r3, [r4, #3780]
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	NandcSetMode
.L2570:
	ldr	r4, .L2609+4
	ldr	r3, [r4, #3624]
	ldrb	r3, [r3, #26]	@ zero_extendqisi2
	strb	r3, [r4, #3072]
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L2569
	ldr	r3, [r4, #3784]
	cmp	r3, #0
	beq	.L2572
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	b	.L2607
.L2572:
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
	bl	FlashSetInterfaceMode
	ldrb	r0, [r4, #3772]	@ zero_extendqisi2
.L2607:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cmp	r0, #0
	beq	.L2569
	ldr	r4, .L2609+4
	mov	r0, #1
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
	ldr	r0, .L2609+28
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
	ldr	r1, .L2609+32
	ldr	r0, .L2609
	ldr	r3, [r4, #3624]
	ldrh	r1, [r0, r1]
	ldrb	r2, [r3, #9]	@ zero_extendqisi2
	add	r1, r1, #4080
	add	r1, r1, #15
	cmp	r2, r1, lsr #12
	blt	.L2575
	ldrh	r1, [r3, #14]
	add	r1, r1, #255
	cmp	r2, r1, lsr #8
	bge	.L2576
.L2575:
	ldrh	r2, [r3, #14]
	bic	r2, r2, #255
	strh	r2, [r3, #14]	@ movhi
.L2576:
	ldr	r3, .L2609+4
	ldrb	r3, [r3, #3772]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L2577
	bl	FlashSavePhyInfo
	ldr	r3, .L2609
	mov	r0, #0
	ldr	r1, [r3, #172]
	bl	FlashDdrParaScan
.L2577:
	bl	FlashSavePhyInfo
.L2569:
	ldr	r4, .L2609+4
	ldr	r5, .L2609
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
	beq	.L2578
	ldrb	r0, [r3, #19]	@ zero_extendqisi2
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	strb	r0, [r4, #3761]
	strb	r3, [r4, #3760]
	ldrb	r3, [r4, #2]	@ zero_extendqisi2
	strb	r3, [r5, #152]
	sub	r3, r0, #1
	uxtb	r3, r3
	cmp	r3, #7
	bhi	.L2579
	ldr	r3, .L2609+36
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
	ldreq	r1, .L2609+40
	beq	.L2582
	ldr	r3, .L2609+44
	cmp	r1, #0
	add	r2, r3, #8
	moveq	r1, r3
	movne	r1, r2
.L2582:
	mov	r3, #0
	mov	r2, r3
.L2584:
	ldrsb	ip, [r1, r2]
	add	r2, r2, #1
	cmp	ip, #0
	addeq	r3, r3, #1
	cmp	r2, #32
	bne	.L2584
	cmp	r3, #27
	bls	.L2578
	bl	FlashGetReadRetryDefault
	bl	FlashSavePhyInfo
	b	.L2578
.L2579:
	sub	r3, r0, #17
	uxtb	r3, r3
	cmp	r3, #2
	bhi	.L2585
	ldr	r3, .L2609+48
	cmp	r0, #19
	str	r3, [r5, #184]
	mov	r3, #7
	strb	r3, [r5, #152]
	moveq	r3, #15
	streqb	r3, [r5, #152]
	b	.L2578
.L2585:
	cmp	r0, #33
	cmpne	r0, #65
	beq	.L2586
	cmp	r0, #66
	bne	.L2587
.L2586:
	ldr	r3, .L2609
	mov	r1, #4
	ldr	r2, .L2609+52
	str	r2, [r3, #184]
	ldr	r2, .L2609+4
	strb	r1, [r2, #3760]
	mov	r2, #7
	strb	r2, [r3, #152]
	b	.L2578
.L2587:
	cmp	r0, #67
	cmpne	r0, #34
	beq	.L2588
	cmp	r0, #35
	beq	.L2588
	cmp	r0, #68
	bne	.L2589
.L2588:
	ldr	r3, .L2609
	cmp	r0, #35
	cmpne	r0, #68
	ldr	r2, .L2609+52
	sub	r0, r0, #67
	uxtb	r0, r0
	str	r2, [r3, #184]
	mov	r2, #7
	strb	r2, [r3, #152]
	moveq	r2, #17
	streqb	r2, [r3, #152]
	cmp	r0, #1
	ldr	r3, .L2609+4
	movls	r2, #4
	movhi	r2, #5
	strb	r2, [r3, #3760]
	b	.L2578
.L2589:
	cmp	r0, #49
	ldreq	r3, .L2609+56
	streq	r3, [r5, #184]
.L2578:
	ldr	r4, .L2609+4
	ldr	r3, .L2609+12
	ldr	r2, [r4, #3792]
	cmp	r2, r3
	bne	.L2592
	ldrb	r3, [r4, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, [r4, #3624]
	movne	r2, #0
	strneb	r2, [r3, #18]
.L2592:
	ldrb	r2, [r4, #2976]	@ zero_extendqisi2
	ldr	r3, .L2609+4
	cmp	r2, #44
	bne	.L2593
	ldrb	r2, [r3, #3780]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2593
	ldr	r1, [r3, #3792]
	ldr	r2, .L2609+12
	cmp	r1, r2
	bne	.L2594
	ldrb	r3, [r3, #3072]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2593
.L2594:
	ldr	r3, .L2609+4
	mov	r2, #0
	mov	r0, #1
	strb	r2, [r3, #3780]
	bl	FlashSetInterfaceMode
	mov	r0, #1
	bl	NandcSetMode
.L2593:
	ldrb	r3, [r4, #3772]	@ zero_extendqisi2
	tst	r3, #6
	beq	.L2595
	ldr	r2, .L2609+4
	ldrb	r2, [r2, #3780]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L2596
	tst	r3, #1
	bne	.L2595
.L2596:
	mov	r0, #0
	bl	flash_enter_slc_mode
	ldr	r3, .L2609
	mov	r0, #0
	ldr	r1, [r3, #172]
	bl	FlashDdrParaScan
	mov	r0, #0
	bl	flash_exit_slc_mode
.L2595:
	ldr	r4, .L2609+4
	mov	r8, #16
	movw	r6, #3058
	ldr	r3, [r4, #3624]
	ldrb	r0, [r3, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	add	r0, r4, #928
	bl	FlashReadIdbDataRaw
	ldr	r0, .L2609+60
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
	bne	.L2598
	mov	r7, r7, lsr #1
	mov	ip, ip, asl #1
	mov	r1, r1, asl #1
	strb	r8, [r4, #853]
	strh	r1, [r4, r2]	@ movhi
	mov	r2, #8
	strh	r7, [r4, r6]	@ movhi
	strh	ip, [r4, r0]	@ movhi
	strh	r2, [r4, r3]	@ movhi
.L2598:
	ldrb	r0, [r5, #20]	@ zero_extendqisi2
	bl	FlashBchSel
	bl	ftl_flash_suspend
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2599:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2600:
	mvn	r0, #1
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2610:
	.align	2
.L2609:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR1+3116
	.word	1446522928
	.word	.LANCHOR1
	.word	.LANCHOR1+3020
	.word	.LANCHOR0+896
	.word	.LC164
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
	ldr	r4, .L2615
	ldr	r6, .L2615+4
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
	beq	.L2612
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
	bne	.L2613
	bl	FtlInit
.L2613:
	ldr	r0, .L2615+8
	mov	r1, r4
	bl	printk
.L2612:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L2616:
	.align	2
.L2615:
	.word	.LANCHOR2
	.word	.LANCHOR4
	.word	.LC165
	.fnend
	.size	rk_ftl_init, .-rk_ftl_init
	.align	2
	.global	FtlReInitForSDUpdata
	.type	FtlReInitForSDUpdata, %function
FtlReInitForSDUpdata:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2642
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
	.pad #20
	sub	sp, sp, #20
	ldrb	r4, [r3, #3072]	@ zero_extendqisi2
	cmp	r4, #0
	movne	r0, #0
	bne	.L2618
	ldr	r5, .L2642+4
	ldr	r0, [r5, #144]
	bl	FlashInit
	cmp	r0, #0
	movne	r0, r4
	bne	.L2618
	bl	FlashLoadFactorBbt
	cmp	r0, #0
	beq	.L2619
	bl	FlashMakeFactorBbt
.L2619:
	ldr	r0, [r5, #148]
	bl	FlashReadIdbDataRaw
	cmp	r0, #0
	beq	.L2620
	mov	r1, #0
	mov	r2, #16
	mov	r0, sp
	bl	FlashReadFacBbtData
	ldr	r1, [sp, #0]
	mov	r3, #0
	mov	r0, #1
	mov	r2, r3
.L2622:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #16
	bne	.L2622
	cmp	r3, #6
	ldrls	r3, .L2642
	bls	.L2638
	mov	r2, #0
	mov	r0, #1
.L2623:
	ands	ip, r1, r0, asl r2
	add	r2, r2, #1
	addne	r3, r3, #1
	cmp	r2, #24
	bne	.L2623
	cmp	r3, #17
	ldr	r3, .L2642
	movhi	r2, #36
.L2638:
	strb	r2, [r3, #853]
	movw	r2, #3070
	ldr	r3, .L2642
	ldrb	r1, [r3, #853]	@ zero_extendqisi2
	strh	r1, [r3, r2]	@ movhi
.L2620:
	ldr	r1, .L2642+8
	mov	r4, #1
	ldr	r0, .L2642+12
	bl	printk
	ldr	r0, .L2642+16
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldr	r3, .L2642
	ldr	r0, [r3, #3856]
	uxth	r0, r0
	bl	FtlFreeSysBlkQueueInit
	b	.L2627
.L2629:
	add	r4, r4, #1
.L2627:
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L2640
.L2628:
	bl	FtlSysBlkInit
	cmp	r0, #0
	beq	.L2630
.L2640:
	bl	FtlLowFormat
	cmp	r4, #3
	bls	.L2629
	b	.L2641
.L2630:
	ldr	r3, .L2642+20
	mov	r2, #1
	str	r2, [r3, #3220]
	b	.L2618
.L2641:
	mvn	r0, #0
.L2618:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, pc}
.L2643:
	.align	2
.L2642:
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
	ldr	r3, .L2646
	ldr	r2, [r3, #3792]
	ldr	r3, .L2646+4
	cmp	r2, r3
	bxne	lr
	b	flash_enter_slc_mode
.L2647:
	.align	2
.L2646:
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
	ldr	r3, .L2650
	ldr	r2, [r3, #3792]
	ldr	r3, .L2650+4
	cmp	r2, r3
	bxne	lr
	b	flash_exit_slc_mode
.L2651:
	.align	2
.L2650:
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
	ldr	r5, .L2656
	mov	fp, r1
	.pad #28
	sub	sp, sp, #28
	mov	r1, r4
	mov	sl, r2
	ldr	r0, .L2656+4
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
	b	.L2653
.L2655:
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
	beq	.L2654
	ldr	r0, [r5, #3792]
	ldr	lr, .L2656+8
	cmp	r0, lr
	moveq	r3, r1
.L2654:
	ldr	r1, [sp, #16]
	add	r2, r2, r1
	mla	r2, r7, r3, r2
	ldr	r3, [r9, #3624]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	ldrb	r3, [r9, #3840]	@ zero_extendqisi2
	str	r3, [sp, #20]
	ldr	r3, .L2656+12
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
.L2653:
	cmp	r6, fp
	bcc	.L2655
	mov	r1, r4
	mov	r2, fp
	mov	r3, #0
	ldr	r0, .L2656+16
	bl	printk
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2657:
	.align	2
.L2656:
	.word	.LANCHOR0
	.word	.LC166
	.word	1446522928
	.word	.LANCHOR2
	.word	.LC167
	.fnend
	.size	IdBlockReadData, .-IdBlockReadData
	.section	.text.unlikely
	.align	2
	.type	IDBlockWriteData, %function
IDBlockWriteData:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2666
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r2
	ldr	r2, [r3, #3624]
	mov	r5, r0
	mov	fp, r1
	.pad #92
	sub	sp, sp, #92
	mov	r1, r5
	ldr	r0, .L2666+4
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
	bls	.L2659
	mov	r0, #0
	add	r1, r7, r5
	mov	r2, r0
	mov	r8, #2
	bl	FlashEraseBlock
.L2659:
	ldr	r4, .L2666
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
	b	.L2660
.L2664:
	add	r3, r8, r7
	ubfx	r3, r3, #2, #16
	cmp	r3, #0
	beq	.L2661
	add	r1, r3, #1
	ldrb	r0, [r4, #3072]	@ zero_extendqisi2
	add	r2, r4, r1, asl #1
	cmp	r0, #0
	add	r2, r2, #3072
	add	r2, r2, #4
	ldrh	r2, [r2, #0]
	beq	.L2662
	ldr	r0, [r4, #3792]
	ldr	ip, .L2666+8
	cmp	r0, ip
	moveq	r2, r1
.L2662:
	sub	r2, r2, #-1073741823
	mov	r2, r2, asl #2
	str	r2, [sp, #24]
.L2661:
	movw	r2, #61424
	str	r2, [sp, #28]
	add	r2, r4, r3, asl #1
	add	r2, r2, #3072
	ldrh	r5, [r2, #4]
	ldrb	r2, [r4, #3072]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2663
	ldr	r2, [r4, #3792]
	ldr	r1, .L2666+8
	cmp	r2, r1
	moveq	r5, r3
.L2663:
	ldr	ip, [sp, #12]
	add	r8, r8, #4
	ldr	r2, .L2666+12
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
.L2660:
	ldr	ip, [sp, #16]
	cmp	r8, ip
	bcc	.L2664
	ldr	r5, [sp, #20]
	mov	r2, fp
	mov	r3, #0
	ldr	r0, .L2666+16
	mov	r1, r5
	bl	printk
	mov	r0, #0
	add	sp, sp, #92
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2667:
	.align	2
.L2666:
	.word	.LANCHOR0
	.word	.LC168
	.word	1446522928
	.word	.LANCHOR2
	.word	.LC169
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
	ldr	r3, .L2693
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
	beq	.L2687
	add	r5, r5, #508
	add	r5, r5, #3
	mov	r5, r5, lsr #9
	cmp	r5, #8
	movls	r5, #8
	bls	.L2670
	cmp	r5, #500
	bhi	.L2687
.L2670:
	ldr	r2, [r6, #0]
	ldr	r3, .L2693+4
	cmp	r2, r3
	bne	.L2687
	mul	sl, sl, r4
	uxth	sl, sl
	sub	r0, sl, #1
	add	r0, r0, r5
	mov	r1, sl
	bl	__aeabi_uidiv
	mov	r3, #0
	movw	r2, #63871
	movw	ip, #4097
	str	r0, [sp, #20]
	add	r0, r6, #254976
	add	r0, r0, #512
.L2675:
	ldr	r1, [r0, #-4]!
	cmp	r1, #0
	bne	.L2671
	ldr	r1, [r6, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, ip
	sub	r2, r2, #1
	movcs	r3, #0
	cmp	r2, #4096
	str	r1, [r0, #512]
	bne	.L2675
	b	.L2674
.L2671:
	ldr	r0, .L2693+8
	bl	printk
.L2674:
	ldr	r4, .L2693
	mov	r1, r7
	mov	r3, #5
	ldr	r0, .L2693+12
	mov	r2, #4
	mov	r8, r5
	bl	rknand_print_hex
	ldrb	r2, [r4, #853]	@ zero_extendqisi2
	ldr	r1, [r6, #512]
	ldr	r0, .L2693+16
	bl	printk
	ldr	r2, .L2693+20
	movw	r3, #3070
	mov	r1, r5
	ldrh	r3, [r4, r3]
	ldr	r2, [r2, #168]
	ldr	r0, .L2693+24
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
.L2683:
	ldr	r1, .L2693
	ldr	r3, [r4], #4
	ldrb	r2, [r1, #853]	@ zero_extendqisi2
	cmp	r3, r2
	bcs	.L2677
	ldr	r1, .L2693+20
	ldr	r2, [r1, #168]
	cmp	r3, r2
	bcc	.L2677
	ldr	r2, [sp, #20]
	ldr	r1, [sp, #12]
	cmp	r2, #1
	movls	r2, #0
	movhi	r2, #1
	str	r2, [sp, #28]
	ldr	r2, [sp, #20]
	cmp	r1, #0
	cmpne	r2, #1
	bls	.L2678
	ldr	r2, [r4, #-8]
	add	r2, r2, #1
	cmp	r3, r2
	beq	.L2677
.L2678:
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
.L2682:
	mov	fp, r2
	mov	r5, r3
	ldr	r0, [fp, #0]
	add	r2, r2, #4
	ldr	r1, [r5, #0]
	add	r3, r3, #4
	cmp	r0, r1
	beq	.L2679
	mov	r0, r9
	mov	r1, #512
	bl	__memzero
	ldr	r3, [fp, #0]
	ldr	r1, [sp, #12]
	ldr	r0, .L2693+28
	str	r3, [sp, #0]
	ldr	r3, [r5, #0]
	bic	r5, r6, #255
	mov	r5, r5, asl #2
	str	r3, [sp, #4]
	mov	r3, r6
	ldr	r2, [r4, #-4]
	bl	printk
	ldr	r0, .L2693+32
	add	r1, r7, r5
	mov	r2, #4
	mov	r3, #256
	bl	rknand_print_hex
	mov	r3, #256
	add	r1, r9, r5
	mov	r2, #4
	ldr	r0, .L2693+36
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
	beq	.L2680
	ldr	r1, [r4, #-4]
	mov	r0, #0
	mov	r2, r0
	add	r1, r1, #1
	mul	r1, sl, r1
	bl	FlashEraseBlock
.L2680:
	mov	r0, #0
	bl	flash_boot_exit_slc_mode
	ldr	r1, [r4, #-4]
	ldr	r0, .L2693+40
	bl	printk
	ldr	r1, [sp, #24]
	cmp	r6, r1
	bcc	.L2677
	b	.L2681
.L2679:
	ldr	r1, [sp, #24]
	add	r6, r6, #1
	cmp	r6, r1
	bne	.L2682
.L2681:
	ldr	r2, [sp, #16]
	add	r2, r2, #1
	str	r2, [sp, #16]
.L2677:
	ldr	r3, [sp, #12]
	add	r3, r3, #1
	str	r3, [sp, #12]
	cmp	r3, #5
	bne	.L2683
	mov	r0, r9
	bl	ftl_free
	ldr	r1, [sp, #16]
	cmp	r1, #0
	mvneq	r0, #0
	movne	r0, #0
	b	.L2669
.L2687:
	mvn	r0, #0
.L2669:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2694:
	.align	2
.L2693:
	.word	.LANCHOR0
	.word	-52655045
	.word	.LC170
	.word	.LC171
	.word	.LC172
	.word	.LANCHOR2
	.word	.LC173
	.word	.LC174
	.word	.LC175
	.word	.LC176
	.word	.LC177
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
	ldr	ip, .L2698
	b	.L2696
.L2697:
	ldrb	r4, [r3, r2]	@ zero_extendqisi2
	add	r2, r2, #1
	eor	r4, r4, r0, lsr #24
	add	r4, ip, r4, asl #2
	ldr	r4, [r4, #3292]
	eor	r0, r4, r0, asl #8
.L2696:
	cmp	r2, r1
	bne	.L2697
	ldmfd	sp!, {r4, pc}
.L2699:
	.align	2
.L2698:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r4, r0
	.pad #48
	sub	sp, sp, #48
	mov	r6, r1
	mov	r7, r2
	bne	.L2701
	ldr	r2, [r2, #0]
	ldr	r3, .L2718
	cmp	r2, r3
	bne	.L2701
	ldr	r5, .L2718+4
	mov	r0, #256000
	mov	r3, #1
	strb	r3, [r5, #488]
	bl	ftl_malloc
	mov	r1, #0
	mov	r2, #256000
	str	r0, [r5, #492]
	bl	ftl_memset
	str	r4, [r5, #496]
.L2701:
	ldr	r5, .L2718+4
	mov	r3, r4
	str	r6, [sp, #0]
	ldr	r0, .L2718+8
	ldr	r1, [r5, #492]
	ldr	r2, [r7, #0]
	bl	printk
	ldrb	r3, [r5, #488]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2700
	sub	r0, r4, #64
	ldr	r8, [r5, #492]
	cmp	r0, #500
	bcs	.L2703
	rsb	r2, r4, #564
	add	r0, r8, r0, asl #9
	cmp	r2, r6
	movcs	r2, r6
	mov	r1, r7
	mov	r2, r2, asl #9
	bl	memcpy
	b	.L2704
.L2703:
	cmp	r4, #564
	bcc	.L2704
	ldr	r3, .L2718+12
	ldr	r0, [r5, #496]
	ldr	r3, [r3, #3624]
	sub	r0, r0, #64
	cmp	r0, #500
	movcs	r0, #500
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	cmp	r3, #4
	beq	.L2714
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
	b	.L2706
.L2714:
	mov	r1, #0
	mov	r2, r1
.L2705:
	add	r3, sp, #8
	cmp	r0, #256
	movhi	ip, r2, asl #1
	strls	r2, [r3, r1]
	add	r2, r2, #1
	strhi	ip, [r3, r1]
	cmp	r2, #5
	add	r1, r1, #4
	bne	.L2705
.L2706:
	movw	r3, #63872
.L2711:
	ldr	r2, [r8, r3, asl #2]
	cmp	r2, #0
	addne	r3, r3, #128
	movne	r0, r3, asl #2
	bne	.L2710
.L2709:
	sub	r3, r3, #1
	cmp	r3, #4096
	bne	.L2711
	mov	r0, r0, asl #9
.L2710:
	ldr	r5, .L2718+4
	mov	r1, r8
	add	r2, sp, #8
	mov	r7, #0
	bl	write_idblock
	strb	r7, [r5, #488]
	ldr	r0, [r5, #492]
	b	.L2717
.L2704:
	ldr	r5, .L2718+4
	ldr	r3, [r5, #496]
	cmp	r3, r4
	beq	.L2712
	mov	r7, #0
	cmp	r8, r7
	strb	r7, [r5, #488]
	beq	.L2713
	mov	r0, r8
.L2717:
	bl	ftl_free
.L2713:
	str	r7, [r5, #492]
.L2712:
	ldr	r3, .L2718+4
	add	r4, r6, r4
	str	r4, [r3, #496]
.L2700:
	add	sp, sp, #48
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2719:
	.align	2
.L2718:
	.word	-52655045
	.word	.LANCHOR4
	.word	.LC178
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
	beq	.L2721
	mov	r0, r1
	mov	r1, r2
	mov	r2, r6
	bl	write_loader_lba
.L2721:
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
	ldr	r3, .L2838
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	cmp	r1, r3
	.pad #528
	sub	sp, sp, #528
	mov	r5, r1
	mov	r4, r2
	beq	.L2731
	bhi	.L2740
	ldr	r3, .L2838+4
	cmp	r1, r3
	beq	.L2728
	bhi	.L2741
	sub	r3, r3, #125
	cmp	r1, r3
	beq	.L2725
	bhi	.L2742
	sub	r3, r3, #237
	cmp	r1, r3
	bne	.L2784
	b	.L2834
.L2742:
	ldr	r3, .L2838+8
	cmp	r1, r3
	beq	.L2726
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2784
	b	.L2835
.L2741:
	ldr	r3, .L2838+12
	cmp	r1, r3
	beq	.L2730
	bhi	.L2743
	ldr	r3, .L2838+16
	cmp	r1, r3
	bne	.L2784
	b	.L2836
.L2743:
	ldr	r3, .L2838+20
	cmp	r1, r3
	beq	.L2730
	add	r3, r3, #10
	cmp	r1, r3
	bne	.L2784
	b	.L2730
.L2740:
	ldr	r3, .L2838+24
	cmp	r1, r3
	beq	.L2736
	bhi	.L2744
	sub	r3, r3, #78
	cmp	r1, r3
	beq	.L2733
	bcc	.L2732
	add	r3, r3, #21
	cmp	r1, r3
	beq	.L2734
	add	r3, r3, #56
	cmp	r1, r3
	bne	.L2784
	b	.L2837
.L2744:
	ldr	r3, .L2838+28
	cmp	r1, r3
	beq	.L2739
	bhi	.L2745
	sub	r3, r3, #956
	sub	r3, r3, #1
	cmp	r1, r3
	beq	.L2737
	add	r3, r3, #956
	cmp	r1, r3
	bne	.L2784
	b	.L2738
.L2745:
	ldr	r3, .L2838+32
	cmp	r1, r3
	beq	.L2738
	add	r3, r3, #1
	cmp	r1, r3
	bne	.L2784
	b	.L2739
.L2732:
	ldr	r0, .L2838+36
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2791
	mov	r1, r4
	mov	r2, #512
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2833
.L2746:
	ldr	r0, .L2838+40
	ldmia	r5, {r1, r2}
	bl	printk
	ldr	r3, [r5, #4]
	cmp	r3, #8
	str	r3, [sp, #4]
	bhi	.L2755
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
	beq	.L2819
	ldr	r0, .L2838+44
.L2832:
	bl	printk
	b	.L2755
.L2733:
	ldr	r0, .L2838+48
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2791
	mov	r1, r4
	mov	r2, #4096
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2833
.L2750:
	ldr	r4, .L2838+52
	ldr	r0, .L2838+56
	ldmia	r5, {r1, r2}
	bl	printk
	ldr	r3, [r4, #500]
	cmp	r3, #0
	bne	.L2751
	mov	r0, #260096
	bl	ftl_malloc
	cmp	r0, #0
	str	r0, [r4, #500]
	beq	.L2755
.L2752:
	mov	r1, #260096
	bl	__memzero
.L2751:
	ldr	r2, [r5, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L2755
	ldr	r3, [r5, #0]
	cmp	r3, #251904
	bhi	.L2755
	ldr	r1, .L2838+52
	ldr	r0, [r1, #500]
	add	r1, r5, #8
	add	r0, r0, r3
	bl	memcpy
.L2819:
	mov	r0, r5
	bl	ftl_free
.L2820:
	mov	r4, #0
	b	.L2749
.L2837:
	ldr	r0, .L2838+60
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2791
	mov	r1, r4
	mov	r2, #28
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L2754
.L2833:
	ldr	r0, .L2838+64
	b	.L2832
.L2754:
	ldmia	r5, {r1, r2}
	ldr	r0, .L2838+68
	bl	printk
	ldr	r1, [r5, #0]
	cmp	r1, #256000
	bhi	.L2755
	ldr	r4, .L2838+52
	ldr	r0, [r4, #500]
	cmp	r0, #0
	beq	.L2755
.L2756:
	bl	CRC_32
	ldr	r3, [r5, #4]
	cmp	r3, r0
	beq	.L2757
	mov	r0, r5
	bl	ftl_free
	b	.L2797
.L2757:
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
	b	.L2749
.L2755:
	mov	r0, r5
	b	.L2823
.L2734:
	ldr	r0, .L2838+72
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2791
	bl	ftl_read_flash_info
	mov	r0, r4
	mov	r1, r5
	mov	r2, #11
	b	.L2828
.L2731:
	ldr	r0, .L2838+76
	bl	printk
	bl	nand_blk_add_whole_disk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2791
	mov	r1, #0
	mov	r2, #64
	bl	ftl_memset
	mov	r0, r4
	mov	r1, r5
	mov	r2, #64
	b	.L2828
.L2736:
	ldr	r0, .L2838+80
	bl	printk
	ldr	r3, .L2838+52
	add	r1, sp, #528
	mov	r0, r4
	mov	r2, #4
	ldr	r3, [r3, #504]
	ldr	r3, [r3, #20]
	str	r3, [r1, #-524]!
	b	.L2824
.L2737:
	ldr	r0, .L2838+84
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r5, r0, #0
	beq	.L2791
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
.L2828:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	mov	r0, r5
	beq	.L2760
.L2823:
	bl	ftl_free
	b	.L2814
.L2760:
	bl	ftl_free
	b	.L2749
.L2725:
	ldr	r0, .L2838+88
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r5, r0, #0
	bne	.L2825
.L2761:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+92
	cmp	r2, r3
	bne	.L2816
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2816
	ldr	r6, .L2838+52
	mov	r2, #512
	add	r0, sp, #8
	ldr	r1, [r6, #504]
	bl	memcpy
	ldr	r2, [r6, #508]
	ldr	r3, .L2838+96
	cmp	r2, r3
	beq	.L2762
	add	r0, sp, #72
	mov	r1, #128
	str	r5, [sp, #16]
	str	r5, [sp, #20]
	bl	__memzero
.L2762:
	add	r0, sp, #264
	mov	r1, #256
	mov	r3, #0
	str	r3, [sp, #24]
	bl	__memzero
	b	.L2831
.L2726:
	ldr	r0, .L2838+100
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2825
.L2763:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+92
	cmp	r2, r3
	bne	.L2816
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2816
	ldr	r4, .L2838+52
	ldr	r3, .L2838+96
	ldr	r2, [r4, #508]
	cmp	r2, r3
	bne	.L2797
	ldr	r3, [sp, #20]
	sub	r2, r3, #1
	cmp	r2, #127
	mvnhi	r4, #2
	bhi	.L2723
	ldr	r0, [r4, #504]
	add	r1, sp, #72
	str	r3, [r0, #12]
	add	r0, r0, #64
	ldr	r2, [sp, #20]
	bl	memcpy
	mov	r0, #1
	ldr	r1, [r4, #504]
	b	.L2821
.L2836:
	ldr	r0, .L2838+104
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2825
.L2764:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+108
	cmp	r2, r3
	bne	.L2816
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2816
	ldr	r5, .L2838+52
	ldr	r4, [r5, #512]
	cmp	r4, #0
	beq	.L2723
	ldr	r3, [r5, #516]
	ldr	r2, .L2838+112
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2765
	str	r2, [r3, #0]
	mov	r2, #504
	ldr	r3, [r5, #516]
	str	r2, [r3, #4]
	mov	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L2765:
	ldr	r1, [r5, #516]
	mov	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #504]
	ldr	r2, .L2838+92
	ldr	r5, .L2838+52
	ldr	r1, [r3, #0]
	cmp	r1, r2
	strne	r2, [r3, #0]
	ldr	r0, [r5, #504]
	mov	r1, #128
	ldrne	r3, .L2838+52
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
	b	.L2749
.L2728:
	ldr	r0, .L2838+116
	bl	printk
	mov	r1, r4
	add	r0, sp, #8
	mov	r2, #520
	bl	rk_copy_from_user
	subs	r4, r0, #0
	bne	.L2825
.L2767:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+120
	cmp	r2, r3
	bne	.L2816
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L2816
	ldr	r5, .L2838+52
	ldr	r3, [r5, #512]
	cmp	r3, #1
	beq	.L2723
	ldr	r3, [r5, #516]
	mov	r0, #0
	ldr	r2, .L2838+112
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
	ldr	r2, .L2838+92
	ldr	r1, [r3, #0]
	cmp	r1, r2
	beq	.L2769
	str	r2, [r3, #0]
	mov	lr, #0
	ldr	r3, .L2838+52
	mov	r2, #504
	ldr	r3, [r3, #504]
	stmib	r3, {r2, lr}
.L2769:
	ldr	r5, .L2838+52
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
	b	.L2749
.L2835:
	ldr	r0, .L2838+124
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2825
.L2770:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+128
	cmp	r2, r3
	bne	.L2816
	ldr	r2, [sp, #12]
	cmp	r2, #512
	addls	r0, sp, #16
	ldrls	r1, .L2838+132
	bls	.L2830
	b	.L2816
.L2730:
	ldr	r3, .L2838+20
	cmp	r5, r3
	ldreq	r0, .L2838+136
	beq	.L2817
	ldr	r3, .L2838+140
	cmp	r5, r3
	ldreq	r0, .L2838+144
	ldrne	r0, .L2838+148
.L2817:
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2825
.L2774:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+152
	cmp	r2, r3
	bne	.L2814
	ldr	r3, .L2838+140
	ldr	r6, .L2838+52
	cmp	r5, r3
	bne	.L2775
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
	b	.L2723
.L2775:
	ldr	r3, [r6, #1032]
	cmp	r3, #10
	bhi	.L2814
	ldr	r3, [r6, #504]
	ldr	r1, [sp, #12]
	ldr	r2, [r3, #24]
	cmp	r2, r1
	beq	.L2776
	cmp	r2, #0
	beq	.L2776
	ldr	r0, .L2838+156
	bl	printk
	ldr	r3, [r6, #1032]
	add	r3, r3, #1
	str	r3, [r6, #1032]
	b	.L2814
.L2776:
	ldr	r0, .L2838+52
	mov	r2, #0
	str	r2, [r0, #1032]
	ldr	r0, .L2838+20
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
	b	.L2749
.L2738:
	ldr	r0, .L2838+160
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2825
.L2779:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+164
	cmp	r2, r3
	bne	.L2816
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L2816
	ldr	r3, .L2838+168
	add	r0, sp, #16
	cmp	r5, r3
	ldr	r3, .L2838+52
	ldreq	r1, [r3, #1036]
	ldrne	r1, [r3, #1040]
	add	r1, r1, #8
.L2830:
	bl	memcpy
.L2831:
	add	r1, sp, #8
	mov	r0, r4
	mov	r2, #520
.L2824:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	bne	.L2814
	b	.L2749
.L2739:
	ldr	r0, .L2838+172
	bl	printk
	add	r0, sp, #8
	mov	r1, r4
	mov	r2, #520
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L2782
.L2825:
	ldr	r0, .L2838+64
	bl	printk
	b	.L2814
.L2782:
	ldr	r2, [sp, #8]
	ldr	r3, .L2838+164
	cmp	r2, r3
	bne	.L2816
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L2816
	ldr	r3, .L2838+28
	add	r2, r2, #8
	ldr	r4, .L2838+52
	cmp	r5, r3
	bne	.L2783
	add	r1, sp, #8
	ldr	r0, [r4, #1036]
	bl	memcpy
	ldr	r1, [r4, #1036]
	mov	r0, #2
.L2821:
	bl	StorageSysDataStore
	mov	r4, r0
	b	.L2749
.L2783:
	add	r1, sp, #8
	ldr	r0, [r4, #1040]
	bl	memcpy
	mov	r0, #3
	ldr	r1, [r4, #1040]
	b	.L2821
.L2834:
	bl	rknand_dev_flush
	b	.L2820
.L2816:
	mvn	r4, #0
.L2749:
	mov	r1, r4
	ldr	r0, .L2838+176
	bl	printk
	b	.L2723
.L2784:
	mvn	r4, #21
	b	.L2723
.L2791:
	mvn	r4, #11
	b	.L2723
.L2797:
	mvn	r4, #1
	b	.L2723
.L2814:
	mvn	r4, #13
.L2723:
	mov	r0, r4
	add	sp, sp, #528
	ldmfd	sp!, {r4, r5, r6, pc}
.L2839:
	.align	2
.L2838:
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
	ldr	r4, .L2843
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
	beq	.L2841
	mov	r0, r5
	mov	r1, #508
	bl	js_hash
	cmp	r7, r0
	beq	.L2841
	str	r6, [r5, #16]
	ldr	r0, .L2843+4
	str	r6, [r4, #512]
	bl	printk
.L2841:
	ldr	r3, [r4, #512]
	mov	r0, #2
	ldr	r4, .L2843
	cmp	r3, #0
	ldrne	r2, .L2843+8
	ldrne	r3, .L2843
	ldr	r1, [r4, #1036]
	strne	r2, [r3, #508]
	bl	StorageSysDataLoad
	ldr	r1, [r4, #1040]
	mov	r0, #3
	bl	StorageSysDataLoad
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	rknand_sys_storage_init
.L2844:
	.align	2
.L2843:
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
	ldr	r7, .L2854
	cmp	r0, #0
	str	r0, [r7, #1044]
	beq	.L2852
	ldr	sl, .L2854+4
	mov	r6, #0
	mov	r4, r6
	mov	r5, r6
	movw	r8, #65532
.L2850:
	ldr	r0, [r7, #1044]
	mov	r1, r5, asl #7
	mov	r2, #0
	bl	rk_ftl_vendor_ops.constprop.29
	cmp	r0, #0
	bne	.L2848
	ldr	r3, [r7, #1044]
	ldr	r0, .L2854+8
	ldr	r1, [r3, #0]
	ldr	r2, [r3, r8]
	ldr	r3, [r3, #4]
	bl	printk
	ldr	r0, [r7, #1044]
	ldr	r3, [r0, #0]
	cmp	r3, sl
	bne	.L2849
	ldr	r3, [r0, r8]
	ldr	r2, [r0, #4]
	cmp	r3, r2
	bne	.L2849
	cmp	r4, r3
	movcc	r6, r5
	movcc	r4, r3
.L2849:
	add	r5, r5, #1
	cmp	r5, #2
	bne	.L2850
	cmp	r4, #0
	beq	.L2851
	mov	r1, r6, asl #7
	mov	r2, #0
	bl	rk_ftl_vendor_ops.constprop.29
	cmp	r0, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
	b	.L2848
.L2851:
	mov	r1, #65536
	bl	__memzero
	ldr	r3, .L2854
	ldr	r1, .L2854+4
	mov	r2, #1
	mov	r0, r4
	ldr	r3, [r3, #1044]
	stmia	r3, {r1, r2}
	movw	r1, #65532
	str	r2, [r3, r1]
	ldr	r2, .L2854+12
	strh	r4, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2848:
	ldr	r4, .L2854
	ldr	r0, [r4, #1044]
	bl	kfree
	mov	r3, #0
	mvn	r0, #0
	str	r3, [r4, #1044]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2852:
	mvn	r0, #11
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L2855:
	.align	2
.L2854:
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
	ldr	r1, .L2863
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	ip, [r1, #1044]
	cmp	ip, #0
	ldrneh	r4, [ip, #10]
	movne	r5, ip
	movne	r1, #0
	bne	.L2858
	b	.L2862
.L2860:
	ldrh	r6, [r5, #16]
	add	r5, r5, #8
	cmp	r6, r3
	bne	.L2859
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
.L2859:
	add	r1, r1, #1
.L2858:
	cmp	r1, r4
	bcc	.L2860
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2862:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L2864:
	.align	2
.L2863:
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
	ldr	r3, .L2882
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	mov	sl, r0
	ldr	r4, [r3, #1044]
	.pad #20
	sub	sp, sp, #20
	mov	r9, r1
	mov	r5, r2
	cmp	r4, #0
	beq	.L2878
	add	r8, r2, #63
	ldrh	r2, [r4, #8]
	ldrh	r3, [r4, #10]
	bic	r8, r8, #63
	mov	r7, #0
	str	r2, [sp, #8]
	b	.L2867
.L2874:
	add	r2, r7, #2
	mov	r2, r2, asl #3
	add	r6, r4, r2
	ldrh	r2, [r4, r2]
	cmp	r2, sl
	bne	.L2868
	ldrh	r2, [r6, #4]
	add	r2, r2, #63
	bic	r2, r2, #63
	str	r2, [sp, #12]
	cmp	r5, r2
	bls	.L2869
	ldrh	r2, [r4, #14]
	cmp	r2, r8
	bcc	.L2878
	sub	r3, r3, #1
	mov	fp, r8
	ldrh	r6, [r6, #2]
	mov	r8, r3
	b	.L2870
.L2871:
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
.L2870:
	cmp	r7, r8
	add	r2, r7, #2
	bcc	.L2871
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
	b	.L2881
.L2869:
	ldrh	r0, [r6, #2]
	mov	r1, r9
	mov	r2, r5
	add	r0, r0, #1024
	add	r0, r4, r0
	bl	memcpy
	strh	r5, [r6, #4]	@ movhi
	b	.L2881
.L2868:
	add	r7, r7, #1
.L2867:
	cmp	r7, r3
	bcc	.L2874
	ldrh	r2, [r4, #14]
	cmp	r2, r8
	bcc	.L2878
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
.L2881:
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
	b	.L2866
.L2878:
	mvn	r0, #0
.L2866:
	add	sp, sp, #20
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L2883:
	.align	2
.L2882:
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
	beq	.L2885
	ldr	r3, .L2901
	cmp	r6, r3
	beq	.L2887
	add	r3, r3, #1
	cmp	r6, r3
	bne	.L2898
	b	.L2900
.L2887:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2898
	ldr	r2, [r4, #0]
	ldr	r3, .L2901+4
	cmp	r2, r3
	bne	.L2897
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_read
	cmn	r0, #1
	beq	.L2893
	uxth	r2, r0
	mov	r1, r4
	strh	r2, [r4, #6]	@ movhi
	mov	r0, r5
	add	r2, r2, #8
	bl	rk_copy_to_user
	cmp	r0, #0
	moveq	r5, r0
	mvnne	r5, #13
	b	.L2886
.L2900:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2898
	ldr	r2, [r4, #0]
	ldr	r3, .L2901+4
	cmp	r2, r3
	bne	.L2897
	ldrh	r2, [r4, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L2897
	mov	r0, r4
	mov	r1, r5
	add	r2, r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L2898
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_write
.L2893:
	mov	r5, r0
	b	.L2886
.L2897:
	mvn	r5, #0
	b	.L2886
.L2898:
	mvn	r5, #13
.L2886:
	mov	r0, r4
	bl	kfree
.L2885:
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L2902:
	.align	2
.L2901:
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
	.ascii	"FTL version: 5.0.54 20190402\000"
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
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC152:
	.ascii	"rk_ftl_de_init %x\012\000"
.LC153:
	.ascii	"fix power lost blk = %x vpc=%x\012\000"
.LC154:
	.ascii	"erase power lost blk = %x vpc=%x\012\000"
.LC155:
	.ascii	"...%s: no bad block mapping table, format device\012"
	.ascii	"\000"
.LC156:
	.ascii	"...%s FtlSysBlkInit error ,format device!\012\000"
.LC157:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
.LC158:
	.ascii	"vpc1\000"
.LC159:
	.ascii	"vpc2\000"
.LC160:
	.ascii	"FtlCheckVpc2 %x = %x  %x\012\000"
.LC161:
	.ascii	"free blk vpc error %x = %x  %x\012\000"
.LC162:
	.ascii	"otp error! %d\000"
.LC163:
	.ascii	"rr\000"
.LC164:
	.ascii	"FlashLoadPhyInfo fail %x!!\012\000"
.LC165:
	.ascii	"FtlInit %x\012\000"
.LC166:
	.ascii	"IdBlockReadData %x %x\012\000"
.LC167:
	.ascii	"IdBlockReadData %x %x ret= %x\012\000"
.LC168:
	.ascii	"IDBlockWriteData %x %x\012\000"
.LC169:
	.ascii	"IDBlockWriteData %x %x ret= %x\012\000"
.LC170:
	.ascii	"write_idblock fix data %x %x\012\000"
.LC171:
	.ascii	"idblk:\000"
.LC172:
	.ascii	"idb reverse %x %x\012\000"
.LC173:
	.ascii	"write_idblock totle_sec %x %x %x %x\012\000"
.LC174:
	.ascii	"write and check error:%d idb=%x,offset=%x,r=%x,w=%x"
	.ascii	"\012\000"
.LC175:
	.ascii	"write\000"
.LC176:
	.ascii	"read\000"
.LC177:
	.ascii	"write_idblock error %d\012\000"
.LC178:
	.ascii	"wl_lba %p %x %x %x\012\000"
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
