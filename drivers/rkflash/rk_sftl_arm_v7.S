/* SPDX-License-Identifier: GPL-2.0 */

/* Copyright (c) 2018 Rockchip Electronics Co. Ltd.
 * date: 2020-09-29
 */
	.file	"rk_sftl.c"
	.global	__udivsi3
	.text
	.align	2
	.syntax unified
	.arm
	.fpu softvfp
	.type	l2p_addr_tran, %function
l2p_addr_tran:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L3
	mov	r8, r1
	mov	r9, r2
	ldr	r6, [r0, #4]
	ldrh	r4, [r3, #8]
	ldrh	r5, [r3, #10]
	ldrh	r3, [r3, #14]
	lsr	r7, r6, #10
	ubfx	r6, r6, #0, #10
	cmp	r3, #4
	uxth	r0, r7
	lsreq	r4, r4, #1
	lsleq	r5, r5, #1
	uxth	r7, r7
	mov	r1, r4
	uxtheq	r5, r5
	bl	__udivsi3
	uxth	r0, r0
	mls	r4, r0, r4, r7
	mla	r4, r5, r4, r6
	str	r4, [r8]
	str	r0, [r9]
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, fp, sp, pc}
.L4:
	.align	2
.L3:
	.word	.LANCHOR0
	.size	l2p_addr_tran, .-l2p_addr_tran
	.align	2
	.global	ftl_print_sblk_info
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_print_sblk_info, %function
ftl_print_sblk_info:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #12
	ldr	r4, .L6
	ldr	r1, .L6+4
	ldr	r0, .L6+8
	bl	sftl_printk
	ldrh	r1, [r4, #24]
	ldr	ip, [r4, #72]
	ldrb	r3, [r4, #30]	@ zero_extendqisi2
	ldrh	r2, [r4, #26]
	lsl	r0, r1, #1
	ldrh	r0, [ip, r0]
	str	r0, [sp, #8]
	ldrh	r0, [r4, #28]
	str	r0, [sp, #4]
	ldrb	r0, [r4, #32]	@ zero_extendqisi2
	str	r0, [sp]
	ldr	r0, .L6+12
	bl	sftl_printk
	ldrh	r1, [r4, #76]
	ldr	ip, [r4, #72]
	ldrb	r3, [r4, #82]	@ zero_extendqisi2
	ldrh	r2, [r4, #78]
	lsl	r0, r1, #1
	ldrh	r0, [ip, r0]
	str	r0, [sp, #8]
	ldrh	r0, [r4, #80]
	str	r0, [sp, #4]
	ldrb	r0, [r4, #84]	@ zero_extendqisi2
	str	r0, [sp]
	ldr	r0, .L6+16
	bl	sftl_printk
	ldrh	r1, [r4, #124]
	ldr	ip, [r4, #72]
	ldrb	r3, [r4, #130]	@ zero_extendqisi2
	ldrh	r2, [r4, #126]
	lsl	r0, r1, #1
	ldrh	r0, [ip, r0]
	str	r0, [sp, #8]
	ldrh	r0, [r4, #128]
	str	r0, [sp, #4]
	ldrb	r0, [r4, #132]	@ zero_extendqisi2
	str	r0, [sp]
	ldr	r0, .L6+20
	bl	sftl_printk
	ldrh	r1, [r4, #172]
	ldr	ip, [r4, #72]
	ldrb	r3, [r4, #178]	@ zero_extendqisi2
	ldrh	r2, [r4, #174]
	lsl	r0, r1, #1
	ldrh	r0, [ip, r0]
	str	r0, [sp, #8]
	ldrh	r0, [r4, #176]
	str	r0, [sp, #4]
	ldrb	r0, [r4, #180]	@ zero_extendqisi2
	str	r0, [sp]
	ldr	r0, .L6+24
	bl	sftl_printk
	ldrh	r3, [r4, #220]
	ldrh	r2, [r4, #222]
	ldrh	r1, [r4, #224]
	ldr	r0, .L6+28
	bl	sftl_printk
	sub	sp, fp, #16
	ldmfd	sp, {r4, fp, sp, pc}
.L7:
	.align	2
.L6:
	.word	.LANCHOR0
	.word	.LC0
	.word	.LC1
	.word	.LC2
	.word	.LC3
	.word	.LC4
	.word	.LC5
	.word	.LC6
	.size	ftl_print_sblk_info, .-ftl_print_sblk_info
	.align	2
	.global	Ftl_log2
	.syntax unified
	.arm
	.fpu softvfp
	.type	Ftl_log2, %function
Ftl_log2:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r1, #0
	mov	r2, #1
.L9:
	cmp	r2, r0
	uxth	r3, r1
	add	r1, r1, #1
	bls	.L10
	sub	r0, r3, #1
	uxth	r0, r0
	ldmfd	sp, {fp, sp, pc}
.L10:
	lsl	r2, r2, #1
	b	.L9
	.size	Ftl_log2, .-Ftl_log2
	.align	2
	.global	FtlPrintInfo
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlPrintInfo, %function
FtlPrintInfo:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldmfd	sp, {fp, sp, pc}
	.size	FtlPrintInfo, .-FtlPrintInfo
	.align	2
	.global	FtlSysBlkNumInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlSysBlkNumInit, %function
FtlSysBlkNumInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L13
	uxth	r0, r0
	ldrh	r2, [r3, #232]
	cmp	r0, #24
	ldrh	r1, [r3, #242]
	movcc	r0, #24
	str	r0, [r3, #228]
	mul	r2, r0, r2
	sub	r0, r1, r0
	ldr	r1, [r3, #248]
	strh	r0, [r3, #240]	@ movhi
	mov	r0, #0
	str	r2, [r3, #236]
	sub	r2, r1, r2
	str	r2, [r3, #244]
	ldmfd	sp, {fp, sp, pc}
.L14:
	.align	2
.L13:
	.word	.LANCHOR0
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.global	__divsi3
	.align	2
	.global	FtlConstantsInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlConstantsInit, %function
FtlConstantsInit:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #8
	mov	r3, r0
	ldrh	ip, [r3, #14]
	movw	r5, #258
	ldr	r4, .L29
	ldrh	r0, [r0]
	ldrh	r6, [r3, #2]
	cmp	ip, #4
	ldrh	lr, [r3, #4]
	add	r1, r4, #256
	ldrh	r2, [r3, #6]
	strh	r0, [r4, #252]	@ movhi
	strh	r6, [r4, #254]	@ movhi
	strh	lr, [r1]	@ movhi
	strh	r2, [r4, #242]	@ movhi
	strh	ip, [r4, r5]	@ movhi
	bne	.L16
	lsr	r2, r2, #1
	strh	r2, [r4, #242]	@ movhi
	mov	r2, #8
	strh	r2, [r4, r5]	@ movhi
.L17:
	ldr	ip, .L29+4
	mov	r2, #0
.L18:
	strb	r2, [r2, ip]
	add	r2, r2, #1
	cmp	r2, #32
	bne	.L18
	ldr	r2, .L29+8
	cmp	r0, #1
	mov	ip, #5
	ldrh	r5, [r4, #242]
	ldr	r8, .L29+12
	mov	lr, #0
	strh	ip, [r2]	@ movhi
	movw	ip, #294
	strheq	r0, [r2]	@ movhi
	mov	r2, #640
	ldrh	r0, [r1]
	mov	r7, r8
	strh	r2, [r7], #4	@ movhi
	movw	r2, #298
	strh	lr, [r4, ip]	@ movhi
	str	r3, [fp, #-44]
	smulbb	r6, r6, r0
	smulbb	r0, r0, r5
	uxth	r6, r6
	uxth	r0, r0
	strh	r6, [r4, #232]	@ movhi
	strh	r0, [r4, r2]	@ movhi
	bl	Ftl_log2
	ldr	r3, [fp, #-44]
	movw	r2, #302
	strh	r0, [r7]	@ movhi
	mov	r7, r8
	add	r8, r8, #24
	ldrh	r10, [r3, #12]
	str	r3, [fp, #-48]
	strh	r10, [r4, r2]	@ movhi
	movw	r2, #306
	smulbb	r1, r6, r10
	strh	r10, [r8, #-16]	@ movhi
	strh	r1, [r4, r2]	@ movhi
	movw	r2, #258
	ldrh	r1, [r4, r2]
	mov	r0, r1
	str	r1, [fp, #-44]
	bl	Ftl_log2
	ldr	r1, [fp, #-44]
	mov	r9, r0
	strh	r0, [r7, #12]!	@ movhi
	movw	r0, #310
	ldr	r3, [fp, #-48]
	lsl	r2, r1, #9
	uxth	r2, r2
	strh	r2, [r4, r0]	@ movhi
	lsr	r2, r2, #8
	mov	r0, #5120
	strh	r2, [r8, #-8]	@ movhi
	ldrh	r2, [r3, #20]
	movw	r3, #314
	strh	r2, [r4, r3]	@ movhi
	mul	r3, r5, r6
	lsl	r5, r5, #6
	str	r3, [r4, #248]
	mul	r3, r1, r3
	mul	r1, r10, r1
	mul	r3, r10, r3
	asr	r3, r3, #11
	str	r3, [r4, #316]
	bl	__divsi3
	uxth	r0, r0
	movw	r2, #322
	mov	r1, r6
	cmp	r0, #4
	movls	r3, #4
	strhhi	r0, [r8]	@ movhi
	strhls	r3, [r8]	@ movhi
	mov	r3, #640
	asr	r3, r3, r9
	add	r9, r9, #9
	asr	r5, r5, r9
	ldrh	r0, [r8]
	add	r3, r3, #2
	strh	r3, [r4, r2]	@ movhi
	ldr	r3, .L29+16
	strh	r5, [r3]	@ movhi
	uxth	r5, r5
	mul	r3, r6, r5
	add	r5, r5, #8
	str	r3, [r4, #328]
	bl	__udivsi3
	uxtah	r0, r5, r0
	cmp	r6, #1
	addeq	r0, r0, #4
	str	r0, [r4, #228]
	ldrh	r0, [r4, #228]
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #228]
	mov	r0, #0
	str	r0, [r4, #340]
	str	r3, [r4, #332]
	ldr	r3, [r4, #244]
	lsl	r2, r3, #2
	movw	r3, #302
	ldrh	r3, [r4, r3]
	mul	r3, r3, r2
	ldrh	r2, [r7]
	add	r2, r2, #9
	lsr	r3, r3, r2
	ldr	r2, .L29+20
	add	r3, r3, #2
	strh	r3, [r2]	@ movhi
	mov	r2, #32
	movw	r3, #338
	strh	r2, [r4, r3]	@ movhi
	ldrh	r3, [r8]
	add	r3, r3, #3
	strh	r3, [r8]	@ movhi
	ldr	r3, [r4, #328]
	add	r3, r3, #3
	str	r3, [r4, #328]
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L16:
	cmp	lr, #1
	cmpeq	ip, #8
	lsreq	r2, r2, #1
	strheq	r2, [r4, #242]	@ movhi
	moveq	r2, #2
	strheq	r2, [r1]	@ movhi
	b	.L17
.L30:
	.align	2
.L29:
	.word	.LANCHOR0
	.word	.LANCHOR0+260
	.word	.LANCHOR0+292
	.word	.LANCHOR0+296
	.word	.LANCHOR0+324
	.word	.LANCHOR0+336
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	2
	.global	IsBlkInVendorPart
	.syntax unified
	.arm
	.fpu softvfp
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L37
	uxth	r1, r0
	add	r2, r3, #344
	ldrh	r0, [r2]
	cmp	r0, #0
	ldmfdeq	sp, {fp, sp, pc}
	ldr	r2, [r3, #348]
	add	r3, r3, #320
	ldrh	r3, [r3]
	add	r3, r2, r3, lsl #1
.L33:
	cmp	r2, r3
	bne	.L34
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L34:
	ldrh	r0, [r2], #2
	cmp	r1, r0
	bne	.L33
	mov	r0, #1
	ldmfd	sp, {fp, sp, pc}
.L38:
	.align	2
.L37:
	.word	.LANCHOR0
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheWriteBack
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	2
	.global	sftl_get_density
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_get_density, %function
sftl_get_density:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L41
	ldr	r0, [r3, #340]
	ldmfd	sp, {fp, sp, pc}
.L42:
	.align	2
.L41:
	.word	.LANCHOR0
	.size	sftl_get_density, .-sftl_get_density
	.global	__umodsi3
	.align	2
	.global	FtlBbmMapBadBlock
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	movw	r3, #298
	ldr	r4, .L44
	uxth	r5, r0
	mov	r0, r5
	ldrh	r7, [r4, r3]
	mov	r1, r7
	bl	__udivsi3
	uxth	r6, r0
	mov	r1, r7
	mov	r0, r5
	bl	__umodsi3
	add	r2, r4, r6, lsl #2
	uxth	r3, r0
	ldr	r2, [r2, #380]
	lsr	r0, r3, #5
	and	ip, r3, #31
	mov	lr, #1
	add	r4, r4, #352
	ldr	r1, [r2, r0, lsl #2]
	orr	r1, r1, lr, lsl ip
	str	r1, [r2, r0, lsl #2]
	mov	r2, r6
	str	r1, [sp]
	mov	r1, r5
	ldr	r0, .L44+4
	bl	sftl_printk
	ldrh	r3, [r4, #6]
	mov	r0, #0
	add	r3, r3, #1
	strh	r3, [r4, #6]	@ movhi
	ldmib	sp, {r4, r5, r6, r7, fp, sp, pc}
.L45:
	.align	2
.L44:
	.word	.LANCHOR0
	.word	.LC7
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.align	2
	.global	FtlBbmIsBadBlock
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlBbmIsBadBlock, %function
FtlBbmIsBadBlock:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r5, .L47
	movw	r3, #298
	uxth	r6, r0
	ldrh	r7, [r5, r3]
	mov	r0, r6
	mov	r1, r7
	bl	__umodsi3
	mov	r1, r7
	uxth	r4, r0
	mov	r0, r6
	bl	__udivsi3
	uxth	r0, r0
	lsr	r2, r4, #5
	add	r5, r5, r0, lsl #2
	and	r4, r4, #31
	ldr	r3, [r5, #380]
	ldr	r0, [r3, r2, lsl #2]
	lsr	r0, r0, r4
	and	r0, r0, #1
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L48:
	.align	2
.L47:
	.word	.LANCHOR0
	.size	FtlBbmIsBadBlock, .-FtlBbmIsBadBlock
	.align	2
	.global	FtlBbtInfoPrint
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlBbtInfoPrint, %function
FtlBbtInfoPrint:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldmfd	sp, {fp, sp, pc}
	.size	FtlBbtInfoPrint, .-FtlBbtInfoPrint
	.align	2
	.global	FtlBbtMemInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r0, .L51
	mvn	r2, #0
	mov	r1, #255
	add	r3, r0, #352
	add	r0, r0, #364
	strh	r2, [r3]	@ movhi
	mov	r2, #0
	strh	r2, [r3, #6]	@ movhi
	mov	r2, #16
	bl	memset
	ldmfd	sp, {fp, sp, pc}
.L52:
	.align	2
.L51:
	.word	.LANCHOR0
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlBbtCalcTotleCnt
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L60
	movw	r2, #298
	mov	r5, #0
	mov	r4, r5
	ldrh	r2, [r3, r2]
	ldrh	r6, [r3, #254]
	mul	r6, r6, r2
.L54:
	uxth	r0, r5
	cmp	r0, r6
	blt	.L56
	mov	r0, r4
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L56:
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	add	r5, r5, #1
	addne	r4, r4, #1
	uxthne	r4, r4
	b	.L54
.L61:
	.align	2
.L60:
	.word	.LANCHOR0
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	2
	.global	V2P_block
	.syntax unified
	.arm
	.fpu softvfp
	.type	V2P_block, %function
V2P_block:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L63
	uxth	r7, r0
	uxth	r5, r1
	add	r3, r4, #256
	mov	r0, r7
	ldrh	r6, [r3]
	mov	r1, r6
	bl	__udivsi3
	movw	r3, #298
	smulbb	r5, r6, r5
	ldrh	r4, [r4, r3]
	mov	r1, r6
	smulbb	r4, r4, r0
	mov	r0, r7
	bl	__umodsi3
	add	r0, r5, r0
	add	r0, r4, r0
	uxth	r0, r0
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L64:
	.align	2
.L63:
	.word	.LANCHOR0
	.size	V2P_block, .-V2P_block
	.align	2
	.global	P2V_plane
	.syntax unified
	.arm
	.fpu softvfp
	.type	P2V_plane, %function
P2V_plane:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L66
	uxth	r5, r0
	add	r2, r3, #256
	mov	r0, r5
	ldrh	r6, [r2]
	movw	r2, #298
	ldrh	r1, [r3, r2]
	bl	__udivsi3
	mov	r1, r6
	smulbb	r4, r0, r6
	mov	r0, r5
	bl	__umodsi3
	add	r0, r4, r0
	uxth	r0, r0
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L67:
	.align	2
.L66:
	.word	.LANCHOR0
	.size	P2V_plane, .-P2V_plane
	.align	2
	.global	P2V_block_in_plane
	.syntax unified
	.arm
	.fpu softvfp
	.type	P2V_block_in_plane, %function
P2V_block_in_plane:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L69
	movw	r3, #298
	uxth	r0, r0
	ldrh	r1, [r4, r3]
	add	r4, r4, #256
	bl	__umodsi3
	ldrh	r1, [r4]
	uxth	r0, r0
	bl	__udivsi3
	uxth	r0, r0
	ldmfd	sp, {r4, fp, sp, pc}
.L70:
	.align	2
.L69:
	.word	.LANCHOR0
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	2
	.global	ftl_cmp_data_ver
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	cmp	r0, r1
	bls	.L72
	sub	r0, r0, r1
	cmp	r0, #-2147483648
	movhi	r0, #0
	movls	r0, #1
	ldmfd	sp, {fp, sp, pc}
.L72:
	sub	r0, r1, r0
	cmp	r0, #-2147483648
	movls	r0, #0
	movhi	r0, #1
	ldmfd	sp, {fp, sp, pc}
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r2, .L75
	mov	r4, #0
	mov	r1, #2048
	add	r3, r2, #412
	strh	r0, [r3]	@ movhi
	add	r0, r2, #420
	strh	r4, [r3, #2]	@ movhi
	strh	r4, [r3, #4]	@ movhi
	strh	r4, [r3, #6]	@ movhi
	bl	__memzero
	mov	r0, r4
	ldmfd	sp, {r4, fp, sp, pc}
.L76:
	.align	2
.L75:
	.word	.LANCHOR0
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	2
	.global	FtlFreeSysBlkQueueEmpty
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L78
	ldrh	r0, [r3, #6]
	clz	r0, r0
	lsr	r0, r0, #5
	ldmfd	sp, {fp, sp, pc}
.L79:
	.align	2
.L78:
	.word	.LANCHOR0+412
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	2
	.global	FtlFreeSysBlkQueueFull
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L81
	ldrh	r0, [r3, #6]
	sub	r0, r0, #1024
	clz	r0, r0
	lsr	r0, r0, #5
	ldmfd	sp, {fp, sp, pc}
.L82:
	.align	2
.L81:
	.word	.LANCHOR0+412
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBLkSort
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L92
	ldrh	r2, [r3, #6]
	cmp	r2, #0
	ldmfdeq	sp, {r4, fp, sp, pc}
	ldr	r2, .L92+4
	mov	r4, #0
	ldrh	r1, [r3, #2]
	mov	lr, r4
	ldrh	ip, [r2, #28]
	ldrh	r2, [r3, #4]
	and	ip, ip, #31
.L85:
	uxth	r0, r4
	add	r4, r4, #1
	cmp	ip, r0
	bgt	.L86
	cmp	lr, #0
	strhne	r1, [r3, #2]	@ movhi
	strhne	r2, [r3, #4]	@ movhi
	ldmfd	sp, {r4, fp, sp, pc}
.L86:
	add	r0, r3, r1, lsl #1
	add	r1, r1, #1
	ubfx	r1, r1, #0, #10
	ldrh	lr, [r0, #8]
	add	r0, r3, r2, lsl #1
	strh	lr, [r0, #8]	@ movhi
	mov	lr, #1
	add	r2, r2, lr
	ubfx	r2, r2, #0, #10
	b	.L85
.L93:
	.align	2
.L92:
	.word	.LANCHOR0+412
	.word	.LANCHOR0+2468
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	IsInFreeQueue
	.syntax unified
	.arm
	.fpu softvfp
	.type	IsInFreeQueue, %function
IsInFreeQueue:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L100
	uxth	r0, r0
	ldrh	ip, [r3, #6]
	cmp	ip, #1024
	ldrhne	lr, [r3, #2]
	movne	r1, #0
	bne	.L96
.L98:
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L97:
	add	r2, r1, lr
	ubfx	r2, r2, #0, #10
	add	r2, r3, r2, lsl #1
	ldrh	r2, [r2, #8]
	cmp	r2, r0
	beq	.L99
	add	r1, r1, #1
.L96:
	cmp	r1, ip
	bcc	.L97
	b	.L98
.L99:
	mov	r0, #1
	ldmfd	sp, {fp, sp, pc}
.L101:
	.align	2
.L100:
	.word	.LANCHOR0+412
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	2
	.global	insert_data_list
	.syntax unified
	.arm
	.fpu softvfp
	.type	insert_data_list, %function
insert_data_list:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	uxth	r0, r0
	ldr	r1, .L117
	ldrh	r3, [r1, #240]
	mov	lr, r1
	cmp	r3, r0
	bls	.L104
	ldr	r6, [r1, #2516]
	lsl	r5, r0, #3
	mvn	ip, #0
	add	r2, r6, r5
	strh	ip, [r2, #2]	@ movhi
	strh	ip, [r6, r5]	@ movhi
	ldr	r3, [r1, #2520]
	cmp	r3, #0
	bne	.L105
.L116:
	str	r2, [r1, #2520]
	b	.L104
.L105:
	ldr	r8, [r1, #72]
	lsl	r1, r0, #1
	ldr	r7, [lr, #2516]
	movw	r9, #65535
	ldrh	lr, [lr, #240]
	ldrh	r4, [r8, r1]
	ldrh	r1, [r2, #4]
	str	lr, [fp, #-44]
	mov	lr, #0
	cmp	r1, #0
	mulne	ip, r1, r4
	sub	r1, r3, r7
	ubfx	r1, r1, #3, #16
.L110:
	ldr	r4, [fp, #-44]
	add	lr, lr, #1
	uxth	lr, lr
	cmp	lr, r4
	movls	r4, #0
	movhi	r4, #1
	cmp	r0, r1
	orreq	r4, r4, #1
	cmp	r4, #0
	bne	.L104
	ldrh	r4, [r3, #4]
	cmp	r4, #0
	beq	.L108
	lsl	r10, r1, #1
	ldrh	r10, [r8, r10]
	mul	r4, r4, r10
	cmp	r4, ip
	bcs	.L108
	ldrh	r4, [r3]
	cmp	r4, r9
	bne	.L109
	strh	r1, [r2, #2]	@ movhi
	strh	r0, [r3]	@ movhi
	ldr	r3, .L117
	str	r2, [r3, #2524]
.L104:
	mov	r0, #0
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L109:
	add	r3, r7, r4, lsl #3
	mov	r1, r4
	b	.L110
.L108:
	strh	r1, [r6, r5]	@ movhi
	ldrh	r1, [r3, #2]
	strh	r1, [r2, #2]	@ movhi
	ldr	r1, .L117
	ldr	ip, [r1, #2520]
	cmp	r3, ip
	strheq	r0, [r3, #2]	@ movhi
	beq	.L116
.L111:
	ldrh	r2, [r3, #2]
	ldr	r1, [r1, #2516]
	lsl	r2, r2, #3
	strh	r0, [r1, r2]	@ movhi
	strh	r0, [r3, #2]	@ movhi
	b	.L104
.L118:
	.align	2
.L117:
	.word	.LANCHOR0
	.size	insert_data_list, .-insert_data_list
	.align	2
	.global	INSERT_DATA_LIST
	.syntax unified
	.arm
	.fpu softvfp
	.type	INSERT_DATA_LIST, %function
INSERT_DATA_LIST:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	uxth	r0, r0
	bl	insert_data_list
	ldr	r2, .L121
	add	r1, r2, #2528
	ldrh	r2, [r2, #240]
	ldrh	r3, [r1]
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r2, r3
	strh	r3, [r1]	@ movhi
	ldmfdcs	sp, {fp, sp, pc}
	mov	r2, #214
	ldr	r1, .L121+4
	ldr	r0, .L121+8
	bl	sftl_printk
	ldmfd	sp, {fp, sp, pc}
.L122:
	.align	2
.L121:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC8
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	2
	.global	insert_free_list
	.syntax unified
	.arm
	.fpu softvfp
	.type	insert_free_list, %function
insert_free_list:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, fp, ip, lr, pc}
	sub	fp, ip, #4
	uxth	r0, r0
	movw	lr, #65535
	cmp	r0, lr
	beq	.L124
	ldr	r2, .L130
	lsl	r5, r0, #3
	mvn	r3, #0
	ldr	r6, [r2, #2516]
	mov	r1, r2
	add	ip, r6, r5
	strh	r3, [ip, #2]	@ movhi
	strh	r3, [r6, r5]	@ movhi
	ldr	r3, [r2, #2532]
	cmp	r3, #0
	streq	ip, [r2, #2532]
	beq	.L124
	ldr	r8, [r2, #2536]
	lsl	r2, r0, #1
	ldr	r7, [r1, #2516]
	mov	r4, lr
	ldrh	r9, [r8, r2]
	sub	r2, r3, r7
	ubfx	r2, r2, #3, #16
.L128:
	lsl	lr, r2, #1
	ldrh	lr, [r8, lr]
	cmp	lr, r9
	bcs	.L126
	ldrh	lr, [r3]
	cmp	lr, r4
	bne	.L127
	strh	r2, [ip, #2]	@ movhi
	strh	r0, [r3]	@ movhi
.L124:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, fp, sp, pc}
.L127:
	add	r3, r7, lr, lsl #3
	mov	r2, lr
	b	.L128
.L126:
	ldrh	lr, [r3, #2]
	strh	lr, [ip, #2]	@ movhi
	strh	r2, [r6, r5]	@ movhi
	ldr	r2, [r1, #2532]
	cmp	r3, r2
	ldrhne	r2, [r3, #2]
	ldrne	r1, [r1, #2516]
	strheq	r0, [r3, #2]	@ movhi
	streq	ip, [r1, #2532]
	lslne	r2, r2, #3
	strhne	r0, [r1, r2]	@ movhi
	strhne	r0, [r3, #2]	@ movhi
	b	.L124
.L131:
	.align	2
.L130:
	.word	.LANCHOR0
	.size	insert_free_list, .-insert_free_list
	.align	2
	.global	INSERT_FREE_LIST
	.syntax unified
	.arm
	.fpu softvfp
	.type	INSERT_FREE_LIST, %function
INSERT_FREE_LIST:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	uxth	r0, r0
	bl	insert_free_list
	ldr	r2, .L134
	ldrh	r3, [r2, #224]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r2, #224]	@ movhi
	ldrh	r2, [r2, #240]
	cmp	r2, r3
	ldmfdcs	sp, {fp, sp, pc}
	mov	r2, #207
	ldr	r1, .L134+4
	ldr	r0, .L134+8
	bl	sftl_printk
	ldmfd	sp, {fp, sp, pc}
.L135:
	.align	2
.L134:
	.word	.LANCHOR0
	.word	.LANCHOR1+17
	.word	.LC8
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.syntax unified
	.arm
	.fpu softvfp
	.type	List_remove_node, %function
List_remove_node:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L143
	uxth	r1, r1
	movw	r3, #65535
	mov	r8, r0
	ldr	r7, [r4, #2516]
	lsl	r6, r1, #3
	add	r5, r7, r6
	ldrh	r2, [r5, #2]
	cmp	r2, r3
	bne	.L137
	ldr	r3, [r0]
	cmp	r5, r3
	beq	.L137
	mov	r2, #372
	ldr	r1, .L143+4
	ldr	r0, .L143+8
	bl	sftl_printk
.L137:
	ldr	r3, [r8]
	ldrh	r2, [r7, r6]
	cmp	r5, r3
	movw	r3, #65535
	bne	.L138
	cmp	r2, r3
	ldrne	r3, [r4, #2516]
	moveq	r3, #0
	streq	r3, [r8]
	addne	r2, r3, r2, lsl #3
	mvnne	r3, #0
	strne	r2, [r8]
	strhne	r3, [r2, #2]	@ movhi
.L140:
	mvn	r3, #0
	mov	r0, #0
	strh	r3, [r7, r6]	@ movhi
	strh	r3, [r5, #2]	@ movhi
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L138:
	cmp	r2, r3
	ldrh	r3, [r5, #2]
	bne	.L141
	cmp	r3, r2
	ldrne	r2, [r4, #2516]
	lslne	r3, r3, #3
	mvnne	r1, #0
	beq	.L140
.L142:
	strh	r1, [r2, r3]	@ movhi
	b	.L140
.L141:
	ldr	r1, [r4, #2516]
	add	r2, r1, r2, lsl #3
	strh	r3, [r2, #2]	@ movhi
	ldrh	r3, [r5, #2]
	ldrh	r1, [r7, r6]
	ldr	r2, [r4, #2516]
	lsl	r3, r3, #3
	b	.L142
.L144:
	.align	2
.L143:
	.word	.LANCHOR0
	.word	.LANCHOR1+34
	.word	.LC8
	.size	List_remove_node, .-List_remove_node
	.align	2
	.global	List_pop_index_node
	.syntax unified
	.arm
	.fpu softvfp
	.type	List_pop_index_node, %function
List_pop_index_node:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, [r0]
	uxth	r1, r1
	cmp	r3, #0
	movweq	r4, #65535
	beq	.L145
	ldr	r2, .L152
	movw	ip, #65535
	ldr	r4, [r2, #2516]
.L147:
	cmp	r1, #0
	bne	.L148
.L150:
	sub	r3, r3, r4
	ubfx	r4, r3, #3, #16
	mov	r1, r4
	bl	List_remove_node
.L145:
	mov	r0, r4
	ldmfd	sp, {r4, fp, sp, pc}
.L148:
	ldrh	r2, [r3]
	cmp	r2, ip
	beq	.L150
	sub	r1, r1, #1
	add	r3, r4, r2, lsl #3
	uxth	r1, r1
	b	.L147
.L153:
	.align	2
.L152:
	.word	.LANCHOR0
	.size	List_pop_index_node, .-List_pop_index_node
	.align	2
	.global	List_pop_head_node
	.syntax unified
	.arm
	.fpu softvfp
	.type	List_pop_head_node, %function
List_pop_head_node:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r1, #0
	bl	List_pop_index_node
	ldmfd	sp, {fp, sp, pc}
	.size	List_pop_head_node, .-List_pop_head_node
	.align	2
	.global	List_get_gc_head_node
	.syntax unified
	.arm
	.fpu softvfp
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r1, .L161
	uxth	r2, r0
	ldr	r3, [r1, #2520]
	cmp	r3, #0
	ldrne	r1, [r1, #2516]
	movwne	r0, #65535
	bne	.L157
	movw	r0, #65535
	ldmfd	sp, {fp, sp, pc}
.L159:
	sub	r2, r2, #1
	add	r3, r1, r3, lsl #3
	uxth	r2, r2
.L157:
	cmp	r2, #0
	beq	.L158
	ldrh	r3, [r3]
	cmp	r3, r0
	bne	.L159
	ldmfd	sp, {fp, sp, pc}
.L158:
	sub	r3, r3, r1
	ubfx	r0, r3, #3, #16
	ldmfd	sp, {fp, sp, pc}
.L162:
	.align	2
.L161:
	.word	.LANCHOR0
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	2
	.global	List_update_data_list
	.syntax unified
	.arm
	.fpu softvfp
	.type	List_update_data_list, %function
List_update_data_list:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L176
	uxth	r5, r0
	ldrh	r3, [r4, #24]
	cmp	r3, r5
	beq	.L165
	ldrh	r3, [r4, #76]
	cmp	r3, r5
	beq	.L165
	ldrh	r3, [r4, #124]
	cmp	r3, r5
	beq	.L165
	ldr	r8, [r4, #2516]
	ldr	r3, [r4, #2520]
	add	r7, r8, r5, lsl #3
	cmp	r7, r3
	beq	.L165
	ldr	r2, [r4, #72]
	lsl	r3, r5, #1
	ldrh	r6, [r7, #4]
	ldrh	r3, [r2, r3]
	cmp	r6, #0
	mvneq	r6, #0
	mulne	r6, r6, r3
	ldr	r3, [r8, r5, lsl #3]
	cmn	r3, #1
	bne	.L168
	movw	r2, #463
	ldr	r1, .L176+4
	ldr	r0, .L176+8
	bl	sftl_printk
.L168:
	ldr	r3, [r8, r5, lsl #3]
	cmn	r3, #1
	beq	.L165
	ldrh	r2, [r7, #2]
	ldr	r1, [r4, #72]
	lsl	r2, r2, #3
	lsr	r3, r2, #2
	ldrh	r0, [r1, r3]
	ldr	r1, [r4, #2516]
	add	r2, r1, r2
	ldrh	r3, [r2, #4]
	cmp	r3, #0
	mulne	r3, r3, r0
	mvneq	r3, #0
	cmp	r6, r3
	bcs	.L165
	ldr	r4, .L176+12
	mov	r1, r5
	ldr	r0, .L176+16
	bl	List_remove_node
	ldrh	r3, [r4]
	cmp	r3, #0
	bne	.L170
	movw	r2, #474
	ldr	r1, .L176+4
	ldr	r0, .L176+8
	bl	sftl_printk
.L170:
	ldrh	r3, [r4]
	mov	r0, r5
	sub	r3, r3, #1
	strh	r3, [r4]	@ movhi
	bl	INSERT_DATA_LIST
.L165:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L177:
	.align	2
.L176:
	.word	.LANCHOR0
	.word	.LANCHOR1+51
	.word	.LC8
	.word	.LANCHOR0+2528
	.word	.LANCHOR0+2520
	.size	List_update_data_list, .-List_update_data_list
	.align	2
	.global	select_l2p_ram_region
	.syntax unified
	.arm
	.fpu softvfp
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L188
	movw	r2, #338
	mov	r1, #0
	mov	r0, #12
	movw	ip, #65535
	ldrh	r2, [r3, r2]
	ldr	r3, [r3, #2540]
.L179:
	uxth	r4, r1
	cmp	r4, r2
	bcc	.L181
	mov	r4, r2
	mov	r1, #0
	mov	r0, #-2147483648
	mov	r5, #12
.L182:
	uxth	lr, r1
	cmp	lr, r2
	bcc	.L184
	cmp	r4, r2
	bcc	.L180
	ldr	r1, .L188+4
	mov	r4, r2
	mvn	r0, #0
	ldrh	r5, [r1]
	mov	r1, #0
.L185:
	uxth	ip, r1
	cmp	ip, r2
	bcc	.L187
	cmp	r4, r2
	bcc	.L180
	movw	r2, #789
	ldr	r1, .L188+8
	ldr	r0, .L188+12
	bl	sftl_printk
	b	.L180
.L181:
	add	r1, r1, #1
	mla	lr, r0, r1, r3
	ldrh	lr, [lr, #-12]
	cmp	lr, ip
	bne	.L179
.L180:
	mov	r0, r4
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L184:
	mla	ip, r5, r1, r3
	add	r1, r1, #1
	ldr	ip, [ip, #4]
	cmp	r0, ip
	movls	r6, #0
	movhi	r6, #1
	cmp	ip, #0
	movlt	r6, #0
	cmp	r6, #0
	movne	r0, ip
	movne	r4, lr
	b	.L182
.L187:
	ldr	lr, [r3, #4]
	cmp	r0, lr
	bls	.L186
	ldrh	r6, [r3]
	cmp	r6, r5
	movne	r0, lr
	movne	r4, ip
.L186:
	add	r1, r1, #1
	add	r3, r3, #12
	b	.L185
.L189:
	.align	2
.L188:
	.word	.LANCHOR0
	.word	.LANCHOR0+2544
	.word	.LANCHOR1+73
	.word	.LC8
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	ip, .L196
	movw	r1, #2546
	ldrh	r2, [ip, r1]
	mov	r3, ip
	cmp	r2, #4
	cmpls	r0, #0
	bne	.L191
	add	r2, r2, #1
	strh	r2, [ip, r1]	@ movhi
	ldmfd	sp, {fp, sp, pc}
.L191:
	mov	r2, #0
	movw	lr, #65535
	strh	r2, [ip, r1]	@ movhi
	str	r2, [ip, #2548]
	ldrh	r1, [ip, #240]
	ldr	r2, [ip, #72]
	add	r1, r2, r1, lsl #1
.L192:
	cmp	r2, r1
	bne	.L194
	ldmfd	sp, {fp, sp, pc}
.L194:
	ldrh	ip, [r2], #2
	cmp	ip, lr
	ldrne	r0, [r3, #2548]
	addne	r0, r0, ip
	strne	r0, [r3, #2548]
	b	.L192
.L197:
	.align	2
.L196:
	.word	.LANCHOR0
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	ftl_sb_update_avl_pages
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r3, #0
	uxth	r2, r2
	uxth	r5, r1
	strh	r3, [r0, #4]	@ movhi
	ldr	r3, .L205
	add	ip, r0, r2, lsl #1
	movw	r1, #65535
	add	ip, ip, #14
	ldrh	lr, [r3, #232]
.L199:
	cmp	r2, lr
	bcc	.L201
	movw	r2, #302
	add	ip, r0, #16
	ldrh	r1, [r3, r2]
	movw	r4, #65535
	mov	r3, #0
	sub	r1, r1, #1
	sub	r1, r1, r5
	uxth	r1, r1
.L202:
	uxth	r2, r3
	cmp	lr, r2
	bhi	.L204
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L201:
	ldrh	r4, [ip, #2]!
	add	r2, r2, #1
	uxth	r2, r2
	cmp	r4, r1
	ldrhne	r4, [r0, #4]
	addne	r4, r4, #1
	strhne	r4, [r0, #4]	@ movhi
	b	.L199
.L204:
	ldrh	r2, [ip], #2
	add	r3, r3, #1
	cmp	r2, r4
	ldrhne	r2, [r0, #4]
	addne	r2, r1, r2
	strhne	r2, [r0, #4]	@ movhi
	b	.L202
.L206:
	.align	2
.L205:
	.word	.LANCHOR0
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.align	2
	.global	FtlSlcSuperblockCheck
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldrh	r3, [r0, #4]
	cmp	r3, #0
	ldmfdeq	sp, {fp, sp, pc}
	ldrh	r2, [r0]
	movw	r3, #65535
	cmp	r2, r3
	ldmfdeq	sp, {fp, sp, pc}
	ldrb	r2, [r0, #6]	@ zero_extendqisi2
	mov	lr, #0
	add	r2, r0, r2, lsl #1
	ldrh	r1, [r2, #16]
	ldr	r2, .L213
	ldrh	ip, [r2, #232]
	mov	r2, r3
.L210:
	cmp	r1, r2
	beq	.L212
	ldmfd	sp, {fp, sp, pc}
.L212:
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, ip
	strb	r3, [r0, #6]
	ldrheq	r3, [r0, #2]
	strbeq	lr, [r0, #6]
	addeq	r3, r3, #1
	strheq	r3, [r0, #2]	@ movhi
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	add	r3, r0, r3, lsl #1
	ldrh	r1, [r3, #16]
	b	.L210
.L214:
	.align	2
.L213:
	.word	.LANCHOR0
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	2
	.global	make_superblock
	.syntax unified
	.arm
	.fpu softvfp
	.type	make_superblock, %function
make_superblock:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	mov	r4, r0
	ldr	r6, .L221
	ldrh	r2, [r0]
	ldrh	r3, [r6, #240]
	cmp	r2, r3
	bcc	.L216
	movw	r2, #2150
	ldr	r1, .L221+4
	ldr	r0, .L221+8
	bl	sftl_printk
.L216:
	ldrh	r9, [r6, #232]
	add	r8, r4, #16
	ldr	r10, .L221+12
	mvn	r7, #0
	mov	r5, #0
	strh	r5, [r4, #4]	@ movhi
	strb	r5, [r4, #7]
.L217:
	uxth	r3, r5
	ldrh	r1, [r4]
	cmp	r9, r3
	bhi	.L219
	movw	r2, #302
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	ldrh	r2, [r6, r2]
	lsl	r1, r1, #1
	mov	r0, #0
	smulbb	r3, r3, r2
	strh	r3, [r4, #4]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #9]
	ldr	r3, [r6, #2536]
	ldrh	r2, [r3, r1]
	movw	r3, #10000
	cmp	r2, r3
	movhi	r3, #1
	strbhi	r3, [r4, #9]
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L219:
	ldrb	r0, [r10, r5]	@ zero_extendqisi2
	add	r8, r8, #2
	bl	V2P_block
	strh	r7, [r8, #-2]	@ movhi
	add	r5, r5, #1
	str	r0, [fp, #-44]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldreq	r3, [fp, #-44]
	strheq	r3, [r8, #-2]	@ movhi
	ldrbeq	r3, [r4, #7]	@ zero_extendqisi2
	addeq	r3, r3, #1
	strbeq	r3, [r4, #7]
	b	.L217
.L222:
	.align	2
.L221:
	.word	.LANCHOR0
	.word	.LANCHOR1+95
	.word	.LC8
	.word	.LANCHOR0+260
	.size	make_superblock, .-make_superblock
	.align	2
	.global	update_multiplier_value
	.syntax unified
	.arm
	.fpu softvfp
	.type	update_multiplier_value, %function
update_multiplier_value:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L229
	movw	r2, #302
	mov	r5, #0
	uxth	r6, r0
	mov	r4, r5
	ldrh	r8, [r3, #232]
	mov	r7, r3
	ldrh	r9, [r3, r2]
	add	r10, r3, #260
.L224:
	uxth	r3, r5
	cmp	r8, r3
	bhi	.L226
	cmp	r4, #0
	moveq	r0, r4
	beq	.L227
	mov	r1, r4
	mov	r0, #32768
	bl	__divsi3
.L227:
	ldr	r1, [r7, #2516]
	add	r1, r1, r6, lsl #3
	strh	r0, [r1, #4]	@ movhi
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L226:
	mov	r1, r6
	ldrb	r0, [r10, r5]	@ zero_extendqisi2
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	add	r5, r5, #1
	addeq	r4, r4, r9
	uxtheq	r4, r4
	b	.L224
.L230:
	.align	2
.L229:
	.word	.LANCHOR0
	.size	update_multiplier_value, .-update_multiplier_value
	.align	2
	.global	GetFreeBlockMinEraseCount
	.syntax unified
	.arm
	.fpu softvfp
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L234
	ldr	r0, [r3, #2532]
	cmp	r0, #0
	ldrne	r2, [r3, #2516]
	ldrne	r3, [r3, #2536]
	subne	r0, r0, r2
	ubfxne	r0, r0, #3, #16
	lslne	r0, r0, #1
	ldrhne	r0, [r3, r0]
	ldmfd	sp, {fp, sp, pc}
.L235:
	.align	2
.L234:
	.word	.LANCHOR0
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	2
	.global	GetFreeBlockMaxEraseCount
	.syntax unified
	.arm
	.fpu softvfp
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r2, .L244
	uxth	r4, r0
	ldr	r0, [r2, #2532]
	cmp	r0, #0
	ldmfdeq	sp, {r4, r5, fp, sp, pc}
	ldrh	r3, [r2, #224]
	mov	r1, #0
	ldr	ip, [r2, #2516]
	movw	r5, #65535
	rsb	r3, r3, r3, lsl #3
	asr	r3, r3, #3
	cmp	r4, r3
	uxthgt	r4, r3
	sub	r3, r0, ip
	ubfx	r3, r3, #3, #16
.L239:
	uxth	r0, r1
	cmp	r4, r0
	bls	.L241
	lsl	r0, r3, #3
	add	r1, r1, #1
	ldrh	lr, [ip, r0]
	cmp	lr, r5
	bne	.L243
.L241:
	ldr	r2, [r2, #2536]
	lsl	r3, r3, #1
	ldrh	r0, [r2, r3]
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L243:
	mov	r3, lr
	b	.L239
.L245:
	.align	2
.L244:
	.word	.LANCHOR0
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	2
	.global	FtlPrintInfo2buf
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #16
	mov	r6, r0
	ldr	r5, .L258
	add	r4, r6, #12
	ldr	r1, .L258+4
	bl	strcpy
	ldr	r3, [r5, #316]
	mov	r1, #64
	ldr	r2, .L258+8
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r1, .L258+12
	mov	r0, r4
	add	r4, r4, #10
	bl	strcpy
	ldr	r3, [r5, #2552]
	mov	r1, #64
	ldr	r2, .L258+16
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2548]
	ldr	r2, .L258+20
	mov	r1, #64
	mov	r0, r4
	add	r7, r5, #352
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2556]
	ldr	r2, .L258+24
	mov	r1, #64
	mov	r0, r4
	ldr	r8, .L258+28
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2560]
	ldr	r2, .L258+32
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2564]
	ldr	r2, .L258+36
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2568]
	ldr	r2, .L258+40
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2572]
	ldr	r2, .L258+44
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2576]
	ldr	r2, .L258+48
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	ldr	r3, [r5, #2580]
	add	r4, r4, r0
	ldr	r2, .L258+52
	mov	r1, #64
	mov	r0, r4
	lsr	r3, r3, #11
	bl	snprintf
	ldr	r3, [r5, #2584]
	add	r4, r4, r0
	ldr	r2, .L258+56
	mov	r1, #64
	mov	r0, r4
	lsr	r3, r3, #11
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2588]
	ldr	r2, .L258+60
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2592]
	ldr	r2, .L258+64
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r7, #6]
	ldr	r2, .L258+68
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #224]
	ldr	r2, .L258+72
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2596]
	ldr	r2, .L258+76
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2600]
	ldr	r2, .L258+80
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2604]
	ldr	r2, .L258+84
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2608]
	ldr	r2, .L258+88
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2612]
	ldr	r2, .L258+92
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2616]
	ldr	r2, .L258+96
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r8, #30]
	ldr	r2, .L258+100
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r8, #28]
	ldr	r2, .L258+104
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #340]
	ldr	r2, .L258+108
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #332]
	ldr	r2, .L258+112
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #228]
	ldr	r2, .L258+116
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r3, r5, #412
	add	r4, r4, r0
	ldrh	r3, [r3, #6]
	mov	r1, #64
	ldr	r2, .L258+120
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #240]
	ldr	r2, .L258+124
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	movw	r3, #2620
	add	r4, r4, r0
	ldrh	r3, [r5, r3]
	mov	r1, #64
	ldr	r2, .L258+128
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #244]
	ldr	r2, .L258+132
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r3, r5, #2624
	add	r4, r4, r0
	ldrh	r3, [r3]
	mov	r1, #64
	ldr	r2, .L258+136
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r7]
	ldr	r2, .L258+140
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #26]
	ldr	r2, .L258+144
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #30]	@ zero_extendqisi2
	ldr	r2, .L258+148
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #24]
	ldr	r2, .L258+152
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #32]	@ zero_extendqisi2
	ldr	r2, .L258+156
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #28]
	ldr	r2, .L258+160
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	ldrh	r3, [r5, #24]
	add	r4, r4, r0
	ldr	r2, [r5, #72]
	mov	r1, #64
	mov	r0, r4
	lsl	r3, r3, #1
	ldrh	r3, [r2, r3]
	ldr	r2, .L258+164
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #78]
	ldr	r2, .L258+168
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #82]	@ zero_extendqisi2
	ldr	r2, .L258+172
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #76]
	ldr	r2, .L258+176
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #84]	@ zero_extendqisi2
	ldr	r2, .L258+180
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #80]
	ldr	r2, .L258+184
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	ldrh	r3, [r5, #76]
	add	r4, r4, r0
	ldr	r2, [r5, #72]
	mov	r1, #64
	mov	r0, r4
	lsl	r3, r3, #1
	ldrh	r3, [r2, r3]
	ldr	r2, .L258+188
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #126]
	ldr	r2, .L258+192
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #130]	@ zero_extendqisi2
	ldr	r2, .L258+196
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #124]
	ldr	r2, .L258+200
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #132]	@ zero_extendqisi2
	ldr	r2, .L258+204
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #128]
	ldr	r2, .L258+208
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #174]
	ldr	r2, .L258+212
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #178]	@ zero_extendqisi2
	ldr	r2, .L258+216
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #172]
	ldr	r2, .L258+220
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #180]	@ zero_extendqisi2
	ldr	r2, .L258+224
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #176]
	ldr	r2, .L258+228
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	ldr	r3, [r5, #2716]
	add	r4, r4, r0
	ldr	r2, .L258+232
	mov	r1, #64
	mov	r0, r4
	str	r3, [sp, #4]
	ldr	r3, [r5, #2724]
	str	r3, [sp]
	ldr	r3, [r5, #2720]
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2712]
	ldr	r2, .L258+236
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2736]
	ldr	r2, .L258+240
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r3, r5, #3152
	add	r4, r4, r0
	ldrh	r3, [r3]
	mov	r1, #64
	ldr	r2, .L258+244
	mov	r0, r4
	bl	snprintf
	movw	r3, #3154
	add	r4, r4, r0
	ldrh	r3, [r5, r3]
	mov	r1, #64
	ldr	r2, .L258+248
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #3156]
	ldr	r2, .L258+252
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	movw	r3, #3160
	add	r4, r4, r0
	ldrh	r3, [r5, r3]
	mov	r1, #64
	ldr	r2, .L258+256
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r2, .L258+260
	mov	r3, r0
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r0, [r5, #224]
	bl	GetFreeBlockMaxEraseCount
	ldr	r2, .L258+264
	mov	r3, r0
	mov	r1, #64
	mov	r0, r4
	bl	snprintf
	ldr	r3, .L258+268
	add	r4, r4, r0
	ldr	r3, [r3]
	cmp	r3, #1
	beq	.L247
.L252:
	sub	r0, r4, r6
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L247:
	ldrh	r3, [r5, #172]
	movw	r2, #65535
	cmp	r3, r2
	beq	.L249
	ldr	r2, [r5, #72]
	lsl	r3, r3, #1
	mov	r0, r4
	mov	r1, #64
	ldrh	r3, [r2, r3]
	ldr	r2, .L258+272
	bl	snprintf
	add	r4, r4, r0
.L249:
	mov	r0, #0
	ldr	r9, .L258+276
	bl	List_get_gc_head_node
	uxth	r0, r0
	mov	r7, #0
	movw	r10, #65535
.L251:
	cmp	r0, r10
	beq	.L250
	ldr	r3, [r5, #2536]
	lsl	r2, r0, #1
	lsl	r8, r0, #3
	mov	r1, #64
	ldrh	r3, [r3, r2]
	str	r3, [sp, #12]
	ldr	r3, [r5, #2516]
	add	r3, r3, r8
	ldrh	r3, [r3, #4]
	str	r3, [sp, #8]
	ldr	r3, [r5, #72]
	ldrh	r3, [r3, r2]
	mov	r2, r9
	stm	sp, {r0, r3}
	mov	r3, r7
	mov	r0, r4
	add	r7, r7, #1
	bl	snprintf
	ldr	r3, [r5, #2516]
	cmp	r7, #16
	add	r4, r4, r0
	ldrh	r0, [r3, r8]
	bne	.L251
.L250:
	ldr	r3, [r5, #2532]
	mov	r7, #0
	ldr	r2, [r5, #2516]
	movw	r9, #65535
	ldr	r10, .L258+280
	sub	r3, r3, r2
	ubfx	r3, r3, #3, #16
.L253:
	cmp	r3, r9
	beq	.L252
	ldr	r1, [r5, #2536]
	lsl	r2, r3, #1
	lsl	r8, r3, #3
	mov	r0, r4
	ldrh	r2, [r1, r2]
	mov	r1, #64
	str	r2, [sp, #8]
	ldr	r2, [r5, #2516]
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r3, [sp]
	mov	r3, r7
	add	r7, r7, #1
	str	r2, [sp, #4]
	mov	r2, r10
	bl	snprintf
	cmp	r7, #4
	add	r4, r4, r0
	beq	.L252
	ldr	r3, [r5, #2516]
	ldrh	r3, [r3, r8]
	b	.L253
.L259:
	.align	2
.L258:
	.word	.LANCHOR0
	.word	.LC9
	.word	.LC10
	.word	.LC11
	.word	.LC12
	.word	.LC13
	.word	.LC14
	.word	.LANCHOR0+2468
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
	.word	.LANCHOR2
	.word	.LC74
	.word	.LC75
	.word	.LC76
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	2
	.global	rknand_proc_ftlread
	.syntax unified
	.arm
	.fpu softvfp
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	cmp	r0, #2048
	mov	r5, r1
	blt	.L262
	ldr	r3, .L263
	mov	r1, #64
	ldr	r2, .L263+4
	mov	r0, r5
	bl	snprintf
	add	r4, r5, r0
	mov	r0, r4
	bl	FtlPrintInfo2buf
	add	r0, r4, r0
	sub	r0, r0, r5
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L262:
	mov	r0, #0
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L264:
	.align	2
.L263:
	.word	.LC0
	.word	.LC77
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	2
	.global	GetSwlReplaceBlock
	.syntax unified
	.arm
	.fpu softvfp
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #16
	ldr	r4, .L289
	ldr	r2, [r4, #2604]
	ldr	r3, [r4, #2616]
	cmp	r2, r3
	bcs	.L266
	ldr	r2, [r4, #2536]
	mov	r3, #0
	ldrh	r1, [r4, #240]
	str	r3, [r4, #2596]
	sub	r2, r2, #2
.L267:
	cmp	r3, r1
	bcc	.L268
	ldr	r5, [r4, #2596]
	mov	r0, r5
	bl	__udivsi3
	ldr	r3, .L289+4
	str	r0, [r4, #2604]
	ldr	r0, [r4, #2600]
	ldrh	r1, [r3]
	sub	r0, r5, r0
	bl	__udivsi3
	str	r0, [r4, #2596]
.L269:
	ldr	r6, [r4, #2616]
	ldr	r7, [r4, #2604]
	add	r3, r6, #256
	cmp	r3, r7
	bls	.L274
	ldr	r2, [r4, #2612]
	add	r3, r6, #768
	cmp	r3, r2
	bls	.L274
.L276:
	movw	r5, #65535
.L275:
	mov	r0, r5
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L268:
	ldrh	r0, [r2, #2]!
	add	r3, r3, #1
	ldr	ip, [r4, #2596]
	add	r0, r0, ip
	str	r0, [r4, #2596]
	b	.L267
.L266:
	ldr	r3, [r4, #2612]
	cmp	r2, r3
	addhi	r3, r3, #1
	strhi	r3, [r4, #2612]
	movhi	r3, #0
	bls	.L269
.L271:
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	bcs	.L269
	ldr	r0, [r4, #2536]
	lsl	r1, r3, #1
	add	r3, r3, #1
	ldrh	r2, [r0, r1]
	add	r2, r2, #1
	strh	r2, [r0, r1]	@ movhi
	b	.L271
.L274:
	ldrh	r0, [r4, #224]
	add	r0, r0, r0, lsl #1
	asr	r0, r0, #2
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	mov	r9, r0
	cmp	r0, r3
	bcc	.L276
	ldr	r3, [r4, #2520]
	cmp	r3, #0
	beq	.L276
	ldrh	r10, [r4, #240]
	movw	r1, #65535
	ldr	lr, [r4, #2516]
	mov	r0, #0
	ldr	r8, [r4, #2536]
	mov	r2, r1
.L277:
	ldrh	ip, [r3]
	movw	r5, #65535
	cmp	ip, r5
	bne	.L280
	mov	r5, r2
.L279:
	movw	r3, #65535
	cmp	r5, r3
	beq	.L276
	lsl	r3, r5, #1
	ldrh	r10, [r8, r3]
	cmp	r6, r10
	bcs	.L281
	str	r3, [fp, #-44]
	bl	GetFreeBlockMinEraseCount
	ldr	r3, [fp, #-44]
	cmp	r6, r0
	strcc	r1, [r4, #2616]
.L281:
	cmp	r7, r10
	bls	.L276
	add	r2, r10, #128
	cmp	r9, r2
	ble	.L276
	add	r2, r10, #256
	cmp	r7, r2
	bhi	.L282
	ldr	r2, [r4, #2612]
	add	r10, r10, #768
	cmp	r10, r2
	bcs	.L276
.L282:
	str	r9, [sp, #8]
	mov	r1, r5
	ldrh	r2, [r8, r3]
	ldr	r0, .L289+8
	str	r2, [sp, #4]
	ldr	r2, [r4, #72]
	ldrh	r3, [r2, r3]
	mov	r2, r7
	str	r3, [sp]
	ldr	r3, [r4, #2612]
	bl	sftl_printk
	mov	r3, #1
	str	r3, [r4, #3164]
	b	.L275
.L280:
	add	r0, r0, #1
	uxth	r0, r0
	cmp	r0, r10
	bhi	.L276
	ldrh	r5, [r3, #4]
	cmp	r5, #0
	beq	.L278
	sub	r3, r3, lr
	asr	r3, r3, #3
	uxth	r5, r3
	lsl	r3, r5, #1
	ldrh	r3, [r8, r3]
	cmp	r6, r3
	bcs	.L279
	cmp	r1, r3
	movhi	r1, r3
	movhi	r2, r5
.L278:
	add	r3, lr, ip, lsl #3
	b	.L277
.L290:
	.align	2
.L289:
	.word	.LANCHOR0
	.word	.LANCHOR0+292
	.word	.LC78
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.align	2
	.global	free_data_superblock
	.syntax unified
	.arm
	.fpu softvfp
	.type	free_data_superblock, %function
free_data_superblock:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	uxth	r0, r0
	movw	r3, #65535
	cmp	r0, r3
	beq	.L292
	ldr	r3, .L293
	mov	r1, #0
	ldr	r2, [r3, #72]
	lsl	r3, r0, #1
	strh	r1, [r2, r3]	@ movhi
	bl	INSERT_FREE_LIST
.L292:
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L294:
	.align	2
.L293:
	.word	.LANCHOR0
	.size	free_data_superblock, .-free_data_superblock
	.align	2
	.global	get_new_active_ppa
	.syntax unified
	.arm
	.fpu softvfp
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldrh	r2, [r0]
	movw	r3, #65535
	mov	r4, r0
	cmp	r2, r3
	bne	.L296
	movw	r2, #2785
	ldr	r1, .L310
	ldr	r0, .L310+4
	bl	sftl_printk
.L296:
	ldr	r6, .L310+8
	movw	r3, #302
	ldrh	r2, [r4, #2]
	ldrh	r3, [r6, r3]
	cmp	r2, r3
	bne	.L297
	movw	r2, #2786
	ldr	r1, .L310
	ldr	r0, .L310+4
	bl	sftl_printk
.L297:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L298
	movw	r2, #2787
	ldr	r1, .L310
	ldr	r0, .L310+4
	bl	sftl_printk
.L298:
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	mov	r3, #0
	ldrh	r0, [r6, #232]
	movw	ip, #65535
	strb	r3, [r4, #10]
	add	r2, r4, r2, lsl #1
	ldrh	r1, [r2, #16]
	mov	r2, r3
.L299:
	cmp	r1, ip
	beq	.L301
	movw	r3, #302
	ldrh	r5, [r4, #2]
	ldrh	ip, [r6, r3]
	cmp	r5, ip
	movwcs	r5, #65535
	bcs	.L295
	ldrh	r2, [r4, #4]
	orr	r5, r5, r1, lsl #10
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	lr, #65535
	sub	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r4, #4]	@ movhi
.L304:
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r0, r3
	ldrheq	r1, [r4, #2]
	moveq	r3, #0
	addeq	r1, r1, #1
	strheq	r1, [r4, #2]	@ movhi
	add	r1, r4, r3, lsl #1
	ldrh	r1, [r1, #16]
	cmp	r1, lr
	beq	.L304
	strb	r3, [r4, #6]
	cmp	r2, #0
	ldrh	r3, [r4, #2]
	sub	r3, r3, ip
	clz	r3, r3
	lsr	r3, r3, #5
	moveq	r3, #0
	cmp	r3, #0
	beq	.L295
	movw	r2, #2810
	ldr	r1, .L310
	ldr	r0, .L310+4
	bl	sftl_printk
.L295:
	mov	r0, r5
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L301:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, r0
	strb	r3, [r4, #6]
	ldrheq	r3, [r4, #2]
	strbeq	r2, [r4, #6]
	addeq	r3, r3, #1
	strheq	r3, [r4, #2]	@ movhi
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	add	r3, r4, r3, lsl #1
	ldrh	r1, [r3, #16]
	b	.L299
.L311:
	.align	2
.L310:
	.word	.LANCHOR1+111
	.word	.LC8
	.word	.LANCHOR0
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.global	FtlGcBufInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r5, .L317
	mov	lr, #12
	mov	r4, #1
	ldr	r2, .L317+4
	mov	r6, #20
	mov	r3, #0
	str	r3, [r2, #3168]
.L313:
	ldrh	r1, [r2, #232]
	uxth	r0, r3
	add	ip, r3, #1
	cmp	r0, r1
	bcc	.L314
	ldr	lr, .L317
	mov	r0, #12
	mov	ip, #0
.L315:
	ldr	r3, [r2, #3188]
	cmp	r1, r3
	bcc	.L316
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, fp, sp, pc}
.L314:
	uxth	r3, r3
	ldr	r7, [r2, #3172]
	mul	r0, lr, r3
	add	r1, r7, r0
	str	r4, [r1, #8]
	ldrh	r1, [r5]
	mul	r1, r3, r1
	add	r8, r1, #3
	cmp	r1, #0
	movlt	r1, r8
	ldr	r8, [r2, #3176]
	bic	r1, r1, #3
	add	r1, r8, r1
	str	r1, [r7, r0]
	ldr	r1, .L317+8
	ldr	r8, [r2, #3172]
	ldrh	r1, [r1]
	add	r7, r8, r0
	mul	r1, r3, r1
	add	r9, r1, #3
	cmp	r1, #0
	movlt	r1, r9
	ldr	r9, [r2, #3180]
	bic	r1, r1, #3
	add	r1, r9, r1
	str	r1, [r7, #4]
	ldr	r1, [r2, #3184]
	mla	r3, r6, r3, r1
	ldr	r1, [r8, r0]
	str	r1, [r3, #8]
	ldr	r1, [r7, #4]
	str	r1, [r3, #12]
	mov	r3, ip
	b	.L313
.L316:
	mul	r4, r0, r1
	ldr	r6, [r2, #3172]
	add	r3, r6, r4
	str	ip, [r3, #8]
	ldrh	r3, [lr]
	mul	r3, r1, r3
	add	r5, r3, #3
	cmp	r3, #0
	movlt	r3, r5
	ldr	r5, [r2, #3176]
	bic	r3, r3, #3
	add	r3, r5, r3
	str	r3, [r6, r4]
	ldr	r3, [r2, #3172]
	add	r4, r3, r4
	ldr	r3, .L317+8
	ldrh	r3, [r3]
	mul	r3, r1, r3
	add	r1, r1, #1
	uxth	r1, r1
	add	r5, r3, #3
	cmp	r3, #0
	movlt	r3, r5
	ldr	r5, [r2, #3180]
	bic	r3, r3, #3
	add	r3, r5, r3
	str	r3, [r4, #4]
	b	.L315
.L318:
	.align	2
.L317:
	.word	.LANCHOR0+310
	.word	.LANCHOR0
	.word	.LANCHOR0+312
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlGcBufFree
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L326
	mov	lr, #0
	mov	r5, #20
	mov	r7, #12
	mov	r8, lr
	ldr	r6, [r3, #3188]
	ldr	r4, [r3, #3172]
.L320:
	uxth	r3, lr
	cmp	r1, r3
	ldmfdls	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
	mla	ip, r5, r3, r0
	mov	r2, #0
.L321:
	uxth	r3, r2
	cmp	r6, r3
	bls	.L322
	mul	r3, r7, r3
	add	r2, r2, #1
	ldr	r10, [r4, r3]
	add	r9, r4, r3
	ldr	r3, [ip, #8]
	cmp	r10, r3
	bne	.L321
	str	r8, [r9, #8]
.L322:
	add	lr, lr, #1
	b	.L320
.L327:
	.align	2
.L326:
	.word	.LANCHOR0
	.size	FtlGcBufFree, .-FtlGcBufFree
	.align	2
	.global	FtlGcBufAlloc
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcBufAlloc, %function
FtlGcBufAlloc:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L335
	mov	ip, #0
	mov	r6, #12
	mov	r7, #1
	mov	r8, #20
	ldr	r4, [r3, #3188]
	ldr	r5, [r3, #3172]
.L329:
	uxth	r2, ip
	cmp	r1, r2
	bhi	.L333
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, fp, sp, pc}
.L333:
	mov	lr, #0
.L330:
	uxth	r3, lr
	cmp	r4, r3
	bls	.L331
	mla	r3, r6, r3, r5
	add	lr, lr, #1
	ldr	r9, [r3, #8]
	cmp	r9, #0
	bne	.L330
	mla	r2, r8, r2, r0
	ldr	lr, [r3]
	str	r7, [r3, #8]
	str	lr, [r2, #8]
	ldr	r3, [r3, #4]
	str	r3, [r2, #12]
.L331:
	add	ip, ip, #1
	b	.L329
.L336:
	.align	2
.L335:
	.word	.LANCHOR0
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	2
	.global	IsBlkInGcList
	.syntax unified
	.arm
	.fpu softvfp
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r2, .L342
	movw	r1, #3196
	uxth	r0, r0
	ldr	r3, [r2, #3192]
	ldrh	r2, [r2, r1]
	add	r2, r3, r2, lsl #1
.L338:
	cmp	r3, r2
	bne	.L340
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L340:
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L338
	mov	r0, #1
	ldmfd	sp, {fp, sp, pc}
.L343:
	.align	2
.L342:
	.word	.LANCHOR0
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	2
	.global	FtlGcUpdatePage
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r4, r0
	ubfx	r0, r0, #10, #16
	mov	r5, r1
	mov	r6, r2
	bl	P2V_block_in_plane
	ldr	r3, .L348
	movw	r2, #3196
	mov	ip, #0
	ldrh	lr, [r3, r2]
	ldr	r2, [r3, #3192]
	sub	r1, r2, #2
.L345:
	uxth	r7, ip
	cmp	r7, lr
	bcc	.L347
	bne	.L346
	lsl	ip, r7, #1
	strh	r0, [r2, ip]	@ movhi
	movw	r2, #3196
	ldrh	r0, [r3, r2]
	add	r0, r0, #1
	strh	r0, [r3, r2]	@ movhi
	b	.L346
.L347:
	ldrh	r7, [r1, #2]!
	add	ip, ip, #1
	cmp	r7, r0
	bne	.L345
.L346:
	movw	ip, #3204
	mov	r0, #12
	ldrh	r2, [r3, ip]
	mul	r0, r0, r2
	ldr	r2, [r3, #3200]
	add	r1, r2, r0
	stmib	r1, {r5, r6}
	str	r4, [r2, r0]
	ldrh	r2, [r3, ip]
	add	r2, r2, #1
	strh	r2, [r3, ip]	@ movhi
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L349:
	.align	2
.L348:
	.word	.LANCHOR0
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshBlock
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, fp, ip, lr, pc}
	sub	fp, ip, #4
	uxth	r4, r0
	ldr	r0, .L353
	mov	r1, r4
	bl	sftl_printk
	ldr	r3, .L353+4
	ldrh	r0, [r3, #222]
	cmp	r4, r0
	beq	.L351
	ldrh	r1, [r3, #220]
	cmp	r4, r1
	beq	.L351
	movw	r2, #65535
	cmp	r0, r2
	strheq	r4, [r3, #222]	@ movhi
	beq	.L351
	cmp	r1, r2
	strheq	r4, [r3, #220]	@ movhi
.L351:
	mov	r0, #0
	ldmfd	sp, {r4, fp, sp, pc}
.L354:
	.align	2
.L353:
	.word	.LC79
	.word	.LANCHOR0
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.align	2
	.global	FtlGcMarkBadPhyBlk
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcMarkBadPhyBlk, %function
FtlGcMarkBadPhyBlk:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L359
	uxth	r5, r0
	movw	r6, #3206
	mov	r0, r5
	bl	P2V_block_in_plane
	mov	r2, r5
	mov	r7, r0
	ldrh	r1, [r4, r6]
	ldr	r0, .L359+4
	bl	sftl_printk
	mov	r0, r7
	bl	FtlGcRefreshBlock
	ldrh	r3, [r4, r6]
	mov	r2, #0
	ldr	r0, .L359+8
.L356:
	uxth	r1, r2
	cmp	r3, r1
	bhi	.L358
	cmp	r3, #15
	movwls	r2, #3206
	addls	r1, r3, #1
	strhls	r1, [r4, r2]	@ movhi
	lslls	r3, r3, #1
	ldrls	r2, .L359+8
	strhls	r5, [r2, r3]	@ movhi
	b	.L357
.L358:
	add	r2, r2, #1
	add	r1, r0, r2, lsl #1
	ldrh	r1, [r1, #-2]
	cmp	r1, r5
	bne	.L356
.L357:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L360:
	.align	2
.L359:
	.word	.LANCHOR0
	.word	.LC80
	.word	.LANCHOR0+3208
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	2
	.global	FtlGcReFreshBadBlk
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L367
	movw	r3, #3206
	ldrh	r2, [r4, r3]
	cmp	r2, #0
	beq	.L362
	ldrh	r1, [r4, #222]
	movw	r3, #65535
	cmp	r1, r3
	bne	.L362
	movw	r3, #3242
	movw	r5, #3242
	ldrh	r1, [r4, r3]
	cmp	r1, r2
	movcs	r2, #0
	strhcs	r2, [r4, r3]	@ movhi
	ldrh	r3, [r4, r5]
	ldr	r2, .L367+4
	lsl	r3, r3, #1
	ldrh	r0, [r2, r3]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r4, r5]
	add	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
.L362:
	mov	r0, #0
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L368:
	.align	2
.L367:
	.word	.LANCHOR0
	.word	.LANCHOR0+3208
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	2
	.global	ftl_malloc
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_malloc, %function
ftl_malloc:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r1, .L370
	bl	__kmalloc
	ldmfd	sp, {fp, sp, pc}
.L371:
	.align	2
.L370:
	.word	37748929
	.size	ftl_malloc, .-ftl_malloc
	.align	2
	.global	ftl_free
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_free, %function
ftl_free:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	bl	kfree
	ldmfd	sp, {fp, sp, pc}
	.size	ftl_free, .-ftl_free
	.align	2
	.global	rknand_print_hex
	.syntax unified
	.arm
	.fpu softvfp
	.type	rknand_print_hex, %function
rknand_print_hex:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r6, #0
	mov	r10, r0
	mov	r7, r1
	mov	r8, r2
	mov	r9, r3
	mov	r5, r6
	mov	r4, r6
.L374:
	cmp	r4, r9
	bcc	.L380
	ldr	r0, .L382
	bl	sftl_printk
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L380:
	cmp	r5, #0
	bne	.L375
	mov	r3, r6
	mov	r2, r7
	mov	r1, r10
	ldr	r0, .L382+4
	bl	sftl_printk
.L375:
	cmp	r8, #4
	ldreq	r1, [r7, r4, lsl #2]
	ldreq	r0, .L382+8
	beq	.L381
	cmp	r8, #2
	lsleq	r3, r4, #1
	ldreq	r0, .L382+12
	ldrbne	r1, [r7, r4]	@ zero_extendqisi2
	ldrheq	r1, [r7, r3]
	ldrne	r0, .L382+16
.L381:
	add	r5, r5, #1
	bl	sftl_printk
	cmp	r5, #15
	bls	.L379
	mov	r5, #0
	ldr	r0, .L382
	bl	sftl_printk
.L379:
	add	r4, r4, #1
	add	r6, r6, r8
	b	.L374
.L383:
	.align	2
.L382:
	.word	.LC85
	.word	.LC81
	.word	.LC82
	.word	.LC83
	.word	.LC84
	.size	rknand_print_hex, .-rknand_print_hex
	.align	2
	.global	FlashEraseBlocks
	.syntax unified
	.arm
	.fpu softvfp
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	@ args = 0, pretend = 0, frame = 12
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #12
	mov	r7, #0
	ldr	r6, .L400
	mov	r8, r2
	mov	r4, r0
	mov	r5, r0
	ldrh	r9, [r6, #12]
	lsl	r10, r9, #3
.L385:
	cmp	r7, r8
	beq	.L389
	sub	r2, fp, #48
	sub	r1, fp, #44
	mov	r0, r5
	bl	l2p_addr_tran
	ldr	ip, [fp, #-48]
	cmp	ip, #0
	bne	.L386
	ldr	r3, [fp, #-44]
	cmp	r10, r3
	bls	.L386
	ldr	r5, .L400+4
	ldr	r6, .L400+8
	b	.L399
.L388:
	mvn	r3, #0
	ldr	r2, [fp, #-44]
	str	r3, [r4, #-20]
	mov	r1, r5
	mov	r0, r6
	str	ip, [fp, #-52]
	bl	sftl_printk
	mov	r3, #16
	mov	r2, #4
	ldr	r1, [r4, #-12]
	ldr	r0, .L400+12
	bl	rknand_print_hex
	mov	r3, #4
	ldr	r1, [r4, #-8]
	mov	r2, r3
	ldr	r0, .L400+16
	bl	rknand_print_hex
	ldr	ip, [fp, #-52]
	add	ip, ip, #1
.L399:
	cmp	ip, r8
	add	r4, r4, #20
	bne	.L388
	bl	dump_stack
.L389:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L386:
	ldr	r3, [r6, #3248]
	uxtb	r0, ip
	ldr	r1, [fp, #-44]
	blx	r3
	cmp	r0, #0
	mvnne	r3, #0
	streq	r0, [r5]
	strne	r3, [r5]
	ldrh	r3, [r6, #14]
	cmp	r3, #4
	bne	.L393
	ldr	r1, [fp, #-44]
	ldr	r3, [r6, #3248]
	ldrb	r0, [fp, #-48]	@ zero_extendqisi2
	add	r1, r9, r1
	blx	r3
	cmp	r0, #0
	mvnne	r3, #0
	strne	r3, [r5]
.L393:
	add	r7, r7, #1
	add	r5, r5, #20
	b	.L385
.L401:
	.align	2
.L400:
	.word	.LANCHOR0
	.word	.LANCHOR1+130
	.word	.LC86
	.word	.LC87
	.word	.LC88
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	uxth	r6, r0
	movw	r2, #65533
	uxth	r1, r1
	sub	r3, r6, #1
	uxth	r3, r3
	cmp	r3, r2
	ldmfdhi	sp, {r4, r5, r6, r7, fp, sp, pc}
	ldr	r5, .L411
	add	r4, r5, #412
	ldrh	r3, [r4, #6]
	cmp	r3, #1024
	ldmfdeq	sp, {r4, r5, r6, r7, fp, sp, pc}
	cmp	r1, #0
	beq	.L404
	mov	r0, r6
	bl	P2V_block_in_plane
	mov	r7, r0
	ldr	r0, [r5, #3268]
	lsl	r3, r6, #10
	mov	r2, #1
	mov	r1, r2
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r2, [r5, #2536]
	lsl	r0, r7, #1
	ldrh	r3, [r2, r0]
	add	r3, r3, #1
	strh	r3, [r2, r0]	@ movhi
	ldr	r3, [r5, #2608]
	add	r3, r3, #1
	str	r3, [r5, #2608]
.L404:
	ldrh	r3, [r4, #6]
	add	r3, r3, #1
	strh	r3, [r4, #6]	@ movhi
	ldrh	r3, [r4, #4]
	add	r2, r4, r3, lsl #1
	add	r3, r3, #1
	ubfx	r3, r3, #0, #10
	strh	r6, [r2, #8]	@ movhi
	strh	r3, [r4, #4]	@ movhi
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L412:
	.align	2
.L411:
	.word	.LANCHOR0
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r6, .L418
	add	r4, r6, #412
	ldrh	r2, [r4, #6]
	cmp	r2, #0
	movweq	r5, #65535
	beq	.L414
	ldrh	r3, [r4, #2]
	sub	r2, r2, #1
	ldr	r0, [r6, #3268]
	strh	r2, [r4, #6]	@ movhi
	mov	r2, #1
	add	r1, r4, r3, lsl #1
	add	r3, r3, #1
	ubfx	r3, r3, #0, #10
	ldrh	r5, [r1, #8]
	mov	r1, r2
	strh	r3, [r4, #2]	@ movhi
	lsl	r3, r5, #10
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r6, #2608]
	add	r3, r3, #1
	str	r3, [r6, #2608]
.L414:
	sub	r3, r5, #1
	movw	r2, #65533
	uxth	r3, r3
	cmp	r3, r2
	bls	.L415
	ldrh	r2, [r4, #6]
	mov	r1, r5
	ldr	r0, .L418+4
	bl	sftl_printk
.L416:
	b	.L416
.L415:
	mov	r0, r5
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L419:
	.align	2
.L418:
	.word	.LANCHOR0
	.word	.LC89
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	ftl_map_blk_alloc_new_blk
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_map_blk_alloc_new_blk, %function
ftl_map_blk_alloc_new_blk:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldrh	r1, [r0, #10]
	mov	r4, r0
	mov	r3, #0
	ldr	r2, [r0, #12]
.L421:
	uxth	r5, r3
	cmp	r5, r1
	bcs	.L424
	mov	r7, r2
	add	r3, r3, #1
	ldrh	r6, [r7]
	add	r2, r2, #2
	cmp	r6, #0
	bne	.L421
	bl	FtlFreeSysBlkQueueOut
	sub	r3, r0, #1
	movw	r2, #65533
	uxth	r3, r3
	mov	r1, r0
	strh	r0, [r7]	@ movhi
	cmp	r3, r2
	bls	.L422
	ldr	r3, .L428
	ldr	r0, .L428+4
	ldrh	r2, [r3, #6]
	bl	sftl_printk
.L423:
	b	.L423
.L422:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	strh	r5, [r4]	@ movhi
	add	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L424:
	ldrh	r3, [r4, #10]
	cmp	r3, r5
	bhi	.L426
	movw	r2, #578
	ldr	r1, .L428+8
	ldr	r0, .L428+12
	bl	sftl_printk
.L426:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L429:
	.align	2
.L428:
	.word	.LANCHOR0+412
	.word	.LC90
	.word	.LANCHOR1+147
	.word	.LC8
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	FlashGetBadBlockList
	.syntax unified
	.arm
	.fpu softvfp
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r5, .L437
	mov	r4, r0
	mov	r6, r1
	mov	r2, #256
	mov	r1, #255
	bl	memset
	ldr	r3, [r5, #3244]
	mov	r1, r6
	mov	r0, r4
	blx	r3
	uxth	r0, r0
	cmp	r0, #50
	bls	.L431
	mov	r2, #256
	mov	r1, #255
	mov	r0, r4
	bl	memset
	mov	r0, #0
.L431:
	ldrh	r3, [r5, #14]
	cmp	r3, #4
	moveq	r3, r4
	addeq	r1, r3, r0, lsl #1
	beq	.L433
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L434:
	ldrh	r2, [r3]
	lsr	r2, r2, #1
	strh	r2, [r3], #2	@ movhi
.L433:
	cmp	r3, r1
	bne	.L434
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L438:
	.align	2
.L437:
	.word	.LANCHOR0
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	2
	.global	ftl_memset
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_memset, %function
ftl_memset:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, fp, ip, lr, pc}
	sub	fp, ip, #4
	cmp	r2, #0
	mov	r4, r0
	beq	.L440
	bl	memset
.L440:
	mov	r0, r4
	ldmfd	sp, {r4, fp, sp, pc}
	.size	ftl_memset, .-ftl_memset
	.align	2
	.global	FtlMemInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlMemInit, %function
FtlMemInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L545
	movw	r3, #65535
	movw	r6, #306
	mov	r2, #32
	mov	r5, #0
	str	r3, [r4, #3276]
	mvn	r3, #0
	ldrh	r0, [r4, r6]
	mov	r7, #12
	strh	r3, [r4, #222]	@ movhi
	add	r9, r4, #312
	strh	r3, [r4, #220]	@ movhi
	add	r3, r4, #3152
	strh	r2, [r3]	@ movhi
	mov	r2, #128
	movw	r3, #3154
	lsl	r0, r0, #1
	strh	r2, [r4, r3]	@ movhi
	movw	r3, #3160
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3206
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3242
	strh	r5, [r4, r3]	@ movhi
	str	r5, [r4, #2588]
	str	r5, [r4, #2592]
	str	r5, [r4, #2576]
	str	r5, [r4, #2564]
	str	r5, [r4, #2560]
	str	r5, [r4, #2568]
	str	r5, [r4, #2572]
	str	r5, [r4, #2556]
	str	r5, [r4, #2596]
	str	r5, [r4, #2600]
	str	r5, [r4, #2608]
	str	r5, [r4, #2612]
	str	r5, [r4, #2616]
	str	r5, [r4, #3272]
	str	r5, [r4, #3164]
	str	r5, [r4, #3280]
	str	r5, [r4, #3156]
	str	r5, [r4, #3284]
	bl	ftl_malloc
	str	r0, [r4, #3192]
	ldrh	r0, [r4, r6]
	mov	r6, #20
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #232]
	str	r0, [r4, #3200]
	mul	r6, r6, r3
	lsl	r8, r6, #2
	mov	r0, r8
	bl	ftl_malloc
	str	r0, [r4, #3288]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3292]
	mov	r0, r8
	bl	ftl_malloc
	str	r0, [r4, #3296]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3268]
	mov	r0, r6
	bl	ftl_malloc
	ldrh	r3, [r4, #232]
	movw	r8, #310
	ldrh	r6, [r4, r8]
	str	r0, [r4, #3184]
	lsl	r3, r3, #1
	mov	r0, r6
	add	r3, r3, #1
	str	r3, [r4, #3188]
	bl	ftl_malloc
	str	r0, [r4, #3300]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3304]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3308]
	ldr	r0, [r4, #3188]
	mul	r0, r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3176]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3312]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3316]
	ldr	r0, [r4, #3188]
	mul	r0, r7, r0
	bl	ftl_malloc
	str	r0, [r4, #3172]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3320]
	mov	r0, r6
	bl	ftl_malloc
	movw	r3, #258
	str	r0, [r4, #3324]
	ldrh	r0, [r4, r3]
	lsl	r0, r0, #2
	bl	ftl_malloc
	ldrh	r3, [r9]
	ldrh	r6, [r4, #232]
	str	r0, [r4, #3328]
	mul	r6, r6, r3
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3332]
	lsl	r0, r6, #2
	movw	r6, #3340
	bl	ftl_malloc
	ldrh	r3, [r9]
	str	r0, [r4, #3336]
	ldr	r0, [r4, #3188]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #3180]
	ldrh	r0, [r4, #242]
	lsl	r0, r0, #1
	uxth	r0, r0
	strh	r0, [r4, r6]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #3344]
	ldrh	r0, [r4, r6]
	ldr	r3, .L545+4
	add	r0, r0, #544
	add	r0, r0, #3
	lsr	r0, r0, #9
	strh	r0, [r4, r6]	@ movhi
	and	r0, r3, r0, lsl #9
	bl	ftl_malloc
	ldrh	r6, [r4, #242]
	str	r0, [r4, #3348]
	add	r0, r0, #32
	str	r0, [r4, #2536]
	lsl	r6, r6, #1
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3352]
	mov	r0, r6
	bl	ftl_malloc
	ldr	r6, [r4, #328]
	str	r0, [r4, #72]
	lsl	r6, r6, #1
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3356]
	mov	r0, r6
	add	r6, r4, #320
	bl	ftl_malloc
	str	r0, [r4, #3360]
	ldrh	r0, [r6]
	lsl	r0, r0, #1
	bl	ftl_malloc
	str	r0, [r4, #348]
	ldrh	r0, [r6]
	lsl	r0, r0, #1
	bl	ftl_malloc
	str	r0, [r4, #3364]
	ldrh	r0, [r6]
	movw	r6, #322
	lsl	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #3368]
	ldrh	r0, [r4, r6]
	lsl	r0, r0, #2
	bl	ftl_malloc
	ldrh	r2, [r4, r6]
	mov	r1, r5
	str	r0, [r4, #3372]
	movw	r5, #338
	lsl	r2, r2, #2
	bl	ftl_memset
	add	r3, r4, #336
	ldrh	r0, [r3]
	lsl	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #3376]
	ldr	r0, [r4, #328]
	lsl	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #3380]
	ldrh	r0, [r4, r5]
	mul	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, r5]
	movw	r5, #3388
	str	r0, [r4, #2540]
	ldrh	r0, [r4, r8]
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r4, #3384]
	ldrh	r0, [r4, #242]
	lsl	r0, r0, #3
	bl	ftl_malloc
	movw	r3, #298
	str	r0, [r4, #2516]
	ldrh	r0, [r4, r3]
	ldrh	r3, [r4, #254]
	add	r0, r0, #31
	asr	r0, r0, #5
	strh	r0, [r4, r5]	@ movhi
	mul	r0, r0, r3
	lsl	r0, r0, #2
	bl	ftl_malloc
	ldrh	r2, [r4, r5]
	add	ip, r4, #380
	ldrh	lr, [r4, #254]
	mov	r3, #1
	str	r0, [r4, #380]
	lsl	r2, r2, #2
	mov	r1, r2
.L445:
	cmp	r3, lr
	bcc	.L446
	ldr	r2, .L545+8
	mov	r1, #0
	add	r3, r2, r3, lsl #2
	add	r2, r2, #56
	add	r3, r3, #24
.L447:
	cmp	r2, r3
	bne	.L448
	ldr	r3, [r4, #3356]
	cmp	r3, #0
	bne	.L449
.L451:
	ldr	r1, .L545+12
	ldr	r0, .L545+16
	bl	sftl_printk
	mvn	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, fp, sp, pc}
.L446:
	ldr	r0, [r4, #380]
	add	r3, r3, #1
	add	r0, r0, r1
	add	r1, r1, r2
	str	r0, [ip, #4]!
	b	.L445
.L448:
	str	r1, [r3, #4]!
	b	.L447
.L449:
	ldr	r3, [r4, #3360]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3376]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3380]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #2540]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3384]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #2516]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #380]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #72]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3192]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3200]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3288]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3296]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3268]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3184]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3292]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3300]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3304]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3308]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3176]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3312]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3316]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3172]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3332]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3336]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3180]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #2536]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #3344]
	cmp	r3, #0
	beq	.L451
	ldr	r3, [r4, #348]
	cmp	r3, #0
	beq	.L451
	ldr	r3, .L545
	ldr	r2, [r3, #3364]
	cmp	r2, #0
	beq	.L451
	ldr	r2, [r3, #3368]
	cmp	r2, #0
	beq	.L451
	ldr	r3, [r3, #3372]
	cmp	r3, #0
	beq	.L451
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, fp, sp, pc}
.L546:
	.align	2
.L545:
	.word	.LANCHOR0
	.word	33553920
	.word	.LANCHOR0+352
	.word	.LANCHOR1+173
	.word	.LC91
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	FtlBbt2Bitmap
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r2, .L553
	movw	r3, #3388
	mov	r4, r0
	ldr	r7, .L553+4
	sub	r5, r4, #2
	ldrh	r2, [r2, r3]
	add	r4, r4, #1020
	ldr	r8, .L553+8
	mov	r6, r1
	add	r4, r4, #2
	mov	r1, #0
	mov	r0, r6
	lsl	r2, r2, #2
	bl	ftl_memset
.L550:
	ldrh	r3, [r5, #2]
	movw	r2, #65535
	cmp	r3, r2
	ldmfdeq	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
	ldrh	r2, [r7]
	cmp	r2, r3
	bhi	.L549
	mov	r2, #74
	mov	r1, r8
	ldr	r0, .L553+12
	bl	sftl_printk
.L549:
	ldrh	r3, [r5, #2]!
	mov	r0, #1
	cmp	r4, r5
	lsr	r1, r3, #5
	and	r3, r3, #31
	ldr	r2, [r6, r1, lsl #2]
	orr	r3, r2, r0, lsl r3
	str	r3, [r6, r1, lsl #2]
	bne	.L550
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L554:
	.align	2
.L553:
	.word	.LANCHOR0
	.word	.LANCHOR0+298
	.word	.LANCHOR1+184
	.word	.LC8
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	2
	.global	ftl_free_no_use_map_blk
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	mov	r1, #0
	ldrh	r2, [r0, #10]
	mov	r4, r0
	ldr	r5, [r0, #20]
	ldr	r7, [r0, #12]
	ldr	r6, [r0, #24]
	lsl	r2, r2, #1
	mov	r0, r5
	bl	ftl_memset
	mov	r2, #0
.L556:
	ldrh	r1, [r4, #6]
	uxth	r3, r2
	cmp	r1, r3
	bhi	.L560
	ldr	r3, .L575
	mov	r6, #0
	mov	r8, r6
	mov	r10, r6
	ldrh	r2, [r3]
	ldrh	r3, [r4]
	lsl	r3, r3, #1
	strh	r2, [r5, r3]	@ movhi
	ldrh	r9, [r5]
.L561:
	ldrh	r3, [r4, #10]
	uxth	r1, r6
	cmp	r3, r1
	bhi	.L565
	mov	r0, r8
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L560:
	uxth	r3, r2
	mov	r1, #0
	ldr	r0, [r6, r3, lsl #2]
	ubfx	r0, r0, #10, #16
.L557:
	ldrh	ip, [r4, #10]
	uxth	r3, r1
	cmp	ip, r3
	addls	r2, r2, #1
	bls	.L556
.L559:
	uxth	r3, r1
	add	r1, r1, #1
	lsl	r3, r3, #1
	ldrh	ip, [r7, r3]
	adds	lr, ip, #0
	movne	lr, #1
	cmp	r0, ip
	movne	lr, #0
	cmp	lr, #0
	ldrhne	ip, [r5, r3]
	addne	ip, ip, #1
	strhne	ip, [r5, r3]	@ movhi
	b	.L557
.L565:
	uxth	r3, r6
	lsl	r3, r3, #1
	ldrh	r2, [r5, r3]
	cmp	r9, r2
	bls	.L562
	ldrh	r0, [r7, r3]
	add	ip, r7, r3
	cmp	r0, #0
	bne	.L563
.L564:
	add	r6, r6, #1
	b	.L561
.L562:
	cmp	r2, #0
	bne	.L564
	ldrh	r0, [r7, r3]
	add	ip, r7, r3
	cmp	r0, #0
	beq	.L564
.L566:
	mov	r1, #1
	str	ip, [fp, #-44]
	bl	FtlFreeSysBlkQueueIn
	ldr	ip, [fp, #-44]
	strh	r10, [ip]	@ movhi
	ldrh	r3, [r4, #8]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L564
.L563:
	subs	r9, r2, #0
	mov	r8, r1
	beq	.L566
	b	.L564
.L576:
	.align	2
.L575:
	.word	.LANCHOR0+304
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	FtlL2PDataInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L580
	mov	r1, #0
	mvn	r5, #0
	ldr	r2, [r4, #328]
	ldr	r0, [r4, #3360]
	lsl	r2, r2, #1
	bl	ftl_memset
	movw	r3, #310
	movw	r2, #338
	ldrh	r3, [r4, r3]
	mov	r1, #255
	ldrh	r2, [r4, r2]
	ldr	r0, [r4, #3384]
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r0, .L580+4
	mov	r2, #0
	mov	r3, r4
	mov	lr, #12
	mov	r4, r2
	sub	r6, r0, #28
.L578:
	ldrh	r7, [r0]
	uxth	r1, r2
	add	ip, r2, #1
	cmp	r7, r1
	bhi	.L579
	ldr	r2, .L580+8
	mvn	r1, #0
	strh	r1, [r2, #2]	@ movhi
	strh	r1, [r2]	@ movhi
	ldr	r1, [r3, #328]
	strh	r1, [r2, #10]	@ movhi
	ldr	r1, .L580+12
	strh	r1, [r2, #4]	@ movhi
	movw	r1, #3436
	ldrh	r1, [r3, r1]
	strh	r1, [r2, #8]	@ movhi
	sub	r1, r2, #3056
	ldrh	r1, [r1]
	strh	r1, [r2, #6]	@ movhi
	ldr	r2, [r3, #3356]
	str	r2, [r3, #3404]
	ldr	r2, [r3, #3380]
	str	r2, [r3, #3408]
	ldr	r2, [r3, #3360]
	str	r2, [r3, #3412]
	ldr	r2, [r3, #3376]
	str	r2, [r3, #3416]
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L579:
	uxth	r2, r2
	ldr	r7, [r3, #2540]
	mul	r1, lr, r2
	add	r8, r7, r1
	str	r4, [r8, #4]
	strh	r5, [r7, r1]	@ movhi
	ldr	r7, [r3, #2540]
	add	r1, r7, r1
	ldrh	r7, [r6]
	mul	r2, r2, r7
	ldr	r7, [r3, #3384]
	bic	r2, r2, #3
	add	r2, r7, r2
	str	r2, [r1, #8]
	mov	r2, ip
	b	.L578
.L581:
	.align	2
.L580:
	.word	.LANCHOR0
	.word	.LANCHOR0+338
	.word	.LANCHOR0+3392
	.word	-3902
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	FtlVariablesInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L583
	mvn	r3, #0
	movw	r2, #3438
	mov	r5, #0
	strh	r3, [r4, r2]	@ movhi
	mov	r1, r5
	str	r3, [r4, #3448]
	add	r3, r4, #344
	strh	r5, [r3]	@ movhi
	add	r3, r4, #320
	ldrh	r2, [r3]
	ldr	r0, [r4, #348]
	str	r5, [r4, #3440]
	str	r5, [r4, #3444]
	lsl	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r4, #242]
	mov	r1, r5
	ldr	r0, [r4, #2536]
	lsl	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r4, #242]
	mov	r1, r5
	ldr	r0, [r4, #3344]
	lsl	r2, r2, #1
	bl	ftl_memset
	add	r0, r4, #2464
	mov	r1, #48
	add	r0, r0, #4
	bl	__memzero
	add	r0, r4, #2640
	mov	r1, #512
	bl	__memzero
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r5
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L584:
	.align	2
.L583:
	.word	.LANCHOR0
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	SupperBlkListInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #16
	mov	r5, #0
	ldr	r4, .L596
	mov	r1, #0
	mov	r8, r5
	mov	r6, r5
	ldrh	r2, [r4, #242]
	ldr	r0, [r4, #2516]
	lsl	r2, r2, #3
	bl	ftl_memset
	add	r3, r4, #2528
	str	r5, [r4, #2532]
	str	r5, [r4, #2520]
	str	r5, [r4, #2524]
	strh	r5, [r3]	@ movhi
	strh	r5, [r4, #224]	@ movhi
	str	r3, [fp, #-48]
.L586:
	ldrh	r3, [r4, #240]
	uxth	r10, r5
	cmp	r10, r3
	bcs	.L593
	ldr	r3, .L596+4
	mov	r9, r10
	ldrh	ip, [r4, #232]
	ldrh	r3, [r3]
	str	r3, [fp, #-44]
	mov	r3, #0
	mov	r7, r3
	b	.L594
.L588:
	ldr	r2, .L596+8
	mov	r1, r9
	str	ip, [fp, #-56]
	str	r3, [fp, #-52]
	ldrb	r0, [r2, r3]	@ zero_extendqisi2
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldr	r3, [fp, #-52]
	ldreq	r2, [fp, #-44]
	ldr	ip, [fp, #-56]
	add	r3, r3, #1
	addeq	r7, r7, r2
	uxtheq	r7, r7
.L594:
	uxth	r1, r3
	cmp	ip, r1
	bhi	.L588
	cmp	r7, #0
	beq	.L589
	mov	r1, r7
	mov	r0, #32768
	bl	__divsi3
	uxth	r7, r0
.L590:
	ldr	r3, [r4, #2516]
	add	r3, r3, r9, lsl #3
	strh	r7, [r3, #4]	@ movhi
	ldrh	r3, [r4, #24]
	cmp	r3, r10
	beq	.L591
	ldrh	r3, [r4, #76]
	cmp	r3, r10
	beq	.L591
	ldrh	r3, [r4, #124]
	cmp	r3, r10
	beq	.L591
	ldr	r2, [r4, #72]
	lsl	r3, r9, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L592
	add	r8, r8, #1
	mov	r0, r9
	uxth	r8, r8
	bl	INSERT_FREE_LIST
.L591:
	add	r5, r5, #1
	b	.L586
.L589:
	ldr	r1, [r4, #72]
	lsl	r3, r9, #1
	mvn	r0, #0
	strh	r0, [r1, r3]	@ movhi
	b	.L590
.L592:
	add	r6, r6, #1
	mov	r0, r9
	uxth	r6, r6
	bl	INSERT_DATA_LIST
	b	.L591
.L593:
	ldr	r2, [fp, #-48]
	strh	r8, [r4, #224]	@ movhi
	strh	r6, [r2]	@ movhi
	add	r6, r6, r8
	cmp	r6, r3
	ble	.L595
	movw	r2, #2212
	ldr	r1, .L596+12
	ldr	r0, .L596+16
	bl	sftl_printk
.L595:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L597:
	.align	2
.L596:
	.word	.LANCHOR0
	.word	.LANCHOR0+302
	.word	.LANCHOR0+260
	.word	.LANCHOR1+198
	.word	.LC8
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	FtlGcPageVarInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L599
	mov	r3, #0
	movw	r2, #3196
	movw	r5, #306
	mov	r1, #255
	strh	r3, [r4, r2]	@ movhi
	movw	r2, #3204
	strh	r3, [r4, r2]	@ movhi
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #3192]
	lsl	r2, r2, #1
	bl	ftl_memset
	ldrh	r3, [r4, r5]
	mov	r2, #12
	mov	r1, #255
	ldr	r0, [r4, #3200]
	mul	r2, r2, r3
	bl	ftl_memset
	bl	FtlGcBufInit
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L600:
	.align	2
.L599:
	.word	.LANCHOR0
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	ftl_memcpy
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_memcpy, %function
ftl_memcpy:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	bl	memcpy
	ldmfd	sp, {fp, sp, pc}
	.size	ftl_memcpy, .-ftl_memcpy
	.align	2
	.global	FlashReadPages
	.syntax unified
	.arm
	.fpu softvfp
	.type	FlashReadPages, %function
FlashReadPages:
	@ args = 0, pretend = 0, frame = 12
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #12
	mov	r7, #0
	ldr	r5, .L641
	mov	r8, r1
	ldr	r9, .L641+4
	mov	r4, r0
	ldr	r10, .L641+8
	ldrh	r3, [r5, #12]
	str	r3, [fp, #-52]
.L603:
	cmp	r7, r8
	bne	.L616
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L616:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L604
	ldr	r3, [r4, #12]
	cmp	r3, #0
	bne	.L605
.L604:
	mov	r2, #96
	mov	r1, r9
	mov	r0, r10
	bl	sftl_printk
.L605:
	sub	r2, fp, #48
	sub	r1, fp, #44
	mov	r0, r4
	bl	l2p_addr_tran
	ldr	r0, [fp, #-48]
	cmp	r0, #3
	mvnhi	r3, #0
	strhi	r3, [r4]
	bhi	.L607
	ldr	r6, [r4, #8]
	ldr	r3, [r4, #12]
	ldr	ip, [r5, #3256]
	tst	r6, #63
	ldr	r1, [fp, #-44]
	ldrne	r6, [r5, #3320]
	mov	r2, r6
	blx	ip
	str	r0, [r4]
	ldrh	r3, [r5, #14]
	cmp	r3, #4
	bne	.L610
	ldr	r0, [fp, #-52]
	add	r2, r6, #2048
	ldr	r3, [r4, #12]
	ldr	r1, [fp, #-44]
	ldr	ip, [r5, #3256]
	add	r3, r3, #8
	add	r1, r0, r1
	ldrb	r0, [fp, #-48]	@ zero_extendqisi2
	blx	ip
	cmn	r0, #1
	beq	.L611
	ldr	r3, [r4, #12]
	ldr	r2, [r3, #12]
	cmn	r2, #1
	bne	.L612
	ldr	r2, [r3, #8]
	cmn	r2, #1
	bne	.L612
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L612
.L611:
	mvn	r3, #0
	str	r3, [r4]
.L612:
	ldr	r3, [r4]
	sub	r0, r0, #256
	clz	r0, r0
	lsr	r0, r0, #5
	cmn	r3, #1
	moveq	r0, #0
	cmp	r0, #0
	movne	r3, #256
	strne	r3, [r4]
	ldr	r3, [r4]
	cmn	r3, #1
	cmpne	r3, #256
	bne	.L610
	ldr	r1, [r4, #4]
	ldr	r2, [fp, #-44]
	ldr	r0, .L641+12
	bl	sftl_printk
	ldr	r1, [r4, #8]
	cmp	r1, #0
	beq	.L615
	mov	r3, #4
	ldr	r0, .L641+16
	mov	r2, r3
	bl	rknand_print_hex
.L615:
	ldr	r1, [r4, #12]
	cmp	r1, #0
	beq	.L610
	mov	r3, #4
	ldr	r0, .L641+20
	mov	r2, r3
	bl	rknand_print_hex
.L610:
	ldr	r3, [r5, #3320]
	cmp	r6, r3
	bne	.L607
	ldr	r0, [r4, #8]
	cmp	r6, r0
	beq	.L607
	ldr	r3, .L641+24
	mov	r1, r6
	ldrh	r2, [r3]
	lsl	r2, r2, #9
	bl	ftl_memcpy
.L607:
	add	r7, r7, #1
	add	r4, r4, #20
	b	.L603
.L642:
	.align	2
.L641:
	.word	.LANCHOR0
	.word	.LANCHOR1+216
	.word	.LC8
	.word	.LC92
	.word	.LC93
	.word	.LC94
	.word	.LANCHOR0+258
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FtlLoadFactoryBbt
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r5, .L653
	mov	r7, #0
	add	r6, r5, #360
	ldr	r3, [r5, #3300]
	ldr	r8, [r5, #3332]
	add	r6, r6, #2
	sub	r9, r6, #64
	str	r3, [r5, #3460]
	str	r8, [r5, #3464]
.L644:
	ldrh	r3, [r5, #254]
	cmp	r7, r3
	bcc	.L649
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L649:
	ldrh	r4, [r9]
	mvn	r3, #0
	ldr	r10, .L653+4
	strh	r3, [r6, #2]!	@ movhi
	add	r4, r4, r3
	uxth	r4, r4
.L645:
	ldrh	r3, [r9]
	sub	r2, r3, #16
	cmp	r4, r2
	ble	.L647
	mla	r3, r7, r3, r4
	mov	r2, #1
	mov	r1, r2
	mov	r0, r10
	lsl	r3, r3, #10
	str	r3, [r5, #3456]
	bl	FlashReadPages
	ldr	r3, [r5, #3452]
	cmn	r3, #1
	beq	.L646
	ldrh	r2, [r8]
	movw	r3, #61664
	cmp	r2, r3
	bne	.L646
	strh	r4, [r6]	@ movhi
.L647:
	add	r7, r7, #1
	b	.L644
.L646:
	sub	r4, r4, #1
	uxth	r4, r4
	b	.L645
.L654:
	.align	2
.L653:
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FtlGetLastWrittenPage
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	@ args = 0, pretend = 0, frame = 84
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #84
	cmp	r1, #1
	ldr	r3, .L666
	movwne	r2, #302
	lsl	r6, r0, #10
	mov	r7, r1
	sub	r0, fp, #116
	addeq	r2, r3, #304
	ldrhne	r5, [r3, r2]
	ldrheq	r5, [r2]
	mov	r2, r1
	ldr	r3, [r3, #3324]
	mov	r1, #1
	sub	r5, r5, #1
	sxth	r5, r5
	str	r3, [fp, #-108]
	sub	r3, fp, #96
	str	r3, [fp, #-104]
	orr	r3, r5, r6
	str	r3, [fp, #-112]
	bl	FlashReadPages
	ldr	r3, [fp, #-96]
	cmn	r3, #1
	moveq	r8, #0
	beq	.L659
.L658:
	mov	r0, r5
	sub	sp, fp, #32
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L662:
	add	r3, r8, r5
	mov	r2, r7
	add	r3, r3, r3, lsr #31
	mov	r1, #1
	sub	r0, fp, #116
	asr	r4, r3, #1
	sxth	r3, r4
	orr	r3, r3, r6
	str	r3, [fp, #-112]
	bl	FlashReadPages
	ldr	r3, [fp, #-96]
	cmn	r3, #1
	bne	.L660
	ldr	r3, [fp, #-92]
	cmn	r3, #1
	bne	.L660
	ldr	r3, [fp, #-116]
	cmn	r3, #1
	subne	r4, r4, #1
	sxthne	r5, r4
	bne	.L659
.L660:
	add	r4, r4, #1
	sxth	r8, r4
.L659:
	cmp	r8, r5
	ble	.L662
	b	.L658
.L667:
	.align	2
.L666:
	.word	.LANCHOR0
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlScanSysBlk
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #24
	mov	r5, #0
	ldr	r4, .L750
	movw	r3, #3436
	mov	r1, r5
	ldr	r2, [r4, #328]
	add	r6, r4, #320
	strh	r5, [r4, r3]	@ movhi
	add	r3, r4, #344
	ldr	r0, [r4, #3380]
	mov	r7, r4
	strh	r5, [r3]	@ movhi
	lsl	r2, r2, #2
	bl	ftl_memset
	ldr	r2, [r4, #328]
	mov	r1, r5
	ldr	r0, [r4, #3356]
	lsl	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r6]
	mov	r1, r5
	ldr	r0, [r4, #3368]
	lsl	r2, r2, #2
	bl	ftl_memset
	ldrh	r2, [r6]
	mov	r1, r5
	ldr	r0, [r4, #348]
	mov	r5, r4
	lsl	r2, r2, #1
	bl	ftl_memset
	mov	r2, #16
	mov	r1, #255
	add	r0, r4, #2624
	bl	memset
	ldrh	r3, [r4, #240]
	str	r3, [fp, #-44]
.L669:
	ldrh	r3, [r5, #242]
	ldr	r2, [fp, #-44]
	cmp	r3, r2
	bls	.L710
	ldr	r8, .L750+4
	mov	r6, #0
	ldrh	ip, [r5, #232]
	mov	r10, r6
	ldr	r9, [r5, #3288]
	ldrh	r1, [r8], #-52
	ldr	r2, [r5, #3176]
	ldr	r3, [r5, #3180]
	str	r1, [fp, #-48]
	b	.L711
.L671:
	ldr	r1, [fp, #-44]
	ldrb	r0, [r8, r6]	@ zero_extendqisi2
	str	r3, [fp, #-64]
	str	r2, [fp, #-60]
	str	ip, [fp, #-56]
	bl	V2P_block
	str	r0, [fp, #-52]
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	ldr	ip, [fp, #-56]
	ldr	r2, [fp, #-60]
	ldr	r3, [fp, #-64]
	bne	.L670
	mov	r1, #20
	mla	r0, r1, r10, r9
	ldr	r1, [fp, #-52]
	lsl	r1, r1, #10
	stmib	r0, {r1, r2}
	ldr	r1, [fp, #-48]
	mul	r1, r1, r10
	add	lr, r1, #3
	cmp	r1, #0
	movlt	r1, lr
	bic	r1, r1, #3
	add	r1, r3, r1
	str	r1, [r0, #12]
	add	r1, r10, #1
	uxth	r10, r1
.L670:
	add	r6, r6, #1
.L711:
	uxth	r1, r6
	cmp	ip, r1
	bhi	.L671
	cmp	r10, #0
	bne	.L672
.L709:
	ldr	r3, [fp, #-44]
	add	r3, r3, #1
	uxth	r3, r3
	str	r3, [fp, #-44]
	b	.L669
.L672:
	mov	r2, #1
	mov	r1, r10
	mov	r0, r9
	bl	FlashReadPages
	mov	r3, #0
.L749:
	str	r3, [fp, #-48]
	ldrh	r3, [fp, #-48]
	cmp	r10, r3
	bls	.L709
	ldr	r3, [fp, #-48]
	mov	r9, #20
	mul	r9, r9, r3
	ldr	r3, [r4, #3288]
	add	r2, r3, r9
	ldr	r3, [r3, r9]
	ldr	r1, [r2, #4]
	ldr	r6, [r2, #12]
	cmn	r3, #1
	lsr	ip, r1, #10
	uxth	r8, ip
	bne	.L676
	mov	r3, #16
.L678:
	ldr	r0, [r4, #3288]
	str	ip, [fp, #-56]
	str	r3, [fp, #-52]
	add	r0, r0, r9
	ldr	r2, [r0, #4]
	add	r2, r2, #1
	str	r2, [r0, #4]
	mov	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldrh	r2, [r6]
	movw	r3, #65535
	ldr	ip, [fp, #-56]
	cmp	r2, r3
	ldr	r3, [fp, #-52]
	bne	.L675
	ldr	r3, [r4, #3288]
	mvn	r2, #0
	str	r2, [r3, r9]
	ldr	r3, [r4, #3288]
	ldr	r3, [r3, r9]
	cmp	r3, r2
	bne	.L676
.L677:
	mov	r1, #1
	b	.L748
.L675:
	ldr	r2, [r4, #3288]
	ldr	r2, [r2, r9]
	cmn	r2, #1
	bne	.L676
	sub	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #0
	bne	.L678
	b	.L677
.L676:
	ldr	r2, [r5, #2588]
	ldr	r3, [r6, #4]
	cmn	r2, #1
	beq	.L679
	cmp	r2, r3
	bhi	.L680
.L679:
	cmn	r3, #1
	addne	r2, r3, #1
	strne	r2, [r7, #2588]
.L680:
	ldrh	r2, [r6]
	movw	r1, #61604
	cmp	r2, r1
	beq	.L682
	bhi	.L683
	movw	r3, #61574
	cmp	r2, r3
	beq	.L684
.L681:
	ldr	r3, [fp, #-48]
	add	r3, r3, #1
	b	.L749
.L683:
	movw	r3, #61634
	cmp	r2, r3
	beq	.L685
	movw	r3, #65535
	cmp	r2, r3
	moveq	r1, #0
	bne	.L681
.L748:
	uxth	r0, ip
	bl	FtlFreeSysBlkQueueIn
	b	.L681
.L685:
	ldr	r9, .L750+8
	ldr	r3, [r4, #328]
	ldrh	r2, [r9]
	cmp	r2, r3
	bls	.L687
	movw	r2, #1225
	ldr	r1, .L750+12
	ldr	r0, .L750+16
	bl	sftl_printk
.L687:
	ldr	r0, [r4, #328]
	ldrh	r2, [r9]
	ldr	ip, [r4, #3380]
	uxth	r1, r0
	sub	r3, r1, #1
	sub	r1, r1, r2
	sub	r1, r1, #1
	sxth	r3, r3
	sxth	r1, r1
	str	r1, [fp, #-52]
.L688:
	ldr	r1, [fp, #-52]
	cmp	r3, r1
	bgt	.L694
	cmp	r3, #0
	bge	.L726
	b	.L681
.L694:
	lsl	lr, r3, #2
	ldr	r1, [r6, #4]
	str	lr, [fp, #-56]
	ldr	lr, [ip, r3, lsl #2]
	cmp	r1, lr
	bls	.L689
	ldr	r1, [ip]
	cmp	r1, #0
	bne	.L690
	cmp	r0, r2
	addne	r2, r2, #1
	strhne	r2, [r9]	@ movhi
.L690:
	uxth	ip, r3
	mov	r1, #0
.L691:
	uxth	r0, r1
	sxth	r2, r1
	cmp	ip, r0
	bhi	.L692
	ldr	r1, [r6, #4]
	cmp	r3, #0
	ldr	r2, [r7, #3380]
	ldr	r0, [fp, #-56]
	str	r1, [r2, r0]
	lsl	r2, r3, #1
	ldr	r1, [r7, #3356]
	strh	r8, [r1, r2]	@ movhi
	blt	.L681
	ldr	r2, .L750+8
	ldr	r1, [r7, #328]
	ldrh	r2, [r2]
	sub	r1, r1, r2
	sub	r1, r1, #1
	sxth	r1, r1
	cmp	r3, r1
	bgt	.L681
.L726:
	add	r2, r2, #1
	ldr	r1, [r6, #4]
	strh	r2, [r9]	@ movhi
	ldr	r2, [r4, #3380]
	str	r1, [r2, r3, lsl #2]
	lsl	r3, r3, #1
	ldr	r2, [r4, #3356]
.L746:
	strh	r8, [r2, r3]	@ movhi
	b	.L681
.L692:
	ldr	r0, [r5, #3380]
	add	r1, r1, #1
	add	lr, r0, r2, lsl #2
	ldr	lr, [lr, #4]
	str	lr, [r0, r2, lsl #2]
	lsl	r2, r2, #1
	ldr	r0, [r5, #3356]
	add	lr, r0, r2
	ldrh	lr, [lr, #2]
	strh	lr, [r0, r2]	@ movhi
	b	.L691
.L689:
	sub	r3, r3, #1
	sxth	r3, r3
	b	.L688
.L684:
	ldr	r9, .L750+20
	ldrh	r2, [r9]
	ldrh	r3, [r9, #-24]
	cmp	r2, r3
	bls	.L697
	movw	r2, #1266
	ldr	r1, .L750+12
	ldr	r0, .L750+16
	bl	sftl_printk
.L697:
	ldrh	r2, [r9, #-24]
	ldrh	r1, [r9]
	ldr	ip, [r4, #3368]
	sub	r0, r2, #1
	sxth	r3, r0
	sub	r0, r0, r1
	str	r0, [fp, #-52]
.L698:
	ldr	r0, [fp, #-52]
	cmp	r3, r0
	ble	.L703
	lsl	lr, r3, #2
	ldr	r0, [r6, #4]
	str	lr, [fp, #-56]
	ldr	lr, [ip, r3, lsl #2]
	cmp	r0, lr
	bls	.L699
	sub	r2, r2, r1
	ldr	r0, [ip]
	clz	r2, r2
	uxth	ip, r3
	lsr	r2, r2, #5
	cmp	r0, #0
	orrne	r2, r2, #1
	cmp	r2, #0
	addeq	r1, r1, #1
	strheq	r1, [r9]	@ movhi
	mov	r1, #0
.L701:
	uxth	r0, r1
	sxth	r2, r1
	cmp	ip, r0
	bhi	.L702
	ldr	r1, [r6, #4]
	ldr	r2, [r7, #3368]
	ldr	r0, [fp, #-56]
	str	r1, [r2, r0]
	lsl	r2, r3, #1
	ldr	r1, [r7, #348]
	strh	r8, [r1, r2]	@ movhi
.L703:
	cmp	r3, #0
	blt	.L681
	ldr	r0, .L750+20
	ldrh	r2, [r0, #-24]
	ldrh	r1, [r0]
	sub	r2, r2, #1
	sub	r2, r2, r1
	sxth	r2, r2
	cmp	r3, r2
	bgt	.L681
	add	r1, r1, #1
	ldr	r2, [r4, #3368]
	strh	r1, [r0]	@ movhi
	ldr	r1, [r6, #4]
	str	r1, [r2, r3, lsl #2]
	lsl	r3, r3, #1
	ldr	r2, [r4, #348]
	b	.L746
.L702:
	ldr	r0, [r5, #3368]
	add	r1, r1, #1
	add	lr, r0, r2, lsl #2
	ldr	lr, [lr, #4]
	str	lr, [r0, r2, lsl #2]
	lsl	r2, r2, #1
	ldr	r0, [r5, #348]
	add	lr, r0, r2
	ldrh	lr, [lr, #2]
	strh	lr, [r0, r2]	@ movhi
	b	.L701
.L699:
	sub	r3, r3, #1
	sxth	r3, r3
	b	.L698
.L682:
	ldr	r9, .L750+24
	movw	r2, #65535
	ldrh	r1, [r9]
	cmp	r1, r2
	strheq	r8, [r9]	@ movhi
	beq	.L747
	ldrh	r0, [r9, #4]
	cmp	r0, r2
	beq	.L706
	mov	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L706:
	ldr	r3, [r6, #4]
	ldr	r2, [r4, #2632]
	cmp	r2, r3
	strhcs	r8, [r9, #4]	@ movhi
	bcs	.L681
	ldrh	r3, [r9]
	strh	r8, [r9]	@ movhi
	strh	r3, [r9, #4]	@ movhi
	ldr	r3, [r6, #4]
.L747:
	str	r3, [r4, #2632]
	b	.L681
.L710:
	ldr	r2, [r5, #3356]
	ldrh	r3, [r2]
	cmp	r3, #0
	beq	.L712
.L715:
	ldr	r0, [r4, #348]
	ldrh	r2, [r0]
	cmp	r2, #0
	beq	.L713
.L714:
	movw	r3, #3436
	ldrh	r2, [r4, r3]
	ldr	r3, [r4, #328]
	cmp	r2, r3
	bls	.L745
	movw	r2, #1391
	ldr	r1, .L750+12
	ldr	r0, .L750+16
	bl	sftl_printk
.L745:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L712:
	movw	r1, #3436
	ldrh	r1, [r5, r1]
	cmp	r1, #0
	ldrne	r1, [r5, #328]
	beq	.L715
.L716:
	sxth	ip, r3
	cmp	ip, r1
	bcs	.L715
	lsl	r0, ip, #1
	add	r3, r3, #1
	ldrh	r0, [r2, r0]
	cmp	r0, #0
	beq	.L716
	mov	r3, ip
	mov	lr, #0
.L717:
	ldr	r2, [r7, #328]
	cmp	r3, r2
	bcs	.L715
	ldr	r1, [r7, #3356]
	lsl	r2, r3, #1
	sub	r0, r3, ip
	lsl	r5, r0, #1
	ldrh	r6, [r1, r2]
	strh	r6, [r1, r5]	@ movhi
	ldr	r1, [r7, #3380]
	ldr	r5, [r1, r3, lsl #2]
	add	r3, r3, #1
	sxth	r3, r3
	str	r5, [r1, r0, lsl #2]
	ldr	r1, [r7, #3356]
	strh	lr, [r1, r2]	@ movhi
	b	.L717
.L713:
	ldr	r3, .L750+20
	ldrh	r1, [r3]
	cmp	r1, #0
	ldrhne	ip, [r3, #-24]!
	movne	r1, r3
	beq	.L714
.L722:
	sxth	r3, r2
	cmp	r3, ip
	mov	lr, r3
	bge	.L714
	lsl	r5, r3, #1
	add	r2, r2, #1
	ldrh	r5, [r0, r5]
	cmp	r5, #0
	beq	.L722
	mov	r5, #0
.L723:
	ldrh	r2, [r1]
	cmp	r3, r2
	bge	.L714
	ldr	r0, [r4, #348]
	lsl	r2, r3, #1
	sub	ip, r3, lr
	lsl	r6, ip, #1
	ldrh	r7, [r0, r2]
	strh	r7, [r0, r6]	@ movhi
	ldr	r0, [r4, #3368]
	ldr	r6, [r0, r3, lsl #2]
	add	r3, r3, #1
	sxth	r3, r3
	str	r6, [r0, ip, lsl #2]
	ldr	r0, [r4, #348]
	strh	r5, [r0, r2]	@ movhi
	b	.L723
.L751:
	.align	2
.L750:
	.word	.LANCHOR0
	.word	.LANCHOR0+312
	.word	.LANCHOR0+3436
	.word	.LANCHOR1+231
	.word	.LC8
	.word	.LANCHOR0+344
	.word	.LANCHOR0+2624
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	FtlLoadBbt
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L784
	ldr	r8, .L784+4
	ldr	r3, [r4, #3300]
	ldr	r6, [r4, #3332]
	ldr	r7, .L784+8
	str	r3, [r4, #3460]
	str	r6, [r4, #3464]
	bl	FtlBbtMemInit
	movw	r3, #298
	ldrh	r5, [r4, r3]
	sub	r5, r5, #1
	uxth	r5, r5
.L753:
	ldrh	r3, [r8]
	sub	r3, r3, #16
	cmp	r5, r3
	ble	.L756
	lsl	r3, r5, #10
	mov	r2, #1
	mov	r1, r2
	mov	r0, r7
	str	r3, [r4, #3456]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	cmn	r3, #1
	bne	.L754
	ldr	r3, [r4, #3456]
	mov	r2, #1
	mov	r1, r2
	mov	r0, r7
	add	r3, r3, #1
	str	r3, [r4, #3456]
	bl	FlashReadPages
.L754:
	ldr	r3, [r4, #3452]
	cmn	r3, #1
	beq	.L755
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L755
	ldr	r2, [r6, #4]
	ldr	r3, .L784+12
	str	r2, [r4, #360]
	ldrh	r2, [r6, #8]
	strh	r5, [r3]	@ movhi
	strh	r2, [r3, #4]	@ movhi
.L756:
	ldr	r5, .L784+12
	movw	r2, #65535
	ldrh	r3, [r5]
	cmp	r3, r2
	beq	.L770
	ldrh	r3, [r5, #4]
	cmp	r3, r2
	beq	.L760
	lsl	r3, r3, #10
	mov	r2, #1
	mov	r1, r2
	ldr	r0, .L784+8
	str	r3, [r4, #3456]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	cmn	r3, #1
	beq	.L760
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L760
	ldr	r3, [r6, #4]
	ldr	r2, [r4, #360]
	cmp	r3, r2
	ldrhhi	r2, [r5, #4]
	strhi	r3, [r4, #360]
	ldrhhi	r3, [r6, #8]
	strhhi	r2, [r5]	@ movhi
	strhhi	r3, [r5, #4]	@ movhi
.L760:
	ldr	r8, .L784+8
	mov	r1, #1
	ldrh	r0, [r5]
	bl	FtlGetLastWrittenPage
	sxth	r7, r0
	add	r0, r0, #1
	strh	r0, [r5, #2]	@ movhi
.L762:
	cmp	r7, #0
	bge	.L765
	mov	r2, #251
	ldr	r1, .L784+16
	ldr	r0, .L784+20
	bl	sftl_printk
.L764:
	ldrh	r3, [r6, #10]
	ldrh	r0, [r6, #12]
	strh	r3, [r5, #6]	@ movhi
	movw	r3, #65535
	cmp	r0, r3
	beq	.L767
	ldr	r2, [r4, #228]
	cmp	r0, r2
	beq	.L767
	ldrh	r3, [r4, #242]
	lsr	r3, r3, #2
	cmp	r0, r3
	cmpcc	r2, r3
	bcs	.L767
	bl	FtlSysBlkNumInit
.L767:
	ldr	r6, .L784+24
	mov	r5, #0
	ldr	r7, .L784+28
.L768:
	ldrh	r3, [r4, #254]
	cmp	r5, r3
	bcc	.L769
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L755:
	sub	r5, r5, #1
	uxth	r5, r5
	b	.L753
.L765:
	ldrh	r3, [r5]
	mov	r2, #1
	mov	r1, r2
	mov	r0, r8
	orr	r3, r7, r3, lsl #10
	str	r3, [r4, #3456]
	ldr	r3, [r4, #3300]
	str	r3, [r4, #3460]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	cmn	r3, #1
	beq	.L763
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	beq	.L764
.L763:
	sub	r7, r7, #1
	sxth	r7, r7
	b	.L762
.L769:
	ldrh	r2, [r7]
	ldr	r1, [r4, #3460]
	ldr	r0, [r6, #4]!
	lsl	r2, r2, #2
	mla	r1, r5, r2, r1
	add	r5, r5, #1
	bl	ftl_memcpy
	b	.L768
.L770:
	mvn	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L785:
	.align	2
.L784:
	.word	.LANCHOR0
	.word	.LANCHOR0+298
	.word	.LANCHOR0+3452
	.word	.LANCHOR0+352
	.word	.LANCHOR1+245
	.word	.LC8
	.word	.LANCHOR0+376
	.word	.LANCHOR0+3388
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	FlashProgPages
	.syntax unified
	.arm
	.fpu softvfp
	.type	FlashProgPages, %function
FlashProgPages:
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #40
	mov	r8, #0
	ldr	r6, .L823
	mov	r9, r1
	mov	r4, r0
	mov	r5, r0
	str	r2, [fp, #-76]
	ldrh	r10, [r6, #12]
	lsl	r3, r10, #3
	str	r3, [fp, #-72]
.L787:
	cmp	r8, r9
	bne	.L801
	ldr	r7, .L823+4
	mov	r5, #0
	ldr	r9, .L823+8
.L802:
	cmp	r8, r5
	beq	.L795
	ldr	r2, [r6, #3320]
	mov	r3, #0
	mov	r1, #1
	sub	r0, fp, #60
	str	r3, [r2]
	ldr	r2, [r6, #3328]
	str	r3, [r2]
	ldr	r3, [r4, #4]
	ldr	r2, [fp, #-76]
	str	r3, [fp, #-56]
	ldr	r3, [r6, #3320]
	str	r3, [fp, #-52]
	ldr	r3, [r6, #3328]
	str	r3, [fp, #-48]
	bl	FlashReadPages
	ldr	r10, [fp, #-60]
	cmn	r10, #1
	bne	.L803
	ldr	r1, [r4, #4]
	mov	r0, r7
	bl	sftl_printk
	str	r10, [r4]
.L803:
	ldr	r10, [fp, #-60]
	cmp	r10, #256
	bne	.L804
	ldr	r1, [r4, #4]
	mov	r0, r9
	bl	sftl_printk
	str	r10, [r4]
.L804:
	ldr	r3, [r4, #12]
	cmp	r3, #0
	beq	.L805
	ldr	r2, [r3]
	ldr	r3, [r6, #3328]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L805
	ldr	r1, [r4, #4]
	ldr	r0, .L823+12
	bl	sftl_printk
	mvn	r3, #0
	str	r3, [r4]
.L805:
	ldr	r3, [r4, #8]
	cmp	r3, #0
	beq	.L806
	ldr	r2, [r3]
	ldr	r3, [r6, #3320]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L806
	ldr	r1, [r4, #4]
	ldr	r0, .L823+16
	bl	sftl_printk
	mvn	r3, #0
	str	r3, [r4]
.L806:
	add	r5, r5, #1
	add	r4, r4, #20
	b	.L802
.L801:
	ldr	r3, [r5, #8]
	cmp	r3, #0
	beq	.L788
	ldr	r3, [r5, #12]
	cmp	r3, #0
	bne	.L789
.L788:
	mov	r2, #148
	ldr	r1, .L823+20
	ldr	r0, .L823+24
	bl	sftl_printk
.L789:
	sub	r2, fp, #68
	sub	r1, fp, #64
	mov	r0, r5
	bl	l2p_addr_tran
	ldr	r7, [fp, #-68]
	cmp	r7, #3
	bls	.L790
.L822:
	mvn	r3, #0
	str	r3, [r5]
	b	.L791
.L790:
	cmp	r7, #0
	bne	.L792
	ldr	r3, [fp, #-64]
	ldr	r2, [fp, #-72]
	cmp	r2, r3
	bls	.L792
	ldr	r5, .L823+20
	ldr	r6, .L823+28
	b	.L821
.L794:
	mvn	r3, #0
	ldr	r2, [r4, #-16]
	str	r3, [r4, #-20]
	mov	r1, r5
	mov	r0, r6
	add	r7, r7, #1
	bl	sftl_printk
	mov	r3, #16
	mov	r2, #4
	ldr	r1, [r4, #-12]
	ldr	r0, .L823+32
	bl	rknand_print_hex
	mov	r3, #4
	ldr	r1, [r4, #-8]
	mov	r2, r3
	ldr	r0, .L823+36
	bl	rknand_print_hex
.L821:
	cmp	r7, r9
	add	r4, r4, #20
	bne	.L794
	bl	dump_stack
.L795:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L792:
	ldr	r1, [r5, #8]
	ldr	r3, .L823
	tst	r1, #63
	moveq	r7, r1
	beq	.L796
	ldr	r7, [r3, #3320]
	cmp	r1, r7
	beq	.L796
	ldr	r2, .L823+40
	mov	r0, r7
	str	r3, [fp, #-80]
	ldrh	r2, [r2]
	lsl	r2, r2, #9
	bl	ftl_memcpy
	ldr	r3, [fp, #-80]
.L796:
	ldr	ip, [r3, #3252]
	mov	r2, r7
	ldr	r3, [r5, #12]
	ldr	r1, [fp, #-64]
	ldrb	r0, [fp, #-68]	@ zero_extendqisi2
	blx	ip
	ldr	r2, .L823
	cmp	r0, #0
	mvnne	r3, #0
	streq	r0, [r5]
	strne	r3, [r5]
	ldrh	r3, [r2, #14]
	cmp	r3, #4
	bne	.L791
	ldr	r3, [r5, #12]
	ldr	r1, [fp, #-64]
	ldr	ip, [r2, #3252]
	add	r2, r7, #2048
	add	r3, r3, #8
	ldrb	r0, [fp, #-68]	@ zero_extendqisi2
	add	r1, r10, r1
	blx	ip
	cmp	r0, #0
	bne	.L822
.L791:
	add	r8, r8, #1
	add	r5, r5, #20
	b	.L787
.L824:
	.align	2
.L823:
	.word	.LANCHOR0
	.word	.LC95
	.word	.LC96
	.word	.LC97
	.word	.LC98
	.word	.LANCHOR1+256
	.word	.LC8
	.word	.LC86
	.word	.LC87
	.word	.LC88
	.word	.LANCHOR0+258
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FtlLowFormatEraseBlock
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	@ args = 0, pretend = 0, frame = 12
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #12
	mov	r10, #0
	ldr	ip, .L867
	uxtb	r5, r1
	uxth	r3, r0
	mov	r6, r10
	mov	r4, r10
	mov	r9, ip
	mov	r8, #20
	str	r3, [fp, #-44]
	str	r3, [ip, #3284]
.L826:
	ldrh	r1, [r9, #232]
	uxth	r3, r10
	cmp	r1, r3
	bhi	.L830
	cmp	r6, #0
	beq	.L825
	mov	r7, #0
	mov	r8, #20
	mov	r2, r6
	mov	r1, #0
	ldr	r0, [r9, #3268]
	bl	FlashEraseBlocks
.L833:
	uxth	r3, r7
	cmp	r6, r3
	bhi	.L835
.L836:
	cmp	r5, #0
	mov	r7, #0
	ldrne	r3, .L867+4
	moveq	r3, #2
	streq	r5, [fp, #-48]
	streq	r3, [fp, #-52]
	ldrhne	r3, [r3]
	strne	r3, [fp, #-52]
	movne	r3, #1
	strne	r3, [fp, #-48]
.L845:
	mov	r10, #0
	mov	r6, r10
.L837:
	ldrh	r1, [r9, #232]
	uxth	r3, r10
	cmp	r1, r3
	bhi	.L840
	cmp	r6, #0
	beq	.L825
	mov	r8, #0
	mov	r10, #20
	mov	r3, #1
	ldr	r2, [fp, #-48]
	mov	r1, r6
	ldr	r0, [r9, #3268]
	bl	FlashProgPages
.L842:
	uxth	r3, r8
	cmp	r6, r3
	bhi	.L844
	add	r7, r7, #1
	ldr	r2, [fp, #-52]
	uxth	r3, r7
	cmp	r2, r3
	bhi	.L845
	mov	r7, #0
	mov	r8, #20
.L846:
	uxth	r3, r7
	cmp	r6, r3
	bhi	.L848
	ldr	r3, [fp, #-44]
	adds	r5, r5, #0
	movne	r5, #1
	cmp	r3, #63
	orrls	r5, r5, #1
	cmp	r5, #0
	beq	.L825
	mov	r2, r6
	ldr	r1, [fp, #-48]
	ldr	r0, [r9, #3268]
	bl	FlashEraseBlocks
.L825:
	mov	r0, r4
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L830:
	uxth	r3, r10
	ldr	r0, [r9, #3268]
	mov	r2, #0
	mul	r1, r8, r3
	add	r3, r9, r3
	str	r2, [r0, r1]
	ldr	r1, [fp, #-44]
	ldrb	r0, [r3, #260]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r5, #0
	mov	r7, r0
	beq	.L827
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L828
.L827:
	mov	r0, r7
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	addne	r4, r4, #1
	uxthne	r4, r4
	bne	.L828
	ldr	r1, [r9, #3268]
	lsl	r7, r7, #10
	ldr	r3, [r9, #3316]
	mla	r1, r8, r6, r1
	str	r3, [r1, #8]
	ldr	r3, .L867+8
	str	r7, [r1, #4]
	ldrh	r3, [r3]
	mul	r3, r6, r3
	add	r6, r6, #1
	uxth	r6, r6
	add	r0, r3, #3
	cmp	r3, #0
	movlt	r3, r0
	ldr	r0, [r9, #3336]
	bic	r3, r3, #3
	add	r3, r0, r3
	str	r3, [r1, #12]
.L828:
	add	r10, r10, #1
	b	.L826
.L835:
	mul	r3, r8, r7
	ldr	r2, [r9, #3268]
	add	r1, r2, r3
	ldr	r3, [r2, r3]
	cmn	r3, #1
	bne	.L834
	ldr	r0, [r1, #4]
	add	r4, r4, #1
	uxth	r4, r4
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
.L834:
	add	r7, r7, #1
	b	.L833
.L840:
	uxth	r3, r10
	mov	r2, #20
	ldr	r0, [r9, #3268]
	mul	r1, r2, r3
	mov	r2, #0
	add	r3, r9, r3
	str	r2, [r0, r1]
	ldr	r1, [fp, #-44]
	ldrb	r0, [r3, #260]	@ zero_extendqisi2
	bl	V2P_block
	cmp	r5, #0
	mov	r8, r0
	beq	.L838
	bl	IsBlkInVendorPart
	cmp	r0, #0
	bne	.L839
.L838:
	mov	r0, r8
	bl	FtlBbmIsBadBlock
	cmp	r0, #0
	bne	.L839
	ldr	r1, [r9, #3268]
	mov	r3, #20
	add	r8, r7, r8, lsl #10
	mla	r1, r3, r6, r1
	ldr	r3, [r9, #3312]
	str	r3, [r1, #8]
	ldr	r3, .L867+8
	str	r8, [r1, #4]
	ldrh	r3, [r3]
	mul	r3, r6, r3
	add	r6, r6, #1
	uxth	r6, r6
	add	r0, r3, #3
	cmp	r3, #0
	movlt	r3, r0
	ldr	r0, [r9, #3316]
	bic	r3, r3, #3
	add	r3, r0, r3
	str	r3, [r1, #12]
.L839:
	add	r10, r10, #1
	b	.L837
.L844:
	mul	r3, r10, r8
	ldr	r2, [r9, #3268]
	add	r1, r2, r3
	ldr	r3, [r2, r3]
	cmp	r3, #0
	beq	.L843
	ldr	r0, [r1, #4]
	add	r4, r4, #1
	uxth	r4, r4
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
.L843:
	add	r8, r8, #1
	b	.L842
.L848:
	cmp	r5, #0
	beq	.L847
	mul	r3, r8, r7
	ldr	r2, [r9, #3268]
	add	r1, r2, r3
	ldr	r3, [r2, r3]
	cmp	r3, #0
	bne	.L847
	ldr	r0, [r1, #4]
	mov	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L847:
	add	r7, r7, #1
	b	.L846
.L868:
	.align	2
.L867:
	.word	.LANCHOR0
	.word	.LANCHOR0+304
	.word	.LANCHOR0+312
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.global	FlashTestBlk
	.syntax unified
	.arm
	.fpu softvfp
	.type	FlashTestBlk, %function
FlashTestBlk:
	@ args = 0, pretend = 0, frame = 84
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #84
	uxth	r5, r0
	cmp	r5, #11
	movls	r4, #0
	bls	.L869
	ldr	r3, .L874
	sub	r0, fp, #84
	mov	r2, #32
	mov	r1, #165
	str	r0, [fp, #-92]
	lsl	r5, r5, #10
	ldr	r4, [r3, #3324]
	str	r4, [fp, #-96]
	bl	memset
	ldr	r3, .L874+4
	mov	r2, #1
	mov	r1, r2
	sub	r0, fp, #104
	str	r3, [r4]	@ unaligned
	str	r3, [r4, #4]	@ unaligned
	str	r5, [fp, #-100]
	bl	FlashEraseBlocks
	mov	r3, #1
	sub	r0, fp, #104
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldr	r3, [fp, #-104]
	cmp	r3, #0
	mvnne	r4, #0
	bne	.L871
	add	r3, r5, #1
	sub	r0, fp, #104
	str	r3, [fp, #-100]
	mov	r3, #1
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldr	r4, [fp, #-104]
	adds	r4, r4, #0
	movne	r4, #1
	rsb	r4, r4, #0
.L871:
	mov	r2, #1
	mov	r1, #0
	sub	r0, fp, #104
	str	r5, [fp, #-100]
	bl	FlashEraseBlocks
.L869:
	mov	r0, r4
	sub	sp, fp, #20
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L875:
	.align	2
.L874:
	.word	.LANCHOR0
	.word	1515870810
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FtlBbmTblFlush
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #8
	mov	r5, #0
	ldr	r4, .L888
	mov	r1, #0
	ldr	r7, .L888+4
	ldr	r3, [r4, #3332]
	add	r6, r4, #376
	ldr	r0, [r4, #3300]
	str	r3, [r4, #3464]
	movw	r3, #310
	ldrh	r2, [r4, r3]
	str	r0, [r4, #3460]
	bl	ftl_memset
.L877:
	ldrh	r3, [r4, #254]
	cmp	r5, r3
	blt	.L878
	ldr	r6, [r4, #3464]
	mov	r2, #16
	mov	r1, #255
	ldr	r5, .L888+8
	ldr	r10, .L888+12
	mov	r7, #0
	mov	r0, r6
	mov	r8, r7
	bl	memset
	ldr	r3, .L888+16
	sub	r9, r5, #48
	strh	r3, [r6]	@ movhi
	ldr	r3, [r4, #360]
	str	r3, [r6, #4]
	ldrh	r3, [r5]
	strh	r3, [r6, #2]	@ movhi
	ldrh	r3, [r5, #4]
	strh	r3, [r6, #8]	@ movhi
	ldrh	r3, [r5, #6]
	strh	r3, [r6, #10]	@ movhi
	ldr	r3, [r4, #228]
	strh	r3, [r6, #12]	@ movhi
	ldr	r3, [r4, #2604]
	strh	r3, [r6, #14]	@ movhi
.L879:
	ldr	r3, [r4, #3300]
	mov	ip, #0
	ldrh	r2, [r5, #2]
	ldrh	r1, [r5]
	str	r3, [r4, #3460]
	ldr	r3, [r4, #3332]
	str	ip, [r4, #3452]
	str	ip, [fp, #-44]
	str	r3, [r4, #3464]
	orr	r3, r2, r1, lsl #10
	ldrh	r0, [r6, #10]
	str	r3, [r4, #3456]
	ldrh	r3, [r5, #4]
	str	r0, [sp]
	mov	r0, r10
	bl	sftl_printk
	ldrh	r3, [r9]
	ldrh	r2, [r5, #2]
	ldr	ip, [fp, #-44]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L880
	ldr	r3, [r4, #360]
	ldrh	r2, [r5]
	ldr	r0, [r4, #3268]
	add	r3, r3, #1
	strh	ip, [r5, #2]	@ movhi
	str	r3, [r4, #360]
	str	r3, [r6, #4]
	ldrh	r3, [r5, #4]
	strh	r2, [r6, #8]	@ movhi
	strh	r2, [r5, #4]	@ movhi
	mov	r2, #1
	strh	r3, [r5]	@ movhi
	mov	r1, r2
	lsl	r3, r3, #10
	str	r3, [r4, #3456]
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
.L880:
	mov	r3, #1
	ldr	r0, .L888+20
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldrh	r3, [r5, #2]
	add	r3, r3, #1
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, [r4, #3452]
	cmn	r3, #1
	bne	.L881
	add	r7, r7, #1
	ldr	r1, [r4, #3456]
	uxth	r7, r7
	ldr	r0, .L888+24
	bl	sftl_printk
	cmp	r7, #3
	bls	.L879
	mov	r2, r7
	ldr	r1, [r4, #3456]
	ldr	r0, .L888+28
	bl	sftl_printk
.L883:
	b	.L883
.L878:
	ldrh	r2, [r7]
	ldr	r3, [r4, #3460]
	ldr	r1, [r6, #4]!
	mul	r0, r2, r5
	lsl	r2, r2, #2
	add	r5, r5, #1
	add	r0, r3, r0, lsl #2
	bl	ftl_memcpy
	b	.L877
.L884:
	mov	r8, #1
	b	.L879
.L881:
	add	r8, r8, #1
	cmp	r8, #1
	ble	.L884
	cmp	r3, #256
	beq	.L879
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L889:
	.align	2
.L888:
	.word	.LANCHOR0
	.word	.LANCHOR0+3388
	.word	.LANCHOR0+352
	.word	.LC99
	.word	-3887
	.word	.LANCHOR0+3452
	.word	.LC100
	.word	.LC101
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	allocate_data_superblock
	.syntax unified
	.arm
	.fpu softvfp
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #16
	mov	r5, r0
	ldr	r4, .L926
	add	r10, r4, #2528
.L891:
	ldrh	r2, [r10]
	ldrh	r3, [r4, #224]
	add	r3, r3, r2
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	ble	.L892
	movw	r2, #2660
	ldr	r1, .L926+4
	ldr	r0, .L926+8
	bl	sftl_printk
.L892:
	ldr	r3, .L926+12
	cmp	r5, r3
	movne	r1, #0
	bne	.L893
	ldrh	r3, [r4, #224]
	ldr	r1, [r4, #3164]
	mul	r2, r1, r3
	lsr	r1, r3, #1
	add	r1, r1, #1
	add	r1, r1, r2, lsr #2
	uxth	r1, r1
	cmp	r1, #0
	subne	r1, r1, #1
	uxthne	r1, r1
.L893:
	ldr	r0, .L926+16
	bl	List_pop_index_node
	ldrh	r3, [r4, #224]
	mov	r8, r0
	uxth	r9, r0
	cmp	r3, #0
	bne	.L894
	movw	r2, #2669
	ldr	r1, .L926+4
	ldr	r0, .L926+8
	bl	sftl_printk
.L894:
	ldrh	r3, [r4, #224]
	sub	r3, r3, #1
	strh	r3, [r4, #224]	@ movhi
	ldrh	r3, [r4, #240]
	cmp	r3, r9
	bls	.L891
	uxth	r8, r8
	ldr	r3, [r4, #72]
	lsl	r7, r8, #1
	ldrh	r6, [r3, r7]
	cmp	r6, #0
	bne	.L891
	strh	r9, [r5]	@ movhi
	mov	r0, r5
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L896
	ldr	r3, [r4, #72]
	mvn	r2, #0
	mov	r0, r8
	strh	r2, [r3, r7]	@ movhi
	bl	INSERT_DATA_LIST
	ldrh	r2, [r10]
	ldrh	r3, [r4, #224]
	add	r3, r3, r2
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	ble	.L891
	movw	r2, #2683
	ldr	r1, .L926+4
	ldr	r0, .L926+8
	bl	sftl_printk
	b	.L891
.L896:
	ldrh	r2, [r10]
	ldrh	r3, [r4, #224]
	add	r3, r3, r2
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	ble	.L898
	movw	r2, #2686
	ldr	r1, .L926+4
	ldr	r0, .L926+8
	bl	sftl_printk
.L898:
	ldr	r1, [r4, #3268]
	mov	r2, #20
	ldrh	ip, [r4, #232]
	add	r0, r5, #16
	str	r2, [fp, #-48]
	mov	r3, r1
	mla	ip, r2, ip, r1
	str	ip, [fp, #-44]
	mov	ip, #0
.L899:
	ldr	r2, [fp, #-44]
	cmp	r2, r3
	bne	.L901
	cmp	r6, #0
	bne	.L902
	movw	r2, #2697
	ldr	r1, .L926+4
	ldr	r0, .L926+8
	bl	sftl_printk
.L902:
	ldrh	r3, [r4, #172]
	cmp	r3, r9
	bne	.L903
	movw	r2, #2699
	ldr	r1, .L926+4
	ldr	r0, .L926+8
	bl	sftl_printk
.L903:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r1, [r4, #2536]
	ldr	r2, .L926+20
	cmp	r3, #0
	ldrh	r3, [r1, r7]
	bne	.L904
	cmp	r3, #0
	ldrhne	r0, [r2]
	moveq	r3, #2
	addne	r3, r3, r0
	strh	r3, [r1, r7]	@ movhi
	ldr	r3, [r4, #2596]
	add	r3, r3, #1
	str	r3, [r4, #2596]
.L907:
	ldr	r3, [r4, #2536]
	ldr	r1, [r4, #2612]
	ldrh	r2, [r2]
	ldrh	r3, [r3, r7]
	ldr	r0, [r4, #2596]
	cmp	r3, r1
	ldrh	r1, [r4, #240]
	strhi	r3, [r4, #2612]
	ldr	r3, [r4, #2600]
	mla	r0, r0, r2, r3
	bl	__udivsi3
	ldr	r2, [r4, #3348]
	ldr	r1, [r4, #3268]
	str	r0, [r4, #2604]
	ldr	r3, [r2, #16]
	ldr	r0, .L926+24
	add	r3, r3, #1
	str	r3, [r2, #16]
	mov	r2, #20
	mla	r2, r2, r6, r1
	add	r3, r1, #4
	add	r2, r2, #24
.L909:
	add	r3, r3, #20
	cmp	r2, r3
	bne	.L910
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	mov	r2, r6
	ldr	r0, [r4, #3268]
	bl	FlashEraseBlocks
	mov	r3, #0
	mov	ip, #20
	mov	r1, r3
.L911:
	uxth	r2, r3
	cmp	r6, r2
	bhi	.L913
	cmp	r1, #0
	ble	.L914
	mov	r0, r8
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L914:
	ldrb	r2, [r5, #7]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L915
	ldr	r3, [r4, #72]
	mvn	r2, #0
	strh	r2, [r3, r7]	@ movhi
	b	.L891
.L901:
	str	ip, [r3, #8]
	movw	lr, #65535
	str	ip, [r3, #12]
	add	r3, r3, #20
	ldrh	r2, [r0], #2
	cmp	r2, lr
	ldrne	lr, [fp, #-48]
	lslne	r2, r2, #10
	mlane	lr, lr, r6, r1
	addne	r6, r6, #1
	uxthne	r6, r6
	strne	r2, [lr, #4]
	b	.L899
.L904:
	add	r3, r3, #1
	strh	r3, [r1, r7]	@ movhi
	ldr	r3, [r4, #2600]
	add	r3, r3, #1
	str	r3, [r4, #2600]
	b	.L907
.L910:
	ldr	r1, [r3, #-20]
	and	r1, r1, r0
	str	r1, [r3, #-20]
	b	.L909
.L913:
	mul	r2, ip, r3
	ldr	lr, [r4, #3268]
	add	r0, lr, r2
	ldr	r2, [lr, r2]
	cmn	r2, #1
	bne	.L912
	ldr	r0, [r0, #4]
	add	r1, r1, #1
	str	ip, [fp, #-56]
	str	r2, [fp, #-52]
	ubfx	r0, r0, #10, #16
	str	r3, [fp, #-48]
	str	r1, [fp, #-44]
	bl	FtlBbmMapBadBlock
	ldr	r3, [fp, #-48]
	ldr	r2, [fp, #-52]
	ldr	ip, [fp, #-56]
	ldr	r1, [fp, #-44]
	add	r0, r5, r3, lsl #1
	strh	r2, [r0, #16]	@ movhi
	ldrb	r2, [r5, #7]	@ zero_extendqisi2
	sub	r2, r2, #1
	strb	r2, [r5, #7]
.L912:
	add	r3, r3, #1
	b	.L911
.L915:
	movw	r3, #302
	ldrh	r3, [r4, r3]
	strh	r9, [r5]	@ movhi
	smulbb	r3, r3, r2
	mov	r2, #0
	strh	r2, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	ldr	r2, [r4, #2588]
	uxth	r3, r3
	ldr	r1, [r4, #72]
	strh	r3, [r5, #4]	@ movhi
	str	r2, [r5, #12]
	add	r2, r2, #1
	str	r2, [r4, #2588]
	ldrh	r2, [r5]
	lsl	r2, r2, #1
	strh	r3, [r1, r2]	@ movhi
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L916
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L917
.L916:
	mov	r2, #2752
	ldr	r1, .L926+4
	ldr	r0, .L926+8
	bl	sftl_printk
.L917:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L927:
	.align	2
.L926:
	.word	.LANCHOR0
	.word	.LANCHOR1+271
	.word	.LC8
	.word	.LANCHOR0+124
	.word	.LANCHOR0+2532
	.word	.LANCHOR0+292
	.word	-1024
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #8
	uxth	r3, r0
	ldr	r6, .L940
	str	r3, [fp, #-44]
	movw	r3, #3206
	ldrh	r3, [r6, r3]
	cmp	r3, #0
	movne	r7, #0
	ldrne	r10, .L940+4
	bne	.L930
.L929:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L936:
	uxtah	r3, r6, r7
	ldr	r1, [fp, #-44]
	mov	r8, #0
	ldrb	r0, [r3, #260]	@ zero_extendqisi2
	bl	V2P_block
	mov	r9, r0
.L931:
	ldrh	r3, [r10]
	uxth	r4, r8
	cmp	r3, r4
	bhi	.L935
	add	r7, r7, #1
.L930:
	ldrh	r2, [r6, #232]
	uxth	r3, r7
	cmp	r2, r3
	bhi	.L936
	bl	FtlGcReFreshBadBlk
	b	.L929
.L935:
	uxth	r3, r8
	ldr	r5, .L940+8
	lsl	r2, r3, #1
	ldrh	r2, [r5, r2]
	cmp	r2, r9
	bne	.L932
	mov	r1, r9
	ldr	r0, .L940+12
	str	r3, [fp, #-48]
	bl	sftl_printk
	mov	r0, r9
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [fp, #-48]
	ldrh	r2, [r10]
	add	r3, r5, r3, lsl #1
.L933:
	cmp	r4, r2
	bcc	.L934
	sub	r2, r2, #1
	strh	r2, [r10]	@ movhi
.L932:
	add	r8, r8, #1
	b	.L931
.L934:
	ldrh	r1, [r3, #2]!
	add	r4, r4, #1
	uxth	r4, r4
	strh	r1, [r3, #-2]	@ movhi
	b	.L933
.L941:
	.align	2
.L940:
	.word	.LANCHOR0
	.word	.LANCHOR0+3206
	.word	.LANCHOR0+3208
	.word	.LC102
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.global	update_vpc_list
	.syntax unified
	.arm
	.fpu softvfp
	.type	update_vpc_list, %function
update_vpc_list:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L954
	uxth	r5, r0
	ldr	r2, [r4, #72]
	lsl	r3, r5, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L943
	ldrh	r3, [r4, #222]
	cmp	r3, r5
	mvneq	r3, #0
	strheq	r3, [r4, #222]	@ movhi
	ldrh	r3, [r4, #220]
	cmp	r3, r5
	mvneq	r3, #0
	strheq	r3, [r4, #220]	@ movhi
	ldrh	r3, [r4, #172]
	cmp	r3, r5
	mvneq	r3, #0
	strheq	r3, [r4, #172]	@ movhi
	beq	.L947
	ldrh	r3, [r4, #24]
	cmp	r3, r5
	beq	.L952
	ldrh	r3, [r4, #76]
	cmp	r3, r5
	beq	.L952
	ldrh	r3, [r4, #124]
	cmp	r3, r5
	beq	.L952
.L947:
	ldr	r6, .L954+4
	mov	r1, r5
	ldr	r0, .L954+8
	bl	List_remove_node
	ldrh	r3, [r6]
	cmp	r3, #0
	bne	.L949
	movw	r2, #2828
	ldr	r1, .L954+12
	ldr	r0, .L954+16
	bl	sftl_printk
.L949:
	ldrh	r3, [r6]
	mov	r0, r5
	sub	r3, r3, #1
	strh	r3, [r6]	@ movhi
	bl	free_data_superblock
	mov	r0, r5
	bl	FtlGcFreeBadSuperBlk
	ldrh	r2, [r6]
	ldrh	r3, [r4, #224]
	add	r3, r3, r2
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	ble	.L953
	movw	r2, #2831
	ldr	r1, .L954+12
	ldr	r0, .L954+16
	bl	sftl_printk
.L953:
	mov	r0, #1
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L943:
	mov	r0, r5
	bl	List_update_data_list
.L952:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L955:
	.align	2
.L954:
	.word	.LANCHOR0
	.word	.LANCHOR0+2528
	.word	.LANCHOR0+2520
	.word	.LANCHOR1+296
	.word	.LC8
	.size	update_vpc_list, .-update_vpc_list
	.align	2
	.global	decrement_vpc_count
	.syntax unified
	.arm
	.fpu softvfp
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	uxth	r6, r0
	movw	r3, #65535
	cmp	r6, r3
	ldr	r4, .L968
	beq	.L957
	ldr	r3, [r4, #72]
	lsl	r5, r6, #1
	ldrh	r2, [r3, r5]
	cmp	r2, #0
	subne	r2, r2, #1
	strhne	r2, [r3, r5]	@ movhi
	bne	.L957
	mov	r1, r6
	ldr	r0, .L968+4
	bl	sftl_printk
	ldr	r3, [r4, #72]
	ldrh	r5, [r3, r5]
	cmp	r5, #0
	beq	.L959
.L966:
	mov	r5, #0
.L956:
	mov	r0, r5
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L959:
	movw	r2, #2846
.L967:
	ldr	r1, .L968+8
	ldr	r0, .L968+12
	bl	sftl_printk
	b	.L956
.L957:
	movw	r7, #3438
	movw	r3, #65535
	ldrh	r0, [r4, r7]
	cmp	r0, r3
	strheq	r6, [r4, r7]	@ movhi
	beq	.L966
.L961:
	cmp	r6, r0
	beq	.L966
	bl	update_vpc_list
	ldr	r2, [r4, #2516]
	adds	r5, r0, #0
	ldr	r3, [r4, #2520]
	movne	r5, #1
	ldr	r1, [r4, #72]
	strh	r6, [r4, r7]	@ movhi
	sub	r3, r3, r2
	asr	r3, r3, #3
	uxth	r2, r3
	uxth	r3, r3
	lsl	r2, r2, #1
	cmp	r3, r6
	ldrh	r2, [r1, r2]
	clz	r2, r2
	lsr	r2, r2, #5
	moveq	r2, #0
	cmp	r2, #0
	beq	.L956
	movw	r2, #2862
	b	.L967
.L969:
	.align	2
.L968:
	.word	.LANCHOR0
	.word	.LC103
	.word	.LANCHOR1+312
	.word	.LC8
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	FtlSuperblockPowerLostFix
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	@ args = 0, pretend = 0, frame = 20
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #20
	mvn	r3, #0
	str	r3, [fp, #-36]
	mov	r6, #0
	ldr	r3, .L983
	movw	r1, #61589
	mov	r4, r0
	ldr	r2, [r3, #3300]
	mov	r8, r3
	ldr	r5, [r3, #3332]
	str	r2, [fp, #-44]
	mvn	r2, #2
	str	r5, [fp, #-40]
	str	r2, [r5, #8]
	mvn	r2, #1
	str	r2, [r5, #12]
	ldrh	r2, [r0]
	strh	r6, [r5]	@ movhi
	strh	r2, [r5, #2]	@ movhi
	ldr	r2, [r3, #3300]
	str	r1, [r2]
	ldr	r1, .L983+4
	ldr	r2, [r3, #3300]
	str	r1, [r2, #4]
	ldrh	r2, [r0, #4]
	tst	r2, #1
	moveq	r7, #6
	movne	r7, #7
.L976:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L972
.L973:
	ldrh	r3, [r4]
	ldr	r1, [r8, #72]
	ldrh	r0, [r4, #4]
	lsl	r3, r3, #1
	ldrh	r2, [r1, r3]
	sub	r2, r2, r0
	strh	r2, [r1, r3]	@ movhi
	movw	r3, #302
	ldrh	r3, [r8, r3]
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	sub	sp, fp, #32
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L972:
	mov	r0, r4
	bl	get_new_active_ppa
	cmn	r0, #1
	str	r0, [fp, #-48]
	beq	.L973
	ldr	r3, [r8, #2592]
	mov	r1, #1
	sub	r0, fp, #52
	str	r3, [r5, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, r6
	str	r3, [r8, #2592]
	mov	r3, #0
	mov	r2, r3
	bl	FlashProgPages
	ldrh	r0, [r4]
	bl	decrement_vpc_count
	subs	r7, r7, #1
	bne	.L976
	b	.L973
.L984:
	.align	2
.L983:
	.word	.LANCHOR0
	.word	305419896
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlMakeBbt
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #8
	mov	r8, #0
	ldr	r7, .L1005
	bl	FtlBbtMemInit
	bl	FtlLoadFactoryBbt
	sub	r6, r7, #380
	sub	r9, r7, #18
	sub	r4, r7, #28
	mov	r5, r6
.L986:
	ldrh	r3, [r6, #254]
	cmp	r8, r3
	bcc	.L992
	ldr	r6, .L1005+4
	mov	r0, #0
.L993:
	ldrh	r2, [r6]
	uxth	r3, r0
	add	r7, r0, #1
	cmp	r2, r3
	bhi	.L994
	ldrh	r6, [r4, #12]
	movw	r7, #65535
	sub	r6, r6, #1
	uxth	r6, r6
.L995:
	ldrh	r3, [r4, #12]
	sub	r3, r3, #48
	cmp	r6, r3
	ble	.L999
	mov	r0, r6
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L996
	mov	r0, r6
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L997
	mov	r0, r6
	bl	FtlBbmMapBadBlock
.L996:
	sub	r6, r6, #1
	uxth	r6, r6
	b	.L995
.L992:
	ldr	r3, [r6, #3332]
	ldrh	r2, [r9, #2]!
	ldr	r0, [r6, #3300]
	str	r3, [fp, #-48]
	str	r3, [r6, #3464]
	movw	r3, #65535
	cmp	r2, r3
	str	r0, [r6, #3460]
	ldr	r3, .L1005+8
	beq	.L987
	ldrh	r10, [r3]
	ldr	r0, .L1005+12
	str	r3, [fp, #-44]
	mla	r10, r8, r10, r2
	lsl	r2, r10, #10
	str	r2, [r6, #3456]
	mov	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r3, [fp, #-44]
	ldr	r1, [r6, #3460]
	ldr	r0, [r7]
	ldrh	r2, [r3]
	add	r2, r2, #7
	asr	r2, r2, #3
	bl	ftl_memcpy
.L988:
	uxth	r0, r10
	add	r8, r8, #1
	add	r7, r7, #4
	bl	FtlBbmMapBadBlock
	b	.L986
.L987:
	mov	r1, r8
	str	r3, [fp, #-44]
	bl	FlashGetBadBlockList
	ldr	r1, [r7]
	ldr	r0, [r6, #3460]
	bl	FtlBbt2Bitmap
	ldr	r3, [fp, #-44]
	ldrh	r3, [r3]
.L1004:
	sub	r3, r3, #1
	uxth	r3, r3
	str	r3, [fp, #-44]
.L989:
	ldr	r10, .L1005+8
	ldr	r3, [fp, #-44]
	ldrh	r0, [r10]
	smlabb	r0, r0, r8, r3
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L990
	ldrh	r3, [fp, #-44]
	mov	r1, #16
	ldr	r0, [r5, #3332]
	strh	r3, [r9]	@ movhi
	bl	__memzero
	mov	r1, #4096
	ldr	r0, [r5, #3300]
	bl	__memzero
	ldr	r2, [fp, #-48]
	ldr	r3, .L1005+16
	strh	r3, [r2]	@ movhi
	mov	r3, #0
	str	r3, [r2, #4]
	ldrh	r3, [r9]
	ldrh	r10, [r10]
	strh	r3, [r2, #2]	@ movhi
	ldrh	r3, [r9]
	ldr	r1, [r7]
	ldr	r0, [r5, #3460]
	mla	r10, r8, r10, r3
	lsl	r3, r10, #10
	str	r3, [r5, #3456]
	ldr	r3, .L1005+20
	ldrh	r2, [r3]
	lsl	r2, r2, #2
	bl	ftl_memcpy
	mov	r2, #1
	ldr	r0, .L1005+12
	mov	r1, r2
	bl	FlashEraseBlocks
	mov	r3, #1
	ldr	r0, .L1005+12
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldr	r3, [r5, #3452]
	cmn	r3, #1
	bne	.L988
	uxth	r0, r10
	bl	FtlBbmMapBadBlock
	b	.L989
.L990:
	ldr	r3, [fp, #-44]
	b	.L1004
.L994:
	uxth	r0, r0
	bl	FtlBbmMapBadBlock
	mov	r0, r7
	b	.L993
.L997:
	ldrh	r3, [r4]
	cmp	r3, r7
	strheq	r6, [r4]	@ movhi
	beq	.L996
.L998:
	strh	r6, [r4, #4]	@ movhi
.L999:
	ldrh	r3, [r4]
	mov	r6, #0
	ldr	r0, [r5, #3268]
	mov	r1, #1
	str	r6, [r5, #360]
	mov	r2, #2
	strh	r6, [r4, #2]	@ movhi
	lsl	r3, r3, #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, #4]
	lsl	r3, r3, #10
	str	r3, [r0, #24]
	bl	FlashEraseBlocks
	ldrh	r0, [r4]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, #4]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r5, #360]
	ldrh	r2, [r4, #4]
	strh	r6, [r4, #2]	@ movhi
	add	r3, r3, #1
	str	r3, [r5, #360]
	ldrh	r3, [r4]
	strh	r2, [r4]	@ movhi
	strh	r3, [r4, #4]	@ movhi
	bl	FtlBbmTblFlush
	mov	r0, r6
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1006:
	.align	2
.L1005:
	.word	.LANCHOR0+380
	.word	.LANCHOR0+314
	.word	.LANCHOR0+298
	.word	.LANCHOR0+3452
	.word	-3872
	.word	.LANCHOR0+3388
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	ftl_memcmp
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_memcmp, %function
ftl_memcmp:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	bl	memcmp
	ldmfd	sp, {fp, sp, pc}
	.size	ftl_memcmp, .-ftl_memcmp
	.align	2
	.global	js_hash
	.syntax unified
	.arm
	.fpu softvfp
	.type	js_hash, %function
js_hash:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L1011
	add	r1, r0, r1
.L1009:
	cmp	r0, r1
	bne	.L1010
	mov	r0, r3
	ldmfd	sp, {fp, sp, pc}
.L1010:
	lsr	r2, r3, #2
	ldrb	ip, [r0], #1	@ zero_extendqisi2
	add	r2, r2, r3, lsl #5
	add	r2, r2, ip
	eor	r3, r3, r2
	b	.L1009
.L1012:
	.align	2
.L1011:
	.word	1204201446
	.size	js_hash, .-js_hash
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.syntax unified
	.arm
	.fpu softvfp
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldrh	r3, [r0]
	movw	r2, #65535
	mov	r4, r0
	ldr	r5, [r0, #12]
	cmp	r3, r2
	bne	.L1014
	ldrh	r3, [r0, #8]
	cmp	r3, #0
	beq	.L1015
	movw	r2, #641
	ldr	r1, .L1023
	ldr	r0, .L1023+4
	bl	sftl_printk
.L1015:
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	mov	r3, #0
	strh	r0, [r5]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	strh	r3, [r4]	@ movhi
	ldr	r3, [r4, #28]
	add	r3, r3, #1
	str	r3, [r4, #28]
.L1016:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L1014:
	lsl	r3, r3, #1
	ldr	r7, [r0, #24]
	mov	r1, #255
	ldrh	r8, [r5, r3]
	ldrh	r3, [r0, #2]
	ldr	r5, .L1023+8
	orr	r3, r3, r8, lsl #10
	ldr	r6, [r5, #3332]
	str	r3, [r5, #3456]
	ldr	r3, [r5, #3300]
	str	r6, [r5, #3464]
	str	r3, [r5, #3460]
	ldr	r3, [r0, #28]
	str	r3, [r6, #4]
	ldr	r3, .L1023+12
	strh	r3, [r6, #8]	@ movhi
	ldrh	r3, [r0, #4]
	strh	r8, [r6, #2]	@ movhi
	strh	r3, [r6]	@ movhi
	add	r3, r5, #304
	ldrh	r2, [r3]
	ldr	r0, [r5, #3300]
	lsl	r2, r2, #3
	bl	ftl_memset
	mov	r2, #0
	mov	r3, r2
.L1017:
	ldrh	r0, [r4, #6]
	uxth	r1, r2
	cmp	r0, r1
	bhi	.L1019
	movw	r3, #310
	ldr	r0, [r5, #3460]
	ldrh	r1, [r5, r3]
	bl	js_hash
	mov	r2, #1
	str	r0, [r6, #12]
	mov	r3, #0
	mov	r1, r2
	ldr	r0, .L1023+16
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	add	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
	b	.L1016
.L1019:
	uxth	r1, r2
	ldr	r0, [r7, r1, lsl #2]
	cmp	r8, r0, lsr #10
	bne	.L1018
	ldr	r0, [r5, #3300]
	add	r3, r3, #1
	uxth	r3, r3
	str	r1, [r0, r3, lsl #3]
	ldr	r0, [r7, r1, lsl #2]
	ldr	r1, [r5, #3300]
	add	r1, r1, r3, lsl #3
	str	r0, [r1, #4]
.L1018:
	add	r2, r2, #1
	b	.L1017
.L1024:
	.align	2
.L1023:
	.word	.LANCHOR1+332
	.word	.LC8
	.word	.LANCHOR0
	.word	-1291
	.word	.LANCHOR0+3452
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	FtlMapWritePage
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	mov	r6, #0
	ldr	r5, .L1043
	mov	r4, r0
	mov	r7, r1
	mov	r9, r2
	add	r8, r5, #304
.L1026:
	ldr	r3, [r5, #2572]
	add	r3, r3, #1
	str	r3, [r5, #2572]
	ldrh	r3, [r8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L1027
	ldrh	r2, [r4]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1028
.L1027:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L1028:
	ldrh	r3, [r4]
	ldr	r2, [r4, #12]
	lsl	r3, r3, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L1029
	mov	r2, #700
	ldr	r1, .L1043+4
	ldr	r0, .L1043+8
	bl	sftl_printk
.L1029:
	ldrh	r2, [r4]
	ldrh	r3, [r4, #10]
	cmp	r2, r3
	bcc	.L1030
	movw	r2, #701
	ldr	r1, .L1043+4
	ldr	r0, .L1043+8
	bl	sftl_printk
.L1030:
	ldrh	r3, [r4]
	mov	r1, #16
	ldr	r2, [r4, #12]
	ldr	r0, [r5, #3332]
	lsl	r3, r3, #1
	ldrh	r2, [r2, r3]
	ldrh	r3, [r4, #2]
	str	r0, [r5, #3464]
	str	r2, [fp, #-44]
	str	r9, [r5, #3460]
	orr	r3, r3, r2, lsl #10
	str	r3, [r5, #3456]
	bl	__memzero
	ldr	r3, [r4, #28]
	ldr	r10, [r5, #3464]
	ldr	r2, [fp, #-44]
	str	r3, [r10, #4]
	strh	r7, [r10, #8]	@ movhi
	ldrh	r3, [r4, #4]
	strh	r2, [r10, #2]	@ movhi
	strh	r3, [r10]	@ movhi
	ldr	r3, .L1043+12
	ldr	r0, [r5, #3460]
	ldrh	r1, [r3]
	bl	js_hash
	mov	r3, #1
	str	r0, [r10, #12]
	mov	r2, r3
	mov	r1, r3
	ldr	r0, .L1043+16
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #2]	@ movhi
	ldr	r2, [r5, #3452]
	cmn	r2, #1
	bne	.L1031
	ldr	r1, [r5, #3456]
	add	r6, r6, #1
	ldr	r0, .L1043+20
	uxth	r6, r6
	bl	sftl_printk
	ldrh	r3, [r4, #2]
	cmp	r3, #2
	ldrhls	r3, [r8]
	subls	r3, r3, #1
	strhls	r3, [r4, #2]	@ movhi
	cmp	r6, #3
	bls	.L1033
	mov	r2, r6
	ldr	r1, [r5, #3456]
	ldr	r0, .L1043+24
	bl	sftl_printk
.L1034:
	b	.L1034
.L1033:
	ldr	r3, [r4, #32]
	cmp	r3, #0
	beq	.L1026
.L1042:
	b	.L1042
.L1031:
	cmp	r3, #1
	cmpne	r2, #256
	beq	.L1037
	ldr	r0, [r4, #36]
	cmp	r0, #0
	beq	.L1038
.L1037:
	mov	r3, #0
	str	r3, [r4, #36]
	b	.L1026
.L1038:
	ldr	r2, [r5, #3456]
	ldr	r3, [r4, #24]
	str	r2, [r3, r7, lsl #2]
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1044:
	.align	2
.L1043:
	.word	.LANCHOR0
	.word	.LANCHOR1+363
	.word	.LC8
	.word	.LANCHOR0+310
	.word	.LANCHOR0+3452
	.word	.LC104
	.word	.LC105
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	2
	.global	load_l2p_region
	.syntax unified
	.arm
	.fpu softvfp
	.type	load_l2p_region, %function
load_l2p_region:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r9, .L1052
	uxth	r6, r0
	uxth	r10, r1
	ldrh	r3, [r9]
	cmp	r3, r6
	bcs	.L1046
	movw	r2, #485
	ldr	r1, .L1052+4
	ldr	r0, .L1052+8
	bl	sftl_printk
.L1046:
	ldr	r7, .L1052+12
	mov	r5, #12
	ldr	r3, [r7, #3376]
	mov	r4, r7
	ldr	r8, [r3, r6, lsl #2]
	cmp	r8, #0
	bne	.L1047
	mul	r5, r5, r10
	ldr	r3, [r7, #2540]
	movw	r2, #310
	mov	r1, #255
	ldrh	r2, [r7, r2]
	add	r3, r3, r5
	ldr	r0, [r3, #8]
	bl	ftl_memset
	ldr	r3, [r7, #2540]
	strh	r6, [r3, r5]	@ movhi
	ldr	r3, [r7, #2540]
	add	r5, r3, r5
	str	r8, [r5, #4]
.L1048:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1047:
	mul	r5, r5, r10
	ldr	r3, [r7, #2540]
	mov	r2, #1
	ldr	r0, .L1052+16
	mov	r1, r2
	str	r8, [r7, #3456]
	add	r3, r3, r5
	ldr	r3, [r3, #8]
	str	r3, [r7, #3460]
	ldr	r3, [r7, #3332]
	str	r3, [r7, #3464]
	bl	FlashReadPages
	ldr	r7, [r7, #3464]
	ldrh	r3, [r7, #8]
	cmp	r3, r6
	beq	.L1049
	mov	r2, r8
	mov	r1, r6
	ldr	r0, .L1052+20
	bl	sftl_printk
	mov	r3, #4
	ldr	r1, [r4, #3464]
	mov	r2, r3
	ldr	r0, .L1052+24
	bl	rknand_print_hex
	ldrh	r3, [r9]
	mov	r2, #4
	ldr	r1, [r4, #3376]
	ldr	r0, .L1052+28
	bl	rknand_print_hex
.L1050:
	ldrh	r3, [r7, #8]
	cmp	r3, r6
	beq	.L1051
	mov	r2, #508
	ldr	r1, .L1052+4
	ldr	r0, .L1052+8
	bl	sftl_printk
.L1051:
	ldr	r3, [r4, #2540]
	mov	r1, #0
	add	r2, r3, r5
	str	r1, [r2, #4]
	strh	r6, [r3, r5]	@ movhi
	b	.L1048
.L1049:
	ldr	r3, [r4, #3452]
	cmp	r3, #256
	bne	.L1050
	mov	r2, r8
	mov	r1, r6
	ldr	r0, .L1052+32
	bl	sftl_printk
	ldr	r3, [r4, #2540]
	mov	r1, r6
	add	r0, r4, #3392
	add	r3, r3, r5
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
	b	.L1050
.L1053:
	.align	2
.L1052:
	.word	.LANCHOR0+336
	.word	.LANCHOR1+379
	.word	.LC8
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.word	.LC106
	.word	.LC94
	.word	.LC107
	.word	.LC108
	.size	load_l2p_region, .-load_l2p_region
	.align	2
	.global	ftl_map_blk_gc
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #8
	mov	r4, r0
	ldr	r5, [r0, #12]
	ldr	r9, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	sub	r3, r3, #5
	cmp	r2, r3
	blt	.L1055
	uxth	r0, r0
	lsl	r0, r0, #1
	ldrh	r8, [r5, r0]
	cmp	r8, #0
	beq	.L1055
	ldr	r3, [r4, #32]
	cmp	r3, #0
	bne	.L1055
	mov	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r5, r0]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	sub	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, .L1070
	ldrh	r3, [r3]
	cmp	r2, r3
	bcc	.L1056
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1056:
	ldr	r5, .L1070+4
	mov	r6, #0
.L1057:
	ldrh	r3, [r4, #6]
	uxth	r10, r6
	cmp	r3, r10
	bhi	.L1064
	mov	r1, #1
	mov	r0, r8
	bl	FtlFreeSysBlkQueueIn
	mov	r3, #0
	str	r3, [r4, #32]
.L1055:
	ldr	r3, .L1070
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3]
	cmp	r2, r3
	bcc	.L1065
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1065:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1064:
	uxth	r7, r6
	add	r3, r9, r7, lsl #2
	str	r3, [fp, #-44]
	ldr	r3, [r9, r7, lsl #2]
	cmp	r8, r3, lsr #10
	bne	.L1058
	ldr	r3, [r5, #3304]
	ldr	r0, .L1070+8
	str	r3, [r5, #3460]
	ldr	r3, [r5, #3332]
	str	r3, [r5, #3464]
	ldr	r2, [r9, r7, lsl #2]
	str	r3, [fp, #-48]
	str	r2, [r5, #3456]
	mov	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r3, [fp, #-48]
	ldrh	r2, [r3, #8]
	cmp	r2, r10
	beq	.L1059
	movw	r2, #611
	ldr	r1, .L1070+12
	ldr	r0, .L1070+16
	bl	sftl_printk
	ldr	r3, [fp, #-48]
.L1059:
	ldr	r2, [r5, #3452]
	cmn	r2, #1
	bne	.L1060
.L1062:
	ldr	r2, [fp, #-44]
	mov	r3, #0
	str	r3, [r2]
.L1061:
	b	.L1061
.L1060:
	ldrh	r2, [r3, #8]
	cmp	r2, r10
	bne	.L1062
	ldrh	r2, [r3]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L1062
	ldr	r2, [r5, #3460]
	mov	r1, r7
	mov	r0, r4
	bl	FtlMapWritePage
.L1058:
	add	r6, r6, #1
	b	.L1057
.L1071:
	.align	2
.L1070:
	.word	.LANCHOR0+304
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.word	.LANCHOR1+395
	.word	.LC8
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	flush_l2p_region
	.syntax unified
	.arm
	.fpu softvfp
	.type	flush_l2p_region, %function
flush_l2p_region:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r5, .L1073
	uxth	r0, r0
	mov	r4, #12
	mul	r4, r4, r0
	ldr	r3, [r5, #2540]
	add	r0, r5, #3392
	add	r2, r3, r4
	ldrh	r1, [r3, r4]
	ldr	r2, [r2, #8]
	bl	FtlMapWritePage
	ldr	r0, [r5, #2540]
	add	r4, r0, r4
	mov	r0, #0
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1074:
	.align	2
.L1073:
	.word	.LANCHOR0
	.size	flush_l2p_region, .-flush_l2p_region
	.align	2
	.global	log2phys
	.syntax unified
	.arm
	.fpu softvfp
	.type	log2phys, %function
log2phys:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #8
	mov	r6, r0
	ldr	r4, .L1088
	mov	r7, r1
	mov	r10, r2
	add	r3, r4, #308
	ldrh	r5, [r3]
	ldr	r3, [r4, #2552]
	cmp	r0, r3
	bcc	.L1076
	movw	r2, #813
	ldr	r1, .L1088+4
	ldr	r0, .L1088+8
	bl	sftl_printk
.L1076:
	ldr	r3, [r4, #2552]
	cmp	r6, r3
	bcs	.L1077
	add	r5, r5, #7
	ldr	ip, [r4, #2540]
	lsr	r3, r6, r5
	mov	r1, #0
	mov	r2, #12
	str	r3, [fp, #-44]
	movw	r3, #338
	ldrh	r9, [fp, #-44]
	ldrh	r0, [r4, r3]
.L1078:
	uxth	r8, r1
	cmp	r8, r0
	bcc	.L1083
	str	r2, [fp, #-48]
	bl	select_l2p_ram_region
	ldr	r2, [fp, #-48]
	mov	r8, r0
	ldr	r1, [r4, #2540]
	mul	r2, r2, r0
	add	ip, r1, r2
	ldrh	r1, [r1, r2]
	movw	r2, #65535
	cmp	r1, r2
	beq	.L1084
	ldr	r2, [ip, #4]
	cmp	r2, #0
	bge	.L1084
	bl	flush_l2p_region
.L1084:
	mov	r1, r8
	ldrh	r0, [fp, #-44]
	bl	load_l2p_region
	b	.L1080
.L1077:
	cmp	r10, #0
	mvn	r0, #0
	streq	r0, [r7]
.L1075:
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1083:
	add	r1, r1, #1
	mla	lr, r2, r1, ip
	ldrh	lr, [lr, #-12]
	cmp	lr, r9
	bne	.L1078
.L1080:
	mvn	r2, #0
	cmp	r10, #0
	bic	r5, r6, r2, lsl r5
	mov	r2, #12
	uxth	r5, r5
	bne	.L1081
	ldr	r1, [r4, #2540]
	mla	r2, r2, r8, r1
	ldr	r2, [r2, #8]
	ldr	r2, [r2, r5, lsl #2]
	str	r2, [r7]
.L1082:
	ldr	r1, [r4, #2540]
	mov	r2, #12
	mov	r0, #0
	mla	r3, r2, r8, r1
	ldr	r2, [r3, #4]
	cmn	r2, #1
	addne	r2, r2, #1
	strne	r2, [r3, #4]
	b	.L1075
.L1081:
	mul	r2, r2, r8
	ldr	r1, [r4, #2540]
	ldr	r0, [r7]
	add	r1, r1, r2
	ldr	r1, [r1, #8]
	str	r0, [r1, r5, lsl #2]
	ldr	r1, [r4, #2540]
	add	r2, r1, r2
	ldr	r1, [r2, #4]
	orr	r1, r1, #-2147483648
	str	r1, [r2, #4]
	ldr	r2, .L1088+12
	strh	r9, [r2]	@ movhi
	b	.L1082
.L1089:
	.align	2
.L1088:
	.word	.LANCHOR0
	.word	.LANCHOR1+410
	.word	.LC8
	.word	.LANCHOR0+2544
	.size	log2phys, .-log2phys
	.align	2
	.global	FtlWriteDump_data
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlWriteDump_data, %function
FtlWriteDump_data:
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #24
	ldr	r4, .L1108
	ldrh	r2, [r4, #28]
	cmp	r2, #0
	beq	.L1091
	ldrb	r3, [r4, #32]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1091
	movw	r3, #302
	ldrb	r1, [r4, #31]	@ zero_extendqisi2
	ldrh	r3, [r4, r3]
	mul	r3, r3, r1
	cmp	r2, r3
	beq	.L1091
	ldrb	r7, [r4, #34]	@ zero_extendqisi2
	cmp	r7, #0
	bne	.L1090
	ldr	r6, [r4, #2552]
	mov	r2, r7
	sub	r1, fp, #64
	ldrh	r8, [r4, #232]
	sub	r6, r6, #1
	mov	r0, r6
	bl	log2phys
	ldr	r3, [fp, #-64]
	ldr	r5, [r4, #3332]
	ldr	r0, [r4, #3300]
	cmn	r3, #1
	str	r3, [fp, #-56]
	str	r6, [fp, #-44]
	str	r0, [fp, #-52]
	str	r5, [fp, #-48]
	str	r7, [r5, #4]
	beq	.L1093
	mov	r2, r7
	mov	r1, #1
	sub	r0, fp, #60
	bl	FlashReadPages
.L1094:
	ldr	r10, .L1108+4
	mov	r7, #0
	ldr	r3, .L1108+8
	lsl	r8, r8, #2
	mov	r9, r7
	strh	r3, [r5]	@ movhi
.L1095:
	cmp	r8, r7
	bne	.L1099
.L1096:
	mov	r3, #1
.L1107:
	strb	r3, [r4, #34]
.L1090:
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1093:
	movw	r3, #310
	mov	r1, #255
	ldrh	r2, [r4, r3]
	bl	ftl_memset
	b	.L1094
.L1099:
	ldrh	r3, [r4, #28]
	cmp	r3, #0
	beq	.L1096
	ldr	r3, [fp, #-56]
	mov	r0, r10
	str	r6, [r5, #8]
	add	r7, r7, #1
	str	r3, [r5, #12]
	ldrh	r3, [r4, #24]
	strh	r3, [r5, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r4, #2592]
	mov	r1, #1
	str	r0, [fp, #-56]
	sub	r0, fp, #60
	str	r3, [r5, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, r9
	str	r3, [r4, #2592]
	mov	r3, #0
	mov	r2, r3
	bl	FlashProgPages
	ldrh	r0, [r4, #24]
	bl	decrement_vpc_count
	b	.L1095
.L1091:
	mov	r3, #0
	b	.L1107
.L1109:
	.align	2
.L1108:
	.word	.LANCHOR0
	.word	.LANCHOR0+24
	.word	-3947
	.size	FtlWriteDump_data, .-FtlWriteDump_data
	.align	2
	.global	FtlReUsePrevPpa
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	mov	r6, r0
	ldr	r7, .L1120
	ubfx	r0, r1, #10, #16
	str	r1, [fp, #-32]
	bl	P2V_block_in_plane
	ldr	r2, [r7, #72]
	lsl	r5, r0, #1
	ldrh	r3, [r2, r5]
	cmp	r3, #0
	bne	.L1111
	ldr	r4, [r7, #2532]
	cmp	r4, #0
	beq	.L1112
	ldr	r2, [r7, #2516]
	movw	lr, #65535
	ldrh	ip, [r7, #224]
	sub	r4, r4, r2
	ubfx	r4, r4, #3, #16
.L1113:
	uxth	r1, r3
	cmp	ip, r1
	bls	.L1112
	cmp	r4, r0
	bne	.L1114
	mov	r1, r4
	ldr	r0, .L1120+4
	bl	List_remove_node
	ldrh	r3, [r7, #224]
	cmp	r3, #0
	bne	.L1115
	movw	r2, #1735
	ldr	r1, .L1120+8
	ldr	r0, .L1120+12
	bl	sftl_printk
.L1115:
	ldrh	r3, [r7, #224]
	mov	r0, r4
	sub	r3, r3, #1
	strh	r3, [r7, #224]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r2, [r7, #72]
	ldrh	r3, [r2, r5]
.L1111:
	add	r3, r3, #1
	strh	r3, [r2, r5]	@ movhi
	b	.L1112
.L1114:
	lsl	r4, r4, #3
	add	r3, r3, #1
	ldrh	r4, [r2, r4]
	cmp	r4, lr
	bne	.L1113
.L1112:
	mov	r2, #1
	sub	r1, fp, #32
	mov	r0, r6
	bl	log2phys
	ldmib	sp, {r4, r5, r6, r7, fp, sp, pc}
.L1121:
	.align	2
.L1120:
	.word	.LANCHOR0
	.word	.LANCHOR0+2532
	.word	.LANCHOR1+419
	.word	.LC8
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	FtlRecoverySuperblock
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	@ args = 0, pretend = 0, frame = 44
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #44
	movw	r2, #65535
	ldrh	r3, [r0]
	mov	r10, r0
	cmp	r3, r2
	beq	.L1246
	ldrh	r3, [r0, #2]
	ldr	r5, .L1254
	str	r3, [fp, #-64]
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	ldr	r1, [fp, #-64]
	str	r3, [fp, #-80]
	movw	r3, #302
	ldrh	r3, [r5, r3]
	cmp	r3, r1
	mov	r3, #0
	strheq	r3, [r0, #4]	@ movhi
	ldrhne	r0, [r0, #16]
	bne	.L1126
.L1252:
	strb	r3, [r10, #6]
.L1246:
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1127:
	uxth	r1, r3
	add	r1, r10, r1, lsl #1
	ldrh	r0, [r1, #16]
.L1126:
	cmp	r0, r2
	uxth	r6, r3
	add	r3, r3, #1
	beq	.L1127
	mov	r1, #1
	bl	FtlGetLastWrittenPage
	cmn	r0, #1
	mov	r4, r0
	beq	.L1128
	ldrh	r3, [r5, #232]
	mov	r2, #0
	ldr	r0, [r5, #3288]
	movw	r7, #65535
	ldr	lr, [r5, #3180]
	mov	r8, #20
	str	r3, [fp, #-56]
	ldr	r3, [r5, #3176]
	str	r3, [fp, #-60]
	movw	r3, #310
	ldrh	r3, [r5, r3]
	mov	r5, r2
	str	r3, [fp, #-68]
	ldr	r3, .L1254+4
	ldrh	r6, [r3]
	add	r3, r10, #16
	mov	ip, r3
	str	r3, [fp, #-76]
.L1129:
	ldr	r1, [fp, #-56]
	uxth	r3, r2
	cmp	r1, r3
	bhi	.L1133
	ldr	r6, .L1254
	mov	r2, #0
	mov	r1, r5
	movw	r8, #65535
	bl	FlashReadPages
	ldr	r3, [r6, #2592]
	uxth	r2, r4
	ldr	r9, [r6, #3288]
	str	r2, [fp, #-60]
	sub	r3, r3, #1
	str	r3, [fp, #-56]
	mov	r7, r9
	mov	r3, #0
.L1134:
	uxth	r2, r3
	cmp	r5, r2
	bhi	.L1139
	addeq	r3, r4, #1
	uxtheq	r3, r3
	streq	r3, [fp, #-60]
	bne	.L1137
.L1247:
	ldr	r0, [r9, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	movw	r3, #302
	ldr	r2, [fp, #-60]
	ldrh	r3, [r6, r3]
	sub	r1, fp, #68
	str	r0, [fp, #-68]
	ldr	r0, [fp, #-80]
	cmp	r3, r2
	ldrheq	r3, [fp, #-60]
	strheq	r3, [r10, #2]	@ movhi
	moveq	r3, #0
	strbeq	r3, [r10, #6]
	strheq	r3, [r10, #4]	@ movhi
	ldm	r1, {r1, r2, r3}
	cmp	r3, r2
	cmpeq	r1, r0
	moveq	r2, r1
	moveq	r1, r3
	beq	.L1253
	movw	r3, #65535
	cmp	r8, r3
	bne	.L1143
	ldrb	r3, [r10, #8]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1144
.L1143:
	ldr	r3, [r6, #3448]
	uxth	r7, r4
	uxth	r4, r4
	cmn	r3, #1
	ldreq	r3, [fp, #-56]
	streq	r3, [r6, #3448]
	ldr	r3, [fp, #-64]
	ldr	r6, .L1254
	add	r3, r3, #7
	cmp	r4, r3
	subgt	r4, r7, #7
	ldrle	r4, [fp, #-64]
	uxthgt	r4, r4
.L1147:
	cmp	r4, r7
	bhi	.L1154
	mov	r3, #0
	ldrh	r8, [r6, #232]
	ldr	r0, [r6, #3288]
	mov	r5, r3
	ldr	r1, [fp, #-76]
	movw	lr, #65535
	mov	ip, #20
	b	.L1155
.L1128:
	ldr	r3, [fp, #-64]
	cmp	r3, #0
	beq	.L1130
	movw	r2, #1802
	ldr	r1, .L1254+8
	ldr	r0, .L1254+12
	bl	sftl_printk
.L1130:
	ldr	r3, [fp, #-80]
	cmp	r3, #0
	cmpne	r6, r3
	beq	.L1131
	movw	r2, #1803
	ldr	r1, .L1254+8
	ldr	r0, .L1254+12
	bl	sftl_printk
.L1131:
	mov	r3, #0
	strh	r3, [r10, #2]	@ movhi
	b	.L1252
.L1133:
	ldrh	r3, [ip], #2
	cmp	r3, r7
	beq	.L1132
	mla	r1, r8, r5, r0
	orr	r3, r4, r3, lsl #10
	str	r3, [r1, #4]
	ldr	r3, [fp, #-68]
	mul	r3, r3, r5
	add	r9, r3, #3
	cmp	r3, #0
	movlt	r3, r9
	ldr	r9, [fp, #-60]
	bic	r3, r3, #3
	add	r3, r9, r3
	str	r3, [r1, #8]
	mul	r3, r6, r5
	add	r5, r5, #1
	uxth	r5, r5
	add	r9, r3, #3
	cmp	r3, #0
	movlt	r3, r9
	bic	r3, r3, #3
	add	r3, lr, r3
	str	r3, [r1, #12]
.L1132:
	add	r2, r2, #1
	b	.L1129
.L1139:
	ldr	r2, [r7]
	cmp	r2, #0
	bne	.L1135
	ldr	ip, [r7, #12]
	ldr	r2, [ip, #4]
	cmn	r2, #1
	beq	.L1136
	ldr	r1, .L1254
	mov	r0, r2
	str	ip, [fp, #-68]
	ldr	r1, [r1, #2592]
	bl	ftl_cmp_data_ver
	ldr	ip, [fp, #-68]
	cmp	r0, #0
	ldrne	r1, .L1254
	addne	r2, r2, #1
	strne	r2, [r1, #2592]
.L1136:
	ldr	r2, [ip]
	cmn	r2, #1
	bne	.L1138
.L1137:
	uxth	r2, r4
	uxth	r3, r3
	str	r2, [fp, #-60]
	mov	r2, #20
	mla	r9, r2, r3, r9
	b	.L1247
.L1135:
	ldr	r8, [fp, #-60]
.L1138:
	add	r3, r3, #1
	add	r7, r7, #20
	b	.L1134
.L1149:
	ldrh	r2, [r1], #2
	add	r3, r3, #1
	cmp	r2, lr
	mlane	r9, ip, r5, r0
	addne	r5, r5, #1
	orrne	r2, r4, r2, lsl #10
	uxthne	r5, r5
	strne	r2, [r9, #4]
.L1155:
	uxth	r2, r3
	cmp	r8, r2
	bhi	.L1149
	mov	r1, r5
	mov	r2, #0
	bl	FlashReadPages
	ldr	r3, [r6, #3288]
	mov	r2, #20
	movw	r1, #65535
	mla	r5, r2, r5, r3
.L1150:
	cmp	r5, r3
	addeq	r4, r4, #1
	uxtheq	r4, r4
	beq	.L1147
.L1153:
	ldr	r2, [r3]
	cmp	r2, #0
	bne	.L1144
	ldr	r2, [r3, #12]
	ldrh	r0, [r2]
	cmp	r0, r1
	beq	.L1152
	ldr	r2, [r2, #4]
	cmn	r2, #1
	strne	r2, [r6, #3448]
.L1152:
	add	r3, r3, #20
	b	.L1150
.L1154:
	mvn	r3, #0
	str	r3, [r6, #3448]
.L1144:
	ldr	r4, .L1254+16
	mov	r3, #1
	ldr	r8, [fp, #-64]
	strh	r3, [r4]	@ movhi
	sub	r4, r4, #3472
.L1156:
	ldrh	ip, [r4, #232]
	mov	r3, #0
	ldr	r0, [r4, #3288]
	mov	r9, r3
	ldr	r1, [fp, #-76]
	movw	lr, #65535
	mov	r5, #20
.L1157:
	uxth	r2, r3
	cmp	ip, r2
	bhi	.L1159
	mov	r2, #0
	mov	r1, r9
	bl	FlashReadPages
	mov	r3, #0
.L1251:
	str	r3, [fp, #-72]
	ldrh	r3, [fp, #-72]
	cmp	r9, r3
	bhi	.L1187
	ldr	r3, .L1254+20
	add	r8, r8, #1
	uxth	r8, r8
	ldrh	r3, [r3]
	cmp	r3, r8
	bne	.L1156
	ldrh	r2, [r4, #232]
	movw	r0, #65535
	mov	r3, #0
	strh	r8, [r10, #2]	@ movhi
	strh	r3, [r10, #4]	@ movhi
.L1188:
	uxth	r1, r3
	cmp	r1, r2
	bcs	.L1246
	ldr	r1, [fp, #-76]
	ldrh	ip, [r1], #2
	cmp	ip, r0
	str	r1, [fp, #-76]
	add	r1, r3, #1
	bne	.L1252
	mov	r3, r1
	b	.L1188
.L1159:
	ldrh	r2, [r1], #2
	add	r3, r3, #1
	cmp	r2, lr
	mlane	r6, r5, r9, r0
	orrne	r2, r8, r2, lsl #10
	addne	r9, r9, #1
	uxthne	r9, r9
	strne	r2, [r6, #4]
	b	.L1157
.L1187:
	ldr	r3, [fp, #-72]
	mov	r6, #20
	ldr	r2, [r4, #3288]
	mul	r6, r6, r3
	str	r2, [fp, #-84]
	add	r7, r2, r6
	ldr	r5, [r7, #4]
	ubfx	r0, r5, #10, #16
	str	r5, [fp, #-44]
	bl	P2V_plane
	ldr	r3, [fp, #-64]
	cmp	r8, r3
	bcc	.L1161
	ldr	r2, [fp, #-80]
	moveq	r3, #1
	movne	r3, #0
	cmp	r2, r0
	movls	r3, #0
	andhi	r3, r3, #1
	cmp	r3, #0
	bne	.L1161
	ldr	r3, [fp, #-60]
	ldr	r2, [fp, #-68]
	cmp	r8, r3
	cmpeq	r2, r0
	beq	.L1162
	ldr	r2, [fp, #-84]
	ldr	r3, [r2, r6]
	cmn	r3, #1
	beq	.L1163
	ldr	r7, [r7, #12]
	movw	r3, #61589
	ldrh	r2, [r7]
	cmp	r2, r3
	ldrhne	r0, [r10]
	bne	.L1248
	ldr	r3, [r7, #4]
	cmn	r3, #1
	str	r3, [fp, #-56]
	beq	.L1165
	mov	r0, r3
	ldr	r1, [r4, #2592]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	ldrne	r3, [fp, #-56]
	addne	r3, r3, #1
	strne	r3, [r4, #2592]
.L1165:
	ldrh	r2, [r7]
	movw	r3, #61589
	cmp	r2, r3
	beq	.L1166
	movw	r2, #1954
	ldr	r1, .L1254+8
	ldr	r0, .L1254+12
	bl	sftl_printk
.L1166:
	ldr	r5, [r7, #8]
	sub	r1, fp, #48
	ldr	r3, [r7, #12]
	mov	r2, #0
	mov	r0, r5
	str	r3, [fp, #-52]
	bl	log2phys
	ldr	r1, [r4, #3448]
	cmn	r1, #1
	beq	.L1167
	ldr	r0, [fp, #-56]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1167
	ldr	r2, [fp, #-52]
	cmn	r2, #1
	beq	.L1168
	ldr	r0, [r4, #3288]
	mov	r1, #1
	add	r0, r0, r6
	str	r2, [r0, #4]
	mov	r2, #0
	ldr	r7, [r0, #12]
	bl	FlashReadPages
	ldr	r2, [r4, #3288]
	ldr	r1, [r2, r6]
	add	r3, r2, r6
	cmn	r1, #1
	bne	.L1169
.L1170:
	mvn	r3, #0
	str	r3, [fp, #-52]
.L1177:
	ldr	r6, [fp, #-52]
	cmn	r6, #1
	beq	.L1161
.L1191:
	ubfx	r0, r6, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #240]
	mov	r5, r0
	cmp	r3, r0
	bhi	.L1183
	movw	r2, #2059
	ldr	r1, .L1254+8
	ldr	r0, .L1254+12
	bl	sftl_printk
.L1183:
	ldr	r2, [r4, #72]
	lsl	r3, r5, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1184
	mov	r0, r5
.L1248:
	bl	decrement_vpc_count
	b	.L1161
.L1168:
	ldr	r3, [fp, #-44]
	ldr	r2, [fp, #-48]
	cmp	r2, r3
	bne	.L1161
	mov	r2, #1
	sub	r1, fp, #52
	mov	r0, r5
	bl	log2phys
.L1161:
	ldr	r3, [fp, #-72]
	add	r3, r3, #1
	b	.L1251
.L1169:
	ldr	r1, [r7, #8]
	cmp	r5, r1
	bne	.L1170
	ldr	r1, [r7, #4]
	ldr	r0, [r4, #3448]
	str	r1, [fp, #-84]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1170
	ldr	r1, [fp, #-48]
	ldr	r0, [fp, #-44]
	cmp	r1, r0
	bne	.L1172
.L1175:
	ldr	r1, [fp, #-52]
	mov	r0, r5
	bl	FtlReUsePrevPpa
	b	.L1170
.L1172:
	ldr	r0, [fp, #-52]
	cmp	r1, r0
	beq	.L1170
	cmn	r1, #1
	streq	r1, [r2, r6]
	beq	.L1174
	str	r1, [r3, #4]
	mov	r2, #0
	mov	r1, #1
	mov	r0, r3
	ldr	r7, [r3, #12]
	bl	FlashReadPages
.L1174:
	ldr	r2, [r4, #3288]
	ldr	r2, [r2, r6]
	cmn	r2, #1
	beq	.L1175
	ldr	r3, [r7, #4]
	ldr	r0, [r4, #3448]
	mov	r1, r3
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1175
	mov	r1, r3
	ldr	r0, [fp, #-84]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1170
	b	.L1175
.L1167:
	ldr	r3, [fp, #-44]
	ldr	r2, [fp, #-48]
	cmp	r2, r3
	beq	.L1177
	ldr	r1, [fp, #-52]
	cmn	r1, #1
	beq	.L1179
	ldr	r3, [r4, #248]
	cmp	r3, r1, lsr #10
	ldrls	r0, .L1254+24
	bls	.L1250
.L1179:
	mov	r2, #1
	sub	r1, fp, #44
	mov	r0, r5
	bl	log2phys
	ldr	r6, [fp, #-48]
	cmn	r6, #1
	beq	.L1177
	ldr	r3, [fp, #-52]
	cmp	r6, r3
	beq	.L1191
	ubfx	r0, r6, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #24]
	cmp	r3, r0
	beq	.L1182
	ldrh	r3, [r4, #76]
	cmp	r3, r0
	beq	.L1182
	ldrh	r3, [r4, #124]
	cmp	r3, r0
	bne	.L1177
.L1182:
	ldr	r0, [r4, #3288]
	mov	r2, #0
	mov	r1, #1
	str	r6, [r0, #4]
	ldr	r7, [r0, #12]
	bl	FlashReadPages
	ldr	r3, [r4, #3288]
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L1177
	ldr	r1, [r7, #4]
	ldr	r0, [fp, #-56]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L1177
	mov	r2, #1
	sub	r1, fp, #48
	mov	r0, r5
	bl	log2phys
	b	.L1177
.L1184:
	ldr	r0, .L1254+28
	mov	r1, r5
.L1250:
	bl	sftl_printk
	b	.L1161
.L1163:
	ldr	r3, [r4, #3476]
	cmp	r3, #31
	addls	r2, r4, r3, lsl #2
	addls	r3, r3, #1
	strls	r3, [r4, #3476]
	strls	r5, [r2, #3480]
	ldrh	r0, [r10]
	bl	decrement_vpc_count
	ldr	r3, [r4, #3448]
	cmn	r3, #1
	ldreq	r3, [fp, #-56]
	beq	.L1249
	ldr	r2, [fp, #-56]
	cmp	r2, r3
	bcs	.L1161
	mov	r3, r2
.L1249:
	str	r3, [r4, #3448]
	b	.L1161
.L1162:
	ldrb	r3, [fp, #-68]	@ zero_extendqisi2
	ldr	r2, [fp, #-68]
	ldr	r1, [fp, #-60]
	strb	r3, [r10, #6]
	ldrh	r3, [fp, #-60]
	strh	r3, [r10, #2]	@ movhi
.L1253:
	mov	r0, r10
	bl	ftl_sb_update_avl_pages
	b	.L1246
.L1255:
	.align	2
.L1254:
	.word	.LANCHOR0
	.word	.LANCHOR0+312
	.word	.LANCHOR1+435
	.word	.LC8
	.word	.LANCHOR0+3472
	.word	.LANCHOR0+302
	.word	.LC109
	.word	.LC110
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	2
	.global	ftl_check_vpc
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_check_vpc, %function
ftl_check_vpc:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	mov	r4, #0
	ldr	r6, .L1280
	ldr	r1, .L1280+4
	ldr	r0, .L1280+8
	mov	r5, r6
	bl	sftl_printk
	mov	r1, #8192
	ldr	r0, .L1280+12
	bl	__memzero
.L1257:
	ldr	r3, [r6, #2552]
	cmp	r4, r3
	bcc	.L1259
	ldr	r8, .L1280+12
	mov	r4, #0
	ldr	r9, .L1280+16
	mov	r6, r4
.L1260:
	ldrh	r2, [r5, #240]
	uxth	r3, r4
	cmp	r2, r3
	bhi	.L1262
	ldr	r4, [r5, #2532]
	cmp	r4, #0
	beq	.L1263
	ldr	r3, [r5, #2516]
	mov	r7, #0
	ldrh	r8, [r5, #224]
	ldr	r9, .L1280+12
	ldr	r10, .L1280+20
	sub	r4, r4, r3
	ubfx	r4, r4, #3, #16
.L1264:
	uxth	r3, r7
	cmp	r8, r3
	bls	.L1263
	ldr	r2, [r5, #72]
	lsl	r3, r4, #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	beq	.L1265
	mov	r6, #1
	ldrh	r3, [r9, r3]
	mov	r1, r4
	mov	r0, r10
	bl	sftl_printk
.L1265:
	ldr	r3, [r5, #2516]
	lsl	r4, r4, #3
	add	r7, r7, #1
	ldrh	r4, [r3, r4]
	movw	r3, #65535
	cmp	r4, r3
	bne	.L1264
.L1263:
	cmp	r6, #0
	ldmibeq	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
	movw	r2, #2387
	ldr	r1, .L1280+4
	ldr	r0, .L1280+24
	bl	sftl_printk
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1259:
	mov	r2, #0
	sub	r1, fp, #44
	mov	r0, r4
	bl	log2phys
	ldr	r0, [fp, #-44]
	cmn	r0, #1
	beq	.L1258
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, .L1280+12
	lsl	r0, r0, #1
	ldrh	r3, [r2, r0]
	add	r3, r3, #1
	strh	r3, [r2, r0]	@ movhi
.L1258:
	add	r4, r4, #1
	b	.L1257
.L1262:
	uxth	r1, r4
	ldr	r3, [r5, #72]
	lsl	r7, r1, #1
	ldrh	r2, [r3, r7]
	ldrh	r3, [r8, r7]
	cmp	r2, r3
	beq	.L1261
	mov	r0, r9
	bl	sftl_printk
	ldr	r3, [r5, #72]
	movw	r2, #65535
	ldrh	r3, [r3, r7]
	cmp	r3, r2
	beq	.L1261
	ldrh	r2, [r8, r7]
	cmp	r2, r3
	movhi	r6, #1
.L1261:
	add	r4, r4, #1
	b	.L1260
.L1281:
	.align	2
.L1280:
	.word	.LANCHOR0
	.word	.LANCHOR1+457
	.word	.LC111
	.word	check_vpc_table
	.word	.LC112
	.word	.LC113
	.word	.LC8
	.size	ftl_check_vpc, .-ftl_check_vpc
	.align	2
	.global	ftl_scan_all_data
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_scan_all_data, %function
ftl_scan_all_data:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #24
	mov	r5, #0
	ldr	r6, .L1290
	mov	r1, #0
	ldr	r0, .L1290+4
	bl	sftl_printk
	mov	r4, r6
.L1283:
	ldr	r3, [r6, #2552]
	cmp	r5, r3
	bcc	.L1289
	sub	sp, fp, #28
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L1289:
	mov	r2, #0
	sub	r1, fp, #32
	mov	r0, r5
	bl	log2phys
	ubfx	r3, r5, #0, #11
	cmp	r3, #0
	bne	.L1284
	ldr	r2, [fp, #-32]
	mov	r1, r5
	ldr	r0, .L1290+8
	bl	sftl_printk
.L1284:
	ldr	r3, [fp, #-32]
	cmn	r3, #1
	beq	.L1286
	str	r3, [r4, #3456]
	mov	r2, #0
	ldr	r3, [r4, #3300]
	mov	r1, #1
	ldr	r7, [r4, #3332]
	ldr	r0, .L1290+12
	str	r3, [r4, #3460]
	str	r5, [r4, #3468]
	str	r7, [r4, #3464]
	str	r2, [r4, #3452]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	cmn	r3, #1
	cmpne	r3, #256
	beq	.L1287
	ldr	r3, [r7, #8]
	cmp	r5, r3
	beq	.L1286
.L1287:
	ldr	r2, [r4, #3460]
	ldr	r3, [r4, #3464]
	ldr	r0, .L1290+16
	ldr	r1, [r2, #4]
	str	r1, [sp, #16]
	mov	r1, r5
	ldr	r2, [r2]
	str	r2, [sp, #12]
	ldr	r2, [r3, #12]
	str	r2, [sp, #8]
	ldr	r2, [r3, #8]
	str	r2, [sp, #4]
	ldr	r2, [r3, #4]
	str	r2, [sp]
	ldr	r3, [r3]
	ldr	r2, [r4, #3456]
	bl	sftl_printk
.L1286:
	add	r5, r5, #1
	b	.L1283
.L1291:
	.align	2
.L1290:
	.word	.LANCHOR0
	.word	.LC114
	.word	.LC115
	.word	.LANCHOR0+3452
	.word	.LC116
	.size	ftl_scan_all_data, .-ftl_scan_all_data
	.align	2
	.global	FtlGcScanTempBlk
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #48
	mov	r4, r0
	ldr	r3, .L1324
	str	r1, [fp, #-76]
	ldrh	r5, [r3, #4]
	movw	r3, #65535
	cmp	r5, r3
	beq	.L1317
	cmp	r5, #0
	bne	.L1293
.L1294:
	bl	FtlGcPageVarInit
	b	.L1295
.L1317:
	mov	r5, #0
.L1293:
	ldr	r2, .L1324+4
	movw	r3, #302
	ldrh	r3, [r2, r3]
	ldr	r2, [fp, #-76]
	cmp	r3, r2
	beq	.L1294
.L1295:
	ldr	r6, .L1324+4
	mov	r3, #0
	str	r3, [fp, #-68]
.L1311:
	ldrh	r2, [r4]
	mov	r3, #0
	strb	r3, [r4, #8]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1296
.L1314:
	ldrh	r3, [r6, #232]
	add	ip, r4, #16
	ldr	r0, [r6, #3288]
	movw	r9, #65535
	mov	lr, #20
	str	r3, [fp, #-72]
	ldr	r3, [r6, #3176]
	str	r3, [fp, #-80]
	ldr	r3, .L1324+8
	ldrh	r2, [r3]
	ldrh	r10, [r3, #2]
	str	r2, [fp, #-84]
	ldr	r2, [r6, #3180]
	str	r2, [fp, #-88]
	mov	r2, #0
	mov	r7, r2
.L1297:
	ldr	r1, [fp, #-72]
	uxth	r3, r2
	cmp	r1, r3
	bhi	.L1299
	mov	r8, #0
	mov	r2, #0
	mov	r1, r7
	bl	FlashReadPages
.L1300:
	uxth	r3, r8
	cmp	r7, r3
	bhi	.L1312
	ldr	r3, [fp, #-68]
	add	r5, r5, #1
	uxth	r5, r5
	add	r3, r3, #1
	str	r3, [fp, #-68]
	ldr	r2, [fp, #-68]
	ldr	r3, [fp, #-76]
	cmp	r3, r2
	ldr	r2, .L1324+12
	bls	.L1313
.L1315:
	ldrh	r3, [r2]
	cmp	r3, r5
	bhi	.L1314
.L1296:
	ldr	r3, .L1324
	mvn	r2, #0
	strh	r5, [r4, #2]	@ movhi
	mov	r1, r5
	mov	r0, r4
	strh	r2, [r3, #4]	@ movhi
	mov	r2, #0
	strb	r2, [r4, #6]
	bl	ftl_sb_update_avl_pages
	b	.L1316
.L1299:
	ldrh	r3, [ip], #2
	cmp	r3, r9
	beq	.L1298
	mla	r1, lr, r7, r0
	orr	r3, r5, r3, lsl #10
	str	r3, [r1, #4]
	ldr	r3, [fp, #-84]
	mul	r3, r3, r7
	add	r8, r3, #3
	cmp	r3, #0
	movlt	r3, r8
	ldr	r8, [fp, #-80]
	bic	r3, r3, #3
	add	r3, r8, r3
	str	r3, [r1, #8]
	mul	r3, r10, r7
	add	r7, r7, #1
	uxth	r7, r7
	add	r8, r3, #3
	cmp	r3, #0
	movlt	r3, r8
	ldr	r8, [fp, #-88]
	bic	r3, r3, #3
	add	r3, r8, r3
	str	r3, [r1, #12]
.L1298:
	add	r2, r2, #1
	b	.L1297
.L1312:
	mov	ip, #20
	ldr	r2, [r6, #3288]
	mul	ip, ip, r8
	ldr	r10, [r2, ip]
	add	r3, r2, ip
	ldr	r1, [r3, #4]
	ldr	r9, [r3, #12]
	cmp	r10, #0
	str	r1, [fp, #-72]
	bne	.L1301
	ldrh	r1, [r9]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L1302
.L1322:
	ldrh	r3, [r4]
	mov	r1, #0
	ldr	r2, [r6, #72]
	mov	r5, #0
	lsl	r3, r3, #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r4]
	bl	INSERT_FREE_LIST
	mvn	r3, #0
	strh	r3, [r4]	@ movhi
	strh	r3, [r6, #172]	@ movhi
	bl	FtlGcPageVarInit
	b	.L1311
.L1302:
	ldr	r0, [r9, #8]
	ldr	r2, [r6, #2552]
	str	ip, [fp, #-80]
	cmp	r0, r2
	bhi	.L1322
	mov	r2, r10
	sub	r1, fp, #64
	bl	log2phys
	ldr	r1, [fp, #-64]
	ldr	r2, [r9, #12]
	ldr	ip, [fp, #-80]
	cmn	r1, #1
	sub	r0, r2, r1
	clz	r0, r0
	lsr	r0, r0, #5
	moveq	r0, #0
	cmp	r0, #0
	bne	.L1305
.L1310:
	ldr	r2, [r9, #8]
.L1323:
	ldr	r1, [fp, #-72]
	add	r8, r8, #1
	ldr	r0, [r9, #12]
	bl	FtlGcUpdatePage
	b	.L1300
.L1305:
	str	r2, [fp, #-56]
	mov	r1, #1
	ldr	r2, [r6, #3316]
	sub	r0, fp, #60
	str	ip, [fp, #-80]
	str	r2, [fp, #-52]
	ldr	r2, [r6, #3336]
	str	r2, [fp, #-48]
	mov	r2, r10
	bl	FlashReadPages
	ldr	r2, .L1324+16
	ldr	ip, [fp, #-80]
	ldr	r0, [fp, #-52]
	ldrh	r1, [r2]
	ldr	r2, [r6, #3288]
	lsl	r1, r1, #7
	add	ip, r2, ip
	mov	r2, r10
.L1307:
	cmp	r2, r1
	beq	.L1310
	ldr	lr, [ip, #8]
	ldr	r10, [lr, r2, lsl #2]
	ldr	lr, [r0, r2, lsl #2]
	cmp	r10, lr
	beq	.L1308
	ldr	r2, [fp, #-56]
	ldrh	r1, [r4]
	ldr	r0, .L1324+20
	bl	sftl_printk
	b	.L1322
.L1308:
	add	r2, r2, #1
	b	.L1307
.L1301:
	mvn	r2, #0
	b	.L1323
.L1313:
	ldr	r1, .L1324
	movw	r0, #65535
	ldrh	r3, [r1, #4]
	cmp	r3, r0
	beq	.L1315
	ldr	r0, [fp, #-68]
	add	r3, r3, r0
	strh	r3, [r1, #4]	@ movhi
	ldrh	r3, [r2]
	cmp	r3, r5
	bls	.L1315
.L1316:
	mvn	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1325:
	.align	2
.L1324:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR0+310
	.word	.LANCHOR0+302
	.word	.LANCHOR0+258
	.word	.LC117
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	FtlReadRefresh
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #88
	ldr	r6, .L1342
	ldr	r9, [r6, #2720]
	mov	r5, r6
	cmp	r9, #0
	beq	.L1327
	ldr	r2, [r6, #2724]
	ldr	r3, [r6, #2552]
	cmp	r2, r3
	bcs	.L1328
	mov	r4, #2048
.L1333:
	ldr	r0, [r5, #2724]
	ldr	r3, [r5, #2552]
	cmp	r0, r3
	bcc	.L1329
.L1332:
	mvn	r0, #0
.L1326:
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1329:
	mov	r2, #0
	sub	r1, fp, #128
	bl	log2phys
	ldr	r2, [fp, #-128]
	ldr	r3, [r5, #2724]
	cmn	r2, #1
	add	r3, r3, #1
	str	r3, [r5, #2724]
	beq	.L1331
	str	r3, [fp, #-108]
	sub	r0, fp, #40
	ldr	r3, [r5, #3324]
	mov	r1, #1
	str	r2, [fp, #-120]
	mov	r2, #0
	str	r2, [r0, #-84]!
	str	r3, [fp, #-116]
	sub	r3, fp, #104
	str	r3, [fp, #-112]
	bl	FlashReadPages
	ldr	r3, [fp, #-124]
	cmp	r3, #256
	bne	.L1332
	ldr	r0, [fp, #-128]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L1332
.L1331:
	subs	r4, r4, #1
	bne	.L1333
	b	.L1332
.L1328:
	ldr	r3, [r6, #2556]
	mov	r0, #0
	str	r0, [r6, #2720]
	str	r0, [r6, #2724]
	str	r3, [r6, #2716]
	b	.L1326
.L1327:
	ldr	r1, [r6, #2612]
	movw	r4, #10000
	ldr	r8, [r6, #2556]
	ldr	r7, [r6, #2716]
	cmp	r1, r4
	ldr	r10, .L1342+4
	add	r3, r8, #1048576
	movhi	r4, #31
	movls	r4, #63
	cmp	r7, r3
	bhi	.L1337
	ldr	r3, [r6, #2552]
	lsr	r1, r1, #10
	mov	r0, #1000
	add	r1, r1, #1
	mul	r0, r0, r3
	bl	__udivsi3
	add	r0, r0, r7
	cmp	r8, r0
	bhi	.L1337
	ldrh	r3, [r10, #28]
	ands	r0, r4, r3
	movne	r0, r9
	bne	.L1326
	ldr	r2, [r6, #2740]
	cmp	r3, r2
	beq	.L1326
.L1337:
	ldrh	r3, [r10, #28]
	mov	r0, #0
	str	r0, [r5, #2724]
	str	r8, [r5, #2716]
	str	r3, [r5, #2740]
	mov	r3, #1
	str	r3, [r5, #2720]
	b	.L1326
.L1343:
	.align	2
.L1342:
	.word	.LANCHOR0
	.word	.LANCHOR0+2468
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	l2p_flush
	.syntax unified
	.arm
	.fpu softvfp
	.type	l2p_flush, %function
l2p_flush:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r6, .L1348
	mov	r4, #0
	ldr	r5, .L1348+4
	bl	FtlWriteDump_data
.L1345:
	ldrh	r2, [r5]
	uxth	r3, r4
	cmp	r2, r3
	bhi	.L1347
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L1347:
	ldr	r2, [r6, #2540]
	uxth	r0, r4
	mov	r3, #12
	mla	r3, r3, r0, r2
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L1346
	bl	flush_l2p_region
.L1346:
	add	r4, r4, #1
	b	.L1345
.L1349:
	.align	2
.L1348:
	.word	.LANCHOR0
	.word	.LANCHOR0+338
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlVendorPartWrite
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	@ args = 0, pretend = 0, frame = 100
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #100
	mov	r7, r0
	ldr	r4, .L1359
	mov	r5, r1
	str	r2, [fp, #-128]
	add	r2, r0, r1
	ldrh	r3, [r4]
	sub	r4, r4, #296
	cmp	r2, r3
	mvnhi	r8, #0
	bhi	.L1350
	add	r3, r4, #308
	mov	r8, #0
	ldrh	r6, [r3]
	lsr	r6, r0, r6
	lsl	r10, r6, #2
.L1352:
	cmp	r5, #0
	bne	.L1357
.L1350:
	mov	r0, r8
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1357:
	ldr	r3, [r4, #3372]
	mov	r0, r7
	ldr	ip, [r3, r10]
	ldr	r3, .L1359+4
	str	ip, [fp, #-140]
	ldrh	r2, [r3]
	mov	r1, r2
	str	r2, [fp, #-136]
	bl	__umodsi3
	ldr	r2, [fp, #-136]
	ldr	ip, [fp, #-140]
	str	r0, [fp, #-132]
	sub	r3, r2, r0
	uxth	r9, r3
	cmp	r5, r9
	uxthcc	r9, r5
	cmp	ip, #0
	cmpne	r9, r2
	movne	r1, #1
	moveq	r1, #0
	beq	.L1354
	ldr	r2, [r4, #3308]
	sub	r0, fp, #124
	str	ip, [fp, #-120]
	str	r2, [fp, #-116]
	sub	r2, fp, #104
	str	r2, [fp, #-112]
	mov	r2, #1
	mov	r1, r2
	bl	FlashReadPages
.L1355:
	ldr	r3, [fp, #-132]
	lsl	ip, r9, #9
	ldr	r0, [r4, #3308]
	sub	r5, r5, r9
	mov	r2, ip
	ldr	r1, [fp, #-128]
	str	ip, [fp, #-136]
	add	r7, r7, r9
	add	r10, r10, #4
	add	r0, r0, r3, lsl #9
	bl	ftl_memcpy
	mov	r1, r6
	ldr	r2, [r4, #3308]
	ldr	r0, .L1359+8
	add	r6, r6, #1
	bl	FtlMapWritePage
	ldr	r3, [fp, #-128]
	cmn	r0, #1
	ldr	ip, [fp, #-136]
	mvneq	r8, #0
	add	r3, r3, ip
	str	r3, [fp, #-128]
	b	.L1352
.L1354:
	ldr	r3, .L1359+12
	ldr	r0, [r4, #3308]
	ldrh	r2, [r3]
	bl	ftl_memset
	b	.L1355
.L1360:
	.align	2
.L1359:
	.word	.LANCHOR0+296
	.word	.LANCHOR0+258
	.word	.LANCHOR0+3608
	.word	.LANCHOR0+310
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	2
	.global	Ftl_save_ext_data
	.syntax unified
	.arm
	.fpu softvfp
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r2, .L1363
	ldr	r3, .L1363+4
	ldr	r1, [r2, #2640]
	cmp	r1, r3
	ldmfdne	sp, {fp, sp, pc}
	ldr	r3, .L1363+8
	add	r2, r2, #2640
	mov	r1, #1
	mov	r0, #0
	str	r3, [r2, #4]
	ldr	r3, [r2, #-60]
	str	r3, [r2, #88]
	ldr	r3, [r2, #-56]
	str	r3, [r2, #92]
	ldr	r3, [r2, #-64]
	str	r3, [r2, #8]
	ldr	r3, [r2, #-76]
	str	r3, [r2, #12]
	ldr	r3, [r2, #-84]
	str	r3, [r2, #16]
	ldr	r3, [r2, #-68]
	str	r3, [r2, #20]
	ldr	r3, [r2, #-40]
	str	r3, [r2, #28]
	ldr	r3, [r2, #-32]
	str	r3, [r2, #32]
	ldr	r3, [r2, #-80]
	str	r3, [r2, #36]
	ldr	r3, [r2, #-72]
	str	r3, [r2, #40]
	ldr	r3, [r2, #-28]
	str	r3, [r2, #44]
	ldr	r3, [r2, #-24]
	str	r3, [r2, #48]
	bl	FtlVendorPartWrite
	ldmfd	sp, {fp, sp, pc}
.L1364:
	.align	2
.L1363:
	.word	.LANCHOR0
	.word	1179929683
	.word	1342177365
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L1370
	movw	r1, #3652
	ldrh	r2, [r3, r1]
	cmp	r2, #31
	addls	r2, r2, #1
	movhi	r2, #32
	strhls	r2, [r3, r1]	@ movhi
	movls	r2, #1
	cmp	r0, #0
	bne	.L1367
	ldr	r1, [r3, #3348]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r2, r2, r0
	cmp	r1, r2
	bcc	.L1368
.L1367:
	ldr	r2, [r3, #3348]
	mov	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L1370+4
	str	r1, [r2]
	movw	r1, #3340
	ldrh	r1, [r3, r1]
	ldr	r2, [r3, #3348]
	lsl	r3, r1, #9
	str	r3, [r2, #12]
	ldr	r3, [r2, #8]
	add	r3, r3, #1
	str	r3, [r2, #8]
	mov	r3, #0
	str	r3, [r2, #4]
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L1368:
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L1371:
	.align	2
.L1370:
	.word	.LANCHOR0
	.word	1112818501
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	2
	.global	FtlVendorPartRead
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #96
	mov	r10, r2
	ldr	r5, .L1382
	add	r2, r0, r1
	mov	r8, r0
	mov	r7, r1
	ldrh	r3, [r5]
	sub	r5, r5, #296
	cmp	r2, r3
	mvnhi	r9, #0
	bhi	.L1372
	add	r3, r5, #308
	mov	r9, #0
	ldrh	r6, [r3]
	lsr	r6, r0, r6
	lsl	r3, r6, #2
	str	r3, [fp, #-128]
.L1374:
	cmp	r7, #0
	bne	.L1380
.L1372:
	mov	r0, r9
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1380:
	ldr	r2, [fp, #-128]
	mov	r0, r8
	ldr	r3, [r5, #3372]
	ldr	r3, [r3, r2]
	str	r3, [fp, #-136]
	ldr	r3, .L1382+4
	ldrh	r4, [r3]
	mov	r1, r4
	bl	__umodsi3
	sub	r4, r4, r0
	ldr	r3, [fp, #-136]
	uxth	r4, r4
	str	r0, [fp, #-132]
	cmp	r7, r4
	uxthcc	r4, r7
	cmp	r3, #0
	lsl	r2, r4, #9
	str	r2, [fp, #-136]
	beq	.L1376
	ldr	r2, [r5, #3308]
	sub	r0, fp, #124
	str	r3, [fp, #-120]
	str	r3, [fp, #-136]
	str	r2, [fp, #-116]
	sub	r2, fp, #104
	str	r2, [fp, #-112]
	mov	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r2, [fp, #-124]
	ldr	r3, [fp, #-136]
	cmn	r2, #1
	ldr	r2, [r5, #3452]
	mvneq	r9, #0
	cmp	r2, #256
	bne	.L1378
	mov	r2, r3
	mov	r1, r6
	ldr	r0, .L1382+8
	bl	sftl_printk
	ldr	r2, [r5, #3308]
	mov	r1, r6
	ldr	r0, .L1382+12
	bl	FtlMapWritePage
.L1378:
	ldr	r1, [r5, #3308]
	lsl	r2, r4, #9
	ldr	r3, [fp, #-132]
	mov	r0, r10
	add	r1, r1, r3, lsl #9
	bl	ftl_memcpy
.L1379:
	ldr	r3, [fp, #-128]
	add	r6, r6, #1
	sub	r7, r7, r4
	add	r8, r8, r4
	add	r10, r10, r4, lsl #9
	add	r3, r3, #4
	str	r3, [fp, #-128]
	b	.L1374
.L1376:
	lsl	r2, r4, #9
	mov	r1, r3
	mov	r0, r10
	bl	ftl_memset
	b	.L1379
.L1383:
	.align	2
.L1382:
	.word	.LANCHOR0+296
	.word	.LANCHOR0+258
	.word	.LC118
	.word	.LANCHOR0+3608
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	2
	.global	FtlLoadEctTbl
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlLoadEctTbl, %function
FtlLoadEctTbl:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1386
	movw	r5, #3340
	mov	r0, #64
	ldr	r2, [r4, #3348]
	ldrh	r1, [r4, r5]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #3348]
	ldr	r2, [r3]
	ldr	r3, .L1386+4
	cmp	r2, r3
	beq	.L1385
	ldr	r1, .L1386+8
	ldr	r0, .L1386+12
	bl	sftl_printk
	ldrh	r2, [r4, r5]
	mov	r1, #0
	ldr	r0, [r4, #3348]
	lsl	r2, r2, #9
	bl	ftl_memset
.L1385:
	mov	r0, #0
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1387:
	.align	2
.L1386:
	.word	.LANCHOR0
	.word	1112818501
	.word	.LC119
	.word	.LC77
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	Ftl_load_ext_data
	.syntax unified
	.arm
	.fpu softvfp
	.type	Ftl_load_ext_data, %function
Ftl_load_ext_data:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1391
	mov	r1, #1
	mov	r0, #0
	ldr	r5, .L1391+4
	add	r2, r4, #2640
	bl	FtlVendorPartRead
	ldr	r3, [r4, #2640]
	cmp	r3, r5
	beq	.L1389
	mov	r1, #512
	add	r0, r4, #2640
	bl	__memzero
	str	r5, [r4, #2640]
.L1389:
	ldr	r3, [r4, #2640]
	cmp	r3, r5
	bne	.L1390
	ldr	r3, [r4, #2728]
	str	r3, [r4, #2580]
	ldr	r3, [r4, #2732]
	str	r3, [r4, #2584]
	ldr	r3, [r4, #2648]
	str	r3, [r4, #2576]
	ldr	r3, [r4, #2652]
	str	r3, [r4, #2564]
	ldr	r3, [r4, #2656]
	str	r3, [r4, #2556]
	ldr	r3, [r4, #2660]
	str	r3, [r4, #2572]
	ldr	r3, [r4, #2668]
	str	r3, [r4, #2600]
	ldr	r3, [r4, #2672]
	str	r3, [r4, #2608]
	ldr	r3, [r4, #2676]
	str	r3, [r4, #2560]
	ldr	r3, [r4, #2680]
	str	r3, [r4, #2568]
	ldr	r3, [r4, #2684]
	str	r3, [r4, #2612]
	ldr	r3, [r4, #2688]
	str	r3, [r4, #2616]
.L1390:
	ldr	r3, .L1391+8
	ldr	r0, [r4, #2596]
	ldrh	r1, [r4, #240]
	ldrh	r2, [r3]
	ldr	r3, [r4, #2600]
	mla	r0, r0, r2, r3
	bl	__udivsi3
	str	r0, [r4, #2604]
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1392:
	.align	2
.L1391:
	.word	.LANCHOR0
	.word	1179929683
	.word	.LANCHOR0+292
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	2
	.global	FtlMapBlkWriteDump_data
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlMapBlkWriteDump_data, %function
FtlMapBlkWriteDump_data:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, [r0, #36]
	mov	r6, r0
	cmp	r3, #0
	ldmfdeq	sp, {r4, r5, r6, fp, sp, pc}
	mov	r3, #0
	ldrh	r5, [r0, #6]
	str	r3, [r0, #36]
	ldr	r3, .L1400
	ldr	r2, [r0, #24]
	sub	r5, r5, #1
	uxth	r5, r5
	ldr	r0, [r3, #3304]
	mov	r4, r3
	ldr	r1, [r3, #3332]
	str	r0, [r3, #3460]
	str	r1, [r3, #3464]
	ldr	r2, [r2, r5, lsl #2]
	cmp	r2, #0
	str	r2, [r3, #3456]
	beq	.L1395
	mov	r2, #1
	ldr	r0, .L1400+4
	mov	r1, r2
	bl	FlashReadPages
.L1396:
	ldr	r2, [r4, #3460]
	mov	r1, r5
	mov	r0, r6
	bl	FtlMapWritePage
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L1395:
	movw	r3, #310
	mov	r1, #255
	ldrh	r2, [r4, r3]
	bl	ftl_memset
	b	.L1396
.L1401:
	.align	2
.L1400:
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.size	FtlMapBlkWriteDump_data, .-FtlMapBlkWriteDump_data
	.align	2
	.global	FtlVpcTblFlush
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1411
	mov	r6, #0
	mov	r1, #255
	ldr	r3, [r4, #3300]
	add	r5, r4, #2624
	ldr	r7, [r4, #3332]
	sub	r8, r5, #156
	add	r9, r4, #304
	str	r3, [r4, #3460]
	mov	r10, r9
	ldrh	r3, [r5]
	str	r7, [r4, #3464]
	str	r6, [r7, #12]
	strh	r3, [r7, #2]	@ movhi
	ldr	r3, .L1411+4
	strh	r3, [r7]	@ movhi
	ldr	r3, [r4, #2632]
	str	r6, [r7, #8]
	ldrh	r2, [r4, #26]
	str	r3, [r7, #4]
	ldr	r3, .L1411+8
	str	r3, [r4, #2468]
	ldr	r3, .L1411+12
	str	r3, [r4, #2472]
	ldrh	r3, [r5, #6]
	strh	r3, [r5, #-148]	@ movhi
	ldrh	r3, [r4, #254]
	strb	r3, [r4, #2478]
	ldrh	r3, [r4, #24]
	strh	r3, [r5, #-142]	@ movhi
	ldrb	r3, [r4, #30]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #6
	ldrh	r2, [r4, #78]
	strh	r3, [r5, #-140]	@ movhi
	ldrb	r3, [r4, #32]	@ zero_extendqisi2
	strb	r3, [r4, #2479]
	ldrh	r3, [r4, #76]
	strh	r3, [r5, #-138]	@ movhi
	ldrb	r3, [r4, #82]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #6
	strh	r3, [r5, #-136]	@ movhi
	ldrb	r3, [r4, #84]	@ zero_extendqisi2
	strb	r3, [r4, #2480]
	ldrh	r3, [r4, #124]
	strh	r3, [r5, #-134]	@ movhi
	ldrh	r2, [r4, #126]
	ldrb	r3, [r4, #130]	@ zero_extendqisi2
	ldr	r0, [r4, #3460]
	orr	r3, r3, r2, lsl #6
	strh	r3, [r5, #-132]	@ movhi
	ldrb	r3, [r4, #132]	@ zero_extendqisi2
	strb	r3, [r4, #2481]
	ldr	r3, [r4, #2596]
	str	r3, [r4, #2500]
	ldr	r3, [r4, #2588]
	str	r3, [r4, #2508]
	ldr	r3, [r4, #2592]
	str	r3, [r4, #2504]
	movw	r3, #310
	ldrh	r2, [r4, r3]
	bl	ftl_memset
	mov	r1, r8
	mov	r2, #48
	ldr	r0, [r4, #3460]
	movw	r8, #65535
	bl	ftl_memcpy
	ldrh	r2, [r4, #240]
	ldr	r0, [r4, #3460]
	ldr	r1, [r4, #72]
	lsl	r2, r2, #1
	add	r0, r0, #48
	bl	ftl_memcpy
	mov	r0, r6
	bl	FtlUpdateVaildLpn
.L1403:
	ldr	r3, [r4, #3300]
	ldrh	r1, [r5, #2]
	ldrh	r2, [r5]
	str	r3, [r4, #3460]
	ldr	r3, [r4, #3332]
	str	r3, [r4, #3464]
	orr	r3, r1, r2, lsl #10
	str	r3, [r4, #3456]
	ldrh	r3, [r9]
	sub	r3, r3, #1
	cmp	r1, r3
	blt	.L1404
	mov	r3, #0
	ldrh	r8, [r5, #4]
	strh	r3, [r5, #2]	@ movhi
	strh	r2, [r5, #4]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r4, #2588]
	strh	r0, [r5]	@ movhi
	add	r2, r3, #1
	str	r3, [r4, #2632]
	str	r2, [r4, #2588]
	lsl	r2, r0, #10
	str	r2, [r4, #3456]
	str	r3, [r7, #4]
	strh	r0, [r7, #2]	@ movhi
.L1404:
	ldr	r3, .L1411+16
	ldr	r0, [r4, #3300]
	ldrh	r1, [r3]
	bl	js_hash
	mov	r3, #1
	str	r0, [r7, #12]
	mov	r2, r3
	mov	r1, r3
	ldr	r0, .L1411+20
	bl	FlashProgPages
	ldrh	r3, [r5, #2]
	ldr	r2, [r4, #3452]
	add	r3, r3, #1
	uxth	r3, r3
	cmn	r2, #1
	strh	r3, [r5, #2]	@ movhi
	bne	.L1405
	cmp	r3, #1
	bne	.L1406
	movw	r2, #1138
	ldr	r1, .L1411+24
	ldr	r0, .L1411+28
	bl	sftl_printk
.L1406:
	ldrh	r3, [r5, #2]
	add	r6, r6, #1
	uxth	r6, r6
	cmp	r3, #1
	ldrheq	r3, [r10]
	subeq	r3, r3, #1
	strheq	r3, [r5, #2]	@ movhi
	cmp	r6, #3
	bls	.L1403
	mov	r2, r6
	ldr	r1, [r4, #3456]
	ldr	r0, .L1411+32
	bl	sftl_printk
.L1409:
	b	.L1409
.L1405:
	cmp	r2, #256
	cmpne	r3, #1
	beq	.L1403
	movw	r3, #65535
	cmp	r8, r3
	beq	.L1410
	mov	r1, #1
	mov	r0, r8
	bl	FtlFreeSysBlkQueueIn
.L1410:
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1412:
	.align	2
.L1411:
	.word	.LANCHOR0
	.word	-3932
	.word	1179929683
	.word	1342177365
	.word	.LANCHOR0+310
	.word	.LANCHOR0+3452
	.word	.LANCHOR1+471
	.word	.LC8
	.word	.LC120
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlSysFlush
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlSysFlush, %function
FtlSysFlush:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	bl	l2p_flush
	mov	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
	.size	FtlSysFlush, .-FtlSysFlush
	.align	2
	.global	sftl_deinit
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_deinit, %function
sftl_deinit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L1416
	ldr	r3, [r3]
	cmp	r3, #1
	bne	.L1415
	bl	FtlSysFlush
.L1415:
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L1417:
	.align	2
.L1416:
	.word	.LANCHOR2
	.size	sftl_deinit, .-sftl_deinit
	.align	2
	.global	sftl_discard
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_discard, %function
sftl_discard:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #8
	mov	r7, r0
	ldr	r4, .L1434
	mov	r5, r1
	ldr	r3, [r4, #340]
	cmp	r3, r1
	cmpcs	r3, r0
	bls	.L1427
	add	r2, r0, r1
	cmp	r3, r2
	bcc	.L1427
	cmp	r1, #31
	bhi	.L1420
.L1425:
	mov	r0, #0
.L1418:
	sub	sp, fp, #32
	ldmfd	sp, {r4, r5, r6, r7, r8, fp, sp, pc}
.L1420:
	movw	r3, #258
	ldrh	r6, [r4, r3]
	mov	r1, r6
	bl	__udivsi3
	smulbb	r3, r0, r6
	mov	r8, r0
	sub	r7, r7, r3
	uxth	r7, r7
	cmp	r7, #0
	beq	.L1421
	sub	r6, r6, r7
	add	r8, r0, #1
	cmp	r6, r5
	movcs	r6, r5
	uxth	r6, r6
	sub	r5, r5, r6
.L1421:
	ldr	r6, .L1434+4
	mvn	r3, #0
	str	r3, [fp, #-36]
	mov	r7, r6
.L1422:
	ldrh	r3, [r6]
	cmp	r5, r3
	bcs	.L1424
	ldr	r3, [r4, #3656]
	cmp	r3, #32
	bls	.L1425
	mov	r5, #0
	str	r5, [r4, #3656]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L1425
.L1424:
	mov	r2, #0
	sub	r1, fp, #40
	mov	r0, r8
	bl	log2phys
	ldr	r3, [fp, #-40]
	cmn	r3, #1
	beq	.L1423
	ldr	r3, [r4, #3656]
	mov	r2, #1
	sub	r1, fp, #36
	mov	r0, r8
	add	r3, r3, #1
	str	r3, [r4, #3656]
	ldr	r3, [r4, #2560]
	add	r3, r3, #1
	str	r3, [r4, #2560]
	bl	log2phys
	ldr	r0, [fp, #-40]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L1423:
	ldrh	r3, [r7]
	add	r8, r8, #1
	sub	r5, r5, r3
	b	.L1422
.L1427:
	mvn	r0, #0
	b	.L1418
.L1435:
	.align	2
.L1434:
	.word	.LANCHOR0
	.word	.LANCHOR0+258
	.size	sftl_discard, .-sftl_discard
	.align	2
	.global	FtlVpcCheckAndModify
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	mov	r5, #0
	ldr	r4, .L1450
	ldr	r1, .L1450+4
	ldr	r0, .L1450+8
	bl	sftl_printk
	ldrh	r2, [r4, #242]
	mov	r1, #0
	ldr	r0, [r4, #3352]
	lsl	r2, r2, #1
	bl	ftl_memset
.L1437:
	ldr	r3, [r4, #2552]
	cmp	r5, r3
	bcc	.L1439
	ldr	r9, .L1450+12
	mov	r8, #0
	ldr	r10, .L1450+16
.L1440:
	ldrh	r3, [r4, #240]
	uxth	r7, r8
	cmp	r3, r7
	bhi	.L1446
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1439:
	mov	r2, #0
	sub	r1, fp, #44
	mov	r0, r5
	bl	log2phys
	ldr	r0, [fp, #-44]
	cmn	r0, #1
	beq	.L1438
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r4, #3352]
	lsl	r0, r0, #1
	ldrh	r3, [r2, r0]
	add	r3, r3, #1
	strh	r3, [r2, r0]	@ movhi
.L1438:
	add	r5, r5, #1
	b	.L1437
.L1446:
	uxth	r6, r8
	ldr	r3, [r4, #72]
	movw	r1, #65535
	lsl	r5, r6, #1
	ldrh	r2, [r3, r5]
	ldr	r3, [r4, #3352]
	ldrh	r3, [r3, r5]
	cmp	r2, r1
	cmpne	r2, r3
	beq	.L1442
	mov	r1, r6
	mov	r0, r9
	bl	sftl_printk
	ldrh	r3, [r4, #24]
	cmp	r3, r7
	beq	.L1442
	ldrh	r3, [r4, #124]
	cmp	r3, r7
	beq	.L1442
	ldrh	r3, [r4, #76]
	cmp	r3, r7
	beq	.L1442
	ldr	r3, [r4, #72]
	ldrh	r2, [r3, r5]
	cmp	r2, #0
	ldr	r2, [r4, #3352]
	ldrh	r2, [r2, r5]
	strh	r2, [r3, r5]	@ movhi
	bne	.L1444
	mov	r1, r6
	mov	r0, r10
	bl	List_remove_node
	ldrh	r3, [r4, #224]
	cmp	r3, #0
	bne	.L1445
	movw	r2, #2323
	ldr	r1, .L1450+4
	ldr	r0, .L1450+20
	bl	sftl_printk
.L1445:
	ldrh	r3, [r4, #224]
	mov	r0, r6
	sub	r3, r3, #1
	strh	r3, [r4, #224]	@ movhi
	bl	INSERT_DATA_LIST
.L1442:
	add	r8, r8, #1
	b	.L1440
.L1444:
	mov	r0, r6
	bl	update_vpc_list
	b	.L1442
.L1451:
	.align	2
.L1450:
	.word	.LANCHOR0
	.word	.LANCHOR1+486
	.word	.LC111
	.word	.LC121
	.word	.LANCHOR0+2532
	.word	.LC8
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	allocate_new_data_superblock
	.syntax unified
	.arm
	.fpu softvfp
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1462
	mov	r6, r0
	ldrh	r5, [r0]
	ldrh	r3, [r4, #240]
	cmp	r3, r5
	bcs	.L1453
	movw	r2, #2759
	ldr	r1, .L1462+4
	ldr	r0, .L1462+8
	bl	sftl_printk
.L1453:
	movw	r3, #65535
	cmp	r5, r3
	beq	.L1454
	ldr	r2, [r4, #72]
	lsl	r3, r5, #1
	mov	r0, r5
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1455
	bl	INSERT_DATA_LIST
.L1454:
	mov	r3, #1
	strb	r3, [r6, #8]
	movw	r3, #3438
	ldrh	r0, [r4, r3]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1456
	cmp	r5, r0
	bne	.L1457
	ldr	r2, [r4, #72]
	lsl	r3, r0, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1458
.L1457:
	bl	update_vpc_list
.L1458:
	mvn	r2, #0
	movw	r3, #3438
	strh	r2, [r4, r3]	@ movhi
.L1456:
	mov	r0, r6
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	r0, #0
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L1455:
	bl	INSERT_FREE_LIST
	b	.L1454
.L1463:
	.align	2
.L1462:
	.word	.LANCHOR0
	.word	.LANCHOR1+507
	.word	.LC8
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlProgPages
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlProgPages, %function
FtlProgPages:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	mov	r6, #0
	ldr	r8, .L1477
	mov	r5, r3
	mov	r2, #0
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	mov	r4, r0
	mov	r9, r1
	bl	FlashProgPages
.L1465:
	cmp	r6, r9
	beq	.L1471
	ldr	r7, .L1477+4
	b	.L1472
.L1467:
	ldr	r1, [r4, #4]
	mov	r0, r7
	bl	sftl_printk
	ldr	r0, [r4, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	bne	.L1466
	mov	r0, r5
	bl	allocate_new_data_superblock
.L1466:
	mov	r0, r5
	bl	get_new_active_ppa
	mov	r2, #0
	str	r0, [r4, #4]
	str	r0, [fp, #-44]
	mov	r1, #1
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	mov	r0, r4
	bl	FlashProgPages
.L1472:
	ldr	r2, [r4]
	cmn	r2, #1
	cmpne	r2, #256
	beq	.L1467
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	ldrh	r3, [r8, #232]
	cmp	r2, r3
	bcc	.L1468
	movw	r2, #985
	ldr	r1, .L1477+8
	ldr	r0, .L1477+12
	bl	sftl_printk
.L1468:
	ldr	r3, [r4, #4]
	sub	r1, fp, #40
	mov	r2, #1
	ldr	r0, [r4, #16]
	str	r3, [r1, #-4]!
	bl	log2phys
	ldr	r3, [r4, #12]
	ldr	r10, [r3, #12]
	ubfx	r0, r10, #10, #16
	bl	P2V_block_in_plane
	cmn	r10, #1
	mov	r7, r0
	beq	.L1469
	ldr	r2, [r8, #72]
	lsl	r3, r0, #1
	ldrh	r2, [r2, r3]
	cmp	r2, #0
	bne	.L1470
	mov	r1, r0
	ldr	r0, .L1477+16
	bl	sftl_printk
.L1470:
	mov	r0, r7
	bl	decrement_vpc_count
.L1469:
	add	r6, r6, #1
	add	r4, r4, #20
	b	.L1465
.L1471:
	ldr	r3, .L1477
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	ldrh	r3, [r3, #232]
	cmp	r2, r3
	ldmibcc	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
	mov	r2, #1000
	ldr	r1, .L1477+8
	ldr	r0, .L1477+12
	bl	sftl_printk
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1478:
	.align	2
.L1477:
	.word	.LANCHOR0
	.word	.LC122
	.word	.LANCHOR1+536
	.word	.LC8
	.word	.LC123
	.size	FtlProgPages, .-FtlProgPages
	.align	2
	.global	FtlGcFreeTempBlock
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #4
	movw	r3, #302
	ldr	r4, .L1503
	ldrh	r2, [r4, #124]
	ldrh	r1, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1480
.L1487:
	ldrh	r2, [r4, #124]
	mov	r3, #0
	str	r3, [r4, #3444]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1500
.L1481:
	mov	r0, #0
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1480:
	cmp	r0, #0
	ldr	r5, .L1503+4
	beq	.L1483
	ldrh	r2, [r5, #4]
	cmp	r2, r3
	beq	.L1484
.L1485:
	mov	r1, #2
.L1483:
	ldr	r0, .L1503+8
	bl	FtlGcScanTempBlk
	ldrh	r2, [r5, #4]
	movw	r3, #65535
	str	r0, [fp, #-44]
	cmp	r2, r3
	beq	.L1487
	mov	r0, #1
	ldmib	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1484:
	mov	r3, #0
	strh	r3, [r5, #4]	@ movhi
	ldrh	r3, [r4, #224]
	cmp	r3, #17
	bhi	.L1485
	b	.L1483
.L1500:
	movw	r3, #3204
	ldrb	r1, [r4, #131]	@ zero_extendqisi2
	ldrh	r2, [r4, r3]
	movw	r3, #302
	ldrh	r3, [r4, r3]
	mul	r3, r3, r1
	cmp	r2, r3
	beq	.L1488
	mov	r2, #164
	ldr	r1, .L1503+12
	ldr	r0, .L1503+16
	bl	sftl_printk
.L1488:
	movw	r0, #302
	ldrb	r2, [r4, #131]	@ zero_extendqisi2
	ldrh	r0, [r4, r0]
	mov	r5, #0
	ldrh	r3, [r4, #124]
	mov	r9, #12
	ldr	r1, [r4, #72]
	ldr	r8, .L1503+20
	smulbb	r2, r2, r0
	lsl	r3, r3, #1
	strh	r2, [r1, r3]	@ movhi
	movw	r3, #3204
	ldr	r2, [r4, #2576]
	ldrh	r3, [r4, r3]
	add	r3, r3, r2
	str	r3, [r4, #2576]
.L1489:
	ldrh	r2, [r8]
	uxth	r3, r5
	cmp	r2, r3
	bhi	.L1493
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r0, [r4, #124]
	ldr	r2, [r4, #72]
	lsl	r3, r0, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1494
	bl	INSERT_DATA_LIST
.L1495:
	mov	r5, #0
	movw	r3, #3204
	strh	r5, [r4, r3]	@ movhi
	mvn	r6, #0
	movw	r3, #3196
	strh	r6, [r4, #124]	@ movhi
	strh	r5, [r4, r3]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	movw	r3, #2620
	ldrh	r2, [r4, #224]
	ldrh	r3, [r4, r3]
	strh	r6, [r4, #172]	@ movhi
	add	r3, r3, r3, lsl #1
	cmp	r2, r3, asr #2
	ldrgt	r3, .L1503+24
	movgt	r2, #20
	strhgt	r2, [r3]	@ movhi
	b	.L1481
.L1493:
	uxth	r7, r5
	ldr	r10, [r4, #3200]
	ldr	r3, [r4, #2552]
	mul	r7, r9, r7
	add	r6, r10, r7
	ldr	r0, [r6, #8]
	cmp	r0, r3
	bcc	.L1490
.L1501:
	ldrh	r0, [r4, #124]
	b	.L1502
.L1490:
	mov	r2, #0
	sub	r1, fp, #44
	bl	log2phys
	ldr	r3, [fp, #-44]
	ldr	r0, [r10, r7]
	cmp	r0, r3
	bne	.L1492
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	mov	r2, #1
	mov	r7, r0
	add	r1, r6, #4
	ldr	r0, [r6, #8]
	bl	log2phys
	mov	r0, r7
.L1502:
	bl	decrement_vpc_count
	b	.L1491
.L1492:
	ldr	r2, [r6, #4]
	cmp	r3, r2
	bne	.L1501
.L1491:
	add	r5, r5, #1
	b	.L1489
.L1494:
	bl	INSERT_FREE_LIST
	b	.L1495
.L1504:
	.align	2
.L1503:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+124
	.word	.LANCHOR1+549
	.word	.LC8
	.word	.LANCHOR0+3204
	.word	.LANCHOR0+3152
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	FtlGcPageRecovery
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1507
	movw	r5, #302
	ldrh	r1, [r4, r5]
	add	r0, r4, #124
	bl	FtlGcScanTempBlk
	ldrh	r2, [r4, #126]
	ldrh	r3, [r4, r5]
	cmp	r2, r3
	ldmfdcc	sp, {r4, r5, fp, sp, pc}
	add	r0, r4, #3392
	bl	FtlMapBlkWriteDump_data
	mov	r0, #0
	bl	FtlGcFreeTempBlock
	mov	r3, #0
	str	r3, [r4, #3444]
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1508:
	.align	2
.L1507:
	.word	.LANCHOR0
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1510
	mov	r5, #0
	add	r6, r4, #24
	str	r5, [r4, #3476]
	mov	r0, r6
	add	r4, r4, #76
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
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L1511:
	.align	2
.L1510:
	.word	.LANCHOR0
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.syntax unified
	.arm
	.fpu softvfp
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1520
	mov	r3, #0
	mov	r5, #0
	mov	r6, #20
	mov	r2, r3
	ldr	r1, [r4, #3168]
	ldr	r0, [r4, #3292]
	bl	FlashProgPages
.L1513:
	ldr	r1, [r4, #3168]
	uxth	r3, r5
	cmp	r1, r3
	bhi	.L1516
	ldr	r0, [r4, #3292]
	bl	FtlGcBufFree
	ldrh	r3, [r4, #128]
	mov	r0, #0
	str	r0, [r4, #3168]
	cmp	r3, r0
	ldmfdne	sp, {r4, r5, r6, fp, sp, pc}
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	mov	r0, #1
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L1516:
	mul	r3, r6, r3
	ldr	r2, [r4, #3292]
	add	r5, r5, #1
	add	r1, r2, r3
	ldr	r2, [r2, r3]
	ldr	r0, [r1, #12]
	ldr	r1, [r1, #4]
	cmn	r2, #1
	ldrne	r2, [r0, #8]
	ldr	r0, [r0, #12]
	bl	FtlGcUpdatePage
	b	.L1513
.L1521:
	.align	2
.L1520:
	.word	.LANCHOR0
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	Ftl_get_new_temp_ppa
	.syntax unified
	.arm
	.fpu softvfp
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1525
	movw	r3, #65535
	ldrh	r2, [r4, #124]
	cmp	r2, r3
	beq	.L1523
	ldrh	r3, [r4, #128]
	cmp	r3, #0
	bne	.L1524
.L1523:
	mov	r0, #0
	mov	r5, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L1525+4
	strb	r5, [r4, #132]
	bl	allocate_data_superblock
	movw	r3, #3196
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3204
	strh	r5, [r4, r3]	@ movhi
	bl	l2p_flush
	mov	r0, r5
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L1524:
	ldr	r0, .L1525+4
	bl	get_new_active_ppa
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1526:
	.align	2
.L1525:
	.word	.LANCHOR0
	.word	.LANCHOR0+124
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.global	rk_ftl_garbage_collect
	.syntax unified
	.arm
	.fpu softvfp
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #40
	ldr	r3, .L1653
	str	r0, [fp, #-64]
	ldr	r0, [r3, #3272]
	mov	r4, r3
	cmp	r0, #0
	movne	r0, #0
	bne	.L1527
	add	r2, r3, #2528
	ldrh	r2, [r2]
	cmp	r2, #47
	bls	.L1527
	ldr	r2, .L1653+4
	ldrh	r1, [r2, #4]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L1529
.L1532:
	ldrh	r3, [r4, #220]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1530
.L1531:
	bl	FtlReadRefresh
	ldr	r3, [r4, #3156]
	movw	r6, #65535
	ldr	r2, [fp, #-64]
	add	r3, r3, #1
	add	r3, r3, r2, lsl #7
	ldrh	r2, [r4, #172]
	str	r3, [r4, #3156]
	cmp	r2, r6
	bne	.L1533
	ldrh	r6, [r4, #124]
	cmp	r6, r2
	movne	r6, r2
	bne	.L1533
	ldrh	r5, [r4, #222]
	cmp	r5, r6
	bne	.L1533
	ldrh	r2, [r4, #224]
	cmp	r2, #24
	movcc	r2, #5120
	movcs	r2, #1024
	cmp	r3, r2
	movls	r6, r5
	bls	.L1533
	mov	r8, #0
	movw	r3, #3660
	str	r8, [r4, #3156]
	strh	r8, [r4, r3]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r5
	mov	r6, r0
	movne	r5, r0
	bne	.L1535
	movw	r7, #3154
	ldrh	r2, [r4, #224]
	ldrh	r3, [r4, r7]
	cmp	r2, r3
	bcs	.L1536
	mov	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r6
	beq	.L1538
	mov	r0, r3
	ldr	r3, [r4, #72]
	lsl	r0, r0, #1
	ldrh	r3, [r3, r0]
	cmp	r3, #7
	bhi	.L1539
	mov	r0, r8
	bl	List_get_gc_head_node
	uxth	r5, r0
	mov	r3, #128
	strh	r3, [r4, r7]	@ movhi
	cmp	r5, r6
	bne	.L1535
.L1538:
	bl	FtlGcReFreshBadBlk
.L1533:
	movw	r1, #65535
	ldr	r2, [fp, #-64]
	sub	r3, r6, r1
	clz	r3, r3
	lsr	r3, r3, #5
	cmp	r2, #0
	movne	r2, #0
	andeq	r2, r3, #1
	cmp	r2, #0
	beq	.L1541
	ldrh	r3, [r4, #224]
	cmp	r3, #24
	movhi	r5, #1
	bhi	.L1542
	movw	r2, #302
	cmp	r3, #16
	ldrh	r5, [r4, r2]
	lsrhi	r5, r5, #5
	bhi	.L1542
	cmp	r3, #12
	lsrhi	r5, r5, #4
	bhi	.L1542
	cmp	r3, #8
	lsrhi	r5, r5, #2
.L1542:
	ldr	r1, .L1653+8
	ldrh	r2, [r1]
	cmp	r2, r3
	mov	r2, r1
	bcs	.L1546
	ldrh	r3, [r4, #124]
	movw	r0, #65535
	cmp	r3, r0
	bne	.L1547
	ldrh	r0, [r4, #222]
	cmp	r0, r3
	bne	.L1547
	movw	r3, #3660
	ldrh	r0, [r4, r3]
	cmp	r0, #0
	bne	.L1548
	ldr	r3, [r4, #2552]
	ldr	ip, [r4, #2548]
	add	r3, r3, r3, lsl #1
	cmp	ip, r3, lsr #2
	movcs	r3, #18
	strhcs	r3, [r1]	@ movhi
	bcs	.L1550
.L1548:
	movw	r3, #2620
	ldrh	r3, [r4, r3]
	add	r3, r3, r3, lsl #1
	asr	r3, r3, #2
	strh	r3, [r2]	@ movhi
.L1550:
	mov	r3, #0
	str	r3, [r4, #3164]
.L1527:
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1529:
	ldrh	r3, [r3, #124]
	cmp	r3, r2
	beq	.L1532
	mov	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	beq	.L1532
	mov	r0, #1
	b	.L1527
.L1530:
	ldrh	r1, [r4, #222]
	cmp	r1, r2
	strheq	r3, [r4, #222]	@ movhi
	mvneq	r3, #0
	strheq	r3, [r4, #220]	@ movhi
	b	.L1531
.L1539:
	mov	r3, #64
.L1648:
	strh	r3, [r4, r7]	@ movhi
	b	.L1538
.L1536:
	mov	r3, #80
	b	.L1648
.L1535:
	ldr	r0, .L1653+8
	lsl	r1, r5, #1
	ldr	r3, [r4, #72]
	mov	r6, r5
	ldrh	r2, [r4, #224]
	ldrh	r0, [r0]
	ldrh	r3, [r3, r1]
	str	r0, [sp, #4]
	ldr	r0, [r4, #2536]
	ldrh	r1, [r0, r1]
	ldr	r0, .L1653+12
	str	r1, [sp]
	mov	r1, r5
	bl	sftl_printk
	b	.L1538
.L1547:
	movw	r3, #2620
	ldrh	r3, [r4, r3]
	add	r3, r3, r3, lsl #1
	asr	r3, r3, #2
	strh	r3, [r2]	@ movhi
.L1546:
	movw	r3, #3206
	movw	r6, #65535
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	addne	r5, r5, #32
	uxthne	r5, r5
.L1552:
	ldrh	r3, [r4, #172]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1561
	cmp	r6, r3
	strhne	r6, [r4, #172]	@ movhi
	bne	.L1563
	ldrh	r3, [r4, #222]
	cmp	r3, r6
	beq	.L1563
	ldr	r2, [r4, #72]
	lsl	r3, r3, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	mvneq	r3, #0
	strheq	r3, [r4, #222]	@ movhi
	ldrh	r3, [r4, #222]
	strh	r3, [r4, #172]	@ movhi
	mvn	r3, #0
	strh	r3, [r4, #222]	@ movhi
.L1563:
	ldrh	r0, [r4, #172]
	mov	r3, #0
	strb	r3, [r4, #180]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1561
	bl	IsBlkInGcList
	cmp	r0, #0
	mvnne	r3, #0
	strhne	r3, [r4, #172]	@ movhi
	movw	r3, #65535
	ldrh	r2, [r4, #172]
	cmp	r2, r3
	beq	.L1561
	ldr	r0, .L1653+16
	bl	make_superblock
	mov	r3, #0
	movw	r2, #3662
	strh	r3, [r4, r2]	@ movhi
	strh	r3, [r4, #174]	@ movhi
	strb	r3, [r4, #178]
	ldrh	r3, [r4, #172]
	ldr	r2, [r4, #72]
	lsl	r3, r3, #1
	ldrh	r2, [r2, r3]
	ldr	r3, .L1653+20
	strh	r2, [r3]	@ movhi
.L1561:
	ldrh	r3, [r4, #172]
	ldrh	r2, [r4, #24]
	cmp	r2, r3
	beq	.L1567
	ldrh	r2, [r4, #76]
	cmp	r2, r3
	beq	.L1567
	ldrh	r2, [r4, #124]
	cmp	r2, r3
	bne	.L1568
.L1567:
	mvn	r3, #0
	strh	r3, [r4, #172]	@ movhi
.L1568:
	ldr	r4, .L1653
	mov	r7, r4
.L1603:
	ldrh	r2, [r4, #172]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1569
	ldr	r10, .L1653+24
	mov	r3, #0
	str	r3, [r4, #3164]
	mov	r9, r10
.L1570:
	ldrh	r8, [r10]
	mov	r0, r8
	bl	List_get_gc_head_node
	uxth	r2, r0
	movw	r3, #65535
	cmp	r2, r3
	strh	r2, [r7, #172]	@ movhi
	bne	.L1571
	mov	r3, #0
	mov	r0, #8
	strh	r3, [r10]	@ movhi
	b	.L1527
.L1541:
	ldrh	r7, [r4, #124]
	cmp	r7, r1
	bne	.L1617
	ldrh	r1, [r4, #222]
	cmp	r1, r7
	movne	r3, #0
	andeq	r3, r3, #1
	cmp	r3, #0
	beq	.L1617
	ldrh	r3, [r4, #172]
	cmp	r3, r7
	beq	.L1553
.L1558:
	mov	r6, r7
.L1617:
	mov	r5, #1
	b	.L1552
.L1553:
	ldr	r5, .L1653+8
	str	r2, [r4, #3164]
	ldrh	r2, [r4, #224]
	ldrh	r3, [r5]
	cmp	r2, r3
	bls	.L1554
	movw	r3, #3660
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L1555
	ldr	r3, [r4, #2552]
	ldr	r2, [r4, #2548]
	add	r3, r3, r3, lsl #1
	cmp	r2, r3, lsr #2
	movcs	r3, #18
	bcs	.L1649
.L1555:
	movw	r3, #2620
	ldrh	r3, [r4, r3]
	add	r3, r3, r3, lsl #1
	asr	r3, r3, #2
.L1649:
	strh	r3, [r5]	@ movhi
	bl	FtlReadRefresh
	mov	r0, #0
	bl	List_get_gc_head_node
	uxth	r0, r0
	ldr	r3, [r4, #72]
	lsl	r0, r0, #1
	ldrh	r3, [r3, r0]
	cmp	r3, #4
	bls	.L1554
.L1652:
	movw	r3, #3660
	ldrh	r0, [r4, r3]
	b	.L1527
.L1554:
	movw	r6, #3660
	ldrh	r0, [r4, r6]
	cmp	r0, #0
	bne	.L1558
	movw	r3, #2620
	ldrh	r8, [r4, r3]
	add	r3, r8, r8, lsl #1
	asr	r3, r3, #2
	strh	r3, [r5]	@ movhi
	bl	List_get_gc_head_node
	uxth	r0, r0
	ldr	r3, [r4, #72]
	lsl	r0, r0, #1
	ldrh	r2, [r3, r0]
	ldr	r3, .L1653+28
	ldrh	r1, [r3]
	ldrh	r3, [r4, #232]
	mul	r3, r3, r1
	add	r3, r3, r3, lsr #31
	cmp	r2, r3, asr #1
	ble	.L1559
	ldrh	r3, [r4, #224]
	sub	r8, r8, #1
	cmp	r3, r8
	blt	.L1559
	bl	FtlReadRefresh
	ldrh	r0, [r4, r6]
	b	.L1527
.L1559:
	cmp	r2, #0
	bne	.L1558
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r0, [r4, #224]
	add	r0, r0, #1
	b	.L1527
.L1571:
	uxth	r3, r0
	str	r2, [fp, #-56]
	add	r8, r8, #1
	mov	r0, r3
	str	r3, [fp, #-52]
	bl	IsBlkInGcList
	cmp	r0, #0
	ldr	r3, [fp, #-52]
	ldr	r2, [fp, #-56]
	strhne	r8, [r10]	@ movhi
	bne	.L1570
	lsl	r0, r3, #1
	ldr	r3, .L1653+32
	ldrh	lr, [r7, #232]
	uxth	r8, r8
	ldr	r1, [r7, #72]
	ldrh	r3, [r3]
	strh	r8, [r10]	@ movhi
	ldrh	ip, [r1, r0]
	mul	r3, lr, r3
	add	lr, r3, r3, lsr #31
	cmp	ip, lr, asr #1
	bgt	.L1574
	cmp	r8, #48
	cmphi	ip, #8
	bls	.L1575
	ldr	ip, .L1653+36
	ldrh	ip, [ip]
	cmp	ip, #35
	bhi	.L1575
.L1574:
	mov	ip, #0
	strh	ip, [r9]	@ movhi
.L1575:
	ldrh	r1, [r1, r0]
	movw	r0, #65535
	cmp	r3, r1
	cmple	r6, r0
	bne	.L1576
	ldrh	r3, [r9]
	cmp	r3, #3
	bhi	.L1576
	mvn	r3, #0
	strh	r3, [r7, #172]	@ movhi
	mov	r3, #0
	strh	r3, [r9]	@ movhi
	movw	r3, #3660
	ldrh	r0, [r7, r3]
	b	.L1527
.L1576:
	cmp	r1, #0
	bne	.L1577
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r9]
	add	r3, r3, #1
	strh	r3, [r9]	@ movhi
	b	.L1570
.L1577:
	mov	r3, #0
	strb	r3, [r7, #180]
	ldrh	r3, [r7, #24]
	cmp	r3, r2
	bne	.L1578
	movw	r2, #717
	ldr	r1, .L1653+40
	ldr	r0, .L1653+44
	bl	sftl_printk
.L1578:
	ldrh	r2, [r7, #172]
	ldrh	r3, [r7, #76]
	cmp	r2, r3
	bne	.L1579
	movw	r2, #718
	ldr	r1, .L1653+40
	ldr	r0, .L1653+44
	bl	sftl_printk
.L1579:
	ldrh	r2, [r7, #172]
	ldrh	r3, [r7, #124]
	cmp	r2, r3
	bne	.L1580
	movw	r2, #719
	ldr	r1, .L1653+40
	ldr	r0, .L1653+44
	bl	sftl_printk
.L1580:
	ldr	r0, .L1653+16
	bl	make_superblock
	ldrh	r2, [r7, #172]
	mov	r3, #0
	ldr	r1, .L1653+48
	ldr	r0, [r7, #72]
	lsl	r2, r2, #1
	strh	r3, [r1]	@ movhi
	ldrh	r2, [r0, r2]
	strh	r3, [r7, #174]	@ movhi
	strb	r3, [r7, #178]
	strh	r2, [r1, #2]	@ movhi
.L1569:
	mov	r3, #1
	str	r3, [r7, #3272]
	ldr	r3, .L1653+32
	ldrh	r3, [r3]
	str	r3, [fp, #-52]
	ldr	r3, [fp, #-64]
	cmp	r3, #0
	beq	.L1581
	ldr	r2, [fp, #-52]
	ldrh	r3, [r7, #232]
	ldr	r1, [r7, #72]
	mul	r3, r2, r3
	ldrh	r2, [r7, #172]
	lsl	r2, r2, #1
	ldrh	r2, [r1, r2]
	sub	r3, r3, r2
	add	r2, r3, #3
	cmp	r3, #0
	movlt	r3, r2
	add	r5, r5, r3, asr #2
	uxth	r5, r5
.L1581:
	ldrh	r3, [r7, #174]
	ldr	r1, [fp, #-52]
	add	r2, r3, r5
	cmp	r2, r1
	movgt	r2, r1
	subgt	r5, r2, r3
	mov	r3, #0
	uxthgt	r5, r5
.L1651:
	str	r3, [fp, #-60]
	ldrh	r3, [fp, #-60]
	cmp	r5, r3
	bls	.L1591
	ldr	r3, [fp, #-60]
	mov	lr, #20
	ldrh	r1, [r4, #174]
	ldrh	r9, [r4, #232]
	ldr	r0, [r4, #3184]
	add	r1, r1, r3
	mov	r3, #0
	ldr	ip, .L1653+52
	mov	r8, r3
	b	.L1592
.L1585:
	ldrh	r2, [ip, #2]!
	movw	r10, #65535
	add	r3, r3, #1
	cmp	r2, r10
	mlane	r10, lr, r8, r0
	addne	r8, r8, #1
	orrne	r2, r1, r2, lsl #10
	uxthne	r8, r8
	strne	r2, [r10, #4]
.L1592:
	uxth	r2, r3
	cmp	r9, r2
	bhi	.L1585
	ldrb	r2, [r4, #180]	@ zero_extendqisi2
	mov	r1, r8
	bl	FlashReadPages
	mov	r3, #0
.L1650:
	str	r3, [fp, #-56]
	ldrh	r3, [fp, #-56]
	cmp	r8, r3
	ldrls	r3, [fp, #-60]
	addls	r3, r3, #1
	bls	.L1651
.L1590:
	ldr	r3, [fp, #-56]
	mov	r9, #20
	mul	r9, r9, r3
	ldr	r3, [r4, #3184]
	add	r2, r3, r9
	ldr	r3, [r3, r9]
	cmn	r3, #1
	beq	.L1587
	ldr	r10, [r2, #12]
	movw	r2, #61589
	ldrh	r1, [r10]
	cmp	r1, r2
	bne	.L1587
	ldr	ip, [r10, #8]
	cmn	ip, #1
	bne	.L1588
	movw	r2, #753
	ldr	r1, .L1653+40
	ldr	r0, .L1653+44
	str	ip, [fp, #-68]
	bl	sftl_printk
	ldr	ip, [fp, #-68]
.L1588:
	mov	r2, #0
	sub	r1, fp, #48
	mov	r0, ip
	bl	log2phys
	ldr	r2, [r4, #3184]
	ldr	r1, [fp, #-48]
	add	r2, r2, r9
	ldr	r0, [r2, #4]
	cmp	r0, r1
	bne	.L1587
	ldr	r3, .L1653+48
	mov	ip, #20
	ldr	r0, [r4, #3168]
	ldr	r2, [r2, #16]
	ldrh	r1, [r3]
	str	ip, [fp, #-72]
	add	r1, r1, #1
	strh	r1, [r3]	@ movhi
	ldr	r1, [r4, #3292]
	mla	r1, ip, r0, r1
	str	r2, [r1, #16]
	str	r1, [fp, #-68]
	bl	Ftl_get_new_temp_ppa
	ldr	r1, [fp, #-68]
	ldr	r2, [r4, #3292]
	ldr	ip, [fp, #-72]
	str	r0, [r1, #4]
	ldr	r1, [r4, #3168]
	mla	r2, ip, r1, r2
	ldr	r1, [r4, #3184]
	add	r1, r1, r9
	ldr	r0, [r1, #8]
	str	r0, [r2, #8]
	ldr	r1, [r1, #12]
	str	r1, [r2, #12]
	mov	r1, #1
	ldr	r2, [fp, #-48]
	str	r2, [r10, #12]
	ldrh	r2, [r4, #124]
	strh	r2, [r10, #2]	@ movhi
	ldr	r2, [r4, #2592]
	ldr	r3, [r4, #3168]
	ldr	r0, [r4, #3184]
	str	r2, [r10, #4]
	add	r3, r3, #1
	add	r0, r0, r9
	str	r3, [r4, #3168]
	bl	FtlGcBufAlloc
	ldrb	r2, [r4, #131]	@ zero_extendqisi2
	ldr	r3, [r4, #3168]
	cmp	r2, r3
	beq	.L1589
	ldrh	r3, [r4, #128]
	cmp	r3, #0
	bne	.L1587
.L1589:
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	beq	.L1587
	ldr	r3, .L1653
	mov	r2, #0
	mvn	r1, #0
	str	r2, [r3, #3272]
	strh	r2, [r3, #174]	@ movhi
	movw	r2, #3660
	strh	r1, [r3, #172]	@ movhi
	ldrh	r0, [r3, r2]
	b	.L1527
.L1587:
	ldr	r3, [fp, #-56]
	add	r3, r3, #1
	b	.L1650
.L1591:
	ldrh	r3, [r4, #174]
	add	r5, r5, r3
	ldr	r3, [fp, #-52]
	uxth	r5, r5
	cmp	r3, r5
	strh	r5, [r4, #174]	@ movhi
	bhi	.L1593
	ldr	r3, [r4, #3168]
	cmp	r3, #0
	beq	.L1594
	bl	Ftl_gc_temp_data_write_back
	cmp	r0, #0
	movne	r3, #0
	strne	r3, [r4, #3272]
	bne	.L1652
.L1594:
	ldr	r3, .L1653+48
	ldrh	r5, [r3]
	cmp	r5, #0
	bne	.L1595
	ldrh	r3, [r4, #172]
	ldr	r2, [r4, #72]
	lsl	r3, r3, #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L1595
.L1596:
	ldr	r3, [r4, #2552]
	cmp	r5, r3
	bcs	.L1601
	mov	r2, #0
	sub	r1, fp, #44
	mov	r0, r5
	bl	log2phys
	ldr	r0, [fp, #-44]
	cmn	r0, #1
	beq	.L1597
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #172]
	cmp	r3, r0
	bne	.L1597
.L1601:
	ldr	r3, [r4, #2552]
	cmp	r5, r3
	bcc	.L1595
	ldrh	r3, [r4, #172]
	mov	r1, #0
	ldr	r2, [r4, #72]
	lsl	r3, r3, #1
	strh	r1, [r2, r3]	@ movhi
	ldrh	r0, [r4, #172]
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1595:
	mvn	r3, #0
	strh	r3, [r4, #172]	@ movhi
.L1593:
	mov	r3, #0
	str	r3, [r4, #3272]
	ldrh	r3, [r4, #224]
	cmp	r3, #2
	bhi	.L1602
	ldr	r3, .L1653+32
	ldrh	r5, [r3]
	b	.L1603
.L1597:
	add	r5, r5, #1
	b	.L1596
.L1602:
	movw	r2, #3660
	ldrh	r0, [r4, r2]
	cmp	r0, #0
	addeq	r0, r3, #1
	b	.L1527
.L1654:
	.align	2
.L1653:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+3152
	.word	.LC124
	.word	.LANCHOR0+172
	.word	.LANCHOR0+3664
	.word	.LANCHOR0+3160
	.word	.LANCHOR0+304
	.word	.LANCHOR0+302
	.word	.LANCHOR0+3196
	.word	.LANCHOR1+568
	.word	.LC8
	.word	.LANCHOR0+3662
	.word	.LANCHOR0+186
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	2
	.global	FtlRead
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlRead, %function
FtlRead:
	@ args = 0, pretend = 0, frame = 52
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #52
	uxtb	r0, r0
	mov	r4, r1
	str	r2, [fp, #-52]
	cmp	r0, #16
	mov	r9, r3
	bne	.L1656
	mov	r2, r3
	ldr	r1, [fp, #-52]
	add	r0, r4, #256
	bl	FtlVendorPartRead
	str	r0, [fp, #-48]
.L1655:
	ldr	r0, [fp, #-48]
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1656:
	ldr	r10, .L1692
	ldr	r2, [fp, #-52]
	ldr	r3, [r10, #340]
	cmp	r2, r3
	cmpls	r1, r3
	movcs	r8, #1
	movcc	r8, #0
	bcs	.L1678
	add	r2, r1, r2
	cmp	r3, r2
	str	r2, [fp, #-60]
	bcc	.L1678
	ldr	r3, .L1692+4
	ldr	r5, [r3]
	cmn	r5, #1
	streq	r5, [fp, #-48]
	beq	.L1655
	movw	r3, #258
	mov	r0, r4
	ldrh	r5, [r10, r3]
	mov	r7, r8
	mov	r1, r5
	bl	__udivsi3
	ldr	r3, [fp, #-60]
	mov	r1, r5
	str	r0, [fp, #-64]
	mov	r5, r10
	sub	r0, r3, #1
	bl	__udivsi3
	ldr	r3, [fp, #-64]
	ldr	r2, [fp, #-52]
	ldr	r6, [fp, #-64]
	rsb	r3, r3, #1
	str	r0, [fp, #-68]
	add	r3, r3, r0
	str	r8, [fp, #-76]
	str	r3, [fp, #-56]
	ldr	r3, [r10, #2584]
	str	r8, [fp, #-72]
	str	r8, [fp, #-48]
	add	r3, r3, r2
	ldr	r2, [fp, #-56]
	str	r3, [r10, #2584]
	ldr	r3, [r10, #2556]
	add	r3, r3, r2
	str	r3, [r10, #2556]
.L1658:
	ldr	r3, [fp, #-56]
	cmp	r3, #0
	bne	.L1675
	ldr	r3, .L1692
	movw	r2, #3206
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	bne	.L1676
	ldrh	r3, [r3, #224]
	cmp	r3, #31
	bhi	.L1655
.L1676:
	mov	r1, #1
	mov	r0, #0
	bl	rk_ftl_garbage_collect
	b	.L1655
.L1675:
	mov	r2, #0
	sub	r1, fp, #44
	mov	r0, r6
	bl	log2phys
	ldr	r2, [fp, #-44]
	cmn	r2, #1
	moveq	r10, #0
	ldreq	r2, .L1692+8
	beq	.L1660
	ldr	r1, [r5, #3288]
	mov	r10, #20
	ldr	r3, [fp, #-64]
	mla	r10, r10, r7, r1
	cmp	r6, r3
	str	r2, [r10, #4]
	ldr	r2, .L1692+8
	bne	.L1664
	ldrh	r2, [r2]
	mov	r0, r4
	ldr	r1, [r5, #3312]
	str	r2, [fp, #-76]
	str	r1, [r10, #8]
	mov	r1, r2
	bl	__umodsi3
	ldr	r2, [fp, #-76]
	ldr	r3, [fp, #-52]
	str	r0, [fp, #-72]
	sub	r1, r2, r0
	cmp	r3, r1
	movcs	r3, r1
	cmp	r3, r2
	str	r3, [fp, #-76]
	streq	r9, [r10, #8]
.L1665:
	ldr	r3, .L1692+12
	ldr	r1, [r5, #3336]
	str	r6, [r10, #16]
	ldrh	r2, [r3]
	mul	r2, r7, r2
	add	r7, r7, #1
	bic	r2, r2, #3
	add	r2, r1, r2
	str	r2, [r10, #12]
	b	.L1663
.L1662:
	mla	r0, r0, r6, r10
	ldr	r3, [fp, #-60]
	cmp	r4, r0
	movls	r1, #1
	movhi	r1, #0
	cmp	r3, r0
	movls	r1, #0
	cmp	r1, #0
	beq	.L1661
	sub	r0, r0, r4
	mov	r1, #512
	add	r0, r9, r0, lsl #9
	str	r2, [fp, #-80]
	bl	__memzero
	ldr	r2, [fp, #-80]
.L1661:
	add	r10, r10, #1
.L1660:
	ldrh	r0, [r2]
	cmp	r10, r0
	bcc	.L1662
.L1663:
	ldr	r3, [fp, #-56]
	add	r6, r6, #1
	subs	r3, r3, #1
	str	r3, [fp, #-56]
	beq	.L1667
	ldrh	r2, [r5, #232]
	cmp	r7, r2, lsl #2
	bne	.L1658
.L1667:
	cmp	r7, #0
	beq	.L1658
	mov	r2, #0
	mov	r1, r7
	ldr	r0, [r5, #3288]
	mov	r10, #0
	bl	FlashReadPages
	lsl	r3, r8, #9
	str	r3, [fp, #-88]
	ldr	r3, [fp, #-72]
	lsl	r3, r3, #9
	str	r3, [fp, #-80]
	ldr	r3, [fp, #-76]
	lsl	r3, r3, #9
	str	r3, [fp, #-84]
.L1674:
	mov	ip, #20
	ldr	r2, [r5, #3288]
	mul	ip, ip, r10
	ldr	r3, [fp, #-64]
	add	r2, r2, ip
	ldr	r1, [r2, #16]
	cmp	r3, r1
	bne	.L1669
	ldr	r1, [r2, #8]
	ldr	r2, [r5, #3312]
	cmp	r1, r2
	bne	.L1670
	ldr	r3, [fp, #-80]
	mov	r0, r9
	ldr	r2, [fp, #-84]
	str	ip, [fp, #-92]
	add	r1, r1, r3
.L1691:
	bl	ftl_memcpy
	ldr	ip, [fp, #-92]
.L1670:
	ldr	r1, [r5, #3288]
	add	r2, r1, ip
	ldr	r0, [r2, #12]
	ldr	lr, [r2, #16]
	ldr	r0, [r0, #8]
	cmp	lr, r0
	ldrne	r0, [r5, #2712]
	addne	r0, r0, #1
	strne	r0, [r5, #2712]
	ldr	lr, [r1, ip]
	cmn	lr, #1
	ldreq	r0, [r5, #2712]
	streq	lr, [fp, #-48]
	addeq	r0, r0, #1
	streq	r0, [r5, #2712]
	ldr	r1, [r1, ip]
	cmp	r1, #256
	bne	.L1673
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L1673:
	add	r10, r10, #1
	cmp	r7, r10
	bne	.L1674
	mov	r7, #0
	b	.L1658
.L1664:
	ldr	r3, [fp, #-68]
	cmp	r6, r3
	ldrhne	r2, [r2]
	mulne	r2, r6, r2
	bne	.L1690
	ldr	r1, [r5, #3316]
	ldr	r3, [fp, #-60]
	str	r1, [r10, #8]
	ldrh	r1, [r2]
	mul	r2, r1, r6
	sub	r8, r3, r2
	cmp	r1, r8
	bne	.L1665
.L1690:
	sub	r2, r2, r4
	add	r2, r9, r2, lsl #9
	str	r2, [r10, #8]
	b	.L1665
.L1669:
	ldr	r3, [fp, #-68]
	cmp	r3, r1
	bne	.L1670
	ldr	r1, [r2, #8]
	ldr	r2, [r5, #3316]
	cmp	r1, r2
	bne	.L1670
	ldr	r2, .L1692+8
	str	ip, [fp, #-92]
	ldrh	r0, [r2]
	ldr	r2, [fp, #-88]
	mul	r0, r3, r0
	sub	r0, r0, r4
	add	r0, r9, r0, lsl #9
	b	.L1691
.L1678:
	mvn	r3, #0
	str	r3, [fp, #-48]
	b	.L1655
.L1693:
	.align	2
.L1692:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+258
	.word	.LANCHOR0+312
	.size	FtlRead, .-FtlRead
	.align	2
	.global	sftl_read
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_read, %function
sftl_read:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r3, r2
	mov	r2, r1
	mov	r1, r0
	mov	r0, #0
	bl	FtlRead
	ldmfd	sp, {fp, sp, pc}
	.size	sftl_read, .-sftl_read
	.align	2
	.global	sftl_vendor_read
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_vendor_read, %function
sftl_vendor_read:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r3, r2
	mov	r2, r1
	mov	r1, r0
	mov	r0, #16
	bl	FtlRead
	ldmfd	sp, {fp, sp, pc}
	.size	sftl_vendor_read, .-sftl_vendor_read
	.align	2
	.global	FtlWrite
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlWrite, %function
FtlWrite:
	@ args = 0, pretend = 0, frame = 68
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #68
	uxtb	r0, r0
	mov	r9, r1
	str	r2, [fp, #-80]
	cmp	r0, #16
	str	r3, [fp, #-84]
	bne	.L1697
	mov	r2, r3
	ldr	r1, [fp, #-80]
	add	r0, r9, #256
	bl	FtlVendorPartWrite
.L1696:
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1697:
	ldr	r4, .L1749
	ldr	r2, [fp, #-80]
	ldr	r3, [r4, #340]
	cmp	r2, r3
	cmpls	r1, r3
	bcs	.L1734
	add	r6, r1, r2
	cmp	r3, r6
	bcc	.L1734
	ldr	r5, .L1749+4
	ldr	r0, [r5]
	cmn	r0, #1
	beq	.L1696
	mov	r3, #2048
	mov	r0, r9
	str	r3, [r4, #3668]
	movw	r3, #258
	ldrh	r7, [r4, r3]
	mov	r1, r7
	bl	__udivsi3
	mov	r1, r7
	str	r0, [fp, #-76]
	sub	r0, r6, #1
	bl	__udivsi3
	ldr	r2, [fp, #-76]
	str	r0, [fp, #-92]
	sub	r3, r0, r2
	ldr	r2, [fp, #-80]
	str	r3, [fp, #-96]
	add	r3, r3, #1
	str	r3, [fp, #-72]
	ldr	r3, [r4, #2580]
	add	r3, r3, r2
	ldr	r2, [fp, #-72]
	str	r3, [r4, #2580]
	ldr	r3, [r4, #2564]
	add	r3, r3, r2
	str	r3, [r4, #2564]
	ldr	r3, [r5, #8]
	cmp	r3, #0
	addeq	r4, r4, #24
	beq	.L1699
	ldrh	r3, [r4, #28]
	cmp	r3, #0
	addne	r4, r4, #24
	addeq	r4, r4, #76
.L1699:
	ldr	r6, [fp, #-76]
	ldr	r10, .L1749
.L1700:
	ldr	r3, [fp, #-72]
	cmp	r3, #0
	bne	.L1728
	mov	r0, r3
	ldr	r1, [fp, #-96]
	bl	rk_ftl_garbage_collect
	ldrh	r3, [r10, #224]
	cmp	r3, #5
	movls	r4, #256
	ldrls	r5, .L1749+8
	bls	.L1732
.L1731:
	mov	r0, #0
	b	.L1696
.L1728:
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r10, #232]
	cmp	r2, r3
	bcc	.L1701
	movw	r2, #1041
	ldr	r1, .L1749+12
	ldr	r0, .L1749+16
	bl	sftl_printk
.L1701:
	ldrh	r2, [r4, #4]
	cmp	r2, #0
	bne	.L1702
	ldr	r3, .L1749+20
	ldr	r5, .L1749+4
	cmp	r4, r3
	bne	.L1703
	ldrh	r7, [r10, #80]
	cmp	r7, #0
	bne	.L1704
	add	r0, r4, #52
	bl	allocate_new_data_superblock
	str	r7, [r5, #8]
.L1704:
	ldr	r0, .L1749+20
	bl	allocate_new_data_superblock
	ldr	r4, .L1749+20
	ldr	r2, [r5, #8]
	add	r3, r4, #52
	cmp	r2, #0
	movne	r4, r3
.L1705:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	bne	.L1702
	mov	r0, r4
	bl	allocate_new_data_superblock
.L1702:
	ldrb	r2, [r4, #7]	@ zero_extendqisi2
	ldrh	r3, [r4, #4]
	ldr	r1, [fp, #-72]
	lsl	r2, r2, #2
	cmp	r3, r1
	movcs	r3, r1
	cmp	r2, r3
	movcc	r3, r2
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	str	r3, [fp, #-108]
	ldrh	r3, [r10, #232]
	cmp	r2, r3
	bcc	.L1706
	movw	r2, #1074
	ldr	r1, .L1749+12
	ldr	r0, .L1749+16
	bl	sftl_printk
.L1706:
	mov	r3, #0
.L1748:
	str	r3, [fp, #-68]
	ldr	r3, [fp, #-68]
	ldr	r2, [fp, #-108]
	cmp	r3, r2
	bne	.L1726
.L1708:
	mov	r3, r4
	mov	r2, #0
	ldr	r1, [fp, #-68]
	ldr	r0, [r10, #3296]
	bl	FtlProgPages
	ldr	r3, [fp, #-68]
	ldr	r2, [fp, #-72]
	cmp	r3, r2
	bls	.L1727
	mov	r2, #1152
	ldr	r1, .L1749+12
	ldr	r0, .L1749+16
	bl	sftl_printk
.L1727:
	ldr	r3, [fp, #-72]
	ldr	r2, [fp, #-68]
	sub	r3, r3, r2
	str	r3, [fp, #-72]
	b	.L1700
.L1703:
	str	r2, [r5, #8]
	ldrh	r2, [r10, #28]
	cmp	r2, #0
	movne	r4, r3
	bne	.L1702
	mov	r0, r4
	bl	allocate_new_data_superblock
	b	.L1705
.L1726:
	ldrh	r3, [r4, #4]
	cmp	r3, #0
	beq	.L1708
	mov	r2, #0
	sub	r1, fp, #64
	mov	r0, r6
	mov	r7, #20
	bl	log2phys
	mov	r0, r4
	bl	get_new_active_ppa
	ldr	r2, .L1749+24
	ldr	r1, [fp, #-68]
	ldr	r3, [fp, #-68]
	ldrh	r2, [r2]
	mul	r7, r7, r3
	ldr	r3, [r10, #3296]
	mul	r1, r2, r1
	add	r3, r3, r7
	str	r0, [r3, #4]
	bic	r1, r1, #3
	str	r6, [r3, #16]
	str	r1, [fp, #-100]
	ldr	r0, [fp, #-100]
	ldr	r1, [r10, #3336]
	add	r8, r1, r0
	str	r1, [fp, #-104]
	str	r8, [r3, #12]
	mov	r1, #0
	mov	r0, r8
	bl	ftl_memset
	ldr	r3, [fp, #-76]
	ldr	r2, [fp, #-92]
	cmp	r6, r2
	cmpne	r6, r3
	ldr	r3, .L1749+28
	bne	.L1709
	ldr	r2, [fp, #-76]
	cmp	r6, r2
	bne	.L1710
	ldrh	r5, [r3]
	mov	r0, r9
	mov	r1, r5
	bl	__umodsi3
	ldr	r3, [fp, #-80]
	sub	r5, r5, r0
	str	r0, [fp, #-88]
	cmp	r5, r3
	movcs	r5, r3
.L1711:
	ldr	r3, .L1749+28
	ldrh	r3, [r3]
	cmp	r5, r3
	ldr	r3, [fp, #-76]
	bne	.L1712
	cmp	r6, r3
	ldr	r3, [r10, #3296]
	add	r7, r3, r7
	ldreq	r3, [fp, #-84]
	beq	.L1745
	mul	r5, r5, r6
	ldr	r3, [fp, #-84]
	sub	r5, r5, r9
	add	r5, r3, r5, lsl #9
	str	r5, [r7, #8]
.L1714:
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	ldrh	r3, [r10, #232]
	cmp	r2, r3
	bcc	.L1723
	movw	r2, #1143
	ldr	r1, .L1749+12
	ldr	r0, .L1749+16
	bl	sftl_printk
.L1723:
	ldr	r3, .L1749+32
	ldr	r2, [fp, #-104]
	ldr	r1, [fp, #-100]
	strh	r3, [r2, r1]	@ movhi
	ldr	r3, [r10, #2592]
	str	r6, [r8, #8]
	add	r6, r6, #1
	str	r3, [r8, #4]
	add	r3, r3, #1
	cmn	r3, #1
	moveq	r3, #0
	str	r3, [r10, #2592]
	ldr	r3, [fp, #-64]
	str	r3, [r8, #12]
	ldrh	r3, [r4]
	strh	r3, [r8, #2]	@ movhi
	ldr	r3, [fp, #-68]
	add	r3, r3, #1
	b	.L1748
.L1710:
	ldr	r2, [fp, #-80]
	add	r5, r9, r2
	ldrh	r2, [r3]
	mov	r3, #0
	str	r3, [fp, #-88]
	smulbb	r2, r2, r6
	sub	r5, r5, r2
	uxth	r5, r5
	b	.L1711
.L1712:
	cmp	r6, r3
	ldr	r3, [r10, #3296]
	ldreq	r2, [r10, #3312]
	ldrne	r2, [r10, #3316]
	add	r3, r3, r7
	str	r2, [r3, #8]
	ldr	r3, [fp, #-64]
	cmn	r3, #1
	beq	.L1717
	str	r3, [fp, #-56]
	mov	r1, #1
	ldr	r3, [r10, #3296]
	sub	r0, fp, #60
	str	r6, [fp, #-44]
	add	r3, r3, r7
	ldr	r2, [r3, #8]
	ldr	r3, [r3, #12]
	str	r2, [fp, #-52]
	mov	r2, #0
	str	r3, [fp, #-48]
	bl	FlashReadPages
	ldr	r3, [fp, #-60]
	cmn	r3, #1
	bne	.L1718
	ldr	r2, [r10, #2712]
	ldr	r0, .L1749+36
	add	r2, r2, #1
	str	r2, [r10, #2712]
	mov	r2, r6
	ldr	r1, [r8, #8]
	bl	sftl_printk
.L1721:
	ldr	r3, [fp, #-76]
	lsl	r2, r5, #9
	cmp	r6, r3
	bne	.L1722
	ldr	r3, [r10, #3296]
	ldr	r1, [fp, #-84]
	add	r7, r3, r7
	ldr	r3, [fp, #-88]
	ldr	r0, [r7, #8]
	add	r0, r0, r3, lsl #9
.L1746:
	bl	ftl_memcpy
	b	.L1714
.L1718:
	ldr	r3, [r8, #8]
	cmp	r6, r3
	beq	.L1720
	ldr	r3, [r10, #2712]
	mov	r2, r6
	ldr	r0, .L1749+40
	add	r3, r3, #1
	str	r3, [r10, #2712]
	ldr	r1, [r8, #8]
	bl	sftl_printk
.L1720:
	ldr	r3, [r8, #8]
	cmp	r6, r3
	beq	.L1721
	movw	r2, #1128
	ldr	r1, .L1749+12
	ldr	r0, .L1749+16
	bl	sftl_printk
	b	.L1721
.L1717:
	ldr	r3, [r10, #3296]
	mov	r1, #0
	ldr	r2, .L1749+44
	add	r3, r3, r7
	ldrh	r2, [r2]
	ldr	r0, [r3, #8]
	bl	ftl_memset
	b	.L1721
.L1722:
	ldr	r3, .L1749+28
	ldrh	r1, [r3]
	ldr	r3, [r10, #3296]
	mul	r1, r6, r1
	add	r7, r3, r7
	ldr	r3, [fp, #-84]
	ldr	r0, [r7, #8]
	sub	r1, r1, r9
	add	r1, r3, r1, lsl #9
	b	.L1746
.L1709:
	ldrh	r3, [r3]
	ldr	r2, [r10, #3296]
	mul	r3, r6, r3
	add	r7, r2, r7
	ldr	r2, [fp, #-84]
	sub	r3, r3, r9
	add	r3, r2, r3, lsl #9
.L1745:
	str	r3, [r7, #8]
	b	.L1714
.L1732:
	ldrh	r3, [r10, #172]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1730
	ldrh	r2, [r10, #222]
	cmp	r2, r3
	bne	.L1730
	mov	r0, #0
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L1730:
	ldr	r2, .L1749+48
	mov	r1, #1
	mov	r3, #128
	mov	r0, r1
	strh	r3, [r5]	@ movhi
	strh	r3, [r2]	@ movhi
	bl	rk_ftl_garbage_collect
	mov	r1, #1
	mov	r0, #0
	bl	rk_ftl_garbage_collect
	ldrh	r3, [r10, #224]
	cmp	r3, #2
	bhi	.L1731
	subs	r4, r4, #1
	bne	.L1732
	b	.L1731
.L1734:
	mvn	r0, #0
	b	.L1696
.L1750:
	.align	2
.L1749:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+3154
	.word	.LANCHOR1+591
	.word	.LC8
	.word	.LANCHOR0+24
	.word	.LANCHOR0+312
	.word	.LANCHOR0+258
	.word	-3947
	.word	.LC125
	.word	.LC126
	.word	.LANCHOR0+310
	.word	.LANCHOR0+3152
	.size	FtlWrite, .-FtlWrite
	.align	2
	.global	sftl_vendor_write
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_vendor_write, %function
sftl_vendor_write:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r3, r2
	mov	r2, r1
	mov	r1, r0
	mov	r0, #16
	bl	FtlWrite
	ldmfd	sp, {fp, sp, pc}
	.size	sftl_vendor_write, .-sftl_vendor_write
	.align	2
	.global	sftl_gc
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_gc, %function
sftl_gc:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r1, #1
	mov	r0, r1
	bl	rk_ftl_garbage_collect
	ldmfd	sp, {fp, sp, pc}
	.size	sftl_gc, .-sftl_gc
	.align	2
	.global	FtlLoadSysInfo
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	@ args = 0, pretend = 0, frame = 4
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #16
	mov	r1, #0
	ldr	r4, .L1783
	ldr	r3, [r4, #3300]
	add	r6, r4, #2624
	ldrh	r2, [r4, #240]
	ldr	r0, [r4, #72]
	str	r3, [r4, #3460]
	ldr	r3, [r4, #3332]
	lsl	r2, r2, #1
	str	r3, [r4, #3464]
	bl	ftl_memset
	ldrh	r0, [r6]
	movw	r3, #65535
	cmp	r0, r3
	bne	.L1754
.L1765:
	mvn	r0, #0
.L1753:
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1754:
	mov	r1, #1
	ldr	r8, .L1783+4
	bl	FtlGetLastWrittenPage
	ldr	r9, .L1783+8
	sxth	r5, r0
	ldrsh	r7, [r6]
	add	r0, r0, #1
	strh	r0, [r6, #2]	@ movhi
.L1756:
	cmp	r5, #0
	bge	.L1762
	movw	r2, #1467
	ldr	r1, .L1783+12
	ldr	r0, .L1783+16
	bl	sftl_printk
.L1761:
	movw	r2, #310
	ldrh	r3, [r4, #240]
	ldrh	r2, [r4, r2]
	add	r3, r3, #24
	cmp	r2, r3, lsl #1
	bcs	.L1764
	movw	r2, #1469
	ldr	r1, .L1783+12
	ldr	r0, .L1783+16
	bl	sftl_printk
.L1764:
	ldr	r5, .L1783+20
	mov	r2, #48
	ldr	r1, [r4, #3460]
	mov	r0, r5
	bl	ftl_memcpy
	ldrh	r2, [r4, #240]
	ldr	r1, [r4, #3460]
	ldr	r0, [r4, #72]
	lsl	r2, r2, #1
	add	r1, r1, #48
	bl	ftl_memcpy
	ldr	r2, [r4, #2468]
	ldr	r3, .L1783+8
	cmp	r2, r3
	bne	.L1765
	ldrb	r2, [r4, #2478]	@ zero_extendqisi2
	ldrh	r3, [r4, #254]
	ldrh	r7, [r5, #8]
	cmp	r2, r3
	strh	r7, [r6, #6]	@ movhi
	bne	.L1765
	movw	r3, #302
	movw	r2, #258
	ldrh	r3, [r4, r3]
	ldrh	r2, [r4, r2]
	ldr	r6, [r4, #244]
	str	r7, [r4, #3672]
	mul	r3, r7, r3
	ldrh	r1, [r4, #232]
	str	r3, [r4, #2552]
	mul	r3, r3, r2
	str	r3, [r4, #340]
	ldr	r3, .L1783+24
	ldrh	r0, [r3, #6]
	sub	r0, r6, r0
	sub	r0, r0, r7
	bl	__udivsi3
	cmp	r7, r6
	movw	r3, #2620
	strh	r0, [r4, r3]	@ movhi
	bls	.L1766
	movw	r2, #1491
	ldr	r1, .L1783+12
	ldr	r0, .L1783+16
	bl	sftl_printk
.L1766:
	ldrh	r3, [r5, #16]
	ldrh	r1, [r5, #14]
	lsr	r2, r3, #6
	and	r3, r3, #63
	strb	r3, [r4, #30]
	strh	r2, [r4, #26]	@ movhi
	ldrh	r2, [r5, #18]
	ldrb	r3, [r4, #2479]	@ zero_extendqisi2
	strh	r1, [r4, #24]	@ movhi
	strh	r2, [r4, #76]	@ movhi
	ldrh	r2, [r5, #20]
	strb	r3, [r4, #32]
	mvn	r3, #0
	strh	r3, [r4, #172]	@ movhi
	mov	r3, #0
	strh	r3, [r4, #174]	@ movhi
	lsr	r0, r2, #6
	and	r2, r2, #63
	strb	r2, [r4, #82]
	ldrb	r2, [r4, #2480]	@ zero_extendqisi2
	strh	r0, [r4, #78]	@ movhi
	strb	r3, [r4, #178]
	strb	r2, [r4, #84]
	ldrh	r2, [r5, #22]
	strb	r3, [r4, #180]
	str	r3, [r4, #2576]
	strh	r2, [r4, #124]	@ movhi
	ldrh	r2, [r5, #24]
	str	r3, [r4, #2564]
	str	r3, [r4, #2556]
	str	r3, [r4, #2572]
	lsr	r0, r2, #6
	and	r2, r2, #63
	strb	r2, [r4, #130]
	ldrb	r2, [r4, #2481]	@ zero_extendqisi2
	str	r3, [r4, #2600]
	strh	r0, [r4, #126]	@ movhi
	strb	r2, [r4, #132]
	ldr	r2, [r4, #2500]
	str	r2, [r4, #2596]
	str	r3, [r4, #2612]
	ldr	r2, [r4, #2588]
	str	r3, [r4, #2568]
	ldr	r3, [r4, #2508]
	cmp	r3, r2
	ldr	r2, [r4, #2592]
	strhi	r3, [r4, #2588]
	ldr	r3, [r4, #2504]
	cmp	r3, r2
	strhi	r3, [r4, #2592]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1769
	ldr	r0, .L1783+28
	bl	make_superblock
.L1769:
	ldrh	r2, [r4, #76]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1770
	ldr	r0, .L1783+32
	bl	make_superblock
.L1770:
	ldrh	r2, [r4, #124]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1771
	ldr	r0, .L1783+36
	bl	make_superblock
.L1771:
	ldrh	r2, [r4, #172]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1772
	ldr	r0, .L1783+40
	bl	make_superblock
.L1772:
	mov	r0, #0
	b	.L1753
.L1762:
	orr	r3, r5, r7, lsl #10
	mov	r2, #1
	mov	r1, r2
	str	r3, [r4, #3456]
	mov	r0, r8
	ldr	r3, [r4, #3300]
	str	r3, [r4, #3460]
	bl	FlashReadPages
	ldr	r3, [r4, #3464]
	ldr	r3, [r3, #12]
	cmp	r3, #0
	str	r3, [fp, #-44]
	beq	.L1757
	ldr	r2, [r4, #3452]
	cmn	r2, #1
	beq	.L1757
	ldr	r10, .L1783+44
	ldr	r0, [r4, #3460]
	ldrh	r1, [r10]
	bl	js_hash
	ldr	r3, [fp, #-44]
	cmp	r3, r0
	beq	.L1757
	str	r0, [sp, #8]
	mov	r2, r7
	str	r3, [sp, #4]
	ldrh	r3, [r6, #4]
	ldr	r1, .L1783+12
	ldr	r0, .L1783+48
	str	r3, [sp]
	mov	r3, r5
	bl	sftl_printk
	cmp	r5, #0
	bne	.L1758
	ldrh	r3, [r6, #4]
	cmp	r7, r3
	sxthne	r7, r3
	ldrhne	r5, [r10, #-6]
	bne	.L1760
.L1758:
	mvn	r3, #0
	str	r3, [r4, #3452]
.L1757:
	ldr	r3, [r4, #3452]
	cmn	r3, #1
	beq	.L1760
	ldr	r3, [r4, #3300]
	ldr	r3, [r3]
	cmp	r3, r9
	bne	.L1760
	ldr	r3, [r4, #3332]
	ldrh	r2, [r3]
	movw	r3, #61604
	cmp	r2, r3
	beq	.L1761
.L1760:
	sub	r5, r5, #1
	sxth	r5, r5
	b	.L1756
.L1784:
	.align	2
.L1783:
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.word	1179929683
	.word	.LANCHOR1+600
	.word	.LC8
	.word	.LANCHOR0+2468
	.word	.LANCHOR0+352
	.word	.LANCHOR0+24
	.word	.LANCHOR0+76
	.word	.LANCHOR0+124
	.word	.LANCHOR0+172
	.word	.LANCHOR0+310
	.word	.LC127
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlMapTblRecovery
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	@ args = 0, pretend = 0, frame = 28
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #36
	mov	r1, #0
	ldr	r3, [r0, #24]
	mov	r4, r0
	ldr	r9, .L1826
	mov	r7, #0
	ldr	r8, [r0, #12]
	str	r3, [fp, #-44]
	ldr	r3, [r0, #16]
	str	r3, [fp, #-56]
	ldrh	r3, [r0, #6]
	str	r3, [fp, #-48]
	ldrh	r3, [r0, #8]
	ldr	r0, [fp, #-44]
	str	r3, [fp, #-52]
	ldr	r3, [fp, #-48]
	lsl	r2, r3, #2
	bl	ftl_memset
	ldr	r3, [r9, #3300]
	ldr	r5, [r9, #3332]
	str	r7, [r4, #32]
	str	r3, [r9, #3460]
	mvn	r3, #0
	str	r5, [r9, #3464]
	strh	r3, [r4]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	mov	r3, #1
	str	r7, [r4, #28]
	str	r3, [r4, #36]
.L1786:
	ldr	r3, [fp, #-52]
	sxth	r6, r7
	cmp	r6, r3
	bge	.L1805
	ldr	r3, [fp, #-52]
	sub	r3, r3, #1
	cmp	r6, r3
	lsl	r3, r6, #1
	bne	.L1787
	add	r2, r8, r3
	ldrh	r0, [r8, r3]
	mov	r1, #1
	str	r2, [fp, #-52]
	bl	FtlGetLastWrittenPage
	sxth	r3, r0
	ldr	r9, .L1826
	add	r0, r0, #1
	ldr	r10, .L1826+4
	str	r3, [fp, #-60]
	ldr	r3, [fp, #-56]
	strh	r7, [r4]	@ movhi
	mov	r7, #0
	strh	r0, [r4, #2]	@ movhi
	ldr	r3, [r3, r6, lsl #2]
	str	r3, [r4, #28]
.L1788:
	ldr	r3, [fp, #-60]
	sxth	r8, r7
	add	r2, r3, #1
	cmp	r8, r2
	blt	.L1791
.L1805:
	mov	r0, r4
	bl	ftl_free_no_use_map_blk
	ldr	r3, .L1826+8
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3]
	cmp	r2, r3
	bne	.L1793
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1793:
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, r4
	bl	ftl_map_blk_gc
	mov	r0, #0
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1791:
	ldr	r3, [fp, #-52]
	mov	r0, r10
	ldrh	r2, [r3]
	orr	r2, r8, r2, lsl #10
	str	r2, [r9, #3456]
	mov	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r2, [r9, #3464]
	ldr	r2, [r2, #12]
	cmp	r2, #0
	str	r2, [fp, #-56]
	beq	.L1789
	ldr	r1, [r9, #3452]
	cmn	r1, #1
	beq	.L1789
	ldr	r1, .L1826+12
	ldr	r0, [r9, #3460]
	ldrh	r1, [r1]
	bl	js_hash
	ldr	r2, [fp, #-56]
	cmp	r2, r0
	beq	.L1789
	str	r0, [sp, #4]
	mov	r3, r8
	str	r2, [sp]
	mov	r2, r6
	ldr	r1, .L1826+16
	ldr	r0, .L1826+20
	bl	sftl_printk
	mvn	r3, #0
	str	r3, [r9, #3452]
.L1789:
	ldr	r3, [r9, #3452]
	cmn	r3, #1
	beq	.L1790
	ldrh	r3, [r5, #8]
	ldr	r2, [fp, #-48]
	cmp	r2, r3
	bls	.L1790
	ldrh	r1, [r5]
	ldrh	r2, [r4, #4]
	cmp	r1, r2
	ldreq	r2, [r9, #3456]
	ldreq	r1, [fp, #-44]
	streq	r2, [r1, r3, lsl #2]
.L1790:
	add	r7, r7, #1
	b	.L1788
.L1787:
	ldr	r2, [r9, #3300]
	ldr	r0, .L1826+4
	str	r2, [r9, #3460]
	add	r2, r8, r3
	str	r2, [fp, #-60]
	ldrh	r2, [r8, r3]
	ldr	r3, .L1826+8
	ldrh	r3, [r3]
	sub	r3, r3, #1
	orr	r3, r3, r2, lsl #10
	mov	r2, #1
	mov	r1, r2
	str	r3, [r9, #3456]
	bl	FlashReadPages
	ldr	r3, [r9, #3452]
	cmn	r3, #1
	beq	.L1807
	ldrh	r2, [r5]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L1807
	ldrh	r2, [r5, #8]
	movw	r3, #64245
	cmp	r2, r3
	beq	.L1795
.L1807:
	mov	r10, #0
.L1796:
	ldr	r2, .L1826+8
	sxth	r3, r10
	ldrh	r2, [r2]
	cmp	r3, r2
	bge	.L1803
	ldr	r2, [fp, #-60]
	ldr	r0, .L1826+4
	str	r3, [fp, #-68]
	ldrh	r2, [r2]
	orr	r2, r3, r2, lsl #10
	str	r2, [r9, #3456]
	mov	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r2, [r9, #3464]
	ldr	r2, [r2, #12]
	cmp	r2, #0
	str	r2, [fp, #-64]
	beq	.L1800
	ldr	r1, [r9, #3452]
	cmn	r1, #1
	beq	.L1800
	ldr	r1, .L1826+12
	ldr	r0, [r9, #3460]
	ldrh	r1, [r1]
	bl	js_hash
	ldr	r2, [fp, #-64]
	cmp	r2, r0
	beq	.L1800
	str	r0, [sp, #4]
	str	r2, [sp]
	mov	r2, r6
	ldr	r3, [fp, #-68]
	ldr	r1, .L1826+16
	ldr	r0, .L1826+24
	bl	sftl_printk
	mvn	r3, #0
	str	r3, [r9, #3452]
.L1800:
	ldr	r3, [r9, #3452]
	cmn	r3, #1
	beq	.L1801
	ldrh	r3, [r5, #8]
	ldr	r2, [fp, #-48]
	cmp	r2, r3
	bls	.L1801
	ldrh	r1, [r5]
	ldrh	r2, [r4, #4]
	cmp	r1, r2
	ldreq	r2, [r9, #3456]
	ldreq	r1, [fp, #-44]
	streq	r2, [r1, r3, lsl #2]
.L1801:
	add	r10, r10, #1
	b	.L1796
.L1795:
	ldr	ip, .L1826+8
	mov	r1, #0
	mov	lr, #4
.L1797:
	ldrh	r2, [ip]
	sxth	r3, r1
	sub	r2, r2, #1
	cmp	r3, r2
	blt	.L1799
.L1803:
	add	r7, r7, #1
	b	.L1786
.L1799:
	ldr	r0, [r9, #3300]
	add	r1, r1, #1
	ldr	r10, [fp, #-48]
	ldr	r2, [r0, r3, lsl #3]
	uxth	r6, r2
	cmp	r10, r6
	addhi	r3, lr, r3, lsl #3
	movhi	r2, r6
	ldrhi	r3, [r0, r3]
	ldrhi	r0, [fp, #-44]
	strhi	r3, [r0, r2, lsl #2]
	b	.L1797
.L1827:
	.align	2
.L1826:
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.word	.LANCHOR0+304
	.word	.LANCHOR0+310
	.word	.LANCHOR1+615
	.word	.LC128
	.word	.LC129
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	2
	.global	FtlLoadVonderInfo
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L1829
	add	r2, r3, #320
	add	r0, r3, #3600
	ldrh	r2, [r2]
	add	r0, r0, #8
	strh	r2, [r0, #10]	@ movhi
	ldr	r2, .L1829+4
	strh	r2, [r0, #4]	@ movhi
	add	r2, r3, #344
	ldrh	r2, [r2]
	strh	r2, [r0, #8]	@ movhi
	movw	r2, #322
	ldrh	r2, [r3, r2]
	strh	r2, [r0, #6]	@ movhi
	ldr	r2, [r3, #348]
	str	r2, [r3, #3620]
	ldr	r2, [r3, #3368]
	str	r2, [r3, #3624]
	ldr	r2, [r3, #3364]
	str	r2, [r3, #3628]
	ldr	r2, [r3, #3372]
	str	r2, [r3, #3632]
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L1830:
	.align	2
.L1829:
	.word	.LANCHOR0
	.word	-3962
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FtlLoadMapInfo
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlLoadMapInfo, %function
FtlLoadMapInfo:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	bl	FtlL2PDataInit
	ldr	r0, .L1832
	bl	FtlMapTblRecovery
	mov	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L1833:
	.align	2
.L1832:
	.word	.LANCHOR0+3392
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	FtlSysBlkInit
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1850
	mov	r3, #0
	add	r7, r4, #3472
	ldrh	r0, [r4, #236]
	strh	r3, [r7]	@ movhi
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	add	r3, r4, #2624
	ldrh	r2, [r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1835
.L1837:
	mvn	r6, #0
.L1834:
	mov	r0, r6
	ldmfd	sp, {r4, r5, r6, r7, fp, sp, pc}
.L1835:
	bl	FtlLoadSysInfo
	subs	r6, r0, #0
	bne	.L1837
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	r0, #1
	bl	FtlUpdateVaildLpn
	ldr	r2, [r4, #2540]
	movw	r3, #338
	ldrh	r1, [r4, r3]
	mov	r0, #12
	mov	r3, r6
.L1838:
	cmp	r3, r1
	bge	.L1843
	mla	ip, r0, r3, r2
	ldr	ip, [ip, #4]
	cmp	ip, #0
	bge	.L1839
.L1843:
	ldr	r5, .L1850+4
	cmp	r3, r1
	ldrh	r2, [r5, #28]
	add	r2, r2, #1
	strh	r2, [r5, #28]	@ movhi
	bge	.L1849
.L1840:
	ldr	r0, .L1850+8
	bl	FtlSuperblockPowerLostFix
	ldr	r0, .L1850+12
	bl	FtlSuperblockPowerLostFix
	ldrh	r3, [r4, #24]
	ldr	r1, [r4, #72]
	ldrh	r0, [r4, #28]
	lsl	r3, r3, #1
	ldrh	r2, [r1, r3]
	sub	r2, r2, r0
	movw	r0, #302
	strh	r2, [r1, r3]	@ movhi
	ldrh	r2, [r4, #76]
	ldrh	r3, [r4, r0]
	ldr	ip, [r4, #72]
	ldrh	lr, [r4, #80]
	lsl	r2, r2, #1
	strh	r3, [r4, #26]	@ movhi
	mov	r3, #0
	strb	r3, [r4, #30]
	strh	r3, [r4, #28]	@ movhi
	ldrh	r1, [ip, r2]
	sub	r1, r1, lr
	strh	r1, [ip, r2]	@ movhi
	strb	r3, [r4, #82]
	strh	r3, [r4, #80]	@ movhi
	ldrh	r3, [r5, #30]
	ldrh	r2, [r4, r0]
	add	r3, r3, #1
	strh	r2, [r4, #78]	@ movhi
	strh	r3, [r5, #30]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
	b	.L1844
.L1839:
	add	r3, r3, #1
	b	.L1838
.L1849:
	ldrh	r3, [r7]
	cmp	r3, #0
	bne	.L1840
.L1844:
	ldrh	r0, [r4, #24]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1845
	ldrh	r3, [r4, #28]
	cmp	r3, #0
	bne	.L1845
	ldrh	r3, [r4, #80]
	cmp	r3, #0
	bne	.L1845
	bl	FtlGcRefreshBlock
	ldrh	r0, [r4, #76]
	bl	FtlGcRefreshBlock
	bl	FtlVpcTblFlush
	ldr	r0, .L1850+8
	bl	allocate_new_data_superblock
	ldr	r0, .L1850+12
	bl	allocate_new_data_superblock
.L1845:
	bl	FtlVpcCheckAndModify
	b	.L1834
.L1851:
	.align	2
.L1850:
	.word	.LANCHOR0
	.word	.LANCHOR0+2468
	.word	.LANCHOR0+24
	.word	.LANCHOR0+76
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	ftl_low_format
	.syntax unified
	.arm
	.fpu softvfp
	.type	ftl_low_format, %function
ftl_low_format:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1876
	mov	r3, #0
	ldrh	r0, [r4, #236]
	str	r3, [r4, #2588]
	str	r3, [r4, #2592]
	str	r3, [r4, #2604]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	beq	.L1853
	bl	FtlMakeBbt
.L1853:
	ldr	r0, .L1876+4
	mov	r2, #0
	ldr	ip, .L1876+8
.L1854:
	ldrh	r1, [r0]
	uxth	r3, r2
	add	r2, r2, #1
	cmp	r3, r1, lsl #7
	blt	.L1855
	ldrh	r6, [r4, #240]
	mov	r5, #0
.L1856:
	ldrh	r3, [r4, #242]
	cmp	r3, r6
	bhi	.L1857
	ldrh	r1, [r4, #232]
	sub	r3, r5, #3
	cmp	r3, r1, lsl #1
	bge	.L1858
.L1862:
	mov	r0, #0
	mov	r6, r0
.L1859:
	ldrh	r2, [r4, #240]
	uxth	r3, r0
	add	r5, r0, #1
	cmp	r2, r3
	bhi	.L1863
	ldrh	r3, [r4, #242]
	movw	r9, #2620
	ldrh	r5, [r4, #232]
	ldr	r10, [r4, #244]
	str	r3, [r4, #3284]
	mov	r1, r5
	mov	r0, r10
	bl	__udivsi3
	ubfx	r7, r0, #5, #16
	mov	r8, r0
	add	r3, r7, #36
	str	r0, [r4, #2552]
	strh	r3, [r4, r9]	@ movhi
	mov	r3, #24
	mul	r3, r3, r5
	cmp	r6, r3
	ble	.L1864
	mov	r1, r5
	sub	r0, r10, r6
	bl	__udivsi3
	str	r0, [r4, #2552]
	lsr	r0, r0, #5
	add	r0, r0, #24
	strh	r0, [r4, r9]	@ movhi
.L1864:
	movw	r3, #294
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	beq	.L1866
	movw	r2, #2620
	ldrh	r1, [r4, r2]
	add	r1, r1, r3, lsr #1
	strh	r1, [r4, r2]	@ movhi
	mul	r1, r5, r3
	cmp	r6, r1
	addlt	r3, r3, #32
	strlt	r8, [r4, #2552]
	addlt	r3, r7, r3
	strhlt	r3, [r4, r2]	@ movhi
.L1866:
	movw	r3, #2620
	ldr	r6, .L1876+12
	ldrh	r2, [r4, r3]
	ldr	r3, [r4, #2552]
	sub	r3, r3, r2
	mul	r5, r5, r3
	movw	r3, #302
	ldrh	r3, [r4, r3]
	str	r5, [r4, #3672]
	mul	r5, r5, r3
	movw	r3, #258
	ldrh	r3, [r4, r3]
	str	r5, [r4, #2552]
	mul	r5, r5, r3
	str	r5, [r4, #340]
	bl	FtlBbmTblFlush
	ldrh	r2, [r4, #242]
	mov	r1, #0
	ldr	r0, [r4, #72]
	lsl	r2, r2, #1
	bl	ftl_memset
	mvn	r2, #0
	mov	r3, #0
	mov	r5, r2
	str	r3, [r4, #2548]
	strh	r3, [r4, #174]	@ movhi
	strb	r3, [r4, #178]
	strb	r3, [r4, #180]
	strh	r3, [r4, #26]	@ movhi
	strb	r3, [r4, #30]
	strh	r3, [r4, #24]	@ movhi
	mov	r3, #1
	strh	r2, [r4, #172]	@ movhi
	strb	r3, [r4, #32]
.L1868:
	mov	r0, r6
	bl	make_superblock
	ldrb	r3, [r4, #31]	@ zero_extendqisi2
	cmp	r3, #0
	ldrh	r3, [r4, #24]
	bne	.L1869
	ldr	r2, [r4, #72]
	lsl	r3, r3, #1
	strh	r5, [r2, r3]	@ movhi
	ldrh	r3, [r4, #24]
	add	r3, r3, #1
	strh	r3, [r4, #24]	@ movhi
	b	.L1868
.L1855:
	ldr	lr, [r4, #3312]
	mvn	r1, r3
	orr	r1, r3, r1, lsl #16
	str	r1, [lr, r3, lsl #2]
	ldr	r1, [r4, #3316]
	str	ip, [r1, r3, lsl #2]
	b	.L1854
.L1857:
	mov	r0, r6
	mov	r1, #1
	bl	FtlLowFormatEraseBlock
	add	r6, r6, #1
	add	r5, r5, r0
	uxth	r5, r5
	uxth	r6, r6
	b	.L1856
.L1858:
	mov	r0, r5
	bl	__udivsi3
	ldr	r3, [r4, #332]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldrh	r0, [r4, #236]
	bl	FtlFreeSysBlkQueueInit
	ldrh	r5, [r4, #240]
.L1860:
	ldrh	r3, [r4, #242]
	cmp	r3, r5
	bls	.L1862
	mov	r0, r5
	mov	r1, #1
	add	r5, r5, #1
	bl	FtlLowFormatEraseBlock
	uxth	r5, r5
	b	.L1860
.L1863:
	mov	r1, #0
	uxth	r0, r0
	bl	FtlLowFormatEraseBlock
	add	r6, r6, r0
	mov	r0, r5
	uxth	r6, r6
	b	.L1859
.L1869:
	ldr	r2, [r4, #2588]
	lsl	r3, r3, #1
	ldrh	r1, [r4, #28]
	mvn	r5, #0
	ldr	r6, .L1876+16
	str	r2, [r4, #36]
	add	r2, r2, #1
	str	r2, [r4, #2588]
	ldr	r2, [r4, #72]
	strh	r1, [r2, r3]	@ movhi
	mov	r3, #0
	strh	r3, [r4, #78]	@ movhi
	strb	r3, [r4, #82]
	ldrh	r3, [r4, #24]
	add	r3, r3, #1
	strh	r3, [r4, #76]	@ movhi
	mov	r3, #1
	strb	r3, [r4, #84]
.L1870:
	mov	r0, r6
	bl	make_superblock
	ldrb	r3, [r4, #83]	@ zero_extendqisi2
	cmp	r3, #0
	ldrh	r3, [r4, #76]
	bne	.L1871
	ldr	r2, [r4, #72]
	lsl	r3, r3, #1
	strh	r5, [r2, r3]	@ movhi
	ldrh	r3, [r4, #76]
	add	r3, r3, #1
	strh	r3, [r4, #76]	@ movhi
	b	.L1870
.L1871:
	ldr	r2, [r4, #2588]
	lsl	r3, r3, #1
	ldrh	r1, [r4, #80]
	mvn	r5, #0
	str	r2, [r4, #88]
	add	r2, r2, #1
	str	r2, [r4, #2588]
	ldr	r2, [r4, #72]
	strh	r1, [r2, r3]	@ movhi
	strh	r5, [r4, #124]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, .L1876+20
	mov	r2, #0
	strh	r2, [r3, #2]	@ movhi
	ldr	r2, [r4, #3672]
	strh	r5, [r3, #4]	@ movhi
	strh	r0, [r3]	@ movhi
	strh	r2, [r3, #6]	@ movhi
	ldr	r3, [r4, #2588]
	str	r3, [r4, #2632]
	add	r3, r3, #1
	str	r3, [r4, #2588]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cmp	r0, #0
	mov	r0, #0
	ldreq	r3, .L1876+24
	moveq	r2, #1
	streq	r2, [r3]
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1877:
	.align	2
.L1876:
	.word	.LANCHOR0
	.word	.LANCHOR0+258
	.word	168778952
	.word	.LANCHOR0+24
	.word	.LANCHOR0+76
	.word	.LANCHOR0+2624
	.word	.LANCHOR2
	.size	ftl_low_format, .-ftl_low_format
	.align	2
	.global	sftl_init
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_init, %function
sftl_init:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r4, .L1884
	mvn	r3, #0
	ldr	r5, .L1884+4
	ldr	r1, .L1884+8
	ldr	r0, .L1884+12
	str	r3, [r5]
	bl	sftl_printk
	mov	r0, r4
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldrh	r0, [r4, #236]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cmp	r0, #0
	bne	.L1882
	bl	FtlSysBlkInit
	cmp	r0, #0
	bne	.L1882
	mov	r3, #1
	str	r3, [r5]
	ldrh	r3, [r4, #224]
	cmp	r3, #15
	bhi	.L1882
	movw	r4, #8129
.L1881:
	mov	r1, #1
	mov	r0, #0
	bl	rk_ftl_garbage_collect
	subs	r4, r4, #1
	bne	.L1881
.L1882:
	mov	r0, #0
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1885:
	.align	2
.L1884:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC0
	.word	.LC77
	.size	sftl_init, .-sftl_init
	.align	2
	.global	FtlWriteToIDB
	.syntax unified
	.arm
	.fpu softvfp
	.type	FtlWriteToIDB, %function
FtlWriteToIDB:
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #100
	add	r8, r1, r0
	sub	r9, r8, #1
	mov	r4, r0
	cmp	r9, #63
	mov	r6, r1
	movhi	r3, #0
	movls	r3, #1
	cmp	r0, #576
	orrcs	r3, r3, #1
	mov	r7, r2
	cmp	r3, #0
	ldr	r5, .L1946
	beq	.L1887
	ldr	r3, [r5, #3676]
	cmp	r3, #0
	bne	.L1888
.L1945:
	mov	r4, #0
	b	.L1886
.L1888:
	ldr	r7, [r5, #3680]
	ldr	r3, .L1946+4
	ldr	r2, [r7]
	cmp	r2, r3
	bne	.L1890
	ldrh	r1, [r5, #10]
	add	r0, r7, #260096
	mov	r2, #0
	movw	r3, #65023
.L1894:
	ldr	ip, [r0, #-4]!
	cmp	ip, #0
	bne	.L1891
	ldr	ip, [r7, r2, lsl #2]
	add	r2, r2, #1
	cmp	r2, #4096
	sub	r3, r3, #1
	movhi	r2, #0
	cmp	r3, #4096
	str	ip, [r0, #2048]
	bne	.L1894
	mov	r3, #512
	b	.L1943
.L1891:
	add	r3, r3, #127
	lsr	r3, r3, #7
.L1943:
	str	r3, [fp, #-108]
	lsl	r1, r1, #2
	ldr	r3, [fp, #-108]
	uxth	r1, r1
	add	r0, r3, #4
	bl	__udivsi3
	add	r3, r0, #1
	ldr	r1, [fp, #-108]
	mov	r2, r3
	ldr	r0, .L1946+8
	str	r3, [fp, #-112]
	bl	sftl_printk
	ldr	r3, [fp, #-108]
	lsl	r3, r3, #7
	str	r3, [fp, #-128]
	mov	r3, #0
	mov	r6, r3
	str	r3, [fp, #-116]
.L1895:
	ldr	r3, [fp, #-112]
	add	r3, r3, r6
	cmp	r3, #8
	str	r3, [fp, #-120]
	bls	.L1917
	ldr	r3, [fp, #-116]
	cmp	r3, #0
	bne	.L1918
.L1890:
	mvn	r3, #0
	str	r3, [fp, #-108]
.L1918:
	ldr	r5, .L1946
	mov	r3, #0
	ldr	r4, [fp, #-108]
	ldr	r0, [r5, #3680]
	str	r3, [r5, #3676]
	bl	kfree
	ldr	r0, [r5, #3684]
	bl	kfree
.L1886:
	mov	r0, r4
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1917:
	ldr	r4, .L1946
	mov	r1, #512
	ldr	r0, [r4, #3684]
	bl	__memzero
	ldrh	r5, [r4, #10]
	mul	r3, r5, r6
	str	r3, [fp, #-124]
	ldr	r3, [r4, #3260]
	cmp	r3, #0
	moveq	r8, #6
	beq	.L1896
	ldr	r3, [r4, #3264]
	cmp	r3, #0
	moveq	r8, #6
	movne	r8, #9
.L1896:
	mul	r10, r5, r6
	ldr	r9, .L1946
	mov	r4, #0
.L1897:
	ldr	r3, [r9, #3248]
	mov	r1, r10
	mov	r0, #0
	add	r4, r4, #1
	blx	r3
	ldr	r3, [fp, #-112]
	add	r10, r10, r5
	cmp	r3, r4
	bhi	.L1897
	cmp	r8, #9
	movne	r4, #0
	bne	.L1898
	ldr	r4, [r9, #3684]
	mov	r1, #1024
	mov	r0, r4
	bl	__memzero
	ldr	r3, .L1946+12
	mov	r1, #12
	mov	r2, #4
	str	r1, [r4, #4]
	strb	r2, [r4, #17]
	add	r0, r4, r1
	str	r3, [r4]
	mov	r3, #0
	ldrh	r2, [r9, #10]
	str	r3, [r4, #12]
	strb	r3, [r4, #16]
	strh	r2, [r4, #18]	@ movhi
	mov	r2, #16
	strb	r3, [r4, #20]
	strb	r2, [r4, #21]
	strh	r3, [r4, #22]	@ movhi
	bl	js_hash
	str	r0, [r4, #8]
.L1898:
	ldr	r3, [fp, #-112]
	mov	r10, r7
	ldr	r9, .L1946
	mul	r3, r5, r3
	mov	r5, #0
	str	r3, [fp, #-132]
.L1899:
	ldr	r3, [fp, #-132]
	cmp	r5, r3
	beq	.L1906
	cmp	r8, #9
	lslne	r3, r5, #2
	addeq	r3, r5, #1
	cmp	r5, #0
	cmpeq	r8, #9
	str	r3, [fp, #-104]
	movw	r3, #61424
	str	r3, [fp, #-100]
	moveq	r0, #1
	movne	r0, #0
	bne	.L1902
	ldr	r3, [r9, #3260]
	mov	r0, #70
	blx	r3
	sub	r3, fp, #104
	ldr	ip, [r9, #3252]
	mov	r2, r4
	ldr	r1, [fp, #-124]
	mov	r0, #0
	blx	ip
	ldr	r2, [r9, #3260]
	str	r0, [fp, #-136]
	ldrb	r0, [r9, #22]	@ zero_extendqisi2
	blx	r2
	ldr	r3, [fp, #-136]
	cmn	r3, #1
	bne	.L1903
.L1906:
	ldr	r3, .L1946
	ldrb	r2, [r3, #14]	@ zero_extendqisi2
	ldr	r1, [r3, #3260]
	ldr	r8, [r3, #3684]
	str	r2, [fp, #-124]
	ldrh	r2, [r3, #10]
	cmp	r1, #0
	moveq	r9, #6
	mul	r10, r2, r6
	beq	.L1905
	ldr	r3, [r3, #3264]
	cmp	r3, #0
	moveq	r9, #6
	movne	r9, #9
.L1905:
	ldr	r3, [fp, #-112]
	mov	r5, #0
	ldr	r4, .L1946
	mul	r3, r2, r3
	str	r3, [fp, #-132]
.L1908:
	ldr	r3, [fp, #-132]
	cmp	r5, r3
	beq	.L1912
	cmp	r5, #0
	cmpeq	r9, #9
	moveq	r0, #1
	movne	r0, #0
	bne	.L1909
	ldr	r3, [r4, #3260]
	mov	r0, #70
	blx	r3
	ldr	r3, [r4, #3264]
	mov	r0, #2
	blx	r3
	mov	r2, r8
	ldr	ip, [r4, #3256]
	mov	r1, r10
	sub	r3, fp, #104
	mov	r0, #0
	blx	ip
	ldr	r3, [r4, #3264]
	ldr	r0, [fp, #-124]
	blx	r3
	ldr	r3, [r4, #3260]
	ldrb	r0, [r4, #22]	@ zero_extendqisi2
	blx	r3
	ldr	r3, [r8]
	ldr	r2, .L1946+12
	cmp	r3, r2
	beq	.L1910
.L1912:
	ldr	r5, .L1946
	mov	r3, r7
	mov	r4, #0
	ldr	r0, [r5, #3684]
.L1911:
	mov	r9, r3
	ldr	r1, [r0, r4, lsl #2]
	ldr	r2, [r9]
	lsl	r8, r4, #2
	add	r3, r3, #4
	cmp	r1, r2
	beq	.L1914
	mov	r1, #512
	bl	__memzero
	ldr	r2, [r5, #3684]
	mov	r1, r6
	str	r4, [sp]
	ldr	r0, .L1946+16
	ldr	r3, [r9]
	ldr	r2, [r2, r8]
	bl	sftl_printk
	ldrh	r1, [r5, #10]
	mov	r0, #0
	ldr	r3, [r5, #3248]
	mul	r1, r6, r1
	blx	r3
.L1915:
	ldr	r6, [fp, #-120]
	b	.L1895
.L1902:
	ldr	r1, [fp, #-124]
	sub	r3, fp, #104
	ldr	ip, [r9, #3252]
	mov	r2, r10
	add	r1, r1, r5
	blx	ip
	cmn	r0, #1
	beq	.L1906
	add	r10, r10, #2048
.L1903:
	add	r5, r5, #1
	b	.L1899
.L1909:
	ldr	ip, [r4, #3256]
	sub	r3, fp, #104
	mov	r2, r8
	add	r1, r10, r5
	blx	ip
	cmn	r0, #1
	beq	.L1912
	ldr	r2, [fp, #-100]
	movw	r3, #61424
	cmp	r2, r3
	bne	.L1912
	add	r8, r8, #2048
.L1910:
	add	r5, r5, #1
	b	.L1908
.L1914:
	ldr	r2, [fp, #-128]
	add	r4, r4, #1
	cmp	r4, r2
	bne	.L1911
	ldr	r3, [fp, #-116]
	add	r3, r3, #1
	cmp	r3, #5
	str	r3, [fp, #-116]
	bls	.L1915
	b	.L1918
.L1887:
	cmp	r0, #64
	bne	.L1919
	mov	r0, #262144
	bl	ftl_malloc
	str	r0, [r5, #3680]
	mov	r0, #262144
	bl	ftl_malloc
	ldr	r3, [r5, #3680]
	str	r0, [r5, #3684]
	cmp	r3, #0
	cmpne	r0, #0
	beq	.L1920
	mov	r2, #1
	mov	r1, #262144
	mov	r0, r3
	str	r2, [r5, #3676]
	bl	__memzero
.L1919:
	ldr	r3, [r5, #3676]
	cmp	r3, #0
	beq	.L1945
	cmp	r4, #63
	ldr	r0, [r5, #3680]
	ldrhi	r3, .L1946+20
	rsbls	r1, r4, #64
	subls	r6, r6, r1
	movhi	r1, r7
	addls	r1, r7, r1, lsl #9
	addhi	r3, r4, r3
	addhi	r0, r0, r3, lsl #9
	cmp	r9, #576
	subcs	r6, r6, r8
	subcs	r6, r6, #444
	subcs	r6, r6, #2
	lsl	r2, r6, #9
	bl	ftl_memcpy
	b	.L1945
.L1920:
	ldr	r1, .L1946+24
	ldr	r0, .L1946+28
	bl	sftl_printk
	b	.L1919
.L1947:
	.align	2
.L1946:
	.word	.LANCHOR0
	.word	-52655045
	.word	.LC130
	.word	1179535694
	.word	.LC131
	.word	8388544
	.word	.LANCHOR1+633
	.word	.LC132
	.size	FtlWriteToIDB, .-FtlWriteToIDB
	.align	2
	.global	sftl_write
	.syntax unified
	.arm
	.fpu softvfp
	.type	sftl_write, %function
sftl_write:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r5, r1
	mov	r6, r2
	mov	r4, r0
	bl	FtlWriteToIDB
	mov	r3, r6
	mov	r2, r5
	mov	r1, r4
	mov	r0, #0
	bl	FtlWrite
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
	.size	sftl_write, .-sftl_write
	.align	2
	.global	rk_sftl_vendor_dev_ops_register
	.syntax unified
	.arm
	.fpu softvfp
	.type	rk_sftl_vendor_dev_ops_register, %function
rk_sftl_vendor_dev_ops_register:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r2, .L1952
	ldr	r3, [r2, #3688]
	cmp	r3, #0
	streq	r0, [r2, #3688]
	moveq	r0, r3
	streq	r1, [r2, #3692]
	mvnne	r0, #0
	ldmfd	sp, {fp, sp, pc}
.L1953:
	.align	2
.L1952:
	.word	.LANCHOR0
	.size	rk_sftl_vendor_dev_ops_register, .-rk_sftl_vendor_dev_ops_register
	.align	2
	.global	rk_sftl_vendor_storage_init
	.syntax unified
	.arm
	.fpu softvfp
	.type	rk_sftl_vendor_storage_init, %function
rk_sftl_vendor_storage_init:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r0, #65536
	ldr	r4, .L1963
	bl	ftl_malloc
	cmp	r0, #0
	str	r0, [r4, #3696]
	mvneq	r9, #11
	beq	.L1954
	ldr	r8, .L1963+4
	mov	r6, #0
	mov	r5, r6
	mov	r7, r6
.L1958:
	ldr	r3, [r4, #3688]
	mov	r1, #128
	ldr	r2, [r4, #3696]
	lsl	r0, r7, #7
	blx	r3
	subs	r9, r0, #0
	bne	.L1956
	ldr	r2, [r4, #3696]
	ldr	r3, [r2]
	cmp	r3, r8
	bne	.L1957
	add	r1, r2, #61440
	ldr	r3, [r2, #4]
	ldr	r1, [r1, #4092]
	cmp	r3, r5
	sub	r1, r1, r3
	clz	r1, r1
	lsr	r1, r1, #5
	movls	r1, #0
	cmp	r1, #0
	movne	r6, r7
	movne	r5, r3
.L1957:
	add	r7, r7, #1
	cmp	r7, #2
	bne	.L1958
	cmp	r5, #0
	beq	.L1959
	ldr	r3, [r4, #3688]
	mov	r1, #128
	lsl	r0, r6, #7
	blx	r3
	subs	r9, r0, #0
	beq	.L1954
.L1956:
	ldr	r0, [r4, #3696]
	mvn	r9, #0
	bl	kfree
	mov	r3, #0
	str	r3, [r4, #3696]
	b	.L1954
.L1959:
	mov	r1, #65536
	mov	r0, r2
	bl	__memzero
	ldr	r3, [r4, #3696]
	mov	r2, #1
	add	r1, r3, #61440
	str	r2, [r3, #4]
	str	r8, [r3]
	str	r2, [r1, #4092]
	ldr	r2, .L1963+8
	strh	r5, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
.L1954:
	mov	r0, r9
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, fp, sp, pc}
.L1964:
	.align	2
.L1963:
	.word	.LANCHOR0
	.word	1380668996
	.word	-1032
	.size	rk_sftl_vendor_storage_init, .-rk_sftl_vendor_storage_init
	.align	2
	.global	rk_sftl_vendor_read
	.syntax unified
	.arm
	.fpu softvfp
	.type	rk_sftl_vendor_read, %function
rk_sftl_vendor_read:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r3, .L1971
	mov	r4, r0
	mov	r0, r1
	ldr	ip, [r3, #3696]
	cmp	ip, #0
	ldrhne	lr, [ip, #10]
	movne	r3, #0
	bne	.L1967
.L1970:
	mvn	r0, #0
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1968:
	add	r3, r3, #1
.L1967:
	cmp	r3, lr
	bcs	.L1970
	add	r1, ip, r3, lsl #3
	ldrh	r5, [r1, #16]
	cmp	r5, r4
	bne	.L1968
	ldrh	r4, [r1, #20]
	ldrh	r1, [r1, #18]
	cmp	r4, r2
	movcs	r4, r2
	add	r1, r1, #1024
	mov	r2, r4
	add	r1, ip, r1
	bl	memcpy
	mov	r0, r4
	ldmfd	sp, {r4, r5, fp, sp, pc}
.L1972:
	.align	2
.L1971:
	.word	.LANCHOR0
	.size	rk_sftl_vendor_read, .-rk_sftl_vendor_read
	.align	2
	.global	rk_sftl_vendor_write
	.syntax unified
	.arm
	.fpu softvfp
	.type	rk_sftl_vendor_write, %function
rk_sftl_vendor_write:
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, r7, r8, r9, r10, fp, ip, lr, pc}
	sub	fp, ip, #4
	sub	sp, sp, #24
	mov	ip, r1
	ldr	r8, .L1992
	mov	r9, r2
	ldr	r4, [r8, #3696]
	cmp	r4, #0
	beq	.L1988
	ldrh	r2, [r4, #10]
	add	r7, r9, #63
	ldrh	r3, [r4, #8]
	bic	r7, r7, #63
	mov	r5, #0
	str	r3, [fp, #-44]
.L1975:
	cmp	r5, r2
	bcc	.L1983
	ldrh	r1, [r4, #14]
	cmp	r7, r1
	bhi	.L1988
	add	r3, r4, r2, lsl #3
	uxth	r7, r7
	strh	r0, [r3, #16]	@ movhi
	ldrh	r2, [r4, #12]
	strh	r9, [r3, #20]	@ movhi
	strh	r2, [r3, #18]	@ movhi
	add	r2, r2, r7
	sub	r7, r1, r7
	strh	r2, [r4, #12]	@ movhi
	strh	r7, [r4, #14]	@ movhi
	mov	r2, r9
	ldrh	r0, [r3, #18]
	mov	r1, ip
	add	r0, r0, #1024
	add	r0, r4, r0
	bl	memcpy
	ldrh	r3, [r4, #10]
	add	r2, r4, #61440
	add	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
	ldr	r3, [r4, #4]
	add	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r2, #4092]
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #1
	movhi	r3, #0
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, [r8, #3692]
	b	.L1991
.L1983:
	add	r6, r4, r5, lsl #3
	ldrh	r3, [r6, #16]
	cmp	r3, r0
	str	r3, [fp, #-48]
	bne	.L1976
	ldrh	r1, [r6, #20]
	add	r3, r4, #1024
	add	r1, r1, #63
	bic	r1, r1, #63
	cmp	r9, r1
	str	r1, [fp, #-52]
	bls	.L1977
	ldrh	r1, [r4, #14]
	cmp	r7, r1
	bhi	.L1988
	ldrh	r8, [r6, #18]
	sub	r2, r2, #1
	str	r2, [fp, #-56]
.L1978:
	ldr	r2, [fp, #-56]
	add	r6, r6, #8
	cmp	r5, r2
	bcc	.L1979
	ldrh	r2, [fp, #-48]
	add	r5, r4, r5, lsl #3
	uxth	r8, r8
	mov	r1, ip
	strh	r9, [r5, #20]	@ movhi
	add	r0, r3, r8
	strh	r2, [r5, #16]	@ movhi
	strh	r8, [r5, #18]	@ movhi
	mov	r2, r9
	bl	memcpy
	uxth	r3, r7
	ldrh	r7, [r4, #14]
	add	r8, r8, r3
	sub	r7, r7, r3
	ldr	r3, [fp, #-52]
	strh	r8, [r4, #12]	@ movhi
	add	r7, r7, r3
	strh	r7, [r4, #14]	@ movhi
.L1980:
	ldr	r3, [r4, #4]
	add	r2, r4, #61440
	add	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r2, #4092]
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #1
	movhi	r3, #0
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, .L1992
	ldr	r3, [r3, #3692]
.L1991:
	ldr	r0, [fp, #-44]
	mov	r2, r4
	mov	r1, #128
	lsl	r0, r0, #7
	blx	r3
	mov	r0, #0
.L1973:
	sub	sp, fp, #40
	ldmfd	sp, {r4, r5, r6, r7, r8, r9, r10, fp, sp, pc}
.L1979:
	ldrh	r10, [r6, #20]
	add	r0, r3, r8
	ldrh	r2, [r6, #16]
	add	r5, r5, #1
	ldrh	r1, [r6, #18]
	strh	r10, [r6, #12]	@ movhi
	add	r10, r10, #63
	bic	r10, r10, #63
	strh	r2, [r6, #8]	@ movhi
	strh	r8, [r6, #10]	@ movhi
	add	r1, r3, r1
	mov	r2, r10
	str	ip, [fp, #-64]
	str	r3, [fp, #-60]
	add	r8, r8, r10
	bl	memcpy
	ldr	ip, [fp, #-64]
	ldr	r3, [fp, #-60]
	b	.L1978
.L1977:
	add	r5, r4, r5, lsl #3
	mov	r2, r9
	mov	r1, ip
	ldrh	r0, [r5, #18]
	add	r0, r3, r0
	bl	memcpy
	strh	r9, [r6, #20]	@ movhi
	b	.L1980
.L1976:
	add	r5, r5, #1
	b	.L1975
.L1988:
	mvn	r0, #0
	b	.L1973
.L1993:
	.align	2
.L1992:
	.word	.LANCHOR0
	.size	rk_sftl_vendor_write, .-rk_sftl_vendor_write
	.align	2
	.global	rk_sftl_vendor_storage_ioctl
	.syntax unified
	.arm
	.fpu softvfp
	.type	rk_sftl_vendor_storage_ioctl, %function
rk_sftl_vendor_storage_ioctl:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{r4, r5, r6, fp, ip, lr, pc}
	sub	fp, ip, #4
	mov	r0, #4096
	mov	r4, r2
	mov	r6, r1
	bl	ftl_malloc
	subs	r5, r0, #0
	mvneq	r4, #0
	beq	.L1994
	ldr	r3, .L2022
	cmp	r6, r3
	beq	.L1997
	add	r3, r3, #1
	cmp	r6, r3
	beq	.L1998
.L2017:
	mvn	r4, #13
	b	.L1996
.L1997:
	ldr	r6, .L2022+4
	mov	r3, sp
	and	r3, r3, r6
	ldr	r3, [r3, #72]
	.syntax divided
@ 564 "./arch/arm/include/asm/uaccess.h" 1
	adds r2, r4, #8; sbcccs r2, r2, r3; movcc r3, #0
@ 0 "" 2
	.arm
	.syntax unified
	cmp	r3, #0
	bne	.L1999
	mov	r2, #8
	mov	r1, r4
	bl	arm_copy_from_user
	cmp	r0, #0
	beq	.L2018
.L2000:
	mov	r2, r4
	mov	r1, #256
.L2021:
	ldr	r0, .L2022+8
	bl	sftl_printk
	b	.L2017
.L1999:
	mov	r1, #8
	bl	__memzero
	b	.L2000
.L2018:
	ldr	r2, [r5]
	ldr	r3, .L2022+12
	cmp	r2, r3
	beq	.L2002
.L2003:
	mvn	r4, #0
.L1996:
	mov	r0, r5
	bl	kfree
.L1994:
	mov	r0, r4
	ldmfd	sp, {r4, r5, r6, fp, sp, pc}
.L2002:
	ldrh	r2, [r5, #6]
	add	r1, r5, #8
	ldrh	r0, [r5, #4]
	bl	rk_sftl_vendor_read
	cmn	r0, #1
	beq	.L2003
	mov	r3, sp
	uxth	r2, r0
	and	r6, r6, r3
	strh	r0, [r5, #6]	@ movhi
	add	r2, r2, #8
	ldr	r3, [r6, #72]
	.syntax divided
@ 573 "./arch/arm/include/asm/uaccess.h" 1
	adds r1, r4, r2; sbcccs r1, r1, r3; movcc r3, #0
@ 0 "" 2
	.arm
	.syntax unified
	cmp	r3, #0
	bne	.L2017
	mov	r0, r4
	mov	r1, r5
	bl	arm_copy_to_user
	subs	r4, r0, #0
	beq	.L1996
	b	.L2017
.L1998:
	ldr	r6, .L2022+4
	mov	r3, sp
	and	r3, r3, r6
	ldr	r3, [r3, #72]
	.syntax divided
@ 564 "./arch/arm/include/asm/uaccess.h" 1
	adds r2, r4, #8; sbcccs r2, r2, r3; movcc r3, #0
@ 0 "" 2
	.arm
	.syntax unified
	cmp	r3, #0
	bne	.L2005
	mov	r2, #8
	mov	r1, r4
	bl	arm_copy_from_user
	cmp	r0, #0
	beq	.L2019
.L2006:
	mov	r2, r4
	mov	r1, #276
	b	.L2021
.L2005:
	mov	r1, #8
	bl	__memzero
	b	.L2006
.L2019:
	ldr	r2, [r5]
	ldr	r3, .L2022+12
	cmp	r2, r3
	bne	.L2003
	ldrh	r2, [r5, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L2003
	mov	r3, sp
	add	r2, r2, #8
	and	r6, r6, r3
	ldr	r3, [r6, #72]
	.syntax divided
@ 564 "./arch/arm/include/asm/uaccess.h" 1
	adds r1, r4, r2; sbcccs r1, r1, r3; movcc r3, #0
@ 0 "" 2
	.arm
	.syntax unified
	cmp	r3, #0
	bne	.L2008
	mov	r1, r4
	mov	r0, r5
	bl	arm_copy_from_user
	cmp	r0, #0
	beq	.L2020
.L2009:
	mov	r2, r4
	movw	r1, #283
	b	.L2021
.L2008:
	mov	r1, r2
	mov	r0, r5
	bl	__memzero
	b	.L2009
.L2020:
	ldrh	r2, [r5, #6]
	add	r1, r5, #8
	ldrh	r0, [r5, #4]
	bl	rk_sftl_vendor_write
	mov	r4, r0
	b	.L1996
.L2023:
	.align	2
.L2022:
	.word	1074034177
	.word	-8192
	.word	.LC133
	.word	1448232273
	.size	rk_sftl_vendor_storage_ioctl, .-rk_sftl_vendor_storage_ioctl
	.align	2
	.global	rk_sftl_vendor_register
	.syntax unified
	.arm
	.fpu softvfp
	.type	rk_sftl_vendor_register, %function
rk_sftl_vendor_register:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 1, uses_anonymous_args = 0
	mov	ip, sp
	push	{fp, ip, lr, pc}
	sub	fp, ip, #4
	ldr	r0, .L2025
	bl	misc_register
	ldmfd	sp, {fp, sp, pc}
.L2026:
	.align	2
.L2025:
	.word	.LANCHOR2+12
	.size	rk_sftl_vendor_register, .-rk_sftl_vendor_register
	.global	g_nand_ops
	.global	g_nand_phy_info
	.global	gc_ink_free_return_value
	.global	check_vpc_table
	.global	FtlUpdateVaildLpnCount
	.global	g_ect_tbl_power_up_flush
	.global	power_up_flag
	.global	gFtlInitStatus
	.global	DeviceCapacity
	.global	g_power_lost_recovery_flag
	.global	c_mlc_erase_count_value
	.global	g_recovery_ppa_tbl
	.global	g_recovery_page_min_ver
	.global	g_recovery_page_num
	.global	sftl_nand_check_spare_buf
	.global	sftl_temp_buf
	.global	sftl_nand_check_buf
	.global	g_cur_erase_blk
	.global	g_gc_skip_write_count
	.global	g_gc_head_data_block_count
	.global	g_gc_head_data_block
	.global	g_ftl_nand_free_count
	.global	g_in_swl_replace
	.global	g_in_gc_progress
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
	.global	p_plane_order_table
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
	.section	.rodata
	.align	2
	.set	.LANCHOR1,. + 0
	.type	__func__.3971, %object
	.size	__func__.3971, 17
__func__.3971:
	.ascii	"INSERT_DATA_LIST\000"
	.type	__func__.3966, %object
	.size	__func__.3966, 17
__func__.3966:
	.ascii	"INSERT_FREE_LIST\000"
	.type	__func__.4002, %object
	.size	__func__.4002, 17
__func__.4002:
	.ascii	"List_remove_node\000"
	.type	__func__.4034, %object
	.size	__func__.4034, 22
__func__.4034:
	.ascii	"List_update_data_list\000"
	.type	__func__.4141, %object
	.size	__func__.4141, 22
__func__.4141:
	.ascii	"select_l2p_ram_region\000"
	.type	__func__.4461, %object
	.size	__func__.4461, 16
__func__.4461:
	.ascii	"make_superblock\000"
	.type	__func__.4637, %object
	.size	__func__.4637, 19
__func__.4637:
	.ascii	"get_new_active_ppa\000"
	.type	__func__.16396, %object
	.size	__func__.16396, 17
__func__.16396:
	.ascii	"FlashEraseBlocks\000"
	.type	__func__.4076, %object
	.size	__func__.4076, 26
__func__.4076:
	.ascii	"ftl_map_blk_alloc_new_blk\000"
	.type	__func__.3581, %object
	.size	__func__.3581, 11
__func__.3581:
	.ascii	"FtlMemInit\000"
	.type	__func__.3808, %object
	.size	__func__.3808, 14
__func__.3808:
	.ascii	"FtlBbt2Bitmap\000"
	.type	__func__.4482, %object
	.size	__func__.4482, 18
__func__.4482:
	.ascii	"SupperBlkListInit\000"
	.type	__func__.16353, %object
	.size	__func__.16353, 15
__func__.16353:
	.ascii	"FlashReadPages\000"
	.type	__func__.4253, %object
	.size	__func__.4253, 14
__func__.4253:
	.ascii	"FtlScanSysBlk\000"
	.type	__func__.3851, %object
	.size	__func__.3851, 11
__func__.3851:
	.ascii	"FtlLoadBbt\000"
	.type	__func__.16372, %object
	.size	__func__.16372, 15
__func__.16372:
	.ascii	"FlashProgPages\000"
	.type	__func__.4609, %object
	.size	__func__.4609, 25
__func__.4609:
	.ascii	"allocate_data_superblock\000"
	.type	__func__.4650, %object
	.size	__func__.4650, 16
__func__.4650:
	.ascii	"update_vpc_list\000"
	.type	__func__.4657, %object
	.size	__func__.4657, 20
__func__.4657:
	.ascii	"decrement_vpc_count\000"
	.type	__func__.4102, %object
	.size	__func__.4102, 31
__func__.4102:
	.ascii	"Ftl_write_map_blk_to_last_page\000"
	.type	__func__.4116, %object
	.size	__func__.4116, 16
__func__.4116:
	.ascii	"FtlMapWritePage\000"
	.type	__func__.4043, %object
	.size	__func__.4043, 16
__func__.4043:
	.ascii	"load_l2p_region\000"
	.type	__func__.4087, %object
	.size	__func__.4087, 15
__func__.4087:
	.ascii	"ftl_map_blk_gc\000"
	.type	__func__.4158, %object
	.size	__func__.4158, 9
__func__.4158:
	.ascii	"log2phys\000"
	.type	__func__.4370, %object
	.size	__func__.4370, 16
__func__.4370:
	.ascii	"FtlReUsePrevPpa\000"
	.type	__func__.4404, %object
	.size	__func__.4404, 22
__func__.4404:
	.ascii	"FtlRecoverySuperblock\000"
	.type	__func__.4524, %object
	.size	__func__.4524, 14
__func__.4524:
	.ascii	"ftl_check_vpc\000"
	.type	__func__.4231, %object
	.size	__func__.4231, 15
__func__.4231:
	.ascii	"FtlVpcTblFlush\000"
	.type	__func__.4507, %object
	.size	__func__.4507, 21
__func__.4507:
	.ascii	"FtlVpcCheckAndModify\000"
	.type	__func__.4630, %object
	.size	__func__.4630, 29
__func__.4630:
	.ascii	"allocate_new_data_superblock\000"
	.type	__func__.3705, %object
	.size	__func__.3705, 13
__func__.3705:
	.ascii	"FtlProgPages\000"
	.type	__func__.4727, %object
	.size	__func__.4727, 19
__func__.4727:
	.ascii	"FtlGcFreeTempBlock\000"
	.type	__func__.4840, %object
	.size	__func__.4840, 23
__func__.4840:
	.ascii	"rk_ftl_garbage_collect\000"
	.type	__func__.3733, %object
	.size	__func__.3733, 9
__func__.3733:
	.ascii	"FtlWrite\000"
	.type	__func__.4302, %object
	.size	__func__.4302, 15
__func__.4302:
	.ascii	"FtlLoadSysInfo\000"
	.type	__func__.4324, %object
	.size	__func__.4324, 18
__func__.4324:
	.ascii	"FtlMapTblRecovery\000"
	.type	__func__.16524, %object
	.size	__func__.16524, 14
__func__.16524:
	.ascii	"FtlWriteToIDB\000"
	.space	1
	.type	rk_sftl_vendor_storage_fops, %object
	.size	rk_sftl_vendor_storage_fops, 108
rk_sftl_vendor_storage_fops:
	.space	32
	.word	rk_sftl_vendor_storage_ioctl
	.word	rk_sftl_vendor_storage_ioctl
	.space	68
	.data
	.align	2
	.set	.LANCHOR2,. + 0
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
	.type	rkflash_vender_storage_dev, %object
	.size	rkflash_vender_storage_dev, 40
rkflash_vender_storage_dev:
	.word	255
	.word	.LC134
	.word	rk_sftl_vendor_storage_fops
	.space	28
	.bss
	.align	2
	.set	.LANCHOR0,. + 0
	.type	g_nand_phy_info, %object
	.size	g_nand_phy_info, 24
g_nand_phy_info:
	.space	24
	.type	g_active_superblock, %object
	.size	g_active_superblock, 48
g_active_superblock:
	.space	48
	.type	p_valid_page_count_table, %object
	.size	p_valid_page_count_table, 4
p_valid_page_count_table:
	.space	4
	.type	g_buffer_superblock, %object
	.size	g_buffer_superblock, 48
g_buffer_superblock:
	.space	48
	.type	g_gc_temp_superblock, %object
	.size	g_gc_temp_superblock, 48
g_gc_temp_superblock:
	.space	48
	.type	g_gc_superblock, %object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.space	48
	.type	g_gc_next_blk_1, %object
	.size	g_gc_next_blk_1, 2
g_gc_next_blk_1:
	.space	2
	.type	g_gc_next_blk, %object
	.size	g_gc_next_blk, 2
g_gc_next_blk:
	.space	2
	.type	g_num_free_superblocks, %object
	.size	g_num_free_superblocks, 2
g_num_free_superblocks:
	.space	2
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
	.type	c_ftl_nand_sec_pre_page, %object
	.size	c_ftl_nand_sec_pre_page, 2
c_ftl_nand_sec_pre_page:
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
	.type	gBbtInfo, %object
	.size	gBbtInfo, 60
gBbtInfo:
	.space	60
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
	.type	p_erase_count_table, %object
	.size	p_erase_count_table, 4
p_erase_count_table:
	.space	4
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
	.type	g_MaxLpn, %object
	.size	g_MaxLpn, 4
g_MaxLpn:
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
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.space	2
	.space	2
	.type	gSysInfo, %object
	.size	gSysInfo, 16
gSysInfo:
	.space	16
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
	.type	g_nand_ops, %object
	.size	g_nand_ops, 24
g_nand_ops:
	.space	24
	.type	req_erase, %object
	.size	req_erase, 4
req_erase:
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
	.type	p_io_data_buf_0, %object
	.size	p_io_data_buf_0, 4
p_io_data_buf_0:
	.space	4
	.type	p_io_data_buf_1, %object
	.size	p_io_data_buf_1, 4
p_io_data_buf_1:
	.space	4
	.type	sftl_nand_check_buf, %object
	.size	sftl_nand_check_buf, 4
sftl_nand_check_buf:
	.space	4
	.type	sftl_temp_buf, %object
	.size	sftl_temp_buf, 4
sftl_temp_buf:
	.space	4
	.type	sftl_nand_check_spare_buf, %object
	.size	sftl_nand_check_spare_buf, 4
sftl_nand_check_spare_buf:
	.space	4
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 4
p_sys_spare_buf:
	.space	4
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 4
p_io_spare_buf:
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
	.type	g_tmp_data_superblock_id, %object
	.size	g_tmp_data_superblock_id, 2
g_tmp_data_superblock_id:
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
	.type	req_sys, %object
	.size	req_sys, 20
req_sys:
	.space	20
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
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 44
gVendorBlkInfo:
	.space	44
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.space	2
	.space	2
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
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.space	4
	.type	idb_need_write_back, %object
	.size	idb_need_write_back, 4
idb_need_write_back:
	.space	4
	.type	idb_buf, %object
	.size	idb_buf, 4
idb_buf:
	.space	4
	.type	gp_flash_check_buf, %object
	.size	gp_flash_check_buf, 4
gp_flash_check_buf:
	.space	4
	.type	_flash_read, %object
	.size	_flash_read, 4
_flash_read:
	.space	4
	.type	_flash_write, %object
	.size	_flash_write, 4
_flash_write:
	.space	4
	.type	g_vendor, %object
	.size	g_vendor, 4
g_vendor:
	.space	4
	.type	check_vpc_table, %object
	.size	check_vpc_table, 16384
check_vpc_table:
	.space	16384
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 4
gp_last_act_superblock:
	.space	4
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"SFTL version: 5.0.55 20200925\000"
.LC1:
	.ascii	"\012%s\012\000"
.LC2:
	.ascii	"act blk: %x %x %x %x %x %x\012\000"
.LC3:
	.ascii	"buf blk: %x %x %x %x %x %x\012\000"
.LC4:
	.ascii	"tmp blk: %x %x %x %x %x %x\012\000"
.LC5:
	.ascii	"gc blk: %x %x %x %x %x %x\012\000"
.LC6:
	.ascii	"free blk: %x %x %x\012\000"
.LC7:
	.ascii	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\012"
	.ascii	"\000"
.LC8:
	.ascii	"\012!!!!! error @ func:%s - line:%d\012\000"
.LC9:
	.ascii	"FLASH INFO:\012\000"
.LC10:
	.ascii	"Device Capacity: %d MB\012\000"
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
	.ascii	"bad blk num = %d\012\000"
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
	.ascii	"WR_CHK = %x %x %x\012\000"
.LC66:
	.ascii	"Read Err Cnt = 0x%x\012\000"
.LC67:
	.ascii	"Prog Err Cnt = 0x%x\012\000"
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
	.ascii	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x"
	.ascii	"\012\000"
.LC79:
	.ascii	"FtlGcRefreshBlock  0x%x\012\000"
.LC80:
	.ascii	"FtlGcMarkBadPhyBlk %d 0x%x\012\000"
.LC81:
	.ascii	"%s %p + 0x%x:\000"
.LC82:
	.ascii	"0x%08x,\000"
.LC83:
	.ascii	"0x%04x,\000"
.LC84:
	.ascii	"0x%02x,\000"
.LC85:
	.ascii	"\012\000"
.LC86:
	.ascii	"%s: addr: %x is in id block!!!!!!!!!!\012\000"
.LC87:
	.ascii	"not free: w: d:\000"
.LC88:
	.ascii	"not free: w: s:\000"
.LC89:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\012"
	.ascii	"\000"
.LC90:
	.ascii	"FtlFreeSysBlkQueueOut = %x, free count = %d\012\000"
.LC91:
	.ascii	"%s error allocating memory. return -1\012\000"
.LC92:
	.ascii	"FlashReadPages %x %x error_ecc_bits %d\012\000"
.LC93:
	.ascii	"data:\000"
.LC94:
	.ascii	"spare:\000"
.LC95:
	.ascii	"prog read error: = %x\012\000"
.LC96:
	.ascii	"prog read REFRESH: = %x\012\000"
.LC97:
	.ascii	"prog read s error: = %x %x %x\012\000"
.LC98:
	.ascii	"prog read d error: = %x %x %x\012\000"
.LC99:
	.ascii	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\012\000"
.LC100:
	.ascii	"FtlBbmTblFlush error:%x\012\000"
.LC101:
	.ascii	"FtlBbmTblFlush error = %x error count = %d\012\000"
.LC102:
	.ascii	"FtlGcFreeBadSuperBlk 0x%x\012\000"
.LC103:
	.ascii	"decrement_vpc_count %x = %d\012\000"
.LC104:
	.ascii	"FtlMapWritePage error = %x \012\000"
.LC105:
	.ascii	"FtlMapWritePage error = %x error count = %d\012\000"
.LC106:
	.ascii	"region_id = %x phyAddr = %x\012\000"
.LC107:
	.ascii	"map_ppn:\000"
.LC108:
	.ascii	"load_l2p_region refresh = %x phyAddr = %x\012\000"
.LC109:
	.ascii	"data prev_ppa = %x error...................\012\000"
.LC110:
	.ascii	"spuer block %x vpn is 0\012 \000"
.LC111:
	.ascii	"...%s enter...\012\000"
.LC112:
	.ascii	"FtlCheckVpc2 %x = %x  %x\012\000"
.LC113:
	.ascii	"free blk vpc error %x = %x  %x\012\000"
.LC114:
	.ascii	"ftl_scan_all_data = %x\012\000"
.LC115:
	.ascii	"scan lpa = %x ppa= %x\012\000"
.LC116:
	.ascii	"lpa = %x,addr= %x,spare= %x %x %x %x data=%x %x\012"
	.ascii	"\000"
.LC117:
	.ascii	"FtlGcScanTempBlk Error ID %x %x!!!!!!! \012\000"
.LC118:
	.ascii	"FtlVendorPartRead refresh = %x phyAddr = %x\012\000"
.LC119:
	.ascii	"no ect\000"
.LC120:
	.ascii	"FtlVpcTblFlush error = %x error count = %d\012\000"
.LC121:
	.ascii	"FtlCheckVpc %x = %x  %x\012\000"
.LC122:
	.ascii	"FtlProgPages error %x = %d\012\000"
.LC123:
	.ascii	"Ftlwrite decrement_vpc_count %x = %d\012\000"
.LC124:
	.ascii	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\012\000"
.LC125:
	.ascii	"FtlWrite: ecc error:%x %x %x\012\000"
.LC126:
	.ascii	"FtlWrite: lpa error:%x %x\012\000"
.LC127:
	.ascii	"%s hash error this.id =%x page =%x pre_id =%x hash "
	.ascii	"=%x hash_r =%x\012\000"
.LC128:
	.ascii	"%s last blk_id =%x page =%x hash error hash =%x has"
	.ascii	"h_r =%x\012\000"
.LC129:
	.ascii	"%s scan blk_id =%x page =%x hash error hash =%x has"
	.ascii	"h_r =%x\012\000"
.LC130:
	.ascii	"write_idblock %x %x\012\000"
.LC131:
	.ascii	"write_idblock fail! %x %x %x %x\012\000"
.LC132:
	.ascii	"%s idb buffer alloc fail\012\000"
.LC133:
	.ascii	"copy_from_user error %d %p %p\012\000"
.LC134:
	.ascii	"vendor_storage\000"
