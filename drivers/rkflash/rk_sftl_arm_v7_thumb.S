/* SPDX-License-Identifier: GPL-2.0 */

/* Copyright (c) 2018 Rockchip Electronics Co. Ltd.
 * date: 2020-09-29
 */
	.arch armv7-a
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 2
	.eabi_attribute 30, 4
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"rk_sftl.c"
	.global	__aeabi_uidiv
	.text
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	l2p_addr_tran, %function
l2p_addr_tran:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r8, r1
	ldr	r3, .L3
	mov	r9, r2
	ldr	r6, [r0, #4]
	ldrh	r4, [r3, #8]
	ldrh	r5, [r3, #10]
	ldrh	r3, [r3, #14]
	lsrs	r7, r6, #10
	ubfx	r6, r6, #0, #10
	cmp	r3, #4
	uxth	r0, r7
	itt	eq
	lsreq	r4, r4, #1
	lsleq	r5, r5, #1
	uxth	r7, r7
	mov	r1, r4
	it	eq
	uxtheq	r5, r5
	bl	__aeabi_uidiv
	uxth	r0, r0
	mls	r4, r0, r4, r7
	mla	r4, r5, r4, r6
	str	r4, [r8]
	str	r0, [r9]
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L4:
	.align	2
.L3:
	.word	.LANCHOR0
	.fnend
	.size	l2p_addr_tran, .-l2p_addr_tran
	.align	1
	.global	ftl_print_sblk_info
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_print_sblk_info, %function
ftl_print_sblk_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r3, r4, lr}
	.save {r4, lr}
	.pad #16
	ldr	r4, .L6
	ldr	r1, .L6+4
	ldr	r0, .L6+8
	bl	sftl_printk
	ldrh	r1, [r4, #24]
	ldr	r0, [r4, #72]
	ldrb	r3, [r4, #30]	@ zero_extendqisi2
	ldrh	r2, [r4, #26]
	ldrh	r0, [r0, r1, lsl #1]
	str	r0, [sp, #8]
	ldrh	r0, [r4, #28]
	str	r0, [sp, #4]
	ldrb	r0, [r4, #32]	@ zero_extendqisi2
	str	r0, [sp]
	ldr	r0, .L6+12
	bl	sftl_printk
	ldrh	r1, [r4, #76]
	ldr	r0, [r4, #72]
	ldrb	r3, [r4, #82]	@ zero_extendqisi2
	ldrh	r2, [r4, #78]
	ldrh	r0, [r0, r1, lsl #1]
	str	r0, [sp, #8]
	ldrh	r0, [r4, #80]
	str	r0, [sp, #4]
	ldrb	r0, [r4, #84]	@ zero_extendqisi2
	str	r0, [sp]
	ldr	r0, .L6+16
	bl	sftl_printk
	ldrh	r1, [r4, #124]
	ldr	r0, [r4, #72]
	ldrb	r3, [r4, #130]	@ zero_extendqisi2
	ldrh	r2, [r4, #126]
	ldrh	r0, [r0, r1, lsl #1]
	str	r0, [sp, #8]
	ldrh	r0, [r4, #128]
	str	r0, [sp, #4]
	ldrb	r0, [r4, #132]	@ zero_extendqisi2
	str	r0, [sp]
	ldr	r0, .L6+20
	bl	sftl_printk
	ldrh	r1, [r4, #172]
	ldr	r0, [r4, #72]
	ldrb	r3, [r4, #178]	@ zero_extendqisi2
	ldrh	r2, [r4, #174]
	ldrh	r0, [r0, r1, lsl #1]
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
	add	sp, sp, #16
	@ sp needed
	pop	{r4, lr}
	b	sftl_printk
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
	.fnend
	.size	ftl_print_sblk_info, .-ftl_print_sblk_info
	.align	1
	.global	Ftl_log2
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	Ftl_log2, %function
Ftl_log2:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r1, #0
	movs	r2, #1
.L9:
	cmp	r2, r0
	uxth	r3, r1
	add	r1, r1, #1
	bls	.L10
	subs	r0, r3, #1
	uxth	r0, r0
	bx	lr
.L10:
	lsls	r2, r2, #1
	b	.L9
	.fnend
	.size	Ftl_log2, .-Ftl_log2
	.align	1
	.global	FtlPrintInfo
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
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
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlSysBlkNumInit, %function
FtlSysBlkNumInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L13
	cmp	r0, #24
	it	cc
	movcc	r0, #24
	ldrh	r2, [r3, #232]
	ldrh	r1, [r3, #242]
	str	r0, [r3, #228]
	muls	r2, r0, r2
	subs	r0, r1, r0
	ldr	r1, [r3, #248]
	strh	r0, [r3, #240]	@ movhi
	movs	r0, #0
	str	r2, [r3, #236]
	subs	r2, r1, r2
	str	r2, [r3, #244]
	bx	lr
.L14:
	.align	2
.L13:
	.word	.LANCHOR0
	.fnend
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.global	__aeabi_idiv
	.align	1
	.global	FtlConstantsInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlConstantsInit, %function
FtlConstantsInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r0
	ldrh	r1, [fp, #14]
	ldrh	r2, [r0]
	ldr	r4, .L26
	cmp	r1, #4
	ldrh	r6, [r0, #2]
	ldrh	r3, [fp, #6]
	ldrh	r0, [r0, #4]
	strh	r2, [r4, #252]	@ movhi
	strh	r6, [r4, #254]	@ movhi
	strh	r0, [r4, #256]	@ movhi
	strh	r3, [r4, #242]	@ movhi
	strh	r1, [r4, #258]	@ movhi
	bne	.L16
	lsrs	r3, r3, #1
	strh	r3, [r4, #242]	@ movhi
	movs	r3, #8
	strh	r3, [r4, #258]	@ movhi
.L17:
	ldr	r1, .L26+4
	movs	r3, #0
.L18:
	strb	r3, [r3, r1]
	adds	r3, r3, #1
	cmp	r3, #32
	bne	.L18
	ldrh	r5, [r4, #242]
	movs	r3, #5
	ldrh	r0, [r4, #256]
	cmp	r2, #1
	strh	r3, [r4, #292]	@ movhi
	mov	r7, #640
	mov	r3, #0
	it	eq
	strheq	r2, [r4, #292]	@ movhi
	strh	r3, [r4, #294]	@ movhi
	smulbb	r6, r6, r0
	strh	r7, [r4, #296]	@ movhi
	smulbb	r0, r0, r5
	uxth	r6, r6
	uxth	r0, r0
	strh	r6, [r4, #232]	@ movhi
	strh	r0, [r4, #298]	@ movhi
	bl	Ftl_log2
	ldrh	r9, [fp, #12]
	ldrh	r10, [r4, #258]
	strh	r0, [r4, #300]	@ movhi
	strh	r9, [r4, #302]	@ movhi
	smulbb	r3, r6, r9
	mov	r0, r10
	strh	r9, [r4, #304]	@ movhi
	strh	r3, [r4, #306]	@ movhi
	bl	Ftl_log2
	lsl	r3, r10, #9
	mov	r8, r0
	strh	r0, [r4, #308]	@ movhi
	mul	r1, r9, r10
	uxth	r3, r3
	mov	r0, #5120
	asr	r7, r7, r8
	strh	r3, [r4, #310]	@ movhi
	lsrs	r3, r3, #8
	add	r8, r8, #9
	strh	r3, [r4, #312]	@ movhi
	adds	r7, r7, #2
	ldrh	r3, [fp, #20]
	strh	r3, [r4, #314]	@ movhi
	mul	r3, r5, r6
	lsls	r5, r5, #6
	asr	r5, r5, r8
	str	r3, [r4, #248]
	mul	r3, r10, r3
	mul	r3, r9, r3
	asrs	r3, r3, #11
	str	r3, [r4, #316]
	bl	__aeabi_idiv
	uxth	r0, r0
	strh	r5, [r4, #324]	@ movhi
	uxth	r5, r5
	mov	r1, r6
	cmp	r0, #4
	strh	r7, [r4, #322]	@ movhi
	itet	ls
	movls	r3, #4
	strhhi	r0, [r4, #320]	@ movhi
	strhls	r3, [r4, #320]	@ movhi
	mul	r3, r6, r5
	ldrh	r0, [r4, #320]
	adds	r5, r5, #8
	str	r3, [r4, #328]
	bl	__aeabi_uidiv
	uxtah	r0, r5, r0
	cmp	r6, #1
	it	eq
	addeq	r0, r0, #4
	str	r0, [r4, #228]
	ldrh	r0, [r4, #228]
	bl	FtlSysBlkNumInit
	ldr	r3, [r4, #228]
	movs	r0, #0
	str	r0, [r4, #340]
	str	r3, [r4, #332]
	ldr	r3, [r4, #244]
	lsls	r2, r3, #2
	ldrh	r3, [r4, #302]
	muls	r3, r2, r3
	ldrh	r2, [r4, #308]
	adds	r2, r2, #9
	lsrs	r3, r3, r2
	adds	r3, r3, #2
	strh	r3, [r4, #336]	@ movhi
	movs	r3, #32
	strh	r3, [r4, #338]	@ movhi
	ldrh	r3, [r4, #320]
	adds	r3, r3, #3
	strh	r3, [r4, #320]	@ movhi
	ldr	r3, [r4, #328]
	adds	r3, r3, #3
	str	r3, [r4, #328]
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L16:
	cmp	r1, #8
	bne	.L17
	cmp	r0, #1
	itttt	eq
	lsreq	r3, r3, #1
	strheq	r3, [r4, #242]	@ movhi
	moveq	r3, #2
	strheq	r3, [r4, #256]	@ movhi
	b	.L17
.L27:
	.align	2
.L26:
	.word	.LANCHOR0
	.word	.LANCHOR0+260
	.fnend
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	1
	.global	IsBlkInVendorPart
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L35
	ldrh	r3, [r2, #344]
	cbz	r3, .L34
	ldr	r3, [r2, #348]
	ldrh	r2, [r2, #320]
	add	r2, r3, r2, lsl #1
.L30:
	cmp	r3, r2
	bne	.L31
.L34:
	movs	r0, #0
	bx	lr
.L31:
	ldrh	r1, [r3], #2
	cmp	r0, r1
	bne	.L30
	movs	r0, #1
	bx	lr
.L36:
	.align	2
.L35:
	.word	.LANCHOR0
	.fnend
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	1
	.global	FtlCacheWriteBack
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r0, #0
	bx	lr
	.fnend
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	1
	.global	sftl_get_density
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_get_density, %function
sftl_get_density:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L39
	ldr	r0, [r3, #340]
	bx	lr
.L40:
	.align	2
.L39:
	.word	.LANCHOR0
	.fnend
	.size	sftl_get_density, .-sftl_get_density
	.global	__aeabi_uidivmod
	.align	1
	.global	FtlBbmMapBadBlock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r5, r0
	ldr	r4, .L42
	ldrh	r7, [r4, #298]
	mov	r1, r7
	bl	__aeabi_uidiv
	uxth	r6, r0
	mov	r1, r7
	mov	r0, r5
	bl	__aeabi_uidivmod
	add	r2, r4, r6, lsl #2
	uxth	r3, r1
	ldr	r2, [r2, #380]
	lsrs	r1, r3, #5
	and	r7, r3, #31
	movs	r0, #1
	lsls	r0, r0, r7
	ldr	r7, [r2, r1, lsl #2]
	orrs	r0, r0, r7
	str	r0, [r2, r1, lsl #2]
	mov	r2, r6
	str	r0, [sp]
	mov	r1, r5
	ldr	r0, .L42+4
	bl	sftl_printk
	ldrh	r3, [r4, #358]
	movs	r0, #0
	adds	r3, r3, #1
	strh	r3, [r4, #358]	@ movhi
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L43:
	.align	2
.L42:
	.word	.LANCHOR0
	.word	.LC7
	.fnend
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.align	1
	.global	FtlBbmIsBadBlock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlBbmIsBadBlock, %function
FtlBbmIsBadBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r7, r0
	ldr	r5, .L45
	ldrh	r6, [r5, #298]
	mov	r1, r6
	bl	__aeabi_uidivmod
	mov	r0, r7
	uxth	r4, r1
	mov	r1, r6
	bl	__aeabi_uidiv
	uxth	r0, r0
	lsrs	r2, r4, #5
	add	r5, r5, r0, lsl #2
	and	r4, r4, #31
	ldr	r3, [r5, #380]
	ldr	r0, [r3, r2, lsl #2]
	lsrs	r0, r0, r4
	and	r0, r0, #1
	pop	{r3, r4, r5, r6, r7, pc}
.L46:
	.align	2
.L45:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbmIsBadBlock, .-FtlBbmIsBadBlock
	.align	1
	.global	FtlBbtInfoPrint
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
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
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, .L49
	movw	r3, #65535
	movs	r2, #16
	movs	r1, #255
	strh	r3, [r0, #352]	@ movhi
	movs	r3, #0
	strh	r3, [r0, #358]	@ movhi
	add	r0, r0, #364
	b	memset
.L50:
	.align	2
.L49:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	1
	.global	FtlBbtCalcTotleCnt
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L58
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r5, #0
	mov	r4, r5
	ldrh	r2, [r3, #298]
	ldrh	r6, [r3, #254]
	muls	r6, r2, r6
.L52:
	uxth	r0, r5
	cmp	r0, r6
	blt	.L54
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L54:
	bl	FtlBbmIsBadBlock
	cbz	r0, .L53
	adds	r4, r4, #1
	uxth	r4, r4
.L53:
	adds	r5, r5, #1
	b	.L52
.L59:
	.align	2
.L58:
	.word	.LANCHOR0
	.fnend
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	1
	.global	V2P_block
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	V2P_block, %function
V2P_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, r1
	ldr	r4, .L61
	mov	r7, r0
	ldrh	r6, [r4, #256]
	mov	r1, r6
	bl	__aeabi_uidiv
	ldrh	r4, [r4, #298]
	smulbb	r5, r6, r5
	mov	r1, r6
	smulbb	r4, r4, r0
	mov	r0, r7
	bl	__aeabi_uidivmod
	adds	r0, r5, r1
	add	r0, r0, r4
	uxth	r0, r0
	pop	{r3, r4, r5, r6, r7, pc}
.L62:
	.align	2
.L61:
	.word	.LANCHOR0
	.fnend
	.size	V2P_block, .-V2P_block
	.align	1
	.global	P2V_plane
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	P2V_plane, %function
P2V_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L64
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r5, [r3, #256]
	ldrh	r1, [r3, #298]
	bl	__aeabi_uidiv
	mov	r1, r5
	smulbb	r4, r0, r5
	mov	r0, r6
	bl	__aeabi_uidivmod
	add	r1, r1, r4
	uxth	r0, r1
	pop	{r4, r5, r6, pc}
.L65:
	.align	2
.L64:
	.word	.LANCHOR0
	.fnend
	.size	P2V_plane, .-P2V_plane
	.align	1
	.global	P2V_block_in_plane
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	P2V_block_in_plane, %function
P2V_block_in_plane:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	ldr	r4, .L67
	ldrh	r1, [r4, #298]
	bl	__aeabi_uidivmod
	uxth	r0, r1
	ldrh	r1, [r4, #256]
	bl	__aeabi_uidiv
	uxth	r0, r0
	pop	{r4, pc}
.L68:
	.align	2
.L67:
	.word	.LANCHOR0
	.fnend
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	1
	.global	ftl_cmp_data_ver
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, r1
	bls	.L70
	subs	r0, r0, r1
	cmp	r0, #-2147483648
	ite	hi
	movhi	r0, #0
	movls	r0, #1
	bx	lr
.L70:
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
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L73
	mov	r1, #2048
	push	{r4, lr}
	.save {r4, lr}
	movs	r4, #0
	strh	r0, [r3, #412]	@ movhi
	add	r0, r3, #420
	strh	r4, [r3, #414]	@ movhi
	strh	r4, [r3, #416]	@ movhi
	strh	r4, [r3, #418]	@ movhi
	bl	__memzero
	mov	r0, r4
	pop	{r4, pc}
.L74:
	.align	2
.L73:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	1
	.global	FtlFreeSysBlkQueueEmpty
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L76
	ldrh	r0, [r3, #418]
	clz	r0, r0
	lsrs	r0, r0, #5
	bx	lr
.L77:
	.align	2
.L76:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	1
	.global	FtlFreeSysBlkQueueFull
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L79
	ldrh	r0, [r3, #418]
	sub	r3, r0, #1024
	rsbs	r0, r3, #0
	adcs	r0, r0, r3
	bx	lr
.L80:
	.align	2
.L79:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	1
	.global	FtlFreeSysBLkSort
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L90
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldrh	r2, [r3, #418]
	cbz	r2, .L81
	ldrh	r5, [r3, #2496]
	movs	r0, #0
	ldrh	r1, [r3, #414]
	mov	r6, r0
	ldrh	r2, [r3, #416]
	and	r5, r5, #31
.L83:
	uxth	r4, r0
	adds	r0, r0, #1
	cmp	r5, r4
	bgt	.L84
	cbz	r6, .L81
	strh	r1, [r3, #414]	@ movhi
	strh	r2, [r3, #416]	@ movhi
.L81:
	pop	{r4, r5, r6, pc}
.L84:
	add	r4, r3, r1, lsl #1
	adds	r1, r1, #1
	ubfx	r1, r1, #0, #10
	ldrh	r6, [r4, #420]
	add	r4, r3, r2, lsl #1
	strh	r6, [r4, #420]	@ movhi
	movs	r6, #1
	add	r2, r2, r6
	ubfx	r2, r2, #0, #10
	b	.L83
.L91:
	.align	2
.L90:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	1
	.global	IsInFreeQueue
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	IsInFreeQueue, %function
IsInFreeQueue:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L98
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	ldrh	r4, [r3, #418]
	cmp	r4, #1024
	beq	.L96
	ldrh	r5, [r3, #414]
	movs	r1, #0
.L94:
	cmp	r1, r4
	bcc	.L95
.L96:
	movs	r0, #0
	pop	{r4, r5, pc}
.L95:
	adds	r2, r1, r5
	ubfx	r2, r2, #0, #10
	add	r2, r3, r2, lsl #1
	ldrh	r2, [r2, #420]
	cmp	r2, r0
	beq	.L97
	adds	r1, r1, #1
	b	.L94
.L97:
	movs	r0, #1
	pop	{r4, r5, pc}
.L99:
	.align	2
.L98:
	.word	.LANCHOR0
	.fnend
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	1
	.global	insert_data_list
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	insert_data_list, %function
insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r2, .L115
	ldrh	r3, [r2, #240]
	mov	r1, r2
	cmp	r3, r0
	bls	.L102
	movs	r5, #6
	ldr	lr, [r2, #2516]
	muls	r5, r0, r5
	movw	r3, #65535
	add	r4, lr, r5
	strh	r3, [r4, #2]	@ movhi
	strh	r3, [lr, r5]	@ movhi
	ldr	r3, [r2, #2520]
	cbnz	r3, .L103
	str	r4, [r2, #2520]
.L102:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L103:
	ldrh	r7, [r4, #4]
	ldr	r9, [r2, #72]
	ldrh	r2, [r9, r0, lsl #1]
	cbz	r7, .L110
	muls	r2, r7, r2
.L114:
	ldr	r8, [r1, #2516]
	movw	r10, #65535
	str	r2, [sp]
	sub	r2, r3, r8
	asrs	r6, r2, #1
	ldr	r2, .L115+4
	muls	r2, r6, r2
	ldrh	r6, [r1, #240]
	str	r6, [sp, #4]
	movs	r6, #0
	uxth	r2, r2
.L108:
	adds	r6, r6, #1
	ldr	r7, [sp, #4]
	uxth	r6, r6
	cmp	r6, r7
	bhi	.L102
	cmp	r0, r2
	beq	.L102
	ldrh	ip, [r3, #4]
	cmp	ip, #0
	beq	.L106
	ldrh	fp, [r9, r2, lsl #1]
	ldr	r7, [sp]
	mul	ip, ip, fp
	cmp	ip, r7
	bcs	.L106
	ldrh	ip, [r3]
	cmp	ip, r10
	bne	.L107
	strh	r2, [r4, #2]	@ movhi
	strh	r0, [r3]	@ movhi
	str	r4, [r1, #2524]
	b	.L102
.L110:
	mov	r2, #-1
	b	.L114
.L107:
	movs	r3, #6
	mov	r2, ip
	mla	r3, r3, ip, r8
	b	.L108
.L106:
	strh	r2, [lr, r5]	@ movhi
	ldrh	r2, [r3, #2]
	strh	r2, [r4, #2]	@ movhi
	ldr	r2, [r1, #2520]
	cmp	r3, r2
	ittte	ne
	ldrhne	r4, [r3, #2]
	movne	r2, #6
	ldrne	r1, [r1, #2516]
	strheq	r0, [r3, #2]	@ movhi
	iteee	eq
	streq	r4, [r1, #2520]
	mulne	r2, r2, r4
	strhne	r0, [r1, r2]	@ movhi
	strhne	r0, [r3, #2]	@ movhi
	b	.L102
.L116:
	.align	2
.L115:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	insert_data_list, .-insert_data_list
	.align	1
	.global	INSERT_DATA_LIST
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	INSERT_DATA_LIST, %function
INSERT_DATA_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	insert_data_list
	ldr	r2, .L119
	ldrh	r3, [r2, #2528]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r2, #2528]	@ movhi
	ldrh	r2, [r2, #240]
	cmp	r2, r3
	bcs	.L117
	movs	r2, #214
	ldr	r1, .L119+4
	ldr	r0, .L119+8
	pop	{r3, lr}
	b	sftl_printk
.L117:
	pop	{r3, pc}
.L120:
	.align	2
.L119:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC8
	.fnend
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	1
	.global	insert_free_list
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	insert_free_list, %function
insert_free_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movw	r4, #65535
	cmp	r0, r4
	beq	.L122
	ldr	r2, .L128
	movs	r1, #6
	mul	r7, r1, r0
	ldr	ip, [r2, #2516]
	mov	r5, r2
	add	r6, ip, r7
	strh	r4, [r6, #2]	@ movhi
	strh	r4, [ip, r7]	@ movhi
	ldr	r3, [r2, #2532]
	cbnz	r3, .L123
	str	r6, [r2, #2532]
.L122:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L123:
	ldr	lr, [r2, #2516]
	ldr	r8, [r2, #2536]
	sub	r2, r3, lr
	asr	r10, r2, #1
	ldr	r2, .L128+4
	ldrh	r9, [r8, r0, lsl #1]
	mul	r2, r2, r10
	mov	r10, r4
	uxth	r2, r2
.L126:
	ldrh	r4, [r8, r2, lsl #1]
	cmp	r4, r9
	bcs	.L124
	ldrh	r4, [r3]
	cmp	r4, r10
	bne	.L125
	strh	r2, [r6, #2]	@ movhi
	strh	r0, [r3]	@ movhi
	b	.L122
.L125:
	mla	r3, r1, r4, lr
	mov	r2, r4
	b	.L126
.L124:
	ldrh	r1, [r3, #2]
	strh	r1, [r6, #2]	@ movhi
	strh	r2, [ip, r7]	@ movhi
	ldr	r2, [r5, #2532]
	cmp	r3, r2
	ittte	ne
	ldrhne	r4, [r3, #2]
	movne	r2, #6
	ldrne	r1, [r5, #2516]
	strheq	r0, [r3, #2]	@ movhi
	iteee	eq
	streq	r6, [r5, #2532]
	mulne	r2, r2, r4
	strhne	r0, [r1, r2]	@ movhi
	strhne	r0, [r3, #2]	@ movhi
	b	.L122
.L129:
	.align	2
.L128:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	insert_free_list, .-insert_free_list
	.align	1
	.global	INSERT_FREE_LIST
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	INSERT_FREE_LIST, %function
INSERT_FREE_LIST:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	insert_free_list
	ldr	r2, .L132
	ldrh	r3, [r2, #224]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r2, #224]	@ movhi
	ldrh	r2, [r2, #240]
	cmp	r2, r3
	bcs	.L130
	movs	r2, #207
	ldr	r1, .L132+4
	ldr	r0, .L132+8
	pop	{r3, lr}
	b	sftl_printk
.L130:
	pop	{r3, pc}
.L133:
	.align	2
.L132:
	.word	.LANCHOR0
	.word	.LANCHOR1+17
	.word	.LC8
	.fnend
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	1
	.global	List_remove_node
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	List_remove_node, %function
List_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movs	r6, #6
	ldr	r4, .L140
	muls	r6, r1, r6
	movw	r3, #65535
	mov	r8, r0
	ldr	r7, [r4, #2516]
	adds	r5, r7, r6
	ldrh	r2, [r5, #2]
	cmp	r2, r3
	bne	.L135
	ldr	r3, [r0]
	cmp	r5, r3
	beq	.L135
	mov	r2, #372
	ldr	r1, .L140+4
	ldr	r0, .L140+8
	bl	sftl_printk
.L135:
	ldr	r3, [r8]
	movw	r1, #65535
	cmp	r5, r3
	ldrh	r3, [r7, r6]
	bne	.L136
	cmp	r3, r1
	ittee	ne
	ldrne	r0, [r4, #2516]
	movne	r2, #6
	moveq	r3, #0
	streq	r3, [r8]
	ittt	ne
	mlane	r3, r2, r3, r0
	strne	r3, [r8]
	strhne	r1, [r3, #2]	@ movhi
.L138:
	movw	r3, #65535
	movs	r0, #0
	strh	r3, [r7, r6]	@ movhi
	strh	r3, [r5, #2]	@ movhi
	pop	{r4, r5, r6, r7, r8, pc}
.L136:
	cmp	r3, r1
	ldrh	r1, [r5, #2]
	bne	.L139
	cmp	r1, r3
	beq	.L138
	movs	r2, #6
	ldr	r0, [r4, #2516]
	muls	r1, r2, r1
	strh	r3, [r0, r1]	@ movhi
	b	.L138
.L139:
	ldr	r0, [r4, #2516]
	movs	r2, #6
	mla	r3, r2, r3, r0
	strh	r1, [r3, #2]	@ movhi
	ldrh	r0, [r5, #2]
	ldrh	r1, [r7, r6]
	ldr	r3, [r4, #2516]
	muls	r2, r0, r2
	strh	r1, [r3, r2]	@ movhi
	b	.L138
.L141:
	.align	2
.L140:
	.word	.LANCHOR0
	.word	.LANCHOR1+34
	.word	.LC8
	.fnend
	.size	List_remove_node, .-List_remove_node
	.align	1
	.global	List_pop_index_node
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	List_pop_index_node, %function
List_pop_index_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r3, [r0]
	cbz	r3, .L148
	ldr	r2, .L149
	movw	r5, #65535
	movs	r6, #6
	ldr	r2, [r2, #2516]
.L144:
	cbnz	r1, .L145
.L147:
	ldr	r4, .L149+4
	subs	r3, r3, r2
	asrs	r3, r3, #1
	muls	r4, r3, r4
	uxth	r1, r4
	bl	List_remove_node
	uxth	r0, r4
	pop	{r4, r5, r6, pc}
.L145:
	ldrh	r4, [r3]
	cmp	r4, r5
	beq	.L147
	subs	r1, r1, #1
	mla	r3, r6, r4, r2
	uxth	r1, r1
	b	.L144
.L148:
	movw	r0, #65535
	pop	{r4, r5, r6, pc}
.L150:
	.align	2
.L149:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	List_pop_index_node, .-List_pop_index_node
	.align	1
	.global	List_pop_head_node
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	List_pop_head_node, %function
List_pop_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r1, #0
	b	List_pop_index_node
	.fnend
	.size	List_pop_head_node, .-List_pop_head_node
	.align	1
	.global	List_get_gc_head_node
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L158
	push	{r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #2520]
	cbz	r3, .L157
	ldr	r1, [r2, #2516]
	movs	r4, #6
	movw	r2, #65535
.L154:
	cbz	r0, .L155
	ldrh	r3, [r3]
	cmp	r3, r2
	bne	.L156
.L157:
	movw	r0, #65535
	pop	{r4, pc}
.L156:
	subs	r0, r0, #1
	mla	r3, r4, r3, r1
	uxth	r0, r0
	b	.L154
.L155:
	ldr	r0, .L158+4
	subs	r3, r3, r1
	asrs	r3, r3, #1
	muls	r3, r0, r3
	uxth	r0, r3
	pop	{r4, pc}
.L159:
	.align	2
.L158:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	1
	.global	List_update_data_list
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	List_update_data_list, %function
List_update_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, r0
	ldr	r4, .L171
	ldrh	r3, [r4, #24]
	cmp	r3, r0
	beq	.L162
	ldrh	r3, [r4, #76]
	cmp	r3, r0
	beq	.L162
	ldrh	r3, [r4, #124]
	cmp	r3, r0
	beq	.L162
	movs	r7, #6
	ldr	r9, [r4, #2516]
	muls	r7, r0, r7
	ldr	r3, [r4, #2520]
	add	r8, r9, r7
	cmp	r8, r3
	beq	.L162
	ldrh	r6, [r8, #4]
	ldr	r3, [r4, #72]
	ldrh	r3, [r3, r0, lsl #1]
	cmp	r6, #0
	beq	.L169
	muls	r6, r3, r6
.L164:
	ldrh	r3, [r8, #2]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L165
	ldrh	r2, [r9, r7]
	cmp	r2, r3
	bne	.L165
	movw	r2, #463
	ldr	r1, .L171+4
	ldr	r0, .L171+8
	bl	sftl_printk
.L165:
	ldrh	r3, [r8, #2]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L166
	ldrh	r2, [r9, r7]
	cmp	r2, r3
	beq	.L162
.L166:
	movs	r2, #6
	muls	r2, r3, r2
	ldr	r3, .L171+12
	asrs	r1, r2, #1
	muls	r3, r1, r3
	ldr	r1, [r4, #72]
	ldrh	r0, [r1, r3, lsl #1]
	ldr	r1, [r4, #2516]
	add	r2, r2, r1
	ldrh	r3, [r2, #4]
	cbz	r3, .L170
	muls	r3, r0, r3
.L167:
	cmp	r6, r3
	bcs	.L162
	mov	r1, r5
	ldr	r0, .L171+16
	bl	List_remove_node
	ldrh	r3, [r4, #2528]
	cbnz	r3, .L168
	mov	r2, #474
	ldr	r1, .L171+4
	ldr	r0, .L171+8
	bl	sftl_printk
.L168:
	ldrh	r3, [r4, #2528]
	mov	r0, r5
	subs	r3, r3, #1
	strh	r3, [r4, #2528]	@ movhi
	bl	INSERT_DATA_LIST
.L162:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L169:
	mov	r6, #-1
	b	.L164
.L170:
	mov	r3, #-1
	b	.L167
.L172:
	.align	2
.L171:
	.word	.LANCHOR0
	.word	.LANCHOR1+51
	.word	.LC8
	.word	-1431655765
	.word	.LANCHOR0+2520
	.fnend
	.size	List_update_data_list, .-List_update_data_list
	.align	1
	.global	select_l2p_ram_region
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movs	r1, #0
	ldr	r5, .L183
	movs	r0, #12
	movw	r6, #65535
	ldrh	r2, [r5, #338]
	ldr	r3, [r5, #2540]
.L174:
	uxth	r4, r1
	cmp	r4, r2
	bcc	.L176
	mov	r4, r2
	movs	r1, #0
	mov	r7, #-2147483648
	mov	ip, #12
.L177:
	uxth	r6, r1
	cmp	r6, r2
	bcc	.L179
	cmp	r4, r2
	bcc	.L175
	ldrh	r7, [r5, #2544]
	mov	r4, r2
	movs	r1, #0
	mov	r0, #-1
.L180:
	uxth	r5, r1
	cmp	r5, r2
	bcc	.L182
	cmp	r4, r2
	bcc	.L175
	movw	r2, #789
	ldr	r1, .L183+4
	ldr	r0, .L183+8
	bl	sftl_printk
	b	.L175
.L176:
	adds	r1, r1, #1
	mla	r7, r0, r1, r3
	ldrh	r7, [r7, #-12]
	cmp	r7, r6
	bne	.L174
.L175:
	mov	r0, r4
	pop	{r3, r4, r5, r6, r7, pc}
.L179:
	mla	r0, ip, r1, r3
	ldr	r0, [r0, #4]
	cmp	r0, #0
	blt	.L178
	cmp	r7, r0
	itt	hi
	movhi	r7, r0
	movhi	r4, r6
.L178:
	adds	r1, r1, #1
	b	.L177
.L182:
	ldr	r6, [r3, #4]
	cmp	r0, r6
	bls	.L181
	ldrh	ip, [r3]
	cmp	ip, r7
	itt	ne
	movne	r0, r6
	movne	r4, r5
.L181:
	adds	r1, r1, #1
	adds	r3, r3, #12
	b	.L180
.L184:
	.align	2
.L183:
	.word	.LANCHOR0
	.word	.LANCHOR1+73
	.word	.LC8
	.fnend
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	1
	.global	FtlUpdateVaildLpn
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L191
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	mov	r3, r1
	ldrh	r2, [r1, #2546]
	cmp	r2, #4
	bhi	.L186
	cbnz	r0, .L186
	adds	r2, r2, #1
	strh	r2, [r1, #2546]	@ movhi
	pop	{r4, r5, pc}
.L186:
	movs	r2, #0
	ldrh	r1, [r3, #240]
	strh	r2, [r3, #2546]	@ movhi
	movw	r5, #65535
	str	r2, [r3, #2548]
	ldr	r2, [r3, #72]
	add	r1, r2, r1, lsl #1
.L187:
	cmp	r2, r1
	bne	.L189
	pop	{r4, r5, pc}
.L189:
	ldrh	r4, [r2], #2
	cmp	r4, r5
	ittt	ne
	ldrne	r0, [r3, #2548]
	addne	r0, r0, r4
	strne	r0, [r3, #2548]
	b	.L187
.L192:
	.align	2
.L191:
	.word	.LANCHOR0
	.fnend
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	1
	.global	ftl_sb_update_avl_pages
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movs	r3, #0
	push	{r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	strh	r3, [r0, #4]	@ movhi
	add	r4, r0, r2, lsl #1
	movw	r7, #65535
	ldr	r3, .L200
	adds	r4, r4, #14
	ldrh	r5, [r3, #232]
.L194:
	cmp	r2, r5
	bcc	.L196
	ldrh	r3, [r3, #302]
	add	r4, r0, #16
	movw	r6, #65535
	subs	r3, r3, #1
	subs	r1, r3, r1
	movs	r3, #0
	uxth	r1, r1
.L197:
	uxth	r2, r3
	cmp	r5, r2
	bhi	.L199
	pop	{r4, r5, r6, r7, pc}
.L196:
	ldrh	r6, [r4, #2]!
	adds	r2, r2, #1
	uxth	r2, r2
	cmp	r6, r7
	ittt	ne
	ldrhne	r6, [r0, #4]
	addne	r6, r6, #1
	strhne	r6, [r0, #4]	@ movhi
	b	.L194
.L199:
	ldrh	r2, [r4], #2
	adds	r3, r3, #1
	cmp	r2, r6
	ittt	ne
	ldrhne	r2, [r0, #4]
	addne	r2, r2, r1
	strhne	r2, [r0, #4]	@ movhi
	b	.L197
.L201:
	.align	2
.L200:
	.word	.LANCHOR0
	.fnend
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.align	1
	.global	FtlSlcSuperblockCheck
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldrh	r3, [r0, #4]
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	cbz	r3, .L202
	ldrh	r2, [r0]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L202
	ldrb	r2, [r0, #6]	@ zero_extendqisi2
	movs	r5, #0
	adds	r2, r2, #8
	ldrh	r1, [r0, r2, lsl #1]
	ldr	r2, .L208
	ldrh	r4, [r2, #232]
	mov	r2, r3
.L205:
	cmp	r1, r2
	beq	.L207
.L202:
	pop	{r4, r5, pc}
.L207:
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	adds	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, r4
	strb	r3, [r0, #6]
	itttt	eq
	ldrheq	r3, [r0, #2]
	strbeq	r5, [r0, #6]
	addeq	r3, r3, #1
	strheq	r3, [r0, #2]	@ movhi
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	adds	r3, r3, #8
	ldrh	r1, [r0, r3, lsl #1]
	b	.L205
.L209:
	.align	2
.L208:
	.word	.LANCHOR0
	.fnend
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	1
	.global	make_superblock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	make_superblock, %function
make_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r0
	ldr	r6, .L216
	ldrh	r2, [r0]
	ldrh	r3, [r6, #240]
	cmp	r2, r3
	bcc	.L211
	movw	r2, #2150
	ldr	r1, .L216+4
	ldr	r0, .L216+8
	bl	sftl_printk
.L211:
	ldrh	r9, [r6, #232]
	add	r7, r4, #16
	ldr	r10, .L216+12
	movw	r8, #65535
	movs	r5, #0
	strh	r5, [r4, #4]	@ movhi
	strb	r5, [r4, #7]
.L212:
	uxth	r3, r5
	ldrh	r1, [r4]
	cmp	r9, r3
	bhi	.L214
	ldrh	r2, [r6, #302]
	movs	r0, #0
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	smulbb	r3, r3, r2
	strh	r3, [r4, #4]	@ movhi
	movs	r3, #0
	strb	r3, [r4, #9]
	ldr	r3, [r6, #2536]
	ldrh	r2, [r3, r1, lsl #1]
	movw	r3, #10000
	cmp	r2, r3
	itt	hi
	movhi	r3, #1
	strbhi	r3, [r4, #9]
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L214:
	ldrb	r0, [r10, r5]	@ zero_extendqisi2
	bl	V2P_block
	strh	r8, [r7]	@ movhi
	mov	fp, r0
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L213
	strh	fp, [r7]	@ movhi
	ldrb	r3, [r4, #7]	@ zero_extendqisi2
	adds	r3, r3, #1
	strb	r3, [r4, #7]
.L213:
	adds	r5, r5, #1
	adds	r7, r7, #2
	b	.L212
.L217:
	.align	2
.L216:
	.word	.LANCHOR0
	.word	.LANCHOR1+95
	.word	.LC8
	.word	.LANCHOR0+260
	.fnend
	.size	make_superblock, .-make_superblock
	.align	1
	.global	update_multiplier_value
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	update_multiplier_value, %function
update_multiplier_value:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movs	r5, #0
	ldr	r6, .L224
	mov	r7, r0
	mov	r4, r5
	ldrh	r8, [r6, #232]
	add	r10, r6, #260
	ldrh	r9, [r6, #302]
.L219:
	uxth	r3, r5
	cmp	r8, r3
	bhi	.L221
	cbz	r4, .L223
	mov	r1, r4
	mov	r0, #32768
	bl	__aeabi_idiv
.L222:
	ldr	r1, [r6, #2516]
	movs	r3, #6
	mla	r1, r3, r7, r1
	strh	r0, [r1, #4]	@ movhi
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L221:
	mov	r1, r7
	ldrb	r0, [r10, r5]	@ zero_extendqisi2
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L220
	add	r4, r4, r9
	uxth	r4, r4
.L220:
	adds	r5, r5, #1
	b	.L219
.L223:
	mov	r0, r4
	b	.L222
.L225:
	.align	2
.L224:
	.word	.LANCHOR0
	.fnend
	.size	update_multiplier_value, .-update_multiplier_value
	.align	1
	.global	GetFreeBlockMinEraseCount
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L229
	ldr	r0, [r2, #2532]
	cbz	r0, .L227
	ldr	r3, [r2, #2516]
	subs	r0, r0, r3
	ldr	r3, .L229+4
	asrs	r0, r0, #1
	muls	r0, r3, r0
	ldr	r3, [r2, #2536]
	uxth	r0, r0
	ldrh	r0, [r3, r0, lsl #1]
.L227:
	bx	lr
.L230:
	.align	2
.L229:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	1
	.global	GetFreeBlockMaxEraseCount
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r1, .L239
	push	{r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	ldr	r3, [r1, #2532]
	cbz	r3, .L237
	ldrh	r2, [r1, #224]
	movs	r6, #6
	ldr	r4, [r1, #2516]
	movw	r7, #65535
	rsb	r2, r2, r2, lsl #3
	subs	r3, r3, r4
	asrs	r3, r3, #1
	asrs	r2, r2, #3
	cmp	r0, r2
	it	gt
	uxthgt	r0, r2
	ldr	r2, .L239+4
	muls	r3, r2, r3
	movs	r2, #0
	uxth	r3, r3
.L234:
	uxth	r5, r2
	cmp	r0, r5
	bls	.L236
	mul	r5, r6, r3
	adds	r2, r2, #1
	ldrh	r5, [r4, r5]
	cmp	r5, r7
	bne	.L238
.L236:
	ldr	r2, [r1, #2536]
	ldrh	r0, [r2, r3, lsl #1]
	pop	{r4, r5, r6, r7, pc}
.L238:
	mov	r3, r5
	b	.L234
.L237:
	mov	r0, r3
	pop	{r4, r5, r6, r7, pc}
.L240:
	.align	2
.L239:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	1
	.global	FtlPrintInfo2buf
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r0
	ldr	r5, .L253
	add	r4, r6, #12
	.pad #20
	sub	sp, sp, #20
	ldr	r1, .L253+4
	bl	strcpy
	ldr	r3, [r5, #316]
	movs	r1, #64
	ldr	r2, .L253+8
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r1, .L253+12
	mov	r0, r4
	adds	r4, r4, #10
	bl	strcpy
	ldr	r3, [r5, #2552]
	movs	r1, #64
	ldr	r2, .L253+16
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2548]
	ldr	r2, .L253+20
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2556]
	ldr	r2, .L253+24
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2560]
	ldr	r2, .L253+28
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2564]
	ldr	r2, .L253+32
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2568]
	ldr	r2, .L253+36
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2572]
	ldr	r2, .L253+40
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2576]
	ldr	r2, .L253+44
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	ldr	r3, [r5, #2580]
	add	r4, r4, r0
	ldr	r2, .L253+48
	movs	r1, #64
	mov	r0, r4
	lsrs	r3, r3, #11
	bl	snprintf
	ldr	r3, [r5, #2584]
	add	r4, r4, r0
	ldr	r2, .L253+52
	movs	r1, #64
	mov	r0, r4
	lsrs	r3, r3, #11
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2588]
	ldr	r2, .L253+56
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2592]
	ldr	r2, .L253+60
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #358]
	ldr	r2, .L253+64
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #224]
	ldr	r2, .L253+68
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2596]
	ldr	r2, .L253+72
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2600]
	ldr	r2, .L253+76
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2604]
	ldr	r2, .L253+80
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2608]
	ldr	r2, .L253+84
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2612]
	ldr	r2, .L253+88
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2616]
	ldr	r2, .L253+92
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #2498]
	ldr	r2, .L253+96
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #2496]
	ldr	r2, .L253+100
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #340]
	ldr	r2, .L253+104
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #332]
	ldr	r2, .L253+108
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #228]
	ldr	r2, .L253+112
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #418]
	ldr	r2, .L253+116
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #240]
	ldr	r2, .L253+120
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #2620]
	ldr	r2, .L253+124
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #244]
	ldr	r2, .L253+128
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #2624]
	ldr	r2, .L253+132
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #352]
	ldr	r2, .L253+136
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #26]
	ldr	r2, .L253+140
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #30]	@ zero_extendqisi2
	ldr	r2, .L253+144
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #24]
	ldr	r2, .L253+148
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #32]	@ zero_extendqisi2
	ldr	r2, .L253+152
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #28]
	ldr	r2, .L253+156
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	ldrh	r2, [r5, #24]
	add	r4, r4, r0
	ldr	r3, [r5, #72]
	movs	r1, #64
	mov	r0, r4
	ldrh	r3, [r3, r2, lsl #1]
	ldr	r2, .L253+160
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #78]
	ldr	r2, .L253+164
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #82]	@ zero_extendqisi2
	ldr	r2, .L253+168
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #76]
	ldr	r2, .L253+172
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #84]	@ zero_extendqisi2
	ldr	r2, .L253+176
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #80]
	ldr	r2, .L253+180
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	ldrh	r2, [r5, #76]
	add	r4, r4, r0
	ldr	r3, [r5, #72]
	movs	r1, #64
	mov	r0, r4
	ldrh	r3, [r3, r2, lsl #1]
	ldr	r2, .L253+184
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #126]
	ldr	r2, .L253+188
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #130]	@ zero_extendqisi2
	ldr	r2, .L253+192
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #124]
	ldr	r2, .L253+196
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #132]	@ zero_extendqisi2
	ldr	r2, .L253+200
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #128]
	ldr	r2, .L253+204
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #174]
	ldr	r2, .L253+208
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #178]	@ zero_extendqisi2
	ldr	r2, .L253+212
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #172]
	ldr	r2, .L253+216
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrb	r3, [r5, #180]	@ zero_extendqisi2
	ldr	r2, .L253+220
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #176]
	b	.L254
.L255:
	.align	2
.L253:
	.word	.LANCHOR0
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
.L254:
	ldr	r2, .L256
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	ldr	r3, [r5, #2716]
	add	r4, r4, r0
	ldr	r2, .L256+4
	movs	r1, #64
	mov	r0, r4
	str	r3, [sp, #4]
	ldr	r3, [r5, #2724]
	str	r3, [sp]
	ldr	r3, [r5, #2720]
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2712]
	ldr	r2, .L256+8
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #2736]
	ldr	r2, .L256+12
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #3152]
	ldr	r2, .L256+16
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #3154]
	ldr	r2, .L256+20
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldr	r3, [r5, #3156]
	ldr	r2, .L256+24
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r3, [r5, #3160]
	ldr	r2, .L256+28
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	bl	GetFreeBlockMinEraseCount
	ldr	r2, .L256+32
	mov	r3, r0
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	add	r4, r4, r0
	ldrh	r0, [r5, #224]
	bl	GetFreeBlockMaxEraseCount
	ldr	r2, .L256+36
	mov	r3, r0
	movs	r1, #64
	mov	r0, r4
	bl	snprintf
	ldr	r3, .L256+40
	add	r4, r4, r0
	ldr	r3, [r3]
	cmp	r3, #1
	beq	.L242
.L247:
	subs	r0, r4, r6
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L242:
	ldrh	r3, [r5, #172]
	movw	r2, #65535
	cmp	r3, r2
	beq	.L244
	ldr	r2, [r5, #72]
	mov	r0, r4
	movs	r1, #64
	ldrh	r3, [r2, r3, lsl #1]
	ldr	r2, .L256+44
	bl	snprintf
	add	r4, r4, r0
.L244:
	movs	r0, #0
	ldr	r9, .L256+52
	bl	List_get_gc_head_node
	uxth	r0, r0
	movs	r7, #0
	movw	fp, #65535
	mov	r10, #6
.L246:
	cmp	r0, fp
	beq	.L245
	ldr	r3, [r5, #2536]
	mul	r8, r10, r0
	mov	r2, r9
	movs	r1, #64
	ldrh	r3, [r3, r0, lsl #1]
	str	r3, [sp, #12]
	ldr	r3, [r5, #2516]
	add	r3, r3, r8
	ldrh	r3, [r3, #4]
	str	r3, [sp, #8]
	ldr	r3, [r5, #72]
	ldrh	r3, [r3, r0, lsl #1]
	stm	sp, {r0, r3}
	mov	r3, r7
	mov	r0, r4
	adds	r7, r7, #1
	bl	snprintf
	ldr	r3, [r5, #2516]
	cmp	r7, #16
	add	r4, r4, r0
	ldrh	r0, [r3, r8]
	bne	.L246
.L245:
	ldr	r2, [r5, #2516]
	movs	r7, #0
	ldr	r3, [r5, #2532]
	movw	r9, #65535
	ldr	fp, .L256+56
	mov	r10, #6
	subs	r3, r3, r2
	asrs	r2, r3, #1
	ldr	r3, .L256+48
	muls	r3, r2, r3
	uxth	r3, r3
.L248:
	cmp	r3, r9
	beq	.L247
	ldr	r2, [r5, #2536]
	mul	r8, r10, r3
	mov	r0, r4
	movs	r1, #64
	ldrh	r2, [r2, r3, lsl #1]
	str	r2, [sp, #8]
	ldr	r2, [r5, #2516]
	add	r2, r2, r8
	ldrh	r2, [r2, #4]
	str	r3, [sp]
	mov	r3, r7
	adds	r7, r7, #1
	str	r2, [sp, #4]
	mov	r2, fp
	bl	snprintf
	cmp	r7, #4
	add	r4, r4, r0
	beq	.L247
	ldr	r3, [r5, #2516]
	ldrh	r3, [r3, r8]
	b	.L248
.L257:
	.align	2
.L256:
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
	.word	-1431655765
	.word	.LC75
	.word	.LC76
	.fnend
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	1
	.global	rknand_proc_ftlread
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #2048
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r1
	blt	.L260
	ldr	r3, .L261
	movs	r1, #64
	ldr	r2, .L261+4
	mov	r0, r5
	bl	snprintf
	adds	r4, r5, r0
	mov	r0, r4
	bl	FtlPrintInfo2buf
	add	r0, r0, r4
	subs	r0, r0, r5
	pop	{r3, r4, r5, pc}
.L260:
	movs	r0, #0
	pop	{r3, r4, r5, pc}
.L262:
	.align	2
.L261:
	.word	.LC0
	.word	.LC77
	.fnend
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	1
	.global	GetSwlReplaceBlock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L287
	ldr	r2, [r4, #2604]
	ldr	r3, [r4, #2616]
	cmp	r2, r3
	bcs	.L264
	ldrh	r1, [r4, #240]
	movs	r3, #0
	ldr	r0, [r4, #2536]
	str	r3, [r4, #2596]
.L265:
	cmp	r3, r1
	bcc	.L266
	ldr	r5, [r4, #2596]
	mov	r0, r5
	bl	__aeabi_uidiv
	str	r0, [r4, #2604]
	ldr	r0, [r4, #2600]
	ldrh	r1, [r4, #292]
	subs	r0, r5, r0
	bl	__aeabi_uidiv
	str	r0, [r4, #2596]
.L267:
	ldr	r6, [r4, #2616]
	ldr	r7, [r4, #2604]
	add	r3, r6, #256
	cmp	r3, r7
	bls	.L272
	ldr	r2, [r4, #2612]
	add	r3, r6, #768
	cmp	r3, r2
	bls	.L272
.L274:
	movw	r5, #65535
.L273:
	mov	r0, r5
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L266:
	ldrh	r2, [r0, r3, lsl #1]
	adds	r3, r3, #1
	ldr	r5, [r4, #2596]
	add	r2, r2, r5
	str	r2, [r4, #2596]
	b	.L265
.L264:
	ldr	r3, [r4, #2612]
	cmp	r2, r3
	bls	.L267
	adds	r3, r3, #1
	str	r3, [r4, #2612]
	movs	r3, #0
.L269:
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	bcs	.L267
	ldr	r1, [r4, #2536]
	ldrh	r2, [r1, r3, lsl #1]
	adds	r2, r2, #1
	strh	r2, [r1, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	b	.L269
.L272:
	ldrh	r0, [r4, #224]
	add	r0, r0, r0, lsl #1
	ubfx	r0, r0, #2, #16
	bl	GetFreeBlockMaxEraseCount
	add	r3, r6, #64
	mov	r9, r0
	cmp	r0, r3
	bcc	.L274
	ldr	r3, [r4, #2520]
	cmp	r3, #0
	beq	.L274
	ldrh	r2, [r4, #240]
	movw	r1, #65535
	ldr	lr, [r4, #2516]
	movs	r0, #0
	ldr	r8, [r4, #2536]
	mov	r10, r1
	str	r2, [sp, #20]
	mov	fp, #6
	mov	r2, r1
.L275:
	ldrh	ip, [r3]
	cmp	ip, r10
	bne	.L278
	mov	r5, r2
.L277:
	movw	r3, #65535
	cmp	r5, r3
	beq	.L274
	ldrh	r10, [r8, r5, lsl #1]
	lsl	fp, r5, #1
	cmp	r6, r10
	bcs	.L279
	bl	GetFreeBlockMinEraseCount
	cmp	r6, r0
	it	cc
	strcc	r1, [r4, #2616]
.L279:
	cmp	r7, r10
	bls	.L274
	add	r3, r10, #128
	cmp	r9, r3
	ble	.L274
	add	r3, r10, #256
	cmp	r7, r3
	bhi	.L280
	ldr	r3, [r4, #2612]
	add	r10, r10, #768
	cmp	r10, r3
	bcs	.L274
.L280:
	str	r9, [sp, #8]
	mov	r2, r7
	ldrh	r3, [r8, fp]
	mov	r1, r5
	ldr	r0, .L287+4
	str	r3, [sp, #4]
	ldr	r3, [r4, #72]
	ldrh	r3, [r3, fp]
	str	r3, [sp]
	ldr	r3, [r4, #2612]
	bl	sftl_printk
	movs	r3, #1
	str	r3, [r4, #3164]
	b	.L273
.L278:
	adds	r0, r0, #1
	ldr	r5, [sp, #20]
	uxth	r0, r0
	cmp	r0, r5
	bhi	.L274
	ldrh	r5, [r3, #4]
	cbz	r5, .L276
	ldr	r5, .L287+8
	sub	r3, r3, lr
	asrs	r3, r3, #1
	muls	r3, r5, r3
	uxth	r5, r3
	ldrh	r3, [r8, r5, lsl #1]
	cmp	r6, r3
	bcs	.L277
	cmp	r1, r3
	itt	hi
	movhi	r1, r3
	movhi	r2, r5
.L276:
	mla	r3, fp, ip, lr
	b	.L275
.L288:
	.align	2
.L287:
	.word	.LANCHOR0
	.word	.LC78
	.word	-1431655765
	.fnend
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.align	1
	.global	free_data_superblock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	free_data_superblock, %function
free_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r2, #65535
	push	{r3, lr}
	.save {r3, lr}
	cmp	r0, r2
	beq	.L290
	ldr	r2, .L291
	movs	r1, #0
	ldr	r2, [r2, #72]
	strh	r1, [r2, r0, lsl #1]	@ movhi
	bl	INSERT_FREE_LIST
.L290:
	movs	r0, #0
	pop	{r3, pc}
.L292:
	.align	2
.L291:
	.word	.LANCHOR0
	.fnend
	.size	free_data_superblock, .-free_data_superblock
	.align	1
	.global	get_new_active_ppa
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r3, #65535
	ldrh	r2, [r0]
	mov	r4, r0
	cmp	r2, r3
	bne	.L294
	movw	r2, #2785
	ldr	r1, .L308
	ldr	r0, .L308+4
	bl	sftl_printk
.L294:
	ldr	r6, .L308+8
	ldrh	r2, [r4, #2]
	ldrh	r3, [r6, #302]
	cmp	r2, r3
	bne	.L295
	movw	r2, #2786
	ldr	r1, .L308
	ldr	r0, .L308+4
	bl	sftl_printk
.L295:
	ldrh	r3, [r4, #4]
	cbnz	r3, .L296
	movw	r2, #2787
	ldr	r1, .L308
	ldr	r0, .L308+4
	bl	sftl_printk
.L296:
	ldrb	r2, [r4, #6]	@ zero_extendqisi2
	movs	r3, #0
	ldrh	r1, [r6, #232]
	movw	r5, #65535
	strb	r3, [r4, #10]
	adds	r2, r2, #8
	ldrh	r0, [r4, r2, lsl #1]
	mov	r2, r3
.L297:
	cmp	r0, r5
	beq	.L299
	ldrh	r5, [r4, #2]
	ldrh	r6, [r6, #302]
	cmp	r5, r6
	bcs	.L303
	ldrh	r2, [r4, #4]
	orr	r5, r5, r0, lsl #10
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	movw	r0, #65535
	subs	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r4, #4]	@ movhi
.L302:
	adds	r3, r3, #1
	uxtb	r3, r3
	cmp	r1, r3
	itttt	eq
	ldrheq	r3, [r4, #2]
	addeq	r3, r3, #1
	strheq	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	add	r7, r3, #8
	ldrh	r7, [r4, r7, lsl #1]
	cmp	r7, r0
	beq	.L302
	strb	r3, [r4, #6]
	ldrh	r3, [r4, #2]
	cmp	r3, r6
	bne	.L293
	cbz	r2, .L293
	movw	r2, #2810
	ldr	r1, .L308
	ldr	r0, .L308+4
	bl	sftl_printk
.L293:
	mov	r0, r5
	pop	{r3, r4, r5, r6, r7, pc}
.L299:
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	adds	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, r1
	strb	r3, [r4, #6]
	itttt	eq
	ldrheq	r3, [r4, #2]
	strbeq	r2, [r4, #6]
	addeq	r3, r3, #1
	strheq	r3, [r4, #2]	@ movhi
	ldrb	r3, [r4, #6]	@ zero_extendqisi2
	adds	r3, r3, #8
	ldrh	r0, [r4, r3, lsl #1]
	b	.L297
.L303:
	movw	r5, #65535
	b	.L293
.L309:
	.align	2
.L308:
	.word	.LANCHOR1+111
	.word	.LC8
	.word	.LANCHOR0
	.fnend
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	1
	.global	FtlGcBufInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L319
	mov	ip, #20
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r9, #12
	movs	r6, #1
	movs	r2, #0
	mov	r7, r3
	str	r2, [r3, #3168]
.L311:
	ldrh	r1, [r3, #232]
	uxth	r0, r2
	adds	r4, r2, #1
	cmp	r0, r1
	bcc	.L314
	ldr	r7, .L319
	mov	ip, #12
	movs	r6, #0
.L315:
	ldr	r2, [r3, #3188]
	cmp	r1, r2
	bcc	.L318
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L314:
	uxth	r2, r2
	ldr	r8, [r3, #3172]
	mul	lr, r9, r2
	add	r1, r8, lr
	str	r6, [r1, #8]
	ldrh	r1, [r3, #310]
	muls	r1, r2, r1
	it	mi
	addmi	r1, r1, #3
	bic	r0, r1, #3
	ldr	r1, [r3, #3176]
	add	r1, r1, r0
	str	r1, [r8, lr]
	ldr	r5, [r7, #3180]
	ldr	r0, [r3, #3172]
	ldrh	r1, [r3, #312]
	muls	r1, r2, r1
	it	mi
	addmi	r1, r1, #3
	bic	r1, r1, #3
	add	r8, r0, lr
	add	r1, r1, r5
	str	r1, [r8, #4]
	ldr	r1, [r3, #3184]
	mla	r2, ip, r2, r1
	ldr	r1, [r0, lr]
	str	r1, [r2, #8]
	ldr	r1, [r8, #4]
	str	r1, [r2, #12]
	mov	r2, r4
	b	.L311
.L318:
	mul	r0, ip, r1
	ldr	r4, [r3, #3172]
	ldr	r5, [r3, #3176]
	adds	r2, r4, r0
	str	r6, [r2, #8]
	ldrh	r2, [r3, #310]
	muls	r2, r1, r2
	it	mi
	addmi	r2, r2, #3
	bic	r2, r2, #3
	add	r2, r2, r5
	str	r2, [r4, r0]
	ldr	r2, [r3, #3172]
	ldr	r4, [r7, #3180]
	add	r0, r0, r2
	ldrh	r2, [r3, #312]
	muls	r2, r1, r2
	it	mi
	addmi	r2, r2, #3
	bic	r2, r2, #3
	adds	r1, r1, #1
	add	r2, r2, r4
	uxth	r1, r1
	str	r2, [r0, #4]
	b	.L315
.L320:
	.align	2
.L319:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	1
	.global	FtlGcBufFree
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L328
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movs	r5, #0
	mov	r10, #20
	mov	lr, #12
	mov	r8, r5
	ldr	ip, [r3, #3188]
	ldr	r6, [r3, #3172]
.L322:
	uxth	r3, r5
	cmp	r1, r3
	bls	.L321
	mla	r4, r10, r3, r0
	movs	r2, #0
.L323:
	uxth	r3, r2
	cmp	ip, r3
	bls	.L324
	mul	r3, lr, r3
	ldr	r7, [r4, #8]
	adds	r2, r2, #1
	add	r9, r6, r3
	ldr	r3, [r6, r3]
	cmp	r3, r7
	bne	.L323
	str	r8, [r9, #8]
.L324:
	adds	r5, r5, #1
	b	.L322
.L321:
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L329:
	.align	2
.L328:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufFree, .-FtlGcBufFree
	.align	1
	.global	FtlGcBufAlloc
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcBufAlloc, %function
FtlGcBufAlloc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L337
	mov	ip, #12
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	movs	r4, #0
	mov	lr, #1
	mov	r8, #20
	ldr	r5, [r3, #3188]
	ldr	r6, [r3, #3172]
.L331:
	uxth	r2, r4
	cmp	r1, r2
	bhi	.L335
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L335:
	mov	r9, #0
.L332:
	uxth	r3, r9
	cmp	r5, r3
	bls	.L333
	mla	r3, ip, r3, r6
	add	r9, r9, #1
	ldr	r7, [r3, #8]
	cmp	r7, #0
	bne	.L332
	mla	r2, r8, r2, r0
	ldr	r7, [r3]
	str	lr, [r3, #8]
	str	r7, [r2, #8]
	ldr	r3, [r3, #4]
	str	r3, [r2, #12]
.L333:
	adds	r4, r4, #1
	b	.L331
.L338:
	.align	2
.L337:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	1
	.global	IsBlkInGcList
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L344
	ldr	r3, [r2, #3192]
	ldrh	r2, [r2, #3196]
	add	r2, r3, r2, lsl #1
.L340:
	cmp	r3, r2
	bne	.L342
	movs	r0, #0
	bx	lr
.L342:
	ldrh	r1, [r3], #2
	cmp	r1, r0
	bne	.L340
	movs	r0, #1
	bx	lr
.L345:
	.align	2
.L344:
	.word	.LANCHOR0
	.fnend
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	1
	.global	FtlGcUpdatePage
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, r0
	ubfx	r0, r0, #10, #16
	mov	r6, r1
	mov	r7, r2
	movs	r4, #0
	bl	P2V_block_in_plane
	ldr	r3, .L350
	ldrh	r1, [r3, #3196]
	ldr	r2, [r3, #3192]
.L347:
	uxth	ip, r4
	cmp	ip, r1
	bcc	.L349
	bne	.L348
	strh	r0, [r2, ip, lsl #1]	@ movhi
	ldrh	r0, [r3, #3196]
	adds	r0, r0, #1
	strh	r0, [r3, #3196]	@ movhi
	b	.L348
.L349:
	adds	r4, r4, #1
	add	ip, r2, r4, lsl #1
	ldrh	ip, [ip, #-2]
	cmp	ip, r0
	bne	.L347
.L348:
	ldrh	r2, [r3, #3204]
	movs	r0, #12
	muls	r0, r2, r0
	ldr	r2, [r3, #3200]
	adds	r1, r2, r0
	str	r6, [r1, #4]
	str	r7, [r1, #8]
	str	r5, [r2, r0]
	ldrh	r2, [r3, #3204]
	adds	r2, r2, #1
	strh	r2, [r3, #3204]	@ movhi
	pop	{r3, r4, r5, r6, r7, pc}
.L351:
	.align	2
.L350:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	1
	.global	FtlGcRefreshBlock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	mov	r1, r0
	mov	r4, r0
	ldr	r0, .L355
	bl	sftl_printk
	ldr	r3, .L355+4
	ldrh	r0, [r3, #222]
	cmp	r4, r0
	beq	.L353
	ldrh	r1, [r3, #220]
	cmp	r4, r1
	beq	.L353
	movw	r2, #65535
	cmp	r0, r2
	bne	.L354
	strh	r4, [r3, #222]	@ movhi
.L353:
	movs	r0, #0
	pop	{r4, pc}
.L354:
	cmp	r1, r2
	it	eq
	strheq	r4, [r3, #220]	@ movhi
	b	.L353
.L356:
	.align	2
.L355:
	.word	.LC79
	.word	.LANCHOR0
	.fnend
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.align	1
	.global	FtlGcMarkBadPhyBlk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcMarkBadPhyBlk, %function
FtlGcMarkBadPhyBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r4, .L361
	bl	P2V_block_in_plane
	mov	r2, r5
	mov	r6, r0
	ldrh	r1, [r4, #3206]
	ldr	r0, .L361+4
	bl	sftl_printk
	mov	r0, r6
	bl	FtlGcRefreshBlock
	ldrh	r3, [r4, #3206]
	movs	r2, #0
	addw	r0, r4, #3208
.L358:
	uxth	r1, r2
	cmp	r3, r1
	bhi	.L360
	cmp	r3, #15
	itttt	ls
	addls	r2, r3, #1
	strhls	r2, [r4, #3206]	@ movhi
	addls	r4, r4, r3, lsl #1
	strhls	r5, [r4, #3208]	@ movhi
	b	.L359
.L360:
	adds	r2, r2, #1
	add	r1, r0, r2, lsl #1
	ldrh	r1, [r1, #-2]
	cmp	r1, r5
	bne	.L358
.L359:
	movs	r0, #0
	pop	{r4, r5, r6, pc}
.L362:
	.align	2
.L361:
	.word	.LANCHOR0
	.word	.LC80
	.fnend
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	1
	.global	FtlGcReFreshBadBlk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	ldr	r4, .L369
	ldrh	r3, [r4, #3206]
	cbz	r3, .L364
	ldrh	r1, [r4, #222]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L364
	ldrh	r2, [r4, #3242]
	cmp	r2, r3
	itt	cs
	movcs	r3, #0
	strhcs	r3, [r4, #3242]	@ movhi
	ldrh	r3, [r4, #3242]
	add	r3, r4, r3, lsl #1
	ldrh	r0, [r3, #3208]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	r3, [r4, #3242]
	adds	r3, r3, #1
	strh	r3, [r4, #3242]	@ movhi
.L364:
	movs	r0, #0
	pop	{r4, pc}
.L370:
	.align	2
.L369:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	1
	.global	ftl_malloc
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_malloc, %function
ftl_malloc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r1, .L372
	b	__kmalloc
.L373:
	.align	2
.L372:
	.word	37748929
	.fnend
	.size	ftl_malloc, .-ftl_malloc
	.align	1
	.global	ftl_free
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_free, %function
ftl_free:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	kfree
	.fnend
	.size	ftl_free, .-ftl_free
	.align	1
	.global	rknand_print_hex
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rknand_print_hex, %function
rknand_print_hex:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r6, #0
	ldr	fp, .L384+16
	mov	r10, r0
	mov	r7, r1
	mov	r8, r2
	mov	r9, r3
	mov	r5, r6
	mov	r4, r6
.L376:
	cmp	r4, r9
	bcc	.L382
	ldr	r0, .L384
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	sftl_printk
.L382:
	cbnz	r5, .L377
	mov	r3, r6
	mov	r2, r7
	mov	r1, r10
	ldr	r0, .L384+4
	bl	sftl_printk
.L377:
	cmp	r8, #4
	bne	.L378
	ldr	r1, [r7, r4, lsl #2]
	ldr	r0, .L384+8
.L383:
	adds	r5, r5, #1
	bl	sftl_printk
	cmp	r5, #15
	bls	.L381
	movs	r5, #0
	ldr	r0, .L384
	bl	sftl_printk
.L381:
	adds	r4, r4, #1
	add	r6, r6, r8
	b	.L376
.L378:
	cmp	r8, #2
	ittee	eq
	ldrheq	r1, [r7, r4, lsl #1]
	moveq	r0, fp
	ldrbne	r1, [r7, r4]	@ zero_extendqisi2
	ldrne	r0, .L384+12
	b	.L383
.L385:
	.align	2
.L384:
	.word	.LC85
	.word	.LC81
	.word	.LC82
	.word	.LC84
	.word	.LC83
	.fnend
	.size	rknand_print_hex, .-rknand_print_hex
	.align	1
	.global	FlashEraseBlocks
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r2
	ldr	r5, .L402
	mov	r4, r0
	mov	fp, r0
	movs	r7, #0
	mov	r10, #-1
	.pad #20
	sub	sp, sp, #20
	ldrh	r9, [r5, #12]
	lsl	r3, r9, #3
	str	r3, [sp, #4]
.L387:
	cmp	r7, r8
	beq	.L391
	add	r2, sp, #8
	add	r1, sp, #12
	mov	r0, fp
	bl	l2p_addr_tran
	ldr	r6, [sp, #8]
	cbnz	r6, .L388
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #4]
	cmp	r3, r2
	bls	.L388
	ldr	r5, .L402+4
	ldr	r7, .L402+8
	ldr	r9, .L402+16
	b	.L401
.L390:
	mov	r3, #-1
	ldr	r2, [sp, #12]
	str	r3, [r4, #-20]
	mov	r1, r5
	mov	r0, r7
	adds	r6, r6, #1
	bl	sftl_printk
	movs	r3, #16
	movs	r2, #4
	ldr	r1, [r4, #-12]
	mov	r0, r9
	bl	rknand_print_hex
	movs	r3, #4
	ldr	r1, [r4, #-8]
	mov	r2, r3
	ldr	r0, .L402+12
	bl	rknand_print_hex
.L401:
	cmp	r6, r8
	add	r4, r4, #20
	bne	.L390
	bl	dump_stack
.L391:
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L388:
	ldr	r2, [r5, #3248]
	uxtb	r0, r6
	ldr	r1, [sp, #12]
	blx	r2
	cmp	r0, #0
	mov	r2, r0
	it	ne
	movne	r2, r10
	str	r2, [fp]
	ldrh	r2, [r5, #14]
	cmp	r2, #4
	bne	.L395
	ldr	r1, [sp, #12]
	ldr	r2, [r5, #3248]
	ldrb	r0, [sp, #8]	@ zero_extendqisi2
	add	r1, r1, r9
	blx	r2
	cbz	r0, .L395
	str	r10, [fp]
.L395:
	adds	r7, r7, #1
	add	fp, fp, #20
	b	.L387
.L403:
	.align	2
.L402:
	.word	.LANCHOR0
	.word	.LANCHOR1+130
	.word	.LC86
	.word	.LC88
	.word	.LC87
	.fnend
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	1
	.global	FtlFreeSysBlkQueueIn
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	subs	r3, r0, #1
	movw	r2, #65533
	uxth	r3, r3
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	cmp	r3, r2
	bhi	.L404
	ldr	r4, .L413
	ldrh	r3, [r4, #418]
	cmp	r3, #1024
	beq	.L404
	cbz	r1, .L406
	bl	P2V_block_in_plane
	mov	r6, r0
	ldr	r0, [r4, #3268]
	lsls	r3, r5, #10
	movs	r2, #1
	mov	r1, r2
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r2, [r4, #2536]
	ldrh	r3, [r2, r6, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r2, r6, lsl #1]	@ movhi
	ldr	r3, [r4, #2608]
	adds	r3, r3, #1
	str	r3, [r4, #2608]
.L406:
	ldrh	r3, [r4, #418]
	adds	r3, r3, #1
	strh	r3, [r4, #418]	@ movhi
	ldrh	r3, [r4, #416]
	add	r2, r4, r3, lsl #1
	adds	r3, r3, #1
	ubfx	r3, r3, #0, #10
	strh	r5, [r2, #420]	@ movhi
	strh	r3, [r4, #416]	@ movhi
.L404:
	pop	{r4, r5, r6, pc}
.L414:
	.align	2
.L413:
	.word	.LANCHOR0
	.fnend
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	1
	.global	FtlFreeSysBlkQueueOut
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L420
	ldrh	r2, [r4, #418]
	cbz	r2, .L419
	ldrh	r3, [r4, #414]
	subs	r2, r2, #1
	ldr	r0, [r4, #3268]
	strh	r2, [r4, #418]	@ movhi
	movs	r2, #1
	add	r1, r4, r3, lsl #1
	adds	r3, r3, #1
	ubfx	r3, r3, #0, #10
	ldrh	r5, [r1, #420]
	mov	r1, r2
	strh	r3, [r4, #414]	@ movhi
	lsls	r3, r5, #10
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
	ldr	r3, [r4, #2608]
	adds	r3, r3, #1
	str	r3, [r4, #2608]
.L416:
	subs	r3, r5, #1
	movw	r2, #65533
	uxth	r3, r3
	cmp	r3, r2
	bls	.L417
	ldrh	r2, [r4, #418]
	mov	r1, r5
	ldr	r0, .L420+4
	bl	sftl_printk
.L418:
	b	.L418
.L419:
	movw	r5, #65535
	b	.L416
.L417:
	mov	r0, r5
	pop	{r3, r4, r5, pc}
.L421:
	.align	2
.L420:
	.word	.LANCHOR0
	.word	.LC89
	.fnend
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	1
	.global	ftl_map_blk_alloc_new_blk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_map_blk_alloc_new_blk, %function
ftl_map_blk_alloc_new_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r4, r0
	ldrh	r1, [r0, #10]
	movs	r3, #0
	ldr	r2, [r0, #12]
.L423:
	uxth	r5, r3
	cmp	r5, r1
	bcs	.L426
	mov	r7, r2
	adds	r3, r3, #1
	ldrh	r6, [r7]
	adds	r2, r2, #2
	cmp	r6, #0
	bne	.L423
	bl	FtlFreeSysBlkQueueOut
	subs	r3, r0, #1
	movw	r2, #65533
	uxth	r3, r3
	mov	r1, r0
	strh	r0, [r7]	@ movhi
	cmp	r3, r2
	bls	.L424
	ldr	r3, .L430
	ldr	r0, .L430+4
	ldrh	r2, [r3, #418]
	bl	sftl_printk
.L425:
	b	.L425
.L424:
	ldr	r3, [r4, #28]
	strh	r6, [r4, #2]	@ movhi
	strh	r5, [r4]	@ movhi
	adds	r3, r3, #1
	str	r3, [r4, #28]
	ldrh	r3, [r4, #8]
	adds	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
.L426:
	ldrh	r3, [r4, #10]
	cmp	r3, r5
	bhi	.L428
	movw	r2, #578
	ldr	r1, .L430+8
	ldr	r0, .L430+12
	bl	sftl_printk
.L428:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, pc}
.L431:
	.align	2
.L430:
	.word	.LANCHOR0
	.word	.LC90
	.word	.LANCHOR1+147
	.word	.LC8
	.fnend
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	1
	.global	FlashGetBadBlockList
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r2, #256
	ldr	r5, .L439
	mov	r4, r0
	mov	r6, r1
	movs	r1, #255
	bl	memset
	ldr	r3, [r5, #3244]
	mov	r1, r6
	mov	r0, r4
	blx	r3
	uxth	r0, r0
	cmp	r0, #50
	bls	.L433
	mov	r2, #256
	movs	r1, #255
	mov	r0, r4
	bl	memset
	movs	r0, #0
.L433:
	ldrh	r3, [r5, #14]
	cmp	r3, #4
	bne	.L438
	add	r1, r4, r0, lsl #1
	mov	r3, r4
.L435:
	cmp	r3, r1
	bne	.L436
.L438:
	pop	{r4, r5, r6, pc}
.L436:
	ldrh	r2, [r3]
	lsrs	r2, r2, #1
	strh	r2, [r3], #2	@ movhi
	b	.L435
.L440:
	.align	2
.L439:
	.word	.LANCHOR0
	.fnend
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	1
	.global	ftl_memset
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_memset, %function
ftl_memset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	mov	r4, r0
	cbz	r2, .L442
	bl	memset
.L442:
	mov	r0, r4
	pop	{r4, pc}
	.fnend
	.size	ftl_memset, .-ftl_memset
	.align	1
	.global	FtlMemInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlMemInit, %function
FtlMemInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r3, #65535
	ldr	r4, .L547
	movs	r5, #0
	movs	r7, #12
	movs	r6, #20
	ldrh	r0, [r4, #306]
	str	r3, [r4, #3276]
	strh	r3, [r4, #222]	@ movhi
	strh	r3, [r4, #220]	@ movhi
	movs	r3, #32
	strh	r3, [r4, #3152]	@ movhi
	lsls	r0, r0, #1
	movs	r3, #128
	str	r5, [r4, #2588]
	strh	r3, [r4, #3154]	@ movhi
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
	strh	r5, [r4, #3160]	@ movhi
	strh	r5, [r4, #3206]	@ movhi
	strh	r5, [r4, #3242]	@ movhi
	bl	ftl_malloc
	str	r0, [r4, #3192]
	ldrh	r0, [r4, #306]
	muls	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #232]
	str	r0, [r4, #3200]
	muls	r6, r3, r6
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
	ldrh	r6, [r4, #310]
	str	r0, [r4, #3184]
	lsls	r3, r3, #1
	mov	r0, r6
	adds	r3, r3, #1
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
	muls	r0, r6, r0
	bl	ftl_malloc
	str	r0, [r4, #3176]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3312]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3316]
	ldr	r0, [r4, #3188]
	muls	r0, r7, r0
	bl	ftl_malloc
	str	r0, [r4, #3172]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3320]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3324]
	ldrh	r0, [r4, #258]
	lsls	r0, r0, #2
	bl	ftl_malloc
	ldrh	r3, [r4, #312]
	ldrh	r6, [r4, #232]
	str	r0, [r4, #3328]
	muls	r6, r3, r6
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3332]
	lsls	r0, r6, #2
	bl	ftl_malloc
	ldrh	r3, [r4, #312]
	str	r0, [r4, #3336]
	ldr	r0, [r4, #3188]
	muls	r0, r3, r0
	bl	ftl_malloc
	str	r0, [r4, #3180]
	ldrh	r0, [r4, #242]
	lsls	r0, r0, #1
	uxth	r0, r0
	strh	r0, [r4, #3340]	@ movhi
	bl	ftl_malloc
	ldrh	r3, [r4, #3340]
	str	r0, [r4, #3344]
	ldr	r0, .L547+4
	addw	r3, r3, #547
	lsrs	r3, r3, #9
	and	r0, r0, r3, lsl #9
	strh	r3, [r4, #3340]	@ movhi
	bl	ftl_malloc
	ldrh	r6, [r4, #242]
	str	r0, [r4, #3348]
	adds	r0, r0, #32
	str	r0, [r4, #2536]
	lsls	r6, r6, #1
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3352]
	mov	r0, r6
	bl	ftl_malloc
	ldr	r6, [r4, #328]
	str	r0, [r4, #72]
	lsls	r6, r6, #1
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3356]
	mov	r0, r6
	bl	ftl_malloc
	str	r0, [r4, #3360]
	ldrh	r0, [r4, #320]
	lsls	r0, r0, #1
	bl	ftl_malloc
	str	r0, [r4, #348]
	ldrh	r0, [r4, #320]
	lsls	r0, r0, #1
	bl	ftl_malloc
	str	r0, [r4, #3364]
	ldrh	r0, [r4, #320]
	lsls	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #3368]
	ldrh	r0, [r4, #322]
	lsls	r0, r0, #2
	bl	ftl_malloc
	ldrh	r2, [r4, #322]
	mov	r1, r5
	str	r0, [r4, #3372]
	add	r5, r4, #380
	lsls	r2, r2, #2
	bl	ftl_memset
	ldrh	r0, [r4, #336]
	lsls	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #3376]
	ldr	r0, [r4, #328]
	lsls	r0, r0, #2
	bl	ftl_malloc
	str	r0, [r4, #3380]
	ldrh	r0, [r4, #338]
	muls	r0, r7, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #338]
	str	r0, [r4, #2540]
	ldrh	r0, [r4, #310]
	muls	r0, r3, r0
	bl	ftl_malloc
	ldrh	r3, [r4, #242]
	str	r0, [r4, #3384]
	movs	r0, #6
	muls	r0, r3, r0
	bl	ftl_malloc
	str	r0, [r4, #2516]
	ldrh	r0, [r4, #298]
	ldrh	r3, [r4, #254]
	adds	r0, r0, #31
	asrs	r0, r0, #5
	strh	r0, [r4, #3388]	@ movhi
	muls	r0, r3, r0
	lsls	r0, r0, #2
	bl	ftl_malloc
	ldrh	r2, [r4, #3388]
	movs	r3, #1
	ldrh	r6, [r4, #254]
	str	r0, [r4, #380]
	lsls	r2, r2, #2
	mov	r1, r2
.L447:
	cmp	r3, r6
	bcc	.L448
	ldr	r2, .L547+8
	movs	r1, #0
	add	r3, r2, r3, lsl #2
	adds	r2, r2, #56
	adds	r3, r3, #24
.L449:
	cmp	r2, r3
	bne	.L450
	ldr	r3, [r4, #3356]
	cbnz	r3, .L451
.L453:
	ldr	r1, .L547+12
	ldr	r0, .L547+16
	bl	sftl_printk
	mov	r0, #-1
	pop	{r4, r5, r6, r7, r8, pc}
.L448:
	ldr	r0, [r4, #380]
	adds	r3, r3, #1
	add	r0, r0, r1
	add	r1, r1, r2
	str	r0, [r5, #4]!
	b	.L447
.L450:
	str	r1, [r3, #4]!
	b	.L449
.L548:
	.align	2
.L547:
	.word	.LANCHOR0
	.word	33553920
	.word	.LANCHOR0+352
	.word	.LANCHOR1+173
	.word	.LC91
.L451:
	ldr	r3, [r4, #3360]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3376]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3380]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #2540]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3384]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #2516]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #380]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #72]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3192]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3200]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3288]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3296]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3268]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3184]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3292]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3300]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3304]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3308]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3176]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3312]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3316]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3172]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3332]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3336]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3180]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #2536]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #3344]
	cmp	r3, #0
	beq	.L453
	ldr	r3, [r4, #348]
	cmp	r3, #0
	beq	.L453
	ldr	r3, .L549
	ldr	r2, [r3, #3364]
	cmp	r2, #0
	beq	.L453
	ldr	r2, [r3, #3368]
	cmp	r2, #0
	beq	.L453
	ldr	r3, [r3, #3372]
	cmp	r3, #0
	beq	.L453
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, pc}
.L550:
	.align	2
.L549:
	.word	.LANCHOR0
	.fnend
	.size	FtlMemInit, .-FtlMemInit
	.align	1
	.global	FtlBbt2Bitmap
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r5, r0
	ldr	r6, .L557
	mov	r7, r1
	subs	r4, r5, #2
	movw	r8, #65535
	ldr	r9, .L557+4
	addw	r5, r5, #1022
	ldrh	r2, [r6, #3388]
	movs	r1, #0
	ldr	r10, .L557+8
	mov	r0, r7
	lsls	r2, r2, #2
	bl	ftl_memset
.L554:
	ldrh	r3, [r4, #2]
	cmp	r3, r8
	beq	.L551
	ldrh	r2, [r6, #298]
	cmp	r2, r3
	bhi	.L553
	movs	r2, #74
	mov	r1, r9
	mov	r0, r10
	bl	sftl_printk
.L553:
	ldrh	r3, [r4, #2]!
	movs	r2, #1
	cmp	r5, r4
	lsr	r1, r3, #5
	and	r3, r3, #31
	lsl	r3, r2, r3
	ldr	r2, [r7, r1, lsl #2]
	orr	r2, r2, r3
	str	r2, [r7, r1, lsl #2]
	bne	.L554
.L551:
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L558:
	.align	2
.L557:
	.word	.LANCHOR0
	.word	.LANCHOR1+184
	.word	.LC8
	.fnend
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	1
	.global	ftl_free_no_use_map_blk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
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
	ldr	r5, [r0, #20]
	ldr	r7, [r0, #12]
	ldr	r6, [r0, #24]
	lsls	r2, r2, #1
	mov	r0, r5
	bl	ftl_memset
	movs	r3, #0
.L560:
	ldrh	r1, [r4, #6]
	uxth	r2, r3
	cmp	r1, r2
	bhi	.L564
	ldr	r3, .L579
	movs	r6, #0
	mov	r9, r6
	mov	fp, r6
	ldrh	r2, [r3, #304]
	ldrh	r3, [r4]
	strh	r2, [r5, r3, lsl #1]	@ movhi
	ldrh	r10, [r5]
.L565:
	ldrh	r3, [r4, #10]
	uxth	r1, r6
	cmp	r3, r1
	bhi	.L569
	mov	r0, r9
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L564:
	uxth	r2, r3
	ldr	r1, [r6, r2, lsl #2]
	movs	r2, #0
	ubfx	r1, r1, #10, #16
.L561:
	ldrh	ip, [r4, #10]
	uxth	r0, r2
	cmp	ip, r0
	bhi	.L563
	adds	r3, r3, #1
	b	.L560
.L563:
	uxth	r0, r2
	ldrh	ip, [r7, r0, lsl #1]
	cmp	ip, r1
	bne	.L562
	cbz	r1, .L562
	ldrh	ip, [r5, r0, lsl #1]
	add	ip, ip, #1
	strh	ip, [r5, r0, lsl #1]	@ movhi
.L562:
	adds	r2, r2, #1
	b	.L561
.L569:
	uxth	r2, r6
	ldrh	r3, [r5, r2, lsl #1]
	lsl	r8, r2, #1
	cmp	r10, r3
	bls	.L566
	ldrh	r0, [r7, r2, lsl #1]
	add	r8, r8, r7
	cbnz	r0, .L567
.L568:
	adds	r6, r6, #1
	b	.L565
.L566:
	cmp	r3, #0
	bne	.L568
	ldrh	r0, [r7, r2, lsl #1]
	add	r8, r8, r7
	cmp	r0, #0
	beq	.L568
.L570:
	movs	r1, #1
	bl	FtlFreeSysBlkQueueIn
	strh	fp, [r8]	@ movhi
	ldrh	r3, [r4, #8]
	subs	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	b	.L568
.L567:
	mov	r9, r1
	mov	r10, r3
	cmp	r3, #0
	beq	.L570
	b	.L568
.L580:
	.align	2
.L579:
	.word	.LANCHOR0
	.fnend
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	1
	.global	FtlL2PDataInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movs	r1, #0
	ldr	r4, .L584
	movs	r6, #12
	ldr	r2, [r4, #328]
	ldr	r0, [r4, #3360]
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r3, [r4, #310]
	movs	r1, #255
	ldrh	r2, [r4, #338]
	ldr	r0, [r4, #3384]
	muls	r2, r3, r2
	bl	ftl_memset
	movs	r2, #0
	mov	r3, r4
	movw	r1, #65535
	mov	r7, r2
.L582:
	ldrh	r4, [r3, #338]
	uxth	r0, r2
	adds	r5, r2, #1
	cmp	r4, r0
	bhi	.L583
	ldr	r2, [r3, #328]
	strh	r1, [r3, #3394]	@ movhi
	strh	r1, [r3, #3392]	@ movhi
	strh	r2, [r3, #3402]	@ movhi
	movw	r2, #61634
	strh	r2, [r3, #3396]	@ movhi
	ldrh	r2, [r3, #3436]
	strh	r2, [r3, #3400]	@ movhi
	ldrh	r2, [r3, #336]
	strh	r2, [r3, #3398]	@ movhi
	ldr	r2, [r3, #3356]
	str	r2, [r3, #3404]
	ldr	r2, [r3, #3380]
	str	r2, [r3, #3408]
	ldr	r2, [r3, #3360]
	str	r2, [r3, #3412]
	ldr	r2, [r3, #3376]
	str	r2, [r3, #3416]
	pop	{r3, r4, r5, r6, r7, pc}
.L583:
	uxth	r2, r2
	ldr	r0, [r3, #2540]
	mul	r4, r6, r2
	add	ip, r0, r4
	str	r7, [ip, #4]
	strh	r1, [r0, r4]	@ movhi
	ldr	r0, [r3, #2540]
	add	r0, r0, r4
	ldrh	r4, [r3, #310]
	muls	r2, r4, r2
	ldr	r4, [r3, #3384]
	bic	r2, r2, #3
	add	r2, r2, r4
	str	r2, [r0, #8]
	mov	r2, r5
	b	.L582
.L585:
	.align	2
.L584:
	.word	.LANCHOR0
	.fnend
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	1
	.global	FtlVariablesInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r5, #0
	ldr	r4, .L587
	movw	r3, #65535
	mov	r1, r5
	ldrh	r2, [r4, #320]
	strh	r3, [r4, #3438]	@ movhi
	mov	r3, #-1
	ldr	r0, [r4, #348]
	str	r3, [r4, #3448]
	lsls	r2, r2, #1
	str	r5, [r4, #3440]
	str	r5, [r4, #3444]
	strh	r5, [r4, #344]	@ movhi
	bl	ftl_memset
	ldrh	r2, [r4, #242]
	mov	r1, r5
	ldr	r0, [r4, #2536]
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r4, #242]
	mov	r1, r5
	ldr	r0, [r4, #3344]
	lsls	r2, r2, #1
	bl	ftl_memset
	addw	r0, r4, #2468
	movs	r1, #48
	bl	__memzero
	add	r0, r4, #2640
	mov	r1, #512
	bl	__memzero
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	r0, r5
	pop	{r3, r4, r5, pc}
.L588:
	.align	2
.L587:
	.word	.LANCHOR0
	.fnend
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	1
	.global	SupperBlkListInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r5, #0
	ldr	r4, .L600
	movs	r2, #6
	mov	r9, r5
	mov	r6, r5
	movw	fp, #65535
	.pad #20
	sub	sp, sp, #20
	ldrh	r3, [r4, #242]
	add	r10, r4, #260
	movs	r1, #0
	ldr	r0, [r4, #2516]
	muls	r2, r3, r2
	bl	ftl_memset
	str	r5, [r4, #2532]
	str	r5, [r4, #2520]
	str	r5, [r4, #2524]
	strh	r5, [r4, #2528]	@ movhi
	strh	r5, [r4, #224]	@ movhi
.L590:
	ldrh	r3, [r4, #240]
	uxth	r8, r5
	cmp	r8, r3
	bcs	.L597
	ldrh	r3, [r4, #302]
	ldrh	r2, [r4, #232]
	str	r3, [sp, #4]
	movs	r3, #0
	mov	r7, r3
	b	.L598
.L592:
	ldrb	r0, [r10, r3]	@ zero_extendqisi2
	mov	r1, r8
	str	r2, [sp, #12]
	str	r3, [sp, #8]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #8]
	ldr	r2, [sp, #12]
	cbnz	r0, .L591
	ldr	r1, [sp, #4]
	add	r7, r7, r1
	uxth	r7, r7
.L591:
	adds	r3, r3, #1
.L598:
	uxth	r1, r3
	cmp	r2, r1
	bhi	.L592
	uxth	r3, r5
	cbz	r7, .L593
	mov	r1, r7
	str	r3, [sp, #4]
	mov	r0, #32768
	bl	__aeabi_idiv
	ldr	r3, [sp, #4]
	uxth	r7, r0
.L594:
	ldr	r1, [r4, #2516]
	movs	r2, #6
	mla	r2, r2, r3, r1
	strh	r7, [r2, #4]	@ movhi
	ldrh	r2, [r4, #24]
	cmp	r2, r8
	beq	.L595
	ldrh	r2, [r4, #76]
	cmp	r2, r8
	beq	.L595
	ldrh	r2, [r4, #124]
	cmp	r2, r8
	beq	.L595
	ldr	r2, [r4, #72]
	ldrh	r3, [r2, r3, lsl #1]
	cbnz	r3, .L596
	add	r9, r9, #1
	mov	r0, r8
	uxth	r9, r9
	bl	INSERT_FREE_LIST
.L595:
	adds	r5, r5, #1
	b	.L590
.L593:
	ldr	r2, [r4, #72]
	strh	fp, [r2, r3, lsl #1]	@ movhi
	b	.L594
.L596:
	adds	r6, r6, #1
	mov	r0, r8
	uxth	r6, r6
	bl	INSERT_DATA_LIST
	b	.L595
.L597:
	strh	r6, [r4, #2528]	@ movhi
	add	r6, r6, r9
	cmp	r6, r3
	strh	r9, [r4, #224]	@ movhi
	ble	.L599
	movw	r2, #2212
	ldr	r1, .L600+4
	ldr	r0, .L600+8
	bl	sftl_printk
.L599:
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L601:
	.align	2
.L600:
	.word	.LANCHOR0
	.word	.LANCHOR1+198
	.word	.LC8
	.fnend
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	1
	.global	FtlGcPageVarInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	movs	r3, #0
	ldr	r4, .L603
	movs	r1, #255
	ldrh	r2, [r4, #306]
	ldr	r0, [r4, #3192]
	strh	r3, [r4, #3196]	@ movhi
	strh	r3, [r4, #3204]	@ movhi
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r3, [r4, #306]
	movs	r2, #12
	ldr	r0, [r4, #3200]
	movs	r1, #255
	muls	r2, r3, r2
	bl	ftl_memset
	pop	{r4, lr}
	b	FtlGcBufInit
.L604:
	.align	2
.L603:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	1
	.global	ftl_memcpy
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
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
	.global	FlashReadPages
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FlashReadPages, %function
FlashReadPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r1
	ldr	r5, .L646
	mov	r4, r0
	mov	r8, #0
	.pad #20
	sub	sp, sp, #20
	ldr	r10, .L646+16
	ldr	fp, .L646+20
	ldrh	r3, [r5, #12]
	str	r3, [sp, #4]
.L607:
	cmp	r8, r9
	bne	.L621
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L621:
	ldr	r3, [r4, #8]
	cbz	r3, .L608
	ldr	r3, [r4, #12]
	cbnz	r3, .L609
.L608:
	movs	r2, #96
	ldr	r1, .L646+4
	mov	r0, r10
	bl	sftl_printk
.L609:
	add	r2, sp, #8
	add	r1, sp, #12
	mov	r0, r4
	bl	l2p_addr_tran
	ldr	r0, [sp, #8]
	cmp	r0, #3
	bls	.L610
	mov	r3, #-1
	str	r3, [r4]
.L611:
	add	r8, r8, #1
	adds	r4, r4, #20
	b	.L607
.L610:
	ldr	r6, [r4, #8]
	uxtb	r0, r0
	ldr	r3, [r4, #12]
	ldr	r1, [sp, #12]
	lsls	r7, r6, #26
	ldr	r7, [r5, #3256]
	it	ne
	ldrne	r6, [r5, #3320]
	mov	r2, r6
	blx	r7
	str	r0, [r4]
	ldrh	r3, [r5, #14]
	cmp	r3, #4
	bne	.L614
	ldr	r0, [sp, #4]
	add	r2, r6, #2048
	ldr	r3, [r4, #12]
	ldr	r1, [sp, #12]
	ldr	r7, [r5, #3256]
	adds	r3, r3, #8
	add	r1, r1, r0
	ldrb	r0, [sp, #8]	@ zero_extendqisi2
	blx	r7
	adds	r1, r0, #1
	beq	.L615
	ldr	r3, [r4, #12]
	ldr	r2, [r3, #12]
	adds	r2, r2, #1
	bne	.L616
	ldr	r2, [r3, #8]
	adds	r2, r2, #1
	bne	.L616
	ldr	r3, [r3]
	adds	r3, r3, #1
	beq	.L616
.L615:
	mov	r3, #-1
	str	r3, [r4]
.L616:
	ldr	r3, [r4]
	adds	r3, r3, #1
	beq	.L617
	cmp	r0, #256
	it	eq
	streq	r0, [r4]
.L617:
	ldr	r3, [r4]
	adds	r2, r3, #1
	beq	.L618
	cmp	r3, #256
	bne	.L614
.L618:
	ldr	r1, [r4, #4]
	mov	r0, fp
	ldr	r2, [sp, #12]
	bl	sftl_printk
	ldr	r1, [r4, #8]
	cbz	r1, .L620
	movs	r3, #4
	ldr	r0, .L646+8
	mov	r2, r3
	bl	rknand_print_hex
.L620:
	ldr	r1, [r4, #12]
	cbz	r1, .L614
	movs	r3, #4
	ldr	r0, .L646+12
	mov	r2, r3
	bl	rknand_print_hex
.L614:
	ldr	r3, [r5, #3320]
	cmp	r6, r3
	bne	.L611
	ldr	r0, [r4, #8]
	cmp	r6, r0
	beq	.L611
	ldrh	r2, [r5, #258]
	mov	r1, r6
	lsls	r2, r2, #9
	bl	ftl_memcpy
	b	.L611
.L647:
	.align	2
.L646:
	.word	.LANCHOR0
	.word	.LANCHOR1+216
	.word	.LC93
	.word	.LC94
	.word	.LC8
	.word	.LC92
	.fnend
	.size	FlashReadPages, .-FlashReadPages
	.align	1
	.global	FtlLoadFactoryBbt
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movs	r6, #0
	ldr	r4, .L658
	movw	r10, #61664
	ldr	r3, [r4, #3300]
	add	r7, r4, #362
	ldr	r8, [r4, #3332]
	addw	r9, r4, #3452
	str	r3, [r4, #3460]
	str	r8, [r4, #3464]
.L649:
	ldrh	r3, [r4, #254]
	cmp	r6, r3
	bcc	.L654
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L654:
	ldrh	r5, [r4, #298]
	movw	r3, #65535
	strh	r3, [r7, #2]!	@ movhi
.L651:
	ldrh	r3, [r4, #298]
	subs	r5, r5, #1
	uxth	r5, r5
	sub	r2, r3, #16
	cmp	r5, r2
	ble	.L652
	mla	r3, r6, r3, r5
	movs	r2, #1
	mov	r1, r2
	mov	r0, r9
	lsls	r3, r3, #10
	str	r3, [r4, #3456]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L651
	ldrh	r3, [r8]
	cmp	r3, r10
	bne	.L651
	strh	r5, [r7]	@ movhi
.L652:
	adds	r6, r6, #1
	b	.L649
.L659:
	.align	2
.L658:
	.word	.LANCHOR0
	.fnend
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	1
	.global	FtlGetLastWrittenPage
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L671
	cmp	r1, #1
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	lsl	r6, r0, #10
	.pad #88
	sub	sp, sp, #88
	mov	r2, r1
	mov	r7, r1
	it	eq
	ldrheq	r5, [r3, #304]
	mov	r1, #1
	it	ne
	ldrhne	r5, [r3, #302]
	add	r0, sp, #4
	ldr	r3, [r3, #3324]
	subs	r5, r5, #1
	sxth	r5, r5
	str	r3, [sp, #12]
	add	r3, sp, #24
	str	r3, [sp, #16]
	orr	r3, r5, r6
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #24]
	adds	r3, r3, #1
	bne	.L663
	mov	r8, #0
.L664:
	cmp	r8, r5
	ble	.L667
.L663:
	mov	r0, r5
	add	sp, sp, #88
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L667:
	add	r3, r8, r5
	mov	r2, r7
	add	r3, r3, r3, lsr #31
	movs	r1, #1
	add	r0, sp, #4
	asrs	r4, r3, #1
	sxth	r3, r4
	orrs	r3, r3, r6
	str	r3, [sp, #8]
	bl	FlashReadPages
	ldr	r3, [sp, #24]
	adds	r3, r3, #1
	bne	.L665
	ldr	r3, [sp, #28]
	adds	r3, r3, #1
	bne	.L665
	ldr	r3, [sp, #4]
	adds	r3, r3, #1
	beq	.L665
	subs	r4, r4, #1
	sxth	r5, r4
	b	.L664
.L665:
	adds	r4, r4, #1
	sxth	r8, r4
	b	.L664
.L672:
	.align	2
.L671:
	.word	.LANCHOR0
	.fnend
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	1
	.global	FtlScanSysBlk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movs	r5, #0
	ldr	r4, .L755
	mov	r1, r5
	.pad #20
	sub	sp, sp, #20
	ldr	r2, [r4, #328]
	ldr	r0, [r4, #3380]
	strh	r5, [r4, #3436]	@ movhi
	strh	r5, [r4, #344]	@ movhi
	lsls	r2, r2, #2
	bl	ftl_memset
	ldr	r2, [r4, #328]
	mov	r1, r5
	ldr	r0, [r4, #3356]
	lsls	r2, r2, #1
	bl	ftl_memset
	ldrh	r2, [r4, #320]
	mov	r1, r5
	ldr	r0, [r4, #3368]
	lsls	r2, r2, #2
	bl	ftl_memset
	ldrh	r2, [r4, #320]
	mov	r1, r5
	ldr	r0, [r4, #348]
	lsls	r2, r2, #1
	bl	ftl_memset
	movs	r2, #16
	movs	r1, #255
	add	r0, r4, #2624
	bl	memset
	ldrh	r8, [r4, #240]
.L674:
	ldrh	r3, [r4, #242]
	cmp	r3, r8
	bls	.L716
	movs	r5, #0
	ldrh	fp, [r4, #232]
	ldr	r9, [r4, #3288]
	mov	r7, r5
	ldr	r3, [r4, #3176]
	mov	r10, #20
	ldr	r2, [r4, #3180]
	ldrh	r6, [r4, #312]
	b	.L717
.L677:
	str	r3, [sp, #8]
	mov	r1, r8
	ldr	r3, .L755+4
	str	r2, [sp, #12]
	ldrb	r0, [r3, r5]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #4]
	bl	FtlBbmIsBadBlock
	ldr	r3, [sp, #8]
	ldr	r2, [sp, #12]
	cbnz	r0, .L675
	ldr	r1, [sp, #4]
	mla	r0, r10, r7, r9
	lsls	r1, r1, #10
	str	r3, [r0, #8]
	str	r1, [r0, #4]
	mov	r1, r6
	muls	r1, r7, r1
	add	r7, r7, #1
	it	mi
	addmi	r1, r1, #3
	uxth	r7, r7
	bic	r1, r1, #3
	add	r1, r1, r2
	str	r1, [r0, #12]
.L675:
	adds	r5, r5, #1
.L717:
	uxth	r1, r5
	cmp	fp, r1
	bhi	.L677
	cbnz	r7, .L678
.L715:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L674
.L678:
	mov	r0, r9
	mov	r9, #0
	movs	r2, #1
	mov	r1, r7
	bl	FlashReadPages
.L679:
	uxth	r3, r9
	cmp	r7, r3
	bls	.L715
	movs	r3, #20
	mul	r10, r3, r9
	ldr	r3, [r4, #3288]
	add	r2, r3, r10
	ldr	r3, [r3, r10]
	ldr	r5, [r2, #4]
	ldr	r6, [r2, #12]
	adds	r3, r3, #1
	ubfx	r5, r5, #10, #16
	bne	.L682
	mov	fp, #16
	movw	r3, #65535
.L684:
	ldr	r0, [r4, #3288]
	str	r3, [sp, #4]
	add	r0, r0, r10
	ldr	r2, [r0, #4]
	adds	r2, r2, #1
	str	r2, [r0, #4]
	movs	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldrh	r2, [r6]
	ldr	r3, [sp, #4]
	cmp	r2, r3
	bne	.L681
	ldr	r3, [r4, #3288]
	mov	r2, #-1
	str	r2, [r3, r10]
	ldr	r3, [r4, #3288]
	ldr	r3, [r3, r10]
	cmp	r3, r2
	bne	.L682
.L683:
	movs	r1, #1
	b	.L754
.L681:
	ldr	r2, [r4, #3288]
	ldr	r2, [r2, r10]
	adds	r2, r2, #1
	bne	.L682
	add	fp, fp, #-1
	uxth	fp, fp
	cmp	fp, #0
	bne	.L684
	b	.L683
.L682:
	ldr	r2, [r4, #2588]
	ldr	r3, [r6, #4]
	adds	r1, r2, #1
	beq	.L685
	cmp	r2, r3
	bhi	.L686
.L685:
	adds	r2, r3, #1
	itt	ne
	addne	r2, r3, #1
	strne	r2, [r4, #2588]
.L686:
	ldrh	r2, [r6]
	movw	r1, #61604
	cmp	r2, r1
	beq	.L688
	bhi	.L689
	movw	r3, #61574
	cmp	r2, r3
	beq	.L690
.L687:
	add	r9, r9, #1
	b	.L679
.L689:
	movw	r3, #61634
	cmp	r2, r3
	beq	.L691
	movw	r3, #65535
	cmp	r2, r3
	bne	.L687
	movs	r1, #0
.L754:
	mov	r0, r5
	bl	FtlFreeSysBlkQueueIn
	b	.L687
.L691:
	ldrh	r2, [r4, #3436]
	ldr	r3, [r4, #328]
	cmp	r2, r3
	bls	.L693
	movw	r2, #1225
	ldr	r1, .L755+8
	ldr	r0, .L755+12
	bl	sftl_printk
.L693:
	ldr	r1, [r4, #328]
	ldrh	r0, [r4, #3436]
	ldr	r10, [r4, #3380]
	uxth	ip, r1
	add	r3, ip, #-1
	sub	ip, ip, r0
	add	ip, ip, #-1
	sxth	r3, r3
	sxth	ip, ip
.L694:
	cmp	r3, ip
	bgt	.L700
	cmp	r3, #0
	bge	.L732
	b	.L687
.L700:
	ldr	r2, [r6, #4]
	lsl	fp, r3, #2
	mov	lr, r2
	ldr	r2, [r10, r3, lsl #2]
	cmp	lr, r2
	bls	.L695
	ldr	r2, [r10]
	cbnz	r2, .L696
	cmp	r1, r0
	itt	ne
	addne	r0, r0, #1
	strhne	r0, [r4, #3436]	@ movhi
.L696:
	uxth	ip, r3
	movs	r2, #0
.L697:
	uxth	r0, r2
	sxth	r1, r2
	cmp	ip, r0
	bhi	.L698
	ldr	r2, [r4, #3380]
	cmp	r3, #0
	ldr	r1, [r6, #4]
	str	r1, [r2, fp]
	ldr	r2, [r4, #3356]
	strh	r5, [r2, r3, lsl #1]	@ movhi
	blt	.L687
	ldrh	r0, [r4, #3436]
	ldr	r2, [r4, #328]
	subs	r2, r2, r0
	subs	r2, r2, #1
	sxth	r2, r2
	cmp	r3, r2
	bgt	.L687
.L732:
	ldr	r2, [r4, #3380]
	adds	r0, r0, #1
	ldr	r1, [r6, #4]
	strh	r0, [r4, #3436]	@ movhi
	str	r1, [r2, r3, lsl #2]
	ldr	r2, [r4, #3356]
.L752:
	strh	r5, [r2, r3, lsl #1]	@ movhi
	b	.L687
.L698:
	ldr	r0, [r4, #3380]
	adds	r2, r2, #1
	add	lr, r0, r1, lsl #2
	ldr	lr, [lr, #4]
	str	lr, [r0, r1, lsl #2]
	ldr	r0, [r4, #3356]
	add	lr, r0, r1, lsl #1
	ldrh	lr, [lr, #2]
	strh	lr, [r0, r1, lsl #1]	@ movhi
	b	.L697
.L695:
	subs	r3, r3, #1
	sxth	r3, r3
	b	.L694
.L756:
	.align	2
.L755:
	.word	.LANCHOR0
	.word	.LANCHOR0+260
	.word	.LANCHOR1+231
	.word	.LC8
.L690:
	ldrh	r2, [r4, #344]
	ldrh	r3, [r4, #320]
	cmp	r2, r3
	bls	.L703
	movw	r2, #1266
	ldr	r1, .L757
	ldr	r0, .L757+4
	bl	sftl_printk
.L703:
	ldrh	lr, [r4, #320]
	ldrh	r0, [r4, #344]
	ldr	r10, [r4, #3368]
	add	ip, lr, #-1
	sxth	r3, ip
	sub	ip, ip, r0
.L704:
	cmp	r3, ip
	ble	.L709
	ldr	r2, [r6, #4]
	lsl	fp, r3, #2
	ldr	r1, [r10, r3, lsl #2]
	cmp	r2, r1
	bls	.L705
	ldr	r2, [r10]
	cbnz	r2, .L706
	cmp	lr, r0
	itt	ne
	addne	r0, r0, #1
	strhne	r0, [r4, #344]	@ movhi
.L706:
	uxth	ip, r3
	movs	r2, #0
.L707:
	uxth	r0, r2
	sxth	r1, r2
	cmp	ip, r0
	bhi	.L708
	ldr	r2, [r4, #3368]
	ldr	r1, [r6, #4]
	str	r1, [r2, fp]
	ldr	r2, [r4, #348]
	strh	r5, [r2, r3, lsl #1]	@ movhi
.L709:
	cmp	r3, #0
	blt	.L687
	ldrh	r2, [r4, #320]
	ldrh	r1, [r4, #344]
	subs	r2, r2, #1
	subs	r2, r2, r1
	sxth	r2, r2
	cmp	r3, r2
	bgt	.L687
	adds	r1, r1, #1
	ldr	r2, [r4, #3368]
	strh	r1, [r4, #344]	@ movhi
	ldr	r1, [r6, #4]
	str	r1, [r2, r3, lsl #2]
	ldr	r2, [r4, #348]
	b	.L752
.L708:
	ldr	r0, [r4, #3368]
	adds	r2, r2, #1
	add	lr, r0, r1, lsl #2
	ldr	lr, [lr, #4]
	str	lr, [r0, r1, lsl #2]
	ldr	r0, [r4, #348]
	add	lr, r0, r1, lsl #1
	ldrh	lr, [lr, #2]
	strh	lr, [r0, r1, lsl #1]	@ movhi
	b	.L707
.L705:
	subs	r3, r3, #1
	sxth	r3, r3
	b	.L704
.L688:
	ldrh	r1, [r4, #2624]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L711
	strh	r5, [r4, #2624]	@ movhi
.L753:
	str	r3, [r4, #2632]
	b	.L687
.L711:
	ldrh	r0, [r4, #2628]
	cmp	r0, r2
	beq	.L712
	movs	r1, #1
	bl	FtlFreeSysBlkQueueIn
.L712:
	ldr	r2, [r4, #2632]
	ldr	r3, [r6, #4]
	cmp	r2, r3
	bcs	.L713
	ldrh	r3, [r4, #2624]
	strh	r5, [r4, #2624]	@ movhi
	strh	r3, [r4, #2628]	@ movhi
	ldr	r3, [r6, #4]
	b	.L753
.L713:
	strh	r5, [r4, #2628]	@ movhi
	b	.L687
.L716:
	ldr	r1, [r4, #3356]
	ldrh	r3, [r1]
	cbz	r3, .L718
.L721:
	ldr	r1, [r4, #348]
	ldrh	r2, [r1]
	cmp	r2, #0
	beq	.L719
.L720:
	ldrh	r2, [r4, #3436]
	ldr	r3, [r4, #328]
	cmp	r2, r3
	bls	.L751
	movw	r2, #1391
	ldr	r1, .L757
	ldr	r0, .L757+4
	bl	sftl_printk
.L751:
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L718:
	ldrh	r2, [r4, #3436]
	cmp	r2, #0
	beq	.L721
	ldr	r0, [r4, #328]
.L722:
	sxth	r2, r3
	cmp	r2, r0
	bcs	.L721
	ldrh	r5, [r1, r2, lsl #1]
	adds	r3, r3, #1
	cmp	r5, #0
	beq	.L722
	mov	r3, r2
	movs	r5, #0
.L723:
	ldr	r1, [r4, #328]
	cmp	r3, r1
	bcs	.L721
	ldr	r1, [r4, #3356]
	subs	r0, r3, r2
	ldrh	r6, [r1, r3, lsl #1]
	strh	r6, [r1, r0, lsl #1]	@ movhi
	ldr	r1, [r4, #3380]
	ldr	r6, [r1, r3, lsl #2]
	str	r6, [r1, r0, lsl #2]
	ldr	r1, [r4, #3356]
	strh	r5, [r1, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	sxth	r3, r3
	b	.L723
.L719:
	ldrh	r3, [r4, #344]
	cmp	r3, #0
	beq	.L720
	ldrh	r0, [r4, #320]
.L728:
	sxth	r3, r2
	cmp	r3, r0
	mov	r5, r3
	bge	.L720
	ldrh	r6, [r1, r3, lsl #1]
	adds	r2, r2, #1
	cmp	r6, #0
	beq	.L728
	movs	r0, #0
.L729:
	ldrh	r2, [r4, #320]
	cmp	r3, r2
	bge	.L720
	ldr	r2, [r4, #348]
	subs	r1, r3, r5
	ldrh	r6, [r2, r3, lsl #1]
	strh	r6, [r2, r1, lsl #1]	@ movhi
	ldr	r2, [r4, #3368]
	ldr	r6, [r2, r3, lsl #2]
	str	r6, [r2, r1, lsl #2]
	ldr	r2, [r4, #348]
	strh	r0, [r2, r3, lsl #1]	@ movhi
	adds	r3, r3, #1
	sxth	r3, r3
	b	.L729
.L758:
	.align	2
.L757:
	.word	.LANCHOR1+231
	.word	.LC8
	.fnend
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	1
	.global	FtlLoadBbt
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r8, #61649
	ldr	r4, .L788
	ldr	r3, [r4, #3300]
	addw	r7, r4, #3452
	ldr	r6, [r4, #3332]
	str	r3, [r4, #3460]
	str	r6, [r4, #3464]
	bl	FtlBbtMemInit
	ldrh	r5, [r4, #298]
	subs	r5, r5, #1
	uxth	r5, r5
.L760:
	ldrh	r3, [r4, #298]
	subs	r3, r3, #16
	cmp	r5, r3
	ble	.L763
	lsls	r3, r5, #10
	movs	r2, #1
	mov	r1, r2
	mov	r0, r7
	str	r3, [r4, #3456]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	bne	.L761
	ldr	r3, [r4, #3456]
	movs	r2, #1
	mov	r1, r2
	mov	r0, r7
	adds	r3, r3, #1
	str	r3, [r4, #3456]
	bl	FlashReadPages
.L761:
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L762
	ldrh	r3, [r6]
	cmp	r3, r8
	bne	.L762
	ldr	r3, [r6, #4]
	strh	r5, [r4, #352]	@ movhi
	str	r3, [r4, #360]
	ldrh	r3, [r6, #8]
	strh	r3, [r4, #356]	@ movhi
.L763:
	ldrh	r3, [r4, #352]
	movw	r2, #65535
	cmp	r3, r2
	beq	.L777
	ldrh	r3, [r4, #356]
	cmp	r3, r2
	beq	.L767
	lsls	r3, r3, #10
	movs	r2, #1
	mov	r1, r2
	ldr	r0, .L788+4
	str	r3, [r4, #3456]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L767
	ldrh	r2, [r6]
	movw	r3, #61649
	cmp	r2, r3
	bne	.L767
	ldr	r3, [r6, #4]
	ldr	r2, [r4, #360]
	cmp	r3, r2
	bls	.L767
	ldrh	r2, [r4, #356]
	str	r3, [r4, #360]
	ldrh	r3, [r6, #8]
	strh	r2, [r4, #352]	@ movhi
	strh	r3, [r4, #356]	@ movhi
.L767:
	ldr	r7, .L788+4
	movs	r1, #1
	ldrh	r0, [r4, #352]
	movw	r8, #61649
	bl	FtlGetLastWrittenPage
	sxth	r5, r0
	adds	r0, r0, #1
	strh	r0, [r4, #354]	@ movhi
.L769:
	cmp	r5, #0
	bge	.L772
	movs	r2, #251
	ldr	r1, .L788+8
	ldr	r0, .L788+12
	bl	sftl_printk
.L771:
	ldrh	r3, [r6, #10]
	ldrh	r0, [r6, #12]
	strh	r3, [r4, #358]	@ movhi
	movw	r3, #65535
	cmp	r0, r3
	beq	.L774
	ldr	r2, [r4, #228]
	cmp	r0, r2
	beq	.L774
	ldrh	r3, [r4, #242]
	lsrs	r3, r3, #2
	cmp	r2, r3
	bcs	.L774
	cmp	r0, r3
	bcs	.L774
	bl	FtlSysBlkNumInit
.L774:
	ldr	r6, .L788+16
	movs	r5, #0
.L775:
	ldrh	r3, [r4, #254]
	cmp	r5, r3
	bcc	.L776
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, pc}
.L762:
	subs	r5, r5, #1
	uxth	r5, r5
	b	.L760
.L772:
	ldrh	r3, [r4, #352]
	movs	r2, #1
	mov	r1, r2
	mov	r0, r7
	orr	r3, r5, r3, lsl #10
	str	r3, [r4, #3456]
	ldr	r3, [r4, #3300]
	str	r3, [r4, #3460]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L770
	ldrh	r3, [r6]
	cmp	r3, r8
	beq	.L771
.L770:
	subs	r5, r5, #1
	sxth	r5, r5
	b	.L769
.L776:
	ldrh	r2, [r4, #3388]
	ldr	r1, [r4, #3460]
	ldr	r0, [r6, #4]!
	lsls	r2, r2, #2
	mla	r1, r5, r2, r1
	adds	r5, r5, #1
	bl	ftl_memcpy
	b	.L775
.L777:
	mov	r0, #-1
	pop	{r4, r5, r6, r7, r8, pc}
.L789:
	.align	2
.L788:
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.word	.LANCHOR1+245
	.word	.LC8
	.word	.LANCHOR0+376
	.fnend
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	1
	.global	FlashProgPages
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FlashProgPages, %function
FlashProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r1
	ldr	r5, .L827
	mov	r4, r0
	mov	r6, r0
	mov	r9, #0
	.pad #44
	sub	sp, sp, #44
	ldrh	fp, [r5, #12]
	str	r2, [sp, #4]
	lsl	r3, fp, #3
	str	r3, [sp]
.L791:
	cmp	r9, r10
	bne	.L805
	ldr	r7, .L827+4
	movs	r6, #0
	ldr	r8, .L827+28
	ldr	r10, .L827+32
.L806:
	cmp	r9, r6
	beq	.L799
	ldr	r2, [r5, #3320]
	movs	r3, #0
	movs	r1, #1
	add	r0, sp, #20
	str	r3, [r2]
	ldr	r2, [r5, #3328]
	str	r3, [r2]
	ldr	r3, [r4, #4]
	ldr	r2, [sp, #4]
	str	r3, [sp, #24]
	ldr	r3, [r5, #3320]
	str	r3, [sp, #28]
	ldr	r3, [r5, #3328]
	str	r3, [sp, #32]
	bl	FlashReadPages
	ldr	fp, [sp, #20]
	cmp	fp, #-1
	bne	.L807
	ldr	r1, [r4, #4]
	mov	r0, r7
	bl	sftl_printk
	str	fp, [r4]
.L807:
	ldr	fp, [sp, #20]
	cmp	fp, #256
	bne	.L808
	ldr	r1, [r4, #4]
	mov	r0, r8
	bl	sftl_printk
	str	fp, [r4]
.L808:
	ldr	r3, [r4, #12]
	cbz	r3, .L809
	ldr	r2, [r3]
	ldr	r3, [r5, #3328]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L809
	ldr	r1, [r4, #4]
	mov	r0, r10
	bl	sftl_printk
	mov	r3, #-1
	str	r3, [r4]
.L809:
	ldr	r3, [r4, #8]
	cbz	r3, .L810
	ldr	r2, [r3]
	ldr	r3, [r5, #3320]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L810
	ldr	r1, [r4, #4]
	ldr	r0, .L827+8
	bl	sftl_printk
	mov	r3, #-1
	str	r3, [r4]
.L810:
	adds	r6, r6, #1
	adds	r4, r4, #20
	b	.L806
.L805:
	ldr	r3, [r6, #8]
	cbz	r3, .L792
	ldr	r3, [r6, #12]
	cbnz	r3, .L793
.L792:
	movs	r2, #148
	ldr	r1, .L827+12
	ldr	r0, .L827+16
	bl	sftl_printk
.L793:
	add	r2, sp, #12
	add	r1, sp, #16
	mov	r0, r6
	bl	l2p_addr_tran
	ldr	r7, [sp, #12]
	cmp	r7, #3
	bls	.L794
.L826:
	mov	r3, #-1
	str	r3, [r6]
	b	.L795
.L794:
	cbnz	r7, .L796
	ldr	r3, [sp, #16]
	ldr	r2, [sp]
	cmp	r2, r3
	bls	.L796
	ldr	r5, .L827+12
	ldr	r6, .L827+20
	ldr	r8, .L827+36
	b	.L825
.L798:
	mov	r3, #-1
	ldr	r2, [r4, #-16]
	str	r3, [r4, #-20]
	mov	r1, r5
	mov	r0, r6
	adds	r7, r7, #1
	bl	sftl_printk
	movs	r3, #16
	movs	r2, #4
	ldr	r1, [r4, #-12]
	mov	r0, r8
	bl	rknand_print_hex
	movs	r3, #4
	ldr	r1, [r4, #-8]
	mov	r2, r3
	ldr	r0, .L827+24
	bl	rknand_print_hex
.L825:
	cmp	r7, r10
	add	r4, r4, #20
	bne	.L798
	bl	dump_stack
.L799:
	movs	r0, #0
	add	sp, sp, #44
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L796:
	ldr	r1, [r6, #8]
	lsls	r3, r1, #26
	beq	.L812
	ldr	r8, [r5, #3320]
	cmp	r1, r8
	beq	.L800
	ldrh	r2, [r5, #258]
	mov	r0, r8
	lsls	r2, r2, #9
	bl	ftl_memcpy
.L800:
	ldr	r3, [r6, #12]
	mov	r2, r8
	ldr	r1, [sp, #16]
	ldrb	r0, [sp, #12]	@ zero_extendqisi2
	ldr	r7, [r5, #3252]
	blx	r7
	cbnz	r0, .L801
	str	r0, [r6]
.L802:
	ldrh	r3, [r5, #14]
	cmp	r3, #4
	bne	.L795
	ldr	r3, [r6, #12]
	add	r2, r8, #2048
	ldr	r1, [sp, #16]
	ldrb	r0, [sp, #12]	@ zero_extendqisi2
	adds	r3, r3, #8
	ldr	r7, [r5, #3252]
	add	r1, r1, fp
	blx	r7
	cmp	r0, #0
	bne	.L826
.L795:
	add	r9, r9, #1
	adds	r6, r6, #20
	b	.L791
.L812:
	mov	r8, r1
	b	.L800
.L801:
	mov	r3, #-1
	str	r3, [r6]
	b	.L802
.L828:
	.align	2
.L827:
	.word	.LANCHOR0
	.word	.LC95
	.word	.LC98
	.word	.LANCHOR1+256
	.word	.LC8
	.word	.LC86
	.word	.LC88
	.word	.LC96
	.word	.LC97
	.word	.LC87
	.fnend
	.size	FlashProgPages, .-FlashProgPages
	.align	1
	.global	FtlLowFormatEraseBlock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, #0
	ldr	r4, .L874
	mov	r7, r1
	mov	r6, r10
	mov	r5, r10
	mov	r9, #20
	.pad #20
	sub	sp, sp, #20
	mov	fp, r4
	str	r0, [sp, #4]
	str	r0, [r4, #3284]
.L830:
	ldrh	r1, [r4, #232]
	uxth	r3, r10
	cmp	r1, r3
	bhi	.L835
	cmp	r6, #0
	beq	.L829
	mov	r8, #0
	mov	r9, #20
	mov	r2, r6
	movs	r1, #0
	ldr	r0, [r4, #3268]
	bl	FlashEraseBlocks
.L838:
	uxth	r3, r8
	cmp	r6, r3
	bhi	.L840
	cmp	r7, #0
	beq	.L857
	ldrh	r3, [r4, #304]
	mov	r10, #1
	str	r3, [sp, #8]
.L841:
	mov	r8, #0
	mov	fp, #20
.L851:
	mov	r9, #0
	mov	r6, r9
.L842:
	ldrh	r1, [r4, #232]
	uxth	r3, r9
	cmp	r1, r3
	bhi	.L846
	cbz	r6, .L829
	mov	r9, #0
	movs	r3, #1
	mov	r2, r10
	mov	r1, r6
	ldr	r0, [r4, #3268]
	bl	FlashProgPages
.L848:
	uxth	r3, r9
	cmp	r6, r3
	bhi	.L850
	add	r8, r8, #1
	ldr	r2, [sp, #8]
	uxth	r3, r8
	cmp	r2, r3
	bhi	.L851
	mov	r8, #0
	mov	r9, #20
.L852:
	uxth	r3, r8
	cmp	r6, r3
	bhi	.L854
	ldr	r3, [sp, #4]
	cmp	r3, #63
	bls	.L855
	cbz	r7, .L829
.L855:
	mov	r2, r6
	mov	r1, r10
	ldr	r0, [r4, #3268]
	bl	FlashEraseBlocks
.L829:
	mov	r0, r5
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L835:
	uxth	r3, r10
	ldr	r0, [r4, #3268]
	movs	r2, #0
	mul	r1, r9, r3
	add	r3, r3, r4
	str	r2, [r0, r1]
	ldr	r1, [sp, #4]
	ldrb	r0, [r3, #260]	@ zero_extendqisi2
	bl	V2P_block
	mov	r8, r0
	cbz	r7, .L831
	bl	IsBlkInVendorPart
	cbnz	r0, .L832
.L831:
	mov	r0, r8
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L833
	ldr	r1, [r4, #3268]
	lsl	r3, r8, #10
	ldr	r0, [fp, #3336]
	mla	r1, r9, r6, r1
	str	r3, [r1, #4]
	ldr	r3, [r4, #3316]
	str	r3, [r1, #8]
	ldrh	r3, [r4, #312]
	muls	r3, r6, r3
	add	r6, r6, #1
	it	mi
	addmi	r3, r3, #3
	uxth	r6, r6
	bic	r3, r3, #3
	add	r3, r3, r0
	str	r3, [r1, #12]
.L832:
	add	r10, r10, #1
	b	.L830
.L833:
	adds	r5, r5, #1
	uxth	r5, r5
	b	.L832
.L840:
	mul	r3, r9, r8
	ldr	r2, [r4, #3268]
	adds	r1, r2, r3
	ldr	r3, [r2, r3]
	adds	r3, r3, #1
	bne	.L839
	ldr	r0, [r1, #4]
	adds	r5, r5, #1
	uxth	r5, r5
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
.L839:
	add	r8, r8, #1
	b	.L838
.L857:
	movs	r3, #2
	mov	r10, r7
	str	r3, [sp, #8]
	b	.L841
.L846:
	uxth	r3, r9
	ldr	r0, [r4, #3268]
	movs	r2, #0
	mul	r1, fp, r3
	add	r3, r3, r4
	str	r2, [r0, r1]
	ldr	r1, [sp, #4]
	ldrb	r0, [r3, #260]	@ zero_extendqisi2
	bl	V2P_block
	str	r0, [sp, #12]
	cbz	r7, .L843
	bl	IsBlkInVendorPart
	cbnz	r0, .L844
.L843:
	ldr	r0, [sp, #12]
	bl	FtlBbmIsBadBlock
	cbnz	r0, .L844
	ldr	r1, [r4, #3268]
	ldr	r3, [sp, #12]
	ldr	r2, .L874
	mla	r1, fp, r6, r1
	ldr	r0, [r2, #3316]
	add	r3, r8, r3, lsl #10
	str	r3, [r1, #4]
	ldr	r3, [r4, #3312]
	str	r3, [r1, #8]
	ldrh	r3, [r4, #312]
	muls	r3, r6, r3
	add	r6, r6, #1
	it	mi
	addmi	r3, r3, #3
	uxth	r6, r6
	bic	r3, r3, #3
	add	r3, r3, r0
	str	r3, [r1, #12]
.L844:
	add	r9, r9, #1
	b	.L842
.L850:
	mul	r3, fp, r9
	ldr	r2, [r4, #3268]
	adds	r1, r2, r3
	ldr	r3, [r2, r3]
	cbz	r3, .L849
	ldr	r0, [r1, #4]
	adds	r5, r5, #1
	uxth	r5, r5
	ubfx	r0, r0, #10, #16
	bl	FtlBbmMapBadBlock
.L849:
	add	r9, r9, #1
	b	.L848
.L854:
	cbz	r7, .L853
	mul	r3, r9, r8
	ldr	r2, [r4, #3268]
	adds	r1, r2, r3
	ldr	r3, [r2, r3]
	cbnz	r3, .L853
	ldr	r0, [r1, #4]
	movs	r1, #1
	ubfx	r0, r0, #10, #16
	bl	FtlFreeSysBlkQueueIn
.L853:
	add	r8, r8, #1
	b	.L852
.L875:
	.align	2
.L874:
	.word	.LANCHOR0
	.fnend
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	1
	.global	FlashTestBlk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FlashTestBlk, %function
FlashTestBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #11
	push	{r4, r5, lr}
	.save {r4, r5, lr}
	mov	r5, r0
	.pad #92
	sub	sp, sp, #92
	bls	.L879
	ldr	r3, .L881
	add	r0, sp, #24
	movs	r2, #32
	movs	r1, #165
	str	r0, [sp, #16]
	lsls	r5, r5, #10
	ldr	r4, [r3, #3324]
	str	r4, [sp, #12]
	bl	memset
	mov	r3, #1515870810
	movs	r2, #1
	str	r3, [r4]	@ unaligned
	mov	r1, r2
	str	r3, [r4, #4]	@ unaligned
	add	r0, sp, #4
	str	r5, [sp, #8]
	bl	FlashEraseBlocks
	movs	r3, #1
	add	r0, sp, #4
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldr	r3, [sp, #4]
	cbnz	r3, .L880
	adds	r3, r5, #1
	add	r0, sp, #4
	str	r3, [sp, #8]
	movs	r3, #1
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldr	r4, [sp, #4]
	adds	r4, r4, #0
	it	ne
	movne	r4, #1
	negs	r4, r4
.L878:
	movs	r2, #1
	movs	r1, #0
	add	r0, sp, #4
	str	r5, [sp, #8]
	bl	FlashEraseBlocks
.L876:
	mov	r0, r4
	add	sp, sp, #92
	@ sp needed
	pop	{r4, r5, pc}
.L880:
	mov	r4, #-1
	b	.L878
.L879:
	movs	r4, #0
	b	.L876
.L882:
	.align	2
.L881:
	.word	.LANCHOR0
	.fnend
	.size	FlashTestBlk, .-FlashTestBlk
	.align	1
	.global	FtlBbmTblFlush
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #8
	movs	r5, #0
	ldr	r4, .L895
	movs	r1, #0
	ldr	r0, [r4, #3300]
	add	r6, r4, #376
	ldr	r3, [r4, #3332]
	ldrh	r2, [r4, #310]
	str	r0, [r4, #3460]
	str	r3, [r4, #3464]
	bl	ftl_memset
.L884:
	ldrh	r3, [r4, #254]
	cmp	r5, r3
	blt	.L885
	ldr	r6, [r4, #3464]
	movs	r2, #16
	movs	r1, #255
	ldr	r8, .L895+12
	ldr	r9, .L895+16
	movs	r5, #0
	mov	r0, r6
	mov	r7, r5
	bl	memset
	movw	r3, #61649
	strh	r3, [r6]	@ movhi
	ldr	r3, [r4, #360]
	str	r3, [r6, #4]
	ldrh	r3, [r4, #352]
	strh	r3, [r6, #2]	@ movhi
	ldrh	r3, [r4, #356]
	strh	r3, [r6, #8]	@ movhi
	ldrh	r3, [r4, #358]
	strh	r3, [r6, #10]	@ movhi
	ldr	r3, [r4, #228]
	strh	r3, [r6, #12]	@ movhi
	ldr	r3, [r4, #2604]
	strh	r3, [r6, #14]	@ movhi
.L886:
	ldr	r3, [r4, #3300]
	mov	r10, #0
	ldrh	r2, [r4, #354]
	ldrh	r1, [r4, #352]
	str	r3, [r4, #3460]
	ldr	r3, [r4, #3332]
	str	r10, [r4, #3452]
	str	r3, [r4, #3464]
	orr	r3, r2, r1, lsl #10
	ldrh	r0, [r6, #10]
	str	r3, [r4, #3456]
	ldrh	r3, [r4, #356]
	str	r0, [sp]
	mov	r0, r8
	bl	sftl_printk
	ldrh	r3, [r4, #304]
	ldrh	r2, [r4, #354]
	subs	r3, r3, #1
	cmp	r2, r3
	blt	.L887
	ldr	r3, [r4, #360]
	ldrh	r2, [r4, #352]
	ldr	r0, [r4, #3268]
	adds	r3, r3, #1
	strh	r10, [r4, #354]	@ movhi
	str	r3, [r4, #360]
	str	r3, [r6, #4]
	ldrh	r3, [r4, #356]
	strh	r2, [r6, #8]	@ movhi
	strh	r2, [r4, #356]	@ movhi
	movs	r2, #1
	strh	r3, [r4, #352]	@ movhi
	mov	r1, r2
	lsls	r3, r3, #10
	str	r3, [r4, #3456]
	str	r3, [r0, #4]
	bl	FlashEraseBlocks
.L887:
	movs	r3, #1
	mov	r0, r9
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldrh	r3, [r4, #354]
	adds	r3, r3, #1
	strh	r3, [r4, #354]	@ movhi
	ldr	r3, [r4, #3452]
	adds	r2, r3, #1
	bne	.L888
	adds	r5, r5, #1
	ldr	r1, [r4, #3456]
	uxth	r5, r5
	ldr	r0, .L895+4
	bl	sftl_printk
	cmp	r5, #3
	bls	.L886
	mov	r2, r5
	ldr	r1, [r4, #3456]
	ldr	r0, .L895+8
	bl	sftl_printk
.L890:
	b	.L890
.L885:
	ldrh	r2, [r4, #3388]
	ldr	r3, [r4, #3460]
	ldr	r1, [r6, #4]!
	mul	r0, r2, r5
	lsls	r2, r2, #2
	adds	r5, r5, #1
	add	r0, r3, r0, lsl #2
	bl	ftl_memcpy
	b	.L884
.L891:
	movs	r7, #1
	b	.L886
.L888:
	adds	r7, r7, #1
	cmp	r7, #1
	ble	.L891
	cmp	r3, #256
	beq	.L886
	movs	r0, #0
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L896:
	.align	2
.L895:
	.word	.LANCHOR0
	.word	.LC100
	.word	.LC101
	.word	.LC99
	.word	.LANCHOR0+3452
	.fnend
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	1
	.global	allocate_data_superblock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r0
	ldr	r4, .L933
	.pad #20
	sub	sp, sp, #20
	ldr	r9, .L933+20
.L898:
	ldr	r3, .L933
	ldr	r2, .L933
	ldrh	r3, [r3, #224]
	ldrh	r2, [r2, #2528]
	add	r3, r3, r2
	ldr	r2, .L933
	ldrh	r2, [r2, #240]
	cmp	r3, r2
	ble	.L899
	movw	r2, #2660
	mov	r1, r9
	ldr	r0, .L933+4
	bl	sftl_printk
.L899:
	ldr	r3, .L933+8
	cmp	r5, r3
	bne	.L925
	ldrh	r3, [r4, #224]
	ldr	r1, [r4, #3164]
	mul	r2, r1, r3
	lsrs	r1, r3, #1
	adds	r1, r1, #1
	add	r1, r1, r2, lsr #2
	uxth	r1, r1
	cbz	r1, .L900
	subs	r1, r1, #1
	uxth	r1, r1
.L900:
	ldr	r0, .L933+12
	bl	List_pop_index_node
	ldrh	r3, [r4, #224]
	mov	r6, r0
	uxth	r8, r0
	cbnz	r3, .L901
	movw	r2, #2669
	mov	r1, r9
	ldr	r0, .L933+4
	bl	sftl_printk
.L901:
	ldrh	r3, [r4, #224]
	subs	r3, r3, #1
	strh	r3, [r4, #224]	@ movhi
	ldrh	r3, [r4, #240]
	cmp	r3, r8
	bls	.L898
	ldr	r3, [r4, #72]
	uxth	r6, r6
	ldrh	r7, [r3, r6, lsl #1]
	cmp	r7, #0
	bne	.L898
	strh	r8, [r5]	@ movhi
	mov	r0, r5
	bl	make_superblock
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cbnz	r3, .L903
	ldr	r3, [r4, #72]
	movw	r2, #65535
	mov	r0, r8
	strh	r2, [r3, r6, lsl #1]	@ movhi
	bl	INSERT_DATA_LIST
	ldrh	r2, [r4, #2528]
	ldrh	r3, [r4, #224]
	add	r3, r3, r2
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	ble	.L898
	movw	r2, #2683
	mov	r1, r9
	ldr	r0, .L933+4
	bl	sftl_printk
	b	.L898
.L925:
	movs	r1, #0
	b	.L900
.L903:
	ldrh	r2, [r4, #2528]
	ldrh	r3, [r4, #224]
	add	r3, r3, r2
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	ble	.L905
	movw	r2, #2686
	mov	r1, r9
	ldr	r0, .L933+4
	bl	sftl_printk
.L905:
	ldr	r1, [r4, #3268]
	mov	r10, #20
	ldrh	r2, [r4, #232]
	add	fp, r5, #16
	mov	r0, fp
	mov	ip, #0
	mov	r3, r1
	mla	r2, r10, r2, r1
	str	r2, [sp, #4]
.L906:
	ldr	r2, [sp, #4]
	cmp	r2, r3
	bne	.L908
	cbnz	r7, .L909
	movw	r2, #2697
	mov	r1, r9
	ldr	r0, .L933+4
	bl	sftl_printk
.L909:
	ldrh	r3, [r4, #172]
	cmp	r3, r8
	bne	.L910
	movw	r2, #2699
	mov	r1, r9
	ldr	r0, .L933+4
	bl	sftl_printk
.L910:
	ldrb	r3, [r5, #8]	@ zero_extendqisi2
	ldr	r2, [r4, #2536]
	cmp	r3, #0
	bne	.L911
	ldrh	r3, [r2, r6, lsl #1]
	cmp	r3, #0
	beq	.L912
	ldrh	r1, [r4, #292]
	add	r3, r3, r1
.L932:
	strh	r3, [r2, r6, lsl #1]	@ movhi
	ldr	r3, [r4, #2596]
	adds	r3, r3, #1
	str	r3, [r4, #2596]
.L914:
	ldr	r3, [r4, #2536]
	ldr	r2, [r4, #2612]
	ldrh	r0, [r4, #292]
	ldrh	r3, [r3, r6, lsl #1]
	ldrh	r1, [r4, #240]
	cmp	r3, r2
	ldr	r2, [r4, #2596]
	it	hi
	strhi	r3, [r4, #2612]
	ldr	r3, [r4, #2600]
	mla	r0, r2, r0, r3
	bl	__aeabi_uidiv
	ldr	r2, [r4, #3348]
	ldr	r1, [r4, #3268]
	str	r0, [r4, #2604]
	ldr	r3, [r2, #16]
	ldr	r0, .L933+16
	adds	r3, r3, #1
	str	r3, [r2, #16]
	movs	r2, #20
	mla	r2, r2, r7, r1
	adds	r3, r1, #4
	adds	r2, r2, #24
.L916:
	adds	r3, r3, #20
	cmp	r2, r3
	bne	.L917
	ldrb	r1, [r5, #8]	@ zero_extendqisi2
	mov	r2, r7
	ldr	r0, [r4, #3268]
	mov	r10, #0
	bl	FlashEraseBlocks
	mov	r3, r10
	movs	r1, #20
.L918:
	uxth	r2, r10
	cmp	r7, r2
	bhi	.L920
	cmp	r3, #0
	ble	.L921
	mov	r0, r8
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L921:
	ldrb	r2, [r5, #7]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L922
	ldr	r3, [r4, #72]
	movw	r2, #65535
	strh	r2, [r3, r6, lsl #1]	@ movhi
	b	.L898
.L908:
	str	ip, [r3, #8]
	movw	lr, #65535
	str	ip, [r3, #12]
	ldrh	r2, [r0], #2
	cmp	r2, lr
	beq	.L907
	mla	lr, r10, r7, r1
	adds	r7, r7, #1
	lsls	r2, r2, #10
	uxth	r7, r7
	str	r2, [lr, #4]
.L907:
	adds	r3, r3, #20
	b	.L906
.L912:
	movs	r3, #2
	b	.L932
.L911:
	ldrh	r3, [r2, r6, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r2, r6, lsl #1]	@ movhi
	ldr	r3, [r4, #2600]
	adds	r3, r3, #1
	str	r3, [r4, #2600]
	b	.L914
.L917:
	ldr	r1, [r3, #-20]
	ands	r1, r1, r0
	str	r1, [r3, #-20]
	b	.L916
.L920:
	mul	r2, r1, r10
	ldr	r0, [r4, #3268]
	add	ip, r0, r2
	ldr	r2, [r0, r2]
	adds	r0, r2, #1
	bne	.L919
	ldr	r0, [ip, #4]
	adds	r3, r3, #1
	str	r1, [sp, #12]
	str	r2, [sp, #8]
	ubfx	r0, r0, #10, #16
	str	r3, [sp, #4]
	bl	FtlBbmMapBadBlock
	ldr	r2, [sp, #8]
	ldr	r1, [sp, #12]
	ldr	r3, [sp, #4]
	strh	r2, [fp]	@ movhi
	ldrb	r2, [r5, #7]	@ zero_extendqisi2
	subs	r2, r2, #1
	strb	r2, [r5, #7]
.L919:
	add	r10, r10, #1
	add	fp, fp, #2
	b	.L918
.L922:
	ldrh	r3, [r4, #302]
	strh	r8, [r5]	@ movhi
	smulbb	r3, r3, r2
	movs	r2, #0
	strh	r2, [r5, #2]	@ movhi
	strb	r2, [r5, #6]
	ldr	r2, [r4, #2588]
	uxth	r3, r3
	strh	r3, [r5, #4]	@ movhi
	str	r2, [r5, #12]
	adds	r2, r2, #1
	str	r2, [r4, #2588]
	ldr	r2, [r4, #72]
	ldrh	r1, [r5]
	strh	r3, [r2, r1, lsl #1]	@ movhi
	ldrh	r3, [r5, #4]
	cbz	r3, .L923
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	cbnz	r3, .L924
.L923:
	mov	r2, #2752
	ldr	r1, .L933+20
	ldr	r0, .L933+4
	bl	sftl_printk
.L924:
	movs	r0, #0
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L934:
	.align	2
.L933:
	.word	.LANCHOR0
	.word	.LC8
	.word	.LANCHOR0+124
	.word	.LANCHOR0+2532
	.word	-1024
	.word	.LANCHOR1+271
	.fnend
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	1
	.global	FtlGcFreeBadSuperBlk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r0
	ldr	r4, .L947
	ldrh	r3, [r4, #3206]
	cbz	r3, .L936
	movs	r7, #0
	addw	r10, r4, #3208
.L937:
	ldrh	r2, [r4, #232]
	uxth	r3, r7
	cmp	r2, r3
	bhi	.L943
	bl	FtlGcReFreshBadBlk
.L936:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L943:
	uxtah	r3, r4, r7
	mov	r1, r8
	mov	fp, #0
	ldrb	r0, [r3, #260]	@ zero_extendqisi2
	bl	V2P_block
	mov	r9, r0
.L938:
	ldrh	r3, [r4, #3206]
	uxth	r5, fp
	cmp	r3, r5
	bhi	.L942
	adds	r7, r7, #1
	b	.L937
.L942:
	uxth	r6, fp
	add	r3, r4, r6, lsl #1
	ldrh	r3, [r3, #3208]
	cmp	r3, r9
	bne	.L939
	mov	r1, r9
	ldr	r0, .L947+4
	bl	sftl_printk
	mov	r0, r9
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	r1, [r4, #3206]
	add	r3, r10, r6, lsl #1
.L940:
	cmp	r5, r1
	bcc	.L941
	subs	r1, r1, #1
	strh	r1, [r4, #3206]	@ movhi
.L939:
	add	fp, fp, #1
	b	.L938
.L941:
	ldrh	r0, [r3, #2]!
	adds	r5, r5, #1
	uxth	r5, r5
	strh	r0, [r3, #-2]	@ movhi
	b	.L940
.L948:
	.align	2
.L947:
	.word	.LANCHOR0
	.word	.LC102
	.fnend
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	1
	.global	update_vpc_list
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	update_vpc_list, %function
update_vpc_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L961
	ldr	r3, [r4, #72]
	ldrh	r3, [r3, r0, lsl #1]
	cmp	r3, #0
	bne	.L950
	ldrh	r3, [r4, #222]
	cmp	r3, r0
	itt	eq
	movweq	r3, #65535
	strheq	r3, [r4, #222]	@ movhi
	ldrh	r3, [r4, #220]
	cmp	r3, r0
	itt	eq
	movweq	r3, #65535
	strheq	r3, [r4, #220]	@ movhi
	ldrh	r3, [r4, #172]
	cmp	r3, r0
	bne	.L953
	movw	r3, #65535
	strh	r3, [r4, #172]	@ movhi
.L954:
	mov	r1, r5
	ldr	r0, .L961+4
	bl	List_remove_node
	ldrh	r3, [r4, #2528]
	cbnz	r3, .L956
	movw	r2, #2828
	ldr	r1, .L961+8
	ldr	r0, .L961+12
	bl	sftl_printk
.L956:
	ldrh	r3, [r4, #2528]
	mov	r0, r5
	subs	r3, r3, #1
	strh	r3, [r4, #2528]	@ movhi
	bl	free_data_superblock
	mov	r0, r5
	bl	FtlGcFreeBadSuperBlk
	ldrh	r2, [r4, #2528]
	ldrh	r3, [r4, #224]
	add	r3, r3, r2
	ldrh	r2, [r4, #240]
	cmp	r3, r2
	ble	.L960
	movw	r2, #2831
	ldr	r1, .L961+8
	ldr	r0, .L961+12
	bl	sftl_printk
.L960:
	movs	r0, #1
	pop	{r3, r4, r5, pc}
.L953:
	ldrh	r3, [r4, #24]
	cmp	r3, r0
	beq	.L959
	ldrh	r3, [r4, #76]
	cmp	r3, r0
	beq	.L959
	ldrh	r3, [r4, #124]
	cmp	r3, r0
	bne	.L954
.L959:
	movs	r0, #0
	pop	{r3, r4, r5, pc}
.L950:
	bl	List_update_data_list
	b	.L959
.L962:
	.align	2
.L961:
	.word	.LANCHOR0
	.word	.LANCHOR0+2520
	.word	.LANCHOR1+296
	.word	.LC8
	.fnend
	.size	update_vpc_list, .-update_vpc_list
	.align	1
	.global	decrement_vpc_count
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r3, #65535
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	cmp	r0, r3
	mov	r4, r0
	ldr	r5, .L972
	beq	.L964
	ldr	r3, [r5, #72]
	ldrh	r2, [r3, r0, lsl #1]
	cbnz	r2, .L965
	mov	r1, r0
	ldr	r0, .L972+4
	bl	sftl_printk
	ldr	r3, [r5, #72]
	ldrh	r6, [r3, r4, lsl #1]
	cbz	r6, .L966
.L970:
	movs	r6, #0
.L963:
	mov	r0, r6
	pop	{r4, r5, r6, pc}
.L966:
	movw	r2, #2846
.L971:
	ldr	r1, .L972+8
	ldr	r0, .L972+12
	bl	sftl_printk
	b	.L963
.L965:
	subs	r2, r2, #1
	strh	r2, [r3, r0, lsl #1]	@ movhi
.L964:
	ldrh	r0, [r5, #3438]
	movw	r3, #65535
	cmp	r0, r3
	bne	.L968
	strh	r4, [r5, #3438]	@ movhi
	b	.L970
.L968:
	cmp	r4, r0
	beq	.L970
	bl	update_vpc_list
	ldr	r2, [r5, #2516]
	adds	r6, r0, #0
	ldr	r3, [r5, #2520]
	it	ne
	movne	r6, #1
	strh	r4, [r5, #3438]	@ movhi
	subs	r3, r3, r2
	asrs	r2, r3, #1
	ldr	r3, .L972+16
	muls	r3, r2, r3
	ldr	r2, [r5, #72]
	uxth	r1, r3
	ldrh	r2, [r2, r1, lsl #1]
	cmp	r2, #0
	bne	.L963
	cmp	r4, r1
	beq	.L963
	movw	r2, #2862
	b	.L971
.L973:
	.align	2
.L972:
	.word	.LANCHOR0
	.word	.LC103
	.word	.LANCHOR1+312
	.word	.LC8
	.word	-1431655765
	.fnend
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	1
	.global	FtlSuperblockPowerLostFix
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #24
	sub	sp, sp, #24
	ldr	r5, .L987
	mov	r3, #-1
	movs	r7, #0
	movw	r1, #61589
	str	r3, [sp, #20]
	mov	r4, r0
	ldr	r2, [r5, #3300]
	ldr	r6, [r5, #3332]
	str	r2, [sp, #12]
	mvn	r2, #2
	str	r6, [sp, #16]
	str	r2, [r6, #8]
	mvn	r2, #1
	str	r2, [r6, #12]
	ldrh	r2, [r0]
	strh	r7, [r6]	@ movhi
	strh	r2, [r6, #2]	@ movhi
	ldr	r2, [r5, #3300]
	str	r1, [r2]
	add	r1, r1, #304087040
	ldr	r2, [r5, #3300]
	add	r1, r1, #1269760
	addw	r1, r1, #1507
	str	r1, [r2, #4]
	ldrh	r2, [r0, #4]
	tst	r2, #1
	ite	eq
	moveq	r8, #6
	movne	r8, #7
.L980:
	ldrh	r3, [r4, #4]
	cbnz	r3, .L976
.L977:
	ldr	r2, [r5, #72]
	ldrh	r1, [r4]
	ldrh	r0, [r4, #4]
	ldrh	r3, [r2, r1, lsl #1]
	subs	r3, r3, r0
	strh	r3, [r2, r1, lsl #1]	@ movhi
	ldrh	r3, [r5, #302]
	strh	r3, [r4, #2]	@ movhi
	movs	r3, #0
	strb	r3, [r4, #6]
	strh	r3, [r4, #4]	@ movhi
	add	sp, sp, #24
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L976:
	mov	r0, r4
	bl	get_new_active_ppa
	str	r0, [sp, #8]
	adds	r0, r0, #1
	beq	.L977
	ldr	r3, [r5, #2592]
	movs	r1, #1
	add	r0, sp, #4
	str	r3, [r6, #4]
	adds	r3, r3, #1
	adds	r2, r3, #1
	it	eq
	moveq	r3, r7
	str	r3, [r5, #2592]
	movs	r3, #0
	mov	r2, r3
	bl	FlashProgPages
	ldrh	r0, [r4]
	bl	decrement_vpc_count
	subs	r8, r8, #1
	bne	.L980
	b	.L977
.L988:
	.align	2
.L987:
	.word	.LANCHOR0
	.fnend
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	1
	.global	FtlMakeBbt
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, #0
	ldr	r9, .L1008+4
	bl	FtlBbtMemInit
	sub	r5, r9, #380
	sub	r10, r9, #18
	mov	r4, r5
	bl	FtlLoadFactoryBbt
.L990:
	ldrh	r3, [r5, #254]
	cmp	r8, r3
	bcc	.L996
	movs	r5, #0
.L997:
	ldrh	r3, [r4, #314]
	uxth	r0, r5
	adds	r5, r5, #1
	cmp	r3, r0
	bhi	.L998
	ldrh	r5, [r4, #364]
	movw	r6, #65535
	subs	r5, r5, #1
	uxth	r5, r5
.L999:
	ldrh	r3, [r4, #364]
	subs	r3, r3, #48
	cmp	r5, r3
	ble	.L1003
	mov	r0, r5
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L1000
	mov	r0, r5
	bl	FlashTestBlk
	cmp	r0, #0
	beq	.L1001
	mov	r0, r5
	bl	FtlBbmMapBadBlock
.L1000:
	subs	r5, r5, #1
	uxth	r5, r5
	b	.L999
.L996:
	ldrh	r3, [r10, #2]!
	movw	r2, #65535
	ldr	r0, [r5, #3300]
	ldr	fp, [r5, #3332]
	cmp	r3, r2
	str	r0, [r5, #3460]
	str	fp, [r5, #3464]
	beq	.L991
	ldrh	r7, [r5, #298]
	movs	r2, #1
	mov	r1, r2
	ldr	r0, .L1008
	mla	r7, r8, r7, r3
	lsls	r3, r7, #10
	str	r3, [r5, #3456]
	bl	FlashReadPages
	ldrh	r2, [r5, #298]
	ldr	r1, [r5, #3460]
	ldr	r0, [r9]
	adds	r2, r2, #7
	asrs	r2, r2, #3
	bl	ftl_memcpy
.L992:
	uxth	r0, r7
	add	r8, r8, #1
	add	r9, r9, #4
	bl	FtlBbmMapBadBlock
	b	.L990
.L991:
	mov	r1, r8
	bl	FlashGetBadBlockList
	ldr	r1, [r9]
	ldr	r0, [r5, #3460]
	bl	FtlBbt2Bitmap
	ldrh	r6, [r5, #298]
.L994:
	subs	r6, r6, #1
	uxth	r6, r6
.L993:
	ldrh	r0, [r4, #298]
	smlabb	r0, r0, r8, r6
	uxth	r0, r0
	bl	FtlBbmIsBadBlock
	cmp	r0, #1
	beq	.L994
	movs	r1, #16
	ldr	r0, [r4, #3332]
	strh	r6, [r10]	@ movhi
	bl	__memzero
	mov	r1, #4096
	ldr	r0, [r4, #3300]
	bl	__memzero
	movw	r3, #61664
	strh	r3, [fp]	@ movhi
	movs	r3, #0
	str	r3, [fp, #4]
	ldrh	r3, [r10]
	ldrh	r7, [r4, #298]
	ldrh	r2, [r4, #3388]
	strh	r3, [fp, #2]	@ movhi
	ldrh	r3, [r10]
	ldr	r1, [r9]
	lsls	r2, r2, #2
	ldr	r0, [r4, #3460]
	mla	r7, r8, r7, r3
	lsls	r3, r7, #10
	str	r3, [r4, #3456]
	bl	ftl_memcpy
	movs	r2, #1
	ldr	r0, .L1008
	mov	r1, r2
	bl	FlashEraseBlocks
	movs	r3, #1
	ldr	r0, .L1008
	mov	r2, r3
	mov	r1, r3
	bl	FlashProgPages
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	bne	.L992
	uxth	r0, r7
	bl	FtlBbmMapBadBlock
	b	.L993
.L998:
	bl	FtlBbmMapBadBlock
	b	.L997
.L1001:
	ldrh	r3, [r4, #352]
	cmp	r3, r6
	bne	.L1002
	strh	r5, [r4, #352]	@ movhi
	b	.L1000
.L1002:
	strh	r5, [r4, #356]	@ movhi
.L1003:
	ldrh	r3, [r4, #352]
	movs	r5, #0
	ldr	r0, [r4, #3268]
	movs	r1, #1
	str	r5, [r4, #360]
	movs	r2, #2
	strh	r5, [r4, #354]	@ movhi
	lsls	r3, r3, #10
	str	r3, [r0, #4]
	ldrh	r3, [r4, #356]
	lsls	r3, r3, #10
	str	r3, [r0, #24]
	bl	FlashEraseBlocks
	ldrh	r0, [r4, #352]
	bl	FtlBbmMapBadBlock
	ldrh	r0, [r4, #356]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldr	r3, [r4, #360]
	ldrh	r2, [r4, #356]
	strh	r5, [r4, #354]	@ movhi
	adds	r3, r3, #1
	str	r3, [r4, #360]
	ldrh	r3, [r4, #352]
	strh	r2, [r4, #352]	@ movhi
	strh	r3, [r4, #356]	@ movhi
	bl	FtlBbmTblFlush
	mov	r0, r5
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1009:
	.align	2
.L1008:
	.word	.LANCHOR0+3452
	.word	.LANCHOR0+380
	.fnend
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	1
	.global	ftl_memcmp
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
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
	.global	js_hash
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	js_hash, %function
js_hash:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1014
	add	r1, r1, r0
	push	{r4, lr}
	.save {r4, lr}
.L1012:
	cmp	r0, r1
	bne	.L1013
	mov	r0, r3
	pop	{r4, pc}
.L1013:
	lsrs	r2, r3, #2
	ldrb	r4, [r0], #1	@ zero_extendqisi2
	add	r2, r2, r3, lsl #5
	add	r2, r2, r4
	eors	r3, r3, r2
	b	.L1012
.L1015:
	.align	2
.L1014:
	.word	1204201446
	.fnend
	.size	js_hash, .-js_hash
	.align	1
	.global	Ftl_write_map_blk_to_last_page
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r2, #65535
	ldrh	r3, [r0]
	mov	r4, r0
	ldr	r5, [r0, #12]
	cmp	r3, r2
	bne	.L1017
	ldrh	r3, [r0, #8]
	cbz	r3, .L1018
	movw	r2, #641
	ldr	r1, .L1026
	ldr	r0, .L1026+4
	bl	sftl_printk
.L1018:
	ldrh	r3, [r4, #8]
	adds	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	movs	r3, #0
	strh	r0, [r5]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	strh	r3, [r4]	@ movhi
	ldr	r3, [r4, #28]
	adds	r3, r3, #1
	str	r3, [r4, #28]
.L1019:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, pc}
.L1017:
	ldrh	r7, [r5, r3, lsl #1]
	movs	r1, #255
	ldrh	r3, [r0, #2]
	ldr	r5, .L1026+8
	ldr	r8, [r0, #24]
	orr	r3, r3, r7, lsl #10
	ldr	r6, [r5, #3332]
	str	r3, [r5, #3456]
	ldr	r3, [r5, #3300]
	str	r6, [r5, #3464]
	str	r3, [r5, #3460]
	ldr	r3, [r0, #28]
	str	r3, [r6, #4]
	movw	r3, #64245
	strh	r3, [r6, #8]	@ movhi
	ldrh	r3, [r0, #4]
	strh	r7, [r6, #2]	@ movhi
	strh	r3, [r6]	@ movhi
	ldrh	r2, [r5, #304]
	ldr	r0, [r5, #3300]
	lsls	r2, r2, #3
	bl	ftl_memset
	movs	r2, #0
	mov	r3, r2
.L1020:
	ldrh	r0, [r4, #6]
	uxth	r1, r2
	cmp	r0, r1
	bhi	.L1022
	ldrh	r1, [r5, #310]
	ldr	r0, [r5, #3460]
	bl	js_hash
	movs	r2, #1
	str	r0, [r6, #12]
	movs	r3, #0
	mov	r1, r2
	ldr	r0, .L1026+12
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	mov	r0, r4
	adds	r3, r3, #1
	strh	r3, [r4, #2]	@ movhi
	bl	ftl_map_blk_gc
	b	.L1019
.L1022:
	uxth	r0, r2
	ldr	r1, [r8, r0, lsl #2]
	cmp	r7, r1, lsr #10
	bne	.L1021
	ldr	r1, [r5, #3300]
	adds	r3, r3, #1
	uxth	r3, r3
	str	r0, [r1, r3, lsl #3]
	ldr	r1, [r5, #3300]
	ldr	r0, [r8, r0, lsl #2]
	add	r1, r1, r3, lsl #3
	str	r0, [r1, #4]
.L1021:
	adds	r2, r2, #1
	b	.L1020
.L1027:
	.align	2
.L1026:
	.word	.LANCHOR1+332
	.word	.LC8
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.fnend
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	1
	.global	FtlMapWritePage
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r0
	ldr	r7, .L1052
	mov	r8, r1
	movs	r6, #0
	ldr	r9, .L1052+20
	mov	r5, r7
	str	r2, [sp, #4]
.L1029:
	ldr	r3, [r7, #2572]
	adds	r3, r3, #1
	str	r3, [r7, #2572]
	ldrh	r3, [r7, #304]
	ldrh	r2, [r4, #2]
	subs	r3, r3, #1
	cmp	r2, r3
	bge	.L1030
	ldrh	r2, [r4]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1031
.L1030:
	mov	r0, r4
	bl	Ftl_write_map_blk_to_last_page
.L1031:
	ldrh	r2, [r4]
	ldr	r3, [r4, #12]
	ldrh	r3, [r3, r2, lsl #1]
	cbnz	r3, .L1032
	mov	r2, #700
	mov	r1, r9
	ldr	r0, .L1052+4
	bl	sftl_printk
.L1032:
	ldrh	r2, [r4]
	ldrh	r3, [r4, #10]
	cmp	r2, r3
	bcc	.L1033
	movw	r2, #701
	mov	r1, r9
	ldr	r0, .L1052+4
	bl	sftl_printk
.L1033:
	ldrh	r2, [r4]
	movs	r1, #16
	ldr	r3, [r4, #12]
	ldr	r0, [r5, #3332]
	ldrh	r10, [r3, r2, lsl #1]
	ldrh	r3, [r4, #2]
	str	r0, [r5, #3464]
	orr	r3, r3, r10, lsl #10
	str	r3, [r5, #3456]
	ldr	r3, [sp, #4]
	str	r3, [r5, #3460]
	bl	__memzero
	ldr	r3, [r4, #28]
	ldr	fp, [r5, #3464]
	str	r3, [fp, #4]
	strh	r8, [fp, #8]	@ movhi
	ldrh	r3, [r4, #4]
	strh	r10, [fp, #2]	@ movhi
	strh	r3, [fp]	@ movhi
	ldrh	r1, [r5, #310]
	ldr	r0, [r5, #3460]
	bl	js_hash
	movs	r3, #1
	str	r0, [fp, #12]
	mov	r2, r3
	mov	r1, r3
	ldr	r0, .L1052+8
	bl	FlashProgPages
	ldrh	r3, [r4, #2]
	adds	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r4, #2]	@ movhi
	ldr	r2, [r5, #3452]
	adds	r1, r2, #1
	bne	.L1034
	ldr	r1, [r5, #3456]
	adds	r6, r6, #1
	ldr	r0, .L1052+12
	uxth	r6, r6
	bl	sftl_printk
	ldrh	r3, [r4, #2]
	cmp	r3, #2
	ittt	ls
	ldrhls	r3, [r5, #304]
	addls	r3, r3, #-1
	strhls	r3, [r4, #2]	@ movhi
	cmp	r6, #3
	bls	.L1036
	mov	r2, r6
	ldr	r1, [r5, #3456]
	ldr	r0, .L1052+16
	bl	sftl_printk
.L1037:
	b	.L1037
.L1036:
	ldr	r3, [r4, #32]
	cmp	r3, #0
	beq	.L1029
.L1051:
	b	.L1051
.L1034:
	cmp	r3, #1
	beq	.L1040
	cmp	r2, #256
	beq	.L1040
	ldr	r0, [r4, #36]
	cbz	r0, .L1041
.L1040:
	movs	r3, #0
	str	r3, [r4, #36]
	b	.L1029
.L1041:
	ldr	r2, [r5, #3456]
	ldr	r3, [r4, #24]
	str	r2, [r3, r8, lsl #2]
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1053:
	.align	2
.L1052:
	.word	.LANCHOR0
	.word	.LC8
	.word	.LANCHOR0+3452
	.word	.LC104
	.word	.LC105
	.word	.LANCHOR1+363
	.fnend
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	1
	.global	load_l2p_region
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	load_l2p_region, %function
load_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r6, r0
	ldr	r4, .L1061
	mov	r8, r1
	ldrh	r3, [r4, #336]
	cmp	r3, r0
	bcs	.L1055
	movw	r2, #485
	ldr	r1, .L1061+4
	ldr	r0, .L1061+8
	bl	sftl_printk
.L1055:
	ldr	r3, [r4, #3376]
	movs	r5, #12
	ldr	r7, [r3, r6, lsl #2]
	cbnz	r7, .L1056
	mul	r5, r5, r8
	ldr	r3, [r4, #2540]
	ldrh	r2, [r4, #310]
	movs	r1, #255
	add	r3, r3, r5
	ldr	r0, [r3, #8]
	bl	ftl_memset
	ldr	r3, [r4, #2540]
	strh	r6, [r3, r5]	@ movhi
	ldr	r3, [r4, #2540]
	add	r5, r5, r3
	str	r7, [r5, #4]
.L1057:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, pc}
.L1056:
	mul	r5, r5, r8
	ldr	r3, [r4, #2540]
	movs	r2, #1
	ldr	r0, .L1061+12
	mov	r1, r2
	str	r7, [r4, #3456]
	add	r3, r3, r5
	ldr	r3, [r3, #8]
	str	r3, [r4, #3460]
	ldr	r3, [r4, #3332]
	str	r3, [r4, #3464]
	bl	FlashReadPages
	ldr	r8, [r4, #3464]
	ldrh	r3, [r8, #8]
	cmp	r3, r6
	beq	.L1058
	mov	r2, r7
	mov	r1, r6
	ldr	r0, .L1061+16
	bl	sftl_printk
	movs	r3, #4
	ldr	r1, [r4, #3464]
	mov	r2, r3
	ldr	r0, .L1061+20
	bl	rknand_print_hex
	ldrh	r3, [r4, #336]
	movs	r2, #4
	ldr	r1, [r4, #3376]
	ldr	r0, .L1061+24
	bl	rknand_print_hex
.L1059:
	ldrh	r3, [r8, #8]
	cmp	r3, r6
	beq	.L1060
	mov	r2, #508
	ldr	r1, .L1061+4
	ldr	r0, .L1061+8
	bl	sftl_printk
.L1060:
	ldr	r3, [r4, #2540]
	movs	r1, #0
	adds	r2, r3, r5
	str	r1, [r2, #4]
	strh	r6, [r3, r5]	@ movhi
	b	.L1057
.L1058:
	ldr	r3, [r4, #3452]
	cmp	r3, #256
	bne	.L1059
	mov	r2, r7
	mov	r1, r6
	ldr	r0, .L1061+28
	bl	sftl_printk
	ldr	r3, [r4, #2540]
	mov	r1, r6
	ldr	r0, .L1061+32
	add	r3, r3, r5
	ldr	r2, [r3, #8]
	bl	FtlMapWritePage
	b	.L1059
.L1062:
	.align	2
.L1061:
	.word	.LANCHOR0
	.word	.LANCHOR1+379
	.word	.LC8
	.word	.LANCHOR0+3452
	.word	.LC106
	.word	.LC94
	.word	.LC107
	.word	.LC108
	.word	.LANCHOR0+3392
	.fnend
	.size	load_l2p_region, .-load_l2p_region
	.align	1
	.global	ftl_map_blk_gc
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r0
	ldr	r6, [r0, #12]
	ldr	r10, [r0, #24]
	bl	ftl_free_no_use_map_blk
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #8]
	ldr	r5, .L1079
	subs	r3, r3, #5
	cmp	r2, r3
	blt	.L1064
	uxth	r0, r0
	ldrh	r9, [r6, r0, lsl #1]
	cmp	r9, #0
	beq	.L1064
	ldr	r3, [r4, #32]
	cbnz	r3, .L1064
	movs	r2, #1
	str	r2, [r4, #32]
	strh	r3, [r6, r0, lsl #1]	@ movhi
	ldrh	r3, [r4, #8]
	ldrh	r2, [r4, #2]
	subs	r3, r3, #1
	strh	r3, [r4, #8]	@ movhi
	ldrh	r3, [r5, #304]
	cmp	r2, r3
	bcc	.L1065
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1065:
	movs	r6, #0
.L1066:
	ldrh	r2, [r4, #6]
	uxth	fp, r6
	cmp	r2, fp
	bhi	.L1073
	movs	r1, #1
	mov	r0, r9
	bl	FtlFreeSysBlkQueueIn
	movs	r3, #0
	str	r3, [r4, #32]
.L1064:
	ldrh	r2, [r4, #2]
	ldrh	r3, [r5, #304]
	cmp	r2, r3
	bcc	.L1074
	mov	r0, r4
	bl	ftl_map_blk_alloc_new_blk
.L1074:
	movs	r0, #0
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1073:
	uxth	r7, r6
	ldr	r2, [r10, r7, lsl #2]
	add	r3, r10, r7, lsl #2
	str	r3, [sp, #4]
	cmp	r9, r2, lsr #10
	bne	.L1067
	ldr	r2, [r5, #3304]
	ldr	r8, [r5, #3332]
	ldr	r0, .L1079+4
	str	r2, [r5, #3460]
	str	r8, [r5, #3464]
	ldr	r2, [r10, r7, lsl #2]
	str	r2, [r5, #3456]
	movs	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldrh	r2, [r8, #8]
	cmp	r2, fp
	beq	.L1068
	movw	r2, #611
	ldr	r1, .L1079+8
	ldr	r0, .L1079+12
	bl	sftl_printk
.L1068:
	ldr	r2, [r5, #3452]
	adds	r2, r2, #1
	bne	.L1069
.L1071:
	ldr	r2, [sp, #4]
	movs	r3, #0
	str	r3, [r2]
.L1070:
	b	.L1070
.L1069:
	ldrh	r2, [r8, #8]
	cmp	r2, fp
	bne	.L1071
	ldrh	r2, [r8]
	ldrh	r3, [r4, #4]
	cmp	r2, r3
	bne	.L1071
	ldr	r2, [r5, #3460]
	mov	r1, r7
	mov	r0, r4
	bl	FtlMapWritePage
.L1067:
	adds	r6, r6, #1
	b	.L1066
.L1080:
	.align	2
.L1079:
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.word	.LANCHOR1+395
	.word	.LC8
	.fnend
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	1
	.global	flush_l2p_region
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	flush_l2p_region, %function
flush_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r4, #12
	ldr	r5, .L1082
	muls	r4, r0, r4
	ldr	r3, [r5, #2540]
	add	r0, r5, #3392
	adds	r2, r3, r4
	ldrh	r1, [r3, r4]
	ldr	r2, [r2, #8]
	bl	FtlMapWritePage
	ldr	r3, [r5, #2540]
	movs	r0, #0
	add	r4, r4, r3
	ldr	r3, [r4, #4]
	bic	r3, r3, #-2147483648
	str	r3, [r4, #4]
	pop	{r3, r4, r5, pc}
.L1083:
	.align	2
.L1082:
	.word	.LANCHOR0
	.fnend
	.size	flush_l2p_region, .-flush_l2p_region
	.align	1
	.global	log2phys
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	log2phys, %function
log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r0
	ldr	r4, .L1097
	mov	r5, r1
	mov	r7, r2
	ldr	r3, [r4, #2552]
	ldrh	r10, [r4, #308]
	cmp	r0, r3
	bcc	.L1085
	movw	r2, #813
	ldr	r1, .L1097+4
	ldr	r0, .L1097+8
	bl	sftl_printk
.L1085:
	ldr	r3, [r4, #2552]
	cmp	r9, r3
	bcs	.L1086
	add	r10, r10, #7
	ldrh	r2, [r4, #338]
	lsr	r6, r9, r10
	ldr	r1, [r4, #2540]
	movs	r3, #0
	uxth	r6, r6
	mov	fp, #12
.L1087:
	uxth	r8, r3
	cmp	r8, r2
	bcc	.L1092
	bl	select_l2p_ram_region
	mul	fp, fp, r0
	ldr	r3, [r4, #2540]
	mov	r8, r0
	ldrh	r1, [r3, fp]
	add	r2, r3, fp
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1093
	ldr	r3, [r2, #4]
	cmp	r3, #0
	bge	.L1093
	bl	flush_l2p_region
.L1093:
	mov	r1, r8
	mov	r0, r6
	bl	load_l2p_region
	b	.L1089
.L1086:
	mov	r0, #-1
	cbnz	r7, .L1084
	str	r0, [r5]
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1092:
	adds	r3, r3, #1
	mla	r0, fp, r3, r1
	ldrh	r0, [r0, #-12]
	cmp	r0, r6
	bne	.L1087
.L1089:
	movs	r0, #1
	movs	r3, #12
	lsl	r0, r0, r10
	subs	r0, r0, #1
	and	r0, r0, r9
	uxth	r0, r0
	cbnz	r7, .L1090
	ldr	r2, [r4, #2540]
	mla	r3, r3, r8, r2
	ldr	r3, [r3, #8]
	ldr	r3, [r3, r0, lsl #2]
	str	r3, [r5]
.L1091:
	ldr	r2, [r4, #2540]
	movs	r3, #12
	mla	r8, r3, r8, r2
	ldr	r3, [r8, #4]
	adds	r2, r3, #1
	beq	.L1095
	adds	r3, r3, #1
	str	r3, [r8, #4]
.L1095:
	movs	r0, #0
.L1084:
	pop	{r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1090:
	mul	r3, r3, r8
	ldr	r2, [r4, #2540]
	ldr	r1, [r5]
	add	r2, r2, r3
	ldr	r2, [r2, #8]
	str	r1, [r2, r0, lsl #2]
	ldr	r2, [r4, #2540]
	strh	r6, [r4, #2544]	@ movhi
	add	r3, r3, r2
	ldr	r2, [r3, #4]
	orr	r2, r2, #-2147483648
	str	r2, [r3, #4]
	b	.L1091
.L1098:
	.align	2
.L1097:
	.word	.LANCHOR0
	.word	.LANCHOR1+410
	.word	.LC8
	.fnend
	.size	log2phys, .-log2phys
	.align	1
	.global	FtlWriteDump_data
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlWriteDump_data, %function
FtlWriteDump_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #24
	sub	sp, sp, #24
	ldr	r4, .L1117
	ldrh	r2, [r4, #28]
	cmp	r2, #0
	beq	.L1100
	ldrb	r3, [r4, #32]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1100
	ldrb	r1, [r4, #31]	@ zero_extendqisi2
	ldrh	r3, [r4, #302]
	muls	r3, r1, r3
	cmp	r2, r3
	beq	.L1100
	ldrb	r7, [r4, #34]	@ zero_extendqisi2
	cbnz	r7, .L1099
	ldr	r6, [r4, #2552]
	mov	r2, r7
	mov	r1, sp
	ldrh	r8, [r4, #232]
	subs	r6, r6, #1
	mov	r0, r6
	bl	log2phys
	ldr	r3, [sp]
	ldr	r5, [r4, #3332]
	ldr	r0, [r4, #3300]
	str	r3, [sp, #8]
	adds	r3, r3, #1
	str	r6, [sp, #20]
	str	r0, [sp, #12]
	str	r5, [sp, #16]
	str	r7, [r5, #4]
	beq	.L1102
	mov	r2, r7
	movs	r1, #1
	add	r0, sp, #4
	bl	FlashReadPages
.L1103:
	ldr	r9, .L1117+4
	movs	r7, #0
	lsl	r8, r8, #2
	mov	r10, r7
	movw	r3, #61589
	strh	r3, [r5]	@ movhi
.L1104:
	cmp	r8, r7
	bne	.L1108
.L1105:
	movs	r3, #1
.L1116:
	strb	r3, [r4, #34]
.L1099:
	add	sp, sp, #24
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L1102:
	ldrh	r2, [r4, #310]
	movs	r1, #255
	bl	ftl_memset
	b	.L1103
.L1108:
	ldrh	r3, [r4, #28]
	cmp	r3, #0
	beq	.L1105
	ldr	r3, [sp, #8]
	mov	r0, r9
	str	r6, [r5, #8]
	adds	r7, r7, #1
	str	r3, [r5, #12]
	ldrh	r3, [r4, #24]
	strh	r3, [r5, #2]	@ movhi
	bl	get_new_active_ppa
	ldr	r3, [r4, #2592]
	movs	r1, #1
	str	r0, [sp, #8]
	add	r0, sp, #4
	str	r3, [r5, #4]
	adds	r3, r3, #1
	adds	r2, r3, #1
	it	eq
	moveq	r3, r10
	str	r3, [r4, #2592]
	movs	r3, #0
	mov	r2, r3
	bl	FlashProgPages
	ldrh	r0, [r4, #24]
	bl	decrement_vpc_count
	b	.L1104
.L1100:
	movs	r3, #0
	b	.L1116
.L1118:
	.align	2
.L1117:
	.word	.LANCHOR0
	.word	.LANCHOR0+24
	.fnend
	.size	FtlWriteDump_data, .-FtlWriteDump_data
	.align	1
	.global	FtlReUsePrevPpa
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r6, r0
	ldr	r5, .L1129
	ubfx	r0, r1, #10, #16
	str	r1, [sp, #4]
	bl	P2V_block_in_plane
	ldr	r2, [r5, #72]
	mov	r7, r0
	ldrh	r3, [r2, r0, lsl #1]
	cbnz	r3, .L1120
	ldr	r4, [r5, #2532]
	cbz	r4, .L1121
	ldr	r1, [r5, #2516]
	mov	ip, #6
	ldr	r2, .L1129+4
	movw	lr, #65535
	ldrh	r0, [r5, #224]
	subs	r4, r4, r1
	asrs	r4, r4, #1
	muls	r4, r2, r4
	uxth	r4, r4
.L1122:
	uxth	r2, r3
	cmp	r0, r2
	bls	.L1121
	cmp	r4, r7
	bne	.L1123
	mov	r1, r4
	ldr	r0, .L1129+8
	bl	List_remove_node
	ldrh	r3, [r5, #224]
	cbnz	r3, .L1124
	movw	r2, #1735
	ldr	r1, .L1129+12
	ldr	r0, .L1129+16
	bl	sftl_printk
.L1124:
	ldrh	r3, [r5, #224]
	mov	r0, r4
	subs	r3, r3, #1
	strh	r3, [r5, #224]	@ movhi
	bl	INSERT_DATA_LIST
	ldr	r2, [r5, #72]
	ldrh	r3, [r2, r7, lsl #1]
.L1120:
	adds	r3, r3, #1
	strh	r3, [r2, r7, lsl #1]	@ movhi
	b	.L1121
.L1123:
	mul	r4, ip, r4
	adds	r3, r3, #1
	ldrh	r4, [r1, r4]
	cmp	r4, lr
	bne	.L1122
.L1121:
	movs	r2, #1
	add	r1, sp, #4
	mov	r0, r6
	bl	log2phys
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L1130:
	.align	2
.L1129:
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LANCHOR0+2532
	.word	.LANCHOR1+419
	.word	.LC8
	.fnend
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	1
	.global	FtlRecoverySuperblock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r2, #65535
	ldrh	r3, [r0]
	.pad #52
	sub	sp, sp, #52
	mov	r10, r0
	cmp	r3, r2
	beq	.L1259
	ldrh	r3, [r0, #2]
	ldr	r4, .L1267
	str	r3, [sp]
	ldrb	r3, [r0, #6]	@ zero_extendqisi2
	ldr	r1, [sp]
	str	r3, [sp, #20]
	ldrh	r3, [r4, #302]
	cmp	r3, r1
	mov	r3, #0
	bne	.L1134
	strh	r3, [r0, #4]	@ movhi
.L1265:
	strb	r3, [r10, #6]
.L1259:
	movs	r0, #0
	add	sp, sp, #52
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1134:
	ldrh	r0, [r0, #16]
.L1135:
	cmp	r0, r2
	uxth	r6, r3
	add	r3, r3, #1
	beq	.L1136
	movs	r1, #1
	bl	FtlGetLastWrittenPage
	adds	r7, r0, #1
	mov	r5, r0
	beq	.L1137
	ldr	r3, [r4, #3176]
	movs	r2, #0
	ldrh	fp, [r4, #232]
	movw	r9, #65535
	ldr	r0, [r4, #3288]
	mov	lr, #20
	str	r3, [sp, #4]
	ldrh	r3, [r4, #310]
	ldr	r8, [r4, #3180]
	ldrh	r7, [r4, #312]
	mov	r4, r2
	str	r3, [sp, #8]
	add	r3, r10, #16
	mov	ip, r3
	str	r3, [sp, #16]
.L1138:
	uxth	r3, r2
	cmp	fp, r3
	bhi	.L1144
	ldr	r6, .L1267
	movs	r2, #0
	mov	r1, r4
	movw	r9, #65535
	bl	FlashReadPages
	ldr	r3, [r6, #2592]
	uxth	r2, r5
	ldr	r8, [r6, #3288]
	str	r2, [sp, #8]
	add	fp, r3, #-1
	movs	r3, #0
	mov	r7, r8
.L1145:
	uxth	r2, r3
	cmp	r4, r2
	bhi	.L1150
	bne	.L1148
	adds	r3, r5, #1
	uxth	r3, r3
	str	r3, [sp, #8]
.L1260:
	ldr	r0, [r8, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_plane
	ldrh	r3, [r6, #302]
	ldr	r2, [sp, #8]
	str	r0, [sp, #4]
	cmp	r3, r2
	bne	.L1152
	ldrh	r3, [sp, #8]
	strh	r3, [r10, #2]	@ movhi
	movs	r3, #0
	strb	r3, [r10, #6]
	strh	r3, [r10, #4]	@ movhi
.L1152:
	ldr	r3, [sp, #8]
	ldr	r2, [sp]
	cmp	r3, r2
	bne	.L1153
	ldr	r3, [sp, #4]
	ldr	r2, [sp, #20]
	cmp	r3, r2
	bne	.L1153
	ldr	r1, [sp, #8]
	mov	r2, r3
.L1266:
	mov	r0, r10
	bl	ftl_sb_update_avl_pages
	b	.L1259
.L1136:
	uxth	r1, r3
	adds	r1, r1, #8
	ldrh	r0, [r10, r1, lsl #1]
	b	.L1135
.L1137:
	ldr	r3, [sp]
	cbz	r3, .L1139
	movw	r2, #1802
	ldr	r1, .L1267+4
	ldr	r0, .L1267+8
	bl	sftl_printk
.L1139:
	ldr	r3, [sp, #20]
	cbz	r3, .L1140
	cmp	r6, r3
	beq	.L1140
	movw	r2, #1803
	ldr	r1, .L1267+4
	ldr	r0, .L1267+8
	bl	sftl_printk
.L1140:
	movs	r3, #0
	strh	r3, [r10, #2]	@ movhi
	b	.L1265
.L1144:
	ldrh	r3, [ip], #2
	cmp	r3, r9
	beq	.L1141
	mla	r1, lr, r4, r0
	orr	r3, r5, r3, lsl #10
	str	r3, [r1, #4]
	ldr	r3, [sp, #8]
	muls	r3, r4, r3
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	mov	r6, r3
	ldr	r3, [sp, #4]
	add	r6, r6, r3
	mov	r3, r7
	muls	r3, r4, r3
	add	r4, r4, #1
	it	mi
	addmi	r3, r3, #3
	uxth	r4, r4
	bic	r3, r3, #3
	str	r6, [r1, #8]
	add	r3, r3, r8
	str	r3, [r1, #12]
.L1141:
	adds	r2, r2, #1
	b	.L1138
.L1150:
	ldr	r2, [r7]
	cbnz	r2, .L1146
	ldr	r2, [r7, #12]
	str	r2, [sp, #4]
	ldr	r2, [r2, #4]
	adds	r0, r2, #1
	beq	.L1147
	ldr	r1, [r6, #2592]
	mov	r0, r2
	bl	ftl_cmp_data_ver
	cbz	r0, .L1147
	adds	r2, r2, #1
	str	r2, [r6, #2592]
.L1147:
	ldr	r2, [sp, #4]
	ldr	r2, [r2]
	adds	r2, r2, #1
	bne	.L1149
.L1148:
	uxth	r2, r5
	uxth	r3, r3
	str	r2, [sp, #8]
	movs	r2, #20
	mla	r8, r2, r3, r8
	b	.L1260
.L1146:
	ldr	r9, [sp, #8]
.L1149:
	adds	r3, r3, #1
	adds	r7, r7, #20
	b	.L1145
.L1153:
	movw	r3, #65535
	cmp	r9, r3
	bne	.L1154
	ldrb	r3, [r10, #8]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1155
.L1154:
	ldr	r3, [r6, #3448]
	uxth	r7, r5
	uxth	r5, r5
	movw	r8, #65535
	adds	r3, r3, #1
	ldr	r3, [sp]
	it	eq
	streq	fp, [r6, #3448]
	ldr	r6, .L1267
	adds	r3, r3, #7
	cmp	r5, r3
	itet	gt
	subgt	r4, r7, #7
	ldrle	r4, [sp]
	uxthgt	r4, r4
.L1158:
	cmp	r4, r7
	bhi	.L1165
	movs	r3, #0
	ldrh	lr, [r6, #232]
	ldr	r0, [r6, #3288]
	mov	r5, r3
	ldr	r1, [sp, #16]
	mov	ip, #20
	b	.L1166
.L1160:
	ldrh	r2, [r1], #2
	cmp	r2, r8
	beq	.L1159
	mla	r9, ip, r5, r0
	adds	r5, r5, #1
	orr	r2, r4, r2, lsl #10
	uxth	r5, r5
	str	r2, [r9, #4]
.L1159:
	adds	r3, r3, #1
.L1166:
	uxth	r2, r3
	cmp	lr, r2
	bhi	.L1160
	movs	r2, #0
	mov	r1, r5
	bl	FlashReadPages
	ldr	r3, [r6, #3288]
	movs	r2, #20
	mla	r5, r2, r5, r3
.L1161:
	cmp	r5, r3
	bne	.L1164
	adds	r4, r4, #1
	uxth	r4, r4
	b	.L1158
.L1164:
	ldr	r2, [r3]
	cbnz	r2, .L1155
	ldr	r2, [r3, #12]
	ldrh	r1, [r2]
	cmp	r1, r8
	beq	.L1163
	ldr	r2, [r2, #4]
	adds	r1, r2, #1
	it	ne
	strne	r2, [r6, #3448]
.L1163:
	adds	r3, r3, #20
	b	.L1161
.L1165:
	mov	r3, #-1
	str	r3, [r6, #3448]
.L1155:
	ldr	r8, .L1267
	movs	r3, #1
	ldr	r5, [sp]
	mov	r4, r8
	strh	r3, [r8, #3472]	@ movhi
.L1167:
	ldrh	r6, [r4, #232]
	movw	r9, #65535
	ldr	r0, [r4, #3288]
	mov	ip, #20
	ldr	r1, [sp, #16]
	movs	r3, #0
	str	r3, [sp, #12]
.L1168:
	uxth	r2, r3
	cmp	r6, r2
	bhi	.L1170
	movs	r2, #0
	ldr	r1, [sp, #12]
	bl	FlashReadPages
	movs	r3, #0
.L1264:
	str	r3, [sp, #24]
	ldr	r2, [sp, #12]
	ldrh	r3, [sp, #24]
	cmp	r2, r3
	bhi	.L1200
	adds	r5, r5, #1
	ldrh	r3, [r4, #302]
	uxth	r5, r5
	cmp	r3, r5
	bne	.L1167
	ldrh	r2, [r4, #232]
	movw	r0, #65535
	movs	r3, #0
	strh	r5, [r10, #2]	@ movhi
	strh	r3, [r10, #4]	@ movhi
.L1201:
	uxth	r1, r3
	cmp	r1, r2
	bcs	.L1259
	ldr	r1, [sp, #16]
	ldrh	r4, [r1], #2
	cmp	r4, r0
	str	r1, [sp, #16]
	add	r1, r3, #1
	bne	.L1265
	mov	r3, r1
	b	.L1201
.L1170:
	ldrh	r2, [r1], #2
	cmp	r2, r9
	beq	.L1169
	ldr	r7, [sp, #12]
	orr	r2, r5, r2, lsl #10
	mla	lr, ip, r7, r0
	str	r2, [lr, #4]
	mov	r2, r7
	adds	r2, r2, #1
	uxth	r2, r2
	str	r2, [sp, #12]
.L1169:
	adds	r3, r3, #1
	b	.L1168
.L1268:
	.align	2
.L1267:
	.word	.LANCHOR0
	.word	.LANCHOR1+435
	.word	.LC8
.L1200:
	ldr	r3, [sp, #24]
	movs	r7, #20
	muls	r7, r3, r7
	ldr	r3, [r4, #3288]
	str	r3, [sp, #28]
	add	r9, r3, r7
	ldr	r6, [r9, #4]
	ubfx	r0, r6, #10, #16
	str	r6, [sp, #44]
	bl	P2V_plane
	ldr	r3, [sp]
	cmp	r5, r3
	bcc	.L1172
	ldr	r3, [sp, #28]
	bne	.L1173
	ldr	r2, [sp, #20]
	cmp	r2, r0
	bhi	.L1172
.L1173:
	ldr	r2, [sp, #8]
	cmp	r5, r2
	bne	.L1174
	ldr	r2, [sp, #4]
	cmp	r2, r0
	beq	.L1175
.L1174:
	ldr	r3, [r3, r7]
	adds	r3, r3, #1
	beq	.L1176
	ldr	r9, [r9, #12]
	movw	r3, #61589
	ldrh	r2, [r9]
	cmp	r2, r3
	beq	.L1177
	ldrh	r0, [r10]
.L1261:
	bl	decrement_vpc_count
	b	.L1172
.L1177:
	ldr	fp, [r9, #4]
	cmp	fp, #-1
	beq	.L1178
	ldr	r1, [r4, #2592]
	mov	r0, fp
	bl	ftl_cmp_data_ver
	cbz	r0, .L1178
	add	r3, fp, #1
	str	r3, [r4, #2592]
.L1178:
	ldrh	r2, [r9]
	movw	r3, #61589
	cmp	r2, r3
	beq	.L1179
	movw	r2, #1954
	ldr	r1, .L1269
	ldr	r0, .L1269+4
	bl	sftl_printk
.L1179:
	ldr	r6, [r9, #8]
	add	r1, sp, #40
	ldr	r3, [r9, #12]
	movs	r2, #0
	mov	r0, r6
	str	r3, [sp, #36]
	bl	log2phys
	ldr	r1, [r4, #3448]
	adds	r0, r1, #1
	beq	.L1180
	mov	r0, fp
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1180
	ldr	r3, [sp, #36]
	adds	r2, r3, #1
	beq	.L1181
	ldr	r0, [r4, #3288]
	movs	r2, #0
	movs	r1, #1
	add	r0, r0, r7
	str	r3, [r0, #4]
	ldr	r9, [r0, #12]
	bl	FlashReadPages
	ldr	r2, [r4, #3288]
	ldr	r1, [r2, r7]
	adds	r3, r2, r7
	adds	r1, r1, #1
	bne	.L1182
.L1183:
	mov	r3, #-1
	str	r3, [sp, #36]
.L1190:
	ldr	r0, [sp, #36]
	adds	r1, r0, #1
	beq	.L1172
.L1204:
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #240]
	mov	r6, r0
	cmp	r3, r0
	bhi	.L1196
	movw	r2, #2059
	ldr	r1, .L1269
	ldr	r0, .L1269+4
	bl	sftl_printk
.L1196:
	ldr	r3, [r8, #72]
	ldrh	r3, [r3, r6, lsl #1]
	cmp	r3, #0
	beq	.L1197
	mov	r0, r6
	b	.L1261
.L1181:
	ldr	r3, [sp, #44]
	ldr	r2, [sp, #40]
	cmp	r2, r3
	bne	.L1172
	movs	r2, #1
	add	r1, sp, #36
	mov	r0, r6
	bl	log2phys
.L1172:
	ldr	r3, [sp, #24]
	adds	r3, r3, #1
	b	.L1264
.L1182:
	ldr	r1, [r9, #8]
	cmp	r6, r1
	bne	.L1183
	ldr	r1, [r9, #4]
	ldr	r0, [r4, #3448]
	str	r1, [sp, #28]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1183
	ldr	r1, [sp, #40]
	ldr	r0, [sp, #44]
	cmp	r1, r0
	bne	.L1185
.L1188:
	ldr	r1, [sp, #36]
	mov	r0, r6
	bl	FtlReUsePrevPpa
	b	.L1183
.L1185:
	ldr	r0, [sp, #36]
	cmp	r1, r0
	beq	.L1183
	adds	r0, r1, #1
	beq	.L1186
	str	r1, [r3, #4]
	movs	r2, #0
	movs	r1, #1
	mov	r0, r3
	ldr	r9, [r3, #12]
	bl	FlashReadPages
.L1187:
	ldr	r3, [r4, #3288]
	ldr	r3, [r3, r7]
	adds	r3, r3, #1
	beq	.L1188
	ldr	r3, [r9, #4]
	ldr	r0, [r4, #3448]
	mov	r1, r3
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1188
	mov	r1, r3
	ldr	r0, [sp, #28]
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	beq	.L1183
	b	.L1188
.L1186:
	str	r1, [r2, r7]
	b	.L1187
.L1180:
	ldr	r3, [sp, #44]
	ldr	r2, [sp, #40]
	cmp	r2, r3
	beq	.L1190
	ldr	r1, [sp, #36]
	adds	r0, r1, #1
	beq	.L1192
	ldr	r3, [r8, #248]
	cmp	r3, r1, lsr #10
	bhi	.L1192
	ldr	r0, .L1269+8
.L1263:
	bl	sftl_printk
	b	.L1172
.L1192:
	movs	r2, #1
	add	r1, sp, #44
	mov	r0, r6
	bl	log2phys
	ldr	r9, [sp, #40]
	cmp	r9, #-1
	beq	.L1190
	ldr	r3, [sp, #36]
	cmp	r9, r3
	beq	.L1194
	ubfx	r0, r9, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r4, #24]
	cmp	r3, r0
	beq	.L1195
	ldrh	r3, [r4, #76]
	cmp	r3, r0
	beq	.L1195
	ldrh	r3, [r4, #124]
	cmp	r3, r0
	bne	.L1190
.L1195:
	ldr	r0, [r8, #3288]
	movs	r2, #0
	movs	r1, #1
	str	r9, [r0, #4]
	ldr	r7, [r0, #12]
	bl	FlashReadPages
	ldr	r3, [r8, #3288]
	ldr	r3, [r3]
	adds	r3, r3, #1
	beq	.L1190
	ldr	r1, [r7, #4]
	mov	r0, fp
	bl	ftl_cmp_data_ver
	cmp	r0, #0
	bne	.L1190
	movs	r2, #1
	add	r1, sp, #40
	mov	r0, r6
	bl	log2phys
	b	.L1190
.L1197:
	mov	r1, r6
	ldr	r0, .L1269+12
	b	.L1263
.L1176:
	ldr	r3, [r4, #3476]
	cmp	r3, #31
	itttt	ls
	addls	r2, r4, r3, lsl #2
	addls	r3, r3, #1
	strls	r3, [r4, #3476]
	strls	r6, [r2, #3480]
	ldrh	r0, [r10]
	bl	decrement_vpc_count
	ldr	r3, [r4, #3448]
	adds	r2, r3, #1
	bne	.L1199
.L1262:
	str	fp, [r4, #3448]
	b	.L1172
.L1199:
	cmp	fp, r3
	bcs	.L1172
	b	.L1262
.L1175:
	ldrb	r3, [sp, #4]	@ zero_extendqisi2
	mov	r1, r5
	strh	r5, [r10, #2]	@ movhi
	ldr	r2, [sp, #4]
	strb	r3, [r10, #6]
	b	.L1266
.L1194:
	mov	r0, r9
	b	.L1204
.L1270:
	.align	2
.L1269:
	.word	.LANCHOR1+435
	.word	.LC8
	.word	.LC109
	.word	.LC110
	.fnend
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	1
	.global	ftl_check_vpc
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_check_vpc, %function
ftl_check_vpc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	movs	r4, #0
	ldr	r6, .L1295
	ldr	r7, .L1295+4
	ldr	r1, .L1295+8
	mov	r5, r6
	ldr	r0, .L1295+12
	bl	sftl_printk
	mov	r1, #8192
	ldr	r0, .L1295+4
	bl	__memzero
.L1272:
	ldr	r3, [r6, #2552]
	cmp	r4, r3
	bcc	.L1274
	ldr	r8, .L1295+4
	movs	r4, #0
	ldr	r10, .L1295+24
	mov	r7, r4
	movw	r9, #65535
.L1275:
	ldrh	r2, [r5, #240]
	uxth	r3, r4
	cmp	r2, r3
	bhi	.L1277
	ldr	r4, [r5, #2532]
	cbz	r4, .L1278
	ldr	r3, [r5, #2516]
	movs	r6, #0
	ldrh	r8, [r5, #224]
	mov	fp, #6
	ldr	r9, .L1295+4
	subs	r4, r4, r3
	ldr	r3, .L1295+16
	asrs	r4, r4, #1
	ldr	r10, .L1295+28
	muls	r4, r3, r4
	uxth	r4, r4
.L1279:
	uxth	r3, r6
	cmp	r8, r3
	bls	.L1278
	ldr	r3, [r5, #72]
	ldrh	r2, [r3, r4, lsl #1]
	cbz	r2, .L1280
	movs	r7, #1
	ldrh	r3, [r9, r4, lsl #1]
	mov	r1, r4
	mov	r0, r10
	bl	sftl_printk
.L1280:
	mul	r4, fp, r4
	ldr	r3, [r5, #2516]
	adds	r6, r6, #1
	ldrh	r4, [r3, r4]
	movw	r3, #65535
	cmp	r4, r3
	bne	.L1279
.L1278:
	cbz	r7, .L1271
	movw	r2, #2387
	ldr	r1, .L1295+8
	ldr	r0, .L1295+20
	bl	sftl_printk
.L1271:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1274:
	movs	r2, #0
	add	r1, sp, #4
	mov	r0, r4
	bl	log2phys
	ldr	r0, [sp, #4]
	adds	r3, r0, #1
	beq	.L1273
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r7, r0, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r7, r0, lsl #1]	@ movhi
.L1273:
	adds	r4, r4, #1
	b	.L1272
.L1277:
	ldr	r3, [r5, #72]
	uxth	r6, r4
	ldrh	r2, [r3, r6, lsl #1]
	ldrh	r3, [r8, r6, lsl #1]
	cmp	r2, r3
	beq	.L1276
	mov	r1, r6
	mov	r0, r10
	bl	sftl_printk
	ldr	r3, [r5, #72]
	ldrh	r3, [r3, r6, lsl #1]
	cmp	r3, r9
	beq	.L1276
	ldrh	r2, [r8, r6, lsl #1]
	cmp	r2, r3
	it	hi
	movhi	r7, #1
.L1276:
	adds	r4, r4, #1
	b	.L1275
.L1296:
	.align	2
.L1295:
	.word	.LANCHOR0
	.word	check_vpc_table
	.word	.LANCHOR1+457
	.word	.LC111
	.word	-1431655765
	.word	.LC8
	.word	.LC112
	.word	.LC113
	.fnend
	.size	ftl_check_vpc, .-ftl_check_vpc
	.align	1
	.global	ftl_scan_all_data
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_scan_all_data, %function
ftl_scan_all_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movs	r5, #0
	ldr	r6, .L1311
	.pad #32
	sub	sp, sp, #32
	movs	r1, #0
	ldr	r8, .L1311+16
	mov	r4, r6
	ldr	r0, .L1311+4
	bl	sftl_printk
.L1298:
	ldr	r3, [r6, #2552]
	cmp	r5, r3
	bcc	.L1304
	add	sp, sp, #32
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L1304:
	movs	r2, #0
	add	r1, sp, #28
	mov	r0, r5
	bl	log2phys
	ubfx	r3, r5, #0, #11
	cbnz	r3, .L1299
	ldr	r2, [sp, #28]
	mov	r1, r5
	mov	r0, r8
	bl	sftl_printk
.L1299:
	ldr	r3, [sp, #28]
	adds	r2, r3, #1
	beq	.L1301
	str	r3, [r4, #3456]
	movs	r2, #0
	ldr	r3, [r4, #3300]
	movs	r1, #1
	ldr	r7, [r4, #3332]
	ldr	r0, .L1311+8
	str	r3, [r4, #3460]
	str	r5, [r4, #3468]
	str	r7, [r4, #3464]
	str	r2, [r4, #3452]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	cmp	r3, #256
	beq	.L1302
	adds	r3, r3, #1
	beq	.L1302
	ldr	r3, [r7, #8]
	cmp	r5, r3
	beq	.L1301
.L1302:
	ldr	r2, [r4, #3460]
	ldr	r3, [r4, #3464]
	ldr	r0, .L1311+12
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
.L1301:
	adds	r5, r5, #1
	b	.L1298
.L1312:
	.align	2
.L1311:
	.word	.LANCHOR0
	.word	.LC114
	.word	.LANCHOR0+3452
	.word	.LC116
	.word	.LC115
	.fnend
	.size	ftl_scan_all_data, .-ftl_scan_all_data
	.align	1
	.global	FtlGcScanTempBlk
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1350
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	mov	r5, r0
	str	r1, [sp, #4]
	ldrh	r6, [r3, #4]
	movw	r3, #65535
	cmp	r6, r3
	beq	.L1340
	cbnz	r6, .L1314
.L1315:
	bl	FtlGcPageVarInit
	b	.L1316
.L1340:
	movs	r6, #0
.L1314:
	ldr	r3, .L1350+4
	ldr	r2, [sp, #4]
	ldrh	r3, [r3, #302]
	cmp	r3, r2
	beq	.L1315
.L1316:
	mov	fp, #0
.L1334:
	ldrh	r2, [r5]
	movs	r3, #0
	strb	r3, [r5, #8]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1317
	ldr	r4, .L1350+4
.L1337:
	ldr	r3, [r4, #3288]
	movs	r2, #0
	ldrh	ip, [r4, #232]
	add	lr, r5, #16
	ldrh	r10, [r4, #310]
	mov	r7, r2
	str	r3, [sp]
	movw	r8, #65535
	ldr	r3, [r4, #3176]
	ldr	r9, [r4, #3180]
	str	r3, [sp, #8]
	ldrh	r3, [r4, #312]
	str	r3, [sp, #12]
.L1318:
	uxth	r3, r2
	cmp	ip, r3
	bhi	.L1322
	mov	r10, #0
	movs	r2, #0
	mov	r1, r7
	ldr	r0, [sp]
	bl	FlashReadPages
.L1323:
	uxth	r3, r10
	cmp	r7, r3
	bhi	.L1335
	ldr	r3, [sp, #4]
	add	fp, fp, #1
	adds	r6, r6, #1
	uxth	r6, r6
	cmp	r3, fp
	bls	.L1336
.L1338:
	ldrh	r3, [r4, #302]
	cmp	r3, r6
	bhi	.L1337
.L1317:
	ldr	r3, .L1350
	movw	r2, #65535
	strh	r6, [r5, #2]	@ movhi
	mov	r1, r6
	mov	r0, r5
	strh	r2, [r3, #4]	@ movhi
	movs	r2, #0
	strb	r2, [r5, #6]
	bl	ftl_sb_update_avl_pages
	b	.L1339
.L1322:
	ldrh	r3, [lr], #2
	cmp	r3, r8
	beq	.L1319
	ldr	r0, [sp]
	movs	r1, #20
	orr	r3, r6, r3, lsl #10
	mla	r1, r1, r7, r0
	str	r3, [r1, #4]
	mov	r3, r10
	muls	r3, r7, r3
	it	mi
	addmi	r3, r3, #3
	bic	r3, r3, #3
	mov	r0, r3
	ldr	r3, [sp, #8]
	add	r0, r0, r3
	ldr	r3, [sp, #12]
	muls	r3, r7, r3
	add	r7, r7, #1
	it	mi
	addmi	r3, r3, #3
	uxth	r7, r7
	bic	r3, r3, #3
	str	r0, [r1, #8]
	add	r3, r3, r9
	str	r3, [r1, #12]
.L1319:
	adds	r2, r2, #1
	b	.L1318
.L1335:
	movs	r3, #20
	ldr	r2, [r4, #3288]
	mul	r3, r3, r10
	adds	r1, r2, r3
	ldr	r8, [r2, r3]
	ldr	r0, [r1, #4]
	ldr	r9, [r1, #12]
	str	r0, [sp]
	cmp	r8, #0
	bne	.L1324
	ldrh	r1, [r9]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L1325
.L1348:
	ldrh	r2, [r5]
	movs	r1, #0
	ldr	r3, [r4, #72]
	movs	r6, #0
	strh	r1, [r3, r2, lsl #1]	@ movhi
	ldrh	r0, [r5]
	bl	INSERT_FREE_LIST
	movw	r3, #65535
	strh	r3, [r5]	@ movhi
	strh	r3, [r4, #172]	@ movhi
	bl	FtlGcPageVarInit
	b	.L1334
.L1325:
	ldr	r0, [r9, #8]
	ldr	r2, [r4, #2552]
	str	r3, [sp, #8]
	cmp	r0, r2
	bhi	.L1348
	mov	r2, r8
	add	r1, sp, #16
	bl	log2phys
	ldr	r2, [r9, #12]
	ldr	r1, [sp, #16]
	ldr	r3, [sp, #8]
	cmp	r2, r1
	beq	.L1328
.L1330:
	ldr	r2, [r9, #8]
.L1349:
	ldr	r1, [sp]
	add	r10, r10, #1
	ldr	r0, [r9, #12]
	bl	FtlGcUpdatePage
	b	.L1323
.L1328:
	str	r3, [sp, #8]
	adds	r3, r2, #1
	beq	.L1330
	str	r2, [sp, #24]
	movs	r1, #1
	ldr	r2, [r4, #3316]
	add	r0, sp, #20
	str	r2, [sp, #28]
	ldr	r2, [r4, #3336]
	str	r2, [sp, #32]
	mov	r2, r8
	bl	FlashReadPages
	ldrh	r2, [r4, #258]
	ldr	r1, [r4, #3288]
	ldr	r3, [sp, #8]
	ldr	r0, [sp, #28]
	lsls	r2, r2, #7
	add	ip, r3, r1
.L1331:
	cmp	r8, r2
	beq	.L1330
	ldr	r1, [ip, #8]
	ldr	r3, [r0, r8, lsl #2]
	ldr	r1, [r1, r8, lsl #2]
	cmp	r1, r3
	beq	.L1332
	ldr	r2, [sp, #24]
	ldrh	r1, [r5]
	ldr	r0, .L1350+8
	bl	sftl_printk
	b	.L1348
.L1332:
	add	r8, r8, #1
	b	.L1331
.L1324:
	mov	r2, #-1
	b	.L1349
.L1336:
	ldr	r2, .L1350
	movw	r1, #65535
	ldrh	r3, [r2, #4]
	cmp	r3, r1
	beq	.L1338
	add	r3, r3, fp
	strh	r3, [r2, #4]	@ movhi
	ldrh	r3, [r4, #302]
	cmp	r3, r6
	bls	.L1338
.L1339:
	mov	r0, #-1
	add	sp, sp, #44
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1351:
	.align	2
.L1350:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC117
	.fnend
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	1
	.global	FtlReadRefresh
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #92
	sub	sp, sp, #92
	ldr	r6, .L1368
	ldr	r9, [r6, #2720]
	mov	r5, r6
	cmp	r9, #0
	beq	.L1353
	ldr	r2, [r6, #2724]
	ldr	r3, [r6, #2552]
	cmp	r2, r3
	bcs	.L1354
	mov	r4, #2048
.L1359:
	ldr	r0, [r5, #2724]
	ldr	r3, [r5, #2552]
	cmp	r0, r3
	bcc	.L1355
.L1358:
	mov	r0, #-1
.L1352:
	add	sp, sp, #92
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L1355:
	movs	r2, #0
	mov	r1, sp
	bl	log2phys
	ldr	r2, [sp]
	ldr	r3, [r5, #2724]
	adds	r1, r2, #1
	add	r3, r3, #1
	str	r3, [r5, #2724]
	beq	.L1357
	str	r3, [sp, #20]
	add	r0, sp, #88
	ldr	r3, [r5, #3324]
	movs	r1, #1
	str	r2, [sp, #8]
	movs	r2, #0
	str	r2, [r0, #-84]!
	str	r3, [sp, #12]
	add	r3, sp, #24
	str	r3, [sp, #16]
	bl	FlashReadPages
	ldr	r3, [sp, #4]
	cmp	r3, #256
	bne	.L1358
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L1358
.L1357:
	subs	r4, r4, #1
	bne	.L1359
	b	.L1358
.L1354:
	ldr	r3, [r6, #2556]
	movs	r0, #0
	str	r0, [r6, #2720]
	str	r0, [r6, #2724]
	str	r3, [r6, #2716]
	b	.L1352
.L1353:
	ldr	r8, [r6, #2556]
	movw	r4, #10000
	ldr	r1, [r6, #2612]
	ldr	r7, [r6, #2716]
	add	r3, r8, #1048576
	cmp	r1, r4
	ite	hi
	movhi	r4, #31
	movls	r4, #63
	cmp	r7, r3
	bhi	.L1363
	ldr	r3, [r6, #2552]
	lsrs	r1, r1, #10
	mov	r0, #1000
	adds	r1, r1, #1
	muls	r0, r3, r0
	bl	__aeabi_uidiv
	add	r0, r0, r7
	cmp	r8, r0
	bhi	.L1363
	ldrh	r3, [r6, #2496]
	ands	r0, r4, r3
	bne	.L1365
	ldr	r2, [r6, #2740]
	cmp	r3, r2
	beq	.L1352
.L1363:
	ldrh	r3, [r5, #2496]
	movs	r0, #0
	str	r0, [r5, #2724]
	str	r8, [r5, #2716]
	str	r3, [r5, #2740]
	movs	r3, #1
	str	r3, [r5, #2720]
	b	.L1352
.L1365:
	mov	r0, r9
	b	.L1352
.L1369:
	.align	2
.L1368:
	.word	.LANCHOR0
	.fnend
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	1
	.global	l2p_flush
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	l2p_flush, %function
l2p_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r4, #0
	ldr	r5, .L1374
	movs	r6, #12
	bl	FtlWriteDump_data
.L1371:
	ldrh	r3, [r5, #338]
	uxth	r0, r4
	cmp	r3, r0
	bhi	.L1373
	movs	r0, #0
	pop	{r4, r5, r6, pc}
.L1373:
	ldr	r2, [r5, #2540]
	uxth	r3, r4
	mla	r3, r6, r3, r2
	ldr	r3, [r3, #4]
	cmp	r3, #0
	bge	.L1372
	bl	flush_l2p_region
.L1372:
	adds	r4, r4, #1
	b	.L1371
.L1375:
	.align	2
.L1374:
	.word	.LANCHOR0
	.fnend
	.size	l2p_flush, .-l2p_flush
	.align	1
	.global	FtlVendorPartWrite
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r2
	ldr	r4, .L1388
	adds	r2, r0, r1
	.pad #108
	sub	sp, sp, #108
	mov	r8, r0
	mov	r6, r1
	ldrh	r3, [r4, #296]
	cmp	r2, r3
	bhi	.L1384
	ldrh	r7, [r4, #308]
	mov	r9, #0
	lsr	r7, r0, r7
	lsl	fp, r7, #2
.L1378:
	cbnz	r6, .L1383
.L1376:
	mov	r0, r9
	add	sp, sp, #108
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1383:
	ldr	r3, [r4, #3372]
	mov	r0, r8
	ldr	r2, [r3, fp]
	ldrh	r3, [r4, #258]
	str	r2, [sp, #12]
	mov	r1, r3
	str	r3, [sp, #8]
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #8]
	ldr	r2, [sp, #12]
	str	r1, [sp, #4]
	subs	r5, r3, r1
	uxth	r5, r5
	cmp	r6, r5
	it	cc
	uxthcc	r5, r6
	cbz	r2, .L1380
	cmp	r5, r3
	beq	.L1380
	ldr	r3, [r4, #3308]
	add	r0, sp, #20
	str	r2, [sp, #24]
	movs	r2, #1
	mov	r1, r2
	str	r3, [sp, #28]
	add	r3, sp, #40
	str	r3, [sp, #32]
	bl	FlashReadPages
.L1381:
	lsls	r3, r5, #9
	ldr	r0, [r4, #3308]
	mov	r1, r10
	mov	r2, r3
	str	r3, [sp, #8]
	ldr	r3, [sp, #4]
	subs	r6, r6, r5
	add	r8, r8, r5
	add	fp, fp, #4
	add	r0, r0, r3, lsl #9
	bl	ftl_memcpy
	mov	r1, r7
	ldr	r2, [r4, #3308]
	ldr	r0, .L1388+4
	adds	r7, r7, #1
	bl	FtlMapWritePage
	ldr	r3, [sp, #8]
	adds	r0, r0, #1
	it	eq
	moveq	r9, #-1
	add	r10, r10, r3
	b	.L1378
.L1380:
	ldrh	r2, [r4, #310]
	movs	r1, #0
	ldr	r0, [r4, #3308]
	bl	ftl_memset
	b	.L1381
.L1384:
	mov	r9, #-1
	b	.L1376
.L1389:
	.align	2
.L1388:
	.word	.LANCHOR0
	.word	.LANCHOR0+3608
	.fnend
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	1
	.global	Ftl_save_ext_data
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L1392
	ldr	r3, .L1392+4
	ldr	r1, [r2, #2640]
	cmp	r1, r3
	bne	.L1390
	ldr	r3, .L1392+8
	add	r2, r2, #2640
	movs	r1, #1
	movs	r0, #0
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
	b	FtlVendorPartWrite
.L1390:
	bx	lr
.L1393:
	.align	2
.L1392:
	.word	.LANCHOR0
	.word	1179929683
	.word	1342177365
	.fnend
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	1
	.global	FtlEctTblFlush
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	ldr	r3, .L1399
	ldrh	r2, [r3, #3652]
	cmp	r2, #31
	itett	ls
	addls	r2, r2, #1
	movhi	r2, #32
	strhls	r2, [r3, #3652]	@ movhi
	movls	r2, #1
	cbnz	r0, .L1396
	ldr	r1, [r3, #3348]
	ldr	r0, [r1, #20]
	ldr	r1, [r1, #16]
	add	r2, r2, r0
	cmp	r1, r2
	bcc	.L1397
.L1396:
	ldr	r2, [r3, #3348]
	movs	r0, #64
	ldr	r1, [r2, #16]
	str	r1, [r2, #20]
	ldr	r1, .L1399+4
	str	r1, [r2]
	ldrh	r1, [r3, #3340]
	ldr	r2, [r3, #3348]
	lsls	r3, r1, #9
	str	r3, [r2, #12]
	ldr	r3, [r2, #8]
	adds	r3, r3, #1
	str	r3, [r2, #8]
	movs	r3, #0
	str	r3, [r2, #4]
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L1397:
	movs	r0, #0
	pop	{r3, pc}
.L1400:
	.align	2
.L1399:
	.word	.LANCHOR0
	.word	1112818501
	.fnend
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	1
	.global	sftl_vendor_write
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_vendor_write, %function
sftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	add	r0, r0, #256
	b	FtlVendorPartWrite
	.fnend
	.size	sftl_vendor_write, .-sftl_vendor_write
	.align	1
	.global	FtlVendorPartRead
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 104
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r2
	ldr	r4, .L1412
	adds	r2, r0, r1
	.pad #108
	sub	sp, sp, #108
	mov	r8, r0
	mov	r7, r1
	ldrh	r3, [r4, #296]
	cmp	r2, r3
	bhi	.L1411
	ldrh	r6, [r4, #308]
	mov	r9, #0
	lsr	r6, r0, r6
	lsl	fp, r6, #2
.L1404:
	cbnz	r7, .L1410
.L1402:
	mov	r0, r9
	add	sp, sp, #108
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1410:
	ldr	r3, [r4, #3372]
	mov	r0, r8
	ldrh	r5, [r4, #258]
	ldr	r3, [r3, fp]
	mov	r1, r5
	str	r3, [sp, #8]
	bl	__aeabi_uidivmod
	subs	r5, r5, r1
	ldr	r3, [sp, #8]
	uxth	r5, r5
	str	r1, [sp, #4]
	cmp	r7, r5
	it	cc
	uxthcc	r5, r7
	lsls	r2, r5, #9
	str	r2, [sp, #8]
	cbz	r3, .L1406
	ldr	r2, [r4, #3308]
	add	r0, sp, #20
	str	r3, [sp, #24]
	str	r3, [sp, #12]
	str	r2, [sp, #28]
	add	r2, sp, #40
	str	r2, [sp, #32]
	movs	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r2, [sp, #20]
	ldr	r3, [sp, #12]
	adds	r2, r2, #1
	ldr	r2, [r4, #3452]
	it	eq
	moveq	r9, #-1
	cmp	r2, #256
	bne	.L1408
	mov	r2, r3
	mov	r1, r6
	ldr	r0, .L1412+4
	bl	sftl_printk
	ldr	r2, [r4, #3308]
	mov	r1, r6
	ldr	r0, .L1412+8
	bl	FtlMapWritePage
.L1408:
	ldr	r1, [r4, #3308]
	lsls	r2, r5, #9
	ldr	r3, [sp, #4]
	mov	r0, r10
	add	r1, r1, r3, lsl #9
	bl	ftl_memcpy
.L1409:
	ldr	r3, [sp, #8]
	adds	r6, r6, #1
	subs	r7, r7, r5
	add	r8, r8, r5
	add	fp, fp, #4
	add	r10, r10, r3
	b	.L1404
.L1406:
	lsls	r2, r5, #9
	mov	r1, r3
	mov	r0, r10
	bl	ftl_memset
	b	.L1409
.L1411:
	mov	r9, #-1
	b	.L1402
.L1413:
	.align	2
.L1412:
	.word	.LANCHOR0
	.word	.LC118
	.word	.LANCHOR0+3608
	.fnend
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	1
	.global	FtlLoadEctTbl
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlLoadEctTbl, %function
FtlLoadEctTbl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	movs	r0, #64
	ldr	r4, .L1416
	ldr	r2, [r4, #3348]
	ldrh	r1, [r4, #3340]
	bl	FtlVendorPartRead
	ldr	r3, [r4, #3348]
	ldr	r2, [r3]
	ldr	r3, .L1416+4
	cmp	r2, r3
	beq	.L1415
	ldr	r1, .L1416+8
	ldr	r0, .L1416+12
	bl	sftl_printk
	ldrh	r2, [r4, #3340]
	movs	r1, #0
	ldr	r0, [r4, #3348]
	lsls	r2, r2, #9
	bl	ftl_memset
.L1415:
	movs	r0, #0
	pop	{r4, pc}
.L1417:
	.align	2
.L1416:
	.word	.LANCHOR0
	.word	1112818501
	.word	.LC119
	.word	.LC77
	.fnend
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	1
	.global	Ftl_load_ext_data
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	Ftl_load_ext_data, %function
Ftl_load_ext_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movs	r1, #1
	ldr	r4, .L1421
	movs	r0, #0
	ldr	r5, .L1421+4
	add	r2, r4, #2640
	bl	FtlVendorPartRead
	ldr	r3, [r4, #2640]
	cmp	r3, r5
	beq	.L1419
	mov	r1, #512
	add	r0, r4, #2640
	bl	__memzero
	str	r5, [r4, #2640]
.L1419:
	ldr	r3, [r4, #2640]
	cmp	r3, r5
	bne	.L1420
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
.L1420:
	ldrh	r2, [r4, #292]
	ldr	r3, [r4, #2600]
	ldr	r0, [r4, #2596]
	ldrh	r1, [r4, #240]
	mla	r0, r0, r2, r3
	bl	__aeabi_uidiv
	str	r0, [r4, #2604]
	pop	{r3, r4, r5, pc}
.L1422:
	.align	2
.L1421:
	.word	.LANCHOR0
	.word	1179929683
	.fnend
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	1
	.global	sftl_vendor_read
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_vendor_read, %function
sftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	add	r0, r0, #256
	b	FtlVendorPartRead
	.fnend
	.size	sftl_vendor_read, .-sftl_vendor_read
	.align	1
	.global	FtlMapBlkWriteDump_data
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlMapBlkWriteDump_data, %function
FtlMapBlkWriteDump_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldr	r3, [r0, #36]
	cbz	r3, .L1424
	ldrh	r5, [r0, #6]
	movs	r3, #0
	ldr	r4, .L1428
	ldr	r2, [r0, #24]
	str	r3, [r0, #36]
	subs	r5, r5, #1
	ldr	r0, [r4, #3304]
	uxth	r5, r5
	ldr	r1, [r4, #3332]
	str	r0, [r4, #3460]
	str	r1, [r4, #3464]
	ldr	r2, [r2, r5, lsl #2]
	str	r2, [r4, #3456]
	cbz	r2, .L1426
	movs	r2, #1
	addw	r0, r4, #3452
	mov	r1, r2
	bl	FlashReadPages
.L1427:
	ldr	r2, [r4, #3460]
	mov	r1, r5
	mov	r0, r6
	pop	{r4, r5, r6, lr}
	b	FtlMapWritePage
.L1426:
	ldrh	r2, [r4, #310]
	movs	r1, #255
	bl	ftl_memset
	b	.L1427
.L1424:
	pop	{r4, r5, r6, pc}
.L1429:
	.align	2
.L1428:
	.word	.LANCHOR0
	.fnend
	.size	FtlMapBlkWriteDump_data, .-FtlMapBlkWriteDump_data
	.align	1
	.global	FtlVpcTblFlush
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	movs	r5, #0
	ldr	r4, .L1445
	movs	r1, #255
	movw	r7, #65535
	ldr	r9, .L1445+16
	ldr	r3, [r4, #3300]
	addw	r8, r4, #3452
	ldr	r6, [r4, #3332]
	ldr	r10, .L1445+20
	str	r3, [r4, #3460]
	ldrh	r3, [r4, #2624]
	str	r6, [r4, #3464]
	str	r5, [r6, #12]
	strh	r3, [r6, #2]	@ movhi
	movw	r3, #61604
	strh	r3, [r6]	@ movhi
	ldr	r3, [r4, #2632]
	str	r5, [r6, #8]
	ldrh	r2, [r4, #26]
	str	r3, [r6, #4]
	ldr	r3, .L1445+4
	str	r3, [r4, #2468]
	ldr	r3, .L1445+8
	str	r3, [r4, #2472]
	ldrh	r3, [r4, #2630]
	strh	r3, [r4, #2476]	@ movhi
	ldrh	r3, [r4, #254]
	strb	r3, [r4, #2478]
	ldrh	r3, [r4, #24]
	strh	r3, [r4, #2482]	@ movhi
	ldrb	r3, [r4, #30]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #6
	ldrh	r2, [r4, #78]
	strh	r3, [r4, #2484]	@ movhi
	ldrb	r3, [r4, #32]	@ zero_extendqisi2
	strb	r3, [r4, #2479]
	ldrh	r3, [r4, #76]
	strh	r3, [r4, #2486]	@ movhi
	ldrb	r3, [r4, #82]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #6
	strh	r3, [r4, #2488]	@ movhi
	ldrb	r3, [r4, #84]	@ zero_extendqisi2
	strb	r3, [r4, #2480]
	ldrh	r3, [r4, #124]
	ldrh	r2, [r4, #126]
	ldr	r0, [r4, #3460]
	strh	r3, [r4, #2490]	@ movhi
	ldrb	r3, [r4, #130]	@ zero_extendqisi2
	orr	r3, r3, r2, lsl #6
	ldrh	r2, [r4, #310]
	strh	r3, [r4, #2492]	@ movhi
	ldrb	r3, [r4, #132]	@ zero_extendqisi2
	strb	r3, [r4, #2481]
	ldr	r3, [r4, #2596]
	str	r3, [r4, #2500]
	ldr	r3, [r4, #2588]
	str	r3, [r4, #2508]
	ldr	r3, [r4, #2592]
	str	r3, [r4, #2504]
	bl	ftl_memset
	movs	r2, #48
	addw	r1, r4, #2468
	ldr	r0, [r4, #3460]
	bl	ftl_memcpy
	ldrh	r2, [r4, #240]
	ldr	r0, [r4, #3460]
	ldr	r1, [r4, #72]
	lsls	r2, r2, #1
	adds	r0, r0, #48
	bl	ftl_memcpy
	mov	r0, r5
	bl	FtlUpdateVaildLpn
.L1431:
	ldr	r3, [r4, #3300]
	ldrh	r1, [r4, #2626]
	ldrh	r2, [r4, #2624]
	str	r3, [r4, #3460]
	ldr	r3, [r4, #3332]
	str	r3, [r4, #3464]
	orr	r3, r1, r2, lsl #10
	str	r3, [r4, #3456]
	ldrh	r3, [r4, #304]
	subs	r3, r3, #1
	cmp	r1, r3
	blt	.L1432
	movs	r3, #0
	ldrh	r7, [r4, #2628]
	strh	r3, [r4, #2626]	@ movhi
	strh	r2, [r4, #2628]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	ldr	r3, [r4, #2588]
	strh	r0, [r4, #2624]	@ movhi
	adds	r2, r3, #1
	str	r3, [r4, #2632]
	str	r2, [r4, #2588]
	lsls	r2, r0, #10
	str	r2, [r4, #3456]
	str	r3, [r6, #4]
	strh	r0, [r6, #2]	@ movhi
.L1432:
	ldrh	r1, [r4, #310]
	ldr	r0, [r4, #3300]
	bl	js_hash
	movs	r3, #1
	str	r0, [r6, #12]
	mov	r2, r3
	mov	r1, r3
	mov	r0, r8
	bl	FlashProgPages
	ldrh	r3, [r4, #2626]
	ldr	r2, [r4, #3452]
	adds	r3, r3, #1
	uxth	r3, r3
	adds	r1, r2, #1
	strh	r3, [r4, #2626]	@ movhi
	bne	.L1433
	cmp	r3, #1
	bne	.L1434
	movw	r2, #1138
	mov	r1, r9
	mov	r0, r10
	bl	sftl_printk
.L1434:
	ldrh	r3, [r4, #2626]
	adds	r5, r5, #1
	uxth	r5, r5
	cmp	r3, #1
	ittt	eq
	ldrheq	r3, [r4, #304]
	addeq	r3, r3, #-1
	strheq	r3, [r4, #2626]	@ movhi
	cmp	r5, #3
	bls	.L1431
	mov	r2, r5
	ldr	r1, [r4, #3456]
	ldr	r0, .L1445+12
	bl	sftl_printk
.L1437:
	b	.L1437
.L1433:
	cmp	r3, #1
	beq	.L1431
	cmp	r2, #256
	beq	.L1431
	movw	r3, #65535
	cmp	r7, r3
	beq	.L1438
	movs	r1, #1
	mov	r0, r7
	bl	FtlFreeSysBlkQueueIn
.L1438:
	movs	r0, #0
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L1446:
	.align	2
.L1445:
	.word	.LANCHOR0
	.word	1179929683
	.word	1342177365
	.word	.LC120
	.word	.LANCHOR1+471
	.word	.LC8
	.fnend
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	1
	.global	FtlSysFlush
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlSysFlush, %function
FtlSysFlush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	l2p_flush
	movs	r0, #1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	movs	r0, #0
	pop	{r3, pc}
	.fnend
	.size	FtlSysFlush, .-FtlSysFlush
	.align	1
	.global	sftl_deinit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_deinit, %function
sftl_deinit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	ldr	r3, .L1450
	ldr	r3, [r3]
	cmp	r3, #1
	bne	.L1449
	bl	FtlSysFlush
.L1449:
	movs	r0, #0
	pop	{r3, pc}
.L1451:
	.align	2
.L1450:
	.word	.LANCHOR2
	.fnend
	.size	sftl_deinit, .-sftl_deinit
	.align	1
	.global	sftl_discard
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_discard, %function
sftl_discard:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r7, r0
	ldr	r4, .L1469
	mov	r5, r1
	ldr	r3, [r4, #340]
	cmp	r3, r0
	bls	.L1462
	cmp	r3, r1
	bcc	.L1462
	adds	r2, r0, r1
	cmp	r3, r2
	bcc	.L1462
	cmp	r1, #31
	bhi	.L1454
.L1459:
	movs	r0, #0
.L1452:
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L1454:
	ldrh	r6, [r4, #258]
	mov	r1, r6
	bl	__aeabi_uidiv
	smulbb	r3, r0, r6
	mov	r8, r0
	subs	r7, r7, r3
	uxth	r7, r7
	cbz	r7, .L1455
	subs	r6, r6, r7
	add	r8, r0, #1
	cmp	r6, r5
	it	cs
	movcs	r6, r5
	uxth	r6, r6
	subs	r5, r5, r6
.L1455:
	mov	r3, #-1
	str	r3, [sp, #4]
.L1456:
	ldrh	r3, [r4, #258]
	cmp	r5, r3
	bcs	.L1458
	ldr	r3, [r4, #3656]
	cmp	r3, #32
	bls	.L1459
	movs	r5, #0
	str	r5, [r4, #3656]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L1459
.L1458:
	movs	r2, #0
	mov	r1, sp
	mov	r0, r8
	bl	log2phys
	ldr	r3, [sp]
	adds	r3, r3, #1
	beq	.L1457
	ldr	r3, [r4, #3656]
	movs	r2, #1
	add	r1, sp, #4
	mov	r0, r8
	adds	r3, r3, #1
	str	r3, [r4, #3656]
	ldr	r3, [r4, #2560]
	adds	r3, r3, #1
	str	r3, [r4, #2560]
	bl	log2phys
	ldr	r0, [sp]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L1457:
	ldrh	r3, [r4, #258]
	add	r8, r8, #1
	subs	r5, r5, r3
	b	.L1456
.L1462:
	mov	r0, #-1
	b	.L1452
.L1470:
	.align	2
.L1469:
	.word	.LANCHOR0
	.fnend
	.size	sftl_discard, .-sftl_discard
	.align	1
	.global	FtlVpcCheckAndModify
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #8
	movs	r5, #0
	ldr	r4, .L1485
	ldr	r1, .L1485+4
	ldr	r0, .L1485+8
	bl	sftl_printk
	ldrh	r2, [r4, #242]
	movs	r1, #0
	ldr	r0, [r4, #3352]
	lsls	r2, r2, #1
	bl	ftl_memset
.L1472:
	ldr	r3, [r4, #2552]
	cmp	r5, r3
	bcc	.L1474
	ldr	r8, .L1485+16
	movs	r7, #0
	ldr	r9, .L1485+20
	ldr	r10, .L1485+4
.L1475:
	ldrh	r3, [r4, #240]
	uxth	r6, r7
	cmp	r3, r6
	bhi	.L1481
	bl	l2p_flush
	bl	FtlVpcTblFlush
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L1474:
	movs	r2, #0
	add	r1, sp, #4
	mov	r0, r5
	bl	log2phys
	ldr	r0, [sp, #4]
	adds	r3, r0, #1
	beq	.L1473
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldr	r2, [r4, #3352]
	ldrh	r3, [r2, r0, lsl #1]
	adds	r3, r3, #1
	strh	r3, [r2, r0, lsl #1]	@ movhi
.L1473:
	adds	r5, r5, #1
	b	.L1472
.L1481:
	ldr	r3, [r4, #72]
	uxth	r5, r7
	ldrh	r2, [r3, r5, lsl #1]
	ldr	r3, [r4, #3352]
	ldrh	r3, [r3, r5, lsl #1]
	cmp	r2, r3
	beq	.L1477
	movw	r1, #65535
	cmp	r2, r1
	beq	.L1477
	mov	r1, r5
	mov	r0, r8
	bl	sftl_printk
	ldrh	r3, [r4, #24]
	cmp	r3, r6
	beq	.L1477
	ldrh	r3, [r4, #124]
	cmp	r3, r6
	beq	.L1477
	ldrh	r3, [r4, #76]
	cmp	r3, r6
	beq	.L1477
	ldr	r3, [r4, #72]
	ldrh	r2, [r3, r5, lsl #1]
	cbnz	r2, .L1479
	ldr	r2, [r4, #3352]
	mov	r1, r6
	mov	r0, r9
	ldrh	r2, [r2, r5, lsl #1]
	strh	r2, [r3, r5, lsl #1]	@ movhi
	bl	List_remove_node
	ldrh	r3, [r4, #224]
	cbnz	r3, .L1480
	movw	r2, #2323
	mov	r1, r10
	ldr	r0, .L1485+12
	bl	sftl_printk
.L1480:
	ldrh	r3, [r4, #224]
	mov	r0, r6
	subs	r3, r3, #1
	strh	r3, [r4, #224]	@ movhi
	bl	INSERT_DATA_LIST
.L1477:
	adds	r7, r7, #1
	b	.L1475
.L1479:
	ldr	r2, [r4, #3352]
	mov	r0, r6
	ldrh	r2, [r2, r5, lsl #1]
	strh	r2, [r3, r5, lsl #1]	@ movhi
	bl	update_vpc_list
	b	.L1477
.L1486:
	.align	2
.L1485:
	.word	.LANCHOR0
	.word	.LANCHOR1+486
	.word	.LC111
	.word	.LC8
	.word	.LC121
	.word	.LANCHOR0+2532
	.fnend
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	1
	.global	allocate_new_data_superblock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldr	r4, .L1497
	ldrh	r5, [r0]
	ldrh	r3, [r4, #240]
	cmp	r3, r5
	bcs	.L1488
	movw	r2, #2759
	ldr	r1, .L1497+4
	ldr	r0, .L1497+8
	bl	sftl_printk
.L1488:
	movw	r3, #65535
	cmp	r5, r3
	beq	.L1489
	ldr	r3, [r4, #72]
	mov	r0, r5
	ldrh	r3, [r3, r5, lsl #1]
	cbz	r3, .L1490
	bl	INSERT_DATA_LIST
.L1489:
	ldrh	r0, [r4, #3438]
	movs	r3, #1
	strb	r3, [r6, #8]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1491
	cmp	r5, r0
	bne	.L1492
	ldr	r3, [r4, #72]
	ldrh	r3, [r3, r0, lsl #1]
	cbz	r3, .L1493
.L1492:
	bl	update_vpc_list
.L1493:
	movw	r3, #65535
	strh	r3, [r4, #3438]	@ movhi
.L1491:
	mov	r0, r6
	bl	allocate_data_superblock
	bl	l2p_flush
	movs	r0, #0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	movs	r0, #0
	pop	{r4, r5, r6, pc}
.L1490:
	bl	INSERT_FREE_LIST
	b	.L1489
.L1498:
	.align	2
.L1497:
	.word	.LANCHOR0
	.word	.LANCHOR1+507
	.word	.LC8
	.fnend
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	1
	.global	FtlProgPages
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlProgPages, %function
FtlProgPages:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	movs	r6, #0
	ldr	r10, .L1518+16
	mov	r5, r3
	movs	r2, #0
	mov	r4, r0
	ldr	r8, .L1518
	mov	r9, r1
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	bl	FlashProgPages
.L1500:
	cmp	r6, r9
	bne	.L1507
	ldr	r3, .L1518
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	ldrh	r3, [r3, #232]
	cmp	r2, r3
	bcc	.L1499
	mov	r2, #1000
	ldr	r1, .L1518+4
	ldr	r0, .L1518+8
	bl	sftl_printk
.L1499:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1502:
	ldr	r1, [r4, #4]
	mov	r0, r10
	bl	sftl_printk
	ldr	r0, [r4, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
	ldrh	r3, [r5, #4]
	cbnz	r3, .L1501
	mov	r0, r5
	bl	allocate_new_data_superblock
.L1501:
	mov	r0, r5
	bl	get_new_active_ppa
	movs	r2, #0
	str	r0, [r4, #4]
	str	r0, [sp, #4]
	movs	r1, #1
	ldrb	r3, [r5, #9]	@ zero_extendqisi2
	mov	r0, r4
	bl	FlashProgPages
.L1507:
	ldr	r2, [r4]
	adds	r3, r2, #1
	beq	.L1502
	cmp	r2, #256
	beq	.L1502
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	ldrh	r3, [r8, #232]
	cmp	r2, r3
	bcc	.L1503
	movw	r2, #985
	ldr	r1, .L1518+4
	ldr	r0, .L1518+8
	bl	sftl_printk
.L1503:
	ldr	r3, [r4, #4]
	add	r1, sp, #8
	movs	r2, #1
	ldr	r0, [r4, #16]
	str	r3, [r1, #-4]!
	bl	log2phys
	ldr	r3, [r4, #12]
	ldr	fp, [r3, #12]
	ubfx	r0, fp, #10, #16
	bl	P2V_block_in_plane
	cmp	fp, #-1
	mov	r7, r0
	beq	.L1504
	ldr	r3, [r8, #72]
	ldrh	r2, [r3, r0, lsl #1]
	cbnz	r2, .L1505
	mov	r1, r0
	ldr	r0, .L1518+12
	bl	sftl_printk
.L1505:
	mov	r0, r7
	bl	decrement_vpc_count
.L1504:
	adds	r6, r6, #1
	adds	r4, r4, #20
	b	.L1500
.L1519:
	.align	2
.L1518:
	.word	.LANCHOR0
	.word	.LANCHOR1+536
	.word	.LC8
	.word	.LC123
	.word	.LC122
	.fnend
	.size	FtlProgPages, .-FtlProgPages
	.align	1
	.global	FtlGcFreeTempBlock
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	movw	r3, #65535
	ldr	r4, .L1544
	ldrh	r2, [r4, #124]
	ldrh	r1, [r4, #302]
	cmp	r2, r3
	bne	.L1521
.L1528:
	ldrh	r2, [r4, #124]
	movs	r3, #0
	str	r3, [r4, #3444]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1541
.L1522:
	movs	r0, #0
.L1520:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L1521:
	ldr	r5, .L1544+4
	cbz	r0, .L1524
	ldrh	r2, [r5, #4]
	cmp	r2, r3
	beq	.L1525
.L1526:
	movs	r1, #2
.L1524:
	ldr	r0, .L1544+8
	bl	FtlGcScanTempBlk
	ldrh	r2, [r5, #4]
	movw	r3, #65535
	str	r0, [sp, #4]
	cmp	r2, r3
	beq	.L1528
	movs	r0, #1
	b	.L1520
.L1525:
	movs	r3, #0
	strh	r3, [r5, #4]	@ movhi
	ldrh	r3, [r4, #224]
	cmp	r3, #17
	bhi	.L1526
	b	.L1524
.L1541:
	ldrb	r1, [r4, #131]	@ zero_extendqisi2
	ldrh	r3, [r4, #302]
	ldrh	r2, [r4, #3204]
	muls	r3, r1, r3
	cmp	r2, r3
	beq	.L1529
	movs	r2, #164
	ldr	r1, .L1544+12
	ldr	r0, .L1544+16
	bl	sftl_printk
.L1529:
	ldrb	r3, [r4, #131]	@ zero_extendqisi2
	movs	r5, #0
	ldrh	r0, [r4, #302]
	mov	r8, #12
	ldr	r2, [r4, #72]
	ldrh	r1, [r4, #124]
	smulbb	r3, r3, r0
	strh	r3, [r2, r1, lsl #1]	@ movhi
	ldr	r2, [r4, #2576]
	ldrh	r3, [r4, #3204]
	add	r3, r3, r2
	str	r3, [r4, #2576]
.L1530:
	ldrh	r2, [r4, #3204]
	uxth	r3, r5
	cmp	r2, r3
	bhi	.L1534
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r0, [r4, #124]
	ldr	r3, [r4, #72]
	ldrh	r3, [r3, r0, lsl #1]
	cmp	r3, #0
	beq	.L1535
	bl	INSERT_DATA_LIST
.L1536:
	movw	r6, #65535
	movs	r5, #0
	strh	r6, [r4, #124]	@ movhi
	strh	r5, [r4, #3204]	@ movhi
	strh	r5, [r4, #3196]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldrh	r3, [r4, #2620]
	ldrh	r2, [r4, #224]
	strh	r6, [r4, #172]	@ movhi
	add	r3, r3, r3, lsl #1
	cmp	r2, r3, asr #2
	ble	.L1522
	movs	r3, #20
	strh	r3, [r4, #3152]	@ movhi
	b	.L1522
.L1534:
	uxth	r7, r5
	ldr	r9, [r4, #3200]
	ldr	r3, [r4, #2552]
	mul	r7, r8, r7
	add	r6, r9, r7
	ldr	r0, [r6, #8]
	cmp	r0, r3
	bcc	.L1531
.L1542:
	ldrh	r0, [r4, #124]
	b	.L1543
.L1531:
	movs	r2, #0
	add	r1, sp, #4
	bl	log2phys
	ldr	r0, [r9, r7]
	ldr	r3, [sp, #4]
	cmp	r0, r3
	bne	.L1533
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	movs	r2, #1
	mov	r7, r0
	adds	r1, r6, #4
	ldr	r0, [r6, #8]
	bl	log2phys
	mov	r0, r7
.L1543:
	bl	decrement_vpc_count
	b	.L1532
.L1533:
	ldr	r2, [r6, #4]
	cmp	r3, r2
	bne	.L1542
.L1532:
	adds	r5, r5, #1
	b	.L1530
.L1535:
	bl	INSERT_FREE_LIST
	b	.L1536
.L1545:
	.align	2
.L1544:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+124
	.word	.LANCHOR1+549
	.word	.LC8
	.fnend
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	1
	.global	FtlGcPageRecovery
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	.save {r4, lr}
	ldr	r4, .L1548
	ldrh	r1, [r4, #302]
	add	r0, r4, #124
	bl	FtlGcScanTempBlk
	ldrh	r2, [r4, #126]
	ldrh	r3, [r4, #302]
	cmp	r2, r3
	bcc	.L1546
	add	r0, r4, #3392
	bl	FtlMapBlkWriteDump_data
	movs	r0, #0
	bl	FtlGcFreeTempBlock
	movs	r3, #0
	str	r3, [r4, #3444]
.L1546:
	pop	{r4, pc}
.L1549:
	.align	2
.L1548:
	.word	.LANCHOR0
	.fnend
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	1
	.global	FtlPowerLostRecovery
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r5, #0
	ldr	r4, .L1551
	add	r6, r4, #24
	str	r5, [r4, #3476]
	mov	r0, r6
	adds	r4, r4, #76
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
.L1552:
	.align	2
.L1551:
	.word	.LANCHOR0
	.fnend
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	1
	.global	Ftl_gc_temp_data_write_back
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r3, #0
	ldr	r4, .L1561
	movs	r5, #0
	movs	r6, #20
	mov	r2, r3
	ldr	r1, [r4, #3168]
	ldr	r0, [r4, #3292]
	bl	FlashProgPages
.L1554:
	ldr	r1, [r4, #3168]
	uxth	r3, r5
	cmp	r1, r3
	bhi	.L1557
	ldr	r0, [r4, #3292]
	bl	FtlGcBufFree
	ldrh	r3, [r4, #128]
	movs	r0, #0
	str	r0, [r4, #3168]
	cbnz	r3, .L1553
	movs	r0, #1
	bl	FtlGcFreeTempBlock
	movs	r0, #1
.L1553:
	pop	{r4, r5, r6, pc}
.L1557:
	muls	r3, r6, r3
	ldr	r2, [r4, #3292]
	adds	r5, r5, #1
	adds	r1, r2, r3
	ldr	r2, [r2, r3]
	ldr	r0, [r1, #12]
	ldr	r1, [r1, #4]
	adds	r3, r2, #1
	it	ne
	ldrne	r2, [r0, #8]
	ldr	r0, [r0, #12]
	bl	FtlGcUpdatePage
	b	.L1554
.L1562:
	.align	2
.L1561:
	.word	.LANCHOR0
	.fnend
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	1
	.global	Ftl_get_new_temp_ppa
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r3, #65535
	ldr	r4, .L1566
	ldrh	r2, [r4, #124]
	cmp	r2, r3
	beq	.L1564
	ldrh	r3, [r4, #128]
	cbnz	r3, .L1565
.L1564:
	movs	r0, #0
	movs	r5, #0
	bl	FtlGcFreeTempBlock
	ldr	r0, .L1566+4
	strb	r5, [r4, #132]
	bl	allocate_data_superblock
	strh	r5, [r4, #3196]	@ movhi
	strh	r5, [r4, #3204]	@ movhi
	bl	l2p_flush
	mov	r0, r5
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L1565:
	ldr	r0, .L1566+4
	pop	{r3, r4, r5, lr}
	b	get_new_active_ppa
.L1567:
	.align	2
.L1566:
	.word	.LANCHOR0
	.word	.LANCHOR0+124
	.fnend
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	1
	.global	rk_ftl_garbage_collect
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1699
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r0
	.pad #44
	sub	sp, sp, #44
	mov	r4, r3
	ldr	r0, [r3, #3272]
	cmp	r0, #0
	bne	.L1651
	ldrh	r2, [r3, #2528]
	cmp	r2, #47
	bls	.L1568
	ldr	r2, .L1699+4
	ldrh	r1, [r2, #4]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L1570
.L1573:
	ldrh	r2, [r4, #220]
	movw	r1, #65535
	cmp	r2, r1
	bne	.L1571
.L1572:
	bl	FtlReadRefresh
	ldr	r3, [r4, #3156]
	movw	r7, #65535
	ldrh	r1, [r4, #172]
	adds	r3, r3, #1
	add	r3, r3, r9, lsl #7
	cmp	r1, r7
	str	r3, [r4, #3156]
	bne	.L1574
	ldrh	r2, [r4, #124]
	cmp	r2, r1
	bne	.L1575
	ldrh	r6, [r4, #222]
	cmp	r6, r2
	bne	.L1576
	ldrh	r2, [r4, #224]
	cmp	r2, #24
	ite	cc
	movcc	r2, #5120
	movcs	r2, #1024
	cmp	r3, r2
	bls	.L1576
	movs	r5, #0
	str	r5, [r4, #3156]
	strh	r5, [r4, #3660]	@ movhi
	bl	GetSwlReplaceBlock
	cmp	r0, r6
	mov	r7, r0
	bne	.L1655
	ldrh	r2, [r4, #224]
	ldrh	r3, [r4, #3154]
	cmp	r2, r3
	bcs	.L1579
	movs	r0, #64
	bl	List_get_gc_head_node
	uxth	r3, r0
	cmp	r3, r7
	beq	.L1581
	mov	r0, r3
	ldr	r3, [r4, #72]
	ldrh	r3, [r3, r0, lsl #1]
	cmp	r3, #7
	bhi	.L1582
	mov	r0, r5
	bl	List_get_gc_head_node
	uxth	r5, r0
	movs	r3, #128
	strh	r3, [r4, #3154]	@ movhi
	cmp	r5, r7
	bne	.L1578
.L1581:
	bl	FtlGcReFreshBadBlk
	cmp	r9, #0
	bne	.L1584
	movw	r3, #65535
	cmp	r7, r3
	bne	.L1584
.L1649:
	ldrh	r3, [r4, #224]
	cmp	r3, #24
	bhi	.L1656
	cmp	r3, #16
	ldrh	r6, [r4, #302]
	bls	.L1586
	lsrs	r6, r6, #5
.L1585:
	ldrh	r2, [r4, #3152]
	cmp	r2, r3
	bcs	.L1589
	ldrh	r3, [r4, #124]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1590
	ldrh	r2, [r4, #222]
	cmp	r2, r3
	bne	.L1590
	ldrh	r0, [r4, #3660]
	cbnz	r0, .L1591
	ldr	r3, [r4, #2552]
	ldr	r2, [r4, #2548]
	add	r3, r3, r3, lsl #1
	cmp	r2, r3, lsr #2
	bcs	.L1592
.L1591:
	ldrh	r3, [r4, #2620]
	add	r3, r3, r3, lsl #1
	asrs	r3, r3, #2
.L1695:
	strh	r3, [r4, #3152]	@ movhi
	movs	r3, #0
	str	r3, [r4, #3164]
.L1568:
	add	sp, sp, #44
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1570:
	ldrh	r3, [r3, #124]
	cmp	r3, r2
	beq	.L1573
	movs	r0, #1
	bl	FtlGcFreeTempBlock
	cmp	r0, #0
	beq	.L1573
	movs	r0, #1
	b	.L1568
.L1571:
	ldrh	r3, [r4, #222]
	cmp	r3, r1
	itt	eq
	strheq	r2, [r4, #222]	@ movhi
	strheq	r3, [r4, #220]	@ movhi
	b	.L1572
.L1582:
	movs	r3, #64
.L1694:
	strh	r3, [r4, #3154]	@ movhi
	b	.L1581
.L1579:
	movs	r3, #80
	b	.L1694
.L1655:
	mov	r5, r0
.L1578:
	ldr	r3, [r4, #72]
	mov	r7, r5
	ldrh	r1, [r4, #3152]
	ldrh	r2, [r4, #224]
	ldrh	r3, [r3, r5, lsl #1]
	str	r1, [sp, #4]
	ldr	r1, [r4, #2536]
	ldr	r0, .L1699+8
	ldrh	r1, [r1, r5, lsl #1]
	str	r1, [sp]
	mov	r1, r5
	bl	sftl_printk
	b	.L1581
.L1586:
	cmp	r3, #12
	bls	.L1587
	lsrs	r6, r6, #4
	b	.L1585
.L1587:
	cmp	r3, #8
	bls	.L1585
	lsrs	r6, r6, #2
	b	.L1585
.L1656:
	movs	r6, #1
	b	.L1585
.L1592:
	movs	r3, #18
	b	.L1695
.L1590:
	ldrh	r3, [r4, #2620]
	add	r3, r3, r3, lsl #1
	asrs	r3, r3, #2
	strh	r3, [r4, #3152]	@ movhi
.L1589:
	ldrh	r3, [r4, #3206]
	cbz	r3, .L1657
	adds	r6, r6, #32
	uxth	r6, r6
.L1657:
	movw	r7, #65535
.L1595:
	ldrh	r3, [r4, #172]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1604
	cmp	r7, r3
	beq	.L1605
	strh	r7, [r4, #172]	@ movhi
.L1606:
	ldrh	r0, [r4, #172]
	movw	r5, #65535
	movs	r3, #0
	strb	r3, [r4, #180]
	cmp	r0, r5
	beq	.L1604
	bl	IsBlkInGcList
	cbz	r0, .L1609
	strh	r5, [r4, #172]	@ movhi
.L1609:
	ldrh	r2, [r4, #172]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1604
	ldr	r0, .L1699+12
	bl	make_superblock
	movs	r3, #0
	ldrh	r2, [r4, #172]
	strh	r3, [r4, #174]	@ movhi
	strb	r3, [r4, #178]
	strh	r3, [r4, #3662]	@ movhi
	ldr	r3, [r4, #72]
	ldrh	r3, [r3, r2, lsl #1]
	strh	r3, [r4, #3664]	@ movhi
.L1604:
	ldrh	r3, [r4, #172]
	ldrh	r2, [r4, #24]
	cmp	r2, r3
	beq	.L1610
	ldrh	r2, [r4, #76]
	cmp	r2, r3
	beq	.L1610
	ldrh	r2, [r4, #124]
	cmp	r2, r3
	bne	.L1611
.L1610:
	movw	r3, #65535
	strh	r3, [r4, #172]	@ movhi
.L1611:
	ldr	r5, .L1699
	mov	r4, r5
.L1647:
	ldrh	r10, [r5, #172]
	movw	r3, #65535
	cmp	r10, r3
	bne	.L1612
	movs	r3, #0
	mov	fp, r3
	str	r3, [r5, #3164]
.L1613:
	ldrh	r8, [r4, #3160]
	mov	r0, r8
	bl	List_get_gc_head_node
	uxth	r2, r0
	cmp	r2, r10
	strh	r2, [r4, #172]	@ movhi
	bne	.L1614
	movs	r3, #0
	movs	r0, #8
	strh	r3, [r4, #3160]	@ movhi
	b	.L1568
.L1596:
	movs	r3, #0
	ldrh	r2, [r4, #224]
	str	r3, [r4, #3164]
	ldrh	r3, [r4, #3152]
	cmp	r2, r3
	bls	.L1597
	ldrh	r3, [r4, #3660]
	cbnz	r3, .L1598
	ldr	r3, [r4, #2552]
	ldr	r2, [r4, #2548]
	add	r3, r3, r3, lsl #1
	cmp	r2, r3, lsr #2
	bcs	.L1599
.L1598:
	ldrh	r3, [r4, #2620]
	add	r3, r3, r3, lsl #1
	asrs	r3, r3, #2
.L1696:
	strh	r3, [r4, #3152]	@ movhi
	bl	FtlReadRefresh
	movs	r0, #0
	bl	List_get_gc_head_node
	ldr	r3, [r4, #72]
	uxth	r0, r0
	ldrh	r3, [r3, r0, lsl #1]
	cmp	r3, #4
	bls	.L1597
.L1698:
	ldrh	r0, [r4, #3660]
	b	.L1568
.L1599:
	movs	r3, #18
	b	.L1696
.L1597:
	ldrh	r0, [r4, #3660]
	cmp	r0, #0
	bne	.L1601
	ldrh	r5, [r4, #2620]
	add	r3, r5, r5, lsl #1
	asrs	r3, r3, #2
	strh	r3, [r4, #3152]	@ movhi
	bl	List_get_gc_head_node
	ldr	r3, [r4, #72]
	uxth	r0, r0
	ldrh	r1, [r4, #304]
	ldrh	r2, [r3, r0, lsl #1]
	ldrh	r3, [r4, #232]
	muls	r3, r1, r3
	add	r3, r3, r3, lsr #31
	cmp	r2, r3, asr #1
	ble	.L1602
	ldrh	r3, [r4, #224]
	subs	r5, r5, #1
	cmp	r3, r5
	blt	.L1602
	bl	FtlReadRefresh
	b	.L1698
.L1602:
	cmp	r2, #0
	bne	.L1601
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r0, [r4, #224]
	adds	r0, r0, #1
	b	.L1568
.L1700:
	.align	2
.L1699:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC124
	.word	.LANCHOR0+172
.L1605:
	ldrh	r3, [r4, #222]
	cmp	r3, r7
	beq	.L1606
	ldr	r2, [r4, #72]
	ldrh	r3, [r2, r3, lsl #1]
	cbnz	r3, .L1607
	strh	r7, [r4, #222]	@ movhi
.L1607:
	ldrh	r3, [r4, #222]
	strh	r3, [r4, #172]	@ movhi
	movw	r3, #65535
	strh	r3, [r4, #222]	@ movhi
	b	.L1606
.L1614:
	str	r0, [sp, #16]
	mov	r0, r2
	str	r2, [sp, #12]
	add	r8, r8, #1
	bl	IsBlkInGcList
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #16]
	cbz	r0, .L1615
	strh	r8, [r4, #3160]	@ movhi
	b	.L1613
.L1615:
	ldrh	lr, [r4, #232]
	uxth	r8, r8
	ldrh	r1, [r4, #302]
	uxth	r3, r3
	ldr	r0, [r4, #72]
	strh	r8, [r4, #3160]	@ movhi
	mul	r1, lr, r1
	ldrh	ip, [r0, r3, lsl #1]
	add	lr, r1, r1, lsr #31
	cmp	ip, lr, asr #1
	bgt	.L1617
	cmp	r8, #48
	bls	.L1618
	cmp	ip, #8
	bls	.L1618
	ldrh	ip, [r4, #3196]
	cmp	ip, #35
	bhi	.L1618
.L1617:
	strh	fp, [r4, #3160]	@ movhi
.L1618:
	ldrh	r3, [r0, r3, lsl #1]
	cmp	r1, r3
	bgt	.L1619
	cmp	r7, r10
	bne	.L1619
	ldrh	r1, [r4, #3160]
	cmp	r1, #3
	bhi	.L1619
	movs	r3, #0
	strh	r7, [r4, #172]	@ movhi
	strh	r3, [r4, #3160]	@ movhi
	b	.L1698
.L1619:
	cbnz	r3, .L1620
	movw	r0, #65535
	bl	decrement_vpc_count
	ldrh	r3, [r4, #3160]
	adds	r3, r3, #1
	strh	r3, [r4, #3160]	@ movhi
	b	.L1613
.L1620:
	movs	r3, #0
	strb	r3, [r4, #180]
	ldrh	r3, [r4, #24]
	cmp	r3, r2
	bne	.L1621
	movw	r2, #717
	ldr	r1, .L1701
	ldr	r0, .L1701+4
	bl	sftl_printk
.L1621:
	ldrh	r2, [r4, #172]
	ldrh	r3, [r4, #76]
	cmp	r2, r3
	bne	.L1622
	movw	r2, #718
	ldr	r1, .L1701
	ldr	r0, .L1701+4
	bl	sftl_printk
.L1622:
	ldrh	r2, [r4, #172]
	ldrh	r3, [r4, #124]
	cmp	r2, r3
	bne	.L1623
	movw	r2, #719
	ldr	r1, .L1701
	ldr	r0, .L1701+4
	bl	sftl_printk
.L1623:
	ldr	r0, .L1701+8
	bl	make_superblock
	ldrh	r1, [r4, #172]
	movs	r3, #0
	ldr	r2, [r4, #72]
	strh	r3, [r4, #3662]	@ movhi
	ldrh	r2, [r2, r1, lsl #1]
	strh	r3, [r4, #174]	@ movhi
	strb	r3, [r4, #178]
	strh	r2, [r4, #3664]	@ movhi
.L1612:
	movs	r3, #1
	str	r3, [r4, #3272]
	ldrh	r3, [r4, #302]
	str	r3, [sp, #16]
	cmp	r9, #0
	beq	.L1624
	ldr	r2, [sp, #16]
	ldrh	r3, [r4, #232]
	ldrh	r1, [r4, #172]
	muls	r3, r2, r3
	ldr	r2, [r4, #72]
	ldrh	r2, [r2, r1, lsl #1]
	subs	r3, r3, r2
	it	mi
	addmi	r3, r3, #3
	add	r6, r6, r3, asr #2
	uxth	r6, r6
.L1624:
	ldrh	r3, [r4, #174]
	ldr	r1, [sp, #16]
	adds	r2, r3, r6
	cmp	r2, r1
	itt	gt
	movgt	r2, r1
	subgt	r6, r2, r3
	mov	r3, #0
	it	gt
	uxthgt	r6, r6
.L1697:
	str	r3, [sp, #20]
	ldrh	r3, [sp, #20]
	cmp	r6, r3
	bls	.L1635
	ldr	r3, [sp, #20]
	movw	r10, #65535
	ldrh	r8, [r5, #174]
	mov	fp, #20
	ldrh	ip, [r5, #232]
	ldr	r0, [r5, #3184]
	add	r8, r8, r3
	ldr	r3, .L1701+12
	str	r3, [sp, #24]
	movs	r3, #0
	str	r3, [sp, #12]
	b	.L1636
.L1629:
	ldr	r1, [sp, #24]
	ldrh	r2, [r1, #2]!
	cmp	r2, r10
	str	r1, [sp, #24]
	beq	.L1628
	ldr	r1, [sp, #12]
	orr	r2, r8, r2, lsl #10
	mla	lr, fp, r1, r0
	str	r2, [lr, #4]
	mov	r2, r1
	adds	r2, r2, #1
	uxth	r2, r2
	str	r2, [sp, #12]
.L1628:
	adds	r3, r3, #1
.L1636:
	uxth	r2, r3
	cmp	ip, r2
	bhi	.L1629
	mov	fp, #0
	ldrb	r2, [r5, #180]	@ zero_extendqisi2
	ldr	r1, [sp, #12]
	bl	FlashReadPages
.L1630:
	ldr	r2, [sp, #12]
	uxth	r3, fp
	cmp	r2, r3
	bhi	.L1634
	ldr	r3, [sp, #20]
	adds	r3, r3, #1
	b	.L1697
.L1634:
	mov	r8, #20
	ldr	r3, [r5, #3184]
	mul	r8, r8, fp
	add	r2, r3, r8
	ldr	r3, [r3, r8]
	adds	r3, r3, #1
	beq	.L1631
	ldr	r10, [r2, #12]
	movw	r3, #61589
	ldrh	r2, [r10]
	cmp	r2, r3
	bne	.L1631
	ldr	r3, [r10, #8]
	adds	r2, r3, #1
	bne	.L1632
	movw	r2, #753
	ldr	r1, .L1701
	ldr	r0, .L1701+4
	str	r3, [sp, #24]
	bl	sftl_printk
	ldr	r3, [sp, #24]
.L1632:
	movs	r2, #0
	mov	r0, r3
	add	r1, sp, #32
	bl	log2phys
	ldr	r0, [r4, #3184]
	ldr	r3, [sp, #32]
	add	r0, r0, r8
	ldr	r2, [r0, #4]
	cmp	r2, r3
	bne	.L1631
	ldrh	r3, [r4, #3662]
	movs	r2, #20
	ldr	r1, [r4, #3168]
	str	r2, [sp, #28]
	adds	r3, r3, #1
	strh	r3, [r4, #3662]	@ movhi
	ldr	r3, [r4, #3292]
	mla	r3, r2, r1, r3
	ldr	r1, [r0, #16]
	str	r1, [r3, #16]
	str	r3, [sp, #24]
	bl	Ftl_get_new_temp_ppa
	ldr	r3, [sp, #24]
	ldr	r1, [r4, #3292]
	ldr	r2, [sp, #28]
	str	r0, [r3, #4]
	ldr	r3, [r4, #3168]
	mla	r3, r2, r3, r1
	ldr	r2, [r4, #3184]
	add	r2, r2, r8
	ldr	r1, [r2, #8]
	str	r1, [r3, #8]
	movs	r1, #1
	ldr	r2, [r2, #12]
	str	r2, [r3, #12]
	ldr	r3, [sp, #32]
	str	r3, [r10, #12]
	ldrh	r3, [r4, #124]
	strh	r3, [r10, #2]	@ movhi
	ldr	r3, [r4, #2592]
	ldr	r0, [r4, #3184]
	str	r3, [r10, #4]
	ldr	r3, [r4, #3168]
	add	r0, r0, r8
	adds	r3, r3, #1
	str	r3, [r4, #3168]
	bl	FtlGcBufAlloc
	ldrb	r2, [r4, #131]	@ zero_extendqisi2
	ldr	r3, [r4, #3168]
	cmp	r2, r3
	beq	.L1633
	ldrh	r3, [r4, #128]
	cbnz	r3, .L1631
.L1633:
	bl	Ftl_gc_temp_data_write_back
	cbz	r0, .L1631
	ldr	r3, .L1701+16
	movs	r2, #0
	movw	r1, #65535
	str	r2, [r3, #3272]
	strh	r1, [r3, #172]	@ movhi
	strh	r2, [r3, #174]	@ movhi
	ldrh	r0, [r3, #3660]
	b	.L1568
.L1631:
	add	fp, fp, #1
	b	.L1630
.L1635:
	ldrh	r3, [r5, #174]
	add	r6, r6, r3
	ldr	r3, [sp, #16]
	uxth	r6, r6
	cmp	r3, r6
	strh	r6, [r5, #174]	@ movhi
	bhi	.L1637
	ldr	r3, [r5, #3168]
	cbz	r3, .L1638
	bl	Ftl_gc_temp_data_write_back
	cbz	r0, .L1638
	movs	r3, #0
	ldrh	r0, [r5, #3660]
	str	r3, [r5, #3272]
	b	.L1568
.L1638:
	ldrh	r6, [r5, #3662]
	cbnz	r6, .L1639
	ldrh	r2, [r5, #172]
	ldr	r3, [r5, #72]
	ldrh	r3, [r3, r2, lsl #1]
	cbz	r3, .L1639
.L1640:
	ldr	r3, [r5, #2552]
	cmp	r6, r3
	bcs	.L1645
	movs	r2, #0
	add	r1, sp, #36
	mov	r0, r6
	bl	log2phys
	ldr	r0, [sp, #36]
	adds	r3, r0, #1
	beq	.L1641
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	ldrh	r3, [r5, #172]
	cmp	r3, r0
	bne	.L1641
.L1645:
	ldr	r3, [r5, #2552]
	cmp	r6, r3
	bcc	.L1639
	ldrh	r2, [r5, #172]
	movs	r1, #0
	ldr	r3, [r5, #72]
	strh	r1, [r3, r2, lsl #1]	@ movhi
	ldrh	r0, [r5, #172]
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1639:
	movw	r3, #65535
	strh	r3, [r5, #172]	@ movhi
.L1637:
	movs	r3, #0
	str	r3, [r5, #3272]
	ldrh	r3, [r5, #224]
	cmp	r3, #2
	bhi	.L1646
	ldrh	r6, [r5, #302]
	b	.L1647
.L1641:
	adds	r6, r6, #1
	b	.L1640
.L1646:
	ldrh	r0, [r5, #3660]
	cmp	r0, #0
	bne	.L1568
	adds	r0, r3, #1
	b	.L1568
.L1651:
	movs	r0, #0
	b	.L1568
.L1574:
	cmp	r9, #0
	beq	.L1649
.L1584:
	ldrh	r2, [r4, #124]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1660
.L1650:
	ldrh	r3, [r4, #222]
	movw	r2, #65535
	cmp	r3, r2
	bne	.L1660
	cmp	r7, r3
	bne	.L1660
	ldrh	r3, [r4, #172]
	cmp	r3, r7
	beq	.L1596
.L1601:
	movw	r7, #65535
.L1660:
	movs	r6, #1
	b	.L1595
.L1576:
	cmp	r9, #0
	beq	.L1649
	movw	r7, #65535
	b	.L1650
.L1702:
	.align	2
.L1701:
	.word	.LANCHOR1+568
	.word	.LC8
	.word	.LANCHOR0+172
	.word	.LANCHOR0+186
	.word	.LANCHOR0
.L1575:
	cmp	r9, #0
	bne	.L1601
	b	.L1649
	.fnend
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	1
	.global	FtlRead
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlRead, %function
FtlRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #16
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #60
	sub	sp, sp, #60
	mov	r6, r1
	mov	r10, r3
	str	r2, [sp, #4]
	bne	.L1704
	mov	r2, r3
	ldr	r1, [sp, #4]
	add	r0, r6, #256
	bl	FtlVendorPartRead
	mov	r5, r0
.L1703:
	mov	r0, r5
	add	sp, sp, #60
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1704:
	ldr	r4, .L1738
	ldr	r3, [r4, #340]
	cmp	r1, r3
	bcs	.L1727
	ldr	r2, [sp, #4]
	cmp	r2, r3
	bhi	.L1727
	adds	r2, r1, r2
	cmp	r3, r2
	str	r2, [sp, #12]
	bcc	.L1727
	ldr	r3, .L1738+4
	ldr	r5, [r3]
	adds	r7, r5, #1
	beq	.L1703
	ldrh	r5, [r4, #258]
	mov	r0, r6
	mov	r9, #0
	mov	r8, r9
	mov	r1, r5
	bl	__aeabi_uidiv
	ldr	r3, [sp, #12]
	mov	r1, r5
	str	r0, [sp, #16]
	mov	r5, r9
	subs	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	r3, [sp, #16]
	ldr	r2, [sp, #4]
	ldr	r7, [sp, #16]
	rsb	r3, r3, #1
	str	r0, [sp, #20]
	add	r3, r3, r0
	str	r9, [sp, #28]
	str	r3, [sp, #8]
	ldr	r3, [r4, #2584]
	str	r9, [sp, #24]
	add	r3, r3, r2
	ldr	r2, [sp, #8]
	str	r3, [r4, #2584]
	ldr	r3, [r4, #2556]
	add	r3, r3, r2
	str	r3, [r4, #2556]
.L1706:
	ldr	r3, [sp, #8]
	cbnz	r3, .L1723
	ldr	r3, .L1738
	ldrh	r2, [r3, #3206]
	cbnz	r2, .L1724
	ldrh	r3, [r3, #224]
	cmp	r3, #31
	bhi	.L1703
.L1724:
	movs	r1, #1
	movs	r0, #0
	bl	rk_ftl_garbage_collect
	b	.L1703
.L1723:
	movs	r2, #0
	add	r1, sp, #52
	mov	r0, r7
	bl	log2phys
	ldr	r2, [sp, #52]
	adds	r0, r2, #1
	bne	.L1707
	mov	fp, #0
.L1708:
	ldrh	r0, [r4, #258]
	cmp	fp, r0
	bcc	.L1710
.L1711:
	ldr	r3, [sp, #8]
	adds	r7, r7, #1
	subs	r3, r3, #1
	str	r3, [sp, #8]
	beq	.L1715
	ldrh	r3, [r4, #232]
	cmp	r8, r3, lsl #2
	bne	.L1706
.L1715:
	cmp	r8, #0
	beq	.L1706
	movs	r2, #0
	mov	r1, r8
	ldr	r0, [r4, #3288]
	mov	fp, #0
	bl	FlashReadPages
	lsl	r3, r9, #9
	str	r3, [sp, #40]
	ldr	r3, [sp, #24]
	lsls	r3, r3, #9
	str	r3, [sp, #32]
	ldr	r3, [sp, #28]
	lsls	r3, r3, #9
	str	r3, [sp, #36]
.L1722:
	movs	r3, #20
	ldr	r2, [r4, #3288]
	mul	r3, r3, fp
	ldr	r0, [sp, #16]
	add	r2, r2, r3
	ldr	r1, [r2, #16]
	cmp	r0, r1
	bne	.L1717
	ldr	r1, [r2, #8]
	ldr	r2, [r4, #3312]
	cmp	r1, r2
	bne	.L1718
	str	r3, [sp, #44]
	mov	r0, r10
	ldr	r3, [sp, #32]
	ldr	r2, [sp, #36]
	add	r1, r1, r3
.L1737:
	bl	ftl_memcpy
	ldr	r3, [sp, #44]
.L1718:
	ldr	ip, [r4, #3288]
	add	r2, ip, r3
	ldr	r0, [r2, #12]
	ldr	r1, [r2, #16]
	ldr	r0, [r0, #8]
	cmp	r1, r0
	ittt	ne
	ldrne	r1, [r4, #2712]
	addne	r1, r1, #1
	strne	r1, [r4, #2712]
	ldr	r0, [ip, r3]
	adds	r1, r0, #1
	itttt	eq
	ldreq	r1, [r4, #2712]
	moveq	r5, r0
	addeq	r1, r1, #1
	streq	r1, [r4, #2712]
	ldr	r3, [ip, r3]
	cmp	r3, #256
	bne	.L1721
	ldr	r0, [r2, #4]
	ubfx	r0, r0, #10, #16
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L1721:
	add	fp, fp, #1
	cmp	r8, fp
	bne	.L1722
	mov	r8, #0
	b	.L1706
.L1710:
	mla	r0, r0, r7, fp
	cmp	r6, r0
	bhi	.L1709
	ldr	r3, [sp, #12]
	cmp	r3, r0
	bls	.L1709
	subs	r0, r0, r6
	mov	r1, #512
	add	r0, r10, r0, lsl #9
	bl	__memzero
.L1709:
	add	fp, fp, #1
	b	.L1708
.L1707:
	ldr	r3, [r4, #3288]
	movs	r1, #20
	ldrh	fp, [r4, #258]
	mla	r3, r1, r8, r3
	str	r2, [r3, #4]
	ldr	r2, [sp, #16]
	cmp	r7, r2
	bne	.L1712
	ldr	r2, [r4, #3312]
	mov	r1, fp
	mov	r0, r6
	str	r3, [sp, #32]
	str	r2, [r3, #8]
	bl	__aeabi_uidivmod
	sub	r2, fp, r1
	str	r1, [sp, #24]
	ldr	r1, [sp, #4]
	mov	r3, r2
	cmp	r2, r1
	it	cs
	movcs	r3, r1
	cmp	r3, fp
	str	r3, [sp, #28]
	ldr	r3, [sp, #32]
	bne	.L1713
	str	r10, [r3, #8]
.L1713:
	ldrh	r2, [r4, #312]
	ldr	r1, [r4, #3336]
	str	r7, [r3, #16]
	mul	r2, r8, r2
	add	r8, r8, #1
	bic	r2, r2, #3
	add	r2, r2, r1
	str	r2, [r3, #12]
	b	.L1711
.L1712:
	ldr	r2, [sp, #20]
	cmp	r7, r2
	bne	.L1714
	ldr	r2, [r4, #3316]
	ldr	r1, [sp, #12]
	str	r2, [r3, #8]
	mul	r2, fp, r7
	sub	r9, r1, r2
	cmp	fp, r9
	bne	.L1713
.L1736:
	subs	r2, r2, r6
	add	r2, r10, r2, lsl #9
	str	r2, [r3, #8]
	b	.L1713
.L1714:
	mul	r2, r7, fp
	b	.L1736
.L1717:
	ldr	r0, [sp, #20]
	cmp	r0, r1
	bne	.L1718
	ldr	r1, [r2, #8]
	ldr	r2, [r4, #3316]
	cmp	r1, r2
	bne	.L1718
	str	r3, [sp, #44]
	ldrh	r0, [r4, #258]
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #40]
	muls	r0, r3, r0
	subs	r0, r0, r6
	add	r0, r10, r0, lsl #9
	b	.L1737
.L1727:
	mov	r5, #-1
	b	.L1703
.L1739:
	.align	2
.L1738:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	FtlRead, .-FtlRead
	.align	1
	.global	sftl_read
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_read, %function
sftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, r2
	mov	r2, r1
	mov	r1, r0
	movs	r0, #0
	b	FtlRead
	.fnend
	.size	sftl_read, .-sftl_read
	.align	1
	.global	FtlWrite
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlWrite, %function
FtlWrite:
	.fnstart
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #16
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #68
	sub	sp, sp, #68
	mov	r10, r1
	str	r2, [sp, #8]
	str	r3, [sp, #12]
	bne	.L1742
	mov	r2, r3
	ldr	r1, [sp, #8]
	add	r0, r10, #256
	bl	FtlVendorPartWrite
.L1741:
	add	sp, sp, #68
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1742:
	ldr	r5, .L1795
	ldr	r3, [r5, #340]
	cmp	r1, r3
	bcs	.L1781
	ldr	r2, [sp, #8]
	cmp	r2, r3
	bhi	.L1781
	adds	r6, r1, r2
	cmp	r3, r6
	bcc	.L1781
	ldr	r4, .L1795+4
	ldr	r0, [r4]
	adds	r7, r0, #1
	beq	.L1741
	ldrh	r7, [r5, #258]
	mov	r3, #2048
	str	r3, [r5, #3668]
	mov	r0, r10
	mov	r1, r7
	bl	__aeabi_uidiv
	mov	r1, r7
	str	r0, [sp, #4]
	subs	r0, r6, #1
	bl	__aeabi_uidiv
	ldr	r2, [sp, #4]
	str	r0, [sp, #20]
	subs	r3, r0, r2
	ldr	r2, [sp, #8]
	str	r3, [sp, #24]
	adds	r3, r3, #1
	str	r3, [sp]
	ldr	r3, [r5, #2580]
	add	r3, r3, r2
	ldr	r2, [sp]
	str	r3, [r5, #2580]
	ldr	r3, [r5, #2564]
	add	r3, r3, r2
	str	r3, [r5, #2564]
	ldr	r3, [r4, #8]
	cbz	r3, .L1783
	ldrh	r2, [r5, #28]
	add	r3, r5, #24
	adds	r5, r5, #76
	cmp	r2, #0
	it	ne
	movne	r5, r3
.L1744:
	ldr	r7, [sp, #4]
	ldr	r4, .L1795
.L1745:
	ldr	r3, [sp]
	cbnz	r3, .L1774
	mov	r0, r3
	ldr	r1, [sp, #24]
	bl	rk_ftl_garbage_collect
	ldrh	r3, [r4, #224]
	cmp	r3, #5
	bls	.L1775
.L1777:
	movs	r0, #0
	b	.L1741
.L1783:
	adds	r5, r5, #24
	b	.L1744
.L1774:
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	ldrh	r3, [r4, #232]
	cmp	r2, r3
	bcc	.L1746
	movw	r2, #1041
	ldr	r1, .L1795+8
	ldr	r0, .L1795+12
	bl	sftl_printk
.L1746:
	ldrh	r2, [r5, #4]
	cbnz	r2, .L1747
	ldr	r3, .L1795+16
	ldr	r6, .L1795+4
	cmp	r5, r3
	bne	.L1748
	ldrh	r8, [r4, #80]
	cmp	r8, #0
	bne	.L1749
	add	r0, r5, #52
	bl	allocate_new_data_superblock
	str	r8, [r6, #8]
.L1749:
	ldr	r5, .L1795+16
	ldr	r0, .L1795+16
	bl	allocate_new_data_superblock
	ldr	r2, [r6, #8]
	add	r3, r5, #52
	cmp	r2, #0
	it	ne
	movne	r5, r3
.L1750:
	ldrh	r3, [r5, #4]
	cbnz	r3, .L1747
	mov	r0, r5
	bl	allocate_new_data_superblock
.L1747:
	ldrh	r2, [r5, #4]
	ldr	r3, [sp]
	cmp	r2, r3
	it	cs
	movcs	r2, r3
	ldrb	r3, [r5, #7]	@ zero_extendqisi2
	lsls	r3, r3, #2
	cmp	r2, r3
	it	cs
	movcs	r2, r3
	ldrh	r3, [r4, #232]
	str	r2, [sp, #36]
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	cmp	r2, r3
	bcc	.L1751
	movw	r2, #1074
	ldr	r1, .L1795+8
	ldr	r0, .L1795+12
	bl	sftl_printk
.L1751:
	mov	fp, #0
.L1752:
	ldr	r3, [sp, #36]
	cmp	fp, r3
	bne	.L1772
.L1753:
	mov	r3, r5
	movs	r2, #0
	mov	r1, fp
	ldr	r0, [r4, #3296]
	bl	FtlProgPages
	ldr	r3, [sp]
	cmp	fp, r3
	bls	.L1773
	mov	r2, #1152
	ldr	r1, .L1795+8
	ldr	r0, .L1795+12
	bl	sftl_printk
.L1773:
	ldr	r3, [sp]
	sub	r3, r3, fp
	str	r3, [sp]
	b	.L1745
.L1748:
	str	r2, [r6, #8]
	ldrh	r2, [r4, #28]
	cbnz	r2, .L1786
	mov	r0, r5
	bl	allocate_new_data_superblock
	b	.L1750
.L1786:
	mov	r5, r3
	b	.L1747
.L1772:
	ldrh	r3, [r5, #4]
	cmp	r3, #0
	beq	.L1753
	movs	r2, #0
	add	r1, sp, #40
	mov	r0, r7
	mov	r8, #20
	bl	log2phys
	mov	r0, r5
	mul	r8, r8, fp
	bl	get_new_active_ppa
	ldrh	r2, [r4, #312]
	ldr	r3, [r4, #3296]
	mul	r1, r2, fp
	add	r3, r3, r8
	str	r0, [r3, #4]
	str	r7, [r3, #16]
	bic	r1, r1, #3
	str	r1, [sp, #28]
	ldr	r0, [sp, #28]
	ldr	r1, [r4, #3336]
	add	r9, r1, r0
	str	r1, [sp, #32]
	str	r9, [r3, #12]
	movs	r1, #0
	mov	r0, r9
	bl	ftl_memset
	ldr	r3, [sp, #4]
	cmp	r7, r3
	beq	.L1754
	ldr	r3, [sp, #20]
	cmp	r7, r3
	bne	.L1791
	ldrh	r2, [r4, #258]
	ldr	r3, [sp, #8]
	smulbb	r2, r2, r7
	add	r6, r10, r3
	movs	r3, #0
	str	r3, [sp, #16]
	subs	r6, r6, r2
	uxth	r6, r6
	b	.L1757
.L1754:
	ldrh	r6, [r4, #258]
	mov	r0, r10
	mov	r1, r6
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #8]
	subs	r6, r6, r1
	str	r1, [sp, #16]
	cmp	r6, r3
	it	cs
	movcs	r6, r3
.L1757:
	ldrh	r3, [r4, #258]
	cmp	r6, r3
	ldr	r3, [sp, #4]
	bne	.L1758
	cmp	r7, r3
	ldr	r3, [r4, #3296]
	itte	ne
	mulne	r6, r6, r7
	ldrne	r2, [sp, #12]
	ldreq	r2, [sp, #12]
	add	r3, r3, r8
	itett	ne
	subne	r6, r6, r10
	streq	r2, [r3, #8]
	addne	r6, r2, r6, lsl #9
	strne	r6, [r3, #8]
.L1760:
	ldrb	r2, [r5, #6]	@ zero_extendqisi2
	ldrh	r3, [r4, #232]
	cmp	r2, r3
	bcc	.L1769
	movw	r2, #1143
	ldr	r1, .L1795+8
	ldr	r0, .L1795+12
	bl	sftl_printk
.L1769:
	ldr	r2, [sp, #32]
	movw	r3, #61589
	ldr	r1, [sp, #28]
	add	fp, fp, #1
	strh	r3, [r2, r1]	@ movhi
	ldr	r3, [r4, #2592]
	str	r7, [r9, #8]
	adds	r7, r7, #1
	str	r3, [r9, #4]
	adds	r3, r3, #1
	adds	r2, r3, #1
	it	eq
	moveq	r3, #0
	str	r3, [r4, #2592]
	ldr	r3, [sp, #40]
	str	r3, [r9, #12]
	ldrh	r3, [r5]
	strh	r3, [r9, #2]	@ movhi
	b	.L1752
.L1758:
	cmp	r7, r3
	ldr	r3, [r4, #3296]
	ite	eq
	ldreq	r2, [r4, #3312]
	ldrne	r2, [r4, #3316]
	add	r3, r3, r8
	str	r2, [r3, #8]
	ldr	r3, [sp, #40]
	adds	r0, r3, #1
	beq	.L1763
	str	r3, [sp, #48]
	movs	r1, #1
	ldr	r3, [r4, #3296]
	add	r0, sp, #44
	str	r7, [sp, #60]
	add	r3, r3, r8
	ldr	r2, [r3, #8]
	ldr	r3, [r3, #12]
	str	r2, [sp, #52]
	movs	r2, #0
	str	r3, [sp, #56]
	bl	FlashReadPages
	ldr	r3, [sp, #44]
	adds	r1, r3, #1
	bne	.L1764
	ldr	r2, [r4, #2712]
	ldr	r0, .L1795+20
	adds	r2, r2, #1
	str	r2, [r4, #2712]
	mov	r2, r7
	ldr	r1, [r9, #8]
	bl	sftl_printk
.L1767:
	ldr	r3, [sp, #4]
	lsls	r2, r6, #9
	cmp	r7, r3
	bne	.L1768
	ldr	r3, [r4, #3296]
	ldr	r1, [sp, #12]
	add	r3, r3, r8
	ldr	r0, [r3, #8]
	ldr	r3, [sp, #16]
	add	r0, r0, r3, lsl #9
.L1793:
	bl	ftl_memcpy
	b	.L1760
.L1796:
	.align	2
.L1795:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR1+591
	.word	.LC8
	.word	.LANCHOR0+24
	.word	.LC125
.L1764:
	ldr	r3, [r9, #8]
	cmp	r7, r3
	beq	.L1766
	ldr	r3, [r4, #2712]
	mov	r2, r7
	ldr	r0, .L1797
	adds	r3, r3, #1
	str	r3, [r4, #2712]
	ldr	r1, [r9, #8]
	bl	sftl_printk
.L1766:
	ldr	r3, [r9, #8]
	cmp	r7, r3
	beq	.L1767
	mov	r2, #1128
	ldr	r1, .L1797+4
	ldr	r0, .L1797+8
	bl	sftl_printk
	b	.L1767
.L1763:
	ldr	r3, [r4, #3296]
	movs	r1, #0
	ldrh	r2, [r4, #310]
	add	r3, r3, r8
	ldr	r0, [r3, #8]
	bl	ftl_memset
	b	.L1767
.L1768:
	ldrh	r1, [r4, #258]
	ldr	r0, [sp, #12]
	ldr	r3, [r4, #3296]
	muls	r1, r7, r1
	add	r3, r3, r8
	sub	r1, r1, r10
	add	r1, r0, r1, lsl #9
	ldr	r0, [r3, #8]
	b	.L1793
.L1791:
	ldr	r3, [r4, #3296]
	ldr	r2, [sp, #12]
	add	r8, r8, r3
	ldrh	r3, [r4, #258]
	muls	r3, r7, r3
	sub	r3, r3, r10
	add	r3, r2, r3, lsl #9
	str	r3, [r8, #8]
	b	.L1760
.L1775:
	mov	r5, #256
	movw	r6, #65535
.L1778:
	ldrh	r3, [r4, #172]
	cmp	r3, r6
	bne	.L1776
	ldrh	r3, [r4, #222]
	cmp	r3, r6
	bne	.L1776
	movs	r0, #0
	bl	List_get_gc_head_node
	uxth	r0, r0
	bl	FtlGcRefreshBlock
.L1776:
	movs	r1, #1
	movs	r3, #128
	mov	r0, r1
	strh	r3, [r4, #3154]	@ movhi
	strh	r3, [r4, #3152]	@ movhi
	bl	rk_ftl_garbage_collect
	movs	r1, #1
	movs	r0, #0
	bl	rk_ftl_garbage_collect
	ldrh	r3, [r4, #224]
	cmp	r3, #2
	bhi	.L1777
	subs	r5, r5, #1
	bne	.L1778
	b	.L1777
.L1781:
	mov	r0, #-1
	b	.L1741
.L1798:
	.align	2
.L1797:
	.word	.LC126
	.word	.LANCHOR1+591
	.word	.LC8
	.fnend
	.size	FtlWrite, .-FtlWrite
	.align	1
	.global	sftl_gc
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_gc, %function
sftl_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	movs	r1, #1
	mov	r0, r1
	b	rk_ftl_garbage_collect
	.fnend
	.size	sftl_gc, .-sftl_gc
	.align	1
	.global	FtlLoadSysInfo
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #16
	movs	r1, #0
	ldr	r4, .L1830
	ldr	r3, [r4, #3300]
	ldrh	r2, [r4, #240]
	ldr	r0, [r4, #72]
	str	r3, [r4, #3460]
	ldr	r3, [r4, #3332]
	lsls	r2, r2, #1
	str	r3, [r4, #3464]
	bl	ftl_memset
	ldrh	r0, [r4, #2624]
	movw	r3, #65535
	cmp	r0, r3
	bne	.L1801
.L1812:
	mov	r0, #-1
.L1800:
	add	sp, sp, #16
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, pc}
.L1801:
	movs	r1, #1
	ldr	r8, .L1830+20
	bl	FtlGetLastWrittenPage
	ldr	r9, .L1830+4
	addw	r7, r4, #3452
	ldrsh	r6, [r4, #2624]
	sxth	r5, r0
	adds	r0, r0, #1
	strh	r0, [r4, #2626]	@ movhi
.L1803:
	cmp	r5, #0
	bge	.L1809
	movw	r2, #1467
	ldr	r1, .L1830+4
	ldr	r0, .L1830+8
	bl	sftl_printk
	b	.L1808
.L1809:
	orr	r3, r5, r6, lsl #10
	movs	r2, #1
	mov	r1, r2
	str	r3, [r4, #3456]
	mov	r0, r7
	ldr	r3, [r4, #3300]
	str	r3, [r4, #3460]
	bl	FlashReadPages
	ldr	r3, [r4, #3464]
	ldr	r10, [r3, #12]
	cmp	r10, #0
	beq	.L1804
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L1804
	ldrh	r1, [r4, #310]
	ldr	r0, [r4, #3460]
	bl	js_hash
	cmp	r10, r0
	beq	.L1804
	ldrh	r3, [r4, #2628]
	mov	r2, r6
	str	r0, [sp, #8]
	mov	r1, r9
	str	r10, [sp, #4]
	str	r3, [sp]
	mov	r3, r5
	ldr	r0, .L1830+12
	bl	sftl_printk
	cbnz	r5, .L1805
	ldrh	r3, [r4, #2628]
	cmp	r6, r3
	beq	.L1805
	ldrh	r5, [r4, #304]
	sxth	r6, r3
.L1807:
	subs	r5, r5, #1
	sxth	r5, r5
	b	.L1803
.L1805:
	mov	r3, #-1
	str	r3, [r4, #3452]
.L1804:
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L1807
	ldr	r3, [r4, #3300]
	ldr	r3, [r3]
	cmp	r3, r8
	bne	.L1807
	ldr	r3, [r4, #3332]
	ldrh	r2, [r3]
	movw	r3, #61604
	cmp	r2, r3
	bne	.L1807
.L1808:
	ldrh	r3, [r4, #240]
	ldrh	r2, [r4, #310]
	adds	r3, r3, #24
	cmp	r2, r3, lsl #1
	bcs	.L1811
	movw	r2, #1469
	ldr	r1, .L1830+4
	ldr	r0, .L1830+8
	bl	sftl_printk
.L1811:
	movs	r2, #48
	ldr	r1, [r4, #3460]
	ldr	r0, .L1830+16
	bl	ftl_memcpy
	ldrh	r2, [r4, #240]
	ldr	r1, [r4, #3460]
	ldr	r0, [r4, #72]
	lsls	r2, r2, #1
	adds	r1, r1, #48
	bl	ftl_memcpy
	ldr	r2, [r4, #2468]
	ldr	r3, .L1830+20
	cmp	r2, r3
	bne	.L1812
	ldrb	r2, [r4, #2478]	@ zero_extendqisi2
	ldrh	r3, [r4, #254]
	ldrh	r5, [r4, #2476]
	cmp	r2, r3
	strh	r5, [r4, #2630]	@ movhi
	bne	.L1812
	ldrh	r3, [r4, #302]
	ldrh	r2, [r4, #258]
	ldr	r6, [r4, #244]
	ldrh	r0, [r4, #358]
	muls	r3, r5, r3
	str	r5, [r4, #3672]
	ldrh	r1, [r4, #232]
	subs	r0, r6, r0
	subs	r0, r0, r5
	str	r3, [r4, #2552]
	muls	r3, r2, r3
	str	r3, [r4, #340]
	bl	__aeabi_uidiv
	cmp	r5, r6
	strh	r0, [r4, #2620]	@ movhi
	bls	.L1813
	movw	r2, #1491
	ldr	r1, .L1830+4
	ldr	r0, .L1830+8
	bl	sftl_printk
.L1813:
	ldrh	r3, [r4, #2484]
	ldrh	r1, [r4, #2482]
	lsrs	r2, r3, #6
	and	r3, r3, #63
	strb	r3, [r4, #30]
	strh	r2, [r4, #26]	@ movhi
	ldrh	r2, [r4, #2486]
	ldrb	r3, [r4, #2479]	@ zero_extendqisi2
	strh	r1, [r4, #24]	@ movhi
	strh	r2, [r4, #76]	@ movhi
	ldrh	r2, [r4, #2488]
	strb	r3, [r4, #32]
	movw	r3, #65535
	strh	r3, [r4, #172]	@ movhi
	movs	r3, #0
	strh	r3, [r4, #174]	@ movhi
	lsrs	r0, r2, #6
	and	r2, r2, #63
	strb	r2, [r4, #82]
	ldrb	r2, [r4, #2480]	@ zero_extendqisi2
	strh	r0, [r4, #78]	@ movhi
	strb	r3, [r4, #178]
	strb	r2, [r4, #84]
	ldrh	r2, [r4, #2490]
	strb	r3, [r4, #180]
	str	r3, [r4, #2576]
	strh	r2, [r4, #124]	@ movhi
	ldrh	r2, [r4, #2492]
	str	r3, [r4, #2564]
	str	r3, [r4, #2556]
	str	r3, [r4, #2572]
	lsrs	r0, r2, #6
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
	it	hi
	strhi	r3, [r4, #2588]
	ldr	r3, [r4, #2504]
	cmp	r3, r2
	it	hi
	strhi	r3, [r4, #2592]
	movw	r3, #65535
	cmp	r1, r3
	beq	.L1816
	ldr	r0, .L1830+24
	bl	make_superblock
.L1816:
	ldrh	r2, [r4, #76]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1817
	ldr	r0, .L1830+28
	bl	make_superblock
.L1817:
	ldrh	r2, [r4, #124]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1818
	ldr	r0, .L1830+32
	bl	make_superblock
.L1818:
	ldrh	r2, [r4, #172]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1819
	ldr	r0, .L1830+36
	bl	make_superblock
.L1819:
	movs	r0, #0
	b	.L1800
.L1831:
	.align	2
.L1830:
	.word	.LANCHOR0
	.word	.LANCHOR1+600
	.word	.LC8
	.word	.LC127
	.word	.LANCHOR0+2468
	.word	1179929683
	.word	.LANCHOR0+24
	.word	.LANCHOR0+76
	.word	.LANCHOR0+124
	.word	.LANCHOR0+172
	.fnend
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	1
	.global	FtlMapTblRecovery
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #36
	sub	sp, sp, #36
	ldr	r3, [r0, #24]
	mov	r5, r0
	movs	r1, #0
	ldrh	r10, [r0, #6]
	str	r3, [sp, #8]
	ldr	r3, [r0, #16]
	ldr	r4, .L1873
	ldr	r9, [r0, #12]
	lsl	r2, r10, #2
	str	r3, [sp, #16]
	ldrh	r3, [r0, #8]
	ldr	r0, [sp, #8]
	str	r3, [sp, #12]
	bl	ftl_memset
	ldr	r3, [r4, #3300]
	movs	r2, #1
	ldr	fp, [r4, #3332]
	str	r2, [r5, #36]
	str	r3, [r4, #3460]
	movw	r3, #65535
	str	fp, [r4, #3464]
	strh	r3, [r5]	@ movhi
	strh	r3, [r5, #2]	@ movhi
	movs	r3, #0
	mov	r8, r3
	str	r3, [r5, #32]
	str	r3, [r5, #28]
.L1833:
	ldr	r3, [sp, #12]
	sxth	r6, r8
	cmp	r6, r3
	bge	.L1852
	ldr	r3, [sp, #12]
	lsls	r7, r6, #1
	subs	r3, r3, #1
	cmp	r6, r3
	bne	.L1834
	movs	r1, #1
	ldrh	r0, [r9, r6, lsl #1]
	bl	FtlGetLastWrittenPage
	sxth	r3, r0
	adds	r0, r0, #1
	strh	r8, [r5]	@ movhi
	add	r7, r7, r9
	str	r3, [sp, #12]
	mov	r8, #0
	ldr	r3, [sp, #16]
	strh	r0, [r5, #2]	@ movhi
	ldr	r3, [r3, r6, lsl #2]
	str	r3, [r5, #28]
.L1835:
	ldr	r3, [sp, #12]
	sxth	r9, r8
	adds	r2, r3, #1
	cmp	r9, r2
	blt	.L1838
.L1852:
	mov	r0, r5
	bl	ftl_free_no_use_map_blk
	ldrh	r2, [r5, #2]
	ldrh	r3, [r4, #304]
	cmp	r2, r3
	bne	.L1840
	mov	r0, r5
	bl	ftl_map_blk_alloc_new_blk
.L1840:
	mov	r0, r5
	bl	ftl_map_blk_gc
	mov	r0, r5
	bl	ftl_map_blk_gc
	movs	r0, #0
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1838:
	ldrh	r2, [r7]
	ldr	r0, .L1873+4
	orr	r2, r9, r2, lsl #10
	str	r2, [r4, #3456]
	movs	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r2, [r4, #3464]
	ldr	r2, [r2, #12]
	str	r2, [sp, #16]
	cbz	r2, .L1836
	ldr	r1, [r4, #3452]
	adds	r1, r1, #1
	beq	.L1836
	ldrh	r1, [r4, #310]
	ldr	r0, [r4, #3460]
	bl	js_hash
	ldr	r2, [sp, #16]
	cmp	r2, r0
	beq	.L1836
	str	r0, [sp, #4]
	mov	r3, r9
	str	r2, [sp]
	mov	r2, r6
	ldr	r1, .L1873+8
	ldr	r0, .L1873+12
	bl	sftl_printk
	mov	r3, #-1
	str	r3, [r4, #3452]
.L1836:
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L1837
	ldrh	r3, [fp, #8]
	cmp	r10, r3
	bls	.L1837
	ldrh	r1, [fp]
	ldrh	r2, [r5, #4]
	cmp	r1, r2
	ittt	eq
	ldreq	r2, [r4, #3456]
	ldreq	r1, [sp, #8]
	streq	r2, [r1, r3, lsl #2]
.L1837:
	add	r8, r8, #1
	b	.L1835
.L1834:
	ldr	r3, [r4, #3300]
	ldr	r0, .L1873+4
	str	r3, [r4, #3460]
	add	r3, r9, r7
	ldrh	r2, [r9, r6, lsl #1]
	str	r3, [sp, #20]
	ldrh	r3, [r4, #304]
	subs	r3, r3, #1
	orr	r3, r3, r2, lsl #10
	movs	r2, #1
	mov	r1, r2
	str	r3, [r4, #3456]
	bl	FlashReadPages
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L1854
	ldrh	r2, [fp]
	ldrh	r3, [r5, #4]
	cmp	r2, r3
	bne	.L1854
	ldrh	r2, [fp, #8]
	movw	r3, #64245
	cmp	r2, r3
	beq	.L1842
.L1854:
	movs	r7, #0
.L1843:
	ldrh	r2, [r4, #304]
	sxth	r3, r7
	cmp	r3, r2
	bge	.L1850
	ldr	r2, [sp, #20]
	ldr	r0, .L1873+4
	str	r3, [sp, #28]
	ldrh	r2, [r2]
	orr	r2, r3, r2, lsl #10
	str	r2, [r4, #3456]
	movs	r2, #1
	mov	r1, r2
	bl	FlashReadPages
	ldr	r2, [r4, #3464]
	ldr	r2, [r2, #12]
	str	r2, [sp, #24]
	cbz	r2, .L1847
	ldr	r1, [r4, #3452]
	adds	r1, r1, #1
	beq	.L1847
	ldrh	r1, [r4, #310]
	ldr	r0, [r4, #3460]
	bl	js_hash
	ldr	r2, [sp, #24]
	cmp	r2, r0
	beq	.L1847
	str	r0, [sp, #4]
	str	r2, [sp]
	mov	r2, r6
	ldr	r3, [sp, #28]
	ldr	r1, .L1873+8
	ldr	r0, .L1873+16
	bl	sftl_printk
	mov	r3, #-1
	str	r3, [r4, #3452]
.L1847:
	ldr	r3, [r4, #3452]
	adds	r3, r3, #1
	beq	.L1848
	ldrh	r3, [fp, #8]
	cmp	r10, r3
	bls	.L1848
	ldrh	r1, [fp]
	ldrh	r2, [r5, #4]
	cmp	r1, r2
	ittt	eq
	ldreq	r2, [r4, #3456]
	ldreq	r1, [sp, #8]
	streq	r2, [r1, r3, lsl #2]
.L1848:
	adds	r7, r7, #1
	b	.L1843
.L1842:
	movs	r1, #0
	movs	r0, #4
.L1844:
	ldrh	r2, [r4, #304]
	sxth	r3, r1
	subs	r2, r2, #1
	cmp	r3, r2
	blt	.L1846
.L1850:
	add	r8, r8, #1
	b	.L1833
.L1846:
	ldr	r6, [r4, #3300]
	ldr	r2, [r6, r3, lsl #3]
	uxth	r7, r2
	cmp	r10, r7
	bls	.L1845
	add	r3, r0, r3, lsl #3
	ldr	r3, [r6, r3]
	ldr	r6, [sp, #8]
	str	r3, [r6, r7, lsl #2]
.L1845:
	adds	r1, r1, #1
	b	.L1844
.L1874:
	.align	2
.L1873:
	.word	.LANCHOR0
	.word	.LANCHOR0+3452
	.word	.LANCHOR1+615
	.word	.LC128
	.word	.LC129
	.fnend
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	1
	.global	FtlLoadVonderInfo
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r0, .L1876
	push	{r3, lr}
	.save {r3, lr}
	ldrh	r3, [r0, #320]
	strh	r3, [r0, #3618]	@ movhi
	movw	r3, #61574
	strh	r3, [r0, #3612]	@ movhi
	ldrh	r3, [r0, #344]
	strh	r3, [r0, #3616]	@ movhi
	ldrh	r3, [r0, #322]
	strh	r3, [r0, #3614]	@ movhi
	ldr	r3, [r0, #348]
	addw	r0, r0, #3608
	str	r3, [r0, #12]
	ldr	r3, [r0, #-240]
	str	r3, [r0, #16]
	ldr	r3, [r0, #-244]
	str	r3, [r0, #20]
	ldr	r3, [r0, #-236]
	str	r3, [r0, #24]
	bl	FtlMapTblRecovery
	movs	r0, #0
	pop	{r3, pc}
.L1877:
	.align	2
.L1876:
	.word	.LANCHOR0
	.fnend
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	1
	.global	FtlLoadMapInfo
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlLoadMapInfo, %function
FtlLoadMapInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, lr}
	.save {r3, lr}
	bl	FtlL2PDataInit
	ldr	r0, .L1879
	bl	FtlMapTblRecovery
	movs	r0, #0
	pop	{r3, pc}
.L1880:
	.align	2
.L1879:
	.word	.LANCHOR0+3392
	.fnend
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	1
	.global	FtlSysBlkInit
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	movs	r3, #0
	ldr	r4, .L1897
	ldrh	r0, [r4, #236]
	strh	r3, [r4, #3472]	@ movhi
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldrh	r2, [r4, #2624]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1882
.L1884:
	mov	r5, #-1
.L1881:
	mov	r0, r5
	pop	{r4, r5, r6, pc}
.L1882:
	bl	FtlLoadSysInfo
	mov	r5, r0
	cmp	r0, #0
	bne	.L1884
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	movs	r0, #1
	bl	FtlUpdateVaildLpn
	ldrh	r1, [r4, #338]
	mov	r3, r5
	ldr	r2, [r4, #2540]
	movs	r0, #12
.L1885:
	cmp	r3, r1
	bge	.L1890
	mla	r6, r0, r3, r2
	ldr	r6, [r6, #4]
	cmp	r6, #0
	bge	.L1886
.L1890:
	ldrh	r2, [r4, #2496]
	cmp	r3, r1
	add	r2, r2, #1
	strh	r2, [r4, #2496]	@ movhi
	bge	.L1896
.L1887:
	ldr	r0, .L1897+4
	bl	FtlSuperblockPowerLostFix
	ldr	r0, .L1897+8
	bl	FtlSuperblockPowerLostFix
	ldrh	r1, [r4, #24]
	ldr	r2, [r4, #72]
	ldrh	r0, [r4, #28]
	ldrh	r3, [r2, r1, lsl #1]
	subs	r3, r3, r0
	strh	r3, [r2, r1, lsl #1]	@ movhi
	ldrh	r3, [r4, #302]
	ldr	r1, [r4, #72]
	ldrh	r0, [r4, #76]
	strh	r3, [r4, #26]	@ movhi
	movs	r3, #0
	strb	r3, [r4, #30]
	strh	r3, [r4, #28]	@ movhi
	ldrh	r6, [r4, #80]
	ldrh	r2, [r1, r0, lsl #1]
	subs	r2, r2, r6
	strh	r2, [r1, r0, lsl #1]	@ movhi
	strb	r3, [r4, #82]
	strh	r3, [r4, #80]	@ movhi
	ldrh	r3, [r4, #2498]
	ldrh	r2, [r4, #302]
	adds	r3, r3, #1
	strh	r2, [r4, #78]	@ movhi
	strh	r3, [r4, #2498]	@ movhi
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
	b	.L1891
.L1886:
	adds	r3, r3, #1
	b	.L1885
.L1896:
	ldrh	r3, [r4, #3472]
	cmp	r3, #0
	bne	.L1887
.L1891:
	ldrh	r0, [r4, #24]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L1892
	ldrh	r3, [r4, #28]
	cbnz	r3, .L1892
	ldrh	r3, [r4, #80]
	cbnz	r3, .L1892
	bl	FtlGcRefreshBlock
	ldrh	r0, [r4, #76]
	bl	FtlGcRefreshBlock
	bl	FtlVpcTblFlush
	ldr	r0, .L1897+4
	bl	allocate_new_data_superblock
	ldr	r0, .L1897+8
	bl	allocate_new_data_superblock
.L1892:
	bl	FtlVpcCheckAndModify
	b	.L1881
.L1898:
	.align	2
.L1897:
	.word	.LANCHOR0
	.word	.LANCHOR0+24
	.word	.LANCHOR0+76
	.fnend
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	1
	.global	ftl_low_format
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	ftl_low_format, %function
ftl_low_format:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	movs	r3, #0
	ldr	r4, .L1923
	ldrh	r0, [r4, #236]
	str	r3, [r4, #2588]
	str	r3, [r4, #2592]
	str	r3, [r4, #2604]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbz	r0, .L1900
	bl	FtlMakeBbt
.L1900:
	ldr	r0, .L1923+4
	movs	r2, #0
.L1901:
	ldrh	r1, [r4, #258]
	uxth	r3, r2
	adds	r2, r2, #1
	cmp	r3, r1, lsl #7
	blt	.L1902
	ldrh	r6, [r4, #240]
	movs	r5, #0
.L1903:
	ldrh	r3, [r4, #242]
	cmp	r3, r6
	bhi	.L1904
	ldrh	r1, [r4, #232]
	subs	r3, r5, #3
	cmp	r3, r1, lsl #1
	bge	.L1905
.L1909:
	movs	r5, #0
	mov	r6, r5
.L1906:
	ldrh	r3, [r4, #240]
	uxth	r0, r5
	adds	r5, r5, #1
	cmp	r3, r0
	bhi	.L1910
	ldrh	r3, [r4, #242]
	ldrh	r5, [r4, #232]
	ldr	r9, [r4, #244]
	str	r3, [r4, #3284]
	mov	r1, r5
	mov	r0, r9
	bl	__aeabi_uidiv
	ubfx	r7, r0, #5, #16
	mov	r8, r0
	add	r3, r7, #36
	str	r0, [r4, #2552]
	strh	r3, [r4, #2620]	@ movhi
	movs	r3, #24
	muls	r3, r5, r3
	cmp	r6, r3
	ble	.L1911
	mov	r1, r5
	sub	r0, r9, r6
	bl	__aeabi_uidiv
	str	r0, [r4, #2552]
	lsrs	r0, r0, #5
	adds	r0, r0, #24
	strh	r0, [r4, #2620]	@ movhi
.L1911:
	ldrh	r3, [r4, #294]
	cbz	r3, .L1913
	ldrh	r2, [r4, #2620]
	add	r2, r2, r3, lsr #1
	strh	r2, [r4, #2620]	@ movhi
	mul	r2, r5, r3
	cmp	r6, r2
	itttt	lt
	addlt	r3, r3, #32
	strlt	r8, [r4, #2552]
	addlt	r3, r3, r7
	strhlt	r3, [r4, #2620]	@ movhi
.L1913:
	ldrh	r2, [r4, #2620]
	ldr	r3, [r4, #2552]
	ldr	r6, .L1923+8
	subs	r3, r3, r2
	muls	r5, r3, r5
	ldrh	r3, [r4, #302]
	str	r5, [r4, #3672]
	muls	r5, r3, r5
	ldrh	r3, [r4, #258]
	str	r5, [r4, #2552]
	muls	r5, r3, r5
	str	r5, [r4, #340]
	bl	FtlBbmTblFlush
	ldrh	r2, [r4, #242]
	movs	r1, #0
	ldr	r0, [r4, #72]
	lsls	r2, r2, #1
	bl	ftl_memset
	movw	r2, #65535
	movs	r3, #0
	mov	r5, r2
	str	r3, [r4, #2548]
	strh	r3, [r4, #174]	@ movhi
	strb	r3, [r4, #178]
	strb	r3, [r4, #180]
	strh	r3, [r4, #26]	@ movhi
	strb	r3, [r4, #30]
	strh	r3, [r4, #24]	@ movhi
	movs	r3, #1
	strh	r2, [r4, #172]	@ movhi
	strb	r3, [r4, #32]
.L1915:
	mov	r0, r6
	bl	make_superblock
	ldrb	r3, [r4, #31]	@ zero_extendqisi2
	ldrh	r2, [r4, #24]
	cmp	r3, #0
	bne	.L1916
	ldr	r3, [r4, #72]
	strh	r5, [r3, r2, lsl #1]	@ movhi
	ldrh	r3, [r4, #24]
	adds	r3, r3, #1
	strh	r3, [r4, #24]	@ movhi
	b	.L1915
.L1902:
	ldr	r5, [r4, #3312]
	mvns	r1, r3
	orr	r1, r3, r1, lsl #16
	str	r1, [r5, r3, lsl #2]
	ldr	r1, [r4, #3316]
	str	r0, [r1, r3, lsl #2]
	b	.L1901
.L1904:
	mov	r0, r6
	movs	r1, #1
	bl	FtlLowFormatEraseBlock
	adds	r6, r6, #1
	add	r5, r5, r0
	uxth	r5, r5
	uxth	r6, r6
	b	.L1903
.L1905:
	mov	r0, r5
	bl	__aeabi_uidiv
	ldr	r3, [r4, #332]
	add	r0, r0, r3
	uxth	r0, r0
	bl	FtlSysBlkNumInit
	ldrh	r0, [r4, #236]
	bl	FtlFreeSysBlkQueueInit
	ldrh	r5, [r4, #240]
.L1907:
	ldrh	r3, [r4, #242]
	cmp	r3, r5
	bls	.L1909
	mov	r0, r5
	movs	r1, #1
	adds	r5, r5, #1
	bl	FtlLowFormatEraseBlock
	uxth	r5, r5
	b	.L1907
.L1910:
	movs	r1, #0
	bl	FtlLowFormatEraseBlock
	add	r6, r6, r0
	uxth	r6, r6
	b	.L1906
.L1916:
	ldr	r3, [r4, #2588]
	movw	r5, #65535
	ldrh	r1, [r4, #28]
	ldr	r6, .L1923+12
	str	r3, [r4, #36]
	adds	r3, r3, #1
	str	r3, [r4, #2588]
	ldr	r3, [r4, #72]
	strh	r1, [r3, r2, lsl #1]	@ movhi
	movs	r3, #0
	strh	r3, [r4, #78]	@ movhi
	strb	r3, [r4, #82]
	ldrh	r3, [r4, #24]
	adds	r3, r3, #1
	strh	r3, [r4, #76]	@ movhi
	movs	r3, #1
	strb	r3, [r4, #84]
.L1917:
	mov	r0, r6
	bl	make_superblock
	ldrb	r3, [r4, #83]	@ zero_extendqisi2
	ldrh	r2, [r4, #76]
	cbnz	r3, .L1918
	ldr	r3, [r4, #72]
	strh	r5, [r3, r2, lsl #1]	@ movhi
	ldrh	r3, [r4, #76]
	adds	r3, r3, #1
	strh	r3, [r4, #76]	@ movhi
	b	.L1917
.L1918:
	ldr	r3, [r4, #2588]
	movw	r5, #65535
	ldrh	r1, [r4, #80]
	str	r3, [r4, #88]
	adds	r3, r3, #1
	str	r3, [r4, #2588]
	ldr	r3, [r4, #72]
	strh	r1, [r3, r2, lsl #1]	@ movhi
	strh	r5, [r4, #124]	@ movhi
	bl	FtlFreeSysBlkQueueOut
	movs	r3, #0
	strh	r0, [r4, #2624]	@ movhi
	strh	r3, [r4, #2626]	@ movhi
	ldr	r3, [r4, #3672]
	strh	r5, [r4, #2628]	@ movhi
	strh	r3, [r4, #2630]	@ movhi
	ldr	r3, [r4, #2588]
	str	r3, [r4, #2632]
	adds	r3, r3, #1
	str	r3, [r4, #2588]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cbnz	r0, .L1919
	ldr	r3, .L1923+16
	movs	r2, #1
	str	r2, [r3]
.L1919:
	movs	r0, #0
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L1924:
	.align	2
.L1923:
	.word	.LANCHOR0
	.word	168778952
	.word	.LANCHOR0+24
	.word	.LANCHOR0+76
	.word	.LANCHOR2
	.fnend
	.size	ftl_low_format, .-ftl_low_format
	.align	1
	.global	sftl_init
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_init, %function
sftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r3, #-1
	ldr	r4, .L1931
	ldr	r5, .L1931+4
	ldr	r1, .L1931+8
	ldr	r0, .L1931+12
	str	r3, [r5]
	bl	sftl_printk
	mov	r0, r4
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldrh	r0, [r4, #236]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbnz	r0, .L1929
	bl	FtlSysBlkInit
	cbnz	r0, .L1929
	movs	r3, #1
	str	r3, [r5]
	ldrh	r3, [r4, #224]
	cmp	r3, #15
	bhi	.L1929
	movw	r4, #8129
.L1928:
	movs	r1, #1
	movs	r0, #0
	bl	rk_ftl_garbage_collect
	subs	r4, r4, #1
	bne	.L1928
.L1929:
	movs	r0, #0
	pop	{r3, r4, r5, pc}
.L1932:
	.align	2
.L1931:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC0
	.word	.LC77
	.fnend
	.size	sftl_init, .-sftl_init
	.align	1
	.global	FtlWriteToIDB
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	FtlWriteToIDB, %function
FtlWriteToIDB:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	add	r8, r1, r0
	add	r9, r8, #-1
	.pad #108
	sub	sp, sp, #108
	cmp	r9, #63
	mov	r4, r0
	mov	r6, r1
	mov	r7, r2
	ldr	r5, .L2000
	bls	.L1934
	cmp	r0, #576
	bcc	.L1935
.L1934:
	ldr	r3, [r5, #3676]
	cbnz	r3, .L1936
.L1999:
	movs	r4, #0
	b	.L1933
.L1936:
	ldr	r8, [r5, #3680]
	ldr	r3, .L2000+4
	ldr	r2, [r8]
	cmp	r2, r3
	bne	.L1938
	ldrh	r1, [r5, #10]
	add	r0, r8, #260096
	movs	r2, #0
	movw	r3, #65023
.L1942:
	ldr	r4, [r0, #-4]!
	cbnz	r4, .L1939
	ldr	r4, [r8, r2, lsl #2]
	adds	r2, r2, #1
	cmp	r2, #4096
	add	r3, r3, #-1
	it	hi
	movhi	r2, #0
	cmp	r3, #4096
	str	r4, [r0, #2048]
	bne	.L1942
	mov	r3, #512
	b	.L1997
.L1939:
	adds	r3, r3, #127
	lsrs	r3, r3, #7
.L1997:
	str	r3, [sp, #12]
	lsls	r1, r1, #2
	ldr	r3, [sp, #12]
	uxth	r1, r1
	ldr	r4, .L2000
	adds	r0, r3, #4
	bl	__aeabi_uidiv
	adds	r3, r0, #1
	ldr	r1, [sp, #12]
	mov	r2, r3
	ldr	r0, .L2000+8
	str	r3, [sp, #16]
	bl	sftl_printk
	ldr	r3, [sp, #12]
	lsls	r3, r3, #7
	str	r3, [sp, #32]
	movs	r3, #0
	mov	fp, r3
	str	r3, [sp, #20]
.L1943:
	ldr	r3, [sp, #16]
	add	r3, r3, fp
	cmp	r3, #8
	str	r3, [sp, #28]
	bls	.L1965
	ldr	r3, [sp, #20]
	cbnz	r3, .L1966
.L1938:
	mov	r3, #-1
	str	r3, [sp, #12]
.L1966:
	ldr	r5, .L2000
	movs	r3, #0
	ldr	r4, [sp, #12]
	ldr	r0, [r5, #3680]
	str	r3, [r5, #3676]
	bl	kfree
	ldr	r0, [r5, #3684]
	bl	kfree
.L1933:
	mov	r0, r4
	add	sp, sp, #108
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1965:
	mov	r1, #512
	ldr	r0, [r4, #3684]
	bl	__memzero
	ldrh	r6, [r4, #10]
	mul	r3, r6, fp
	str	r3, [sp, #24]
	ldr	r3, [r4, #3260]
	cmp	r3, #0
	beq	.L1973
	ldr	r3, [r4, #3264]
	cmp	r3, #0
	ite	eq
	moveq	r9, #6
	movne	r9, #9
.L1944:
	ldr	r7, [sp, #24]
	movs	r5, #0
.L1945:
	ldr	r3, [r4, #3248]
	mov	r1, r7
	movs	r0, #0
	adds	r5, r5, #1
	blx	r3
	ldr	r3, [sp, #16]
	add	r7, r7, r6
	cmp	r3, r5
	bhi	.L1945
	cmp	r9, #9
	bne	.L1975
	ldr	r5, [r4, #3684]
	mov	r1, #1024
	mov	r0, r5
	bl	__memzero
	ldr	r3, .L2000+12
	movs	r1, #12
	movs	r2, #4
	str	r1, [r5, #4]
	strb	r2, [r5, #17]
	adds	r0, r5, r1
	str	r3, [r5]
	movs	r3, #0
	ldrh	r2, [r4, #10]
	str	r3, [r5, #12]
	strb	r3, [r5, #16]
	strh	r2, [r5, #18]	@ movhi
	movs	r2, #16
	strb	r3, [r5, #20]
	strb	r2, [r5, #21]
	strh	r3, [r5, #22]	@ movhi
	bl	js_hash
	str	r0, [r5, #8]
.L1946:
	ldr	r3, [sp, #16]
	mov	r10, r8
	muls	r3, r6, r3
	movs	r6, #0
	str	r3, [sp, #36]
.L1947:
	ldr	r3, [sp, #36]
	cmp	r6, r3
	beq	.L1954
	cmp	r9, #9
	ite	ne
	lslne	r3, r6, #2
	addeq	r3, r6, #1
	str	r3, [sp, #40]
	movw	r3, #61424
	str	r3, [sp, #44]
	cmp	r6, #0
	bne	.L1950
	cmp	r9, #9
	bne	.L1950
	ldr	r3, [r4, #3260]
	movs	r0, #70
	blx	r3
	ldr	r7, [r4, #3252]
	add	r3, sp, #40
	mov	r2, r5
	ldr	r1, [sp, #24]
	mov	r0, r6
	blx	r7
	ldr	r3, [r4, #3260]
	mov	r7, r0
	ldrb	r0, [r4, #22]	@ zero_extendqisi2
	blx	r3
	adds	r7, r7, #1
	bne	.L1951
.L1954:
	ldrb	r3, [r4, #14]	@ zero_extendqisi2
	ldr	r2, [r4, #3260]
	ldr	r7, [r4, #3684]
	str	r3, [sp, #24]
	ldrh	r3, [r4, #10]
	mul	r10, r3, fp
	cmp	r2, #0
	bne	.L1952
	mov	r9, #6
.L1953:
	ldr	r2, [sp, #16]
	movs	r6, #0
	muls	r3, r2, r3
	str	r3, [sp, #36]
.L1956:
	ldr	r3, [sp, #36]
	cmp	r6, r3
	beq	.L1960
	cmp	r6, #0
	bne	.L1957
	cmp	r9, #9
	bne	.L1957
	ldr	r3, [r4, #3260]
	movs	r0, #70
	blx	r3
	ldr	r3, [r4, #3264]
	movs	r0, #2
	blx	r3
	mov	r2, r7
	mov	r1, r10
	add	r3, sp, #40
	mov	r0, r6
	ldr	r5, [r4, #3256]
	blx	r5
	ldr	r3, [r4, #3264]
	ldr	r0, [sp, #24]
	blx	r3
	ldr	r3, [r4, #3260]
	ldrb	r0, [r4, #22]	@ zero_extendqisi2
	blx	r3
	ldr	r3, [r7]
	ldr	r2, .L2000+12
	cmp	r3, r2
	beq	.L1958
.L1960:
	ldr	r0, [r4, #3684]
	mov	r1, r8
	movs	r5, #0
.L1959:
	mov	r7, r1
	ldr	r2, [r0, r5, lsl #2]
	ldr	r3, [r7]
	lsls	r6, r5, #2
	adds	r1, r1, #4
	cmp	r2, r3
	beq	.L1962
	mov	r1, #512
	bl	__memzero
	ldr	r2, [r4, #3684]
	mov	r1, fp
	str	r5, [sp]
	ldr	r3, [r7]
	ldr	r0, .L2000+16
	ldr	r2, [r2, r6]
	bl	sftl_printk
	ldrh	r1, [r4, #10]
	movs	r0, #0
	ldr	r3, [r4, #3248]
	mul	r1, fp, r1
	blx	r3
.L1963:
	ldr	fp, [sp, #28]
	b	.L1943
.L1973:
	mov	r9, #6
	b	.L1944
.L1975:
	movs	r5, #0
	b	.L1946
.L1950:
	ldr	r1, [sp, #24]
	add	r3, sp, #40
	ldr	r7, [r4, #3252]
	mov	r2, r10
	movs	r0, #0
	add	r1, r1, r6
	blx	r7
	adds	r0, r0, #1
	beq	.L1954
	add	r10, r10, #2048
.L1951:
	adds	r6, r6, #1
	b	.L1947
.L1952:
	ldr	r2, [r4, #3264]
	cmp	r2, #0
	ite	eq
	moveq	r9, #6
	movne	r9, #9
	b	.L1953
.L1957:
	add	r3, sp, #40
	mov	r2, r7
	add	r1, r10, r6
	movs	r0, #0
	ldr	r5, [r4, #3256]
	blx	r5
	adds	r0, r0, #1
	beq	.L1960
	ldr	r2, [sp, #44]
	movw	r3, #61424
	cmp	r2, r3
	bne	.L1960
	add	r7, r7, #2048
.L1958:
	adds	r6, r6, #1
	b	.L1956
.L1962:
	ldr	r3, [sp, #32]
	adds	r5, r5, #1
	cmp	r5, r3
	bne	.L1959
	ldr	r3, [sp, #20]
	adds	r3, r3, #1
	cmp	r3, #5
	str	r3, [sp, #20]
	bls	.L1963
	b	.L1966
.L2001:
	.align	2
.L2000:
	.word	.LANCHOR0
	.word	-52655045
	.word	.LC130
	.word	1179535694
	.word	.LC131
.L1935:
	cmp	r0, #64
	bne	.L1967
	mov	r0, #262144
	bl	ftl_malloc
	str	r0, [r5, #3680]
	mov	r0, #262144
	bl	ftl_malloc
	ldr	r3, [r5, #3680]
	str	r0, [r5, #3684]
	cbz	r3, .L1968
	cbz	r0, .L1968
	movs	r2, #1
	mov	r1, #262144
	mov	r0, r3
	str	r2, [r5, #3676]
	bl	__memzero
.L1967:
	ldr	r3, [r5, #3676]
	cmp	r3, #0
	beq	.L1999
	cmp	r4, #63
	ldr	r0, [r5, #3680]
	iteet	hi
	ldrhi	r3, .L2002
	rsbls	r1, r4, #64
	subls	r6, r6, r1
	movhi	r1, r7
	itee	ls
	addls	r1, r7, r1, lsl #9
	addhi	r3, r3, r4
	addhi	r0, r0, r3, lsl #9
	cmp	r9, #576
	itt	cs
	subcs	r6, r6, r8
	subcs	r6, r6, #446
	lsls	r2, r6, #9
	bl	ftl_memcpy
	b	.L1999
.L1968:
	ldr	r1, .L2002+4
	ldr	r0, .L2002+8
	bl	sftl_printk
	b	.L1967
.L2003:
	.align	2
.L2002:
	.word	8388544
	.word	.LANCHOR1+633
	.word	.LC132
	.fnend
	.size	FtlWriteToIDB, .-FtlWriteToIDB
	.align	1
	.global	sftl_write
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	sftl_write, %function
sftl_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r1
	mov	r6, r2
	mov	r4, r0
	bl	FtlWriteToIDB
	mov	r3, r6
	mov	r2, r5
	mov	r1, r4
	movs	r0, #0
	pop	{r4, r5, r6, lr}
	b	FtlWrite
	.fnend
	.size	sftl_write, .-sftl_write
	.align	1
	.global	rk_sftl_vendor_dev_ops_register
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rk_sftl_vendor_dev_ops_register, %function
rk_sftl_vendor_dev_ops_register:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L2008
	ldr	r3, [r2, #3688]
	cbnz	r3, .L2007
	str	r0, [r2, #3688]
	mov	r0, r3
	str	r1, [r2, #3692]
	bx	lr
.L2007:
	mov	r0, #-1
	bx	lr
.L2009:
	.align	2
.L2008:
	.word	.LANCHOR0
	.fnend
	.size	rk_sftl_vendor_dev_ops_register, .-rk_sftl_vendor_dev_ops_register
	.align	1
	.global	rk_sftl_vendor_storage_init
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rk_sftl_vendor_storage_init, %function
rk_sftl_vendor_storage_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r0, #65536
	ldr	r4, .L2018
	bl	ftl_malloc
	str	r0, [r4, #3696]
	cmp	r0, #0
	beq	.L2016
	ldr	r8, .L2018+4
	mov	r9, #0
	mov	r7, r9
	mov	r6, r9
.L2014:
	ldr	r3, [r4, #3688]
	movs	r1, #128
	ldr	r2, [r4, #3696]
	lsls	r0, r6, #7
	blx	r3
	mov	r5, r0
	cbnz	r0, .L2012
	ldr	r2, [r4, #3696]
	ldr	r3, [r2]
	cmp	r3, r8
	bne	.L2013
	movw	r3, #65532
	ldr	r1, [r2, r3]
	ldr	r3, [r2, #4]
	cmp	r1, r3
	bne	.L2013
	cmp	r1, r7
	itt	hi
	movhi	r9, r6
	movhi	r7, r1
.L2013:
	adds	r6, r6, #1
	cmp	r6, #2
	bne	.L2014
	cbz	r7, .L2015
	ldr	r3, [r4, #3688]
	movs	r1, #128
	lsl	r0, r9, #7
	blx	r3
	mov	r5, r0
	cbz	r0, .L2010
.L2012:
	ldr	r0, [r4, #3696]
	mov	r5, #-1
	bl	kfree
	movs	r3, #0
	str	r3, [r4, #3696]
	b	.L2010
.L2015:
	mov	r1, #65536
	mov	r0, r2
	bl	__memzero
	ldr	r3, [r4, #3696]
	movs	r2, #1
	movw	r1, #65532
	str	r2, [r3, #4]
	str	r8, [r3]
	str	r2, [r3, r1]
	movw	r2, #64504
	strh	r7, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
.L2010:
	mov	r0, r5
	pop	{r3, r4, r5, r6, r7, r8, r9, pc}
.L2016:
	mvn	r5, #11
	b	.L2010
.L2019:
	.align	2
.L2018:
	.word	.LANCHOR0
	.word	1380668996
	.fnend
	.size	rk_sftl_vendor_storage_init, .-rk_sftl_vendor_storage_init
	.align	1
	.global	rk_sftl_vendor_read
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rk_sftl_vendor_read, %function
rk_sftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r7, r0
	ldr	r3, .L2026
	mov	r0, r1
	ldr	r5, [r3, #3696]
	cbz	r5, .L2025
	ldrh	r6, [r5, #10]
	movs	r3, #0
.L2022:
	cmp	r3, r6
	bcc	.L2024
.L2025:
	mov	r0, #-1
	pop	{r3, r4, r5, r6, r7, pc}
.L2024:
	add	r1, r5, r3, lsl #3
	ldrh	r4, [r1, #16]
	cmp	r4, r7
	bne	.L2023
	ldrh	r4, [r1, #20]
	ldrh	r1, [r1, #18]
	cmp	r4, r2
	it	cs
	movcs	r4, r2
	add	r1, r1, #1024
	mov	r2, r4
	add	r1, r1, r5
	bl	memcpy
	mov	r0, r4
	pop	{r3, r4, r5, r6, r7, pc}
.L2023:
	adds	r3, r3, #1
	b	.L2022
.L2027:
	.align	2
.L2026:
	.word	.LANCHOR0
	.fnend
	.size	rk_sftl_vendor_read, .-rk_sftl_vendor_read
	.align	1
	.global	rk_sftl_vendor_write
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rk_sftl_vendor_write, %function
rk_sftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r2
	ldr	r2, .L2047
	.pad #28
	sub	sp, sp, #28
	mov	r3, r1
	ldr	r4, [r2, #3696]
	mov	r9, r2
	cmp	r4, #0
	beq	.L2043
	ldrh	r2, [r4, #10]
	add	r6, r8, #63
	ldrh	r1, [r4, #8]
	bic	r6, r6, #63
	movs	r7, #0
	str	r1, [sp, #4]
.L2030:
	cmp	r7, r2
	bcc	.L2038
	ldrh	r1, [r4, #14]
	cmp	r6, r1
	bhi	.L2043
	add	r2, r4, r2, lsl #3
	uxth	r6, r6
	strh	r0, [r2, #16]	@ movhi
	ldrh	r0, [r4, #12]
	strh	r8, [r2, #20]	@ movhi
	strh	r0, [r2, #18]	@ movhi
	add	r0, r0, r6
	subs	r6, r1, r6
	strh	r0, [r4, #12]	@ movhi
	strh	r6, [r4, #14]	@ movhi
	mov	r1, r3
	ldrh	r0, [r2, #18]
	mov	r2, r8
	add	r0, r0, #1024
	add	r0, r0, r4
	bl	memcpy
	ldrh	r3, [r4, #10]
	movw	r2, #65532
	adds	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
	ldr	r3, [r4, #4]
	adds	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r4, r2]
	ldrh	r3, [r4, #8]
	adds	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #1
	it	hi
	movhi	r3, #0
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, [r9, #3692]
	b	.L2046
.L2038:
	add	r5, r4, r7, lsl #3
	ldrh	r1, [r5, #16]
	cmp	r1, r0
	str	r1, [sp, #8]
	bne	.L2031
	ldrh	r1, [r5, #20]
	add	fp, r4, #1024
	adds	r1, r1, #63
	bic	r1, r1, #63
	cmp	r8, r1
	str	r1, [sp, #12]
	bls	.L2032
	ldrh	r1, [r4, #14]
	cmp	r6, r1
	bhi	.L2043
	ldrh	r10, [r5, #18]
	subs	r2, r2, #1
	str	r2, [sp, #16]
.L2033:
	ldr	r2, [sp, #16]
	adds	r5, r5, #8
	cmp	r7, r2
	bcc	.L2034
	ldrh	r2, [sp, #8]
	add	r7, r4, r7, lsl #3
	uxth	r5, r10
	mov	r1, r3
	strh	r8, [r7, #20]	@ movhi
	uxtah	r0, fp, r10
	strh	r2, [r7, #16]	@ movhi
	mov	r2, r8
	strh	r5, [r7, #18]	@ movhi
	bl	memcpy
	uxth	r3, r6
	ldrh	r6, [r4, #14]
	add	r5, r5, r3
	subs	r6, r6, r3
	ldr	r3, [sp, #12]
	strh	r5, [r4, #12]	@ movhi
	add	r6, r6, r3
	strh	r6, [r4, #14]	@ movhi
.L2035:
	ldr	r3, [r4, #4]
	movw	r2, #65532
	adds	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r4, r2]
	ldrh	r3, [r4, #8]
	adds	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #1
	it	hi
	movhi	r3, #0
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, .L2047
	ldr	r3, [r3, #3692]
.L2046:
	ldr	r0, [sp, #4]
	mov	r2, r4
	movs	r1, #128
	lsls	r0, r0, #7
	blx	r3
	movs	r0, #0
.L2028:
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2034:
	ldrh	r9, [r5, #20]
	add	r0, fp, r10
	ldrh	r2, [r5, #16]
	adds	r7, r7, #1
	ldrh	r1, [r5, #18]
	strh	r9, [r5, #12]	@ movhi
	add	r9, r9, #63
	bic	r9, r9, #63
	strh	r2, [r5, #8]	@ movhi
	strh	r10, [r5, #10]	@ movhi
	mov	r2, r9
	add	r1, r1, fp
	str	r3, [sp, #20]
	add	r10, r10, r9
	bl	memcpy
	ldr	r3, [sp, #20]
	b	.L2033
.L2032:
	ldrh	r0, [r5, #18]
	mov	r2, r8
	mov	r1, r3
	add	r0, r0, fp
	bl	memcpy
	strh	r8, [r5, #20]	@ movhi
	b	.L2035
.L2031:
	adds	r7, r7, #1
	b	.L2030
.L2043:
	mov	r0, #-1
	b	.L2028
.L2048:
	.align	2
.L2047:
	.word	.LANCHOR0
	.fnend
	.size	rk_sftl_vendor_write, .-rk_sftl_vendor_write
	.align	1
	.global	rk_sftl_vendor_storage_ioctl
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rk_sftl_vendor_storage_ioctl, %function
rk_sftl_vendor_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r0, #4096
	mov	r6, r1
	mov	r4, r2
	bl	ftl_malloc
	mov	r5, r0
	cmp	r0, #0
	beq	.L2066
	ldr	r3, .L2077
	cmp	r6, r3
	beq	.L2052
	adds	r3, r3, #1
	cmp	r6, r3
	beq	.L2053
.L2072:
	mvn	r4, #13
	b	.L2051
.L2052:
	ldr	r6, .L2077+4
	mov	r3, sp
	ands	r3, r3, r6
	ldr	r3, [r3, #72]
	.syntax unified
@ 564 "./arch/arm/include/asm/uaccess.h" 1
	adds r2, r4, #8; sbcccs r2, r2, r3; movcc r3, #0
@ 0 "" 2
	.thumb
	.syntax unified
	cbnz	r3, .L2054
	movs	r2, #8
	mov	r1, r4
	bl	arm_copy_from_user
	cbz	r0, .L2073
.L2055:
	mov	r2, r4
	mov	r1, #256
.L2076:
	ldr	r0, .L2077+8
	bl	sftl_printk
	b	.L2072
.L2054:
	movs	r1, #8
	bl	__memzero
	b	.L2055
.L2073:
	ldr	r2, [r5]
	ldr	r3, .L2077+12
	cmp	r2, r3
	beq	.L2057
.L2058:
	mov	r4, #-1
.L2051:
	mov	r0, r5
	bl	kfree
.L2049:
	mov	r0, r4
	pop	{r4, r5, r6, pc}
.L2057:
	ldrh	r2, [r5, #6]
	add	r1, r5, #8
	ldrh	r0, [r5, #4]
	bl	rk_sftl_vendor_read
	adds	r3, r0, #1
	beq	.L2058
	mov	r3, sp
	uxth	r2, r0
	ands	r6, r6, r3
	strh	r0, [r5, #6]	@ movhi
	adds	r2, r2, #8
	ldr	r3, [r6, #72]
	.syntax unified
@ 573 "./arch/arm/include/asm/uaccess.h" 1
	adds r1, r4, r2; sbcccs r1, r1, r3; movcc r3, #0
@ 0 "" 2
	.thumb
	.syntax unified
	cmp	r3, #0
	bne	.L2072
	mov	r1, r5
	mov	r0, r4
	bl	arm_copy_to_user
	cmp	r0, #0
	bne	.L2072
.L2068:
	mov	r4, r0
	b	.L2051
.L2053:
	ldr	r6, .L2077+4
	mov	r3, sp
	ands	r3, r3, r6
	ldr	r3, [r3, #72]
	.syntax unified
@ 564 "./arch/arm/include/asm/uaccess.h" 1
	adds r2, r4, #8; sbcccs r2, r2, r3; movcc r3, #0
@ 0 "" 2
	.thumb
	.syntax unified
	cbnz	r3, .L2060
	movs	r2, #8
	mov	r1, r4
	bl	arm_copy_from_user
	cbz	r0, .L2074
.L2061:
	mov	r2, r4
	mov	r1, #276
	b	.L2076
.L2060:
	movs	r1, #8
	bl	__memzero
	b	.L2061
.L2074:
	ldr	r2, [r5]
	ldr	r3, .L2077+12
	cmp	r2, r3
	bne	.L2058
	ldrh	r2, [r5, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L2058
	mov	r3, sp
	adds	r2, r2, #8
	ands	r6, r6, r3
	ldr	r3, [r6, #72]
	.syntax unified
@ 564 "./arch/arm/include/asm/uaccess.h" 1
	adds r1, r4, r2; sbcccs r1, r1, r3; movcc r3, #0
@ 0 "" 2
	.thumb
	.syntax unified
	cbnz	r3, .L2063
	mov	r1, r4
	mov	r0, r5
	bl	arm_copy_from_user
	cbz	r0, .L2075
.L2064:
	mov	r2, r4
	movw	r1, #283
	b	.L2076
.L2063:
	mov	r1, r2
	mov	r0, r5
	bl	__memzero
	b	.L2064
.L2075:
	ldrh	r2, [r5, #6]
	add	r1, r5, #8
	ldrh	r0, [r5, #4]
	bl	rk_sftl_vendor_write
	b	.L2068
.L2066:
	mov	r4, #-1
	b	.L2049
.L2078:
	.align	2
.L2077:
	.word	1074034177
	.word	-8192
	.word	.LC133
	.word	1448232273
	.fnend
	.size	rk_sftl_vendor_storage_ioctl, .-rk_sftl_vendor_storage_ioctl
	.align	1
	.global	rk_sftl_vendor_register
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	rk_sftl_vendor_register, %function
rk_sftl_vendor_register:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, .L2080
	b	misc_register
.L2081:
	.align	2
.L2080:
	.word	.LANCHOR2+12
	.fnend
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
	.type	__func__.6710, %object
	.size	__func__.6710, 17
__func__.6710:
	.ascii	"INSERT_DATA_LIST\000"
	.type	__func__.6705, %object
	.size	__func__.6705, 17
__func__.6705:
	.ascii	"INSERT_FREE_LIST\000"
	.type	__func__.6741, %object
	.size	__func__.6741, 17
__func__.6741:
	.ascii	"List_remove_node\000"
	.type	__func__.6773, %object
	.size	__func__.6773, 22
__func__.6773:
	.ascii	"List_update_data_list\000"
	.type	__func__.6880, %object
	.size	__func__.6880, 22
__func__.6880:
	.ascii	"select_l2p_ram_region\000"
	.type	__func__.7200, %object
	.size	__func__.7200, 16
__func__.7200:
	.ascii	"make_superblock\000"
	.type	__func__.7376, %object
	.size	__func__.7376, 19
__func__.7376:
	.ascii	"get_new_active_ppa\000"
	.type	__func__.19135, %object
	.size	__func__.19135, 17
__func__.19135:
	.ascii	"FlashEraseBlocks\000"
	.type	__func__.6815, %object
	.size	__func__.6815, 26
__func__.6815:
	.ascii	"ftl_map_blk_alloc_new_blk\000"
	.type	__func__.6320, %object
	.size	__func__.6320, 11
__func__.6320:
	.ascii	"FtlMemInit\000"
	.type	__func__.6547, %object
	.size	__func__.6547, 14
__func__.6547:
	.ascii	"FtlBbt2Bitmap\000"
	.type	__func__.7221, %object
	.size	__func__.7221, 18
__func__.7221:
	.ascii	"SupperBlkListInit\000"
	.type	__func__.19092, %object
	.size	__func__.19092, 15
__func__.19092:
	.ascii	"FlashReadPages\000"
	.type	__func__.6992, %object
	.size	__func__.6992, 14
__func__.6992:
	.ascii	"FtlScanSysBlk\000"
	.type	__func__.6590, %object
	.size	__func__.6590, 11
__func__.6590:
	.ascii	"FtlLoadBbt\000"
	.type	__func__.19111, %object
	.size	__func__.19111, 15
__func__.19111:
	.ascii	"FlashProgPages\000"
	.type	__func__.7348, %object
	.size	__func__.7348, 25
__func__.7348:
	.ascii	"allocate_data_superblock\000"
	.type	__func__.7389, %object
	.size	__func__.7389, 16
__func__.7389:
	.ascii	"update_vpc_list\000"
	.type	__func__.7396, %object
	.size	__func__.7396, 20
__func__.7396:
	.ascii	"decrement_vpc_count\000"
	.type	__func__.6841, %object
	.size	__func__.6841, 31
__func__.6841:
	.ascii	"Ftl_write_map_blk_to_last_page\000"
	.type	__func__.6855, %object
	.size	__func__.6855, 16
__func__.6855:
	.ascii	"FtlMapWritePage\000"
	.type	__func__.6782, %object
	.size	__func__.6782, 16
__func__.6782:
	.ascii	"load_l2p_region\000"
	.type	__func__.6826, %object
	.size	__func__.6826, 15
__func__.6826:
	.ascii	"ftl_map_blk_gc\000"
	.type	__func__.6897, %object
	.size	__func__.6897, 9
__func__.6897:
	.ascii	"log2phys\000"
	.type	__func__.7109, %object
	.size	__func__.7109, 16
__func__.7109:
	.ascii	"FtlReUsePrevPpa\000"
	.type	__func__.7143, %object
	.size	__func__.7143, 22
__func__.7143:
	.ascii	"FtlRecoverySuperblock\000"
	.type	__func__.7263, %object
	.size	__func__.7263, 14
__func__.7263:
	.ascii	"ftl_check_vpc\000"
	.type	__func__.6970, %object
	.size	__func__.6970, 15
__func__.6970:
	.ascii	"FtlVpcTblFlush\000"
	.type	__func__.7246, %object
	.size	__func__.7246, 21
__func__.7246:
	.ascii	"FtlVpcCheckAndModify\000"
	.type	__func__.7369, %object
	.size	__func__.7369, 29
__func__.7369:
	.ascii	"allocate_new_data_superblock\000"
	.type	__func__.6444, %object
	.size	__func__.6444, 13
__func__.6444:
	.ascii	"FtlProgPages\000"
	.type	__func__.7466, %object
	.size	__func__.7466, 19
__func__.7466:
	.ascii	"FtlGcFreeTempBlock\000"
	.type	__func__.7579, %object
	.size	__func__.7579, 23
__func__.7579:
	.ascii	"rk_ftl_garbage_collect\000"
	.type	__func__.6472, %object
	.size	__func__.6472, 9
__func__.6472:
	.ascii	"FtlWrite\000"
	.type	__func__.7041, %object
	.size	__func__.7041, 15
__func__.7041:
	.ascii	"FtlLoadSysInfo\000"
	.type	__func__.7063, %object
	.size	__func__.7063, 18
__func__.7063:
	.ascii	"FtlMapTblRecovery\000"
	.type	__func__.19263, %object
	.size	__func__.19263, 14
__func__.19263:
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
