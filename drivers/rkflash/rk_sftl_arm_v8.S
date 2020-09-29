/* SPDX-License-Identifier: GPL-2.0 */

/* Copyright (c) 2018 Rockchip Electronics Co. Ltd.
 * date: 2020-09-29
 */
	.arch armv8-a
	.file	"rk_sftl.c"
#APP
	.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30
	.equ	.L__reg_num_x\num, \num
	.endr
	.equ	.L__reg_num_xzr, 31

	.macro	mrs_s, rt, sreg
	.inst	0xd5200000|(\sreg)|(.L__reg_num_\rt)
	.endm

	.macro	msr_s, sreg, rt
	.inst	0xd5000000|(\sreg)|(.L__reg_num_\rt)
	.endm

#NO_APP
	.text
	.align	2
	.type	l2p_addr_tran.isra.0, %function
l2p_addr_tran.isra.0:
	adrp	x4, .LANCHOR0
	add	x4, x4, :lo12:.LANCHOR0
	ldrh	w3, [x4, 8]
	ldrh	w6, [x4, 10]
	ldrh	w4, [x4, 14]
	cmp	w4, 4
	bne	.L2
	lsr	w3, w3, 1
	ubfiz	w6, w6, 1, 15
.L2:
	lsr	w5, w0, 10
	and	w0, w0, 1023
	and	w4, w5, 65535
	and	w5, w5, 65535
	udiv	w4, w4, w3
	msub	w3, w3, w4, w5
	madd	w3, w3, w6, w0
	mov	w0, 0
	str	w3, [x1]
	str	w4, [x2]
	ret
	.size	l2p_addr_tran.isra.0, .-l2p_addr_tran.isra.0
	.align	2
	.global	ftl_print_sblk_info
	.type	ftl_print_sblk_info, %function
ftl_print_sblk_info:
	stp	x29, x30, [sp, -32]!
	adrp	x1, .LC0
	adrp	x0, .LC1
	add	x1, x1, :lo12:.LC0
	add	x29, sp, 0
	str	x19, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	add	x0, x0, :lo12:.LC1
	bl	sftl_printk
	adrp	x0, .LC2
	add	x0, x0, :lo12:.LC2
	ldrh	w1, [x19, 24]
	ldr	x3, [x19, 72]
	ldrh	w5, [x19, 28]
	ubfiz	x2, x1, 1, 16
	ldrb	w4, [x19, 32]
	ldrh	w6, [x3, x2]
	ldrb	w3, [x19, 30]
	ldrh	w2, [x19, 26]
	bl	sftl_printk
	ldrh	w1, [x19, 80]
	adrp	x0, .LC3
	ldr	x3, [x19, 72]
	add	x0, x0, :lo12:.LC3
	ldrh	w5, [x19, 84]
	ubfiz	x2, x1, 1, 16
	ldrb	w4, [x19, 88]
	ldrh	w6, [x3, x2]
	ldrb	w3, [x19, 86]
	ldrh	w2, [x19, 82]
	bl	sftl_printk
	ldrh	w1, [x19, 128]
	adrp	x0, .LC4
	ldr	x3, [x19, 72]
	add	x0, x0, :lo12:.LC4
	ldrh	w5, [x19, 132]
	ubfiz	x2, x1, 1, 16
	ldrb	w4, [x19, 136]
	ldrh	w6, [x3, x2]
	ldrb	w3, [x19, 134]
	ldrh	w2, [x19, 130]
	bl	sftl_printk
	ldrh	w1, [x19, 176]
	adrp	x0, .LC5
	ldr	x3, [x19, 72]
	add	x0, x0, :lo12:.LC5
	ldrh	w5, [x19, 180]
	ubfiz	x2, x1, 1, 16
	ldrb	w4, [x19, 184]
	ldrh	w6, [x3, x2]
	ldrb	w3, [x19, 182]
	ldrh	w2, [x19, 178]
	bl	sftl_printk
	ldrh	w3, [x19, 224]
	adrp	x0, .LC6
	ldrh	w2, [x19, 226]
	add	x0, x0, :lo12:.LC6
	ldrh	w1, [x19, 228]
	bl	sftl_printk
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	ftl_print_sblk_info, .-ftl_print_sblk_info
	.align	2
	.global	Ftl_log2
	.type	Ftl_log2, %function
Ftl_log2:
	mov	w2, 1
	mov	w1, 0
.L6:
	cmp	w2, w0
	bls	.L7
	sub	w0, w1, #1
	ret
.L7:
	add	w1, w1, 1
	lsl	w2, w2, 1
	and	w1, w1, 65535
	b	.L6
	.size	Ftl_log2, .-Ftl_log2
	.align	2
	.global	FtlPrintInfo
	.type	FtlPrintInfo, %function
FtlPrintInfo:
	ret
	.size	FtlPrintInfo, .-FtlPrintInfo
	.align	2
	.global	FtlSysBlkNumInit
	.type	FtlSysBlkNumInit, %function
FtlSysBlkNumInit:
	and	w0, w0, 65535
	mov	w1, 24
	cmp	w0, 24
	csel	w0, w0, w1, cs
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	and	w0, w0, 65535
	ldrh	w2, [x1, 236]
	ldrh	w3, [x1, 246]
	str	w0, [x1, 232]
	mul	w2, w2, w0
	sub	w0, w3, w0
	strh	w0, [x1, 244]
	ldr	w0, [x1, 252]
	str	w2, [x1, 240]
	sub	w2, w0, w2
	mov	w0, 0
	str	w2, [x1, 248]
	ret
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.align	2
	.global	FtlConstantsInit
	.type	FtlConstantsInit, %function
FtlConstantsInit:
	stp	x29, x30, [sp, -16]!
	mov	x7, x0
	adrp	x4, .LANCHOR0
	add	x29, sp, 0
	ldrh	w2, [x0]
	add	x0, x4, :lo12:.LANCHOR0
	ldrh	w3, [x7, 14]
	ldrh	w6, [x7, 2]
	ldrh	w5, [x7, 4]
	cmp	w3, 4
	ldrh	w1, [x7, 6]
	strh	w2, [x0, 256]
	strh	w6, [x0, 258]
	strh	w5, [x0, 260]
	strh	w1, [x0, 246]
	strh	w3, [x0, 262]
	bne	.L11
	lsr	w1, w1, 1
	strh	w1, [x0, 246]
	mov	w1, 8
	strh	w1, [x0, 262]
.L12:
	add	x1, x4, :lo12:.LANCHOR0
	mov	x0, 0
	add	x3, x1, 264
.L13:
	strb	w0, [x0, x3]
	add	x0, x0, 1
	cmp	x0, 32
	bne	.L13
	mov	w0, 5
	strh	wzr, [x1, 298]
	strh	w0, [x1, 296]
	cmp	w2, 1
	bne	.L14
	strh	w2, [x1, 296]
.L14:
	add	x3, x4, :lo12:.LANCHOR0
	mov	w0, 640
	strh	w0, [x3, 300]
	ldrh	w0, [x3, 260]
	ldrh	w5, [x3, 246]
	mul	w6, w6, w0
	mul	w0, w0, w5
	and	w6, w6, 65535
	strh	w6, [x3, 236]
	and	w0, w0, 65535
	strh	w0, [x3, 302]
	bl	Ftl_log2
	ldrh	w9, [x7, 12]
	ldrh	w8, [x3, 262]
	strh	w0, [x3, 304]
	strh	w9, [x3, 306]
	mul	w0, w6, w9
	strh	w9, [x3, 308]
	strh	w0, [x3, 310]
	mov	w0, w8
	bl	Ftl_log2
	and	w2, w0, 65535
	strh	w0, [x3, 312]
	ubfiz	w0, w8, 9, 7
	strh	w0, [x3, 314]
	ubfx	w0, w0, 8, 8
	strh	w0, [x3, 316]
	ldrh	w0, [x7, 20]
	strh	w0, [x3, 318]
	mul	w0, w6, w5
	str	w0, [x3, 252]
	mul	w0, w0, w8
	mul	w8, w8, w9
	mul	w0, w0, w9
	asr	w0, w0, 11
	str	w0, [x3, 320]
	mov	w0, 5120
	sdiv	w0, w0, w8
	and	w0, w0, 65535
	cmp	w0, 4
	bls	.L15
.L21:
	add	x1, x4, :lo12:.LANCHOR0
	strh	w0, [x3, 324]
	mov	w0, 640
	lsl	w5, w5, 6
	asr	w0, w0, w2
	add	w2, w2, 9
	add	w0, w0, 2
	strh	w0, [x1, 326]
	asr	w5, w5, w2
	strh	w5, [x1, 328]
	and	w5, w5, 65535
	cmp	w6, 1
	mul	w0, w5, w6
	add	w5, w5, 8
	str	w0, [x1, 332]
	ldrh	w0, [x1, 324]
	udiv	w0, w0, w6
	add	w5, w0, w5
	beq	.L17
.L22:
	add	x4, x4, :lo12:.LANCHOR0
	str	w5, [x1, 232]
	ldrh	w0, [x4, 232]
	bl	FtlSysBlkNumInit
	ldr	w0, [x4, 232]
	str	w0, [x4, 336]
	ldr	w0, [x4, 248]
	ldrh	w1, [x4, 306]
	str	wzr, [x4, 344]
	lsl	w0, w0, 2
	mul	w0, w0, w1
	ldrh	w1, [x4, 312]
	add	w1, w1, 9
	lsr	w0, w0, w1
	add	w0, w0, 2
	strh	w0, [x4, 340]
	mov	w0, 32
	strh	w0, [x4, 342]
	ldrh	w0, [x4, 324]
	add	w0, w0, 3
	strh	w0, [x4, 324]
	ldr	w0, [x4, 332]
	add	w0, w0, 3
	str	w0, [x4, 332]
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
.L11:
	cmp	w3, 8
	bne	.L12
	cmp	w5, 1
	bne	.L12
	lsr	w1, w1, 1
	strh	w1, [x0, 246]
	mov	w1, 2
	strh	w1, [x0, 260]
	b	.L12
.L15:
	mov	w0, 4
	b	.L21
.L17:
	add	w5, w5, 4
	b	.L22
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	2
	.global	IsBlkInVendorPart
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	and	w0, w0, 65535
	ldrh	w2, [x1, 348]
	cbz	w2, .L27
	ldrh	w2, [x1, 324]
	ldr	x3, [x1, 352]
	mov	x1, 0
.L25:
	cmp	w2, w1, uxth
	bhi	.L26
.L27:
	mov	w0, 0
	ret
.L26:
	add	x1, x1, 1
	add	x4, x3, x1, lsl 1
	ldrh	w4, [x4, -2]
	cmp	w4, w0
	bne	.L25
	mov	w0, 1
	ret
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	mov	w0, 0
	ret
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	2
	.global	sftl_get_density
	.type	sftl_get_density, %function
sftl_get_density:
	adrp	x0, .LANCHOR0+344
	ldr	w0, [x0, #:lo12:.LANCHOR0+344]
	ret
	.size	sftl_get_density, .-sftl_get_density
	.align	2
	.global	FtlBbmMapBadBlock
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	stp	x29, x30, [sp, -32]!
	and	w1, w0, 65535
	mov	w4, 1
	add	x29, sp, 0
	str	x19, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	add	x19, x19, 360
	ldrh	w0, [x19, -58]
	udiv	w3, w1, w0
	and	w2, w3, 65535
	msub	w3, w3, w0, w1
	add	x0, x19, x2, uxth 3
	and	w3, w3, 65535
	ldr	x0, [x0, 32]
	ubfx	x5, x3, 5, 11
	lsl	x5, x5, 2
	lsl	w4, w4, w3
	ldr	w6, [x0, x5]
	orr	w4, w4, w6
	str	w4, [x0, x5]
	adrp	x0, .LC7
	add	x0, x0, :lo12:.LC7
	bl	sftl_printk
	ldrh	w0, [x19, 6]
	add	w0, w0, 1
	strh	w0, [x19, 6]
	mov	w0, 0
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.align	2
	.global	FtlBbmIsBadBlock
	.type	FtlBbmIsBadBlock, %function
FtlBbmIsBadBlock:
	adrp	x2, .LANCHOR0
	add	x3, x2, :lo12:.LANCHOR0
	and	w0, w0, 65535
	ldrh	w1, [x3, 302]
	udiv	w2, w0, w1
	msub	w0, w2, w1, w0
	add	x2, x3, x2, uxth 3
	and	w0, w0, 65535
	ldr	x1, [x2, 392]
	ubfx	x3, x0, 5, 11
	ldr	w1, [x1, x3, lsl 2]
	lsr	w0, w1, w0
	and	w0, w0, 1
	ret
	.size	FtlBbmIsBadBlock, .-FtlBbmIsBadBlock
	.align	2
	.global	FtlBbtInfoPrint
	.type	FtlBbtInfoPrint, %function
FtlBbtInfoPrint:
	ret
	.size	FtlBbtInfoPrint, .-FtlBbtInfoPrint
	.align	2
	.global	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	mov	w4, 0
	mov	w5, 0
	ldrh	w6, [x0, 302]
	ldrh	w0, [x0, 258]
	mul	w6, w6, w0
	cmp	w4, w6
	blt	.L45
	mov	w0, w5
	ret
.L45:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
.L38:
	mov	w0, w4
	bl	FtlBbmIsBadBlock
	cbz	w0, .L37
	add	w5, w5, 1
	and	w5, w5, 65535
.L37:
	add	w4, w4, 1
	and	w4, w4, 65535
	cmp	w4, w6
	blt	.L38
	mov	w0, w5
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	2
	.global	V2P_block
	.type	V2P_block, %function
V2P_block:
	adrp	x4, .LANCHOR0
	add	x4, x4, :lo12:.LANCHOR0
	and	w0, w0, 65535
	and	w1, w1, 65535
	ldrh	w2, [x4, 260]
	ldrh	w4, [x4, 302]
	udiv	w3, w0, w2
	msub	w0, w3, w2, w0
	madd	w2, w2, w1, w0
	madd	w0, w3, w4, w2
	ret
	.size	V2P_block, .-V2P_block
	.align	2
	.global	P2V_plane
	.type	P2V_plane, %function
P2V_plane:
	adrp	x2, .LANCHOR0
	add	x2, x2, :lo12:.LANCHOR0
	and	w3, w0, 65535
	ldrh	w1, [x2, 260]
	ldrh	w2, [x2, 302]
	udiv	w0, w3, w1
	udiv	w2, w3, w2
	msub	w0, w0, w1, w3
	madd	w0, w1, w2, w0
	ret
	.size	P2V_plane, .-P2V_plane
	.align	2
	.global	P2V_block_in_plane
	.type	P2V_block_in_plane, %function
P2V_block_in_plane:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	and	w3, w0, 65535
	ldrh	w2, [x1, 302]
	ldrh	w1, [x1, 260]
	udiv	w0, w3, w2
	msub	w0, w0, w2, w3
	and	w0, w0, 65535
	udiv	w0, w0, w1
	ret
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	2
	.global	ftl_cmp_data_ver
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	cmp	w0, w1
	mov	w2, -2147483648
	bls	.L50
	sub	w1, w0, w1
	cmp	w1, w2
	cset	w0, ls
	ret
.L50:
	sub	w1, w1, w0
	cmp	w1, w2
	cset	w0, hi
	ret
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlFreeSysBlkQueueEmpty
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	adrp	x0, .LANCHOR0+462
	ldrh	w0, [x0, #:lo12:.LANCHOR0+462]
	cmp	w0, 0
	cset	w0, eq
	ret
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	2
	.global	FtlFreeSysBlkQueueFull
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	adrp	x0, .LANCHOR0+462
	ldrh	w0, [x0, #:lo12:.LANCHOR0+462]
	cmp	w0, 1024
	cset	w0, eq
	ret
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	adrp	x0, .LANCHOR0
	add	x2, x0, :lo12:.LANCHOR0
	add	x1, x2, 456
	ldrh	w3, [x1, 6]
	cbz	w3, .L54
	ldrh	w5, [x2, 2540]
	mov	w6, 0
	ldrh	w3, [x1, 2]
	mov	w4, 0
	ldrh	w2, [x1, 4]
	and	w5, w5, 31
.L56:
	cmp	w5, w4
	bgt	.L57
	cbz	w6, .L54
	add	x0, x0, :lo12:.LANCHOR0
	strh	w3, [x0, 458]
	strh	w2, [x0, 460]
.L54:
	ret
.L57:
	add	x6, x1, x3, sxtw 1
	add	w4, w4, 1
	add	w3, w3, 1
	and	w4, w4, 65535
	and	w3, w3, 1023
	ldrh	w7, [x6, 8]
	add	x6, x1, x2, sxtw 1
	strh	w7, [x6, 8]
	mov	w6, 1
	add	w2, w2, w6
	and	w2, w2, 1023
	b	.L56
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	IsInFreeQueue
	.type	IsInFreeQueue, %function
IsInFreeQueue:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 456
	and	w0, w0, 65535
	ldrh	w4, [x1, 6]
	cmp	w4, 1024
	beq	.L67
	ldrh	w5, [x1, 2]
	mov	w3, 0
.L65:
	cmp	w3, w4
	bcc	.L66
.L67:
	mov	w0, 0
	ret
.L66:
	add	w2, w3, w5
	ubfiz	x2, x2, 1, 10
	add	x2, x1, x2
	ldrh	w2, [x2, 8]
	cmp	w2, w0
	beq	.L68
	add	w3, w3, 1
	b	.L65
.L68:
	mov	w0, 1
	ret
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	adrp	x2, .LANCHOR0
	add	x3, x2, :lo12:.LANCHOR0
	and	w0, w0, 65535
	mov	x4, x2
	ldrh	w1, [x3, 244]
	cmp	w1, w0
	bls	.L71
	mov	w6, 6
	ldr	x8, [x3, 2560]
	mov	w1, -1
	umull	x7, w0, w6
	add	x13, x8, x7
	strh	w1, [x13, 2]
	strh	w1, [x8, x7]
	ldr	x1, [x3, 2568]
	cbnz	x1, .L72
	str	x13, [x3, 2568]
.L71:
	mov	w0, 0
	ret
.L72:
	ldr	x11, [x3, 72]
	ubfiz	x2, x0, 1, 16
	ldr	x10, [x3, 2560]
	mov	x9, -6148914691236517206
	movk	x9, 0xaaab, lsl 0
	ldrh	w14, [x3, 244]
	mov	w15, 65535
	mov	w3, 0
	ldrh	w5, [x11, x2]
	ldrh	w2, [x13, 4]
	cmp	w2, 0
	mul	w5, w5, w2
	sub	x2, x1, x10
	asr	x2, x2, 1
	csinv	w5, w5, wzr, ne
	mul	x2, x2, x9
	and	w2, w2, 65535
.L77:
	add	w3, w3, 1
	and	w3, w3, 65535
	cmp	w3, w14
	bhi	.L71
	cmp	w0, w2
	beq	.L71
	ldrh	w12, [x1, 4]
	cbz	w12, .L75
	ubfiz	x9, x2, 1, 16
	ldrh	w9, [x11, x9]
	mul	w9, w9, w12
	cmp	w9, w5
	bcs	.L75
	ldrh	w9, [x1]
	cmp	w9, w15
	bne	.L76
	strh	w2, [x13, 2]
	add	x2, x4, :lo12:.LANCHOR0
	strh	w0, [x1]
	str	x13, [x2, 2576]
	b	.L71
.L76:
	umaddl	x1, w9, w6, x10
	mov	w2, w9
	b	.L77
.L75:
	strh	w2, [x8, x7]
	ldrh	w2, [x1, 2]
	strh	w2, [x13, 2]
	add	x2, x4, :lo12:.LANCHOR0
	ldr	x3, [x2, 2568]
	cmp	x1, x3
	bne	.L78
	strh	w0, [x1, 2]
	str	x13, [x2, 2568]
	b	.L71
.L78:
	ldrh	w3, [x1, 2]
	mov	w4, 6
	ldr	x2, [x2, 2560]
	umull	x3, w3, w4
	strh	w0, [x2, x3]
	strh	w0, [x1, 2]
	b	.L71
	.size	insert_data_list, .-insert_data_list
	.align	2
	.global	INSERT_DATA_LIST
	.type	INSERT_DATA_LIST, %function
INSERT_DATA_LIST:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	insert_data_list
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w0, [x1, 2584]
	add	w0, w0, 1
	and	w0, w0, 65535
	strh	w0, [x1, 2584]
	ldrh	w1, [x1, 244]
	cmp	w1, w0
	bcs	.L83
	adrp	x1, .LANCHOR1
	adrp	x0, .LC8
	mov	w2, 214
	add	x1, x1, :lo12:.LANCHOR1
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L83:
	ldp	x29, x30, [sp], 16
	ret
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	2
	.global	insert_free_list
	.type	insert_free_list, %function
insert_free_list:
	and	w0, w0, 65535
	mov	w7, 65535
	cmp	w0, w7
	beq	.L87
	adrp	x2, .LANCHOR0
	add	x5, x2, :lo12:.LANCHOR0
	mov	w6, 6
	mov	w1, -1
	mov	x3, x2
	ldr	x9, [x5, 2560]
	umull	x8, w0, w6
	add	x4, x9, x8
	strh	w1, [x4, 2]
	strh	w1, [x9, x8]
	ldr	x1, [x5, 2592]
	cbnz	x1, .L88
	str	x4, [x5, 2592]
.L87:
	mov	w0, 0
	ret
.L88:
	ldr	x11, [x5, 2600]
	ubfiz	x2, x0, 1, 16
	ldr	x10, [x5, 2560]
	mov	x5, -6148914691236517206
	movk	x5, 0xaaab, lsl 0
	ldrh	w12, [x11, x2]
	sub	x2, x1, x10
	asr	x2, x2, 1
	mul	x2, x2, x5
	and	w2, w2, 65535
.L91:
	ubfiz	x5, x2, 1, 16
	ldrh	w5, [x11, x5]
	cmp	w5, w12
	bcs	.L89
	ldrh	w5, [x1]
	cmp	w5, w7
	bne	.L90
	strh	w2, [x4, 2]
	strh	w0, [x1]
	b	.L87
.L90:
	umaddl	x1, w5, w6, x10
	mov	w2, w5
	b	.L91
.L89:
	ldrh	w5, [x1, 2]
	strh	w5, [x4, 2]
	strh	w2, [x9, x8]
	add	x2, x3, :lo12:.LANCHOR0
	ldr	x3, [x2, 2592]
	cmp	x1, x3
	bne	.L92
	strh	w0, [x1, 2]
	str	x4, [x2, 2592]
	b	.L87
.L92:
	ldrh	w3, [x1, 2]
	mov	w4, 6
	ldr	x2, [x2, 2560]
	umull	x3, w3, w4
	strh	w0, [x2, x3]
	strh	w0, [x1, 2]
	b	.L87
	.size	insert_free_list, .-insert_free_list
	.align	2
	.global	INSERT_FREE_LIST
	.type	INSERT_FREE_LIST, %function
INSERT_FREE_LIST:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	insert_free_list
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w0, [x1, 228]
	add	w0, w0, 1
	and	w0, w0, 65535
	strh	w0, [x1, 228]
	ldrh	w1, [x1, 244]
	cmp	w1, w0
	bcs	.L93
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 207
	add	x1, x1, 24
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L93:
	ldp	x29, x30, [sp], 16
	ret
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.type	List_remove_node, %function
List_remove_node:
	stp	x29, x30, [sp, -64]!
	and	w1, w1, 65535
	mov	w2, 6
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	str	x23, [sp, 48]
	mov	x23, x0
	add	x0, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	umull	x21, w1, w2
	ldr	x22, [x0, 2560]
	mov	w0, 65535
	add	x20, x22, x21
	ldrh	w1, [x20, 2]
	cmp	w1, w0
	bne	.L97
	ldr	x0, [x23]
	cmp	x20, x0
	beq	.L97
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 372
	add	x1, x1, 48
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L97:
	ldr	x0, [x23]
	mov	w1, 65535
	cmp	x20, x0
	ldrh	w0, [x22, x21]
	bne	.L98
	cmp	w0, w1
	bne	.L99
	str	xzr, [x23]
.L100:
	mov	w0, -1
	strh	w0, [x22, x21]
	strh	w0, [x20, 2]
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldr	x23, [sp, 48]
	ldp	x29, x30, [sp], 64
	ret
.L99:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w2, 6
	ldr	x1, [x19, 2560]
	umaddl	x0, w0, w2, x1
	mov	w1, -1
	str	x0, [x23]
	strh	w1, [x0, 2]
	b	.L100
.L98:
	cmp	w0, w1
	ldrh	w1, [x20, 2]
	bne	.L101
	cmp	w1, w0
	beq	.L100
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, 6
	mov	w2, -1
	umull	x1, w1, w0
	ldr	x0, [x19, 2560]
	strh	w2, [x0, x1]
	b	.L100
.L101:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w2, 6
	ldr	x3, [x19, 2560]
	umaddl	x0, w0, w2, x3
	strh	w1, [x0, 2]
	ldrh	w0, [x20, 2]
	ldr	x1, [x19, 2560]
	ldrh	w3, [x22, x21]
	umull	x0, w0, w2
	strh	w3, [x1, x0]
	b	.L100
	.size	List_remove_node, .-List_remove_node
	.align	2
	.global	List_pop_index_node
	.type	List_pop_index_node, %function
List_pop_index_node:
	ldr	x2, [x0]
	cbz	x2, .L109
	stp	x29, x30, [sp, -32]!
	adrp	x3, .LANCHOR0+2560
	and	w1, w1, 65535
	mov	w4, 65535
	add	x29, sp, 0
	str	x19, [sp, 16]
	mov	w5, 6
	ldr	x19, [x3, #:lo12:.LANCHOR0+2560]
.L105:
	cbnz	w1, .L106
.L108:
	sub	x19, x2, x19
	mov	x2, -6148914691236517206
	asr	x19, x19, 1
	movk	x2, 0xaaab, lsl 0
	mul	x19, x19, x2
	and	w19, w19, 65535
	mov	w1, w19
	bl	List_remove_node
	mov	w0, w19
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L106:
	ldrh	w3, [x2]
	cmp	w3, w4
	beq	.L108
	sub	w1, w1, #1
	umaddl	x2, w3, w5, x19
	and	w1, w1, 65535
	b	.L105
.L109:
	mov	w0, 65535
	ret
	.size	List_pop_index_node, .-List_pop_index_node
	.align	2
	.global	List_pop_head_node
	.type	List_pop_head_node, %function
List_pop_head_node:
	stp	x29, x30, [sp, -16]!
	mov	w1, 0
	add	x29, sp, 0
	bl	List_pop_index_node
	ldp	x29, x30, [sp], 16
	ret
	.size	List_pop_head_node, .-List_pop_head_node
	.align	2
	.global	List_get_gc_head_node
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	and	w2, w0, 65535
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldr	x1, [x0, 2568]
	cbz	x1, .L121
	ldr	x3, [x0, 2560]
	mov	w4, 6
	mov	w0, 65535
.L118:
	cbz	w2, .L119
	ldrh	w1, [x1]
	cmp	w1, w0
	bne	.L120
	ret
.L120:
	sub	w2, w2, #1
	umaddl	x1, w1, w4, x3
	and	w2, w2, 65535
	b	.L118
.L121:
	mov	w0, 65535
	ret
.L119:
	sub	x0, x1, x3
	mov	x1, -6148914691236517206
	asr	x0, x0, 1
	movk	x1, 0xaaab, lsl 0
	mul	x0, x0, x1
	and	w0, w0, 65535
	ret
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	2
	.global	List_update_data_list
	.type	List_update_data_list, %function
List_update_data_list:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	and	w20, w0, 65535
	adrp	x19, .LANCHOR0
	add	x0, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	ldrh	w1, [x0, 24]
	cmp	w1, w20
	beq	.L124
	ldrh	w1, [x0, 80]
	cmp	w1, w20
	beq	.L124
	ldrh	w1, [x0, 128]
	cmp	w1, w20
	beq	.L124
	mov	w22, 6
	ldr	x24, [x0, 2560]
	ldr	x1, [x0, 2568]
	umull	x22, w20, w22
	add	x23, x24, x22
	cmp	x23, x1
	beq	.L124
	ldr	x2, [x0, 72]
	ubfiz	x1, x20, 1, 16
	ldrh	w0, [x23, 4]
	cmp	w0, 0
	ldrh	w21, [x2, x1]
	mov	w1, 65535
	mul	w21, w21, w0
	ldrh	w0, [x23, 2]
	csinv	w21, w21, wzr, ne
	cmp	w0, w1
	bne	.L127
	ldrh	w1, [x24, x22]
	cmp	w1, w0
	bne	.L127
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 463
	add	x1, x1, 72
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L127:
	ldrh	w0, [x23, 2]
	mov	w1, 65535
	cmp	w0, w1
	bne	.L128
	ldrh	w1, [x24, x22]
	cmp	w1, w0
	beq	.L124
.L128:
	mov	w1, 6
	add	x22, x19, :lo12:.LANCHOR0
	mov	x2, -6148914691236517206
	umull	x0, w0, w1
	movk	x2, 0xaaab, lsl 0
	asr	x1, x0, 1
	mul	x1, x1, x2
	ldr	x2, [x22, 72]
	ldrh	w1, [x2, x1, lsl 1]
	ldr	x2, [x22, 2560]
	add	x0, x2, x0
	ldrh	w2, [x0, 4]
	cmp	w2, 0
	mul	w0, w1, w2
	csinv	w0, w0, wzr, ne
	cmp	w21, w0
	bcs	.L124
	mov	w1, w20
	add	x0, x22, 2568
	bl	List_remove_node
	ldrh	w0, [x22, 2584]
	cbnz	w0, .L130
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 474
	add	x1, x1, 72
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L130:
	add	x19, x19, :lo12:.LANCHOR0
	ldrh	w0, [x19, 2584]
	sub	w0, w0, #1
	strh	w0, [x19, 2584]
	mov	w0, w20
	bl	INSERT_DATA_LIST
.L124:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 64
	ret
	.size	List_update_data_list, .-List_update_data_list
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	stp	x29, x30, [sp, -32]!
	adrp	x1, .LANCHOR0
	add	x0, x1, :lo12:.LANCHOR0
	mov	x3, 0
	add	x29, sp, 0
	str	x19, [sp, 16]
	mov	w4, 65535
	ldrh	w2, [x0, 342]
	ldr	x0, [x0, 2608]
.L135:
	and	w19, w3, 65535
	cmp	w19, w2
	bcc	.L137
	add	x4, x0, 4
	mov	w19, w2
	mov	w6, -2147483648
	mov	w3, 0
.L138:
	cmp	w3, w2
	bne	.L140
	cmp	w19, w2
	bcc	.L136
	add	x1, x1, :lo12:.LANCHOR0
	mov	w19, w2
	mov	w3, -1
	ldrh	w4, [x1, 2616]
	mov	w1, 0
.L141:
	cmp	w1, w2
	bne	.L143
	cmp	w19, w1
	bcc	.L136
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	mov	w2, 789
	add	x1, x1, 96
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	b	.L136
.L137:
	add	x3, x3, 1
	add	x5, x0, x3, lsl 4
	ldrh	w5, [x5, -16]
	cmp	w5, w4
	bne	.L135
.L136:
	mov	w0, w19
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L140:
	ldr	w5, [x4]
	tbnz	w5, #31, .L139
	cmp	w6, w5
	bls	.L139
	mov	w6, w5
	mov	w19, w3
.L139:
	add	w3, w3, 1
	add	x4, x4, 16
	and	w3, w3, 65535
	b	.L138
.L143:
	ldr	w6, [x0, 4]
	cmp	w3, w6
	bls	.L142
	ldrh	w5, [x0]
	cmp	w5, w4
	csel	w3, w3, w6, eq
	csel	w19, w19, w1, eq
.L142:
	add	w1, w1, 1
	add	x0, x0, 16
	and	w1, w1, 65535
	b	.L141
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	adrp	x1, .LANCHOR0
	add	x3, x1, :lo12:.LANCHOR0
	ldrh	w2, [x3, 2618]
	cmp	w2, 4
	bhi	.L146
	cbnz	w0, .L146
	add	w2, w2, 1
	strh	w2, [x3, 2618]
	ret
.L146:
	add	x0, x1, :lo12:.LANCHOR0
	mov	w6, 65535
	mov	x1, 0
	ldrh	w4, [x0, 244]
	ldr	x5, [x0, 72]
	strh	wzr, [x0, 2618]
	str	wzr, [x0, 2620]
.L147:
	cmp	w4, w1, uxth
	bhi	.L149
	ret
.L149:
	ldrh	w3, [x5, x1, lsl 1]
	cmp	w3, w6
	beq	.L148
	ldr	w2, [x0, 2620]
	add	w2, w2, w3
	str	w2, [x0, 2620]
.L148:
	add	x1, x1, 1
	b	.L147
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	and	w6, w1, 65535
	adrp	x4, .LANCHOR0
	add	x1, x4, :lo12:.LANCHOR0
	and	w2, w2, 65535
	strh	wzr, [x0, 4]
	ldrh	w3, [x1, 236]
	mov	w1, 65535
.L152:
	cmp	w3, w2, uxth
	bhi	.L154
	add	x4, x4, :lo12:.LANCHOR0
	ubfiz	x3, x3, 1, 16
	add	x3, x3, 16
	add	x2, x0, 16
	add	x3, x0, x3
	mov	w5, 65535
	ldrh	w1, [x4, 306]
	sub	w1, w1, #1
	and	w1, w1, 65535
	sub	w1, w1, w6
.L155:
	cmp	x2, x3
	bne	.L157
	ret
.L154:
	add	x5, x0, x2, sxtw 1
	ldrh	w5, [x5, 16]
	cmp	w5, w1
	beq	.L153
	ldrh	w5, [x0, 4]
	add	w5, w5, 1
	strh	w5, [x0, 4]
.L153:
	add	w2, w2, 1
	b	.L152
.L157:
	ldrh	w4, [x2]
	cmp	w4, w5
	beq	.L156
	ldrh	w4, [x0, 4]
	add	w4, w1, w4
	strh	w4, [x0, 4]
.L156:
	add	x2, x2, 2
	b	.L155
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.align	2
	.global	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	ldrh	w1, [x0, 4]
	cbz	w1, .L158
	ldrh	w2, [x0]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L158
	ldrb	w2, [x0, 6]
	add	x2, x2, 8
	ldrh	w3, [x0, x2, lsl 1]
	adrp	x2, .LANCHOR0+236
	ldrh	w4, [x2, #:lo12:.LANCHOR0+236]
	mov	w2, w1
.L161:
	cmp	w3, w2
	beq	.L163
.L158:
	ret
.L163:
	ldrb	w1, [x0, 6]
	add	w1, w1, 1
	and	w1, w1, 255
	strb	w1, [x0, 6]
	cmp	w1, w4
	bne	.L162
	ldrh	w1, [x0, 2]
	strb	wzr, [x0, 6]
	add	w1, w1, 1
	strh	w1, [x0, 2]
.L162:
	ldrb	w1, [x0, 6]
	add	x1, x1, 8
	ldrh	w3, [x0, x1, lsl 1]
	b	.L161
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	2
	.global	make_superblock
	.type	make_superblock, %function
make_superblock:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	adrp	x20, .LANCHOR0
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w1, [x19]
	ldrh	w0, [x0, 244]
	cmp	w1, w0
	bcc	.L165
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2150
	add	x1, x1, 120
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L165:
	add	x0, x20, :lo12:.LANCHOR0
	add	x6, x19, 16
	add	x7, x0, 264
	strh	wzr, [x19, 4]
	strb	wzr, [x19, 7]
	mov	x5, 0
	ldrh	w8, [x0, 236]
	mov	w9, -1
.L166:
	cmp	w8, w5, uxth
	bhi	.L168
	add	x20, x20, :lo12:.LANCHOR0
	ldrb	w0, [x19, 7]
	strb	wzr, [x19, 9]
	ldrh	w1, [x20, 306]
	mul	w0, w0, w1
	ldrh	w1, [x19]
	strh	w0, [x19, 4]
	ldr	x0, [x20, 2600]
	ldrh	w1, [x0, x1, lsl 1]
	mov	w0, 10000
	cmp	w1, w0
	bls	.L169
	mov	w0, 1
	strb	w0, [x19, 9]
.L169:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L168:
	ldrh	w1, [x19]
	ldrb	w0, [x7, x5]
	bl	V2P_block
	mov	w4, w0
	strh	w9, [x6]
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L167
	strh	w4, [x6]
	ldrb	w0, [x19, 7]
	add	w0, w0, 1
	strb	w0, [x19, 7]
.L167:
	add	x5, x5, 1
	add	x6, x6, 2
	b	.L166
	.size	make_superblock, .-make_superblock
	.align	2
	.global	update_multiplier_value
	.type	update_multiplier_value, %function
update_multiplier_value:
	and	w8, w0, 65535
	adrp	x0, .LANCHOR0
	add	x1, x0, :lo12:.LANCHOR0
	mov	x9, 0
	mov	w6, 0
	mov	x5, x0
	add	x7, x1, 264
	ldrh	w10, [x1, 236]
	ldrh	w11, [x1, 306]
	cmp	w10, w9, uxth
	bhi	.L183
	cbz	w6, .L181
	mov	w0, 32768
	sdiv	w6, w0, w6
.L182:
	add	x0, x5, :lo12:.LANCHOR0
	mov	w1, 6
	ldr	x0, [x0, 2560]
	umaddl	x8, w8, w1, x0
	mov	w0, 0
	strh	w6, [x8, 4]
	ret
.L176:
	mov	w6, 0
	b	.L175
.L181:
	mov	w6, 0
	b	.L182
.L183:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
.L174:
	ldrb	w0, [x7, x9]
	mov	w1, w8
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L173
	add	w6, w6, w11
	and	w6, w6, 65535
.L173:
	add	x9, x9, 1
	cmp	w10, w9, uxth
	bhi	.L174
	cbz	w6, .L176
	mov	w0, 32768
	sdiv	w6, w0, w6
.L175:
	add	x0, x5, :lo12:.LANCHOR0
	mov	w1, 6
	ldr	x0, [x0, 2560]
	umaddl	x8, w8, w1, x0
	mov	w0, 0
	strh	w6, [x8, 4]
	ldp	x29, x30, [sp], 16
	ret
	.size	update_multiplier_value, .-update_multiplier_value
	.align	2
	.global	GetFreeBlockMinEraseCount
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldr	x0, [x1, 2592]
	cbz	x0, .L186
	ldr	x2, [x1, 2560]
	ldr	x1, [x1, 2600]
	sub	x0, x0, x2
	mov	x2, -6148914691236517206
	asr	x0, x0, 1
	movk	x2, 0xaaab, lsl 0
	mul	x0, x0, x2
	and	x0, x0, 65535
	ldrh	w0, [x1, x0, lsl 1]
	ret
.L186:
	mov	w0, 0
	ret
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	2
	.global	GetFreeBlockMaxEraseCount
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	adrp	x2, .LANCHOR0
	add	x4, x2, :lo12:.LANCHOR0
	and	w0, w0, 65535
	ldr	x1, [x4, 2592]
	cbz	x1, .L194
	ldrh	w3, [x4, 228]
	mov	w5, 7
	ldr	x4, [x4, 2560]
	mov	w6, 6
	mov	w7, 65535
	sub	x1, x1, x4
	mul	w3, w3, w5
	asr	x1, x1, 1
	asr	w3, w3, 3
	cmp	w0, w3
	csel	w0, w3, w0, gt
	mov	x3, -6148914691236517206
	movk	x3, 0xaaab, lsl 0
	mul	x1, x1, x3
	mov	w3, 0
	and	w1, w1, 65535
.L190:
	cmp	w0, w3
	beq	.L193
	umull	x5, w1, w6
	ldrh	w5, [x4, x5]
	cmp	w5, w7
	bne	.L191
.L193:
	add	x2, x2, :lo12:.LANCHOR0
	ubfiz	x1, x1, 1, 16
	ldr	x0, [x2, 2600]
	ldrh	w0, [x0, x1]
	ret
.L191:
	add	w3, w3, 1
	mov	w1, w5
	and	w3, w3, 65535
	b	.L190
.L194:
	mov	w0, 0
	ret
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	2
	.global	FtlPrintInfo2buf
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	stp	x29, x30, [sp, -96]!
	adrp	x1, .LC9
	add	x1, x1, :lo12:.LC9
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	mov	x22, x0
	stp	x23, x24, [sp, 48]
	adrp	x23, .LANCHOR0
	add	x21, x23, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	stp	x25, x26, [sp, 64]
	add	x20, x0, 12
	str	x27, [sp, 80]
	bl	strcpy
	ldr	w3, [x21, 320]
	adrp	x2, .LC10
	add	x2, x2, :lo12:.LC10
	mov	x1, 64
	mov	x0, x20
	bl	snprintf
	add	x20, x20, x0, sxtw
	mov	x0, x20
	adrp	x1, .LC11
	add	x1, x1, :lo12:.LC11
	bl	strcpy
	ldr	w3, [x21, 2624]
	add	x20, x20, 10
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC12
	add	x2, x2, :lo12:.LC12
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2620]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC13
	add	x2, x2, :lo12:.LC13
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2628]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC14
	add	x2, x2, :lo12:.LC14
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2632]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC15
	add	x2, x2, :lo12:.LC15
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2636]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC16
	add	x2, x2, :lo12:.LC16
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2640]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC17
	add	x2, x2, :lo12:.LC17
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2644]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC18
	add	x2, x2, :lo12:.LC18
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2648]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC19
	add	x2, x2, :lo12:.LC19
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2652]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC20
	add	x2, x2, :lo12:.LC20
	lsr	w3, w3, 11
	bl	snprintf
	ldr	w3, [x21, 2656]
	add	x20, x20, x0, sxtw
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC21
	add	x2, x2, :lo12:.LC21
	lsr	w3, w3, 11
	bl	snprintf
	ldr	w3, [x21, 2660]
	add	x20, x20, x0, sxtw
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC22
	add	x2, x2, :lo12:.LC22
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2664]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC23
	add	x2, x2, :lo12:.LC23
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 366]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC24
	add	x2, x2, :lo12:.LC24
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 228]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC25
	add	x2, x2, :lo12:.LC25
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2668]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC26
	add	x2, x2, :lo12:.LC26
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2672]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC27
	add	x2, x2, :lo12:.LC27
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2676]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC28
	add	x2, x2, :lo12:.LC28
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2680]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC29
	add	x2, x2, :lo12:.LC29
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2684]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC30
	add	x2, x2, :lo12:.LC30
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 2688]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC31
	add	x2, x2, :lo12:.LC31
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 2542]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC32
	add	x2, x2, :lo12:.LC32
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 2540]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC33
	add	x2, x2, :lo12:.LC33
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 344]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC34
	add	x2, x2, :lo12:.LC34
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 336]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC35
	add	x2, x2, :lo12:.LC35
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 232]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC36
	add	x2, x2, :lo12:.LC36
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 462]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC37
	add	x2, x2, :lo12:.LC37
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 244]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC38
	add	x2, x2, :lo12:.LC38
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 2692]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC39
	add	x2, x2, :lo12:.LC39
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w3, [x21, 248]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC40
	add	x2, x2, :lo12:.LC40
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 2696]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC41
	add	x2, x2, :lo12:.LC41
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 360]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC42
	add	x2, x2, :lo12:.LC42
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 26]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC43
	add	x2, x2, :lo12:.LC43
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 30]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC44
	add	x2, x2, :lo12:.LC44
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 24]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC45
	add	x2, x2, :lo12:.LC45
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 32]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC46
	add	x2, x2, :lo12:.LC46
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 28]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC47
	add	x2, x2, :lo12:.LC47
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w1, [x21, 24]
	adrp	x2, .LC48
	ldr	x0, [x21, 72]
	add	x2, x2, :lo12:.LC48
	ldrh	w3, [x0, x1, lsl 1]
	mov	x1, 64
	mov	x0, x20
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 82]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC49
	add	x2, x2, :lo12:.LC49
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 86]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC50
	add	x2, x2, :lo12:.LC50
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 80]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC51
	add	x2, x2, :lo12:.LC51
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 88]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC52
	add	x2, x2, :lo12:.LC52
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 84]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC53
	add	x2, x2, :lo12:.LC53
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w1, [x21, 80]
	adrp	x2, .LC54
	ldr	x0, [x21, 72]
	add	x2, x2, :lo12:.LC54
	ldrh	w3, [x0, x1, lsl 1]
	mov	x1, 64
	mov	x0, x20
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 130]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC55
	add	x2, x2, :lo12:.LC55
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 134]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC56
	add	x2, x2, :lo12:.LC56
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 128]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC57
	add	x2, x2, :lo12:.LC57
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 136]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC58
	add	x2, x2, :lo12:.LC58
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 132]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC59
	add	x2, x2, :lo12:.LC59
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 178]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC60
	add	x2, x2, :lo12:.LC60
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 182]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC61
	add	x2, x2, :lo12:.LC61
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 176]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC62
	add	x2, x2, :lo12:.LC62
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrb	w3, [x21, 184]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC63
	add	x2, x2, :lo12:.LC63
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldrh	w3, [x21, 180]
	mov	x1, 64
	mov	x0, x20
	adrp	x2, .LC64
	add	x2, x2, :lo12:.LC64
	bl	snprintf
	add	x20, x20, x0, sxtw
	ldr	w5, [x21, 2788]
	mov	x1, 64
	ldr	w4, [x21, 2796]
	adrp	x2, .LC65
	ldr	w3, [x21, 2792]
	add	x2, x2, :lo12:.LC65
	mov	x0, x20
	bl	snprintf
	add	x19, x20, x0, sxtw
	ldr	w3, [x21, 2784]
	mov	x1, 64
	mov	x0, x19
	adrp	x2, .LC66
	add	x2, x2, :lo12:.LC66
	bl	snprintf
	add	x19, x19, x0, sxtw
	ldr	w3, [x21, 2808]
	mov	x1, 64
	mov	x0, x19
	adrp	x2, .LC67
	add	x2, x2, :lo12:.LC67
	bl	snprintf
	add	x19, x19, x0, sxtw
	ldrh	w3, [x21, 3224]
	mov	x1, 64
	mov	x0, x19
	adrp	x2, .LC68
	add	x2, x2, :lo12:.LC68
	bl	snprintf
	add	x19, x19, x0, sxtw
	ldrh	w3, [x21, 3226]
	mov	x1, 64
	mov	x0, x19
	adrp	x2, .LC69
	add	x2, x2, :lo12:.LC69
	bl	snprintf
	add	x19, x19, x0, sxtw
	ldr	w3, [x21, 3228]
	mov	x1, 64
	mov	x0, x19
	adrp	x2, .LC70
	add	x2, x2, :lo12:.LC70
	bl	snprintf
	add	x19, x19, x0, sxtw
	ldrh	w3, [x21, 3232]
	adrp	x2, .LC71
	add	x2, x2, :lo12:.LC71
	mov	x1, 64
	mov	x0, x19
	bl	snprintf
	add	x19, x19, x0, sxtw
	bl	GetFreeBlockMinEraseCount
	and	w3, w0, 65535
	mov	x1, 64
	mov	x0, x19
	adrp	x2, .LC72
	add	x2, x2, :lo12:.LC72
	bl	snprintf
	add	x19, x19, x0, sxtw
	ldrh	w0, [x21, 228]
	bl	GetFreeBlockMaxEraseCount
	and	w3, w0, 65535
	adrp	x2, .LC73
	mov	x0, x19
	add	x2, x2, :lo12:.LC73
	mov	x1, 64
	bl	snprintf
	add	x19, x19, x0, sxtw
	adrp	x0, .LANCHOR2
	ldr	w0, [x0, #:lo12:.LANCHOR2]
	cmp	w0, 1
	beq	.L196
.L201:
	sub	w0, w19, w22
	ldr	x27, [sp, 80]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 96
	ret
.L196:
	ldrh	w0, [x21, 176]
	mov	w1, 65535
	cmp	w0, w1
	beq	.L198
	ldr	x1, [x21, 72]
	ubfiz	x0, x0, 1, 16
	adrp	x2, .LC74
	add	x2, x2, :lo12:.LC74
	ldrh	w3, [x1, x0]
	mov	x0, x19
	mov	x1, 64
	bl	snprintf
	add	x19, x19, x0, sxtw
.L198:
	mov	w0, 0
	adrp	x24, .LC75
	bl	List_get_gc_head_node
	add	x21, x23, :lo12:.LANCHOR0
	and	w4, w0, 65535
	add	x24, x24, :lo12:.LC75
	mov	w20, 0
	mov	w27, 65535
	mov	w26, 6
.L200:
	cmp	w4, w27
	beq	.L199
	umull	x25, w4, w26
	ldr	x0, [x21, 2560]
	ldr	x2, [x21, 72]
	ubfiz	x1, x4, 1, 16
	ldr	x3, [x21, 2600]
	add	x0, x0, x25
	ldrh	w5, [x2, x1]
	mov	x2, x24
	ldrh	w7, [x3, x1]
	mov	w3, w20
	ldrh	w6, [x0, 4]
	mov	x1, 64
	mov	x0, x19
	add	w20, w20, 1
	bl	snprintf
	add	x19, x19, x0, sxtw
	ldr	x0, [x21, 2560]
	cmp	w20, 16
	ldrh	w4, [x0, x25]
	bne	.L200
.L199:
	add	x23, x23, :lo12:.LANCHOR0
	adrp	x21, .LC76
	add	x21, x21, :lo12:.LC76
	mov	w20, 0
	mov	w25, 65535
	mov	w26, 6
	ldr	x0, [x23, 2560]
	ldr	x4, [x23, 2592]
	sub	x4, x4, x0
	mov	x0, -6148914691236517206
	asr	x4, x4, 1
	movk	x0, 0xaaab, lsl 0
	mul	x4, x4, x0
	and	w4, w4, 65535
.L202:
	cmp	w4, w25
	beq	.L201
	umull	x24, w4, w26
	ldr	x0, [x23, 2560]
	ldr	x2, [x23, 2600]
	ubfiz	x1, x4, 1, 16
	add	x0, x0, x24
	mov	w3, w20
	add	w20, w20, 1
	ldrh	w6, [x2, x1]
	mov	x2, x21
	ldrh	w5, [x0, 4]
	mov	x1, 64
	mov	x0, x19
	bl	snprintf
	add	x19, x19, x0, sxtw
	cmp	w20, 4
	beq	.L201
	ldr	x0, [x23, 2560]
	ldrh	w4, [x0, x24]
	b	.L202
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	2
	.global	rknand_proc_ftlread
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	cmp	w0, 2047
	ble	.L210
	stp	x29, x30, [sp, -32]!
	adrp	x3, .LC0
	adrp	x2, .LC77
	add	x3, x3, :lo12:.LC0
	add	x29, sp, 0
	add	x2, x2, :lo12:.LC77
	stp	x19, x20, [sp, 16]
	mov	x20, x1
	mov	x1, 64
	mov	x0, x20
	bl	snprintf
	add	x19, x20, x0, sxtw
	mov	x0, x19
	bl	FtlPrintInfo2buf
	add	x0, x19, x0, sxtw
	sub	w0, w0, w20
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L210:
	mov	w0, 0
	ret
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	2
	.global	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	stp	x29, x30, [sp, -32]!
	adrp	x8, .LANCHOR0
	add	x0, x8, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	ldr	w2, [x0, 2676]
	ldr	w1, [x0, 2688]
	cmp	w2, w1
	bcs	.L216
	ldrh	w2, [x0, 244]
	mov	x1, 0
	ldr	x4, [x0, 2600]
	str	wzr, [x0, 2668]
.L217:
	cmp	w2, w1
	bhi	.L218
	add	x1, x8, :lo12:.LANCHOR0
	ldr	w0, [x1, 2668]
	udiv	w2, w0, w2
	str	w2, [x1, 2676]
	ldr	w2, [x1, 2672]
	sub	w0, w0, w2
	ldrh	w2, [x1, 296]
	udiv	w0, w0, w2
	str	w0, [x1, 2668]
.L219:
	add	x1, x8, :lo12:.LANCHOR0
	ldr	w9, [x1, 2688]
	ldr	w11, [x1, 2676]
	add	w0, w9, 256
	cmp	w0, w11
	bls	.L224
	ldr	w1, [x1, 2684]
	add	w0, w9, 768
	cmp	w0, w1
	bls	.L224
.L226:
	mov	w20, 65535
.L225:
	mov	w0, w20
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L218:
	ldrh	w3, [x4, x1, lsl 1]
	add	x1, x1, 1
	ldr	w5, [x0, 2668]
	add	w3, w3, w5
	str	w3, [x0, 2668]
	b	.L217
.L216:
	ldr	w1, [x0, 2684]
	cmp	w2, w1
	bls	.L219
	add	w1, w1, 1
	str	w1, [x0, 2684]
	mov	w1, 0
.L221:
	ldrh	w2, [x0, 244]
	cmp	w1, w2
	bcs	.L219
	ldr	x4, [x0, 2600]
	ubfiz	x3, x1, 1, 32
	add	w1, w1, 1
	ldrh	w2, [x4, x3]
	add	w2, w2, 1
	strh	w2, [x4, x3]
	b	.L221
.L224:
	add	x10, x8, :lo12:.LANCHOR0
	ldrh	w0, [x10, 228]
	add	w0, w0, w0, lsl 1
	lsr	w0, w0, 2
	bl	GetFreeBlockMaxEraseCount
	and	w6, w0, 65535
	add	w0, w9, 64
	cmp	w6, w0
	bcc	.L226
	ldr	x0, [x10, 2568]
	cbz	x0, .L226
	mov	w4, 65535
	ldrh	w12, [x10, 244]
	mov	x13, -6148914691236517206
	ldr	x7, [x10, 2560]
	ldr	x5, [x10, 2600]
	mov	w2, w4
	mov	w10, w4
	mov	w1, 0
	movk	x13, 0xaaab, lsl 0
	mov	w14, 6
.L227:
	ldrh	w3, [x0]
	cmp	w3, w10
	bne	.L230
	mov	w20, w2
.L229:
	mov	w0, 65535
	cmp	w20, w0
	beq	.L226
	ubfiz	x7, x20, 1, 32
	ldrh	w3, [x5, x7]
	cmp	w9, w3
	bcs	.L231
	bl	GetFreeBlockMinEraseCount
	cmp	w9, w0, uxth
	bcs	.L231
	add	x0, x8, :lo12:.LANCHOR0
	str	w4, [x0, 2688]
.L231:
	cmp	w11, w3
	bls	.L226
	add	w0, w3, 128
	cmp	w6, w0
	ble	.L226
	add	w0, w3, 256
	cmp	w11, w0
	bhi	.L232
	add	x0, x8, :lo12:.LANCHOR0
	add	w3, w3, 768
	ldr	w0, [x0, 2684]
	cmp	w3, w0
	bcs	.L226
.L232:
	add	x19, x8, :lo12:.LANCHOR0
	ldrh	w5, [x5, x7]
	mov	w2, w11
	mov	w1, w20
	ldr	x0, [x19, 72]
	ldr	w3, [x19, 2684]
	ldrh	w4, [x0, x7]
	adrp	x0, .LC78
	add	x0, x0, :lo12:.LC78
	bl	sftl_printk
	mov	w0, 1
	str	w0, [x19, 3236]
	b	.L225
.L230:
	add	w1, w1, 1
	and	w1, w1, 65535
	cmp	w1, w12
	bhi	.L226
	ldrh	w15, [x0, 4]
	cbz	w15, .L228
	sub	x0, x0, x7
	asr	x0, x0, 1
	mul	x0, x0, x13
	and	w20, w0, 65535
	and	x0, x0, 65535
	ldrh	w0, [x5, x0, lsl 1]
	cmp	w9, w0
	bcs	.L229
	cmp	w4, w0
	bls	.L228
	mov	w4, w0
	mov	w2, w20
.L228:
	umaddl	x0, w3, w14, x7
	b	.L227
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.align	2
	.global	free_data_superblock
	.type	free_data_superblock, %function
free_data_superblock:
	and	w0, w0, 65535
	mov	w1, 65535
	cmp	w0, w1
	beq	.L243
	stp	x29, x30, [sp, -16]!
	adrp	x2, .LANCHOR0+72
	ubfiz	x1, x0, 1, 16
	add	x29, sp, 0
	ldr	x2, [x2, #:lo12:.LANCHOR0+72]
	strh	wzr, [x2, x1]
	bl	INSERT_FREE_LIST
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
.L243:
	mov	w0, 0
	ret
	.size	free_data_superblock, .-free_data_superblock
	.align	2
	.global	get_new_active_ppa
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	ldrh	w1, [x0]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L247
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2785
	add	x1, x1, 136
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L247:
	adrp	x20, .LANCHOR0
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w1, [x19, 2]
	ldrh	w0, [x0, 306]
	cmp	w1, w0
	bne	.L248
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2786
	add	x1, x1, 136
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L248:
	ldrh	w0, [x19, 4]
	cbnz	w0, .L249
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2787
	add	x1, x1, 136
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L249:
	ldrb	w0, [x19, 6]
	add	x1, x20, :lo12:.LANCHOR0
	strb	wzr, [x19, 10]
	add	x0, x0, 8
	ldrh	w3, [x1, 236]
	mov	w1, 65535
	ldrh	w0, [x19, x0, lsl 1]
.L250:
	cmp	w0, w1
	beq	.L252
	add	x20, x20, :lo12:.LANCHOR0
	ldrh	w1, [x19, 2]
	ldrh	w2, [x20, 306]
	cmp	w1, w2
	bcs	.L256
	orr	w20, w1, w0, lsl 10
	ldrh	w1, [x19, 4]
	ldrb	w0, [x19, 6]
	mov	w4, 65535
	sub	w1, w1, #1
	and	w1, w1, 65535
	strh	w1, [x19, 4]
.L255:
	add	w0, w0, 1
	and	w0, w0, 255
	cmp	w3, w0
	bne	.L254
	ldrh	w0, [x19, 2]
	add	w0, w0, 1
	strh	w0, [x19, 2]
	mov	w0, 0
.L254:
	add	x5, x19, x0, sxtw 1
	ldrh	w5, [x5, 16]
	cmp	w5, w4
	beq	.L255
	strb	w0, [x19, 6]
	ldrh	w0, [x19, 2]
	cmp	w0, w2
	bne	.L246
	cbz	w1, .L246
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2810
	add	x1, x1, 136
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L246:
	mov	w0, w20
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L252:
	ldrb	w0, [x19, 6]
	add	w0, w0, 1
	and	w0, w0, 255
	strb	w0, [x19, 6]
	cmp	w0, w3
	bne	.L251
	ldrh	w0, [x19, 2]
	strb	wzr, [x19, 6]
	add	w0, w0, 1
	strh	w0, [x19, 2]
.L251:
	ldrb	w0, [x19, 6]
	add	x0, x0, 8
	ldrh	w0, [x19, x0, lsl 1]
	b	.L250
.L256:
	mov	w20, 65535
	b	.L246
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.global	FtlGcBufInit
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	adrp	x0, .LANCHOR0
	add	x1, x0, :lo12:.LANCHOR0
	mov	w3, 0
	mov	w8, 24
	mov	w9, 1
	mov	w5, 4
	str	wzr, [x1, 3240]
.L263:
	ldrh	w2, [x1, 236]
	cmp	w3, w2
	bcc	.L264
	add	x0, x0, :lo12:.LANCHOR0
	mov	w6, 24
	mov	w5, 4
.L265:
	ldr	w1, [x0, 3280]
	cmp	w2, w1
	bcc	.L266
	ret
.L264:
	umull	x4, w3, w8
	ldr	x6, [x1, 3248]
	ldr	x7, [x1, 3256]
	add	x2, x6, x4
	str	w9, [x2, 16]
	ldrh	w2, [x1, 314]
	mul	w2, w2, w3
	sdiv	w2, w2, w5
	add	x2, x7, x2, sxtw 2
	str	x2, [x6, x4]
	ldrh	w2, [x1, 316]
	ldr	x7, [x1, 3248]
	ldr	x10, [x1, 3264]
	add	x6, x7, x4
	mul	w2, w2, w3
	sdiv	w2, w2, w5
	add	x2, x10, x2, sxtw 2
	str	x2, [x6, 8]
	ldr	x2, [x1, 3272]
	ubfiz	x10, x3, 5, 16
	ldr	x4, [x7, x4]
	add	w3, w3, 1
	add	x2, x2, x10
	and	w3, w3, 65535
	str	x4, [x2, 8]
	ldr	x4, [x6, 8]
	str	x4, [x2, 16]
	b	.L263
.L266:
	umull	x4, w2, w6
	ldr	x3, [x0, 3248]
	ldr	x7, [x0, 3256]
	add	x1, x3, x4
	str	wzr, [x1, 16]
	ldrh	w1, [x0, 314]
	mul	w1, w1, w2
	sdiv	w1, w1, w5
	add	x1, x7, x1, sxtw 2
	str	x1, [x3, x4]
	ldrh	w1, [x0, 316]
	ldr	x3, [x0, 3248]
	add	x3, x3, x4
	ldr	x4, [x0, 3264]
	mul	w1, w1, w2
	add	w2, w2, 1
	and	w2, w2, 65535
	sdiv	w1, w1, w5
	add	x1, x4, x1, sxtw 2
	str	x1, [x3, 8]
	b	.L265
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	adrp	x2, .LANCHOR0
	add	x2, x2, :lo12:.LANCHOR0
	mov	w3, 0
	mov	w8, 24
	ldr	w7, [x2, 3280]
	ldr	x5, [x2, 3248]
.L268:
	cmp	w3, w1
	bcs	.L267
	ubfiz	x4, x3, 5, 16
	mov	w2, 0
	add	x4, x0, x4
	b	.L273
.L269:
	add	w2, w2, 1
	and	w2, w2, 65535
.L273:
	cmp	w2, w7
	bcs	.L270
	umull	x6, w2, w8
	add	x9, x5, x6
	ldr	x10, [x5, x6]
	ldr	x6, [x4, 8]
	cmp	x10, x6
	bne	.L269
	str	wzr, [x9, 16]
.L270:
	add	w3, w3, 1
	and	w3, w3, 65535
	b	.L268
.L267:
	ret
	.size	FtlGcBufFree, .-FtlGcBufFree
	.align	2
	.global	FtlGcBufAlloc
	.type	FtlGcBufAlloc, %function
FtlGcBufAlloc:
	adrp	x2, .LANCHOR0
	add	x2, x2, :lo12:.LANCHOR0
	mov	w3, 0
	mov	w7, 24
	mov	w8, 1
	ldr	w5, [x2, 3280]
	ldr	x6, [x2, 3248]
.L275:
	cmp	w3, w1
	bcs	.L274
	mov	w2, 0
	b	.L280
.L276:
	add	w2, w2, 1
	and	w2, w2, 65535
.L280:
	cmp	w2, w5
	bcs	.L277
	umaddl	x4, w2, w7, x6
	ldr	w9, [x4, 16]
	cbnz	w9, .L276
	ubfiz	x2, x3, 5, 16
	ldr	x9, [x4]
	add	x2, x0, x2
	str	w8, [x4, 16]
	str	x9, [x2, 8]
	ldr	x4, [x4, 8]
	str	x4, [x2, 16]
.L277:
	add	w3, w3, 1
	and	w3, w3, 65535
	b	.L275
.L274:
	ret
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	2
	.global	IsBlkInGcList
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	and	w0, w0, 65535
	ldrh	w2, [x1, 3284]
	ldr	x3, [x1, 3288]
	mov	x1, 0
.L282:
	cmp	w2, w1, uxth
	bhi	.L284
	mov	w0, 0
	ret
.L284:
	add	x1, x1, 1
	add	x4, x3, x1, lsl 1
	ldrh	w4, [x4, -2]
	cmp	w4, w0
	bne	.L282
	mov	w0, 1
	ret
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	2
	.global	FtlGcUpdatePage
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	mov	w5, w0
	mov	w8, w1
	mov	w7, w2
	stp	x29, x30, [sp, -16]!
	lsr	w0, w0, 10
	add	x29, sp, 0
	bl	P2V_block_in_plane
	and	w9, w0, 65535
	adrp	x3, .LANCHOR0
	add	x4, x3, :lo12:.LANCHOR0
	ldrh	w1, [x4, 3284]
	ldr	x6, [x4, 3288]
	mov	x4, 0
.L287:
	and	w2, w4, 65535
	cmp	w2, w1
	bcc	.L289
	bne	.L288
	and	x4, x4, 65535
	strh	w0, [x6, x4, lsl 1]
	add	x4, x3, :lo12:.LANCHOR0
	ldrh	w0, [x4, 3284]
	add	w0, w0, 1
	strh	w0, [x4, 3284]
	b	.L288
.L289:
	add	x4, x4, 1
	add	x2, x6, x4, lsl 1
	ldrh	w2, [x2, -2]
	cmp	w2, w9
	bne	.L287
.L288:
	add	x0, x3, :lo12:.LANCHOR0
	mov	w1, 12
	ldrh	w3, [x0, 3296]
	ldr	x4, [x0, 3304]
	umull	x3, w3, w1
	add	x4, x4, x3
	str	w8, [x4, 4]
	ldr	x1, [x0, 3304]
	add	x4, x1, x3
	str	w7, [x4, 8]
	str	w5, [x1, x3]
	ldrh	w1, [x0, 3296]
	add	w1, w1, 1
	strh	w1, [x0, 3296]
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	and	w19, w0, 65535
	mov	w1, w19
	adrp	x0, .LC79
	add	x0, x0, :lo12:.LC79
	bl	sftl_printk
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w3, [x0, 226]
	cmp	w19, w3
	beq	.L292
	ldrh	w2, [x0, 224]
	cmp	w19, w2
	beq	.L292
	mov	w1, 65535
	cmp	w3, w1
	bne	.L293
	strh	w19, [x0, 226]
.L292:
	mov	w0, 0
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L293:
	cmp	w2, w1
	bne	.L292
	strh	w19, [x0, 224]
	b	.L292
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.align	2
	.global	FtlGcMarkBadPhyBlk
	.type	FtlGcMarkBadPhyBlk, %function
FtlGcMarkBadPhyBlk:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x20, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	and	w21, w0, 65535
	add	x20, x20, 3320
	mov	w0, w21
	bl	P2V_block_in_plane
	ldrh	w1, [x20, -8]
	mov	w2, w21
	mov	w22, w0
	adrp	x0, .LC80
	add	x0, x0, :lo12:.LC80
	bl	sftl_printk
	mov	w0, w22
	bl	FtlGcRefreshBlock
	ldrh	w0, [x20, -8]
	mov	x1, 0
.L296:
	cmp	w0, w1, uxth
	bhi	.L298
	cmp	w0, 15
	bhi	.L297
	add	x19, x19, :lo12:.LANCHOR0
	add	w1, w0, 1
	add	x19, x19, 3320
	strh	w1, [x19, -8]
	strh	w21, [x19, w0, sxtw 1]
	b	.L297
.L298:
	add	x1, x1, 1
	add	x2, x20, x1, lsl 1
	ldrh	w2, [x2, -2]
	cmp	w2, w21
	bne	.L296
.L297:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	2
	.global	FtlGcReFreshBadBlk
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 3312]
	cbz	w1, .L301
	ldrh	w3, [x0, 226]
	mov	w2, 65535
	cmp	w3, w2
	bne	.L301
	ldrh	w2, [x0, 3354]
	cmp	w2, w1
	bcc	.L302
	strh	wzr, [x0, 3354]
.L302:
	add	x19, x19, :lo12:.LANCHOR0
	add	x0, x19, 3320
	ldrh	w1, [x19, 3354]
	ldrh	w0, [x0, x1, lsl 1]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	w0, [x19, 3354]
	add	w0, w0, 1
	strh	w0, [x19, 3354]
.L301:
	mov	w0, 0
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	2
	.global	ftl_malloc
	.type	ftl_malloc, %function
ftl_malloc:
	stp	x29, x30, [sp, -16]!
	mov	w1, 193
	movk	w1, 0x240, lsl 16
	sxtw	x0, w0
	add	x29, sp, 0
	bl	__kmalloc
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_malloc, .-ftl_malloc
	.align	2
	.global	ftl_free
	.type	ftl_free, %function
ftl_free:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	kfree
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_free, .-ftl_free
	.align	2
	.global	rknand_print_hex
	.type	rknand_print_hex, %function
rknand_print_hex:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	mov	x21, x1
	stp	x23, x24, [sp, 48]
	adrp	x24, .LC81
	stp	x25, x26, [sp, 64]
	mov	w23, w2
	stp	x27, x28, [sp, 80]
	adrp	x25, .LC84
	mov	x27, x0
	mov	w26, w3
	add	x24, x24, :lo12:.LC81
	add	x25, x25, :lo12:.LC84
	stp	x19, x20, [sp, 16]
	mov	w22, 0
	mov	x19, 0
	mov	w20, 0
	adrp	x28, .LC83
.L312:
	cmp	w26, w19
	bhi	.L318
	adrp	x0, .LC85
	add	x0, x0, :lo12:.LC85
	bl	sftl_printk
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 96
	ret
.L318:
	cbnz	w20, .L313
	mov	w3, w22
	mov	x2, x21
	mov	x1, x27
	mov	x0, x24
	bl	sftl_printk
.L313:
	cmp	w23, 4
	bne	.L314
	ldr	w1, [x21, x19, lsl 2]
	adrp	x0, .LC82
	add	x0, x0, :lo12:.LC82
.L320:
	bl	sftl_printk
	add	w20, w20, 1
	cmp	w20, 15
	bls	.L317
	mov	w20, 0
	adrp	x0, .LC85
	add	x0, x0, :lo12:.LC85
	bl	sftl_printk
.L317:
	add	x19, x19, 1
	add	w22, w22, w23
	b	.L312
.L314:
	cmp	w23, 2
	bne	.L316
	ldrh	w1, [x21, x19, lsl 1]
	add	x0, x28, :lo12:.LC83
	b	.L320
.L316:
	ldrb	w1, [x21, x19]
	mov	x0, x25
	b	.L320
	.size	rknand_print_hex, .-rknand_print_hex
	.align	2
	.global	FlashEraseBlocks
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x25, x26, [sp, 64]
	ubfiz	x22, x2, 5, 32
	stp	x23, x24, [sp, 48]
	add	x23, x22, 4
	mov	x20, x0
	add	x21, x0, 4
	ldrh	w24, [x19, 12]
	add	x23, x0, x23
	mov	w25, -1
	lsl	w26, w24, 3
.L322:
	cmp	x21, x23
	beq	.L326
	ldr	w0, [x21]
	add	x2, x29, 88
	add	x1, x29, 92
	bl	l2p_addr_tran.isra.0
	ldr	w0, [x29, 88]
	cbnz	w0, .L323
	ldr	w1, [x29, 92]
	cmp	w26, w1
	bls	.L323
	mov	x19, x20
	add	x22, x20, x22
	adrp	x20, .LANCHOR1
	add	x20, x20, :lo12:.LANCHOR1
	adrp	x21, .LC86
	add	x20, x20, 160
	add	x21, x21, :lo12:.LC86
.L324:
	cmp	x19, x22
	bne	.L325
	bl	dump_stack
.L326:
	ldp	x19, x20, [sp, 16]
	mov	w0, 0
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 96
	ret
.L325:
	ldr	w2, [x29, 92]
	mov	w0, -1
	str	w0, [x19]
	mov	x1, x20
	mov	x0, x21
	add	x19, x19, 32
	bl	sftl_printk
	ldr	x1, [x19, -24]
	mov	w3, 16
	mov	w2, 4
	adrp	x0, .LC87
	add	x0, x0, :lo12:.LC87
	bl	rknand_print_hex
	ldr	x1, [x19, -16]
	mov	w3, 4
	adrp	x0, .LC88
	mov	w2, w3
	add	x0, x0, :lo12:.LC88
	bl	rknand_print_hex
	b	.L324
.L323:
	ldr	w1, [x29, 92]
	ldr	x2, [x19, 3368]
	blr	x2
	cmp	w0, 0
	csel	w0, w25, wzr, ne
	str	w0, [x21, -4]
	ldrh	w0, [x19, 14]
	cmp	w0, 4
	bne	.L330
	ldrb	w0, [x29, 88]
	ldr	x2, [x19, 3368]
	ldr	w1, [x29, 92]
	add	w1, w24, w1
	blr	x2
	cbz	w0, .L330
	str	w25, [x21, -4]
.L330:
	add	x21, x21, 32
	b	.L322
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	and	w21, w0, 65535
	stp	x19, x20, [sp, 16]
	sub	w2, w21, #1
	mov	w0, 65533
	cmp	w0, w2, uxth
	bcc	.L337
	adrp	x0, .LANCHOR0
	add	x20, x0, :lo12:.LANCHOR0
	mov	x19, x0
	ldrh	w2, [x20, 462]
	cmp	w2, 1024
	beq	.L337
	and	w1, w1, 65535
	cbz	w1, .L339
	mov	w0, w21
	bl	P2V_block_in_plane
	and	w22, w0, 65535
	ldr	x0, [x20, 3408]
	lsl	w1, w21, 10
	mov	w2, 1
	str	w1, [x0, 4]
	mov	w1, w2
	ldr	x0, [x20, 3408]
	bl	FlashEraseBlocks
	ldr	x2, [x20, 2600]
	ubfiz	x0, x22, 1, 16
	ldrh	w1, [x2, x0]
	add	w1, w1, 1
	strh	w1, [x2, x0]
	ldr	w0, [x20, 2680]
	add	w0, w0, 1
	str	w0, [x20, 2680]
.L339:
	add	x0, x19, :lo12:.LANCHOR0
	add	x0, x0, 456
	ldrh	w1, [x0, 6]
	add	w1, w1, 1
	strh	w1, [x0, 6]
	ldrh	w1, [x0, 4]
	add	x2, x0, x1, sxtw 1
	add	w1, w1, 1
	and	w1, w1, 1023
	strh	w1, [x0, 4]
	strh	w21, [x2, 8]
.L337:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	stp	x29, x30, [sp, -48]!
	adrp	x0, .LANCHOR0
	add	x29, sp, 0
	str	x21, [sp, 32]
	add	x21, x0, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	ldrh	w2, [x21, 462]
	cbz	w2, .L351
	ldrh	w0, [x21, 458]
	add	x1, x21, 456
	sub	w2, w2, #1
	strh	w2, [x21, 462]
	mov	w2, 1
	add	x3, x1, x0, sxtw 1
	add	w0, w0, 1
	and	w0, w0, 1023
	strh	w0, [x21, 458]
	ldr	x0, [x21, 3408]
	ldrh	w20, [x3, 8]
	lsl	w1, w20, 10
	str	w1, [x0, 4]
	mov	w1, w2
	ldr	x0, [x21, 3408]
	bl	FlashEraseBlocks
	ldr	w0, [x21, 2680]
	add	w0, w0, 1
	str	w0, [x21, 2680]
.L348:
	sub	w0, w20, #1
	mov	w1, 65533
	cmp	w1, w0, uxth
	bcs	.L349
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, w20
	ldrh	w2, [x0, 462]
	adrp	x0, .LC89
	add	x0, x0, :lo12:.LC89
	bl	sftl_printk
.L350:
	b	.L350
.L351:
	mov	w20, 65535
	b	.L348
.L349:
	mov	w0, w20
	ldr	x21, [sp, 32]
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	ftl_map_blk_alloc_new_blk
	.type	ftl_map_blk_alloc_new_blk, %function
ftl_map_blk_alloc_new_blk:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	str	x21, [sp, 32]
	mov	w20, 0
	ldrh	w1, [x0, 10]
	ldr	x0, [x0, 16]
.L354:
	cmp	w20, w1
	beq	.L358
	mov	x21, x0
	ldrh	w2, [x0], 2
	cbnz	w2, .L355
	bl	FtlFreeSysBlkQueueOut
	and	w1, w0, 65535
	strh	w0, [x21]
	sub	w2, w1, #1
	mov	w0, 65533
	cmp	w0, w2, uxth
	bcs	.L356
	adrp	x0, .LANCHOR0+462
	ldrh	w2, [x0, #:lo12:.LANCHOR0+462]
	adrp	x0, .LC90
	add	x0, x0, :lo12:.LC90
	bl	sftl_printk
.L357:
	b	.L357
.L356:
	ldr	w0, [x19, 48]
	strh	wzr, [x19, 2]
	add	w0, w0, 1
	str	w0, [x19, 48]
	ldrh	w0, [x19, 8]
	strh	w20, [x19]
	add	w0, w0, 1
	strh	w0, [x19, 8]
.L358:
	ldrh	w0, [x19, 10]
	cmp	w0, w20
	bhi	.L360
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 578
	add	x1, x1, 184
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L360:
	mov	w0, 0
	ldr	x21, [sp, 32]
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
.L355:
	add	w20, w20, 1
	and	w20, w20, 65535
	b	.L354
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	ftl_memset
	.type	ftl_memset, %function
ftl_memset:
	stp	x29, x30, [sp, -16]!
	uxtw	x2, w2
	add	x29, sp, 0
	bl	memset
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_memset, .-ftl_memset
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	stp	x29, x30, [sp, -48]!
	mov	w0, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	add	x19, x20, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	add	x1, x19, 2560
	add	x2, x19, 2560
	add	x3, x19, 3328
	strh	wzr, [x19, 3232]
	strh	wzr, [x19, 3312]
	stp	wzr, wzr, [x1, 68]
	stp	wzr, wzr, [x1, 76]
	str	wzr, [x19, 2644]
	str	wzr, [x19, 2648]
	stp	wzr, wzr, [x2, 100]
	stp	wzr, wzr, [x2, 108]
	str	wzr, [x19, 2680]
	str	wzr, [x19, 2684]
	str	wzr, [x19, 2688]
	str	wzr, [x19, 3228]
	str	wzr, [x19, 3236]
	stp	wzr, w0, [x3, 88]
	mov	w0, -1
	stp	wzr, wzr, [x3, 96]
	strh	w0, [x19, 226]
	strh	w0, [x19, 224]
	mov	w0, 32
	strh	w0, [x19, 3224]
	mov	w0, 128
	strh	w0, [x19, 3226]
	strh	wzr, [x19, 3354]
	ldrh	w0, [x19, 310]
	lsl	w0, w0, 1
	bl	ftl_malloc
	ldrh	w1, [x19, 310]
	str	x0, [x19, 3288]
	mov	w0, 12
	mul	w0, w1, w0
	bl	ftl_malloc
	ldrh	w21, [x19, 236]
	str	x0, [x19, 3304]
	lsl	w22, w21, 5
	lsl	w21, w21, 7
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3432]
	mov	w0, w22
	bl	ftl_malloc
	str	x0, [x19, 3440]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3448]
	mov	w0, w22
	bl	ftl_malloc
	str	x0, [x19, 3408]
	mov	w0, w22
	bl	ftl_malloc
	str	x0, [x19, 3272]
	ldrh	w0, [x19, 236]
	ldrh	w21, [x19, 314]
	lsl	w0, w0, 1
	add	w0, w0, 1
	str	w0, [x19, 3280]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3456]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3464]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3472]
	ldr	w0, [x19, 3280]
	mul	w0, w21, w0
	bl	ftl_malloc
	str	x0, [x19, 3256]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3480]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3488]
	ldr	w1, [x19, 3280]
	mov	w0, 24
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19, 3248]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3496]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3504]
	ldrh	w0, [x19, 262]
	lsl	w0, w0, 2
	bl	ftl_malloc
	ldrh	w21, [x19, 316]
	str	x0, [x19, 3512]
	ldrh	w0, [x19, 236]
	mul	w21, w21, w0
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3520]
	lsl	w0, w21, 2
	bl	ftl_malloc
	str	x0, [x19, 3528]
	ldrh	w1, [x19, 316]
	ldr	w0, [x19, 3280]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19, 3264]
	ldrh	w0, [x19, 246]
	ubfiz	w0, w0, 1, 15
	strh	w0, [x19, 3536]
	and	w0, w0, 65534
	bl	ftl_malloc
	str	x0, [x19, 3544]
	ldrh	w0, [x19, 3536]
	add	x0, x0, 547
	lsr	x0, x0, 9
	strh	w0, [x19, 3536]
	lsl	w0, w0, 9
	bl	ftl_malloc
	ldrh	w21, [x19, 246]
	str	x0, [x19, 3552]
	add	x0, x0, 32
	str	x0, [x19, 2600]
	lsl	w21, w21, 1
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3560]
	mov	w0, w21
	bl	ftl_malloc
	ldr	w21, [x19, 332]
	str	x0, [x19, 72]
	lsl	w21, w21, 1
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3568]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19, 3576]
	ldrh	w0, [x19, 324]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x19, 352]
	ldrh	w0, [x19, 324]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x19, 3584]
	ldrh	w0, [x19, 324]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19, 3592]
	ldrh	w0, [x19, 326]
	lsl	w0, w0, 2
	bl	ftl_malloc
	ldrh	w2, [x19, 326]
	mov	w1, 0
	str	x0, [x19, 3600]
	lsl	w2, w2, 2
	bl	ftl_memset
	ldrh	w0, [x19, 340]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19, 3608]
	ldr	w0, [x19, 332]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19, 3616]
	ldrh	w0, [x19, 342]
	lsl	w0, w0, 4
	bl	ftl_malloc
	ldrh	w1, [x19, 342]
	str	x0, [x19, 2608]
	ldrh	w0, [x19, 314]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19, 3624]
	ldrh	w1, [x19, 246]
	mov	w0, 6
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19, 2560]
	ldrh	w0, [x19, 302]
	ldrh	w1, [x19, 258]
	add	w0, w0, 31
	asr	w0, w0, 5
	strh	w0, [x19, 3632]
	mul	w0, w1, w0
	lsl	w0, w0, 2
	bl	ftl_malloc
	ldrh	w4, [x19, 3632]
	add	x5, x19, 400
	ldrh	w6, [x19, 258]
	mov	x1, 1
	str	x0, [x19, 392]
	mov	w2, w4
	mov	x0, x20
.L365:
	cmp	w1, w6
	bcc	.L366
	mov	w2, 8
	add	x4, x0, :lo12:.LANCHOR0
	sub	w2, w2, w1
	add	x4, x4, 360
	add	x2, x2, 1
	mov	x3, 0
.L367:
	add	x3, x3, 1
	cmp	x3, x2
	bne	.L368
	add	x1, x0, :lo12:.LANCHOR0
	ldr	x2, [x1, 3568]
	cbnz	x2, .L369
.L371:
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x1, x1, 216
	adrp	x0, .LC91
	add	x0, x0, :lo12:.LC91
	bl	sftl_printk
	mov	w0, -1
.L364:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
.L366:
	ldr	x3, [x19, 392]
	add	w1, w1, 1
	add	x3, x3, x2, uxtw 2
	add	w2, w2, w4
	str	x3, [x5], 8
	b	.L365
.L368:
	add	x5, x1, x3
	add	x5, x4, x5, lsl 3
	str	xzr, [x5, 24]
	b	.L367
.L369:
	ldr	x2, [x1, 3576]
	cbz	x2, .L371
	ldr	x2, [x1, 3608]
	cbz	x2, .L371
	ldr	x2, [x1, 3616]
	cbz	x2, .L371
	ldr	x2, [x1, 2608]
	cbz	x2, .L371
	ldr	x2, [x1, 3624]
	cbz	x2, .L371
	ldr	x2, [x1, 2560]
	cbz	x2, .L371
	ldr	x2, [x1, 392]
	cbz	x2, .L371
	ldr	x1, [x1, 72]
	cbz	x1, .L371
	add	x1, x0, :lo12:.LANCHOR0
	ldr	x2, [x1, 3288]
	cbz	x2, .L371
	ldr	x2, [x1, 3304]
	cbz	x2, .L371
	ldr	x2, [x1, 3432]
	cbz	x2, .L371
	ldr	x2, [x1, 3448]
	cbz	x2, .L371
	ldr	x2, [x1, 3408]
	cbz	x2, .L371
	ldr	x2, [x1, 3272]
	cbz	x2, .L371
	ldr	x2, [x1, 3440]
	cbz	x2, .L371
	ldr	x2, [x1, 3456]
	cbz	x2, .L371
	ldr	x2, [x1, 3464]
	cbz	x2, .L371
	ldr	x1, [x1, 3472]
	cbz	x1, .L371
	add	x0, x0, :lo12:.LANCHOR0
	ldr	x1, [x0, 3256]
	cbz	x1, .L371
	ldr	x1, [x0, 3480]
	cbz	x1, .L371
	ldr	x1, [x0, 3488]
	cbz	x1, .L371
	ldr	x1, [x0, 3248]
	cbz	x1, .L371
	ldr	x1, [x0, 3520]
	cbz	x1, .L371
	ldr	x1, [x0, 3528]
	cbz	x1, .L371
	ldr	x1, [x0, 3264]
	cbz	x1, .L371
	ldr	x1, [x0, 2600]
	cbz	x1, .L371
	ldr	x1, [x0, 3544]
	cbz	x1, .L371
	ldr	x0, [x0, 352]
	cbz	x0, .L371
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldr	x1, [x0, 3584]
	cbz	x1, .L371
	ldr	x1, [x0, 3592]
	cbz	x1, .L371
	ldr	x0, [x0, 3600]
	cbz	x0, .L371
	mov	w0, 0
	b	.L364
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	adrp	x21, .LANCHOR0
	stp	x23, x24, [sp, 48]
	mov	x23, x0
	add	x0, x21, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR1
	add	x20, x20, :lo12:.LANCHOR1
	adrp	x24, .LC8
	add	x20, x20, 232
	ldrh	w2, [x0, 3632]
	add	x24, x24, :lo12:.LC8
	mov	x19, 0
	mov	x22, x1
	mov	x0, x22
	mov	w1, 0
	lsl	w2, w2, 2
	bl	ftl_memset
.L469:
	ldrh	w0, [x23, x19]
	mov	w1, 65535
	cmp	w0, w1
	beq	.L466
	add	x1, x21, :lo12:.LANCHOR0
	ldrh	w1, [x1, 302]
	cmp	w1, w0
	bhi	.L468
	mov	w2, 74
	mov	x1, x20
	mov	x0, x24
	bl	sftl_printk
.L468:
	ldrh	w2, [x23, x19]
	mov	w1, 1
	add	x19, x19, 2
	cmp	x19, 1024
	ubfx	x0, x2, 5, 11
	lsl	w2, w1, w2
	lsl	x0, x0, 2
	ldr	w1, [x22, x0]
	orr	w1, w1, w2
	str	w1, [x22, x0]
	bne	.L469
.L466:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	2
	.global	FtlBbtMemInit
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	stp	x29, x30, [sp, -16]!
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	mov	w1, -1
	add	x29, sp, 0
	mov	w2, 16
	add	x0, x0, 372
	strh	w1, [x0, -12]
	mov	w1, 255
	strh	wzr, [x0, -6]
	bl	ftl_memset
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	stp	x29, x30, [sp, -16]!
	adrp	x3, .LANCHOR0
	add	x3, x3, :lo12:.LANCHOR0
	mov	w2, 2048
	add	x29, sp, 0
	mov	w1, 0
	strh	w0, [x3, 456]
	add	x0, x3, 464
	strh	wzr, [x3, 458]
	strh	wzr, [x3, 460]
	strh	wzr, [x3, 462]
	bl	ftl_memset
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	stp	x29, x30, [sp, -80]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	str	x25, [sp, 64]
	ldrh	w2, [x0, 10]
	ldp	x21, x20, [x0, 32]
	ldr	x22, [x0, 16]
	lsl	w2, w2, 1
	mov	x0, x21
	bl	ftl_memset
	mov	w0, 0
.L478:
	ldrh	w1, [x19, 6]
	cmp	w1, w0
	bhi	.L482
	adrp	x0, .LANCHOR0+308
	mov	w23, 0
	mov	w20, 0
	ldrh	w1, [x0, #:lo12:.LANCHOR0+308]
	ldrh	w0, [x19]
	strh	w1, [x21, x0, lsl 1]
	ldrh	w24, [x21]
.L483:
	ldrh	w0, [x19, 10]
	cmp	w0, w20
	bhi	.L487
	mov	w0, w23
	ldr	x25, [sp, 64]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 80
	ret
.L482:
	ubfiz	x1, x0, 2, 16
	ldr	w2, [x20, x1]
	mov	w1, 0
	ubfx	x2, x2, 10, 16
.L479:
	ldrh	w3, [x19, 10]
	cmp	w3, w1
	bhi	.L481
	add	w0, w0, 1
	and	w0, w0, 65535
	b	.L478
.L481:
	ubfiz	x3, x1, 1, 16
	ldrh	w4, [x22, x3]
	cmp	w4, w2
	bne	.L480
	cbz	w2, .L480
	ldrh	w4, [x21, x3]
	add	w4, w4, 1
	strh	w4, [x21, x3]
.L480:
	add	w1, w1, 1
	and	w1, w1, 65535
	b	.L479
.L487:
	ubfiz	x0, x20, 1, 16
	ldrh	w1, [x21, x0]
	cmp	w24, w1
	bls	.L484
	add	x25, x22, x0
	ldrh	w0, [x22, x0]
	cbnz	w0, .L485
.L486:
	add	w20, w20, 1
	and	w20, w20, 65535
	b	.L483
.L484:
	cbnz	w1, .L486
	add	x25, x22, x0
	ldrh	w0, [x22, x0]
	cbz	w0, .L486
.L488:
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
	strh	wzr, [x25]
	ldrh	w0, [x19, 8]
	sub	w0, w0, #1
	strh	w0, [x19, 8]
	b	.L486
.L489:
	mov	w24, 0
	b	.L488
.L485:
	mov	w23, w20
	cbz	w1, .L489
	mov	w24, w1
	b	.L486
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	FtlL2PDataInit
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	stp	x29, x30, [sp, -32]!
	adrp	x0, .LANCHOR0
	mov	w1, 0
	add	x29, sp, 0
	str	x19, [sp, 16]
	add	x19, x0, :lo12:.LANCHOR0
	ldr	x0, [x19, 3576]
	ldr	w2, [x19, 332]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w0, [x19, 342]
	mov	w1, 255
	ldrh	w2, [x19, 314]
	mul	w2, w2, w0
	ldr	x0, [x19, 3624]
	bl	ftl_memset
	mov	x0, x19
	mov	w1, 0
	mov	w5, -1
.L499:
	ldrh	w2, [x0, 342]
	cmp	w2, w1
	bhi	.L500
	mov	w2, -1
	strh	w2, [x0, 3642]
	strh	w2, [x0, 3640]
	add	x1, x0, 3640
	ldr	w2, [x0, 332]
	strh	w2, [x0, 3650]
	mov	w2, -3902
	strh	w2, [x0, 3644]
	ldrh	w2, [x0, 3704]
	strh	w2, [x0, 3648]
	ldrh	w2, [x0, 340]
	strh	w2, [x0, 3646]
	ldr	x2, [x0, 3568]
	str	x2, [x0, 3656]
	ldr	x2, [x0, 3616]
	str	x2, [x0, 3664]
	ldr	x2, [x0, 3576]
	str	x2, [x0, 3672]
	ldr	x0, [x0, 3608]
	str	x0, [x1, 40]
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L500:
	ldr	x3, [x0, 2608]
	ubfiz	x2, x1, 4, 16
	add	x3, x3, x2
	str	wzr, [x3, 4]
	ldr	x3, [x0, 2608]
	strh	w5, [x3, x2]
	ldr	x3, [x0, 2608]
	ldr	x4, [x0, 3624]
	add	x3, x3, x2
	ldrh	w2, [x0, 314]
	mul	w2, w2, w1
	add	w1, w1, 1
	and	w1, w1, 65535
	sxtw	x2, w2
	and	x2, x2, -4
	add	x2, x4, x2
	str	x2, [x3, 8]
	b	.L499
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	FtlVariablesInit
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	stp	x29, x30, [sp, -32]!
	mov	w0, -1
	mov	w1, 0
	add	x29, sp, 0
	str	x19, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	strh	w0, [x19, 3706]
	mov	w0, -1
	str	w0, [x19, 3716]
	ldr	x0, [x19, 352]
	ldrh	w2, [x19, 324]
	strh	wzr, [x19, 348]
	str	wzr, [x19, 3708]
	lsl	w2, w2, 1
	str	wzr, [x19, 3712]
	bl	ftl_memset
	ldr	x0, [x19, 2600]
	mov	w1, 0
	ldrh	w2, [x19, 246]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldr	x0, [x19, 3544]
	mov	w1, 0
	ldrh	w2, [x19, 246]
	lsl	w2, w2, 1
	bl	ftl_memset
	mov	w2, 48
	mov	w1, 0
	add	x0, x19, 2512
	bl	ftl_memset
	mov	w2, 512
	mov	w1, 0
	add	x0, x19, 2712
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	ldr	x19, [sp, 16]
	mov	w0, 0
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	stp	x29, x30, [sp, -64]!
	mov	w0, 6
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	str	x23, [sp, 48]
	add	x23, x19, 264
	stp	x21, x22, [sp, 32]
	mov	w20, 0
	mov	w22, 0
	ldrh	w2, [x19, 246]
	mov	w21, 0
	mul	w2, w2, w0
	ldr	x0, [x19, 2560]
	bl	ftl_memset
	strh	wzr, [x19, 2584]
	strh	wzr, [x19, 228]
	str	xzr, [x19, 2568]
	str	xzr, [x19, 2576]
	str	xzr, [x19, 2592]
.L505:
	ldrh	w0, [x19, 244]
	cmp	w20, w0
	bcs	.L512
	ldrh	w8, [x19, 236]
	mov	x6, 0
	ldrh	w7, [x19, 306]
	mov	w5, 0
	b	.L513
.L507:
	ldrb	w0, [x23, x6]
	mov	w1, w20
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L506
	add	w5, w5, w7
	and	w5, w5, 65535
.L506:
	add	x6, x6, 1
.L513:
	cmp	w8, w6, uxth
	bhi	.L507
	cbz	w5, .L508
	mov	w0, 32768
	sdiv	w5, w0, w5
.L509:
	ldr	x1, [x19, 2560]
	mov	w0, 6
	umaddl	x0, w20, w0, x1
	strh	w5, [x0, 4]
	ldrh	w0, [x19, 24]
	cmp	w0, w20
	beq	.L510
	ldrh	w0, [x19, 80]
	cmp	w0, w20
	beq	.L510
	ldrh	w0, [x19, 128]
	cmp	w0, w20
	beq	.L510
	ldr	x1, [x19, 72]
	ubfiz	x0, x20, 1, 16
	ldrh	w0, [x1, x0]
	cbnz	w0, .L511
	add	w22, w22, 1
	mov	w0, w20
	and	w22, w22, 65535
	bl	INSERT_FREE_LIST
.L510:
	add	w20, w20, 1
	and	w20, w20, 65535
	b	.L505
.L508:
	ldr	x1, [x19, 72]
	ubfiz	x0, x20, 1, 16
	mov	w2, -1
	strh	w2, [x1, x0]
	b	.L509
.L511:
	add	w21, w21, 1
	mov	w0, w20
	and	w21, w21, 65535
	bl	INSERT_DATA_LIST
	b	.L510
.L512:
	strh	w21, [x19, 2584]
	add	w21, w21, w22
	strh	w22, [x19, 228]
	cmp	w21, w0
	ble	.L514
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2212
	add	x1, x1, 248
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L514:
	mov	w0, 0
	ldr	x23, [sp, 48]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 64
	ret
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	stp	x29, x30, [sp, -32]!
	mov	w1, 255
	add	x29, sp, 0
	str	x19, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	ldr	x0, [x19, 3288]
	ldrh	w2, [x19, 310]
	strh	wzr, [x19, 3284]
	strh	wzr, [x19, 3296]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w2, [x19, 310]
	mov	w0, 12
	mov	w1, 255
	mul	w2, w2, w0
	ldr	x0, [x19, 3304]
	bl	ftl_memset
	bl	FtlGcBufInit
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	FlashGetBadBlockList
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	stp	x29, x30, [sp, -48]!
	mov	w2, 256
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x20, x0
	str	x21, [sp, 32]
	mov	w21, w1
	mov	w1, 255
	bl	ftl_memset
	adrp	x19, .LANCHOR0
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, w21
	ldr	x2, [x0, 3360]
	mov	x0, x20
	blr	x2
	and	w0, w0, 65535
	cmp	w0, 50
	bls	.L519
	mov	w2, 256
	mov	w1, 255
	mov	x0, x20
	bl	ftl_memset
	mov	w0, 0
.L519:
	add	x19, x19, :lo12:.LANCHOR0
	ldrh	w1, [x19, 14]
	cmp	w1, 4
	bne	.L523
	mov	x1, 0
.L521:
	cmp	w0, w1, uxth
	bhi	.L522
.L523:
	ldp	x19, x20, [sp, 16]
	ldr	x21, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
.L522:
	ldrh	w2, [x20, x1, lsl 1]
	lsr	w2, w2, 1
	strh	w2, [x20, x1, lsl 1]
	add	x1, x1, 1
	b	.L521
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	2
	.global	ftl_memcpy
	.type	ftl_memcpy, %function
ftl_memcpy:
	stp	x29, x30, [sp, -16]!
	uxtw	x2, w2
	add	x29, sp, 0
	bl	memcpy
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_memcpy, .-ftl_memcpy
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	stp	x29, x30, [sp, -112]!
	ubfiz	x1, x1, 5, 32
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	adrp	x21, .LANCHOR0
	add	x2, x21, :lo12:.LANCHOR0
	stp	x23, x24, [sp, 48]
	stp	x19, x20, [sp, 16]
	adrp	x22, .LANCHOR1
	stp	x25, x26, [sp, 64]
	add	x22, x22, :lo12:.LANCHOR1
	str	x27, [sp, 80]
	adrp	x24, .LC8
	mov	x19, x0
	add	x23, x0, x1
	ldrh	w25, [x2, 12]
	add	x22, x22, 272
	add	x24, x24, :lo12:.LC8
.L528:
	cmp	x23, x19
	bne	.L541
	ldp	x19, x20, [sp, 16]
	mov	w0, 0
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldr	x27, [sp, 80]
	ldp	x29, x30, [sp], 112
	ret
.L541:
	ldr	x0, [x19, 8]
	cbz	x0, .L529
	ldr	x0, [x19, 16]
	cbnz	x0, .L530
.L529:
	mov	w2, 96
	mov	x1, x22
	mov	x0, x24
	bl	sftl_printk
.L530:
	ldr	w0, [x19, 4]
	add	x2, x29, 104
	add	x1, x29, 108
	bl	l2p_addr_tran.isra.0
	ldr	w0, [x29, 104]
	cmp	w0, 3
	bls	.L531
	mov	w0, -1
	str	w0, [x19]
.L532:
	add	x19, x19, 32
	b	.L528
.L531:
	ldr	x20, [x19, 8]
	tst	x20, 63
	beq	.L533
	add	x1, x21, :lo12:.LANCHOR0
	ldr	x20, [x1, 3496]
.L533:
	add	x27, x21, :lo12:.LANCHOR0
	ldr	w1, [x29, 108]
	ldr	x3, [x19, 16]
	mov	x2, x20
	ldr	x4, [x27, 3384]
	blr	x4
	str	w0, [x19]
	ldrh	w0, [x27, 14]
	cmp	w0, 4
	bne	.L535
	ldrb	w0, [x29, 104]
	add	x2, x20, 2048
	ldr	x4, [x27, 3384]
	ldr	w1, [x29, 108]
	ldr	x3, [x19, 16]
	add	w1, w25, w1
	add	x3, x3, 8
	blr	x4
	cmn	w0, #1
	beq	.L536
	ldr	x1, [x19, 16]
	ldr	w2, [x1, 12]
	cmn	w2, #1
	bne	.L537
	ldr	w2, [x1, 8]
	cmn	w2, #1
	bne	.L537
	ldr	w1, [x1]
	cmn	w1, #1
	beq	.L537
.L536:
	mov	w1, -1
	str	w1, [x19]
.L537:
	ldr	w1, [x19]
	cmn	w1, #1
	beq	.L538
	cmp	w0, 256
	bne	.L538
	str	w0, [x19]
.L538:
	ldr	w3, [x19]
	cmp	w3, 256
	ccmn	w3, #1, 4, ne
	bne	.L535
	ldr	w1, [x19, 4]
	adrp	x0, .LC92
	ldr	w2, [x29, 108]
	add	x0, x0, :lo12:.LC92
	bl	sftl_printk
	ldr	x1, [x19, 8]
	cbz	x1, .L540
	mov	w3, 4
	adrp	x0, .LC93
	mov	w2, w3
	add	x0, x0, :lo12:.LC93
	bl	rknand_print_hex
.L540:
	ldr	x1, [x19, 16]
	cbz	x1, .L535
	mov	w3, 4
	adrp	x0, .LC94
	mov	w2, w3
	add	x0, x0, :lo12:.LC94
	bl	rknand_print_hex
.L535:
	add	x1, x21, :lo12:.LANCHOR0
	ldr	x0, [x1, 3496]
	cmp	x20, x0
	bne	.L532
	ldr	x0, [x19, 8]
	cmp	x20, x0
	beq	.L532
	ldrh	w2, [x1, 262]
	mov	x1, x20
	lsl	w2, w2, 9
	bl	ftl_memcpy
	b	.L532
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	stp	x29, x30, [sp, -80]!
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	add	x22, x0, 372
	stp	x25, x26, [sp, 64]
	mov	w21, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	stp	x23, x24, [sp, 48]
	add	x23, x0, 3720
	mov	x25, x23
	mov	w26, 61664
	ldr	x1, [x0, 3456]
	ldr	x24, [x0, 3520]
	stp	x1, x24, [x23, 8]
.L568:
	ldrh	w0, [x19, 258]
	cmp	w21, w0
	bcc	.L573
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 80
	ret
.L573:
	ldrh	w20, [x19, 302]
	mov	w0, -1
	strh	w0, [x22]
.L570:
	ldrh	w0, [x19, 302]
	sub	w20, w20, #1
	and	w20, w20, 65535
	sub	w1, w0, #16
	cmp	w20, w1
	ble	.L571
	madd	w0, w0, w21, w20
	mov	w2, 1
	mov	w1, w2
	lsl	w0, w0, 10
	str	w0, [x25, 4]
	mov	x0, x23
	bl	FlashReadPages
	ldr	w0, [x25]
	cmn	w0, #1
	beq	.L570
	ldrh	w0, [x24]
	cmp	w0, w26
	bne	.L570
	strh	w20, [x22]
.L571:
	add	w21, w21, 1
	add	x22, x22, 2
	b	.L568
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	stp	x29, x30, [sp, -160]!
	cmp	w1, 1
	add	x29, sp, 0
	stp	x23, x24, [sp, 48]
	mov	w23, w1
	stp	x19, x20, [sp, 16]
	adrp	x1, .LANCHOR0
	stp	x21, x22, [sp, 32]
	add	x2, x1, :lo12:.LANCHOR0
	bne	.L579
	ldrh	w19, [x2, 308]
.L580:
	add	x1, x1, :lo12:.LANCHOR0
	sub	w19, w19, #1
	sxth	w19, w19
	lsl	w21, w0, 10
	orr	w0, w19, w21
	mov	w2, w23
	ldr	x1, [x1, 3504]
	str	x1, [x29, 72]
	add	x1, x29, 96
	str	w0, [x29, 68]
	add	x0, x29, 64
	str	x1, [x29, 80]
	mov	w1, 1
	bl	FlashReadPages
	ldr	w0, [x29, 96]
	cmn	w0, #1
	bne	.L581
	mov	w22, 0
	mov	w24, 2
.L582:
	cmp	w22, w19
	ble	.L585
.L581:
	mov	w0, w19
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 160
	ret
.L579:
	ldrh	w19, [x2, 306]
	b	.L580
.L585:
	add	w20, w22, w19
	mov	w2, w23
	mov	w1, 1
	sdiv	w20, w20, w24
	sxth	w0, w20
	orr	w0, w0, w21
	str	w0, [x29, 68]
	add	x0, x29, 64
	bl	FlashReadPages
	ldr	w0, [x29, 96]
	cmn	w0, #1
	bne	.L583
	ldr	w0, [x29, 100]
	cmn	w0, #1
	bne	.L583
	ldr	w0, [x29, 64]
	cmn	w0, #1
	beq	.L583
	sub	w19, w20, #1
	sxth	w19, w19
	b	.L582
.L583:
	add	w20, w20, 1
	sxth	w22, w20
	b	.L582
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	stp	x29, x30, [sp, -112]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	stp	x21, x22, [sp, 32]
	adrp	x21, .LANCHOR0
	add	x19, x21, :lo12:.LANCHOR0
	stp	x23, x24, [sp, 48]
	stp	x25, x26, [sp, 64]
	adrp	x25, .LANCHOR1
	stp	x27, x28, [sp, 80]
	add	x25, x25, :lo12:.LANCHOR1
	ldr	x0, [x19, 3616]
	add	x25, x25, 288
	ldr	w2, [x19, 332]
	strh	wzr, [x19, 3704]
	strh	wzr, [x19, 348]
	lsl	w2, w2, 2
	bl	ftl_memset
	ldr	x0, [x19, 3568]
	mov	w1, 0
	ldr	w2, [x19, 332]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldr	x0, [x19, 3592]
	mov	w1, 0
	ldrh	w2, [x19, 324]
	lsl	w2, w2, 2
	bl	ftl_memset
	ldr	x0, [x19, 352]
	mov	w1, 0
	ldrh	w2, [x19, 324]
	lsl	w2, w2, 1
	bl	ftl_memset
	mov	w2, 16
	mov	w1, 255
	add	x0, x19, 2696
	bl	ftl_memset
	ldrh	w24, [x19, 244]
.L591:
	ldrh	w0, [x19, 246]
	cmp	w0, w24
	bls	.L631
	ldrh	w9, [x19, 236]
	mov	x5, 0
	ldrh	w8, [x19, 316]
	mov	w20, 0
	add	x7, x19, 264
	mov	w6, 4
	b	.L632
.L593:
	ldrb	w0, [x7, x5]
	mov	w1, w24
	bl	V2P_block
	and	w4, w0, 65535
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L592
	ldr	x0, [x19, 3432]
	ubfiz	x2, x20, 5, 16
	lsl	w4, w4, 10
	add	x0, x0, x2
	str	w4, [x0, 4]
	ldr	x1, [x19, 3432]
	ldr	x0, [x19, 3256]
	add	x1, x1, x2
	ldr	x2, [x19, 3264]
	str	x0, [x1, 8]
	mul	w0, w20, w8
	add	w20, w20, 1
	and	w20, w20, 65535
	sdiv	w0, w0, w6
	add	x0, x2, x0, sxtw 2
	str	x0, [x1, 16]
.L592:
	add	x5, x5, 1
.L632:
	cmp	w9, w5, uxth
	bhi	.L593
	cbnz	w20, .L594
.L630:
	add	w24, w24, 1
	and	w24, w24, 65535
	b	.L591
.L594:
	ldr	x0, [x19, 3432]
	mov	w1, w20
	mov	w2, 1
	ubfiz	x20, x20, 5, 16
	mov	x23, 0
	add	x27, x19, 2696
	bl	FlashReadPages
.L629:
	ldr	x0, [x19, 3432]
	add	x1, x0, x23
	ldr	w0, [x0, x23]
	ldr	w22, [x1, 4]
	cmn	w0, #1
	ldr	x26, [x1, 16]
	ubfx	x22, x22, 10, 16
	bne	.L597
	mov	w28, 16
	mov	w3, 65535
.L599:
	ldr	x0, [x19, 3432]
	mov	w2, 1
	str	w3, [x29, 108]
	add	x0, x0, x23
	ldr	w1, [x0, 4]
	add	w1, w1, 1
	str	w1, [x0, 4]
	mov	w1, w2
	ldr	x0, [x19, 3432]
	add	x0, x0, x23
	bl	FlashReadPages
	ldrh	w0, [x26]
	ldr	w3, [x29, 108]
	cmp	w0, w3
	ldr	x0, [x19, 3432]
	bne	.L596
	mov	w1, -1
	str	w1, [x0, x23]
	ldr	x0, [x19, 3432]
	ldr	w0, [x0, x23]
	cmp	w0, w1
	bne	.L597
.L598:
	mov	w1, 1
	b	.L669
.L596:
	ldr	w0, [x0, x23]
	cmn	w0, #1
	bne	.L597
	sub	w28, w28, #1
	ands	w28, w28, 65535
	bne	.L599
	b	.L598
.L597:
	ldr	w1, [x19, 2660]
	ldr	w0, [x26, 4]
	cmn	w1, #1
	beq	.L600
	cmp	w1, w0
	bhi	.L601
.L600:
	cmn	w0, #1
	beq	.L601
	add	w1, w0, 1
	str	w1, [x19, 2660]
.L601:
	ldrh	w1, [x26]
	mov	w2, 61604
	cmp	w1, w2
	beq	.L603
	bhi	.L604
	mov	w0, 61574
	cmp	w1, w0
	beq	.L605
.L602:
	add	x23, x23, 32
	cmp	x20, x23
	bne	.L629
	b	.L630
.L604:
	mov	w0, 61634
	cmp	w1, w0
	beq	.L606
	mov	w0, 65535
	cmp	w1, w0
	bne	.L602
	mov	w1, 0
.L669:
	mov	w0, w22
	bl	FtlFreeSysBlkQueueIn
	b	.L602
.L606:
	ldrh	w1, [x19, 3704]
	ldr	w0, [x19, 332]
	cmp	w1, w0
	bls	.L608
	adrp	x0, .LC8
	mov	w2, 1225
	mov	x1, x25
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L608:
	ldr	w3, [x19, 332]
	ldrh	w1, [x19, 3704]
	and	w2, w3, 65535
	ldr	x4, [x19, 3616]
	sub	w0, w2, #1
	sub	w2, w2, w1
	sub	w2, w2, #1
	sxth	x0, w0
	sxth	w2, w2
.L609:
	cmp	w0, w2
	bgt	.L615
	tbz	w0, #31, .L649
	b	.L602
.L615:
	sxtw	x5, w0
	ldr	w8, [x26, 4]
	lsl	x6, x5, 2
	ldr	w7, [x4, x6]
	cmp	w8, w7
	bls	.L610
	ldr	w2, [x4]
	cbnz	w2, .L611
	cmp	w3, w1
	beq	.L611
	add	w1, w1, 1
	strh	w1, [x19, 3704]
.L611:
	mov	w1, 0
.L612:
	cmp	w1, w0
	bne	.L613
	ldr	x1, [x19, 3616]
	ldr	w2, [x26, 4]
	str	w2, [x1, x6]
	ldr	x1, [x19, 3568]
	strh	w22, [x1, x5, lsl 1]
	tbnz	w0, #31, .L602
	ldrh	w1, [x19, 3704]
	ldr	w2, [x19, 332]
	sub	w2, w2, w1
	sub	w2, w2, #1
	cmp	w0, w2, sxth
	bgt	.L602
.L649:
	add	w1, w1, 1
	strh	w1, [x19, 3704]
	ldr	x1, [x19, 3616]
	ldr	w2, [x26, 4]
	str	w2, [x1, x0, lsl 2]
	ldr	x1, [x19, 3568]
.L667:
	strh	w22, [x1, x0, lsl 1]
	b	.L602
.L613:
	ldr	x4, [x19, 3616]
	sxtw	x2, w1
	lsl	x3, x2, 2
	lsl	x2, x2, 1
	add	x7, x4, x3
	add	w1, w1, 1
	sxth	w1, w1
	ldr	w7, [x7, 4]
	str	w7, [x4, x3]
	ldr	x3, [x19, 3568]
	add	x4, x3, x2
	ldrh	w4, [x4, 2]
	strh	w4, [x3, x2]
	b	.L612
.L610:
	sub	w0, w0, #1
	sxth	x0, w0
	b	.L609
.L605:
	ldrh	w1, [x19, 348]
	ldrh	w0, [x19, 324]
	cmp	w1, w0
	bls	.L618
	adrp	x0, .LC8
	mov	w2, 1266
	mov	x1, x25
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L618:
	ldrh	w3, [x19, 324]
	ldrh	w1, [x19, 348]
	sub	w2, w3, #1
	ldr	x4, [x19, 3592]
	sxth	x0, w2
	sub	w2, w2, w1
.L619:
	cmp	w0, w2
	ble	.L624
	sxtw	x5, w0
	ldr	w8, [x26, 4]
	lsl	x6, x5, 2
	ldr	w7, [x4, x6]
	cmp	w8, w7
	bls	.L620
	ldr	w2, [x4]
	cbnz	w2, .L621
	cmp	w3, w1
	beq	.L621
	add	w1, w1, 1
	strh	w1, [x19, 348]
.L621:
	mov	w1, 0
.L622:
	cmp	w1, w0
	bne	.L623
	ldr	x1, [x19, 3592]
	ldr	w2, [x26, 4]
	str	w2, [x1, x6]
	ldr	x1, [x19, 352]
	strh	w22, [x1, x5, lsl 1]
.L624:
	tbnz	w0, #31, .L602
	ldrh	w1, [x19, 324]
	ldrh	w2, [x19, 348]
	sub	w1, w1, #1
	sub	w1, w1, w2
	cmp	w0, w1, sxth
	bgt	.L602
	ldr	x1, [x19, 3592]
	add	w2, w2, 1
	strh	w2, [x19, 348]
	ldr	w2, [x26, 4]
	str	w2, [x1, x0, lsl 2]
	ldr	x1, [x19, 352]
	b	.L667
.L623:
	ldr	x4, [x19, 3592]
	sxtw	x2, w1
	lsl	x3, x2, 2
	lsl	x2, x2, 1
	add	x7, x4, x3
	add	w1, w1, 1
	sxth	w1, w1
	ldr	w7, [x7, 4]
	str	w7, [x4, x3]
	ldr	x3, [x19, 352]
	add	x4, x3, x2
	ldrh	w4, [x4, 2]
	strh	w4, [x3, x2]
	b	.L622
.L620:
	sub	w0, w0, #1
	sxth	x0, w0
	b	.L619
.L603:
	ldrh	w2, [x27]
	mov	w1, 65535
	cmp	w2, w1
	bne	.L626
	strh	w22, [x27]
.L668:
	str	w0, [x27, 8]
	b	.L602
.L626:
	ldrh	w0, [x27, 4]
	cmp	w0, w1
	beq	.L627
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
.L627:
	ldr	w0, [x26, 4]
	ldr	w1, [x27, 8]
	cmp	w1, w0
	bcs	.L628
	ldrh	w0, [x27]
	strh	w0, [x27, 4]
	strh	w22, [x27]
	ldr	w0, [x26, 4]
	b	.L668
.L628:
	strh	w22, [x27, 4]
	b	.L602
.L631:
	ldr	x1, [x19, 3568]
	ldrh	w0, [x1]
	cbz	w0, .L633
.L636:
	add	x0, x21, :lo12:.LANCHOR0
	ldr	x1, [x0, 352]
	ldrh	w2, [x1]
	cbz	w2, .L634
.L635:
	add	x21, x21, :lo12:.LANCHOR0
	ldrh	w1, [x21, 3704]
	ldr	w0, [x21, 332]
	cmp	w1, w0
	bls	.L665
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 1391
	add	x1, x1, 288
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L665:
	ldp	x19, x20, [sp, 16]
	mov	w0, 0
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 112
	ret
.L633:
	ldrh	w0, [x19, 3704]
	cbz	w0, .L636
	ldr	w2, [x19, 332]
	mov	w0, 0
.L637:
	cmp	w0, w2
	bcs	.L636
	ldrh	w3, [x1, w0, sxtw 1]
	cbz	w3, .L638
	mov	w1, w0
	add	x2, x21, :lo12:.LANCHOR0
.L639:
	ldr	w3, [x2, 332]
	cmp	w1, w3
	bcs	.L636
	ldr	x5, [x2, 3568]
	sxtw	x6, w1
	lsl	x4, x6, 1
	sub	w3, w1, w0
	sxtw	x3, w3
	add	w1, w1, 1
	sxth	w1, w1
	ldrh	w7, [x5, x4]
	strh	w7, [x5, x3, lsl 1]
	ldr	x5, [x2, 3616]
	ldr	w6, [x5, x6, lsl 2]
	str	w6, [x5, x3, lsl 2]
	ldr	x3, [x2, 3568]
	strh	wzr, [x3, x4]
	b	.L639
.L638:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L637
.L634:
	ldrh	w2, [x0, 348]
	cbz	w2, .L635
	ldrh	w2, [x0, 324]
	mov	w0, 0
.L644:
	mov	w6, w0
	cmp	w0, w2
	bge	.L635
	ldrh	w3, [x1, w0, sxtw 1]
	cbz	w3, .L645
	add	x1, x21, :lo12:.LANCHOR0
.L646:
	ldrh	w2, [x1, 324]
	cmp	w0, w2
	bge	.L635
	ldr	x4, [x1, 352]
	sxtw	x5, w0
	lsl	x3, x5, 1
	sub	w2, w0, w6
	sxtw	x2, w2
	add	w0, w0, 1
	sxth	w0, w0
	ldrh	w7, [x4, x3]
	strh	w7, [x4, x2, lsl 1]
	ldr	x4, [x1, 3592]
	ldr	w5, [x4, x5, lsl 2]
	str	w5, [x4, x2, lsl 2]
	ldr	x2, [x1, 352]
	strh	wzr, [x2, x3]
	b	.L646
.L645:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L644
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	FtlLoadBbt
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	stp	x23, x24, [sp, 48]
	add	x23, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	add	x21, x23, 3720
	mov	w24, 61649
	ldr	x0, [x23, 3456]
	ldr	x22, [x23, 3520]
	stp	x0, x22, [x21, 8]
	bl	FtlBbtMemInit
	ldrh	w20, [x23, 302]
	sub	w20, w20, #1
	and	w20, w20, 65535
.L671:
	ldrh	w0, [x23, 302]
	sub	w0, w0, #16
	cmp	w20, w0
	ble	.L674
	lsl	w0, w20, 10
	mov	w2, 1
	str	w0, [x21, 4]
	mov	w1, w2
	mov	x0, x21
	bl	FlashReadPages
	ldr	w0, [x21]
	cmn	w0, #1
	bne	.L672
	ldr	w0, [x21, 4]
	mov	w2, 1
	mov	w1, w2
	add	w0, w0, 1
	str	w0, [x21, 4]
	mov	x0, x21
	bl	FlashReadPages
.L672:
	ldr	w0, [x21]
	cmn	w0, #1
	beq	.L673
	ldrh	w0, [x22]
	cmp	w0, w24
	bne	.L673
	ldr	w1, [x22, 4]
	str	w1, [x23, 368]
	strh	w20, [x23, 360]
	ldrh	w1, [x22, 8]
	strh	w1, [x23, 364]
.L674:
	add	x21, x19, :lo12:.LANCHOR0
	mov	w0, 65535
	ldrh	w1, [x21, 360]
	cmp	w1, w0
	beq	.L688
	ldrh	w1, [x21, 364]
	cmp	w1, w0
	beq	.L678
	add	x0, x21, 3720
	lsl	w1, w1, 10
	mov	w2, 1
	str	w1, [x0, 4]
	mov	w1, w2
	bl	FlashReadPages
	ldr	w0, [x21, 3720]
	cmn	w0, #1
	beq	.L678
	ldrh	w1, [x22]
	mov	w0, 61649
	cmp	w1, w0
	bne	.L678
	ldr	w1, [x21, 368]
	ldr	w0, [x22, 4]
	cmp	w0, w1
	bls	.L678
	str	w0, [x21, 368]
	ldrh	w1, [x21, 364]
	ldrh	w0, [x22, 8]
	strh	w1, [x21, 360]
	strh	w0, [x21, 364]
.L678:
	add	x20, x19, :lo12:.LANCHOR0
	mov	w1, 1
	add	x23, x20, 3720
	mov	w24, 61649
	ldrh	w0, [x20, 360]
	bl	FtlGetLastWrittenPage
	sxth	w21, w0
	add	w0, w0, 1
	strh	w0, [x20, 362]
.L680:
	tbz	w21, #31, .L683
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 251
	add	x1, x1, 304
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L682:
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w0, [x22, 10]
	mov	w2, 65535
	strh	w0, [x1, 366]
	ldrh	w0, [x22, 12]
	cmp	w0, w2
	beq	.L685
	ldr	w2, [x1, 232]
	cmp	w0, w2
	beq	.L685
	ldrh	w1, [x1, 246]
	lsr	w1, w1, 2
	cmp	w2, w1
	bcs	.L685
	cmp	w0, w1
	bcs	.L685
	bl	FtlSysBlkNumInit
.L685:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w20, 0
	add	x21, x19, 392
.L686:
	ldrh	w0, [x19, 258]
	cmp	w20, w0
	bcc	.L687
	mov	w0, 0
.L670:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 64
	ret
.L673:
	sub	w20, w20, #1
	and	w20, w20, 65535
	b	.L671
.L683:
	ldrh	w0, [x20, 360]
	mov	w2, 1
	mov	w1, w2
	orr	w0, w21, w0, lsl 10
	str	w0, [x23, 4]
	ldr	x0, [x20, 3456]
	str	x0, [x23, 8]
	mov	x0, x23
	bl	FlashReadPages
	ldr	w0, [x23]
	cmn	w0, #1
	beq	.L681
	ldrh	w0, [x22]
	cmp	w0, w24
	beq	.L682
.L681:
	sub	w21, w21, #1
	sxth	w21, w21
	b	.L680
.L687:
	ldrh	w2, [x19, 3632]
	ldr	x0, [x19, 3728]
	mul	w1, w2, w20
	lsl	w2, w2, 2
	add	w20, w20, 1
	add	x1, x0, x1, lsl 2
	ldr	x0, [x21], 8
	bl	ftl_memcpy
	b	.L686
.L688:
	mov	w0, -1
	b	.L670
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	stp	x29, x30, [sp, -144]!
	ubfiz	x1, x1, 5, 32
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	adrp	x21, .LANCHOR0
	stp	x27, x28, [sp, 80]
	mov	w27, w2
	add	x2, x21, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	stp	x23, x24, [sp, 48]
	adrp	x23, .LANCHOR1
	stp	x25, x26, [sp, 64]
	add	x23, x23, :lo12:.LANCHOR1
	adrp	x25, .LC8
	mov	x19, x0
	ldrh	w26, [x2, 12]
	add	x22, x0, x1
	mov	x20, x0
	add	x23, x23, 320
	add	x25, x25, :lo12:.LC8
	lsl	w28, w26, 3
.L701:
	cmp	x20, x22
	bne	.L715
	adrp	x20, .LC95
	adrp	x23, .LC96
	add	x20, x20, :lo12:.LC95
	add	x23, x23, :lo12:.LC96
.L716:
	cmp	x19, x22
	beq	.L709
	add	x0, x21, :lo12:.LANCHOR0
	mov	w2, w27
	ldr	x1, [x0, 3496]
	str	wzr, [x1]
	ldr	x1, [x0, 3512]
	str	wzr, [x1]
	ldr	w1, [x19, 4]
	str	w1, [x29, 116]
	ldr	x1, [x0, 3496]
	ldr	x0, [x0, 3512]
	stp	x1, x0, [x29, 120]
	mov	w1, 1
	add	x0, x29, 112
	bl	FlashReadPages
	ldr	w24, [x29, 112]
	cmn	w24, #1
	bne	.L717
	ldr	w1, [x19, 4]
	mov	x0, x20
	bl	sftl_printk
	str	w24, [x19]
.L717:
	ldr	w24, [x29, 112]
	cmp	w24, 256
	bne	.L718
	ldr	w1, [x19, 4]
	mov	x0, x23
	bl	sftl_printk
	str	w24, [x19]
.L718:
	ldr	x0, [x19, 16]
	cbz	x0, .L719
	ldr	w2, [x0]
	add	x0, x21, :lo12:.LANCHOR0
	ldr	x0, [x0, 3512]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L719
	ldr	w1, [x19, 4]
	adrp	x0, .LC97
	add	x0, x0, :lo12:.LC97
	bl	sftl_printk
	mov	w0, -1
	str	w0, [x19]
.L719:
	ldr	x0, [x19, 8]
	cbz	x0, .L720
	ldr	w2, [x0]
	add	x0, x21, :lo12:.LANCHOR0
	ldr	x0, [x0, 3496]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L720
	ldr	w1, [x19, 4]
	adrp	x0, .LC98
	add	x0, x0, :lo12:.LC98
	bl	sftl_printk
	mov	w0, -1
	str	w0, [x19]
.L720:
	add	x19, x19, 32
	b	.L716
.L715:
	ldr	x0, [x20, 8]
	cbz	x0, .L702
	ldr	x0, [x20, 16]
	cbnz	x0, .L703
.L702:
	mov	w2, 148
	mov	x1, x23
	mov	x0, x25
	bl	sftl_printk
.L703:
	ldr	w0, [x20, 4]
	add	x2, x29, 104
	add	x1, x29, 108
	bl	l2p_addr_tran.isra.0
	ldr	w0, [x29, 104]
	cmp	w0, 3
	bls	.L704
.L737:
	mov	w0, -1
	str	w0, [x20]
	b	.L705
.L704:
	cbnz	w0, .L706
	ldr	w0, [x29, 108]
	cmp	w28, w0
	bls	.L706
	adrp	x20, .LANCHOR1
	add	x20, x20, :lo12:.LANCHOR1
	adrp	x21, .LC86
	add	x20, x20, 320
	add	x21, x21, :lo12:.LC86
	b	.L735
.L708:
	ldr	w2, [x19, 4]
	mov	w0, -1
	str	w0, [x19]
	mov	x1, x20
	mov	x0, x21
	add	x19, x19, 32
	bl	sftl_printk
	ldr	x1, [x19, -24]
	mov	w3, 16
	mov	w2, 4
	adrp	x0, .LC87
	add	x0, x0, :lo12:.LC87
	bl	rknand_print_hex
	ldr	x1, [x19, -16]
	mov	w3, 4
	adrp	x0, .LC88
	mov	w2, w3
	add	x0, x0, :lo12:.LC88
	bl	rknand_print_hex
.L735:
	cmp	x19, x22
	bne	.L708
	bl	dump_stack
.L709:
	ldp	x19, x20, [sp, 16]
	mov	w0, 0
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 144
	ret
.L706:
	ldr	x1, [x20, 8]
	tst	x1, 63
	beq	.L722
	add	x0, x21, :lo12:.LANCHOR0
	ldr	x24, [x0, 3496]
	cmp	x1, x24
	beq	.L710
	ldrh	w2, [x0, 262]
	mov	x0, x24
	lsl	w2, w2, 9
	bl	ftl_memcpy
.L710:
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w1, [x29, 108]
	ldr	x3, [x20, 16]
	mov	x2, x24
	ldr	x4, [x0, 3376]
	ldrb	w0, [x29, 104]
	blr	x4
	cbnz	w0, .L711
	str	wzr, [x20]
.L712:
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w1, [x0, 14]
	cmp	w1, 4
	bne	.L705
	ldr	x4, [x0, 3376]
	add	x2, x24, 2048
	ldrb	w0, [x29, 104]
	ldr	w1, [x29, 108]
	ldr	x3, [x20, 16]
	add	w1, w26, w1
	add	x3, x3, 8
	blr	x4
	cbnz	w0, .L737
.L705:
	add	x20, x20, 32
	b	.L701
.L722:
	mov	x24, x1
	b	.L710
.L711:
	mov	w0, -1
	str	w0, [x20]
	b	.L712
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	stp	x29, x30, [sp, -112]!
	mov	w10, 0
	mov	w9, 4
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	adrp	x21, .LANCHOR0
	add	x5, x21, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	stp	x23, x24, [sp, 48]
	and	w22, w1, 255
	stp	x25, x26, [sp, 64]
	add	x8, x5, 264
	stp	x27, x28, [sp, 80]
	and	w23, w0, 65535
	mov	w20, 0
	mov	w19, 0
	str	w23, [x5, 3428]
.L739:
	ldrh	w0, [x5, 236]
	cmp	w0, w10
	bhi	.L743
	cbz	w20, .L738
	ldr	x0, [x5, 3408]
	mov	w2, w20
	mov	w1, 0
	ubfiz	x20, x20, 5, 16
	mov	x24, 0
	add	x25, x21, :lo12:.LANCHOR0
	bl	FlashEraseBlocks
.L747:
	ldr	x0, [x25, 3408]
	add	x1, x0, x24
	ldr	w0, [x0, x24]
	cmn	w0, #1
	bne	.L746
	ldr	w0, [x1, 4]
	add	w19, w19, 1
	and	w19, w19, 65535
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
.L746:
	add	x24, x24, 32
	cmp	x24, x20
	bne	.L747
	cbz	w22, .L760
	add	x0, x21, :lo12:.LANCHOR0
	mov	w25, 1
	ldrh	w26, [x0, 308]
.L748:
	add	x28, x21, :lo12:.LANCHOR0
	mov	w24, 0
	add	x0, x28, 264
	str	x0, [x29, 104]
.L756:
	mov	w5, 0
	mov	w20, 0
.L749:
	ldrh	w0, [x28, 236]
	cmp	w0, w5
	bhi	.L752
	cbz	w20, .L738
	ldr	x0, [x28, 3408]
	mov	w1, w20
	mov	w3, 1
	mov	w2, w25
	mov	x27, 0
	bl	FlashProgPages
	ubfiz	x1, x20, 5, 16
.L755:
	ldr	x0, [x28, 3408]
	add	x3, x0, x27
	ldr	w0, [x0, x27]
	cbz	w0, .L754
	ldr	w0, [x3, 4]
	add	w19, w19, 1
	str	x1, [x29, 96]
	and	w19, w19, 65535
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
	ldr	x1, [x29, 96]
.L754:
	add	x27, x27, 32
	cmp	x1, x27
	bne	.L755
	add	w24, w24, 1
	cmp	w26, w24, uxth
	bhi	.L756
	add	x26, x21, :lo12:.LANCHOR0
	mov	x24, 0
.L758:
	cbz	w22, .L757
	ldr	x0, [x26, 3408]
	add	x1, x0, x24
	ldr	w0, [x0, x24]
	cbnz	w0, .L757
	ldr	w0, [x1, 4]
	mov	w1, 1
	lsr	w0, w0, 10
	bl	FtlFreeSysBlkQueueIn
.L757:
	add	x24, x24, 32
	cmp	x24, x27
	bne	.L758
	cmp	w23, 63
	ccmp	w22, 0, 0, hi
	beq	.L738
	add	x21, x21, :lo12:.LANCHOR0
	mov	w2, w20
	mov	w1, w25
	ldr	x0, [x21, 3408]
	bl	FlashEraseBlocks
.L738:
	mov	w0, w19
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 112
	ret
.L743:
	ldr	x1, [x5, 3408]
	ubfiz	x0, x10, 5, 16
	str	wzr, [x1, x0]
	mov	w1, w23
	ldrb	w0, [x8, w10, sxtw]
	bl	V2P_block
	and	w6, w0, 65535
	mov	w7, w6
	cbz	w22, .L740
	bl	IsBlkInVendorPart
	cbnz	w0, .L741
.L740:
	mov	w0, w7
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L742
	ldr	x0, [x5, 3408]
	ubfiz	x2, x20, 5, 16
	lsl	w6, w6, 10
	add	x0, x0, x2
	str	w6, [x0, 4]
	ldr	x1, [x5, 3408]
	ldr	x0, [x5, 3488]
	add	x1, x1, x2
	ldr	x2, [x5, 3528]
	str	x0, [x1, 8]
	ldrh	w0, [x5, 316]
	mul	w0, w0, w20
	add	w20, w20, 1
	and	w20, w20, 65535
	sdiv	w0, w0, w9
	add	x0, x2, x0, sxtw 2
	str	x0, [x1, 16]
.L741:
	add	w10, w10, 1
	and	w10, w10, 65535
	b	.L739
.L742:
	add	w19, w19, 1
	and	w19, w19, 65535
	b	.L741
.L760:
	mov	w25, 0
	mov	w26, 2
	b	.L748
.L752:
	ldr	x1, [x28, 3408]
	ubfiz	x0, x5, 5, 16
	str	wzr, [x1, x0]
	mov	w1, w23
	ldr	x0, [x29, 104]
	ldrb	w0, [x0, w5, sxtw]
	bl	V2P_block
	and	w6, w0, 65535
	mov	w7, w6
	cbz	w22, .L750
	bl	IsBlkInVendorPart
	cbnz	w0, .L751
.L750:
	mov	w0, w7
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L751
	ldr	x0, [x28, 3408]
	ubfiz	x2, x20, 5, 16
	add	w6, w24, w6, lsl 10
	add	x0, x0, x2
	str	w6, [x0, 4]
	ldr	x1, [x28, 3408]
	ldr	x0, [x28, 3480]
	add	x1, x1, x2
	mov	w2, 4
	str	x0, [x1, 8]
	ldrh	w0, [x28, 316]
	mul	w0, w0, w20
	add	w20, w20, 1
	and	w20, w20, 65535
	sdiv	w0, w0, w2
	ldr	x2, [x28, 3488]
	add	x0, x2, x0, sxtw 2
	str	x0, [x1, 16]
.L751:
	add	w5, w5, 1
	and	w5, w5, 65535
	b	.L749
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	and	w19, w0, 65535
	cmp	w19, 11
	bls	.L784
	adrp	x20, .LANCHOR0
	add	x20, x20, :lo12:.LANCHOR0
	mov	w2, 32
	mov	w1, 165
	lsl	w19, w19, 10
	ldr	x0, [x20, 3504]
	str	x0, [x29, 40]
	add	x0, x29, 64
	str	x0, [x29, 48]
	bl	ftl_memset
	ldr	x0, [x20, 3504]
	mov	w2, 8
	mov	w1, 90
	bl	ftl_memset
	str	w19, [x29, 36]
	mov	w2, 1
	add	x0, x29, 32
	mov	w1, w2
	bl	FlashEraseBlocks
	mov	w3, 1
	add	x0, x29, 32
	mov	w2, w3
	mov	w1, w3
	bl	FlashProgPages
	ldr	w0, [x29, 32]
	cbnz	w0, .L785
	add	w0, w19, 1
	mov	w3, 1
	str	w0, [x29, 36]
	mov	w2, w3
	mov	w1, w3
	add	x0, x29, 32
	bl	FlashProgPages
	ldr	w0, [x29, 32]
	cmp	w0, 0
	csetm	w20, ne
.L783:
	mov	w2, 1
	mov	w1, 0
	add	x0, x29, 32
	str	w19, [x29, 36]
	bl	FlashEraseBlocks
.L781:
	mov	w0, w20
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 128
	ret
.L785:
	mov	w20, -1
	b	.L783
.L784:
	mov	w20, 0
	b	.L781
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FtlBbmTblFlush
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	stp	x29, x30, [sp, -80]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	w19, 0
	stp	x25, x26, [sp, 64]
	adrp	x25, .LANCHOR0
	add	x20, x25, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	add	x21, x20, 392
	ldr	x2, [x20, 3520]
	ldr	x0, [x20, 3456]
	str	x2, [x20, 3736]
	ldrh	w2, [x20, 314]
	str	x0, [x20, 3728]
	bl	ftl_memset
.L788:
	ldrh	w0, [x20, 258]
	add	x1, x20, 3720
	cmp	w19, w0
	blt	.L789
	ldr	x26, [x1, 16]
	add	x19, x20, 360
	mov	w2, 16
	mov	w1, 255
	mov	x0, x26
	bl	ftl_memset
	mov	w0, -3887
	adrp	x23, .LC99
	strh	w0, [x26]
	add	x25, x25, :lo12:.LANCHOR0
	add	x23, x23, :lo12:.LC99
	mov	w21, 0
	ldr	w0, [x19, 8]
	mov	w22, 0
	str	w0, [x26, 4]
	adrp	x24, .LC100
	ldrh	w0, [x20, 360]
	strh	w0, [x26, 2]
	ldrh	w0, [x19, 4]
	strh	w0, [x26, 8]
	ldrh	w0, [x19, 6]
	strh	w0, [x26, 10]
	ldr	w0, [x20, 232]
	strh	w0, [x26, 12]
	ldr	w0, [x20, 2676]
	strh	w0, [x26, 14]
.L790:
	ldr	x0, [x25, 3456]
	str	x0, [x25, 3728]
	ldr	x0, [x25, 3520]
	str	x0, [x25, 3736]
	ldrh	w1, [x19]
	ldrh	w2, [x19, 2]
	ldrh	w3, [x19, 4]
	ldrh	w4, [x26, 10]
	orr	w0, w2, w1, lsl 10
	str	wzr, [x25, 3720]
	str	w0, [x25, 3724]
	mov	x0, x23
	bl	sftl_printk
	ldrh	w0, [x25, 308]
	ldrh	w1, [x19, 2]
	sub	w0, w0, #1
	cmp	w1, w0
	blt	.L791
	ldr	w0, [x19, 8]
	mov	w2, 1
	ldrh	w1, [x19]
	add	w0, w0, 1
	str	w0, [x19, 8]
	str	w0, [x26, 4]
	strh	w1, [x26, 8]
	ldrh	w0, [x19, 4]
	strh	w1, [x19, 4]
	ldr	x1, [x25, 3408]
	strh	w0, [x19]
	lsl	w0, w0, 10
	str	w0, [x25, 3724]
	strh	wzr, [x19, 2]
	str	w0, [x1, 4]
	mov	w1, w2
	ldr	x0, [x25, 3408]
	bl	FlashEraseBlocks
.L791:
	add	x20, x25, 3720
	mov	w3, 1
	mov	x0, x20
	mov	w2, w3
	mov	w1, w3
	bl	FlashProgPages
	ldrh	w0, [x19, 2]
	add	w0, w0, 1
	strh	w0, [x19, 2]
	ldr	w0, [x25, 3720]
	cmn	w0, #1
	bne	.L792
	ldr	w1, [x25, 3724]
	add	w21, w21, 1
	add	x0, x24, :lo12:.LC100
	and	w21, w21, 65535
	bl	sftl_printk
	cmp	w21, 3
	bls	.L790
	ldr	w1, [x25, 3724]
	adrp	x0, .LC101
	mov	w2, w21
	add	x0, x0, :lo12:.LC101
	bl	sftl_printk
.L794:
	b	.L794
.L789:
	ldrh	w2, [x20, 3632]
	ldr	x0, [x1, 8]
	ldr	x1, [x21], 8
	mul	w3, w19, w2
	lsl	w2, w2, 2
	add	w19, w19, 1
	add	x0, x0, x3, sxtw 2
	bl	ftl_memcpy
	b	.L788
.L795:
	mov	w22, 1
	b	.L790
.L792:
	add	w22, w22, 1
	cmp	w22, 1
	ble	.L795
	cmp	w0, 256
	beq	.L790
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	allocate_data_superblock
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	adrp	x21, .LANCHOR0
	add	x22, x21, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	stp	x23, x24, [sp, 48]
	mov	x20, x0
	mov	x19, x22
	add	x23, x22, 128
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
.L801:
	ldrh	w1, [x22, 2584]
	ldrh	w0, [x22, 228]
	add	w0, w0, w1
	ldrh	w1, [x22, 244]
	cmp	w0, w1
	ble	.L802
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2660
	add	x1, x1, 336
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L802:
	cmp	x20, x23
	bne	.L828
	ldrh	w0, [x19, 228]
	ldr	w1, [x19, 3236]
	mul	w1, w0, w1
	lsr	w0, w0, 1
	add	w0, w0, 1
	add	w1, w0, w1, lsr 2
	ands	w1, w1, 65535
	beq	.L803
	sub	w1, w1, #1
	and	w1, w1, 65535
.L803:
	add	x0, x19, 2592
	bl	List_pop_index_node
	and	w26, w0, 65535
	ldrh	w0, [x19, 228]
	cbnz	w0, .L804
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2669
	add	x1, x1, 336
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L804:
	ldrh	w0, [x19, 228]
	sub	w0, w0, #1
	strh	w0, [x19, 228]
	ldrh	w0, [x19, 244]
	cmp	w0, w26
	bls	.L801
	ldr	x0, [x19, 72]
	ubfiz	x25, x26, 1, 16
	ldrh	w24, [x0, x25]
	cbnz	w24, .L801
	strh	w26, [x20]
	mov	x0, x20
	bl	make_superblock
	ldrb	w0, [x20, 7]
	cbnz	w0, .L806
	ldr	x0, [x19, 72]
	mov	w1, -1
	strh	w1, [x0, x25]
	mov	w0, w26
	bl	INSERT_DATA_LIST
	ldrh	w1, [x19, 2584]
	ldrh	w0, [x19, 228]
	add	w0, w0, w1
	ldrh	w1, [x19, 244]
	cmp	w0, w1
	ble	.L801
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	mov	w2, 2683
	add	x1, x1, 336
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	b	.L801
.L828:
	mov	w1, 0
	b	.L803
.L806:
	ldrh	w1, [x19, 2584]
	ldrh	w0, [x19, 228]
	add	w0, w0, w1
	ldrh	w1, [x19, 244]
	cmp	w0, w1
	ble	.L808
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2686
	add	x1, x1, 336
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L808:
	add	x27, x20, 16
	ldrh	w4, [x19, 236]
	mov	x3, x27
	mov	x0, 0
	mov	w5, 65535
.L809:
	cmp	w4, w0, uxth
	bhi	.L811
	cbnz	w24, .L812
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2697
	add	x1, x1, 336
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L812:
	ldrh	w0, [x19, 176]
	cmp	w0, w26
	bne	.L813
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2699
	add	x1, x1, 336
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L813:
	ldrb	w0, [x20, 8]
	ldr	x1, [x19, 2600]
	cbnz	w0, .L814
	ldrh	w0, [x1, x25]
	cbz	w0, .L815
	ldrh	w2, [x19, 296]
	add	w0, w0, w2
.L836:
	strh	w0, [x1, x25]
	ldr	w0, [x19, 2668]
	add	w0, w0, 1
	str	w0, [x19, 2668]
.L817:
	ldr	x0, [x19, 2600]
	ldr	w1, [x19, 2684]
	ldrh	w0, [x0, x25]
	cmp	w0, w1
	bls	.L818
	str	w0, [x19, 2684]
.L818:
	ldr	w1, [x19, 2672]
	ubfiz	x3, x24, 5, 16
	ldrh	w0, [x19, 296]
	ldr	w2, [x19, 2668]
	madd	w0, w0, w2, w1
	ldrh	w1, [x19, 244]
	udiv	w0, w0, w1
	ldr	x1, [x19, 3552]
	str	w0, [x19, 2676]
	ldr	w0, [x1, 16]
	add	w0, w0, 1
	str	w0, [x1, 16]
	mov	x0, 0
.L819:
	cmp	x3, x0
	bne	.L820
	ldrb	w1, [x20, 8]
	mov	w2, w24
	ldr	x0, [x19, 3408]
	mov	x28, 0
	bl	FlashEraseBlocks
	mov	w1, 0
.L821:
	cmp	w24, w28, uxth
	bhi	.L823
	cmp	w1, 0
	ble	.L824
	mov	w0, w26
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L824:
	ldrb	w0, [x20, 7]
	cbnz	w0, .L825
	ldr	x0, [x19, 72]
	mov	w1, -1
	strh	w1, [x0, x25]
	b	.L801
.L811:
	ldr	x1, [x19, 3408]
	add	x2, x1, x0, lsl 5
	stp	xzr, xzr, [x2, 8]
	ldrh	w2, [x3]
	cmp	w2, w5
	beq	.L810
	ubfiz	x6, x24, 5, 16
	add	w24, w24, 1
	add	x1, x1, x6
	and	w24, w24, 65535
	lsl	w2, w2, 10
	str	w2, [x1, 4]
.L810:
	add	x0, x0, 1
	add	x3, x3, 2
	b	.L809
.L815:
	mov	w0, 2
	b	.L836
.L814:
	ldrh	w0, [x1, x25]
	add	w0, w0, 1
	strh	w0, [x1, x25]
	ldr	w0, [x19, 2672]
	add	w0, w0, 1
	str	w0, [x19, 2672]
	b	.L817
.L820:
	ldr	x1, [x19, 3408]
	add	x1, x1, x0
	add	x0, x0, 32
	ldr	w2, [x1, 4]
	and	w2, w2, -1024
	str	w2, [x1, 4]
	b	.L819
.L823:
	ldr	x2, [x19, 3408]
	lsl	x0, x28, 5
	add	x3, x2, x0
	ldr	w2, [x2, x0]
	cmn	w2, #1
	bne	.L822
	add	w1, w1, 1
	ldr	w0, [x3, 4]
	stp	w2, w1, [x29, 104]
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
	ldp	w2, w1, [x29, 104]
	strh	w2, [x27]
	ldrb	w0, [x20, 7]
	sub	w0, w0, #1
	strb	w0, [x20, 7]
.L822:
	add	x28, x28, 1
	add	x27, x27, 2
	b	.L821
.L825:
	add	x21, x21, :lo12:.LANCHOR0
	strh	wzr, [x20, 2]
	strb	wzr, [x20, 6]
	ldrh	w1, [x21, 306]
	strh	w26, [x20]
	mul	w0, w0, w1
	ldr	w1, [x21, 2660]
	str	w1, [x20, 12]
	and	w0, w0, 65535
	add	w1, w1, 1
	strh	w0, [x20, 4]
	str	w1, [x21, 2660]
	ldr	x1, [x21, 72]
	ldrh	w2, [x20]
	strh	w0, [x1, x2, lsl 1]
	ldrh	w0, [x20, 4]
	cbz	w0, .L826
	ldrb	w0, [x20, 7]
	cbnz	w0, .L827
.L826:
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2752
	add	x1, x1, 336
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L827:
	ldp	x19, x20, [sp, 16]
	mov	w0, 0
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 112
	ret
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	and	w22, w0, 65535
	stp	x23, x24, [sp, 48]
	stp	x25, x26, [sp, 64]
	ldrh	w0, [x19, 3312]
	str	x27, [sp, 80]
	cbz	w0, .L838
	add	x23, x19, 264
	add	x24, x19, 3320
	mov	w21, 0
.L839:
	ldrh	w0, [x19, 236]
	cmp	w0, w21
	bhi	.L845
	bl	FtlGcReFreshBadBlk
.L838:
	mov	w0, 0
	ldr	x27, [sp, 80]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 96
	ret
.L845:
	ldrb	w0, [x23, w21, sxtw]
	mov	w1, w22
	adrp	x26, .LC102
	add	x26, x26, :lo12:.LC102
	mov	w20, 0
	bl	V2P_block
	and	w25, w0, 65535
.L840:
	ldrh	w0, [x19, 3312]
	cmp	w0, w20
	bhi	.L844
	add	w21, w21, 1
	and	w21, w21, 65535
	b	.L839
.L844:
	ldrh	w0, [x24, w20, sxtw 1]
	add	w27, w20, 1
	cmp	w0, w25
	bne	.L841
	mov	w1, w25
	mov	x0, x26
	bl	sftl_printk
	mov	w0, w25
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	w0, [x19, 3312]
	mov	w1, w27
.L842:
	cmp	w20, w0
	bcc	.L843
	sub	w0, w0, #1
	strh	w0, [x19, 3312]
.L841:
	and	w20, w27, 65535
	b	.L840
.L843:
	sub	w2, w1, #1
	ldrh	w3, [x24, w1, sxtw 1]
	add	w20, w20, 1
	add	w1, w1, 1
	and	w20, w20, 65535
	strh	w3, [x24, w2, sxtw 1]
	b	.L842
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	and	w20, w0, 65535
	adrp	x19, .LANCHOR0
	add	x0, x19, :lo12:.LANCHOR0
	str	x21, [sp, 32]
	ubfiz	x1, x20, 1, 16
	ldr	x2, [x0, 72]
	ldrh	w1, [x2, x1]
	cbnz	w1, .L851
	ldrh	w1, [x0, 226]
	cmp	w1, w20
	bne	.L852
	mov	w1, -1
	strh	w1, [x0, 226]
.L852:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 224]
	cmp	w1, w20
	bne	.L853
	mov	w1, -1
	strh	w1, [x0, 224]
.L853:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 176]
	cmp	w1, w20
	bne	.L854
	mov	w1, -1
	strh	w1, [x0, 176]
.L855:
	add	x21, x19, :lo12:.LANCHOR0
	mov	w1, w20
	add	x0, x21, 2568
	bl	List_remove_node
	ldrh	w0, [x21, 2584]
	cbnz	w0, .L857
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2828
	add	x1, x1, 368
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L857:
	add	x19, x19, :lo12:.LANCHOR0
	ldrh	w0, [x19, 2584]
	sub	w0, w0, #1
	strh	w0, [x19, 2584]
	mov	w0, w20
	bl	free_data_superblock
	mov	w0, w20
	bl	FtlGcFreeBadSuperBlk
	ldrh	w1, [x19, 2584]
	ldrh	w0, [x19, 228]
	add	w0, w0, w1
	ldrh	w1, [x19, 244]
	cmp	w0, w1
	ble	.L861
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2831
	add	x1, x1, 368
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L861:
	mov	w0, 1
	b	.L850
.L854:
	ldrh	w1, [x0, 24]
	cmp	w1, w20
	beq	.L860
	ldrh	w1, [x0, 80]
	cmp	w1, w20
	beq	.L860
	ldrh	w0, [x0, 128]
	cmp	w0, w20
	bne	.L855
.L860:
	mov	w0, 0
.L850:
	ldp	x19, x20, [sp, 16]
	ldr	x21, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
.L851:
	mov	w0, w20
	bl	List_update_data_list
	b	.L860
	.size	update_vpc_list, .-update_vpc_list
	.align	2
	.global	decrement_vpc_count
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	and	w20, w0, 65535
	stp	x21, x22, [sp, 32]
	mov	w0, 65535
	adrp	x19, .LANCHOR0
	cmp	w20, w0
	beq	.L864
	add	x22, x19, :lo12:.LANCHOR0
	ubfiz	x21, x20, 1, 16
	ldr	x1, [x22, 72]
	ldrh	w0, [x1, x21]
	cbnz	w0, .L865
	mov	w2, 0
	mov	w1, w20
	adrp	x0, .LC103
	add	x0, x0, :lo12:.LC103
	bl	sftl_printk
	ldr	x0, [x22, 72]
	ldrh	w0, [x0, x21]
	cbz	w0, .L866
.L871:
	mov	w21, 0
.L863:
	mov	w0, w21
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
.L866:
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	mov	w2, 2846
	add	x1, x1, 384
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	b	.L871
.L865:
	sub	w0, w0, #1
	strh	w0, [x1, x21]
.L864:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w1, 65535
	ldrh	w0, [x19, 3706]
	cmp	w0, w1
	bne	.L868
	strh	w20, [x19, 3706]
	b	.L871
.L868:
	cmp	w20, w0
	beq	.L871
	bl	update_vpc_list
	cmp	w0, 0
	ldr	x1, [x19, 2560]
	cset	w21, ne
	ldr	x0, [x19, 2568]
	strh	w20, [x19, 3706]
	sub	x0, x0, x1
	mov	x1, -6148914691236517206
	asr	x0, x0, 1
	movk	x1, 0xaaab, lsl 0
	mul	x0, x0, x1
	ldr	x1, [x19, 72]
	and	x2, x0, 65535
	ldrh	w1, [x1, x2, lsl 1]
	cbnz	w1, .L863
	cmp	w20, w0, uxth
	beq	.L863
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	mov	w2, 2862
	add	x1, x1, 384
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	b	.L863
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	stp	x29, x30, [sp, -96]!
	mov	w2, 61589
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	mov	w0, -1
	stp	x21, x22, [sp, 32]
	str	w0, [x29, 88]
	adrp	x21, .LANCHOR0
	add	x0, x21, :lo12:.LANCHOR0
	str	x23, [sp, 48]
	mov	x20, x0
	ldr	x23, [x0, 3520]
	ldr	x1, [x0, 3456]
	stp	x1, x23, [x29, 72]
	mov	w1, -3
	str	w1, [x23, 8]
	mov	w1, -2
	str	w1, [x23, 12]
	ldrh	w1, [x19]
	strh	w1, [x23, 2]
	strh	wzr, [x23]
	ldr	x1, [x0, 3456]
	str	w2, [x1]
	mov	w2, 22136
	movk	w2, 0x1234, lsl 16
	ldr	x1, [x0, 3456]
	str	w2, [x1, 4]
	ldrh	w22, [x19, 4]
	and	w22, w22, 1
	add	w22, w22, 6
.L878:
	ldrh	w0, [x19, 4]
	cbnz	w0, .L874
.L875:
	add	x21, x21, :lo12:.LANCHOR0
	ldrh	w0, [x19]
	ldrh	w3, [x19, 4]
	ldr	x2, [x21, 72]
	lsl	x0, x0, 1
	ldrh	w1, [x2, x0]
	sub	w1, w1, w3
	strh	w1, [x2, x0]
	strb	wzr, [x19, 6]
	ldrh	w0, [x21, 306]
	strh	w0, [x19, 2]
	strh	wzr, [x19, 4]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldr	x23, [sp, 48]
	ldp	x29, x30, [sp], 96
	ret
.L874:
	mov	x0, x19
	bl	get_new_active_ppa
	str	w0, [x29, 68]
	cmn	w0, #1
	beq	.L875
	ldr	w0, [x20, 2664]
	mov	w3, 0
	str	w0, [x23, 4]
	mov	w2, 0
	add	w0, w0, 1
	mov	w1, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x20, 2664]
	add	x0, x29, 64
	bl	FlashProgPages
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	subs	w22, w22, #1
	bne	.L878
	b	.L875
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlMakeBbt
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	stp	x25, x26, [sp, 64]
	stp	x21, x22, [sp, 32]
	mov	x21, x19
	stp	x23, x24, [sp, 48]
	mov	w24, 0
	stp	x27, x28, [sp, 80]
	bl	FtlBbtMemInit
	bl	FtlLoadFactoryBbt
	mov	w28, -3872
	add	x0, x19, :lo12:.LANCHOR0
	add	x26, x0, 392
	add	x25, x0, 372
	mov	x19, x0
	add	x20, x0, 3720
.L887:
	ldrh	w0, [x19, 258]
	cmp	w24, w0
	bcc	.L893
	add	x20, x21, :lo12:.LANCHOR0
	mov	w19, 0
.L894:
	ldrh	w0, [x20, 318]
	cmp	w0, w19
	bhi	.L895
	add	x20, x20, 360
	ldrh	w19, [x20, 12]
	mov	w22, 65535
	sub	w19, w19, #1
	and	w19, w19, 65535
.L896:
	ldrh	w0, [x20, 12]
	sub	w0, w0, #48
	cmp	w19, w0
	ble	.L900
	mov	w0, w19
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	beq	.L897
	mov	w0, w19
	bl	FlashTestBlk
	cbz	w0, .L898
	mov	w0, w19
	bl	FtlBbmMapBadBlock
.L897:
	sub	w19, w19, #1
	and	w19, w19, 65535
	b	.L896
.L893:
	ldr	x0, [x19, 3456]
	mov	w2, 65535
	ldr	x27, [x19, 3520]
	ldrh	w1, [x25]
	stp	x0, x27, [x20, 8]
	cmp	w1, w2
	beq	.L888
	ldrh	w23, [x19, 302]
	mov	w2, 1
	madd	w23, w23, w24, w1
	mov	w1, w2
	lsl	w0, w23, 10
	str	w0, [x20, 4]
	mov	x0, x20
	bl	FlashReadPages
	ldr	x1, [x20, 8]
	ldr	x0, [x26]
	ldrh	w2, [x19, 302]
	add	w2, w2, 7
	lsr	w2, w2, 3
	bl	ftl_memcpy
.L889:
	mov	w0, w23
	add	w24, w24, 1
	bl	FtlBbmMapBadBlock
	add	x26, x26, 8
	add	x25, x25, 2
	b	.L887
.L888:
	mov	w1, w24
	bl	FlashGetBadBlockList
	ldr	x0, [x20, 8]
	ldr	x1, [x26]
	bl	FtlBbt2Bitmap
	ldrh	w22, [x19, 302]
.L891:
	sub	w22, w22, #1
	and	w22, w22, 65535
.L890:
	ldrh	w0, [x19, 302]
	madd	w0, w24, w0, w22
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	beq	.L891
	ldr	x0, [x19, 3520]
	mov	w2, 16
	strh	w22, [x25]
	mov	w1, 0
	bl	ftl_memset
	ldr	x0, [x19, 3456]
	mov	w2, 4096
	mov	w1, 0
	bl	ftl_memset
	strh	w28, [x27]
	str	wzr, [x27, 4]
	ldrh	w23, [x19, 302]
	ldrh	w0, [x25]
	strh	w0, [x27, 2]
	ldrh	w2, [x19, 3632]
	ldrh	w0, [x25]
	ldr	x1, [x26]
	lsl	w2, w2, 2
	madd	w23, w23, w24, w0
	lsl	w0, w23, 10
	str	w0, [x20, 4]
	ldr	x0, [x20, 8]
	bl	ftl_memcpy
	mov	w2, 1
	mov	x0, x20
	mov	w1, w2
	bl	FlashEraseBlocks
	mov	w3, 1
	mov	x0, x20
	mov	w2, w3
	mov	w1, w3
	bl	FlashProgPages
	ldr	w0, [x20]
	cmn	w0, #1
	bne	.L889
	mov	w0, w23
	bl	FtlBbmMapBadBlock
	b	.L890
.L895:
	mov	w0, w19
	add	w19, w19, 1
	bl	FtlBbmMapBadBlock
	and	w19, w19, 65535
	b	.L894
.L898:
	ldrh	w0, [x20]
	cmp	w0, w22
	bne	.L899
	strh	w19, [x20]
	b	.L897
.L899:
	strh	w19, [x20, 4]
.L900:
	add	x21, x21, :lo12:.LANCHOR0
	mov	w2, 2
	ldr	x1, [x21, 3408]
	ldrh	w0, [x21, 360]
	str	wzr, [x21, 368]
	strh	wzr, [x21, 362]
	lsl	w0, w0, 10
	str	w0, [x1, 4]
	ldr	x0, [x21, 3408]
	ldrh	w1, [x21, 364]
	lsl	w1, w1, 10
	str	w1, [x0, 36]
	mov	w1, 1
	bl	FlashEraseBlocks
	ldrh	w0, [x21, 360]
	bl	FtlBbmMapBadBlock
	ldrh	w0, [x21, 364]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	strh	wzr, [x21, 362]
	ldr	w0, [x21, 368]
	ldrh	w1, [x21, 364]
	add	w0, w0, 1
	str	w0, [x21, 368]
	ldrh	w0, [x21, 360]
	strh	w0, [x21, 364]
	strh	w1, [x21, 360]
	bl	FtlBbmTblFlush
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	ftl_memcmp
	.type	ftl_memcmp, %function
ftl_memcmp:
	stp	x29, x30, [sp, -16]!
	uxtw	x2, w2
	add	x29, sp, 0
	bl	memcmp
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_memcmp, .-ftl_memcmp
	.align	2
	.global	js_hash
	.type	js_hash, %function
js_hash:
	mov	x4, x0
	mov	w0, 42982
	mov	x3, 0
	movk	w0, 0x47c6, lsl 16
.L909:
	cmp	w1, w3
	bhi	.L910
	ret
.L910:
	lsr	w2, w0, 2
	ldrb	w5, [x4, x3]
	add	w2, w2, w0, lsl 5
	add	x3, x3, 1
	add	w2, w2, w5
	eor	w0, w0, w2
	b	.L909
	.size	js_hash, .-js_hash
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	stp	x29, x30, [sp, -64]!
	mov	w1, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	ldr	x20, [x0, 16]
	ldrh	w0, [x0]
	cmp	w0, w1
	bne	.L912
	ldrh	w0, [x19, 8]
	cbz	w0, .L913
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 641
	add	x1, x1, 408
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L913:
	ldrh	w0, [x19, 8]
	add	w0, w0, 1
	strh	w0, [x19, 8]
	bl	FtlFreeSysBlkQueueOut
	strh	w0, [x20]
	strh	wzr, [x19, 2]
	ldr	w0, [x19, 48]
	strh	wzr, [x19]
	add	w0, w0, 1
	str	w0, [x19, 48]
.L914:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 64
	ret
.L912:
	ubfiz	x0, x0, 1, 16
	adrp	x21, .LANCHOR0
	ldrh	w1, [x19, 2]
	ldr	x24, [x19, 40]
	ldrh	w23, [x20, x0]
	add	x20, x21, :lo12:.LANCHOR0
	ldr	w0, [x19, 48]
	ldr	x22, [x20, 3520]
	orr	w1, w1, w23, lsl 10
	str	w1, [x20, 3724]
	ldr	x1, [x20, 3456]
	str	x1, [x20, 3728]
	str	x22, [x20, 3736]
	mov	w1, 255
	str	w0, [x22, 4]
	mov	w0, -1291
	strh	w0, [x22, 8]
	ldrh	w0, [x19, 4]
	strh	w0, [x22]
	strh	w23, [x22, 2]
	ldrh	w2, [x20, 308]
	ldr	x0, [x20, 3456]
	lsl	w2, w2, 3
	bl	ftl_memset
	mov	w1, 0
	mov	w0, 0
.L915:
	ldrh	w2, [x19, 6]
	cmp	w2, w0
	bhi	.L917
	add	x21, x21, :lo12:.LANCHOR0
	add	x6, x21, 3720
	ldrh	w1, [x21, 314]
	ldr	x0, [x21, 3728]
	bl	js_hash
	str	w0, [x22, 12]
	mov	w2, 1
	mov	w3, 0
	mov	w1, w2
	mov	x0, x6
	bl	FlashProgPages
	ldrh	w0, [x19, 2]
	add	w0, w0, 1
	strh	w0, [x19, 2]
	mov	x0, x19
	bl	ftl_map_blk_gc
	b	.L914
.L917:
	ubfiz	x2, x0, 2, 16
	ldr	w3, [x24, x2]
	cmp	w23, w3, lsr 10
	bne	.L916
	add	w1, w1, 1
	ldr	x4, [x20, 3456]
	and	w1, w1, 65535
	ubfiz	x3, x1, 3, 16
	str	w0, [x4, x3]
	ldr	w4, [x24, x2]
	ldr	x2, [x20, 3456]
	add	x2, x2, x3
	str	w4, [x2, 4]
.L916:
	add	w0, w0, 1
	and	w0, w0, 65535
	b	.L915
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	FtlMapWritePage
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	mov	w21, 0
	stp	x23, x24, [sp, 48]
	adrp	x24, .LANCHOR0
	add	x22, x24, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	stp	x25, x26, [sp, 64]
	adrp	x23, .LANCHOR1
	stp	x27, x28, [sp, 80]
	mov	x19, x0
	mov	w26, w1
	mov	x27, x2
	mov	x25, x24
	add	x23, x23, :lo12:.LANCHOR1
	add	x20, x22, 3720
.L923:
	add	x0, x24, :lo12:.LANCHOR0
	ldr	w1, [x0, 2644]
	add	w1, w1, 1
	str	w1, [x0, 2644]
	ldrh	w0, [x0, 308]
	ldrh	w1, [x19, 2]
	sub	w0, w0, #1
	cmp	w1, w0
	bge	.L924
	ldrh	w1, [x19]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L925
.L924:
	mov	x0, x19
	bl	Ftl_write_map_blk_to_last_page
.L925:
	ldrh	w1, [x19]
	ldr	x0, [x19, 16]
	ldrh	w0, [x0, x1, lsl 1]
	cbnz	w0, .L926
	adrp	x0, .LC8
	mov	w2, 700
	add	x1, x23, 440
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L926:
	ldrh	w1, [x19]
	ldrh	w0, [x19, 10]
	cmp	w1, w0
	bcc	.L927
	adrp	x0, .LC8
	mov	w2, 701
	add	x1, x23, 440
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L927:
	ldrh	w1, [x19]
	mov	w2, 16
	ldr	x0, [x19, 16]
	ldrh	w28, [x0, x1, lsl 1]
	mov	w1, 0
	ldrh	w0, [x19, 2]
	str	x27, [x20, 8]
	orr	w0, w0, w28, lsl 10
	str	w0, [x20, 4]
	ldr	x0, [x22, 3520]
	str	x0, [x20, 16]
	bl	ftl_memset
	ldr	x6, [x20, 16]
	ldr	w0, [x19, 48]
	str	w0, [x6, 4]
	strh	w26, [x6, 8]
	ldrh	w0, [x19, 4]
	strh	w0, [x6]
	strh	w28, [x6, 2]
	ldrh	w1, [x22, 314]
	ldr	x0, [x20, 8]
	bl	js_hash
	str	w0, [x6, 12]
	mov	w3, 1
	mov	x0, x20
	mov	w1, w3
	mov	w2, w3
	bl	FlashProgPages
	ldrh	w0, [x19, 2]
	add	w0, w0, 1
	and	w0, w0, 65535
	strh	w0, [x19, 2]
	ldr	w1, [x20]
	cmn	w1, #1
	bne	.L928
	ldr	w1, [x20, 4]
	adrp	x0, .LC104
	add	x0, x0, :lo12:.LC104
	add	w21, w21, 1
	and	w21, w21, 65535
	bl	sftl_printk
	ldrh	w0, [x19, 2]
	cmp	w0, 2
	bhi	.L929
	ldrh	w0, [x22, 308]
	sub	w0, w0, #1
	strh	w0, [x19, 2]
.L929:
	cmp	w21, 3
	bls	.L930
	add	x25, x25, :lo12:.LANCHOR0
	adrp	x0, .LC105
	mov	w2, w21
	add	x0, x0, :lo12:.LC105
	ldr	w1, [x25, 3724]
	bl	sftl_printk
.L931:
	b	.L931
.L930:
	ldr	w0, [x19, 52]
	cbz	w0, .L923
.L945:
	b	.L945
.L928:
	cmp	w0, 1
	beq	.L934
	cmp	w1, 256
	beq	.L934
	ldr	w0, [x19, 56]
	cbz	w0, .L935
.L934:
	str	wzr, [x19, 56]
	b	.L923
.L935:
	ldr	x0, [x19, 40]
	ldr	w1, [x20, 4]
	str	w1, [x0, w26, uxtw 2]
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	2
	.global	load_l2p_region
	.type	load_l2p_region, %function
load_l2p_region:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	and	w22, w0, 65535
	adrp	x21, .LANCHOR0
	add	x0, x21, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	and	x19, x1, 65535
	stp	x23, x24, [sp, 48]
	str	x25, [sp, 64]
	ldrh	w0, [x0, 340]
	cmp	w0, w22
	bcs	.L948
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 485
	add	x1, x1, 456
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L948:
	add	x20, x21, :lo12:.LANCHOR0
	ubfiz	x0, x22, 2, 16
	ldr	x1, [x20, 3608]
	ldr	w24, [x1, x0]
	cbnz	w24, .L949
	ldr	x0, [x20, 2608]
	lsl	x19, x19, 4
	ldrh	w2, [x20, 314]
	mov	w1, 255
	add	x0, x0, x19
	ldr	x0, [x0, 8]
	bl	ftl_memset
	ldr	x0, [x20, 2608]
	strh	w22, [x0, x19]
	ldr	x0, [x20, 2608]
	add	x19, x0, x19
	str	wzr, [x19, 4]
.L950:
	mov	w0, 0
	ldr	x25, [sp, 64]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 80
	ret
.L949:
	ldr	x0, [x20, 2608]
	lsl	x19, x19, 4
	add	x23, x20, 3720
	mov	w2, 1
	add	x0, x0, x19
	mov	w1, w2
	str	w24, [x20, 3724]
	ldr	x0, [x0, 8]
	str	x0, [x20, 3728]
	ldr	x0, [x20, 3520]
	str	x0, [x20, 3736]
	mov	x0, x23
	bl	FlashReadPages
	ldr	x25, [x20, 3736]
	ldrh	w0, [x25, 8]
	cmp	w0, w22
	beq	.L951
	mov	w2, w24
	mov	w1, w22
	adrp	x0, .LC106
	add	x0, x0, :lo12:.LC106
	bl	sftl_printk
	ldr	x1, [x20, 3736]
	mov	w3, 4
	adrp	x0, .LC94
	mov	w2, w3
	add	x0, x0, :lo12:.LC94
	bl	rknand_print_hex
	ldrh	w3, [x20, 340]
	adrp	x0, .LC107
	ldr	x1, [x20, 3608]
	mov	w2, 4
	add	x0, x0, :lo12:.LC107
	bl	rknand_print_hex
.L952:
	ldrh	w0, [x25, 8]
	cmp	w0, w22
	beq	.L953
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 508
	add	x1, x1, 456
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L953:
	add	x21, x21, :lo12:.LANCHOR0
	ldr	x0, [x21, 2608]
	add	x0, x0, x19
	str	wzr, [x0, 4]
	ldr	x0, [x21, 2608]
	strh	w22, [x0, x19]
	b	.L950
.L951:
	ldr	w0, [x20, 3720]
	cmp	w0, 256
	bne	.L952
	mov	w2, w24
	mov	w1, w22
	adrp	x0, .LC108
	add	x0, x0, :lo12:.LC108
	bl	sftl_printk
	ldr	x0, [x20, 2608]
	mov	w1, w22
	add	x0, x0, x19
	ldr	x2, [x0, 8]
	add	x0, x20, 3640
	bl	FtlMapWritePage
	b	.L952
	.size	load_l2p_region, .-load_l2p_region
	.align	2
	.global	ftl_map_blk_gc
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	stp	x21, x22, [sp, 32]
	adrp	x22, .LANCHOR0
	stp	x23, x24, [sp, 48]
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
	ldr	x20, [x0, 16]
	ldr	x25, [x0, 40]
	bl	ftl_free_no_use_map_blk
	ldrh	w1, [x19, 10]
	ldrh	w2, [x19, 8]
	sub	w1, w1, #5
	cmp	w2, w1
	blt	.L956
	ubfiz	x0, x0, 1, 16
	ldrh	w26, [x20, x0]
	cbz	w26, .L956
	ldr	w1, [x19, 52]
	cbnz	w1, .L956
	mov	w1, 1
	str	w1, [x19, 52]
	strh	wzr, [x20, x0]
	ldrh	w0, [x19, 8]
	ldrh	w1, [x19, 2]
	sub	w0, w0, #1
	strh	w0, [x19, 8]
	add	x0, x22, :lo12:.LANCHOR0
	ldrh	w0, [x0, 308]
	cmp	w1, w0
	bcc	.L957
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L957:
	add	x27, x22, :lo12:.LANCHOR0
	adrp	x23, .LANCHOR1
	add	x23, x23, :lo12:.LANCHOR1
	add	x21, x27, 3720
	add	x23, x23, 472
	mov	w20, 0
.L958:
	ldrh	w0, [x19, 6]
	cmp	w0, w20
	bhi	.L965
	mov	w1, 1
	mov	w0, w26
	bl	FtlFreeSysBlkQueueIn
	str	wzr, [x19, 52]
.L956:
	add	x22, x22, :lo12:.LANCHOR0
	ldrh	w1, [x19, 2]
	ldrh	w0, [x22, 308]
	cmp	w1, w0
	bcc	.L966
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L966:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 96
	ret
.L965:
	ubfiz	x0, x20, 2, 16
	add	x28, x25, x0
	ldr	w1, [x25, x0]
	cmp	w26, w1, lsr 10
	bne	.L959
	ldr	x1, [x27, 3464]
	mov	w2, 1
	ldr	x24, [x27, 3520]
	stp	x1, x24, [x21, 8]
	mov	w1, w2
	ldr	w0, [x25, x0]
	str	w0, [x21, 4]
	mov	x0, x21
	bl	FlashReadPages
	ldrh	w0, [x24, 8]
	cmp	w0, w20
	beq	.L960
	adrp	x0, .LC8
	mov	w2, 611
	mov	x1, x23
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L960:
	ldr	w0, [x21]
	cmn	w0, #1
	bne	.L961
.L963:
	str	wzr, [x28]
.L962:
	b	.L962
.L961:
	ldrh	w0, [x24, 8]
	cmp	w0, w20
	bne	.L963
	ldrh	w1, [x24]
	ldrh	w0, [x19, 4]
	cmp	w1, w0
	bne	.L963
	ldr	x2, [x21, 8]
	mov	w1, w20
	mov	x0, x19
	bl	FtlMapWritePage
.L959:
	add	w20, w20, 1
	and	w20, w20, 65535
	b	.L958
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	flush_l2p_region
	.type	flush_l2p_region, %function
flush_l2p_region:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	ubfiz	x20, x0, 4, 16
	ldr	x0, [x19, 2608]
	add	x1, x0, x20
	ldr	x2, [x1, 8]
	ldrh	w1, [x0, x20]
	add	x0, x19, 3640
	bl	FtlMapWritePage
	ldr	x0, [x19, 2608]
	add	x0, x0, x20
	ldr	w1, [x0, 4]
	and	w1, w1, 2147483647
	str	w1, [x0, 4]
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	flush_l2p_region, .-flush_l2p_region
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	stp	x25, x26, [sp, 64]
	mov	w25, w0
	add	x0, x20, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	mov	x22, x1
	mov	w23, w2
	ldrh	w24, [x0, 312]
	ldr	w0, [x0, 2624]
	cmp	w25, w0
	bcc	.L975
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 813
	add	x1, x1, 488
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L975:
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w1, [x0, 2624]
	cmp	w25, w1
	bcs	.L976
	add	w24, w24, 7
	ldrh	w1, [x0, 342]
	ldr	x2, [x0, 2608]
	mov	x0, 0
	lsr	w21, w25, w24
	and	w21, w21, 65535
.L977:
	and	x19, x0, 65535
	cmp	w19, w1
	bcc	.L982
	bl	select_l2p_ram_region
	and	x19, x0, 65535
	add	x2, x20, :lo12:.LANCHOR0
	ubfiz	x1, x19, 4, 16
	mov	w26, w0
	ldr	x2, [x2, 2608]
	add	x3, x2, x1
	ldrh	w2, [x2, x1]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L983
	ldr	w1, [x3, 4]
	tbz	w1, #31, .L983
	bl	flush_l2p_region
.L983:
	mov	w1, w26
	mov	w0, w21
	bl	load_l2p_region
	b	.L979
.L976:
	mov	w0, -1
	cbnz	w23, .L974
	str	w0, [x22]
.L974:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 80
	ret
.L982:
	add	x0, x0, 1
	add	x3, x2, x0, lsl 4
	ldrh	w3, [x3, -16]
	cmp	w3, w21
	bne	.L977
.L979:
	mov	x0, 1
	lsl	x0, x0, x24
	sub	w0, w0, #1
	and	w0, w0, w25
	and	x0, x0, 65535
	cbnz	w23, .L980
	add	x1, x20, :lo12:.LANCHOR0
	ldr	x1, [x1, 2608]
	add	x1, x1, x19, lsl 4
	ldr	x1, [x1, 8]
	ldr	w0, [x1, x0, lsl 2]
	str	w0, [x22]
.L981:
	add	x20, x20, :lo12:.LANCHOR0
	ldr	x0, [x20, 2608]
	add	x19, x0, x19, lsl 4
	ldr	w0, [x19, 4]
	cmn	w0, #1
	beq	.L985
	add	w0, w0, 1
	str	w0, [x19, 4]
.L985:
	mov	w0, 0
	b	.L974
.L980:
	add	x2, x20, :lo12:.LANCHOR0
	lsl	x1, x19, 4
	ldr	w4, [x22]
	ldr	x3, [x2, 2608]
	add	x3, x3, x1
	ldr	x3, [x3, 8]
	str	w4, [x3, x0, lsl 2]
	strh	w21, [x2, 2616]
	ldr	x0, [x2, 2608]
	add	x0, x0, x1
	ldr	w1, [x0, 4]
	orr	w1, w1, -2147483648
	str	w1, [x0, 4]
	b	.L981
	.size	log2phys, .-log2phys
	.align	2
	.global	FtlWriteDump_data
	.type	FtlWriteDump_data, %function
FtlWriteDump_data:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x20, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	str	x25, [sp, 64]
	ldrh	w2, [x20, 28]
	cbz	w2, .L989
	ldrb	w1, [x20, 32]
	cbnz	w1, .L989
	ldrb	w1, [x20, 31]
	ldrh	w3, [x20, 306]
	mul	w1, w1, w3
	cmp	w2, w1
	beq	.L989
	ldrb	w0, [x20, 34]
	cbnz	w0, .L988
	ldr	w22, [x20, 2624]
	add	x1, x29, 92
	ldrh	w24, [x20, 236]
	mov	w2, 0
	sub	w22, w22, #1
	mov	w0, w22
	bl	log2phys
	ldr	x21, [x20, 3520]
	ldr	w0, [x29, 92]
	ldr	x1, [x20, 3456]
	str	w0, [x29, 100]
	cmn	w0, #1
	stp	x1, x21, [x29, 104]
	str	w22, [x29, 120]
	str	wzr, [x21, 4]
	beq	.L991
	mov	w2, 0
	mov	w1, 1
	add	x0, x29, 96
	bl	FlashReadPages
.L992:
	add	x25, x19, :lo12:.LANCHOR0
	lsl	w24, w24, 2
	add	x20, x25, 24
	mov	w0, -3947
	mov	w23, 0
	strh	w0, [x21]
.L993:
	cmp	w24, w23
	bne	.L997
.L994:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, 1
	strb	w0, [x19, 34]
.L988:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldr	x25, [sp, 64]
	ldp	x29, x30, [sp], 128
	ret
.L991:
	ldrh	w2, [x20, 314]
	mov	w1, 255
	ldr	x0, [x20, 3456]
	bl	ftl_memset
	b	.L992
.L997:
	ldrh	w0, [x20, 4]
	cbz	w0, .L994
	ldr	w0, [x29, 100]
	add	w23, w23, 1
	stp	w22, w0, [x21, 8]
	ldrh	w0, [x20]
	strh	w0, [x21, 2]
	mov	x0, x20
	bl	get_new_active_ppa
	str	w0, [x29, 100]
	ldr	w0, [x25, 2664]
	mov	w3, 0
	str	w0, [x21, 4]
	mov	w2, 0
	add	w0, w0, 1
	mov	w1, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x25, 2664]
	add	x0, x29, 96
	bl	FlashProgPages
	ldrh	w0, [x20]
	bl	decrement_vpc_count
	b	.L993
.L989:
	add	x19, x19, :lo12:.LANCHOR0
	strb	wzr, [x19, 34]
	b	.L988
	.size	FtlWriteDump_data, .-FtlWriteDump_data
	.align	2
	.global	FtlReUsePrevPpa
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	mov	w21, w0
	stp	x19, x20, [sp, 16]
	lsr	w0, w1, 10
	str	w1, [x29, 76]
	adrp	x22, .LANCHOR0
	str	x23, [sp, 48]
	bl	P2V_block_in_plane
	add	x2, x22, :lo12:.LANCHOR0
	and	w0, w0, 65535
	ubfiz	x20, x0, 1, 16
	ldr	x3, [x2, 72]
	ldrh	w1, [x3, x20]
	cbnz	w1, .L1007
	ldr	x19, [x2, 2592]
	cbz	x19, .L1008
	ldrh	w3, [x2, 228]
	mov	x4, -6148914691236517206
	ldr	x2, [x2, 2560]
	movk	x4, 0xaaab, lsl 0
	mov	w5, 65535
	sub	x19, x19, x2
	asr	x19, x19, 1
	mul	x19, x19, x4
	mov	w4, 6
	and	w19, w19, 65535
.L1009:
	cmp	w1, w3
	beq	.L1008
	cmp	w19, w0
	bne	.L1010
	add	x23, x22, :lo12:.LANCHOR0
	mov	w1, w19
	add	x0, x23, 2592
	bl	List_remove_node
	ldrh	w0, [x23, 228]
	cbnz	w0, .L1011
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 1735
	add	x1, x1, 504
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1011:
	add	x22, x22, :lo12:.LANCHOR0
	ldrh	w0, [x22, 228]
	sub	w0, w0, #1
	strh	w0, [x22, 228]
	mov	w0, w19
	bl	INSERT_DATA_LIST
	ldr	x1, [x22, 72]
	ldrh	w0, [x1, x20]
	add	w0, w0, 1
	strh	w0, [x1, x20]
.L1008:
	add	x1, x29, 76
	mov	w2, 1
	mov	w0, w21
	bl	log2phys
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldr	x23, [sp, 48]
	ldp	x29, x30, [sp], 80
	ret
.L1010:
	umull	x19, w19, w4
	ldrh	w19, [x2, x19]
	cmp	w19, w5
	beq	.L1008
	add	w1, w1, 1
	and	w1, w1, 65535
	b	.L1009
.L1007:
	add	w1, w1, 1
	strh	w1, [x3, x20]
	b	.L1008
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	stp	x29, x30, [sp, -160]!
	mov	w1, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x20, x0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
	ldrh	w0, [x0]
	cmp	w0, w1
	beq	.L1138
	ldrb	w0, [x20, 6]
	adrp	x19, .LANCHOR0
	str	w0, [x29, 140]
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w26, [x20, 2]
	ldrh	w0, [x0, 306]
	cmp	w0, w26
	bne	.L1020
	strh	wzr, [x20, 4]
.L1145:
	strb	wzr, [x20, 6]
.L1138:
	ldp	x19, x20, [sp, 16]
	mov	w0, 0
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 160
	ret
.L1020:
	ldrh	w0, [x20, 16]
	mov	w21, 0
.L1021:
	cmp	w0, w1
	beq	.L1022
	mov	w1, 1
	bl	FtlGetLastWrittenPage
	mov	w23, w0
	cmn	w0, #1
	beq	.L1023
	add	x0, x19, :lo12:.LANCHOR0
	add	x28, x20, 16
	mov	x3, x28
	mov	w22, 0
	mov	w10, 65535
	mov	w6, 4
	ldrh	w1, [x0, 236]
	ldrh	w8, [x0, 314]
	add	x1, x1, 8
	ldrh	w9, [x0, 316]
	add	x1, x20, x1, lsl 1
.L1024:
	cmp	x1, x3
	bne	.L1028
	add	x24, x19, :lo12:.LANCHOR0
	mov	w2, 0
	mov	w1, w22
	ldr	x0, [x24, 3432]
	bl	FlashReadPages
	ldr	w21, [x24, 2664]
	ldr	x4, [x24, 3432]
	and	w9, w23, 65535
	sub	w21, w21, #1
	mov	w7, 65535
	mov	x6, x4
	mov	w3, 0
.L1029:
	cmp	w22, w3
	bne	.L1034
	add	w22, w23, 1
	ldr	w0, [x4, 4]
	and	w22, w22, 65535
.L1140:
	lsr	w0, w0, 10
	bl	P2V_plane
	and	w27, w0, 65535
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w0, [x0, 306]
	cmp	w0, w22
	bne	.L1036
	strh	w22, [x20, 2]
	strb	wzr, [x20, 6]
	strh	wzr, [x20, 4]
.L1036:
	ldr	w0, [x29, 140]
	cmp	w22, w26
	ccmp	w27, w0, 0, eq
	bne	.L1037
.L1146:
	mov	w2, w27
	mov	w1, w22
	mov	x0, x20
	bl	ftl_sb_update_avl_pages
	b	.L1138
.L1022:
	add	w21, w21, 1
	and	w21, w21, 65535
	add	x0, x20, x21, sxtw 1
	ldrh	w0, [x0, 16]
	b	.L1021
.L1023:
	cbz	w26, .L1025
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 1802
	add	x1, x1, 520
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1025:
	ldr	w0, [x29, 140]
	cmp	w0, 0
	ccmp	w21, w0, 4, ne
	beq	.L1026
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 1803
	add	x1, x1, 520
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1026:
	strh	wzr, [x20, 2]
	b	.L1145
.L1028:
	ldrh	w2, [x3]
	cmp	w2, w10
	beq	.L1027
	ldr	x5, [x0, 3432]
	ubfiz	x7, x22, 5, 16
	orr	w2, w23, w2, lsl 10
	add	x5, x5, x7
	str	w2, [x5, 4]
	mul	w2, w22, w8
	ldr	x4, [x0, 3432]
	ldr	x5, [x0, 3256]
	sdiv	w2, w2, w6
	add	x4, x4, x7
	add	x2, x5, x2, sxtw 2
	str	x2, [x4, 8]
	mul	w2, w22, w9
	ldr	x5, [x0, 3264]
	add	w22, w22, 1
	and	w22, w22, 65535
	sdiv	w2, w2, w6
	add	x2, x5, x2, sxtw 2
	str	x2, [x4, 16]
.L1027:
	add	x3, x3, 2
	b	.L1024
.L1034:
	ldr	w0, [x6]
	cbnz	w0, .L1030
	ldr	x8, [x6, 16]
	ldr	w5, [x8, 4]
	cmn	w5, #1
	beq	.L1031
	ldr	w1, [x24, 2664]
	mov	w0, w5
	bl	ftl_cmp_data_ver
	cbz	w0, .L1031
	add	w5, w5, 1
	str	w5, [x24, 2664]
.L1031:
	ldr	w0, [x8]
	cmn	w0, #1
	bne	.L1033
	ubfiz	x3, x3, 5, 16
	and	w22, w23, 65535
	add	x3, x4, x3
	ldr	w0, [x3, 4]
	b	.L1140
.L1030:
	mov	w7, w9
.L1033:
	add	w3, w3, 1
	add	x6, x6, 32
	and	w3, w3, 65535
	b	.L1029
.L1037:
	mov	w0, 65535
	cmp	w7, w0
	bne	.L1038
	ldrb	w0, [x20, 8]
	cbnz	w0, .L1039
.L1038:
	add	x0, x19, :lo12:.LANCHOR0
	and	w3, w23, 65535
	ldr	w1, [x0, 3716]
	cmn	w1, #1
	bne	.L1040
	str	w21, [x0, 3716]
.L1040:
	add	w0, w26, 7
	cmp	w0, w23, uxth
	bge	.L1088
	sub	w24, w3, #7
	and	w24, w24, 65535
.L1041:
	add	x25, x19, :lo12:.LANCHOR0
	mov	w4, 65535
.L1042:
	cmp	w24, w3
	bhi	.L1049
	ldrh	w0, [x25, 236]
	mov	x1, x28
	mov	w23, 0
	add	x0, x0, 8
	add	x0, x20, x0, lsl 1
	b	.L1050
.L1088:
	mov	w24, w26
	b	.L1041
.L1044:
	ldrh	w2, [x1]
	cmp	w2, w4
	beq	.L1043
	ldr	x5, [x25, 3432]
	ubfiz	x6, x23, 5, 16
	add	w23, w23, 1
	orr	w2, w24, w2, lsl 10
	add	x5, x5, x6
	and	w23, w23, 65535
	str	w2, [x5, 4]
.L1043:
	add	x1, x1, 2
.L1050:
	cmp	x0, x1
	bne	.L1044
	ldr	x0, [x25, 3432]
	mov	w1, w23
	str	w4, [x29, 120]
	mov	w2, 0
	str	w3, [x29, 128]
	ubfiz	x23, x23, 5, 16
	add	x23, x23, 16
	bl	FlashReadPages
	ldr	x1, [x25, 3432]
	ldr	w4, [x29, 120]
	ldr	w3, [x29, 128]
	add	x0, x1, 16
	add	x23, x1, x23
.L1045:
	cmp	x23, x0
	bne	.L1048
	add	w24, w24, 1
	and	w24, w24, 65535
	b	.L1042
.L1048:
	ldr	w1, [x0, -16]
	cbnz	w1, .L1039
	ldr	x1, [x0]
	ldrh	w2, [x1]
	cmp	w2, w4
	beq	.L1047
	ldr	w1, [x1, 4]
	cmn	w1, #1
	beq	.L1047
	str	w1, [x25, 3716]
.L1047:
	add	x0, x0, 32
	b	.L1045
.L1049:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, -1
	str	w1, [x0, 3716]
.L1039:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, 1
	mov	w24, w26
	strh	w0, [x19, 3752]
	add	x0, x19, 3760
	str	x0, [x29, 104]
.L1051:
	ldrh	w0, [x19, 236]
	mov	x1, x28
	mov	w23, 0
	mov	w4, 65535
	add	x0, x0, 8
	add	x0, x20, x0, lsl 1
.L1052:
	cmp	x0, x1
	bne	.L1054
	ldr	x0, [x19, 3432]
	mov	w1, w23
	mov	w2, 0
	bl	FlashReadPages
	ubfiz	x0, x23, 5, 16
	str	x0, [x29, 112]
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	add	x0, x0, 520
	mov	x23, 0
	str	x0, [x29, 120]
.L1055:
	ldr	x0, [x29, 112]
	cmp	x0, x23
	bne	.L1082
	add	w24, w24, 1
	ldrh	w0, [x19, 306]
	and	w24, w24, 65535
	cmp	w0, w24
	bne	.L1051
	ldrh	w1, [x19, 236]
	mov	w0, 0
	strh	w24, [x20, 2]
	mov	w2, 65535
	strh	wzr, [x20, 4]
.L1083:
	cmp	w0, w1
	beq	.L1138
	ldrh	w3, [x28], 2
	cmp	w3, w2
	beq	.L1084
	strb	w0, [x20, 6]
	b	.L1138
.L1054:
	ldrh	w2, [x1]
	cmp	w2, w4
	beq	.L1053
	ldr	x3, [x19, 3432]
	ubfiz	x5, x23, 5, 16
	add	w23, w23, 1
	orr	w2, w24, w2, lsl 10
	add	x3, x3, x5
	and	w23, w23, 65535
	str	w2, [x3, 4]
.L1053:
	add	x1, x1, 2
	b	.L1052
.L1082:
	ldr	x4, [x19, 3432]
	add	x4, x4, x23
	ldr	w5, [x4, 4]
	str	w5, [x29, 156]
	lsr	w0, w5, 10
	bl	P2V_plane
	and	w0, w0, 65535
	cmp	w24, w26
	bcc	.L1056
	ldr	w1, [x29, 140]
	ccmp	w1, w0, 0, eq
	bhi	.L1056
	cmp	w24, w22
	ccmp	w27, w0, 0, eq
	beq	.L1057
	ldr	w0, [x4]
	cmn	w0, #1
	beq	.L1058
	ldr	x3, [x4, 16]
	mov	w0, 61589
	ldrh	w1, [x3]
	cmp	w1, w0
	beq	.L1059
	ldrh	w0, [x20]
.L1142:
	bl	decrement_vpc_count
	b	.L1056
.L1059:
	ldr	w21, [x3, 4]
	cmn	w21, #1
	beq	.L1060
	ldr	w1, [x19, 2664]
	mov	w0, w21
	str	x3, [x29, 128]
	bl	ftl_cmp_data_ver
	ldr	x3, [x29, 128]
	cbz	w0, .L1060
	add	w0, w21, 1
	str	w0, [x19, 2664]
.L1060:
	ldrh	w1, [x3]
	mov	w0, 61589
	cmp	w1, w0
	beq	.L1061
	ldr	x1, [x29, 120]
	mov	w2, 1954
	str	x3, [x29, 128]
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	ldr	x3, [x29, 128]
.L1061:
	ldp	w25, w0, [x3, 8]
	add	x1, x29, 152
	str	w0, [x29, 148]
	mov	w2, 0
	mov	w0, w25
	bl	log2phys
	ldr	w1, [x19, 3716]
	ldr	w3, [x29, 148]
	cmn	w1, #1
	beq	.L1062
	mov	w0, w21
	bl	ftl_cmp_data_ver
	cbz	w0, .L1062
	cmn	w3, #1
	beq	.L1063
	ldr	x0, [x19, 3432]
	mov	w2, 0
	mov	w1, 1
	add	x0, x0, x23
	ldr	x4, [x0, 16]
	str	w3, [x0, 4]
	str	x4, [x29, 128]
	ldr	x0, [x19, 3432]
	add	x0, x0, x23
	bl	FlashReadPages
	ldr	x0, [x19, 3432]
	ldr	x4, [x29, 128]
	add	x3, x0, x23
	ldr	w0, [x0, x23]
	cmn	w0, #1
	bne	.L1064
.L1065:
	mov	w0, -1
	str	w0, [x29, 148]
.L1072:
	ldr	w4, [x29, 148]
	cmn	w4, #1
	beq	.L1056
.L1087:
	lsr	w0, w4, 10
	bl	P2V_block_in_plane
	and	w25, w0, 65535
	ldrh	w0, [x19, 244]
	mov	w3, w25
	cmp	w0, w25
	bhi	.L1078
	ldr	x1, [x29, 120]
	mov	w2, 2059
	adrp	x0, .LC8
	str	w25, [x29, 128]
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	ldr	w3, [x29, 128]
.L1078:
	ldr	x1, [x19, 72]
	ubfiz	x0, x25, 1, 16
	ldrh	w0, [x1, x0]
	cbz	w0, .L1079
	mov	w0, w3
	b	.L1142
.L1063:
	ldp	w1, w0, [x29, 152]
	cmp	w1, w0
	bne	.L1056
	mov	w2, 1
	add	x1, x29, 148
	mov	w0, w25
	bl	log2phys
.L1056:
	add	x23, x23, 32
	b	.L1055
.L1064:
	ldr	w0, [x4, 8]
	cmp	w25, w0
	bne	.L1065
	ldr	w0, [x4, 4]
	str	w0, [x29, 128]
	str	x4, [x29, 96]
	uxtw	x1, w0
	ldr	w0, [x19, 3716]
	bl	ftl_cmp_data_ver
	cbz	w0, .L1065
	ldp	w0, w1, [x29, 152]
	ldr	x4, [x29, 96]
	cmp	w0, w1
	ldr	w1, [x29, 148]
	bne	.L1067
.L1141:
	mov	w0, w25
	bl	FtlReUsePrevPpa
	b	.L1065
.L1067:
	cmp	w0, w1
	beq	.L1065
	cmn	w0, #1
	beq	.L1068
	ldr	x4, [x3, 16]
	mov	w2, 0
	str	w0, [x3, 4]
	mov	w1, 1
	str	x4, [x29, 96]
	ldr	x0, [x19, 3432]
	add	x0, x0, x23
	bl	FlashReadPages
	ldr	x4, [x29, 96]
.L1069:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldr	x1, [x0, 3432]
	ldr	w1, [x1, x23]
	cmn	w1, #1
	beq	.L1070
	ldr	w3, [x4, 4]
	ldr	w0, [x0, 3716]
	mov	w1, w3
	bl	ftl_cmp_data_ver
	cbz	w0, .L1070
	ldr	w0, [x29, 128]
	mov	w1, w3
	bl	ftl_cmp_data_ver
	cbz	w0, .L1065
.L1070:
	ldr	w1, [x29, 148]
	b	.L1141
.L1068:
	str	w0, [x3]
	b	.L1069
.L1062:
	ldp	w1, w0, [x29, 152]
	cmp	w1, w0
	beq	.L1072
	cmn	w3, #1
	beq	.L1074
	ldr	w0, [x19, 252]
	cmp	w0, w3, lsr 10
	bhi	.L1074
	adrp	x0, .LC109
	mov	w1, w3
	add	x0, x0, :lo12:.LC109
.L1144:
	bl	sftl_printk
	b	.L1056
.L1074:
	mov	w2, 1
	add	x1, x29, 156
	mov	w0, w25
	bl	log2phys
	ldr	w4, [x29, 152]
	cmn	w4, #1
	beq	.L1072
	ldr	w0, [x29, 148]
	cmp	w4, w0
	beq	.L1087
	lsr	w0, w4, 10
	bl	P2V_block_in_plane
	ldrh	w1, [x19, 24]
	and	w0, w0, 65535
	cmp	w1, w0
	beq	.L1077
	ldrh	w1, [x19, 80]
	cmp	w1, w0
	beq	.L1077
	ldrh	w1, [x19, 128]
	cmp	w1, w0
	bne	.L1072
.L1077:
	ldr	x0, [x19, 3432]
	mov	w2, 0
	mov	w1, 1
	str	w4, [x0, 4]
	ldr	x3, [x0, 16]
	ldr	x0, [x19, 3432]
	str	x3, [x29, 128]
	bl	FlashReadPages
	ldr	x0, [x19, 3432]
	ldr	w0, [x0]
	cmn	w0, #1
	beq	.L1072
	ldr	x3, [x29, 128]
	mov	w0, w21
	ldr	w1, [x3, 4]
	bl	ftl_cmp_data_ver
	cbnz	w0, .L1072
	mov	w2, 1
	add	x1, x29, 152
	mov	w0, w25
	bl	log2phys
	b	.L1072
.L1079:
	adrp	x0, .LC110
	mov	w1, w25
	add	x0, x0, :lo12:.LC110
	b	.L1144
.L1058:
	ldr	w0, [x19, 3756]
	cmp	w0, 31
	bhi	.L1080
	ldr	x1, [x29, 104]
	str	w5, [x1, w0, uxtw 2]
	add	w0, w0, 1
	str	w0, [x19, 3756]
.L1080:
	ldrh	w0, [x20]
	bl	decrement_vpc_count
	ldr	w0, [x19, 3716]
	cmn	w0, #1
	bne	.L1081
.L1143:
	str	w21, [x19, 3716]
	b	.L1056
.L1081:
	cmp	w21, w0
	bcs	.L1056
	b	.L1143
.L1084:
	add	w0, w0, 1
	and	w0, w0, 65535
	b	.L1083
.L1057:
	strb	w27, [x20, 6]
	strh	w22, [x20, 2]
	b	.L1146
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	2
	.global	ftl_check_vpc
	.type	ftl_check_vpc, %function
ftl_check_vpc:
	stp	x29, x30, [sp, -128]!
	adrp	x0, .LC111
	add	x0, x0, :lo12:.LC111
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	stp	x23, x24, [sp, 48]
	add	x24, x20, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	mov	w23, 0
	adrp	x21, .LANCHOR1
	add	x1, x21, :lo12:.LANCHOR1
	add	x1, x1, 544
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
	adrp	x22, check_vpc_table
	bl	sftl_printk
	add	x19, x22, :lo12:check_vpc_table
	mov	w2, 8192
	mov	w1, 0
	mov	x0, x19
	bl	ftl_memset
.L1148:
	ldr	w0, [x24, 2624]
	cmp	w23, w0
	bcc	.L1150
	adrp	x25, .LC112
	add	x23, x20, :lo12:.LANCHOR0
	add	x26, x22, :lo12:check_vpc_table
	add	x25, x25, :lo12:.LC112
	mov	w24, 0
	mov	w19, 0
	mov	w4, 65535
.L1151:
	ldrh	w0, [x23, 244]
	cmp	w0, w19
	bhi	.L1153
	ldr	x19, [x23, 2592]
	cbz	x19, .L1154
	ldr	x0, [x23, 2560]
	adrp	x25, .LC113
	add	x20, x20, :lo12:.LANCHOR0
	add	x22, x22, :lo12:check_vpc_table
	sub	x19, x19, x0
	mov	x0, -6148914691236517206
	asr	x19, x19, 1
	movk	x0, 0xaaab, lsl 0
	add	x25, x25, :lo12:.LC113
	ldrh	w26, [x23, 228]
	mov	w27, 6
	mov	w23, 0
	mul	x19, x19, x0
	and	w19, w19, 65535
.L1155:
	cmp	w23, w26
	bne	.L1157
.L1154:
	cbz	w24, .L1147
	add	x1, x21, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2387
	add	x1, x1, 544
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1147:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 128
	ret
.L1150:
	mov	w2, 0
	add	x1, x29, 124
	mov	w0, w23
	bl	log2phys
	ldr	w0, [x29, 124]
	cmn	w0, #1
	beq	.L1149
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	and	x0, x0, 65535
	ldrh	w1, [x19, x0, lsl 1]
	add	w1, w1, 1
	strh	w1, [x19, x0, lsl 1]
.L1149:
	add	w23, w23, 1
	b	.L1148
.L1153:
	ldr	x0, [x23, 72]
	ubfiz	x28, x19, 1, 16
	sxtw	x27, w19
	ldrh	w2, [x0, x28]
	ldrh	w3, [x26, x27, lsl 1]
	cmp	w2, w3
	beq	.L1152
	mov	x0, x25
	str	w4, [x29, 108]
	mov	w1, w19
	bl	sftl_printk
	ldr	x0, [x23, 72]
	ldr	w4, [x29, 108]
	ldrh	w0, [x0, x28]
	cmp	w0, w4
	beq	.L1152
	ldrh	w1, [x26, x27, lsl 1]
	cmp	w1, w0
	csinc	w24, w24, wzr, ls
.L1152:
	add	w19, w19, 1
	and	w19, w19, 65535
	b	.L1151
.L1157:
	ldr	x1, [x20, 72]
	ubfiz	x0, x19, 1, 16
	ldrh	w2, [x1, x0]
	cbz	w2, .L1156
	ldrh	w3, [x22, w19, sxtw 1]
	mov	w24, 1
	mov	w1, w19
	mov	x0, x25
	bl	sftl_printk
.L1156:
	ldr	x0, [x20, 2560]
	umull	x19, w19, w27
	ldrh	w19, [x0, x19]
	mov	w0, 65535
	cmp	w19, w0
	beq	.L1154
	add	w23, w23, 1
	and	w23, w23, 65535
	b	.L1155
	.size	ftl_check_vpc, .-ftl_check_vpc
	.align	2
	.global	ftl_scan_all_data
	.type	ftl_scan_all_data, %function
ftl_scan_all_data:
	sub	sp, sp, #96
	adrp	x0, .LC114
	mov	w1, 0
	add	x0, x0, :lo12:.LC114
	stp	x29, x30, [sp, 16]
	add	x29, sp, 16
	stp	x21, x22, [sp, 48]
	adrp	x22, .LANCHOR0
	add	x21, x22, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 32]
	str	x23, [sp, 64]
	mov	w20, 0
	add	x19, x21, 3720
	bl	sftl_printk
.L1173:
	add	x0, x22, :lo12:.LANCHOR0
	ldr	w0, [x0, 2624]
	cmp	w20, w0
	bcc	.L1179
	ldp	x19, x20, [sp, 32]
	ldp	x21, x22, [sp, 48]
	ldp	x29, x30, [sp, 16]
	ldr	x23, [sp, 64]
	add	sp, sp, 96
	ret
.L1179:
	mov	w2, 0
	add	x1, x29, 76
	mov	w0, w20
	bl	log2phys
	tst	x20, 2047
	bne	.L1174
	ldr	w2, [x29, 76]
	adrp	x0, .LC115
	mov	w1, w20
	add	x0, x0, :lo12:.LC115
	bl	sftl_printk
.L1174:
	ldr	w0, [x29, 76]
	cmn	w0, #1
	beq	.L1176
	ldr	x23, [x21, 3520]
	mov	w2, 0
	stp	wzr, w0, [x19]
	mov	w1, 1
	ldr	x0, [x21, 3456]
	str	x0, [x19, 8]
	mov	x0, x19
	str	x23, [x19, 16]
	str	w20, [x19, 24]
	bl	FlashReadPages
	ldr	w0, [x19]
	cmp	w0, 256
	ccmn	w0, #1, 4, ne
	beq	.L1177
	ldr	w0, [x23, 8]
	cmp	w20, w0
	beq	.L1176
.L1177:
	ldp	x1, x0, [x19, 8]
	ldr	w2, [x1, 4]
	str	w2, [sp]
	ldp	w3, w4, [x0]
	ldp	w5, w6, [x0, 8]
	adrp	x0, .LC116
	ldr	w7, [x1]
	add	x0, x0, :lo12:.LC116
	ldr	w2, [x19, 4]
	mov	w1, w20
	bl	sftl_printk
.L1176:
	add	w20, w20, 1
	b	.L1173
	.size	ftl_scan_all_data, .-ftl_scan_all_data
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	stp	x29, x30, [sp, -160]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x20, x0
	stp	x21, x22, [sp, 32]
	adrp	x22, .LANCHOR2
	add	x0, x22, :lo12:.LANCHOR2
	stp	x25, x26, [sp, 64]
	stp	x23, x24, [sp, 48]
	mov	w25, w1
	stp	x27, x28, [sp, 80]
	ldrh	w28, [x0, 4]
	mov	w0, 65535
	cmp	w28, w0
	beq	.L1205
	cbnz	w28, .L1182
.L1183:
	bl	FtlGcPageVarInit
	b	.L1184
.L1205:
	mov	w28, 0
.L1182:
	adrp	x0, .LANCHOR0+306
	ldrh	w0, [x0, #:lo12:.LANCHOR0+306]
	cmp	w0, w25
	beq	.L1183
.L1184:
	adrp	x26, .LANCHOR0
	add	x19, x26, :lo12:.LANCHOR0
	mov	w24, 0
.L1199:
	ldrh	w1, [x20]
	mov	w0, 65535
	strb	wzr, [x20, 8]
	cmp	w1, w0
	beq	.L1185
.L1202:
	add	x0, x26, :lo12:.LANCHOR0
	add	x2, x20, 16
	mov	w21, 0
	mov	w10, 65535
	mov	w6, 4
	ldrh	w8, [x0, 314]
	ldrh	w9, [x0, 316]
	ldrh	w0, [x0, 236]
	add	x0, x0, 8
	add	x0, x20, x0, lsl 1
.L1186:
	cmp	x0, x2
	bne	.L1188
	ldr	x0, [x19, 3432]
	mov	w2, 0
	mov	w1, w21
	mov	x23, 0
	bl	FlashReadPages
	ubfiz	x0, x21, 5, 16
	str	x0, [x29, 104]
.L1189:
	ldr	x0, [x29, 104]
	cmp	x0, x23
	bne	.L1200
	add	w3, w28, 1
	add	w24, w24, 1
	and	w28, w3, 65535
	cmp	w25, w24
	bls	.L1201
.L1203:
	ldrh	w0, [x19, 306]
	cmp	w0, w28
	bhi	.L1202
.L1185:
	add	x22, x22, :lo12:.LANCHOR2
	strh	w28, [x20, 2]
	strb	wzr, [x20, 6]
	mov	w0, -1
	mov	w2, 0
	mov	w1, w28
	strh	w0, [x22, 4]
	mov	x0, x20
	bl	ftl_sb_update_avl_pages
	b	.L1204
.L1188:
	ldrh	w1, [x2]
	cmp	w1, w10
	beq	.L1187
	ldr	x5, [x19, 3432]
	ubfiz	x7, x21, 5, 16
	orr	w1, w28, w1, lsl 10
	add	x5, x5, x7
	str	w1, [x5, 4]
	mul	w1, w21, w8
	ldr	x4, [x19, 3432]
	ldr	x5, [x19, 3256]
	sdiv	w1, w1, w6
	add	x4, x4, x7
	add	x1, x5, x1, sxtw 2
	str	x1, [x4, 8]
	mul	w1, w21, w9
	ldr	x5, [x19, 3264]
	add	w21, w21, 1
	and	w21, w21, 65535
	sdiv	w1, w1, w6
	add	x1, x5, x1, sxtw 2
	str	x1, [x4, 16]
.L1187:
	add	x2, x2, 2
	b	.L1186
.L1200:
	ldr	x0, [x19, 3432]
	add	x1, x0, x23
	ldr	w0, [x0, x23]
	ldr	w27, [x1, 4]
	ldr	x21, [x1, 16]
	cbnz	w0, .L1190
	ldrh	w0, [x21]
	mov	w1, 65535
	cmp	w0, w1
	bne	.L1191
.L1215:
	ldrh	w1, [x20]
	mov	w28, 0
	ldr	x0, [x19, 72]
	strh	wzr, [x0, x1, lsl 1]
	ldrh	w0, [x20]
	bl	INSERT_FREE_LIST
	mov	w0, -1
	strh	w0, [x20]
	strh	w0, [x19, 176]
	bl	FtlGcPageVarInit
	b	.L1199
.L1191:
	ldr	w1, [x19, 2624]
	ldr	w0, [x21, 8]
	cmp	w0, w1
	bhi	.L1215
	add	x1, x29, 124
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x21, 12]
	ldr	w1, [x29, 124]
	cmp	w0, w1
	beq	.L1194
.L1196:
	ldr	w2, [x21, 8]
.L1216:
	ldr	w0, [x21, 12]
	mov	w1, w27
	add	x23, x23, 32
	bl	FtlGcUpdatePage
	b	.L1189
.L1194:
	cmn	w0, #1
	beq	.L1196
	str	w0, [x29, 132]
	mov	w2, 0
	ldr	x0, [x19, 3488]
	mov	w1, 1
	str	x0, [x29, 136]
	ldr	x0, [x19, 3528]
	str	x0, [x29, 144]
	add	x0, x29, 128
	bl	FlashReadPages
	ldrh	w1, [x19, 262]
	mov	x0, 0
	ldr	x2, [x19, 3432]
	ldr	x5, [x29, 136]
	ubfiz	x1, x1, 9, 16
	add	x2, x2, x23
.L1197:
	cmp	x0, x1
	beq	.L1196
	ldr	x6, [x2, 8]
	ldr	w7, [x6, x0]
	add	x0, x0, 4
	add	x6, x5, x0
	ldr	w6, [x6, -4]
	cmp	w7, w6
	beq	.L1197
	ldrh	w1, [x20]
	adrp	x0, .LC117
	ldr	w2, [x29, 132]
	add	x0, x0, :lo12:.LC117
	bl	sftl_printk
	b	.L1215
.L1190:
	mov	w2, -1
	b	.L1216
.L1201:
	add	x1, x22, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w0, [x1, 4]
	cmp	w0, w2
	beq	.L1203
	add	w0, w0, w24
	strh	w0, [x1, 4]
	ldrh	w0, [x19, 306]
	cmp	w0, w28
	bls	.L1203
.L1204:
	ldp	x19, x20, [sp, 16]
	mov	w0, -1
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 160
	ret
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	adrp	x1, .LANCHOR0
	add	x0, x1, :lo12:.LANCHOR0
	add	x2, x0, 2712
	ldr	w3, [x2, 80]
	cbz	w3, .L1218
	ldr	w1, [x0, 2624]
	ldr	w3, [x2, 84]
	cmp	w3, w1
	bcs	.L1219
	stp	x29, x30, [sp, -160]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	mov	w20, 2048
	str	x21, [sp, 32]
.L1224:
	add	x21, x19, 2712
	ldr	w1, [x19, 2624]
	ldr	w0, [x21, 84]
	cmp	w0, w1
	bcc	.L1220
.L1223:
	ldp	x19, x20, [sp, 16]
	mov	w0, -1
	ldr	x21, [sp, 32]
	ldp	x29, x30, [sp], 160
	ret
.L1220:
	add	x1, x29, 60
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x21, 84]
	ldr	w1, [x29, 60]
	add	w0, w0, 1
	str	w0, [x21, 84]
	cmn	w1, #1
	beq	.L1222
	str	w0, [x29, 88]
	mov	w2, 0
	ldr	x0, [x19, 3504]
	str	x0, [x29, 72]
	add	x0, x29, 96
	str	x0, [x29, 80]
	add	x0, x29, 160
	str	w1, [x29, 68]
	mov	w1, 1
	str	wzr, [x0, -96]!
	bl	FlashReadPages
	ldr	w0, [x29, 64]
	cmp	w0, 256
	bne	.L1223
	ldr	w0, [x29, 60]
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L1223
.L1222:
	subs	w20, w20, #1
	bne	.L1224
	b	.L1223
.L1219:
	ldr	w0, [x0, 2628]
	stp	w0, wzr, [x2, 76]
	str	wzr, [x2, 84]
.L1231:
	mov	w0, 0
	ret
.L1218:
	ldr	w4, [x0, 2684]
	mov	w3, 10000
	ldr	w5, [x0, 2628]
	mov	w6, 31
	cmp	w4, w3
	ldr	w7, [x2, 76]
	mov	w3, 63
	csel	w6, w6, w3, hi
	add	w3, w5, 1048576
	cmp	w7, w3
	bhi	.L1228
	ldr	w3, [x0, 2624]
	mov	w8, 1000
	lsr	w4, w4, 10
	add	w4, w4, 1
	mul	w3, w3, w8
	udiv	w3, w3, w4
	add	w3, w3, w7
	cmp	w5, w3
	bhi	.L1228
	ldrh	w0, [x0, 2540]
	tst	w6, w0
	bne	.L1231
	ldr	w2, [x2, 100]
	cmp	w0, w2
	beq	.L1231
.L1228:
	add	x0, x1, :lo12:.LANCHOR0
	add	x1, x0, 2712
	ldrh	w0, [x0, 2540]
	str	w0, [x1, 100]
	str	w5, [x1, 76]
	mov	w0, 1
	str	wzr, [x1, 84]
	str	w0, [x1, 80]
	b	.L1231
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	l2p_flush
	.type	l2p_flush, %function
l2p_flush:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	add	x20, x20, :lo12:.LANCHOR0
	mov	w19, 0
	bl	FtlWriteDump_data
.L1238:
	ldrh	w0, [x20, 342]
	cmp	w0, w19
	bhi	.L1240
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L1240:
	ldr	x1, [x20, 2608]
	ubfiz	x0, x19, 4, 16
	add	x0, x1, x0
	ldr	w0, [x0, 4]
	tbz	w0, #31, .L1239
	mov	w0, w19
	bl	flush_l2p_region
.L1239:
	add	w19, w19, 1
	and	w19, w19, 65535
	b	.L1238
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	stp	x29, x30, [sp, -208]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x27, x28, [sp, 80]
	mov	w22, w1
	mov	w28, w0
	add	w1, w0, w1
	stp	x23, x24, [sp, 48]
	ldrh	w0, [x19, 300]
	stp	x25, x26, [sp, 64]
	cmp	w1, w0
	bhi	.L1250
	ldrh	w23, [x19, 312]
	mov	x25, x2
	add	x26, x29, 144
	add	x27, x19, 3888
	mov	w24, 0
	lsr	w23, w28, w23
.L1244:
	cbnz	w22, .L1249
.L1242:
	mov	w0, w24
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 208
	ret
.L1249:
	ldrh	w1, [x19, 262]
	ldr	x0, [x19, 3600]
	udiv	w21, w28, w1
	ldr	w2, [x0, w23, uxtw 2]
	and	w0, w22, 65535
	msub	w21, w21, w1, w28
	sub	w20, w1, w21
	and	w20, w20, 65535
	cmp	w22, w20
	csel	w20, w0, w20, cc
	cbz	w2, .L1246
	cmp	w20, w1
	beq	.L1246
	ldr	x0, [x19, 3472]
	str	w2, [x29, 116]
	mov	w2, 1
	stp	x0, x26, [x29, 120]
	mov	w1, w2
	add	x0, x29, 112
	bl	FlashReadPages
.L1247:
	lsl	w3, w20, 9
	ldr	x0, [x19, 3472]
	lsl	w21, w21, 9
	mov	w2, w3
	asr	w21, w21, 2
	mov	x1, x25
	str	w3, [x29, 108]
	add	x0, x0, x21, sxtw 2
	bl	ftl_memcpy
	sub	w22, w22, w20
	ldr	x2, [x19, 3472]
	mov	w1, w23
	mov	x0, x27
	add	w28, w28, w20
	add	w23, w23, 1
	bl	FtlMapWritePage
	cmn	w0, #1
	ldr	w3, [x29, 108]
	csinv	w24, w24, wzr, ne
	add	x25, x25, x3, sxtw
	b	.L1244
.L1246:
	ldrh	w2, [x19, 314]
	mov	w1, 0
	ldr	x0, [x19, 3472]
	bl	ftl_memset
	b	.L1247
.L1250:
	mov	w24, -1
	b	.L1242
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	2
	.global	Ftl_save_ext_data
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	mov	w1, 19539
	movk	w1, 0x4654, lsl 16
	ldr	w3, [x0, 2712]
	cmp	w3, w1
	bne	.L1258
	stp	x29, x30, [sp, -16]!
	add	x2, x0, 2712
	mov	w1, 85
	add	x29, sp, 0
	movk	w1, 0x5000, lsl 16
	str	w1, [x2, 4]
	ldr	w1, [x0, 2652]
	str	w1, [x2, 88]
	ldr	w1, [x0, 2656]
	str	w1, [x2, 92]
	ldr	w1, [x0, 2648]
	str	w1, [x2, 8]
	ldr	w1, [x0, 2636]
	str	w1, [x2, 12]
	ldr	w1, [x0, 2628]
	str	w1, [x2, 16]
	ldr	w1, [x0, 2644]
	str	w1, [x2, 20]
	ldr	w1, [x0, 2672]
	str	w1, [x2, 28]
	ldr	w1, [x0, 2680]
	str	w1, [x2, 32]
	ldr	w1, [x0, 2632]
	str	w1, [x2, 36]
	ldr	w1, [x0, 2640]
	str	w1, [x2, 40]
	ldr	w1, [x0, 2684]
	ldr	w0, [x0, 2688]
	stp	w1, w0, [x2, 44]
	mov	w1, 1
	mov	w0, 0
	bl	FtlVendorPartWrite
	ldp	x29, x30, [sp], 16
	ret
.L1258:
	ret
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	adrp	x1, .LANCHOR0
	add	x3, x1, :lo12:.LANCHOR0
	ldrh	w2, [x3, 3952]
	cmp	w2, 31
	bhi	.L1265
	add	w2, w2, 1
	strh	w2, [x3, 3952]
	mov	w2, 1
.L1262:
	cbnz	w0, .L1263
	add	x0, x1, :lo12:.LANCHOR0
	ldr	x0, [x0, 3552]
	ldr	w3, [x0, 20]
	ldr	w0, [x0, 16]
	add	w2, w2, w3
	cmp	w0, w2
	bcc	.L1267
.L1263:
	add	x0, x1, :lo12:.LANCHOR0
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	x1, [x0, 3552]
	ldr	w2, [x1, 16]
	str	w2, [x1, 20]
	mov	w2, 17221
	movk	w2, 0x4254, lsl 16
	str	w2, [x1]
	ldr	x2, [x0, 3552]
	ldrh	w1, [x0, 3536]
	lsl	w3, w1, 9
	str	wzr, [x2, 4]
	str	w3, [x2, 12]
	ldr	w3, [x2, 8]
	add	w3, w3, 1
	str	w3, [x2, 8]
	ldr	x2, [x0, 3552]
	mov	w0, 64
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
.L1265:
	mov	w2, 32
	b	.L1262
.L1267:
	mov	w0, 0
	ret
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	2
	.global	sftl_vendor_write
	.type	sftl_vendor_write, %function
sftl_vendor_write:
	stp	x29, x30, [sp, -16]!
	add	w0, w0, 256
	add	x29, sp, 0
	bl	FtlVendorPartWrite
	ldp	x29, x30, [sp], 16
	ret
	.size	sftl_vendor_write, .-sftl_vendor_write
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	stp	x29, x30, [sp, -208]!
	adrp	x3, .LANCHOR0
	add	x3, x3, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	mov	w22, w1
	stp	x23, x24, [sp, 48]
	add	w1, w0, w1
	stp	x19, x20, [sp, 16]
	mov	w23, w0
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
	ldrh	w0, [x3, 300]
	cmp	w1, w0
	bhi	.L1280
	ldrh	w21, [x3, 312]
	adrp	x26, .LC118
	mov	x25, x2
	mov	x28, x3
	add	x26, x26, :lo12:.LC118
	mov	w24, 0
	lsr	w21, w23, w21
.L1273:
	cbnz	w22, .L1279
.L1271:
	mov	w0, w24
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 208
	ret
.L1279:
	ldrh	w19, [x28, 262]
	ldr	x0, [x28, 3600]
	udiv	w20, w23, w19
	ldr	w4, [x0, w21, uxtw 2]
	and	w0, w22, 65535
	msub	w20, w20, w19, w23
	sub	w19, w19, w20
	and	w19, w19, 65535
	cmp	w22, w19
	csel	w19, w0, w19, cc
	lsl	w27, w19, 9
	cbz	w4, .L1275
	ldr	x0, [x28, 3472]
	mov	w2, 1
	str	x0, [x29, 120]
	add	x0, x29, 144
	str	w4, [x29, 108]
	mov	w1, w2
	str	w4, [x29, 116]
	str	x0, [x29, 128]
	add	x0, x29, 112
	bl	FlashReadPages
	ldp	w4, w0, [x29, 108]
	cmn	w0, #1
	ldr	w0, [x28, 3720]
	csinv	w24, w24, wzr, ne
	cmp	w0, 256
	bne	.L1277
	mov	w2, w4
	mov	w1, w21
	mov	x0, x26
	bl	sftl_printk
	ldr	x2, [x28, 3472]
	mov	w1, w21
	add	x0, x28, 3888
	bl	FtlMapWritePage
.L1277:
	ldr	x1, [x28, 3472]
	lsl	w20, w20, 9
	asr	w20, w20, 2
	mov	w2, w27
	mov	x0, x25
	add	x1, x1, x20, sxtw 2
	bl	ftl_memcpy
.L1278:
	add	w21, w21, 1
	sub	w22, w22, w19
	add	w23, w23, w19
	add	x25, x25, x27, sxtw
	b	.L1273
.L1275:
	mov	w2, w27
	mov	w1, 0
	mov	x0, x25
	bl	ftl_memset
	b	.L1278
.L1280:
	mov	w24, -1
	b	.L1271
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	2
	.global	FtlLoadEctTbl
	.type	FtlLoadEctTbl, %function
FtlLoadEctTbl:
	stp	x29, x30, [sp, -32]!
	mov	w0, 64
	add	x29, sp, 0
	str	x19, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	ldrh	w1, [x19, 3536]
	ldr	x2, [x19, 3552]
	bl	FtlVendorPartRead
	ldr	x0, [x19, 3552]
	ldr	w1, [x0]
	mov	w0, 17221
	movk	w0, 0x4254, lsl 16
	cmp	w1, w0
	beq	.L1283
	adrp	x1, .LC119
	adrp	x0, .LC77
	add	x1, x1, :lo12:.LC119
	add	x0, x0, :lo12:.LC77
	bl	sftl_printk
	ldr	x0, [x19, 3552]
	mov	w1, 0
	ldrh	w2, [x19, 3536]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1283:
	mov	w0, 0
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	Ftl_load_ext_data
	.type	Ftl_load_ext_data, %function
Ftl_load_ext_data:
	stp	x29, x30, [sp, -48]!
	mov	w1, 1
	mov	w0, 0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x20, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	add	x22, x20, 2712
	mov	w21, 19539
	mov	x2, x22
	bl	FtlVendorPartRead
	ldr	w0, [x20, 2712]
	movk	w21, 0x4654, lsl 16
	cmp	w0, w21
	beq	.L1286
	mov	w2, 512
	mov	w1, 0
	mov	x0, x22
	bl	ftl_memset
	str	w21, [x20, 2712]
.L1286:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 19539
	movk	w2, 0x4654, lsl 16
	ldr	w3, [x0, 2712]
	cmp	w3, w2
	bne	.L1287
	ldr	w2, [x0, 2800]
	str	w2, [x0, 2652]
	ldr	w2, [x0, 2804]
	str	w2, [x0, 2656]
	ldr	w2, [x0, 2720]
	str	w2, [x0, 2648]
	ldr	w2, [x0, 2724]
	str	w2, [x0, 2636]
	ldr	w2, [x0, 2728]
	str	w2, [x0, 2628]
	ldr	w2, [x0, 2732]
	str	w2, [x0, 2644]
	ldr	w2, [x0, 2740]
	str	w2, [x0, 2672]
	ldr	w2, [x0, 2744]
	str	w2, [x0, 2680]
	ldr	w2, [x0, 2748]
	str	w2, [x0, 2632]
	ldr	w2, [x0, 2752]
	ldr	w1, [x0, 2760]
	str	w2, [x0, 2640]
	ldr	w2, [x0, 2756]
	str	w1, [x0, 2688]
	str	w2, [x0, 2684]
.L1287:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w2, [x0, 2672]
	ldrh	w1, [x0, 296]
	ldr	w3, [x0, 2668]
	madd	w1, w1, w3, w2
	ldrh	w2, [x0, 244]
	udiv	w1, w1, w2
	str	w1, [x0, 2676]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	2
	.global	sftl_vendor_read
	.type	sftl_vendor_read, %function
sftl_vendor_read:
	stp	x29, x30, [sp, -16]!
	add	w0, w0, 256
	add	x29, sp, 0
	bl	FtlVendorPartRead
	ldp	x29, x30, [sp], 16
	ret
	.size	sftl_vendor_read, .-sftl_vendor_read
	.align	2
	.global	FtlMapBlkWriteDump_data
	.type	FtlMapBlkWriteDump_data, %function
FtlMapBlkWriteDump_data:
	ldr	w1, [x0, 56]
	cbz	w1, .L1299
	stp	x29, x30, [sp, -48]!
	adrp	x1, .LANCHOR0
	add	x2, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	str	x21, [sp, 32]
	add	x0, x2, 3720
	mov	x21, x1
	ldrh	w20, [x19, 6]
	ldr	x4, [x2, 3520]
	sub	w20, w20, #1
	ldr	x3, [x2, 3464]
	and	w20, w20, 65535
	ldr	x5, [x19, 40]
	stp	x3, x4, [x0, 8]
	ubfiz	x4, x20, 2, 16
	str	wzr, [x19, 56]
	ldr	w4, [x5, x4]
	str	w4, [x0, 4]
	cbz	w4, .L1293
	mov	w2, 1
	mov	w1, w2
	bl	FlashReadPages
.L1294:
	add	x1, x21, :lo12:.LANCHOR0
	mov	x0, x19
	ldr	x2, [x1, 3728]
	mov	w1, w20
	bl	FtlMapWritePage
	ldr	x21, [sp, 32]
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
.L1293:
	ldrh	w2, [x2, 314]
	mov	w1, 255
	mov	x0, x3
	bl	ftl_memset
	b	.L1294
.L1299:
	ret
	.size	FtlMapBlkWriteDump_data, .-FtlMapBlkWriteDump_data
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	stp	x21, x22, [sp, 32]
	adrp	x22, .LANCHOR0
	add	x19, x22, :lo12:.LANCHOR0
	stp	x25, x26, [sp, 64]
	stp	x23, x24, [sp, 48]
	add	x20, x19, 2512
	add	x23, x19, 2696
	mov	w21, 0
	ldr	x26, [x19, 3520]
	mov	w24, 65535
	ldr	x0, [x19, 3456]
	str	x0, [x19, 3728]
	ldrh	w0, [x19, 2696]
	str	x26, [x19, 3736]
	strh	w0, [x26, 2]
	mov	w0, -3932
	strh	w0, [x26]
	str	wzr, [x26, 12]
	ldr	w0, [x19, 2704]
	stp	w0, wzr, [x26, 4]
	mov	w0, 19539
	ldrh	w1, [x19, 24]
	movk	w0, 0x4654, lsl 16
	str	w0, [x19, 2512]
	mov	w0, 85
	ldrb	w2, [x19, 30]
	strh	w1, [x19, 2526]
	movk	w0, 0x5000, lsl 16
	ldrh	w1, [x19, 26]
	str	w0, [x19, 2516]
	ldrh	w0, [x19, 2702]
	strh	w0, [x19, 2520]
	orr	w1, w2, w1, lsl 6
	ldrh	w0, [x19, 258]
	strb	w0, [x19, 2522]
	strh	w1, [x19, 2528]
	ldrh	w1, [x19, 80]
	ldrb	w2, [x19, 86]
	strh	w1, [x19, 2530]
	ldrh	w1, [x19, 82]
	ldrb	w0, [x19, 32]
	strb	w0, [x19, 2523]
	orr	w1, w2, w1, lsl 6
	ldrb	w0, [x19, 88]
	strh	w1, [x19, 2532]
	strb	w0, [x19, 2524]
	ldrh	w1, [x19, 128]
	ldrb	w0, [x19, 136]
	ldrb	w2, [x19, 134]
	strh	w1, [x19, 2534]
	strb	w0, [x19, 2525]
	ldrh	w1, [x19, 130]
	ldr	w0, [x19, 2668]
	str	w0, [x19, 2544]
	ldr	w0, [x19, 2660]
	orr	w1, w2, w1, lsl 6
	str	w0, [x19, 2552]
	ldr	w0, [x19, 2664]
	ldrh	w2, [x19, 314]
	str	w0, [x19, 2548]
	ldr	x0, [x19, 3728]
	strh	w1, [x19, 2536]
	mov	w1, 255
	bl	ftl_memset
	ldr	x0, [x19, 3728]
	mov	x1, x20
	mov	w2, 48
	mov	x20, x23
	adrp	x23, .LANCHOR1
	add	x23, x23, :lo12:.LANCHOR1
	add	x23, x23, 560
	bl	ftl_memcpy
	ldrh	w2, [x19, 244]
	ldr	x1, [x19, 72]
	ldr	x0, [x19, 3728]
	lsl	w2, w2, 1
	add	x0, x0, 48
	bl	ftl_memcpy
	mov	w0, 0
	bl	FtlUpdateVaildLpn
.L1303:
	ldrh	w2, [x20, 2]
	ldrh	w1, [x20]
	ldr	x0, [x19, 3456]
	str	x0, [x19, 3728]
	ldr	x0, [x19, 3520]
	str	x0, [x19, 3736]
	orr	w0, w2, w1, lsl 10
	str	w0, [x19, 3724]
	ldrh	w0, [x19, 308]
	sub	w0, w0, #1
	cmp	w2, w0
	blt	.L1304
	ldrh	w24, [x20, 4]
	strh	wzr, [x20, 2]
	strh	w1, [x20, 4]
	bl	FtlFreeSysBlkQueueOut
	ldr	w1, [x19, 2660]
	str	w1, [x20, 8]
	add	w2, w1, 1
	str	w2, [x19, 2660]
	ubfiz	w2, w0, 10, 16
	str	w2, [x19, 3724]
	strh	w0, [x20]
	strh	w0, [x26, 2]
	str	w1, [x26, 4]
.L1304:
	ldrh	w1, [x19, 314]
	ldr	x0, [x19, 3456]
	bl	js_hash
	str	w0, [x26, 12]
	mov	w3, 1
	add	x0, x19, 3720
	mov	w1, w3
	mov	w2, w3
	bl	FlashProgPages
	ldrh	w0, [x20, 2]
	ldr	w1, [x19, 3720]
	add	w0, w0, 1
	and	w0, w0, 65535
	strh	w0, [x20, 2]
	cmn	w1, #1
	bne	.L1305
	cmp	w0, 1
	bne	.L1306
	adrp	x0, .LC8
	mov	w2, 1138
	mov	x1, x23
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1306:
	ldrh	w0, [x20, 2]
	cmp	w0, 1
	bne	.L1307
	ldrh	w0, [x19, 308]
	sub	w0, w0, #1
	strh	w0, [x20, 2]
.L1307:
	add	w21, w21, 1
	and	w21, w21, 65535
	cmp	w21, 3
	bls	.L1303
	add	x22, x22, :lo12:.LANCHOR0
	adrp	x0, .LC120
	mov	w2, w21
	add	x0, x0, :lo12:.LC120
	ldr	w1, [x22, 3724]
	bl	sftl_printk
.L1309:
	b	.L1309
.L1305:
	cmp	w0, 1
	beq	.L1303
	cmp	w1, 256
	beq	.L1303
	mov	w0, 65535
	cmp	w24, w0
	beq	.L1310
	mov	w1, 1
	mov	w0, w24
	bl	FtlFreeSysBlkQueueIn
.L1310:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	l2p_flush
	mov	w0, 1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlSysFlush, .-FtlSysFlush
	.align	2
	.global	sftl_deinit
	.type	sftl_deinit, %function
sftl_deinit:
	adrp	x0, .LANCHOR2
	ldr	w0, [x0, #:lo12:.LANCHOR2]
	cmp	w0, 1
	bne	.L1323
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FtlSysFlush
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
.L1323:
	mov	w0, 0
	ret
	.size	sftl_deinit, .-sftl_deinit
	.align	2
	.global	sftl_discard
	.type	sftl_discard, %function
sftl_discard:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x2, x19, :lo12:.LANCHOR0
	str	x21, [sp, 32]
	mov	w20, w1
	ldr	w1, [x2, 344]
	cmp	w1, w0
	bls	.L1336
	cmp	w1, w20
	bcc	.L1336
	add	w3, w0, w20
	cmp	w1, w3
	bcc	.L1336
	cmp	w20, 31
	bhi	.L1328
.L1344:
	mov	w0, 0
.L1326:
	ldp	x19, x20, [sp, 16]
	ldr	x21, [sp, 32]
	ldp	x29, x30, [sp], 64
	ret
.L1328:
	ldrh	w1, [x2, 262]
	udiv	w21, w0, w1
	msub	w0, w1, w21, w0
	ands	w0, w0, 65535
	beq	.L1329
	sub	w0, w1, w0
	add	w21, w21, 1
	cmp	w0, w20
	csel	w0, w0, w20, ls
	sub	w20, w20, w0, uxth
.L1329:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, -1
	str	w0, [x29, 60]
.L1330:
	ldrh	w0, [x19, 262]
	cmp	w20, w0
	bcs	.L1332
	ldr	w0, [x19, 3956]
	cmp	w0, 32
	bls	.L1344
	str	wzr, [x19, 3956]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	b	.L1344
.L1332:
	mov	w2, 0
	add	x1, x29, 56
	mov	w0, w21
	bl	log2phys
	ldr	w0, [x29, 56]
	cmn	w0, #1
	beq	.L1331
	ldr	w0, [x19, 3956]
	mov	w2, 1
	add	x1, x29, 60
	add	w0, w0, 1
	str	w0, [x19, 3956]
	ldr	w0, [x19, 2632]
	add	w0, w0, 1
	str	w0, [x19, 2632]
	mov	w0, w21
	bl	log2phys
	ldr	w0, [x29, 56]
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L1331:
	ldrh	w0, [x19, 262]
	add	w21, w21, 1
	sub	w20, w20, w0
	b	.L1330
.L1336:
	mov	w0, -1
	b	.L1326
	.size	sftl_discard, .-sftl_discard
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	stp	x29, x30, [sp, -96]!
	adrp	x0, .LC111
	add	x0, x0, :lo12:.LC111
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	add	x19, x20, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	adrp	x22, .LANCHOR1
	add	x1, x22, :lo12:.LANCHOR1
	add	x1, x1, 576
	stp	x23, x24, [sp, 48]
	str	x25, [sp, 64]
	bl	sftl_printk
	ldr	x0, [x19, 3560]
	mov	w21, 0
	ldrh	w2, [x19, 246]
	mov	w1, 0
	lsl	w2, w2, 1
	bl	ftl_memset
.L1346:
	ldr	w0, [x19, 2624]
	cmp	w21, w0
	bcc	.L1348
	add	x20, x20, :lo12:.LANCHOR0
	adrp	x23, .LC121
	add	x25, x20, 2592
	add	x23, x23, :lo12:.LC121
	mov	w19, 0
	mov	w24, 65535
.L1349:
	ldrh	w0, [x20, 244]
	cmp	w0, w19
	bhi	.L1355
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	x25, [sp, 64]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x29, x30, [sp], 96
	ret
.L1348:
	mov	w2, 0
	add	x1, x29, 92
	mov	w0, w21
	bl	log2phys
	ldr	w0, [x29, 92]
	cmn	w0, #1
	beq	.L1347
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	ldr	x2, [x19, 3560]
	ubfiz	x0, x0, 1, 16
	ldrh	w1, [x2, x0]
	add	w1, w1, 1
	strh	w1, [x2, x0]
.L1347:
	add	w21, w21, 1
	b	.L1346
.L1355:
	ldr	x0, [x20, 72]
	ubfiz	x21, x19, 1, 16
	ldrh	w2, [x0, x21]
	ldr	x0, [x20, 3560]
	ldrh	w3, [x0, x21]
	cmp	w2, w3
	beq	.L1351
	cmp	w2, w24
	beq	.L1351
	mov	x0, x23
	mov	w1, w19
	bl	sftl_printk
	ldrh	w0, [x20, 24]
	cmp	w0, w19
	beq	.L1351
	ldrh	w0, [x20, 128]
	cmp	w0, w19
	beq	.L1351
	ldrh	w0, [x20, 80]
	cmp	w0, w19
	beq	.L1351
	ldr	x0, [x20, 72]
	ldrh	w1, [x0, x21]
	cbnz	w1, .L1353
	ldr	x1, [x20, 3560]
	ldrh	w1, [x1, x21]
	strh	w1, [x0, x21]
	mov	w1, w19
	mov	x0, x25
	bl	List_remove_node
	ldrh	w0, [x20, 228]
	cbnz	w0, .L1354
	add	x1, x22, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2323
	add	x1, x1, 576
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1354:
	ldrh	w0, [x20, 228]
	sub	w0, w0, #1
	strh	w0, [x20, 228]
	mov	w0, w19
	bl	INSERT_DATA_LIST
.L1351:
	add	w19, w19, 1
	and	w19, w19, 65535
	b	.L1349
.L1353:
	ldr	x1, [x20, 3560]
	ldrh	w1, [x1, x21]
	strh	w1, [x0, x21]
	mov	w0, w19
	bl	update_vpc_list
	b	.L1351
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	allocate_new_data_superblock
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	str	x21, [sp, 32]
	mov	x21, x0
	ldrh	w20, [x0]
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w0, [x0, 244]
	cmp	w0, w20
	bcs	.L1361
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 2759
	add	x1, x1, 600
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1361:
	mov	w0, 65535
	cmp	w20, w0
	beq	.L1362
	add	x1, x19, :lo12:.LANCHOR0
	ubfiz	x0, x20, 1, 16
	ldr	x1, [x1, 72]
	ldrh	w0, [x1, x0]
	cbz	w0, .L1363
	mov	w0, w20
	bl	INSERT_DATA_LIST
.L1362:
	add	x2, x19, :lo12:.LANCHOR0
	mov	w0, 1
	strb	w0, [x21, 8]
	mov	w1, 65535
	ldrh	w0, [x2, 3706]
	cmp	w0, w1
	beq	.L1364
	cmp	w20, w0
	bne	.L1365
	ldr	x2, [x2, 72]
	ubfiz	x1, x0, 1, 16
	ldrh	w1, [x2, x1]
	cbz	w1, .L1366
.L1365:
	bl	update_vpc_list
.L1366:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, -1
	strh	w0, [x19, 3706]
.L1364:
	mov	x0, x21
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	w0, 0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	ldr	x21, [sp, 32]
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
.L1363:
	mov	w0, w20
	bl	INSERT_FREE_LIST
	b	.L1362
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlProgPages
	.type	FtlProgPages, %function
FtlProgPages:
	stp	x29, x30, [sp, -96]!
	mov	w2, 0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x20, x3
	stp	x23, x24, [sp, 48]
	adrp	x23, .LC122
	stp	x21, x22, [sp, 32]
	adrp	x24, .LANCHOR0
	str	x25, [sp, 64]
	mov	w21, w1
	add	x23, x23, :lo12:.LC122
	add	x25, x24, :lo12:.LANCHOR0
	ldrb	w3, [x3, 9]
	ubfiz	x21, x21, 5, 32
	add	x21, x21, 4
	add	x19, x0, 4
	add	x21, x0, x21
	bl	FlashProgPages
.L1372:
	cmp	x21, x19
	beq	.L1378
	sub	x22, x19, #4
	b	.L1379
.L1374:
	ldr	w1, [x19]
	mov	x0, x23
	bl	sftl_printk
	ldr	w0, [x19]
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
	ldrh	w0, [x20, 4]
	cbnz	w0, .L1373
	mov	x0, x20
	bl	allocate_new_data_superblock
.L1373:
	mov	x0, x20
	bl	get_new_active_ppa
	str	w0, [x19]
	mov	w2, 0
	str	w0, [x29, 92]
	mov	w1, 1
	ldrb	w3, [x20, 9]
	mov	x0, x22
	bl	FlashProgPages
.L1379:
	ldr	w2, [x19, -4]
	cmp	w2, 256
	ccmn	w2, #1, 4, ne
	beq	.L1374
	ldrb	w1, [x20, 6]
	ldrh	w0, [x25, 236]
	cmp	w1, w0
	bcc	.L1375
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 985
	add	x1, x1, 632
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1375:
	add	x1, x29, 96
	ldr	w0, [x22, 4]
	mov	w2, 1
	str	w0, [x1, -4]!
	ldr	w0, [x22, 24]
	bl	log2phys
	ldr	x0, [x22, 16]
	ldr	w4, [x0, 12]
	lsr	w0, w4, 10
	bl	P2V_block_in_plane
	and	w1, w0, 65535
	mov	w22, w1
	cmn	w4, #1
	beq	.L1376
	add	x2, x24, :lo12:.LANCHOR0
	ubfiz	x0, x1, 1, 16
	ldr	x2, [x2, 72]
	ldrh	w0, [x2, x0]
	cbnz	w0, .L1377
	adrp	x0, .LC123
	mov	w2, 0
	add	x0, x0, :lo12:.LC123
	bl	sftl_printk
.L1377:
	mov	w0, w22
	bl	decrement_vpc_count
.L1376:
	add	x19, x19, 32
	b	.L1372
.L1378:
	adrp	x0, .LANCHOR0+236
	ldrb	w1, [x20, 6]
	ldrh	w0, [x0, #:lo12:.LANCHOR0+236]
	cmp	w1, w0
	bcc	.L1371
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 1000
	add	x1, x1, 632
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1371:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldr	x25, [sp, 64]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlProgPages, .-FtlProgPages
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	stp	x29, x30, [sp, -96]!
	mov	w3, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x2, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	str	x25, [sp, 64]
	ldrh	w4, [x2, 128]
	ldrh	w1, [x2, 306]
	cmp	w4, w3
	bne	.L1386
.L1393:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 65535
	add	x1, x0, 128
	ldrh	w3, [x0, 128]
	str	wzr, [x0, 3712]
	cmp	w3, w2
	bne	.L1406
.L1410:
	mov	w0, 0
.L1385:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldr	x25, [sp, 64]
	ldp	x29, x30, [sp], 96
	ret
.L1386:
	adrp	x20, .LANCHOR2
	cbz	w0, .L1389
	add	x0, x20, :lo12:.LANCHOR2
	ldrh	w4, [x0, 4]
	cmp	w4, w3
	beq	.L1390
.L1391:
	mov	w1, 2
.L1389:
	add	x20, x20, :lo12:.LANCHOR2
	add	x0, x19, :lo12:.LANCHOR0
	add	x0, x0, 128
	bl	FtlGcScanTempBlk
	str	w0, [x29, 92]
	mov	w0, 65535
	ldrh	w1, [x20, 4]
	cmp	w1, w0
	beq	.L1393
	mov	w0, 1
	b	.L1385
.L1390:
	strh	wzr, [x0, 4]
	ldrh	w0, [x2, 228]
	cmp	w0, 17
	bhi	.L1391
	b	.L1389
.L1406:
	ldrh	w2, [x0, 3296]
	ldrb	w1, [x1, 7]
	ldrh	w0, [x0, 306]
	mul	w0, w1, w0
	cmp	w2, w0
	beq	.L1394
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 164
	add	x1, x1, 648
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1394:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w21, 0
	mov	x20, x0
	mov	w25, 12
	ldrb	w1, [x0, 135]
	ldrh	w4, [x0, 306]
	ldr	x2, [x0, 72]
	ldrh	w3, [x0, 128]
	mul	w1, w1, w4
	strh	w1, [x2, x3, lsl 1]
	ldr	w2, [x0, 2648]
	ldrh	w1, [x0, 3296]
	add	w1, w1, w2
	str	w1, [x0, 2648]
.L1395:
	ldrh	w0, [x20, 3296]
	cmp	w0, w21
	bhi	.L1399
	mov	w0, -1
	bl	decrement_vpc_count
	ldrh	w0, [x20, 128]
	ldr	x2, [x20, 72]
	ubfiz	x1, x0, 1, 16
	ldrh	w1, [x2, x1]
	cbz	w1, .L1400
	bl	INSERT_DATA_LIST
.L1401:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w20, -1
	strh	wzr, [x19, 3296]
	strh	w20, [x19, 128]
	strh	wzr, [x19, 3284]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	strh	w20, [x19, 176]
	ldrh	w0, [x19, 2692]
	ldrh	w1, [x19, 228]
	add	w0, w0, w0, lsl 1
	cmp	w1, w0, lsr 2
	ble	.L1410
	mov	w0, 20
	strh	w0, [x19, 3224]
	b	.L1410
.L1399:
	umull	x23, w21, w25
	ldr	x24, [x20, 3304]
	ldr	w1, [x20, 2624]
	add	x22, x24, x23
	ldr	w0, [x22, 8]
	cmp	w0, w1
	bcc	.L1396
.L1408:
	ldrh	w0, [x20, 128]
	b	.L1409
.L1396:
	add	x1, x29, 92
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x24, x23]
	ldr	w1, [x29, 92]
	cmp	w0, w1
	bne	.L1398
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	mov	w23, w0
	ldr	w0, [x22, 8]
	mov	w2, 1
	add	x1, x22, 4
	bl	log2phys
	mov	w0, w23
.L1409:
	bl	decrement_vpc_count
	b	.L1397
.L1398:
	ldr	w0, [x22, 4]
	cmp	w1, w0
	bne	.L1408
.L1397:
	add	w21, w21, 1
	and	w21, w21, 65535
	b	.L1395
.L1400:
	bl	INSERT_FREE_LIST
	b	.L1401
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	add	x20, x19, 128
	mov	x0, x20
	ldrh	w1, [x19, 306]
	bl	FtlGcScanTempBlk
	ldrh	w1, [x19, 130]
	ldrh	w0, [x19, 306]
	cmp	w1, w0
	bcc	.L1411
	add	x0, x19, 3640
	bl	FtlMapBlkWriteDump_data
	mov	w0, 0
	bl	FtlGcFreeTempBlock
	str	wzr, [x19, 3712]
.L1411:
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	add	x20, x19, 24
	add	x19, x19, 80
	mov	x0, x20
	str	wzr, [x19, 3676]
	bl	FtlRecoverySuperblock
	mov	x0, x20
	bl	FtlSlcSuperblockCheck
	mov	x0, x19
	bl	FtlRecoverySuperblock
	mov	x0, x19
	bl	FtlSlcSuperblockCheck
	bl	FtlGcPageRecovery
	mov	w0, -1
	bl	decrement_vpc_count
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	stp	x29, x30, [sp, -32]!
	adrp	x11, .LANCHOR0
	mov	w3, 0
	mov	w2, 0
	add	x29, sp, 0
	str	x19, [sp, 16]
	add	x19, x11, :lo12:.LANCHOR0
	ldr	w1, [x19, 3240]
	ldr	x0, [x19, 3440]
	bl	FlashProgPages
	mov	w10, 0
.L1417:
	ldr	w1, [x19, 3240]
	cmp	w10, w1
	bcc	.L1420
	ldr	x0, [x19, 3440]
	bl	FtlGcBufFree
	str	wzr, [x19, 3240]
	ldrh	w0, [x19, 132]
	cbnz	w0, .L1422
	mov	w0, 1
	bl	FtlGcFreeTempBlock
	mov	w0, 1
.L1416:
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L1420:
	ldr	x2, [x19, 3440]
	ubfiz	x0, x10, 5, 16
	add	x1, x2, x0
	ldr	w2, [x2, x0]
	ldr	x3, [x1, 16]
	cmn	w2, #1
	bne	.L1418
.L1424:
	ldr	w1, [x1, 4]
	ldr	w0, [x3, 12]
	bl	FtlGcUpdatePage
	add	w10, w10, 1
	and	w10, w10, 65535
	b	.L1417
.L1418:
	ldr	w2, [x3, 8]
	b	.L1424
.L1422:
	mov	w0, 0
	b	.L1416
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x0, x19, :lo12:.LANCHOR0
	add	x1, x0, 128
	ldrh	w2, [x0, 128]
	mov	w0, 65535
	cmp	w2, w0
	beq	.L1426
	ldrh	w0, [x1, 4]
	cbnz	w0, .L1427
.L1426:
	add	x20, x19, :lo12:.LANCHOR0
	mov	w0, 0
	bl	FtlGcFreeTempBlock
	add	x0, x20, 128
	strb	wzr, [x0, 8]
	bl	allocate_data_superblock
	strh	wzr, [x20, 3284]
	strh	wzr, [x20, 3296]
	bl	l2p_flush
	mov	w0, 0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L1427:
	add	x0, x19, :lo12:.LANCHOR0
	add	x0, x0, 128
	bl	get_new_active_ppa
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x19, .LANCHOR0
	add	x1, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	stp	x23, x24, [sp, 48]
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
	ldr	w2, [x1, 3416]
	cbnz	w2, .L1508
	ldrh	w2, [x1, 2584]
	cmp	w2, 47
	bls	.L1508
	mov	w23, w0
	adrp	x0, .LANCHOR2+4
	ldrh	w2, [x0, #:lo12:.LANCHOR2+4]
	mov	w0, 65535
	cmp	w2, w0
	bne	.L1431
.L1434:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 65535
	ldrh	w1, [x0, 224]
	cmp	w1, w2
	bne	.L1432
.L1433:
	add	x20, x19, :lo12:.LANCHOR0
	bl	FtlReadRefresh
	mov	w21, 65535
	ldr	w0, [x20, 3228]
	ldrh	w1, [x20, 176]
	add	w0, w0, 1
	add	w0, w0, w23, lsl 7
	str	w0, [x20, 3228]
	cmp	w1, w21
	bne	.L1435
	ldrh	w21, [x20, 128]
	cmp	w21, w1
	bne	.L1511
	ldrh	w22, [x20, 226]
	cmp	w22, w21
	bne	.L1435
	ldrh	w1, [x20, 228]
	mov	w2, 1024
	cmp	w1, 24
	mov	w1, 5120
	csel	w1, w1, w2, cc
	cmp	w0, w1
	bls	.L1514
	strh	wzr, [x20, 3960]
	str	wzr, [x20, 3228]
	bl	GetSwlReplaceBlock
	and	w21, w0, 65535
	cmp	w21, w22
	bne	.L1515
	ldrh	w1, [x20, 228]
	ldrh	w0, [x20, 3226]
	cmp	w1, w0
	bcs	.L1438
	mov	w0, 64
	bl	List_get_gc_head_node
	and	w0, w0, 65535
	cmp	w0, w21
	beq	.L1440
	ldr	x1, [x20, 72]
	ubfiz	x0, x0, 1, 16
	ldrh	w0, [x1, x0]
	cmp	w0, 7
	bhi	.L1441
	mov	w0, 0
	bl	List_get_gc_head_node
	and	w22, w0, 65535
	mov	w0, 128
	strh	w0, [x20, 3226]
	cmp	w22, w21
	bne	.L1437
.L1440:
	bl	FtlGcReFreshBadBlk
.L1435:
	cmp	w23, 0
	mov	w0, 65535
	add	x1, x19, :lo12:.LANCHOR0
	ccmp	w21, w0, 0, eq
	bne	.L1443
	ldrh	w0, [x1, 228]
	cmp	w0, 24
	bhi	.L1516
	cmp	w0, 16
	ldrh	w20, [x1, 306]
	bls	.L1445
	lsr	w20, w20, 5
.L1444:
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w2, [x1, 3224]
	cmp	w2, w0
	bcs	.L1448
	ldrh	w0, [x1, 128]
	mov	w2, 65535
	cmp	w0, w2
	bne	.L1449
	ldrh	w2, [x1, 226]
	cmp	w2, w0
	bne	.L1449
	ldrh	w0, [x1, 3960]
	cbnz	w0, .L1450
	ldr	w2, [x1, 2624]
	ldr	w3, [x1, 2620]
	add	w2, w2, w2, lsl 1
	cmp	w3, w2, lsr 2
	bcs	.L1451
.L1450:
	add	x2, x19, :lo12:.LANCHOR0
	ldrh	w1, [x2, 2692]
	add	w1, w1, w1, lsl 1
	asr	w1, w1, 2
	strh	w1, [x2, 3224]
.L1452:
	add	x19, x19, :lo12:.LANCHOR0
	str	wzr, [x19, 3236]
.L1429:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 128
	ret
.L1431:
	ldrh	w1, [x1, 128]
	cmp	w1, w0
	beq	.L1434
	mov	w0, 1
	bl	FtlGcFreeTempBlock
	cbz	w0, .L1434
	mov	w0, 1
	b	.L1429
.L1432:
	ldrh	w3, [x0, 226]
	cmp	w3, w2
	bne	.L1433
	strh	w1, [x0, 226]
	mov	w1, -1
	strh	w1, [x0, 224]
	b	.L1433
.L1441:
	mov	w0, 64
.L1546:
	strh	w0, [x20, 3226]
	b	.L1440
.L1438:
	mov	w0, 80
	b	.L1546
.L1515:
	mov	w22, w21
.L1437:
	add	x0, x19, :lo12:.LANCHOR0
	ubfiz	x1, x22, 1, 32
	mov	w21, w22
	ldr	x2, [x0, 72]
	ldr	x3, [x0, 2600]
	ldrh	w5, [x0, 3224]
	ldrh	w4, [x3, x1]
	ldrh	w3, [x2, x1]
	mov	w1, w22
	ldrh	w2, [x0, 228]
	adrp	x0, .LC124
	add	x0, x0, :lo12:.LC124
	bl	sftl_printk
	b	.L1440
.L1511:
	mov	w21, w1
	b	.L1435
.L1514:
	mov	w21, w22
	b	.L1435
.L1445:
	cmp	w0, 12
	bls	.L1446
	lsr	w20, w20, 4
	b	.L1444
.L1446:
	mov	w1, w20
	cmp	w0, 9
	lsr	w20, w20, 2
	csel	w20, w20, w1, cs
	b	.L1444
.L1516:
	mov	w20, 1
	b	.L1444
.L1451:
	mov	w2, 18
	strh	w2, [x1, 3224]
	b	.L1452
.L1449:
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w0, [x1, 2692]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1, 3224]
.L1448:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w0, [x0, 3312]
	cbz	w0, .L1517
	add	w20, w20, 32
	and	w20, w20, 65535
.L1517:
	mov	w21, 65535
.L1454:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 65535
	ldrh	w1, [x0, 176]
	cmp	w1, w2
	bne	.L1463
	cmp	w21, w1
	beq	.L1464
	strh	w21, [x0, 176]
.L1465:
	add	x5, x19, :lo12:.LANCHOR0
	mov	w1, 65535
	ldrh	w0, [x5, 176]
	strb	wzr, [x5, 184]
	cmp	w0, w1
	beq	.L1463
	bl	IsBlkInGcList
	cbz	w0, .L1468
	mov	w0, -1
	strh	w0, [x5, 176]
.L1468:
	add	x22, x19, :lo12:.LANCHOR0
	mov	w0, 65535
	add	x24, x22, 176
	ldrh	w1, [x22, 176]
	cmp	w1, w0
	beq	.L1463
	mov	x0, x24
	bl	make_superblock
	ldrh	w1, [x22, 176]
	ldr	x0, [x22, 72]
	strh	wzr, [x22, 178]
	strb	wzr, [x22, 182]
	strh	wzr, [x22, 3962]
	ldrh	w0, [x0, x1, lsl 1]
	strh	w0, [x22, 3964]
.L1463:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 176]
	ldrh	w2, [x0, 24]
	cmp	w2, w1
	beq	.L1469
	ldrh	w2, [x0, 80]
	cmp	w2, w1
	beq	.L1469
	ldrh	w0, [x0, 128]
	cmp	w0, w1
	bne	.L1470
.L1469:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, -1
	strh	w1, [x0, 176]
.L1470:
	adrp	x25, .LANCHOR1
	add	x25, x25, :lo12:.LANCHOR1
	add	x25, x25, 672
.L1505:
	add	x22, x19, :lo12:.LANCHOR0
	mov	w0, 65535
	ldrh	w24, [x22, 176]
	cmp	w24, w0
	bne	.L1471
	mov	w26, 2
	str	wzr, [x22, 3236]
.L1472:
	ldrh	w5, [x22, 3232]
	mov	w0, w5
	bl	List_get_gc_head_node
	and	w6, w0, 65535
	strh	w6, [x22, 176]
	cmp	w6, w24
	bne	.L1473
	strh	wzr, [x22, 3232]
	mov	w0, 8
	b	.L1429
.L1443:
	ldrh	w2, [x1, 128]
	cmp	w2, w0
	bne	.L1520
	ldrh	w0, [x1, 226]
	cmp	w0, w2
	bne	.L1520
	cmp	w21, w0
	bne	.L1520
	ldrh	w0, [x1, 176]
	cmp	w0, w21
	beq	.L1455
.L1520:
	mov	w20, 1
	b	.L1454
.L1455:
	ldrh	w2, [x1, 228]
	ldrh	w0, [x1, 3224]
	str	wzr, [x1, 3236]
	cmp	w2, w0
	bls	.L1456
	ldrh	w0, [x1, 3960]
	cbnz	w0, .L1457
	ldr	w0, [x1, 2624]
	ldr	w2, [x1, 2620]
	add	w0, w0, w0, lsl 1
	cmp	w2, w0, lsr 2
	bcs	.L1458
.L1457:
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w0, [x1, 2692]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
.L1547:
	strh	w0, [x1, 3224]
	bl	FtlReadRefresh
	mov	w0, 0
	bl	List_get_gc_head_node
	add	x1, x19, :lo12:.LANCHOR0
	ubfiz	x0, x0, 1, 16
	ldr	x2, [x1, 72]
	ldrh	w0, [x2, x0]
	cmp	w0, 4
	bls	.L1456
	ldrh	w0, [x1, 3960]
	b	.L1429
.L1458:
	mov	w0, 18
	b	.L1547
.L1456:
	add	x20, x19, :lo12:.LANCHOR0
	ldrh	w0, [x20, 3960]
	cbnz	w0, .L1520
	ldrh	w5, [x20, 2692]
	add	w0, w5, w5, lsl 1
	asr	w0, w0, 2
	strh	w0, [x20, 3224]
	mov	w0, 0
	bl	List_get_gc_head_node
	ldr	x1, [x20, 72]
	ubfiz	x0, x0, 1, 16
	ldrh	w2, [x20, 236]
	ldrh	w1, [x1, x0]
	ldrh	w0, [x20, 308]
	mul	w0, w0, w2
	mov	w2, 2
	sdiv	w0, w0, w2
	cmp	w1, w0
	ble	.L1461
	ldrh	w0, [x20, 228]
	sub	w5, w5, #1
	cmp	w0, w5
	blt	.L1461
	bl	FtlReadRefresh
	ldrh	w0, [x20, 3960]
	b	.L1429
.L1461:
	cbnz	w1, .L1520
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, -1
	bl	decrement_vpc_count
	ldrh	w0, [x19, 228]
	add	w0, w0, 1
	b	.L1429
.L1464:
	ldrh	w1, [x0, 226]
	cmp	w1, w21
	beq	.L1465
	ldr	x2, [x0, 72]
	ubfiz	x1, x1, 1, 16
	ldrh	w1, [x2, x1]
	cbnz	w1, .L1466
	mov	w1, -1
	strh	w1, [x0, 226]
.L1466:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 226]
	strh	w1, [x0, 176]
	mov	w1, -1
	strh	w1, [x0, 226]
	b	.L1465
.L1473:
	mov	w0, w6
	bl	IsBlkInGcList
	add	w5, w5, 1
	cbz	w0, .L1474
	strh	w5, [x22, 3232]
	b	.L1472
.L1474:
	ldrh	w4, [x22, 236]
	ubfiz	x1, x6, 1, 16
	ldrh	w0, [x22, 306]
	and	w5, w5, 65535
	ldr	x2, [x22, 72]
	strh	w5, [x22, 3232]
	mul	w0, w0, w4
	ldrh	w3, [x2, x1]
	sdiv	w4, w0, w26
	cmp	w3, w4
	bgt	.L1476
	cmp	w5, 48
	bls	.L1477
	cmp	w3, 8
	bls	.L1477
	ldrh	w3, [x22, 3284]
	cmp	w3, 35
	bhi	.L1477
.L1476:
	strh	wzr, [x22, 3232]
.L1477:
	ldrh	w1, [x2, x1]
	cmp	w0, w1
	bgt	.L1478
	cmp	w21, w24
	bne	.L1478
	ldrh	w0, [x22, 3232]
	cmp	w0, 3
	bhi	.L1478
	mov	w0, -1
	strh	wzr, [x22, 3232]
	strh	w0, [x22, 176]
.L1548:
	ldrh	w0, [x22, 3960]
	b	.L1429
.L1478:
	cbnz	w1, .L1479
	mov	w0, -1
	bl	decrement_vpc_count
	ldrh	w0, [x22, 3232]
	add	w0, w0, 1
	strh	w0, [x22, 3232]
	b	.L1472
.L1479:
	add	x0, x19, :lo12:.LANCHOR0
	strb	wzr, [x0, 184]
	ldrh	w0, [x0, 24]
	cmp	w0, w6
	bne	.L1480
	adrp	x0, .LC8
	mov	w2, 717
	mov	x1, x25
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1480:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 176]
	ldrh	w0, [x0, 80]
	cmp	w1, w0
	bne	.L1481
	adrp	x0, .LC8
	mov	w2, 718
	mov	x1, x25
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1481:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 176]
	ldrh	w0, [x0, 128]
	cmp	w1, w0
	bne	.L1482
	adrp	x0, .LC8
	mov	w2, 719
	mov	x1, x25
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1482:
	add	x22, x19, :lo12:.LANCHOR0
	add	x24, x22, 176
	mov	x0, x24
	bl	make_superblock
	ldrh	w1, [x22, 176]
	ldr	x0, [x22, 72]
	strh	wzr, [x22, 3962]
	ldrh	w0, [x0, x1, lsl 1]
	strh	w0, [x22, 3964]
	strh	wzr, [x22, 178]
	strb	wzr, [x22, 182]
.L1471:
	add	x1, x19, :lo12:.LANCHOR0
	mov	w0, 1
	str	w0, [x1, 3416]
	ldrh	w26, [x1, 306]
	cbz	w23, .L1483
	ldrh	w0, [x1, 236]
	ldrh	w2, [x1, 176]
	ldr	x1, [x1, 72]
	mul	w0, w0, w26
	ldrh	w1, [x1, x2, lsl 1]
	sub	w0, w0, w1
	mov	w1, 4
	sdiv	w0, w0, w1
	add	w20, w20, w0
	and	w20, w20, 65535
.L1483:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w0, [x0, 178]
	add	w1, w0, w20
	cmp	w1, w26
	ble	.L1484
	sub	w20, w26, w0
	and	w20, w20, 65535
.L1484:
	mov	w27, 0
.L1485:
	cmp	w20, w27, uxth
	bls	.L1493
	add	x0, x19, :lo12:.LANCHOR0
	mov	w24, 0
	add	x3, x0, 192
	mov	w1, 0
	mov	w6, 65535
	ldrh	w4, [x0, 178]
	ldrh	w7, [x0, 236]
	add	w4, w4, w27
	b	.L1494
.L1487:
	ldrh	w2, [x3]
	cmp	w2, w6
	beq	.L1486
	ldr	x5, [x0, 3272]
	ubfiz	x8, x24, 5, 16
	add	w24, w24, 1
	orr	w2, w4, w2, lsl 10
	add	x5, x5, x8
	and	w24, w24, 65535
	str	w2, [x5, 4]
.L1486:
	add	w1, w1, 1
	add	x3, x3, 2
	and	w1, w1, 65535
.L1494:
	cmp	w1, w7
	bne	.L1487
	add	x22, x19, :lo12:.LANCHOR0
	mov	w1, w24
	mov	x28, 0
	ldrb	w2, [x22, 184]
	ldr	x0, [x22, 3272]
	bl	FlashReadPages
	ubfiz	x0, x24, 5, 16
	str	x0, [x29, 104]
.L1488:
	ldr	x0, [x29, 104]
	cmp	x0, x28
	bne	.L1492
	add	w27, w27, 1
	b	.L1485
.L1492:
	ldr	x0, [x22, 3272]
	add	x1, x0, x28
	ldr	w0, [x0, x28]
	cmn	w0, #1
	beq	.L1489
	ldr	x24, [x1, 16]
	mov	w0, 61589
	ldrh	w1, [x24]
	cmp	w1, w0
	bne	.L1489
	ldr	w4, [x24, 8]
	cmn	w4, #1
	bne	.L1490
	str	w4, [x29, 96]
	mov	w2, 753
	mov	x1, x25
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	ldr	w4, [x29, 96]
.L1490:
	mov	w2, 0
	add	x1, x29, 120
	mov	w0, w4
	bl	log2phys
	ldr	x0, [x22, 3272]
	ldr	w1, [x29, 120]
	add	x0, x0, x28
	ldr	w2, [x0, 4]
	cmp	w2, w1
	bne	.L1489
	ldrh	w1, [x22, 3962]
	ldr	x2, [x22, 3440]
	add	w1, w1, 1
	strh	w1, [x22, 3962]
	ldr	w1, [x22, 3240]
	ldr	w0, [x0, 24]
	add	x1, x2, x1, lsl 5
	str	x1, [x29, 96]
	str	w0, [x1, 24]
	bl	Ftl_get_new_temp_ppa
	ldr	x1, [x29, 96]
	str	w0, [x1, 4]
	ldr	w0, [x22, 3240]
	ldr	x1, [x22, 3440]
	add	x0, x1, x0, lsl 5
	ldr	x1, [x22, 3272]
	add	x1, x1, x28
	ldr	x2, [x1, 8]
	str	x2, [x0, 8]
	ldr	x1, [x1, 16]
	str	x1, [x0, 16]
	ldr	w0, [x29, 120]
	mov	w1, 1
	str	w0, [x24, 12]
	ldrh	w0, [x22, 128]
	strh	w0, [x24, 2]
	ldr	w0, [x22, 2664]
	str	w0, [x24, 4]
	ldr	w0, [x22, 3240]
	add	w0, w0, 1
	str	w0, [x22, 3240]
	ldr	x0, [x22, 3272]
	add	x0, x0, x28
	bl	FtlGcBufAlloc
	ldrb	w1, [x22, 135]
	ldr	w0, [x22, 3240]
	cmp	w1, w0
	beq	.L1491
	ldrh	w0, [x22, 132]
	cbnz	w0, .L1489
.L1491:
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L1489
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, -1
	strh	wzr, [x19, 178]
	strh	w0, [x19, 176]
	str	wzr, [x19, 3416]
	ldrh	w0, [x19, 3960]
	b	.L1429
.L1489:
	add	x28, x28, 32
	b	.L1488
.L1493:
	add	x22, x19, :lo12:.LANCHOR0
	ldrh	w0, [x22, 178]
	add	w20, w20, w0
	and	w20, w20, 65535
	strh	w20, [x22, 178]
	cmp	w26, w20
	bhi	.L1495
	ldr	w0, [x22, 3240]
	cbz	w0, .L1496
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L1496
	str	wzr, [x22, 3416]
	b	.L1548
.L1496:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 3962]
	cbnz	w1, .L1497
	ldrh	w2, [x0, 176]
	ldr	x1, [x0, 72]
	ldrh	w1, [x1, x2, lsl 1]
	cbz	w1, .L1497
	mov	x20, x0
	mov	w22, 0
.L1498:
	ldr	w0, [x20, 2624]
	cmp	w22, w0
	bcs	.L1503
	mov	w2, 0
	add	x1, x29, 124
	mov	w0, w22
	bl	log2phys
	ldr	w0, [x29, 124]
	cmn	w0, #1
	beq	.L1499
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	ldrh	w1, [x20, 176]
	cmp	w1, w0, uxth
	bne	.L1499
.L1503:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w1, [x0, 2624]
	cmp	w22, w1
	bcc	.L1497
	ldrh	w2, [x0, 176]
	ldr	x1, [x0, 72]
	strh	wzr, [x1, x2, lsl 1]
	ldrh	w0, [x0, 176]
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L1497:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, -1
	strh	w1, [x0, 176]
.L1495:
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w1, [x0, 228]
	str	wzr, [x0, 3416]
	cmp	w1, 2
	bhi	.L1504
	ldrh	w20, [x0, 306]
	b	.L1505
.L1499:
	add	w22, w22, 1
	b	.L1498
.L1504:
	ldrh	w0, [x0, 3960]
	cmp	w0, 0
	csinc	w0, w0, w1, ne
	b	.L1429
.L1508:
	mov	w0, 0
	b	.L1429
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	2
	.global	FtlRead
	.type	FtlRead, %function
FtlRead:
	stp	x29, x30, [sp, -160]!
	and	w0, w0, 255
	cmp	w0, 16
	add	x29, sp, 0
	stp	x25, x26, [sp, 64]
	mov	w26, w1
	stp	x21, x22, [sp, 32]
	mov	w25, w2
	stp	x19, x20, [sp, 16]
	mov	x22, x3
	stp	x23, x24, [sp, 48]
	stp	x27, x28, [sp, 80]
	bne	.L1550
	mov	x2, x3
	mov	w1, w25
	add	w0, w26, 256
	bl	FtlVendorPartRead
	mov	w19, w0
.L1549:
	mov	w0, w19
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 160
	ret
.L1550:
	adrp	x1, .LANCHOR0
	add	x3, x1, :lo12:.LANCHOR0
	str	x1, [x29, 96]
	ldr	w0, [x3, 344]
	cmp	w26, w0
	bcs	.L1573
	cmp	w2, w0
	bhi	.L1573
	add	w1, w26, w2
	str	w1, [x29, 128]
	cmp	w0, w1
	bcc	.L1573
	adrp	x0, .LANCHOR2
	ldr	w19, [x0, #:lo12:.LANCHOR2]
	cmn	w19, #1
	beq	.L1549
	ldrh	w0, [x3, 262]
	mov	x27, x3
	mov	w23, 0
	mov	w28, 0
	mov	w19, 0
	stp	wzr, wzr, [x29, 132]
	udiv	w1, w26, w0
	str	w1, [x29, 140]
	add	w1, w26, w2
	sub	w24, w1, #1
	ldr	w20, [x29, 140]
	udiv	w24, w24, w0
	ldr	w0, [x29, 140]
	sub	w21, w24, w0
	ldr	w0, [x3, 2656]
	add	w21, w21, 1
	add	w0, w0, w2
	str	w0, [x3, 2656]
	ldr	w0, [x3, 2628]
	add	w0, w0, w21
	str	w0, [x3, 2628]
.L1552:
	cbnz	w21, .L1569
	ldr	x0, [x29, 96]
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w1, [x0, 3312]
	cbnz	w1, .L1570
	ldrh	w0, [x0, 228]
	cmp	w0, 31
	bhi	.L1549
.L1570:
	mov	w1, 1
	mov	w0, 0
	bl	rk_ftl_garbage_collect
	b	.L1549
.L1569:
	add	x1, x29, 156
	mov	w2, 0
	mov	w0, w20
	bl	log2phys
	ldr	w1, [x29, 156]
	cmn	w1, #1
	bne	.L1553
	mov	w6, 0
.L1554:
	ldrh	w0, [x27, 262]
	cmp	w6, w0
	bcc	.L1556
.L1557:
	add	w20, w20, 1
	subs	w21, w21, #1
	beq	.L1561
	ldrh	w0, [x27, 236]
	cmp	w28, w0, lsl 2
	bne	.L1552
.L1561:
	cbz	w28, .L1552
	ldr	x0, [x27, 3432]
	mov	w1, w28
	mov	w2, 0
	bl	FlashReadPages
	ldr	w0, [x29, 132]
	lsl	w0, w0, 9
	str	w0, [x29, 112]
	ldr	w0, [x29, 136]
	lsl	w0, w0, 9
	str	x0, [x29, 120]
	lsl	w0, w23, 9
	str	w0, [x29, 116]
	ubfiz	x0, x28, 5, 32
	mov	x28, 0
	str	x0, [x29, 104]
.L1568:
	ldr	x0, [x27, 3432]
	ldr	w2, [x29, 140]
	add	x0, x0, x28
	ldr	w1, [x0, 24]
	cmp	w2, w1
	bne	.L1563
	ldr	x1, [x0, 8]
	ldr	x0, [x27, 3480]
	cmp	x1, x0
	bne	.L1564
	ldr	x0, [x29, 120]
	ldr	w2, [x29, 116]
	add	x1, x1, x0
	mov	x0, x22
.L1583:
	bl	ftl_memcpy
.L1564:
	ldr	x0, [x27, 3432]
	add	x0, x0, x28
	ldr	x1, [x0, 16]
	ldr	w2, [x0, 24]
	ldr	w1, [x1, 8]
	cmp	w2, w1
	beq	.L1565
	ldr	w1, [x27, 2784]
	add	w1, w1, 1
	str	w1, [x27, 2784]
.L1565:
	ldr	w2, [x0]
	cmn	w2, #1
	bne	.L1566
	ldr	w1, [x27, 2784]
	mov	w19, w2
	add	w1, w1, 1
	str	w1, [x27, 2784]
.L1566:
	ldr	w1, [x0]
	cmp	w1, 256
	bne	.L1567
	ldr	w0, [x0, 4]
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L1567:
	ldr	x0, [x29, 104]
	add	x28, x28, 32
	cmp	x0, x28
	bne	.L1568
	mov	w28, 0
	b	.L1552
.L1556:
	madd	w0, w20, w0, w6
	cmp	w26, w0
	bhi	.L1555
	ldr	w1, [x29, 128]
	cmp	w1, w0
	bls	.L1555
	sub	w0, w0, w26
	str	w6, [x29, 120]
	lsl	w0, w0, 9
	mov	w2, 512
	mov	w1, 0
	add	x0, x22, x0
	bl	ftl_memset
	ldr	w6, [x29, 120]
.L1555:
	add	w6, w6, 1
	b	.L1554
.L1553:
	ldr	x0, [x27, 3432]
	ubfiz	x2, x28, 5, 32
	add	x0, x0, x2
	str	w1, [x0, 4]
	ldr	w0, [x29, 140]
	cmp	w20, w0
	ldrh	w0, [x27, 262]
	bne	.L1558
	ldr	x1, [x27, 3432]
	ldr	x6, [x27, 3480]
	add	x1, x1, x2
	str	x6, [x1, 8]
	udiv	w6, w26, w0
	msub	w3, w6, w0, w26
	str	w3, [x29, 136]
	sub	w23, w0, w3
	cmp	w25, w23
	csel	w23, w25, w23, ls
	cmp	w23, w0
	bne	.L1559
	str	x22, [x1, 8]
.L1559:
	ldr	x0, [x27, 3432]
	ldr	x1, [x27, 3528]
	add	x2, x0, x2
	ldrh	w0, [x27, 316]
	str	w20, [x2, 24]
	mul	w0, w0, w28
	add	w28, w28, 1
	and	x0, x0, 4294967292
	add	x0, x1, x0
	str	x0, [x2, 16]
	b	.L1557
.L1558:
	cmp	w20, w24
	bne	.L1560
	ldr	x6, [x27, 3432]
	ldr	x1, [x27, 3488]
	add	x6, x6, x2
	ldr	w3, [x29, 128]
	str	x1, [x6, 8]
	mul	w1, w20, w0
	sub	w3, w3, w1
	str	w3, [x29, 132]
	cmp	w0, w3
	bne	.L1559
	sub	w1, w1, w26
	lsl	w1, w1, 9
	add	x1, x22, x1
	str	x1, [x6, 8]
	b	.L1559
.L1560:
	ldr	x1, [x27, 3432]
	mul	w0, w0, w20
	add	x1, x1, x2
	sub	w0, w0, w26
	lsl	w0, w0, 9
	add	x0, x22, x0
	str	x0, [x1, 8]
	b	.L1559
.L1563:
	cmp	w24, w1
	bne	.L1564
	ldr	x1, [x0, 8]
	ldr	x0, [x27, 3488]
	cmp	x1, x0
	bne	.L1564
	ldrh	w0, [x27, 262]
	ldr	w2, [x29, 112]
	mul	w0, w0, w24
	sub	w0, w0, w26
	lsl	w0, w0, 9
	add	x0, x22, x0
	b	.L1583
.L1573:
	mov	w19, -1
	b	.L1549
	.size	FtlRead, .-FtlRead
	.align	2
	.global	sftl_read
	.type	sftl_read, %function
sftl_read:
	stp	x29, x30, [sp, -16]!
	mov	x3, x2
	mov	w2, w1
	mov	w1, w0
	add	x29, sp, 0
	mov	w0, 0
	bl	FtlRead
	ldp	x29, x30, [sp], 16
	ret
	.size	sftl_read, .-sftl_read
	.align	2
	.global	FtlWrite
	.type	FtlWrite, %function
FtlWrite:
	stp	x29, x30, [sp, -224]!
	and	w0, w0, 255
	cmp	w0, 16
	add	x29, sp, 0
	stp	x23, x24, [sp, 48]
	mov	w23, w1
	stp	x25, x26, [sp, 64]
	mov	w26, w2
	stp	x27, x28, [sp, 80]
	mov	x27, x3
	stp	x19, x20, [sp, 16]
	stp	x21, x22, [sp, 32]
	bne	.L1587
	mov	x2, x3
	mov	w1, w26
	add	w0, w23, 256
	bl	FtlVendorPartWrite
.L1586:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 224
	ret
.L1587:
	adrp	x19, .LANCHOR0
	add	x1, x19, :lo12:.LANCHOR0
	ldr	w0, [x1, 344]
	cmp	w23, w0
	bcs	.L1625
	cmp	w2, w0
	bhi	.L1625
	add	w2, w23, w2
	str	w2, [x29, 108]
	cmp	w0, w2
	bcc	.L1625
	adrp	x2, .LANCHOR2
	str	x2, [x29, 136]
	add	x3, x2, :lo12:.LANCHOR2
	ldr	w0, [x2, #:lo12:.LANCHOR2]
	cmn	w0, #1
	beq	.L1586
	mov	w0, 2048
	add	w2, w23, w26
	str	w0, [x1, 3968]
	sub	w2, w2, #1
	ldrh	w0, [x1, 262]
	add	x20, x1, 24
	udiv	w25, w23, w0
	udiv	w0, w2, w0
	str	w0, [x29, 144]
	sub	w0, w0, w25
	add	w24, w0, 1
	str	w0, [x29, 132]
	ldr	w0, [x1, 2652]
	add	w0, w0, w26
	str	w0, [x1, 2652]
	ldr	w0, [x1, 2636]
	add	w0, w0, w24
	str	w0, [x1, 2636]
	ldr	w0, [x3, 8]
	cbz	w0, .L1589
	ldrh	w0, [x20, 4]
	add	x1, x1, 80
	cmp	w0, 0
	csel	x20, x1, x20, eq
.L1589:
	adrp	x0, .LANCHOR1
	mov	w21, w25
	add	x0, x0, :lo12:.LANCHOR1
	add	x0, x0, 696
	str	x0, [x29, 160]
.L1590:
	cbnz	w24, .L1618
	ldr	w1, [x29, 132]
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, 0
	bl	rk_ftl_garbage_collect
	ldrh	w0, [x19, 228]
	cmp	w0, 5
	bls	.L1619
.L1621:
	mov	w0, 0
	b	.L1586
.L1618:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x20, 6]
	ldrh	w0, [x0, 236]
	cmp	w1, w0
	bcc	.L1591
	ldr	x1, [x29, 160]
	adrp	x0, .LC8
	mov	w2, 1041
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1591:
	ldrh	w0, [x20, 4]
	cbnz	w0, .L1592
	add	x0, x19, :lo12:.LANCHOR0
	add	x1, x0, 24
	cmp	x20, x1
	bne	.L1593
	add	x0, x0, 80
	ldrh	w1, [x0, 4]
	cbnz	w1, .L1594
	bl	allocate_new_data_superblock
	ldr	x0, [x29, 136]
	add	x0, x0, :lo12:.LANCHOR2
	str	wzr, [x0, 8]
.L1594:
	add	x20, x19, :lo12:.LANCHOR0
	add	x22, x20, 24
	add	x20, x20, 80
	mov	x0, x22
	bl	allocate_new_data_superblock
	ldr	x0, [x29, 136]
	add	x0, x0, :lo12:.LANCHOR2
	ldr	w0, [x0, 8]
	cmp	w0, 0
	csel	x20, x20, x22, ne
.L1595:
	ldrh	w0, [x20, 4]
	cbnz	w0, .L1592
	mov	x0, x20
	bl	allocate_new_data_superblock
.L1592:
	ldrh	w0, [x20, 4]
	ldrb	w1, [x20, 7]
	cmp	w0, w24
	csel	w0, w0, w24, ls
	lsl	w1, w1, 2
	cmp	w0, w1
	csel	w0, w0, w1, ls
	str	w0, [x29, 128]
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x20, 6]
	ldrh	w0, [x0, 236]
	cmp	w1, w0
	bcc	.L1596
	ldr	x1, [x29, 160]
	adrp	x0, .LC8
	mov	w2, 1074
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1596:
	str	xzr, [x29, 168]
.L1597:
	ldr	w1, [x29, 128]
	ldr	w22, [x29, 168]
	cmp	w22, w1
	bcc	.L1616
	mov	x22, x1
.L1598:
	add	x0, x19, :lo12:.LANCHOR0
	mov	x3, x20
	mov	w2, 0
	mov	w1, w22
	ldr	x0, [x0, 3448]
	bl	FtlProgPages
	cmp	w24, w22
	bcs	.L1617
	ldr	x1, [x29, 160]
	adrp	x0, .LC8
	mov	w2, 1152
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1617:
	sub	w24, w24, w22
	b	.L1590
.L1593:
	ldr	x0, [x29, 136]
	add	x0, x0, :lo12:.LANCHOR2
	str	wzr, [x0, 8]
	ldrh	w0, [x1, 4]
	cbnz	w0, .L1630
	mov	x0, x20
	bl	allocate_new_data_superblock
	b	.L1595
.L1630:
	mov	x20, x1
	b	.L1592
.L1616:
	ldrh	w0, [x20, 4]
	cbz	w0, .L1598
	add	x1, x29, 188
	mov	w2, 0
	mov	w0, w21
	bl	log2phys
	mov	x0, x20
	bl	get_new_active_ppa
	add	x3, x19, :lo12:.LANCHOR0
	ldr	x1, [x29, 168]
	str	x3, [x29, 152]
	lsl	x28, x1, 5
	ldr	x1, [x3, 3448]
	ldrh	w2, [x3, 316]
	add	x1, x1, x28
	str	w0, [x1, 4]
	mul	w22, w22, w2
	ldr	x0, [x3, 3448]
	and	x1, x22, 4294967292
	str	x1, [x29, 120]
	add	x0, x0, x28
	ldr	x1, [x3, 3528]
	ldr	x3, [x29, 120]
	str	x1, [x29, 112]
	add	x22, x1, x3
	str	w21, [x0, 24]
	str	x22, [x0, 16]
	mov	w1, 0
	mov	x0, x22
	bl	ftl_memset
	ldr	w0, [x29, 144]
	cmp	w21, w25
	ldr	x3, [x29, 152]
	ccmp	w21, w0, 4, ne
	bne	.L1599
	cmp	w21, w25
	bne	.L1600
	ldrh	w2, [x3, 262]
	udiv	w0, w23, w2
	msub	w0, w0, w2, w23
	str	w0, [x29, 148]
	sub	w2, w2, w0
	cmp	w2, w26
	csel	w0, w2, w26, ls
	str	w0, [x29, 152]
.L1601:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w2, [x29, 152]
	ldrh	w1, [x0, 262]
	cmp	w2, w1
	bne	.L1602
	ldr	x0, [x0, 3448]
	cmp	w21, w25
	add	x4, x0, x28
	bne	.L1603
	str	x27, [x4, 8]
.L1604:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x20, 6]
	ldrh	w0, [x0, 236]
	cmp	w1, w0
	bcc	.L1613
	ldr	x1, [x29, 160]
	adrp	x0, .LC8
	mov	w2, 1143
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1613:
	ldp	x1, x2, [x29, 112]
	mov	w0, -3947
	strh	w0, [x1, x2]
	add	x1, x19, :lo12:.LANCHOR0
	ldr	w0, [x1, 2664]
	stp	w0, w21, [x22, 4]
	add	w21, w21, 1
	add	w0, w0, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x1, 2664]
	ldr	w0, [x29, 188]
	str	w0, [x22, 12]
	ldrh	w0, [x20]
	strh	w0, [x22, 2]
	ldr	x0, [x29, 168]
	add	x0, x0, 1
	str	x0, [x29, 168]
	b	.L1597
.L1600:
	ldr	w0, [x29, 108]
	ldrh	w2, [x3, 262]
	msub	w2, w21, w2, w0
	and	w0, w2, 65535
	stp	wzr, w0, [x29, 148]
	b	.L1601
.L1603:
	ldr	w0, [x29, 152]
.L1637:
	mul	w0, w0, w21
	sub	w0, w0, w23
	lsl	w0, w0, 9
	add	x0, x27, x0
	str	x0, [x4, 8]
	b	.L1604
.L1602:
	ldr	x1, [x0, 3448]
	cmp	w21, w25
	add	x1, x1, x28
	bne	.L1605
	ldr	x0, [x0, 3480]
.L1636:
	str	x0, [x1, 8]
	ldr	w0, [x29, 188]
	cmn	w0, #1
	beq	.L1607
	add	x5, x19, :lo12:.LANCHOR0
	str	w0, [x29, 196]
	str	w21, [x29, 216]
	mov	w2, 0
	str	x5, [x29, 96]
	ldr	x0, [x5, 3448]
	add	x0, x0, x28
	ldp	x1, x0, [x0, 8]
	stp	x1, x0, [x29, 200]
	mov	w1, 1
	add	x0, x29, 192
	bl	FlashReadPages
	ldr	w3, [x29, 192]
	ldr	x5, [x29, 96]
	cmn	w3, #1
	bne	.L1608
	ldr	w0, [x5, 2784]
	mov	w2, w21
	add	w0, w0, 1
	str	w0, [x5, 2784]
	adrp	x0, .LC125
	add	x0, x0, :lo12:.LC125
	ldr	w1, [x22, 8]
	bl	sftl_printk
.L1611:
	ldr	w0, [x29, 152]
	cmp	w21, w25
	lsl	w2, w0, 9
	add	x0, x19, :lo12:.LANCHOR0
	bne	.L1612
	ldr	x0, [x0, 3448]
	mov	x1, x27
	add	x4, x0, x28
	ldr	w0, [x29, 148]
	ldr	x3, [x4, 8]
	lsl	w0, w0, 9
	add	x0, x3, x0
.L1638:
	bl	ftl_memcpy
	b	.L1604
.L1605:
	ldr	x0, [x0, 3488]
	b	.L1636
.L1608:
	ldr	w0, [x22, 8]
	cmp	w21, w0
	beq	.L1610
	ldr	w0, [x5, 2784]
	mov	w2, w21
	add	w0, w0, 1
	str	w0, [x5, 2784]
	adrp	x0, .LC126
	add	x0, x0, :lo12:.LC126
	ldr	w1, [x22, 8]
	bl	sftl_printk
.L1610:
	ldr	w0, [x22, 8]
	cmp	w21, w0
	beq	.L1611
	ldr	x1, [x29, 160]
	mov	w2, 1128
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	b	.L1611
.L1607:
	add	x1, x19, :lo12:.LANCHOR0
	ldr	x0, [x1, 3448]
	ldrh	w2, [x1, 314]
	mov	w1, 0
	add	x0, x0, x28
	ldr	x0, [x0, 8]
	bl	ftl_memset
	b	.L1611
.L1612:
	ldrh	w1, [x0, 262]
	ldr	x0, [x0, 3448]
	add	x4, x0, x28
	mul	w1, w1, w21
	sub	w1, w1, w23
	ldr	x0, [x4, 8]
	lsl	w1, w1, 9
	add	x1, x27, x1
	b	.L1638
.L1599:
	ldr	x0, [x3, 3448]
	add	x4, x0, x28
	ldrh	w0, [x3, 262]
	b	.L1637
.L1619:
	mov	w20, 256
	mov	w21, 65535
	mov	w22, 128
.L1622:
	ldrh	w0, [x19, 176]
	cmp	w0, w21
	bne	.L1620
	ldrh	w0, [x19, 226]
	cmp	w0, w21
	bne	.L1620
	mov	w0, 0
	bl	List_get_gc_head_node
	bl	FtlGcRefreshBlock
.L1620:
	mov	w1, 1
	strh	w22, [x19, 3226]
	mov	w0, w1
	strh	w22, [x19, 3224]
	bl	rk_ftl_garbage_collect
	mov	w1, 1
	mov	w0, 0
	bl	rk_ftl_garbage_collect
	ldrh	w0, [x19, 228]
	cmp	w0, 2
	bhi	.L1621
	subs	w20, w20, #1
	bne	.L1622
	b	.L1621
.L1625:
	mov	w0, -1
	b	.L1586
	.size	FtlWrite, .-FtlWrite
	.align	2
	.global	sftl_gc
	.type	sftl_gc, %function
sftl_gc:
	stp	x29, x30, [sp, -16]!
	mov	w1, 1
	mov	w0, w1
	add	x29, sp, 0
	bl	rk_ftl_garbage_collect
	ldp	x29, x30, [sp], 16
	ret
	.size	sftl_gc, .-sftl_gc
	.align	2
	.global	FtlLoadSysInfo
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	stp	x29, x30, [sp, -80]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x21, x22, [sp, 32]
	adrp	x22, .LANCHOR0
	add	x21, x22, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	add	x20, x21, 3720
	stp	x23, x24, [sp, 48]
	str	x25, [sp, 64]
	ldr	x0, [x21, 3456]
	str	x0, [x20, 8]
	ldr	x0, [x21, 3520]
	str	x0, [x20, 16]
	ldr	x0, [x21, 72]
	ldrh	w2, [x21, 244]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w0, [x21, 2696]
	mov	w1, 65535
	cmp	w0, w1
	bne	.L1642
.L1653:
	mov	w0, -1
.L1641:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldr	x25, [sp, 64]
	ldp	x29, x30, [sp], 80
	ret
.L1642:
	add	x23, x21, 2696
	mov	w1, 1
	bl	FtlGetLastWrittenPage
	sxth	w19, w0
	adrp	x1, .LANCHOR1
	add	w0, w0, 1
	add	x1, x1, :lo12:.LANCHOR1
	strh	w0, [x23, 2]
	ldrsh	w23, [x21, 2696]
	add	x24, x1, 712
.L1644:
	tbz	w19, #31, .L1650
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	mov	w2, 1467
	add	x1, x1, 712
	adrp	x0, .LC8
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
	b	.L1649
.L1650:
	orr	w0, w19, w23, lsl 10
	str	w0, [x20, 4]
	ldr	x0, [x21, 3456]
	mov	w2, 1
	str	x0, [x20, 8]
	mov	w1, w2
	mov	x0, x20
	bl	FlashReadPages
	ldr	x0, [x20, 16]
	ldr	w7, [x0, 12]
	cbz	w7, .L1645
	ldr	w0, [x20]
	cmn	w0, #1
	beq	.L1645
	ldrh	w1, [x21, 314]
	ldr	x0, [x20, 8]
	bl	js_hash
	cmp	w7, w0
	beq	.L1645
	ldrh	w4, [x21, 2700]
	mov	w6, w0
	mov	w5, w7
	adrp	x0, .LC127
	mov	w3, w19
	mov	w2, w23
	mov	x1, x24
	add	x0, x0, :lo12:.LC127
	bl	sftl_printk
	cbnz	w19, .L1646
	ldrh	w0, [x21, 2700]
	cmp	w23, w0
	beq	.L1646
	ldrh	w19, [x21, 308]
	sxth	w23, w0
.L1648:
	sub	w19, w19, #1
	sxth	w19, w19
	b	.L1644
.L1646:
	mov	w0, -1
	str	w0, [x20]
.L1645:
	ldr	w0, [x20]
	cmn	w0, #1
	beq	.L1648
	ldr	x0, [x21, 3456]
	ldr	w1, [x0]
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	cmp	w1, w0
	bne	.L1648
	ldr	x0, [x21, 3520]
	ldrh	w1, [x0]
	mov	w0, 61604
	cmp	w1, w0
	bne	.L1648
.L1649:
	add	x1, x22, :lo12:.LANCHOR0
	ldrh	w0, [x1, 244]
	ldrh	w1, [x1, 314]
	add	x0, x0, 24
	cmp	x1, x0, lsl 1
	bcs	.L1652
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 1469
	add	x1, x1, 712
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1652:
	add	x19, x22, :lo12:.LANCHOR0
	mov	w2, 48
	add	x20, x19, 2512
	mov	x0, x20
	ldr	x1, [x19, 3728]
	bl	ftl_memcpy
	ldr	x0, [x19, 72]
	ldrh	w2, [x19, 244]
	ldr	x1, [x19, 3728]
	lsl	w2, w2, 1
	add	x1, x1, 48
	bl	ftl_memcpy
	ldr	w1, [x19, 2512]
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	cmp	w1, w0
	bne	.L1653
	ldrh	w1, [x19, 2520]
	ldrb	w2, [x19, 2522]
	ldrh	w0, [x19, 258]
	strh	w1, [x19, 2702]
	cmp	w2, w0
	bne	.L1653
	ldrh	w0, [x19, 306]
	ldrh	w2, [x19, 262]
	ldrh	w3, [x19, 236]
	str	w1, [x19, 3972]
	mul	w0, w0, w1
	str	w0, [x19, 2624]
	mul	w0, w2, w0
	ldr	w2, [x19, 248]
	str	w0, [x19, 344]
	ldrh	w0, [x19, 366]
	cmp	w1, w2
	sub	w0, w2, w0
	sub	w0, w0, w1
	udiv	w0, w0, w3
	strh	w0, [x19, 2692]
	bls	.L1654
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	adrp	x0, .LC8
	mov	w2, 1491
	add	x1, x1, 712
	add	x0, x0, :lo12:.LC8
	bl	sftl_printk
.L1654:
	add	x0, x22, :lo12:.LANCHOR0
	mov	w3, -1
	ldrh	w2, [x0, 2528]
	ldrh	w4, [x0, 2526]
	strh	w4, [x0, 24]
	lsr	w5, w2, 6
	and	w2, w2, 63
	strb	w2, [x0, 30]
	ldrb	w2, [x0, 2523]
	strb	w2, [x0, 32]
	ldrh	w2, [x0, 2530]
	strh	w2, [x0, 80]
	ldrh	w2, [x0, 2532]
	strh	w5, [x0, 26]
	strh	w3, [x0, 176]
	lsr	w5, w2, 6
	and	w2, w2, 63
	strb	w2, [x0, 86]
	ldrb	w2, [x0, 2524]
	strb	w2, [x0, 88]
	ldrh	w2, [x0, 2534]
	strh	w2, [x0, 128]
	ldrh	w2, [x0, 2536]
	strh	w5, [x0, 82]
	strh	wzr, [x0, 178]
	lsr	w5, w2, 6
	and	w2, w2, 63
	strb	w2, [x0, 134]
	ldrb	w2, [x0, 2525]
	strb	w2, [x0, 136]
	ldr	w2, [x0, 2544]
	str	w2, [x0, 2668]
	strb	wzr, [x0, 182]
	strb	wzr, [x0, 184]
	strh	w5, [x0, 130]
	str	wzr, [x0, 2628]
	str	wzr, [x0, 2636]
	str	wzr, [x0, 2644]
	str	wzr, [x0, 2648]
	str	wzr, [x0, 2672]
	str	wzr, [x0, 2684]
	ldr	w1, [x0, 2552]
	ldr	w2, [x0, 2660]
	str	wzr, [x0, 2640]
	cmp	w1, w2
	bls	.L1655
	str	w1, [x0, 2660]
.L1655:
	add	x0, x22, :lo12:.LANCHOR0
	ldr	w1, [x0, 2548]
	ldr	w2, [x0, 2664]
	cmp	w1, w2
	bls	.L1656
	str	w1, [x0, 2664]
.L1656:
	mov	w0, 65535
	cmp	w4, w0
	beq	.L1657
	add	x0, x22, :lo12:.LANCHOR0
	add	x0, x0, 24
	bl	make_superblock
.L1657:
	add	x1, x22, :lo12:.LANCHOR0
	add	x0, x1, 80
	ldrh	w2, [x1, 80]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L1658
	bl	make_superblock
.L1658:
	add	x1, x22, :lo12:.LANCHOR0
	add	x0, x1, 128
	ldrh	w2, [x1, 128]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L1659
	bl	make_superblock
.L1659:
	add	x22, x22, :lo12:.LANCHOR0
	mov	w1, 65535
	add	x0, x22, 176
	ldrh	w2, [x22, 176]
	cmp	w2, w1
	beq	.L1660
	bl	make_superblock
.L1660:
	mov	w0, 0
	b	.L1641
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlMapTblRecovery
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	stp	x29, x30, [sp, -144]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x19, x0
	stp	x21, x22, [sp, 32]
	adrp	x21, .LANCHOR0
	stp	x23, x24, [sp, 48]
	mov	w20, 0
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
	ldrh	w25, [x0, 6]
	ldr	x23, [x0, 40]
	ldr	x24, [x0, 16]
	ldr	x0, [x0, 24]
	lsl	w2, w25, 2
	str	x0, [x29, 128]
	ldrh	w0, [x19, 8]
	str	w0, [x29, 136]
	mov	x0, x23
	bl	ftl_memset
	stp	wzr, wzr, [x19, 48]
	add	x0, x21, :lo12:.LANCHOR0
	ldr	x2, [x0, 3456]
	ldr	x22, [x0, 3520]
	str	x2, [x0, 3728]
	str	x22, [x0, 3736]
	mov	w0, -1
	strh	w0, [x19]
	strh	w0, [x19, 2]
	mov	w0, 1
	str	w0, [x19, 56]
	ldr	w0, [x29, 136]
	sub	w0, w0, #1
	str	w0, [x29, 116]
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	add	x0, x0, 728
	str	x0, [x29, 104]
.L1673:
	ldr	w0, [x29, 136]
	cmp	w20, w0
	bge	.L1692
	ldr	w0, [x29, 116]
	sxtw	x28, w20
	cmp	w20, w0
	bne	.L1674
	lsl	x0, x28, 1
	mov	w26, 0
	add	x1, x24, x0
	str	x1, [x29, 136]
	mov	w1, 1
	ldrh	w0, [x24, x0]
	adrp	x24, .LANCHOR1
	add	x24, x24, :lo12:.LANCHOR1
	add	x24, x24, 728
	bl	FtlGetLastWrittenPage
	sxth	w1, w0
	add	w0, w0, 1
	strh	w0, [x19, 2]
	ldr	x0, [x29, 128]
	strh	w20, [x19]
	str	w1, [x29, 120]
	ldr	w0, [x0, x28, lsl 2]
	add	x28, x21, :lo12:.LANCHOR0
	add	x27, x28, 3720
	str	w0, [x19, 48]
.L1675:
	ldr	w0, [x29, 120]
	add	w0, w0, 1
	cmp	w26, w0
	blt	.L1678
.L1692:
	add	x21, x21, :lo12:.LANCHOR0
	mov	x0, x19
	bl	ftl_free_no_use_map_blk
	ldrh	w1, [x19, 2]
	ldrh	w0, [x21, 308]
	cmp	w1, w0
	bne	.L1680
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L1680:
	mov	x0, x19
	bl	ftl_map_blk_gc
	mov	x0, x19
	bl	ftl_map_blk_gc
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 144
	ret
.L1678:
	ldr	x0, [x29, 136]
	mov	w2, 1
	mov	w1, w2
	ldrh	w0, [x0]
	orr	w0, w26, w0, lsl 10
	str	w0, [x27, 4]
	mov	x0, x27
	bl	FlashReadPages
	ldr	x0, [x27, 16]
	ldr	w8, [x0, 12]
	cbz	w8, .L1676
	ldr	w0, [x27]
	cmn	w0, #1
	beq	.L1676
	ldrh	w1, [x28, 314]
	ldr	x0, [x27, 8]
	bl	js_hash
	cmp	w8, w0
	beq	.L1676
	mov	w5, w0
	mov	w4, w8
	mov	w3, w26
	mov	w2, w20
	mov	x1, x24
	adrp	x0, .LC128
	add	x0, x0, :lo12:.LC128
	bl	sftl_printk
	mov	w0, -1
	str	w0, [x27]
.L1676:
	ldr	w0, [x27]
	cmn	w0, #1
	beq	.L1677
	ldrh	w0, [x22, 8]
	cmp	w25, w0
	bls	.L1677
	ldrh	w2, [x22]
	ldrh	w1, [x19, 4]
	cmp	w2, w1
	bne	.L1677
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x27, 4]
	str	w1, [x23, x0]
.L1677:
	add	w7, w26, 1
	sxth	w26, w7
	b	.L1675
.L1674:
	add	x26, x21, :lo12:.LANCHOR0
	lsl	x2, x28, 1
	add	x0, x26, 3720
	ldr	x1, [x26, 3456]
	str	x1, [x0, 8]
	add	x1, x24, x2
	str	x1, [x29, 120]
	ldrh	w1, [x24, x2]
	ldrh	w2, [x26, 308]
	sub	w2, w2, #1
	orr	w1, w2, w1, lsl 10
	mov	w2, 1
	str	w1, [x0, 4]
	mov	w1, w2
	bl	FlashReadPages
	ldr	w0, [x26, 3720]
	cmn	w0, #1
	beq	.L1694
	ldrh	w1, [x22]
	ldrh	w0, [x19, 4]
	cmp	w1, w0
	bne	.L1694
	ldrh	w1, [x22, 8]
	mov	w0, 64245
	cmp	w1, w0
	beq	.L1682
.L1694:
	add	x26, x21, :lo12:.LANCHOR0
	mov	w27, 0
	add	x28, x26, 3720
.L1683:
	ldrh	w0, [x26, 308]
	cmp	w27, w0
	bge	.L1690
	ldr	x0, [x29, 120]
	mov	w2, 1
	mov	w1, w2
	ldrh	w0, [x0]
	orr	w0, w27, w0, lsl 10
	str	w0, [x28, 4]
	mov	x0, x28
	bl	FlashReadPages
	ldr	x0, [x28, 16]
	ldr	w7, [x0, 12]
	cbz	w7, .L1687
	ldr	w0, [x28]
	cmn	w0, #1
	beq	.L1687
	ldrh	w1, [x26, 314]
	ldr	x0, [x28, 8]
	bl	js_hash
	cmp	w7, w0
	beq	.L1687
	ldr	x1, [x29, 104]
	mov	w5, w0
	mov	w4, w7
	mov	w3, w27
	mov	w2, w20
	adrp	x0, .LC129
	add	x0, x0, :lo12:.LC129
	bl	sftl_printk
	mov	w0, -1
	str	w0, [x28]
.L1687:
	ldr	w0, [x28]
	cmn	w0, #1
	beq	.L1688
	ldrh	w0, [x22, 8]
	cmp	w25, w0
	bls	.L1688
	ldrh	w2, [x22]
	ldrh	w1, [x19, 4]
	cmp	w2, w1
	bne	.L1688
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x28, 4]
	str	w1, [x23, x0]
.L1688:
	add	w6, w27, 1
	sxth	w27, w6
	b	.L1683
.L1682:
	mov	w0, 0
.L1684:
	ldrh	w1, [x26, 308]
	sub	w1, w1, #1
	cmp	w0, w1
	blt	.L1686
.L1690:
	add	w20, w20, 1
	sxth	w20, w20
	b	.L1673
.L1686:
	ldr	x2, [x26, 3456]
	sbfiz	x3, x0, 3, 32
	ldrh	w1, [x2, x3]
	cmp	w25, w1
	bls	.L1685
	add	x2, x2, x3
	ubfiz	x1, x1, 2, 16
	ldr	w2, [x2, 4]
	str	w2, [x23, x1]
.L1685:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L1684
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	2
	.global	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	stp	x29, x30, [sp, -16]!
	add	x0, x1, 3888
	add	x29, sp, 0
	ldrh	w2, [x1, 324]
	strh	w2, [x0, 10]
	mov	w2, -3962
	strh	w2, [x0, 4]
	ldrh	w2, [x1, 348]
	strh	w2, [x0, 8]
	ldrh	w2, [x1, 326]
	strh	w2, [x0, 6]
	ldr	x2, [x1, 352]
	str	x2, [x0, 16]
	ldr	x2, [x1, 3592]
	str	x2, [x0, 24]
	ldr	x2, [x1, 3584]
	ldr	x1, [x1, 3600]
	stp	x2, x1, [x0, 32]
	bl	FtlMapTblRecovery
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FtlLoadMapInfo
	.type	FtlLoadMapInfo, %function
FtlLoadMapInfo:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FtlL2PDataInit
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, 3640
	bl	FtlMapTblRecovery
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	stp	x21, x22, [sp, 32]
	add	x21, x20, :lo12:.LANCHOR0
	str	x23, [sp, 48]
	ldrh	w0, [x21, 240]
	strh	wzr, [x21, 3752]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldrh	w1, [x21, 2696]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L1719
.L1721:
	mov	w23, -1
.L1718:
	mov	w0, w23
	ldr	x23, [sp, 48]
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 64
	ret
.L1719:
	bl	FtlLoadSysInfo
	mov	w23, w0
	cbnz	w0, .L1721
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	w0, 1
	bl	FtlUpdateVaildLpn
	ldr	x1, [x21, 2608]
	mov	w0, 0
	ldrh	w3, [x21, 342]
	add	x1, x1, 4
.L1722:
	cmp	w0, w3
	bge	.L1727
	ldr	w2, [x1], 16
	tbz	w2, #31, .L1723
.L1727:
	add	x2, x20, :lo12:.LANCHOR0
	cmp	w0, w3
	ldrh	w1, [x2, 2540]
	add	w1, w1, 1
	strh	w1, [x2, 2540]
	bge	.L1733
.L1724:
	add	x19, x20, :lo12:.LANCHOR0
	add	x22, x19, 24
	add	x21, x19, 80
	mov	x0, x22
	bl	FtlSuperblockPowerLostFix
	mov	x0, x21
	bl	FtlSuperblockPowerLostFix
	ldrh	w0, [x19, 24]
	ldr	x2, [x19, 72]
	ldrh	w3, [x19, 28]
	lsl	x0, x0, 1
	ldrh	w1, [x2, x0]
	sub	w1, w1, w3
	strh	w1, [x2, x0]
	strb	wzr, [x19, 30]
	ldrh	w0, [x19, 306]
	strh	w0, [x19, 26]
	ldrh	w0, [x19, 80]
	ldr	x2, [x19, 72]
	strh	wzr, [x19, 28]
	lsl	x0, x0, 1
	ldrh	w3, [x19, 84]
	ldrh	w1, [x2, x0]
	sub	w1, w1, w3
	strh	w1, [x2, x0]
	strb	wzr, [x19, 86]
	ldrh	w0, [x19, 306]
	strh	w0, [x19, 82]
	ldrh	w0, [x19, 2542]
	strh	wzr, [x19, 84]
	add	w0, w0, 1
	strh	w0, [x19, 2542]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
	b	.L1728
.L1723:
	add	w0, w0, 1
	b	.L1722
.L1733:
	ldrh	w0, [x2, 3752]
	cbnz	w0, .L1724
.L1728:
	add	x19, x20, :lo12:.LANCHOR0
	mov	w1, 65535
	add	x21, x19, 24
	ldrh	w0, [x19, 24]
	cmp	w0, w1
	beq	.L1729
	ldrh	w1, [x19, 28]
	cbnz	w1, .L1729
	ldrh	w1, [x19, 84]
	add	x20, x19, 80
	cbnz	w1, .L1729
	bl	FtlGcRefreshBlock
	ldrh	w0, [x19, 80]
	bl	FtlGcRefreshBlock
	bl	FtlVpcTblFlush
	mov	x0, x21
	bl	allocate_new_data_superblock
	mov	x0, x20
	bl	allocate_new_data_superblock
.L1729:
	bl	FtlVpcCheckAndModify
	b	.L1718
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	ftl_low_format
	.type	ftl_low_format, %function
ftl_low_format:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	add	x0, x20, :lo12:.LANCHOR0
	stp	x21, x22, [sp, 32]
	str	wzr, [x0, 2660]
	str	wzr, [x0, 2664]
	str	wzr, [x0, 2676]
	ldrh	w0, [x0, 240]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbz	w0, .L1736
	bl	FtlMakeBbt
.L1736:
	mov	w4, 23752
	add	x1, x20, :lo12:.LANCHOR0
	mov	w0, 0
	movk	w4, 0xa0f, lsl 16
.L1737:
	ldrh	w2, [x1, 262]
	cmp	w0, w2, lsl 7
	blt	.L1738
	ldrh	w21, [x1, 244]
	add	x22, x20, :lo12:.LANCHOR0
	mov	w19, 0
.L1739:
	ldrh	w0, [x22, 246]
	cmp	w0, w21
	bhi	.L1740
	ldrh	w0, [x22, 236]
	sub	w1, w19, #3
	cmp	w1, w0, lsl 1
	bge	.L1741
.L1745:
	add	x19, x20, :lo12:.LANCHOR0
	mov	w21, 0
	mov	w22, 0
.L1742:
	ldrh	w0, [x19, 244]
	cmp	w0, w22
	bhi	.L1746
	ldrh	w2, [x19, 236]
	ldr	w1, [x19, 248]
	ldrh	w0, [x19, 246]
	str	w0, [x19, 3428]
	udiv	w4, w1, w2
	ubfx	x3, x4, 5, 16
	str	w4, [x19, 2624]
	add	w0, w3, 36
	strh	w0, [x19, 2692]
	mov	w0, 24
	mul	w0, w2, w0
	cmp	w21, w0
	ble	.L1747
	sub	w1, w1, w21
	udiv	w1, w1, w2
	str	w1, [x19, 2624]
	lsr	w1, w1, 5
	add	w1, w1, 24
	strh	w1, [x19, 2692]
.L1747:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w1, [x0, 298]
	cbz	w1, .L1749
	ldrh	w5, [x0, 2692]
	add	w5, w5, w1, lsr 1
	strh	w5, [x0, 2692]
	mul	w5, w1, w2
	cmp	w21, w5
	bge	.L1749
	add	w1, w1, 32
	str	w4, [x0, 2624]
	add	w1, w3, w1
	strh	w1, [x0, 2692]
.L1749:
	add	x19, x20, :lo12:.LANCHOR0
	add	x21, x19, 24
	ldrh	w1, [x19, 2692]
	ldr	w0, [x19, 2624]
	sub	w0, w0, w1
	ldrh	w1, [x19, 306]
	mul	w0, w0, w2
	str	w0, [x19, 3972]
	mul	w0, w1, w0
	ldrh	w1, [x19, 262]
	str	w0, [x19, 2624]
	mul	w0, w1, w0
	str	w0, [x19, 344]
	bl	FtlBbmTblFlush
	ldr	x0, [x19, 72]
	mov	w1, 0
	ldrh	w2, [x19, 246]
	lsl	w2, w2, 1
	bl	ftl_memset
	mov	w0, -1
	mov	w22, w0
	mov	w1, 1
	strh	w0, [x19, 176]
	strh	wzr, [x19, 178]
	strb	wzr, [x19, 182]
	strb	wzr, [x19, 184]
	strh	wzr, [x21, 2]
	strb	wzr, [x21, 6]
	strh	wzr, [x19, 24]
	strb	w1, [x21, 8]
	str	wzr, [x19, 2620]
.L1751:
	mov	x0, x21
	bl	make_superblock
	ldrb	w1, [x21, 7]
	ldrh	w0, [x21]
	cbnz	w1, .L1752
	ldr	x1, [x19, 72]
	ubfiz	x0, x0, 1, 16
	strh	w22, [x1, x0]
	ldrh	w0, [x21]
	add	w0, w0, 1
	strh	w0, [x21]
	b	.L1751
.L1738:
	ldr	x5, [x1, 3480]
	ubfiz	x3, x0, 2, 16
	mvn	w2, w0
	orr	w2, w0, w2, lsl 16
	add	w0, w0, 1
	and	w0, w0, 65535
	str	w2, [x5, x3]
	ldr	x2, [x1, 3488]
	str	w4, [x2, x3]
	b	.L1737
.L1740:
	mov	w0, w21
	mov	w1, 1
	add	w21, w21, 1
	bl	FtlLowFormatEraseBlock
	add	w19, w19, w0
	and	w21, w21, 65535
	and	w19, w19, 65535
	b	.L1739
.L1741:
	udiv	w0, w19, w0
	ldr	w19, [x22, 336]
	add	x21, x20, :lo12:.LANCHOR0
	add	w0, w0, w19
	bl	FtlSysBlkNumInit
	ldrh	w0, [x22, 240]
	bl	FtlFreeSysBlkQueueInit
	ldrh	w19, [x22, 244]
.L1743:
	ldrh	w0, [x21, 246]
	cmp	w0, w19
	bls	.L1745
	mov	w0, w19
	add	w19, w19, 1
	mov	w1, 1
	and	w19, w19, 65535
	bl	FtlLowFormatEraseBlock
	b	.L1743
.L1746:
	mov	w0, w22
	mov	w1, 0
	add	w22, w22, 1
	bl	FtlLowFormatEraseBlock
	add	w21, w21, w0
	and	w22, w22, 65535
	and	w21, w21, 65535
	b	.L1742
.L1752:
	ldr	w1, [x19, 2660]
	ubfiz	x0, x0, 1, 16
	str	w1, [x21, 12]
	add	w1, w1, 1
	str	w1, [x19, 2660]
	ldr	x1, [x19, 72]
	ldrh	w2, [x21, 4]
	strh	w2, [x1, x0]
	add	x0, x19, 80
	strh	wzr, [x19, 82]
	ldrh	w1, [x21]
	mov	w21, -1
	strb	wzr, [x19, 86]
	add	w1, w1, 1
	strh	w1, [x19, 80]
	mov	w1, 1
	strb	w1, [x19, 88]
	add	x19, x20, :lo12:.LANCHOR0
	mov	x20, x0
.L1753:
	mov	x0, x20
	bl	make_superblock
	ldrb	w1, [x20, 7]
	ldrh	w0, [x20]
	cbnz	w1, .L1754
	ldr	x1, [x19, 72]
	ubfiz	x0, x0, 1, 16
	strh	w21, [x1, x0]
	ldrh	w0, [x20]
	add	w0, w0, 1
	strh	w0, [x20]
	b	.L1753
.L1754:
	ldr	w1, [x19, 2660]
	ubfiz	x0, x0, 1, 16
	str	w1, [x20, 12]
	add	w1, w1, 1
	str	w1, [x19, 2660]
	ldr	x1, [x19, 72]
	ldrh	w2, [x20, 4]
	mov	w20, -1
	strh	w2, [x1, x0]
	strh	w20, [x19, 128]
	bl	FtlFreeSysBlkQueueOut
	strh	w0, [x19, 2696]
	ldr	w0, [x19, 3972]
	strh	w0, [x19, 2702]
	ldr	w0, [x19, 2660]
	str	w0, [x19, 2704]
	add	w0, w0, 1
	strh	wzr, [x19, 2698]
	strh	w20, [x19, 2700]
	str	w0, [x19, 2660]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cbnz	w0, .L1755
	adrp	x0, .LANCHOR2
	mov	w1, 1
	str	w1, [x0, #:lo12:.LANCHOR2]
.L1755:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x29, x30, [sp], 48
	ret
	.size	ftl_low_format, .-ftl_low_format
	.align	2
	.global	sftl_init
	.type	sftl_init, %function
sftl_init:
	stp	x29, x30, [sp, -32]!
	adrp	x1, .LC0
	add	x1, x1, :lo12:.LC0
	mov	w0, -1
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR2
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	str	w0, [x20, #:lo12:.LANCHOR2]
	adrp	x0, .LC77
	add	x0, x0, :lo12:.LC77
	bl	sftl_printk
	mov	x0, x19
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldrh	w0, [x19, 240]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbnz	w0, .L1764
	bl	FtlSysBlkInit
	cbnz	w0, .L1764
	mov	w0, 1
	str	w0, [x20, #:lo12:.LANCHOR2]
	ldrh	w0, [x19, 228]
	cmp	w0, 15
	bhi	.L1764
	mov	w19, 8129
.L1763:
	mov	w1, 1
	mov	w0, 0
	bl	rk_ftl_garbage_collect
	subs	w19, w19, #1
	bne	.L1763
.L1764:
	mov	w0, 0
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
	.size	sftl_init, .-sftl_init
	.align	2
	.global	FtlWriteToIDB
	.type	FtlWriteToIDB, %function
FtlWriteToIDB:
	stp	x29, x30, [sp, -208]!
	add	x29, sp, 0
	stp	x23, x24, [sp, 48]
	add	w23, w1, w0
	stp	x19, x20, [sp, 16]
	sub	w24, w23, #1
	stp	x21, x22, [sp, 32]
	cmp	w24, 63
	stp	x25, x26, [sp, 64]
	adrp	x20, .LANCHOR0
	stp	x27, x28, [sp, 80]
	bls	.L1768
	mov	w19, w0
	cmp	w0, 575
	bls	.L1769
.L1768:
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w1, [x0, 3976]
	cbnz	w1, .L1770
.L1834:
	mov	w19, 0
	b	.L1767
.L1770:
	ldr	x24, [x0, 3984]
	mov	w1, 35899
	movk	w1, 0xfcdc, lsl 16
	ldr	w2, [x24]
	cmp	w2, w1
	bne	.L1772
	add	x1, x24, 258048
	ldrh	w2, [x0, 10]
	add	x1, x1, 2044
	mov	w0, 0
	mov	w19, 65023
	mov	w3, 4097
.L1776:
	ldr	w4, [x1]
	cbnz	w4, .L1773
	ldr	w4, [x24, w0, uxtw 2]
	add	w0, w0, 1
	str	w4, [x1, 2048]
	cmp	w0, w3
	sub	w19, w19, #1
	csel	w0, w0, wzr, cc
	sub	x1, x1, #4
	cmp	w19, 4096
	bne	.L1776
	mov	w19, 512
	b	.L1775
.L1773:
	add	w19, w19, 127
	lsr	w19, w19, 7
.L1775:
	ubfiz	w0, w2, 2, 14
	add	w21, w19, 4
	mov	w1, w19
	add	x27, x20, :lo12:.LANCHOR0
	mov	w25, 0
	mov	w22, 0
	udiv	w21, w21, w0
	adrp	x0, .LC130
	add	x0, x0, :lo12:.LC130
	add	w21, w21, 1
	mov	w2, w21
	bl	sftl_printk
	lsl	w0, w19, 7
	str	w0, [x29, 124]
.L1777:
	add	w0, w21, w22
	str	w0, [x29, 128]
	cmp	w0, 8
	bls	.L1799
	cbnz	w25, .L1800
.L1772:
	mov	w19, -1
.L1800:
	add	x20, x20, :lo12:.LANCHOR0
	ldr	x0, [x20, 3984]
	str	wzr, [x20, 3976]
	bl	kfree
	ldr	x0, [x20, 3992]
	bl	kfree
.L1767:
	mov	w0, w19
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 208
	ret
.L1799:
	ldr	x0, [x27, 3992]
	mov	w1, 0
	mov	x2, 512
	bl	memset
	ldrh	w28, [x27, 10]
	ldr	x1, [x27, 3392]
	mul	w0, w22, w28
	str	w0, [x29, 136]
	cbz	x1, .L1807
	ldr	x0, [x27, 3400]
	mov	w23, 6
	cmp	x0, 0
	mov	w0, 9
	csel	w23, w23, w0, eq
.L1778:
	mul	w4, w22, w28
	mov	w26, 0
.L1779:
	ldr	x6, [x27, 3368]
	mov	w1, w4
	str	w4, [x29, 132]
	mov	w0, 0
	add	w26, w26, 1
	blr	x6
	ldr	w4, [x29, 132]
	cmp	w21, w26
	add	w4, w4, w28
	bhi	.L1779
	cmp	w23, 9
	bne	.L1809
	ldr	x26, [x27, 3992]
	mov	w2, 1024
	mov	w1, 0
	mov	x0, x26
	bl	ftl_memset
	mov	w0, 18766
	mov	w1, 12
	movk	w0, 0x464e, lsl 16
	stp	w0, w1, [x26]
	mov	w0, 4
	strb	w0, [x26, 17]
	ldrh	w0, [x27, 10]
	strh	w0, [x26, 18]
	strb	wzr, [x26, 16]
	mov	w0, 16
	strb	wzr, [x26, 20]
	strb	w0, [x26, 21]
	add	x0, x26, 12
	strh	wzr, [x26, 22]
	str	wzr, [x26, 12]
	bl	js_hash
	str	w0, [x26, 8]
.L1780:
	mul	w0, w21, w28
	mov	x7, x24
	mov	x28, 0
	str	w0, [x29, 132]
.L1781:
	ldr	w0, [x29, 132]
	mov	w1, w28
	cmp	w28, w0
	bcs	.L1788
	cmp	w23, 9
	bne	.L1782
	add	w0, w1, 1
.L1832:
	str	w0, [x29, 144]
	mov	w0, 61424
	str	w0, [x29, 148]
	cmp	w1, 0
	ccmp	w23, 9, 0, eq
	bne	.L1784
	ldr	x1, [x27, 3392]
	add	x8, x27, 3360
	str	x7, [x29, 96]
	mov	w0, 70
	str	x8, [x29, 112]
	blr	x1
	ldr	x8, [x29, 112]
	add	x3, x29, 144
	ldr	w1, [x29, 136]
	mov	x2, x26
	str	x8, [x29, 104]
	mov	w0, 0
	ldr	x9, [x8, 16]
	blr	x9
	mov	w1, w0
	ldr	x8, [x29, 104]
	ldrb	w0, [x27, 22]
	str	w1, [x29, 112]
	ldr	x2, [x8, 32]
	blr	x2
	ldr	w1, [x29, 112]
	ldr	x7, [x29, 96]
	cmn	w1, #1
	bne	.L1785
.L1788:
	ldrb	w0, [x27, 14]
	str	w0, [x29, 112]
	ldrh	w0, [x27, 10]
	ldr	x28, [x27, 3992]
	mul	w1, w22, w0
	str	w1, [x29, 136]
	add	x1, x27, 3360
	ldr	x2, [x1, 32]
	cbnz	x2, .L1786
	mov	w1, 6
.L1833:
	mul	w0, w21, w0
	mov	x26, 0
	str	w0, [x29, 104]
	str	w1, [x29, 132]
.L1790:
	ldr	w0, [x29, 104]
	mov	w1, w26
	cmp	w0, w26
	bls	.L1794
	ldr	w0, [x29, 132]
	cmp	w26, 0
	ccmp	w0, 9, 0, eq
	bne	.L1791
	ldr	x1, [x27, 3392]
	mov	w0, 70
	blr	x1
	ldr	x1, [x27, 3400]
	mov	w0, 2
	blr	x1
	ldr	x8, [x27, 3384]
	add	x3, x29, 144
	ldr	w1, [x29, 136]
	mov	x2, x28
	mov	w0, 0
	blr	x8
	ldrb	w0, [x29, 112]
	ldr	x1, [x27, 3400]
	blr	x1
	ldrb	w0, [x27, 22]
	ldr	x1, [x27, 3392]
	blr	x1
	ldr	w0, [x28]
	mov	w1, 18766
	movk	w1, 0x464e, lsl 16
	cmp	w0, w1
	beq	.L1792
.L1794:
	ldr	x0, [x27, 3992]
	mov	x1, x24
	mov	w23, 0
	mov	x2, x0
.L1793:
	mov	x26, x2
	mov	x3, x1
	add	x2, x2, 4
	add	x1, x1, 4
	ldr	w4, [x3]
	ldr	w6, [x26]
	cmp	w6, w4
	beq	.L1796
	mov	x2, 512
	mov	w1, 0
	str	x3, [x29, 136]
	bl	memset
	ldr	x3, [x29, 136]
	mov	w1, w22
	ldr	w2, [x26]
	mov	w4, w23
	adrp	x0, .LC131
	add	x0, x0, :lo12:.LC131
	ldr	w3, [x3]
	bl	sftl_printk
	ldrh	w1, [x27, 10]
	mov	w0, 0
	ldr	x2, [x27, 3368]
	mul	w1, w1, w22
	blr	x2
.L1797:
	ldr	w22, [x29, 128]
	b	.L1777
.L1807:
	mov	w23, 6
	b	.L1778
.L1809:
	mov	x26, 0
	b	.L1780
.L1782:
	lsl	w0, w1, 2
	b	.L1832
.L1784:
	ldr	x8, [x27, 3376]
	add	x3, x29, 144
	ldr	w0, [x29, 136]
	mov	x2, x7
	str	x7, [x29, 112]
	add	w1, w1, w0
	mov	w0, 0
	blr	x8
	cmn	w0, #1
	beq	.L1788
	ldr	x7, [x29, 112]
	add	x7, x7, 2048
.L1785:
	add	x28, x28, 1
	b	.L1781
.L1786:
	ldr	x1, [x1, 40]
	mov	w23, 6
	cmp	x1, 0
	mov	w1, 9
	csel	w1, w23, w1, eq
	b	.L1833
.L1791:
	ldr	x7, [x27, 3384]
	add	x3, x29, 144
	ldr	w0, [x29, 136]
	mov	x2, x28
	add	w1, w0, w1
	mov	w0, 0
	blr	x7
	cmn	w0, #1
	beq	.L1794
	ldr	w0, [x29, 148]
	mov	w1, 61424
	cmp	w0, w1
	bne	.L1794
	add	x28, x28, 2048
.L1792:
	add	x26, x26, 1
	b	.L1790
.L1796:
	ldr	w3, [x29, 124]
	add	w23, w23, 1
	cmp	w23, w3
	bne	.L1793
	add	w25, w25, 1
	cmp	w25, 5
	bls	.L1797
	b	.L1800
.L1769:
	mov	w21, w1
	mov	x22, x2
	cmp	w0, 64
	bne	.L1801
	add	x25, x20, :lo12:.LANCHOR0
	mov	w0, 262144
	bl	ftl_malloc
	str	x0, [x25, 3984]
	mov	w0, 262144
	bl	ftl_malloc
	mov	x1, x0
	ldr	x0, [x25, 3984]
	str	x1, [x25, 3992]
	cbz	x0, .L1802
	cbz	x1, .L1802
	mov	w1, 1
	mov	w2, 262144
	str	w1, [x25, 3976]
	mov	w1, 0
	bl	ftl_memset
.L1801:
	add	x20, x20, :lo12:.LANCHOR0
	ldr	w0, [x20, 3976]
	cbz	w0, .L1834
	cmp	w19, 63
	ldr	x0, [x20, 3984]
	bhi	.L1804
	mov	w1, 64
	sub	w19, w1, w19
	sub	w21, w21, w19
	ubfiz	x19, x19, 9, 25
	add	x1, x22, x19
.L1805:
	cmp	w24, 575
	bls	.L1806
	sub	w21, w21, w23
	sub	w21, w21, #446
.L1806:
	lsl	w2, w21, 9
	bl	ftl_memcpy
	b	.L1834
.L1802:
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x1, x1, 752
	adrp	x0, .LC132
	add	x0, x0, :lo12:.LC132
	bl	sftl_printk
	b	.L1801
.L1804:
	lsl	w2, w19, 7
	mov	x1, x22
	sub	w2, w2, #8192
	add	x0, x0, x2, lsl 2
	b	.L1805
	.size	FtlWriteToIDB, .-FtlWriteToIDB
	.align	2
	.global	sftl_write
	.type	sftl_write, %function
sftl_write:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	w20, w1
	mov	w19, w0
	str	x21, [sp, 32]
	mov	x21, x2
	bl	FtlWriteToIDB
	mov	x3, x21
	mov	w2, w20
	mov	w1, w19
	mov	w0, 0
	bl	FtlWrite
	ldr	x21, [sp, 32]
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
	.size	sftl_write, .-sftl_write
	.align	2
	.global	rk_sftl_vendor_dev_ops_register
	.type	rk_sftl_vendor_dev_ops_register, %function
rk_sftl_vendor_dev_ops_register:
	adrp	x2, .LANCHOR0
	add	x2, x2, :lo12:.LANCHOR0
	ldr	x3, [x2, 4000]
	cbnz	x3, .L1839
	str	x0, [x2, 4000]
	mov	w0, 0
	str	x1, [x2, 4008]
	ret
.L1839:
	mov	w0, -1
	ret
	.size	rk_sftl_vendor_dev_ops_register, .-rk_sftl_vendor_dev_ops_register
	.align	2
	.global	rk_sftl_vendor_storage_init
	.type	rk_sftl_vendor_storage_init, %function
rk_sftl_vendor_storage_init:
	stp	x29, x30, [sp, -80]!
	mov	w0, 65536
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	adrp	x20, .LANCHOR0
	stp	x21, x22, [sp, 32]
	add	x22, x20, :lo12:.LANCHOR0
	stp	x23, x24, [sp, 48]
	stp	x25, x26, [sp, 64]
	bl	ftl_malloc
	str	x0, [x22, 4016]
	cbz	x0, .L1846
	mov	w26, 22084
	mov	w25, 0
	mov	w24, 0
	mov	w23, 0
	movk	w26, 0x524b, lsl 16
.L1844:
	ldr	x3, [x22, 4000]
	mov	w1, 128
	ldr	x2, [x22, 4016]
	lsl	w0, w23, 7
	blr	x3
	mov	w21, w0
	cbnz	w0, .L1842
	ldr	x19, [x22, 4016]
	ldr	w0, [x19]
	cmp	w0, w26
	bne	.L1843
	add	x0, x19, 61440
	ldr	w1, [x19, 4]
	ldr	w0, [x0, 4092]
	cmp	w0, w1
	bne	.L1843
	cmp	w0, w24
	bls	.L1843
	mov	w25, w23
	mov	w24, w0
.L1843:
	add	w23, w23, 1
	cmp	w23, 2
	bne	.L1844
	cbz	w24, .L1845
	add	x0, x20, :lo12:.LANCHOR0
	mov	x2, x19
	mov	w1, 128
	ldr	x3, [x0, 4000]
	lsl	w0, w25, 7
	blr	x3
	mov	w21, w0
	cbz	w0, .L1840
.L1842:
	add	x20, x20, :lo12:.LANCHOR0
	mov	w21, -1
	ldr	x0, [x20, 4016]
	bl	kfree
	str	xzr, [x20, 4016]
	b	.L1840
.L1845:
	mov	w1, 0
	mov	x2, 65536
	mov	x0, x19
	bl	memset
	mov	w1, 22084
	mov	w0, 1
	movk	w1, 0x524b, lsl 16
	stp	w1, w0, [x19]
	add	x1, x19, 61440
	str	w0, [x1, 4092]
	mov	w0, -1032
	strh	w0, [x19, 14]
.L1840:
	mov	w0, w21
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x29, x30, [sp], 80
	ret
.L1846:
	mov	w21, -12
	b	.L1840
	.size	rk_sftl_vendor_storage_init, .-rk_sftl_vendor_storage_init
	.align	2
	.global	rk_sftl_vendor_read
	.type	rk_sftl_vendor_read, %function
rk_sftl_vendor_read:
	adrp	x3, .LANCHOR0+4016
	ldr	x4, [x3, #:lo12:.LANCHOR0+4016]
	cbz	x4, .L1854
	ldrh	w6, [x4, 10]
	add	x5, x4, 16
	mov	w3, 0
.L1851:
	cmp	w3, w6
	bcc	.L1853
.L1854:
	mov	w0, -1
	ret
.L1853:
	ldrh	w7, [x5], 8
	cmp	w7, w0
	bne	.L1852
	stp	x29, x30, [sp, -32]!
	add	x3, x4, x3, uxtw 3
	mov	x0, x1
	add	x29, sp, 0
	str	x19, [sp, 16]
	ldrh	w19, [x3, 20]
	ldrh	w1, [x3, 18]
	cmp	w19, w2
	csel	w19, w19, w2, ls
	add	x1, x1, 1024
	uxtw	x2, w19
	add	x1, x4, x1
	bl	memcpy
	mov	w0, w19
	ldr	x19, [sp, 16]
	ldp	x29, x30, [sp], 32
	ret
.L1852:
	add	w3, w3, 1
	b	.L1851
	.size	rk_sftl_vendor_read, .-rk_sftl_vendor_read
	.align	2
	.global	rk_sftl_vendor_write
	.type	rk_sftl_vendor_write, %function
rk_sftl_vendor_write:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x23, x24, [sp, 48]
	adrp	x23, .LANCHOR0
	add	x3, x23, :lo12:.LANCHOR0
	stp	x19, x20, [sp, 16]
	stp	x21, x22, [sp, 32]
	stp	x25, x26, [sp, 64]
	stp	x27, x28, [sp, 80]
	ldr	x19, [x3, 4016]
	cbz	x19, .L1874
	add	w21, w2, 63
	ldrh	w3, [x19, 10]
	ldrh	w24, [x19, 8]
	mov	x4, x1
	mov	w28, w2
	and	w21, w21, -64
	add	x1, x19, 16
	mov	w20, 0
.L1861:
	cmp	w20, w3
	bcc	.L1869
	ldrh	w1, [x19, 14]
	cmp	w21, w1
	bhi	.L1874
	add	x3, x19, x3, uxth 3
	and	w21, w21, 65535
	uxtw	x2, w28
	strh	w0, [x3, 16]
	ldrh	w0, [x19, 12]
	strh	w0, [x3, 18]
	strh	w28, [x3, 20]
	add	w0, w0, w21
	sub	w21, w1, w21
	strh	w0, [x19, 12]
	strh	w21, [x19, 14]
	mov	x1, x4
	ldrh	w0, [x3, 18]
	add	x0, x0, 1024
	add	x0, x19, x0
	bl	memcpy
	ldrh	w0, [x19, 10]
	add	w0, w0, 1
	strh	w0, [x19, 10]
	b	.L1876
.L1869:
	ldrh	w6, [x1], 8
	cmp	w6, w0
	bne	.L1862
	uxtw	x27, w20
	add	x5, x19, 1024
	add	x22, x19, x27, lsl 3
	ldrh	w25, [x22, 20]
	add	w25, w25, 63
	and	w25, w25, -64
	cmp	w28, w25
	bls	.L1863
	ldrh	w0, [x19, 14]
	cmp	w21, w0
	bhi	.L1874
	add	x27, x27, 2
	ldrh	w22, [x22, 18]
	add	x27, x19, x27, lsl 3
	sub	w26, w3, #1
.L1864:
	cmp	w20, w26
	bcc	.L1865
	add	x20, x19, x20, uxtw 3
	and	w22, w22, 65535
	add	x0, x5, x22, uxth
	uxtw	x2, w28
	mov	x1, x4
	and	w21, w21, 65535
	strh	w22, [x20, 18]
	add	w22, w22, w21
	strh	w6, [x20, 16]
	strh	w28, [x20, 20]
	bl	memcpy
	strh	w22, [x19, 12]
	ldrh	w0, [x19, 14]
	sub	w21, w0, w21
	add	w25, w21, w25
	strh	w25, [x19, 14]
.L1876:
	ldr	w0, [x19, 4]
	add	x1, x19, 61440
	add	x23, x23, :lo12:.LANCHOR0
	mov	x2, x19
	add	w0, w0, 1
	str	w0, [x19, 4]
	str	w0, [x1, 4092]
	mov	w1, 128
	ldr	x3, [x23, 4008]
	ldrh	w0, [x19, 8]
	add	w0, w0, 1
	and	w0, w0, 65535
	cmp	w0, 1
	csel	w0, w0, wzr, ls
	strh	w0, [x19, 8]
	lsl	w0, w24, 7
	blr	x3
	mov	w0, 0
.L1859:
	ldp	x19, x20, [sp, 16]
	ldp	x21, x22, [sp, 32]
	ldp	x23, x24, [sp, 48]
	ldp	x25, x26, [sp, 64]
	ldp	x27, x28, [sp, 80]
	ldp	x29, x30, [sp], 128
	ret
.L1865:
	add	w20, w20, 1
	str	x4, [x29, 104]
	add	x0, x19, x20, uxtw 3
	str	x5, [x29, 120]
	add	x27, x27, 8
	ldrh	w1, [x0, 16]
	strh	w1, [x27, -8]
	ldrh	w1, [x0, 20]
	strh	w1, [x27, -4]
	strh	w22, [x27, -6]
	ldrh	w3, [x0, 20]
	ldrh	w1, [x0, 18]
	add	x0, x5, x22, uxtw
	add	w3, w3, 63
	and	w3, w3, -64
	add	x1, x5, x1
	and	x2, x3, 131008
	stp	w6, w3, [x29, 112]
	bl	memcpy
	ldp	w6, w3, [x29, 112]
	ldr	x4, [x29, 104]
	add	w22, w22, w3
	ldr	x5, [x29, 120]
	b	.L1864
.L1863:
	ldrh	w0, [x22, 18]
	uxtw	x2, w28
	mov	x1, x4
	add	x0, x5, x0
	bl	memcpy
	strh	w28, [x22, 20]
	b	.L1876
.L1862:
	add	w20, w20, 1
	b	.L1861
.L1874:
	mov	w0, -1
	b	.L1859
	.size	rk_sftl_vendor_write, .-rk_sftl_vendor_write
	.align	2
	.global	rk_sftl_vendor_storage_ioctl
	.type	rk_sftl_vendor_storage_ioctl, %function
rk_sftl_vendor_storage_ioctl:
	stp	x29, x30, [sp, -48]!
	mov	w0, 4096
	add	x29, sp, 0
	stp	x19, x20, [sp, 16]
	mov	x20, x2
	str	x21, [sp, 32]
	mov	w21, w1
	bl	ftl_malloc
	cbz	x0, .L1894
	mov	w1, 30209
	mov	x19, x0
	movk	w1, 0x4004, lsl 16
	cmp	w21, w1
	beq	.L1880
	add	w1, w1, 1
	cmp	w21, w1
	beq	.L1881
.L1904:
	mov	x20, -14
	b	.L1879
.L1880:
#APP
// 21 "./arch/arm64/include/asm/current.h" 1
	mrs x21, sp_el0
// 0 "" 2
#NO_APP
	mov	x1, x20
	ldr	x2, [x21, 8]
#APP
// 413 "./arch/arm64/include/asm/uaccess.h" 1
	adds x1, x1, 8; ccmp x1, x2, #2, cc; cset x3, ls
// 0 "" 2
#NO_APP
	cbz	x3, .L1882
	mov	x2, 8
	mov	x1, x20
	bl	__arch_copy_from_user
	cbz	x0, .L1905
.L1883:
	mov	x2, x20
	mov	w1, 256
.L1909:
	adrp	x0, .LC133
	add	x0, x0, :lo12:.LC133
	bl	sftl_printk
	b	.L1904
.L1882:
	str	xzr, [x0]
	b	.L1883
.L1905:
	ldr	w1, [x19]
	mov	w0, 17745
	movk	w0, 0x5652, lsl 16
	cmp	w1, w0
	beq	.L1885
.L1886:
	mov	x20, -1
.L1879:
	mov	x0, x19
	bl	kfree
.L1877:
	mov	x0, x20
	ldr	x21, [sp, 32]
	ldp	x19, x20, [sp, 16]
	ldp	x29, x30, [sp], 48
	ret
.L1885:
	ldrh	w2, [x19, 6]
	add	x1, x19, 8
	ldrh	w0, [x19, 4]
	bl	rk_sftl_vendor_read
	cmn	w0, #1
	beq	.L1886
	strh	w0, [x19, 6]
	and	x2, x0, 65535
	add	x2, x2, 8
	mov	x0, x20
	ldr	x1, [x21, 8]
#APP
// 423 "./arch/arm64/include/asm/uaccess.h" 1
	adds x0, x0, x2; ccmp x0, x1, #2, cc; cset x3, ls
// 0 "" 2
#NO_APP
	cbz	x3, .L1904
	mov	x0, x20
	mov	x1, x19
	mov	x20, 0
	bl	__arch_copy_to_user
	cbz	x0, .L1879
	b	.L1904
.L1881:
#APP
// 21 "./arch/arm64/include/asm/current.h" 1
	mrs x21, sp_el0
// 0 "" 2
#NO_APP
	mov	x1, x20
	ldr	x2, [x21, 8]
#APP
// 413 "./arch/arm64/include/asm/uaccess.h" 1
	adds x1, x1, 8; ccmp x1, x2, #2, cc; cset x3, ls
// 0 "" 2
#NO_APP
	cbz	x3, .L1888
	mov	x2, 8
	mov	x1, x20
	bl	__arch_copy_from_user
	cbz	x0, .L1906
.L1889:
	mov	x2, x20
	mov	w1, 276
	b	.L1909
.L1888:
	str	xzr, [x0]
	b	.L1889
.L1906:
	ldr	w1, [x19]
	mov	w0, 17745
	movk	w0, 0x5652, lsl 16
	cmp	w1, w0
	bne	.L1886
	ldrh	w2, [x19, 6]
	cmp	w2, 4087
	bhi	.L1886
	add	w2, w2, 8
	mov	x0, x20
	sxtw	x2, w2
	ldr	x1, [x21, 8]
#APP
// 413 "./arch/arm64/include/asm/uaccess.h" 1
	adds x0, x0, x2; ccmp x0, x1, #2, cc; cset x3, ls
// 0 "" 2
#NO_APP
	cbz	x3, .L1891
	mov	x1, x20
	mov	x0, x19
	bl	__arch_copy_from_user
	cbz	x0, .L1907
.L1892:
	mov	x2, x20
	mov	w1, 283
	b	.L1909
.L1891:
	mov	w1, 0
	mov	x0, x19
	bl	memset
	b	.L1892
.L1907:
	ldrh	w2, [x19, 6]
	add	x1, x19, 8
	ldrh	w0, [x19, 4]
	bl	rk_sftl_vendor_write
	sxtw	x20, w0
	b	.L1879
.L1894:
	mov	x20, -1
	b	.L1877
	.size	rk_sftl_vendor_storage_ioctl, .-rk_sftl_vendor_storage_ioctl
	.align	2
	.global	rk_sftl_vendor_register
	.type	rk_sftl_vendor_register, %function
rk_sftl_vendor_register:
	stp	x29, x30, [sp, -16]!
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x29, sp, 0
	add	x0, x0, 16
	bl	misc_register
	ldp	x29, x30, [sp], 16
	ret
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
	.align	3
	.set	.LANCHOR1,. + 0
	.type	__func__.3806, %object
	.size	__func__.3806, 17
__func__.3806:
	.string	"INSERT_DATA_LIST"
	.zero	7
	.type	__func__.3801, %object
	.size	__func__.3801, 17
__func__.3801:
	.string	"INSERT_FREE_LIST"
	.zero	7
	.type	__func__.3837, %object
	.size	__func__.3837, 17
__func__.3837:
	.string	"List_remove_node"
	.zero	7
	.type	__func__.3869, %object
	.size	__func__.3869, 22
__func__.3869:
	.string	"List_update_data_list"
	.zero	2
	.type	__func__.3976, %object
	.size	__func__.3976, 22
__func__.3976:
	.string	"select_l2p_ram_region"
	.zero	2
	.type	__func__.4296, %object
	.size	__func__.4296, 16
__func__.4296:
	.string	"make_superblock"
	.type	__func__.4472, %object
	.size	__func__.4472, 19
__func__.4472:
	.string	"get_new_active_ppa"
	.zero	5
	.type	__func__.16187, %object
	.size	__func__.16187, 17
__func__.16187:
	.string	"FlashEraseBlocks"
	.zero	7
	.type	__func__.3911, %object
	.size	__func__.3911, 26
__func__.3911:
	.string	"ftl_map_blk_alloc_new_blk"
	.zero	6
	.type	__func__.3416, %object
	.size	__func__.3416, 11
__func__.3416:
	.string	"FtlMemInit"
	.zero	5
	.type	__func__.3643, %object
	.size	__func__.3643, 14
__func__.3643:
	.string	"FtlBbt2Bitmap"
	.zero	2
	.type	__func__.4317, %object
	.size	__func__.4317, 18
__func__.4317:
	.string	"SupperBlkListInit"
	.zero	6
	.type	__func__.16144, %object
	.size	__func__.16144, 15
__func__.16144:
	.string	"FlashReadPages"
	.zero	1
	.type	__func__.4088, %object
	.size	__func__.4088, 14
__func__.4088:
	.string	"FtlScanSysBlk"
	.zero	2
	.type	__func__.3686, %object
	.size	__func__.3686, 11
__func__.3686:
	.string	"FtlLoadBbt"
	.zero	5
	.type	__func__.16163, %object
	.size	__func__.16163, 15
__func__.16163:
	.string	"FlashProgPages"
	.zero	1
	.type	__func__.4444, %object
	.size	__func__.4444, 25
__func__.4444:
	.string	"allocate_data_superblock"
	.zero	7
	.type	__func__.4485, %object
	.size	__func__.4485, 16
__func__.4485:
	.string	"update_vpc_list"
	.type	__func__.4492, %object
	.size	__func__.4492, 20
__func__.4492:
	.string	"decrement_vpc_count"
	.zero	4
	.type	__func__.3937, %object
	.size	__func__.3937, 31
__func__.3937:
	.string	"Ftl_write_map_blk_to_last_page"
	.zero	1
	.type	__func__.3951, %object
	.size	__func__.3951, 16
__func__.3951:
	.string	"FtlMapWritePage"
	.type	__func__.3878, %object
	.size	__func__.3878, 16
__func__.3878:
	.string	"load_l2p_region"
	.type	__func__.3922, %object
	.size	__func__.3922, 15
__func__.3922:
	.string	"ftl_map_blk_gc"
	.zero	1
	.type	__func__.3993, %object
	.size	__func__.3993, 9
__func__.3993:
	.string	"log2phys"
	.zero	7
	.type	__func__.4205, %object
	.size	__func__.4205, 16
__func__.4205:
	.string	"FtlReUsePrevPpa"
	.type	__func__.4239, %object
	.size	__func__.4239, 22
__func__.4239:
	.string	"FtlRecoverySuperblock"
	.zero	2
	.type	__func__.4359, %object
	.size	__func__.4359, 14
__func__.4359:
	.string	"ftl_check_vpc"
	.zero	2
	.type	__func__.4066, %object
	.size	__func__.4066, 15
__func__.4066:
	.string	"FtlVpcTblFlush"
	.zero	1
	.type	__func__.4342, %object
	.size	__func__.4342, 21
__func__.4342:
	.string	"FtlVpcCheckAndModify"
	.zero	3
	.type	__func__.4465, %object
	.size	__func__.4465, 29
__func__.4465:
	.string	"allocate_new_data_superblock"
	.zero	3
	.type	__func__.3540, %object
	.size	__func__.3540, 13
__func__.3540:
	.string	"FtlProgPages"
	.zero	3
	.type	__func__.4562, %object
	.size	__func__.4562, 19
__func__.4562:
	.string	"FtlGcFreeTempBlock"
	.zero	5
	.type	__func__.4675, %object
	.size	__func__.4675, 23
__func__.4675:
	.string	"rk_ftl_garbage_collect"
	.zero	1
	.type	__func__.3568, %object
	.size	__func__.3568, 9
__func__.3568:
	.string	"FtlWrite"
	.zero	7
	.type	__func__.4137, %object
	.size	__func__.4137, 15
__func__.4137:
	.string	"FtlLoadSysInfo"
	.zero	1
	.type	__func__.4159, %object
	.size	__func__.4159, 18
__func__.4159:
	.string	"FtlMapTblRecovery"
	.zero	6
	.type	__func__.16306, %object
	.size	__func__.16306, 14
__func__.16306:
	.string	"FtlWriteToIDB"
	.zero	2
	.type	rk_sftl_vendor_storage_fops, %object
	.size	rk_sftl_vendor_storage_fops, 216
rk_sftl_vendor_storage_fops:
	.zero	64
	.xword	rk_sftl_vendor_storage_ioctl
	.xword	rk_sftl_vendor_storage_ioctl
	.zero	136
	.data
	.align	3
	.set	.LANCHOR2,. + 0
	.type	gFtlInitStatus, %object
	.size	gFtlInitStatus, 4
gFtlInitStatus:
	.word	-1
	.type	ftl_gc_temp_block_bops_scan_page_addr, %object
	.size	ftl_gc_temp_block_bops_scan_page_addr, 2
ftl_gc_temp_block_bops_scan_page_addr:
	.hword	-1
	.zero	2
	.type	power_up_flag, %object
	.size	power_up_flag, 4
power_up_flag:
	.word	1
	.zero	4
	.type	rkflash_vender_storage_dev, %object
	.size	rkflash_vender_storage_dev, 80
rkflash_vender_storage_dev:
	.word	255
	.zero	4
	.xword	.LC134
	.xword	rk_sftl_vendor_storage_fops
	.zero	56
	.bss
	.align	3
	.set	.LANCHOR0,. + 0
	.type	g_nand_phy_info, %object
	.size	g_nand_phy_info, 24
g_nand_phy_info:
	.zero	24
	.type	g_active_superblock, %object
	.size	g_active_superblock, 48
g_active_superblock:
	.zero	48
	.type	p_valid_page_count_table, %object
	.size	p_valid_page_count_table, 8
p_valid_page_count_table:
	.zero	8
	.type	g_buffer_superblock, %object
	.size	g_buffer_superblock, 48
g_buffer_superblock:
	.zero	48
	.type	g_gc_temp_superblock, %object
	.size	g_gc_temp_superblock, 48
g_gc_temp_superblock:
	.zero	48
	.type	g_gc_superblock, %object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.zero	48
	.type	g_gc_next_blk_1, %object
	.size	g_gc_next_blk_1, 2
g_gc_next_blk_1:
	.zero	2
	.type	g_gc_next_blk, %object
	.size	g_gc_next_blk, 2
g_gc_next_blk:
	.zero	2
	.type	g_num_free_superblocks, %object
	.size	g_num_free_superblocks, 2
g_num_free_superblocks:
	.zero	2
	.zero	2
	.type	c_ftl_nand_sys_blks_per_plane, %object
	.size	c_ftl_nand_sys_blks_per_plane, 4
c_ftl_nand_sys_blks_per_plane:
	.zero	4
	.type	c_ftl_nand_planes_num, %object
	.size	c_ftl_nand_planes_num, 2
c_ftl_nand_planes_num:
	.zero	2
	.zero	2
	.type	c_ftl_nand_max_sys_blks, %object
	.size	c_ftl_nand_max_sys_blks, 4
c_ftl_nand_max_sys_blks:
	.zero	4
	.type	c_ftl_nand_data_blks_per_plane, %object
	.size	c_ftl_nand_data_blks_per_plane, 2
c_ftl_nand_data_blks_per_plane:
	.zero	2
	.type	c_ftl_nand_blk_pre_plane, %object
	.size	c_ftl_nand_blk_pre_plane, 2
c_ftl_nand_blk_pre_plane:
	.zero	2
	.type	c_ftl_nand_max_data_blks, %object
	.size	c_ftl_nand_max_data_blks, 4
c_ftl_nand_max_data_blks:
	.zero	4
	.type	c_ftl_nand_totle_phy_blks, %object
	.size	c_ftl_nand_totle_phy_blks, 4
c_ftl_nand_totle_phy_blks:
	.zero	4
	.type	c_ftl_nand_type, %object
	.size	c_ftl_nand_type, 2
c_ftl_nand_type:
	.zero	2
	.type	c_ftl_nand_die_num, %object
	.size	c_ftl_nand_die_num, 2
c_ftl_nand_die_num:
	.zero	2
	.type	c_ftl_nand_planes_per_die, %object
	.size	c_ftl_nand_planes_per_die, 2
c_ftl_nand_planes_per_die:
	.zero	2
	.type	c_ftl_nand_sec_pre_page, %object
	.size	c_ftl_nand_sec_pre_page, 2
c_ftl_nand_sec_pre_page:
	.zero	2
	.type	p_plane_order_table, %object
	.size	p_plane_order_table, 32
p_plane_order_table:
	.zero	32
	.type	c_mlc_erase_count_value, %object
	.size	c_mlc_erase_count_value, 2
c_mlc_erase_count_value:
	.zero	2
	.type	c_ftl_nand_ext_blk_pre_plane, %object
	.size	c_ftl_nand_ext_blk_pre_plane, 2
c_ftl_nand_ext_blk_pre_plane:
	.zero	2
	.type	c_ftl_vendor_part_size, %object
	.size	c_ftl_vendor_part_size, 2
c_ftl_vendor_part_size:
	.zero	2
	.type	c_ftl_nand_blks_per_die, %object
	.size	c_ftl_nand_blks_per_die, 2
c_ftl_nand_blks_per_die:
	.zero	2
	.type	c_ftl_nand_blks_per_die_shift, %object
	.size	c_ftl_nand_blks_per_die_shift, 2
c_ftl_nand_blks_per_die_shift:
	.zero	2
	.type	c_ftl_nand_page_pre_blk, %object
	.size	c_ftl_nand_page_pre_blk, 2
c_ftl_nand_page_pre_blk:
	.zero	2
	.type	c_ftl_nand_page_pre_slc_blk, %object
	.size	c_ftl_nand_page_pre_slc_blk, 2
c_ftl_nand_page_pre_slc_blk:
	.zero	2
	.type	c_ftl_nand_page_pre_super_blk, %object
	.size	c_ftl_nand_page_pre_super_blk, 2
c_ftl_nand_page_pre_super_blk:
	.zero	2
	.type	c_ftl_nand_sec_pre_page_shift, %object
	.size	c_ftl_nand_sec_pre_page_shift, 2
c_ftl_nand_sec_pre_page_shift:
	.zero	2
	.type	c_ftl_nand_byte_pre_page, %object
	.size	c_ftl_nand_byte_pre_page, 2
c_ftl_nand_byte_pre_page:
	.zero	2
	.type	c_ftl_nand_byte_pre_oob, %object
	.size	c_ftl_nand_byte_pre_oob, 2
c_ftl_nand_byte_pre_oob:
	.zero	2
	.type	c_ftl_nand_reserved_blks, %object
	.size	c_ftl_nand_reserved_blks, 2
c_ftl_nand_reserved_blks:
	.zero	2
	.type	DeviceCapacity, %object
	.size	DeviceCapacity, 4
DeviceCapacity:
	.zero	4
	.type	c_ftl_nand_max_vendor_blks, %object
	.size	c_ftl_nand_max_vendor_blks, 2
c_ftl_nand_max_vendor_blks:
	.zero	2
	.type	c_ftl_nand_vendor_region_num, %object
	.size	c_ftl_nand_vendor_region_num, 2
c_ftl_nand_vendor_region_num:
	.zero	2
	.type	c_ftl_nand_map_blks_per_plane, %object
	.size	c_ftl_nand_map_blks_per_plane, 2
c_ftl_nand_map_blks_per_plane:
	.zero	2
	.zero	2
	.type	c_ftl_nand_max_map_blks, %object
	.size	c_ftl_nand_max_map_blks, 4
c_ftl_nand_max_map_blks:
	.zero	4
	.type	c_ftl_nand_init_sys_blks_per_plane, %object
	.size	c_ftl_nand_init_sys_blks_per_plane, 4
c_ftl_nand_init_sys_blks_per_plane:
	.zero	4
	.type	c_ftl_nand_map_region_num, %object
	.size	c_ftl_nand_map_region_num, 2
c_ftl_nand_map_region_num:
	.zero	2
	.type	c_ftl_nand_l2pmap_ram_region_num, %object
	.size	c_ftl_nand_l2pmap_ram_region_num, 2
c_ftl_nand_l2pmap_ram_region_num:
	.zero	2
	.type	g_MaxLbaSector, %object
	.size	g_MaxLbaSector, 4
g_MaxLbaSector:
	.zero	4
	.type	g_totle_vendor_block, %object
	.size	g_totle_vendor_block, 2
g_totle_vendor_block:
	.zero	2
	.zero	2
	.type	p_vendor_block_table, %object
	.size	p_vendor_block_table, 8
p_vendor_block_table:
	.zero	8
	.type	gBbtInfo, %object
	.size	gBbtInfo, 96
gBbtInfo:
	.zero	96
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.zero	2056
	.type	g_sys_save_data, %object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.zero	48
	.type	p_data_block_list_table, %object
	.size	p_data_block_list_table, 8
p_data_block_list_table:
	.zero	8
	.type	p_data_block_list_head, %object
	.size	p_data_block_list_head, 8
p_data_block_list_head:
	.zero	8
	.type	p_data_block_list_tail, %object
	.size	p_data_block_list_tail, 8
p_data_block_list_tail:
	.zero	8
	.type	g_num_data_superblocks, %object
	.size	g_num_data_superblocks, 2
g_num_data_superblocks:
	.zero	2
	.zero	6
	.type	p_free_data_block_list_head, %object
	.size	p_free_data_block_list_head, 8
p_free_data_block_list_head:
	.zero	8
	.type	p_erase_count_table, %object
	.size	p_erase_count_table, 8
p_erase_count_table:
	.zero	8
	.type	p_l2p_ram_map, %object
	.size	p_l2p_ram_map, 8
p_l2p_ram_map:
	.zero	8
	.type	g_l2p_last_update_region_id, %object
	.size	g_l2p_last_update_region_id, 2
g_l2p_last_update_region_id:
	.zero	2
	.type	FtlUpdateVaildLpnCount, %object
	.size	FtlUpdateVaildLpnCount, 2
FtlUpdateVaildLpnCount:
	.zero	2
	.type	g_VaildLpn, %object
	.size	g_VaildLpn, 4
g_VaildLpn:
	.zero	4
	.type	g_MaxLpn, %object
	.size	g_MaxLpn, 4
g_MaxLpn:
	.zero	4
	.type	g_totle_read_page_count, %object
	.size	g_totle_read_page_count, 4
g_totle_read_page_count:
	.zero	4
	.type	g_totle_discard_page_count, %object
	.size	g_totle_discard_page_count, 4
g_totle_discard_page_count:
	.zero	4
	.type	g_totle_write_page_count, %object
	.size	g_totle_write_page_count, 4
g_totle_write_page_count:
	.zero	4
	.type	g_totle_cache_write_count, %object
	.size	g_totle_cache_write_count, 4
g_totle_cache_write_count:
	.zero	4
	.type	g_totle_l2p_write_count, %object
	.size	g_totle_l2p_write_count, 4
g_totle_l2p_write_count:
	.zero	4
	.type	g_totle_gc_page_count, %object
	.size	g_totle_gc_page_count, 4
g_totle_gc_page_count:
	.zero	4
	.type	g_totle_write_sector, %object
	.size	g_totle_write_sector, 4
g_totle_write_sector:
	.zero	4
	.type	g_totle_read_sector, %object
	.size	g_totle_read_sector, 4
g_totle_read_sector:
	.zero	4
	.type	g_GlobalSysVersion, %object
	.size	g_GlobalSysVersion, 4
g_GlobalSysVersion:
	.zero	4
	.type	g_GlobalDataVersion, %object
	.size	g_GlobalDataVersion, 4
g_GlobalDataVersion:
	.zero	4
	.type	g_totle_mlc_erase_count, %object
	.size	g_totle_mlc_erase_count, 4
g_totle_mlc_erase_count:
	.zero	4
	.type	g_totle_slc_erase_count, %object
	.size	g_totle_slc_erase_count, 4
g_totle_slc_erase_count:
	.zero	4
	.type	g_totle_avg_erase_count, %object
	.size	g_totle_avg_erase_count, 4
g_totle_avg_erase_count:
	.zero	4
	.type	g_totle_sys_slc_erase_count, %object
	.size	g_totle_sys_slc_erase_count, 4
g_totle_sys_slc_erase_count:
	.zero	4
	.type	g_max_erase_count, %object
	.size	g_max_erase_count, 4
g_max_erase_count:
	.zero	4
	.type	g_min_erase_count, %object
	.size	g_min_erase_count, 4
g_min_erase_count:
	.zero	4
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.zero	2
	.zero	2
	.type	gSysInfo, %object
	.size	gSysInfo, 16
gSysInfo:
	.zero	16
	.type	g_sys_ext_data, %object
	.size	g_sys_ext_data, 512
g_sys_ext_data:
	.zero	512
	.type	g_gc_free_blk_threshold, %object
	.size	g_gc_free_blk_threshold, 2
g_gc_free_blk_threshold:
	.zero	2
	.type	g_gc_merge_free_blk_threshold, %object
	.size	g_gc_merge_free_blk_threshold, 2
g_gc_merge_free_blk_threshold:
	.zero	2
	.type	g_gc_skip_write_count, %object
	.size	g_gc_skip_write_count, 4
g_gc_skip_write_count:
	.zero	4
	.type	g_gc_blk_index, %object
	.size	g_gc_blk_index, 2
g_gc_blk_index:
	.zero	2
	.zero	2
	.type	g_in_swl_replace, %object
	.size	g_in_swl_replace, 4
g_in_swl_replace:
	.zero	4
	.type	g_gc_num_req, %object
	.size	g_gc_num_req, 4
g_gc_num_req:
	.zero	4
	.zero	4
	.type	gp_gc_page_buf_info, %object
	.size	gp_gc_page_buf_info, 8
gp_gc_page_buf_info:
	.zero	8
	.type	p_gc_data_buf, %object
	.size	p_gc_data_buf, 8
p_gc_data_buf:
	.zero	8
	.type	p_gc_spare_buf, %object
	.size	p_gc_spare_buf, 8
p_gc_spare_buf:
	.zero	8
	.type	req_gc, %object
	.size	req_gc, 8
req_gc:
	.zero	8
	.type	c_gc_page_buf_num, %object
	.size	c_gc_page_buf_num, 4
c_gc_page_buf_num:
	.zero	4
	.type	g_gc_blk_num, %object
	.size	g_gc_blk_num, 2
g_gc_blk_num:
	.zero	2
	.zero	2
	.type	p_gc_blk_tbl, %object
	.size	p_gc_blk_tbl, 8
p_gc_blk_tbl:
	.zero	8
	.type	g_gc_page_offset, %object
	.size	g_gc_page_offset, 2
g_gc_page_offset:
	.zero	2
	.zero	6
	.type	p_gc_page_info, %object
	.size	p_gc_page_info, 8
p_gc_page_info:
	.zero	8
	.type	g_gc_bad_block_temp_num, %object
	.size	g_gc_bad_block_temp_num, 2
g_gc_bad_block_temp_num:
	.zero	2
	.zero	6
	.type	g_gc_bad_block_temp_tbl, %object
	.size	g_gc_bad_block_temp_tbl, 34
g_gc_bad_block_temp_tbl:
	.zero	34
	.type	g_gc_bad_block_gc_index, %object
	.size	g_gc_bad_block_gc_index, 2
g_gc_bad_block_gc_index:
	.zero	2
	.zero	4
	.type	g_nand_ops, %object
	.size	g_nand_ops, 48
g_nand_ops:
	.zero	48
	.type	req_erase, %object
	.size	req_erase, 8
req_erase:
	.zero	8
	.type	g_in_gc_progress, %object
	.size	g_in_gc_progress, 4
g_in_gc_progress:
	.zero	4
	.type	g_gc_head_data_block, %object
	.size	g_gc_head_data_block, 4
g_gc_head_data_block:
	.zero	4
	.type	g_gc_head_data_block_count, %object
	.size	g_gc_head_data_block_count, 4
g_gc_head_data_block_count:
	.zero	4
	.type	g_cur_erase_blk, %object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.zero	4
	.type	req_read, %object
	.size	req_read, 8
req_read:
	.zero	8
	.type	req_gc_dst, %object
	.size	req_gc_dst, 8
req_gc_dst:
	.zero	8
	.type	req_prgm, %object
	.size	req_prgm, 8
req_prgm:
	.zero	8
	.type	p_sys_data_buf, %object
	.size	p_sys_data_buf, 8
p_sys_data_buf:
	.zero	8
	.type	p_sys_data_buf_1, %object
	.size	p_sys_data_buf_1, 8
p_sys_data_buf_1:
	.zero	8
	.type	p_vendor_data_buf, %object
	.size	p_vendor_data_buf, 8
p_vendor_data_buf:
	.zero	8
	.type	p_io_data_buf_0, %object
	.size	p_io_data_buf_0, 8
p_io_data_buf_0:
	.zero	8
	.type	p_io_data_buf_1, %object
	.size	p_io_data_buf_1, 8
p_io_data_buf_1:
	.zero	8
	.type	sftl_nand_check_buf, %object
	.size	sftl_nand_check_buf, 8
sftl_nand_check_buf:
	.zero	8
	.type	sftl_temp_buf, %object
	.size	sftl_temp_buf, 8
sftl_temp_buf:
	.zero	8
	.type	sftl_nand_check_spare_buf, %object
	.size	sftl_nand_check_spare_buf, 8
sftl_nand_check_spare_buf:
	.zero	8
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 8
p_sys_spare_buf:
	.zero	8
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 8
p_io_spare_buf:
	.zero	8
	.type	g_ect_tbl_info_size, %object
	.size	g_ect_tbl_info_size, 2
g_ect_tbl_info_size:
	.zero	2
	.zero	6
	.type	p_swl_mul_table, %object
	.size	p_swl_mul_table, 8
p_swl_mul_table:
	.zero	8
	.type	gp_ect_tbl_info, %object
	.size	gp_ect_tbl_info, 8
gp_ect_tbl_info:
	.zero	8
	.type	p_valid_page_count_check_table, %object
	.size	p_valid_page_count_check_table, 8
p_valid_page_count_check_table:
	.zero	8
	.type	p_map_block_table, %object
	.size	p_map_block_table, 8
p_map_block_table:
	.zero	8
	.type	p_map_block_valid_page_count, %object
	.size	p_map_block_valid_page_count, 8
p_map_block_valid_page_count:
	.zero	8
	.type	p_vendor_block_valid_page_count, %object
	.size	p_vendor_block_valid_page_count, 8
p_vendor_block_valid_page_count:
	.zero	8
	.type	p_vendor_block_ver_table, %object
	.size	p_vendor_block_ver_table, 8
p_vendor_block_ver_table:
	.zero	8
	.type	p_vendor_region_ppn_table, %object
	.size	p_vendor_region_ppn_table, 8
p_vendor_region_ppn_table:
	.zero	8
	.type	p_map_region_ppn_table, %object
	.size	p_map_region_ppn_table, 8
p_map_region_ppn_table:
	.zero	8
	.type	p_map_block_ver_table, %object
	.size	p_map_block_ver_table, 8
p_map_block_ver_table:
	.zero	8
	.type	p_l2p_map_buf, %object
	.size	p_l2p_map_buf, 8
p_l2p_map_buf:
	.zero	8
	.type	c_ftl_nand_bbm_buf_size, %object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
	.zero	2
	.zero	6
	.type	gL2pMapInfo, %object
	.size	gL2pMapInfo, 64
gL2pMapInfo:
	.zero	64
	.type	g_totle_map_block, %object
	.size	g_totle_map_block, 2
g_totle_map_block:
	.zero	2
	.type	g_tmp_data_superblock_id, %object
	.size	g_tmp_data_superblock_id, 2
g_tmp_data_superblock_id:
	.zero	2
	.type	g_totle_swl_count, %object
	.size	g_totle_swl_count, 4
g_totle_swl_count:
	.zero	4
	.type	ftl_gc_temp_power_lost_recovery_flag, %object
	.size	ftl_gc_temp_power_lost_recovery_flag, 4
ftl_gc_temp_power_lost_recovery_flag:
	.zero	4
	.type	g_recovery_page_min_ver, %object
	.size	g_recovery_page_min_ver, 4
g_recovery_page_min_ver:
	.zero	4
	.type	req_sys, %object
	.size	req_sys, 32
req_sys:
	.zero	32
	.type	g_power_lost_recovery_flag, %object
	.size	g_power_lost_recovery_flag, 2
g_power_lost_recovery_flag:
	.zero	2
	.zero	2
	.type	g_recovery_page_num, %object
	.size	g_recovery_page_num, 4
g_recovery_page_num:
	.zero	4
	.type	g_recovery_ppa_tbl, %object
	.size	g_recovery_ppa_tbl, 128
g_recovery_ppa_tbl:
	.zero	128
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 64
gVendorBlkInfo:
	.zero	64
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.zero	2
	.zero	2
	.type	gc_discard_updated, %object
	.size	gc_discard_updated, 4
gc_discard_updated:
	.zero	4
	.type	gc_ink_free_return_value, %object
	.size	gc_ink_free_return_value, 2
gc_ink_free_return_value:
	.zero	2
	.type	g_gc_cur_blk_valid_pages, %object
	.size	g_gc_cur_blk_valid_pages, 2
g_gc_cur_blk_valid_pages:
	.zero	2
	.type	g_gc_cur_blk_max_valid_pages, %object
	.size	g_gc_cur_blk_max_valid_pages, 2
g_gc_cur_blk_max_valid_pages:
	.zero	2
	.zero	2
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.zero	4
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.zero	4
	.type	idb_need_write_back, %object
	.size	idb_need_write_back, 4
idb_need_write_back:
	.zero	4
	.zero	4
	.type	idb_buf, %object
	.size	idb_buf, 8
idb_buf:
	.zero	8
	.type	gp_flash_check_buf, %object
	.size	gp_flash_check_buf, 8
gp_flash_check_buf:
	.zero	8
	.type	_flash_read, %object
	.size	_flash_read, 8
_flash_read:
	.zero	8
	.type	_flash_write, %object
	.size	_flash_write, 8
_flash_write:
	.zero	8
	.type	g_vendor, %object
	.size	g_vendor, 8
g_vendor:
	.zero	8
	.type	check_vpc_table, %object
	.size	check_vpc_table, 16384
check_vpc_table:
	.zero	16384
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 8
gp_last_act_superblock:
	.zero	8
	.section	.rodata.str1.1,"aMS",@progbits,1
.LC0:
	.string	"SFTL version: 5.0.55 20200925"
.LC1:
	.string	"\n%s\n"
.LC2:
	.string	"act blk: %x %x %x %x %x %x\n"
.LC3:
	.string	"buf blk: %x %x %x %x %x %x\n"
.LC4:
	.string	"tmp blk: %x %x %x %x %x %x\n"
.LC5:
	.string	"gc blk: %x %x %x %x %x %x\n"
.LC6:
	.string	"free blk: %x %x %x\n"
.LC7:
	.string	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\n"
.LC8:
	.string	"\n!!!!! error @ func:%s - line:%d\n"
.LC9:
	.string	"FLASH INFO:\n"
.LC10:
	.string	"Device Capacity: %d MB\n"
.LC11:
	.string	"FTL INFO:\n"
.LC12:
	.string	"g_MaxLpn = 0x%x\n"
.LC13:
	.string	"g_VaildLpn = 0x%x\n"
.LC14:
	.string	"read_page_count = 0x%x\n"
.LC15:
	.string	"discard_page_count = 0x%x\n"
.LC16:
	.string	"write_page_count = 0x%x\n"
.LC17:
	.string	"cache_write_count = 0x%x\n"
.LC18:
	.string	"l2p_write_count = 0x%x\n"
.LC19:
	.string	"gc_page_count = 0x%x\n"
.LC20:
	.string	"totle_write = %d MB\n"
.LC21:
	.string	"totle_read = %d MB\n"
.LC22:
	.string	"GSV = 0x%x\n"
.LC23:
	.string	"GDV = 0x%x\n"
.LC24:
	.string	"bad blk num = %d\n"
.LC25:
	.string	"free_superblocks = 0x%x\n"
.LC26:
	.string	"mlc_EC = 0x%x\n"
.LC27:
	.string	"slc_EC = 0x%x\n"
.LC28:
	.string	"avg_EC = 0x%x\n"
.LC29:
	.string	"sys_EC = 0x%x\n"
.LC30:
	.string	"max_EC = 0x%x\n"
.LC31:
	.string	"min_EC = 0x%x\n"
.LC32:
	.string	"PLT = 0x%x\n"
.LC33:
	.string	"POT = 0x%x\n"
.LC34:
	.string	"MaxSector = 0x%x\n"
.LC35:
	.string	"init_sys_blks_pp = 0x%x\n"
.LC36:
	.string	"sys_blks_pp = 0x%x\n"
.LC37:
	.string	"free sysblock = 0x%x\n"
.LC38:
	.string	"data_blks_pp = 0x%x\n"
.LC39:
	.string	"data_op_blks_pp = 0x%x\n"
.LC40:
	.string	"max_data_blks = 0x%x\n"
.LC41:
	.string	"Sys.id = 0x%x\n"
.LC42:
	.string	"Bbt.id = 0x%x\n"
.LC43:
	.string	"ACT.page = 0x%x\n"
.LC44:
	.string	"ACT.plane = 0x%x\n"
.LC45:
	.string	"ACT.id = 0x%x\n"
.LC46:
	.string	"ACT.mode = 0x%x\n"
.LC47:
	.string	"ACT.a_pages = 0x%x\n"
.LC48:
	.string	"ACT VPC = 0x%x\n"
.LC49:
	.string	"BUF.page = 0x%x\n"
.LC50:
	.string	"BUF.plane = 0x%x\n"
.LC51:
	.string	"BUF.id = 0x%x\n"
.LC52:
	.string	"BUF.mode = 0x%x\n"
.LC53:
	.string	"BUF.a_pages = 0x%x\n"
.LC54:
	.string	"BUF VPC = 0x%x\n"
.LC55:
	.string	"TMP.page = 0x%x\n"
.LC56:
	.string	"TMP.plane = 0x%x\n"
.LC57:
	.string	"TMP.id = 0x%x\n"
.LC58:
	.string	"TMP.mode = 0x%x\n"
.LC59:
	.string	"TMP.a_pages = 0x%x\n"
.LC60:
	.string	"GC.page = 0x%x\n"
.LC61:
	.string	"GC.plane = 0x%x\n"
.LC62:
	.string	"GC.id = 0x%x\n"
.LC63:
	.string	"GC.mode = 0x%x\n"
.LC64:
	.string	"GC.a_pages = 0x%x\n"
.LC65:
	.string	"WR_CHK = %x %x %x\n"
.LC66:
	.string	"Read Err Cnt = 0x%x\n"
.LC67:
	.string	"Prog Err Cnt = 0x%x\n"
.LC68:
	.string	"gc_free_blk_th= 0x%x\n"
.LC69:
	.string	"gc_merge_free_blk_th= 0x%x\n"
.LC70:
	.string	"gc_skip_write_count= 0x%x\n"
.LC71:
	.string	"gc_blk_index= 0x%x\n"
.LC72:
	.string	"free min EC= 0x%x\n"
.LC73:
	.string	"free max EC= 0x%x\n"
.LC74:
	.string	"GC__SB VPC = 0x%x\n"
.LC75:
	.string	"%d. [0x%x]=0x%x 0x%x  0x%x\n"
.LC76:
	.string	"free %d. [0x%x] 0x%x  0x%x\n"
.LC77:
	.string	"%s\n"
.LC78:
	.string	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x\n"
.LC79:
	.string	"FtlGcRefreshBlock  0x%x\n"
.LC80:
	.string	"FtlGcMarkBadPhyBlk %d 0x%x\n"
.LC81:
	.string	"%s %p + 0x%x:"
.LC82:
	.string	"0x%08x,"
.LC83:
	.string	"0x%04x,"
.LC84:
	.string	"0x%02x,"
.LC85:
	.string	"\n"
.LC86:
	.string	"%s: addr: %x is in id block!!!!!!!!!!\n"
.LC87:
	.string	"not free: w: d:"
.LC88:
	.string	"not free: w: s:"
.LC89:
	.string	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\n"
.LC90:
	.string	"FtlFreeSysBlkQueueOut = %x, free count = %d\n"
.LC91:
	.string	"%s error allocating memory. return -1\n"
.LC92:
	.string	"FlashReadPages %x %x error_ecc_bits %d\n"
.LC93:
	.string	"data:"
.LC94:
	.string	"spare:"
.LC95:
	.string	"prog read error: = %x\n"
.LC96:
	.string	"prog read REFRESH: = %x\n"
.LC97:
	.string	"prog read s error: = %x %x %x\n"
.LC98:
	.string	"prog read d error: = %x %x %x\n"
.LC99:
	.string	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\n"
.LC100:
	.string	"FtlBbmTblFlush error:%x\n"
.LC101:
	.string	"FtlBbmTblFlush error = %x error count = %d\n"
.LC102:
	.string	"FtlGcFreeBadSuperBlk 0x%x\n"
.LC103:
	.string	"decrement_vpc_count %x = %d\n"
.LC104:
	.string	"FtlMapWritePage error = %x \n"
.LC105:
	.string	"FtlMapWritePage error = %x error count = %d\n"
.LC106:
	.string	"region_id = %x phyAddr = %x\n"
.LC107:
	.string	"map_ppn:"
.LC108:
	.string	"load_l2p_region refresh = %x phyAddr = %x\n"
.LC109:
	.string	"data prev_ppa = %x error...................\n"
.LC110:
	.string	"spuer block %x vpn is 0\n "
.LC111:
	.string	"...%s enter...\n"
.LC112:
	.string	"FtlCheckVpc2 %x = %x  %x\n"
.LC113:
	.string	"free blk vpc error %x = %x  %x\n"
.LC114:
	.string	"ftl_scan_all_data = %x\n"
.LC115:
	.string	"scan lpa = %x ppa= %x\n"
.LC116:
	.string	"lpa = %x,addr= %x,spare= %x %x %x %x data=%x %x\n"
.LC117:
	.string	"FtlGcScanTempBlk Error ID %x %x!!!!!!! \n"
.LC118:
	.string	"FtlVendorPartRead refresh = %x phyAddr = %x\n"
.LC119:
	.string	"no ect"
.LC120:
	.string	"FtlVpcTblFlush error = %x error count = %d\n"
.LC121:
	.string	"FtlCheckVpc %x = %x  %x\n"
.LC122:
	.string	"FtlProgPages error %x = %d\n"
.LC123:
	.string	"Ftlwrite decrement_vpc_count %x = %d\n"
.LC124:
	.string	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\n"
.LC125:
	.string	"FtlWrite: ecc error:%x %x %x\n"
.LC126:
	.string	"FtlWrite: lpa error:%x %x\n"
.LC127:
	.string	"%s hash error this.id =%x page =%x pre_id =%x hash =%x hash_r =%x\n"
.LC128:
	.string	"%s last blk_id =%x page =%x hash error hash =%x hash_r =%x\n"
.LC129:
	.string	"%s scan blk_id =%x page =%x hash error hash =%x hash_r =%x\n"
.LC130:
	.string	"write_idblock %x %x\n"
.LC131:
	.string	"write_idblock fail! %x %x %x %x\n"
.LC132:
	.string	"%s idb buffer alloc fail\n"
.LC133:
	.string	"copy_from_user error %d %p %p\n"
.LC134:
	.string	"vendor_storage"
