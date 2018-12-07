/*
 * Copyright (c) 2016-2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-01-07
 * function: rk ftl v5 for rockchip soc base on arm v8 to support MLC NAND.
 */
	.cpu generic+fp+simd
	.file	"rk_ftlv5_arm_v8.c"
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
	.type	FlashReadFacBbtData.part.6, %function
FlashReadFacBbtData.part.6:
	stp	x29, x30, [sp, -32]!
	mov	w3, 0
	adrp	x5, .LANCHOR0
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	w7, 1
	mov	x19, x0
	cbnz	w1, .L4
.L6:
	add	x0, x5, :lo12:.LANCHOR0
	ldr	w0, [x0,8]
	cmp	w3, w0
	bcs	.L4
	ubfx	x4, x3, 5, 11
	ldr	x0, [x5,#:lo12:.LANCHOR0]
	lsl	x4, x4, 2
	lsl	w6, w7, w3
	add	w3, w3, 1
	ldr	w1, [x0,x4]
	uxth	w3, w3
	orr	w6, w1, w6
	str	w6, [x0,x4]
	b	.L6
.L4:
	ldr	x1, [x5,#:lo12:.LANCHOR0]
	mov	x0, x19
	bl	ftl_memcpy
	mov	w2, 4
	adrp	x0, .LC0
	mov	x1, x19
	add	x0, x0, :lo12:.LC0
	mov	w3, w2
	bl	rknand_print_hex
	ldr	x19, [sp,16]
	mov	w0, 0
	ldp	x29, x30, [sp], 32
	ret
	.size	FlashReadFacBbtData.part.6, .-FlashReadFacBbtData.part.6
	.align	2
	.type	ftl_set_blk_mode.part.17, %function
ftl_set_blk_mode.part.17:
	adrp	x1, .LANCHOR0+16
	uxth	w0, w0
	ubfx	x2, x0, 5, 11
	lsl	x2, x2, 2
	ldr	x3, [x1,#:lo12:.LANCHOR0+16]
	mov	w1, 1
	lsl	w0, w1, w0
	ldr	w1, [x3,x2]
	orr	w0, w1, w0
	str	w0, [x3,x2]
	ret
	.size	ftl_set_blk_mode.part.17, .-ftl_set_blk_mode.part.17
	.align	2
	.global	FlashMemCmp8
	.type	FlashMemCmp8, %function
FlashMemCmp8:
	mov	x5, x0
	adrp	x0, .LANCHOR0+24
	ldrb	w0, [x0,#:lo12:.LANCHOR0+24]
	cbz	w0, .L15
	ldrb	w4, [x5,1]
	mov	w0, 0
	ldrb	w3, [x1,1]
	cmp	w4, w3
	beq	.L12
.L15:
	mov	x3, 0
.L13:
	cmp	w3, w2
	mov	w4, w3
	bcs	.L18
	ldrb	w0, [x5,x3]
	add	x3, x3, 1
	add	x6, x1, x3
	ldrb	w6, [x6,-1]
	cmp	w6, w0
	beq	.L13
	add	w0, w4, 1
	b	.L12
.L18:
	mov	w0, 0
.L12:
	ret
	.size	FlashMemCmp8, .-FlashMemCmp8
	.align	2
	.global	FlashRsvdBlkChk
	.type	FlashRsvdBlkChk, %function
FlashRsvdBlkChk:
	adrp	x2, .LANCHOR0
	uxtb	w0, w0
	add	x2, x2, :lo12:.LANCHOR0
	ldrb	w3, [x2,25]
	ldr	w2, [x2,28]
	mul	w2, w3, w2
	cmp	w1, w2
	cset	w1, cc
	cmp	w0, wzr
	cset	w0, eq
	and	w0, w1, w0
	eor	w0, w0, 1
	ret
	.size	FlashRsvdBlkChk, .-FlashRsvdBlkChk
	.align	2
	.global	FlashGetRandomizer
	.type	FlashGetRandomizer, %function
FlashGetRandomizer:
	stp	x29, x30, [sp, -32]!
	adrp	x2, .LANCHOR1
	and	w3, w1, 127
	add	x2, x2, :lo12:.LANCHOR1
	add	x29, sp, 0
	uxtb	w0, w0
	str	x19, [sp,16]
	ldrh	w19, [x2,w3,uxtw 1]
	adrp	x2, .LANCHOR0+32
	ldrb	w2, [x2,#:lo12:.LANCHOR0+32]
	cbz	w2, .L21
	bl	FlashRsvdBlkChk
	cmp	w0, wzr
	orr	w1, w19, -1073741824
	csel	w19, w1, w19, ne
.L21:
	mov	w0, w19
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FlashGetRandomizer, .-FlashGetRandomizer
	.align	2
	.global	FlashSetRandomizer
	.type	FlashSetRandomizer, %function
FlashSetRandomizer:
	stp	x29, x30, [sp, -48]!
	and	w2, w1, 127
	add	x29, sp, 0
	str	x21, [sp,32]
	uxtb	w21, w0
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	stp	x19, x20, [sp,16]
	ldrh	w20, [x0,w2,uxtw 1]
	adrp	x2, .LANCHOR0
	add	x0, x2, :lo12:.LANCHOR0
	mov	x19, x2
	ldrb	w0, [x0,32]
	cbz	w0, .L28
	mov	w0, w21
	bl	FlashRsvdBlkChk
	cmp	w0, wzr
	orr	w1, w20, -1073741824
	csel	w20, w1, w20, ne
.L28:
	add	x2, x19, :lo12:.LANCHOR0
	add	x2, x2, x21, sxtw 4
	ldr	x0, [x2,40]
	str	w20, [x0,336]
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashSetRandomizer, .-FlashSetRandomizer
	.align	2
	.global	FlashBlockAlignInit
	.type	FlashBlockAlignInit, %function
FlashBlockAlignInit:
	uxth	w0, w0
	adrp	x1, .LANCHOR0
	cmp	w0, 256
	add	x1, x1, :lo12:.LANCHOR0
	bls	.L35
	mov	w0, 512
	b	.L38
.L35:
	cmp	w0, 128
	bls	.L38
	mov	w0, 256
.L38:
	str	w0, [x1,28]
	ret
	.size	FlashBlockAlignInit, .-FlashBlockAlignInit
	.align	2
	.global	FlashReadCmd
	.type	FlashReadCmd, %function
FlashReadCmd:
	adrp	x3, .LANCHOR0
	uxtb	w0, w0
	add	x3, x3, :lo12:.LANCHOR0
	sbfiz	x5, x0, 4, 32
	add	x3, x3, 40
	lsr	w4, w1, 16
	add	x2, x3, x5
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldrb	w2, [x2,8]
	ldr	x3, [x3,x5]
	add	x2, x3, x2, lsl 8
	and	w3, w1, 255
	str	wzr, [x2,2056]
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
	str	w3, [x2,2052]
	lsr	w3, w1, 8
	str	w3, [x2,2052]
	mov	w3, 48
	str	w4, [x2,2052]
	str	w3, [x2,2056]
	bl	FlashSetRandomizer
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashReadCmd, .-FlashReadCmd
	.align	2
	.global	FlashReadDpDataOutCmd
	.type	FlashReadDpDataOutCmd, %function
FlashReadDpDataOutCmd:
	adrp	x3, .LANCHOR0
	uxtb	w0, w0
	add	x3, x3, :lo12:.LANCHOR0
	sbfiz	x2, x0, 4, 32
	add	x4, x3, 40
	stp	x29, x30, [sp, -16]!
	add	x5, x4, x2
	add	x29, sp, 0
	ldr	x6, [x4,x2]
	lsr	w4, w1, 8
	ldrb	w2, [x3,120]
	lsr	w3, w1, 16
	cmp	w2, 1
	ldrb	w2, [x5,8]
	and	w5, w1, 255
	add	x2, x6, x2, lsl 8
	bne	.L41
	mov	w6, 6
	str	w6, [x2,2056]
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
	str	w5, [x2,2052]
	str	w4, [x2,2052]
	str	w3, [x2,2052]
	b	.L43
.L41:
	str	wzr, [x2,2056]
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
	str	w5, [x2,2052]
	str	w4, [x2,2052]
	str	w3, [x2,2052]
	mov	w3, 5
	str	w3, [x2,2056]
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
.L43:
	mov	w3, 224
	str	w3, [x2,2056]
	bl	FlashSetRandomizer
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashReadDpDataOutCmd, .-FlashReadDpDataOutCmd
	.align	2
	.global	FlashProgFirstCmd
	.type	FlashProgFirstCmd, %function
FlashProgFirstCmd:
	adrp	x3, .LANCHOR0
	uxtb	w0, w0
	add	x3, x3, :lo12:.LANCHOR0
	sbfiz	x5, x0, 4, 32
	add	x3, x3, 40
	lsr	w4, w1, 16
	add	x2, x3, x5
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldrb	w2, [x2,8]
	ldr	x3, [x3,x5]
	add	x2, x3, x2, lsl 8
	mov	w3, 128
	str	w3, [x2,2056]
	and	w3, w1, 255
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
	str	w3, [x2,2052]
	lsr	w3, w1, 8
	str	w3, [x2,2052]
	str	w4, [x2,2052]
	bl	FlashSetRandomizer
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashProgFirstCmd, .-FlashProgFirstCmd
	.align	2
	.global	FlashEraseCmd
	.type	FlashEraseCmd, %function
FlashEraseCmd:
	adrp	x3, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x3, x3, :lo12:.LANCHOR0
	add	x4, x3, 40
	add	x5, x4, x0
	ldr	x4, [x4,x0]
	ldrb	w0, [x5,8]
	cbz	w2, .L46
	add	x2, x4, x0, lsl 8
	mov	w5, 96
	str	w5, [x2,2056]
	and	w5, w1, 255
	str	w5, [x2,2052]
	lsr	w5, w1, 8
	str	w5, [x2,2052]
	lsr	w5, w1, 16
	str	w5, [x2,2052]
	ldr	w2, [x3,28]
	add	w1, w1, w2
.L46:
	add	x0, x4, x0, lsl 8
	mov	w2, 96
	str	w2, [x0,2056]
	and	w2, w1, 255
	str	w2, [x0,2052]
	lsr	w2, w1, 8
	str	w2, [x0,2052]
	lsr	w1, w1, 16
	str	w1, [x0,2052]
	mov	w1, 208
	str	w1, [x0,2056]
	ret
	.size	FlashEraseCmd, .-FlashEraseCmd
	.align	2
	.global	FlashProgDpSecondCmd
	.type	FlashProgDpSecondCmd, %function
FlashProgDpSecondCmd:
	adrp	x2, .LANCHOR0
	uxtb	w0, w0
	add	x2, x2, :lo12:.LANCHOR0
	sbfiz	x4, x0, 4, 32
	add	x5, x2, 40
	lsr	w3, w1, 16
	add	x7, x5, x4
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldrb	w6, [x2,115]
	ldrb	w2, [x7,8]
	ldr	x4, [x5,x4]
	add	x2, x4, x2, lsl 8
	and	w4, w1, 255
	str	w6, [x2,2056]
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
	str	w4, [x2,2052]
	lsr	w4, w1, 8
	str	w4, [x2,2052]
	str	w3, [x2,2052]
	bl	FlashSetRandomizer
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashProgDpSecondCmd, .-FlashProgDpSecondCmd
	.align	2
	.global	FlashProgSecondCmd
	.type	FlashProgSecondCmd, %function
FlashProgSecondCmd:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 40
	add	x2, x1, x0
	ldr	x1, [x1,x0]
	ldrb	w0, [x2,8]
	add	x0, x0, 8
	add	x0, x1, x0, lsl 8
	mov	w1, 16
	str	w1, [x0,8]
	ret
	.size	FlashProgSecondCmd, .-FlashProgSecondCmd
	.align	2
	.global	FlashProgDpFirstCmd
	.type	FlashProgDpFirstCmd, %function
FlashProgDpFirstCmd:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x2, x1, 40
	add	x3, x2, x0
	ldrb	w1, [x1,114]
	ldr	x2, [x2,x0]
	ldrb	w0, [x3,8]
	add	x0, x0, 8
	add	x0, x2, x0, lsl 8
	str	w1, [x0,8]
	ret
	.size	FlashProgDpFirstCmd, .-FlashProgDpFirstCmd
	.align	2
	.global	FlashReadStatus
	.type	FlashReadStatus, %function
FlashReadStatus:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	stp	x29, x30, [sp, -32]!
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 40
	add	x29, sp, 0
	add	x2, x1, x0
	str	x19, [sp,16]
	ldrb	w2, [x2,8]
	ldr	x19, [x1,x0]
	mov	w0, 112
	add	x19, x19, x2, lsl 8
	str	w0, [x19,2056]
	mov	x0, 400
	bl	__const_udelay
	ldr	w0, [x19,2048]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FlashReadStatus, .-FlashReadStatus
	.align	2
	.global	js_hash
	.type	js_hash, %function
js_hash:
	mov	x5, x0
	mov	w0, 42982
	mov	x2, 0
	movk	w0, 0x47c6, lsl 16
.L55:
	cmp	w1, w2
	bls	.L57
	lsl	w3, w0, 5
	ldrb	w4, [x5,x2]
	add	w3, w3, w0, lsr 2
	add	x2, x2, 1
	add	w3, w3, w4
	eor	w0, w0, w3
	b	.L55
.L57:
	ret
	.size	js_hash, .-js_hash
	.align	2
	.global	FlashLoadIdbInfo
	.type	FlashLoadIdbInfo, %function
FlashLoadIdbInfo:
	mov	w0, 0
	ret
	.size	FlashLoadIdbInfo, .-FlashLoadIdbInfo
	.align	2
	.global	BuildFlashLsbPageTable
	.type	BuildFlashLsbPageTable, %function
BuildFlashLsbPageTable:
	stp	x29, x30, [sp, -32]!
	adrp	x2, .LANCHOR0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	w20, w1
	cbnz	w0, .L60
	mov	x0, 0
	add	x3, x2, :lo12:.LANCHOR0
.L61:
	add	x1, x3, 136
	strh	w0, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L61
.L67:
	add	x19, x2, :lo12:.LANCHOR0
	mov	w1, 255
	mov	w2, 1024
	add	x0, x19, 648
	uxth	w20, w20
	bl	ftl_memset
	mov	x0, 0
	mov	x2, x19
	b	.L62
.L60:
	cmp	w0, 1
	bne	.L63
	mov	x0, 0
	mov	w5, 3
	mov	w6, 2
	add	x4, x2, :lo12:.LANCHOR0
.L66:
	cmp	x0, 3
	uxth	w1, w0
	mov	w3, w1
	bls	.L64
	ubfiz	w3, w1, 1, 15
	and	w1, w1, 1
	cmp	w1, wzr
	csel	w1, w5, w6, ne
	sub	w3, w3, w1
	uxth	w3, w3
.L64:
	add	x1, x4, 136
	strh	w3, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L66
	b	.L67
.L63:
	cmp	w0, 2
	bne	.L68
	mov	w1, 65535
	mov	x0, 0
	add	x5, x2, :lo12:.LANCHOR0
.L70:
	add	x4, x5, 136
	cmp	x0, 1
	uxth	w3, w0
	csel	w3, w1, w3, hi
	add	w1, w1, 2
	strh	w3, [x4,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	uxth	w1, w1
	bne	.L70
	b	.L67
.L68:
	cmp	w0, 3
	bne	.L71
	mov	x0, 0
	mov	w5, 5
	mov	w6, 4
	add	x4, x2, :lo12:.LANCHOR0
.L74:
	cmp	x0, 5
	uxth	w1, w0
	mov	w3, w1
	bls	.L72
	ubfiz	w3, w1, 1, 15
	and	w1, w1, 1
	cmp	w1, wzr
	csel	w1, w5, w6, ne
	sub	w3, w3, w1
	uxth	w3, w3
.L72:
	add	x1, x4, 136
	strh	w3, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L74
	b	.L67
.L71:
	cmp	w0, 4
	bne	.L75
	add	x1, x2, :lo12:.LANCHOR0
	mov	w6, 7
	add	x3, x1, 136
	mov	w5, 6
	strh	w0, [x3,8]
	mov	w0, 5
	strh	wzr, [x1,136]
	mov	w1, 1
	strh	w0, [x3,10]
	mov	w0, 7
	strh	w1, [x3,2]
	mov	w1, 2
	strh	w0, [x3,12]
	mov	w0, 8
	strh	w1, [x3,4]
	mov	w1, 3
	strh	w0, [x3,14]
	mov	w0, 8
	strh	w1, [x3,6]
.L77:
	and	w4, w0, 1
	ubfiz	w1, w0, 1, 15
	add	w0, w0, 1
	cmp	w4, wzr
	csel	w4, w6, w5, ne
	add	x3, x3, 2
	uxth	w0, w0
	sub	w1, w1, w4
	cmp	w0, 256
	strh	w1, [x3,14]
	bne	.L77
	b	.L67
.L75:
	cmp	w0, 5
	bne	.L78
	mov	x0, 0
	add	x3, x2, :lo12:.LANCHOR0
.L79:
	add	x1, x3, 136
	strh	w0, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 16
	bne	.L79
	mov	x0, 0
.L80:
	add	x3, x1, x0
	add	w4, w0, 16
	add	x0, x0, 2
	cmp	x0, 480
	strh	w4, [x3,32]
	bne	.L80
	b	.L67
.L78:
	cmp	w0, 6
	bne	.L81
	mov	x3, 0
	mov	w6, 12
	mov	w7, 10
	add	x5, x2, :lo12:.LANCHOR0
.L84:
	cmp	x3, 5
	uxth	w4, w3
	mov	w0, w4
	bls	.L82
	add	w0, w4, w4, lsl 1
	and	w4, w4, 1
	cmp	w4, wzr
	uxth	w0, w0
	csel	w1, w6, w7, ne
	sub	w0, w0, w1
	uxth	w0, w0
.L82:
	add	x1, x5, 136
	strh	w0, [x1,x3,lsl 1]
	add	x3, x3, 1
	cmp	x3, 256
	bne	.L84
	b	.L67
.L81:
	cmp	w0, 9
	bne	.L67
	add	x0, x2, :lo12:.LANCHOR0
	add	x3, x0, 136
	strh	wzr, [x0,136]
	mov	w0, 1
	strh	w0, [x3,2]
	mov	w0, 2
	strh	w0, [x3,4]
	mov	x0, 0
.L85:
	add	x1, x3, x0
	add	w4, w0, 3
	add	x0, x0, 2
	cmp	x0, 506
	strh	w4, [x1,6]
	bne	.L85
	b	.L67
.L62:
	cmp	w20, w0, uxth
	bls	.L104
	add	x1, x2, 136
	add	x3, x2, 648
	ldrh	w1, [x1,x0,lsl 1]
	add	x0, x0, 1
	strh	w1, [x3,w1,sxtw 1]
	b	.L62
.L104:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	BuildFlashLsbPageTable, .-BuildFlashLsbPageTable
	.align	2
	.global	FlashPrintInfo
	.type	FlashPrintInfo, %function
FlashPrintInfo:
	ret
	.size	FlashPrintInfo, .-FlashPrintInfo
	.align	2
	.global	ToshibaSetRRPara
	.type	ToshibaSetRRPara, %function
ToshibaSetRRPara:
	uxtb	w1, w1
	mov	w3, 5
	add	w2, w1, 1
	stp	x29, x30, [sp, -80]!
	smull	x3, w2, w3
	adrp	x2, .LANCHOR1
	add	x29, sp, 0
	add	x2, x2, :lo12:.LANCHOR1
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	add	x23, x2, x1, sxtw
	stp	x25, x26, [sp,64]
	add	x21, x2, 304
	add	x25, x2, 256
	adrp	x24, g_maxRegNum
	adrp	x26, g_retryMode
	stp	x19, x20, [sp,16]
	mov	x22, x0
	add	x25, x25, x3
	add	x21, x21, x3
	mov	x20, 0
	add	x24, x24, :lo12:g_maxRegNum
	mov	x19, x2
	add	x26, x26, :lo12:g_retryMode
.L107:
	ldrb	w0, [x24]
	cmp	w0, w20
	bls	.L113
	mov	w0, 85
	str	w0, [x22,8]
	add	x0, x19, 256
	ldrsb	w0, [x20,x0]
	str	w0, [x22,4]
	mov	x0, 1000
	bl	__const_udelay
	ldrb	w0, [x26]
	cmp	w0, 34
	bne	.L108
	ldrsb	w0, [x25,x20]
	b	.L112
.L108:
	cmp	w0, 35
	bne	.L110
	ldrsb	w0, [x21,x20]
	b	.L112
.L110:
	ldrsb	w0, [x23,400]
.L112:
	str	w0, [x22]
	add	x20, x20, 1
	b	.L107
.L113:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	ToshibaSetRRPara, .-ToshibaSetRRPara
	.align	2
	.global	SamsungSetRRPara
	.type	SamsungSetRRPara, %function
SamsungSetRRPara:
	stp	x29, x30, [sp, -64]!
	adrp	x2, .LANCHOR1
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	add	x2, x2, :lo12:.LANCHOR1
	adrp	x22, g_maxRegNum
	stp	x19, x20, [sp,16]
	add	x1, x2, x1, uxtb 2
	stp	x23, x24, [sp,48]
	mov	x21, x0
	add	x23, x1, 412
	mov	x20, 0
	add	x22, x22, :lo12:g_maxRegNum
	mov	w24, 161
	mov	x19, x2
.L115:
	ldrb	w0, [x22]
	cmp	w0, w20
	bls	.L117
	str	w24, [x21,8]
	add	x0, x19, 408
	str	wzr, [x21]
	ldrsb	w0, [x20,x0]
	str	w0, [x21]
	ldrsb	w0, [x23,x20]
	add	x20, x20, 1
	str	w0, [x21]
	mov	x0, 1500
	bl	__const_udelay
	b	.L115
.L117:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	SamsungSetRRPara, .-SamsungSetRRPara
	.align	2
	.global	FlashDieInfoInit
	.type	FlashDieInfoInit, %function
FlashDieInfoInit:
	stp	x29, x30, [sp, -96]!
	adrp	x0, .LANCHOR1+482
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR0
	ldrh	w0, [x0,#:lo12:.LANCHOR1+482]
	stp	x19, x20, [sp,16]
	add	x19, x21, :lo12:.LANCHOR0
	str	x27, [sp,80]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	strb	wzr, [x19,1672]
	add	x25, x19, 1676
	strb	wzr, [x19,1673]
	bl	FlashBlockAlignInit
	mov	w1, 0
	mov	w2, 8
	mov	x0, x25
	add	x24, x19, 1684
	adrp	x26, IDByte
	mov	x23, 0
	bl	ftl_memset
	add	x26, x26, :lo12:IDByte
	mov	w1, 0
	mov	w2, 32
	mov	x0, x24
	bl	ftl_memset
	add	x0, x19, 1716
	mov	w1, 0
	mov	w2, 192
	bl	ftl_memset
	ldr	x20, [x19,1912]
	add	x22, x20, 1
.L120:
	ldrb	w2, [x20]
	mov	x0, x22
	add	x1, x26, x23, lsl 3
	bl	FlashMemCmp8
	cbnz	w0, .L119
	ldrb	w1, [x19,1672]
	str	w0, [x24,w1,sxtw 2]
	add	w0, w1, 1
	strb	w23, [x25,w1,sxtw]
	strb	w0, [x19,1672]
.L119:
	add	x23, x23, 1
	cmp	x23, 4
	bne	.L120
	add	x19, x21, :lo12:.LANCHOR0
	ldrb	w0, [x19,1672]
	strb	w0, [x19,1673]
	ldrb	w0, [x20,8]
	cmp	w0, 2
	beq	.L121
.L125:
	add	x21, x21, :lo12:.LANCHOR0
	ldrb	w1, [x20,13]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldrb	w0, [x21,1672]
	ldr	x27, [sp,80]
	mul	w0, w0, w1
	ldrh	w1, [x20,14]
	ldp	x19, x20, [sp,16]
	mul	w0, w0, w1
	strh	w0, [x21,1920]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 96
	ret
.L121:
	adrp	x24, IDByte
	ldr	w26, [x19,28]
	mov	x23, 0
	add	x24, x24, :lo12:IDByte
	add	x25, x19, 1684
	add	x27, x19, 1676
.L124:
	ldrb	w2, [x20]
	mov	x0, x22
	add	x1, x24, x23, lsl 3
	bl	FlashMemCmp8
	cbnz	w0, .L122
	ldrb	w1, [x20,13]
	ldrb	w3, [x19,1672]
	mul	w2, w1, w26
	ldrh	w1, [x20,14]
	mov	x0, x3
	and	w1, w1, 65280
	mul	w1, w2, w1
	str	w1, [x25,w3,sxtw 2]
	ldrb	w2, [x20,23]
	cbz	w2, .L123
	lsl	w1, w1, 1
	str	w1, [x25,w3,sxtw 2]
.L123:
	add	w0, w0, 1
	strb	w23, [x27,x3]
	strb	w0, [x19,1672]
.L122:
	add	x23, x23, 1
	cmp	x23, 4
	bne	.L124
	b	.L125
	.size	FlashDieInfoInit, .-FlashDieInfoInit
	.align	2
	.global	FlashReadIdbData
	.type	FlashReadIdbData, %function
FlashReadIdbData:
	stp	x29, x30, [sp, -16]!
	adrp	x1, .LANCHOR0
	mov	w2, 2048
	add	x1, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x1, x1, 1928
	bl	ftl_memcpy
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashReadIdbData, .-FlashReadIdbData
	.align	2
	.global	FlashLoadPhyInfoInRam
	.type	FlashLoadPhyInfoInRam, %function
FlashLoadPhyInfoInRam:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR1
	str	x23, [sp,48]
	stp	x21, x22, [sp,32]
	adrp	x23, IDByte
	add	x21, x19, :lo12:.LANCHOR1
	add	x21, x21, 504
	mov	w20, 0
	add	x23, x23, :lo12:IDByte
.L137:
	ldrb	w2, [x21]
	add	x0, x21, 1
	mov	x1, x23
	bl	FlashMemCmp8
	mov	w22, w0
	cbnz	w0, .L134
	add	x0, x19, :lo12:.LANCHOR1
	ubfiz	x20, x20, 5, 32
	add	x1, x0, 504
	adds	x20, x1, x20
	beq	.L140
	ldrb	w3, [x20,22]
	mov	x2, 0
	mov	x1, x0
	b	.L139
.L134:
	add	w20, w20, 1
	add	x21, x21, 32
	cmp	w20, 81
	bne	.L137
	b	.L140
.L139:
	add	x4, x1, x2, lsl 5
	mov	w0, w2
	ldrb	w4, [x4,3096]
	cmp	w4, w3
	beq	.L138
	add	x2, x2, 1
	cmp	x2, 4
	bne	.L139
	mov	w0, w2
.L138:
	add	x19, x19, :lo12:.LANCHOR1
	ubfiz	x1, x0, 5, 32
	adrp	x0, .LANCHOR0
	add	x2, x19, 3096
	add	x0, x0, :lo12:.LANCHOR0
	add	x1, x2, x1
	add	x0, x0, 104
	mov	w2, 32
	add	x19, x19, 472
	bl	ftl_memcpy
	mov	x0, x19
	mov	x1, x20
	mov	w2, 32
	bl	ftl_memcpy
	ldrh	w0, [x19,10]
	bl	FlashBlockAlignInit
	b	.L135
.L140:
	mov	w22, -1
.L135:
	mov	w0, w22
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	FlashLoadPhyInfoInRam, .-FlashLoadPhyInfoInRam
	.align	2
	.global	ftl_flash_suspend
	.type	ftl_flash_suspend, %function
ftl_flash_suspend:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldr	x1, [x0,3976]
	ldr	w2, [x1]
	str	w2, [x0,3984]
	ldr	w2, [x1,4]
	str	w2, [x0,3988]
	ldr	w2, [x1,8]
	str	w2, [x0,3992]
	ldr	w2, [x1,12]
	str	w2, [x0,3996]
	ldr	w2, [x1,304]
	str	w2, [x0,4000]
	ldr	w2, [x1,308]
	str	w2, [x0,4004]
	ldr	w2, [x1,336]
	ldr	w1, [x1,344]
	str	w2, [x0,4008]
	str	w1, [x0,4012]
	ret
	.size	ftl_flash_suspend, .-ftl_flash_suspend
	.align	2
	.global	LogAddr2PhyAddr
	.type	LogAddr2PhyAddr, %function
LogAddr2PhyAddr:
	adrp	x7, .LANCHOR0
	uxtb	w4, w4
	add	x6, x7, :lo12:.LANCHOR0
	ldrh	w8, [x6,4030]
	ldrh	w5, [x6,4028]
	ldrh	w9, [x6,28]
	ldrb	w6, [x6,24]
	mul	w5, w5, w8
	cmp	w6, 1
	ubfiz	w8, w9, 1, 15
	csel	w9, w8, w9, eq
	ldr	w8, [x0,4]
	uxth	w5, w5
	cmp	w1, 1
	ubfx	x6, x8, 10, 16
	and	w8, w8, 1023
	udiv	w10, w6, w5
	msub	w5, w10, w5, w6
	uxth	w6, w5
	bne	.L147
	add	x1, x7, :lo12:.LANCHOR0
	ldrb	w5, [x1,4044]
	cbnz	w5, .L147
	add	x1, x1, 136
	ldrh	w8, [x1,w8,sxtw 1]
.L147:
	add	x7, x7, :lo12:.LANCHOR0
	cmp	w4, 1
	add	x7, x7, 1684
	mov	w1, 0
	ldr	w5, [x7,w10,uxtw 2]
	madd	w5, w6, w9, w5
	add	w5, w5, w8
	str	w5, [x2]
	str	w10, [x3]
	bls	.L148
	ldr	w1, [x0,4]
	ldr	w0, [x0,60]
	add	w1, w1, 1024
	cmp	w1, w0
	cset	w1, eq
.L148:
	mov	w0, w1
	ret
	.size	LogAddr2PhyAddr, .-LogAddr2PhyAddr
	.align	2
	.global	FlashReadStatusEN
	.type	FlashReadStatusEN, %function
FlashReadStatusEN:
	stp	x29, x30, [sp, -32]!
	adrp	x3, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x4, x3, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x5, x4, 40
	stp	x19, x20, [sp,16]
	add	x6, x5, x0
	uxtb	w2, w2
	ldr	x20, [x5,x0]
	ldr	x0, [x4,1912]
	ldrb	w19, [x6,8]
	ldrb	w0, [x0,8]
	cmp	w0, 2
	bne	.L151
	add	x4, x4, 104
	cbnz	w2, .L152
	ldrb	w2, [x4,13]
	b	.L160
.L152:
	ldrb	w2, [x4,14]
.L160:
	add	x0, x19, 8
	add	x3, x3, :lo12:.LANCHOR0
	add	x0, x20, x0, lsl 8
	str	w2, [x0,8]
	mov	w2, 0
	ldrb	w4, [x3,119]
	cbz	w4, .L155
	add	x3, x19, 8
	lsl	x3, x3, 8
.L159:
	cmp	w2, w4
	bcs	.L155
	add	x5, x20, x3
	lsl	w0, w2, 3
	lsr	w0, w1, w0
	add	w2, w2, 1
	and	w0, w0, 255
	str	w0, [x5,4]
	b	.L159
.L151:
	add	x0, x19, 8
	mov	w1, 112
	add	x0, x20, x0, lsl 8
	str	w1, [x0,8]
.L155:
	add	x19, x19, 8
	mov	x0, 400
	lsl	x19, x19, 8
	bl	__const_udelay
	ldr	w0, [x20,x19]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	uxtb	w0, w0
	ret
	.size	FlashReadStatusEN, .-FlashReadStatusEN
	.align	2
	.global	FlashWaitReadyEN
	.type	FlashWaitReadyEN, %function
FlashWaitReadyEN:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	uxtb	w19, w0
	mov	w20, w1
	uxtb	w21, w2
.L165:
	mov	w0, w19
	mov	w1, w20
	mov	w2, w21
	bl	FlashReadStatusEN
	cmp	w0, 255
	mov	w3, w0
	beq	.L165
	tbz	x3, 6, .L165
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashWaitReadyEN, .-FlashWaitReadyEN
	.align	2
	.global	ftl_read_flash_info
	.type	ftl_read_flash_info, %function
ftl_read_flash_info:
	stp	x29, x30, [sp, -32]!
	mov	w1, 0
	mov	w2, 11
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	x19, x0
	bl	ftl_memset
	adrp	x1, .LANCHOR0
	mov	w5, 1
	add	x1, x1, :lo12:.LANCHOR0
	ldr	x0, [x1,1912]
	ldrb	w2, [x0,9]
	ldr	w0, [x1,28]
	add	x1, x1, 1676
	mul	w0, w0, w2
	strh	w0, [x19,4]
	ldrb	w0, [x1,2369]
	strb	w0, [x19,7]
	ldr	w0, [x1,2372]
	str	w0, [x19]
	ldr	x0, [x1,236]
	ldrb	w4, [x1,-4]
	ldrb	w0, [x0,9]
	strb	w0, [x19,6]
	mov	w0, 32
	strb	w0, [x19,8]
	ldr	x0, [x1,236]
	ldrb	w0, [x0,7]
	strb	w0, [x19,9]
	mov	x0, 0
	strb	wzr, [x19,10]
.L169:
	cmp	w4, w0, uxtb
	bls	.L171
	ldrb	w2, [x0,x1]
	add	x0, x0, 1
	ldrb	w3, [x19,10]
	lsl	w2, w5, w2
	orr	w2, w2, w3
	strb	w2, [x19,10]
	b	.L169
.L171:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	ftl_read_flash_info, .-ftl_read_flash_info
	.align	2
	.global	FlashScheduleEnSet
	.type	FlashScheduleEnSet, %function
FlashScheduleEnSet:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldr	w2, [x1,4052]
	str	w0, [x1,4052]
	mov	w0, w2
	ret
	.size	FlashScheduleEnSet, .-FlashScheduleEnSet
	.align	2
	.global	FlashGetPageSize
	.type	FlashGetPageSize, %function
FlashGetPageSize:
	adrp	x0, .LANCHOR0+1912
	ldr	x0, [x0,#:lo12:.LANCHOR0+1912]
	ldrb	w0, [x0,9]
	ret
	.size	FlashGetPageSize, .-FlashGetPageSize
	.align	2
	.global	NandcReadDontCaseBusyEn
	.type	NandcReadDontCaseBusyEn, %function
NandcReadDontCaseBusyEn:
	ret
	.size	NandcReadDontCaseBusyEn, .-NandcReadDontCaseBusyEn
	.align	2
	.global	NandcGetChipIf
	.type	NandcGetChipIf, %function
NandcGetChipIf:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 40
	add	x2, x1, x0
	ldr	x0, [x1,x0]
	ldrb	w2, [x2,8]
	add	x2, x2, 8
	add	x0, x0, x2, lsl 8
	ret
	.size	NandcGetChipIf, .-NandcGetChipIf
	.align	2
	.global	NandcSetDdrPara
	.type	NandcSetDdrPara, %function
NandcSetDdrPara:
	adrp	x1, .LANCHOR0+3976
	uxtb	w0, w0
	lsl	w2, w0, 8
	orr	w0, w2, w0, lsl 16
	ldr	x1, [x1,#:lo12:.LANCHOR0+3976]
	orr	w0, w0, 1
	str	w0, [x1,304]
	ret
	.size	NandcSetDdrPara, .-NandcSetDdrPara
	.align	2
	.global	NandcSetDdrDiv
	.type	NandcSetDdrDiv, %function
NandcSetDdrDiv:
	adrp	x1, .LANCHOR0+3976
	uxtb	w0, w0
	ldr	x2, [x1,#:lo12:.LANCHOR0+3976]
	mov	w1, 16640
	orr	w0, w0, w1
	str	w0, [x2,344]
	ret
	.size	NandcSetDdrDiv, .-NandcSetDdrDiv
	.align	2
	.global	NandcSetDdrMode
	.type	NandcSetDdrMode, %function
NandcSetDdrMode:
	adrp	x1, .LANCHOR0+3976
	cmp	w0, wzr
	ldr	x2, [x1,#:lo12:.LANCHOR0+3976]
	ldr	w1, [x2]
	and	w3, w1, -8193
	orr	w1, w1, 253952
	csel	w1, w1, w3, ne
	str	w1, [x2]
	ret
	.size	NandcSetDdrMode, .-NandcSetDdrMode
	.align	2
	.global	NandcSetMode
	.type	NandcSetMode, %function
NandcSetMode:
	adrp	x1, .LANCHOR0+3976
	uxtb	w0, w0
	tst	w0, 6
	ldr	x2, [x1,#:lo12:.LANCHOR0+3976]
	ldr	w1, [x2]
	beq	.L182
	and	w0, w0, 4
	orr	w1, w1, 24576
	cmp	w0, wzr
	mov	w0, 8322
	and	w1, w1, -32769
	str	w0, [x2,344]
	mov	w0, 4099
	orr	w1, w1, 196608
	movk	w0, 0x10, lsl 16
	str	w0, [x2,304]
	orr	w3, w1, 32768
	mov	w0, 38
	str	w0, [x2,308]
	csel	w1, w3, w1, ne
	mov	w0, 39
	str	w0, [x2,308]
	b	.L184
.L182:
	and	w1, w1, -8193
.L184:
	str	w1, [x2]
	mov	w0, 0
	ret
	.size	NandcSetMode, .-NandcSetMode
	.align	2
	.global	NandcFlashCs
	.type	NandcFlashCs, %function
NandcFlashCs:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 40
	add	x3, x1, x0
	ldr	x2, [x1,x0]
	mov	w1, 1
	ldrb	w3, [x3,8]
	ldr	w0, [x2]
	lsl	w1, w1, w3
	bfi	w0, w1, 0, 8
	str	w0, [x2]
	ret
	.size	NandcFlashCs, .-NandcFlashCs
	.align	2
	.global	NandcFlashDeCs
	.type	NandcFlashDeCs, %function
NandcFlashDeCs:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 40
	ldr	x1, [x1,x0]
	ldr	w0, [x1]
	and	w0, w0, -256
	and	w0, w0, -131073
	str	w0, [x1]
	ret
	.size	NandcFlashDeCs, .-NandcFlashDeCs
	.align	2
	.global	HynixSetRRPara
	.type	HynixSetRRPara, %function
HynixSetRRPara:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	x20, x2
	adrp	x2, .LANCHOR0
	add	x19, x2, :lo12:.LANCHOR0
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x21, x22, [sp,32]
	uxtb	x23, w0
	uxtb	w26, w1
	ldr	x0, [x19,1912]
	mov	x25, x23
	uxtb	w24, w3
	ldrb	w0, [x0,19]
	cmp	w0, 6
	bne	.L190
	add	x19, x19, x23, lsl 6
	lsl	w4, w24, 2
	add	x19, x19, 4076
	b	.L196
.L190:
	cmp	w0, 7
	bne	.L192
	mov	x0, 160
	mov	w4, 10
	madd	x19, x23, x0, x19
	mul	w4, w24, w4
	add	x19, x19, 4084
.L196:
	add	x19, x19, x4, sxtw
	b	.L191
.L192:
	cmp	w0, 8
	bne	.L193
	add	x19, x19, 4084
	add	w4, w24, w24, lsl 2
	b	.L196
.L193:
	lsl	x4, x23, 3
	add	x4, x4, x24, sxtw
	add	x19, x19, x4, lsl 3
	add	x19, x19, 4076
.L191:
	add	x2, x2, :lo12:.LANCHOR0
	lsl	x0, x23, 4
	add	x2, x2, 40
	uxtb	x26, w26
	add	x1, x2, x0
	ldr	x21, [x2,x0]
	mov	w0, w25
	ldrb	w22, [x1,8]
	bl	NandcFlashCs
	ubfiz	x22, x22, 8, 8
	mov	w0, 54
	add	x21, x21, x22
	mov	x22, 0
	str	w0, [x21,2056]
.L194:
	cmp	x22, x26
	beq	.L197
	ldrb	w0, [x20,x22]
	str	w0, [x21,2052]
	mov	x0, 1000
	bl	__const_udelay
	ldrsb	w0, [x19,x22]
	add	x22, x22, 1
	str	w0, [x21,2048]
	b	.L194
.L197:
	mov	w0, 22
	str	w0, [x21,2056]
	mov	w0, w25
	bl	NandcFlashDeCs
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x23, x0, x23
	strb	w24, [x23,560]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	HynixSetRRPara, .-HynixSetRRPara
	.align	2
	.global	FlashSetReadRetryDefault
	.type	FlashSetReadRetryDefault, %function
FlashSetReadRetryDefault:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	mov	x20, 0
	add	x19, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	ldr	x0, [x19,1912]
	ldrb	w0, [x0,19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 7
	bhi	.L198
	adrp	x21, IDByte
	add	x22, x19, 4060
	add	x21, x21, :lo12:IDByte
.L203:
	lsl	x1, x20, 3
	uxtb	w0, w20
	ldrb	w1, [x1,x21]
	cmp	w1, 173
	bne	.L200
	ldrb	w1, [x19,4057]
	mov	x2, x22
	mov	w3, 0
	bl	HynixSetRRPara
.L200:
	add	x20, x20, 1
	cmp	x20, 4
	bne	.L203
.L198:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashSetReadRetryDefault, .-FlashSetReadRetryDefault
	.align	2
	.global	FlashWaitCmdDone
	.type	FlashWaitCmdDone, %function
FlashWaitCmdDone:
	stp	x29, x30, [sp, -48]!
	uxtb	x1, w0
	mov	x2, 24
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	stp	x21, x22, [sp,32]
	add	x20, x20, :lo12:.LANCHOR0
	mov	x22, x1
	mul	x1, x1, x2
	add	x0, x20, 1716
	add	x19, x0, x1
	ldrb	w21, [x0,x1]
	ldr	x0, [x19,8]
	cbz	x0, .L207
	mov	w0, w21
	add	x20, x20, 1684
	bl	NandcFlashCs
	ldr	w0, [x20,w22,sxtw 2]
	ldr	w1, [x19,4]
	cmp	w0, wzr
	mov	w0, w21
	cset	w2, ne
	bl	FlashWaitReadyEN
	mov	w20, w0
	mov	w0, w21
	bl	NandcFlashDeCs
	ldr	x1, [x19,8]
	sbfx	x0, x20, 0, 1
	str	w0, [x1]
	str	xzr, [x19,8]
	ldr	x1, [x19,16]
	cbz	x1, .L207
	str	w0, [x1]
	str	xzr, [x19,16]
.L207:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashWaitCmdDone, .-FlashWaitCmdDone
	.align	2
	.global	NandcDelayns
	.type	NandcDelayns, %function
NandcDelayns:
	stp	x29, x30, [sp, -16]!
	uxtw	x0, w0
	add	x29, sp, 0
	bl	__ndelay
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	NandcDelayns, .-NandcDelayns
	.align	2
	.global	NandcWaitFlashReadyNoDelay
	.type	NandcWaitFlashReadyNoDelay, %function
NandcWaitFlashReadyNoDelay:
	stp	x29, x30, [sp, -48]!
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x1, x1, 40
	stp	x19, x20, [sp,16]
	mov	w19, 34464
	ldr	x20, [x1,x0]
	movk	w19, 0x1, lsl 16
.L215:
	ldr	w0, [x20]
	str	w0, [x29,40]
	ldr	w0, [x29,40]
	tbnz	x0, 9, .L216
	mov	x0, 50
	bl	__const_udelay
	subs	w19, w19, #1
	bne	.L215
	mov	w0, -1
	b	.L214
.L216:
	mov	w0, 0
.L214:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	NandcWaitFlashReadyNoDelay, .-NandcWaitFlashReadyNoDelay
	.align	2
	.global	NandcWaitFlashReady
	.type	NandcWaitFlashReady, %function
NandcWaitFlashReady:
	stp	x29, x30, [sp, -48]!
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x1, x1, 40
	stp	x19, x20, [sp,16]
	mov	w19, 34464
	ldr	x20, [x1,x0]
	movk	w19, 0x1, lsl 16
	mov	x0, 750
	bl	__const_udelay
.L220:
	ldr	w0, [x20]
	str	w0, [x29,40]
	ldr	w0, [x29,40]
	tbnz	x0, 9, .L221
	mov	x0, 50
	bl	__const_udelay
	subs	w19, w19, #1
	bne	.L220
	mov	w0, -1
	b	.L219
.L221:
	mov	w0, 0
.L219:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	NandcWaitFlashReady, .-NandcWaitFlashReady
	.align	2
	.global	FlashReset
	.type	FlashReset, %function
FlashReset:
	stp	x29, x30, [sp, -48]!
	adrp	x1, .LANCHOR0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	add	x1, x1, :lo12:.LANCHOR0
	uxtb	w20, w0
	add	x1, x1, 40
	sbfiz	x0, x20, 4, 32
	add	x2, x1, x0
	str	x21, [sp,32]
	ldr	x21, [x1,x0]
	mov	w0, w20
	ldrb	w19, [x2,8]
	bl	NandcFlashCs
	add	x19, x19, 8
	mov	w0, 255
	add	x19, x21, x19, lsl 8
	str	w0, [x19,8]
	mov	w0, w20
	bl	NandcWaitFlashReady
	mov	w0, w20
	bl	NandcFlashDeCs
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashReset, .-FlashReset
	.align	2
	.global	flash_enter_slc_mode
	.type	flash_enter_slc_mode, %function
flash_enter_slc_mode:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	uxtb	w20, w0
	add	x19, x19, :lo12:.LANCHOR0
	ldrb	w0, [x19,4044]
	cbz	w0, .L224
	mov	w0, w20
	add	x19, x19, 40
	bl	NandcFlashCs
	sbfiz	x0, x20, 4, 32
	add	x1, x19, x0
	ldr	x19, [x19,x0]
	mov	w0, 239
	ldrb	w1, [x1,8]
	add	x19, x19, x1, lsl 8
	str	w0, [x19,2056]
	mov	w0, 145
	str	w0, [x19,2052]
	mov	x0, 500
	bl	__const_udelay
	str	wzr, [x19,2048]
	mov	w0, 1
	str	w0, [x19,2048]
	str	wzr, [x19,2048]
	mov	x0, 500
	str	wzr, [x19,2048]
	bl	__const_udelay
	mov	w0, w20
	bl	NandcWaitFlashReadyNoDelay
	mov	w0, 218
	str	w0, [x19,2056]
	mov	w0, w20
	bl	NandcWaitFlashReady
.L224:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	flash_enter_slc_mode, .-flash_enter_slc_mode
	.align	2
	.global	flash_exit_slc_mode
	.type	flash_exit_slc_mode, %function
flash_exit_slc_mode:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	uxtb	w20, w0
	add	x19, x19, :lo12:.LANCHOR0
	ldrb	w0, [x19,4044]
	cbz	w0, .L229
	mov	w0, w20
	add	x19, x19, 40
	bl	NandcFlashCs
	sbfiz	x0, x20, 4, 32
	add	x1, x19, x0
	ldr	x19, [x19,x0]
	mov	w0, 239
	ldrb	w1, [x1,8]
	add	x19, x19, x1, lsl 8
	str	w0, [x19,2056]
	mov	w0, 145
	str	w0, [x19,2052]
	mov	x0, 500
	bl	__const_udelay
	mov	w0, 2
	str	w0, [x19,2048]
	mov	w0, 1
	str	w0, [x19,2048]
	str	wzr, [x19,2048]
	mov	x0, 500
	str	wzr, [x19,2048]
	bl	__const_udelay
	mov	w0, w20
	bl	NandcWaitFlashReadyNoDelay
	mov	w0, 223
	str	w0, [x19,2056]
	mov	w0, w20
	bl	NandcWaitFlashReady
.L229:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	flash_exit_slc_mode, .-flash_exit_slc_mode
	.align	2
	.global	FlashEraseBlock
	.type	FlashEraseBlock, %function
FlashEraseBlock:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxtb	w19, w0
	mov	w20, w1
	mov	w0, w19
	str	x2, [x29,40]
	bl	NandcWaitFlashReady
	mov	w0, w19
	bl	NandcFlashCs
	ldr	x2, [x29,40]
	mov	w1, w20
	mov	w0, w19
	bl	FlashEraseCmd
	mov	w0, w19
	bl	NandcWaitFlashReady
	mov	w1, w20
	mov	w0, w19
	bl	FlashReadStatus
	mov	w20, w0
	mov	w0, w19
	bl	NandcFlashDeCs
	and	w0, w20, 1
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashEraseBlock, .-FlashEraseBlock
	.align	2
	.global	FlashSetInterfaceMode
	.type	FlashSetInterfaceMode, %function
FlashSetInterfaceMode:
	adrp	x1, .LANCHOR2+564
	adrp	x3, .LANCHOR0
	stp	x29, x30, [sp, -16]!
	add	x3, x3, :lo12:.LANCHOR0
	adrp	x7, IDByte
	add	x29, sp, 0
	ldrb	w1, [x1,#:lo12:.LANCHOR2+564]
	add	x3, x3, 40
	mov	x6, 0
	add	x7, x7, :lo12:IDByte
	and	w11, w1, 1
	and	w1, w1, 4
	mov	w8, 239
	mov	w9, 128
	uxtb	w5, w1
	mov	w10, 1
	mov	w12, 35
	mov	w13, 32
	mov	w14, 5
.L246:
	ldrb	w4, [x6,x7]
	ldr	x1, [x3]
	cmp	w4, 152
	ldrb	w2, [x3,8]
	beq	.L236
	cmp	w4, 69
	beq	.L236
	cmp	w4, 173
	beq	.L236
	cmp	w4, 44
	bne	.L237
.L236:
	cmp	w0, 1
	bne	.L239
	cbz	w11, .L237
	ubfiz	x2, x2, 8, 8
	cmp	w4, 173
	add	x1, x1, x2
	str	w8, [x1,2056]
	bne	.L240
	str	w0, [x1,2052]
	b	.L257
.L240:
	cmp	w4, 44
	bne	.L242
	str	w0, [x1,2052]
	str	w14, [x1,2048]
	b	.L244
.L242:
	str	w9, [x1,2052]
	str	w0, [x1,2048]
	b	.L244
.L239:
	cbz	w5, .L237
	ubfiz	x2, x2, 8, 8
	cmp	w4, 173
	add	x1, x1, x2
	str	w8, [x1,2056]
	bne	.L243
	str	w10, [x1,2052]
	str	w13, [x1,2048]
	b	.L244
.L243:
	cmp	w4, 44
	bne	.L245
	str	w10, [x1,2052]
	str	w12, [x1,2048]
	b	.L244
.L245:
	str	w9, [x1,2052]
.L257:
	str	wzr, [x1,2048]
.L244:
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
.L237:
	add	x6, x6, 8
	add	x3, x3, 16
	cmp	x6, 32
	bne	.L246
	mov	w0, 0
	bl	NandcWaitFlashReady
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashSetInterfaceMode, .-FlashSetInterfaceMode
	.align	2
	.global	FlashReadSpare
	.type	FlashReadSpare, %function
FlashReadSpare:
	adrp	x3, .LANCHOR1+481
	adrp	x5, .LANCHOR0
	stp	x29, x30, [sp, -48]!
	uxtb	w0, w0
	add	x5, x5, :lo12:.LANCHOR0
	add	x5, x5, 40
	sbfiz	x6, x0, 4, 32
	add	x29, sp, 0
	ldrb	w4, [x3,#:lo12:.LANCHOR1+481]
	add	x3, x5, x6
	str	x19, [sp,16]
	lsl	w4, w4, 9
	str	x2, [x29,40]
	ldrb	w3, [x3,8]
	ldr	x19, [x5,x6]
	add	x19, x19, x3, lsl 8
	and	w3, w1, 255
	str	wzr, [x19,2056]
	str	w4, [x19,2052]
	lsr	w4, w4, 8
	str	w4, [x19,2052]
	str	w3, [x19,2052]
	lsr	w3, w1, 8
	str	w3, [x19,2052]
	lsr	w1, w1, 16
	str	w1, [x19,2052]
	mov	w1, 48
	str	w1, [x19,2056]
	bl	NandcWaitFlashReady
	ldr	x2, [x29,40]
	ldr	w0, [x19,2048]
	strb	w0, [x2]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashReadSpare, .-FlashReadSpare
	.align	2
	.global	SandiskProgTestBadBlock
	.type	SandiskProgTestBadBlock, %function
SandiskProgTestBadBlock:
	adrp	x3, .LANCHOR0
	uxtb	w0, w0
	stp	x29, x30, [sp, -32]!
	add	x3, x3, :lo12:.LANCHOR0
	sbfiz	x4, x0, 4, 32
	add	x3, x3, 40
	add	x29, sp, 0
	add	x2, x3, x4
	str	x19, [sp,16]
	ldrb	w2, [x2,8]
	ldr	x19, [x3,x4]
	add	x19, x19, x2, lsl 8
	mov	w2, 162
	str	w2, [x19,2056]
	mov	w2, 128
	str	w2, [x19,2056]
	and	w2, w1, 255
	str	wzr, [x19,2052]
	str	wzr, [x19,2052]
	str	w2, [x19,2052]
	lsr	w2, w1, 8
	str	w2, [x19,2052]
	lsr	w1, w1, 16
	str	w1, [x19,2052]
	mov	w1, 16
	str	w1, [x19,2056]
	bl	NandcWaitFlashReady
	mov	w0, 112
	str	w0, [x19,2056]
	mov	x0, 400
	bl	__const_udelay
	ldr	w0, [x19,2048]
	ldr	x19, [sp,16]
	and	w0, w0, 1
	ldp	x29, x30, [sp], 32
	ret
	.size	SandiskProgTestBadBlock, .-SandiskProgTestBadBlock
	.align	2
	.global	SandiskSetRRPara
	.type	SandiskSetRRPara, %function
SandiskSetRRPara:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	x20, x0
	mov	w0, 239
	uxtb	w19, w1
	add	w19, w19, 1
	str	w0, [x20,8]
	mov	w0, 17
	str	w0, [x20,4]
	mov	x0, 1000
	bl	__const_udelay
	mov	w1, 5
	adrp	x2, .LANCHOR1
	add	x2, x2, :lo12:.LANCHOR1
	adrp	x0, g_maxRegNum
	smull	x1, w19, w1
	add	x3, x2, 256
	adrp	x4, g_retryMode
	add	x2, x2, 304
	add	x3, x3, x1
	add	x0, x0, :lo12:g_maxRegNum
	add	x1, x2, x1
	add	x4, x4, :lo12:g_retryMode
	mov	x2, 0
.L261:
	ldrb	w5, [x0]
	cmp	w5, w2
	bls	.L266
	ldrb	w5, [x4]
	cmp	w5, 67
	bne	.L262
	ldrsb	w5, [x3,x2]
	b	.L265
.L262:
	ldrsb	w5, [x1,x2]
.L265:
	str	w5, [x20]
	add	x2, x2, 1
	b	.L261
.L266:
	mov	w0, 0
	bl	NandcWaitFlashReady
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	SandiskSetRRPara, .-SandiskSetRRPara
	.align	2
	.global	micron_auto_read_calibration_config
	.type	micron_auto_read_calibration_config, %function
micron_auto_read_calibration_config:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x19, [sp,16]
	uxtb	w19, w0
	mov	w0, w19
	sbfiz	x19, x19, 4, 32
	str	x1, [x29,40]
	bl	NandcWaitFlashReady
	adrp	x3, .LANCHOR0
	add	x3, x3, :lo12:.LANCHOR0
	add	x3, x3, 40
	add	x0, x3, x19
	ldr	x19, [x3,x19]
	ldrb	w2, [x0,8]
	mov	w0, 239
	add	x19, x19, x2, lsl 8
	str	w0, [x19,2056]
	mov	w0, 150
	str	w0, [x19,2052]
	mov	x0, 1000
	bl	__const_udelay
	ldr	x1, [x29,40]
	str	w1, [x19,2048]
	str	wzr, [x19,2048]
	str	wzr, [x19,2048]
	str	wzr, [x19,2048]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	micron_auto_read_calibration_config, .-micron_auto_read_calibration_config
	.align	2
	.global	FlashEraseSLc2KBlocks
	.type	FlashEraseSLc2KBlocks, %function
FlashEraseSLc2KBlocks:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR0
	str	x25, [sp,64]
	stp	x23, x24, [sp,48]
	add	x21, x21, :lo12:.LANCHOR0
	adrp	x23, .LC1
	stp	x19, x20, [sp,16]
	mov	w24, w1
	mov	x20, x0
	mov	w22, 0
	add	x25, x21, 1676
	add	x23, x23, :lo12:.LC1
.L269:
	cmp	w22, w24
	beq	.L278
	add	x2, x29, 88
	sub	w4, w24, w22
	mov	x0, x20
	mov	w1, 0
	add	x3, x29, 92
	bl	LogAddr2PhyAddr
	ldr	w2, [x29,92]
	ldrb	w0, [x21,1672]
	cmp	w2, w0
	bcc	.L270
	mov	w0, -1
	str	w0, [x20]
	b	.L271
.L270:
	uxtw	x2, w2
	mov	x0, 24
	ldrb	w19, [x25,x2]
	madd	x2, x2, x0, x21
	mov	w0, w19
	strb	w19, [x2,1716]
	bl	NandcWaitFlashReady
	mov	w0, w19
	bl	NandcFlashCs
	ldr	w1, [x29,88]
	mov	w2, 0
	mov	w0, w19
	bl	FlashEraseCmd
	mov	w0, w19
	bl	NandcWaitFlashReady
	ldr	w1, [x29,88]
	mov	w0, w19
	bl	FlashReadStatus
	sbfx	x0, x0, 0, 1
	str	w0, [x20]
	mov	w0, w19
	ldr	w1, [x29,88]
	ldr	w2, [x21,28]
	add	w1, w2, w1
	mov	w2, 0
	bl	FlashEraseCmd
	mov	w0, w19
	bl	NandcWaitFlashReady
	ldr	w1, [x29,88]
	mov	w0, w19
	bl	FlashReadStatus
	tbz	x0, 0, .L272
	mov	w0, -1
	str	w0, [x20]
.L272:
	ldr	w0, [x20]
	cmn	w0, #1
	bne	.L273
	ldr	w1, [x29,88]
	mov	x0, x23
	bl	printk
.L273:
	mov	w0, w19
	bl	NandcFlashDeCs
.L271:
	add	w22, w22, 1
	add	x20, x20, 56
	b	.L269
.L278:
	mov	w0, 0
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 96
	ret
	.size	FlashEraseSLc2KBlocks, .-FlashEraseSLc2KBlocks
	.align	2
	.global	HynixGetReadRetryDefault
	.type	HynixGetReadRetryDefault, %function
HynixGetReadRetryDefault:
	stp	x29, x30, [sp, -144]!
	mov	w4, -83
	mov	w3, -82
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	stp	x25, x26, [sp,64]
	mov	w26, w0
	add	x0, x20, :lo12:.LANCHOR0
	add	x1, x0, 4056
	cmp	w26, 2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	strb	w26, [x0,4056]
	mov	w0, -84
	strb	w4, [x1,5]
	strb	w0, [x1,4]
	mov	w0, -81
	strb	w3, [x1,6]
	strb	w0, [x1,7]
	bne	.L280
	mov	w0, -89
	strb	w0, [x1,4]
	adrp	x0, .LANCHOR1+3241
	mov	w1, -9
	strb	w1, [x0,#:lo12:.LANCHOR1+3241]
	b	.L343
.L280:
	cmp	w26, 3
	bne	.L282
	mov	w0, -80
	strb	w0, [x1,4]
	mov	w0, -79
	strb	w0, [x1,5]
	mov	w0, -78
	strb	w0, [x1,6]
	mov	w0, -77
	strb	w0, [x1,7]
	mov	w0, -76
	strb	w0, [x1,8]
	mov	w0, -75
	strb	w0, [x1,9]
	mov	w0, -74
	strb	w0, [x1,10]
	mov	w0, -73
	b	.L383
.L282:
	cmp	w26, 4
	bne	.L283
	mov	w5, -52
	strb	w5, [x1,4]
	mov	w5, -65
	strb	w5, [x1,5]
	mov	w5, -86
	strb	w5, [x1,6]
	mov	w5, -85
	strb	w4, [x1,9]
	strb	w5, [x1,7]
	mov	w5, -51
	strb	w3, [x1,10]
	strb	w5, [x1,8]
.L383:
	mov	w28, 8
	strb	w0, [x1,11]
	mov	w27, w28
	b	.L281
.L283:
	cmp	w26, 5
	bne	.L284
	mov	w0, 56
	strb	w0, [x1,4]
	mov	w0, 57
	strb	w0, [x1,5]
	mov	w0, 58
	mov	w28, 8
	strb	w0, [x1,6]
	mov	w0, 59
	strb	w0, [x1,7]
	b	.L384
.L284:
	cmp	w26, 6
	bne	.L285
	mov	w0, 14
	strb	w0, [x1,4]
	mov	w0, 15
	strb	w0, [x1,5]
	mov	w0, 16
	mov	w28, 12
	strb	w0, [x1,6]
	mov	w0, 17
	strb	w0, [x1,7]
	b	.L384
.L285:
	cmp	w26, 7
	bne	.L286
	mov	w0, -80
	strb	w0, [x1,4]
	mov	w0, -79
	strb	w0, [x1,5]
	mov	w0, -78
	strb	w0, [x1,6]
	mov	w0, -77
	strb	w0, [x1,7]
	mov	w0, -76
	strb	w0, [x1,8]
	mov	w0, -75
	strb	w0, [x1,9]
	mov	w0, -74
	strb	w0, [x1,10]
	mov	w0, -73
	strb	w0, [x1,11]
	mov	w0, -44
	mov	w28, 12
	strb	w0, [x1,12]
	mov	w27, 10
	mov	w0, -43
	strb	w0, [x1,13]
	b	.L281
.L286:
	cmp	w26, 8
	bne	.L343
	mov	w0, 6
	strb	w0, [x1,4]
	mov	w0, 7
	strb	w0, [x1,5]
	mov	w0, 9
	strb	w26, [x1,6]
	strb	w0, [x1,7]
	mov	w28, 50
	mov	w0, 10
	mov	w27, 5
	strb	w0, [x1,8]
	b	.L281
.L343:
	mov	w28, 7
.L384:
	mov	w27, 4
.L281:
	sub	w0, w26, #1
	cmp	w0, 1
	bhi	.L380
	adrp	x22, .LANCHOR1
	add	x25, x20, :lo12:.LANCHOR0
	add	x22, x22, :lo12:.LANCHOR1
	mov	w21, 0
	add	x3, x25, 1676
	add	x26, x25, 4056
	add	x22, x22, 3224
.L287:
	ldrb	w0, [x25,1672]
	cmp	w0, w21
	bls	.L294
	ldrb	w0, [x3,w21,sxtw]
	add	x1, x25, 40
	mov	x23, 0
	add	x19, x26, x0, lsl 6
	lsl	x0, x0, 4
	add	x2, x1, x0
	add	x19, x19, 20
	ldr	x24, [x1,x0]
	mov	w1, 55
	ldrb	w0, [x2,8]
	add	x24, x24, x0, lsl 8
.L289:
	add	x0, x26, x23
	str	w1, [x24,2056]
	str	x3, [x29,128]
	str	x1, [x29,136]
	ldrb	w0, [x0,4]
	str	w0, [x24,2052]
	mov	x0, 400
	bl	__const_udelay
	ldr	w0, [x24,2048]
	strb	w0, [x19,x23]
	add	x23, x23, 1
	ldr	x1, [x29,136]
	cmp	w27, w23, uxtb
	ldr	x3, [x29,128]
	bhi	.L289
	mov	x0, 0
.L290:
	add	w2, w0, 8
	mov	x1, 0
.L291:
	add	x5, x1, x0
	add	x1, x1, 4
	add	x5, x22, x5
	cmp	x1, 24
	ldrb	w6, [x5,4]
	ldrb	w5, [x19,x0]
	add	w5, w6, w5
	strb	w5, [x19,w2,sxtw]
	add	w2, w2, 8
	bne	.L291
	add	x0, x0, 1
	cmp	x0, 4
	bne	.L290
	add	w21, w21, 1
	strb	wzr, [x19,16]
	strb	wzr, [x19,24]
	strb	wzr, [x19,32]
	uxtb	w21, w21
	strb	wzr, [x19,40]
	strb	wzr, [x19,48]
	strb	wzr, [x19,41]
	strb	wzr, [x19,49]
	b	.L287
.L380:
	sub	w0, w26, #3
	cmp	w0, 5
	bhi	.L294
	mul	w25, w27, w28
	sub	w24, w27, #1
	mov	w21, 0
	lsl	w0, w25, 4
	asr	w23, w25, 1
	str	w0, [x29,128]
	add	x25, x20, :lo12:.LANCHOR0
	lsl	w0, w23, 1
	uxtb	x24, w24
	str	w0, [x29,124]
	add	x0, x25, 1676
	add	x24, x24, 1
	str	x0, [x29,112]
	sub	w0, w26, #5
	str	w0, [x29,136]
.L295:
	ldrb	w0, [x25,1672]
	cmp	w0, w21
	bhi	.L341
.L294:
	add	x20, x20, :lo12:.LANCHOR0
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	strb	w27, [x20,4057]
	strb	w28, [x20,4058]
	ldp	x25, x26, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
.L341:
	ldr	x0, [x29,112]
	ldrb	w5, [x0,w21,sxtw]
	add	x0, x25, 40
	str	x5, [x29,104]
	lsl	x1, x5, 4
	mov	x22, x5
	add	x7, x0, x1
	ldr	x0, [x0,x1]
	ldrb	w19, [x7,8]
	add	x19, x0, x19, lsl 8
	mov	w0, 255
	str	w0, [x19,2056]
	mov	w0, w5
	bl	NandcWaitFlashReady
	cmp	w26, 7
	ldr	x5, [x29,104]
	bne	.L296
	mov	x0, 160
	madd	x5, x5, x0, x25
	add	x5, x5, 4084
	b	.L297
.L296:
	cmp	w26, 8
	beq	.L298
	add	x5, x25, x5, lsl 6
	add	x5, x5, 4076
.L297:
	mov	w0, 54
	cmp	w26, 4
	str	w0, [x19,2056]
	bne	.L299
	mov	w0, 255
	str	w0, [x19,2052]
	mov	w0, 64
	str	w0, [x19,2048]
	mov	w0, 204
	b	.L385
.L299:
	ldr	w0, [x29,136]
	cmp	w0, 1
	bhi	.L301
	ldrb	w0, [x25,4060]
	str	w0, [x19,2052]
	mov	w0, 82
	b	.L386
.L301:
	cmp	w26, 7
	bne	.L300
	mov	w0, 174
	str	w0, [x19,2052]
	str	wzr, [x19,2048]
	mov	w0, 176
.L385:
	str	w0, [x19,2052]
	mov	w0, 77
.L386:
	str	w0, [x19,2048]
.L300:
	mov	w0, 22
	str	w0, [x19,2056]
	mov	w0, 23
	str	w0, [x19,2056]
	mov	w0, 4
	str	w0, [x19,2056]
	mov	w0, 25
	str	w0, [x19,2056]
	str	wzr, [x19,2056]
	cmp	w26, 6
	str	wzr, [x19,2052]
	str	wzr, [x19,2052]
	bne	.L302
	mov	w0, 31
	str	w0, [x19,2052]
	b	.L303
.L302:
	str	wzr, [x19,2052]
.L303:
	mov	w0, 2
	str	w0, [x19,2052]
	str	wzr, [x19,2052]
.L342:
	mov	w0, 48
	str	w0, [x19,2056]
	mov	w0, w22
	str	x5, [x29,104]
	bl	NandcWaitFlashReady
	ldr	w0, [x29,136]
	ldr	x5, [x29,104]
	cmp	w0, 1
	cset	w8, ls
	cmp	w26, 8
	cset	w7, eq
	cbnz	w8, .L345
	cbnz	w7, .L345
	cmp	w26, 7
	mov	w1, 32
	mov	w0, 2
	csel	w1, w0, w1, ne
	b	.L304
.L345:
	mov	w1, 16
.L304:
	ldr	x0, [x25]
	mov	x9, 0
.L305:
	ldr	w10, [x19,2048]
	strb	w10, [x0,x9]
	add	x9, x9, 1
	cmp	w1, w9, uxtb
	bhi	.L305
	cbz	w7, .L306
	mov	w1, 0
.L308:
	ldrb	w9, [x0]
	cmp	w9, 50
	beq	.L307
	ldrb	w9, [x0,1]
	cmp	w9, 5
	beq	.L307
	add	w1, w1, 1
	add	x0, x0, 4
	uxtb	w1, w1
	cmp	w1, 8
	bne	.L308
	b	.L309
.L307:
	cmp	w1, 6
	bls	.L310
.L309:
	adrp	x0, .LC2
	mov	w1, 0
	add	x0, x0, :lo12:.LC2
	bl	printk
.L311:
	b	.L311
.L306:
	cmp	w26, 7
	bne	.L312
	mov	w1, w7
.L314:
	ldrb	w9, [x0]
	cmp	w9, 12
	beq	.L313
	ldrb	w9, [x0,1]
	cmp	w9, 10
	beq	.L313
	add	w1, w1, 1
	add	x0, x0, 4
	uxtb	w1, w1
	cmp	w1, 8
	bne	.L314
	b	.L315
.L313:
	cmp	w1, 7
	bne	.L310
.L315:
	adrp	x0, .LC2
	mov	w1, 0
	add	x0, x0, :lo12:.LC2
	bl	printk
.L316:
	b	.L316
.L312:
	cmp	w26, 6
	bne	.L310
	mov	x1, 0
.L317:
	ldrb	w9, [x0,x1]
	cmp	w9, 12
	beq	.L310
	add	x9, x0, x1
	ldrb	w9, [x9,8]
	cmp	w9, 4
	beq	.L310
	add	x1, x1, 1
	cmp	x1, 8
	bne	.L317
	adrp	x0, .LC2
	mov	w1, 0
	add	x0, x0, :lo12:.LC2
	bl	printk
.L319:
	b	.L319
.L310:
	ldr	x10, [x25]
	mov	x0, 0
.L320:
	ldr	w1, [x29,128]
	cmp	w1, w0
	ble	.L388
	ldr	w1, [x19,2048]
	strb	w1, [x10,x0]
	add	x0, x0, 1
	b	.L320
.L388:
	ldr	x12, [x25]
	mov	w11, w23
	mov	w9, 8
.L323:
	mov	w0, 0
.L322:
	add	w1, w0, w11
	add	w0, w0, 1
	sbfiz	x1, x1, 1, 32
	cmp	w0, w23
	ldrh	w13, [x12,x1]
	mvn	w13, w13
	strh	w13, [x12,x1]
	bne	.L322
	ldr	w0, [x29,124]
	subs	w9, w9, #1
	add	w11, w11, w0
	bne	.L323
	mov	x9, 0
	mov	w17, 1
.L324:
	mov	w1, 0
	mov	w11, w1
.L328:
	lsl	w13, w17, w11
	mov	w15, w9
	mov	w0, 16
	mov	w14, 0
.L326:
	ldrh	w16, [x12,w15,sxtw 1]
	add	w15, w15, w23
	and	w16, w16, w13
	cmp	w16, w13
	csinc	w14, w14, w14, ne
	subs	w0, w0, #1
	bne	.L326
	cmp	w14, 8
	bls	.L327
	orr	w1, w1, w13
	uxth	w1, w1
.L327:
	add	w11, w11, 1
	cmp	w11, 16
	bne	.L328
	strh	w1, [x12,x9,lsl 1]
	add	x9, x9, 1
	cmp	w23, w9
	bgt	.L324
	mov	x9, 0
	ldr	x1, [x25]
	mov	w11, w9
.L331:
	ldr	w12, [x1,x9]
	add	x9, x9, 4
	cmp	w12, wzr
	csinc	w11, w11, w11, ne
	cmp	x9, 32
	bne	.L331
	cmp	w11, 7
	ble	.L332
	adrp	x0, .LC3
	mov	w2, 1
	add	x0, x0, :lo12:.LC3
	mov	w3, 1024
	bl	rknand_print_hex
	adrp	x0, .LC2
	mov	w1, 0
	add	x0, x0, :lo12:.LC2
	bl	printk
.L333:
	b	.L333
.L332:
	cmp	w26, 6
	mov	w9, 4
	beq	.L334
	cmp	w26, 7
	mov	w9, 10
	beq	.L334
	cmp	w7, wzr
	mov	w9, 5
	mov	w1, 8
	csel	w9, w1, w9, eq
.L334:
	mov	w11, 0
.L335:
	mov	x1, 0
.L336:
	add	w12, w0, w1
	ldrb	w13, [x10,x1]
	add	x1, x1, 1
	cmp	w27, w1, uxtb
	strb	w13, [x5,w12,sxtw]
	bhi	.L336
	add	w11, w11, 1
	add	x10, x10, x24
	cmp	w11, w28
	add	w0, w0, w9
	blt	.L335
	mov	w0, 255
	str	w0, [x19,2056]
	mov	w0, w22
	str	x7, [x29,96]
	str	x8, [x29,104]
	bl	NandcWaitFlashReady
	ldr	x8, [x29,104]
	ldr	x7, [x29,96]
	cbz	w8, .L338
	mov	w0, 54
	str	w0, [x19,2056]
	adrp	x0, .LANCHOR0+4060
	mov	w1, -1
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4060]
	str	w0, [x19,2052]
	mov	w0, 22
	str	wzr, [x19,2048]
	str	w0, [x19,2056]
	mov	w0, w21
	bl	FlashReadCmd
	b	.L339
.L338:
	mov	w0, 190
	cbnz	w7, .L387
	mov	w0, 56
.L387:
	str	w0, [x19,2056]
.L339:
	mov	w0, w22
	add	w21, w21, 1
	bl	NandcWaitFlashReady
	uxtb	w21, w21
	b	.L295
.L298:
	mov	w0, 120
	str	w0, [x19,2056]
	str	wzr, [x19,2052]
	mov	w0, 23
	str	wzr, [x19,2052]
	mov	w1, 25
	str	wzr, [x19,2052]
	add	x5, x25, 4084
	str	w0, [x19,2056]
	mov	w0, 4
	str	w0, [x19,2056]
	str	w1, [x19,2056]
	mov	w1, 218
	str	w1, [x19,2056]
	mov	w1, 21
	str	wzr, [x19,2056]
	str	wzr, [x19,2052]
	str	wzr, [x19,2052]
	str	w1, [x19,2052]
	str	w0, [x19,2052]
	str	wzr, [x19,2052]
	b	.L342
	.size	HynixGetReadRetryDefault, .-HynixGetReadRetryDefault
	.align	2
	.global	FlashGetReadRetryDefault
	.type	FlashGetReadRetryDefault, %function
FlashGetReadRetryDefault:
	stp	x29, x30, [sp, -16]!
	mov	w1, w0
	add	x29, sp, 0
	cbz	w0, .L389
	sub	w2, w0, #1
	cmp	w2, 7
	bhi	.L391
	bl	HynixGetReadRetryDefault
	b	.L389
.L391:
	cmp	w0, 49
	bne	.L392
	adrp	x0, .LANCHOR0
	mov	w2, 64
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, 4060
	strb	w1, [x0,-4]
	mov	w1, 4
	strb	w1, [x0,-3]
	mov	w1, 15
	strb	w1, [x0,-2]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x1, x1, 408
	b	.L407
.L392:
	sub	w0, w0, #65
	cmp	w0, 1
	bls	.L398
	cmp	w1, 33
	bne	.L393
.L398:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	strb	w1, [x0,4056]
	mov	w1, 4
	b	.L408
.L393:
	cmp	w1, 67
	beq	.L399
	cmp	w1, 34
	bne	.L395
.L399:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	strb	w1, [x0,4056]
	mov	w1, 5
.L408:
	strb	w1, [x0,4057]
	mov	w1, 7
	strb	w1, [x0,4058]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x0, x0, 4060
	add	x1, x1, 256
	mov	w2, 45
	b	.L407
.L395:
	cmp	w1, 68
	beq	.L400
	cmp	w1, 35
	bne	.L389
.L400:
	adrp	x0, .LANCHOR0
	mov	w2, 95
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, 4060
	strb	w1, [x0,-4]
	mov	w1, 5
	strb	w1, [x0,-3]
	mov	w1, 17
	strb	w1, [x0,-2]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x1, x1, 304
.L407:
	bl	ftl_memcpy
.L389:
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
	.align	2
	.global	FlashReadDpCmd
	.type	FlashReadDpCmd, %function
FlashReadDpCmd:
	stp	x29, x30, [sp, -80]!
	adrp	x3, .LANCHOR0
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	add	x3, x3, :lo12:.LANCHOR0
	uxtb	w21, w0
	add	x4, x3, 40
	sbfiz	x0, x21, 4, 32
	add	x3, x3, 104
	str	x23, [sp,48]
	stp	x19, x20, [sp,16]
	add	x5, x4, x0
	and	w23, w2, 255
	ldr	x6, [x4,x0]
	lsr	w22, w2, 8
	ldrb	w0, [x3,16]
	lsr	w4, w1, 8
	ldrb	w19, [x5,8]
	lsr	w20, w2, 16
	cmp	w0, 1
	and	w5, w1, 255
	lsr	w0, w1, 16
	bne	.L410
	add	x19, x6, x19, lsl 8
	ldrb	w2, [x3,8]
	str	x1, [x29,72]
	str	w2, [x19,2056]
	str	wzr, [x19,2052]
	str	wzr, [x19,2052]
	str	w5, [x19,2052]
	str	w4, [x19,2052]
	str	w0, [x19,2052]
	ldrb	w0, [x3,9]
	str	w0, [x19,2056]
	mov	w0, w21
	bl	NandcWaitFlashReady
	str	wzr, [x19,2056]
	str	wzr, [x19,2052]
	mov	w0, 48
	str	wzr, [x19,2052]
	str	w23, [x19,2052]
	str	w22, [x19,2052]
	str	w20, [x19,2052]
	str	w0, [x19,2056]
	ldr	x1, [x29,72]
	b	.L411
.L410:
	add	x19, x6, x19, lsl 8
	ldrb	w2, [x3,8]
	str	w2, [x19,2056]
	str	w5, [x19,2052]
	str	w4, [x19,2052]
	str	w0, [x19,2052]
	ldrb	w0, [x3,9]
	str	w0, [x19,2056]
	mov	w0, 48
	str	w23, [x19,2052]
	str	w22, [x19,2052]
	str	w20, [x19,2052]
	str	w0, [x19,2056]
.L411:
	mov	w0, w21
	bl	FlashSetRandomizer
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 80
	ret
	.size	FlashReadDpCmd, .-FlashReadDpCmd
	.align	2
	.global	ftl_flash_de_init
	.type	ftl_flash_de_init, %function
ftl_flash_de_init:
	stp	x29, x30, [sp, -32]!
	mov	w0, 0
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,568]
	cbz	w0, .L413
	mov	w0, 0
	bl	flash_enter_slc_mode
	b	.L414
.L413:
	bl	flash_exit_slc_mode
.L414:
	add	x19, x19, :lo12:.LANCHOR2
	ldrb	w0, [x19,572]
	cbz	w0, .L415
	ldrb	w0, [x19,564]
	tbz	x0, 0, .L415
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	strb	wzr, [x19,572]
.L415:
	adrp	x0, .LANCHOR0+40
	ldr	x0, [x0,#:lo12:.LANCHOR0+40]
	str	wzr, [x0,336]
	mov	w0, 0
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	ftl_flash_de_init, .-ftl_flash_de_init
	.align	2
	.global	NandcRandmzSel
	.type	NandcRandmzSel, %function
NandcRandmzSel:
	adrp	x2, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x2, x2, :lo12:.LANCHOR0
	add	x2, x2, 40
	ldr	x0, [x2,x0]
	str	w1, [x0,336]
	ret
	.size	NandcRandmzSel, .-NandcRandmzSel
	.align	2
	.global	NandcTimeCfg
	.type	NandcTimeCfg, %function
NandcTimeCfg:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	w19, w0
	mov	w0, 0
	bl	rknand_get_clk_rate
	mov	w2, 16960
	adrp	x1, .LANCHOR0
	movk	w2, 0xf, lsl 16
	add	x1, x1, :lo12:.LANCHOR0
	sdiv	w0, w0, w2
	cmp	w0, 250
	ble	.L424
	ldr	x0, [x1,3976]
	mov	w1, 8354
	b	.L431
.L424:
	cmp	w0, 220
	ble	.L426
	ldr	x0, [x1,3976]
	b	.L432
.L426:
	cmp	w0, 185
	ble	.L427
	ldr	x0, [x1,3976]
	mov	w1, 4226
	b	.L431
.L427:
	cmp	w0, 160
	ldr	x0, [x1,3976]
	ble	.L428
	mov	w1, 4194
	b	.L431
.L428:
	cmp	w19, 35
	mov	w1, 4193
	bls	.L431
	cmp	w19, 99
	mov	w1, 4225
	bls	.L431
.L432:
	mov	w1, 8322
.L431:
	str	w1, [x0,4]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	NandcTimeCfg, .-NandcTimeCfg
	.align	2
	.global	FlashTimingCfg
	.type	FlashTimingCfg, %function
FlashTimingCfg:
	sub	w1, w0, #4096
	sub	w1, w1, #97
	stp	x29, x30, [sp, -16]!
	and	w1, w1, -33
	cmp	w1, 1
	add	x29, sp, 0
	bls	.L434
	mov	w1, 8322
	cmp	w0, w1
	bne	.L435
.L434:
	adrp	x1, .LANCHOR0+3976
	ldr	x1, [x1,#:lo12:.LANCHOR0+3976]
	str	w0, [x1,4]
.L435:
	adrp	x0, .LANCHOR1+493
	ldrb	w0, [x0,#:lo12:.LANCHOR1+493]
	bl	NandcTimeCfg
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashTimingCfg, .-FlashTimingCfg
	.align	2
	.global	NandcInit
	.type	NandcInit, %function
NandcInit:
	stp	x29, x30, [sp, -32]!
	mov	w3, 1
	adrp	x1, .LANCHOR2
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	mov	x19, x1
	add	x2, x20, :lo12:.LANCHOR0
	str	w3, [x2,64]
	mov	w3, 2
	str	wzr, [x2,48]
	str	w3, [x2,80]
	mov	w3, 3
	str	x0, [x2,40]
	str	w3, [x2,96]
	add	x3, x1, :lo12:.LANCHOR2
	str	x0, [x2,56]
	str	x0, [x2,72]
	str	x0, [x2,88]
	str	x0, [x2,3976]
	ldr	w2, [x0]
	ubfx	x4, x2, 13, 1
	str	w4, [x3,576]
	ldr	w4, [x0,352]
	and	w2, w2, 245760
	orr	w2, w2, 256
	ubfx	x4, x4, 16, 4
	str	w4, [x3,580]
	ldr	w4, [x0,352]
	str	w4, [x3,584]
	cmp	w4, 2049
	bne	.L437
	mov	w1, 8
	str	w1, [x3,580]
.L437:
	add	x20, x20, :lo12:.LANCHOR0
	str	w2, [x0]
	ldr	x0, [x20,3976]
	str	wzr, [x0,336]
	mov	w0, 40
	bl	NandcTimeCfg
	ldr	x0, [x20,3976]
	mov	w1, 8322
	str	w1, [x0,344]
	mov	w1, 6145
	movk	w1, 0x18, lsl 16
	str	w1, [x0,304]
	mov	w0, 36864
	bl	ftl_malloc
	add	x1, x19, :lo12:.LANCHOR2
	str	x0, [x1,592]
	str	x0, [x1,600]
	add	x0, x0, 32768
	str	wzr, [x1,640]
	str	x0, [x1,608]
	str	wzr, [x1,648]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	NandcInit, .-NandcInit
	.align	2
	.global	NandcGetTimeCfg
	.type	NandcGetTimeCfg, %function
NandcGetTimeCfg:
	adrp	x4, .LANCHOR0
	add	x4, x4, :lo12:.LANCHOR0
	ldr	x5, [x4,3976]
	ldr	w5, [x5,4]
	str	w5, [x0]
	ldr	x0, [x4,3976]
	ldr	w0, [x0]
	str	w0, [x1]
	ldr	x0, [x4,3976]
	ldr	w0, [x0,304]
	str	w0, [x2]
	ldr	x1, [x4,3976]
	ldr	w0, [x1,308]
	ldr	w1, [x1,344]
	and	w0, w0, 255
	orr	w0, w0, w1, lsl 16
	str	w0, [x3]
	ret
	.size	NandcGetTimeCfg, .-NandcGetTimeCfg
	.align	2
	.global	NandcBchSel
	.type	NandcBchSel, %function
NandcBchSel:
	adrp	x1, .LANCHOR0+3976
	uxtb	w0, w0
	cmp	w0, 16
	ldr	x2, [x1,#:lo12:.LANCHOR0+3976]
	mov	w1, 1
	str	w1, [x2,8]
	adrp	x1, .LANCHOR2+652
	str	w0, [x1,#:lo12:.LANCHOR2+652]
	mov	w1, 4096
	bne	.L440
.L443:
	and	w1, w1, -17
	b	.L441
.L440:
	cmp	w0, 24
	bne	.L442
	orr	w1, w1, 16
	b	.L441
.L442:
	cmp	w0, 40
	orr	w1, w1, 262144
	orr	w1, w1, 16
	beq	.L443
.L441:
	orr	w1, w1, 1
	str	w1, [x2,12]
	ret
	.size	NandcBchSel, .-NandcBchSel
	.align	2
	.global	FlashBchSel
	.type	FlashBchSel, %function
FlashBchSel:
	stp	x29, x30, [sp, -16]!
	adrp	x1, .LANCHOR0+4045
	uxtb	w0, w0
	add	x29, sp, 0
	strb	w0, [x1,#:lo12:.LANCHOR0+4045]
	bl	NandcBchSel
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashBchSel, .-FlashBchSel
	.align	2
	.global	ftl_flash_resume
	.type	ftl_flash_resume, %function
ftl_flash_resume:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	str	x21, [sp,32]
	add	x0, x19, :lo12:.LANCHOR0
	adrp	x21, IDByte
	mov	x20, 0
	add	x21, x21, :lo12:IDByte
	ldr	x1, [x0,3976]
	ldr	w2, [x0,3984]
	str	w2, [x1]
	ldr	w2, [x0,3988]
	ldr	x1, [x0,3976]
	str	w2, [x1,4]
	ldr	w2, [x0,3992]
	ldr	x1, [x0,3976]
	str	w2, [x1,8]
	ldr	w2, [x0,3996]
	str	w2, [x1,12]
	ldr	w2, [x0,4000]
	str	w2, [x1,304]
	ldr	w2, [x0,4004]
	str	w2, [x1,308]
	ldr	w2, [x0,4008]
	str	w2, [x1,336]
	ldr	w0, [x0,4012]
	str	w0, [x1,344]
.L450:
	lsl	x0, x20, 3
	ldrb	w0, [x0,x21]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L449
	mov	w0, w20
	bl	FlashReset
.L449:
	add	x20, x20, 1
	cmp	x20, 4
	bne	.L450
	adrp	x20, .LANCHOR2
	add	x20, x20, :lo12:.LANCHOR2
	ldrb	w0, [x20,572]
	cbz	w0, .L451
	mov	w0, 1
	bl	NandcSetMode
	ldrb	w0, [x20,564]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x20,564]
	bl	NandcSetMode
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w0, [x0,4000]
	lsr	w0, w0, 8
	bl	NandcSetDdrPara
.L451:
	add	x19, x19, :lo12:.LANCHOR0
	ldr	x0, [x19,1912]
	ldrb	w0, [x0,20]
	bl	FlashBchSel
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	ftl_flash_resume, .-ftl_flash_resume
	.align	2
	.global	ftl_nandc_get_irq_status
	.type	ftl_nandc_get_irq_status, %function
ftl_nandc_get_irq_status:
	ldr	w0, [x0,372]
	ret
	.size	ftl_nandc_get_irq_status, .-ftl_nandc_get_irq_status
	.align	2
	.global	NandcIqrWaitFlashReady
	.type	NandcIqrWaitFlashReady, %function
NandcIqrWaitFlashReady:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	x19, x0
	bl	rk_nandc_rb_irq_flag_init
	ldr	w0, [x19,368]
	orr	w0, w0, 2
	str	w0, [x19,368]
	ldr	w0, [x19,364]
	orr	w0, w0, 2
	str	w0, [x19,364]
	ldr	w0, [x19]
	tbnz	x0, 9, .L458
	mov	x0, x19
	bl	wait_for_nand_flash_ready
	b	.L457
.L458:
	ldr	w0, [x19,368]
	orr	w0, w0, 2
	str	w0, [x19,368]
	ldr	w0, [x19,364]
	and	w0, w0, -3
	str	w0, [x19,364]
.L457:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	NandcIqrWaitFlashReady, .-NandcIqrWaitFlashReady
	.align	2
	.global	FlashEraseBlocks
	.type	FlashEraseBlocks, %function
FlashEraseBlocks:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	mov	w26, w1
	adrp	x1, .LANCHOR0
	stp	x19, x20, [sp,16]
	add	x20, x1, :lo12:.LANCHOR0
	str	x1, [x29,104]
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	stp	x27, x28, [sp,80]
	mov	w23, w2
	mov	x24, x0
	ldrb	w2, [x20,24]
	mov	w19, 0
	cbz	w2, .L461
	mov	w1, w23
	bl	FlashEraseSLc2KBlocks
	b	.L462
.L461:
	mov	w27, 56
	adrp	x28, .LANCHOR2
	add	x25, x20, 1716
.L492:
	cmp	w19, w23
	bcs	.L493
	umull	x5, w19, w27
	mov	w1, 0
	sub	w4, w23, w19
	add	x21, x24, x5
	add	x2, x29, 120
	mov	x0, x21
	add	x3, x29, 124
	str	x5, [x29,96]
	bl	LogAddr2PhyAddr
	mov	w22, w0
	ldrb	w1, [x20,1672]
	ldr	w0, [x29,124]
	ldr	x5, [x29,96]
	cmp	w0, w1
	bcc	.L463
	mov	w0, -1
	str	w0, [x24,x5]
	b	.L464
.L463:
	add	x1, x28, :lo12:.LANCHOR2
	mov	x2, 24
	ldrb	w1, [x1,656]
	cmp	w1, wzr
	uxtw	x1, w0
	csel	w22, w22, wzr, ne
	madd	x1, x1, x2, x25
	ldr	x1, [x1,8]
	cbz	x1, .L466
	bl	FlashWaitCmdDone
.L466:
	ldr	w1, [x29,124]
	mov	x0, 24
	ldr	w2, [x29,120]
	madd	x0, x1, x0, x25
	str	x21, [x0,8]
	str	xzr, [x0,16]
	str	w2, [x0,4]
	cbz	w22, .L467
	add	w2, w19, 1
	umull	x2, w2, w27
	add	x2, x24, x2
	str	x2, [x0,16]
.L467:
	add	x0, x20, 1676
	ldrb	w21, [x0,x1]
	mov	x0, 24
	mul	x1, x1, x0
	mov	w0, w21
	strb	w21, [x25,x1]
	bl	NandcFlashCs
	cmp	w26, 1
	bne	.L468
	ldrb	w0, [x20,4044]
	cbz	w0, .L468
	mov	w0, w21
	bl	flash_enter_slc_mode
	b	.L469
.L468:
	mov	w0, w21
	bl	flash_exit_slc_mode
.L469:
	ldr	w1, [x29,124]
	add	x0, x20, 1684
	add	w19, w19, w22
	ldr	w0, [x0,x1,lsl 2]
	ldr	w1, [x29,120]
	cmp	w0, wzr
	mov	w0, w21
	cset	w2, ne
	bl	FlashWaitReadyEN
	ldr	w1, [x29,120]
	mov	w0, w21
	mov	w2, w22
	bl	FlashEraseCmd
	mov	w0, w21
	bl	NandcFlashDeCs
.L464:
	add	w19, w19, 1
	b	.L492
.L493:
	ldr	x0, [x29,104]
	mov	x20, 0
	mov	x22, 24
	add	x19, x0, :lo12:.LANCHOR0
	add	x21, x19, 1716
	ldr	x0, [x19,3976]
	bl	NandcIqrWaitFlashReady
.L471:
	ldrb	w0, [x19,1672]
	cmp	w0, w20
	bls	.L494
	mov	w0, w20
	bl	FlashWaitCmdDone
	cmp	w26, 1
	bne	.L472
	ldrb	w0, [x19,4044]
	cbz	w0, .L472
	mul	x0, x20, x22
	ldrb	w0, [x0,x21]
	bl	flash_exit_slc_mode
.L472:
	add	x20, x20, 1
	b	.L471
.L494:
	adrp	x0, .LANCHOR2+660
	ldr	w0, [x0,#:lo12:.LANCHOR2+660]
	cbnz	w0, .L474
.L476:
	mov	w0, 0
	b	.L462
.L474:
	adrp	x0, IDByte
	ldrb	w0, [x0,#:lo12:IDByte]
	cmp	w0, 69
	bne	.L476
	mov	x0, 0
	mov	x1, 56
.L475:
	cmp	w23, w0
	bls	.L476
	mul	x2, x0, x1
	add	x0, x0, 1
	str	wzr, [x24,x2]
	b	.L475
.L462:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	2
	.global	NandcSendDumpDataStart
	.type	NandcSendDumpDataStart, %function
NandcSendDumpDataStart:
	sub	sp, sp, #16
	mov	w2, 1
	mov	w1, 10
	bfi	w1, w2, 5, 2
	ldr	w2, [x0,16]
	str	w2, [sp,8]
	orr	w1, w1, 536870912
	orr	w1, w1, 1024
	ldr	w2, [sp,8]
	orr	w1, w1, 2097152
	and	w2, w2, -5
	str	w2, [sp,8]
	ldr	w2, [sp,8]
	add	sp, sp, 16
	str	w2, [x0,16]
	str	w1, [x0,8]
	orr	w1, w1, 4
	str	w1, [x0,8]
	ret
	.size	NandcSendDumpDataStart, .-NandcSendDumpDataStart
	.align	2
	.global	NandcSendDumpDataDone
	.type	NandcSendDumpDataDone, %function
NandcSendDumpDataDone:
	sub	sp, sp, #16
.L497:
	ldr	w1, [x0,8]
	str	w1, [sp,8]
	ldr	w1, [sp,8]
	tbz	x1, 20, .L497
	add	sp, sp, 16
	ret
	.size	NandcSendDumpDataDone, .-NandcSendDumpDataDone
	.align	2
	.global	NandcXferStart
	.type	NandcXferStart, %function
NandcXferStart:
	stp	x29, x30, [sp, -96]!
	ubfiz	x0, x0, 4, 8
	ubfx	x3, x3, 1, 7
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	mov	x22, x4
	adrp	x4, .LANCHOR0
	add	x4, x4, :lo12:.LANCHOR0
	str	x25, [sp,64]
	add	x4, x4, 40
	uxtb	w25, w1
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	add	x1, x4, x0
	ubfiz	w20, w25, 1, 1
	ldr	x21, [x4,x0]
	orr	w20, w20, 8
	adrp	x19, .LANCHOR2
	uxtb	w23, w2
	ldrb	w0, [x1,8]
	mov	w1, 16
	ldr	w24, [x21,12]
	bfi	w24, w1, 8, 8
	and	w24, w24, -9
	bfi	w24, w0, 5, 3
	mov	w0, 1
	bfi	w20, w0, 5, 2
	add	x0, x19, :lo12:.LANCHOR2
	orr	w20, w20, 536870912
	orr	w20, w20, 1024
	ldr	w0, [x0,580]
	bfi	w20, w3, 4, 1
	cmp	w0, 3
	bls	.L501
	ldr	w0, [x21,16]
	cmp	x5, xzr
	str	w0, [x29,88]
	cset	w7, ne
	ldr	w0, [x29,88]
	and	w0, w0, -5
	str	w0, [x29,88]
	cbnz	w7, .L515
	cbz	x22, .L502
.L515:
	cbnz	w25, .L504
.L512:
	mov	x0, x21
	add	w23, w23, 1
	asr	w23, w23, 1
	bl	rk_nandc_xfer_irq_flag_init
	bfi	w20, w23, 22, 6
	ldr	w0, [x21,368]
	orr	w0, w0, 1
	str	w0, [x21,368]
	ldr	w0, [x21,364]
	orr	w0, w0, 1
	str	w0, [x21,364]
	mov	x0, x22
	cbnz	x22, .L506
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x0, [x0,600]
	b	.L506
.L504:
	add	x1, x19, :lo12:.LANCHOR2
	mov	w4, 128
	mov	w2, 0
	lsr	w8, w23, 1
	mov	w3, w2
	mov	w9, -1
	ldr	w0, [x1,652]
	cmp	w0, 25
	mov	w0, 64
	csel	w4, w0, w4, cc
.L508:
	cmp	w3, w8
	bcs	.L512
	lsr	w0, w2, 2
	ubfiz	x0, x0, 2, 30
	cbz	w7, .L509
	ldrh	w6, [x5,2]
	ldr	x10, [x1,608]
	ldrh	w11, [x5],4
	orr	w6, w11, w6, lsl 16
	str	w6, [x10,x0]
	b	.L510
.L509:
	ldr	x6, [x1,608]
	str	w9, [x6,x0]
.L510:
	add	w3, w3, 1
	add	w2, w2, w4
	b	.L508
.L506:
	add	x19, x19, :lo12:.LANCHOR2
	ubfx	x23, x20, 22, 5
	mov	w2, w25
	and	x22, x22, 3
	ldr	x1, [x19,608]
	str	x1, [x19,624]
	lsl	w1, w23, 10
	str	x0, [x19,616]
	bl	rknand_dma_map_single
	str	w0, [x19,632]
	ldr	x0, [x19,624]
	lsl	w1, w23, 7
	mov	w2, w25
	bl	rknand_dma_map_single
	str	w0, [x19,636]
	mov	w0, 1
	str	w0, [x19,640]
	ldr	w0, [x19,632]
	mov	w1, 16
	str	w0, [x21,20]
	ldr	w0, [x19,636]
	str	w0, [x21,24]
	str	wzr, [x29,88]
	ldr	w0, [x29,88]
	bfi	w0, w1, 9, 5
	str	w0, [x29,88]
	ldr	w0, [x29,88]
	orr	w0, w0, 448
	str	w0, [x29,88]
	cbnz	x22, .L513
	ldr	w0, [x29,88]
	mov	w1, 2
	bfi	w0, w1, 3, 3
	str	w0, [x29,88]
.L513:
	ldr	w0, [x29,88]
	cmp	w25, wzr
	cset	w1, eq
	orr	w0, w0, 4
	str	w0, [x29,88]
	ldr	w0, [x29,88]
	bfi	w0, w1, 1, 1
	str	w0, [x29,88]
	ldr	w0, [x29,88]
	orr	w0, w0, 1
	str	w0, [x29,88]
.L502:
	ldr	w0, [x29,88]
	str	w0, [x21,16]
.L501:
	str	w24, [x21,12]
	str	w20, [x21,8]
	orr	w20, w20, 4
	str	w20, [x21,8]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	NandcXferStart, .-NandcXferStart
	.align	2
	.global	NandcXferComp
	.type	NandcXferComp, %function
NandcXferComp:
	stp	x29, x30, [sp, -80]!
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x1, x1, 40
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	str	x23, [sp,48]
	adrp	x20, .LANCHOR2
	ldr	x19, [x1,x0]
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w0, [x0,580]
	cmp	w0, 3
	bls	.L550
	ldr	w0, [x19,16]
	tbz	x0, 2, .L550
	mov	x0, x19
	bl	wait_for_nandc_xfer_completed
	ldr	w21, [x19,16]
	ldr	w0, [x19,8]
	ubfx	x21, x21, 1, 1
	str	w0, [x29,64]
	cbz	w21, .L521
	adrp	x22, .LC4
	adrp	x23, .LC5
	mov	w21, 0
	add	x22, x22, :lo12:.LC4
	add	x23, x23, :lo12:.LC5
.L522:
	ldr	w2, [x19,28]
	ldr	w1, [x29,64]
	ubfx	x2, x2, 16, 5
	ubfx	x1, x1, 22, 6
	cmp	w2, w1
	bge	.L530
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w0, [x0,580]
	cmp	w0, 5
	bhi	.L523
.L526:
	add	w21, w21, 1
	and	w0, w21, 16777215
	cbnz	w0, .L522
	ldr	w2, [x19,28]
	mov	w1, w21
	ldr	w3, [x29,64]
	mov	x0, x22
	ubfx	x2, x2, 16, 5
	ubfx	x3, x3, 22, 6
	bl	printk
	mov	x0, x23
	mov	x1, x19
	mov	w2, 4
	mov	w3, 512
	bl	rknand_print_hex
	b	.L522
.L523:
	ldr	w0, [x19]
	str	w0, [x29,72]
	ldr	w0, [x29,72]
	tbz	x0, 13, .L526
	ldr	w0, [x29,72]
	tbz	x0, 17, .L526
.L530:
	add	x19, x20, :lo12:.LANCHOR2
	add	x19, x19, 600
	ldr	w0, [x19,40]
	cbz	w0, .L531
	ldr	w1, [x29,64]
	mov	w2, 0
	ldr	w0, [x19,32]
	ubfx	x1, x1, 22, 5
	lsl	w1, w1, 10
	bl	rknand_dma_unmap_single
	ldr	w1, [x29,64]
	mov	w2, 0
	ldr	w0, [x19,36]
	ubfx	x1, x1, 22, 5
	lsl	w1, w1, 7
	bl	rknand_dma_unmap_single
	b	.L531
.L521:
	adrp	x22, .LC6
	adrp	x23, .LC5
	add	x22, x22, :lo12:.LC6
	add	x23, x23, :lo12:.LC5
.L532:
	ldr	w0, [x29,64]
	tbnz	x0, 20, .L558
	ldr	w0, [x19,8]
	add	w21, w21, 1
	str	w0, [x29,64]
	and	w0, w21, 16777215
	cbnz	w0, .L532
	ldr	w2, [x29,64]
	mov	w1, w21
	ldr	w3, [x19,28]
	mov	x0, x22
	ubfx	x3, x3, 16, 5
	bl	printk
	mov	x0, x23
	mov	x1, x19
	mov	w2, 4
	mov	w3, 512
	bl	rknand_print_hex
	b	.L532
.L558:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w0, [x0,648]
	cbz	w0, .L535
	mov	x0, x19
	bl	NandcSendDumpDataStart
.L535:
	add	x21, x20, :lo12:.LANCHOR2
	add	x21, x21, 600
	ldr	w0, [x21,40]
	cbz	w0, .L536
	ldr	w1, [x29,64]
	mov	w2, 1
	ldr	w0, [x21,32]
	ubfx	x1, x1, 22, 5
	lsl	w1, w1, 10
	bl	rknand_dma_unmap_single
	ldr	w1, [x29,64]
	mov	w2, 1
	ldr	w0, [x21,36]
	ubfx	x1, x1, 22, 5
	lsl	w1, w1, 7
	bl	rknand_dma_unmap_single
.L536:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w0, [x0,648]
	cbz	w0, .L531
	mov	x0, x19
	bl	NandcSendDumpDataDone
.L531:
	add	x20, x20, :lo12:.LANCHOR2
	str	wzr, [x20,640]
	b	.L519
.L550:
	ldr	w0, [x19,8]
	str	w0, [x29,64]
	ldr	w0, [x29,64]
	tbz	x0, 20, .L550
.L519:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldr	x23, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	NandcXferComp, .-NandcXferComp
	.align	2
	.global	NandcCopy1KB
	.type	NandcCopy1KB, %function
NandcCopy1KB:
	stp	x29, x30, [sp, -48]!
	uxtb	w1, w1
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	uxtb	w19, w2
	cmp	w1, 1
	mov	x20, x4
	ubfiz	x2, x19, 9, 8
	add	x4, x0, 4096
	add	x21, x0, 512
	add	x2, x4, x2
	bne	.L560
	cbz	x3, .L561
	mov	x0, x2
	mov	x1, x3
	mov	w2, 1024
	bl	ftl_memcpy
.L561:
	cbz	x20, .L559
	mov	w0, 48
	lsr	w19, w19, 1
	ldrb	w1, [x20,1]
	mul	w19, w19, w0
	ldrb	w0, [x20,2]
	asr	w19, w19, 2
	lsl	w0, w0, 16
	orr	w0, w0, w1, lsl 8
	ldrb	w1, [x20]
	orr	w0, w0, w1
	ldrb	w1, [x20,3]
	orr	w0, w0, w1, lsl 24
	str	w0, [x21,w19,sxtw 2]
	b	.L559
.L560:
	cbz	x3, .L564
	mov	x1, x2
	mov	x0, x3
	mov	w2, 1024
	bl	ftl_memcpy
.L564:
	cbz	x20, .L559
	mov	w0, 48
	lsr	w19, w19, 1
	mul	w19, w19, w0
	asr	w19, w19, 2
	ldr	w0, [x21,w19,sxtw 2]
	strb	w0, [x20]
	lsr	w1, w0, 8
	strb	w1, [x20,1]
	lsr	w1, w0, 16
	lsr	w0, w0, 24
	strb	w1, [x20,2]
	strb	w0, [x20,3]
.L559:
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	NandcCopy1KB, .-NandcCopy1KB
	.align	2
	.global	NandcXferData
	.type	NandcXferData, %function
NandcXferData:
	stp	x29, x30, [sp, -176]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	uxtb	w24, w0
	adrp	x0, .LANCHOR0
	stp	x27, x28, [sp,80]
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, x24, sxtw 4
	ldr	x19, [x0,40]
	and	x0, x3, 63
	uxtb	w20, w1
	uxtb	w23, w2
	mov	x25, x3
	mov	x21, x4
	cbnz	x0, .L576
	cbnz	x4, .L577
	add	x0, x29, 112
	mov	w1, 255
	mov	w2, 64
	add	x21, x29, 112
	bl	ftl_memset
.L577:
	mov	w0, w24
	mov	w1, w20
	mov	w2, w23
	mov	w3, 0
	mov	x4, x25
	mov	x5, x21
	mov	w22, 0
	bl	NandcXferStart
	mov	w0, w24
	bl	NandcXferComp
	cbnz	w20, .L578
	adrp	x0, .LANCHOR2
	ubfx	x3, x23, 1, 7
	add	x2, x0, :lo12:.LANCHOR2
	mov	w4, 128
	add	x3, x21, x3, lsl 2
	ldr	w1, [x2,652]
	cmp	w1, 25
	mov	w1, 64
	csel	w4, w1, w4, cc
	mov	w1, w20
.L580:
	cmp	x21, x3
	add	w5, w1, w4
	beq	.L622
	ldr	x6, [x2,608]
	and	x1, x1, 4294967292
	add	x21, x21, 4
	ldr	w1, [x6,x1]
	strb	w1, [x21,-4]
	lsr	w6, w1, 8
	strb	w6, [x21,-3]
	lsr	w6, w1, 16
	strb	w6, [x21,-2]
	lsr	w1, w1, 24
	strb	w1, [x21,-1]
	mov	w1, w5
	b	.L580
.L622:
	add	x0, x0, :lo12:.LANCHOR2
	lsr	w23, w23, 2
	ldr	w4, [x0,652]
	ldr	w5, [x0,580]
	mov	w0, 0
	mov	w22, w0
.L582:
	cmp	w0, w23
	bcs	.L578
	cbz	w4, .L578
	uxtw	x1, w0
	add	x1, x1, 8
	ldr	w1, [x19,x1,lsl 2]
	str	w1, [x29,104]
	ldr	w1, [x29,104]
	tbnz	x1, 2, .L606
	ldr	w3, [x29,104]
	ubfx	x3, x3, 15, 1
	cbnz	w3, .L606
	cmp	w5, 5
	bls	.L584
	ldr	w2, [x29,104]
	ubfx	x6, x2, 3, 5
	ldr	w2, [x29,104]
	ldr	w1, [x29,104]
	ubfx	x2, x2, 27, 1
	ubfx	x3, x1, 16, 5
	ldr	w1, [x29,104]
	orr	w2, w6, w2, lsl 5
	ubfx	x1, x1, 29, 1
	orr	w1, w3, w1, lsl 5
	ldr	w3, [x29,104]
	cmp	w2, w1
	bls	.L585
	ldr	w1, [x29,104]
	ubfx	x3, x3, 3, 5
	ubfx	x1, x1, 27, 1
	orr	w3, w3, w1, lsl 5
	b	.L586
.L585:
	ubfx	x1, x3, 16, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 29, 1
	b	.L621
.L584:
	cmp	w5, 3
	bls	.L586
	ldr	w2, [x29,104]
	ubfx	x6, x2, 3, 5
	ldr	w2, [x29,104]
	ldr	w1, [x29,104]
	ubfx	x2, x2, 28, 1
	ubfx	x3, x1, 16, 5
	ldr	w1, [x29,104]
	orr	w2, w6, w2, lsl 5
	ubfx	x1, x1, 30, 1
	orr	w1, w3, w1, lsl 5
	ldr	w3, [x29,104]
	cmp	w2, w1
	bls	.L587
	ubfx	x1, x3, 3, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 28, 1
	b	.L621
.L587:
	ubfx	x1, x3, 16, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 30, 1
.L621:
	orr	w3, w1, w3, lsl 5
.L586:
	cmp	w22, w3
	csel	w22, w22, w3, cs
	b	.L583
.L606:
	mov	w22, -1
.L583:
	add	w0, w0, 1
	b	.L582
.L578:
	str	wzr, [x19,16]
	b	.L589
.L576:
	cmp	w20, 1
	bne	.L620
	mov	w22, 0
	mov	w27, 2
.L590:
	cmp	w22, w23
	bcs	.L623
	and	w26, w22, 3
	mov	x3, x25
	cbz	x25, .L592
	ubfiz	x0, x22, 9, 23
	add	x3, x25, x0
.L592:
	cmp	x21, xzr
	mov	x0, x19
	csel	w4, w27, wzr, ne
	mov	w1, 1
	mov	w2, w26
	mul	w4, w4, w22
	add	w22, w22, 2
	add	x4, x21, x4
	bl	NandcCopy1KB
	mov	x4, 0
	mov	w0, w24
	mov	w1, 1
	mov	w2, 2
	mov	w3, w26
	mov	x5, x4
	bl	NandcXferStart
	mov	w0, w24
	bl	NandcXferComp
	b	.L590
.L623:
	mov	w22, 0
	b	.L589
.L620:
	mov	w1, 0
	mov	x4, 0
	mov	w26, 0
	mov	w0, w24
	mov	w2, 2
	mov	w3, w1
	mov	x5, x4
	mov	w22, w26
	mov	w27, 2
	bl	NandcXferStart
.L595:
	cmp	w26, w23
	bcs	.L589
	mov	w0, w24
	add	w28, w26, 2
	bl	NandcXferComp
	ldr	w0, [x19,32]
	cmp	w28, w23
	str	w0, [x29,104]
	bcs	.L596
	mov	x4, 0
	mov	w0, w24
	mov	w1, 0
	mov	w2, 2
	and	w3, w28, 3
	mov	x5, x4
	bl	NandcXferStart
.L596:
	ldr	w0, [x29,104]
	tbnz	x0, 2, .L611
	ldr	w0, [x29,104]
	ubfx	x1, x0, 3, 5
	ldr	w0, [x29,104]
	ubfx	x0, x0, 27, 1
	orr	w0, w1, w0, lsl 5
	cmp	w22, w0
	csel	w22, w22, w0, cs
	b	.L597
.L611:
	mov	w22, -1
.L597:
	and	w2, w26, 3
	mov	x3, x25
	cbz	x25, .L598
	ubfiz	x3, x26, 9, 23
	add	x3, x25, x3
.L598:
	cmp	x21, xzr
	mov	x0, x19
	csel	w4, w27, wzr, ne
	mov	w1, 0
	mul	w4, w4, w26
	mov	w26, w28
	add	x4, x21, x4
	bl	NandcCopy1KB
	b	.L595
.L589:
	cbnz	w20, .L601
	adrp	x0, .LANCHOR2+580
	ldr	w0, [x0,#:lo12:.LANCHOR2+580]
	cmp	w0, 5
	bls	.L601
	ldr	w0, [x19]
	mov	w1, 8192
	movk	w1, 0x2, lsl 16
	and	w1, w0, w1
	cmp	w1, 139264
	bne	.L601
	orr	w0, w0, 131072
	mov	w22, -1
	str	w0, [x19]
.L601:
	mov	w0, w22
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 176
	ret
	.size	NandcXferData, .-NandcXferData
	.align	2
	.global	FlashReadRawPage
	.type	FlashReadRawPage, %function
FlashReadRawPage:
	stp	x29, x30, [sp, -64]!
	mov	x4, x3
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxtb	w19, w0
	adrp	x0, .LANCHOR1+481
	str	x21, [sp,32]
	mov	x21, x2
	ldrb	w20, [x0,#:lo12:.LANCHOR1+481]
	cbnz	w19, .L625
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w5, [x0,25]
	ldr	w0, [x0,28]
	mul	w0, w5, w0
	cmp	w1, w0
	mov	w0, 4
	csel	w20, w20, w0, cs
.L625:
	mov	w0, w19
	str	x4, [x29,48]
	str	x1, [x29,56]
	bl	NandcWaitFlashReady
	mov	w0, w19
	bl	NandcFlashCs
	ldr	x1, [x29,56]
	mov	w0, w19
	bl	FlashReadCmd
	mov	w0, w19
	bl	NandcWaitFlashReady
	ldr	x4, [x29,48]
	mov	w2, w20
	mov	x3, x21
	mov	w1, 0
	mov	w0, w19
	bl	NandcXferData
	mov	w20, w0
	mov	w0, w19
	bl	NandcFlashDeCs
	ldr	x21, [sp,32]
	mov	w0, w20
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 64
	ret
	.size	FlashReadRawPage, .-FlashReadRawPage
	.align	2
	.global	FlashDdrTunningRead
	.type	FlashDdrTunningRead, %function
FlashDdrTunningRead:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	adrp	x19, .LANCHOR2
	add	x20, x20, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	uxtb	w24, w0
	add	x27, x19, :lo12:.LANCHOR2
	ldr	x0, [x20,3976]
	mov	w22, 6
	mov	w25, w1
	mov	x28, x2
	mov	x23, x3
	mov	w26, w4
	ldr	w0, [x0,304]
	mov	w21, 1024
	str	w0, [x29,124]
	ldr	w0, [x27,580]
	cmp	w0, 8
	mov	w0, 12
	csel	w22, w22, w0, cc
	cbz	w4, .L628
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	mov	w0, w24
	bl	FlashReset
	mov	w1, w25
	mov	x2, x28
	mov	x3, x23
	mov	w0, w24
	bl	FlashReadRawPage
	mov	w21, w0
	ldrb	w0, [x27,564]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x27,564]
	bl	NandcSetMode
	cmn	w21, #1
	bne	.L629
.L638:
	mov	w21, -1
	b	.L630
.L629:
	adrp	x0, .LC7
	mov	w1, w25
	add	x0, x0, :lo12:.LC7
	mov	w2, w21
	bl	printk
	cmp	w21, 9
	bhi	.L631
	add	x20, x20, x24, sxtw 4
	ldr	x0, [x20,40]
	ldr	w1, [x0,3840]
	ldr	w1, [x0]
	orr	w1, w1, 131072
	str	w1, [x0]
.L631:
	add	x19, x19, :lo12:.LANCHOR2
	ldr	w0, [x19,664]
	add	w0, w0, 1
	str	w0, [x19,664]
	cmp	w0, 2047
	bls	.L630
	mov	x23, 0
	str	wzr, [x19,664]
	mov	x28, x23
.L628:
	mov	w19, 0
	mov	w27, -1
	mov	w5, w19
	mov	w6, w19
	mov	w20, w19
.L636:
	mov	w0, w22
	str	x5, [x29,104]
	str	x6, [x29,112]
	bl	NandcSetDdrPara
	mov	w1, w25
	mov	w0, w24
	mov	x2, x28
	mov	x3, x23
	bl	FlashReadRawPage
	add	w1, w21, 1
	ldr	x6, [x29,112]
	cmp	w0, w1
	ldr	x5, [x29,104]
	bhi	.L632
	cmp	w0, 2
	bhi	.L642
	add	w20, w20, 1
	cmp	w20, 9
	bls	.L642
	sub	w19, w22, w20
	mov	w21, w0
	mov	w27, 0
	b	.L634
.L632:
	cmp	w6, w20
	bcs	.L643
	cmp	w20, 7
	sub	w5, w19, w20
	bhi	.L644
	mov	w6, w20
	b	.L643
.L642:
	mov	x23, 0
	mov	w19, w22
	mov	w21, w0
	mov	w27, 0
	mov	x28, x23
	b	.L633
.L643:
	mov	w20, 0
.L633:
	add	w22, w22, 2
	cmp	w22, 69
	bls	.L636
.L634:
	cmp	w6, w20
	csel	w19, w19, w5, cc
	b	.L635
.L644:
	mov	w19, w5
.L635:
	cbz	w19, .L637
	adrp	x0, .LC8
	mov	w1, w19
	add	x0, x0, :lo12:.LC8
	bl	printk
	mov	w0, w19
	bl	NandcSetDdrPara
.L637:
	cbz	w27, .L630
	adrp	x0, .LC9
	mov	w1, w24
	add	x0, x0, :lo12:.LC9
	mov	w2, w25
	bl	printk
	cbz	w26, .L638
	ldr	w0, [x29,124]
	lsr	w0, w0, 8
	bl	NandcSetDdrPara
.L630:
	mov	w0, w21
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	FlashDdrTunningRead, .-FlashDdrTunningRead
	.align	2
	.global	FlashReadPage
	.type	FlashReadPage, %function
FlashReadPage:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxtb	w20, w0
	str	x27, [sp,80]
	mov	w0, w20
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	mov	w22, w1
	mov	x25, x2
	mov	x24, x3
	bl	FlashReadRawPage
	cmn	w0, #1
	mov	w19, w0
	adrp	x23, .LANCHOR2
	bne	.L654
	adrp	x21, .LANCHOR0
	add	x26, x21, :lo12:.LANCHOR0
	ldrb	w27, [x26,32]
	cbnz	w27, .L655
.L657:
	add	x0, x23, :lo12:.LANCHOR2
	ldrb	w0, [x0,572]
	cbz	w0, .L654
	b	.L675
.L655:
	mov	w0, w20
	mov	w1, w22
	mov	x2, x25
	mov	x3, x24
	strb	wzr, [x26,32]
	bl	FlashReadRawPage
	strb	w27, [x26,32]
	cmn	w0, #1
	beq	.L657
	mov	w19, w0
	b	.L654
.L675:
	add	x21, x21, :lo12:.LANCHOR0
	mov	w1, w22
	mov	x2, x25
	mov	x3, x24
	mov	w4, 1
	ldr	x0, [x21,3976]
	ldr	w26, [x0,304]
	mov	w0, w20
	bl	FlashDdrTunningRead
	cmn	w0, #1
	mov	w19, w0
	beq	.L658
	ldrb	w0, [x21,4045]
	cmp	w19, w0, lsr 1
	bls	.L654
.L658:
	lsr	w0, w26, 8
	bl	NandcSetDdrPara
.L654:
	add	x21, x23, :lo12:.LANCHOR2
	cmn	w19, #1
	ldr	x4, [x21,672]
	bne	.L659
	cbz	x4, .L659
	mov	w1, w22
	mov	x2, x25
	mov	x3, x24
	mov	w0, w20
	blr	x4
	mov	w19, w0
	adrp	x0, .LC10
	mov	w1, w19
	add	x0, x0, :lo12:.LC10
	mov	w2, w20
	mov	w3, w22
	bl	printk
	cmn	w19, #1
	bne	.L659
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cbz	w0, .L659
	mov	w0, w20
	bl	flash_enter_slc_mode
	ldr	x4, [x21,672]
	mov	w0, w20
	mov	w1, w22
	mov	x2, x25
	mov	x3, x24
	blr	x4
	mov	w19, w0
	mov	w0, w20
	bl	flash_exit_slc_mode
.L659:
	mov	w0, w19
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	FlashReadPage, .-FlashReadPage
	.align	2
	.global	FlashDdrParaScan
	.type	FlashDdrParaScan, %function
FlashDdrParaScan:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	mov	w20, w1
	stp	x21, x22, [sp,32]
	add	x22, x19, :lo12:.LANCHOR2
	uxtb	w21, w0
	ldrb	w0, [x22,564]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x22,564]
	bl	NandcSetMode
	mov	x2, 0
	mov	w1, w20
	mov	x3, x2
	mov	w4, w2
	mov	w0, w21
	bl	FlashDdrTunningRead
	mov	w22, w0
	mov	x2, 0
	mov	w0, w21
	mov	w1, w20
	mov	x3, x2
	bl	FlashReadRawPage
	cmn	w0, #1
	beq	.L680
	cmn	w22, #1
	bne	.L677
.L680:
	add	x20, x19, :lo12:.LANCHOR2
	ldrb	w0, [x20,564]
	tbz	x0, 0, .L677
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	strb	wzr, [x20,572]
	b	.L679
.L677:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, 1
	strb	w0, [x19,572]
.L679:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashDdrParaScan, .-FlashDdrParaScan
	.align	2
	.global	FlashLoadPhyInfo
	.type	FlashLoadPhyInfo, %function
FlashLoadPhyInfo:
	stp	x29, x30, [sp, -112]!
	mov	w0, 60
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	str	x27, [sp,80]
	strb	w0, [x29,104]
	mov	w0, 40
	strb	w0, [x29,105]
	mov	w0, 24
	strb	w0, [x29,106]
	mov	w0, 16
	strb	w0, [x29,107]
	add	x0, x19, :lo12:.LANCHOR0
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	adrp	x23, .LANCHOR2
	adrp	x22, .LANCHOR1
	ldr	w25, [x0,28]
	add	x0, x23, :lo12:.LANCHOR2
	ldr	x1, [x19,#:lo12:.LANCHOR0]
	add	x22, x22, :lo12:.LANCHOR1
	mov	w20, 0
	mov	w24, 4
	str	x1, [x0,680]
	mov	w26, -1
	str	wzr, [x0,688]
	add	x22, x22, 472
	mov	w0, 0
	bl	flash_enter_slc_mode
.L688:
	add	w27, w20, 1
	mov	x21, 0
.L690:
	add	x0, x29, 104
	ldrb	w0, [x21,x0]
	bl	FlashBchSel
	ldr	x2, [x19,#:lo12:.LANCHOR0]
	mov	w0, 0
	mov	w1, w20
	mov	x3, 0
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L689
	ldr	x2, [x19,#:lo12:.LANCHOR0]
	mov	w0, 0
	mov	w1, w27
	mov	x3, 0
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L689
	add	x21, x21, 1
	cmp	x21, 4
	beq	.L691
	b	.L690
.L692:
	mov	w1, 2036
	add	x0, x27, 12
	mov	w26, -1
	bl	js_hash
	ldr	w1, [x27,8]
	cmp	w1, w0
	beq	.L702
.L691:
	subs	w24, w24, #1
	add	w20, w20, w25
	bne	.L688
	mov	w0, w24
.L701:
	bl	flash_exit_slc_mode
	ldr	x27, [sp,80]
	mov	w0, w26
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 112
	ret
.L689:
	add	x21, x23, :lo12:.LANCHOR2
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	ldr	x27, [x21,680]
	ldr	w1, [x27]
	cmp	w1, w0
	bne	.L691
	cbnz	w26, .L692
	add	x19, x19, :lo12:.LANCHOR0
	ldr	w0, [x19,28]
	udiv	w20, w20, w0
	mov	w0, w26
	add	w20, w20, 1
	str	w20, [x19,8]
	b	.L701
.L702:
	add	x1, x27, 160
	mov	w2, 32
	mov	x0, x22
	add	x26, x19, :lo12:.LANCHOR0
	bl	ftl_memcpy
	ldr	x1, [x21,680]
	mov	w2, 32
	add	x0, x26, 104
	add	x1, x1, 192
	bl	ftl_memcpy
	ldr	x1, [x21,680]
	mov	w2, 852
	add	x0, x26, 4056
	add	x1, x1, 224
	bl	ftl_memcpy
	ldrh	w0, [x22,10]
	bl	FlashBlockAlignInit
	str	w20, [x21,688]
	ldr	w1, [x26,28]
	ldr	x2, [x21,680]
	udiv	w1, w20, w1
	ldr	w0, [x2,1076]
	add	w1, w1, 1
	strb	w0, [x21,572]
	cmp	w1, 1
	bls	.L694
	str	w1, [x26,8]
	b	.L695
.L694:
	mov	w0, 2
	str	w0, [x26,8]
.L695:
	add	x0, x23, :lo12:.LANCHOR2
	ldrh	w1, [x2,14]
	mov	w26, 0
	strb	w1, [x0,692]
	b	.L691
	.size	FlashLoadPhyInfo, .-FlashLoadPhyInfo
	.align	2
	.global	ToshibaReadRetrial
	.type	ToshibaReadRetrial, %function
ToshibaReadRetrial:
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	uxtb	w23, w0
	mov	w0, w23
	str	w1, [x29,132]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	mov	x27, x2
	mov	x26, x3
	bl	NandcWaitFlashReady
	adrp	x20, g_retryMode
	adrp	x2, .LANCHOR0
	sbfiz	x1, x23, 4, 32
	add	x0, x2, :lo12:.LANCHOR0
	str	x2, [x29,120]
	add	x0, x0, 40
	mov	w22, 0
	add	x3, x0, x1
	ldr	x24, [x0,x1]
	ldrb	w0, [x3,8]
	str	x0, [x29,136]
	add	x19, x0, 8
	mov	x28, x0
	ldrb	w0, [x20,#:lo12:g_retryMode]
	add	x19, x24, x19, lsl 8
	sub	w0, w0, #67
	uxtb	w0, w0
	cmp	w0, 1
	bls	.L704
	adrp	x0, .LANCHOR2+572
	ldrb	w0, [x0,#:lo12:.LANCHOR2+572]
	cbz	w0, .L705
	mov	w0, w22
	mov	w22, 1
	bl	NandcSetDdrMode
.L705:
	ubfiz	x0, x28, 8, 8
	mov	w2, 92
	add	x0, x24, x0
	str	w2, [x0,2056]
	mov	w2, 197
	str	w2, [x0,2056]
.L704:
	adrp	x0, g_maxRetryCount
	mov	w21, 1
	add	x0, x0, :lo12:g_maxRetryCount
	str	x0, [x29,112]
	ldr	x0, [x29,136]
	mov	w25, -1
	add	x0, x0, 8
	lsl	x0, x0, 8
	str	x0, [x29,104]
	ubfiz	x0, x28, 8, 8
	str	x0, [x29,96]
.L706:
	adrp	x0, g_maxRetryCount
	ldrb	w0, [x0,#:lo12:g_maxRetryCount]
	add	w0, w0, 1
	cmp	w21, w0
	bcs	.L731
	ldrb	w0, [x20,#:lo12:g_retryMode]
	mov	w1, w21
	sub	w0, w0, #67
	uxtb	w0, w0
	cmp	w0, 1
	mov	x0, x19
	bhi	.L707
	bl	SandiskSetRRPara
	b	.L708
.L707:
	bl	ToshibaSetRRPara
.L708:
	ldrb	w0, [x20,#:lo12:g_retryMode]
	cmp	w0, 34
	bne	.L709
	ldr	x0, [x29,112]
	ldrb	w0, [x0]
	sub	w0, w0, #3
	cmp	w21, w0
	bne	.L709
	ldr	x0, [x29,104]
	mov	w1, 179
	add	x0, x24, x0
	str	w1, [x0,8]
.L709:
	ldr	x0, [x29,96]
	mov	w1, 38
	add	x0, x24, x0
	str	w1, [x0,2056]
	mov	w1, 93
	str	w1, [x0,2056]
	cbz	w22, .L710
	mov	w0, 4
	bl	NandcSetDdrMode
	ldr	w1, [x29,132]
	mov	w0, w23
	mov	x2, x27
	mov	x3, x26
	bl	FlashReadRawPage
	mov	w28, w0
	mov	w0, 0
	bl	NandcSetDdrMode
	b	.L711
.L710:
	ldr	w1, [x29,132]
	mov	w0, w23
	mov	x2, x27
	mov	x3, x26
	bl	FlashReadRawPage
	mov	w28, w0
.L711:
	cmn	w28, #1
	beq	.L712
	ldr	x0, [x29,120]
	cmn	w25, #1
	csel	w25, w25, w28, ne
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,4045]
	add	w0, w0, w0, lsl 1
	cmp	w28, w0, lsr 2
	bcc	.L714
	mov	x26, 0
	mov	x27, x26
.L712:
	add	w21, w21, 1
	b	.L706
.L731:
	mov	w28, w25
.L714:
	ldrb	w0, [x20,#:lo12:g_retryMode]
	mov	w1, 0
	sub	w0, w0, #67
	uxtb	w0, w0
	cmp	w0, 1
	mov	x0, x19
	bhi	.L716
	bl	SandiskSetRRPara
	b	.L717
.L716:
	bl	ToshibaSetRRPara
.L717:
	ldr	x0, [x29,136]
	add	x0, x0, 8
	add	x24, x24, x0, lsl 8
	mov	w0, 255
	str	w0, [x24,8]
	ldr	x0, [x29,120]
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,4045]
	add	w0, w0, w0, lsl 1
	cmp	w28, w0, lsr 2
	bcc	.L718
	cmn	w28, #1
	mov	w0, 256
	csel	w28, w28, w0, eq
.L718:
	mov	w0, w23
	bl	NandcWaitFlashReady
	cbz	w22, .L719
	mov	w0, 4
	bl	NandcSetDdrMode
.L719:
	mov	w0, w28
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
	.size	ToshibaReadRetrial, .-ToshibaReadRetrial
	.align	2
	.global	SamsungReadRetrial
	.type	SamsungReadRetrial, %function
SamsungReadRetrial:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	uxtb	w22, w0
	str	x27, [sp,80]
	mov	w0, w22
	adrp	x27, g_maxRetryCount
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	mov	x24, x2
	mov	w25, w1
	mov	x23, x3
	adrp	x20, .LANCHOR0
	bl	NandcWaitFlashReady
	mov	w26, 1
	add	x0, x20, :lo12:.LANCHOR0
	sbfiz	x1, x22, 4, 32
	add	x0, x0, 40
	mov	w19, -1
	add	x2, x0, x1
	add	x27, x27, :lo12:g_maxRetryCount
	ldr	x0, [x0,x1]
	ldrb	w21, [x2,8]
	add	x21, x21, 8
	add	x21, x0, x21, lsl 8
.L733:
	ldrb	w0, [x27]
	add	w0, w0, 1
	cmp	w26, w0
	bcs	.L736
	mov	x0, x21
	mov	w1, w26
	bl	SamsungSetRRPara
	mov	w0, w22
	mov	w1, w25
	mov	x2, x24
	mov	x3, x23
	bl	FlashReadRawPage
	cmn	w0, #1
	beq	.L734
	add	x1, x20, :lo12:.LANCHOR0
	cmn	w19, #1
	csel	w19, w19, w0, ne
	ldrb	w1, [x1,4045]
	add	w1, w1, w1, lsl 1
	cmp	w0, w1, lsr 2
	bcc	.L739
	mov	x23, 0
	mov	x24, x23
.L734:
	add	w26, w26, 1
	b	.L733
.L739:
	mov	w19, w0
.L736:
	mov	x0, x21
	mov	w1, 0
	add	x20, x20, :lo12:.LANCHOR0
	bl	SamsungSetRRPara
	ldrb	w0, [x20,4045]
	add	w0, w0, w0, lsl 1
	cmp	w19, w0, lsr 2
	bcc	.L738
	cmn	w19, #1
	mov	w0, 256
	csel	w19, w19, w0, eq
.L738:
	mov	w0, w19
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	SamsungReadRetrial, .-SamsungReadRetrial
	.align	2
	.global	MicronReadRetrial
	.type	MicronReadRetrial, %function
MicronReadRetrial:
	stp	x29, x30, [sp, -160]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	stp	x21, x22, [sp,32]
	uxtb	w21, w0
	add	x0, x20, :lo12:.LANCHOR0
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x23, x24, [sp,48]
	mov	w28, w1
	mov	x26, x2
	ldrb	w19, [x0,4045]
	mov	x22, x3
	ldrb	w0, [x0,4044]
	cbnz	w0, .L746
	add	w19, w19, w19, lsl 1
	ubfx	x19, x19, 2, 8
	b	.L747
.L746:
	mov	w0, 3
	sdiv	w19, w19, w0
.L747:
	adrp	x0, g_maxRetryCount
	adrp	x27, .LC11
	add	x0, x0, :lo12:g_maxRetryCount
	mov	w25, 0
	str	x0, [x29,152]
	add	x0, x27, :lo12:.LC11
	str	x0, [x29,144]
	add	x20, x20, :lo12:.LANCHOR0
	add	x23, x20, 40
	add	x23, x23, x21, sxtw 4
.L758:
	mov	w0, w21
	mov	w24, 0
	mov	w27, -1
	bl	NandcWaitFlashReady
	ldrb	w4, [x23,8]
	mov	w9, 239
	ldr	x6, [x23]
	mov	w10, 137
	lsl	x8, x4, 8
.L748:
	ldr	x0, [x29,152]
	ldrb	w0, [x0]
	cmp	w24, w0
	bcs	.L751
	add	x2, x6, x8
	mov	x0, 1000
	str	x4, [x29,96]
	str	x8, [x29,104]
	str	w9, [x2,2056]
	str	w10, [x2,2052]
	str	x6, [x29,112]
	str	x9, [x29,120]
	str	x10, [x29,128]
	str	x2, [x29,136]
	bl	__const_udelay
	ldr	x2, [x29,136]
	add	w7, w24, 1
	mov	w0, w21
	mov	w1, w28
	mov	x3, x22
	str	x7, [x29,136]
	str	w7, [x2,2048]
	str	wzr, [x2,2048]
	str	wzr, [x2,2048]
	str	wzr, [x2,2048]
	mov	x2, x26
	bl	FlashReadRawPage
	cmn	w0, #1
	ldr	x7, [x29,136]
	ldr	x10, [x29,128]
	ldr	x9, [x29,120]
	ldr	x6, [x29,112]
	ldr	x8, [x29,104]
	ldr	x4, [x29,96]
	beq	.L749
	cmn	w27, #1
	csel	w27, w27, w0, ne
	cmp	w0, w19
	bcc	.L760
	mov	x22, 0
	mov	x26, x22
.L749:
	mov	w24, w7
	b	.L748
.L760:
	mov	x22, 0
	mov	w27, w0
	mov	x26, x22
.L751:
	add	x1, x6, x4, lsl 8
	mov	w0, 239
	str	x1, [x29,136]
	str	w0, [x1,2056]
	mov	w0, 137
	str	w0, [x1,2052]
	mov	x0, 1000
	bl	__const_udelay
	ldr	x1, [x29,136]
	cmp	w27, w19
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
	bcc	.L753
	cmn	w27, #1
	mov	w0, 256
	csel	w27, w27, w0, eq
.L753:
	cmn	w27, #1
	cset	w6, eq
	cbnz	w6, .L762
	cmp	w27, 256
	cset	w1, eq
	cbz	w1, .L754
.L762:
	ldr	x0, [x29,144]
	mov	w1, w24
	mov	w2, w28
	mov	w3, w24
	mov	w4, w27
	str	x6, [x29,136]
	bl	printk
	cbnz	w25, .L756
	ldr	x6, [x29,136]
	cbz	w6, .L767
	ldrb	w0, [x20,4044]
	cbz	w0, .L767
	mov	w0, w21
	mov	w1, 3
	mov	w25, 1
	bl	micron_auto_read_calibration_config
	b	.L758
.L756:
	mov	w0, w21
	mov	w1, 0
	bl	micron_auto_read_calibration_config
	cmn	w27, #1
	mov	w0, 256
	csel	w27, w27, w0, eq
	b	.L767
.L754:
	cbz	w25, .L767
	mov	w0, w21
	mov	w27, 256
	bl	micron_auto_read_calibration_config
.L767:
	mov	w0, w27
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 160
	ret
	.size	MicronReadRetrial, .-MicronReadRetrial
	.align	2
	.global	HynixReadRetrial
	.type	HynixReadRetrial, %function
HynixReadRetrial:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR0
	uxtb	x22, w0
	add	x0, x21, :lo12:.LANCHOR0
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x19, x20, [sp,16]
	mov	x26, x2
	add	x2, x0, 4056
	ldrb	w25, [x0,4058]
	mov	w27, w1
	ldr	x0, [x0,1912]
	add	x1, x2, x22
	mov	x23, x22
	mov	x24, x3
	ldrb	w0, [x0,19]
	ldrb	w19, [x1,12]
	sub	w0, w0, #7
	uxtb	w0, w0
	cmp	w0, 1
	bhi	.L776
	ldrb	w19, [x1,20]
.L776:
	mov	w0, w23
	mov	w28, 0
	mov	w20, -1
	bl	NandcWaitFlashReady
	add	x4, x21, :lo12:.LANCHOR0
	add	x5, x4, 4060
.L777:
	cmp	w28, w25
	bcs	.L781
	add	w19, w19, 1
	ldrb	w1, [x4,4057]
	mov	x2, x5
	mov	w0, w23
	uxtb	w19, w19
	str	x4, [x29,96]
	cmp	w19, w25
	str	x5, [x29,104]
	csel	w19, w19, wzr, cc
	mov	w3, w19
	bl	HynixSetRRPara
	mov	w0, w23
	mov	w1, w27
	mov	x2, x26
	mov	x3, x24
	bl	FlashReadRawPage
	cmn	w0, #1
	ldr	x5, [x29,104]
	ldr	x4, [x29,96]
	beq	.L779
	ldrb	w1, [x4,4045]
	cmn	w20, #1
	csel	w20, w20, w0, ne
	add	w1, w1, w1, lsl 1
	cmp	w0, w1, lsr 2
	bcc	.L786
	mov	x24, 0
	mov	x26, x24
.L779:
	add	w28, w28, 1
	b	.L777
.L786:
	mov	w20, w0
.L781:
	add	x0, x21, :lo12:.LANCHOR0
	ldr	x1, [x0,1912]
	add	x0, x0, 4056
	add	x0, x0, x22
	ldrb	w1, [x1,19]
	sub	w1, w1, #7
	uxtb	w1, w1
	cmp	w1, 1
	bhi	.L783
	strb	w19, [x0,20]
	b	.L784
.L783:
	strb	w19, [x0,12]
.L784:
	add	x21, x21, :lo12:.LANCHOR0
	ldrb	w0, [x21,4045]
	add	w0, w0, w0, lsl 1
	cmp	w20, w0, lsr 2
	bcc	.L785
	cmn	w20, #1
	mov	w0, 256
	csel	w20, w20, w0, eq
.L785:
	mov	w0, w20
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	HynixReadRetrial, .-HynixReadRetrial
	.align	2
	.global	FlashProgPage
	.type	FlashProgPage, %function
FlashProgPage:
	stp	x29, x30, [sp, -64]!
	mov	x4, x3
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxtb	w19, w0
	adrp	x0, .LANCHOR1+481
	stp	x21, x22, [sp,32]
	mov	w20, w1
	mov	x22, x2
	ldrb	w21, [x0,#:lo12:.LANCHOR1+481]
	cbnz	w19, .L793
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w1, [x0,25]
	ldr	w2, [x0,28]
	mul	w1, w1, w2
	cmp	w20, w1
	bcs	.L793
	ldrb	w0, [x0,24]
	cbnz	w0, .L794
	sub	w21, w21, #2
	b	.L793
.L794:
	mov	w21, 4
.L793:
	mov	w0, w19
	str	x4, [x29,56]
	bl	NandcWaitFlashReady
	mov	w0, w19
	bl	NandcFlashCs
	mov	w0, w19
	mov	w1, w20
	bl	FlashProgFirstCmd
	ldr	x4, [x29,56]
	mov	w2, w21
	mov	x3, x22
	mov	w0, w19
	mov	w1, 1
	bl	NandcXferData
	mov	w1, w20
	mov	w0, w19
	bl	FlashProgSecondCmd
	mov	w0, w19
	bl	NandcWaitFlashReady
	mov	w1, w20
	mov	w0, w19
	bl	FlashReadStatus
	mov	w20, w0
	mov	w0, w19
	bl	NandcFlashDeCs
	and	w0, w20, 1
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	FlashProgPage, .-FlashProgPage
	.align	2
	.global	FlashSavePhyInfo
	.type	FlashSavePhyInfo, %function
FlashSavePhyInfo:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	adrp	x22, .LANCHOR0
	adrp	x23, .LANCHOR2
	stp	x19, x20, [sp,16]
	add	x19, x23, :lo12:.LANCHOR2
	add	x20, x22, :lo12:.LANCHOR0
	stp	x25, x26, [sp,64]
	ldr	x0, [x22,#:lo12:.LANCHOR0]
	mov	w21, 0
	str	x0, [x19,680]
	ldrb	w0, [x19,693]
	bl	FlashBchSel
	ldr	x0, [x22,#:lo12:.LANCHOR0]
	mov	w1, 0
	mov	w2, 2048
	bl	ftl_memset
	ldr	x1, [x19,680]
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	mov	w2, 32
	str	w0, [x1]
	ldr	x0, [x19,680]
	ldrb	w1, [x20,1672]
	add	x0, x0, 16
	strh	w1, [x0,-4]
	ldrb	w1, [x20,25]
	strh	w1, [x0,-2]
	ldrb	w1, [x19,572]
	str	w1, [x0,1060]
	adrp	x1, IDByte
	add	x1, x1, :lo12:IDByte
	bl	ftl_memcpy
	ldr	x0, [x19,680]
	add	x1, x20, 1676
	mov	w2, 8
	add	x0, x0, 80
	bl	ftl_memcpy
	ldr	x0, [x19,680]
	add	x1, x20, 1684
	mov	w2, 32
	add	x0, x0, 96
	bl	ftl_memcpy
	ldr	x0, [x19,680]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	mov	w2, 32
	add	x1, x1, 472
	add	x0, x0, 160
	bl	ftl_memcpy
	ldr	x0, [x19,680]
	add	x1, x20, 104
	mov	w2, 32
	add	x0, x0, 192
	bl	ftl_memcpy
	ldr	x0, [x19,680]
	add	x1, x20, 4056
	mov	w2, 852
	add	x0, x0, 224
	bl	ftl_memcpy
	ldr	x20, [x19,680]
	mov	w1, 2036
	add	x0, x20, 12
	bl	js_hash
	str	w0, [x20,8]
	mov	w0, 1592
	str	w0, [x20,4]
	ldr	x0, [x19,696]
	str	x0, [x19,680]
	mov	w0, 0
	mov	w19, w21
	bl	flash_enter_slc_mode
.L798:
	add	x20, x22, :lo12:.LANCHOR0
	mov	w2, 0
	mov	w0, 0
	add	x24, x23, :lo12:.LANCHOR2
	add	w26, w19, 1
	ldr	w1, [x20,28]
	mul	w1, w19, w1
	bl	FlashEraseBlock
	ldr	w1, [x20,28]
	mov	x3, 0
	ldr	x2, [x22,#:lo12:.LANCHOR0]
	mov	w0, 0
	mul	w1, w19, w1
	bl	FlashProgPage
	ldr	w1, [x20,28]
	mov	x3, 0
	ldr	x2, [x22,#:lo12:.LANCHOR0]
	mov	w0, 0
	mul	w1, w19, w1
	add	w1, w1, 1
	bl	FlashProgPage
	ldr	w1, [x20,28]
	mov	w0, 0
	ldr	x2, [x24,696]
	mov	x3, 0
	mul	w1, w19, w1
	bl	FlashReadRawPage
	cmn	w0, #1
	beq	.L796
	ldr	x25, [x24,680]
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	ldr	w1, [x25]
	cmp	w1, w0
	bne	.L796
	mov	w1, 2036
	add	x0, x25, 12
	bl	js_hash
	ldr	w1, [x25,8]
	cmp	w1, w0
	bne	.L796
	ldr	w0, [x20,28]
	cmp	w21, 1
	str	w26, [x20,8]
	mul	w19, w19, w0
	str	w19, [x24,688]
	beq	.L799
	mov	w21, 1
.L796:
	cmp	w26, 4
	mov	w19, w26
	bne	.L798
	b	.L797
.L799:
	mov	w21, 2
.L797:
	mov	w0, 0
	bl	flash_exit_slc_mode
	cmp	w21, wzr
	csetm	w0, eq
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FlashSavePhyInfo, .-FlashSavePhyInfo
	.align	2
	.global	FlashReadIdbDataRaw
	.type	FlashReadIdbDataRaw, %function
FlashReadIdbDataRaw:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x27, x28, [sp,80]
	mov	x27, x0
	mov	w0, 60
	strb	w0, [x29,104]
	mov	w0, 40
	strb	w0, [x29,105]
	mov	w0, 24
	stp	x21, x22, [sp,32]
	strb	w0, [x29,106]
	adrp	x21, .LANCHOR0
	mov	w0, 16
	strb	w0, [x29,107]
	add	x0, x21, :lo12:.LANCHOR0
	adrp	x22, .LANCHOR2
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	ldrb	w28, [x0,4045]
	add	x0, x22, :lo12:.LANCHOR2
	ldr	w0, [x0,568]
	cbz	w0, .L805
	mov	w0, 0
	bl	flash_enter_slc_mode
.L805:
	adrp	x25, .LC12
	mov	x0, x27
	mov	w1, 0
	mov	w2, 2048
	mov	w24, -1
	mov	w20, 2
	add	x25, x25, :lo12:.LC12
	bl	ftl_memset
.L806:
	add	x0, x21, :lo12:.LANCHOR0
	ldrb	w0, [x0,25]
	cmp	w20, w0
	bcs	.L810
	mov	x23, 0
.L808:
	add	x0, x29, 104
	add	x19, x21, :lo12:.LANCHOR0
	ldrb	w26, [x23,x0]
	mov	w0, w26
	bl	FlashBchSel
	ldr	w1, [x19,28]
	mov	w0, 0
	ldr	x2, [x21,#:lo12:.LANCHOR0]
	mov	x3, 0
	mul	w1, w20, w1
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L807
	add	x23, x23, 1
	cmp	x23, 4
	bne	.L808
	b	.L809
.L807:
	ldr	x0, [x19]
	ldr	w1, [x0]
	mov	w0, 35899
	movk	w0, 0xfcdc, lsl 16
	cmp	w1, w0
	bne	.L809
	mov	w1, w26
	mov	x0, x25
	bl	printk
	ldr	x1, [x19]
	mov	x0, x27
	mov	w2, 2048
	bl	ftl_memcpy
	ldr	x0, [x19]
	ldr	w0, [x0,512]
	strb	w0, [x19,25]
	ldr	w0, [x19,8]
	cmp	w0, w20
	bls	.L813
	mov	w24, 0
	str	w20, [x19,8]
	bl	FlashSavePhyInfo
.L809:
	add	w20, w20, 1
	b	.L806
.L813:
	mov	w24, 0
.L810:
	mov	w0, w28
	add	x22, x22, :lo12:.LANCHOR2
	bl	FlashBchSel
	ldr	w0, [x22,568]
	cbz	w0, .L817
	mov	w0, 0
	bl	flash_exit_slc_mode
.L817:
	mov	w0, w24
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FlashReadIdbDataRaw, .-FlashReadIdbDataRaw
	.align	2
	.global	FlashInit
	.type	FlashInit, %function
FlashInit:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	adrp	x19, .LANCHOR0
	mov	x22, x0
	mov	w0, 32768
	adrp	x20, .LANCHOR2
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	bl	ftl_malloc
	add	x21, x20, :lo12:.LANCHOR2
	str	x0, [x19,#:lo12:.LANCHOR0]
	mov	w0, 32768
	add	x24, x19, :lo12:.LANCHOR0
	adrp	x23, IDByte
	bl	ftl_malloc
	str	x0, [x21,696]
	mov	w0, 4096
	adrp	x25, .LC13
	add	x24, x24, 40
	mov	w27, 0
	bl	ftl_malloc
	str	x0, [x21,704]
	mov	w0, 32768
	mov	w28, 44
	bl	ftl_malloc
	str	x0, [x21,712]
	mov	w0, 4096
	bl	ftl_malloc
	str	x0, [x21,720]
	mov	w0, 128
	str	w0, [x24,-12]
	mov	w0, 60
	mov	w1, 50
	strb	w0, [x21,693]
	mov	x0, x22
	strb	w1, [x24,-15]
	strb	w1, [x21,692]
	str	wzr, [x24,-32]
	strb	wzr, [x21,572]
	str	wzr, [x21,664]
	add	x21, x23, :lo12:IDByte
	strb	wzr, [x24,-16]
	bl	NandcInit
	add	x0, x25, :lo12:.LC13
	mov	x26, x21
	str	x0, [x29,104]
.L828:
	uxtb	w1, w27
	ldr	x2, [x24]
	mov	w0, w1
	mov	w25, w1
	str	x2, [x29,96]
	ldrb	w22, [x24,8]
	bl	FlashReset
	mov	w0, w25
	ubfiz	x22, x22, 8, 8
	bl	NandcFlashCs
	ldr	x2, [x29,96]
	mov	w0, 144
	add	x22, x2, x22
	str	w0, [x22,2056]
	mov	x0, 1000
	str	wzr, [x22,2052]
	bl	__const_udelay
	ldr	w0, [x22,2048]
	strb	w0, [x21]
	ldr	w0, [x22,2048]
	strb	w0, [x21,1]
	ldr	w0, [x22,2048]
	strb	w0, [x21,2]
	ldr	w0, [x22,2048]
	strb	w0, [x21,3]
	ldr	w0, [x22,2048]
	strb	w0, [x21,4]
	ldr	w0, [x22,2048]
	strb	w0, [x21,5]
	mov	w0, w25
	bl	NandcFlashDeCs
	ldrb	w2, [x21]
	sub	w0, w2, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L824
	ldr	x0, [x29,104]
	add	w1, w27, 1
	ldrb	w3, [x21,1]
	ldrb	w4, [x21,2]
	ldrb	w5, [x21,3]
	ldrb	w6, [x21,4]
	ldrb	w7, [x21,5]
	bl	printk
.L824:
	cbnz	w27, .L825
	ldrb	w0, [x26]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L876
	ldrb	w0, [x26,1]
	cmp	w0, 255
	beq	.L876
	bl	FlashCs123Init
.L825:
	ldrb	w0, [x21]
	cmp	w0, 181
	bne	.L827
	strb	w28, [x21]
.L827:
	add	w27, w27, 1
	add	x24, x24, 16
	cmp	w27, 4
	add	x21, x21, 8
	bne	.L828
	ldrb	w0, [x23,#:lo12:IDByte]
	cmp	w0, 173
	beq	.L829
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w0, [x0,576]
	bl	NandcSetDdrMode
.L829:
	add	x21, x19, :lo12:.LANCHOR0
	mov	w1, 0
	mov	w2, 852
	add	x0, x21, 4056
	bl	ftl_memset
	strb	wzr, [x21,32]
	adrp	x1, .LANCHOR1
	mov	w2, 12336
	add	x0, x1, :lo12:.LANCHOR1
	movk	w2, 0x5638, lsl 16
	add	x0, x0, 472
	str	x0, [x21,1912]
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w3, [x0,584]
	cmp	w3, w2
	bne	.L830
	mov	w2, 1
	str	w2, [x0,568]
.L830:
	add	x5, x23, :lo12:IDByte
	ldrb	w0, [x5,1]
	cmp	w0, 218
	cset	w4, eq
	cmp	w0, 241
	cset	w6, eq
	cmp	w0, 161
	cset	w2, eq
	orr	w2, w6, w2
	orr	w2, w4, w2
	cbnz	w2, .L831
	and	w2, w0, -3
	cmp	w2, 209
	beq	.L831
	cmp	w0, 220
	bne	.L832
	ldrb	w2, [x5,3]
	cmp	w2, 149
	bne	.L832
.L831:
	add	x2, x19, :lo12:.LANCHOR0
	mov	w5, 1
	mov	w6, 16
	add	x7, x23, :lo12:IDByte
	strb	w5, [x2,24]
	add	x5, x20, :lo12:.LANCHOR2
	strb	w6, [x2,25]
	add	x2, x1, :lo12:.LANCHOR1
	strb	w6, [x5,693]
	ldrb	w6, [x23,#:lo12:IDByte]
	strb	w6, [x2,3257]
	strb	w0, [x2,3258]
	cmp	w6, 152
	bne	.L833
	ldrsb	w2, [x7,4]
	tbnz	w2, #31, .L833
	mov	w2, 24
	strb	w2, [x5,693]
.L833:
	cmp	w3, 2049
	beq	.L879
	mov	w2, 12336
	movk	w2, 0x5638, lsl 16
	cmp	w3, w2
	bne	.L834
.L879:
	add	x2, x20, :lo12:.LANCHOR2
	mov	w3, 16
	strb	w3, [x2,693]
.L834:
	cbz	w4, .L836
	add	x0, x1, :lo12:.LANCHOR1
	mov	w2, 2048
	strh	w2, [x0,3270]
	mov	w2, -38
	b	.L920
.L836:
	cmp	w0, 220
	bne	.L838
	add	x0, x1, :lo12:.LANCHOR1
	mov	w2, 4096
	strh	w2, [x0,3270]
	mov	w2, -36
.L920:
	strb	w2, [x0,3258]
	b	.L837
.L838:
	cmp	w0, 211
	bne	.L837
	add	x0, x1, :lo12:.LANCHOR1
	mov	w2, 4096
	strh	w2, [x0,3270]
	mov	w2, 2
	strb	w2, [x0,3269]
.L837:
	add	x21, x1, :lo12:.LANCHOR1
	add	x0, x19, :lo12:.LANCHOR0
	add	x1, x21, 3128
	mov	w2, 32
	add	x0, x0, 104
	bl	ftl_memcpy
	add	x0, x21, 472
	add	x1, x21, 3256
	mov	w2, 32
	bl	ftl_memcpy
.L832:
	add	x21, x19, :lo12:.LANCHOR0
	ldrb	w0, [x21,24]
	cbnz	w0, .L839
	bl	FlashLoadPhyInfoInRam
	cbnz	w0, .L841
	ldr	x0, [x21,1912]
	add	x21, x20, :lo12:.LANCHOR2
	ldrb	w1, [x0,17]
	and	w0, w1, 7
	strb	w0, [x21,564]
	tbnz	x1, 0, .L841
	mov	w1, 1
	strb	w1, [x21,572]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x21,564]
	bl	NandcSetMode
.L841:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x1, [x0,1912]
	ldrb	w1, [x1,26]
	strb	w1, [x0,4044]
	bl	FlashLoadPhyInfo
	cbz	w0, .L839
	add	x21, x20, :lo12:.LANCHOR2
	ldr	w0, [x21,576]
	cbz	w0, .L844
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	b	.L916
.L844:
	ldrb	w0, [x21,564]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x21,564]
.L916:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cbz	w0, .L839
	mov	w0, 1
	add	x21, x19, :lo12:.LANCHOR0
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	ldr	x1, [x21,1912]
	adrp	x0, .LC14
	add	x0, x0, :lo12:.LC14
	ldrh	w1, [x1,14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	cmn	w0, #1
	beq	.L826
	bl	FlashDieInfoInit
	ldr	x0, [x21,1912]
	ldrb	w0, [x0,19]
	bl	FlashGetReadRetryDefault
	ldr	x0, [x21,1912]
	ldrh	w1, [x21,1920]
	ldrb	w2, [x0,9]
	add	w1, w1, 4095
	cmp	w2, w1, lsr 12
	blt	.L846
	ldrh	w1, [x0,14]
	add	w1, w1, 255
	cmp	w2, w1, lsr 8
	bge	.L847
.L846:
	ldrh	w1, [x0,14]
	and	w1, w1, -256
	strh	w1, [x0,14]
.L847:
	add	x21, x20, :lo12:.LANCHOR2
	ldrb	w0, [x21,564]
	tst	w0, 6
	beq	.L848
	bl	FlashSavePhyInfo
	ldr	w1, [x21,688]
	mov	w0, 0
	bl	FlashDdrParaScan
.L848:
	bl	FlashSavePhyInfo
.L839:
	add	x21, x19, :lo12:.LANCHOR0
	add	x22, x20, :lo12:.LANCHOR2
	ldr	x2, [x21,1912]
	str	xzr, [x22,672]
	ldrb	w0, [x2,26]
	strb	w0, [x21,4044]
	ldrh	w0, [x2,16]
	ldrh	w3, [x2,10]
	ubfx	x1, x0, 7, 1
	strb	w1, [x21,32]
	ubfx	x1, x0, 3, 1
	strb	w1, [x22,728]
	ubfx	x1, x0, 4, 1
	strb	w1, [x22,656]
	ldrb	w1, [x2,12]
	ubfx	x0, x0, 8, 3
	strb	w0, [x22,564]
	ldrb	w0, [x2,18]
	sdiv	w1, w3, w1
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	ldr	x0, [x21,1912]
	ldrh	w1, [x0,16]
	tbz	x1, 6, .L850
	adrp	x1, g_retryMode
	ldrb	w0, [x0,19]
	ldrb	w2, [x21,4057]
	ldrb	w3, [x21,4058]
	strb	w0, [x1,#:lo12:g_retryMode]
	adrp	x1, g_maxRegNum
	strb	w2, [x1,#:lo12:g_maxRegNum]
	adrp	x2, g_maxRetryCount
	strb	w3, [x2,#:lo12:g_maxRetryCount]
	sub	w3, w0, #1
	uxtb	w3, w3
	cmp	w3, 7
	mov	x3, x2
	bhi	.L851
	adrp	x1, HynixReadRetrial
	cmp	w0, 8
	add	x1, x1, :lo12:HynixReadRetrial
	str	x1, [x22,672]
	sub	w1, w0, #5
	cset	w2, eq
	uxtb	w1, w1
	cmp	w1, 1
	bls	.L880
	cbz	w2, .L852
.L880:
	add	x1, x20, :lo12:.LANCHOR2
	mov	w3, 1
	str	w3, [x1,648]
.L852:
	cmp	w0, 7
	add	x1, x19, :lo12:.LANCHOR0
	beq	.L878
	cbnz	w2, .L878
	add	x1, x1, 4076
	b	.L855
.L878:
	add	x1, x1, 4084
.L855:
	mov	x2, 0
	mov	w3, w2
.L857:
	ldrsb	w4, [x1,x2]
	add	x2, x2, 1
	cmp	w4, wzr
	csinc	w3, w3, w3, ne
	cmp	x2, 32
	bne	.L857
	cmp	w3, 27
	bls	.L850
	bl	FlashGetReadRetryDefault
	bl	FlashSavePhyInfo
	b	.L850
.L851:
	sub	w4, w0, #17
	uxtb	w4, w4
	cmp	w4, 2
	bhi	.L859
	adrp	x1, MicronReadRetrial
	cmp	w0, 19
	add	x1, x1, :lo12:MicronReadRetrial
	str	x1, [x22,672]
	beq	.L860
	mov	w0, 7
	b	.L918
.L860:
	mov	w0, 15
.L918:
	strb	w0, [x2,#:lo12:g_maxRetryCount]
	b	.L850
.L859:
	sub	w2, w0, #65
	uxtb	w2, w2
	cmp	w2, 1
	bls	.L881
	cmp	w0, 33
	bne	.L861
.L881:
	add	x0, x20, :lo12:.LANCHOR2
	adrp	x2, ToshibaReadRetrial
	add	x2, x2, :lo12:ToshibaReadRetrial
	str	x2, [x0,672]
	mov	w0, 4
	strb	w0, [x1,#:lo12:g_maxRegNum]
	mov	w0, 7
	strb	w0, [x3,#:lo12:g_maxRetryCount]
	b	.L850
.L861:
	sub	w2, w0, #67
	uxtb	w2, w2
	cmp	w2, 1
	sub	w2, w0, #34
	cset	w5, ls
	uxtb	w2, w2
	cmp	w2, 1
	bls	.L882
	cbz	w5, .L863
.L882:
	add	x2, x20, :lo12:.LANCHOR2
	adrp	x4, ToshibaReadRetrial
	add	x4, x4, :lo12:ToshibaReadRetrial
	cmp	w0, 35
	str	x4, [x2,672]
	beq	.L865
	cmp	w0, 68
	beq	.L865
	mov	w0, 7
	b	.L917
.L865:
	mov	w0, 17
.L917:
	strb	w0, [x3,#:lo12:g_maxRetryCount]
	mov	w0, 4
	cbnz	w5, .L919
	mov	w0, 5
.L919:
	strb	w0, [x1,#:lo12:g_maxRegNum]
	b	.L850
.L863:
	cmp	w0, 49
	bne	.L850
	adrp	x0, SamsungReadRetrial
	add	x0, x0, :lo12:SamsungReadRetrial
	str	x0, [x22,672]
.L850:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w1, [x0,584]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w1, w0
	bne	.L868
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w2, [x0,4044]
	cbz	w2, .L868
	ldr	x0, [x0,1912]
	strb	wzr, [x0,18]
.L868:
	ldrb	w0, [x23,#:lo12:IDByte]
	cmp	w0, 44
	bne	.L869
	add	x0, x20, :lo12:.LANCHOR2
	ldrb	w0, [x0,572]
	cbz	w0, .L869
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w1, w0
	bne	.L870
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbnz	w0, .L869
.L870:
	add	x0, x20, :lo12:.LANCHOR2
	strb	wzr, [x0,572]
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
.L869:
	add	x1, x20, :lo12:.LANCHOR2
	ldrb	w0, [x1,564]
	tst	w0, 6
	beq	.L871
	ldrb	w1, [x1,572]
	cbnz	w1, .L872
	tbnz	x0, 0, .L871
.L872:
	mov	w0, 0
	add	x20, x20, :lo12:.LANCHOR2
	bl	flash_enter_slc_mode
	ldr	w1, [x20,688]
	mov	w0, 0
	bl	FlashDdrParaScan
	mov	w0, 0
	bl	flash_exit_slc_mode
.L871:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w20, 16
	add	x23, x23, :lo12:IDByte
	ldr	x0, [x19,1912]
	ldrb	w0, [x0,20]
	bl	FlashBchSel
	add	x0, x19, 1928
	bl	FlashReadIdbDataRaw
	strb	w20, [x19,25]
	mov	w0, 18928
	movk	w0, 0x2, lsl 16
	bl	FlashTimingCfg
	ldr	x1, [x19,1912]
	ldrb	w2, [x1,12]
	strh	w2, [x19,4024]
	ldrh	w4, [x1,14]
	ldrb	w2, [x1,7]
	str	w2, [x19,4020]
	ldr	w2, [x23]
	str	w2, [x19,4016]
	ldrb	w2, [x19,1672]
	strh	w2, [x19,4026]
	ldrb	w2, [x1,13]
	strh	w2, [x19,4028]
	strh	w4, [x19,4030]
	ldrh	w2, [x1,10]
	strh	w2, [x19,4032]
	ldrb	w3, [x1,12]
	ldrh	w2, [x1,10]
	sdiv	w2, w2, w3
	strh	w2, [x19,4034]
	ldrb	w3, [x1,9]
	strh	w3, [x19,4036]
	ldrh	w5, [x1,10]
	ldrb	w2, [x1,9]
	mul	w2, w2, w5
	mov	w5, 512
	strh	w5, [x19,4040]
	ldrb	w5, [x19,25]
	strh	w5, [x19,4042]
	uxth	w2, w2
	ldrb	w5, [x19,24]
	strh	w2, [x19,4038]
	cmp	w5, 1
	bne	.L874
	lsl	w2, w2, 1
	lsr	w4, w4, 1
	lsl	w3, w3, 1
	strh	w2, [x19,4038]
	strb	w20, [x19,25]
	mov	w2, 8
	strh	w4, [x19,4030]
	strh	w3, [x19,4036]
	strh	w2, [x19,4042]
.L874:
	ldrb	w0, [x1,20]
	bl	FlashBchSel
	bl	ftl_flash_suspend
	mov	w0, 0
	b	.L826
.L876:
	mov	w0, -2
.L826:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FlashInit, .-FlashInit
	.align	2
	.global	FlashPageProgMsbFFData
	.type	FlashPageProgMsbFFData, %function
FlashPageProgMsbFFData:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxth	w19, w2
	adrp	x2, .LANCHOR0
	stp	x21, x22, [sp,32]
	uxtb	w21, w0
	add	x0, x2, :lo12:.LANCHOR0
	stp	x23, x24, [sp,48]
	str	x25, [sp,64]
	ldr	x3, [x0,1912]
	ldrb	w0, [x0,4044]
	ldrb	w20, [x3,19]
	mov	x3, x2
	cbz	w0, .L922
	adrp	x0, .LANCHOR2+568
	ldr	w0, [x0,#:lo12:.LANCHOR2+568]
	cbnz	w0, .L921
.L922:
	sub	w2, w20, #5
	uxtb	w2, w2
	cmp	w2, 3
	bls	.L923
	cmp	w20, 68
	beq	.L923
	sub	w2, w20, #19
	and	w2, w2, -17
	uxtb	w2, w2
	cbnz	w2, .L921
.L923:
	mov	w24, w1
	add	x22, x3, :lo12:.LANCHOR0
	mov	w25, 65535
	adrp	x23, .LANCHOR2
.L925:
	ldr	x0, [x22,1912]
	ldrh	w0, [x0,10]
	cmp	w0, w19
	bls	.L921
	add	x0, x22, 648
	ldrh	w0, [x0,w19,sxtw 1]
	cmp	w0, w25
	bne	.L921
	add	x0, x23, :lo12:.LANCHOR2
	cmp	w20, 8
	mov	w1, 0
	ldr	x0, [x0,696]
	beq	.L935
	mov	w1, 255
.L935:
	mov	w2, 32768
	bl	ftl_memset
	add	x0, x23, :lo12:.LANCHOR2
	add	w1, w19, w24
	add	w19, w19, 1
	ldr	x2, [x0,696]
	mov	w0, w21
	uxth	w19, w19
	mov	x3, x2
	bl	FlashProgPage
	b	.L925
.L921:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FlashPageProgMsbFFData, .-FlashPageProgMsbFFData
	.align	2
	.global	FlashReadSlc2KPages
	.type	FlashReadSlc2KPages, %function
FlashReadSlc2KPages:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	mov	w24, w1
	adrp	x1, .LANCHOR1+481
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR0
	adrp	x23, .LC15
	stp	x25, x26, [sp,64]
	add	x21, x21, :lo12:.LANCHOR0
	ldrb	w25, [x1,#:lo12:.LANCHOR1+481]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	mov	w26, w2
	mov	x19, x0
	mov	w22, 0
	add	x27, x21, 1676
	add	x23, x23, :lo12:.LC15
.L937:
	cmp	w22, w24
	beq	.L971
	mov	w1, w26
	sub	w4, w24, w22
	mov	x0, x19
	add	x2, x29, 124
	add	x3, x29, 120
	bl	LogAddr2PhyAddr
	ldr	w0, [x29,120]
	adrp	x5, .LANCHOR0
	ldrb	w1, [x21,1672]
	cmp	w0, w1
	bcc	.L938
	mov	w0, -1
	str	w0, [x19]
	b	.L939
.L938:
	ldrb	w28, [x27,w0,uxtw]
	str	x5, [x29,96]
	mov	w0, w28
	bl	NandcWaitFlashReady
	mov	w0, w28
	bl	NandcFlashCs
	ldr	w1, [x29,124]
	mov	w0, w28
	bl	FlashReadCmd
	mov	w0, w28
	bl	NandcWaitFlashReady
	ldr	x3, [x19,8]
	mov	w1, 0
	ldr	x4, [x19,16]
	mov	w2, w25
	mov	w0, w28
	bl	NandcXferData
	mov	w6, w0
	ldr	w2, [x21,28]
	mov	w0, w28
	ldr	w1, [x29,124]
	str	x6, [x29,104]
	add	w1, w2, w1
	bl	FlashReadCmd
	ldr	x0, [x19,8]
	mov	w1, 0
	mov	w2, w25
	cmp	x0, xzr
	add	x3, x0, 2048
	ldr	x0, [x19,16]
	csel	x3, x3, xzr, ne
	add	x4, x0, 8
	cmp	x0, xzr
	csel	x4, x4, xzr, ne
	mov	w0, w28
	bl	NandcXferData
	mov	w20, w0
	mov	w0, w28
	bl	NandcFlashDeCs
	ldr	x5, [x29,96]
	ldr	x6, [x29,104]
	add	x0, x5, :lo12:.LANCHOR0
	cmp	w20, w6
	csel	w3, w20, w6, cs
	ldrb	w0, [x0,4045]
	add	w0, w0, w0, lsl 1
	cmp	w3, w0, lsr 2
	bls	.L942
	cmn	w3, #1
	mov	w0, 256
	csel	w3, w3, w0, eq
.L942:
	cmp	w3, 256
	beq	.L953
	cmn	w3, #1
	bne	.L943
.L953:
	str	w3, [x19]
	b	.L945
.L943:
	str	wzr, [x19]
.L945:
	ldr	x0, [x19,16]
	cbz	x0, .L946
	ldr	w1, [x0,12]
	cmn	w1, #1
	bne	.L946
	ldr	w1, [x0,8]
	cmn	w1, #1
	bne	.L946
	ldr	w0, [x0]
	cmn	w0, #1
	beq	.L946
	str	w1, [x19]
.L946:
	ldr	w3, [x19]
	cmn	w3, #1
	bne	.L939
	add	x5, x5, :lo12:.LANCHOR0
	ldr	w1, [x19,4]
	mov	x0, x23
	ldrb	w2, [x5,4045]
	bl	printk
	ldr	x1, [x19,8]
	cbz	x1, .L948
	adrp	x0, .LC16
	mov	w2, 4
	add	x0, x0, :lo12:.LC16
	mov	w3, 8
	bl	rknand_print_hex
.L948:
	ldr	x1, [x19,16]
	cbz	x1, .L939
	adrp	x0, .LC17
	mov	w2, 4
	add	x0, x0, :lo12:.LC17
	mov	w3, w2
	bl	rknand_print_hex
.L939:
	add	w22, w22, 1
	add	x19, x19, 56
	b	.L937
.L971:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	FlashReadSlc2KPages, .-FlashReadSlc2KPages
	.align	2
	.global	FlashReadPages
	.type	FlashReadPages, %function
FlashReadPages:
	stp	x29, x30, [sp, -160]!
	adrp	x3, .LANCHOR1+481
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	ldrb	w3, [x3,#:lo12:.LANCHOR1+481]
	add	x19, x19, :lo12:.LANCHOR0
	str	w3, [x29,136]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	ldrb	w3, [x19,32]
	mov	x26, x0
	ldrb	w23, [x19,24]
	stp	x21, x22, [sp,32]
	stp	x27, x28, [sp,80]
	str	w1, [x29,132]
	str	w2, [x29,128]
	str	w3, [x29,124]
	cbz	w23, .L1012
	bl	FlashReadSlc2KPages
	b	.L1061
.L1012:
	add	x0, x19, 1676
	str	x0, [x29,112]
	adrp	x0, .LC15
	mov	w25, w23
	add	x0, x0, :lo12:.LC15
	str	x0, [x29,104]
	adrp	x0, .LC17
	add	x0, x0, :lo12:.LC17
	str	x0, [x29,96]
.L973:
	ldr	w0, [x29,132]
	cmp	w25, w0
	bcs	.L1063
	mov	w27, 56
	ldr	w0, [x29,132]
	ldr	w1, [x29,128]
	add	x2, x29, 156
	umull	x27, w25, w27
	sub	w4, w0, w25
	add	x3, x29, 152
	add	x22, x26, x27
	mov	x0, x22
	ldr	w28, [x22,4]
	bl	LogAddr2PhyAddr
	ldrb	w1, [x19,1672]
	mov	w21, w0
	ldr	w0, [x29,152]
	cmp	w0, w1
	bcc	.L975
	mov	w0, -1
	str	w0, [x26,x27]
	b	.L976
.L975:
	ldr	x1, [x29,112]
	adrp	x24, .LANCHOR2
	ldrb	w20, [x1,w0,uxtw]
	add	x0, x24, :lo12:.LANCHOR2
	ldrb	w0, [x0,728]
	cmp	w0, wzr
	mov	w0, w20
	csel	w21, w21, wzr, ne
	bl	NandcWaitFlashReady
	ldr	x0, [x19,1912]
	ldrb	w1, [x0,19]
	sub	w0, w1, #1
	uxtb	w0, w0
	cmp	w0, 7
	bhi	.L978
	sub	w1, w1, #7
	add	x0, x19, 4056
	sxtw	x2, w20
	add	x0, x0, x2
	uxtb	w1, w1
	cmp	w1, 1
	ldrb	w3, [x0,12]
	bhi	.L979
	ldrb	w3, [x0,20]
.L979:
	add	x0, x24, :lo12:.LANCHOR2
	add	x2, x0, x2
	ldrb	w0, [x2,560]
	cmp	w0, w3
	beq	.L978
	ldrb	w1, [x19,4057]
	mov	w0, w20
	add	x2, x19, 4060
	bl	HynixSetRRPara
.L978:
	mov	w0, w20
	bl	NandcFlashCs
	ldr	w0, [x29,128]
	cmp	w0, 1
	cset	w0, eq
	orr	w0, w0, w28, lsr 31
	str	w0, [x29,140]
	cbz	w0, .L980
	ldrb	w0, [x19,4044]
	cbz	w0, .L980
	mov	w0, w20
	bl	flash_enter_slc_mode
	b	.L981
.L980:
	mov	w0, w20
	bl	flash_exit_slc_mode
.L981:
	cmp	w20, 255
	ldr	w1, [x29,156]
	bne	.L1019
	cmn	w1, #1
	cset	w0, ne
	cbz	w0, .L1013
.L1019:
	cbz	w21, .L985
	ldr	w2, [x19,28]
	mov	w0, w20
	add	w2, w1, w2
	bl	FlashReadDpCmd
	b	.L986
.L985:
	mov	w0, w20
	bl	FlashReadCmd
	b	.L986
.L1013:
	mov	w21, w0
.L983:
	ldrb	w2, [x29,136]
	mov	w0, w20
	ldr	x3, [x22,8]
	mov	w1, 0
	ldr	x4, [x22,16]
	bl	NandcXferData
	cmn	w0, #1
	mov	w28, w0
	bne	.L987
	ldrb	w0, [x19,32]
	cbz	w0, .L987
	strb	wzr, [x19,32]
	mov	w21, 0
	b	.L981
.L987:
	cbz	w21, .L988
	ldr	w2, [x19,28]
	mov	w0, w20
	ldr	w1, [x29,156]
	add	w1, w2, w1
	bl	FlashReadDpDataOutCmd
	add	w0, w25, 1
	mov	w4, 56
	ldrb	w2, [x29,136]
	mov	w1, 0
	umull	x4, w0, w4
	mov	w0, w20
	add	x4, x26, x4
	ldr	x3, [x4,8]
	ldr	x4, [x4,16]
	bl	NandcXferData
	cmn	w0, #1
	mov	w23, w0
	csel	w21, w21, wzr, ne
.L988:
	mov	w0, w20
	bl	NandcFlashDeCs
	ldrb	w0, [x29,124]
	cmn	w28, #1
	strb	w0, [x19,32]
	bne	.L995
	add	x0, x24, :lo12:.LANCHOR2
	ldrb	w0, [x0,572]
	cbnz	w0, .L990
.L994:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	x21, [x0,672]
	cbnz	x21, .L991
	b	.L1064
.L990:
	ldr	x0, [x19,3976]
	mov	w4, 1
	ldr	w1, [x29,156]
	ldr	x2, [x22,8]
	ldr	x3, [x22,16]
	ldr	w21, [x0,304]
	mov	w0, w20
	bl	FlashDdrTunningRead
	cmn	w0, #1
	mov	w28, w0
	beq	.L993
	ldrb	w0, [x19,4045]
	cmp	w28, w0, lsr 1
	bls	.L1015
.L993:
	lsr	w0, w21, 8
	bl	NandcSetDdrPara
	cmn	w28, #1
	beq	.L994
	b	.L1015
.L991:
	ldr	w1, [x29,156]
	mov	w0, w20
	ldr	x2, [x22,8]
	ldr	x3, [x22,16]
	blr	x21
	cmn	w0, #1
	mov	w28, w0
	mov	w21, 0
	bne	.L996
	ldr	x0, [x19,1912]
	ldrb	w0, [x0,19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 7
	bhi	.L997
	ldrb	w1, [x19,4057]
	mov	w0, w20
	add	x2, x19, 4060
	mov	w3, w21
	bl	HynixSetRRPara
.L997:
	ldr	w1, [x29,156]
	mov	w0, w20
	ldr	x2, [x22,8]
	mov	w21, 0
	ldr	x3, [x22,16]
	bl	FlashReadRawPage
	mov	w28, w0
	ldr	w1, [x22,4]
	adrp	x0, .LC18
	ldrb	w2, [x19,4045]
	add	x0, x0, :lo12:.LC18
	mov	w3, w28
	bl	printk
	cmn	w28, #1
	bne	.L996
	ldrb	w0, [x19,4044]
	cbz	w0, .L996
	ldr	w0, [x29,140]
	cbz	w0, .L998
	mov	w0, w20
	bl	flash_enter_slc_mode
	b	.L999
.L998:
	mov	w0, w20
	bl	flash_exit_slc_mode
.L999:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	w1, [x29,156]
	ldr	x2, [x22,8]
	mov	w21, 0
	ldr	x3, [x22,16]
	ldr	x4, [x0,672]
	mov	w0, w20
	blr	x4
	mov	w28, w0
	b	.L996
.L1064:
	ldr	w1, [x29,156]
	mov	w0, w20
	ldr	x2, [x22,8]
	ldr	x3, [x22,16]
	bl	FlashReadRawPage
	mov	w28, w0
	b	.L996
.L1015:
	mov	w21, 0
.L995:
	ldrb	w0, [x19,4045]
	add	w0, w0, w0, lsl 1
	cmp	w28, w0, lsr 2
	bls	.L996
	add	x0, x24, :lo12:.LANCHOR2
	ldr	x0, [x0,672]
	cmp	x0, xzr
	mov	w0, 256
	csel	w28, w28, w0, ne
.L996:
	cmp	w28, 256
	beq	.L1020
	cmn	w28, #1
	bne	.L1000
.L1020:
	str	w28, [x26,x27]
	b	.L1002
.L1000:
	str	wzr, [x26,x27]
.L1002:
	ldr	w3, [x26,x27]
	cmn	w3, #1
	bne	.L1004
	ldr	w1, [x22,4]
	ldr	x0, [x29,104]
	ldrb	w2, [x19,4045]
	bl	printk
	ldr	x1, [x22,16]
	cbz	x1, .L1004
	mov	w2, 4
	ldr	x0, [x29,96]
	mov	w3, w2
	bl	rknand_print_hex
.L1004:
	cbz	w21, .L1006
	ldrb	w0, [x19,4045]
	add	w0, w0, w0, lsl 1
	cmp	w23, w0, lsr 2
	bls	.L1007
	add	x24, x24, :lo12:.LANCHOR2
	ldr	x0, [x24,672]
	cmp	x0, xzr
	mov	w0, 256
	csel	w23, w23, w0, ne
.L1007:
	add	w0, w25, 1
	mov	w1, 56
	cmp	w23, 256
	umull	x0, w0, w1
	beq	.L1021
	cmn	w23, #1
	bne	.L1008
.L1021:
	str	w23, [x26,x0]
	b	.L1006
.L1008:
	str	wzr, [x26,x0]
.L1006:
	ldr	w0, [x29,140]
	add	w25, w25, w21
	cbz	w0, .L976
	ldrb	w0, [x19,4044]
	cbz	w0, .L976
	mov	w0, w20
	bl	flash_exit_slc_mode
.L976:
	add	w25, w25, 1
	b	.L973
.L1063:
	mov	w0, 0
	b	.L1061
.L986:
	mov	w0, w20
	bl	NandcWaitFlashReady
	cbz	w21, .L983
	ldr	w1, [x29,156]
	mov	w0, w20
	bl	FlashReadDpDataOutCmd
	b	.L983
.L1061:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 160
	ret
	.size	FlashReadPages, .-FlashReadPages
	.align	2
	.global	FlashLoadFactorBbt
	.type	FlashLoadFactorBbt, %function
FlashLoadFactorBbt:
	stp	x29, x30, [sp, -192]!
	mov	w2, 16
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR0
	add	x0, x22, :lo12:.LANCHOR0
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	adrp	x23, .LANCHOR2
	mov	w27, -1
	ldrh	w1, [x0,4028]
	add	x19, x23, :lo12:.LANCHOR2
	ldrh	w21, [x0,4030]
	add	x0, x19, 736
	mov	w28, 0
	adrp	x26, .LC19
	add	x26, x26, :lo12:.LC19
	mul	w21, w1, w21
	mov	w1, 0
	uxth	w21, w21
	bl	ftl_memset
	add	w25, w21, w27
	ldr	x0, [x19,704]
	str	x0, [x29,152]
	mov	w19, w28
	sub	w0, w21, #12
	str	xzr, [x29,144]
	uxth	w25, w25
	str	w0, [x29,124]
.L1066:
	add	x1, x22, :lo12:.LANCHOR0
	ldrb	w1, [x1,1672]
	cmp	w1, w19
	bls	.L1076
	mul	w3, w19, w21
	mov	w20, w25
	mov	w4, 61664
.L1067:
	ldr	w0, [x29,124]
	cmp	w20, w0
	ble	.L1069
	add	w1, w3, w20
	add	x0, x29, 136
	lsl	w1, w1, 10
	str	w1, [x29,140]
	mov	w1, 1
	str	x4, [x29,104]
	mov	w2, w1
	str	x3, [x29,112]
	bl	FlashReadPages
	ldr	w0, [x29,136]
	ldr	x3, [x29,112]
	cmn	w0, #1
	ldr	x4, [x29,104]
	beq	.L1068
	add	x24, x23, :lo12:.LANCHOR2
	ldr	x0, [x24,704]
	ldrh	w0, [x0]
	cmp	w0, w4
	bne	.L1068
	add	x24, x24, 736
	mov	x0, x26
	mov	w1, w19
	mov	w2, w20
	add	w28, w28, 1
	bl	printk
	strh	w20, [x24,w19,sxtw 1]
	uxth	w28, w28
	b	.L1069
.L1068:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L1067
.L1069:
	add	x1, x22, :lo12:.LANCHOR0
	add	w19, w19, 1
	uxtb	w19, w19
	ldrb	w1, [x1,1672]
	cmp	w1, w28
	csel	w27, w27, wzr, ne
	b	.L1066
.L1076:
	mov	w0, w27
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 192
	ret
	.size	FlashLoadFactorBbt, .-FlashLoadFactorBbt
	.align	2
	.global	FlashReadFacBbtData
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	mov	x22, x0
	adrp	x0, .LANCHOR0
	add	x4, x0, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	mov	w23, w1
	ldr	x0, [x0,#:lo12:.LANCHOR0]
	ldrh	w3, [x4,4028]
	mov	w25, w2
	ldrh	w1, [x4,4030]
	adrp	x4, .LANCHOR2
	str	x0, [x29,96]
	add	x0, x4, :lo12:.LANCHOR2
	mov	x21, x4
	mov	w26, 61664
	mul	w3, w3, w1
	ldr	x0, [x0,704]
	str	x0, [x29,104]
	uxth	w3, w3
	sub	w20, w3, #1
	sub	w19, w3, #16
	mul	w24, w23, w3
	uxth	w20, w20
.L1078:
	cmp	w20, w19
	ble	.L1086
	add	w0, w20, w24
	mov	w1, 1
	lsl	w0, w0, 10
	mov	w2, w1
	str	w0, [x29,92]
	add	x0, x29, 88
	bl	FlashReadPages
	ldr	w0, [x29,88]
	cmn	w0, #1
	beq	.L1079
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x0, [x0,704]
	ldrh	w0, [x0]
	cmp	w0, w26
	bne	.L1079
	mov	w0, w22
	cbz	x22, .L1080
	mov	x0, x22
	mov	w1, w23
	mov	w2, w25
	bl	FlashReadFacBbtData.part.6
	b	.L1080
.L1079:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L1078
.L1086:
	mov	w0, -1
.L1080:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 144
	ret
	.size	FlashReadFacBbtData, .-FlashReadFacBbtData
	.align	2
	.global	FlashGetBadBlockList
	.type	FlashGetBadBlockList, %function
FlashGetBadBlockList:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x21, [sp,32]
	mov	x21, x0
	adrp	x0, .LANCHOR0+1912
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	ldr	x0, [x0,#:lo12:.LANCHOR0+1912]
	ldrb	w2, [x0,13]
	ldrh	w19, [x0,14]
	add	x0, x20, :lo12:.LANCHOR2
	mul	w19, w2, w19
	ldr	x0, [x0,696]
	uxth	w19, w19
	add	w2, w19, 7
	lsr	w2, w2, 3
	bl	FlashReadFacBbtData
	cmn	w0, #1
	bne	.L1088
.L1092:
	mov	w1, 0
	b	.L1089
.L1088:
	mov	w2, 0
	lsr	w0, w19, 4
	mov	w1, w2
	sub	w19, w19, #1
	mov	w5, 1
.L1090:
	cmp	w2, w19
	bge	.L1089
	add	x3, x20, :lo12:.LANCHOR2
	ubfx	x4, x2, 5, 11
	ldr	x6, [x3,696]
	lsl	w3, w5, w2
	ldr	w4, [x6,x4,lsl 2]
	tst	w3, w4
	beq	.L1091
	add	w3, w1, 1
	ubfiz	x1, x1, 1, 16
	strh	w2, [x21,x1]
	uxth	w1, w3
.L1091:
	cmp	w1, w0
	bcs	.L1092
	add	w2, w2, 1
	uxth	w2, w2
	b	.L1090
.L1089:
	ubfiz	x1, x1, 1, 16
	mov	w0, -1
	ldp	x19, x20, [sp,16]
	strh	w0, [x21,x1]
	mov	w0, 0
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashGetBadBlockList, .-FlashGetBadBlockList
	.align	2
	.global	FlashProgSlc2KPages
	.type	FlashProgSlc2KPages, %function
FlashProgSlc2KPages:
	stp	x29, x30, [sp, -160]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	mov	w23, w1
	adrp	x1, .LANCHOR1+481
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR0
	mov	w24, w2
	stp	x25, x26, [sp,64]
	add	x22, x22, :lo12:.LANCHOR0
	ldrb	w26, [x1,#:lo12:.LANCHOR1+481]
	stp	x27, x28, [sp,80]
	stp	x19, x20, [sp,16]
	mov	w27, w3
	mov	x19, x0
	mov	x21, x0
	mov	w25, 0
	add	x28, x22, 1676
.L1098:
	cmp	w25, w23
	beq	.L1127
	mov	w1, w24
	sub	w4, w23, w25
	mov	x0, x21
	add	x2, x29, 96
	add	x3, x29, 100
	bl	LogAddr2PhyAddr
	ldr	w0, [x29,100]
	ldrb	w1, [x22,1672]
	cmp	w0, w1
	bcc	.L1099
	mov	w0, -1
	str	w0, [x21]
	b	.L1100
.L1099:
	ldrb	w20, [x28,w0,uxtw]
	mov	w0, w20
	bl	NandcWaitFlashReady
	mov	w0, w20
	bl	NandcFlashCs
	ldr	w1, [x29,96]
	mov	w0, w20
	bl	FlashProgFirstCmd
	ldr	x3, [x21,8]
	mov	w2, w26
	ldr	x4, [x21,16]
	mov	w1, 1
	mov	w0, w20
	bl	NandcXferData
	ldr	w1, [x29,96]
	mov	w0, w20
	bl	FlashProgSecondCmd
	mov	w0, w20
	bl	NandcWaitFlashReady
	ldr	w1, [x29,96]
	mov	w0, w20
	bl	FlashReadStatus
	sbfx	x0, x0, 0, 1
	str	w0, [x21]
	mov	w0, w20
	ldr	w1, [x29,96]
	ldr	w2, [x22,28]
	add	w1, w2, w1
	bl	FlashProgFirstCmd
	ldr	x0, [x21,8]
	mov	w1, 1
	mov	w2, w26
	cmp	x0, xzr
	add	x3, x0, 2048
	ldr	x0, [x21,16]
	csel	x3, x3, xzr, ne
	add	x4, x0, 8
	cmp	x0, xzr
	csel	x4, x4, xzr, ne
	mov	w0, w20
	bl	NandcXferData
	adrp	x0, .LANCHOR0
	ldr	w1, [x29,96]
	add	x0, x0, :lo12:.LANCHOR0
	ldr	w2, [x0,28]
	mov	w0, w20
	add	w1, w2, w1
	bl	FlashProgSecondCmd
	mov	w0, w20
	bl	NandcWaitFlashReady
	ldr	w1, [x29,96]
	mov	w0, w20
	bl	FlashReadStatus
	tbz	x0, 0, .L1103
	mov	w0, -1
	str	w0, [x21]
.L1103:
	mov	w0, w20
	bl	NandcFlashDeCs
.L1100:
	add	w25, w25, 1
	add	x21, x21, 56
	b	.L1098
.L1127:
	mov	w21, 0
	cbz	w27, .L1125
	adrp	x25, .LC23
	adrp	x26, .LC22
	adrp	x27, .LC21
	add	x25, x25, :lo12:.LC23
	add	x26, x26, :lo12:.LC22
	add	x27, x27, :lo12:.LC21
.L1126:
	cmp	w21, w23
	beq	.L1125
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1106
	adrp	x0, .LC20
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC20
	bl	printk
	b	.L1107
.L1106:
	adrp	x20, .LANCHOR2
	sub	w4, w23, w21
	add	x22, x20, :lo12:.LANCHOR2
	add	x3, x29, 100
	mov	w1, w24
	add	x2, x29, 96
	mov	x0, x19
	bl	LogAddr2PhyAddr
	ldr	x0, [x22,712]
	mov	x1, x19
	mov	x2, 56
	str	wzr, [x0]
	ldr	x0, [x22,720]
	str	wzr, [x0]
	add	x0, x29, 104
	bl	memcpy
	ldr	x0, [x22,712]
	mov	w1, 1
	str	x0, [x29,112]
	mov	w2, w24
	ldr	x0, [x22,720]
	str	x0, [x29,120]
	add	x0, x29, 104
	bl	FlashReadPages
	ldr	w22, [x29,104]
	cmn	w22, #1
	bne	.L1108
	ldr	w1, [x19,4]
	mov	x0, x27
	bl	printk
	str	w22, [x19]
.L1108:
	ldr	x0, [x19,16]
	cbz	x0, .L1109
	ldr	w2, [x0]
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x0, [x0,720]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1109
	ldr	w1, [x19,4]
	mov	x0, x26
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1109:
	ldr	x0, [x19,8]
	cbz	x0, .L1107
	add	x20, x20, :lo12:.LANCHOR2
	ldr	w2, [x0]
	ldr	x0, [x20,712]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1107
	ldr	w1, [x19,4]
	mov	x0, x25
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1107:
	add	w21, w21, 1
	add	x19, x19, 56
	b	.L1126
.L1125:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 160
	ret
	.size	FlashProgSlc2KPages, .-FlashProgSlc2KPages
	.align	2
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	stp	x29, x30, [sp, -192]!
	adrp	x6, .LANCHOR1+481
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR0
	mov	w22, 0
	add	x4, x21, :lo12:.LANCHOR0
	str	w3, [x29,124]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	mov	x19, x0
	mov	w25, w1
	ldr	x5, [x4,1912]
	mov	w23, w2
	ldrb	w4, [x4,24]
	ldrb	w27, [x6,#:lo12:.LANCHOR1+481]
	ldrb	w5, [x5,19]
	cbz	w4, .L1129
	bl	FlashProgSlc2KPages
	b	.L1130
.L1129:
	sub	w0, w5, #1
	adrp	x28, .LANCHOR2
	str	w0, [x29,120]
	add	x0, x28, :lo12:.LANCHOR2
	str	x0, [x29,112]
.L1181:
	cmp	w22, w25
	bcs	.L1182
	mov	w20, 56
	sub	w4, w25, w22
	add	x2, x29, 128
	mov	w1, w23
	umull	x20, w22, w20
	add	x3, x29, 132
	add	x26, x19, x20
	mov	x0, x26
	bl	LogAddr2PhyAddr
	mov	w24, w0
	add	x2, x21, :lo12:.LANCHOR0
	ldr	w0, [x29,132]
	ldrb	w4, [x2,1672]
	cmp	w0, w4
	bcc	.L1131
	mov	w0, -1
	str	w0, [x19,x20]
	b	.L1132
.L1131:
	add	x1, x28, :lo12:.LANCHOR2
	add	x3, x2, 1716
	mov	x5, 24
	ldrb	w1, [x1,656]
	cmp	w1, wzr
	uxtw	x1, w0
	csel	w24, w24, wzr, ne
	madd	x0, x1, x5, x3
	ldr	x0, [x0,8]
	cbz	x0, .L1134
	cmp	w4, 1
	bne	.L1135
	ldr	x0, [x2,3976]
	bl	NandcIqrWaitFlashReady
.L1135:
	ldrb	w0, [x29,132]
	bl	FlashWaitCmdDone
.L1134:
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w2, [x29,132]
	mov	x1, 24
	add	x0, x0, 1716
	madd	x0, x2, x1, x0
	ldr	w1, [x29,128]
	str	x26, [x0,8]
	str	xzr, [x0,16]
	str	w1, [x0,4]
	cbz	w24, .L1136
	add	w1, w22, 1
	mov	w3, 56
	umull	x1, w1, w3
	add	x1, x19, x1
	str	x1, [x0,16]
.L1136:
	add	x1, x21, :lo12:.LANCHOR0
	add	x0, x1, 1676
	ldrb	w20, [x0,x2]
	mov	x0, 24
	madd	x2, x2, x0, x1
	ldrb	w0, [x1,1672]
	strb	w20, [x2,1716]
	cmp	w0, 1
	bne	.L1137
	mov	w0, w20
	bl	NandcWaitFlashReady
	b	.L1138
.L1137:
	mov	w0, w20
	str	x1, [x29,104]
	bl	NandcFlashCs
	ldr	x1, [x29,104]
	ldr	w0, [x29,132]
	add	x1, x1, 1684
	ldr	w0, [x1,x0,lsl 2]
	ldr	w1, [x29,128]
	cmp	w0, wzr
	mov	w0, w20
	cset	w2, ne
	bl	FlashWaitReadyEN
	mov	w0, w20
	bl	NandcFlashDeCs
.L1138:
	ldr	w0, [x29,120]
	cmp	w0, 7
	bhi	.L1139
	ldr	x0, [x29,112]
	add	x0, x0, x20, sxtw
	ldrb	w0, [x0,560]
	cbz	w0, .L1139
	add	x2, x21, :lo12:.LANCHOR0
	mov	w0, w20
	add	x2, x2, 4060
	mov	w3, 0
	ldrb	w1, [x2,-3]
	bl	HynixSetRRPara
.L1139:
	mov	w0, w20
	bl	NandcFlashCs
	cmp	w23, 1
	bne	.L1140
	add	x0, x21, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L1140
	mov	w0, w20
	bl	flash_enter_slc_mode
	b	.L1141
.L1140:
	mov	w0, w20
	bl	flash_exit_slc_mode
.L1141:
	ldr	w1, [x29,128]
	mov	w0, w20
	bl	FlashProgFirstCmd
	ldr	x3, [x26,8]
	mov	w0, w20
	ldr	x4, [x26,16]
	mov	w1, 1
	mov	w2, w27
	bl	NandcXferData
	cbz	w24, .L1142
	ldr	w1, [x29,128]
	mov	w0, w20
	add	x26, x21, :lo12:.LANCHOR0
	bl	FlashProgDpFirstCmd
	ldr	w1, [x29,132]
	add	x0, x26, 1684
	ldr	w0, [x0,x1,lsl 2]
	ldr	w1, [x29,128]
	cmp	w0, wzr
	mov	w0, w20
	cset	w2, ne
	bl	FlashWaitReadyEN
	ldr	w2, [x26,28]
	mov	w0, w20
	ldr	w1, [x29,128]
	add	w1, w2, w1
	bl	FlashProgDpSecondCmd
	add	w0, w22, 1
	mov	w4, 56
	mov	w1, 1
	mov	w2, w27
	umull	x4, w0, w4
	mov	w0, w20
	add	x4, x19, x4
	ldr	x3, [x4,8]
	ldr	x4, [x4,16]
	bl	NandcXferData
.L1142:
	ldr	w1, [x29,128]
	mov	w0, w20
	add	w22, w22, w24
	bl	FlashProgSecondCmd
	mov	w0, w20
	bl	NandcFlashDeCs
.L1132:
	add	w22, w22, 1
	b	.L1181
.L1182:
	add	x21, x21, :lo12:.LANCHOR0
	mov	x20, 0
	add	x22, x21, 1716
	mov	x24, 24
	ldr	x0, [x21,3976]
	bl	NandcIqrWaitFlashReady
.L1144:
	ldrb	w0, [x21,1672]
	cmp	w0, w20
	bls	.L1183
	mov	w0, w20
	bl	FlashWaitCmdDone
	cmp	w23, 1
	bne	.L1145
	ldrb	w0, [x21,4044]
	cbz	w0, .L1145
	mul	x0, x20, x24
	ldrb	w0, [x0,x22]
	bl	flash_exit_slc_mode
.L1145:
	add	x20, x20, 1
	b	.L1144
.L1183:
	ldr	w0, [x29,124]
	cbnz	w0, .L1147
.L1155:
	mov	w0, 0
	b	.L1130
.L1147:
	adrp	x24, .LC23
	adrp	x26, .LC22
	adrp	x27, .LC21
	mov	w21, 0
	add	x24, x24, :lo12:.LC23
	add	x26, x26, :lo12:.LC22
	add	x27, x27, :lo12:.LC21
.L1148:
	cmp	w21, w25
	beq	.L1155
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1149
	adrp	x0, .LC20
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC20
	bl	printk
	b	.L1150
.L1149:
	adrp	x20, .LANCHOR2
	sub	w4, w25, w21
	add	x22, x20, :lo12:.LANCHOR2
	add	x3, x29, 132
	mov	w1, w23
	add	x2, x29, 128
	mov	x0, x19
	bl	LogAddr2PhyAddr
	ldr	x0, [x22,712]
	mov	x1, x19
	mov	x2, 56
	str	wzr, [x0]
	ldr	x0, [x22,720]
	str	wzr, [x0]
	add	x0, x29, 136
	bl	memcpy
	ldr	x0, [x22,712]
	mov	w1, 1
	str	x0, [x29,144]
	mov	w2, w23
	ldr	x0, [x22,720]
	str	x0, [x29,152]
	add	x0, x29, 136
	bl	FlashReadPages
	ldr	w22, [x29,136]
	cmn	w22, #1
	bne	.L1151
	ldr	w1, [x19,4]
	mov	x0, x27
	bl	printk
	str	w22, [x19]
.L1151:
	ldr	x0, [x19,16]
	cbz	x0, .L1152
	ldr	w2, [x0]
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x0, [x0,720]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1152
	ldr	w1, [x19,4]
	mov	x0, x26
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1152:
	ldr	x0, [x19,8]
	cbz	x0, .L1150
	add	x20, x20, :lo12:.LANCHOR2
	ldr	w2, [x0]
	ldr	x0, [x20,712]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1150
	ldr	w1, [x19,4]
	mov	x0, x24
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1150:
	add	w21, w21, 1
	add	x19, x19, 56
	b	.L1148
.L1130:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 192
	ret
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.type	FlashTestBlk.part.7, %function
FlashTestBlk.part.7:
	stp	x29, x30, [sp, -160]!
	mov	w1, 165
	mov	w2, 32
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	uxth	w20, w0
	add	x19, x19, :lo12:.LANCHOR2
	lsl	w20, w20, 10
	ldr	x0, [x19,696]
	str	x0, [x29,48]
	add	x0, x29, 96
	str	x0, [x29,56]
	bl	ftl_memset
	ldr	x0, [x19,696]
	mov	w1, 90
	mov	w2, 8
	bl	ftl_memset
	str	w20, [x29,44]
	mov	w1, 1
	add	x0, x29, 40
	mov	w2, w1
	bl	FlashEraseBlocks
	mov	w1, 1
	add	x0, x29, 40
	mov	w3, w1
	mov	w2, w1
	bl	FlashProgPages
	ldr	w0, [x29,40]
	mov	w1, 0
	mov	w2, 1
	cmp	w0, wzr
	add	x0, x29, 40
	csetm	w19, ne
	bl	FlashEraseBlocks
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 160
	ret
	.size	FlashTestBlk.part.7, .-FlashTestBlk.part.7
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	uxth	w1, w0
	adrp	x0, .LANCHOR0+8
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w2, [x0,#:lo12:.LANCHOR0+8]
	mov	w0, 0
	cmp	w1, w2
	bcc	.L1186
	mov	w0, w1
	bl	FlashTestBlk.part.7
.L1186:
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FlashMakeFactorBbt
	.type	FlashMakeFactorBbt, %function
FlashMakeFactorBbt:
	stp	x29, x30, [sp, -240]!
	adrp	x2, .LANCHOR2
	adrp	x4, .LANCHOR0
	add	x29, sp, 0
	add	x0, x2, :lo12:.LANCHOR2
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	mov	x20, x4
	str	x2, [x29,136]
	ldr	x0, [x0,704]
	add	x24, x20, :lo12:.LANCHOR0
	str	x0, [x29,160]
	add	x0, x4, :lo12:.LANCHOR0
	mov	w19, 0
	ldrh	w1, [x0,4028]
	ldrh	w21, [x0,4030]
	mul	w21, w1, w21
	ldr	x1, [x0,1912]
	uxth	w21, w21
	ldrb	w1, [x1,24]
	str	w1, [x29,144]
	ldrh	w1, [x0,28]
	str	w1, [x29,172]
	ldrb	w0, [x0,24]
	ubfiz	w1, w1, 1, 15
	cmp	w0, 1
	ldr	w0, [x29,172]
	csel	w0, w1, w0, eq
	str	w0, [x29,172]
	adrp	x0, .LC24
	mov	w1, 1
	add	x0, x0, :lo12:.LC24
	bl	printk
	ldr	x0, [x29,136]
	mov	w1, 0
	mov	w2, 4096
	add	x0, x0, :lo12:.LANCHOR2
	ldr	x0, [x0,704]
	bl	ftl_memset
	lsr	w0, w21, 4
	str	w0, [x29,168]
	add	x0, x24, 1676
	str	x0, [x29,120]
	ldr	w0, [x29,144]
	and	w0, w0, 1
	str	w0, [x29,116]
	sub	w0, w21, #1
	uxth	w0, w0
	str	w0, [x29,132]
.L1190:
	ldrb	w0, [x24,1672]
	cmp	w0, w19
	bls	.L1241
	ldr	x0, [x29,136]
	sxtw	x23, w19
	add	x0, x0, :lo12:.LANCHOR2
	add	x0, x0, 736
	ldrh	w20, [x0,w19,sxtw 1]
	cbnz	w20, .L1220
	ldrh	w2, [x24,4036]
	mov	w1, w20
	ldr	x0, [x24]
	mov	w25, w20
	lsl	w2, w2, 9
	mov	w22, w20
	add	x28, x24, 1684
	bl	ftl_memset
	ldr	x0, [x29,120]
	ldrb	w26, [x0,x23]
	ldr	w0, [x29,144]
	and	w0, w0, 2
	uxtb	w0, w0
	str	w0, [x29,128]
.L1192:
	uxth	w27, w25
	cmp	w27, w21
	bcs	.L1202
	mov	w0, -1
	strb	w0, [x29,182]
	strb	w0, [x29,183]
	ldr	w0, [x29,116]
	cbz	w0, .L1194
	ldr	w3, [x28,x23,lsl 2]
	mov	w0, w26
	add	x2, x29, 182
	add	w3, w20, w3
	str	x3, [x29,104]
	mov	w1, w3
	bl	FlashReadSpare
	ldrb	w0, [x24,24]
	ldr	x3, [x29,104]
	cmp	w0, 1
	bne	.L1194
	ldr	w1, [x24,28]
	mov	w0, w26
	add	x2, x29, 183
	add	w1, w3, w1
	bl	FlashReadSpare
	ldrb	w0, [x29,182]
	ldrb	w1, [x29,183]
	and	w0, w1, w0
	strb	w0, [x29,182]
.L1194:
	ldr	w0, [x29,128]
	cbz	w0, .L1196
	ldr	x0, [x24,1912]
	add	x2, x29, 183
	ldrh	w1, [x0,10]
	ldr	w0, [x28,x23,lsl 2]
	sub	w1, w1, #1
	add	w1, w1, w0
	mov	w0, w26
	add	w1, w1, w20
	bl	FlashReadSpare
.L1196:
	ldr	x0, [x24,1912]
	ldrb	w1, [x29,182]
	ldrb	w0, [x0,7]
	cmp	w0, 8
	beq	.L1221
	cmp	w0, 1
	bne	.L1197
.L1221:
	mov	w0, 1
	cbz	w1, .L1199
	ldrb	w0, [x29,183]
	cmp	w0, wzr
	cset	w0, eq
	b	.L1199
.L1197:
	cmp	w1, 255
	mov	w0, 1
	bne	.L1199
	ldrb	w0, [x29,183]
	cmp	w0, 255
	cset	w0, ne
.L1199:
	ldr	x1, [x29,144]
	tbz	x1, 2, .L1200
	ldr	w1, [x28,x23,lsl 2]
	mov	w0, w26
	add	w1, w20, w1
	bl	SandiskProgTestBadBlock
.L1200:
	cbz	w0, .L1201
	adrp	x0, .LC25
	mov	w1, w19
	mov	w2, w25
	add	x0, x0, :lo12:.LC25
	add	w22, w22, 1
	bl	printk
	ubfx	x1, x27, 5, 11
	ldr	x2, [x24]
	lsl	x1, x1, 2
	mov	w0, 1
	lsl	w27, w0, w27
	uxth	w22, w22
	ldr	w0, [x2,x1]
	orr	w27, w0, w27
	str	w27, [x2,x1]
	ldr	w1, [x29,168]
	ldrb	w0, [x24,1672]
	mul	w0, w1, w0
	cmp	w22, w0
	bgt	.L1202
.L1201:
	ldr	w0, [x29,172]
	add	w25, w25, 1
	add	w20, w20, w0
	b	.L1192
.L1202:
	adrp	x0, .LC26
	mov	w1, w19
	add	x0, x0, :lo12:.LC26
	mov	w2, w22
	bl	printk
	ldrb	w0, [x24,1672]
	ldr	w1, [x29,168]
	mul	w0, w1, w0
	cmp	w22, w0
	blt	.L1204
	ldrh	w2, [x24,4036]
	mov	w1, 0
	ldr	x0, [x24]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1204:
	cbnz	w19, .L1206
	adrp	x25, .LC27
	ldrh	w26, [x24,8]
	mov	w20, w19
	add	x25, x25, :lo12:.LC27
	mov	w22, 1
.L1207:
	ldrb	w0, [x24,25]
	cmp	w0, w26
	bls	.L1242
	mov	w0, w26
	bl	FlashTestBlk
	cbz	w0, .L1208
	mov	w1, w26
	mov	x0, x25
	add	w20, w20, 1
	bl	printk
	ubfx	x1, x26, 5, 11
	ldr	x2, [x24]
	lsl	x1, x1, 2
	lsl	w0, w22, w26
	uxth	w20, w20
	ldr	w3, [x2,x1]
	orr	w0, w3, w0
	str	w0, [x2,x1]
.L1208:
	add	w26, w26, 1
	uxth	w26, w26
	b	.L1207
.L1242:
	adrp	x25, .LC27
	ldr	w27, [x29,132]
	sub	w26, w21, #50
	add	x25, x25, :lo12:.LC27
	mov	w22, 1
.L1210:
	cmp	w27, w26
	ble	.L1243
	mov	w0, w27
	bl	FlashTestBlk
	cbz	w0, .L1211
	mov	w1, w27
	mov	x0, x25
	bl	printk
	ubfx	x1, x27, 5, 11
	ldr	x2, [x24]
	lsl	x1, x1, 2
	lsl	w0, w22, w27
	ldr	w3, [x2,x1]
	orr	w0, w3, w0
	str	w0, [x2,x1]
.L1211:
	sub	w27, w27, #1
	uxth	w27, w27
	b	.L1210
.L1243:
	ldrb	w0, [x24,25]
	ldr	w1, [x24,8]
	sub	w0, w0, w1
	cmp	w20, w0
	bcc	.L1206
	ldrh	w2, [x24,4036]
	mov	w1, 0
	ldr	x0, [x24]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1206:
	ldr	x0, [x29,136]
	adrp	x22, .LC28
	ldr	w26, [x29,132]
	add	x22, x22, :lo12:.LC28
	mul	w25, w19, w21
	add	x27, x0, :lo12:.LANCHOR2
	mov	w20, -3872
.L1214:
	mov	w2, w26
	mov	x0, x22
	mov	w1, w19
	bl	printk
	ldr	x2, [x24]
.L1215:
	ubfx	x0, x26, 5, 11
	ldr	w0, [x2,x0,lsl 2]
	lsr	w0, w0, w26
	and	w1, w0, 1
	tbz	x0, 0, .L1244
	sub	w26, w26, #1
	uxth	w26, w26
	b	.L1215
.L1244:
	add	x0, x27, 736
	strh	w26, [x0,x23,lsl 1]
	ldr	x0, [x29,160]
	strh	w20, [x0]
	strh	w26, [x0,2]
	strh	w1, [x0,8]
	mov	w1, 1
	ldr	x0, [x24]
	mov	w2, w1
	str	x0, [x29,192]
	ldr	x0, [x27,704]
	str	x0, [x29,200]
	add	w0, w26, w25
	lsl	w0, w0, 10
	str	w0, [x29,188]
	add	x0, x29, 184
	bl	FlashEraseBlocks
	mov	w1, 1
	add	x0, x29, 184
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
	ldr	w0, [x29,184]
	cbz	w0, .L1220
	sub	w26, w26, #1
	uxth	w26, w26
	b	.L1214
.L1220:
	add	w19, w19, 1
	uxtb	w19, w19
	b	.L1190
.L1241:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 240
	ret
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
	.align	2
	.global	Ftl_log2
	.type	Ftl_log2, %function
Ftl_log2:
	mov	w2, 1
	mov	w1, 0
.L1246:
	cmp	w2, w0
	bhi	.L1248
	add	w1, w1, 1
	lsl	w2, w2, 1
	uxth	w1, w1
	b	.L1246
.L1248:
	sub	w0, w1, #1
	ret
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
	uxth	w0, w0
	mov	w1, 24
	cmp	w0, 23
	csel	w0, w0, w1, hi
	adrp	x1, .LANCHOR2
	add	x1, x1, :lo12:.LANCHOR2
	ldrh	w2, [x1,756]
	ldrh	w3, [x1,766]
	str	w0, [x1,752]
	mul	w2, w0, w2
	sub	w0, w3, w0
	strh	w0, [x1,764]
	ldr	w0, [x1,772]
	str	w2, [x1,760]
	sub	w2, w0, w2
	mov	w0, 0
	str	w2, [x1,768]
	ret
	.size	FtlSysBlkNumInit, .-FtlSysBlkNumInit
	.align	2
	.global	FtlConstantsInit
	.type	FtlConstantsInit, %function
FtlConstantsInit:
	stp	x29, x30, [sp, -96]!
	mov	x3, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	adrp	x19, .LANCHOR2
	mov	x24, x0
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w4, [x24,14]
	ldrh	w6, [x0,8]
	ldrh	w2, [x0,10]
	ldrh	w0, [x0,12]
	strh	w6, [x1,776]
	strh	w2, [x1,778]
	strh	w0, [x1,780]
	strh	w4, [x1,766]
.L1253:
	add	x5, x1, 784
	strb	w3, [x3,x5]
	add	x3, x3, 1
	cmp	x3, 32
	bne	.L1253
	ldrh	w3, [x24,20]
	ldrb	w1, [x24,15]
	cmp	w3, w1
	bcs	.L1254
	uxtb	w8, w0
	mov	w3, 0
	mul	w11, w0, w2
	ubfiz	w10, w8, 1, 7
	add	x12, x19, :lo12:.LANCHOR2
.L1255:
	cmp	w3, w0
	bcs	.L1257
	uxtb	w1, w3
	mov	w5, w3
	mov	w7, 0
.L1258:
	cmp	w7, w2
	bcs	.L1280
	add	x9, x12, 784
	add	w13, w5, w11
	add	w14, w1, w8
	add	w7, w7, 1
	strb	w1, [x9,w5,uxtw]
	add	w1, w1, w10
	strb	w14, [x9,x13]
	add	w5, w5, w0
	uxtb	w1, w1
	b	.L1258
.L1280:
	add	w3, w3, 1
	b	.L1255
.L1257:
	add	x1, x19, :lo12:.LANCHOR2
	lsl	w2, w2, 1
	lsr	w4, w4, 1
	strh	w2, [x1,778]
	strh	w4, [x1,766]
.L1254:
	add	x1, x19, :lo12:.LANCHOR2
	mov	w2, 5
	cmp	w6, 1
	strh	w2, [x1,816]
	strh	wzr, [x1,818]
	bne	.L1259
	strh	w6, [x1,816]
.L1259:
	add	x1, x19, :lo12:.LANCHOR2
	mov	w2, 4352
	adrp	x21, .LANCHOR0
	strh	w2, [x1,820]
	add	x2, x21, :lo12:.LANCHOR0
	ldrb	w27, [x2,24]
	cbz	w27, .L1260
	mov	w2, 384
	strh	w2, [x1,820]
.L1260:
	add	x22, x19, :lo12:.LANCHOR2
	ldrh	w26, [x24,16]
	ldrh	w25, [x24,20]
	ldrh	w28, [x24,18]
	ldrh	w23, [x22,778]
	ldrh	w20, [x22,766]
	strh	w26, [x22,824]
	mul	w23, w0, w23
	mul	w0, w20, w0
	strh	w0, [x22,822]
	uxth	w23, w23
	strh	w28, [x22,826]
	strh	w23, [x22,756]
	mul	w0, w23, w26
	strh	w0, [x22,828]
	mov	w0, w25
	strh	w25, [x22,830]
	bl	Ftl_log2
	uxth	w3, w0
	lsl	w2, w25, 9
	strh	w0, [x22,832]
	ldrh	w0, [x24,26]
	cmp	w20, 1024
	strh	w2, [x22,834]
	ubfx	x2, x2, 9, 7
	strh	w0, [x22,838]
	lsl	w2, w2, 1
	mul	w0, w23, w20
	strh	w2, [x22,836]
	str	w0, [x22,772]
	bls	.L1261
	and	w0, w20, 255
	strh	w0, [x22,818]
.L1261:
	add	x2, x19, :lo12:.LANCHOR2
	mul	w0, w25, w28
	ldrh	w1, [x2,818]
	sub	w1, w20, w1
	mul	w1, w23, w1
	mul	w1, w1, w25
	mul	w1, w1, w26
	asr	w1, w1, 11
	str	w1, [x2,840]
	ldrh	w1, [x2,820]
	lsl	w1, w1, 3
	sdiv	w0, w1, w0
	uxth	w0, w0
	cmp	w0, 4
	bhi	.L1277
	mov	w0, 4
.L1277:
	strh	w0, [x2,844]
	cbz	w27, .L1264
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 640
	strh	w1, [x0,820]
.L1264:
	add	x1, x19, :lo12:.LANCHOR2
	lsl	w20, w20, 6
	cmp	w23, 1
	ldrh	w0, [x1,820]
	asr	w0, w0, w3
	add	w3, w3, 9
	asr	w20, w20, w3
	strh	w20, [x1,848]
	and	w20, w20, 65535
	add	w0, w0, 2
	strh	w0, [x1,846]
	mul	w0, w20, w23
	str	w0, [x1,852]
	add	w20, w20, 8
	ldrh	w0, [x1,844]
	udiv	w0, w0, w23
	add	w20, w20, w0
	bne	.L1278
	add	w20, w20, 4
.L1278:
	str	w20, [x1,752]
	add	x20, x19, :lo12:.LANCHOR2
	add	x21, x21, :lo12:.LANCHOR0
	ldrh	w0, [x20,752]
	bl	FtlSysBlkNumInit
	str	wzr, [x21,4048]
	ldr	w0, [x20,752]
	str	w0, [x20,856]
	ldr	w0, [x20,768]
	ldrh	w1, [x20,824]
	lsl	w0, w0, 2
	ldrh	w3, [x20,830]
	ldrh	w2, [x20,844]
	mul	w1, w0, w1
	ldrh	w0, [x20,832]
	ldrb	w5, [x21,4044]
	add	w0, w0, 9
	lsr	w0, w1, w0
	mov	w1, 2048
	add	w0, w0, 2
	sdiv	w1, w1, w3
	strh	w1, [x20,862]
	uxth	w0, w0
	strh	w0, [x20,860]
	add	w1, w2, 3
	strh	w1, [x20,844]
	ldr	w1, [x20,852]
	add	w4, w1, 3
	str	w4, [x20,852]
	cbz	w5, .L1267
	add	w2, w2, 4
	add	w1, w1, 5
	strh	w2, [x20,844]
	b	.L1279
.L1267:
	cmp	w4, 7
	bhi	.L1268
	mov	w1, 8
.L1279:
	str	w1, [x20,852]
.L1268:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w1, [x19,764]
	strh	wzr, [x19,864]
	lsr	w2, w1, 3
	add	w1, w2, w1, lsl 1
	add	w1, w1, 52
	add	w0, w1, w0, lsl 2
	cmp	w0, w3, lsl 9
	bcs	.L1269
	mov	w0, 1
	strh	w0, [x19,864]
.L1269:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	stp	x29, x30, [sp, -64]!
	mov	w1, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	str	x23, [sp,48]
	add	x0, x20, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	str	w1, [x0,936]
	mov	w1, -1
	strh	wzr, [x0,866]
	strh	w1, [x0,952]
	strh	w1, [x0,954]
	strh	w1, [x0,956]
	str	wzr, [x0,868]
	str	wzr, [x0,872]
	str	wzr, [x0,876]
	str	wzr, [x0,880]
	str	wzr, [x0,884]
	str	wzr, [x0,888]
	str	wzr, [x0,892]
	str	wzr, [x0,896]
	str	wzr, [x0,900]
	str	wzr, [x0,904]
	str	wzr, [x0,908]
	str	wzr, [x0,912]
	str	wzr, [x0,916]
	str	wzr, [x0,920]
	str	wzr, [x0,924]
	str	wzr, [x0,928]
	str	wzr, [x0,932]
	str	wzr, [x0,940]
	str	wzr, [x0,944]
	str	wzr, [x0,948]
	strh	w1, [x0,958]
	mov	w1, 32
	ldrh	w2, [x0,830]
	strh	w1, [x0,960]
	mov	w1, 128
	strh	w1, [x0,962]
	mov	w1, 1024
	strh	wzr, [x0,964]
	sdiv	w2, w1, w2
	ldrh	w1, [x0,756]
	strh	wzr, [x0,966]
	lsl	w1, w1, 2
	strh	wzr, [x0,968]
	strh	wzr, [x0,970]
	cmp	w2, w1
	str	w2, [x0,972]
	bls	.L1282
	str	w1, [x0,972]
.L1282:
	add	x19, x20, :lo12:.LANCHOR2
	mov	w22, 56
	ldrh	w0, [x19,828]
	str	wzr, [x19,976]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x19,984]
	ldrh	w1, [x19,828]
	mov	w0, 12
	mul	w0, w1, w0
	bl	ftl_malloc
	ldrh	w21, [x19,756]
	str	x0, [x19,992]
	mul	w21, w21, w22
	lsl	w23, w21, 3
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,1000]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1008]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,1016]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1024]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1032]
	ldr	w0, [x19,972]
	mul	w0, w0, w22
	bl	ftl_malloc
	str	x0, [x19,1040]
	ldrh	w0, [x19,756]
	ldrh	w21, [x19,834]
	lsl	w0, w0, 1
	add	w0, w0, 1
	str	w0, [x19,1048]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1056]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1064]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1072]
	ldr	w0, [x19,1048]
	mul	w0, w21, w0
	bl	ftl_malloc
	str	x0, [x19,1080]
	ldr	w0, [x19,972]
	mul	w0, w21, w0
	bl	ftl_malloc
	str	x0, [x19,1088]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1096]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1104]
	ldr	w1, [x19,1048]
	mov	w0, 24
	mul	w0, w1, w0
	bl	ftl_malloc
	ldrh	w21, [x19,756]
	str	x0, [x19,1112]
	ldrh	w0, [x19,836]
	mul	w21, w0, w21
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1120]
	lsl	w0, w21, 3
	bl	ftl_malloc
	str	x0, [x19,1128]
	ldrh	w1, [x19,836]
	ldr	w0, [x19,1048]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,1136]
	ldrh	w1, [x19,836]
	ldr	w0, [x19,972]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,1144]
	ldrh	w0, [x19,766]
	lsl	w0, w0, 1
	strh	w0, [x19,1152]
	and	w0, w0, 65534
	bl	ftl_malloc
	str	x0, [x19,1160]
	ldrh	w0, [x19,1152]
	add	x0, x0, 547
	lsr	x0, x0, 9
	strh	w0, [x19,1152]
	lsl	w0, w0, 9
	bl	ftl_malloc
	ldrh	w21, [x19,766]
	str	x0, [x19,1168]
	add	x0, x0, 32
	lsl	w21, w21, 1
	str	x0, [x19,1176]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1184]
	mov	w0, w21
	bl	ftl_malloc
	ldr	w21, [x19,852]
	str	x0, [x19,1192]
	lsl	w21, w21, 1
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1200]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1208]
	ldrh	w0, [x19,766]
	lsr	w0, w0, 3
	add	w0, w0, 4
	bl	ftl_malloc
	adrp	x1, .LANCHOR0+16
	str	x0, [x1,#:lo12:.LANCHOR0+16]
	ldrh	w0, [x19,844]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x19,1216]
	ldrh	w0, [x19,844]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x19,1224]
	ldrh	w0, [x19,844]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,1232]
	ldrh	w0, [x19,846]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,1240]
	ldrh	w2, [x19,846]
	mov	w1, 0
	lsl	w2, w2, 2
	bl	ftl_memset
	ldrh	w21, [x19,860]
	lsl	w21, w21, 2
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1248]
	mov	w0, w21
	bl	ftl_malloc
	str	x0, [x19,1256]
	ldr	w0, [x19,852]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,1264]
	ldrh	w0, [x19,862]
	lsl	w0, w0, 4
	bl	ftl_malloc
	str	x0, [x19,1272]
	ldrh	w1, [x19,862]
	ldrh	w0, [x19,834]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,1280]
	ldrh	w1, [x19,766]
	mov	w0, 6
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,1288]
	ldrh	w0, [x19,822]
	ldrh	w1, [x19,778]
	add	w0, w0, 31
	asr	w0, w0, 5
	strh	w0, [x19,1296]
	mul	w0, w0, w1
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,1336]
	add	x1, x19, 1304
	ldrh	w4, [x19,1296]
	ldrh	w5, [x19,778]
	mov	w0, 1
	mov	w2, w4
.L1283:
	cmp	w0, w5
	add	x1, x1, 8
	bcs	.L1383
	ldr	x3, [x19,1336]
	add	w0, w0, 1
	add	x3, x3, x2, uxtw 2
	add	w2, w2, w4
	str	x3, [x1,32]
	b	.L1283
.L1383:
	add	x1, x20, :lo12:.LANCHOR2
.L1285:
	cmp	w0, 8
	beq	.L1384
	add	x2, x1, x0, uxtw 3
	add	w0, w0, 1
	str	xzr, [x2,1336]
	b	.L1285
.L1384:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x1, [x0,1200]
	cbnz	x1, .L1287
.L1289:
	adrp	x0, .LC29
	adrp	x1, .LANCHOR3
	add	x0, x0, :lo12:.LC29
	add	x1, x1, :lo12:.LANCHOR3
	bl	printk
	mov	w0, -1
	b	.L1288
.L1287:
	ldr	x1, [x0,1208]
	cbz	x1, .L1289
	ldr	x1, [x0,1248]
	cbz	x1, .L1289
	ldr	x1, [x0,1264]
	cbz	x1, .L1289
	ldr	x1, [x0,1272]
	cbz	x1, .L1289
	ldr	x1, [x0,1280]
	cbz	x1, .L1289
	ldr	x1, [x0,1288]
	cbz	x1, .L1289
	ldr	x1, [x0,1336]
	cbz	x1, .L1289
	ldr	x0, [x0,1192]
	cbz	x0, .L1289
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x1, [x0,984]
	cbz	x1, .L1289
	ldr	x1, [x0,992]
	cbz	x1, .L1289
	ldr	x1, [x0,1000]
	cbz	x1, .L1289
	ldr	x1, [x0,1016]
	cbz	x1, .L1289
	ldr	x1, [x0,1024]
	cbz	x1, .L1289
	ldr	x1, [x0,1032]
	cbz	x1, .L1289
	ldr	x1, [x0,1008]
	cbz	x1, .L1289
	ldr	x1, [x0,1056]
	cbz	x1, .L1289
	ldr	x1, [x0,1064]
	cbz	x1, .L1289
	ldr	x0, [x0,1072]
	cbz	x0, .L1289
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x1, [x0,1080]
	cbz	x1, .L1289
	ldr	x1, [x0,1096]
	cbz	x1, .L1289
	ldr	x1, [x0,1104]
	cbz	x1, .L1289
	ldr	x1, [x0,1112]
	cbz	x1, .L1289
	ldr	x1, [x0,1120]
	cbz	x1, .L1289
	ldr	x1, [x0,1128]
	cbz	x1, .L1289
	ldr	x1, [x0,1136]
	cbz	x1, .L1289
	ldr	x1, [x0,1176]
	cbz	x1, .L1289
	ldr	x1, [x0,1160]
	cbz	x1, .L1289
	ldr	x0, [x0,1216]
	cbz	x0, .L1289
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldr	x1, [x0,1224]
	cbz	x1, .L1289
	ldr	x1, [x0,1232]
	cbz	x1, .L1289
	ldr	x0, [x0,1240]
	cbz	x0, .L1289
	mov	w0, 0
.L1288:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldr	x23, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	IsBlkInVendorPart
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	adrp	x1, .LANCHOR2
	uxth	w4, w0
	add	x1, x1, :lo12:.LANCHOR2
	mov	w0, 0
	ldrh	w2, [x1,1400]
	cbz	w2, .L1386
	ldrh	w2, [x1,844]
	ldr	x3, [x1,1216]
	mov	x1, 0
.L1387:
	cmp	w2, w1, uxth
	bls	.L1392
	add	x1, x1, 1
	add	x0, x3, x1, lsl 1
	ldrh	w0, [x0,-2]
	cmp	w0, w4
	bne	.L1387
	mov	w0, 1
	b	.L1386
.L1392:
	mov	w0, 0
.L1386:
	ret
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheMetchLpa
	.type	FtlCacheMetchLpa, %function
FtlCacheMetchLpa:
	adrp	x2, .LANCHOR2
	mov	w6, w0
	add	x2, x2, :lo12:.LANCHOR2
	mov	w0, 0
	ldr	w3, [x2,976]
	cbz	w3, .L1394
	ldr	x5, [x2,1040]
	mov	x0, 56
	mov	x2, 0
.L1395:
	cmp	w3, w2
	bls	.L1404
	madd	x4, x2, x0, x5
	add	x2, x2, 1
	ldr	w4, [x4,24]
	cmp	w4, w6
	bcc	.L1395
	cmp	w4, w1
	bhi	.L1395
	mov	w0, 1
	b	.L1394
.L1404:
	mov	w0, 0
.L1394:
	ret
	.size	FtlCacheMetchLpa, .-FtlCacheMetchLpa
	.align	2
	.global	FtlGetCap
	.type	FtlGetCap, %function
FtlGetCap:
	adrp	x0, .LANCHOR0+4048
	ldr	w0, [x0,#:lo12:.LANCHOR0+4048]
	ret
	.size	FtlGetCap, .-FtlGetCap
	.align	2
	.global	ftl_get_density
	.type	ftl_get_density, %function
ftl_get_density:
	adrp	x0, .LANCHOR0+4048
	ldr	w0, [x0,#:lo12:.LANCHOR0+4048]
	ret
	.size	ftl_get_density, .-ftl_get_density
	.align	2
	.global	FtlGetLpn
	.type	FtlGetLpn, %function
FtlGetLpn:
	adrp	x0, .LANCHOR2+1404
	ldr	w0, [x0,#:lo12:.LANCHOR2+1404]
	ret
	.size	FtlGetLpn, .-FtlGetLpn
	.align	2
	.global	FtlBbmMapBadBlock
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	stp	x29, x30, [sp, -32]!
	uxth	w1, w0
	mov	w4, 1
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w3, [x19,822]
	add	x19, x19, 1304
	udiv	w2, w1, w3
	add	x0, x19, x2, uxth 3
	ldr	x0, [x0,32]
	msub	w3, w2, w3, w1
	uxth	w3, w3
	ubfx	x6, x3, 5, 11
	lsl	w4, w4, w3
	lsl	x6, x6, 2
	ldr	w5, [x0,x6]
	orr	w4, w4, w5
	str	w4, [x0,x6]
	adrp	x0, .LC30
	add	x0, x0, :lo12:.LC30
	bl	printk
	ldrh	w0, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,6]
	mov	w0, 0
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlBbmMapBadBlock, .-FtlBbmMapBadBlock
	.align	2
	.global	FtlBbmIsBadBlock
	.type	FtlBbmIsBadBlock, %function
FtlBbmIsBadBlock:
	adrp	x2, .LANCHOR2
	uxth	w0, w0
	add	x3, x2, :lo12:.LANCHOR2
	ldrh	w1, [x3,822]
	udiv	w2, w0, w1
	msub	w0, w2, w1, w0
	add	x2, x3, x2, uxth 3
	ldr	x1, [x2,1336]
	uxth	w0, w0
	ubfx	x3, x0, 5, 11
	ldr	w1, [x1,x3,lsl 2]
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
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	x20, x0
	adrp	x0, .LANCHOR2+1296
	mov	x19, x1
	ldrh	w2, [x0,#:lo12:.LANCHOR2+1296]
	mov	x0, x1
	mov	w1, 0
	lsl	w2, w2, 2
	bl	ftl_memset
	mov	x3, 0
	mov	w0, 65535
	mov	w5, 1
.L1413:
	ldrh	w2, [x20,x3]
	cmp	w2, w0
	beq	.L1411
	ubfx	x4, x2, 5, 11
	add	x3, x3, 2
	lsl	x4, x4, 2
	lsl	w2, w5, w2
	cmp	x3, 1024
	ldr	w1, [x19,x4]
	orr	w2, w1, w2
	str	w2, [x19,x4]
	bne	.L1413
.L1411:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	2
	.global	FtlBbmTblFlush
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	ldr	w1, [x19,1408]
	stp	x25, x26, [sp,64]
	cbnz	w1, .L1418
	ldr	x0, [x19,1056]
	mov	x22, 0
	ldr	x3, [x19,1120]
	add	x21, x19, 1304
	ldrh	w2, [x19,834]
	str	x0, [x19,1424]
	str	x3, [x19,1432]
	bl	ftl_memset
.L1419:
	ldrh	w1, [x19,778]
	mov	w0, w22
	add	x20, x19, 1416
	add	x22, x22, 1
	cmp	w0, w1
	bge	.L1427
	add	x1, x21, x22, lsl 3
	ldrh	w2, [x19,1296]
	ldr	x3, [x20,8]
	ldr	x1, [x1,24]
	mul	w0, w0, w2
	add	x0, x3, x0, sxtw 2
	lsl	w2, w2, 2
	bl	ftl_memcpy
	b	.L1419
.L1427:
	ldr	x26, [x20,16]
	mov	w1, 255
	mov	w2, 16
	mov	w22, 0
	mov	x0, x26
	adrp	x23, .LC31
	adrp	x25, .LC32
	mov	w24, w22
	bl	ftl_memset
	add	x23, x23, :lo12:.LC31
	mov	w0, -3887
	strh	w0, [x26]
	add	x25, x25, :lo12:.LC32
	ldr	w0, [x21,8]
	str	w0, [x26,4]
	ldrh	w0, [x19,1304]
	strh	w0, [x26,2]
	ldrh	w0, [x21,4]
	strh	w0, [x26,8]
	ldrh	w0, [x21,6]
	strh	w0, [x26,10]
	ldr	w0, [x19,752]
	strh	w0, [x26,12]
.L1421:
	ldr	x0, [x19,1056]
	str	x0, [x20,8]
	ldrh	w1, [x19,1304]
	ldr	x0, [x19,1120]
	str	x0, [x20,16]
	ldrh	w2, [x19,1306]
	ldrh	w3, [x19,1308]
	orr	w0, w2, w1, lsl 10
	ldrh	w4, [x26,10]
	str	w0, [x20,4]
	mov	x0, x23
	str	wzr, [x20]
	bl	printk
	ldrh	w1, [x19,826]
	ldrh	w0, [x19,1306]
	sub	w1, w1, #1
	cmp	w0, w1
	blt	.L1422
	ldr	w1, [x19,1312]
	ldrh	w0, [x19,1304]
	add	w1, w1, 1
	str	w1, [x19,1312]
	str	w1, [x26,4]
	strh	w0, [x26,8]
	ldrh	w1, [x19,1308]
	strh	w0, [x19,1308]
	ldr	x0, [x19,1024]
	strh	w1, [x19,1304]
	lsl	w1, w1, 10
	str	w1, [x20,4]
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	strh	wzr, [x19,1306]
	ldr	x0, [x19,1024]
	bl	FlashEraseBlocks
.L1422:
	mov	w1, 1
	mov	x0, x20
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
	ldrh	w1, [x19,1306]
	ldr	w0, [x20]
	add	w1, w1, 1
	strh	w1, [x19,1306]
	cmn	w0, #1
	bne	.L1423
	ldr	w1, [x20,4]
	mov	x0, x25
	add	w22, w22, 1
	bl	printk
	uxth	w22, w22
	cmp	w22, 3
	bls	.L1421
	adrp	x0, .LC33
	ldr	w1, [x20,4]
	add	x0, x0, :lo12:.LC33
	mov	w2, w22
	bl	printk
	mov	w0, 1
	str	w0, [x19,1408]
	b	.L1418
.L1423:
	cbnz	w24, .L1418
	mov	w24, 1
	b	.L1421
.L1418:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	stp	x29, x30, [sp, -80]!
	adrp	x1, .LANCHOR2
	add	x1, x1, :lo12:.LANCHOR2
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	add	x21, x1, 1416
	add	x23, x1, 1304
	ldr	x0, [x1,1056]
	mov	w22, 0
	ldr	x24, [x1,1120]
	mov	x19, x1
	str	x0, [x21,8]
	mov	w25, -1
	str	x24, [x21,16]
	mov	w26, 61664
.L1429:
	ldrh	w0, [x19,778]
	cmp	w22, w0
	bcs	.L1438
	ldrh	w20, [x19,822]
	strh	w25, [x23,12]
.L1431:
	sub	w20, w20, #1
	ldrh	w0, [x19,822]
	uxth	w20, w20
	sub	w1, w0, #16
	cmp	w20, w1
	ble	.L1432
	madd	w0, w22, w0, w20
	mov	w1, 1
	lsl	w0, w0, 10
	mov	w2, w1
	str	w0, [x21,4]
	mov	x0, x21
	bl	FlashReadPages
	ldr	w0, [x21]
	cmn	w0, #1
	beq	.L1431
	ldrh	w0, [x24]
	cmp	w0, w26
	bne	.L1431
	strh	w20, [x23,12]
.L1432:
	add	w22, w22, 1
	add	x23, x23, 2
	b	.L1429
.L1438:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FtlBbtMemInit
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	adrp	x0, .LANCHOR2
	mov	w1, -1
	stp	x29, x30, [sp, -16]!
	add	x0, x0, :lo12:.LANCHOR2
	mov	w2, 16
	add	x29, sp, 0
	add	x0, x0, 1316
	strh	w1, [x0,-12]
	mov	w1, 255
	strh	wzr, [x0,-6]
	bl	ftl_memset
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	stp	x29, x30, [sp, -48]!
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	mov	w19, 0
	mov	w20, w19
	ldrh	w1, [x0,822]
	ldrh	w21, [x0,778]
	mul	w21, w1, w21
.L1441:
	cmp	w20, w21
	bge	.L1447
	mov	w0, w20
	bl	FtlBbmIsBadBlock
	cbz	w0, .L1442
	add	w19, w19, 1
	uxth	w19, w19
.L1442:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L1441
.L1447:
	mov	w0, w19
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	2
	.global	FtlMakeBbt
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x20, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	mov	x21, x19
	ldr	w22, [x20,1408]
	cbnz	w22, .L1449
	add	x24, x20, 1304
	mov	x19, x20
	mov	x25, x24
	add	x20, x20, 1416
	mov	w28, -3872
	bl	FtlBbtMemInit
	bl	FtlLoadFactoryBbt
.L1450:
	ldrh	w0, [x19,778]
	cmp	w22, w0
	bcs	.L1468
	ldrh	w4, [x25,12]
	mov	w1, 65535
	ldr	x0, [x19,1056]
	ldr	x26, [x19,1120]
	cmp	w4, w1
	str	x0, [x20,8]
	str	x26, [x20,16]
	beq	.L1451
	ldrh	w0, [x19,822]
	mov	w1, 1
	mov	w2, w1
	madd	w27, w22, w0, w4
	lsl	w0, w27, 10
	str	w0, [x20,4]
	mov	x0, x20
	bl	FlashReadPages
	ldrh	w2, [x19,822]
	ldr	x0, [x24,32]
	add	w2, w2, 7
	ldr	x1, [x20,8]
	lsr	w2, w2, 3
	bl	ftl_memcpy
	b	.L1452
.L1451:
	mov	w1, w22
	bl	FlashGetBadBlockList
	ldr	x0, [x20,8]
	ldr	x1, [x24,32]
	bl	FtlBbt2Bitmap
	ldrh	w23, [x19,822]
.L1454:
	sub	w23, w23, #1
	uxth	w23, w23
.L1453:
	ldrh	w0, [x19,822]
	madd	w0, w22, w0, w23
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	beq	.L1454
	ldr	x0, [x19,1120]
	mov	w1, 0
	mov	w2, 16
	strh	w23, [x25,12]
	bl	ftl_memset
	strh	w28, [x26]
	str	wzr, [x26,4]
	ldrh	w2, [x19,1296]
	ldrh	w0, [x25,12]
	strh	w0, [x26,2]
	lsl	w2, w2, 2
	ldrh	w0, [x19,822]
	ldrh	w4, [x25,12]
	ldr	x1, [x24,32]
	madd	w27, w22, w0, w4
	lsl	w0, w27, 10
	str	w0, [x20,4]
	ldr	x0, [x20,8]
	bl	ftl_memcpy
	mov	w1, 1
	mov	x0, x20
	mov	w2, w1
	bl	FlashEraseBlocks
	mov	w1, 1
	mov	x0, x20
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
	ldr	w0, [x20]
	cmn	w0, #1
	bne	.L1452
	mov	w0, w27
	bl	FtlBbmMapBadBlock
	b	.L1453
.L1452:
	mov	w0, w27
	add	w22, w22, 1
	add	x24, x24, 8
	add	x25, x25, 2
	bl	FtlBbmMapBadBlock
	b	.L1450
.L1468:
	mov	w19, 0
.L1457:
	add	x20, x21, :lo12:.LANCHOR2
	ldrh	w0, [x20,838]
	cmp	w0, w19
	bls	.L1469
	mov	w0, w19
	add	w19, w19, 1
	bl	FtlBbmMapBadBlock
	uxth	w19, w19
	b	.L1457
.L1469:
	ldrh	w19, [x20,1316]
	mov	w23, 65535
	sub	w19, w19, #1
	uxth	w19, w19
.L1459:
	add	x22, x20, 1304
	ldrh	w0, [x22,12]
	sub	w0, w0, #48
	cmp	w19, w0
	ble	.L1463
	mov	w0, w19
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	beq	.L1460
	mov	w0, w19
	bl	FlashTestBlk
	cbz	w0, .L1461
	mov	w0, w19
	bl	FtlBbmMapBadBlock
	b	.L1460
.L1461:
	ldrh	w0, [x20,1304]
	cmp	w0, w23
	bne	.L1462
	strh	w19, [x20,1304]
	b	.L1460
.L1462:
	strh	w19, [x22,4]
	b	.L1463
.L1460:
	sub	w19, w19, #1
	uxth	w19, w19
	b	.L1459
.L1463:
	add	x19, x21, :lo12:.LANCHOR2
	mov	w2, 2
	ldr	x1, [x19,1024]
	ldrh	w0, [x19,1304]
	str	wzr, [x19,1312]
	lsl	w0, w0, 10
	str	w0, [x1,4]
	strh	wzr, [x19,1306]
	ldr	x0, [x19,1024]
	ldrh	w1, [x19,1308]
	lsl	w1, w1, 10
	str	w1, [x0,60]
	mov	w1, 1
	bl	FlashEraseBlocks
	ldrh	w0, [x19,1304]
	bl	FtlBbmMapBadBlock
	ldrh	w0, [x19,1308]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	strh	wzr, [x19,1306]
	ldr	w0, [x19,1312]
	ldrh	w1, [x19,1308]
	add	w0, w0, 1
	str	w0, [x19,1312]
	ldrh	w0, [x19,1304]
	strh	w0, [x19,1308]
	strh	w1, [x19,1304]
	bl	FtlBbmTblFlush
.L1449:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	V2P_block
	.type	V2P_block, %function
V2P_block:
	adrp	x3, .LANCHOR2
	uxth	w0, w0
	add	x3, x3, :lo12:.LANCHOR2
	uxth	w1, w1
	ldrh	w4, [x3,780]
	udiv	w2, w0, w4
	msub	w0, w2, w4, w0
	madd	w0, w1, w4, w0
	ldrh	w1, [x3,822]
	madd	w0, w2, w1, w0
	ret
	.size	V2P_block, .-V2P_block
	.align	2
	.global	P2V_plane
	.type	P2V_plane, %function
P2V_plane:
	adrp	x2, .LANCHOR2
	uxth	w0, w0
	add	x2, x2, :lo12:.LANCHOR2
	ldrh	w1, [x2,780]
	ldrh	w3, [x2,822]
	udiv	w2, w0, w1
	udiv	w3, w0, w3
	msub	w0, w2, w1, w0
	madd	w0, w1, w3, w0
	ret
	.size	P2V_plane, .-P2V_plane
	.align	2
	.global	P2V_block_in_plane
	.type	P2V_block_in_plane, %function
P2V_block_in_plane:
	adrp	x2, .LANCHOR2
	uxth	w0, w0
	add	x2, x2, :lo12:.LANCHOR2
	ldrh	w3, [x2,822]
	udiv	w1, w0, w3
	msub	w0, w1, w3, w0
	ldrh	w1, [x2,780]
	uxth	w0, w0
	udiv	w0, w0, w1
	ret
	.size	P2V_block_in_plane, .-P2V_block_in_plane
	.align	2
	.global	ftl_cmp_data_ver
	.type	ftl_cmp_data_ver, %function
ftl_cmp_data_ver:
	cmp	w0, w1
	mov	w2, -2147483648
	bls	.L1474
	sub	w1, w0, w1
	cmp	w1, w2
	cset	w0, ls
	b	.L1475
.L1474:
	sub	w1, w1, w0
	cmp	w1, w2
	cset	w0, hi
.L1475:
	ret
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	stp	x29, x30, [sp, -208]!
	cmp	w1, 1
	adrp	x3, .LANCHOR2
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	str	x25, [sp,64]
	mov	w23, w1
	add	x3, x3, :lo12:.LANCHOR2
	bne	.L1477
	ldrh	w19, [x3,826]
	b	.L1478
.L1477:
	ldrh	w19, [x3,824]
.L1478:
	sub	w19, w19, #1
	lsl	w21, w0, 10
	mov	w1, 1
	mov	w2, w23
	sxth	w19, w19
	str	xzr, [x29,96]
	orr	w0, w19, w21
	str	w0, [x29,92]
	add	x0, x29, 88
	add	x24, x29, 144
	str	x24, [x29,104]
	bl	FlashReadPages
	ldr	w0, [x29,144]
	cmn	w0, #1
	bne	.L1479
	mov	w22, 0
	mov	w25, 2
.L1480:
	cmp	w22, w19
	bgt	.L1479
	add	w3, w22, w19
	mov	w1, 1
	mov	w2, w23
	sdiv	w20, w3, w25
	sxth	w0, w20
	orr	w0, w0, w21
	str	w0, [x29,92]
	add	x0, x29, 88
	bl	FlashReadPages
	ldr	w0, [x24]
	cmn	w0, #1
	bne	.L1481
	ldr	w0, [x24,4]
	cmn	w0, #1
	bne	.L1481
	ldr	w0, [x29,88]
	cmn	w0, #1
	beq	.L1481
	sub	w19, w20, #1
	sxth	w19, w19
	b	.L1480
.L1481:
	add	w20, w20, 1
	sxth	w22, w20
	b	.L1480
.L1479:
	mov	w0, w19
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 208
	ret
	.size	FtlGetLastWrittenPage, .-FtlGetLastWrittenPage
	.align	2
	.global	FtlLoadBbt
	.type	FtlLoadBbt, %function
FtlLoadBbt:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	stp	x23, x24, [sp,48]
	add	x23, x19, :lo12:.LANCHOR2
	mov	w24, 61649
	stp	x21, x22, [sp,32]
	add	x21, x23, 1416
	ldr	x0, [x23,1056]
	ldr	x22, [x23,1120]
	str	x0, [x21,8]
	str	x22, [x21,16]
	bl	FtlBbtMemInit
	ldrh	w20, [x23,822]
	sub	w20, w20, #1
	uxth	w20, w20
.L1488:
	ldrh	w0, [x23,822]
	sub	w0, w0, #48
	cmp	w20, w0
	ble	.L1491
	lsl	w0, w20, 10
	mov	w1, 1
	str	w0, [x21,4]
	mov	w2, w1
	mov	x0, x21
	bl	FlashReadPages
	ldr	w0, [x21]
	cmn	w0, #1
	bne	.L1489
	ldr	w0, [x21,4]
	mov	w1, 1
	mov	w2, w1
	add	w0, w0, 1
	str	w0, [x21,4]
	mov	x0, x21
	bl	FlashReadPages
.L1489:
	ldr	w0, [x21]
	cmn	w0, #1
	beq	.L1490
	ldrh	w0, [x22]
	cmp	w0, w24
	bne	.L1490
	ldr	w1, [x22,4]
	str	w1, [x23,1312]
	strh	w20, [x23,1304]
	ldrh	w1, [x22,8]
	strh	w1, [x23,1308]
	b	.L1491
.L1490:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L1488
.L1491:
	add	x21, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	mov	w0, -1
	ldrh	w1, [x21,1304]
	cmp	w1, w2
	beq	.L1493
	ldrh	w1, [x21,1308]
	cmp	w1, w2
	beq	.L1495
	add	x0, x21, 1416
	lsl	w1, w1, 10
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	bl	FlashReadPages
	ldr	w0, [x21,1416]
	cmn	w0, #1
	beq	.L1495
	ldrh	w1, [x22]
	mov	w0, 61649
	cmp	w1, w0
	bne	.L1495
	ldr	w0, [x22,4]
	ldr	w1, [x21,1312]
	cmp	w0, w1
	bls	.L1495
	str	w0, [x21,1312]
	ldrh	w1, [x21,1308]
	ldrh	w0, [x22,8]
	strh	w1, [x21,1304]
	strh	w0, [x21,1308]
.L1495:
	add	x20, x19, :lo12:.LANCHOR2
	mov	w1, 1
	add	x23, x20, 1416
	mov	w24, 61649
	ldrh	w0, [x20,1304]
	bl	FtlGetLastWrittenPage
	sxth	w21, w0
	add	w0, w0, 1
	strh	w0, [x20,1306]
.L1497:
	tbnz	w21, #31, .L1502
	ldrh	w0, [x20,1304]
	mov	w1, 1
	mov	w2, w1
	orr	w0, w21, w0, lsl 10
	str	w0, [x23,4]
	ldr	x0, [x20,1056]
	str	x0, [x23,8]
	mov	x0, x23
	bl	FlashReadPages
	ldr	w0, [x23]
	cmn	w0, #1
	beq	.L1498
	ldrh	w0, [x22]
	cmp	w0, w24
	bne	.L1498
.L1502:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x22,10]
	mov	w2, 65535
	strh	w0, [x1,1310]
	ldrh	w0, [x22,12]
	cmp	w0, w2
	bne	.L1499
	b	.L1500
.L1498:
	sub	w21, w21, #1
	sxth	w21, w21
	b	.L1497
.L1499:
	ldr	w2, [x1,752]
	cmp	w0, w2
	beq	.L1500
	ldrh	w1, [x1,766]
	lsr	w1, w1, 2
	cmp	w0, w1
	bcs	.L1500
	cmp	w2, w1
	bcs	.L1500
	bl	FtlSysBlkNumInit
.L1500:
	add	x19, x19, :lo12:.LANCHOR2
	mov	x20, 0
	add	x21, x19, 1304
.L1503:
	ldrh	w0, [x19,778]
	mov	w1, w20
	add	x20, x20, 1
	cmp	w1, w0
	bcs	.L1521
	ldrh	w2, [x19,1296]
	add	x0, x21, x20, lsl 3
	ldr	x3, [x19,1424]
	mul	w1, w1, w2
	ldr	x0, [x0,24]
	lsl	w2, w2, 2
	add	x1, x3, x1, lsl 2
	bl	ftl_memcpy
	b	.L1503
.L1521:
	mov	w0, 0
.L1493:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	adrp	x1, .LANCHOR2
	mov	w2, 2048
	stp	x29, x30, [sp, -16]!
	add	x1, x1, :lo12:.LANCHOR2
	add	x29, sp, 0
	strh	w0, [x1,1472]
	add	x0, x1, 1480
	strh	wzr, [x1,1474]
	strh	wzr, [x1,1476]
	strh	wzr, [x1,1478]
	mov	w1, 0
	bl	ftl_memset
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	2
	.global	FtlFreeSysBlkQueueEmpty
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	adrp	x0, .LANCHOR2+1478
	ldrh	w0, [x0,#:lo12:.LANCHOR2+1478]
	cmp	w0, wzr
	cset	w0, eq
	ret
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	2
	.global	FtlFreeSysBlkQueueFull
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	adrp	x0, .LANCHOR2+1478
	ldrh	w0, [x0,#:lo12:.LANCHOR2+1478]
	cmp	w0, 1024
	cset	w0, eq
	ret
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	stp	x29, x30, [sp, -48]!
	uxth	w1, w1
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	stp	x19, x20, [sp,16]
	uxth	w21, w0
	mov	w0, 65533
	sub	w2, w21, #1
	cmp	w0, w2, uxth
	bcc	.L1525
	adrp	x2, .LANCHOR2
	add	x20, x2, :lo12:.LANCHOR2
	mov	x19, x2
	ldrh	w0, [x20,1478]
	cmp	w0, 1024
	beq	.L1525
	cbz	w1, .L1527
	ldr	w0, [x20,1408]
	cbnz	w0, .L1527
	mov	w0, w21
	bl	P2V_block_in_plane
	uxth	w22, w0
	ldr	x0, [x20,1024]
	lsl	w1, w21, 10
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	ldr	x0, [x20,1024]
	bl	FlashEraseBlocks
	ubfiz	x0, x22, 1, 16
	ldr	x2, [x20,1176]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
	ldr	w0, [x20,916]
	add	w0, w0, 1
	str	w0, [x20,916]
.L1527:
	add	x1, x19, :lo12:.LANCHOR2
	add	x1, x1, 1472
	ldrh	w0, [x1,6]
	ldrh	w2, [x1,4]
	add	w0, w0, 1
	strh	w0, [x1,6]
	add	x0, x1, x2, sxtw 1
	add	w2, w2, 1
	and	w2, w2, 1023
	strh	w21, [x0,8]
	strh	w2, [x1,4]
.L1525:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	add	x3, x21, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	uxth	w25, w0
	uxtb	w24, w1
	ldrb	w0, [x3,656]
	ldr	w2, [x3,1408]
	str	w0, [x29,120]
	mov	w0, 0
	cbnz	w2, .L1535
	adrp	x28, .LANCHOR0
	str	w25, [x3,948]
	add	x0, x28, :lo12:.LANCHOR0
	mov	w27, w2
	mov	w20, w2
	mov	w19, w2
	mov	x26, x3
	mov	w23, 56
	ldrb	w0, [x0,4044]
	str	w0, [x29,124]
.L1536:
	ldrh	w0, [x26,756]
	cmp	w0, w27
	bls	.L1580
	umull	x0, w27, w23
	ldr	x1, [x26,1024]
	str	wzr, [x1,x0]
	add	x0, x26, 784
	mov	w1, w25
	ldrb	w0, [x0,w27,sxtw]
	bl	V2P_block
	uxth	w22, w0
	mov	w1, w22
	cbz	w24, .L1537
	str	x1, [x29,112]
	bl	IsBlkInVendorPart
	ldr	x1, [x29,112]
	cbnz	w0, .L1538
.L1537:
	mov	w0, w1
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L1539
	umull	x0, w20, w23
	ldr	x1, [x26,1024]
	lsl	w22, w22, 10
	mov	w2, 4
	add	x1, x1, x0
	str	w22, [x1,4]
	ldr	x1, [x26,1024]
	ldr	x4, [x26,1128]
	add	x1, x1, x0
	ldrh	w0, [x26,836]
	mul	w0, w20, w0
	add	w20, w20, 1
	str	xzr, [x1,8]
	sdiv	w0, w0, w2
	uxth	w20, w20
	add	x0, x4, x0, sxtw 2
	str	x0, [x1,16]
	b	.L1538
.L1539:
	add	w19, w19, 1
	uxth	w19, w19
.L1538:
	add	w2, w27, 1
	uxth	w27, w2
	b	.L1536
.L1580:
	cbz	w20, .L1559
	ldr	w0, [x29,124]
	mov	w2, w20
	strb	wzr, [x26,656]
	mov	x23, 0
	cmp	w0, wzr
	ldr	x0, [x26,1024]
	cset	w22, ne
	mov	w1, w22
	bl	FlashEraseBlocks
	ldrb	w0, [x29,120]
	strb	w0, [x26,656]
	mov	x26, 56
.L1542:
	cmp	w20, w23, uxth
	bls	.L1581
	add	x1, x21, :lo12:.LANCHOR2
	mul	x0, x23, x26
	ldr	x1, [x1,1024]
	add	x2, x1, x0
	ldr	w0, [x1,x0]
	cmn	w0, #1
	bne	.L1543
	ldr	w0, [x2,4]
	add	w19, w19, 1
	lsr	w0, w0, 10
	uxth	w19, w19
	bl	FtlBbmMapBadBlock
.L1543:
	add	x23, x23, 1
	b	.L1542
.L1581:
	cbnz	w24, .L1545
	mov	w0, 1
	uxth	w22, w22
	mov	w27, 6
	str	w0, [x29,124]
	b	.L1546
.L1545:
	add	x0, x21, :lo12:.LANCHOR2
	add	x28, x28, :lo12:.LANCHOR0
	ldrh	w0, [x0,826]
	str	w0, [x29,124]
	ldrb	w0, [x28,4044]
	cbnz	w0, .L1562
	ldr	w0, [x29,124]
	mov	w22, 1
	lsr	w27, w0, 2
	b	.L1546
.L1562:
	mov	w22, 1
	mov	w27, w22
.L1546:
	mov	w23, 0
	add	x28, x21, :lo12:.LANCHOR2
.L1555:
	mov	w26, 0
	mov	w20, w26
.L1547:
	ldrh	w0, [x28,756]
	cmp	w0, w26
	bls	.L1582
	mov	w0, 56
	ldr	x1, [x28,1024]
	umull	x0, w26, w0
	str	wzr, [x1,x0]
	add	x0, x28, 784
	mov	w1, w25
	ldrb	w0, [x0,w26,sxtw]
	bl	V2P_block
	uxth	w2, w0
	str	w2, [x29,112]
	mov	w1, w2
	cbz	w24, .L1548
	str	x1, [x29,104]
	bl	IsBlkInVendorPart
	ldr	x1, [x29,104]
	cbnz	w0, .L1549
.L1548:
	mov	w0, w1
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L1549
	mov	w0, 56
	ldr	x1, [x28,1024]
	ldr	w2, [x29,112]
	umull	x0, w20, w0
	add	x1, x1, x0
	add	w2, w23, w2, lsl 10
	str	w2, [x1,4]
	mov	w2, 4
	ldr	x1, [x28,1024]
	add	x1, x1, x0
	ldr	x0, [x28,1096]
	str	x0, [x1,8]
	ldrh	w0, [x28,836]
	mul	w0, w20, w0
	add	w20, w20, 1
	sdiv	w0, w0, w2
	uxth	w20, w20
	ldr	x2, [x28,1104]
	add	x0, x2, x0, sxtw 2
	str	x0, [x1,16]
.L1549:
	add	w3, w26, 1
	uxth	w26, w3
	b	.L1547
.L1582:
	cbz	w20, .L1559
	ldr	x0, [x28,1024]
	mov	w1, w20
	mov	w2, w22
	mov	w3, 1
	strb	wzr, [x28,656]
	bl	FlashProgPages
	ldrb	w0, [x29,120]
	mov	x1, 0
	strb	w0, [x28,656]
.L1552:
	cmp	w20, w1, uxth
	bls	.L1583
	mov	x0, 56
	ldr	x2, [x28,1024]
	mul	x0, x1, x0
	add	x3, x2, x0
	ldr	w0, [x2,x0]
	cbz	w0, .L1553
	ldr	w0, [x3,4]
	add	w19, w19, 1
	str	x1, [x29,112]
	lsr	w0, w0, 10
	uxth	w19, w19
	bl	FtlBbmMapBadBlock
	ldr	x1, [x29,112]
.L1553:
	add	x1, x1, 1
	b	.L1552
.L1583:
	add	w23, w23, w27
	ldr	w0, [x29,124]
	uxth	w23, w23
	cmp	w23, w0
	bcc	.L1555
	mov	x23, 0
	mov	x26, 56
.L1556:
	cmp	w20, w23, uxth
	bls	.L1584
	cbz	w24, .L1557
	add	x1, x21, :lo12:.LANCHOR2
	mul	x0, x23, x26
	ldr	x1, [x1,1024]
	add	x2, x1, x0
	ldr	w0, [x1,x0]
	cbnz	w0, .L1557
	ldr	w0, [x2,4]
	mov	w1, 1
	lsr	w0, w0, 10
	bl	FtlFreeSysBlkQueueIn
.L1557:
	add	x23, x23, 1
	b	.L1556
.L1584:
	cmp	w25, 63
	bls	.L1563
	cbz	w24, .L1559
.L1563:
	add	x21, x21, :lo12:.LANCHOR2
	mov	w1, w22
	mov	w2, w20
	ldr	x0, [x21,1024]
	bl	FlashEraseBlocks
.L1559:
	mov	w0, w19
.L1535:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	adrp	x1, .LANCHOR2
	add	x0, x1, :lo12:.LANCHOR2
	add	x0, x0, 1472
	ldrh	w6, [x0,2084]
	ldrh	w2, [x0,6]
	cbz	w2, .L1585
	mov	w5, 0
	ldrh	w3, [x0,2]
	ldrh	w2, [x0,4]
	mov	w4, w5
	and	w6, w6, 31
.L1587:
	cmp	w4, w6
	bge	.L1595
	add	x5, x0, x3, sxtw 1
	ldrh	w7, [x5,8]
	add	x5, x0, x2, sxtw 1
	add	w4, w4, 1
	strh	w7, [x5,8]
	mov	w5, 1
	add	w3, w3, 1
	add	w2, w2, w5
	uxth	w4, w4
	and	w3, w3, 1023
	and	w2, w2, 1023
	b	.L1587
.L1595:
	cbz	w5, .L1585
	add	x0, x1, :lo12:.LANCHOR2
	strh	w3, [x0,1474]
	strh	w2, [x0,1476]
.L1585:
	ret
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	stp	x23, x24, [sp,48]
	add	x19, x19, :lo12:.LANCHOR2
	adrp	x23, .LC35
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	add	x21, x19, 1472
	adrp	x24, .LANCHOR0
	add	x23, x23, :lo12:.LC35
.L1597:
	ldrh	w1, [x21,6]
	adrp	x22, .LANCHOR2
	cbz	w1, .L1598
	ldrh	w2, [x21,2]
	sub	w1, w1, #1
	add	x0, x21, x2, sxtw 1
	strh	w1, [x21,6]
	add	w2, w2, 1
	ldr	w1, [x19,1408]
	and	w2, w2, 1023
	strh	w2, [x21,2]
	ldrh	w20, [x0,8]
	cbnz	w1, .L1599
	mov	w0, w20
	str	x1, [x29,88]
	bl	P2V_block_in_plane
	uxth	w26, w0
	ldr	x0, [x19,1024]
	lsl	w2, w20, 10
	ldr	x1, [x29,88]
	str	w2, [x0,4]
	add	x0, x24, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L1600
	ldr	x0, [x19,1024]
	mov	w2, 1
	bl	FlashEraseBlocks
.L1600:
	add	x25, x22, :lo12:.LANCHOR2
	mov	w1, 1
	mov	w2, w1
	ldr	x0, [x25,1024]
	bl	FlashEraseBlocks
	ubfiz	x0, x26, 1, 16
	ldr	x2, [x25,1176]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
	ldr	w0, [x25,916]
	add	w0, w0, 1
	str	w0, [x25,916]
	b	.L1599
.L1598:
	adrp	x0, .LC34
	add	x0, x0, :lo12:.LC34
	bl	printk
.L1601:
	b	.L1601
.L1599:
	sub	w0, w20, #1
	mov	w1, 65533
	cmp	w1, w0, uxth
	bcs	.L1602
	add	x22, x22, :lo12:.LANCHOR2
	mov	x0, x23
	mov	w1, w20
	ldrh	w2, [x22,1478]
	bl	printk
	b	.L1597
.L1602:
	mov	w0, w20
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	test_node_in_list
	.type	test_node_in_list, %function
test_node_in_list:
	ldr	x2, [x0]
	adrp	x0, .LANCHOR2+1288
	mov	x4, -6148914691236517206
	uxth	w1, w1
	mov	w5, 65535
	ldr	x3, [x0,#:lo12:.LANCHOR2+1288]
	sub	x0, x2, x3
	asr	x0, x0, 1
	madd	x0, x4, x0, x0
	mov	w4, 6
	uxth	w0, w0
.L1608:
	cmp	w1, w0
	beq	.L1609
	ldrh	w0, [x2]
	cmp	w0, w5
	beq	.L1610
	umull	x2, w0, w4
	add	x2, x3, x2
	b	.L1608
.L1609:
	mov	w0, 1
	b	.L1607
.L1610:
	mov	w0, 0
.L1607:
	ret
	.size	test_node_in_list, .-test_node_in_list
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	adrp	x2, .LANCHOR2
	uxth	w0, w0
	add	x3, x2, :lo12:.LANCHOR2
	mov	x4, x2
	ldrh	w1, [x3,764]
	cmp	w1, w0
	bls	.L1613
	mov	w7, 6
	ldr	x11, [x3,1288]
	mov	w1, -1
	umull	x10, w0, w7
	add	x5, x11, x10
	strh	w1, [x5,2]
	strh	w1, [x11,x10]
	ldr	x1, [x3,3576]
	cbnz	x1, .L1614
	str	x5, [x3,3576]
	b	.L1613
.L1614:
	ubfiz	x8, x0, 1, 16
	ldr	x13, [x3,1192]
	ldrh	w2, [x5,4]
	mov	x9, -6148914691236517206
	ldr	x12, [x3,1288]
	mov	w18, 65535
	ldrh	w6, [x13,x8]
	cmp	w2, wzr
	ldr	x14, [x3,1176]
	ldrh	w17, [x3,764]
	mov	w3, 0
	mul	w6, w6, w2
	sub	x2, x1, x12
	asr	x2, x2, 1
	csinv	w6, w6, wzr, ne
	madd	x2, x9, x2, x2
	add	x9, x14, x8
	mov	w8, w7
	uxth	w2, w2
.L1622:
	add	w3, w3, 1
	cmp	w0, w2
	uxth	w3, w3
	beq	.L1613
	cmp	w3, w17
	bhi	.L1613
	ubfiz	x15, x2, 1, 16
	ldrh	w16, [x1,4]
	cmp	w16, wzr
	ldrh	w7, [x13,x15]
	mul	w7, w7, w16
	csinv	w7, w7, wzr, ne
	cmp	w7, w6
	bne	.L1618
	ldrh	w15, [x14,x15]
	ldrh	w7, [x9]
	cmp	w15, w7
	bcc	.L1620
	b	.L1619
.L1618:
	bhi	.L1619
.L1620:
	ldrh	w7, [x1]
	cmp	w7, w18
	bne	.L1621
	strh	w2, [x5,2]
	add	x2, x4, :lo12:.LANCHOR2
	strh	w0, [x1]
	str	x5, [x2,3584]
	b	.L1613
.L1621:
	umull	x1, w7, w8
	mov	w2, w7
	add	x1, x12, x1
	b	.L1622
.L1619:
	strh	w2, [x11,x10]
	ldrh	w2, [x1,2]
	strh	w2, [x5,2]
	add	x2, x4, :lo12:.LANCHOR2
	ldr	x3, [x2,3576]
	cmp	x1, x3
	bne	.L1623
	strh	w0, [x1,2]
	str	x5, [x2,3576]
	b	.L1613
.L1623:
	ldrh	w3, [x1,2]
	mov	w4, 6
	ldr	x2, [x2,1288]
	umull	x3, w3, w4
	strh	w0, [x2,x3]
	strh	w0, [x1,2]
.L1613:
	mov	w0, 0
	ret
	.size	insert_data_list, .-insert_data_list
	.align	2
	.global	INSERT_DATA_LIST
	.type	INSERT_DATA_LIST, %function
INSERT_DATA_LIST:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	insert_data_list
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldrh	w1, [x0,3592]
	add	w1, w1, 1
	strh	w1, [x0,3592]
	ldp	x29, x30, [sp], 16
	ret
	.size	INSERT_DATA_LIST, .-INSERT_DATA_LIST
	.align	2
	.global	insert_free_list
	.type	insert_free_list, %function
insert_free_list:
	uxth	w0, w0
	mov	w7, 65535
	cmp	w0, w7
	beq	.L1628
	adrp	x2, .LANCHOR2
	mov	w6, 6
	add	x5, x2, :lo12:.LANCHOR2
	mov	w1, -1
	umull	x8, w0, w6
	mov	x3, x2
	ldr	x9, [x5,1288]
	add	x4, x9, x8
	strh	w1, [x4,2]
	strh	w1, [x9,x8]
	ldr	x1, [x5,3600]
	cbnz	x1, .L1629
	str	x4, [x5,3600]
	b	.L1628
.L1629:
	ldr	x11, [x5,1176]
	ubfiz	x2, x0, 1, 16
	ldr	x10, [x5,1288]
	mov	x5, -6148914691236517206
	ldrh	w12, [x11,x2]
	sub	x2, x1, x10
	asr	x2, x2, 1
	madd	x2, x5, x2, x2
	uxth	w2, w2
.L1632:
	ubfiz	x5, x2, 1, 16
	ldrh	w5, [x11,x5]
	cmp	w5, w12
	bcs	.L1630
	ldrh	w5, [x1]
	cmp	w5, w7
	bne	.L1631
	strh	w2, [x4,2]
	strh	w0, [x1]
	b	.L1628
.L1631:
	umull	x1, w5, w6
	mov	w2, w5
	add	x1, x10, x1
	b	.L1632
.L1630:
	ldrh	w5, [x1,2]
	strh	w5, [x4,2]
	strh	w2, [x9,x8]
	add	x2, x3, :lo12:.LANCHOR2
	ldr	x3, [x2,3600]
	cmp	x1, x3
	bne	.L1633
	strh	w0, [x1,2]
	str	x4, [x2,3600]
	b	.L1628
.L1633:
	ldrh	w3, [x1,2]
	mov	w4, 6
	ldr	x2, [x2,1288]
	umull	x3, w3, w4
	strh	w0, [x2,x3]
	strh	w0, [x1,2]
.L1628:
	mov	w0, 0
	ret
	.size	insert_free_list, .-insert_free_list
	.align	2
	.global	INSERT_FREE_LIST
	.type	INSERT_FREE_LIST, %function
INSERT_FREE_LIST:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	insert_free_list
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldrh	w1, [x0,3608]
	add	w1, w1, 1
	strh	w1, [x0,3608]
	ldp	x29, x30, [sp], 16
	ret
	.size	INSERT_FREE_LIST, .-INSERT_FREE_LIST
	.align	2
	.global	List_remove_node
	.type	List_remove_node, %function
List_remove_node:
	adrp	x6, .LANCHOR2
	uxth	w1, w1
	add	x6, x6, :lo12:.LANCHOR2
	mov	w5, 6
	ldr	x3, [x0]
	mov	w7, 65535
	umull	x1, w1, w5
	ldr	x2, [x6,1288]
	add	x4, x2, x1
	cmp	x4, x3
	bne	.L1636
	ldrh	w3, [x2,x1]
	cmp	w3, w7
	bne	.L1637
	str	xzr, [x0]
	b	.L1638
.L1637:
	umull	x3, w3, w5
	add	x3, x2, x3
	str	x3, [x0]
	mov	w0, -1
	strh	w0, [x3,2]
	b	.L1638
.L1636:
	ldrh	w0, [x2,x1]
	ldrh	w3, [x4,2]
	cmp	w0, w7
	bne	.L1639
	cmp	w3, w0
	beq	.L1638
	umull	x3, w3, w5
	mov	w0, -1
	strh	w0, [x2,x3]
	b	.L1638
.L1639:
	umull	x0, w0, w5
	add	x0, x2, x0
	strh	w3, [x0,2]
	ldrh	w3, [x4,2]
	ldr	x0, [x6,1288]
	ldrh	w7, [x2,x1]
	umull	x3, w3, w5
	strh	w7, [x0,x3]
.L1638:
	mov	w0, -1
	strh	w0, [x2,x1]
	strh	w0, [x4,2]
	mov	w0, 0
	ret
	.size	List_remove_node, .-List_remove_node
	.align	2
	.global	List_pop_index_node
	.type	List_pop_index_node, %function
List_pop_index_node:
	stp	x29, x30, [sp, -32]!
	uxth	w1, w1
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	w19, 65535
	ldr	x2, [x0]
	cbz	x2, .L1641
	adrp	x3, .LANCHOR2+1288
	mov	w5, w19
	mov	w6, 6
	ldr	x4, [x3,#:lo12:.LANCHOR2+1288]
.L1642:
	cbnz	w1, .L1643
.L1645:
	sub	x2, x2, x4
	mov	x1, -6148914691236517206
	asr	x2, x2, 1
	madd	x2, x1, x2, x2
	uxth	w19, w2
	mov	w1, w19
	bl	List_remove_node
	b	.L1641
.L1643:
	ldrh	w3, [x2]
	cmp	w3, w5
	beq	.L1645
	umull	x3, w3, w6
	sub	w1, w1, #1
	add	x2, x4, x3
	uxth	w1, w1
	b	.L1642
.L1641:
	mov	w0, w19
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	List_pop_index_node, .-List_pop_index_node
	.align	2
	.global	List_get_gc_head_node
	.type	List_get_gc_head_node, %function
List_get_gc_head_node:
	adrp	x2, .LANCHOR2
	uxth	w3, w0
	add	x2, x2, :lo12:.LANCHOR2
	mov	w0, 65535
	ldr	x1, [x2,3576]
	cbz	x1, .L1653
	ldr	x4, [x2,1288]
	mov	w5, 6
.L1649:
	cbz	w3, .L1650
	ldrh	w2, [x1]
	cmp	w2, w0
	beq	.L1653
	umull	x2, w2, w5
	sub	w3, w3, #1
	add	x1, x4, x2
	uxth	w3, w3
	b	.L1649
.L1650:
	sub	x1, x1, x4
	mov	x0, -6148914691236517206
	asr	x1, x1, 1
	madd	x1, x0, x1, x1
	uxth	w0, w1
.L1653:
	ret
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	2
	.global	List_update_data_list
	.type	List_update_data_list, %function
List_update_data_list:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	uxth	w20, w0
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w1, [x0,3616]
	cmp	w1, w20
	beq	.L1655
	ldrh	w1, [x0,3664]
	cmp	w1, w20
	beq	.L1655
	ldrh	w1, [x0,3712]
	cmp	w1, w20
	beq	.L1655
	mov	w3, 6
	ldr	x4, [x0,1288]
	ldr	x2, [x0,3576]
	umull	x3, w20, w3
	add	x1, x4, x3
	cmp	x1, x2
	beq	.L1655
	ldr	x5, [x0,1192]
	ubfiz	x0, x20, 1, 16
	ldrh	w2, [x5,x0]
	ldrh	w0, [x1,4]
	ldrh	w1, [x1,2]
	cmp	w0, wzr
	mul	w2, w2, w0
	mov	w0, 65535
	csinv	w2, w2, wzr, ne
	cmp	w1, w0
	bne	.L1657
	ldrh	w0, [x4,x3]
	cmp	w0, w1
	beq	.L1655
.L1657:
	mov	w0, 6
	mov	x3, -6148914691236517206
	umull	x1, w1, w0
	asr	x0, x1, 1
	add	x1, x4, x1
	madd	x0, x3, x0, x0
	ldrh	w1, [x1,4]
	ldrh	w0, [x5,x0,lsl 1]
	cmp	w1, wzr
	mul	w0, w0, w1
	csinv	w0, w0, wzr, ne
	cmp	w2, w0
	bcs	.L1655
	add	x19, x19, :lo12:.LANCHOR2
	mov	w1, w20
	add	x0, x19, 3576
	bl	List_remove_node
	ldrh	w0, [x19,3592]
	sub	w0, w0, #1
	strh	w0, [x19,3592]
	mov	w0, w20
	bl	INSERT_DATA_LIST
.L1655:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	List_update_data_list, .-List_update_data_list
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	stp	x29, x30, [sp, -80]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	str	x25, [sp,64]
	mov	x19, x0
	ldrh	w2, [x0,10]
	ldr	x21, [x0,32]
	ldr	x22, [x0,16]
	lsl	w2, w2, 1
	ldr	x20, [x0,40]
	mov	x0, x21
	bl	ftl_memset
	mov	w0, 0
.L1662:
	ldrh	w1, [x19,6]
	cmp	w1, w0
	bls	.L1684
	ubfiz	x1, x0, 2, 16
	ldr	w2, [x20,x1]
	mov	w1, 0
	ubfx	x2, x2, 10, 16
.L1663:
	ldrh	w3, [x19,10]
	cmp	w3, w1
	bls	.L1685
	ubfiz	x3, x1, 1, 16
	ldrh	w4, [x22,x3]
	cmp	w2, w4
	bne	.L1664
	cbz	w4, .L1664
	ldrh	w4, [x21,x3]
	add	w4, w4, 1
	strh	w4, [x21,x3]
.L1664:
	add	w1, w1, 1
	uxth	w1, w1
	b	.L1663
.L1685:
	add	w0, w0, 1
	uxth	w0, w0
	b	.L1662
.L1684:
	adrp	x0, .LANCHOR2+826
	mov	w23, 0
	mov	w20, w23
	ldrh	w1, [x0,#:lo12:.LANCHOR2+826]
	ldrh	w0, [x19]
	strh	w1, [x21,x0,lsl 1]
	ldrh	w24, [x21]
.L1667:
	ldrh	w0, [x19,10]
	cmp	w0, w20
	bls	.L1686
	ubfiz	x2, x20, 1, 16
	ldrh	w1, [x21,x2]
	cmp	w24, w1
	bls	.L1668
	ldrh	w0, [x22,x2]
	add	x25, x22, x2
	cbnz	w0, .L1669
	b	.L1670
.L1668:
	cbnz	w1, .L1670
	ldrh	w0, [x22,x2]
	add	x25, x22, x2
	cbz	w0, .L1670
.L1672:
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
	strh	wzr, [x25]
	ldrh	w0, [x19,8]
	sub	w0, w0, #1
	strh	w0, [x19,8]
.L1670:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L1667
.L1669:
	mov	w23, w20
	mov	w24, w1
	cbz	w1, .L1672
	b	.L1670
.L1686:
	mov	w0, w23
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	ftl_map_blk_alloc_new_blk
	.type	ftl_map_blk_alloc_new_blk, %function
ftl_map_blk_alloc_new_blk:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	mov	w20, 0
	ldrh	w2, [x0,10]
	ldr	x1, [x0,16]
.L1688:
	cmp	w20, w2
	beq	.L1692
	mov	x22, x1
	ldrh	w21, [x1],2
	cbnz	w21, .L1689
	mov	x19, x0
	bl	FtlFreeSysBlkQueueOut
	strh	w0, [x22]
	uxth	w1, w0
	sub	w2, w1, #1
	mov	w0, 65533
	cmp	w0, w2, uxth
	bcs	.L1690
	adrp	x2, .LANCHOR2+1478
	adrp	x0, .LC36
	add	x0, x0, :lo12:.LC36
	ldrh	w2, [x2,#:lo12:.LANCHOR2+1478]
	bl	printk
.L1691:
	b	.L1691
.L1690:
	ldr	w0, [x19,48]
	strh	w21, [x19,2]
	add	w0, w0, 1
	str	w0, [x19,48]
	ldrh	w0, [x19,8]
	strh	w20, [x19]
	add	w0, w0, 1
	strh	w0, [x19,8]
	b	.L1692
.L1689:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L1688
.L1692:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	FtlMapWritePage
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	stp	x23, x24, [sp,48]
	adrp	x26, .LANCHOR2
	adrp	x23, .LC37
	stp	x21, x22, [sp,32]
	stp	x27, x28, [sp,80]
	stp	x19, x20, [sp,16]
	mov	w25, w1
	mov	x19, x0
	mov	x27, x2
	mov	w22, 0
	mov	w28, 65535
	add	x21, x26, :lo12:.LANCHOR2
	add	x23, x23, :lo12:.LC37
.L1695:
	add	x0, x26, :lo12:.LANCHOR2
	adrp	x20, .LANCHOR2
	ldr	w1, [x0,900]
	add	w1, w1, 1
	str	w1, [x0,900]
	ldrh	w0, [x0,826]
	ldrh	w1, [x19,2]
	sub	w0, w0, #1
	cmp	w1, w0
	bge	.L1696
	ldrh	w0, [x19]
	cmp	w0, w28
	bne	.L1697
.L1696:
	mov	x0, x19
	bl	Ftl_write_map_blk_to_last_page
.L1697:
	ldr	w1, [x21,1408]
	cbnz	w1, .L1698
	ldrh	w2, [x19]
	add	x3, x21, 1416
	ldr	x0, [x19,16]
	str	x3, [x29,104]
	ldrh	w24, [x0,x2,lsl 1]
	mov	w2, 16
	ldrh	w0, [x19,2]
	str	x27, [x21,1424]
	orr	w0, w0, w24, lsl 10
	str	w0, [x21,1420]
	ldr	x0, [x21,1120]
	str	x0, [x21,1432]
	bl	ftl_memset
	ldr	x3, [x29,104]
	ldr	w0, [x19,48]
	ldr	x2, [x3,16]
	str	w0, [x2,4]
	strh	w25, [x2,8]
	ldrh	w0, [x19,4]
	strh	w0, [x2]
	adrp	x0, .LANCHOR0+24
	strh	w24, [x2,2]
	ldrb	w0, [x0,#:lo12:.LANCHOR0+24]
	cbz	w0, .L1699
	ldr	x0, [x3,8]
	ldrh	w1, [x21,834]
	str	x2, [x29,104]
	bl	js_hash
	ldr	x2, [x29,104]
	str	w0, [x2,12]
.L1699:
	add	x4, x21, 1416
	mov	w1, 1
	mov	w2, w1
	mov	x0, x4
	mov	w3, w1
	str	x4, [x29,104]
	bl	FlashProgPages
	ldrh	w1, [x19,2]
	ldr	x4, [x29,104]
	add	w1, w1, 1
	uxth	w1, w1
	strh	w1, [x19,2]
	ldr	w2, [x21,1416]
	cmn	w2, #1
	bne	.L1700
	ldr	w1, [x4,4]
	mov	x0, x23
	add	w22, w22, 1
	bl	printk
	uxth	w22, w22
	ldrh	w0, [x19,2]
	cmp	w0, 2
	bhi	.L1701
	ldrh	w0, [x21,826]
	sub	w0, w0, #1
	strh	w0, [x19,2]
.L1701:
	cmp	w22, 3
	bls	.L1695
	add	x20, x20, :lo12:.LANCHOR2
	adrp	x0, .LC38
	add	x0, x0, :lo12:.LC38
	mov	w2, w22
	ldr	w1, [x20,1420]
	bl	printk
	mov	w0, 1
	str	w0, [x20,1408]
	b	.L1698
.L1700:
	cbz	w2, .L1703
	strh	w24, [x19,60]
.L1703:
	cmp	w1, 1
	beq	.L1704
	cmp	w2, 256
	beq	.L1704
	ldr	w0, [x19,56]
	cbz	w0, .L1705
.L1704:
	str	wzr, [x19,56]
	b	.L1695
.L1705:
	add	x20, x20, :lo12:.LANCHOR2
	ldr	x0, [x19,40]
	ldr	w1, [x20,1420]
	str	w1, [x0,w25,uxtw 2]
.L1698:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlMapWritePage, .-FtlMapWritePage
	.align	2
	.global	ftl_map_blk_gc
	.type	ftl_map_blk_gc, %function
ftl_map_blk_gc:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	str	x27, [sp,80]
	mov	x19, x0
	adrp	x20, .LANCHOR2
	ldr	x21, [x0,16]
	ldr	x25, [x0,40]
	bl	ftl_free_no_use_map_blk
	ldrh	w1, [x19,10]
	ldrh	w2, [x19,8]
	sub	w1, w1, #4
	cmp	w2, w1
	blt	.L1713
	ubfiz	x1, x0, 1, 16
	ldrh	w24, [x21,x1]
	cbz	w24, .L1713
	ldr	w0, [x19,52]
	cbnz	w0, .L1713
	mov	w2, 1
	str	w2, [x19,52]
	strh	w0, [x21,x1]
	ldrh	w0, [x19,8]
	ldrh	w1, [x19,2]
	sub	w0, w0, #1
	strh	w0, [x19,8]
	add	x0, x20, :lo12:.LANCHOR2
	ldrh	w0, [x0,826]
	cmp	w1, w0
	bcc	.L1714
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L1714:
	add	x26, x20, :lo12:.LANCHOR2
	mov	w21, 0
	add	x22, x26, 1416
.L1715:
	ldrh	w0, [x19,6]
	cmp	w0, w21
	bls	.L1724
	ubfiz	x0, x21, 2, 16
	add	x27, x25, x0
	ldr	w1, [x25,x0]
	cmp	w24, w1, lsr 10
	bne	.L1716
	ldr	x1, [x26,1064]
	ldr	x23, [x26,1120]
	str	x1, [x22,8]
	mov	w1, 1
	str	x23, [x22,16]
	mov	w2, w1
	ldr	w0, [x25,x0]
	str	w0, [x22,4]
	mov	x0, x22
	bl	FlashReadPages
	ldr	w0, [x22]
	cmn	w0, #1
	bne	.L1717
.L1719:
	add	x20, x20, :lo12:.LANCHOR2
	str	wzr, [x27]
	adrp	x0, .LC39
	ldrh	w2, [x23,8]
	add	x0, x0, :lo12:.LC39
	ldr	w1, [x20,1420]
	bl	printk
	mov	w0, 1
	str	w0, [x20,1408]
	b	.L1718
.L1717:
	ldrh	w0, [x23,8]
	cmp	w0, w21
	bne	.L1719
	ldrh	w1, [x23]
	ldrh	w0, [x19,4]
	cmp	w1, w0
	bne	.L1719
	ldr	x2, [x22,8]
	mov	x0, x19
	mov	w1, w21
	bl	FtlMapWritePage
.L1716:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1715
.L1724:
	mov	w0, w24
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
	str	wzr, [x19,52]
.L1713:
	add	x20, x20, :lo12:.LANCHOR2
	ldrh	w1, [x19,2]
	ldrh	w0, [x20,826]
	cmp	w1, w0
	bcc	.L1718
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L1718:
	mov	w0, 0
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	add	x1, x21, :lo12:.LANCHOR2
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	ldr	w20, [x1,1408]
	ldr	x22, [x0,16]
	ldr	x24, [x0,40]
	cbnz	w20, .L1726
	mov	x19, x0
	ldrh	w0, [x0]
	mov	w2, 65535
	cmp	w0, w2
	bne	.L1727
	ldrh	w0, [x19,8]
	add	w0, w0, 1
	strh	w0, [x19,8]
	bl	FtlFreeSysBlkQueueOut
	strh	w0, [x22]
	strh	w20, [x19,2]
	ldr	w0, [x19,48]
	strh	w20, [x19]
	add	w0, w0, 1
	str	w0, [x19,48]
	b	.L1726
.L1727:
	ubfiz	x0, x0, 1, 16
	ldrh	w2, [x19,2]
	ldrh	w23, [x22,x0]
	ldr	x22, [x1,1120]
	str	x22, [x1,1432]
	orr	w2, w2, w23, lsl 10
	str	w2, [x1,1420]
	ldr	x2, [x1,1056]
	str	x2, [x1,1424]
	ldr	w0, [x19,48]
	str	w0, [x22,4]
	mov	w0, -1291
	strh	w0, [x22,8]
	ldrh	w0, [x19,4]
	strh	w0, [x22]
	strh	w23, [x22,2]
	ldrh	w2, [x1,826]
	ldr	x0, [x1,1056]
	mov	w1, 255
	lsl	w2, w2, 3
	bl	ftl_memset
	mov	w0, w20
.L1728:
	ldrh	w1, [x19,6]
	cmp	w1, w0
	bls	.L1735
	ubfiz	x3, x0, 2, 32
	ldr	w1, [x24,x3]
	cmp	w23, w1, lsr 10
	bne	.L1729
	add	x2, x21, :lo12:.LANCHOR2
	add	w20, w20, 1
	uxth	w20, w20
	ldr	x4, [x2,1056]
	ubfiz	x1, x20, 3, 16
	str	w0, [x4,x1]
	ldr	x2, [x2,1056]
	ldr	w3, [x24,x3]
	add	x1, x2, x1
	str	w3, [x1,4]
.L1729:
	add	w0, w0, 1
	uxth	w0, w0
	b	.L1728
.L1735:
	adrp	x0, .LANCHOR0+24
	ldrb	w0, [x0,#:lo12:.LANCHOR0+24]
	cbz	w0, .L1731
	add	x1, x21, :lo12:.LANCHOR2
	ldr	x0, [x1,1424]
	ldrh	w1, [x1,834]
	bl	js_hash
	str	w0, [x22,12]
.L1731:
	mov	w1, 1
	add	x0, x21, :lo12:.LANCHOR2
	add	x0, x0, 1416
	mov	w2, w1
	mov	w3, 0
	bl	FlashProgPages
	ldrh	w0, [x19,2]
	add	w0, w0, 1
	strh	w0, [x19,2]
	mov	x0, x19
	bl	ftl_map_blk_gc
.L1726:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	flush_l2p_region
	.type	flush_l2p_region, %function
flush_l2p_region:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	ubfiz	x19, x0, 4, 16
	add	x20, x20, :lo12:.LANCHOR2
	add	x0, x20, 3760
	ldr	x1, [x20,1272]
	add	x2, x1, x19
	ldrh	w1, [x1,x19]
	ldr	x2, [x2,8]
	bl	FtlMapWritePage
	ldr	x0, [x20,1272]
	add	x19, x0, x19
	ldr	w0, [x19,4]
	and	w0, w0, 2147483647
	str	w0, [x19,4]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	flush_l2p_region, .-flush_l2p_region
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	adrp	x0, .LANCHOR2
	mov	x4, 0
	add	x1, x0, :lo12:.LANCHOR2
	mov	x3, x0
	mov	w5, 65535
	ldrh	w2, [x1,862]
	ldr	x1, [x1,1272]
.L1738:
	uxth	w0, w4
	cmp	w0, w2
	bcs	.L1754
	add	x4, x4, 1
	add	x6, x1, x4, lsl 4
	ldrh	w6, [x6,-16]
	cmp	w6, w5
	bne	.L1738
	b	.L1739
.L1754:
	mov	w0, w2
	mov	x4, 0
	mov	w7, -2147483648
.L1741:
	uxth	w6, w4
	cmp	w6, w2
	bcs	.L1755
	add	x5, x1, x4, lsl 4
	ldr	w5, [x5,4]
	tbnz	w5, #31, .L1750
	cmp	w5, w7
	bcc	.L1742
.L1750:
	mov	w5, w7
	mov	w6, w0
.L1742:
	add	x4, x4, 1
	mov	w7, w5
	mov	w0, w6
	b	.L1741
.L1755:
	cmp	w0, w2
	bcc	.L1739
	add	x0, x3, :lo12:.LANCHOR2
	add	x1, x1, 4
	mov	w4, -1
	mov	w3, 0
	ldrh	w5, [x0,3824]
	mov	w0, w2
.L1745:
	cmp	w3, w2
	beq	.L1739
	ldr	w7, [x1]
	cmp	w7, w4
	bcs	.L1746
	ldrh	w6, [x1,-4]
	cmp	w6, w5
	csel	w4, w4, w7, eq
	cmp	w6, w5
	csel	w0, w0, w3, eq
.L1746:
	add	w3, w3, 1
	add	x1, x1, 16
	uxth	w3, w3
	b	.L1745
.L1739:
	ret
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	log2phys
	.type	log2phys, %function
log2phys:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	add	x4, x21, :lo12:.LANCHOR2
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	ldr	w5, [x4,1404]
	ldrh	w3, [x4,832]
	cmp	w0, w5
	bcs	.L1757
	add	w27, w3, 7
	str	x1, [x29,104]
	lsr	w28, w0, w27
	mov	w25, w0
	str	w2, [x29,100]
	mov	x0, 0
	uxth	w22, w28
	ldrh	w1, [x4,862]
	ldr	x19, [x4,1272]
	b	.L1758
.L1757:
	mov	w0, -1
	cbnz	w2, .L1759
	str	w0, [x1]
	b	.L1759
.L1763:
	add	x0, x0, 1
	add	x2, x19, x0, lsl 4
	ldrh	w2, [x2,-16]
	cmp	w2, w22
	beq	.L1760
.L1758:
	uxth	x20, w0
	cmp	w20, w1
	bcc	.L1763
	bl	select_l2p_ram_region
	uxth	x20, w0
	ubfiz	x26, x20, 4, 16
	mov	w2, 65535
	add	x1, x19, x26
	ldrh	w3, [x19,x26]
	cmp	w3, w2
	bne	.L1771
.L1764:
	add	x19, x21, :lo12:.LANCHOR2
	ubfiz	x0, x22, 2, 16
	ldr	x1, [x19,1248]
	ldr	w24, [x1,x0]
	cbnz	w24, .L1765
	ldr	x0, [x19,1272]
	mov	w1, 255
	ldrh	w2, [x19,834]
	add	x0, x0, x26
	ldr	x0, [x0,8]
	bl	ftl_memset
	ldr	x0, [x19,1272]
	strh	w22, [x0,x26]
	ldr	x0, [x19,1272]
	add	x26, x0, x26
	str	w24, [x26,4]
.L1760:
	mov	x1, 1
	lsl	x3, x1, x27
	ldr	w1, [x29,100]
	sub	w3, w3, #1
	and	w0, w25, w3
	uxth	x0, w0
	cbnz	w1, .L1761
	add	x1, x21, :lo12:.LANCHOR2
	ldr	x1, [x1,1272]
	add	x1, x1, x20, lsl 4
	ldr	x1, [x1,8]
	ldr	w0, [x1,x0,lsl 2]
	ldr	x1, [x29,104]
	str	w0, [x1]
	b	.L1762
.L1761:
	ldr	x2, [x29,104]
	lsl	x1, x20, 4
	ldr	w4, [x2]
	add	x2, x21, :lo12:.LANCHOR2
	ldr	x3, [x2,1272]
	add	x3, x3, x1
	ldr	x3, [x3,8]
	str	w4, [x3,x0,lsl 2]
	strh	w22, [x2,3824]
	ldr	x0, [x2,1272]
	add	x0, x0, x1
	ldr	w1, [x0,4]
	orr	w1, w1, -2147483648
	str	w1, [x0,4]
.L1762:
	add	x21, x21, :lo12:.LANCHOR2
	ldr	x0, [x21,1272]
	add	x20, x0, x20, lsl 4
	mov	w0, 0
	ldr	w1, [x20,4]
	cmn	w1, #1
	beq	.L1759
	add	w1, w1, 1
	str	w1, [x20,4]
	b	.L1759
.L1771:
	ldr	w1, [x1,4]
	tbz	w1, #31, .L1764
	bl	flush_l2p_region
	b	.L1764
.L1765:
	ldr	x0, [x19,1272]
	mov	w1, 1
	add	x23, x19, 1416
	mov	w2, w1
	add	x0, x0, x26
	str	w24, [x19,1420]
	ldr	x0, [x0,8]
	str	x0, [x19,1424]
	ldr	x0, [x19,1120]
	str	x0, [x19,1432]
	mov	x0, x23
	bl	FlashReadPages
	ldr	x0, [x19,1432]
	ldrh	w0, [x0,8]
	cmp	w0, w22
	beq	.L1766
	adrp	x0, .LC40
	uxth	w1, w28
	mov	w2, w24
	add	x0, x0, :lo12:.LC40
	bl	printk
	mov	w2, 4
	ldr	x1, [x19,1432]
	adrp	x0, .LC17
	mov	w3, w2
	add	x0, x0, :lo12:.LC17
	bl	rknand_print_hex
	adrp	x0, .LC41
	ldr	x1, [x19,1248]
	ldrh	w3, [x19,860]
	add	x0, x0, :lo12:.LC41
	mov	w2, 4
	bl	rknand_print_hex
	mov	w0, 1
	str	w0, [x19,1408]
	b	.L1767
.L1766:
	ldr	w0, [x19,1416]
	cmp	w0, 256
	bne	.L1767
	and	w28, w28, 65535
	adrp	x0, .LC42
	mov	w1, w28
	mov	w2, w24
	add	x0, x0, :lo12:.LC42
	bl	printk
	ldr	x2, [x19,1272]
	add	x0, x19, 3760
	mov	w1, w28
	add	x2, x2, x26
	ldr	x2, [x2,8]
	bl	FtlMapWritePage
.L1767:
	add	x1, x21, :lo12:.LANCHOR2
	ldr	x0, [x1,1272]
	add	x0, x0, x26
	str	wzr, [x0,4]
	ldr	x0, [x1,1272]
	strh	w22, [x0,x26]
	b	.L1760
.L1759:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	log2phys, .-log2phys
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	stp	x29, x30, [sp, -160]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	mov	w22, w1
	mov	w25, w0
	add	w1, w0, w1
	ldrh	w0, [x19,820]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	cmp	w1, w0
	mov	x24, x2
	ldrh	w23, [x19,832]
	mov	w21, -1
	bhi	.L1773
	lsr	w23, w25, w23
	mov	w21, 0
	add	x27, x19, 3832
.L1774:
	cbz	w22, .L1773
	ldrh	w1, [x19,830]
	ldr	x0, [x19,1240]
	udiv	w26, w25, w1
	ldr	w2, [x0,w23,uxtw 2]
	uxth	w0, w22
	msub	w26, w26, w1, w25
	sub	w20, w1, w26
	uxth	w20, w20
	cmp	w20, w22
	csel	w20, w0, w20, hi
	cmp	w20, w1
	beq	.L1776
	cbz	w2, .L1776
	ldr	x0, [x19,1072]
	mov	w1, 1
	str	w2, [x29,108]
	mov	w2, w1
	str	x0, [x29,112]
	add	x0, x29, 104
	str	xzr, [x29,120]
	bl	FlashReadPages
	b	.L1777
.L1776:
	ldr	x0, [x19,1072]
	mov	w1, 0
	ldrh	w2, [x19,834]
	bl	ftl_memset
.L1777:
	ldr	x0, [x19,1072]
	lsl	w28, w20, 9
	ubfiz	x26, x26, 9, 16
	mov	x1, x24
	mov	w2, w28
	add	x0, x0, x26
	sub	w22, w22, w20
	add	w25, w25, w20
	bl	ftl_memcpy
	add	x24, x24, x28, sxtw
	ldr	x2, [x19,1072]
	mov	w1, w23
	mov	x0, x27
	add	w23, w23, 1
	bl	FtlMapWritePage
	cmn	w0, #1
	csinv	w21, w21, wzr, ne
	b	.L1774
.L1773:
	mov	w0, w21
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 160
	ret
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	stp	x29, x30, [sp, -176]!
	adrp	x3, .LANCHOR2
	add	x3, x3, :lo12:.LANCHOR2
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	mov	w22, w1
	mov	x25, x2
	add	w2, w0, w1
	ldrh	w1, [x3,820]
	mov	w23, w0
	ldrh	w20, [x3,832]
	cmp	w2, w1
	mov	w0, -1
	bhi	.L1788
	adrp	x26, .LC43
	add	x0, x3, 3832
	lsr	w20, w23, w20
	mov	w21, 0
	mov	x28, x3
	add	x26, x26, :lo12:.LC43
	str	x0, [x29,104]
.L1789:
	cbz	w22, .L1797
	ldrh	w19, [x28,830]
	ldr	x0, [x28,1240]
	udiv	w24, w23, w19
	ldr	w4, [x0,w20,uxtw 2]
	uxth	w0, w22
	msub	w24, w24, w19, w23
	sub	w19, w19, w24
	uxth	w19, w19
	cmp	w19, w22
	csel	w19, w0, w19, hi
	lsl	w27, w19, 9
	cbz	w4, .L1791
	ldr	x0, [x28,1072]
	mov	w1, 1
	str	x0, [x29,128]
	mov	w2, w1
	add	x0, x29, 120
	str	w4, [x29,124]
	str	x4, [x29,96]
	str	xzr, [x29,136]
	bl	FlashReadPages
	ldr	w0, [x29,120]
	ldr	x4, [x29,96]
	cmn	w0, #1
	ldr	w0, [x28,1416]
	csinv	w21, w21, wzr, ne
	cmp	w0, 256
	bne	.L1793
	mov	w1, w20
	mov	w2, w4
	mov	x0, x26
	bl	printk
	ldr	x0, [x29,104]
	mov	w1, w20
	ldr	x2, [x28,1072]
	bl	FtlMapWritePage
.L1793:
	ldr	x1, [x28,1072]
	ubfiz	x24, x24, 9, 16
	mov	x0, x25
	mov	w2, w27
	add	x1, x1, x24
	bl	ftl_memcpy
	b	.L1794
.L1791:
	mov	x0, x25
	mov	w1, w4
	mov	w2, w27
	bl	ftl_memset
.L1794:
	add	w20, w20, 1
	sub	w22, w22, w19
	add	w23, w23, w19
	add	x25, x25, x27, sxtw
	b	.L1789
.L1797:
	mov	w0, w21
.L1788:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 176
	ret
	.size	FtlVendorPartRead, .-FtlVendorPartRead
	.align	2
	.global	ftl_vendor_read
	.type	ftl_vendor_read, %function
ftl_vendor_read:
	stp	x29, x30, [sp, -16]!
	add	w0, w0, 256
	add	x29, sp, 0
	bl	FtlVendorPartRead
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_vendor_read, .-ftl_vendor_read
	.align	2
	.global	ftl_sys_read
	.type	ftl_sys_read, %function
ftl_sys_read:
	stp	x29, x30, [sp, -16]!
	add	w0, w0, 512
	add	x29, sp, 0
	bl	FtlVendorPartRead
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_sys_read, .-ftl_sys_read
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	adrp	x1, .LANCHOR2
	add	x3, x1, :lo12:.LANCHOR2
	ldrh	w2, [x3,3896]
	cbnz	w0, .L1801
	cmp	w2, 4
	bhi	.L1801
	add	w2, w2, 1
	strh	w2, [x3,3896]
	b	.L1800
.L1801:
	add	x0, x1, :lo12:.LANCHOR2
	mov	w7, 65535
	strh	wzr, [x0,3896]
	str	wzr, [x0,3900]
	ldrh	w5, [x0,764]
	ldr	x6, [x0,1192]
	mov	x0, 0
.L1802:
	cmp	w5, w0, uxth
	bls	.L1800
	ldrh	w4, [x6,x0,lsl 1]
	cmp	w4, w7
	beq	.L1803
	add	x3, x1, :lo12:.LANCHOR2
	ldr	w2, [x3,3900]
	add	w2, w4, w2
	str	w2, [x3,3900]
.L1803:
	add	x0, x0, 1
	b	.L1802
.L1800:
	ret
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	FtlMapBlkWriteDumpData
	.type	FtlMapBlkWriteDumpData, %function
FtlMapBlkWriteDumpData:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	ldr	w1, [x0,56]
	ldrh	w19, [x0,6]
	ldr	x4, [x0,40]
	cbz	w1, .L1806
	adrp	x1, .LANCHOR2
	str	wzr, [x0,56]
	add	x2, x1, :lo12:.LANCHOR2
	mov	x21, x1
	ldr	w3, [x2,1408]
	cbnz	w3, .L1806
	mov	x20, x0
	sub	w19, w19, #1
	add	x0, x2, 1416
	ldr	x3, [x2,1120]
	uxth	w19, w19
	ldr	x1, [x2,1064]
	str	x3, [x0,16]
	ubfiz	x3, x19, 2, 16
	str	x1, [x0,8]
	ldr	w3, [x4,x3]
	str	w3, [x0,4]
	cbz	w3, .L1810
	mov	w1, 1
	mov	w2, w1
	bl	FlashReadPages
	b	.L1811
.L1810:
	ldrh	w2, [x2,834]
	mov	x0, x1
	mov	w1, 255
	bl	ftl_memset
.L1811:
	add	x2, x21, :lo12:.LANCHOR2
	mov	x0, x20
	mov	w1, w19
	ldr	x2, [x2,1424]
	bl	FtlMapWritePage
.L1806:
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR2
	str	x25, [sp,64]
	stp	x19, x20, [sp,16]
	add	x19, x22, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	ldr	w0, [x19,1408]
	cbnz	w0, .L1814
	ldr	x21, [x19,1120]
	add	x20, x19, 3528
	ldr	x1, [x19,1056]
	str	x1, [x19,1424]
	str	x21, [x19,1432]
	ldrh	w2, [x19,3904]
	strh	w2, [x21,2]
	mov	w2, -3932
	strh	w2, [x21]
	str	w0, [x21,12]
	ldr	w2, [x19,3912]
	str	w2, [x21,4]
	str	w0, [x21,8]
	mov	w0, 19539
	ldrh	w1, [x19,3616]
	movk	w0, 0x4654, lsl 16
	strh	w1, [x20,14]
	ldrh	w1, [x19,3618]
	ldrb	w2, [x19,3622]
	str	w0, [x19,3528]
	mov	w0, 83
	orr	w1, w2, w1, lsl 6
	strh	w1, [x20,16]
	ldrh	w1, [x19,3664]
	movk	w0, 0x5000, lsl 16
	strh	w1, [x20,18]
	ldrh	w1, [x19,3666]
	ldrb	w2, [x19,3670]
	str	w0, [x20,4]
	ldrh	w0, [x19,3910]
	orr	w1, w2, w1, lsl 6
	strh	w0, [x20,8]
	ldrh	w0, [x19,778]
	strb	w0, [x20,10]
	ldrb	w0, [x19,3624]
	strb	w0, [x20,11]
	strh	w1, [x20,20]
	ldrb	w0, [x19,3672]
	ldrh	w1, [x19,3712]
	strb	w0, [x20,12]
	strh	w1, [x20,22]
	ldrh	w1, [x19,3714]
	ldrb	w0, [x19,3720]
	strb	w0, [x20,13]
	ldr	w0, [x19,908]
	str	w0, [x20,32]
	ldr	w0, [x19,876]
	str	w0, [x20,40]
	ldr	w0, [x19,880]
	ldrb	w2, [x19,3718]
	str	w0, [x20,36]
	ldrh	w0, [x19,952]
	orr	w1, w2, w1, lsl 6
	strh	w0, [x20,44]
	ldrh	w0, [x19,954]
	strh	w0, [x20,46]
	ldrh	w2, [x19,834]
	ldr	x0, [x19,1424]
	strh	w1, [x20,24]
	mov	w1, 255
	bl	ftl_memset
	ldr	x0, [x19,1424]
	mov	x1, x20
	mov	w2, 48
	bl	ftl_memcpy
	ldr	x0, [x19,1424]
	ldrh	w2, [x19,764]
	ldr	x1, [x19,1192]
	add	x0, x0, 48
	lsl	w2, w2, 1
	bl	ftl_memcpy
	ldrh	w2, [x19,764]
	ldr	x1, [x19,1424]
	uxth	x0, w2
	lsr	w2, w2, 3
	add	x0, x0, 24
	add	w2, w2, 4
	lsr	x0, x0, 1
	add	x0, x1, x0, lsl 2
	adrp	x1, .LANCHOR0+16
	ldr	x1, [x1,#:lo12:.LANCHOR0+16]
	bl	ftl_memcpy
	ldrh	w0, [x19,864]
	cbz	w0, .L1815
	ldrh	w0, [x19,764]
	ldrh	w2, [x19,860]
	lsr	w1, w0, 3
	add	w0, w1, w0, lsl 1
	ldr	x1, [x19,1424]
	add	w0, w0, 52
	lsl	w2, w2, 2
	and	x0, x0, 65532
	add	x0, x1, x0
	ldr	x1, [x19,1248]
	bl	ftl_memcpy
.L1815:
	mov	w0, 0
	mov	w20, 0
	mov	w25, 65535
	add	x19, x22, :lo12:.LANCHOR2
	bl	FtlUpdateVaildLpn
.L1816:
	ldrh	w1, [x19,3904]
	ldr	x0, [x19,1056]
	ldrh	w2, [x19,3906]
	str	x0, [x19,1424]
	ldr	x0, [x19,1120]
	str	x0, [x19,1432]
	orr	w0, w2, w1, lsl 10
	str	w0, [x19,1420]
	ldrh	w0, [x19,826]
	sub	w0, w0, #1
	cmp	w2, w0
	blt	.L1817
	ldrh	w25, [x19,3908]
	strh	wzr, [x19,3906]
	strh	w1, [x19,3908]
	bl	FtlFreeSysBlkQueueOut
	ldr	w1, [x19,876]
	str	w1, [x19,3912]
	add	w2, w1, 1
	str	w2, [x19,876]
	ubfiz	w2, w0, 10, 16
	str	w2, [x19,1420]
	strh	w0, [x19,3904]
	str	w1, [x21,4]
	strh	w0, [x21,2]
.L1817:
	mov	w1, 1
	add	x0, x19, 1416
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
	ldrh	w1, [x19,3906]
	ldr	w2, [x19,1416]
	add	w1, w1, 1
	cmn	w2, #1
	uxth	w1, w1
	strh	w1, [x19,3906]
	bne	.L1818
	cmp	w1, 1
	bne	.L1819
	ldrh	w1, [x19,826]
	sub	w1, w1, #1
	strh	w1, [x19,3906]
.L1819:
	add	w20, w20, 1
	uxth	w20, w20
	cmp	w20, 3
	bls	.L1816
	add	x22, x22, :lo12:.LANCHOR2
	adrp	x0, .LC44
	add	x0, x0, :lo12:.LC44
	mov	w2, w20
	ldr	w1, [x22,1420]
	bl	printk
	mov	w0, 1
	str	w0, [x22,1408]
	b	.L1814
.L1818:
	cmp	w1, 1
	beq	.L1816
	cmp	w2, 256
	beq	.L1816
	mov	w0, 65535
	cmp	w25, w0
	beq	.L1814
	mov	w0, w25
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
.L1814:
	mov	w0, 0
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	stp	x29, x30, [sp, -128]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x20, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	strh	wzr, [x20,3920]
	mov	x26, 56
	ldr	w2, [x20,852]
	mov	w22, 65535
	ldr	x0, [x20,1264]
	lsl	w2, w2, 2
	strh	wzr, [x20,1400]
	bl	ftl_memset
	ldr	w2, [x20,852]
	mov	w1, 0
	ldr	x0, [x20,1200]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w2, [x20,844]
	mov	w1, 0
	ldr	x0, [x20,1232]
	lsl	w2, w2, 2
	bl	ftl_memset
	ldrh	w2, [x20,844]
	mov	w1, 0
	ldr	x0, [x20,1216]
	lsl	w2, w2, 1
	bl	ftl_memset
	add	x0, x20, 3904
	mov	w1, 255
	mov	w2, 16
	bl	ftl_memset
	ldrh	w21, [x20,764]
.L1826:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,766]
	cmp	w0, w21
	bls	.L1867
	mov	x27, 0
	ldrh	w3, [x1,756]
	ldrh	w2, [x1,836]
	mov	w20, w27
	mov	x23, x1
.L1868:
	cmp	w3, w27, uxth
	bls	.L1901
	add	x0, x23, 784
	mov	w1, w21
	str	x2, [x29,112]
	str	x3, [x29,120]
	ldrb	w0, [x0,x27]
	bl	V2P_block
	uxth	w28, w0
	bl	FtlBbmIsBadBlock
	ldr	x3, [x29,120]
	ldr	x2, [x29,112]
	cbnz	w0, .L1827
	mov	w0, 56
	ldr	x1, [x23,1000]
	lsl	w28, w28, 10
	mov	w4, 4
	umull	x0, w20, w0
	add	x1, x1, x0
	str	w28, [x1,4]
	ldr	x1, [x23,1000]
	add	x0, x1, x0
	ldr	x1, [x23,1080]
	str	x1, [x0,8]
	mul	w1, w20, w2
	add	w20, w20, 1
	sdiv	w1, w1, w4
	uxth	w20, w20
	ldr	x4, [x23,1136]
	add	x1, x4, x1, sxtw 2
	str	x1, [x0,16]
.L1827:
	add	x27, x27, 1
	b	.L1868
.L1901:
	cbz	w20, .L1830
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, w20
	mov	w2, 1
	mov	x23, 0
	mov	w27, -1
	adrp	x28, .LANCHOR0
	ldr	x0, [x0,1000]
	bl	FlashReadPages
.L1831:
	cmp	w20, w23, uxth
	bls	.L1830
	add	x0, x19, :lo12:.LANCHOR2
	mul	x5, x23, x26
	ldr	x0, [x0,1000]
	add	x1, x0, x5
	ldr	w0, [x0,x5]
	ldr	w4, [x1,4]
	cmn	w0, #1
	ldr	x25, [x1,16]
	ubfx	x24, x4, 10, 16
	bne	.L1833
	mov	w6, 16
.L1832:
	add	x7, x19, :lo12:.LANCHOR2
	str	x6, [x29,104]
	str	x7, [x29,112]
	str	x5, [x29,120]
	ldr	x0, [x7,1000]
	add	x0, x0, x5
	ldr	w1, [x0,4]
	add	w1, w1, 1
	str	w1, [x0,4]
	mov	w1, 1
	ldr	x0, [x7,1000]
	mov	w2, w1
	add	x0, x0, x5
	bl	FlashReadPages
	ldrh	w0, [x25]
	ldr	x7, [x29,112]
	cmp	w0, w22
	ldr	x5, [x29,120]
	ldr	x6, [x29,104]
	ldr	x0, [x7,1000]
	bne	.L1834
	str	w27, [x0,x5]
	b	.L1833
.L1834:
	ldr	w0, [x0,x5]
	cmn	w0, #1
	bne	.L1833
	sub	w6, w6, #1
	uxth	w6, w6
	cbnz	w6, .L1832
.L1833:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x1, [x0,1000]
	ldr	w1, [x1,x5]
	cmn	w1, #1
	beq	.L1836
	ldr	w0, [x0,876]
	cmn	w0, #1
	beq	.L1837
	ldr	w1, [x25,4]
	cmp	w0, w1
	bhi	.L1838
.L1837:
	ldr	w0, [x25,4]
	cmn	w0, #1
	beq	.L1838
	add	x1, x19, :lo12:.LANCHOR2
	add	w0, w0, 1
	str	w0, [x1,876]
.L1838:
	ldrh	w0, [x25]
	mov	w1, 61604
	cmp	w0, w1
	beq	.L1840
	bhi	.L1841
	mov	w1, 61574
	cmp	w0, w1
	bne	.L1839
	add	x5, x19, :lo12:.LANCHOR2
	ldrh	w6, [x5,844]
	ldrh	w0, [x5,1400]
	sub	w2, w6, #1
	ldr	x5, [x5,1232]
	sxth	x1, w2
	sub	w2, w2, w0
	b	.L1854
.L1841:
	mov	w1, 61634
	cmp	w0, w1
	beq	.L1843
	cmp	w0, w22
	bne	.L1839
	mov	w0, w24
	b	.L1900
.L1843:
	add	x5, x19, :lo12:.LANCHOR2
	ldr	w6, [x5,852]
	ldrh	w0, [x5,3920]
	ldr	x5, [x5,1264]
	uxth	w2, w6
	sub	w1, w2, #1
	sub	w2, w2, w0
	sub	w2, w2, #1
	sxth	x1, w1
	sxth	w2, w2
.L1845:
	cmp	w1, w2
	ble	.L1902
	sbfiz	x7, x1, 2, 32
	ldr	w10, [x25,4]
	sxth	x8, w1
	ldr	w9, [x5,x7]
	cmp	w10, w9
	bls	.L1846
	ldr	w2, [x5]
	cbnz	w2, .L1847
	cmp	w0, w6
	beq	.L1847
	add	x2, x19, :lo12:.LANCHOR2
	add	w0, w0, 1
	strh	w0, [x2,3920]
.L1847:
	mov	w0, 0
.L1848:
	cmp	w0, w1
	beq	.L1903
	add	x5, x19, :lo12:.LANCHOR2
	sxtw	x2, w0
	lsl	x6, x2, 2
	lsl	x2, x2, 1
	add	w0, w0, 1
	ldr	x9, [x5,1264]
	sxth	w0, w0
	add	x10, x9, x6
	ldr	w10, [x10,4]
	str	w10, [x9,x6]
	ldr	x5, [x5,1200]
	add	x6, x5, x2
	ldrh	w6, [x6,2]
	strh	w6, [x5,x2]
	b	.L1848
.L1903:
	add	x2, x19, :lo12:.LANCHOR2
	ldr	w5, [x25,4]
	ldr	x0, [x2,1264]
	str	w5, [x0,x7]
	ldr	x0, [x2,1200]
	strh	w24, [x0,x8,lsl 1]
	tbz	w1, #31, .L1850
	b	.L1839
.L1846:
	sub	w1, w1, #1
	sxth	x1, w1
	b	.L1845
.L1902:
	tbz	w1, #31, .L1883
	b	.L1839
.L1850:
	ldrh	w0, [x2,3920]
	ldr	w2, [x2,852]
	sub	w2, w2, w0
	sub	w2, w2, #1
	cmp	w1, w2, sxth
	bgt	.L1839
.L1883:
	add	x2, x19, :lo12:.LANCHOR2
	add	w0, w0, 1
	ldr	w3, [x25,4]
	strh	w0, [x2,3920]
	ldr	x0, [x2,1264]
	str	w3, [x0,x1,lsl 2]
	ldr	x0, [x2,1200]
	b	.L1899
.L1860:
	sbfiz	x7, x1, 2, 32
	ldr	w10, [x25,4]
	sxth	x8, w1
	ldr	w9, [x5,x7]
	cmp	w10, w9
	bhi	.L1904
	sub	w1, w1, #1
	sxth	x1, w1
.L1854:
	cmp	w1, w2
	bgt	.L1860
	b	.L1859
.L1904:
	ldr	w2, [x5]
	cbnz	w2, .L1856
	cmp	w0, w6
	beq	.L1856
	add	x2, x19, :lo12:.LANCHOR2
	add	w0, w0, 1
	strh	w0, [x2,1400]
.L1856:
	mov	w0, 0
.L1857:
	cmp	w0, w1
	beq	.L1905
	add	x5, x19, :lo12:.LANCHOR2
	sxtw	x2, w0
	lsl	x6, x2, 2
	lsl	x2, x2, 1
	add	w0, w0, 1
	ldr	x9, [x5,1232]
	sxth	w0, w0
	add	x10, x9, x6
	ldr	w10, [x10,4]
	str	w10, [x9,x6]
	ldr	x5, [x5,1216]
	add	x6, x5, x2
	ldrh	w6, [x6,2]
	strh	w6, [x5,x2]
	b	.L1857
.L1905:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w5, [x25,4]
	ldr	x2, [x0,1232]
	str	w5, [x2,x7]
	ldr	x0, [x0,1216]
	strh	w24, [x0,x8,lsl 1]
.L1859:
	tbnz	w1, #31, .L1839
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w2, [x0,844]
	ldrh	w5, [x0,1400]
	sub	w2, w2, #1
	sub	w2, w2, w5
	cmp	w1, w2, sxth
	bgt	.L1839
	ldr	x2, [x0,1232]
	add	w5, w5, 1
	ldr	w3, [x25,4]
	strh	w5, [x0,1400]
	str	w3, [x2,x1,lsl 2]
	ldr	x0, [x0,1216]
.L1899:
	strh	w24, [x0,x1,lsl 1]
	b	.L1839
.L1840:
	add	x0, x19, :lo12:.LANCHOR2
	add	x1, x0, 3904
	ldrh	w2, [x0,3904]
	cmp	w2, w22
	bne	.L1861
	strh	w24, [x0,3904]
	ldr	w0, [x25,4]
	str	w0, [x1,8]
	b	.L1839
.L1861:
	ldrh	w0, [x1,4]
	cmp	w0, w22
	beq	.L1862
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
.L1862:
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w2, [x25,4]
	add	x0, x1, 3904
	ldr	w5, [x0,8]
	cmp	w5, w2
	bcs	.L1863
	ldrh	w2, [x1,3904]
	strh	w2, [x0,4]
	strh	w24, [x1,3904]
	ldr	w1, [x25,4]
	str	w1, [x0,8]
	b	.L1839
.L1863:
	strh	w24, [x0,4]
	b	.L1839
.L1836:
	add	x0, x28, :lo12:.LANCHOR0
	ldrb	w1, [x0,4044]
	mov	w0, w24
	cbz	w1, .L1864
.L1900:
	mov	w1, 0
.L1864:
	bl	FtlFreeSysBlkQueueIn
.L1839:
	add	x23, x23, 1
	b	.L1831
.L1830:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1826
.L1867:
	ldr	x2, [x1,1200]
	ldrh	w0, [x2]
	cbz	w0, .L1869
.L1872:
	add	x1, x19, :lo12:.LANCHOR2
	ldr	x2, [x1,1216]
	ldrh	w0, [x2]
	cbz	w0, .L1870
	b	.L1893
.L1869:
	ldrh	w3, [x1,3920]
	cbz	w3, .L1872
	ldr	w1, [x1,852]
.L1873:
	cmp	w0, w1
	bcs	.L1872
	ldrh	w3, [x2,w0,sxtw 1]
	cbz	w3, .L1874
	mov	w6, w0
.L1875:
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w2, [x1,852]
	cmp	w0, w2
	bcs	.L1872
	sxtw	x5, w0
	sub	w2, w0, w6
	ldr	x4, [x1,1200]
	lsl	x3, x5, 1
	sxtw	x2, w2
	add	w0, w0, 1
	ldrh	w7, [x4,x3]
	sxth	w0, w0
	strh	w7, [x4,x2,lsl 1]
	ldr	x4, [x1,1264]
	ldr	w5, [x4,x5,lsl 2]
	str	w5, [x4,x2,lsl 2]
	ldr	x1, [x1,1200]
	strh	wzr, [x1,x3]
	b	.L1875
.L1874:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L1873
.L1870:
	ldrh	w3, [x1,1400]
	cbz	w3, .L1893
	ldrh	w1, [x1,844]
.L1878:
	cmp	w0, w1
	mov	w6, w0
	bge	.L1893
	ldrh	w3, [x2,w0,sxtw 1]
	cbz	w3, .L1879
.L1880:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w2, [x1,844]
	cmp	w0, w2
	bge	.L1893
	sxtw	x5, w0
	sub	w2, w0, w6
	ldr	x4, [x1,1216]
	lsl	x3, x5, 1
	sxtw	x2, w2
	add	w0, w0, 1
	ldrh	w7, [x4,x3]
	sxth	w0, w0
	strh	w7, [x4,x2,lsl 1]
	ldr	x4, [x1,1232]
	ldr	w5, [x4,x5,lsl 2]
	str	w5, [x4,x2,lsl 2]
	ldr	x1, [x1,1216]
	strh	wzr, [x1,x3]
	b	.L1880
.L1879:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L1878
.L1893:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	FtlScanSysBlk, .-FtlScanSysBlk
	.align	2
	.global	FtlLoadEctTbl
	.type	FtlLoadEctTbl, %function
FtlLoadEctTbl:
	stp	x29, x30, [sp, -32]!
	mov	w0, 64
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w1, [x19,1152]
	ldr	x2, [x19,1168]
	bl	FtlVendorPartRead
	ldr	x0, [x19,1168]
	ldr	w1, [x0]
	mov	w0, 17221
	movk	w0, 0x4254, lsl 16
	cmp	w1, w0
	beq	.L1907
	adrp	x0, .LC45
	adrp	x1, .LC46
	add	x1, x1, :lo12:.LC46
	add	x0, x0, :lo12:.LC45
	bl	printk
	ldrh	w2, [x19,1152]
	mov	w1, 0
	ldr	x0, [x19,1168]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1907:
	mov	w0, 0
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlLoadEctTbl, .-FtlLoadEctTbl
	.align	2
	.global	ftl_set_blk_mode
	.type	ftl_set_blk_mode, %function
ftl_set_blk_mode:
	stp	x29, x30, [sp, -16]!
	uxth	w0, w0
	add	x29, sp, 0
	cbz	w1, .L1909
	bl	ftl_set_blk_mode.part.17
	b	.L1908
.L1909:
	adrp	x1, .LANCHOR0+16
	ubfx	x2, x0, 5, 11
	lsl	x2, x2, 2
	ldr	x3, [x1,#:lo12:.LANCHOR0+16]
	mov	w1, 1
	lsl	w0, w1, w0
	ldr	w1, [x3,x2]
	bic	w0, w1, w0
	str	w0, [x3,x2]
.L1908:
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_set_blk_mode, .-ftl_set_blk_mode
	.align	2
	.global	ftl_get_blk_mode
	.type	ftl_get_blk_mode, %function
ftl_get_blk_mode:
	adrp	x1, .LANCHOR0+16
	uxth	w0, w0
	ubfx	x2, x0, 5, 11
	ldr	x1, [x1,#:lo12:.LANCHOR0+16]
	ldr	w1, [x1,x2,lsl 2]
	lsr	w0, w1, w0
	and	w0, w0, 1
	ret
	.size	ftl_get_blk_mode, .-ftl_get_blk_mode
	.align	2
	.global	FtlCheckVpc
	.type	FtlCheckVpc, %function
FtlCheckVpc:
	stp	x29, x30, [sp, -96]!
	adrp	x1, .LANCHOR3
	adrp	x0, .LC47
	add	x1, x1, :lo12:.LANCHOR3
	add	x29, sp, 0
	add	x1, x1, 16
	add	x0, x0, :lo12:.LC47
	str	x25, [sp,64]
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	mov	w20, 0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	add	x19, x19, :lo12:.LANCHOR2
	bl	printk
	add	x0, x19, 3928
	mov	w1, 0
	mov	x2, 8192
	bl	memset
.L1913:
	ldr	w0, [x19,1404]
	cmp	w20, w0
	bcs	.L1930
	mov	w0, w20
	add	x1, x29, 92
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x29,92]
	cmn	w0, #1
	beq	.L1914
	lsr	w0, w0, 10
	bl	P2V_block_in_plane
	uxth	w0, w0
	add	x2, x19, 3928
	ldrh	w1, [x2,w0,sxtw 1]
	add	w1, w1, 1
	strh	w1, [x2,w0,sxtw 1]
.L1914:
	add	w20, w20, 1
	b	.L1913
.L1930:
	ldr	x1, [x19,1192]
	adrp	x0, .LC48
	ldrh	w3, [x19,764]
	add	x0, x0, :lo12:.LC48
	mov	w2, 2
	adrp	x21, .LC50
	mov	w20, 0
	add	x21, x21, :lo12:.LC50
	bl	rknand_print_hex
	adrp	x0, .LC49
	ldrh	w3, [x19,764]
	add	x0, x0, :lo12:.LC49
	add	x1, x19, 3928
	mov	w2, 2
	bl	rknand_print_hex
.L1916:
	ldrh	w0, [x19,764]
	cmp	w0, w20
	bls	.L1931
	ubfiz	x0, x20, 1, 16
	ldr	x1, [x19,1192]
	ldrh	w2, [x1,x0]
	add	x0, x19, 3928
	ldrh	w3, [x0,w20,sxtw 1]
	cmp	w2, w3
	beq	.L1917
	mov	x0, x21
	mov	w1, w20
	bl	printk
.L1917:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L1916
.L1931:
	ldr	x0, [x19,3600]
	cbz	x0, .L1912
	ldr	x20, [x19,1288]
	adrp	x22, .LC51
	ldrh	w23, [x19,3608]
	mov	w21, 0
	sub	x20, x0, x20
	mov	x0, -6148914691236517206
	asr	x20, x20, 1
	add	x22, x22, :lo12:.LC51
	mov	w24, 6
	mov	w25, 65535
	madd	x20, x0, x20, x20
	uxth	w20, w20
.L1920:
	cmp	w21, w23
	beq	.L1912
	ubfiz	x0, x20, 1, 16
	ldr	x1, [x19,1192]
	ldrh	w2, [x1,x0]
	cbz	w2, .L1921
	add	x3, x19, 3928
	mov	x0, x22
	mov	w1, w20
	ldrh	w3, [x3,w20,sxtw 1]
	bl	printk
.L1921:
	umull	x20, w20, w24
	ldr	x0, [x19,1288]
	ldrh	w20, [x0,x20]
	cmp	w20, w25
	beq	.L1912
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1920
.L1912:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlCheckVpc, .-FtlCheckVpc
	.align	2
	.global	FtlDumpSysBlock
	.type	FtlDumpSysBlock, %function
FtlDumpSysBlock:
	sub	sp, sp, #96
	adrp	x2, .LANCHOR2
	add	x2, x2, :lo12:.LANCHOR2
	stp	x29, x30, [sp,16]
	add	x29, sp, 16
	stp	x19, x20, [sp,32]
	stp	x21, x22, [sp,48]
	stp	x23, x24, [sp,64]
	str	x25, [sp,80]
	add	x19, x2, 1416
	uxth	w23, w0
	adrp	x22, .LC52
	ldr	x0, [x2,1056]
	adrp	x25, .LC53
	str	x0, [x19,8]
	lsl	w24, w23, 10
	ldr	x0, [x2,1120]
	mov	w20, 0
	str	x0, [x19,16]
	mov	x21, x2
	add	x22, x22, :lo12:.LC52
	add	x25, x25, :lo12:.LC53
.L1933:
	ldrh	w0, [x21,826]
	cmp	w20, w0
	bge	.L1939
	mov	w1, 1
	orr	w0, w20, w24
	mov	w2, w1
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
	ldr	x0, [x19,8]
	mov	w1, w23
	ldr	x7, [x19,16]
	ldr	w0, [x0]
	str	w0, [sp]
	mov	x0, x22
	ldr	w4, [x7]
	ldr	w5, [x7,4]
	ldr	w6, [x7,8]
	ldr	w2, [x19]
	ldr	w3, [x19,4]
	ldr	w7, [x7,12]
	bl	printk
	ldr	x0, [x19,16]
	ldr	w0, [x0]
	cmn	w0, #1
	beq	.L1934
	ldr	x1, [x21,1056]
	mov	x0, x25
	mov	w2, 4
	mov	w3, 768
	bl	rknand_print_hex
.L1934:
	add	w20, w20, 1
	sxth	w20, w20
	b	.L1933
.L1939:
	sub	sp, x29, #16
	ldp	x19, x20, [sp,32]
	ldp	x21, x22, [sp,48]
	ldp	x23, x24, [sp,64]
	ldr	x25, [sp,80]
	ldp	x29, x30, [sp,16]
	add	sp, sp, 96
	ret
	.size	FtlDumpSysBlock, .-FtlDumpSysBlock
	.align	2
	.global	Ftlscanalldata
	.type	Ftlscanalldata, %function
Ftlscanalldata:
	sub	sp, sp, #96
	adrp	x0, .LC54
	add	x0, x0, :lo12:.LC54
	mov	w1, 0
	stp	x29, x30, [sp,16]
	add	x29, sp, 16
	stp	x23, x24, [sp,64]
	adrp	x23, .LANCHOR2
	stp	x21, x22, [sp,48]
	adrp	x21, .LC55
	add	x22, x23, :lo12:.LANCHOR2
	stp	x19, x20, [sp,32]
	add	x21, x21, :lo12:.LC55
	mov	w20, 0
	add	x19, x22, 1416
	bl	printk
.L1941:
	add	x0, x23, :lo12:.LANCHOR2
	ldr	w0, [x0,1404]
	cmp	w20, w0
	bcs	.L1948
	mov	w0, w20
	add	x1, x29, 76
	mov	w2, 0
	bl	log2phys
	and	w0, w20, 2047
	cbnz	w0, .L1942
	ldr	w2, [x29,76]
	mov	x0, x21
	mov	w1, w20
	bl	printk
.L1942:
	ldr	w0, [x29,76]
	cmn	w0, #1
	beq	.L1944
	str	w0, [x19,4]
	mov	w1, 1
	ldr	x0, [x22,1056]
	mov	w2, 0
	str	x0, [x19,8]
	mov	x0, x19
	ldr	x24, [x22,1120]
	str	w20, [x19,24]
	str	x24, [x19,16]
	str	wzr, [x19]
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L1945
	cmp	w0, 256
	beq	.L1945
	ldr	w0, [x24,8]
	cmp	w0, w20
	beq	.L1944
.L1945:
	ldr	x7, [x19,8]
	mov	w1, w20
	ldr	x6, [x19,16]
	ldr	w0, [x7,4]
	str	w0, [sp]
	adrp	x0, .LC56
	add	x0, x0, :lo12:.LC56
	ldr	w3, [x6]
	ldr	w4, [x6,4]
	ldr	w5, [x6,8]
	ldr	w2, [x19,4]
	ldr	w6, [x6,12]
	ldr	w7, [x7]
	bl	printk
.L1944:
	add	w20, w20, 1
	b	.L1941
.L1948:
	sub	sp, x29, #16
	ldp	x19, x20, [sp,32]
	ldp	x21, x22, [sp,48]
	ldp	x23, x24, [sp,64]
	ldp	x29, x30, [sp,16]
	add	sp, sp, 96
	ret
	.size	Ftlscanalldata, .-Ftlscanalldata
	.align	2
	.global	dump_map_info
	.type	dump_map_info, %function
dump_map_info:
	sub	sp, sp, #128
	stp	x29, x30, [sp,16]
	add	x29, sp, 16
	stp	x21, x22, [sp,48]
	adrp	x22, .LANCHOR2
	add	x0, x22, :lo12:.LANCHOR2
	stp	x25, x26, [sp,80]
	stp	x23, x24, [sp,64]
	stp	x27, x28, [sp,96]
	stp	x19, x20, [sp,32]
	adrp	x24, .LC57
	mov	w25, 56
	ldrh	w21, [x0,764]
	mov	w26, 4
	mov	x27, 56
	add	x24, x24, :lo12:.LC57
.L1950:
	add	x19, x22, :lo12:.LANCHOR2
	ldrh	w0, [x19,766]
	cmp	w0, w21
	bls	.L1963
	mov	x23, 0
	ldrh	w3, [x19,756]
	ldrh	w2, [x19,836]
	mov	w20, w23
.L1959:
	cmp	w3, w23, uxth
	bls	.L1964
	add	x0, x19, 784
	mov	w1, w21
	str	x2, [x29,96]
	str	x3, [x29,104]
	ldrb	w0, [x0,x23]
	bl	V2P_block
	uxth	w28, w0
	bl	FtlBbmIsBadBlock
	ldr	x3, [x29,104]
	ldr	x2, [x29,96]
	cbnz	w0, .L1951
	umull	x0, w20, w25
	ldr	x1, [x19,1000]
	lsl	w28, w28, 10
	add	x1, x1, x0
	str	w28, [x1,4]
	ldr	x1, [x19,1000]
	ldr	x4, [x19,1136]
	add	x0, x1, x0
	ldr	x1, [x19,1080]
	str	x1, [x0,8]
	mul	w1, w20, w2
	add	w20, w20, 1
	sdiv	w1, w1, w26
	uxth	w20, w20
	add	x1, x4, x1, sxtw 2
	str	x1, [x0,16]
.L1951:
	add	x23, x23, 1
	b	.L1959
.L1964:
	cbz	w20, .L1954
	add	x0, x22, :lo12:.LANCHOR2
	mov	w1, w20
	mov	w2, 1
	mov	x19, 0
	ldr	x0, [x0,1000]
	bl	FlashReadPages
.L1955:
	cmp	w20, w19, uxth
	bls	.L1954
	add	x0, x22, :lo12:.LANCHOR2
	ldr	x0, [x0,1000]
	madd	x0, x19, x27, x0
	add	x19, x19, 1
	ldr	x7, [x0,8]
	ldr	x6, [x0,16]
	ldr	w2, [x0,4]
	ldr	w0, [x7,4]
	str	w0, [sp]
	ubfx	x1, x2, 10, 16
	mov	x0, x24
	ldr	w3, [x6]
	ldr	w4, [x6,4]
	ldr	w5, [x6,8]
	ldr	w7, [x7]
	ldr	w6, [x6,12]
	bl	printk
	b	.L1955
.L1954:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1950
.L1963:
	adrp	x23, .LC58
	mov	w21, 0
	add	x22, x19, 1416
	add	x23, x23, :lo12:.LC58
.L1958:
	ldrh	w0, [x19,3920]
	cmp	w21, w0
	bge	.L1961
	sbfiz	x24, x21, 1, 32
	mov	w20, 0
.L1962:
	ldrh	w0, [x19,826]
	cmp	w20, w0
	bge	.L1965
	ldr	x0, [x19,1200]
	mov	w1, 1
	mov	w2, w1
	ldrh	w0, [x0,x24]
	orr	w0, w20, w0, lsl 10
	str	w0, [x22,4]
	mov	x0, x22
	add	w20, w20, 1
	bl	FlashReadPages
	sxth	w20, w20
	ldr	x0, [x22,8]
	ldr	x1, [x19,1200]
	ldr	x7, [x22,16]
	ldr	w2, [x0]
	ldrh	w1, [x1,x24]
	str	w2, [sp]
	ldr	w0, [x0,4]
	str	w0, [sp,8]
	mov	x0, x23
	ldr	w4, [x7]
	ldr	w5, [x7,4]
	ldr	w6, [x7,8]
	ldr	w2, [x22]
	ldr	w3, [x22,4]
	ldr	w7, [x7,12]
	bl	printk
	b	.L1962
.L1965:
	add	w21, w21, 1
	sxth	w21, w21
	b	.L1958
.L1961:
	ldr	x1, [x19,1200]
	adrp	x0, .LC59
	ldr	w3, [x19,852]
	add	x0, x0, :lo12:.LC59
	mov	w2, 2
	bl	rknand_print_hex
	ldr	x1, [x19,1248]
	adrp	x0, .LC60
	ldrh	w3, [x19,860]
	add	x0, x0, :lo12:.LC60
	mov	w2, 4
	bl	rknand_print_hex
	ldr	x1, [x19,1256]
	adrp	x0, .LC61
	ldrh	w3, [x19,860]
	add	x0, x0, :lo12:.LC61
	mov	w2, 4
	bl	rknand_print_hex
	sub	sp, x29, #16
	ldp	x19, x20, [sp,32]
	ldp	x21, x22, [sp,48]
	ldp	x23, x24, [sp,64]
	ldp	x25, x26, [sp,80]
	ldp	x27, x28, [sp,96]
	ldp	x29, x30, [sp,16]
	add	sp, sp, 128
	ret
	.size	dump_map_info, .-dump_map_info
	.align	2
	.global	FtlMapTblRecovery
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	stp	x29, x30, [sp, -144]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x23, x24, [sp,48]
	mov	x27, x0
	adrp	x20, .LANCHOR2
	ldr	x22, [x0,40]
	mov	w19, 0
	ldrh	w24, [x0,6]
	ldr	x23, [x0,16]
	ldr	x26, [x0,24]
	lsl	w2, w24, 2
	ldrh	w0, [x0,8]
	str	w0, [x29,136]
	mov	x0, x22
	bl	ftl_memset
	str	x20, [x29,128]
	add	x6, x20, :lo12:.LANCHOR2
	str	wzr, [x27,52]
	add	x4, x6, 1416
	str	wzr, [x27,48]
	mov	x20, x6
	mov	x25, x4
	ldr	x0, [x6,1056]
	ldr	x21, [x6,1120]
	str	x0, [x6,1424]
	mov	w0, -1
	str	x21, [x6,1432]
	strh	w0, [x27]
	strh	w0, [x27,2]
	mov	w0, 1
	str	w0, [x27,56]
	ldr	w0, [x29,136]
	sub	w0, w0, #1
	str	w0, [x29,116]
.L1967:
	ldr	w0, [x29,136]
	cmp	w19, w0
	bge	.L1985
	ldr	w0, [x29,116]
	sxtw	x28, w19
	cmp	w19, w0
	bne	.L1968
	lsl	x0, x28, 1
	mov	w1, 1
	add	x25, x23, x0
	mov	w20, 0
	ldrh	w0, [x23,x0]
	bl	FtlGetLastWrittenPage
	strh	w19, [x27]
	sxth	w1, w0
	add	w0, w0, 1
	strh	w0, [x27,2]
	add	w19, w1, 1
	ldr	w0, [x26,x28,lsl 2]
	adrp	x26, .LANCHOR0
	str	w0, [x27,48]
	ldr	x0, [x29,128]
	add	x23, x0, :lo12:.LANCHOR2
	add	x28, x23, 1416
.L1969:
	cmp	w20, w19
	bge	.L1985
	ldrh	w0, [x25]
	mov	w1, 1
	mov	w2, w1
	orr	w0, w20, w0, lsl 10
	str	w0, [x28,4]
	mov	x0, x28
	bl	FlashReadPages
	add	x0, x26, :lo12:.LANCHOR0
	ldrb	w0, [x0,24]
	cbz	w0, .L1970
	ldr	x0, [x28,16]
	ldr	w2, [x0,12]
	cbz	w2, .L1970
	ldr	x0, [x28,8]
	ldrh	w1, [x23,834]
	str	x2, [x29,136]
	bl	js_hash
	ldr	x2, [x29,136]
	cmp	w2, w0
	beq	.L1970
	mov	w0, -1
	str	w0, [x28]
.L1970:
	ldr	w0, [x28]
	cmn	w0, #1
	beq	.L1971
	ldrh	w0, [x21,8]
	cmp	w0, w24
	bcs	.L1971
	ldrh	w2, [x21]
	ldrh	w1, [x27,4]
	cmp	w2, w1
	bne	.L1971
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x28,4]
	str	w1, [x22,x0]
.L1971:
	add	w4, w20, 1
	sxth	w20, w4
	b	.L1969
.L1985:
	mov	x0, x27
	bl	ftl_free_no_use_map_blk
	ldr	x0, [x29,128]
	ldrh	w1, [x27,2]
	add	x20, x0, :lo12:.LANCHOR2
	ldrh	w0, [x20,826]
	cmp	w1, w0
	bne	.L1974
	mov	x0, x27
	bl	ftl_map_blk_alloc_new_blk
	b	.L1974
.L1968:
	ldr	x0, [x20,1056]
	str	x0, [x25,8]
	lsl	x0, x28, 1
	add	x1, x23, x0
	str	x1, [x29,120]
	ldrh	w1, [x23,x0]
	ldrh	w0, [x20,826]
	sub	w0, w0, #1
	orr	w0, w0, w1, lsl 10
	mov	w1, 1
	str	w0, [x25,4]
	mov	w2, w1
	mov	x0, x25
	bl	FlashReadPages
	ldr	w0, [x25]
	cmn	w0, #1
	beq	.L1986
	ldrh	w1, [x21]
	mov	w28, 0
	ldrh	w0, [x27,4]
	cmp	w1, w0
	bne	.L2007
	ldrh	w1, [x21,8]
	mov	w0, 64245
	cmp	w1, w0
	bne	.L2007
	mov	w5, w28
.L1976:
	ldrh	w0, [x20,826]
	sub	w0, w0, #1
	cmp	w5, w0
	bge	.L1979
	ldr	x2, [x20,1056]
	sbfiz	x1, x5, 3, 32
	ldrh	w0, [x2,x1]
	cmp	w0, w24
	bcs	.L1977
	add	x1, x2, x1
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x1,4]
	str	w1, [x22,x0]
.L1977:
	add	w5, w5, 1
	sxth	w5, w5
	b	.L1976
.L1986:
	mov	w28, 0
.L2007:
	ldrh	w0, [x20,826]
	cmp	w28, w0
	bge	.L1979
	ldr	x0, [x29,120]
	mov	w1, 1
	mov	w2, w1
	ldrh	w0, [x0]
	orr	w0, w28, w0, lsl 10
	str	w0, [x25,4]
	mov	x0, x25
	bl	FlashReadPages
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,24]
	cbz	w0, .L1980
	ldr	x0, [x25,16]
	ldr	w2, [x0,12]
	cbz	w2, .L1980
	ldr	x0, [x25,8]
	ldrh	w1, [x20,834]
	str	x2, [x29,104]
	bl	js_hash
	ldr	x2, [x29,104]
	cmp	w2, w0
	beq	.L1980
	mov	w0, -1
	str	w0, [x25]
.L1980:
	ldr	w0, [x25]
	cmn	w0, #1
	beq	.L1981
	ldrh	w0, [x21,8]
	cmp	w0, w24
	bcs	.L1981
	ldrh	w2, [x21]
	ldrh	w1, [x27,4]
	cmp	w2, w1
	bne	.L1981
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x25,4]
	str	w1, [x22,x0]
.L1981:
	add	w5, w28, 1
	sxth	w28, w5
	b	.L2007
.L1979:
	add	w19, w19, 1
	sxth	w19, w19
	b	.L1967
.L1974:
	mov	x0, x27
	bl	ftl_map_blk_gc
	mov	x0, x27
	bl	ftl_map_blk_gc
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
	.size	FtlMapTblRecovery, .-FtlMapTblRecovery
	.align	2
	.global	FtlLoadVonderInfo
	.type	FtlLoadVonderInfo, %function
FtlLoadVonderInfo:
	adrp	x1, .LANCHOR2
	add	x1, x1, :lo12:.LANCHOR2
	stp	x29, x30, [sp, -16]!
	add	x0, x1, 3832
	add	x29, sp, 0
	ldrh	w2, [x1,844]
	strh	w2, [x0,10]
	mov	w2, -3962
	strh	w2, [x0,4]
	ldrh	w2, [x1,1400]
	strh	w2, [x0,8]
	ldrh	w2, [x1,846]
	strh	w2, [x0,6]
	ldr	x2, [x1,1216]
	str	x2, [x0,16]
	ldr	x2, [x1,1232]
	str	x2, [x0,24]
	ldr	x2, [x1,1224]
	ldr	x1, [x1,1240]
	str	x2, [x0,32]
	str	x1, [x0,40]
	bl	FtlMapTblRecovery
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlLoadVonderInfo, .-FtlLoadVonderInfo
	.align	2
	.global	FtlL2PDataInit
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	stp	x29, x30, [sp, -32]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	add	x19, x20, :lo12:.LANCHOR2
	ldr	w2, [x19,852]
	ldr	x0, [x19,1208]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w3, [x19,834]
	mov	w1, 255
	ldrh	w2, [x19,862]
	ldr	x0, [x19,1280]
	mul	w2, w3, w2
	bl	ftl_memset
	mov	w2, 0
	mov	w4, -1
.L2010:
	add	x0, x20, :lo12:.LANCHOR2
	ldrh	w1, [x0,862]
	cmp	w1, w2
	bls	.L2012
	ldr	x3, [x0,1272]
	ubfiz	x1, x2, 4, 16
	add	x3, x3, x1
	str	wzr, [x3,4]
	ldr	x3, [x0,1272]
	strh	w4, [x3,x1]
	ldr	x3, [x0,1272]
	add	x3, x3, x1
	ldrh	w1, [x0,834]
	ldr	x0, [x0,1280]
	mul	w1, w1, w2
	add	w2, w2, 1
	sxtw	x1, w1
	uxth	w2, w2
	and	x1, x1, -4
	add	x0, x0, x1
	str	x0, [x3,8]
	b	.L2010
.L2012:
	mov	w2, -1
	strh	w2, [x0,3762]
	strh	w2, [x0,3760]
	add	x1, x0, 3760
	ldr	w2, [x0,852]
	strh	w2, [x0,3770]
	mov	w2, -3902
	strh	w2, [x0,3764]
	ldrh	w2, [x0,3920]
	strh	w2, [x0,3768]
	ldrh	w2, [x0,860]
	strh	w2, [x0,3766]
	ldr	x2, [x0,1200]
	str	x2, [x0,3776]
	ldr	x2, [x0,1264]
	str	x2, [x0,3784]
	ldr	x2, [x0,1208]
	str	x2, [x0,3792]
	ldr	x0, [x0,1248]
	str	x0, [x1,40]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	FtlLoadMapInfo
	.type	FtlLoadMapInfo, %function
FtlLoadMapInfo:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FtlL2PDataInit
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x0, x0, 3760
	bl	FtlMapTblRecovery
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
	.align	2
	.global	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	adrp	x3, .LANCHOR2
	uxth	w1, w1
	add	x4, x3, :lo12:.LANCHOR2
	uxth	w2, w2
	strh	wzr, [x0,4]
	mov	w6, 65535
	ldrh	w4, [x4,756]
.L2015:
	cmp	w2, w4
	bcs	.L2021
	add	x5, x0, x2, sxtw 1
	ldrh	w5, [x5,16]
	cmp	w5, w6
	beq	.L2016
	ldrh	w5, [x0,4]
	add	w5, w5, 1
	strh	w5, [x0,4]
.L2016:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L2015
.L2021:
	add	x3, x3, :lo12:.LANCHOR2
	mov	w6, 65535
	add	x4, x0, x4, uxth 1
	ldrh	w5, [x3,824]
	mov	x3, x0
.L2018:
	cmp	x3, x4
	beq	.L2022
	ldrh	w2, [x3,16]
	cmp	w2, w6
	beq	.L2019
	ldrh	w2, [x0,4]
	add	w2, w5, w2
	sub	w2, w2, #1
	sub	w2, w2, w1
	strh	w2, [x0,4]
.L2019:
	add	x3, x3, 2
	b	.L2018
.L2022:
	ret
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
	.align	2
	.global	FtlReUsePrevPpa
	.type	FtlReUsePrevPpa, %function
FtlReUsePrevPpa:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	w20, w0
	lsr	x0, x1, 10
	stp	x21, x22, [sp,32]
	str	w1, [x29,60]
	adrp	x21, .LANCHOR2
	bl	P2V_block_in_plane
	uxth	w0, w0
	add	x2, x21, :lo12:.LANCHOR2
	ubfiz	x22, x0, 1, 16
	ldr	x3, [x2,1192]
	ldrh	w1, [x3,x22]
	cbnz	w1, .L2024
	ldr	x19, [x2,3600]
	cbz	x19, .L2025
	ldrh	w3, [x2,3608]
	mov	x4, -6148914691236517206
	ldr	x2, [x2,1288]
	mov	w5, 65535
	sub	x19, x19, x2
	asr	x19, x19, 1
	madd	x19, x4, x19, x19
	mov	w4, 6
	uxth	w19, w19
.L2026:
	cmp	w1, w3
	beq	.L2025
	cmp	w19, w0
	bne	.L2027
	add	x21, x21, :lo12:.LANCHOR2
	mov	w1, w19
	add	x0, x21, 3600
	bl	List_remove_node
	ldrh	w0, [x21,3608]
	sub	w0, w0, #1
	strh	w0, [x21,3608]
	mov	w0, w19
	bl	INSERT_DATA_LIST
	ldr	x1, [x21,1192]
	ldrh	w0, [x1,x22]
	add	w0, w0, 1
	strh	w0, [x1,x22]
	b	.L2025
.L2027:
	umull	x19, w19, w4
	ldrh	w19, [x2,x19]
	cmp	w19, w5
	beq	.L2025
	add	w1, w1, 1
	uxth	w1, w1
	b	.L2026
.L2024:
	add	w1, w1, 1
	strh	w1, [x3,x22]
.L2025:
	add	x1, x29, 60
	mov	w0, w20
	mov	w2, 1
	bl	log2phys
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlReUsePrevPpa, .-FtlReUsePrevPpa
	.align	2
	.global	make_superblock
	.type	make_superblock, %function
make_superblock:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	mov	x19, x0
	strh	wzr, [x0,4]
	strb	wzr, [x0,7]
	adrp	x0, .LANCHOR2
	add	x1, x0, :lo12:.LANCHOR2
	add	x22, x19, 16
	mov	x21, 0
	mov	x20, x0
	mov	x23, x1
	mov	w25, -1
	ldrh	w24, [x1,756]
.L2033:
	cmp	w24, w21, uxth
	bls	.L2044
	add	x0, x23, 784
	ldrh	w1, [x19]
	ldrb	w0, [x0,x21]
	bl	V2P_block
	strh	w25, [x22]
	mov	w26, w0
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L2034
	strh	w26, [x22]
	ldrb	w0, [x19,7]
	add	w0, w0, 1
	strb	w0, [x19,7]
.L2034:
	add	x21, x21, 1
	add	x22, x22, 2
	b	.L2033
.L2044:
	add	x0, x20, :lo12:.LANCHOR2
	ldrb	w1, [x19,7]
	strb	wzr, [x19,9]
	ldrh	w2, [x0,824]
	mul	w1, w1, w2
	strh	w1, [x19,4]
	ldr	w1, [x0,660]
	cbz	w1, .L2036
	ldrh	w1, [x19]
	ldr	x0, [x0,1176]
	ldrh	w0, [x0,x1,lsl 1]
	cmp	w0, 79
	bhi	.L2036
	mov	w0, 1
	strb	w0, [x19,9]
.L2036:
	adrp	x0, .LANCHOR0+24
	ldrb	w0, [x0,#:lo12:.LANCHOR0+24]
	cbz	w0, .L2037
	mov	w0, 1
	strb	w0, [x19,9]
.L2037:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	make_superblock, .-make_superblock
	.align	2
	.global	FtlLoadSysInfo
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	stp	x29, x30, [sp, -96]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	add	x19, x20, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	add	x22, x19, 1416
	add	x23, x19, 3904
	ldr	x0, [x19,1056]
	ldrh	w2, [x19,764]
	str	x0, [x22,8]
	ldr	x0, [x19,1120]
	lsl	w2, w2, 1
	str	x0, [x22,16]
	ldr	x0, [x19,1192]
	bl	ftl_memset
	ldrh	w0, [x19,3904]
	mov	w1, 65535
	cmp	w0, w1
	bne	.L2046
.L2057:
	mov	w19, -1
	b	.L2047
.L2046:
	mov	w1, 1
	adrp	x24, .LANCHOR0
	mov	w25, 61604
	mov	w26, -1
	bl	FtlGetLastWrittenPage
	ldrsh	w28, [x19,3904]
	sxth	w21, w0
	add	w0, w0, 1
	strh	w0, [x23,2]
.L2048:
	adrp	x23, .LANCHOR0
	tbnz	w21, #31, .L2056
	orr	w0, w21, w28, lsl 10
	mov	w1, 1
	str	w0, [x22,4]
	mov	w2, w1
	ldr	x0, [x19,1056]
	str	x0, [x22,8]
	mov	x0, x22
	bl	FlashReadPages
	add	x0, x24, :lo12:.LANCHOR0
	ldrb	w0, [x0,24]
	cbz	w0, .L2049
	ldr	x0, [x22,16]
	ldr	w27, [x0,12]
	cbz	w27, .L2049
	ldr	x0, [x22,8]
	ldrh	w1, [x19,834]
	bl	js_hash
	cmp	w27, w0
	beq	.L2049
	cbnz	w21, .L2050
	ldrh	w0, [x19,3908]
	cmp	w28, w0
	beq	.L2050
	sxth	w28, w0
	ldrh	w21, [x19,826]
	b	.L2052
.L2050:
	str	w26, [x22]
.L2049:
	ldr	w0, [x22]
	cmn	w0, #1
	beq	.L2052
	ldr	x0, [x19,1056]
	ldr	w1, [x0]
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	cmp	w1, w0
	bne	.L2052
	ldr	x0, [x19,1120]
	ldrh	w0, [x0]
	cmp	w0, w25
	bne	.L2052
.L2056:
	add	x19, x20, :lo12:.LANCHOR2
	mov	w2, 48
	add	x21, x19, 1416
	add	x0, x19, 3528
	ldr	x1, [x21,8]
	bl	ftl_memcpy
	ldr	x1, [x21,8]
	ldrh	w2, [x19,764]
	ldr	x0, [x19,1192]
	add	x1, x1, 48
	lsl	w2, w2, 1
	bl	ftl_memcpy
	ldrh	w2, [x19,764]
	add	x0, x23, :lo12:.LANCHOR0
	ldr	x3, [x21,8]
	uxth	x1, w2
	ldr	x0, [x0,16]
	add	x1, x1, 24
	lsr	w2, w2, 3
	lsr	x1, x1, 1
	add	w2, w2, 4
	add	x1, x3, x1, lsl 2
	bl	ftl_memcpy
	ldrh	w0, [x19,864]
	cbz	w0, .L2054
	ldrh	w0, [x19,764]
	ldr	x3, [x21,8]
	lsr	w1, w0, 3
	ldrh	w2, [x19,860]
	add	w1, w1, w0, lsl 1
	ldr	x0, [x19,1256]
	add	w1, w1, 52
	lsl	w2, w2, 2
	and	x1, x1, 65532
	add	x1, x3, x1
	bl	ftl_memcpy
	b	.L2054
.L2052:
	sub	w21, w21, #1
	sxth	w21, w21
	b	.L2048
.L2054:
	add	x0, x20, :lo12:.LANCHOR2
	mov	w2, 19539
	movk	w2, 0x4654, lsl 16
	ldr	w3, [x0,3528]
	cmp	w3, w2
	bne	.L2057
	ldrb	w3, [x0,3538]
	ldrh	w2, [x0,778]
	ldrh	w5, [x0,3536]
	strh	w5, [x0,3910]
	cmp	w3, w2
	bne	.L2057
	ldrh	w3, [x0,824]
	add	x23, x23, :lo12:.LANCHOR0
	ldrh	w2, [x0,830]
	adrp	x6, .LANCHOR4
	add	x4, x6, :lo12:.LANCHOR4
	mov	x19, x6
	mul	w3, w3, w5
	str	w3, [x0,1404]
	str	w5, [x4,3416]
	mul	w3, w3, w2
	str	w3, [x23,4048]
	ldrh	w2, [x0,1310]
	ldr	w3, [x0,768]
	strh	wzr, [x4,3426]
	sub	w2, w3, w2
	ldrh	w3, [x0,756]
	sub	w2, w2, w5
	strb	wzr, [x4,3430]
	strb	wzr, [x4,3432]
	udiv	w2, w2, w3
	ldrh	w3, [x0,3544]
	strh	w2, [x4,3420]
	lsr	w7, w3, 6
	and	w3, w3, 63
	strb	w3, [x0,3622]
	ldrb	w3, [x0,3539]
	strb	w3, [x0,3624]
	mov	w3, -1
	strh	w3, [x4,3424]
	ldrh	w3, [x0,3546]
	strh	w3, [x0,3664]
	ldrh	w3, [x0,3548]
	ldrh	w5, [x0,3542]
	lsr	w4, w3, 6
	and	w3, w3, 63
	strb	w3, [x0,3670]
	ldrb	w3, [x0,3540]
	strb	w3, [x0,3672]
	ldrh	w3, [x0,3550]
	strh	w3, [x0,3712]
	ldrh	w3, [x0,3552]
	strh	w4, [x0,3666]
	lsr	w4, w3, 6
	and	w3, w3, 63
	strh	w5, [x0,3616]
	strb	w3, [x0,3718]
	strh	w7, [x0,3618]
	strh	w4, [x0,3714]
	ldrb	w3, [x0,3541]
	strb	w3, [x0,3720]
	ldr	w2, [x0,3560]
	str	w2, [x0,908]
	ldr	w1, [x0,3568]
	ldr	w2, [x0,876]
	str	wzr, [x0,884]
	str	wzr, [x0,888]
	cmp	w1, w2
	str	wzr, [x0,904]
	str	wzr, [x0,900]
	str	wzr, [x0,912]
	str	wzr, [x0,920]
	str	wzr, [x0,896]
	bls	.L2058
	str	w1, [x0,876]
.L2058:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w1, [x0,3564]
	ldr	w2, [x0,880]
	cmp	w1, w2
	bls	.L2059
	str	w1, [x0,880]
.L2059:
	mov	w0, 65535
	cmp	w5, w0
	beq	.L2060
	add	x0, x20, :lo12:.LANCHOR2
	add	x0, x0, 3616
	bl	make_superblock
.L2060:
	add	x1, x20, :lo12:.LANCHOR2
	add	x0, x1, 3664
	ldrh	w2, [x1,3664]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L2061
	bl	make_superblock
.L2061:
	add	x20, x20, :lo12:.LANCHOR2
	mov	w1, 65535
	add	x0, x20, 3712
	ldrh	w2, [x20,3712]
	cmp	w2, w1
	beq	.L2062
	bl	make_superblock
.L2062:
	add	x1, x19, :lo12:.LANCHOR4
	mov	w19, 0
	add	x0, x1, 3424
	ldrh	w2, [x1,3424]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L2047
	bl	make_superblock
.L2047:
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlDumpBlockInfo
	.type	FtlDumpBlockInfo, %function
FtlDumpBlockInfo:
	sub	sp, sp, #176
	lsr	x0, x0, 10
	stp	x29, x30, [sp,16]
	add	x29, sp, 16
	stp	x19, x20, [sp,32]
	stp	x21, x22, [sp,48]
	stp	x23, x24, [sp,64]
	stp	x25, x26, [sp,80]
	stp	x27, x28, [sp,96]
	uxtb	w24, w1
	bl	P2V_block_in_plane
	adrp	x2, .LANCHOR3
	adrp	x20, .LANCHOR2
	uxth	w22, w0
	add	x1, x2, :lo12:.LANCHOR3
	add	x19, x20, :lo12:.LANCHOR2
	adrp	x0, .LC47
	add	x1, x1, 32
	add	x0, x0, :lo12:.LC47
	ldrh	w23, [x19,824]
	bl	printk
	ldr	x3, [x19,1192]
	ubfiz	x2, x22, 1, 16
	adrp	x0, .LC62
	mov	w1, w22
	add	x0, x0, :lo12:.LC62
	ldrh	w2, [x3,x2]
	bl	printk
	strh	w22, [x29,112]
	add	x0, x29, 112
	bl	make_superblock
	cbnz	w24, .L2082
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cbz	w0, .L2082
	mov	w0, w22
	bl	ftl_get_blk_mode
	cmp	w0, 1
	mov	w22, w0
	bne	.L2074
	ldrh	w23, [x19,826]
	b	.L2074
.L2082:
	mov	w22, 0
.L2074:
	add	x3, x20, :lo12:.LANCHOR2
	adrp	x0, .LC63
	adrp	x24, .LC58
	add	x0, x0, :lo12:.LC63
	mov	w1, w22
	mov	w2, w23
	ldrh	w3, [x3,824]
	mov	w21, 0
	mov	w26, 65535
	mov	w27, 56
	mov	w25, 4
	add	x24, x24, :lo12:.LC58
	bl	printk
.L2075:
	add	x0, x20, :lo12:.LANCHOR2
	mov	x3, 0
	mov	w19, w3
	ldrh	w5, [x0,756]
	ldrh	w6, [x0,834]
	ldrh	w7, [x0,836]
.L2076:
	cmp	w5, w3, uxth
	bls	.L2090
	add	x0, x29, 112
	add	x0, x0, x3, lsl 1
	ldrh	w1, [x0,16]
	cmp	w1, w26
	beq	.L2077
	add	x2, x20, :lo12:.LANCHOR2
	orr	w1, w21, w1, lsl 10
	umull	x0, w19, w27
	ldr	x4, [x2,1000]
	add	x4, x4, x0
	str	w1, [x4,4]
	ldr	x1, [x2,1000]
	ldr	x4, [x2,1080]
	add	x0, x1, x0
	ldr	x2, [x2,1136]
	mul	w1, w19, w6
	sdiv	w1, w1, w25
	add	x1, x4, x1, sxtw 2
	str	x1, [x0,8]
	mul	w1, w19, w7
	add	w19, w19, 1
	uxth	w19, w19
	sdiv	w1, w1, w25
	add	x1, x2, x1, sxtw 2
	str	x1, [x0,16]
.L2077:
	add	x3, x3, 1
	b	.L2076
.L2090:
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, w19
	mov	w2, w22
	mov	x28, 0
	ldr	x0, [x0,1000]
	bl	FlashReadPages
	mov	x8, 56
.L2079:
	cmp	w19, w28, uxth
	bls	.L2091
	add	x0, x20, :lo12:.LANCHOR2
	ldrh	w1, [x29,112]
	mul	x2, x28, x8
	str	x8, [x29,104]
	add	x28, x28, 1
	ldr	x4, [x0,1000]
	add	x3, x4, x2
	ldr	x0, [x3,8]
	ldr	x7, [x3,16]
	ldr	w5, [x0]
	str	w5, [sp]
	ldr	w0, [x0,4]
	str	w0, [sp,8]
	mov	x0, x24
	ldr	w2, [x4,x2]
	ldr	w5, [x7,4]
	ldr	w4, [x7]
	ldr	w6, [x7,8]
	ldr	w3, [x3,4]
	ldr	w7, [x7,12]
	bl	printk
	ldr	x8, [x29,104]
	b	.L2079
.L2091:
	add	w21, w21, 1
	uxth	w21, w21
	cmp	w21, w23
	bne	.L2075
.L2081:
	sub	sp, x29, #16
	mov	w0, 0
	ldp	x19, x20, [sp,32]
	ldp	x21, x22, [sp,48]
	ldp	x23, x24, [sp,64]
	ldp	x25, x26, [sp,80]
	ldp	x27, x28, [sp,96]
	ldp	x29, x30, [sp,16]
	add	sp, sp, 176
	ret
	.size	FtlDumpBlockInfo, .-FtlDumpBlockInfo
	.align	2
	.global	FtlScanAllBlock
	.type	FtlScanAllBlock, %function
FtlScanAllBlock:
	sub	sp, sp, #144
	adrp	x1, .LANCHOR3
	adrp	x0, .LC47
	add	x1, x1, :lo12:.LANCHOR3
	add	x0, x0, :lo12:.LC47
	add	x1, x1, 56
	stp	x29, x30, [sp,16]
	add	x29, sp, 16
	stp	x23, x24, [sp,64]
	stp	x21, x22, [sp,48]
	adrp	x23, .LC65
	adrp	x22, .LC64
	adrp	x24, .LC66
	add	x22, x22, :lo12:.LC64
	stp	x19, x20, [sp,32]
	add	x23, x23, :lo12:.LC65
	mov	w20, 0
	add	x24, x24, :lo12:.LC66
	stp	x25, x26, [sp,80]
	bl	printk
.L2093:
	adrp	x21, .LANCHOR2
	add	x19, x21, :lo12:.LANCHOR2
	ldrh	w0, [x19,766]
	cmp	w0, w20
	bls	.L2102
	mov	w0, w20
	strh	w20, [x29,80]
	bl	ftl_get_blk_mode
	mov	w3, w0
	ldr	x4, [x19,1192]
	ubfiz	x2, x20, 1, 16
	mov	w1, w20
	mov	x0, x22
	ldrh	w2, [x4,x2]
	bl	printk
	add	x0, x29, 80
	bl	make_superblock
	mov	x1, 0
	ldrh	w6, [x19,756]
	ldrh	w7, [x19,834]
	mov	w9, 65535
	ldrh	w8, [x19,836]
	mov	w10, 56
	mov	w19, w1
	mov	w5, 4
.L2094:
	cmp	w6, w1, uxth
	bls	.L2103
	add	x0, x29, 80
	add	x0, x0, x1, lsl 1
	ldrh	w0, [x0,16]
	cmp	w0, w9
	beq	.L2095
	add	x3, x21, :lo12:.LANCHOR2
	lsl	w0, w0, 10
	umull	x2, w19, w10
	ldr	x4, [x3,1000]
	add	x4, x4, x2
	str	w0, [x4,4]
	ldr	x0, [x3,1000]
	ldr	x4, [x3,1080]
	add	x2, x0, x2
	ldr	x3, [x3,1136]
	mul	w0, w19, w7
	sdiv	w0, w0, w5
	add	x0, x4, x0, sxtw 2
	str	x0, [x2,8]
	mul	w0, w19, w8
	add	w19, w19, 1
	uxth	w19, w19
	sdiv	w0, w0, w5
	add	x0, x3, x0, sxtw 2
	str	x0, [x2,16]
.L2095:
	add	x1, x1, 1
	b	.L2094
.L2103:
	add	x0, x21, :lo12:.LANCHOR2
	mov	w1, w19
	mov	w2, 0
	mov	x25, 0
	mov	x26, 56
	ldr	x0, [x0,1000]
	bl	FlashReadPages
.L2097:
	add	x0, x21, :lo12:.LANCHOR2
	cmp	w19, w25, uxth
	bls	.L2104
	ldr	x4, [x0,1000]
	mul	x2, x25, x26
	ldrh	w1, [x29,80]
	add	x25, x25, 1
	add	x3, x4, x2
	ldr	x0, [x3,8]
	ldr	x7, [x3,16]
	ldr	w5, [x0]
	str	w5, [sp]
	ldr	w0, [x0,4]
	str	w0, [sp,8]
	mov	x0, x23
	ldr	w2, [x4,x2]
	ldr	w5, [x7,4]
	ldr	w4, [x7]
	ldr	w6, [x7,8]
	ldr	w3, [x3,4]
	ldr	w7, [x7,12]
	bl	printk
	b	.L2097
.L2104:
	ldr	x0, [x0,1000]
	mov	w1, w19
	mov	w2, 1
	mov	x25, 0
	mov	x26, 56
	bl	FlashReadPages
.L2099:
	cmp	w19, w25, uxth
	bls	.L2105
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w1, [x29,80]
	mul	x2, x25, x26
	add	x25, x25, 1
	ldr	x4, [x0,1000]
	add	x3, x4, x2
	ldr	x0, [x3,8]
	ldr	x7, [x3,16]
	ldr	w5, [x0]
	str	w5, [sp]
	ldr	w0, [x0,4]
	str	w0, [sp,8]
	mov	x0, x24
	ldr	w2, [x4,x2]
	ldr	w5, [x7,4]
	ldr	w4, [x7]
	ldr	w6, [x7,8]
	ldr	w3, [x3,4]
	ldr	w7, [x7,12]
	bl	printk
	b	.L2099
.L2105:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L2093
.L2102:
	sub	sp, x29, #16
	mov	w0, 0
	ldp	x19, x20, [sp,32]
	ldp	x21, x22, [sp,48]
	ldp	x23, x24, [sp,64]
	ldp	x25, x26, [sp,80]
	ldp	x29, x30, [sp,16]
	add	sp, sp, 144
	ret
	.size	FtlScanAllBlock, .-FtlScanAllBlock
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	stp	x29, x30, [sp, -112]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	stp	x19, x20, [sp,16]
	add	x19, x21, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	mov	w23, 6
	mov	w27, 0
	ldrh	w2, [x19,766]
	mov	w22, w27
	ldr	x0, [x19,1288]
	mov	w24, -1
	mov	w25, 32768
	mul	w2, w2, w23
	bl	ftl_memset
	str	xzr, [x19,3600]
	str	xzr, [x19,3576]
	str	xzr, [x19,3584]
	strh	wzr, [x19,3592]
	strh	wzr, [x19,3608]
	strh	wzr, [x19,866]
	mov	w19, w27
.L2107:
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w1, [x0,764]
	cmp	w19, w1
	bge	.L2114
	mov	w20, 0
	ldrh	w3, [x0,756]
	ldrh	w2, [x0,824]
	mov	w28, w20
	mov	x26, x0
.L2115:
	cmp	w28, w3
	bge	.L2116
	add	x0, x26, 784
	mov	w1, w19
	str	x2, [x29,96]
	str	x3, [x29,104]
	ldrb	w0, [x0,w28,sxtw]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	ldr	x3, [x29,104]
	ldr	x2, [x29,96]
	cbnz	w0, .L2108
	add	w20, w2, w20
	sxth	w20, w20
.L2108:
	add	w28, w28, 1
	sxth	w28, w28
	b	.L2115
.L2116:
	cbz	w20, .L2110
	sdiv	w20, w25, w20
	sxth	w20, w20
	b	.L2111
.L2110:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x0, [x0,1192]
	strh	w24, [x0,w19,sxtw 1]
.L2111:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x1, [x0,1288]
	smaddl	x1, w19, w23, x1
	strh	w20, [x1,4]
	ldrh	w1, [x0,3616]
	cmp	w19, w1
	beq	.L2112
	ldrh	w1, [x0,3664]
	cmp	w19, w1
	beq	.L2112
	ldrh	w1, [x0,3712]
	cmp	w19, w1
	beq	.L2112
	ldr	x0, [x0,1192]
	ldrh	w0, [x0,w19,sxtw 1]
	cbnz	w0, .L2113
	add	w27, w27, 1
	mov	w0, w19
	uxth	w27, w27
	bl	INSERT_FREE_LIST
	b	.L2112
.L2113:
	add	w22, w22, 1
	mov	w0, w19
	uxth	w22, w22
	bl	INSERT_DATA_LIST
.L2112:
	add	w19, w19, 1
	sxth	w19, w19
	b	.L2107
.L2114:
	strh	w22, [x0,3592]
	strh	w27, [x0,3608]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	Ftl_save_ext_data
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	adrp	x0, .LANCHOR4
	mov	w1, 19539
	add	x0, x0, :lo12:.LANCHOR4
	movk	w1, 0x4654, lsl 16
	stp	x29, x30, [sp, -16]!
	add	x2, x0, 3472
	add	x29, sp, 0
	ldr	w3, [x0,3472]
	cmp	w3, w1
	bne	.L2117
	mov	w1, 83
	movk	w1, 0x5000, lsl 16
	str	w1, [x2,4]
	ldr	w1, [x0,3984]
	ldr	w0, [x0,3988]
	str	w0, [x2,92]
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	str	w1, [x2,88]
	ldr	w1, [x0,884]
	str	w1, [x2,8]
	ldr	w1, [x0,888]
	str	w1, [x2,12]
	ldr	w1, [x0,904]
	str	w1, [x2,16]
	ldr	w1, [x0,900]
	str	w1, [x2,20]
	ldr	w1, [x0,912]
	str	w1, [x2,28]
	ldr	w1, [x0,916]
	str	w1, [x2,32]
	ldr	w1, [x0,892]
	str	w1, [x2,36]
	ldr	w1, [x0,896]
	str	w1, [x2,40]
	ldr	w1, [x0,920]
	str	w1, [x2,44]
	ldr	w1, [x0,924]
	str	w1, [x2,48]
	ldr	w1, [x0,872]
	ldr	w0, [x0,868]
	str	w1, [x2,60]
	mov	w1, 1
	str	w0, [x2,64]
	mov	w0, 0
	bl	FtlVendorPartWrite
.L2117:
	ldp	x29, x30, [sp], 16
	ret
	.size	Ftl_save_ext_data, .-Ftl_save_ext_data
	.align	2
	.global	FtlEctTblFlush
	.type	FtlEctTblFlush, %function
FtlEctTblFlush:
	adrp	x2, .LANCHOR2
	mov	w1, 32
	add	x3, x2, :lo12:.LANCHOR2
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w4, [x3,660]
	cbz	w4, .L2120
	ldr	w3, [x3,924]
	cmp	w3, 39
	mov	w3, 4
	csel	w1, w1, w3, hi
.L2120:
	adrp	x3, .LANCHOR4
	add	x3, x3, :lo12:.LANCHOR4
	ldrh	w4, [x3,3992]
	cmp	w4, 31
	bhi	.L2121
	add	w4, w4, 1
	mov	w1, 1
	strh	w4, [x3,3992]
.L2121:
	cbnz	w0, .L2122
	add	x0, x2, :lo12:.LANCHOR2
	ldr	x3, [x0,1168]
	ldr	w0, [x3,20]
	add	w1, w1, w0
	ldr	w0, [x3,16]
	cmp	w0, w1
	bcc	.L2123
.L2122:
	add	x2, x2, :lo12:.LANCHOR2
	ldr	x0, [x2,1168]
	ldr	w1, [x0,16]
	str	w1, [x0,20]
	mov	w1, 17221
	movk	w1, 0x4254, lsl 16
	str	w1, [x0]
	ldr	x0, [x2,1168]
	ldrh	w1, [x2,1152]
	str	wzr, [x0,4]
	lsl	w3, w1, 9
	str	w3, [x0,12]
	ldr	w3, [x0,8]
	ldr	x2, [x2,1168]
	add	w3, w3, 1
	str	w3, [x0,8]
	mov	w0, 64
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L2123:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	2
	.global	Ftl_load_ext_data
	.type	Ftl_load_ext_data, %function
Ftl_load_ext_data:
	stp	x29, x30, [sp, -48]!
	mov	w1, 1
	mov	w0, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR4
	str	x21, [sp,32]
	add	x20, x19, :lo12:.LANCHOR4
	add	x21, x20, 3472
	mov	x2, x21
	bl	FtlVendorPartRead
	ldr	w1, [x20,3472]
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	cmp	w1, w0
	beq	.L2128
	mov	x0, x21
	mov	w1, 0
	mov	w2, 512
	bl	ftl_memset
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	str	w0, [x20,3472]
.L2128:
	add	x1, x19, :lo12:.LANCHOR4
	mov	w2, 19539
	movk	w2, 0x4654, lsl 16
	add	x0, x1, 3472
	adrp	x20, .LANCHOR2
	ldr	w3, [x1,3472]
	cmp	w3, w2
	bne	.L2129
	ldr	w2, [x1,3560]
	str	w2, [x1,3984]
	ldr	w2, [x1,3564]
	str	w2, [x1,3988]
	add	x1, x20, :lo12:.LANCHOR2
	ldr	w2, [x0,8]
	str	w2, [x1,884]
	ldr	w2, [x0,12]
	str	w2, [x1,888]
	ldr	w2, [x0,16]
	str	w2, [x1,904]
	ldr	w2, [x0,20]
	str	w2, [x1,900]
	ldr	w2, [x0,28]
	str	w2, [x1,912]
	ldr	w2, [x0,32]
	str	w2, [x1,916]
	ldr	w2, [x0,36]
	str	w2, [x1,892]
	ldr	w2, [x0,40]
	str	w2, [x1,896]
	ldr	w2, [x0,44]
	str	w2, [x1,920]
	ldr	w2, [x0,48]
	ldr	w0, [x0,60]
	str	w2, [x1,924]
	str	w0, [x1,872]
.L2129:
	add	x1, x19, :lo12:.LANCHOR4
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w2, [x1,3540]
	mov	w1, 34661
	movk	w1, 0x1234, lsl 16
	str	wzr, [x0,868]
	cmp	w2, w1
	bne	.L2130
	mov	w1, 1
	str	w1, [x0,660]
	adrp	x0, .LC45
	adrp	x1, .LC67
	add	x0, x0, :lo12:.LC45
	add	x1, x1, :lo12:.LC67
	bl	printk
.L2130:
	add	x0, x20, :lo12:.LANCHOR2
	add	x19, x19, :lo12:.LANCHOR4
	ldr	x21, [sp,32]
	ldrh	w2, [x0,816]
	ldr	w3, [x0,908]
	ldr	w1, [x0,912]
	ldrh	w0, [x0,764]
	madd	w1, w2, w3, w1
	udiv	w0, w1, w0
	str	w0, [x19,3996]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	2
	.global	update_multiplier_value
	.type	update_multiplier_value, %function
update_multiplier_value:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxth	w20, w0
	adrp	x0, .LANCHOR2
	add	x1, x0, :lo12:.LANCHOR2
	str	x25, [sp,64]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	x22, 0
	mov	x21, x0
	ldrh	w24, [x1,756]
	mov	w19, w22
	ldrh	w25, [x1,824]
	mov	x23, x1
.L2132:
	cmp	w24, w22, uxth
	bls	.L2139
	add	x0, x23, 784
	mov	w1, w20
	ldrb	w0, [x0,x22]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L2133
	add	w19, w19, w25
	uxth	w19, w19
.L2133:
	add	x22, x22, 1
	b	.L2132
.L2139:
	cbz	w19, .L2135
	mov	w0, 32768
	sdiv	w19, w0, w19
.L2135:
	add	x0, x21, :lo12:.LANCHOR2
	mov	w1, 6
	umull	x20, w20, w1
	ldr	x0, [x0,1288]
	add	x20, x0, x20
	mov	w0, 0
	strh	w19, [x20,4]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	update_multiplier_value, .-update_multiplier_value
	.align	2
	.global	GetFreeBlockMinEraseCount
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	adrp	x2, .LANCHOR2
	add	x2, x2, :lo12:.LANCHOR2
	ldr	x1, [x2,3600]
	mov	w0, w1
	cbz	x1, .L2141
	ldr	x0, [x2,1288]
	sub	x1, x1, x0
	mov	x0, -6148914691236517206
	asr	x1, x1, 1
	madd	x1, x0, x1, x1
	ldr	x0, [x2,1176]
	and	x1, x1, 65535
	ldrh	w0, [x0,x1,lsl 1]
.L2141:
	ret
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	2
	.global	GetFreeBlockMaxEraseCount
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	adrp	x2, .LANCHOR2
	uxth	w4, w0
	add	x5, x2, :lo12:.LANCHOR2
	ldr	x1, [x5,3600]
	mov	w0, w1
	cbz	x1, .L2144
	ldrh	w3, [x5,3608]
	mov	w0, 7
	mov	w6, 6
	mov	w7, 65535
	mul	w3, w3, w0
	ldr	x0, [x5,1288]
	asr	w3, w3, 3
	cmp	w4, w3
	sub	x1, x1, x0
	csel	w4, w3, w4, gt
	asr	x1, x1, 1
	mov	x3, -6148914691236517206
	madd	x1, x3, x1, x1
	mov	w3, 0
	uxth	w1, w1
.L2146:
	cmp	w3, w4
	beq	.L2149
	umull	x5, w1, w6
	ldrh	w5, [x0,x5]
	cmp	w5, w7
	bne	.L2147
.L2149:
	add	x2, x2, :lo12:.LANCHOR2
	ubfiz	x1, x1, 1, 16
	ldr	x0, [x2,1176]
	ldrh	w0, [x0,x1]
	b	.L2144
.L2147:
	add	w3, w3, 1
	mov	w1, w5
	uxth	w3, w3
	b	.L2146
.L2144:
	ret
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	2
	.global	FtlPrintInfo2buf
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	stp	x29, x30, [sp, -112]!
	adrp	x1, .LC68
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	adrp	x25, .LANCHOR0
	add	x1, x1, :lo12:.LC68
	add	x25, x25, :lo12:.LANCHOR0
	str	x27, [sp,80]
	stp	x23, x24, [sp,48]
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	mov	x23, x0
	bl	strcpy
	add	x20, x23, 12
	ldr	w2, [x25,4016]
	adrp	x1, .LC69
	adrp	x24, .LANCHOR2
	mov	x0, x20
	add	x1, x1, :lo12:.LC69
	add	x22, x24, :lo12:.LANCHOR2
	bl	sprintf
	add	x20, x20, x0, sxtw
	adrp	x1, .LC70
	mov	x0, x20
	ldr	w2, [x22,840]
	add	x1, x1, :lo12:.LC70
	bl	sprintf
	add	x20, x20, x0, sxtw
	adrp	x0, .LANCHOR1+3288
	ldr	w0, [x0,#:lo12:.LANCHOR1+3288]
	cmp	w0, 1
	beq	.L2152
	sub	w0, w20, w23
	b	.L2153
.L2152:
	add	x0, x29, 96
	add	x1, x29, 100
	add	x2, x29, 104
	add	x3, x29, 108
	bl	NandcGetTimeCfg
	ldr	w4, [x29,104]
	adrp	x1, .LC71
	ldr	w5, [x29,108]
	mov	x0, x20
	ldr	w3, [x29,100]
	add	x1, x1, :lo12:.LC71
	ldr	w2, [x29,96]
	bl	sprintf
	add	x21, x20, x0, sxtw
	adrp	x1, .LC72
	mov	x0, x21
	add	x1, x1, :lo12:.LC72
	add	x21, x21, 10
	adrp	x20, .LANCHOR4
	bl	strcpy
	add	x20, x20, :lo12:.LANCHOR4
	ldr	w2, [x22,1404]
	adrp	x1, .LC73
	mov	x0, x21
	add	x1, x1, :lo12:.LC73
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,3900]
	adrp	x1, .LC74
	mov	x0, x21
	add	x1, x1, :lo12:.LC74
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,904]
	adrp	x1, .LC75
	mov	x0, x21
	add	x1, x1, :lo12:.LC75
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,892]
	adrp	x1, .LC76
	mov	x0, x21
	add	x1, x1, :lo12:.LC76
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,888]
	adrp	x1, .LC77
	mov	x0, x21
	add	x1, x1, :lo12:.LC77
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,896]
	adrp	x1, .LC78
	mov	x0, x21
	add	x1, x1, :lo12:.LC78
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,900]
	adrp	x1, .LC79
	mov	x0, x21
	add	x1, x1, :lo12:.LC79
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,884]
	adrp	x1, .LC80
	mov	x0, x21
	add	x1, x1, :lo12:.LC80
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x20,3984]
	adrp	x1, .LC81
	mov	x0, x21
	add	x1, x1, :lo12:.LC81
	lsr	w2, w2, 11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x20,3988]
	adrp	x1, .LC82
	mov	x0, x21
	add	x1, x1, :lo12:.LC82
	lsr	w2, w2, 11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,876]
	adrp	x1, .LC83
	mov	x0, x21
	add	x1, x1, :lo12:.LC83
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,880]
	adrp	x1, .LC84
	add	x1, x1, :lo12:.LC84
	mov	x0, x21
	bl	sprintf
	add	x21, x21, x0, sxtw
	bl	FtlBbtCalcTotleCnt
	uxth	w3, w0
	ldrh	w2, [x22,1310]
	adrp	x1, .LC85
	mov	x0, x21
	add	x1, x1, :lo12:.LC85
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3608]
	adrp	x1, .LC86
	mov	x0, x21
	add	x1, x1, :lo12:.LC86
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,908]
	adrp	x1, .LC87
	mov	x0, x21
	add	x1, x1, :lo12:.LC87
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,912]
	adrp	x1, .LC88
	mov	x0, x21
	add	x1, x1, :lo12:.LC88
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x20,3996]
	adrp	x1, .LC89
	mov	x0, x21
	add	x1, x1, :lo12:.LC89
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,916]
	adrp	x1, .LC90
	mov	x0, x21
	add	x1, x1, :lo12:.LC90
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,920]
	adrp	x1, .LC91
	mov	x0, x21
	add	x1, x1, :lo12:.LC91
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,924]
	adrp	x1, .LC92
	mov	x0, x21
	add	x1, x1, :lo12:.LC92
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3558]
	adrp	x1, .LC93
	mov	x0, x21
	add	x1, x1, :lo12:.LC93
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3556]
	adrp	x1, .LC94
	mov	x0, x21
	add	x1, x1, :lo12:.LC94
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,4048]
	adrp	x1, .LC95
	mov	x0, x21
	add	x1, x1, :lo12:.LC95
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,856]
	adrp	x1, .LC96
	mov	x0, x21
	add	x1, x1, :lo12:.LC96
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,752]
	adrp	x1, .LC97
	mov	x0, x21
	add	x1, x1, :lo12:.LC97
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,1478]
	adrp	x1, .LC98
	mov	x0, x21
	add	x1, x1, :lo12:.LC98
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,764]
	adrp	x1, .LC99
	mov	x0, x21
	add	x1, x1, :lo12:.LC99
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x20,3420]
	adrp	x1, .LC100
	mov	x0, x21
	add	x1, x1, :lo12:.LC100
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,768]
	adrp	x1, .LC101
	mov	x0, x21
	add	x1, x1, :lo12:.LC101
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3904]
	adrp	x1, .LC102
	mov	x0, x21
	add	x1, x1, :lo12:.LC102
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,1304]
	adrp	x1, .LC103
	mov	x0, x21
	add	x1, x1, :lo12:.LC103
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3618]
	adrp	x1, .LC104
	mov	x0, x21
	add	x1, x1, :lo12:.LC104
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,3622]
	adrp	x1, .LC105
	mov	x0, x21
	add	x1, x1, :lo12:.LC105
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3616]
	adrp	x1, .LC106
	mov	x0, x21
	add	x1, x1, :lo12:.LC106
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,3624]
	adrp	x1, .LC107
	mov	x0, x21
	add	x1, x1, :lo12:.LC107
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3620]
	adrp	x1, .LC108
	mov	x0, x21
	add	x1, x1, :lo12:.LC108
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w3, [x22,3616]
	adrp	x1, .LC109
	ldr	x2, [x22,1192]
	mov	x0, x21
	add	x1, x1, :lo12:.LC109
	ldrh	w2, [x2,x3,lsl 1]
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3666]
	adrp	x1, .LC110
	mov	x0, x21
	add	x1, x1, :lo12:.LC110
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,3670]
	adrp	x1, .LC111
	mov	x0, x21
	add	x1, x1, :lo12:.LC111
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3664]
	adrp	x1, .LC112
	mov	x0, x21
	add	x1, x1, :lo12:.LC112
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,3672]
	adrp	x1, .LC113
	mov	x0, x21
	add	x1, x1, :lo12:.LC113
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3668]
	adrp	x1, .LC114
	mov	x0, x21
	add	x1, x1, :lo12:.LC114
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w3, [x22,3664]
	adrp	x1, .LC115
	ldr	x2, [x22,1192]
	mov	x0, x21
	add	x1, x1, :lo12:.LC115
	ldrh	w2, [x2,x3,lsl 1]
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3714]
	adrp	x1, .LC116
	mov	x0, x21
	add	x1, x1, :lo12:.LC116
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,3718]
	adrp	x1, .LC117
	mov	x0, x21
	add	x1, x1, :lo12:.LC117
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3712]
	adrp	x1, .LC118
	mov	x0, x21
	add	x1, x1, :lo12:.LC118
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,3720]
	adrp	x1, .LC119
	mov	x0, x21
	add	x1, x1, :lo12:.LC119
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,3716]
	adrp	x1, .LC120
	mov	x0, x21
	add	x1, x1, :lo12:.LC120
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x20,3426]
	adrp	x1, .LC121
	mov	x0, x21
	add	x1, x1, :lo12:.LC121
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x20,3430]
	adrp	x1, .LC122
	mov	x0, x21
	add	x1, x1, :lo12:.LC122
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x20,3424]
	adrp	x1, .LC123
	mov	x0, x21
	add	x1, x1, :lo12:.LC123
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x20,3432]
	adrp	x1, .LC124
	mov	x0, x21
	add	x1, x1, :lo12:.LC124
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x20,3428]
	adrp	x1, .LC125
	mov	x0, x21
	add	x1, x1, :lo12:.LC125
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w3, [x22,872]
	adrp	x1, .LC126
	ldr	w2, [x22,660]
	add	x1, x1, :lo12:.LC126
	ldr	w4, [x20,3556]
	mov	x0, x21
	ldr	w5, [x20,3548]
	orr	w2, w2, w3, lsl 8
	ldr	w3, [x20,3552]
	bl	sprintf
	add	x19, x21, x0, sxtw
	ldr	w2, [x20,3544]
	adrp	x1, .LC127
	mov	x0, x19
	add	x1, x1, :lo12:.LC127
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	w2, [x20,3568]
	adrp	x1, .LC128
	mov	x0, x19
	add	x1, x1, :lo12:.LC128
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,960]
	adrp	x1, .LC129
	mov	x0, x19
	add	x1, x1, :lo12:.LC129
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,962]
	adrp	x1, .LC130
	mov	x0, x19
	add	x1, x1, :lo12:.LC130
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	w2, [x22,944]
	adrp	x1, .LC131
	mov	x0, x19
	add	x1, x1, :lo12:.LC131
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,964]
	adrp	x1, .LC132
	add	x1, x1, :lo12:.LC132
	mov	x0, x19
	bl	sprintf
	add	x19, x19, x0, sxtw
	bl	GetFreeBlockMinEraseCount
	uxth	w2, w0
	adrp	x1, .LC133
	mov	x0, x19
	add	x1, x1, :lo12:.LC133
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w0, [x22,3608]
	bl	GetFreeBlockMaxEraseCount
	uxth	w2, w0
	adrp	x1, .LC134
	mov	x0, x19
	add	x1, x1, :lo12:.LC134
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w0, [x20,3424]
	mov	w1, 65535
	cmp	w0, w1
	beq	.L2154
	ubfiz	x2, x0, 1, 16
	ldr	x3, [x22,1192]
	adrp	x1, .LC135
	mov	x0, x19
	add	x1, x1, :lo12:.LC135
	ldrh	w2, [x3,x2]
	bl	sprintf
	add	x19, x19, x0, sxtw
.L2154:
	mov	w0, 0
	adrp	x22, .LC136
	mov	w20, 0
	mov	w27, 65535
	bl	List_get_gc_head_node
	mov	w26, 6
	uxth	w3, w0
	add	x22, x22, :lo12:.LC136
.L2156:
	cmp	w3, w27
	beq	.L2155
	add	x21, x24, :lo12:.LANCHOR2
	ubfiz	x6, x3, 1, 16
	umull	x25, w3, w26
	mov	x0, x19
	mov	w2, w20
	ldr	x5, [x21,1288]
	mov	x1, x22
	ldr	x4, [x21,1192]
	add	w20, w20, 1
	ldr	x7, [x21,1176]
	add	x5, x5, x25
	ldrh	w4, [x4,x6]
	ldrh	w5, [x5,4]
	ldrh	w6, [x7,x6]
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	x0, [x21,1288]
	cmp	w20, 16
	ldrh	w3, [x0,x25]
	bne	.L2156
.L2155:
	add	x1, x24, :lo12:.LANCHOR2
	adrp	x22, .LC137
	mov	w20, 0
	mov	w27, 65535
	mov	w26, 6
	add	x22, x22, :lo12:.LC137
	ldr	x0, [x1,3600]
	ldr	x3, [x1,1288]
	sub	x3, x0, x3
	mov	x0, -6148914691236517206
	asr	x3, x3, 1
	madd	x3, x0, x3, x3
	uxth	w3, w3
.L2158:
	cmp	w3, w27
	beq	.L2157
	add	x21, x24, :lo12:.LANCHOR2
	ubfiz	x5, x3, 1, 16
	umull	x25, w3, w26
	mov	x0, x19
	mov	w2, w20
	ldr	x4, [x21,1288]
	mov	x1, x22
	ldr	x6, [x21,1176]
	add	w20, w20, 1
	add	x4, x4, x25
	ldrh	w5, [x6,x5]
	ldrh	w4, [x4,4]
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	x0, [x21,1288]
	cmp	w20, 4
	ldrh	w3, [x0,x25]
	bne	.L2158
.L2157:
	sub	w0, w19, w23
.L2153:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldr	x27, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	2
	.global	ftl_proc_ftl_read
	.type	ftl_proc_ftl_read, %function
ftl_proc_ftl_read:
	stp	x29, x30, [sp, -32]!
	adrp	x1, .LC45
	adrp	x2, .LC138
	add	x29, sp, 0
	add	x1, x1, :lo12:.LC45
	add	x2, x2, :lo12:.LC138
	stp	x19, x20, [sp,16]
	mov	x20, x0
	bl	sprintf
	add	x19, x20, x0, sxtw
	mov	x0, x19
	bl	FtlPrintInfo2buf
	add	x0, x19, x0, sxtw
	sub	w0, w0, w20
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	ftl_proc_ftl_read, .-ftl_proc_ftl_read
	.align	2
	.global	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	stp	x29, x30, [sp, -96]!
	adrp	x0, .LANCHOR4
	add	x1, x0, :lo12:.LANCHOR4
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	adrp	x19, .LANCHOR2
	ldr	w3, [x1,3996]
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w2, [x1,924]
	cmp	w3, w2
	bcs	.L2163
	str	wzr, [x1,908]
	ldrh	w3, [x1,764]
	ldr	x5, [x1,1176]
	mov	x1, 0
.L2164:
	cmp	w3, w1
	bls	.L2196
	add	x4, x19, :lo12:.LANCHOR2
	ldrh	w6, [x5,x1,lsl 1]
	add	x1, x1, 1
	ldr	w2, [x4,908]
	add	w2, w6, w2
	str	w2, [x4,908]
	b	.L2164
.L2196:
	add	x2, x19, :lo12:.LANCHOR2
	add	x4, x0, :lo12:.LANCHOR4
	ldr	w1, [x2,908]
	udiv	w3, w1, w3
	str	w3, [x4,3996]
	ldr	w3, [x2,912]
	sub	w1, w1, w3
	ldrh	w3, [x2,816]
	udiv	w1, w1, w3
	str	w1, [x2,908]
	b	.L2166
.L2163:
	ldr	w2, [x1,920]
	cmp	w3, w2
	bls	.L2166
	add	w2, w2, 1
	str	w2, [x1,920]
	mov	w1, 0
.L2168:
	add	x2, x19, :lo12:.LANCHOR2
	ldrh	w3, [x2,764]
	cmp	w1, w3
	bcs	.L2166
	ubfiz	x3, x1, 1, 32
	ldr	x4, [x2,1176]
	add	w1, w1, 1
	ldrh	w2, [x4,x3]
	add	w2, w2, 1
	strh	w2, [x4,x3]
	b	.L2168
.L2166:
	add	x1, x19, :lo12:.LANCHOR2
	add	x0, x0, :lo12:.LANCHOR4
	ldr	w21, [x1,924]
	ldr	w23, [x0,3996]
	add	w0, w21, 256
	cmp	w0, w23
	bls	.L2171
	ldr	w2, [x1,920]
	add	w0, w21, 768
	cmp	w0, w2
	bls	.L2171
	ldr	w0, [x1,660]
	cbz	w0, .L2174
	cmp	w21, 40
	bls	.L2171
.L2174:
	mov	w0, 65535
	b	.L2173
.L2171:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,3608]
	add	w0, w0, w0, lsl 1
	lsr	w0, w0, 2
	bl	GetFreeBlockMaxEraseCount
	uxth	w6, w0
	add	w0, w21, 64
	cmp	w6, w0
	bcs	.L2183
	cmp	w21, 40
	bhi	.L2174
.L2183:
	add	x1, x19, :lo12:.LANCHOR2
	ldr	x3, [x1,3576]
	cbz	x3, .L2174
	mov	w26, 65535
	mov	x0, -6148914691236517206
	ldrh	w9, [x1,764]
	mov	w20, w26
	ldr	x7, [x1,1288]
	mov	w8, w26
	ldr	x25, [x1,1176]
	add	x0, x0, 1
	mov	w1, 0
	mov	w10, 6
.L2176:
	ldrh	w5, [x3]
	cmp	w5, w8
	beq	.L2178
	add	w1, w1, 1
	uxth	w1, w1
	cmp	w1, w9
	bhi	.L2174
	ldrh	w2, [x3,4]
	cbz	w2, .L2177
	sub	x4, x3, x7
	asr	x4, x4, 1
	mul	x4, x4, x0
	uxth	w3, w4
	and	x4, x4, 65535
	ldrh	w2, [x25,x4,lsl 1]
	cmp	w2, w21
	bls	.L2182
	cmp	w2, w26
	bcs	.L2177
	mov	w26, w2
	mov	w20, w3
.L2177:
	umull	x3, w5, w10
	add	x3, x7, x3
	b	.L2176
.L2182:
	mov	w20, w3
.L2178:
	mov	w0, 65535
	cmp	w20, w0
	beq	.L2174
	ubfiz	x24, x20, 1, 16
	ldrh	w22, [x25,x24]
	cmp	w22, w21
	bls	.L2180
	str	x6, [x29,88]
	bl	GetFreeBlockMinEraseCount
	ldr	x6, [x29,88]
	cmp	w21, w0, uxth
	bcs	.L2180
	add	x0, x19, :lo12:.LANCHOR2
	str	w26, [x0,924]
.L2180:
	cmp	w22, w23
	bcs	.L2174
	add	w0, w22, 128
	cmp	w6, w0
	ble	.L2174
	add	w0, w22, 256
	cmp	w0, w23
	bcc	.L2181
	add	x0, x19, :lo12:.LANCHOR2
	add	w22, w22, 768
	ldr	w0, [x0,920]
	cmp	w22, w0
	bcs	.L2174
.L2181:
	add	x19, x19, :lo12:.LANCHOR2
	adrp	x0, .LC139
	ldrh	w5, [x25,x24]
	add	x0, x0, :lo12:.LC139
	mov	w1, w20
	mov	w2, w23
	ldr	x4, [x19,1192]
	ldr	w3, [x19,920]
	ldrh	w4, [x4,x24]
	bl	printk
	mov	w0, 1
	str	w0, [x19,932]
	mov	w0, w20
.L2173:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	GetSwlReplaceBlock, .-GetSwlReplaceBlock
	.align	2
	.global	free_data_superblock
	.type	free_data_superblock, %function
free_data_superblock:
	stp	x29, x30, [sp, -16]!
	uxth	w0, w0
	mov	w1, 65535
	cmp	w0, w1
	add	x29, sp, 0
	beq	.L2198
	adrp	x2, .LANCHOR2+1192
	ubfiz	x1, x0, 1, 16
	ldr	x2, [x2,#:lo12:.LANCHOR2+1192]
	strh	wzr, [x2,x1]
	bl	INSERT_FREE_LIST
.L2198:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	free_data_superblock, .-free_data_superblock
	.align	2
	.global	allocate_data_superblock
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	stp	x19, x20, [sp,16]
	add	x19, x21, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	ldr	w1, [x19,1408]
	stp	x27, x28, [sp,80]
	cbnz	w1, .L2200
	mov	x20, x0
	add	x23, x19, 3712
	adrp	x22, .LANCHOR0
	mov	w24, 7
	add	x25, x19, 3600
.L2201:
	cmp	x20, x23
	bne	.L2202
	ldrh	w2, [x19,3608]
	ldr	w4, [x19,932]
	lsr	w3, w2, 1
	add	w1, w3, 1
	mul	w0, w2, w4
	add	w1, w1, w0, lsr 2
	ldr	w0, [x19,660]
	uxth	w1, w1
	cbz	w0, .L2203
	ldr	w0, [x19,924]
	cmp	w0, 39
	bhi	.L2203
	cmp	w0, 2
	mov	w1, 0
	bls	.L2204
	tbz	x2, 0, .L2231
	cbz	w4, .L2204
.L2231:
	mov	w1, w3
	b	.L2203
.L2202:
	ldrb	w0, [x20,8]
	mov	w1, 0
	cmp	w0, 1
	bne	.L2204
	ldrh	w0, [x19,776]
	cmp	w0, 1
	beq	.L2204
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbnz	w0, .L2204
	ldr	w2, [x19,660]
	ldrh	w0, [x19,3608]
	lsr	w1, w0, 3
	cbz	w2, .L2203
	ldr	w2, [x19,924]
	cmp	w2, 1
	bhi	.L2203
	mul	w1, w0, w24
	lsr	w1, w1, 3
.L2203:
	cbz	w1, .L2204
	sub	w1, w1, #1
	uxth	w1, w1
.L2204:
	ldrb	w2, [x20,8]
	mov	x0, x25
	bl	List_pop_index_node
	uxth	w28, w0
	ldrh	w0, [x19,3608]
	sub	w0, w0, #1
	strh	w0, [x19,3608]
	ldrh	w0, [x19,764]
	cmp	w0, w28
	bls	.L2201
	ubfiz	x26, x28, 1, 16
	ldr	x0, [x19,1192]
	ldrh	w0, [x0,x26]
	cbnz	w0, .L2201
	strh	w28, [x20]
	mov	x0, x20
	bl	make_superblock
	ldrb	w0, [x20,7]
	cbz	w0, .L2256
	mov	x0, 0
	ldrh	w5, [x19,756]
	mov	w27, w0
	mov	x3, 56
	mov	w6, 65535
.L2208:
	cmp	w5, w0, uxth
	bls	.L2259
	ldr	x4, [x19,1024]
	madd	x1, x0, x3, x4
	str	xzr, [x1,16]
	str	xzr, [x1,8]
	add	x1, x20, x0, lsl 1
	ldrh	w1, [x1,16]
	cmp	w1, w6
	beq	.L2209
	umull	x2, w27, w3
	add	w27, w27, 1
	lsl	w1, w1, 10
	add	x2, x4, x2
	uxth	w27, w27
	str	w1, [x2,4]
.L2209:
	add	x0, x0, 1
	b	.L2208
.L2259:
	add	x0, x19, 3616
	cmp	x20, x0
	bne	.L2211
	ldr	w1, [x19,660]
	cbz	w1, .L2211
	ldr	x1, [x19,1176]
	ldrh	w1, [x1,x26]
	cmp	w1, 40
	bls	.L2211
	strb	wzr, [x19,3624]
.L2211:
	ldrb	w0, [x20,8]
	cbnz	w0, .L2212
	ldr	x0, [x19,1176]
	mov	w1, 2
	ldrh	w2, [x0,x26]
	cbz	w2, .L2257
	ldrh	w1, [x19,816]
	add	w1, w2, w1
.L2257:
	strh	w1, [x0,x26]
	mov	w1, 0
	ldr	w0, [x19,908]
	add	w0, w0, 1
	str	w0, [x19,908]
	mov	w0, w28
	bl	ftl_set_blk_mode
	b	.L2215
.L2212:
	ldr	x1, [x19,1176]
	ldrh	w0, [x1,x26]
	add	w0, w0, 1
	strh	w0, [x1,x26]
	ldr	w0, [x19,912]
	add	w0, w0, 1
	str	w0, [x19,912]
	mov	w0, w28
	bl	ftl_set_blk_mode.part.17
.L2215:
	ldr	x0, [x19,1176]
	ldr	w1, [x19,920]
	ldrh	w0, [x0,x26]
	cmp	w0, w1
	bls	.L2216
	str	w0, [x19,920]
.L2216:
	ldrh	w1, [x19,816]
	mov	x3, 56
	ldr	w2, [x19,908]
	ldr	w0, [x19,912]
	madd	w0, w1, w2, w0
	ldrh	w1, [x19,764]
	udiv	w0, w0, w1
	adrp	x1, .LANCHOR4+3996
	str	w0, [x1,#:lo12:.LANCHOR4+3996]
	ldr	x1, [x19,1168]
	ldr	w0, [x1,16]
	add	w0, w0, 1
	str	w0, [x1,16]
	mov	x0, 0
.L2217:
	cmp	w27, w0, uxth
	bls	.L2260
	ldr	x1, [x19,1024]
	madd	x1, x0, x3, x1
	add	x0, x0, 1
	ldr	w2, [x1,4]
	and	w2, w2, -1024
	str	w2, [x1,4]
	b	.L2217
.L2260:
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L2219
	ldrb	w0, [x20,8]
	cmp	w0, 1
	ldr	x0, [x19,1024]
	bne	.L2220
	mov	w1, 0
	b	.L2258
.L2220:
	mov	w1, 1
.L2258:
	mov	w2, w27
	bl	FlashEraseBlocks
.L2219:
	ldrb	w1, [x20,8]
	mov	w2, w27
	ldr	x0, [x19,1024]
	bl	FlashEraseBlocks
	mov	x1, 0
	mov	x4, 56
	mov	w2, w1
.L2221:
	cmp	w27, w1, uxth
	bls	.L2261
	mul	x0, x1, x4
	ldr	x3, [x19,1024]
	add	x5, x3, x0
	ldr	w3, [x3,x0]
	cmn	w3, #1
	bne	.L2222
	ldr	w0, [x5,4]
	add	w2, w2, 1
	str	x4, [x29,104]
	lsr	w0, w0, 10
	str	x3, [x29,112]
	str	x1, [x29,120]
	str	x2, [x29,128]
	str	x2, [x29,136]
	bl	FtlBbmMapBadBlock
	ldr	x1, [x29,120]
	ldr	x3, [x29,112]
	ldr	x4, [x29,104]
	add	x0, x20, x1, lsl 1
	ldr	x2, [x29,136]
	strh	w3, [x0,16]
	ldrb	w0, [x20,7]
	sub	w0, w0, #1
	strb	w0, [x20,7]
.L2222:
	add	x1, x1, 1
	b	.L2221
.L2261:
	cbz	w2, .L2224
	mov	w0, w28
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L2224:
	ldrb	w0, [x20,7]
	cbnz	w0, .L2225
.L2256:
	ldr	x0, [x19,1192]
	mov	w1, -1
	strh	w1, [x0,x26]
	b	.L2201
.L2225:
	add	x21, x21, :lo12:.LANCHOR2
	strh	wzr, [x20,2]
	strb	wzr, [x20,6]
	ldrh	w1, [x21,824]
	strh	w28, [x20]
	mul	w0, w0, w1
	ldr	w1, [x21,876]
	str	w1, [x20,12]
	uxth	w0, w0
	add	w1, w1, 1
	strh	w0, [x20,4]
	str	w1, [x21,876]
	ldr	x1, [x21,1192]
	ldrh	w2, [x20]
	strh	w0, [x1,x2,lsl 1]
.L2200:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlGcBufInit
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	adrp	x0, .LANCHOR4+4000
	mov	w1, 0
	adrp	x7, .LANCHOR2
	mov	w8, 24
	mov	w9, 1
	mov	w4, 4
	str	wzr, [x0,#:lo12:.LANCHOR4+4000]
	mov	w10, 56
.L2263:
	add	x2, x7, :lo12:.LANCHOR2
	adrp	x6, .LANCHOR2
	ldrh	w0, [x2,756]
	cmp	w1, w0
	bcs	.L2267
	umull	x3, w1, w8
	ldr	x5, [x2,1112]
	ldr	x6, [x2,1080]
	add	x0, x5, x3
	str	w9, [x0,16]
	ldrh	w0, [x2,834]
	mul	w0, w1, w0
	sdiv	w0, w0, w4
	add	x0, x6, x0, sxtw 2
	str	x0, [x5,x3]
	ldrh	w0, [x2,836]
	ldr	x6, [x2,1112]
	ldr	x11, [x2,1136]
	mul	w0, w1, w0
	add	x5, x6, x3
	ldr	x2, [x2,1032]
	sdiv	w0, w0, w4
	add	x0, x11, x0, sxtw 2
	str	x0, [x5,8]
	umull	x0, w1, w10
	add	w1, w1, 1
	add	x0, x2, x0
	ldr	x2, [x6,x3]
	uxth	w1, w1
	str	x2, [x0,8]
	ldr	x2, [x5,8]
	str	x2, [x0,16]
	b	.L2263
.L2267:
	mov	w7, 24
	mov	w4, 4
.L2265:
	add	x1, x6, :lo12:.LANCHOR2
	ldr	w2, [x1,1048]
	cmp	w0, w2
	bcs	.L2268
	umull	x3, w0, w7
	ldr	x5, [x1,1112]
	ldr	x8, [x1,1080]
	add	x2, x5, x3
	str	wzr, [x2,16]
	ldrh	w2, [x1,834]
	mul	w2, w0, w2
	sdiv	w2, w2, w4
	add	x2, x8, x2, sxtw 2
	str	x2, [x5,x3]
	ldrh	w2, [x1,836]
	ldr	x5, [x1,1112]
	ldr	x1, [x1,1136]
	mul	w2, w0, w2
	add	x3, x5, x3
	add	w0, w0, 1
	sdiv	w2, w2, w4
	uxth	w0, w0
	add	x1, x1, x2, sxtw 2
	str	x1, [x3,8]
	b	.L2265
.L2268:
	ret
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlVariablesInit
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	stp	x29, x30, [sp, -32]!
	mov	w0, -1
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR4
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	add	x20, x20, :lo12:.LANCHOR4
	ldrh	w2, [x19,844]
	strh	w0, [x20,4016]
	mov	w0, -1
	str	w0, [x20,4028]
	lsl	w2, w2, 1
	ldr	x0, [x19,1216]
	str	xzr, [x20,4008]
	str	wzr, [x20,4020]
	str	wzr, [x20,4024]
	str	wzr, [x19,660]
	strh	wzr, [x19,1400]
	bl	ftl_memset
	ldrh	w2, [x19,766]
	mov	w1, 0
	ldr	x0, [x19,1176]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w2, [x19,766]
	mov	w1, 0
	ldr	x0, [x19,1160]
	lsl	w2, w2, 1
	bl	ftl_memset
	add	x0, x19, 3528
	mov	w1, 0
	mov	w2, 48
	bl	ftl_memset
	mov	w1, 0
	mov	w2, 512
	add	x0, x20, 3472
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	adrp	x2, .LANCHOR2
	mov	w3, 0
	add	x2, x2, :lo12:.LANCHOR2
	mov	w7, 56
	mov	w9, 24
	ldr	w8, [x2,1048]
	ldr	x5, [x2,1112]
.L2271:
	cmp	w3, w1
	bcs	.L2270
	umull	x4, w3, w7
	mov	w2, 0
	add	x4, x0, x4
.L2276:
	cmp	w2, w8
	bcs	.L2273
	umull	x6, w2, w9
	add	x10, x5, x6
	ldr	x11, [x5,x6]
	ldr	x6, [x4,8]
	cmp	x11, x6
	bne	.L2272
	str	wzr, [x10,16]
	b	.L2273
.L2272:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L2276
.L2273:
	add	w3, w3, 1
	uxth	w3, w3
	b	.L2271
.L2270:
	ret
	.size	FtlGcBufFree, .-FtlGcBufFree
	.align	2
	.global	FtlGcBufAlloc
	.type	FtlGcBufAlloc, %function
FtlGcBufAlloc:
	adrp	x2, .LANCHOR2
	mov	w7, 24
	add	x2, x2, :lo12:.LANCHOR2
	mov	w8, 1
	mov	w9, 56
	ldr	w5, [x2,1048]
	ldr	x6, [x2,1112]
	mov	w2, 0
.L2278:
	cmp	w2, w1
	bcs	.L2284
	mov	w3, 0
.L2282:
	cmp	w3, w5
	bcs	.L2280
	umull	x4, w3, w7
	add	x4, x6, x4
	ldr	w10, [x4,16]
	cbnz	w10, .L2279
	umull	x3, w2, w9
	str	w8, [x4,16]
	add	x3, x0, x3
	ldr	x10, [x4]
	str	x10, [x3,8]
	ldr	x4, [x4,8]
	str	x4, [x3,16]
	b	.L2280
.L2279:
	add	w3, w3, 1
	uxth	w3, w3
	b	.L2282
.L2280:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L2278
.L2284:
	ret
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	2
	.global	IsBlkInGcList
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	adrp	x1, .LANCHOR4+4032
	uxth	w0, w0
	ldrh	w2, [x1,#:lo12:.LANCHOR4+4032]
	adrp	x1, .LANCHOR2+984
	ldr	x3, [x1,#:lo12:.LANCHOR2+984]
	mov	x1, 0
.L2286:
	cmp	w2, w1, uxth
	bls	.L2290
	add	x1, x1, 1
	add	x4, x3, x1, lsl 1
	ldrh	w4, [x4,-2]
	cmp	w4, w0
	bne	.L2286
	mov	w0, 1
	b	.L2287
.L2290:
	mov	w0, 0
.L2287:
	ret
	.size	IsBlkInGcList, .-IsBlkInGcList
	.align	2
	.global	FtlGcUpdatePage
	.type	FtlGcUpdatePage, %function
FtlGcUpdatePage:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	w19, w0
	lsr	x0, x19, 10
	str	x1, [x29,32]
	str	x2, [x29,40]
	bl	P2V_block_in_plane
	adrp	x3, .LANCHOR4
	ldr	x2, [x29,40]
	add	x4, x3, :lo12:.LANCHOR4
	ldr	x1, [x29,32]
	uxth	w9, w0
	mov	x6, 0
	ldrh	w8, [x4,4032]
	adrp	x4, .LANCHOR2
	add	x5, x4, :lo12:.LANCHOR2
	ldr	x7, [x5,984]
.L2292:
	uxth	w5, w6
	cmp	w5, w8
	bcs	.L2296
	add	x6, x6, 1
	add	x10, x7, x6, lsl 1
	ldrh	w10, [x10,-2]
	cmp	w10, w9
	bne	.L2292
.L2296:
	cmp	w5, w8
	bne	.L2294
	ubfiz	x5, x5, 1, 16
	strh	w0, [x7,x5]
	add	x0, x3, :lo12:.LANCHOR4
	ldrh	w5, [x0,4032]
	add	w5, w5, 1
	strh	w5, [x0,4032]
.L2294:
	add	x3, x3, :lo12:.LANCHOR4
	add	x4, x4, :lo12:.LANCHOR2
	mov	w0, 12
	ldrh	w5, [x3,4034]
	ldr	x6, [x4,992]
	umull	x5, w5, w0
	add	x6, x6, x5
	str	w1, [x6,4]
	ldr	x1, [x4,992]
	add	x0, x1, x5
	str	w2, [x0,8]
	str	w19, [x1,x5]
	ldrh	w0, [x3,4034]
	add	w0, w0, 1
	strh	w0, [x3,4034]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	stp	x29, x30, [sp, -32]!
	adrp	x0, .LANCHOR4
	mov	w1, 255
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	add	x0, x0, :lo12:.LANCHOR4
	ldrh	w2, [x19,828]
	strh	wzr, [x0,4032]
	strh	wzr, [x0,4034]
	lsl	w2, w2, 1
	ldr	x0, [x19,984]
	bl	ftl_memset
	ldrh	w3, [x19,828]
	mov	w2, 12
	ldr	x0, [x19,992]
	mov	w1, 255
	mul	w2, w3, w2
	bl	ftl_memset
	bl	FtlGcBufInit
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	stp	x29, x30, [sp, -208]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	x19, x0
	adrp	x0, .LANCHOR1
	str	w1, [x29,128]
	add	x1, x0, :lo12:.LANCHOR1
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	str	x0, [x29,120]
	ldrh	w26, [x1,3292]
	mov	w1, 65535
	cmp	w26, w1
	beq	.L2329
	cbnz	w26, .L2300
	b	.L2301
.L2329:
	mov	w26, 0
.L2300:
	adrp	x0, .LANCHOR2+824
	ldr	w1, [x29,128]
	ldrh	w0, [x0,#:lo12:.LANCHOR2+824]
	cmp	w1, w0
	bne	.L2302
.L2301:
	bl	FtlGcPageVarInit
.L2302:
	mov	w0, -1
	adrp	x25, .LANCHOR2
	str	w0, [x29,132]
	adrp	x0, .LC141
	add	x0, x0, :lo12:.LC141
	mov	w23, 0
	mov	x22, x25
	mov	x21, x25
	str	x0, [x29,112]
.L2303:
	ldrh	w0, [x19]
	mov	w20, 65535
	strb	wzr, [x19,8]
	cmp	w0, w20
	beq	.L2304
.L2305:
	str	w20, [x29,140]
.L2326:
	add	x0, x25, :lo12:.LANCHOR2
	mov	x2, x19
	mov	w27, 0
	mov	w11, 56
	mov	w8, 4
	ldrh	w6, [x0,756]
	ldrh	w9, [x0,834]
	ldrh	w10, [x0,836]
	add	x6, x19, x6, lsl 1
.L2306:
	cmp	x2, x6
	beq	.L2344
	ldrh	w1, [x2,16]
	ldr	w0, [x29,140]
	cmp	w1, w0
	beq	.L2307
	add	x5, x22, :lo12:.LANCHOR2
	orr	w1, w26, w1, lsl 10
	umull	x0, w27, w11
	add	w4, w27, 1
	ldr	x7, [x5,1000]
	add	x7, x7, x0
	str	w1, [x7,4]
	ldr	x1, [x5,1000]
	ldr	x7, [x5,1080]
	add	x0, x1, x0
	ldr	x5, [x5,1136]
	mul	w1, w27, w9
	sdiv	w1, w1, w8
	add	x1, x7, x1, sxtw 2
	str	x1, [x0,8]
	mul	w1, w27, w10
	uxth	w27, w4
	sdiv	w1, w1, w8
	add	x1, x5, x1, sxtw 2
	str	x1, [x0,16]
.L2307:
	add	x2, x2, 2
	b	.L2306
.L2344:
	add	x0, x22, :lo12:.LANCHOR2
	mov	w1, w27
	mov	w2, 0
	mov	x24, 0
	ldr	x0, [x0,1000]
	bl	FlashReadPages
.L2309:
	cmp	w27, w24, uxth
	bls	.L2345
	add	x20, x22, :lo12:.LANCHOR2
	mov	x0, 56
	mul	x28, x24, x0
	ldr	x1, [x20,1000]
	add	x0, x1, x28
	ldr	w8, [x1,x28]
	ldr	w2, [x0,4]
	str	w2, [x29,136]
	cbnz	w8, .L2310
	ldr	x7, [x0,16]
	ldr	w1, [x29,140]
	ldrh	w0, [x7]
	cmp	w0, w1
	beq	.L2323
	ldr	w0, [x7,8]
	ldr	w1, [x20,1404]
	cmp	w0, w1
	bhi	.L2323
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrb	w1, [x1,24]
	cbz	w1, .L2315
	add	x1, x29, 148
	mov	w2, w8
	str	x7, [x29,104]
	str	x8, [x29,96]
	bl	log2phys
	ldr	x7, [x29,104]
	ldr	w1, [x29,148]
	ldr	w0, [x7,12]
	cmp	w1, w0
	bne	.L2315
	cmn	w1, #1
	beq	.L2315
	str	w0, [x29,156]
	mov	w1, 1
	ldr	x0, [x20,1104]
	ldr	x8, [x29,96]
	str	x0, [x29,160]
	ldr	x0, [x20,1128]
	mov	w2, w8
	str	x0, [x29,168]
	add	x0, x29, 152
	bl	FlashReadPages
	ldr	x0, [x20,1000]
	ldrh	w2, [x20,830]
	ldr	x6, [x29,160]
	add	x5, x0, x28
	ldr	x7, [x29,104]
	lsl	w2, w2, 7
	mov	x0, 0
.L2317:
	cmp	w2, w0
	bls	.L2315
	lsl	x1, x0, 2
	ldr	x8, [x5,8]
	add	x0, x0, 1
	ldr	w8, [x8,x1]
	ldr	w1, [x6,x1]
	cmp	w8, w1
	beq	.L2317
	adrp	x0, .LC140
	ldrh	w1, [x19]
	ldr	w2, [x29,156]
	add	x0, x0, :lo12:.LC140
	bl	printk
	b	.L2323
.L2315:
	ldr	w0, [x7,12]
	add	x24, x24, 1
	ldr	w1, [x29,136]
	ldr	w2, [x7,8]
	bl	FtlGcUpdatePage
	b	.L2309
.L2310:
	ldr	x0, [x29,112]
	ldrh	w1, [x19]
	ldr	w2, [x29,136]
	bl	printk
	ldr	w0, [x20,660]
	cbnz	w0, .L2320
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cbz	w0, .L2321
.L2320:
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w1, [x19]
	ldr	x0, [x0,1176]
	ldrh	w0, [x0,x1,lsl 1]
	cmp	w0, 159
	bls	.L2322
.L2321:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x0, [x0,1000]
	ldr	w0, [x0,x28]
	cmn	w0, #1
	bne	.L2323
.L2322:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x0, [x0,1000]
	add	x5, x0, x28
	ldr	w0, [x5,4]
	str	w0, [x29,132]
.L2323:
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w1, [x19]
	mov	w26, 0
	ldr	x0, [x0,1192]
	strh	wzr, [x0,x1,lsl 1]
	ldrh	w0, [x19]
	bl	INSERT_FREE_LIST
	mov	w0, -1
	strh	w0, [x19]
	bl	FtlGcPageVarInit
	b	.L2303
.L2345:
	ldr	w0, [x29,128]
	add	w23, w23, 1
	add	w3, w26, 1
	cmp	w23, w0
	uxth	w26, w3
	bcs	.L2325
.L2327:
	add	x0, x22, :lo12:.LANCHOR2
	ldrh	w0, [x0,824]
	cmp	w0, w26
	bhi	.L2326
	b	.L2304
.L2325:
	ldr	x0, [x29,120]
	ldr	w2, [x29,140]
	add	x1, x0, :lo12:.LANCHOR1
	ldrh	w0, [x1,3292]
	cmp	w0, w2
	beq	.L2327
	add	w0, w0, w23
	strh	w0, [x1,3292]
	add	x0, x22, :lo12:.LANCHOR2
	ldrh	w0, [x0,824]
	cmp	w0, w26
	bls	.L2327
	b	.L2328
.L2304:
	ldr	x0, [x29,120]
	mov	w1, -1
	strh	w26, [x19,2]
	mov	w2, 0
	add	x0, x0, :lo12:.LANCHOR1
	strb	wzr, [x19,6]
	strh	w1, [x0,3292]
	mov	x0, x19
	mov	w1, w26
	bl	ftl_sb_update_avl_pages
.L2328:
	ldr	w0, [x29,132]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 208
	ret
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	FtlGcRefreshOpenBlock
	.type	FtlGcRefreshOpenBlock, %function
FtlGcRefreshOpenBlock:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	uxth	w19, w0
	adrp	x0, .LC142
	mov	w1, w19
	add	x0, x0, :lo12:.LC142
	bl	printk
	adrp	x2, .LANCHOR2
	add	x2, x2, :lo12:.LANCHOR2
	ldrh	w5, [x2,952]
	cmp	w5, w19
	beq	.L2347
	ldrh	w4, [x2,954]
	cmp	w4, w19
	beq	.L2347
	ldrh	w3, [x2,956]
	cmp	w3, w19
	beq	.L2347
	ldrh	w1, [x2,958]
	cmp	w1, w19
	beq	.L2347
	mov	w0, 65535
	cmp	w5, w0
	bne	.L2348
	strh	w19, [x2,952]
	b	.L2347
.L2348:
	cmp	w4, w0
	bne	.L2349
	strh	w19, [x2,954]
	b	.L2347
.L2349:
	cmp	w3, w0
	bne	.L2350
	strh	w19, [x2,956]
	b	.L2347
.L2350:
	cmp	w1, w0
	bne	.L2347
	strh	w19, [x2,958]
.L2347:
	mov	w0, 0
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcRefreshOpenBlock, .-FtlGcRefreshOpenBlock
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	uxth	w19, w0
	adrp	x0, .LC142
	mov	w1, w19
	add	x0, x0, :lo12:.LC142
	bl	printk
	adrp	x2, .LANCHOR2
	mov	w0, 0
	add	x2, x2, :lo12:.LANCHOR2
	ldrh	w6, [x2,952]
	cmp	w6, w19
	beq	.L2352
	ldrh	w5, [x2,954]
	cmp	w5, w19
	beq	.L2352
	ldrh	w4, [x2,956]
	cmp	w4, w19
	beq	.L2352
	ldrh	w3, [x2,958]
	cmp	w3, w19
	beq	.L2352
	mov	w1, 65535
	cmp	w6, w1
	bne	.L2353
	strh	w19, [x2,952]
	b	.L2352
.L2353:
	cmp	w5, w1
	bne	.L2354
	strh	w19, [x2,954]
	b	.L2361
.L2354:
	cmp	w4, w1
	bne	.L2355
	strh	w19, [x2,956]
	b	.L2361
.L2355:
	cmp	w3, w1
	mov	w0, -1
	bne	.L2352
	strh	w19, [x2,958]
.L2361:
	mov	w0, 0
.L2352:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcRefreshBlock, .-FtlGcRefreshBlock
	.align	2
	.global	FtlGcMarkBadPhyBlk
	.type	FtlGcMarkBadPhyBlk, %function
FtlGcMarkBadPhyBlk:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxth	w20, w0
	adrp	x19, .LANCHOR2
	stp	x21, x22, [sp,32]
	mov	w0, w20
	add	x22, x19, :lo12:.LANCHOR2
	str	x23, [sp,48]
	bl	P2V_block_in_plane
	uxth	w21, w0
	ldrh	w1, [x22,966]
	adrp	x0, .LC143
	mov	w2, w20
	add	x0, x0, :lo12:.LC143
	bl	printk
	mov	w0, w21
	bl	FtlGcRefreshBlock
	ldr	w0, [x22,660]
	cbz	w0, .L2363
	ubfiz	x0, x21, 1, 16
	ldr	x2, [x22,1176]
	ldrh	w1, [x2,x0]
	cmp	w1, 39
	bls	.L2363
	sub	w1, w1, #40
	strh	w1, [x2,x0]
.L2363:
	add	x0, x19, :lo12:.LANCHOR2
	adrp	x1, .LANCHOR4
	add	x1, x1, :lo12:.LANCHOR4
	add	x1, x1, 4040
	ldrh	w3, [x0,966]
	mov	x0, 0
.L2364:
	cmp	w3, w0, uxth
	bls	.L2370
	add	x0, x0, 1
	add	x2, x1, x0, lsl 1
	ldrh	w2, [x2,-2]
	cmp	w2, w20
	bne	.L2364
	b	.L2365
.L2370:
	cmp	w3, 15
	bhi	.L2365
	adrp	x1, .LANCHOR4
	add	x19, x19, :lo12:.LANCHOR2
	add	x1, x1, :lo12:.LANCHOR4
	add	w0, w3, 1
	add	x1, x1, 4040
	strh	w0, [x19,966]
	strh	w20, [x1,w3,sxtw 1]
.L2365:
	mov	w0, 0
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlGcMarkBadPhyBlk, .-FtlGcMarkBadPhyBlk
	.align	2
	.global	FtlGcReFreshBadBlk
	.type	FtlGcReFreshBadBlk, %function
FtlGcReFreshBadBlk:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w1, [x0,966]
	cbz	w1, .L2372
	ldrh	w3, [x0,952]
	mov	w2, 65535
	cmp	w3, w2
	bne	.L2372
	ldrh	w2, [x0,970]
	cmp	w2, w1
	bcc	.L2373
	strh	wzr, [x0,970]
.L2373:
	add	x19, x19, :lo12:.LANCHOR2
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	add	x0, x0, 4040
	ldrh	w1, [x19,970]
	ldrh	w0, [x0,w1,sxtw 1]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	w0, [x19,970]
	add	w0, w0, 1
	strh	w0, [x19,970]
.L2372:
	mov	w0, 0
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	x27, [sp,80]
	add	x19, x19, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	uxth	w23, w0
	ldrh	w0, [x19,966]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	mov	w21, 0
	cbz	w0, .L2379
	adrp	x24, .LANCHOR4
	adrp	x22, .LC144
	add	x22, x22, :lo12:.LC144
	add	x25, x24, :lo12:.LANCHOR4
.L2387:
	ldrh	w0, [x19,756]
	cmp	w0, w21
	bls	.L2388
	add	x0, x19, 784
	mov	w1, w23
	mov	w20, 0
	add	x27, x24, :lo12:.LANCHOR4
	ldrb	w0, [x0,w21,sxtw]
	bl	V2P_block
	uxth	w26, w0
.L2380:
	ldrh	w0, [x19,966]
	cmp	w0, w20
	bls	.L2389
	add	x0, x27, 4040
	ldrh	w0, [x0,w20,sxtw 1]
	cmp	w0, w26
	bne	.L2381
	mov	w1, w26
	mov	x0, x22
	bl	printk
	mov	w0, w26
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	w2, [x19,966]
	mov	w3, w20
.L2382:
	cmp	w3, w2
	bcs	.L2390
	add	x4, x25, 4040
	add	w0, w3, 1
	ldrh	w1, [x4,w0,sxtw 1]
	strh	w1, [x4,w3,sxtw 1]
	uxth	w3, w0
	b	.L2382
.L2390:
	sub	w2, w2, #1
	strh	w2, [x19,966]
.L2381:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L2380
.L2389:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L2387
.L2388:
	bl	FtlGcReFreshBadBlk
.L2379:
	mov	w0, 0
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	uxth	w19, w0
	add	x2, x20, :lo12:.LANCHOR2
	ubfiz	x0, x19, 1, 16
	ldr	x1, [x2,1192]
	ldrh	w1, [x1,x0]
	cbnz	w1, .L2392
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	ldrh	w3, [x0,3424]
	cmp	w3, w19
	bne	.L2393
	mov	w1, -1
	strh	w1, [x0,3424]
	b	.L2394
.L2393:
	ldrh	w3, [x2,3616]
	mov	w0, w1
	cmp	w3, w19
	beq	.L2395
	ldrh	w3, [x2,3664]
	cmp	w3, w19
	beq	.L2395
	ldrh	w2, [x2,3712]
	cmp	w2, w19
	beq	.L2395
.L2394:
	add	x20, x20, :lo12:.LANCHOR2
	mov	w1, w19
	add	x0, x20, 3576
	bl	List_remove_node
	ldrh	w0, [x20,3592]
	sub	w0, w0, #1
	strh	w0, [x20,3592]
	mov	w0, w19
	bl	free_data_superblock
	mov	w0, w19
	bl	FtlGcFreeBadSuperBlk
	mov	w0, 1
	b	.L2395
.L2392:
	mov	w0, w19
	bl	List_update_data_list
	mov	w0, 0
.L2395:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	update_vpc_list, .-update_vpc_list
	.align	2
	.global	decrement_vpc_count
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxth	w19, w0
	mov	w0, 65535
	stp	x21, x22, [sp,32]
	cmp	w19, w0
	beq	.L2400
	adrp	x20, .LANCHOR2
	ubfiz	x21, x19, 1, 16
	add	x20, x20, :lo12:.LANCHOR2
	ldr	x0, [x20,1192]
	ldrh	w2, [x0,x21]
	cbnz	w2, .L2401
	adrp	x0, .LC145
	mov	w1, w19
	add	x0, x0, :lo12:.LC145
	add	x22, x20, 3600
	bl	printk
	ldr	x0, [x20,1192]
	mov	w1, 32
	strh	w1, [x0,x21]
	mov	x0, x22
	mov	w1, w19
	bl	test_node_in_list
	cbz	w0, .L2402
	mov	w1, w19
	mov	x0, x22
	bl	List_remove_node
	ldrh	w0, [x20,3608]
	sub	w0, w0, #1
	strh	w0, [x20,3608]
	mov	w0, w19
	bl	INSERT_DATA_LIST
	ldr	x2, [x20,1192]
	adrp	x0, .LC146
	add	x0, x0, :lo12:.LC146
	mov	w1, w19
	ldrh	w2, [x2,x21]
	bl	printk
.L2402:
	mov	w0, w19
	bl	FtlGcRefreshBlock
	b	.L2409
.L2401:
	sub	w2, w2, #1
	strh	w2, [x0,x21]
.L2400:
	adrp	x20, .LANCHOR4
	mov	w0, 65535
	add	x20, x20, :lo12:.LANCHOR4
	ldrh	w1, [x20,4016]
	cmp	w1, w0
	bne	.L2404
	strh	w19, [x20,4016]
.L2409:
	mov	w0, 0
	b	.L2403
.L2404:
	cmp	w1, w19
	mov	w0, 0
	beq	.L2403
	mov	w0, w1
	bl	update_vpc_list
	cmp	w0, wzr
	cset	w0, ne
	strh	w19, [x20,4016]
.L2403:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	FtlRecoverySuperblock
	.type	FtlRecoverySuperblock, %function
FtlRecoverySuperblock:
	stp	x29, x30, [sp, -208]!
	mov	w2, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	mov	x19, x0
	ldrh	w0, [x0]
	cmp	w0, w2
	beq	.L2572
	ldrh	w0, [x19,2]
	adrp	x20, .LANCHOR2
	str	w0, [x29,188]
	add	x0, x20, :lo12:.LANCHOR2
	ldrb	w28, [x19,6]
	ldr	w1, [x29,188]
	ldrh	w0, [x0,824]
	cmp	w0, w1
	bne	.L2413
	strh	wzr, [x19,4]
	b	.L2578
.L2413:
	ldrh	w0, [x19,16]
	mov	w1, 0
.L2414:
	cmp	w0, w2
	bne	.L2581
	add	w1, w1, 1
	uxth	w1, w1
	add	x0, x19, x1, sxtw 1
	ldrh	w0, [x0,16]
	b	.L2414
.L2581:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2416
	bl	FtlGetLastWrittenPage
	cmn	w0, #1
	mov	w21, w0
	beq	.L2417
	adrp	x0, .LANCHOR0
	mov	w23, w21
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w1, [x0,4044]
	cbnz	w1, .L2491
	add	x0, x0, 136
	ldrh	w23, [x0,w21,sxtw 1]
	b	.L2491
.L2416:
	mov	w1, 0
	bl	FtlGetLastWrittenPage
	cmn	w0, #1
	mov	w21, w0
	beq	.L2417
	mov	w23, w0
.L2491:
	add	x0, x20, :lo12:.LANCHOR2
	mov	x2, x19
	mov	w25, 0
	mov	w8, 65535
	mov	w9, 56
	mov	w10, 4
	ldrh	w4, [x0,756]
	ldrh	w7, [x0,836]
	add	x4, x19, x4, lsl 1
	b	.L2419
.L2417:
	strh	wzr, [x19,2]
.L2578:
	strb	wzr, [x19,6]
	b	.L2572
.L2421:
	ldrh	w1, [x2,16]
	cmp	w1, w8
	beq	.L2420
	add	x5, x20, :lo12:.LANCHOR2
	orr	w1, w23, w1, lsl 10
	umull	x0, w25, w9
	add	w3, w25, 1
	ldr	x6, [x5,1000]
	add	x6, x6, x0
	str	w1, [x6,4]
	ldr	x1, [x5,1000]
	ldr	x5, [x5,1136]
	add	x0, x1, x0
	mul	w1, w25, w7
	uxth	w25, w3
	str	xzr, [x0,8]
	sdiv	w1, w1, w10
	add	x1, x5, x1, sxtw 2
	str	x1, [x0,16]
.L2420:
	add	x2, x2, 2
.L2419:
	cmp	x2, x4
	bne	.L2421
	ldrb	w0, [x19,8]
	str	wzr, [x29,184]
	cmp	w0, 1
	bne	.L2422
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cmp	w0, wzr
	cset	w0, ne
	str	w0, [x29,184]
.L2422:
	add	x22, x20, :lo12:.LANCHOR2
	ldr	w2, [x29,184]
	mov	w1, w25
	mov	x26, 0
	mov	w24, 65535
	adrp	x27, .LANCHOR4
	ldr	x0, [x22,1000]
	bl	FlashReadPages
	ldr	w0, [x22,880]
	adrp	x22, .LC147
	str	w0, [x29,176]
	add	x22, x22, :lo12:.LC147
	uxth	w0, w23
	str	w0, [x29,168]
.L2423:
	uxth	w2, w26
	cmp	w2, w25
	bcs	.L2430
	add	x6, x20, :lo12:.LANCHOR2
	mov	x0, 56
	mul	x0, x26, x0
	ldr	x4, [x6,1000]
	add	x1, x4, x0
	ldr	w0, [x4,x0]
	cbnz	w0, .L2424
	ldr	x7, [x1,16]
	ldr	w4, [x7,4]
	cmn	w4, #1
	beq	.L2425
	ldr	w1, [x6,880]
	mov	w0, w4
	str	x7, [x29,136]
	str	x2, [x29,144]
	str	x4, [x29,152]
	str	x6, [x29,160]
	bl	ftl_cmp_data_ver
	ldr	x6, [x29,160]
	ldr	x4, [x29,152]
	ldr	x2, [x29,144]
	ldr	x7, [x29,136]
	cbz	w0, .L2425
	add	w4, w4, 1
	str	w4, [x6,880]
.L2425:
	ldr	w0, [x7]
	cmn	w0, #1
	bne	.L2426
.L2430:
	cmp	w2, w25
	bne	.L2573
	add	x0, x20, :lo12:.LANCHOR2
	add	w21, w21, 1
	uxth	w21, w21
	ldr	x0, [x0,1000]
	ldr	w0, [x0,4]
	b	.L2574
.L2424:
	ldr	w1, [x1,4]
	mov	x0, x22
	bl	printk
	ldr	w24, [x29,168]
	add	x0, x27, :lo12:.LANCHOR4
	ldrh	w1, [x19]
	strh	w1, [x0,4074]
.L2426:
	add	x26, x26, 1
	b	.L2423
.L2573:
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, 56
	uxth	w21, w21
	umull	x2, w2, w1
	ldr	x0, [x0,1000]
	add	x2, x0, x2
	ldr	w0, [x2,4]
.L2574:
	lsr	x0, x0, 10
	bl	P2V_plane
	uxth	w26, w0
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2432
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w2, [x0,4044]
	cbnz	w2, .L2432
	add	x0, x0, 136
	ldrh	w21, [x0,w21,sxtw 1]
.L2432:
	add	x0, x20, :lo12:.LANCHOR2
	ldrh	w0, [x0,824]
	cmp	w0, w21
	bne	.L2433
	strh	w21, [x19,2]
	strb	wzr, [x19,6]
	strh	wzr, [x19,4]
.L2433:
	uxth	w0, w28
	str	w0, [x29,152]
	ldr	w0, [x29,188]
	cmp	w21, w0
	bne	.L2434
	ldr	w0, [x29,152]
	cmp	w26, w0
	beq	.L2579
.L2434:
	ldr	w0, [x29,176]
	adrp	x25, .LANCHOR4
	sub	w22, w0, #1
	mov	w0, 65535
	cmp	w24, w0
	bne	.L2435
	cbnz	w1, .L2436
.L2435:
	add	x0, x25, :lo12:.LANCHOR4
	uxth	w5, w23
	ldr	w1, [x0,4028]
	cmn	w1, #1
	bne	.L2437
	str	w22, [x0,4028]
.L2437:
	add	x0, x25, :lo12:.LANCHOR4
	ldr	w24, [x29,188]
	ldr	w3, [x0,4028]
	ldr	w0, [x29,188]
	add	w0, w0, 7
	cmp	w0, w23, uxth
	bge	.L2438
	sub	w24, w5, #7
	uxth	w24, w24
.L2438:
	mov	w28, -1
	adrp	x6, .LANCHOR0
	mov	w27, w28
	mov	w4, 65535
	mov	w7, 56
	add	x8, x6, :lo12:.LANCHOR0
.L2439:
	cmp	w24, w5
	bhi	.L2452
	add	x0, x20, :lo12:.LANCHOR2
	mov	w23, 0
	ldrh	w2, [x0,756]
	mov	x0, x19
	add	x2, x19, x2, lsl 1
.L2453:
	cmp	x0, x2
	beq	.L2582
	ldrh	w1, [x0,16]
	cmp	w1, w4
	beq	.L2440
	add	x10, x20, :lo12:.LANCHOR2
	orr	w1, w24, w1, lsl 10
	umull	x9, w23, w7
	add	w23, w23, 1
	ldr	x10, [x10,1000]
	uxth	w23, w23
	add	x9, x10, x9
	str	w1, [x9,4]
.L2440:
	add	x0, x0, 2
	b	.L2453
.L2582:
	add	x9, x20, :lo12:.LANCHOR2
	ldr	w2, [x29,184]
	mov	w1, w23
	str	x8, [x29,120]
	str	x6, [x29,128]
	ldr	x0, [x9,1000]
	str	x7, [x29,136]
	str	x4, [x29,144]
	str	x3, [x29,160]
	str	x5, [x29,168]
	str	x9, [x29,176]
	bl	FlashReadPages
	ldr	x6, [x29,128]
	mov	w1, 0
	ldr	x9, [x29,176]
	sxtw	x11, w24
	add	x0, x6, :lo12:.LANCHOR0
	ldr	x5, [x29,168]
	ldr	x3, [x29,160]
	ldr	x4, [x29,144]
	ldrb	w10, [x0,4044]
	ldr	x0, [x9,1000]
	ldr	x7, [x29,136]
	ldr	x8, [x29,120]
	add	x0, x0, 16
.L2442:
	cmp	w1, w23
	beq	.L2583
	ldr	w2, [x0,-16]
	cbnz	w2, .L2443
	ldr	x2, [x0]
	ldrh	w9, [x2]
	cmp	w9, w4
	beq	.L2444
	ldr	w9, [x2,4]
	cmn	w9, #1
	beq	.L2444
	add	x2, x25, :lo12:.LANCHOR4
	cmn	w28, #1
	ldr	w27, [x2,4028]
	str	w9, [x2,4028]
	bne	.L2444
	add	x2, x8, 648
	ldrh	w2, [x2,x11,lsl 1]
	cmp	w2, w4
	bne	.L2445
	cbz	w10, .L2444
.L2445:
	cmp	w27, w22
	csinv	w28, w27, wzr, ne
	b	.L2444
.L2443:
	add	x0, x25, :lo12:.LANCHOR4
	ldrh	w1, [x19]
	strh	w1, [x0,4074]
	ldrb	w1, [x19,8]
	cbnz	w1, .L2436
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 648
	ldrh	w2, [x1,w24,sxtw 1]
	mov	w1, 65535
	cmp	w2, w1
	bne	.L2447
	cmn	w28, #1
	beq	.L2448
	str	w28, [x0,4028]
	b	.L2436
.L2448:
	cmp	w3, w22
	beq	.L2449
	str	w3, [x0,4028]
	b	.L2436
.L2449:
	ldr	w1, [x0,4028]
	b	.L2580
.L2447:
	cmp	w27, w22
	beq	.L2450
	cmn	w27, #1
	beq	.L2436
	str	w27, [x0,4028]
	b	.L2436
.L2450:
	ldr	w1, [x0,4028]
	cmp	w1, w22
	beq	.L2436
.L2580:
	sub	w1, w1, #1
	b	.L2575
.L2444:
	add	w1, w1, 1
	add	x0, x0, 56
	uxth	w1, w1
	b	.L2442
.L2583:
	add	w24, w24, 1
	uxth	w24, w24
	b	.L2439
.L2452:
	add	x0, x25, :lo12:.LANCHOR4
	mov	w1, -1
.L2575:
	str	w1, [x0,4028]
.L2436:
	add	x0, x25, :lo12:.LANCHOR4
	mov	w1, 1
	ldr	w23, [x29,188]
	strh	w1, [x0,4076]
	adrp	x0, .LC149
	add	x0, x0, :lo12:.LC149
	str	x0, [x29,128]
	adrp	x0, .LC148
	add	x0, x0, :lo12:.LC148
	str	x0, [x29,120]
.L2454:
	add	x0, x20, :lo12:.LANCHOR2
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	str	wzr, [x29,176]
	mov	w7, 65535
	mov	w8, 56
	ldrh	w3, [x0,756]
	ldrb	w6, [x1,4044]
	mov	x1, x19
	add	x3, x19, x3, lsl 1
.L2455:
	cmp	x1, x3
	beq	.L2584
	ldrh	w2, [x1,16]
	cmp	w2, w7
	beq	.L2456
	ldr	w0, [x29,176]
	add	x5, x20, :lo12:.LANCHOR2
	orr	w2, w23, w2, lsl 10
	ldr	x4, [x5,1000]
	umull	x0, w0, w8
	add	x4, x4, x0
	str	w2, [x4,4]
	ldrb	w2, [x19,8]
	cmp	w2, 1
	bne	.L2457
	cbz	w6, .L2457
	ldr	x2, [x5,1000]
	add	x0, x2, x0
	ldr	w2, [x0,4]
	orr	w2, w2, -2147483648
	str	w2, [x0,4]
.L2457:
	ldr	w0, [x29,176]
	add	w0, w0, 1
	uxth	w0, w0
	str	w0, [x29,176]
.L2456:
	add	x1, x1, 2
	b	.L2455
.L2584:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w1, [x29,176]
	ldr	w2, [x29,184]
	ldr	x0, [x0,1000]
	bl	FlashReadPages
	str	xzr, [x29,160]
.L2459:
	ldrh	w0, [x29,160]
	ldr	w1, [x29,176]
	cmp	w1, w0, uxth
	bls	.L2585
	ldr	x0, [x29,160]
	mov	x24, 56
	mul	x24, x0, x24
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x28, [x0,1000]
	add	x27, x28, x24
	ldr	w1, [x27,4]
	str	w1, [x29,204]
	lsr	x0, x1, 10
	str	x1, [x29,168]
	bl	P2V_plane
	uxth	w0, w0
	ldr	w1, [x29,188]
	cmp	w23, w1
	ldr	x1, [x29,168]
	bcc	.L2460
	ldr	w2, [x29,152]
	cmp	w0, w2
	bcs	.L2495
	ldr	w2, [x29,188]
	cmp	w23, w2
	beq	.L2460
.L2495:
	cmp	w0, w26
	bne	.L2496
	cmp	w23, w21
	beq	.L2462
.L2496:
	ldr	w0, [x28,x24]
	cmn	w0, #1
	beq	.L2464
	ldr	x28, [x27,16]
	mov	w0, 61589
	ldrh	w1, [x28]
	cmp	w1, w0
	beq	.L2465
	ldrh	w0, [x19]
	b	.L2576
.L2465:
	ldr	w22, [x28,4]
	cmn	w22, #1
	beq	.L2466
	add	x27, x20, :lo12:.LANCHOR2
	mov	w0, w22
	ldr	w1, [x27,880]
	bl	ftl_cmp_data_ver
	cbz	w0, .L2466
	add	w0, w22, 1
	str	w0, [x27,880]
.L2466:
	ldr	w27, [x28,8]
	add	x1, x29, 200
	ldr	w0, [x28,12]
	mov	w2, 0
	str	w0, [x29,196]
	mov	w0, w27
	bl	log2phys
	ldr	w28, [x29,196]
	add	x0, x25, :lo12:.LANCHOR4
	str	x0, [x29,136]
	ldr	w1, [x0,4028]
	cmn	w1, #1
	beq	.L2467
	mov	w0, w22
	bl	ftl_cmp_data_ver
	cbz	w0, .L2467
	cmn	w28, #1
	beq	.L2468
	add	x3, x20, :lo12:.LANCHOR2
	mov	w2, 0
	str	x3, [x29,112]
	ldr	x0, [x3,1000]
	add	x0, x0, x24
	str	w28, [x0,4]
	ldr	x1, [x0,16]
	ldr	x0, [x3,1000]
	str	x1, [x29,168]
	mov	w1, 1
	add	x0, x0, x24
	bl	FlashReadPages
	ldr	x3, [x29,112]
	ldr	x0, [x29,168]
	ldr	x2, [x3,1000]
	ldr	w0, [x0,4]
	str	w0, [x29,144]
	add	x4, x2, x24
	ldr	w0, [x2,x24]
	cmn	w0, #1
	bne	.L2469
	b	.L2470
.L2468:
	ldr	w0, [x29,204]
	ldr	w1, [x29,200]
	cmp	w1, w0
	bne	.L2460
	mov	w0, w27
	add	x1, x29, 196
	mov	w2, 1
	bl	log2phys
	b	.L2460
.L2469:
	ldr	x0, [x29,168]
	ldr	w28, [x0,8]
	cmp	w28, w27
	bne	.L2470
	ldr	x0, [x29,136]
	ldr	w1, [x29,144]
	str	x2, [x29,96]
	ldr	w0, [x0,4028]
	str	x3, [x29,104]
	str	x4, [x29,112]
	bl	ftl_cmp_data_ver
	ldr	x4, [x29,112]
	ldr	x3, [x29,104]
	ldr	x2, [x29,96]
	cbz	w0, .L2470
	ldr	w0, [x29,200]
	ldr	w1, [x29,204]
	cmp	w0, w1
	beq	.L2475
	ldr	w1, [x29,196]
	cmp	w0, w1
	beq	.L2470
	cmn	w0, #1
	beq	.L2473
	str	w0, [x4,4]
	mov	w2, 0
	ldr	x1, [x4,16]
	ldr	x0, [x3,1000]
	str	x1, [x29,168]
	mov	w1, 1
	add	x0, x0, x24
	bl	FlashReadPages
	b	.L2474
.L2473:
	str	w0, [x2,x24]
.L2474:
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldr	x0, [x0,1000]
	ldr	w0, [x0,x24]
	cmn	w0, #1
	beq	.L2475
	ldr	x0, [x29,168]
	ldr	w24, [x0,4]
	add	x0, x25, :lo12:.LANCHOR4
	mov	w1, w24
	ldr	w0, [x0,4028]
	bl	ftl_cmp_data_ver
	cbz	w0, .L2475
	ldr	w0, [x29,144]
	mov	w1, w24
	bl	ftl_cmp_data_ver
	cbz	w0, .L2470
.L2475:
	ldr	w1, [x29,196]
	mov	w0, w28
	bl	FtlReUsePrevPpa
.L2470:
	mov	w0, -1
	str	w0, [x29,196]
	b	.L2477
.L2467:
	ldr	w0, [x29,204]
	ldr	w1, [x29,200]
	cmp	w1, w0
	beq	.L2477
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w0, [x0,772]
	cmp	w0, w28, lsr 10
	bhi	.L2497
	cmn	w28, #1
	bne	.L2460
.L2497:
	mov	w0, w27
	add	x1, x29, 204
	mov	w2, 1
	bl	log2phys
	ldr	w28, [x29,200]
	cmn	w28, #1
	beq	.L2477
	ldr	w0, [x29,196]
	cmp	w28, w0
	beq	.L2477
	lsr	x0, x28, 10
	bl	P2V_block_in_plane
	uxth	w0, w0
	add	x1, x20, :lo12:.LANCHOR2
	ldrh	w2, [x1,3616]
	cmp	w2, w0
	beq	.L2481
	ldrh	w2, [x1,3664]
	cmp	w2, w0
	beq	.L2481
	ldrh	w1, [x1,3712]
	cmp	w1, w0
	bne	.L2477
.L2481:
	adrp	x0, .LANCHOR2
	mov	w1, 1
	add	x24, x0, :lo12:.LANCHOR2
	mov	w2, 0
	ldr	x0, [x24,1000]
	str	w28, [x0,4]
	ldr	x3, [x0,16]
	ldr	x0, [x24,1000]
	str	x3, [x29,168]
	bl	FlashReadPages
	ldr	x0, [x24,1000]
	ldr	x3, [x29,168]
	ldr	w0, [x0]
	ldr	w1, [x3,4]
	cmn	w0, #1
	beq	.L2477
	mov	w0, w22
	bl	ftl_cmp_data_ver
	cbnz	w0, .L2477
	mov	w0, w27
	add	x1, x29, 200
	mov	w2, 1
	bl	log2phys
.L2477:
	ldr	w0, [x29,196]
	cmn	w0, #1
	beq	.L2460
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	add	x2, x20, :lo12:.LANCHOR2
	ubfiz	x3, x1, 1, 16
	ldr	x2, [x2,1192]
	ldrh	w2, [x2,x3]
	cbz	w2, .L2482
.L2576:
	bl	decrement_vpc_count
	b	.L2460
.L2482:
	ldr	x0, [x29,120]
	bl	printk
	b	.L2460
.L2464:
	add	x24, x25, :lo12:.LANCHOR4
	ldrh	w0, [x19]
	mov	w2, w22
	strh	w0, [x24,4074]
	ldr	x0, [x29,128]
	bl	printk
	ldr	w0, [x24,4080]
	cmp	w0, 31
	bhi	.L2483
	add	x1, x24, 4088
	ldr	w2, [x29,204]
	str	w2, [x1,w0,uxtw 2]
	add	w0, w0, 1
	str	w0, [x24,4080]
.L2483:
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	add	x0, x25, :lo12:.LANCHOR4
	ldr	w1, [x0,4028]
	cmn	w1, #1
	beq	.L2577
	cmp	w1, w22
	bls	.L2460
.L2577:
	str	w22, [x0,4028]
.L2460:
	ldr	x0, [x29,160]
	add	x0, x0, 1
	str	x0, [x29,160]
	b	.L2459
.L2585:
	ldrb	w0, [x19,8]
	add	w23, w23, 1
	cmp	w0, 1
	uxth	w23, w23
	bne	.L2486
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L2486
	cmp	w21, w23
	bne	.L2486
	add	x0, x20, :lo12:.LANCHOR2
	ldrh	w0, [x0,826]
	cmp	w0, w23
	beq	.L2462
.L2486:
	add	x0, x20, :lo12:.LANCHOR2
	ldrh	w1, [x0,824]
	cmp	w23, w1
	bne	.L2454
	ldrh	w2, [x0,756]
	mov	w3, 65535
	strh	w23, [x19,2]
	mov	x0, 0
	strh	wzr, [x19,4]
.L2488:
	uxth	w1, w0
	cmp	w1, w2
	bcs	.L2572
	add	x0, x0, 1
	add	x4, x19, x0, lsl 1
	ldrh	w4, [x4,14]
	cmp	w4, w3
	beq	.L2488
	strb	w1, [x19,6]
	b	.L2572
.L2462:
	strb	w26, [x19,6]
	strh	w21, [x19,2]
.L2579:
	mov	x0, x19
	mov	w1, w21
	mov	w2, w26
	bl	ftl_sb_update_avl_pages
.L2572:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 208
	ret
	.size	FtlRecoverySuperblock, .-FtlRecoverySuperblock
	.align	2
	.global	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	ldrh	w1, [x0,4]
	cbz	w1, .L2586
	ldrh	w1, [x0]
	mov	w20, 65535
	cmp	w1, w20
	beq	.L2586
	mov	x19, x0
	ldrb	w0, [x0,6]
	adrp	x21, .LANCHOR0
	adrp	x22, .LANCHOR2
	add	x0, x0, 8
	add	x21, x21, :lo12:.LANCHOR0
	ldrh	w0, [x19,x0,lsl 1]
.L2590:
	cmp	w0, w20
	bne	.L2597
.L2592:
	ldrb	w0, [x19,6]
	add	x1, x22, :lo12:.LANCHOR2
	add	w0, w0, 1
	ldrh	w1, [x1,756]
	uxtb	w0, w0
	strb	w0, [x19,6]
	cmp	w1, w0
	bne	.L2591
	ldrh	w0, [x19,2]
	strb	wzr, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
.L2591:
	ldrb	w0, [x19,6]
	add	x0, x0, 8
	ldrh	w0, [x19,x0,lsl 1]
	b	.L2590
.L2597:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2593
	ldrb	w0, [x21,4044]
	cbnz	w0, .L2593
	add	x0, x21, 648
	ldrh	w2, [x19,2]
	ldrh	w0, [x0,w2,sxtw 1]
	cmp	w0, w20
	bne	.L2593
	ldrh	w0, [x19,4]
	sub	w0, w0, #1
	strh	w0, [x19,4]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	ldrh	w1, [x19,4]
	cbnz	w1, .L2592
	ldrh	w0, [x19,2]
	strb	w1, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
	b	.L2586
.L2593:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L2586
	cmp	w1, 1
	bne	.L2586
	adrp	x0, .LANCHOR2
	ldrh	w2, [x19,2]
	add	x0, x0, :lo12:.LANCHOR2
	ldrh	w1, [x0,826]
	cmp	w2, w1
	bcc	.L2586
	ldrh	w1, [x19]
	ldr	x3, [x0,1192]
	lsl	x1, x1, 1
	ldrh	w4, [x19,4]
	ldrh	w2, [x3,x1]
	sub	w2, w2, w4
	strh	w2, [x3,x1]
	strh	wzr, [x19,4]
	ldrh	w0, [x0,824]
	strh	w0, [x19,2]
	strb	wzr, [x19,6]
.L2586:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlSlcSuperblockCheck, .-FtlSlcSuperblockCheck
	.align	2
	.global	get_new_active_ppa
	.type	get_new_active_ppa, %function
get_new_active_ppa:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	x19, x0
	strb	wzr, [x0,10]
	ldrb	w0, [x0,6]
	adrp	x20, .LANCHOR0
	mov	w22, 65535
	adrp	x23, .LANCHOR2
	add	x0, x0, 8
	add	x20, x20, :lo12:.LANCHOR0
	ldrh	w0, [x19,x0,lsl 1]
.L2599:
	cmp	w0, w22
	adrp	x21, .LANCHOR2
	bne	.L2614
.L2600:
	ldrb	w0, [x19,6]
	add	x1, x23, :lo12:.LANCHOR2
	add	w0, w0, 1
	ldrh	w1, [x1,756]
	uxtb	w0, w0
	strb	w0, [x19,6]
	cmp	w1, w0
	bne	.L2601
	ldrh	w0, [x19,2]
	strb	wzr, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
.L2601:
	ldrb	w0, [x19,6]
	add	x0, x0, 8
	ldrh	w0, [x19,x0,lsl 1]
	b	.L2599
.L2614:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2602
	ldrb	w1, [x20,4044]
	cbnz	w1, .L2602
	add	x1, x20, 648
	ldrh	w2, [x19,2]
	ldrh	w1, [x1,w2,sxtw 1]
	cmp	w1, w22
	bne	.L2602
	ldrh	w0, [x19,4]
	sub	w0, w0, #1
	strh	w0, [x19,4]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L2600
.L2602:
	ldrh	w1, [x19,4]
	mov	w23, 65535
	ldrh	w22, [x19,2]
	adrp	x20, .LANCHOR0
	sub	w1, w1, #1
	mov	w24, w23
	orr	w22, w22, w0, lsl 10
	strh	w1, [x19,4]
	add	x20, x20, :lo12:.LANCHOR0
.L2603:
	add	x0, x21, :lo12:.LANCHOR2
	ldrb	w1, [x19,6]
	ldrh	w2, [x0,756]
.L2605:
	add	w1, w1, 1
	uxtb	w1, w1
	cmp	w1, w2
	bne	.L2604
	ldrh	w1, [x19,2]
	add	w1, w1, 1
	strh	w1, [x19,2]
	mov	w1, 0
.L2604:
	add	x0, x19, x1, sxtw 1
	ldrh	w0, [x0,16]
	cmp	w0, w23
	beq	.L2605
	ldrb	w0, [x19,8]
	strb	w1, [x19,6]
	cmp	w0, 1
	bne	.L2610
	ldrb	w0, [x20,4044]
	cbnz	w0, .L2607
	add	x1, x20, 648
	ldrh	w0, [x19,2]
	ldrh	w0, [x1,w0,sxtw 1]
	cmp	w0, w24
	bne	.L2607
	ldrh	w1, [x19,4]
	cbz	w1, .L2607
	ldrh	w0, [x19]
	sub	w1, w1, #1
	strh	w1, [x19,4]
	bl	decrement_vpc_count
	b	.L2603
.L2607:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrb	w0, [x1,4044]
	cbz	w0, .L2610
	add	x2, x21, :lo12:.LANCHOR2
	ldrh	w1, [x19,2]
	ldrh	w0, [x2,826]
	cmp	w1, w0
	bcc	.L2610
	ldrh	w1, [x19]
	ldr	x0, [x2,1192]
	lsl	x1, x1, 1
	ldrh	w4, [x19,4]
	ldrh	w3, [x0,x1]
	sub	w3, w3, w4
	strh	w3, [x0,x1]
	strh	wzr, [x19,4]
	ldrh	w0, [x2,824]
	strh	w0, [x19,2]
	strb	wzr, [x19,6]
.L2610:
	mov	w0, w22
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.global	FtlWriteDumpData
	.type	FtlWriteDumpData, %function
FtlWriteDumpData:
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	x25, [sp,64]
	add	x20, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	ldr	w0, [x20,1408]
	cbnz	w0, .L2615
	ldrh	w2, [x20,3620]
	cbz	w2, .L2617
	ldrb	w1, [x20,3624]
	cbnz	w1, .L2617
	ldrb	w1, [x20,3623]
	ldrh	w3, [x20,824]
	mul	w1, w1, w3
	cmp	w2, w1
	beq	.L2617
	ldrb	w23, [x20,3626]
	ldr	w22, [x20,1404]
	ldrh	w24, [x20,756]
	cbnz	w23, .L2615
	sub	w22, w22, #1
	add	x1, x29, 84
	mov	w0, w22
	mov	w2, w23
	bl	log2phys
	ldr	x21, [x20,1120]
	ldr	w0, [x29,84]
	ldr	x1, [x20,1056]
	str	w0, [x29,92]
	cmn	w0, #1
	str	w22, [x29,112]
	str	x1, [x29,96]
	str	x21, [x29,104]
	str	w23, [x21,4]
	beq	.L2619
	add	x0, x29, 88
	mov	w1, 1
	mov	w2, w23
	bl	FlashReadPages
	b	.L2620
.L2619:
	ldr	x0, [x20,1056]
	mov	w1, 255
	ldrh	w2, [x20,834]
	bl	ftl_memset
.L2620:
	add	x25, x19, :lo12:.LANCHOR2
	mov	w0, -3947
	lsl	w24, w24, 2
	strh	w0, [x21]
	mov	w23, 0
	add	x20, x25, 3616
.L2621:
	cmp	w23, w24
	beq	.L2622
	ldrh	w0, [x20,4]
	cbz	w0, .L2622
	ldr	w0, [x29,92]
	add	w23, w23, 1
	str	w0, [x21,12]
	ldrh	w0, [x20]
	strh	w0, [x21,2]
	mov	x0, x20
	str	w22, [x21,8]
	bl	get_new_active_ppa
	str	w0, [x29,92]
	ldr	w1, [x25,880]
	mov	w2, 0
	str	w1, [x21,4]
	mov	w3, w2
	add	w1, w1, 1
	add	x0, x29, 88
	cmn	w1, #1
	csel	w1, w1, wzr, ne
	str	w1, [x25,880]
	mov	w1, 1
	bl	FlashProgPages
	ldrh	w0, [x20]
	bl	decrement_vpc_count
	b	.L2621
.L2622:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, 1
	strb	w0, [x19,3626]
	b	.L2615
.L2617:
	add	x19, x19, :lo12:.LANCHOR2
	strb	wzr, [x19,3626]
.L2615:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
	ldp	x29, x30, [sp], 144
	ret
	.size	FtlWriteDumpData, .-FtlWriteDumpData
	.align	2
	.global	l2p_flush
	.type	l2p_flush, %function
l2p_flush:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	w19, 0
	adrp	x20, .LANCHOR2
	bl	FtlWriteDumpData
.L2633:
	add	x1, x20, :lo12:.LANCHOR2
	ldrh	w0, [x1,862]
	cmp	w0, w19
	bls	.L2636
	ldr	x1, [x1,1272]
	ubfiz	x0, x19, 4, 16
	add	x0, x1, x0
	ldr	w0, [x0,4]
	tbz	w0, #31, .L2634
	mov	w0, w19
	bl	flush_l2p_region
.L2634:
	add	w19, w19, 1
	uxth	w19, w19
	b	.L2633
.L2636:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	str	x23, [sp,48]
	add	x1, x20, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	ldr	w1, [x1,1408]
	cbnz	w1, .L2637
	adrp	x2, .LANCHOR0+4044
	ldrb	w21, [x2,#:lo12:.LANCHOR0+4044]
	cbz	w21, .L2652
	ldrb	w21, [x0,8]
	cmp	w21, 1
	bne	.L2648
	ldrh	w23, [x0,4]
	b	.L2639
.L2648:
	mov	w21, w1
.L2652:
	mov	w23, 12
.L2639:
	mov	x19, x0
	mov	w0, -1
	str	w0, [x29,96]
	add	x0, x20, :lo12:.LANCHOR2
	mov	w2, 61589
	ldr	x22, [x0,1120]
	ldr	x1, [x0,1056]
	str	x1, [x29,80]
	mov	w1, -3
	str	x22, [x29,88]
	str	w1, [x22,8]
	mov	w1, -2
	str	w1, [x22,12]
	ldrh	w1, [x19]
	strh	w1, [x22,2]
	strh	wzr, [x22]
	ldr	x1, [x0,1056]
	str	w2, [x1]
	ldr	x1, [x0,1056]
	mov	w0, 22136
	movk	w0, 0x1234, lsl 16
	str	w0, [x1,4]
.L2640:
	sub	w23, w23, #1
	cmn	w23, #1
	beq	.L2643
	ldrh	w0, [x19,4]
	cbnz	w0, .L2641
.L2643:
	add	x20, x20, :lo12:.LANCHOR2
	ldrh	w0, [x19]
	ldrh	w3, [x19,4]
	lsl	x0, x0, 1
	ldr	x2, [x20,1192]
	ldrh	w1, [x2,x0]
	sub	w1, w1, w3
	strh	w1, [x2,x0]
	strb	wzr, [x19,6]
	ldrh	w0, [x20,824]
	strh	w0, [x19,2]
	strh	wzr, [x19,4]
	b	.L2637
.L2641:
	mov	x0, x19
	bl	get_new_active_ppa
	str	w0, [x29,76]
	cmn	w0, #1
	beq	.L2643
	add	x1, x20, :lo12:.LANCHOR2
	mov	w2, w21
	mov	w3, 0
	ldr	w0, [x1,880]
	str	w0, [x22,4]
	add	w0, w0, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x1,880]
	add	x0, x29, 72
	mov	w1, 1
	bl	FlashProgPages
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L2640
.L2637:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldr	x23, [sp,48]
	ldp	x29, x30, [sp], 128
	ret
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	stp	x29, x30, [sp, -80]!
	adrp	x1, .LANCHOR3
	adrp	x0, .LC47
	add	x1, x1, :lo12:.LANCHOR3
	add	x29, sp, 0
	add	x1, x1, 72
	add	x0, x0, :lo12:.LC47
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	adrp	x19, .LANCHOR2
	str	x23, [sp,48]
	bl	printk
	mov	w20, 0
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 0
	mov	x21, x19
	ldrh	w2, [x0,766]
	ldr	x0, [x0,1184]
	lsl	w2, w2, 1
	bl	ftl_memset
.L2654:
	add	x19, x21, :lo12:.LANCHOR2
	ldr	w0, [x19,1404]
	cmp	w20, w0
	bcs	.L2670
	mov	w0, w20
	add	x1, x29, 76
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x29,76]
	cmn	w0, #1
	beq	.L2655
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	ubfiz	x0, x0, 1, 16
	ldr	x2, [x19,1184]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
.L2655:
	add	w20, w20, 1
	b	.L2654
.L2670:
	adrp	x22, .LC150
	mov	w20, 0
	mov	w23, 65535
	add	x22, x22, :lo12:.LC150
.L2657:
	ldrh	w0, [x19,764]
	cmp	w0, w20
	bls	.L2671
	ubfiz	x21, x20, 1, 16
	ldr	x0, [x19,1192]
	ldrh	w2, [x0,x21]
	ldr	x0, [x19,1184]
	ldrh	w3, [x0,x21]
	cmp	w2, w3
	beq	.L2660
	cmp	w2, w23
	beq	.L2660
	mov	x0, x22
	mov	w1, w20
	bl	printk
	ldrh	w0, [x19,3616]
	cmp	w0, w20
	beq	.L2660
	ldrh	w0, [x19,3712]
	cmp	w0, w20
	beq	.L2660
	ldrh	w0, [x19,3664]
	cmp	w0, w20
	beq	.L2660
	ldr	x0, [x19,1192]
	ldrh	w1, [x0,x21]
	cbnz	w1, .L2662
	ldr	x1, [x19,1184]
	ldrh	w1, [x1,x21]
	strh	w1, [x0,x21]
	b	.L2660
.L2662:
	ldr	x1, [x19,1184]
	ldrh	w1, [x1,x21]
	strh	w1, [x0,x21]
	mov	w0, w20
	bl	update_vpc_list
.L2660:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L2657
.L2671:
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlVpcCheckAndModify, .-FtlVpcCheckAndModify
	.align	2
	.global	allocate_new_data_superblock
	.type	allocate_new_data_superblock, %function
allocate_new_data_superblock:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x2, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	ldr	w1, [x2,1408]
	ldrh	w22, [x0]
	cbnz	w1, .L2673
	mov	x21, x0
	mov	w0, 65535
	cmp	w22, w0
	beq	.L2674
	ubfiz	x1, x22, 1, 16
	ldr	x0, [x2,1192]
	ldrh	w0, [x0,x1]
	cbz	w0, .L2675
	mov	w0, w22
	bl	INSERT_DATA_LIST
	b	.L2674
.L2675:
	mov	w0, w22
	bl	INSERT_FREE_LIST
.L2674:
	add	x1, x19, :lo12:.LANCHOR2
	strb	wzr, [x21,8]
	add	x0, x1, 3664
	cmp	x21, x0
	beq	.L2676
	ldrh	w2, [x1,776]
	cmp	w2, 1
	beq	.L2676
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cbz	w0, .L2677
.L2676:
	mov	w0, 1
	strb	w0, [x21,8]
	b	.L2678
.L2677:
	add	x0, x1, 3616
	cmp	x21, x0
	bne	.L2678
	cmp	w2, 3
	beq	.L2680
	ldr	w0, [x1,872]
	cmp	w0, 1
	bne	.L2681
.L2680:
	add	x1, x19, :lo12:.LANCHOR2
	mov	w0, 1
	strb	w0, [x1,3624]
.L2681:
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w0, [x1,660]
	cbz	w0, .L2678
	ldr	w0, [x1,924]
	cmp	w0, 39
	bhi	.L2678
	mov	w0, 1
	strb	w0, [x1,3624]
.L2678:
	adrp	x1, .LANCHOR4
	add	x0, x1, :lo12:.LANCHOR4
	mov	x20, x1
	ldrh	w2, [x0,4016]
	mov	w0, 65535
	cmp	w2, w0
	beq	.L2683
	cmp	w22, w2
	bne	.L2684
	add	x19, x19, :lo12:.LANCHOR2
	ubfiz	x1, x2, 1, 16
	ldr	x0, [x19,1192]
	ldrh	w0, [x0,x1]
	cbz	w0, .L2685
.L2684:
	mov	w0, w2
	bl	update_vpc_list
.L2685:
	add	x1, x20, :lo12:.LANCHOR4
	mov	w0, -1
	strh	w0, [x1,4016]
.L2683:
	mov	x0, x21
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	w0, 0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2673:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	stp	x29, x30, [sp, -112]!
	adrp	x1, .LANCHOR4
	add	x2, x1, :lo12:.LANCHOR4
	add	x29, sp, 0
	add	x0, x2, 3472
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	adrp	x19, .LANCHOR2
	ldr	w3, [x0,80]
	cbz	w3, .L2699
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w4, [x0,84]
	ldr	w3, [x1,1404]
	cmp	w4, w3
	bcs	.L2700
	mov	w21, 2048
	mov	x20, x2
.L2705:
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w0, [x20,3556]
	ldr	w1, [x1,1404]
	cmp	w0, w1
	bcs	.L2702
	add	x1, x29, 52
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x20,3556]
	ldr	w1, [x29,52]
	add	w0, w0, 1
	str	w0, [x20,3556]
	cmn	w1, #1
	beq	.L2703
	str	w1, [x29,60]
	mov	w2, 0
	str	w0, [x29,80]
	mov	w1, 1
	add	x0, x29, 56
	str	xzr, [x29,64]
	str	xzr, [x29,72]
	str	wzr, [x29,56]
	bl	FlashReadPages
	ldr	w0, [x29,56]
	cmp	w0, 256
	bne	.L2702
	ldr	w0, [x29,52]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L2702
.L2703:
	subs	w21, w21, #1
	bne	.L2705
.L2702:
	mov	w0, -1
	b	.L2707
.L2700:
	ldr	w1, [x1,904]
	str	wzr, [x0,80]
	str	wzr, [x0,84]
	str	w1, [x0,76]
	b	.L2713
.L2699:
	add	x2, x19, :lo12:.LANCHOR2
	ldr	w0, [x2,920]
	ldr	w3, [x2,660]
	adrp	x2, .LANCHOR0+4044
	ldrb	w4, [x2,#:lo12:.LANCHOR0+4044]
	mov	w2, 4194304
	cbnz	w4, .L2708
	add	w0, w3, w0, lsr 10
	mov	w3, 33554432
	asr	w2, w3, w0
.L2708:
	add	x19, x19, :lo12:.LANCHOR2
	add	x0, x1, :lo12:.LANCHOR4
	ldr	w3, [x19,904]
	ldr	w0, [x0,3548]
	add	w4, w3, 1048576
	cmp	w0, w4
	bhi	.L2709
	add	w0, w2, w0
	cmp	w0, w3
	bcc	.L2709
	ldrb	w2, [x19,3556]
	mov	w0, 0
	cbnz	w2, .L2707
.L2709:
	add	x0, x1, :lo12:.LANCHOR4
	mov	w1, 1
	str	wzr, [x0,3556]
	str	w1, [x0,3552]
	str	w3, [x0,3548]
.L2713:
	mov	w0, 0
.L2707:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	ftl_do_gc
	.type	ftl_do_gc, %function
ftl_do_gc:
	stp	x29, x30, [sp, -160]!
	mov	w2, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x3, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	ldr	w4, [x3,1408]
	cbnz	w4, .L2859
	ldr	w4, [x3,928]
	cbnz	w4, .L2859
	ldrh	w4, [x3,3592]
	cmp	w4, 47
	bls	.L2859
	str	w0, [x29,140]
	adrp	x0, .LANCHOR1+3292
	mov	w24, w1
	ldrh	w1, [x0,#:lo12:.LANCHOR1+3292]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L2716
.L2719:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w4, [x0,954]
	cmp	w4, w2
	bne	.L2717
	b	.L2718
.L2716:
	ldrh	w1, [x3,3712]
	cmp	w1, w0
	beq	.L2719
	mov	w0, 1
	bl	FtlGcFreeTempBlock
	mov	w2, 1
	cbz	w0, .L2719
	b	.L2859
.L2717:
	ldrh	w1, [x0,952]
	cmp	w1, w2
	bne	.L2718
	ldrh	w3, [x0,956]
	cmp	w3, w1
	beq	.L2718
	ldrh	w2, [x0,958]
	cmp	w2, w1
	beq	.L2718
	mov	w1, -1
	strh	w4, [x0,952]
	strh	w3, [x0,954]
	strh	w2, [x0,956]
	strh	w1, [x0,958]
.L2718:
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w2, [x29,140]
	adrp	x23, .LANCHOR4
	cmp	w2, 1
	ldr	w0, [x1,944]
	add	w0, w0, 1
	add	w0, w0, w2, lsl 7
	str	w0, [x1,944]
	bne	.L2730
	ldr	w1, [x1,660]
	cbnz	w1, .L2721
	adrp	x1, .LANCHOR0+4044
	ldrb	w1, [x1,#:lo12:.LANCHOR0+4044]
	cbz	w1, .L2730
.L2721:
	add	x20, x19, :lo12:.LANCHOR2
	ldr	w1, [x20,924]
	cmp	w1, 39
	bhi	.L2730
	adrp	x21, .LANCHOR5
	add	x1, x21, :lo12:.LANCHOR5
	ldrh	w1, [x1,-136]
	add	w0, w0, w1
	str	w0, [x20,944]
	bl	FtlGcReFreshBadBlk
	add	x0, x23, :lo12:.LANCHOR4
	mov	w1, 65535
	ldrh	w0, [x0,3424]
	cmp	w0, w1
	bne	.L2730
	ldrh	w1, [x20,952]
	cmp	w1, w0
	bne	.L2730
	ldr	w0, [x20,944]
	cmp	w0, 1024
	bhi	.L2722
	ldrh	w0, [x20,3608]
	cmp	w0, 63
	bhi	.L2730
.L2722:
	add	x0, x19, :lo12:.LANCHOR2
	add	x1, x21, :lo12:.LANCHOR5
	ldrh	w2, [x0,962]
	ldrh	w3, [x0,3608]
	add	w2, w2, 64
	strh	wzr, [x1,-136]
	cmp	w3, w2
	bgt	.L2730
	str	wzr, [x0,944]
	ldr	w0, [x0,924]
	cbnz	w0, .L2723
	mov	w0, 6
	b	.L2881
.L2723:
	cmp	w0, 5
	bhi	.L2724
	mov	w0, 18
.L2881:
	strh	w0, [x1,-136]
.L2724:
	mov	w0, 32
	mov	w28, 65535
	bl	List_get_gc_head_node
	uxth	w26, w0
	cmp	w26, w28
	beq	.L2729
	add	x22, x19, :lo12:.LANCHOR2
	ldrh	w0, [x22,964]
	cbz	w0, .L2726
	ldrh	w1, [x22,826]
	ubfiz	x26, x26, 1, 16
	ldrh	w3, [x22,756]
	ldr	x27, [x22,1192]
	mul	w1, w1, w3
	ldrh	w2, [x27,x26]
	add	w1, w1, 1
	cmp	w2, w1
	bgt	.L2729
	add	w1, w0, 1
	str	wzr, [x22,932]
	uxth	w1, w1
	strh	w1, [x22,964]
	str	x1, [x29,128]
	bl	List_get_gc_head_node
	uxth	w20, w0
	cmp	w20, w28
	ldr	x1, [x29,128]
	beq	.L2729
	ubfiz	x28, x20, 1, 16
	adrp	x0, .LC151
	ldrh	w4, [x27,x26]
	add	x0, x0, :lo12:.LC151
	mov	w2, w20
	ldrh	w3, [x27,x28]
	bl	printk
	ldrh	w0, [x22,964]
	cmp	w0, 40
	bls	.L2727
	ldr	x0, [x22,1192]
	ldrh	w0, [x0,x28]
	cmp	w0, 32
	bls	.L2727
	strh	wzr, [x22,964]
.L2727:
	add	x21, x21, :lo12:.LANCHOR5
	mov	w0, 6
	strh	w0, [x21,-136]
	b	.L2731
.L2726:
	mov	w0, 1
	strh	w0, [x22,964]
.L2729:
	bl	GetSwlReplaceBlock
	uxth	w20, w0
	mov	w0, 65535
	cmp	w20, w0
	bne	.L2731
	add	x21, x21, :lo12:.LANCHOR5
	strh	wzr, [x21,-136]
.L2730:
	add	x0, x23, :lo12:.LANCHOR4
	ldrh	w20, [x0,3424]
	mov	w0, 65535
	cmp	w20, w0
	bne	.L2886
	add	x22, x19, :lo12:.LANCHOR2
	ldrh	w0, [x22,3712]
	cmp	w0, w20
	beq	.L2887
.L2731:
	mov	w0, 65535
	cmp	w20, w0
	cset	w2, eq
	cbz	w2, .L2744
	ldr	w0, [x29,140]
	cbnz	w0, .L2744
	add	x0, x19, :lo12:.LANCHOR2
	mov	w22, 1
	ldrh	w1, [x0,3608]
	cmp	w1, 24
	bhi	.L2745
	cmp	w1, 16
	ldrh	w22, [x0,824]
	bls	.L2746
	lsr	w22, w22, 5
	b	.L2745
.L2746:
	cmp	w1, 12
	bls	.L2747
	lsr	w22, w22, 4
	b	.L2745
.L2747:
	lsr	w0, w22, 2
	cmp	w1, 9
	csel	w22, w0, w22, cs
.L2745:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w2, [x0,960]
	cmp	w2, w1
	bcs	.L2749
	ldrh	w1, [x0,3712]
	mov	w2, 65535
	cmp	w1, w2
	bne	.L2750
	ldrh	w2, [x0,952]
	cmp	w2, w1
	bne	.L2750
	adrp	x1, .LANCHOR5-136
	ldrh	w2, [x1,#:lo12:.LANCHOR5-136]
	cbnz	w2, .L2751
	ldr	w1, [x0,1404]
	ldr	w3, [x0,3900]
	add	w1, w1, w1, lsl 1
	cmp	w3, w1, lsr 2
	bcs	.L2752
.L2751:
	add	x23, x23, :lo12:.LANCHOR4
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x23,3420]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1,960]
	b	.L2753
.L2752:
	mov	w1, 18
	strh	w1, [x0,960]
.L2753:
	add	x19, x19, :lo12:.LANCHOR2
	str	wzr, [x19,932]
	b	.L2859
.L2750:
	add	x0, x23, :lo12:.LANCHOR4
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,3420]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1,960]
.L2749:
	cmp	w24, 2
	bhi	.L2814
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,660]
	cbz	w0, .L2814
	add	w22, w22, 1
	uxth	w22, w22
	b	.L2814
.L2744:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w3, 65535
	ldrh	w1, [x0,3712]
	cmp	w1, w3
	bne	.L2756
	cbz	w2, .L2756
	ldrh	w2, [x0,952]
	cmp	w2, w1
	bne	.L2756
	add	x2, x23, :lo12:.LANCHOR4
	ldrh	w2, [x2,3424]
	cmp	w2, w1
	beq	.L2757
.L2762:
	mov	w20, 65535
	b	.L2756
.L2757:
	ldrh	w2, [x0,3608]
	adrp	x20, .LANCHOR5
	ldrh	w1, [x0,960]
	str	wzr, [x0,932]
	cmp	w2, w1
	bls	.L2758
	add	x1, x20, :lo12:.LANCHOR5
	ldrh	w1, [x1,-136]
	cbnz	w1, .L2759
	ldr	w1, [x0,1404]
	ldr	w2, [x0,3900]
	add	w1, w1, w1, lsl 1
	cmp	w2, w1, lsr 2
	bcs	.L2760
.L2759:
	add	x0, x23, :lo12:.LANCHOR4
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,3420]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1,960]
	b	.L2761
.L2760:
	mov	w1, 18
	strh	w1, [x0,960]
.L2761:
	bl	FtlReadRefresh
	mov	w0, 0
	bl	List_get_gc_head_node
	ubfiz	x0, x0, 1, 16
	add	x1, x19, :lo12:.LANCHOR2
	ldr	x1, [x1,1192]
	ldrh	w0, [x1,x0]
	cmp	w0, 4
	bls	.L2758
	add	x20, x20, :lo12:.LANCHOR5
	ldrh	w2, [x20,-136]
	b	.L2859
.L2758:
	add	x22, x20, :lo12:.LANCHOR5
	ldrh	w0, [x22,-136]
	cbnz	w0, .L2762
	add	x1, x23, :lo12:.LANCHOR4
	add	x21, x19, :lo12:.LANCHOR2
	ldrh	w20, [x1,3420]
	add	w1, w20, w20, lsl 1
	asr	w1, w1, 2
	strh	w1, [x21,960]
	bl	List_get_gc_head_node
	ubfiz	x0, x0, 1, 16
	ldr	x1, [x21,1192]
	ldrh	w2, [x21,756]
	ldrh	w1, [x1,x0]
	ldrh	w0, [x21,826]
	mul	w0, w0, w2
	mov	w2, 2
	sdiv	w0, w0, w2
	cmp	w1, w0
	ble	.L2763
	ldrh	w0, [x21,3608]
	sub	w20, w20, #1
	cmp	w0, w20
	blt	.L2763
	bl	FtlReadRefresh
	ldrh	w2, [x22,-136]
	b	.L2859
.L2763:
	cbnz	w1, .L2762
	mov	w0, -1
	add	x19, x19, :lo12:.LANCHOR2
	bl	decrement_vpc_count
	ldrh	w2, [x19,3608]
	add	w2, w2, 1
	b	.L2859
.L2756:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w22, 2
	ldr	w0, [x0,660]
	cmp	w0, wzr
	csinc	w22, w22, wzr, ne
	b	.L2755
.L2814:
	mov	w20, 65535
.L2755:
	add	x0, x23, :lo12:.LANCHOR4
	mov	w2, 65535
	ldrh	w1, [x0,3424]
	cmp	w1, w2
	bne	.L2765
	cmp	w20, w1
	beq	.L2766
	strh	w20, [x0,3424]
	b	.L2767
.L2766:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,952]
	cmp	w0, w20
	beq	.L2767
	ubfiz	x0, x0, 1, 16
	ldr	x2, [x1,1192]
	ldrh	w0, [x2,x0]
	cbnz	w0, .L2768
	mov	w0, -1
	strh	w0, [x1,952]
.L2768:
	add	x1, x19, :lo12:.LANCHOR2
	add	x0, x23, :lo12:.LANCHOR4
	ldrh	w2, [x1,952]
	strh	w2, [x0,3424]
	mov	w0, -1
	strh	w0, [x1,952]
.L2767:
	add	x21, x23, :lo12:.LANCHOR4
	mov	w1, 65535
	ldrh	w0, [x21,3424]
	strb	wzr, [x21,3432]
	cmp	w0, w1
	beq	.L2765
	bl	IsBlkInGcList
	cbz	w0, .L2770
	mov	w0, -1
	strh	w0, [x21,3424]
.L2770:
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cbz	w0, .L2771
	add	x0, x23, :lo12:.LANCHOR4
	add	x21, x0, 3424
	ldrh	w0, [x0,3424]
	bl	ftl_get_blk_mode
	strb	w0, [x21,8]
.L2771:
	add	x21, x23, :lo12:.LANCHOR4
	mov	w0, 65535
	add	x24, x21, 3424
	ldrh	w1, [x21,3424]
	cmp	w1, w0
	beq	.L2765
	mov	x0, x24
	bl	make_superblock
	strh	wzr, [x21,3426]
	adrp	x0, .LANCHOR5
	add	x1, x19, :lo12:.LANCHOR2
	add	x0, x0, :lo12:.LANCHOR5
	ldrh	w2, [x21,3424]
	strb	wzr, [x21,3430]
	ldr	x1, [x1,1192]
	strh	wzr, [x0,-134]
	ldrh	w1, [x1,x2,lsl 1]
	strh	w1, [x0,-132]
.L2765:
	add	x21, x23, :lo12:.LANCHOR4
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x21,3424]
	ldrh	w2, [x1,3616]
	cmp	w2, w0
	beq	.L2772
	ldrh	w1, [x1,3664]
	cmp	w1, w0
	beq	.L2772
.L2773:
	adrp	x0, .LC153
	add	x0, x0, :lo12:.LC153
	str	x0, [x29,128]
	b	.L2774
.L2772:
	add	x23, x23, :lo12:.LANCHOR4
	mov	w0, -1
	strh	w0, [x23,3424]
	b	.L2885
.L2795:
	ldrh	w0, [x21,3426]
	add	w22, w22, w0
	uxth	w22, w22
	strh	w22, [x21,3426]
	cmp	w22, w26
	bcs	.L2888
.L2797:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,3608]
	cmp	w0, 2
	bhi	.L2800
	ldrh	w22, [x1,824]
.L2774:
	ldrh	w26, [x21,3424]
	mov	w0, 65535
	cmp	w26, w0
	bne	.L2775
	add	x0, x19, :lo12:.LANCHOR2
	mov	w27, 2
	str	wzr, [x0,932]
.L2776:
	add	x28, x19, :lo12:.LANCHOR2
	ldrh	w24, [x28,964]
	mov	w0, w24
	bl	List_get_gc_head_node
	uxth	w25, w0
	cmp	w25, w26
	strh	w25, [x21,3424]
	bne	.L2777
	strh	wzr, [x28,964]
	mov	w2, 8
	b	.L2859
.L2777:
	mov	w0, w25
	add	w24, w24, 1
	bl	IsBlkInGcList
	cbz	w0, .L2778
	strh	w24, [x28,964]
	b	.L2776
.L2778:
	ldrh	w5, [x28,756]
	ubfiz	x2, x25, 1, 16
	ldrh	w0, [x28,824]
	uxth	w24, w24
	ldr	x3, [x28,1192]
	strh	w24, [x28,964]
	mul	w0, w0, w5
	ldrh	w4, [x3,x2]
	sdiv	w5, w0, w27
	cmp	w4, w5
	bgt	.L2780
	cmp	w4, 8
	bls	.L2781
	cmp	w24, 48
	bls	.L2781
	ldrh	w4, [x21,4032]
	cmp	w4, 35
	bhi	.L2781
.L2780:
	add	x4, x19, :lo12:.LANCHOR2
	strh	wzr, [x4,964]
.L2781:
	cmp	w20, w26
	ldrh	w2, [x3,x2]
	bne	.L2782
	cmp	w2, w0
	blt	.L2782
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w3, [x0,964]
	cmp	w3, 3
	bhi	.L2782
	add	x23, x23, :lo12:.LANCHOR4
	mov	w1, -1
	strh	wzr, [x0,964]
	strh	w1, [x23,3424]
.L2885:
	adrp	x0, .LANCHOR5-136
	ldrh	w2, [x0,#:lo12:.LANCHOR5-136]
	b	.L2859
.L2782:
	cbnz	w2, .L2783
	mov	w0, -1
	bl	decrement_vpc_count
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,964]
	add	w0, w0, 1
	strh	w0, [x1,964]
	b	.L2776
.L2783:
	adrp	x0, .LANCHOR0
	strb	wzr, [x21,3432]
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L2784
	mov	w0, w25
	bl	ftl_get_blk_mode
	strb	w0, [x21,3432]
.L2784:
	add	x24, x21, 3424
	mov	x0, x24
	bl	make_superblock
	adrp	x1, .LANCHOR5
	add	x0, x19, :lo12:.LANCHOR2
	add	x1, x1, :lo12:.LANCHOR5
	ldrh	w2, [x21,3424]
	ldr	x0, [x0,1192]
	strh	wzr, [x1,-134]
	ldrh	w0, [x0,x2,lsl 1]
	strh	w0, [x1,-132]
	strh	wzr, [x21,3426]
	strb	wzr, [x21,3430]
.L2775:
	ldr	w0, [x29,140]
	cmp	w0, 1
	bne	.L2785
	bl	FtlReadRefresh
.L2785:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 1
	str	w1, [x0,928]
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w26, [x0,824]
	ldrb	w1, [x1,4044]
	cbz	w1, .L2786
	ldrb	w1, [x21,3432]
	cmp	w1, 1
	bne	.L2786
	ldrh	w26, [x0,826]
.L2786:
	ldrh	w0, [x21,3426]
	add	w1, w0, w22
	cmp	w1, w26
	ble	.L2787
	sub	w22, w26, w0
	uxth	w22, w22
.L2787:
	mov	w27, 0
.L2788:
	cmp	w22, w27, uxth
	bls	.L2795
	add	x1, x21, 3424
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w3, [x1,2]
	ldrh	w6, [x0,756]
	mov	x0, 0
	add	w3, w3, w27
	mov	w24, w0
.L2796:
	cmp	w6, w0, uxth
	bls	.L2889
	add	x2, x1, x0, lsl 1
	mov	w4, 65535
	ldrh	w2, [x2,16]
	cmp	w2, w4
	beq	.L2789
	add	x5, x19, :lo12:.LANCHOR2
	mov	w4, 56
	orr	w2, w3, w2, lsl 10
	umull	x4, w24, w4
	ldr	x5, [x5,1032]
	add	w24, w24, 1
	add	x4, x5, x4
	uxth	w24, w24
	str	w2, [x4,4]
.L2789:
	add	x0, x0, 1
	b	.L2796
.L2889:
	add	x25, x19, :lo12:.LANCHOR2
	ldrb	w2, [x21,3432]
	mov	w1, w24
	mov	x28, 0
	ldr	x0, [x25,1032]
	bl	FlashReadPages
.L2791:
	cmp	w24, w28, uxth
	bls	.L2890
	mov	x0, 56
	mul	x5, x28, x0
	ldr	x0, [x25,1032]
	add	x1, x0, x5
	ldr	w0, [x0,x5]
	cmn	w0, #1
	ldr	x4, [x1,16]
	beq	.L2816
	ldrh	w0, [x4]
	mov	w1, 61589
	cmp	w0, w1
	bne	.L2816
	ldr	w0, [x4,8]
	add	x1, x29, 156
	mov	w2, 0
	str	x5, [x29,112]
	str	x4, [x29,120]
	bl	log2phys
	ldr	x2, [x25,1032]
	ldr	x5, [x29,112]
	ldr	w0, [x29,156]
	add	x2, x2, x5
	ldr	x4, [x29,120]
	and	w0, w0, 2147483647
	ldr	w1, [x2,4]
	cmp	w0, w1
	bne	.L2816
	adrp	x0, .LANCHOR5
	mov	x3, 56
	add	x1, x0, :lo12:.LANCHOR5
	str	x5, [x29,104]
	str	x4, [x29,112]
	ldrh	w0, [x1,-134]
	add	w0, w0, 1
	strh	w0, [x1,-134]
	ldr	w0, [x21,4000]
	ldr	x1, [x25,1008]
	madd	x1, x0, x3, x1
	ldr	w0, [x2,24]
	str	x1, [x29,120]
	str	w0, [x1,24]
	bl	Ftl_get_new_temp_ppa
	ldr	x1, [x29,120]
	mov	x2, 56
	ldr	x5, [x29,104]
	ldr	x4, [x29,112]
	str	w0, [x1,4]
	ldr	w0, [x21,4000]
	ldr	x1, [x25,1008]
	madd	x0, x0, x2, x1
	ldr	x1, [x25,1032]
	add	x1, x1, x5
	ldr	x2, [x1,8]
	str	x2, [x0,8]
	add	x2, x25, 3712
	str	x2, [x29,120]
	ldr	x1, [x1,16]
	str	x1, [x0,16]
	mov	w1, 1
	ldr	w0, [x29,156]
	str	w0, [x4,12]
	ldrh	w0, [x25,3712]
	strh	w0, [x4,2]
	ldr	w0, [x25,880]
	str	w0, [x4,4]
	ldr	w0, [x21,4000]
	add	w0, w0, 1
	str	w0, [x21,4000]
	ldr	x0, [x25,1032]
	add	x0, x0, x5
	bl	FtlGcBufAlloc
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbnz	w0, .L2793
	ldr	x2, [x29,120]
	ldr	w1, [x21,4000]
	ldrb	w0, [x2,7]
	cmp	w1, w0
	beq	.L2793
	ldrh	w0, [x2,4]
	cbnz	w0, .L2816
.L2793:
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L2816
	adrp	x0, .LANCHOR5
	add	x19, x19, :lo12:.LANCHOR2
	add	x0, x0, :lo12:.LANCHOR5
	str	wzr, [x19,928]
	ldrh	w2, [x0,-136]
	b	.L2859
.L2816:
	add	x28, x28, 1
	b	.L2791
.L2890:
	add	w27, w27, 1
	b	.L2788
.L2888:
	ldr	w0, [x21,4000]
	adrp	x22, .LANCHOR5
	cbz	w0, .L2798
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L2798
	add	x19, x19, :lo12:.LANCHOR2
	add	x5, x22, :lo12:.LANCHOR5
	str	wzr, [x19,928]
	ldrh	w2, [x5,-136]
	b	.L2859
.L2798:
	add	x5, x22, :lo12:.LANCHOR5
	ldrh	w22, [x5,-134]
	cbnz	w22, .L2799
	add	x24, x19, :lo12:.LANCHOR2
	ldrh	w1, [x21,3424]
	ubfiz	x0, x1, 1, 16
	ldr	x3, [x24,1192]
	ldrh	w4, [x3,x0]
	cbz	w4, .L2799
	ldrh	w2, [x21,3426]
	mov	w3, w22
	ldrh	w5, [x5,-132]
	ldr	x0, [x29,128]
	bl	printk
	ldrh	w1, [x21,3424]
	ldr	x0, [x24,1192]
	strh	w22, [x0,x1,lsl 1]
	ldrh	w0, [x21,3424]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2799:
	mov	w0, -1
	strh	w0, [x21,3424]
	b	.L2797
.L2800:
	str	wzr, [x1,928]
	adrp	x1, .LANCHOR5-136
	ldrh	w2, [x1,#:lo12:.LANCHOR5-136]
	cmp	w2, wzr
	csinc	w2, w2, w0, ne
	b	.L2859
.L2887:
	ldrh	w26, [x22,952]
	cmp	w26, w20
	bne	.L2731
	ldrh	w0, [x22,3608]
	mov	w2, 1024
	ldr	w1, [x22,944]
	cmp	w0, 24
	mov	w0, 5120
	csel	w0, w0, w2, cc
	cmp	w1, w0
	bls	.L2731
	adrp	x0, .LANCHOR5-136
	str	wzr, [x22,944]
	strh	wzr, [x0,#:lo12:.LANCHOR5-136]
	bl	GetSwlReplaceBlock
	uxth	w20, w0
	cmp	w20, w26
	bne	.L2733
	ldrh	w1, [x22,3608]
	ldrh	w0, [x22,962]
	cmp	w1, w0
	bcs	.L2734
	mov	w0, 64
	bl	List_get_gc_head_node
	uxth	w0, w0
	cmp	w0, w20
	beq	.L2743
	ldr	w2, [x22,868]
	uxtw	x1, w0
	cbnz	w2, .L2736
	ldrh	w0, [x22,776]
	cmp	w0, 3
	beq	.L2736
	ldr	w0, [x22,872]
	cbnz	w0, .L2736
	ldr	w0, [x22,660]
	cbnz	w0, .L2736
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cbz	w0, .L2737
.L2736:
	add	x2, x19, :lo12:.LANCHOR2
	ldr	x0, [x2,1192]
	ldrh	w4, [x0,x1,lsl 1]
	ldrh	w1, [x2,826]
	ldrh	w0, [x2,756]
	ldrh	w2, [x2,776]
	mul	w3, w1, w0
	cmp	w2, 3
	mov	w0, 0
	lsr	w1, w1, 1
	csel	w0, w1, w0, eq
	add	w0, w3, w0
	cmp	w4, w0
	bgt	.L2739
	mov	w0, 0
	bl	List_get_gc_head_node
	uxth	w20, w0
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w1, [x0,1404]
	ldr	w2, [x0,3900]
	add	w1, w1, w1, lsl 1
	cmp	w2, w1, lsr 2
	bhi	.L2883
	mov	w1, 160
	b	.L2882
.L2739:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 128
	b	.L2884
.L2737:
	ldr	x2, [x22,1192]
	ldrh	w1, [x2,x1,lsl 1]
	cmp	w1, 7
	bhi	.L2742
	bl	List_get_gc_head_node
	uxth	w20, w0
	add	x0, x19, :lo12:.LANCHOR2
.L2883:
	mov	w1, 128
.L2882:
	strh	w1, [x0,962]
	mov	w0, 65535
	cmp	w20, w0
	beq	.L2743
	b	.L2733
.L2742:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 64
.L2884:
	strh	w1, [x0,962]
	b	.L2743
.L2734:
	mov	w0, 80
	strh	w0, [x22,962]
	b	.L2743
.L2733:
	add	x5, x19, :lo12:.LANCHOR2
	ubfiz	x4, x20, 1, 32
	adrp	x0, .LC152
	mov	w1, w20
	add	x0, x0, :lo12:.LC152
	ldr	x3, [x5,1192]
	ldr	x6, [x5,1176]
	ldrh	w2, [x5,3608]
	ldrh	w3, [x3,x4]
	ldrh	w5, [x5,960]
	ldrh	w4, [x6,x4]
	bl	printk
.L2743:
	bl	FtlGcReFreshBadBlk
	b	.L2731
.L2886:
	mov	w20, w0
	b	.L2731
.L2859:
	mov	w0, w2
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 160
	ret
	.size	ftl_do_gc, .-ftl_do_gc
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	stp	x29, x30, [sp, -112]!
	adrp	x0, .LANCHOR5-128
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	ldr	x20, [x0,#:lo12:.LANCHOR5-128]
	add	x0, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	ldr	w22, [x0,1408]
	cbnz	w22, .L2893
	ldr	w1, [x0,976]
	cbz	w1, .L2893
	adrp	x0, .LANCHOR0+4044
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	cbz	w0, .L2895
	ldrb	w0, [x20,8]
	cmp	w0, 1
	cset	w22, eq
.L2895:
	add	x0, x19, :lo12:.LANCHOR2
	ldrb	w3, [x20,9]
	adrp	x24, .LC154
	mov	w2, w22
	mov	w23, 0
	mov	w25, 56
	ldr	x0, [x0,1040]
	add	x24, x24, :lo12:.LC154
	bl	FlashProgPages
.L2896:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w1, [x0,976]
	cmp	w23, w1
	bcs	.L2914
	umull	x21, w23, w25
	ldr	x1, [x0,1040]
	add	x0, x1, x21
	ldr	w1, [x1,x21]
	cmn	w1, #1
	beq	.L2917
	ldr	w1, [x0,4]
	cbz	w22, .L2931
	orr	w1, w1, -2147483648
.L2931:
	ldr	w0, [x0,24]
	mov	w2, 1
	str	w1, [x29,108]
	add	x26, x19, :lo12:.LANCHOR2
	add	x1, x29, 108
	bl	log2phys
	ldr	x0, [x26,1040]
	add	x21, x0, x21
	ldr	x0, [x21,16]
	ldr	w0, [x0,12]
	cmn	w0, #1
	beq	.L2900
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	ubfiz	x0, x1, 1, 16
	ldr	x2, [x26,1192]
	mov	w21, w1
	ldrh	w2, [x2,x0]
	cbnz	w2, .L2901
	mov	x0, x24
	bl	printk
.L2901:
	mov	w0, w21
	bl	decrement_vpc_count
.L2900:
	add	w23, w23, 1
	b	.L2896
.L2933:
	mov	w20, 16386
.L2913:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,966]
	cbz	w0, .L2914
	mov	w0, 1
	mov	w1, w0
	bl	ftl_do_gc
	subs	w20, w20, #1
	bne	.L2913
.L2914:
	add	x19, x19, :lo12:.LANCHOR2
	str	wzr, [x19,976]
	b	.L2893
.L2917:
	adrp	x25, .LANCHOR4
	adrp	x26, .LC154
	mov	w24, 0
	mov	w27, 56
	add	x25, x25, :lo12:.LANCHOR4
	add	x26, x26, :lo12:.LC154
.L2897:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w1, [x0,976]
	cmp	w24, w1
	bcs	.L2933
	umull	x21, w24, w27
	ldr	x0, [x0,1040]
	mov	w1, -1
	mov	w28, 1
	str	w1, [x0,x21]
.L2903:
	add	x23, x19, :lo12:.LANCHOR2
	ldr	x0, [x23,1040]
	add	x1, x0, x21
	ldr	w0, [x0,x21]
	cmn	w0, #1
	ldr	w0, [x1,4]
	bne	.L2934
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	ldrh	w1, [x20]
	cmp	w1, w0, uxth
	bne	.L2904
	ldr	x2, [x23,1192]
	ubfiz	x1, x1, 1, 16
	ldrh	w3, [x20,4]
	ldrh	w0, [x2,x1]
	sub	w0, w0, w3
	strh	w0, [x2,x1]
	strb	wzr, [x20,6]
	ldrh	w0, [x23,824]
	strh	w0, [x20,2]
	strh	wzr, [x20,4]
.L2904:
	ldrh	w0, [x20,4]
	cbnz	w0, .L2905
	mov	x0, x20
	bl	allocate_new_data_superblock
.L2905:
	add	x23, x19, :lo12:.LANCHOR2
	ldr	w0, [x25,3568]
	add	w0, w0, 1
	str	w0, [x25,3568]
	ldr	x0, [x23,1040]
	add	x0, x0, x21
	ldr	w0, [x0,4]
	lsr	x0, x0, 10
	bl	FtlGcMarkBadPhyBlk
	mov	x0, x20
	bl	get_new_active_ppa
	str	w0, [x29,108]
	ldr	x1, [x23,1040]
	mov	w2, w22
	add	x1, x1, x21
	str	w0, [x1,4]
	mov	w1, 1
	ldr	x0, [x23,1040]
	ldrb	w3, [x20,9]
	add	x0, x0, x21
	bl	FlashProgPages
	ldr	x0, [x23,1040]
	ldr	w0, [x0,x21]
	cmn	w0, #1
	bne	.L2906
	str	w28, [x23,1408]
.L2906:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,1408]
	cbz	w0, .L2903
	b	.L2893
.L2934:
	cbz	w22, .L2932
	orr	w0, w0, -2147483648
.L2932:
	str	w0, [x29,108]
	mov	w2, 1
	ldr	w0, [x1,24]
	add	x23, x19, :lo12:.LANCHOR2
	add	x1, x29, 108
	bl	log2phys
	ldr	x0, [x23,1040]
	add	x21, x0, x21
	ldr	x0, [x21,16]
	ldr	w0, [x0,12]
	cmn	w0, #1
	beq	.L2910
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	ubfiz	x0, x1, 1, 16
	ldr	x2, [x23,1192]
	mov	w21, w1
	ldrh	w2, [x2,x0]
	cbnz	w2, .L2911
	mov	x0, x26
	bl	printk
.L2911:
	mov	w0, w21
	bl	decrement_vpc_count
.L2910:
	add	w24, w24, 1
	b	.L2897
.L2893:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	2
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	adrp	x0, .LANCHOR2+1408
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w0, [x0,#:lo12:.LANCHOR2+1408]
	cbnz	w0, .L2936
	bl	FtlCacheWriteBack
	bl	l2p_flush
	mov	w0, 1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2936:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlSysFlush, .-FtlSysFlush
	.align	2
	.global	FtlDeInit
	.type	FtlDeInit, %function
FtlDeInit:
	adrp	x0, .LANCHOR1+3288
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w0, [x0,#:lo12:.LANCHOR1+3288]
	cmp	w0, 1
	bne	.L2938
	bl	FtlSysFlush
.L2938:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlDeInit, .-FtlDeInit
	.align	2
	.global	ftl_deinit
	.type	ftl_deinit, %function
ftl_deinit:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	ftl_flash_de_init
	bl	FtlDeInit
	bl	ftl_flash_de_init
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_deinit, .-ftl_deinit
	.align	2
	.global	ftl_cache_flush
	.type	ftl_cache_flush, %function
ftl_cache_flush:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FtlCacheWriteBack
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_cache_flush, .-ftl_cache_flush
	.align	2
	.global	ftl_discard
	.type	ftl_discard, %function
ftl_discard:
	stp	x29, x30, [sp, -80]!
	add	w2, w0, w1
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	w20, w0
	adrp	x0, .LANCHOR0+4048
	mov	w19, w1
	str	x23, [sp,48]
	stp	x21, x22, [sp,32]
	ldr	w1, [x0,#:lo12:.LANCHOR0+4048]
	mov	w0, -1
	cmp	w2, w1
	bhi	.L2942
	cmp	w19, 31
	bls	.L2956
	adrp	x22, .LANCHOR2
	add	x21, x22, :lo12:.LANCHOR2
	ldr	w0, [x21,1408]
	cbnz	w0, .L2956
	bl	FtlCacheWriteBack
	ldrh	w1, [x21,830]
	udiv	w21, w20, w1
	msub	w20, w1, w21, w20
	uxth	w20, w20
	cbz	w20, .L2945
	sub	w1, w1, w20
	add	w21, w21, 1
	cmp	w1, w19
	csel	w1, w1, w19, ls
	sub	w19, w19, w1, uxth
.L2945:
	mov	w0, -1
	adrp	x23, .LANCHOR5
	str	w0, [x29,76]
.L2946:
	add	x20, x22, :lo12:.LANCHOR2
	ldrh	w0, [x20,830]
	cmp	w19, w0
	bcc	.L2957
	mov	w0, w21
	add	x1, x29, 72
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x29,72]
	cmn	w0, #1
	beq	.L2947
	add	x1, x23, :lo12:.LANCHOR5
	mov	w2, 1
	ldr	w0, [x1,-120]
	add	w0, w0, 1
	str	w0, [x1,-120]
	ldr	w0, [x20,892]
	add	x1, x29, 76
	add	w0, w0, 1
	str	w0, [x20,892]
	mov	w0, w21
	bl	log2phys
	ldr	w0, [x29,72]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L2947:
	add	x0, x22, :lo12:.LANCHOR2
	add	w21, w21, 1
	ldrh	w0, [x0,830]
	sub	w19, w19, w0
	b	.L2946
.L2957:
	adrp	x0, .LANCHOR5
	add	x0, x0, :lo12:.LANCHOR5
	ldr	w1, [x0,-120]
	cmp	w1, 32
	bls	.L2956
	str	wzr, [x0,-120]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2956:
	mov	w0, 0
.L2942:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldr	x23, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	ftl_discard, .-ftl_discard
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x2, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	ldr	w3, [x2,1408]
	ldrh	w1, [x2,824]
	cbnz	w3, .L2992
	ldrh	w20, [x2,3712]
	mov	w4, 65535
	cmp	w20, w4
	bne	.L2961
.L2970:
	add	x20, x19, :lo12:.LANCHOR2
	adrp	x21, .LANCHOR4
	add	x22, x21, :lo12:.LANCHOR4
	mov	w0, 65535
	add	x23, x20, 3712
	ldrh	w1, [x20,3712]
	str	wzr, [x22,4024]
	cmp	w1, w0
	beq	.L2992
	bl	FtlCacheWriteBack
	mov	w26, 12
	ldrb	w0, [x23,7]
	ldrh	w3, [x20,824]
	ldr	x1, [x20,1192]
	ldrh	w2, [x20,3712]
	mul	w0, w0, w3
	strh	w0, [x1,x2,lsl 1]
	ldrh	w1, [x22,4034]
	mov	w22, 0
	ldr	w0, [x20,884]
	add	w0, w1, w0
	str	w0, [x20,884]
	b	.L2971
.L2961:
	cbz	w0, .L2964
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	ldrh	w5, [x0,3292]
	cmp	w5, w4
	beq	.L2965
.L2966:
	mov	w1, 2
	b	.L2964
.L2965:
	strh	w3, [x0,3292]
	ldrh	w0, [x2,3608]
	cmp	w0, 17
	bhi	.L2966
.L2964:
	add	x21, x19, :lo12:.LANCHOR2
	add	x0, x21, 3712
	bl	FtlGcScanTempBlk
	str	w0, [x29,92]
	cmn	w0, #1
	beq	.L2967
	ubfiz	x20, x20, 1, 16
	ldr	x1, [x21,1176]
	ldrh	w0, [x1,x20]
	cmp	w0, 4
	bls	.L2968
	sub	w0, w0, #5
	strh	w0, [x1,x20]
	mov	w0, 1
	bl	FtlEctTblFlush
.L2968:
	adrp	x0, .LANCHOR4
	add	x1, x0, :lo12:.LANCHOR4
	mov	x19, x0
	ldr	w2, [x1,4024]
	cbnz	w2, .L2969
	ldr	w0, [x1,3568]
	add	w0, w0, 1
	str	w0, [x1,3568]
	ldr	w0, [x29,92]
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L2969:
	add	x0, x19, :lo12:.LANCHOR4
	str	wzr, [x0,4024]
	mov	w0, 1
	b	.L2960
.L2967:
	adrp	x0, .LANCHOR1+3292
	mov	w1, 65535
	ldrh	w2, [x0,#:lo12:.LANCHOR1+3292]
	mov	w0, 1
	cmp	w2, w1
	bne	.L2960
	b	.L2970
.L2974:
	ldr	x25, [x20,992]
	add	x1, x29, 92
	umull	x24, w22, w26
	mov	w2, 0
	add	x23, x25, x24
	ldr	w0, [x23,8]
	bl	log2phys
	ldr	w0, [x29,92]
	ldr	w1, [x25,x24]
	cmp	w0, w1
	bne	.L2972
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	mov	w24, w0
	ldr	w0, [x23,8]
	add	x1, x23, 4
	mov	w2, 1
	bl	log2phys
	mov	w0, w24
.L2991:
	bl	decrement_vpc_count
.L2973:
	add	w22, w22, 1
	uxth	w22, w22
.L2971:
	add	x0, x21, :lo12:.LANCHOR4
	ldrh	w0, [x0,4034]
	cmp	w0, w22
	bhi	.L2974
	b	.L2993
.L2972:
	ldr	w1, [x23,4]
	cmp	w0, w1
	beq	.L2973
	ldrh	w0, [x20,3712]
	b	.L2991
.L2993:
	mov	w0, -1
	adrp	x20, .LANCHOR0
	bl	decrement_vpc_count
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L2975
	add	x1, x19, :lo12:.LANCHOR2
	adrp	x0, .LC155
	add	x0, x0, :lo12:.LC155
	ldrh	w1, [x1,3712]
	bl	printk
.L2975:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,3712]
	ldr	x1, [x1,1192]
	ubfiz	x2, x0, 1, 16
	ldrh	w1, [x1,x2]
	cbz	w1, .L2976
	bl	INSERT_DATA_LIST
	b	.L2977
.L2976:
	bl	INSERT_FREE_LIST
.L2977:
	add	x23, x21, :lo12:.LANCHOR4
	add	x22, x19, :lo12:.LANCHOR2
	mov	w24, -1
	strh	wzr, [x23,4034]
	strh	w24, [x22,3712]
	strh	wzr, [x23,4032]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	strh	w24, [x23,3424]
	ldr	w0, [x22,660]
	cbz	w0, .L2978
	ldr	w0, [x22,924]
	cmp	w0, 39
	bhi	.L2978
	ldrh	w0, [x23,3420]
	ldrh	w1, [x22,3608]
	cmp	w1, w0
	bcs	.L2992
	lsl	w0, w0, 1
	strh	w0, [x22,960]
.L2992:
	mov	w0, 0
	b	.L2960
.L2978:
	add	x21, x21, :lo12:.LANCHOR4
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w1, [x21,3420]
	ldrh	w0, [x19,3608]
	add	w2, w1, w1, lsl 1
	cmp	w0, w2, lsr 2
	ble	.L2992
	add	x20, x20, :lo12:.LANCHOR0
	ldrb	w0, [x20,4044]
	cbz	w0, .L2979
	sub	w1, w1, #2
	strh	w1, [x19,960]
	b	.L2992
.L2979:
	mov	w1, 20
	strh	w1, [x19,960]
.L2960:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	add	x20, x19, 3712
	mov	x0, x20
	ldrh	w1, [x19,824]
	bl	FtlGcScanTempBlk
	ldrh	w1, [x19,3714]
	ldrh	w0, [x19,824]
	cmp	w1, w0
	bcc	.L2994
	add	x0, x19, 3760
	bl	FtlMapBlkWriteDumpData
	mov	w0, 0
	bl	FtlGcFreeTempBlock
	adrp	x0, .LANCHOR4+4024
	str	wzr, [x0,#:lo12:.LANCHOR4+4024]
.L2994:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	stp	x29, x30, [sp, -32]!
	adrp	x0, .LANCHOR4+4080
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	wzr, [x0,#:lo12:.LANCHOR4+4080]
	add	x19, x19, :lo12:.LANCHOR2
	add	x20, x19, 3616
	add	x19, x19, 3664
	mov	x0, x20
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
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	stp	x29, x30, [sp, -48]!
	mov	w1, -1
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR4
	adrp	x19, .LANCHOR2
	stp	x21, x22, [sp,32]
	add	x0, x20, :lo12:.LANCHOR4
	add	x22, x19, :lo12:.LANCHOR2
	strh	w1, [x0,4074]
	strh	wzr, [x0,4076]
	ldrh	w0, [x22,760]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldrh	w1, [x22,3904]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L2998
.L3000:
	mov	w21, -1
	b	.L2999
.L2998:
	bl	FtlLoadSysInfo
	mov	w21, w0
	cbnz	w0, .L3000
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	w0, 1
	bl	FtlUpdateVaildLpn
	ldrh	w2, [x22,862]
	mov	x1, 0
	ldr	x0, [x22,1272]
.L3001:
	cmp	w1, w2
	mov	w3, w1
	bge	.L3005
	add	x4, x0, x1, lsl 4
	add	x1, x1, 1
	ldr	w4, [x4,4]
	tbz	w4, #31, .L3001
.L3005:
	add	x0, x19, :lo12:.LANCHOR2
	cmp	w3, w2
	ldrh	w1, [x0,3556]
	add	w1, w1, 1
	strh	w1, [x0,3556]
	blt	.L3002
	add	x20, x20, :lo12:.LANCHOR4
	ldrh	w0, [x20,4076]
	cbz	w0, .L3006
.L3002:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w2, [x1,3616]
	ldr	x4, [x1,1192]
	lsl	x2, x2, 1
	ldrh	w5, [x1,3620]
	ldrh	w3, [x4,x2]
	sub	w3, w3, w5
	strh	w3, [x4,x2]
	strb	wzr, [x1,3622]
	ldrh	w3, [x1,3664]
	ldrh	w2, [x1,824]
	lsl	x3, x3, 1
	ldr	x4, [x1,1192]
	strh	w2, [x1,3618]
	strh	wzr, [x1,3620]
	ldrh	w5, [x1,3668]
	ldrh	w0, [x4,x3]
	sub	w0, w0, w5
	strh	w0, [x4,x3]
	strb	wzr, [x1,3670]
	ldrh	w0, [x1,824]
	strh	w0, [x1,3666]
	ldrh	w0, [x1,3558]
	strh	wzr, [x1,3668]
	add	w0, w0, 1
	strh	w0, [x1,3558]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L3006:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,3556]
	and	w0, w0, 31
	cbnz	w0, .L3007
	bl	FtlVpcCheckAndModify
.L3007:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w1, 65535
	add	x22, x19, 3616
	ldrh	w0, [x19,3616]
	cmp	w0, w1
	beq	.L2999
	ldrh	w1, [x19,3620]
	cbnz	w1, .L2999
	ldrh	w1, [x19,3668]
	add	x20, x19, 3664
	cbnz	w1, .L2999
	bl	FtlGcRefreshOpenBlock
	ldrh	w0, [x19,3664]
	bl	FtlGcRefreshOpenBlock
	bl	FtlVpcTblFlush
	mov	x0, x22
	bl	allocate_new_data_superblock
	mov	x0, x20
	bl	allocate_new_data_superblock
.L2999:
	mov	w0, w21
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	FtlLowFormat
	.type	FtlLowFormat, %function
FtlLowFormat:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	x23, [sp,48]
	add	x20, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	ldr	w21, [x20,1408]
	cbnz	w21, .L3015
	ldrh	w2, [x20,860]
	mov	w1, w21
	ldr	x0, [x20,1256]
	lsl	w2, w2, 2
	bl	ftl_memset
	ldrh	w2, [x20,860]
	mov	w1, w21
	ldr	x0, [x20,1248]
	lsl	w2, w2, 2
	bl	ftl_memset
	str	w21, [x20,876]
	ldrh	w0, [x20,760]
	str	w21, [x20,880]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbz	w0, .L3016
	bl	FtlMakeBbt
.L3016:
	mov	w0, 0
.L3017:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w2, [x1,830]
	cmp	w0, w2, lsl 7
	bge	.L3044
	ubfiz	x3, x0, 2, 16
	ldr	x4, [x1,1096]
	mvn	w2, w0
	orr	w2, w0, w2, lsl 16
	str	w2, [x4,x3]
	add	w0, w0, 1
	ldr	x2, [x1,1104]
	mov	w1, 23752
	movk	w1, 0xa0f, lsl 16
	uxth	w0, w0
	str	w1, [x2,x3]
	b	.L3017
.L3044:
	ldrh	w21, [x1,764]
	mov	w20, 0
.L3019:
	add	x22, x19, :lo12:.LANCHOR2
	ldrh	w0, [x22,766]
	cmp	w0, w21
	bls	.L3045
	mov	w0, w21
	mov	w1, 1
	add	w21, w21, 1
	bl	FtlLowFormatEraseBlock
	add	w0, w20, w0
	uxth	w21, w21
	uxth	w20, w0
	b	.L3019
.L3045:
	ldrh	w0, [x22,756]
	sub	w1, w20, #3
	cmp	w1, w0, lsl 1
	bge	.L3021
.L3025:
	mov	w20, 0
	mov	w21, w20
	b	.L3022
.L3021:
	udiv	w20, w20, w0
	ldr	w0, [x22,856]
	add	w0, w20, w0
	bl	FtlSysBlkNumInit
	ldrh	w0, [x22,760]
	bl	FtlFreeSysBlkQueueInit
	ldrh	w20, [x22,764]
.L3023:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,766]
	cmp	w0, w20
	bls	.L3025
	mov	w0, w20
	mov	w1, 1
	add	w20, w20, 1
	bl	FtlLowFormatEraseBlock
	uxth	w20, w20
	b	.L3023
.L3022:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w1, [x0,764]
	cmp	w1, w21
	bls	.L3046
	mov	w0, w21
	mov	w1, 0
	add	w21, w21, 1
	bl	FtlLowFormatEraseBlock
	add	w0, w20, w0
	uxth	w21, w21
	uxth	w20, w0
	b	.L3022
.L3046:
	ldrh	w1, [x0,766]
	adrp	x21, .LANCHOR4
	ldrh	w2, [x0,756]
	add	x6, x21, :lo12:.LANCHOR4
	str	w1, [x0,948]
	ldr	w1, [x0,768]
	udiv	w3, w1, w2
	str	w3, [x0,1404]
	ubfx	x5, x3, 5, 16
	add	w4, w5, 36
	strh	w4, [x6,3420]
	mov	w4, 24
	mul	w4, w2, w4
	cmp	w20, w4
	ble	.L3027
	sub	w1, w1, w20
	udiv	w1, w1, w2
	str	w1, [x0,1404]
	lsr	w1, w1, 5
	add	w1, w1, 24
	strh	w1, [x6,3420]
.L3027:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,660]
	cmp	w0, 1
	bne	.L3028
	add	x1, x21, :lo12:.LANCHOR4
	udiv	w0, w20, w2
	ldrh	w4, [x1,3420]
	add	w0, w4, w0
	add	w0, w4, w0, asr 2
	strh	w0, [x1,3420]
.L3028:
	adrp	x22, .LANCHOR0
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L3029
	add	x1, x21, :lo12:.LANCHOR4
	udiv	w0, w20, w2
	ldrh	w4, [x1,3420]
	add	w0, w4, w0
	add	w0, w4, w0, asr 2
	strh	w0, [x1,3420]
.L3029:
	add	x6, x19, :lo12:.LANCHOR2
	ldrh	w1, [x6,818]
	cbz	w1, .L3031
	add	x0, x21, :lo12:.LANCHOR4
	ldrh	w4, [x0,3420]
	add	w4, w4, w1, lsr 1
	strh	w4, [x0,3420]
	mul	w4, w1, w2
	cmp	w4, w20
	ble	.L3031
	add	w1, w1, 32
	str	w3, [x6,1404]
	add	w1, w5, w1
	strh	w1, [x0,3420]
.L3031:
	add	x19, x19, :lo12:.LANCHOR2
	add	x20, x21, :lo12:.LANCHOR4
	add	x22, x22, :lo12:.LANCHOR0
	mov	w23, -1
	ldrh	w0, [x20,3420]
	ldr	w1, [x19,1404]
	sub	w0, w1, w0
	ldrh	w1, [x19,824]
	mul	w0, w0, w2
	str	w0, [x20,3416]
	mul	w0, w0, w1
	ldrh	w1, [x19,830]
	str	w0, [x19,1404]
	mul	w0, w0, w1
	str	w0, [x22,4048]
	bl	FtlBbmTblFlush
	ldrh	w2, [x19,766]
	mov	w1, 0
	ldr	x0, [x19,1192]
	lsl	w2, w2, 1
	bl	ftl_memset
	strh	w23, [x20,3424]
	strh	wzr, [x20,3426]
	mov	w0, 1
	strb	wzr, [x20,3430]
	mov	w1, 255
	strb	wzr, [x20,3432]
	add	x20, x19, 3616
	ldrh	w2, [x19,764]
	strb	w0, [x20,8]
	ldr	x0, [x22,16]
	lsr	w2, w2, 3
	str	wzr, [x19,3900]
	strh	wzr, [x20,2]
	strb	wzr, [x20,6]
	strh	wzr, [x19,3616]
	bl	ftl_memset
.L3033:
	mov	x0, x20
	bl	make_superblock
	ldrb	w0, [x20,7]
	cbnz	w0, .L3034
	ldrh	w1, [x20]
	ldr	x0, [x19,1192]
	strh	w23, [x0,x1,lsl 1]
	ldrh	w0, [x20]
	add	w0, w0, 1
	strh	w0, [x20]
	b	.L3033
.L3034:
	ldr	w0, [x19,876]
	mov	w22, -1
	str	w0, [x20,12]
	add	w0, w0, 1
	ldrh	w1, [x20]
	str	w0, [x19,876]
	ldr	x0, [x19,1192]
	ldrh	w2, [x20,4]
	strh	w2, [x0,x1,lsl 1]
	add	x0, x19, 3664
	strh	wzr, [x19,3666]
	ldrh	w1, [x20]
	mov	x20, x0
	strb	wzr, [x19,3670]
	add	w1, w1, 1
	strh	w1, [x19,3664]
	mov	w1, 1
	strb	w1, [x19,3672]
.L3035:
	mov	x0, x20
	bl	make_superblock
	ldrb	w0, [x20,7]
	cbnz	w0, .L3036
	ldrh	w1, [x20]
	ldr	x0, [x19,1192]
	strh	w22, [x0,x1,lsl 1]
	ldrh	w0, [x20]
	add	w0, w0, 1
	strh	w0, [x20]
	b	.L3035
.L3036:
	ldr	w0, [x19,876]
	add	x21, x21, :lo12:.LANCHOR4
	str	w0, [x20,12]
	add	w0, w0, 1
	ldrh	w1, [x20]
	str	w0, [x19,876]
	ldr	x0, [x19,1192]
	ldrh	w2, [x20,4]
	mov	w20, -1
	strh	w2, [x0,x1,lsl 1]
	strh	w20, [x19,3712]
	bl	FtlFreeSysBlkQueueOut
	strh	w0, [x19,3904]
	ldr	w0, [x21,3416]
	strh	w0, [x19,3910]
	ldr	w0, [x19,876]
	str	w0, [x19,3912]
	add	w0, w0, 1
	strh	wzr, [x19,3906]
	strh	w20, [x19,3908]
	str	w0, [x19,876]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cbnz	w0, .L3015
	adrp	x0, .LANCHOR1+3288
	mov	w1, 1
	str	w1, [x0,#:lo12:.LANCHOR1+3288]
.L3015:
	mov	w0, 0
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlLowFormat, .-FtlLowFormat
	.align	2
	.global	FtlReInitForSDUpdata
	.type	FtlReInitForSDUpdata, %function
FtlReInitForSDUpdata:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,4044]
	cbz	w0, .L3048
.L3050:
	mov	w0, 0
	b	.L3049
.L3048:
	adrp	x0, RK29_NANDC_REG_BASE
	ldr	x0, [x0,#:lo12:RK29_NANDC_REG_BASE]
	bl	FlashInit
	cbnz	w0, .L3050
	bl	FlashLoadFactorBbt
	cbz	w0, .L3051
	bl	FlashMakeFactorBbt
.L3051:
	adrp	x20, .LANCHOR2
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x0, [x0,696]
	bl	FlashReadIdbDataRaw
	cbz	w0, .L3052
	mov	w1, 0
	mov	w2, 16
	add	x0, x29, 32
	bl	FlashReadFacBbtData
	mov	w0, 0
	ldr	w2, [x29,32]
	mov	w1, w0
	mov	w4, 1
.L3054:
	lsl	w3, w4, w1
	add	w1, w1, 1
	tst	w3, w2
	cset	w3, ne
	cmp	w1, 16
	add	w0, w0, w3
	bne	.L3054
	cmp	w0, 6
	bhi	.L3055
	add	x0, x19, :lo12:.LANCHOR0
	b	.L3077
.L3055:
	mov	w1, 0
	mov	w4, 1
.L3058:
	lsl	w3, w4, w1
	add	w1, w1, 1
	tst	w3, w2
	cset	w3, ne
	cmp	w1, 24
	add	w0, w0, w3
	bne	.L3058
	cmp	w0, 17
	add	x0, x19, :lo12:.LANCHOR0
	bls	.L3077
	mov	w1, 36
.L3077:
	strb	w1, [x0,25]
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x0,25]
	strh	w1, [x0,4042]
.L3052:
	adrp	x0, .LC45
	adrp	x1, .LC138
	add	x1, x1, :lo12:.LC138
	add	x0, x0, :lo12:.LC45
	add	x20, x20, :lo12:.LANCHOR2
	bl	printk
	add	x0, x19, :lo12:.LANCHOR0
	mov	w19, 1
	add	x0, x0, 4016
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldrh	w0, [x20,760]
	bl	FtlFreeSysBlkQueueInit
.L3060:
	bl	FtlLoadBbt
	cbz	w0, .L3061
.L3079:
	bl	FtlLowFormat
	cmp	w19, 3
	bhi	.L3080
	add	w19, w19, 1
	b	.L3060
.L3080:
	mov	w0, -1
	b	.L3049
.L3061:
	bl	FtlSysBlkInit
	cbnz	w0, .L3079
	adrp	x1, .LANCHOR1+3288
	mov	w2, 1
	str	w2, [x1,#:lo12:.LANCHOR1+3288]
.L3049:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlReInitForSDUpdata, .-FtlReInitForSDUpdata
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	x23, [sp,48]
	add	x0, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	ldr	w1, [x0,1408]
	cbz	w1, .L3082
.L3085:
	mov	w0, 0
	b	.L3083
.L3082:
	adrp	x1, .LANCHOR0+4044
	adrp	x22, .LANCHOR4
	ldrb	w1, [x1,#:lo12:.LANCHOR0+4044]
	cbz	w1, .L3084
	add	x1, x22, :lo12:.LANCHOR4
	ldr	w1, [x1,4000]
	tbz	x1, 0, .L3084
	ldrh	w0, [x0,3716]
	cbnz	w0, .L3085
.L3084:
	add	x0, x19, :lo12:.LANCHOR2
	add	x1, x22, :lo12:.LANCHOR4
	mov	w2, 0
	mov	w21, 0
	mov	w3, w2
	mov	w23, 56
	ldr	x0, [x0,1008]
	ldr	w1, [x1,4000]
	bl	FlashProgPages
.L3086:
	add	x20, x22, :lo12:.LANCHOR4
	ldr	w1, [x20,4000]
	cmp	w21, w1
	bcs	.L3096
	add	x1, x19, :lo12:.LANCHOR2
	umull	x0, w21, w23
	ldr	x2, [x1,1008]
	add	x3, x2, x0
	ldr	w2, [x2,x0]
	cmn	w2, #1
	bne	.L3087
	ldrh	w4, [x1,3712]
	ldr	x3, [x1,1192]
	strh	wzr, [x3,x4,lsl 1]
	strh	w2, [x1,3712]
	ldr	x1, [x1,1008]
	ldr	w2, [x20,3568]
	add	x0, x1, x0
	add	w2, w2, 1
	str	w2, [x20,3568]
	ldr	w0, [x0,4]
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	b	.L3095
.L3087:
	ldr	x2, [x3,16]
	add	w21, w21, 1
	ldr	w1, [x3,4]
	uxth	w21, w21
	ldr	w0, [x2,12]
	ldr	w2, [x2,8]
	bl	FtlGcUpdatePage
	b	.L3086
.L3096:
	add	x19, x19, :lo12:.LANCHOR2
	ldr	x0, [x19,1008]
	bl	FtlGcBufFree
	str	wzr, [x20,4000]
	ldrh	w0, [x19,3716]
	cbnz	w0, .L3085
	mov	w0, 1
	bl	FtlGcFreeTempBlock
.L3095:
	mov	w0, 1
.L3083:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldr	x23, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	add	x0, x19, :lo12:.LANCHOR2
	add	x1, x0, 3712
	ldrh	w2, [x0,3712]
	mov	w0, 65535
	cmp	w2, w0
	beq	.L3098
	ldrh	w0, [x1,4]
	cbnz	w0, .L3099
.L3098:
	bl	FtlCacheWriteBack
	mov	w0, 0
	bl	FtlGcFreeTempBlock
	add	x0, x19, :lo12:.LANCHOR2
	add	x0, x0, 3712
	strb	wzr, [x0,8]
	bl	allocate_data_superblock
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	strh	wzr, [x0,4032]
	strh	wzr, [x0,4034]
	bl	l2p_flush
	mov	w0, 0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L3099:
	add	x0, x19, :lo12:.LANCHOR2
	add	x0, x0, 3712
	bl	get_new_active_ppa
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.global	ftl_read
	.type	ftl_read, %function
ftl_read:
	sub	sp, sp, #224
	cmp	w0, 16
	stp	x29, x30, [sp,16]
	add	x29, sp, 16
	stp	x19, x20, [sp,32]
	stp	x25, x26, [sp,80]
	stp	x27, x28, [sp,96]
	stp	x21, x22, [sp,48]
	stp	x23, x24, [sp,64]
	mov	w20, w1
	mov	w28, w2
	mov	x25, x3
	bne	.L3101
	add	w0, w1, 256
	mov	w1, w2
	mov	x2, x3
	bl	FtlVendorPartRead
	b	.L3102
.L3101:
	add	w0, w1, w2
	str	w0, [x29,188]
	adrp	x0, .LANCHOR0+4048
	ldr	w2, [x29,188]
	ldr	w1, [x0,#:lo12:.LANCHOR0+4048]
	mov	w0, -1
	cmp	w2, w1
	bhi	.L3102
	adrp	x19, .LANCHOR2
	adrp	x21, .LANCHOR4
	add	x0, x19, :lo12:.LANCHOR2
	sub	w24, w2, #1
	add	x2, x21, :lo12:.LANCHOR4
	ldrh	w1, [x0,830]
	udiv	w27, w20, w1
	udiv	w24, w24, w1
	ldr	w1, [x2,3988]
	add	w1, w28, w1
	sub	w23, w24, w27
	str	w1, [x2,3988]
	add	w23, w23, 1
	ldr	w1, [x0,904]
	add	w1, w23, w1
	str	w1, [x0,904]
	mov	w0, w27
	mov	w1, w24
	bl	FtlCacheMetchLpa
	str	x21, [x29,120]
	cbz	w0, .L3103
	bl	FtlCacheWriteBack
.L3103:
	mov	w26, 0
	adrp	x0, .LC56
	add	x0, x0, :lo12:.LC56
	mov	w21, w27
	str	w26, [x29,184]
	mov	w22, w26
	str	w26, [x29,172]
	str	x0, [x29,112]
.L3104:
	cbz	w23, .L3146
	mov	w0, w21
	add	x1, x29, 204
	mov	w2, 0
	bl	log2phys
	ldr	w3, [x29,204]
	cmn	w3, #1
	bne	.L3144
	mov	w3, 0
.L3105:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,830]
	cmp	w3, w0
	bcs	.L3109
	madd	w0, w21, w0, w3
	cmp	w0, w20
	bcc	.L3107
	ldr	w1, [x29,188]
	cmp	w0, w1
	bcs	.L3107
	sub	w0, w0, w20
	mov	w1, 0
	ubfiz	x0, x0, 9, 23
	mov	w2, 512
	add	x0, x25, x0
	str	x3, [x29,176]
	bl	ftl_memset
	ldr	x3, [x29,176]
.L3107:
	add	w3, w3, 1
	b	.L3105
.L3144:
	add	x2, x19, :lo12:.LANCHOR2
	mov	w0, 56
	cmp	w21, w27
	umull	x1, w22, w0
	ldr	x0, [x2,1000]
	add	x0, x0, x1
	str	w3, [x0,4]
	ldrh	w0, [x2,830]
	bne	.L3110
	ldr	x3, [x2,1000]
	ldr	x2, [x2,1096]
	add	x3, x3, x1
	str	x2, [x3,8]
	udiv	w2, w20, w0
	msub	w2, w2, w0, w20
	str	w2, [x29,144]
	sub	w2, w0, w2
	cmp	w2, w28
	csel	w2, w2, w28, ls
	str	w2, [x29,184]
	cmp	w2, w0
	bne	.L3111
	str	x25, [x3,8]
	b	.L3111
.L3110:
	cmp	w21, w24
	bne	.L3112
	ldr	x3, [x2,1000]
	ldr	x2, [x2,1104]
	add	x3, x3, x1
	ldr	w4, [x29,188]
	str	x2, [x3,8]
	mul	w2, w21, w0
	sub	w26, w4, w2
	cmp	w26, w0
	bne	.L3111
	sub	w2, w2, w20
	ubfiz	x2, x2, 9, 23
	add	x2, x25, x2
	str	x2, [x3,8]
	b	.L3111
.L3112:
	ldr	x2, [x2,1000]
	mul	w0, w21, w0
	add	x2, x2, x1
	sub	w0, w0, w20
	ubfiz	x0, x0, 9, 23
	add	x0, x25, x0
	str	x0, [x2,8]
.L3111:
	add	x2, x19, :lo12:.LANCHOR2
	ldr	x0, [x2,1000]
	add	x1, x0, x1
	ldrh	w0, [x2,836]
	ldr	x2, [x2,1128]
	mul	w0, w22, w0
	str	w21, [x1,24]
	add	w22, w22, 1
	and	x0, x0, 4294967292
	add	x0, x2, x0
	str	x0, [x1,16]
.L3109:
	subs	w23, w23, #1
	add	w21, w21, 1
	beq	.L3113
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,756]
	cmp	w22, w0, lsl 3
	bne	.L3104
.L3113:
	cbz	w22, .L3104
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, w22
	mov	w2, 0
	ldr	x0, [x0,1000]
	bl	FlashReadPages
	str	xzr, [x29,176]
	ldr	x0, [x29,144]
	ubfiz	x0, x0, 9, 23
	str	x0, [x29,136]
	ldr	w0, [x29,184]
	lsl	w0, w0, 9
	str	w0, [x29,168]
	lsl	w0, w26, 9
	str	w0, [x29,132]
	ldr	x0, [x29,120]
	add	x0, x0, :lo12:.LANCHOR4
	str	x0, [x29,160]
.L3115:
	ldr	w0, [x29,176]
	cmp	w22, w0
	bls	.L3147
	ldr	x0, [x29,176]
	add	x2, x19, :lo12:.LANCHOR2
	mov	x1, 56
	mul	x8, x0, x1
	ldr	x0, [x2,1000]
	add	x0, x0, x8
	ldr	w1, [x0,24]
	cmp	w1, w27
	bne	.L3116
	ldr	x1, [x0,8]
	ldr	x0, [x2,1096]
	cmp	x1, x0
	bne	.L3117
	ldr	x2, [x29,136]
	mov	x0, x25
	str	x8, [x29,104]
	add	x1, x1, x2
	ldr	w2, [x29,168]
	b	.L3145
.L3116:
	cmp	w1, w24
	bne	.L3117
	ldr	x1, [x0,8]
	ldr	x0, [x2,1104]
	cmp	x1, x0
	bne	.L3117
	ldrh	w0, [x2,830]
	ldr	w2, [x29,132]
	str	x8, [x29,104]
	mul	w0, w24, w0
	sub	w0, w0, w20
	ubfiz	x0, x0, 9, 23
	add	x0, x25, x0
.L3145:
	bl	ftl_memcpy
	ldr	x8, [x29,104]
.L3117:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x0, [x0,1000]
	add	x2, x0, x8
	ldr	w1, [x0,x8]
	cmn	w1, #1
	bne	.L3118
	ldr	x0, [x29,160]
	str	w1, [x29,172]
	add	x3, x0, 3472
	ldr	w0, [x0,3544]
	add	w0, w0, 1
	str	w0, [x3,72]
.L3118:
	ldr	x0, [x2,16]
	ldr	w1, [x2,24]
	ldr	w0, [x0,8]
	cmp	w1, w0
	beq	.L3119
	ldr	x0, [x29,160]
	str	x8, [x29,104]
	add	x1, x0, 3472
	ldr	w0, [x0,3544]
	add	w0, w0, 1
	str	w0, [x1,72]
	ldr	x7, [x2,8]
	ldr	x6, [x2,16]
	ldr	w0, [x7,4]
	str	w0, [sp]
	ldr	x0, [x29,112]
	ldr	w1, [x2,24]
	ldr	w3, [x6]
	ldr	w4, [x6,4]
	ldr	w5, [x6,8]
	ldr	w2, [x2,4]
	ldr	w6, [x6,12]
	ldr	w7, [x7]
	bl	printk
	ldr	x8, [x29,104]
.L3119:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x0, [x0,1000]
	add	x1, x0, x8
	ldr	w0, [x0,x8]
	cmp	w0, 256
	bne	.L3120
	ldr	w0, [x1,4]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L3120:
	ldr	x0, [x29,176]
	add	x0, x0, 1
	str	x0, [x29,176]
	b	.L3115
.L3147:
	mov	w22, 0
	b	.L3104
.L3146:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w0, [x19,966]
	cbz	w0, .L3123
	mov	w0, w23
	mov	w1, 1
	bl	ftl_do_gc
.L3123:
	ldr	w0, [x29,172]
.L3102:
	sub	sp, x29, #16
	ldp	x19, x20, [sp,32]
	ldp	x21, x22, [sp,48]
	ldp	x23, x24, [sp,64]
	ldp	x25, x26, [sp,80]
	ldp	x27, x28, [sp,96]
	ldp	x29, x30, [sp,16]
	add	sp, sp, 224
	ret
	.size	ftl_read, .-ftl_read
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	stp	x29, x30, [sp, -48]!
	mov	w0, -1
	adrp	x1, .LC138
	add	x29, sp, 0
	str	x21, [sp,32]
	adrp	x21, .LANCHOR1
	add	x21, x21, :lo12:.LANCHOR1
	add	x1, x1, :lo12:.LC138
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	adrp	x20, .LANCHOR0
	str	w0, [x21,3288]
	adrp	x0, .LANCHOR5-116
	add	x19, x19, :lo12:.LANCHOR2
	add	x20, x20, :lo12:.LANCHOR0
	str	wzr, [x0,#:lo12:.LANCHOR5-116]
	adrp	x0, .LC45
	add	x0, x0, :lo12:.LC45
	str	wzr, [x19,1408]
	bl	printk
	add	x0, x20, 4016
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	ldrh	w0, [x19,760]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbz	w0, .L3149
	adrp	x0, .LC156
	add	x0, x0, :lo12:.LC156
	b	.L3161
.L3149:
	bl	FtlSysBlkInit
	cbz	w0, .L3151
	adrp	x0, .LC157
	add	x0, x0, :lo12:.LC157
.L3161:
	adrp	x1, .LANCHOR3
	add	x1, x1, :lo12:.LANCHOR3
	add	x1, x1, 96
	bl	printk
	b	.L3150
.L3151:
	mov	w1, 1
	str	w1, [x21,3288]
	bl	ftl_do_gc
	ldrh	w0, [x19,3608]
	cmp	w0, 15
	bhi	.L3152
	mov	w19, 1024
.L3153:
	mov	w0, 1
	mov	w1, w0
	bl	ftl_do_gc
	subs	w19, w19, #1
	bne	.L3153
	b	.L3150
.L3152:
	ldrb	w0, [x20,4044]
	cbz	w0, .L3150
	mov	w19, 128
.L3155:
	mov	w0, 1
	mov	w1, w0
	bl	ftl_do_gc
	subs	w19, w19, #1
	bne	.L3155
.L3150:
	mov	w0, 0
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlInit, .-FtlInit
	.align	2
	.global	ftl_write
	.type	ftl_write, %function
ftl_write:
	stp	x29, x30, [sp, -272]!
	mov	w4, w0
	mov	w0, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	w20, w1
	adrp	x1, .LANCHOR2
	stp	x21, x22, [sp,32]
	mov	w21, w2
	add	x2, x1, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	mov	x22, x3
	str	x1, [x29,152]
	ldr	w3, [x2,1408]
	cbnz	w3, .L3163
	cmp	w4, 16
	bne	.L3164
	add	w0, w20, 256
	mov	w1, w21
	mov	x2, x22
	bl	FtlVendorPartWrite
	b	.L3163
.L3164:
	adrp	x0, .LANCHOR0+4048
	add	w24, w20, w21
	ldr	w1, [x0,#:lo12:.LANCHOR0+4048]
	mov	w0, -1
	cmp	w24, w1
	bhi	.L3163
	adrp	x5, .LANCHOR5
	ldrh	w1, [x2,830]
	add	x26, x5, :lo12:.LANCHOR5
	sub	w24, w24, #1
	mov	w0, 2048
	adrp	x4, .LANCHOR4
	udiv	w23, w20, w1
	add	x3, x4, :lo12:.LANCHOR4
	str	w0, [x26,-112]
	cmp	w21, w1, lsl 1
	str	x5, [x29,144]
	str	x4, [x29,112]
	udiv	w0, w24, w1
	str	w0, [x29,200]
	sub	w28, w0, w23
	add	w25, w28, 1
	ldr	w0, [x2,888]
	add	w0, w25, w0
	str	w0, [x2,888]
	ldr	w0, [x3,3984]
	add	w0, w21, w0
	str	w0, [x3,3984]
	ldr	w3, [x2,976]
	cset	w0, cs
	str	w0, [x29,204]
	cbz	w3, .L3166
	mov	w0, 56
	sub	w3, w3, #1
	umull	x3, w3, w0
	ldr	x0, [x2,1040]
	add	x3, x0, x3
	ldr	w0, [x3,24]
	cmp	w23, w0
	bne	.L3167
	ldr	w0, [x2,896]
	add	w0, w0, 1
	str	w0, [x2,896]
	ldr	w0, [x26,-108]
	add	w0, w0, 1
	str	w0, [x26,-108]
	msub	w0, w23, w1, w20
	sub	w1, w1, w0
	ubfiz	x0, x0, 9, 23
	cmp	w1, w21
	csel	w19, w1, w21, ls
	ldr	x1, [x3,8]
	lsl	w25, w19, 9
	add	x0, x1, x0
	mov	w2, w25
	mov	x1, x22
	bl	ftl_memcpy
	cbnz	w28, .L3168
	ldr	w0, [x26,-108]
	cmp	w0, 2
	bgt	.L3168
.L3201:
	mov	w0, 0
	b	.L3163
.L3168:
	add	x22, x22, x25
	sub	w21, w21, w19
	add	w20, w20, w19
	add	w23, w23, 1
	mov	w25, w28
.L3167:
	ldr	x0, [x29,144]
	add	x0, x0, :lo12:.LANCHOR5
	str	wzr, [x0,-108]
.L3166:
	ldr	w1, [x29,200]
	mov	w0, w23
	bl	FtlCacheMetchLpa
	cbz	w0, .L3169
	bl	FtlCacheWriteBack
.L3169:
	ldr	x0, [x29,144]
	ldr	x1, [x29,152]
	add	x0, x0, :lo12:.LANCHOR5
	str	w23, [x29,192]
	add	x3, x1, :lo12:.LANCHOR2
	add	x19, x3, 3616
	mov	x24, x3
	str	x19, [x0,-128]
	adrp	x0, .LC158
	add	x0, x0, :lo12:.LC158
	str	x19, [x29,176]
	str	x0, [x29,96]
.L3170:
	cbz	w25, .L3230
	ldrh	w0, [x19,4]
	cbnz	w0, .L3171
	ldr	x1, [x29,176]
	adrp	x26, .LANCHOR1
	cmp	x19, x1
	bne	.L3172
	add	x0, x24, 3664
	ldrh	w19, [x0,4]
	cbnz	w19, .L3173
	bl	allocate_new_data_superblock
	add	x0, x26, :lo12:.LANCHOR1
	str	w19, [x0,3296]
.L3173:
	ldr	x0, [x29,176]
	add	x26, x26, :lo12:.LANCHOR1
	add	x19, x24, 3664
	bl	allocate_new_data_superblock
	ldr	w0, [x26,3296]
	cbnz	w0, .L3174
.L3175:
	ldr	x19, [x29,176]
	b	.L3174
.L3172:
	add	x26, x26, :lo12:.LANCHOR1
	str	w0, [x26,3296]
	ldr	x0, [x29,176]
	ldrh	w0, [x0,4]
	cbnz	w0, .L3175
	mov	x0, x19
	bl	allocate_new_data_superblock
.L3174:
	ldrh	w0, [x19,4]
	cbnz	w0, .L3176
	mov	x0, x19
	bl	allocate_new_data_superblock
.L3176:
	ldr	x0, [x29,144]
	add	x0, x0, :lo12:.LANCHOR5
	str	x19, [x0,-128]
.L3171:
	ldr	w0, [x24,972]
	mov	x28, 0
	ldr	w2, [x24,976]
	ldrh	w1, [x19,4]
	sub	w0, w0, w2
	cmp	w1, w0
	csel	w0, w1, w0, ls
	cmp	w0, w25
	csel	w0, w0, w25, ls
	str	w0, [x29,140]
	ldr	w0, [x29,204]
	and	w0, w0, 1
	str	w0, [x29,108]
	add	w0, w21, w20
	str	w0, [x29,136]
.L3177:
	ldr	w1, [x29,192]
	mov	w0, w28
	add	w27, w1, w28
	ldr	w1, [x29,140]
	cmp	w28, w1
	bcs	.L3231
	ldrh	w1, [x19,4]
	cbz	w1, .L3207
	ldr	w1, [x29,200]
	cmp	w27, w1
	cset	w7, eq
	cbz	w0, .L3179
	ldr	w1, [x29,108]
	tst	w7, w1
	beq	.L3179
	ldrh	w1, [x24,830]
	ldr	w2, [x29,136]
	msub	w2, w27, w1, w2
	cmp	w2, w1
	bne	.L3207
.L3179:
	add	x1, x29, 212
	mov	w2, 0
	mov	w0, w27
	str	x7, [x29,160]
	bl	log2phys
	mov	x0, x19
	bl	get_new_active_ppa
	ldr	w6, [x24,976]
	mov	w5, 56
	ldr	x2, [x24,1040]
	str	x5, [x29,184]
	umull	x1, w6, w5
	add	x2, x2, x1
	str	w0, [x2,4]
	ldrh	w2, [x24,836]
	ldr	x0, [x24,1040]
	add	x1, x0, x1
	mul	w0, w2, w6
	and	x0, x0, 4294967292
	str	x0, [x29,128]
	ldr	x0, [x24,1144]
	ldr	x3, [x29,128]
	str	x0, [x29,120]
	add	x26, x0, x3
	ldrh	w0, [x24,834]
	str	x26, [x1,16]
	str	w27, [x1,24]
	mul	w0, w0, w6
	ldr	x6, [x24,1088]
	and	x0, x0, 4294967292
	add	x0, x6, x0
	str	x0, [x1,8]
	mov	x0, x26
	mov	w1, 0
	bl	ftl_memset
	cmp	w27, w23
	cset	w0, eq
	str	w0, [x29,196]
	cbnz	w0, .L3210
	ldr	x7, [x29,160]
	ldr	x5, [x29,184]
	cbz	w7, .L3180
	ldr	w0, [x29,136]
	ldrh	w2, [x24,830]
	msub	w2, w27, w2, w0
	uxth	w0, w2
	str	w0, [x29,184]
	ldr	w0, [x29,196]
	str	w0, [x29,160]
	b	.L3183
.L3210:
	ldrh	w2, [x24,830]
	udiv	w0, w20, w2
	msub	w0, w0, w2, w20
	str	w0, [x29,160]
	sub	w2, w2, w0
	cmp	w2, w21
	csel	w0, w2, w21, ls
	str	w0, [x29,184]
.L3183:
	ldrh	w0, [x24,830]
	ldr	w1, [x29,184]
	cmp	w1, w0
	bne	.L3184
	ldr	w0, [x29,196]
	mov	x1, x22
	cbnz	w0, .L3185
	ldr	w0, [x29,184]
	mul	w1, w0, w27
	sub	w1, w1, w20
	ubfiz	x1, x1, 9, 23
	add	x1, x22, x1
.L3185:
	ldr	w2, [x29,204]
	mov	w0, 56
	cbz	w2, .L3186
	ldr	w2, [x24,976]
	umull	x0, w2, w0
	ldr	x2, [x24,1040]
	add	x0, x2, x0
	b	.L3227
.L3186:
	ldr	w2, [x24,976]
	umull	x0, w2, w0
	ldr	x2, [x24,1040]
	add	x0, x2, x0
	ldr	x0, [x0,8]
	b	.L3228
.L3184:
	ldr	w0, [x29,212]
	cmn	w0, #1
	beq	.L3188
	str	w0, [x29,220]
	mov	w1, 56
	ldr	w0, [x24,976]
	mov	w2, 0
	str	w27, [x29,240]
	umull	x0, w0, w1
	ldr	x1, [x24,1040]
	add	x0, x1, x0
	ldr	x1, [x0,8]
	ldr	x0, [x0,16]
	str	x1, [x29,224]
	mov	w1, 1
	str	x0, [x29,232]
	add	x0, x29, 216
	bl	FlashReadPages
	ldr	w0, [x29,216]
	cmn	w0, #1
	bne	.L3189
	ldr	x0, [x29,112]
	add	x0, x0, :lo12:.LANCHOR4
	ldr	w1, [x0,3544]
	add	w1, w1, 1
	str	w1, [x0,3544]
	b	.L3191
.L3189:
	ldr	w0, [x26,8]
	cmp	w0, w27
	beq	.L3191
	ldr	x0, [x29,112]
	mov	w2, w27
	add	x0, x0, :lo12:.LANCHOR4
	ldr	w1, [x0,3544]
	add	w1, w1, 1
	str	w1, [x0,3544]
	ldr	x0, [x29,96]
	ldr	w1, [x26,8]
	bl	printk
	b	.L3191
.L3188:
	ldr	w0, [x24,976]
	mov	w1, 56
	ldrh	w2, [x24,834]
	umull	x0, w0, w1
	ldr	x1, [x24,1040]
	add	x0, x1, x0
	mov	w1, 0
	ldr	x0, [x0,8]
	bl	ftl_memset
.L3191:
	ldr	w1, [x29,184]
	mov	w0, 56
	lsl	w2, w1, 9
	ldr	w1, [x29,196]
	cbz	w1, .L3192
	ldr	w1, [x24,976]
	umull	x0, w1, w0
	ldr	x1, [x24,1040]
	add	x1, x1, x0
	ldr	x0, [x29,160]
	ubfiz	x0, x0, 9, 23
	ldr	x1, [x1,8]
	add	x0, x1, x0
	mov	x1, x22
	b	.L3229
.L3192:
	ldr	w1, [x24,976]
	umull	x0, w1, w0
	ldr	x1, [x24,1040]
	add	x0, x1, x0
	ldrh	w1, [x24,830]
	mul	w1, w27, w1
	ldr	x0, [x0,8]
	sub	w1, w1, w20
	ubfiz	x1, x1, 9, 23
	add	x1, x22, x1
	b	.L3229
.L3180:
	ldr	w0, [x29,204]
	ldrh	w1, [x24,830]
	ldr	w2, [x24,976]
	cbz	w0, .L3193
	mul	w1, w27, w1
	umull	x0, w2, w5
	sub	w1, w1, w20
	ldr	x2, [x24,1040]
	ubfiz	x1, x1, 9, 23
	add	x0, x2, x0
	add	x1, x22, x1
.L3227:
	str	x1, [x0,8]
	b	.L3187
.L3193:
	umull	x0, w2, w5
	ldr	x2, [x24,1040]
	mul	w1, w27, w1
	add	x0, x2, x0
	sub	w1, w1, w20
	ubfiz	x1, x1, 9, 23
	ldr	x0, [x0,8]
	add	x1, x22, x1
.L3228:
	ldrh	w2, [x24,834]
.L3229:
	bl	ftl_memcpy
.L3187:
	ldr	x1, [x29,120]
	mov	w0, -3947
	ldr	x2, [x29,128]
	add	x28, x28, 1
	strh	w0, [x1,x2]
	str	w27, [x26,8]
	ldr	w0, [x24,880]
	str	w0, [x26,4]
	add	w0, w0, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x24,880]
	ldr	w0, [x29,212]
	str	w0, [x26,12]
	ldrh	w0, [x19]
	strh	w0, [x26,2]
	ldr	w0, [x24,976]
	add	w0, w0, 1
	str	w0, [x24,976]
	b	.L3177
.L3231:
	str	w27, [x29,192]
	mov	x0, x1
	b	.L3178
.L3207:
	str	w27, [x29,192]
.L3178:
	sub	w25, w25, w0
	ldr	w0, [x29,204]
	cbnz	w0, .L3197
	ldr	w1, [x24,976]
	ldr	w0, [x24,972]
	cmp	w1, w0
	bcs	.L3197
	ldrh	w0, [x19,4]
	cbz	w0, .L3197
.L3199:
	str	wzr, [x29,204]
	b	.L3170
.L3197:
	bl	FtlCacheWriteBack
	str	wzr, [x24,976]
	cmp	w25, 1
	bhi	.L3170
	b	.L3199
.L3230:
	ldr	w1, [x29,200]
	mov	w0, w25
	sub	w1, w1, w23
	bl	ftl_do_gc
	ldr	x0, [x29,152]
	add	x0, x0, :lo12:.LANCHOR2
	ldrh	w1, [x0,3608]
	cmp	w1, 31
	bhi	.L3201
	mov	w1, 128
	mov	w19, 16
	strh	w1, [x0,962]
	strh	w1, [x0,960]
.L3202:
	mov	w0, 0
	mov	w1, 1
	bl	ftl_do_gc
	ldr	x0, [x29,152]
	add	x0, x0, :lo12:.LANCHOR2
	ldr	w0, [x0,1408]
	cbnz	w0, .L3201
	subs	w19, w19, #1
	bne	.L3202
	b	.L3201
.L3163:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 272
	ret
	.size	ftl_write, .-ftl_write
	.align	2
	.global	ftl_vendor_write
	.type	ftl_vendor_write, %function
ftl_vendor_write:
	stp	x29, x30, [sp, -16]!
	mov	w5, w0
	mov	w4, w1
	add	x29, sp, 0
	mov	x3, x2
	mov	w1, w5
	mov	w2, w4
	mov	w0, 16
	bl	ftl_write
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_vendor_write, .-ftl_vendor_write
	.align	2
	.global	ftl_sys_write
	.type	ftl_sys_write, %function
ftl_sys_write:
	stp	x29, x30, [sp, -16]!
	mov	w4, w1
	mov	x3, x2
	add	x29, sp, 0
	add	w1, w0, 256
	mov	w2, w4
	mov	w0, 16
	bl	ftl_write
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_sys_write, .-ftl_sys_write
	.align	2
	.global	ftl_fix_nand_power_lost_error
	.type	ftl_fix_nand_power_lost_error, %function
ftl_fix_nand_power_lost_error:
	adrp	x0, .LANCHOR0+4044
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	ldrb	w0, [x0,#:lo12:.LANCHOR0+4044]
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	cbz	w0, .L3234
	adrp	x21, .LANCHOR4
	adrp	x19, .LANCHOR2
	add	x0, x21, :lo12:.LANCHOR4
	add	x20, x19, :lo12:.LANCHOR2
	adrp	x24, .LC159
	add	x26, x20, 3616
	add	x25, x20, 3664
	ldrh	w23, [x0,4074]
	add	x0, x24, :lo12:.LC159
	ldr	x2, [x20,1192]
	ubfiz	x22, x23, 1, 16
	mov	w1, w23
	ldrh	w2, [x2,x22]
	bl	printk
	ldrh	w0, [x20,3616]
	bl	FtlGcRefreshOpenBlock
	ldrh	w0, [x20,3664]
	mov	w20, 4097
	bl	FtlGcRefreshOpenBlock
	mov	x0, x26
	bl	allocate_new_data_superblock
	mov	x0, x25
	bl	allocate_new_data_superblock
.L3236:
	subs	w20, w20, #1
	beq	.L3240
	mov	w0, 1
	mov	w1, w0
	bl	ftl_do_gc
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x0, [x0,1192]
	ldrh	w0, [x0,x22]
	cbnz	w0, .L3236
.L3240:
	add	x20, x19, :lo12:.LANCHOR2
	add	x0, x24, :lo12:.LC159
	mov	w1, w23
	ldr	x2, [x20,1192]
	ldrh	w2, [x2,x22]
	bl	printk
	ldr	x0, [x20,1192]
	ldrh	w0, [x0,x22]
	cbnz	w0, .L3238
	add	x0, x29, 80
	strh	w23, [x29,80]
	bl	make_superblock
	mov	x0, 0
	ldrh	w5, [x20,756]
	mov	w6, 65535
	mov	w20, w0
	mov	w7, 56
.L3241:
	cmp	w5, w0, uxth
	bls	.L3248
	add	x1, x29, 80
	add	x1, x1, x0, lsl 1
	ldrh	w2, [x1,16]
	cmp	w2, w6
	beq	.L3242
	add	x4, x19, :lo12:.LANCHOR2
	lsl	w2, w2, 10
	umull	x1, w20, w7
	add	w20, w20, 1
	ldr	x3, [x4,1024]
	uxth	w20, w20
	add	x3, x3, x1
	str	w2, [x3,4]
	ldr	x2, [x4,1024]
	add	x1, x2, x1
	str	xzr, [x1,8]
	str	xzr, [x1,16]
.L3242:
	add	x0, x0, 1
	b	.L3241
.L3248:
	add	x19, x19, :lo12:.LANCHOR2
	adrp	x0, .LC160
	mov	w1, w23
	add	x0, x0, :lo12:.LC160
	ldr	x2, [x19,1192]
	ldrh	w2, [x2,x22]
	bl	printk
	ldr	x0, [x19,1024]
	mov	w1, 0
	mov	w2, w20
	bl	FlashEraseBlocks
	ldr	x0, [x19,1024]
	mov	w1, 1
	mov	w2, w20
	bl	FlashEraseBlocks
.L3238:
	add	x21, x21, :lo12:.LANCHOR4
	mov	w0, -1
	strh	w0, [x21,4074]
.L3234:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 128
	ret
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
	.align	3
.LANCHOR3 = . + 0
	.type	__func__.19578, %object
	.size	__func__.19578, 11
__func__.19578:
	.string	"FtlMemInit"
	.zero	5
	.type	__func__.20320, %object
	.size	__func__.20320, 12
__func__.20320:
	.string	"FtlCheckVpc"
	.zero	4
	.type	__func__.20352, %object
	.size	__func__.20352, 17
__func__.20352:
	.string	"FtlDumpBlockInfo"
	.zero	7
	.type	__func__.20371, %object
	.size	__func__.20371, 16
__func__.20371:
	.string	"FtlScanAllBlock"
	.type	__func__.20619, %object
	.size	__func__.20619, 21
__func__.20619:
	.string	"FtlVpcCheckAndModify"
	.zero	3
	.type	__func__.19651, %object
	.size	__func__.19651, 8
__func__.19651:
	.string	"FtlInit"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.string	"BBT:"
.LC1:
	.string	"FlashEraseBlocks pageAddr error %x\n"
.LC2:
	.string	"otp error! %d"
.LC3:
	.string	"rr"
.LC4:
	.string	"%d statReg->V6.mtrans_cnt=%d flReg.V6.page_num=%d\n"
.LC5:
	.string	"nandc:"
.LC6:
	.string	"%d flReg.d32=%x %x\n"
.LC7:
	.string	"sdr read ok %x ecc=%d\n"
.LC8:
	.string	"sync para %d\n"
.LC9:
	.string	"TOG mode Read error %x %x\n"
.LC10:
	.string	"read retry status %x %x %x\n"
.LC11:
	.string	"micron RR %d row=%x,count %d,status=%d\n"
.LC12:
	.string	"ECC:%d\n"
.LC13:
	.string	"No.%d FLASH ID:%x %x %x %x %x %x\n"
.LC14:
	.string	"FlashLoadPhyInfo fail %x!!\n"
.LC15:
	.string	"Read pageadd=%x  ecc=%x err=%x\n"
.LC16:
	.string	"data:"
.LC17:
	.string	"spare:"
.LC18:
	.string	"ReadRetry pageadd=%x ecc=%x err=%x\n"
.LC19:
	.string	"FLFB:%d %d\n"
.LC20:
	.string	"prog error: = %x\n"
.LC21:
	.string	"prog read error: = %x\n"
.LC22:
	.string	"prog read s error: = %x %x %x\n"
.LC23:
	.string	"prog read d error: = %x %x %x\n"
.LC24:
	.string	"FlashMakeFactorBbt %d\n"
.LC25:
	.string	"bad block:%d %d\n"
.LC26:
	.string	"FMFB:%d %d\n"
.LC27:
	.string	"E:bad block:%d\n"
.LC28:
	.string	"FMFB:Save %d %d\n"
.LC29:
	.string	"%s error allocating memory. return -1\n"
.LC30:
	.string	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\n"
.LC31:
	.string	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\n"
.LC32:
	.string	"FtlBbmTblFlush error:%x\n"
.LC33:
	.string	"FtlBbmTblFlush error = %x error count = %d\n"
.LC34:
	.string	"FtlFreeSysBlkQueueOut free count = %d\n"
.LC35:
	.string	"FtlFreeSysBlkQueueOut = %x, free count = %d, error\n"
.LC36:
	.string	"FtlFreeSysBlkQueueOut = %x, free count = %d\n"
.LC37:
	.string	"FtlMapWritePage error = %x\n"
.LC38:
	.string	"FtlMapWritePage error = %x error count = %d\n"
.LC39:
	.string	"page map lost: %x %x\n"
.LC40:
	.string	"region_id = %x phyAddr = %x\n"
.LC41:
	.string	"map_ppn:"
.LC42:
	.string	"load_l2p_region refresh = %x phyAddr = %x\n"
.LC43:
	.string	"FtlVendorPartRead refresh = %x phyAddr = %x\n"
.LC44:
	.string	"FtlVpcTblFlush error = %x error count = %d\n"
.LC45:
	.string	"%s\n"
.LC46:
	.string	"no ect"
.LC47:
	.string	"...%s enter...\n"
.LC48:
	.string	"vpc1"
.LC49:
	.string	"vpc2"
.LC50:
	.string	"FtlCheckVpc2 %x = %x  %x\n"
.LC51:
	.string	"free blk vpc error %x = %x  %x\n"
.LC52:
	.string	"id = %x,%x addr= %x,spare= %x %x %x %x data = %x\n"
.LC53:
	.string	":"
.LC54:
	.string	"Ftlscanalldata = %x\n"
.LC55:
	.string	"scan lpa = %x ppa= %x\n"
.LC56:
	.string	"lba = %x,addr= %x,spare= %x %x %x %x data=%x %x\n"
.LC57:
	.string	"phyBlk = %x,addr= %x,spare= %x %x %x %x data=%x %x\n"
.LC58:
	.string	"id = %x,%x addr= %x,spare= %x %x %x %x data=%x %x\n"
.LC59:
	.string	"Mblk:"
.LC60:
	.string	"L2P:"
.LC61:
	.string	"L2PC:"
.LC62:
	.string	"superBlkID = %x vpc=%x\n"
.LC63:
	.string	"flashmode = %x pagenum = %x %x\n"
.LC64:
	.string	"blk = %x vpc=%x mode = %x\n"
.LC65:
	.string	"mlc id = %x,%x addr= %x,spare= %x %x %x %x data=%x %x\n"
.LC66:
	.string	"slc id = %x,%x addr= %x,spare= %x %x %x %x data=%x %x\n"
.LC67:
	.string	"slc mode"
.LC68:
	.string	"FLASH INFO:\n"
.LC69:
	.string	"FLASH ID: %x\n"
.LC70:
	.string	"Device Capacity: %d MB\n"
.LC71:
	.string	"FMWAIT: %x %x %x %x\n"
.LC72:
	.string	"FTL INFO:\n"
.LC73:
	.string	"g_MaxLpn = 0x%x\n"
.LC74:
	.string	"g_VaildLpn = 0x%x\n"
.LC75:
	.string	"read_page_count = 0x%x\n"
.LC76:
	.string	"discard_page_count = 0x%x\n"
.LC77:
	.string	"write_page_count = 0x%x\n"
.LC78:
	.string	"cache_write_count = 0x%x\n"
.LC79:
	.string	"l2p_write_count = 0x%x\n"
.LC80:
	.string	"gc_page_count = 0x%x\n"
.LC81:
	.string	"totle_write = %d MB\n"
.LC82:
	.string	"totle_read = %d MB\n"
.LC83:
	.string	"GSV = 0x%x\n"
.LC84:
	.string	"GDV = 0x%x\n"
.LC85:
	.string	"bad blk num = %d %d\n"
.LC86:
	.string	"free_superblocks = 0x%x\n"
.LC87:
	.string	"mlc_EC = 0x%x\n"
.LC88:
	.string	"slc_EC = 0x%x\n"
.LC89:
	.string	"avg_EC = 0x%x\n"
.LC90:
	.string	"sys_EC = 0x%x\n"
.LC91:
	.string	"max_EC = 0x%x\n"
.LC92:
	.string	"min_EC = 0x%x\n"
.LC93:
	.string	"PLT = 0x%x\n"
.LC94:
	.string	"POT = 0x%x\n"
.LC95:
	.string	"MaxSector = 0x%x\n"
.LC96:
	.string	"init_sys_blks_pp = 0x%x\n"
.LC97:
	.string	"sys_blks_pp = 0x%x\n"
.LC98:
	.string	"free sysblock = 0x%x\n"
.LC99:
	.string	"data_blks_pp = 0x%x\n"
.LC100:
	.string	"data_op_blks_pp = 0x%x\n"
.LC101:
	.string	"max_data_blks = 0x%x\n"
.LC102:
	.string	"Sys.id = 0x%x\n"
.LC103:
	.string	"Bbt.id = 0x%x\n"
.LC104:
	.string	"ACT.page = 0x%x\n"
.LC105:
	.string	"ACT.plane = 0x%x\n"
.LC106:
	.string	"ACT.id = 0x%x\n"
.LC107:
	.string	"ACT.mode = 0x%x\n"
.LC108:
	.string	"ACT.a_pages = 0x%x\n"
.LC109:
	.string	"ACT VPC = 0x%x\n"
.LC110:
	.string	"BUF.page = 0x%x\n"
.LC111:
	.string	"BUF.plane = 0x%x\n"
.LC112:
	.string	"BUF.id = 0x%x\n"
.LC113:
	.string	"BUF.mode = 0x%x\n"
.LC114:
	.string	"BUF.a_pages = 0x%x\n"
.LC115:
	.string	"BUF VPC = 0x%x\n"
.LC116:
	.string	"TMP.page = 0x%x\n"
.LC117:
	.string	"TMP.plane = 0x%x\n"
.LC118:
	.string	"TMP.id = 0x%x\n"
.LC119:
	.string	"TMP.mode = 0x%x\n"
.LC120:
	.string	"TMP.a_pages = 0x%x\n"
.LC121:
	.string	"GC.page = 0x%x\n"
.LC122:
	.string	"GC.plane = 0x%x\n"
.LC123:
	.string	"GC.id = 0x%x\n"
.LC124:
	.string	"GC.mode = 0x%x\n"
.LC125:
	.string	"GC.a_pages = 0x%x\n"
.LC126:
	.string	"WR_CHK = 0x%x %x %x %x\n"
.LC127:
	.string	"Read Err = 0x%x\n"
.LC128:
	.string	"Prog Err = 0x%x\n"
.LC129:
	.string	"gc_free_blk_th= 0x%x\n"
.LC130:
	.string	"gc_merge_free_blk_th= 0x%x\n"
.LC131:
	.string	"gc_skip_write_count= 0x%x\n"
.LC132:
	.string	"gc_blk_index= 0x%x\n"
.LC133:
	.string	"free min EC= 0x%x\n"
.LC134:
	.string	"free max EC= 0x%x\n"
.LC135:
	.string	"GC__SB VPC = 0x%x\n"
.LC136:
	.string	"%d. [0x%x]=0x%x 0x%x  0x%x\n"
.LC137:
	.string	"free %d. [0x%x] 0x%x  0x%x\n"
.LC138:
	.string	"FTL version: 5.0.53 20190107"
.LC139:
	.string	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x\n"
.LC140:
	.string	"FtlGcScanTempBlk Error ID %x %x!!!!!!! \n"
.LC141:
	.string	"FtlGcScanTempBlkError ID %x %x!!!!!!!\n"
.LC142:
	.string	"FtlGcRefreshBlock  0x%x\n"
.LC143:
	.string	"FtlGcMarkBadPhyBlk %d 0x%x\n"
.LC144:
	.string	"FtlGcFreeBadSuperBlk 0x%x\n"
.LC145:
	.string	"decrement_vpc_count %x = %d\n"
.LC146:
	.string	"decrement_vpc_count %x = %d in free list\n"
.LC147:
	.string	"RSB refresh addr %x\n"
.LC148:
	.string	"spuer block %x vpn is 0\n "
.LC149:
	.string	"g_recovery_ppa %x ver %x\n "
.LC150:
	.string	"FtlCheckVpc %x = %x  %x\n"
.LC151:
	.string	"%d GC datablk  = %x vpc %x %x\n"
.LC152:
	.string	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\n"
.LC153:
	.string	"g_gc_superblock_free %x %x %x %x %x\n"
.LC154:
	.string	"Ftlwrite decrement_vpc_count %x = %d\n"
.LC155:
	.string	"GC des block %x done\n"
.LC156:
	.string	"...%s: no bad block mapping table, format device\n"
.LC157:
	.string	"...%s FtlSysBlkInit error ,format device!\n"
.LC158:
	.string	"FtlWrite: lpa error:%x %x\n"
.LC159:
	.string	"fix power lost blk = %x vpc=%x\n"
.LC160:
	.string	"erase power lost blk = %x vpc=%x\n"
	.data
	.align	3
.LANCHOR1 = . + 0
	.type	random_seed, %object
	.size	random_seed, 256
random_seed:
	.hword	22378
	.hword	1512
	.hword	25245
	.hword	17827
	.hword	25756
	.hword	19440
	.hword	9026
	.hword	10030
	.hword	29528
	.hword	20467
	.hword	29676
	.hword	24432
	.hword	31328
	.hword	6872
	.hword	13426
	.hword	13842
	.hword	8783
	.hword	1108
	.hword	782
	.hword	28837
	.hword	30729
	.hword	9505
	.hword	18676
	.hword	23085
	.hword	18730
	.hword	1085
	.hword	32609
	.hword	14697
	.hword	20858
	.hword	15170
	.hword	30365
	.hword	1607
	.hword	32298
	.hword	4995
	.hword	18905
	.hword	1976
	.hword	9592
	.hword	20204
	.hword	17443
	.hword	13615
	.hword	23330
	.hword	29369
	.hword	13947
	.hword	9398
	.hword	32398
	.hword	8984
	.hword	27600
	.hword	21785
	.hword	6019
	.hword	6311
	.hword	31598
	.hword	30210
	.hword	19327
	.hword	13896
	.hword	11347
	.hword	27545
	.hword	3107
	.hword	26575
	.hword	32270
	.hword	19852
	.hword	20601
	.hword	8349
	.hword	9290
	.hword	29819
	.hword	13579
	.hword	3661
	.hword	28676
	.hword	27331
	.hword	32574
	.hword	8693
	.hword	31253
	.hword	9081
	.hword	5399
	.hword	6842
	.hword	20087
	.hword	5537
	.hword	1274
	.hword	11617
	.hword	9530
	.hword	4866
	.hword	8035
	.hword	23219
	.hword	1178
	.hword	23272
	.hword	7383
	.hword	18944
	.hword	12488
	.hword	12871
	.hword	29340
	.hword	20532
	.hword	11022
	.hword	22514
	.hword	228
	.hword	22363
	.hword	24978
	.hword	14584
	.hword	12138
	.hword	3092
	.hword	17916
	.hword	16863
	.hword	14554
	.hword	31457
	.hword	29474
	.hword	25311
	.hword	24121
	.hword	3684
	.hword	28037
	.hword	22865
	.hword	22839
	.hword	25217
	.hword	13217
	.hword	27186
	.hword	14938
	.hword	11180
	.hword	29754
	.hword	24180
	.hword	15150
	.hword	32455
	.hword	20434
	.hword	23848
	.hword	29983
	.hword	16120
	.hword	14769
	.hword	20041
	.hword	29803
	.hword	28406
	.hword	17598
	.hword	28087
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
	.zero	3
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
	.zero	1
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
	.type	gNandParaInfo, %object
	.size	gNandParaInfo, 32
gNandParaInfo:
	.byte	0
	.byte	0
	.zero	5
	.byte	0
	.byte	1
	.byte	8
	.hword	128
	.byte	2
	.byte	1
	.hword	2048
	.hword	0
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	479
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1064
	.hword	479
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	287
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	287
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	287
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	287
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1024
	.hword	287
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	2
	.hword	1024
	.hword	479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	2
	.hword	1024
	.hword	479
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	4096
	.hword	279
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1024
	.hword	287
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	287
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	287
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
	.zero	4
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
	.hword	128
	.byte	1
	.byte	2
	.hword	2048
	.hword	287
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	2
	.hword	700
	.hword	479
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	479
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
	.zero	4
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
	.hword	64
	.byte	1
	.byte	2
	.hword	2048
	.hword	279
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
	.zero	4
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
	.hword	64
	.byte	1
	.byte	2
	.hword	1024
	.hword	279
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	1024
	.hword	479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	2184
	.hword	1479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	1048
	.hword	1479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	1044
	.hword	479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	2184
	.hword	1479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	2184
	.hword	1479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	2184
	.hword	1479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	2184
	.hword	1479
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	2
	.hword	1024
	.hword	479
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1024
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1046
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2090
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1066
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	530
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1024
	.hword	281
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1060
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1056
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1060
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1056
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2092
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	1024
	.hword	273
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
	.zero	4
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
	.hword	388
	.byte	2
	.byte	2
	.hword	1362
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	479
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	479
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	279
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	279
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	279
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	279
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2048
	.hword	279
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
	.zero	4
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
	.hword	512
	.byte	2
	.byte	1
	.hword	1024
	.hword	455
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	1060
	.hword	1473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	2092
	.hword	1473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	2092
	.hword	1505
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	1
	.hword	2056
	.hword	1473
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	2058
	.hword	1489
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2062
	.hword	1489
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1066
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1066
	.hword	1497
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	2050
	.hword	401
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1058
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	1060
	.hword	1473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1074
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2106
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	1056
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1058
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	1060
	.hword	1473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2082
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1066
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	1060
	.hword	1473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2090
	.hword	1241
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	1
	.hword	2092
	.hword	1473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2106
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1074
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2138
	.hword	1497
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	2062
	.hword	473
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
	.zero	4
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
	.hword	256
	.byte	2
	.byte	2
	.hword	1058
	.hword	1497
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	2048
	.hword	473
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	2048
	.hword	505
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	2076
	.hword	409
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	2076
	.hword	441
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	1038
	.hword	281
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
	.zero	4
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
	.hword	128
	.byte	2
	.byte	2
	.hword	2076
	.hword	1169
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
	.zero	4
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
	.zero	14
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
	.zero	14
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
	.zero	14
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
	.zero	14
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
	.zero	4
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
	.hword	64
	.byte	1
	.byte	1
	.hword	1024
	.hword	256
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
	.zero	4
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
	.bss
	.align	3
.LANCHOR0 = . + 0
.LANCHOR2 = . + 4352
.LANCHOR4 = . + 13056
.LANCHOR5 = . + 17408
	.type	gFlashPageBuffer0, %object
	.size	gFlashPageBuffer0, 8
gFlashPageBuffer0:
	.zero	8
	.type	gNandFlashIdbBlockAddr, %object
	.size	gNandFlashIdbBlockAddr, 4
gNandFlashIdbBlockAddr:
	.zero	4
	.zero	4
	.type	p_blk_mode_table, %object
	.size	p_blk_mode_table, 8
p_blk_mode_table:
	.zero	8
	.type	g_slc2KBNand, %object
	.size	g_slc2KBNand, 1
g_slc2KBNand:
	.zero	1
	.type	gNandIDBResBlkNum, %object
	.size	gNandIDBResBlkNum, 1
gNandIDBResBlkNum:
	.zero	1
	.zero	2
	.type	gBlockPageAlignSize, %object
	.size	gBlockPageAlignSize, 4
gBlockPageAlignSize:
	.zero	4
	.type	gNandRandomizer, %object
	.size	gNandRandomizer, 1
gNandRandomizer:
	.zero	1
	.zero	7
	.type	gNandChipMap, %object
	.size	gNandChipMap, 64
gNandChipMap:
	.zero	64
	.type	gNandOptPara, %object
	.size	gNandOptPara, 32
gNandOptPara:
	.zero	32
	.type	slcPageToMlcPageTbl, %object
	.size	slcPageToMlcPageTbl, 512
slcPageToMlcPageTbl:
	.zero	512
	.type	mlcPageToSlcPageTbl, %object
	.size	mlcPageToSlcPageTbl, 1024
mlcPageToSlcPageTbl:
	.zero	1024
	.type	gNandMaxDie, %object
	.size	gNandMaxDie, 1
gNandMaxDie:
	.zero	1
	.type	gNandMaxChip, %object
	.size	gNandMaxChip, 1
gNandMaxChip:
	.zero	1
	.zero	2
	.type	DieCsIndex, %object
	.size	DieCsIndex, 8
DieCsIndex:
	.zero	8
	.type	DieAddrs, %object
	.size	DieAddrs, 32
DieAddrs:
	.zero	32
	.type	gDieOp, %object
	.size	gDieOp, 192
gDieOp:
	.zero	192
	.zero	4
	.type	gpNandParaInfo, %object
	.size	gpNandParaInfo, 8
gpNandParaInfo:
	.zero	8
	.type	gTotleBlock, %object
	.size	gTotleBlock, 2
gTotleBlock:
	.zero	2
	.zero	6
	.type	gNandIDataBuf, %object
	.size	gNandIDataBuf, 2048
gNandIDataBuf:
	.zero	2048
	.type	gpNandc, %object
	.size	gpNandc, 8
gpNandc:
	.zero	8
	.type	NANDC_FMCTL, %object
	.size	NANDC_FMCTL, 4
NANDC_FMCTL:
	.zero	4
	.type	NANDC_FMWAIT, %object
	.size	NANDC_FMWAIT, 4
NANDC_FMWAIT:
	.zero	4
	.type	NANDC_FLCTL, %object
	.size	NANDC_FLCTL, 4
NANDC_FLCTL:
	.zero	4
	.type	NANDC_BCHCTL, %object
	.size	NANDC_BCHCTL, 4
NANDC_BCHCTL:
	.zero	4
	.type	NANDC_DLL_CTL_REG0, %object
	.size	NANDC_DLL_CTL_REG0, 4
NANDC_DLL_CTL_REG0:
	.zero	4
	.type	NANDC_DLL_CTL_REG1, %object
	.size	NANDC_DLL_CTL_REG1, 4
NANDC_DLL_CTL_REG1:
	.zero	4
	.type	NANDC_RANDMZ_CFG, %object
	.size	NANDC_RANDMZ_CFG, 4
NANDC_RANDMZ_CFG:
	.zero	4
	.type	NANDC_FMWAIT_SYN, %object
	.size	NANDC_FMWAIT_SYN, 4
NANDC_FMWAIT_SYN:
	.zero	4
	.type	gNandPhyInfo, %object
	.size	gNandPhyInfo, 28
gNandPhyInfo:
	.zero	28
	.type	gFlashSlcMode, %object
	.size	gFlashSlcMode, 1
gFlashSlcMode:
	.zero	1
	.type	gNandFlashEccBits, %object
	.size	gNandFlashEccBits, 1
gNandFlashEccBits:
	.zero	1
	.zero	2
	.type	g_MaxLbaSector, %object
	.size	g_MaxLbaSector, 4
g_MaxLbaSector:
	.zero	4
	.type	FlashWaitBusyScheduleEn, %object
	.size	FlashWaitBusyScheduleEn, 4
FlashWaitBusyScheduleEn:
	.zero	4
	.type	gReadRetryInfo, %object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.zero	852
	.zero	4
	.type	read_retry_cur_offset, %object
	.size	read_retry_cur_offset, 4
read_retry_cur_offset:
	.zero	4
	.type	gFlashInterfaceMode, %object
	.size	gFlashInterfaceMode, 1
gFlashInterfaceMode:
	.zero	1
	.zero	3
	.type	idb_flash_slc_mode, %object
	.size	idb_flash_slc_mode, 4
idb_flash_slc_mode:
	.zero	4
	.type	gFlashToggleModeEn, %object
	.size	gFlashToggleModeEn, 1
gFlashToggleModeEn:
	.zero	1
	.zero	3
	.type	gBootDdrMode, %object
	.size	gBootDdrMode, 4
gBootDdrMode:
	.zero	4
	.type	gNandcVer, %object
	.size	gNandcVer, 4
gNandcVer:
	.zero	4
	.type	g_nandc_version_data, %object
	.size	g_nandc_version_data, 4
g_nandc_version_data:
	.zero	4
	.zero	4
	.type	gMasterTempBuf, %object
	.size	gMasterTempBuf, 8
gMasterTempBuf:
	.zero	8
	.type	gMasterInfo, %object
	.size	gMasterInfo, 48
gMasterInfo:
	.zero	48
	.type	gNandcDumpWriteEn, %object
	.size	gNandcDumpWriteEn, 4
gNandcDumpWriteEn:
	.zero	4
	.type	gNandcEccBits, %object
	.size	gNandcEccBits, 4
gNandcEccBits:
	.zero	4
	.type	gMultiPageProgEn, %object
	.size	gMultiPageProgEn, 1
gMultiPageProgEn:
	.zero	1
	.zero	3
	.type	g_inkDie_check_enable, %object
	.size	g_inkDie_check_enable, 4
g_inkDie_check_enable:
	.zero	4
	.type	FlashDdrTunningReadCount, %object
	.size	FlashDdrTunningReadCount, 4
FlashDdrTunningReadCount:
	.zero	4
	.zero	4
	.type	gpReadRetrial, %object
	.size	gpReadRetrial, 8
gpReadRetrial:
	.zero	8
	.type	gpFlashSaveInfo, %object
	.size	gpFlashSaveInfo, 8
gpFlashSaveInfo:
	.zero	8
	.type	gNandFlashInfoBlockAddr, %object
	.size	gNandFlashInfoBlockAddr, 4
gNandFlashInfoBlockAddr:
	.zero	4
	.type	gNandIDBResBlkNumSaveInFlash, %object
	.size	gNandIDBResBlkNumSaveInFlash, 1
gNandIDBResBlkNumSaveInFlash:
	.zero	1
	.type	gNandFlashIDBEccBits, %object
	.size	gNandFlashIDBEccBits, 1
gNandFlashIDBEccBits:
	.zero	1
	.zero	2
	.type	gFlashPageBuffer1, %object
	.size	gFlashPageBuffer1, 8
gFlashPageBuffer1:
	.zero	8
	.type	gFlashSpareBuffer, %object
	.size	gFlashSpareBuffer, 8
gFlashSpareBuffer:
	.zero	8
	.type	gFlashProgCheckBuffer, %object
	.size	gFlashProgCheckBuffer, 8
gFlashProgCheckBuffer:
	.zero	8
	.type	gFlashProgCheckSpareBuffer, %object
	.size	gFlashProgCheckSpareBuffer, 8
gFlashProgCheckSpareBuffer:
	.zero	8
	.type	gMultiPageReadEn, %object
	.size	gMultiPageReadEn, 1
gMultiPageReadEn:
	.zero	1
	.zero	7
	.type	FbbtBlk, %object
	.size	FbbtBlk, 16
FbbtBlk:
	.zero	16
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
	.type	c_ftl_nand_sec_pre_page, %object
	.size	c_ftl_nand_sec_pre_page, 2
c_ftl_nand_sec_pre_page:
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
	.type	g_page_map_check_enable, %object
	.size	g_page_map_check_enable, 2
g_page_map_check_enable:
	.zero	2
	.type	g_free_slc_blk_num, %object
	.size	g_free_slc_blk_num, 2
g_free_slc_blk_num:
	.zero	2
	.type	g_SlcPartLbaEndSector, %object
	.size	g_SlcPartLbaEndSector, 4
g_SlcPartLbaEndSector:
	.zero	4
	.type	g_all_blk_used_slc_mode, %object
	.size	g_all_blk_used_slc_mode, 4
g_all_blk_used_slc_mode:
	.zero	4
	.type	g_GlobalSysVersion, %object
	.size	g_GlobalSysVersion, 4
g_GlobalSysVersion:
	.zero	4
	.type	g_GlobalDataVersion, %object
	.size	g_GlobalDataVersion, 4
g_GlobalDataVersion:
	.zero	4
	.type	g_totle_gc_page_count, %object
	.size	g_totle_gc_page_count, 4
g_totle_gc_page_count:
	.zero	4
	.type	g_totle_write_page_count, %object
	.size	g_totle_write_page_count, 4
g_totle_write_page_count:
	.zero	4
	.type	g_totle_discard_page_count, %object
	.size	g_totle_discard_page_count, 4
g_totle_discard_page_count:
	.zero	4
	.type	g_totle_cache_write_count, %object
	.size	g_totle_cache_write_count, 4
g_totle_cache_write_count:
	.zero	4
	.type	g_totle_l2p_write_count, %object
	.size	g_totle_l2p_write_count, 4
g_totle_l2p_write_count:
	.zero	4
	.type	g_totle_read_page_count, %object
	.size	g_totle_read_page_count, 4
g_totle_read_page_count:
	.zero	4
	.type	g_totle_mlc_erase_count, %object
	.size	g_totle_mlc_erase_count, 4
g_totle_mlc_erase_count:
	.zero	4
	.type	g_totle_slc_erase_count, %object
	.size	g_totle_slc_erase_count, 4
g_totle_slc_erase_count:
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
	.type	g_in_gc_progress, %object
	.size	g_in_gc_progress, 4
g_in_gc_progress:
	.zero	4
	.type	g_in_swl_replace, %object
	.size	g_in_swl_replace, 4
g_in_swl_replace:
	.zero	4
	.type	g_gc_head_data_block, %object
	.size	g_gc_head_data_block, 4
g_gc_head_data_block:
	.zero	4
	.type	g_gc_head_data_block_count, %object
	.size	g_gc_head_data_block_count, 4
g_gc_head_data_block_count:
	.zero	4
	.type	g_gc_skip_write_count, %object
	.size	g_gc_skip_write_count, 4
g_gc_skip_write_count:
	.zero	4
	.type	g_cur_erase_blk, %object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.zero	4
	.type	g_gc_next_blk, %object
	.size	g_gc_next_blk, 2
g_gc_next_blk:
	.zero	2
	.type	g_gc_next_blk_1, %object
	.size	g_gc_next_blk_1, 2
g_gc_next_blk_1:
	.zero	2
	.type	g_gc_next_blk_2, %object
	.size	g_gc_next_blk_2, 2
g_gc_next_blk_2:
	.zero	2
	.type	g_gc_next_blk_3, %object
	.size	g_gc_next_blk_3, 2
g_gc_next_blk_3:
	.zero	2
	.type	g_gc_free_blk_threshold, %object
	.size	g_gc_free_blk_threshold, 2
g_gc_free_blk_threshold:
	.zero	2
	.type	g_gc_merge_free_blk_threshold, %object
	.size	g_gc_merge_free_blk_threshold, 2
g_gc_merge_free_blk_threshold:
	.zero	2
	.type	g_gc_blk_index, %object
	.size	g_gc_blk_index, 2
g_gc_blk_index:
	.zero	2
	.type	g_gc_bad_block_temp_num, %object
	.size	g_gc_bad_block_temp_num, 2
g_gc_bad_block_temp_num:
	.zero	2
	.type	g_gc_refresh_block_temp_num, %object
	.size	g_gc_refresh_block_temp_num, 2
g_gc_refresh_block_temp_num:
	.zero	2
	.type	g_gc_bad_block_gc_index, %object
	.size	g_gc_bad_block_gc_index, 2
g_gc_bad_block_gc_index:
	.zero	2
	.type	c_wr_page_buf_num, %object
	.size	c_wr_page_buf_num, 4
c_wr_page_buf_num:
	.zero	4
	.type	g_wr_page_num, %object
	.size	g_wr_page_num, 4
g_wr_page_num:
	.zero	4
	.zero	4
	.type	p_gc_blk_tbl, %object
	.size	p_gc_blk_tbl, 8
p_gc_blk_tbl:
	.zero	8
	.type	p_gc_page_info, %object
	.size	p_gc_page_info, 8
p_gc_page_info:
	.zero	8
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
	.type	req_erase, %object
	.size	req_erase, 8
req_erase:
	.zero	8
	.type	req_gc, %object
	.size	req_gc, 8
req_gc:
	.zero	8
	.type	req_wr_io, %object
	.size	req_wr_io, 8
req_wr_io:
	.zero	8
	.type	c_gc_page_buf_num, %object
	.size	c_gc_page_buf_num, 4
c_gc_page_buf_num:
	.zero	4
	.zero	4
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
	.type	p_gc_data_buf, %object
	.size	p_gc_data_buf, 8
p_gc_data_buf:
	.zero	8
	.type	p_wr_io_data_buf, %object
	.size	p_wr_io_data_buf, 8
p_wr_io_data_buf:
	.zero	8
	.type	p_io_data_buf_0, %object
	.size	p_io_data_buf_0, 8
p_io_data_buf_0:
	.zero	8
	.type	p_io_data_buf_1, %object
	.size	p_io_data_buf_1, 8
p_io_data_buf_1:
	.zero	8
	.type	gp_gc_page_buf_info, %object
	.size	gp_gc_page_buf_info, 8
gp_gc_page_buf_info:
	.zero	8
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 8
p_sys_spare_buf:
	.zero	8
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 8
p_io_spare_buf:
	.zero	8
	.type	p_gc_spare_buf, %object
	.size	p_gc_spare_buf, 8
p_gc_spare_buf:
	.zero	8
	.type	p_wr_io_spare_buf, %object
	.size	p_wr_io_spare_buf, 8
p_wr_io_spare_buf:
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
	.type	p_erase_count_table, %object
	.size	p_erase_count_table, 8
p_erase_count_table:
	.zero	8
	.type	p_valid_page_count_check_table, %object
	.size	p_valid_page_count_check_table, 8
p_valid_page_count_check_table:
	.zero	8
	.type	p_valid_page_count_table, %object
	.size	p_valid_page_count_table, 8
p_valid_page_count_table:
	.zero	8
	.type	p_map_block_table, %object
	.size	p_map_block_table, 8
p_map_block_table:
	.zero	8
	.type	p_map_block_valid_page_count, %object
	.size	p_map_block_valid_page_count, 8
p_map_block_valid_page_count:
	.zero	8
	.type	p_vendor_block_table, %object
	.size	p_vendor_block_table, 8
p_vendor_block_table:
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
	.type	p_map_region_ppn_check_table, %object
	.size	p_map_region_ppn_check_table, 8
p_map_region_ppn_check_table:
	.zero	8
	.type	p_map_block_ver_table, %object
	.size	p_map_block_ver_table, 8
p_map_block_ver_table:
	.zero	8
	.type	p_l2p_ram_map, %object
	.size	p_l2p_ram_map, 8
p_l2p_ram_map:
	.zero	8
	.type	p_l2p_map_buf, %object
	.size	p_l2p_map_buf, 8
p_l2p_map_buf:
	.zero	8
	.type	p_data_block_list_table, %object
	.size	p_data_block_list_table, 8
p_data_block_list_table:
	.zero	8
	.type	c_ftl_nand_bbm_buf_size, %object
	.size	c_ftl_nand_bbm_buf_size, 2
c_ftl_nand_bbm_buf_size:
	.zero	2
	.zero	6
	.type	gBbtInfo, %object
	.size	gBbtInfo, 96
gBbtInfo:
	.zero	96
	.type	g_totle_vendor_block, %object
	.size	g_totle_vendor_block, 2
g_totle_vendor_block:
	.zero	2
	.zero	2
	.type	g_MaxLpn, %object
	.size	g_MaxLpn, 4
g_MaxLpn:
	.zero	4
	.type	g_flash_read_only_en, %object
	.size	g_flash_read_only_en, 4
g_flash_read_only_en:
	.zero	4
	.zero	4
	.type	req_sys, %object
	.size	req_sys, 56
req_sys:
	.zero	56
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.zero	2056
	.type	g_sys_save_data, %object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.zero	48
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
	.type	g_num_free_superblocks, %object
	.size	g_num_free_superblocks, 2
g_num_free_superblocks:
	.zero	2
	.zero	6
	.type	g_active_superblock, %object
	.size	g_active_superblock, 48
g_active_superblock:
	.zero	48
	.type	g_buffer_superblock, %object
	.size	g_buffer_superblock, 48
g_buffer_superblock:
	.zero	48
	.type	g_gc_temp_superblock, %object
	.size	g_gc_temp_superblock, 48
g_gc_temp_superblock:
	.zero	48
	.type	gL2pMapInfo, %object
	.size	gL2pMapInfo, 64
gL2pMapInfo:
	.zero	64
	.type	g_l2p_last_update_region_id, %object
	.size	g_l2p_last_update_region_id, 2
g_l2p_last_update_region_id:
	.zero	2
	.zero	6
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 64
gVendorBlkInfo:
	.zero	64
	.type	FtlUpdateVaildLpnCount, %object
	.size	FtlUpdateVaildLpnCount, 2
FtlUpdateVaildLpnCount:
	.zero	2
	.zero	2
	.type	g_VaildLpn, %object
	.size	g_VaildLpn, 4
g_VaildLpn:
	.zero	4
	.type	gSysInfo, %object
	.size	gSysInfo, 16
gSysInfo:
	.zero	16
	.type	g_totle_map_block, %object
	.size	g_totle_map_block, 2
g_totle_map_block:
	.zero	2
	.zero	6
	.type	check_valid_page_count_table, %object
	.size	check_valid_page_count_table, 8192
check_valid_page_count_table:
	.zero	8192
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.zero	4
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.zero	2
	.zero	2
	.type	g_gc_superblock, %object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.zero	48
	.type	g_sys_ext_data, %object
	.size	g_sys_ext_data, 512
g_sys_ext_data:
	.zero	512
	.type	g_totle_write_sector, %object
	.size	g_totle_write_sector, 4
g_totle_write_sector:
	.zero	4
	.type	g_totle_read_sector, %object
	.size	g_totle_read_sector, 4
g_totle_read_sector:
	.zero	4
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.zero	2
	.zero	2
	.type	g_totle_avg_erase_count, %object
	.size	g_totle_avg_erase_count, 4
g_totle_avg_erase_count:
	.zero	4
	.type	g_gc_num_req, %object
	.size	g_gc_num_req, 4
g_gc_num_req:
	.zero	4
	.zero	4
	.type	g_req_cache, %object
	.size	g_req_cache, 8
g_req_cache:
	.zero	8
	.type	g_tmp_data_superblock_id, %object
	.size	g_tmp_data_superblock_id, 2
g_tmp_data_superblock_id:
	.zero	2
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
	.type	g_gc_blk_num, %object
	.size	g_gc_blk_num, 2
g_gc_blk_num:
	.zero	2
	.type	g_gc_page_offset, %object
	.size	g_gc_page_offset, 2
g_gc_page_offset:
	.zero	2
	.zero	4
	.type	g_gc_bad_block_temp_tbl, %object
	.size	g_gc_bad_block_temp_tbl, 34
g_gc_bad_block_temp_tbl:
	.zero	34
	.type	g_power_lost_ecc_error_blk, %object
	.size	g_power_lost_ecc_error_blk, 2
g_power_lost_ecc_error_blk:
	.zero	2
	.type	g_power_lost_recovery_flag, %object
	.size	g_power_lost_recovery_flag, 2
g_power_lost_recovery_flag:
	.zero	2
	.zero	2
	.type	g_recovery_page_num, %object
	.size	g_recovery_page_num, 4
g_recovery_page_num:
	.zero	4
	.zero	4
	.type	g_recovery_ppa_tbl, %object
	.size	g_recovery_ppa_tbl, 128
g_recovery_ppa_tbl:
	.zero	128
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
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 8
gp_last_act_superblock:
	.zero	8
	.type	gc_discard_updated, %object
	.size	gc_discard_updated, 4
gc_discard_updated:
	.zero	4
	.type	g_LowFormat, %object
	.size	g_LowFormat, 4
g_LowFormat:
	.zero	4
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.zero	4
	.type	last_cache_match_count, %object
	.size	last_cache_match_count, 4
last_cache_match_count:
	.zero	4
	.type	g_gc_refresh_block_temp_tbl, %object
	.size	g_gc_refresh_block_temp_tbl, 34
g_gc_refresh_block_temp_tbl:
	.zero	34
	.zero	2
	.type	gToggleModeClkDiv, %object
	.size	gToggleModeClkDiv, 4
gToggleModeClkDiv:
	.zero	4
	.type	gpNandc1, %object
	.size	gpNandc1, 8
gpNandc1:
	.zero	8
	.type	gNandFlashResEndPageAddr, %object
	.size	gNandFlashResEndPageAddr, 4
gNandFlashResEndPageAddr:
	.zero	4
	.type	gNandFlashInfoBlockEcc, %object
	.size	gNandFlashInfoBlockEcc, 1
gNandFlashInfoBlockEcc:
	.zero	1
	.type	gFlashOnfiModeEn, %object
	.size	gFlashOnfiModeEn, 1
gFlashOnfiModeEn:
	.zero	1
	.type	gFlashSdrModeEn, %object
	.size	gFlashSdrModeEn, 1
gFlashSdrModeEn:
	.zero	1
