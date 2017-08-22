/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2017-08-16
 */
	.cpu generic+fp+simd
	.file	"rk_ftl_arm_v8.S"
#APP
	.irp	num,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17\
		,18,19,20,21,22,23,24,25,26,27,28,29,30
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
	.type	ftl_set_blk_mode.part.7, %function
ftl_set_blk_mode.part.7:
	adrp	x1, .LANCHOR0
	uxth	w0, w0
	ubfx	x2, x0, 5, 11
	lsl	x2, x2, 2
	ldr	x3, [x1,#:lo12:.LANCHOR0]
	mov	w1, 1
	lsl	w0, w1, w0
	ldr	w1, [x3,x2]
	orr	w0, w1, w0
	str	w0, [x3,x2]
	ret
	.size	ftl_set_blk_mode.part.7, .-ftl_set_blk_mode.part.7
	.align	2
	.global	FlashMemCmp8
	.type	FlashMemCmp8, %function
FlashMemCmp8:
	mov	x5, x0
	adrp	x0, .LANCHOR0+8
	ldrb	w0, [x0,#:lo12:.LANCHOR0+8]
	cbz	w0, .L7
	ldrb	w4, [x5,1]
	mov	w0, 0
	ldrb	w3, [x1,1]
	cmp	w4, w3
	beq	.L4
.L7:
	mov	x3, 0
.L5:
	cmp	w3, w2
	mov	w4, w3
	bcs	.L11
	ldrb	w0, [x5,x3]
	add	x3, x3, 1
	add	x6, x1, x3
	ldrb	w6, [x6,-1]
	cmp	w6, w0
	beq	.L5
	add	w0, w4, 1
	b	.L4
.L11:
	mov	w0, 0
.L4:
	ret
	.size	FlashMemCmp8, .-FlashMemCmp8
	.align	2
	.global	FlashRsvdBlkChk
	.type	FlashRsvdBlkChk, %function
FlashRsvdBlkChk:
	adrp	x2, .LANCHOR0
	uxtb	w0, w0
	add	x2, x2, :lo12:.LANCHOR0
	ldrb	w3, [x2,9]
	ldr	w2, [x2,12]
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
	adrp	x2, .LANCHOR0+16
	ldrb	w2, [x2,#:lo12:.LANCHOR0+16]
	cbz	w2, .L14
	bl	FlashRsvdBlkChk
	cmp	w0, wzr
	orr	w1, w19, -1073741824
	csel	w19, w1, w19, ne
.L14:
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
	ldrb	w0, [x0,16]
	cbz	w0, .L21
	mov	w0, w21
	bl	FlashRsvdBlkChk
	cmp	w0, wzr
	orr	w1, w20, -1073741824
	csel	w20, w1, w20, ne
.L21:
	add	x2, x19, :lo12:.LANCHOR0
	add	x2, x2, x21, sxtw 4
	ldr	x0, [x2,24]
	str	w20, [x0,336]
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashSetRandomizer, .-FlashSetRandomizer
	.align	2
	.global	FlashReadCmd
	.type	FlashReadCmd, %function
FlashReadCmd:
	adrp	x3, .LANCHOR0
	uxtb	w0, w0
	add	x3, x3, :lo12:.LANCHOR0
	sbfiz	x5, x0, 4, 32
	add	x3, x3, 24
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
	add	x4, x3, 24
	stp	x29, x30, [sp, -16]!
	add	x5, x4, x2
	add	x29, sp, 0
	ldr	x6, [x4,x2]
	lsr	w4, w1, 8
	ldrb	w2, [x3,104]
	lsr	w3, w1, 16
	cmp	w2, 1
	ldrb	w2, [x5,8]
	and	w5, w1, 255
	add	x2, x6, x2, lsl 8
	bne	.L29
	mov	w6, 6
	str	w6, [x2,2056]
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
	str	w5, [x2,2052]
	str	w4, [x2,2052]
	str	w3, [x2,2052]
	b	.L31
.L29:
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
.L31:
	mov	w3, 224
	str	w3, [x2,2056]
	bl	FlashSetRandomizer
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashReadDpDataOutCmd, .-FlashReadDpDataOutCmd
	.align	2
	.global	flash_enter_slc_mode
	.type	flash_enter_slc_mode, %function
flash_enter_slc_mode:
	adrp	x1, .LANCHOR0
	uxtb	w0, w0
	add	x1, x1, :lo12:.LANCHOR0
	ldrb	w2, [x1,120]
	cbz	w2, .L32
	sbfiz	x0, x0, 4, 32
	add	x1, x1, 24
	add	x2, x1, x0
	ldr	x1, [x1,x0]
	ldrb	w0, [x2,8]
	add	x0, x0, 8
	add	x0, x1, x0, lsl 8
	mov	w1, 218
	str	w1, [x0,8]
.L32:
	ret
	.size	flash_enter_slc_mode, .-flash_enter_slc_mode
	.align	2
	.global	flash_exit_slc_mode
	.type	flash_exit_slc_mode, %function
flash_exit_slc_mode:
	adrp	x1, .LANCHOR0
	uxtb	w0, w0
	add	x1, x1, :lo12:.LANCHOR0
	ldrb	w2, [x1,120]
	cbz	w2, .L37
	sbfiz	x0, x0, 4, 32
	add	x1, x1, 24
	add	x2, x1, x0
	ldr	x1, [x1,x0]
	ldrb	w0, [x2,8]
	add	x0, x0, 8
	add	x0, x1, x0, lsl 8
	mov	w1, 223
	str	w1, [x0,8]
.L37:
	ret
	.size	flash_exit_slc_mode, .-flash_exit_slc_mode
	.align	2
	.global	FlashProgFirstCmd
	.type	FlashProgFirstCmd, %function
FlashProgFirstCmd:
	adrp	x3, .LANCHOR0
	uxtb	w0, w0
	add	x3, x3, :lo12:.LANCHOR0
	sbfiz	x5, x0, 4, 32
	add	x3, x3, 24
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
	add	x4, x3, 24
	add	x5, x4, x0
	ldr	x4, [x4,x0]
	ldrb	w0, [x5,8]
	cbz	w2, .L44
	add	x2, x4, x0, lsl 8
	mov	w5, 96
	str	w5, [x2,2056]
	and	w5, w1, 255
	str	w5, [x2,2052]
	lsr	w5, w1, 8
	str	w5, [x2,2052]
	lsr	w5, w1, 16
	str	w5, [x2,2052]
	ldr	w2, [x3,12]
	add	w1, w1, w2
.L44:
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
	add	x5, x2, 24
	lsr	w3, w1, 16
	add	x7, x5, x4
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldrb	w6, [x2,99]
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
	add	x1, x1, 24
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
	add	x2, x1, 24
	add	x3, x2, x0
	ldrb	w1, [x1,98]
	ldr	x2, [x2,x0]
	ldrb	w0, [x3,8]
	add	x0, x0, 8
	add	x0, x2, x0, lsl 8
	str	w1, [x0,8]
	ret
	.size	FlashProgDpFirstCmd, .-FlashProgDpFirstCmd
	.align	2
	.global	JSHash
	.type	JSHash, %function
JSHash:
	mov	x5, x0
	mov	w0, 42982
	mov	x2, 0
	movk	w0, 0x47c6, lsl 16
.L52:
	cmp	w1, w2
	bls	.L54
	lsl	w3, w0, 5
	ldrb	w4, [x5,x2]
	add	w3, w3, w0, lsr 2
	add	x2, x2, 1
	add	w3, w3, w4
	eor	w0, w0, w3
	b	.L52
.L54:
	ret
	.size	JSHash, .-JSHash
	.align	2
	.global	FlashLoadIdbInfo
	.type	FlashLoadIdbInfo, %function
FlashLoadIdbInfo:
	mov	w0, 0
	ret
	.size	FlashLoadIdbInfo, .-FlashLoadIdbInfo
	.align	2
	.global	FlashPrintInfo
	.type	FlashPrintInfo, %function
FlashPrintInfo:
	ret
	.size	FlashPrintInfo, .-FlashPrintInfo
	.align	2
	.global	FlashSuspend
	.type	FlashSuspend, %function
FlashSuspend:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldr	x1, [x0,128]
	ldr	w2, [x1]
	str	w2, [x0,136]
	ldr	w2, [x1,4]
	str	w2, [x0,140]
	ldr	w2, [x1,8]
	str	w2, [x0,144]
	ldr	w2, [x1,12]
	str	w2, [x0,148]
	ldr	w2, [x1,304]
	str	w2, [x0,152]
	ldr	w2, [x1,308]
	str	w2, [x0,156]
	ldr	w2, [x1,336]
	ldr	w1, [x1,344]
	str	w2, [x0,160]
	str	w1, [x0,164]
	mov	w0, 0
	ret
	.size	FlashSuspend, .-FlashSuspend
	.align	2
	.global	LogAddr2PhyAddr
	.type	LogAddr2PhyAddr, %function
LogAddr2PhyAddr:
	adrp	x7, .LANCHOR0
	uxtb	w4, w4
	add	x6, x7, :lo12:.LANCHOR0
	ldrh	w8, [x6,182]
	ldrh	w5, [x6,180]
	ldrh	w9, [x6,12]
	ldrb	w6, [x6,8]
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
	bne	.L60
	add	x1, x7, :lo12:.LANCHOR0
	ldrb	w5, [x1,120]
	cbnz	w5, .L60
	add	x1, x1, 196
	ldrh	w8, [x1,w8,sxtw 1]
.L60:
	add	x7, x7, :lo12:.LANCHOR0
	cmp	w4, 1
	add	x7, x7, 708
	mov	w1, 0
	ldr	w5, [x7,w10,uxtw 2]
	madd	w5, w6, w9, w5
	add	w5, w5, w8
	str	w5, [x2]
	str	w10, [x3]
	bls	.L61
	ldr	w1, [x0,4]
	ldr	w0, [x0,60]
	add	w1, w1, 1024
	cmp	w1, w0
	cset	w1, eq
.L61:
	mov	w0, w1
	ret
	.size	LogAddr2PhyAddr, .-LogAddr2PhyAddr
	.align	2
	.global	FlashScheduleEnSet
	.type	FlashScheduleEnSet, %function
FlashScheduleEnSet:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldr	w2, [x1,740]
	str	w0, [x1,740]
	mov	w0, w2
	ret
	.size	FlashScheduleEnSet, .-FlashScheduleEnSet
	.align	2
	.global	FlashGetPageSize
	.type	FlashGetPageSize, %function
FlashGetPageSize:
	adrp	x0, .LANCHOR0+744
	ldr	x0, [x0,#:lo12:.LANCHOR0+744]
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
	add	x1, x1, 24
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
	adrp	x1, .LANCHOR0+128
	uxtb	w0, w0
	lsl	w2, w0, 8
	orr	w0, w2, w0, lsl 16
	ldr	x1, [x1,#:lo12:.LANCHOR0+128]
	orr	w0, w0, 1
	str	w0, [x1,304]
	ret
	.size	NandcSetDdrPara, .-NandcSetDdrPara
	.align	2
	.global	NandcSetDdrDiv
	.type	NandcSetDdrDiv, %function
NandcSetDdrDiv:
	adrp	x1, .LANCHOR0+128
	uxtb	w0, w0
	ldr	x2, [x1,#:lo12:.LANCHOR0+128]
	mov	w1, 16640
	orr	w0, w0, w1
	str	w0, [x2,344]
	ret
	.size	NandcSetDdrDiv, .-NandcSetDdrDiv
	.align	2
	.global	NandcSetDdrMode
	.type	NandcSetDdrMode, %function
NandcSetDdrMode:
	adrp	x1, .LANCHOR0+128
	cmp	w0, wzr
	ldr	x2, [x1,#:lo12:.LANCHOR0+128]
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
	stp	x29, x30, [sp, -48]!
	adrp	x1, .LANCHOR0
	uxtb	w0, w0
	add	x29, sp, 0
	str	x21, [sp,32]
	add	x21, x1, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	tst	w0, 6
	mov	x20, x1
	ldr	x2, [x21,128]
	ldr	w19, [x2]
	beq	.L73
	orr	w19, w19, 24576
	and	w0, w0, 4
	and	w19, w19, -32769
	cmp	w0, wzr
	orr	w19, w19, 196608
	mov	w0, 0
	orr	w1, w19, 32768
	csel	w19, w1, w19, ne
	bl	rknand_get_clk_rate
	ldr	x0, [x21,128]
	mov	w1, 8322
	str	w1, [x0,344]
	mov	w1, 4099
	movk	w1, 0x10, lsl 16
	str	w1, [x0,304]
	mov	w1, 38
	str	w1, [x0,308]
	mov	w1, 39
	str	w1, [x0,308]
	b	.L75
.L73:
	and	w19, w19, -8193
.L75:
	add	x1, x20, :lo12:.LANCHOR0
	ldr	x0, [x1,128]
	str	w19, [x0]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	NandcSetMode, .-NandcSetMode
	.align	2
	.global	NandcFlashCsDebug
	.type	NandcFlashCsDebug, %function
NandcFlashCsDebug:
	adrp	x1, .LANCHOR0+24
	uxtb	w0, w0
	ldr	x2, [x1,#:lo12:.LANCHOR0+24]
	ldr	w1, [x2]
	bfi	w1, w0, 0, 8
	str	w1, [x2]
	ret
	.size	NandcFlashCsDebug, .-NandcFlashCsDebug
	.align	2
	.global	NandcFlashCs
	.type	NandcFlashCs, %function
NandcFlashCs:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 24
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
	add	x1, x1, 24
	ldr	x1, [x1,x0]
	ldr	w0, [x1]
	and	w0, w0, -256
	and	w0, w0, -131073
	str	w0, [x1]
	ret
	.size	NandcFlashDeCs, .-NandcFlashDeCs
	.align	2
	.global	NandcDelayns
	.type	NandcDelayns, %function
NandcDelayns:
	sub	sp, sp, #16
	lsr	w0, w0, 4
	str	w0, [sp,12]
.L82:
	ldr	w0, [sp,12]
	sub	w1, w0, #1
	str	w1, [sp,12]
	cbnz	w0, .L82
	add	sp, sp, 16
	ret
	.size	NandcDelayns, .-NandcDelayns
	.align	2
	.global	FlashReadStatus
	.type	FlashReadStatus, %function
FlashReadStatus:
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	stp	x29, x30, [sp, -32]!
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 24
	add	x29, sp, 0
	add	x2, x1, x0
	str	x19, [sp,16]
	ldrb	w2, [x2,8]
	ldr	x19, [x1,x0]
	mov	w0, 112
	add	x19, x19, x2, lsl 8
	str	w0, [x19,2056]
	mov	w0, 80
	bl	NandcDelayns
	ldr	w0, [x19,2048]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FlashReadStatus, .-FlashReadStatus
	.align	2
	.global	ToshibaSetRRPara
	.type	ToshibaSetRRPara, %function
ToshibaSetRRPara:
	uxtb	w1, w1
	mov	w3, 5
	add	w2, w1, 1
	stp	x29, x30, [sp, -96]!
	smull	x3, w2, w3
	adrp	x2, .LANCHOR1
	add	x29, sp, 0
	add	x2, x2, :lo12:.LANCHOR1
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	add	x23, x2, x1, sxtw
	add	x24, x2, 256
	add	x21, x2, 304
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	str	x27, [sp,80]
	mov	x22, x0
	add	x24, x24, x3
	add	x21, x21, x3
	mov	x20, 0
	adrp	x26, .LANCHOR0
	mov	w27, 85
	mov	x19, x2
.L86:
	add	x25, x26, :lo12:.LANCHOR0
	ldrb	w0, [x25,753]
	cmp	w0, w20
	bls	.L92
	add	x0, x19, 256
	str	w27, [x22,8]
	ldrsb	w0, [x20,x0]
	str	w0, [x22,4]
	mov	w0, 200
	bl	NandcDelayns
	ldrb	w0, [x25,752]
	cmp	w0, 34
	bne	.L87
	ldrsb	w0, [x24,x20]
	b	.L91
.L87:
	cmp	w0, 35
	bne	.L89
	ldrsb	w0, [x21,x20]
	b	.L91
.L89:
	ldrsb	w0, [x23,400]
.L91:
	str	w0, [x22]
	add	x20, x20, 1
	b	.L86
.L92:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldr	x27, [sp,80]
	ldp	x29, x30, [sp], 96
	ret
	.size	ToshibaSetRRPara, .-ToshibaSetRRPara
	.align	2
	.global	SamsungSetRRPara
	.type	SamsungSetRRPara, %function
SamsungSetRRPara:
	stp	x29, x30, [sp, -64]!
	adrp	x2, .LANCHOR1
	add	x2, x2, :lo12:.LANCHOR1
	add	x29, sp, 0
	add	x1, x2, x1, uxtb 2
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	x21, x0
	add	x22, x1, 412
	mov	x20, 0
	adrp	x23, .LANCHOR0
	mov	w24, 161
	mov	x19, x2
.L94:
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w0, [x0,753]
	cmp	w0, w20
	bls	.L96
	str	w24, [x21,8]
	add	x0, x19, 408
	str	wzr, [x21]
	ldrsb	w0, [x20,x0]
	str	w0, [x21]
	ldrsb	w0, [x22,x20]
	add	x20, x20, 1
	str	w0, [x21]
	mov	w0, 300
	bl	NandcDelayns
	b	.L94
.L96:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	SamsungSetRRPara, .-SamsungSetRRPara
	.align	2
	.global	HynixSetRRPara
	.type	HynixSetRRPara, %function
HynixSetRRPara:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR0
	str	x27, [sp,80]
	stp	x19, x20, [sp,16]
	add	x19, x22, :lo12:.LANCHOR0
	uxtb	w27, w1
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	uxtb	x23, w0
	mov	x25, x2
	ldr	x0, [x19,744]
	mov	x26, x23
	uxtb	w24, w3
	add	x19, x19, 760
	ldrb	w0, [x0,19]
	cmp	w0, 6
	bne	.L98
	add	x19, x19, x23, lsl 6
	lsl	w0, w24, 2
	add	x19, x19, 20
	add	x19, x19, x0, sxtw
	b	.L99
.L98:
	lsl	x2, x23, 3
	add	x2, x2, x24, sxtw
	add	x19, x19, x2, lsl 3
	add	x19, x19, 20
.L99:
	add	x2, x22, :lo12:.LANCHOR0
	lsl	x0, x23, 4
	add	x2, x2, 24
	uxtb	x27, w27
	add	x1, x2, x0
	ldr	x20, [x2,x0]
	mov	w0, w26
	ldrb	w21, [x1,8]
	bl	NandcFlashCs
	ubfiz	x21, x21, 8, 8
	mov	w0, 54
	add	x20, x20, x21
	mov	x21, 0
	str	w0, [x20,2056]
.L100:
	cmp	x21, x27
	beq	.L102
	ldrb	w0, [x25,x21]
	str	w0, [x20,2052]
	mov	w0, 200
	bl	NandcDelayns
	ldrsb	w0, [x19,x21]
	add	x21, x21, 1
	str	w0, [x20,2048]
	b	.L100
.L102:
	add	x22, x22, :lo12:.LANCHOR0
	mov	w0, 22
	add	x23, x22, x23
	str	w0, [x20,2056]
	mov	w0, w26
	bl	NandcFlashDeCs
	strb	w24, [x23,1616]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldr	x27, [sp,80]
	ldp	x29, x30, [sp], 96
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
	ldr	x0, [x19,744]
	ldrb	w0, [x0,19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 5
	bhi	.L103
	add	x21, x19, 1620
	add	x22, x19, 764
.L108:
	lsl	x1, x20, 3
	uxtb	w0, w20
	ldrb	w1, [x1,x21]
	cmp	w1, 173
	bne	.L105
	ldrb	w1, [x19,761]
	mov	x2, x22
	mov	w3, 0
	bl	HynixSetRRPara
.L105:
	add	x20, x20, 1
	cmp	x20, 4
	bne	.L108
.L103:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashSetReadRetryDefault, .-FlashSetReadRetryDefault
	.align	2
	.global	FlashReadStatusEN
	.type	FlashReadStatusEN, %function
FlashReadStatusEN:
	stp	x29, x30, [sp, -32]!
	adrp	x3, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x4, x3, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x5, x4, 24
	stp	x19, x20, [sp,16]
	add	x6, x5, x0
	uxtb	w2, w2
	ldr	x20, [x5,x0]
	ldr	x0, [x4,744]
	ldrb	w19, [x6,8]
	ldrb	w0, [x0,8]
	cmp	w0, 2
	bne	.L111
	add	x4, x4, 88
	cbnz	w2, .L112
	ldrb	w2, [x4,13]
	b	.L120
.L112:
	ldrb	w2, [x4,14]
.L120:
	add	x0, x19, 8
	add	x3, x3, :lo12:.LANCHOR0
	add	x0, x20, x0, lsl 8
	str	w2, [x0,8]
	mov	w2, 0
	ldrb	w4, [x3,103]
	cbz	w4, .L115
	add	x3, x19, 8
	lsl	x3, x3, 8
.L119:
	cmp	w2, w4
	bcs	.L115
	add	x5, x20, x3
	lsl	w0, w2, 3
	lsr	w0, w1, w0
	add	w2, w2, 1
	and	w0, w0, 255
	str	w0, [x5,4]
	b	.L119
.L111:
	add	x0, x19, 8
	mov	w1, 112
	add	x0, x20, x0, lsl 8
	str	w1, [x0,8]
.L115:
	add	x19, x19, 8
	mov	w0, 80
	lsl	x19, x19, 8
	bl	NandcDelayns
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
.L125:
	mov	w0, w19
	mov	w1, w20
	mov	w2, w21
	bl	FlashReadStatusEN
	cmp	w0, 255
	mov	w3, w0
	beq	.L125
	tbz	x3, 6, .L125
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashWaitReadyEN, .-FlashWaitReadyEN
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
	add	x0, x20, 1652
	add	x19, x0, x1
	ldrb	w21, [x0,x1]
	ldr	x0, [x19,8]
	cbz	x0, .L130
	mov	w0, w21
	add	x20, x20, 708
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
	cbz	x1, .L130
	str	w0, [x1]
	str	xzr, [x19,16]
.L130:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashWaitCmdDone, .-FlashWaitCmdDone
	.align	2
	.global	NandcWaitFlashReady
	.type	NandcWaitFlashReady, %function
NandcWaitFlashReady:
	stp	x29, x30, [sp, -48]!
	adrp	x1, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x1, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x1, x1, 24
	stp	x19, x20, [sp,16]
	mov	w19, 34464
	ldr	x20, [x1,x0]
	movk	w19, 0x1, lsl 16
.L137:
	mov	w0, 100
	bl	NandcDelayns
	ldr	w0, [x20]
	str	w0, [x29,40]
	ldr	w0, [x29,40]
	tbnz	x0, 9, .L138
	subs	w19, w19, #1
	bne	.L137
	mov	w0, -1
	b	.L136
.L138:
	mov	w0, 0
.L136:
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
	add	x1, x1, 24
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
	adrp	x1, .LANCHOR0
	mov	x6, 0
	add	x1, x1, :lo12:.LANCHOR0
	mov	w8, 239
	stp	x29, x30, [sp, -16]!
	add	x7, x1, 24
	add	x4, x1, 1620
	add	x29, sp, 0
	ldrb	w2, [x1,1844]
	mov	w9, 128
	mov	w10, 1
	mov	w12, 35
	and	w11, w2, 1
	and	w2, w2, 4
	mov	w13, 32
	mov	w14, 5
	uxtb	w5, w2
.L153:
	ldrb	w3, [x6,x4]
	ldr	x1, [x7]
	cmp	w3, 152
	ldrb	w2, [x7,8]
	beq	.L143
	cmp	w3, 69
	beq	.L143
	cmp	w3, 173
	beq	.L143
	cmp	w3, 44
	bne	.L144
.L143:
	cmp	w0, 1
	bne	.L146
	cbz	w11, .L144
	ubfiz	x2, x2, 8, 8
	cmp	w3, 173
	add	x1, x1, x2
	str	w8, [x1,2056]
	bne	.L147
	str	w0, [x1,2052]
	b	.L164
.L147:
	cmp	w3, 44
	bne	.L149
	str	w0, [x1,2052]
	str	w14, [x1,2048]
	b	.L151
.L149:
	str	w9, [x1,2052]
	str	w0, [x1,2048]
	b	.L151
.L146:
	cbz	w5, .L144
	ubfiz	x2, x2, 8, 8
	cmp	w3, 173
	add	x1, x1, x2
	str	w8, [x1,2056]
	bne	.L150
	str	w10, [x1,2052]
	str	w13, [x1,2048]
	b	.L151
.L150:
	cmp	w3, 44
	bne	.L152
	str	w10, [x1,2052]
	str	w12, [x1,2048]
	b	.L151
.L152:
	str	w9, [x1,2052]
.L164:
	str	wzr, [x1,2048]
.L151:
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
.L144:
	add	x6, x6, 8
	add	x7, x7, 16
	cmp	x6, 32
	bne	.L153
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
	add	x5, x5, 24
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
	add	x3, x3, 24
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
	mov	w0, 80
	bl	NandcDelayns
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
	mov	w0, 200
	bl	NandcDelayns
	mov	w1, 5
	adrp	x2, .LANCHOR1
	add	x2, x2, :lo12:.LANCHOR1
	adrp	x5, .LANCHOR0
	smull	x1, w19, w1
	add	x0, x2, 256
	add	x2, x2, 304
	add	x0, x0, x1
	add	x1, x2, x1
	mov	x2, 0
.L168:
	add	x3, x5, :lo12:.LANCHOR0
	ldrb	w4, [x3,753]
	cmp	w4, w2
	bls	.L173
	ldrb	w3, [x3,752]
	cmp	w3, 67
	bne	.L169
	ldrsb	w3, [x0,x2]
	b	.L172
.L169:
	ldrsb	w3, [x1,x2]
.L172:
	str	w3, [x20]
	add	x2, x2, 1
	b	.L168
.L173:
	mov	w0, 0
	bl	NandcWaitFlashReady
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	SandiskSetRRPara, .-SandiskSetRRPara
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
	adrp	x23, .LC0
	stp	x19, x20, [sp,16]
	mov	w24, w1
	mov	x20, x0
	mov	w22, 0
	add	x25, x21, 1652
	add	x23, x23, :lo12:.LC0
.L175:
	cmp	w22, w24
	beq	.L184
	add	x2, x29, 88
	sub	w4, w24, w22
	mov	x0, x20
	mov	w1, 0
	add	x3, x29, 92
	bl	LogAddr2PhyAddr
	ldr	w2, [x29,92]
	ldrb	w0, [x21,1845]
	cmp	w2, w0
	bcc	.L176
	mov	w0, -1
	str	w0, [x20]
	b	.L177
.L176:
	uxtw	x2, w2
	add	x0, x21, x2
	ldrb	w19, [x0,1848]
	mov	x0, 24
	mul	x2, x2, x0
	mov	w0, w19
	strb	w19, [x25,x2]
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
	ldr	w2, [x21,12]
	add	w1, w2, w1
	mov	w2, 0
	bl	FlashEraseCmd
	mov	w0, w19
	bl	NandcWaitFlashReady
	ldr	w1, [x29,88]
	mov	w0, w19
	bl	FlashReadStatus
	tbz	x0, 0, .L178
	mov	w0, -1
	str	w0, [x20]
.L178:
	ldr	w0, [x20]
	cmn	w0, #1
	bne	.L179
	ldr	w1, [x29,88]
	mov	x0, x23
	bl	printk
.L179:
	mov	w0, w19
	bl	NandcFlashDeCs
.L177:
	add	w22, w22, 1
	add	x20, x20, 56
	b	.L175
.L184:
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
	adrp	x19, .LANCHOR0
	stp	x25, x26, [sp,64]
	mov	w26, w0
	add	x0, x19, :lo12:.LANCHOR0
	add	x1, x0, 760
	cmp	w26, 2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	strb	w26, [x0,760]
	mov	w0, -84
	strb	w4, [x1,5]
	strb	w0, [x1,4]
	mov	w0, -81
	strb	w3, [x1,6]
	strb	w0, [x1,7]
	bne	.L186
	mov	w0, -89
	strb	w0, [x1,4]
	adrp	x0, .LANCHOR1+521
	mov	w1, -9
	strb	w1, [x0,#:lo12:.LANCHOR1+521]
	b	.L223
.L186:
	cmp	w26, 3
	bne	.L188
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
	b	.L239
.L188:
	cmp	w26, 4
	bne	.L189
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
.L239:
	mov	w21, 8
	strb	w0, [x1,11]
	mov	w20, w21
	b	.L187
.L189:
	cmp	w26, 5
	bne	.L190
	mov	w0, 56
	strb	w0, [x1,4]
	mov	w0, 57
	strb	w0, [x1,5]
	mov	w0, 58
	mov	w21, 8
	strb	w0, [x1,6]
	mov	w0, 59
	strb	w0, [x1,7]
	b	.L238
.L190:
	cmp	w26, 6
	bne	.L223
	mov	w0, 14
	strb	w0, [x1,4]
	mov	w0, 15
	strb	w0, [x1,5]
	mov	w0, 16
	mov	w21, 12
	strb	w0, [x1,6]
	mov	w0, 17
	strb	w0, [x1,7]
	b	.L238
.L223:
	mov	w21, 7
.L238:
	mov	w20, 4
.L187:
	sub	w0, w26, #1
	cmp	w0, 1
	bhi	.L235
	adrp	x24, .LANCHOR1
	add	x27, x19, :lo12:.LANCHOR0
	add	x24, x24, :lo12:.LANCHOR1
	mov	w23, 0
	add	x28, x27, 760
	add	x4, x27, 24
	add	x24, x24, 504
.L191:
	ldrb	w0, [x27,1845]
	cmp	w0, w23
	bls	.L198
	add	x0, x27, x23, sxtw
	ldrb	w0, [x0,1848]
	mov	x25, 0
	add	x22, x28, x0, lsl 6
	lsl	x0, x0, 4
	add	x1, x4, x0
	add	x22, x22, 20
	ldr	x26, [x4,x0]
	ldrb	w0, [x1,8]
	mov	w1, 55
	add	x26, x26, x0, lsl 8
.L193:
	add	x0, x28, x25
	str	w1, [x26,2056]
	str	x4, [x29,128]
	str	x1, [x29,136]
	ldrb	w0, [x0,4]
	str	w0, [x26,2052]
	mov	w0, 80
	bl	NandcDelayns
	ldr	w0, [x26,2048]
	strb	w0, [x22,x25]
	add	x25, x25, 1
	ldr	x1, [x29,136]
	cmp	w20, w25, uxtb
	ldr	x4, [x29,128]
	bhi	.L193
	mov	x0, 0
.L194:
	add	w2, w0, 8
	mov	x1, 0
.L195:
	add	x3, x1, x0
	add	x1, x1, 4
	add	x3, x24, x3
	cmp	x1, 24
	ldrb	w5, [x3,4]
	ldrb	w3, [x22,x0]
	add	w3, w5, w3
	strb	w3, [x22,w2,sxtw]
	add	w2, w2, 8
	bne	.L195
	add	x0, x0, 1
	cmp	x0, 4
	bne	.L194
	add	w23, w23, 1
	strb	wzr, [x22,16]
	strb	wzr, [x22,24]
	strb	wzr, [x22,32]
	uxtb	w23, w23
	strb	wzr, [x22,40]
	strb	wzr, [x22,48]
	strb	wzr, [x22,41]
	strb	wzr, [x22,49]
	b	.L191
.L235:
	sub	w0, w26, #3
	cmp	w0, 3
	bhi	.L198
	mul	w0, w20, w21
	cmp	w26, 6
	mov	w1, 4
	asr	w24, w0, 2
	lsl	w0, w0, 4
	str	w0, [x29,124]
	lsl	w0, w24, 1
	str	w0, [x29,116]
	mov	w0, 8
	csel	w0, w1, w0, eq
	add	x23, x19, :lo12:.LANCHOR0
	str	w0, [x29,120]
	add	x0, x23, 760
	str	x0, [x29,136]
	sub	w0, w20, #1
	mov	w22, 0
	sub	w27, w26, #5
	uxtb	x0, w0
	add	x0, x0, 1
	str	x0, [x29,104]
.L199:
	ldrb	w0, [x23,1845]
	cmp	w0, w22
	bhi	.L222
.L198:
	add	x19, x19, :lo12:.LANCHOR0
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	strb	w21, [x19,762]
	strb	w20, [x19,761]
	ldp	x21, x22, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
.L222:
	add	x0, x23, x22, sxtw
	ldrb	w0, [x0,1848]
	ldr	x1, [x29,136]
	add	x3, x23, 24
	mov	x25, x0
	add	x28, x1, x0, lsl 6
	lsl	x0, x0, 4
	add	x1, x28, 20
	str	x1, [x29,128]
	add	x1, x3, x0
	ldr	x0, [x3,x0]
	mov	w3, 255
	str	x3, [x29,96]
	ldrb	w1, [x1,8]
	add	x28, x0, x1, lsl 8
	mov	w0, w25
	str	w3, [x28,2056]
	bl	NandcWaitFlashReady
	mov	w0, 54
	cmp	w26, 4
	str	w0, [x28,2056]
	ldr	x3, [x29,96]
	bne	.L200
	mov	w0, 64
	str	w3, [x28,2052]
	str	w0, [x28,2048]
	mov	w0, 204
	b	.L240
.L200:
	cmp	w27, 1
	bhi	.L202
	ldr	x0, [x29,136]
	ldrb	w0, [x0,4]
	str	w0, [x28,2052]
	mov	w0, 82
	b	.L241
.L202:
	mov	w0, 174
	str	w0, [x28,2052]
	str	wzr, [x28,2048]
	mov	w0, 176
.L240:
	str	w0, [x28,2052]
	mov	w0, 77
.L241:
	str	w0, [x28,2048]
	mov	w0, 22
	str	w0, [x28,2056]
	mov	w0, 23
	str	w0, [x28,2056]
	mov	w0, 4
	str	w0, [x28,2056]
	mov	w0, 25
	str	w0, [x28,2056]
	cmp	w26, 6
	str	wzr, [x28,2056]
	str	wzr, [x28,2052]
	str	wzr, [x28,2052]
	bne	.L203
	mov	w0, 31
	str	w0, [x28,2052]
	b	.L204
.L203:
	str	wzr, [x28,2052]
.L204:
	mov	w3, 2
	str	w3, [x28,2052]
	str	wzr, [x28,2052]
	mov	w0, 48
	str	w0, [x28,2056]
	mov	w0, w25
	str	x3, [x29,96]
	bl	NandcWaitFlashReady
	ldr	x3, [x29,96]
	mov	w0, 16
	cmp	w27, 1
	csel	w3, w3, w0, hi
	mov	w0, 0
.L206:
	add	w0, w0, 1
	ldr	w4, [x28,2048]
	uxtb	w0, w0
	cmp	w0, w3
	bne	.L206
	ldr	x6, [x23,1856]
	mov	x0, 0
.L207:
	ldr	w1, [x29,124]
	cmp	w1, w0
	ble	.L242
	ldr	w3, [x28,2048]
	strb	w3, [x6,x0]
	add	x0, x0, 1
	b	.L207
.L242:
	ldr	x8, [x23,1856]
	mov	w5, w24
	mov	w4, 8
.L210:
	mov	w0, 0
.L209:
	add	w3, w0, w5
	add	w0, w0, 1
	sbfiz	x3, x3, 2, 32
	cmp	w0, w24
	ldr	w7, [x8,x3]
	mvn	w7, w7
	str	w7, [x8,x3]
	bne	.L209
	ldr	w0, [x29,116]
	subs	w4, w4, #1
	add	w5, w5, w0
	bne	.L210
	mov	x4, 0
	mov	w12, 1
.L211:
	mov	w3, 0
	ldr	x11, [x23,1856]
	mov	w7, w3
.L215:
	lsl	w5, w12, w7
	mov	w9, w4
	mov	w0, 16
	mov	w8, 0
.L213:
	ldr	w10, [x11,w9,sxtw 2]
	add	w9, w9, w24
	and	w10, w5, w10
	cmp	w10, w5
	csinc	w8, w8, w8, ne
	subs	w0, w0, #1
	bne	.L213
	cmp	w8, 9
	orr	w5, w3, w5
	add	w7, w7, 1
	csel	w3, w5, w3, cs
	cmp	w7, 32
	bne	.L215
	str	w3, [x11,x4,lsl 2]
	add	x4, x4, 1
	cmp	w24, w4
	bgt	.L211
	mov	w4, w0
.L218:
	mov	x3, 0
.L217:
	add	w5, w0, w3
	ldr	x1, [x29,128]
	ldrb	w7, [x6,x3]
	add	x3, x3, 1
	cmp	w20, w3, uxtb
	strb	w7, [x1,w5,sxtw]
	bhi	.L217
	ldr	x1, [x29,104]
	add	w4, w4, 1
	cmp	w4, w21
	add	x6, x6, x1
	ldr	w1, [x29,120]
	add	w0, w0, w1
	blt	.L218
	mov	w0, 255
	str	w0, [x28,2056]
	mov	w0, w25
	bl	NandcWaitFlashReady
	cmp	w27, 1
	bhi	.L220
	mov	w0, 54
	str	w0, [x28,2056]
	ldr	x0, [x29,136]
	mov	w1, -1
	ldrb	w0, [x0,4]
	str	w0, [x28,2052]
	mov	w0, 22
	str	wzr, [x28,2048]
	str	w0, [x28,2056]
	mov	w0, w22
	bl	FlashReadCmd
	b	.L221
.L220:
	mov	w0, 56
	str	w0, [x28,2056]
.L221:
	mov	w0, w25
	add	w22, w22, 1
	bl	NandcWaitFlashReady
	uxtb	w22, w22
	b	.L199
	.size	HynixGetReadRetryDefault, .-HynixGetReadRetryDefault
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
	add	x4, x3, 24
	sbfiz	x0, x21, 4, 32
	add	x3, x3, 88
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
	bne	.L244
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
	b	.L245
.L244:
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
.L245:
	mov	w0, w21
	bl	FlashSetRandomizer
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 80
	ret
	.size	FlashReadDpCmd, .-FlashReadDpCmd
	.align	2
	.global	FlashDeInit
	.type	FlashDeInit, %function
FlashDeInit:
	stp	x29, x30, [sp, -32]!
	mov	w0, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	bl	NandcWaitFlashReady
	bl	FlashSetReadRetryDefault
	adrp	x0, .LANCHOR0
	add	x20, x0, :lo12:.LANCHOR0
	mov	x19, x0
	ldrb	w1, [x20,1864]
	cbz	w1, .L247
	ldrb	w0, [x20,1844]
	tbz	x0, 0, .L247
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	strb	wzr, [x20,1864]
.L247:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x0, [x0,24]
	str	wzr, [x0,336]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FlashDeInit, .-FlashDeInit
	.align	2
	.global	NandcRandmzSel
	.type	NandcRandmzSel, %function
NandcRandmzSel:
	adrp	x2, .LANCHOR0
	ubfiz	x0, x0, 4, 8
	add	x2, x2, :lo12:.LANCHOR0
	add	x2, x2, 24
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
	ble	.L256
	ldr	x0, [x1,128]
	mov	w1, 8354
	b	.L263
.L256:
	cmp	w0, 220
	ble	.L258
	ldr	x0, [x1,128]
	b	.L264
.L258:
	cmp	w0, 185
	ble	.L259
	ldr	x0, [x1,128]
	mov	w1, 4226
	b	.L263
.L259:
	cmp	w0, 160
	ldr	x0, [x1,128]
	ble	.L260
	mov	w1, 4194
	b	.L263
.L260:
	cmp	w19, 35
	mov	w1, 4193
	bls	.L263
	cmp	w19, 99
	mov	w1, 4225
	bls	.L263
.L264:
	mov	w1, 8322
.L263:
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
	bls	.L266
	mov	w1, 8322
	cmp	w0, w1
	bne	.L267
.L266:
	adrp	x1, .LANCHOR0+128
	ldr	x1, [x1,#:lo12:.LANCHOR0+128]
	str	w0, [x1,4]
.L267:
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
	mov	w2, 1
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	str	w2, [x19,48]
	mov	w2, 2
	str	x0, [x19,24]
	str	x0, [x19,40]
	str	w2, [x19,64]
	mov	w2, 3
	str	x0, [x19,56]
	str	w2, [x19,80]
	str	x0, [x19,72]
	str	x0, [x19,128]
	str	wzr, [x19,32]
	ldr	w1, [x0]
	ubfx	x2, x1, 13, 1
	str	w2, [x19,1868]
	ldr	w2, [x0,352]
	and	w1, w1, 245760
	orr	w1, w1, 256
	ubfx	x2, x2, 16, 4
	str	w2, [x19,1872]
	ldr	w2, [x0,352]
	str	w2, [x19,1876]
	str	w1, [x0]
	ldr	x0, [x19,128]
	str	wzr, [x0,336]
	mov	w0, 40
	bl	NandcTimeCfg
	ldr	x0, [x19,128]
	mov	w1, 8322
	str	w1, [x0,344]
	mov	w1, 6145
	movk	w1, 0x18, lsl 16
	str	w1, [x0,304]
	mov	w0, 36864
	bl	ftl_malloc
	str	wzr, [x19,1928]
	str	x0, [x19,1880]
	str	x0, [x19,1888]
	add	x0, x0, 32768
	str	wzr, [x19,1936]
	str	x0, [x19,1896]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	NandcInit, .-NandcInit
	.align	2
	.global	NandcGetTimeCfg
	.type	NandcGetTimeCfg, %function
NandcGetTimeCfg:
	adrp	x4, .LANCHOR0
	add	x4, x4, :lo12:.LANCHOR0
	ldr	x5, [x4,128]
	ldr	w5, [x5,4]
	str	w5, [x0]
	ldr	x0, [x4,128]
	ldr	w0, [x0]
	str	w0, [x1]
	ldr	x0, [x4,128]
	ldr	w0, [x0,304]
	str	w0, [x2]
	ldr	x1, [x4,128]
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
	adrp	x1, .LANCHOR0
	uxtb	w0, w0
	add	x1, x1, :lo12:.LANCHOR0
	mov	w3, 1
	cmp	w0, 16
	ldr	x2, [x1,128]
	str	w0, [x1,1940]
	mov	w1, 4096
	str	w3, [x2,8]
	bne	.L271
.L274:
	and	w1, w1, -17
	b	.L272
.L271:
	cmp	w0, 24
	bne	.L273
	orr	w1, w1, 16
	b	.L272
.L273:
	cmp	w0, 40
	orr	w1, w1, 262144
	orr	w1, w1, 16
	beq	.L274
.L272:
	orr	w1, w1, 1
	str	w1, [x2,12]
	ret
	.size	NandcBchSel, .-NandcBchSel
	.align	2
	.global	FlashBchSel
	.type	FlashBchSel, %function
FlashBchSel:
	stp	x29, x30, [sp, -16]!
	adrp	x1, .LANCHOR0+1944
	uxtb	w0, w0
	add	x29, sp, 0
	strb	w0, [x1,#:lo12:.LANCHOR0+1944]
	bl	NandcBchSel
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashBchSel, .-FlashBchSel
	.align	2
	.global	FlashResume
	.type	FlashResume, %function
FlashResume:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	str	x21, [sp,32]
	add	x0, x20, :lo12:.LANCHOR0
	mov	x21, 0
	add	x19, x0, 1620
	ldr	x1, [x0,128]
	ldr	w2, [x0,136]
	str	w2, [x1]
	ldr	w2, [x0,140]
	ldr	x1, [x0,128]
	str	w2, [x1,4]
	ldr	w2, [x0,144]
	ldr	x1, [x0,128]
	str	w2, [x1,8]
	ldr	w2, [x0,148]
	str	w2, [x1,12]
	ldr	w2, [x0,152]
	str	w2, [x1,304]
	ldr	w2, [x0,156]
	str	w2, [x1,308]
	ldr	w2, [x0,160]
	str	w2, [x1,336]
	ldr	w2, [x0,164]
	str	w2, [x1,344]
.L281:
	lsl	x0, x21, 3
	ldrb	w0, [x0,x19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L280
	mov	w0, w21
	bl	FlashReset
.L280:
	add	x21, x21, 1
	cmp	x21, 4
	bne	.L281
	add	x19, x20, :lo12:.LANCHOR0
	ldrb	w0, [x19,1864]
	cbz	w0, .L282
	mov	w0, 1
	bl	NandcSetMode
	ldrb	w0, [x19,1844]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x19,1844]
	bl	NandcSetMode
	ldr	w0, [x19,152]
	lsr	w0, w0, 8
	bl	NandcSetDdrPara
.L282:
	add	x20, x20, :lo12:.LANCHOR0
	ldr	x0, [x20,744]
	ldrb	w0, [x0,20]
	bl	FlashBchSel
	ldr	x21, [sp,32]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FlashResume, .-FlashResume
	.align	2
	.global	NandCIrqEnable
	.type	NandCIrqEnable, %function
NandCIrqEnable:
	ldr	w3, [x0,368]
	mov	x2, 1
	lsl	x1, x2, x1
	orr	w3, w3, w1
	str	w3, [x0,368]
	ldr	w2, [x0,364]
	orr	w1, w2, w1
	str	w1, [x0,364]
	ret
	.size	NandCIrqEnable, .-NandCIrqEnable
	.align	2
	.global	NandCIrqDisable
	.type	NandCIrqDisable, %function
NandCIrqDisable:
	ldr	w3, [x0,368]
	mov	x2, 1
	lsl	x1, x2, x1
	orr	w3, w3, w1
	str	w3, [x0,368]
	ldr	w2, [x0,364]
	bic	w1, w2, w1
	str	w1, [x0,364]
	ret
	.size	NandCIrqDisable, .-NandCIrqDisable
	.align	2
	.global	rk_nandc_get_irq_status
	.type	rk_nandc_get_irq_status, %function
rk_nandc_get_irq_status:
	ldr	w0, [x0,372]
	ret
	.size	rk_nandc_get_irq_status, .-rk_nandc_get_irq_status
	.align	2
	.global	rk_nandc_flash_ready
	.type	rk_nandc_flash_ready, %function
rk_nandc_flash_ready:
	stp	x29, x30, [sp, -16]!
	mov	w1, 1
	add	x29, sp, 0
	bl	NandCIrqDisable
	ldp	x29, x30, [sp], 16
	ret
	.size	rk_nandc_flash_ready, .-rk_nandc_flash_ready
	.align	2
	.global	NandcIqrWaitFlashReady
	.type	NandcIqrWaitFlashReady, %function
NandcIqrWaitFlashReady:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	x19, x0
	bl	rk_nandc_rb_irq_flag_init
	mov	x0, x19
	mov	w1, 1
	bl	NandCIrqEnable
	ldr	w0, [x19]
	tbnz	x0, 9, .L292
	mov	x0, x19
	bl	wait_for_nand_flash_ready
	b	.L291
.L292:
	mov	x0, x19
	mov	w1, 1
	bl	NandCIrqDisable
.L291:
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
	stp	x27, x28, [sp,80]
	mov	w27, w1
	adrp	x1, .LANCHOR0
	stp	x19, x20, [sp,16]
	add	x19, x1, :lo12:.LANCHOR0
	str	x1, [x29,104]
	stp	x23, x24, [sp,48]
	mov	w23, w2
	mov	x24, x0
	ldrb	w2, [x19,8]
	mov	w20, 0
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	cbz	w2, .L295
	mov	w1, w23
	bl	FlashEraseSLc2KBlocks
	b	.L296
.L295:
	mov	w28, 56
	add	x25, x19, 1652
	mov	x26, 24
.L321:
	cmp	w20, w23
	bcs	.L322
	umull	x5, w20, w28
	mov	w1, 0
	sub	w4, w23, w20
	add	x21, x24, x5
	add	x2, x29, 120
	mov	x0, x21
	add	x3, x29, 124
	str	x5, [x29,96]
	bl	LogAddr2PhyAddr
	mov	w22, w0
	ldrb	w1, [x19,1845]
	ldr	w0, [x29,124]
	ldr	x5, [x29,96]
	cmp	w0, w1
	bcc	.L297
	mov	w0, -1
	str	w0, [x24,x5]
	b	.L298
.L297:
	ldrb	w1, [x19,1945]
	cmp	w1, wzr
	uxtw	x1, w0
	csel	w22, w22, wzr, ne
	madd	x1, x1, x26, x25
	ldr	x1, [x1,8]
	cbz	x1, .L300
	bl	FlashWaitCmdDone
.L300:
	ldr	w0, [x29,124]
	ldr	w1, [x29,120]
	madd	x2, x0, x26, x25
	str	x21, [x2,8]
	str	xzr, [x2,16]
	str	w1, [x2,4]
	cbz	w22, .L301
	add	w1, w20, 1
	umull	x1, w1, w28
	add	x1, x24, x1
	str	x1, [x2,16]
.L301:
	add	x1, x19, x0
	mul	x0, x0, x26
	ldrb	w21, [x1,1848]
	strb	w21, [x25,x0]
	mov	w0, w21
	bl	NandcFlashCs
	cmp	w27, 1
	bne	.L302
	ldrb	w0, [x19,120]
	cbz	w0, .L302
	mov	w0, w21
	bl	flash_enter_slc_mode
	b	.L303
.L302:
	mov	w0, w21
	bl	flash_exit_slc_mode
.L303:
	ldr	w1, [x29,124]
	add	x0, x19, 708
	add	w20, w20, w22
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
.L298:
	add	w20, w20, 1
	b	.L321
.L322:
	ldr	x0, [x29,104]
	mov	x20, 0
	mov	x22, 24
	add	x19, x0, :lo12:.LANCHOR0
	add	x21, x19, 1652
	ldr	x0, [x19,128]
	bl	NandcIqrWaitFlashReady
.L305:
	ldrb	w0, [x19,1845]
	cmp	w0, w20
	bls	.L323
	mov	w0, w20
	bl	FlashWaitCmdDone
	cmp	w27, 1
	bne	.L306
	ldrb	w0, [x19,120]
	cbz	w0, .L306
	mul	x0, x20, x22
	ldrb	w0, [x0,x21]
	bl	flash_exit_slc_mode
.L306:
	add	x20, x20, 1
	b	.L305
.L323:
	mov	w0, 0
.L296:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	FlashEraseBlocks, .-FlashEraseBlocks
	.align	2
	.global	rk_nandc_flash_xfer_completed
	.type	rk_nandc_flash_xfer_completed, %function
rk_nandc_flash_xfer_completed:
	stp	x29, x30, [sp, -16]!
	mov	w1, 0
	add	x29, sp, 0
	bl	NandCIrqDisable
	ldp	x29, x30, [sp], 16
	ret
	.size	rk_nandc_flash_xfer_completed, .-rk_nandc_flash_xfer_completed
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
.L327:
	ldr	w1, [x0,8]
	str	w1, [sp,8]
	ldr	w1, [sp,8]
	tbz	x1, 20, .L327
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
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	str	x25, [sp,64]
	stp	x23, x24, [sp,48]
	uxtb	w24, w1
	add	x1, x19, :lo12:.LANCHOR0
	uxtb	w25, w2
	add	x2, x1, 24
	stp	x21, x22, [sp,32]
	mov	x22, x4
	add	x4, x2, x0
	ldr	x21, [x2,x0]
	mov	w2, 16
	ubfiz	w20, w24, 1, 1
	ldrb	w0, [x4,8]
	orr	w20, w20, 8
	ldr	w23, [x21,12]
	bfi	w23, w2, 8, 8
	and	w23, w23, -9
	bfi	w23, w0, 5, 3
	mov	w0, 1
	bfi	w20, w0, 5, 2
	ldr	w0, [x1,1872]
	orr	w20, w20, 536870912
	cmp	w0, 3
	orr	w20, w20, 1024
	bfi	w20, w3, 4, 1
	bls	.L331
	ldr	w0, [x21,16]
	cmp	x5, xzr
	str	w0, [x29,88]
	cset	w8, ne
	ldr	w0, [x29,88]
	and	w0, w0, -5
	str	w0, [x29,88]
	cbnz	w8, .L345
	cbz	x22, .L332
.L345:
	cbnz	w24, .L334
.L342:
	mov	x0, x21
	bl	rk_nandc_xfer_irq_flag_init
	mov	x0, x21
	mov	w1, 0
	bl	NandCIrqEnable
	add	w2, w25, 1
	mov	x0, x22
	asr	w2, w2, 1
	bfi	w20, w2, 22, 6
	cbnz	x22, .L336
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x0, [x0,1888]
	b	.L336
.L334:
	add	x1, x19, :lo12:.LANCHOR0
	mov	w6, 128
	mov	w3, 0
	lsr	w2, w25, 1
	mov	w4, w3
	mov	w9, -1
	ldr	w0, [x1,1940]
	cmp	w0, 25
	mov	w0, 64
	csel	w6, w0, w6, cc
.L338:
	cmp	w4, w2
	bcs	.L342
	lsr	w0, w3, 2
	ubfiz	x0, x0, 2, 30
	cbz	w8, .L339
	ldrh	w7, [x5,2]
	ldr	x10, [x1,1896]
	ldrh	w11, [x5],4
	orr	w7, w11, w7, lsl 16
	str	w7, [x10,x0]
	b	.L340
.L339:
	ldr	x7, [x1,1896]
	str	w9, [x7,x0]
.L340:
	add	w4, w4, 1
	add	w3, w3, w6
	b	.L338
.L336:
	add	x19, x19, :lo12:.LANCHOR0
	ubfx	x25, x20, 22, 5
	mov	w2, w24
	and	x22, x22, 3
	ldr	x1, [x19,1896]
	str	x1, [x19,1912]
	lsl	w1, w25, 10
	str	x0, [x19,1904]
	bl	rknand_dma_map_single
	str	w0, [x19,1920]
	ldr	x0, [x19,1912]
	lsl	w1, w25, 7
	mov	w2, w24
	bl	rknand_dma_map_single
	str	w0, [x19,1924]
	mov	w0, 1
	str	w0, [x19,1928]
	ldr	w0, [x19,1920]
	mov	w1, 16
	str	w0, [x21,20]
	ldr	w0, [x19,1924]
	str	w0, [x21,24]
	str	wzr, [x29,88]
	ldr	w0, [x29,88]
	bfi	w0, w1, 9, 5
	str	w0, [x29,88]
	ldr	w0, [x29,88]
	orr	w0, w0, 448
	str	w0, [x29,88]
	cbnz	x22, .L343
	ldr	w0, [x29,88]
	mov	w1, 2
	bfi	w0, w1, 3, 3
	str	w0, [x29,88]
.L343:
	ldr	w0, [x29,88]
	cmp	w24, wzr
	cset	w1, eq
	orr	w0, w0, 4
	str	w0, [x29,88]
	ldr	w0, [x29,88]
	bfi	w0, w1, 1, 1
	str	w0, [x29,88]
	ldr	w0, [x29,88]
	orr	w0, w0, 1
	str	w0, [x29,88]
.L332:
	ldr	w0, [x29,88]
	str	w0, [x21,16]
.L331:
	str	w23, [x21,12]
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
	.global	Ftl_log2
	.type	Ftl_log2, %function
Ftl_log2:
	mov	w2, 1
	mov	w1, 0
.L350:
	cmp	w2, w0
	bhi	.L352
	add	w1, w1, 1
	lsl	w2, w2, 1
	uxth	w1, w1
	b	.L350
.L352:
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
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w2, [x1,1952]
	ldrh	w3, [x1,1962]
	str	w0, [x1,1948]
	mul	w2, w0, w2
	sub	w0, w3, w0
	strh	w0, [x1,1960]
	ldr	w0, [x1,1968]
	str	w2, [x1,1956]
	sub	w2, w0, w2
	mov	w0, 0
	str	w2, [x1,1964]
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
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	str	x27, [sp,80]
	mov	x23, x0
	adrp	x19, .LANCHOR0
	ldrh	w6, [x0,8]
	add	x0, x19, :lo12:.LANCHOR0
	ldrh	w2, [x23,10]
	ldrh	w1, [x23,12]
	ldrh	w4, [x23,14]
	strh	w6, [x0,1972]
	strh	w2, [x0,1974]
	strh	w1, [x0,1976]
	strh	w4, [x0,1962]
.L357:
	add	x5, x0, 1984
	strb	w3, [x3,x5]
	add	x3, x3, 1
	cmp	x3, 32
	bne	.L357
	ldrh	w3, [x23,20]
	ldrb	w0, [x23,15]
	cmp	w3, w0
	bcs	.L358
	uxtb	w8, w1
	mov	w3, 0
	mul	w11, w1, w2
	ubfiz	w10, w8, 1, 7
	add	x12, x19, :lo12:.LANCHOR0
.L359:
	cmp	w3, w1
	bcs	.L361
	uxtb	w0, w3
	mov	w5, w3
	mov	w7, 0
.L362:
	cmp	w7, w2
	bcs	.L380
	add	x9, x12, 1984
	add	w13, w5, w11
	add	w14, w0, w8
	add	w7, w7, 1
	strb	w0, [x9,w5,uxtw]
	add	w0, w0, w10
	strb	w14, [x9,x13]
	add	w5, w5, w1
	uxtb	w0, w0
	b	.L362
.L380:
	add	w3, w3, 1
	b	.L359
.L361:
	add	x0, x19, :lo12:.LANCHOR0
	lsl	w2, w2, 1
	lsr	w4, w4, 1
	strh	w2, [x0,1974]
	strh	w4, [x0,1962]
.L358:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 5
	cmp	w6, 1
	strh	w2, [x0,2016]
	strh	wzr, [x0,2018]
	bne	.L363
	strh	w6, [x0,2016]
.L363:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 4352
	ldrb	w26, [x0,8]
	strh	w2, [x0,2020]
	cbz	w26, .L364
	mov	w2, 384
	strh	w2, [x0,2020]
.L364:
	add	x21, x19, :lo12:.LANCHOR0
	ldrh	w20, [x21,1962]
	ldrh	w22, [x21,1974]
	mul	w22, w1, w22
	mul	w1, w20, w1
	uxth	w22, w22
	strh	w22, [x21,1952]
	uxth	w0, w1
	strh	w0, [x21,2022]
	bl	Ftl_log2
	ldrh	w25, [x23,16]
	ldrh	w24, [x23,20]
	strh	w0, [x21,2024]
	mul	w0, w22, w25
	strh	w0, [x21,2030]
	mov	w0, w24
	ldrh	w27, [x23,18]
	strh	w25, [x21,2026]
	strh	w27, [x21,2028]
	strh	w24, [x21,2032]
	bl	Ftl_log2
	lsl	w2, w24, 9
	uxth	w3, w0
	strh	w0, [x21,2034]
	cmp	w20, 1024
	ldrh	w0, [x23,26]
	strh	w2, [x21,2036]
	ubfx	x2, x2, 9, 7
	strh	w0, [x21,2040]
	lsl	w2, w2, 1
	mul	w0, w22, w20
	strh	w2, [x21,2038]
	str	w0, [x21,1968]
	bls	.L365
	and	w0, w20, 255
	strh	w0, [x21,2018]
.L365:
	add	x2, x19, :lo12:.LANCHOR0
	mul	w0, w24, w27
	ldrh	w1, [x2,2018]
	sub	w1, w20, w1
	mul	w1, w22, w1
	mul	w1, w1, w24
	mul	w1, w1, w25
	asr	w1, w1, 11
	str	w1, [x2,2044]
	ldrh	w1, [x2,2020]
	lsl	w1, w1, 3
	sdiv	w0, w1, w0
	uxth	w0, w0
	cmp	w0, 4
	bhi	.L378
	mov	w0, 4
.L378:
	strh	w0, [x2,2048]
	cbz	w26, .L368
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, 640
	strh	w1, [x0,2020]
.L368:
	add	x1, x19, :lo12:.LANCHOR0
	lsl	w20, w20, 6
	cmp	w22, 1
	ldrh	w0, [x1,2020]
	asr	w0, w0, w3
	add	w3, w3, 9
	asr	w20, w20, w3
	strh	w20, [x1,2052]
	and	w20, w20, 65535
	add	w0, w0, 2
	strh	w0, [x1,2050]
	mul	w0, w20, w22
	str	w0, [x1,2056]
	add	w20, w20, 8
	ldrh	w0, [x1,2048]
	udiv	w0, w0, w22
	add	w20, w20, w0
	bne	.L379
	add	w20, w20, 4
.L379:
	add	x19, x19, :lo12:.LANCHOR0
	str	w20, [x1,1948]
	ldrh	w0, [x19,1948]
	bl	FtlSysBlkNumInit
	str	wzr, [x19,2068]
	ldr	w0, [x19,1948]
	str	w0, [x19,2060]
	ldr	w0, [x19,1964]
	ldrh	w1, [x19,2026]
	lsl	w0, w0, 2
	ldr	x27, [sp,80]
	ldp	x21, x22, [sp,32]
	mul	w1, w0, w1
	ldrh	w0, [x19,2034]
	ldp	x23, x24, [sp,48]
	add	w0, w0, 9
	lsr	w0, w1, w0
	ldrh	w1, [x19,2032]
	add	w0, w0, 2
	strh	w0, [x19,2064]
	mov	w0, 2048
	ldp	x25, x26, [sp,64]
	sdiv	w0, w0, w1
	strh	w0, [x19,2066]
	ldrh	w0, [x19,2048]
	add	w0, w0, 3
	strh	w0, [x19,2048]
	ldr	w0, [x19,2056]
	add	w0, w0, 3
	str	w0, [x19,2056]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlConstantsInit, .-FtlConstantsInit
	.align	2
	.global	IsBlkInVendorPart
	.type	IsBlkInVendorPart, %function
IsBlkInVendorPart:
	adrp	x1, .LANCHOR0
	uxth	w4, w0
	add	x1, x1, :lo12:.LANCHOR0
	mov	w0, 0
	ldrh	w2, [x1,2072]
	cbz	w2, .L382
	ldrh	w2, [x1,2048]
	ldr	x3, [x1,2080]
	mov	x1, 0
.L383:
	cmp	w2, w1, uxth
	bls	.L388
	add	x1, x1, 1
	add	x0, x3, x1, lsl 1
	ldrh	w0, [x0,-2]
	cmp	w0, w4
	bne	.L383
	mov	w0, 1
	b	.L382
.L388:
	mov	w0, 0
.L382:
	ret
	.size	IsBlkInVendorPart, .-IsBlkInVendorPart
	.align	2
	.global	FtlCacheMetchLpa
	.type	FtlCacheMetchLpa, %function
FtlCacheMetchLpa:
	adrp	x2, .LANCHOR0
	mov	w6, w0
	add	x2, x2, :lo12:.LANCHOR0
	mov	w0, 0
	ldr	w3, [x2,2088]
	cbz	w3, .L390
	ldr	x5, [x2,2096]
	mov	x0, 56
	mov	x2, 0
.L391:
	cmp	w3, w2
	bls	.L400
	madd	x4, x2, x0, x5
	add	x2, x2, 1
	ldr	w4, [x4,24]
	cmp	w4, w6
	bcc	.L391
	cmp	w4, w1
	bhi	.L391
	mov	w0, 1
	b	.L390
.L400:
	mov	w0, 0
.L390:
	ret
	.size	FtlCacheMetchLpa, .-FtlCacheMetchLpa
	.align	2
	.global	FtlGetCap
	.type	FtlGetCap, %function
FtlGetCap:
	adrp	x0, .LANCHOR0+2068
	ldr	w0, [x0,#:lo12:.LANCHOR0+2068]
	ret
	.size	FtlGetCap, .-FtlGetCap
	.align	2
	.global	FtlGetCapacity
	.type	FtlGetCapacity, %function
FtlGetCapacity:
	adrp	x0, .LANCHOR0+2068
	ldr	w0, [x0,#:lo12:.LANCHOR0+2068]
	ret
	.size	FtlGetCapacity, .-FtlGetCapacity
	.align	2
	.global	FtlGetLpn
	.type	FtlGetLpn, %function
FtlGetLpn:
	adrp	x0, .LANCHOR0+2104
	ldr	w0, [x0,#:lo12:.LANCHOR0+2104]
	ret
	.size	FtlGetLpn, .-FtlGetLpn
	.align	2
	.global	FtlGetCurEraseBlock
	.type	FtlGetCurEraseBlock, %function
FtlGetCurEraseBlock:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w2, [x1,1952]
	ldr	w0, [x1,2108]
	mul	w0, w2, w0
	ret
	.size	FtlGetCurEraseBlock, .-FtlGetCurEraseBlock
	.align	2
	.global	FtlGetAllBlockNum
	.type	FtlGetAllBlockNum, %function
FtlGetAllBlockNum:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w2, [x1,1962]
	ldrh	w0, [x1,1952]
	mul	w0, w2, w0
	ret
	.size	FtlGetAllBlockNum, .-FtlGetAllBlockNum
	.align	2
	.global	FtlBbmMapBadBlock
	.type	FtlBbmMapBadBlock, %function
FtlBbmMapBadBlock:
	stp	x29, x30, [sp, -32]!
	uxth	w1, w0
	mov	w4, 1
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	add	x19, x19, 2112
	ldrh	w3, [x19,-90]
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
	adrp	x0, .LC1
	add	x0, x0, :lo12:.LC1
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
	adrp	x2, .LANCHOR0
	uxth	w0, w0
	add	x3, x2, :lo12:.LANCHOR0
	ldrh	w1, [x3,2022]
	udiv	w2, w0, w1
	msub	w0, w2, w1, w0
	add	x2, x3, x2, uxth 3
	ldr	x1, [x2,2144]
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
	.global	FtlBbtCalcTotleCnt
	.type	FtlBbtCalcTotleCnt, %function
FtlBbtCalcTotleCnt:
	stp	x29, x30, [sp, -48]!
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	mov	w19, 0
	mov	w20, w19
	ldrh	w1, [x0,2022]
	ldrh	w21, [x0,1974]
	mul	w21, w1, w21
.L410:
	cmp	w20, w21
	bge	.L416
	mov	w0, w20
	bl	FtlBbmIsBadBlock
	cbz	w0, .L411
	add	w19, w19, 1
	uxth	w19, w19
.L411:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L410
.L416:
	mov	w0, w19
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlBbtCalcTotleCnt, .-FtlBbtCalcTotleCnt
	.align	2
	.global	V2P_block
	.type	V2P_block, %function
V2P_block:
	adrp	x3, .LANCHOR0
	uxth	w0, w0
	add	x3, x3, :lo12:.LANCHOR0
	uxth	w1, w1
	ldrh	w4, [x3,1976]
	udiv	w2, w0, w4
	msub	w0, w2, w4, w0
	madd	w0, w1, w4, w0
	ldrh	w1, [x3,2022]
	madd	w0, w2, w1, w0
	ret
	.size	V2P_block, .-V2P_block
	.align	2
	.global	P2V_plane
	.type	P2V_plane, %function
P2V_plane:
	adrp	x2, .LANCHOR0
	uxth	w0, w0
	add	x2, x2, :lo12:.LANCHOR0
	ldrh	w1, [x2,1976]
	ldrh	w3, [x2,2022]
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
	adrp	x2, .LANCHOR0
	uxth	w0, w0
	add	x2, x2, :lo12:.LANCHOR0
	ldrh	w3, [x2,2022]
	udiv	w1, w0, w3
	msub	w0, w1, w3, w0
	ldrh	w1, [x2,1976]
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
	bls	.L421
	sub	w1, w0, w1
	cmp	w1, w2
	cset	w0, ls
	b	.L422
.L421:
	sub	w1, w1, w0
	cmp	w1, w2
	cset	w0, hi
.L422:
	ret
	.size	ftl_cmp_data_ver, .-ftl_cmp_data_ver
	.align	2
	.global	FtlFreeSysBlkQueueEmpty
	.type	FtlFreeSysBlkQueueEmpty, %function
FtlFreeSysBlkQueueEmpty:
	adrp	x0, .LANCHOR0+2214
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2214]
	cmp	w0, wzr
	cset	w0, eq
	ret
	.size	FtlFreeSysBlkQueueEmpty, .-FtlFreeSysBlkQueueEmpty
	.align	2
	.global	FtlFreeSysBlkQueueFull
	.type	FtlFreeSysBlkQueueFull, %function
FtlFreeSysBlkQueueFull:
	adrp	x0, .LANCHOR0+2214
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2214]
	cmp	w0, 1024
	cset	w0, eq
	ret
	.size	FtlFreeSysBlkQueueFull, .-FtlFreeSysBlkQueueFull
	.align	2
	.global	FtlFreeSysBlkQueueIn
	.type	FtlFreeSysBlkQueueIn, %function
FtlFreeSysBlkQueueIn:
	stp	x29, x30, [sp, -48]!
	adrp	x2, .LANCHOR0
	uxth	w1, w1
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	uxth	w22, w0
	add	x0, x2, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	mov	x19, x2
	ldrh	w0, [x0,2214]
	cmp	w0, 1024
	beq	.L425
	cbz	w1, .L427
	adrp	x20, .LANCHOR2
	mov	w0, w22
	add	x20, x20, :lo12:.LANCHOR2
	bl	P2V_block_in_plane
	uxth	w21, w0
	lsl	w1, w22, 10
	ldr	x0, [x20,-88]
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	ldr	x0, [x20,-88]
	bl	FlashEraseBlocks
	ubfiz	x1, x21, 1, 16
	ldr	x2, [x20,-80]
	ldrh	w0, [x2,x1]
	add	w0, w0, 1
	strh	w0, [x2,x1]
	ldr	w0, [x20,-72]
	add	w0, w0, 1
	str	w0, [x20,-72]
.L427:
	add	x1, x19, :lo12:.LANCHOR0
	add	x1, x1, 2208
	ldrh	w0, [x1,6]
	ldrh	w2, [x1,4]
	add	w0, w0, 1
	strh	w0, [x1,6]
	add	x0, x1, x2, sxtw 1
	add	w2, w2, 1
	and	w2, w2, 1023
	strh	w22, [x0,8]
	strh	w2, [x1,4]
.L425:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlFreeSysBlkQueueIn, .-FtlFreeSysBlkQueueIn
	.align	2
	.global	FtlFreeSysBLkSort
	.type	FtlFreeSysBLkSort, %function
FtlFreeSysBLkSort:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	str	x21, [sp,32]
	add	x20, x20, :lo12:.LANCHOR0
	mov	w19, 0
	add	x20, x20, 2208
	adrp	x21, .LANCHOR2
.L435:
	ldrh	w0, [x20,6]
	cmp	w0, w19
	bls	.L443
	ldrh	w0, [x20,2]
	add	w0, w0, w19
	add	x0, x20, x0, sxtw 1
	ldrh	w0, [x0,8]
	bl	P2V_block_in_plane
	ubfiz	x0, x0, 1, 16
	add	x1, x21, :lo12:.LANCHOR2
	ubfiz	x2, x19, 2, 16
	add	w19, w19, 1
	ldr	x3, [x1,-64]
	uxth	w19, w19
	ldr	x1, [x1,-80]
	ldrh	w0, [x1,x0]
	str	w0, [x3,x2]
	b	.L435
.L443:
	adrp	x4, .LANCHOR0
	adrp	x7, .LANCHOR2
	mov	w0, 0
	add	x4, x4, :lo12:.LANCHOR0
	mov	x8, x7
.L437:
	ldrh	w6, [x4,2214]
	sub	w1, w6, #1
	cmp	w0, w1
	bge	.L444
	add	x1, x7, :lo12:.LANCHOR2
	add	w5, w0, 1
	uxth	w5, w5
	ldr	x3, [x1,-64]
	mov	w2, w5
	mov	w1, w0
.L438:
	cmp	w2, w6
	bcs	.L445
	ubfiz	x9, x2, 2, 16
	ldr	w10, [x3,w1,uxtw 2]
	ldr	w9, [x3,x9]
	cmp	w10, w9
	csel	w1, w1, w2, ls
	add	w2, w2, 1
	uxth	w2, w2
	b	.L438
.L445:
	cmp	w0, w1
	beq	.L441
	ubfiz	x2, x0, 2, 16
	ubfiz	x6, x1, 2, 32
	ldr	w10, [x3,x2]
	ldr	w9, [x3,x6]
	str	w10, [x3,x6]
	add	x3, x8, :lo12:.LANCHOR2
	ldr	x3, [x3,-64]
	str	w9, [x3,x2]
	add	x2, x4, 2208
	ldrh	w3, [x4,2210]
	add	w1, w3, w1
	add	w0, w3, w0
	add	x1, x2, x1, sxtw 1
	add	x0, x2, x0, sxtw 1
	ldrh	w2, [x0,8]
	ldrh	w6, [x1,8]
	strh	w2, [x1,8]
	strh	w6, [x0,8]
.L441:
	mov	w0, w5
	b	.L437
.L444:
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlFreeSysBLkSort, .-FtlFreeSysBLkSort
	.align	2
	.global	IsInFreeQueue
	.type	IsInFreeQueue, %function
IsInFreeQueue:
	adrp	x1, .LANCHOR0
	uxth	w6, w0
	add	x1, x1, :lo12:.LANCHOR0
	mov	w0, 0
	add	x1, x1, 2208
	ldrh	w4, [x1,6]
	cmp	w4, 1024
	beq	.L447
	ldrh	w5, [x1,2]
	mov	w3, w0
.L448:
	cmp	w3, w4
	bcs	.L452
	add	w2, w3, w5
	ubfiz	x2, x2, 1, 10
	add	x2, x1, x2
	ldrh	w0, [x2,8]
	cmp	w0, w6
	beq	.L451
	add	w3, w3, 1
	b	.L448
.L452:
	mov	w0, 0
	b	.L447
.L451:
	mov	w0, 1
.L447:
	ret
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	stp	x29, x30, [sp, -48]!
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	add	x29, sp, 0
	add	x2, x0, 2208
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	mov	w20, 65535
	ldrh	w3, [x0,2214]
	cbz	w3, .L454
	ldrh	w1, [x0,2210]
	add	x4, x2, x1, sxtw 1
	sub	w3, w3, #1
	strh	w3, [x0,2214]
	add	w1, w1, 1
	ldrh	w20, [x4,8]
	and	w1, w1, 1023
	strh	w1, [x0,2210]
	ldrb	w0, [x0,120]
	cbz	w0, .L454
	adrp	x19, .LANCHOR2
	mov	w0, w20
	add	x19, x19, :lo12:.LANCHOR2
	bl	P2V_block_in_plane
	uxth	w21, w0
	lsl	w1, w20, 10
	ldr	x0, [x19,-88]
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	ldr	x0, [x19,-88]
	bl	FlashEraseBlocks
	ubfiz	x0, x21, 1, 16
	ldr	x2, [x19,-80]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
	ldr	w0, [x19,-72]
	add	w0, w0, 1
	str	w0, [x19,-72]
.L454:
	mov	w0, w20
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	uxth	w0, w0
	mov	w9, 65535
	cmp	w0, w9
	beq	.L461
	adrp	x2, .LANCHOR2
	mov	w7, 6
	add	x5, x2, :lo12:.LANCHOR2
	mov	w1, -1
	umull	x10, w0, w7
	mov	x3, x2
	ldr	x11, [x5,-56]
	add	x4, x11, x10
	strh	w1, [x4,2]
	strh	w1, [x11,x10]
	ldr	x1, [x5,-48]
	cbnz	x1, .L462
	str	x4, [x5,-48]
	b	.L461
.L462:
	ubfiz	x8, x0, 1, 16
	ldr	x13, [x5,-40]
	ldrh	w2, [x4,4]
	mov	x14, -6148914691236517206
	ldr	x12, [x5,-56]
	ldrh	w6, [x13,x8]
	cmp	w2, wzr
	mul	w6, w6, w2
	sub	x2, x1, x12
	asr	x2, x2, 1
	csinv	w6, w6, wzr, ne
	madd	x2, x14, x2, x2
	ldr	x14, [x5,-80]
	uxth	w2, w2
	add	x8, x14, x8
.L469:
	ubfiz	x15, x2, 1, 16
	ldrh	w16, [x1,4]
	cmp	w16, wzr
	ldrh	w5, [x13,x15]
	mul	w5, w5, w16
	csinv	w5, w5, wzr, ne
	cmp	w5, w6
	bne	.L465
	ldrh	w15, [x14,x15]
	ldrh	w5, [x8]
	cmp	w15, w5
	bcc	.L467
	b	.L466
.L465:
	bhi	.L466
.L467:
	ldrh	w5, [x1]
	cmp	w5, w9
	bne	.L468
	strh	w2, [x4,2]
	add	x2, x3, :lo12:.LANCHOR2
	strh	w0, [x1]
	str	x4, [x2,-32]
	b	.L461
.L468:
	umull	x1, w5, w7
	mov	w2, w5
	add	x1, x12, x1
	b	.L469
.L466:
	strh	w2, [x11,x10]
	ldrh	w2, [x1,2]
	strh	w2, [x4,2]
	add	x2, x3, :lo12:.LANCHOR2
	ldr	x3, [x2,-48]
	cmp	x1, x3
	bne	.L470
	strh	w0, [x1,2]
	str	x4, [x2,-48]
	b	.L461
.L470:
	ldrh	w3, [x1,2]
	mov	w4, 6
	ldr	x2, [x2,-56]
	umull	x3, w3, w4
	strh	w0, [x2,x3]
	strh	w0, [x1,2]
.L461:
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
	ldrh	w1, [x0,-24]
	add	w1, w1, 1
	strh	w1, [x0,-24]
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
	beq	.L475
	adrp	x2, .LANCHOR2
	mov	w6, 6
	add	x5, x2, :lo12:.LANCHOR2
	mov	w1, -1
	umull	x8, w0, w6
	mov	x3, x2
	ldr	x9, [x5,-56]
	add	x4, x9, x8
	strh	w1, [x4,2]
	strh	w1, [x9,x8]
	ldr	x1, [x5,-16]
	cbnz	x1, .L476
	str	x4, [x5,-16]
	b	.L475
.L476:
	ldr	x11, [x5,-80]
	ubfiz	x2, x0, 1, 16
	ldr	x10, [x5,-56]
	mov	x5, -6148914691236517206
	ldrh	w12, [x11,x2]
	sub	x2, x1, x10
	asr	x2, x2, 1
	madd	x2, x5, x2, x2
	uxth	w2, w2
.L479:
	ubfiz	x5, x2, 1, 16
	ldrh	w5, [x11,x5]
	cmp	w5, w12
	bcs	.L477
	ldrh	w5, [x1]
	cmp	w5, w7
	bne	.L478
	strh	w2, [x4,2]
	strh	w0, [x1]
	b	.L475
.L478:
	umull	x1, w5, w6
	mov	w2, w5
	add	x1, x10, x1
	b	.L479
.L477:
	ldrh	w5, [x1,2]
	strh	w5, [x4,2]
	strh	w2, [x9,x8]
	add	x2, x3, :lo12:.LANCHOR2
	ldr	x3, [x2,-16]
	cmp	x1, x3
	bne	.L480
	strh	w0, [x1,2]
	str	x4, [x2,-16]
	b	.L475
.L480:
	ldrh	w3, [x1,2]
	mov	w4, 6
	ldr	x2, [x2,-56]
	umull	x3, w3, w4
	strh	w0, [x2,x3]
	strh	w0, [x1,2]
.L475:
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
	ldrh	w1, [x0,-8]
	add	w1, w1, 1
	strh	w1, [x0,-8]
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
	ldr	x2, [x6,-56]
	add	x4, x2, x1
	cmp	x4, x3
	bne	.L483
	ldrh	w3, [x2,x1]
	cmp	w3, w7
	bne	.L484
	str	xzr, [x0]
	b	.L485
.L484:
	umull	x3, w3, w5
	add	x3, x2, x3
	str	x3, [x0]
	mov	w0, -1
	strh	w0, [x3,2]
	b	.L485
.L483:
	ldrh	w0, [x2,x1]
	ldrh	w3, [x4,2]
	cmp	w0, w7
	bne	.L486
	umull	x3, w3, w5
	mov	w0, -1
	strh	w0, [x2,x3]
	b	.L485
.L486:
	umull	x0, w0, w5
	add	x0, x2, x0
	strh	w3, [x0,2]
	ldrh	w3, [x4,2]
	ldr	x0, [x6,-56]
	ldrh	w7, [x2,x1]
	umull	x3, w3, w5
	strh	w7, [x0,x3]
.L485:
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
	cbz	x2, .L488
	adrp	x3, .LANCHOR2-56
	mov	w5, w19
	mov	w6, 6
	ldr	x4, [x3,#:lo12:.LANCHOR2-56]
.L489:
	cbnz	w1, .L490
.L492:
	sub	x2, x2, x4
	mov	x1, -6148914691236517206
	asr	x2, x2, 1
	madd	x2, x1, x2, x2
	uxth	w19, w2
	mov	w1, w19
	bl	List_remove_node
	b	.L488
.L490:
	ldrh	w3, [x2]
	cmp	w3, w5
	beq	.L492
	umull	x3, w3, w6
	sub	w1, w1, #1
	add	x2, x4, x3
	uxth	w1, w1
	b	.L489
.L488:
	mov	w0, w19
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
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
	adrp	x2, .LANCHOR2
	uxth	w3, w0
	add	x2, x2, :lo12:.LANCHOR2
	mov	w0, 65535
	ldr	x1, [x2,-48]
	cbz	x1, .L501
	ldr	x4, [x2,-56]
	mov	w5, 6
.L497:
	cbz	w3, .L498
	ldrh	w2, [x1]
	cmp	w2, w0
	beq	.L501
	umull	x2, w2, w5
	sub	w3, w3, #1
	add	x1, x4, x2
	uxth	w3, w3
	b	.L497
.L498:
	sub	x1, x1, x4
	mov	x0, -6148914691236517206
	asr	x1, x1, 1
	madd	x1, x0, x1, x1
	uxth	w0, w1
.L501:
	ret
	.size	List_get_gc_head_node, .-List_get_gc_head_node
	.align	2
	.global	List_update_data_list
	.type	List_update_data_list, %function
List_update_data_list:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxth	w20, w0
	adrp	x0, .LANCHOR2
	add	x19, x0, :lo12:.LANCHOR2
	ldrh	w0, [x0,#:lo12:.LANCHOR2]
	cmp	w0, w20
	beq	.L503
	ldrh	w0, [x19,48]
	cmp	w0, w20
	beq	.L503
	ldrh	w0, [x19,96]
	cmp	w0, w20
	beq	.L503
	mov	w2, 6
	ldr	x4, [x19,-56]
	ldr	x0, [x19,-48]
	umull	x3, w20, w2
	add	x3, x4, x3
	cmp	x3, x0
	beq	.L503
	ldr	x5, [x19,-40]
	ubfiz	x0, x20, 1, 16
	ldrh	w1, [x3,4]
	ldrh	w0, [x5,x0]
	mul	w1, w0, w1
	ldrh	w0, [x3,2]
	mov	x3, -6148914691236517206
	cmp	w1, wzr
	csinv	w1, w1, wzr, ne
	umull	x2, w0, w2
	asr	x0, x2, 1
	add	x2, x4, x2
	madd	x0, x3, x0, x0
	ldrh	w2, [x2,4]
	ldrh	w0, [x5,x0,lsl 1]
	mul	w0, w0, w2
	cmp	w0, wzr
	csinv	w0, w0, wzr, ne
	cmp	w1, w0
	bcs	.L503
	sub	x0, x19, #48
	mov	w1, w20
	bl	List_remove_node
	ldrh	w0, [x19,-24]
	sub	w0, w0, #1
	strh	w0, [x19,-24]
	mov	w0, w20
	bl	INSERT_DATA_LIST
.L503:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	List_update_data_list, .-List_update_data_list
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
.L507:
	cmp	w20, w2
	beq	.L510
	mov	x22, x1
	ldrh	w21, [x1],2
	cbnz	w21, .L508
	mov	x19, x0
	bl	FtlFreeSysBlkQueueOut
	uxth	w1, w0
	strh	w0, [x22]
	cbz	w1, .L510
	ldr	w0, [x19,48]
	strh	w21, [x19,2]
	add	w0, w0, 1
	str	w0, [x19,48]
	ldrh	w0, [x19,8]
	strh	w20, [x19]
	add	w0, w0, 1
	strh	w0, [x19,8]
	b	.L510
.L508:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L507
.L510:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	ftl_map_blk_alloc_new_blk, .-ftl_map_blk_alloc_new_blk
	.align	2
	.global	select_l2p_ram_region
	.type	select_l2p_ram_region, %function
select_l2p_ram_region:
	adrp	x0, .LANCHOR0+2066
	mov	x4, 0
	mov	w5, 65535
	ldrh	w2, [x0,#:lo12:.LANCHOR0+2066]
	adrp	x0, .LANCHOR2
	add	x1, x0, :lo12:.LANCHOR2
	mov	x3, x0
	ldr	x1, [x1,144]
.L514:
	uxth	w0, w4
	cmp	w0, w2
	bcs	.L530
	add	x4, x4, 1
	add	x6, x1, x4, lsl 4
	ldrh	w6, [x6,-16]
	cmp	w6, w5
	bne	.L514
	b	.L515
.L530:
	mov	w0, w2
	mov	x4, 0
	mov	w7, -2147483648
.L517:
	uxth	w6, w4
	cmp	w6, w2
	bcs	.L531
	add	x5, x1, x4, lsl 4
	ldr	w5, [x5,4]
	tbnz	w5, #31, .L526
	cmp	w5, w7
	bcc	.L518
.L526:
	mov	w5, w7
	mov	w6, w0
.L518:
	add	x4, x4, 1
	mov	w7, w5
	mov	w0, w6
	b	.L517
.L531:
	cmp	w0, w2
	bcc	.L515
	add	x0, x3, :lo12:.LANCHOR2
	add	x1, x1, 4
	mov	w4, -1
	mov	w3, 0
	ldrh	w5, [x0,152]
	mov	w0, w2
.L521:
	cmp	w3, w2
	beq	.L515
	ldr	w7, [x1]
	cmp	w7, w4
	bcs	.L522
	ldrh	w6, [x1,-4]
	cmp	w6, w5
	csel	w4, w4, w7, eq
	cmp	w6, w5
	csel	w0, w0, w3, eq
.L522:
	add	w3, w3, 1
	add	x1, x1, 16
	uxth	w3, w3
	b	.L521
.L515:
	ret
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	adrp	x1, .LANCHOR2
	add	x3, x1, :lo12:.LANCHOR2
	ldrh	w2, [x3,154]
	cbnz	w0, .L533
	cmp	w2, 4
	bhi	.L533
	add	w2, w2, 1
	strh	w2, [x3,154]
	b	.L532
.L533:
	adrp	x2, .LANCHOR0+1960
	add	x0, x1, :lo12:.LANCHOR2
	mov	w7, 65535
	ldrh	w5, [x2,#:lo12:.LANCHOR0+1960]
	strh	wzr, [x0,154]
	str	wzr, [x0,156]
	ldr	x6, [x0,-40]
	mov	x0, 0
.L534:
	cmp	w5, w0, uxth
	bls	.L532
	ldrh	w4, [x6,x0,lsl 1]
	cmp	w4, w7
	beq	.L535
	add	x3, x1, :lo12:.LANCHOR2
	ldr	w2, [x3,156]
	add	w2, w4, w2
	str	w2, [x3,156]
.L535:
	add	x0, x0, 1
	b	.L534
.L532:
	ret
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	ftl_set_blk_mode
	.type	ftl_set_blk_mode, %function
ftl_set_blk_mode:
	stp	x29, x30, [sp, -16]!
	uxth	w0, w0
	add	x29, sp, 0
	cbz	w1, .L539
	bl	ftl_set_blk_mode.part.7
	b	.L538
.L539:
	adrp	x1, .LANCHOR0
	ubfx	x2, x0, 5, 11
	lsl	x2, x2, 2
	ldr	x3, [x1,#:lo12:.LANCHOR0]
	mov	w1, 1
	lsl	w0, w1, w0
	ldr	w1, [x3,x2]
	bic	w0, w1, w0
	str	w0, [x3,x2]
.L538:
	ldp	x29, x30, [sp], 16
	ret
	.size	ftl_set_blk_mode, .-ftl_set_blk_mode
	.align	2
	.global	ftl_get_blk_mode
	.type	ftl_get_blk_mode, %function
ftl_get_blk_mode:
	adrp	x1, .LANCHOR0
	uxth	w0, w0
	ubfx	x2, x0, 5, 11
	ldr	x1, [x1,#:lo12:.LANCHOR0]
	ldr	w1, [x1,x2,lsl 2]
	lsr	w0, w1, w0
	and	w0, w0, 1
	ret
	.size	ftl_get_blk_mode, .-ftl_get_blk_mode
	.align	2
	.global	ftl_sb_update_avl_pages
	.type	ftl_sb_update_avl_pages, %function
ftl_sb_update_avl_pages:
	adrp	x3, .LANCHOR0
	uxth	w1, w1
	add	x4, x3, :lo12:.LANCHOR0
	uxth	w2, w2
	strh	wzr, [x0,4]
	mov	w6, 65535
	ldrh	w4, [x4,1952]
.L543:
	cmp	w2, w4
	bcs	.L549
	add	x5, x0, x2, sxtw 1
	ldrh	w5, [x5,16]
	cmp	w5, w6
	beq	.L544
	ldrh	w5, [x0,4]
	add	w5, w5, 1
	strh	w5, [x0,4]
.L544:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L543
.L549:
	add	x3, x3, :lo12:.LANCHOR0
	mov	w6, 65535
	add	x4, x0, x4, uxth 1
	ldrh	w5, [x3,2026]
	mov	x3, x0
.L546:
	cmp	x3, x4
	beq	.L550
	ldrh	w2, [x3,16]
	cmp	w2, w6
	beq	.L547
	ldrh	w2, [x0,4]
	add	w2, w5, w2
	sub	w2, w2, #1
	sub	w2, w2, w1
	strh	w2, [x0,4]
.L547:
	add	x3, x3, 2
	b	.L546
.L550:
	ret
	.size	ftl_sb_update_avl_pages, .-ftl_sb_update_avl_pages
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
	adrp	x20, .LANCHOR0
	mov	x19, x0
	strh	wzr, [x0,4]
	add	x22, x19, 16
	strb	wzr, [x0,7]
	add	x0, x20, :lo12:.LANCHOR0
	mov	x21, 0
	mov	x23, x0
	mov	w25, -1
	ldrh	w24, [x0,1952]
.L552:
	cmp	w24, w21, uxth
	bls	.L563
	add	x0, x23, 1984
	ldrh	w1, [x19]
	ldrb	w0, [x0,x21]
	bl	V2P_block
	strh	w25, [x22]
	mov	w26, w0
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L553
	strh	w26, [x22]
	ldrb	w0, [x19,7]
	add	w0, w0, 1
	strb	w0, [x19,7]
.L553:
	add	x21, x21, 1
	add	x22, x22, 2
	b	.L552
.L563:
	add	x1, x20, :lo12:.LANCHOR0
	ldrb	w0, [x19,7]
	strb	wzr, [x19,9]
	ldrh	w1, [x1,2026]
	mul	w0, w0, w1
	strh	w0, [x19,4]
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldr	w1, [x0,160]
	cbz	w1, .L555
	ldrh	w1, [x19]
	ldr	x0, [x0,-80]
	ldrh	w0, [x0,x1,lsl 1]
	cmp	w0, 59
	bhi	.L555
	mov	w0, 1
	strb	w0, [x19,9]
.L555:
	add	x20, x20, :lo12:.LANCHOR0
	ldrb	w0, [x20,120]
	cbz	w0, .L556
	mov	w0, 1
	strb	w0, [x19,9]
.L556:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	make_superblock, .-make_superblock
	.align	2
	.global	update_multiplier_value
	.type	update_multiplier_value, %function
update_multiplier_value:
	stp	x29, x30, [sp, -64]!
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	x22, 0
	uxth	w21, w0
	ldrh	w23, [x1,1952]
	mov	w19, w22
	ldrh	w24, [x1,2026]
	mov	x20, x1
.L565:
	cmp	w23, w22, uxth
	bls	.L572
	add	x0, x20, 1984
	mov	w1, w21
	ldrb	w0, [x0,x22]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L566
	add	w19, w19, w24
	uxth	w19, w19
.L566:
	add	x22, x22, 1
	b	.L565
.L572:
	cbz	w19, .L568
	mov	w0, 32768
	sdiv	w19, w0, w19
.L568:
	mov	w0, 6
	umull	x21, w21, w0
	adrp	x0, .LANCHOR2-56
	ldr	x0, [x0,#:lo12:.LANCHOR2-56]
	add	x21, x0, x21
	mov	w0, 0
	strh	w19, [x21,4]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	update_multiplier_value, .-update_multiplier_value
	.align	2
	.global	GetFreeBlockMinEraseCount
	.type	GetFreeBlockMinEraseCount, %function
GetFreeBlockMinEraseCount:
	adrp	x2, .LANCHOR2
	add	x2, x2, :lo12:.LANCHOR2
	ldr	x1, [x2,-16]
	mov	w0, w1
	cbz	x1, .L574
	ldr	x0, [x2,-56]
	sub	x1, x1, x0
	mov	x0, -6148914691236517206
	asr	x1, x1, 1
	madd	x1, x0, x1, x1
	ldr	x0, [x2,-80]
	and	x1, x1, 65535
	ldrh	w0, [x0,x1,lsl 1]
.L574:
	ret
	.size	GetFreeBlockMinEraseCount, .-GetFreeBlockMinEraseCount
	.align	2
	.global	GetFreeBlockMaxEraseCount
	.type	GetFreeBlockMaxEraseCount, %function
GetFreeBlockMaxEraseCount:
	adrp	x2, .LANCHOR2
	uxth	w4, w0
	add	x5, x2, :lo12:.LANCHOR2
	ldr	x1, [x5,-16]
	mov	w0, w1
	cbz	x1, .L577
	ldrh	w3, [x5,-8]
	mov	w0, 7
	mov	w6, 6
	mov	w7, 65535
	mul	w3, w3, w0
	ldr	x0, [x5,-56]
	asr	w3, w3, 3
	cmp	w4, w3
	sub	x1, x1, x0
	csel	w4, w3, w4, gt
	asr	x1, x1, 1
	mov	x3, -6148914691236517206
	madd	x1, x3, x1, x1
	mov	w3, 0
	uxth	w1, w1
.L579:
	cmp	w3, w4
	beq	.L582
	umull	x5, w1, w6
	ldrh	w5, [x0,x5]
	cmp	w5, w7
	bne	.L580
.L582:
	add	x2, x2, :lo12:.LANCHOR2
	ubfiz	x1, x1, 1, 16
	ldr	x0, [x2,-80]
	ldrh	w0, [x0,x1]
	b	.L577
.L580:
	add	w3, w3, 1
	mov	w1, w5
	uxth	w3, w3
	b	.L579
.L577:
	ret
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	2
	.global	FtlPrintInfo2buf
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	stp	x29, x30, [sp, -112]!
	adrp	x1, .LC2
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	adrp	x25, .LANCHOR0
	add	x1, x1, :lo12:.LC2
	add	x25, x25, :lo12:.LANCHOR0
	str	x27, [sp,80]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	mov	x23, x0
	bl	strcpy
	add	x20, x23, 12
	ldr	w2, [x25,168]
	adrp	x1, .LC3
	mov	x0, x20
	add	x1, x1, :lo12:.LC3
	bl	sprintf
	add	x20, x20, x0, sxtw
	adrp	x1, .LC4
	ldr	w2, [x25,2044]
	mov	x0, x20
	add	x1, x1, :lo12:.LC4
	bl	sprintf
	add	x20, x20, x0, sxtw
	adrp	x0, .LANCHOR1+532
	ldr	w0, [x0,#:lo12:.LANCHOR1+532]
	cmp	w0, 1
	beq	.L585
	sub	w0, w20, w23
	b	.L586
.L585:
	add	x0, x29, 96
	add	x1, x29, 100
	add	x2, x29, 104
	add	x3, x29, 108
	adrp	x24, .LANCHOR2
	bl	NandcGetTimeCfg
	add	x22, x24, :lo12:.LANCHOR2
	ldr	w4, [x29,104]
	adrp	x1, .LC5
	ldr	w5, [x29,108]
	add	x1, x1, :lo12:.LC5
	ldr	w3, [x29,100]
	mov	x0, x20
	ldr	w2, [x29,96]
	bl	sprintf
	add	x21, x20, x0, sxtw
	adrp	x1, .LC6
	mov	x0, x21
	add	x1, x1, :lo12:.LC6
	add	x21, x21, 10
	bl	strcpy
	ldr	w2, [x25,2104]
	adrp	x1, .LC7
	mov	x0, x21
	add	x1, x1, :lo12:.LC7
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,156]
	adrp	x1, .LC8
	mov	x0, x21
	add	x1, x1, :lo12:.LC8
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,164]
	adrp	x1, .LC9
	mov	x0, x21
	add	x1, x1, :lo12:.LC9
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,168]
	adrp	x1, .LC10
	mov	x0, x21
	add	x1, x1, :lo12:.LC10
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,172]
	adrp	x1, .LC11
	mov	x0, x21
	add	x1, x1, :lo12:.LC11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,176]
	adrp	x1, .LC12
	mov	x0, x21
	add	x1, x1, :lo12:.LC12
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,180]
	adrp	x1, .LC13
	mov	x0, x21
	add	x1, x1, :lo12:.LC13
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,184]
	adrp	x1, .LC14
	mov	x0, x21
	add	x1, x1, :lo12:.LC14
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,188]
	adrp	x1, .LC15
	mov	x0, x21
	add	x1, x1, :lo12:.LC15
	lsr	w2, w2, 11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,192]
	adrp	x1, .LC16
	mov	x0, x21
	add	x1, x1, :lo12:.LC16
	lsr	w2, w2, 11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,196]
	adrp	x1, .LC17
	mov	x0, x21
	add	x1, x1, :lo12:.LC17
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,200]
	adrp	x1, .LC18
	add	x1, x1, :lo12:.LC18
	mov	x0, x21
	bl	sprintf
	add	x21, x21, x0, sxtw
	bl	FtlBbtCalcTotleCnt
	uxth	w3, w0
	ldrh	w2, [x25,2118]
	adrp	x1, .LC19
	mov	x0, x21
	add	x1, x1, :lo12:.LC19
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,-8]
	adrp	x1, .LC20
	mov	x0, x21
	add	x1, x1, :lo12:.LC20
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,204]
	adrp	x1, .LC21
	mov	x0, x21
	add	x1, x1, :lo12:.LC21
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,208]
	adrp	x1, .LC22
	mov	x0, x21
	add	x1, x1, :lo12:.LC22
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,212]
	adrp	x1, .LC23
	mov	x0, x21
	add	x1, x1, :lo12:.LC23
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,-72]
	adrp	x1, .LC24
	mov	x0, x21
	add	x1, x1, :lo12:.LC24
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,216]
	adrp	x1, .LC25
	mov	x0, x21
	add	x1, x1, :lo12:.LC25
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,220]
	adrp	x1, .LC26
	mov	x0, x21
	add	x1, x1, :lo12:.LC26
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,254]
	adrp	x1, .LC27
	mov	x0, x21
	add	x1, x1, :lo12:.LC27
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,252]
	adrp	x1, .LC28
	mov	x0, x21
	add	x1, x1, :lo12:.LC28
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,2068]
	adrp	x1, .LC29
	mov	x0, x21
	add	x1, x1, :lo12:.LC29
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,2060]
	adrp	x1, .LC30
	mov	x0, x21
	add	x1, x1, :lo12:.LC30
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,1948]
	adrp	x1, .LC31
	mov	x0, x21
	add	x1, x1, :lo12:.LC31
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x25,2214]
	adrp	x1, .LC32
	mov	x0, x21
	add	x1, x1, :lo12:.LC32
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x25,1960]
	adrp	x1, .LC33
	mov	x0, x21
	add	x1, x1, :lo12:.LC33
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,272]
	adrp	x1, .LC34
	mov	x0, x21
	add	x1, x1, :lo12:.LC34
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,1964]
	adrp	x1, .LC35
	mov	x0, x21
	add	x1, x1, :lo12:.LC35
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,280]
	adrp	x1, .LC36
	mov	x0, x21
	add	x1, x1, :lo12:.LC36
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x25,2112]
	adrp	x1, .LC37
	mov	x0, x21
	add	x1, x1, :lo12:.LC37
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,2]
	adrp	x1, .LC38
	mov	x0, x21
	add	x1, x1, :lo12:.LC38
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,6]
	adrp	x1, .LC39
	mov	x0, x21
	add	x1, x1, :lo12:.LC39
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x24,#:lo12:.LANCHOR2]
	adrp	x1, .LC40
	mov	x0, x21
	add	x1, x1, :lo12:.LC40
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,8]
	adrp	x1, .LC41
	mov	x0, x21
	add	x1, x1, :lo12:.LC41
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,4]
	adrp	x1, .LC42
	mov	x0, x21
	add	x1, x1, :lo12:.LC42
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w3, [x24,#:lo12:.LANCHOR2]
	adrp	x1, .LC43
	ldr	x2, [x22,-40]
	mov	x0, x21
	add	x1, x1, :lo12:.LC43
	ldrh	w2, [x2,x3,lsl 1]
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,50]
	adrp	x1, .LC44
	mov	x0, x21
	add	x1, x1, :lo12:.LC44
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,54]
	adrp	x1, .LC45
	mov	x0, x21
	add	x1, x1, :lo12:.LC45
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,48]
	adrp	x1, .LC46
	mov	x0, x21
	add	x1, x1, :lo12:.LC46
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,56]
	adrp	x1, .LC47
	mov	x0, x21
	add	x1, x1, :lo12:.LC47
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,52]
	adrp	x1, .LC48
	mov	x0, x21
	add	x1, x1, :lo12:.LC48
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w3, [x22,48]
	adrp	x1, .LC49
	ldr	x2, [x22,-40]
	mov	x0, x21
	add	x1, x1, :lo12:.LC49
	ldrh	w2, [x2,x3,lsl 1]
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,98]
	adrp	x1, .LC50
	mov	x0, x21
	add	x1, x1, :lo12:.LC50
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,102]
	adrp	x1, .LC51
	mov	x0, x21
	add	x1, x1, :lo12:.LC51
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,96]
	adrp	x1, .LC52
	mov	x0, x21
	add	x1, x1, :lo12:.LC52
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,104]
	adrp	x1, .LC53
	mov	x0, x21
	add	x1, x1, :lo12:.LC53
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,100]
	adrp	x1, .LC54
	mov	x0, x21
	add	x1, x1, :lo12:.LC54
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,298]
	adrp	x1, .LC55
	mov	x0, x21
	add	x1, x1, :lo12:.LC55
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,302]
	adrp	x1, .LC56
	mov	x0, x21
	add	x1, x1, :lo12:.LC56
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,296]
	adrp	x1, .LC57
	mov	x0, x21
	add	x1, x1, :lo12:.LC57
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,304]
	adrp	x1, .LC58
	mov	x0, x21
	add	x1, x1, :lo12:.LC58
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,300]
	adrp	x1, .LC59
	mov	x0, x21
	add	x1, x1, :lo12:.LC59
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w3, [x22,344]
	adrp	x1, .LC60
	ldr	w2, [x22,160]
	add	x1, x1, :lo12:.LC60
	ldr	w4, [x22,436]
	mov	x0, x21
	ldr	w5, [x22,428]
	orr	w2, w2, w3, lsl 8
	ldr	w3, [x22,432]
	bl	sprintf
	add	x19, x21, x0, sxtw
	ldr	w2, [x22,424]
	adrp	x1, .LC61
	mov	x0, x19
	add	x1, x1, :lo12:.LC61
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	w2, [x22,448]
	adrp	x1, .LC62
	mov	x0, x19
	add	x1, x1, :lo12:.LC62
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,864]
	adrp	x1, .LC63
	mov	x0, x19
	add	x1, x1, :lo12:.LC63
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,866]
	adrp	x1, .LC64
	mov	x0, x19
	add	x1, x1, :lo12:.LC64
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	w2, [x22,868]
	adrp	x1, .LC65
	mov	x0, x19
	add	x1, x1, :lo12:.LC65
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,872]
	adrp	x1, .LC66
	add	x1, x1, :lo12:.LC66
	mov	x0, x19
	bl	sprintf
	add	x19, x19, x0, sxtw
	bl	GetFreeBlockMinEraseCount
	uxth	w2, w0
	adrp	x1, .LC67
	mov	x0, x19
	add	x1, x1, :lo12:.LC67
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w0, [x22,-8]
	bl	GetFreeBlockMaxEraseCount
	uxth	w2, w0
	adrp	x1, .LC68
	mov	x0, x19
	add	x1, x1, :lo12:.LC68
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w0, [x22,296]
	mov	w1, 65535
	cmp	w0, w1
	beq	.L587
	ubfiz	x2, x0, 1, 16
	ldr	x3, [x22,-40]
	adrp	x1, .LC69
	mov	x0, x19
	add	x1, x1, :lo12:.LC69
	ldrh	w2, [x3,x2]
	bl	sprintf
	add	x19, x19, x0, sxtw
.L587:
	mov	w0, 0
	adrp	x22, .LC70
	mov	w20, 0
	mov	w27, 65535
	bl	List_get_gc_head_node
	mov	w26, 6
	uxth	w3, w0
	add	x22, x22, :lo12:.LC70
.L589:
	cmp	w3, w27
	beq	.L588
	add	x21, x24, :lo12:.LANCHOR2
	ubfiz	x6, x3, 1, 16
	umull	x25, w3, w26
	mov	x0, x19
	mov	w2, w20
	ldr	x5, [x21,-56]
	mov	x1, x22
	ldr	x4, [x21,-40]
	add	w20, w20, 1
	ldr	x7, [x21,-80]
	add	x5, x5, x25
	ldrh	w4, [x4,x6]
	ldrh	w5, [x5,4]
	ldrh	w6, [x7,x6]
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	x0, [x21,-56]
	cmp	w20, 16
	ldrh	w3, [x0,x25]
	bne	.L589
.L588:
	add	x1, x24, :lo12:.LANCHOR2
	adrp	x22, .LC71
	mov	w20, 0
	mov	w27, 65535
	mov	w26, 6
	add	x22, x22, :lo12:.LC71
	ldr	x0, [x1,-16]
	ldr	x3, [x1,-56]
	sub	x3, x0, x3
	mov	x0, -6148914691236517206
	asr	x3, x3, 1
	madd	x3, x0, x3, x3
	uxth	w3, w3
.L591:
	cmp	w3, w27
	beq	.L590
	add	x21, x24, :lo12:.LANCHOR2
	ubfiz	x5, x3, 1, 16
	umull	x25, w3, w26
	mov	x0, x19
	mov	w2, w20
	ldr	x4, [x21,-56]
	mov	x1, x22
	ldr	x6, [x21,-80]
	add	w20, w20, 1
	add	x4, x4, x25
	ldrh	w5, [x6,x5]
	ldrh	w4, [x4,4]
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	x0, [x21,-56]
	cmp	w20, 4
	ldrh	w3, [x0,x25]
	bne	.L591
.L590:
	sub	w0, w19, w23
.L586:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldr	x27, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlPrintInfo2buf, .-FtlPrintInfo2buf
	.align	2
	.global	rknand_proc_ftlread
	.type	rknand_proc_ftlread, %function
rknand_proc_ftlread:
	stp	x29, x30, [sp, -32]!
	adrp	x1, .LC72
	adrp	x2, .LC73
	add	x29, sp, 0
	add	x1, x1, :lo12:.LC72
	add	x2, x2, :lo12:.LC73
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
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	2
	.global	GetSwlReplaceBlock
	.type	GetSwlReplaceBlock, %function
GetSwlReplaceBlock:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x0, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	ldr	w2, [x0,212]
	ldr	w1, [x0,220]
	cmp	w2, w1
	bcs	.L596
	adrp	x1, .LANCHOR0
	str	wzr, [x0,204]
	add	x2, x1, :lo12:.LANCHOR0
	ldr	x5, [x0,-80]
	mov	x0, 0
	ldrh	w3, [x2,1960]
.L597:
	cmp	w3, w0
	bls	.L638
	add	x4, x19, :lo12:.LANCHOR2
	ldrh	w6, [x5,x0,lsl 1]
	add	x0, x0, 1
	ldr	w2, [x4,204]
	add	w2, w6, w2
	str	w2, [x4,204]
	b	.L597
.L638:
	add	x2, x19, :lo12:.LANCHOR2
	add	x1, x1, :lo12:.LANCHOR0
	ldr	w0, [x2,204]
	ldrh	w1, [x1,2016]
	udiv	w3, w0, w3
	str	w3, [x2,212]
	ldr	w3, [x2,208]
	sub	w0, w0, w3
	udiv	w0, w0, w1
	str	w0, [x2,204]
	b	.L599
.L596:
	ldr	w1, [x0,216]
	cmp	w2, w1
	bls	.L599
	add	w1, w1, 1
	adrp	x4, .LANCHOR0
	str	w1, [x0,216]
	mov	w0, 0
.L601:
	add	x1, x4, :lo12:.LANCHOR0
	ldrh	w1, [x1,1960]
	cmp	w0, w1
	bcs	.L599
	add	x1, x19, :lo12:.LANCHOR2
	ubfiz	x2, x0, 1, 32
	add	w0, w0, 1
	ldr	x3, [x1,-80]
	ldrh	w1, [x3,x2]
	add	w1, w1, 1
	strh	w1, [x3,x2]
	b	.L601
.L599:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w22, [x0,220]
	ldr	w23, [x0,212]
	add	w1, w22, 256
	cmp	w1, w23
	bls	.L604
	ldr	w2, [x0,216]
	add	w1, w22, 768
	cmp	w1, w2
	bls	.L604
	ldr	w0, [x0,160]
	cbz	w0, .L607
	cmp	w22, 30
	bls	.L604
.L607:
	mov	w0, 65535
	b	.L606
.L604:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,-8]
	add	w0, w0, w0, lsl 1
	lsr	w0, w0, 2
	bl	GetFreeBlockMaxEraseCount
	uxth	w6, w0
	add	w0, w22, 64
	cmp	w6, w0
	bcs	.L618
	cmp	w22, 30
	bhi	.L607
.L618:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x3, [x0,-48]
	cbz	x3, .L607
	mov	w24, 65535
	ldr	x5, [x0,-56]
	ldr	x26, [x0,-80]
	mov	x0, -6148914691236517206
	mov	w20, w24
	mov	w7, w24
	add	x0, x0, 1
	mov	w8, 6
.L609:
	ldrh	w1, [x3]
	cmp	w1, w7
	beq	.L611
	ldrh	w2, [x3,4]
	cbz	w2, .L610
	sub	x4, x3, x5
	asr	x4, x4, 1
	mul	x4, x4, x0
	uxth	w3, w4
	and	x4, x4, 65535
	ldrh	w2, [x26,x4,lsl 1]
	cmp	w2, w22
	bls	.L617
	cmp	w2, w24
	bcs	.L610
	mov	w24, w2
	mov	w20, w3
.L610:
	umull	x3, w1, w8
	add	x3, x5, x3
	b	.L609
.L617:
	mov	w20, w3
.L611:
	mov	w0, 65535
	cmp	w20, w0
	beq	.L607
	ubfiz	x25, x20, 1, 16
	ldrh	w21, [x26,x25]
	cmp	w21, w22
	bls	.L613
	str	x6, [x29,88]
	bl	GetFreeBlockMinEraseCount
	ldr	x6, [x29,88]
	cmp	w22, w0, uxth
	bcs	.L613
	add	x0, x19, :lo12:.LANCHOR2
	str	w24, [x0,220]
.L613:
	cmp	w21, 29
	bhi	.L614
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,160]
	cbz	w0, .L614
	add	w0, w21, 10
	cmp	w0, w23
	bls	.L615
	adrp	x0, .LANCHOR0+2016
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2016]
	cmp	w0, w21
	bls	.L614
.L615:
	add	x22, x19, :lo12:.LANCHOR2
	ldrh	w0, [x22,-24]
	cmp	w0, 64
	bls	.L614
	adrp	x0, .LC74
	ldrh	w2, [x26,x25]
	add	x0, x0, :lo12:.LC74
	mov	w1, w20
	bl	printk
	mov	w0, 1
	str	w0, [x22,876]
	b	.L637
.L614:
	cmp	w21, w23
	bcs	.L607
	add	w0, w21, 128
	cmp	w6, w0
	ble	.L607
	add	w0, w21, 256
	cmp	w0, w23
	bcc	.L616
	add	x0, x19, :lo12:.LANCHOR2
	add	w21, w21, 768
	ldr	w0, [x0,216]
	cmp	w21, w0
	bcs	.L607
.L616:
	add	x19, x19, :lo12:.LANCHOR2
	adrp	x0, .LC75
	ldrh	w5, [x26,x25]
	add	x0, x0, :lo12:.LC75
	mov	w1, w20
	mov	w2, w23
	ldr	x4, [x19,-40]
	ldr	w3, [x19,216]
	ldrh	w4, [x4,x25]
	bl	printk
	mov	w0, 1
	str	w0, [x19,876]
.L637:
	mov	w0, w20
.L606:
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
	beq	.L640
	adrp	x2, .LANCHOR2-40
	ubfiz	x1, x0, 1, 16
	ldr	x2, [x2,#:lo12:.LANCHOR2-40]
	strh	wzr, [x2,x1]
	bl	INSERT_FREE_LIST
.L640:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	free_data_superblock, .-free_data_superblock
	.align	2
	.global	FtlGcBufInit
	.type	FtlGcBufInit, %function
FtlGcBufInit:
	adrp	x3, .LANCHOR2
	mov	w1, 0
	add	x0, x3, :lo12:.LANCHOR2
	adrp	x10, .LANCHOR0
	mov	w11, 24
	mov	w12, 1
	mov	w6, 4
	mov	w13, 56
	str	wzr, [x0,880]
.L642:
	add	x5, x10, :lo12:.LANCHOR0
	adrp	x9, .LANCHOR0
	ldrh	w0, [x5,1952]
	cmp	w1, w0
	bcs	.L646
	add	x2, x3, :lo12:.LANCHOR2
	umull	x4, w1, w11
	ldr	x7, [x2,888]
	ldr	x8, [x2,896]
	add	x0, x7, x4
	str	w12, [x0,16]
	ldrh	w0, [x5,2036]
	mul	w0, w1, w0
	sdiv	w0, w0, w6
	add	x0, x8, x0, sxtw 2
	str	x0, [x7,x4]
	ldrh	w0, [x5,2038]
	ldr	x8, [x2,888]
	ldr	x5, [x2,904]
	mul	w0, w1, w0
	add	x7, x8, x4
	ldr	x2, [x2,912]
	sdiv	w0, w0, w6
	add	x0, x5, x0, sxtw 2
	str	x0, [x7,8]
	umull	x0, w1, w13
	add	w1, w1, 1
	add	x0, x2, x0
	ldr	x2, [x8,x4]
	uxth	w1, w1
	str	x2, [x0,8]
	ldr	x2, [x7,8]
	str	x2, [x0,16]
	b	.L642
.L646:
	mov	w8, 24
	mov	w5, 4
.L644:
	add	x2, x3, :lo12:.LANCHOR2
	ldr	w1, [x2,920]
	cmp	w0, w1
	bcs	.L647
	umull	x4, w0, w8
	ldr	x7, [x2,888]
	add	x6, x9, :lo12:.LANCHOR0
	ldr	x10, [x2,896]
	add	x1, x7, x4
	str	wzr, [x1,16]
	ldrh	w1, [x6,2036]
	mul	w1, w0, w1
	sdiv	w1, w1, w5
	add	x1, x10, x1, sxtw 2
	str	x1, [x7,x4]
	ldrh	w1, [x6,2038]
	ldr	x6, [x2,888]
	ldr	x2, [x2,904]
	mul	w1, w0, w1
	add	x4, x6, x4
	add	w0, w0, 1
	sdiv	w1, w1, w5
	uxth	w0, w0
	add	x1, x2, x1, sxtw 2
	str	x1, [x4,8]
	b	.L644
.L647:
	ret
	.size	FtlGcBufInit, .-FtlGcBufInit
	.align	2
	.global	FtlGcBufFree
	.type	FtlGcBufFree, %function
FtlGcBufFree:
	adrp	x2, .LANCHOR2
	mov	w3, 0
	add	x2, x2, :lo12:.LANCHOR2
	mov	w7, 56
	mov	w9, 24
	ldr	w8, [x2,920]
	ldr	x5, [x2,888]
.L649:
	cmp	w3, w1
	bcs	.L648
	umull	x4, w3, w7
	mov	w2, 0
	add	x4, x0, x4
.L654:
	cmp	w2, w8
	bcs	.L651
	umull	x6, w2, w9
	add	x10, x5, x6
	ldr	x11, [x5,x6]
	ldr	x6, [x4,8]
	cmp	x11, x6
	bne	.L650
	str	wzr, [x10,16]
	b	.L651
.L650:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L654
.L651:
	add	w3, w3, 1
	uxth	w3, w3
	b	.L649
.L648:
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
	ldr	w5, [x2,920]
	ldr	x6, [x2,888]
	mov	w2, 0
.L656:
	cmp	w2, w1
	bcs	.L662
	mov	w3, 0
.L660:
	cmp	w3, w5
	bcs	.L658
	umull	x4, w3, w7
	add	x4, x6, x4
	ldr	w10, [x4,16]
	cbnz	w10, .L657
	umull	x3, w2, w9
	str	w8, [x4,16]
	add	x3, x0, x3
	ldr	x10, [x4]
	str	x10, [x3,8]
	ldr	x4, [x4,8]
	str	x4, [x3,16]
	b	.L658
.L657:
	add	w3, w3, 1
	uxth	w3, w3
	b	.L660
.L658:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L656
.L662:
	ret
	.size	FtlGcBufAlloc, .-FtlGcBufAlloc
	.align	2
	.global	IsBlkInGcList
	.type	IsBlkInGcList, %function
IsBlkInGcList:
	adrp	x1, .LANCHOR2
	uxth	w0, w0
	add	x1, x1, :lo12:.LANCHOR2
	ldrh	w2, [x1,924]
	ldr	x3, [x1,928]
	mov	x1, 0
.L664:
	cmp	w2, w1, uxth
	bls	.L668
	add	x1, x1, 1
	add	x4, x3, x1, lsl 1
	ldrh	w4, [x4,-2]
	cmp	w4, w0
	bne	.L664
	mov	w0, 1
	b	.L665
.L668:
	mov	w0, 0
.L665:
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
	adrp	x3, .LANCHOR2
	ldr	x2, [x29,40]
	add	x4, x3, :lo12:.LANCHOR2
	ldr	x1, [x29,32]
	uxth	w8, w0
	mov	x5, 0
	ldrh	w7, [x4,924]
	ldr	x6, [x4,928]
.L670:
	uxth	w4, w5
	cmp	w4, w7
	bcs	.L674
	add	x5, x5, 1
	add	x9, x6, x5, lsl 1
	ldrh	w9, [x9,-2]
	cmp	w9, w8
	bne	.L670
.L674:
	cmp	w4, w7
	bne	.L672
	ubfiz	x4, x4, 1, 16
	strh	w0, [x6,x4]
	add	x0, x3, :lo12:.LANCHOR2
	ldrh	w4, [x0,924]
	add	w4, w4, 1
	strh	w4, [x0,924]
.L672:
	add	x3, x3, :lo12:.LANCHOR2
	mov	w0, 12
	ldrh	w4, [x3,936]
	ldr	x5, [x3,944]
	umull	x4, w4, w0
	add	x5, x5, x4
	str	w1, [x5,4]
	ldr	x1, [x3,944]
	add	x0, x1, x4
	str	w2, [x0,8]
	str	w19, [x1,x4]
	ldrh	w0, [x3,936]
	add	w0, w0, 1
	strh	w0, [x3,936]
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlGcUpdatePage, .-FtlGcUpdatePage
	.align	2
	.global	FtlGcRefreshBlock
	.type	FtlGcRefreshBlock, %function
FtlGcRefreshBlock:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x19, [sp,16]
	uxth	w19, w0
	adrp	x0, .LC76
	mov	w1, w19
	add	x0, x0, :lo12:.LC76
	bl	printk
	adrp	x2, .LANCHOR2
	add	x2, x2, :lo12:.LANCHOR2
	ldrh	w3, [x2,952]
	cmp	w3, w19
	beq	.L677
	ldrh	w1, [x2,954]
	cmp	w1, w19
	beq	.L677
	mov	w0, 65535
	cmp	w3, w0
	bne	.L678
	strh	w19, [x2,952]
	b	.L677
.L678:
	cmp	w1, w0
	bne	.L677
	strh	w19, [x2,954]
.L677:
	mov	w0, 0
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
	ldrh	w1, [x22,956]
	adrp	x0, .LC77
	mov	w2, w20
	add	x0, x0, :lo12:.LC77
	bl	printk
	mov	w0, w21
	bl	FtlGcRefreshBlock
	ldr	w0, [x22,160]
	cbz	w0, .L680
	ubfiz	x0, x21, 1, 16
	ldr	x2, [x22,-80]
	ldrh	w1, [x2,x0]
	cmp	w1, 29
	bls	.L680
	sub	w1, w1, #30
	strh	w1, [x2,x0]
.L680:
	add	x2, x19, :lo12:.LANCHOR2
	mov	x0, 0
	add	x2, x2, 960
	ldrh	w1, [x2,-4]
.L681:
	cmp	w1, w0, uxth
	bls	.L687
	add	x0, x0, 1
	add	x3, x2, x0, lsl 1
	ldrh	w3, [x3,-2]
	cmp	w3, w20
	bne	.L681
	b	.L682
.L687:
	cmp	w1, 15
	bhi	.L682
	add	x19, x19, :lo12:.LANCHOR2
	add	w0, w1, 1
	add	x19, x19, 960
	strh	w0, [x19,-4]
	strh	w20, [x19,w1,sxtw 1]
.L682:
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
	ldrh	w1, [x0,956]
	cbz	w1, .L689
	ldrh	w3, [x0,952]
	mov	w2, 65535
	cmp	w3, w2
	bne	.L689
	ldrh	w2, [x0,994]
	cmp	w2, w1
	bcc	.L690
	strh	wzr, [x0,994]
.L690:
	add	x19, x19, :lo12:.LANCHOR2
	add	x0, x19, 960
	ldrh	w1, [x19,994]
	ldrh	w0, [x0,w1,sxtw 1]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	w0, [x19,994]
	add	w0, w0, 1
	strh	w0, [x19,994]
.L689:
	mov	w0, 0
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcReFreshBadBlk, .-FtlGcReFreshBadBlk
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
	.global	BuildFlashLsbPageTable
	.type	BuildFlashLsbPageTable, %function
BuildFlashLsbPageTable:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	w20, w1
	cbnz	w0, .L696
	adrp	x1, .LANCHOR0
	mov	x0, 0
	add	x1, x1, :lo12:.LANCHOR0
.L697:
	add	x2, x1, 196
	strh	w0, [x2,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L697
.L703:
	adrp	x19, .LANCHOR2
	mov	w1, 255
	add	x19, x19, :lo12:.LANCHOR2
	mov	w2, 1024
	add	x19, x19, 996
	uxth	w20, w20
	mov	x0, x19
	bl	ftl_memset
	adrp	x1, .LANCHOR0
	mov	x0, 0
	add	x1, x1, :lo12:.LANCHOR0
	b	.L698
.L696:
	cmp	w0, 1
	bne	.L699
	adrp	x3, .LANCHOR0
	mov	x0, 0
	mov	w4, 3
	mov	w5, 2
	add	x3, x3, :lo12:.LANCHOR0
.L702:
	cmp	x0, 3
	uxth	w1, w0
	mov	w2, w1
	bls	.L700
	ubfiz	w2, w1, 1, 15
	and	w1, w1, 1
	cmp	w1, wzr
	csel	w1, w4, w5, ne
	sub	w2, w2, w1
	uxth	w2, w2
.L700:
	add	x1, x3, 196
	strh	w2, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L702
	b	.L703
.L699:
	cmp	w0, 2
	bne	.L704
	adrp	x3, .LANCHOR0
	mov	w1, 65535
	mov	x0, 0
	add	x3, x3, :lo12:.LANCHOR0
.L706:
	add	x4, x3, 196
	cmp	x0, 1
	uxth	w2, w0
	csel	w2, w1, w2, hi
	add	w1, w1, 2
	strh	w2, [x4,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	uxth	w1, w1
	bne	.L706
	b	.L703
.L704:
	cmp	w0, 3
	bne	.L707
	adrp	x3, .LANCHOR0
	mov	x0, 0
	mov	w4, 5
	mov	w5, 4
	add	x3, x3, :lo12:.LANCHOR0
.L710:
	cmp	x0, 5
	uxth	w1, w0
	mov	w2, w1
	bls	.L708
	ubfiz	w2, w1, 1, 15
	and	w1, w1, 1
	cmp	w1, wzr
	csel	w1, w4, w5, ne
	sub	w2, w2, w1
	uxth	w2, w2
.L708:
	add	x1, x3, 196
	strh	w2, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L710
	b	.L703
.L707:
	cmp	w0, 4
	bne	.L711
	adrp	x1, .LANCHOR0
	mov	w5, 7
	add	x1, x1, :lo12:.LANCHOR0
	mov	w4, 6
	add	x2, x1, 196
	strh	w0, [x2,8]
	mov	w0, 5
	strh	wzr, [x1,196]
	mov	w1, 1
	strh	w0, [x2,10]
	mov	w0, 7
	strh	w1, [x2,2]
	mov	w1, 2
	strh	w0, [x2,12]
	mov	w0, 8
	strh	w1, [x2,4]
	mov	w1, 3
	strh	w0, [x2,14]
	mov	w0, 8
	strh	w1, [x2,6]
.L713:
	and	w3, w0, 1
	ubfiz	w1, w0, 1, 15
	add	w0, w0, 1
	cmp	w3, wzr
	csel	w3, w5, w4, ne
	add	x2, x2, 2
	uxth	w0, w0
	sub	w1, w1, w3
	cmp	w0, 256
	strh	w1, [x2,14]
	bne	.L713
	b	.L703
.L711:
	cmp	w0, 5
	bne	.L714
	adrp	x2, .LANCHOR0
	mov	x0, 0
	add	x2, x2, :lo12:.LANCHOR0
.L715:
	add	x1, x2, 196
	strh	w0, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 16
	bne	.L715
	mov	x0, 0
.L716:
	add	x2, x1, x0
	add	w3, w0, 16
	add	x0, x0, 2
	cmp	x0, 480
	strh	w3, [x2,32]
	bne	.L716
	b	.L703
.L714:
	cmp	w0, 6
	bne	.L703
	adrp	x1, .LANCHOR0
	mov	x2, 0
	mov	w4, 12
	mov	w5, 10
	add	x1, x1, :lo12:.LANCHOR0
.L719:
	cmp	x2, 5
	uxth	w3, w2
	mov	w0, w3
	bls	.L717
	add	w0, w3, w3, lsl 1
	and	w3, w3, 1
	cmp	w3, wzr
	uxth	w0, w0
	csel	w3, w4, w5, ne
	sub	w0, w0, w3
	uxth	w0, w0
.L717:
	add	x3, x1, 196
	strh	w0, [x3,x2,lsl 1]
	add	x2, x2, 1
	cmp	x2, 256
	bne	.L719
	b	.L703
.L698:
	cmp	w20, w0, uxth
	bls	.L737
	add	x2, x1, 196
	ldrh	w2, [x2,x0,lsl 1]
	add	x0, x0, 1
	strh	w2, [x19,w2,sxtw 1]
	b	.L698
.L737:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	BuildFlashLsbPageTable, .-BuildFlashLsbPageTable
	.align	2
	.global	FlashDieInfoInit
	.type	FlashDieInfoInit, %function
FlashDieInfoInit:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR2
	str	x27, [sp,80]
	add	x1, x22, :lo12:.LANCHOR2
	adrp	x21, .LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	strb	wzr, [x1,2020]
	adrp	x1, .LANCHOR1+482
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w1, [x1,#:lo12:.LANCHOR1+482]
	strb	wzr, [x0,1845]
	cmp	w1, 256
	bls	.L739
	mov	w1, 512
	b	.L755
.L739:
	cmp	w1, 128
	bls	.L755
	mov	w1, 256
.L755:
	add	x19, x21, :lo12:.LANCHOR0
	str	w1, [x0,12]
	mov	w2, 8
	mov	w1, 0
	add	x0, x19, 1848
	add	x25, x19, 708
	mov	x24, 0
	add	x26, x19, 1620
	bl	ftl_memset
	mov	w1, 0
	mov	w2, 32
	mov	x0, x25
	bl	ftl_memset
	add	x0, x19, 1652
	mov	w1, 0
	mov	w2, 192
	bl	ftl_memset
	ldr	x20, [x19,744]
	add	x23, x20, 1
.L743:
	ldrb	w2, [x20]
	mov	x0, x23
	add	x1, x26, x24, lsl 3
	bl	FlashMemCmp8
	cbnz	w0, .L742
	ldrb	w1, [x19,1845]
	str	w0, [x25,w1,sxtw 2]
	add	w0, w1, 1
	add	x1, x19, x1, sxtw
	strb	w0, [x19,1845]
	strb	w24, [x1,1848]
.L742:
	add	x24, x24, 1
	cmp	x24, 4
	bne	.L743
	add	x19, x21, :lo12:.LANCHOR0
	add	x0, x22, :lo12:.LANCHOR2
	ldrb	w1, [x19,1845]
	strb	w1, [x0,2020]
	ldrb	w0, [x20,8]
	cmp	w0, 2
	beq	.L744
.L748:
	add	x21, x21, :lo12:.LANCHOR0
	ldrb	w1, [x20,13]
	add	x22, x22, :lo12:.LANCHOR2
	ldr	x27, [sp,80]
	ldp	x23, x24, [sp,48]
	ldrb	w0, [x21,1845]
	ldp	x25, x26, [sp,64]
	mul	w0, w0, w1
	ldrh	w1, [x20,14]
	ldp	x19, x20, [sp,16]
	mul	w0, w0, w1
	strh	w0, [x22,2022]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 96
	ret
.L744:
	ldr	w26, [x19,12]
	mov	x24, 0
	add	x27, x19, 1620
	add	x25, x19, 708
.L747:
	ldrb	w2, [x20]
	mov	x0, x23
	add	x1, x27, x24, lsl 3
	bl	FlashMemCmp8
	cbnz	w0, .L745
	ldrb	w1, [x20,13]
	ldrb	w3, [x19,1845]
	mul	w0, w1, w26
	ldrh	w1, [x20,14]
	mov	x2, x3
	and	w1, w1, 65280
	mul	w1, w0, w1
	str	w1, [x25,w3,sxtw 2]
	ldrb	w0, [x20,23]
	cbz	w0, .L746
	lsl	w1, w1, 1
	str	w1, [x25,w3,sxtw 2]
.L746:
	add	x0, x19, x3
	add	w2, w2, 1
	strb	w2, [x19,1845]
	strb	w24, [x0,1848]
.L745:
	add	x24, x24, 1
	cmp	x24, 4
	bne	.L747
	b	.L748
	.size	FlashDieInfoInit, .-FlashDieInfoInit
	.align	2
	.global	ReadFlashInfo
	.type	ReadFlashInfo, %function
ReadFlashInfo:
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
	ldr	x0, [x1,744]
	ldrb	w2, [x0,9]
	ldr	w0, [x1,12]
	mul	w0, w0, w2
	ldrb	w2, [x1,1944]
	strb	w2, [x19,7]
	uxth	w0, w0
	strh	w0, [x19,4]
	ldr	x3, [x1,744]
	ldrb	w2, [x3,13]
	mul	w0, w2, w0
	ldrh	w2, [x3,14]
	mul	w0, w0, w2
	ldrb	w2, [x3,8]
	mul	w0, w0, w2
	str	w0, [x19]
	ldr	x0, [x1,744]
	ldrb	w4, [x1,1845]
	ldrb	w0, [x0,9]
	strb	w0, [x19,6]
	mov	w0, 32
	strb	w0, [x19,8]
	ldr	x0, [x1,744]
	ldrb	w0, [x0,7]
	strb	w0, [x19,9]
	mov	x0, 0
	strb	wzr, [x19,10]
.L757:
	cmp	w4, w0, uxtb
	bls	.L759
	add	x2, x1, 1848
	ldrb	w3, [x19,10]
	ldrb	w2, [x0,x2]
	add	x0, x0, 1
	lsl	w2, w5, w2
	orr	w2, w2, w3
	strb	w2, [x19,10]
	b	.L757
.L759:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	ReadFlashInfo, .-ReadFlashInfo
	.align	2
	.global	FtlMemInit
	.type	FtlMemInit, %function
FtlMemInit:
	stp	x29, x30, [sp, -80]!
	mov	w1, 65535
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	adrp	x22, .LANCHOR0
	add	x0, x21, :lo12:.LANCHOR2
	add	x3, x22, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	str	x25, [sp,64]
	str	w1, [x0,2032]
	mov	w1, -1
	str	wzr, [x0,2024]
	strh	w1, [x0,952]
	strh	w1, [x0,954]
	mov	w1, 32
	str	wzr, [x0,344]
	str	wzr, [x0,196]
	str	wzr, [x0,200]
	str	wzr, [x0,184]
	str	wzr, [x0,172]
	str	wzr, [x0,168]
	str	wzr, [x0,176]
	str	wzr, [x0,180]
	str	wzr, [x0,164]
	str	wzr, [x0,204]
	str	wzr, [x0,208]
	str	wzr, [x0,-72]
	str	wzr, [x0,216]
	str	wzr, [x0,220]
	str	wzr, [x0,2028]
	str	wzr, [x0,876]
	str	wzr, [x0,2036]
	str	wzr, [x0,868]
	str	wzr, [x3,2108]
	strh	w1, [x0,864]
	mov	w1, 128
	ldrh	w2, [x3,2032]
	strh	w1, [x0,866]
	mov	w1, 1024
	strh	wzr, [x0,872]
	sdiv	w2, w1, w2
	ldrh	w1, [x3,1952]
	strh	wzr, [x0,956]
	lsl	w1, w1, 2
	strh	wzr, [x0,994]
	str	w2, [x0,2040]
	cmp	w2, w1
	bls	.L761
	str	w1, [x0,2040]
.L761:
	add	x20, x22, :lo12:.LANCHOR0
	add	x19, x21, :lo12:.LANCHOR2
	mov	w24, 56
	ldrh	w0, [x20,2030]
	str	wzr, [x20,2088]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x19,928]
	ldrh	w1, [x20,2030]
	mov	w0, 12
	mul	w0, w1, w0
	bl	ftl_malloc
	ldrh	w23, [x20,1952]
	str	x0, [x19,944]
	mul	w23, w23, w24
	lsl	w25, w23, 3
	mov	w0, w25
	bl	ftl_malloc
	str	x0, [x19,2048]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2056]
	mov	w0, w25
	bl	ftl_malloc
	str	x0, [x19,2064]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,-88]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,912]
	ldr	w0, [x19,2040]
	mul	w0, w0, w24
	bl	ftl_malloc
	str	x0, [x20,2096]
	ldrh	w0, [x20,1952]
	ldrh	w23, [x20,2036]
	lsl	w0, w0, 1
	add	w0, w0, 1
	str	w0, [x19,920]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,-64]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2072]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2080]
	ldr	w0, [x19,920]
	mul	w0, w23, w0
	bl	ftl_malloc
	str	x0, [x19,896]
	ldr	w0, [x19,2040]
	mul	w0, w23, w0
	bl	ftl_malloc
	str	x0, [x19,2088]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2096]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2104]
	ldr	w1, [x19,920]
	mov	w0, 24
	mul	w0, w1, w0
	bl	ftl_malloc
	ldrh	w23, [x20,1952]
	str	x0, [x19,888]
	ldrh	w0, [x20,2038]
	mul	w23, w0, w23
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2112]
	lsl	w0, w23, 3
	bl	ftl_malloc
	str	x0, [x19,2120]
	ldrh	w1, [x20,2038]
	ldr	w0, [x19,920]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,904]
	ldrh	w1, [x20,2038]
	ldr	w0, [x19,2040]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,2128]
	ldrh	w0, [x20,1962]
	lsl	w0, w0, 1
	strh	w0, [x19,2136]
	and	w0, w0, 65534
	bl	ftl_malloc
	str	x0, [x19,2144]
	ldrh	w0, [x19,2136]
	add	x0, x0, 547
	lsr	x0, x0, 9
	strh	w0, [x19,2136]
	lsl	w0, w0, 9
	bl	ftl_malloc
	ldrh	w23, [x20,1962]
	str	x0, [x19,2152]
	add	x0, x0, 32
	lsl	w23, w23, 1
	str	x0, [x19,-80]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2160]
	mov	w0, w23
	bl	ftl_malloc
	ldr	w23, [x20,2056]
	str	x0, [x19,-40]
	lsl	w23, w23, 1
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2168]
	mov	w0, w23
	bl	ftl_malloc
	str	x0, [x19,2176]
	ldrh	w0, [x20,1962]
	lsr	w0, w0, 3
	add	w0, w0, 4
	bl	ftl_malloc
	str	x0, [x22,#:lo12:.LANCHOR0]
	ldrh	w0, [x20,2048]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x20,2080]
	ldrh	w0, [x20,2048]
	lsl	w0, w0, 1
	bl	ftl_malloc
	str	x0, [x19,2184]
	ldrh	w0, [x20,2048]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,2192]
	ldrh	w0, [x20,2050]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,2200]
	ldrh	w2, [x20,2050]
	mov	w1, 0
	lsl	w2, w2, 2
	bl	ftl_memset
	ldrh	w0, [x20,2064]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,2208]
	ldr	w0, [x20,2056]
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x19,2216]
	ldrh	w0, [x20,2066]
	lsl	w0, w0, 4
	bl	ftl_malloc
	str	x0, [x19,144]
	ldrh	w1, [x20,2066]
	ldrh	w0, [x20,2036]
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,2224]
	ldrh	w1, [x20,1962]
	mov	w0, 6
	mul	w0, w1, w0
	bl	ftl_malloc
	str	x0, [x19,-56]
	ldrh	w0, [x20,2022]
	ldrh	w1, [x20,1974]
	add	w0, w0, 31
	asr	w0, w0, 5
	strh	w0, [x19,2232]
	mul	w0, w0, w1
	lsl	w0, w0, 2
	bl	ftl_malloc
	str	x0, [x20,2144]
	add	x1, x20, 2112
	ldrh	w4, [x19,2232]
	ldrh	w5, [x20,1974]
	mov	w0, 1
	mov	w2, w4
.L762:
	cmp	w0, w5
	add	x1, x1, 8
	bcs	.L862
	ldr	x3, [x20,2144]
	add	w0, w0, 1
	add	x3, x3, x2, uxtw 2
	add	w2, w2, w4
	str	x3, [x1,32]
	b	.L762
.L862:
	add	x1, x22, :lo12:.LANCHOR0
.L764:
	cmp	w0, 8
	beq	.L863
	add	x2, x1, x0, uxtw 3
	add	w0, w0, 1
	str	xzr, [x2,2144]
	b	.L764
.L863:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x1, [x0,2168]
	cbnz	x1, .L766
.L768:
	adrp	x0, .LC78
	adrp	x1, .LANCHOR3
	add	x0, x0, :lo12:.LC78
	add	x1, x1, :lo12:.LANCHOR3
	bl	printk
	mov	w0, -1
	b	.L767
.L766:
	ldr	x1, [x0,2176]
	cbz	x1, .L768
	ldr	x1, [x0,2208]
	cbz	x1, .L768
	ldr	x1, [x0,2216]
	cbz	x1, .L768
	ldr	x1, [x0,144]
	cbz	x1, .L768
	ldr	x1, [x0,2224]
	cbz	x1, .L768
	ldr	x1, [x0,-56]
	cbz	x1, .L768
	add	x22, x22, :lo12:.LANCHOR0
	ldr	x1, [x22,2144]
	cbz	x1, .L768
	ldr	x0, [x0,-40]
	cbz	x0, .L768
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x1, [x0,928]
	cbz	x1, .L768
	ldr	x1, [x0,944]
	cbz	x1, .L768
	ldr	x1, [x0,2048]
	cbz	x1, .L768
	ldr	x1, [x0,2064]
	cbz	x1, .L768
	ldr	x1, [x0,-88]
	cbz	x1, .L768
	ldr	x1, [x0,912]
	cbz	x1, .L768
	ldr	x1, [x0,2056]
	cbz	x1, .L768
	ldr	x1, [x0,-64]
	cbz	x1, .L768
	ldr	x1, [x0,2072]
	cbz	x1, .L768
	ldr	x0, [x0,2080]
	cbz	x0, .L768
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x1, [x0,896]
	cbz	x1, .L768
	ldr	x1, [x0,2096]
	cbz	x1, .L768
	ldr	x1, [x0,2104]
	cbz	x1, .L768
	ldr	x1, [x0,888]
	cbz	x1, .L768
	ldr	x1, [x0,2112]
	cbz	x1, .L768
	ldr	x1, [x0,2120]
	cbz	x1, .L768
	ldr	x1, [x0,904]
	cbz	x1, .L768
	ldr	x1, [x0,-80]
	cbz	x1, .L768
	ldr	x0, [x0,2144]
	cbz	x0, .L768
	adrp	x0, .LANCHOR0+2080
	ldr	x0, [x0,#:lo12:.LANCHOR0+2080]
	cbz	x0, .L768
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldr	x1, [x0,2184]
	cbz	x1, .L768
	ldr	x1, [x0,2192]
	cbz	x1, .L768
	ldr	x0, [x0,2200]
	cbz	x0, .L768
	mov	w0, 0
.L767:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlMemInit, .-FtlMemInit
	.align	2
	.global	FtlBbt2Bitmap
	.type	FtlBbt2Bitmap, %function
FtlBbt2Bitmap:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	x20, x0
	adrp	x0, .LANCHOR2+2232
	mov	x19, x1
	ldrh	w2, [x0,#:lo12:.LANCHOR2+2232]
	mov	x0, x1
	mov	w1, 0
	lsl	w2, w2, 2
	bl	ftl_memset
	mov	x3, 0
	mov	w0, 65535
	mov	w5, 1
.L866:
	ldrh	w2, [x20,x3]
	cmp	w2, w0
	beq	.L864
	ubfx	x4, x2, 5, 11
	add	x3, x3, 2
	lsl	x4, x4, 2
	lsl	w2, w5, w2
	cmp	x3, 1024
	ldr	w1, [x19,x4]
	orr	w2, w1, w2
	str	w2, [x19,x4]
	bne	.L866
.L864:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlBbt2Bitmap, .-FtlBbt2Bitmap
	.align	2
	.global	FtlBbtMemInit
	.type	FtlBbtMemInit, %function
FtlBbtMemInit:
	adrp	x0, .LANCHOR0
	mov	w1, -1
	stp	x29, x30, [sp, -16]!
	add	x0, x0, :lo12:.LANCHOR0
	mov	w2, 16
	add	x29, sp, 0
	add	x0, x0, 2124
	strh	w1, [x0,-12]
	mov	w1, 255
	strh	wzr, [x0,-6]
	bl	ftl_memset
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlBbtMemInit, .-FtlBbtMemInit
	.align	2
	.global	FtlFreeSysBlkQueueInit
	.type	FtlFreeSysBlkQueueInit, %function
FtlFreeSysBlkQueueInit:
	adrp	x1, .LANCHOR0
	mov	w2, 2048
	stp	x29, x30, [sp, -16]!
	add	x1, x1, :lo12:.LANCHOR0
	add	x29, sp, 0
	strh	w0, [x1,2208]
	add	x0, x1, 2216
	strh	wzr, [x1,2210]
	strh	wzr, [x1,2212]
	strh	wzr, [x1,2214]
	mov	w1, 0
	bl	ftl_memset
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlFreeSysBlkQueueInit, .-FtlFreeSysBlkQueueInit
	.align	2
	.global	ftl_free_no_use_map_blk
	.type	ftl_free_no_use_map_blk, %function
ftl_free_no_use_map_blk:
	stp	x29, x30, [sp, -96]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	str	x27, [sp,80]
	mov	x19, x0
	ldrh	w2, [x0,10]
	ldr	x21, [x0,32]
	ldr	x25, [x0,16]
	lsl	w2, w2, 1
	ldr	x20, [x0,40]
	mov	x0, x21
	bl	ftl_memset
	mov	w0, 0
.L872:
	ldrh	w1, [x19,6]
	cmp	w1, w0
	bls	.L885
	ubfiz	x1, x0, 2, 16
	ldr	w2, [x20,x1]
	mov	w1, 0
	ubfx	x2, x2, 10, 16
.L873:
	ldrh	w3, [x19,10]
	cmp	w3, w1
	bls	.L886
	ubfiz	x3, x1, 1, 16
	ldrh	w4, [x25,x3]
	cmp	w4, w2
	bne	.L874
	ldrh	w4, [x21,x3]
	add	w4, w4, 1
	strh	w4, [x21,x3]
.L874:
	add	w1, w1, 1
	uxth	w1, w1
	b	.L873
.L886:
	add	w0, w0, 1
	uxth	w0, w0
	b	.L872
.L885:
	mov	w24, 0
	ldrh	w26, [x21]
	mov	w20, w24
	adrp	x27, .LANCHOR0
.L877:
	ldrh	w0, [x19,10]
	cmp	w0, w20
	bls	.L887
	ldrh	w0, [x19]
	uxtw	x22, w20
	cmp	w0, w20
	bne	.L878
	add	x0, x27, :lo12:.LANCHOR0
	ldrh	w1, [x19,2]
	ldrh	w0, [x0,2028]
	cmp	w1, w0
	bcs	.L878
	strh	w0, [x21,x22,lsl 1]
.L878:
	lsl	x22, x22, 1
	ldrh	w23, [x21,x22]
	cmp	w26, w23
	bls	.L879
	mov	w24, w20
	mov	w26, w23
.L879:
	cbnz	w23, .L880
	ldrh	w0, [x25,x22]
	cbz	w0, .L880
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
	strh	w23, [x25,x22]
	ldrh	w0, [x19,8]
	sub	w0, w0, #1
	strh	w0, [x19,8]
.L880:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L877
.L887:
	mov	w0, w24
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	ftl_free_no_use_map_blk, .-ftl_free_no_use_map_blk
	.align	2
	.global	FtlL2PDataInit
	.type	FtlL2PDataInit, %function
FtlL2PDataInit:
	stp	x29, x30, [sp, -48]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	adrp	x19, .LANCHOR2
	stp	x21, x22, [sp,32]
	add	x21, x20, :lo12:.LANCHOR0
	add	x22, x19, :lo12:.LANCHOR2
	ldr	w2, [x21,2056]
	ldr	x0, [x22,2176]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w3, [x21,2036]
	mov	w1, 255
	ldrh	w2, [x21,2066]
	ldr	x0, [x22,2224]
	mul	w2, w3, w2
	bl	ftl_memset
	mov	w1, 0
	mov	w5, -1
.L889:
	add	x3, x20, :lo12:.LANCHOR0
	ldrh	w0, [x3,2066]
	cmp	w0, w1
	bls	.L891
	add	x4, x19, :lo12:.LANCHOR2
	ubfiz	x0, x1, 4, 16
	ldr	x2, [x4,144]
	add	x2, x2, x0
	str	wzr, [x2,4]
	ldr	x2, [x4,144]
	strh	w5, [x2,x0]
	ldr	x2, [x4,144]
	add	x2, x2, x0
	ldrh	w0, [x3,2036]
	ldr	x3, [x4,2224]
	mul	w0, w0, w1
	add	w1, w1, 1
	sxtw	x0, w0
	uxth	w1, w1
	and	x0, x0, -4
	add	x0, x3, x0
	str	x0, [x2,8]
	b	.L889
.L891:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, -1
	add	x1, x0, 2240
	strh	w2, [x0,2242]
	strh	w2, [x0,2240]
	ldr	w2, [x3,2056]
	strh	w2, [x0,2250]
	mov	w2, -3902
	strh	w2, [x0,2244]
	ldrh	w2, [x0,2304]
	strh	w2, [x0,2248]
	ldrh	w2, [x3,2064]
	strh	w2, [x0,2246]
	ldr	x2, [x0,2168]
	str	x2, [x0,2256]
	ldr	x2, [x0,2216]
	str	x2, [x0,2264]
	ldr	x2, [x0,2176]
	str	x2, [x0,2272]
	ldr	x0, [x0,2208]
	str	x0, [x1,40]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlL2PDataInit, .-FtlL2PDataInit
	.align	2
	.global	FtlVariablesInit
	.type	FtlVariablesInit, %function
FtlVariablesInit:
	stp	x29, x30, [sp, -32]!
	mov	w0, -1
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	adrp	x20, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR2
	add	x20, x20, :lo12:.LANCHOR0
	strh	w0, [x19,2320]
	mov	w0, -1
	ldrh	w2, [x20,2048]
	str	w0, [x19,2332]
	ldr	x0, [x20,2080]
	lsl	w2, w2, 1
	str	xzr, [x19,2312]
	str	wzr, [x19,2324]
	str	wzr, [x19,2328]
	str	wzr, [x19,160]
	strh	wzr, [x20,2072]
	bl	ftl_memset
	ldrh	w2, [x20,1962]
	mov	w1, 0
	ldr	x0, [x19,-80]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w2, [x20,1962]
	mov	w1, 0
	ldr	x0, [x19,2144]
	lsl	w2, w2, 1
	bl	ftl_memset
	add	x0, x19, 224
	mov	w1, 0
	mov	w2, 48
	bl	ftl_memset
	mov	w1, 0
	mov	w2, 512
	add	x0, x19, 352
	bl	ftl_memset
	bl	FtlGcBufInit
	bl	FtlL2PDataInit
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlVariablesInit, .-FtlVariablesInit
	.align	2
	.global	SupperBlkListInit
	.type	SupperBlkListInit, %function
SupperBlkListInit:
	stp	x29, x30, [sp, -112]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	adrp	x24, .LANCHOR0
	add	x0, x24, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	adrp	x22, .LANCHOR2
	mov	w26, 6
	add	x20, x22, :lo12:.LANCHOR2
	ldrh	w2, [x0,1962]
	mov	w25, 0
	mov	x21, x20
	mov	w23, w25
	mov	w19, w25
	ldr	x0, [x20,-56]
	mul	w2, w2, w26
	bl	ftl_memset
	str	xzr, [x20,-16]
	str	xzr, [x20,-48]
	str	xzr, [x20,-32]
	strh	wzr, [x20,-24]
	strh	wzr, [x20,-8]
.L894:
	add	x0, x24, :lo12:.LANCHOR0
	ldrh	w1, [x0,1960]
	cmp	w1, w19
	bls	.L900
	mov	x28, 0
	ldrh	w3, [x0,1952]
	ldrh	w2, [x0,2026]
	mov	w20, w28
	mov	x27, x0
.L901:
	cmp	w3, w28, uxth
	bls	.L905
	add	x0, x27, 1984
	mov	w1, w19
	str	x2, [x29,96]
	str	x3, [x29,104]
	ldrb	w0, [x0,x28]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	ldr	x3, [x29,104]
	ldr	x2, [x29,96]
	cbnz	w0, .L895
	add	w20, w20, w2
	uxth	w20, w20
.L895:
	add	x28, x28, 1
	b	.L901
.L905:
	cbz	w20, .L897
	mov	w0, 32768
	sdiv	w20, w0, w20
.L897:
	ldr	x1, [x21,-56]
	umull	x0, w19, w26
	add	x0, x1, x0
	strh	w20, [x0,4]
	ldrh	w0, [x21]
	cmp	w0, w19
	beq	.L898
	ldrh	w0, [x21,48]
	cmp	w0, w19
	beq	.L898
	ldrh	w0, [x21,96]
	cmp	w0, w19
	beq	.L898
	ubfiz	x0, x19, 1, 16
	ldr	x1, [x21,-40]
	ldrh	w0, [x1,x0]
	cbnz	w0, .L899
	add	w4, w25, 1
	mov	w0, w19
	uxth	w25, w4
	bl	INSERT_FREE_LIST
	b	.L898
.L899:
	add	w23, w23, 1
	mov	w0, w19
	uxth	w23, w23
	bl	INSERT_DATA_LIST
.L898:
	add	w19, w19, 1
	uxth	w19, w19
	b	.L894
.L900:
	add	x22, x22, :lo12:.LANCHOR2
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x27, x28, [sp,80]
	strh	w23, [x22,-24]
	strh	w25, [x22,-8]
	ldp	x23, x24, [sp,48]
	ldp	x21, x22, [sp,32]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 112
	ret
	.size	SupperBlkListInit, .-SupperBlkListInit
	.align	2
	.global	FtlGcPageVarInit
	.type	FtlGcPageVarInit, %function
FtlGcPageVarInit:
	stp	x29, x30, [sp, -32]!
	mov	w1, 255
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	adrp	x19, .LANCHOR2
	add	x20, x20, :lo12:.LANCHOR0
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w2, [x20,2030]
	ldr	x0, [x19,928]
	lsl	w2, w2, 1
	strh	wzr, [x19,924]
	strh	wzr, [x19,936]
	bl	ftl_memset
	ldrh	w3, [x20,2030]
	mov	w2, 12
	ldr	x0, [x19,944]
	mov	w1, 255
	mul	w2, w3, w2
	bl	ftl_memset
	bl	FtlGcBufInit
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlGcPageVarInit, .-FtlGcPageVarInit
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
	.global	FlashGetReadRetryDefault
	.type	FlashGetReadRetryDefault, %function
FlashGetReadRetryDefault:
	stp	x29, x30, [sp, -16]!
	mov	w1, w0
	add	x29, sp, 0
	cbz	w0, .L908
	sub	w2, w0, #1
	cmp	w2, 5
	bhi	.L910
	bl	HynixGetReadRetryDefault
	b	.L908
.L910:
	cmp	w0, 49
	bne	.L911
	adrp	x0, .LANCHOR0
	mov	w2, 64
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, 764
	strb	w1, [x0,-4]
	mov	w1, 4
	strb	w1, [x0,-3]
	mov	w1, 15
	strb	w1, [x0,-2]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x1, x1, 408
	b	.L926
.L911:
	sub	w0, w0, #65
	cmp	w0, 1
	bls	.L917
	cmp	w1, 33
	bne	.L912
.L917:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	strb	w1, [x0,760]
	mov	w1, 4
	b	.L927
.L912:
	cmp	w1, 67
	beq	.L918
	cmp	w1, 34
	bne	.L914
.L918:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	strb	w1, [x0,760]
	mov	w1, 5
.L927:
	strb	w1, [x0,761]
	mov	w1, 7
	strb	w1, [x0,762]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x0, x0, 764
	add	x1, x1, 256
	mov	w2, 45
	b	.L926
.L914:
	cmp	w1, 68
	beq	.L919
	cmp	w1, 35
	bne	.L908
.L919:
	adrp	x0, .LANCHOR0
	mov	w2, 95
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, 764
	strb	w1, [x0,-4]
	mov	w1, 5
	strb	w1, [x0,-3]
	mov	w1, 17
	strb	w1, [x0,-2]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x1, x1, 304
.L926:
	bl	ftl_memcpy
.L908:
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashGetReadRetryDefault, .-FlashGetReadRetryDefault
	.align	2
	.global	FlashReadIdbData
	.type	FlashReadIdbData, %function
FlashReadIdbData:
	stp	x29, x30, [sp, -16]!
	adrp	x1, .LANCHOR2
	mov	w2, 2048
	add	x1, x1, :lo12:.LANCHOR2
	add	x29, sp, 0
	add	x1, x1, 2336
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
	stp	x21, x22, [sp,32]
	adrp	x20, .LANCHOR1
	adrp	x22, .LANCHOR0
	add	x21, x20, :lo12:.LANCHOR1
	add	x22, x22, :lo12:.LANCHOR0
	str	x23, [sp,48]
	add	x21, x21, 536
	mov	w19, 0
	add	x22, x22, 1620
.L933:
	ldrb	w2, [x21]
	add	x0, x21, 1
	mov	x1, x22
	bl	FlashMemCmp8
	mov	w23, w0
	cbnz	w0, .L930
	add	x0, x20, :lo12:.LANCHOR1
	ubfiz	x19, x19, 5, 32
	add	x1, x0, 536
	adds	x19, x1, x19
	beq	.L936
	ldrb	w3, [x19,22]
	mov	x2, 0
	mov	x1, x0
	b	.L935
.L930:
	add	w19, w19, 1
	add	x21, x21, 32
	cmp	w19, 72
	bne	.L933
	b	.L936
.L935:
	add	x4, x1, x2, lsl 5
	mov	w0, w2
	ldrb	w4, [x4,2840]
	cmp	w4, w3
	beq	.L934
	add	x2, x2, 1
	cmp	x2, 4
	bne	.L935
	mov	w0, w2
.L934:
	add	x20, x20, :lo12:.LANCHOR1
	ubfiz	x1, x0, 5, 32
	adrp	x0, .LANCHOR0
	add	x2, x20, 2840
	add	x0, x0, :lo12:.LANCHOR0
	add	x1, x2, x1
	add	x0, x0, 88
	mov	w2, 32
	bl	ftl_memcpy
	add	x0, x20, 472
	mov	x1, x19
	mov	w2, 32
	bl	ftl_memcpy
	b	.L931
.L936:
	mov	w23, -1
.L931:
	mov	w0, w23
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	FlashLoadPhyInfoInRam, .-FlashLoadPhyInfoInRam
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
	bne	.L941
	cbz	x3, .L942
	mov	x0, x2
	mov	x1, x3
	mov	w2, 1024
	bl	ftl_memcpy
.L942:
	cbz	x20, .L940
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
	b	.L940
.L941:
	cbz	x3, .L945
	mov	x1, x2
	mov	x0, x3
	mov	w2, 1024
	bl	ftl_memcpy
.L945:
	cbz	x20, .L940
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
.L940:
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	NandcCopy1KB, .-NandcCopy1KB
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
	.global	FlashSramLoadStore
	.type	FlashSramLoadStore, %function
FlashSramLoadStore:
	adrp	x4, .LANCHOR4+32
	mov	x6, x0
	stp	x29, x30, [sp, -16]!
	uxtw	x1, w1
	add	x29, sp, 0
	ldr	x4, [x4,#:lo12:.LANCHOR4+32]
	add	x4, x4, 4096
	cbnz	w2, .L958
	add	x1, x4, x1
	b	.L960
.L958:
	add	x0, x4, x1
	mov	x1, x6
.L960:
	mov	w2, w3
	bl	ftl_memcpy
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashSramLoadStore, .-FlashSramLoadStore
	.align	2
	.global	FlashCs123Init
	.type	FlashCs123Init, %function
FlashCs123Init:
	ret
	.size	FlashCs123Init, .-FlashCs123Init
	.align	2
	.global	rk_nand_de_init
	.type	rk_nand_de_init, %function
rk_nand_de_init:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FlashDeInit
	ldp	x29, x30, [sp], 16
	ret
	.size	rk_nand_de_init, .-rk_nand_de_init
	.align	2
	.global	rk_nand_suspend
	.type	rk_nand_suspend, %function
rk_nand_suspend:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FlashSuspend
	ldp	x29, x30, [sp], 16
	ret
	.size	rk_nand_suspend, .-rk_nand_suspend
	.align	2
	.global	rk_nand_resume
	.type	rk_nand_resume, %function
rk_nand_resume:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FlashResume
	ldp	x29, x30, [sp], 16
	ret
	.size	rk_nand_resume, .-rk_nand_resume
	.align	2
	.global	rk_ftl_get_capacity
	.type	rk_ftl_get_capacity, %function
rk_ftl_get_capacity:
	adrp	x0, .LANCHOR0+2068
	ldr	w0, [x0,#:lo12:.LANCHOR0+2068]
	ret
	.size	rk_ftl_get_capacity, .-rk_ftl_get_capacity
	.align	2
	.global	rknand_print_hex
	.type	rknand_print_hex, %function
rknand_print_hex:
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	x19, 0
	adrp	x23, .LC79
	adrp	x21, .LC80
	adrp	x24, .LC72
	stp	x25, x26, [sp,64]
	str	x27, [sp,80]
	mov	x22, x1
	mov	x27, x0
	mov	w25, w2
	uxtw	x26, w3
	mov	w20, w19
	add	x23, x23, :lo12:.LC79
	add	x21, x21, :lo12:.LC80
	add	x24, x24, :lo12:.LC72
.L967:
	cmp	x19, x26
	beq	.L975
	cbnz	w20, .L968
	mov	x0, x23
	mov	x1, x27
	mov	w2, w19
	bl	printk
.L968:
	cmp	w25, 4
	mov	x0, x21
	bne	.L969
	ldr	w1, [x22,x19,lsl 2]
	b	.L974
.L969:
	cmp	w25, 2
	bne	.L971
	ldrsh	w1, [x22,x19,lsl 1]
	b	.L974
.L971:
	ldrb	w1, [x22,x19]
.L974:
	bl	printk
	add	w20, w20, 1
	cmp	w20, 15
	bls	.L972
	adrp	x1, .LC81
	mov	x0, x24
	add	x1, x1, :lo12:.LC81
	mov	w20, 0
	bl	printk
.L972:
	add	x19, x19, 1
	b	.L967
.L975:
	adrp	x0, .LC72
	adrp	x1, .LC81
	add	x1, x1, :lo12:.LC81
	add	x0, x0, :lo12:.LC72
	bl	printk
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	rknand_print_hex, .-rknand_print_hex
	.align	2
	.global	NandcXferComp
	.type	NandcXferComp, %function
NandcXferComp:
	stp	x29, x30, [sp, -80]!
	ubfiz	x0, x0, 4, 8
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	str	x23, [sp,48]
	add	x1, x20, :lo12:.LANCHOR0
	add	x2, x1, 24
	stp	x21, x22, [sp,32]
	ldr	x19, [x2,x0]
	ldr	w0, [x1,1872]
	cmp	w0, 3
	bls	.L1007
	ldr	w0, [x19,16]
	tbz	x0, 2, .L1007
	mov	x0, x19
	bl	wait_for_nandc_xfer_completed
	ldr	w21, [x19,16]
	ldr	w0, [x19,8]
	ubfx	x21, x21, 1, 1
	str	w0, [x29,64]
	cbz	w21, .L978
	adrp	x22, .LC82
	adrp	x23, .LC83
	mov	w21, 0
	add	x22, x22, :lo12:.LC82
	add	x23, x23, :lo12:.LC83
.L979:
	ldr	w2, [x19,28]
	ldr	w1, [x29,64]
	ubfx	x2, x2, 16, 5
	ubfx	x1, x1, 22, 6
	cmp	w2, w1
	bge	.L987
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w0, [x0,1872]
	cmp	w0, 5
	bhi	.L980
.L983:
	add	w21, w21, 1
	and	w0, w21, 16777215
	cbnz	w0, .L979
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
	b	.L979
.L980:
	ldr	w0, [x19]
	str	w0, [x29,72]
	ldr	w0, [x29,72]
	tbz	x0, 13, .L983
	ldr	w0, [x29,72]
	tbz	x0, 17, .L983
.L987:
	add	x19, x20, :lo12:.LANCHOR0
	add	x19, x19, 1888
	ldr	w0, [x19,40]
	cbz	w0, .L988
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
	b	.L988
.L978:
	adrp	x22, .LC84
	adrp	x23, .LC83
	add	x22, x22, :lo12:.LC84
	add	x23, x23, :lo12:.LC83
.L989:
	ldr	w0, [x29,64]
	tbnz	x0, 20, .L1015
	ldr	w0, [x19,8]
	add	w21, w21, 1
	str	w0, [x29,64]
	and	w0, w21, 16777215
	cbnz	w0, .L989
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
	b	.L989
.L1015:
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w0, [x0,1936]
	cbz	w0, .L992
	mov	x0, x19
	bl	NandcSendDumpDataStart
.L992:
	add	x21, x20, :lo12:.LANCHOR0
	add	x21, x21, 1888
	ldr	w0, [x21,40]
	cbz	w0, .L993
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
.L993:
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w0, [x0,1936]
	cbz	w0, .L988
	mov	x0, x19
	bl	NandcSendDumpDataDone
.L988:
	add	x20, x20, :lo12:.LANCHOR0
	str	wzr, [x20,1928]
	b	.L976
.L1007:
	ldr	w0, [x19,8]
	str	w0, [x29,64]
	ldr	w0, [x29,64]
	tbz	x0, 20, .L1007
.L976:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldr	x23, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	NandcXferComp, .-NandcXferComp
	.align	2
	.global	NandcXferData
	.type	NandcXferData, %function
NandcXferData:
	stp	x29, x30, [sp, -176]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	adrp	x19, .LANCHOR0
	uxtb	w25, w0
	add	x0, x19, :lo12:.LANCHOR0
	add	x0, x0, x25, sxtw 4
	ldr	x20, [x0,24]
	and	x0, x3, 63
	uxtb	w21, w1
	uxtb	w24, w2
	mov	x26, x3
	mov	x22, x4
	cbnz	x0, .L1017
	cbnz	x4, .L1018
	add	x0, x29, 112
	mov	w1, 255
	mov	w2, 64
	add	x22, x29, 112
	bl	ftl_memset
.L1018:
	mov	w0, w25
	mov	w1, w21
	mov	w2, w24
	mov	w3, 0
	mov	x4, x26
	mov	x5, x22
	mov	w23, 0
	bl	NandcXferStart
	mov	w0, w25
	bl	NandcXferComp
	cbnz	w21, .L1019
	add	x1, x19, :lo12:.LANCHOR0
	ubfx	x2, x24, 1, 7
	mov	w3, 128
	add	x2, x22, x2, lsl 2
	ldr	w0, [x1,1940]
	cmp	w0, 25
	mov	w0, 64
	csel	w3, w0, w3, cc
	mov	w0, w21
.L1021:
	cmp	x22, x2
	add	w4, w0, w3
	beq	.L1063
	ldr	x5, [x1,1896]
	and	x0, x0, 4294967292
	add	x22, x22, 4
	ldr	w0, [x5,x0]
	strb	w0, [x22,-4]
	lsr	w5, w0, 8
	strb	w5, [x22,-3]
	lsr	w5, w0, 16
	strb	w5, [x22,-2]
	lsr	w0, w0, 24
	strb	w0, [x22,-1]
	mov	w0, w4
	b	.L1021
.L1063:
	add	x0, x19, :lo12:.LANCHOR0
	lsr	w24, w24, 2
	ldr	w5, [x0,1940]
	ldr	w4, [x0,1872]
	mov	w0, 0
	mov	w23, w0
.L1023:
	cmp	w0, w24
	bcs	.L1019
	cbz	w5, .L1019
	uxtw	x1, w0
	add	x1, x1, 8
	ldr	w1, [x20,x1,lsl 2]
	str	w1, [x29,104]
	ldr	w1, [x29,104]
	tbnz	x1, 2, .L1047
	ldr	w3, [x29,104]
	ubfx	x3, x3, 15, 1
	cbnz	w3, .L1047
	cmp	w4, 5
	bls	.L1025
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
	bls	.L1026
	ldr	w1, [x29,104]
	ubfx	x3, x3, 3, 5
	ubfx	x1, x1, 27, 1
	orr	w3, w3, w1, lsl 5
	b	.L1027
.L1026:
	ubfx	x1, x3, 16, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 29, 1
	b	.L1062
.L1025:
	cmp	w4, 3
	bls	.L1027
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
	bls	.L1028
	ubfx	x1, x3, 3, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 28, 1
	b	.L1062
.L1028:
	ubfx	x1, x3, 16, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 30, 1
.L1062:
	orr	w3, w1, w3, lsl 5
.L1027:
	cmp	w23, w3
	csel	w23, w23, w3, cs
	b	.L1024
.L1047:
	mov	w23, -1
.L1024:
	add	w0, w0, 1
	b	.L1023
.L1019:
	str	wzr, [x20,16]
	b	.L1030
.L1017:
	cmp	w21, 1
	bne	.L1061
	mov	w23, 0
	mov	w28, 2
.L1031:
	cmp	w23, w24
	bcs	.L1064
	and	w27, w23, 3
	mov	x3, x26
	cbz	x26, .L1033
	ubfiz	x0, x23, 9, 23
	add	x3, x26, x0
.L1033:
	cmp	x22, xzr
	mov	x0, x20
	csel	w4, w28, wzr, ne
	mov	w1, 1
	mov	w2, w27
	mul	w4, w4, w23
	add	w23, w23, 2
	add	x4, x22, x4
	bl	NandcCopy1KB
	mov	x4, 0
	mov	w0, w25
	mov	w1, 1
	mov	w2, 2
	mov	w3, w27
	mov	x5, x4
	bl	NandcXferStart
	mov	w0, w25
	bl	NandcXferComp
	b	.L1031
.L1064:
	mov	w23, 0
	b	.L1030
.L1061:
	mov	w1, 0
	mov	x4, 0
	mov	w27, 0
	mov	w0, w25
	mov	w2, 2
	mov	w3, w1
	mov	x5, x4
	mov	w23, w27
	bl	NandcXferStart
.L1036:
	cmp	w27, w24
	bcs	.L1030
	mov	w0, w25
	add	w28, w27, 2
	bl	NandcXferComp
	ldr	w0, [x20,32]
	cmp	w28, w24
	str	w0, [x29,104]
	bcs	.L1037
	mov	x4, 0
	mov	w0, w25
	mov	w1, 0
	mov	w2, 2
	and	w3, w28, 3
	mov	x5, x4
	bl	NandcXferStart
.L1037:
	ldr	w0, [x29,104]
	tbnz	x0, 2, .L1052
	ldr	w0, [x29,104]
	ubfx	x1, x0, 3, 5
	ldr	w0, [x29,104]
	ubfx	x0, x0, 27, 1
	orr	w0, w1, w0, lsl 5
	cmp	w23, w0
	csel	w23, w23, w0, cs
	b	.L1038
.L1052:
	mov	w23, -1
.L1038:
	and	w2, w27, 3
	mov	x3, x26
	cbz	x26, .L1039
	ubfiz	x3, x27, 9, 23
	add	x3, x26, x3
.L1039:
	cmp	x22, xzr
	mov	w0, 2
	csel	w4, w0, wzr, ne
	mov	w1, 0
	mov	x0, x20
	mul	w4, w4, w27
	mov	w27, w28
	add	x4, x22, x4
	bl	NandcCopy1KB
	b	.L1036
.L1030:
	cbnz	w21, .L1042
	add	x19, x19, :lo12:.LANCHOR0
	ldr	w0, [x19,1872]
	cmp	w0, 5
	bls	.L1042
	ldr	w0, [x20]
	mov	w1, 8192
	movk	w1, 0x2, lsl 16
	and	w1, w0, w1
	cmp	w1, 139264
	bne	.L1042
	orr	w0, w0, 131072
	mov	w23, -1
	str	w0, [x20]
.L1042:
	mov	w0, w23
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
	cbnz	w19, .L1066
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w5, [x0,9]
	ldr	w0, [x0,12]
	mul	w0, w5, w0
	cmp	w1, w0
	mov	w0, 4
	csel	w20, w20, w0, cs
.L1066:
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
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	stp	x25, x26, [sp,64]
	uxtb	w23, w0
	mov	w22, 6
	ldr	x0, [x19,128]
	mov	w24, w1
	mov	x28, x2
	mov	x27, x3
	mov	w25, w4
	mov	w21, 1024
	ldr	w0, [x0,304]
	str	w0, [x29,124]
	ldr	w0, [x19,1872]
	cmp	w0, 8
	mov	w0, 12
	csel	w22, w22, w0, cc
	cbz	w4, .L1069
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	mov	w0, w23
	bl	FlashReset
	mov	w1, w24
	mov	x2, x28
	mov	x3, x27
	mov	w0, w23
	bl	FlashReadRawPage
	mov	w21, w0
	ldrb	w0, [x19,1844]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x19,1844]
	bl	NandcSetMode
	cmn	w21, #1
	bne	.L1070
.L1079:
	mov	w21, -1
	b	.L1071
.L1070:
	adrp	x0, .LC85
	mov	w1, w24
	add	x0, x0, :lo12:.LC85
	mov	w2, w21
	bl	printk
	cmp	w21, 9
	bhi	.L1072
	add	x19, x19, x23, sxtw 4
	ldr	x0, [x19,24]
	ldr	w1, [x0,3840]
	ldr	w1, [x0]
	orr	w1, w1, 131072
	str	w1, [x0]
.L1072:
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	ldr	w1, [x0,40]
	add	w1, w1, 1
	str	w1, [x0,40]
	cmp	w1, 2047
	bls	.L1071
	mov	x27, 0
	str	wzr, [x0,40]
	mov	x28, x27
.L1069:
	mov	w19, 0
	mov	w26, -1
	mov	w5, w19
	mov	w6, w19
	mov	w20, w19
.L1077:
	mov	w0, w22
	str	x5, [x29,104]
	str	x6, [x29,112]
	bl	NandcSetDdrPara
	mov	w1, w24
	mov	w0, w23
	mov	x2, x28
	mov	x3, x27
	bl	FlashReadRawPage
	add	w1, w21, 1
	ldr	x6, [x29,112]
	cmp	w0, w1
	ldr	x5, [x29,104]
	bhi	.L1073
	cmp	w0, 2
	bhi	.L1083
	add	w20, w20, 1
	cmp	w20, 9
	bls	.L1083
	sub	w19, w22, w20
	mov	w21, w0
	mov	w26, 0
	b	.L1075
.L1073:
	cmp	w6, w20
	bcs	.L1084
	cmp	w20, 7
	sub	w5, w19, w20
	bhi	.L1085
	mov	w6, w20
	b	.L1084
.L1083:
	mov	x27, 0
	mov	w19, w22
	mov	w21, w0
	mov	w26, 0
	mov	x28, x27
	b	.L1074
.L1084:
	mov	w20, 0
.L1074:
	add	w22, w22, 2
	cmp	w22, 69
	bls	.L1077
.L1075:
	cmp	w6, w20
	csel	w19, w19, w5, cc
	b	.L1076
.L1085:
	mov	w19, w5
.L1076:
	cbz	w19, .L1078
	adrp	x0, .LC86
	mov	w1, w19
	add	x0, x0, :lo12:.LC86
	bl	printk
	mov	w0, w19
	bl	NandcSetDdrPara
.L1078:
	cbz	w26, .L1071
	adrp	x0, .LC87
	mov	w1, w23
	add	x0, x0, :lo12:.LC87
	mov	w2, w24
	bl	printk
	cbz	w25, .L1079
	ldr	w0, [x29,124]
	lsr	w0, w0, 8
	bl	NandcSetDdrPara
.L1071:
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
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	uxtb	w21, w0
	mov	w22, w1
	mov	w0, w21
	stp	x23, x24, [sp,48]
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	mov	x24, x2
	mov	x23, x3
	bl	FlashReadRawPage
	cmn	w0, #1
	mov	w19, w0
	bne	.L1095
	adrp	x20, .LANCHOR0
	add	x25, x20, :lo12:.LANCHOR0
	ldrb	w26, [x25,16]
	cbnz	w26, .L1096
.L1098:
	add	x20, x20, :lo12:.LANCHOR0
	ldrb	w0, [x20,1864]
	cbz	w0, .L1095
	b	.L1116
.L1096:
	mov	w0, w21
	mov	w1, w22
	mov	x2, x24
	mov	x3, x23
	strb	wzr, [x25,16]
	bl	FlashReadRawPage
	strb	w26, [x25,16]
	cmn	w0, #1
	beq	.L1098
	mov	w19, w0
	b	.L1095
.L1116:
	ldr	x0, [x20,128]
	mov	w1, w22
	mov	x2, x24
	mov	x3, x23
	mov	w4, 1
	ldr	w25, [x0,304]
	mov	w0, w21
	bl	FlashDdrTunningRead
	cmn	w0, #1
	mov	w19, w0
	beq	.L1099
	ldrb	w0, [x20,1944]
	cmp	w19, w0, lsr 1
	bls	.L1095
.L1099:
	lsr	w0, w25, 8
	bl	NandcSetDdrPara
.L1095:
	adrp	x20, .LANCHOR4
	cmn	w19, #1
	add	x20, x20, :lo12:.LANCHOR4
	ldr	x4, [x20,48]
	bne	.L1100
	cbz	x4, .L1100
	mov	w1, w22
	mov	x2, x24
	mov	x3, x23
	mov	w0, w21
	blr	x4
	mov	w19, w0
	adrp	x0, .LC88
	mov	w1, w19
	add	x0, x0, :lo12:.LC88
	mov	w2, w21
	mov	w3, w22
	bl	printk
	cmn	w19, #1
	bne	.L1100
	adrp	x0, .LANCHOR0+120
	ldrb	w0, [x0,#:lo12:.LANCHOR0+120]
	cbz	w0, .L1100
	mov	w0, w21
	bl	flash_enter_slc_mode
	ldr	x4, [x20,48]
	mov	w0, w21
	mov	w1, w22
	mov	x2, x24
	mov	x3, x23
	blr	x4
	mov	w19, w0
.L1100:
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FlashReadPage, .-FlashReadPage
	.align	2
	.global	FlashDdrParaScan
	.type	FlashDdrParaScan, %function
FlashDdrParaScan:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	mov	w20, w1
	stp	x21, x22, [sp,32]
	add	x22, x19, :lo12:.LANCHOR0
	uxtb	w21, w0
	ldrb	w0, [x22,1844]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x22,1844]
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
	beq	.L1121
	cmn	w22, #1
	bne	.L1118
.L1121:
	add	x20, x19, :lo12:.LANCHOR0
	ldrb	w0, [x20,1844]
	tbz	x0, 0, .L1118
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	strb	wzr, [x20,1864]
	b	.L1120
.L1118:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, 1
	strb	w0, [x19,1864]
.L1120:
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
	stp	x29, x30, [sp, -128]!
	mov	w0, 60
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	adrp	x25, .LANCHOR0
	mov	w26, 4
	strb	w0, [x29,120]
	mov	w0, 40
	strb	w0, [x29,121]
	mov	w0, 24
	stp	x23, x24, [sp,48]
	strb	w0, [x29,122]
	adrp	x24, .LANCHOR4
	mov	w0, 16
	add	x1, x25, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	strb	w0, [x29,123]
	adrp	x22, .LANCHOR1
	add	x0, x24, :lo12:.LANCHOR4
	add	x21, x22, :lo12:.LANCHOR1
	ldr	x1, [x1,1856]
	add	x21, x21, 472
	mov	w20, 0
	mov	w23, -1
	str	x1, [x0,56]
	str	wzr, [x0,64]
	mov	w0, 0
	ldrh	w27, [x21,10]
	bl	flash_enter_slc_mode
.L1129:
	add	w28, w20, 1
	mov	x19, 0
.L1131:
	add	x0, x29, 120
	ldrb	w0, [x19,x0]
	bl	FlashBchSel
	add	x4, x25, :lo12:.LANCHOR0
	mov	w0, 0
	mov	w1, w20
	mov	x3, 0
	str	x4, [x29,104]
	ldr	x2, [x4,1856]
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L1130
	ldr	x4, [x29,104]
	mov	w0, 0
	mov	w1, w28
	mov	x3, 0
	ldr	x2, [x4,1856]
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L1130
	add	x19, x19, 1
	cmp	x19, 4
	beq	.L1132
	b	.L1131
.L1133:
	mov	w1, 2036
	add	x0, x28, 12
	mov	w23, -1
	bl	JSHash
	ldr	w1, [x28,8]
	cmp	w1, w0
	beq	.L1143
.L1132:
	subs	w26, w26, #1
	add	w20, w20, w27
	bne	.L1129
	mov	w0, w26
.L1142:
	bl	flash_exit_slc_mode
	mov	w0, w23
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
.L1130:
	add	x19, x24, :lo12:.LANCHOR4
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	ldr	x28, [x19,56]
	ldr	w1, [x28]
	cmp	w1, w0
	bne	.L1132
	cbnz	w23, .L1133
	add	x22, x22, :lo12:.LANCHOR1
	ldrh	w0, [x22,482]
	udiv	w20, w20, w0
	mov	w0, w23
	add	w20, w20, 1
	str	w20, [x19,68]
	b	.L1142
.L1143:
	add	x1, x28, 160
	mov	w2, 32
	mov	x0, x21
	add	x23, x25, :lo12:.LANCHOR0
	bl	ftl_memcpy
	ldr	x1, [x19,56]
	mov	w2, 32
	add	x0, x23, 88
	add	x1, x1, 192
	bl	ftl_memcpy
	ldr	x1, [x19,56]
	mov	w2, 852
	add	x0, x23, 760
	add	x1, x1, 224
	bl	ftl_memcpy
	str	w20, [x19,64]
	ldr	x1, [x19,56]
	ldr	w0, [x1,1076]
	strb	w0, [x23,1864]
	ldrh	w0, [x21,10]
	udiv	w0, w20, w0
	add	w2, w0, 1
	cbz	w0, .L1135
	str	w2, [x19,68]
	b	.L1136
.L1135:
	mov	w0, 2
	str	w0, [x19,68]
.L1136:
	add	x0, x24, :lo12:.LANCHOR4
	ldrh	w1, [x1,14]
	mov	w23, 0
	strb	w1, [x0,72]
	b	.L1132
	.size	FlashLoadPhyInfo, .-FlashLoadPhyInfo
	.align	2
	.global	ToshibaReadRetrial
	.type	ToshibaReadRetrial, %function
ToshibaReadRetrial:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	uxtb	w23, w0
	mov	w0, w23
	str	w1, [x29,116]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	mov	x27, x2
	mov	x26, x3
	adrp	x19, .LANCHOR0
	bl	NandcWaitFlashReady
	add	x2, x19, :lo12:.LANCHOR0
	sbfiz	x0, x23, 4, 32
	add	x1, x2, 24
	mov	w22, 0
	add	x3, x1, x0
	ldr	x24, [x1,x0]
	ldrb	w0, [x3,8]
	str	x0, [x29,120]
	add	x20, x0, 8
	mov	x28, x0
	ldrb	w0, [x2,752]
	add	x20, x24, x20, lsl 8
	sub	w0, w0, #67
	uxtb	w0, w0
	cmp	w0, 1
	bls	.L1145
	ldrb	w0, [x2,1864]
	cbz	w0, .L1146
	mov	w0, w22
	mov	w22, 1
	bl	NandcSetDdrMode
.L1146:
	ubfiz	x0, x28, 8, 8
	mov	w2, 92
	add	x0, x24, x0
	str	w2, [x0,2056]
	mov	w2, 197
	str	w2, [x0,2056]
.L1145:
	ldr	x0, [x29,120]
	mov	w21, 1
	mov	w25, -1
	add	x0, x0, 8
	lsl	x0, x0, 8
	str	x0, [x29,104]
	ubfiz	x0, x28, 8, 8
	str	x0, [x29,96]
.L1147:
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	ldrb	w0, [x0,73]
	add	w0, w0, 1
	cmp	w21, w0
	bcs	.L1172
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, w21
	ldrb	w0, [x0,752]
	sub	w0, w0, #67
	uxtb	w0, w0
	cmp	w0, 1
	mov	x0, x20
	bhi	.L1148
	bl	SandiskSetRRPara
	b	.L1149
.L1148:
	bl	ToshibaSetRRPara
.L1149:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,752]
	cmp	w0, 34
	bne	.L1150
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	ldrb	w0, [x0,73]
	sub	w0, w0, #3
	cmp	w21, w0
	bne	.L1150
	ldr	x0, [x29,104]
	mov	w1, 179
	add	x0, x24, x0
	str	w1, [x0,8]
.L1150:
	ldr	x0, [x29,96]
	mov	w1, 38
	add	x0, x24, x0
	str	w1, [x0,2056]
	mov	w1, 93
	str	w1, [x0,2056]
	cbz	w22, .L1151
	mov	w0, 4
	bl	NandcSetDdrMode
	ldr	w1, [x29,116]
	mov	w0, w23
	mov	x2, x27
	mov	x3, x26
	bl	FlashReadRawPage
	mov	w28, w0
	mov	w0, 0
	bl	NandcSetDdrMode
	b	.L1152
.L1151:
	ldr	w1, [x29,116]
	mov	w0, w23
	mov	x2, x27
	mov	x3, x26
	bl	FlashReadRawPage
	mov	w28, w0
.L1152:
	cmn	w28, #1
	beq	.L1153
	add	x0, x19, :lo12:.LANCHOR0
	cmn	w25, #1
	csel	w25, w25, w28, ne
	ldrb	w0, [x0,1944]
	add	w0, w0, w0, lsl 1
	cmp	w28, w0, lsr 2
	bcc	.L1155
	mov	x26, 0
	mov	x27, x26
.L1153:
	add	w21, w21, 1
	b	.L1147
.L1172:
	mov	w28, w25
.L1155:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, 0
	ldrb	w0, [x0,752]
	sub	w0, w0, #67
	uxtb	w0, w0
	cmp	w0, 1
	mov	x0, x20
	bhi	.L1157
	bl	SandiskSetRRPara
	b	.L1158
.L1157:
	bl	ToshibaSetRRPara
.L1158:
	ldr	x0, [x29,120]
	add	x19, x19, :lo12:.LANCHOR0
	add	x0, x0, 8
	add	x24, x24, x0, lsl 8
	mov	w0, 255
	str	w0, [x24,8]
	ldrb	w0, [x19,1944]
	add	w0, w0, w0, lsl 1
	cmp	w28, w0, lsr 2
	bcc	.L1159
	cmn	w28, #1
	mov	w0, 256
	csel	w28, w28, w0, eq
.L1159:
	mov	w0, w23
	bl	NandcWaitFlashReady
	cbz	w22, .L1160
	mov	w0, 4
	bl	NandcSetDdrMode
.L1160:
	mov	w0, w28
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
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
	adrp	x27, .LANCHOR4
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
	add	x0, x0, 24
	mov	w19, -1
	add	x2, x0, x1
	ldr	x0, [x0,x1]
	ldrb	w21, [x2,8]
	add	x21, x21, 8
	add	x21, x0, x21, lsl 8
.L1174:
	add	x0, x27, :lo12:.LANCHOR4
	ldrb	w0, [x0,73]
	add	w0, w0, 1
	cmp	w26, w0
	bcs	.L1177
	mov	x0, x21
	mov	w1, w26
	bl	SamsungSetRRPara
	mov	w0, w22
	mov	w1, w25
	mov	x2, x24
	mov	x3, x23
	bl	FlashReadRawPage
	cmn	w0, #1
	beq	.L1175
	add	x1, x20, :lo12:.LANCHOR0
	cmn	w19, #1
	csel	w19, w19, w0, ne
	ldrb	w1, [x1,1944]
	add	w1, w1, w1, lsl 1
	cmp	w0, w1, lsr 2
	bcc	.L1180
	mov	x23, 0
	mov	x24, x23
.L1175:
	add	w26, w26, 1
	b	.L1174
.L1180:
	mov	w19, w0
.L1177:
	mov	x0, x21
	mov	w1, 0
	add	x20, x20, :lo12:.LANCHOR0
	bl	SamsungSetRRPara
	ldrb	w0, [x20,1944]
	add	w0, w0, w0, lsl 1
	cmp	w19, w0, lsr 2
	bcc	.L1179
	cmn	w19, #1
	mov	w0, 256
	csel	w19, w19, w0, eq
.L1179:
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
	stp	x29, x30, [sp, -144]!
	adrp	x4, .LANCHOR0
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	mov	w24, w1
	add	x1, x4, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	uxtb	w22, w0
	mov	x25, x2
	ldrb	w0, [x1,1944]
	mov	x23, x3
	ldrb	w1, [x1,120]
	mov	x19, x4
	lsr	w21, w0, 2
	cbnz	w1, .L1188
	add	w0, w0, w0, lsl 1
	ubfx	x21, x0, 2, 8
.L1188:
	mov	w0, w22
	mov	w27, 0
	bl	NandcWaitFlashReady
	add	x4, x19, :lo12:.LANCHOR0
	sbfiz	x0, x22, 4, 32
	add	x4, x4, 24
	mov	w19, -1
	add	x1, x4, x0
	adrp	x5, .LANCHOR4
	mov	w7, 239
	mov	w8, 137
	ldr	x28, [x4,x0]
	ldrb	w20, [x1,8]
	lsl	x6, x20, 8
.L1189:
	add	x0, x5, :lo12:.LANCHOR4
	ldrb	w0, [x0,73]
	cmp	w27, w0
	bcs	.L1192
	add	x26, x28, x6
	mov	w0, 200
	str	x5, [x29,104]
	str	x6, [x29,112]
	str	w7, [x26,2056]
	str	w8, [x26,2052]
	str	x7, [x29,120]
	str	x8, [x29,128]
	bl	NandcDelayns
	add	w4, w27, 1
	str	w4, [x26,2048]
	str	wzr, [x26,2048]
	mov	w0, w22
	str	wzr, [x26,2048]
	mov	w1, w24
	str	wzr, [x26,2048]
	mov	x2, x25
	mov	x3, x23
	str	x4, [x29,136]
	bl	FlashReadRawPage
	cmn	w0, #1
	ldr	x4, [x29,136]
	ldr	x8, [x29,128]
	ldr	x7, [x29,120]
	ldr	x6, [x29,112]
	ldr	x5, [x29,104]
	beq	.L1190
	cmn	w19, #1
	csel	w19, w19, w0, ne
	cmp	w0, w21
	bcc	.L1197
	mov	x23, 0
	mov	x25, x23
.L1190:
	mov	w27, w4
	b	.L1189
.L1197:
	mov	w19, w0
.L1192:
	add	x20, x28, x20, lsl 8
	mov	w0, 239
	str	w0, [x20,2056]
	mov	w0, 137
	str	w0, [x20,2052]
	mov	w0, 200
	bl	NandcDelayns
	str	wzr, [x20,2048]
	str	wzr, [x20,2048]
	cmp	w19, w21
	str	wzr, [x20,2048]
	str	wzr, [x20,2048]
	bcc	.L1194
	cmn	w19, #1
	mov	w0, 256
	csel	w19, w19, w0, eq
.L1194:
	cmn	w19, #1
	beq	.L1198
	cmp	w19, 256
	bne	.L1195
.L1198:
	adrp	x0, .LC89
	mov	w1, w27
	add	x0, x0, :lo12:.LC89
	mov	w2, w24
	mov	w3, w27
	mov	w4, w19
	bl	printk
.L1195:
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
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
	stp	x27, x28, [sp,80]
	add	x28, x21, :lo12:.LANCHOR0
	mov	w27, w1
	add	x0, x28, 760
	add	x1, x0, x22
	mov	w0, w22
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	mov	x26, x2
	mov	x24, x3
	ldrb	w20, [x1,12]
	mov	x23, x22
	ldrb	w25, [x28,762]
	bl	NandcWaitFlashReady
	mov	w4, 0
	mov	w19, -1
	add	x5, x28, 764
.L1208:
	cmp	w4, w25
	bcs	.L1212
	add	w20, w20, 1
	ldrb	w1, [x28,761]
	mov	x2, x5
	mov	w0, w23
	uxtb	w20, w20
	str	x4, [x29,96]
	cmp	w20, w25
	str	x5, [x29,104]
	csel	w20, w20, wzr, cc
	mov	w3, w20
	bl	HynixSetRRPara
	mov	w0, w23
	mov	w1, w27
	mov	x2, x26
	mov	x3, x24
	bl	FlashReadRawPage
	cmn	w0, #1
	ldr	x5, [x29,104]
	ldr	x4, [x29,96]
	beq	.L1210
	ldrb	w1, [x28,1944]
	cmn	w19, #1
	csel	w19, w19, w0, ne
	add	w1, w1, w1, lsl 1
	cmp	w0, w1, lsr 2
	bcc	.L1215
	mov	x24, 0
	mov	x26, x24
.L1210:
	add	w4, w4, 1
	b	.L1208
.L1215:
	mov	w19, w0
.L1212:
	add	x21, x21, :lo12:.LANCHOR0
	add	x22, x21, x22
	ldrb	w0, [x21,1944]
	strb	w20, [x22,772]
	add	w0, w0, w0, lsl 1
	cmp	w19, w0, lsr 2
	bcc	.L1214
	cmn	w19, #1
	mov	w0, 256
	csel	w19, w19, w0, eq
.L1214:
	mov	w0, w19
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
	cbnz	w19, .L1222
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w1, [x0,9]
	ldr	w2, [x0,12]
	mul	w1, w1, w2
	cmp	w20, w1
	bcs	.L1222
	ldrb	w0, [x0,8]
	cbnz	w0, .L1223
	sub	w21, w21, #2
	b	.L1222
.L1223:
	mov	w21, 4
.L1222:
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
	stp	x23, x24, [sp,48]
	adrp	x23, .LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	add	x20, x23, :lo12:.LANCHOR0
	adrp	x22, .LANCHOR4
	add	x19, x22, :lo12:.LANCHOR4
	mov	w21, 0
	stp	x25, x26, [sp,64]
	ldr	x0, [x20,1856]
	str	x0, [x19,56]
	ldrb	w0, [x19,74]
	bl	FlashBchSel
	ldr	x0, [x20,1856]
	mov	w1, 0
	mov	w2, 2048
	bl	ftl_memset
	ldr	x1, [x19,56]
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	mov	w2, 32
	str	w0, [x1]
	ldr	x0, [x19,56]
	ldrb	w1, [x20,1845]
	add	x0, x0, 16
	strh	w1, [x0,-4]
	ldrb	w1, [x20,9]
	strh	w1, [x0,-2]
	ldrb	w1, [x20,1864]
	str	w1, [x0,1060]
	add	x1, x20, 1620
	bl	ftl_memcpy
	ldr	x0, [x19,56]
	add	x1, x20, 1848
	mov	w2, 8
	add	x0, x0, 80
	bl	ftl_memcpy
	ldr	x0, [x19,56]
	add	x1, x20, 708
	mov	w2, 32
	add	x0, x0, 96
	bl	ftl_memcpy
	ldr	x0, [x19,56]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	mov	w2, 32
	add	x1, x1, 472
	add	x0, x0, 160
	bl	ftl_memcpy
	ldr	x0, [x19,56]
	add	x1, x20, 88
	mov	w2, 32
	add	x0, x0, 192
	bl	ftl_memcpy
	ldr	x0, [x19,56]
	add	x1, x20, 760
	mov	w2, 852
	add	x0, x0, 224
	bl	ftl_memcpy
	ldr	x20, [x19,56]
	mov	w1, 2036
	add	x0, x20, 12
	bl	JSHash
	str	w0, [x20,8]
	mov	w0, 1592
	str	w0, [x20,4]
	ldr	x0, [x19,80]
	str	x0, [x19,56]
	mov	w0, 0
	mov	w19, w21
	bl	flash_enter_slc_mode
.L1227:
	add	x20, x23, :lo12:.LANCHOR0
	mov	w2, 0
	mov	w0, 0
	add	x24, x22, :lo12:.LANCHOR4
	add	w26, w19, 1
	ldr	w1, [x20,12]
	mul	w1, w19, w1
	bl	FlashEraseBlock
	ldr	w1, [x20,12]
	mov	x3, 0
	ldr	x2, [x20,1856]
	mov	w0, 0
	mul	w1, w19, w1
	bl	FlashProgPage
	ldr	w1, [x20,12]
	mov	x3, 0
	ldr	x2, [x20,1856]
	mov	w0, 0
	mul	w1, w19, w1
	add	w1, w1, 1
	bl	FlashProgPage
	ldr	w1, [x20,12]
	mov	w0, 0
	ldr	x2, [x24,80]
	mov	x3, 0
	mul	w1, w19, w1
	bl	FlashReadRawPage
	cmn	w0, #1
	beq	.L1225
	ldr	x25, [x24,56]
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	ldr	w1, [x25]
	cmp	w1, w0
	bne	.L1225
	mov	w1, 2036
	add	x0, x25, 12
	bl	JSHash
	ldr	w1, [x25,8]
	cmp	w1, w0
	bne	.L1225
	ldr	w0, [x20,12]
	cmp	w21, 1
	str	w26, [x24,68]
	mul	w19, w19, w0
	str	w19, [x24,64]
	beq	.L1228
	mov	w21, 1
.L1225:
	cmp	w26, 4
	mov	w19, w26
	bne	.L1227
	b	.L1226
.L1228:
	mov	w21, 2
.L1226:
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
	stp	x25, x26, [sp,64]
	mov	x26, x0
	mov	w0, 60
	strb	w0, [x29,104]
	mov	w0, 40
	strb	w0, [x29,105]
	mov	w0, 24
	stp	x19, x20, [sp,16]
	strb	w0, [x29,106]
	adrp	x19, .LANCHOR0
	mov	w0, 16
	strb	w0, [x29,107]
	add	x0, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	ldr	w1, [x0,1876]
	ldrb	w27, [x0,1944]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w1, w0
	bne	.L1234
	mov	w0, 0
	bl	flash_enter_slc_mode
.L1234:
	adrp	x24, .LC90
	mov	x0, x26
	mov	w1, 0
	mov	w2, 2048
	mov	w23, -1
	mov	w20, 2
	add	x24, x24, :lo12:.LC90
	adrp	x28, .LANCHOR4
	bl	ftl_memset
.L1235:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,9]
	cmp	w20, w0
	bcs	.L1239
	mov	x22, 0
.L1237:
	add	x0, x29, 104
	add	x21, x19, :lo12:.LANCHOR0
	ldrb	w25, [x22,x0]
	mov	w0, w25
	bl	FlashBchSel
	ldr	w1, [x21,12]
	mov	w0, 0
	ldr	x2, [x21,1856]
	mov	x3, 0
	mul	w1, w20, w1
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L1236
	add	x22, x22, 1
	cmp	x22, 4
	bne	.L1237
	b	.L1238
.L1236:
	ldr	x0, [x21,1856]
	ldr	w1, [x0]
	mov	w0, 35899
	movk	w0, 0xfcdc, lsl 16
	cmp	w1, w0
	bne	.L1238
	mov	w1, w25
	mov	x0, x24
	bl	printk
	ldr	x1, [x21,1856]
	mov	x0, x26
	mov	w2, 2048
	bl	ftl_memcpy
	ldr	x0, [x21,1856]
	ldr	w0, [x0,512]
	strb	w0, [x21,9]
	add	x0, x28, :lo12:.LANCHOR4
	ldr	w1, [x0,68]
	cmp	w1, w20
	bls	.L1242
	mov	w23, 0
	str	w20, [x0,68]
	bl	FlashSavePhyInfo
.L1238:
	add	w20, w20, 1
	b	.L1235
.L1242:
	mov	w23, 0
.L1239:
	mov	w0, w27
	add	x19, x19, :lo12:.LANCHOR0
	bl	FlashBchSel
	ldr	w1, [x19,1876]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w1, w0
	bne	.L1244
	mov	w0, 0
	bl	flash_exit_slc_mode
.L1244:
	mov	w0, w23
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
	adrp	x19, .LANCHOR0
	add	x20, x19, :lo12:.LANCHOR0
	stp	x23, x24, [sp,48]
	mov	x23, x0
	mov	w0, 32768
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	adrp	x21, .LANCHOR4
	bl	ftl_malloc
	add	x22, x21, :lo12:.LANCHOR4
	str	x0, [x20,1856]
	mov	w0, 32768
	add	x24, x20, 24
	adrp	x25, .LC91
	mov	w28, 0
	bl	ftl_malloc
	str	x0, [x22,80]
	mov	w0, 4096
	mov	w27, 144
	bl	ftl_malloc
	str	x0, [x22,88]
	mov	w0, 32768
	bl	ftl_malloc
	str	x0, [x22,96]
	mov	w0, 4096
	bl	ftl_malloc
	str	x0, [x22,104]
	mov	w0, 128
	str	w0, [x20,12]
	mov	w0, 60
	mov	w1, 50
	strb	w0, [x22,74]
	mov	x0, x23
	strb	wzr, [x20,1864]
	strb	w1, [x20,9]
	strb	wzr, [x20,8]
	add	x20, x20, 1620
	strb	w1, [x22,72]
	mov	x26, x20
	str	wzr, [x22,68]
	str	wzr, [x22,40]
	bl	NandcInit
	add	x0, x25, :lo12:.LC91
	str	x0, [x29,104]
.L1250:
	uxtb	w25, w28
	mov	w0, w25
	bl	FlashReset
	ldrb	w23, [x24,8]
	ldr	x22, [x24]
	mov	w0, w25
	ubfiz	x23, x23, 8, 8
	add	x22, x22, x23
	bl	NandcFlashCs
	mov	w0, 200
	str	w27, [x22,2056]
	str	wzr, [x22,2052]
	bl	NandcDelayns
	ldr	w0, [x22,2048]
	strb	w0, [x20]
	ldr	w0, [x22,2048]
	strb	w0, [x20,1]
	ldr	w0, [x22,2048]
	strb	w0, [x20,2]
	ldr	w0, [x22,2048]
	strb	w0, [x20,3]
	ldr	w0, [x22,2048]
	strb	w0, [x20,4]
	ldr	w0, [x22,2048]
	strb	w0, [x20,5]
	mov	w0, w25
	bl	NandcFlashDeCs
	ldrb	w2, [x20]
	sub	w0, w2, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L1247
	ldr	x0, [x29,104]
	add	w1, w28, 1
	ldrb	w3, [x20,1]
	ldrb	w4, [x20,2]
	ldrb	w5, [x20,3]
	ldrb	w6, [x20,4]
	ldrb	w7, [x20,5]
	bl	printk
.L1247:
	cbnz	w28, .L1248
	ldrb	w0, [x26]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L1290
	ldrb	w0, [x26,1]
	cmp	w0, 255
	beq	.L1290
.L1248:
	add	w28, w28, 1
	add	x24, x24, 16
	cmp	w28, 4
	add	x20, x20, 8
	bne	.L1250
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x0,1620]
	cmp	w1, 173
	beq	.L1251
	ldr	w0, [x0,1868]
	bl	NandcSetDdrMode
.L1251:
	add	x20, x19, :lo12:.LANCHOR0
	mov	w1, 0
	add	x0, x20, 760
	mov	w2, 852
	bl	ftl_memset
	strb	wzr, [x20,16]
	ldrb	w2, [x20,1621]
	adrp	x1, .LANCHOR1
	add	x0, x1, :lo12:.LANCHOR1
	cmp	w2, 218
	add	x0, x0, 472
	cset	w3, eq
	str	x0, [x20,744]
	add	x20, x20, 1620
	cbnz	w3, .L1252
	cmp	w2, 241
	beq	.L1252
	cmp	w2, 220
	bne	.L1253
	ldrb	w0, [x20,3]
	cmp	w0, 149
	bne	.L1253
.L1252:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w4, 1
	mov	w5, 16
	add	x6, x1, :lo12:.LANCHOR1
	strb	w4, [x0,8]
	add	x4, x21, :lo12:.LANCHOR4
	strb	w5, [x0,9]
	strb	w5, [x4,74]
	add	x5, x0, 1620
	ldrb	w0, [x0,1620]
	strb	w0, [x6,2969]
	cmp	w0, 152
	bne	.L1254
	ldrsb	w0, [x5,4]
	tbnz	w0, #31, .L1254
	mov	w0, 24
	strb	w0, [x4,74]
.L1254:
	cbz	w3, .L1255
	add	x0, x1, :lo12:.LANCHOR1
	mov	w2, 2048
	strh	w2, [x0,2982]
	mov	w2, -38
	b	.L1321
.L1255:
	cmp	w2, 220
	bne	.L1256
	add	x0, x1, :lo12:.LANCHOR1
	mov	w2, 4096
	strh	w2, [x0,2982]
	mov	w2, -36
.L1321:
	strb	w2, [x0,2970]
.L1256:
	add	x20, x1, :lo12:.LANCHOR1
	add	x0, x19, :lo12:.LANCHOR0
	add	x1, x20, 2872
	mov	w2, 32
	add	x0, x0, 88
	bl	ftl_memcpy
	add	x0, x20, 472
	add	x1, x20, 2968
	mov	w2, 32
	bl	ftl_memcpy
.L1253:
	add	x20, x19, :lo12:.LANCHOR0
	ldrb	w0, [x20,8]
	cbnz	w0, .L1257
	bl	FlashLoadPhyInfoInRam
	cbnz	w0, .L1259
	ldr	x0, [x20,744]
	ldrb	w1, [x0,17]
	and	w0, w1, 7
	strb	w0, [x20,1844]
	tbnz	x1, 0, .L1259
	mov	w1, 1
	strb	w1, [x20,1864]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x20,1844]
	bl	NandcSetMode
.L1259:
	add	x20, x19, :lo12:.LANCHOR0
	ldr	x0, [x20,744]
	ldrb	w0, [x0,26]
	strb	w0, [x20,120]
	bl	FlashLoadPhyInfo
	cbz	w0, .L1257
	ldr	w0, [x20,1868]
	cbz	w0, .L1262
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	b	.L1322
.L1262:
	ldrb	w0, [x20,1844]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x20,1844]
.L1322:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cbz	w0, .L1257
	mov	w0, 1
	add	x20, x19, :lo12:.LANCHOR0
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	ldr	x1, [x20,744]
	adrp	x0, .LC92
	add	x0, x0, :lo12:.LC92
	ldrh	w1, [x1,14]
	bl	printk
	bl	FlashLoadPhyInfoInRam
	cmn	w0, #1
	beq	.L1249
	bl	FlashDieInfoInit
	ldr	x0, [x20,744]
	ldrb	w0, [x0,19]
	bl	FlashGetReadRetryDefault
	adrp	x1, .LANCHOR2+2022
	ldr	x0, [x20,744]
	ldrh	w1, [x1,#:lo12:.LANCHOR2+2022]
	ldrb	w2, [x0,9]
	add	w1, w1, 4095
	cmp	w2, w1, lsr 12
	blt	.L1264
	ldrh	w1, [x0,14]
	add	w1, w1, 255
	cmp	w2, w1, lsr 8
	bge	.L1265
.L1264:
	ldrh	w1, [x0,14]
	and	w1, w1, -256
	strh	w1, [x0,14]
.L1265:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,1844]
	tst	w0, 6
	beq	.L1266
	bl	FlashSavePhyInfo
	add	x1, x21, :lo12:.LANCHOR4
	mov	w0, 0
	ldr	w1, [x1,64]
	bl	FlashDdrParaScan
.L1266:
	bl	FlashSavePhyInfo
.L1257:
	add	x1, x19, :lo12:.LANCHOR0
	ldr	x3, [x1,744]
	ldrh	w0, [x3,16]
	ldrb	w4, [x3,26]
	ubfx	x2, x0, 7, 1
	strb	w2, [x1,16]
	add	x2, x21, :lo12:.LANCHOR4
	ubfx	x5, x0, 3, 1
	strb	w4, [x1,120]
	strb	w5, [x2,112]
	ubfx	x5, x0, 4, 1
	str	xzr, [x2,48]
	strb	w5, [x1,1945]
	ubfx	x5, x0, 8, 3
	strb	w5, [x1,1844]
	tbz	x0, 6, .L1268
	ldrb	w0, [x3,19]
	ldrb	w5, [x1,762]
	strb	w5, [x2,73]
	sub	w5, w0, #1
	ldrb	w6, [x1,761]
	uxtb	w5, w5
	strb	w0, [x1,752]
	strb	w6, [x1,753]
	cmp	w5, 5
	bhi	.L1269
	sub	w0, w0, #5
	adrp	x5, HynixReadRetrial
	add	x5, x5, :lo12:HynixReadRetrial
	str	x5, [x2,48]
	uxtb	w0, w0
	cmp	w0, 1
	bhi	.L1268
	mov	w0, 1
	str	w0, [x1,1936]
	b	.L1268
.L1269:
	sub	w1, w0, #17
	uxtb	w1, w1
	cmp	w1, 2
	bhi	.L1271
	adrp	x1, MicronReadRetrial
	cmp	w0, 19
	add	x1, x1, :lo12:MicronReadRetrial
	str	x1, [x2,48]
	beq	.L1272
	mov	w0, 7
	b	.L1323
.L1272:
	mov	w0, 15
.L1323:
	strb	w0, [x2,73]
	b	.L1268
.L1271:
	sub	w1, w0, #65
	uxtb	w1, w1
	cmp	w1, 1
	bls	.L1291
	cmp	w0, 33
	bne	.L1273
.L1291:
	add	x0, x21, :lo12:.LANCHOR4
	adrp	x1, ToshibaReadRetrial
	add	x1, x1, :lo12:ToshibaReadRetrial
	mov	w2, 4
	str	x1, [x0,48]
	add	x1, x19, :lo12:.LANCHOR0
	strb	w2, [x1,753]
	mov	w1, 7
	strb	w1, [x0,73]
	b	.L1268
.L1273:
	sub	w1, w0, #67
	uxtb	w1, w1
	cmp	w1, 1
	sub	w1, w0, #34
	cset	w5, ls
	uxtb	w1, w1
	cmp	w1, 1
	bls	.L1292
	cbz	w5, .L1275
.L1292:
	add	x2, x21, :lo12:.LANCHOR4
	adrp	x1, ToshibaReadRetrial
	add	x1, x1, :lo12:ToshibaReadRetrial
	cmp	w0, 35
	str	x1, [x2,48]
	beq	.L1277
	cmp	w0, 68
	beq	.L1277
	mov	w0, 7
	strb	w0, [x2,73]
	b	.L1278
.L1277:
	add	x0, x21, :lo12:.LANCHOR4
	mov	w1, 17
	strb	w1, [x0,73]
.L1278:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, 4
	cbnz	w5, .L1324
	mov	w1, 5
.L1324:
	strb	w1, [x0,753]
	b	.L1268
.L1275:
	cmp	w0, 49
	bne	.L1268
	adrp	x0, SamsungReadRetrial
	add	x0, x0, :lo12:SamsungReadRetrial
	str	x0, [x2,48]
.L1268:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w1, [x0,1876]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w1, w0
	bne	.L1280
	cbz	w4, .L1280
	strb	wzr, [x3,18]
.L1280:
	ldrh	w2, [x3,10]
	ldrb	w1, [x3,12]
	ldrb	w0, [x3,18]
	sdiv	w1, w2, w1
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x0,1620]
	cmp	w1, 44
	bne	.L1281
	ldrb	w1, [x0,1864]
	cbz	w1, .L1281
	ldr	w2, [x0,1876]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	bne	.L1282
	ldrb	w0, [x0,120]
	cbnz	w0, .L1281
.L1282:
	add	x0, x19, :lo12:.LANCHOR0
	strb	wzr, [x0,1864]
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
.L1281:
	mov	w0, 0
	bl	flash_enter_slc_mode
	add	x1, x19, :lo12:.LANCHOR0
	ldrb	w0, [x1,1844]
	tst	w0, 6
	beq	.L1283
	ldrb	w1, [x1,1864]
	cbnz	w1, .L1284
	tbnz	x0, 0, .L1283
.L1284:
	add	x1, x21, :lo12:.LANCHOR4
	mov	w0, 0
	ldr	w1, [x1,64]
	bl	FlashDdrParaScan
.L1283:
	mov	w0, 0
	add	x20, x19, :lo12:.LANCHOR0
	bl	flash_exit_slc_mode
	ldr	x0, [x20,744]
	ldrb	w0, [x0,20]
	bl	FlashBchSel
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x0, x0, 2336
	bl	FlashReadIdbDataRaw
	cbnz	w0, .L1285
	add	x21, x21, :lo12:.LANCHOR4
	ldrb	w1, [x20,9]
	ldrb	w0, [x21,72]
	cmp	w1, w0
	bls	.L1286
	strb	w0, [x20,9]
.L1286:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,9]
	cmp	w0, 15
	bhi	.L1287
.L1285:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, 16
	strb	w1, [x0,9]
.L1287:
	mov	w0, 18928
	add	x19, x19, :lo12:.LANCHOR0
	movk	w0, 0x2, lsl 16
	bl	FlashTimingCfg
	ldr	x1, [x19,744]
	ldrb	w2, [x1,12]
	strh	w2, [x19,176]
	ldrh	w4, [x1,14]
	ldrb	w2, [x1,7]
	str	w2, [x19,172]
	ldr	w2, [x19,1620]
	str	w2, [x19,168]
	ldrb	w2, [x19,1845]
	strh	w2, [x19,178]
	ldrb	w2, [x1,13]
	strh	w2, [x19,180]
	strh	w4, [x19,182]
	ldrh	w2, [x1,10]
	strh	w2, [x19,184]
	ldrb	w3, [x1,12]
	ldrh	w2, [x1,10]
	sdiv	w2, w2, w3
	strh	w2, [x19,186]
	ldrb	w3, [x1,9]
	strh	w3, [x19,188]
	ldrh	w5, [x1,10]
	ldrb	w2, [x1,9]
	mul	w2, w2, w5
	mov	w5, 512
	strh	w5, [x19,192]
	ldrb	w5, [x19,9]
	strh	w5, [x19,194]
	uxth	w2, w2
	ldrb	w5, [x19,8]
	strh	w2, [x19,190]
	cmp	w5, 1
	bne	.L1288
	lsl	w2, w2, 1
	mov	w5, 16
	lsr	w4, w4, 1
	lsl	w3, w3, 1
	strh	w2, [x19,190]
	mov	w2, 8
	strb	w5, [x19,9]
	strh	w4, [x19,182]
	strh	w3, [x19,188]
	strh	w2, [x19,194]
.L1288:
	ldrb	w0, [x1,20]
	bl	FlashBchSel
	bl	FlashSuspend
	mov	w0, 0
	b	.L1249
.L1290:
	mov	w0, -2
.L1249:
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
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	adrp	x23, .LANCHOR0
	stp	x21, x22, [sp,32]
	uxtb	w21, w0
	add	x0, x23, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	uxth	w22, w2
	ldr	x2, [x0,744]
	ldrb	w24, [x2,19]
	ldrb	w2, [x0,120]
	cbz	w2, .L1326
	ldr	w2, [x0,1876]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w2, w0
	beq	.L1325
.L1326:
	sub	w0, w24, #5
	uxtb	w0, w0
	cmp	w0, 1
	bls	.L1328
	cmp	w24, 68
	beq	.L1328
	cmp	w24, 35
	beq	.L1328
	sub	w0, w24, #17
	uxtb	w0, w0
	cmp	w0, 2
	bhi	.L1325
.L1328:
	mov	w27, w1
	sub	w24, w24, #17
	add	x23, x23, :lo12:.LANCHOR0
	add	x26, x23, 24
	add	x26, x26, x21, sxtw 4
.L1331:
	ldr	x0, [x23,744]
	ldrh	w0, [x0,10]
	cmp	w0, w22
	bls	.L1325
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x0, x0, 996
	ldrh	w1, [x0,w22,sxtw 1]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L1325
	adrp	x25, .LANCHOR4
	mov	w1, 255
	add	x19, x25, :lo12:.LANCHOR4
	mov	w2, 32768
	ldr	x0, [x19,80]
	bl	ftl_memset
	uxtb	w0, w24
	cmp	w0, 1
	bhi	.L1332
	adrp	x28, .LANCHOR1
	add	x28, x28, :lo12:.LANCHOR1
	ldr	w1, [x28,3000]
	cbz	w1, .L1333
	ldrb	w20, [x26,8]
	mov	w0, w21
	ldr	x19, [x26]
	ubfiz	x20, x20, 8, 8
	str	x1, [x29,104]
	add	x19, x19, x20
	bl	NandcFlashCs
	mov	w0, 239
	mov	w20, 146
	str	w0, [x19,2056]
	mov	w0, w21
	str	w20, [x19,2052]
	str	wzr, [x19,2048]
	str	wzr, [x19,2048]
	str	wzr, [x19,2048]
	str	wzr, [x19,2048]
	bl	NandcFlashDeCs
	ldr	x1, [x29,104]
	cmp	w1, 1
	bne	.L1334
	mov	w0, w21
	bl	NandcFlashCs
	mov	w0, 238
	str	w0, [x19,2056]
	str	w20, [x19,2052]
	mov	w0, 2000
	bl	NandcDelayns
	ldr	w20, [x19,2048]
	mov	w0, w21
	uxtb	w20, w20
	bl	NandcFlashDeCs
	cbnz	w20, .L1335
	mov	w0, 2
	str	w0, [x28,3000]
	b	.L1334
.L1335:
	str	wzr, [x28,3000]
	b	.L1325
.L1334:
	add	x25, x25, :lo12:.LANCHOR4
	add	w1, w22, w27
	mov	x3, 0
	mov	w0, w21
	ldr	x2, [x25,80]
	bl	FlashProgPage
	mov	w0, w21
	bl	NandcFlashCs
	mov	w0, 239
	str	w0, [x19,2056]
	mov	w0, 146
	str	w0, [x19,2052]
	mov	w0, 1
	str	w0, [x19,2048]
	str	wzr, [x19,2048]
	mov	w0, w21
	str	wzr, [x19,2048]
	str	wzr, [x19,2048]
	bl	NandcFlashDeCs
	b	.L1333
.L1332:
	ldr	x2, [x19,80]
	mov	w0, w21
	add	w1, w22, w27
	mov	x3, 0
	bl	FlashProgPage
.L1333:
	add	w22, w22, 1
	uxth	w22, w22
	b	.L1331
.L1325:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FlashPageProgMsbFFData, .-FlashPageProgMsbFFData
	.align	2
	.global	FlashReadSlc2KPages
	.type	FlashReadSlc2KPages, %function
FlashReadSlc2KPages:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	mov	w25, w1
	adrp	x1, .LANCHOR1+481
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	adrp	x21, .LANCHOR0
	adrp	x23, .LC93
	adrp	x24, .LC95
	ldrb	w26, [x1,#:lo12:.LANCHOR1+481]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	mov	x19, x0
	mov	w27, w2
	mov	w22, 0
	add	x21, x21, :lo12:.LANCHOR0
	add	x23, x23, :lo12:.LC93
	add	x24, x24, :lo12:.LC95
.L1347:
	cmp	w22, w25
	beq	.L1381
	mov	w1, w27
	sub	w4, w25, w22
	mov	x0, x19
	add	x2, x29, 124
	add	x3, x29, 120
	bl	LogAddr2PhyAddr
	ldr	w0, [x29,120]
	adrp	x5, .LANCHOR0
	ldrb	w1, [x21,1845]
	cmp	w0, w1
	bcc	.L1348
	mov	w0, -1
	str	w0, [x19]
	b	.L1349
.L1348:
	add	x0, x21, x0, uxtw
	ldrb	w28, [x0,1848]
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
	mov	w2, w26
	mov	w0, w28
	bl	NandcXferData
	mov	w6, w0
	ldr	w2, [x21,12]
	mov	w0, w28
	ldr	w1, [x29,124]
	str	x6, [x29,104]
	add	w1, w2, w1
	bl	FlashReadCmd
	ldr	x0, [x19,8]
	mov	w1, 0
	mov	w2, w26
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
	ldrb	w0, [x0,1944]
	add	w0, w0, w0, lsl 1
	cmp	w3, w0, lsr 2
	bls	.L1352
	cmn	w3, #1
	mov	w0, 256
	csel	w3, w3, w0, eq
.L1352:
	cmp	w3, 256
	beq	.L1363
	cmn	w3, #1
	bne	.L1353
.L1363:
	str	w3, [x19]
	b	.L1355
.L1353:
	str	wzr, [x19]
.L1355:
	ldr	x0, [x19,16]
	cbz	x0, .L1356
	ldr	w1, [x0,8]
	cmn	w1, #1
	bne	.L1356
	ldr	w0, [x0]
	cmn	w0, #1
	beq	.L1356
	str	w1, [x19]
.L1356:
	ldr	w3, [x19]
	cmn	w3, #1
	bne	.L1349
	add	x5, x5, :lo12:.LANCHOR0
	ldr	w1, [x19,4]
	mov	x0, x23
	ldrb	w2, [x5,1944]
	bl	printk
	ldr	x1, [x19,8]
	cbz	x1, .L1358
	adrp	x0, .LC94
	mov	w2, 4
	add	x0, x0, :lo12:.LC94
	mov	w3, 8
	bl	rknand_print_hex
.L1358:
	ldr	x1, [x19,16]
	cbz	x1, .L1349
	mov	w2, 4
	mov	x0, x24
	mov	w3, w2
	bl	rknand_print_hex
.L1349:
	add	w22, w22, 1
	add	x19, x19, 56
	b	.L1347
.L1381:
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
	stp	x29, x30, [sp, -176]!
	adrp	x3, .LANCHOR1+481
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	ldrb	w3, [x3,#:lo12:.LANCHOR1+481]
	adrp	x22, .LANCHOR0
	str	w3, [x29,148]
	add	x3, x22, :lo12:.LANCHOR0
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	str	w1, [x29,144]
	mov	x26, x0
	ldrb	w4, [x3,16]
	ldrb	w23, [x3,8]
	str	w2, [x29,140]
	str	w4, [x29,136]
	cbz	w23, .L1421
	bl	FlashReadSlc2KPages
	b	.L1470
.L1421:
	adrp	x0, .LC93
	mov	w24, w23
	add	x0, x0, :lo12:.LC93
	str	x0, [x29,128]
	adrp	x0, .LC95
	add	x0, x0, :lo12:.LC95
	str	x0, [x29,120]
	adrp	x0, .LC96
	add	x0, x0, :lo12:.LC96
	str	x0, [x29,112]
.L1383:
	ldr	w0, [x29,144]
	cmp	w24, w0
	bcs	.L1472
	mov	w27, 56
	ldr	w0, [x29,144]
	ldr	w1, [x29,140]
	add	x2, x29, 172
	umull	x27, w24, w27
	sub	w4, w0, w24
	add	x3, x29, 168
	add	x21, x26, x27
	mov	x0, x21
	ldr	w28, [x21,4]
	bl	LogAddr2PhyAddr
	add	x2, x22, :lo12:.LANCHOR0
	mov	w20, w0
	ldr	w0, [x29,168]
	ldrb	w1, [x2,1845]
	cmp	w0, w1
	bcc	.L1385
	mov	w0, -1
	str	w0, [x26,x27]
	b	.L1386
.L1385:
	add	x0, x2, x0, uxtw
	adrp	x25, .LANCHOR4
	ldrb	w19, [x0,1848]
	add	x0, x25, :lo12:.LANCHOR4
	str	x2, [x29,152]
	ldrb	w0, [x0,112]
	cmp	w0, wzr
	mov	w0, w19
	csel	w20, w20, wzr, ne
	bl	NandcWaitFlashReady
	ldr	x2, [x29,152]
	ldr	x0, [x2,744]
	ldrb	w0, [x0,19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 5
	bhi	.L1388
	sxtw	x0, w19
	add	x1, x2, 760
	add	x3, x1, x0
	add	x0, x2, x0
	ldrb	w3, [x3,12]
	ldrb	w0, [x0,1616]
	cmp	w0, w3
	beq	.L1388
	ldrb	w1, [x2,761]
	mov	w0, w19
	add	x2, x2, 764
	bl	HynixSetRRPara
.L1388:
	mov	w0, w19
	bl	NandcFlashCs
	ldr	w0, [x29,140]
	cmp	w0, 1
	cset	w0, eq
	orr	w0, w0, w28, lsr 31
	str	w0, [x29,152]
	cbz	w0, .L1389
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1389
	mov	w0, w19
	bl	flash_enter_slc_mode
	b	.L1390
.L1389:
	mov	w0, w19
	bl	flash_exit_slc_mode
.L1390:
	cmp	w19, 255
	ldr	w1, [x29,172]
	bne	.L1428
	cmn	w1, #1
	cset	w0, ne
	cbz	w0, .L1422
.L1428:
	cbz	w20, .L1394
	add	x0, x22, :lo12:.LANCHOR0
	ldr	w2, [x0,12]
	mov	w0, w19
	add	w2, w1, w2
	bl	FlashReadDpCmd
	b	.L1395
.L1394:
	mov	w0, w19
	bl	FlashReadCmd
	b	.L1395
.L1422:
	mov	w20, w0
.L1392:
	ldrb	w2, [x29,148]
	mov	w0, w19
	ldr	x3, [x21,8]
	mov	w1, 0
	ldr	x4, [x21,16]
	bl	NandcXferData
	cmn	w0, #1
	mov	w28, w0
	bne	.L1396
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w1, [x0,16]
	cbz	w1, .L1396
	strb	wzr, [x0,16]
	mov	w20, 0
	b	.L1390
.L1396:
	cbz	w20, .L1397
	add	x0, x22, :lo12:.LANCHOR0
	ldr	w1, [x29,172]
	ldr	w2, [x0,12]
	mov	w0, w19
	add	w1, w2, w1
	bl	FlashReadDpDataOutCmd
	add	w0, w24, 1
	mov	w4, 56
	ldrb	w2, [x29,148]
	mov	w1, 0
	umull	x4, w0, w4
	mov	w0, w19
	add	x4, x26, x4
	ldr	x3, [x4,8]
	ldr	x4, [x4,16]
	bl	NandcXferData
	cmn	w0, #1
	mov	w23, w0
	csel	w20, w20, wzr, ne
.L1397:
	mov	w0, w19
	bl	NandcFlashDeCs
	add	x6, x22, :lo12:.LANCHOR0
	ldrb	w0, [x29,136]
	cmn	w28, #1
	strb	w0, [x6,16]
	bne	.L1404
	ldrb	w0, [x6,1864]
	cbnz	w0, .L1399
.L1403:
	add	x0, x25, :lo12:.LANCHOR4
	ldr	x20, [x0,48]
	cbnz	x20, .L1400
	b	.L1473
.L1399:
	ldr	x0, [x6,128]
	mov	w4, 1
	ldr	w1, [x29,172]
	ldr	x2, [x21,8]
	ldr	x3, [x21,16]
	ldr	w20, [x0,304]
	mov	w0, w19
	str	x6, [x29,104]
	bl	FlashDdrTunningRead
	cmn	w0, #1
	mov	w28, w0
	beq	.L1402
	ldr	x6, [x29,104]
	ldrb	w0, [x6,1944]
	cmp	w28, w0, lsr 1
	bls	.L1424
.L1402:
	lsr	w0, w20, 8
	bl	NandcSetDdrPara
	cmn	w28, #1
	beq	.L1403
	b	.L1424
.L1400:
	ldr	w1, [x29,172]
	mov	w0, w19
	ldr	x2, [x21,8]
	ldr	x3, [x21,16]
	blr	x20
	cmn	w0, #1
	mov	w28, w0
	mov	w20, 0
	bne	.L1405
	add	x2, x22, :lo12:.LANCHOR0
	ldr	x0, [x2,744]
	ldrb	w0, [x0,19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 5
	bhi	.L1406
	ldrb	w1, [x2,761]
	mov	w0, w19
	add	x2, x2, 764
	mov	w3, w20
	bl	HynixSetRRPara
.L1406:
	ldr	w1, [x29,172]
	mov	w0, w19
	ldr	x2, [x21,8]
	mov	w20, 0
	ldr	x3, [x21,16]
	bl	FlashReadRawPage
	mov	w28, w0
	add	x4, x22, :lo12:.LANCHOR0
	ldr	x0, [x29,112]
	ldr	w1, [x21,4]
	mov	w3, w28
	str	x4, [x29,104]
	ldrb	w2, [x4,1944]
	bl	printk
	cmn	w28, #1
	ldr	x4, [x29,104]
	bne	.L1405
	ldrb	w0, [x4,120]
	cbz	w0, .L1405
	ldr	w0, [x29,152]
	cbz	w0, .L1407
	mov	w0, w19
	bl	flash_enter_slc_mode
	b	.L1408
.L1407:
	mov	w0, w19
	bl	flash_exit_slc_mode
.L1408:
	add	x0, x25, :lo12:.LANCHOR4
	ldr	w1, [x29,172]
	ldr	x2, [x21,8]
	mov	w20, 0
	ldr	x3, [x21,16]
	ldr	x4, [x0,48]
	mov	w0, w19
	blr	x4
	mov	w28, w0
	b	.L1405
.L1473:
	ldr	w1, [x29,172]
	mov	w0, w19
	ldr	x2, [x21,8]
	ldr	x3, [x21,16]
	bl	FlashReadRawPage
	mov	w28, w0
	b	.L1405
.L1424:
	mov	w20, 0
.L1404:
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,1944]
	add	w0, w0, w0, lsl 1
	cmp	w28, w0, lsr 2
	bls	.L1405
	add	x0, x25, :lo12:.LANCHOR4
	ldr	x0, [x0,48]
	cmp	x0, xzr
	mov	w0, 256
	csel	w28, w28, w0, ne
.L1405:
	cmp	w28, 256
	beq	.L1429
	cmn	w28, #1
	bne	.L1409
.L1429:
	str	w28, [x26,x27]
	b	.L1411
.L1409:
	str	wzr, [x26,x27]
.L1411:
	ldr	w3, [x26,x27]
	cmn	w3, #1
	bne	.L1413
	add	x2, x22, :lo12:.LANCHOR0
	ldr	w1, [x21,4]
	ldr	x0, [x29,128]
	ldrb	w2, [x2,1944]
	bl	printk
	ldr	x1, [x21,16]
	cbz	x1, .L1413
	mov	w2, 4
	ldr	x0, [x29,120]
	mov	w3, w2
	bl	rknand_print_hex
.L1413:
	cbz	w20, .L1415
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,1944]
	add	w0, w0, w0, lsl 1
	cmp	w23, w0, lsr 2
	bls	.L1416
	add	x25, x25, :lo12:.LANCHOR4
	ldr	x0, [x25,48]
	cmp	x0, xzr
	mov	w0, 256
	csel	w23, w23, w0, ne
.L1416:
	add	w0, w24, 1
	mov	w1, 56
	cmp	w23, 256
	umull	x0, w0, w1
	beq	.L1430
	cmn	w23, #1
	bne	.L1417
.L1430:
	str	w23, [x26,x0]
	b	.L1415
.L1417:
	str	wzr, [x26,x0]
.L1415:
	ldr	w0, [x29,152]
	add	w24, w24, w20
	cbz	w0, .L1386
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1386
	mov	w0, w19
	bl	flash_exit_slc_mode
.L1386:
	add	w24, w24, 1
	b	.L1383
.L1472:
	mov	w0, 0
	b	.L1470
.L1395:
	mov	w0, w19
	bl	NandcWaitFlashReady
	cbz	w20, .L1392
	ldr	w1, [x29,172]
	mov	w0, w19
	bl	FlashReadDpDataOutCmd
	b	.L1392
.L1470:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 176
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
	adrp	x23, .LANCHOR4
	mov	w27, -1
	ldrh	w1, [x0,180]
	add	x19, x23, :lo12:.LANCHOR4
	ldrh	w21, [x0,182]
	add	x0, x19, 120
	mov	w28, 0
	adrp	x26, .LC97
	add	x26, x26, :lo12:.LC97
	mul	w21, w1, w21
	mov	w1, 0
	uxth	w21, w21
	bl	ftl_memset
	add	w25, w21, w27
	ldr	x0, [x19,88]
	str	x0, [x29,152]
	mov	w19, w28
	sub	w0, w21, #12
	str	xzr, [x29,144]
	uxth	w25, w25
	str	w0, [x29,124]
.L1475:
	add	x1, x22, :lo12:.LANCHOR0
	ldrb	w1, [x1,1845]
	cmp	w1, w19
	bls	.L1485
	mul	w3, w19, w21
	mov	w20, w25
	mov	w4, 61664
.L1476:
	ldr	w0, [x29,124]
	cmp	w20, w0
	ble	.L1478
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
	beq	.L1477
	add	x24, x23, :lo12:.LANCHOR4
	ldr	x0, [x24,88]
	ldrh	w0, [x0]
	cmp	w0, w4
	bne	.L1477
	add	x24, x24, 120
	mov	x0, x26
	mov	w1, w19
	mov	w2, w20
	add	w28, w28, 1
	bl	printk
	strh	w20, [x24,w19,sxtw 1]
	uxth	w28, w28
	b	.L1478
.L1477:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L1476
.L1478:
	add	x1, x22, :lo12:.LANCHOR0
	add	w19, w19, 1
	uxtb	w19, w19
	ldrb	w1, [x1,1845]
	cmp	w1, w28
	csel	w27, w27, wzr, ne
	b	.L1475
.L1485:
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
	.global	FlashProgSlc2KPages
	.type	FlashProgSlc2KPages, %function
FlashProgSlc2KPages:
	stp	x29, x30, [sp, -160]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	mov	w23, w1
	adrp	x1, .LANCHOR1+481
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	adrp	x22, .LANCHOR0
	ldrb	w26, [x1,#:lo12:.LANCHOR1+481]
	stp	x27, x28, [sp,80]
	stp	x19, x20, [sp,16]
	mov	w24, w2
	mov	w27, w3
	mov	x19, x0
	mov	x21, x0
	mov	w25, 0
	add	x22, x22, :lo12:.LANCHOR0
	mov	w28, -1
.L1487:
	cmp	w25, w23
	beq	.L1516
	mov	w1, w24
	sub	w4, w23, w25
	mov	x0, x21
	add	x2, x29, 96
	add	x3, x29, 100
	bl	LogAddr2PhyAddr
	ldr	w0, [x29,100]
	ldrb	w1, [x22,1845]
	cmp	w0, w1
	bcc	.L1488
	str	w28, [x21]
	b	.L1489
.L1488:
	add	x0, x22, x0, uxtw
	ldrb	w20, [x0,1848]
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
	ldr	w2, [x22,12]
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
	ldr	w2, [x0,12]
	mov	w0, w20
	add	w1, w2, w1
	bl	FlashProgSecondCmd
	mov	w0, w20
	bl	NandcWaitFlashReady
	ldr	w1, [x29,96]
	mov	w0, w20
	bl	FlashReadStatus
	tbz	x0, 0, .L1492
	str	w28, [x21]
.L1492:
	mov	w0, w20
	bl	NandcFlashDeCs
.L1489:
	add	w25, w25, 1
	add	x21, x21, 56
	b	.L1487
.L1516:
	mov	w21, 0
	cbz	w27, .L1514
	adrp	x25, .LC101
	adrp	x26, .LC100
	adrp	x27, .LC99
	add	x25, x25, :lo12:.LC101
	add	x26, x26, :lo12:.LC100
	add	x27, x27, :lo12:.LC99
.L1515:
	cmp	w21, w23
	beq	.L1514
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1495
	adrp	x0, .LC98
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC98
	bl	printk
	b	.L1496
.L1495:
	adrp	x20, .LANCHOR4
	sub	w4, w23, w21
	add	x22, x20, :lo12:.LANCHOR4
	add	x3, x29, 100
	mov	w1, w24
	add	x2, x29, 96
	mov	x0, x19
	bl	LogAddr2PhyAddr
	ldr	x0, [x22,96]
	mov	x1, x19
	mov	x2, 56
	str	wzr, [x0]
	ldr	x0, [x22,104]
	str	wzr, [x0]
	add	x0, x29, 104
	bl	memcpy
	ldr	x0, [x22,96]
	mov	w1, 1
	str	x0, [x29,112]
	mov	w2, w24
	ldr	x0, [x22,104]
	str	x0, [x29,120]
	add	x0, x29, 104
	bl	FlashReadPages
	ldr	w22, [x29,104]
	cmn	w22, #1
	bne	.L1497
	ldr	w1, [x19,4]
	mov	x0, x27
	bl	printk
	str	w22, [x19]
.L1497:
	ldr	x0, [x19,16]
	cbz	x0, .L1498
	ldr	w2, [x0]
	add	x0, x20, :lo12:.LANCHOR4
	ldr	x0, [x0,104]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1498
	ldr	w1, [x19,4]
	mov	x0, x26
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1498:
	ldr	x0, [x19,8]
	cbz	x0, .L1496
	add	x20, x20, :lo12:.LANCHOR4
	ldr	w2, [x0]
	ldr	x0, [x20,96]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1496
	ldr	w1, [x19,4]
	mov	x0, x25
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1496:
	add	w21, w21, 1
	add	x19, x19, 56
	b	.L1515
.L1514:
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
	.global	FtlLoadFactoryBbt
	.type	FtlLoadFactoryBbt, %function
FtlLoadFactoryBbt:
	stp	x29, x30, [sp, -80]!
	adrp	x0, .LANCHOR2
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR2
	add	x19, x19, :lo12:.LANCHOR4
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x23, x24, [sp,48]
	add	x19, x19, 136
	adrp	x22, .LANCHOR0
	ldr	x1, [x0,-64]
	add	x21, x22, :lo12:.LANCHOR0
	ldr	x24, [x0,2112]
	add	x21, x21, 2112
	str	x1, [x19,8]
	mov	w23, 0
	str	x24, [x19,16]
	mov	w25, -1
	mov	w26, 61664
.L1518:
	add	x0, x22, :lo12:.LANCHOR0
	ldrh	w1, [x0,1974]
	cmp	w23, w1
	bcs	.L1527
	ldrh	w20, [x0,2022]
	strh	w25, [x21,12]
.L1520:
	add	x0, x22, :lo12:.LANCHOR0
	sub	w20, w20, #1
	uxth	w20, w20
	ldrh	w0, [x0,2022]
	sub	w1, w0, #16
	cmp	w20, w1
	ble	.L1521
	madd	w0, w23, w0, w20
	mov	w1, 1
	lsl	w0, w0, 10
	mov	w2, w1
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L1520
	ldrh	w0, [x24]
	cmp	w0, w26
	bne	.L1520
	strh	w20, [x21,12]
.L1521:
	add	w23, w23, 1
	add	x21, x21, 2
	b	.L1518
.L1527:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlLoadFactoryBbt, .-FtlLoadFactoryBbt
	.align	2
	.global	FtlGetLastWrittenPage
	.type	FtlGetLastWrittenPage, %function
FtlGetLastWrittenPage:
	stp	x29, x30, [sp, -208]!
	cmp	w1, 1
	adrp	x3, .LANCHOR0
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	str	x25, [sp,64]
	mov	w23, w1
	add	x3, x3, :lo12:.LANCHOR0
	bne	.L1529
	ldrh	w19, [x3,2028]
	b	.L1530
.L1529:
	ldrh	w19, [x3,2026]
.L1530:
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
	bne	.L1531
	mov	w22, 0
	mov	w25, 2
.L1532:
	cmp	w22, w19
	bgt	.L1531
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
	bne	.L1533
	ldr	w0, [x24,4]
	cmn	w0, #1
	bne	.L1533
	ldr	w0, [x29,88]
	cmn	w0, #1
	beq	.L1533
	sub	w19, w20, #1
	sxth	w19, w19
	b	.L1532
.L1533:
	add	w20, w20, 1
	sxth	w22, w20
	b	.L1532
.L1531:
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
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	adrp	x23, .LANCHOR4
	adrp	x24, .LANCHOR2
	stp	x19, x20, [sp,16]
	add	x0, x24, :lo12:.LANCHOR2
	add	x19, x23, :lo12:.LANCHOR4
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	add	x19, x19, 136
	adrp	x20, .LANCHOR0
	ldr	x22, [x0,2112]
	mov	w25, 61649
	str	xzr, [x19,8]
	str	x22, [x19,16]
	bl	FtlBbtMemInit
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w21, [x0,2022]
	sub	w21, w21, #1
	uxth	w21, w21
.L1540:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w0, [x0,2022]
	sub	w0, w0, #48
	cmp	w21, w0
	ble	.L1543
	lsl	w0, w21, 10
	mov	w1, 1
	str	w0, [x19,4]
	mov	w2, w1
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1541
	ldr	w0, [x19,4]
	mov	w1, 1
	mov	w2, w1
	add	w0, w0, 1
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
.L1541:
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L1542
	ldrh	w0, [x22]
	cmp	w0, w25
	bne	.L1542
	add	x1, x20, :lo12:.LANCHOR0
	add	x0, x1, 2112
	strh	w21, [x1,2112]
	ldr	w1, [x22,4]
	str	w1, [x0,8]
	ldrh	w1, [x22,8]
	strh	w1, [x0,4]
	b	.L1543
.L1542:
	sub	w21, w21, #1
	uxth	w21, w21
	b	.L1540
.L1543:
	add	x21, x20, :lo12:.LANCHOR0
	mov	w2, 65535
	mov	w0, -1
	ldrh	w1, [x21,2112]
	cmp	w1, w2
	beq	.L1545
	ldrh	w1, [x21,2116]
	cmp	w1, w2
	beq	.L1547
	add	x25, x23, :lo12:.LANCHOR4
	lsl	w1, w1, 10
	add	x0, x25, 136
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	bl	FlashReadPages
	ldr	w0, [x25,136]
	cmn	w0, #1
	beq	.L1547
	ldrh	w1, [x22]
	mov	w0, 61649
	cmp	w1, w0
	bne	.L1547
	ldr	w0, [x22,4]
	ldr	w1, [x21,2120]
	cmp	w0, w1
	bls	.L1547
	str	w0, [x21,2120]
	ldrh	w1, [x21,2116]
	ldrh	w0, [x22,8]
	strh	w1, [x21,2112]
	strh	w0, [x21,2116]
.L1547:
	add	x25, x20, :lo12:.LANCHOR0
	mov	w1, 1
	add	x21, x23, :lo12:.LANCHOR4
	mov	w26, 61649
	add	x21, x21, 136
	ldrh	w0, [x25,2112]
	bl	FtlGetLastWrittenPage
	sxth	w19, w0
	add	w0, w0, 1
	strh	w0, [x25,2114]
.L1549:
	tbnz	w19, #31, .L1554
	ldrh	w0, [x25,2112]
	mov	w1, 1
	mov	w2, w1
	orr	w0, w19, w0, lsl 10
	str	w0, [x21,4]
	add	x0, x24, :lo12:.LANCHOR2
	ldr	x0, [x0,-64]
	str	x0, [x21,8]
	mov	x0, x21
	bl	FlashReadPages
	ldr	w0, [x21]
	cmn	w0, #1
	beq	.L1550
	ldrh	w0, [x22]
	cmp	w0, w26
	bne	.L1550
.L1554:
	add	x1, x20, :lo12:.LANCHOR0
	ldrh	w0, [x22,10]
	mov	w2, 65535
	strh	w0, [x1,2118]
	ldrh	w0, [x22,12]
	cmp	w0, w2
	bne	.L1551
	b	.L1552
.L1550:
	sub	w19, w19, #1
	sxth	w19, w19
	b	.L1549
.L1551:
	ldr	w2, [x1,1948]
	cmp	w0, w2
	beq	.L1552
	ldrh	w1, [x1,1962]
	lsr	w1, w1, 2
	cmp	w0, w1
	bcs	.L1552
	cmp	w2, w1
	bcs	.L1552
	bl	FtlSysBlkNumInit
.L1552:
	add	x20, x20, :lo12:.LANCHOR0
	mov	x19, 0
	add	x21, x20, 2112
	add	x23, x23, :lo12:.LANCHOR4
.L1555:
	ldrh	w0, [x20,1974]
	mov	w1, w19
	add	x19, x19, 1
	cmp	w1, w0
	bcs	.L1573
	add	x0, x24, :lo12:.LANCHOR2
	ldr	x3, [x23,144]
	ldrh	w2, [x0,2232]
	add	x0, x21, x19, lsl 3
	mul	w1, w1, w2
	ldr	x0, [x0,24]
	lsl	w2, w2, 2
	add	x1, x3, x1, lsl 2
	bl	ftl_memcpy
	b	.L1555
.L1573:
	mov	w0, 0
.L1545:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlLoadBbt, .-FtlLoadBbt
	.align	2
	.global	load_l2p_region
	.type	load_l2p_region, %function
load_l2p_region:
	stp	x29, x30, [sp, -48]!
	uxth	x1, w1
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	add	x20, x20, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	uxth	w21, w0
	ubfiz	x0, x21, 2, 16
	ldr	x2, [x20,2208]
	ldr	w22, [x2,x0]
	cbnz	w22, .L1575
	ldr	x0, [x20,144]
	lsl	x19, x1, 4
	adrp	x2, .LANCHOR0+2036
	mov	w1, 255
	add	x0, x0, x19
	ldrh	w2, [x2,#:lo12:.LANCHOR0+2036]
	ldr	x0, [x0,8]
	bl	ftl_memset
	ldr	x0, [x20,144]
	strh	w21, [x0,x19]
	ldr	x1, [x20,144]
	add	x19, x1, x19
	str	w22, [x19,4]
	b	.L1576
.L1575:
	lsl	x19, x1, 4
	ldr	x1, [x20,144]
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	add	x1, x1, x19
	add	x0, x0, 136
	ldr	x1, [x1,8]
	str	x1, [x0,8]
	ldr	x1, [x20,2112]
	str	x1, [x0,16]
	mov	w1, 1
	mov	w2, w1
	str	w22, [x0,4]
	bl	FlashReadPages
	ldr	x0, [x20,144]
	add	x0, x0, x19
	str	wzr, [x0,4]
	ldr	x0, [x20,144]
	strh	w21, [x0,x19]
.L1576:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	load_l2p_region, .-load_l2p_region
	.align	2
	.global	FtlVendorPartRead
	.type	FtlVendorPartRead, %function
FtlVendorPartRead:
	stp	x29, x30, [sp, -176]!
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	adrp	x25, .LANCHOR0
	stp	x21, x22, [sp,32]
	mov	w22, w0
	add	x0, x25, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	mov	w21, w1
	add	w1, w22, w1
	ldrh	w28, [x0,2034]
	mov	x24, x2
	ldrh	w0, [x0,2020]
	mov	w20, -1
	cmp	w1, w0
	bhi	.L1578
	lsr	w28, w22, w28
	mov	w20, 0
	adrp	x26, .LANCHOR2
.L1579:
	cbz	w21, .L1578
	add	x3, x26, :lo12:.LANCHOR2
	ldr	x0, [x3,2200]
	ldr	w1, [x0,w28,uxtw 2]
	add	x0, x25, :lo12:.LANCHOR0
	ldrh	w19, [x0,2032]
	uxth	w0, w21
	udiv	w23, w22, w19
	msub	w23, w23, w19, w22
	sub	w19, w19, w23
	uxth	w19, w19
	cmp	w19, w21
	csel	w19, w0, w19, hi
	lsl	w27, w19, 9
	cbz	w1, .L1581
	ldr	x0, [x3,2080]
	ubfiz	x23, x23, 9, 16
	str	w1, [x29,124]
	mov	w1, 1
	str	x0, [x29,128]
	mov	w2, w1
	add	x0, x29, 120
	str	x3, [x29,104]
	str	xzr, [x29,136]
	bl	FlashReadPages
	ldr	x3, [x29,104]
	mov	w2, w27
	ldr	w0, [x29,120]
	ldr	x1, [x3,2080]
	cmn	w0, #1
	mov	x0, x24
	csinv	w20, w20, wzr, ne
	add	x1, x1, x23
	bl	ftl_memcpy
	b	.L1583
.L1581:
	mov	x0, x24
	mov	w2, w27
	bl	ftl_memset
.L1583:
	add	w28, w28, 1
	sub	w21, w21, w19
	add	w22, w22, w19
	add	x24, x24, x27, sxtw
	b	.L1579
.L1578:
	mov	w0, w20
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 176
	ret
	.size	FtlVendorPartRead, .-FtlVendorPartRead
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
	ldrh	w1, [x19,2136]
	ldr	x2, [x19,2152]
	bl	FtlVendorPartRead
	ldr	x0, [x19,2152]
	ldr	w1, [x0]
	mov	w0, 17221
	movk	w0, 0x4254, lsl 16
	cmp	w1, w0
	beq	.L1587
	adrp	x0, .LC72
	adrp	x1, .LC102
	add	x1, x1, :lo12:.LC102
	add	x0, x0, :lo12:.LC72
	bl	printk
	ldrh	w2, [x19,2136]
	mov	w1, 0
	ldr	x0, [x19,2152]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1587:
	mov	w0, 0
	ldr	x19, [sp,16]
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
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	x21, [sp,32]
	add	x20, x19, :lo12:.LANCHOR2
	add	x21, x20, 352
	mov	x2, x21
	bl	FtlVendorPartRead
	ldr	w1, [x20,352]
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	cmp	w1, w0
	beq	.L1589
	mov	x0, x21
	mov	w1, 0
	mov	w2, 512
	bl	ftl_memset
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	str	w0, [x20,352]
.L1589:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, 19539
	movk	w2, 0x4654, lsl 16
	ldr	w3, [x0,352]
	cmp	w3, w2
	bne	.L1590
	ldr	w2, [x0,440]
	str	w2, [x0,188]
	ldr	w2, [x0,444]
	str	w2, [x0,192]
	ldr	w2, [x0,360]
	str	w2, [x0,184]
	ldr	w2, [x0,364]
	str	w2, [x0,172]
	ldr	w2, [x0,368]
	str	w2, [x0,164]
	ldr	w2, [x0,372]
	str	w2, [x0,180]
	ldr	w2, [x0,380]
	str	w2, [x0,208]
	ldr	w2, [x0,384]
	str	w2, [x0,-72]
	ldr	w2, [x0,388]
	str	w2, [x0,168]
	ldr	w2, [x0,392]
	str	w2, [x0,176]
	ldr	w2, [x0,396]
	str	w2, [x0,216]
	ldr	w1, [x0,412]
	ldr	w2, [x0,400]
	str	w2, [x0,220]
	str	w1, [x0,344]
.L1590:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 34661
	movk	w1, 0x1234, lsl 16
	ldr	w2, [x0,420]
	str	wzr, [x0,2024]
	cmp	w2, w1
	bne	.L1591
	mov	w1, 1
	str	w1, [x0,160]
	adrp	x0, .LC72
	adrp	x1, .LC103
	add	x0, x0, :lo12:.LC72
	add	x1, x1, :lo12:.LC103
	bl	printk
.L1591:
	adrp	x2, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR2
	add	x2, x2, :lo12:.LANCHOR0
	ldr	x21, [sp,32]
	ldr	w3, [x19,204]
	ldrh	w1, [x2,2016]
	ldr	w0, [x19,208]
	madd	w0, w1, w3, w0
	ldrh	w1, [x2,1960]
	udiv	w0, w0, w1
	str	w0, [x19,212]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	Ftl_load_ext_data, .-Ftl_load_ext_data
	.align	2
	.global	StorageSysDataLoad
	.type	StorageSysDataLoad, %function
StorageSysDataLoad:
	stp	x29, x30, [sp, -48]!
	mov	x2, 512
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	mov	w20, w0
	mov	x19, x1
	mov	x0, x1
	mov	w1, 0
	bl	memset
	bl	rknand_device_lock
	mov	x2, x19
	mov	w1, 1
	add	w0, w20, 512
	bl	FtlVendorPartRead
	str	x0, [x29,40]
	bl	rknand_device_unlock
	ldr	x0, [x29,40]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	StorageSysDataLoad, .-StorageSysDataLoad
	.align	2
	.global	FtlScanSysBlk
	.type	FtlScanSysBlk, %function
FtlScanSysBlk:
	stp	x29, x30, [sp, -128]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	stp	x21, x22, [sp,32]
	add	x20, x19, :lo12:.LANCHOR0
	adrp	x22, .LANCHOR2
	add	x21, x22, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	strh	wzr, [x21,2304]
	mov	x24, x22
	ldr	w2, [x20,2056]
	mov	w22, 65535
	ldr	x0, [x21,2216]
	lsl	w2, w2, 2
	strh	wzr, [x20,2072]
	bl	ftl_memset
	ldr	w2, [x20,2056]
	mov	w1, 0
	ldr	x0, [x21,2168]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w2, [x20,2048]
	mov	w1, 0
	ldr	x0, [x21,2192]
	lsl	w2, w2, 2
	bl	ftl_memset
	ldrh	w2, [x20,2048]
	mov	w1, 0
	ldr	x0, [x20,2080]
	lsl	w2, w2, 1
	bl	ftl_memset
	add	x0, x21, 280
	mov	w1, 255
	mov	w2, 12
	bl	ftl_memset
	ldrh	w21, [x20,1960]
.L1594:
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w0, [x1,1962]
	cmp	w0, w21
	bls	.L1634
	mov	x27, 0
	ldrh	w5, [x1,1952]
	ldrh	w4, [x1,2038]
	mov	w20, w27
	mov	x23, x1
.L1635:
	cmp	w5, w27, uxth
	bls	.L1667
	add	x0, x23, 1984
	mov	w1, w21
	str	x4, [x29,112]
	str	x5, [x29,120]
	ldrb	w0, [x0,x27]
	bl	V2P_block
	uxth	w28, w0
	bl	FtlBbmIsBadBlock
	ldr	x5, [x29,120]
	ldr	x4, [x29,112]
	cbnz	w0, .L1595
	add	x2, x24, :lo12:.LANCHOR2
	mov	w0, 56
	lsl	w28, w28, 10
	mov	w3, 4
	umull	x0, w20, w0
	ldr	x1, [x2,2048]
	add	x1, x1, x0
	str	w28, [x1,4]
	ldr	x1, [x2,2048]
	add	x0, x1, x0
	ldr	x1, [x2,896]
	ldr	x2, [x2,904]
	str	x1, [x0,8]
	mul	w1, w20, w4
	add	w20, w20, 1
	sdiv	w1, w1, w3
	uxth	w20, w20
	add	x1, x2, x1, sxtw 2
	str	x1, [x0,16]
.L1595:
	add	x27, x27, 1
	b	.L1635
.L1667:
	cbz	w20, .L1598
	add	x0, x24, :lo12:.LANCHOR2
	mov	w1, w20
	mov	w2, 1
	mov	x23, 0
	mov	w27, 61634
	mov	w28, 61574
	ldr	x0, [x0,2048]
	bl	FlashReadPages
.L1599:
	cmp	w20, w23, uxth
	bls	.L1598
	add	x7, x24, :lo12:.LANCHOR2
	mov	x0, 56
	mul	x6, x23, x0
	ldr	x2, [x7,2048]
	add	x1, x2, x6
	ldr	w8, [x2,x6]
	ldr	w0, [x1,4]
	cmn	w8, #1
	ldr	x26, [x1,16]
	ubfx	x25, x0, 10, 16
	bne	.L1601
	add	w0, w0, 1
	str	w0, [x1,4]
	mov	w1, 1
	str	x8, [x29,104]
	ldr	x0, [x7,2048]
	mov	w2, w1
	str	x7, [x29,112]
	add	x0, x0, x6
	str	x6, [x29,120]
	bl	FlashReadPages
	ldrh	w0, [x26]
	ldr	x6, [x29,120]
	cmp	w0, w22
	ldr	x7, [x29,112]
	ldr	x8, [x29,104]
	bne	.L1601
	ldr	x0, [x7,2048]
	str	w8, [x0,x6]
.L1601:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	x1, [x0,2048]
	ldr	w1, [x1,x6]
	cmn	w1, #1
	beq	.L1603
	ldr	w0, [x0,196]
	cmn	w0, #1
	beq	.L1604
	ldr	w1, [x26,4]
	cmp	w0, w1
	bhi	.L1605
.L1604:
	ldr	w0, [x26,4]
	cmn	w0, #1
	beq	.L1605
	add	x1, x24, :lo12:.LANCHOR2
	add	w0, w0, 1
	str	w0, [x1,196]
.L1605:
	ldrh	w0, [x26]
	mov	w1, 61604
	cmp	w0, w1
	beq	.L1607
	bhi	.L1608
	cmp	w0, w28
	bne	.L1606
	add	x0, x19, :lo12:.LANCHOR0
	add	x7, x24, :lo12:.LANCHOR2
	ldrh	w6, [x0,2048]
	ldrh	w0, [x0,2072]
	sub	w2, w6, #1
	ldr	x7, [x7,2192]
	sxth	x1, w2
	sub	w2, w2, w0
	b	.L1621
.L1608:
	cmp	w0, w27
	beq	.L1610
	cmp	w0, w22
	beq	.L1666
	b	.L1606
.L1610:
	add	x0, x19, :lo12:.LANCHOR0
	add	x7, x24, :lo12:.LANCHOR2
	ldr	w6, [x0,2056]
	ldrh	w0, [x7,2304]
	ldr	x7, [x7,2216]
	uxth	w2, w6
	sub	w1, w2, #1
	sub	w2, w2, w0
	sub	w2, w2, #1
	sxth	x1, w1
	sxth	w2, w2
.L1612:
	cmp	w1, w2
	ble	.L1668
	sbfiz	x8, x1, 2, 32
	ldr	w11, [x26,4]
	sxth	x9, w1
	ldr	w10, [x7,x8]
	cmp	w11, w10
	bls	.L1613
	ldr	w2, [x7]
	cbnz	w2, .L1614
	cmp	w0, w6
	beq	.L1614
	add	x2, x24, :lo12:.LANCHOR2
	add	w0, w0, 1
	strh	w0, [x2,2304]
.L1614:
	mov	w0, 0
.L1615:
	cmp	w0, w1
	beq	.L1669
	add	x6, x24, :lo12:.LANCHOR2
	sxtw	x2, w0
	lsl	x7, x2, 2
	lsl	x2, x2, 1
	add	w0, w0, 1
	ldr	x10, [x6,2216]
	sxth	w0, w0
	add	x11, x10, x7
	ldr	w11, [x11,4]
	str	w11, [x10,x7]
	ldr	x6, [x6,2168]
	add	x7, x6, x2
	ldrh	w7, [x7,2]
	strh	w7, [x6,x2]
	b	.L1615
.L1669:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	w6, [x26,4]
	ldr	x2, [x0,2216]
	str	w6, [x2,x8]
	ldr	x2, [x0,2168]
	strh	w25, [x2,x9,lsl 1]
	tbz	w1, #31, .L1617
	b	.L1606
.L1613:
	sub	w1, w1, #1
	sxth	x1, w1
	b	.L1612
.L1668:
	tbz	w1, #31, .L1650
	b	.L1606
.L1617:
	add	x2, x19, :lo12:.LANCHOR0
	ldrh	w0, [x0,2304]
	ldr	w2, [x2,2056]
	sub	w2, w2, w0
	sub	w2, w2, #1
	cmp	w1, w2, sxth
	bgt	.L1606
.L1650:
	add	x2, x24, :lo12:.LANCHOR2
	add	w0, w0, 1
	ldr	w4, [x26,4]
	strh	w0, [x2,2304]
	ldr	x0, [x2,2216]
	str	w4, [x0,x1,lsl 2]
	ldr	x0, [x2,2168]
	b	.L1665
.L1627:
	sbfiz	x8, x1, 2, 32
	ldr	w11, [x26,4]
	sxth	x9, w1
	ldr	w10, [x7,x8]
	cmp	w11, w10
	bhi	.L1670
	sub	w1, w1, #1
	sxth	x1, w1
.L1621:
	cmp	w1, w2
	bgt	.L1627
	b	.L1626
.L1670:
	ldr	w2, [x7]
	cbnz	w2, .L1623
	cmp	w0, w6
	beq	.L1623
	add	x2, x19, :lo12:.LANCHOR0
	add	w0, w0, 1
	strh	w0, [x2,2072]
.L1623:
	mov	w0, 0
.L1624:
	cmp	w0, w1
	beq	.L1671
	add	x2, x24, :lo12:.LANCHOR2
	ldr	x7, [x2,2192]
	sxtw	x2, w0
	lsl	x6, x2, 2
	lsl	x2, x2, 1
	add	x10, x7, x6
	add	w0, w0, 1
	sxth	w0, w0
	ldr	w10, [x10,4]
	str	w10, [x7,x6]
	add	x6, x19, :lo12:.LANCHOR0
	ldr	x6, [x6,2080]
	add	x7, x6, x2
	ldrh	w7, [x7,2]
	strh	w7, [x6,x2]
	b	.L1624
.L1671:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	w2, [x26,4]
	ldr	x0, [x0,2192]
	str	w2, [x0,x8]
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x0, [x0,2080]
	strh	w25, [x0,x9,lsl 1]
.L1626:
	tbnz	w1, #31, .L1606
	add	x2, x19, :lo12:.LANCHOR0
	ldrh	w0, [x2,2048]
	ldrh	w6, [x2,2072]
	sub	w0, w0, #1
	sub	w0, w0, w6
	cmp	w1, w0, sxth
	bgt	.L1606
	add	x0, x24, :lo12:.LANCHOR2
	add	w6, w6, 1
	ldr	w4, [x26,4]
	strh	w6, [x2,2072]
	ldr	x0, [x0,2192]
	str	w4, [x0,x1,lsl 2]
	ldr	x0, [x2,2080]
.L1665:
	strh	w25, [x0,x1,lsl 1]
	b	.L1606
.L1607:
	add	x0, x24, :lo12:.LANCHOR2
	add	x1, x0, 280
	ldrh	w2, [x0,280]
	cmp	w2, w22
	bne	.L1628
	strh	w25, [x0,280]
	ldr	w0, [x26,4]
	str	w0, [x1,8]
	b	.L1606
.L1628:
	ldrh	w0, [x1,4]
	cmp	w0, w22
	beq	.L1629
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
.L1629:
	add	x1, x24, :lo12:.LANCHOR2
	ldr	w2, [x26,4]
	add	x0, x1, 280
	ldr	w6, [x0,8]
	cmp	w6, w2
	bcs	.L1630
	ldrh	w2, [x1,280]
	strh	w2, [x0,4]
	strh	w25, [x1,280]
	ldr	w1, [x26,4]
	str	w1, [x0,8]
	b	.L1606
.L1630:
	strh	w25, [x0,4]
	b	.L1606
.L1603:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1631
.L1666:
	mov	w0, w25
	mov	w1, 0
	b	.L1664
.L1631:
	mov	w0, w25
	mov	w1, 1
.L1664:
	bl	FtlFreeSysBlkQueueIn
.L1606:
	add	x23, x23, 1
	b	.L1599
.L1598:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1594
.L1634:
	add	x4, x24, :lo12:.LANCHOR2
	ldr	x2, [x4,2168]
	ldrh	w0, [x2]
	cbz	w0, .L1636
.L1639:
	add	x1, x19, :lo12:.LANCHOR0
	ldr	x2, [x1,2080]
	ldrh	w0, [x2]
	cbz	w0, .L1637
	b	.L1659
.L1636:
	ldrh	w4, [x4,2304]
	cbz	w4, .L1639
	ldr	w1, [x1,2056]
.L1640:
	cmp	w0, w1
	bcs	.L1639
	ldrh	w4, [x2,w0,sxtw 1]
	cbz	w4, .L1641
	mov	w7, w0
.L1642:
	add	x1, x19, :lo12:.LANCHOR0
	ldr	w1, [x1,2056]
	cmp	w0, w1
	bcs	.L1639
	add	x2, x24, :lo12:.LANCHOR2
	sxtw	x6, w0
	sub	w1, w0, w7
	lsl	x4, x6, 1
	add	w0, w0, 1
	ldr	x5, [x2,2168]
	sxtw	x1, w1
	sxth	w0, w0
	ldrh	w8, [x5,x4]
	strh	w8, [x5,x1,lsl 1]
	ldr	x5, [x2,2216]
	ldr	w6, [x5,x6,lsl 2]
	str	w6, [x5,x1,lsl 2]
	ldr	x1, [x2,2168]
	strh	wzr, [x1,x4]
	b	.L1642
.L1641:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L1640
.L1637:
	ldrh	w4, [x1,2072]
	cbz	w4, .L1659
	ldrh	w1, [x1,2048]
.L1645:
	cmp	w0, w1
	mov	w7, w0
	bge	.L1659
	ldrh	w4, [x2,w0,sxtw 1]
	cbz	w4, .L1646
.L1647:
	add	x2, x19, :lo12:.LANCHOR0
	ldrh	w1, [x2,2048]
	cmp	w0, w1
	bge	.L1659
	sxtw	x6, w0
	sub	w1, w0, w7
	ldr	x5, [x2,2080]
	lsl	x4, x6, 1
	sxtw	x1, w1
	add	w0, w0, 1
	ldrh	w8, [x5,x4]
	sxth	w0, w0
	strh	w8, [x5,x1,lsl 1]
	add	x5, x24, :lo12:.LANCHOR2
	ldr	x5, [x5,2192]
	ldr	w6, [x5,x6,lsl 2]
	str	w6, [x5,x1,lsl 2]
	ldr	x1, [x2,2080]
	strh	wzr, [x1,x4]
	b	.L1647
.L1646:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L1645
.L1659:
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
	.global	FtlLoadSysInfo
	.type	FtlLoadSysInfo, %function
FtlLoadSysInfo:
	stp	x29, x30, [sp, -80]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR4
	adrp	x21, .LANCHOR2
	stp	x19, x20, [sp,16]
	add	x20, x21, :lo12:.LANCHOR2
	add	x19, x22, :lo12:.LANCHOR4
	stp	x23, x24, [sp,48]
	str	x25, [sp,64]
	add	x19, x19, 136
	adrp	x23, .LANCHOR0
	str	xzr, [x19,8]
	ldr	x0, [x20,2112]
	add	x25, x20, 280
	str	x0, [x19,16]
	add	x0, x23, :lo12:.LANCHOR0
	ldrh	w2, [x0,1960]
	ldr	x0, [x20,-40]
	lsl	w2, w2, 1
	bl	ftl_memset
	ldrh	w0, [x20,280]
	mov	w1, 65535
	cmp	w0, w1
	bne	.L1673
.L1677:
	mov	w19, -1
	b	.L1674
.L1673:
	mov	w1, 1
	bl	FtlGetLastWrittenPage
	sxth	w24, w0
	add	w0, w0, 1
	strh	w0, [x25,2]
	mov	w25, 61604
.L1675:
	tbnz	w24, #31, .L1680
	ldrh	w0, [x20,280]
	mov	w1, 1
	mov	w2, w1
	orr	w0, w24, w0, lsl 10
	str	w0, [x19,4]
	ldr	x0, [x20,-64]
	str	x0, [x19,8]
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L1676
	ldr	x0, [x20,-64]
	ldr	w1, [x0]
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	cmp	w1, w0
	bne	.L1676
	ldr	x0, [x20,2112]
	ldrh	w0, [x0]
	cmp	w0, w25
	bne	.L1676
.L1680:
	add	x24, x22, :lo12:.LANCHOR4
	add	x19, x21, :lo12:.LANCHOR2
	add	x20, x19, 224
	mov	w2, 48
	add	x22, x23, :lo12:.LANCHOR0
	mov	x0, x20
	ldr	x1, [x24,144]
	bl	ftl_memcpy
	ldr	x1, [x24,144]
	ldrh	w2, [x22,1960]
	ldr	x0, [x19,-40]
	add	x1, x1, 48
	lsl	w2, w2, 1
	bl	ftl_memcpy
	ldrh	w2, [x22,1960]
	ldr	x3, [x24,144]
	ldr	x0, [x23,#:lo12:.LANCHOR0]
	uxth	x1, w2
	lsr	w2, w2, 3
	add	x1, x1, 24
	add	w2, w2, 4
	lsr	x1, x1, 1
	add	x1, x3, x1, lsl 2
	bl	ftl_memcpy
	ldr	w1, [x19,224]
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	cmp	w1, w0
	beq	.L1690
	b	.L1677
.L1676:
	sub	w24, w24, #1
	sxth	w24, w24
	b	.L1675
.L1690:
	ldrb	w1, [x20,10]
	ldrh	w0, [x22,1974]
	ldrh	w2, [x20,8]
	strh	w2, [x19,286]
	cmp	w1, w0
	bne	.L1677
	ldrh	w1, [x22,2026]
	ldrh	w0, [x22,2032]
	str	w2, [x24,192]
	mul	w1, w1, w2
	str	w1, [x22,2104]
	strh	wzr, [x19,298]
	mul	w1, w1, w0
	str	w1, [x22,2068]
	ldrh	w0, [x22,2118]
	ldr	w1, [x22,1964]
	strb	wzr, [x19,302]
	sub	w0, w1, w0
	ldrh	w1, [x22,1952]
	sub	w0, w0, w2
	ldrh	w2, [x20,14]
	strb	wzr, [x19,304]
	udiv	w0, w0, w1
	strh	w0, [x19,272]
	adrp	x0, .LANCHOR2
	strh	w2, [x0,#:lo12:.LANCHOR2]
	ldrh	w0, [x20,16]
	lsr	w1, w0, 6
	strh	w1, [x19,2]
	mov	w1, -1
	strh	w1, [x19,296]
	ldrh	w1, [x20,18]
	and	w0, w0, 63
	strh	w1, [x19,48]
	ldrh	w1, [x20,20]
	strb	w0, [x19,6]
	lsr	w3, w1, 6
	and	w1, w1, 63
	strb	w1, [x19,54]
	ldrb	w1, [x20,12]
	strb	w1, [x19,56]
	ldrh	w1, [x20,22]
	strh	w1, [x19,96]
	ldrh	w1, [x20,24]
	ldrb	w0, [x20,11]
	strh	w3, [x19,50]
	lsr	w3, w1, 6
	and	w1, w1, 63
	strb	w0, [x19,8]
	strb	w1, [x19,102]
	strh	w3, [x19,98]
	ldrb	w1, [x20,13]
	strb	w1, [x19,104]
	ldr	w0, [x20,32]
	str	w0, [x19,204]
	ldr	w1, [x19,196]
	ldr	w0, [x20,40]
	str	wzr, [x19,184]
	str	wzr, [x19,172]
	cmp	w0, w1
	str	wzr, [x19,164]
	str	wzr, [x19,180]
	str	wzr, [x19,208]
	str	wzr, [x19,216]
	str	wzr, [x19,176]
	bls	.L1681
	str	w0, [x19,196]
.L1681:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	w1, [x0,260]
	ldr	w3, [x0,200]
	cmp	w1, w3
	bls	.L1682
	str	w1, [x0,200]
.L1682:
	mov	w0, 65535
	cmp	w2, w0
	beq	.L1683
	add	x0, x21, :lo12:.LANCHOR2
	bl	make_superblock
.L1683:
	add	x1, x21, :lo12:.LANCHOR2
	add	x0, x1, 48
	ldrh	w2, [x1,48]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L1684
	bl	make_superblock
.L1684:
	add	x1, x21, :lo12:.LANCHOR2
	add	x0, x1, 96
	ldrh	w2, [x1,96]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L1685
	bl	make_superblock
.L1685:
	add	x21, x21, :lo12:.LANCHOR2
	mov	w1, 65535
	add	x0, x21, 296
	mov	w19, 0
	ldrh	w2, [x21,296]
	cmp	w2, w1
	beq	.L1674
	bl	make_superblock
.L1674:
	mov	w0, w19
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlLoadSysInfo, .-FtlLoadSysInfo
	.align	2
	.global	FtlGcScanTempBlk
	.type	FtlGcScanTempBlk, %function
FtlGcScanTempBlk:
	stp	x29, x30, [sp, -160]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR1
	stp	x27, x28, [sp,80]
	mov	x28, x0
	add	x0, x19, :lo12:.LANCHOR1
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	str	w1, [x29,156]
	ldrh	w27, [x0,3004]
	mov	w0, 65535
	str	x19, [x29,144]
	cmp	w27, w0
	beq	.L1712
	cbnz	w27, .L1692
	b	.L1693
.L1712:
	mov	w27, 0
.L1692:
	adrp	x0, .LANCHOR0+2026
	ldr	w1, [x29,156]
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2026]
	cmp	w1, w0
	bne	.L1694
.L1693:
	bl	FtlGcPageVarInit
.L1694:
	mov	w22, -1
	mov	w20, 0
	mov	w21, 65535
	adrp	x23, .LANCHOR0
.L1706:
	ldrh	w0, [x28]
	strb	wzr, [x28,8]
	cmp	w0, w21
	beq	.L1713
.L1696:
	adrp	x26, .LANCHOR2
.L1709:
	add	x0, x23, :lo12:.LANCHOR0
	mov	x2, x28
	mov	w19, 0
	mov	w10, 4
	ldrh	w6, [x0,1952]
	ldrh	w9, [x0,2038]
	add	x6, x28, x6, lsl 1
.L1697:
	cmp	x2, x6
	beq	.L1723
	ldrh	w1, [x2,16]
	cmp	w1, w21
	beq	.L1698
	add	x7, x26, :lo12:.LANCHOR2
	mov	w0, 56
	orr	w1, w27, w1, lsl 10
	add	w5, w19, 1
	umull	x0, w19, w0
	ldr	x8, [x7,2048]
	add	x8, x8, x0
	str	w1, [x8,4]
	ldr	x1, [x7,2048]
	ldr	x7, [x7,904]
	add	x0, x1, x0
	mul	w1, w19, w9
	uxth	w19, w5
	str	xzr, [x0,8]
	sdiv	w1, w1, w10
	add	x1, x7, x1, sxtw 2
	str	x1, [x0,16]
.L1698:
	add	x2, x2, 2
	b	.L1697
.L1723:
	add	x0, x26, :lo12:.LANCHOR2
	mov	w1, w19
	mov	w2, 0
	mov	x24, 0
	adrp	x25, .LANCHOR2
	ldr	x0, [x0,2048]
	bl	FlashReadPages
.L1700:
	cmp	w19, w24, uxth
	bls	.L1724
	add	x9, x25, :lo12:.LANCHOR2
	mov	x0, 56
	str	x9, [x29,104]
	mul	x8, x24, x0
	ldr	x11, [x9,2048]
	str	x8, [x29,112]
	add	x10, x11, x8
	str	x11, [x29,120]
	str	x10, [x29,128]
	ldr	w1, [x10,4]
	str	x1, [x29,136]
	lsr	x0, x1, 10
	bl	P2V_plane
	uxth	w2, w0
	ldr	x11, [x29,120]
	ldr	x8, [x29,112]
	ldr	x1, [x29,136]
	ldr	x9, [x29,104]
	ldr	w8, [x11,x8]
	cbnz	w8, .L1701
	ldr	x10, [x29,128]
	add	x24, x24, 1
	ldr	x8, [x10,16]
	ldrh	w0, [x8]
	cmp	w0, w21
	bne	.L1702
	mov	w0, 1
	str	w0, [x9,2328]
	b	.L1695
.L1702:
	ldr	w0, [x8,12]
	ldr	w2, [x8,8]
	bl	FtlGcUpdatePage
	b	.L1700
.L1701:
	ldr	w0, [x9,160]
	cbnz	w0, .L1703
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1704
.L1703:
	add	x0, x25, :lo12:.LANCHOR2
	ldrh	w2, [x28]
	ldr	x0, [x0,-80]
	ldrh	w0, [x0,x2,lsl 1]
	cmp	w0, 119
	bls	.L1705
.L1704:
	cmn	w8, #1
	csel	w1, w1, w22, eq
.L1705:
	add	x6, x25, :lo12:.LANCHOR2
	ldrh	w2, [x28]
	str	x1, [x29,136]
	mov	w27, 0
	ldr	x0, [x6,-40]
	strh	wzr, [x0,x2,lsl 1]
	ldrh	w0, [x28]
	bl	INSERT_FREE_LIST
	mov	w0, -1
	strh	w0, [x28]
	bl	FtlGcPageVarInit
	ldr	x1, [x29,136]
	mov	w22, w1
	b	.L1706
.L1724:
	ldr	w0, [x29,156]
	add	w20, w20, 1
	add	w3, w27, 1
	cmp	w20, w0
	uxth	w27, w3
	bcs	.L1708
.L1710:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w0, [x0,2026]
	cmp	w0, w27
	bhi	.L1709
	b	.L1713
.L1708:
	ldr	x0, [x29,144]
	add	x1, x0, :lo12:.LANCHOR1
	ldrh	w0, [x1,3004]
	cmp	w0, w21
	beq	.L1710
	add	w0, w0, w20
	strh	w0, [x1,3004]
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w0, [x0,2026]
	cmp	w0, w27
	bls	.L1710
	b	.L1711
.L1713:
	mov	w2, 0
.L1695:
	ldr	x0, [x29,144]
	mov	w1, w27
	strh	w27, [x28,2]
	add	x19, x0, :lo12:.LANCHOR1
	mov	w0, -1
	strb	w2, [x28,6]
	strh	w0, [x19,3004]
	mov	x0, x28
	bl	ftl_sb_update_avl_pages
.L1711:
	mov	w0, w22
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 160
	ret
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.type	FlashReadFacBbtData.part.12, %function
FlashReadFacBbtData.part.12:
	stp	x29, x30, [sp, -32]!
	mov	w3, 0
	adrp	x4, .LANCHOR0
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	x19, x0
	cbnz	w1, .L1728
	adrp	x1, .LANCHOR4
	mov	w7, 1
.L1731:
	add	x0, x1, :lo12:.LANCHOR4
	ldr	w0, [x0,68]
	cmp	w3, w0
	bcs	.L1728
	add	x6, x4, :lo12:.LANCHOR0
	ubfx	x5, x3, 5, 11
	lsl	x5, x5, 2
	ldr	x0, [x6,1856]
	lsl	w6, w7, w3
	add	w3, w3, 1
	ldr	w8, [x0,x5]
	uxth	w3, w3
	orr	w6, w8, w6
	str	w6, [x0,x5]
	b	.L1731
.L1728:
	add	x4, x4, :lo12:.LANCHOR0
	mov	x0, x19
	ldr	x1, [x4,1856]
	bl	ftl_memcpy
	mov	w2, 4
	adrp	x0, .LC104
	mov	x1, x19
	add	x0, x0, :lo12:.LC104
	mov	w3, w2
	bl	rknand_print_hex
	ldr	x19, [sp,16]
	mov	w0, 0
	ldp	x29, x30, [sp], 32
	ret
	.size	FlashReadFacBbtData.part.12, .-FlashReadFacBbtData.part.12
	.align	2
	.global	FlashReadFacBbtData
	.type	FlashReadFacBbtData, %function
FlashReadFacBbtData:
	stp	x29, x30, [sp, -144]!
	adrp	x4, .LANCHOR0
	add	x4, x4, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	mov	x22, x0
	mov	w23, w1
	ldrh	w0, [x4,182]
	mov	w25, w2
	ldrh	w3, [x4,180]
	mov	w26, 61664
	mul	w3, w3, w0
	ldr	x0, [x4,1856]
	adrp	x4, .LANCHOR4
	str	x0, [x29,96]
	add	x0, x4, :lo12:.LANCHOR4
	uxth	w3, w3
	mov	x21, x4
	sub	w20, w3, #1
	sub	w19, w3, #16
	ldr	x0, [x0,88]
	mul	w24, w1, w3
	str	x0, [x29,104]
	uxth	w20, w20
.L1733:
	cmp	w20, w19
	ble	.L1741
	add	w0, w20, w24
	mov	w1, 1
	lsl	w0, w0, 10
	mov	w2, w1
	str	w0, [x29,92]
	add	x0, x29, 88
	bl	FlashReadPages
	ldr	w0, [x29,88]
	cmn	w0, #1
	beq	.L1734
	add	x0, x21, :lo12:.LANCHOR4
	ldr	x0, [x0,88]
	ldrh	w0, [x0]
	cmp	w0, w26
	bne	.L1734
	mov	w0, w22
	cbz	x22, .L1735
	mov	x0, x22
	mov	w1, w23
	mov	w2, w25
	bl	FlashReadFacBbtData.part.12
	b	.L1735
.L1734:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L1733
.L1741:
	mov	w0, -1
.L1735:
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
	adrp	x0, .LANCHOR0+744
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR4
	ldr	x0, [x0,#:lo12:.LANCHOR0+744]
	ldrb	w2, [x0,13]
	ldrh	w19, [x0,14]
	add	x0, x20, :lo12:.LANCHOR4
	mul	w19, w2, w19
	ldr	x0, [x0,80]
	uxth	w19, w19
	add	w2, w19, 7
	lsr	w2, w2, 3
	bl	FlashReadFacBbtData
	cmn	w0, #1
	bne	.L1743
.L1747:
	mov	w1, 0
	b	.L1744
.L1743:
	mov	w2, 0
	lsr	w0, w19, 4
	mov	w1, w2
	sub	w19, w19, #1
	mov	w5, 1
.L1745:
	cmp	w2, w19
	bge	.L1744
	add	x3, x20, :lo12:.LANCHOR4
	ubfx	x4, x2, 5, 11
	ldr	x6, [x3,80]
	lsl	w3, w5, w2
	ldr	w4, [x6,x4,lsl 2]
	tst	w3, w4
	beq	.L1746
	add	w3, w1, 1
	ubfiz	x1, x1, 1, 16
	strh	w2, [x21,x1]
	uxth	w1, w3
.L1746:
	cmp	w1, w0
	bcs	.L1747
	add	w2, w2, 1
	uxth	w2, w2
	b	.L1745
.L1744:
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
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	stp	x29, x30, [sp, -192]!
	adrp	x6, .LANCHOR1+481
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR0
	ldrb	w6, [x6,#:lo12:.LANCHOR1+481]
	add	x4, x21, :lo12:.LANCHOR0
	str	w3, [x29,120]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	str	w6, [x29,124]
	mov	x19, x0
	ldr	x5, [x4,744]
	mov	w25, w1
	ldrb	w4, [x4,8]
	mov	w23, w2
	mov	w22, 0
	ldrb	w5, [x5,19]
	cbz	w4, .L1753
	bl	FlashProgSlc2KPages
	b	.L1754
.L1753:
	sub	w0, w5, #1
	mov	w27, 56
	mov	x28, 24
	str	w0, [x29,116]
.L1805:
	cmp	w22, w25
	bcs	.L1806
	umull	x20, w22, w27
	sub	w4, w25, w22
	add	x2, x29, 128
	add	x26, x19, x20
	mov	w1, w23
	mov	x0, x26
	add	x3, x29, 132
	bl	LogAddr2PhyAddr
	mov	w24, w0
	add	x2, x21, :lo12:.LANCHOR0
	ldr	w0, [x29,132]
	ldrb	w4, [x2,1845]
	cmp	w0, w4
	bcc	.L1755
	mov	w0, -1
	str	w0, [x19,x20]
	b	.L1756
.L1755:
	ldrb	w1, [x2,1945]
	add	x3, x2, 1652
	cmp	w1, wzr
	uxtw	x1, w0
	csel	w24, w24, wzr, ne
	madd	x0, x1, x28, x3
	ldr	x0, [x0,8]
	cbz	x0, .L1758
	cmp	w4, 1
	bne	.L1759
	ldr	x0, [x2,128]
	bl	NandcIqrWaitFlashReady
.L1759:
	ldrb	w0, [x29,132]
	bl	FlashWaitCmdDone
.L1758:
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w2, [x29,132]
	add	x0, x0, 1652
	ldr	w1, [x29,128]
	madd	x0, x2, x28, x0
	str	x26, [x0,8]
	str	xzr, [x0,16]
	str	w1, [x0,4]
	cbz	w24, .L1760
	add	w1, w22, 1
	umull	x1, w1, w27
	add	x1, x19, x1
	str	x1, [x0,16]
.L1760:
	add	x1, x21, :lo12:.LANCHOR0
	add	x0, x1, x2
	madd	x2, x2, x28, x1
	ldrb	w20, [x0,1848]
	ldrb	w0, [x1,1845]
	strb	w20, [x2,1652]
	cmp	w0, 1
	bne	.L1761
	mov	w0, w20
	bl	NandcWaitFlashReady
	b	.L1762
.L1761:
	mov	w0, w20
	str	x1, [x29,104]
	bl	NandcFlashCs
	ldr	x1, [x29,104]
	ldr	w0, [x29,132]
	add	x1, x1, 708
	ldr	w0, [x1,x0,lsl 2]
	ldr	w1, [x29,128]
	cmp	w0, wzr
	mov	w0, w20
	cset	w2, ne
	bl	FlashWaitReadyEN
	mov	w0, w20
	bl	NandcFlashDeCs
.L1762:
	ldr	w0, [x29,116]
	cmp	w0, 5
	bhi	.L1763
	add	x2, x21, :lo12:.LANCHOR0
	add	x0, x2, x20, sxtw
	ldrb	w0, [x0,1616]
	cbz	w0, .L1763
	ldrb	w1, [x2,761]
	mov	w0, w20
	add	x2, x2, 764
	mov	w3, 0
	bl	HynixSetRRPara
.L1763:
	mov	w0, w20
	bl	NandcFlashCs
	cmp	w23, 1
	bne	.L1764
	add	x0, x21, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1764
	mov	w0, w20
	bl	flash_enter_slc_mode
	b	.L1765
.L1764:
	mov	w0, w20
	bl	flash_exit_slc_mode
.L1765:
	ldr	w1, [x29,128]
	mov	w0, w20
	bl	FlashProgFirstCmd
	ldrb	w2, [x29,124]
	mov	w0, w20
	ldr	x3, [x26,8]
	mov	w1, 1
	ldr	x4, [x26,16]
	bl	NandcXferData
	cbz	w24, .L1766
	ldr	w1, [x29,128]
	mov	w0, w20
	add	x26, x21, :lo12:.LANCHOR0
	bl	FlashProgDpFirstCmd
	ldr	w1, [x29,132]
	add	x0, x26, 708
	ldr	w0, [x0,x1,lsl 2]
	ldr	w1, [x29,128]
	cmp	w0, wzr
	mov	w0, w20
	cset	w2, ne
	bl	FlashWaitReadyEN
	ldr	w2, [x26,12]
	mov	w0, w20
	ldr	w1, [x29,128]
	add	w1, w2, w1
	bl	FlashProgDpSecondCmd
	add	w4, w22, 1
	ldrb	w2, [x29,124]
	mov	w0, w20
	mov	w1, 1
	umull	x4, w4, w27
	add	x4, x19, x4
	ldr	x3, [x4,8]
	ldr	x4, [x4,16]
	bl	NandcXferData
.L1766:
	ldr	w1, [x29,128]
	mov	w0, w20
	add	w22, w22, w24
	bl	FlashProgSecondCmd
	mov	w0, w20
	bl	NandcFlashDeCs
.L1756:
	add	w22, w22, 1
	b	.L1805
.L1806:
	add	x21, x21, :lo12:.LANCHOR0
	mov	x20, 0
	add	x22, x21, 1652
	mov	x24, 24
	ldr	x0, [x21,128]
	bl	NandcIqrWaitFlashReady
.L1768:
	ldrb	w0, [x21,1845]
	cmp	w0, w20
	bls	.L1807
	mov	w0, w20
	bl	FlashWaitCmdDone
	cmp	w23, 1
	bne	.L1769
	ldrb	w0, [x21,120]
	cbz	w0, .L1769
	mul	x0, x20, x24
	ldrb	w0, [x0,x22]
	bl	flash_exit_slc_mode
.L1769:
	add	x20, x20, 1
	b	.L1768
.L1807:
	ldr	w0, [x29,120]
	cbnz	w0, .L1771
.L1779:
	mov	w0, 0
	b	.L1754
.L1771:
	adrp	x24, .LC101
	adrp	x26, .LC107
	adrp	x27, .LC108
	mov	w21, 0
	add	x24, x24, :lo12:.LC101
	add	x26, x26, :lo12:.LC107
	add	x27, x27, :lo12:.LC108
.L1772:
	cmp	w21, w25
	beq	.L1779
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1773
	adrp	x0, .LC98
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC98
	bl	printk
	b	.L1774
.L1773:
	adrp	x20, .LANCHOR4
	sub	w4, w25, w21
	add	x22, x20, :lo12:.LANCHOR4
	add	x3, x29, 132
	mov	w1, w23
	add	x2, x29, 128
	mov	x0, x19
	bl	LogAddr2PhyAddr
	ldr	x0, [x22,96]
	mov	x1, x19
	mov	x2, 56
	str	wzr, [x0]
	ldr	x0, [x22,104]
	str	wzr, [x0]
	add	x0, x29, 136
	bl	memcpy
	ldr	x0, [x22,96]
	mov	w1, 1
	str	x0, [x29,144]
	mov	w2, w23
	ldr	x0, [x22,104]
	str	x0, [x29,152]
	add	x0, x29, 136
	bl	FlashReadPages
	ldr	w22, [x29,136]
	cmn	w22, #1
	bne	.L1775
	adrp	x0, .LC99
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC99
	bl	printk
	str	w22, [x19]
.L1775:
	ldr	x0, [x19,16]
	cbz	x0, .L1776
	add	x22, x20, :lo12:.LANCHOR4
	ldr	w2, [x0]
	ldr	x0, [x22,104]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1776
	ldr	w1, [x19,4]
	adrp	x0, .LC100
	add	x0, x0, :lo12:.LC100
	bl	printk
	ldr	x1, [x19,16]
	adrp	x0, .LC105
	add	x0, x0, :lo12:.LC105
	mov	w2, 4
	mov	w3, 16
	bl	rknand_print_hex
	adrp	x0, .LC106
	ldr	x1, [x22,104]
	add	x0, x0, :lo12:.LC106
	mov	w2, 4
	mov	w3, 16
	bl	rknand_print_hex
	mov	w0, -1
	str	w0, [x19]
.L1776:
	ldr	x0, [x19,8]
	cbz	x0, .L1774
	add	x20, x20, :lo12:.LANCHOR4
	ldr	w2, [x0]
	ldr	x0, [x20,96]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1774
	ldr	w1, [x19,4]
	mov	x0, x24
	bl	printk
	mov	w0, -1
	ldr	x1, [x19,8]
	str	w0, [x19]
	mov	w2, 4
	mov	x0, x26
	mov	w3, 16
	bl	rknand_print_hex
	ldr	x1, [x20,96]
	mov	x0, x27
	mov	w2, 4
	mov	w3, 16
	bl	rknand_print_hex
.L1774:
	add	w21, w21, 1
	add	x19, x19, 56
	b	.L1772
.L1754:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 192
	ret
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.type	FlashTestBlk.part.15, %function
FlashTestBlk.part.15:
	stp	x29, x30, [sp, -160]!
	mov	w1, 165
	mov	w2, 32
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR4
	uxth	w20, w0
	add	x19, x19, :lo12:.LANCHOR4
	lsl	w20, w20, 10
	ldr	x0, [x19,80]
	str	x0, [x29,48]
	add	x0, x29, 96
	str	x0, [x29,56]
	bl	ftl_memset
	ldr	x0, [x19,80]
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
	.size	FlashTestBlk.part.15, .-FlashTestBlk.part.15
	.align	2
	.global	FlashTestBlk
	.type	FlashTestBlk, %function
FlashTestBlk:
	uxth	w1, w0
	adrp	x0, .LANCHOR4+68
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w2, [x0,#:lo12:.LANCHOR4+68]
	mov	w0, 0
	cmp	w1, w2
	bcc	.L1810
	mov	w0, w1
	bl	FlashTestBlk.part.15
.L1810:
	ldp	x29, x30, [sp], 16
	ret
	.size	FlashTestBlk, .-FlashTestBlk
	.align	2
	.global	FlashMakeFactorBbt
	.type	FlashMakeFactorBbt, %function
FlashMakeFactorBbt:
	stp	x29, x30, [sp, -240]!
	adrp	x4, .LANCHOR0
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR4
	add	x0, x22, :lo12:.LANCHOR4
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x23, x24, [sp,48]
	mov	x20, x4
	mov	w19, 0
	ldr	x0, [x0,88]
	add	x25, x20, :lo12:.LANCHOR0
	str	x0, [x29,160]
	add	x0, x4, :lo12:.LANCHOR0
	add	x28, x25, 708
	ldrh	w1, [x0,180]
	ldrh	w21, [x0,182]
	mul	w21, w1, w21
	ldr	x1, [x0,744]
	uxth	w21, w21
	ldrb	w1, [x1,24]
	str	w1, [x29,128]
	ldrh	w1, [x0,12]
	str	w1, [x29,172]
	ldrb	w0, [x0,8]
	ubfiz	w1, w1, 1, 15
	cmp	w0, 1
	ldr	w0, [x29,172]
	csel	w0, w1, w0, eq
	str	w0, [x29,172]
	adrp	x0, .LC109
	mov	w1, 1
	add	x0, x0, :lo12:.LC109
	bl	printk
	add	x0, x22, :lo12:.LANCHOR4
	mov	w1, 0
	mov	w2, 4096
	ldr	x0, [x0,88]
	bl	ftl_memset
	lsr	w0, w21, 4
	str	w0, [x29,168]
	ldr	w0, [x29,128]
	and	w0, w0, 1
	str	w0, [x29,116]
	sub	w0, w21, #1
	uxth	w0, w0
	str	w0, [x29,124]
.L1814:
	ldrb	w0, [x25,1845]
	cmp	w0, w19
	bls	.L1865
	add	x0, x22, :lo12:.LANCHOR4
	sxtw	x24, w19
	add	x0, x0, 120
	ldrh	w20, [x0,w19,sxtw 1]
	cbnz	w20, .L1844
	ldrh	w2, [x25,188]
	mov	w1, w20
	ldr	x0, [x25,1856]
	mov	w26, w20
	lsl	w2, w2, 9
	mov	w23, w20
	bl	ftl_memset
	add	x0, x25, x24
	ldrb	w27, [x0,1848]
	ldr	w0, [x29,128]
	and	w0, w0, 2
	uxtb	w0, w0
	str	w0, [x29,120]
.L1816:
	uxth	w0, w26
	str	w0, [x29,144]
	cmp	w0, w21
	bcs	.L1826
	mov	w0, -1
	strb	w0, [x29,182]
	strb	w0, [x29,183]
	ldr	w0, [x29,116]
	cbz	w0, .L1818
	ldr	w3, [x28,x24,lsl 2]
	mov	w0, w27
	add	x2, x29, 182
	add	w3, w20, w3
	str	x3, [x29,104]
	mov	w1, w3
	bl	FlashReadSpare
	ldrb	w0, [x25,8]
	ldr	x3, [x29,104]
	cmp	w0, 1
	bne	.L1818
	ldr	w1, [x25,12]
	mov	w0, w27
	add	x2, x29, 183
	add	w1, w3, w1
	bl	FlashReadSpare
	ldrb	w0, [x29,182]
	ldrb	w1, [x29,183]
	and	w0, w1, w0
	strb	w0, [x29,182]
.L1818:
	ldr	w0, [x29,120]
	cbz	w0, .L1820
	ldr	x0, [x25,744]
	add	x2, x29, 183
	ldrh	w1, [x0,10]
	ldr	w0, [x28,x24,lsl 2]
	sub	w1, w1, #1
	add	w1, w1, w0
	mov	w0, w27
	add	w1, w1, w20
	bl	FlashReadSpare
.L1820:
	ldr	x0, [x25,744]
	ldrb	w1, [x29,182]
	ldrb	w0, [x0,7]
	cmp	w0, 8
	beq	.L1845
	cmp	w0, 1
	bne	.L1821
.L1845:
	mov	w0, 1
	cbz	w1, .L1823
	ldrb	w0, [x29,183]
	cmp	w0, wzr
	cset	w0, eq
	b	.L1823
.L1821:
	cmp	w1, 255
	mov	w0, 1
	bne	.L1823
	ldrb	w0, [x29,183]
	cmp	w0, 255
	cset	w0, ne
.L1823:
	ldr	x1, [x29,128]
	tbz	x1, 2, .L1824
	ldr	w1, [x28,x24,lsl 2]
	mov	w0, w27
	add	w1, w20, w1
	bl	SandiskProgTestBadBlock
.L1824:
	cbz	w0, .L1825
	adrp	x0, .LC110
	mov	w1, w19
	mov	w2, w26
	add	x0, x0, :lo12:.LC110
	add	w23, w23, 1
	bl	printk
	ldr	x0, [x29,144]
	uxth	w23, w23
	ldr	x2, [x25,1856]
	ubfx	x1, x0, 5, 11
	ldrb	w3, [x29,144]
	lsl	x1, x1, 2
	mov	w0, 1
	lsl	w0, w0, w3
	ldr	w3, [x2,x1]
	orr	w0, w3, w0
	str	w0, [x2,x1]
	ldr	w1, [x29,168]
	ldrb	w0, [x25,1845]
	mul	w0, w1, w0
	cmp	w23, w0
	bgt	.L1826
.L1825:
	ldr	w0, [x29,172]
	add	w26, w26, 1
	add	w20, w20, w0
	b	.L1816
.L1826:
	adrp	x0, .LC111
	mov	w1, w19
	add	x0, x0, :lo12:.LC111
	mov	w2, w23
	bl	printk
	ldrb	w0, [x25,1845]
	ldr	w1, [x29,168]
	mul	w0, w1, w0
	cmp	w23, w0
	blt	.L1828
	ldrh	w2, [x25,188]
	mov	w1, 0
	ldr	x0, [x25,1856]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1828:
	cbnz	w19, .L1830
	add	x0, x22, :lo12:.LANCHOR4
	adrp	x26, .LC112
	mov	w20, w19
	add	x26, x26, :lo12:.LC112
	mov	w23, 1
	ldrh	w27, [x0,68]
.L1831:
	ldrb	w0, [x25,9]
	cmp	w0, w27
	bls	.L1866
	mov	w0, w27
	bl	FlashTestBlk
	cbz	w0, .L1832
	mov	w1, w27
	mov	x0, x26
	add	w20, w20, 1
	bl	printk
	ubfx	x1, x27, 5, 11
	ldr	x2, [x25,1856]
	lsl	x1, x1, 2
	lsl	w0, w23, w27
	uxth	w20, w20
	ldr	w3, [x2,x1]
	orr	w0, w3, w0
	str	w0, [x2,x1]
.L1832:
	add	w27, w27, 1
	uxth	w27, w27
	b	.L1831
.L1866:
	adrp	x26, .LC112
	ldr	w23, [x29,124]
	sub	w27, w21, #50
	add	x26, x26, :lo12:.LC112
.L1834:
	cmp	w23, w27
	ble	.L1867
	mov	w0, w23
	bl	FlashTestBlk
	cbz	w0, .L1835
	mov	w1, w23
	mov	x0, x26
	bl	printk
	ubfx	x1, x23, 5, 11
	ldr	x3, [x25,1856]
	lsl	x1, x1, 2
	mov	w0, 1
	lsl	w0, w0, w23
	ldr	w5, [x3,x1]
	orr	w0, w5, w0
	str	w0, [x3,x1]
.L1835:
	sub	w2, w23, #1
	uxth	w23, w2
	b	.L1834
.L1867:
	add	x1, x22, :lo12:.LANCHOR4
	ldrb	w0, [x25,9]
	ldr	w1, [x1,68]
	sub	w0, w0, w1
	cmp	w20, w0
	bcc	.L1830
	ldrh	w2, [x25,188]
	mov	w1, 0
	ldr	x0, [x25,1856]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1830:
	adrp	x23, .LC113
	ldr	w27, [x29,124]
	mul	w26, w19, w21
	add	x23, x23, :lo12:.LC113
	add	x20, x22, :lo12:.LANCHOR4
.L1838:
	mov	w2, w27
	mov	x0, x23
	mov	w1, w19
	bl	printk
	ldr	x2, [x25,1856]
.L1839:
	ubfx	x0, x27, 5, 11
	ldr	w0, [x2,x0,lsl 2]
	lsr	w0, w0, w27
	and	w1, w0, 1
	tbz	x0, 0, .L1868
	sub	w27, w27, #1
	uxth	w27, w27
	b	.L1839
.L1868:
	add	x0, x20, 120
	mov	w2, -3872
	strh	w27, [x0,x24,lsl 1]
	ldr	x0, [x29,160]
	strh	w2, [x0]
	strh	w27, [x0,2]
	strh	w1, [x0,8]
	mov	w1, 1
	ldr	x0, [x25,1856]
	mov	w2, w1
	str	x0, [x29,192]
	ldr	x0, [x20,88]
	str	x0, [x29,200]
	add	w0, w27, w26
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
	cbz	w0, .L1844
	sub	w27, w27, #1
	uxth	w27, w27
	b	.L1838
.L1844:
	add	w19, w19, 1
	uxtb	w19, w19
	b	.L1814
.L1865:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 240
	ret
	.size	FlashMakeFactorBbt, .-FlashMakeFactorBbt
	.align	2
	.global	FtlLowFormatEraseBlock
	.type	FtlLowFormatEraseBlock, %function
FtlLowFormatEraseBlock:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR0
	stp	x19, x20, [sp,16]
	add	x20, x22, :lo12:.LANCHOR0
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x23, x24, [sp,48]
	uxth	w26, w0
	mov	w28, 0
	ldrb	w0, [x20,120]
	uxtb	w25, w1
	str	w0, [x29,120]
	mov	w21, w28
	str	w26, [x20,2108]
	mov	w19, w28
	adrp	x24, .LANCHOR2
	mov	w27, 56
.L1870:
	ldrh	w0, [x20,1952]
	cmp	w0, w28
	bls	.L1914
	add	x1, x24, :lo12:.LANCHOR2
	umull	x0, w28, w27
	ldr	x1, [x1,-88]
	str	wzr, [x1,x0]
	add	x0, x20, 1984
	mov	w1, w26
	ldrb	w0, [x0,w28,sxtw]
	bl	V2P_block
	uxth	w23, w0
	mov	w1, w23
	cbz	w25, .L1871
	str	x1, [x29,112]
	bl	IsBlkInVendorPart
	ldr	x1, [x29,112]
	cbnz	w0, .L1872
.L1871:
	mov	w0, w1
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L1873
	adrp	x1, .LANCHOR2
	lsl	w23, w23, 10
	add	x3, x1, :lo12:.LANCHOR2
	mov	w2, 4
	umull	x0, w21, w27
	ldr	x1, [x3,-88]
	add	x1, x1, x0
	str	w23, [x1,4]
	ldr	x1, [x3,-88]
	ldr	x3, [x3,2120]
	add	x1, x1, x0
	ldrh	w0, [x20,2038]
	mul	w0, w21, w0
	add	w21, w21, 1
	str	xzr, [x1,8]
	sdiv	w0, w0, w2
	uxth	w21, w21
	add	x0, x3, x0, sxtw 2
	str	x0, [x1,16]
	b	.L1872
.L1873:
	add	w19, w19, 1
	uxth	w19, w19
.L1872:
	add	w2, w28, 1
	uxth	w28, w2
	b	.L1870
.L1914:
	cbz	w21, .L1876
	ldr	w0, [x29,120]
	adrp	x20, .LANCHOR2
	mov	w2, w21
	mov	x24, 0
	cmp	w0, wzr
	add	x0, x20, :lo12:.LANCHOR2
	cset	w23, ne
	mov	x27, 56
	mov	w1, w23
	ldr	x0, [x0,-88]
	bl	FlashEraseBlocks
.L1877:
	cmp	w21, w24, uxth
	bls	.L1915
	add	x1, x20, :lo12:.LANCHOR2
	mul	x0, x24, x27
	ldr	x1, [x1,-88]
	add	x2, x1, x0
	ldr	w0, [x1,x0]
	cmn	w0, #1
	bne	.L1878
	ldr	w0, [x2,4]
	add	w19, w19, 1
	lsr	w0, w0, 10
	uxth	w19, w19
	bl	FtlBbmMapBadBlock
.L1878:
	add	x24, x24, 1
	b	.L1877
.L1915:
	cbnz	w25, .L1880
	uxth	w23, w23
	mov	w28, 6
	mov	w27, 1
	b	.L1881
.L1880:
	add	x0, x22, :lo12:.LANCHOR0
	ldrh	w27, [x0,2028]
	ldrb	w0, [x0,120]
	cbnz	w0, .L1896
	lsr	w28, w27, 2
	mov	w23, 1
	b	.L1881
.L1896:
	mov	w23, 1
	mov	w28, w23
.L1881:
	add	x0, x22, :lo12:.LANCHOR0
	mov	w24, 0
	str	x0, [x29,120]
.L1890:
	mov	w22, 0
	mov	w21, w22
.L1882:
	ldr	x0, [x29,120]
	ldrh	w0, [x0,1952]
	cmp	w0, w22
	bls	.L1916
	add	x1, x20, :lo12:.LANCHOR2
	mov	w0, 56
	umull	x0, w22, w0
	ldr	x1, [x1,-88]
	str	wzr, [x1,x0]
	mov	w1, w26
	ldr	x0, [x29,120]
	add	x0, x0, 1984
	ldrb	w0, [x0,w22,sxtw]
	bl	V2P_block
	uxth	w2, w0
	str	w2, [x29,112]
	mov	w1, w2
	cbz	w25, .L1883
	str	x1, [x29,104]
	bl	IsBlkInVendorPart
	ldr	x1, [x29,104]
	cbnz	w0, .L1884
.L1883:
	mov	w0, w1
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L1884
	add	x3, x20, :lo12:.LANCHOR2
	mov	w0, 56
	ldr	w2, [x29,112]
	umull	x0, w21, w0
	ldr	x1, [x3,-88]
	add	w4, w24, w2, lsl 10
	mov	w2, 4
	add	x1, x1, x0
	str	w4, [x1,4]
	ldr	x1, [x3,-88]
	add	x1, x1, x0
	ldr	x0, [x3,2096]
	ldr	x3, [x3,2104]
	str	x0, [x1,8]
	ldr	x0, [x29,120]
	ldrh	w0, [x0,2038]
	mul	w0, w21, w0
	add	w21, w21, 1
	sdiv	w0, w0, w2
	uxth	w21, w21
	add	x0, x3, x0, sxtw 2
	str	x0, [x1,16]
.L1884:
	add	w2, w22, 1
	uxth	w22, w2
	b	.L1882
.L1916:
	cbz	w21, .L1876
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, w21
	mov	w2, w23
	mov	w3, 1
	ldr	x0, [x0,-88]
	bl	FlashProgPages
	mov	x1, 0
.L1887:
	cmp	w21, w1, uxth
	bls	.L1917
	add	x2, x20, :lo12:.LANCHOR2
	mov	x0, 56
	mul	x0, x1, x0
	ldr	x2, [x2,-88]
	add	x3, x2, x0
	ldr	w0, [x2,x0]
	cbz	w0, .L1888
	ldr	w0, [x3,4]
	add	w19, w19, 1
	str	x1, [x29,112]
	lsr	w0, w0, 10
	uxth	w19, w19
	bl	FtlBbmMapBadBlock
	ldr	x1, [x29,112]
.L1888:
	add	x1, x1, 1
	b	.L1887
.L1917:
	add	w24, w24, w28
	uxth	w24, w24
	cmp	w24, w27
	bcc	.L1890
	mov	x22, 0
	mov	x24, 56
.L1891:
	cmp	w21, w22, uxth
	bls	.L1918
	cbz	w25, .L1892
	add	x1, x20, :lo12:.LANCHOR2
	mul	x0, x22, x24
	ldr	x1, [x1,-88]
	add	x2, x1, x0
	ldr	w0, [x1,x0]
	cbnz	w0, .L1892
	ldr	w0, [x2,4]
	mov	w1, 1
	lsr	w0, w0, 10
	bl	FtlFreeSysBlkQueueIn
.L1892:
	add	x22, x22, 1
	b	.L1891
.L1918:
	cmp	w26, 63
	bls	.L1897
	cbz	w25, .L1876
.L1897:
	add	x20, x20, :lo12:.LANCHOR2
	mov	w1, w23
	mov	w2, w21
	ldr	x0, [x20,-88]
	bl	FlashEraseBlocks
.L1876:
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	FtlLowFormatEraseBlock, .-FtlLowFormatEraseBlock
	.align	2
	.global	FtlBbmTblFlush
	.type	FtlBbmTblFlush, %function
FtlBbmTblFlush:
	stp	x29, x30, [sp, -96]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR2
	stp	x19, x20, [sp,16]
	add	x2, x22, :lo12:.LANCHOR2
	adrp	x19, .LANCHOR4
	adrp	x20, .LANCHOR0
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	add	x24, x19, :lo12:.LANCHOR4
	add	x20, x20, :lo12:.LANCHOR0
	ldr	x0, [x2,-64]
	mov	x23, 0
	ldr	x2, [x2,2112]
	add	x21, x20, 2112
	str	x2, [x24,152]
	ldrh	w2, [x20,2036]
	str	x0, [x24,144]
	bl	ftl_memset
.L1920:
	ldrh	w1, [x20,1974]
	mov	w0, w23
	add	x23, x23, 1
	cmp	w0, w1
	bge	.L1928
	add	x1, x22, :lo12:.LANCHOR2
	ldr	x3, [x24,144]
	ldrh	w2, [x1,2232]
	add	x1, x21, x23, lsl 3
	ldr	x1, [x1,24]
	mul	w0, w0, w2
	add	x0, x3, x0, sxtw 2
	lsl	w2, w2, 2
	bl	ftl_memcpy
	b	.L1920
.L1928:
	add	x19, x19, :lo12:.LANCHOR4
	mov	w1, 255
	add	x19, x19, 136
	mov	w2, 16
	mov	w28, 0
	adrp	x25, .LC114
	ldr	x27, [x19,16]
	adrp	x26, .LC115
	mov	w24, w28
	add	x25, x25, :lo12:.LC114
	mov	x0, x27
	add	x26, x26, :lo12:.LC115
	bl	ftl_memset
	mov	w0, -3887
	strh	w0, [x27]
	ldr	w0, [x21,8]
	str	w0, [x27,4]
	ldrh	w0, [x20,2112]
	strh	w0, [x27,2]
	ldrh	w0, [x21,4]
	strh	w0, [x27,8]
	ldrh	w0, [x21,6]
	strh	w0, [x27,10]
	ldr	w0, [x20,1948]
	strh	w0, [x27,12]
.L1922:
	add	x23, x22, :lo12:.LANCHOR2
	ldrh	w1, [x20,2112]
	ldrh	w2, [x20,2114]
	ldrh	w3, [x20,2116]
	ldr	x0, [x23,-64]
	str	x0, [x19,8]
	ldr	x0, [x23,2112]
	str	x0, [x19,16]
	orr	w0, w2, w1, lsl 10
	str	w0, [x19,4]
	mov	x0, x25
	ldrh	w4, [x27,10]
	str	wzr, [x19]
	bl	printk
	mov	w1, 1
	mov	x0, x19
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
	ldrh	w0, [x20,2028]
	ldrh	w1, [x20,2114]
	sub	w0, w0, #1
	cmp	w1, w0
	blt	.L1923
	ldr	w1, [x20,2120]
	ldrh	w0, [x20,2112]
	add	w1, w1, 1
	str	w1, [x20,2120]
	str	w1, [x27,4]
	strh	w0, [x27,8]
	ldrh	w1, [x20,2116]
	strh	w0, [x20,2116]
	ldr	x0, [x23,-88]
	strh	w1, [x20,2112]
	lsl	w1, w1, 10
	str	w1, [x19,4]
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	strh	wzr, [x20,2114]
	ldr	x0, [x23,-88]
	bl	FlashEraseBlocks
	mov	w1, 1
	mov	x0, x19
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
.L1923:
	ldrh	w0, [x20,2114]
	add	w0, w0, 1
	strh	w0, [x20,2114]
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1924
	ldr	w1, [x19,4]
	mov	x0, x26
	add	w28, w28, 1
	bl	printk
	uxth	w28, w28
	cmp	w28, 3
	bls	.L1922
	adrp	x0, .LC116
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC116
	mov	w2, w28
	bl	printk
.L1926:
	b	.L1926
.L1924:
	cbnz	w24, .L1929
	mov	w24, 1
	b	.L1922
.L1929:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlBbmTblFlush, .-FtlBbmTblFlush
	.align	2
	.global	allocate_data_superblock
	.type	allocate_data_superblock, %function
allocate_data_superblock:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	add	x21, x21, :lo12:.LANCHOR2
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	mov	x20, x0
	add	x23, x21, 96
	mov	x19, x21
	sub	x24, x21, #16
.L1931:
	cmp	x20, x23
	bne	.L1932
	ldrh	w0, [x21,-8]
	ldr	w3, [x21,876]
	lsr	w2, w0, 1
	add	w4, w2, 1
	mul	w1, w0, w3
	add	w1, w4, w1, lsr 2
	ldr	w4, [x21,160]
	uxth	w1, w1
	cbz	w4, .L1933
	ldr	w4, [x21,220]
	cmp	w4, 29
	bhi	.L1933
	cmp	w4, 2
	mov	w1, 0
	bls	.L1934
	tbz	x0, 0, .L1959
	cbz	w3, .L1934
.L1959:
	mov	w1, w2
	b	.L1933
.L1932:
	ldrb	w0, [x20,8]
	mov	w1, 0
	cmp	w0, 1
	bne	.L1934
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w2, [x0,1972]
	cmp	w2, 1
	beq	.L1934
	ldrb	w0, [x0,120]
	cbnz	w0, .L1934
	ldr	w2, [x21,160]
	ldrh	w0, [x21,-8]
	lsr	w1, w0, 3
	cbz	w2, .L1933
	ldr	w2, [x21,220]
	cmp	w2, 1
	bhi	.L1933
	mov	w1, 7
	mul	w1, w0, w1
	lsr	w1, w1, 3
.L1933:
	cbz	w1, .L1934
	sub	w1, w1, #1
	uxth	w1, w1
.L1934:
	mov	x0, x24
	bl	List_pop_index_node
	uxth	w22, w0
	ldrh	w0, [x19,-8]
	sub	w0, w0, #1
	strh	w0, [x19,-8]
	strh	w22, [x20]
	mov	x0, x20
	bl	make_superblock
	ldrb	w0, [x20,7]
	cbz	w0, .L1936
	adrp	x25, .LANCHOR0
	mov	x3, 56
	add	x0, x25, :lo12:.LANCHOR0
	mov	w6, 65535
	ldrh	w5, [x0,1952]
	mov	x0, 0
	mov	w26, w0
	b	.L1937
.L1936:
	ubfiz	x0, x22, 1, 16
	ldr	x1, [x19,-40]
	mov	w2, -1
	strh	w2, [x1,x0]
	b	.L1981
.L1940:
	ldr	x4, [x19,-88]
	madd	x1, x0, x3, x4
	str	xzr, [x1,16]
	str	xzr, [x1,8]
	add	x1, x20, x0, lsl 1
	ldrh	w1, [x1,16]
	cmp	w1, w6
	beq	.L1939
	umull	x2, w26, w3
	add	w26, w26, 1
	lsl	w1, w1, 10
	add	x2, x4, x2
	uxth	w26, w26
	str	w1, [x2,4]
.L1939:
	add	x0, x0, 1
.L1937:
	cmp	w5, w0, uxth
	bhi	.L1940
	cmp	x20, x19
	uxtw	x27, w22
	bne	.L1941
	ldr	w0, [x19,160]
	cbz	w0, .L1941
	ldr	x0, [x19,-80]
	ldrh	w0, [x0,x27,lsl 1]
	cmp	w0, 30
	bls	.L1941
	strb	wzr, [x19,8]
.L1941:
	ldrb	w0, [x20,8]
	cbnz	w0, .L1942
	lsl	x0, x27, 1
	ldr	x1, [x19,-80]
	mov	w2, 2
	ldrh	w3, [x1,x0]
	cbz	w3, .L1982
	add	x2, x25, :lo12:.LANCHOR0
	ldrh	w2, [x2,2016]
	add	w2, w3, w2
.L1982:
	strh	w2, [x1,x0]
	mov	w1, 0
	ldr	w0, [x19,204]
	add	w0, w0, 1
	str	w0, [x19,204]
	mov	w0, w22
	bl	ftl_set_blk_mode
	b	.L1945
.L1942:
	lsl	x0, x27, 1
	ldr	x2, [x19,-80]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
	ldr	w0, [x19,208]
	add	w0, w0, 1
	str	w0, [x19,208]
	mov	w0, w22
	bl	ftl_set_blk_mode.part.7
.L1945:
	lsl	x27, x27, 1
	ldr	x0, [x19,-80]
	ldr	w1, [x19,216]
	ldrh	w0, [x0,x27]
	cmp	w0, w1
	bls	.L1946
	str	w0, [x19,216]
.L1946:
	add	x2, x25, :lo12:.LANCHOR0
	ldr	w3, [x19,204]
	ldr	w0, [x19,208]
	ldrh	w1, [x2,2016]
	madd	w0, w1, w3, w0
	ldrh	w1, [x2,1960]
	mov	x3, 56
	udiv	w0, w0, w1
	ldr	x1, [x19,2152]
	str	w0, [x19,212]
	ldr	w0, [x1,16]
	add	w0, w0, 1
	str	w0, [x1,16]
	mov	x0, 0
.L1947:
	cmp	w26, w0, uxth
	bls	.L1983
	ldr	x1, [x19,-88]
	madd	x1, x0, x3, x1
	add	x0, x0, 1
	ldr	w2, [x1,4]
	and	w2, w2, -1024
	str	w2, [x1,4]
	b	.L1947
.L1983:
	ldrb	w1, [x20,8]
	mov	w2, w26
	ldr	x0, [x19,-88]
	mov	x28, 0
	bl	FlashEraseBlocks
	mov	w1, w28
	mov	x3, 56
.L1949:
	cmp	w26, w28, uxth
	bls	.L1984
	mul	x0, x28, x3
	ldr	x2, [x19,-88]
	add	x4, x2, x0
	ldr	w2, [x2,x0]
	cmn	w2, #1
	bne	.L1950
	ldr	w0, [x4,4]
	add	w1, w1, 1
	str	x3, [x29,96]
	lsr	w0, w0, 10
	str	x2, [x29,104]
	str	x1, [x29,112]
	str	x1, [x29,120]
	bl	FtlBbmMapBadBlock
	add	x0, x20, x28, lsl 1
	ldr	x2, [x29,104]
	ldr	x3, [x29,96]
	ldr	x1, [x29,120]
	strh	w2, [x0,16]
	ldrb	w0, [x20,7]
	sub	w0, w0, #1
	strb	w0, [x20,7]
.L1950:
	add	x28, x28, 1
	b	.L1949
.L1984:
	cbz	w1, .L1952
	mov	w0, w22
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1952:
	ldrb	w0, [x20,7]
	cbnz	w0, .L1953
	ldr	x0, [x19,-40]
	mov	w1, -1
	strh	w1, [x0,x27]
.L1981:
	mov	w0, w22
	bl	INSERT_DATA_LIST
	b	.L1931
.L1953:
	add	x25, x25, :lo12:.LANCHOR0
	strh	wzr, [x20,2]
	strb	wzr, [x20,6]
	ldrh	w1, [x25,2026]
	strh	w22, [x20]
	mul	w0, w0, w1
	adrp	x1, .LANCHOR2
	add	x1, x1, :lo12:.LANCHOR2
	uxth	w0, w0
	strh	w0, [x20,4]
	ldr	w2, [x1,196]
	str	w2, [x20,12]
	add	w2, w2, 1
	str	w2, [x1,196]
	ldr	x1, [x1,-40]
	ldrh	w2, [x20]
	strh	w0, [x1,x2,lsl 1]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	allocate_data_superblock, .-allocate_data_superblock
	.align	2
	.global	FtlGcFreeBadSuperBlk
	.type	FtlGcFreeBadSuperBlk, %function
FtlGcFreeBadSuperBlk:
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	x25, [sp,64]
	add	x19, x19, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	uxth	w23, w0
	ldrh	w0, [x19,956]
	stp	x21, x22, [sp,32]
	mov	w21, 0
	cbz	w0, .L1987
	adrp	x22, .LANCHOR0
	adrp	x24, .LC117
	add	x22, x22, :lo12:.LANCHOR0
	add	x24, x24, :lo12:.LC117
.L1995:
	ldrh	w0, [x22,1952]
	cmp	w0, w21
	bls	.L1996
	add	x0, x22, 1984
	mov	w1, w23
	mov	w20, 0
	ldrb	w0, [x0,w21,sxtw]
	bl	V2P_block
	uxth	w25, w0
.L1988:
	ldrh	w0, [x19,956]
	cmp	w0, w20
	bls	.L1997
	add	x0, x19, 960
	ldrh	w0, [x0,w20,sxtw 1]
	cmp	w0, w25
	bne	.L1989
	mov	w1, w25
	mov	x0, x24
	bl	printk
	mov	w0, w25
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	w2, [x19,956]
	mov	w3, w20
.L1990:
	cmp	w3, w2
	bcs	.L1998
	add	x4, x19, 960
	add	w0, w3, 1
	ldrh	w1, [x4,w0,sxtw 1]
	strh	w1, [x4,w3,sxtw 1]
	uxth	w3, w0
	b	.L1990
.L1998:
	sub	w2, w2, #1
	strh	w2, [x19,956]
.L1989:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L1988
.L1997:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1995
.L1996:
	bl	FtlGcReFreshBadBlk
.L1987:
	mov	w0, 0
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlGcFreeBadSuperBlk, .-FtlGcFreeBadSuperBlk
	.align	2
	.global	update_vpc_list
	.type	update_vpc_list, %function
update_vpc_list:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	uxth	w19, w0
	adrp	x0, .LANCHOR2
	add	x1, x0, :lo12:.LANCHOR2
	ubfiz	x2, x19, 1, 16
	mov	x20, x0
	ldr	x3, [x1,-40]
	ldrh	w2, [x3,x2]
	cbnz	w2, .L2000
	ldrh	w3, [x1,296]
	cmp	w3, w19
	bne	.L2001
	mov	w0, -1
	strh	w0, [x1,296]
	b	.L2002
.L2001:
	ldrh	w3, [x0,#:lo12:.LANCHOR2]
	mov	w0, w2
	cmp	w3, w19
	beq	.L2003
	ldrh	w3, [x1,48]
	cmp	w3, w19
	beq	.L2003
	ldrh	w1, [x1,96]
	cmp	w1, w19
	beq	.L2003
.L2002:
	add	x20, x20, :lo12:.LANCHOR2
	mov	w1, w19
	sub	x0, x20, #48
	bl	List_remove_node
	ldrh	w0, [x20,-24]
	sub	w0, w0, #1
	strh	w0, [x20,-24]
	mov	w0, w19
	bl	free_data_superblock
	mov	w0, w19
	bl	FtlGcFreeBadSuperBlk
	mov	w0, 1
	b	.L2003
.L2000:
	mov	w0, w19
	bl	List_update_data_list
	mov	w0, 0
.L2003:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	update_vpc_list, .-update_vpc_list
	.align	2
	.global	decrement_vpc_count
	.type	decrement_vpc_count, %function
decrement_vpc_count:
	stp	x29, x30, [sp, -48]!
	uxth	w1, w0
	mov	w0, 65535
	add	x29, sp, 0
	str	x19, [sp,16]
	cmp	w1, w0
	adrp	x2, .LANCHOR2
	beq	.L2008
	add	x0, x2, :lo12:.LANCHOR2
	ubfiz	x3, x1, 1, 16
	ldr	x0, [x0,-40]
	ldrh	w19, [x0,x3]
	cbnz	w19, .L2009
	adrp	x0, .LC118
	mov	w2, w19
	add	x0, x0, :lo12:.LC118
	bl	printk
	b	.L2013
.L2009:
	sub	w19, w19, #1
	strh	w19, [x0,x3]
.L2008:
	add	x19, x2, :lo12:.LANCHOR2
	mov	w0, 65535
	ldrh	w2, [x19,2320]
	cmp	w2, w0
	bne	.L2011
	strh	w1, [x19,2320]
.L2013:
	mov	w0, 0
	b	.L2010
.L2011:
	cmp	w2, w1
	str	x1, [x29,40]
	mov	w0, 0
	beq	.L2010
	mov	w0, w2
	bl	update_vpc_list
	cmp	w0, wzr
	ldr	x1, [x29,40]
	cset	w0, ne
	strh	w1, [x19,2320]
.L2010:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	decrement_vpc_count, .-decrement_vpc_count
	.align	2
	.global	FtlSlcSuperblockCheck
	.type	FtlSlcSuperblockCheck, %function
FtlSlcSuperblockCheck:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	ldrh	w1, [x0,4]
	cbz	w1, .L2014
	ldrh	w1, [x0]
	mov	w20, 65535
	cmp	w1, w20
	beq	.L2014
	mov	x19, x0
	ldrb	w0, [x0,6]
	adrp	x21, .LANCHOR2
	adrp	x22, .LANCHOR0
	add	x0, x0, 8
	add	x21, x21, :lo12:.LANCHOR2
	add	x21, x21, 996
	ldrh	w0, [x19,x0,lsl 1]
.L2018:
	cmp	w0, w20
	bne	.L2025
.L2020:
	ldrb	w0, [x19,6]
	add	x1, x22, :lo12:.LANCHOR0
	add	w0, w0, 1
	ldrh	w1, [x1,1952]
	uxtb	w0, w0
	strb	w0, [x19,6]
	cmp	w1, w0
	bne	.L2019
	ldrh	w0, [x19,2]
	strb	wzr, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
.L2019:
	ldrb	w0, [x19,6]
	add	x0, x0, 8
	ldrh	w0, [x19,x0,lsl 1]
	b	.L2018
.L2025:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2021
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbnz	w0, .L2021
	ldrh	w0, [x19,2]
	ldrh	w0, [x21,w0,sxtw 1]
	cmp	w0, w20
	bne	.L2021
	ldrh	w0, [x19,4]
	sub	w0, w0, #1
	strh	w0, [x19,4]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	ldrh	w1, [x19,4]
	cbnz	w1, .L2020
	ldrh	w0, [x19,2]
	strb	w1, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
	b	.L2014
.L2021:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w2, [x0,120]
	cbz	w2, .L2014
	cmp	w1, 1
	bne	.L2014
	ldrh	w2, [x19,2]
	ldrh	w1, [x0,2028]
	cmp	w2, w1
	bcc	.L2014
	adrp	x2, .LANCHOR2-40
	ldrh	w1, [x19]
	ldrh	w4, [x19,4]
	lsl	x1, x1, 1
	ldr	x3, [x2,#:lo12:.LANCHOR2-40]
	ldrh	w2, [x3,x1]
	sub	w2, w2, w4
	strh	w2, [x3,x1]
	strh	wzr, [x19,4]
	ldrh	w0, [x0,2026]
	strh	w0, [x19,2]
	strb	wzr, [x19,6]
.L2014:
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
	stp	x21, x22, [sp,32]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	mov	x19, x0
	strb	wzr, [x0,10]
	ldrb	w0, [x0,6]
	adrp	x21, .LANCHOR2
	add	x21, x21, :lo12:.LANCHOR2
	mov	w22, 65535
	add	x0, x0, 8
	adrp	x23, .LANCHOR0
	add	x21, x21, 996
	ldrh	w0, [x19,x0,lsl 1]
.L2027:
	cmp	w0, w22
	adrp	x20, .LANCHOR0
	bne	.L2042
.L2028:
	ldrb	w0, [x19,6]
	add	x1, x23, :lo12:.LANCHOR0
	add	w0, w0, 1
	ldrh	w1, [x1,1952]
	uxtb	w0, w0
	strb	w0, [x19,6]
	cmp	w1, w0
	bne	.L2029
	ldrh	w0, [x19,2]
	strb	wzr, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
.L2029:
	ldrb	w0, [x19,6]
	add	x0, x0, 8
	ldrh	w0, [x19,x0,lsl 1]
	b	.L2027
.L2042:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2030
	add	x1, x23, :lo12:.LANCHOR0
	ldrb	w1, [x1,120]
	cbnz	w1, .L2030
	ldrh	w1, [x19,2]
	ldrh	w1, [x21,w1,sxtw 1]
	cmp	w1, w22
	bne	.L2030
	ldrh	w0, [x19,4]
	sub	w0, w0, #1
	strh	w0, [x19,4]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L2028
.L2030:
	ldrh	w1, [x19,4]
	adrp	x21, .LANCHOR2
	ldrh	w22, [x19,2]
	mov	w23, 65535
	add	x21, x21, :lo12:.LANCHOR2
	sub	w1, w1, #1
	orr	w22, w22, w0, lsl 10
	strh	w1, [x19,4]
	mov	w24, w23
	add	x21, x21, 996
.L2031:
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w1, [x19,6]
	ldrh	w2, [x0,1952]
.L2033:
	add	w1, w1, 1
	uxtb	w1, w1
	cmp	w1, w2
	bne	.L2032
	ldrh	w1, [x19,2]
	add	w1, w1, 1
	strh	w1, [x19,2]
	mov	w1, 0
.L2032:
	add	x0, x19, x1, sxtw 1
	ldrh	w0, [x0,16]
	cmp	w0, w23
	beq	.L2033
	ldrb	w0, [x19,8]
	strb	w1, [x19,6]
	cmp	w0, 1
	bne	.L2038
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbnz	w0, .L2035
	ldrh	w0, [x19,2]
	ldrh	w0, [x21,w0,sxtw 1]
	cmp	w0, w24
	bne	.L2035
	ldrh	w1, [x19,4]
	cbz	w1, .L2035
	ldrh	w0, [x19]
	sub	w1, w1, #1
	strh	w1, [x19,4]
	bl	decrement_vpc_count
	b	.L2031
.L2035:
	add	x20, x20, :lo12:.LANCHOR0
	ldrb	w0, [x20,120]
	cbz	w0, .L2038
	ldrh	w1, [x19,2]
	ldrh	w0, [x20,2028]
	cmp	w1, w0
	bcc	.L2038
	adrp	x0, .LANCHOR2-40
	ldrh	w1, [x19]
	ldrh	w3, [x19,4]
	lsl	x1, x1, 1
	ldr	x0, [x0,#:lo12:.LANCHOR2-40]
	ldrh	w2, [x0,x1]
	sub	w2, w2, w3
	strh	w2, [x0,x1]
	strh	wzr, [x19,4]
	ldrh	w0, [x20,2026]
	strh	w0, [x19,2]
	strb	wzr, [x19,6]
.L2038:
	mov	w0, w22
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.global	FtlMakeBbt
	.type	FtlMakeBbt, %function
FtlMakeBbt:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	adrp	x20, .LANCHOR4
	stp	x21, x22, [sp,32]
	add	x20, x20, :lo12:.LANCHOR4
	add	x22, x19, :lo12:.LANCHOR0
	add	x22, x22, 2112
	add	x20, x20, 136
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	mov	x24, x22
	mov	w23, 0
	mov	x25, x20
	stp	x27, x28, [sp,80]
	bl	FtlBbtMemInit
	bl	FtlLoadFactoryBbt
.L2044:
	add	x21, x19, :lo12:.LANCHOR0
	ldrh	w0, [x21,1974]
	cmp	w23, w0
	bcs	.L2062
	adrp	x1, .LANCHOR2
	ldrh	w27, [x24,12]
	add	x2, x1, :lo12:.LANCHOR2
	str	x1, [x29,104]
	ldr	x0, [x2,-64]
	ldr	x26, [x2,2112]
	mov	w2, 65535
	str	x0, [x20,8]
	cmp	w27, w2
	str	x26, [x20,16]
	beq	.L2045
	ldrh	w0, [x21,2022]
	mov	w1, 1
	mov	w2, w1
	madd	w27, w23, w0, w27
	lsl	w0, w27, 10
	str	w0, [x20,4]
	mov	x0, x20
	bl	FlashReadPages
	ldrh	w2, [x21,2022]
	ldr	x0, [x22,32]
	add	w2, w2, 7
	ldr	x1, [x20,8]
	lsr	w2, w2, 3
	bl	ftl_memcpy
	b	.L2046
.L2045:
	mov	w1, w23
	mov	w28, -3872
	bl	FlashGetBadBlockList
	ldr	x0, [x20,8]
	ldr	x1, [x22,32]
	bl	FtlBbt2Bitmap
	ldrh	w21, [x21,2022]
	sub	w21, w21, #1
	uxth	w21, w21
.L2047:
	add	x27, x19, :lo12:.LANCHOR0
	ldrh	w0, [x27,2022]
	madd	w0, w23, w0, w21
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	bne	.L2063
	sub	w21, w21, #1
	uxth	w21, w21
	b	.L2047
.L2063:
	ldr	x0, [x29,104]
	mov	w1, 0
	mov	w2, 16
	strh	w21, [x24,12]
	add	x3, x0, :lo12:.LANCHOR2
	str	x3, [x29,96]
	ldr	x0, [x3,2112]
	bl	ftl_memset
	strh	w28, [x26]
	str	wzr, [x26,4]
	ldr	x3, [x29,96]
	ldrh	w0, [x24,12]
	strh	w0, [x26,2]
	ldrh	w0, [x27,2022]
	ldrh	w1, [x24,12]
	ldrh	w2, [x3,2232]
	madd	w27, w23, w0, w1
	ldr	x1, [x22,32]
	lsl	w2, w2, 2
	lsl	w0, w27, 10
	str	w0, [x25,4]
	ldr	x0, [x25,8]
	bl	ftl_memcpy
	mov	w1, 1
	mov	x0, x25
	mov	w2, w1
	bl	FlashEraseBlocks
	mov	w1, 1
	mov	x0, x25
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
	ldr	w0, [x25]
	cmn	w0, #1
	bne	.L2046
	mov	w0, w27
	bl	FtlBbmMapBadBlock
	b	.L2047
.L2046:
	mov	w0, w27
	add	w23, w23, 1
	add	x22, x22, 8
	add	x24, x24, 2
	bl	FtlBbmMapBadBlock
	b	.L2044
.L2062:
	mov	w20, 0
.L2051:
	add	x21, x19, :lo12:.LANCHOR0
	ldrh	w0, [x21,2040]
	cmp	w0, w20
	bls	.L2064
	mov	w0, w20
	add	w20, w20, 1
	bl	FtlBbmMapBadBlock
	uxth	w20, w20
	b	.L2051
.L2064:
	ldrh	w20, [x21,2124]
	mov	w23, 65535
	sub	w20, w20, #1
	uxth	w20, w20
.L2053:
	add	x22, x21, 2112
	ldrh	w0, [x22,12]
	sub	w0, w0, #48
	cmp	w20, w0
	ble	.L2057
	mov	w0, w20
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	beq	.L2054
	mov	w0, w20
	bl	FlashTestBlk
	cbz	w0, .L2055
	mov	w0, w20
	bl	FtlBbmMapBadBlock
	b	.L2054
.L2055:
	ldrh	w0, [x21,2112]
	cmp	w0, w23
	bne	.L2056
	strh	w20, [x21,2112]
	b	.L2054
.L2056:
	strh	w20, [x22,4]
	b	.L2057
.L2054:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L2053
.L2057:
	adrp	x0, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR0
	add	x0, x0, :lo12:.LANCHOR2
	ldrh	w1, [x19,2112]
	ldr	x2, [x0,-88]
	str	wzr, [x19,2120]
	lsl	w1, w1, 10
	strh	wzr, [x19,2114]
	str	w1, [x2,4]
	mov	w2, 2
	ldrh	w1, [x19,2116]
	ldr	x0, [x0,-88]
	lsl	w1, w1, 10
	str	w1, [x0,60]
	mov	w1, 1
	bl	FlashEraseBlocks
	ldrh	w0, [x19,2112]
	bl	FtlBbmMapBadBlock
	ldrh	w0, [x19,2116]
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	strh	wzr, [x19,2114]
	ldr	w0, [x19,2120]
	ldrh	w1, [x19,2116]
	add	w0, w0, 1
	str	w0, [x19,2120]
	ldrh	w0, [x19,2112]
	strh	w0, [x19,2116]
	strh	w1, [x19,2112]
	bl	FtlBbmTblFlush
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlMakeBbt, .-FtlMakeBbt
	.align	2
	.global	Ftl_write_map_blk_to_last_page
	.type	Ftl_write_map_blk_to_last_page, %function
Ftl_write_map_blk_to_last_page:
	stp	x29, x30, [sp, -64]!
	mov	w1, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	str	x23, [sp,48]
	mov	x19, x0
	ldr	x20, [x0,16]
	ldr	x23, [x0,40]
	ldrh	w0, [x0]
	cmp	w0, w1
	bne	.L2066
	ldrh	w0, [x19,8]
	add	w0, w0, 1
	strh	w0, [x19,8]
	bl	FtlFreeSysBlkQueueOut
	strh	w0, [x20]
	strh	wzr, [x19,2]
	ldr	w0, [x19,48]
	strh	wzr, [x19]
	add	w0, w0, 1
	str	w0, [x19,48]
	b	.L2067
.L2066:
	ubfiz	x0, x0, 1, 16
	adrp	x21, .LANCHOR2
	add	x3, x21, :lo12:.LANCHOR2
	ldrh	w22, [x20,x0]
	adrp	x20, .LANCHOR4
	ldrh	w0, [x19,2]
	add	x1, x20, :lo12:.LANCHOR4
	orr	w0, w0, w22, lsl 10
	str	w0, [x1,140]
	ldr	x0, [x3,-64]
	str	x0, [x1,144]
	ldr	x0, [x3,2112]
	str	x0, [x1,152]
	ldr	w1, [x19,48]
	str	w1, [x0,4]
	mov	w1, -1291
	strh	w1, [x0,8]
	ldrh	w1, [x19,4]
	strh	w1, [x0]
	mov	w1, 255
	strh	w22, [x0,2]
	adrp	x0, .LANCHOR0+2028
	ldrh	w2, [x0,#:lo12:.LANCHOR0+2028]
	ldr	x0, [x3,-64]
	lsl	w2, w2, 3
	bl	ftl_memset
	mov	w0, 0
	mov	w1, w0
.L2068:
	ldrh	w2, [x19,6]
	cmp	w2, w1
	bls	.L2071
	ubfiz	x4, x1, 2, 16
	ldr	w2, [x23,x4]
	cmp	w22, w2, lsr 10
	bne	.L2069
	add	x3, x21, :lo12:.LANCHOR2
	add	w0, w0, 1
	uxth	w0, w0
	ldr	x5, [x3,-64]
	ubfiz	x2, x0, 3, 16
	str	w1, [x5,x2]
	ldr	x3, [x3,-64]
	ldr	w4, [x23,x4]
	add	x2, x3, x2
	str	w4, [x2,4]
.L2069:
	add	w1, w1, 1
	uxth	w1, w1
	b	.L2068
.L2071:
	mov	w1, 1
	add	x0, x20, :lo12:.LANCHOR4
	add	x0, x0, 136
	mov	w2, w1
	mov	w3, 0
	bl	FlashProgPages
	ldrh	w0, [x19,2]
	add	w0, w0, 1
	strh	w0, [x19,2]
	mov	x0, x19
	bl	ftl_map_blk_gc
.L2067:
	mov	w0, 0
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	Ftl_write_map_blk_to_last_page, .-Ftl_write_map_blk_to_last_page
	.align	2
	.global	FtlMapWritePage
	.type	FtlMapWritePage, %function
FtlMapWritePage:
	stp	x29, x30, [sp, -112]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR4
	mov	w20, 0
	stp	x21, x22, [sp,32]
	add	x19, x19, :lo12:.LANCHOR4
	adrp	x21, .LC119
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	stp	x25, x26, [sp,64]
	mov	x28, x0
	mov	w22, w1
	mov	x23, x2
	adrp	x24, .LANCHOR2
	add	x19, x19, 136
	add	x21, x21, :lo12:.LC119
.L2073:
	adrp	x26, .LANCHOR0
	adrp	x25, .LANCHOR2
	mov	w27, 65535
.L2076:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	w1, [x0,180]
	add	w1, w1, 1
	str	w1, [x0,180]
	add	x1, x26, :lo12:.LANCHOR0
	ldrh	w0, [x28,2]
	ldrh	w1, [x1,2028]
	sub	w1, w1, #1
	cmp	w0, w1
	bge	.L2074
	ldrh	w0, [x28]
	cmp	w0, w27
	bne	.L2075
.L2074:
	mov	x0, x28
	bl	Ftl_write_map_blk_to_last_page
.L2075:
	ldrh	w1, [x28]
	mov	w2, 16
	ldr	x0, [x28,16]
	ldrh	w3, [x0,x1,lsl 1]
	add	x0, x25, :lo12:.LANCHOR2
	ldrh	w1, [x28,2]
	str	x3, [x29,104]
	orr	w1, w1, w3, lsl 10
	ldr	x0, [x0,2112]
	str	w1, [x19,4]
	mov	w1, 0
	str	x23, [x19,8]
	str	x0, [x19,16]
	bl	ftl_memset
	ldr	x1, [x19,16]
	ldr	w0, [x28,48]
	ldr	x3, [x29,104]
	str	w0, [x1,4]
	strh	w22, [x1,8]
	ldrh	w0, [x28,4]
	strh	w0, [x1]
	mov	x0, x19
	strh	w3, [x1,2]
	mov	w1, 1
	mov	w3, w1
	mov	w2, w1
	bl	FlashProgPages
	ldrh	w3, [x28,2]
	add	w3, w3, 1
	uxth	w3, w3
	strh	w3, [x28,2]
	cmp	w3, 1
	beq	.L2076
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L2077
	ldr	w1, [x19,4]
	mov	x0, x21
	add	w20, w20, 1
	bl	printk
	uxth	w20, w20
	cmp	w20, 4
	bne	.L2073
	adrp	x0, .LC120
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC120
	mov	w2, w20
	bl	printk
.L2078:
	b	.L2078
.L2077:
	ldr	x0, [x28,40]
	ldr	w1, [x19,4]
	str	w1, [x0,w22,uxtw 2]
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
	stp	x29, x30, [sp, -80]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	mov	x19, x0
	adrp	x22, .LANCHOR0
	ldr	x20, [x0,16]
	ldr	x23, [x0,40]
	bl	ftl_free_no_use_map_blk
	ldrh	w1, [x19,10]
	ldrh	w2, [x19,8]
	sub	w1, w1, #4
	cmp	w2, w1
	blt	.L2083
	ubfiz	x1, x0, 1, 16
	ldrh	w24, [x20,x1]
	cbz	w24, .L2083
	ldr	w0, [x19,52]
	cbnz	w0, .L2083
	mov	w2, 1
	str	w2, [x19,52]
	strh	w0, [x20,x1]
	ldrh	w0, [x19,8]
	ldrh	w1, [x19,2]
	sub	w0, w0, #1
	strh	w0, [x19,8]
	add	x0, x22, :lo12:.LANCHOR0
	ldrh	w0, [x0,2028]
	cmp	w1, w0
	bcc	.L2084
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L2084:
	adrp	x20, .LANCHOR4
	mov	w21, 0
	add	x20, x20, :lo12:.LANCHOR4
	adrp	x26, .LANCHOR2
	add	x20, x20, 136
.L2085:
	ldrh	w0, [x19,6]
	cmp	w0, w21
	bls	.L2093
	ubfiz	x25, x21, 2, 16
	ldr	w0, [x23,x25]
	cmp	w24, w0, lsr 10
	bne	.L2086
	add	x0, x26, :lo12:.LANCHOR2
	ldr	x1, [x0,2072]
	ldr	x0, [x0,2112]
	str	x1, [x20,8]
	mov	w1, 1
	str	x0, [x20,16]
	mov	w2, w1
	ldr	w0, [x23,x25]
	str	w0, [x20,4]
	mov	x0, x20
	bl	FlashReadPages
	ldr	w0, [x20]
	cmn	w0, #1
	bne	.L2087
	str	wzr, [x23,x25]
	b	.L2086
.L2087:
	ldr	x2, [x20,8]
	mov	x0, x19
	mov	w1, w21
	bl	FtlMapWritePage
.L2086:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L2085
.L2093:
	mov	w0, w24
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
	str	wzr, [x19,52]
.L2083:
	add	x22, x22, :lo12:.LANCHOR0
	ldrh	w1, [x19,2]
	ldrh	w0, [x22,2028]
	cmp	w1, w0
	bcc	.L2089
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L2089:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
	.align	2
	.global	FtlMapTblRecovery
	.type	FtlMapTblRecovery, %function
FtlMapTblRecovery:
	stp	x29, x30, [sp, -144]!
	mov	w1, 0
	add	x29, sp, 0
	stp	x27, x28, [sp,80]
	mov	x27, x0
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	ldrh	w22, [x27,6]
	adrp	x19, .LANCHOR2
	ldr	x20, [x0,40]
	mov	w25, 0
	ldr	x21, [x0,16]
	lsl	w2, w22, 2
	ldr	x0, [x0,24]
	adrp	x24, .LANCHOR0
	str	x0, [x29,128]
	mov	x0, x20
	ldrh	w23, [x27,8]
	bl	ftl_memset
	str	x19, [x29,136]
	add	x1, x19, :lo12:.LANCHOR2
	adrp	x0, .LANCHOR4
	add	x4, x0, :lo12:.LANCHOR4
	str	x0, [x29,120]
	add	x4, x4, 136
	sub	w0, w23, #1
	ldr	x2, [x1,-64]
	ldr	x26, [x1,2112]
	mov	w1, -1
	str	x2, [x4,8]
	str	x26, [x4,16]
	mov	x19, x4
	strh	w1, [x27]
	strh	w1, [x27,2]
	mov	w1, 1
	str	wzr, [x27,52]
	str	wzr, [x27,48]
	str	w1, [x27,56]
	str	w0, [x29,116]
.L2095:
	cmp	w25, w23
	bge	.L2112
	ldr	w0, [x29,116]
	sxtw	x28, w25
	cmp	w25, w0
	bne	.L2096
	lsl	x0, x28, 1
	mov	w1, 1
	add	x23, x21, x0
	ldrh	w0, [x21,x0]
	mov	w21, 0
	bl	FtlGetLastWrittenPage
	sxth	w24, w0
	add	w0, w0, 1
	strh	w0, [x27,2]
	ldr	x0, [x29,128]
	add	w24, w24, 1
	strh	w25, [x27]
	ldr	w0, [x0,x28,lsl 2]
	str	w0, [x27,48]
	ldr	x0, [x29,120]
	add	x19, x0, :lo12:.LANCHOR4
	add	x19, x19, 136
	str	xzr, [x19,8]
.L2097:
	cmp	w21, w24
	bge	.L2112
	ldrh	w0, [x23]
	mov	w1, 1
	mov	w2, w1
	orr	w0, w21, w0, lsl 10
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L2098
	ldrh	w0, [x26,8]
	cmp	w0, w22
	bcs	.L2098
	ldrh	w2, [x26]
	ldrh	w1, [x27,4]
	cmp	w2, w1
	bne	.L2098
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x19,4]
	str	w1, [x20,x0]
.L2098:
	add	w21, w21, 1
	sxth	w21, w21
	b	.L2097
.L2112:
	mov	x0, x27
	bl	ftl_free_no_use_map_blk
	adrp	x0, .LANCHOR0+2028
	ldrh	w1, [x27,2]
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2028]
	cmp	w1, w0
	bne	.L2101
	mov	x0, x27
	bl	ftl_map_blk_alloc_new_blk
	b	.L2101
.L2096:
	ldr	x0, [x29,136]
	add	x0, x0, :lo12:.LANCHOR2
	ldr	x0, [x0,-64]
	str	x0, [x19,8]
	lsl	x0, x28, 1
	add	x28, x21, x0
	ldrh	w1, [x21,x0]
	add	x0, x24, :lo12:.LANCHOR0
	ldrh	w0, [x0,2028]
	sub	w0, w0, #1
	orr	w0, w0, w1, lsl 10
	mov	w1, 1
	str	w0, [x19,4]
	mov	w2, w1
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L2102
	ldrh	w1, [x26]
	ldrh	w0, [x27,4]
	cmp	w1, w0
	bne	.L2102
	ldrh	w1, [x26,8]
	mov	w0, 64245
	cmp	w1, w0
	bne	.L2102
	mov	w0, 0
.L2103:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w1, [x1,2028]
	sub	w1, w1, #1
	cmp	w0, w1
	bge	.L2106
	ldr	x1, [x29,136]
	sbfiz	x2, x0, 3, 32
	add	x1, x1, :lo12:.LANCHOR2
	ldr	x5, [x1,-64]
	ldrh	w1, [x5,x2]
	cmp	w1, w22
	bcs	.L2104
	add	x2, x5, x2
	ubfiz	x1, x1, 2, 16
	ldr	w2, [x2,4]
	str	w2, [x20,x1]
.L2104:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L2103
.L2102:
	str	xzr, [x19,8]
	mov	w5, 0
.L2107:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w0, [x0,2028]
	cmp	w5, w0
	bge	.L2106
	ldrh	w0, [x28]
	mov	w1, 1
	mov	w2, w1
	str	x5, [x29,104]
	orr	w0, w5, w0, lsl 10
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	ldr	x5, [x29,104]
	cmn	w0, #1
	beq	.L2108
	ldrh	w0, [x26,8]
	cmp	w0, w22
	bcs	.L2108
	ldrh	w2, [x26]
	ldrh	w1, [x27,4]
	cmp	w2, w1
	bne	.L2108
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x19,4]
	str	w1, [x20,x0]
.L2108:
	add	w5, w5, 1
	sxth	w5, w5
	b	.L2107
.L2106:
	add	w6, w25, 1
	sxth	w25, w6
	b	.L2095
.L2101:
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
	adrp	x0, .LANCHOR4
	adrp	x1, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR4
	add	x1, x1, :lo12:.LANCHOR0
	stp	x29, x30, [sp, -16]!
	add	x0, x0, 200
	add	x29, sp, 0
	ldrh	w2, [x1,2048]
	strh	w2, [x0,10]
	mov	w2, -3962
	strh	w2, [x0,4]
	ldrh	w2, [x1,2072]
	strh	w2, [x0,8]
	ldrh	w2, [x1,2050]
	ldr	x1, [x1,2080]
	str	x1, [x0,16]
	adrp	x1, .LANCHOR2
	add	x1, x1, :lo12:.LANCHOR2
	strh	w2, [x0,6]
	ldr	x2, [x1,2192]
	str	x2, [x0,24]
	ldr	x2, [x1,2184]
	ldr	x1, [x1,2200]
	str	x2, [x0,32]
	str	x1, [x0,40]
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
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x0, x0, 2240
	bl	FtlMapTblRecovery
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlLoadMapInfo, .-FtlLoadMapInfo
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
	add	x0, x20, 2240
	ldr	x1, [x20,144]
	add	x2, x1, x19
	ldrh	w1, [x1,x19]
	ldr	x2, [x2,8]
	bl	FtlMapWritePage
	ldr	x0, [x20,144]
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
	.global	log2phys
	.type	log2phys, %function
log2phys:
	stp	x29, x30, [sp, -96]!
	adrp	x5, .LANCHOR0
	mov	x4, 1
	add	x5, x5, :lo12:.LANCHOR0
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x19, x20, [sp,16]
	str	x25, [sp,64]
	adrp	x23, .LANCHOR2
	mov	x21, x1
	add	x1, x23, :lo12:.LANCHOR2
	ldrh	w3, [x5,2034]
	mov	w22, w2
	add	w3, w3, 7
	ldr	x25, [x1,144]
	lsr	w20, w0, w3
	lsl	x3, x4, x3
	sub	w3, w3, #1
	mov	x4, 0
	and	w0, w3, w0
	uxth	w20, w20
	uxth	x24, w0
	ldrh	w0, [x5,2066]
.L2127:
	uxth	x19, w4
	cmp	w19, w0
	bcs	.L2139
	add	x4, x4, 1
	add	x1, x25, x4, lsl 4
	ldrh	w1, [x1,-16]
	cmp	w1, w20
	bne	.L2127
.L2128:
	cbnz	w22, .L2129
	add	x0, x23, :lo12:.LANCHOR2
	ldr	x1, [x0,144]
	add	x1, x1, x19, lsl 4
	ldr	x0, [x1,8]
	ldr	w0, [x0,x24,lsl 2]
	str	w0, [x21]
	b	.L2130
.L2129:
	add	x1, x23, :lo12:.LANCHOR2
	lsl	x0, x19, 4
	ldr	w3, [x21]
	ldr	x2, [x1,144]
	add	x2, x2, x0
	ldr	x2, [x2,8]
	str	w3, [x2,x24,lsl 2]
	strh	w20, [x1,152]
	ldr	x2, [x1,144]
	add	x0, x2, x0
	ldr	w2, [x0,4]
	orr	w2, w2, -2147483648
	str	w2, [x0,4]
.L2130:
	add	x23, x23, :lo12:.LANCHOR2
	ldr	x0, [x23,144]
	add	x19, x0, x19, lsl 4
	ldr	w0, [x19,4]
	cmn	w0, #1
	beq	.L2136
	add	w0, w0, 1
	str	w0, [x19,4]
	b	.L2136
.L2139:
	bl	select_l2p_ram_region
	uxth	x19, w0
	ubfiz	x2, x19, 4, 16
	mov	w1, w0
	add	x3, x25, x2
	ldrh	w4, [x25,x2]
	mov	w2, 65535
	cmp	w4, w2
	beq	.L2133
	ldr	w2, [x3,4]
	tbz	w2, #31, .L2133
	str	x1, [x29,88]
	bl	flush_l2p_region
	ldr	x1, [x29,88]
.L2133:
	mov	w0, w20
	bl	load_l2p_region
	b	.L2128
.L2136:
	mov	w0, 0
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 96
	ret
	.size	log2phys, .-log2phys
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
	ldr	x3, [x2,-40]
	ldrh	w1, [x3,x22]
	cbnz	w1, .L2141
	ldr	x19, [x2,-16]
	cbz	x19, .L2142
	ldrh	w3, [x2,-8]
	mov	x4, -6148914691236517206
	ldr	x2, [x2,-56]
	mov	w5, 65535
	sub	x19, x19, x2
	asr	x19, x19, 1
	madd	x19, x4, x19, x19
	mov	w4, 6
	uxth	w19, w19
.L2143:
	cmp	w1, w3
	beq	.L2142
	cmp	w19, w0
	bne	.L2144
	add	x21, x21, :lo12:.LANCHOR2
	mov	w1, w19
	sub	x0, x21, #16
	bl	List_remove_node
	ldrh	w0, [x21,-8]
	sub	w0, w0, #1
	strh	w0, [x21,-8]
	mov	w0, w19
	bl	INSERT_DATA_LIST
	ldr	x1, [x21,-40]
	ldrh	w0, [x1,x22]
	add	w0, w0, 1
	strh	w0, [x1,x22]
	b	.L2142
.L2144:
	umull	x19, w19, w4
	ldrh	w19, [x2,x19]
	cmp	w19, w5
	beq	.L2142
	add	w1, w1, 1
	uxth	w1, w1
	b	.L2143
.L2141:
	add	w1, w1, 1
	strh	w1, [x3,x22]
.L2142:
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
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	stp	x29, x30, [sp, -112]!
	adrp	x0, .LANCHOR2
	add	x1, x0, :lo12:.LANCHOR2
	add	x29, sp, 0
	add	x2, x1, 352
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	adrp	x20, .LANCHOR0
	ldr	w3, [x2,80]
	cbz	w3, .L2150
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w3, [x2,84]
	ldr	w0, [x0,2104]
	cmp	w3, w0
	bcs	.L2151
	mov	w21, 2048
	mov	x19, x1
.L2156:
	add	x1, x20, :lo12:.LANCHOR0
	ldr	w0, [x19,436]
	ldr	w1, [x1,2104]
	cmp	w0, w1
	bcs	.L2153
	add	x1, x29, 52
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x19,436]
	ldr	w1, [x29,52]
	add	w0, w0, 1
	str	w0, [x19,436]
	cmn	w1, #1
	beq	.L2154
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
	bne	.L2153
	ldr	w0, [x29,52]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L2153
.L2154:
	subs	w21, w21, #1
	bne	.L2156
.L2153:
	mov	w0, -1
	b	.L2158
.L2151:
	ldr	w0, [x1,164]
	str	wzr, [x2,80]
	str	wzr, [x2,84]
	str	w0, [x2,76]
	b	.L2162
.L2150:
	add	x20, x20, :lo12:.LANCHOR0
	ldr	w3, [x1,216]
	mov	w2, 1048576
	ldr	w1, [x1,160]
	ldrb	w5, [x20,120]
	cbnz	w5, .L2159
	add	w1, w1, w3, lsr 10
	mov	w3, 33554432
	asr	w2, w3, w1
.L2159:
	add	x4, x0, :lo12:.LANCHOR2
	ldr	w3, [x4,164]
	ldr	w1, [x4,428]
	add	w6, w3, 1048576
	cmp	w1, w6
	bhi	.L2160
	add	w1, w2, w1
	cmp	w1, w3
	bcc	.L2160
	ldrh	w1, [x4,252]
	and	w1, w1, 63
	cbnz	w1, .L2162
	cbz	w5, .L2162
.L2160:
	add	x0, x0, :lo12:.LANCHOR2
	mov	w1, 1
	str	wzr, [x0,436]
	str	w1, [x0,432]
	str	w3, [x0,428]
.L2162:
	mov	w0, 0
.L2158:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlReadRefresh, .-FtlReadRefresh
	.align	2
	.global	FtlVendorPartWrite
	.type	FtlVendorPartWrite, %function
FtlVendorPartWrite:
	stp	x29, x30, [sp, -176]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	adrp	x23, .LANCHOR0
	stp	x25, x26, [sp,64]
	mov	w26, w0
	add	x0, x23, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	stp	x21, x22, [sp,32]
	mov	w28, w1
	add	w1, w26, w1
	ldrh	w21, [x0,2034]
	mov	x25, x2
	ldrh	w0, [x0,2020]
	mov	w20, -1
	cmp	w1, w0
	bhi	.L2171
	adrp	x24, .LANCHOR4
	lsr	w21, w26, w21
	add	x24, x24, :lo12:.LANCHOR4
	mov	w20, 0
	adrp	x27, .LANCHOR2
	add	x24, x24, 200
.L2172:
	cbz	w28, .L2171
	adrp	x0, .LANCHOR2
	add	x2, x0, :lo12:.LANCHOR2
	ldr	x0, [x2,2200]
	ldr	w4, [x0,w21,uxtw 2]
	add	x0, x23, :lo12:.LANCHOR0
	ldrh	w1, [x0,2032]
	uxth	w0, w28
	udiv	w22, w26, w1
	msub	w22, w22, w1, w26
	sub	w19, w1, w22
	uxth	w19, w19
	cmp	w19, w28
	csel	w19, w0, w19, hi
	cmp	w19, w1
	beq	.L2174
	cbz	w4, .L2174
	ldr	x0, [x2,2080]
	mov	w1, 1
	str	x0, [x29,128]
	mov	w2, w1
	add	x0, x29, 120
	str	w4, [x29,124]
	str	xzr, [x29,136]
	bl	FlashReadPages
	b	.L2175
.L2174:
	add	x0, x27, :lo12:.LANCHOR2
	add	x2, x23, :lo12:.LANCHOR0
	mov	w1, 0
	ldr	x0, [x0,2080]
	ldrh	w2, [x2,2036]
	bl	ftl_memset
.L2175:
	add	x5, x27, :lo12:.LANCHOR2
	lsl	w4, w19, 9
	ubfiz	x22, x22, 9, 16
	mov	w2, w4
	mov	x1, x25
	str	x4, [x29,104]
	ldr	x0, [x5,2080]
	sub	w28, w28, w19
	str	x5, [x29,96]
	add	w26, w26, w19
	add	x0, x0, x22
	bl	ftl_memcpy
	ldr	x5, [x29,96]
	mov	w1, w21
	mov	x0, x24
	add	w21, w21, 1
	ldr	x2, [x5,2080]
	bl	FtlMapWritePage
	cmn	w0, #1
	ldr	x4, [x29,104]
	csinv	w20, w20, wzr, ne
	add	x25, x25, x4, sxtw
	b	.L2172
.L2171:
	mov	w0, w20
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 176
	ret
	.size	FtlVendorPartWrite, .-FtlVendorPartWrite
	.align	2
	.global	Ftl_save_ext_data
	.type	Ftl_save_ext_data, %function
Ftl_save_ext_data:
	adrp	x0, .LANCHOR2
	mov	w1, 19539
	add	x0, x0, :lo12:.LANCHOR2
	movk	w1, 0x4654, lsl 16
	stp	x29, x30, [sp, -16]!
	add	x2, x0, 352
	add	x29, sp, 0
	ldr	w3, [x0,352]
	cmp	w3, w1
	bne	.L2185
	mov	w1, 64
	movk	w1, 0x5000, lsl 16
	str	w1, [x2,4]
	ldr	w1, [x0,188]
	str	w1, [x2,88]
	ldr	w1, [x0,192]
	str	w1, [x2,92]
	ldr	w1, [x0,184]
	str	w1, [x2,8]
	ldr	w1, [x0,172]
	str	w1, [x2,12]
	ldr	w1, [x0,164]
	str	w1, [x2,16]
	ldr	w1, [x0,180]
	str	w1, [x2,20]
	ldr	w1, [x0,208]
	str	w1, [x2,28]
	ldr	w1, [x0,-72]
	str	w1, [x2,32]
	ldr	w1, [x0,168]
	str	w1, [x2,36]
	ldr	w1, [x0,176]
	str	w1, [x2,40]
	ldr	w1, [x0,216]
	str	w1, [x2,44]
	ldr	w1, [x0,220]
	str	w1, [x2,48]
	ldr	w1, [x0,344]
	ldr	w0, [x0,2024]
	str	w1, [x2,60]
	mov	w1, 1
	str	w0, [x2,64]
	mov	w0, 0
	bl	FtlVendorPartWrite
.L2185:
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
	ldr	w4, [x3,160]
	cbz	w4, .L2188
	ldr	w3, [x3,220]
	cmp	w3, 29
	mov	w3, 4
	csel	w1, w1, w3, hi
.L2188:
	adrp	x3, .LANCHOR4
	add	x3, x3, :lo12:.LANCHOR4
	ldrh	w4, [x3,264]
	cmp	w4, 31
	bhi	.L2189
	add	w4, w4, 1
	mov	w1, 1
	strh	w4, [x3,264]
.L2189:
	cbnz	w0, .L2190
	add	x0, x2, :lo12:.LANCHOR2
	ldr	x3, [x0,2152]
	ldr	w0, [x3,20]
	add	w1, w1, w0
	ldr	w0, [x3,16]
	cmp	w0, w1
	bcc	.L2191
.L2190:
	add	x2, x2, :lo12:.LANCHOR2
	ldr	x0, [x2,2152]
	ldr	w1, [x0,16]
	str	w1, [x0,20]
	mov	w1, 17221
	movk	w1, 0x4254, lsl 16
	str	w1, [x0]
	ldr	x0, [x2,2152]
	ldrh	w1, [x2,2136]
	str	wzr, [x0,4]
	lsl	w3, w1, 9
	str	w3, [x0,12]
	ldr	w3, [x0,8]
	ldr	x2, [x2,2152]
	add	w3, w3, 1
	str	w3, [x0,8]
	mov	w0, 64
	bl	FtlVendorPartWrite
	bl	Ftl_save_ext_data
.L2191:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlEctTblFlush, .-FtlEctTblFlush
	.align	2
	.global	StorageSysDataStore
	.type	StorageSysDataStore, %function
StorageSysDataStore:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	str	x0, [x29,16]
	str	x1, [x29,24]
	bl	rknand_device_lock
	ldr	x0, [x29,16]
	mov	w1, 1
	ldr	x2, [x29,24]
	add	w0, w0, 512
	bl	FtlVendorPartWrite
	str	x0, [x29,24]
	bl	rknand_device_unlock
	ldr	x0, [x29,24]
	ldp	x29, x30, [sp], 32
	ret
	.size	StorageSysDataStore, .-StorageSysDataStore
	.align	2
	.global	FtlMapBlkWriteDumpData
	.type	FtlMapBlkWriteDumpData, %function
FtlMapBlkWriteDumpData:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	ldr	w1, [x0,56]
	ldrh	w20, [x0,6]
	ldr	x4, [x0,40]
	cbz	w1, .L2196
	adrp	x3, .LANCHOR2
	adrp	x2, .LANCHOR4
	add	x3, x3, :lo12:.LANCHOR2
	add	x1, x2, :lo12:.LANCHOR4
	sub	w20, w20, #1
	str	wzr, [x0,56]
	mov	x21, x0
	add	x1, x1, 136
	ldr	x0, [x3,2072]
	uxth	w20, w20
	ldr	x3, [x3,2112]
	mov	x19, x2
	str	x3, [x1,16]
	ubfiz	x3, x20, 2, 16
	str	x0, [x1,8]
	ldr	w3, [x4,x3]
	str	w3, [x1,4]
	cbz	w3, .L2198
	mov	x0, x1
	mov	w1, 1
	mov	w2, w1
	bl	FlashReadPages
	b	.L2199
.L2198:
	adrp	x2, .LANCHOR0+2036
	mov	w1, 255
	ldrh	w2, [x2,#:lo12:.LANCHOR0+2036]
	bl	ftl_memset
.L2199:
	add	x2, x19, :lo12:.LANCHOR4
	mov	x0, x21
	mov	w1, w20
	ldr	x2, [x2,144]
	bl	FtlMapWritePage
.L2196:
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlMapBlkWriteDumpData, .-FtlMapBlkWriteDumpData
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
	beq	.L2359
	ldrh	w0, [x19,2]
	adrp	x23, .LANCHOR0
	str	w0, [x29,188]
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w27, [x19,6]
	ldr	w1, [x29,188]
	ldrh	w0, [x0,2026]
	cmp	w0, w1
	bne	.L2206
	strh	wzr, [x19,4]
	b	.L2365
.L2206:
	ldrh	w0, [x19,16]
	mov	w1, 0
.L2207:
	cmp	w0, w2
	bne	.L2368
	add	w1, w1, 1
	uxth	w1, w1
	add	x0, x19, x1, sxtw 1
	ldrh	w0, [x0,16]
	b	.L2207
.L2368:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2209
	bl	FtlGetLastWrittenPage
	cmn	w0, #1
	mov	w21, w0
	beq	.L2210
	add	x0, x23, :lo12:.LANCHOR0
	mov	w24, w21
	ldrb	w1, [x0,120]
	cbnz	w1, .L2282
	add	x0, x0, 196
	ldrh	w24, [x0,w21,sxtw 1]
	b	.L2282
.L2209:
	mov	w1, 0
	bl	FtlGetLastWrittenPage
	cmn	w0, #1
	mov	w21, w0
	beq	.L2210
	mov	w24, w0
.L2282:
	add	x0, x23, :lo12:.LANCHOR0
	mov	x2, x19
	mov	w28, 0
	mov	w7, 65535
	mov	w8, 56
	adrp	x9, .LANCHOR2
	ldrh	w3, [x0,1952]
	mov	w10, 4
	ldrh	w6, [x0,2038]
	add	x3, x19, x3, lsl 1
	b	.L2212
.L2210:
	strh	wzr, [x19,2]
.L2365:
	strb	wzr, [x19,6]
	b	.L2359
.L2214:
	ldrh	w1, [x2,16]
	cmp	w1, w7
	beq	.L2213
	add	x4, x9, :lo12:.LANCHOR2
	orr	w1, w24, w1, lsl 10
	umull	x0, w28, w8
	ldr	x5, [x4,2048]
	add	x5, x5, x0
	str	w1, [x5,4]
	ldr	x1, [x4,2048]
	ldr	x4, [x4,904]
	add	x0, x1, x0
	mul	w1, w28, w6
	add	w28, w28, 1
	str	xzr, [x0,8]
	sdiv	w1, w1, w10
	uxth	w28, w28
	add	x1, x4, x1, sxtw 2
	str	x1, [x0,16]
.L2213:
	add	x2, x2, 2
.L2212:
	cmp	x2, x3
	bne	.L2214
	ldrb	w0, [x19,8]
	str	wzr, [x29,184]
	cmp	w0, 1
	bne	.L2215
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cmp	w0, wzr
	cset	w0, ne
	str	w0, [x29,184]
.L2215:
	adrp	x20, .LANCHOR2
	ldr	w2, [x29,184]
	add	x25, x20, :lo12:.LANCHOR2
	mov	w1, w28
	mov	w26, 0
	ldr	x0, [x25,2048]
	bl	FlashReadPages
	ldr	w22, [x25,200]
	ldr	x3, [x25,2048]
	uxth	w6, w24
	mov	w25, 65535
.L2216:
	cmp	w26, w28
	beq	.L2369
	ldr	w0, [x3]
	cbnz	w0, .L2217
	ldr	x4, [x3,16]
	ldr	w2, [x4,4]
	cmn	w2, #1
	beq	.L2218
	add	x5, x20, :lo12:.LANCHOR2
	mov	w0, w2
	str	x6, [x29,144]
	str	x4, [x29,152]
	ldr	w1, [x5,200]
	str	x3, [x29,160]
	str	x2, [x29,168]
	str	x5, [x29,176]
	bl	ftl_cmp_data_ver
	ldr	x5, [x29,176]
	ldr	x2, [x29,168]
	ldr	x3, [x29,160]
	ldr	x4, [x29,152]
	ldr	x6, [x29,144]
	cbz	w0, .L2218
	add	w2, w2, 1
	str	w2, [x5,200]
.L2218:
	ldr	w0, [x4]
	cmn	w0, #1
	bne	.L2219
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, 56
	uxth	w21, w21
	umull	x26, w26, w1
	ldr	x0, [x0,2048]
	add	x26, x0, x26
	ldr	w0, [x26,4]
	b	.L2361
.L2217:
	mov	w25, w6
.L2219:
	add	w26, w26, 1
	add	x3, x3, 56
	uxth	w26, w26
	b	.L2216
.L2369:
	add	x0, x20, :lo12:.LANCHOR2
	add	w21, w21, 1
	uxth	w21, w21
	ldr	x0, [x0,2048]
	ldr	w0, [x0,4]
.L2361:
	lsr	x0, x0, 10
	bl	P2V_plane
	uxth	w26, w0
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2223
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w2, [x0,120]
	cbnz	w2, .L2223
	add	x0, x0, 196
	ldrh	w21, [x0,w21,sxtw 1]
.L2223:
	add	x0, x23, :lo12:.LANCHOR0
	ldrh	w0, [x0,2026]
	cmp	w0, w21
	bne	.L2224
	strh	w21, [x19,2]
	strb	wzr, [x19,6]
	strh	wzr, [x19,4]
.L2224:
	uxth	w0, w27
	str	w0, [x29,152]
	ldr	w0, [x29,188]
	cmp	w21, w0
	bne	.L2225
	ldr	w0, [x29,152]
	cmp	w26, w0
	beq	.L2366
.L2225:
	mov	w0, 65535
	sub	w22, w22, #1
	cmp	w25, w0
	bne	.L2226
	cbnz	w1, .L2227
.L2226:
	add	x0, x20, :lo12:.LANCHOR2
	uxth	w5, w24
	ldr	w1, [x0,2332]
	cmn	w1, #1
	bne	.L2228
	str	w22, [x0,2332]
.L2228:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w25, [x29,188]
	ldr	w6, [x0,2332]
	ldr	w0, [x29,188]
	add	w0, w0, 7
	cmp	w0, w24, uxth
	bge	.L2229
	sub	w25, w5, #7
	uxth	w25, w25
.L2229:
	mov	w3, -1
	add	x28, x20, :lo12:.LANCHOR2
	mov	w27, w3
	mov	w4, 65535
	mov	w7, 56
	add	x8, x28, 996
.L2230:
	cmp	w25, w5
	bhi	.L2243
	add	x0, x23, :lo12:.LANCHOR0
	mov	w24, 0
	ldrh	w2, [x0,1952]
	mov	x0, x19
	add	x2, x19, x2, lsl 1
.L2244:
	cmp	x0, x2
	beq	.L2370
	ldrh	w1, [x0,16]
	cmp	w1, w4
	beq	.L2231
	umull	x9, w24, w7
	ldr	x10, [x28,2048]
	add	w24, w24, 1
	orr	w1, w25, w1, lsl 10
	add	x9, x10, x9
	uxth	w24, w24
	str	w1, [x9,4]
.L2231:
	add	x0, x0, 2
	b	.L2244
.L2370:
	ldr	x0, [x28,2048]
	mov	w1, w24
	ldr	w2, [x29,184]
	str	x8, [x29,128]
	str	x7, [x29,136]
	str	x4, [x29,144]
	str	x6, [x29,160]
	str	x5, [x29,168]
	str	x3, [x29,176]
	bl	FlashReadPages
	add	x0, x23, :lo12:.LANCHOR0
	ldr	x3, [x29,176]
	ldr	x5, [x29,168]
	mov	w1, 0
	ldr	x6, [x29,160]
	sxtw	x10, w25
	ldrb	w9, [x0,120]
	ldr	x0, [x28,2048]
	ldr	x4, [x29,144]
	ldr	x7, [x29,136]
	add	x0, x0, 16
	ldr	x8, [x29,128]
.L2233:
	cmp	w1, w24
	beq	.L2371
	ldr	w2, [x0,-16]
	cbnz	w2, .L2234
	ldr	x2, [x0]
	ldrh	w11, [x2]
	cmp	w11, w4
	beq	.L2235
	ldr	w2, [x2,4]
	cmn	w2, #1
	beq	.L2235
	ldr	w27, [x28,2332]
	cmn	w3, #1
	str	w2, [x28,2332]
	bne	.L2235
	ldrh	w2, [x8,x10,lsl 1]
	cmp	w2, w4
	bne	.L2236
	cbz	w9, .L2235
.L2236:
	cmp	w27, w22
	csinv	w3, w27, wzr, ne
	b	.L2235
.L2234:
	ldrb	w0, [x19,8]
	cbnz	w0, .L2227
	add	x0, x20, :lo12:.LANCHOR2
	add	x1, x0, 996
	ldrh	w2, [x1,w25,sxtw 1]
	mov	w1, 65535
	cmp	w2, w1
	bne	.L2238
	cmn	w3, #1
	beq	.L2239
	str	w3, [x0,2332]
	b	.L2227
.L2239:
	cmp	w6, w22
	beq	.L2240
	str	w6, [x0,2332]
	b	.L2227
.L2240:
	ldr	w1, [x0,2332]
	b	.L2367
.L2238:
	cmp	w27, w22
	beq	.L2241
	cmn	w27, #1
	beq	.L2227
	str	w27, [x0,2332]
	b	.L2227
.L2241:
	ldr	w1, [x0,2332]
	cmp	w1, w22
	beq	.L2227
.L2367:
	sub	w1, w1, #1
	b	.L2362
.L2235:
	add	w1, w1, 1
	add	x0, x0, 56
	uxth	w1, w1
	b	.L2233
.L2371:
	add	w25, w25, 1
	uxth	w25, w25
	b	.L2230
.L2243:
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, -1
.L2362:
	str	w1, [x0,2332]
.L2227:
	adrp	x25, .LANCHOR4
	mov	w1, 1
	add	x0, x25, :lo12:.LANCHOR4
	strh	w1, [x0,266]
	add	x0, x20, :lo12:.LANCHOR2
	add	x0, x0, 2240
	bl	FtlMapBlkWriteDumpData
	ldr	w24, [x29,188]
	adrp	x0, .LC121
	str	x25, [x29,136]
	add	x0, x0, :lo12:.LC121
	str	x0, [x29,128]
.L2245:
	add	x0, x23, :lo12:.LANCHOR0
	mov	x1, x19
	str	wzr, [x29,176]
	mov	w7, 65535
	mov	w8, 56
	ldrh	w3, [x0,1952]
	ldrb	w6, [x0,120]
	add	x3, x19, x3, lsl 1
.L2246:
	cmp	x1, x3
	beq	.L2372
	ldrh	w2, [x1,16]
	cmp	w2, w7
	beq	.L2247
	ldr	w0, [x29,176]
	add	x5, x20, :lo12:.LANCHOR2
	orr	w2, w24, w2, lsl 10
	ldr	x4, [x5,2048]
	umull	x0, w0, w8
	add	x4, x4, x0
	str	w2, [x4,4]
	ldrb	w2, [x19,8]
	cmp	w2, 1
	bne	.L2248
	cbz	w6, .L2248
	ldr	x2, [x5,2048]
	add	x0, x2, x0
	ldr	w2, [x0,4]
	orr	w2, w2, -2147483648
	str	w2, [x0,4]
.L2248:
	ldr	w0, [x29,176]
	add	w0, w0, 1
	uxth	w0, w0
	str	w0, [x29,176]
.L2247:
	add	x1, x1, 2
	b	.L2246
.L2372:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w1, [x29,176]
	ldr	w2, [x29,184]
	ldr	x0, [x0,2048]
	bl	FlashReadPages
	str	xzr, [x29,160]
.L2250:
	ldrh	w0, [x29,160]
	ldr	w1, [x29,176]
	cmp	w1, w0, uxth
	bls	.L2373
	ldr	x0, [x29,160]
	mov	x1, 56
	mul	x25, x0, x1
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x1, [x0,2048]
	str	x1, [x29,168]
	add	x28, x1, x25
	ldr	w27, [x28,4]
	str	w27, [x29,204]
	lsr	x0, x27, 10
	bl	P2V_plane
	uxth	w0, w0
	ldr	w1, [x29,188]
	cmp	w24, w1
	ldr	x1, [x29,168]
	bcc	.L2251
	ldr	w2, [x29,152]
	cmp	w0, w2
	bcs	.L2286
	ldr	w2, [x29,188]
	cmp	w24, w2
	beq	.L2251
.L2286:
	cmp	w0, w26
	bne	.L2287
	cmp	w24, w21
	beq	.L2253
.L2287:
	ldr	w0, [x1,x25]
	cmn	w0, #1
	beq	.L2255
	ldr	x28, [x28,16]
	mov	w0, 61589
	ldrh	w1, [x28]
	cmp	w1, w0
	bne	.L2262
	ldr	w22, [x28,4]
	cmn	w22, #1
	beq	.L2257
	add	x27, x20, :lo12:.LANCHOR2
	mov	w0, w22
	ldr	w1, [x27,200]
	bl	ftl_cmp_data_ver
	cbz	w0, .L2257
	add	w0, w22, 1
	str	w0, [x27,200]
.L2257:
	ldr	w27, [x28,8]
	add	x1, x29, 200
	ldr	w0, [x28,12]
	mov	w2, 0
	str	w0, [x29,196]
	mov	w0, w27
	bl	log2phys
	add	x3, x20, :lo12:.LANCHOR2
	str	x3, [x29,168]
	ldr	w1, [x3,2332]
	cmn	w1, #1
	beq	.L2258
	mov	w0, w22
	bl	ftl_cmp_data_ver
	ldr	x3, [x29,168]
	cbz	w0, .L2258
	ldr	w1, [x29,196]
	cmn	w1, #1
	beq	.L2259
	ldr	x0, [x3,2048]
	str	x3, [x29,120]
	add	x0, x0, x25
	str	w1, [x0,4]
	mov	w1, 1
	ldr	x2, [x0,16]
	ldr	x0, [x3,2048]
	str	x2, [x29,168]
	mov	w2, 0
	add	x0, x0, x25
	bl	FlashReadPages
	ldr	x3, [x29,120]
	ldr	x0, [x29,168]
	ldr	x2, [x3,2048]
	ldr	w0, [x0,4]
	str	w0, [x29,144]
	add	x4, x2, x25
	ldr	w0, [x2,x25]
	cmn	w0, #1
	bne	.L2260
	b	.L2261
.L2259:
	ldr	w0, [x29,204]
	ldr	w1, [x29,200]
	cmp	w1, w0
	bne	.L2262
	mov	w0, w27
	add	x1, x29, 196
	mov	w2, 1
	bl	log2phys
.L2262:
	ldrh	w0, [x19]
	b	.L2364
.L2260:
	ldr	x0, [x29,168]
	ldr	w28, [x0,8]
	cmp	w28, w27
	bne	.L2261
	ldr	w0, [x3,2332]
	ldr	w1, [x29,144]
	str	x2, [x29,104]
	str	x4, [x29,112]
	str	x3, [x29,120]
	bl	ftl_cmp_data_ver
	ldr	x3, [x29,120]
	ldr	x4, [x29,112]
	ldr	x2, [x29,104]
	cbz	w0, .L2261
	ldr	w0, [x29,200]
	ldr	w1, [x29,204]
	cmp	w0, w1
	beq	.L2267
	ldr	w1, [x29,196]
	cmp	w0, w1
	beq	.L2261
	cmn	w0, #1
	beq	.L2265
	str	w0, [x4,4]
	mov	w2, 0
	ldr	x1, [x4,16]
	ldr	x0, [x3,2048]
	str	x1, [x29,168]
	mov	w1, 1
	add	x0, x0, x25
	bl	FlashReadPages
	b	.L2266
.L2265:
	str	w0, [x2,x25]
.L2266:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x1, [x0,2048]
	ldr	w1, [x1,x25]
	cmn	w1, #1
	beq	.L2267
	ldr	x1, [x29,168]
	ldr	w0, [x0,2332]
	ldr	w25, [x1,4]
	mov	w1, w25
	bl	ftl_cmp_data_ver
	cbz	w0, .L2267
	ldr	w0, [x29,144]
	mov	w1, w25
	bl	ftl_cmp_data_ver
	cbz	w0, .L2261
.L2267:
	ldr	w1, [x29,196]
	mov	w0, w28
	bl	FtlReUsePrevPpa
.L2261:
	mov	w0, -1
	str	w0, [x29,196]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L2269
.L2258:
	ldr	w0, [x29,204]
	ldr	w1, [x29,200]
	cmp	w1, w0
	beq	.L2269
	mov	w0, w27
	add	x1, x29, 204
	mov	w2, 1
	bl	log2phys
	ldr	w25, [x29,200]
	cmn	w25, #1
	beq	.L2269
	ldr	w0, [x29,196]
	cmp	w25, w0
	beq	.L2269
	lsr	x0, x25, 10
	bl	P2V_block_in_plane
	uxth	w0, w0
	ldrh	w2, [x20,#:lo12:.LANCHOR2]
	add	x1, x20, :lo12:.LANCHOR2
	cmp	w2, w0
	beq	.L2272
	ldrh	w2, [x1,48]
	cmp	w2, w0
	beq	.L2272
	ldrh	w1, [x1,96]
	cmp	w1, w0
	bne	.L2269
.L2272:
	add	x28, x20, :lo12:.LANCHOR2
	mov	w1, 1
	mov	w2, 0
	ldr	x0, [x28,2048]
	str	w25, [x0,4]
	ldr	x3, [x0,16]
	ldr	x0, [x28,2048]
	str	x3, [x29,168]
	bl	FlashReadPages
	ldr	x0, [x28,2048]
	ldr	x3, [x29,168]
	ldr	w0, [x0]
	ldr	w1, [x3,4]
	cmn	w0, #1
	beq	.L2269
	mov	w0, w22
	bl	ftl_cmp_data_ver
	cbnz	w0, .L2269
	mov	w0, w27
	add	x1, x29, 200
	mov	w2, 1
	bl	log2phys
.L2269:
	ldr	w0, [x29,196]
	cmn	w0, #1
	beq	.L2251
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	add	x2, x20, :lo12:.LANCHOR2
	ubfiz	x3, x1, 1, 16
	ldr	x2, [x2,-40]
	ldrh	w2, [x2,x3]
	cbz	w2, .L2273
.L2364:
	bl	decrement_vpc_count
	b	.L2251
.L2273:
	ldr	x0, [x29,128]
	bl	printk
	b	.L2251
.L2255:
	ldr	x0, [x29,136]
	add	x1, x0, :lo12:.LANCHOR4
	ldr	w0, [x1,268]
	cmp	w0, 31
	bhi	.L2274
	add	x2, x1, 272
	str	w27, [x2,w0,uxtw 2]
	add	w0, w0, 1
	str	w0, [x1,268]
.L2274:
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w1, [x0,2332]
	cmn	w1, #1
	beq	.L2363
	cmp	w1, w22
	bls	.L2251
.L2363:
	str	w22, [x0,2332]
.L2251:
	ldr	x0, [x29,160]
	add	x0, x0, 1
	str	x0, [x29,160]
	b	.L2250
.L2373:
	ldrb	w0, [x19,8]
	add	w24, w24, 1
	cmp	w0, 1
	uxth	w24, w24
	bne	.L2277
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w1, [x0,120]
	cbz	w1, .L2277
	cmp	w21, w24
	bne	.L2277
	ldrh	w0, [x0,2028]
	cmp	w0, w24
	beq	.L2253
.L2277:
	add	x0, x23, :lo12:.LANCHOR0
	ldrh	w1, [x0,2026]
	cmp	w24, w1
	bne	.L2245
	ldrh	w2, [x0,1952]
	mov	w3, 65535
	strh	w24, [x19,2]
	mov	x0, 0
	strh	wzr, [x19,4]
.L2279:
	uxth	w1, w0
	cmp	w1, w2
	bcs	.L2359
	add	x0, x0, 1
	add	x4, x19, x0, lsl 1
	ldrh	w4, [x4,14]
	cmp	w4, w3
	beq	.L2279
	strb	w1, [x19,6]
	b	.L2359
.L2253:
	strb	w26, [x19,6]
	strh	w21, [x19,2]
.L2366:
	mov	x0, x19
	mov	w1, w21
	mov	w2, w26
	bl	ftl_sb_update_avl_pages
.L2359:
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
	.global	FtlWriteDumpData
	.type	FtlWriteDumpData, %function
FtlWriteDumpData:
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	str	x25, [sp,64]
	stp	x21, x22, [sp,32]
	add	x21, x19, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	ldrh	w1, [x21,4]
	cbz	w1, .L2375
	ldrb	w0, [x21,8]
	cbnz	w0, .L2375
	adrp	x22, .LANCHOR0
	ldrb	w0, [x21,7]
	add	x22, x22, :lo12:.LANCHOR0
	ldrh	w2, [x22,2026]
	mul	w0, w0, w2
	cmp	w1, w0
	beq	.L2375
	ldrb	w24, [x21,10]
	ldr	w23, [x22,2104]
	ldrh	w25, [x22,1952]
	cbnz	w24, .L2374
	sub	w23, w23, #1
	add	x1, x29, 84
	mov	w0, w23
	mov	w2, w24
	bl	log2phys
	ldr	x20, [x21,2112]
	ldr	w0, [x29,84]
	ldr	x1, [x21,-64]
	str	w0, [x29,92]
	cmn	w0, #1
	str	w23, [x29,112]
	str	x1, [x29,96]
	str	x20, [x29,104]
	str	w24, [x20,4]
	beq	.L2377
	add	x0, x29, 88
	mov	w1, 1
	mov	w2, w24
	bl	FlashReadPages
	b	.L2378
.L2377:
	ldr	x0, [x21,-64]
	mov	w1, 255
	ldrh	w2, [x22,2036]
	bl	ftl_memset
.L2378:
	mov	w0, -3947
	lsl	w25, w25, 2
	strh	w0, [x20]
	mov	w22, 0
	add	x21, x19, :lo12:.LANCHOR2
.L2379:
	cmp	w22, w25
	beq	.L2380
	ldrh	w0, [x21,4]
	cbz	w0, .L2380
	ldr	w0, [x29,92]
	add	w22, w22, 1
	str	w0, [x20,12]
	ldrh	w0, [x21]
	strh	w0, [x20,2]
	mov	x0, x21
	str	w23, [x20,8]
	bl	get_new_active_ppa
	str	w0, [x29,92]
	ldr	w1, [x21,200]
	mov	w2, 0
	str	w1, [x20,4]
	mov	w3, w2
	add	w1, w1, 1
	add	x0, x29, 88
	cmn	w1, #1
	csel	w1, w1, wzr, ne
	str	w1, [x21,200]
	mov	w1, 1
	bl	FlashProgPages
	ldrh	w0, [x21]
	bl	decrement_vpc_count
	b	.L2379
.L2380:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, 1
	strb	w0, [x19,10]
	b	.L2374
.L2375:
	add	x19, x19, :lo12:.LANCHOR2
	strb	wzr, [x19,10]
.L2374:
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
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	mov	w19, 0
	adrp	x20, .LANCHOR0
	adrp	x21, .LANCHOR2
	bl	FtlWriteDumpData
.L2392:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w0, [x0,2066]
	cmp	w0, w19
	bls	.L2395
	add	x1, x21, :lo12:.LANCHOR2
	ubfiz	x0, x19, 4, 16
	ldr	x1, [x1,144]
	add	x0, x1, x0
	ldr	w0, [x0,4]
	tbz	w0, #31, .L2393
	mov	w0, w19
	bl	flush_l2p_region
.L2393:
	add	w19, w19, 1
	uxth	w19, w19
	b	.L2392
.L2395:
	mov	w0, 0
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	stp	x29, x30, [sp, -80]!
	adrp	x0, .LANCHOR2
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	add	x19, x0, :lo12:.LANCHOR2
	stp	x23, x24, [sp,48]
	adrp	x23, .LANCHOR4
	adrp	x24, .LANCHOR0
	add	x20, x23, :lo12:.LANCHOR4
	ldr	x1, [x19,-64]
	stp	x21, x22, [sp,32]
	ldr	x22, [x19,2112]
	add	x21, x20, 136
	stp	x25, x26, [sp,64]
	str	x1, [x20,144]
	add	x25, x24, :lo12:.LANCHOR0
	str	x22, [x20,152]
	add	x20, x19, 224
	ldrh	w2, [x19,280]
	mov	w26, 65535
	strh	w2, [x22,2]
	mov	w2, -3932
	strh	w2, [x22]
	str	wzr, [x22,12]
	ldr	w2, [x19,288]
	str	w2, [x22,4]
	mov	w2, 19539
	str	wzr, [x22,8]
	movk	w2, 0x4654, lsl 16
	ldrh	w1, [x19,286]
	ldrh	w0, [x0,#:lo12:.LANCHOR2]
	strh	w1, [x19,232]
	strh	w0, [x19,238]
	ldrh	w1, [x25,1974]
	ldrh	w0, [x19,2]
	strb	w1, [x19,234]
	ldrb	w1, [x19,6]
	str	w2, [x19,224]
	mov	w2, 64
	orr	w0, w1, w0, lsl 6
	movk	w2, 0x5000, lsl 16
	strh	w0, [x19,240]
	str	w2, [x19,228]
	ldrb	w0, [x19,8]
	ldrh	w1, [x19,48]
	strh	w1, [x19,242]
	ldrh	w1, [x19,50]
	ldrb	w2, [x19,54]
	strb	w0, [x19,235]
	orr	w1, w2, w1, lsl 6
	ldrb	w0, [x19,56]
	strh	w1, [x19,244]
	strb	w0, [x19,236]
	ldrh	w1, [x19,96]
	ldrb	w0, [x19,104]
	strh	w1, [x19,246]
	strb	w0, [x19,237]
	ldrh	w1, [x19,98]
	ldr	w0, [x19,204]
	ldrb	w2, [x19,102]
	str	w0, [x19,256]
	ldr	w0, [x19,196]
	orr	w1, w2, w1, lsl 6
	str	w0, [x19,264]
	ldr	w0, [x19,200]
	ldrh	w2, [x25,2036]
	str	w0, [x19,260]
	ldr	x0, [x21,8]
	strh	w1, [x19,248]
	mov	w1, 255
	bl	ftl_memset
	ldr	x0, [x21,8]
	mov	x1, x20
	mov	w2, 48
	mov	x20, x21
	bl	ftl_memcpy
	ldrh	w2, [x25,1960]
	ldr	x0, [x21,8]
	ldr	x1, [x19,-40]
	lsl	w2, w2, 1
	add	x0, x0, 48
	bl	ftl_memcpy
	ldrh	w2, [x25,1960]
	mov	w25, 0
	ldr	x1, [x21,8]
	uxth	x0, w2
	lsr	w2, w2, 3
	add	x0, x0, 24
	add	w2, w2, 4
	lsr	x0, x0, 1
	add	x0, x1, x0, lsl 2
	ldr	x1, [x24,#:lo12:.LANCHOR0]
	bl	ftl_memcpy
	mov	w0, 0
	bl	FtlUpdateVaildLpn
.L2397:
	ldr	x0, [x19,-64]
	str	x0, [x20,8]
	ldr	x0, [x19,2112]
	str	x0, [x20,16]
	ldrh	w0, [x19,280]
	ldrh	w1, [x19,282]
	orr	w0, w1, w0, lsl 10
	mov	w1, 1
	str	w0, [x20,4]
	mov	w2, w1
	mov	w3, w1
	mov	x0, x20
	bl	FlashProgPages
	add	x0, x24, :lo12:.LANCHOR0
	ldrh	w1, [x19,282]
	ldrh	w0, [x0,2028]
	sub	w0, w0, #1
	cmp	w1, w0
	blt	.L2398
	ldrh	w0, [x19,280]
	ldrh	w26, [x19,284]
	strh	wzr, [x19,282]
	strh	w0, [x19,284]
	bl	FtlFreeSysBlkQueueOut
	ldr	w1, [x19,196]
	str	w1, [x19,288]
	add	w2, w1, 1
	str	w2, [x19,196]
	ubfiz	w2, w0, 10, 16
	str	w2, [x20,4]
	str	w1, [x22,4]
	mov	w1, 1
	strh	w0, [x19,280]
	mov	w2, w1
	strh	w0, [x22,2]
	mov	w3, w1
	mov	x0, x20
	bl	FlashProgPages
.L2398:
	ldrh	w1, [x19,282]
	ldr	w2, [x20]
	add	w1, w1, 1
	cmn	w2, #1
	uxth	w1, w1
	strh	w1, [x19,282]
	bne	.L2399
	cmp	w1, 1
	bne	.L2400
	add	x1, x24, :lo12:.LANCHOR0
	ldrh	w1, [x1,2028]
	sub	w1, w1, #1
	strh	w1, [x19,282]
.L2400:
	add	w25, w25, 1
	uxth	w25, w25
	cmp	w25, 3
	bls	.L2397
	add	x23, x23, :lo12:.LANCHOR4
	adrp	x0, .LC122
	add	x0, x0, :lo12:.LC122
	mov	w2, w25
	ldr	w1, [x23,140]
	bl	printk
.L2402:
	b	.L2402
.L2399:
	cmp	w1, 1
	beq	.L2397
	mov	w0, 65535
	cmp	w26, w0
	beq	.L2403
	mov	w0, w26
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
.L2403:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	stp	x29, x30, [sp, -80]!
	adrp	x1, .LANCHOR3
	adrp	x0, .LC123
	add	x1, x1, :lo12:.LANCHOR3
	add	x29, sp, 0
	add	x1, x1, 16
	add	x0, x0, :lo12:.LC123
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	adrp	x21, .LANCHOR0
	bl	printk
	add	x1, x21, :lo12:.LANCHOR0
	adrp	x20, .LANCHOR2
	add	x0, x20, :lo12:.LANCHOR2
	mov	w19, 0
	ldrh	w2, [x1,1962]
	mov	w1, 0
	ldr	x0, [x0,2160]
	lsl	w2, w2, 1
	bl	ftl_memset
.L2408:
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w0, [x0,2104]
	cmp	w19, w0
	bcs	.L2423
	mov	w0, w19
	add	x1, x29, 76
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x29,76]
	cmn	w0, #1
	beq	.L2409
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	ubfiz	x0, x0, 1, 16
	add	x1, x20, :lo12:.LANCHOR2
	ldr	x2, [x1,2160]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
.L2409:
	add	w19, w19, 1
	b	.L2408
.L2423:
	adrp	x23, .LC124
	mov	w19, 0
	add	x20, x20, :lo12:.LANCHOR2
	mov	w24, 65535
	add	x23, x23, :lo12:.LC124
.L2411:
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,1960]
	cmp	w0, w19
	bls	.L2424
	ubfiz	x22, x19, 1, 16
	ldr	x0, [x20,-40]
	ldrh	w2, [x0,x22]
	ldr	x0, [x20,2160]
	ldrh	w3, [x0,x22]
	cmp	w2, w3
	beq	.L2414
	cmp	w2, w24
	beq	.L2414
	mov	x0, x23
	mov	w1, w19
	bl	printk
	ldrh	w0, [x20]
	cmp	w0, w19
	beq	.L2414
	ldrh	w0, [x20,96]
	cmp	w0, w19
	beq	.L2414
	ldrh	w0, [x20,48]
	cmp	w0, w19
	beq	.L2414
	ldr	x0, [x20,2160]
	ldrh	w1, [x0,x22]
	ldr	x0, [x20,-40]
	strh	w1, [x0,x22]
	mov	w0, w19
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2414:
	add	w19, w19, 1
	uxth	w19, w19
	b	.L2411
.L2424:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
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
	str	x21, [sp,32]
	mov	x20, x0
	adrp	x19, .LANCHOR2
	ldrh	w21, [x0]
	mov	w0, 65535
	cmp	w21, w0
	beq	.L2426
	add	x1, x19, :lo12:.LANCHOR2
	ubfiz	x0, x21, 1, 16
	ldr	x1, [x1,-40]
	ldrh	w0, [x1,x0]
	cbz	w0, .L2427
	mov	w0, w21
	bl	INSERT_DATA_LIST
	b	.L2426
.L2427:
	mov	w0, w21
	bl	INSERT_FREE_LIST
.L2426:
	add	x2, x19, :lo12:.LANCHOR2
	strb	wzr, [x20,8]
	add	x0, x2, 48
	cmp	x20, x0
	beq	.L2428
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w1, [x0,1972]
	cmp	w1, 1
	beq	.L2428
	ldrb	w0, [x0,120]
	cbz	w0, .L2429
.L2428:
	mov	w0, 1
	strb	w0, [x20,8]
	b	.L2430
.L2429:
	cmp	x20, x2
	bne	.L2430
	cmp	w1, 3
	beq	.L2432
	ldr	w0, [x20,344]
	cmp	w0, 1
	bne	.L2433
.L2432:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 1
	strb	w1, [x0,8]
.L2433:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w1, [x0,160]
	cbz	w1, .L2430
	ldr	w1, [x0,220]
	cmp	w1, 29
	bhi	.L2430
	mov	w1, 1
	strb	w1, [x0,8]
.L2430:
	add	x2, x19, :lo12:.LANCHOR2
	mov	w1, 65535
	ldrh	w0, [x2,2320]
	cmp	w0, w1
	beq	.L2435
	cmp	w21, w0
	bne	.L2436
	ubfiz	x1, x0, 1, 16
	ldr	x2, [x2,-40]
	ldrh	w1, [x2,x1]
	cbz	w1, .L2437
.L2436:
	bl	update_vpc_list
.L2437:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	w0, [x19,2320]
.L2435:
	mov	x0, x20
	bl	allocate_data_superblock
	bl	l2p_flush
	mov	w0, 0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	ldr	x21, [sp,32]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	mov	x19, x0
	add	x0, x20, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	w22, 0
	ldrb	w0, [x0,120]
	cbz	w0, .L2451
	ldrb	w0, [x19,8]
	cmp	w0, 1
	cset	w22, eq
.L2451:
	mov	w21, 7
	adrp	x23, .LANCHOR2
	mov	w24, -1
.L2452:
	subs	w21, w21, #1
	beq	.L2454
	ldrh	w0, [x19,4]
	cbnz	w0, .L2453
.L2454:
	adrp	x1, .LANCHOR2
	ldrh	w0, [x19]
	add	x1, x1, :lo12:.LANCHOR2
	ldrh	w3, [x19,4]
	lsl	x0, x0, 1
	add	x20, x20, :lo12:.LANCHOR0
	ldr	x2, [x1,-40]
	ldrh	w1, [x2,x0]
	sub	w1, w1, w3
	strh	w1, [x2,x0]
	strb	wzr, [x19,6]
	ldrh	w0, [x20,2026]
	strh	w0, [x19,2]
	strh	wzr, [x19,4]
	ldp	x21, x22, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 128
	ret
.L2453:
	mov	x0, x19
	bl	get_new_active_ppa
	str	w0, [x29,76]
	cmn	w0, #1
	beq	.L2454
	add	x3, x23, :lo12:.LANCHOR2
	str	w24, [x29,96]
	mov	w2, w22
	ldr	x1, [x3,2112]
	ldr	x0, [x3,-64]
	str	x0, [x29,80]
	str	x1, [x29,88]
	str	w24, [x1,8]
	str	w24, [x1,12]
	ldrh	w0, [x19]
	strh	w0, [x1,2]
	strh	wzr, [x1]
	ldr	w0, [x3,200]
	str	w0, [x1,4]
	mov	w1, 1
	add	w0, w0, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x3,200]
	add	x0, x29, 72
	mov	w3, 0
	bl	FlashProgPages
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L2452
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
	.align	2
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	stp	x29, x30, [sp, -112]!
	adrp	x0, .LANCHOR4+400
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	ldr	x20, [x0,#:lo12:.LANCHOR4+400]
	add	x0, x19, :lo12:.LANCHOR0
	str	x27, [sp,80]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	ldr	w1, [x0,2088]
	cbz	w1, .L2465
	ldrb	w0, [x0,120]
	mov	w22, 0
	cbz	w0, .L2466
	ldrb	w0, [x20,8]
	cmp	w0, 1
	cset	w22, eq
.L2466:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w3, [x20,9]
	adrp	x24, .LC125
	mov	w2, w22
	mov	w23, 0
	mov	w25, 56
	ldr	x0, [x0,2096]
	adrp	x26, .LANCHOR2
	add	x24, x24, :lo12:.LC125
	bl	FlashProgPages
.L2467:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w1, [x0,2088]
	cmp	w23, w1
	bcs	.L2484
	umull	x21, w23, w25
	ldr	x1, [x0,2096]
	add	x0, x1, x21
	ldr	w1, [x1,x21]
	cmn	w1, #1
	beq	.L2487
	ldr	w1, [x0,4]
	cbz	w22, .L2501
	orr	w1, w1, -2147483648
.L2501:
	ldr	w0, [x0,24]
	mov	w2, 1
	str	w1, [x29,108]
	add	x1, x29, 108
	bl	log2phys
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x0, [x0,2096]
	add	x21, x0, x21
	ldr	x0, [x21,16]
	ldr	w0, [x0,12]
	cmn	w0, #1
	beq	.L2471
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	add	x2, x26, :lo12:.LANCHOR2
	ubfiz	x0, x1, 1, 16
	mov	w21, w1
	ldr	x2, [x2,-40]
	ldrh	w2, [x2,x0]
	cbnz	w2, .L2472
	mov	x0, x24
	bl	printk
.L2472:
	mov	w0, w21
	bl	decrement_vpc_count
.L2471:
	add	w23, w23, 1
	b	.L2467
.L2503:
	mov	w20, 16386
	adrp	x21, .LANCHOR2
.L2483:
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w0, [x0,956]
	cbz	w0, .L2484
	mov	w0, 1
	mov	w1, w0
	bl	rk_ftl_garbage_collect
	subs	w20, w20, #1
	bne	.L2483
.L2484:
	add	x19, x19, :lo12:.LANCHOR0
	str	wzr, [x19,2088]
	b	.L2465
.L2487:
	adrp	x23, .LANCHOR2
	adrp	x25, .LC125
	mov	w24, 0
	mov	w26, 56
	add	x23, x23, :lo12:.LANCHOR2
	add	x25, x25, :lo12:.LC125
.L2468:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w1, [x0,2088]
	cmp	w24, w1
	bcs	.L2503
	umull	x21, w24, w26
	ldr	x0, [x0,2096]
	mov	w1, -1
	str	w1, [x0,x21]
.L2474:
	add	x27, x19, :lo12:.LANCHOR0
	ldr	x0, [x27,2096]
	add	x1, x0, x21
	ldr	w0, [x0,x21]
	cmn	w0, #1
	ldr	w0, [x1,4]
	bne	.L2504
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	ldrh	w1, [x20]
	cmp	w1, w0, uxth
	bne	.L2475
	ldr	x2, [x23,-40]
	ubfiz	x1, x1, 1, 16
	ldrh	w3, [x20,4]
	ldrh	w0, [x2,x1]
	sub	w0, w0, w3
	strh	w0, [x2,x1]
	strb	wzr, [x20,6]
	ldrh	w0, [x27,2026]
	strh	w0, [x20,2]
	strh	wzr, [x20,4]
.L2475:
	ldrh	w0, [x20,4]
	cbnz	w0, .L2476
	mov	x0, x20
	bl	allocate_new_data_superblock
.L2476:
	ldr	w0, [x23,448]
	add	x27, x19, :lo12:.LANCHOR0
	add	w0, w0, 1
	str	w0, [x23,448]
	ldr	x0, [x27,2096]
	add	x0, x0, x21
	ldr	w0, [x0,4]
	lsr	x0, x0, 10
	bl	FtlGcMarkBadPhyBlk
	mov	x0, x20
	bl	get_new_active_ppa
	str	w0, [x29,108]
	ldr	x1, [x27,2096]
	mov	w2, w22
	add	x1, x1, x21
	str	w0, [x1,4]
	mov	w1, 1
	ldr	x0, [x27,2096]
	ldrb	w3, [x20,9]
	add	x0, x0, x21
	bl	FlashProgPages
	b	.L2474
.L2504:
	cbz	w22, .L2502
	orr	w0, w0, -2147483648
.L2502:
	str	w0, [x29,108]
	mov	w2, 1
	ldr	w0, [x1,24]
	add	x1, x29, 108
	bl	log2phys
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x0, [x0,2096]
	add	x21, x0, x21
	ldr	x0, [x21,16]
	ldr	w0, [x0,12]
	cmn	w0, #1
	beq	.L2480
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	ubfiz	x0, x1, 1, 16
	ldr	x2, [x23,-40]
	mov	w21, w1
	ldrh	w2, [x2,x0]
	cbnz	w2, .L2481
	mov	x0, x25
	bl	printk
.L2481:
	mov	w0, w21
	bl	decrement_vpc_count
.L2480:
	add	w24, w24, 1
	b	.L2468
.L2465:
	mov	w0, 0
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 112
	ret
	.size	FtlCacheWriteBack, .-FtlCacheWriteBack
	.align	2
	.global	FtlSysFlush
	.type	FtlSysFlush, %function
FtlSysFlush:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FtlCacheWriteBack
	bl	l2p_flush
	mov	w0, 1
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlSysFlush, .-FtlSysFlush
	.align	2
	.global	FtlDeInit
	.type	FtlDeInit, %function
FtlDeInit:
	adrp	x0, .LANCHOR1+532
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w0, [x0,#:lo12:.LANCHOR1+532]
	cmp	w0, 1
	bne	.L2507
	bl	FtlSysFlush
.L2507:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	FtlDeInit, .-FtlDeInit
	.align	2
	.global	rk_ftl_de_init
	.type	rk_ftl_de_init, %function
rk_ftl_de_init:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FlashDeInit
	bl	FtlDeInit
	bl	FlashDeInit
	ldp	x29, x30, [sp], 16
	ret
	.size	rk_ftl_de_init, .-rk_ftl_de_init
	.align	2
	.global	FtlDiscard
	.type	FtlDiscard, %function
FtlDiscard:
	stp	x29, x30, [sp, -80]!
	add	w2, w0, w1
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x22, .LANCHOR0
	str	x23, [sp,48]
	add	x21, x22, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	mov	w19, w1
	mov	w20, w0
	ldr	w1, [x21,2068]
	mov	w0, -1
	cmp	w2, w1
	bhi	.L2510
	cmp	w19, 31
	bls	.L2524
	bl	FtlCacheWriteBack
	ldrh	w1, [x21,2032]
	udiv	w21, w20, w1
	msub	w20, w1, w21, w20
	uxth	w20, w20
	cbz	w20, .L2512
	sub	w1, w1, w20
	add	w21, w21, 1
	cmp	w1, w19
	csel	w1, w1, w19, ls
	sub	w19, w19, w1, uxth
.L2512:
	mov	w0, -1
	adrp	x20, .LANCHOR4
	str	w0, [x29,76]
	adrp	x23, .LANCHOR2
.L2513:
	add	x0, x22, :lo12:.LANCHOR0
	ldrh	w0, [x0,2032]
	cmp	w19, w0
	bcc	.L2525
	mov	w0, w21
	add	x1, x29, 72
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x29,72]
	cmn	w0, #1
	beq	.L2514
	add	x1, x20, :lo12:.LANCHOR4
	mov	w2, 1
	ldr	w0, [x1,408]
	add	w0, w0, 1
	str	w0, [x1,408]
	add	x1, x23, :lo12:.LANCHOR2
	ldr	w0, [x1,168]
	add	w0, w0, 1
	str	w0, [x1,168]
	add	x1, x29, 76
	mov	w0, w21
	bl	log2phys
	ldr	w0, [x29,72]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	decrement_vpc_count
.L2514:
	add	x0, x22, :lo12:.LANCHOR0
	add	w21, w21, 1
	ldrh	w0, [x0,2032]
	sub	w19, w19, w0
	b	.L2513
.L2525:
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	ldr	w1, [x0,408]
	cmp	w1, 32
	bls	.L2524
	str	wzr, [x0,408]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2524:
	mov	w0, 0
.L2510:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldr	x23, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlDiscard, .-FtlDiscard
	.align	2
	.global	FtlGcFreeTempBlock
	.type	FtlGcFreeTempBlock, %function
FtlGcFreeTempBlock:
	stp	x29, x30, [sp, -96]!
	mov	w3, 65535
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x2, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	adrp	x21, .LANCHOR0
	ldrh	w20, [x2,96]
	add	x1, x21, :lo12:.LANCHOR0
	cmp	w20, w3
	ldrh	w1, [x1,2026]
	bne	.L2527
.L2537:
	add	x20, x19, :lo12:.LANCHOR2
	mov	w0, 65535
	add	x22, x20, 96
	ldrh	w1, [x20,96]
	str	wzr, [x20,2328]
	cmp	w1, w0
	beq	.L2560
	bl	FtlCacheWriteBack
	mov	w26, 12
	add	x3, x21, :lo12:.LANCHOR0
	ldrb	w0, [x22,7]
	ldr	x1, [x20,-40]
	mov	w22, 0
	ldrh	w2, [x20,96]
	ldrh	w3, [x3,2026]
	mul	w0, w0, w3
	strh	w0, [x1,x2,lsl 1]
	ldr	w0, [x20,184]
	ldrh	w1, [x20,936]
	add	w0, w1, w0
	str	w0, [x20,184]
	b	.L2538
.L2527:
	cbz	w0, .L2530
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	ldrh	w4, [x0,3004]
	cmp	w4, w3
	beq	.L2531
.L2532:
	mov	w1, 2
	b	.L2530
.L2531:
	strh	wzr, [x0,3004]
	ldrh	w0, [x2,-8]
	cmp	w0, 17
	bhi	.L2532
.L2530:
	add	x22, x19, :lo12:.LANCHOR2
	add	x0, x22, 96
	bl	FtlGcScanTempBlk
	str	w0, [x29,92]
	cmn	w0, #1
	beq	.L2533
	ubfiz	x20, x20, 1, 16
	ldr	x1, [x22,-80]
	ldrh	w0, [x1,x20]
	cmp	w0, 4
	bls	.L2534
	sub	w0, w0, #5
	strh	w0, [x1,x20]
	mov	w0, 1
	bl	FtlEctTblFlush
.L2534:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w1, [x0,2328]
	cbnz	w1, .L2535
	ldr	w1, [x0,448]
	add	w1, w1, 1
	str	w1, [x0,448]
	ldr	w0, [x29,92]
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L2535:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, 1
	str	wzr, [x19,2328]
	b	.L2536
.L2533:
	adrp	x0, .LANCHOR1+3004
	mov	w1, 65535
	ldrh	w2, [x0,#:lo12:.LANCHOR1+3004]
	mov	w0, 1
	cmp	w2, w1
	bne	.L2536
	b	.L2537
.L2541:
	ldr	x25, [x20,944]
	add	x1, x29, 92
	umull	x24, w22, w26
	mov	w2, 0
	add	x23, x25, x24
	ldr	w0, [x23,8]
	bl	log2phys
	ldr	w0, [x29,92]
	ldr	w1, [x25,x24]
	cmp	w0, w1
	bne	.L2539
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	mov	w24, w0
	ldr	w0, [x23,8]
	add	x1, x23, 4
	mov	w2, 1
	bl	log2phys
	mov	w0, w24
.L2559:
	bl	decrement_vpc_count
.L2540:
	add	w22, w22, 1
	uxth	w22, w22
.L2538:
	ldrh	w0, [x20,936]
	cmp	w0, w22
	bhi	.L2541
	b	.L2561
.L2539:
	ldr	w1, [x23,4]
	cmp	w0, w1
	beq	.L2540
	ldrh	w0, [x20,96]
	b	.L2559
.L2561:
	mov	w0, -1
	bl	decrement_vpc_count
	add	x0, x21, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L2542
	adrp	x0, .LC126
	ldrh	w1, [x20,96]
	add	x0, x0, :lo12:.LC126
	bl	printk
.L2542:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,96]
	ldr	x1, [x1,-40]
	ubfiz	x2, x0, 1, 16
	ldrh	w1, [x1,x2]
	cbz	w1, .L2543
	bl	INSERT_DATA_LIST
	b	.L2544
.L2543:
	bl	INSERT_FREE_LIST
.L2544:
	add	x20, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	wzr, [x20,936]
	strh	w0, [x20,96]
	strh	wzr, [x20,924]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	w0, [x20,160]
	cbz	w0, .L2545
	ldr	w0, [x20,220]
	cmp	w0, 29
	bhi	.L2545
	ldrh	w0, [x20,272]
	ldrh	w1, [x20,-8]
	cmp	w1, w0
	bcs	.L2546
	lsl	w0, w0, 1
	strh	w0, [x20,864]
.L2546:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	w0, [x19,296]
	b	.L2560
.L2545:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w1, [x19,272]
	ldrh	w0, [x19,-8]
	add	w2, w1, w1, lsl 1
	cmp	w0, w2, lsr 2
	ble	.L2560
	add	x21, x21, :lo12:.LANCHOR0
	mov	w0, -1
	strh	w0, [x19,296]
	ldrb	w0, [x21,120]
	cbz	w0, .L2547
	sub	w1, w1, #2
	strh	w1, [x19,864]
.L2560:
	mov	w0, 0
	b	.L2536
.L2547:
	mov	w1, 20
	strh	w1, [x19,864]
.L2536:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlGcFreeTempBlock, .-FtlGcFreeTempBlock
	.align	2
	.global	Ftl_gc_temp_data_write_back
	.type	Ftl_gc_temp_data_write_back, %function
Ftl_gc_temp_data_write_back:
	adrp	x0, .LANCHOR0+120
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	ldrb	w0, [x0,#:lo12:.LANCHOR0+120]
	stp	x21, x22, [sp,32]
	stp	x19, x20, [sp,16]
	adrp	x21, .LANCHOR2
	cbz	w0, .L2563
	add	x0, x21, :lo12:.LANCHOR2
	ldr	w1, [x0,880]
	tbz	x1, 0, .L2563
	ldrh	w0, [x0,100]
	cbz	w0, .L2563
.L2568:
	mov	w0, 0
	b	.L2564
.L2563:
	add	x1, x21, :lo12:.LANCHOR2
	mov	w2, 0
	mov	w3, w2
	mov	w20, 0
	mov	w22, 56
	ldr	x0, [x1,2056]
	ldr	w1, [x1,880]
	bl	FlashProgPages
.L2565:
	add	x19, x21, :lo12:.LANCHOR2
	ldr	w1, [x19,880]
	cmp	w20, w1
	bcs	.L2579
	umull	x0, w20, w22
	ldr	x2, [x19,2056]
	add	x1, x2, x0
	ldr	w2, [x2,x0]
	cmn	w2, #1
	bne	.L2566
	ldrh	w3, [x19,96]
	ldr	x1, [x19,-40]
	strh	wzr, [x1,x3,lsl 1]
	strh	w2, [x19,96]
	ldr	w1, [x19,448]
	add	w1, w1, 1
	str	w1, [x19,448]
	ldr	x1, [x19,2056]
	add	x0, x1, x0
	ldr	w0, [x0,4]
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	bl	FtlGcPageVarInit
	b	.L2578
.L2566:
	ldr	x2, [x1,16]
	add	w20, w20, 1
	ldr	w1, [x1,4]
	uxth	w20, w20
	ldr	w0, [x2,12]
	ldr	w2, [x2,8]
	bl	FtlGcUpdatePage
	b	.L2565
.L2579:
	ldr	x0, [x19,2056]
	bl	FtlGcBufFree
	str	wzr, [x19,880]
	ldrh	w0, [x19,100]
	cbnz	w0, .L2568
	mov	w0, 1
	bl	FtlGcFreeTempBlock
.L2578:
	mov	w0, 1
.L2564:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	Ftl_gc_temp_data_write_back, .-Ftl_gc_temp_data_write_back
	.align	2
	.global	FtlGcPageRecovery
	.type	FtlGcPageRecovery, %function
FtlGcPageRecovery:
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	adrp	x19, .LANCHOR2
	add	x20, x20, :lo12:.LANCHOR0
	add	x19, x19, :lo12:.LANCHOR2
	str	x21, [sp,32]
	add	x21, x19, 96
	mov	x0, x21
	ldrh	w1, [x20,2026]
	bl	FtlGcScanTempBlk
	ldrh	w1, [x19,98]
	ldrh	w0, [x20,2026]
	cmp	w1, w0
	bcc	.L2580
	add	x0, x19, 2240
	bl	FtlMapBlkWriteDumpData
	mov	w0, 0
	bl	FtlGcFreeTempBlock
	str	wzr, [x19,2328]
.L2580:
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlGcPageRecovery, .-FtlGcPageRecovery
	.align	2
	.global	FtlPowerLostRecovery
	.type	FtlPowerLostRecovery, %function
FtlPowerLostRecovery:
	stp	x29, x30, [sp, -32]!
	adrp	x0, .LANCHOR4+268
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	str	wzr, [x0,#:lo12:.LANCHOR4+268]
	mov	x0, x19
	bl	FtlRecoverySuperblock
	mov	x0, x19
	add	x19, x19, 48
	bl	FtlSlcSuperblockCheck
	mov	x0, x19
	bl	FtlRecoverySuperblock
	mov	x0, x19
	bl	FtlSlcSuperblockCheck
	bl	FtlGcPageRecovery
	mov	w0, -1
	bl	decrement_vpc_count
	ldr	x19, [sp,16]
	mov	w0, 0
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlPowerLostRecovery, .-FtlPowerLostRecovery
	.align	2
	.global	FtlSysBlkInit
	.type	FtlSysBlkInit, %function
FtlSysBlkInit:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	adrp	x21, .LANCHOR4
	adrp	x24, .LANCHOR0
	add	x0, x21, :lo12:.LANCHOR4
	add	x22, x24, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	strh	wzr, [x0,266]
	add	x19, x20, :lo12:.LANCHOR2
	ldrh	w0, [x22,1956]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldrh	w1, [x19,280]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L2584
.L2586:
	mov	w23, -1
	b	.L2585
.L2584:
	bl	FtlLoadSysInfo
	mov	w23, w0
	cbnz	w0, .L2586
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	mov	w0, 1
	bl	FtlUpdateVaildLpn
	ldrh	w2, [x22,2066]
	mov	x1, 0
	ldr	x0, [x19,144]
.L2587:
	cmp	w1, w2
	mov	w3, w1
	bge	.L2591
	add	x4, x0, x1, lsl 4
	add	x1, x1, 1
	ldr	w4, [x4,4]
	tbz	w4, #31, .L2587
.L2591:
	add	x0, x20, :lo12:.LANCHOR2
	cmp	w3, w2
	ldrh	w1, [x0,252]
	add	w1, w1, 1
	strh	w1, [x0,252]
	blt	.L2588
	add	x0, x21, :lo12:.LANCHOR4
	ldrh	w0, [x0,266]
	cbz	w0, .L2592
.L2588:
	add	x19, x20, :lo12:.LANCHOR2
	mov	x0, x19
	add	x22, x19, 48
	bl	FtlSuperblockPowerLostFix
	mov	x0, x22
	bl	FtlSuperblockPowerLostFix
	ldrh	w0, [x20,#:lo12:.LANCHOR2]
	ldr	x2, [x19,-40]
	lsl	x0, x0, 1
	ldrh	w3, [x19,4]
	ldrh	w1, [x2,x0]
	sub	w1, w1, w3
	strh	w1, [x2,x0]
	add	x0, x24, :lo12:.LANCHOR0
	strb	wzr, [x19,6]
	ldr	x3, [x19,-40]
	strh	wzr, [x19,4]
	ldrh	w1, [x0,2026]
	strh	w1, [x19,2]
	ldrh	w1, [x19,48]
	ldrh	w4, [x19,52]
	lsl	x1, x1, 1
	ldrh	w2, [x3,x1]
	sub	w2, w2, w4
	strh	w2, [x3,x1]
	strb	wzr, [x19,54]
	ldrh	w0, [x0,2026]
	strh	w0, [x19,50]
	add	x0, x19, 2240
	strh	wzr, [x19,52]
	bl	FtlMapBlkWriteDumpData
	add	x0, x21, :lo12:.LANCHOR4
	add	x0, x0, 200
	bl	FtlMapBlkWriteDumpData
	ldrh	w0, [x19,254]
	add	w0, w0, 1
	strh	w0, [x19,254]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L2592:
	ldrh	w0, [x20,#:lo12:.LANCHOR2]
	mov	w1, 65535
	add	x19, x20, :lo12:.LANCHOR2
	cmp	w0, w1
	beq	.L2593
	ldrh	w1, [x19,4]
	cbnz	w1, .L2593
	ldrh	w1, [x19,52]
	add	x22, x19, 48
	cbnz	w1, .L2593
	bl	FtlGcRefreshBlock
	ldrh	w0, [x19,48]
	bl	FtlGcRefreshBlock
	mov	x0, x19
	bl	allocate_new_data_superblock
	mov	x0, x22
	bl	allocate_new_data_superblock
	add	x0, x21, :lo12:.LANCHOR4
	add	x0, x0, 200
	bl	FtlMapBlkWriteDumpData
.L2593:
	add	x20, x20, :lo12:.LANCHOR2
	ldrh	w0, [x20,252]
	and	w0, w0, 31
	cbnz	w0, .L2585
	bl	FtlVpcCheckAndModify
.L2585:
	mov	w0, w23
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	FtlLowFormat
	.type	FtlLowFormat, %function
FtlLowFormat:
	stp	x29, x30, [sp, -64]!
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	adrp	x21, .LANCHOR2
	add	x0, x21, :lo12:.LANCHOR2
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	adrp	x20, .LANCHOR0
	str	wzr, [x0,196]
	str	wzr, [x0,200]
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w0, [x0,1956]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbz	w0, .L2600
	bl	FtlMakeBbt
.L2600:
	mov	w0, 0
.L2601:
	add	x1, x20, :lo12:.LANCHOR0
	ldrh	w2, [x1,2032]
	cmp	w0, w2, lsl 7
	bge	.L2628
	add	x3, x21, :lo12:.LANCHOR2
	ubfiz	x2, x0, 2, 16
	mvn	w1, w0
	ldr	x4, [x3,2096]
	orr	w1, w0, w1, lsl 16
	add	w0, w0, 1
	str	w1, [x4,x2]
	mov	w1, 23752
	movk	w1, 0xa0f, lsl 16
	uxth	w0, w0
	ldr	x3, [x3,2104]
	str	w1, [x3,x2]
	b	.L2601
.L2628:
	ldrh	w22, [x1,1960]
	mov	w19, 0
.L2603:
	add	x23, x20, :lo12:.LANCHOR0
	ldrh	w0, [x23,1962]
	cmp	w0, w22
	bls	.L2629
	mov	w0, w22
	mov	w1, 1
	add	w22, w22, 1
	bl	FtlLowFormatEraseBlock
	add	w0, w19, w0
	uxth	w22, w22
	uxth	w19, w0
	b	.L2603
.L2629:
	ldrh	w0, [x23,1952]
	sub	w1, w19, #3
	cmp	w1, w0, lsl 1
	bge	.L2605
.L2609:
	mov	w19, 0
	mov	w22, w19
	b	.L2606
.L2605:
	udiv	w19, w19, w0
	ldr	w0, [x23,2060]
	add	w0, w19, w0
	bl	FtlSysBlkNumInit
	ldrh	w0, [x23,1956]
	bl	FtlFreeSysBlkQueueInit
	ldrh	w19, [x23,1960]
.L2607:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w0, [x0,1962]
	cmp	w0, w19
	bls	.L2609
	mov	w0, w19
	mov	w1, 1
	add	w19, w19, 1
	bl	FtlLowFormatEraseBlock
	uxth	w19, w19
	b	.L2607
.L2606:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w1, [x0,1960]
	cmp	w1, w22
	bls	.L2630
	mov	w0, w22
	mov	w1, 0
	add	w22, w22, 1
	bl	FtlLowFormatEraseBlock
	add	w0, w19, w0
	uxth	w22, w22
	uxth	w19, w0
	b	.L2606
.L2630:
	ldrh	w1, [x0,1962]
	add	x6, x21, :lo12:.LANCHOR2
	ldrh	w2, [x0,1952]
	str	w1, [x0,2108]
	ldr	w1, [x0,1964]
	udiv	w3, w1, w2
	str	w3, [x0,2104]
	ubfx	x5, x3, 5, 16
	add	w4, w5, 36
	strh	w4, [x6,272]
	mov	w4, 24
	mul	w4, w2, w4
	cmp	w19, w4
	ble	.L2611
	sub	w1, w1, w19
	udiv	w1, w1, w2
	str	w1, [x0,2104]
	lsr	w1, w1, 5
	add	w1, w1, 24
	strh	w1, [x6,272]
.L2611:
	add	x4, x21, :lo12:.LANCHOR2
	ldr	w0, [x4,160]
	cmp	w0, 1
	bne	.L2612
	ldrh	w1, [x4,272]
	udiv	w0, w19, w2
	add	w0, w1, w0
	add	w0, w1, w0, asr 2
	strh	w0, [x4,272]
.L2612:
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L2613
	add	x1, x21, :lo12:.LANCHOR2
	udiv	w0, w19, w2
	ldrh	w4, [x1,272]
	add	w0, w4, w0
	add	w0, w4, w0, asr 2
	strh	w0, [x1,272]
.L2613:
	add	x6, x20, :lo12:.LANCHOR0
	ldrh	w1, [x6,2018]
	cbz	w1, .L2615
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w4, [x0,272]
	add	w4, w4, w1, lsr 1
	strh	w4, [x0,272]
	mul	w4, w1, w2
	cmp	w4, w19
	ble	.L2615
	add	w1, w1, 32
	str	w3, [x6,2104]
	add	w1, w5, w1
	strh	w1, [x0,272]
.L2615:
	add	x19, x21, :lo12:.LANCHOR2
	add	x22, x20, :lo12:.LANCHOR0
	adrp	x24, .LANCHOR4
	mov	w23, -1
	ldr	w1, [x22,2104]
	ldrh	w0, [x19,272]
	sub	w0, w1, w0
	add	x1, x24, :lo12:.LANCHOR4
	mul	w0, w0, w2
	str	w0, [x1,192]
	ldrh	w1, [x22,2026]
	mul	w0, w0, w1
	ldrh	w1, [x22,2032]
	str	w0, [x22,2104]
	mul	w0, w0, w1
	str	w0, [x22,2068]
	bl	FtlBbmTblFlush
	ldrh	w2, [x22,1962]
	mov	w1, 0
	ldr	x0, [x19,-40]
	lsl	w2, w2, 1
	bl	ftl_memset
	strh	wzr, [x21,#:lo12:.LANCHOR2]
	mov	w0, 1
	ldrh	w2, [x22,1960]
	strb	w0, [x19,8]
	mov	w1, 255
	ldr	x0, [x20,#:lo12:.LANCHOR0]
	lsr	w2, w2, 3
	mov	x21, x24
	str	wzr, [x19,156]
	strh	w23, [x19,296]
	strh	wzr, [x19,298]
	strb	wzr, [x19,302]
	strb	wzr, [x19,304]
	strh	wzr, [x19,2]
	strb	wzr, [x19,6]
	bl	ftl_memset
.L2617:
	mov	x0, x19
	bl	make_superblock
	ldrb	w0, [x19,7]
	cbnz	w0, .L2618
	ldrh	w1, [x19]
	ldr	x0, [x19,-40]
	strh	w23, [x0,x1,lsl 1]
	ldrh	w0, [x19]
	add	w0, w0, 1
	strh	w0, [x19]
	b	.L2617
.L2618:
	ldr	w0, [x19,196]
	add	x20, x19, 48
	str	w0, [x19,12]
	mov	w22, -1
	add	w0, w0, 1
	ldrh	w1, [x19]
	str	w0, [x19,196]
	ldr	x0, [x19,-40]
	ldrh	w2, [x19,4]
	strh	w2, [x0,x1,lsl 1]
	strh	wzr, [x20,2]
	ldrh	w0, [x19]
	strb	wzr, [x20,6]
	add	w0, w0, 1
	strh	w0, [x19,48]
	mov	w0, 1
	strb	w0, [x20,8]
.L2619:
	mov	x0, x20
	bl	make_superblock
	ldrb	w0, [x20,7]
	cbnz	w0, .L2620
	ldrh	w1, [x20]
	ldr	x0, [x19,-40]
	strh	w22, [x0,x1,lsl 1]
	ldrh	w0, [x20]
	add	w0, w0, 1
	strh	w0, [x20]
	b	.L2619
.L2620:
	ldr	w0, [x19,196]
	add	x21, x21, :lo12:.LANCHOR4
	str	w0, [x20,12]
	add	w0, w0, 1
	ldrh	w1, [x20]
	str	w0, [x19,196]
	ldr	x0, [x19,-40]
	ldrh	w2, [x20,4]
	mov	w20, -1
	strh	w2, [x0,x1,lsl 1]
	strh	w20, [x19,96]
	bl	FtlFreeSysBlkQueueOut
	strh	w0, [x19,280]
	ldr	w0, [x21,192]
	strh	w0, [x19,286]
	ldr	w0, [x19,196]
	str	w0, [x19,288]
	add	w0, w0, 1
	strh	wzr, [x19,282]
	strh	w20, [x19,284]
	str	w0, [x19,196]
	bl	FtlVpcTblFlush
	bl	FtlSysBlkInit
	cbnz	w0, .L2621
	adrp	x0, .LANCHOR1+532
	mov	w1, 1
	str	w1, [x0,#:lo12:.LANCHOR1+532]
.L2621:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
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
	ldrb	w0, [x0,120]
	cbz	w0, .L2632
.L2634:
	mov	w0, 0
	b	.L2633
.L2632:
	adrp	x20, .LANCHOR4
	add	x0, x20, :lo12:.LANCHOR4
	ldr	x0, [x0,32]
	bl	FlashInit
	cbnz	w0, .L2634
	bl	FlashLoadFactorBbt
	cbz	w0, .L2635
	bl	FlashMakeFactorBbt
.L2635:
	add	x20, x20, :lo12:.LANCHOR4
	ldr	x0, [x20,80]
	bl	FlashReadIdbDataRaw
	cbz	w0, .L2636
	mov	w1, 0
	mov	w2, 16
	add	x0, x29, 32
	bl	FlashReadFacBbtData
	mov	w0, 0
	ldr	w2, [x29,32]
	mov	w1, w0
	mov	w4, 1
.L2638:
	lsl	w3, w4, w1
	add	w1, w1, 1
	tst	w3, w2
	cset	w3, ne
	cmp	w1, 16
	add	w0, w0, w3
	bne	.L2638
	cmp	w0, 6
	bhi	.L2639
	add	x0, x19, :lo12:.LANCHOR0
	b	.L2662
.L2639:
	mov	w1, 0
	mov	w4, 1
.L2642:
	lsl	w3, w4, w1
	add	w1, w1, 1
	tst	w3, w2
	cset	w3, ne
	cmp	w1, 24
	add	w0, w0, w3
	bne	.L2642
	cmp	w0, 17
	add	x0, x19, :lo12:.LANCHOR0
	bls	.L2662
	mov	w1, 36
.L2662:
	strb	w1, [x0,9]
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x0,9]
	strh	w1, [x0,194]
.L2636:
	adrp	x0, .LC72
	adrp	x1, .LC73
	add	x1, x1, :lo12:.LC73
	add	x0, x0, :lo12:.LC72
	add	x19, x19, :lo12:.LANCHOR0
	bl	printk
	add	x0, x19, 168
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldrh	w0, [x19,1956]
	mov	w19, 1
	bl	FtlFreeSysBlkQueueInit
.L2644:
	bl	FtlLoadBbt
	cbz	w0, .L2645
.L2664:
	bl	FtlLowFormat
	cmp	w19, 3
	bhi	.L2665
	add	w19, w19, 1
	b	.L2644
.L2665:
	mov	w0, -1
	b	.L2633
.L2645:
	bl	FtlSysBlkInit
	cbnz	w0, .L2664
	adrp	x1, .LANCHOR1+532
	mov	w2, 1
	str	w2, [x1,#:lo12:.LANCHOR1+532]
.L2633:
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlReInitForSDUpdata, .-FtlReInitForSDUpdata
	.align	2
	.global	Ftl_get_new_temp_ppa
	.type	Ftl_get_new_temp_ppa, %function
Ftl_get_new_temp_ppa:
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x0, x19, :lo12:.LANCHOR2
	add	x1, x0, 96
	ldrh	w2, [x0,96]
	mov	w0, 65535
	cmp	w2, w0
	beq	.L2667
	ldrh	w0, [x1,4]
	cbnz	w0, .L2668
.L2667:
	bl	FtlCacheWriteBack
	add	x20, x19, :lo12:.LANCHOR2
	mov	w0, 0
	bl	FtlGcFreeTempBlock
	strb	wzr, [x20,104]
	add	x0, x20, 96
	bl	allocate_data_superblock
	strh	wzr, [x20,924]
	strh	wzr, [x20,936]
	bl	l2p_flush
	mov	w0, 0
	bl	FtlEctTblFlush
	bl	FtlVpcTblFlush
.L2668:
	add	x0, x19, :lo12:.LANCHOR2
	add	x0, x0, 96
	bl	get_new_active_ppa
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	stp	x29, x30, [sp, -144]!
	mov	w2, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR2
	add	x3, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	ldr	w4, [x3,2028]
	cbnz	w4, .L2816
	ldrh	w4, [x3,-24]
	cmp	w4, 47
	bls	.L2816
	mov	w24, w0
	adrp	x0, .LANCHOR1+3004
	mov	w22, w1
	ldrh	w1, [x0,#:lo12:.LANCHOR1+3004]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L2671
.L2674:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w1, [x0,954]
	cmp	w1, w2
	bne	.L2672
	b	.L2673
.L2671:
	ldrh	w1, [x3,96]
	cmp	w1, w0
	beq	.L2674
	mov	w0, 1
	bl	FtlGcFreeTempBlock
	mov	w2, 1
	cbz	w0, .L2674
	b	.L2816
.L2672:
	ldrh	w3, [x0,952]
	cmp	w3, w2
	bne	.L2673
	strh	w1, [x0,952]
	mov	w1, -1
	strh	w1, [x0,954]
.L2673:
	bl	FtlReadRefresh
	add	x1, x19, :lo12:.LANCHOR2
	cmp	w24, 1
	ldr	w0, [x1,868]
	add	w0, w0, 1
	add	w0, w0, w24, lsl 7
	str	w0, [x1,868]
	bne	.L2686
	ldr	w1, [x1,160]
	cbnz	w1, .L2676
	adrp	x1, .LANCHOR0+120
	ldrb	w1, [x1,#:lo12:.LANCHOR0+120]
	cbz	w1, .L2686
.L2676:
	add	x20, x19, :lo12:.LANCHOR2
	ldr	w1, [x20,220]
	cmp	w1, 29
	bhi	.L2686
	adrp	x21, .LANCHOR4
	add	x1, x21, :lo12:.LANCHOR4
	ldrh	w1, [x1,412]
	add	w0, w0, w1
	str	w0, [x20,868]
	bl	FtlGcReFreshBadBlk
	ldrh	w0, [x20,296]
	mov	w1, 65535
	cmp	w0, w1
	bne	.L2686
	ldrh	w1, [x20,952]
	cmp	w1, w0
	bne	.L2686
	ldr	w0, [x20,868]
	cmp	w0, 1024
	bhi	.L2677
	ldrh	w0, [x20,-8]
	cmp	w0, 63
	bhi	.L2686
.L2677:
	add	x1, x19, :lo12:.LANCHOR2
	add	x0, x21, :lo12:.LANCHOR4
	str	wzr, [x1,868]
	ldr	w1, [x1,220]
	strh	wzr, [x0,412]
	cbnz	w1, .L2678
	mov	w1, 6
	b	.L2839
.L2678:
	cmp	w1, 5
	bhi	.L2679
	mov	w1, 18
.L2839:
	strh	w1, [x0,412]
.L2679:
	adrp	x1, .LANCHOR0
	add	x0, x1, :lo12:.LANCHOR0
	mov	x20, x1
	ldrb	w0, [x0,120]
	cbz	w0, .L2680
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w1, [x0,-8]
	ldrh	w0, [x0,866]
	add	w0, w0, 64
	cmp	w1, w0
	bgt	.L2686
.L2680:
	mov	w0, 32
	mov	w27, 65535
	bl	List_get_gc_head_node
	uxth	w25, w0
	cmp	w25, w27
	beq	.L2685
	add	x23, x19, :lo12:.LANCHOR2
	ldrh	w0, [x23,872]
	cbz	w0, .L2682
	add	x1, x20, :lo12:.LANCHOR0
	ldr	x26, [x23,-40]
	ubfiz	x25, x25, 1, 16
	ldrh	w2, [x1,2028]
	ldrh	w1, [x1,1952]
	ldrh	w3, [x26,x25]
	mul	w1, w2, w1
	add	w1, w1, 1
	cmp	w3, w1
	bgt	.L2685
	add	w1, w0, 1
	str	wzr, [x23,876]
	uxth	w1, w1
	strh	w1, [x23,872]
	str	x1, [x29,120]
	bl	List_get_gc_head_node
	uxth	w20, w0
	cmp	w20, w27
	ldr	x1, [x29,120]
	beq	.L2685
	ubfiz	x27, x20, 1, 16
	adrp	x0, .LC127
	ldrh	w4, [x26,x25]
	add	x0, x0, :lo12:.LC127
	mov	w2, w20
	ldrh	w3, [x26,x27]
	bl	printk
	ldrh	w0, [x23,872]
	cmp	w0, 40
	bls	.L2683
	ldr	x0, [x23,-40]
	ldrh	w0, [x0,x27]
	cmp	w0, 32
	bls	.L2683
	strh	wzr, [x23,872]
.L2683:
	add	x21, x21, :lo12:.LANCHOR4
	mov	w0, 6
	strh	w0, [x21,412]
	b	.L2687
.L2682:
	mov	w0, 1
	strh	w0, [x23,872]
.L2685:
	bl	GetSwlReplaceBlock
	uxth	w20, w0
	mov	w0, 65535
	cmp	w20, w0
	bne	.L2687
	add	x21, x21, :lo12:.LANCHOR4
	strh	wzr, [x21,412]
.L2686:
	add	x21, x19, :lo12:.LANCHOR2
	mov	w0, 65535
	ldrh	w20, [x21,296]
	cmp	w20, w0
	bne	.L2842
	ldrh	w0, [x21,96]
	cmp	w0, w20
	beq	.L2843
.L2687:
	mov	w0, 65535
	cmp	w20, w0
	cset	w1, eq
	cbz	w1, .L2700
	cbnz	w24, .L2700
	add	x0, x19, :lo12:.LANCHOR2
	mov	w21, 1
	ldrh	w1, [x0,-8]
	cmp	w1, 24
	bhi	.L2701
	adrp	x0, .LANCHOR0
	cmp	w1, 16
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w21, [x0,2026]
	bls	.L2702
	lsr	w21, w21, 5
	b	.L2701
.L2702:
	cmp	w1, 12
	bls	.L2703
	lsr	w21, w21, 4
	b	.L2701
.L2703:
	cmp	w1, 8
	bls	.L2701
	lsr	w21, w21, 2
.L2701:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w2, [x0,864]
	cmp	w2, w1
	bcs	.L2705
	ldrh	w1, [x0,96]
	mov	w2, 65535
	cmp	w1, w2
	bne	.L2706
	ldrh	w2, [x0,952]
	cmp	w2, w1
	bne	.L2706
	adrp	x1, .LANCHOR4+412
	ldrh	w2, [x1,#:lo12:.LANCHOR4+412]
	cbnz	w2, .L2707
	adrp	x1, .LANCHOR0+2104
	ldr	w3, [x0,156]
	ldr	w1, [x1,#:lo12:.LANCHOR0+2104]
	add	w1, w1, w1, lsl 1
	cmp	w3, w1, lsr 2
	bcs	.L2708
.L2707:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,272]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1,864]
	b	.L2709
.L2708:
	mov	w1, 18
	strh	w1, [x0,864]
.L2709:
	add	x19, x19, :lo12:.LANCHOR2
	str	wzr, [x19,876]
	b	.L2816
.L2706:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,272]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1,864]
.L2705:
	cmp	w22, 2
	bhi	.L2769
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,160]
	cbz	w0, .L2769
	add	w21, w21, 1
	uxth	w21, w21
	b	.L2769
.L2700:
	add	x21, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w0, [x21,96]
	cmp	w0, w2
	bne	.L2712
	cbz	w1, .L2712
	ldrh	w1, [x21,952]
	cmp	w1, w0
	bne	.L2712
	ldrh	w1, [x21,296]
	cmp	w1, w0
	beq	.L2713
.L2718:
	mov	w20, 65535
	b	.L2712
.L2713:
	ldrh	w23, [x21,-8]
	adrp	x20, .LANCHOR4
	ldrh	w0, [x21,864]
	str	wzr, [x21,876]
	cmp	w0, w23
	bcs	.L2714
	add	x0, x20, :lo12:.LANCHOR4
	ldrh	w0, [x0,412]
	cbnz	w0, .L2715
	adrp	x0, .LANCHOR0+2104
	ldr	w1, [x21,156]
	ldr	w0, [x0,#:lo12:.LANCHOR0+2104]
	add	w0, w0, w0, lsl 1
	cmp	w1, w0, lsr 2
	bcs	.L2716
.L2715:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w0, [x19,272]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x19,864]
	b	.L2717
.L2716:
	mov	w0, 18
	strh	w0, [x21,864]
.L2717:
	add	x20, x20, :lo12:.LANCHOR4
	bl	FtlReadRefresh
	ldrh	w2, [x20,412]
	b	.L2816
.L2714:
	add	x22, x20, :lo12:.LANCHOR4
	ldrh	w0, [x22,412]
	cbnz	w0, .L2718
	ldrh	w20, [x21,272]
	add	w1, w20, w20, lsl 1
	asr	w1, w1, 2
	strh	w1, [x21,864]
	bl	List_get_gc_head_node
	ubfiz	x0, x0, 1, 16
	ldr	x1, [x21,-40]
	ldrh	w2, [x1,x0]
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w0, [x1,2028]
	ldrh	w1, [x1,1952]
	mul	w0, w0, w1
	mov	w1, 2
	sdiv	w0, w0, w1
	cmp	w2, w0
	ble	.L2719
	sub	w20, w20, #1
	cmp	w23, w20
	blt	.L2719
	bl	FtlReadRefresh
	ldrh	w2, [x22,412]
	b	.L2816
.L2719:
	cbnz	w2, .L2718
	mov	w0, -1
	add	x19, x19, :lo12:.LANCHOR2
	bl	decrement_vpc_count
	ldrh	w2, [x19,-8]
	add	w2, w2, 1
	b	.L2816
.L2712:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w21, 2
	ldr	w0, [x0,160]
	cmp	w0, wzr
	csinc	w21, w21, wzr, ne
	b	.L2711
.L2769:
	mov	w20, 65535
.L2711:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w1, [x0,296]
	cmp	w1, w2
	bne	.L2721
	cmp	w20, w1
	beq	.L2722
	strh	w20, [x0,296]
	b	.L2723
.L2722:
	ldrh	w1, [x0,952]
	cmp	w1, w20
	beq	.L2723
	ubfiz	x1, x1, 1, 16
	ldr	x2, [x0,-40]
	ldrh	w1, [x2,x1]
	cbnz	w1, .L2724
	mov	w1, -1
	strh	w1, [x0,952]
.L2724:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w1, [x0,952]
	strh	w1, [x0,296]
	mov	w1, -1
	strh	w1, [x0,952]
.L2723:
	add	x22, x19, :lo12:.LANCHOR2
	mov	w1, 65535
	ldrh	w0, [x22,296]
	strb	wzr, [x22,304]
	cmp	w0, w1
	beq	.L2721
	bl	IsBlkInGcList
	cbz	w0, .L2726
	mov	w0, -1
	strh	w0, [x22,296]
.L2726:
	adrp	x0, .LANCHOR0+120
	ldrb	w0, [x0,#:lo12:.LANCHOR0+120]
	cbz	w0, .L2727
	add	x0, x19, :lo12:.LANCHOR2
	add	x22, x0, 296
	ldrh	w0, [x0,296]
	bl	ftl_get_blk_mode
	strb	w0, [x22,8]
.L2727:
	add	x22, x19, :lo12:.LANCHOR2
	mov	w0, 65535
	add	x23, x22, 296
	ldrh	w1, [x22,296]
	cmp	w1, w0
	beq	.L2721
	mov	x0, x23
	bl	make_superblock
	strh	wzr, [x22,298]
	adrp	x0, .LANCHOR4
	ldrh	w2, [x22,296]
	add	x0, x0, :lo12:.LANCHOR4
	ldr	x1, [x22,-40]
	strb	wzr, [x22,302]
	strh	wzr, [x0,414]
	ldrh	w1, [x1,x2,lsl 1]
	strh	w1, [x0,416]
.L2721:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w2, [x19,#:lo12:.LANCHOR2]
	ldrh	w0, [x1,296]
	cmp	w2, w0
	beq	.L2728
	ldrh	w1, [x1,48]
	cmp	w1, w0
	bne	.L2729
.L2728:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, -1
	strh	w1, [x0,296]
.L2729:
	add	x28, x19, :lo12:.LANCHOR2
	mov	w24, 65535
.L2756:
	ldrh	w0, [x28,296]
	cmp	w0, w24
	bne	.L2730
	str	wzr, [x28,876]
.L2731:
	ldrh	w22, [x28,872]
	mov	w0, w22
	bl	List_get_gc_head_node
	uxth	w23, w0
	cmp	w23, w24
	strh	w23, [x28,296]
	bne	.L2732
	strh	wzr, [x28,872]
	mov	w2, 8
	b	.L2816
.L2732:
	mov	w0, w23
	add	w22, w22, 1
	bl	IsBlkInGcList
	cbz	w0, .L2733
	strh	w22, [x28,872]
	b	.L2731
.L2733:
	adrp	x0, .LANCHOR0
	ubfiz	x1, x23, 1, 16
	add	x5, x0, :lo12:.LANCHOR0
	ldr	x2, [x28,-40]
	uxth	w22, w22
	mov	w3, 2
	strh	w22, [x28,872]
	ldrh	w0, [x5,2026]
	ldrh	w5, [x5,1952]
	ldrh	w4, [x2,x1]
	mul	w0, w0, w5
	sdiv	w5, w0, w3
	cmp	w4, w5
	bgt	.L2735
	cmp	w4, 8
	bls	.L2736
	cmp	w22, 48
	bls	.L2736
	ldrh	w4, [x28,924]
	cmp	w4, 35
	bhi	.L2736
.L2735:
	strh	wzr, [x28,872]
.L2736:
	ldrh	w1, [x2,x1]
	cmp	w1, w0
	blt	.L2737
	cmp	w20, w24
	bne	.L2737
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	w0, [x19,296]
	adrp	x0, .LANCHOR4+412
	strh	wzr, [x19,872]
	ldrh	w2, [x0,#:lo12:.LANCHOR4+412]
	b	.L2816
.L2737:
	cbnz	w1, .L2738
	mov	w0, -1
	bl	decrement_vpc_count
	ldrh	w0, [x28,872]
	add	w0, w0, 1
	strh	w0, [x28,872]
	b	.L2731
.L2738:
	adrp	x0, .LANCHOR0
	strb	wzr, [x28,304]
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L2739
	mov	w0, w23
	bl	ftl_get_blk_mode
	strb	w0, [x28,304]
.L2739:
	add	x22, x28, 296
	mov	x0, x22
	bl	make_superblock
	adrp	x1, .LANCHOR4
	ldrh	w2, [x28,296]
	add	x1, x1, :lo12:.LANCHOR4
	ldr	x0, [x28,-40]
	strh	wzr, [x1,414]
	ldrh	w0, [x0,x2,lsl 1]
	strh	w0, [x1,416]
	strh	wzr, [x28,298]
	strb	wzr, [x28,302]
.L2730:
	mov	w0, 1
	str	w0, [x28,2028]
	adrp	x0, .LANCHOR0
	adrp	x23, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w1, [x0,120]
	ldrh	w25, [x0,2026]
	cbz	w1, .L2740
	ldrb	w1, [x28,304]
	cmp	w1, 1
	bne	.L2740
	ldrh	w25, [x0,2028]
.L2740:
	ldrh	w0, [x28,298]
	add	w1, w0, w21
	cmp	w1, w25
	ble	.L2741
	sub	w21, w25, w0
	uxth	w21, w21
.L2741:
	mov	w26, 0
.L2742:
	cmp	w21, w26, uxth
	bls	.L2749
	add	x1, x28, 296
	add	x0, x23, :lo12:.LANCHOR0
	ldrh	w4, [x1,2]
	ldrh	w6, [x0,1952]
	mov	x0, 0
	add	w4, w4, w26
	mov	w22, w0
.L2750:
	cmp	w6, w0, uxth
	bls	.L2844
	add	x2, x1, x0, lsl 1
	ldrh	w2, [x2,16]
	cmp	w2, w24
	beq	.L2743
	mov	w3, 56
	ldr	x7, [x28,912]
	orr	w2, w4, w2, lsl 10
	umull	x5, w22, w3
	add	w22, w22, 1
	add	x5, x7, x5
	uxth	w22, w22
	str	w2, [x5,4]
.L2743:
	add	x0, x0, 1
	b	.L2750
.L2844:
	ldr	x0, [x28,912]
	mov	w1, w22
	ldrb	w2, [x28,304]
	mov	x27, 0
	bl	FlashReadPages
.L2745:
	cmp	w22, w27, uxth
	bls	.L2845
	mov	x0, 56
	mul	x5, x27, x0
	ldr	x0, [x28,912]
	add	x1, x0, x5
	ldr	w0, [x0,x5]
	cmn	w0, #1
	ldr	x4, [x1,16]
	beq	.L2771
	ldrh	w0, [x4]
	mov	w1, 61589
	cmp	w0, w1
	bne	.L2771
	ldr	w0, [x4,8]
	add	x1, x29, 140
	mov	w2, 0
	str	x5, [x29,112]
	str	x4, [x29,120]
	bl	log2phys
	ldr	x2, [x28,912]
	ldr	x5, [x29,112]
	ldr	w0, [x29,140]
	add	x2, x2, x5
	ldr	x4, [x29,120]
	and	w0, w0, 2147483647
	ldr	w1, [x2,4]
	cmp	w0, w1
	bne	.L2771
	adrp	x0, .LANCHOR4
	mov	x3, 56
	add	x1, x0, :lo12:.LANCHOR4
	str	x5, [x29,104]
	str	x4, [x29,112]
	ldrh	w0, [x1,414]
	add	w0, w0, 1
	strh	w0, [x1,414]
	ldr	w0, [x28,880]
	ldr	x1, [x28,2056]
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
	ldr	w0, [x28,880]
	ldr	x1, [x28,2056]
	madd	x0, x0, x2, x1
	ldr	x1, [x28,912]
	add	x1, x1, x5
	ldr	x2, [x1,8]
	str	x2, [x0,8]
	add	x2, x28, 96
	str	x2, [x29,120]
	ldr	x1, [x1,16]
	str	x1, [x0,16]
	mov	w1, 1
	ldr	w0, [x29,140]
	str	w0, [x4,12]
	ldrh	w0, [x28,96]
	strh	w0, [x4,2]
	ldr	w0, [x28,200]
	str	w0, [x4,4]
	ldr	w0, [x28,880]
	add	w0, w0, 1
	str	w0, [x28,880]
	ldr	x0, [x28,912]
	add	x0, x0, x5
	bl	FtlGcBufAlloc
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbnz	w0, .L2747
	ldr	x2, [x29,120]
	ldr	w1, [x28,880]
	ldrb	w0, [x2,7]
	cmp	w1, w0
	beq	.L2747
	ldrh	w0, [x2,4]
	cbnz	w0, .L2771
.L2747:
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L2771
	adrp	x0, .LANCHOR4
	add	x19, x19, :lo12:.LANCHOR2
	add	x0, x0, :lo12:.LANCHOR4
	str	wzr, [x19,2028]
	ldrh	w2, [x0,412]
	b	.L2816
.L2771:
	add	x27, x27, 1
	b	.L2745
.L2845:
	add	w26, w26, 1
	b	.L2742
.L2749:
	ldrh	w0, [x28,298]
	add	w21, w21, w0
	uxth	w21, w21
	strh	w21, [x28,298]
	cmp	w21, w25
	adrp	x21, .LANCHOR4
	bcs	.L2751
	add	x0, x21, :lo12:.LANCHOR4
	ldrh	w1, [x0,414]
	ldrh	w0, [x0,416]
	cmp	w1, w0
	bne	.L2752
.L2751:
	ldr	w0, [x28,880]
	cbz	w0, .L2753
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L2753
	add	x21, x21, :lo12:.LANCHOR4
	str	wzr, [x28,2028]
	ldrh	w2, [x21,412]
	b	.L2816
.L2753:
	add	x0, x21, :lo12:.LANCHOR4
	ldrh	w2, [x0,414]
	cbnz	w2, .L2754
	ldrh	w0, [x28,296]
	ldr	x1, [x28,-40]
	lsl	x0, x0, 1
	ldrh	w4, [x1,x0]
	cbz	w4, .L2754
	strh	w2, [x1,x0]
	ldrh	w0, [x28,296]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2754:
	mov	w0, -1
	strh	w0, [x28,296]
.L2752:
	ldrh	w0, [x28,-8]
	str	wzr, [x28,2028]
	cmp	w0, 2
	bhi	.L2755
	add	x23, x23, :lo12:.LANCHOR0
	ldrh	w21, [x23,2026]
	b	.L2756
.L2755:
	add	x21, x21, :lo12:.LANCHOR4
	ldrh	w2, [x21,412]
	cmp	w2, wzr
	csinc	w2, w2, w0, ne
	b	.L2816
.L2843:
	ldrh	w25, [x21,952]
	cmp	w25, w20
	bne	.L2687
	ldrh	w0, [x21,-8]
	mov	w2, 1024
	ldr	w1, [x21,868]
	cmp	w0, 24
	mov	w0, 5120
	csel	w0, w0, w2, cc
	cmp	w1, w0
	bls	.L2687
	adrp	x0, .LANCHOR4+412
	str	wzr, [x21,868]
	mov	x23, x21
	strh	wzr, [x0,#:lo12:.LANCHOR4+412]
	bl	GetSwlReplaceBlock
	uxth	w20, w0
	cmp	w20, w25
	bne	.L2689
	ldrh	w1, [x21,-8]
	ldrh	w0, [x21,866]
	cmp	w1, w0
	bcs	.L2690
	mov	w0, 64
	bl	List_get_gc_head_node
	uxth	w0, w0
	cmp	w0, w20
	beq	.L2699
	ldr	w2, [x21,2024]
	uxtw	x1, w0
	adrp	x21, .LANCHOR0
	cbnz	w2, .L2692
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w2, [x0,1972]
	cmp	w2, 3
	beq	.L2692
	ldr	w2, [x23,344]
	cbnz	w2, .L2692
	ldr	w2, [x23,160]
	cbnz	w2, .L2692
	ldrb	w0, [x0,120]
	cbz	w0, .L2693
.L2692:
	add	x0, x19, :lo12:.LANCHOR2
	add	x3, x21, :lo12:.LANCHOR0
	ldr	x0, [x0,-40]
	ldrh	w4, [x0,x1,lsl 1]
	ldrh	w1, [x3,2028]
	ldrh	w0, [x3,1952]
	ldrh	w3, [x3,1972]
	mul	w2, w1, w0
	cmp	w3, 3
	mov	w0, 0
	lsr	w1, w1, 1
	csel	w0, w1, w0, eq
	add	w0, w2, w0
	cmp	w4, w0
	bgt	.L2695
	mov	w0, 0
	add	x21, x21, :lo12:.LANCHOR0
	bl	List_get_gc_head_node
	uxth	w20, w0
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w0, [x21,2104]
	ldr	w2, [x1,156]
	add	w0, w0, w0, lsl 1
	cmp	w2, w0, lsr 2
	bls	.L2696
	mov	w0, 128
	b	.L2840
.L2696:
	mov	w0, 160
.L2840:
	strh	w0, [x1,866]
	b	.L2697
.L2695:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 128
	b	.L2841
.L2693:
	ldr	x2, [x23,-40]
	ldrh	w1, [x2,x1,lsl 1]
	cmp	w1, 7
	bhi	.L2698
	bl	List_get_gc_head_node
	uxth	w20, w0
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 128
	strh	w1, [x0,866]
	b	.L2697
.L2698:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 64
.L2841:
	strh	w1, [x0,866]
	b	.L2699
.L2690:
	mov	w0, 80
	strh	w0, [x21,866]
	b	.L2699
.L2697:
	mov	w0, 65535
	cmp	w20, w0
	beq	.L2699
.L2689:
	add	x5, x19, :lo12:.LANCHOR2
	ubfiz	x4, x20, 1, 32
	adrp	x0, .LC128
	mov	w1, w20
	add	x0, x0, :lo12:.LC128
	ldr	x3, [x5,-40]
	ldr	x6, [x5,-80]
	ldrh	w2, [x5,-8]
	ldrh	w3, [x3,x4]
	ldrh	w5, [x5,864]
	ldrh	w4, [x6,x4]
	bl	printk
.L2699:
	bl	FtlGcReFreshBadBlk
	b	.L2687
.L2842:
	mov	w20, w0
	b	.L2687
.L2816:
	mov	w0, w2
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	2
	.global	FtlRead
	.type	FtlRead, %function
FtlRead:
	stp	x29, x30, [sp, -176]!
	uxtb	w0, w0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	cmp	w0, 16
	mov	w20, w1
	mov	w28, w2
	mov	x26, x3
	bne	.L2847
	add	w0, w1, 256
	mov	w1, w2
	mov	x2, x3
	bl	FtlVendorPartRead
	b	.L2848
.L2847:
	adrp	x21, .LANCHOR0
	add	w0, w1, w2
	add	x1, x21, :lo12:.LANCHOR0
	str	w0, [x29,152]
	mov	w0, -1
	ldr	w3, [x29,152]
	ldr	w2, [x1,2068]
	cmp	w3, w2
	bhi	.L2848
	ldrh	w0, [x1,2032]
	sub	w25, w3, #1
	adrp	x19, .LANCHOR2
	udiv	w1, w20, w0
	str	w1, [x29,156]
	udiv	w25, w25, w0
	ldr	w0, [x29,156]
	sub	w24, w25, w0
	add	x0, x19, :lo12:.LANCHOR2
	add	w24, w24, 1
	ldr	w1, [x0,192]
	add	w1, w28, w1
	str	w1, [x0,192]
	ldr	w1, [x0,164]
	add	w1, w24, w1
	str	w1, [x0,164]
	ldr	w0, [x29,156]
	mov	w1, w25
	bl	FtlCacheMetchLpa
	cbz	w0, .L2849
	bl	FtlCacheWriteBack
.L2849:
	mov	w27, 0
	ldr	w22, [x29,156]
	str	w27, [x29,140]
	mov	w23, w27
	str	w27, [x29,136]
.L2850:
	cbz	w24, .L2892
	mov	w0, w22
	add	x1, x29, 172
	mov	w2, 0
	bl	log2phys
	ldr	w3, [x29,172]
	cmn	w3, #1
	bne	.L2889
	mov	w3, 0
.L2851:
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,2032]
	cmp	w3, w0
	bcs	.L2855
	madd	w0, w22, w0, w3
	cmp	w0, w20
	bcc	.L2853
	ldr	w1, [x29,152]
	cmp	w0, w1
	bcs	.L2853
	sub	w0, w0, w20
	mov	w1, 0
	ubfiz	x0, x0, 9, 23
	mov	w2, 512
	add	x0, x26, x0
	str	x3, [x29,144]
	bl	ftl_memset
	ldr	x3, [x29,144]
.L2853:
	add	w3, w3, 1
	b	.L2851
.L2889:
	mov	w0, 56
	umull	x1, w23, w0
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x2, [x0,2048]
	add	x2, x2, x1
	str	w3, [x2,4]
	ldr	w2, [x29,156]
	cmp	w22, w2
	ldr	x2, [x0,2048]
	add	x2, x2, x1
	bne	.L2856
	ldr	x0, [x0,2096]
	str	x0, [x2,8]
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,2032]
	udiv	w3, w20, w0
	msub	w3, w3, w0, w20
	str	w3, [x29,112]
	sub	w3, w0, w3
	cmp	w3, w28
	csel	w3, w3, w28, ls
	str	w3, [x29,140]
	cmp	w3, w0
	bne	.L2857
	str	x26, [x2,8]
	b	.L2857
.L2856:
	cmp	w22, w25
	bne	.L2858
	ldr	x0, [x0,2104]
	str	x0, [x2,8]
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w4, [x29,152]
	ldrh	w3, [x0,2032]
	mul	w0, w22, w3
	sub	w27, w4, w0
	cmp	w27, w3
	bne	.L2857
	b	.L2890
.L2858:
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,2032]
	mul	w0, w22, w0
.L2890:
	sub	w0, w0, w20
	ubfiz	x0, x0, 9, 23
	add	x0, x26, x0
	str	x0, [x2,8]
.L2857:
	add	x2, x19, :lo12:.LANCHOR2
	ldr	x0, [x2,2048]
	ldr	x2, [x2,2120]
	add	x1, x0, x1
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,2038]
	str	w22, [x1,24]
	mul	w0, w23, w0
	add	w23, w23, 1
	and	x0, x0, 4294967292
	add	x0, x2, x0
	str	x0, [x1,16]
.L2855:
	subs	w24, w24, #1
	add	w22, w22, 1
	beq	.L2859
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,1952]
	cmp	w23, w0, lsl 3
	bne	.L2850
.L2859:
	cbz	w23, .L2850
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, w23
	mov	w2, 0
	ldr	x0, [x0,2048]
	bl	FlashReadPages
	str	xzr, [x29,144]
	ldr	x0, [x29,112]
	ubfiz	x0, x0, 9, 23
	str	x0, [x29,128]
	ldr	w0, [x29,140]
	lsl	w0, w0, 9
	str	w0, [x29,108]
	lsl	w0, w27, 9
	str	w0, [x29,104]
.L2861:
	ldr	w0, [x29,144]
	cmp	w23, w0
	bls	.L2893
	ldr	x0, [x29,144]
	add	x2, x19, :lo12:.LANCHOR2
	mov	x1, 56
	ldr	w4, [x29,156]
	mul	x3, x0, x1
	ldr	x0, [x2,2048]
	add	x0, x0, x3
	ldr	w1, [x0,24]
	cmp	w1, w4
	bne	.L2862
	ldr	x1, [x0,8]
	ldr	x0, [x2,2096]
	cmp	x1, x0
	bne	.L2863
	ldr	x2, [x29,128]
	mov	x0, x26
	str	x3, [x29,96]
	add	x1, x1, x2
	ldr	w2, [x29,108]
	b	.L2891
.L2862:
	cmp	w1, w25
	bne	.L2863
	ldr	x1, [x0,8]
	ldr	x0, [x2,2104]
	cmp	x1, x0
	bne	.L2863
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w2, [x29,104]
	str	x3, [x29,96]
	ldrh	w0, [x0,2032]
	mul	w0, w25, w0
	sub	w0, w0, w20
	ubfiz	x0, x0, 9, 23
	add	x0, x26, x0
.L2891:
	bl	ftl_memcpy
	ldr	x3, [x29,96]
.L2863:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x1, [x0,2048]
	add	x5, x1, x3
	ldr	w4, [x1,x3]
	cmn	w4, #1
	bne	.L2864
	ldr	w2, [x0,424]
	str	w4, [x29,136]
	add	w2, w2, 1
	str	w2, [x0,424]
.L2864:
	ldr	w0, [x1,x3]
	cmp	w0, 256
	bne	.L2865
	ldr	w0, [x5,4]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L2865:
	ldr	x0, [x29,144]
	add	x0, x0, 1
	str	x0, [x29,144]
	b	.L2861
.L2893:
	mov	w23, 0
	b	.L2850
.L2892:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w0, [x19,956]
	cbz	w0, .L2868
	mov	w0, w24
	mov	w1, 1
	bl	rk_ftl_garbage_collect
.L2868:
	ldr	w0, [x29,136]
.L2848:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 176
	ret
	.size	FtlRead, .-FtlRead
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	stp	x29, x30, [sp, -32]!
	adrp	x1, .LC73
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR1
	mov	x20, x0
	add	x19, x19, :lo12:.LANCHOR1
	mov	w0, -1
	add	x1, x1, :lo12:.LC73
	str	w0, [x19,532]
	adrp	x0, .LANCHOR4+420
	str	wzr, [x0,#:lo12:.LANCHOR4+420]
	adrp	x0, .LC72
	add	x0, x0, :lo12:.LC72
	bl	printk
	mov	x0, x20
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	adrp	x0, .LANCHOR0+1956
	ldrh	w0, [x0,#:lo12:.LANCHOR0+1956]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbnz	w0, .L2898
	bl	FtlSysBlkInit
	cbnz	w0, .L2898
	mov	w0, 1
	str	w0, [x19,532]
	adrp	x0, .LANCHOR2-8
	ldrh	w0, [x0,#:lo12:.LANCHOR2-8]
	cmp	w0, 15
	bhi	.L2898
	mov	w19, 8129
.L2897:
	mov	w0, 0
	mov	w1, 1
	bl	rk_ftl_garbage_collect
	subs	w19, w19, #1
	bne	.L2897
.L2898:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	FtlInit, .-FtlInit
	.align	2
	.global	rk_ftl_init
	.type	rk_ftl_init, %function
rk_ftl_init:
	stp	x29, x30, [sp, -32]!
	mov	w0, 2048
	add	x29, sp, 0
	str	x19, [sp,16]
	adrp	x19, .LANCHOR4
	add	x19, x19, :lo12:.LANCHOR4
	bl	ftl_malloc
	add	x1, x19, 432
	str	x0, [x19,424]
	add	x0, x19, 32
	str	xzr, [x19,32]
	str	xzr, [x19,432]
	bl	rknand_get_reg_addr
	ldr	x1, [x19,32]
	mov	w0, -1
	cbz	x1, .L2901
	bl	rk_nandc_irq_init
	mov	w1, 0
	ldr	x0, [x19,424]
	mov	w2, w1
	mov	w3, 2048
	bl	FlashSramLoadStore
	bl	rknand_flash_cs_init
	ldr	x0, [x19,32]
	bl	FlashInit
	mov	w19, w0
	cbnz	w0, .L2902
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, 168
	bl	FtlInit
.L2902:
	adrp	x0, .LC129
	mov	w1, w19
	add	x0, x0, :lo12:.LC129
	bl	printk
	mov	w0, w19
.L2901:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	rk_ftl_init, .-rk_ftl_init
	.align	2
	.global	rk_ftl_cache_write_back
	.type	rk_ftl_cache_write_back, %function
rk_ftl_cache_write_back:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	bl	FtlCacheWriteBack
	ldp	x29, x30, [sp], 16
	ret
	.size	rk_ftl_cache_write_back, .-rk_ftl_cache_write_back
	.align	2
	.global	FtlWrite
	.type	FtlWrite, %function
FtlWrite:
	stp	x29, x30, [sp, -272]!
	uxtb	w0, w0
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	cmp	w0, 16
	mov	w22, w1
	mov	w23, w2
	mov	x24, x3
	bne	.L2907
	add	w0, w1, 256
	mov	w1, w2
	mov	x2, x3
	bl	FtlVendorPartWrite
	b	.L2908
.L2907:
	adrp	x20, .LANCHOR0
	add	w26, w1, w2
	add	x2, x20, :lo12:.LANCHOR0
	mov	w0, -1
	ldr	w1, [x2,2068]
	cmp	w26, w1
	bhi	.L2908
	adrp	x6, .LANCHOR4
	ldrh	w1, [x2,2032]
	add	x28, x6, :lo12:.LANCHOR4
	sub	w26, w26, #1
	mov	w0, 2048
	adrp	x5, .LANCHOR2
	udiv	w25, w22, w1
	cmp	w23, w1, lsl 1
	str	w0, [x28,440]
	str	x6, [x29,152]
	str	x5, [x29,120]
	udiv	w0, w26, w1
	str	w0, [x29,180]
	sub	w26, w0, w25
	add	w27, w26, 1
	add	x0, x5, :lo12:.LANCHOR2
	add	x19, x0, 48
	ldr	w3, [x0,172]
	add	w3, w27, w3
	str	w3, [x0,172]
	ldr	w3, [x0,188]
	add	w3, w23, w3
	str	w3, [x0,188]
	cset	w3, cs
	str	w3, [x29,204]
	ldr	w3, [x2,2088]
	cmp	w23, 8
	csel	x19, x0, x19, hi
	cbz	w3, .L2910
	sub	w3, w3, #1
	mov	w5, 56
	ldr	x2, [x2,2096]
	umull	x3, w3, w5
	add	x3, x2, x3
	ldr	w2, [x3,24]
	cmp	w25, w2
	bne	.L2911
	ldr	w2, [x0,176]
	add	w2, w2, 1
	str	w2, [x0,176]
	ldr	w0, [x28,444]
	add	w0, w0, 1
	str	w0, [x28,444]
	msub	w0, w25, w1, w22
	sub	w1, w1, w0
	ubfiz	x0, x0, 9, 23
	cmp	w1, w23
	csel	w19, w1, w23, ls
	ldr	x1, [x3,8]
	lsl	w21, w19, 9
	add	x0, x1, x0
	mov	w2, w21
	mov	x1, x24
	bl	ftl_memcpy
	cbnz	w26, .L2912
	ldr	w0, [x28,444]
	cmp	w0, 2
	bgt	.L2912
.L2945:
	mov	w0, 0
	b	.L2908
.L2912:
	sub	w23, w23, w19
	add	w22, w22, w19
	add	x24, x24, x21
	add	w25, w25, 1
	mov	w27, w26
.L2911:
	ldr	x0, [x29,152]
	add	x0, x0, :lo12:.LANCHOR4
	ldr	x19, [x0,400]
	str	wzr, [x0,444]
.L2910:
	ldr	w1, [x29,180]
	mov	w0, w25
	bl	FtlCacheMetchLpa
	cbz	w0, .L2913
	bl	FtlCacheWriteBack
.L2913:
	ldr	x0, [x29,152]
	str	w25, [x29,200]
	add	x0, x0, :lo12:.LANCHOR4
	str	x19, [x0,400]
	ldr	x0, [x29,120]
	add	x21, x0, :lo12:.LANCHOR2
	adrp	x0, .LC130
	add	x0, x0, :lo12:.LC130
	str	x0, [x29,104]
.L2914:
	cbz	w27, .L2975
	ldrh	w0, [x19,4]
	cbnz	w0, .L2915
	cmp	x19, x21
	bne	.L2916
	add	x0, x21, 48
	ldrh	w19, [x0,4]
	cbnz	w19, .L2917
	bl	allocate_new_data_superblock
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	str	w19, [x0,3008]
.L2917:
	mov	x0, x21
	add	x19, x21, 48
	bl	allocate_new_data_superblock
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	ldr	w0, [x0,3008]
	cbnz	w0, .L2918
.L2919:
	mov	x19, x21
	b	.L2918
.L2916:
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	str	w0, [x1,3008]
	ldrh	w0, [x21,4]
	cbnz	w0, .L2919
	mov	x0, x19
	bl	allocate_new_data_superblock
.L2918:
	ldrh	w0, [x19,4]
	cbnz	w0, .L2920
	mov	x0, x19
	bl	allocate_new_data_superblock
.L2920:
	ldr	x0, [x29,152]
	add	x0, x0, :lo12:.LANCHOR4
	str	x19, [x0,400]
.L2915:
	add	x2, x20, :lo12:.LANCHOR0
	ldr	w0, [x21,2040]
	ldrh	w1, [x19,4]
	str	xzr, [x29,192]
	ldr	w2, [x2,2088]
	sub	w0, w0, w2
	cmp	w1, w0
	csel	w0, w1, w0, ls
	cmp	w0, w27
	csel	w0, w0, w27, ls
	str	w0, [x29,148]
	ldr	w0, [x29,204]
	and	w0, w0, 1
	str	w0, [x29,116]
	add	w0, w23, w22
	str	w0, [x29,144]
.L2921:
	ldr	w0, [x29,192]
	ldr	w2, [x29,148]
	ldr	w1, [x29,200]
	cmp	w0, w2
	add	w26, w1, w0
	bcs	.L2976
	ldrh	w1, [x19,4]
	cbz	w1, .L2952
	ldr	w1, [x29,180]
	cmp	w26, w1
	cset	w7, eq
	cbz	w0, .L2923
	ldr	w1, [x29,116]
	tst	w7, w1
	beq	.L2923
	add	x1, x20, :lo12:.LANCHOR0
	ldr	w2, [x29,144]
	ldrh	w1, [x1,2032]
	msub	w2, w26, w1, w2
	cmp	w2, w1
	bne	.L2952
.L2923:
	add	x1, x29, 212
	mov	w2, 0
	mov	w0, w26
	str	x7, [x29,96]
	bl	log2phys
	mov	x0, x19
	bl	get_new_active_ppa
	add	x4, x20, :lo12:.LANCHOR0
	mov	w5, 56
	str	x4, [x29,184]
	str	x5, [x29,160]
	ldr	w6, [x4,2088]
	ldr	x2, [x4,2096]
	umull	x1, w6, w5
	add	x2, x2, x1
	str	w0, [x2,4]
	ldrh	w2, [x4,2038]
	ldr	x0, [x4,2096]
	add	x1, x0, x1
	mul	w0, w2, w6
	and	x0, x0, 4294967292
	str	x0, [x29,136]
	ldr	x0, [x21,2128]
	ldr	x3, [x29,136]
	str	x0, [x29,128]
	add	x28, x0, x3
	ldrh	w0, [x4,2036]
	str	x28, [x1,16]
	str	w26, [x1,24]
	mul	w0, w0, w6
	ldr	x6, [x21,2088]
	and	x0, x0, 4294967292
	add	x0, x6, x0
	str	x0, [x1,8]
	mov	x0, x28
	mov	w1, 0
	bl	ftl_memset
	cmp	w26, w25
	ldr	x4, [x29,184]
	cset	w0, eq
	str	w0, [x29,176]
	cbnz	w0, .L2955
	ldr	x7, [x29,96]
	ldr	x5, [x29,160]
	cbz	w7, .L2924
	ldr	w0, [x29,144]
	ldrh	w2, [x4,2032]
	msub	w2, w26, w2, w0
	uxth	w0, w2
	str	w0, [x29,184]
	ldr	w0, [x29,176]
	str	w0, [x29,160]
	b	.L2927
.L2955:
	ldrh	w2, [x4,2032]
	udiv	w0, w22, w2
	msub	w0, w0, w2, w22
	str	w0, [x29,160]
	sub	w2, w2, w0
	cmp	w2, w23
	csel	w0, w2, w23, ls
	str	w0, [x29,184]
.L2927:
	add	x2, x20, :lo12:.LANCHOR0
	ldr	w1, [x29,184]
	ldrh	w0, [x2,2032]
	cmp	w1, w0
	bne	.L2928
	ldr	w0, [x29,176]
	mov	x1, x24
	cbnz	w0, .L2929
	ldr	w0, [x29,184]
	mul	w1, w0, w26
	sub	w1, w1, w22
	ubfiz	x1, x1, 9, 23
	add	x1, x24, x1
.L2929:
	ldr	w2, [x29,204]
	mov	w0, 56
	add	x4, x20, :lo12:.LANCHOR0
	cbz	w2, .L2930
	ldr	w2, [x4,2088]
	umull	x0, w2, w0
	ldr	x2, [x4,2096]
	add	x0, x2, x0
	b	.L2974
.L2930:
	ldr	w2, [x4,2088]
	umull	x0, w2, w0
	ldr	x2, [x4,2096]
	add	x0, x2, x0
	ldr	x0, [x0,8]
	b	.L2972
.L2928:
	ldr	w0, [x29,212]
	cmn	w0, #1
	beq	.L2932
	str	w0, [x29,220]
	mov	w1, 56
	ldr	w0, [x2,2088]
	str	w26, [x29,240]
	umull	x0, w0, w1
	ldr	x1, [x2,2096]
	mov	w2, 0
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
	bne	.L2933
	ldr	w0, [x21,424]
	add	w0, w0, 1
	str	w0, [x21,424]
	b	.L2935
.L2933:
	ldr	w0, [x28,8]
	cmp	w0, w26
	beq	.L2935
	ldr	w0, [x21,424]
	mov	w2, w26
	add	w0, w0, 1
	str	w0, [x21,424]
	ldr	x0, [x29,104]
	ldr	w1, [x28,8]
	bl	printk
	b	.L2935
.L2932:
	ldr	w0, [x2,2088]
	mov	w1, 56
	umull	x0, w0, w1
	ldr	x1, [x2,2096]
	ldrh	w2, [x2,2036]
	add	x0, x1, x0
	mov	w1, 0
	ldr	x0, [x0,8]
	bl	ftl_memset
.L2935:
	ldr	w1, [x29,184]
	mov	w0, 56
	lsl	w2, w1, 9
	ldr	w1, [x29,176]
	cbz	w1, .L2936
	add	x1, x20, :lo12:.LANCHOR0
	ldr	w4, [x1,2088]
	umull	x0, w4, w0
	ldr	x4, [x1,2096]
	add	x4, x4, x0
	ldr	x0, [x29,160]
	ubfiz	x0, x0, 9, 23
	ldr	x1, [x4,8]
	add	x0, x1, x0
	mov	x1, x24
	b	.L2973
.L2936:
	add	x5, x20, :lo12:.LANCHOR0
	ldr	w4, [x5,2088]
	ldr	x1, [x5,2096]
	umull	x0, w4, w0
	add	x0, x1, x0
	ldrh	w1, [x5,2032]
	mul	w1, w26, w1
	ldr	x0, [x0,8]
	sub	w1, w1, w22
	ubfiz	x1, x1, 9, 23
	add	x1, x24, x1
	b	.L2973
.L2924:
	ldr	w0, [x29,204]
	ldrh	w1, [x4,2032]
	ldr	w2, [x4,2088]
	cbz	w0, .L2937
	mul	w1, w26, w1
	umull	x0, w2, w5
	sub	w1, w1, w22
	ldr	x2, [x4,2096]
	ubfiz	x1, x1, 9, 23
	add	x0, x2, x0
	add	x1, x24, x1
.L2974:
	str	x1, [x0,8]
	b	.L2931
.L2937:
	umull	x0, w2, w5
	ldr	x2, [x4,2096]
	mul	w1, w26, w1
	add	x0, x2, x0
	sub	w1, w1, w22
	ubfiz	x1, x1, 9, 23
	ldr	x0, [x0,8]
	add	x1, x24, x1
.L2972:
	ldrh	w2, [x4,2036]
.L2973:
	bl	ftl_memcpy
.L2931:
	ldr	x1, [x29,128]
	mov	w0, -3947
	ldr	x2, [x29,136]
	strh	w0, [x1,x2]
	add	x1, x20, :lo12:.LANCHOR0
	str	w26, [x28,8]
	ldr	w0, [x21,200]
	str	w0, [x28,4]
	add	w0, w0, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x21,200]
	ldr	w0, [x29,212]
	str	w0, [x28,12]
	ldrh	w0, [x19]
	strh	w0, [x28,2]
	ldr	w0, [x1,2088]
	add	w0, w0, 1
	str	w0, [x1,2088]
	ldr	x0, [x29,192]
	add	x0, x0, 1
	str	x0, [x29,192]
	b	.L2921
.L2976:
	str	w26, [x29,200]
	mov	x0, x2
	b	.L2922
.L2952:
	str	w26, [x29,200]
.L2922:
	sub	w27, w27, w0
	ldr	w0, [x29,204]
	cbnz	w0, .L2941
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w1, [x0,2088]
	ldr	w0, [x21,2040]
	cmp	w1, w0
	bcs	.L2941
	ldrh	w0, [x19,4]
	cbz	w0, .L2941
.L2943:
	str	wzr, [x29,204]
	b	.L2914
.L2941:
	bl	FtlCacheWriteBack
	add	x0, x20, :lo12:.LANCHOR0
	cmp	w27, 3
	str	wzr, [x0,2088]
	bls	.L2943
	b	.L2914
.L2975:
	ldr	w1, [x29,180]
	mov	w0, w27
	sub	w1, w1, w25
	bl	rk_ftl_garbage_collect
	ldr	x0, [x29,120]
	add	x19, x0, :lo12:.LANCHOR2
	ldrh	w0, [x19,-8]
	cmp	w0, 15
	bhi	.L2945
	mov	w21, 65535
	mov	w20, 128
.L2971:
	ldrh	w0, [x19,296]
	cmp	w0, w21
	bne	.L2946
	ldrh	w0, [x19,952]
	cmp	w0, w21
	bne	.L2946
	mov	w0, 0
	bl	List_get_gc_head_node
	bl	FtlGcRefreshBlock
.L2946:
	mov	w0, 1
	strh	w20, [x19,866]
	mov	w1, w0
	strh	w20, [x19,864]
	bl	rk_ftl_garbage_collect
	mov	w0, 0
	mov	w1, 1
	bl	rk_ftl_garbage_collect
	ldrh	w0, [x19,-8]
	cmp	w0, 8
	bls	.L2971
	b	.L2945
.L2908:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 272
	ret
	.size	FtlWrite, .-FtlWrite
	.align	2
	.global	flash_boot_enter_slc_mode
	.type	flash_boot_enter_slc_mode, %function
flash_boot_enter_slc_mode:
	adrp	x1, .LANCHOR0+1876
	uxtb	w0, w0
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w2, [x1,#:lo12:.LANCHOR0+1876]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	bne	.L2977
	bl	flash_enter_slc_mode
.L2977:
	ldp	x29, x30, [sp], 16
	ret
	.size	flash_boot_enter_slc_mode, .-flash_boot_enter_slc_mode
	.align	2
	.global	flash_boot_exit_slc_mode
	.type	flash_boot_exit_slc_mode, %function
flash_boot_exit_slc_mode:
	adrp	x1, .LANCHOR0+1876
	uxtb	w0, w0
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w2, [x1,#:lo12:.LANCHOR0+1876]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	bne	.L2979
	bl	flash_exit_slc_mode
.L2979:
	ldp	x29, x30, [sp], 16
	ret
	.size	flash_boot_exit_slc_mode, .-flash_boot_exit_slc_mode
	.align	2
	.global	IdBlockReadData
	.type	IdBlockReadData, %function
IdBlockReadData:
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	mov	x20, x2
	add	x19, x19, :lo12:.LANCHOR0
	stp	x21, x22, [sp,32]
	mov	w22, w0
	ldr	x0, [x19,744]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	ldr	w3, [x19,12]
	mov	w26, w1
	ldrb	w21, [x0,9]
	adrp	x0, .LC131
	add	x0, x0, :lo12:.LC131
	mov	w1, w22
	mov	w2, w26
	mov	w23, 0
	mul	w21, w3, w21
	bl	printk
	ldr	x0, [x19,744]
	mov	w6, 4
	uxth	w21, w21
	adrp	x7, .LANCHOR4
	ldrb	w28, [x0,9]
	udiv	w0, w22, w21
	msub	w21, w0, w21, w22
	sub	w5, w22, w21
	mul	w3, w21, w28
	ubfx	x3, x3, 2, 2
.L2982:
	cmp	w23, w26
	bcs	.L2988
	ldrb	w1, [x19,120]
	sub	w24, w6, w3
	add	w4, w23, w21
	lsr	w4, w4, 2
	uxth	w24, w24
	add	x0, x19, x4, uxth 1
	ldrh	w0, [x0,196]
	cbz	w1, .L2983
	ldr	w2, [x19,1876]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	uxth	w4, w4
	cmp	w2, w1
	csel	w0, w4, w0, eq
.L2983:
	add	w3, w3, w5
	add	x27, x7, :lo12:.LANCHOR4
	ldrb	w4, [x19,1944]
	madd	w25, w0, w28, w3
	ldr	x0, [x19,744]
	ldr	x2, [x27,80]
	str	x6, [x29,96]
	ldrb	w1, [x0,9]
	ldrb	w0, [x27,74]
	str	x5, [x29,104]
	str	x7, [x29,112]
	str	x4, [x29,136]
	str	x2, [x29,120]
	str	x1, [x29,128]
	bl	FlashBchSel
	mov	w0, 0
	bl	flash_boot_enter_slc_mode
	ldr	x1, [x29,128]
	mov	x3, 0
	ldr	x2, [x29,120]
	mov	w0, 0
	udiv	w1, w25, w1
	bl	FlashReadPage
	mov	w0, 0
	bl	flash_boot_exit_slc_mode
	ldr	x4, [x29,136]
	mov	w0, w4
	bl	FlashBchSel
	ubfiz	x0, x23, 9, 16
	ldr	x1, [x27,80]
	add	x0, x20, x0
	mov	w2, 2048
	add	w23, w24, w23
	bl	ftl_memcpy
	uxth	w23, w23
	mov	w3, 0
	ldr	x6, [x29,96]
	ldr	x5, [x29,104]
	ldr	x7, [x29,112]
	b	.L2982
.L2988:
	adrp	x0, .LC132
	mov	w1, w22
	mov	w2, w26
	mov	w3, 0
	add	x0, x0, :lo12:.LC132
	bl	printk
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
	.size	IdBlockReadData, .-IdBlockReadData
	.align	2
	.global	IDBlockWriteData
	.type	IDBlockWriteData, %function
IDBlockWriteData:
	stp	x29, x30, [sp, -192]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	add	x19, x19, :lo12:.LANCHOR0
	stp	x27, x28, [sp,80]
	mov	w27, w1
	adrp	x28, .LANCHOR4
	ldr	x1, [x19,744]
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	mov	w24, w0
	ldr	w0, [x19,12]
	ldrb	w21, [x1,9]
	mov	x22, x2
	mov	w1, w24
	mov	w2, w27
	mov	w23, 0
	mul	w21, w0, w21
	adrp	x0, .LC133
	add	x0, x0, :lo12:.LC133
	uxth	w21, w21
	bl	printk
	mov	w0, 0
	bl	flash_boot_enter_slc_mode
	ldrh	w1, [x19,188]
	mov	w2, 0
	mov	w0, 0
	udiv	w1, w24, w1
	bl	FlashEraseBlock
	mov	w0, 0
	bl	flash_boot_exit_slc_mode
	udiv	w26, w24, w21
	ldr	x0, [x19,744]
	ldrb	w20, [x0,9]
	msub	w26, w26, w21, w24
	sub	w21, w24, w26
.L2990:
	cmp	w23, w27
	bcs	.L3004
	add	w4, w23, w26
	lsr	w4, w4, 2
	uxth	w0, w4
	and	w4, w4, 65535
	cbz	w0, .L2991
	add	x1, x19, 196
	add	w2, w4, 1
	ldrh	w3, [x1,w2,sxtw 1]
	ldrb	w1, [x19,120]
	cbz	w1, .L2992
	ldr	w2, [x19,1876]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	bne	.L2992
	add	w3, w0, 1
	uxth	w3, w3
.L2992:
	sub	w3, w3, #1
	str	wzr, [x29,132]
	lsl	w3, w3, 2
	str	w3, [x29,128]
.L2991:
	add	x1, x19, 196
	ldrh	w25, [x1,w4,sxtw 1]
	ldrb	w1, [x19,120]
	cbz	w1, .L2993
	ldr	w2, [x19,1876]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	csel	w25, w25, w0, ne
.L2993:
	add	x0, x28, :lo12:.LANCHOR4
	ldrb	w4, [x19,1944]
	madd	w3, w25, w20, w21
	ubfiz	x2, x23, 9, 16
	str	x4, [x29,120]
	ldrb	w0, [x0,74]
	add	x2, x22, x2
	str	x3, [x29,104]
	add	w23, w23, 4
	str	x2, [x29,112]
	bl	FlashBchSel
	uxth	w23, w23
	mov	w0, 0
	bl	flash_boot_enter_slc_mode
	ldr	x0, [x19,744]
	ldr	x3, [x29,104]
	ldr	x2, [x29,112]
	ldrb	w1, [x0,9]
	mov	w0, 0
	udiv	w1, w3, w1
	add	x3, x29, 128
	bl	FlashProgPage
	mov	w0, 0
	bl	flash_boot_exit_slc_mode
	ldr	x4, [x29,120]
	mov	w0, w4
	bl	FlashBchSel
	udiv	w1, w21, w20
	add	w2, w25, 1
	mov	w0, 0
	bl	FlashPageProgMsbFFData
	b	.L2990
.L3004:
	adrp	x0, .LC134
	mov	w1, w24
	mov	w2, w27
	mov	w3, 0
	add	x0, x0, :lo12:.LC134
	bl	printk
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 192
	ret
	.size	IDBlockWriteData, .-IDBlockWriteData
	.align	2
	.global	write_idblock
	.type	write_idblock, %function
write_idblock:
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	adrp	x24, .LANCHOR0
	mov	x23, x2
	stp	x25, x26, [sp,64]
	mov	w25, w0
	add	x0, x24, :lo12:.LANCHOR0
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x27, x28, [sp,80]
	mov	x21, x1
	ldr	w26, [x0,12]
	ldr	x1, [x0,744]
	mov	w0, 59392
	movk	w0, 0x3, lsl 16
	ldrb	w22, [x1,9]
	bl	ftl_malloc
	str	x24, [x29,120]
	mov	x20, x0
	mov	w0, -1
	cbz	x20, .L3015
	add	w19, w25, 511
	lsr	w19, w19, 9
	cmp	w19, 255
	bhi	.L3007
	ubfiz	x0, x19, 9, 23
	mov	w2, 256
	add	x0, x21, x0
	mov	x1, x21
	sub	w2, w2, w19
	bl	memcpy
.L3007:
	add	w19, w19, 128
	mov	w0, 256
	cmp	w19, 256
	mov	x1, x23
	csel	w19, w19, w0, ls
	adrp	x0, .LC135
	add	x0, x0, :lo12:.LC135
	mov	w2, 4
	mov	w3, 5
	bl	rknand_print_hex
	ldr	x0, [x29,120]
	ldr	w1, [x21,512]
	add	x27, x0, :lo12:.LANCHOR0
	adrp	x0, .LC136
	add	x0, x0, :lo12:.LC136
	ldrb	w2, [x27,9]
	bl	printk
	ldrb	w0, [x27,9]
	ldr	w1, [x21,512]
	cmp	w1, w0
	bls	.L3008
	str	w0, [x21,512]
.L3008:
	adrp	x0, .LC137
	mov	w2, w25
	mul	w22, w22, w26
	add	x0, x0, :lo12:.LC137
	mov	w1, w19
	mov	x24, 0
	adrp	x26, .LC138
	bl	printk
	adrp	x27, .LC139
	adrp	x28, .LC140
	lsl	w0, w19, 7
	uxth	w22, w22
	str	w0, [x29,116]
	mov	w25, w24
	add	x26, x26, :lo12:.LC138
	add	x27, x27, :lo12:.LC139
	add	x28, x28, :lo12:.LC140
.L3013:
	ldr	x1, [x29,120]
	mov	w8, w24
	ldr	w0, [x23,x24,lsl 2]
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w1, [x1,194]
	cmp	w0, w1
	bcs	.L3009
	adrp	x1, .LANCHOR4+68
	ldr	w1, [x1,#:lo12:.LANCHOR4+68]
	cmp	w0, w1
	bcc	.L3009
	mov	w1, 0
	mov	x2, 512
	mov	x0, x20
	str	x8, [x29,104]
	bl	memset
	ldr	w0, [x23,x24,lsl 2]
	mov	w1, w19
	mov	x2, x21
	mul	w0, w22, w0
	bl	IDBlockWriteData
	ldr	w0, [x23,x24,lsl 2]
	mov	w1, w19
	mov	x2, x20
	mul	w0, w22, w0
	bl	IdBlockReadData
	ldr	x8, [x29,104]
	mov	x0, 0
.L3010:
	ldr	w1, [x29,116]
	mov	w3, w0
	cmp	w0, w1
	bcs	.L3020
	ldr	w4, [x20,x0,lsl 2]
	mov	x7, x0
	add	x0, x0, 1
	add	x1, x21, x0, lsl 2
	ldr	w5, [x1,-4]
	cmp	w4, w5
	beq	.L3010
	ldr	w2, [x23,x24,lsl 2]
	mov	w1, w8
	mov	x0, x26
	str	x7, [x29,104]
	bl	printk
	ldr	x7, [x29,104]
	mov	x0, x27
	mov	w2, 4
	mov	w3, 256
	and	x4, x7, -256
	lsl	x4, x4, 2
	str	x4, [x29,104]
	add	x1, x21, x4
	bl	rknand_print_hex
	ldr	x4, [x29,104]
	mov	w3, 256
	mov	x0, x28
	mov	w2, 4
	add	x1, x20, x4
	bl	rknand_print_hex
	mov	x0, x20
	mov	w1, 0
	mov	x2, 512
	bl	memset
	ldr	w0, [x23,x24,lsl 2]
	mov	w1, 4
	mov	x2, x20
	mul	w0, w22, w0
	bl	IDBlockWriteData
	adrp	x0, .LC141
	add	x0, x0, :lo12:.LC141
	bl	printk
	b	.L3009
.L3020:
	add	w25, w25, 1
.L3009:
	add	x24, x24, 1
	cmp	x24, 5
	bne	.L3013
	mov	x0, x20
	bl	ftl_free
	cmp	w25, wzr
	csetm	w0, eq
.L3015:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
	.size	write_idblock, .-write_idblock
	.align	2
	.global	CRC_32
	.type	CRC_32, %function
CRC_32:
	mov	x2, 0
	adrp	x3, .LANCHOR1
	mov	x5, x0
	add	x3, x3, :lo12:.LANCHOR1
	mov	w0, w2
.L3022:
	cmp	w1, w2
	bls	.L3024
	ldrb	w4, [x5,x2]
	add	x6, x3, 3016
	add	x2, x2, 1
	eor	w4, w4, w0, lsr 24
	ldr	w4, [x6,w4,uxtw 2]
	eor	w0, w4, w0, lsl 8
	b	.L3022
.L3024:
	ret
	.size	CRC_32, .-CRC_32
	.align	2
	.global	rknand_sys_storage_ioctl
	.type	rknand_sys_storage_ioctl, %function
rknand_sys_storage_ioctl:
	sub	sp, sp, #528
	mov	w0, 29187
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	mov	w19, w1
	mov	x20, x2
	beq	.L3027
	mov	w0, 29187
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L3028
	mov	w0, 25726
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3029
	mov	w0, 25726
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L3030
	mov	w0, 25601
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3031
	mov	w0, 25601
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L3032
	mov	w0, 25364
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bne	.L3089
	bl	rknand_dev_flush
	b	.L3121
.L3032:
	mov	w0, 25602
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3034
	mov	w0, 25603
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3035
	b	.L3089
.L3030:
	mov	w0, 27688
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3036
	mov	w0, 27688
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L3037
	mov	w0, 25727
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3038
	b	.L3089
.L3037:
	mov	w0, 27698
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3036
	mov	w0, 27708
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3036
	b	.L3089
.L3028:
	mov	w0, 29267
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3039
	mov	w0, 29267
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L3040
	mov	w0, 29189
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3041
	mov	w0, 29189
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bcc	.L3042
	mov	w0, 29210
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3043
	mov	w0, 29266
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3044
	b	.L3089
.L3040:
	mov	w0, 30225
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3045
	mov	w0, 30225
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L3046
	mov	w0, 29268
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3047
	mov	w0, 30224
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3048
	b	.L3089
.L3046:
	mov	w0, 30226
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3048
	mov	w0, 30227
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L3045
	b	.L3089
.L3042:
	adrp	x0, .LC142
	add	x0, x0, :lo12:.LC142
	bl	printk
	mov	w0, 4096
	bl	ftl_malloc
	mov	x19, x0
	cbnz	x0, .L3049
.L3054:
	mov	x0, -12
	b	.L3026
.L3049:
	mov	x1, x20
	mov	x2, 512
	bl	rk_copy_from_user
	cbnz	x0, .L3128
	adrp	x0, .LC144
	ldr	w1, [x19]
	ldr	w2, [x19,4]
	add	x0, x0, :lo12:.LC144
	bl	printk
	ldr	w0, [x19,4]
	str	w0, [x29,52]
	cmp	w0, 8
	bhi	.L3124
	bl	rknand_device_lock
	ldr	w1, [x29,52]
	mov	x2, x19
	ldr	w0, [x19]
	bl	IdBlockReadData
	bl	rknand_device_unlock
	ldr	w2, [x29,52]
	mov	x0, x20
	mov	x1, x19
	ubfiz	x2, x2, 9, 23
	bl	rk_copy_to_user
	cbz	x0, .L3130
	adrp	x0, .LC145
	add	x0, x0, :lo12:.LC145
.L3123:
	bl	printk
.L3124:
	mov	x0, x19
.L3125:
	bl	ftl_free
.L3126:
	mov	x0, -14
	b	.L3026
.L3041:
	adrp	x0, .LC146
	add	x0, x0, :lo12:.LC146
	bl	printk
	mov	w0, 4096
	bl	ftl_malloc
	mov	x19, x0
	cbz	x0, .L3054
	mov	x1, x20
	mov	x2, 4096
	bl	rk_copy_from_user
	cbnz	x0, .L3128
	ldr	w1, [x19]
	adrp	x0, .LC147
	ldr	w2, [x19,4]
	add	x0, x0, :lo12:.LC147
	bl	printk
	adrp	x0, .LANCHOR4
	add	x21, x0, :lo12:.LANCHOR4
	mov	x20, x0
	ldr	x1, [x21,448]
	cbnz	x1, .L3056
	mov	w0, 260096
	bl	ftl_malloc
	str	x0, [x21,448]
	cbz	x0, .L3124
.L3056:
	ldr	w2, [x19,4]
	cmp	w2, 4088
	bhi	.L3124
	ldr	w0, [x19]
	mov	w1, 55296
	movk	w1, 0x3, lsl 16
	cmp	w0, w1
	bhi	.L3124
	add	x20, x20, :lo12:.LANCHOR4
	uxtw	x2, w2
	ldr	x1, [x20,448]
	add	x0, x1, x0, uxtw
	add	x1, x19, 8
	bl	memcpy
.L3130:
	mov	x0, x19
	bl	ftl_free
	b	.L3121
.L3044:
	adrp	x0, .LC148
	add	x0, x0, :lo12:.LC148
	bl	printk
	mov	w0, 4096
	bl	ftl_malloc
	mov	x19, x0
	cbz	x0, .L3054
	mov	x1, x20
	mov	x2, 28
	bl	rk_copy_from_user
	cbz	x0, .L3058
.L3128:
	adrp	x0, .LC143
	add	x0, x0, :lo12:.LC143
	b	.L3123
.L3058:
	ldr	w1, [x19]
	adrp	x0, .LC149
	ldr	w2, [x19,4]
	add	x0, x0, :lo12:.LC149
	bl	printk
	ldr	w1, [x19]
	mov	w0, 59392
	movk	w0, 0x3, lsl 16
	cmp	w1, w0
	bhi	.L3124
	adrp	x20, .LANCHOR4
	add	x20, x20, :lo12:.LANCHOR4
	ldr	x0, [x20,448]
	cbz	x0, .L3124
	bl	CRC_32
	ldr	w1, [x19,4]
	cmp	w1, w0
	beq	.L3061
	mov	x0, x19
	bl	ftl_free
	mov	x0, -2
	b	.L3026
.L3061:
	bl	rknand_device_lock
	ldr	x1, [x20,448]
	add	x2, x19, 8
	ldr	w0, [x19]
	bl	write_idblock
	bl	rknand_device_unlock
	ldr	x0, [x20,448]
	bl	ftl_free
	str	xzr, [x20,448]
	b	.L3130
.L3043:
	adrp	x0, .LC150
	add	x0, x0, :lo12:.LC150
	bl	printk
	mov	w0, 4096
	bl	ftl_malloc
	mov	x21, x0
	cbz	x0, .L3054
	bl	ReadFlashInfo
	mov	x0, x20
	mov	x1, x21
	mov	x2, 11
	b	.L3132
.L3027:
	adrp	x0, .LC151
	add	x0, x0, :lo12:.LC151
	bl	printk
	bl	rknand_device_lock
	bl	FtlReInitForSDUpdata
	mov	w19, w0
	bl	rknand_device_unlock
	cbnz	w19, .L3126
	bl	nand_blk_add_whole_disk
	mov	w0, 4096
	bl	ftl_malloc
	mov	x21, x0
	cbz	x0, .L3054
	bl	rknand_device_lock
	mov	w1, w19
	mov	w2, 64
	mov	x0, x21
	bl	FlashReadFacBbtData
	bl	rknand_device_unlock
	adrp	x0, .LC152
	mov	x1, x21
	add	x0, x0, :lo12:.LC152
	mov	w2, 4
	mov	w3, 8
	bl	rknand_print_hex
	mov	x0, x20
	mov	x1, x21
	mov	x2, 64
	b	.L3132
.L3039:
	adrp	x0, .LC153
	add	x0, x0, :lo12:.LC153
	bl	printk
	adrp	x0, .LANCHOR4+456
	add	x1, x29, 52
	mov	x2, 4
	ldr	x0, [x0,#:lo12:.LANCHOR4+456]
	ldr	w0, [x0,20]
	str	w0, [x29,52]
	mov	x0, x20
	b	.L3117
.L3047:
	adrp	x0, .LC154
	add	x0, x0, :lo12:.LC154
	bl	printk
	mov	w0, 4096
	bl	ftl_malloc
	mov	x21, x0
	cbz	x0, .L3054
	bl	rknand_device_lock
	mov	w1, 2
	mov	x2, x21
	mov	w0, 520
	bl	FtlVendorPartRead
	bl	rknand_device_unlock
	mov	x0, x20
	mov	x1, x21
	mov	x2, 1024
.L3132:
	bl	rk_copy_to_user
	mov	x19, x0
	mov	x0, x21
	cbnz	x19, .L3125
	bl	ftl_free
	b	.L3053
.L3031:
	adrp	x0, .LC155
	add	x0, x0, :lo12:.LC155
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	mov	x19, x0
	cbz	x0, .L3067
.L3072:
	adrp	x0, .LC143
	add	x0, x0, :lo12:.LC143
	bl	printk
	b	.L3126
.L3067:
	ldr	w1, [x29,56]
	mov	w0, 21060
	movk	w0, 0x4b4d, lsl 16
	cmp	w1, w0
	beq	.L3068
.L3069:
	mov	x19, -1
	b	.L3053
.L3068:
	ldr	w0, [x29,60]
	cmp	w0, 512
	bhi	.L3069
	adrp	x21, .LANCHOR4
	add	x0, x29, 56
	add	x21, x21, :lo12:.LANCHOR4
	mov	x2, 512
	ldr	x1, [x21,456]
	bl	memcpy
	ldr	w1, [x21,464]
	mov	w0, 5161
	movk	w0, 0xc059, lsl 16
	cmp	w1, w0
	beq	.L3070
	add	x0, x29, 120
	mov	w1, w19
	mov	x2, 128
	str	w19, [x29,64]
	str	w19, [x29,68]
	bl	memset
.L3070:
	add	x0, x29, 312
	mov	w1, 0
	mov	x2, 256
	str	wzr, [x29,72]
	bl	memset
.L3118:
	mov	x0, x20
	add	x1, x29, 56
	mov	x2, 520
.L3117:
	bl	rk_copy_to_user
	cbnz	x0, .L3126
	b	.L3121
.L3034:
	adrp	x0, .LC156
	add	x0, x0, :lo12:.LC156
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L3072
	ldr	w1, [x29,56]
	mov	w0, 21060
	movk	w0, 0x4b4d, lsl 16
	cmp	w1, w0
	bne	.L3069
	ldr	w0, [x29,60]
	cmp	w0, 512
	bhi	.L3069
	adrp	x1, .LANCHOR4
	mov	w2, 5161
	add	x1, x1, :lo12:.LANCHOR4
	movk	w2, 0xc059, lsl 16
	mov	x0, -2
	ldr	w3, [x1,464]
	cmp	w3, w2
	bne	.L3026
	ldr	w2, [x29,68]
	mov	x0, -3
	sub	w3, w2, #1
	cmp	w3, 127
	bhi	.L3026
	ldr	x19, [x1,456]
	add	x1, x29, 120
	add	x0, x19, 64
	str	w2, [x19,12]
	ldr	w2, [x29,68]
	bl	memcpy
	mov	w0, 1
	mov	x1, x19
	b	.L3122
.L3038:
	adrp	x0, .LC157
	add	x0, x0, :lo12:.LC157
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L3072
	ldr	w1, [x29,56]
	mov	w0, 17476
	movk	w0, 0x4253, lsl 16
	cmp	w1, w0
	bne	.L3069
	ldr	w0, [x29,60]
	cmp	w0, 512
	bhi	.L3069
	adrp	x19, .LANCHOR4
	add	x0, x19, :lo12:.LANCHOR4
	ldr	w1, [x0,468]
	cbnz	w1, .L3073
.L3076:
	mov	x0, 0
	b	.L3026
.L3073:
	ldr	x1, [x0,472]
	mov	w2, 22867
	movk	w2, 0x4453, lsl 16
	ldr	w3, [x1]
	cmp	w3, w2
	beq	.L3074
	mov	w2, 22867
	movk	w2, 0x4453, lsl 16
	str	w2, [x1]
	mov	w2, 504
	ldr	x1, [x0,472]
	str	w2, [x1,4]
	ldr	x0, [x0,472]
	str	wzr, [x0,8]
	str	wzr, [x0,12]
.L3074:
	add	x20, x19, :lo12:.LANCHOR4
	mov	w0, 0
	ldr	x1, [x20,472]
	str	wzr, [x1,16]
	bl	StorageSysDataStore
	ldr	x0, [x20,456]
	mov	w1, 21060
	movk	w1, 0x4b4d, lsl 16
	ldr	w2, [x0]
	cmp	w2, w1
	beq	.L3075
	mov	w1, 21060
	movk	w1, 0x4b4d, lsl 16
	str	w1, [x0]
	mov	w1, 504
	ldr	x0, [x20,456]
	str	w1, [x0,4]
	ldr	x0, [x20,456]
	str	wzr, [x0,8]
.L3075:
	add	x19, x19, :lo12:.LANCHOR4
	mov	w1, 0
	mov	x2, 128
	ldr	x20, [x19,456]
	add	x0, x20, 64
	str	wzr, [x20,12]
	bl	memset
	mov	w0, 1
	mov	x1, x20
	bl	StorageSysDataStore
	str	wzr, [x19,468]
	str	wzr, [x19,464]
	b	.L3121
.L3029:
	adrp	x0, .LC158
	add	x0, x0, :lo12:.LC158
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L3072
	ldr	w2, [x29,56]
	mov	w1, 20037
	movk	w1, 0x4253, lsl 16
	cmp	w2, w1
	bne	.L3069
	ldr	w1, [x29,60]
	cmp	w1, 512
	bhi	.L3069
	adrp	x19, .LANCHOR4
	add	x1, x19, :lo12:.LANCHOR4
	ldr	w2, [x1,468]
	cmp	w2, 1
	beq	.L3076
	ldr	x2, [x1,472]
	mov	w3, 22867
	movk	w3, 0x4453, lsl 16
	ldr	w4, [x2]
	cmp	w4, w3
	beq	.L3077
	mov	w3, 22867
	movk	w3, 0x4453, lsl 16
	str	w3, [x2]
	mov	w3, 504
	ldr	x2, [x1,472]
	str	w3, [x2,4]
	ldr	x1, [x1,472]
	str	w0, [x1,8]
	str	w0, [x1,12]
.L3077:
	add	x20, x19, :lo12:.LANCHOR4
	mov	w0, 1
	ldr	x1, [x20,472]
	str	w0, [x1,16]
	mov	w0, 0
	bl	StorageSysDataStore
	ldr	x0, [x20,456]
	mov	w1, 21060
	movk	w1, 0x4b4d, lsl 16
	ldr	w2, [x0]
	cmp	w2, w1
	beq	.L3078
	mov	w1, 21060
	movk	w1, 0x4b4d, lsl 16
	str	w1, [x0]
	mov	w1, 504
	ldr	x0, [x20,456]
	str	w1, [x0,4]
	ldr	x0, [x20,456]
	str	wzr, [x0,8]
.L3078:
	add	x19, x19, :lo12:.LANCHOR4
	mov	w1, 0
	mov	x2, 128
	ldr	x20, [x19,456]
	add	x0, x20, 64
	str	wzr, [x20,12]
	bl	memset
	mov	w0, 1
	mov	x1, x20
	bl	StorageSysDataStore
	mov	w0, 1
	str	w0, [x19,468]
	b	.L3121
.L3035:
	adrp	x0, .LC159
	add	x0, x0, :lo12:.LC159
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L3072
	ldr	w1, [x29,56]
	mov	w0, 20051
	movk	w0, 0x4144, lsl 16
	cmp	w1, w0
	bne	.L3069
	ldr	w2, [x29,60]
	cmp	w2, 512
	bhi	.L3069
	adrp	x1, .LANCHOR4
	add	x0, x29, 64
	add	x1, x1, :lo12:.LANCHOR4
	uxtw	x2, w2
	add	x1, x1, 480
	b	.L3119
.L3036:
	mov	w0, 27698
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	bne	.L3079
	adrp	x0, .LC160
	add	x0, x0, :lo12:.LC160
	b	.L3120
.L3079:
	mov	w0, 27708
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	bne	.L3081
	adrp	x0, .LC161
	add	x0, x0, :lo12:.LC161
	b	.L3120
.L3081:
	adrp	x0, .LC162
	add	x0, x0, :lo12:.LC162
.L3120:
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L3072
	ldr	w1, [x29,56]
	mov	w0, 17227
	movk	w0, 0x4c4f, lsl 16
	cmp	w1, w0
	bne	.L3126
	mov	w0, 27708
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	adrp	x0, .LANCHOR4
	bne	.L3082
	add	x0, x0, :lo12:.LANCHOR4
	add	x1, x29, 56
	mov	x2, 16
	ldr	x0, [x0,456]
	ldr	w0, [x0,20]
	str	w0, [x29,60]
	strb	w0, [x29,64]
	mov	x0, x20
	bl	rk_copy_to_user
	cbz	x0, .L3026
	b	.L3126
.L3082:
	add	x20, x0, :lo12:.LANCHOR4
	ldr	w1, [x20,992]
	cmp	w1, 10
	bhi	.L3126
	ldr	x1, [x20,456]
	ldr	w2, [x29,60]
	ldr	w3, [x1,24]
	cmp	w3, w2
	beq	.L3083
	cbz	w3, .L3083
	adrp	x0, .LC163
	mov	w1, w2
	add	x0, x0, :lo12:.LC163
	bl	printk
	ldr	w0, [x20,992]
	add	w0, w0, 1
	str	w0, [x20,992]
	b	.L3126
.L3083:
	add	x0, x0, :lo12:.LANCHOR4
	str	wzr, [x0,992]
	mov	w0, 27698
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	bne	.L3084
	str	wzr, [x1,20]
	str	wzr, [x1,24]
	b	.L3085
.L3084:
	mov	w0, 1
	str	w2, [x1,24]
	str	w0, [x1,20]
.L3085:
	mov	w0, 1
	mov	x19, -2
	bl	StorageSysDataStore
	cmn	w0, #1
	bne	.L3121
	b	.L3053
.L3048:
	adrp	x0, .LC164
	add	x0, x0, :lo12:.LC164
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L3072
	ldr	w1, [x29,56]
	mov	w0, 17750
	movk	w0, 0x444e, lsl 16
	cmp	w1, w0
	bne	.L3069
	ldr	w2, [x29,60]
	cmp	w2, 504
	bhi	.L3069
	mov	w0, 30224
	adrp	x1, .LANCHOR4
	movk	w0, 0x4004, lsl 16
	uxtw	x2, w2
	cmp	w19, w0
	add	x1, x1, :lo12:.LANCHOR4
	add	x0, x29, 64
	bne	.L3086
	ldr	x1, [x1,1000]
	b	.L3129
.L3086:
	ldr	x1, [x1,1008]
.L3129:
	add	x1, x1, 8
.L3119:
	bl	memcpy
	b	.L3118
.L3045:
	adrp	x0, .LC165
	add	x0, x0, :lo12:.LC165
	bl	printk
	add	x0, x29, 56
	mov	x1, x20
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L3072
	ldr	w1, [x29,56]
	mov	w0, 17750
	movk	w0, 0x444e, lsl 16
	cmp	w1, w0
	bne	.L3069
	ldr	w2, [x29,60]
	cmp	w2, 504
	bhi	.L3069
	mov	w0, 30225
	add	w2, w2, 8
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	adrp	x19, .LANCHOR4
	add	x19, x19, :lo12:.LANCHOR4
	bne	.L3088
	ldr	x0, [x19,1000]
	add	x1, x29, 56
	bl	memcpy
	mov	w0, 2
	ldr	x1, [x19,1000]
	b	.L3122
.L3088:
	ldr	x0, [x19,1008]
	add	x1, x29, 56
	bl	memcpy
	ldr	x1, [x19,1008]
	mov	w0, 3
.L3122:
	bl	StorageSysDataStore
	uxtw	x19, w0
	b	.L3053
.L3121:
	mov	x19, 0
.L3053:
	adrp	x0, .LC166
	mov	x1, x19
	add	x0, x0, :lo12:.LC166
	bl	printk
	mov	x0, x19
	b	.L3026
.L3089:
	mov	x0, -22
.L3026:
	ldp	x19, x20, [sp,16]
	ldr	x21, [sp,32]
	ldp	x29, x30, [sp], 48
	add	sp, sp, 528
	ret
	.size	rknand_sys_storage_ioctl, .-rknand_sys_storage_ioctl
	.align	2
	.global	rk_ftl_storage_sys_init
	.type	rk_ftl_storage_sys_init, %function
rk_ftl_storage_sys_init:
	stp	x29, x30, [sp, -48]!
	mov	w2, 512
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR4
	add	x19, x20, :lo12:.LANCHOR4
	stp	x21, x22, [sp,32]
	ldr	x1, [x19,424]
	str	x1, [x19,472]
	add	x0, x1, 512
	str	x0, [x19,456]
	add	x0, x1, 1024
	add	x1, x1, 1536
	str	x0, [x19,1000]
	add	x0, x19, 480
	str	xzr, [x19,448]
	str	x1, [x19,1008]
	bl	ftl_memcpy
	ldr	x21, [x19,472]
	str	wzr, [x19,464]
	str	wzr, [x19,992]
	ldr	w0, [x21,16]
	ldr	w22, [x21,508]
	str	w0, [x19,468]
	cbz	w22, .L3135
	mov	x0, x21
	mov	w1, 508
	bl	JSHash
	cmp	w22, w0
	beq	.L3135
	adrp	x0, .LC167
	str	wzr, [x21,16]
	add	x0, x0, :lo12:.LC167
	str	wzr, [x19,468]
	bl	printk
.L3135:
	add	x0, x20, :lo12:.LANCHOR4
	ldr	w1, [x0,468]
	cbz	w1, .L3136
	mov	w1, 5161
	movk	w1, 0xc059, lsl 16
	str	w1, [x0,464]
.L3136:
	add	x20, x20, :lo12:.LANCHOR4
	mov	w0, 2
	ldr	x1, [x20,1000]
	bl	StorageSysDataLoad
	ldr	x1, [x20,1008]
	mov	w0, 3
	bl	StorageSysDataLoad
	bl	rknand_sys_storage_init
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	rk_ftl_storage_sys_init, .-rk_ftl_storage_sys_init
	.align	2
	.global	StorageSysDataDeInit
	.type	StorageSysDataDeInit, %function
StorageSysDataDeInit:
	mov	w0, 0
	ret
	.size	StorageSysDataDeInit, .-StorageSysDataDeInit
	.align	2
	.global	rk_ftl_vendor_storage_init
	.type	rk_ftl_vendor_storage_init, %function
rk_ftl_vendor_storage_init:
	stp	x29, x30, [sp, -80]!
	mov	w1, 193
	mov	x0, 65536
	add	x29, sp, 0
	movk	w1, 0x240, lsl 16
	mov	w2, 4
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	adrp	x21, .LANCHOR4
	bl	kmalloc_order_trace
	add	x1, x21, :lo12:.LANCHOR4
	mov	w19, -12
	str	x0, [x1,1016]
	cbz	x0, .L3145
	mov	w22, 0
	adrp	x24, .LC168
	mov	w19, w22
	mov	w23, w22
	add	x24, x24, :lo12:.LC168
.L3149:
	add	x20, x21, :lo12:.LANCHOR4
	ldr	x2, [x20,1016]
	str	x2, [x29,72]
	bl	rknand_device_lock
	ldr	x2, [x29,72]
	add	w0, w23, 2
	mov	w1, 128
	lsl	w0, w0, 7
	bl	FtlVendorPartRead
	str	x0, [x29,72]
	bl	rknand_device_unlock
	ldr	x0, [x29,72]
	cbnz	w0, .L3146
	ldr	x3, [x20,1016]
	mov	x0, x24
	add	x2, x3, 61440
	ldr	w1, [x3]
	ldr	w3, [x3,4]
	ldr	w2, [x2,4092]
	bl	printk
	ldr	x20, [x20,1016]
	mov	w0, 22084
	movk	w0, 0x524b, lsl 16
	ldr	w1, [x20]
	cmp	w1, w0
	bne	.L3147
	ldr	w0, [x20,4]
	cmp	w19, w0
	bcs	.L3147
	add	x1, x20, 61440
	ldr	w1, [x1,4092]
	cmp	w1, w0
	csel	w22, w22, w23, ne
	csel	w19, w19, w0, ne
.L3147:
	cbnz	w23, .L3157
	mov	w23, 1
	b	.L3149
.L3157:
	cbz	w19, .L3150
	bl	rknand_device_lock
	lsl	w0, w22, 7
	mov	w1, 128
	add	w0, w0, 256
	mov	x2, x20
	bl	FtlVendorPartRead
	mov	w19, w0
	bl	rknand_device_unlock
	cbz	w19, .L3145
	b	.L3146
.L3150:
	mov	x0, x20
	mov	w1, w19
	mov	x2, 65536
	bl	memset
	str	w23, [x20,4]
	mov	w0, 22084
	movk	w0, 0x524b, lsl 16
	str	w0, [x20]
	add	x0, x20, 61440
	str	w23, [x0,4092]
	mov	w0, -1032
	strh	w19, [x20,12]
	strh	w0, [x20,14]
	b	.L3145
.L3146:
	add	x21, x21, :lo12:.LANCHOR4
	mov	w19, -1
	ldr	x0, [x21,1016]
	bl	kfree
	str	xzr, [x21,1016]
.L3145:
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	rk_ftl_vendor_storage_init, .-rk_ftl_vendor_storage_init
	.align	2
	.global	rk_ftl_vendor_read
	.type	rk_ftl_vendor_read, %function
rk_ftl_vendor_read:
	adrp	x3, .LANCHOR4+1016
	stp	x29, x30, [sp, -32]!
	add	x29, sp, 0
	ldr	x5, [x3,#:lo12:.LANCHOR4+1016]
	mov	w3, -1
	str	x19, [sp,16]
	cbz	x5, .L3159
	ldrh	w3, [x5,10]
	mov	x4, 0
.L3160:
	cmp	w4, w3
	mov	w6, w4
	bcs	.L3164
	add	x7, x5, x4, lsl 3
	add	x4, x4, 1
	ldrh	w7, [x7,16]
	cmp	w7, w0
	bne	.L3160
	add	x6, x5, x6, uxtw 3
	ldrh	w4, [x6,20]
	mov	x0, x1
	ldrh	w1, [x6,18]
	cmp	w2, w4
	csel	w19, w2, w4, ls
	add	x1, x1, 1024
	add	x1, x5, x1
	uxtw	x2, w19
	bl	memcpy
	mov	w3, w19
	b	.L3159
.L3164:
	mov	w3, -1
.L3159:
	mov	w0, w3
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	rk_ftl_vendor_read, .-rk_ftl_vendor_read
	.align	2
	.global	rk_ftl_vendor_write
	.type	rk_ftl_vendor_write, %function
rk_ftl_vendor_write:
	stp	x29, x30, [sp, -112]!
	mov	w6, w0
	adrp	x0, .LANCHOR4+1016
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	ldr	x19, [x0,#:lo12:.LANCHOR4+1016]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	w26, w2
	mov	x28, x1
	mov	w2, -1
	cbz	x19, .L3166
	add	w20, w26, 63
	ldrh	w3, [x19,10]
	ldrh	w23, [x19,8]
	and	w20, w20, -64
	mov	x2, 0
.L3167:
	cmp	w2, w3
	mov	w4, w2
	bcs	.L3182
	add	x0, x19, x2, lsl 3
	add	x2, x2, 1
	ldrh	w0, [x0,16]
	cmp	w0, w6
	bne	.L3167
	add	x21, x19, x4, uxtw 3
	ldrh	w24, [x21,20]
	add	x5, x19, 1024
	add	w24, w24, 63
	and	w24, w24, -64
	cmp	w26, w24
	bls	.L3168
	ldrh	w0, [x19,14]
	mov	w2, -1
	cmp	w0, w20
	bcc	.L3166
	ldrh	w21, [x21,18]
	add	w22, w3, w2
.L3169:
	cmp	w4, w22
	uxtw	x3, w4
	bcs	.L3183
	add	x3, x19, x3, lsl 3
	add	w25, w4, 1
	add	x1, x19, x25, uxtw 3
	ldrh	w0, [x1,16]
	str	x6, [x29,96]
	strh	w0, [x3,16]
	str	x5, [x29,104]
	ldrh	w0, [x1,20]
	strh	w0, [x3,20]
	strh	w21, [x3,18]
	add	x0, x5, x21, uxtw
	ldrh	w27, [x1,20]
	ldrh	w1, [x1,18]
	add	w27, w27, 63
	and	w27, w27, -64
	add	x1, x5, x1
	and	x2, x27, 131008
	add	w21, w21, w27
	bl	memcpy
	mov	w4, w25
	ldr	x6, [x29,96]
	ldr	x5, [x29,104]
	b	.L3169
.L3183:
	add	x3, x19, x3, lsl 3
	uxth	w21, w21
	mov	x1, x28
	uxtw	x2, w26
	add	x0, x5, x21, uxth
	uxth	w20, w20
	strh	w21, [x3,18]
	add	w21, w21, w20
	strh	w6, [x3,16]
	strh	w26, [x3,20]
	bl	memcpy
	strh	w21, [x19,12]
	ldrh	w0, [x19,14]
	add	w24, w24, w0
	sub	w24, w24, w20
	strh	w24, [x19,14]
	b	.L3181
.L3168:
	ldrh	w0, [x21,18]
	mov	x1, x28
	uxtw	x2, w26
	add	x0, x5, x0
	bl	memcpy
	strh	w26, [x21,20]
	b	.L3181
.L3182:
	ldrh	w4, [x19,14]
	mov	w2, -1
	cmp	w4, w20
	bcc	.L3166
	add	x3, x19, x3, uxth 3
	strh	w6, [x3,16]
	uxth	w20, w20
	mov	x1, x28
	ldrh	w0, [x19,12]
	uxtw	x2, w26
	strh	w0, [x3,18]
	strh	w26, [x3,20]
	add	w0, w20, w0
	sub	w20, w4, w20
	strh	w0, [x19,12]
	strh	w20, [x19,14]
	ldrh	w0, [x3,18]
	add	x0, x0, 1024
	add	x0, x19, x0
	bl	memcpy
	ldrh	w0, [x19,10]
	add	w0, w0, 1
	strh	w0, [x19,10]
.L3181:
	ldr	w0, [x19,4]
	add	x1, x19, 61440
	add	w0, w0, 1
	str	w0, [x19,4]
	str	w0, [x1,4092]
	ldrh	w0, [x19,8]
	add	w0, w0, 1
	uxth	w0, w0
	cmp	w0, 1
	csel	w0, w0, wzr, ls
	strh	w0, [x19,8]
	bl	rknand_device_lock
	lsl	w0, w23, 7
	mov	x2, x19
	mov	w1, 128
	add	w0, w0, 256
	bl	FtlVendorPartWrite
	bl	rknand_device_unlock
	mov	w2, 0
.L3166:
	mov	w0, w2
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 112
	ret
	.size	rk_ftl_vendor_write, .-rk_ftl_vendor_write
	.align	2
	.global	rk_ftl_vendor_storage_ioctl
	.type	rk_ftl_vendor_storage_ioctl, %function
rk_ftl_vendor_storage_ioctl:
	stp	x29, x30, [sp, -48]!
	adrp	x0, kmalloc_caches+96
	add	x29, sp, 0
	stp	x21, x22, [sp,32]
	ldr	x0, [x0,#:lo12:kmalloc_caches+96]
	mov	w22, w1
	mov	w1, 192
	mov	x21, x2
	movk	w1, 0x240, lsl 16
	mov	x2, 4096
	stp	x19, x20, [sp,16]
	bl	kmem_cache_alloc_trace
	mov	x20, -1
	mov	x19, x0
	cbz	x0, .L3185
	mov	w1, 30209
	movk	w1, 0x4004, lsl 16
	cmp	w22, w1
	beq	.L3187
	mov	w2, 30210
	movk	w2, 0x4004, lsl 16
	cmp	w22, w2
	beq	.L3188
	b	.L3207
.L3187:
#APP
// 86 "./arch/arm64/include/asm/thread_info.h" 1
	mrs x22, sp_el0
// 0 "" 2
#NO_APP
	mov	x1, x21
	ldr	x3, [x22,8]
#APP
// 393 "./arch/arm64/include/asm/uaccess.h" 1
	adds x1, x1, 8; ccmp x1, x3, #2, cc; cset x2, ls
// 0 "" 2
#NO_APP
	cbz	x2, .L3194
	mov	x1, x21
	mov	x2, 8
	bl	__arch_copy_from_user
	cbnz	x0, .L3207
	ldr	w1, [x19]
	mov	w0, 17745
	movk	w0, 0x5652, lsl 16
	cmp	w1, w0
	beq	.L3191
.L3192:
	mov	x20, -1
	b	.L3186
.L3191:
	ldrh	w0, [x19,4]
	add	x1, x19, 8
	ldrh	w2, [x19,6]
	bl	rk_ftl_vendor_read
	cmn	w0, #1
	beq	.L3192
	strh	w0, [x19,6]
	uxth	x20, w0
	add	x20, x20, 8
	mov	x0, x21
	ldr	x1, [x22,8]
#APP
// 403 "./arch/arm64/include/asm/uaccess.h" 1
	adds x0, x0, x20; ccmp x0, x1, #2, cc; cset x2, ls
// 0 "" 2
#NO_APP
	cbz	x2, .L3207
	mov	x1, x20
	mov	x0, x19
	mov	w2, 1
	bl	__check_object_size
	mov	x2, x20
	mov	x0, x21
	mov	x1, x19
	mov	x20, -14
	bl	__arch_copy_to_user
	cmp	x0, xzr
	csel	x20, xzr, x20, eq
	b	.L3186
.L3188:
#APP
// 86 "./arch/arm64/include/asm/thread_info.h" 1
	mrs x22, sp_el0
// 0 "" 2
#NO_APP
	mov	x1, x21
	ldr	x3, [x22,8]
#APP
// 393 "./arch/arm64/include/asm/uaccess.h" 1
	adds x1, x1, 8; ccmp x1, x3, #2, cc; cset x2, ls
// 0 "" 2
#NO_APP
	cbz	x2, .L3194
	mov	x2, 8
	mov	x1, x21
	bl	__arch_copy_from_user
	mov	x2, x0
	cbz	x0, .L3208
	b	.L3207
.L3194:
	str	x2, [x19]
	b	.L3207
.L3208:
	ldr	w1, [x19]
	mov	w0, 17745
	movk	w0, 0x5652, lsl 16
	cmp	w1, w0
	bne	.L3192
	ldrh	w20, [x19,6]
	cmp	w20, 4087
	bhi	.L3192
	add	w20, w20, 8
	ldr	x0, [x22,8]
	mov	x3, x21
	sxtw	x20, w20
#APP
// 393 "./arch/arm64/include/asm/uaccess.h" 1
	adds x3, x3, x20; ccmp x3, x0, #2, cc; cset x1, ls
// 0 "" 2
#NO_APP
	mov	x0, x19
	cbz	x1, .L3196
	mov	x1, x20
	bl	__check_object_size
	mov	x0, x19
	mov	x1, x21
	mov	x2, x20
	bl	__arch_copy_from_user
	cbnz	x0, .L3207
	ldrh	w0, [x19,4]
	add	x1, x19, 8
	ldrh	w2, [x19,6]
	bl	rk_ftl_vendor_write
	sxtw	x20, w0
	b	.L3186
.L3196:
	mov	x2, x20
	bl	memset
.L3207:
	mov	x20, -14
.L3186:
	mov	x0, x19
	bl	kfree
.L3185:
	mov	x0, x20
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
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
	.align	3
.LANCHOR3 = . + 0
	.type	__func__.19259, %object
	.size	__func__.19259, 11
__func__.19259:
	.string	"FtlMemInit"
	.zero	5
	.type	__func__.20152, %object
	.size	__func__.20152, 21
__func__.20152:
	.string	"FtlVpcCheckAndModify"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.string	"FlashEraseBlocks pageAddr error %x\n"
.LC1:
	.string	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\n"
.LC2:
	.string	"FLASH INFO:\n"
.LC3:
	.string	"FLASH ID: %x\n"
.LC4:
	.string	"Device Capacity: %d MB\n"
.LC5:
	.string	"FMWAIT: %x %x %x %x\n"
.LC6:
	.string	"FTL INFO:\n"
.LC7:
	.string	"g_MaxLpn = 0x%x\n"
.LC8:
	.string	"g_VaildLpn = 0x%x\n"
.LC9:
	.string	"read_page_count = 0x%x\n"
.LC10:
	.string	"discard_page_count = 0x%x\n"
.LC11:
	.string	"write_page_count = 0x%x\n"
.LC12:
	.string	"cache_write_count = 0x%x\n"
.LC13:
	.string	"l2p_write_count = 0x%x\n"
.LC14:
	.string	"gc_page_count = 0x%x\n"
.LC15:
	.string	"totle_write = %d MB\n"
.LC16:
	.string	"totle_read = %d MB\n"
.LC17:
	.string	"GSV = 0x%x\n"
.LC18:
	.string	"GDV = 0x%x\n"
.LC19:
	.string	"bad blk num = %d %d\n"
.LC20:
	.string	"free_superblocks = 0x%x\n"
.LC21:
	.string	"mlc_EC = 0x%x\n"
.LC22:
	.string	"slc_EC = 0x%x\n"
.LC23:
	.string	"avg_EC = 0x%x\n"
.LC24:
	.string	"sys_EC = 0x%x\n"
.LC25:
	.string	"max_EC = 0x%x\n"
.LC26:
	.string	"min_EC = 0x%x\n"
.LC27:
	.string	"PLT = 0x%x\n"
.LC28:
	.string	"POT = 0x%x\n"
.LC29:
	.string	"MaxSector = 0x%x\n"
.LC30:
	.string	"init_sys_blks_pp = 0x%x\n"
.LC31:
	.string	"sys_blks_pp = 0x%x\n"
.LC32:
	.string	"free sysblock = 0x%x\n"
.LC33:
	.string	"data_blks_pp = 0x%x\n"
.LC34:
	.string	"data_op_blks_pp = 0x%x\n"
.LC35:
	.string	"max_data_blks = 0x%x\n"
.LC36:
	.string	"Sys.id = 0x%x\n"
.LC37:
	.string	"Bbt.id = 0x%x\n"
.LC38:
	.string	"ACT.page = 0x%x\n"
.LC39:
	.string	"ACT.plane = 0x%x\n"
.LC40:
	.string	"ACT.id = 0x%x\n"
.LC41:
	.string	"ACT.mode = 0x%x\n"
.LC42:
	.string	"ACT.a_pages = 0x%x\n"
.LC43:
	.string	"ACT VPC = 0x%x\n"
.LC44:
	.string	"BUF.page = 0x%x\n"
.LC45:
	.string	"BUF.plane = 0x%x\n"
.LC46:
	.string	"BUF.id = 0x%x\n"
.LC47:
	.string	"BUF.mode = 0x%x\n"
.LC48:
	.string	"BUF.a_pages = 0x%x\n"
.LC49:
	.string	"BUF VPC = 0x%x\n"
.LC50:
	.string	"TMP.page = 0x%x\n"
.LC51:
	.string	"TMP.plane = 0x%x\n"
.LC52:
	.string	"TMP.id = 0x%x\n"
.LC53:
	.string	"TMP.mode = 0x%x\n"
.LC54:
	.string	"TMP.a_pages = 0x%x\n"
.LC55:
	.string	"GC.page = 0x%x\n"
.LC56:
	.string	"GC.plane = 0x%x\n"
.LC57:
	.string	"GC.id = 0x%x\n"
.LC58:
	.string	"GC.mode = 0x%x\n"
.LC59:
	.string	"GC.a_pages = 0x%x\n"
.LC60:
	.string	"WR_CHK = 0x%x %x %x %x\n"
.LC61:
	.string	"Read Err Cnt = 0x%x\n"
.LC62:
	.string	"Prog Err Cnt = 0x%x\n"
.LC63:
	.string	"gc_free_blk_th= 0x%x\n"
.LC64:
	.string	"gc_merge_free_blk_th= 0x%x\n"
.LC65:
	.string	"gc_skip_write_count= 0x%x\n"
.LC66:
	.string	"gc_blk_index= 0x%x\n"
.LC67:
	.string	"free min EC= 0x%x\n"
.LC68:
	.string	"free max EC= 0x%x\n"
.LC69:
	.string	"GC__SB VPC = 0x%x\n"
.LC70:
	.string	"%d. [0x%x]=0x%x 0x%x  0x%x\n"
.LC71:
	.string	"free %d. [0x%x] 0x%x  0x%x\n"
.LC72:
	.string	"%s\n"
.LC73:
	.string	"FTL version: 5.0.40 20170816"
.LC74:
	.string	"GetSwlReplaceBlock min_ec_id =%x %x\n"
.LC75:
	.string	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x\n"
.LC76:
	.string	"FtlGcRefreshBlock  0x%x\n"
.LC77:
	.string	"FtlGcMarkBadPhyBlk %d 0x%x\n"
.LC78:
	.string	"%s error allocating memory. return -1\n"
.LC79:
	.string	"%s 0x%x:"
.LC80:
	.string	"%x "
.LC81:
	.string	""
.LC82:
	.string	"%d statReg->V6.mtrans_cnt=%d flReg.V6.page_num=%d\n"
.LC83:
	.string	"nandc:"
.LC84:
	.string	"%d flReg.d32=%x %x\n"
.LC85:
	.string	"sdr read ok %x ecc=%d\n"
.LC86:
	.string	"sync para %d\n"
.LC87:
	.string	"TOG mode Read error %x %x\n"
.LC88:
	.string	"read retry status %x %x %x\n"
.LC89:
	.string	"micron RR %d row=%x,count %d,status=%d\n"
.LC90:
	.string	"ECC:%d\n"
.LC91:
	.string	"No.%d FLASH ID:%x %x %x %x %x %x\n"
.LC92:
	.string	"FlashLoadPhyInfo fail %x!!\n"
.LC93:
	.string	"Read pageadd=%x  ecc=%x err=%x\n"
.LC94:
	.string	"data:"
.LC95:
	.string	"spare:"
.LC96:
	.string	"ReadRetry pageadd=%x ecc=%x err=%x\n"
.LC97:
	.string	"FLFB:%d %d\n"
.LC98:
	.string	"prog error: = %x\n"
.LC99:
	.string	"prog read error: = %x\n"
.LC100:
	.string	"prog read s error: = %x %x %x\n"
.LC101:
	.string	"prog read d error: = %x %x %x\n"
.LC102:
	.string	"no ect"
.LC103:
	.string	"slc mode"
.LC104:
	.string	"BBT:"
.LC105:
	.string	"s raw s:"
.LC106:
	.string	"s read s:"
.LC107:
	.string	"d raw d:"
.LC108:
	.string	"d read d:"
.LC109:
	.string	"FlashMakeFactorBbt %d\n"
.LC110:
	.string	"bad block:%d %d\n"
.LC111:
	.string	"FMFB:%d %d\n"
.LC112:
	.string	"E:bad block:%d\n"
.LC113:
	.string	"FMFB:Save %d %d\n"
.LC114:
	.string	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\n"
.LC115:
	.string	"FtlBbmTblFlush error:%x\n"
.LC116:
	.string	"FtlBbmTblFlush error = %x error count = %d\n"
.LC117:
	.string	"FtlGcFreeBadSuperBlk 0x%x\n"
.LC118:
	.string	"decrement_vpc_count %x = %d\n"
.LC119:
	.string	"FtlMapWritePage error = %x\n"
.LC120:
	.string	"FtlMapWritePage error = %x error count = %d\n"
.LC121:
	.string	"spuer block %x vpn is 0\n "
.LC122:
	.string	"FtlVpcTblFlush error = %x error count = %d\n"
.LC123:
	.string	"...%s enter...\n"
.LC124:
	.string	"FtlCheckVpc %x = %x  %x\n"
.LC125:
	.string	"Ftlwrite decrement_vpc_count %x = %d\n"
.LC126:
	.string	"GC des block %x done\n"
.LC127:
	.string	"%d GC datablk  = %x vpc %x %x\n"
.LC128:
	.string	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\n"
.LC129:
	.string	"FtlInit %x\n"
.LC130:
	.string	"FtlWrite: lpa error:%x %x\n"
.LC131:
	.string	"IdBlockReadData %x %x\n"
.LC132:
	.string	"IdBlockReadData %x %x ret= %x\n"
.LC133:
	.string	"IDBlockWriteData %x %x\n"
.LC134:
	.string	"IDBlockWriteData %x %x ret= %x\n"
.LC135:
	.string	"idblk:"
.LC136:
	.string	"idb reverse %x %x\n"
.LC137:
	.string	"write_idblock totle_sec %x %x\n"
.LC138:
	.string	"write and check error:%d idb=%x,offset=%x,r=%x,w=%x\n"
.LC139:
	.string	"write"
.LC140:
	.string	"read"
.LC141:
	.string	"write_idblock error\n"
.LC142:
	.string	"READ_SECTOR_IO\n"
.LC143:
	.string	"rk_copy_from_user error\n"
.LC144:
	.string	"READ_SECTOR_IO %x %x\n"
.LC145:
	.string	"rk_copy_to_user error\n"
.LC146:
	.string	"WRITE_SECTOR_IO\n"
.LC147:
	.string	"WRITE_SECTOR_IO %x %x\n"
.LC148:
	.string	"END_WRITE_SECTOR_IO\n"
.LC149:
	.string	"END_WRITE_SECTOR_IO %x %x\n"
.LC150:
	.string	"GET_FLASH_INFO_IO\n"
.LC151:
	.string	"GET_BAD_BLOCK_IO\n"
.LC152:
	.string	"bbt:"
.LC153:
	.string	"GET_LOCK_FLAG_IO\n"
.LC154:
	.string	"GET_PUBLIC_KEY_IO\n"
.LC155:
	.string	"RKNAND_GET_DRM_KEY\n"
.LC156:
	.string	"RKNAND_STORE_DRM_KEY\n"
.LC157:
	.string	"RKNAND_DIASBLE_SECURE_BOOT\n"
.LC158:
	.string	"RKNAND_ENASBLE_SECURE_BOOT\n"
.LC159:
	.string	"RKNAND_GET_SN_SECTOR\n"
.LC160:
	.string	"RKNAND_LOADER_UNLOCK\n"
.LC161:
	.string	"RKNAND_LOADER_STATUS\n"
.LC162:
	.string	"RKNAND_LOADER_LOCK\n"
.LC163:
	.string	"LockKey not match %d\n"
.LC164:
	.string	"RKNAND_GET_VENDOR_SECTOR\n"
.LC165:
	.string	"RKNAND_STORE_VENDOR_SECTOR\n"
.LC166:
	.string	"return ret = %lx\n"
.LC167:
	.string	"secureBootEn check error\n"
.LC168:
	.string	"\0013vendor storage %x,%x,%x\n"
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
	.byte	-75
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
	.hword	1503
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
	.hword	1497
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
	.hword	1497
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
	.type	micron_flash_randomizer_flag, %object
	.size	micron_flash_randomizer_flag, 4
micron_flash_randomizer_flag:
	.word	1
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
	.align	3
.LANCHOR0 = . + 0
.LANCHOR2 = . + 4352
.LANCHOR4 = . + 8704
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
	.type	gFlashSlcMode, %object
	.size	gFlashSlcMode, 1
gFlashSlcMode:
	.zero	1
	.zero	7
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
	.type	slcPageToMlcPageTbl, %object
	.size	slcPageToMlcPageTbl, 512
slcPageToMlcPageTbl:
	.zero	512
	.type	DieAddrs, %object
	.size	DieAddrs, 32
DieAddrs:
	.zero	32
	.type	FlashWaitBusyScheduleEn, %object
	.size	FlashWaitBusyScheduleEn, 4
FlashWaitBusyScheduleEn:
	.zero	4
	.type	gpNandParaInfo, %object
	.size	gpNandParaInfo, 8
gpNandParaInfo:
	.zero	8
	.type	g_retryMode, %object
	.size	g_retryMode, 1
g_retryMode:
	.zero	1
	.type	g_maxRegNum, %object
	.size	g_maxRegNum, 1
g_maxRegNum:
	.zero	1
	.zero	6
	.type	gReadRetryInfo, %object
	.size	gReadRetryInfo, 852
gReadRetryInfo:
	.zero	852
	.zero	4
	.type	read_retry_cur_offset, %object
	.size	read_retry_cur_offset, 4
read_retry_cur_offset:
	.zero	4
	.type	IDByte, %object
	.size	IDByte, 32
IDByte:
	.zero	32
	.type	gDieOp, %object
	.size	gDieOp, 192
gDieOp:
	.zero	192
	.type	gFlashInterfaceMode, %object
	.size	gFlashInterfaceMode, 1
gFlashInterfaceMode:
	.zero	1
	.type	gNandMaxDie, %object
	.size	gNandMaxDie, 1
gNandMaxDie:
	.zero	1
	.zero	2
	.type	DieCsIndex, %object
	.size	DieCsIndex, 8
DieCsIndex:
	.zero	8
	.type	gFlashPageBuffer0, %object
	.size	gFlashPageBuffer0, 8
gFlashPageBuffer0:
	.zero	8
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
	.type	gNandFlashEccBits, %object
	.size	gNandFlashEccBits, 1
gNandFlashEccBits:
	.zero	1
	.type	gMultiPageProgEn, %object
	.size	gMultiPageProgEn, 1
gMultiPageProgEn:
	.zero	1
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
	.zero	6
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
	.zero	6
	.type	p_vendor_block_table, %object
	.size	p_vendor_block_table, 8
p_vendor_block_table:
	.zero	8
	.type	g_wr_page_num, %object
	.size	g_wr_page_num, 4
g_wr_page_num:
	.zero	4
	.zero	4
	.type	req_wr_io, %object
	.size	req_wr_io, 8
req_wr_io:
	.zero	8
	.type	g_MaxLpn, %object
	.size	g_MaxLpn, 4
g_MaxLpn:
	.zero	4
	.type	g_cur_erase_blk, %object
	.size	g_cur_erase_blk, 4
g_cur_erase_blk:
	.zero	4
	.type	gBbtInfo, %object
	.size	gBbtInfo, 96
gBbtInfo:
	.zero	96
	.type	gSysFreeQueue, %object
	.size	gSysFreeQueue, 2056
gSysFreeQueue:
	.zero	2056
	.type	req_erase, %object
	.size	req_erase, 8
req_erase:
	.zero	8
	.type	p_erase_count_table, %object
	.size	p_erase_count_table, 8
p_erase_count_table:
	.zero	8
	.type	g_totle_sys_slc_erase_count, %object
	.size	g_totle_sys_slc_erase_count, 4
g_totle_sys_slc_erase_count:
	.zero	4
	.zero	4
	.type	p_sys_data_buf, %object
	.size	p_sys_data_buf, 8
p_sys_data_buf:
	.zero	8
	.type	p_data_block_list_table, %object
	.size	p_data_block_list_table, 8
p_data_block_list_table:
	.zero	8
	.type	p_data_block_list_head, %object
	.size	p_data_block_list_head, 8
p_data_block_list_head:
	.zero	8
	.type	p_valid_page_count_table, %object
	.size	p_valid_page_count_table, 8
p_valid_page_count_table:
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
	.type	g_inkDie_check_enable, %object
	.size	g_inkDie_check_enable, 4
g_inkDie_check_enable:
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
	.type	g_max_erase_count, %object
	.size	g_max_erase_count, 4
g_max_erase_count:
	.zero	4
	.type	g_min_erase_count, %object
	.size	g_min_erase_count, 4
g_min_erase_count:
	.zero	4
	.type	g_sys_save_data, %object
	.size	g_sys_save_data, 48
g_sys_save_data:
	.zero	48
	.type	c_ftl_nand_data_op_blks_per_plane, %object
	.size	c_ftl_nand_data_op_blks_per_plane, 2
c_ftl_nand_data_op_blks_per_plane:
	.zero	2
	.zero	6
	.type	gSysInfo, %object
	.size	gSysInfo, 12
gSysInfo:
	.zero	12
	.zero	4
	.type	g_gc_superblock, %object
	.size	g_gc_superblock, 48
g_gc_superblock:
	.zero	48
	.type	g_all_blk_used_slc_mode, %object
	.size	g_all_blk_used_slc_mode, 4
g_all_blk_used_slc_mode:
	.zero	4
	.zero	4
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
	.type	g_gc_next_blk, %object
	.size	g_gc_next_blk, 2
g_gc_next_blk:
	.zero	2
	.type	g_gc_next_blk_1, %object
	.size	g_gc_next_blk_1, 2
g_gc_next_blk_1:
	.zero	2
	.type	g_gc_bad_block_temp_num, %object
	.size	g_gc_bad_block_temp_num, 2
g_gc_bad_block_temp_num:
	.zero	2
	.zero	2
	.type	g_gc_bad_block_temp_tbl, %object
	.size	g_gc_bad_block_temp_tbl, 34
g_gc_bad_block_temp_tbl:
	.zero	34
	.type	g_gc_bad_block_gc_index, %object
	.size	g_gc_bad_block_gc_index, 2
g_gc_bad_block_gc_index:
	.zero	2
	.type	mlcPageToSlcPageTbl, %object
	.size	mlcPageToSlcPageTbl, 1024
mlcPageToSlcPageTbl:
	.zero	1024
	.type	gNandMaxChip, %object
	.size	gNandMaxChip, 1
gNandMaxChip:
	.zero	1
	.zero	1
	.type	gTotleBlock, %object
	.size	gTotleBlock, 2
gTotleBlock:
	.zero	2
	.type	g_SlcPartLbaEndSector, %object
	.size	g_SlcPartLbaEndSector, 4
g_SlcPartLbaEndSector:
	.zero	4
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
	.type	c_wr_page_buf_num, %object
	.size	c_wr_page_buf_num, 4
c_wr_page_buf_num:
	.zero	4
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
	.type	p_sys_data_buf_1, %object
	.size	p_sys_data_buf_1, 8
p_sys_data_buf_1:
	.zero	8
	.type	p_vendor_data_buf, %object
	.size	p_vendor_data_buf, 8
p_vendor_data_buf:
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
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 8
p_sys_spare_buf:
	.zero	8
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 8
p_io_spare_buf:
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
	.zero	6
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
	.type	gNandIDataBuf, %object
	.size	gNandIDataBuf, 2048
gNandIDataBuf:
	.zero	2048
	.type	RK29_NANDC_REG_BASE, %object
	.size	RK29_NANDC_REG_BASE, 8
RK29_NANDC_REG_BASE:
	.zero	8
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
	.type	gNandFlashIdbBlockAddr, %object
	.size	gNandFlashIdbBlockAddr, 4
gNandFlashIdbBlockAddr:
	.zero	4
	.type	gNandIDBResBlkNumSaveInFlash, %object
	.size	gNandIDBResBlkNumSaveInFlash, 1
gNandIDBResBlkNumSaveInFlash:
	.zero	1
	.type	g_maxRetryCount, %object
	.size	g_maxRetryCount, 1
g_maxRetryCount:
	.zero	1
	.type	gNandFlashIDBEccBits, %object
	.size	gNandFlashIDBEccBits, 1
gNandFlashIDBEccBits:
	.zero	1
	.zero	5
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
	.type	req_sys, %object
	.size	req_sys, 56
req_sys:
	.zero	56
	.type	g_MaxLbn, %object
	.size	g_MaxLbn, 4
g_MaxLbn:
	.zero	4
	.zero	4
	.type	gVendorBlkInfo, %object
	.size	gVendorBlkInfo, 64
gVendorBlkInfo:
	.zero	64
	.type	g_ect_tbl_power_up_flush, %object
	.size	g_ect_tbl_power_up_flush, 2
g_ect_tbl_power_up_flush:
	.zero	2
	.type	g_power_lost_recovery_flag, %object
	.size	g_power_lost_recovery_flag, 2
g_power_lost_recovery_flag:
	.zero	2
	.type	g_recovery_page_num, %object
	.size	g_recovery_page_num, 4
g_recovery_page_num:
	.zero	4
	.type	g_recovery_ppa_tbl, %object
	.size	g_recovery_ppa_tbl, 128
g_recovery_ppa_tbl:
	.zero	128
	.type	gp_last_act_superblock, %object
	.size	gp_last_act_superblock, 8
gp_last_act_superblock:
	.zero	8
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
	.type	g_LowFormat, %object
	.size	g_LowFormat, 4
g_LowFormat:
	.zero	4
	.type	gLoaderBootInfo, %object
	.size	gLoaderBootInfo, 8
gLoaderBootInfo:
	.zero	8
	.type	RK29_NANDC1_REG_BASE, %object
	.size	RK29_NANDC1_REG_BASE, 8
RK29_NANDC1_REG_BASE:
	.zero	8
	.type	g_ftl_nand_free_count, %object
	.size	g_ftl_nand_free_count, 4
g_ftl_nand_free_count:
	.zero	4
	.type	last_cache_match_count, %object
	.size	last_cache_match_count, 4
last_cache_match_count:
	.zero	4
	.type	g_idb_buffer, %object
	.size	g_idb_buffer, 8
g_idb_buffer:
	.zero	8
	.type	gpDrmKeyInfo, %object
	.size	gpDrmKeyInfo, 8
gpDrmKeyInfo:
	.zero	8
	.type	SecureBootCheckOK, %object
	.size	SecureBootCheckOK, 4
SecureBootCheckOK:
	.zero	4
	.type	SecureBootEn, %object
	.size	SecureBootEn, 4
SecureBootEn:
	.zero	4
	.type	gpBootConfig, %object
	.size	gpBootConfig, 8
gpBootConfig:
	.zero	8
	.type	gSnSectorData, %object
	.size	gSnSectorData, 512
gSnSectorData:
	.zero	512
	.type	SecureBootUnlockTryCount, %object
	.size	SecureBootUnlockTryCount, 4
SecureBootUnlockTryCount:
	.zero	4
	.zero	4
	.type	gpVendor0Info, %object
	.size	gpVendor0Info, 8
gpVendor0Info:
	.zero	8
	.type	gpVendor1Info, %object
	.size	gpVendor1Info, 8
gpVendor1Info:
	.zero	8
	.type	g_vendor, %object
	.size	g_vendor, 8
g_vendor:
	.zero	8
	.type	gToggleModeClkDiv, %object
	.size	gToggleModeClkDiv, 4
gToggleModeClkDiv:
	.zero	4
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
