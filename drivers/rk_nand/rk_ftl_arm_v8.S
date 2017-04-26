/*
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2017-05-19
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
	.type	rknand_sys_storage_release, %function
rknand_sys_storage_release:
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	x0, [x1,208]
	cbz	x0, .L2
	str	xzr, [x1,208]
	bl	kfree
.L2:
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
	.size	rknand_sys_storage_release, .-rknand_sys_storage_release
	.align	2
	.type	rknand_sys_storage_open, %function
rknand_sys_storage_open:
	stp	x29, x30, [sp, -32]!
	adrp	x0, kmalloc_caches+96
	mov	x2, 4096
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	x19, x1
	ldr	x0, [x0,#:lo12:kmalloc_caches+96]
	mov	w1, 192
	movk	w1, 0x240, lsl 16
	bl	kmem_cache_alloc_trace
	str	xzr, [x19,208]
	cbnz	x0, .L8
	adrp	x0, .LC0
	add	x0, x0, :lo12:.LC0
	bl	printk
	mov	w0, -12
	b	.L9
.L8:
	str	x0, [x19,208]
	mov	w0, 0
.L9:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	rknand_sys_storage_open, .-rknand_sys_storage_open
	.align	2
	.type	ftl_set_blk_mode.part.8, %function
ftl_set_blk_mode.part.8:
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
	.size	ftl_set_blk_mode.part.8, .-ftl_set_blk_mode.part.8
	.align	2
	.global	FlashMemCmp8
	.type	FlashMemCmp8, %function
FlashMemCmp8:
	mov	x5, x0
	adrp	x0, .LANCHOR0+8
	ldrb	w0, [x0,#:lo12:.LANCHOR0+8]
	cbz	w0, .L16
	ldrb	w4, [x5,1]
	mov	w0, 0
	ldrb	w3, [x1,1]
	cmp	w4, w3
	beq	.L13
.L16:
	mov	x3, 0
.L14:
	cmp	w3, w2
	mov	w4, w3
	bcs	.L19
	ldrb	w0, [x5,x3]
	add	x3, x3, 1
	add	x6, x1, x3
	ldrb	w6, [x6,-1]
	cmp	w6, w0
	beq	.L14
	add	w0, w4, 1
	b	.L13
.L19:
	mov	w0, 0
.L13:
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
	cbz	w2, .L22
	bl	FlashRsvdBlkChk
	cmp	w0, wzr
	orr	w1, w19, -1073741824
	csel	w19, w1, w19, ne
.L22:
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
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	mov	w20, w1
	stp	x21, x22, [sp,32]
	uxtb	w21, w0
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w2, [x0,20]
	cmp	w2, 5
	bls	.L29
	adrp	x2, .LANCHOR1
	and	w20, w1, 127
	add	x2, x2, :lo12:.LANCHOR1
	ldrb	w0, [x0,16]
	ldrh	w20, [x2,w20,uxtw 1]
	cbz	w0, .L30
	mov	w0, w21
	bl	FlashRsvdBlkChk
	cmp	w0, wzr
	orr	w1, w20, -1073741824
	csel	w20, w1, w20, ne
.L30:
	add	x19, x19, :lo12:.LANCHOR0
	add	x21, x19, x21, sxtw 4
	ldr	x0, [x21,24]
	str	w20, [x0,336]
	b	.L28
.L29:
	cmp	w2, 4
	bne	.L28
	adrp	x2, .LANCHOR1
	and	w3, w1, 127
	add	x2, x2, :lo12:.LANCHOR1
	ldrb	w0, [x0,16]
	ldrh	w22, [x2,w3,uxtw 1]
	lsl	w22, w22, 8
	cbz	w0, .L32
	mov	w0, w21
	bl	FlashRsvdBlkChk
	cbz	w0, .L32
	ubfiz	w20, w20, 1, 7
	orr	w20, w20, 1
	orr	w22, w22, w20
.L32:
	add	x19, x19, :lo12:.LANCHOR0
	add	x21, x19, x21, sxtw 4
	ldr	x0, [x21,24]
	str	w22, [x0,336]
.L28:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
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
	bne	.L46
	mov	w6, 6
	str	w6, [x2,2056]
	str	wzr, [x2,2052]
	str	wzr, [x2,2052]
	str	w5, [x2,2052]
	str	w4, [x2,2052]
	str	w3, [x2,2052]
	b	.L48
.L46:
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
.L48:
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
	cbz	w2, .L49
	sbfiz	x0, x0, 4, 32
	add	x1, x1, 24
	add	x2, x1, x0
	ldr	x1, [x1,x0]
	ldrb	w0, [x2,8]
	add	x0, x0, 8
	add	x0, x1, x0, lsl 8
	mov	w1, 218
	str	w1, [x0,8]
.L49:
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
	cbz	w2, .L54
	sbfiz	x0, x0, 4, 32
	add	x1, x1, 24
	add	x2, x1, x0
	ldr	x1, [x1,x0]
	ldrb	w0, [x2,8]
	add	x0, x0, 8
	add	x0, x1, x0, lsl 8
	mov	w1, 223
	str	w1, [x0,8]
.L54:
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
	cbz	w2, .L61
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
.L61:
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
.L69:
	cmp	w1, w2
	bls	.L71
	lsl	w3, w0, 5
	ldrb	w4, [x5,x2]
	add	w3, w3, w0, lsr 2
	add	x2, x2, 1
	add	w3, w3, w4
	eor	w0, w0, w3
	b	.L69
.L71:
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
	bne	.L77
	add	x1, x7, :lo12:.LANCHOR0
	ldrb	w5, [x1,120]
	cbnz	w5, .L77
	add	x1, x1, 196
	ldrh	w8, [x1,w8,sxtw 1]
.L77:
	add	x7, x7, :lo12:.LANCHOR0
	cmp	w4, 1
	add	x7, x7, 708
	mov	w1, 0
	ldr	w5, [x7,w10,uxtw 2]
	madd	w5, w6, w9, w5
	add	w5, w5, w8
	str	w5, [x2]
	str	w10, [x3]
	bls	.L78
	ldr	w1, [x0,4]
	ldr	w0, [x0,60]
	add	w1, w1, 1024
	cmp	w1, w0
	cset	w1, eq
.L78:
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
	beq	.L90
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
	b	.L92
.L90:
	and	w19, w19, -8193
.L92:
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
.L99:
	ldr	w0, [sp,12]
	sub	w1, w0, #1
	str	w1, [sp,12]
	cbnz	w0, .L99
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
.L103:
	add	x25, x26, :lo12:.LANCHOR0
	ldrb	w0, [x25,753]
	cmp	w0, w20
	bls	.L109
	add	x0, x19, 256
	str	w27, [x22,8]
	ldrsb	w0, [x20,x0]
	str	w0, [x22,4]
	mov	w0, 200
	bl	NandcDelayns
	ldrb	w0, [x25,752]
	cmp	w0, 34
	bne	.L104
	ldrsb	w0, [x24,x20]
	b	.L108
.L104:
	cmp	w0, 35
	bne	.L106
	ldrsb	w0, [x21,x20]
	b	.L108
.L106:
	ldrsb	w0, [x23,400]
.L108:
	str	w0, [x22]
	add	x20, x20, 1
	b	.L103
.L109:
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
.L111:
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w0, [x0,753]
	cmp	w0, w20
	bls	.L113
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
	b	.L111
.L113:
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
	bne	.L115
	add	x19, x19, x23, lsl 6
	lsl	w0, w24, 2
	add	x19, x19, 20
	add	x19, x19, x0, sxtw
	b	.L116
.L115:
	lsl	x2, x23, 3
	add	x2, x2, x24, sxtw
	add	x19, x19, x2, lsl 3
	add	x19, x19, 20
.L116:
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
.L117:
	cmp	x21, x27
	beq	.L119
	ldrb	w0, [x25,x21]
	str	w0, [x20,2052]
	mov	w0, 200
	bl	NandcDelayns
	ldrb	w0, [x19,x21]
	add	x21, x21, 1
	str	w0, [x20,2048]
	b	.L117
.L119:
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
	bhi	.L120
	add	x21, x19, 1620
	add	x22, x19, 764
.L125:
	lsl	x1, x20, 3
	uxtb	w0, w20
	ldrb	w1, [x1,x21]
	cmp	w1, 173
	bne	.L122
	ldrb	w1, [x19,761]
	mov	x2, x22
	mov	w3, 0
	bl	HynixSetRRPara
.L122:
	add	x20, x20, 1
	cmp	x20, 4
	bne	.L125
.L120:
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
	bne	.L128
	add	x4, x4, 88
	cbnz	w2, .L129
	ldrb	w2, [x4,13]
	b	.L137
.L129:
	ldrb	w2, [x4,14]
.L137:
	add	x0, x19, 8
	add	x3, x3, :lo12:.LANCHOR0
	add	x0, x20, x0, lsl 8
	str	w2, [x0,8]
	mov	w2, 0
	ldrb	w4, [x3,103]
	cbz	w4, .L132
	add	x3, x19, 8
	lsl	x3, x3, 8
.L136:
	cmp	w2, w4
	bcs	.L132
	add	x5, x20, x3
	lsl	w0, w2, 3
	lsr	w0, w1, w0
	add	w2, w2, 1
	and	w0, w0, 255
	str	w0, [x5,4]
	b	.L136
.L128:
	add	x0, x19, 8
	mov	w1, 112
	add	x0, x20, x0, lsl 8
	str	w1, [x0,8]
.L132:
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
.L142:
	mov	w0, w19
	mov	w1, w20
	mov	w2, w21
	bl	FlashReadStatusEN
	cmp	w0, 255
	mov	w3, w0
	beq	.L142
	tbz	x3, 6, .L142
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
	cbz	x0, .L147
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
	cbz	x1, .L147
	str	w0, [x1]
	str	xzr, [x19,16]
.L147:
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
.L154:
	mov	w0, 100
	bl	NandcDelayns
	ldr	w0, [x20]
	str	w0, [x29,40]
	ldr	w0, [x29,40]
	tbnz	x0, 9, .L155
	subs	w19, w19, #1
	bne	.L154
	mov	w0, -1
	b	.L153
.L155:
	mov	w0, 0
.L153:
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
.L170:
	ldrb	w3, [x6,x4]
	ldr	x1, [x7]
	cmp	w3, 152
	ldrb	w2, [x7,8]
	beq	.L160
	cmp	w3, 69
	beq	.L160
	cmp	w3, 173
	beq	.L160
	cmp	w3, 44
	bne	.L161
.L160:
	cmp	w0, 1
	bne	.L163
	cbz	w11, .L161
	ubfiz	x2, x2, 8, 8
	cmp	w3, 173
	add	x1, x1, x2
	str	w8, [x1,2056]
	bne	.L164
	str	w0, [x1,2052]
	b	.L181
.L164:
	cmp	w3, 44
	bne	.L166
	str	w0, [x1,2052]
	str	w14, [x1,2048]
	b	.L168
.L166:
	str	w9, [x1,2052]
	str	w0, [x1,2048]
	b	.L168
.L163:
	cbz	w5, .L161
	ubfiz	x2, x2, 8, 8
	cmp	w3, 173
	add	x1, x1, x2
	str	w8, [x1,2056]
	bne	.L167
	str	w10, [x1,2052]
	str	w13, [x1,2048]
	b	.L168
.L167:
	cmp	w3, 44
	bne	.L169
	str	w10, [x1,2052]
	str	w12, [x1,2048]
	b	.L168
.L169:
	str	w9, [x1,2052]
.L181:
	str	wzr, [x1,2048]
.L168:
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
	str	wzr, [x1,2048]
.L161:
	add	x6, x6, 8
	add	x7, x7, 16
	cmp	x6, 32
	bne	.L170
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
.L185:
	add	x3, x5, :lo12:.LANCHOR0
	ldrb	w4, [x3,753]
	cmp	w4, w2
	bls	.L190
	ldrb	w3, [x3,752]
	cmp	w3, 67
	bne	.L186
	ldrsb	w3, [x0,x2]
	b	.L189
.L186:
	ldrsb	w3, [x1,x2]
.L189:
	str	w3, [x20]
	add	x2, x2, 1
	b	.L185
.L190:
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
	adrp	x23, .LC1
	stp	x19, x20, [sp,16]
	mov	w24, w1
	mov	x20, x0
	mov	w22, 0
	add	x25, x21, 1652
	add	x23, x23, :lo12:.LC1
.L192:
	cmp	w22, w24
	beq	.L201
	add	x2, x29, 88
	sub	w4, w24, w22
	mov	x0, x20
	mov	w1, 0
	add	x3, x29, 92
	bl	LogAddr2PhyAddr
	ldr	w2, [x29,92]
	ldrb	w0, [x21,1845]
	cmp	w2, w0
	bcc	.L193
	mov	w0, -1
	str	w0, [x20]
	b	.L194
.L193:
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
	tbz	x0, 0, .L195
	mov	w0, -1
	str	w0, [x20]
.L195:
	ldr	w0, [x20]
	cmn	w0, #1
	bne	.L196
	ldr	w1, [x29,88]
	mov	x0, x23
	bl	printk
.L196:
	mov	w0, w19
	bl	NandcFlashDeCs
.L194:
	add	w22, w22, 1
	add	x20, x20, 56
	b	.L192
.L201:
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
	bne	.L203
	mov	w0, -89
	strb	w0, [x1,4]
	adrp	x0, .LANCHOR1+521
	mov	w1, -9
	strb	w1, [x0,#:lo12:.LANCHOR1+521]
	b	.L240
.L203:
	cmp	w26, 3
	bne	.L205
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
	b	.L256
.L205:
	cmp	w26, 4
	bne	.L206
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
.L256:
	mov	w21, 8
	strb	w0, [x1,11]
	mov	w20, w21
	b	.L204
.L206:
	cmp	w26, 5
	bne	.L207
	mov	w0, 56
	strb	w0, [x1,4]
	mov	w0, 57
	strb	w0, [x1,5]
	mov	w0, 58
	mov	w21, 8
	strb	w0, [x1,6]
	mov	w0, 59
	strb	w0, [x1,7]
	b	.L255
.L207:
	cmp	w26, 6
	bne	.L240
	mov	w0, 14
	strb	w0, [x1,4]
	mov	w0, 15
	strb	w0, [x1,5]
	mov	w0, 16
	mov	w21, 12
	strb	w0, [x1,6]
	mov	w0, 17
	strb	w0, [x1,7]
	b	.L255
.L240:
	mov	w21, 7
.L255:
	mov	w20, 4
.L204:
	sub	w0, w26, #1
	cmp	w0, 1
	bhi	.L252
	adrp	x24, .LANCHOR1
	add	x27, x19, :lo12:.LANCHOR0
	add	x24, x24, :lo12:.LANCHOR1
	mov	w23, 0
	add	x28, x27, 760
	add	x4, x27, 24
	add	x24, x24, 504
.L208:
	ldrb	w0, [x27,1845]
	cmp	w0, w23
	bls	.L215
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
.L210:
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
	bhi	.L210
	mov	x0, 0
.L211:
	add	w2, w0, 8
	mov	x1, 0
.L212:
	add	x3, x1, x0
	add	x1, x1, 4
	add	x3, x24, x3
	cmp	x1, 24
	ldrb	w5, [x3,4]
	ldrb	w3, [x22,x0]
	add	w3, w5, w3
	strb	w3, [x22,w2,sxtw]
	add	w2, w2, 8
	bne	.L212
	add	x0, x0, 1
	cmp	x0, 4
	bne	.L211
	add	w23, w23, 1
	strb	wzr, [x22,16]
	strb	wzr, [x22,24]
	strb	wzr, [x22,32]
	uxtb	w23, w23
	strb	wzr, [x22,40]
	strb	wzr, [x22,48]
	strb	wzr, [x22,41]
	strb	wzr, [x22,49]
	b	.L208
.L252:
	sub	w0, w26, #3
	cmp	w0, 3
	bhi	.L215
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
.L216:
	ldrb	w0, [x23,1845]
	cmp	w0, w22
	bhi	.L239
.L215:
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
.L239:
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
	bne	.L217
	mov	w0, 64
	str	w3, [x28,2052]
	str	w0, [x28,2048]
	mov	w0, 204
	b	.L257
.L217:
	cmp	w27, 1
	bhi	.L219
	ldr	x0, [x29,136]
	ldrb	w0, [x0,4]
	str	w0, [x28,2052]
	mov	w0, 82
	b	.L258
.L219:
	mov	w0, 174
	str	w0, [x28,2052]
	str	wzr, [x28,2048]
	mov	w0, 176
.L257:
	str	w0, [x28,2052]
	mov	w0, 77
.L258:
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
	bne	.L220
	mov	w0, 31
	str	w0, [x28,2052]
	b	.L221
.L220:
	str	wzr, [x28,2052]
.L221:
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
.L223:
	add	w0, w0, 1
	ldr	w4, [x28,2048]
	uxtb	w0, w0
	cmp	w0, w3
	bne	.L223
	ldr	x6, [x23,1856]
	mov	x0, 0
.L224:
	ldr	w1, [x29,124]
	cmp	w1, w0
	ble	.L259
	ldr	w3, [x28,2048]
	strb	w3, [x6,x0]
	add	x0, x0, 1
	b	.L224
.L259:
	ldr	x8, [x23,1856]
	mov	w5, w24
	mov	w4, 8
.L227:
	mov	w0, 0
.L226:
	add	w3, w0, w5
	add	w0, w0, 1
	sbfiz	x3, x3, 2, 32
	cmp	w0, w24
	ldr	w7, [x8,x3]
	mvn	w7, w7
	str	w7, [x8,x3]
	bne	.L226
	ldr	w0, [x29,116]
	subs	w4, w4, #1
	add	w5, w5, w0
	bne	.L227
	mov	x4, 0
	mov	w12, 1
.L228:
	mov	w3, 0
	ldr	x11, [x23,1856]
	mov	w7, w3
.L232:
	lsl	w5, w12, w7
	mov	w9, w4
	mov	w0, 16
	mov	w8, 0
.L230:
	ldr	w10, [x11,w9,sxtw 2]
	add	w9, w9, w24
	and	w10, w5, w10
	cmp	w10, w5
	csinc	w8, w8, w8, ne
	subs	w0, w0, #1
	bne	.L230
	cmp	w8, 9
	orr	w5, w3, w5
	add	w7, w7, 1
	csel	w3, w5, w3, cs
	cmp	w7, 32
	bne	.L232
	str	w3, [x11,x4,lsl 2]
	add	x4, x4, 1
	cmp	w24, w4
	bgt	.L228
	mov	w4, w0
.L235:
	mov	x3, 0
.L234:
	add	w5, w0, w3
	ldr	x1, [x29,128]
	ldrb	w7, [x6,x3]
	add	x3, x3, 1
	cmp	w20, w3, uxtb
	strb	w7, [x1,w5,sxtw]
	bhi	.L234
	ldr	x1, [x29,104]
	add	w4, w4, 1
	cmp	w4, w21
	add	x6, x6, x1
	ldr	w1, [x29,120]
	add	w0, w0, w1
	blt	.L235
	mov	w0, 255
	str	w0, [x28,2056]
	mov	w0, w25
	bl	NandcWaitFlashReady
	cmp	w27, 1
	bhi	.L237
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
	b	.L238
.L237:
	mov	w0, 56
	str	w0, [x28,2056]
.L238:
	mov	w0, w25
	add	w22, w22, 1
	bl	NandcWaitFlashReady
	uxtb	w22, w22
	b	.L216
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
	bne	.L261
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
	b	.L262
.L261:
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
.L262:
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
	cbz	w1, .L264
	ldrb	w0, [x20,1844]
	tbz	x0, 0, .L264
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	strb	wzr, [x20,1864]
.L264:
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
	ble	.L273
	ldr	x0, [x1,128]
	mov	w1, 8354
	b	.L280
.L273:
	cmp	w0, 220
	ble	.L275
	ldr	x0, [x1,128]
	b	.L281
.L275:
	cmp	w0, 185
	ble	.L276
	ldr	x0, [x1,128]
	mov	w1, 4226
	b	.L280
.L276:
	cmp	w0, 160
	ldr	x0, [x1,128]
	ble	.L277
	mov	w1, 4194
	b	.L280
.L277:
	cmp	w19, 35
	mov	w1, 4193
	bls	.L280
	cmp	w19, 99
	mov	w1, 4225
	bls	.L280
.L281:
	mov	w1, 8322
.L280:
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
	bls	.L283
	mov	w1, 8322
	cmp	w0, w1
	bne	.L284
.L283:
	adrp	x1, .LANCHOR0+128
	ldr	x1, [x1,#:lo12:.LANCHOR0+128]
	str	w0, [x1,4]
.L284:
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
	str	w2, [x19,20]
	ldr	w2, [x0,352]
	str	w2, [x19,1872]
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
	bne	.L288
.L291:
	and	w1, w1, -17
	b	.L289
.L288:
	cmp	w0, 24
	bne	.L290
	orr	w1, w1, 16
	b	.L289
.L290:
	cmp	w0, 40
	orr	w1, w1, 262144
	orr	w1, w1, 16
	beq	.L291
.L289:
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
.L298:
	lsl	x0, x21, 3
	ldrb	w0, [x0,x19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L297
	mov	w0, w21
	bl	FlashReset
.L297:
	add	x21, x21, 1
	cmp	x21, 4
	bne	.L298
	add	x19, x20, :lo12:.LANCHOR0
	ldrb	w0, [x19,1864]
	cbz	w0, .L299
	mov	w0, 1
	bl	NandcSetMode
	ldrb	w0, [x19,1844]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x19,1844]
	bl	NandcSetMode
	ldr	w0, [x19,152]
	lsr	w0, w0, 8
	bl	NandcSetDdrPara
.L299:
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
	tbnz	x0, 9, .L309
	mov	x0, x19
	bl	wait_for_nand_flash_ready
	b	.L308
.L309:
	mov	x0, x19
	mov	w1, 1
	bl	NandCIrqDisable
.L308:
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
	cbz	w2, .L312
	mov	w1, w23
	bl	FlashEraseSLc2KBlocks
	b	.L313
.L312:
	mov	w28, 56
	add	x25, x19, 1652
	mov	x26, 24
.L337:
	cmp	w20, w23
	bcs	.L338
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
	bcc	.L314
	mov	w0, -1
	str	w0, [x24,x5]
	b	.L315
.L314:
	ldrb	w1, [x19,1945]
	cmp	w1, wzr
	uxtw	x1, w0
	csel	w22, w22, wzr, ne
	madd	x1, x1, x26, x25
	ldr	x1, [x1,8]
	cbz	x1, .L317
	bl	FlashWaitCmdDone
.L317:
	ldr	w0, [x29,124]
	ldr	w1, [x29,120]
	madd	x2, x0, x26, x25
	str	x21, [x2,8]
	str	xzr, [x2,16]
	str	w1, [x2,4]
	cbz	w22, .L318
	add	w1, w20, 1
	umull	x1, w1, w28
	add	x1, x24, x1
	str	x1, [x2,16]
.L318:
	add	x1, x19, x0
	mul	x0, x0, x26
	ldrb	w21, [x1,1848]
	strb	w21, [x25,x0]
	mov	w0, w21
	bl	NandcFlashCs
	cmp	w27, 1
	bne	.L319
	ldrb	w0, [x19,120]
	cbz	w0, .L319
	mov	w0, w21
	bl	flash_enter_slc_mode
.L319:
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
.L315:
	add	w20, w20, 1
	b	.L337
.L338:
	ldr	x0, [x29,104]
	mov	x20, 0
	mov	x22, 24
	add	x19, x0, :lo12:.LANCHOR0
	add	x21, x19, 1652
	ldr	x0, [x19,128]
	bl	NandcIqrWaitFlashReady
.L321:
	ldrb	w0, [x19,1845]
	cmp	w0, w20
	bls	.L339
	mov	w0, w20
	bl	FlashWaitCmdDone
	cmp	w27, 1
	bne	.L322
	ldrb	w0, [x19,120]
	cbz	w0, .L322
	mul	x0, x20, x22
	ldrb	w0, [x0,x21]
	bl	flash_exit_slc_mode
.L322:
	add	x20, x20, 1
	b	.L321
.L339:
	mov	w0, 0
.L313:
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
.L343:
	ldr	w1, [x0,8]
	str	w1, [sp,8]
	ldr	w1, [sp,8]
	tbz	x1, 20, .L343
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
	ldr	w0, [x1,20]
	orr	w20, w20, 536870912
	cmp	w0, 3
	orr	w20, w20, 1024
	bfi	w20, w3, 4, 1
	bls	.L347
	ldr	w0, [x21,16]
	cmp	x5, xzr
	str	w0, [x29,88]
	cset	w8, ne
	ldr	w0, [x29,88]
	and	w0, w0, -5
	str	w0, [x29,88]
	cbnz	w8, .L361
	cbz	x22, .L348
.L361:
	cbnz	w24, .L350
.L358:
	mov	x0, x21
	bl	rk_nandc_xfer_irq_flag_init
	mov	x0, x21
	mov	w1, 0
	bl	NandCIrqEnable
	add	w2, w25, 1
	mov	x0, x22
	asr	w2, w2, 1
	bfi	w20, w2, 22, 6
	cbnz	x22, .L352
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x0, [x0,1888]
	b	.L352
.L350:
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
.L354:
	cmp	w4, w2
	bcs	.L358
	lsr	w0, w3, 2
	ubfiz	x0, x0, 2, 30
	cbz	w8, .L355
	ldrh	w7, [x5,2]
	ldr	x10, [x1,1896]
	ldrh	w11, [x5],4
	orr	w7, w11, w7, lsl 16
	str	w7, [x10,x0]
	b	.L356
.L355:
	ldr	x7, [x1,1896]
	str	w9, [x7,x0]
.L356:
	add	w4, w4, 1
	add	w3, w3, w6
	b	.L354
.L352:
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
	cbnz	x22, .L359
	ldr	w0, [x29,88]
	mov	w1, 2
	bfi	w0, w1, 3, 3
	str	w0, [x29,88]
.L359:
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
.L348:
	ldr	w0, [x29,88]
	str	w0, [x21,16]
.L347:
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
.L366:
	cmp	w2, w0
	bhi	.L368
	add	w1, w1, 1
	lsl	w2, w2, 1
	uxth	w1, w1
	b	.L366
.L368:
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
	stp	x29, x30, [sp, -80]!
	mov	x3, 0
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
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
.L373:
	add	x5, x0, 1984
	strb	w3, [x3,x5]
	add	x3, x3, 1
	cmp	x3, 32
	bne	.L373
	ldrh	w3, [x23,20]
	ldrb	w0, [x23,15]
	cmp	w3, w0
	bcs	.L374
	uxtb	w8, w1
	mov	w3, 0
	mul	w11, w1, w2
	ubfiz	w10, w8, 1, 7
	add	x12, x19, :lo12:.LANCHOR0
.L375:
	cmp	w3, w1
	bcs	.L377
	uxtb	w0, w3
	mov	w5, w3
	mov	w7, 0
.L378:
	cmp	w7, w2
	bcs	.L391
	add	x9, x12, 1984
	add	w13, w5, w11
	add	w14, w0, w8
	add	w7, w7, 1
	strb	w0, [x9,w5,uxtw]
	add	w0, w0, w10
	strb	w14, [x9,x13]
	add	w5, w5, w1
	uxtb	w0, w0
	b	.L378
.L391:
	add	w3, w3, 1
	b	.L375
.L377:
	add	x0, x19, :lo12:.LANCHOR0
	lsl	w2, w2, 1
	lsr	w4, w4, 1
	strh	w2, [x0,1974]
	strh	w4, [x0,1962]
.L374:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 5
	cmp	w6, 1
	strh	w2, [x0,2016]
	strh	wzr, [x0,2018]
	bne	.L379
	strh	w6, [x0,2016]
.L379:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w2, 4352
	strh	w2, [x0,2020]
	ldrb	w2, [x0,8]
	cbz	w2, .L380
	mov	w2, 384
	strh	w2, [x0,2020]
.L380:
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
	ldrh	w26, [x23,18]
	strh	w25, [x21,2026]
	strh	w26, [x21,2028]
	strh	w24, [x21,2032]
	bl	Ftl_log2
	uxth	w3, w0
	strh	w0, [x21,2034]
	lsl	w0, w24, 9
	strh	w0, [x21,2036]
	ubfx	x0, x0, 9, 7
	cmp	w20, 1024
	lsl	w0, w0, 1
	strh	w0, [x21,2038]
	ldrh	w0, [x23,26]
	strh	w0, [x21,2040]
	mul	w0, w22, w20
	str	w0, [x21,1968]
	bls	.L381
	and	w0, w20, 255
	strh	w0, [x21,2018]
.L381:
	add	x2, x19, :lo12:.LANCHOR0
	ldrh	w1, [x2,2018]
	ldrh	w0, [x2,2020]
	sub	w1, w20, w1
	mul	w1, w22, w1
	mul	w1, w1, w24
	mul	w24, w24, w26
	mul	w1, w1, w25
	asr	w1, w1, 11
	str	w1, [x2,2044]
	lsl	w1, w0, 3
	sdiv	w24, w1, w24
	uxth	w24, w24
	cmp	w24, 4
	bls	.L382
	strh	w24, [x2,2048]
	b	.L383
.L382:
	mov	w1, 4
	strh	w1, [x2,2048]
.L383:
	add	x1, x19, :lo12:.LANCHOR0
	asr	w0, w0, w3
	lsl	w20, w20, 6
	add	w3, w3, 9
	asr	w20, w20, w3
	add	w0, w0, 2
	strh	w20, [x1,2052]
	and	w20, w20, 65535
	strh	w0, [x1,2050]
	cmp	w22, 1
	mul	w0, w20, w22
	str	w0, [x1,2056]
	add	w20, w20, 8
	ldrh	w0, [x1,2048]
	udiv	w0, w0, w22
	add	w20, w20, w0
	bne	.L390
	add	w20, w20, 4
.L390:
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
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 80
	mov	w0, 0
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
	cbz	w2, .L393
	ldrh	w2, [x1,2048]
	ldr	x3, [x1,2080]
	mov	x1, 0
.L394:
	cmp	w2, w1, uxth
	bls	.L399
	add	x1, x1, 1
	add	x0, x3, x1, lsl 1
	ldrh	w0, [x0,-2]
	cmp	w0, w4
	bne	.L394
	mov	w0, 1
	b	.L393
.L399:
	mov	w0, 0
.L393:
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
	cbz	w3, .L401
	ldr	x5, [x2,2096]
	mov	x0, 56
	mov	x2, 0
.L402:
	cmp	w3, w2
	bls	.L411
	madd	x4, x2, x0, x5
	add	x2, x2, 1
	ldr	w4, [x4,24]
	cmp	w4, w6
	bcc	.L402
	cmp	w4, w1
	bhi	.L402
	mov	w0, 1
	b	.L401
.L411:
	mov	w0, 0
.L401:
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
	adrp	x0, .LC2
	add	x0, x0, :lo12:.LC2
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
.L421:
	cmp	w20, w21
	bge	.L427
	mov	w0, w20
	bl	FtlBbmIsBadBlock
	cbz	w0, .L422
	add	w19, w19, 1
	uxth	w19, w19
.L422:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L421
.L427:
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
	bls	.L432
	sub	w1, w0, w1
	cmp	w1, w2
	cset	w0, ls
	b	.L433
.L432:
	sub	w1, w1, w0
	cmp	w1, w2
	cset	w0, hi
.L433:
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
	beq	.L436
	cbz	w1, .L438
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
.L438:
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
.L436:
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
.L446:
	ldrh	w0, [x20,6]
	cmp	w0, w19
	bls	.L454
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
	b	.L446
.L454:
	adrp	x4, .LANCHOR0
	adrp	x7, .LANCHOR2
	mov	w0, 0
	add	x4, x4, :lo12:.LANCHOR0
	mov	x8, x7
.L448:
	ldrh	w6, [x4,2214]
	sub	w1, w6, #1
	cmp	w0, w1
	bge	.L455
	add	x1, x7, :lo12:.LANCHOR2
	add	w5, w0, 1
	uxth	w5, w5
	ldr	x3, [x1,-64]
	mov	w2, w5
	mov	w1, w0
.L449:
	cmp	w2, w6
	bcs	.L456
	ubfiz	x9, x2, 2, 16
	ldr	w10, [x3,w1,uxtw 2]
	ldr	w9, [x3,x9]
	cmp	w10, w9
	csel	w1, w1, w2, ls
	add	w2, w2, 1
	uxth	w2, w2
	b	.L449
.L456:
	cmp	w0, w1
	beq	.L452
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
.L452:
	mov	w0, w5
	b	.L448
.L455:
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
	beq	.L458
	ldrh	w5, [x1,2]
	mov	w3, w0
.L459:
	cmp	w3, w4
	bcs	.L463
	add	w2, w3, w5
	ubfiz	x2, x2, 1, 10
	add	x2, x1, x2
	ldrh	w0, [x2,8]
	cmp	w0, w6
	beq	.L462
	add	w3, w3, 1
	b	.L459
.L463:
	mov	w0, 0
	b	.L458
.L462:
	mov	w0, 1
.L458:
	ret
	.size	IsInFreeQueue, .-IsInFreeQueue
	.align	2
	.global	FtlFreeSysBlkQueueOut
	.type	FtlFreeSysBlkQueueOut, %function
FtlFreeSysBlkQueueOut:
	adrp	x1, .LANCHOR0
	mov	w0, 0
	add	x1, x1, :lo12:.LANCHOR0
	add	x1, x1, 2208
	ldrh	w3, [x1,6]
	cbz	w3, .L465
	ldrh	w2, [x1,2]
	add	x0, x1, x2, sxtw 1
	ldrh	w0, [x0,8]
	sub	w3, w3, #1
	add	w2, w2, 1
	strh	w3, [x1,6]
	and	w2, w2, 1023
	strh	w2, [x1,2]
.L465:
	ret
	.size	FtlFreeSysBlkQueueOut, .-FtlFreeSysBlkQueueOut
	.align	2
	.global	insert_data_list
	.type	insert_data_list, %function
insert_data_list:
	uxth	w0, w0
	mov	w9, 65535
	cmp	w0, w9
	beq	.L469
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
	cbnz	x1, .L470
	str	x4, [x5,-48]
	b	.L469
.L470:
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
.L477:
	ubfiz	x15, x2, 1, 16
	ldrh	w16, [x1,4]
	cmp	w16, wzr
	ldrh	w5, [x13,x15]
	mul	w5, w5, w16
	csinv	w5, w5, wzr, ne
	cmp	w5, w6
	bne	.L473
	ldrh	w15, [x14,x15]
	ldrh	w5, [x8]
	cmp	w15, w5
	bcc	.L475
	b	.L474
.L473:
	bhi	.L474
.L475:
	ldrh	w5, [x1]
	cmp	w5, w9
	bne	.L476
	strh	w2, [x4,2]
	add	x2, x3, :lo12:.LANCHOR2
	strh	w0, [x1]
	str	x4, [x2,-32]
	b	.L469
.L476:
	umull	x1, w5, w7
	mov	w2, w5
	add	x1, x12, x1
	b	.L477
.L474:
	strh	w2, [x11,x10]
	ldrh	w2, [x1,2]
	strh	w2, [x4,2]
	add	x2, x3, :lo12:.LANCHOR2
	ldr	x3, [x2,-48]
	cmp	x1, x3
	bne	.L478
	strh	w0, [x1,2]
	str	x4, [x2,-48]
	b	.L469
.L478:
	ldrh	w3, [x1,2]
	mov	w4, 6
	ldr	x2, [x2,-56]
	umull	x3, w3, w4
	strh	w0, [x2,x3]
	strh	w0, [x1,2]
.L469:
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
	beq	.L483
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
	cbnz	x1, .L484
	str	x4, [x5,-16]
	b	.L483
.L484:
	ldr	x11, [x5,-80]
	ubfiz	x2, x0, 1, 16
	ldr	x10, [x5,-56]
	mov	x5, -6148914691236517206
	ldrh	w12, [x11,x2]
	sub	x2, x1, x10
	asr	x2, x2, 1
	madd	x2, x5, x2, x2
	uxth	w2, w2
.L487:
	ubfiz	x5, x2, 1, 16
	ldrh	w5, [x11,x5]
	cmp	w5, w12
	bcs	.L485
	ldrh	w5, [x1]
	cmp	w5, w7
	bne	.L486
	strh	w2, [x4,2]
	strh	w0, [x1]
	b	.L483
.L486:
	umull	x1, w5, w6
	mov	w2, w5
	add	x1, x10, x1
	b	.L487
.L485:
	ldrh	w5, [x1,2]
	strh	w5, [x4,2]
	strh	w2, [x9,x8]
	add	x2, x3, :lo12:.LANCHOR2
	ldr	x3, [x2,-16]
	cmp	x1, x3
	bne	.L488
	strh	w0, [x1,2]
	str	x4, [x2,-16]
	b	.L483
.L488:
	ldrh	w3, [x1,2]
	mov	w4, 6
	ldr	x2, [x2,-56]
	umull	x3, w3, w4
	strh	w0, [x2,x3]
	strh	w0, [x1,2]
.L483:
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
	bne	.L491
	ldrh	w3, [x2,x1]
	cmp	w3, w7
	bne	.L492
	str	xzr, [x0]
	b	.L493
.L492:
	umull	x3, w3, w5
	add	x3, x2, x3
	str	x3, [x0]
	mov	w0, -1
	strh	w0, [x3,2]
	b	.L493
.L491:
	ldrh	w0, [x2,x1]
	ldrh	w3, [x4,2]
	cmp	w0, w7
	bne	.L494
	umull	x3, w3, w5
	mov	w0, -1
	strh	w0, [x2,x3]
	b	.L493
.L494:
	umull	x0, w0, w5
	add	x0, x2, x0
	strh	w3, [x0,2]
	ldrh	w3, [x4,2]
	ldr	x0, [x6,-56]
	ldrh	w7, [x2,x1]
	umull	x3, w3, w5
	strh	w7, [x0,x3]
.L493:
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
	cbz	x2, .L496
	adrp	x3, .LANCHOR2-56
	mov	w5, w19
	mov	w6, 6
	ldr	x4, [x3,#:lo12:.LANCHOR2-56]
.L497:
	cbnz	w1, .L498
.L500:
	sub	x2, x2, x4
	mov	x1, -6148914691236517206
	asr	x2, x2, 1
	madd	x2, x1, x2, x2
	uxth	w19, w2
	mov	w1, w19
	bl	List_remove_node
	b	.L496
.L498:
	ldrh	w3, [x2]
	cmp	w3, w5
	beq	.L500
	umull	x3, w3, w6
	sub	w1, w1, #1
	add	x2, x4, x3
	uxth	w1, w1
	b	.L497
.L496:
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
	cbz	x1, .L509
	ldr	x4, [x2,-56]
	mov	w5, 6
.L505:
	cbz	w3, .L506
	ldrh	w2, [x1]
	cmp	w2, w0
	beq	.L509
	umull	x2, w2, w5
	sub	w3, w3, #1
	add	x1, x4, x2
	uxth	w3, w3
	b	.L505
.L506:
	sub	x1, x1, x4
	mov	x0, -6148914691236517206
	asr	x1, x1, 1
	madd	x1, x0, x1, x1
	uxth	w0, w1
.L509:
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
	beq	.L511
	ldrh	w0, [x19,48]
	cmp	w0, w20
	beq	.L511
	ldrh	w0, [x19,96]
	cmp	w0, w20
	beq	.L511
	mov	w2, 6
	ldr	x4, [x19,-56]
	ldr	x0, [x19,-48]
	umull	x3, w20, w2
	add	x3, x4, x3
	cmp	x3, x0
	beq	.L511
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
	bcs	.L511
	sub	x0, x19, #48
	mov	w1, w20
	bl	List_remove_node
	ldrh	w0, [x19,-24]
	sub	w0, w0, #1
	strh	w0, [x19,-24]
	mov	w0, w20
	bl	INSERT_DATA_LIST
.L511:
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
.L515:
	cmp	w20, w2
	beq	.L518
	mov	x22, x1
	ldrh	w21, [x1],2
	cbnz	w21, .L516
	mov	x19, x0
	bl	FtlFreeSysBlkQueueOut
	uxth	w1, w0
	strh	w0, [x22]
	cbz	w1, .L518
	ldr	w0, [x19,48]
	strh	w21, [x19,2]
	add	w0, w0, 1
	str	w0, [x19,48]
	ldrh	w0, [x19,8]
	strh	w20, [x19]
	add	w0, w0, 1
	strh	w0, [x19,8]
	b	.L518
.L516:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L515
.L518:
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
.L522:
	uxth	w0, w4
	cmp	w0, w2
	bcs	.L538
	add	x4, x4, 1
	add	x6, x1, x4, lsl 4
	ldrh	w6, [x6,-16]
	cmp	w6, w5
	bne	.L522
	b	.L523
.L538:
	mov	w0, w2
	mov	x4, 0
	mov	w7, -2147483648
.L525:
	uxth	w6, w4
	cmp	w6, w2
	bcs	.L539
	add	x5, x1, x4, lsl 4
	ldr	w5, [x5,4]
	tbnz	w5, #31, .L534
	cmp	w5, w7
	bcc	.L526
.L534:
	mov	w5, w7
	mov	w6, w0
.L526:
	add	x4, x4, 1
	mov	w7, w5
	mov	w0, w6
	b	.L525
.L539:
	cmp	w0, w2
	bcc	.L523
	add	x0, x3, :lo12:.LANCHOR2
	add	x1, x1, 4
	mov	w4, -1
	mov	w3, 0
	ldrh	w5, [x0,152]
	mov	w0, w2
.L529:
	cmp	w3, w2
	beq	.L523
	ldr	w7, [x1]
	cmp	w7, w4
	bcs	.L530
	ldrh	w6, [x1,-4]
	cmp	w6, w5
	csel	w4, w4, w7, eq
	cmp	w6, w5
	csel	w0, w0, w3, eq
.L530:
	add	w3, w3, 1
	add	x1, x1, 16
	uxth	w3, w3
	b	.L529
.L523:
	ret
	.size	select_l2p_ram_region, .-select_l2p_ram_region
	.align	2
	.global	FtlUpdateVaildLpn
	.type	FtlUpdateVaildLpn, %function
FtlUpdateVaildLpn:
	adrp	x1, .LANCHOR2
	add	x3, x1, :lo12:.LANCHOR2
	ldrh	w2, [x3,154]
	cbnz	w0, .L541
	cmp	w2, 4
	bhi	.L541
	add	w2, w2, 1
	strh	w2, [x3,154]
	b	.L540
.L541:
	adrp	x2, .LANCHOR0+1960
	add	x0, x1, :lo12:.LANCHOR2
	mov	w7, 65535
	ldrh	w5, [x2,#:lo12:.LANCHOR0+1960]
	strh	wzr, [x0,154]
	str	wzr, [x0,156]
	ldr	x6, [x0,-40]
	mov	x0, 0
.L542:
	cmp	w5, w0, uxth
	bls	.L540
	ldrh	w4, [x6,x0,lsl 1]
	cmp	w4, w7
	beq	.L543
	add	x3, x1, :lo12:.LANCHOR2
	ldr	w2, [x3,156]
	add	w2, w4, w2
	str	w2, [x3,156]
.L543:
	add	x0, x0, 1
	b	.L542
.L540:
	ret
	.size	FtlUpdateVaildLpn, .-FtlUpdateVaildLpn
	.align	2
	.global	ftl_set_blk_mode
	.type	ftl_set_blk_mode, %function
ftl_set_blk_mode:
	stp	x29, x30, [sp, -16]!
	uxth	w0, w0
	add	x29, sp, 0
	cbz	w1, .L547
	bl	ftl_set_blk_mode.part.8
	b	.L546
.L547:
	adrp	x1, .LANCHOR0
	ubfx	x2, x0, 5, 11
	lsl	x2, x2, 2
	ldr	x3, [x1,#:lo12:.LANCHOR0]
	mov	w1, 1
	lsl	w0, w1, w0
	ldr	w1, [x3,x2]
	bic	w0, w1, w0
	str	w0, [x3,x2]
.L546:
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
.L551:
	cmp	w2, w4
	bcs	.L557
	add	x5, x0, x2, sxtw 1
	ldrh	w5, [x5,16]
	cmp	w5, w6
	beq	.L552
	ldrh	w5, [x0,4]
	add	w5, w5, 1
	strh	w5, [x0,4]
.L552:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L551
.L557:
	add	x3, x3, :lo12:.LANCHOR0
	mov	w6, 65535
	add	x4, x0, x4, uxth 1
	ldrh	w5, [x3,2026]
	mov	x3, x0
.L554:
	cmp	x3, x4
	beq	.L558
	ldrh	w2, [x3,16]
	cmp	w2, w6
	beq	.L555
	ldrh	w2, [x0,4]
	add	w2, w5, w2
	sub	w2, w2, #1
	sub	w2, w2, w1
	strh	w2, [x0,4]
.L555:
	add	x3, x3, 2
	b	.L554
.L558:
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
	mov	x19, x0
	strh	wzr, [x0,4]
	strb	wzr, [x0,7]
	adrp	x0, .LANCHOR0
	add	x1, x0, :lo12:.LANCHOR0
	add	x22, x19, 16
	mov	x21, 0
	mov	x20, x0
	mov	x23, x1
	mov	w25, -1
	ldrh	w24, [x1,1952]
.L560:
	cmp	w24, w21, uxth
	bls	.L567
	add	x0, x23, 1984
	ldrh	w1, [x19]
	ldrb	w0, [x0,x21]
	bl	V2P_block
	strh	w25, [x22]
	mov	w26, w0
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L561
	strh	w26, [x22]
	ldrb	w0, [x19,7]
	add	w0, w0, 1
	strb	w0, [x19,7]
.L561:
	add	x21, x21, 1
	add	x22, x22, 2
	b	.L560
.L567:
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w1, [x19,7]
	strb	wzr, [x19,9]
	ldrh	w0, [x0,2026]
	mul	w0, w1, w0
	strh	w0, [x19,4]
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldr	w1, [x0,160]
	cbz	w1, .L563
	ldrh	w1, [x19]
	ldr	x0, [x0,-80]
	ldrh	w0, [x0,x1,lsl 1]
	cmp	w0, 59
	bhi	.L563
	mov	w0, 1
	strb	w0, [x19,9]
.L563:
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
.L569:
	cmp	w23, w22, uxth
	bls	.L576
	add	x0, x20, 1984
	mov	w1, w21
	ldrb	w0, [x0,x22]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L570
	add	w19, w19, w24
	uxth	w19, w19
.L570:
	add	x22, x22, 1
	b	.L569
.L576:
	cbz	w19, .L572
	mov	w0, 32768
	sdiv	w19, w0, w19
.L572:
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
	cbz	x1, .L578
	ldr	x0, [x2,-56]
	sub	x1, x1, x0
	mov	x0, -6148914691236517206
	asr	x1, x1, 1
	madd	x1, x0, x1, x1
	ldr	x0, [x2,-80]
	and	x1, x1, 65535
	ldrh	w0, [x0,x1,lsl 1]
.L578:
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
	cbz	x1, .L581
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
.L583:
	cmp	w3, w4
	beq	.L586
	umull	x5, w1, w6
	ldrh	w5, [x0,x5]
	cmp	w5, w7
	bne	.L584
.L586:
	add	x2, x2, :lo12:.LANCHOR2
	ubfiz	x1, x1, 1, 16
	ldr	x0, [x2,-80]
	ldrh	w0, [x0,x1]
	b	.L581
.L584:
	add	w3, w3, 1
	mov	w1, w5
	uxth	w3, w3
	b	.L583
.L581:
	ret
	.size	GetFreeBlockMaxEraseCount, .-GetFreeBlockMaxEraseCount
	.align	2
	.global	FtlPrintInfo2buf
	.type	FtlPrintInfo2buf, %function
FtlPrintInfo2buf:
	stp	x29, x30, [sp, -112]!
	adrp	x1, .LC3
	add	x29, sp, 0
	stp	x25, x26, [sp,64]
	adrp	x25, .LANCHOR0
	add	x1, x1, :lo12:.LC3
	add	x25, x25, :lo12:.LANCHOR0
	str	x27, [sp,80]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x21, x22, [sp,32]
	mov	x23, x0
	bl	strcpy
	add	x20, x23, 12
	ldr	w2, [x25,168]
	adrp	x1, .LC4
	mov	x0, x20
	add	x1, x1, :lo12:.LC4
	bl	sprintf
	add	x20, x20, x0, sxtw
	adrp	x1, .LC5
	ldr	w2, [x25,2044]
	mov	x0, x20
	add	x1, x1, :lo12:.LC5
	bl	sprintf
	add	x20, x20, x0, sxtw
	adrp	x0, .LANCHOR1+532
	ldr	w0, [x0,#:lo12:.LANCHOR1+532]
	cmp	w0, 1
	beq	.L589
	sub	w0, w20, w23
	b	.L590
.L589:
	add	x0, x29, 96
	add	x1, x29, 100
	add	x2, x29, 104
	add	x3, x29, 108
	adrp	x24, .LANCHOR2
	bl	NandcGetTimeCfg
	add	x22, x24, :lo12:.LANCHOR2
	ldr	w4, [x29,104]
	adrp	x1, .LC6
	ldr	w5, [x29,108]
	add	x1, x1, :lo12:.LC6
	ldr	w3, [x29,100]
	mov	x0, x20
	ldr	w2, [x29,96]
	bl	sprintf
	add	x21, x20, x0, sxtw
	adrp	x1, .LC7
	mov	x0, x21
	add	x1, x1, :lo12:.LC7
	add	x21, x21, 10
	bl	strcpy
	ldr	w2, [x25,2104]
	adrp	x1, .LC8
	mov	x0, x21
	add	x1, x1, :lo12:.LC8
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,156]
	adrp	x1, .LC9
	mov	x0, x21
	add	x1, x1, :lo12:.LC9
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,164]
	adrp	x1, .LC10
	mov	x0, x21
	add	x1, x1, :lo12:.LC10
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,168]
	adrp	x1, .LC11
	mov	x0, x21
	add	x1, x1, :lo12:.LC11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,172]
	adrp	x1, .LC12
	mov	x0, x21
	add	x1, x1, :lo12:.LC12
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,176]
	adrp	x1, .LC13
	mov	x0, x21
	add	x1, x1, :lo12:.LC13
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,180]
	adrp	x1, .LC14
	mov	x0, x21
	add	x1, x1, :lo12:.LC14
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,184]
	adrp	x1, .LC15
	mov	x0, x21
	add	x1, x1, :lo12:.LC15
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,188]
	adrp	x1, .LC16
	mov	x0, x21
	add	x1, x1, :lo12:.LC16
	lsr	w2, w2, 11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,192]
	adrp	x1, .LC17
	mov	x0, x21
	add	x1, x1, :lo12:.LC17
	lsr	w2, w2, 11
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,196]
	adrp	x1, .LC18
	mov	x0, x21
	add	x1, x1, :lo12:.LC18
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,200]
	adrp	x1, .LC19
	add	x1, x1, :lo12:.LC19
	mov	x0, x21
	bl	sprintf
	add	x21, x21, x0, sxtw
	bl	FtlBbtCalcTotleCnt
	uxth	w3, w0
	ldrh	w2, [x25,2118]
	adrp	x1, .LC20
	mov	x0, x21
	add	x1, x1, :lo12:.LC20
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,-8]
	adrp	x1, .LC21
	mov	x0, x21
	add	x1, x1, :lo12:.LC21
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,204]
	adrp	x1, .LC22
	mov	x0, x21
	add	x1, x1, :lo12:.LC22
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,208]
	adrp	x1, .LC23
	mov	x0, x21
	add	x1, x1, :lo12:.LC23
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,212]
	adrp	x1, .LC24
	mov	x0, x21
	add	x1, x1, :lo12:.LC24
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,-72]
	adrp	x1, .LC25
	mov	x0, x21
	add	x1, x1, :lo12:.LC25
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,216]
	adrp	x1, .LC26
	mov	x0, x21
	add	x1, x1, :lo12:.LC26
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x22,220]
	adrp	x1, .LC27
	mov	x0, x21
	add	x1, x1, :lo12:.LC27
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,254]
	adrp	x1, .LC28
	mov	x0, x21
	add	x1, x1, :lo12:.LC28
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,252]
	adrp	x1, .LC29
	mov	x0, x21
	add	x1, x1, :lo12:.LC29
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,2068]
	adrp	x1, .LC30
	mov	x0, x21
	add	x1, x1, :lo12:.LC30
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,2060]
	adrp	x1, .LC31
	mov	x0, x21
	add	x1, x1, :lo12:.LC31
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,1948]
	adrp	x1, .LC32
	mov	x0, x21
	add	x1, x1, :lo12:.LC32
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x25,2214]
	adrp	x1, .LC33
	mov	x0, x21
	add	x1, x1, :lo12:.LC33
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x25,1960]
	adrp	x1, .LC34
	mov	x0, x21
	add	x1, x1, :lo12:.LC34
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,272]
	adrp	x1, .LC35
	mov	x0, x21
	add	x1, x1, :lo12:.LC35
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w2, [x25,1964]
	adrp	x1, .LC36
	mov	x0, x21
	add	x1, x1, :lo12:.LC36
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,280]
	adrp	x1, .LC37
	mov	x0, x21
	add	x1, x1, :lo12:.LC37
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x25,2112]
	adrp	x1, .LC38
	mov	x0, x21
	add	x1, x1, :lo12:.LC38
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,2]
	adrp	x1, .LC39
	mov	x0, x21
	add	x1, x1, :lo12:.LC39
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,6]
	adrp	x1, .LC40
	mov	x0, x21
	add	x1, x1, :lo12:.LC40
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x24,#:lo12:.LANCHOR2]
	adrp	x1, .LC41
	mov	x0, x21
	add	x1, x1, :lo12:.LC41
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,8]
	adrp	x1, .LC42
	mov	x0, x21
	add	x1, x1, :lo12:.LC42
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,4]
	adrp	x1, .LC43
	mov	x0, x21
	add	x1, x1, :lo12:.LC43
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w3, [x24,#:lo12:.LANCHOR2]
	adrp	x1, .LC44
	ldr	x2, [x22,-40]
	mov	x0, x21
	add	x1, x1, :lo12:.LC44
	ldrh	w2, [x2,x3,lsl 1]
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,50]
	adrp	x1, .LC45
	mov	x0, x21
	add	x1, x1, :lo12:.LC45
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,54]
	adrp	x1, .LC46
	mov	x0, x21
	add	x1, x1, :lo12:.LC46
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,48]
	adrp	x1, .LC47
	mov	x0, x21
	add	x1, x1, :lo12:.LC47
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,56]
	adrp	x1, .LC48
	mov	x0, x21
	add	x1, x1, :lo12:.LC48
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,52]
	adrp	x1, .LC49
	mov	x0, x21
	add	x1, x1, :lo12:.LC49
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w3, [x22,48]
	adrp	x1, .LC50
	ldr	x2, [x22,-40]
	mov	x0, x21
	add	x1, x1, :lo12:.LC50
	ldrh	w2, [x2,x3,lsl 1]
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,98]
	adrp	x1, .LC51
	mov	x0, x21
	add	x1, x1, :lo12:.LC51
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,102]
	adrp	x1, .LC52
	mov	x0, x21
	add	x1, x1, :lo12:.LC52
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,96]
	adrp	x1, .LC53
	mov	x0, x21
	add	x1, x1, :lo12:.LC53
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,104]
	adrp	x1, .LC54
	mov	x0, x21
	add	x1, x1, :lo12:.LC54
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,100]
	adrp	x1, .LC55
	mov	x0, x21
	add	x1, x1, :lo12:.LC55
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,298]
	adrp	x1, .LC56
	mov	x0, x21
	add	x1, x1, :lo12:.LC56
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,302]
	adrp	x1, .LC57
	mov	x0, x21
	add	x1, x1, :lo12:.LC57
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,296]
	adrp	x1, .LC58
	mov	x0, x21
	add	x1, x1, :lo12:.LC58
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrb	w2, [x22,304]
	adrp	x1, .LC59
	mov	x0, x21
	add	x1, x1, :lo12:.LC59
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldrh	w2, [x22,300]
	adrp	x1, .LC60
	mov	x0, x21
	add	x1, x1, :lo12:.LC60
	bl	sprintf
	add	x21, x21, x0, sxtw
	ldr	w3, [x22,344]
	adrp	x1, .LC61
	ldr	w2, [x22,160]
	add	x1, x1, :lo12:.LC61
	ldr	w4, [x22,436]
	mov	x0, x21
	ldr	w5, [x22,428]
	orr	w2, w2, w3, lsl 8
	ldr	w3, [x22,432]
	bl	sprintf
	add	x19, x21, x0, sxtw
	ldr	w2, [x22,424]
	adrp	x1, .LC62
	mov	x0, x19
	add	x1, x1, :lo12:.LC62
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	w2, [x22,448]
	adrp	x1, .LC63
	mov	x0, x19
	add	x1, x1, :lo12:.LC63
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,864]
	adrp	x1, .LC64
	mov	x0, x19
	add	x1, x1, :lo12:.LC64
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,866]
	adrp	x1, .LC65
	mov	x0, x19
	add	x1, x1, :lo12:.LC65
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldr	w2, [x22,868]
	adrp	x1, .LC66
	mov	x0, x19
	add	x1, x1, :lo12:.LC66
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w2, [x22,872]
	adrp	x1, .LC67
	add	x1, x1, :lo12:.LC67
	mov	x0, x19
	bl	sprintf
	add	x19, x19, x0, sxtw
	bl	GetFreeBlockMinEraseCount
	uxth	w2, w0
	adrp	x1, .LC68
	mov	x0, x19
	add	x1, x1, :lo12:.LC68
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w0, [x22,-8]
	bl	GetFreeBlockMaxEraseCount
	uxth	w2, w0
	adrp	x1, .LC69
	mov	x0, x19
	add	x1, x1, :lo12:.LC69
	bl	sprintf
	add	x19, x19, x0, sxtw
	ldrh	w0, [x22,296]
	mov	w1, 65535
	cmp	w0, w1
	beq	.L591
	ubfiz	x2, x0, 1, 16
	ldr	x3, [x22,-40]
	adrp	x1, .LC70
	mov	x0, x19
	add	x1, x1, :lo12:.LC70
	ldrh	w2, [x3,x2]
	bl	sprintf
	add	x19, x19, x0, sxtw
.L591:
	mov	w0, 0
	adrp	x22, .LC71
	mov	w20, 0
	mov	w27, 65535
	bl	List_get_gc_head_node
	mov	w26, 6
	uxth	w3, w0
	add	x22, x22, :lo12:.LC71
.L593:
	cmp	w3, w27
	beq	.L592
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
	bne	.L593
.L592:
	add	x1, x24, :lo12:.LANCHOR2
	adrp	x22, .LC72
	mov	w20, 0
	mov	w27, 65535
	mov	w26, 6
	add	x22, x22, :lo12:.LC72
	ldr	x0, [x1,-16]
	ldr	x3, [x1,-56]
	sub	x3, x0, x3
	mov	x0, -6148914691236517206
	asr	x3, x3, 1
	madd	x3, x0, x3, x3
	uxth	w3, w3
.L595:
	cmp	w3, w27
	beq	.L594
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
	bne	.L595
.L594:
	sub	w0, w19, w23
.L590:
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
	adrp	x1, .LC73
	adrp	x2, .LC74
	add	x29, sp, 0
	add	x1, x1, :lo12:.LC73
	add	x2, x2, :lo12:.LC74
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
	bcs	.L600
	adrp	x1, .LANCHOR0
	str	wzr, [x0,204]
	add	x2, x1, :lo12:.LANCHOR0
	ldr	x5, [x0,-80]
	mov	x0, 0
	ldrh	w3, [x2,1960]
.L601:
	cmp	w3, w0
	bls	.L642
	add	x4, x19, :lo12:.LANCHOR2
	ldrh	w6, [x5,x0,lsl 1]
	add	x0, x0, 1
	ldr	w2, [x4,204]
	add	w2, w6, w2
	str	w2, [x4,204]
	b	.L601
.L642:
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
	b	.L603
.L600:
	ldr	w1, [x0,216]
	cmp	w2, w1
	bls	.L603
	add	w1, w1, 1
	adrp	x4, .LANCHOR0
	str	w1, [x0,216]
	mov	w0, 0
.L605:
	add	x1, x4, :lo12:.LANCHOR0
	ldrh	w1, [x1,1960]
	cmp	w0, w1
	bcs	.L603
	add	x1, x19, :lo12:.LANCHOR2
	ubfiz	x2, x0, 1, 32
	add	w0, w0, 1
	ldr	x3, [x1,-80]
	ldrh	w1, [x3,x2]
	add	w1, w1, 1
	strh	w1, [x3,x2]
	b	.L605
.L603:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w22, [x0,220]
	ldr	w23, [x0,212]
	add	w1, w22, 256
	cmp	w1, w23
	bls	.L608
	ldr	w2, [x0,216]
	add	w1, w22, 768
	cmp	w1, w2
	bls	.L608
	ldr	w0, [x0,160]
	cbz	w0, .L611
	cmp	w22, 30
	bls	.L608
.L611:
	mov	w0, 65535
	b	.L610
.L608:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w0, [x0,-8]
	add	w0, w0, w0, lsl 1
	lsr	w0, w0, 2
	bl	GetFreeBlockMaxEraseCount
	uxth	w6, w0
	add	w0, w22, 64
	cmp	w6, w0
	bcs	.L622
	cmp	w22, 30
	bhi	.L611
.L622:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x3, [x0,-48]
	cbz	x3, .L611
	mov	w24, 65535
	ldr	x5, [x0,-56]
	ldr	x26, [x0,-80]
	mov	x0, -6148914691236517206
	mov	w20, w24
	mov	w7, w24
	add	x0, x0, 1
	mov	w8, 6
.L613:
	ldrh	w1, [x3]
	cmp	w1, w7
	beq	.L615
	ldrh	w2, [x3,4]
	cbz	w2, .L614
	sub	x4, x3, x5
	asr	x4, x4, 1
	mul	x4, x4, x0
	uxth	w3, w4
	and	x4, x4, 65535
	ldrh	w2, [x26,x4,lsl 1]
	cmp	w2, w22
	bls	.L621
	cmp	w2, w24
	bcs	.L614
	mov	w24, w2
	mov	w20, w3
.L614:
	umull	x3, w1, w8
	add	x3, x5, x3
	b	.L613
.L621:
	mov	w20, w3
.L615:
	mov	w0, 65535
	cmp	w20, w0
	beq	.L611
	ubfiz	x25, x20, 1, 16
	ldrh	w21, [x26,x25]
	cmp	w21, w22
	bls	.L617
	str	x6, [x29,88]
	bl	GetFreeBlockMinEraseCount
	ldr	x6, [x29,88]
	cmp	w22, w0, uxth
	bcs	.L617
	add	x0, x19, :lo12:.LANCHOR2
	str	w24, [x0,220]
.L617:
	cmp	w21, 29
	bhi	.L618
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,160]
	cbz	w0, .L618
	add	w0, w21, 10
	cmp	w0, w23
	bls	.L619
	adrp	x0, .LANCHOR0+2016
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2016]
	cmp	w0, w21
	bls	.L618
.L619:
	add	x22, x19, :lo12:.LANCHOR2
	ldrh	w0, [x22,-24]
	cmp	w0, 64
	bls	.L618
	adrp	x0, .LC75
	ldrh	w2, [x26,x25]
	add	x0, x0, :lo12:.LC75
	mov	w1, w20
	bl	printk
	mov	w0, 1
	str	w0, [x22,876]
	b	.L641
.L618:
	cmp	w21, w23
	bcs	.L611
	add	w0, w21, 128
	cmp	w6, w0
	ble	.L611
	add	w0, w21, 256
	cmp	w0, w23
	bcc	.L620
	add	x0, x19, :lo12:.LANCHOR2
	add	w21, w21, 768
	ldr	w0, [x0,216]
	cmp	w21, w0
	bcs	.L611
.L620:
	add	x19, x19, :lo12:.LANCHOR2
	adrp	x0, .LC76
	ldrh	w5, [x26,x25]
	add	x0, x0, :lo12:.LC76
	mov	w1, w20
	mov	w2, w23
	ldr	x4, [x19,-40]
	ldr	w3, [x19,216]
	ldrh	w4, [x4,x25]
	bl	printk
	mov	w0, 1
	str	w0, [x19,876]
.L641:
	mov	w0, w20
.L610:
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
	beq	.L644
	adrp	x2, .LANCHOR2-40
	ubfiz	x1, x0, 1, 16
	ldr	x2, [x2,#:lo12:.LANCHOR2-40]
	strh	wzr, [x2,x1]
	bl	INSERT_FREE_LIST
.L644:
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
.L646:
	add	x5, x10, :lo12:.LANCHOR0
	adrp	x9, .LANCHOR0
	ldrh	w0, [x5,1952]
	cmp	w1, w0
	bcs	.L650
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
	b	.L646
.L650:
	mov	w8, 24
	mov	w5, 4
.L648:
	add	x2, x3, :lo12:.LANCHOR2
	ldr	w1, [x2,920]
	cmp	w0, w1
	bcs	.L651
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
	b	.L648
.L651:
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
.L653:
	cmp	w3, w1
	bcs	.L652
	umull	x4, w3, w7
	mov	w2, 0
	add	x4, x0, x4
.L658:
	cmp	w2, w8
	bcs	.L655
	umull	x6, w2, w9
	add	x10, x5, x6
	ldr	x11, [x5,x6]
	ldr	x6, [x4,8]
	cmp	x11, x6
	bne	.L654
	str	wzr, [x10,16]
	b	.L655
.L654:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L658
.L655:
	add	w3, w3, 1
	uxth	w3, w3
	b	.L653
.L652:
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
.L660:
	cmp	w2, w1
	bcs	.L666
	mov	w3, 0
.L664:
	cmp	w3, w5
	bcs	.L662
	umull	x4, w3, w7
	add	x4, x6, x4
	ldr	w10, [x4,16]
	cbnz	w10, .L661
	umull	x3, w2, w9
	str	w8, [x4,16]
	add	x3, x0, x3
	ldr	x10, [x4]
	str	x10, [x3,8]
	ldr	x4, [x4,8]
	str	x4, [x3,16]
	b	.L662
.L661:
	add	w3, w3, 1
	uxth	w3, w3
	b	.L664
.L662:
	add	w2, w2, 1
	uxth	w2, w2
	b	.L660
.L666:
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
.L668:
	cmp	w2, w1, uxth
	bls	.L672
	add	x1, x1, 1
	add	x4, x3, x1, lsl 1
	ldrh	w4, [x4,-2]
	cmp	w4, w0
	bne	.L668
	mov	w0, 1
	b	.L669
.L672:
	mov	w0, 0
.L669:
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
.L674:
	uxth	w4, w5
	cmp	w4, w7
	bcs	.L678
	add	x5, x5, 1
	add	x9, x6, x5, lsl 1
	ldrh	w9, [x9,-2]
	cmp	w9, w8
	bne	.L674
.L678:
	cmp	w4, w7
	bne	.L676
	ubfiz	x4, x4, 1, 16
	strh	w0, [x6,x4]
	add	x0, x3, :lo12:.LANCHOR2
	ldrh	w4, [x0,924]
	add	w4, w4, 1
	strh	w4, [x0,924]
.L676:
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
	adrp	x0, .LC77
	mov	w1, w19
	add	x0, x0, :lo12:.LC77
	bl	printk
	adrp	x2, .LANCHOR2
	mov	w0, 65535
	add	x2, x2, :lo12:.LANCHOR2
	ldrh	w1, [x2,952]
	cmp	w1, w0
	bne	.L681
	strh	w19, [x2,952]
	b	.L682
.L681:
	ldrh	w1, [x2,954]
	cmp	w1, w0
	bne	.L682
	strh	w19, [x2,954]
.L682:
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
	adrp	x0, .LC78
	mov	w2, w20
	add	x0, x0, :lo12:.LC78
	bl	printk
	mov	w0, w21
	bl	FtlGcRefreshBlock
	ldr	w0, [x22,160]
	cbz	w0, .L684
	ubfiz	x0, x21, 1, 16
	ldr	x2, [x22,-80]
	ldrh	w1, [x2,x0]
	cmp	w1, 29
	bls	.L684
	sub	w1, w1, #30
	strh	w1, [x2,x0]
.L684:
	add	x2, x19, :lo12:.LANCHOR2
	mov	x0, 0
	add	x2, x2, 960
	ldrh	w1, [x2,-4]
.L685:
	cmp	w1, w0, uxth
	bls	.L691
	add	x0, x0, 1
	add	x3, x2, x0, lsl 1
	ldrh	w3, [x3,-2]
	cmp	w3, w20
	bne	.L685
	b	.L686
.L691:
	cmp	w1, 15
	bhi	.L686
	add	x19, x19, :lo12:.LANCHOR2
	add	w0, w1, 1
	add	x19, x19, 960
	strh	w0, [x19,-4]
	strh	w20, [x19,w1,sxtw 1]
.L686:
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
	cbz	w1, .L693
	ldrh	w3, [x0,952]
	mov	w2, 65535
	cmp	w3, w2
	bne	.L693
	ldrh	w2, [x0,994]
	cmp	w2, w1
	bcc	.L694
	strh	wzr, [x0,994]
.L694:
	add	x19, x19, :lo12:.LANCHOR2
	add	x0, x19, 960
	ldrh	w1, [x19,994]
	ldrh	w0, [x0,w1,sxtw 1]
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	ldrh	w0, [x19,994]
	add	w0, w0, 1
	strh	w0, [x19,994]
.L693:
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
	cbnz	w0, .L700
	adrp	x1, .LANCHOR0
	mov	x0, 0
	add	x1, x1, :lo12:.LANCHOR0
.L701:
	add	x2, x1, 196
	strh	w0, [x2,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L701
.L707:
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
	b	.L702
.L700:
	cmp	w0, 1
	bne	.L703
	adrp	x3, .LANCHOR0
	mov	x0, 0
	mov	w4, 3
	mov	w5, 2
	add	x3, x3, :lo12:.LANCHOR0
.L706:
	cmp	x0, 3
	uxth	w1, w0
	mov	w2, w1
	bls	.L704
	ubfiz	w2, w1, 1, 15
	and	w1, w1, 1
	cmp	w1, wzr
	csel	w1, w4, w5, ne
	sub	w2, w2, w1
	uxth	w2, w2
.L704:
	add	x1, x3, 196
	strh	w2, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L706
	b	.L707
.L703:
	cmp	w0, 2
	bne	.L708
	adrp	x3, .LANCHOR0
	mov	w1, 65535
	mov	x0, 0
	add	x3, x3, :lo12:.LANCHOR0
.L710:
	add	x4, x3, 196
	cmp	x0, 1
	uxth	w2, w0
	csel	w2, w1, w2, hi
	add	w1, w1, 2
	strh	w2, [x4,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	uxth	w1, w1
	bne	.L710
	b	.L707
.L708:
	cmp	w0, 3
	bne	.L711
	adrp	x3, .LANCHOR0
	mov	x0, 0
	mov	w4, 5
	mov	w5, 4
	add	x3, x3, :lo12:.LANCHOR0
.L714:
	cmp	x0, 5
	uxth	w1, w0
	mov	w2, w1
	bls	.L712
	ubfiz	w2, w1, 1, 15
	and	w1, w1, 1
	cmp	w1, wzr
	csel	w1, w4, w5, ne
	sub	w2, w2, w1
	uxth	w2, w2
.L712:
	add	x1, x3, 196
	strh	w2, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 256
	bne	.L714
	b	.L707
.L711:
	cmp	w0, 4
	bne	.L715
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
.L717:
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
	bne	.L717
	b	.L707
.L715:
	cmp	w0, 5
	bne	.L718
	adrp	x2, .LANCHOR0
	mov	x0, 0
	add	x2, x2, :lo12:.LANCHOR0
.L719:
	add	x1, x2, 196
	strh	w0, [x1,x0,lsl 1]
	add	x0, x0, 1
	cmp	x0, 16
	bne	.L719
	mov	x0, 0
.L720:
	add	x2, x1, x0
	add	w3, w0, 16
	add	x0, x0, 2
	cmp	x0, 480
	strh	w3, [x2,32]
	bne	.L720
	b	.L707
.L718:
	cmp	w0, 6
	bne	.L707
	adrp	x1, .LANCHOR0
	mov	x2, 0
	mov	w4, 12
	mov	w5, 10
	add	x1, x1, :lo12:.LANCHOR0
.L723:
	cmp	x2, 5
	uxth	w3, w2
	mov	w0, w3
	bls	.L721
	add	w0, w3, w3, lsl 1
	and	w3, w3, 1
	cmp	w3, wzr
	uxth	w0, w0
	csel	w3, w4, w5, ne
	sub	w0, w0, w3
	uxth	w0, w0
.L721:
	add	x3, x1, 196
	strh	w0, [x3,x2,lsl 1]
	add	x2, x2, 1
	cmp	x2, 256
	bne	.L723
	b	.L707
.L702:
	cmp	w20, w0, uxth
	bls	.L741
	add	x2, x1, 196
	ldrh	w2, [x2,x0,lsl 1]
	add	x0, x0, 1
	strh	w2, [x19,w2,sxtw 1]
	b	.L702
.L741:
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
	bls	.L743
	mov	w1, 512
	b	.L759
.L743:
	cmp	w1, 128
	bls	.L759
	mov	w1, 256
.L759:
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
.L747:
	ldrb	w2, [x20]
	mov	x0, x23
	add	x1, x26, x24, lsl 3
	bl	FlashMemCmp8
	cbnz	w0, .L746
	ldrb	w1, [x19,1845]
	str	w0, [x25,w1,sxtw 2]
	add	w0, w1, 1
	add	x1, x19, x1, sxtw
	strb	w0, [x19,1845]
	strb	w24, [x1,1848]
.L746:
	add	x24, x24, 1
	cmp	x24, 4
	bne	.L747
	add	x19, x21, :lo12:.LANCHOR0
	add	x0, x22, :lo12:.LANCHOR2
	ldrb	w1, [x19,1845]
	strb	w1, [x0,2020]
	ldrb	w0, [x20,8]
	cmp	w0, 2
	beq	.L748
.L752:
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
.L748:
	ldr	w26, [x19,12]
	mov	x24, 0
	add	x27, x19, 1620
	add	x25, x19, 708
.L751:
	ldrb	w2, [x20]
	mov	x0, x23
	add	x1, x27, x24, lsl 3
	bl	FlashMemCmp8
	cbnz	w0, .L749
	ldrb	w1, [x20,13]
	ldrb	w3, [x19,1845]
	mul	w0, w1, w26
	ldrh	w1, [x20,14]
	mov	x2, x3
	and	w1, w1, 65280
	mul	w1, w0, w1
	str	w1, [x25,w3,sxtw 2]
	ldrb	w0, [x20,23]
	cbz	w0, .L750
	lsl	w1, w1, 1
	str	w1, [x25,w3,sxtw 2]
.L750:
	add	x0, x19, x3
	add	w2, w2, 1
	strb	w2, [x19,1845]
	strb	w24, [x0,1848]
.L749:
	add	x24, x24, 1
	cmp	x24, 4
	bne	.L751
	b	.L752
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
.L761:
	cmp	w4, w0, uxtb
	bls	.L763
	add	x2, x1, 1848
	ldrb	w3, [x19,10]
	ldrb	w2, [x0,x2]
	add	x0, x0, 1
	lsl	w2, w5, w2
	orr	w2, w2, w3
	strb	w2, [x19,10]
	b	.L761
.L763:
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
	bls	.L765
	str	w1, [x0,2040]
.L765:
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
.L766:
	cmp	w0, w5
	add	x1, x1, 8
	bcs	.L866
	ldr	x3, [x20,2144]
	add	w0, w0, 1
	add	x3, x3, x2, uxtw 2
	add	w2, w2, w4
	str	x3, [x1,32]
	b	.L766
.L866:
	add	x1, x22, :lo12:.LANCHOR0
.L768:
	cmp	w0, 8
	beq	.L867
	add	x2, x1, x0, uxtw 3
	add	w0, w0, 1
	str	xzr, [x2,2144]
	b	.L768
.L867:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x1, [x0,2168]
	cbnz	x1, .L770
.L772:
	adrp	x0, .LC79
	adrp	x1, .LANCHOR3
	add	x0, x0, :lo12:.LC79
	add	x1, x1, :lo12:.LANCHOR3
	bl	printk
	mov	w0, -1
	b	.L771
.L770:
	ldr	x1, [x0,2176]
	cbz	x1, .L772
	ldr	x1, [x0,2208]
	cbz	x1, .L772
	ldr	x1, [x0,2216]
	cbz	x1, .L772
	ldr	x1, [x0,144]
	cbz	x1, .L772
	ldr	x1, [x0,2224]
	cbz	x1, .L772
	ldr	x1, [x0,-56]
	cbz	x1, .L772
	add	x22, x22, :lo12:.LANCHOR0
	ldr	x1, [x22,2144]
	cbz	x1, .L772
	ldr	x0, [x0,-40]
	cbz	x0, .L772
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x1, [x0,928]
	cbz	x1, .L772
	ldr	x1, [x0,944]
	cbz	x1, .L772
	ldr	x1, [x0,2048]
	cbz	x1, .L772
	ldr	x1, [x0,2064]
	cbz	x1, .L772
	ldr	x1, [x0,-88]
	cbz	x1, .L772
	ldr	x1, [x0,912]
	cbz	x1, .L772
	ldr	x1, [x0,2056]
	cbz	x1, .L772
	ldr	x1, [x0,-64]
	cbz	x1, .L772
	ldr	x1, [x0,2072]
	cbz	x1, .L772
	ldr	x0, [x0,2080]
	cbz	x0, .L772
	add	x0, x21, :lo12:.LANCHOR2
	ldr	x1, [x0,896]
	cbz	x1, .L772
	ldr	x1, [x0,2096]
	cbz	x1, .L772
	ldr	x1, [x0,2104]
	cbz	x1, .L772
	ldr	x1, [x0,888]
	cbz	x1, .L772
	ldr	x1, [x0,2112]
	cbz	x1, .L772
	ldr	x1, [x0,2120]
	cbz	x1, .L772
	ldr	x1, [x0,904]
	cbz	x1, .L772
	ldr	x1, [x0,-80]
	cbz	x1, .L772
	ldr	x0, [x0,2144]
	cbz	x0, .L772
	adrp	x0, .LANCHOR0+2080
	ldr	x0, [x0,#:lo12:.LANCHOR0+2080]
	cbz	x0, .L772
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	ldr	x1, [x0,2184]
	cbz	x1, .L772
	ldr	x1, [x0,2192]
	cbz	x1, .L772
	ldr	x0, [x0,2200]
	cbz	x0, .L772
	mov	w0, 0
.L771:
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
.L870:
	ldrh	w2, [x20,x3]
	cmp	w2, w0
	beq	.L868
	ubfx	x4, x2, 5, 11
	add	x3, x3, 2
	lsl	x4, x4, 2
	lsl	w2, w5, w2
	cmp	x3, 1024
	ldr	w1, [x19,x4]
	orr	w2, w1, w2
	str	w2, [x19,x4]
	bne	.L870
.L868:
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
.L876:
	ldrh	w1, [x19,6]
	cmp	w1, w0
	bls	.L889
	ubfiz	x1, x0, 2, 16
	ldr	w2, [x20,x1]
	mov	w1, 0
	ubfx	x2, x2, 10, 16
.L877:
	ldrh	w3, [x19,10]
	cmp	w3, w1
	bls	.L890
	ubfiz	x3, x1, 1, 16
	ldrh	w4, [x25,x3]
	cmp	w4, w2
	bne	.L878
	ldrh	w4, [x21,x3]
	add	w4, w4, 1
	strh	w4, [x21,x3]
.L878:
	add	w1, w1, 1
	uxth	w1, w1
	b	.L877
.L890:
	add	w0, w0, 1
	uxth	w0, w0
	b	.L876
.L889:
	mov	w24, 0
	ldrh	w26, [x21]
	mov	w20, w24
	adrp	x27, .LANCHOR0
.L881:
	ldrh	w0, [x19,10]
	cmp	w0, w20
	bls	.L891
	ldrh	w0, [x19]
	uxtw	x22, w20
	cmp	w0, w20
	bne	.L882
	add	x0, x27, :lo12:.LANCHOR0
	ldrh	w1, [x19,2]
	ldrh	w0, [x0,2028]
	cmp	w1, w0
	bcs	.L882
	strh	w0, [x21,x22,lsl 1]
.L882:
	lsl	x22, x22, 1
	ldrh	w23, [x21,x22]
	cmp	w26, w23
	bls	.L883
	mov	w24, w20
	mov	w26, w23
.L883:
	cbnz	w23, .L884
	ldrh	w0, [x25,x22]
	cbz	w0, .L884
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
	strh	w23, [x25,x22]
	ldrh	w0, [x19,8]
	sub	w0, w0, #1
	strh	w0, [x19,8]
.L884:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L881
.L891:
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
.L893:
	add	x3, x20, :lo12:.LANCHOR0
	ldrh	w0, [x3,2066]
	cmp	w0, w1
	bls	.L895
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
	b	.L893
.L895:
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
.L898:
	add	x0, x24, :lo12:.LANCHOR0
	ldrh	w1, [x0,1960]
	cmp	w1, w19
	bls	.L904
	mov	x28, 0
	ldrh	w3, [x0,1952]
	ldrh	w2, [x0,2026]
	mov	w20, w28
	mov	x27, x0
.L905:
	cmp	w3, w28, uxth
	bls	.L909
	add	x0, x27, 1984
	mov	w1, w19
	str	x2, [x29,96]
	str	x3, [x29,104]
	ldrb	w0, [x0,x28]
	bl	V2P_block
	bl	FtlBbmIsBadBlock
	ldr	x3, [x29,104]
	ldr	x2, [x29,96]
	cbnz	w0, .L899
	add	w20, w20, w2
	uxth	w20, w20
.L899:
	add	x28, x28, 1
	b	.L905
.L909:
	cbz	w20, .L901
	mov	w0, 32768
	sdiv	w20, w0, w20
.L901:
	ldr	x1, [x21,-56]
	umull	x0, w19, w26
	add	x0, x1, x0
	strh	w20, [x0,4]
	ldrh	w0, [x21]
	cmp	w0, w19
	beq	.L902
	ldrh	w0, [x21,48]
	cmp	w0, w19
	beq	.L902
	ldrh	w0, [x21,96]
	cmp	w0, w19
	beq	.L902
	ubfiz	x0, x19, 1, 16
	ldr	x1, [x21,-40]
	ldrh	w0, [x1,x0]
	cbnz	w0, .L903
	add	w4, w25, 1
	mov	w0, w19
	uxth	w25, w4
	bl	INSERT_FREE_LIST
	b	.L902
.L903:
	add	w23, w23, 1
	mov	w0, w19
	uxth	w23, w23
	bl	INSERT_DATA_LIST
.L902:
	add	w19, w19, 1
	uxth	w19, w19
	b	.L898
.L904:
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
	cbz	w0, .L912
	sub	w2, w0, #1
	cmp	w2, 5
	bhi	.L914
	bl	HynixGetReadRetryDefault
	b	.L912
.L914:
	cmp	w0, 49
	bne	.L915
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
	b	.L930
.L915:
	sub	w0, w0, #65
	cmp	w0, 1
	bls	.L921
	cmp	w1, 33
	bne	.L916
.L921:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	strb	w1, [x0,760]
	mov	w1, 4
	b	.L931
.L916:
	cmp	w1, 67
	beq	.L922
	cmp	w1, 34
	bne	.L918
.L922:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	strb	w1, [x0,760]
	mov	w1, 5
.L931:
	strb	w1, [x0,761]
	mov	w1, 7
	strb	w1, [x0,762]
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	add	x0, x0, 764
	add	x1, x1, 256
	mov	w2, 45
	b	.L930
.L918:
	cmp	w1, 68
	beq	.L923
	cmp	w1, 35
	bne	.L912
.L923:
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
.L930:
	bl	ftl_memcpy
.L912:
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
.L937:
	ldrb	w2, [x21]
	add	x0, x21, 1
	mov	x1, x22
	bl	FlashMemCmp8
	mov	w23, w0
	cbnz	w0, .L934
	add	x0, x20, :lo12:.LANCHOR1
	ubfiz	x19, x19, 5, 32
	add	x1, x0, 536
	adds	x19, x1, x19
	beq	.L940
	ldrb	w3, [x19,22]
	mov	x2, 0
	mov	x1, x0
	b	.L939
.L934:
	add	w19, w19, 1
	add	x21, x21, 32
	cmp	w19, 72
	bne	.L937
	b	.L940
.L939:
	add	x4, x1, x2, lsl 5
	mov	w0, w2
	ldrb	w4, [x4,2840]
	cmp	w4, w3
	beq	.L938
	add	x2, x2, 1
	cmp	x2, 4
	bne	.L939
	mov	w0, w2
.L938:
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
	b	.L935
.L940:
	mov	w23, -1
.L935:
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
	bne	.L945
	cbz	x3, .L946
	mov	x0, x2
	mov	x1, x3
	mov	w2, 1024
	bl	ftl_memcpy
.L946:
	cbz	x20, .L944
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
	b	.L944
.L945:
	cbz	x3, .L949
	mov	x1, x2
	mov	x0, x3
	mov	w2, 1024
	bl	ftl_memcpy
.L949:
	cbz	x20, .L944
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
.L944:
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
	cbnz	w2, .L962
	add	x1, x4, x1
	b	.L964
.L962:
	add	x0, x4, x1
	mov	x1, x6
.L964:
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
	adrp	x23, .LC80
	adrp	x21, .LC81
	adrp	x24, .LC73
	stp	x25, x26, [sp,64]
	str	x27, [sp,80]
	mov	x22, x1
	mov	x27, x0
	mov	w25, w2
	uxtw	x26, w3
	mov	w20, w19
	add	x23, x23, :lo12:.LC80
	add	x21, x21, :lo12:.LC81
	add	x24, x24, :lo12:.LC73
.L971:
	cmp	x19, x26
	beq	.L979
	cbnz	w20, .L972
	mov	x0, x23
	mov	x1, x27
	mov	w2, w19
	bl	printk
.L972:
	cmp	w25, 4
	mov	x0, x21
	bne	.L973
	ldr	w1, [x22,x19,lsl 2]
	b	.L978
.L973:
	cmp	w25, 2
	bne	.L975
	ldrsh	w1, [x22,x19,lsl 1]
	b	.L978
.L975:
	ldrb	w1, [x22,x19]
.L978:
	bl	printk
	add	w20, w20, 1
	cmp	w20, 15
	bls	.L976
	adrp	x1, .LC82
	mov	x0, x24
	add	x1, x1, :lo12:.LC82
	mov	w20, 0
	bl	printk
.L976:
	add	x19, x19, 1
	b	.L971
.L979:
	adrp	x0, .LC73
	adrp	x1, .LC82
	add	x1, x1, :lo12:.LC82
	add	x0, x0, :lo12:.LC73
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
	ldr	w0, [x1,20]
	cmp	w0, 3
	bls	.L1011
	ldr	w0, [x19,16]
	tbz	x0, 2, .L1011
	mov	x0, x19
	bl	wait_for_nandc_xfer_completed
	ldr	w21, [x19,16]
	ldr	w0, [x19,8]
	ubfx	x21, x21, 1, 1
	str	w0, [x29,64]
	cbz	w21, .L982
	adrp	x22, .LC83
	adrp	x23, .LC84
	mov	w21, 0
	add	x22, x22, :lo12:.LC83
	add	x23, x23, :lo12:.LC84
.L983:
	ldr	w2, [x19,28]
	ldr	w1, [x29,64]
	ubfx	x2, x2, 16, 5
	ubfx	x1, x1, 22, 6
	cmp	w2, w1
	bge	.L991
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w0, [x0,20]
	cmp	w0, 5
	bhi	.L984
.L987:
	add	w21, w21, 1
	and	w0, w21, 16777215
	cbnz	w0, .L983
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
	b	.L983
.L984:
	ldr	w0, [x19]
	str	w0, [x29,72]
	ldr	w0, [x29,72]
	tbz	x0, 13, .L987
	ldr	w0, [x29,72]
	tbz	x0, 17, .L987
.L991:
	add	x19, x20, :lo12:.LANCHOR0
	add	x19, x19, 1888
	ldr	w0, [x19,40]
	cbz	w0, .L992
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
	b	.L992
.L982:
	adrp	x22, .LC85
	adrp	x23, .LC84
	add	x22, x22, :lo12:.LC85
	add	x23, x23, :lo12:.LC84
.L993:
	ldr	w0, [x29,64]
	tbnz	x0, 20, .L1019
	ldr	w0, [x19,8]
	add	w21, w21, 1
	str	w0, [x29,64]
	and	w0, w21, 16777215
	cbnz	w0, .L993
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
	b	.L993
.L1019:
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w0, [x0,1936]
	cbz	w0, .L996
	mov	x0, x19
	bl	NandcSendDumpDataStart
.L996:
	add	x21, x20, :lo12:.LANCHOR0
	add	x21, x21, 1888
	ldr	w0, [x21,40]
	cbz	w0, .L997
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
.L997:
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w0, [x0,1936]
	cbz	w0, .L992
	mov	x0, x19
	bl	NandcSendDumpDataDone
.L992:
	add	x20, x20, :lo12:.LANCHOR0
	str	wzr, [x20,1928]
	b	.L980
.L1011:
	ldr	w0, [x19,8]
	str	w0, [x29,64]
	ldr	w0, [x29,64]
	tbz	x0, 20, .L1011
.L980:
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
	cbnz	x0, .L1021
	cbnz	x4, .L1022
	add	x0, x29, 112
	mov	w1, 255
	mov	w2, 64
	add	x22, x29, 112
	bl	ftl_memset
.L1022:
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
	cbnz	w21, .L1023
	add	x1, x19, :lo12:.LANCHOR0
	ubfx	x2, x24, 1, 7
	mov	w3, 128
	add	x2, x22, x2, lsl 2
	ldr	w0, [x1,1940]
	cmp	w0, 25
	mov	w0, 64
	csel	w3, w0, w3, cc
	mov	w0, w21
.L1025:
	cmp	x22, x2
	add	w4, w0, w3
	beq	.L1067
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
	b	.L1025
.L1067:
	add	x0, x19, :lo12:.LANCHOR0
	lsr	w24, w24, 2
	ldr	w5, [x0,1940]
	ldr	w4, [x0,20]
	mov	w0, 0
	mov	w23, w0
.L1027:
	cmp	w0, w24
	bcs	.L1023
	cbz	w5, .L1023
	uxtw	x1, w0
	add	x1, x1, 8
	ldr	w1, [x20,x1,lsl 2]
	str	w1, [x29,104]
	ldr	w1, [x29,104]
	tbnz	x1, 2, .L1051
	ldr	w3, [x29,104]
	ubfx	x3, x3, 15, 1
	cbnz	w3, .L1051
	cmp	w4, 5
	bls	.L1029
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
	bls	.L1030
	ldr	w1, [x29,104]
	ubfx	x3, x3, 3, 5
	ubfx	x1, x1, 27, 1
	orr	w3, w3, w1, lsl 5
	b	.L1031
.L1030:
	ubfx	x1, x3, 16, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 29, 1
	b	.L1066
.L1029:
	cmp	w4, 3
	bls	.L1031
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
	bls	.L1032
	ubfx	x1, x3, 3, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 28, 1
	b	.L1066
.L1032:
	ubfx	x1, x3, 16, 5
	ldr	w3, [x29,104]
	ubfx	x3, x3, 30, 1
.L1066:
	orr	w3, w1, w3, lsl 5
.L1031:
	cmp	w23, w3
	csel	w23, w23, w3, cs
	b	.L1028
.L1051:
	mov	w23, -1
.L1028:
	add	w0, w0, 1
	b	.L1027
.L1023:
	str	wzr, [x20,16]
	b	.L1034
.L1021:
	cmp	w21, 1
	bne	.L1065
	mov	w23, 0
	mov	w28, 2
.L1035:
	cmp	w23, w24
	bcs	.L1068
	and	w27, w23, 3
	mov	x3, x26
	cbz	x26, .L1037
	ubfiz	x0, x23, 9, 23
	add	x3, x26, x0
.L1037:
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
	b	.L1035
.L1068:
	mov	w23, 0
	b	.L1034
.L1065:
	mov	w1, 0
	mov	x4, 0
	mov	w27, 0
	mov	w0, w25
	mov	w2, 2
	mov	w3, w1
	mov	x5, x4
	mov	w23, w27
	bl	NandcXferStart
.L1040:
	cmp	w27, w24
	bcs	.L1034
	mov	w0, w25
	add	w28, w27, 2
	bl	NandcXferComp
	ldr	w0, [x20,32]
	cmp	w28, w24
	str	w0, [x29,104]
	bcs	.L1041
	mov	x4, 0
	mov	w0, w25
	mov	w1, 0
	mov	w2, 2
	and	w3, w28, 3
	mov	x5, x4
	bl	NandcXferStart
.L1041:
	ldr	w0, [x29,104]
	tbnz	x0, 2, .L1056
	ldr	w0, [x29,104]
	ubfx	x1, x0, 3, 5
	ldr	w0, [x29,104]
	ubfx	x0, x0, 27, 1
	orr	w0, w1, w0, lsl 5
	cmp	w23, w0
	csel	w23, w23, w0, cs
	b	.L1042
.L1056:
	mov	w23, -1
.L1042:
	and	w2, w27, 3
	mov	x3, x26
	cbz	x26, .L1043
	ubfiz	x3, x27, 9, 23
	add	x3, x26, x3
.L1043:
	cmp	x22, xzr
	mov	w0, 2
	csel	w4, w0, wzr, ne
	mov	w1, 0
	mov	x0, x20
	mul	w4, w4, w27
	mov	w27, w28
	add	x4, x22, x4
	bl	NandcCopy1KB
	b	.L1040
.L1034:
	cbnz	w21, .L1046
	add	x19, x19, :lo12:.LANCHOR0
	ldr	w0, [x19,20]
	cmp	w0, 5
	bls	.L1046
	ldr	w0, [x20]
	mov	w1, 8192
	movk	w1, 0x2, lsl 16
	and	w1, w0, w1
	cmp	w1, 139264
	bne	.L1046
	orr	w0, w0, 131072
	mov	w23, -1
	str	w0, [x20]
.L1046:
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
	cbnz	w19, .L1070
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w5, [x0,9]
	ldr	w0, [x0,12]
	mul	w0, w5, w0
	cmp	w1, w0
	mov	w0, 4
	csel	w20, w20, w0, cs
.L1070:
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
	ldr	w0, [x19,20]
	cmp	w0, 8
	mov	w0, 12
	csel	w22, w22, w0, cc
	cbz	w4, .L1073
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
	bne	.L1074
.L1083:
	mov	w21, -1
	b	.L1075
.L1074:
	adrp	x0, .LC86
	mov	w1, w24
	add	x0, x0, :lo12:.LC86
	mov	w2, w21
	bl	printk
	cmp	w21, 9
	bhi	.L1076
	add	x19, x19, x23, sxtw 4
	ldr	x0, [x19,24]
	ldr	w1, [x0,3840]
	ldr	w1, [x0]
	orr	w1, w1, 131072
	str	w1, [x0]
.L1076:
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	ldr	w1, [x0,40]
	add	w1, w1, 1
	str	w1, [x0,40]
	cmp	w1, 2047
	bls	.L1075
	mov	x27, 0
	str	wzr, [x0,40]
	mov	x28, x27
.L1073:
	mov	w19, 0
	mov	w26, -1
	mov	w5, w19
	mov	w6, w19
	mov	w20, w19
.L1081:
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
	bhi	.L1077
	cmp	w0, 2
	bhi	.L1087
	add	w20, w20, 1
	cmp	w20, 9
	bls	.L1087
	sub	w19, w22, w20
	mov	w21, w0
	mov	w26, 0
	b	.L1079
.L1077:
	cmp	w6, w20
	bcs	.L1088
	cmp	w20, 7
	sub	w5, w19, w20
	bhi	.L1089
	mov	w6, w20
	b	.L1088
.L1087:
	mov	x27, 0
	mov	w19, w22
	mov	w21, w0
	mov	w26, 0
	mov	x28, x27
	b	.L1078
.L1088:
	mov	w20, 0
.L1078:
	add	w22, w22, 2
	cmp	w22, 69
	bls	.L1081
.L1079:
	cmp	w6, w20
	csel	w19, w19, w5, cc
	b	.L1080
.L1089:
	mov	w19, w5
.L1080:
	cbz	w19, .L1082
	adrp	x0, .LC87
	mov	w1, w19
	add	x0, x0, :lo12:.LC87
	bl	printk
	mov	w0, w19
	bl	NandcSetDdrPara
.L1082:
	cbz	w26, .L1075
	adrp	x0, .LC88
	mov	w1, w23
	add	x0, x0, :lo12:.LC88
	mov	w2, w24
	bl	printk
	cbz	w25, .L1083
	ldr	w0, [x29,124]
	lsr	w0, w0, 8
	bl	NandcSetDdrPara
.L1075:
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
	bne	.L1099
	adrp	x20, .LANCHOR0
	add	x25, x20, :lo12:.LANCHOR0
	ldrb	w26, [x25,16]
	cbnz	w26, .L1100
.L1102:
	add	x20, x20, :lo12:.LANCHOR0
	ldrb	w0, [x20,1864]
	cbz	w0, .L1099
	b	.L1117
.L1100:
	mov	w0, w21
	mov	w1, w22
	mov	x2, x24
	mov	x3, x23
	strb	wzr, [x25,16]
	bl	FlashReadRawPage
	strb	w26, [x25,16]
	cmn	w0, #1
	beq	.L1102
	mov	w19, w0
	b	.L1099
.L1117:
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
	beq	.L1103
	ldrb	w0, [x20,1944]
	cmp	w19, w0, lsr 1
	bls	.L1099
.L1103:
	lsr	w0, w25, 8
	bl	NandcSetDdrPara
.L1099:
	adrp	x0, .LANCHOR4+48
	cmn	w19, #1
	ldr	x4, [x0,#:lo12:.LANCHOR4+48]
	bne	.L1104
	cbz	x4, .L1104
	mov	w1, w22
	mov	x2, x24
	mov	x3, x23
	mov	w0, w21
	blr	x4
	mov	w19, w0
	adrp	x0, .LC89
	mov	w1, w19
	add	x0, x0, :lo12:.LC89
	mov	w2, w21
	mov	w3, w22
	bl	printk
.L1104:
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
	beq	.L1122
	cmn	w22, #1
	bne	.L1119
.L1122:
	add	x20, x19, :lo12:.LANCHOR0
	ldrb	w0, [x20,1844]
	tbz	x0, 0, .L1119
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
	strb	wzr, [x20,1864]
	b	.L1121
.L1119:
	add	x19, x19, :lo12:.LANCHOR0
	mov	w0, 1
	strb	w0, [x19,1864]
.L1121:
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
	stp	x23, x24, [sp,48]
	adrp	x24, .LANCHOR0
	adrp	x23, .LANCHOR4
	strb	w0, [x29,120]
	mov	w0, 40
	strb	w0, [x29,121]
	mov	w0, 24
	add	x1, x24, :lo12:.LANCHOR0
	strb	w0, [x29,122]
	mov	w0, 16
	strb	w0, [x29,123]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x27, x28, [sp,80]
	add	x0, x23, :lo12:.LANCHOR4
	adrp	x22, .LANCHOR1
	add	x21, x22, :lo12:.LANCHOR1
	ldr	x1, [x1,1856]
	add	x21, x21, 472
	mov	w20, 0
	str	x1, [x0,56]
	mov	w26, 4
	str	wzr, [x0,64]
	mov	w25, -1
	mov	w0, 0
	ldrh	w27, [x21,10]
	bl	flash_enter_slc_mode
.L1130:
	add	w28, w20, 1
	mov	x19, 0
.L1132:
	add	x0, x29, 120
	ldrb	w0, [x19,x0]
	bl	FlashBchSel
	add	x4, x24, :lo12:.LANCHOR0
	mov	w0, 0
	mov	w1, w20
	mov	x3, 0
	str	x4, [x29,104]
	ldr	x2, [x4,1856]
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L1131
	ldr	x4, [x29,104]
	mov	w0, 0
	mov	w1, w28
	mov	x3, 0
	ldr	x2, [x4,1856]
	bl	FlashReadRawPage
	cmn	w0, #1
	bne	.L1131
	add	x19, x19, 1
	cmp	x19, 4
	beq	.L1133
	b	.L1132
.L1134:
	mov	w1, 2036
	add	x0, x28, 12
	mov	w25, -1
	bl	JSHash
	ldr	w1, [x28,8]
	cmp	w1, w0
	beq	.L1143
.L1133:
	subs	w26, w26, #1
	add	w20, w20, w27
	bne	.L1130
	mov	w0, w26
	bl	flash_exit_slc_mode
	b	.L1142
.L1131:
	add	x19, x23, :lo12:.LANCHOR4
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	ldr	x28, [x19,56]
	ldr	w1, [x28]
	cmp	w1, w0
	bne	.L1133
	cbnz	w25, .L1134
	add	x22, x22, :lo12:.LANCHOR1
	ldrh	w0, [x22,482]
	udiv	w20, w20, w0
	add	w20, w20, 1
	str	w20, [x19,68]
	b	.L1142
.L1143:
	add	x1, x28, 160
	mov	w2, 32
	mov	x0, x21
	add	x25, x24, :lo12:.LANCHOR0
	bl	ftl_memcpy
	ldr	x1, [x19,56]
	mov	w2, 32
	add	x0, x25, 88
	add	x1, x1, 192
	bl	ftl_memcpy
	ldr	x1, [x19,56]
	mov	w2, 852
	add	x0, x25, 760
	add	x1, x1, 224
	bl	ftl_memcpy
	str	w20, [x19,64]
	ldr	x1, [x19,56]
	ldr	w0, [x1,1076]
	strb	w0, [x25,1864]
	ldrh	w0, [x21,10]
	udiv	w0, w20, w0
	add	w2, w0, 1
	cbz	w0, .L1136
	str	w2, [x19,68]
	b	.L1137
.L1136:
	mov	w0, 2
	str	w0, [x19,68]
.L1137:
	add	x0, x23, :lo12:.LANCHOR4
	ldrh	w1, [x1,14]
	mov	w25, 0
	strb	w1, [x0,72]
	b	.L1133
.L1142:
	mov	w0, w25
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
	ret
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
	stp	x29, x30, [sp, -128]!
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	uxtb	w24, w0
	mov	w23, 0
	mov	w0, w24
	stp	x19, x20, [sp,16]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	stp	x21, x22, [sp,32]
	mov	w25, w1
	mov	x27, x2
	mov	x26, x3
	adrp	x21, .LANCHOR0
	bl	NandcWaitFlashReady
	mov	w19, -1
	add	x0, x21, :lo12:.LANCHOR0
	sbfiz	x1, x24, 4, 32
	add	x0, x0, 24
	adrp	x4, .LANCHOR4
	add	x2, x0, x1
	mov	w6, 239
	mov	w7, 137
	ldr	x28, [x0,x1]
	ldrb	w20, [x2,8]
	lsl	x5, x20, 8
.L1187:
	add	x0, x4, :lo12:.LANCHOR4
	ldrb	w0, [x0,73]
	cmp	w23, w0
	bcs	.L1190
	add	x22, x28, x5
	mov	w0, 200
	str	x4, [x29,96]
	add	w23, w23, 1
	str	x5, [x29,104]
	str	w6, [x22,2056]
	str	w7, [x22,2052]
	str	x6, [x29,112]
	str	x7, [x29,120]
	bl	NandcDelayns
	str	w23, [x22,2048]
	mov	w0, w24
	str	wzr, [x22,2048]
	mov	w1, w25
	str	wzr, [x22,2048]
	mov	x2, x27
	str	wzr, [x22,2048]
	mov	x3, x26
	bl	FlashReadRawPage
	cmn	w0, #1
	ldr	x7, [x29,120]
	ldr	x6, [x29,112]
	ldr	x5, [x29,104]
	ldr	x4, [x29,96]
	beq	.L1187
	add	x1, x21, :lo12:.LANCHOR0
	cmn	w19, #1
	csel	w19, w19, w0, ne
	ldrb	w2, [x1,1944]
	add	w2, w2, w2, lsl 1
	cmp	w0, w2, lsr 2
	bcc	.L1193
	mov	x26, 0
	mov	x27, x26
	b	.L1187
.L1193:
	mov	w19, w0
.L1190:
	add	x20, x28, x20, lsl 8
	mov	w0, 239
	add	x21, x21, :lo12:.LANCHOR0
	str	w0, [x20,2056]
	mov	w0, 137
	str	w0, [x20,2052]
	mov	w0, 200
	bl	NandcDelayns
	str	wzr, [x20,2048]
	ldrb	w0, [x21,1944]
	str	wzr, [x20,2048]
	str	wzr, [x20,2048]
	add	w0, w0, w0, lsl 1
	str	wzr, [x20,2048]
	cmp	w19, w0, lsr 2
	bcc	.L1192
	cmn	w19, #1
	mov	w0, 256
	csel	w19, w19, w0, eq
.L1192:
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 128
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
.L1200:
	cmp	w4, w25
	bcs	.L1204
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
	beq	.L1202
	ldrb	w1, [x28,1944]
	cmn	w19, #1
	csel	w19, w19, w0, ne
	add	w1, w1, w1, lsl 1
	cmp	w0, w1, lsr 2
	bcc	.L1207
	mov	x24, 0
	mov	x26, x24
.L1202:
	add	w4, w4, 1
	b	.L1200
.L1207:
	mov	w19, w0
.L1204:
	add	x21, x21, :lo12:.LANCHOR0
	add	x22, x21, x22
	ldrb	w0, [x21,1944]
	strb	w20, [x22,772]
	add	w0, w0, w0, lsl 1
	cmp	w19, w0, lsr 2
	bcc	.L1206
	cmn	w19, #1
	mov	w0, 256
	csel	w19, w19, w0, eq
.L1206:
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
	cbnz	w19, .L1214
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w1, [x0,9]
	ldr	w2, [x0,12]
	mul	w1, w1, w2
	cmp	w20, w1
	bcs	.L1214
	ldrb	w0, [x0,8]
	cbnz	w0, .L1215
	sub	w21, w21, #2
	b	.L1214
.L1215:
	mov	w21, 4
.L1214:
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
.L1219:
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
	beq	.L1217
	ldr	x25, [x24,56]
	mov	w0, 20036
	movk	w0, 0x4e41, lsl 16
	ldr	w1, [x25]
	cmp	w1, w0
	bne	.L1217
	mov	w1, 2036
	add	x0, x25, 12
	bl	JSHash
	ldr	w1, [x25,8]
	cmp	w1, w0
	bne	.L1217
	ldr	w0, [x20,12]
	cmp	w21, 1
	str	w26, [x24,68]
	mul	w19, w19, w0
	str	w19, [x24,64]
	beq	.L1220
	mov	w21, 1
.L1217:
	cmp	w26, 4
	mov	w19, w26
	bne	.L1219
	b	.L1218
.L1220:
	mov	w21, 2
.L1218:
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
	ldr	w1, [x0,1872]
	ldrb	w27, [x0,1944]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w1, w0
	bne	.L1226
	mov	w0, 0
	bl	flash_enter_slc_mode
.L1226:
	adrp	x24, .LC90
	mov	x0, x26
	mov	w1, 0
	mov	w2, 2048
	mov	w23, -1
	mov	w20, 2
	add	x24, x24, :lo12:.LC90
	adrp	x28, .LANCHOR4
	bl	ftl_memset
.L1227:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,9]
	cmp	w20, w0
	bcs	.L1231
	mov	x22, 0
.L1229:
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
	bne	.L1228
	add	x22, x22, 1
	cmp	x22, 4
	bne	.L1229
	b	.L1230
.L1228:
	ldr	x0, [x21,1856]
	ldr	w1, [x0]
	mov	w0, 35899
	movk	w0, 0xfcdc, lsl 16
	cmp	w1, w0
	bne	.L1230
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
	bls	.L1234
	mov	w23, 0
	str	w20, [x0,68]
	bl	FlashSavePhyInfo
.L1230:
	add	w20, w20, 1
	b	.L1227
.L1234:
	mov	w23, 0
.L1231:
	mov	w0, w27
	add	x19, x19, :lo12:.LANCHOR0
	bl	FlashBchSel
	ldr	w1, [x19,1872]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w1, w0
	bne	.L1236
	mov	w0, 0
	bl	flash_exit_slc_mode
.L1236:
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
	stp	x21, x22, [sp,32]
	mov	x22, x0
	mov	w0, 32768
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x27, x28, [sp,80]
	adrp	x23, .LANCHOR4
	bl	ftl_malloc
	add	x21, x23, :lo12:.LANCHOR4
	str	x0, [x20,1856]
	mov	w0, 32768
	add	x24, x20, 24
	adrp	x25, .LC91
	mov	w28, 0
	bl	ftl_malloc
	str	x0, [x21,80]
	mov	w0, 4096
	mov	w27, 144
	bl	ftl_malloc
	str	x0, [x21,88]
	mov	w0, 32768
	bl	ftl_malloc
	str	x0, [x21,96]
	mov	w0, 4096
	bl	ftl_malloc
	str	x0, [x21,104]
	mov	w0, 128
	str	w0, [x20,12]
	mov	w0, 60
	mov	w1, 50
	strb	w0, [x21,74]
	mov	x0, x22
	strb	wzr, [x20,1864]
	strb	w1, [x20,9]
	strb	wzr, [x20,8]
	add	x20, x20, 1620
	strb	w1, [x21,72]
	mov	x26, x20
	str	wzr, [x21,68]
	str	wzr, [x21,40]
	bl	NandcInit
	add	x0, x25, :lo12:.LC91
	str	x0, [x29,104]
.L1242:
	uxtb	w25, w28
	mov	w0, w25
	bl	FlashReset
	ldrb	w22, [x24,8]
	ldr	x21, [x24]
	mov	w0, w25
	ubfiz	x22, x22, 8, 8
	add	x21, x21, x22
	bl	NandcFlashCs
	mov	w0, 200
	str	w27, [x21,2056]
	str	wzr, [x21,2052]
	bl	NandcDelayns
	ldr	w0, [x21,2048]
	strb	w0, [x20]
	ldr	w0, [x21,2048]
	strb	w0, [x20,1]
	ldr	w0, [x21,2048]
	strb	w0, [x20,2]
	ldr	w0, [x21,2048]
	strb	w0, [x20,3]
	ldr	w0, [x21,2048]
	strb	w0, [x20,4]
	ldr	w0, [x21,2048]
	strb	w0, [x20,5]
	ldr	w0, [x21,2048]
	strb	w0, [x20,6]
	ldr	w0, [x21,2048]
	strb	w0, [x20,7]
	mov	w0, w25
	bl	NandcFlashDeCs
	ldrb	w2, [x20]
	sub	w0, w2, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L1239
	ldr	x0, [x29,104]
	add	w1, w28, 1
	ldrb	w3, [x20,1]
	ldrb	w4, [x20,2]
	ldrb	w5, [x20,3]
	ldrb	w6, [x20,4]
	ldrb	w7, [x20,5]
	bl	printk
.L1239:
	cbnz	w28, .L1240
	ldrb	w0, [x26]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 253
	bhi	.L1280
	ldrb	w0, [x26,1]
	cmp	w0, 255
	beq	.L1280
.L1240:
	add	w28, w28, 1
	add	x24, x24, 16
	cmp	w28, 4
	add	x20, x20, 8
	bne	.L1242
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x0,1620]
	cmp	w1, 173
	beq	.L1243
	ldr	w0, [x0,1868]
	bl	NandcSetDdrMode
.L1243:
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
	cbnz	w3, .L1244
	cmp	w2, 241
	beq	.L1244
	cmp	w2, 220
	bne	.L1245
	ldrb	w0, [x20,3]
	cmp	w0, 149
	bne	.L1245
.L1244:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w4, 1
	mov	w5, 16
	strb	w4, [x0,8]
	add	x4, x23, :lo12:.LANCHOR4
	strb	w5, [x0,9]
	ldrb	w0, [x0,1620]
	strb	w5, [x4,74]
	add	x5, x1, :lo12:.LANCHOR1
	cmp	w0, 152
	strb	w0, [x5,2969]
	bne	.L1246
	mov	w0, 24
	strb	w0, [x4,74]
.L1246:
	cbz	w3, .L1247
	add	x0, x1, :lo12:.LANCHOR1
	mov	w2, 2048
	strh	w2, [x0,2982]
	mov	w2, -38
	b	.L1305
.L1247:
	cmp	w2, 220
	bne	.L1248
	add	x0, x1, :lo12:.LANCHOR1
	mov	w2, 4096
	strh	w2, [x0,2982]
	mov	w2, -36
.L1305:
	strb	w2, [x0,2970]
.L1248:
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
.L1245:
	add	x20, x19, :lo12:.LANCHOR0
	ldrb	w0, [x20,8]
	cbnz	w0, .L1249
	bl	FlashLoadPhyInfoInRam
	cbnz	w0, .L1251
	ldr	x0, [x20,744]
	ldrb	w1, [x0,17]
	and	w0, w1, 7
	strb	w0, [x20,1844]
	tbnz	x1, 0, .L1251
	mov	w1, 1
	strb	w1, [x20,1864]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x20,1844]
	bl	NandcSetMode
.L1251:
	add	x20, x19, :lo12:.LANCHOR0
	ldr	x0, [x20,744]
	ldrb	w0, [x0,26]
	strb	w0, [x20,120]
	bl	FlashLoadPhyInfo
	cbz	w0, .L1249
	ldr	w0, [x20,1868]
	cbz	w0, .L1254
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	b	.L1306
.L1254:
	ldrb	w0, [x20,1844]
	bl	FlashSetInterfaceMode
	ldrb	w0, [x20,1844]
.L1306:
	bl	NandcSetMode
	bl	FlashLoadPhyInfo
	cbz	w0, .L1249
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
	beq	.L1241
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
	blt	.L1256
	ldrh	w1, [x0,14]
	add	w1, w1, 255
	cmp	w2, w1, lsr 8
	bge	.L1257
.L1256:
	ldrh	w1, [x0,14]
	and	w1, w1, -256
	strh	w1, [x0,14]
.L1257:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,1844]
	tst	w0, 6
	beq	.L1258
	bl	FlashSavePhyInfo
	add	x1, x23, :lo12:.LANCHOR4
	mov	w0, 0
	ldr	w1, [x1,64]
	bl	FlashDdrParaScan
.L1258:
	bl	FlashSavePhyInfo
.L1249:
	add	x1, x19, :lo12:.LANCHOR0
	ldr	x3, [x1,744]
	ldrb	w0, [x3,26]
	strb	w0, [x1,120]
	ldrh	w0, [x3,16]
	ubfx	x2, x0, 7, 1
	strb	w2, [x1,16]
	add	x2, x23, :lo12:.LANCHOR4
	ubfx	x4, x0, 3, 1
	strb	w4, [x2,112]
	ubfx	x4, x0, 4, 1
	str	xzr, [x2,48]
	strb	w4, [x1,1945]
	ubfx	x4, x0, 8, 3
	strb	w4, [x1,1844]
	tbz	x0, 6, .L1260
	ldrb	w0, [x3,19]
	ldrb	w4, [x1,762]
	strb	w4, [x2,73]
	sub	w4, w0, #1
	ldrb	w5, [x1,761]
	uxtb	w4, w4
	strb	w0, [x1,752]
	strb	w5, [x1,753]
	cmp	w4, 5
	bhi	.L1261
	sub	w0, w0, #5
	adrp	x4, HynixReadRetrial
	add	x4, x4, :lo12:HynixReadRetrial
	str	x4, [x2,48]
	uxtb	w0, w0
	cmp	w0, 1
	bhi	.L1260
	mov	w0, 1
	str	w0, [x1,1936]
	b	.L1260
.L1261:
	sub	w1, w0, #17
	uxtb	w1, w1
	cmp	w1, 2
	bhi	.L1263
	adrp	x1, MicronReadRetrial
	cmp	w0, 19
	add	x1, x1, :lo12:MicronReadRetrial
	str	x1, [x2,48]
	beq	.L1264
	mov	w0, 7
	b	.L1307
.L1264:
	mov	w0, 15
.L1307:
	strb	w0, [x2,73]
	b	.L1260
.L1263:
	sub	w1, w0, #65
	uxtb	w1, w1
	cmp	w1, 1
	bls	.L1281
	cmp	w0, 33
	bne	.L1265
.L1281:
	add	x0, x23, :lo12:.LANCHOR4
	adrp	x1, ToshibaReadRetrial
	add	x1, x1, :lo12:ToshibaReadRetrial
	mov	w2, 4
	str	x1, [x0,48]
	add	x1, x19, :lo12:.LANCHOR0
	strb	w2, [x1,753]
	mov	w1, 7
	strb	w1, [x0,73]
	b	.L1260
.L1265:
	sub	w1, w0, #67
	uxtb	w1, w1
	cmp	w1, 1
	sub	w1, w0, #34
	cset	w4, ls
	uxtb	w1, w1
	cmp	w1, 1
	bls	.L1282
	cbz	w4, .L1267
.L1282:
	add	x2, x23, :lo12:.LANCHOR4
	adrp	x1, ToshibaReadRetrial
	add	x1, x1, :lo12:ToshibaReadRetrial
	cmp	w0, 35
	str	x1, [x2,48]
	beq	.L1269
	cmp	w0, 68
	beq	.L1269
	mov	w0, 7
	strb	w0, [x2,73]
	b	.L1270
.L1269:
	add	x0, x23, :lo12:.LANCHOR4
	mov	w1, 17
	strb	w1, [x0,73]
.L1270:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, 4
	cbnz	w4, .L1308
	mov	w1, 5
.L1308:
	strb	w1, [x0,753]
	b	.L1260
.L1267:
	cmp	w0, 49
	bne	.L1260
	adrp	x0, SamsungReadRetrial
	add	x0, x0, :lo12:SamsungReadRetrial
	str	x0, [x2,48]
.L1260:
	ldrh	w2, [x3,10]
	ldrb	w1, [x3,12]
	ldrb	w0, [x3,18]
	sdiv	w1, w2, w1
	bl	BuildFlashLsbPageTable
	bl	FlashDieInfoInit
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w1, [x0,1620]
	cmp	w1, 44
	bne	.L1272
	ldrb	w1, [x0,1864]
	cbz	w1, .L1272
	strb	wzr, [x0,1864]
	mov	w0, 1
	bl	FlashSetInterfaceMode
	mov	w0, 1
	bl	NandcSetMode
.L1272:
	mov	w0, 0
	bl	flash_enter_slc_mode
	add	x1, x19, :lo12:.LANCHOR0
	ldrb	w0, [x1,1844]
	tst	w0, 6
	beq	.L1273
	ldrb	w1, [x1,1864]
	cbnz	w1, .L1274
	tbnz	x0, 0, .L1273
.L1274:
	add	x1, x23, :lo12:.LANCHOR4
	mov	w0, 0
	ldr	w1, [x1,64]
	bl	FlashDdrParaScan
.L1273:
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
	cbnz	w0, .L1275
	add	x23, x23, :lo12:.LANCHOR4
	ldrb	w1, [x20,9]
	ldrb	w0, [x23,72]
	cmp	w1, w0
	bls	.L1276
	strb	w0, [x20,9]
.L1276:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w0, [x0,9]
	cmp	w0, 15
	bhi	.L1277
.L1275:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, 16
	strb	w1, [x0,9]
.L1277:
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
	bne	.L1278
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
.L1278:
	ldrb	w0, [x1,20]
	bl	FlashBchSel
	bl	FlashSuspend
	mov	w0, 0
	b	.L1241
.L1280:
	mov	w0, -2
.L1241:
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
	cbz	w2, .L1310
	ldr	w2, [x0,1872]
	mov	w0, 12336
	movk	w0, 0x5638, lsl 16
	cmp	w2, w0
	beq	.L1309
.L1310:
	sub	w0, w24, #5
	uxtb	w0, w0
	cmp	w0, 1
	bls	.L1312
	cmp	w24, 68
	beq	.L1312
	cmp	w24, 35
	beq	.L1312
	sub	w0, w24, #17
	uxtb	w0, w0
	cmp	w0, 2
	bhi	.L1309
.L1312:
	mov	w27, w1
	sub	w24, w24, #17
	add	x23, x23, :lo12:.LANCHOR0
	add	x26, x23, 24
	add	x26, x26, x21, sxtw 4
.L1315:
	ldr	x0, [x23,744]
	ldrh	w0, [x0,10]
	cmp	w0, w22
	bls	.L1309
	adrp	x0, .LANCHOR2
	add	x0, x0, :lo12:.LANCHOR2
	add	x0, x0, 996
	ldrh	w1, [x0,w22,sxtw 1]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L1309
	adrp	x25, .LANCHOR4
	mov	w1, 255
	add	x19, x25, :lo12:.LANCHOR4
	mov	w2, 32768
	ldr	x0, [x19,80]
	bl	ftl_memset
	uxtb	w0, w24
	cmp	w0, 1
	bhi	.L1316
	adrp	x28, .LANCHOR1
	add	x28, x28, :lo12:.LANCHOR1
	ldr	w1, [x28,3000]
	cbz	w1, .L1317
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
	bne	.L1318
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
	cbnz	w20, .L1319
	mov	w0, 2
	str	w0, [x28,3000]
	b	.L1318
.L1319:
	str	wzr, [x28,3000]
	b	.L1309
.L1318:
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
	b	.L1317
.L1316:
	ldr	x2, [x19,80]
	mov	w0, w21
	add	w1, w22, w27
	mov	x3, 0
	bl	FlashProgPage
.L1317:
	add	w22, w22, 1
	uxth	w22, w22
	b	.L1315
.L1309:
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
.L1331:
	cmp	w22, w25
	beq	.L1365
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
	bcc	.L1332
	mov	w0, -1
	str	w0, [x19]
	b	.L1333
.L1332:
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
	bls	.L1336
	cmn	w3, #1
	mov	w0, 256
	csel	w3, w3, w0, eq
.L1336:
	cmp	w3, 256
	beq	.L1347
	cmn	w3, #1
	bne	.L1337
.L1347:
	str	w3, [x19]
	b	.L1339
.L1337:
	str	wzr, [x19]
.L1339:
	ldr	x0, [x19,16]
	cbz	x0, .L1340
	ldr	w1, [x0,8]
	cmn	w1, #1
	bne	.L1340
	ldr	w0, [x0]
	cmn	w0, #1
	beq	.L1340
	str	w1, [x19]
.L1340:
	ldr	w3, [x19]
	cmn	w3, #1
	bne	.L1333
	add	x5, x5, :lo12:.LANCHOR0
	ldr	w1, [x19,4]
	mov	x0, x23
	ldrb	w2, [x5,1944]
	bl	printk
	ldr	x1, [x19,8]
	cbz	x1, .L1342
	adrp	x0, .LC94
	mov	w2, 4
	add	x0, x0, :lo12:.LC94
	mov	w3, 8
	bl	rknand_print_hex
.L1342:
	ldr	x1, [x19,16]
	cbz	x1, .L1333
	mov	w2, 4
	mov	x0, x24
	mov	w3, w2
	bl	rknand_print_hex
.L1333:
	add	w22, w22, 1
	add	x19, x19, 56
	b	.L1331
.L1365:
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
	str	w3, [x29,156]
	add	x3, x22, :lo12:.LANCHOR0
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	str	w1, [x29,152]
	mov	x25, x0
	ldrb	w4, [x3,16]
	ldrb	w23, [x3,8]
	str	w2, [x29,140]
	str	w4, [x29,136]
	cbz	w23, .L1402
	bl	FlashReadSlc2KPages
	b	.L1448
.L1402:
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
.L1367:
	ldr	w0, [x29,152]
	cmp	w24, w0
	bcs	.L1450
	mov	w27, 56
	ldr	w0, [x29,152]
	ldr	w1, [x29,140]
	add	x2, x29, 172
	umull	x27, w24, w27
	sub	w4, w0, w24
	add	x3, x29, 168
	add	x21, x25, x27
	mov	x0, x21
	ldr	w28, [x21,4]
	bl	LogAddr2PhyAddr
	add	x2, x22, :lo12:.LANCHOR0
	mov	w19, w0
	ldr	w0, [x29,168]
	ldrb	w1, [x2,1845]
	cmp	w0, w1
	bcc	.L1369
	mov	w0, -1
	str	w0, [x25,x27]
	b	.L1370
.L1369:
	add	x0, x2, x0, uxtw
	adrp	x26, .LANCHOR4
	ldrb	w20, [x0,1848]
	add	x0, x26, :lo12:.LANCHOR4
	str	x2, [x29,144]
	ldrb	w0, [x0,112]
	cmp	w0, wzr
	mov	w0, w20
	csel	w19, w19, wzr, ne
	bl	NandcWaitFlashReady
	ldr	x2, [x29,144]
	ldr	x0, [x2,744]
	ldrb	w0, [x0,19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 5
	bhi	.L1372
	sxtw	x0, w20
	add	x1, x2, 760
	add	x3, x1, x0
	add	x0, x2, x0
	ldrb	w3, [x3,12]
	ldrb	w0, [x0,1616]
	cmp	w0, w3
	beq	.L1372
	ldrb	w1, [x2,761]
	mov	w0, w20
	add	x2, x2, 764
	bl	HynixSetRRPara
.L1372:
	mov	w0, w20
	bl	NandcFlashCs
	ldr	w0, [x29,140]
	cmp	w0, 1
	cset	w0, eq
	orr	w0, w0, w28, lsr 31
	str	w0, [x29,144]
	cbz	w0, .L1379
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1379
	mov	w0, w20
	bl	flash_enter_slc_mode
.L1379:
	cmp	w20, 255
	ldr	w1, [x29,172]
	bne	.L1407
	cmn	w1, #1
	cset	w0, ne
	cbz	w0, .L1403
.L1407:
	cbz	w19, .L1376
	add	x0, x22, :lo12:.LANCHOR0
	ldr	w2, [x0,12]
	mov	w0, w20
	add	w2, w1, w2
	bl	FlashReadDpCmd
	b	.L1377
.L1376:
	mov	w0, w20
	bl	FlashReadCmd
	b	.L1377
.L1403:
	mov	w19, w0
.L1374:
	ldrb	w2, [x29,156]
	mov	w0, w20
	ldr	x3, [x21,8]
	mov	w1, 0
	ldr	x4, [x21,16]
	bl	NandcXferData
	cmn	w0, #1
	mov	w28, w0
	bne	.L1378
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w1, [x0,16]
	cbz	w1, .L1378
	strb	wzr, [x0,16]
	mov	w19, 0
	b	.L1379
.L1378:
	cbz	w19, .L1380
	add	x0, x22, :lo12:.LANCHOR0
	ldr	w1, [x29,172]
	ldr	w2, [x0,12]
	mov	w0, w20
	add	w1, w2, w1
	bl	FlashReadDpDataOutCmd
	add	w0, w24, 1
	mov	w4, 56
	ldrb	w2, [x29,156]
	mov	w1, 0
	umull	x4, w0, w4
	mov	w0, w20
	add	x4, x25, x4
	ldr	x3, [x4,8]
	ldr	x4, [x4,16]
	bl	NandcXferData
	cmn	w0, #1
	mov	w23, w0
	csel	w19, w19, wzr, ne
.L1380:
	mov	w0, w20
	bl	NandcFlashDeCs
	add	x6, x22, :lo12:.LANCHOR0
	ldrb	w0, [x29,136]
	cmn	w28, #1
	strb	w0, [x6,16]
	bne	.L1387
	ldrb	w0, [x6,1864]
	cbnz	w0, .L1382
.L1386:
	add	x0, x26, :lo12:.LANCHOR4
	ldr	x19, [x0,48]
	cbnz	x19, .L1383
	b	.L1451
.L1382:
	ldr	x0, [x6,128]
	mov	w4, 1
	ldr	w1, [x29,172]
	ldr	x2, [x21,8]
	ldr	x3, [x21,16]
	ldr	w19, [x0,304]
	mov	w0, w20
	str	x6, [x29,104]
	bl	FlashDdrTunningRead
	cmn	w0, #1
	mov	w28, w0
	beq	.L1385
	ldr	x6, [x29,104]
	ldrb	w0, [x6,1944]
	cmp	w28, w0, lsr 1
	bls	.L1405
.L1385:
	lsr	w0, w19, 8
	bl	NandcSetDdrPara
	cmn	w28, #1
	beq	.L1386
	b	.L1405
.L1383:
	ldr	w1, [x29,172]
	mov	w0, w20
	ldr	x2, [x21,8]
	ldr	x3, [x21,16]
	blr	x19
	cmn	w0, #1
	mov	w28, w0
	mov	w19, 0
	bne	.L1388
	add	x2, x22, :lo12:.LANCHOR0
	ldr	x0, [x2,744]
	ldrb	w0, [x0,19]
	sub	w0, w0, #1
	uxtb	w0, w0
	cmp	w0, 5
	bhi	.L1389
	ldrb	w1, [x2,761]
	mov	w0, w20
	add	x2, x2, 764
	mov	w3, w19
	bl	HynixSetRRPara
.L1389:
	ldr	w1, [x29,172]
	mov	w0, w20
	ldr	x2, [x21,8]
	mov	w19, 0
	ldr	x3, [x21,16]
	bl	FlashReadRawPage
	mov	w28, w0
	add	x2, x22, :lo12:.LANCHOR0
	ldr	x0, [x29,112]
	ldr	w1, [x21,4]
	mov	w3, w28
	ldrb	w2, [x2,1944]
	bl	printk
	b	.L1388
.L1451:
	ldr	w1, [x29,172]
	mov	w0, w20
	ldr	x2, [x21,8]
	ldr	x3, [x21,16]
	bl	FlashReadRawPage
	mov	w28, w0
	b	.L1388
.L1405:
	mov	w19, 0
.L1387:
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,1944]
	add	w0, w0, w0, lsl 1
	cmp	w28, w0, lsr 2
	bls	.L1388
	add	x0, x26, :lo12:.LANCHOR4
	ldr	x0, [x0,48]
	cmp	x0, xzr
	mov	w0, 256
	csel	w28, w28, w0, ne
.L1388:
	cmp	w28, 256
	beq	.L1408
	cmn	w28, #1
	bne	.L1390
.L1408:
	str	w28, [x25,x27]
	b	.L1392
.L1390:
	str	wzr, [x25,x27]
.L1392:
	ldr	w3, [x25,x27]
	cmn	w3, #1
	bne	.L1394
	add	x2, x22, :lo12:.LANCHOR0
	ldr	w1, [x21,4]
	ldr	x0, [x29,128]
	ldrb	w2, [x2,1944]
	bl	printk
	ldr	x1, [x21,16]
	cbz	x1, .L1394
	mov	w2, 4
	ldr	x0, [x29,120]
	mov	w3, w2
	bl	rknand_print_hex
.L1394:
	cbz	w19, .L1396
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,1944]
	add	w0, w0, w0, lsl 1
	cmp	w23, w0, lsr 2
	bls	.L1397
	add	x26, x26, :lo12:.LANCHOR4
	ldr	x0, [x26,48]
	cmp	x0, xzr
	mov	w0, 256
	csel	w23, w23, w0, ne
.L1397:
	add	w0, w24, 1
	mov	w1, 56
	cmp	w23, 256
	umull	x0, w0, w1
	beq	.L1409
	cmn	w23, #1
	bne	.L1398
.L1409:
	str	w23, [x25,x0]
	b	.L1396
.L1398:
	str	wzr, [x25,x0]
.L1396:
	ldr	w0, [x29,144]
	add	w24, w24, w19
	cbz	w0, .L1370
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1370
	mov	w0, w20
	bl	flash_exit_slc_mode
.L1370:
	add	w24, w24, 1
	b	.L1367
.L1450:
	mov	w0, 0
	b	.L1448
.L1377:
	mov	w0, w20
	bl	NandcWaitFlashReady
	cbz	w19, .L1374
	ldr	w1, [x29,172]
	mov	w0, w20
	bl	FlashReadDpDataOutCmd
	b	.L1374
.L1448:
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
	sub	w0, w21, #8
	str	xzr, [x29,144]
	uxth	w25, w25
	str	w0, [x29,124]
.L1453:
	add	x1, x22, :lo12:.LANCHOR0
	ldrb	w1, [x1,1845]
	cmp	w1, w19
	bls	.L1463
	mul	w3, w19, w21
	mov	w20, w25
	mov	w4, 61664
.L1454:
	ldr	w0, [x29,124]
	cmp	w20, w0
	ble	.L1456
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
	beq	.L1455
	add	x24, x23, :lo12:.LANCHOR4
	ldr	x0, [x24,88]
	ldrh	w0, [x0]
	cmp	w0, w4
	bne	.L1455
	add	x24, x24, 120
	mov	x0, x26
	mov	w1, w19
	mov	w2, w20
	add	w28, w28, 1
	bl	printk
	strh	w20, [x24,w19,sxtw 1]
	uxth	w28, w28
	b	.L1456
.L1455:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L1454
.L1456:
	add	x1, x22, :lo12:.LANCHOR0
	add	w19, w19, 1
	uxtb	w19, w19
	ldrb	w1, [x1,1845]
	cmp	w1, w28
	csel	w27, w27, wzr, ne
	b	.L1453
.L1463:
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
.L1465:
	cmp	w25, w23
	beq	.L1494
	mov	w1, w24
	sub	w4, w23, w25
	mov	x0, x21
	add	x2, x29, 96
	add	x3, x29, 100
	bl	LogAddr2PhyAddr
	ldr	w0, [x29,100]
	ldrb	w1, [x22,1845]
	cmp	w0, w1
	bcc	.L1466
	str	w28, [x21]
	b	.L1467
.L1466:
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
	tbz	x0, 0, .L1470
	str	w28, [x21]
.L1470:
	mov	w0, w20
	bl	NandcFlashDeCs
.L1467:
	add	w25, w25, 1
	add	x21, x21, 56
	b	.L1465
.L1494:
	mov	w21, 0
	cbz	w27, .L1492
	adrp	x25, .LC101
	adrp	x26, .LC100
	adrp	x27, .LC99
	add	x25, x25, :lo12:.LC101
	add	x26, x26, :lo12:.LC100
	add	x27, x27, :lo12:.LC99
.L1493:
	cmp	w21, w23
	beq	.L1492
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1473
	adrp	x0, .LC98
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC98
	bl	printk
	b	.L1474
.L1473:
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
	bne	.L1475
	ldr	w1, [x19,4]
	mov	x0, x27
	bl	printk
	str	w22, [x19]
.L1475:
	ldr	x0, [x19,16]
	cbz	x0, .L1476
	ldr	w2, [x0]
	add	x0, x20, :lo12:.LANCHOR4
	ldr	x0, [x0,104]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1476
	ldr	w1, [x19,4]
	mov	x0, x26
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1476:
	ldr	x0, [x19,8]
	cbz	x0, .L1474
	add	x20, x20, :lo12:.LANCHOR4
	ldr	w2, [x0]
	ldr	x0, [x20,96]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1474
	ldr	w1, [x19,4]
	mov	x0, x25
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1474:
	add	w21, w21, 1
	add	x19, x19, 56
	b	.L1493
.L1492:
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
.L1496:
	add	x0, x22, :lo12:.LANCHOR0
	ldrh	w1, [x0,1974]
	cmp	w23, w1
	bcs	.L1505
	ldrh	w20, [x0,2022]
	strh	w25, [x21,12]
.L1498:
	add	x0, x22, :lo12:.LANCHOR0
	sub	w20, w20, #1
	uxth	w20, w20
	ldrh	w0, [x0,2022]
	sub	w1, w0, #16
	cmp	w20, w1
	ble	.L1499
	madd	w0, w23, w0, w20
	mov	w1, 1
	lsl	w0, w0, 10
	mov	w2, w1
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L1498
	ldrh	w0, [x24]
	cmp	w0, w26
	bne	.L1498
	strh	w20, [x21,12]
.L1499:
	add	w23, w23, 1
	add	x21, x21, 2
	b	.L1496
.L1505:
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
	bne	.L1507
	ldrh	w19, [x3,2028]
	b	.L1508
.L1507:
	ldrh	w19, [x3,2026]
.L1508:
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
	bne	.L1509
	mov	w22, 0
	mov	w25, 2
.L1510:
	cmp	w22, w19
	bgt	.L1509
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
	bne	.L1511
	ldr	w0, [x24,4]
	cmn	w0, #1
	bne	.L1511
	ldr	w0, [x29,88]
	cmn	w0, #1
	beq	.L1511
	sub	w19, w20, #1
	sxth	w19, w19
	b	.L1510
.L1511:
	add	w20, w20, 1
	sxth	w22, w20
	b	.L1510
.L1509:
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
	str	x25, [sp,64]
	add	x19, x19, 136
	adrp	x20, .LANCHOR0
	str	xzr, [x19,8]
	ldr	x22, [x0,2112]
	mov	w25, 61649
	str	x22, [x19,16]
	bl	FtlBbtMemInit
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w21, [x0,2022]
	sub	w21, w21, #1
	uxth	w21, w21
.L1518:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w0, [x0,2022]
	sub	w0, w0, #48
	cmp	w21, w0
	ble	.L1521
	lsl	w0, w21, 10
	mov	w1, 1
	str	w0, [x19,4]
	mov	w2, w1
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1519
	ldr	w0, [x19,4]
	mov	w1, 1
	mov	w2, w1
	add	w0, w0, 1
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
.L1519:
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L1520
	ldrh	w0, [x22]
	cmp	w0, w25
	bne	.L1520
	add	x1, x20, :lo12:.LANCHOR0
	add	x0, x1, 2112
	strh	w21, [x1,2112]
	ldr	w1, [x22,4]
	str	w1, [x0,8]
	ldrh	w1, [x22,8]
	strh	w1, [x0,4]
	b	.L1521
.L1520:
	sub	w21, w21, #1
	uxth	w21, w21
	b	.L1518
.L1521:
	add	x21, x20, :lo12:.LANCHOR0
	mov	w2, 65535
	mov	w0, -1
	ldrh	w1, [x21,2112]
	cmp	w1, w2
	beq	.L1523
	ldrh	w1, [x21,2116]
	cmp	w1, w2
	beq	.L1525
	add	x25, x23, :lo12:.LANCHOR4
	lsl	w1, w1, 10
	add	x0, x25, 136
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	bl	FlashReadPages
	ldr	w0, [x25,136]
	cmn	w0, #1
	beq	.L1525
	ldrh	w1, [x22]
	mov	w0, 61649
	cmp	w1, w0
	bne	.L1525
	ldr	w0, [x22,4]
	ldr	w1, [x21,2120]
	cmp	w0, w1
	bls	.L1525
	str	w0, [x21,2120]
	ldrh	w1, [x21,2116]
	ldrh	w0, [x22,8]
	strh	w1, [x21,2112]
	strh	w0, [x21,2116]
.L1525:
	add	x25, x20, :lo12:.LANCHOR0
	mov	w1, 1
	add	x21, x23, :lo12:.LANCHOR4
	add	x21, x21, 136
	ldrh	w0, [x25,2112]
	bl	FtlGetLastWrittenPage
	sxth	w19, w0
	add	w0, w0, 1
	strh	w0, [x25,2114]
.L1527:
	tbnz	w19, #31, .L1532
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
	beq	.L1528
.L1532:
	add	x1, x20, :lo12:.LANCHOR0
	ldrh	w0, [x22,10]
	mov	w2, 65535
	strh	w0, [x1,2118]
	ldrh	w0, [x22,12]
	cmp	w0, w2
	bne	.L1529
	b	.L1530
.L1528:
	sub	w19, w19, #1
	sxth	w19, w19
	b	.L1527
.L1529:
	ldr	w2, [x1,1948]
	cmp	w0, w2
	beq	.L1530
	ldrh	w1, [x1,1962]
	lsr	w1, w1, 2
	cmp	w0, w1
	bcs	.L1530
	cmp	w2, w1
	bcs	.L1530
	bl	FtlSysBlkNumInit
.L1530:
	add	x20, x20, :lo12:.LANCHOR0
	mov	x19, 0
	add	x21, x20, 2112
	add	x23, x23, :lo12:.LANCHOR4
.L1533:
	ldrh	w0, [x20,1974]
	mov	w1, w19
	add	x19, x19, 1
	cmp	w1, w0
	bcs	.L1548
	add	x0, x24, :lo12:.LANCHOR2
	ldr	x3, [x23,144]
	ldrh	w2, [x0,2232]
	add	x0, x21, x19, lsl 3
	mul	w1, w1, w2
	ldr	x0, [x0,24]
	lsl	w2, w2, 2
	add	x1, x3, x1, lsl 2
	bl	ftl_memcpy
	b	.L1533
.L1548:
	mov	w0, 0
.L1523:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
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
	cbnz	w22, .L1550
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
	b	.L1551
.L1550:
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
.L1551:
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
	bhi	.L1553
	lsr	w28, w22, w28
	mov	w20, 0
	adrp	x26, .LANCHOR2
.L1554:
	cbz	w21, .L1553
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
	cbz	w1, .L1556
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
	b	.L1558
.L1556:
	mov	x0, x24
	mov	w2, w27
	bl	ftl_memset
.L1558:
	add	w28, w28, 1
	sub	w21, w21, w19
	add	w22, w22, w19
	add	x24, x24, x27, sxtw
	b	.L1554
.L1553:
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
	beq	.L1562
	adrp	x0, .LC73
	adrp	x1, .LC102
	add	x1, x1, :lo12:.LC102
	add	x0, x0, :lo12:.LC73
	bl	printk
	ldrh	w2, [x19,2136]
	mov	w1, 0
	ldr	x0, [x19,2152]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1562:
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
	beq	.L1564
	mov	x0, x21
	mov	w1, 0
	mov	w2, 512
	bl	ftl_memset
	mov	w0, 19539
	movk	w0, 0x4654, lsl 16
	str	w0, [x20,352]
.L1564:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, 19539
	movk	w2, 0x4654, lsl 16
	ldr	w3, [x0,352]
	cmp	w3, w2
	bne	.L1565
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
.L1565:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 34661
	movk	w1, 0x1234, lsl 16
	ldr	w2, [x0,420]
	str	wzr, [x0,2024]
	cmp	w2, w1
	bne	.L1566
	mov	w1, 1
	str	w1, [x0,160]
	adrp	x0, .LC73
	adrp	x1, .LC103
	add	x0, x0, :lo12:.LC73
	add	x1, x1, :lo12:.LC103
	bl	printk
.L1566:
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
.L1569:
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w0, [x1,1962]
	cmp	w0, w21
	bls	.L1612
	mov	x27, 0
	ldrh	w5, [x1,1952]
	ldrh	w4, [x1,2038]
	mov	w20, w27
	mov	x23, x1
.L1613:
	cmp	w5, w27, uxth
	bls	.L1644
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
	cbnz	w0, .L1570
	add	x2, x24, :lo12:.LANCHOR2
	mov	w0, 56
	lsl	w28, w28, 10
	mov	w3, 4
	umull	x0, w20, w0
	ldr	x1, [x2,2048]
	add	x1, x1, x0
	str	w28, [x1,4]
	ldr	x1, [x2,2048]
	ldr	x2, [x2,904]
	add	x0, x1, x0
	mul	w1, w20, w4
	add	w20, w20, 1
	str	xzr, [x0,8]
	sdiv	w1, w1, w3
	uxth	w20, w20
	add	x1, x2, x1, sxtw 2
	str	x1, [x0,16]
.L1570:
	add	x27, x27, 1
	b	.L1613
.L1644:
	add	x0, x24, :lo12:.LANCHOR2
	mov	w1, w20
	mov	w2, 1
	mov	x23, 0
	mov	w27, 61634
	mov	w28, 61574
	ldr	x0, [x0,2048]
	bl	FlashReadPages
.L1572:
	cmp	w20, w23, uxth
	bls	.L1645
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
	bne	.L1574
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
	bne	.L1574
	ldr	x0, [x7,2048]
	str	w8, [x0,x6]
.L1574:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	x1, [x0,2048]
	ldr	w1, [x1,x6]
	cmn	w1, #1
	beq	.L1576
	ldr	w0, [x0,196]
	cmn	w0, #1
	beq	.L1577
	ldr	w1, [x26,4]
	cmp	w0, w1
	bhi	.L1578
.L1577:
	ldr	w0, [x26,4]
	cmn	w0, #1
	beq	.L1578
	add	x1, x24, :lo12:.LANCHOR2
	add	w0, w0, 1
	str	w0, [x1,196]
.L1578:
	ldrh	w0, [x26]
	mov	w1, 61604
	cmp	w0, w1
	beq	.L1580
	bhi	.L1581
	cmp	w0, w28
	bne	.L1579
	add	x0, x19, :lo12:.LANCHOR0
	add	x7, x24, :lo12:.LANCHOR2
	ldrh	w6, [x0,2048]
	ldrh	w0, [x0,2072]
	sub	w1, w6, #1
	ldr	x7, [x7,2192]
	sxth	w2, w1
	sub	w1, w1, w0
	b	.L1597
.L1581:
	cmp	w0, w27
	beq	.L1583
	cmp	w0, w22
	bne	.L1579
	mov	w0, w25
	mov	w1, 0
	b	.L1642
.L1583:
	add	x0, x19, :lo12:.LANCHOR0
	add	x7, x24, :lo12:.LANCHOR2
	ldr	w6, [x0,2056]
	ldrh	w0, [x7,2304]
	ldr	x7, [x7,2216]
	uxth	w1, w6
	sub	w2, w1, #1
	sub	w1, w1, w0
	sub	w1, w1, #1
	sxth	w2, w2
	sxth	w1, w1
.L1585:
	cmp	w2, w1
	ble	.L1646
	sbfiz	x8, x2, 2, 32
	ldr	w11, [x26,4]
	sxth	x9, w2
	ldr	w10, [x7,x8]
	cmp	w11, w10
	bls	.L1586
	ldr	w1, [x7]
	cbnz	w1, .L1587
	cmp	w0, w6
	bne	.L1588
.L1587:
	add	x0, x24, :lo12:.LANCHOR2
	mov	w1, 1
	str	x9, [x29,104]
	str	x2, [x29,112]
	ldr	x0, [x0,2168]
	str	x8, [x29,120]
	ldrh	w0, [x0]
	bl	FtlFreeSysBlkQueueIn
	ldr	x8, [x29,120]
	ldr	x2, [x29,112]
	ldr	x9, [x29,104]
	b	.L1589
.L1588:
	add	x1, x24, :lo12:.LANCHOR2
	add	w0, w0, 1
	strh	w0, [x1,2304]
.L1589:
	mov	w0, 0
.L1590:
	cmp	w0, w2
	beq	.L1647
	add	x6, x24, :lo12:.LANCHOR2
	sxtw	x1, w0
	lsl	x7, x1, 2
	lsl	x1, x1, 1
	add	w0, w0, 1
	ldr	x10, [x6,2216]
	sxth	w0, w0
	add	x11, x10, x7
	ldr	w11, [x11,4]
	str	w11, [x10,x7]
	ldr	x6, [x6,2168]
	add	x7, x6, x1
	ldrh	w7, [x7,2]
	strh	w7, [x6,x1]
	b	.L1590
.L1647:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	w6, [x26,4]
	ldr	x1, [x0,2216]
	str	w6, [x1,x8]
	ldr	x1, [x0,2168]
	strh	w25, [x1,x9,lsl 1]
	tbz	w2, #31, .L1641
	b	.L1576
.L1586:
	sub	w2, w2, #1
	sxth	w2, w2
	b	.L1585
.L1646:
	tbz	w2, #31, .L1596
	b	.L1576
.L1641:
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w0, [x0,2304]
	ldr	w1, [x1,2056]
	sub	w1, w1, w0
	sub	w1, w1, #1
	cmp	w2, w1, sxth
	bgt	.L1579
.L1596:
	add	x1, x24, :lo12:.LANCHOR2
	add	w0, w0, 1
	sxtw	x2, w2
	ldr	w4, [x26,4]
	strh	w0, [x1,2304]
	ldr	x0, [x1,2216]
	str	w4, [x0,x2,lsl 2]
	ldr	x0, [x1,2168]
	b	.L1643
.L1605:
	sbfiz	x8, x2, 2, 32
	ldr	w11, [x26,4]
	sxth	x9, w2
	ldr	w10, [x7,x8]
	cmp	w11, w10
	bhi	.L1648
	sub	w2, w2, #1
	sxth	w2, w2
.L1597:
	cmp	w2, w1
	bgt	.L1605
	b	.L1604
.L1648:
	ldr	w1, [x7]
	cbnz	w1, .L1599
	cmp	w0, w6
	bne	.L1600
.L1599:
	add	x0, x19, :lo12:.LANCHOR0
	mov	w1, 1
	str	x9, [x29,104]
	str	x8, [x29,112]
	ldr	x0, [x0,2080]
	str	x2, [x29,120]
	ldrh	w0, [x0]
	bl	FtlFreeSysBlkQueueIn
	ldr	x2, [x29,120]
	ldr	x8, [x29,112]
	ldr	x9, [x29,104]
	b	.L1601
.L1600:
	add	x1, x19, :lo12:.LANCHOR0
	add	w0, w0, 1
	strh	w0, [x1,2072]
.L1601:
	mov	w0, 0
.L1602:
	cmp	w0, w2
	beq	.L1649
	add	x1, x24, :lo12:.LANCHOR2
	ldr	x7, [x1,2192]
	sxtw	x1, w0
	lsl	x6, x1, 2
	lsl	x1, x1, 1
	add	x10, x7, x6
	add	w0, w0, 1
	sxth	w0, w0
	ldr	w10, [x10,4]
	str	w10, [x7,x6]
	add	x6, x19, :lo12:.LANCHOR0
	ldr	x6, [x6,2080]
	add	x7, x6, x1
	ldrh	w7, [x7,2]
	strh	w7, [x6,x1]
	b	.L1602
.L1649:
	add	x0, x24, :lo12:.LANCHOR2
	ldr	w1, [x26,4]
	ldr	x0, [x0,2192]
	str	w1, [x0,x8]
	add	x0, x19, :lo12:.LANCHOR0
	ldr	x0, [x0,2080]
	strh	w25, [x0,x9,lsl 1]
.L1604:
	tbnz	w2, #31, .L1576
	add	x1, x19, :lo12:.LANCHOR0
	ldrh	w0, [x1,2048]
	ldrh	w6, [x1,2072]
	sub	w0, w0, #1
	sub	w0, w0, w6
	cmp	w2, w0, sxth
	bgt	.L1579
	add	x0, x24, :lo12:.LANCHOR2
	sxtw	x2, w2
	add	w6, w6, 1
	ldr	w4, [x26,4]
	strh	w6, [x1,2072]
	ldr	x0, [x0,2192]
	str	w4, [x0,x2,lsl 2]
	ldr	x0, [x1,2080]
.L1643:
	strh	w25, [x0,x2,lsl 1]
	b	.L1579
.L1580:
	add	x0, x24, :lo12:.LANCHOR2
	add	x1, x0, 280
	ldrh	w2, [x0,280]
	cmp	w2, w22
	bne	.L1608
	strh	w25, [x0,280]
	ldr	w0, [x26,4]
	str	w0, [x1,8]
	b	.L1579
.L1608:
	ldrh	w0, [x1,4]
	cmp	w0, w22
	beq	.L1609
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
.L1609:
	add	x1, x24, :lo12:.LANCHOR2
	ldr	w2, [x26,4]
	add	x0, x1, 280
	ldr	w6, [x0,8]
	cmp	w6, w2
	bcs	.L1610
	ldrh	w2, [x1,280]
	strh	w2, [x0,4]
	strh	w25, [x1,280]
	ldr	w1, [x26,4]
	str	w1, [x0,8]
	b	.L1579
.L1610:
	strh	w25, [x0,4]
	b	.L1579
.L1576:
	mov	w0, w25
	mov	w1, 1
.L1642:
	bl	FtlFreeSysBlkQueueIn
.L1579:
	add	x23, x23, 1
	b	.L1572
.L1645:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1569
.L1612:
	add	x4, x24, :lo12:.LANCHOR2
	ldr	x2, [x4,2168]
	ldrh	w0, [x2]
	cbz	w0, .L1614
.L1617:
	add	x1, x19, :lo12:.LANCHOR0
	ldr	x2, [x1,2080]
	ldrh	w0, [x2]
	cbz	w0, .L1615
	b	.L1636
.L1614:
	ldrh	w4, [x4,2304]
	cbz	w4, .L1617
	ldr	w1, [x1,2056]
.L1618:
	cmp	w0, w1
	bcs	.L1617
	ldrh	w4, [x2,w0,sxtw 1]
	cbz	w4, .L1619
	mov	w7, w0
.L1620:
	add	x1, x19, :lo12:.LANCHOR0
	ldr	w1, [x1,2056]
	cmp	w0, w1
	bcs	.L1617
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
	b	.L1620
.L1619:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L1618
.L1615:
	ldrh	w4, [x1,2072]
	cbz	w4, .L1636
	ldrh	w1, [x1,2048]
.L1623:
	cmp	w0, w1
	mov	w7, w0
	bge	.L1636
	ldrh	w4, [x2,w0,sxtw 1]
	cbz	w4, .L1624
.L1625:
	add	x2, x19, :lo12:.LANCHOR0
	ldrh	w1, [x2,2048]
	cmp	w0, w1
	bge	.L1636
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
	b	.L1625
.L1624:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L1623
.L1636:
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
	bne	.L1651
.L1655:
	mov	w19, -1
	b	.L1652
.L1651:
	mov	w1, 1
	bl	FtlGetLastWrittenPage
	sxth	w24, w0
	add	w0, w0, 1
	strh	w0, [x25,2]
.L1653:
	tbnz	w24, #31, .L1658
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
	beq	.L1654
.L1658:
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
	beq	.L1665
	b	.L1655
.L1654:
	sub	w24, w24, #1
	sxth	w24, w24
	b	.L1653
.L1665:
	ldrb	w1, [x20,10]
	ldrh	w0, [x22,1974]
	ldrh	w2, [x20,8]
	strh	w2, [x19,286]
	cmp	w1, w0
	bne	.L1655
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
	bls	.L1659
	str	w0, [x19,196]
.L1659:
	add	x0, x21, :lo12:.LANCHOR2
	ldr	w1, [x0,260]
	ldr	w3, [x0,200]
	cmp	w1, w3
	bls	.L1660
	str	w1, [x0,200]
.L1660:
	mov	w0, 65535
	cmp	w2, w0
	beq	.L1661
	add	x0, x21, :lo12:.LANCHOR2
	bl	make_superblock
.L1661:
	add	x1, x21, :lo12:.LANCHOR2
	add	x0, x1, 48
	ldrh	w2, [x1,48]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L1662
	bl	make_superblock
.L1662:
	add	x1, x21, :lo12:.LANCHOR2
	add	x0, x1, 96
	ldrh	w2, [x1,96]
	mov	w1, 65535
	cmp	w2, w1
	beq	.L1663
	bl	make_superblock
.L1663:
	add	x21, x21, :lo12:.LANCHOR2
	mov	w1, 65535
	add	x0, x21, 296
	mov	w19, 0
	ldrh	w2, [x21,296]
	cmp	w2, w1
	beq	.L1652
	bl	make_superblock
.L1652:
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
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR1
	stp	x23, x24, [sp,48]
	mov	x24, x0
	add	x0, x19, :lo12:.LANCHOR1
	stp	x25, x26, [sp,64]
	stp	x21, x22, [sp,32]
	stp	x27, x28, [sp,80]
	str	x19, [x29,136]
	mov	w25, w1
	ldrh	w28, [x0,3004]
	mov	w0, 65535
	cmp	w28, w0
	beq	.L1686
	cbnz	w28, .L1667
	b	.L1668
.L1686:
	mov	w28, 0
.L1667:
	adrp	x0, .LANCHOR0+2026
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2026]
	cmp	w25, w0
	bne	.L1669
.L1668:
	bl	FtlGcPageVarInit
.L1669:
	mov	w23, -1
	mov	w20, 0
	mov	w21, 65535
	adrp	x26, .LANCHOR0
	adrp	x27, .LANCHOR2
.L1680:
	ldrh	w0, [x24]
	strb	wzr, [x24,8]
	cmp	w0, w21
	beq	.L1687
.L1671:
.L1683:
	add	x0, x26, :lo12:.LANCHOR0
	mov	x2, x24
	mov	w19, 0
	mov	w10, 4
	ldrh	w6, [x0,1952]
	ldrh	w9, [x0,2038]
	add	x6, x24, x6, lsl 1
.L1672:
	cmp	x2, x6
	beq	.L1697
	ldrh	w1, [x2,16]
	cmp	w1, w21
	beq	.L1673
	add	x7, x27, :lo12:.LANCHOR2
	mov	w0, 56
	orr	w1, w28, w1, lsl 10
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
.L1673:
	add	x2, x2, 2
	b	.L1672
.L1697:
	add	x0, x27, :lo12:.LANCHOR2
	mov	w1, w19
	mov	w2, 0
	mov	x22, 0
	ldr	x0, [x0,2048]
	bl	FlashReadPages
.L1675:
	cmp	w19, w22, uxth
	bls	.L1698
	adrp	x0, .LANCHOR2
	add	x7, x0, :lo12:.LANCHOR2
	mov	x0, 56
	str	x7, [x29,96]
	mul	x8, x22, x0
	ldr	x10, [x7,2048]
	str	x8, [x29,104]
	add	x9, x10, x8
	str	x10, [x29,112]
	str	x9, [x29,120]
	ldr	w1, [x9,4]
	str	x1, [x29,128]
	lsr	x0, x1, 10
	bl	P2V_plane
	uxth	w2, w0
	ldr	x10, [x29,112]
	ldr	x8, [x29,104]
	ldr	x1, [x29,128]
	ldr	x7, [x29,96]
	ldr	w0, [x10,x8]
	cbnz	w0, .L1676
	ldr	x9, [x29,120]
	add	x22, x22, 1
	ldr	x8, [x9,16]
	ldrh	w0, [x8]
	cmp	w0, w21
	bne	.L1677
	mov	w0, 1
	str	w0, [x7,2328]
	b	.L1670
.L1677:
	ldr	w0, [x8,12]
	ldr	w2, [x8,8]
	bl	FtlGcUpdatePage
	b	.L1675
.L1676:
	ldr	w2, [x7,160]
	cbz	w2, .L1678
	ldrh	w3, [x24]
	ldr	x2, [x7,-80]
	ldrh	w2, [x2,x3,lsl 1]
	cmp	w2, 119
	bls	.L1679
.L1678:
	cmn	w0, #1
	csel	w1, w1, w23, eq
.L1679:
	adrp	x0, .LANCHOR2
	ldrh	w2, [x24]
	add	x22, x0, :lo12:.LANCHOR2
	str	x1, [x29,128]
	mov	w28, 0
	ldr	x0, [x22,-40]
	strh	wzr, [x0,x2,lsl 1]
	ldrh	w0, [x24]
	bl	INSERT_FREE_LIST
	mov	w0, -1
	strh	w0, [x24]
	bl	FtlGcPageVarInit
	ldr	x1, [x29,128]
	mov	w23, w1
	b	.L1680
.L1698:
	add	w20, w20, 1
	add	w3, w28, 1
	cmp	w20, w25
	uxth	w28, w3
	bcs	.L1682
.L1684:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w0, [x0,2026]
	cmp	w0, w28
	bhi	.L1683
	b	.L1687
.L1682:
	ldr	x0, [x29,136]
	add	x1, x0, :lo12:.LANCHOR1
	ldrh	w0, [x1,3004]
	cmp	w0, w21
	beq	.L1684
	add	w0, w0, w20
	strh	w0, [x1,3004]
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w0, [x0,2026]
	cmp	w0, w28
	bls	.L1684
	b	.L1685
.L1687:
	mov	w2, 0
.L1670:
	ldr	x0, [x29,136]
	mov	w1, w28
	strh	w28, [x24,2]
	add	x19, x0, :lo12:.LANCHOR1
	mov	w0, -1
	strb	w2, [x24,6]
	strh	w0, [x19,3004]
	mov	x0, x24
	bl	ftl_sb_update_avl_pages
.L1685:
	mov	w0, w23
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 144
	ret
	.size	FtlGcScanTempBlk, .-FtlGcScanTempBlk
	.align	2
	.global	FlashProgPages
	.type	FlashProgPages, %function
FlashProgPages:
	stp	x29, x30, [sp, -192]!
	adrp	x6, .LANCHOR1+481
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	ldrb	w6, [x6,#:lo12:.LANCHOR1+481]
	add	x4, x20, :lo12:.LANCHOR0
	str	w3, [x29,120]
	stp	x21, x22, [sp,32]
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
	cbz	w4, .L1700
	bl	FlashProgSlc2KPages
	b	.L1701
.L1700:
	sub	w0, w5, #1
	mov	w27, 56
	mov	x28, 24
	str	w0, [x29,116]
.L1751:
	cmp	w22, w25
	bcs	.L1752
	umull	x21, w22, w27
	sub	w4, w25, w22
	add	x2, x29, 128
	add	x26, x19, x21
	mov	w1, w23
	mov	x0, x26
	add	x3, x29, 132
	bl	LogAddr2PhyAddr
	mov	w24, w0
	add	x2, x20, :lo12:.LANCHOR0
	ldr	w0, [x29,132]
	ldrb	w4, [x2,1845]
	cmp	w0, w4
	bcc	.L1702
	mov	w0, -1
	str	w0, [x19,x21]
	b	.L1703
.L1702:
	ldrb	w1, [x2,1945]
	add	x3, x2, 1652
	cmp	w1, wzr
	uxtw	x1, w0
	csel	w24, w24, wzr, ne
	madd	x0, x1, x28, x3
	ldr	x0, [x0,8]
	cbz	x0, .L1705
	cmp	w4, 1
	bne	.L1706
	ldr	x0, [x2,128]
	bl	NandcIqrWaitFlashReady
.L1706:
	ldrb	w0, [x29,132]
	bl	FlashWaitCmdDone
.L1705:
	add	x0, x20, :lo12:.LANCHOR0
	ldr	w2, [x29,132]
	add	x0, x0, 1652
	ldr	w1, [x29,128]
	madd	x0, x2, x28, x0
	str	x26, [x0,8]
	str	xzr, [x0,16]
	str	w1, [x0,4]
	cbz	w24, .L1707
	add	w1, w22, 1
	umull	x1, w1, w27
	add	x1, x19, x1
	str	x1, [x0,16]
.L1707:
	add	x1, x20, :lo12:.LANCHOR0
	add	x0, x1, x2
	madd	x2, x2, x28, x1
	ldrb	w21, [x0,1848]
	ldrb	w0, [x1,1845]
	strb	w21, [x2,1652]
	cmp	w0, 1
	bne	.L1708
	mov	w0, w21
	bl	NandcWaitFlashReady
	b	.L1709
.L1708:
	mov	w0, w21
	str	x1, [x29,104]
	bl	NandcFlashCs
	ldr	x1, [x29,104]
	ldr	w0, [x29,132]
	add	x1, x1, 708
	ldr	w0, [x1,x0,lsl 2]
	ldr	w1, [x29,128]
	cmp	w0, wzr
	mov	w0, w21
	cset	w2, ne
	bl	FlashWaitReadyEN
	mov	w0, w21
	bl	NandcFlashDeCs
.L1709:
	ldr	w0, [x29,116]
	cmp	w0, 5
	bhi	.L1710
	add	x2, x20, :lo12:.LANCHOR0
	add	x0, x2, x21, sxtw
	ldrb	w0, [x0,1616]
	cbz	w0, .L1710
	ldrb	w1, [x2,761]
	mov	w0, w21
	add	x2, x2, 764
	mov	w3, 0
	bl	HynixSetRRPara
.L1710:
	mov	w0, w21
	bl	NandcFlashCs
	cmp	w23, 1
	bne	.L1711
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L1711
	mov	w0, w21
	bl	flash_enter_slc_mode
.L1711:
	ldr	w1, [x29,128]
	mov	w0, w21
	bl	FlashProgFirstCmd
	ldrb	w2, [x29,124]
	mov	w0, w21
	ldr	x3, [x26,8]
	mov	w1, 1
	ldr	x4, [x26,16]
	bl	NandcXferData
	cbz	w24, .L1712
	ldr	w1, [x29,128]
	mov	w0, w21
	add	x26, x20, :lo12:.LANCHOR0
	bl	FlashProgDpFirstCmd
	ldr	w1, [x29,132]
	add	x0, x26, 708
	ldr	w0, [x0,x1,lsl 2]
	ldr	w1, [x29,128]
	cmp	w0, wzr
	mov	w0, w21
	cset	w2, ne
	bl	FlashWaitReadyEN
	ldr	w2, [x26,12]
	mov	w0, w21
	ldr	w1, [x29,128]
	add	w1, w2, w1
	bl	FlashProgDpSecondCmd
	add	w4, w22, 1
	ldrb	w2, [x29,124]
	mov	w0, w21
	mov	w1, 1
	umull	x4, w4, w27
	add	x4, x19, x4
	ldr	x3, [x4,8]
	ldr	x4, [x4,16]
	bl	NandcXferData
.L1712:
	ldr	w1, [x29,128]
	mov	w0, w21
	add	w22, w22, w24
	bl	FlashProgSecondCmd
	mov	w0, w21
	bl	NandcFlashDeCs
.L1703:
	add	w22, w22, 1
	b	.L1751
.L1752:
	add	x20, x20, :lo12:.LANCHOR0
	mov	x21, 0
	add	x22, x20, 1652
	mov	x24, 24
	ldr	x0, [x20,128]
	bl	NandcIqrWaitFlashReady
.L1714:
	ldrb	w0, [x20,1845]
	cmp	w0, w21
	bls	.L1753
	mov	w0, w21
	bl	FlashWaitCmdDone
	cmp	w23, 1
	bne	.L1715
	ldrb	w0, [x20,120]
	cbz	w0, .L1715
	mul	x0, x21, x24
	ldrb	w0, [x0,x22]
	bl	flash_exit_slc_mode
.L1715:
	add	x21, x21, 1
	b	.L1714
.L1753:
	ldr	w0, [x29,120]
	cbnz	w0, .L1717
.L1725:
	mov	w0, 0
	b	.L1701
.L1717:
	adrp	x24, .LC101
	adrp	x26, .LC100
	adrp	x27, .LC99
	mov	w21, 0
	add	x24, x24, :lo12:.LC101
	add	x26, x26, :lo12:.LC100
	add	x27, x27, :lo12:.LC99
.L1718:
	cmp	w21, w25
	beq	.L1725
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1719
	adrp	x0, .LC98
	ldr	w1, [x19,4]
	add	x0, x0, :lo12:.LC98
	bl	printk
	b	.L1720
.L1719:
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
	bne	.L1721
	ldr	w1, [x19,4]
	mov	x0, x27
	bl	printk
	str	w22, [x19]
.L1721:
	ldr	x0, [x19,16]
	cbz	x0, .L1722
	ldr	w2, [x0]
	add	x0, x20, :lo12:.LANCHOR4
	ldr	x0, [x0,104]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1722
	ldr	w1, [x19,4]
	mov	x0, x26
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1722:
	ldr	x0, [x19,8]
	cbz	x0, .L1720
	add	x20, x20, :lo12:.LANCHOR4
	ldr	w2, [x0]
	ldr	x0, [x20,96]
	ldr	w3, [x0]
	cmp	w2, w3
	beq	.L1720
	ldr	w1, [x19,4]
	mov	x0, x24
	bl	printk
	mov	w0, -1
	str	w0, [x19]
.L1720:
	add	w21, w21, 1
	add	x19, x19, 56
	b	.L1718
.L1701:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 192
	ret
	.size	FlashProgPages, .-FlashProgPages
	.align	2
	.type	FlashTestBlk.part.17, %function
FlashTestBlk.part.17:
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
	cmn	w0, #1
	add	x0, x29, 40
	csetm	w19, eq
	bl	FlashEraseBlocks
	mov	w0, w19
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 160
	ret
	.size	FlashTestBlk.part.17, .-FlashTestBlk.part.17
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
	bcc	.L1756
	mov	w0, w1
	bl	FlashTestBlk.part.17
.L1756:
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
	adrp	x0, .LC104
	mov	w1, 1
	add	x0, x0, :lo12:.LC104
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
.L1760:
	ldrb	w0, [x25,1845]
	cmp	w0, w19
	bls	.L1811
	add	x0, x22, :lo12:.LANCHOR4
	sxtw	x24, w19
	add	x0, x0, 120
	ldrh	w20, [x0,w19,sxtw 1]
	cbnz	w20, .L1790
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
.L1762:
	uxth	w0, w26
	str	w0, [x29,144]
	cmp	w0, w21
	bcs	.L1772
	mov	w0, -1
	strb	w0, [x29,182]
	strb	w0, [x29,183]
	ldr	w0, [x29,116]
	cbz	w0, .L1764
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
	bne	.L1764
	ldr	w1, [x25,12]
	mov	w0, w27
	add	x2, x29, 183
	add	w1, w3, w1
	bl	FlashReadSpare
	ldrb	w0, [x29,182]
	ldrb	w1, [x29,183]
	and	w0, w1, w0
	strb	w0, [x29,182]
.L1764:
	ldr	w0, [x29,120]
	cbz	w0, .L1766
	ldr	x0, [x25,744]
	add	x2, x29, 183
	ldrh	w1, [x0,10]
	ldr	w0, [x28,x24,lsl 2]
	sub	w1, w1, #1
	add	w1, w1, w0
	mov	w0, w27
	add	w1, w1, w20
	bl	FlashReadSpare
.L1766:
	ldr	x0, [x25,744]
	ldrb	w1, [x29,182]
	ldrb	w0, [x0,7]
	cmp	w0, 8
	beq	.L1791
	cmp	w0, 1
	bne	.L1767
.L1791:
	mov	w0, 1
	cbz	w1, .L1769
	ldrb	w0, [x29,183]
	cmp	w0, wzr
	cset	w0, eq
	b	.L1769
.L1767:
	cmp	w1, 255
	mov	w0, 1
	bne	.L1769
	ldrb	w0, [x29,183]
	cmp	w0, 255
	cset	w0, ne
.L1769:
	ldr	x1, [x29,128]
	tbz	x1, 2, .L1770
	ldr	w1, [x28,x24,lsl 2]
	mov	w0, w27
	add	w1, w20, w1
	bl	SandiskProgTestBadBlock
.L1770:
	cbz	w0, .L1771
	adrp	x0, .LC105
	mov	w1, w19
	mov	w2, w26
	add	x0, x0, :lo12:.LC105
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
	bgt	.L1772
.L1771:
	ldr	w0, [x29,172]
	add	w26, w26, 1
	add	w20, w20, w0
	b	.L1762
.L1772:
	adrp	x0, .LC106
	mov	w1, w19
	add	x0, x0, :lo12:.LC106
	mov	w2, w23
	bl	printk
	ldrb	w0, [x25,1845]
	ldr	w1, [x29,168]
	mul	w0, w1, w0
	cmp	w23, w0
	blt	.L1774
	ldrh	w2, [x25,188]
	mov	w1, 0
	ldr	x0, [x25,1856]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1774:
	cbnz	w19, .L1776
	add	x0, x22, :lo12:.LANCHOR4
	adrp	x26, .LC107
	mov	w20, w19
	add	x26, x26, :lo12:.LC107
	mov	w23, 1
	ldrh	w27, [x0,68]
.L1777:
	ldrb	w0, [x25,9]
	cmp	w0, w27
	bls	.L1812
	mov	w0, w27
	bl	FlashTestBlk
	cbz	w0, .L1778
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
.L1778:
	add	w27, w27, 1
	uxth	w27, w27
	b	.L1777
.L1812:
	adrp	x26, .LC107
	ldr	w23, [x29,124]
	sub	w27, w21, #50
	add	x26, x26, :lo12:.LC107
.L1780:
	cmp	w23, w27
	ble	.L1813
	mov	w0, w23
	bl	FlashTestBlk
	cbz	w0, .L1781
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
.L1781:
	sub	w2, w23, #1
	uxth	w23, w2
	b	.L1780
.L1813:
	add	x1, x22, :lo12:.LANCHOR4
	ldrb	w0, [x25,9]
	ldr	w1, [x1,68]
	sub	w0, w0, w1
	cmp	w20, w0
	bcc	.L1776
	ldrh	w2, [x25,188]
	mov	w1, 0
	ldr	x0, [x25,1856]
	lsl	w2, w2, 9
	bl	ftl_memset
.L1776:
	adrp	x23, .LC108
	ldr	w27, [x29,124]
	mul	w26, w19, w21
	add	x23, x23, :lo12:.LC108
	add	x20, x22, :lo12:.LANCHOR4
.L1784:
	mov	w2, w27
	mov	x0, x23
	mov	w1, w19
	bl	printk
	ldr	x2, [x25,1856]
.L1785:
	ubfx	x0, x27, 5, 11
	ldr	w0, [x2,x0,lsl 2]
	lsr	w0, w0, w27
	and	w1, w0, 1
	tbz	x0, 0, .L1814
	sub	w27, w27, #1
	uxth	w27, w27
	b	.L1785
.L1814:
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
	cbz	w0, .L1790
	sub	w27, w27, #1
	uxth	w27, w27
	b	.L1784
.L1790:
	add	w19, w19, 1
	uxtb	w19, w19
	b	.L1760
.L1811:
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
	uxth	w25, w0
	mov	w28, 0
	ldrb	w0, [x20,120]
	uxtb	w26, w1
	str	w0, [x29,124]
	mov	w21, w28
	str	w25, [x20,2108]
	mov	w19, w28
	adrp	x24, .LANCHOR2
	mov	w27, 56
.L1816:
	ldrh	w0, [x20,1952]
	cmp	w0, w28
	bls	.L1849
	add	x1, x24, :lo12:.LANCHOR2
	umull	x0, w28, w27
	ldr	x1, [x1,-88]
	str	wzr, [x1,x0]
	add	x0, x20, 1984
	mov	w1, w25
	ldrb	w0, [x0,w28,sxtw]
	bl	V2P_block
	uxth	w23, w0
	mov	w1, w23
	cbz	w26, .L1817
	str	x1, [x29,112]
	bl	IsBlkInVendorPart
	ldr	x1, [x29,112]
	cbnz	w0, .L1818
.L1817:
	mov	w0, w1
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L1819
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
	b	.L1818
.L1819:
	add	w19, w19, 1
	uxth	w19, w19
.L1818:
	add	w2, w28, 1
	uxth	w28, w2
	b	.L1816
.L1849:
	cbz	w21, .L1822
	ldr	w0, [x29,124]
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
.L1823:
	cmp	w21, w24, uxth
	bls	.L1850
	add	x1, x20, :lo12:.LANCHOR2
	mul	x0, x24, x27
	ldr	x1, [x1,-88]
	add	x2, x1, x0
	ldr	w0, [x1,x0]
	cmn	w0, #1
	bne	.L1824
	ldr	w0, [x2,4]
	add	w19, w19, 1
	lsr	w0, w0, 10
	uxth	w19, w19
	bl	FtlBbmMapBadBlock
.L1824:
	add	x24, x24, 1
	b	.L1823
.L1850:
	cbnz	w26, .L1826
	mov	w0, 6
	uxth	w23, w23
	str	w0, [x29,124]
	mov	w27, 1
	b	.L1827
.L1826:
	add	x0, x22, :lo12:.LANCHOR0
	mov	w23, 1
	ldrh	w27, [x0,2028]
	lsr	w0, w27, 2
	str	w0, [x29,124]
.L1827:
	mov	w24, 0
	add	x22, x22, :lo12:.LANCHOR0
.L1837:
	mov	w28, 0
	mov	w21, w28
.L1828:
	ldrh	w0, [x22,1952]
	cmp	w0, w28
	bls	.L1851
	add	x1, x20, :lo12:.LANCHOR2
	mov	w0, 56
	umull	x0, w28, w0
	ldr	x1, [x1,-88]
	str	wzr, [x1,x0]
	add	x0, x22, 1984
	mov	w1, w25
	ldrb	w0, [x0,w28,sxtw]
	bl	V2P_block
	uxth	w2, w0
	str	w2, [x29,112]
	mov	w1, w2
	cbz	w26, .L1829
	str	x1, [x29,104]
	bl	IsBlkInVendorPart
	ldr	x1, [x29,104]
	cbnz	w0, .L1830
.L1829:
	mov	w0, w1
	bl	FtlBbmIsBadBlock
	cbnz	w0, .L1830
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
	ldrh	w0, [x22,2038]
	mul	w0, w21, w0
	add	w21, w21, 1
	sdiv	w0, w0, w2
	uxth	w21, w21
	add	x0, x3, x0, sxtw 2
	str	x0, [x1,16]
.L1830:
	add	w2, w28, 1
	uxth	w28, w2
	b	.L1828
.L1851:
	cbz	w21, .L1822
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, w21
	mov	w2, w23
	mov	w3, 1
	mov	x28, 0
	ldr	x0, [x0,-88]
	bl	FlashProgPages
.L1833:
	cmp	w21, w28, uxth
	bls	.L1852
	add	x1, x20, :lo12:.LANCHOR2
	mov	x0, 56
	mul	x0, x28, x0
	ldr	x3, [x1,-88]
	add	x1, x3, x0
	ldr	w0, [x3,x0]
	cmn	w0, #1
	bne	.L1834
	ldr	w0, [x1,4]
	add	w19, w19, 1
	lsr	w0, w0, 10
	uxth	w19, w19
	bl	FtlBbmMapBadBlock
	b	.L1835
.L1834:
	cbz	w26, .L1835
	ldr	w0, [x1,4]
	mov	w1, 1
	lsr	w0, w0, 10
	bl	FtlFreeSysBlkQueueIn
.L1835:
	add	x28, x28, 1
	b	.L1833
.L1852:
	ldr	w0, [x29,124]
	add	w24, w24, w0
	uxth	w24, w24
	cmp	w24, w27
	bcc	.L1837
	cmp	w25, 63
	bhi	.L1822
	add	x20, x20, :lo12:.LANCHOR2
	mov	w1, w23
	mov	w2, w21
	ldr	x0, [x20,-88]
	bl	FlashEraseBlocks
.L1822:
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
	str	x27, [sp,80]
	stp	x19, x20, [sp,16]
	add	x2, x22, :lo12:.LANCHOR2
	adrp	x19, .LANCHOR4
	adrp	x20, .LANCHOR0
	mov	x21, 0
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	add	x24, x19, :lo12:.LANCHOR4
	add	x20, x20, :lo12:.LANCHOR0
	ldr	x0, [x2,-64]
	add	x23, x20, 2112
	ldr	x2, [x2,2112]
	str	x2, [x24,152]
	ldrh	w2, [x20,2036]
	str	x0, [x24,144]
	bl	ftl_memset
.L1854:
	ldrh	w1, [x20,1974]
	mov	w0, w21
	add	x21, x21, 1
	cmp	w0, w1
	bge	.L1861
	add	x1, x22, :lo12:.LANCHOR2
	ldr	x3, [x24,144]
	ldrh	w2, [x1,2232]
	add	x1, x23, x21, lsl 3
	ldr	x1, [x1,24]
	mul	w0, w0, w2
	add	x0, x3, x0, sxtw 2
	lsl	w2, w2, 2
	bl	ftl_memcpy
	b	.L1854
.L1861:
	add	x19, x19, :lo12:.LANCHOR4
	mov	w1, 255
	add	x19, x19, 136
	mov	w2, 16
	adrp	x25, .LC109
	adrp	x26, .LC110
	ldr	x21, [x19,16]
	mov	w24, 0
	add	x25, x25, :lo12:.LC109
	add	x26, x26, :lo12:.LC110
	mov	x0, x21
	bl	ftl_memset
	mov	w0, -3887
	strh	w0, [x21]
	ldr	w0, [x23,8]
	str	w0, [x21,4]
	ldrh	w0, [x20,2112]
	strh	w0, [x21,2]
	ldrh	w0, [x23,4]
	strh	w0, [x21,8]
	ldrh	w0, [x23,6]
	strh	w0, [x21,10]
	ldr	w0, [x20,1948]
	strh	w0, [x21,12]
.L1856:
	add	x27, x22, :lo12:.LANCHOR2
	ldrh	w1, [x20,2112]
	ldrh	w2, [x20,2114]
	ldrh	w3, [x20,2116]
	ldr	x0, [x27,-64]
	str	x0, [x19,8]
	ldr	x0, [x27,2112]
	str	x0, [x19,16]
	orr	w0, w2, w1, lsl 10
	str	w0, [x19,4]
	mov	x0, x25
	ldrh	w4, [x21,10]
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
	blt	.L1857
	ldr	w0, [x20,2120]
	strh	wzr, [x20,2114]
	add	w0, w0, 1
	str	w0, [x20,2120]
	str	w0, [x21,4]
	ldrh	w0, [x20,2112]
	ldrh	w1, [x20,2116]
	strh	w0, [x21,8]
	strh	w0, [x20,2116]
	ldr	x0, [x27,-88]
	strh	w1, [x20,2112]
	lsl	w1, w1, 10
	str	w1, [x19,4]
	str	w1, [x0,4]
	mov	w1, 1
	mov	w2, w1
	ldr	x0, [x27,-88]
	bl	FlashEraseBlocks
	mov	w1, 1
	mov	x0, x19
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
.L1857:
	ldrh	w0, [x20,2114]
	add	w0, w0, 1
	strh	w0, [x20,2114]
	ldr	w0, [x19]
	cmn	w0, #1
	bne	.L1858
	ldr	w1, [x19,4]
	mov	x0, x26
	bl	printk
	b	.L1856
.L1858:
	cbnz	w24, .L1862
	mov	w24, 1
	b	.L1856
.L1862:
	mov	w0, 0
	ldr	x27, [sp,80]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
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
.L1864:
	cmp	x20, x23
	bne	.L1865
	ldrh	w0, [x21,-8]
	ldr	w3, [x21,876]
	lsr	w2, w0, 1
	add	w4, w2, 1
	mul	w1, w0, w3
	add	w1, w4, w1, lsr 2
	ldr	w4, [x21,160]
	uxth	w1, w1
	cbz	w4, .L1866
	ldr	w4, [x21,220]
	cmp	w4, 29
	bhi	.L1866
	cmp	w4, 2
	mov	w1, 0
	bls	.L1867
	tbz	x0, 0, .L1892
	cbz	w3, .L1867
.L1892:
	mov	w1, w2
	b	.L1866
.L1865:
	ldrb	w0, [x20,8]
	mov	w1, 0
	cmp	w0, 1
	bne	.L1867
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w2, [x0,1972]
	cmp	w2, 1
	beq	.L1867
	ldrb	w0, [x0,120]
	cbnz	w0, .L1867
	ldr	w2, [x21,160]
	ldrh	w0, [x21,-8]
	lsr	w1, w0, 3
	cbz	w2, .L1866
	ldr	w2, [x21,220]
	cmp	w2, 1
	bhi	.L1866
	mov	w1, 7
	mul	w1, w0, w1
	lsr	w1, w1, 3
.L1866:
	cbz	w1, .L1867
	sub	w1, w1, #1
	uxth	w1, w1
.L1867:
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
	cbz	w0, .L1869
	adrp	x25, .LANCHOR0
	mov	x3, 56
	add	x0, x25, :lo12:.LANCHOR0
	mov	w6, 65535
	ldrh	w5, [x0,1952]
	mov	x0, 0
	mov	w26, w0
	b	.L1870
.L1869:
	ubfiz	x0, x22, 1, 16
	ldr	x1, [x19,-40]
	mov	w2, -1
	strh	w2, [x1,x0]
	b	.L1914
.L1873:
	ldr	x4, [x19,-88]
	madd	x1, x0, x3, x4
	str	xzr, [x1,16]
	str	xzr, [x1,8]
	add	x1, x20, x0, lsl 1
	ldrh	w1, [x1,16]
	cmp	w1, w6
	beq	.L1872
	umull	x2, w26, w3
	add	w26, w26, 1
	lsl	w1, w1, 10
	add	x2, x4, x2
	uxth	w26, w26
	str	w1, [x2,4]
.L1872:
	add	x0, x0, 1
.L1870:
	cmp	w5, w0, uxth
	bhi	.L1873
	cmp	x20, x19
	uxtw	x27, w22
	bne	.L1874
	ldr	w0, [x19,160]
	cbz	w0, .L1874
	ldr	x0, [x19,-80]
	ldrh	w0, [x0,x27,lsl 1]
	cmp	w0, 30
	bls	.L1874
	strb	wzr, [x19,8]
.L1874:
	ldrb	w0, [x20,8]
	cbnz	w0, .L1875
	lsl	x0, x27, 1
	ldr	x1, [x19,-80]
	mov	w2, 2
	ldrh	w3, [x1,x0]
	cbz	w3, .L1915
	add	x2, x25, :lo12:.LANCHOR0
	ldrh	w2, [x2,2016]
	add	w2, w3, w2
.L1915:
	strh	w2, [x1,x0]
	mov	w1, 0
	ldr	w0, [x19,204]
	add	w0, w0, 1
	str	w0, [x19,204]
	mov	w0, w22
	bl	ftl_set_blk_mode
	b	.L1878
.L1875:
	lsl	x0, x27, 1
	ldr	x2, [x19,-80]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
	ldr	w0, [x19,208]
	add	w0, w0, 1
	str	w0, [x19,208]
	mov	w0, w22
	bl	ftl_set_blk_mode.part.8
.L1878:
	lsl	x27, x27, 1
	ldr	x0, [x19,-80]
	ldr	w1, [x19,216]
	ldrh	w0, [x0,x27]
	cmp	w0, w1
	bls	.L1879
	str	w0, [x19,216]
.L1879:
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
.L1880:
	cmp	w26, w0, uxth
	bls	.L1916
	ldr	x1, [x19,-88]
	madd	x1, x0, x3, x1
	add	x0, x0, 1
	ldr	w2, [x1,4]
	and	w2, w2, -1024
	str	w2, [x1,4]
	b	.L1880
.L1916:
	ldrb	w1, [x20,8]
	mov	w2, w26
	ldr	x0, [x19,-88]
	mov	x28, 0
	bl	FlashEraseBlocks
	mov	w1, w28
	mov	x3, 56
.L1882:
	cmp	w26, w28, uxth
	bls	.L1917
	mul	x0, x28, x3
	ldr	x2, [x19,-88]
	add	x4, x2, x0
	ldr	w2, [x2,x0]
	cmn	w2, #1
	bne	.L1883
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
.L1883:
	add	x28, x28, 1
	b	.L1882
.L1917:
	cbz	w1, .L1885
	mov	w0, w22
	bl	update_multiplier_value
	bl	FtlBbmTblFlush
.L1885:
	ldrb	w0, [x20,7]
	cbnz	w0, .L1886
	ldr	x0, [x19,-40]
	mov	w1, -1
	strh	w1, [x0,x27]
.L1914:
	mov	w0, w22
	bl	INSERT_DATA_LIST
	b	.L1864
.L1886:
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
	cbz	w0, .L1920
	adrp	x22, .LANCHOR0
	adrp	x24, .LC111
	add	x22, x22, :lo12:.LANCHOR0
	add	x24, x24, :lo12:.LC111
.L1928:
	ldrh	w0, [x22,1952]
	cmp	w0, w21
	bls	.L1929
	add	x0, x22, 1984
	mov	w1, w23
	mov	w20, 0
	ldrb	w0, [x0,w21,sxtw]
	bl	V2P_block
	uxth	w25, w0
.L1921:
	ldrh	w0, [x19,956]
	cmp	w0, w20
	bls	.L1930
	add	x0, x19, 960
	ldrh	w0, [x0,w20,sxtw 1]
	cmp	w0, w25
	bne	.L1922
	mov	w1, w25
	mov	x0, x24
	bl	printk
	mov	w0, w25
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
	ldrh	w2, [x19,956]
	mov	w3, w20
.L1923:
	cmp	w3, w2
	bcs	.L1931
	add	x4, x19, 960
	add	w0, w3, 1
	ldrh	w1, [x4,w0,sxtw 1]
	strh	w1, [x4,w3,sxtw 1]
	uxth	w3, w0
	b	.L1923
.L1931:
	sub	w2, w2, #1
	strh	w2, [x19,956]
.L1922:
	add	w20, w20, 1
	uxth	w20, w20
	b	.L1921
.L1930:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L1928
.L1929:
	bl	FtlGcReFreshBadBlk
.L1920:
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
	cbnz	w2, .L1933
	ldrh	w3, [x1,296]
	cmp	w3, w19
	bne	.L1934
	mov	w0, -1
	strh	w0, [x1,296]
	b	.L1935
.L1934:
	ldrh	w3, [x0,#:lo12:.LANCHOR2]
	mov	w0, w2
	cmp	w3, w19
	beq	.L1936
	ldrh	w3, [x1,48]
	cmp	w3, w19
	beq	.L1936
	ldrh	w1, [x1,96]
	cmp	w1, w19
	beq	.L1936
.L1935:
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
	b	.L1936
.L1933:
	mov	w0, w19
	bl	List_update_data_list
	mov	w0, 0
.L1936:
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
	beq	.L1941
	add	x0, x2, :lo12:.LANCHOR2
	ubfiz	x3, x1, 1, 16
	ldr	x0, [x0,-40]
	ldrh	w19, [x0,x3]
	cbnz	w19, .L1942
	adrp	x0, .LC112
	mov	w2, w19
	add	x0, x0, :lo12:.LC112
	bl	printk
	b	.L1946
.L1942:
	sub	w19, w19, #1
	strh	w19, [x0,x3]
.L1941:
	add	x19, x2, :lo12:.LANCHOR2
	mov	w0, 65535
	ldrh	w2, [x19,2320]
	cmp	w2, w0
	bne	.L1944
	strh	w1, [x19,2320]
.L1946:
	mov	w0, 0
	b	.L1943
.L1944:
	cmp	w2, w1
	str	x1, [x29,40]
	mov	w0, 0
	beq	.L1943
	mov	w0, w2
	bl	update_vpc_list
	cmp	w0, wzr
	ldr	x1, [x29,40]
	cset	w0, ne
	strh	w1, [x19,2320]
.L1943:
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
	cbz	w1, .L1947
	mov	x19, x0
	ldrb	w0, [x0,6]
	adrp	x20, .LANCHOR2
	mov	w21, 65535
	add	x0, x0, 8
	add	x20, x20, :lo12:.LANCHOR2
	adrp	x22, .LANCHOR0
	add	x20, x20, 996
	ldrh	w0, [x19,x0,lsl 1]
.L1950:
	cmp	w0, w21
	bne	.L1958
.L1952:
	ldrb	w0, [x19,6]
	add	x1, x22, :lo12:.LANCHOR0
	add	w0, w0, 1
	ldrh	w1, [x1,1952]
	uxtb	w0, w0
	strb	w0, [x19,6]
	cmp	w1, w0
	bne	.L1951
	ldrh	w0, [x19,2]
	strb	wzr, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
.L1951:
	ldrb	w0, [x19,6]
	add	x0, x0, 8
	ldrh	w0, [x19,x0,lsl 1]
	b	.L1950
.L1958:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L1953
	add	x0, x22, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbnz	w0, .L1953
	ldrh	w0, [x19,2]
	ldrh	w0, [x20,w0,sxtw 1]
	cmp	w0, w21
	bne	.L1953
	ldrh	w0, [x19,4]
	sub	w0, w0, #1
	strh	w0, [x19,4]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	ldrh	w1, [x19,4]
	cbnz	w1, .L1952
	ldrh	w0, [x19,2]
	strb	w1, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
	b	.L1947
.L1953:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w2, [x0,120]
	cbz	w2, .L1947
	cmp	w1, 1
	bne	.L1947
	ldrh	w2, [x19,2]
	ldrh	w1, [x0,2028]
	cmp	w2, w1
	bcc	.L1947
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
.L1947:
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
.L1960:
	cmp	w0, w22
	adrp	x20, .LANCHOR0
	bne	.L1975
.L1961:
	ldrb	w0, [x19,6]
	add	x1, x23, :lo12:.LANCHOR0
	add	w0, w0, 1
	ldrh	w1, [x1,1952]
	uxtb	w0, w0
	strb	w0, [x19,6]
	cmp	w1, w0
	bne	.L1962
	ldrh	w0, [x19,2]
	strb	wzr, [x19,6]
	add	w0, w0, 1
	strh	w0, [x19,2]
.L1962:
	ldrb	w0, [x19,6]
	add	x0, x0, 8
	ldrh	w0, [x19,x0,lsl 1]
	b	.L1960
.L1975:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L1963
	add	x1, x23, :lo12:.LANCHOR0
	ldrb	w1, [x1,120]
	cbnz	w1, .L1963
	ldrh	w1, [x19,2]
	ldrh	w1, [x21,w1,sxtw 1]
	cmp	w1, w22
	bne	.L1963
	ldrh	w0, [x19,4]
	sub	w0, w0, #1
	strh	w0, [x19,4]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L1961
.L1963:
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
.L1964:
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w1, [x19,6]
	ldrh	w2, [x0,1952]
.L1966:
	add	w1, w1, 1
	uxtb	w1, w1
	cmp	w1, w2
	bne	.L1965
	ldrh	w1, [x19,2]
	add	w1, w1, 1
	strh	w1, [x19,2]
	mov	w1, 0
.L1965:
	add	x0, x19, x1, sxtw 1
	ldrh	w0, [x0,16]
	cmp	w0, w23
	beq	.L1966
	ldrb	w0, [x19,8]
	strb	w1, [x19,6]
	cmp	w0, 1
	bne	.L1971
	add	x0, x20, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbnz	w0, .L1968
	ldrh	w0, [x19,2]
	ldrh	w0, [x21,w0,sxtw 1]
	cmp	w0, w24
	bne	.L1968
	ldrh	w1, [x19,4]
	cbz	w1, .L1968
	ldrh	w0, [x19]
	sub	w1, w1, #1
	strh	w1, [x19,4]
	bl	decrement_vpc_count
	b	.L1964
.L1968:
	add	x20, x20, :lo12:.LANCHOR0
	ldrb	w0, [x20,120]
	cbz	w0, .L1971
	ldrh	w1, [x19,2]
	ldrh	w0, [x20,2028]
	cmp	w1, w0
	bcc	.L1971
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
.L1971:
	mov	w0, w22
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 64
	ret
	.size	get_new_active_ppa, .-get_new_active_ppa
	.align	2
	.global	FtlVpcTblFlush
	.type	FtlVpcTblFlush, %function
FtlVpcTblFlush:
	stp	x29, x30, [sp, -80]!
	adrp	x0, .LANCHOR2
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	add	x19, x0, :lo12:.LANCHOR2
	adrp	x20, .LANCHOR4
	add	x20, x20, :lo12:.LANCHOR4
	str	x25, [sp,64]
	add	x20, x20, 136
	mov	w25, 65535
	stp	x21, x22, [sp,32]
	ldr	x22, [x19,2112]
	add	x21, x19, 224
	stp	x23, x24, [sp,48]
	ldr	x1, [x19,-64]
	adrp	x24, .LANCHOR0
	str	x1, [x20,8]
	add	x23, x24, :lo12:.LANCHOR0
	str	x22, [x20,16]
	ldrh	w2, [x19,280]
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
	ldrh	w1, [x23,1974]
	ldrh	w0, [x19,2]
	strb	w1, [x19,234]
	ldrb	w1, [x19,6]
	str	w2, [x19,224]
	mov	w2, 54
	orr	w0, w1, w0, lsl 6
	movk	w2, 0x5000, lsl 16
	strh	w0, [x19,240]
	str	w2, [x19,228]
	ldrb	w0, [x19,8]
	strb	w0, [x19,235]
	ldrh	w1, [x19,48]
	strh	w1, [x19,242]
	ldrh	w1, [x19,50]
	ldrb	w2, [x19,54]
	ldrb	w0, [x19,56]
	orr	w1, w2, w1, lsl 6
	strb	w0, [x19,236]
	strh	w1, [x19,244]
	ldrb	w0, [x19,104]
	ldrh	w1, [x19,96]
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
	str	w0, [x19,260]
	ldrh	w2, [x23,2036]
	ldr	x0, [x20,8]
	strh	w1, [x19,248]
	mov	w1, 255
	bl	ftl_memset
	ldr	x0, [x20,8]
	mov	x1, x21
	mov	w2, 48
	mov	x21, x24
	bl	ftl_memcpy
	ldr	x0, [x20,8]
	ldrh	w2, [x23,1960]
	ldr	x1, [x19,-40]
	add	x0, x0, 48
	lsl	w2, w2, 1
	bl	ftl_memcpy
	ldrh	w2, [x23,1960]
	ldr	x1, [x20,8]
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
.L1977:
	ldr	x0, [x19,-64]
	str	x0, [x20,8]
	ldrh	w1, [x19,280]
	ldr	x0, [x19,2112]
	str	x0, [x20,16]
	ldrh	w0, [x19,282]
	orr	w1, w0, w1, lsl 10
	str	w1, [x20,4]
	mov	w1, 1
	mov	x0, x20
	mov	w2, w1
	mov	w3, w1
	bl	FlashProgPages
	add	x1, x21, :lo12:.LANCHOR0
	ldrh	w0, [x19,282]
	ldrh	w1, [x1,2028]
	sub	w1, w1, #1
	cmp	w0, w1
	blt	.L1978
	ldrh	w0, [x19,280]
	ldrh	w25, [x19,284]
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
.L1978:
	ldrh	w1, [x19,282]
	ldr	w0, [x20]
	add	w1, w1, 1
	cmn	w0, #1
	uxth	w1, w1
	strh	w1, [x19,282]
	bne	.L1979
	cmp	w1, 1
	bne	.L1977
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w1, [x0,2028]
	sub	w1, w1, #1
	strh	w1, [x19,282]
	b	.L1977
.L1979:
	cmp	w1, 1
	beq	.L1977
	mov	w0, 65535
	cmp	w25, w0
	beq	.L1981
	mov	w0, w25
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
.L1981:
	mov	w0, 0
	ldr	x25, [sp,64]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x29, x30, [sp], 80
	ret
	.size	FtlVpcTblFlush, .-FtlVpcTblFlush
	.align	2
	.global	FtlSuperblockPowerLostFix
	.type	FtlSuperblockPowerLostFix, %function
FtlSuperblockPowerLostFix:
	stp	x29, x30, [sp, -144]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR2
	mov	x19, x0
	add	x0, x20, :lo12:.LANCHOR2
	str	x25, [sp,64]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	mov	w23, 0
	ldr	x22, [x0,904]
	adrp	x0, .LANCHOR0
	add	x1, x0, :lo12:.LANCHOR0
	mov	x21, x0
	ldrb	w1, [x1,120]
	cbz	w1, .L1986
	ldrb	w0, [x19,8]
	cmp	w0, 1
	cset	w23, eq
.L1986:
	mov	w24, 7
	mov	w25, -1
.L1987:
	subs	w24, w24, #1
	beq	.L1988
	ldrh	w0, [x19,4]
	cbz	w0, .L1988
	mov	x0, x19
	bl	get_new_active_ppa
	str	w0, [x29,92]
	add	x1, x20, :lo12:.LANCHOR2
	str	w25, [x29,112]
	mov	w2, w23
	mov	w3, 0
	ldr	x0, [x1,-64]
	str	x0, [x29,96]
	ldr	x0, [x1,2112]
	str	x0, [x29,104]
	str	w25, [x22,8]
	str	w25, [x22,12]
	ldrh	w0, [x19]
	strh	w0, [x22,2]
	strh	wzr, [x22]
	ldr	x22, [x1,2112]
	ldr	w0, [x1,200]
	str	w0, [x22,4]
	add	w0, w0, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x1,200]
	add	x0, x29, 88
	mov	w1, 1
	bl	FlashProgPages
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L1987
.L1988:
	add	x20, x20, :lo12:.LANCHOR2
	ldrh	w0, [x19]
	ldrh	w3, [x19,4]
	lsl	x0, x0, 1
	ldr	x2, [x20,-40]
	ldrh	w1, [x2,x0]
	sub	w1, w1, w3
	strh	w1, [x2,x0]
	add	x0, x21, :lo12:.LANCHOR0
	strb	wzr, [x19,6]
	strh	wzr, [x19,4]
	ldp	x21, x22, [sp,32]
	ldrh	w0, [x0,2026]
	strh	w0, [x19,2]
	ldp	x23, x24, [sp,48]
	ldp	x19, x20, [sp,16]
	ldr	x25, [sp,64]
	ldp	x29, x30, [sp], 144
	ret
	.size	FtlSuperblockPowerLostFix, .-FtlSuperblockPowerLostFix
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
	sub	w1, w1, #1
	cmp	w2, w1
	blt	.L1998
	ubfiz	x1, x0, 1, 16
	ldrh	w24, [x20,x1]
	cbz	w24, .L1998
	ldr	w0, [x19,52]
	cbnz	w0, .L1998
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
	bcc	.L1999
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L1999:
	adrp	x20, .LANCHOR4
	mov	w21, 0
	add	x20, x20, :lo12:.LANCHOR4
	adrp	x26, .LANCHOR2
	add	x20, x20, 136
.L2000:
	ldrh	w0, [x19,6]
	cmp	w0, w21
	bls	.L2008
	ubfiz	x25, x21, 2, 16
	ldr	w0, [x23,x25]
	cmp	w24, w0, lsr 10
	bne	.L2001
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
	bne	.L2002
	str	wzr, [x23,x25]
	b	.L2001
.L2002:
	ldr	x2, [x20,8]
	mov	x0, x19
	mov	w1, w21
	bl	FtlMapWritePage
.L2001:
	add	w21, w21, 1
	uxth	w21, w21
	b	.L2000
.L2008:
	mov	w0, w24
	mov	w1, 1
	bl	FtlFreeSysBlkQueueIn
	str	wzr, [x19,52]
.L1998:
	add	x22, x22, :lo12:.LANCHOR0
	ldrh	w1, [x19,2]
	ldrh	w0, [x22,2028]
	cmp	w1, w0
	bcc	.L2004
	mov	x0, x19
	bl	ftl_map_blk_alloc_new_blk
.L2004:
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x29, x30, [sp], 80
	ret
	.size	ftl_map_blk_gc, .-ftl_map_blk_gc
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
	bne	.L2010
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
	b	.L2011
.L2010:
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
.L2012:
	ldrh	w2, [x19,6]
	cmp	w2, w1
	bls	.L2015
	ubfiz	x4, x1, 2, 16
	ldr	w2, [x23,x4]
	cmp	w22, w2, lsr 10
	bne	.L2013
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
.L2013:
	add	w1, w1, 1
	uxth	w1, w1
	b	.L2012
.L2015:
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
.L2011:
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
	stp	x29, x30, [sp, -96]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR4
	str	x27, [sp,80]
	stp	x21, x22, [sp,32]
	add	x20, x20, :lo12:.LANCHOR4
	adrp	x22, .LANCHOR2
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	mov	x19, x0
	mov	w21, w1
	mov	x23, x2
	adrp	x24, .LANCHOR0
	mov	x25, x22
	mov	w26, 65535
	add	x20, x20, 136
.L2022:
	add	x1, x22, :lo12:.LANCHOR2
	ldr	w0, [x1,180]
	add	w0, w0, 1
	str	w0, [x1,180]
	add	x0, x24, :lo12:.LANCHOR0
	ldrh	w1, [x19,2]
	ldrh	w0, [x0,2028]
	sub	w0, w0, #1
	cmp	w1, w0
	bge	.L2017
	ldrh	w0, [x19]
	cmp	w0, w26
	bne	.L2018
.L2017:
	mov	x0, x19
	bl	Ftl_write_map_blk_to_last_page
.L2018:
	ldrh	w1, [x19]
	mov	w2, 16
	ldr	x0, [x19,16]
	ldrh	w27, [x0,x1,lsl 1]
	mov	w1, 0
	ldrh	w0, [x19,2]
	str	x23, [x20,8]
	orr	w0, w0, w27, lsl 10
	str	w0, [x20,4]
	add	x0, x25, :lo12:.LANCHOR2
	ldr	x0, [x0,2112]
	str	x0, [x20,16]
	bl	ftl_memset
	ldr	x0, [x20,16]
	ldr	w1, [x19,48]
	str	w1, [x0,4]
	strh	w21, [x0,8]
	ldrh	w1, [x19,4]
	strh	w1, [x0]
	mov	w1, 1
	strh	w27, [x0,2]
	mov	w2, w1
	mov	x0, x20
	mov	w3, w1
	bl	FlashProgPages
	ldrh	w0, [x19,2]
	add	w0, w0, 1
	uxth	w0, w0
	strh	w0, [x19,2]
	cmp	w0, 1
	beq	.L2022
	ldr	w0, [x20]
	cmn	w0, #1
	beq	.L2022
	ldr	x0, [x19,40]
	ldr	w1, [x20,4]
	str	w1, [x0,w21,uxtw 2]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldr	x27, [sp,80]
	ldp	x29, x30, [sp], 96
	ret
	.size	FtlMapWritePage, .-FtlMapWritePage
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
.L2027:
	uxth	x19, w4
	cmp	w19, w0
	bcs	.L2039
	add	x4, x4, 1
	add	x1, x25, x4, lsl 4
	ldrh	w1, [x1,-16]
	cmp	w1, w20
	bne	.L2027
.L2028:
	cbnz	w22, .L2029
	add	x0, x23, :lo12:.LANCHOR2
	ldr	x1, [x0,144]
	add	x1, x1, x19, lsl 4
	ldr	x0, [x1,8]
	ldr	w0, [x0,x24,lsl 2]
	str	w0, [x21]
	b	.L2030
.L2029:
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
.L2030:
	add	x23, x23, :lo12:.LANCHOR2
	ldr	x0, [x23,144]
	add	x19, x0, x19, lsl 4
	ldr	w0, [x19,4]
	cmn	w0, #1
	beq	.L2036
	add	w0, w0, 1
	str	w0, [x19,4]
	b	.L2036
.L2039:
	bl	select_l2p_ram_region
	uxth	x19, w0
	ubfiz	x2, x19, 4, 16
	mov	w1, w0
	add	x3, x25, x2
	ldrh	w4, [x25,x2]
	mov	w2, 65535
	cmp	w4, w2
	beq	.L2033
	ldr	w2, [x3,4]
	tbz	w2, #31, .L2033
	str	x1, [x29,88]
	bl	flush_l2p_region
	ldr	x1, [x29,88]
.L2033:
	mov	w0, w20
	bl	load_l2p_region
	b	.L2028
.L2036:
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
	cbnz	w1, .L2041
	ldr	x19, [x2,-16]
	cbz	x19, .L2042
	ldrh	w3, [x2,-8]
	mov	x4, -6148914691236517206
	ldr	x2, [x2,-56]
	mov	w5, 65535
	sub	x19, x19, x2
	asr	x19, x19, 1
	madd	x19, x4, x19, x19
	mov	w4, 6
	uxth	w19, w19
.L2043:
	cmp	w1, w3
	beq	.L2042
	cmp	w19, w0
	bne	.L2044
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
	b	.L2042
.L2044:
	umull	x19, w19, w4
	ldrh	w19, [x2,x19]
	cmp	w19, w5
	beq	.L2042
	add	w1, w1, 1
	uxth	w1, w1
	b	.L2043
.L2041:
	add	w1, w1, 1
	strh	w1, [x3,x22]
.L2042:
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
.L2050:
	cmp	w25, w23
	bge	.L2067
	ldr	w0, [x29,116]
	sxtw	x28, w25
	cmp	w25, w0
	bne	.L2051
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
.L2052:
	cmp	w21, w24
	bge	.L2067
	ldrh	w0, [x23]
	mov	w1, 1
	mov	w2, w1
	orr	w0, w21, w0, lsl 10
	str	w0, [x19,4]
	mov	x0, x19
	bl	FlashReadPages
	ldr	w0, [x19]
	cmn	w0, #1
	beq	.L2053
	ldrh	w0, [x26,8]
	cmp	w0, w22
	bcs	.L2053
	ldrh	w2, [x26]
	ldrh	w1, [x27,4]
	cmp	w2, w1
	bne	.L2053
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x19,4]
	str	w1, [x20,x0]
.L2053:
	add	w21, w21, 1
	sxth	w21, w21
	b	.L2052
.L2067:
	mov	x0, x27
	bl	ftl_free_no_use_map_blk
	adrp	x0, .LANCHOR0+2028
	ldrh	w1, [x27,2]
	ldrh	w0, [x0,#:lo12:.LANCHOR0+2028]
	cmp	w1, w0
	bne	.L2056
	mov	x0, x27
	bl	ftl_map_blk_alloc_new_blk
	b	.L2056
.L2051:
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
	beq	.L2057
	ldrh	w1, [x26]
	ldrh	w0, [x27,4]
	cmp	w1, w0
	bne	.L2057
	ldrh	w1, [x26,8]
	mov	w0, 64245
	cmp	w1, w0
	bne	.L2057
	mov	w0, 0
.L2058:
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w1, [x1,2028]
	sub	w1, w1, #1
	cmp	w0, w1
	bge	.L2061
	ldr	x1, [x29,136]
	sbfiz	x2, x0, 3, 32
	add	x1, x1, :lo12:.LANCHOR2
	ldr	x5, [x1,-64]
	ldrh	w1, [x5,x2]
	cmp	w1, w22
	bcs	.L2059
	add	x2, x5, x2
	ubfiz	x1, x1, 2, 16
	ldr	w2, [x2,4]
	str	w2, [x20,x1]
.L2059:
	add	w0, w0, 1
	sxth	w0, w0
	b	.L2058
.L2057:
	str	xzr, [x19,8]
	mov	w5, 0
.L2062:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w0, [x0,2028]
	cmp	w5, w0
	bge	.L2061
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
	beq	.L2063
	ldrh	w0, [x26,8]
	cmp	w0, w22
	bcs	.L2063
	ldrh	w2, [x26]
	ldrh	w1, [x27,4]
	cmp	w2, w1
	bne	.L2063
	ubfiz	x0, x0, 2, 16
	ldr	w1, [x19,4]
	str	w1, [x20,x0]
.L2063:
	add	w5, w5, 1
	sxth	w5, w5
	b	.L2062
.L2061:
	add	w6, w25, 1
	sxth	w25, w6
	b	.L2050
.L2056:
	ldrh	w1, [x27,8]
	ldrh	w0, [x27,10]
	cmp	w1, w0
	bcc	.L2068
	mov	x0, x27
	bl	ftl_map_blk_gc
.L2068:
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
	.global	FtlReadRefresh
	.type	FtlReadRefresh, %function
FtlReadRefresh:
	stp	x29, x30, [sp, -112]!
	adrp	x1, .LANCHOR2
	add	x0, x1, :lo12:.LANCHOR2
	add	x29, sp, 0
	add	x2, x0, 352
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	adrp	x20, .LANCHOR0
	ldr	w3, [x2,80]
	cbz	w3, .L2082
	add	x1, x20, :lo12:.LANCHOR0
	ldr	w3, [x2,84]
	ldr	w1, [x1,2104]
	cmp	w3, w1
	bcs	.L2083
	mov	w21, 2048
	mov	x19, x0
.L2088:
	add	x1, x20, :lo12:.LANCHOR0
	ldr	w0, [x19,436]
	ldr	w1, [x1,2104]
	cmp	w0, w1
	bcs	.L2085
	add	x1, x29, 52
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x19,436]
	ldr	w1, [x29,52]
	add	w0, w0, 1
	str	w0, [x19,436]
	cmn	w1, #1
	beq	.L2086
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
	bne	.L2085
	ldr	w0, [x29,52]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
	b	.L2085
.L2086:
	subs	w21, w21, #1
	bne	.L2088
.L2085:
	mov	w0, -1
	b	.L2090
.L2083:
	ldr	w0, [x0,164]
	str	wzr, [x2,80]
	str	wzr, [x2,84]
	str	w0, [x2,76]
	b	.L2096
.L2082:
	add	x20, x20, :lo12:.LANCHOR0
	ldr	w3, [x0,216]
	mov	w2, 1048576
	ldr	w0, [x0,160]
	ldrb	w4, [x20,120]
	cbnz	w4, .L2091
	add	w2, w0, w3, lsr 10
	mov	w0, 33554432
	asr	w2, w0, w2
.L2091:
	add	x3, x1, :lo12:.LANCHOR2
	ldr	w0, [x3,428]
	ldr	w3, [x3,164]
	add	w4, w3, 1048576
	cmp	w0, w4
	bhi	.L2092
	add	w2, w2, w0
	mov	w0, 0
	cmp	w2, w3
	bcs	.L2090
.L2092:
	add	x0, x1, :lo12:.LANCHOR2
	mov	w1, 1
	str	wzr, [x0,436]
	str	w1, [x0,432]
	str	w3, [x0,428]
.L2096:
	mov	w0, 0
.L2090:
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
	bhi	.L2098
	adrp	x24, .LANCHOR4
	lsr	w21, w26, w21
	add	x24, x24, :lo12:.LANCHOR4
	mov	w20, 0
	adrp	x27, .LANCHOR2
	add	x24, x24, 200
.L2099:
	cbz	w28, .L2098
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
	beq	.L2101
	cbz	w4, .L2101
	ldr	x0, [x2,2080]
	mov	w1, 1
	str	x0, [x29,128]
	mov	w2, w1
	add	x0, x29, 120
	str	w4, [x29,124]
	str	xzr, [x29,136]
	bl	FlashReadPages
	b	.L2102
.L2101:
	add	x0, x27, :lo12:.LANCHOR2
	add	x2, x23, :lo12:.LANCHOR0
	mov	w1, 0
	ldr	x0, [x0,2080]
	ldrh	w2, [x2,2036]
	bl	ftl_memset
.L2102:
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
	b	.L2099
.L2098:
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
	bne	.L2112
	mov	w1, 54
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
.L2112:
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
	cbz	w4, .L2115
	ldr	w3, [x3,220]
	cmp	w3, 29
	mov	w3, 4
	csel	w1, w1, w3, hi
.L2115:
	adrp	x3, .LANCHOR4
	add	x3, x3, :lo12:.LANCHOR4
	ldrh	w4, [x3,264]
	cmp	w4, 31
	bhi	.L2116
	add	w4, w4, 1
	mov	w1, 1
	strh	w4, [x3,264]
.L2116:
	cbnz	w0, .L2117
	add	x0, x2, :lo12:.LANCHOR2
	ldr	x3, [x0,2152]
	ldr	w0, [x3,20]
	add	w1, w1, w0
	ldr	w0, [x3,16]
	cmp	w0, w1
	bcc	.L2118
.L2117:
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
.L2118:
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
	cbz	w1, .L2123
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
	cbz	w3, .L2125
	mov	x0, x1
	mov	w1, 1
	mov	w2, w1
	bl	FlashReadPages
	b	.L2126
.L2125:
	adrp	x2, .LANCHOR0+2036
	mov	w1, 255
	ldrh	w2, [x2,#:lo12:.LANCHOR0+2036]
	bl	ftl_memset
.L2126:
	add	x2, x19, :lo12:.LANCHOR4
	mov	x0, x21
	mov	w1, w20
	ldr	x2, [x2,144]
	bl	FtlMapWritePage
.L2123:
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
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	stp	x23, x24, [sp,48]
	stp	x27, x28, [sp,80]
	stp	x21, x22, [sp,32]
	stp	x25, x26, [sp,64]
	mov	x19, x0
	adrp	x23, .LANCHOR0
	ldrh	w0, [x0,2]
	str	w0, [x29,188]
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w27, [x19,6]
	ldr	w1, [x29,188]
	ldrh	w0, [x0,2026]
	cmp	w0, w1
	bne	.L2131
	strh	wzr, [x19,4]
	b	.L2282
.L2131:
	ldrh	w0, [x19,16]
	mov	w1, 0
	mov	w2, 65535
.L2133:
	cmp	w0, w2
	bne	.L2285
	add	w1, w1, 1
	uxth	w1, w1
	add	x0, x19, x1, sxtw 1
	ldrh	w0, [x0,16]
	b	.L2133
.L2285:
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2135
	bl	FtlGetLastWrittenPage
	cmn	w0, #1
	mov	w21, w0
	beq	.L2136
	add	x0, x23, :lo12:.LANCHOR0
	mov	w24, w21
	ldrb	w1, [x0,120]
	cbnz	w1, .L2205
	add	x0, x0, 196
	ldrh	w24, [x0,w21,sxtw 1]
	b	.L2205
.L2135:
	mov	w1, 0
	bl	FtlGetLastWrittenPage
	cmn	w0, #1
	mov	w21, w0
	beq	.L2136
	mov	w24, w0
.L2205:
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
	b	.L2138
.L2136:
	strh	wzr, [x19,2]
.L2282:
	strb	wzr, [x19,6]
	b	.L2276
.L2140:
	ldrh	w1, [x2,16]
	cmp	w1, w7
	beq	.L2139
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
.L2139:
	add	x2, x2, 2
.L2138:
	cmp	x2, x3
	bne	.L2140
	ldrb	w0, [x19,8]
	str	wzr, [x29,184]
	cmp	w0, 1
	bne	.L2141
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cmp	w0, wzr
	cset	w0, ne
	str	w0, [x29,184]
.L2141:
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
.L2142:
	cmp	w26, w28
	beq	.L2286
	ldr	w0, [x3]
	cbnz	w0, .L2143
	ldr	x4, [x3,16]
	ldr	w2, [x4,4]
	cmn	w2, #1
	beq	.L2144
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
	cbz	w0, .L2144
	add	w2, w2, 1
	str	w2, [x5,200]
.L2144:
	ldr	w0, [x4]
	cmn	w0, #1
	bne	.L2145
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, 56
	uxth	w21, w21
	umull	x26, w26, w1
	ldr	x0, [x0,2048]
	add	x26, x0, x26
	ldr	w0, [x26,4]
	b	.L2278
.L2143:
	mov	w25, w6
.L2145:
	add	w26, w26, 1
	add	x3, x3, 56
	uxth	w26, w26
	b	.L2142
.L2286:
	add	x0, x20, :lo12:.LANCHOR2
	add	w21, w21, 1
	uxth	w21, w21
	ldr	x0, [x0,2048]
	ldr	w0, [x0,4]
.L2278:
	lsr	x0, x0, 10
	bl	P2V_plane
	uxth	w26, w0
	ldrb	w1, [x19,8]
	cmp	w1, 1
	bne	.L2149
	add	x0, x23, :lo12:.LANCHOR0
	ldrb	w2, [x0,120]
	cbnz	w2, .L2149
	add	x0, x0, 196
	ldrh	w21, [x0,w21,sxtw 1]
.L2149:
	add	x0, x23, :lo12:.LANCHOR0
	ldrh	w0, [x0,2026]
	cmp	w0, w21
	bne	.L2150
	strh	w21, [x19,2]
	strb	wzr, [x19,6]
	strh	wzr, [x19,4]
.L2150:
	uxth	w0, w27
	str	w0, [x29,152]
	ldr	w0, [x29,188]
	cmp	w21, w0
	bne	.L2151
	ldr	w0, [x29,152]
	cmp	w26, w0
	beq	.L2283
.L2151:
	mov	w0, 65535
	sub	w22, w22, #1
	cmp	w25, w0
	bne	.L2152
	cbnz	w1, .L2153
.L2152:
	add	x0, x20, :lo12:.LANCHOR2
	uxth	w5, w24
	ldr	w1, [x0,2332]
	cmn	w1, #1
	bne	.L2154
	str	w22, [x0,2332]
.L2154:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w25, [x29,188]
	ldr	w6, [x0,2332]
	ldr	w0, [x29,188]
	add	w0, w0, 7
	cmp	w0, w24, uxth
	bge	.L2155
	sub	w25, w5, #7
	uxth	w25, w25
.L2155:
	mov	w3, -1
	add	x28, x20, :lo12:.LANCHOR2
	mov	w27, w3
	mov	w4, 65535
	mov	w7, 56
	add	x8, x28, 996
.L2156:
	cmp	w25, w5
	bhi	.L2169
	add	x0, x23, :lo12:.LANCHOR0
	mov	w24, 0
	ldrh	w2, [x0,1952]
	mov	x0, x19
	add	x2, x19, x2, lsl 1
.L2170:
	cmp	x0, x2
	beq	.L2287
	ldrh	w1, [x0,16]
	cmp	w1, w4
	beq	.L2157
	umull	x9, w24, w7
	ldr	x10, [x28,2048]
	add	w24, w24, 1
	orr	w1, w25, w1, lsl 10
	add	x9, x10, x9
	uxth	w24, w24
	str	w1, [x9,4]
.L2157:
	add	x0, x0, 2
	b	.L2170
.L2287:
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
.L2159:
	cmp	w1, w24
	beq	.L2288
	ldr	w2, [x0,-16]
	cbnz	w2, .L2160
	ldr	x2, [x0]
	ldrh	w11, [x2]
	cmp	w11, w4
	beq	.L2161
	ldr	w2, [x2,4]
	cmn	w2, #1
	beq	.L2161
	ldr	w27, [x28,2332]
	cmn	w3, #1
	str	w2, [x28,2332]
	bne	.L2161
	ldrh	w2, [x8,x10,lsl 1]
	cmp	w2, w4
	bne	.L2162
	cbz	w9, .L2161
.L2162:
	cmp	w27, w22
	csinv	w3, w27, wzr, ne
	b	.L2161
.L2160:
	ldrb	w0, [x19,8]
	cbnz	w0, .L2153
	add	x0, x20, :lo12:.LANCHOR2
	add	x1, x0, 996
	ldrh	w2, [x1,w25,sxtw 1]
	mov	w1, 65535
	cmp	w2, w1
	bne	.L2164
	cmn	w3, #1
	beq	.L2165
	str	w3, [x0,2332]
	b	.L2153
.L2165:
	cmp	w6, w22
	beq	.L2166
	str	w6, [x0,2332]
	b	.L2153
.L2166:
	ldr	w1, [x0,2332]
	b	.L2284
.L2164:
	cmp	w27, w22
	beq	.L2167
	cmn	w27, #1
	beq	.L2153
	str	w27, [x0,2332]
	b	.L2153
.L2167:
	ldr	w1, [x0,2332]
	cmp	w1, w22
	beq	.L2153
.L2284:
	sub	w1, w1, #1
	b	.L2279
.L2161:
	add	w1, w1, 1
	add	x0, x0, 56
	uxth	w1, w1
	b	.L2159
.L2288:
	add	w25, w25, 1
	uxth	w25, w25
	b	.L2156
.L2169:
	add	x0, x20, :lo12:.LANCHOR2
	mov	w1, -1
.L2279:
	str	w1, [x0,2332]
.L2153:
	adrp	x25, .LANCHOR4
	mov	w1, 1
	add	x0, x25, :lo12:.LANCHOR4
	strh	w1, [x0,266]
	add	x0, x20, :lo12:.LANCHOR2
	add	x0, x0, 2240
	bl	FtlMapBlkWriteDumpData
	ldr	w24, [x29,188]
	adrp	x0, .LC113
	str	x25, [x29,136]
	add	x0, x0, :lo12:.LC113
	str	x0, [x29,128]
.L2171:
	add	x0, x23, :lo12:.LANCHOR0
	mov	x1, x19
	str	wzr, [x29,176]
	mov	w7, 65535
	mov	w8, 56
	ldrh	w3, [x0,1952]
	ldrb	w6, [x0,120]
	add	x3, x19, x3, lsl 1
.L2172:
	cmp	x1, x3
	beq	.L2289
	ldrh	w2, [x1,16]
	cmp	w2, w7
	beq	.L2173
	ldr	w0, [x29,176]
	add	x5, x20, :lo12:.LANCHOR2
	orr	w2, w24, w2, lsl 10
	ldr	x4, [x5,2048]
	umull	x0, w0, w8
	add	x4, x4, x0
	str	w2, [x4,4]
	ldrb	w2, [x19,8]
	cmp	w2, 1
	bne	.L2174
	cbz	w6, .L2174
	ldr	x2, [x5,2048]
	add	x0, x2, x0
	ldr	w2, [x0,4]
	orr	w2, w2, -2147483648
	str	w2, [x0,4]
.L2174:
	ldr	w0, [x29,176]
	add	w0, w0, 1
	uxth	w0, w0
	str	w0, [x29,176]
.L2173:
	add	x1, x1, 2
	b	.L2172
.L2289:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w1, [x29,176]
	ldr	w2, [x29,184]
	ldr	x0, [x0,2048]
	bl	FlashReadPages
	str	xzr, [x29,160]
.L2176:
	ldrh	w0, [x29,160]
	ldr	w1, [x29,176]
	cmp	w1, w0, uxth
	bls	.L2290
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
	bcc	.L2177
	ldr	w2, [x29,152]
	cmp	w0, w2
	bcs	.L2209
	ldr	w2, [x29,188]
	cmp	w24, w2
	beq	.L2177
.L2209:
	cmp	w0, w26
	bne	.L2210
	cmp	w24, w21
	beq	.L2179
.L2210:
	ldr	w0, [x1,x25]
	cmn	w0, #1
	beq	.L2181
	ldr	x28, [x28,16]
	mov	w0, 61589
	ldrh	w1, [x28]
	cmp	w1, w0
	bne	.L2188
	ldr	w22, [x28,4]
	cmn	w22, #1
	beq	.L2183
	add	x27, x20, :lo12:.LANCHOR2
	mov	w0, w22
	ldr	w1, [x27,200]
	bl	ftl_cmp_data_ver
	cbz	w0, .L2183
	add	w0, w22, 1
	str	w0, [x27,200]
.L2183:
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
	beq	.L2184
	mov	w0, w22
	bl	ftl_cmp_data_ver
	ldr	x3, [x29,168]
	cbz	w0, .L2184
	ldr	w1, [x29,196]
	cmn	w1, #1
	beq	.L2185
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
	bne	.L2186
	b	.L2187
.L2185:
	ldr	w0, [x29,204]
	ldr	w1, [x29,200]
	cmp	w1, w0
	bne	.L2188
	mov	w0, w27
	add	x1, x29, 196
	mov	w2, 1
	bl	log2phys
.L2188:
	ldrh	w0, [x19]
	b	.L2281
.L2186:
	ldr	x0, [x29,168]
	ldr	w28, [x0,8]
	cmp	w28, w27
	bne	.L2187
	ldr	w0, [x3,2332]
	ldr	w1, [x29,144]
	str	x2, [x29,104]
	str	x4, [x29,112]
	str	x3, [x29,120]
	bl	ftl_cmp_data_ver
	ldr	x3, [x29,120]
	ldr	x4, [x29,112]
	ldr	x2, [x29,104]
	cbz	w0, .L2187
	ldr	w0, [x29,200]
	ldr	w1, [x29,204]
	cmp	w0, w1
	beq	.L2193
	ldr	w1, [x29,196]
	cmp	w0, w1
	beq	.L2187
	cmn	w0, #1
	beq	.L2191
	str	w0, [x4,4]
	mov	w2, 0
	ldr	x1, [x4,16]
	ldr	x0, [x3,2048]
	str	x1, [x29,168]
	mov	w1, 1
	add	x0, x0, x25
	bl	FlashReadPages
	b	.L2192
.L2191:
	str	w0, [x2,x25]
.L2192:
	add	x0, x20, :lo12:.LANCHOR2
	ldr	x1, [x0,2048]
	ldr	w1, [x1,x25]
	cmn	w1, #1
	beq	.L2193
	ldr	x1, [x29,168]
	ldr	w0, [x0,2332]
	ldr	w25, [x1,4]
	mov	w1, w25
	bl	ftl_cmp_data_ver
	cbz	w0, .L2193
	ldr	w0, [x29,144]
	mov	w1, w25
	bl	ftl_cmp_data_ver
	cbz	w0, .L2187
.L2193:
	ldr	w1, [x29,196]
	mov	w0, w28
	bl	FtlReUsePrevPpa
.L2187:
	mov	w0, -1
	str	w0, [x29,196]
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	b	.L2195
.L2184:
	ldr	w0, [x29,204]
	ldr	w1, [x29,200]
	cmp	w1, w0
	beq	.L2195
	mov	w0, w27
	add	x1, x29, 204
	mov	w2, 1
	bl	log2phys
	ldr	w25, [x29,200]
	cmn	w25, #1
	beq	.L2195
	ldr	w0, [x29,196]
	cmp	w25, w0
	beq	.L2195
	lsr	x0, x25, 10
	bl	P2V_block_in_plane
	uxth	w0, w0
	ldrh	w2, [x20,#:lo12:.LANCHOR2]
	add	x1, x20, :lo12:.LANCHOR2
	cmp	w2, w0
	beq	.L2198
	ldrh	w2, [x1,48]
	cmp	w2, w0
	beq	.L2198
	ldrh	w1, [x1,96]
	cmp	w1, w0
	bne	.L2195
.L2198:
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
	beq	.L2195
	mov	w0, w22
	bl	ftl_cmp_data_ver
	cbnz	w0, .L2195
	mov	w0, w27
	add	x1, x29, 200
	mov	w2, 1
	bl	log2phys
.L2195:
	ldr	w0, [x29,196]
	cmn	w0, #1
	beq	.L2177
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	add	x2, x20, :lo12:.LANCHOR2
	ubfiz	x3, x1, 1, 16
	ldr	x2, [x2,-40]
	ldrh	w2, [x2,x3]
	cbz	w2, .L2199
.L2281:
	bl	decrement_vpc_count
	b	.L2177
.L2199:
	ldr	x0, [x29,128]
	bl	printk
	b	.L2177
.L2181:
	ldr	x0, [x29,136]
	add	x1, x0, :lo12:.LANCHOR4
	ldr	w0, [x1,268]
	cmp	w0, 31
	bhi	.L2200
	add	x2, x1, 272
	str	w27, [x2,w0,uxtw 2]
	add	w0, w0, 1
	str	w0, [x1,268]
.L2200:
	ldrh	w0, [x19]
	bl	decrement_vpc_count
	add	x0, x20, :lo12:.LANCHOR2
	ldr	w1, [x0,2332]
	cmn	w1, #1
	beq	.L2280
	cmp	w1, w22
	bls	.L2177
.L2280:
	str	w22, [x0,2332]
.L2177:
	ldr	x0, [x29,160]
	add	x0, x0, 1
	str	x0, [x29,160]
	b	.L2176
.L2290:
	add	x0, x23, :lo12:.LANCHOR0
	add	w24, w24, 1
	uxth	w24, w24
	ldrh	w1, [x0,2026]
	cmp	w24, w1
	bne	.L2171
	ldrh	w2, [x0,1952]
	mov	w3, 65535
	strh	w24, [x19,2]
	mov	x0, 0
	strh	wzr, [x19,4]
.L2203:
	uxth	w1, w0
	cmp	w1, w2
	bcs	.L2276
	add	x0, x0, 1
	add	x4, x19, x0, lsl 1
	ldrh	w4, [x4,14]
	cmp	w4, w3
	beq	.L2203
	strb	w1, [x19,6]
	b	.L2276
.L2179:
	strb	w26, [x19,6]
	strh	w21, [x19,2]
.L2283:
	mov	x0, x19
	mov	w1, w21
	mov	w2, w26
	bl	ftl_sb_update_avl_pages
.L2276:
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
	cbz	w1, .L2292
	ldrb	w0, [x21,8]
	cbnz	w0, .L2292
	adrp	x22, .LANCHOR0
	ldrb	w0, [x21,7]
	add	x22, x22, :lo12:.LANCHOR0
	ldrh	w2, [x22,2026]
	mul	w0, w0, w2
	cmp	w1, w0
	beq	.L2292
	ldrb	w24, [x21,10]
	ldr	w23, [x22,2104]
	ldrh	w25, [x22,1952]
	cbnz	w24, .L2291
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
	beq	.L2294
	add	x0, x29, 88
	mov	w1, 1
	mov	w2, w24
	bl	FlashReadPages
	b	.L2295
.L2294:
	ldr	x0, [x21,-64]
	mov	w1, 255
	ldrh	w2, [x22,2036]
	bl	ftl_memset
.L2295:
	mov	w0, -3947
	lsl	w25, w25, 2
	strh	w0, [x20]
	mov	w22, 0
	add	x21, x19, :lo12:.LANCHOR2
.L2296:
	cmp	w22, w25
	beq	.L2297
	ldrh	w0, [x21,4]
	cbz	w0, .L2297
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
	b	.L2296
.L2297:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, 1
	strb	w0, [x19,10]
	b	.L2291
.L2292:
	add	x19, x19, :lo12:.LANCHOR2
	strb	wzr, [x19,10]
.L2291:
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
.L2309:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w0, [x0,2066]
	cmp	w0, w19
	bls	.L2312
	add	x1, x21, :lo12:.LANCHOR2
	ubfiz	x0, x19, 4, 16
	ldr	x1, [x1,144]
	add	x0, x1, x0
	ldr	w0, [x0,4]
	tbz	w0, #31, .L2310
	mov	w0, w19
	bl	flush_l2p_region
.L2310:
	add	w19, w19, 1
	uxth	w19, w19
	b	.L2309
.L2312:
	mov	w0, 0
	ldr	x21, [sp,32]
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	l2p_flush, .-l2p_flush
	.align	2
	.global	FtlVpcCheckAndModify
	.type	FtlVpcCheckAndModify, %function
FtlVpcCheckAndModify:
	stp	x29, x30, [sp, -80]!
	adrp	x1, .LANCHOR3
	adrp	x0, .LC114
	add	x1, x1, :lo12:.LANCHOR3
	add	x29, sp, 0
	add	x1, x1, 16
	add	x0, x0, :lo12:.LC114
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
.L2314:
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w0, [x0,2104]
	cmp	w19, w0
	bcs	.L2329
	mov	w0, w19
	add	x1, x29, 76
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x29,76]
	cmn	w0, #1
	beq	.L2315
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	ubfiz	x0, x0, 1, 16
	add	x1, x20, :lo12:.LANCHOR2
	ldr	x2, [x1,2160]
	ldrh	w1, [x2,x0]
	add	w1, w1, 1
	strh	w1, [x2,x0]
.L2315:
	add	w19, w19, 1
	b	.L2314
.L2329:
	adrp	x23, .LC115
	mov	w19, 0
	add	x20, x20, :lo12:.LANCHOR2
	mov	w24, 65535
	add	x23, x23, :lo12:.LC115
.L2317:
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,1960]
	cmp	w0, w19
	bls	.L2330
	ubfiz	x22, x19, 1, 16
	ldr	x0, [x20,-40]
	ldrh	w2, [x0,x22]
	ldr	x0, [x20,2160]
	ldrh	w3, [x0,x22]
	cmp	w2, w3
	beq	.L2320
	cmp	w2, w24
	beq	.L2320
	mov	x0, x23
	mov	w1, w19
	bl	printk
	ldrh	w0, [x20]
	cmp	w0, w19
	beq	.L2320
	ldrh	w0, [x20,96]
	cmp	w0, w19
	beq	.L2320
	ldrh	w0, [x20,48]
	cmp	w0, w19
	beq	.L2320
	ldr	x0, [x20,2160]
	ldrh	w1, [x0,x22]
	ldr	x0, [x20,-40]
	strh	w1, [x0,x22]
	mov	w0, w19
	bl	update_vpc_list
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2320:
	add	w19, w19, 1
	uxth	w19, w19
	b	.L2317
.L2330:
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
	beq	.L2332
	add	x1, x19, :lo12:.LANCHOR2
	ubfiz	x0, x21, 1, 16
	ldr	x1, [x1,-40]
	ldrh	w0, [x1,x0]
	cbz	w0, .L2333
	mov	w0, w21
	bl	INSERT_DATA_LIST
	b	.L2332
.L2333:
	mov	w0, w21
	bl	INSERT_FREE_LIST
.L2332:
	add	x2, x19, :lo12:.LANCHOR2
	strb	wzr, [x20,8]
	add	x0, x2, 48
	cmp	x20, x0
	beq	.L2334
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w1, [x0,1972]
	cmp	w1, 1
	beq	.L2334
	ldrb	w0, [x0,120]
	cbz	w0, .L2335
.L2334:
	mov	w0, 1
	strb	w0, [x20,8]
	b	.L2336
.L2335:
	cmp	x20, x2
	bne	.L2336
	cmp	w1, 3
	beq	.L2338
	ldr	w0, [x20,344]
	cmp	w0, 1
	bne	.L2339
.L2338:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 1
	strb	w1, [x0,8]
.L2339:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w1, [x0,160]
	cbz	w1, .L2336
	ldr	w1, [x0,220]
	cmp	w1, 29
	bhi	.L2336
	mov	w1, 1
	strb	w1, [x0,8]
.L2336:
	add	x2, x19, :lo12:.LANCHOR2
	mov	w1, 65535
	ldrh	w0, [x2,2320]
	cmp	w0, w1
	beq	.L2341
	cmp	w21, w0
	bne	.L2342
	ubfiz	x1, x0, 1, 16
	ldr	x2, [x2,-40]
	ldrh	w1, [x2,x1]
	cbz	w1, .L2343
.L2342:
	bl	update_vpc_list
.L2343:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	w0, [x19,2320]
.L2341:
	mov	x0, x20
	bl	allocate_data_superblock
	bl	l2p_flush
	bl	FtlVpcTblFlush
	mov	w0, 0
	bl	FtlEctTblFlush
	ldr	x21, [sp,32]
	mov	w0, 0
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	allocate_new_data_superblock, .-allocate_new_data_superblock
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
	cbnz	w4, .L2493
	ldrh	w4, [x3,-24]
	cmp	w4, 47
	bls	.L2493
	mov	w24, w0
	adrp	x0, .LANCHOR1+3004
	mov	w22, w1
	ldrh	w1, [x0,#:lo12:.LANCHOR1+3004]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L2358
.L2361:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w1, [x0,954]
	cmp	w1, w2
	bne	.L2359
	b	.L2360
.L2358:
	ldrh	w1, [x3,96]
	cmp	w1, w0
	beq	.L2361
	mov	w0, 1
	bl	FtlGcFreeTempBlock
	mov	w2, 1
	cbz	w0, .L2361
	b	.L2493
.L2359:
	ldrh	w3, [x0,952]
	cmp	w3, w2
	bne	.L2360
	strh	w1, [x0,952]
	mov	w1, -1
	strh	w1, [x0,954]
.L2360:
	add	x20, x19, :lo12:.LANCHOR2
	cmp	w24, 1
	ldr	w0, [x20,868]
	add	w0, w0, 1
	add	w0, w0, w24, lsl 7
	str	w0, [x20,868]
	beq	.L2362
.L2364:
	mov	w21, 65535
	b	.L2363
.L2362:
	ldr	w1, [x20,160]
	cbz	w1, .L2364
	ldr	w1, [x20,220]
	cmp	w1, 29
	bhi	.L2364
	adrp	x23, .LANCHOR4
	add	x21, x23, :lo12:.LANCHOR4
	ldrh	w1, [x21,400]
	add	w0, w0, w1
	str	w0, [x20,868]
	bl	FtlGcReFreshBadBlk
	ldrh	w0, [x20,296]
	mov	w1, 65535
	cmp	w0, w1
	bne	.L2364
	ldrh	w1, [x20,952]
	cmp	w1, w0
	bne	.L2364
	ldr	w0, [x20,868]
	cmp	w0, 1024
	bls	.L2364
	ldr	w0, [x20,220]
	str	wzr, [x20,868]
	strh	wzr, [x21,400]
	cbnz	w0, .L2365
	mov	w0, 6
	b	.L2515
.L2365:
	cmp	w0, 5
	bhi	.L2366
	mov	w0, 18
.L2515:
	strh	w0, [x21,400]
.L2366:
	mov	w0, 32
	mov	w27, 65535
	bl	List_get_gc_head_node
	uxth	w25, w0
	cmp	w25, w27
	beq	.L2370
	add	x20, x19, :lo12:.LANCHOR2
	ldrh	w0, [x20,872]
	cbz	w0, .L2368
	adrp	x2, .LANCHOR0
	ldr	x26, [x20,-40]
	add	x2, x2, :lo12:.LANCHOR0
	ubfiz	x25, x25, 1, 16
	ldrh	w1, [x2,2028]
	ldrh	w2, [x2,1952]
	ldrh	w3, [x26,x25]
	mul	w1, w1, w2
	add	w1, w1, 1
	cmp	w3, w1
	bgt	.L2370
	add	w1, w0, 1
	str	wzr, [x20,876]
	uxth	w1, w1
	strh	w1, [x20,872]
	str	x1, [x29,120]
	bl	List_get_gc_head_node
	uxth	w21, w0
	cmp	w21, w27
	ldr	x1, [x29,120]
	beq	.L2370
	ubfiz	x27, x21, 1, 16
	adrp	x0, .LC116
	ldrh	w4, [x26,x25]
	add	x0, x0, :lo12:.LC116
	mov	w2, w21
	ldrh	w3, [x26,x27]
	bl	printk
	ldrh	w0, [x20,872]
	cmp	w0, 40
	bls	.L2369
	ldr	x0, [x20,-40]
	ldrh	w0, [x0,x27]
	cmp	w0, 32
	bls	.L2369
	strh	wzr, [x20,872]
.L2369:
	add	x23, x23, :lo12:.LANCHOR4
	mov	w0, 6
	strh	w0, [x23,400]
	b	.L2363
.L2368:
	mov	w0, 1
	strh	w0, [x20,872]
.L2370:
	bl	GetSwlReplaceBlock
	uxth	w21, w0
	mov	w0, 65535
	cmp	w21, w0
	bne	.L2363
	add	x23, x23, :lo12:.LANCHOR4
	strh	wzr, [x23,400]
.L2363:
	add	x20, x19, :lo12:.LANCHOR2
	mov	w0, 65535
	ldrh	w1, [x20,296]
	cmp	w1, w0
	bne	.L2371
	ldrh	w0, [x20,96]
	cmp	w0, w1
	bne	.L2371
	ldrh	w25, [x20,952]
	cmp	w25, w0
	bne	.L2371
	ldrh	w0, [x20,-8]
	mov	w2, 1024
	ldr	w1, [x20,868]
	cmp	w0, 24
	mov	w0, 5120
	csel	w0, w0, w2, cc
	cmp	w1, w0
	bls	.L2371
	adrp	x0, .LANCHOR4+400
	str	wzr, [x20,868]
	mov	x23, x20
	strh	wzr, [x0,#:lo12:.LANCHOR4+400]
	bl	GetSwlReplaceBlock
	uxth	w21, w0
	cmp	w21, w25
	bne	.L2373
	ldrh	w1, [x20,-8]
	ldrh	w0, [x20,866]
	cmp	w1, w0
	bcs	.L2374
	mov	w0, 64
	bl	List_get_gc_head_node
	uxth	w0, w0
	cmp	w0, w21
	beq	.L2383
	ldr	w2, [x20,2024]
	uxtw	x1, w0
	adrp	x20, .LANCHOR0
	cbnz	w2, .L2376
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w2, [x0,1972]
	cmp	w2, 3
	beq	.L2376
	ldr	w2, [x23,344]
	cbnz	w2, .L2376
	ldr	w2, [x23,160]
	cbnz	w2, .L2376
	ldrb	w0, [x0,120]
	cbz	w0, .L2377
.L2376:
	add	x0, x19, :lo12:.LANCHOR2
	add	x3, x20, :lo12:.LANCHOR0
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
	bgt	.L2379
	mov	w0, 0
	add	x20, x20, :lo12:.LANCHOR0
	bl	List_get_gc_head_node
	uxth	w21, w0
	add	x1, x19, :lo12:.LANCHOR2
	ldr	w0, [x20,2104]
	ldr	w2, [x1,156]
	add	w0, w0, w0, lsl 1
	cmp	w2, w0, lsr 2
	bls	.L2380
	mov	w0, 128
	b	.L2516
.L2380:
	mov	w0, 160
.L2516:
	strh	w0, [x1,866]
	b	.L2381
.L2379:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 128
	b	.L2517
.L2377:
	ldr	x2, [x23,-40]
	ldrh	w1, [x2,x1,lsl 1]
	cmp	w1, 7
	bhi	.L2382
	bl	List_get_gc_head_node
	uxth	w21, w0
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 128
	strh	w1, [x0,866]
	b	.L2381
.L2382:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w1, 64
.L2517:
	strh	w1, [x0,866]
	b	.L2383
.L2374:
	mov	w0, 80
	strh	w0, [x20,866]
	b	.L2383
.L2381:
	mov	w0, 65535
	cmp	w21, w0
	beq	.L2383
.L2373:
	add	x5, x19, :lo12:.LANCHOR2
	ubfiz	x4, x21, 1, 16
	adrp	x0, .LC117
	mov	w1, w21
	add	x0, x0, :lo12:.LC117
	ldr	x3, [x5,-40]
	ldr	x6, [x5,-80]
	ldrh	w2, [x5,-8]
	ldrh	w3, [x3,x4]
	ldrh	w5, [x5,864]
	ldrh	w4, [x6,x4]
	bl	printk
.L2383:
	bl	FtlGcReFreshBadBlk
.L2371:
	mov	w0, 65535
	cmp	w21, w0
	cset	w1, eq
	cbz	w1, .L2384
	cbnz	w24, .L2384
	add	x0, x19, :lo12:.LANCHOR2
	mov	w20, 1
	ldrh	w1, [x0,-8]
	cmp	w1, 24
	bhi	.L2385
	adrp	x0, .LANCHOR0
	cmp	w1, 16
	add	x0, x0, :lo12:.LANCHOR0
	ldrh	w20, [x0,2026]
	bls	.L2386
	lsr	w20, w20, 5
	b	.L2385
.L2386:
	cmp	w1, 12
	bls	.L2387
	lsr	w20, w20, 4
	b	.L2385
.L2387:
	cmp	w1, 8
	bls	.L2385
	lsr	w20, w20, 2
.L2385:
	add	x0, x19, :lo12:.LANCHOR2
	ldrh	w2, [x0,864]
	cmp	w2, w1
	bcs	.L2389
	ldrh	w1, [x0,96]
	mov	w2, 65535
	cmp	w1, w2
	bne	.L2390
	ldrh	w2, [x0,952]
	cmp	w2, w1
	bne	.L2390
	adrp	x1, .LANCHOR4+400
	ldrh	w2, [x1,#:lo12:.LANCHOR4+400]
	cbnz	w2, .L2391
	adrp	x1, .LANCHOR0+2104
	ldr	w3, [x0,156]
	ldr	w1, [x1,#:lo12:.LANCHOR0+2104]
	add	w1, w1, w1, lsl 1
	cmp	w3, w1, lsr 2
	bcs	.L2392
.L2391:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,272]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1,864]
	b	.L2393
.L2392:
	mov	w1, 18
	strh	w1, [x0,864]
.L2393:
	add	x19, x19, :lo12:.LANCHOR2
	str	wzr, [x19,876]
	b	.L2493
.L2390:
	add	x1, x19, :lo12:.LANCHOR2
	ldrh	w0, [x1,272]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x1,864]
.L2389:
	cmp	w22, 2
	bhi	.L2448
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w0, [x0,160]
	cbz	w0, .L2448
	add	w20, w20, 1
	uxth	w20, w20
	b	.L2448
.L2384:
	add	x20, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w0, [x20,96]
	cmp	w0, w2
	bne	.L2396
	cbz	w1, .L2396
	ldrh	w1, [x20,952]
	cmp	w1, w0
	bne	.L2396
	ldrh	w1, [x20,296]
	cmp	w1, w0
	beq	.L2397
.L2402:
	mov	w21, 65535
	b	.L2396
.L2397:
	ldrh	w23, [x20,-8]
	adrp	x21, .LANCHOR4
	ldrh	w0, [x20,864]
	str	wzr, [x20,876]
	cmp	w0, w23
	bcs	.L2398
	add	x0, x21, :lo12:.LANCHOR4
	ldrh	w0, [x0,400]
	cbnz	w0, .L2399
	adrp	x0, .LANCHOR0+2104
	ldr	w1, [x20,156]
	ldr	w0, [x0,#:lo12:.LANCHOR0+2104]
	add	w0, w0, w0, lsl 1
	cmp	w1, w0, lsr 2
	bcs	.L2400
.L2399:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w0, [x19,272]
	add	w0, w0, w0, lsl 1
	asr	w0, w0, 2
	strh	w0, [x19,864]
	b	.L2401
.L2400:
	mov	w0, 18
	strh	w0, [x20,864]
.L2401:
	add	x21, x21, :lo12:.LANCHOR4
	bl	FtlReadRefresh
	ldrh	w2, [x21,400]
	b	.L2493
.L2398:
	add	x22, x21, :lo12:.LANCHOR4
	ldrh	w0, [x22,400]
	cbnz	w0, .L2402
	ldrh	w21, [x20,272]
	add	w1, w21, w21, lsl 1
	asr	w1, w1, 2
	strh	w1, [x20,864]
	bl	List_get_gc_head_node
	ubfiz	x0, x0, 1, 16
	ldr	x1, [x20,-40]
	ldrh	w2, [x1,x0]
	adrp	x1, .LANCHOR0
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w0, [x1,2028]
	ldrh	w1, [x1,1952]
	mul	w0, w0, w1
	mov	w1, 2
	sdiv	w0, w0, w1
	cmp	w2, w0
	ble	.L2403
	sub	w21, w21, #1
	cmp	w23, w21
	blt	.L2403
	bl	FtlReadRefresh
	ldrh	w2, [x22,400]
	b	.L2493
.L2403:
	cbnz	w2, .L2402
	mov	w0, -1
	add	x19, x19, :lo12:.LANCHOR2
	bl	decrement_vpc_count
	ldrh	w2, [x19,-8]
	add	w2, w2, 1
	b	.L2493
.L2396:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w20, 2
	ldr	w0, [x0,160]
	cmp	w0, wzr
	csinc	w20, w20, wzr, ne
	b	.L2395
.L2448:
	mov	w21, 65535
.L2395:
	add	x0, x19, :lo12:.LANCHOR2
	mov	w2, 65535
	ldrh	w1, [x0,296]
	cmp	w1, w2
	bne	.L2405
	cmp	w21, w1
	beq	.L2406
	strh	w21, [x0,296]
	b	.L2407
.L2406:
	ldrh	w1, [x0,952]
	cmp	w1, w21
	beq	.L2407
	strh	w1, [x0,296]
	mov	w1, -1
	strh	w1, [x0,952]
.L2407:
	adrp	x1, .LANCHOR0+120
	add	x0, x19, :lo12:.LANCHOR2
	add	x22, x0, 296
	ldrb	w1, [x1,#:lo12:.LANCHOR0+120]
	strb	wzr, [x0,304]
	cbz	w1, .L2408
	ldrh	w0, [x0,296]
	bl	ftl_get_blk_mode
	strb	w0, [x22,8]
.L2408:
	add	x22, x19, :lo12:.LANCHOR2
	ldrh	w0, [x22,296]
	bl	IsBlkInGcList
	cbz	w0, .L2409
	mov	w0, -1
	strh	w0, [x22,296]
.L2409:
	add	x22, x19, :lo12:.LANCHOR2
	mov	w0, 65535
	add	x23, x22, 296
	ldrh	w1, [x22,296]
	cmp	w1, w0
	beq	.L2405
	mov	x0, x23
	bl	make_superblock
	strh	wzr, [x22,298]
	adrp	x0, .LANCHOR4
	ldrh	w2, [x22,296]
	add	x0, x0, :lo12:.LANCHOR4
	ldr	x1, [x22,-40]
	strb	wzr, [x22,302]
	strh	wzr, [x0,402]
	ldrh	w1, [x1,x2,lsl 1]
	strh	w1, [x0,404]
.L2405:
	add	x3, x19, :lo12:.LANCHOR2
	ldrh	w1, [x19,#:lo12:.LANCHOR2]
	ldrh	w0, [x3,296]
	cmp	w1, w0
	beq	.L2411
	ldrh	w1, [x3,48]
	cmp	w1, w0
	beq	.L2411
.L2412:
	mov	x28, x3
	mov	w24, 65535
	b	.L2413
.L2411:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	w0, [x19,296]
	b	.L2518
.L2433:
	ldrh	w0, [x28,298]
	add	w20, w20, w0
	uxth	w20, w20
	strh	w20, [x28,298]
	cmp	w20, w25
	adrp	x20, .LANCHOR4
	bcs	.L2435
	add	x0, x20, :lo12:.LANCHOR4
	ldrh	w1, [x0,402]
	ldrh	w0, [x0,404]
	cmp	w1, w0
	beq	.L2435
.L2436:
	ldrh	w0, [x28,-8]
	cmp	w0, 2
	bhi	.L2439
	add	x23, x23, :lo12:.LANCHOR0
	ldrh	w20, [x23,2026]
.L2413:
	ldrh	w0, [x28,296]
	cmp	w0, w24
	bne	.L2414
	str	wzr, [x28,876]
.L2415:
	ldrh	w22, [x28,872]
	mov	w0, w22
	bl	List_get_gc_head_node
	uxth	w23, w0
	cmp	w23, w24
	strh	w23, [x28,296]
	bne	.L2416
	strh	wzr, [x28,872]
	mov	w2, 8
	b	.L2493
.L2416:
	mov	w0, w23
	add	w22, w22, 1
	bl	IsBlkInGcList
	cbz	w0, .L2417
	strh	w22, [x28,872]
	b	.L2415
.L2417:
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
	bgt	.L2419
	cmp	w4, 8
	bls	.L2420
	cmp	w22, 48
	bls	.L2420
	ldrh	w4, [x28,924]
	cmp	w4, 35
	bhi	.L2420
.L2419:
	strh	wzr, [x28,872]
.L2420:
	ldrh	w1, [x2,x1]
	cmp	w1, w0
	blt	.L2421
	cmp	w21, w24
	bne	.L2421
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	wzr, [x19,872]
	strh	w0, [x19,296]
.L2518:
	adrp	x0, .LANCHOR4+400
	ldrh	w2, [x0,#:lo12:.LANCHOR4+400]
	b	.L2493
.L2421:
	cbnz	w1, .L2422
	mov	w0, -1
	bl	decrement_vpc_count
	ldrh	w0, [x28,872]
	add	w0, w0, 1
	strh	w0, [x28,872]
	b	.L2415
.L2422:
	adrp	x0, .LANCHOR0
	strb	wzr, [x28,304]
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w0, [x0,120]
	cbz	w0, .L2423
	mov	w0, w23
	bl	ftl_get_blk_mode
	strb	w0, [x28,304]
.L2423:
	add	x22, x28, 296
	mov	x0, x22
	bl	make_superblock
	adrp	x1, .LANCHOR4
	ldrh	w2, [x28,296]
	add	x1, x1, :lo12:.LANCHOR4
	ldr	x0, [x28,-40]
	strh	wzr, [x1,402]
	ldrh	w0, [x0,x2,lsl 1]
	strh	w0, [x1,404]
	strh	wzr, [x28,298]
	strb	wzr, [x28,302]
.L2414:
	bl	FtlReadRefresh
	adrp	x23, .LANCHOR0
	mov	w0, 1
	str	w0, [x28,2028]
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w1, [x0,120]
	ldrh	w25, [x0,2026]
	cbz	w1, .L2424
	ldrb	w1, [x28,304]
	cmp	w1, 1
	bne	.L2424
	ldrh	w25, [x0,2028]
.L2424:
	ldrh	w0, [x28,298]
	add	w1, w0, w20
	cmp	w1, w25
	ble	.L2425
	sub	w20, w25, w0
	uxth	w20, w20
.L2425:
	mov	w26, 0
.L2426:
	cmp	w20, w26, uxth
	bls	.L2433
	add	x1, x28, 296
	add	x0, x23, :lo12:.LANCHOR0
	ldrh	w4, [x1,2]
	ldrh	w6, [x0,1952]
	mov	x0, 0
	add	w4, w4, w26
	mov	w22, w0
.L2434:
	cmp	w6, w0, uxth
	bls	.L2519
	add	x2, x1, x0, lsl 1
	ldrh	w2, [x2,16]
	cmp	w2, w24
	beq	.L2427
	mov	w3, 56
	ldr	x7, [x28,912]
	orr	w2, w4, w2, lsl 10
	umull	x5, w22, w3
	add	w22, w22, 1
	add	x5, x7, x5
	uxth	w22, w22
	str	w2, [x5,4]
.L2427:
	add	x0, x0, 1
	b	.L2434
.L2519:
	ldr	x0, [x28,912]
	mov	w1, w22
	ldrb	w2, [x28,304]
	mov	x27, 0
	bl	FlashReadPages
.L2429:
	cmp	w22, w27, uxth
	bls	.L2520
	mov	x0, 56
	mul	x5, x27, x0
	ldr	x0, [x28,912]
	add	x1, x0, x5
	ldr	w0, [x0,x5]
	cmn	w0, #1
	ldr	x4, [x1,16]
	beq	.L2450
	ldrh	w0, [x4]
	mov	w1, 61589
	cmp	w0, w1
	bne	.L2450
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
	bne	.L2450
	adrp	x0, .LANCHOR4
	mov	x3, 56
	add	x1, x0, :lo12:.LANCHOR4
	str	x5, [x29,104]
	str	x4, [x29,112]
	ldrh	w0, [x1,402]
	add	w0, w0, 1
	strh	w0, [x1,402]
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
	cbnz	w0, .L2431
	ldr	x2, [x29,120]
	ldr	w1, [x28,880]
	ldrb	w0, [x2,7]
	cmp	w1, w0
	beq	.L2431
	ldrh	w0, [x2,4]
	cbnz	w0, .L2450
.L2431:
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L2450
	adrp	x0, .LANCHOR4
	add	x19, x19, :lo12:.LANCHOR2
	add	x0, x0, :lo12:.LANCHOR4
	str	wzr, [x19,2028]
	ldrh	w2, [x0,400]
	b	.L2493
.L2450:
	add	x27, x27, 1
	b	.L2429
.L2520:
	add	w26, w26, 1
	b	.L2426
.L2435:
	ldr	w0, [x28,880]
	cbz	w0, .L2437
	bl	Ftl_gc_temp_data_write_back
	cbz	w0, .L2437
	add	x20, x20, :lo12:.LANCHOR4
	str	wzr, [x28,2028]
	ldrh	w2, [x20,400]
	b	.L2493
.L2437:
	add	x0, x20, :lo12:.LANCHOR4
	ldrh	w2, [x0,402]
	cbnz	w2, .L2438
	ldrh	w0, [x28,296]
	ldr	x1, [x28,-40]
	lsl	x0, x0, 1
	ldrh	w4, [x1,x0]
	cbz	w4, .L2438
	strh	w2, [x1,x0]
	ldrh	w0, [x28,296]
	bl	update_vpc_list
	bl	FtlCacheWriteBack
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2438:
	mov	w0, -1
	strh	w0, [x28,296]
	b	.L2436
.L2439:
	add	x20, x20, :lo12:.LANCHOR4
	str	wzr, [x28,2028]
	ldrh	w2, [x20,400]
	cmp	w2, wzr
	csinc	w2, w2, w0, ne
.L2493:
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
	.global	FtlCacheWriteBack
	.type	FtlCacheWriteBack, %function
FtlCacheWriteBack:
	stp	x29, x30, [sp, -112]!
	adrp	x0, .LANCHOR4+408
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR0
	ldr	x20, [x0,#:lo12:.LANCHOR4+408]
	add	x0, x19, :lo12:.LANCHOR0
	str	x27, [sp,80]
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	ldr	w1, [x0,2088]
	cbz	w1, .L2522
	ldrb	w0, [x0,120]
	mov	w22, 0
	cbz	w0, .L2523
	ldrb	w0, [x20,8]
	cmp	w0, 1
	cset	w22, eq
.L2523:
	add	x0, x19, :lo12:.LANCHOR0
	ldrb	w3, [x20,9]
	adrp	x24, .LC118
	mov	w2, w22
	mov	w23, 0
	mov	w25, 56
	ldr	x0, [x0,2096]
	adrp	x26, .LANCHOR2
	add	x24, x24, :lo12:.LC118
	bl	FlashProgPages
.L2524:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w1, [x0,2088]
	cmp	w23, w1
	bcs	.L2541
	umull	x21, w23, w25
	ldr	x1, [x0,2096]
	add	x0, x1, x21
	ldr	w1, [x1,x21]
	cmn	w1, #1
	beq	.L2544
	ldr	w1, [x0,4]
	cbz	w22, .L2558
	orr	w1, w1, -2147483648
.L2558:
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
	beq	.L2528
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	add	x2, x26, :lo12:.LANCHOR2
	ubfiz	x0, x1, 1, 16
	mov	w21, w1
	ldr	x2, [x2,-40]
	ldrh	w2, [x2,x0]
	cbnz	w2, .L2529
	mov	x0, x24
	bl	printk
.L2529:
	mov	w0, w21
	bl	decrement_vpc_count
.L2528:
	add	w23, w23, 1
	b	.L2524
.L2560:
	mov	w20, 16386
	adrp	x21, .LANCHOR2
.L2540:
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w0, [x0,956]
	cbz	w0, .L2541
	mov	w0, 1
	mov	w1, w0
	bl	rk_ftl_garbage_collect
	subs	w20, w20, #1
	bne	.L2540
.L2541:
	add	x19, x19, :lo12:.LANCHOR0
	str	wzr, [x19,2088]
	b	.L2522
.L2544:
	adrp	x23, .LANCHOR2
	adrp	x25, .LC118
	mov	w24, 0
	mov	w26, 56
	add	x23, x23, :lo12:.LANCHOR2
	add	x25, x25, :lo12:.LC118
.L2525:
	add	x0, x19, :lo12:.LANCHOR0
	ldr	w1, [x0,2088]
	cmp	w24, w1
	bcs	.L2560
	umull	x21, w24, w26
	ldr	x0, [x0,2096]
	mov	w1, -1
	str	w1, [x0,x21]
.L2531:
	add	x27, x19, :lo12:.LANCHOR0
	ldr	x0, [x27,2096]
	add	x1, x0, x21
	ldr	w0, [x0,x21]
	cmn	w0, #1
	ldr	w0, [x1,4]
	bne	.L2561
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	ldrh	w1, [x20]
	cmp	w1, w0, uxth
	bne	.L2532
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
.L2532:
	ldrh	w0, [x20,4]
	cbnz	w0, .L2533
	mov	x0, x20
	bl	allocate_new_data_superblock
.L2533:
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
	b	.L2531
.L2561:
	cbz	w22, .L2559
	orr	w0, w0, -2147483648
.L2559:
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
	beq	.L2537
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	uxth	w1, w0
	ubfiz	x0, x1, 1, 16
	ldr	x2, [x23,-40]
	mov	w21, w1
	ldrh	w2, [x2,x0]
	cbnz	w2, .L2538
	mov	x0, x25
	bl	printk
.L2538:
	mov	w0, w21
	bl	decrement_vpc_count
.L2537:
	add	w24, w24, 1
	b	.L2525
.L2522:
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
	bl	FtlVpcTblFlush
	mov	w0, 1
	bl	FtlEctTblFlush
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
	bne	.L2564
	bl	FtlSysFlush
.L2564:
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
	bhi	.L2567
	cmp	w19, 31
	bls	.L2581
	bl	FtlCacheWriteBack
	ldrh	w1, [x21,2032]
	udiv	w21, w20, w1
	msub	w20, w1, w21, w20
	uxth	w20, w20
	cbz	w20, .L2569
	sub	w1, w1, w20
	add	w21, w21, 1
	cmp	w1, w19
	csel	w1, w1, w19, ls
	sub	w19, w19, w1, uxth
.L2569:
	mov	w0, -1
	adrp	x20, .LANCHOR4
	str	w0, [x29,76]
	adrp	x23, .LANCHOR2
.L2570:
	add	x0, x22, :lo12:.LANCHOR0
	ldrh	w0, [x0,2032]
	cmp	w19, w0
	bcc	.L2582
	mov	w0, w21
	add	x1, x29, 72
	mov	w2, 0
	bl	log2phys
	ldr	w0, [x29,72]
	cmn	w0, #1
	beq	.L2571
	add	x1, x20, :lo12:.LANCHOR4
	mov	w2, 1
	ldr	w0, [x1,416]
	add	w0, w0, 1
	str	w0, [x1,416]
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
.L2571:
	add	x0, x22, :lo12:.LANCHOR0
	add	w21, w21, 1
	ldrh	w0, [x0,2032]
	sub	w19, w19, w0
	b	.L2570
.L2582:
	adrp	x0, .LANCHOR4
	add	x0, x0, :lo12:.LANCHOR4
	ldr	w1, [x0,416]
	cmp	w1, 32
	bls	.L2581
	str	wzr, [x0,416]
	bl	l2p_flush
	bl	FtlVpcTblFlush
.L2581:
	mov	w0, 0
.L2567:
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
	str	x25, [sp,64]
	add	x2, x19, :lo12:.LANCHOR2
	stp	x21, x22, [sp,32]
	stp	x23, x24, [sp,48]
	adrp	x21, .LANCHOR0
	ldrh	w20, [x2,96]
	add	x1, x21, :lo12:.LANCHOR0
	cmp	w20, w3
	ldrh	w1, [x1,2026]
	bne	.L2584
.L2594:
	add	x20, x19, :lo12:.LANCHOR2
	mov	w0, 65535
	add	x22, x20, 96
	ldrh	w1, [x20,96]
	str	wzr, [x20,2328]
	cmp	w1, w0
	beq	.L2612
	add	x21, x21, :lo12:.LANCHOR0
	bl	FtlCacheWriteBack
	ldrb	w0, [x22,7]
	mov	w25, 12
	ldr	x1, [x20,-40]
	ldrh	w3, [x21,2026]
	mov	w21, 0
	ldrh	w2, [x20,96]
	mul	w0, w0, w3
	strh	w0, [x1,x2,lsl 1]
	ldr	w0, [x20,184]
	ldrh	w1, [x20,936]
	add	w0, w1, w0
	str	w0, [x20,184]
	b	.L2595
.L2584:
	cbz	w0, .L2587
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	ldrh	w4, [x0,3004]
	cmp	w4, w3
	beq	.L2588
.L2589:
	mov	w1, 2
	b	.L2587
.L2588:
	strh	wzr, [x0,3004]
	ldrh	w0, [x2,-8]
	cmp	w0, 17
	bhi	.L2589
.L2587:
	add	x22, x19, :lo12:.LANCHOR2
	add	x0, x22, 96
	bl	FtlGcScanTempBlk
	str	w0, [x29,92]
	cmn	w0, #1
	beq	.L2590
	ubfiz	x20, x20, 1, 16
	ldr	x1, [x22,-80]
	ldrh	w0, [x1,x20]
	cmp	w0, 4
	bls	.L2591
	sub	w0, w0, #5
	strh	w0, [x1,x20]
	mov	w0, 1
	bl	FtlEctTblFlush
.L2591:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	w1, [x0,2328]
	cbnz	w1, .L2592
	ldr	w1, [x0,448]
	add	w1, w1, 1
	str	w1, [x0,448]
	ldr	w0, [x29,92]
	lsr	w0, w0, 10
	bl	FtlBbmMapBadBlock
	bl	FtlBbmTblFlush
.L2592:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, 1
	str	wzr, [x19,2328]
	b	.L2593
.L2590:
	adrp	x0, .LANCHOR1+3004
	mov	w1, 65535
	ldrh	w2, [x0,#:lo12:.LANCHOR1+3004]
	mov	w0, 1
	cmp	w2, w1
	bne	.L2593
	b	.L2594
.L2598:
	ldr	x24, [x20,944]
	add	x1, x29, 92
	umull	x23, w21, w25
	mov	w2, 0
	add	x22, x24, x23
	ldr	w0, [x22,8]
	bl	log2phys
	ldr	w0, [x29,92]
	ldr	w1, [x24,x23]
	cmp	w0, w1
	bne	.L2596
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	mov	w23, w0
	ldr	w0, [x22,8]
	add	x1, x22, 4
	mov	w2, 1
	bl	log2phys
	mov	w0, w23
.L2611:
	bl	decrement_vpc_count
.L2597:
	add	w21, w21, 1
	uxth	w21, w21
.L2595:
	ldrh	w0, [x20,936]
	cmp	w0, w21
	bhi	.L2598
	b	.L2613
.L2596:
	ldr	w1, [x22,4]
	cmp	w0, w1
	beq	.L2597
	ldrh	w0, [x20,96]
	b	.L2611
.L2613:
	mov	w0, -1
	bl	decrement_vpc_count
	ldrh	w0, [x20,96]
	ldr	x2, [x20,-40]
	ubfiz	x1, x0, 1, 16
	ldrh	w1, [x2,x1]
	cbz	w1, .L2599
	bl	INSERT_DATA_LIST
	b	.L2600
.L2599:
	bl	INSERT_FREE_LIST
.L2600:
	add	x20, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	wzr, [x20,936]
	strh	w0, [x20,96]
	strh	wzr, [x20,924]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	ldr	w0, [x20,160]
	cbz	w0, .L2601
	ldr	w0, [x20,220]
	cmp	w0, 29
	bhi	.L2601
	ldrh	w0, [x20,272]
	ldrh	w1, [x20,-8]
	cmp	w1, w0
	bcs	.L2602
	lsl	w0, w0, 1
	strh	w0, [x20,864]
.L2602:
	add	x19, x19, :lo12:.LANCHOR2
	mov	w0, -1
	strh	w0, [x19,296]
	b	.L2612
.L2601:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w0, [x19,272]
	ldrh	w1, [x19,-8]
	add	w0, w0, w0, lsl 1
	cmp	w1, w0, lsr 2
	ble	.L2612
	mov	w0, -1
	strh	w0, [x19,296]
	mov	w0, 20
	strh	w0, [x19,864]
.L2612:
	mov	w0, 0
.L2593:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldr	x25, [sp,64]
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
	cbz	w0, .L2615
	add	x0, x21, :lo12:.LANCHOR2
	ldr	w1, [x0,880]
	tbz	x1, 0, .L2615
	ldrh	w0, [x0,100]
	cbz	w0, .L2615
.L2620:
	mov	w0, 0
	b	.L2616
.L2615:
	add	x1, x21, :lo12:.LANCHOR2
	mov	w2, 0
	mov	w3, w2
	mov	w20, 0
	mov	w22, 56
	ldr	x0, [x1,2056]
	ldr	w1, [x1,880]
	bl	FlashProgPages
.L2617:
	add	x19, x21, :lo12:.LANCHOR2
	ldr	w1, [x19,880]
	cmp	w20, w1
	bcs	.L2631
	umull	x0, w20, w22
	ldr	x2, [x19,2056]
	add	x1, x2, x0
	ldr	w2, [x2,x0]
	cmn	w2, #1
	bne	.L2618
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
	b	.L2630
.L2618:
	ldr	x2, [x1,16]
	add	w20, w20, 1
	ldr	w1, [x1,4]
	uxth	w20, w20
	ldr	w0, [x2,12]
	ldr	w2, [x2,8]
	bl	FtlGcUpdatePage
	b	.L2617
.L2631:
	ldr	x0, [x19,2056]
	bl	FtlGcBufFree
	str	wzr, [x19,880]
	ldrh	w0, [x19,100]
	cbnz	w0, .L2620
	mov	w0, 1
	bl	FtlGcFreeTempBlock
.L2630:
	mov	w0, 1
.L2616:
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
	bcc	.L2632
	add	x0, x19, 2240
	bl	FtlMapBlkWriteDumpData
	mov	w0, 0
	bl	FtlGcFreeTempBlock
	str	wzr, [x19,2328]
.L2632:
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
	adrp	x0, .LANCHOR4
	add	x29, sp, 0
	add	x0, x0, :lo12:.LANCHOR4
	str	x19, [sp,16]
	adrp	x19, .LANCHOR2
	add	x19, x19, :lo12:.LANCHOR2
	strh	wzr, [x0,266]
	str	wzr, [x0,268]
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
	stp	x19, x20, [sp,16]
	adrp	x20, .LANCHOR0
	str	x23, [sp,48]
	add	x20, x20, :lo12:.LANCHOR0
	adrp	x19, .LANCHOR2
	stp	x21, x22, [sp,32]
	add	x21, x19, :lo12:.LANCHOR2
	ldrh	w0, [x20,1956]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlScanSysBlk
	ldrh	w1, [x21,280]
	mov	w0, 65535
	cmp	w1, w0
	bne	.L2636
.L2638:
	mov	w22, -1
	b	.L2637
.L2636:
	bl	FtlLoadSysInfo
	mov	w22, w0
	cbnz	w0, .L2638
	bl	FtlLoadMapInfo
	bl	FtlLoadVonderInfo
	bl	Ftl_load_ext_data
	bl	FtlLoadEctTbl
	bl	FtlFreeSysBLkSort
	bl	SupperBlkListInit
	bl	FtlPowerLostRecovery
	ldrh	w2, [x20,2066]
	mov	x1, 0
	ldr	x0, [x21,144]
.L2639:
	cmp	w1, w2
	mov	w3, w1
	bge	.L2643
	add	x4, x0, x1, lsl 4
	add	x1, x1, 1
	ldr	w4, [x4,4]
	tbz	w4, #31, .L2639
.L2643:
	add	x1, x19, :lo12:.LANCHOR2
	cmp	w3, w2
	adrp	x21, .LANCHOR4
	ldrh	w0, [x1,252]
	add	w0, w0, 1
	strh	w0, [x1,252]
	blt	.L2640
	add	x0, x21, :lo12:.LANCHOR4
	ldrh	w0, [x0,266]
	cbz	w0, .L2644
.L2640:
	add	x20, x19, :lo12:.LANCHOR2
	mov	x0, x20
	bl	FtlSuperblockPowerLostFix
	add	x0, x20, 48
	bl	FtlSuperblockPowerLostFix
	add	x0, x20, 2240
	bl	FtlMapBlkWriteDumpData
	add	x0, x21, :lo12:.LANCHOR4
	add	x0, x0, 200
	bl	FtlMapBlkWriteDumpData
	ldrh	w0, [x20,254]
	add	w0, w0, 1
	strh	w0, [x20,254]
	bl	l2p_flush
	bl	FtlVpcTblFlush
	bl	FtlVpcTblFlush
.L2644:
	mov	w0, 1
	add	x20, x19, :lo12:.LANCHOR2
	bl	FtlUpdateVaildLpn
	ldrh	w0, [x19,#:lo12:.LANCHOR2]
	mov	w1, 65535
	cmp	w0, w1
	beq	.L2645
	ldrh	w1, [x20,4]
	cbnz	w1, .L2645
	ldrh	w1, [x20,52]
	add	x23, x20, 48
	cbnz	w1, .L2645
	bl	FtlGcRefreshBlock
	ldrh	w0, [x20,48]
	bl	FtlGcRefreshBlock
	mov	x0, x20
	bl	allocate_new_data_superblock
	mov	x0, x23
	bl	allocate_new_data_superblock
	add	x0, x21, :lo12:.LANCHOR4
	add	x0, x0, 200
	bl	FtlMapBlkWriteDumpData
.L2645:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w0, [x19,252]
	and	w0, w0, 31
	cbnz	w0, .L2637
	bl	FtlVpcCheckAndModify
.L2637:
	mov	w0, w22
	ldr	x23, [sp,48]
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 64
	ret
	.size	FtlSysBlkInit, .-FtlSysBlkInit
	.align	2
	.global	FtlInit
	.type	FtlInit, %function
FtlInit:
	stp	x29, x30, [sp, -32]!
	adrp	x1, .LC74
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	adrp	x19, .LANCHOR1
	mov	x20, x0
	add	x19, x19, :lo12:.LANCHOR1
	mov	w0, -1
	add	x1, x1, :lo12:.LC74
	str	w0, [x19,532]
	adrp	x0, .LANCHOR4+420
	str	wzr, [x0,#:lo12:.LANCHOR4+420]
	adrp	x0, .LC73
	add	x0, x0, :lo12:.LC73
	bl	printk
	mov	x0, x20
	bl	FtlConstantsInit
	bl	FtlMemInit
	bl	FtlVariablesInit
	adrp	x0, .LANCHOR0+1956
	ldrh	w0, [x0,#:lo12:.LANCHOR0+1956]
	bl	FtlFreeSysBlkQueueInit
	bl	FtlLoadBbt
	cbnz	w0, .L2653
	bl	FtlSysBlkInit
	cbnz	w0, .L2653
	mov	w0, 1
	str	w0, [x19,532]
.L2653:
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
	cbz	x1, .L2656
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
	cbnz	w0, .L2657
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	add	x0, x0, 168
	bl	FtlInit
.L2657:
	adrp	x0, .LC119
	mov	w1, w19
	add	x0, x0, :lo12:.LC119
	bl	printk
	mov	w0, w19
.L2656:
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	rk_ftl_init, .-rk_ftl_init
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
	beq	.L2661
	ldrh	w0, [x1,4]
	cbnz	w0, .L2662
.L2661:
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
	bl	FtlVpcTblFlush
	mov	w0, 0
	bl	FtlEctTblFlush
.L2662:
	add	x0, x19, :lo12:.LANCHOR2
	add	x0, x0, 96
	bl	get_new_active_ppa
	ldp	x19, x20, [sp,16]
	ldp	x29, x30, [sp], 32
	ret
	.size	Ftl_get_new_temp_ppa, .-Ftl_get_new_temp_ppa
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
	bne	.L2665
	add	w0, w1, 256
	mov	w1, w2
	mov	x2, x3
	bl	FtlVendorPartRead
	b	.L2666
.L2665:
	adrp	x21, .LANCHOR0
	add	w0, w1, w2
	add	x1, x21, :lo12:.LANCHOR0
	str	w0, [x29,152]
	mov	w0, -1
	ldr	w3, [x29,152]
	ldr	w2, [x1,2068]
	cmp	w3, w2
	bhi	.L2666
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
	cbz	w0, .L2667
	bl	FtlCacheWriteBack
.L2667:
	mov	w27, 0
	ldr	w22, [x29,156]
	str	w27, [x29,140]
	mov	w23, w27
	str	w27, [x29,136]
.L2668:
	cbz	w24, .L2710
	mov	w0, w22
	add	x1, x29, 172
	mov	w2, 0
	bl	log2phys
	ldr	w3, [x29,172]
	cmn	w3, #1
	bne	.L2707
	mov	w3, 0
.L2669:
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,2032]
	cmp	w3, w0
	bcs	.L2673
	madd	w0, w22, w0, w3
	cmp	w0, w20
	bcc	.L2671
	ldr	w1, [x29,152]
	cmp	w0, w1
	bcs	.L2671
	sub	w0, w0, w20
	mov	w1, 0
	ubfiz	x0, x0, 9, 23
	mov	w2, 512
	add	x0, x26, x0
	str	x3, [x29,144]
	bl	ftl_memset
	ldr	x3, [x29,144]
.L2671:
	add	w3, w3, 1
	b	.L2669
.L2707:
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
	bne	.L2674
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
	bne	.L2675
	str	x26, [x2,8]
	b	.L2675
.L2674:
	cmp	w22, w25
	bne	.L2676
	ldr	x0, [x0,2104]
	str	x0, [x2,8]
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w4, [x29,152]
	ldrh	w3, [x0,2032]
	mul	w0, w22, w3
	sub	w27, w4, w0
	cmp	w27, w3
	bne	.L2675
	b	.L2708
.L2676:
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,2032]
	mul	w0, w22, w0
.L2708:
	sub	w0, w0, w20
	ubfiz	x0, x0, 9, 23
	add	x0, x26, x0
	str	x0, [x2,8]
.L2675:
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
.L2673:
	subs	w24, w24, #1
	add	w22, w22, 1
	beq	.L2677
	add	x0, x21, :lo12:.LANCHOR0
	ldrh	w0, [x0,1952]
	cmp	w23, w0, lsl 3
	bne	.L2668
.L2677:
	cbz	w23, .L2668
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
.L2679:
	ldr	w0, [x29,144]
	cmp	w23, w0
	bls	.L2711
	ldr	x0, [x29,144]
	add	x2, x19, :lo12:.LANCHOR2
	mov	x1, 56
	ldr	w4, [x29,156]
	mul	x3, x0, x1
	ldr	x0, [x2,2048]
	add	x0, x0, x3
	ldr	w1, [x0,24]
	cmp	w1, w4
	bne	.L2680
	ldr	x1, [x0,8]
	ldr	x0, [x2,2096]
	cmp	x1, x0
	bne	.L2681
	ldr	x2, [x29,128]
	mov	x0, x26
	str	x3, [x29,96]
	add	x1, x1, x2
	ldr	w2, [x29,108]
	b	.L2709
.L2680:
	cmp	w1, w25
	bne	.L2681
	ldr	x1, [x0,8]
	ldr	x0, [x2,2104]
	cmp	x1, x0
	bne	.L2681
	add	x0, x21, :lo12:.LANCHOR0
	ldr	w2, [x29,104]
	str	x3, [x29,96]
	ldrh	w0, [x0,2032]
	mul	w0, w25, w0
	sub	w0, w0, w20
	ubfiz	x0, x0, 9, 23
	add	x0, x26, x0
.L2709:
	bl	ftl_memcpy
	ldr	x3, [x29,96]
.L2681:
	add	x0, x19, :lo12:.LANCHOR2
	ldr	x1, [x0,2048]
	add	x5, x1, x3
	ldr	w4, [x1,x3]
	cmn	w4, #1
	bne	.L2682
	ldr	w2, [x0,424]
	str	w4, [x29,136]
	add	w2, w2, 1
	str	w2, [x0,424]
.L2682:
	ldr	w0, [x1,x3]
	cmp	w0, 256
	bne	.L2683
	ldr	w0, [x5,4]
	lsr	x0, x0, 10
	bl	P2V_block_in_plane
	bl	FtlGcRefreshBlock
.L2683:
	ldr	x0, [x29,144]
	add	x0, x0, 1
	str	x0, [x29,144]
	b	.L2679
.L2711:
	mov	w23, 0
	b	.L2668
.L2710:
	add	x19, x19, :lo12:.LANCHOR2
	ldrh	w0, [x19,956]
	cbz	w0, .L2686
	mov	w0, w24
	mov	w1, 1
	bl	rk_ftl_garbage_collect
.L2686:
	ldr	w0, [x29,136]
.L2666:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 176
	ret
	.size	FtlRead, .-FtlRead
	.align	2
	.global	FtlWrite
	.type	FtlWrite, %function
FtlWrite:
	stp	x29, x30, [sp, -272]!
	uxtb	w0, w0
	add	x29, sp, 0
	stp	x23, x24, [sp,48]
	stp	x25, x26, [sp,64]
	stp	x19, x20, [sp,16]
	stp	x21, x22, [sp,32]
	stp	x27, x28, [sp,80]
	cmp	w0, 16
	mov	w23, w1
	mov	w24, w2
	mov	x25, x3
	bne	.L2713
	add	w0, w1, 256
	mov	w1, w2
	mov	x2, x3
	bl	FtlVendorPartWrite
	b	.L2714
.L2713:
	adrp	x22, .LANCHOR0
	add	w27, w1, w2
	add	x2, x22, :lo12:.LANCHOR0
	mov	w0, -1
	ldr	w1, [x2,2068]
	cmp	w27, w1
	bhi	.L2714
	adrp	x5, .LANCHOR4
	ldrh	w1, [x2,2032]
	add	x4, x5, :lo12:.LANCHOR4
	sub	w27, w27, #1
	mov	w0, 2048
	adrp	x20, .LANCHOR2
	udiv	w26, w23, w1
	cmp	w24, w1, lsl 1
	str	w0, [x4,440]
	str	x5, [x29,152]
	udiv	w0, w27, w1
	str	w0, [x29,180]
	sub	w28, w0, w26
	add	w21, w28, 1
	add	x0, x20, :lo12:.LANCHOR2
	add	x19, x0, 48
	ldr	w3, [x0,172]
	add	w3, w21, w3
	str	w3, [x0,172]
	ldr	w3, [x0,188]
	add	w3, w24, w3
	str	w3, [x0,188]
	cset	w3, cs
	str	w3, [x29,204]
	ldr	w3, [x2,2088]
	cmp	w24, 8
	csel	x19, x0, x19, hi
	cbz	w3, .L2716
	sub	w3, w3, #1
	mov	w5, 56
	ldr	x2, [x2,2096]
	umull	x3, w3, w5
	add	x3, x2, x3
	ldr	w2, [x3,24]
	cmp	w26, w2
	bne	.L2717
	ldr	w2, [x0,176]
	str	x4, [x29,192]
	add	w2, w2, 1
	str	w2, [x0,176]
	ldr	w0, [x4,444]
	add	w0, w0, 1
	str	w0, [x4,444]
	msub	w0, w26, w1, w23
	sub	w1, w1, w0
	ubfiz	x0, x0, 9, 23
	cmp	w1, w24
	csel	w19, w1, w24, ls
	ldr	x1, [x3,8]
	lsl	w21, w19, 9
	add	x0, x1, x0
	mov	w2, w21
	mov	x1, x25
	bl	ftl_memcpy
	ldr	x4, [x29,192]
	cbnz	w28, .L2718
	ldr	w1, [x4,444]
	mov	w0, w28
	cmp	w1, 2
	ble	.L2714
.L2718:
	add	x25, x25, x21
	sub	w24, w24, w19
	add	w23, w23, w19
	add	w26, w26, 1
	mov	w21, w28
.L2717:
	ldr	x0, [x29,152]
	add	x0, x0, :lo12:.LANCHOR4
	ldr	x19, [x0,408]
	str	wzr, [x0,444]
.L2716:
	ldr	w1, [x29,180]
	mov	w0, w26
	bl	FtlCacheMetchLpa
	cbz	w0, .L2719
	bl	FtlCacheWriteBack
.L2719:
	ldr	x0, [x29,152]
	add	x20, x20, :lo12:.LANCHOR2
	str	w26, [x29,200]
	add	x0, x0, :lo12:.LANCHOR4
	str	x19, [x0,408]
	adrp	x0, .LC120
	add	x0, x0, :lo12:.LC120
	str	x0, [x29,112]
.L2720:
	cbz	w21, .L2778
	ldrh	w0, [x19,4]
	cbnz	w0, .L2721
	cmp	x19, x20
	bne	.L2722
	add	x0, x20, 48
	ldrh	w19, [x0,4]
	cbnz	w19, .L2723
	bl	allocate_new_data_superblock
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	str	w19, [x0,3008]
.L2723:
	mov	x0, x20
	add	x19, x20, 48
	bl	allocate_new_data_superblock
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	ldr	w0, [x0,3008]
	cbnz	w0, .L2724
.L2725:
	mov	x19, x20
	b	.L2724
.L2722:
	adrp	x1, .LANCHOR1
	add	x1, x1, :lo12:.LANCHOR1
	str	w0, [x1,3008]
	ldrh	w0, [x20,4]
	cbnz	w0, .L2725
	mov	x0, x19
	bl	allocate_new_data_superblock
.L2724:
	ldrh	w0, [x19,4]
	cbnz	w0, .L2726
	mov	x0, x19
	bl	allocate_new_data_superblock
.L2726:
	ldr	x0, [x29,152]
	add	x0, x0, :lo12:.LANCHOR4
	str	x19, [x0,408]
.L2721:
	add	x2, x22, :lo12:.LANCHOR0
	ldr	w0, [x20,2040]
	ldrh	w1, [x19,4]
	str	xzr, [x29,192]
	ldr	w2, [x2,2088]
	sub	w0, w0, w2
	cmp	w1, w0
	csel	w0, w1, w0, ls
	cmp	w0, w21
	csel	w0, w0, w21, ls
	str	w0, [x29,148]
	ldr	w0, [x29,204]
	and	w0, w0, 1
	str	w0, [x29,124]
	add	w0, w24, w23
	str	w0, [x29,144]
.L2727:
	ldr	w0, [x29,192]
	ldr	w2, [x29,148]
	ldr	w1, [x29,200]
	cmp	w0, w2
	add	w27, w1, w0
	bcs	.L2779
	ldrh	w1, [x19,4]
	cbz	w1, .L2756
	ldr	w1, [x29,180]
	cmp	w27, w1
	cset	w7, eq
	cbz	w0, .L2729
	ldr	w1, [x29,124]
	tst	w7, w1
	beq	.L2729
	add	x1, x22, :lo12:.LANCHOR0
	ldr	w2, [x29,144]
	ldrh	w1, [x1,2032]
	msub	w2, w27, w1, w2
	cmp	w2, w1
	bne	.L2756
.L2729:
	add	x1, x29, 212
	mov	w2, 0
	mov	w0, w27
	str	x7, [x29,104]
	bl	log2phys
	mov	x0, x19
	bl	get_new_active_ppa
	add	x4, x22, :lo12:.LANCHOR0
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
	ldr	x0, [x20,2128]
	ldr	x3, [x29,136]
	str	x0, [x29,128]
	add	x28, x0, x3
	ldrh	w0, [x4,2036]
	str	x28, [x1,16]
	str	w27, [x1,24]
	mul	w0, w0, w6
	ldr	x6, [x20,2088]
	and	x0, x0, 4294967292
	add	x0, x6, x0
	str	x0, [x1,8]
	mov	x0, x28
	mov	w1, 0
	bl	ftl_memset
	cmp	w27, w26
	ldr	x4, [x29,184]
	cset	w0, eq
	str	w0, [x29,176]
	cbnz	w0, .L2759
	ldr	x7, [x29,104]
	ldr	x5, [x29,160]
	cbz	w7, .L2730
	ldr	w0, [x29,144]
	ldrh	w2, [x4,2032]
	msub	w2, w27, w2, w0
	uxth	w0, w2
	str	w0, [x29,184]
	ldr	w0, [x29,176]
	str	w0, [x29,160]
	b	.L2733
.L2759:
	ldrh	w2, [x4,2032]
	udiv	w0, w23, w2
	msub	w0, w0, w2, w23
	str	w0, [x29,160]
	sub	w2, w2, w0
	cmp	w2, w24
	csel	w0, w2, w24, ls
	str	w0, [x29,184]
.L2733:
	add	x2, x22, :lo12:.LANCHOR0
	ldr	w1, [x29,184]
	ldrh	w0, [x2,2032]
	cmp	w1, w0
	bne	.L2734
	ldr	w0, [x29,176]
	mov	x1, x25
	cbnz	w0, .L2735
	ldr	w0, [x29,184]
	mul	w1, w0, w27
	sub	w1, w1, w23
	ubfiz	x1, x1, 9, 23
	add	x1, x25, x1
.L2735:
	ldr	w2, [x29,204]
	mov	w0, 56
	add	x4, x22, :lo12:.LANCHOR0
	cbz	w2, .L2736
	ldr	w2, [x4,2088]
	umull	x0, w2, w0
	ldr	x2, [x4,2096]
	add	x0, x2, x0
	b	.L2777
.L2736:
	ldr	w2, [x4,2088]
	umull	x0, w2, w0
	ldr	x2, [x4,2096]
	add	x0, x2, x0
	ldr	x0, [x0,8]
	b	.L2775
.L2734:
	ldr	w0, [x29,212]
	cmn	w0, #1
	beq	.L2738
	str	w0, [x29,220]
	mov	w1, 56
	ldr	w0, [x2,2088]
	str	w27, [x29,240]
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
	bne	.L2739
	ldr	w0, [x20,424]
	add	w0, w0, 1
	str	w0, [x20,424]
	b	.L2741
.L2739:
	ldr	w0, [x28,8]
	cmp	w0, w27
	beq	.L2741
	ldr	w0, [x20,424]
	mov	w2, w27
	add	w0, w0, 1
	str	w0, [x20,424]
	ldr	x0, [x29,112]
	ldr	w1, [x28,8]
	bl	printk
	b	.L2741
.L2738:
	ldr	w0, [x2,2088]
	mov	w1, 56
	umull	x0, w0, w1
	ldr	x1, [x2,2096]
	ldrh	w2, [x2,2036]
	add	x0, x1, x0
	mov	w1, 0
	ldr	x0, [x0,8]
	bl	ftl_memset
.L2741:
	ldr	w1, [x29,184]
	mov	w0, 56
	lsl	w2, w1, 9
	ldr	w1, [x29,176]
	cbz	w1, .L2742
	add	x1, x22, :lo12:.LANCHOR0
	ldr	w4, [x1,2088]
	umull	x0, w4, w0
	ldr	x4, [x1,2096]
	add	x4, x4, x0
	ldr	x0, [x29,160]
	ubfiz	x0, x0, 9, 23
	ldr	x1, [x4,8]
	add	x0, x1, x0
	mov	x1, x25
	b	.L2776
.L2742:
	add	x5, x22, :lo12:.LANCHOR0
	ldr	w4, [x5,2088]
	ldr	x1, [x5,2096]
	umull	x0, w4, w0
	add	x0, x1, x0
	ldrh	w1, [x5,2032]
	mul	w1, w27, w1
	ldr	x0, [x0,8]
	sub	w1, w1, w23
	ubfiz	x1, x1, 9, 23
	add	x1, x25, x1
	b	.L2776
.L2730:
	ldr	w0, [x29,204]
	ldrh	w1, [x4,2032]
	ldr	w2, [x4,2088]
	cbz	w0, .L2743
	mul	w1, w27, w1
	umull	x0, w2, w5
	sub	w1, w1, w23
	ldr	x2, [x4,2096]
	ubfiz	x1, x1, 9, 23
	add	x0, x2, x0
	add	x1, x25, x1
.L2777:
	str	x1, [x0,8]
	b	.L2737
.L2743:
	umull	x0, w2, w5
	ldr	x2, [x4,2096]
	mul	w1, w27, w1
	add	x0, x2, x0
	sub	w1, w1, w23
	ubfiz	x1, x1, 9, 23
	ldr	x0, [x0,8]
	add	x1, x25, x1
.L2775:
	ldrh	w2, [x4,2036]
.L2776:
	bl	ftl_memcpy
.L2737:
	ldr	x1, [x29,128]
	mov	w0, -3947
	ldr	x2, [x29,136]
	strh	w0, [x1,x2]
	add	x1, x22, :lo12:.LANCHOR0
	str	w27, [x28,8]
	ldr	w0, [x20,200]
	str	w0, [x28,4]
	add	w0, w0, 1
	cmn	w0, #1
	csel	w0, w0, wzr, ne
	str	w0, [x20,200]
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
	b	.L2727
.L2779:
	str	w27, [x29,200]
	mov	x0, x2
	b	.L2728
.L2756:
	str	w27, [x29,200]
.L2728:
	sub	w21, w21, w0
	ldr	w0, [x29,204]
	cbnz	w0, .L2747
	add	x0, x22, :lo12:.LANCHOR0
	ldr	w1, [x0,2088]
	ldr	w0, [x20,2040]
	cmp	w1, w0
	bcs	.L2747
	ldrh	w0, [x19,4]
	cbz	w0, .L2747
.L2749:
	str	wzr, [x29,204]
	b	.L2720
.L2747:
	bl	FtlCacheWriteBack
	add	x0, x22, :lo12:.LANCHOR0
	cmp	w21, 3
	str	wzr, [x0,2088]
	bls	.L2749
	b	.L2720
.L2778:
	ldr	w1, [x29,180]
	mov	w0, w21
	sub	w1, w1, w26
	bl	rk_ftl_garbage_collect
	mov	w0, w21
.L2714:
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x23, x24, [sp,48]
	ldp	x25, x26, [sp,64]
	ldp	x27, x28, [sp,80]
	ldp	x29, x30, [sp], 272
	ret
	.size	FtlWrite, .-FtlWrite
	.align	2
	.type	FlashReadFacBbtData.part.14, %function
FlashReadFacBbtData.part.14:
	stp	x29, x30, [sp, -32]!
	mov	w3, 0
	adrp	x4, .LANCHOR0
	add	x29, sp, 0
	str	x19, [sp,16]
	mov	x19, x0
	cbnz	w1, .L2783
	adrp	x1, .LANCHOR4
	mov	w7, 1
.L2786:
	add	x0, x1, :lo12:.LANCHOR4
	ldr	w0, [x0,68]
	cmp	w3, w0
	bcs	.L2783
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
	b	.L2786
.L2783:
	add	x4, x4, :lo12:.LANCHOR0
	mov	x0, x19
	ldr	x1, [x4,1856]
	bl	ftl_memcpy
	mov	w2, 4
	adrp	x0, .LC121
	mov	x1, x19
	add	x0, x0, :lo12:.LC121
	mov	w3, w2
	bl	rknand_print_hex
	ldr	x19, [sp,16]
	mov	w0, 0
	ldp	x29, x30, [sp], 32
	ret
	.size	FlashReadFacBbtData.part.14, .-FlashReadFacBbtData.part.14
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
.L2788:
	cmp	w20, w19
	ble	.L2796
	add	w0, w20, w24
	mov	w1, 1
	lsl	w0, w0, 10
	mov	w2, w1
	str	w0, [x29,92]
	add	x0, x29, 88
	bl	FlashReadPages
	ldr	w0, [x29,88]
	cmn	w0, #1
	beq	.L2789
	add	x0, x21, :lo12:.LANCHOR4
	ldr	x0, [x0,88]
	ldrh	w0, [x0]
	cmp	w0, w26
	bne	.L2789
	mov	w0, w22
	cbz	x22, .L2790
	mov	x0, x22
	mov	w1, w23
	mov	w2, w25
	bl	FlashReadFacBbtData.part.14
	b	.L2790
.L2789:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L2788
.L2796:
	mov	w0, -1
.L2790:
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
	bne	.L2798
.L2802:
	mov	w1, 0
	b	.L2799
.L2798:
	mov	w2, 0
	lsr	w0, w19, 4
	mov	w1, w2
	sub	w19, w19, #1
	mov	w5, 1
.L2800:
	cmp	w2, w19
	bge	.L2799
	add	x3, x20, :lo12:.LANCHOR4
	ubfx	x4, x2, 5, 11
	ldr	x6, [x3,80]
	lsl	w3, w5, w2
	ldr	w4, [x6,x4,lsl 2]
	tst	w3, w4
	beq	.L2801
	add	w3, w1, 1
	ubfiz	x1, x1, 1, 16
	strh	w2, [x21,x1]
	uxth	w1, w3
.L2801:
	cmp	w1, w0
	bcs	.L2802
	add	w2, w2, 1
	uxth	w2, w2
	b	.L2800
.L2799:
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
.L2808:
	add	x21, x19, :lo12:.LANCHOR0
	ldrh	w0, [x21,1974]
	cmp	w23, w0
	bcs	.L2826
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
	beq	.L2809
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
	b	.L2810
.L2809:
	mov	w1, w23
	mov	w28, -3872
	bl	FlashGetBadBlockList
	ldr	x0, [x20,8]
	ldr	x1, [x22,32]
	bl	FtlBbt2Bitmap
	ldrh	w21, [x21,2022]
	sub	w21, w21, #1
	uxth	w21, w21
.L2811:
	add	x27, x19, :lo12:.LANCHOR0
	ldrh	w0, [x27,2022]
	madd	w0, w23, w0, w21
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	bne	.L2827
	sub	w21, w21, #1
	uxth	w21, w21
	b	.L2811
.L2827:
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
	bne	.L2810
	mov	w0, w27
	bl	FtlBbmMapBadBlock
	b	.L2811
.L2810:
	mov	w0, w27
	add	w23, w23, 1
	add	x22, x22, 8
	add	x24, x24, 2
	bl	FtlBbmMapBadBlock
	b	.L2808
.L2826:
	mov	w20, 0
.L2815:
	add	x21, x19, :lo12:.LANCHOR0
	ldrh	w0, [x21,2040]
	cmp	w0, w20
	bls	.L2828
	mov	w0, w20
	add	w20, w20, 1
	bl	FtlBbmMapBadBlock
	uxth	w20, w20
	b	.L2815
.L2828:
	ldrh	w20, [x21,2124]
	mov	w23, 65535
	sub	w20, w20, #1
	uxth	w20, w20
.L2817:
	add	x22, x21, 2112
	ldrh	w0, [x22,12]
	sub	w0, w0, #48
	cmp	w20, w0
	ble	.L2821
	mov	w0, w20
	bl	FtlBbmIsBadBlock
	cmp	w0, 1
	beq	.L2818
	mov	w0, w20
	bl	FlashTestBlk
	cbz	w0, .L2819
	mov	w0, w20
	bl	FtlBbmMapBadBlock
	b	.L2818
.L2819:
	ldrh	w0, [x21,2112]
	cmp	w0, w23
	bne	.L2820
	strh	w20, [x21,2112]
	b	.L2818
.L2820:
	strh	w20, [x22,4]
	b	.L2821
.L2818:
	sub	w20, w20, #1
	uxth	w20, w20
	b	.L2817
.L2821:
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
	cbz	w0, .L2830
	bl	FtlMakeBbt
.L2830:
	mov	w0, 0
.L2831:
	add	x1, x20, :lo12:.LANCHOR0
	ldrh	w2, [x1,2032]
	cmp	w0, w2, lsl 7
	bge	.L2854
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
	b	.L2831
.L2854:
	ldrh	w22, [x1,1960]
	mov	w19, 0
.L2833:
	add	x23, x20, :lo12:.LANCHOR0
	ldrh	w0, [x23,1962]
	cmp	w0, w22
	bls	.L2855
	mov	w0, w22
	mov	w1, 1
	add	w22, w22, 1
	bl	FtlLowFormatEraseBlock
	add	w0, w19, w0
	uxth	w22, w22
	uxth	w19, w0
	b	.L2833
.L2855:
	ldrh	w0, [x23,1952]
	sub	w1, w19, #3
	cmp	w1, w0, lsl 1
	bge	.L2835
.L2839:
	mov	w19, 0
	mov	w22, w19
	b	.L2836
.L2835:
	udiv	w19, w19, w0
	ldr	w0, [x23,2060]
	add	w0, w19, w0
	bl	FtlSysBlkNumInit
	ldrh	w0, [x23,1956]
	bl	FtlFreeSysBlkQueueInit
	ldrh	w19, [x23,1960]
.L2837:
	add	x0, x20, :lo12:.LANCHOR0
	ldrh	w0, [x0,1962]
	cmp	w0, w19
	bls	.L2839
	mov	w0, w19
	mov	w1, 1
	add	w19, w19, 1
	bl	FtlLowFormatEraseBlock
	uxth	w19, w19
	b	.L2837
.L2836:
	add	x2, x20, :lo12:.LANCHOR0
	ldrh	w0, [x2,1960]
	cmp	w0, w22
	bls	.L2856
	mov	w0, w22
	mov	w1, 0
	add	w22, w22, 1
	bl	FtlLowFormatEraseBlock
	add	w0, w19, w0
	uxth	w22, w22
	uxth	w19, w0
	b	.L2836
.L2856:
	ldrh	w3, [x2,1952]
	add	x6, x21, :lo12:.LANCHOR2
	ldr	w1, [x2,1964]
	ldrh	w0, [x2,1962]
	str	w0, [x2,2108]
	udiv	w4, w1, w3
	str	w4, [x2,2104]
	ubfx	x5, x4, 5, 16
	add	w0, w5, 36
	strh	w0, [x6,272]
	mov	w0, 24
	mul	w0, w3, w0
	cmp	w19, w0
	ble	.L2841
	sub	w1, w1, w19
	udiv	w1, w1, w3
	str	w1, [x2,2104]
	lsr	w1, w1, 5
	add	w1, w1, 24
	strh	w1, [x6,272]
.L2841:
	add	x2, x21, :lo12:.LANCHOR2
	ldr	w0, [x2,160]
	cmp	w0, 1
	bne	.L2842
	ldrh	w1, [x2,272]
	udiv	w0, w19, w3
	add	w0, w1, w0
	add	w0, w1, w0, asr 2
	strh	w0, [x2,272]
.L2842:
	add	x6, x20, :lo12:.LANCHOR0
	ldrh	w1, [x6,2018]
	cbz	w1, .L2844
	add	x0, x21, :lo12:.LANCHOR2
	ldrh	w2, [x0,272]
	add	w2, w2, w1, lsr 1
	strh	w2, [x0,272]
	mul	w2, w1, w3
	cmp	w2, w19
	ble	.L2844
	add	w1, w1, 32
	str	w4, [x6,2104]
	add	w1, w5, w1
	strh	w1, [x0,272]
.L2844:
	add	x19, x21, :lo12:.LANCHOR2
	add	x22, x20, :lo12:.LANCHOR0
	adrp	x24, .LANCHOR4
	mov	w23, -1
	ldr	w1, [x22,2104]
	ldrh	w0, [x19,272]
	sub	w0, w1, w0
	add	x1, x24, :lo12:.LANCHOR4
	mul	w0, w0, w3
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
.L2846:
	mov	x0, x19
	bl	make_superblock
	ldrb	w0, [x19,7]
	cbnz	w0, .L2847
	ldrh	w1, [x19]
	ldr	x0, [x19,-40]
	strh	w23, [x0,x1,lsl 1]
	ldrh	w0, [x19]
	add	w0, w0, 1
	strh	w0, [x19]
	b	.L2846
.L2847:
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
.L2848:
	mov	x0, x20
	bl	make_superblock
	ldrb	w0, [x20,7]
	cbnz	w0, .L2849
	ldrh	w1, [x20]
	ldr	x0, [x19,-40]
	strh	w22, [x0,x1,lsl 1]
	ldrh	w0, [x20]
	add	w0, w0, 1
	strh	w0, [x20]
	b	.L2848
.L2849:
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
	cbnz	w0, .L2850
	adrp	x0, .LANCHOR1+532
	mov	w1, 1
	str	w1, [x0,#:lo12:.LANCHOR1+532]
.L2850:
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
	str	x19, [sp,16]
	adrp	x19, .LANCHOR4
	add	x0, x19, :lo12:.LANCHOR4
	ldr	x0, [x0,32]
	bl	FlashInit
	mov	w1, 0
	cbnz	w0, .L2858
	bl	FlashLoadFactorBbt
	cbz	w0, .L2859
	bl	FlashMakeFactorBbt
.L2859:
	add	x19, x19, :lo12:.LANCHOR4
	ldr	x0, [x19,80]
	bl	FlashReadIdbDataRaw
	cbz	w0, .L2860
	mov	w1, 0
	mov	w2, 16
	add	x0, x29, 32
	bl	FlashReadFacBbtData
	mov	w0, 0
	ldr	w2, [x29,32]
	mov	w1, w0
	mov	w4, 1
.L2862:
	lsl	w3, w4, w1
	add	w1, w1, 1
	tst	w3, w2
	cset	w3, ne
	cmp	w1, 16
	add	w0, w0, w3
	bne	.L2862
	cmp	w0, 6
	bhi	.L2863
	adrp	x0, .LANCHOR0+9
	strb	w1, [x0,#:lo12:.LANCHOR0+9]
	b	.L2864
.L2863:
	mov	w1, 0
	mov	w4, 1
.L2866:
	lsl	w3, w4, w1
	add	w1, w1, 1
	tst	w3, w2
	cset	w3, ne
	cmp	w1, 24
	add	w0, w0, w3
	bne	.L2866
	cmp	w0, 17
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	bls	.L2887
	mov	w1, 36
.L2887:
	strb	w1, [x0,9]
.L2864:
	adrp	x0, .LANCHOR0
	add	x0, x0, :lo12:.LANCHOR0
	ldrb	w1, [x0,9]
	strh	w1, [x0,194]
.L2860:
	adrp	x0, .LC73
	adrp	x1, .LC74
	add	x1, x1, :lo12:.LC74
	add	x0, x0, :lo12:.LC73
	adrp	x19, .LANCHOR0
	bl	printk
	add	x19, x19, :lo12:.LANCHOR0
	add	x0, x19, 168
	bl	FtlConstantsInit
	bl	FtlVariablesInit
	ldrh	w0, [x19,1956]
	mov	w19, 1
	bl	FtlFreeSysBlkQueueInit
.L2868:
	bl	FtlLoadBbt
	cbz	w0, .L2869
.L2889:
	bl	FtlLowFormat
	cmp	w19, 3
	bhi	.L2890
	add	w19, w19, 1
	b	.L2868
.L2890:
	mov	w1, -1
	b	.L2858
.L2869:
	bl	FtlSysBlkInit
	cbnz	w0, .L2889
	adrp	x1, .LANCHOR1+532
	mov	w2, 1
	str	w2, [x1,#:lo12:.LANCHOR1+532]
	mov	w1, w0
.L2858:
	mov	w0, w1
	ldr	x19, [sp,16]
	ldp	x29, x30, [sp], 48
	ret
	.size	FtlReInitForSDUpdata, .-FtlReInitForSDUpdata
	.align	2
	.global	flash_boot_enter_slc_mode
	.type	flash_boot_enter_slc_mode, %function
flash_boot_enter_slc_mode:
	adrp	x1, .LANCHOR0+1872
	uxtb	w0, w0
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w2, [x1,#:lo12:.LANCHOR0+1872]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	bne	.L2891
	bl	flash_enter_slc_mode
.L2891:
	ldp	x29, x30, [sp], 16
	ret
	.size	flash_boot_enter_slc_mode, .-flash_boot_enter_slc_mode
	.align	2
	.global	flash_boot_exit_slc_mode
	.type	flash_boot_exit_slc_mode, %function
flash_boot_exit_slc_mode:
	adrp	x1, .LANCHOR0+1872
	uxtb	w0, w0
	stp	x29, x30, [sp, -16]!
	add	x29, sp, 0
	ldr	w2, [x1,#:lo12:.LANCHOR0+1872]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	bne	.L2893
	bl	flash_exit_slc_mode
.L2893:
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
	adrp	x0, .LC122
	add	x0, x0, :lo12:.LC122
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
.L2896:
	cmp	w23, w26
	bcs	.L2902
	ldrb	w1, [x19,120]
	sub	w24, w6, w3
	add	w4, w23, w21
	lsr	w4, w4, 2
	uxth	w24, w24
	add	x0, x19, x4, uxth 1
	ldrh	w0, [x0,196]
	cbz	w1, .L2897
	ldr	w2, [x19,1872]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	uxth	w4, w4
	cmp	w2, w1
	csel	w0, w4, w0, eq
.L2897:
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
	b	.L2896
.L2902:
	adrp	x0, .LC123
	mov	w1, w22
	mov	w2, w26
	mov	w3, 0
	add	x0, x0, :lo12:.LC123
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
	adrp	x0, .LC124
	add	x0, x0, :lo12:.LC124
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
.L2904:
	cmp	w23, w27
	bcs	.L2918
	add	w4, w23, w26
	lsr	w4, w4, 2
	uxth	w0, w4
	and	w4, w4, 65535
	cbz	w0, .L2905
	add	x1, x19, 196
	add	w2, w4, 1
	ldrh	w3, [x1,w2,sxtw 1]
	ldrb	w1, [x19,120]
	cbz	w1, .L2906
	ldr	w2, [x19,1872]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	bne	.L2906
	add	w3, w0, 1
	uxth	w3, w3
.L2906:
	sub	w3, w3, #1
	str	wzr, [x29,132]
	lsl	w3, w3, 2
	str	w3, [x29,128]
.L2905:
	add	x1, x19, 196
	ldrh	w25, [x1,w4,sxtw 1]
	ldrb	w1, [x19,120]
	cbz	w1, .L2907
	ldr	w2, [x19,1872]
	mov	w1, 12336
	movk	w1, 0x5638, lsl 16
	cmp	w2, w1
	csel	w25, w25, w0, ne
.L2907:
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
	b	.L2904
.L2918:
	adrp	x0, .LC125
	mov	w1, w24
	mov	w2, w27
	mov	w3, 0
	add	x0, x0, :lo12:.LC125
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
	mov	w2, 6
	ldr	x1, [x0,744]
	ldr	w26, [x0,12]
	mov	x0, 59392
	movk	x0, 0x3, lsl 16
	ldrb	w22, [x1,9]
	mov	w1, 192
	movk	w1, 0x240, lsl 16
	bl	kmalloc_order_trace
	str	x24, [x29,120]
	mov	x20, x0
	mov	w0, -1
	cbz	x20, .L2929
	add	w19, w25, 511
	lsr	w19, w19, 9
	cmp	w19, 255
	bhi	.L2921
	ubfiz	x0, x19, 9, 23
	mov	w2, 256
	add	x0, x21, x0
	mov	x1, x21
	sub	w2, w2, w19
	bl	memcpy
.L2921:
	add	w19, w19, 128
	mov	w0, 256
	cmp	w19, 256
	mov	x1, x23
	csel	w19, w19, w0, ls
	adrp	x0, .LC126
	add	x0, x0, :lo12:.LC126
	mov	w2, 4
	mov	w3, 5
	bl	rknand_print_hex
	ldr	x0, [x29,120]
	ldr	w1, [x21,512]
	add	x27, x0, :lo12:.LANCHOR0
	adrp	x0, .LC127
	add	x0, x0, :lo12:.LC127
	ldrb	w2, [x27,9]
	bl	printk
	ldrb	w0, [x27,9]
	ldr	w1, [x21,512]
	cmp	w1, w0
	bls	.L2922
	str	w0, [x21,512]
.L2922:
	adrp	x0, .LC128
	mov	w2, w25
	mul	w22, w22, w26
	add	x0, x0, :lo12:.LC128
	mov	w1, w19
	mov	x24, 0
	adrp	x26, .LC129
	bl	printk
	adrp	x27, .LC130
	adrp	x28, .LC131
	lsl	w0, w19, 7
	uxth	w22, w22
	str	w0, [x29,116]
	mov	w25, w24
	add	x26, x26, :lo12:.LC129
	add	x27, x27, :lo12:.LC130
	add	x28, x28, :lo12:.LC131
.L2927:
	ldr	x1, [x29,120]
	mov	w8, w24
	ldr	w0, [x23,x24,lsl 2]
	add	x1, x1, :lo12:.LANCHOR0
	ldrh	w1, [x1,194]
	cmp	w0, w1
	bcs	.L2923
	adrp	x1, .LANCHOR4+68
	ldr	w1, [x1,#:lo12:.LANCHOR4+68]
	cmp	w0, w1
	bcc	.L2923
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
.L2924:
	ldr	w1, [x29,116]
	mov	w3, w0
	cmp	w0, w1
	bcs	.L2934
	ldr	w4, [x20,x0,lsl 2]
	mov	x7, x0
	add	x0, x0, 1
	add	x1, x21, x0, lsl 2
	ldr	w5, [x1,-4]
	cmp	w4, w5
	beq	.L2924
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
	adrp	x0, .LC132
	add	x0, x0, :lo12:.LC132
	bl	printk
	b	.L2923
.L2934:
	add	w25, w25, 1
.L2923:
	add	x24, x24, 1
	cmp	x24, 5
	bne	.L2927
	mov	x0, x20
	bl	kfree
	cmp	w25, wzr
	csetm	w0, eq
.L2929:
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
.L2936:
	cmp	w1, w2
	bls	.L2938
	ldrb	w4, [x5,x2]
	add	x6, x3, 3016
	add	x2, x2, 1
	eor	w4, w4, w0, lsr 24
	ldr	w4, [x6,w4,uxtw 2]
	eor	w0, w4, w0, lsl 8
	b	.L2936
.L2938:
	ret
	.size	CRC_32, .-CRC_32
	.align	2
	.type	rknand_sys_storage_ioctl, %function
rknand_sys_storage_ioctl:
	sub	sp, sp, #528
	stp	x29, x30, [sp, -48]!
	add	x29, sp, 0
	stp	x19, x20, [sp,16]
	str	x21, [sp,32]
	mov	w19, w1
	mov	x21, x2
	ldr	x20, [x0,208]
	mov	w0, 29187
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2941
	mov	w0, 29187
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L2942
	mov	w0, 25726
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2943
	mov	w0, 25726
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L2944
	mov	w0, 25601
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2945
	mov	w0, 25601
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L2946
	mov	w0, 25364
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bne	.L2995
	bl	rknand_dev_flush
	b	.L3017
.L2946:
	mov	w0, 25602
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2948
	mov	w0, 25603
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2949
	b	.L2995
.L2944:
	mov	w0, 27688
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2950
	mov	w0, 27688
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L2951
	mov	w0, 25727
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2952
	b	.L2995
.L2951:
	mov	w0, 27698
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2950
	mov	w0, 27708
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2950
	b	.L2995
.L2942:
	mov	w0, 29267
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2953
	mov	w0, 29267
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L2954
	mov	w0, 29189
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2955
	mov	w0, 29189
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bcc	.L2956
	mov	w0, 29210
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2957
	mov	w0, 29266
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2958
	b	.L2995
.L2954:
	mov	w0, 30225
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2959
	mov	w0, 30225
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	bhi	.L2960
	mov	w0, 29268
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2961
	mov	w0, 30224
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2962
	b	.L2995
.L2960:
	mov	w0, 30226
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2962
	mov	w0, 30227
	movk	w0, 0x4004, lsl 16
	cmp	w1, w0
	beq	.L2959
	b	.L2995
.L2956:
	adrp	x0, .LC133
	add	x0, x0, :lo12:.LC133
	bl	printk
	mov	x0, x20
	mov	x1, x21
	mov	x2, 512
	bl	rk_copy_from_user
	cbz	x0, .L2963
.L2967:
	adrp	x0, .LC134
	add	x0, x0, :lo12:.LC134
	b	.L3020
.L2963:
	adrp	x0, .LC135
	ldr	w1, [x20]
	ldr	w2, [x20,4]
	add	x0, x0, :lo12:.LC135
	bl	printk
	ldr	w19, [x20,4]
	cmp	w19, 8
	bhi	.L3019
	bl	rknand_device_unlock
	ldr	w0, [x20]
	mov	w1, w19
	mov	x2, x20
	bl	IdBlockReadData
	bl	rknand_device_unlock
	mov	x0, x21
	mov	x1, x20
	ubfiz	x2, x19, 9, 23
	bl	rk_copy_to_user
	cbz	x0, .L3017
	adrp	x0, .LC136
	add	x0, x0, :lo12:.LC136
	b	.L3020
.L2955:
	adrp	x0, .LC137
	add	x0, x0, :lo12:.LC137
	bl	printk
	mov	x0, x20
	mov	x1, x21
	mov	x2, 4096
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w1, [x20]
	adrp	x0, .LC138
	ldr	w2, [x20,4]
	add	x0, x0, :lo12:.LC138
	bl	printk
	adrp	x0, .LANCHOR4
	add	x21, x0, :lo12:.LANCHOR4
	mov	x19, x0
	ldr	x1, [x21,448]
	cbz	x1, .L2968
.L2971:
	ldr	w2, [x20,4]
	cmp	w2, 4088
	bls	.L2969
	b	.L3019
.L2968:
	mov	w1, 192
	mov	x0, 260096
	movk	w1, 0x240, lsl 16
	mov	w2, 6
	bl	kmalloc_order_trace
	str	x0, [x21,448]
	cbnz	x0, .L2971
	b	.L3019
.L2969:
	ldr	w0, [x20]
	mov	w1, 55296
	movk	w1, 0x3, lsl 16
	cmp	w0, w1
	bhi	.L3019
	add	x19, x19, :lo12:.LANCHOR4
	uxtw	x2, w2
	ldr	x1, [x19,448]
	add	x0, x1, x0, uxtw
	add	x1, x20, 8
	bl	memcpy
	b	.L3017
.L2958:
	adrp	x0, .LC139
	add	x0, x0, :lo12:.LC139
	bl	printk
	mov	x0, x20
	mov	x1, x21
	mov	x2, 28
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w1, [x20]
	adrp	x0, .LC140
	ldr	w2, [x20,4]
	add	x0, x0, :lo12:.LC140
	bl	printk
	ldr	w1, [x20]
	mov	w0, 59392
	movk	w0, 0x3, lsl 16
	cmp	w1, w0
	bhi	.L3019
	adrp	x19, .LANCHOR4
	add	x19, x19, :lo12:.LANCHOR4
	ldr	x0, [x19,448]
	cbz	x0, .L3019
	bl	CRC_32
	ldr	w1, [x20,4]
	cmp	w1, w0
	beq	.L2972
.L2978:
	mov	x0, -2
	b	.L2940
.L2972:
	bl	rknand_device_unlock
	ldr	x1, [x19,448]
	add	x2, x20, 8
	ldr	w0, [x20]
	bl	write_idblock
	bl	rknand_device_unlock
	ldr	x0, [x19,448]
	bl	kfree
	str	xzr, [x19,448]
	b	.L3017
.L2957:
	adrp	x0, .LC141
	add	x0, x0, :lo12:.LC141
	bl	printk
	mov	x0, x20
	bl	ReadFlashInfo
	mov	x0, x21
	mov	x1, x20
	mov	x2, 11
	b	.L3013
.L2941:
	adrp	x0, .LC142
	add	x0, x0, :lo12:.LC142
	bl	printk
	bl	rknand_device_unlock
	bl	FtlReInitForSDUpdata
	mov	w19, w0
	bl	rknand_device_unlock
	cbnz	w19, .L3019
	bl	nand_blk_add_whole_disk
	bl	rknand_device_unlock
	mov	w1, w19
	mov	w2, 64
	mov	x0, x20
	bl	FlashReadFacBbtData
	bl	rknand_device_unlock
	adrp	x0, .LC143
	mov	x1, x20
	add	x0, x0, :lo12:.LC143
	mov	w2, 4
	mov	w3, 8
	bl	rknand_print_hex
	mov	x0, x21
	mov	x1, x20
	mov	x2, 64
	b	.L3013
.L2953:
	adrp	x0, .LC144
	add	x0, x0, :lo12:.LC144
	bl	printk
	adrp	x0, .LANCHOR4+456
	mov	x1, x20
	mov	x2, 4
	ldr	x0, [x0,#:lo12:.LANCHOR4+456]
	ldr	w0, [x0,20]
	str	w0, [x20]
	mov	x0, x21
	b	.L3013
.L2961:
	adrp	x0, .LC145
	add	x0, x0, :lo12:.LC145
	bl	printk
	bl	rknand_device_unlock
	mov	w1, 2
	mov	x2, x20
	mov	w0, 520
	bl	FtlVendorPartRead
	bl	rknand_device_unlock
	mov	x0, x21
	mov	x1, x20
	mov	x2, 1024
	b	.L3013
.L2945:
	adrp	x0, .LC146
	add	x0, x0, :lo12:.LC146
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	mov	x19, x0
	cbnz	x0, .L2967
	ldr	w1, [x29,56]
	mov	w0, 21060
	movk	w0, 0x4b4d, lsl 16
	cmp	w1, w0
	beq	.L2974
.L2975:
	mov	x19, -1
	b	.L2966
.L2974:
	ldr	w0, [x29,60]
	cmp	w0, 512
	bhi	.L2975
	adrp	x20, .LANCHOR4
	add	x0, x29, 56
	add	x20, x20, :lo12:.LANCHOR4
	mov	x2, 512
	ldr	x1, [x20,456]
	bl	memcpy
	ldr	w1, [x20,464]
	mov	w0, 5161
	movk	w0, 0xc059, lsl 16
	cmp	w1, w0
	beq	.L2976
	add	x0, x29, 120
	mov	w1, w19
	mov	x2, 128
	str	w19, [x29,64]
	str	w19, [x29,68]
	bl	memset
.L2976:
	add	x0, x29, 312
	mov	w1, 0
	mov	x2, 256
	str	wzr, [x29,72]
	bl	memset
	b	.L2993
.L2948:
	adrp	x0, .LC147
	add	x0, x0, :lo12:.LC147
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	cbz	x0, .L2977
	adrp	x0, .LC148
	add	x0, x0, :lo12:.LC148
.L3020:
	bl	printk
	b	.L3019
.L2977:
	ldr	w1, [x29,56]
	mov	w0, 21060
	movk	w0, 0x4b4d, lsl 16
	cmp	w1, w0
	bne	.L2975
	ldr	w0, [x29,60]
	cmp	w0, 512
	bhi	.L2975
	adrp	x1, .LANCHOR4
	mov	w0, 5161
	add	x1, x1, :lo12:.LANCHOR4
	movk	w0, 0xc059, lsl 16
	ldr	w2, [x1,464]
	cmp	w2, w0
	bne	.L2978
	ldr	w2, [x29,68]
	mov	x0, -3
	sub	w3, w2, #1
	cmp	w3, 127
	bhi	.L2940
	ldr	x19, [x1,456]
	add	x1, x29, 120
	add	x0, x19, 64
	str	w2, [x19,12]
	ldr	w2, [x29,68]
	bl	memcpy
	mov	w0, 1
	mov	x1, x19
	b	.L3018
.L2952:
	adrp	x0, .LC149
	add	x0, x0, :lo12:.LC149
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w1, [x29,56]
	mov	w0, 17476
	movk	w0, 0x4253, lsl 16
	cmp	w1, w0
	bne	.L2975
	ldr	w0, [x29,60]
	cmp	w0, 512
	bhi	.L2975
	adrp	x19, .LANCHOR4
	add	x0, x19, :lo12:.LANCHOR4
	ldr	w1, [x0,468]
	cbnz	w1, .L2979
.L2982:
	mov	x0, 0
	b	.L2940
.L2979:
	ldr	x1, [x0,472]
	mov	w2, 22867
	movk	w2, 0x4453, lsl 16
	ldr	w3, [x1]
	cmp	w3, w2
	beq	.L2980
	mov	w2, 22867
	movk	w2, 0x4453, lsl 16
	str	w2, [x1]
	mov	w2, 504
	ldr	x1, [x0,472]
	str	w2, [x1,4]
	ldr	x0, [x0,472]
	str	wzr, [x0,8]
	str	wzr, [x0,12]
.L2980:
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
	beq	.L2981
	mov	w1, 21060
	movk	w1, 0x4b4d, lsl 16
	str	w1, [x0]
	mov	w1, 504
	ldr	x0, [x20,456]
	str	w1, [x0,4]
	ldr	x0, [x20,456]
	str	wzr, [x0,8]
.L2981:
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
	b	.L3017
.L2943:
	adrp	x0, .LC150
	add	x0, x0, :lo12:.LC150
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w2, [x29,56]
	mov	w1, 20037
	movk	w1, 0x4253, lsl 16
	cmp	w2, w1
	bne	.L2975
	ldr	w1, [x29,60]
	cmp	w1, 512
	bhi	.L2975
	adrp	x19, .LANCHOR4
	add	x1, x19, :lo12:.LANCHOR4
	ldr	w2, [x1,468]
	cmp	w2, 1
	beq	.L2982
	ldr	x2, [x1,472]
	mov	w3, 22867
	movk	w3, 0x4453, lsl 16
	ldr	w4, [x2]
	cmp	w4, w3
	beq	.L2983
	mov	w3, 22867
	movk	w3, 0x4453, lsl 16
	str	w3, [x2]
	mov	w3, 504
	ldr	x2, [x1,472]
	str	w3, [x2,4]
	ldr	x1, [x1,472]
	str	w0, [x1,8]
	str	w0, [x1,12]
.L2983:
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
	beq	.L2984
	mov	w1, 21060
	movk	w1, 0x4b4d, lsl 16
	str	w1, [x0]
	mov	w1, 504
	ldr	x0, [x20,456]
	str	w1, [x0,4]
	ldr	x0, [x20,456]
	str	wzr, [x0,8]
.L2984:
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
	b	.L3017
.L2949:
	adrp	x0, .LC151
	add	x0, x0, :lo12:.LC151
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w1, [x29,56]
	mov	w0, 20051
	movk	w0, 0x4144, lsl 16
	cmp	w1, w0
	bne	.L2975
	ldr	w2, [x29,60]
	cmp	w2, 512
	bhi	.L2975
	adrp	x1, .LANCHOR4
	add	x0, x29, 64
	add	x1, x1, :lo12:.LANCHOR4
	uxtw	x2, w2
	add	x1, x1, 480
	b	.L3014
.L2950:
	mov	w0, 27698
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	bne	.L2985
	adrp	x0, .LC152
	add	x0, x0, :lo12:.LC152
	b	.L3015
.L2985:
	mov	w0, 27708
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	bne	.L2987
	adrp	x0, .LC153
	add	x0, x0, :lo12:.LC153
	b	.L3015
.L2987:
	adrp	x0, .LC154
	add	x0, x0, :lo12:.LC154
.L3015:
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w1, [x29,56]
	mov	w0, 17227
	movk	w0, 0x4c4f, lsl 16
	cmp	w1, w0
	bne	.L3019
	mov	w0, 27708
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	adrp	x0, .LANCHOR4
	bne	.L2988
	add	x0, x0, :lo12:.LANCHOR4
	add	x1, x29, 56
	mov	x2, 16
	ldr	x0, [x0,456]
	ldr	w0, [x0,20]
	str	w0, [x29,60]
	strb	w0, [x29,64]
	mov	x0, x21
	bl	rk_copy_to_user
	cbz	x0, .L2940
	b	.L3019
.L2988:
	add	x20, x0, :lo12:.LANCHOR4
	ldr	w1, [x20,992]
	cmp	w1, 10
	bhi	.L3019
	ldr	x1, [x20,456]
	ldr	w2, [x29,60]
	ldr	w3, [x1,24]
	cmp	w3, w2
	beq	.L2989
	cbz	w3, .L2989
	adrp	x0, .LC155
	mov	w1, w2
	add	x0, x0, :lo12:.LC155
	bl	printk
	ldr	w0, [x20,992]
	add	w0, w0, 1
	str	w0, [x20,992]
.L3019:
	mov	x0, -14
	b	.L2940
.L2989:
	add	x0, x0, :lo12:.LANCHOR4
	str	wzr, [x0,992]
	mov	w0, 27698
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	bne	.L2990
	str	wzr, [x1,20]
	str	wzr, [x1,24]
	b	.L2991
.L2990:
	mov	w0, 1
	str	w2, [x1,24]
	str	w0, [x1,20]
.L2991:
	mov	w0, 1
	mov	x19, -2
	bl	StorageSysDataStore
	cmn	w0, #1
	bne	.L3017
	b	.L2966
.L2962:
	adrp	x0, .LC156
	add	x0, x0, :lo12:.LC156
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w1, [x29,56]
	mov	w0, 17750
	movk	w0, 0x444e, lsl 16
	cmp	w1, w0
	bne	.L2975
	ldr	w2, [x29,60]
	cmp	w2, 504
	bhi	.L2975
	mov	w0, 30224
	adrp	x1, .LANCHOR4
	movk	w0, 0x4004, lsl 16
	uxtw	x2, w2
	cmp	w19, w0
	add	x1, x1, :lo12:.LANCHOR4
	add	x0, x29, 64
	bne	.L2992
	ldr	x1, [x1,1000]
	b	.L3016
.L2992:
	ldr	x1, [x1,1008]
.L3016:
	add	x1, x1, 8
.L3014:
	bl	memcpy
.L2993:
	mov	x0, x21
	add	x1, x29, 56
	mov	x2, 520
.L3013:
	bl	rk_copy_to_user
	cbnz	x0, .L3019
	b	.L3017
.L2959:
	adrp	x0, .LC157
	add	x0, x0, :lo12:.LC157
	bl	printk
	add	x0, x29, 56
	mov	x1, x21
	mov	x2, 520
	bl	rk_copy_from_user
	cbnz	x0, .L2967
	ldr	w1, [x29,56]
	mov	w0, 17750
	movk	w0, 0x444e, lsl 16
	cmp	w1, w0
	bne	.L2975
	ldr	w2, [x29,60]
	cmp	w2, 504
	bhi	.L2975
	mov	w0, 30225
	add	w2, w2, 8
	movk	w0, 0x4004, lsl 16
	cmp	w19, w0
	adrp	x19, .LANCHOR4
	add	x19, x19, :lo12:.LANCHOR4
	bne	.L2994
	ldr	x0, [x19,1000]
	add	x1, x29, 56
	bl	memcpy
	mov	w0, 2
	ldr	x1, [x19,1000]
	b	.L3018
.L2994:
	ldr	x0, [x19,1008]
	add	x1, x29, 56
	bl	memcpy
	ldr	x1, [x19,1008]
	mov	w0, 3
.L3018:
	bl	StorageSysDataStore
	uxtw	x19, w0
	b	.L2966
.L3017:
	mov	x19, 0
.L2966:
	adrp	x0, .LC158
	mov	x1, x19
	add	x0, x0, :lo12:.LC158
	bl	printk
	mov	x0, x19
	b	.L2940
.L2995:
	mov	x0, -22
.L2940:
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
	cbz	w22, .L3022
	mov	x0, x21
	mov	w1, 508
	bl	JSHash
	cmp	w22, w0
	beq	.L3022
	adrp	x0, .LC159
	str	wzr, [x21,16]
	add	x0, x0, :lo12:.LC159
	str	wzr, [x19,468]
	bl	printk
.L3022:
	add	x0, x20, :lo12:.LANCHOR4
	ldr	w1, [x0,468]
	cbz	w1, .L3023
	mov	w1, 5161
	movk	w1, 0xc059, lsl 16
	str	w1, [x0,464]
.L3023:
	add	x20, x20, :lo12:.LANCHOR4
	mov	w0, 2
	ldr	x1, [x20,1000]
	bl	StorageSysDataLoad
	ldr	x1, [x20,1008]
	mov	w0, 3
	bl	StorageSysDataLoad
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	add	x0, x0, 4040
	bl	misc_register
	ldp	x19, x20, [sp,16]
	ldp	x21, x22, [sp,32]
	ldp	x29, x30, [sp], 48
	ret
	.size	rk_ftl_storage_sys_init, .-rk_ftl_storage_sys_init
	.align	2
	.global	StorageSysDataDeInit
	.type	StorageSysDataDeInit, %function
StorageSysDataDeInit:
	stp	x29, x30, [sp, -16]!
	adrp	x0, .LANCHOR1
	add	x0, x0, :lo12:.LANCHOR1
	add	x29, sp, 0
	add	x0, x0, 4040
	bl	misc_deregister
	mov	w0, 0
	ldp	x29, x30, [sp], 16
	ret
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
	.type	__func__.19115, %object
	.size	__func__.19115, 11
__func__.19115:
	.string	"FtlMemInit"
	.zero	5
	.type	__func__.19991, %object
	.size	__func__.19991, 21
__func__.19991:
	.string	"FtlVpcCheckAndModify"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.string	"Context allocation failed\n"
.LC1:
	.string	"FlashEraseBlocks pageAddr error %x \n"
.LC2:
	.string	"phyBlk = 0x%x die = %d block_in_die = 0x%x 0x%8x\n"
.LC3:
	.string	"FLASH INFO:\n"
.LC4:
	.string	"FLASH ID: %x\n"
.LC5:
	.string	"Device Capacity: %d MB\n"
.LC6:
	.string	"FMWAIT: %x %x %x %x\n"
.LC7:
	.string	"FTL INFO:\n"
.LC8:
	.string	"g_MaxLpn = 0x%x\n"
.LC9:
	.string	"g_VaildLpn = 0x%x\n"
.LC10:
	.string	"read_page_count = 0x%x\n"
.LC11:
	.string	"discard_page_count = 0x%x\n"
.LC12:
	.string	"write_page_count = 0x%x\n"
.LC13:
	.string	"cache_write_count = 0x%x\n"
.LC14:
	.string	"l2p_write_count = 0x%x\n"
.LC15:
	.string	"gc_page_count = 0x%x\n"
.LC16:
	.string	"totle_write = %d MB\n"
.LC17:
	.string	"totle_read = %d MB\n"
.LC18:
	.string	"GSV = 0x%x\n"
.LC19:
	.string	"GDV = 0x%x\n"
.LC20:
	.string	"bad blk num = %d %d\n"
.LC21:
	.string	"free_superblocks = 0x%x\n"
.LC22:
	.string	"mlc_EC = 0x%x\n"
.LC23:
	.string	"slc_EC = 0x%x\n"
.LC24:
	.string	"avg_EC = 0x%x\n"
.LC25:
	.string	"sys_EC = 0x%x\n"
.LC26:
	.string	"max_EC = 0x%x\n"
.LC27:
	.string	"min_EC = 0x%x\n"
.LC28:
	.string	"PLT = 0x%x\n"
.LC29:
	.string	"POT = 0x%x\n"
.LC30:
	.string	"MaxSector = 0x%x\n"
.LC31:
	.string	"init_sys_blks_pp = 0x%x\n"
.LC32:
	.string	"sys_blks_pp = 0x%x\n"
.LC33:
	.string	"free sysblock = 0x%x\n"
.LC34:
	.string	"data_blks_pp = 0x%x\n"
.LC35:
	.string	"data_op_blks_pp = 0x%x\n"
.LC36:
	.string	"max_data_blks = 0x%x\n"
.LC37:
	.string	"Sys.id = 0x%x\n"
.LC38:
	.string	"Bbt.id = 0x%x\n"
.LC39:
	.string	"ACT.page = 0x%x\n"
.LC40:
	.string	"ACT.plane = 0x%x\n"
.LC41:
	.string	"ACT.id = 0x%x\n"
.LC42:
	.string	"ACT.mode = 0x%x\n"
.LC43:
	.string	"ACT.a_pages = 0x%x\n"
.LC44:
	.string	"ACT VPC = 0x%x\n"
.LC45:
	.string	"BUF.page = 0x%x\n"
.LC46:
	.string	"BUF.plane = 0x%x\n"
.LC47:
	.string	"BUF.id = 0x%x\n"
.LC48:
	.string	"BUF.mode = 0x%x\n"
.LC49:
	.string	"BUF.a_pages = 0x%x\n"
.LC50:
	.string	"BUF VPC = 0x%x\n"
.LC51:
	.string	"TMP.page = 0x%x\n"
.LC52:
	.string	"TMP.plane = 0x%x\n"
.LC53:
	.string	"TMP.id = 0x%x\n"
.LC54:
	.string	"TMP.mode = 0x%x\n"
.LC55:
	.string	"TMP.a_pages = 0x%x\n"
.LC56:
	.string	"GC.page = 0x%x\n"
.LC57:
	.string	"GC.plane = 0x%x\n"
.LC58:
	.string	"GC.id = 0x%x\n"
.LC59:
	.string	"GC.mode = 0x%x\n"
.LC60:
	.string	"GC.a_pages = 0x%x\n"
.LC61:
	.string	"WR_CHK = 0x%x %x %x %x\n"
.LC62:
	.string	"Read Err Cnt = 0x%x\n"
.LC63:
	.string	"Prog Err Cnt = 0x%x\n"
.LC64:
	.string	"gc_free_blk_th= 0x%x\n"
.LC65:
	.string	"gc_merge_free_blk_th= 0x%x\n"
.LC66:
	.string	"gc_skip_write_count= 0x%x\n"
.LC67:
	.string	"gc_blk_index= 0x%x\n"
.LC68:
	.string	"free min EC= 0x%x\n"
.LC69:
	.string	"free max EC= 0x%x\n"
.LC70:
	.string	"GC__SB VPC = 0x%x\n"
.LC71:
	.string	"%d. [0x%x]=0x%x 0x%x  0x%x\n"
.LC72:
	.string	"free %d. [0x%x] 0x%x  0x%x\n"
.LC73:
	.string	"%s\n"
.LC74:
	.string	"FTL version: 5.0.36 20170316"
.LC75:
	.string	"GetSwlReplaceBlock min_ec_id =%x %x\n"
.LC76:
	.string	"swblk %x ,avg = %x max= %x vpc= %x,ec=%x ,max ec=%x\n"
.LC77:
	.string	"FtlGcRefreshBlock  0x%x \n"
.LC78:
	.string	"FtlGcMarkBadPhyBlk %d 0x%x\n"
.LC79:
	.string	"%s error allocating memory. return -1\n"
.LC80:
	.string	"%s 0x%x:"
.LC81:
	.string	"%x "
.LC82:
	.string	""
.LC83:
	.string	"%d statReg->V6.mtrans_cnt=%d flReg.V6.page_num=%d\n"
.LC84:
	.string	"nandc:"
.LC85:
	.string	"%d flReg.d32=%x %x\n"
.LC86:
	.string	"sdr read ok %x ecc=%d\n"
.LC87:
	.string	"sync para %d\n"
.LC88:
	.string	"TOG mode Read error %x %x\n"
.LC89:
	.string	"read retry status %x %x %x\n"
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
	.string	"ReadRetry pageadd=%x  ecc=%x err=%x\n"
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
	.string	"FlashMakeFactorBbt %d\n"
.LC105:
	.string	"bad block:%d %d\n"
.LC106:
	.string	"FMFB:%d %d\n"
.LC107:
	.string	"E:bad block:%d\n"
.LC108:
	.string	"FMFB:Save %d %d\n"
.LC109:
	.string	"FtlBbmTblFlush id=%x,page=%x,previd=%x cnt=%d\n"
.LC110:
	.string	"FtlBbmTblFlush error:%x\n"
.LC111:
	.string	"FtlGcFreeBadSuperBlk 0x%x\n"
.LC112:
	.string	"decrement_vpc_count %x = %d\n"
.LC113:
	.string	"spuer block %x vpn is 0\n "
.LC114:
	.string	"...%s enter...\n"
.LC115:
	.string	"FtlCheckVpc %x = %x  %x\n"
.LC116:
	.string	"%d GC datablk  = %x vpc %x %x\n"
.LC117:
	.string	"SWL %x, FSB = %x vpc= %x,ec=%x th=%x\n"
.LC118:
	.string	"Ftlwrite decrement_vpc_count %x = %d\n"
.LC119:
	.string	"FtlInit %x\n"
.LC120:
	.string	"FtlWrite: lpa error:%x %x\n"
.LC121:
	.string	"BBT:"
.LC122:
	.string	"IdBlockReadData %x %x\n"
.LC123:
	.string	"IdBlockReadData %x %x ret= %x\n"
.LC124:
	.string	"IDBlockWriteData %x %x\n"
.LC125:
	.string	"IDBlockWriteData %x %x ret= %x\n"
.LC126:
	.string	"idblk:"
.LC127:
	.string	"idb reverse %x %x\n"
.LC128:
	.string	"write_idblock totle_sec %x %x\n"
.LC129:
	.string	"write and check error:%d idb=%x,offset=%x,r=%x,w=%x \n"
.LC130:
	.string	"write"
.LC131:
	.string	"read"
.LC132:
	.string	"write_idblock error\n"
.LC133:
	.string	"READ_SECTOR_IO\n"
.LC134:
	.string	"rk_copy_from_user error \n"
.LC135:
	.string	"READ_SECTOR_IO %x %x\n"
.LC136:
	.string	"rk_copy_to_user error\n"
.LC137:
	.string	"WRITE_SECTOR_IO\n"
.LC138:
	.string	"WRITE_SECTOR_IO %x %x\n"
.LC139:
	.string	"END_WRITE_SECTOR_IO\n"
.LC140:
	.string	"END_WRITE_SECTOR_IO %x %x\n"
.LC141:
	.string	"GET_FLASH_INFO_IO\n"
.LC142:
	.string	"GET_BAD_BLOCK_IO\n"
.LC143:
	.string	"bbt:"
.LC144:
	.string	"GET_LOCK_FLAG_IO\n"
.LC145:
	.string	"GET_PUBLIC_KEY_IO\n"
.LC146:
	.string	"RKNAND_GET_DRM_KEY\n"
.LC147:
	.string	"RKNAND_STORE_DRM_KEY\n"
.LC148:
	.string	"copy_from_user error \n"
.LC149:
	.string	"RKNAND_DIASBLE_SECURE_BOOT\n"
.LC150:
	.string	"RKNAND_ENASBLE_SECURE_BOOT\n"
.LC151:
	.string	"RKNAND_GET_SN_SECTOR\n"
.LC152:
	.string	"RKNAND_LOADER_UNLOCK\n"
.LC153:
	.string	"RKNAND_LOADER_STATUS\n"
.LC154:
	.string	"RKNAND_LOADER_LOCK\n"
.LC155:
	.string	"LockKey not match %d\n"
.LC156:
	.string	"RKNAND_GET_VENDOR_SECTOR\n"
.LC157:
	.string	"RKNAND_STORE_VENDOR_SECTOR\n"
.LC158:
	.string	"return ret = %lx\n"
.LC159:
	.string	"secureBootEn check error\n"
.LC160:
	.string	"rknand_sys_storage"
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
	.hword	1088
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
	.type	rknand_sys_storage_dev, %object
	.size	rknand_sys_storage_dev, 80
rknand_sys_storage_dev:
	.word	255
	.zero	4
	.xword	.LC160
	.xword	rknand_sys_storage_fops
	.zero	56
	.type	rknand_sys_storage_fops, %object
	.size	rknand_sys_storage_fops, 216
rknand_sys_storage_fops:
	.zero	64
	.xword	rknand_sys_storage_ioctl
	.xword	rknand_sys_storage_ioctl
	.zero	8
	.xword	rknand_sys_storage_open
	.zero	8
	.xword	rknand_sys_storage_release
	.zero	104
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
	.zero	3
	.type	gNandcVer, %object
	.size	gNandcVer, 4
gNandcVer:
	.zero	4
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
