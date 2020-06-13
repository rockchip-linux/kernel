/*
 * Copyright (c) 2016-2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-09-18
 * function: rk ftl v6 for rockchip soc base on arm v7 to support 3D/2D
 *	     TLC and MLC.
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
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"rk_zftl_arm_v7.S"
#APP
	.syntax unified
	.global	__aeabi_uidiv
	.text
	.align	2
	.type	slc_phy_page_address_calc, %function
slc_phy_page_address_calc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r6, .L14
	ldrb	r3, [r6]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3
.L2:
	ldrh	r5, [r6, #2]
	mov	r0, r4
	mov	r1, r5
	bl	__aeabi_uidiv
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L14
	mul	r0, r5, r0
	rsb	r4, r0, r4
	mov	r4, r4, asl #1
	addeq	r4, r3, r4
	ldreqh	r4, [r4, #4]
	add	r4, r4, r0
.L3:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L15:
	.align	2
.L14:
	.word	.LANCHOR0
	.fnend
	.size	slc_phy_page_address_calc, .-slc_phy_page_address_calc
	.align	2
	.global	zftl_nandc_get_irq_status
	.type	zftl_nandc_get_irq_status, %function
zftl_nandc_get_irq_status:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L19
	ldrb	r3, [r3, #516]	@ zero_extendqisi2
	cmp	r3, #9
	ldreq	r0, [r0, #296]
	ldrne	r0, [r0, #372]
	bx	lr
.L20:
	.align	2
.L19:
	.word	.LANCHOR0
	.fnend
	.size	zftl_nandc_get_irq_status, .-zftl_nandc_get_irq_status
	.align	2
	.global	zftl_get_density
	.type	zftl_get_density, %function
zftl_get_density:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r0, #0
	bne	.L22
	ldr	r3, .L25
	ldr	r0, [r3, #520]
	bx	lr
.L22:
	cmp	r0, #4
	movcc	r0, #8192
	movcs	r0, #0
	bx	lr
.L26:
	.align	2
.L25:
	.word	.LANCHOR0
	.fnend
	.size	zftl_get_density, .-zftl_get_density
	.align	2
	.type	_list_get_gc_head_node, %function
_list_get_gc_head_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r0, [r0]
	cmp	r0, #0
	beq	.L32
	ldr	r3, .L35
	mov	ip, #6
	ldr	r2, [r3, #524]
	movw	r3, #65535
.L29:
	cmp	r1, #0
	beq	.L30
	ldrh	r0, [r0]
	cmp	r0, r3
	subne	r1, r1, #1
	mlane	r0, ip, r0, r2
	uxthne	r1, r1
	bne	.L29
.L34:
	bx	lr
.L32:
	movw	r0, #65535
	bx	lr
.L30:
	rsb	r0, r2, r0
	ldr	r3, .L35+4
	mov	r0, r0, asr #1
	mul	r0, r3, r0
	uxth	r0, r0
	bx	lr
.L36:
	.align	2
.L35:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	_list_get_gc_head_node, .-_list_get_gc_head_node
	.align	2
	.type	_list_remove_node, %function
_list_remove_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, #6
	mul	r5, r5, r1
	ldr	r6, .L46
	mov	r7, r0
	movw	r3, #65535
	ldr	r8, [r6, #524]
	add	r4, r8, r5
	ldrh	r0, [r8, r5]
	ldrh	r1, [r4, #2]
	cmp	r0, r3
	ldr	r3, [r7]
	bne	.L38
	cmp	r1, r0
	bne	.L38
	cmp	r4, r3
	ldmnefd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L38:
	mov	r9, r2
	movw	r2, #65535
	cmp	r1, r2
	bne	.L40
	cmp	r4, r3
	beq	.L40
	ldr	r1, .L46+4
	mov	r2, #188
	ldr	r0, .L46+8
	bl	printk
	bl	dump_stack
.L40:
	ldr	r3, [r7]
	movw	r2, #65535
	cmp	r4, r3
	ldrh	r3, [r8, r5]
	bne	.L41
	cmp	r3, r2
	ldrne	r2, [r6, #524]
	movne	r1, #6
	moveq	r3, #0
	streq	r3, [r7]
	mlane	r3, r1, r3, r2
	mvnne	r2, #0
	strne	r3, [r7]
	strneh	r2, [r3, #2]	@ movhi
	b	.L43
.L41:
	cmp	r3, r2
	ldrh	r1, [r4, #2]
	bne	.L44
	cmp	r1, r3
	movne	r3, #6
	ldrne	r2, [r6, #524]
	mulne	r1, r3, r1
	mvnne	r3, #0
	strneh	r3, [r2, r1]	@ movhi
	b	.L43
.L44:
	ldr	r0, [r6, #524]
	mov	r2, #6
	mla	r3, r2, r3, r0
	strh	r1, [r3, #2]	@ movhi
	ldrh	r0, [r4, #2]
	ldrh	r1, [r8, r5]
	ldr	r3, [r6, #524]
	mul	r2, r2, r0
	strh	r1, [r3, r2]	@ movhi
.L43:
	mvn	r3, #0
	strh	r3, [r8, r5]	@ movhi
	strh	r3, [r4, #2]	@ movhi
	ldrh	r3, [r9]
	sub	r3, r3, #1
	strh	r3, [r9]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L47:
	.align	2
.L46:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC0
	.fnend
	.size	_list_remove_node, .-_list_remove_node
	.align	2
	.type	_list_pop_index_node, %function
_list_pop_index_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r5, [r0]
	cmp	r5, #0
	beq	.L54
	ldr	r3, .L56
	movw	ip, #65535
	mov	lr, #6
	ldr	r4, [r3, #524]
.L50:
	cmp	r1, #0
	bne	.L51
.L53:
	rsb	r4, r4, r5
	ldr	r3, .L56+4
	mov	r4, r4, asr #1
	mul	r4, r3, r4
	uxth	r4, r4
	mov	r1, r4
	bl	_list_remove_node
	mvn	r3, #0
	mov	r0, r4
	strh	r3, [r5]	@ movhi
	strh	r3, [r5, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, pc}
.L51:
	ldrh	r3, [r5]
	cmp	r3, ip
	beq	.L53
	sub	r1, r1, #1
	mla	r5, lr, r3, r4
	uxth	r1, r1
	b	.L50
.L54:
	movw	r0, #65535
	ldmfd	sp!, {r3, r4, r5, pc}
.L57:
	.align	2
.L56:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	_list_pop_index_node, .-_list_pop_index_node
	.align	2
	.type	zftl_debug_proc_open, %function
zftl_debug_proc_open:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r1
	bl	PDE_DATA
	ldr	r1, .L60
	mov	r2, r0
	mov	r0, r4
	ldmfd	sp!, {r4, lr}
	b	single_open
.L61:
	.align	2
.L60:
	.word	zftl_debug_proc_show
	.fnend
	.size	zftl_debug_proc_open, .-zftl_debug_proc_open
	.align	2
	.type	zftl_debug_proc_show, %function
zftl_debug_proc_show:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	ldr	r1, .L64
	ldr	r2, .L64+4
	bl	seq_printf
	ldr	r3, .L64+8
	mov	r0, r4
	ldr	r1, .L64+12
	ldr	r2, [r3]
	bl	seq_printf
	mov	r0, #0
	ldmfd	sp!, {r4, pc}
.L65:
	.align	2
.L64:
	.word	.LC1
	.word	.LC2
	.word	.LANCHOR2
	.word	.LC3
	.fnend
	.size	zftl_debug_proc_show, .-zftl_debug_proc_show
	.align	2
	.global	zftl_flash_suspend
	.type	zftl_flash_suspend, %function
zftl_flash_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L70
	ldrb	r2, [r3, #516]	@ zero_extendqisi2
	cmp	r2, #9
	ldr	r2, [r3, #528]
	ldr	r1, [r2]
	str	r1, [r3, #532]
	ldr	r1, [r2, #4]
	str	r1, [r3, #536]
	bne	.L67
	ldr	r1, [r2, #16]
	str	r1, [r3, #540]
	ldr	r1, [r2, #32]
	str	r1, [r3, #544]
	ldr	r1, [r2, #80]
	str	r1, [r3, #548]
	ldr	r1, [r2, #84]
	str	r1, [r3, #552]
	ldr	r1, [r2, #520]
	ldr	r2, [r2, #8]
	str	r1, [r3, #556]
	b	.L69
.L67:
	ldr	r1, [r2, #8]
	str	r1, [r3, #540]
	ldr	r1, [r2, #12]
	str	r1, [r3, #544]
	ldr	r1, [r2, #304]
	str	r1, [r3, #548]
	ldr	r1, [r2, #308]
	str	r1, [r3, #552]
	ldr	r1, [r2, #336]
	ldr	r2, [r2, #344]
	str	r1, [r3, #556]
.L69:
	str	r2, [r3, #560]
	bx	lr
.L71:
	.align	2
.L70:
	.word	.LANCHOR0
	.fnend
	.size	zftl_flash_suspend, .-zftl_flash_suspend
	.align	2
	.type	_insert_free_list, %function
_insert_free_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L82
	add	ip, r3, #564
	ldrh	ip, [ip]
	cmp	ip, r1
	bxls	lr
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r3
	ldrh	ip, [r2]
	add	ip, ip, #1
	strh	ip, [r2]	@ movhi
	mov	ip, #6
	ldr	r7, [r3, #524]
	mul	r6, ip, r1
	mvn	r3, #0
	add	lr, r7, r6
	strh	r3, [lr, #2]	@ movhi
	strh	r3, [r7, r6]	@ movhi
	ldr	r3, [r0]
	cmp	r3, #0
	beq	.L81
	ldr	r5, [r4, #568]
	add	r8, r4, #572
	mov	r2, r1, asl #2
	ldr	r9, [r4, #524]
	ldrh	r10, [r8]
	movw	fp, #65535
	ldr	r8, [r5, r1, asl #2]
	ldrh	r2, [r5, r2]
	ubfx	r8, r8, #11, #8
	str	ip, [sp, #4]
	ubfx	r2, r2, #0, #11
	smulbb	r8, r8, r10
	add	r2, r8, r2
	rsb	r8, r9, r3
	uxth	r2, r2
	str	r2, [sp]
	ldr	r2, .L82+4
	mov	r8, r8, asr #1
	mul	r2, r2, r8
	uxth	r2, r2
.L77:
	ldr	ip, [r5, r2, asl #2]
	mov	r8, r2, asl #2
	ubfx	ip, ip, #11, #8
	ldrh	r8, [r5, r8]
	smulbb	ip, ip, r10
	ubfx	r8, r8, #0, #11
	add	r8, ip, r8
	ldr	ip, [sp]
	uxth	r8, r8
	cmp	r8, ip
	bcs	.L75
	ldrh	ip, [r3]
	cmp	ip, fp
	streqh	r2, [lr, #2]	@ movhi
	streqh	r1, [r3]	@ movhi
	beq	.L72
.L76:
	ldr	r3, [sp, #4]
	mov	r2, ip
	mla	r3, r3, ip, r9
	b	.L77
.L75:
	ldrh	ip, [r3, #2]
	strh	ip, [lr, #2]	@ movhi
	strh	r2, [r7, r6]	@ movhi
	ldr	r2, [r0]
	cmp	r3, r2
	bne	.L78
	strh	r1, [r3, #2]	@ movhi
.L81:
	str	lr, [r0]
	b	.L72
.L78:
	ldrh	ip, [r3, #2]
	mov	r2, #6
	ldr	r0, [r4, #524]
	mul	r2, r2, ip
	strh	r1, [r0, r2]	@ movhi
	strh	r1, [r3, #2]	@ movhi
.L72:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L83:
	.align	2
.L82:
	.word	.LANCHOR0
	.word	-1431655765
	.fnend
	.size	_insert_free_list, .-_insert_free_list
	.align	2
	.type	_insert_data_list, %function
_insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L105
	add	ip, r3, #564
	ldrh	ip, [ip]
	cmp	ip, r1
	bxls	lr
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, #6
	ldrh	ip, [r2]
	mov	r4, r3
	mul	r5, r5, r1
	add	ip, ip, #1
	strh	ip, [r2]	@ movhi
	.pad #28
	sub	sp, sp, #28
	ldr	r10, [r3, #524]
	mvn	r3, #0
	add	lr, r10, r5
	strh	r3, [lr, #2]	@ movhi
	strh	r3, [r10, r5]	@ movhi
	ldr	r3, [r0]
	cmp	r3, #0
	beq	.L104
	ldr	r2, [r4, #576]
	add	r8, r4, #572
	ldr	r9, [r4, #568]
	ldrh	r8, [r8]
	str	r2, [sp, #4]
	mov	r2, r1, asl #1
	ldr	ip, [sp, #4]
	ldr	r7, [r9, r1, asl #2]
	ldrh	r6, [lr, #4]
	ldrh	ip, [ip, r2]
	muls	ip, r6, ip
	str	r8, [sp, #8]
	ubfx	r7, r7, #11, #8
	ldrneh	r8, [sp, #8]
	mov	r2, r1, asl #2
	ldrh	r2, [r9, r2]
	smulbbne	r7, r7, r8
	ubfx	r2, r2, #0, #11
	addne	r2, r2, r7
	uxtahne	ip, ip, r2
	ldr	r2, [r4, #524]
	cmp	r6, #0
	mov	r6, #0
	rsb	r4, r2, r3
	str	r2, [sp, #12]
	mvneq	ip, #0
	ldr	r2, .L105+4
	mov	r4, r4, asr #1
	mul	r2, r2, r4
	ldr	r4, .L105+8
	ldrh	r4, [r4]
	uxth	r2, r2
	str	r4, [sp, #16]
.L94:
	add	r6, r6, #1
	ldr	r4, [sp, #16]
	uxth	r6, r6
	cmp	r6, r4
	movls	r4, #0
	movhi	r4, #1
	cmp	r1, r2
	orreq	r4, r4, #1
	cmp	r4, #0
	bne	.L84
	ldr	r7, [sp, #4]
	mov	r4, r2, asl #1
	ldr	r8, [r9, r2, asl #2]
	ldrh	fp, [r3, #4]
	ldrh	r4, [r7, r4]
	mov	r7, r2, asl #2
	muls	r4, fp, r4
	ubfx	r8, r8, #11, #8
	ldrh	r7, [r9, r7]
	ubfx	r7, r7, #0, #11
	str	r7, [sp, #20]
	ldrneh	r7, [sp, #8]
	smulbbne	r8, r8, r7
	ldrne	r7, [sp, #20]
	addne	r8, r7, r8
	uxtahne	r4, r4, r8
	cmp	fp, #0
	cmpne	r4, ip
	bcs	.L92
	ldrh	r4, [r3]
	movw	r7, #65535
	cmp	r4, r7
	streqh	r2, [lr, #2]	@ movhi
	streqh	r1, [r3]	@ movhi
	beq	.L84
.L93:
	ldr	r2, [sp, #12]
	mov	r3, #6
	mla	r3, r3, r4, r2
	mov	r2, r4
	b	.L94
.L92:
	ldrh	ip, [r3, #2]
	strh	ip, [lr, #2]	@ movhi
	strh	r2, [r10, r5]	@ movhi
	ldr	r2, [r0]
	cmp	r3, r2
	bne	.L95
	strh	r1, [r3, #2]	@ movhi
.L104:
	str	lr, [r0]
	b	.L84
.L95:
	ldr	r2, .L105
	ldrh	ip, [r3, #2]
	ldr	r0, [r2, #524]
	mov	r2, #6
	mul	r2, r2, ip
	strh	r1, [r0, r2]	@ movhi
	strh	r1, [r3, #2]	@ movhi
.L84:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L106:
	.align	2
.L105:
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LANCHOR0+564
	.fnend
	.size	_insert_data_list, .-_insert_data_list
	.align	2
	.type	_list_update_data_list, %function
_list_update_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r4, r1
	ldr	r1, .L118
	mov	r6, r0
	ldr	r3, [r1, #580]
	ldrh	r0, [r3, #16]
	cmp	r0, r4
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	mov	r5, r1
	ldrh	r1, [r3, #48]
	cmp	r1, r4
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	ldrh	r3, [r3, #80]
	cmp	r3, r4
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	mov	r8, #6
	ldr	r10, [r5, #524]
	mul	r8, r8, r4
	ldr	r3, [r6]
	add	r9, r10, r8
	cmp	r9, r3
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	ldrh	r3, [r9, #2]
	mov	r7, r2
	movw	r2, #65535
	cmp	r3, r2
	bne	.L111
	ldrh	r2, [r10, r8]
	cmp	r2, r3
	bne	.L111
	ldr	r1, .L118+4
	mov	r2, #255
	ldr	r0, .L118+8
	bl	printk
	bl	dump_stack
.L111:
	ldrh	r2, [r9, #2]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L112
	ldrh	r3, [r10, r8]
	cmp	r3, r2
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L112:
	ldr	r0, [r5, #576]
	mov	r3, r4, asl #1
	ldrh	r1, [r0, r3]
	ldrh	r3, [r9, #4]
	cmp	r3, #0
	mulne	r1, r3, r1
	mov	r3, #6
	mul	r2, r3, r2
	ldr	r3, .L118+12
	mvneq	r1, #0
	mov	ip, r2, asr #1
	mul	r3, r3, ip
	mov	r3, r3, asl #1
	ldrh	r0, [r0, r3]
	ldr	r3, [r5, #524]
	add	r2, r3, r2
	ldrh	r3, [r2, #4]
	cmp	r3, #0
	mulne	r3, r3, r0
	mvneq	r3, #0
	cmp	r1, r3
	ldmcsfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	mov	r0, r6
	mov	r1, r4
	mov	r2, r7
	bl	_list_remove_node
	mov	r0, r6
	mov	r1, r4
	mov	r2, r7
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	b	_insert_data_list
.L119:
	.align	2
.L118:
	.word	.LANCHOR0
	.word	.LANCHOR1+20
	.word	.LC0
	.word	-1431655765
	.fnend
	.size	_list_update_data_list, .-_list_update_data_list
	.align	2
	.global	zftl_read_flash_info
	.type	zftl_read_flash_info, %function
zftl_read_flash_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r1, #11
	mov	r4, r0
	bl	__memzero
	ldr	r2, .L125
	mov	ip, #1
	ldr	r3, .L125+4
	ldrh	r1, [r2, #2]
	add	r2, r2, #588
	ldrb	r0, [r3, #13]	@ zero_extendqisi2
	smulbb	r1, r0, r1
	strh	r1, [r4, #4]	@ unaligned
	ldrb	r1, [r2, #-4]	@ zero_extendqisi2
	strb	r1, [r4, #7]
	ldr	r1, [r2, #-68]
	str	r1, [r4]	@ unaligned
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	ldrb	r0, [r2, #-3]	@ zero_extendqisi2
	strb	r1, [r4, #6]
	mov	r1, #32
	ldrb	r3, [r3, #11]	@ zero_extendqisi2
	strb	r1, [r4, #8]
	strb	r3, [r4, #9]
	mov	r3, #0
	strb	r3, [r4, #10]
.L121:
	uxtb	r1, r3
	cmp	r1, r0
	bcs	.L124
	ldrb	lr, [r3, r2]	@ zero_extendqisi2
	add	r3, r3, #1
	ldrb	r1, [r4, #10]	@ zero_extendqisi2
	orr	r1, r1, ip, asl lr
	strb	r1, [r4, #10]
	b	.L121
.L124:
	ldmfd	sp!, {r4, pc}
.L126:
	.align	2
.L125:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	zftl_read_flash_info, .-zftl_read_flash_info
	.section	.text.unlikely,"ax",%progbits
	.align	2
	.type	rk_simple_strtoull.constprop.34, %function
rk_simple_strtoull.constprop.34:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r3, #48
	movne	r2, r0
	movne	lr, #10
	bne	.L128
	ldrb	r3, [r0, #1]	@ zero_extendqisi2
	add	r2, r0, #1
	orr	r3, r3, #32
	cmp	r3, #120
	bne	.L139
	ldrb	r3, [r0, #2]	@ zero_extendqisi2
	bic	ip, r3, #32
	sub	ip, ip, #65
	cmp	ip, #25
	bls	.L129
	sub	r3, r3, #48
	cmp	r3, #9
	bls	.L129
.L139:
	mov	lr, #8
.L128:
	mov	r0, #0
.L130:
	mov	r4, r2
	ldrb	ip, [r2], #1	@ zero_extendqisi2
	bic	r3, ip, #32
	sub	r3, r3, #65
	cmp	r3, #25
	sub	r3, ip, #48
	bls	.L131
	cmp	r3, #9
	bls	.L131
.L135:
	cmp	r1, #0
	bne	.L132
	ldmfd	sp!, {r4, pc}
.L132:
	str	r4, [r1]
	ldmfd	sp!, {r4, pc}
.L129:
	add	r2, r0, #2
	mov	lr, #16
	b	.L128
.L131:
	cmp	r3, #9
	orrhi	ip, ip, #32
	subhi	r3, ip, #87
	cmp	r3, lr
	bcs	.L135
	mla	r0, lr, r0, r3
	b	.L130
	.fnend
	.size	rk_simple_strtoull.constprop.34, .-rk_simple_strtoull.constprop.34
	.text
	.align	2
	.global	flash_lsb_page_tbl_build
	.type	flash_lsb_page_tbl_build, %function
flash_lsb_page_tbl_build:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r0, #0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bne	.L143
	ldr	r3, .L172
.L144:
	mov	r2, r0, asl #1
	strh	r0, [r2, r3]	@ movhi
	add	r0, r0, #1
	cmp	r0, #256
	bne	.L144
.L148:
	mov	r1, #255
	ldr	r0, .L172+4
	mov	r2, #1024
	bl	memset
	ldr	r1, .L172
	mov	r3, #0
	add	ip, r1, #592
	b	.L145
.L143:
	cmp	r0, #1
	bne	.L146
	ldr	ip, .L172
	mov	r3, #0
.L147:
	uxth	r2, r3
	mov	lr, r3, asl #1
	cmp	r2, #3
	movls	r0, #0
	movhi	r0, #1
	bics	r1, r0, r3
	add	r3, r3, #1
	movne	r1, #2
	moveq	r1, #3
	cmp	r0, #0
	rsb	r1, r1, r2, asl #1
	movne	r2, r1
	cmp	r3, #256
	strh	r2, [lr, ip]	@ movhi
	bne	.L147
	b	.L148
.L146:
	cmp	r0, #2
	bne	.L149
	ldr	r1, .L172
	mov	r3, #0
.L150:
	uxth	r2, r3
	mov	r0, r3, asl #1
	cmp	r2, #1
	add	r3, r3, #1
	mov	ip, r2, asl #1
	subhi	r2, ip, #1
	cmp	r3, #256
	strh	r2, [r0, r1]	@ movhi
	bne	.L150
	b	.L148
.L149:
	cmp	r0, #3
	bne	.L151
	ldr	ip, .L172
	mov	r3, #0
.L152:
	uxth	r2, r3
	mov	lr, r3, asl #1
	cmp	r2, #5
	movls	r0, #0
	movhi	r0, #1
	bics	r1, r0, r3
	add	r3, r3, #1
	movne	r1, #4
	moveq	r1, #5
	cmp	r0, #0
	rsb	r1, r1, r2, asl #1
	movne	r2, r1
	cmp	r3, #256
	strh	r2, [lr, ip]	@ movhi
	bne	.L152
	b	.L148
.L151:
	cmp	r0, #4
	mov	r3, #0
	bne	.L153
	ldr	r2, .L172+8
	strh	r3, [r2, #4]	@ movhi
	mov	r3, #1
	strh	r0, [r2, #12]	@ movhi
	strh	r3, [r2, #6]	@ movhi
	mov	r3, #2
	strh	r3, [r2, #8]	@ movhi
	mov	r3, #3
	strh	r3, [r2, #10]	@ movhi
	mov	r3, #5
	strh	r3, [r2, #14]	@ movhi
	mov	r3, #7
	strh	r3, [r2, #16]	@ movhi
	mov	r3, #8
	strh	r3, [r2, #18]!	@ movhi
.L154:
	tst	r3, #1
	movne	r1, #7
	moveq	r1, #6
	rsb	r1, r1, r3, asl #1
	add	r3, r3, #1
	strh	r1, [r2, #2]!	@ movhi
	uxth	r3, r3
	cmp	r3, #256
	bne	.L154
	b	.L148
.L153:
	cmp	r0, #5
	bne	.L155
	ldr	r2, .L172
.L156:
	mov	r1, r3, asl #1
	strh	r3, [r1, r2]	@ movhi
	add	r3, r3, #1
	cmp	r3, #16
	bne	.L156
	ldr	r2, .L172+12
.L157:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, #496
	bne	.L157
	b	.L148
.L155:
	cmp	r0, #8
	bne	.L158
	ldr	r2, .L172
.L159:
	strh	r3, [r3, r2]	@ movhi
	add	r3, r3, #2
	cmp	r3, #512
	bne	.L159
	b	.L148
.L158:
	cmp	r0, #9
	bne	.L148
	ldr	r2, .L172+8
	movw	r1, #509
	strh	r3, [r2, #4]	@ movhi
	mov	r3, #1
	strh	r3, [r2, #6]	@ movhi
	mov	r3, #2
	strh	r3, [r2, #8]!	@ movhi
	mov	r3, #3
.L160:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, r1
	bne	.L160
	b	.L148
.L145:
	ldrh	r2, [r3, r1]
	add	r3, r3, #2
	cmp	r3, #512
	mov	r0, r2, asl #1
	strh	r2, [ip, r0]	@ movhi
	bne	.L145
	ldmfd	sp!, {r3, pc}
.L173:
	.align	2
.L172:
	.word	.LANCHOR0+4
	.word	.LANCHOR0+596
	.word	.LANCHOR0
	.word	.LANCHOR0+34
	.fnend
	.size	flash_lsb_page_tbl_build, .-flash_lsb_page_tbl_build
	.align	2
	.global	nand_flash_print_info
	.type	nand_flash_print_info, %function
nand_flash_print_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, lr}
	.save {r4, lr}
	.pad #16
	ldr	r4, .L297
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L175
	ldr	r0, .L297+4
	ldr	r1, .L297+8
	bl	printk
.L175:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L176
	ldr	r3, .L297+12
	ldr	r0, [r3, #1620]
	ldrb	ip, [r0, #4]	@ zero_extendqisi2
	ldrb	r1, [r0, #1]	@ zero_extendqisi2
	ldrb	r2, [r0, #2]	@ zero_extendqisi2
	ldrb	r3, [r0, #3]	@ zero_extendqisi2
	str	ip, [sp]
	ldrb	ip, [r0, #5]	@ zero_extendqisi2
	str	ip, [sp, #4]
	ldrb	r0, [r0, #6]	@ zero_extendqisi2
	str	r0, [sp, #8]
	ldr	r0, .L297+16
	bl	printk
.L176:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L177
	ldr	r3, .L297+12
	ldr	r0, .L297+20
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #8]	@ zero_extendqisi2
	bl	printk
.L177:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L178
	ldr	r3, .L297+12
	ldr	r0, .L297+24
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	bl	printk
.L178:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L179
	ldr	r3, .L297+12
	ldr	r0, .L297+28
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #10]
	bl	printk
.L179:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L180
	ldr	r3, .L297+12
	ldr	r0, .L297+32
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	bl	printk
.L180:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L181
	ldr	r3, .L297+12
	ldr	r0, .L297+36
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	bl	printk
.L181:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L182
	ldr	r3, .L297+12
	ldr	r0, .L297+40
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #14]
	bl	printk
.L182:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L183
	ldr	r3, .L297+12
	ldr	r0, .L297+44
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #23]	@ zero_extendqisi2
	bl	printk
.L183:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L184
	ldr	r3, .L297+12
	ldr	r0, .L297+48
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #18]	@ zero_extendqisi2
	bl	printk
.L184:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L185
	ldr	r3, .L297+12
	ldr	r0, .L297+52
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #19]	@ zero_extendqisi2
	bl	printk
.L185:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L186
	ldr	r3, .L297+12
	ldr	r0, .L297+56
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #20]	@ zero_extendqisi2
	bl	printk
.L186:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L187
	ldr	r3, .L297+12
	ldr	r0, .L297+60
	ldrb	r1, [r3, #1624]	@ zero_extendqisi2
	bl	printk
.L187:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L188
	ldr	r3, .L297+12
	ldr	r0, .L297+64
	ldrb	r1, [r3]	@ zero_extendqisi2
	bl	printk
.L188:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L189
	ldr	r3, .L297+12
	ldr	r0, .L297+68
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #22]	@ zero_extendqisi2
	bl	printk
.L189:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L190
	ldr	r3, .L297+12
	ldr	r0, .L297+72
	ldrb	r1, [r3, #585]	@ zero_extendqisi2
	bl	printk
.L190:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L191
	ldr	r3, .L297+12
	ldr	r0, .L297+76
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	and	r1, r1, #1
	bl	printk
.L191:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L192
	ldr	r3, .L297+12
	ldr	r0, .L297+80
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #1, #1
	bl	printk
.L192:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L193
	ldr	r3, .L297+12
	ldr	r0, .L297+84
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #2, #1
	bl	printk
.L193:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L194
	ldr	r3, .L297+12
	ldr	r0, .L297+88
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #3, #1
	bl	printk
.L194:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L195
	ldr	r3, .L297+12
	ldr	r0, .L297+92
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #4, #1
	bl	printk
.L195:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L196
	ldr	r3, .L297+12
	ldr	r0, .L297+96
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #5, #1
	bl	printk
.L196:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L197
	ldr	r3, .L297+12
	ldr	r0, .L297+100
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #6, #1
	bl	printk
.L197:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L198
	ldr	r3, .L297+12
	ldr	r0, .L297+104
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #7, #1
	bl	printk
.L198:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L199
	ldr	r3, .L297+12
	ldr	r0, .L297+108
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #8, #1
	bl	printk
.L199:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L200
	ldr	r3, .L297+12
	ldr	r0, .L297+112
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #9, #1
	bl	printk
.L200:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L201
	ldr	r3, .L297+12
	ldr	r0, .L297+116
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #10, #1
	bl	printk
.L201:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L202
	ldr	r3, .L297+12
	ldr	r0, .L297+120
	ldrb	r1, [r3]	@ zero_extendqisi2
	ldrb	r2, [r3, #1625]	@ zero_extendqisi2
	bl	printk
.L202:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L203
	ldr	r3, .L297+12
	ldr	r0, .L297+124
	ldrb	r1, [r3, #1636]	@ zero_extendqisi2
	ldrb	r2, [r3, #1637]	@ zero_extendqisi2
	bl	printk
.L203:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L204
	ldr	r3, .L297+12
	ldr	r0, .L297+128
	ldrb	r1, [r3, #1634]	@ zero_extendqisi2
	ldrb	r2, [r3, #1635]	@ zero_extendqisi2
	bl	printk
.L204:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L174
	ldr	r3, .L297+12
	ldr	r0, .L297+132
	ldrb	r1, [r3, #1658]	@ zero_extendqisi2
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, lr}
	b	printk
.L174:
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, pc}
.L298:
	.align	2
.L297:
	.word	.LANCHOR2
	.word	.LC4
	.word	.LANCHOR1+44
	.word	.LANCHOR0
	.word	.LC5
	.word	.LC6
	.word	.LC7
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
	.word	.LC29
	.word	.LC30
	.word	.LC31
	.word	.LC32
	.word	.LC33
	.word	.LC34
	.fnend
	.size	nand_flash_print_info, .-nand_flash_print_info
	.align	2
	.global	nandc_init
	.type	nandc_init, %function
nandc_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r5, .L319
	mov	r3, #0
	str	r3, [sp, #12]
	mov	r7, r0
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L300
	ldr	r0, .L319+4
	mov	r2, r7
	ldr	r1, .L319+8
	bl	printk
.L300:
	ldr	r4, .L319+12
	mov	r3, #6
	ldr	r2, [r7, #352]
	ldr	r6, .L319+12
	strb	r3, [r4, #516]
	ldr	r3, .L319+16
	str	r7, [r4, #528]
	cmp	r2, r3
	ldr	r2, [r7, #128]
	moveq	r3, #8
	streqb	r3, [r4, #516]
	ldr	r3, .L319+20
	cmp	r2, r3
	ldr	r2, .L319+24
	moveq	r3, #9
	streqb	r3, [r4, #516]
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L303
	mov	r3, #1
	strb	r3, [r6, #1659]
	ldr	r3, [sp, #12]
	orr	r3, r3, #256
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bic	r3, r3, #1835008
	orr	r3, r3, #524288
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	str	r3, [r7]
	mov	r3, #0
	ldr	r0, [r6, #528]
	str	r3, [r0, #520]
	movw	r3, #4161
	str	r3, [r0, #4]
	movw	r3, #8321
	str	r3, [r0, #8]
	mov	r3, #38
	str	r2, [r0, #80]
	str	r3, [r0, #84]
	mov	r3, #39
	str	r3, [r0, #84]
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L305
	ldr	r1, [r0]
	ldr	r2, [r0, #8]
	ldr	r3, [r0, #80]
	ldr	ip, [r0, #84]
	ldr	r0, [r0, #88]
	b	.L318
.L303:
	ldr	r3, [sp, #12]
	mov	r1, #0
	strb	r1, [r6, #1659]
	mov	r0, #2048
	orr	r3, r3, #256
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	bic	r3, r3, #117440512
	orr	r3, r3, #16777216
	str	r3, [sp, #12]
	ldr	r3, [sp, #12]
	str	r3, [r7]
	ldr	r3, [r6, #528]
	str	r1, [r3, #336]
	movw	r1, #4193
	str	r1, [r3, #4]
	movw	r1, #8321
	str	r1, [r3, #344]
	str	r2, [r3, #304]
	mov	r2, #38
	str	r2, [r3, #308]
	mov	r2, #39
	str	r2, [r3, #308]
	bl	ftl_malloc
	ldr	r3, [r5]
	tst	r3, #4096
	str	r0, [r6, #1660]
	beq	.L305
	ldr	r0, [r6, #528]
	ldr	r1, [r0]
	ldr	r2, [r0, #344]
	ldr	r3, [r0, #304]
	ldr	ip, [r0, #308]
	ldr	r0, [r0, #312]
.L318:
	str	r0, [sp, #4]
	str	ip, [sp]
	ldr	r0, .L319+28
	bl	printk
.L305:
	movw	r2, #1690
	mov	r3, #1
	strb	r3, [r4, #1688]
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	strb	r3, [r4, #584]
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L299
	ldr	r3, .L319+12
	ldr	r0, .L319+32
	ldrb	r1, [r3, #516]	@ zero_extendqisi2
	bl	printk
.L299:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L320:
	.align	2
.L319:
	.word	.LANCHOR2
	.word	.LC35
	.word	.LANCHOR1+68
	.word	.LANCHOR0
	.word	1446522928
	.word	1446588464
	.word	1052675
	.word	.LC36
	.word	.LC37
	.fnend
	.size	nandc_init, .-nandc_init
	.align	2
	.global	timer_delay_ns
	.type	timer_delay_ns, %function
timer_delay_ns:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #8
	sub	sp, sp, #8
	mov	r0, r0, lsr #4
	str	r0, [sp, #4]
.L322:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	sub	r2, r3, #1
	str	r2, [sp, #4]
	bne	.L322
	add	sp, sp, #8
	@ sp needed
	bx	lr
	.fnend
	.size	timer_delay_ns, .-timer_delay_ns
	.align	2
	.global	flash_read_status
	.type	flash_read_status, %function
flash_read_status:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r3, #112
	mov	r4, r0
	str	r3, [r0, #8]
	mov	r0, #120
	bl	timer_delay_ns
	ldr	r0, [r4]
	uxtb	r0, r0
	ldmfd	sp!, {r4, pc}
	.fnend
	.size	flash_read_status, .-flash_read_status
	.align	2
	.global	toshiba_set_rr_para
	.type	toshiba_set_rr_para, %function
toshiba_set_rr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	add	r8, r1, r1, asl #2
	ldr	r9, .L337
	mov	r5, r0
	ldr	r7, .L337+4
	mov	r6, r1
	add	r10, r9, #80
	mov	r4, #0
.L328:
	ldrb	r3, [r7, #1693]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L336
	mov	r3, #85
	str	r3, [r5, #8]
	ldrsb	r3, [r4, r10]
	mov	r0, #200
	str	r3, [r5, #4]
	bl	timer_delay_ns
	ldrb	r3, [r7, #1692]	@ zero_extendqisi2
	cmp	r3, #34
	addeq	r3, r4, r8
	addeq	r3, r10, r3
	beq	.L335
	cmp	r3, #35
	addne	r3, r9, r6
	ldrnesb	r3, [r3, #224]
	bne	.L334
	ldr	r3, .L337+8
	add	r2, r4, r8
	add	r3, r3, r2
.L335:
	ldrsb	r3, [r3, #5]
.L334:
	str	r3, [r5]
	add	r4, r4, #1
	b	.L328
.L336:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L338:
	.align	2
.L337:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR1+128
	.fnend
	.size	toshiba_set_rr_para, .-toshiba_set_rr_para
	.align	2
	.type	hynix_set_rr_para, %function
hynix_set_rr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, r0, asl #8
	ldr	r3, .L346
	ldr	r4, [r3, #1696]
	ldr	r7, [r3, #528]
	add	lr, r4, #128
	add	r9, r4, #127
	mov	r3, r4
	ldrb	ip, [r4, #113]	@ zero_extendqisi2
	ldrb	r2, [r3, #112]!	@ zero_extendqisi2
	add	r6, r7, r5
	cmp	r2, #8
	mul	r1, r1, ip
	movne	r2, #160
	mlane	r1, r2, r0, r1
	sub	ip, ip, #1
	add	r8, lr, ip
	mov	r2, #54
	str	r2, [r6, #2056]
	add	r1, r1, #32
	add	r3, r3, r1
	sub	r4, r3, #1
.L342:
	cmp	r9, r8
	beq	.L345
	ldrb	r3, [r9, #1]!	@ zero_extendqisi2
	mov	r0, #120
	str	r3, [r6, #2052]
	bl	timer_delay_ns
	ldrsb	r3, [r4, #1]!
	str	r3, [r6, #2048]
	b	.L342
.L345:
	add	r5, r7, r5
	mov	r3, #22
	str	r3, [r5, #2056]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L347:
	.align	2
.L346:
	.word	.LANCHOR0
	.fnend
	.size	hynix_set_rr_para, .-hynix_set_rr_para
	.align	2
	.global	hynix_reconfig_rr_para
	.type	hynix_reconfig_rr_para, %function
hynix_reconfig_rr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r5, .L354
	ldrb	r2, [r5, #1692]	@ zero_extendqisi2
	sub	r2, r2, #1
	cmp	r2, #7
	ldmhifd	sp!, {r3, r4, r5, pc}
	ldr	r2, [r5, #1696]
	add	r2, r2, r0
	ldrb	r2, [r2, #120]	@ zero_extendqisi2
	cmp	r2, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	mov	r1, #0
	mov	r4, r0
	bl	hynix_set_rr_para
	ldr	r2, [r5, #1696]
	mov	r3, #0
	add	r4, r2, r4
	strb	r3, [r4, #120]
	ldmfd	sp!, {r3, r4, r5, pc}
.L355:
	.align	2
.L354:
	.word	.LANCHOR0
	.fnend
	.size	hynix_reconfig_rr_para, .-hynix_reconfig_rr_para
	.align	2
	.global	nandc_set_ddr_para
	.type	nandc_set_ddr_para, %function
nandc_set_ddr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L359
	ldrb	r2, [r3, #516]	@ zero_extendqisi2
	ldr	r3, [r3, #528]
	cmp	r2, #9
	mov	r2, r0, asl #16
	mov	r0, r0, asl #8
	orr	r0, r2, r0
	orr	r0, r0, #3
	streq	r0, [r3, #80]
	strne	r0, [r3, #304]
	bx	lr
.L360:
	.align	2
.L359:
	.word	.LANCHOR0
	.fnend
	.size	nandc_set_ddr_para, .-nandc_set_ddr_para
	.align	2
	.global	nandc_get_ddr_para
	.type	nandc_get_ddr_para, %function
nandc_get_ddr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L365
	ldrb	r2, [r3, #516]	@ zero_extendqisi2
	ldr	r3, [r3, #528]
	cmp	r2, #9
	ldreq	r0, [r3, #80]
	ldrne	r0, [r3, #304]
	ubfx	r0, r0, #8, #8
	bx	lr
.L366:
	.align	2
.L365:
	.word	.LANCHOR0
	.fnend
	.size	nandc_get_ddr_para, .-nandc_get_ddr_para
	.align	2
	.global	nandc_set_if_mode
	.type	nandc_set_if_mode, %function
nandc_set_if_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r1, .L375
	ands	ip, r0, #6
	ldr	r3, [r1, #528]
	ldr	r2, [r3]
	bfieq	r2, ip, #13, #1
	beq	.L371
	orr	r2, r2, #24576
	ldrb	r1, [r1, #516]	@ zero_extendqisi2
	bfc	r2, #15, #1
	tst	r0, #4
	orr	r2, r2, #196608
	movw	r0, #8321
	orrne	r2, r2, #32768
	cmp	r1, #9
	ldr	r1, .L375+4
	streq	r0, [r3, #8]
	strne	r0, [r3, #344]
	streq	r1, [r3, #80]
	moveq	r1, #38
	strne	r1, [r3, #304]
	movne	r1, #38
	streq	r1, [r3, #84]
	moveq	r1, #39
	strne	r1, [r3, #308]
	movne	r1, #39
	streq	r1, [r3, #84]
	strne	r1, [r3, #308]
.L371:
	str	r2, [r3]
	bx	lr
.L376:
	.align	2
.L375:
	.word	.LANCHOR0
	.word	1052675
	.fnend
	.size	nandc_set_if_mode, .-nandc_set_if_mode
	.align	2
	.global	nandc_cs
	.type	nandc_cs, %function
nandc_cs:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L378
	mov	r2, #1
	mov	r0, r2, asl r0
	ldr	r1, [r3, #528]
	ldr	r3, [r1]
	bfi	r3, r0, #0, #8
	str	r3, [r1]
	bx	lr
.L379:
	.align	2
.L378:
	.word	.LANCHOR0
	.fnend
	.size	nandc_cs, .-nandc_cs
	.align	2
	.global	nandc_de_cs
	.type	nandc_de_cs, %function
nandc_de_cs:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L381
	ldr	r2, [r3, #528]
	ldr	r3, [r2]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2]
	bx	lr
.L382:
	.align	2
.L381:
	.word	.LANCHOR0
	.fnend
	.size	nandc_de_cs, .-nandc_de_cs
	.align	2
	.global	flash_wait_device_ready_raw
	.type	flash_wait_device_ready_raw, %function
flash_wait_device_ready_raw:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r0
	ldr	r4, .L388
	mov	r6, r1
	mov	r7, r2
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L384
	ldr	r1, .L388+4
	movw	r2, #691
	ldr	r0, .L388+8
	bl	printk
	bl	dump_stack
.L384:
	add	r5, r4, r5
	ldr	r4, [r4, #528]
	mov	r10, r6, lsr #8
	mov	r9, r6, lsr #16
	ldrb	r8, [r5, #588]	@ zero_extendqisi2
	mov	fp, #120
	mov	r0, r8
	add	r4, r4, r8, asl #8
	bl	nandc_cs
.L385:
	str	fp, [r4, #2056]
	uxtb	r3, r6
	mov	r0, #120
	str	r3, [r4, #2052]
	str	r10, [r4, #2052]
	str	r9, [r4, #2052]
	bl	timer_delay_ns
	ldr	r5, [r4, #2048]
	uxtb	r5, r5
	sub	r3, r5, #255
	and	r2, r7, r5
	clz	r3, r3
	mov	r3, r3, lsr #5
	cmp	r2, r7
	orrne	r3, r3, #1
	cmp	r3, #0
	bne	.L385
	mov	r0, r8
	bl	nandc_de_cs
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L389:
	.align	2
.L388:
	.word	.LANCHOR0
	.word	.LANCHOR1+232
	.word	.LC0
	.fnend
	.size	flash_wait_device_ready_raw, .-flash_wait_device_ready_raw
	.align	2
	.global	flash_wait_device_ready
	.type	flash_wait_device_ready, %function
flash_wait_device_ready:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r3, #1
	ldr	r6, .L402
	tst	r0, #50331648
	mov	r8, r1
	ldrb	r2, [r6, #1700]	@ zero_extendqisi2
	rsb	r5, r2, #24
	mov	r4, r3, asl r5
	mov	r3, r3, asl r2
	sub	r3, r3, #1
	sub	r4, r4, #1
	and	r5, r3, r0, asr r5
	and	r4, r4, r0
	uxtb	r5, r5
	bne	.L391
	ldrb	r3, [r6]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L392
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L391
.L392:
	ldrh	r7, [r6, #2]
	mov	r0, r4
	mov	r1, r7
	bl	__aeabi_uidiv
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L402
	mul	r0, r7, r0
	rsb	r4, r0, r4
	mov	r4, r4, asl #1
	addeq	r4, r3, r4
	ldreqh	r4, [r4, #4]
	add	r4, r4, r0
.L391:
	mov	r0, r5
	mov	r1, r4
	mov	r2, r8
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	b	flash_wait_device_ready_raw
.L403:
	.align	2
.L402:
	.word	.LANCHOR0
	.fnend
	.size	flash_wait_device_ready, .-flash_wait_device_ready
	.align	2
	.global	nandc_wait_flash_ready_no_delay
	.type	nandc_wait_flash_ready_no_delay, %function
nandc_wait_flash_ready_no_delay:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	ldr	r4, .L410
	ldr	r5, .L410+4
.L406:
	ldr	r3, [r5, #528]
	ldr	r3, [r3]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L407
	mov	r0, #10
	bl	timer_delay_ns
	subs	r4, r4, #1
	bne	.L406
	mvn	r0, #0
	b	.L405
.L407:
	mov	r0, #0
.L405:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L411:
	.align	2
.L410:
	.word	100000
	.word	.LANCHOR0
	.fnend
	.size	nandc_wait_flash_ready_no_delay, .-nandc_wait_flash_ready_no_delay
	.align	2
	.global	zftl_flash_enter_slc_mode
	.type	zftl_flash_enter_slc_mode, %function
zftl_flash_enter_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L433
	ldrb	r1, [r2]	@ zero_extendqisi2
	cmp	r1, #0
	bxeq	lr
	cmp	r1, #1
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, [r2, #528]
	bne	.L415
	ldr	r2, .L433+4
	ldrb	r2, [r2, #33]	@ zero_extendqisi2
	cmp	r2, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	b	.L432
.L415:
	cmp	r1, #2
	bne	.L417
	add	r2, r2, r0
	ldrb	r1, [r2, #1704]	@ zero_extendqisi2
	cmp	r1, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	mov	r1, #0
	strb	r1, [r2, #1704]
	ldr	r2, .L433+4
	ldrb	r2, [r2, #33]	@ zero_extendqisi2
	cmp	r2, r1
	ldmeqfd	sp!, {r3, r4, r5, pc}
.L432:
	add	r0, r3, r0, asl #8
	str	r2, [r0, #2056]
	ldmfd	sp!, {r3, r4, r5, pc}
.L417:
	cmp	r1, #3
	ldmnefd	sp!, {r3, r4, r5, pc}
	add	r2, r2, r0
	ldrb	r1, [r2, #1704]	@ zero_extendqisi2
	cmp	r1, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	add	r4, r3, r0, asl #8
	mov	r3, #239
	mov	r0, #50
	mov	r5, #0
	str	r3, [r4, #2056]
	mov	r3, #145
	str	r3, [r4, #2052]
	strb	r5, [r2, #1704]
	bl	timer_delay_ns
	str	r5, [r4, #2048]
	mov	r3, #1
	mov	r0, #150
	str	r3, [r4, #2048]
	str	r5, [r4, #2048]
	str	r5, [r4, #2048]
	bl	timer_delay_ns
	bl	nandc_wait_flash_ready_no_delay
	mov	r3, #218
	mov	r0, #150
	str	r3, [r4, #2056]
	bl	timer_delay_ns
	ldmfd	sp!, {r3, r4, r5, lr}
	b	nandc_wait_flash_ready_no_delay
.L434:
	.align	2
.L433:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	zftl_flash_enter_slc_mode, .-zftl_flash_enter_slc_mode
	.global	__aeabi_idiv
	.align	2
	.global	flash_die_info_init
	.type	flash_die_info_init, %function
flash_die_info_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r5, .L466
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L436
	ldr	r0, .L466+4
	ldr	r1, .L466+8
	bl	printk
.L436:
	ldr	r4, .L466+12
	mov	r6, #0
	ldrh	r3, [r5, #30]
	ldrb	r1, [r5, #16]	@ zero_extendqisi2
	add	r9, r4, #1744
	ldrh	r0, [r5, #14]
	strh	r3, [r4, #2]	@ movhi
	strb	r6, [r4, #585]
	bl	__aeabi_idiv
	movw	r3, #1708
	mov	r1, #8
	ldr	r7, .L466+16
	strh	r0, [r4, r3]	@ movhi
	add	r0, r4, #588
	bl	__memzero
	add	r0, r4, #1712
	mov	r1, #32
	bl	__memzero
.L441:
	ldrb	r2, [r5, #4]	@ zero_extendqisi2
	add	r1, r9, r6, asl #3
	ldr	r8, .L466+20
	mov	r3, #2
	ldr	r0, .L466+24
	strb	r3, [r6, r7]
	mov	r3, #0
.L437:
	cmp	r3, r2
	bcs	.L464
	add	ip, r0, r3
	ldrb	lr, [ip, #1]	@ zero_extendqisi2
	ldrb	ip, [r1, r3]	@ zero_extendqisi2
	cmp	lr, ip
	bne	.L438
	add	r3, r3, #1
	b	.L437
.L464:
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	mov	r1, #0
	uxtb	r0, r6
	add	r2, r4, r3, asl #2
	str	r1, [r2, #1712]
	add	r2, r3, #1
	add	r3, r4, r3
	strb	r2, [r4, #585]
	strb	r0, [r3, #588]
	bl	zftl_flash_enter_slc_mode
.L438:
	add	r6, r6, #1
	cmp	r6, #4
	bne	.L441
	ldrb	r2, [r5, #12]	@ zero_extendqisi2
	ldr	r3, .L466
	cmp	r2, #2
	beq	.L442
.L449:
	ldrb	r3, [r5, #17]	@ zero_extendqisi2
	ldrb	r2, [r4, #585]	@ zero_extendqisi2
	smulbb	r2, r2, r3
	ldrh	r3, [r5, #18]
	smulbb	r3, r2, r3
	ldr	r2, .L466+28
	strh	r3, [r2]	@ movhi
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L442:
	ldrh	ip, [r3, #18]
	mov	lr, #0
	ldrh	r2, [r4, #2]
	add	r3, r3, #4
	and	ip, ip, #65280
	ldrb	r1, [r4, #585]	@ zero_extendqisi2
	ldrb	r7, [r3]	@ zero_extendqisi2
	mul	r2, r2, ip
	ldrb	ip, [r3, #13]	@ zero_extendqisi2
	ldrb	r6, [r3, #23]	@ zero_extendqisi2
	mul	ip, ip, r2
	mov	r2, ip, asl #1
	str	r2, [sp, #4]
	mov	r2, lr
.L447:
	add	r9, r8, r2, asl #3
	mov	r0, #0
.L443:
	cmp	r0, r7
	bcs	.L465
	add	r10, r3, r0
	ldrb	fp, [r10, #1]	@ zero_extendqisi2
	ldrb	r10, [r9, r0]	@ zero_extendqisi2
	cmp	fp, r10
	bne	.L444
	add	r0, r0, #1
	b	.L443
.L465:
	ldr	lr, [sp, #4]
	mov	r0, r1
	cmp	r6, #0
	add	r1, r4, r1, asl #2
	moveq	lr, ip
	str	lr, [r1, #1712]
	add	r1, r0, #1
	add	r0, r4, r0
	mov	lr, #1
	uxtb	r1, r1
	strb	r2, [r0, #588]
.L444:
	add	r2, r2, #1
	cmp	r2, #4
	bne	.L447
	cmp	lr, #0
	strneb	r1, [r4, #585]
	b	.L449
.L467:
	.align	2
.L466:
	.word	.LANCHOR2
	.word	.LC4
	.word	.LANCHOR1+260
	.word	.LANCHOR0
	.word	.LANCHOR0+1704
	.word	.LANCHOR0+1744
	.word	.LANCHOR2+4
	.word	.LANCHOR0+1776
	.fnend
	.size	flash_die_info_init, .-flash_die_info_init
	.align	2
	.global	zftl_flash_exit_slc_mode
	.type	zftl_flash_exit_slc_mode, %function
zftl_flash_exit_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L493
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #0
	bxeq	lr
	cmp	r2, #1
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	ldr	r7, [r3, #528]
	ldreq	r3, .L493+4
	ldreqb	r3, [r3, #34]	@ zero_extendqisi2
	beq	.L492
	cmp	r2, #2
	bne	.L473
	add	r3, r3, r0
	ldrb	r2, [r3, #1704]	@ zero_extendqisi2
	cmp	r2, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	ldr	r1, .L493+4
	ldrb	r2, [r1, #16]	@ zero_extendqisi2
	cmp	r2, #2
	movne	r2, #4
	strb	r2, [r3, #1704]
	ldrb	r3, [r1, #34]	@ zero_extendqisi2
.L492:
	cmp	r3, #0
	addne	r0, r7, r0, asl #8
	strne	r3, [r0, #2056]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L473:
	cmp	r2, #3
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	add	r6, r3, r0
	ldrb	r3, [r6, #1704]	@ zero_extendqisi2
	cmp	r3, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	ldr	r8, .L493+4
	mov	r4, r0, asl #8
	add	r5, r7, r4
	mov	r0, #50
	add	r4, r7, r4
	ldrb	r3, [r8, #16]	@ zero_extendqisi2
	cmp	r3, #2
	movne	r3, #4
	strb	r3, [r6, #1704]
	mov	r3, #239
	str	r3, [r5, #2056]
	mov	r3, #145
	str	r3, [r5, #2052]
	bl	timer_delay_ns
	ldrb	r3, [r8, #11]	@ zero_extendqisi2
	mov	r0, #150
	cmp	r3, #9
	mov	r3, #1
	streq	r3, [r5, #2048]
	ldrneb	r2, [r6, #1704]	@ zero_extendqisi2
	strne	r2, [r5, #2048]
	str	r3, [r5, #2048]
	mov	r3, #0
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
	bl	timer_delay_ns
	bl	nandc_wait_flash_ready_no_delay
	mov	r3, #223
	mov	r0, #150
	str	r3, [r4, #2056]
	bl	timer_delay_ns
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	b	nandc_wait_flash_ready_no_delay
.L494:
	.align	2
.L493:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	zftl_flash_exit_slc_mode, .-zftl_flash_exit_slc_mode
	.align	2
	.global	flash_start_page_read
	.type	flash_start_page_read, %function
flash_start_page_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L505
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mvn	r5, #0
	ldrb	r4, [r2, #1700]	@ zero_extendqisi2
	mov	r6, r0
	ldrb	r3, [r2, #585]	@ zero_extendqisi2
	mov	r8, r1
	rsb	r7, r4, #24
	mvn	r4, r5, asl r4
	and	r4, r4, r1, lsr r7
	mov	r5, r2
	uxtb	r4, r4
	cmp	r3, r4
	bhi	.L496
	ldr	r1, .L505+4
	movw	r2, #866
	ldr	r0, .L505+8
	bl	printk
	bl	dump_stack
.L496:
	ldrb	r3, [r5, #585]	@ zero_extendqisi2
	ldr	r9, .L505
	cmp	r3, r4
	ldmlsfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	add	r4, r9, r4
	mvn	r3, #0
	bic	r7, r8, r3, asl r7
	ubfx	r8, r8, #24, #2
	ldrb	r10, [r4, #588]	@ zero_extendqisi2
	ldr	r5, [r9, #528]
	mov	r0, r10
	mov	r4, r10, asl #8
	bl	nandc_cs
	cmp	r8, #0
	bne	.L498
	mov	r0, r7
	bl	slc_phy_page_address_calc
	ldrb	r3, [r9]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r7, r0
	beq	.L499
	mov	r0, r10
	bl	zftl_flash_enter_slc_mode
	b	.L499
.L498:
	ldr	r3, [r9, #1620]
	ldrb	r3, [r3, #12]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L500
	ldrb	r3, [r9, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	addeq	r3, r5, r4
	streq	r8, [r3, #2056]
	beq	.L499
.L500:
	mov	r0, r10
	bl	zftl_flash_exit_slc_mode
.L499:
	add	r4, r5, r4
	mov	r0, #0
	uxtb	r3, r7
	str	r0, [r4, #2056]
	str	r0, [r4, #2052]
	str	r0, [r4, #2052]
	str	r3, [r4, #2052]
	mov	r3, r7, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r7, lsr #16
	str	r3, [r4, #2052]
	str	r6, [r4, #2056]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	b	nandc_de_cs
.L506:
	.align	2
.L505:
	.word	.LANCHOR0
	.word	.LANCHOR1+280
	.word	.LC0
	.fnend
	.size	flash_start_page_read, .-flash_start_page_read
	.align	2
	.global	nandc_wait_flash_ready
	.type	nandc_wait_flash_ready, %function
nandc_wait_flash_ready:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, lr}
	.save {r4, r5, lr}
	.pad #12
	mov	r0, #150
	bl	timer_delay_ns
	ldr	r4, .L513
	ldr	r5, .L513+4
.L509:
	ldr	r3, [r5, #528]
	ldr	r3, [r3]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L510
	mov	r0, #10
	bl	timer_delay_ns
	subs	r4, r4, #1
	bne	.L509
	mvn	r0, #0
	b	.L508
.L510:
	mov	r0, #0
.L508:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L514:
	.align	2
.L513:
	.word	100000
	.word	.LANCHOR0
	.fnend
	.size	nandc_wait_flash_ready, .-nandc_wait_flash_ready
	.align	2
	.global	sandisk_set_rr_para
	.type	sandisk_set_rr_para, %function
sandisk_set_rr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r3, #239
	mov	r5, r0
	str	r3, [r0, #8]
	mov	r3, #17
	str	r3, [r0, #4]
	mov	r0, #200
	mov	r4, r1
	bl	timer_delay_ns
	ldr	r0, .L523
	ldr	r1, .L523+4
	add	r4, r4, r4, asl #2
	sub	ip, r0, #48
	mov	r2, #0
.L516:
	ldrb	r3, [r1, #1693]	@ zero_extendqisi2
	cmp	r2, r3
	bcs	.L522
	ldrb	r3, [r1, #1692]	@ zero_extendqisi2
	cmp	r3, #67
	add	r3, r2, r4
	addeq	r3, ip, r3
	addne	r3, r0, r3
	add	r2, r2, #1
	ldrsb	r3, [r3, #5]
	str	r3, [r5]
	b	.L516
.L522:
	ldmfd	sp!, {r3, r4, r5, lr}
	b	nandc_wait_flash_ready
.L524:
	.align	2
.L523:
	.word	.LANCHOR1+128
	.word	.LANCHOR0
	.fnend
	.size	sandisk_set_rr_para, .-sandisk_set_rr_para
	.align	2
	.global	toshiba_3d_set_tlc_rr_para
	.type	toshiba_3d_set_tlc_rr_para, %function
toshiba_3d_set_tlc_rr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	add	r1, r1, #1
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	rsb	r1, r1, r1, asl #3
	ldr	r5, .L527
	mov	r6, #0
	mov	r7, #213
	mvn	r3, #118
	add	r5, r5, r1
	str	r7, [r0, #8]
	str	r6, [r0, #4]
	mov	r4, r0
	str	r3, [r0, #4]
	add	r3, r5, #304
	ldrsb	r3, [r3]
	str	r3, [r0]
	movw	r3, #305
	ldrsb	r3, [r5, r3]
	str	r3, [r0]
	movw	r3, #306
	ldrsb	r3, [r5, r3]
	str	r3, [r0]
	movw	r3, #307
	ldrsb	r3, [r5, r3]
	str	r3, [r0]
	bl	nandc_wait_flash_ready
	str	r7, [r4, #8]
	mvn	r3, #117
	str	r6, [r4, #4]
	str	r3, [r4, #4]
	add	r3, r5, #308
	ldrsb	r3, [r3]
	str	r3, [r4]
	movw	r3, #309
	ldrsb	r3, [r5, r3]
	str	r3, [r4]
	movw	r3, #310
	ldrsb	r3, [r5, r3]
	str	r3, [r4]
	str	r6, [r4]
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	nandc_wait_flash_ready
.L528:
	.align	2
.L527:
	.word	.LANCHOR1
	.fnend
	.size	toshiba_3d_set_tlc_rr_para, .-toshiba_3d_set_tlc_rr_para
	.align	2
	.global	toshiba_3d_set_slc_rr_para
	.type	toshiba_3d_set_slc_rr_para, %function
toshiba_3d_set_slc_rr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, #213
	mvn	r2, #116
	str	r3, [r0, #8]
	mov	r3, #0
	str	r3, [r0, #4]
	str	r2, [r0, #4]
	ldr	r2, .L530
	add	r2, r2, r1
	movw	r1, #705
	ldrsb	r2, [r2, r1]
	str	r2, [r0]
	str	r3, [r0]
	str	r3, [r0]
	str	r3, [r0]
	b	nandc_wait_flash_ready
.L531:
	.align	2
.L530:
	.word	.LANCHOR1
	.fnend
	.size	toshiba_3d_set_slc_rr_para, .-toshiba_3d_set_slc_rr_para
	.align	2
	.global	toshiba_tlc_set_rr_para
	.type	toshiba_tlc_set_rr_para, %function
toshiba_tlc_set_rr_para:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r2, #0
	ldr	r3, .L536
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, #239
	mov	r4, r0
	str	r6, [r0, #8]
	beq	.L533
	rsb	r1, r1, r1, asl #3
	mov	r2, #18
	add	r5, r3, r1
	str	r2, [r0, #4]
	ldrb	r3, [r5, #36]	@ zero_extendqisi2
	str	r3, [r0]
	ldrb	r3, [r5, #37]	@ zero_extendqisi2
	str	r3, [r0]
	ldrb	r3, [r5, #38]	@ zero_extendqisi2
	str	r3, [r0]
	ldrb	r3, [r5, #39]	@ zero_extendqisi2
	str	r3, [r0]
	bl	nandc_wait_flash_ready
	str	r6, [r4, #8]
	mov	r3, #19
	str	r3, [r4, #4]
	ldrb	r3, [r5, #40]	@ zero_extendqisi2
	str	r3, [r4]
	ldrb	r3, [r5, #41]	@ zero_extendqisi2
	str	r3, [r4]
	ldrb	r3, [r5, #42]	@ zero_extendqisi2
	str	r3, [r4]
	mov	r3, #0
	str	r3, [r4]
	b	.L534
.L533:
	add	r3, r3, r1
	mov	r0, #20
	str	r0, [r4, #4]
	ldrb	r3, [r3, #368]	@ zero_extendqisi2
	str	r3, [r4]
	str	r2, [r4]
	str	r2, [r4]
	str	r2, [r4]
.L534:
	ldmfd	sp!, {r4, r5, r6, lr}
	b	nandc_wait_flash_ready
.L537:
	.align	2
.L536:
	.word	.LANCHOR2
	.fnend
	.size	toshiba_tlc_set_rr_para, .-toshiba_tlc_set_rr_para
	.align	2
	.global	flash_start_plane_read
	.type	flash_start_plane_read, %function
flash_start_plane_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r3, #1
	ldr	r8, .L557
	ubfx	r7, r0, #24, #2
	ldrb	r2, [r8, #1700]	@ zero_extendqisi2
	rsb	r5, r2, #24
	mov	r4, r3, asl r5
	mov	r3, r3, asl r2
	sub	r3, r3, #1
	sub	r4, r4, #1
	and	r5, r3, r0, lsr r5
	ldrb	r3, [r8, #585]	@ zero_extendqisi2
	and	r6, r4, r0
	and	r4, r4, r1
	uxtb	r5, r5
	cmp	r3, r5
	bhi	.L539
	ldr	r1, .L557+4
	movw	r2, #965
	ldr	r0, .L557+8
	bl	printk
	bl	dump_stack
.L539:
	add	r5, r8, r5
	ldr	r3, .L557
	ldr	r9, [r8, #528]
	ldrb	r10, [r5, #588]	@ zero_extendqisi2
	str	r3, [sp, #4]
	mov	r0, r10
	mov	fp, r10, asl #8
	bl	nandc_cs
	cmp	r7, #0
	ldr	r3, [sp, #4]
	bne	.L540
	mov	r0, r6
	bl	slc_phy_page_address_calc
	mov	r6, r0
	mov	r0, r4
	bl	slc_phy_page_address_calc
	ldr	r3, [sp, #4]
	mov	r4, r0
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L541
	mov	r0, r10
	bl	zftl_flash_enter_slc_mode
	b	.L541
.L540:
	ldr	r2, [r3, #1620]
	ldrb	r2, [r2, #12]	@ zero_extendqisi2
	cmp	r2, #3
	bne	.L542
	ldrb	r3, [r3, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	addeq	r3, r9, fp
	streq	r7, [r3, #2056]
	beq	.L541
.L542:
	mov	r0, r10
	bl	zftl_flash_exit_slc_mode
.L541:
	ldrb	r3, [r8, #1642]	@ zero_extendqisi2
	uxtb	r0, r6
	mov	r1, r6, lsr #8
	ldr	r2, .L557
	cmp	r3, #1
	ldrb	ip, [r8, #1634]	@ zero_extendqisi2
	mov	r6, r6, lsr #16
	bne	.L543
	ldrb	r3, [r2, #1635]	@ zero_extendqisi2
	add	r5, r9, fp
	mov	r8, #0
	str	r2, [sp, #4]
	str	ip, [r5, #2056]
	add	fp, r9, fp
	str	r8, [r5, #2052]
	str	r8, [r5, #2052]
	str	r0, [r5, #2052]
	str	r1, [r5, #2052]
	str	r6, [r5, #2052]
	str	r3, [r5, #2056]
	bl	nandc_wait_flash_ready
	ldr	r2, [sp, #4]
	ldr	r3, [r2, #1620]
	ldrb	r2, [r3, #12]	@ zero_extendqisi2
	subs	r3, r7, r8
	movne	r3, #1
	cmp	r2, #3
	movne	r3, #0
	cmp	r3, r8
	uxtb	r3, r4
	strne	r7, [r5, #2056]
	str	r8, [fp, #2056]
	str	r8, [r5, #2052]
	str	r8, [r5, #2052]
	str	r3, [r5, #2052]
	mov	r3, r4, lsr #8
	mov	r4, r4, lsr #16
	str	r3, [r5, #2052]
	str	r4, [r5, #2052]
	mov	r3, #48
	str	r3, [fp, #2056]
	b	.L545
.L543:
	ldr	r2, [r2, #1620]
	add	r3, r9, fp
	add	r9, r9, fp
	str	ip, [r3, #2056]
	str	r0, [r3, #2052]
	str	r1, [r3, #2052]
	str	r6, [r3, #2052]
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	adds	r2, r7, #0
	movne	r2, #1
	cmp	r1, #3
	movne	r2, #0
	cmp	r2, #0
	ldrb	r2, [r8, #1635]	@ zero_extendqisi2
	strne	r7, [r3, #2056]
	str	r2, [r9, #2056]
	uxtb	r2, r4
	str	r2, [r3, #2052]
	mov	r2, r4, lsr #8
	mov	r4, r4, lsr #16
	str	r2, [r3, #2052]
	str	r4, [r3, #2052]
	mov	r3, #48
	str	r3, [r9, #2056]
.L545:
	mov	r0, r10
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	nandc_de_cs
.L558:
	.align	2
.L557:
	.word	.LANCHOR0
	.word	.LANCHOR1+716
	.word	.LC0
	.fnend
	.size	flash_start_plane_read, .-flash_start_plane_read
	.align	2
	.global	flash_set_interface_mode
	.type	flash_set_interface_mode, %function
flash_set_interface_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r8, r0
	ldr	r6, .L592
	mov	r9, #0
	ldr	r10, .L592+4
	mov	r7, r6
.L571:
	ldr	r3, .L592+8
	ldr	r5, [r6, #528]
	ldrb	r4, [r3, r9, asl #3]	@ zero_extendqisi2
	cmp	r4, #69
	cmpne	r4, #152
	beq	.L560
	cmp	r4, #137
	cmpne	r4, #44
	moveq	r3, #1
	movne	r3, #0
	cmp	r4, #155
	orreq	r3, r3, #1
	cmp	r3, #0
	beq	.L561
.L560:
	cmp	r8, #1
	ldrb	r1, [r7, #1779]	@ zero_extendqisi2
	bne	.L562
	tst	r1, #1
	beq	.L561
	ldr	r3, [r10]
	tst	r3, #4096
	beq	.L563
	ldr	r0, .L592+12
	bl	printk
.L563:
	mov	r2, r9, asl #8
	cmp	r4, #44
	cmpne	r4, #137
	add	r3, r5, r2
	mov	r1, #239
	str	r1, [r3, #2056]
	moveq	r1, #1
	streq	r1, [r3, #2052]
	moveq	r1, #5
	beq	.L590
	cmp	r4, #155
	moveq	r1, #1
	movne	r1, #128
	streq	r1, [r3, #2052]
	moveq	r1, #3
	strne	r1, [r3, #2052]
	movne	r1, #1
	b	.L590
.L562:
	tst	r1, #4
	beq	.L561
	ldr	r3, [r10]
	tst	r3, #4096
	beq	.L567
	ldr	r0, .L592+16
	bl	printk
.L567:
	mov	r2, r9, asl #8
	cmp	r4, #44
	cmpne	r4, #137
	add	r3, r5, r2
	mov	r1, #239
	str	r1, [r3, #2056]
	moveq	r1, #1
	movne	r1, #0
	moveq	r1, #1
	streq	r1, [r3, #2052]
	moveq	r1, #35
	beq	.L590
	cmp	r4, #155
	moveq	r1, #1
	streq	r1, [r3, #2052]
	moveq	r1, #37
	movne	r0, #128
	strne	r0, [r3, #2052]
.L590:
	add	r5, r5, r2
	str	r1, [r3, #2048]
	mov	r3, #0
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
.L561:
	add	r9, r9, #1
	cmp	r9, #4
	bne	.L571
	bl	nandc_wait_flash_ready
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L593:
	.align	2
.L592:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+1744
	.word	.LC38
	.word	.LC39
	.fnend
	.size	flash_set_interface_mode, .-flash_set_interface_mode
	.align	2
	.global	flash_reset
	.type	flash_reset, %function
flash_reset:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L595
	mov	r2, #255
	ldr	r3, [r3, #528]
	add	r0, r3, r0, asl #8
	str	r2, [r0, #2056]
	b	nandc_wait_flash_ready
.L596:
	.align	2
.L595:
	.word	.LANCHOR0
	.fnend
	.size	flash_reset, .-flash_reset
	.align	2
	.global	flash_read_id
	.type	flash_read_id, %function
flash_read_id:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #16
	mov	r6, r0
	ldr	r3, .L600
	mov	r4, r1
	ldr	r5, [r3, #528]
	bl	flash_reset
	mov	r0, r6
	add	r5, r5, r6, asl #8
	bl	nandc_cs
	mov	r3, #144
	mov	r0, #200
	str	r3, [r5, #2056]
	mov	r3, #0
	str	r3, [r5, #2052]
	bl	timer_delay_ns
	ldr	r3, [r5, #2048]
	mov	r0, r6
	strb	r3, [r4]
	ldr	r3, [r5, #2048]
	strb	r3, [r4, #1]
	ldr	r3, [r5, #2048]
	strb	r3, [r4, #2]
	ldr	r3, [r5, #2048]
	strb	r3, [r4, #3]
	ldr	r3, [r5, #2048]
	strb	r3, [r4, #4]
	ldr	r3, [r5, #2048]
	strb	r3, [r4, #5]
	ldr	r3, [r5, #2048]
	strb	r3, [r4, #6]
	ldr	r3, [r5, #2048]
	strb	r3, [r4, #7]
	bl	nandc_de_cs
	ldrb	r2, [r4]	@ zero_extendqisi2
	sub	r3, r2, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L597
	ldrb	r1, [r4, #2]	@ zero_extendqisi2
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	ldr	r0, .L600+4
	str	r1, [sp]
	ldrb	r1, [r4, #3]	@ zero_extendqisi2
	str	r1, [sp, #4]
	ldrb	r1, [r4, #4]	@ zero_extendqisi2
	str	r1, [sp, #8]
	ldrb	r1, [r4, #5]	@ zero_extendqisi2
	str	r1, [sp, #12]
	add	r1, r6, #1
	bl	printk
.L597:
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L601:
	.align	2
.L600:
	.word	.LANCHOR0
	.word	.LC40
	.fnend
	.size	flash_read_id, .-flash_read_id
	.align	2
	.global	flash_read_spare
	.type	flash_read_spare, %function
flash_read_spare:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r2
	ldr	r3, .L604
	ldr	r2, .L604+4
	ldrb	r3, [r3, #13]	@ zero_extendqisi2
	ldr	r4, [r2, #528]
	mov	r2, #0
	add	r4, r4, r0, asl #8
	mov	r3, r3, asl #9
	str	r2, [r4, #2056]
	str	r3, [r4, #2052]
	mov	r3, r3, lsr #8
	str	r3, [r4, #2052]
	uxtb	r3, r1
	str	r3, [r4, #2052]
	mov	r3, r1, lsr #8
	mov	r1, r1, lsr #16
	str	r3, [r4, #2052]
	mov	r3, #48
	str	r1, [r4, #2052]
	str	r3, [r4, #2056]
	bl	nandc_wait_flash_ready
	ldr	r3, [r4, #2048]
	strb	r3, [r5]
	ldmfd	sp!, {r3, r4, r5, pc}
.L605:
	.align	2
.L604:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	flash_read_spare, .-flash_read_spare
	.align	2
	.global	flash_read_otp_data
	.type	flash_read_otp_data, %function
flash_read_otp_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L610
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r8, r0
	ldr	r7, [r3, #528]
	mov	r10, r8, asl #8
	mov	r5, r1
	mov	r9, r2
	add	r4, r7, r10
	bl	nandc_cs
	mov	fp, #144
	mov	r3, #239
	mov	r0, #50
	str	r3, [r4, #2056]
	str	fp, [r4, #2052]
	mov	r6, #0
	bl	timer_delay_ns
	mov	r3, #1
	str	r3, [r4, #2048]
	str	r6, [r4, #2048]
	str	r6, [r4, #2048]
	str	r6, [r4, #2048]
	bl	nandc_wait_flash_ready
	mov	r3, #238
	str	r3, [r4, #2056]
	str	fp, [r4, #2052]
	ldr	r2, [r4, #2048]
	ldr	r3, [r4, #2048]
	ldr	r1, [r4, #2048]
	ldr	r0, .L610+4
	str	r1, [sp]
	mov	r1, r5
	bl	printk
	bl	nandc_wait_flash_ready
	str	r6, [r4, #2056]
	uxtb	r3, r5
	str	r6, [r4, #2052]
	str	r6, [r4, #2052]
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #8
	mov	r5, r5, lsr #16
	str	r3, [r4, #2052]
	mov	r3, #48
	str	r5, [r4, #2052]
	str	r3, [r4, #2056]
	bl	nandc_wait_flash_ready
	add	r3, r9, #16384
.L607:
	ldr	r2, [r4, #2048]
	strb	r2, [r9], #1
	cmp	r9, r3
	bne	.L607
	add	r7, r7, r10
	mov	r3, #239
	mov	r0, #50
	str	r3, [r7, #2056]
	mov	r3, #144
	str	r3, [r4, #2052]
	bl	timer_delay_ns
	mov	r3, #0
	mov	r0, r8
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	str	r3, [r4, #2048]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	nandc_de_cs
.L611:
	.align	2
.L610:
	.word	.LANCHOR0
	.word	.LC41
	.fnend
	.size	flash_read_otp_data, .-flash_read_otp_data
	.align	2
	.global	sandisk_prog_test_bad_block
	.type	sandisk_prog_test_bad_block, %function
sandisk_prog_test_bad_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L623
	mov	r0, r0, asl #8
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r1
	ldr	r4, [r3, #528]
	ldr	r3, .L623+4
	ldrb	r2, [r3, #33]	@ zero_extendqisi2
	mov	r6, r3
	cmp	r2, #0
	addne	r3, r4, r0
	addeq	r2, r4, r0
	add	r4, r4, r0
	moveq	r3, #162
	strne	r2, [r3, #2056]
	streq	r3, [r2, #2056]
	mov	r2, #0
	mov	r3, #128
	str	r3, [r4, #2056]
	str	r2, [r4, #2052]
	uxtb	r3, r1
	str	r2, [r4, #2052]
	str	r3, [r4, #2052]
	mov	r3, r1, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r1, lsr #16
	str	r3, [r4, #2052]
	mov	r3, #16
	str	r3, [r4, #2056]
	bl	nandc_wait_flash_ready
	mov	r3, #112
	mov	r0, #80
	str	r3, [r4, #2056]
	bl	timer_delay_ns
	ldr	r2, [r4, #2048]
	ands	r4, r2, #5
	beq	.L615
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L615
	ldr	r0, .L623+8
	mov	r1, r5
	bl	printk
.L615:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L624:
	.align	2
.L623:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC42
	.fnend
	.size	sandisk_prog_test_bad_block, .-sandisk_prog_test_bad_block
	.align	2
	.global	nandc_rdy_status
	.type	nandc_rdy_status, %function
nandc_rdy_status:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L627
	.pad #8
	sub	sp, sp, #8
	ldr	r3, [r3, #528]
	ldr	r3, [r3]
	str	r3, [sp, #4]
	ldr	r0, [sp, #4]
	ubfx	r0, r0, #9, #1
	add	sp, sp, #8
	@ sp needed
	bx	lr
.L628:
	.align	2
.L627:
	.word	.LANCHOR0
	.fnend
	.size	nandc_rdy_status, .-nandc_rdy_status
	.align	2
	.global	nandc_bch_sel
	.type	nandc_bch_sel, %function
nandc_bch_sel:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #8
	sub	sp, sp, #8
	mov	r3, #0
	str	r3, [sp, #4]
	ldr	r2, [sp, #4]
	orr	r2, r2, #1
	str	r2, [sp, #4]
	ldr	r2, .L642
	ldrb	r1, [r2, #516]	@ zero_extendqisi2
	strb	r0, [r2, #584]
	cmp	r1, #9
	bne	.L630
	ldr	r1, [r2, #528]
	cmp	r0, #70
	ldr	r2, [sp, #4]
	str	r2, [r1, #16]
	beq	.L631
	cmp	r0, #60
	moveq	r3, #3
	beq	.L631
	cmp	r0, #40
	moveq	r3, #2
	movne	r3, #1
.L631:
	mov	r2, #0
	str	r2, [sp]
	ldr	r2, [sp]
	bic	r2, r2, #234881024
	orr	r3, r2, r3, asl #25
	str	r3, [sp]
	ldr	r3, [sp]
	orr	r3, r3, #1
	str	r3, [sp]
	ldr	r3, [sp]
	str	r3, [r1, #32]
	b	.L629
.L630:
	ldr	r1, [sp, #4]
	cmp	r0, #16
	ldr	r2, [r2, #528]
	str	r1, [r2, #8]
	str	r3, [sp]
	ldr	r3, [sp]
	bic	r3, r3, #65280
	orr	r3, r3, #4096
	str	r3, [sp]
	ldr	r3, [sp]
	bic	r3, r3, #262144
	str	r3, [sp]
	beq	.L640
	ldr	r3, [sp]
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L641
	orr	r3, r3, #262144
	str	r3, [sp]
	ldr	r3, [sp]
	cmp	r0, #40
	orr	r3, r3, #16
	str	r3, [sp]
	bne	.L634
.L640:
	ldr	r3, [sp]
	bic	r3, r3, #16
.L641:
	str	r3, [sp]
.L634:
	ldr	r3, [sp]
	orr	r3, r3, #1
	str	r3, [sp]
	ldr	r3, [sp]
	str	r3, [r2, #12]
.L629:
	add	sp, sp, #8
	@ sp needed
	bx	lr
.L643:
	.align	2
.L642:
	.word	.LANCHOR0
	.fnend
	.size	nandc_bch_sel, .-nandc_bch_sel
	.align	2
	.global	zftl_flash_resume
	.type	zftl_flash_resume, %function
zftl_flash_resume:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	ldr	r4, .L655
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	ldr	r2, [r4, #532]
	cmp	r3, #9
	ldr	r3, [r4, #528]
	str	r2, [r3]
	ldr	r2, [r4, #536]
	ldr	r3, [r4, #528]
	str	r2, [r3, #4]
	ldr	r2, [r4, #540]
	bne	.L645
	str	r2, [r3, #16]
	ldr	r2, [r4, #544]
	str	r2, [r3, #32]
	ldr	r2, [r4, #548]
	str	r2, [r3, #80]
	ldr	r2, [r4, #552]
	str	r2, [r3, #84]
	ldr	r2, [r4, #556]
	str	r2, [r3, #520]
	ldr	r2, [r4, #560]
	str	r2, [r3, #8]
	b	.L646
.L645:
	str	r2, [r3, #8]
	ldr	r2, [r4, #544]
	str	r2, [r3, #12]
	ldr	r2, [r4, #548]
	str	r2, [r3, #304]
	ldr	r2, [r4, #552]
	str	r2, [r3, #308]
	ldr	r2, [r4, #556]
	str	r2, [r3, #336]
	ldr	r2, [r4, #560]
	str	r2, [r3, #344]
.L646:
	ldr	r6, .L655+4
	mov	r5, #0
	mov	r8, #2
	sub	r7, r6, #40
.L648:
	ldrb	r3, [r6, r5, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L647
	uxtb	r9, r5
	mov	r0, r9
	bl	flash_reset
	strb	r8, [r5, r7]
	mov	r0, r9
	bl	zftl_flash_enter_slc_mode
.L647:
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L648
	ldrb	r3, [r4, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L649
	mov	r0, #1
	bl	nandc_set_if_mode
	mov	r0, r5
	bl	flash_set_interface_mode
	mov	r0, r5
	bl	nandc_set_if_mode
	ldr	r3, .L655
	ldrb	r0, [r3, #549]	@ zero_extendqisi2
	bl	nandc_set_ddr_para
.L649:
	ldr	r3, .L655+8
	ldrb	r0, [r3, #24]	@ zero_extendqisi2
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	b	nandc_bch_sel
.L656:
	.align	2
.L655:
	.word	.LANCHOR0
	.word	.LANCHOR0+1744
	.word	.LANCHOR2
	.fnend
	.size	zftl_flash_resume, .-zftl_flash_resume
	.align	2
	.global	rk_nandc_flash_ready
	.type	rk_nandc_flash_ready, %function
rk_nandc_flash_ready:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L660
	ldrb	r3, [r3, #516]	@ zero_extendqisi2
	cmp	r3, #9
	ldreq	r3, [r0, #292]
	ldrne	r3, [r0, #368]
	orreq	r3, r3, #2
	orrne	r3, r3, #2
	streq	r3, [r0, #292]
	strne	r3, [r0, #368]
	ldreq	r3, [r0, #288]
	ldrne	r3, [r0, #364]
	biceq	r3, r3, #2
	bicne	r3, r3, #2
	streq	r3, [r0, #288]
	strne	r3, [r0, #364]
	bx	lr
.L661:
	.align	2
.L660:
	.word	.LANCHOR0
	.fnend
	.size	rk_nandc_flash_ready, .-rk_nandc_flash_ready
	.align	2
	.global	nandc_iqr_wait_flash_ready
	.type	nandc_iqr_wait_flash_ready, %function
nandc_iqr_wait_flash_ready:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, .L669
	ldr	r0, [r4, #528]
	bl	rk_nandc_rb_irq_flag_init
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	ldr	r0, [r4, #528]
	cmp	r3, #9
	ldreq	r3, [r0, #292]
	ldrne	r3, [r0, #368]
	orreq	r3, r3, #2
	orrne	r3, r3, #2
	streq	r3, [r0, #292]
	strne	r3, [r0, #368]
	ldreq	r3, [r0, #288]
	ldrne	r3, [r0, #364]
	orreq	r3, r3, #2
	orrne	r3, r3, #2
	streq	r3, [r0, #288]
	strne	r3, [r0, #364]
	ldr	r3, [r0]
	tst	r3, #512
	bne	.L665
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L665:
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #9
	ldreq	r3, [r0, #292]
	ldrne	r3, [r0, #368]
	orreq	r3, r3, #2
	orrne	r3, r3, #2
	streq	r3, [r0, #292]
	strne	r3, [r0, #368]
	ldreq	r3, [r0, #288]
	ldrne	r3, [r0, #364]
	biceq	r3, r3, #2
	bicne	r3, r3, #2
	streq	r3, [r0, #288]
	strne	r3, [r0, #364]
	ldmfd	sp!, {r4, pc}
.L670:
	.align	2
.L669:
	.word	.LANCHOR0
	.fnend
	.size	nandc_iqr_wait_flash_ready, .-nandc_iqr_wait_flash_ready
	.align	2
	.global	flash_erase_duplane_block
	.type	flash_erase_duplane_block, %function
flash_erase_duplane_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r3
	ldr	r4, .L688
	mov	r5, r0
	mov	fp, r1
	mov	r9, r2
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L672
	ldr	r1, .L688+4
	movw	r2, #598
	ldr	r0, .L688+8
	bl	printk
	bl	dump_stack
.L672:
	ldrb	r2, [r4, #585]	@ zero_extendqisi2
	ldr	r3, .L688
	cmp	r2, r5
	bls	.L679
	add	r5, r3, r5
	ldr	r7, [r3, #528]
	ldr	r3, .L688+12
	ldrb	r6, [r5, #588]	@ zero_extendqisi2
	ldr	r3, [r3]
	add	r10, r6, #8
	tst	r3, #16
	add	r10, r7, r10, asl #8
	beq	.L674
	ldr	r0, .L688+16
	mov	r1, r6
	mov	r2, r9
	mov	r3, r8
	bl	printk
.L674:
	bl	nandc_wait_flash_ready
	mov	r0, r6
	bl	nandc_cs
	cmp	fp, #0
	mov	r0, r6
	bne	.L675
	bl	zftl_flash_enter_slc_mode
	b	.L676
.L675:
	bl	zftl_flash_exit_slc_mode
.L676:
	mov	fp, r6, asl #8
	mov	r3, #96
	add	r5, r7, fp
	str	r3, [r5, #2056]
	uxtb	r3, r9
	str	r3, [r5, #2052]
	mov	r3, r9, lsr #8
	str	r3, [r5, #2052]
	mov	r3, r9, lsr #16
	str	r3, [r5, #2052]
	ldrb	r3, [r4, #1780]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r4, #0
	bne	.L677
	mov	r3, #208
	str	r3, [r5, #2056]
	bl	nandc_wait_flash_ready
	mov	r0, r10
	bl	flash_read_status
	and	r4, r0, #5
.L677:
	add	r7, r7, fp
	mov	r3, #96
	str	r3, [r7, #2056]
	uxtb	r3, r8
	str	r3, [r5, #2052]
	mov	r3, r8, lsr #8
	mov	r8, r8, lsr #16
	str	r3, [r5, #2052]
	mov	r3, #208
	str	r8, [r5, #2052]
	str	r3, [r7, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	mov	r0, r10
	bl	flash_read_status
	mov	r5, r0
	mov	r0, r6
	bl	nandc_de_cs
	and	r3, r5, #5
	orrs	r4, r3, r4
	beq	.L678
	ldr	r0, .L688+20
	mov	r1, r9
	mov	r2, r5
	bl	printk
.L678:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L679:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L689:
	.align	2
.L688:
	.word	.LANCHOR0
	.word	.LANCHOR1+740
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC43
	.word	.LC44
	.fnend
	.size	flash_erase_duplane_block, .-flash_erase_duplane_block
	.align	2
	.global	flash_erase_block_en
	.type	flash_erase_block_en, %function
flash_erase_block_en:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mvn	r9, #0
	ldr	r6, .L705
	mov	r7, r0
	mov	fp, r1
	mov	r8, r2
	ldrb	r3, [r6, #1700]	@ zero_extendqisi2
	rsb	r3, r3, #24
	bic	r9, r2, r9, asl r3
	ldrb	r3, [r6, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L691
	ldr	r1, .L705+4
	movw	r2, #645
	ldr	r0, .L705+8
	bl	printk
	bl	dump_stack
.L691:
	ldrb	r2, [r6, #585]	@ zero_extendqisi2
	ldr	r3, .L705
	cmp	r2, r7
	bls	.L697
	add	r2, r3, r7
	ldr	r4, [r3, #528]
	ldr	r3, .L705+12
	ldrb	r5, [r2, #588]	@ zero_extendqisi2
	ldr	r3, [r3]
	add	r10, r5, #8
	tst	r3, #16
	add	r10, r4, r10, asl #8
	beq	.L693
	ldr	r0, .L705+16
	mov	r1, r5
	mov	r2, r8
	mov	r3, fp
	bl	printk
.L693:
	bl	nandc_wait_flash_ready
	mov	r0, r5
	bl	nandc_cs
	cmp	fp, #0
	mov	r0, r5
	bne	.L694
	bl	zftl_flash_enter_slc_mode
	b	.L695
.L694:
	bl	zftl_flash_exit_slc_mode
.L695:
	add	r4, r4, r5, asl #8
	mov	r3, #96
	str	r3, [r4, #2056]
	uxtb	r3, r9
	str	r3, [r4, #2052]
	mov	r3, r9, lsr #8
	mov	r9, r9, lsr #16
	str	r3, [r4, #2052]
	mov	r3, #208
	str	r9, [r4, #2052]
	str	r3, [r4, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	mov	r0, r10
	bl	flash_read_status
	mov	r4, r0
	mov	r0, r5
	bl	nandc_de_cs
	ands	r4, r4, #5
	beq	.L696
	ldrh	r1, [r6, #2]
	mov	r0, r8
	bl	__aeabi_uidiv
	mov	r1, r7
	mov	r3, r4
	mov	r2, r0
	ldr	r0, .L705+20
	bl	printk
.L696:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L697:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L706:
	.align	2
.L705:
	.word	.LANCHOR0
	.word	.LANCHOR1+768
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC45
	.word	.LC46
	.fnend
	.size	flash_erase_block_en, .-flash_erase_block_en
	.align	2
	.global	flash_erase_block
	.type	flash_erase_block, %function
flash_erase_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r2, r1
	mov	r1, #0
	b	flash_erase_block_en
	.fnend
	.size	flash_erase_block, .-flash_erase_block
	.align	2
	.global	flash_erase_all
	.type	flash_erase_all, %function
flash_erase_all:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r6, #0
	ldr	r3, .L716
	ldr	r7, .L716+4
	ldrb	r4, [r3, #17]	@ zero_extendqisi2
	ldrh	r3, [r3, #18]
	mov	r8, r7
	smulbb	r4, r4, r3
	uxth	r4, r4
.L709:
	ldrb	r2, [r7, #585]	@ zero_extendqisi2
	uxth	r3, r6
	cmp	r2, r3
	bls	.L714
	add	r3, r7, r3
	mov	r5, #0
	ldrb	r9, [r3, #588]	@ zero_extendqisi2
.L710:
	uxth	r3, r5
	cmp	r3, r4
	bcs	.L715
	ldrh	r1, [r8, #2]
	mov	r0, r9
	mul	r1, r1, r5
	bl	flash_erase_block
	add	r5, r5, #1
	b	.L710
.L715:
	add	r6, r6, #1
	b	.L709
.L714:
	ldr	r0, .L716+8
	mov	r1, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L717:
	.align	2
.L716:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC47
	.fnend
	.size	flash_erase_all, .-flash_erase_all
	.align	2
	.global	rk_nandc_flash_xfer_completed
	.type	rk_nandc_flash_xfer_completed, %function
rk_nandc_flash_xfer_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L721
	ldrb	r3, [r3, #516]	@ zero_extendqisi2
	cmp	r3, #9
	ldreq	r3, [r0, #292]
	ldrne	r3, [r0, #368]
	orreq	r3, r3, #1
	orrne	r3, r3, #1
	streq	r3, [r0, #292]
	strne	r3, [r0, #368]
	ldreq	r3, [r0, #288]
	ldrne	r3, [r0, #364]
	biceq	r3, r3, #1
	bicne	r3, r3, #1
	streq	r3, [r0, #288]
	strne	r3, [r0, #364]
	bx	lr
.L722:
	.align	2
.L721:
	.word	.LANCHOR0
	.fnend
	.size	rk_nandc_flash_xfer_completed, .-rk_nandc_flash_xfer_completed
	.align	2
	.global	nandc_xfer_start
	.type	nandc_xfer_start, %function
nandc_xfer_start:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r5, r0
	ldr	r4, .L741
	.pad #20
	sub	sp, sp, #20
	mov	r8, r1
	mov	r6, r3
	mov	r7, r2
	ldr	r0, [r4, #528]
	bl	rk_nandc_xfer_irq_flag_init
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	ldr	r0, [r4, #528]
	and	r1, r5, #1
	cmp	r3, #9
	add	r3, r8, #1
	bne	.L724
	ldr	r2, [r0, #292]
	ubfx	r3, r3, #1, #6
	orr	r2, r2, #1
	str	r2, [r0, #292]
	ldr	r2, [r0, #288]
	orr	r2, r2, #1
	str	r2, [r0, #288]
	mov	r2, #0
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	bic	r2, r2, #2
	orr	r2, r2, r1, asl #1
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	orr	r2, r2, #8
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	bic	r2, r2, #96
	orr	r2, r2, #32
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	orr	r2, r2, #536870912
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	orr	r2, r2, #1024
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	bic	r2, r2, #16
	str	r2, [sp, #4]
	ldr	r0, [sp, #4]
	bic	r0, r0, #264241152
	orr	r3, r0, r3, asl #22
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	orr	r3, r3, #128
	str	r3, [sp, #4]
	ldrb	r3, [r4, #1659]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L726
	ldrb	r3, [r4, #1688]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, [sp, #4]
	orrne	r3, r3, #512
	strne	r3, [sp, #4]
	b	.L726
.L724:
	ldr	r2, [r0, #368]
	ubfx	r3, r3, #1, #6
	orr	r2, r2, #1
	str	r2, [r0, #368]
	ldr	r2, [r0, #364]
	orr	r2, r2, #1
	str	r2, [r0, #364]
	ldr	r2, [r0, #12]
	mov	r0, #0
	cmp	r5, r0
	str	r2, [sp, #12]
	ldr	r2, [sp, #12]
	bic	r2, r2, #65280
	orr	r2, r2, #4096
	str	r2, [sp, #12]
	ldr	r2, [sp, #12]
	bic	r2, r2, #8
	str	r2, [sp, #12]
	ldr	r2, [sp, #12]
	bic	r2, r2, #224
	str	r2, [sp, #12]
	str	r0, [sp, #4]
	ldr	r2, [sp, #4]
	bic	r2, r2, #2
	orr	r2, r2, r1, asl #1
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	orr	r2, r2, #8
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	bic	r2, r2, #96
	orr	r2, r2, #32
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	orr	r2, r2, #536870912
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	orr	r2, r2, #1024
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	bic	r2, r2, #16
	str	r2, [sp, #4]
	ldr	ip, [sp, #4]
	bic	ip, ip, #264241152
	orr	r3, ip, r3, asl #22
	str	r3, [sp, #4]
	beq	.L740
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	mov	r8, r8, lsr #1
	cmp	r3, #25
	mov	r3, r0
	movcc	r1, #64
	movcs	r1, #128
	b	.L731
.L726:
	ldr	r1, [sp, #4]
	mov	r2, r5
	mov	r0, r7
	str	r7, [r4, #1664]
	ubfx	r1, r1, #22, #6
	str	r6, [r4, #1668]
	mov	r1, r1, asl #10
	bl	rknand_dma_map_single
	mov	r2, r5
	clz	r5, r5
	mov	r5, r5, lsr #5
	ldr	r1, [sp, #4]
	str	r0, [r4, #1672]
	ubfx	r1, r1, #22, #6
	ldr	r0, [r4, #1668]
	mov	r1, r1, asl #2
	bl	rknand_dma_map_single
	ldr	r2, [r4, #528]
	mov	r3, #1
	str	r3, [r4, #1680]
	ldr	r3, [r4, #1672]
	str	r3, [r2, #52]
	str	r0, [r2, #56]
	ldr	r3, [r2, #48]
	str	r0, [r4, #1676]
	str	r3, [sp, #8]
	ldr	r3, [sp, #8]
	bic	r3, r3, #15872
	orr	r3, r3, #8192
	str	r3, [sp, #8]
	ldr	r3, [sp, #8]
	orr	r3, r3, #448
	str	r3, [sp, #8]
	ldr	r3, [sp, #8]
	bic	r3, r3, #56
	orr	r3, r3, #16
	str	r3, [sp, #8]
	ldr	r3, [sp, #8]
	orr	r3, r3, #4
	str	r3, [sp, #8]
	ldr	r3, [sp, #8]
	bic	r3, r3, #2
	orr	r5, r3, r5, asl #1
	str	r5, [sp, #8]
	ldr	r3, [sp, #8]
	orr	r3, r3, #1
	str	r3, [sp, #8]
	movw	r3, #1690
	ldrh	r1, [r4, r3]
	ldr	r3, [sp, #8]
	ubfx	r1, r1, #0, #11
	bic	r3, r3, #133169152
	bic	r3, r3, #983040
	orr	r3, r3, r1, asl #16
	str	r3, [sp, #8]
	ldr	r3, [sp, #8]
	str	r3, [r2, #48]
	ldr	r3, [sp, #4]
	str	r3, [r2, #16]
	ldr	r3, [sp, #4]
	orr	r3, r3, #4
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	str	r3, [r2, #16]
	b	.L723
.L731:
	cmp	r3, r8
	add	r6, r6, #4
	add	lr, r0, r1
	bge	.L733
	ldrh	ip, [r6, #-2]
	mov	r0, r0, lsr #2
	ldr	r9, [r4, #1660]
	add	r3, r3, #1
	ldrh	r2, [r6, #-4]
	orr	ip, r2, ip, asl #16
	str	ip, [r9, r0, asl #2]
	mov	r0, lr
	b	.L731
.L740:
	ldr	r3, [r4, #1660]
	mov	r2, #1
	str	r2, [r3]
.L733:
	ldr	r1, [sp, #4]
	mov	r2, r5
	ldr	r3, [r4, #1660]
	mov	r0, r7
	ubfx	r1, r1, #22, #6
	str	r7, [r4, #1664]
	mov	r1, r1, asl #10
	str	r3, [r4, #1668]
	bl	rknand_dma_map_single
	mov	r2, r5
	clz	r5, r5
	mov	r5, r5, lsr #5
	ldr	r1, [sp, #4]
	str	r0, [r4, #1672]
	ubfx	r1, r1, #22, #6
	ldr	r0, [r4, #1668]
	mov	r1, r1, asl #7
	bl	rknand_dma_map_single
	ldr	r2, [r4, #1672]
	mov	r3, #1
	str	r3, [r4, #1680]
	ldr	r3, [r4, #528]
	str	r2, [r3, #20]
	mov	r2, #0
	str	r0, [r3, #24]
	str	r2, [sp, #8]
	ldr	r2, [sp, #8]
	str	r0, [r4, #1676]
	bic	r2, r2, #15872
	orr	r2, r2, #8192
	str	r2, [sp, #8]
	ldr	r2, [sp, #8]
	orr	r2, r2, #448
	str	r2, [sp, #8]
	ldr	r2, [sp, #8]
	bic	r2, r2, #56
	orr	r2, r2, #16
	str	r2, [sp, #8]
	ldr	r2, [sp, #8]
	orr	r2, r2, #4
	str	r2, [sp, #8]
	ldr	r2, [sp, #8]
	bic	r2, r2, #2
	orr	r5, r2, r5, asl #1
	str	r5, [sp, #8]
	ldr	r2, [sp, #8]
	orr	r2, r2, #1
	str	r2, [sp, #8]
	ldr	r2, [sp, #8]
	str	r2, [r3, #16]
	ldr	r2, [sp, #12]
	str	r2, [r3, #12]
	ldr	r2, [sp, #4]
	str	r2, [r3, #8]
	ldr	r2, [sp, #4]
	orr	r2, r2, #4
	str	r2, [sp, #4]
	ldr	r2, [sp, #4]
	str	r2, [r3, #8]
.L723:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L742:
	.align	2
.L741:
	.word	.LANCHOR0
	.fnend
	.size	nandc_xfer_start, .-nandc_xfer_start
	.align	2
	.global	nandc_set_seed
	.type	nandc_set_seed, %function
nandc_set_seed:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L750
	and	r0, r0, #127
	ldr	r3, .L750+4
	mov	r0, r0, asl #1
	ldrb	r1, [r2, #1688]	@ zero_extendqisi2
	ldrh	r3, [r3, r0]
	cmp	r1, #0
	ldr	r1, .L750
	ldrb	r2, [r2, #516]	@ zero_extendqisi2
	orrne	r3, r3, #-1073741824
	cmp	r2, #9
	ldr	r2, [r1, #528]
	streq	r3, [r2, #520]
	strne	r3, [r2, #336]
	bx	lr
.L751:
	.align	2
.L750:
	.word	.LANCHOR0
	.word	.LANCHOR1+792
	.fnend
	.size	nandc_set_seed, .-nandc_set_seed
	.align	2
	.global	zftl_flash_de_init
	.type	zftl_flash_de_init, %function
zftl_flash_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	bl	nandc_wait_flash_ready
	ldr	r4, .L772
	ldrb	r3, [r4]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L753
	ldrb	r0, [r4, #1625]	@ zero_extendqisi2
	cmp	r0, #0
	bne	.L753
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #9
	beq	.L753
	bl	zftl_flash_exit_slc_mode
.L753:
	mov	r0, #0
	ldr	r5, .L772
	bl	hynix_reconfig_rr_para
	ldrb	r3, [r4, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L754
	ldrb	r3, [r5, #1779]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L754
	mov	r0, #1
	bl	flash_set_interface_mode
	mov	r0, #1
	bl	nandc_set_if_mode
	mov	r3, #0
	strb	r3, [r5, #1658]
.L754:
	ldrb	r3, [r4, #1688]	@ zero_extendqisi2
	ldr	r5, .L772
	cmp	r3, #0
	beq	.L755
	mov	r0, #0
	strb	r0, [r5, #1688]
	bl	nandc_set_seed
	mov	r3, #1
	strb	r3, [r5, #1688]
.L755:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L773:
	.align	2
.L772:
	.word	.LANCHOR0
	.fnend
	.size	zftl_flash_de_init, .-zftl_flash_de_init
	.align	2
	.global	nandc_randomizer_enable
	.type	nandc_randomizer_enable, %function
nandc_randomizer_enable:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L775
	strb	r0, [r3, #1688]
	bx	lr
.L776:
	.align	2
.L775:
	.word	.LANCHOR0
	.fnend
	.size	nandc_randomizer_enable, .-nandc_randomizer_enable
	.align	2
	.global	nandc_get_chip_if
	.type	nandc_get_chip_if, %function
nandc_get_chip_if:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L778
	add	r0, r0, #8
	ldr	r3, [r3, #528]
	add	r0, r3, r0, asl #8
	bx	lr
.L779:
	.align	2
.L778:
	.word	.LANCHOR0
	.fnend
	.size	nandc_get_chip_if, .-nandc_get_chip_if
	.align	2
	.global	zbuf_init
	.type	zbuf_init, %function
zbuf_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r5, #0
	ldr	r4, .L784
	mov	r7, r5
	ldr	r8, .L784+4
.L781:
	ldrb	r0, [r8, #3320]	@ zero_extendqisi2
	uxtb	r3, r5
	add	r2, r3, #1
	strb	r3, [r4, #1]
	strb	r2, [r4]
	add	r5, r5, #1
	mov	r0, r0, asl #9
	strb	r7, [r4, #2]
	str	r7, [r4, #8]
	bl	ftl_malloc
	ldr	r6, .L784+4
	add	r4, r4, #48
	str	r0, [r4, #-44]
	mov	r0, #64
	bl	ftl_malloc
	cmp	r5, #32
	str	r0, [r4, #-36]
	bne	.L781
	mvn	r3, #0
	strb	r5, [r6, #3322]
	strb	r3, [r6, #3272]
	mov	r3, #0
	strb	r3, [r6, #3321]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L785:
	.align	2
.L784:
	.word	.LANCHOR0+1784
	.word	.LANCHOR0
	.fnend
	.size	zbuf_init, .-zbuf_init
	.align	2
	.global	buf_reinit
	.type	buf_reinit, %function
buf_reinit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L789
	mov	r3, #0
	mov	r1, r3
.L787:
	uxtb	r0, r3
	add	r3, r3, #1
	cmp	r3, #32
	add	ip, r0, #1
	strb	r0, [r2, #1]
	add	r2, r2, #48
	strb	ip, [r2, #-48]
	strb	r1, [r2, #-46]
	str	r1, [r2, #-40]
	bne	.L787
	ldr	r2, .L789+4
	mvn	r1, #0
	strb	r3, [r2, #3322]
	strb	r1, [r2, #3272]
	mov	r1, #0
	strb	r1, [r2, #3321]
	bx	lr
.L790:
	.align	2
.L789:
	.word	.LANCHOR0+1784
	.word	.LANCHOR0
	.fnend
	.size	buf_reinit, .-buf_reinit
	.align	2
	.global	buf_add_tail
	.type	buf_add_tail, %function
buf_add_tail:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mvn	r3, #0
	strb	r3, [r1]
	mov	r4, r1
	ldrb	r3, [r0]	@ zero_extendqisi2
	mov	r5, r0
	cmp	r3, #255
	ldrne	r2, .L799
	movne	r1, #48
	bne	.L797
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L793
	ldr	r1, .L799+4
	mov	r2, #74
	ldr	r0, .L799+8
	bl	printk
	bl	dump_stack
.L793:
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	strb	r3, [r5]
	ldmfd	sp!, {r3, r4, r5, pc}
.L797:
	mov	r5, r3
	mla	r3, r1, r3, r2
	ldrb	r3, [r3, #1784]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L797
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L795
	ldr	r1, .L799+4
	mov	r2, #81
	ldr	r0, .L799+8
	bl	printk
	bl	dump_stack
.L795:
	ldr	r2, .L799
	mov	r1, #48
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	mla	r5, r1, r5, r2
	strb	r3, [r5, #1784]
	ldmfd	sp!, {r3, r4, r5, pc}
.L800:
	.align	2
.L799:
	.word	.LANCHOR0
	.word	.LANCHOR1+1048
	.word	.LC0
	.fnend
	.size	buf_add_tail, .-buf_add_tail
	.align	2
	.type	queue_read_cmd, %function
queue_read_cmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	ldr	r1, [r4, #24]
	mov	r0, #48
	bl	flash_start_page_read
	mov	r3, #1
	mov	r1, r4
	strb	r3, [r4, #42]
	ldr	r0, .L803
	mov	r3, #0
	strb	r3, [r4, #43]
	mvn	r3, #0
	strb	r3, [r4]
	ldmfd	sp!, {r4, lr}
	b	buf_add_tail
.L804:
	.align	2
.L803:
	.word	.LANCHOR0+3323
	.fnend
	.size	queue_read_cmd, .-queue_read_cmd
	.align	2
	.global	zbuf_free
	.type	zbuf_free, %function
zbuf_free:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrb	r3, [r0, #2]	@ zero_extendqisi2
	ldr	r4, .L815
	and	r3, r3, #8
	uxtb	r3, r3
	strb	r3, [r0, #2]
	cmp	r3, #0
	beq	.L806
	ldr	r3, [r0, #20]
	cmn	r3, #1
	beq	.L806
	mov	r1, r0
	ldr	r0, .L815+4
	bl	buf_add_tail
	b	.L807
.L806:
	ldrb	r3, [r4, #3321]	@ zero_extendqisi2
	strb	r3, [r0]
	ldrb	r3, [r0, #1]	@ zero_extendqisi2
	strb	r3, [r4, #3321]
.L807:
	ldrb	r3, [r4, #3322]	@ zero_extendqisi2
	add	r3, r3, #1
	strb	r3, [r4, #3322]
	ldmfd	sp!, {r4, pc}
.L816:
	.align	2
.L815:
	.word	.LANCHOR0
	.word	.LANCHOR0+3321
	.fnend
	.size	zbuf_free, .-zbuf_free
	.align	2
	.global	buf_alloc
	.type	buf_alloc, %function
buf_alloc:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r4, .L828
	ldrb	r3, [r4, #3322]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L818
.L821:
	ldrb	ip, [r4, #3321]	@ zero_extendqisi2
	cmp	r5, #0
	ldr	r3, .L828+4
	mov	r0, #48
	mla	r0, r0, ip, r3
	beq	.L819
	b	.L820
.L818:
	ldr	r1, .L828+8
	mov	r2, #121
	ldr	r0, .L828+12
	bl	printk
	bl	dump_stack
	ldrb	r3, [r4, #3322]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L821
.L823:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L819:
	ldr	r2, .L828
	ldrb	r2, [r2, #3322]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L823
.L820:
	mov	r1, #48
	mul	r1, r1, ip
	add	r2, r4, r1
	add	r3, r3, r1
	mov	r1, #0
	ldrb	ip, [r2, #1784]	@ zero_extendqisi2
	strh	r1, [r3, #34]	@ movhi
	str	r1, [r2, #1792]
	strb	ip, [r4, #3321]
	ldrb	ip, [r4, #3322]	@ zero_extendqisi2
	strb	r1, [r2, #1824]
	sub	ip, ip, #1
	strb	r1, [r2, #1825]
	strb	ip, [r4, #3322]
	mov	ip, #1
	strb	ip, [r2, #1786]
	mvn	ip, #0
	strb	ip, [r2, #1784]
	str	ip, [r2, #1804]
	ldmfd	sp!, {r3, r4, r5, pc}
.L829:
	.align	2
.L828:
	.word	.LANCHOR0
	.word	.LANCHOR0+1784
	.word	.LANCHOR1+1064
	.word	.LC0
	.fnend
	.size	buf_alloc, .-buf_alloc
	.align	2
	.global	buf_remove_buf
	.type	buf_remove_buf, %function
buf_remove_buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldrb	ip, [r1, #1]	@ zero_extendqisi2
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	ip, r3
	bne	.L831
	ldrb	r3, [r1]	@ zero_extendqisi2
	strb	r3, [r0]
	mov	r0, #1
	bx	lr
.L831:
	ldr	r0, .L838
	str	lr, [sp, #-4]!
	.save {lr}
	mov	lr, #48
.L833:
	cmp	r3, #255
	beq	.L837
	mov	r2, r3
	mla	r3, lr, r3, r0
	ldrb	r3, [r3, #1784]	@ zero_extendqisi2
	cmp	r3, ip
	bne	.L833
	ldr	r0, .L838
	mov	ip, #48
	ldrb	r3, [r1]	@ zero_extendqisi2
	mla	r2, ip, r2, r0
	mov	r0, #1
	strb	r3, [r2, #1784]
	mvn	r3, #0
	strb	r3, [r1]
	ldr	pc, [sp], #4
.L837:
	mov	r0, #0
	ldr	pc, [sp], #4
.L839:
	.align	2
.L838:
	.word	.LANCHOR0
	.fnend
	.size	buf_remove_buf, .-buf_remove_buf
	.align	2
	.global	buf_remove_free
	.type	buf_remove_free, %function
buf_remove_free:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L846
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldrb	r2, [r3, #3322]	@ zero_extendqisi2
	mov	r4, r3
	cmp	r2, #0
	bne	.L841
	ldr	r1, .L846+4
	mov	r2, #172
	ldr	r0, .L846+8
	bl	printk
	bl	dump_stack
.L841:
	ldrb	r3, [r4, #3322]	@ zero_extendqisi2
	ldr	r6, .L846
	cmp	r3, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldr	r0, .L846+12
	mov	r1, r5
	bl	buf_remove_buf
	cmp	r0, #1
	ldreqb	r3, [r6, #3322]	@ zero_extendqisi2
	subeq	r3, r3, #1
	streqb	r3, [r6, #3322]
	ldreqb	r3, [r5, #2]	@ zero_extendqisi2
	orreq	r3, r3, #1
	streqb	r3, [r5, #2]
	ldmfd	sp!, {r4, r5, r6, pc}
.L847:
	.align	2
.L846:
	.word	.LANCHOR0
	.word	.LANCHOR1+1076
	.word	.LC0
	.word	.LANCHOR0+3321
	.fnend
	.size	buf_remove_free, .-buf_remove_free
	.align	2
	.global	flash_check_bad_block
	.type	flash_check_bad_block, %function
flash_check_bad_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L849
	ldrb	r2, [r3, #17]	@ zero_extendqisi2
	ldrh	r3, [r3, #18]
	smulbb	r3, r2, r3
	ldr	r2, .L849+4
	ldr	r2, [r2, #1696]
	uxth	r3, r3
	add	r3, r3, #31
	mov	r3, r3, asr #5
	mov	r3, r3, asl #2
	uxth	r3, r3
	mul	r0, r3, r0
	add	r3, r0, #912
	mov	r0, r1, lsr #5
	and	r1, r1, #31
	add	r2, r2, r0, asl #2
	ldr	r0, [r2, r3]
	mov	r0, r0, lsr r1
	and	r0, r0, #1
	bx	lr
.L850:
	.align	2
.L849:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.fnend
	.size	flash_check_bad_block, .-flash_check_bad_block
	.align	2
	.global	flash_mask_bad_block
	.type	flash_mask_bad_block, %function
flash_mask_bad_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L853
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r1
	ldrb	r2, [r3, #17]	@ zero_extendqisi2
	mov	r6, r0
	ldrh	r4, [r3, #18]
	mov	r1, r6
	ldr	r0, .L853+4
	smulbb	r4, r2, r4
	mov	r2, r5
	bl	printk
	ldr	r3, .L853+8
	mov	r2, r5, lsr #5
	mov	r1, #1
	and	r5, r5, #31
	ldr	r3, [r3, #1696]
	uxth	r4, r4
	add	r4, r4, #31
	mov	r4, r4, asr #5
	mov	r4, r4, asl #2
	uxth	r4, r4
	mul	r4, r4, r6
	add	r4, r4, #912
	add	r4, r3, r4
	ldr	r3, [r4, r2, asl #2]
	orr	r5, r3, r1, asl r5
	str	r5, [r4, r2, asl #2]
	ldmfd	sp!, {r4, r5, r6, pc}
.L854:
	.align	2
.L853:
	.word	.LANCHOR2
	.word	.LC48
	.word	.LANCHOR0
	.fnend
	.size	flash_mask_bad_block, .-flash_mask_bad_block
	.align	2
	.global	str2hex
	.type	str2hex, %function
str2hex:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r3, #48
	bne	.L856
	ldrb	r3, [r0, #1]	@ zero_extendqisi2
	and	r3, r3, #223
	cmp	r3, #88
	addeq	r0, r0, #2
.L856:
	ldrb	r3, [r0]	@ zero_extendqisi2
	and	r3, r3, #223
	cmp	r3, #88
	addeq	r0, r0, #1
	sub	r3, r0, #1
	mov	r0, #0
.L858:
	ldrb	r2, [r3, #1]!	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L867
	sub	r1, r2, #48
	uxtb	ip, r1
	cmp	ip, #9
	addls	r0, r1, r0, asl #4
	bls	.L858
	sub	r1, r2, #97
	cmp	r1, #5
	subls	r2, r2, #87
	bls	.L866
	sub	r1, r2, #65
	cmp	r1, #5
	bxhi	lr
	sub	r2, r2, #55
.L866:
	add	r0, r2, r0, asl #4
	b	.L858
.L867:
	bx	lr
	.fnend
	.size	str2hex, .-str2hex
	.align	2
	.global	zftl_proc_debug_init
	.type	zftl_proc_debug_init, %function
zftl_proc_debug_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, lr}
	.save {lr}
	.pad #12
	mov	r2, #0
	mov	r1, #292
	str	r2, [sp]
	ldr	r3, .L870
	ldr	r0, .L870+4
	bl	proc_create_data
	add	sp, sp, #12
	@ sp needed
	ldr	pc, [sp], #4
.L871:
	.align	2
.L870:
	.word	.LANCHOR1+1092
	.word	.LC49
	.fnend
	.size	zftl_proc_debug_init, .-zftl_proc_debug_init
	.align	2
	.global	ftl_print_info_to_buf
	.type	ftl_print_info_to_buf, %function
ftl_print_info_to_buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #16
	mov	r6, r0
	ldr	r1, .L874
	ldr	r2, .L874+4
	bl	sprintf
	ldr	r1, .L874+8
	ldr	r5, .L874+12
	ldrb	r3, [r1, #6]	@ zero_extendqisi2
	ldrb	r2, [r1, #5]	@ zero_extendqisi2
	add	r4, r6, r0
	ldrb	r0, [r1, #7]	@ zero_extendqisi2
	str	r0, [sp]
	ldrb	r0, [r1, #8]	@ zero_extendqisi2
	str	r0, [sp, #4]
	ldrb	r0, [r1, #9]	@ zero_extendqisi2
	str	r0, [sp, #8]
	mov	r0, r4
	ldrb	r1, [r1, #10]	@ zero_extendqisi2
	str	r1, [sp, #12]
	ldr	r1, .L874+16
	bl	sprintf
	ldr	r2, [r5, #520]
	ldr	r1, .L874+20
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r2, [r5, #3324]
	ldr	r1, .L874+24
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L874+28
	add	r4, r4, r0
	mov	r0, r4
	add	r4, r4, #10
	bl	strcpy
	ldr	r2, [r5, #3328]
	mov	r0, r4
	ldr	r1, .L874+32
	bl	sprintf
	ldr	r2, [r5, #520]
	ldr	r1, .L874+36
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L874+40
	ldr	r2, [r3, #524]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L874+44
	ldr	r2, [r3, #528]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3332
	ldrh	r2, [r5, r3]
	ldr	r1, .L874+48
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3334
	ldrh	r2, [r5, r3]
	ldr	r1, .L874+52
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3336
	ldrh	r2, [r5, r3]
	ldr	r1, .L874+56
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3338
	ldrh	r2, [r5, r3]
	ldr	r1, .L874+60
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3340
	ldrh	r2, [r5, r3]
	ldr	r1, .L874+64
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3342
	ldrh	r2, [r5, r3]
	ldr	r1, .L874+68
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrb	r1, [r5, #3345]	@ zero_extendqisi2
	ldrb	r2, [r5, #3322]	@ zero_extendqisi2
	ldrb	r3, [r5, #3344]	@ zero_extendqisi2
	add	r4, r4, r0
	str	r1, [sp]
	ldr	r1, .L874+72
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+76
	ldrh	r2, [r3, #148]
	ldrh	r3, [r3, #146]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+80
	ldr	r2, [r3, #16]
	ldr	r3, [r3, #20]
	add	r2, r3, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+84
	ldr	r2, [r3, #24]
	ldr	r3, [r3, #28]
	add	r2, r3, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+88
	ldr	r2, [r3, #64]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+92
	ldr	r2, [r3, #68]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r2, .L874+96
	ldr	r1, .L874+100
	ldr	r3, [r3, #12]
	umull	r2, r3, r3, r2
	mov	r2, r3, lsr #3
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrb	r2, [r5]	@ zero_extendqisi2
	ldr	r1, .L874+104
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L874+108
	ldr	r2, [r3, #556]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L874+112
	ldr	r2, [r3, #552]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+116
	ldr	r2, [r3, #52]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+120
	ldr	r2, [r3, #60]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+124
	ldr	r2, [r3, #76]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldr	r1, .L874+128
	ldr	r2, [r3, #8]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, [r5, #580]
	ldrh	r2, [r1, #16]
	ldrh	r3, [r1, #22]
	ldrb	r1, [r1, #25]	@ zero_extendqisi2
	add	r4, r4, r0
	str	r1, [sp]
	ldr	r1, .L874+132
	mov	r0, r4
	bl	sprintf
	ldr	r1, [r5, #580]
	ldrh	r2, [r1, #48]
	ldrh	r3, [r1, #54]
	ldrb	r1, [r1, #57]	@ zero_extendqisi2
	add	r4, r4, r0
	str	r1, [sp]
	ldr	r1, .L874+136
	mov	r0, r4
	bl	sprintf
	ldr	r1, [r5, #580]
	ldrh	r2, [r1, #80]
	ldrh	r3, [r1, #86]
	ldrb	r1, [r1, #89]	@ zero_extendqisi2
	add	r4, r4, r0
	str	r1, [sp]
	ldr	r1, .L874+140
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldrh	r1, [r3, #88]
	ldrh	r2, [r3, #74]
	str	r1, [sp]
	add	r4, r4, r0
	ldrh	r1, [r3, #92]
	mov	r0, r4
	str	r1, [sp, #4]
	ldrh	r1, [r3, #96]
	str	r1, [sp, #8]
	ldr	r1, .L874+144
	ldr	r3, [r3, #84]
	bl	sprintf
	ldr	r3, [r5, #3348]
	ldrh	r1, [r3, #90]
	ldrh	r2, [r3, #72]
	str	r1, [sp]
	add	r4, r4, r0
	ldrh	r1, [r3, #94]
	mov	r0, r4
	str	r1, [sp, #4]
	ldrh	r1, [r3, #98]
	str	r1, [sp, #8]
	ldr	r1, .L874+148
	ldr	r3, [r3, #80]
	bl	sprintf
	movw	r3, #3352
	ldrh	r2, [r5, r3]
	ldr	r1, .L874+152
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r1, #3358
	ldrh	r1, [r5, r1]
	movw	r3, #3354
	ldrh	r2, [r5, r3]
	movw	r3, #3356
	ldrh	r3, [r5, r3]
	str	r1, [sp]
	add	r1, r5, #3360
	add	r4, r4, r0
	ldrh	r1, [r1]
	mov	r0, r4
	str	r1, [sp, #4]
	ldr	r1, [r5, #3364]
	str	r1, [sp, #8]
	ldr	r1, .L874+156
	bl	sprintf
	ldr	r1, [r5, #580]
	add	r3, r1, #584
	ldrh	r2, [r3]
	movw	r3, #586
	ldrh	r3, [r1, r3]
	add	r4, r4, r0
	add	r0, r1, #588
	ldrh	r0, [r0]
	str	r0, [sp]
	movw	r0, #590
	ldrh	r1, [r1, r0]
	mov	r0, r4
	str	r1, [sp, #4]
	ldr	r1, .L874+160
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L874+164
	ldr	r2, [r3, #544]
	str	r2, [sp]
	add	r4, r4, r0
	ldr	r2, [r5, #3348]
	mov	r0, r4
	ldr	r2, [r2, #44]
	str	r2, [sp, #4]
	ldr	r2, [r3, #548]
	str	r2, [sp, #8]
	ldr	r2, [r3, #536]
	ldr	r3, [r3, #540]
	bl	sprintf
	add	r0, r4, r0
	rsb	r0, r6, r0
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L875:
	.align	2
.L874:
	.word	.LC1
	.word	.LC2
	.word	.LANCHOR2
	.word	.LANCHOR0
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
	.word	-858993459
	.word	.LC70
	.word	.LC71
	.word	.LC72
	.word	.LC73
	.word	.LC74
	.word	.LC75
	.word	.LC76
	.word	.LC77
	.word	.LC78
	.word	.LC79
	.word	.LC80
	.word	.LC81
	.word	.LC82
	.word	.LC83
	.word	.LC84
	.word	.LC85
	.word	.LC86
	.fnend
	.size	ftl_print_info_to_buf, .-ftl_print_info_to_buf
	.align	2
	.global	zftl_proc_ftl_read
	.type	zftl_proc_ftl_read, %function
zftl_proc_ftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	b	ftl_print_info_to_buf
	.fnend
	.size	zftl_proc_ftl_read, .-zftl_proc_ftl_read
	.align	2
	.global	ftl_gc_write_buf
	.type	ftl_gc_write_buf, %function
ftl_gc_write_buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r1, r0
	ldrb	r3, [r0, #2]	@ zero_extendqisi2
	ldr	r4, .L879
	orr	r3, r3, #2
	strb	r3, [r0, #2]
	add	r0, r4, #3360
	add	r0, r0, #8
	bl	buf_add_tail
	ldrb	r0, [r4, #3344]	@ zero_extendqisi2
	add	r0, r0, #1
	uxtb	r0, r0
	strb	r0, [r4, #3344]
	ldmfd	sp!, {r4, pc}
.L880:
	.align	2
.L879:
	.word	.LANCHOR0
	.fnend
	.size	ftl_gc_write_buf, .-ftl_gc_write_buf
	.align	2
	.global	gc_hook
	.type	gc_hook, %function
gc_hook:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	gc_hook, .-gc_hook
	.align	2
	.global	vpn_check
	.type	vpn_check, %function
vpn_check:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	bx	lr
	.fnend
	.size	vpn_check, .-vpn_check
	.align	2
	.global	gc_init
	.type	gc_init, %function
gc_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r7, #3380
	ldr	r4, .L885
	mov	r1, #596
	mov	r5, #0
	add	r6, r4, r7
	add	r3, r4, #3376
	strb	r5, [r4, #3369]
	mov	r0, r6
	strh	r5, [r3]	@ movhi
	strb	r5, [r4, #3370]
	str	r5, [r4, #3372]
	bl	__memzero
	mvn	r3, #0
	strh	r3, [r4, r7]	@ movhi
	movw	r3, #3976
	movw	ip, #3356
	ldrh	r2, [r4, r3]
	str	r5, [r4, #3388]
	mov	r3, r2, lsr #1
	strh	r3, [r6, #34]	@ movhi
	movw	r3, #3978
	mov	r0, r2, lsr #2
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3980
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3982
	strh	r5, [r4, r3]	@ movhi
	movw	r5, #3986
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	strh	r0, [r6, #32]	@ movhi
	smulbb	r1, r3, r2
	uxth	r1, r1
	sub	lr, r1, #32
	strh	lr, [r4, ip]	@ movhi
	movw	ip, #3354
	strh	r1, [r4, ip]	@ movhi
	add	r1, r4, #3360
	strh	r2, [r1]	@ movhi
	movw	r2, #3358
	strh	r0, [r4, r2]	@ movhi
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	str	r0, [r4, #3988]
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r3, [r4, r5]
	str	r0, [r4, #3992]
	ldrb	r0, [r4, #3984]	@ zero_extendqisi2
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	str	r0, [r4, #3996]
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	str	r0, [r4, #4000]
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	movw	r3, #4008
	ldrh	r3, [r4, r3]
	movw	r2, #3352
	mov	r3, r3, lsr #2
	strh	r3, [r4, r2]	@ movhi
	str	r0, [r4, #4004]
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L886:
	.align	2
.L885:
	.word	.LANCHOR0
	.fnend
	.size	gc_init, .-gc_init
	.align	2
	.global	gc_add_sblk
	.type	gc_add_sblk, %function
gc_add_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #16
	mov	r5, r0
	ldr	r8, .L928
	mov	r4, r1
	mov	r9, r2
	ldr	r7, .L928+4
	ldr	r3, [r8]
	tst	r3, #256
	beq	.L888
	ldr	r3, [r7, #568]
	mov	r1, r5
	ldr	r2, [r7, #576]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	mov	r3, r3, lsr #5
	str	r3, [sp]
	mov	r3, r0, asl #1
	ldr	r0, .L928+8
	ldrh	r3, [r2, r3]
	mov	r2, r4
	str	r3, [sp, #4]
	ldr	r3, .L928+12
	ldrh	r3, [r3, #52]
	str	r3, [sp, #8]
	movw	r3, #3358
	ldrh	r3, [r7, r3]
	str	r3, [sp, #12]
	mov	r3, r9
	bl	printk
.L888:
	ldr	r3, .L928+16
	ldrh	r2, [r3]
	mov	r6, r3
	cmp	r2, r5
	bhi	.L889
	ldr	r1, .L928+20
	movw	r2, #454
	ldr	r0, .L928+24
	bl	printk
	bl	dump_stack
.L889:
	ldrh	r3, [r6]
	ldr	r2, .L928+4
	cmp	r3, r5
	bhi	.L890
	ldr	r3, [r8]
	tst	r3, #1024
	beq	.L926
	mov	r1, r5
	mov	r2, r4
	mov	r3, r9
	ldr	r0, .L928+28
	bl	printk
	b	.L926
.L890:
	ldr	r1, [r2, #576]
	mov	r3, r5, asl #1
	ldrh	r3, [r1, r3]
	ldr	r1, [r2, #568]
	add	r1, r1, r5, asl #2
	ldrb	ip, [r1, #2]	@ zero_extendqisi2
	ands	r6, ip, #224
	bne	.L893
	cmp	r3, #0
	beq	.L910
	ldr	r0, .L928+24
	movw	r2, #462
	ldr	r1, .L928+20
	bl	printk
.L926:
	bl	dump_stack
	b	.L910
.L893:
	movw	r1, #3380
	ldrh	r1, [r2, r1]
	cmp	r1, r5
	beq	.L910
	ldr	r2, [r2, #580]
	ldrh	r1, [r2, #48]
	cmp	r1, r5
	beq	.L910
	ldrh	r1, [r2, #16]
	cmp	r1, r5
	beq	.L910
	ldrh	r1, [r2, #80]
	cmp	r1, r5
	beq	.L910
	ldr	r1, .L928+12
	add	lr, r1, #52
	ldrh	r0, [r1, #52]
	mov	r1, #0
.L894:
	cmp	r1, r0
	bcs	.L927
	ldrh	r6, [lr, #2]!
	cmp	r6, r5
	beq	.L910
	add	r1, r1, #1
	b	.L894
.L927:
	cmp	r4, #0
	bne	.L899
	movw	r1, #4010
	ldrh	lr, [r7, r1]
	cmp	lr, r5
	moveq	r0, r4
	beq	.L892
	ldr	r6, .L928+32
	mov	r1, r4
.L898:
	ldrh	r10, [r1, r6]
	cmp	r5, r10
	bne	.L897
	ldr	r0, [r8]
	ands	r0, r0, #256
	beq	.L892
	stmia	sp, {r5, lr}
	mov	r1, r5
	mov	r2, ip, lsr #5
	ldr	r0, .L928+36
	bl	printk
	b	.L910
.L897:
	add	r1, r1, #2
	cmp	r1, #16
	bne	.L898
	add	r6, r2, r9, asl #7
	add	r6, r6, #136
	b	.L900
.L899:
	add	r6, r2, #392
.L900:
	ldr	r2, [r8]
	tst	r2, #256
	beq	.L901
	str	r3, [sp]
	movw	r3, #3358
	ldrh	r3, [r7, r3]
	mov	r1, r5
	str	r0, [sp, #4]
	mov	r2, r4
	ldr	r0, .L928+40
	str	r3, [sp, #8]
	mov	r3, ip, lsr #5
	bl	printk
.L901:
	add	r1, r6, #128
	mov	r3, r6
	movw	r0, #65535
.L904:
	mov	r2, r3
	add	r3, r3, #2
	ldrh	ip, [r2]
	cmp	ip, r0
	bne	.L902
	cmp	r4, #0
	strh	r5, [r2]	@ movhi
	ldreq	r3, [r7, #580]
	ldrne	r2, [r7, #580]
	addeq	r9, r3, r9, asl #1
	ldrneh	r3, [r2, #124]
	ldreqh	r3, [r9, #120]
	addne	r3, r3, #1
	addeq	r3, r3, #1
	strneh	r3, [r2, #124]	@ movhi
	streqh	r3, [r9, #120]	@ movhi
	b	.L925
.L902:
	cmp	r3, r1
	bne	.L904
.L925:
	mov	r0, #1
	b	.L892
.L910:
	mov	r0, #0
.L892:
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L929:
	.align	2
.L928:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC87
	.word	.LANCHOR0+3380
	.word	.LANCHOR0+564
	.word	.LANCHOR1+1200
	.word	.LC0
	.word	.LC88
	.word	.LANCHOR0+4012
	.word	.LC89
	.word	.LC90
	.fnend
	.size	gc_add_sblk, .-gc_add_sblk
	.align	2
	.global	gc_mark_bad_ppa
	.type	gc_mark_bad_ppa, %function
gc_mark_bad_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r3, #4028
	ldr	r5, .L936
	mov	r6, #1
	mov	r7, r0
	ldrb	r2, [r5, #1700]	@ zero_extendqisi2
	ldrh	r3, [r5, r3]
	rsb	r2, r2, #24
	ldrb	r1, [r5, #4030]	@ zero_extendqisi2
	rsb	r2, r3, r2
	mov	r0, r0, lsr r3
	mov	r2, r6, asl r2
	sub	r2, r2, #1
	uxth	r4, r0
	and	r0, r0, r2
	bl	__aeabi_uidiv
	ldr	r1, [r5, #3960]
	mov	r2, r4
	mov	r3, r7
	uxth	r8, r0
	ldr	r0, .L936+4
	bl	printk
	mov	r1, r6
	mov	r2, #0
	mov	r0, r8
	bl	gc_add_sblk
	ldr	r2, [r5, #3960]
	ldr	r0, .L936+8
	mov	r1, #0
.L931:
	uxth	r3, r1
	cmp	r3, r2
	bcs	.L935
	add	r3, r3, #292
	add	r1, r1, #1
	mov	r3, r3, asl #1
	ldrh	r3, [r0, r3]
	cmp	r3, r4
	bne	.L931
	b	.L932
.L935:
	cmp	r2, #5
	addls	r3, r2, #1
	addls	r2, r2, #292
	strls	r3, [r5, #3960]
	ldrls	r3, .L936+8
	movls	r2, r2, asl #1
	strlsh	r4, [r3, r2]	@ movhi
.L932:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L937:
	.align	2
.L936:
	.word	.LANCHOR0
	.word	.LC91
	.word	.LANCHOR0+3380
	.fnend
	.size	gc_mark_bad_ppa, .-gc_mark_bad_ppa
	.align	2
	.global	gc_get_src_ppa_from_index
	.type	gc_get_src_ppa_from_index, %function
gc_get_src_ppa_from_index:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L939
	ldr	r3, [r3, #4000]
	ldr	r0, [r3, r0, asl #2]
	bx	lr
.L940:
	.align	2
.L939:
	.word	.LANCHOR0
	.fnend
	.size	gc_get_src_ppa_from_index, .-gc_get_src_ppa_from_index
	.align	2
	.global	gc_write_completed
	.type	gc_write_completed, %function
gc_write_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L972
	ldr	r9, .L972+4
	mov	r5, r4
.L942:
	ldrb	r10, [r4, #4031]	@ zero_extendqisi2
	cmp	r10, #255
	beq	.L971
	mov	r3, #48
	ldr	r8, .L972+4
	mul	r3, r3, r10
	add	r2, r4, r3
	add	r3, r9, r3
	ldrb	r1, [r2, #1784]	@ zero_extendqisi2
	ldrh	r7, [r3, #32]
	strb	r1, [r4, #4031]
	ldr	r1, [r2, #1820]
	cmp	r1, #0
	beq	.L943
	ldr	r3, .L972+8
	mov	r0, #1
	ldr	r2, [r2, #1808]
	strh	r0, [r3]	@ movhi
	ldr	r0, .L972+12
	str	r2, [r4, #3956]
	bl	printk
	ldr	r1, .L972+16
	movw	r2, #805
	ldr	r0, .L972+20
	bl	printk
	bl	dump_stack
.L943:
	ldrb	r3, [r5, #4032]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L944
	ldrb	r3, [r4, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, [r4, #580]
	ldreqb	r6, [r3, #89]	@ zero_extendqisi2
	addeq	r6, r6, r6, asl #1
	beq	.L945
.L944:
	ldrb	r3, [r5, #4033]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r6, #1
	beq	.L945
	ldrb	r3, [r4, #4034]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r6, #1
	movne	r6, #2
.L945:
	ldr	r3, .L972+24
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L946
	mov	r3, #48
	ldr	r1, [r5, #3996]
	mul	r3, r3, r10
	ldr	r0, .L972+28
	ldrb	r1, [r1, r7]	@ zero_extendqisi2
	add	r2, r5, r3
	add	r3, r8, r3
	ldrb	r2, [r2, #1785]	@ zero_extendqisi2
	ldrh	r3, [r3, #32]
	str	r1, [sp]
	mov	r1, r7
	bl	printk
.L946:
	mov	r3, #48
	ldr	r2, [r5, #3996]
	mla	r3, r3, r10, r5
	mov	fp, r7
	ldrb	r2, [r2, r7]	@ zero_extendqisi2
	ldrb	r3, [r3, #1785]	@ zero_extendqisi2
	cmp	r2, r3
	beq	.L947
	ldr	r1, .L972+16
	movw	r2, #814
	ldr	r0, .L972+20
	bl	printk
	bl	dump_stack
.L947:
	mov	r3, #48
	mla	r10, r3, r10, r5
	ldrb	r3, [r10, #1829]	@ zero_extendqisi2
	cmp	r3, #3
	beq	.L948
	ldrb	r3, [r4, #4032]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L948
	ldrb	r3, [r4, #4035]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L948
	ldrb	r3, [r4, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L948
	ldrb	r3, [r4, #4033]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L949
	ldrb	r3, [r4, #4034]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L948
.L949:
	ldr	r2, [r5, #3996]
	mov	r3, #0
	mov	r1, #48
	mov	r0, r3
	add	r7, r2, r7
	b	.L950
.L948:
	ldr	r3, .L972+32
	mvn	r10, #0
	mov	r2, #0
	strh	r7, [r3]	@ movhi
	mov	r3, #48
.L951:
	rsb	r1, r7, fp
	uxth	r1, r1
	cmp	r1, r6
	bcs	.L942
	ldr	r1, [r5, #3996]
	str	r3, [sp, #12]
	str	r2, [sp, #8]
	ldrb	r0, [r1, fp]	@ zero_extendqisi2
	strb	r10, [r1, fp]
	add	fp, fp, #1
	mul	r0, r3, r0
	add	r1, r5, r0
	add	r0, r8, r0
	strb	r2, [r1, #1829]
	bl	zbuf_free
	ldrb	r1, [r5, #3387]	@ zero_extendqisi2
	sub	r1, r1, #1
	strb	r1, [r5, #3387]
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #8]
	b	.L951
.L950:
	uxth	r2, r3
	cmp	r2, r6
	bcs	.L942
	ldrb	r2, [r7, r3]	@ zero_extendqisi2
	add	r3, r3, #1
	mla	r2, r1, r2, r5
	strb	r0, [r2, #1829]
	b	.L950
.L971:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L973:
	.align	2
.L972:
	.word	.LANCHOR0
	.word	.LANCHOR0+1784
	.word	.LANCHOR0+3952
	.word	.LC92
	.word	.LANCHOR1+1212
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC93
	.word	.LANCHOR0+3948
	.fnend
	.size	gc_write_completed, .-gc_write_completed
	.align	2
	.global	gc_get_src_blk
	.type	gc_get_src_blk, %function
gc_get_src_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L984
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #580]
	ldrb	r0, [r2, #3369]	@ zero_extendqisi2
	ldrh	r1, [r3, #124]
	cmp	r1, #0
	addne	r3, r3, #392
	movne	r1, #1
	bne	.L976
	add	ip, r3, r0, asl #1
	ldrh	ip, [ip, #120]
	cmp	ip, #0
	beq	.L981
	add	r3, r3, r0, asl #7
	add	r3, r3, #136
.L976:
	add	ip, r3, #128
	movw	lr, #65535
.L980:
	mov	r4, r3
	ldrh	r0, [r3], #2
	cmp	r0, lr
	beq	.L978
	cmp	r1, #0
	mvn	r3, #0
	strh	r3, [r4]	@ movhi
	ldreqb	r1, [r2, #3369]	@ zero_extendqisi2
	ldreq	r3, [r2, #580]
	ldrne	r2, [r2, #580]
	addeq	r3, r3, r1, asl #1
	ldrneh	r3, [r2, #124]
	ldreqh	r2, [r3, #120]
	subne	r3, r3, #1
	subeq	r2, r2, #1
	strneh	r3, [r2, #124]	@ movhi
	streqh	r2, [r3, #120]	@ movhi
	ldmfd	sp!, {r4, pc}
.L978:
	cmp	r3, ip
	bne	.L980
	ldmfd	sp!, {r4, pc}
.L981:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L985:
	.align	2
.L984:
	.word	.LANCHOR0
	.fnend
	.size	gc_get_src_blk, .-gc_get_src_blk
	.align	2
	.global	gc_search_src_blk
	.type	gc_search_src_blk, %function
gc_search_src_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r5, .L1089
	mov	r6, r0
	str	r1, [sp, #12]
	ldr	r3, [r5, #580]
	add	r3, r3, r0, asl #1
	ldrh	r4, [r3, #120]
	cmp	r4, #0
	bne	.L1033
	cmp	r0, #0
	mov	r7, r2
	bne	.L989
	ldr	fp, .L1089+4
	movw	r3, #3980
	mov	r10, r0
	mov	r4, r0
	strh	r0, [r5, r3]	@ movhi
.L990:
	uxth	r8, r7
	uxth	r3, r10
	cmp	r3, r8
	bcs	.L995
	ldrh	r2, [fp]
	ldr	ip, .L1089+4
	mov	r1, r2
	add	r0, ip, #56
	str	ip, [sp, #20]
	str	r2, [sp, #16]
	bl	_list_get_gc_head_node
	movw	r3, #65535
	ldr	r2, [sp, #16]
	cmp	r0, r3
	mov	r9, r0
	ldr	ip, [sp, #20]
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [fp]	@ movhi
	beq	.L991
	ldr	r3, .L1089+8
	mov	ip, r0, asl #1
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L992
	ldr	r3, .L1089
	mov	r1, r9
	ldr	r0, .L1089+12
	str	ip, [sp, #16]
	ldr	r3, [r3, #576]
	ldrh	r3, [r3, ip]
	bl	printk
	ldr	ip, [sp, #16]
.L992:
	ldr	r3, [r5, #576]
	ldrh	r2, [r3, ip]
	ldr	r3, .L1089+16
	ldrh	r3, [r3]
	cmp	r2, r3
	bcs	.L993
	mov	r1, #0
	mov	r0, r9
	mov	r2, r1
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L994
	add	r4, r4, #1
	uxth	r4, r4
	cmp	r4, r8
	bls	.L994
	b	.L995
.L993:
	ldr	r2, .L1089
	movw	r3, #3980
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
	b	.L995
.L991:
	mov	r3, #0
	strh	r3, [ip]	@ movhi
	b	.L995
.L994:
	add	r10, r10, #1
	b	.L990
.L995:
	ldr	r3, [sp, #12]
	tst	r3, #2
	movne	r10, #0
	beq	.L997
.L998:
	uxth	r2, r10
	ldr	fp, .L1089+20
	cmp	r2, r8
	bcs	.L1002
	ldr	r2, .L1089+24
	add	r0, fp, #680
	ldrh	r9, [r2]
	str	r2, [sp, #16]
	mov	r1, r9
	add	r9, r9, #1
	bl	_list_get_gc_head_node
	ldr	r3, .L1089+24
	movw	ip, #65535
	strh	r9, [r3]	@ movhi
	cmp	r0, ip
	ldr	r2, [sp, #16]
	beq	.L999
	ldr	r3, .L1089
	mov	r1, r0, asl #1
	add	r3, r3, #3360
	ldr	r9, [r3, #-2784]
	ldrh	ip, [r9, r1]
	ldrh	r1, [r3]
	cmp	ip, r1
	bcs	.L999
	mov	r1, #0
	mov	r2, r1
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1001
	add	r4, r4, #1
	uxth	r4, r4
	cmp	r4, r8
	bls	.L1001
	b	.L1004
.L999:
	mov	r3, #0
	strh	r3, [r2]	@ movhi
	b	.L1002
.L1001:
	add	r10, r10, #1
	b	.L998
.L1002:
	cmp	r4, r8
	bcs	.L1004
	movw	r3, #3976
	ldrh	r2, [fp]
	ldrh	r1, [r5, r3]
	movw	r3, #3986
	ldrh	r0, [r5, r3]
	ldrb	r3, [r5, #3984]	@ zero_extendqisi2
	mul	r3, r3, r0
	sub	r3, r3, r1, lsr #2
	cmp	r2, r3
	ldrlt	r3, .L1089+20
	addlt	r2, r2, r1, lsr #3
	strlth	r2, [r3]	@ movhi
	b	.L997
.L1004:
	movw	r2, #3976
	ldrh	r3, [fp]
	ldrh	r2, [r5, r2]
	cmp	r3, r2
	subhi	r3, r3, r2, lsr #3
	ldrhi	r2, .L1089+20
	strhih	r3, [r2]	@ movhi
.L997:
	ldr	r3, [sp, #12]
	tst	r3, #1
	beq	.L1005
	cmp	r4, r8
	movcc	r10, #0
	ldrcc	r9, .L1089+28
	bcs	.L1005
.L1006:
	uxth	r2, r10
	cmp	r2, r8
	bcs	.L1009
	ldrh	fp, [r9]
	ldr	r2, .L1089+28
	mov	r1, fp
	add	r0, r2, #66
	str	r2, [sp, #16]
	bl	_list_get_gc_head_node
	movw	r3, #65535
	add	fp, fp, #1
	strh	fp, [r9]	@ movhi
	cmp	r0, r3
	ldr	r2, [sp, #16]
	beq	.L1007
	mov	r1, #0
	mov	r2, r1
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1008
	add	r4, r4, #1
	uxth	r4, r4
	cmp	r4, r8
	bcc	.L1008
	b	.L1011
.L1007:
	mov	r3, #0
	strh	r3, [r2]	@ movhi
	b	.L1009
.L1008:
	add	r10, r10, #1
	b	.L1006
.L1009:
	cmp	r4, r8
	bcs	.L1011
	movw	r2, #3358
	movw	r1, #3976
	ldrh	r3, [r5, r2]
	ldrh	r1, [r5, r1]
	cmp	r3, r1
	addcc	r3, r3, r1, lsr #3
	bcc	.L1088
	b	.L1005
.L1011:
	movw	r1, #3976
	movw	r2, #3358
	ldrh	r1, [r5, r1]
	ldrh	r3, [r5, r2]
	cmp	r3, r1, lsr #1
	bls	.L1005
	sub	r3, r3, r1, lsr #3
.L1088:
	ldr	r1, .L1089
	b	.L1087
.L989:
	ldr	r2, .L1089+32
	movw	r3, #570
	strh	r4, [r2, r3]	@ movhi
	add	r3, r5, #3376
	strh	r4, [r3]	@ movhi
	ldr	r3, [sp, #12]
	ands	r8, r3, #1
	moveq	r4, r8
	beq	.L1012
	movw	r2, #4008
	movw	r3, #3340
	ldrh	r2, [r5, r2]
	ldrh	r3, [r5, r3]
	cmp	r3, r2, lsr #2
	bcc	.L1013
	movw	r2, #3338
	ldrh	r2, [r5, r2]
	cmp	r2, r3
	bls	.L1014
.L1013:
	movw	r3, #3338
	movw	r9, #3978
	ldrh	r3, [r5, r3]
	mov	r1, #0
	ldr	r0, .L1089+36
	ldr	r4, .L1089
	mov	r3, r3, lsr #2
	strh	r3, [r5, r9]	@ movhi
	bl	_list_get_gc_head_node
	movw	r2, #65535
	cmp	r0, r2
	moveq	r4, #0
	beq	.L1015
	ldr	r1, [r4, #576]
	mov	r3, r0, asl #1
	movw	r2, #3976
	ldrh	r2, [r4, r2]
	ldrh	r3, [r1, r3]
	mov	r1, #0
	cmp	r3, r2, lsr #2
	movcs	r4, r1
	bcs	.L1015
	mov	r2, r6
	strh	r1, [r4, r9]	@ movhi
	bl	gc_add_sblk
	adds	r4, r0, #0
	movne	r4, #1
.L1015:
	ldr	r9, .L1089+28
	mov	fp, #64
	movw	r10, #65535
.L1017:
	ldrh	r3, [r9]
	ldr	r0, .L1089+36
	mov	r1, r3
	str	r3, [sp, #16]
	bl	_list_get_gc_head_node
	ldr	r3, [sp, #16]
	cmp	r0, r10
	add	r3, r3, #1
	strh	r3, [r9]	@ movhi
	beq	.L1014
	mov	r1, #0
	mov	r2, r6
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1016
	add	r4, r4, #1
	uxth	r4, r4
	cmp	r4, r7
	bcs	.L1014
.L1016:
	sub	fp, fp, #1
	uxth	fp, fp
	cmp	fp, #0
	bne	.L1017
.L1014:
	movw	r3, #3340
	ldrh	r2, [r5, r3]
	movw	r3, #4008
	ldrh	r3, [r5, r3]
	cmp	r2, r3, lsr #3
	bhi	.L1034
	ldr	r1, .L1089
	movw	r3, #3338
	ldrh	r3, [r1, r3]
	add	r3, r3, #8
	cmp	r2, r3
	bgt	.L1034
.L1012:
	ldr	r3, [sp, #12]
	tst	r3, #2
	beq	.L1020
	ldr	r9, .L1089+24
	movw	r3, #3982
	mov	r10, #64
	mov	fp, #0
	strh	fp, [r5, r3]	@ movhi
.L1025:
	ldrh	r2, [r9]
	ldr	r0, .L1089+40
	mov	r1, r2
	str	r2, [sp, #16]
	bl	_list_get_gc_head_node
	ldr	r2, [sp, #16]
	add	r2, r2, #1
	strh	r2, [r9]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	beq	.L1021
	cmp	r7, #1
	ldr	r1, .L1089+44
	bne	.L1022
	ldr	r3, .L1089
	add	r2, r1, #632
	ldrh	r2, [r2]
	ldrb	ip, [r3, #3984]	@ zero_extendqisi2
	smulbb	r2, ip, r2
	ldr	ip, .L1089+48
	ldrh	ip, [ip]
	sub	r2, r2, ip, lsr #3
	strh	r2, [r1]	@ movhi
.L1022:
	ldr	ip, [r5, #576]
	mov	r2, r0, asl #1
	ldrh	ip, [ip, r2]
	ldrh	r2, [r1]
	mov	r1, #0
	cmp	ip, r2
	bcs	.L1023
	mov	r2, r6
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1024
	add	r4, r4, #1
	add	fp, fp, #1
	uxth	r4, r4
	uxth	fp, fp
	cmp	r4, r7
	bcc	.L1024
	b	.L1021
.L1023:
	ldr	r2, .L1089
	movw	r3, #3982
	strh	r1, [r2, r3]	@ movhi
	b	.L1021
.L1024:
	sub	r3, r10, #1
	uxth	r10, r3
	cmp	r10, #0
	bne	.L1025
.L1021:
	cmp	r4, r7
	bcc	.L1026
	cmp	fp, #0
	bne	.L1027
	movw	r3, #3342
	ldrh	r2, [r5, r3]
	ldr	r3, .L1089+52
	ldrh	r3, [r3]
	cmp	r2, r3
	bls	.L1027
.L1026:
	movw	r2, #3986
	movw	r3, #3976
	ldrh	ip, [r5, r2]
	movw	r1, #3354
	ldrb	r2, [r5, #3984]	@ zero_extendqisi2
	ldrh	r3, [r5, r3]
	ldrh	r0, [r5, r1]
	mul	r2, r2, ip
	mov	r3, r3, lsr #3
	rsb	r2, r3, r2
	cmp	r0, r2
	addlt	r3, r3, r0
	blt	.L1086
	b	.L1020
.L1027:
	movw	r2, #3976
	movw	r1, #3354
	ldrh	r0, [r5, r2]
	ldrb	r2, [r5, #3984]	@ zero_extendqisi2
	ldrh	r3, [r5, r1]
	mul	r2, r2, r0
	cmp	r3, r2
	ble	.L1020
	sub	r3, r3, r0, lsr #3
.L1086:
	ldr	r2, .L1089
	strh	r3, [r2, r1]	@ movhi
.L1020:
	cmp	r8, #0
	beq	.L1005
	ldr	r10, .L1089
	movw	r3, #3980
	ldr	r9, .L1089+4
	mov	fp, #64
	mov	r2, #0
	strh	r2, [r5, r3]	@ movhi
.L1031:
	ldrh	r8, [r9]
	ldr	r0, .L1089+56
	mov	r1, r8
	add	r8, r8, #1
	bl	_list_get_gc_head_node
	movw	r3, #65535
	strh	r8, [r9]	@ movhi
	cmp	r0, r3
	beq	.L1028
	ldr	r2, [r10, #576]
	mov	r3, r0, asl #1
	ldrh	r2, [r2, r3]
	ldr	r3, .L1089+60
	ldrh	r1, [r3]
	cmp	r1, r2
	cmphi	r2, #2
	bls	.L1029
	add	r2, r3, #652
	ldrh	r3, [r3, #-16]
	ldrh	r2, [r2]
	cmp	r3, r2, lsr #1
	bls	.L1028
.L1029:
	mov	r1, #0
	mov	r2, r6
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1030
	add	r4, r4, #1
	uxth	r4, r4
	cmp	r4, r7
	bcs	.L1028
.L1030:
	sub	fp, fp, #1
	uxth	fp, fp
	cmp	fp, #0
	bne	.L1031
.L1028:
	cmp	r4, r7
	movw	r0, #3976
	movw	r1, #3356
	bcs	.L1032
	ldrh	r2, [r5, r0]
	ldrh	r3, [r5, r1]
	cmp	r3, r2, lsr #1
	bls	.L1005
	sub	r3, r3, r2, lsr #3
	ldr	r2, .L1089
.L1087:
	strh	r3, [r2, r1]	@ movhi
	b	.L1005
.L1032:
	ldrh	r0, [r5, r0]
	ldrb	r3, [r5, #3984]	@ zero_extendqisi2
	ldrh	r2, [r5, r1]
	mul	r3, r3, r0
	sub	r3, r3, #32
	cmp	r2, r3
	addlt	r2, r2, r0, lsr #3
	ldrlt	r3, .L1089
	strlth	r2, [r3, r1]	@ movhi
.L1005:
	ldr	r3, .L1089+8
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1033
	str	r7, [sp]
	mov	r1, r6
	ldr	r0, .L1089+64
	mov	r3, r4
	ldr	r2, [sp, #12]
	bl	printk
.L1033:
	mov	r0, r4
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1034:
	ldr	r9, .L1089+4
	movw	r3, #3980
	mov	fp, #64
	movw	r10, #65535
	mov	r2, #0
	strh	r2, [r5, r3]	@ movhi
.L1019:
	ldrh	r3, [r9]
	ldr	r0, .L1089+56
	mov	r1, r3
	str	r3, [sp, #16]
	bl	_list_get_gc_head_node
	ldr	r3, [sp, #16]
	cmp	r0, r10
	add	r3, r3, #1
	strh	r3, [r9]	@ movhi
	beq	.L1012
	mov	r1, #0
	mov	r2, r6
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1018
	add	r4, r4, #1
	uxth	r4, r4
	cmp	r4, r7
	bcs	.L1012
.L1018:
	sub	fp, fp, #1
	uxth	fp, fp
	cmp	fp, #0
	bne	.L1019
	b	.L1012
.L1090:
	.align	2
.L1089:
	.word	.LANCHOR0
	.word	.LANCHOR0+3980
	.word	.LANCHOR2
	.word	.LC94
	.word	.LANCHOR0+3358
	.word	.LANCHOR0+3360
	.word	.LANCHOR0+3982
	.word	.LANCHOR0+3978
	.word	.LANCHOR0+3380
	.word	.LANCHOR0+4044
	.word	.LANCHOR0+4040
	.word	.LANCHOR0+3354
	.word	.LANCHOR0+3976
	.word	.LANCHOR0+4048
	.word	.LANCHOR0+4036
	.word	.LANCHOR0+3356
	.word	.LC95
	.fnend
	.size	gc_search_src_blk, .-gc_search_src_blk
	.align	2
	.global	gc_free_temp_buf
	.type	gc_free_temp_buf, %function
gc_free_temp_buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r4, .L1106
	ldrb	r0, [r4, #3387]	@ zero_extendqisi2
	cmp	r0, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	ldrb	r3, [r4, #3322]	@ zero_extendqisi2
	cmp	r3, #1
	bhi	.L1098
	ldr	r3, .L1106+4
	mov	r0, #48
	ldrh	r5, [r3]
	movw	r3, #3986
	ldrh	r1, [r4, r3]
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	add	r2, r5, #24
	mul	r3, r3, r1
	ldr	r1, [r4, #3996]
	cmp	r3, r2
	movcs	r3, r2
.L1093:
	cmp	r5, r3
	bcs	.L1098
	ldrb	r2, [r1, r5]	@ zero_extendqisi2
	cmp	r2, #255
	beq	.L1094
	mul	r2, r0, r2
	add	ip, r4, r2
	ldrb	ip, [ip, #1829]	@ zero_extendqisi2
	cmp	ip, #0
	bne	.L1094
	ldr	r0, .L1106+8
	add	r0, r0, r2
	bl	zbuf_free
	ldr	r3, .L1106+12
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1095
	ldr	r3, .L1106
	mov	r1, r5
	ldr	r0, .L1106+16
	ldr	r3, [r3, #3996]
	ldrb	r2, [r3, r5]	@ zero_extendqisi2
	bl	printk
.L1095:
	ldr	r3, [r4, #3996]
	mvn	r2, #0
	mov	r0, #1
	strb	r2, [r3, r5]
	ldrb	r3, [r4, #3387]	@ zero_extendqisi2
	add	r3, r3, r2
	strb	r3, [r4, #3387]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1094:
	add	r5, r5, #1
	b	.L1093
.L1098:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L1107:
	.align	2
.L1106:
	.word	.LANCHOR0
	.word	.LANCHOR0+3948
	.word	.LANCHOR0+1784
	.word	.LANCHOR2
	.word	.LC96
	.fnend
	.size	gc_free_temp_buf, .-gc_free_temp_buf
	.align	2
	.global	print_gc_debug_info
	.type	print_gc_debug_info, %function
print_gc_debug_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, lr}
	.save {lr}
	.pad #20
	movw	r2, #3380
	ldr	r0, .L1110
	movw	ip, #566
	add	r3, r0, r2
	ldrh	r1, [r0, r2]
	ldrh	r2, [r3, #2]
	ldrh	r3, [r3, ip]
	ldrb	ip, [r0, #3322]	@ zero_extendqisi2
	str	ip, [sp]
	ldrb	ip, [r0, #3387]	@ zero_extendqisi2
	str	ip, [sp, #4]
	movw	ip, #3352
	ldrh	r0, [r0, ip]
	str	r0, [sp, #8]
	ldr	r0, .L1110+4
	bl	printk
	add	sp, sp, #20
	@ sp needed
	ldr	pc, [sp], #4
.L1111:
	.align	2
.L1110:
	.word	.LANCHOR0
	.word	.LC97
	.fnend
	.size	print_gc_debug_info, .-print_gc_debug_info
	.align	2
	.global	zftl_get_gc_node
	.type	zftl_get_gc_node, %function
zftl_get_gc_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	cmp	r1, #5
	mov	r3, r0
	ldreq	r0, .L1116
	beq	.L1115
	cmp	r1, #2
	ldreq	r0, .L1116+4
	ldrne	r0, .L1116+8
.L1115:
	mov	r1, r3
	b	_list_get_gc_head_node
.L1117:
	.align	2
.L1116:
	.word	.LANCHOR0+4040
	.word	.LANCHOR0+4044
	.word	.LANCHOR0+4036
	.fnend
	.size	zftl_get_gc_node, .-zftl_get_gc_node
	.align	2
	.global	zftl_insert_free_list
	.type	zftl_insert_free_list, %function
zftl_insert_free_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1122
	mov	r1, r0
	ldr	r3, [r3, #568]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ands	r3, r3, #24
	ldreq	r0, .L1122+4
	subeq	r2, r0, #720
	beq	.L1121
	cmp	r3, #16
	ldrne	r0, .L1122+8
	ldreq	r0, .L1122+12
	ldreq	r2, .L1122+16
	subne	r2, r0, #724
.L1121:
	b	_insert_free_list
.L1123:
	.align	2
.L1122:
	.word	.LANCHOR0
	.word	.LANCHOR0+4052
	.word	.LANCHOR0+4060
	.word	.LANCHOR0+4056
	.word	.LANCHOR0+3334
	.fnend
	.size	zftl_insert_free_list, .-zftl_insert_free_list
	.align	2
	.global	zftl_insert_data_list
	.type	zftl_insert_data_list, %function
zftl_insert_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1129
	mov	r1, r0
	ldr	r3, [r3, #568]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #64
	ldreq	r0, .L1129+4
	ldreq	r2, .L1129+8
	beq	.L1128
.L1125:
	cmp	r3, #96
	ldreq	r0, .L1129+12
	subeq	r2, r0, #696
	beq	.L1128
	cmp	r3, #160
	bxne	lr
	ldr	r0, .L1129+16
	ldr	r2, .L1129+20
.L1128:
	b	_insert_data_list
.L1130:
	.align	2
.L1129:
	.word	.LANCHOR0
	.word	.LANCHOR0+4044
	.word	.LANCHOR0+3338
	.word	.LANCHOR0+4036
	.word	.LANCHOR0+4040
	.word	.LANCHOR0+3342
	.fnend
	.size	zftl_insert_data_list, .-zftl_insert_data_list
	.align	2
	.global	zftl_gc_get_free_sblk
	.type	zftl_gc_get_free_sblk, %function
zftl_gc_get_free_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #16
	movw	r2, #65535
	ldr	r8, .L1145
	mov	r7, r0
	ldr	r3, [r8, #580]
	add	r3, r3, #588
	ldrh	r4, [r3]
	clz	r3, r0
	cmp	r4, r2
	mov	r3, r3, lsr #5
	moveq	r3, #0
	cmp	r3, #0
	beq	.L1132
	ldr	r0, .L1145+4
	mov	r1, r4
	bl	printk
	ldr	r3, [r8, #580]
	mvn	r2, #0
	add	r3, r3, #588
	strh	r2, [r3]	@ movhi
	b	.L1133
.L1132:
	movw	r3, #3334
	mov	r6, r1
	ldrh	r1, [r8, r3]
	movw	r3, #3336
	ldrh	r3, [r8, r3]
	mov	r5, r8
	cmp	r1, r3
	bls	.L1134
	cmp	r0, #0
	ldr	r2, .L1145+8
	movne	r1, r1, lsr #3
	moveq	r1, r0
	ldr	r0, .L1145+12
	b	.L1144
.L1134:
	cmp	r0, #0
	moveq	r1, r3, lsr #2
	beq	.L1138
	rsb	r1, r1, r1, asl #3
	add	r3, r1, #7
	cmp	r1, #0
	movlt	r1, r3
	ubfx	r1, r1, #3, #16
.L1138:
	ldr	r0, .L1145+16
	sub	r2, r0, #724
.L1144:
	bl	_list_pop_index_node
	movw	r3, #65535
	uxth	r4, r0
	cmp	r4, r3
	bne	.L1139
	movw	r3, #3332
	ldr	r0, .L1145+20
	ldrh	r3, [r5, r3]
	mov	r1, r4
	mov	r2, r6
	str	r3, [sp]
	movw	r3, #3336
	ldrh	r3, [r5, r3]
	str	r3, [sp, #4]
	ldr	r3, [r5, #4052]
	bl	printk
.L1139:
	cmp	r7, #0
	beq	.L1133
	ldr	r3, .L1145+24
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1133
	ldr	r2, [r5, #568]
	mov	r0, r4, asl #2
	add	r3, r2, r0
	ldr	r1, [r2, r4, asl #2]
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ubfx	r1, r1, #11, #8
	str	r1, [sp]
	ldrh	r2, [r2, r0]
	ldr	r0, .L1145+28
	ubfx	r2, r2, #0, #11
	str	r2, [sp, #4]
	ldr	r1, [r5, #576]
	mov	r2, r4, asl #1
	ldrh	r2, [r1, r2]
	mov	r1, r4
	str	r2, [sp, #8]
	mov	r2, r3, lsr #5
	ubfx	r3, r3, #3, #2
	bl	printk
.L1133:
	mov	r0, r4
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1146:
	.align	2
.L1145:
	.word	.LANCHOR0
	.word	.LC98
	.word	.LANCHOR0+3334
	.word	.LANCHOR0+4056
	.word	.LANCHOR0+4060
	.word	.LC99
	.word	.LANCHOR2
	.word	.LC100
	.fnend
	.size	zftl_gc_get_free_sblk, .-zftl_gc_get_free_sblk
	.align	2
	.global	zftl_get_free_sblk
	.type	zftl_get_free_sblk, %function
zftl_get_free_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #5
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r6, r1
	ldr	r5, .L1162
	bne	.L1148
	movw	r3, #3334
	ldrh	r1, [r5, r3]
	movw	r3, #3336
	ldrh	r2, [r5, r3]
	cmp	r1, r2
	bcc	.L1149
	movw	r3, #3332
	ldrh	r3, [r5, r3]
	cmp	r2, #0
	cmpne	r3, r1
	movhi	r1, #1
	movls	r1, #0
	bls	.L1150
.L1149:
	ldr	r0, .L1162+4
	mov	r1, r2, lsr #1
	b	.L1161
.L1150:
	ldr	r0, .L1162+8
	ldr	r2, .L1162+12
	b	.L1160
.L1148:
	ldr	r2, [r5, #580]
	movw	r7, #590
	ldrh	r4, [r2, r7]
	movw	r2, #65535
	cmp	r1, #1
	cmpne	r4, r2
	beq	.L1152
	ldr	r0, .L1162+16
	mov	r1, r4
	bl	printk
	ldr	r3, [r5, #580]
	mvn	r2, #0
	strh	r2, [r3, r7]	@ movhi
	b	.L1153
.L1152:
	movw	r2, #3332
	movw	r1, #3336
	ldrh	r2, [r5, r2]
	mov	r3, r0
	ldrh	r1, [r5, r1]
	cmp	r2, r1
	bcc	.L1154
	movw	r0, #3334
	ldrh	r0, [r5, r0]
	cmp	r1, #0
	cmpne	r0, r2
	bls	.L1155
.L1154:
	cmp	r6, #1
	ldr	r0, .L1162+4
	moveq	r3, r1, lsr #1
	mov	r1, r3
.L1161:
	sub	r2, r0, #724
	b	.L1160
.L1155:
	cmp	r6, #1
	ldr	r0, .L1162+20
	moveq	r3, r2, lsr #1
	sub	r2, r0, #720
	mov	r1, r3
.L1160:
	bl	_list_pop_index_node
	movw	r3, #65535
	uxth	r4, r0
	cmp	r4, r3
	bne	.L1153
	movw	r3, #3332
	ldr	r0, .L1162+24
	ldrh	r3, [r5, r3]
	mov	r1, r4
	mov	r2, r6
	str	r3, [sp]
	movw	r3, #3336
	ldrh	r3, [r5, r3]
	str	r3, [sp, #4]
	ldr	r3, [r5, #4052]
	bl	printk
.L1153:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L1163:
	.align	2
.L1162:
	.word	.LANCHOR0
	.word	.LANCHOR0+4060
	.word	.LANCHOR0+4056
	.word	.LANCHOR0+3334
	.word	.LC101
	.word	.LANCHOR0+4052
	.word	.LC99
	.fnend
	.size	zftl_get_free_sblk, .-zftl_get_free_sblk
	.align	2
	.global	zftl_remove_data_node
	.type	zftl_remove_data_node, %function
zftl_remove_data_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1169
	mov	r1, r0
	ldr	r3, [r3, #568]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #64
	ldreq	r0, .L1169+4
	ldreq	r2, .L1169+8
	beq	.L1168
.L1165:
	cmp	r3, #96
	ldreq	r0, .L1169+12
	subeq	r2, r0, #696
	beq	.L1168
	cmp	r3, #160
	bxne	lr
	ldr	r0, .L1169+16
	ldr	r2, .L1169+20
.L1168:
	b	_list_remove_node
.L1170:
	.align	2
.L1169:
	.word	.LANCHOR0
	.word	.LANCHOR0+4044
	.word	.LANCHOR0+3338
	.word	.LANCHOR0+4036
	.word	.LANCHOR0+4040
	.word	.LANCHOR0+3342
	.fnend
	.size	zftl_remove_data_node, .-zftl_remove_data_node
	.align	2
	.global	zftl_remove_free_node
	.type	zftl_remove_free_node, %function
zftl_remove_free_node:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1175
	mov	r1, r0
	ldr	r3, [r3, #568]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ands	r3, r3, #24
	ldreq	r0, .L1175+4
	subeq	r2, r0, #720
	beq	.L1174
	cmp	r3, #16
	ldrne	r0, .L1175+8
	ldreq	r0, .L1175+12
	ldreq	r2, .L1175+16
	subne	r2, r0, #724
.L1174:
	b	_list_remove_node
.L1176:
	.align	2
.L1175:
	.word	.LANCHOR0
	.word	.LANCHOR0+4052
	.word	.LANCHOR0+4060
	.word	.LANCHOR0+4056
	.word	.LANCHOR0+3334
	.fnend
	.size	zftl_remove_free_node, .-zftl_remove_free_node
	.align	2
	.global	zftl_list_update_data_list
	.type	zftl_list_update_data_list, %function
zftl_list_update_data_list:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1182
	mov	r1, r0
	ldr	r3, [r3, #568]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #64
	ldreq	r0, .L1182+4
	ldreq	r2, .L1182+8
	beq	.L1181
.L1178:
	cmp	r3, #96
	ldreq	r0, .L1182+12
	subeq	r2, r0, #696
	beq	.L1181
	cmp	r3, #160
	bxne	lr
	ldr	r0, .L1182+16
	ldr	r2, .L1182+20
.L1181:
	b	_list_update_data_list
.L1183:
	.align	2
.L1182:
	.word	.LANCHOR0
	.word	.LANCHOR0+4044
	.word	.LANCHOR0+3338
	.word	.LANCHOR0+4036
	.word	.LANCHOR0+4040
	.word	.LANCHOR0+3342
	.fnend
	.size	zftl_list_update_data_list, .-zftl_list_update_data_list
	.align	2
	.global	print_list_info
	.type	print_list_info, %function
print_list_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	mov	r4, r0
	ldrh	r2, [r1]
	.pad #36
	sub	sp, sp, #36
	ldr	r1, [r4]
	ldr	r0, .L1189
	bl	printk
	ldr	r4, [r4]
	cmp	r4, #0
	beq	.L1184
	ldr	r6, .L1189+4
	mov	r5, #0
	ldr	r7, .L1189+8
.L1187:
	ldr	r2, [r6, #524]
	ldrh	r0, [r4, #2]
	rsb	r2, r2, r4
	ldr	r1, [r6, #568]
	ldrh	r3, [r4]
	mov	r2, r2, asr #1
	str	r0, [sp]
	ldrh	r0, [r4, #4]
	mul	r2, r7, r2
	str	r0, [sp, #4]
	uxth	r2, r2
	mov	ip, r2, asl #2
	add	lr, r1, ip
	ldrb	r0, [lr, #2]	@ zero_extendqisi2
	mov	r0, r0, lsr #5
	str	r0, [sp, #8]
	ldrb	r0, [lr, #2]	@ zero_extendqisi2
	ubfx	r0, r0, #3, #2
	str	r0, [sp, #12]
	ldr	r0, [r1, r2, asl #2]
	ubfx	r0, r0, #11, #8
	str	r0, [sp, #16]
	ldrh	r1, [r1, ip]
	ldr	r0, [r6, #576]
	ubfx	r1, r1, #0, #11
	str	r1, [sp, #20]
	mov	r1, r2, asl #1
	ldrh	r1, [r0, r1]
	ldr	r0, .L1189+12
	str	r1, [sp, #24]
	mov	r1, r5
	bl	printk
	ldrh	r2, [r4]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1184
	ldr	r3, [r6, #524]
	mov	r4, #6
	add	r5, r5, #1
	mla	r4, r4, r2, r3
	ldr	r3, .L1189+16
	uxth	r5, r5
	ldrh	r3, [r3]
	cmp	r3, r5
	bcs	.L1187
.L1184:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L1190:
	.align	2
.L1189:
	.word	.LC102
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC103
	.word	.LANCHOR0+4064
	.fnend
	.size	print_list_info, .-print_list_info
	.align	2
	.global	dump_all_list_info
	.type	dump_all_list_info, %function
dump_all_list_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, .L1193
	add	r0, r4, #4048
	add	r1, r4, #3328
	add	r0, r0, #4
	add	r1, r1, #4
	bl	print_list_info
	add	r0, r4, #4048
	add	r1, r4, #3328
	add	r0, r0, #8
	add	r1, r1, #6
	bl	print_list_info
	add	r0, r4, #4048
	add	r1, r4, #3328
	add	r0, r0, #12
	add	r1, r1, #8
	bl	print_list_info
	add	r0, r4, #4032
	add	r1, r4, #3328
	add	r0, r0, #12
	add	r1, r1, #10
	bl	print_list_info
	add	r0, r4, #4032
	add	r1, r4, #3328
	add	r0, r0, #4
	add	r1, r1, #12
	bl	print_list_info
	add	r0, r4, #4032
	add	r1, r4, #3328
	add	r0, r0, #8
	add	r1, r1, #14
	ldmfd	sp!, {r4, lr}
	b	print_list_info
.L1194:
	.align	2
.L1193:
	.word	.LANCHOR0
	.fnend
	.size	dump_all_list_info, .-dump_all_list_info
	.align	2
	.global	ftl_tmp_into_update
	.type	ftl_tmp_into_update, %function
ftl_tmp_into_update:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1200
	ldr	r3, [r3, #3348]
	ldr	r2, [r3, #16]
	cmp	r2, #2048
	movhi	r0, r2, lsr #11
	ldrhi	r1, [r3, #20]
	subhi	r2, r2, r0, asl #11
	strhi	r2, [r3, #16]
	ldr	r2, [r3, #24]
	addhi	r1, r1, r0
	strhi	r1, [r3, #20]
	cmp	r2, #2048
	movhi	r0, r2, lsr #11
	ldrhi	r1, [r3, #28]
	subhi	r2, r2, r0, asl #11
	strhi	r2, [r3, #24]
	ldr	r2, [r3, #32]
	addhi	r1, r1, r0
	strhi	r1, [r3, #28]
	cmp	r2, #1024
	movhi	r0, r2, lsr #10
	ldrhi	r1, [r3, #36]
	subhi	r2, r2, r0, asl #10
	strhi	r2, [r3, #32]
	ldr	r2, [r3, #40]
	addhi	r1, r1, r0
	strhi	r1, [r3, #36]
	cmp	r2, #1024
	movhi	r0, r2, lsr #10
	ldrhi	r1, [r3, #44]
	subhi	r2, r2, r0, asl #10
	strhi	r2, [r3, #40]
	addhi	r1, r1, r0
	strhi	r1, [r3, #44]
	bx	lr
.L1201:
	.align	2
.L1200:
	.word	.LANCHOR0
	.fnend
	.size	ftl_tmp_into_update, .-ftl_tmp_into_update
	.align	2
	.global	gc_static_wearleveling
	.type	gc_static_wearleveling, %function
gc_static_wearleveling:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r4, .L1318
	ldr	r3, [r4, #3348]
	ldr	r3, [r3, #32]
	cmp	r3, #10240
	bls	.L1203
	bl	ftl_tmp_into_update
.L1203:
	ldr	r3, [r4, #580]
	ldr	r2, [r3, #568]
	ldr	r1, [r3, #12]
	add	r2, r2, #860160
	add	r2, r2, #3840
	cmp	r1, r2
	bhi	.L1204
	ldr	r2, .L1318
	ldr	r0, [r2, #3348]
	ldr	r2, [r3, #572]
	ldr	r0, [r0, #36]
	add	r2, r2, #32
	cmp	r0, r2
	movls	r7, #0
	movls	r10, r7
	bls	.L1205
.L1204:
	ldr	r2, [r4, #3348]
	mov	r8, #0
	ldr	ip, .L1318
	movw	r5, #65535
	mov	fp, r8
	mov	r10, r8
	ldr	r0, [r2, #36]
	mov	r9, r8
	str	r1, [r3, #568]
	mov	r6, r5
	str	r8, [sp, #32]
	str	r0, [r3, #572]
	ldrh	r7, [r2, #134]
	str	r8, [sp, #28]
	str	r8, [sp, #24]
	str	r8, [sp, #20]
.L1206:
	ldr	r3, .L1318
	add	r2, r3, #564
	ldrh	r2, [r2]
	cmp	r2, r7
	bls	.L1315
	ldr	r0, [ip, #568]
	mov	r1, r7, asl #2
	add	lr, r0, r1
	ldrb	r3, [lr, #2]	@ zero_extendqisi2
	and	r2, r3, #224
	cmp	r2, #224
	beq	.L1207
	tst	r3, #8
	ldrneh	r3, [r0, r1]
	ldrne	r4, [r0, r1]
	ubfxne	r3, r3, #0, #11
	ubfxne	r4, r4, #11, #8
	bne	.L1209
.L1208:
	tst	r3, #24
	ldreqh	r3, [r0, r1]
	movweq	r4, #65535
	ldrne	r4, [r0, r1]
	movwne	r3, #65535
	ubfxeq	r3, r3, #0, #11
	ubfxne	r4, r4, #11, #8
	bne	.L1211
.L1209:
	ldr	r2, [sp, #32]
	add	r10, r10, #1
	cmp	r5, r3
	add	r2, r2, r3
	uxth	r10, r10
	str	r2, [sp, #32]
	ldrhi	r2, [ip, #580]
	movhi	r5, r3
	addhi	r2, r2, #584
	strhih	r7, [r2, #2]	@ movhi
.L1253:
	ldr	r2, [sp, #20]
	cmp	r2, r3
	movw	r2, #65535
	strcc	r3, [sp, #20]
	movcc	r8, r7
	cmp	r4, r2
	beq	.L1213
.L1211:
	ldr	r2, [sp, #24]
	cmp	r6, r4
	add	r2, r2, #1
	movhi	r6, r4
	uxth	r2, r2
	str	r2, [sp, #24]
	ldr	r2, [sp, #28]
	add	r2, r2, r4
	str	r2, [sp, #28]
	ldrhi	r2, [ip, #580]
	addhi	r2, r2, #584
	strhih	r7, [r2]	@ movhi
	cmp	r9, r4
	movcc	fp, r7
	movcc	r9, r4
.L1213:
	cmp	r4, #9
	cmphi	r3, #9
	bhi	.L1207
	ldr	r3, .L1318+4
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1207
	ldrh	r2, [r0, r1]
	ldr	r3, [r0, r1]
	ldrb	r1, [lr, #2]	@ zero_extendqisi2
	ubfx	r2, r2, #0, #11
	ubfx	r3, r3, #11, #8
	str	ip, [sp, #36]
	ubfx	r1, r1, #3, #2
	str	r1, [sp]
	ldrb	r1, [lr, #2]	@ zero_extendqisi2
	mov	r1, r1, lsr #5
	str	r1, [sp, #4]
	ldrb	r1, [lr, #3]	@ zero_extendqisi2
	str	r1, [sp, #8]
	mov	r1, r7, asl #1
	ldr	r0, [ip, #576]
	ldrh	r1, [r0, r1]
	ldr	r0, .L1318+8
	str	r1, [sp, #12]
	mov	r1, r7
	bl	printk
	ldr	ip, [sp, #36]
.L1207:
	add	r7, r7, #1
	uxth	r7, r7
	b	.L1206
.L1315:
	ldr	r4, [r3, #580]
	mov	r2, #0
	mov	r1, #255
	str	r3, [sp, #36]
	add	r0, r4, #264
	strh	r2, [r4, #122]	@ movhi
	mov	r2, #128
	bl	memset
	ldr	ip, .L1318+4
	movw	r2, #586
	ldrh	r1, [r4, r2]
	ldr	ip, [ip]
	tst	ip, #1024
	mov	r2, r1, asl #2
	ldr	r3, [sp, #36]
	ldr	r0, [r3, #568]
	add	r4, r0, r2
	beq	.L1216
	ldrh	r2, [r0, r2]
	ldr	ip, [r0, r1, asl #2]
	ldrb	r0, [r4, #2]	@ zero_extendqisi2
	ubfx	r2, r2, #0, #11
	ubfx	r0, r0, #3, #2
	str	r0, [sp]
	ldrb	r0, [r4, #2]	@ zero_extendqisi2
	mov	r0, r0, lsr #5
	str	r0, [sp, #4]
	ldrb	r0, [r4, #3]	@ zero_extendqisi2
	str	r0, [sp, #8]
	ldr	r0, [r3, #576]
	mov	r3, r1, asl #1
	ldrh	r3, [r0, r3]
	ldr	r0, .L1318+12
	str	r3, [sp, #12]
	ubfx	r3, ip, #11, #8
	bl	printk
.L1216:
	ldrb	r3, [r4, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #32
	ldr	r3, .L1318
	bne	.L1217
	ldr	r2, [r3, #3348]
	add	r2, r2, #688
	ldrh	r2, [r2]
	cmp	r2, #2
	movhi	r2, #1
	strhi	r2, [r3, #4068]
.L1217:
	ldrb	r2, [r4, #2]	@ zero_extendqisi2
	tst	r2, #8
	beq	.L1218
	and	r1, r2, #192
	cmp	r1, #64
	bne	.L1219
	ldr	r2, [r3, #580]
	movw	r3, #586
	mov	r1, #0
	ldrh	r0, [r2, r3]
	mov	r2, #1
	bl	gc_add_sblk
	ldr	r2, .L1318+16
	movw	r3, #570
	mov	r1, #1
	strh	r1, [r2, r3]	@ movhi
	b	.L1218
.L1219:
	tst	r2, #224
	bne	.L1218
	ldr	r3, [r3, #580]
	movw	r7, #590
	movw	r2, #65535
	ldrh	r1, [r3, r7]
	cmp	r1, r2
	bne	.L1218
	movw	r4, #586
	ldrh	r0, [r3, r4]
	add	r3, r3, #588
	ldrh	r3, [r3]
	cmp	r3, r0
	beq	.L1218
	bl	zftl_remove_free_node
	ldr	r3, .L1318
	ldr	r3, [r3, #580]
	ldrh	r2, [r3, r4]
	strh	r2, [r3, r7]	@ movhi
	mvn	r2, #0
	strh	r2, [r3, r4]	@ movhi
.L1218:
	ldr	r4, .L1318
	ldr	r0, .L1318+4
	ldr	r3, [r4, #580]
	ldr	r0, [r0]
	add	r3, r3, #584
	tst	r0, #1024
	ldrh	r1, [r3]
	ldr	r3, [r4, #568]
	mov	r2, r1, asl #2
	add	r7, r3, r2
	beq	.L1220
	ldrb	r0, [r7, #2]	@ zero_extendqisi2
	ldrh	r2, [r3, r2]
	ubfx	r0, r0, #3, #2
	ldr	r3, [r3, r1, asl #2]
	str	r0, [sp]
	ubfx	r2, r2, #0, #11
	ldrb	r0, [r7, #2]	@ zero_extendqisi2
	ubfx	r3, r3, #11, #8
	mov	r0, r0, lsr #5
	str	r0, [sp, #4]
	ldrb	r0, [r7, #3]	@ zero_extendqisi2
	str	r0, [sp, #8]
	mov	r0, r1, asl #1
	ldr	ip, [r4, #576]
	ldrh	r0, [ip, r0]
	str	r0, [sp, #12]
	ldr	r0, .L1318+20
	bl	printk
.L1220:
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	tst	r3, #8
	beq	.L1221
	and	r2, r3, #192
	ldr	r7, .L1318
	cmp	r2, #64
	bne	.L1222
	ldr	r3, [r7, #580]
	mov	r1, #0
	mov	r2, #1
	add	r3, r3, #584
	ldrh	r0, [r3]
	bl	gc_add_sblk
	ldr	r2, .L1318+16
	movw	r3, #570
	mov	r1, #1
	strh	r1, [r2, r3]	@ movhi
	b	.L1221
.L1222:
	and	r3, r3, #248
	cmp	r3, #16
	bne	.L1221
	ldr	r3, [r7, #580]
	add	r2, r3, #588
	ldrh	r1, [r2]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L1221
	add	r2, r3, #584
	ldrh	r0, [r2]
	movw	r2, #590
	ldrh	r3, [r3, r2]
	cmp	r3, r0
	beq	.L1221
	bl	zftl_remove_free_node
	ldr	r3, [r7, #580]
	add	r2, r3, #588
	add	r3, r3, #584
	ldrh	r1, [r3]
	strh	r1, [r2]	@ movhi
	mvn	r2, #0
	strh	r2, [r3]	@ movhi
.L1221:
	ldr	r2, .L1318+4
	ldr	ip, .L1318
	ldr	r2, [r2]
	ldr	r3, [ip, #568]
	tst	r2, #1024
	beq	.L1223
	mov	r2, r8, asl #2
	add	r1, r3, r2
	ldrh	r2, [r3, r2]
	ldrb	r0, [r1, #2]	@ zero_extendqisi2
	ldr	r3, [r3, r8, asl #2]
	ubfx	r2, r2, #0, #11
	ubfx	r0, r0, #3, #2
	str	r0, [sp]
	ldrb	r0, [r1, #2]	@ zero_extendqisi2
	ubfx	r3, r3, #11, #8
	mov	r0, r0, lsr #5
	str	r0, [sp, #4]
	ldrb	r1, [r1, #3]	@ zero_extendqisi2
	str	r1, [sp, #8]
	mov	r1, r8, asl #1
	ldr	r0, [ip, #576]
	ldrh	r1, [r0, r1]
	ldr	r0, .L1318+24
	str	r1, [sp, #12]
	mov	r1, r8
	bl	printk
.L1223:
	ldr	r2, .L1318+4
	ldr	ip, .L1318
	ldr	r2, [r2]
	ldr	r3, [ip, #568]
	tst	r2, #1024
	beq	.L1224
	mov	r2, fp, asl #2
	add	r1, r3, r2
	ldrh	r2, [r3, r2]
	ldrb	r0, [r1, #2]	@ zero_extendqisi2
	ldr	r3, [r3, fp, asl #2]
	ubfx	r2, r2, #0, #11
	ubfx	r0, r0, #3, #2
	str	r0, [sp]
	ldrb	r0, [r1, #2]	@ zero_extendqisi2
	ubfx	r3, r3, #11, #8
	mov	r0, r0, lsr #5
	str	r0, [sp, #4]
	ldrb	r1, [r1, #3]	@ zero_extendqisi2
	str	r1, [sp, #8]
	mov	r1, fp, asl #1
	ldr	r0, [ip, #576]
	ldrh	r1, [r0, r1]
	ldr	r0, .L1318+28
	str	r1, [sp, #12]
	mov	r1, fp
	bl	printk
.L1224:
	ldr	r4, [r4, #3348]
	mov	r1, r10
	ldrh	r3, [sp, #20]
	ldr	r0, [sp, #32]
	strh	r9, [r4, #98]	@ movhi
	strh	r3, [r4, #96]	@ movhi
	strh	r5, [r4, #92]	@ movhi
	strh	r6, [r4, #94]	@ movhi
	bl	__aeabi_uidiv
	strh	r0, [r4, #88]	@ movhi
	mov	r7, r0
	ldr	r1, [sp, #24]
	ldr	r0, [sp, #28]
	bl	__aeabi_uidiv
	strh	r0, [r4, #90]	@ movhi
	ldr	r4, .L1318+4
	ldr	r3, [r4]
	tst	r3, #1024
	beq	.L1225
	uxth	r0, r0
	mov	r1, r10
	str	r0, [sp]
	uxth	r3, r7
	ldr	r0, .L1318+32
	ldr	r2, [sp, #24]
	bl	printk
.L1225:
	ldr	r3, [r4]
	ldr	r7, .L1318
	tst	r3, #1024
	beq	.L1226
	movw	r3, #4072
	str	r9, [sp]
	ldrh	r3, [r7, r3]
	mov	r1, r5
	ldr	r0, .L1318+36
	mov	r2, r6
	str	r3, [sp, #4]
	movw	r3, #4074
	ldrh	r3, [r7, r3]
	str	r3, [sp, #8]
	ldr	r3, [sp, #20]
	bl	printk
.L1226:
	rsb	r3, r6, r9
	str	r3, [sp, #24]
	movw	r3, #4074
	ldr	r2, [sp, #24]
	ldrh	r3, [r7, r3]
	cmp	r2, r3
	bgt	.L1227
	ldr	r1, .L1318
	movw	r2, #4072
	ldr	r3, [sp, #20]
	ldrh	r2, [r1, r2]
	rsb	r3, r5, r3
	cmp	r3, r2
	movle	r7, #0
	movle	r10, r7
	ble	.L1228
.L1227:
	ldr	r3, [r7, #580]
	ldr	r4, [r3, #580]
	ldr	r3, [r7, #3348]
	mov	r7, #0
	mov	r10, r7
	uxth	r4, r4
	ldrh	fp, [r3, #134]
	ldr	r3, .L1318+40
	sub	r9, r3, #564
.L1229:
	ldr	r3, .L1318+40
	ldrh	r3, [r3]
	cmp	fp, r3
	bcs	.L1238
	add	r4, r4, #1
	uxth	r4, r4
	cmp	r4, r3
	ldr	r3, [r9, #568]
	movcs	r4, #0
	mov	r8, r4, asl #2
	add	r2, r3, r8
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	tst	r2, #192
	and	r0, r2, #224
	moveq	r1, #1
	movne	r1, #0
	cmp	r0, #224
	orreq	r1, r1, #1
	cmp	r1, #0
	bne	.L1231
	ubfx	r2, r2, #3, #2
	ands	r1, r2, #1
	beq	.L1232
	cmp	r0, #160
	b	.L1313
.L1232:
	cmp	r2, #2
.L1313:
	bne	.L1234
	ldr	r2, .L1318+44
	ldr	r0, [sp, #24]
	ldrh	r2, [r2]
	cmp	r0, r2
	ble	.L1235
	ldr	r2, [r3, r8]
	ubfx	r2, r2, #11, #8
	cmp	r2, r6
	bls	.L1236
	cmp	r1, #0
	beq	.L1235
	ldrh	r2, [r3, r8]
	ubfx	r2, r2, #0, #11
	cmp	r2, r5
	bgt	.L1235
.L1236:
	mov	r1, #0
	mov	r2, #1
	mov	r0, r4
	str	r3, [sp, #28]
	bl	gc_add_sblk
	ldr	r2, .L1318+48
	mov	r1, #1
	add	r10, r10, r1
	strh	r1, [r2]	@ movhi
	ldr	r2, .L1318+4
	ldr	r2, [r2]
	tst	r2, #1024
	ldr	r3, [sp, #28]
	beq	.L1235
	ldr	r0, [r9, #576]
	mov	r1, r4, asl r1
	ldr	r2, [r9, #568]
	ldrh	lr, [r0, r1]
	add	r2, r2, r8
	ldr	r1, .L1318+16
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	ldr	r0, .L1318+52
	ldrh	r1, [r1, #52]
	str	r1, [sp]
	ldrh	r1, [r3, r8]
	ubfx	r1, r1, #0, #11
	str	r1, [sp, #4]
	ldr	r3, [r3, r8]
	ubfx	r3, r3, #11, #8
	str	r3, [sp, #8]
	b	.L1314
.L1234:
	ldr	r0, .L1318+56
	ldr	r2, [sp, #20]
	ldrh	r0, [r0]
	rsb	r2, r5, r2
	cmp	r2, r0
	ble	.L1235
	ldrh	r2, [r3, r8]
	add	r0, r5, #8
	ubfx	r2, r2, #0, #11
	cmp	r2, r0
	ble	.L1237
	cmp	r1, #0
	beq	.L1235
	ldr	r2, [r3, r8]
	add	r1, r6, #4
	ubfx	r2, r2, #11, #8
	cmp	r2, r1
	bgt	.L1235
.L1237:
	mov	r1, #0
	mov	r2, #1
	mov	r0, r4
	str	r3, [sp, #28]
	bl	gc_add_sblk
	ldr	r2, .L1318+48
	mov	r1, #1
	add	r7, r7, r1
	strh	r1, [r2]	@ movhi
	ldr	r2, .L1318+4
	ldr	r2, [r2]
	tst	r2, #1024
	ldr	r3, [sp, #28]
	beq	.L1235
	ldr	r0, [r9, #576]
	mov	r1, r4, asl r1
	ldr	r2, [r9, #568]
	ldrh	lr, [r0, r1]
	add	r2, r2, r8
	ldr	r1, .L1318+16
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	ldr	r0, .L1318+60
	ldrh	r1, [r1, #52]
	str	r1, [sp]
	ldrh	r1, [r3, r8]
	ubfx	r1, r1, #0, #11
	str	r1, [sp, #4]
	ldr	r3, [r3, r8]
	ubfx	r3, r3, #11, #8
	str	r3, [sp, #8]
.L1314:
	mov	r1, r4
	mov	r2, r2, lsr #5
	mov	r3, lr
	bl	printk
.L1235:
	cmp	r10, #4
	cmpls	r7, #4
	bhi	.L1238
.L1231:
	add	fp, fp, #1
	uxth	fp, fp
	b	.L1229
.L1238:
	ldr	r3, .L1318
	ldr	r3, [r3, #580]
	str	r4, [r3, #580]
.L1228:
	ldr	r0, .L1318
	cmp	r6, #0
	ldrne	r3, [r0, #3348]
	movne	r4, r0
	ldrneh	r3, [r3, #134]
	beq	.L1241
.L1242:
	ldr	r2, .L1318
	add	r1, r2, #564
	ldrh	r1, [r1]
	cmp	r1, r3
	bls	.L1316
	ldr	ip, [r4, #568]
	mov	lr, r3, asl #2
	add	r8, ip, lr
	ldr	r2, [ip, r3, asl #2]
	ubfx	r1, r2, #11, #8
	cmp	r1, r6
	bcc	.L1243
	ldrb	r8, [r8, #2]	@ zero_extendqisi2
	tst	r8, #24
	rsbne	r1, r6, r1
	bfine	r2, r1, #11, #8
	strne	r2, [ip, lr]
.L1243:
	add	r3, r3, #1
	uxth	r3, r3
	b	.L1242
.L1316:
	ldr	r3, [r2, #3348]
	ldrh	r2, [r3, #72]
	add	r2, r6, r2
	strh	r2, [r3, #72]	@ movhi
	ldrh	r2, [r3, #98]
	cmp	r2, r6
	rsbhi	r2, r6, r2
	strhih	r2, [r3, #98]	@ movhi
.L1241:
	cmp	r5, #0
	beq	.L1247
	ldr	r3, [r0, #3348]
	ldr	r4, .L1318
	ldrh	r3, [r3, #134]
.L1248:
	ldr	r2, .L1318
	add	r1, r2, #564
	ldrh	r1, [r1]
	cmp	r1, r3
	bls	.L1317
	ldr	ip, [r4, #568]
	mov	r0, r3, asl #2
	add	lr, ip, r0
	ldrh	r2, [ip, r0]
	ubfx	r1, r2, #0, #11
	cmp	r1, r5
	blt	.L1249
	ldrb	lr, [lr, #2]	@ zero_extendqisi2
	and	lr, lr, #24
	cmp	lr, #16
	rsbne	r1, r5, r1
	bfine	r2, r1, #0, #11
	strneh	r2, [ip, r0]	@ movhi
.L1249:
	add	r3, r3, #1
	uxth	r3, r3
	b	.L1248
.L1317:
	ldr	r3, [r2, #3348]
	ldrh	r2, [r3, #74]
	add	r2, r5, r2
	strh	r2, [r3, #74]	@ movhi
	ldrh	r2, [r3, #96]
	cmp	r2, r5
	rsbhi	r2, r5, r2
	strhih	r2, [r3, #96]	@ movhi
.L1247:
	ldr	r0, .L1318+64
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r2, #65535
	ldr	r4, .L1318
	cmp	r0, r2
	beq	.L1205
	ldr	r2, [r4, #576]
	mov	r3, r0, asl #1
	ldrh	r2, [r2, r3]
	movw	r3, #3976
	ldrh	r3, [r4, r3]
	cmp	r2, r3
	bhi	.L1205
	mov	r1, #0
	mov	r2, #1
	bl	gc_add_sblk
	add	r10, r10, #1
.L1205:
	add	r0, r7, r10
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1319:
	.align	2
.L1318:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC104
	.word	.LC105
	.word	.LANCHOR0+3380
	.word	.LC106
	.word	.LC107
	.word	.LC108
	.word	.LC109
	.word	.LC110
	.word	.LANCHOR0+564
	.word	.LANCHOR0+4074
	.word	.LANCHOR0+3950
	.word	.LC111
	.word	.LANCHOR0+4072
	.word	.LC112
	.word	.LANCHOR0+4040
	.fnend
	.size	gc_static_wearleveling, .-gc_static_wearleveling
	.align	2
	.global	ftl_get_blk_list_in_sblk
	.type	ftl_get_blk_list_in_sblk, %function
ftl_get_blk_list_in_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r5, #0
	ldr	r6, .L1330
	mov	r10, r0
	ldr	r2, .L1330+4
	mov	r7, r1
	mov	r8, r5
	ldr	r4, [r6, #568]
	mov	r9, r6
	add	r4, r4, r0, asl #2
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
.L1321:
	ldrb	r1, [r6, #3984]	@ zero_extendqisi2
	cmp	r8, r1
	bge	.L1328
	mov	r1, r3, asr r8
	tst	r1, #1
	bne	.L1322
	ldrb	r4, [r9, #4030]	@ zero_extendqisi2
	mov	r0, r8
	str	r2, [sp, #4]
	mov	fp, r5, asl #1
	str	r3, [sp]
	add	r5, r5, #1
	mov	r1, r4
	bl	__aeabi_idiv
	smulbb	r4, r10, r4
	ldrb	r1, [r9, #1700]	@ zero_extendqisi2
	rsb	r1, r1, #24
	ldr	r2, [sp, #4]
	ldr	r3, [sp]
	ldrh	ip, [r2]
	rsb	r1, ip, r1
	add	r0, r4, r0, asl r1
	uxth	r4, r0
	strh	r4, [r7, fp]	@ movhi
	ldrb	r1, [r9, #4030]	@ zero_extendqisi2
	cmp	r1, #1
	subhi	r1, r1, #1
	andhi	r1, r1, r8
	addhi	r4, r4, r1
	strhih	r4, [r7, fp]	@ movhi
.L1322:
	add	r8, r8, #1
	b	.L1321
.L1328:
	add	r7, r7, r5, asl #1
	mov	r0, r5
	mvn	r2, #0
.L1325:
	ldrb	r3, [r6, #3984]	@ zero_extendqisi2
	cmp	r5, r3
	strlth	r2, [r7], #2	@ movhi
	addlt	r5, r5, #1
	blt	.L1325
.L1329:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1331:
	.align	2
.L1330:
	.word	.LANCHOR0
	.word	.LANCHOR0+4028
	.fnend
	.size	ftl_get_blk_list_in_sblk, .-ftl_get_blk_list_in_sblk
	.align	2
	.global	ftl_erase_phy_blk
	.type	ftl_erase_phy_blk, %function
ftl_erase_phy_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	r2, #4028
	ldr	r3, .L1338
	mov	r7, r1
	ldrb	r5, [r3, #1700]	@ zero_extendqisi2
	mov	r4, r3
	ldrh	r2, [r3, r2]
	rsb	r5, r5, #24
	rsb	r5, r2, r5
	mvn	r2, #0
	mov	r6, r0, asr r5
	bic	r5, r0, r2, asl r5
	ldrb	r2, [r3, #4033]	@ zero_extendqisi2
	uxtb	r6, r6
	sxth	r5, r5
	cmp	r2, #0
	beq	.L1333
	ldrb	r3, [r3, #4034]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1333
	movw	r3, #4076
	clz	r1, r1
	ldrh	r2, [r4, r3]
	mov	r0, r6
	mov	r1, r1, lsr #5
	mul	r2, r2, r5
	bl	flash_erase_block_en
.L1333:
	movw	r3, #4076
	mov	r0, r6
	ldrh	r2, [r4, r3]
	uxtb	r1, r7
	mul	r2, r2, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	flash_erase_block_en
.L1339:
	.align	2
.L1338:
	.word	.LANCHOR0
	.fnend
	.size	ftl_erase_phy_blk, .-ftl_erase_phy_blk
	.align	2
	.global	ftl_erase_sblk
	.type	ftl_erase_sblk, %function
ftl_erase_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r0, asl #2
	ldr	r6, .L1368
	.pad #84
	sub	sp, sp, #84
	mov	r8, r0
	mov	r7, r1
	mov	r5, #0
	ldr	r4, [r6, #568]
	mov	r10, r6
	add	r4, r4, r9
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
	str	r3, [sp, #8]
.L1341:
	ldrb	r2, [r6, #4078]	@ zero_extendqisi2
	ldr	r3, .L1368
	cmp	r5, r2
	bge	.L1352
	ldrb	r2, [r10, #4030]	@ zero_extendqisi2
	mov	r4, #0
	sub	r3, r2, #1
	str	r3, [sp, #4]
	ldr	r3, .L1368+4
	mul	lr, r2, r5
	mul	ip, r2, r8
	ldrh	r3, [r3]
	mov	r0, r3
	mov	r3, r4
.L1353:
	cmp	r3, r2
	bge	.L1367
	ldr	fp, [sp, #8]
	add	r1, r3, lr
	mov	r1, fp, asr r1
	tst	r1, #1
	bne	.L1342
	add	r1, sp, #80
	add	fp, r1, r4, asl #2
	ldr	r1, [sp, #4]
	add	r4, r4, #1
	and	r1, r3, r1
	add	r1, ip, r1
	mul	r1, r0, r1
	str	r1, [fp, #-64]
.L1342:
	add	r3, r3, #1
	b	.L1353
.L1367:
	cmp	r2, #4
	bne	.L1365
	uxtb	r3, r5
	uxtb	ip, r7
	mov	fp, #0
.L1344:
	cmp	fp, r4
	beq	.L1347
	mov	r0, r3
	str	r3, [sp, #12]
	add	r3, sp, #16
	mov	r1, ip
	str	ip, [sp, #4]
	ldr	r2, [r3, fp, asl #2]
	add	fp, fp, #1
	bl	flash_erase_block_en
	ldr	ip, [sp, #4]
	ldr	r3, [sp, #12]
	b	.L1344
.L1365:
	cmp	r4, #2
	bne	.L1348
	ldrb	r3, [r6, #4033]	@ zero_extendqisi2
	uxtb	r4, r5
	cmp	r3, #0
	beq	.L1349
	ldrb	r3, [r10, #4034]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1349
	clz	r1, r7
	mov	r0, r4
	ldr	r2, [sp, #16]
	mov	r1, r1, lsr #5
	ldr	r3, [sp, #20]
	bl	flash_erase_duplane_block
.L1349:
	mov	r0, r4
	uxtb	r1, r7
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #20]
	bl	flash_erase_duplane_block
	b	.L1347
.L1348:
	cmp	r4, #1
	bne	.L1347
	ldrb	r3, [r6, #4033]	@ zero_extendqisi2
	uxtb	r4, r5
	cmp	r3, #0
	beq	.L1351
	ldrb	r3, [r10, #4034]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1351
	clz	r1, r7
	mov	r0, r4
	ldr	r2, [sp, #16]
	mov	r1, r1, lsr #5
	bl	flash_erase_block_en
.L1351:
	mov	r0, r4
	uxtb	r1, r7
	ldr	r2, [sp, #16]
	bl	flash_erase_block_en
.L1347:
	add	r5, r5, #1
	b	.L1341
.L1352:
	cmp	r7, #0
	ldr	r0, [r3, #568]
	bne	.L1354
	ldrh	r2, [r0, r9]
	ubfx	r1, r2, #0, #11
	add	r1, r1, #1
	bfi	r2, r1, #0, #11
	strh	r2, [r0, r9]	@ movhi
	ldr	r2, [r3, #3348]
	ldr	r3, [r3, #568]
	ldr	r1, [r2, #84]
	ldrh	r0, [r2, #96]
	add	r1, r1, #1
	str	r1, [r2, #84]
	ldrh	r3, [r3, r9]
	ubfx	r3, r3, #0, #11
	uxth	r1, r3
	cmp	r0, r1
	strlth	r3, [r2, #96]	@ movhi
	b	.L1356
.L1354:
	ldr	r2, [r0, r8, asl #2]
	ubfx	r1, r2, #11, #8
	add	r1, r1, #1
	bfi	r2, r1, #11, #8
	str	r2, [r0, r8, asl #2]
	ldr	r2, [r3, #3348]
	ldr	r3, [r3, #568]
	ldr	r1, [r2, #80]
	add	r1, r1, #1
	str	r1, [r2, #80]
	ldr	r3, [r3, r8, asl #2]
	ldrh	r1, [r2, #98]
	ubfx	r3, r3, #11, #8
	cmp	r1, r3
	strcch	r3, [r2, #98]	@ movhi
.L1356:
	mov	r0, #0
	add	sp, sp, #84
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1369:
	.align	2
.L1368:
	.word	.LANCHOR0
	.word	.LANCHOR0+4076
	.fnend
	.size	ftl_erase_sblk, .-ftl_erase_sblk
	.align	2
	.global	ftl_alloc_sys_blk
	.type	ftl_alloc_sys_blk, %function
ftl_alloc_sys_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1381
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r2, [r3, #3348]
	mov	r4, r3
	ldrh	r1, [r2, #136]
	cmp	r1, #63
	movhi	r3, #0
	strhih	r3, [r2, #136]	@ movhi
	ldrh	r3, [r2, #112]
	cmp	r3, #0
	bne	.L1372
	ldr	r1, .L1381+4
	movw	r2, #1140
	ldr	r0, .L1381+8
	bl	printk
	bl	dump_stack
.L1372:
	ldr	r3, [r4, #3348]
	movw	lr, #65535
	mov	ip, #0
.L1376:
	ldrh	r2, [r3, #136]
	add	r1, r2, #79
	add	r1, r3, r1, asl #1
.L1373:
	cmp	r2, #63
	bgt	.L1380
	ldrh	r0, [r1, #2]!
	cmp	r0, lr
	bne	.L1378
	add	r2, r2, #1
	b	.L1373
.L1380:
	strh	ip, [r3, #136]	@ movhi
	b	.L1376
.L1378:
	add	r1, r3, r2, asl #1
	mvn	ip, #0
	strh	ip, [r1, #160]	@ movhi
	strh	r2, [r3, #136]	@ movhi
	ldrh	r2, [r3, #112]
	add	r2, r2, ip
	strh	r2, [r3, #112]	@ movhi
	ldmfd	sp!, {r4, pc}
.L1382:
	.align	2
.L1381:
	.word	.LANCHOR0
	.word	.LANCHOR1+1232
	.word	.LC0
	.fnend
	.size	ftl_alloc_sys_blk, .-ftl_alloc_sys_blk
	.align	2
	.global	ftl_free_sys_blk
	.type	ftl_free_sys_blk, %function
ftl_free_sys_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r5, r0
	ldr	r3, .L1393
	ldr	r2, [r3, #3348]
	mov	r4, r3
	ldrh	r1, [r2, #138]
	cmp	r1, #63
	movhi	r3, #0
	strhih	r3, [r2, #138]	@ movhi
	ldrh	r3, [r2, #112]
	cmp	r3, #63
	bls	.L1385
	ldr	r1, .L1393+4
	movw	r2, #1164
	ldr	r0, .L1393+8
	bl	printk
	bl	dump_stack
.L1385:
	ldr	r3, [r4, #3348]
	movw	ip, #65535
	mov	r0, #0
.L1389:
	ldrh	r2, [r3, #138]
	add	r1, r2, #79
	add	r1, r3, r1, asl #1
.L1386:
	cmp	r2, #63
	bgt	.L1392
	ldrh	lr, [r1, #2]!
	cmp	lr, ip
	bne	.L1387
	add	r1, r3, r2, asl #1
	strh	r5, [r1, #160]	@ movhi
	strh	r2, [r3, #138]	@ movhi
	ldrh	r2, [r3, #112]
	add	r2, r2, #1
	strh	r2, [r3, #112]	@ movhi
	ldmfd	sp!, {r3, r4, r5, pc}
.L1387:
	add	r2, r2, #1
	b	.L1386
.L1392:
	strh	r0, [r3, #138]	@ movhi
	b	.L1389
.L1394:
	.align	2
.L1393:
	.word	.LANCHOR0
	.word	.LANCHOR1+1252
	.word	.LC0
	.fnend
	.size	ftl_free_sys_blk, .-ftl_free_sys_blk
	.align	2
	.global	ftl_info_data_recovery
	.type	ftl_info_data_recovery, %function
ftl_info_data_recovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r3, #65535
	ldrh	r6, [r0]
	cmp	r6, r3
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, pc}
	ldr	r4, .L1405
	mov	r8, r6, asl #2
	ldr	r5, [r4, #568]
	add	r7, r5, r8
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	tst	r3, #224
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	ldrb	r2, [r0, #4]	@ zero_extendqisi2
	mov	r0, r6
	bfi	r3, r2, #5, #3
	strb	r3, [r7, #2]
	bl	zftl_remove_free_node
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	ldr	r2, [r4, #3348]
	tst	r3, #8
	ldrneh	r3, [r2, #116]
	subne	r3, r3, #1
	strneh	r3, [r2, #116]	@ movhi
	bne	.L1400
	tst	r3, #24
	ldreqh	r3, [r2, #114]
	ldrneh	r3, [r2, #118]
	subeq	r3, r3, #1
	subne	r3, r3, #1
	streqh	r3, [r2, #114]	@ movhi
	strneh	r3, [r2, #118]	@ movhi
.L1400:
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #160
	bne	.L1402
	ldr	r3, [r5, r6, asl #2]
	ubfx	r2, r3, #11, #8
	add	r2, r2, #1
	bfi	r3, r2, #11, #8
	str	r3, [r5, r6, asl #2]
	ldr	r2, [r4, #3348]
	ldrh	r3, [r2, #120]
	sub	r3, r3, #1
	strh	r3, [r2, #120]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1402:
	ldrh	r2, [r5, r8]
	cmp	r3, #64
	ubfx	r1, r2, #0, #11
	add	r1, r1, #1
	bfi	r2, r1, #0, #11
	strh	r2, [r5, r8]	@ movhi
	bne	.L1403
	ldr	r2, [r4, #3348]
	ldrh	r3, [r2, #122]
	sub	r3, r3, #1
	strh	r3, [r2, #122]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1403:
	cmp	r3, #96
	ldreq	r2, [r4, #3348]
	ldreqh	r3, [r2, #124]
	subeq	r3, r3, #1
	streqh	r3, [r2, #124]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1406:
	.align	2
.L1405:
	.word	.LANCHOR0
	.fnend
	.size	ftl_info_data_recovery, .-ftl_info_data_recovery
	.align	2
	.global	ftl_get_ppa_from_index
	.type	ftl_get_ppa_from_index, %function
ftl_get_ppa_from_index:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	movw	r3, #3976
	ldr	r6, .L1412
	mov	r4, r0
	ldrh	r3, [r6, r3]
	ldrb	r2, [r6, #3984]	@ zero_extendqisi2
	ldr	r5, [r6, #580]
	mul	r1, r2, r3
	cmp	r0, r1
	addlt	r5, r5, #16
	addge	r5, r5, #48
	smulbbge	r3, r2, r3
	ldrb	r7, [r5, #9]	@ zero_extendqisi2
	rsbge	r4, r3, r0
	mov	r1, r7
	uxthge	r4, r4
	mov	r0, r4
	bl	__aeabi_idiv
	movw	r3, #65535
	smulbb	r7, r0, r7
	mov	r8, r0
	rsb	r4, r7, r4
	uxth	r4, r4
	add	r5, r5, r4, asl #1
	ldrh	r4, [r5, #16]
	cmp	r4, r3
	bne	.L1410
	ldr	r1, .L1412+4
	movw	r2, #1686
	ldr	r0, .L1412+8
	bl	printk
	bl	dump_stack
.L1410:
	movw	r3, #4076
	ldrh	r0, [r6, r3]
	mla	r0, r0, r4, r8
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1413:
	.align	2
.L1412:
	.word	.LANCHOR0
	.word	.LANCHOR1+1272
	.word	.LC0
	.fnend
	.size	ftl_get_ppa_from_index, .-ftl_get_ppa_from_index
	.align	2
	.global	lpa_hash_get_ppa
	.type	lpa_hash_get_ppa, %function
lpa_hash_get_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	uxtb	r3, r0
	ldr	r2, .L1420
	str	lr, [sp, #-4]!
	.save {lr}
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	ldr	r2, .L1420+4
	ldr	r1, [r2, #-3592]
	ldr	ip, [r2, #-3588]
	movw	r2, #65535
.L1415:
	cmp	r3, r2
	beq	.L1419
	ldr	lr, [r1, r3, asl #2]
	cmp	lr, r0
	bne	.L1416
	mov	r0, r3
	ldr	lr, [sp], #4
	b	ftl_get_ppa_from_index
.L1416:
	mov	r3, r3, asl #1
	ldrh	r3, [ip, r3]
	b	.L1415
.L1419:
	mvn	r0, #0
	ldr	pc, [sp], #4
.L1421:
	.align	2
.L1420:
	.word	.LANCHOR0+4080
	.word	.LANCHOR3
	.fnend
	.size	lpa_hash_get_ppa, .-lpa_hash_get_ppa
	.align	2
	.global	ftl_get_new_free_page
	.type	ftl_get_new_free_page, %function
ftl_get_new_free_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	movw	r3, #65535
	ldrh	r2, [r0]
	mov	r4, r0
	cmp	r2, r3
	bne	.L1423
	ldr	r1, .L1432
	movw	r2, #1809
	ldr	r0, .L1432+4
	bl	printk
	bl	dump_stack
.L1423:
	ldr	r5, .L1432+8
	movw	r3, #3976
	ldrh	r2, [r4, #2]
	ldrh	r3, [r5, r3]
	cmp	r2, r3
	bne	.L1424
	ldr	r1, .L1432
	movw	r2, #1810
	ldr	r0, .L1432+4
	bl	printk
	bl	dump_stack
.L1424:
	ldrh	r3, [r4, #6]
	cmp	r3, #0
	bne	.L1425
	ldr	r1, .L1432
	movw	r2, #1811
	ldr	r0, .L1432+4
	bl	printk
	bl	dump_stack
.L1425:
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	movw	r2, #65535
	ldrb	r0, [r5, #3984]	@ zero_extendqisi2
	mov	ip, #0
	add	r3, r4, r3, asl #1
	ldrh	r1, [r3, #16]
.L1426:
	cmp	r1, r2
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	bne	.L1431
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r4, #5]
	cmp	r3, r0
	streqb	ip, [r4, #5]
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	add	r3, r4, r3, asl #1
	ldrh	r1, [r3, #16]
	b	.L1426
.L1431:
	movw	r0, #4076
	add	r3, r3, #1
	ldrh	r0, [r5, r0]
	uxtb	r3, r3
	ldrh	r2, [r4, #2]
	strb	r3, [r4, #5]
	mul	r0, r0, r1
	ldrh	r1, [r4, #6]
	sub	r1, r1, #1
	strh	r1, [r4, #6]	@ movhi
	ldrh	r1, [r4, #10]
	orr	r0, r0, r2
	add	r1, r1, #1
	strh	r1, [r4, #10]	@ movhi
	ldrb	r1, [r5, #3984]	@ zero_extendqisi2
	cmp	r1, r3
	addeq	r2, r2, #1
	streqh	r2, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [r4, #5]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1433:
	.align	2
.L1432:
	.word	.LANCHOR1+1296
	.word	.LC0
	.word	.LANCHOR0
	.fnend
	.size	ftl_get_new_free_page, .-ftl_get_new_free_page
	.align	2
	.global	ftl_ext_alloc_new_blk
	.type	ftl_ext_alloc_new_blk, %function
ftl_ext_alloc_new_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	bl	ftl_alloc_sys_blk
	movw	r2, #65533
	sub	r3, r0, #1
	mov	r4, r0
	uxth	r3, r3
	cmp	r3, r2
	bls	.L1435
	ldr	r1, .L1437
	movw	r2, #1842
	ldr	r0, .L1437+4
	bl	printk
	bl	dump_stack
.L1435:
	ldr	r5, .L1437+8
	mov	r1, #0
	mov	r0, r4
	bl	ftl_erase_phy_blk
	ldr	r3, [r5, #3348]
	ldrh	r0, [r3, #130]
	bl	ftl_free_sys_blk
	ldr	r3, [r5, #3348]
	mov	r0, #0
	strh	r4, [r3, #130]	@ movhi
	strh	r0, [r3, #140]	@ movhi
	ldmfd	sp!, {r3, r4, r5, pc}
.L1438:
	.align	2
.L1437:
	.word	.LANCHOR1+1320
	.word	.LC0
	.word	.LANCHOR0
	.fnend
	.size	ftl_ext_alloc_new_blk, .-ftl_ext_alloc_new_blk
	.align	2
	.global	ftl_total_vpn_update
	.type	ftl_total_vpn_update, %function
ftl_total_vpn_update:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1452
	ldrh	r3, [r2]
	cmp	r3, #4
	cmpls	r0, #0
	addeq	r3, r3, #1
	streqh	r3, [r2]	@ movhi
	bxeq	lr
	ldr	r3, .L1452+4
	mov	r0, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	ip, r3, #564
	strh	r0, [r2]	@ movhi
	movw	r6, #65535
	ldrh	lr, [ip]
	mov	ip, r0
	ldr	r1, [r3, #576]
	ldr	r2, [r3, #568]
	sub	r1, r1, #2
	add	lr, r2, lr, asl #2
.L1442:
	cmp	r2, lr
	beq	.L1451
	ldrh	r4, [r1, #2]!
	cmp	r4, r6
	beq	.L1443
	ldrb	r5, [r2, #2]	@ zero_extendqisi2
	and	r5, r5, #224
	cmp	r5, #160
	addeq	r0, r0, r4
	addne	ip, ip, r4
.L1443:
	add	r2, r2, #4
	b	.L1442
.L1451:
	ldr	r4, [r3, #580]
	ldr	r3, [r3, #3348]
	str	ip, [r4, #524]
	str	r0, [r4, #528]
	ldrh	r1, [r3, #120]
	cmp	r1, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	bl	__aeabi_uidiv
	str	r0, [r4, #532]
	ldmfd	sp!, {r4, r5, r6, pc}
.L1453:
	.align	2
.L1452:
	.word	.LANCHOR3-3584
	.word	.LANCHOR0
	.fnend
	.size	ftl_total_vpn_update, .-ftl_total_vpn_update
	.align	2
	.global	ftl_debug_info_fill
	.type	ftl_debug_info_fill, %function
ftl_debug_info_fill:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r0, #0
	bx	lr
	.fnend
	.size	ftl_debug_info_fill, .-ftl_debug_info_fill
	.align	2
	.global	ftl_vpn_update
	.type	ftl_vpn_update, %function
ftl_vpn_update:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	bl	zftl_list_update_data_list
	ldr	r3, .L1459
	mov	r4, r4, asl #1
	ldr	r2, [r3, #576]
	ldrh	r2, [r2, r4]
	cmp	r2, #0
	moveq	r0, #1
	movne	r0, #0
	streq	r0, [r3, #3364]
	ldmfd	sp!, {r4, pc}
.L1460:
	.align	2
.L1459:
	.word	.LANCHOR0
	.fnend
	.size	ftl_vpn_update, .-ftl_vpn_update
	.align	2
	.global	ftl_vpn_decrement
	.type	ftl_vpn_decrement, %function
ftl_vpn_decrement:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	movw	r3, #65535
	cmp	r0, r3
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldr	r6, .L1471
	beq	.L1462
	ldr	r2, [r6, #576]
	mov	r3, r0, asl #1
	ldrh	r4, [r2, r3]
	cmp	r4, #0
	subne	r4, r4, #1
	strneh	r4, [r2, r3]	@ movhi
	bne	.L1462
	ldr	r3, [r6, #568]
	mov	r1, r5
	mov	r2, r4
	add	r3, r3, r0, asl #2
	ldr	r0, .L1471+4
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	mov	r3, r3, lsr #5
	bl	printk
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1462:
	movw	r3, #4010
	ldrh	r0, [r6, r3]
	cmp	r0, r5
	beq	.L1468
	movw	r2, #65535
	cmp	r0, r2
	ldreq	r2, .L1471
	streqh	r5, [r2, r3]	@ movhi
	beq	.L1468
	bl	ftl_vpn_update
	ldr	r3, .L1471+8
	ldr	r1, .L1471+12
	ldrh	r2, [r3, #-14]
	add	r2, r2, #1
	uxth	r2, r2
	adds	r0, r0, #0
	movne	r0, #1
	cmp	r2, #7
	movhi	r2, #0
	strh	r2, [r3, #-14]	@ movhi
	ldrh	r3, [r3, #-14]
	movw	r2, #4010
	ldrh	ip, [r6, r2]
	strh	r5, [r6, r2]	@ movhi
	mov	r3, r3, asl #1
	strh	ip, [r1, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L1468:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1472:
	.align	2
.L1471:
	.word	.LANCHOR0
	.word	.LC113
	.word	.LANCHOR3-3568
	.word	.LANCHOR0+4012
	.fnend
	.size	ftl_vpn_decrement, .-ftl_vpn_decrement
	.align	2
	.global	lpa_hash_update_ppa
	.type	lpa_hash_update_ppa, %function
lpa_hash_update_ppa:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	uxtb	r5, r0
	ldr	r3, .L1485
	movw	lr, #65535
	ldr	r6, .L1485+4
	mov	r5, r5, asl #1
	mov	r8, lr
	ldr	r7, [r3, #-3592]
	ldrh	ip, [r6, r5]
	ldr	r9, [r3, #-3588]
.L1474:
	cmp	ip, r8
	beq	.L1478
	ldr	r4, [r7, ip, asl #2]
	add	r10, r7, ip, asl #2
	cmp	r4, r0
	mov	r4, ip, asl #1
	bne	.L1475
	mvn	ip, #0
	str	ip, [r10]
	movw	ip, #65535
	cmp	lr, ip
	ldr	ip, [r3, #-3588]
	movne	lr, lr, asl #1
	ldreqh	ip, [ip, r4]
	ldrneh	r7, [ip, r4]
	streqh	ip, [r6, r5]	@ movhi
	strneh	r7, [ip, lr]	@ movhi
	mvn	lr, #0
	ldr	ip, [r3, #-3588]
	strh	lr, [ip, r4]	@ movhi
	b	.L1478
.L1475:
	mov	lr, ip
	ldrh	ip, [r9, r4]
	b	.L1474
.L1478:
	ldr	ip, [r3, #-3592]
	cmn	r1, #1
	str	r0, [ip, r2, asl #2]
	ldrh	r0, [r6, r5]
	ldr	r3, [r3, #-3588]
	strh	r2, [r6, r5]	@ movhi
	mov	r2, r2, asl #1
	ldr	ip, .L1485+8
	strh	r0, [r3, r2]	@ movhi
	beq	.L1480
	movw	r3, #4028
	ldrh	r0, [ip, r3]
	ldrb	r3, [ip, #1700]	@ zero_extendqisi2
	mov	r1, r1, lsr r0
	rsb	r3, r3, #24
	rsb	r3, r0, r3
	mvn	r0, #0
	bic	r0, r1, r0, asl r3
	ldrb	r1, [ip, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	uxth	r0, r0
	bl	ftl_vpn_decrement
.L1480:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L1486:
	.align	2
.L1485:
	.word	.LANCHOR3
	.word	.LANCHOR0+4080
	.word	.LANCHOR0
	.fnend
	.size	lpa_hash_update_ppa, .-lpa_hash_update_ppa
	.align	2
	.global	ftl_mask_bad_block
	.type	ftl_mask_bad_block, %function
ftl_mask_bad_block:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r3, #1
	ldr	r1, .L1495
	movw	ip, #4028
	ldrb	r4, [r1, #1700]	@ zero_extendqisi2
	ldrb	r5, [r1, #4030]	@ zero_extendqisi2
	rsb	r2, r4, #24
	ldrh	r1, [r1, ip]
	mov	r4, r3, asl r4
	sub	r4, r4, #1
	and	r4, r4, r0, lsr r2
	rsb	r2, r1, r2
	mov	r3, r3, asl r2
	sub	r3, r3, #1
	uxtb	r4, r4
	and	r0, r3, r0, lsr r1
	mov	r1, r5
	smulbb	r6, r4, r5
	uxth	r8, r0
	bl	__aeabi_uidiv
	cmp	r5, #1
	subhi	r3, r5, #1
	uxtb	r6, r6
	andhi	r3, r3, r8
	uxth	r7, r0
	addhi	r3, r6, r3
	uxtbhi	r6, r3
	ldr	r3, .L1495+4
	ldr	r3, [r3]
	tst	r3, #16384
	beq	.L1489
	str	r8, [sp]
	mov	r1, r4
	ldr	r0, .L1495+8
	mov	r2, r6
	mov	r3, r7
	bl	printk
.L1489:
	ldr	r3, .L1495
	add	r2, r3, #564
	ldrh	r2, [r2]
	cmp	r2, r7
	ldrhi	r4, [r3, #568]
	movhi	r2, #1
	addhi	r4, r4, r7, asl #2
	ldrhib	r3, [r4, #3]	@ zero_extendqisi2
	orrhi	r6, r3, r2, asl r6
	strhib	r6, [r4, #3]
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1496:
	.align	2
.L1495:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC114
	.fnend
	.size	ftl_mask_bad_block, .-ftl_mask_bad_block
	.align	2
	.global	gc_free_bad_sblk
	.type	gc_free_bad_sblk, %function
gc_free_bad_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r10, .L1512
	ldr	r3, [r10, #3960]
	cmp	r3, #0
	beq	.L1508
	ldr	r7, .L1512+4
	mov	r9, r0
	mov	fp, #0
	mov	r8, r7
.L1498:
	ldrb	r3, [r10, #3984]	@ zero_extendqisi2
	uxth	r0, fp
	ldr	r5, .L1512
	cmp	r3, r0
	bls	.L1508
	ldrb	r6, [r5, #4030]	@ zero_extendqisi2
	mov	r1, r6
	uxth	r4, r6
	bl	__aeabi_idiv
	ldr	r2, .L1512+8
	ldrb	r3, [r5, #1700]	@ zero_extendqisi2
	cmp	r6, #1
	smulbb	r5, r9, r4
	ldrh	r1, [r2]
	rsb	r3, r3, #24
	subhi	r4, r4, #1
	rsb	r3, r1, r3
	andhi	r4, r4, fp
	ldr	ip, .L1512
	mov	r6, r2
	mov	r2, #0
	add	r0, r5, r0, asl r3
	uxth	r5, r0
	addhi	r5, r5, r4
	uxthhi	r5, r5
.L1500:
	ldr	r3, [r10, #3960]
	uxth	r4, r2
	cmp	r4, r3
	bcs	.L1510
	add	r3, r4, #292
	mov	r3, r3, asl #1
	ldrh	r3, [r7, r3]
	cmp	r3, r5
	bne	.L1501
	mov	r1, r5
	ldr	r0, .L1512+12
	stmia	sp, {r2, ip}
	bl	printk
	ldrh	r0, [r6]
	mov	r0, r5, asl r0
	bl	ftl_mask_bad_block
	ldr	ip, [sp, #4]
	ldr	r2, [sp]
	ldr	r1, [ip, #3960]
.L1502:
	cmp	r4, r1
	bcs	.L1511
	add	r3, r4, #292
	add	r3, r3, #1
	mov	r3, r3, asl #1
	ldrh	r0, [r8, r3]
	add	r3, r4, #292
	add	r4, r4, #1
	mov	r3, r3, asl #1
	uxth	r4, r4
	strh	r0, [r8, r3]	@ movhi
	b	.L1502
.L1511:
	sub	r1, r1, #1
	str	r1, [r10, #3960]
.L1501:
	add	r2, r2, #1
	b	.L1500
.L1510:
	add	fp, fp, #1
	b	.L1498
.L1508:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1513:
	.align	2
.L1512:
	.word	.LANCHOR0
	.word	.LANCHOR0+3380
	.word	.LANCHOR0+4028
	.word	.LC115
	.fnend
	.size	gc_free_bad_sblk, .-gc_free_bad_sblk
	.align	2
	.global	ftl_free_sblk
	.type	ftl_free_sblk, %function
ftl_free_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r2, r0, asl #2
	ldr	r5, .L1536
	.pad #20
	sub	sp, sp, #20
	mov	r6, r0
	ldr	r7, [r5, #568]
	add	r9, r7, r2
	ldrb	r4, [r9, #2]	@ zero_extendqisi2
	tst	r4, #8
	mov	r3, r4, lsr #5
	str	r3, [sp]
	beq	.L1516
	ldr	r8, [r5, #3348]
	add	r1, r5, #572
	ldrh	fp, [r7, r2]
	ldr	r10, [r7, r0, asl #2]
	ldrh	ip, [r8, #74]
	ubfx	fp, fp, #0, #11
	ldrh	r3, [r8, #72]
	ubfx	r10, r10, #11, #8
	add	ip, fp, ip
	ldrh	r8, [r1]
	add	r3, r10, r3
	str	r2, [sp, #12]
	uxth	ip, ip
	str	ip, [sp, #4]
	mov	r1, r8
	uxth	r3, r3
	mov	r0, ip
	str	r3, [sp, #8]
	bl	__aeabi_uidiv
	ldr	r1, .L1536+4
	ldrh	lr, [r1, #-12]
	ldr	r3, [sp, #8]
	ldr	ip, [sp, #4]
	uxtah	r0, r3, r0
	ldr	r2, [sp, #12]
	cmp	r0, lr
	ble	.L1517
	movw	r3, #3342
	ldrh	r0, [r5, r3]
	movw	r3, #3334
	ldrh	r3, [r5, r3]
	add	r0, r0, r3
	add	r3, r5, #4048
	ldrh	r3, [r3]
	add	r3, r3, #8
	cmp	r0, r3
	blt	.L1523
	movw	r3, #3338
	movw	r0, #3332
	ldrh	r0, [r5, r0]
	ldrh	r3, [r5, r3]
	ldrh	r1, [r1, #-10]
	add	r3, r3, r0
	movw	r0, #3340
	ldrh	r0, [r5, r0]
	add	r1, r1, #8
	add	r3, r3, r0
	b	.L1535
.L1517:
	mla	r3, r8, r3, ip
	ldrh	r0, [r1, #-8]
	cmp	r3, r0
	ble	.L1519
	movw	r3, #3338
	movw	r0, #3332
	ldrh	r0, [r5, r0]
	ldrh	r3, [r5, r3]
	ldrh	r1, [r1, #-10]
	add	r3, r3, r0
	movw	r0, #3340
	ldrh	r0, [r5, r0]
	add	r3, r3, r0
	add	r0, r1, #8
	cmp	r3, r0
	blt	.L1533
	movw	r0, #3342
	ldrh	ip, [r5, r0]
	movw	r0, #3334
	ldrh	r0, [r5, r0]
	add	r5, r5, #4048
	add	r0, ip, r0
	ldrh	ip, [r5]
	add	ip, ip, #8
	cmp	r0, ip
	blt	.L1523
	add	r1, r1, #24
.L1535:
	cmp	r3, r1
	bge	.L1523
.L1533:
	bfc	r4, #3, #2
	b	.L1532
.L1523:
	mov	r3, #2
	bfi	r4, r3, #3, #2
.L1532:
	strb	r4, [r9, #2]
.L1519:
	ldrb	r3, [r9, #2]	@ zero_extendqisi2
	ands	r3, r3, #24
	bne	.L1524
	mul	r8, r8, r10
	ldrh	r3, [r7, r2]
	mov	r10, r10, lsr #3
	add	r8, r8, r8, asl #1
	add	fp, fp, r8, asr #2
	bfi	r3, fp, #0, #11
	strh	r3, [r7, r2]	@ movhi
	ldr	r4, [r7, r6, asl #2]
	bfi	r4, r10, #11, #8
	str	r4, [r7, r6, asl #2]
	b	.L1516
.L1524:
	cmp	r3, #16
	bne	.L1516
	mov	r0, fp
	mov	r1, r8
	str	r2, [sp, #4]
	bl	__aeabi_idiv
	ldr	r3, [r7, r6, asl #2]
	mov	fp, fp, asr #5
	add	r0, r0, r0, asl #1
	add	r10, r10, r0, asr #2
	bfi	r3, r10, #11, #8
	str	r3, [r7, r6, asl #2]
	ldr	r2, [sp, #4]
	ldrh	r3, [r7, r2]
	bfi	r3, fp, #0, #11
	strh	r3, [r7, r2]	@ movhi
.L1516:
	mov	r0, r6
	str	r2, [sp, #4]
	bl	zftl_remove_data_node
	ldr	r3, .L1536
	mov	r0, #0
	ldr	r1, [r3, #568]
	mov	r4, r3
	ldr	r2, [sp, #4]
	add	r2, r1, r2
	ldrb	r1, [r2, #2]	@ zero_extendqisi2
	bfc	r1, #5, #3
	strb	r1, [r2, #2]
	ldr	r1, [r3, #576]
	mov	r2, r6, asl #1
	strh	r0, [r1, r2]	@ movhi
	ldr	r2, [sp]
	add	r1, r2, #6
	and	r1, r1, #7
	cmp	r1, #4
	bhi	.L1526
	mov	r0, r6
	bl	gc_free_bad_sblk
.L1526:
	ldrb	r3, [r9, #2]	@ zero_extendqisi2
	tst	r3, #8
	beq	.L1527
	ldr	r3, [r4, #580]
	movw	r2, #586
	ldrh	r1, [r3, r2]
	cmp	r1, r6
	bne	.L1527
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #590
	movw	r1, #65535
	ldrh	r0, [r3, r2]
	cmp	r0, r1
	bne	.L1527
	strh	r6, [r3, r2]	@ movhi
	mov	r1, r6
	ldr	r0, .L1536+8
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	printk
.L1527:
	mov	r0, r6
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	zftl_insert_free_list
.L1537:
	.align	2
.L1536:
	.word	.LANCHOR0
	.word	.LANCHOR3-3568
	.word	.LC116
	.fnend
	.size	ftl_free_sblk, .-ftl_free_sblk
	.align	2
	.global	gc_free_src_blk
	.type	gc_free_src_blk, %function
gc_free_src_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, #0
	ldr	r6, .L1577
	ldr	r8, .L1577+4
	mov	r7, r6
.L1539:
	ldrh	r2, [r8, #52]
	uxth	r3, r5
	cmp	r2, r3
	bls	.L1576
	add	r3, r8, r3, asl #1
	ldrh	r4, [r3, #54]
	ldr	r3, [r6, #576]
	mov	r9, r4, asl #1
	ldrh	r2, [r3, r9]
	cmp	r2, #0
	beq	.L1540
	ldr	r0, .L1577+8
	mov	r1, r4
	bl	printk
.L1540:
	ldr	r3, [r7, #576]
	mov	r2, #0
	strh	r2, [r3, r9]	@ movhi
	ldr	r3, [r7, #576]
	ldrh	r3, [r3, r9]
	cmp	r3, r2
	bne	.L1541
	ldr	r3, .L1577+12
	ldr	r9, [r6, #568]
	ldr	r3, [r3]
	add	r9, r9, r4, asl #2
	tst	r3, #256
	beq	.L1542
	ldrb	r2, [r9, #2]	@ zero_extendqisi2
	mov	r1, r4
	ldr	r0, .L1577+16
	mov	r2, r2, lsr #5
	bl	printk
.L1542:
	ldrb	r3, [r9, #2]	@ zero_extendqisi2
	tst	r3, #192
	and	r3, r3, #224
	moveq	r2, #1
	movne	r2, #0
	cmp	r3, #224
	movne	r3, r2
	orreq	r3, r2, #1
	cmp	r3, #0
	beq	.L1543
	ldr	r1, .L1577+20
	movw	r2, #1115
	ldr	r0, .L1577+24
	bl	printk
	bl	dump_stack
.L1543:
	mov	r0, r4
	bl	ftl_free_sblk
	ldr	r3, [r7, #580]
	ldrh	r1, [r3, #124]
	cmp	r1, #0
	beq	.L1544
	add	r0, r3, #388
	mov	r2, #0
	add	r0, r0, #2
.L1546:
	ldrh	ip, [r0, #2]!
	cmp	ip, r4
	bne	.L1545
	add	r2, r2, #196
	mvn	r0, #0
	add	r1, r1, r0
	mov	r2, r2, asl #1
	strh	r0, [r3, r2]	@ movhi
	strh	r1, [r3, #124]	@ movhi
	b	.L1544
.L1545:
	add	r2, r2, #1
	cmp	r2, #64
	bne	.L1546
.L1544:
	ldrh	r1, [r3, #120]
	cmp	r1, #0
	beq	.L1547
	add	r0, r3, #134
	mov	r2, #0
.L1549:
	ldrh	ip, [r0, #2]!
	cmp	ip, r4
	bne	.L1548
	add	r2, r3, r2, asl #1
	mvn	r0, #0
	add	r1, r1, r0
	strh	r0, [r2, #136]	@ movhi
	strh	r1, [r3, #120]	@ movhi
	b	.L1547
.L1548:
	add	r2, r2, #1
	cmp	r2, #64
	bne	.L1549
.L1547:
	ldrh	r0, [r3, #122]
	cmp	r0, #0
	beq	.L1550
	add	r1, r3, #260
	mov	r2, #0
	add	r1, r1, #2
.L1552:
	ldrh	ip, [r1, #2]!
	cmp	ip, r4
	bne	.L1551
	add	r2, r2, #132
	mvn	r1, #0
	add	r0, r0, r1
	mov	r2, r2, asl #1
	strh	r1, [r3, r2]	@ movhi
	strh	r0, [r3, #122]	@ movhi
	b	.L1550
.L1551:
	add	r2, r2, #1
	cmp	r2, #64
	bne	.L1552
	b	.L1550
.L1541:
	mov	r0, r4
	mov	r1, #1
	bl	gc_add_sblk
.L1550:
	add	r5, r5, #1
	b	.L1539
.L1576:
	ldr	r3, .L1577+4
	mov	r2, #0
	strh	r2, [r3, #52]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L1578:
	.align	2
.L1577:
	.word	.LANCHOR0
	.word	.LANCHOR0+3380
	.word	.LC117
	.word	.LANCHOR2
	.word	.LC118
	.word	.LANCHOR1+1344
	.word	.LC0
	.fnend
	.size	gc_free_src_blk, .-gc_free_src_blk
	.align	2
	.global	print_ftl_debug_info
	.type	print_ftl_debug_info, %function
print_ftl_debug_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	movw	ip, #3334
	ldr	r4, .L1581
	.pad #32
	sub	sp, sp, #32
	ldr	r3, [r4, #3348]
	ldrh	ip, [r4, ip]
	ldr	r0, [r4, #580]
	ldrh	r1, [r3, #148]
	ldrh	r2, [r3, #146]
	movw	r3, #3332
	str	ip, [sp]
	movw	ip, #3336
	ldrh	ip, [r4, ip]
	ldrh	r3, [r4, r3]
	str	ip, [sp, #4]
	movw	ip, #3338
	ldrh	ip, [r4, ip]
	str	ip, [sp, #8]
	movw	ip, #3340
	ldrh	ip, [r4, ip]
	str	ip, [sp, #12]
	movw	ip, #3342
	ldrh	ip, [r4, ip]
	str	ip, [sp, #16]
	ldr	ip, [r0, #524]
	str	ip, [sp, #20]
	ldr	r0, [r0, #528]
	str	r0, [sp, #24]
	ldr	r0, [r4, #3328]
	str	r0, [sp, #28]
	ldr	r0, .L1581+4
	bl	printk
	ldr	r2, [r4, #3348]
	ldrb	r3, [r4, #3345]	@ zero_extendqisi2
	ldr	r0, .L1581+8
	ldr	r1, [r2, #8]
	str	r1, [sp]
	ldr	r1, [r2, #64]
	str	r1, [sp, #4]
	ldr	r1, [r2, #20]
	ldr	r2, [r2, #28]
	bl	printk
	ldr	r2, [r4, #3348]
	ldr	r0, .L1581+12
	ldr	r3, [r2, #16]
	ldr	r1, [r2, #52]
	ldr	r2, [r2, #60]
	mov	r3, r3, lsr #11
	bl	printk
	ldr	r2, [r4, #3348]
	ldrh	r0, [r2, #92]
	ldrh	r1, [r2, #74]
	ldrh	r3, [r2, #88]
	str	r0, [sp]
	ldrh	r0, [r2, #96]
	str	r0, [sp, #4]
	ldrh	r0, [r2, #72]
	str	r0, [sp, #8]
	ldr	r0, [r2, #80]
	str	r0, [sp, #12]
	ldrh	r0, [r2, #90]
	str	r0, [sp, #16]
	ldrh	r0, [r2, #94]
	str	r0, [sp, #20]
	ldrh	r0, [r2, #98]
	str	r0, [sp, #24]
	ldr	r0, .L1581+16
	ldr	r2, [r2, #84]
	bl	printk
	add	r0, r4, #3360
	movw	r3, #3354
	ldrh	r0, [r0]
	ldrh	r1, [r4, r3]
	movw	r3, #3356
	ldrh	r2, [r4, r3]
	movw	r3, #3358
	ldrh	r3, [r4, r3]
	str	r0, [sp]
	movw	r0, #3352
	ldrh	r0, [r4, r0]
	str	r0, [sp, #4]
	ldr	r0, .L1581+20
	bl	printk
	ldr	r0, [r4, #580]
	movw	lr, #590
	ldr	ip, [r4, #3348]
	add	r3, r0, #584
	ldrh	r1, [r3]
	movw	r3, #586
	ldrh	r2, [r0, r3]
	add	r3, r0, #588
	ldrh	r0, [r0, lr]
	ldrh	r3, [r3]
	str	r0, [sp]
	ldr	r0, [ip, #156]
	str	r0, [sp, #4]
	ldrh	r0, [ip, #150]
	str	r0, [sp, #8]
	ldr	r0, .L1581+24
	bl	printk
	add	sp, sp, #32
	@ sp needed
	ldmfd	sp!, {r4, pc}
.L1582:
	.align	2
.L1581:
	.word	.LANCHOR0
	.word	.LC119
	.word	.LC120
	.word	.LC121
	.word	.LC122
	.word	.LC123
	.word	.LC124
	.fnend
	.size	print_ftl_debug_info, .-print_ftl_debug_info
	.align	2
	.global	ftl_write_buf
	.type	ftl_write_buf, %function
ftl_write_buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	subs	r4, r0, #0
	bne	.L1584
	ldr	r1, .L1595
	mov	r2, #676
	ldr	r0, .L1595+4
	bl	printk
	bl	dump_stack
	bl	print_ftl_debug_info
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1584:
	ldr	r5, .L1595+8
	ldrb	r2, [r4, #40]	@ zero_extendqisi2
	ldrb	r3, [r5, #3320]	@ zero_extendqisi2
	cmp	r2, r3
	bls	.L1589
	ldr	r1, .L1595
	movw	r2, #681
	ldr	r0, .L1595+4
	bl	printk
	bl	dump_stack
.L1589:
	ldrb	r3, [r4, #40]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1586
	ldrb	r2, [r5, #3320]	@ zero_extendqisi2
	ldr	r6, .L1595+8
	cmp	r2, r3
	bcs	.L1587
.L1586:
	mov	r0, r4
	bl	zbuf_free
	ldrb	r0, [r5, #3344]	@ zero_extendqisi2
	ldmfd	sp!, {r4, r5, r6, pc}
.L1587:
	mov	r1, r4
	ldr	r0, .L1595+12
	bl	buf_add_tail
	ldr	r3, [r6, #3348]
	ldrb	r1, [r4, #40]	@ zero_extendqisi2
	ldrb	r0, [r6, #3344]	@ zero_extendqisi2
	ldr	r2, [r3, #16]
	add	r0, r0, #1
	add	r2, r2, r1
	str	r2, [r3, #16]
	ldr	r2, [r3, #32]
	uxtb	r0, r0
	strb	r0, [r6, #3344]
	add	r2, r2, #1
	str	r2, [r3, #32]
	ldmfd	sp!, {r4, r5, r6, pc}
.L1596:
	.align	2
.L1595:
	.word	.LANCHOR1+1360
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR0+3368
	.fnend
	.size	ftl_write_buf, .-ftl_write_buf
	.align	2
	.global	ftl_write_completed
	.type	ftl_write_completed, %function
ftl_write_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r5, #0
	ldr	r7, .L1609
	ldr	r8, .L1609+4
	mov	r9, r7
.L1598:
	ldrb	r10, [r8, #-3574]	@ zero_extendqisi2
	cmp	r10, #255
	beq	.L1608
	mov	r4, #48
	ldr	r6, .L1609+8
	mul	r4, r4, r10
	add	r6, r6, r4
	add	r4, r7, r4
	ldrb	r3, [r4, #1784]	@ zero_extendqisi2
	strb	r3, [r8, #-3574]
	ldr	r3, [r4, #1820]
	cmn	r3, #1
	bne	.L1599
	ldr	r3, .L1609+12
	ldr	r2, [r4, #1808]
	ldrb	r1, [r7, #4030]	@ zero_extendqisi2
	ldrh	r0, [r3]
	ldrb	r3, [r7, #1700]	@ zero_extendqisi2
	mov	r2, r2, lsr r0
	rsb	r3, r3, #24
	rsb	r3, r0, r3
	mvn	r0, #0
	bic	r0, r2, r0, asl r3
	bl	__aeabi_uidiv
	ldr	r3, [r7, #580]
	movw	r2, #65535
	ldr	r1, [r3, #560]
	cmp	r1, r2
	uxth	r5, r0
	streq	r5, [r3, #560]
	ldreq	r2, [r4, #1808]
	ldr	r0, .L1609+16
	mov	r1, r5
	streq	r2, [r3, #564]
	mov	r3, #48
	mla	r3, r3, r10, r9
	ldr	r2, [r3, #1804]
	ldr	r3, [r3, #1808]
	bl	printk
	ldr	r3, [r9, #580]
	ldr	r2, [r3, #556]
	add	r2, r2, #1
	str	r2, [r3, #556]
	ldrh	r2, [r3, #16]
	cmp	r2, r5
	moveq	r2, #0
	streqh	r2, [r3, #22]	@ movhi
	beq	.L1602
	ldrh	r2, [r3, #48]
	cmp	r2, r5
	moveq	r2, #0
	streqh	r2, [r3, #54]	@ movhi
.L1602:
	mov	r0, r6
	mov	r5, #1
	bl	ftl_write_buf
	b	.L1598
.L1599:
	cmp	r5, #1
	bne	.L1604
	ldr	r1, [r4, #1804]
	ldr	r2, [r4, #1808]
	ldr	r0, .L1609+20
	bl	printk
	mov	r0, r6
	bl	ftl_write_buf
	b	.L1598
.L1604:
	ldr	r0, [r4, #1804]
	ldr	r1, [r4, #1812]
	ldrh	r2, [r6, #32]
	bl	lpa_hash_update_ppa
	ldrb	r3, [r4, #1786]	@ zero_extendqisi2
	and	r5, r3, #4
	ands	r5, r5, #255
	bicne	r3, r3, #2
	strneb	r3, [r4, #1786]
	movne	r5, #0
	bne	.L1598
	mov	r0, r6
	bl	zbuf_free
	b	.L1598
.L1608:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L1610:
	.align	2
.L1609:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+1784
	.word	.LANCHOR0+4028
	.word	.LC125
	.word	.LC126
	.fnend
	.size	ftl_write_completed, .-ftl_write_completed
	.align	2
	.global	zftl_add_read_buf
	.type	zftl_add_read_buf, %function
zftl_add_read_buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	subs	r4, r0, #0
	bne	.L1612
	ldr	r1, .L1617
	mov	r2, #972
	ldr	r0, .L1617+4
	bl	printk
	bl	dump_stack
	ldmfd	sp!, {r3, r4, r5, lr}
	b	print_ftl_debug_info
.L1612:
	ldr	r3, .L1617+8
	ldrb	r2, [r4, #40]	@ zero_extendqisi2
	ldrb	r3, [r3, #3320]	@ zero_extendqisi2
	cmp	r2, r3
	bls	.L1614
	ldr	r1, .L1617
	movw	r2, #977
	ldr	r0, .L1617+4
	bl	printk
	bl	dump_stack
.L1614:
	ldr	r5, .L1617+12
	mov	r1, r4
	ldr	r0, .L1617+16
	bl	buf_add_tail
	ldrb	r3, [r5, #-3572]	@ zero_extendqisi2
	add	r3, r3, #1
	strb	r3, [r5, #-3572]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1618:
	.align	2
.L1617:
	.word	.LANCHOR1+1376
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR3-3573
	.fnend
	.size	zftl_add_read_buf, .-zftl_add_read_buf
	.align	2
	.global	ftl_alloc_sblk
	.type	ftl_alloc_sblk, %function
ftl_alloc_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	uxth	r6, r0
	cmp	r0, #5
	mov	r5, r0
	mov	r1, r6
	mov	r0, #0
	movne	r7, #0
	moveq	r7, #2
	bl	zftl_get_free_sblk
	movw	r3, #65535
	cmp	r0, r3
	mov	r4, r0
	beq	.L1621
	ldr	r3, .L1630
	ldr	r6, [r3, #568]
	add	r6, r6, r0, asl #2
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	tst	r3, #224
	beq	.L1622
	ldr	r1, .L1630+4
	mov	r2, #836
	ldr	r0, .L1630+8
	bl	printk
	bl	dump_stack
.L1622:
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	bfi	r3, r5, #5, #3
	uxtb	r3, r3
	ubfx	r2, r3, #3, #2
	orr	r2, r7, r2
	bfi	r3, r2, #3, #2
	strb	r3, [r6, #2]
	b	.L1629
.L1621:
	bl	print_ftl_debug_info
	mov	r1, r4
	mov	r2, r5
	ldr	r0, .L1630+12
	bl	printk
	mov	r1, r6
	mov	r0, #0
	bl	zftl_get_free_sblk
	mov	r2, r5
	mov	r4, r0
	ldr	r0, .L1630+12
	mov	r1, r4
	bl	printk
	bl	dump_all_list_info
	ldr	r1, .L1630+4
	movw	r2, #846
	ldr	r0, .L1630+8
	bl	printk
	bl	dump_stack
.L1629:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1631:
	.align	2
.L1630:
	.word	.LANCHOR0
	.word	.LANCHOR1+1396
	.word	.LC0
	.word	.LC127
	.fnend
	.size	ftl_alloc_sblk, .-ftl_alloc_sblk
	.align	2
	.global	sblk_init
	.type	sblk_init, %function
sblk_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r2, .L1633
	mvn	r3, #0
	ldr	r1, .L1633+4
	mov	r0, #0
	strb	r3, [r2, #3323]
	strb	r3, [r1, #-3574]
	strb	r3, [r1, #-3571]
	strb	r3, [r2, #4031]
	bx	lr
.L1634:
	.align	2
.L1633:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	sblk_init, .-sblk_init
	.align	2
	.global	dump_sblk_queue
	.type	dump_sblk_queue, %function
dump_sblk_queue:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r4, .L1645
	ldr	r0, .L1645+4
	ldrb	r1, [r4, #3323]	@ zero_extendqisi2
	bl	printk
	ldrb	r4, [r4, #3323]	@ zero_extendqisi2
	cmp	r4, #255
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldr	r3, .L1645+8
	mov	r2, #48
	mov	r6, r2
	mla	r4, r2, r4, r3
	mov	r5, r3
.L1637:
	ldrb	r1, [r4, #1]	@ zero_extendqisi2
	ldrb	r2, [r4, #42]	@ zero_extendqisi2
	ldr	r3, [r4, #24]
	ldr	r0, .L1645+12
	bl	printk
	ldrb	r4, [r4]	@ zero_extendqisi2
	cmp	r4, #255
	ldmeqfd	sp!, {r4, r5, r6, pc}
	mla	r4, r6, r4, r5
	b	.L1637
.L1646:
	.align	2
.L1645:
	.word	.LANCHOR0
	.word	.LC128
	.word	.LANCHOR0+1784
	.word	.LC129
	.fnend
	.size	dump_sblk_queue, .-dump_sblk_queue
	.align	2
	.global	queue_lun_state
	.type	queue_lun_state, %function
queue_lun_state:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L1663
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	ldrb	lr, [ip, #3323]	@ zero_extendqisi2
	cmp	lr, #255
	beq	.L1655
	ldrb	r3, [ip, #1700]	@ zero_extendqisi2
	movw	r4, #4028
	ldrh	r8, [ip, r4]
	mov	r2, #1
	rsb	r7, r3, #24
	mov	r3, r2, asl r3
	rsb	r4, r8, r7
	sub	r3, r3, #1
	mov	r2, r2, asl r4
	ldrb	r4, [ip, #4030]	@ zero_extendqisi2
	sub	r2, r2, #1
	uxth	r3, r3
	sub	r4, r4, #1
	and	r10, r3, r0, asr r7
	uxth	r2, r2
	mov	r9, #48
	uxth	r4, r4
	and	r5, r4, r0, asr r8
	and	r5, r5, r2
.L1654:
	mla	r0, r9, lr, ip
	ldr	r6, [r0, #1808]
	and	fp, r3, r6, lsr r7
	cmp	fp, r10
	bne	.L1649
	and	r6, r4, r6, lsr r8
	ldrb	r0, [r0, #1826]	@ zero_extendqisi2
	and	r6, r6, r2
	cmp	r6, r5
	bne	.L1650
	cmp	r1, #1
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
	sub	r6, r0, #7
	tst	r6, #253
	beq	.L1649
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1650:
	cmp	r1, #3
	ldrls	pc, [pc, r1, asl #2]
	b	.L1649
.L1651:
	.word	.L1648
	.word	.L1652
	.word	.L1653
	.word	.L1648
.L1652:
	sub	r6, r0, #7
	tst	r6, #253
	beq	.L1649
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1653:
	cmp	r0, #11
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1649:
	mla	lr, r9, lr, ip
	ldrb	lr, [lr, #1784]	@ zero_extendqisi2
	cmp	lr, #255
	bne	.L1654
.L1655:
	mov	r0, #0
.L1648:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1664:
	.align	2
.L1663:
	.word	.LANCHOR0
	.fnend
	.size	queue_lun_state, .-queue_lun_state
	.align	2
	.global	queue_remove_completed_req
	.type	queue_remove_completed_req, %function
queue_remove_completed_req:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1692
	mov	ip, #0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mvn	r9, #0
	ldr	r1, .L1692+4
	mov	r8, r3
	ldrb	r0, [r3, #4031]	@ zero_extendqisi2
	mov	r4, r3
	ldrb	r2, [r3, #3323]	@ zero_extendqisi2
	ldrb	r6, [r1, #-3571]	@ zero_extendqisi2
	ldrb	r7, [r1, #-3574]	@ zero_extendqisi2
	ldr	r10, .L1692+8
	str	r0, [sp, #4]
	mov	r0, #48
	mov	r5, r0
.L1666:
	cmp	r2, #255
	beq	.L1667
	mla	fp, r0, r2, r3
	mov	r1, r2
	ldrb	lr, [fp, #1826]	@ zero_extendqisi2
	sub	lr, lr, #12
	cmp	lr, #1
	bls	.L1668
	cmp	ip, #0
	ldrne	r3, .L1692
	beq	.L1665
	b	.L1690
.L1668:
	mul	ip, r0, r1
	ldrb	r2, [fp, #1784]	@ zero_extendqisi2
	add	lr, r8, ip
	ldrb	fp, [lr, #1827]	@ zero_extendqisi2
	strb	r9, [lr, #1784]
	cmp	fp, #1
	bne	.L1671
	add	ip, r10, ip
	ldrh	ip, [ip, #34]
	cmp	ip, #0
	moveq	ip, r7
	ldreq	lr, .L1692+12
	ldrne	ip, [sp, #4]
	ldrne	lr, .L1692+16
	b	.L1672
.L1671:
	cmp	fp, #0
	beq	.L1691
.L1673:
	mov	ip, #1
	b	.L1666
.L1691:
	ldr	ip, [lr, #1804]
	cmn	ip, #1
	movne	ip, r6
	ldrne	lr, .L1692+20
	beq	.L1673
.L1672:
	cmp	ip, #255
	bne	.L1677
	strb	r2, [r3, #3323]
	mov	r2, #48
	mla	r1, r2, r1, r3
	ldrb	r3, [r1, #1785]	@ zero_extendqisi2
	strb	r3, [lr]
	b	.L1665
.L1677:
	mov	lr, ip
	mla	ip, r0, ip, r3
	ldrb	ip, [ip, #1784]	@ zero_extendqisi2
	cmp	ip, #255
	bne	.L1677
	mla	r1, r5, r1, r4
	mla	lr, r5, lr, r4
	ldrb	r1, [r1, #1785]	@ zero_extendqisi2
	strb	r1, [lr, #1784]
	b	.L1673
.L1667:
	cmp	ip, #0
	beq	.L1665
	mvn	r2, #0
.L1690:
	strb	r2, [r3, #3323]
.L1665:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1693:
	.align	2
.L1692:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+1784
	.word	.LANCHOR3-3574
	.word	.LANCHOR0+4031
	.word	.LANCHOR3-3571
	.fnend
	.size	queue_remove_completed_req, .-queue_remove_completed_req
	.align	2
	.global	pm_free_sblk
	.type	pm_free_sblk, %function
pm_free_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 272
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1724
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #284
	sub	sp, sp, #284
	ldrh	r3, [r2, #-2]
	mov	r5, r2
	cmp	r3, #128
	bls	.L1695
	ldr	r1, .L1724+4
	mov	r2, #84
	ldr	r0, .L1724+8
	bl	printk
	bl	dump_stack
.L1695:
	ldr	r4, .L1724+12
	movw	lr, #65535
	ldrh	ip, [r5, #-2]
	ldr	r3, [r4, #3348]
	add	r2, r3, #688
	add	r3, r3, #412
	add	r3, r3, #2
	ldrh	r0, [r2]
	mov	r2, #0
	mov	r1, r2
.L1698:
	ldrh	r6, [r3, #2]!
	uxth	r5, r2
	cmp	r6, lr
	addne	r1, r1, #1
	uxthne	r1, r1
	cmp	r1, r0
	bcs	.L1697
	cmp	r1, ip
	bcs	.L1697
	add	r2, r2, #1
	cmp	r2, #128
	bne	.L1698
	mov	r5, r2
.L1697:
	add	r5, r5, #1
	add	r0, sp, #24
	mov	r1, #256
	mvn	r10, #0
	uxth	r5, r5
	mov	r6, #0
	cmp	r5, #128
	movcc	r3, r5
	movcs	r3, #128
	str	r3, [sp, #12]
	bl	__memzero
	ldr	r7, [r4, #3348]
	movw	r3, #698
	ldrb	r5, [r4, #4030]	@ zero_extendqisi2
	add	fp, r7, #700
	ldrh	r9, [r7, r3]
	movw	r3, #4028
	ldrh	r8, [r4, r3]
	ldrb	r3, [r4, #1700]	@ zero_extendqisi2
	rsb	r3, r3, #24
	rsb	r3, r8, r3
	mvn	r10, r10, asl r3
.L1699:
	uxth	r2, r6
	cmp	r2, r9
	bcs	.L1721
	ldr	r0, [fp, #4]!
	mov	r1, r5
	and	r0, r10, r0, lsr r8
	bl	__aeabi_uidiv
	add	r2, r7, #412
	add	r2, r2, #2
	mov	ip, #0
	uxth	r0, r0
	add	r1, sp, #24
.L1700:
	ldr	r3, [sp, #12]
	uxth	lr, ip
	cmp	lr, r3
	bcs	.L1722
	ldrh	lr, [r2, #2]!
	add	ip, ip, #1
	add	r1, r1, #2
	cmp	lr, r0
	ldreqh	lr, [r1, #-2]
	addeq	lr, lr, #1
	streqh	lr, [r1, #-2]	@ movhi
	b	.L1700
.L1722:
	add	r6, r6, #1
	b	.L1699
.L1721:
	movw	r3, #3976
	ldrb	r8, [r4, #3984]	@ zero_extendqisi2
	ldrh	r3, [r4, r3]
	mov	r7, #0
	add	r6, sp, #24
	movw	fp, #65535
	mov	r10, r7
	smulbb	r8, r8, r3
	uxth	r8, r8
.L1704:
	ldr	r3, [sp, #12]
	uxth	r5, r7
	cmp	r5, r3
	bcs	.L1723
	add	r9, r7, #208
	ldr	r2, [r4, #3348]
	mov	r3, r9, asl #1
	add	r1, r2, #692
	str	r2, [sp, #16]
	ldrh	r3, [r2, r3]
	ldrh	r0, [r1]
	ldrb	r1, [r4, #4030]	@ zero_extendqisi2
	str	r3, [sp, #20]
	bl	__aeabi_idiv
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #16]
	cmp	r0, r3
	ldreq	r1, .L1724+12
	ldreqb	r0, [r1, #3984]	@ zero_extendqisi2
	ldreq	r1, .L1724+16
	ldreqh	r1, [r1]
	smulbbeq	r1, r0, r1
	movw	r0, #65535
	streqh	r1, [r6]	@ movhi
	ldrh	lr, [r6]
	cmp	lr, #0
	cmpne	r8, lr
	movhi	r1, #1
	movls	r1, #0
	cmp	r1, #0
	ldrh	r1, [r2, #74]
	movne	r10, r5
	movne	r8, lr
	cmp	r3, r0
	cmpne	r1, #2
	bls	.L1707
	ldr	r0, [r4, #568]
	mov	r1, r3, asl #2
	ldrh	r2, [r2, #92]
	ldrh	r1, [r0, r1]
	add	r2, r2, #4
	ubfx	r1, r1, #0, #11
	cmp	r1, r2
	bgt	.L1707
	str	r1, [sp]
	mov	r2, lr
	ldr	r0, .L1724+20
	mov	r1, r7
	bl	printk
	mov	fp, r5
.L1707:
	ldrh	r2, [r6]
	cmp	r2, #0
	bne	.L1708
	ldr	r1, [r4, #3348]
	mov	r3, r9, asl #1
	movw	r0, #65535
	ldrh	r3, [r1, r3]
	cmp	r3, r0
	beq	.L1708
	ldr	r0, .L1724+24
	ldr	r0, [r0]
	tst	r0, #4096
	beq	.L1709
	add	r1, r1, #688
	ldr	r0, .L1724+28
	ldrh	r1, [r1]
	str	r1, [sp]
	mov	r1, r7
	bl	printk
.L1709:
	ldr	r3, [r4, #3348]
	mov	r9, r9, asl #1
	ldrh	r0, [r3, r9]
	bl	ftl_free_sblk
	ldr	r3, [r4, #3348]
	mvn	r2, #0
	strh	r2, [r3, r9]	@ movhi
	add	r3, r3, #688
	ldrh	r2, [r3]
	sub	r2, r2, #1
	strh	r2, [r3]	@ movhi
.L1708:
	add	r7, r7, #1
	add	r6, r6, #2
	b	.L1704
.L1723:
	movw	r0, #65535
	cmp	fp, r0
	movne	r0, fp
	moveq	r0, r10
	add	sp, sp, #284
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1725:
	.align	2
.L1724:
	.word	.LANCHOR3-3568
	.word	.LANCHOR1+1412
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR0+3976
	.word	.LC130
	.word	.LANCHOR2
	.word	.LC131
	.fnend
	.size	pm_free_sblk, .-pm_free_sblk
	.align	2
	.global	pm_alloc_new_blk
	.type	pm_alloc_new_blk, %function
pm_alloc_new_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #8
	movw	r1, #690
	ldr	r4, .L1743
	ldr	r2, [r4, #3348]
	ldrh	r3, [r2, r1]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r2, r1]	@ movhi
	ldrb	r1, [r4, #3984]	@ zero_extendqisi2
	cmp	r1, r3
	bls	.L1727
	add	r3, r3, #336
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1728
.L1727:
	ldr	r5, .L1743
.L1740:
	mov	r0, #1
	bl	ftl_alloc_sblk
	mov	r1, #0
	mov	r6, r0
	bl	ftl_erase_sblk
	ldr	r1, [r4, #3348]
	mov	r0, r6
	add	r1, r1, #672
	bl	ftl_get_blk_list_in_sblk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L1729
	mov	r1, r6
	ldr	r0, .L1743+4
	bl	printk
	ldr	r3, [r5, #568]
	add	r6, r3, r6, asl #2
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	orr	r3, r3, #224
	strb	r3, [r6, #2]
	b	.L1740
.L1729:
	ldr	r3, .L1743
	movw	r2, #690
	mov	r5, #0
	mov	r1, #1
	ldr	r3, [r3, #3348]
	strh	r5, [r3, r2]	@ movhi
	add	r3, r3, #412
	ldr	r2, .L1743+8
	add	r3, r3, #2
	str	r1, [r2, #-3568]
	movw	r1, #65535
.L1731:
	ldrh	r0, [r3, #2]!
	uxth	r2, r5
	cmp	r0, r1
	beq	.L1730
	add	r5, r5, #1
	cmp	r5, #128
	bne	.L1731
	mov	r2, #227
	ldr	r1, .L1743+12
	ldr	r0, .L1743+16
	bl	printk
	bl	dump_stack
	mov	r2, r5
.L1730:
	add	r2, r2, #208
	ldr	r3, [r4, #3348]
	mov	r2, r2, asl #1
	strh	r6, [r3, r2]	@ movhi
	add	r3, r3, #688
	ldrh	r2, [r3]
	add	r2, r2, #1
	strh	r2, [r3]	@ movhi
.L1728:
	ldr	r2, [r4, #3348]
	movw	r3, #690
	ldrh	r3, [r2, r3]
	add	r3, r3, #336
	mov	r3, r3, asl #1
	ldrh	r5, [r2, r3]
	movw	r2, #65533
	sub	r3, r5, #1
	uxth	r3, r3
	cmp	r3, r2
	bls	.L1733
	ldr	r1, .L1743+12
	mov	r2, #232
	ldr	r0, .L1743+16
	bl	printk
	bl	dump_stack
.L1733:
	ldr	r1, [r4, #3348]
	mov	r2, #0
	add	r3, r1, #696
	strh	r2, [r3]	@ movhi
	movw	r2, #4028
	ldrb	r3, [r4, #1700]	@ zero_extendqisi2
	ldrh	r4, [r4, r2]
	movw	r2, #694
	rsb	r3, r3, #24
	rsb	r4, r4, r3
	mov	r3, r5, asr r4
	strh	r3, [r1, r2]	@ movhi
	ldr	r2, .L1743+20
	add	r1, r1, #692
	strh	r5, [r1]	@ movhi
	ldr	r2, [r2]
	tst	r2, #4096
	beq	.L1737
	mvn	r2, #0
	uxth	r3, r3
	ldr	r0, .L1743+24
	mov	r1, r5
	str	r3, [sp]
	mvn	r2, r2, asl r4
	mov	r3, r5
	bl	printk
.L1737:
	mov	r0, #0
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L1744:
	.align	2
.L1743:
	.word	.LANCHOR0
	.word	.LC132
	.word	.LANCHOR3
	.word	.LANCHOR1+1428
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC133
	.fnend
	.size	pm_alloc_new_blk, .-pm_alloc_new_blk
	.align	2
	.global	pm_select_ram_region
	.type	pm_select_ram_region, %function
pm_select_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1760
	mov	r3, #0
	movw	ip, #65535
.L1747:
	mov	r1, r3, asl #3
	uxth	r0, r3
	ldrh	r1, [r1, r2]
	cmp	r1, ip
	ldr	r1, .L1760
	bxeq	lr
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L1747
	mov	r2, #0
	mov	r0, r3
	mov	ip, #32768
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
.L1749:
	add	r3, r1, r2, asl #3
	uxth	lr, r2
	ldrh	r3, [r3, #2]
	tst	r3, #32768
	bne	.L1748
	cmp	r3, ip
	movcc	ip, r3
	movcc	r0, lr
.L1748:
	add	r2, r2, #1
	cmp	r2, #32
	bne	.L1749
	cmp	r0, #32
	mov	r4, r0
	ldmnefd	sp!, {r4, r5, r6, pc}
	ldr	r3, .L1760+4
	mvn	r2, #0
	ldr	r5, .L1760
	ldrb	lr, [r3, #-3308]	@ zero_extendqisi2
	mov	r3, #0
.L1751:
	mov	ip, r3, asl #3
	uxth	r6, r3
	add	r0, r1, ip
	ldrh	r0, [r0, #2]
	cmp	r0, r2
	bcs	.L1750
	ldrh	ip, [ip, r5]
	cmp	ip, lr
	movne	r2, r0
	movne	r4, r6
.L1750:
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L1751
	cmp	r4, #32
	bne	.L1752
	ldr	r1, .L1760+8
	movw	r2, #331
	ldr	r0, .L1760+12
	bl	printk
	bl	dump_stack
.L1752:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1761:
	.align	2
.L1760:
	.word	.LANCHOR3-3564
	.word	.LANCHOR3
	.word	.LANCHOR1+1448
	.word	.LC0
	.fnend
	.size	pm_select_ram_region, .-pm_select_ram_region
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
	beq	.L1763
	bl	memset
.L1763:
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
	.fnend
	.size	ftl_memset, .-ftl_memset
	.align	2
	.global	lpa_hash_init
	.type	lpa_hash_init, %function
lpa_hash_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r1, #255
	ldr	r4, .L1770
	mov	r2, #512
	add	r0, r4, #4080
	bl	memset
	movw	r3, #3976
	ldrh	r2, [r4, r3]
	mov	r1, #255
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	mul	r2, r3, r2
	ldr	r3, .L1770+4
	ldr	r0, [r3, #-3588]
	mov	r2, r2, asl #2
	ldmfd	sp!, {r4, lr}
	b	ftl_memset
.L1771:
	.align	2
.L1770:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	lpa_hash_init, .-lpa_hash_init
	.align	2
	.global	lpa_rebuild_hash
	.type	lpa_rebuild_hash, %function
lpa_rebuild_hash:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1785
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L1773
	ldr	r0, .L1785+4
	mov	r2, #225
	ldr	r1, .L1785+8
	mov	r3, #0
	bl	printk
.L1773:
	ldr	r4, .L1785+12
	mov	r1, #255
	mov	r2, #512
	ldr	r5, .L1785+16
	add	r0, r4, #4080
	bl	memset
	movw	r3, #3976
	ldrh	r2, [r4, r3]
	mov	r1, #255
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	mov	r6, r5
	ldr	r0, [r5, #-3588]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	ip, .L1785+20
	mov	r0, #0
	add	lr, ip, #104
.L1774:
	ldrh	r1, [ip]
	uxth	r2, r0
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	mul	r3, r3, r1
	cmp	r2, r3, asl #1
	bge	.L1784
	ldr	r3, [r5, #-3592]
	ldr	r3, [r3, r2, asl #2]
	cmn	r3, #1
	beq	.L1775
	uxtb	r3, r3
	mov	r3, r3, asl #1
	ldrh	r1, [lr, r3]
	strh	r2, [lr, r3]	@ movhi
	mov	r2, r2, asl #1
	ldr	r3, [r6, #-3588]
	strh	r1, [r3, r2]	@ movhi
.L1775:
	add	r0, r0, #1
	b	.L1774
.L1784:
	ldmfd	sp!, {r4, r5, r6, pc}
.L1786:
	.align	2
.L1785:
	.word	.LANCHOR2
	.word	.LC134
	.word	.LANCHOR1+1472
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+3976
	.fnend
	.size	lpa_rebuild_hash, .-lpa_rebuild_hash
	.align	2
	.global	zftl_sblk_list_init
	.type	zftl_sblk_list_init, %function
zftl_sblk_list_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r2, #6
	ldr	r4, .L1818
	mov	r1, #0
	.pad #20
	sub	sp, sp, #20
	mov	r5, #0
	add	r8, r4, #564
	ldr	fp, .L1818+4
	ldr	r0, [r4, #524]
	mov	r7, r4
	ldrh	r3, [r8]
	mul	r2, r2, r3
	bl	ftl_memset
	add	r3, r4, #4064
	mov	r2, #16
	mov	r0, #32768
	strh	r2, [r3]	@ movhi
	movw	r3, #3332
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3334
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3336
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3338
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3342
	strh	r5, [r4, r3]	@ movhi
	movw	r2, #3986
	movw	r3, #3340
	ldrh	r1, [r4, r2]
	strh	r5, [r4, r3]	@ movhi
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	str	r5, [r4, #4052]
	str	r5, [r4, #4056]
	mul	r1, r1, r3
	str	r5, [r4, #4060]
	str	r5, [r4, #4036]
	str	r5, [r4, #4044]
	str	r5, [r4, #4040]
	bl	__aeabi_idiv
	uxth	r3, r0
	str	r3, [sp, #4]
	ldr	r3, [r4, #3348]
	str	r8, [sp, #8]
	strh	r5, [r3, #146]	@ movhi
	ldrh	r5, [r3, #134]
.L1788:
	ldr	r1, [sp, #8]
	sxth	r2, r5
	ldr	r3, .L1818
	ldrh	r1, [r1]
	cmp	r2, r1
	bge	.L1816
	ldr	r3, [r7, #568]
	add	r8, r3, r2, asl #2
	ldrb	r1, [r8, #3]	@ zero_extendqisi2
	cmp	r1, #0
	ldreq	r1, [sp, #4]
	beq	.L1789
	ldrb	r6, [r7, #3984]	@ zero_extendqisi2
	mov	ip, #0
	ldrh	r10, [fp]
	mov	r1, ip
	ldr	lr, [r7, #3348]
.L1790:
	cmp	ip, r6
	bge	.L1817
	ldrb	r0, [r8, #3]	@ zero_extendqisi2
	mov	r0, r0, asr ip
	add	ip, ip, #1
	tst	r0, #1
	addeq	r1, r10, r1
	ldrneh	r0, [lr, #146]
	uxtheq	r1, r1
	addne	r0, r0, #1
	strneh	r0, [lr, #146]	@ movhi
	b	.L1790
.L1817:
	cmp	r1, #0
	beq	.L1794
	sxth	r1, r1
	mov	r0, #32768
	str	r2, [sp, #12]
	bl	__aeabi_idiv
	add	r0, r0, #1
	ldr	r2, [sp, #12]
	uxth	r1, r0
	b	.L1789
.L1794:
	ldrb	r0, [r8, #2]	@ zero_extendqisi2
	mvn	lr, #0
	orr	r0, r0, #224
	strb	r0, [r8, #2]
	ldr	ip, [r4, #576]
	mov	r0, r2, asl #1
	strh	lr, [ip, r0]	@ movhi
.L1789:
	mov	r9, r2, asl #1
	ldr	lr, [r4, #524]
	add	r0, r9, r2
	mov	r0, r0, asl #1
	add	r6, lr, r0
	strh	r1, [r6, #4]	@ movhi
	mvn	r1, #0
	strh	r1, [r6, #2]	@ movhi
	strh	r1, [lr, r0]	@ movhi
	ldrb	r1, [r8, #2]	@ zero_extendqisi2
	and	r6, r1, #224
	cmp	r6, #32
	cmpne	r6, #224
	moveq	r10, #1
	movne	r10, #0
	beq	.L1796
	ldr	r0, [r7, #580]
	ldrh	lr, [r0, #16]
	cmp	r2, lr
	beq	.L1796
	ldrh	lr, [r0, #48]
	cmp	r2, lr
	beq	.L1796
	ldrh	r0, [r0, #80]
	cmp	r2, r0
	beq	.L1796
	cmp	r6, #64
	uxtheq	r6, r5
	ldreq	r0, .L1818+8
	ldreq	r2, .L1818+12
	moveq	r1, r6
	beq	.L1814
.L1798:
	cmp	r6, #96
	uxtheq	r6, r5
	ldreq	r0, .L1818+16
	moveq	r1, r6
	subeq	r2, r0, #696
	beq	.L1814
.L1799:
	cmp	r6, #160
	bne	.L1800
	uxth	r6, r5
	ldr	r0, .L1818+20
	ldr	r2, .L1818+24
	mov	r1, r6
.L1814:
	bl	_insert_data_list
	ldr	r3, [r7, #576]
	ldrh	r3, [r3, r9]
	cmp	r3, #7
	movls	r0, r6
	movls	r1, #1
	movls	r2, r10
	bhi	.L1796
	b	.L1812
.L1800:
	cmp	r6, #0
	bne	.L1796
	ldr	r0, [r7, #576]
	uxth	r10, r5
	ldrh	ip, [r0, r9]
	cmp	ip, #0
	beq	.L1801
	mov	r1, r2
	ldr	r0, .L1818+28
	mov	r2, ip
	bl	printk
	ldrb	r2, [r8, #2]	@ zero_extendqisi2
	mov	r0, r10
	tst	r2, #16
	movne	r1, #5
	moveq	r1, #2
	bfi	r2, r1, #5, #3
	mov	r1, #1
	strb	r2, [r8, #2]
	mov	r2, r6
.L1812:
	bl	gc_add_sblk
	b	.L1796
.L1801:
	ands	r1, r1, #24
	ldreq	r0, .L1818+32
	moveq	r1, r10
	subeq	r2, r0, #720
	beq	.L1810
.L1803:
	cmp	r1, #16
	ldrne	r0, .L1818+36
	moveq	r1, r10
	ldreq	r0, .L1818+40
	movne	r1, r10
	ldreq	r2, .L1818+44
	subne	r2, r0, #724
.L1810:
	bl	_insert_free_list
.L1796:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L1788
.L1816:
	movw	r1, #3332
	ldr	r2, [r3, #3348]
	ldrh	r1, [r3, r1]
	strh	r1, [r2, #114]	@ movhi
	movw	r1, #3334
	ldrh	r1, [r3, r1]
	strh	r1, [r2, #118]	@ movhi
	movw	r1, #3336
	ldrh	r1, [r3, r1]
	strh	r1, [r2, #116]	@ movhi
	movw	r1, #3338
	ldrh	r1, [r3, r1]
	strh	r1, [r2, #122]	@ movhi
	movw	r1, #3342
	ldrh	r1, [r3, r1]
	strh	r1, [r2, #120]	@ movhi
	movw	r1, #3340
	ldrh	r3, [r3, r1]
	strh	r3, [r2, #124]	@ movhi
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1819:
	.align	2
.L1818:
	.word	.LANCHOR0
	.word	.LANCHOR0+3986
	.word	.LANCHOR0+4044
	.word	.LANCHOR0+3338
	.word	.LANCHOR0+4036
	.word	.LANCHOR0+4040
	.word	.LANCHOR0+3342
	.word	.LC135
	.word	.LANCHOR0+4052
	.word	.LANCHOR0+4060
	.word	.LANCHOR0+4056
	.word	.LANCHOR0+3334
	.fnend
	.size	zftl_sblk_list_init, .-zftl_sblk_list_init
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
	.global	flash_info_data_init
	.type	flash_info_data_init, %function
flash_info_data_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1827
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L1822
	ldr	r0, .L1827+4
	mov	r1, #105
	ldr	r2, .L1827+8
	bl	printk
.L1822:
	ldr	r4, .L1827+12
	mov	r1, #2048
	ldr	r0, [r4, #1696]
	bl	__memzero
	ldr	r3, [r4, #1696]
	ldr	r2, .L1827+16
	ldr	r1, .L1827+20
	str	r2, [r3]
	mov	r3, #2032
	ldr	r0, [r4, #1696]
	mov	r2, #32
	add	r0, r0, #80
	str	r3, [r0, #-72]
	mov	r3, #1
	strh	r3, [r0, #-64]	@ movhi
	bl	ftl_memcpy
	ldr	r0, [r4, #1696]
	ldr	r1, .L1827+24
	mov	r2, #32
	add	r0, r0, #48
	ldmfd	sp!, {r4, lr}
	b	ftl_memcpy
.L1828:
	.align	2
.L1827:
	.word	.LANCHOR2
	.word	.LC136
	.word	.LANCHOR1+1492
	.word	.LANCHOR0
	.word	1398362953
	.word	.LANCHOR0+1626
	.word	.LANCHOR2+4
	.fnend
	.size	flash_info_data_init, .-flash_info_data_init
	.align	2
	.global	ftl_memcpy32
	.type	ftl_memcpy32, %function
ftl_memcpy32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	r3, #0
.L1830:
	cmp	r3, r2
	ldrne	ip, [r1, r3, asl #2]
	strne	ip, [r0, r3, asl #2]
	addne	r3, r3, #1
	bne	.L1830
.L1832:
	bx	lr
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
	ldr	r3, .L1835
	ldr	r0, [r3]
	ldr	r3, .L1835+4
	ldr	r3, [r3, #-3304]
	rsb	r0, r3, r0
	b	jiffies_to_msecs
.L1836:
	.align	2
.L1835:
	.word	jiffies
	.word	.LANCHOR3
	.fnend
	.size	timer_get_time, .-timer_get_time
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
	ldr	r3, .L1839
	mov	r2, r4
	mov	r1, #1
	mov	r0, r5
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #12]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1840:
	.align	2
.L1839:
	.word	.LANCHOR3
	.fnend
	.size	StorageSysDataLoad, .-StorageSysDataLoad
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
	ldr	r3, .L1843
	mov	r2, r4
	mov	r1, #1
	mov	r0, r5
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #16]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1844:
	.align	2
.L1843:
	.word	.LANCHOR3
	.fnend
	.size	StorageSysDataStore, .-StorageSysDataStore
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
	ldr	r3, .L1847
	mov	r2, r4
	mov	r1, r5
	mov	r0, r6
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #4]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1848:
	.align	2
.L1847:
	.word	.LANCHOR3
	.fnend
	.size	FlashBootVendorRead, .-FlashBootVendorRead
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
	ldr	r3, .L1851
	mov	r2, r4
	mov	r1, r5
	mov	r0, r6
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #8]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1852:
	.align	2
.L1851:
	.word	.LANCHOR3
	.fnend
	.size	FlashBootVendorWrite, .-FlashBootVendorWrite
	.align	2
	.global	flash_sram_load_store
	.type	flash_sram_load_store, %function
flash_sram_load_store:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L1857
	cmp	r2, #0
	moveq	r2, r3
	ldr	ip, [ip, #-3292]
	add	ip, ip, #4096
	add	r1, ip, r1
	beq	.L1856
	str	lr, [sp, #-4]!
	.save {lr}
	mov	lr, r0
	mov	r0, r1
	mov	r1, lr
	ldr	lr, [sp], #4
	mov	r2, r3
.L1856:
	b	ftl_memcpy
.L1858:
	.align	2
.L1857:
	.word	.LANCHOR3
	.fnend
	.size	flash_sram_load_store, .-flash_sram_load_store
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
	.global	rk_ftl_de_init
	.type	rk_ftl_de_init, %function
rk_ftl_de_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r1, #0
	ldr	r0, .L1862
	bl	printk
	ldr	r3, .L1862+4
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #40]
	ldmfd	sp!, {r4, lr}
	bx	r3	@ indirect register sibling call
.L1863:
	.align	2
.L1862:
	.word	.LC137
	.word	.LANCHOR3
	.fnend
	.size	rk_ftl_de_init, .-rk_ftl_de_init
	.align	2
	.global	rk_ftl_cache_write_back
	.type	rk_ftl_cache_write_back, %function
rk_ftl_cache_write_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1865
	mov	r0, #0
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #32]
	bx	r3	@ indirect register sibling call
.L1866:
	.align	2
.L1865:
	.word	.LANCHOR3
	.fnend
	.size	rk_ftl_cache_write_back, .-rk_ftl_cache_write_back
	.align	2
	.global	rk_nand_suspend
	.type	rk_nand_suspend, %function
rk_nand_suspend:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1868
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #44]
	bx	r3	@ indirect register sibling call
.L1869:
	.align	2
.L1868:
	.word	.LANCHOR3
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
	ldr	r3, .L1871
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #48]
	bx	r3	@ indirect register sibling call
.L1872:
	.align	2
.L1871:
	.word	.LANCHOR3
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
	ldr	r3, .L1874
	mov	r0, #0
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #36]
	bx	r3
.L1875:
	.align	2
.L1874:
	.word	.LANCHOR3
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
	ldr	r3, .L1877
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #60]
	bx	r3
.L1878:
	.align	2
.L1877:
	.word	.LANCHOR3
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
	ldr	r3, .L1880
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #64]
	bx	r3
.L1881:
	.align	2
.L1880:
	.word	.LANCHOR3
	.fnend
	.size	rknand_proc_ftlread, .-rknand_proc_ftlread
	.align	2
	.global	FtlRead
	.type	FtlRead, %function
FtlRead:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	ip, .L1883
	ldr	ip, [ip, #-3296]
	ldr	ip, [ip, #20]
	bx	ip
.L1884:
	.align	2
.L1883:
	.word	.LANCHOR3
	.fnend
	.size	FtlRead, .-FtlRead
	.align	2
	.global	FtlDiscard
	.type	FtlDiscard, %function
FtlDiscard:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1886
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #28]
	bx	r3
.L1887:
	.align	2
.L1886:
	.word	.LANCHOR3
	.fnend
	.size	FtlDiscard, .-FtlDiscard
	.align	2
	.global	rk_ftl_garbage_collect
	.type	rk_ftl_garbage_collect, %function
rk_ftl_garbage_collect:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1889
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #52]
	bx	r3
.L1890:
	.align	2
.L1889:
	.word	.LANCHOR3
	.fnend
	.size	rk_ftl_garbage_collect, .-rk_ftl_garbage_collect
	.align	2
	.global	ReadFlashInfo
	.type	ReadFlashInfo, %function
ReadFlashInfo:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L1892
	ldr	r3, [r3, #-3296]
	ldr	r3, [r3, #56]
	bx	r3	@ indirect register sibling call
.L1893:
	.align	2
.L1892:
	.word	.LANCHOR3
	.fnend
	.size	ReadFlashInfo, .-ReadFlashInfo
	.align	2
	.global	rknand_print_hex
	.type	rknand_print_hex, %function
rknand_print_hex:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r5, #0
	mov	r9, r0
	mov	r6, r1
	mov	r7, r2
	mov	r8, r3
	mov	r4, r5
.L1895:
	cmp	r4, r8
	beq	.L1904
	cmp	r5, #0
	bne	.L1896
	ldr	r0, .L1905
	mov	r1, r9
	mov	r2, r6
	mov	r3, r4
	bl	printk
.L1896:
	cmp	r7, #4
	ldreq	r0, .L1905+4
	ldreq	r1, [r6, r4, asl #2]
	beq	.L1903
	cmp	r7, #2
	moveq	r3, r4, asl #1
	ldreq	r0, .L1905+4
	ldreqsh	r1, [r6, r3]
	ldrne	r0, .L1905+4
	ldrneb	r1, [r6, r4]	@ zero_extendqisi2
.L1903:
	add	r5, r5, #1
	bl	printk
	cmp	r5, #15
	bls	.L1900
	ldr	r0, .L1905+8
	mov	r5, #0
	ldr	r1, .L1905+12
	bl	printk
.L1900:
	add	r4, r4, #1
	b	.L1895
.L1904:
	ldr	r0, .L1905+8
	ldr	r1, .L1905+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L1906:
	.align	2
.L1905:
	.word	.LC138
	.word	.LC139
	.word	.LC1
	.word	.LC140
	.fnend
	.size	rknand_print_hex, .-rknand_print_hex
	.align	2
	.global	hynix_get_read_retry_default
	.type	hynix_get_read_retry_default, %function
hynix_get_read_retry_default:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	cmp	r0, #2
	ldr	r8, .L2018
	.pad #36
	sub	sp, sp, #36
	mvn	r7, #81
	mvn	r6, #80
	mov	r5, r0
	ldr	r4, [r8, #1696]
	add	r3, r4, #128
	str	r3, [sp]
	strb	r0, [r4, #112]
	mvn	r3, #83
	strb	r7, [r4, #130]
	strb	r3, [r4, #128]
	mvn	r3, #82
	strb	r6, [r4, #131]
	strb	r3, [r4, #129]
	bne	.L1908
	mvn	r3, #88
	strb	r3, [r4, #128]
	ldr	r3, .L2018+4
	mvn	r1, #8
	mov	r6, #7
	strb	r1, [r3, #413]
	b	.L1969
.L1908:
	cmp	r0, #3
	bne	.L1910
	add	r1, r4, #127
	mov	r3, #176
.L1911:
	strb	r3, [r1, #1]!
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #184
	bne	.L1911
	b	.L2012
.L1910:
	cmp	r0, #4
	bne	.L1912
	mvn	r1, #51
	strb	r3, [r4, #133]
	strb	r1, [r4, #128]
	mvn	r1, #64
	strb	r7, [r4, #134]
	strb	r1, [r4, #129]
	mvn	r1, #85
	strb	r6, [r4, #135]
	strb	r1, [r4, #130]
	mvn	r1, #84
	strb	r1, [r4, #131]
	mvn	r1, #50
	strb	r1, [r4, #132]
.L2012:
	mov	r6, #8
	mov	fp, r6
	b	.L1909
.L1912:
	cmp	r0, #5
	bne	.L1913
	mov	r3, #56
	mov	r6, #8
	strb	r3, [r4, #128]
	mov	r3, #57
	strb	r3, [r4, #129]
	mov	r3, #58
	strb	r3, [r4, #130]
	mov	r3, #59
	strb	r3, [r4, #131]
	b	.L1969
.L1913:
	cmp	r0, #6
	bne	.L1914
	mov	r3, #14
	mov	r6, #12
	strb	r3, [r4, #128]
	mov	r3, #15
	strb	r3, [r4, #129]
	mov	r3, #16
	strb	r3, [r4, #130]
	mov	r3, #17
	strb	r3, [r4, #131]
	b	.L1969
.L1914:
	cmp	r0, #7
	bne	.L1915
	add	r1, r4, #127
	mov	r3, #176
.L1916:
	strb	r3, [r1, #1]!
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #184
	bne	.L1916
	mvn	r3, #43
	mov	r6, #12
	strb	r3, [r4, #136]
	mov	fp, #10
	mvn	r3, #42
	strb	r3, [r4, #137]
	b	.L1909
.L1915:
	cmp	r0, #8
	mov	r6, #7
	bne	.L1969
	mov	r3, #6
	strb	r6, [r4, #129]
	strb	r3, [r4, #128]
	mov	r6, #50
	mov	r3, #9
	strb	r0, [r4, #130]
	strb	r3, [r4, #131]
	mov	fp, #5
	mov	r3, #10
	strb	r3, [r4, #132]
	b	.L1909
.L1969:
	mov	fp, #4
.L1909:
	sub	r3, r5, #1
	cmp	r3, #1
	bhi	.L2008
	ldr	r1, .L2018+8
	mov	r9, #0
.L1917:
	ldrb	ip, [r8, #585]	@ zero_extendqisi2
	uxtb	r3, r9
	ldr	r0, .L2018
	cmp	ip, r3
	bls	.L1924
	add	r3, r0, r3
	mov	r5, #160
	ldr	r0, [r0, #528]
	sub	r10, fp, #1
	ldrb	r7, [r3, #588]	@ zero_extendqisi2
	add	r2, r4, #127
	ldr	ip, [sp]
	mla	r5, r5, r7, r4
	add	r7, r0, r7, asl #8
	uxtab	r10, ip, r10
	mov	ip, #55
	add	r5, r5, #144
	sub	r3, r5, #1
.L1919:
	str	ip, [r7, #2056]
	ldrb	r0, [r2, #1]!	@ zero_extendqisi2
	str	r1, [sp, #16]
	str	r3, [sp, #12]
	str	r0, [r7, #2052]
	mov	r0, #80
	str	ip, [sp, #8]
	str	r2, [sp, #4]
	str	r2, [sp, #20]
	bl	timer_delay_ns
	ldr	r0, [r7, #2048]
	ldr	r2, [sp, #4]
	ldr	r3, [sp, #12]
	cmp	r2, r10
	ldr	ip, [sp, #8]
	ldr	r1, [sp, #16]
	strb	r0, [r3, #1]!
	bne	.L1919
	mov	ip, r5
	mov	r0, #0
.L1920:
	add	r7, r1, r0
	mov	r3, #1
.L1921:
	ldrb	lr, [r7, r3, asl #2]	@ zero_extendqisi2
	ldrb	r10, [ip]	@ zero_extendqisi2
	add	lr, lr, r10
	strb	lr, [ip, r3, asl #3]
	add	r3, r3, #1
	cmp	r3, #7
	bne	.L1921
	add	r0, r0, #1
	add	ip, ip, #1
	cmp	r0, #4
	bne	.L1920
	mov	r3, #0
	add	r9, r9, #1
	strb	r3, [r5, #16]
	strb	r3, [r5, #24]
	strb	r3, [r5, #32]
	strb	r3, [r5, #40]
	strb	r3, [r5, #48]
	strb	r3, [r5, #41]
	strb	r3, [r5, #49]
	b	.L1917
.L2008:
	sub	r3, r5, #3
	cmp	r3, #5
	bhi	.L1924
	smulbb	r2, fp, r6
	mov	r3, r2, asl #4
	mov	r2, r2, asr #1
	str	r3, [sp, #28]
	mov	r3, r2, asl #1
	str	r3, [sp, #8]
	mov	r3, #0
.L2016:
	str	r3, [sp, #12]
	ldr	r3, .L2018
	ldrb	r2, [r3, #585]	@ zero_extendqisi2
	ldrb	r3, [sp, #12]	@ zero_extendqisi2
	cmp	r2, r3
	bhi	.L1968
.L1924:
	strb	fp, [r4, #113]
	strb	r6, [r4, #114]
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1968:
	ldr	r2, .L2018
	mov	r10, #160
	mov	r9, #255
	add	r3, r2, r3
	ldrb	r3, [r3, #588]	@ zero_extendqisi2
	mov	r0, r3
	str	r3, [sp]
	bl	zftl_flash_exit_slc_mode
	ldr	r3, [sp]
	mla	r10, r10, r3, r4
	add	r3, r10, #144
	str	r3, [sp, #24]
	ldr	r3, .L2018
	ldr	r3, [r3, #528]
	str	r3, [sp, #4]
	ldr	r3, [sp]
	mov	r8, r3, asl #8
	ldr	r3, [sp, #4]
	add	r7, r3, r8
	str	r9, [r7, #2056]
	bl	nandc_wait_flash_ready
	cmp	r5, #8
	bne	.L1926
	add	r3, r4, #144
	mov	r2, #23
	mov	r1, #25
	str	r3, [sp, #24]
	mov	r3, #120
	str	r3, [r7, #2056]
	mov	r3, #0
	str	r3, [r7, #2052]
	str	r3, [r7, #2052]
	str	r3, [r7, #2052]
	str	r2, [r7, #2056]
	mov	r2, #4
	str	r2, [r7, #2056]
	str	r1, [r7, #2056]
	mov	r1, #218
	str	r1, [r7, #2056]
	mov	r1, #21
	str	r3, [r7, #2056]
	str	r3, [r7, #2052]
	str	r3, [r7, #2052]
	str	r1, [r7, #2052]
	str	r2, [r7, #2052]
	str	r3, [r7, #2052]
	b	.L1927
.L1926:
	cmp	r5, #4
	mov	r3, #54
	str	r3, [r7, #2056]
	moveq	r3, #64
	streq	r9, [r7, #2052]
	streq	r3, [r7, #2048]
	moveq	r3, #204
	beq	.L2013
.L1928:
	sub	r3, r5, #5
	cmp	r3, #1
	ldrlsb	r3, [r4, #128]	@ zero_extendqisi2
	strls	r3, [r7, #2052]
	movls	r3, #82
	bls	.L2014
	cmp	r5, #7
	bne	.L1929
	mov	r3, #174
	str	r3, [r7, #2052]
	mov	r3, #0
	str	r3, [r7, #2048]
	mov	r3, #176
.L2013:
	str	r3, [r7, #2052]
	mov	r3, #77
.L2014:
	str	r3, [r7, #2048]
.L1929:
	ldr	r3, [sp, #4]
	cmp	r5, #6
	add	r8, r3, r8
	mov	r3, #22
	str	r3, [r8, #2056]
	mov	r3, #23
	str	r3, [r8, #2056]
	mov	r3, #4
	str	r3, [r8, #2056]
	mov	r3, #25
	str	r3, [r8, #2056]
	mov	r3, #0
	str	r3, [r8, #2056]
	str	r3, [r8, #2052]
	str	r3, [r8, #2052]
	moveq	r3, #31
	str	r3, [r8, #2052]
	mov	r3, #2
	str	r3, [r8, #2052]
	mov	r3, #0
	str	r3, [r8, #2052]
.L1927:
	ldmia	sp, {r2, r3}
	sub	r10, r5, #8
	sub	r8, r5, #5
	clz	r10, r10
	add	r3, r3, r2, asl #8
	mov	r2, #48
	mov	r10, r10, lsr #5
	str	r2, [r3, #2056]
	bl	nandc_wait_flash_ready
	cmp	r8, #1
	movhi	r3, #0
	movls	r3, #1
	str	r3, [sp, #16]
	orrs	r3, r3, r10
	movne	ip, #16
	bne	.L1933
	cmp	r5, #7
	movne	ip, #2
	moveq	ip, #32
.L1933:
	ldr	r3, .L2018+12
	ldmia	sp, {r1, r2}
	ldr	r3, [r3, #-3288]
	add	r1, r2, r1, asl #8
	mov	r0, r3
.L1934:
	ldr	r2, [r1, #2048]
	strb	r2, [r0], #1
	rsb	r2, r3, r0
	uxtb	r2, r2
	cmp	r2, ip
	bcc	.L1934
	cmp	r10, #0
	beq	.L1935
	mov	r2, #0
.L1937:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r0, r2
	cmp	ip, #50
	beq	.L1936
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #5
	beq	.L1936
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L1937
	b	.L1938
.L1936:
	cmp	r0, #7
	bne	.L1939
.L1938:
	ldr	r0, .L2018+16
	mov	r1, #0
	bl	printk
.L1940:
	b	.L1940
.L1935:
	cmp	r5, #7
	bne	.L1941
	mov	r2, r10
.L1943:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r0, r2
	cmp	ip, #12
	beq	.L1942
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #10
	beq	.L1942
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L1943
	b	.L1944
.L1942:
	cmp	r0, #7
	bne	.L1939
.L1944:
	ldr	r0, .L2018+16
	mov	r1, #0
	bl	printk
.L1945:
	b	.L1945
.L1941:
	cmp	r5, #6
	bne	.L1939
	sub	r2, r3, #1
	add	r3, r3, #7
.L1946:
	ldrb	r0, [r2, #1]!	@ zero_extendqisi2
	cmp	r0, #12
	beq	.L1939
	ldrb	r0, [r2, #8]	@ zero_extendqisi2
	cmp	r0, #4
	beq	.L1939
	cmp	r2, r3
	bne	.L1946
	ldr	r0, .L2018+16
	mov	r1, #0
	bl	printk
.L1948:
	b	.L1948
.L1939:
	ldr	r3, .L2018+12
	ldr	r0, [sp, #28]
	ldr	r2, [r3, #-3288]
	add	r0, r2, r0
	mov	r3, r2
.L1949:
	cmp	r3, r0
	ldrne	ip, [r1, #2048]
	strneb	ip, [r3], #1
	bne	.L1949
.L2017:
	ldr	r3, .L2018+12
	mov	r0, #8
	ldr	r1, [sp, #8]
	ldr	r3, [r3, #-3288]
	add	r1, r3, r1
	str	r1, [sp, #20]
.L1952:
	ldr	ip, [sp, #8]
	add	lr, r1, ip
.L1951:
	ldrh	ip, [r1]
	mvn	ip, ip
	strh	ip, [r1], #2	@ movhi
	cmp	r1, lr
	bne	.L1951
	ldr	ip, [sp, #8]
	subs	r0, r0, #1
	add	r1, r1, ip
	bne	.L1952
.L1953:
	mov	ip, #0
	mov	r0, ip
.L1956:
	mov	r1, #1
	mov	lr, #0
	mov	r1, r1, asl r0
	mov	r8, #16
	mov	r7, lr
.L1954:
	ldrh	r9, [r3, lr]
	and	r9, r9, r1
	cmp	r9, r1
	addeq	r7, r7, #1
	ldr	r9, [sp, #8]
	subs	r8, r8, #1
	add	lr, lr, r9
	bne	.L1954
	cmp	r7, #8
	add	r0, r0, #1
	orrhi	ip, ip, r1
	uxthhi	ip, ip
	cmp	r0, #16
	bne	.L1956
	ldr	r1, [sp, #20]
	strh	ip, [r3], #2	@ movhi
	cmp	r3, r1
	bne	.L1953
	ldr	r3, .L2018+12
	ldr	r1, [r3, #-3288]
	mov	r3, #0
	sub	r0, r1, #4
	add	ip, r1, #28
.L1958:
	ldr	lr, [r0, #4]!
	cmp	lr, #0
	addeq	r3, r3, #1
	cmp	r0, ip
	bne	.L1958
	cmp	r3, #7
	ble	.L1959
	ldr	r0, .L2018+20
	mov	r2, #1
	mov	r3, #1024
	bl	rknand_print_hex
	ldr	r0, .L2018+16
	mov	r1, #0
	bl	printk
.L1960:
	b	.L1960
.L1959:
	cmp	r5, #6
	moveq	r0, #4
	beq	.L1961
	cmp	r5, #7
	moveq	r0, #10
	beq	.L1961
	cmp	r10, #0
	moveq	r0, #8
	movne	r0, #5
.L1961:
	sub	r9, fp, #1
	ldr	r1, [sp, #24]
	mov	ip, #0
	uxtb	r9, r9
	add	r9, r9, #1
.L1962:
	mov	r7, r1
	mov	r3, r2
.L1963:
	ldrb	lr, [r3], #1	@ zero_extendqisi2
	strb	lr, [r7], #1
	rsb	lr, r2, r3
	uxtb	lr, lr
	cmp	lr, fp
	bcc	.L1963
	add	ip, ip, #1
	add	r2, r2, r9
	cmp	ip, r6
	add	r1, r1, r0
	blt	.L1962
	ldmia	sp, {r2, r3}
	mov	r8, #255
	add	r7, r3, r2, asl #8
	str	r8, [r7, #2056]
	bl	nandc_wait_flash_ready
	ldr	r3, [sp, #16]
	cmp	r3, #0
	beq	.L1965
	mov	r3, #54
	str	r3, [r7, #2056]
	ldrb	r3, [r4, #128]	@ zero_extendqisi2
	mov	r2, #22
	str	r3, [r7, #2052]
	mov	r3, #0
	str	r3, [r7, #2048]
	str	r2, [r7, #2056]
	str	r3, [r7, #2056]
	str	r3, [r7, #2052]
	str	r3, [r7, #2052]
	mov	r3, #48
	str	r8, [r7, #2052]
	str	r8, [r7, #2052]
	str	r8, [r7, #2052]
	b	.L2015
.L1965:
	cmp	r10, #0
	movne	r3, #190
	moveq	r3, #56
.L2015:
	str	r3, [r7, #2056]
	bl	nandc_wait_flash_ready
	ldr	r3, [sp, #12]
	add	r3, r3, #1
	b	.L2016
.L2019:
	.align	2
.L2018:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+396
	.word	.LANCHOR3
	.word	.LC141
	.word	.LC142
	.fnend
	.size	hynix_get_read_retry_default, .-hynix_get_read_retry_default
	.align	2
	.global	flash_get_read_retry_tbl
	.type	flash_get_read_retry_tbl, %function
flash_get_read_retry_tbl:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L2022
	ldrb	r0, [r3, #23]	@ zero_extendqisi2
	sub	r3, r0, #1
	cmp	r3, #7
	bxhi	lr
	b	hynix_get_read_retry_default
.L2023:
	.align	2
.L2022:
	.word	.LANCHOR2
	.fnend
	.size	flash_get_read_retry_tbl, .-flash_get_read_retry_tbl
	.align	2
	.global	nandc_xfer_done
	.type	nandc_xfer_done, %function
nandc_xfer_done:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	ldr	r5, .L2073
	ldr	r7, .L2073+4
	ldr	r0, [r5, #528]
	mov	r4, r5
	bl	wait_for_nandc_xfer_completed
	ldrb	r3, [r5, #516]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r6, [r5, #528]
	cmp	r3, #9
	strb	r1, [r7, #-3284]
	bne	.L2025
	ldr	r3, [r6, #16]
	str	r3, [sp]
	ldr	r5, [r6, #48]
	ubfx	r5, r5, #1, #1
	cmp	r5, r1
	moveq	r7, r4
	beq	.L2027
.L2026:
	ldr	r2, [r6, #64]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #6
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	bge	.L2029
	ldr	r3, [r6]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L2028
	ldr	r3, [sp, #4]
	tst	r3, #131072
	beq	.L2028
	ldr	r1, [sp, #4]
	ldr	r0, .L2073+8
	ubfx	r1, r1, #17, #1
	bl	printk
	b	.L2029
.L2028:
	ldr	r3, [sp]
	add	r1, r1, #1
	ubfx	r3, r3, #22, #6
	cmp	r1, r3, asl #12
	bne	.L2026
	ldr	r2, [r6, #64]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L2073+12
	ubfx	r3, r3, #22, #6
	bl	printk
	mov	r3, #64
	ldr	r0, .L2073+16
	mov	r2, #4
	ldr	r1, [r4, #528]
	bl	rknand_print_hex
	ldr	r3, [sp, #4]
	tst	r3, #8192
	mov	r3, #1
	strb	r3, [r7, #-3284]
	bne	.L2029
	ldr	r3, .L2073+20
	ldr	r0, .L2073+24
	ldr	r3, [r3, #4]
	blx	r3
.L2029:
	ldr	r3, [r4, #1680]
	ldr	r5, .L2073
	cmp	r3, #0
	beq	.L2033
	ldr	r1, [sp]
	mov	r2, #0
	ldr	r0, [r5, #1672]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r5, #1676]
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #2
	b	.L2072
.L2035:
	ldr	r3, [r4, #528]
	add	r5, r5, #1
	ldr	r3, [r3, #16]
	str	r3, [sp]
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L2027
	ldr	r2, [sp]
	mov	r1, r5
	ldr	r3, [r6, #64]
	ldr	r0, .L2073+28
	ubfx	r3, r3, #16, #6
	bl	printk
	ldr	r0, .L2073+16
	ldr	r1, [r7, #528]
	mov	r2, #4
	mov	r3, #64
	bl	rknand_print_hex
.L2027:
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L2035
	ldr	r3, [r4, #1680]
	ldr	r5, .L2073
	cmp	r3, #0
	beq	.L2033
	ldr	r1, [sp]
	mov	r2, #1
	ldr	r0, [r5, #1672]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r5, #1676]
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #2
	b	.L2071
.L2025:
	ldr	r3, [r6, #8]
	str	r3, [sp]
	ldr	r5, [r6, #16]
	ubfx	r5, r5, #1, #1
	cmp	r5, #0
	moveq	r7, r4
	beq	.L2038
	mov	r5, r1
.L2037:
	ldr	r2, [r6, #28]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	bge	.L2040
	ldr	r3, [r4, #528]
	ldr	r3, [r3]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L2039
	ldr	r3, [sp, #4]
	tst	r3, #131072
	beq	.L2039
	ldr	r1, [sp, #4]
	ldr	r0, .L2073+32
	bl	printk
	b	.L2040
.L2039:
	add	r5, r5, #1
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L2037
	ldr	r2, [r6, #28]
	mov	r1, r5
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L2073+12
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L2073+16
	ldr	r1, [r4, #528]
	mov	r2, #4
	mov	r3, #64
	bl	rknand_print_hex
	b	.L2037
.L2040:
	ldr	r3, [r4, #1680]
	ldr	r5, .L2073
	cmp	r3, #0
	beq	.L2033
	ldr	r1, [sp]
	mov	r2, #0
	ldr	r0, [r5, #1672]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r5, #1676]
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #7
.L2072:
	mov	r2, #0
	b	.L2070
.L2044:
	ldr	r3, [r4, #528]
	add	r5, r5, #1
	ldr	r3, [r3, #8]
	str	r3, [sp]
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L2038
	ldr	r2, [sp]
	mov	r1, r5
	ldr	r3, [r6, #28]
	ldr	r0, .L2073+28
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L2073+16
	ldr	r1, [r7, #528]
	mov	r2, #4
	mov	r3, #64
	bl	rknand_print_hex
.L2038:
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L2044
	ldr	r3, [r4, #1680]
	ldr	r5, .L2073
	cmp	r3, #0
	beq	.L2033
	ldr	r1, [sp]
	mov	r2, #1
	ldr	r0, [r5, #1672]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #10
	bl	rknand_dma_unmap_single
	ldr	r0, [r5, #1676]
	ldr	r1, [sp]
	ubfx	r1, r1, #22, #6
	mov	r1, r1, asl #7
.L2071:
	mov	r2, #1
.L2070:
	bl	rknand_dma_unmap_single
.L2033:
	mov	r3, #0
	str	r3, [r4, #1680]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2074:
	.align	2
.L2073:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC143
	.word	.LC144
	.word	.LC145
	.word	arm_delay_ops
	.word	322122000
	.word	.LC146
	.word	.LC147
	.fnend
	.size	nandc_xfer_done, .-nandc_xfer_done
	.align	2
	.global	nandc_xfer
	.type	nandc_xfer, %function
nandc_xfer:
	.fnstart
	@ args = 4, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r0, r1
	ldr	r7, [sp, #32]
	mov	r4, r1
	mov	r8, r2
	mov	r1, r2
	mov	r6, r3
	mov	r2, r3
	mov	r3, r7
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	cmp	r4, #0
	movne	r0, #0
	bne	.L2105
	ldr	r5, .L2118
	ldrb	r3, [r5, #516]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L2077
	ldr	lr, [r5, #528]
	mov	r1, r8, lsr #2
	mov	ip, #1
	mov	r3, r4
	mov	r0, r4
.L2078:
	cmp	r3, r1
	bcs	.L2115
	add	r2, r3, #84
	ldr	r2, [lr, r2, asl #2]
	str	r2, [sp, #4]
	ldr	r4, [sp, #4]
	ldr	r2, [sp, #4]
	ubfx	r2, r2, #26, #1
	and	r2, r2, r4, lsr #10
	and	ip, ip, r2
	ldr	r2, [sp, #4]
	tst	r2, #4
	bne	.L2097
	ldr	r2, [sp, #4]
	tst	r2, #262144
	bne	.L2097
	ldr	r4, [sp, #4]
	ldr	r2, [sp, #4]
	ubfx	r4, r4, #3, #7
	ubfx	r2, r2, #19, #7
	cmp	r4, r2
	ldr	r2, [sp, #4]
	ubfxgt	r2, r2, #3, #7
	ubfxle	r2, r2, #19, #7
	cmp	r0, r2
	movcc	r0, r2
	b	.L2079
.L2097:
	mvn	r0, #0
.L2079:
	add	r3, r3, #1
	b	.L2078
.L2115:
	ldr	r3, [r5, #528]
	cmp	ip, #0
	movne	r0, #512
	ldr	r4, [r3]
	and	r3, r4, #139264
	cmp	r3, #139264
	bne	.L2084
	mov	r1, r4
	ldr	r0, .L2118+4
	bl	printk
	ldr	r3, [r5, #528]
	mvn	r0, #0
	orr	r4, r4, #131072
	str	r4, [r3]
.L2084:
	tst	r4, #8192
	beq	.L2085
	ldr	r3, .L2118+8
	ldrb	r3, [r3, #-3284]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2085
	ldr	r0, .L2118+12
	mov	r1, r4
	bl	printk
	ldr	r3, [r5, #528]
	mov	r2, #1
	str	r2, [r3, #16]
	b	.L2086
.L2077:
	ldrb	r3, [r5, #584]	@ zero_extendqisi2
	mov	r0, r8, lsr #1
	mov	r2, r7
	mov	r1, r4
	cmp	r3, #25
	mov	r3, r4
	movcc	lr, #64
	movcs	lr, #128
.L2088:
	cmp	r1, r0
	add	ip, r3, lr
	add	r2, r2, #4
	bcs	.L2116
	ldr	r4, [r5, #1660]
	mov	r3, r3, lsr #2
	add	r1, r1, #1
	ldr	r3, [r4, r3, asl #2]
	mov	r4, r3, lsr #8
	strb	r3, [r2, #-4]
	strb	r4, [r2, #-3]
	mov	r4, r3, lsr #16
	mov	r3, r3, lsr #24
	strb	r4, [r2, #-2]
	strb	r3, [r2, #-1]
	mov	r3, ip
	b	.L2088
.L2116:
	ldr	lr, [r5, #528]
	mov	ip, #0
	mov	r1, r8, lsr #2
	mov	r0, ip
.L2090:
	cmp	ip, r1
	bcs	.L2117
	add	r3, ip, #8
	ldr	r3, [lr, r3, asl #2]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #4
	bne	.L2100
	ldr	r3, [sp, #4]
	tst	r3, #32768
	bne	.L2100
	ldr	r2, [sp, #4]
	ldr	r8, [sp, #4]
	ldr	r3, [sp, #4]
	ubfx	r2, r2, #3, #5
	ldr	r4, [sp, #4]
	ubfx	r8, r8, #27, #1
	ubfx	r3, r3, #16, #5
	ubfx	r4, r4, #29, #1
	orr	r2, r2, r8, asl #5
	orr	r3, r3, r4, asl #5
	cmp	r2, r3
	ldr	r2, [sp, #4]
	ldrhi	r3, [sp, #4]
	ubfxhi	r2, r2, #3, #5
	ldrls	r3, [sp, #4]
	ubfxls	r2, r2, #16, #5
	ubfxhi	r3, r3, #27, #1
	ubfxls	r3, r3, #29, #1
	orr	r3, r2, r3, asl #5
	cmp	r0, r3
	movcc	r0, r3
	b	.L2091
.L2100:
	mvn	r0, #0
.L2091:
	add	ip, ip, #1
	b	.L2090
.L2117:
	ldr	r3, [r5, #528]
	mov	r2, #0
	str	r2, [r3, #16]
	ldr	r4, [r3]
	and	r3, r4, #139264
	cmp	r3, #139264
	bne	.L2085
	mov	r1, r4
	ldr	r0, .L2118+16
	bl	printk
	ldr	r3, .L2118
	orr	r4, r4, #131072
	ldr	r3, [r3, #528]
	str	r4, [r3]
	b	.L2086
.L2085:
	cmn	r0, #1
	beq	.L2105
	ldr	r3, [r7]
	cmn	r3, #1
	bne	.L2105
	ldr	r3, [r7, #4]
	cmn	r3, #1
	bne	.L2105
	ldr	r3, [r6]
	cmn	r3, #1
	moveq	r0, #512
	b	.L2105
.L2086:
	mvn	r0, #0
.L2105:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2119:
	.align	2
.L2118:
	.word	.LANCHOR0
	.word	.LC148
	.word	.LANCHOR3
	.word	.LC149
	.word	.LC150
	.fnend
	.size	nandc_xfer, .-nandc_xfer
	.align	2
	.global	flash_read_page
	.type	flash_read_page, %function
flash_read_page:
	.fnstart
	@ args = 4, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r3
	ldr	r7, .L2135
	mvn	r4, #0
	.pad #20
	sub	sp, sp, #20
	mov	r5, r0
	mov	ip, r1
	ubfx	r8, r1, #24, #2
	ldrb	r3, [r7, #1700]	@ zero_extendqisi2
	str	r2, [sp, #12]
	rsb	r3, r3, #24
	ldr	r6, [r7, #528]
	bic	r4, r1, r4, asl r3
	ldr	r3, .L2135+4
	ldr	r3, [r3]
	tst	r3, #16
	beq	.L2121
	ldr	r0, .L2135+8
	mov	r1, r5
	mov	r2, r8
	mov	r3, ip
	bl	printk
.L2121:
	bl	nandc_wait_flash_ready
	mov	r0, r5
	bl	nandc_cs
	cmp	r8, #0
	mov	r9, r5, asl #8
	bne	.L2122
	mov	r0, r5
	bl	zftl_flash_enter_slc_mode
	b	.L2123
.L2122:
	ldr	r3, [r7, #1620]
	ldrb	r3, [r3, #12]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L2124
	ldr	r3, .L2135
	ldrb	r3, [r3, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	addeq	r3, r6, r9
	streq	r8, [r3, #2056]
	beq	.L2123
.L2124:
	mov	r0, r5
	bl	zftl_flash_exit_slc_mode
.L2123:
	add	r10, r6, r9
	mov	r2, #0
	str	r2, [r10, #2056]
	str	r2, [r10, #2052]
	str	r2, [r10, #2052]
	uxtb	r2, r4
	str	r2, [r10, #2052]
	mov	r2, r4, lsr #8
	str	r2, [r10, #2052]
	mov	r2, r4, lsr #16
	str	r2, [r10, #2052]
	mov	r2, #48
	str	r2, [r10, #2056]
	ldr	r2, [r7, #1620]
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	adds	r2, r8, #0
	movne	r2, #1
	cmp	r1, #3
	movne	r2, #0
	cmp	r2, #0
	beq	.L2125
	ldr	r2, .L2135
	ldrb	r2, [r2, #1778]	@ zero_extendqisi2
	cmp	r2, #0
	addeq	r4, r4, r4, asl #1
	subeq	r0, r4, #1
	addeq	r0, r0, r8
	beq	.L2134
.L2125:
	mov	r0, r4
.L2134:
	bl	nandc_set_seed
	add	r6, r6, r9
	bl	nandc_wait_flash_ready
	mov	r4, #0
	mov	r1, r4
	mov	r2, #5
	mov	r3, #224
	str	r2, [r6, #2056]
	mov	r0, r5
	str	r4, [r10, #2052]
	str	r4, [r10, #2052]
	str	r3, [r6, #2056]
	ldrb	r2, [sp, #56]	@ zero_extendqisi2
	ldr	r3, [sp, #12]
	str	fp, [sp]
	bl	nandc_xfer
	mov	r5, r0
	mov	r0, r4
	bl	nandc_de_cs
	mov	r0, r5
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2136:
	.align	2
.L2135:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC151
	.fnend
	.size	flash_read_page, .-flash_read_page
	.align	2
	.global	micron_read_retrial
	.type	micron_read_retrial, %function
micron_read_retrial:
	.fnstart
	@ args = 4, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #36
	sub	sp, sp, #36
	ldr	r4, .L2165
	mov	r6, r0
	str	r3, [sp, #24]
	mov	r9, r1
	str	r2, [sp, #20]
	mov	r8, #0
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	ldr	r10, .L2165+4
	add	r3, r3, r3, asl #1
	mov	r3, r3, asr #2
	str	r3, [sp, #12]
	bl	nandc_wait_flash_ready
	ldr	r3, [r4, #528]
	str	r3, [sp, #16]
.L2138:
	ldr	r3, [sp, #16]
	mov	r7, #0
	mvn	r4, #0
	add	r5, r3, r6, asl #8
.L2139:
	ldrb	r3, [r10, #-3276]	@ zero_extendqisi2
	cmp	r7, r3
	bcs	.L2143
	mov	r3, #239
	mov	r0, #200
	str	r3, [r5, #2056]
	mov	r3, #137
	str	r3, [r5, #2052]
	add	fp, r7, #1
	bl	timer_delay_ns
	str	fp, [r5, #2048]
	mov	r3, #0
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
	mov	r0, r6
	str	r3, [r5, #2048]
	mov	r1, r9
	ldr	r3, [sp, #72]
	ldr	r2, [sp, #20]
	str	r3, [sp]
	ldr	r3, [sp, #24]
	bl	flash_read_page
	ldr	r3, .L2165+8
	ldr	r3, [r3]
	tst	r3, #4096
	mov	ip, r0
	beq	.L2140
	str	r0, [sp]
	mov	r1, r7
	str	r0, [sp, #28]
	mov	r2, r9
	ldr	r0, .L2165+12
	mov	r3, r4
	bl	printk
	ldr	ip, [sp, #28]
.L2140:
	cmn	ip, #1
	beq	.L2141
	ldr	r3, [r10, #-3288]
	cmn	r4, #1
	moveq	r4, ip
	str	r3, [sp, #20]
	ldr	r3, [r10, #-3280]
	str	r3, [sp, #24]
	ldr	r3, [sp, #12]
	cmp	ip, r3
	bcc	.L2150
.L2141:
	mov	r7, fp
	b	.L2139
.L2150:
	mov	r4, ip
.L2143:
	ldr	r3, [sp, #16]
	mov	r5, r6, asl #8
	mov	r0, #200
	add	fp, r3, r5
	mov	r3, #239
	str	r3, [fp, #2056]
	mov	r3, #137
	str	r3, [fp, #2052]
	bl	timer_delay_ns
	mov	r3, #0
	str	r3, [fp, #2048]
	str	r3, [fp, #2048]
	str	r3, [fp, #2048]
	str	r3, [fp, #2048]
	ldr	r3, [sp, #12]
	cmp	r4, r3
	bcc	.L2145
	cmn	r4, #1
	movne	r4, #256
.L2145:
	cmn	r4, #1
	movne	fp, #0
	moveq	fp, #1
	cmp	r4, #256
	movne	r3, fp
	orreq	r3, fp, #1
	cmp	r3, #0
	beq	.L2146
	mov	r1, r7
	mov	r3, r7
	str	r4, [sp]
	mov	r2, r9
	ldr	r0, .L2165+16
	eor	r7, r8, #1
	bl	printk
	ands	r7, fp, r7
	beq	.L2147
	bl	nandc_wait_flash_ready
	ldr	r3, .L2165
	mov	r0, #200
	mov	r8, #1
	ldr	r3, [r3, #528]
	add	r5, r3, r5
	mov	r3, #239
	str	r3, [r5, #2056]
	mov	r3, #150
	str	r3, [r5, #2052]
	bl	timer_delay_ns
	mov	r3, #3
	str	r3, [r5, #2048]
	mov	r3, #0
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
	b	.L2138
.L2147:
	cmp	r8, #0
	beq	.L2148
	bl	nandc_wait_flash_ready
	ldr	r3, .L2165
	mov	r0, #200
	ldr	r9, [r3, #528]
	mov	r3, #239
	add	r6, r9, r6, asl #8
	add	r5, r9, r5
	str	r3, [r5, #2056]
	mov	r3, #150
	str	r3, [r6, #2052]
	bl	timer_delay_ns
	cmn	r4, #1
	str	r7, [r6, #2048]
	movne	r4, #256
	str	r7, [r6, #2048]
	str	r7, [r6, #2048]
	str	r7, [r6, #2048]
	b	.L2148
.L2146:
	cmp	r8, #0
	beq	.L2148
	str	r3, [sp, #12]
	bl	nandc_wait_flash_ready
	ldr	r2, .L2165
	mov	r0, #200
	mov	r4, #256
	ldr	r9, [r2, #528]
	mov	r2, #239
	add	r6, r9, r6, asl #8
	add	r5, r9, r5
	str	r2, [r5, #2056]
	mov	r2, #150
	str	r2, [r6, #2052]
	bl	timer_delay_ns
	ldr	r3, [sp, #12]
	str	r3, [r6, #2048]
	str	r3, [r6, #2048]
	str	r3, [r6, #2048]
	str	r3, [r6, #2048]
.L2148:
	bl	nandc_wait_flash_ready
	mov	r0, r4
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2166:
	.align	2
.L2165:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR2
	.word	.LC152
	.word	.LC153
	.fnend
	.size	micron_read_retrial, .-micron_read_retrial
	.align	2
	.global	toshiba_3d_read_retrial
	.type	toshiba_3d_read_retrial, %function
toshiba_3d_read_retrial:
	.fnstart
	@ args = 4, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ubfx	r4, r1, #24, #2
	mov	r10, r0
	mov	fp, r3
	mov	r9, r1
	str	r2, [sp, #12]
	bl	nandc_wait_flash_ready
	ldr	r7, .L2215
	add	r6, r10, #8
	ldr	r3, [r7, #528]
	add	r6, r3, r6, asl #8
	str	r3, [sp, #8]
	ldrb	r3, [r7, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	movne	r3, #56
	moveq	r3, #46
	movne	r5, #10
	moveq	r5, #26
	cmp	r4, #0
	str	r3, [sp, #16]
	mvn	r4, #0
	bne	.L2169
	ldr	r3, [sp, #8]
	mov	r8, #1
	add	r3, r3, r10, asl #8
	str	r3, [sp, #16]
.L2176:
	ldr	r3, .L2215
	mov	r0, r6
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2170
	mov	r2, #0
	mov	r1, r8
	bl	toshiba_tlc_set_rr_para
	mov	r3, #93
	ldr	r2, [sp, #16]
	str	r3, [r2, #2056]
	b	.L2171
.L2170:
	uxtb	r1, r8
	bl	toshiba_3d_set_slc_rr_para
.L2171:
	ldr	r3, [sp, #64]
	mov	r0, r10
	mov	r1, r9
	ldr	r2, [sp, #12]
	str	r3, [sp]
	mov	r3, fp
	bl	flash_read_page
	ldr	r3, .L2215+4
	ldr	r3, [r3]
	tst	r3, #16
	mov	ip, r0
	beq	.L2172
	mov	r3, ip
	ldr	r0, .L2215+8
	mov	r1, r8
	mov	r2, r9
	str	ip, [sp, #20]
	bl	printk
	ldr	ip, [sp, #20]
.L2172:
	cmn	ip, #1
	beq	.L2173
	ldr	r3, .L2215+12
	cmn	r4, #1
	moveq	r4, ip
	ldr	r2, [r3, #-3288]
	ldr	fp, [r3, #-3280]
	ldrb	r3, [r7, #584]	@ zero_extendqisi2
	str	r2, [sp, #12]
	add	r3, r3, r3, asl #1
	cmp	ip, r3, asr #2
	bcc	.L2191
.L2173:
	add	r8, r8, #1
	cmp	r8, r5
	bne	.L2176
	b	.L2175
.L2191:
	mov	r5, r8
	mov	r4, ip
.L2175:
	ldr	r3, .L2215
	mov	r1, #0
	mov	r0, r6
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	moveq	r2, r1
	beq	.L2214
	bl	toshiba_3d_set_slc_rr_para
	b	.L2178
.L2169:
	ldr	r3, [sp, #8]
	mov	r5, #1
	add	r8, r3, r10, asl #8
.L2185:
	ldr	r3, .L2215
	mov	r0, r6
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2179
	mov	r1, r5
	mov	r2, #1
	bl	toshiba_tlc_set_rr_para
	mov	r3, #93
	b	.L2213
.L2179:
	uxtb	r1, r5
	bl	toshiba_3d_set_tlc_rr_para
	mov	r3, #38
.L2213:
	str	r3, [r8, #2056]
	mov	r0, r10
	ldr	r3, [sp, #64]
	mov	r1, r9
	ldr	r2, [sp, #12]
	str	r3, [sp]
	mov	r3, fp
	bl	flash_read_page
	ldr	r3, .L2215+4
	ldr	r3, [r3]
	tst	r3, #16
	mov	ip, r0
	beq	.L2181
	mov	r3, ip
	ldr	r0, .L2215+16
	mov	r1, r5
	mov	r2, r9
	str	ip, [sp, #20]
	bl	printk
	ldr	ip, [sp, #20]
.L2181:
	cmn	ip, #1
	beq	.L2182
	ldr	r3, .L2215+12
	cmn	r4, #1
	moveq	r4, ip
	ldr	r2, [r3, #-3288]
	ldr	fp, [r3, #-3280]
	ldrb	r3, [r7, #584]	@ zero_extendqisi2
	str	r2, [sp, #12]
	add	r3, r3, r3, asl #1
	cmp	ip, r3, asr #2
	bcc	.L2192
.L2182:
	ldr	r3, [sp, #16]
	add	r5, r5, #1
	cmp	r5, r3
	bne	.L2185
	b	.L2184
.L2192:
	mov	r4, ip
.L2184:
	ldr	r3, .L2215
	mov	r0, r6
	mov	r1, #0
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2186
	mov	r2, #1
.L2214:
	bl	toshiba_tlc_set_rr_para
	b	.L2178
.L2186:
	bl	toshiba_3d_set_tlc_rr_para
.L2178:
	ldr	r3, .L2215
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2187
	ldr	r3, [sp, #8]
	add	r10, r3, r10, asl #8
	mov	r3, #85
	str	r3, [r10, #2056]
	mov	r3, #0
	str	r3, [r10, #2052]
	str	r3, [r10, #2048]
	mov	r3, #255
	str	r3, [r10, #2056]
.L2187:
	ldrb	r3, [r7, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r4, r3, asr #2
	bcc	.L2188
	cmn	r4, #1
	movne	r4, #256
.L2188:
	cmn	r4, #1
	cmpne	r4, #256
	bne	.L2189
	str	r4, [sp]
	mov	r1, r5
	ldr	r0, .L2215+20
	mov	r2, r9
	mov	r3, r5
	bl	printk
.L2189:
	bl	nandc_wait_flash_ready
	mov	r0, r4
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2216:
	.align	2
.L2215:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC154
	.word	.LANCHOR3
	.word	.LC155
	.word	.LC156
	.fnend
	.size	toshiba_3d_read_retrial, .-toshiba_3d_read_retrial
	.align	2
	.global	toshiba_read_retrial
	.type	toshiba_read_retrial, %function
toshiba_read_retrial:
	.fnstart
	@ args = 4, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #36
	sub	sp, sp, #36
	mov	r8, r0
	ldr	r4, .L2250
	str	r3, [sp, #20]
	add	r9, r8, #8
	str	r1, [sp, #12]
	str	r2, [sp, #16]
	bl	nandc_wait_flash_ready
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	ldr	r6, [r4, #528]
	sub	r3, r3, #67
	cmp	r3, #1
	add	r9, r6, r9, asl #8
	mov	r3, r8, asl #8
	str	r3, [sp, #24]
	movls	r3, #0
	strls	r3, [sp, #8]
	bls	.L2218
	ldrb	r5, [r4, #1658]	@ zero_extendqisi2
	cmp	r5, #0
	streq	r5, [sp, #8]
	beq	.L2219
	mov	r0, #1
	bl	nandc_set_if_mode
	mov	r3, #1
	str	r3, [sp, #8]
.L2219:
	ldr	r3, [sp, #24]
	mov	r2, #92
	add	r3, r6, r3
	str	r2, [r3, #2056]
	mov	r2, #197
	str	r2, [r3, #2056]
.L2218:
	ldr	fp, .L2250+4
	mov	r7, #1
	mvn	r10, #0
	mov	r3, r8, asl #8
	str	r3, [sp, #28]
.L2220:
	ldrb	r3, [fp, #-3276]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r7, r3
	bcs	.L2249
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	mov	r0, r9
	uxtb	r1, r7
	sub	r3, r3, #67
	cmp	r3, #1
	bhi	.L2221
	bl	sandisk_set_rr_para
	b	.L2222
.L2221:
	bl	toshiba_set_rr_para
.L2222:
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L2223
	ldr	r3, .L2250+4
	ldrb	r3, [r3, #-3276]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r7, r3
	ldreq	r3, [sp, #28]
	moveq	r2, #179
	addeq	r3, r6, r3
	streq	r2, [r3, #2056]
.L2223:
	ldr	r3, [sp, #24]
	mov	r2, #38
	add	r1, sp, #12
	mov	r0, r8
	add	r3, r6, r3
	str	r2, [r3, #2056]
	mov	r2, #93
	str	r2, [r3, #2056]
	ldr	r3, [sp, #72]
	str	r3, [sp]
	ldmia	r1, {r1, r2, r3}
	bl	flash_read_page
	cmn	r0, #1
	mov	r5, r0
	beq	.L2226
	ldr	r3, [fp, #-3288]
	cmn	r10, #1
	moveq	r10, r0
	str	r3, [sp, #16]
	ldr	r3, [fp, #-3280]
	str	r3, [sp, #20]
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L2228
.L2226:
	add	r7, r7, #1
	b	.L2220
.L2249:
	mov	r5, r10
.L2228:
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	mov	r0, r9
	mov	r1, #0
	sub	r3, r3, #67
	cmp	r3, #1
	bhi	.L2230
	bl	sandisk_set_rr_para
	b	.L2231
.L2230:
	bl	toshiba_set_rr_para
.L2231:
	add	r6, r6, r8, asl #8
	mov	r3, #255
	str	r3, [r6, #2056]
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r5, r3, asr #2
	bcc	.L2232
	cmn	r5, #1
	movne	r5, #256
.L2232:
	cmn	r5, #1
	cmpne	r5, #256
	bne	.L2233
	str	r5, [sp]
	mov	r1, r7
	ldr	r0, .L2250+8
	mov	r3, r7
	ldr	r2, [sp, #12]
	bl	printk
.L2233:
	bl	nandc_wait_flash_ready
	ldr	r3, [sp, #8]
	cmp	r3, #0
	beq	.L2234
	mov	r0, #4
	bl	nandc_set_if_mode
.L2234:
	mov	r0, r5
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2251:
	.align	2
.L2250:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC156
	.fnend
	.size	toshiba_read_retrial, .-toshiba_read_retrial
	.align	2
	.global	hynix_read_retrial
	.type	hynix_read_retrial, %function
hynix_read_retrial:
	.fnstart
	@ args = 4, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r3
	ldr	r3, .L2271
	.pad #28
	sub	sp, sp, #28
	mov	r10, r2
	mov	r7, r0
	mov	r8, r1
	mov	r6, #0
	ldr	r2, [r3, #1696]
	mvn	r4, #0
	str	r3, [sp, #16]
	add	r3, r2, r0
	str	r3, [sp, #12]
	ldrb	r9, [r2, #114]	@ zero_extendqisi2
	ldrb	r5, [r3, #120]	@ zero_extendqisi2
	bl	nandc_wait_flash_ready
	ldr	ip, .L2271+4
.L2253:
	cmp	r6, r9
	bcs	.L2257
	add	r5, r5, #1
	mov	r0, r7
	str	ip, [sp, #20]
	uxtb	r5, r5
	cmp	r5, r9
	movcs	r5, #0
	mov	r1, r5
	bl	hynix_set_rr_para
	mov	r0, r7
	mov	r1, r8
	mov	r2, r10
	ldr	r3, [sp, #64]
	str	r3, [sp]
	mov	r3, fp
	bl	flash_read_page
	cmn	r0, #1
	ldr	ip, [sp, #20]
	beq	.L2255
	ldr	r3, [sp, #16]
	cmn	r4, #1
	ldr	r10, [ip, #-3288]
	moveq	r4, r0
	ldr	fp, [ip, #-3280]
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L2261
.L2255:
	add	r6, r6, #1
	b	.L2253
.L2261:
	mov	r4, r0
.L2257:
	ldr	r3, [sp, #12]
	strb	r5, [r3, #120]
	ldr	r3, [sp, #16]
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r4, r3, asr #2
	bcc	.L2259
	cmn	r4, #1
	movne	r4, #256
.L2259:
	cmn	r4, #1
	cmpne	r4, #256
	bne	.L2260
	str	r4, [sp]
	mov	r1, r6
	ldr	r0, .L2271+8
	mov	r2, r8
	mov	r3, r6
	bl	printk
.L2260:
	bl	nandc_wait_flash_ready
	mov	r0, r4
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2272:
	.align	2
.L2271:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC157
	.fnend
	.size	hynix_read_retrial, .-hynix_read_retrial
	.align	2
	.global	flash_ddr_tuning_read
	.type	flash_ddr_tuning_read, %function
flash_ddr_tuning_read:
	.fnstart
	@ args = 4, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #36
	sub	sp, sp, #36
	mov	r8, r0
	mov	r10, r1
	mov	fp, r2
	str	r3, [sp, #16]
	bl	nandc_get_ddr_para
	mov	r4, #0
	mov	r5, r4
	mov	r6, #1024
	mov	r7, #6
	mvn	r9, #0
	str	r0, [sp, #24]
	str	r4, [sp, #20]
	str	r4, [sp, #12]
.L2279:
	uxtb	r0, r7
	bl	nandc_set_ddr_para
	mov	r0, r8
	mov	r1, r10
	mov	r2, fp
	ldr	r3, [sp, #72]
	str	r3, [sp]
	ldr	r3, [sp, #16]
	bl	flash_read_page
	ldr	r3, .L2297
	ldr	r3, [r3]
	tst	r3, #16
	mov	ip, r0
	beq	.L2274
	mov	r3, ip
	ldr	r0, .L2297+4
	mov	r1, r7
	mov	r2, r10
	str	ip, [sp, #28]
	bl	printk
	ldr	ip, [sp, #28]
.L2274:
	add	r3, r6, #1
	cmp	ip, r3
	bhi	.L2275
	ldr	r3, .L2297+8
	ldr	fp, [r3, #-3288]
	ldr	r3, [r3, #-3280]
	str	r3, [sp, #16]
	ldr	r3, .L2297+12
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	cmp	ip, r3, lsr #2
	bcs	.L2285
	add	r5, r5, #1
	cmp	r5, #7
	bls	.L2285
	rsb	r4, r5, r7
	mov	r6, ip
	mov	r9, #0
	b	.L2277
.L2275:
	ldr	r3, [sp, #12]
	cmp	r3, r5
	bcs	.L2286
	cmp	r5, #7
	rsb	r3, r5, r4
	str	r3, [sp, #20]
	bhi	.L2278
	str	r5, [sp, #12]
	b	.L2286
.L2285:
	mov	r4, r7
	mov	r6, ip
	mov	r9, #0
	b	.L2276
.L2286:
	mov	r5, #0
.L2276:
	add	r7, r7, #2
	cmp	r7, #50
	bne	.L2279
.L2277:
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #20]
	cmp	r2, r5
	movhi	r4, r3
.L2278:
	cmp	r4, #0
	beq	.L2280
	ldr	r3, .L2297+12
	ldr	r2, .L2297+16
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	umull	r2, r3, r3, r2
	ubfx	r3, r3, #1, #8
	cmp	r6, r3
	bcs	.L2280
	ldr	r0, .L2297+20
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	b	.L2296
.L2280:
	ldrb	r0, [sp, #24]	@ zero_extendqisi2
.L2296:
	bl	nandc_set_ddr_para
	cmn	r9, #1
	bne	.L2282
	mov	r1, r8
	mov	r2, r10
	ldr	r0, .L2297+24
	bl	printk
	mov	r0, r8
	bl	flash_reset
	ldr	r4, .L2297+12
	mov	r0, #1
	bl	flash_set_interface_mode
	mov	r0, #1
	bl	nandc_set_if_mode
	add	r3, r4, r8
	mov	r0, r8
	mov	r2, #2
	strb	r2, [r3, #1704]
	bl	zftl_flash_enter_slc_mode
	mov	r1, r10
	mov	r2, fp
	mov	r0, r8
	mov	r5, r4
	ldr	r3, [sp, #72]
	str	r3, [sp]
	ldr	r3, [sp, #16]
	bl	flash_read_page
	mov	r1, r8
	mov	r2, r10
	mov	r6, r0
	ldr	r0, .L2297+28
	mov	r3, r6
	bl	printk
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	cmp	r6, r3
	bhi	.L2287
	ldr	r2, .L2297+8
	ldr	r3, [r2, #-3272]
	add	r3, r3, #1
	str	r3, [r2, #-3272]
	cmp	r3, #100
	movhi	r3, #0
	strhib	r3, [r4, #1658]
	bls	.L2283
	b	.L2282
.L2287:
	mov	r6, r9
.L2283:
	ldrb	r0, [r5, #1779]	@ zero_extendqisi2
	bl	flash_set_interface_mode
	ldrb	r0, [r5, #1779]	@ zero_extendqisi2
	bl	nandc_set_if_mode
.L2282:
	mov	r0, r6
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2298:
	.align	2
.L2297:
	.word	.LANCHOR2
	.word	.LC158
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC159
	.word	.LC160
	.word	.LC161
	.fnend
	.size	flash_ddr_tuning_read, .-flash_ddr_tuning_read
	.align	2
	.global	flash_read_page_en
	.type	flash_read_page_en, %function
flash_read_page_en:
	.fnstart
	@ args = 4, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r7, r3
	ldr	r5, .L2330
	mov	r10, r0
	mov	r4, r1
	mov	r8, r2
	ldr	r9, [sp, #48]
	ldrb	r3, [r5, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L2300
	ldr	r1, .L2330+4
	mov	r2, #404
	ldr	r0, .L2330+8
	bl	printk
	bl	dump_stack
.L2300:
	add	r2, r5, r10
	ldr	r3, .L2330
	ldrb	r6, [r2, #588]	@ zero_extendqisi2
	ldrb	r2, [r5, #585]	@ zero_extendqisi2
	cmp	r2, r10
	bcs	.L2301
	ldr	r3, .L2330+12
	ldr	r3, [r3]
	tst	r3, #64
	beq	.L2329
	ldr	r0, .L2330+16
	mov	r1, r6
	mov	r2, r10
	mov	r3, r4
	bl	printk
.L2329:
	mvn	r0, #0
	b	.L2326
.L2301:
	tst	r4, #50331648
	bne	.L2304
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2305
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2304
.L2305:
	ldrh	r10, [r5, #2]
	mov	r0, r4
	mov	r1, r10
	bl	__aeabi_uidiv
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L2330
	mul	r0, r10, r0
	rsb	r4, r0, r4
	mov	r4, r4, asl #1
	addeq	r4, r3, r4
	ldreqh	r4, [r4, #4]
	add	r4, r4, r0
.L2304:
	str	r9, [sp]
	mov	r0, r6
	mov	r1, r4
	mov	r2, r8
	mov	r3, r7
	bl	flash_read_page
	cmn	r0, #1
	bne	.L2326
	ldrb	fp, [r5, #1688]	@ zero_extendqisi2
	ldr	r10, .L2330
	cmp	fp, #0
	bne	.L2308
.L2311:
	ldr	r3, .L2330+20
	ldr	ip, [r3, #-3268]
	cmp	ip, #0
	bne	.L2309
	b	.L2310
.L2308:
	mov	r3, #0
	mov	r0, r6
	strb	r3, [r10, #1688]
	mov	r1, r4
	str	r9, [sp]
	mov	r2, r8
	mov	r3, r7
	bl	flash_read_page
	strb	fp, [r10, #1688]
	cmn	r0, #1
	beq	.L2311
	b	.L2326
.L2309:
	str	r9, [sp]
	mov	r0, r6
	mov	r1, r4
	mov	r2, r8
	mov	r3, r7
	blx	ip
	cmn	r0, #1
	bne	.L2326
.L2310:
	ldrb	r3, [r5, #1688]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, .L2330+24
	mov	r2, r4
	str	r3, [sp]
	mvn	r3, #0
	bl	printk
	ldrb	r3, [r5, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2329
	str	r9, [sp, #48]
	mov	r0, r6
	mov	r1, r4
	mov	r2, r8
	mov	r3, r7
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	flash_ddr_tuning_read
.L2326:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2331:
	.align	2
.L2330:
	.word	.LANCHOR0
	.word	.LANCHOR1+1516
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC162
	.word	.LANCHOR3
	.word	.LC163
	.fnend
	.size	flash_read_page_en, .-flash_read_page_en
	.align	2
	.global	flash_get_last_written_page
	.type	flash_get_last_written_page, %function
flash_get_last_written_page:
	.fnstart
	@ args = 4, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r1
	ldr	r6, .L2343
	movw	r1, #1708
	ldr	ip, .L2343+4
	.pad #20
	sub	sp, sp, #20
	mov	r10, r0
	mov	fp, r2
	ldrh	r7, [r6, #30]
	mov	r8, r3
	ldrh	r4, [ip, r1]
	ldr	r1, [sp, #56]
	sub	r4, r4, #1
	mul	r7, r7, r9
	uxth	r4, r4
	str	r1, [sp]
	sxtah	r1, r7, r4
	bl	flash_read_page_en
	cmp	r0, #512
	moveq	ip, #0
	bne	.L2333
.L2334:
	sxth	r3, ip
	sxth	r2, r4
	cmp	r3, r2
	bgt	.L2333
	add	r3, r3, r2
	mov	r0, r10
	mov	r2, fp
	str	ip, [sp, #12]
	add	r3, r3, r3, lsr #31
	mov	r5, r3, asr #1
	ldr	r3, [sp, #56]
	sxtah	r1, r7, r5
	str	r3, [sp]
	mov	r3, r8
	bl	flash_read_page_en
	cmp	r0, #512
	ldr	ip, [sp, #12]
	subeq	r5, r5, #1
	addne	ip, r5, #1
	uxtheq	r4, r5
	uxthne	ip, ip
	b	.L2334
.L2333:
	ldr	r3, [r6]
	sxth	r4, r4
	tst	r3, #4096
	beq	.L2338
	ldr	r0, .L2343+8
	mov	r1, r9
	mov	r2, r4
	ldr	r3, [r8]
	bl	printk
.L2338:
	mov	r0, r4
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2344:
	.align	2
.L2343:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC164
	.fnend
	.size	flash_get_last_written_page, .-flash_get_last_written_page
	.align	2
	.global	flash_get_last_written_page_ext
	.type	flash_get_last_written_page_ext, %function
flash_get_last_written_page_ext:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, lr}
	.save {r4, lr}
	.pad #8
	mov	r4, r1
	ldr	r1, .L2347
	mov	ip, r0
	mov	lr, r2
	movw	r0, #4028
	ldrb	r2, [r1, #1700]	@ zero_extendqisi2
	ldrh	r1, [r1, r0]
	rsb	r2, r2, #24
	str	r3, [sp]
	rsb	r2, r1, r2
	mvn	r1, #0
	mov	r0, ip, asr r2
	bic	r1, ip, r1, asl r2
	mov	r3, lr
	mov	r2, r4
	uxtb	r0, r0
	bl	flash_get_last_written_page
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, pc}
.L2348:
	.align	2
.L2347:
	.word	.LANCHOR0
	.fnend
	.size	flash_get_last_written_page_ext, .-flash_get_last_written_page_ext
	.align	2
	.global	flash_ddr_para_scan
	.type	flash_ddr_para_scan, %function
flash_ddr_para_scan:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	mov	r8, r0
	ldr	r4, .L2356
	mov	r7, r1
	mov	r6, #1
	ldr	r5, .L2356+4
	mov	r9, #4
	ldrb	r0, [r4, #1779]	@ zero_extendqisi2
	strb	r6, [r4, #1658]
	bl	flash_set_interface_mode
	ldrb	r0, [r4, #1779]	@ zero_extendqisi2
	bl	nandc_set_if_mode
	mov	r1, r7
	ldr	r2, [r5, #-3264]
	mov	r0, r8
	ldr	r3, [r5, #-3260]
	str	r9, [sp]
	bl	flash_ddr_tuning_read
	ldr	r3, [r5, #-3260]
	mov	r0, r8
	mov	r1, r7
	ldr	r2, [r5, #-3264]
	str	r9, [sp]
	bl	flash_read_page
	mov	r3, r4
	cmn	r0, #1
	bne	.L2350
	ldrb	r2, [r4, #1779]	@ zero_extendqisi2
	tst	r2, #1
	beq	.L2350
	mov	r0, r6
	bl	flash_set_interface_mode
	mov	r0, r6
	bl	nandc_set_if_mode
	mov	r3, #0
	strb	r3, [r4, #1658]
	b	.L2351
.L2350:
	mov	r2, #1
	strb	r2, [r3, #1658]
.L2351:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2357:
	.align	2
.L2356:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	flash_ddr_para_scan, .-flash_ddr_para_scan
	.global	__aeabi_uidivmod
	.align	2
	.type	id_block_read_data.constprop.31, %function
id_block_read_data.constprop.31:
	.fnstart
	@ args = 0, pretend = 0, frame = 112
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2389
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r2
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	.pad #124
	sub	sp, sp, #124
	ldr	r4, .L2389+4
	mov	r5, #0
	str	r1, [sp, #24]
	mov	r7, #4
	str	r2, [sp, #28]
	mov	r10, r5
	ldrh	r1, [r3, #30]
	ldrh	r3, [sp, #28]
	str	r0, [sp, #40]
	smulbb	r1, r1, r3
	ldrb	r3, [r4, #1688]	@ zero_extendqisi2
	strb	r5, [r4, #1688]
	str	r3, [sp, #44]
	uxth	r1, r1
	bl	__aeabi_uidivmod
	ldr	r3, [sp, #40]
	str	r1, [sp, #12]
	rsb	r3, r1, r3
	str	r3, [sp, #32]
	and	r3, r1, #3
	str	r3, [sp, #16]
.L2359:
	ldr	r3, [sp, #24]
	cmp	r5, r3
	bcs	.L2388
	ldr	r3, [sp, #16]
	mov	r1, r7
	rsb	r3, r3, r7
	uxth	r3, r3
	str	r3, [sp, #20]
	ldr	r3, [sp, #12]
	add	r0, r5, r3
	bl	__aeabi_uidiv
	ldrb	r1, [r4, #1625]	@ zero_extendqisi2
	cmp	r1, #0
	uxth	r0, r0
	mov	r3, r0, asl #1
	add	r2, r4, r3
	ldrh	r2, [r2, #4]
	moveq	r0, r2
	beq	.L2361
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	cmp	r2, #0
	movne	r0, r3
.L2361:
	ldr	r2, [sp, #32]
	ldr	r3, [sp, #28]
	ldr	r1, [sp, #16]
	mla	r3, r3, r0, r2
	str	r3, [sp, #36]
	ldr	r3, .L2389+4
	ldr	r2, [sp, #36]
	ldrb	r9, [r3, #1624]	@ zero_extendqisi2
	add	r0, r2, r1
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r3, .L2389+4
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	mov	fp, r0
	mov	r0, r9
	str	r3, [sp, #48]
	bl	nandc_bch_sel
.L2362:
	str	r7, [sp]
	mov	r0, #0
	mov	r1, fp
	mov	r2, r6
	add	r3, sp, #56
	bl	flash_read_page
	cmn	r0, #1
	mov	r8, r0
	bne	.L2369
	ldrb	ip, [r4, #1688]	@ zero_extendqisi2
	cmp	ip, #0
	bne	.L2364
.L2367:
	ldr	r3, .L2389+8
	ldr	ip, [r3, #-3268]
	cmp	ip, #0
	bne	.L2365
	b	.L2366
.L2364:
	mov	r0, #0
	str	r7, [sp]
	mov	r1, fp
	mov	r2, r6
	add	r3, sp, #56
	str	ip, [sp, #52]
	strb	r0, [r4, #1688]
	bl	flash_read_page
	ldr	ip, [sp, #52]
	cmn	r0, #1
	strb	ip, [r4, #1688]
	beq	.L2367
	b	.L2387
.L2365:
	str	r7, [sp]
	mov	r0, #0
	mov	r1, fp
	mov	r2, r6
	add	r3, sp, #56
	blx	ip
	cmn	r0, #1
	bne	.L2387
.L2366:
	ldrb	r3, [r4, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2369
	str	r7, [sp]
	mov	r0, #0
	mov	r1, fp
	mov	r2, r6
	add	r3, sp, #56
	bl	flash_ddr_tuning_read
.L2387:
	mov	r8, r0
.L2369:
	subs	r9, r9, #16
	movne	r9, #1
	cmn	r8, #1
	movne	r9, #0
	cmp	r9, #0
	beq	.L2370
	mov	r0, #16
	mov	r9, #16
	bl	nandc_bch_sel
	b	.L2362
.L2370:
	ldr	r0, [sp, #48]
	bl	nandc_bch_sel
	cmn	r8, #1
	mvneq	r10, #0
	ldr	r3, [sp, #32]
	ldr	r2, [sp, #40]
	cmp	r5, #0
	cmpeq	r3, r2
	moveq	r3, #1
	movne	r3, #0
	cmp	r10, #0
	movne	r3, #0
	andeq	r3, r3, #1
	cmp	r3, #0
	beq	.L2372
	ldr	r3, [r6]
	ldr	r2, .L2389+12
	cmp	r3, r2
	bne	.L2372
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #20]
	ldrb	r7, [r6, #17]	@ zero_extendqisi2
	add	r3, r3, r2
	str	r3, [sp, #24]
	b	.L2373
.L2372:
	ldr	r3, [sp, #20]
	mov	r2, r5
	ldr	r0, .L2389+16
	ldr	r1, [sp, #36]
	add	r6, r6, r3, asl #9
	ldr	r3, [sp, #60]
	str	r3, [sp]
	ldr	r3, [sp, #56]
	bl	printk
	mov	r3, #0
	str	r3, [sp, #16]
.L2373:
	ldr	r3, [sp, #20]
	add	r5, r3, r5
	uxth	r5, r5
	b	.L2359
.L2388:
	ldr	r3, .L2389+4
	mov	r0, r10
	ldrb	r2, [sp, #44]	@ zero_extendqisi2
	strb	r2, [r3, #1688]
	add	sp, sp, #124
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2390:
	.align	2
.L2389:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	1179535694
	.word	.LC165
	.fnend
	.size	id_block_read_data.constprop.31, .-id_block_read_data.constprop.31
	.align	2
	.global	flash_complete_plane_page_read
	.type	flash_complete_plane_page_read, %function
flash_complete_plane_page_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #8
	mvn	r3, #0
	ldr	r4, .L2408
	mov	r10, r0
	mov	r6, r1
	mov	r7, r2
	ldrb	r8, [r4, #1700]	@ zero_extendqisi2
	rsb	r9, r8, #24
	mvn	r8, r3, asl r8
	and	r3, r8, r0, lsr r9
	uxtb	r5, r3
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	cmp	r3, r5
	bhi	.L2392
	ldr	r1, .L2408+4
	movw	r2, #909
	ldr	r0, .L2408+8
	bl	printk
	bl	dump_stack
.L2392:
	ldrb	r2, [r4, #585]	@ zero_extendqisi2
	ldr	r3, .L2408
	cmp	r2, r5
	mvnls	r0, #0
	bls	.L2393
	add	r5, r3, r5
	mvn	r2, #0
	bic	r9, r10, r2, asl r9
	ubfx	r10, r10, #24, #2
	ldrb	r5, [r5, #588]	@ zero_extendqisi2
	ldr	r8, [r3, #528]
	mov	r0, r5
	bl	nandc_cs
	cmp	r10, #0
	bne	.L2394
	mov	r0, r9
	bl	slc_phy_page_address_calc
	mov	r9, r0
.L2394:
	ldrb	r3, [r4, #1642]	@ zero_extendqisi2
	uxtb	r0, r9
	mov	r1, r9, lsr #8
	mov	r2, r9, lsr #16
	cmp	r3, #1
	mov	r3, r5, asl #8
	addeq	r3, r8, r3
	moveq	ip, #6
	beq	.L2407
	ldr	ip, .L2408
	add	r3, r8, r3
	ldr	ip, [ip, #1620]
	ldrb	ip, [ip, #12]	@ zero_extendqisi2
	cmp	ip, #3
	bne	.L2397
	mov	ip, #5
.L2407:
	str	ip, [r3, #2056]
	mov	ip, #0
	str	ip, [r3, #2052]
	str	ip, [r3, #2052]
	str	r0, [r3, #2052]
	str	r1, [r3, #2052]
	str	r2, [r3, #2052]
	b	.L2405
.L2397:
	mov	ip, #0
	str	ip, [r3, #2056]
	str	ip, [r3, #2052]
	str	ip, [r3, #2052]
	str	r0, [r3, #2052]
	str	r1, [r3, #2052]
	str	r2, [r3, #2052]
	mov	r2, #5
	str	r2, [r3, #2056]
	str	ip, [r3, #2052]
	str	ip, [r3, #2052]
.L2405:
	mov	r2, #224
	str	r2, [r3, #2056]
	ldr	r3, [r4, #1620]
	ldrb	r2, [r3, #12]	@ zero_extendqisi2
	adds	r3, r10, #0
	movne	r3, #1
	cmp	r2, #3
	movne	r3, #0
	cmp	r3, #0
	beq	.L2398
	ldr	r3, .L2408
	ldrb	r3, [r3, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r3, r10, #1
	addeq	r0, r9, r9, asl #1
	addeq	r0, r3, r0
	beq	.L2406
.L2398:
	mov	r0, r9
.L2406:
	bl	nandc_set_seed
	ldr	r3, .L2408+12
	mov	r1, #0
	mov	r0, r5
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	mov	r3, r6
	str	r7, [sp]
	bl	nandc_xfer
	mov	r4, r0
	mov	r0, r5
	bl	nandc_de_cs
	mov	r0, r4
.L2393:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2409:
	.align	2
.L2408:
	.word	.LANCHOR0
	.word	.LANCHOR1+1536
	.word	.LC0
	.word	.LANCHOR2
	.fnend
	.size	flash_complete_plane_page_read, .-flash_complete_plane_page_read
	.align	2
	.global	flash_complete_page_read
	.type	flash_complete_page_read, %function
flash_complete_page_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, #1
	ldr	r5, .L2444
	.pad #20
	sub	sp, sp, #20
	mov	r9, r0
	mov	r10, r1
	mov	fp, r2
	ldrb	r7, [r5, #1700]	@ zero_extendqisi2
	rsb	r3, r7, #24
	mov	r7, r6, asl r7
	sub	r7, r7, #1
	mov	r4, r6, asl r3
	and	r7, r7, r0, lsr r3
	ldrb	r3, [r5, #585]	@ zero_extendqisi2
	sub	r4, r4, #1
	ubfx	r6, r0, #24, #2
	uxtb	r7, r7
	and	r4, r4, r0
	cmp	r3, r7
	bhi	.L2411
	ldr	r1, .L2444+4
	movw	r2, #1033
	ldr	r0, .L2444+8
	bl	printk
	bl	dump_stack
.L2411:
	add	r7, r5, r7
	ldrb	r8, [r7, #588]	@ zero_extendqisi2
	ldr	r7, [r5, #528]
	mov	r0, r8
	bl	nandc_cs
	cmp	r6, #0
	bne	.L2412
	mov	r0, r4
	bl	slc_phy_page_address_calc
	mov	r4, r0
.L2412:
	add	r3, r7, r8, asl #8
	mov	r2, #5
	str	r2, [r3, #2056]
	mov	r2, #0
	str	r2, [r3, #2052]
	str	r2, [r3, #2052]
	uxtb	r2, r4
	str	r2, [r3, #2052]
	mov	r2, r4, lsr #8
	str	r2, [r3, #2052]
	mov	r2, r4, lsr #16
	str	r2, [r3, #2052]
	mov	r2, #224
	str	r2, [r3, #2056]
	ldr	r3, [r5, #1620]
	ldrb	r2, [r3, #12]	@ zero_extendqisi2
	adds	r3, r6, #0
	movne	r3, #1
	cmp	r2, #3
	movne	r3, #0
	cmp	r3, #0
	beq	.L2413
	ldr	r3, .L2444
	ldrb	r3, [r3, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	subeq	r0, r6, #1
	addeq	r3, r4, r4, asl #1
	addeq	r0, r0, r3
	beq	.L2443
.L2413:
	mov	r0, r4
.L2443:
	bl	nandc_set_seed
	ldr	r7, .L2444+12
	mov	r0, r8
	mov	r1, #0
	mov	r3, r10
	ldrb	r2, [r7, #13]	@ zero_extendqisi2
	str	fp, [sp]
	bl	nandc_xfer
	cmn	r0, #1
	bne	.L2415
	ldrb	ip, [r5, #1688]	@ zero_extendqisi2
	cmp	ip, #0
	beq	.L2416
	ldr	r2, .L2444
	mov	r3, #0
	mov	r0, r8
	orr	r1, r4, r6, asl #24
	str	ip, [sp, #12]
	strb	r3, [r2, #1688]
	mov	r2, r10
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	str	r3, [sp]
	mov	r3, fp
	bl	flash_read_page
	ldr	r3, .L2444
	cmp	r6, #0
	ldr	ip, [sp, #12]
	strb	ip, [r3, #1688]
	bne	.L2417
.L2424:
	ldrb	r3, [r5]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2417
	ldr	r3, .L2444
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	blt	.L2417
	ldrb	r3, [r7, #23]	@ zero_extendqisi2
	sub	r3, r3, #4
	cmp	r3, #4
	movls	r0, #256
	b	.L2431
.L2417:
	cmn	r0, #1
	bne	.L2431
.L2425:
	ldr	r3, .L2444+16
	ldr	ip, [r3, #-3268]
	cmp	ip, #0
	bne	.L2420
.L2423:
	ldrb	r3, [r5, #1688]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, .L2444+20
	mov	r2, r9
	str	r3, [sp]
	mvn	r3, #0
	bl	printk
	ldrb	r3, [r5, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	mvneq	r0, #0
	beq	.L2431
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	mov	r0, r8
	orr	r1, r4, r6, asl #24
	mov	r2, r10
	str	r3, [sp]
	mov	r3, fp
	bl	flash_ddr_tuning_read
	b	.L2431
.L2420:
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	mov	r0, r8
	orr	r1, r4, r6, asl #24
	mov	r2, r10
	str	r3, [sp]
	mov	r3, fp
	blx	ip
	cmn	r0, #1
	bne	.L2431
	b	.L2423
.L2415:
	cmp	r6, #0
	beq	.L2424
	b	.L2431
.L2416:
	cmp	r6, #0
	beq	.L2424
	b	.L2425
.L2431:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2445:
	.align	2
.L2444:
	.word	.LANCHOR0
	.word	.LANCHOR1+1568
	.word	.LC0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.word	.LC166
	.fnend
	.size	flash_complete_page_read, .-flash_complete_page_read
	.align	2
	.type	queue_wait_first_req_completed, %function
queue_wait_first_req_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r5, .L2543
	ldrb	r4, [r5, #3323]	@ zero_extendqisi2
	cmp	r4, #255
	beq	.L2480
	mov	r3, #48
	mla	r3, r3, r4, r5
	ldrb	r2, [r3, #1826]	@ zero_extendqisi2
	ldr	r6, [r3, #1808]
	sub	r3, r2, #1
	cmp	r3, #10
	ldrls	pc, [pc, r3, asl #2]
	b	.L2480
.L2450:
	.word	.L2449
	.word	.L2451
	.word	.L2452
	.word	.L2452
	.word	.L2452
	.word	.L2452
	.word	.L2453
	.word	.L2454
	.word	.L2455
	.word	.L2452
	.word	.L2455
.L2449:
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	beq	.L2480
	mov	r3, #48
	mla	r3, r3, r4, r5
	ldr	r2, [r3, #1792]
	ldr	r1, [r3, #1788]
	cmp	r2, #0
	beq	.L2457
	ldrb	r0, [r3, #1824]	@ zero_extendqisi2
	ldr	r3, .L2543
	ldrb	r3, [r3, #3320]	@ zero_extendqisi2
	cmp	r0, r3
	moveq	r1, r2
.L2457:
	mov	r3, #48
	mov	r0, r6
	mla	r5, r3, r4, r5
	ldr	r2, [r5, #1796]
	bl	flash_complete_page_read
	str	r0, [r5, #1820]
	b	.L2541
.L2451:
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	mov	r2, #48
	mla	r3, r2, r4, r5
	ldr	ip, .L2543
	add	r1, r3, #1776
	add	r1, r1, #8
	ldrb	r6, [r3, #1784]	@ zero_extendqisi2
	tst	r0, #64
	beq	.L2480
	mla	r2, r2, r6, ip
	ldr	fp, [r1, #4]
	ldr	r8, [r2, #1788]
	ldr	r2, [r3, #1792]
	cmp	r2, #0
	beq	.L2458
	ldrb	r1, [r3, #1824]	@ zero_extendqisi2
	ldrb	r3, [ip, #3320]	@ zero_extendqisi2
	cmp	r1, r3
	moveq	fp, r2
.L2458:
	mov	r3, #48
	mla	r3, r3, r6, r5
	ldr	r2, [r3, #1792]
	cmp	r2, #0
	beq	.L2459
	ldrb	r1, [r3, #1824]	@ zero_extendqisi2
	ldr	r3, .L2543
	ldrb	r3, [r3, #3320]	@ zero_extendqisi2
	cmp	r1, r3
	moveq	r8, r2
.L2459:
	mov	r9, #48
	mov	r1, fp
	mla	r10, r9, r4, r5
	mla	r9, r9, r6, r5
	ldr	r2, [r10, #1796]
	ldr	r0, [r10, #1808]
	bl	flash_complete_plane_page_read
	ldr	r2, [r9, #1796]
	mov	r1, r8
	mov	r7, r0
	ldr	r0, [r9, #1808]
	bl	flash_complete_plane_page_read
	cmn	r7, #1
	mov	r9, r0
	beq	.L2460
	ldr	r3, [r10, #1804]
	cmn	r3, #1
	beq	.L2461
	ldr	r2, [r10, #1796]
	ldr	r2, [r2, #4]
	cmp	r3, r2
	beq	.L2461
.L2460:
	mov	r10, #48
	ldrb	r0, [r5, #1700]	@ zero_extendqisi2
	mla	r10, r10, r4, r5
	rsb	r1, r0, #24
	ldrb	ip, [r5, #3320]	@ zero_extendqisi2
	mvn	r3, #0
	mvn	r3, r3, asl r0
	ldr	r2, [r10, #1808]
	str	ip, [sp]
	and	r0, r3, r2, lsr r1
	bic	r1, r2, r3, asl r1
	mov	r2, fp
	ldr	r3, [r10, #1796]
	uxtb	r0, r0
	bl	flash_read_page_en
	ldr	r2, [r10, #1804]
	cmn	r2, #1
	mov	r7, r0
	beq	.L2462
	ldr	r3, [r10, #1796]
	ldr	r1, [r3, #4]
	cmp	r2, r1
	beq	.L2462
	ldr	r0, .L2543+4
	ldr	r0, [r0]
	tst	r0, #64
	beq	.L2462
	str	r1, [sp]
	ldr	r0, .L2543+8
	ldr	r1, [r10, #1808]
	ldr	r3, [r3]
	bl	printk
.L2462:
	mov	r3, #48
	mla	r3, r3, r4, r5
	ldr	r2, [r3, #1804]
	cmn	r2, #1
	beq	.L2461
	ldr	r3, [r3, #1796]
	ldr	r3, [r3, #4]
	cmp	r2, r3
	beq	.L2461
	ldr	r1, .L2543+12
	mov	r2, #360
	ldr	r0, .L2543+16
	bl	printk
	bl	dump_stack
.L2461:
	mov	r3, #48
	mov	r2, #13
	mla	r4, r3, r4, r5
	cmn	r9, #1
	strb	r2, [r4, #1826]
	ldrb	r2, [r4, #1786]	@ zero_extendqisi2
	str	r7, [r4, #1820]
	orr	r2, r2, #8
	strb	r2, [r4, #1786]
	beq	.L2463
	ldr	r2, .L2543
	mla	r3, r3, r6, r2
	ldr	r2, [r3, #1804]
	cmn	r2, #1
	beq	.L2465
	ldr	r3, [r3, #1796]
	ldr	r3, [r3, #4]
	cmp	r2, r3
	beq	.L2465
.L2463:
	mov	r4, #48
	ldrb	r0, [r5, #1700]	@ zero_extendqisi2
	mla	r4, r4, r6, r5
	rsb	r1, r0, #24
	ldrb	ip, [r5, #3320]	@ zero_extendqisi2
	mvn	r3, #0
	mvn	r3, r3, asl r0
	ldr	r2, [r4, #1808]
	str	ip, [sp]
	and	r0, r3, r2, lsr r1
	bic	r1, r2, r3, asl r1
	mov	r2, r8
	ldr	r3, [r4, #1796]
	uxtb	r0, r0
	bl	flash_read_page_en
	ldr	r2, [r4, #1804]
	cmn	r2, #1
	beq	.L2467
	ldr	r3, [r4, #1796]
	ldr	r1, [r3, #4]
	cmp	r2, r1
	beq	.L2467
	ldr	r0, .L2543+4
	ldr	r0, [r0]
	tst	r0, #64
	beq	.L2467
	str	r1, [sp]
	ldr	r0, .L2543+8
	ldr	r1, [r4, #1808]
	ldr	r3, [r3]
	bl	printk
.L2467:
	mov	r3, #48
	mla	r3, r3, r6, r5
	ldr	r2, [r3, #1804]
	cmn	r2, #1
	beq	.L2465
	ldr	r3, [r3, #1796]
	ldr	r3, [r3, #4]
	cmp	r2, r3
	beq	.L2465
	ldr	r1, .L2543+12
	movw	r2, #375
	ldr	r0, .L2543+16
	bl	printk
	bl	dump_stack
.L2465:
	mov	r3, #48
	mla	r5, r3, r6, r5
	str	r7, [r5, #1820]
.L2541:
	mov	r3, #13
	strb	r3, [r5, #1826]
	ldrb	r3, [r5, #1786]	@ zero_extendqisi2
	orr	r3, r3, #8
	strb	r3, [r5, #1786]
	b	.L2480
.L2452:
	bl	nandc_iqr_wait_flash_ready
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	mov	r7, r0
	beq	.L2480
	ands	r6, r0, #5
	mov	r3, #48
	beq	.L2468
	mla	r5, r3, r4, r5
	ldr	r0, .L2543+20
	mov	r3, #12
	mov	r2, r7
	mov	r6, r7
	strb	r3, [r5, #1826]
	ldrb	r1, [r5, #1785]	@ zero_extendqisi2
	str	r3, [sp]
	ldr	r3, [r5, #1808]
	bl	printk
	mvn	r3, #0
	str	r3, [r5, #1820]
	b	.L2448
.L2468:
	mul	r3, r3, r4
	mov	r2, #13
	ldr	ip, .L2543
	add	r7, r5, r3
	strb	r2, [r7, #1826]
	ldr	r2, [r5, #3348]
	str	r6, [r7, #1820]
	ldr	r1, [r2, #156]
	ldr	r2, .L2543+24
	cmp	r1, r2
	bne	.L2480
	ldr	r2, .L2543+28
	add	r3, r3, r2
	ldrh	r3, [r3, #2]
	cmp	r3, #0
	bne	.L2480
	ldrb	r0, [ip, #1700]	@ zero_extendqisi2
	mvn	r3, #0
	ldr	r2, [r7, #1808]
	rsb	r1, r0, #24
	ldr	r8, .L2543+32
	ldrb	ip, [ip, #3320]	@ zero_extendqisi2
	mvn	r3, r3, asl r0
	and	r0, r3, r2, lsr r1
	bic	r1, r2, r3, asl r1
	str	ip, [sp]
	uxtb	r0, r0
	ldr	r3, [r8, #-3260]
	ldr	r2, [r8, #-3264]
	bl	flash_read_page_en
	cmn	r0, #1
	mov	r3, r0
	beq	.L2469
	ldr	r1, [r7, #1796]
	ldr	r2, [r8, #-3260]
	ldr	r1, [r1]
	ldr	r2, [r2]
	cmp	r1, r2
	beq	.L2480
.L2469:
	mov	r2, #48
	ldr	r0, .L2543+36
	mla	r4, r2, r4, r5
	ldrb	r2, [r5, #1688]	@ zero_extendqisi2
	ldrb	r1, [r4, #1785]	@ zero_extendqisi2
	str	r2, [sp]
	ldr	r2, [r4, #1808]
	bl	printk
	mvn	r3, #0
	str	r3, [r4, #1820]
	b	.L2448
.L2455:
	cmp	r2, #11
	ldr	r2, .L2543+40
	ldrb	r3, [r5, #1700]	@ zero_extendqisi2
	mov	r1, #48
	mvn	r7, #0
	movne	r9, #3
	mla	ip, r1, r4, r2
	rsb	lr, r3, #24
	mvn	r3, r7, asl r3
	and	r0, r3, r6, lsr lr
	moveq	r9, #10
	mov	r10, r3
	uxth	r0, r0
.L2471:
	ldrb	r8, [ip]	@ zero_extendqisi2
	cmp	r8, #255
	beq	.L2542
	mul	r3, r1, r8
	ldr	r7, .L2543
	add	ip, r2, r3
	add	r3, r5, r3
	ldrb	fp, [r3, #1826]	@ zero_extendqisi2
	cmp	fp, r9
	bne	.L2471
	ldr	r3, [r3, #1808]
	and	r3, r10, r3, lsr lr
	cmp	r0, r3
	bne	.L2471
	bl	nandc_iqr_wait_flash_ready
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	mov	r6, r0
	beq	.L2540
	ands	r2, r0, #15
	mov	r3, #48
	mul	r3, r3, r4
	beq	.L2475
	add	r7, r7, r3
	mov	r9, #12
	ldr	r0, .L2543+44
	mov	r2, r6
	ldrb	r1, [r7, #1785]	@ zero_extendqisi2
	str	r9, [sp]
	ldr	r3, [r7, #1808]
	bl	printk
	mvn	r3, #0
	strb	r9, [r7, #1826]
	str	r3, [r7, #1820]
	b	.L2474
.L2475:
	add	r6, r7, r3
	mov	r1, #13
	str	r2, [r6, #1820]
	ldr	r2, [r7, #3348]
	strb	r1, [r6, #1826]
	ldr	r1, [r2, #156]
	ldr	r2, .L2543+24
	cmp	r1, r2
	bne	.L2540
	ldr	r2, .L2543+40
	add	r3, r2, r3
	ldrh	r3, [r3, #34]
	cmp	r3, #0
	bne	.L2540
	ldr	r2, [r6, #1808]
	mvn	r3, #0
	ldrb	r0, [r7, #1700]	@ zero_extendqisi2
	ldr	r9, .L2543+32
	rsb	r1, r0, #24
	ldrb	ip, [r7, #3320]	@ zero_extendqisi2
	mvn	r3, r3, asl r0
	and	r0, r3, r2, lsr r1
	bic	r1, r2, r3, asl r1
	str	ip, [sp]
	uxtb	r0, r0
	ldr	r3, [r9, #-3260]
	ldr	r2, [r9, #-3264]
	bl	flash_read_page_en
	cmn	r0, #1
	mov	r3, r0
	beq	.L2477
	ldr	r1, [r6, #1796]
	ldr	r2, [r9, #-3260]
	ldr	r1, [r1]
	ldr	r2, [r2]
	cmp	r1, r2
	beq	.L2540
.L2477:
	mov	r6, #48
	ldrb	r2, [r5, #1688]	@ zero_extendqisi2
	mla	r6, r6, r4, r5
	ldr	r0, .L2543+48
	ldrb	r1, [r6, #1785]	@ zero_extendqisi2
	str	r2, [sp]
	ldr	r2, [r6, #1808]
	bl	printk
	mvn	r3, #0
	str	r3, [r6, #1820]
.L2540:
	mov	r6, #0
.L2474:
	mov	r3, #48
	mla	r4, r3, r4, r5
	mla	r5, r3, r8, r5
	ldrb	r2, [r4, #1826]	@ zero_extendqisi2
	ldr	r3, [r4, #1820]
	strb	r2, [r5, #1826]
	str	r3, [r5, #1820]
	b	.L2448
.L2542:
	mvn	r6, #0
	b	.L2448
.L2453:
	mov	r0, r6
	mov	r1, #32
	bl	flash_wait_device_ready
	tst	r0, #32
	beq	.L2480
	ands	r6, r0, #15
	mov	r3, #48
	mlaeq	r5, r3, r4, r5
	movne	r2, #12
	mlane	r3, r3, r4, r5
	moveq	r3, #13
	movne	r6, r0
	streqb	r3, [r5, #1826]
	streq	r6, [r5, #1820]
	strneb	r2, [r3, #1826]
	mvnne	r2, #0
	strne	r2, [r3, #1820]
	b	.L2448
.L2454:
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	movne	r3, #48
	mlane	r5, r3, r4, r5
	movne	r3, #7
	strne	r0, [r5, #1820]
	strneb	r3, [r5, #1826]
.L2480:
	mov	r6, #0
.L2448:
	mov	r0, r6
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2544:
	.align	2
.L2543:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC167
	.word	.LANCHOR1+1596
	.word	.LC0
	.word	.LC168
	.word	1145785929
	.word	.LANCHOR0+1816
	.word	.LANCHOR3
	.word	.LC169
	.word	.LANCHOR0+1784
	.word	.LC170
	.word	.LC171
	.fnend
	.size	queue_wait_first_req_completed, .-queue_wait_first_req_completed
	.align	2
	.global	sblk_wait_write_queue_completed
	.type	sblk_wait_write_queue_completed, %function
sblk_wait_write_queue_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r4, .L2550
.L2546:
	ldrb	r3, [r4, #3323]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2549
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2546
.L2549:
	ldmfd	sp!, {r4, pc}
.L2551:
	.align	2
.L2550:
	.word	.LANCHOR0
	.fnend
	.size	sblk_wait_write_queue_completed, .-sblk_wait_write_queue_completed
	.align	2
	.global	ftl_read_page
	.type	ftl_read_page, %function
ftl_read_page:
	.fnstart
	@ args = 4, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r0
	mov	r6, r1
	mov	r5, r2
	mov	r4, r3
	ldr	r8, [sp, #24]
	bl	sblk_wait_write_queue_completed
	mov	r0, r7
	mov	r1, r6
	mov	r2, r5
	mov	r3, r4
	str	r8, [sp, #24]
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	b	flash_read_page_en
	.fnend
	.size	ftl_read_page, .-ftl_read_page
	.align	2
	.global	ftl_read_ppa_page
	.type	ftl_read_ppa_page, %function
ftl_read_ppa_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	mov	r8, r3
	ldr	r3, .L2556
	mov	r7, r1
	mov	r6, r2
	mvn	r2, #0
	mov	r5, r0
	ldrb	r1, [r3, #1700]	@ zero_extendqisi2
	rsb	r4, r1, #24
	mvn	r2, r2, asl r1
	and	r4, r2, r0, lsr r4
	bl	sblk_wait_write_queue_completed
	mov	r1, r5
	uxtb	r4, r4
	mov	r2, r7
	mov	r3, r6
	mov	r0, r4
	str	r8, [sp]
	bl	flash_read_page_en
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2557:
	.align	2
.L2556:
	.word	.LANCHOR0
	.fnend
	.size	ftl_read_ppa_page, .-ftl_read_ppa_page
	.align	2
	.global	sblk_read_page
	.type	sblk_read_page, %function
sblk_read_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r10, .L2586
	mov	r8, r0
	mov	r6, r1
	mov	r4, r0
	mov	r5, r1
.L2559:
	cmp	r5, #0
	beq	.L2582
	ldrb	fp, [r4]	@ zero_extendqisi2
	ldr	r9, [r4, #24]
.L2560:
	mov	r0, r9
	mov	r1, #0
	bl	queue_lun_state
	cmp	r0, #0
	beq	.L2584
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2560
.L2584:
	cmp	r5, #1
	beq	.L2566
	ldr	r3, .L2586+4
	ldrb	r3, [r3, #-3256]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2566
	ldr	r3, .L2586
	mvn	lr, #0
	ldrb	r2, [r3, #1700]	@ zero_extendqisi2
	rsb	r3, r2, #24
	mov	r3, r9, lsr r3
	bic	lr, r3, lr, asl r2
	uxth	r3, lr
	str	r3, [sp, #4]
	ldrb	r3, [r4]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L2565
	ldr	r1, .L2586+8
	movw	r2, #661
	ldr	r0, .L2586+12
	bl	printk
	bl	dump_stack
.L2565:
	ldrb	r3, [r4]	@ zero_extendqisi2
	mov	r2, #48
	ldrb	r0, [r10, #1700]	@ zero_extendqisi2
	mvn	ip, #0
	mul	r3, r2, r3
	rsb	r2, r0, #24
	add	r7, r10, r3
	ldr	r1, [r7, #1808]
	mov	r2, r1, lsr r2
	bic	r2, r2, ip, asl r0
	ldr	r0, [sp, #4]
	uxth	r2, r2
	cmp	r0, r2
	bne	.L2566
	ldr	r2, .L2586+16
	ldrh	r2, [r2]
	add	r9, r2, r9
	cmp	r1, r9
	bne	.L2566
	ldr	r0, [r4, #24]
	mvn	r9, #0
	str	r3, [sp, #12]
	add	r5, r5, r9
	ldrb	fp, [r7, #1784]	@ zero_extendqisi2
	bl	flash_start_plane_read
	mov	ip, #2
	mov	r2, #0
	strb	ip, [r4, #42]
	mov	r1, r4
	strb	r2, [r4, #43]
	ldr	r0, .L2586+20
	strb	r9, [r4]
	str	ip, [sp, #8]
	str	r2, [sp, #4]
	bl	buf_add_tail
	ldr	r1, .L2586+24
	strb	r9, [r7, #1784]
	ldr	r0, .L2586+20
	ldr	ip, [sp, #8]
	ldr	r2, [sp, #4]
	ldr	r3, [sp, #12]
	strb	ip, [r7, #1826]
	strb	r2, [r7, #1827]
	add	r1, r1, r3
	bl	buf_add_tail
	b	.L2564
.L2566:
	mov	r0, r4
	bl	queue_read_cmd
.L2564:
	subs	r5, r5, #1
	ldrne	r3, .L2586+24
	movne	r4, #48
	mlane	r4, r4, fp, r3
	bne	.L2559
.L2582:
	ldr	r4, .L2586+24
	mov	r5, #48
.L2570:
	cmp	r6, #0
	beq	.L2585
	ldrb	r3, [r8, #42]	@ zero_extendqisi2
	cmp	r3, #13
	bne	.L2571
	ldrb	r3, [r8]	@ zero_extendqisi2
	sub	r6, r6, #1
	cmp	r3, #255
	mlane	r8, r5, r3, r4
.L2571:
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2570
.L2585:
	mov	r0, r6
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2587:
	.align	2
.L2586:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR1+1628
	.word	.LC0
	.word	.LANCHOR0+4076
	.word	.LANCHOR0+3323
	.word	.LANCHOR0+1784
	.fnend
	.size	sblk_read_page, .-sblk_read_page
	.align	2
	.global	flash_prog_page
	.type	flash_prog_page, %function
flash_prog_page:
	.fnstart
	@ args = 4, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r10, r3
	ldr	r6, .L2600
	mov	fp, r2
	add	r8, r0, #8
	mov	r5, r0
	mov	r7, r1
	ubfx	r9, r1, #24, #2
	ldrb	r3, [r6, #1700]	@ zero_extendqisi2
	ldr	r4, [r6, #528]
	mvn	r6, #0
	rsb	r3, r3, #24
	bic	r6, r1, r6, asl r3
	ldr	r3, .L2600+4
	add	r8, r4, r8, asl #8
	ldr	r2, [r3]
	str	r3, [sp, #4]
	tst	r2, #16
	beq	.L2589
	ldr	r0, .L2600+8
	mov	r2, r9
	ldr	r3, [sp, #48]
	bl	printk
.L2589:
	bl	nandc_wait_flash_ready
	mov	r0, r5
	bl	hynix_reconfig_rr_para
	mov	r0, r5
	bl	nandc_cs
	cmp	r9, #0
	mov	r0, r5
	bne	.L2590
	bl	zftl_flash_enter_slc_mode
	b	.L2591
.L2590:
	bl	zftl_flash_exit_slc_mode
.L2591:
	add	r4, r4, r5, asl #8
	mov	r3, #128
	mov	r0, r6
	str	r3, [r4, #2056]
	mov	r3, #0
	str	r3, [r4, #2052]
	str	r3, [r4, #2052]
	uxtb	r3, r6
	str	r3, [r4, #2052]
	mov	r3, r6, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r6, lsr #16
	str	r3, [r4, #2052]
	bl	nandc_set_seed
	mov	r2, fp
	mov	r0, #1
	ldr	r3, [sp, #4]
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	mov	r3, r10
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	ldr	r3, .L2600+12
	ldr	r0, .L2600+16
	ldr	r3, [r3, #4]
	blx	r3
	mov	r3, #16
	str	r3, [r4, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	mov	r0, r8
	bl	flash_read_status
	mov	r4, r0
	mov	r0, r5
	bl	nandc_de_cs
	ands	r4, r4, #4
	beq	.L2592
	ldr	r0, .L2600+20
	mov	r1, r7
	mov	r2, r4
	bl	printk
.L2592:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2601:
	.align	2
.L2600:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC172
	.word	arm_delay_ops
	.word	64424400
	.word	.LC173
	.fnend
	.size	flash_prog_page, .-flash_prog_page
	.align	2
	.global	flash_test_blk
	.type	flash_test_blk, %function
flash_test_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r6, r0
	ldr	r4, .L2613
	mov	r5, r1
	mov	r2, #32
	mov	r1, #165
	ldr	r0, [r4, #-3288]
	bl	memset
	mov	r1, #90
	mov	r2, #8
	ldr	r0, [r4, #-3280]
	bl	memset
	ldr	r3, .L2613+4
	mov	r0, r6
	ldrh	r3, [r3, #2]
	mul	r5, r3, r5
	mov	r1, r5
	bl	flash_erase_block
	cmn	r0, #1
	bne	.L2603
.L2605:
	mvn	r4, #0
	b	.L2604
.L2603:
	ldr	r7, .L2613+8
	mov	r0, r6
	mov	r1, r5
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	str	r3, [sp]
	ldr	r2, [r4, #-3288]
	ldr	r3, [r4, #-3280]
	bl	flash_prog_page
	cmn	r0, #1
	beq	.L2605
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	mov	r0, r6
	mov	r1, r5
	str	r3, [sp]
	ldr	r2, [r4, #-3288]
	ldr	r3, [r4, #-3280]
	bl	flash_read_page
	cmn	r0, #1
	beq	.L2605
	ldr	r3, [r4, #-3288]
	ldr	r2, [r3]
	ldr	r3, .L2613+12
	cmp	r2, r3
	bne	.L2605
	ldr	r3, [r4, #-3280]
	ldr	r4, .L2613+16
	ldr	r3, [r3]
	subs	r4, r3, r4
	mvnne	r4, #0
.L2604:
	mov	r0, r6
	mov	r1, r5
	bl	flash_erase_block
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2614:
	.align	2
.L2613:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1515870811
	.word	1515870810
	.fnend
	.size	flash_test_blk, .-flash_test_blk
	.align	2
	.global	flash_dual_page_prog
	.type	flash_dual_page_prog, %function
flash_dual_page_prog:
	.fnstart
	@ args = 12, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r9, r3
	ldr	fp, .L2630
	add	r3, r0, #8
	ldr	r7, .L2630+4
	mvn	r5, #0
	mov	r6, r0
	mov	r8, r1
	ldr	r4, [fp, #528]
	mov	r10, r2
	add	r3, r4, r3, asl #8
	str	r3, [sp]
	ldrb	r3, [fp, #1700]	@ zero_extendqisi2
	ubfx	fp, r1, #24, #2
	rsb	r3, r3, #24
	bic	r5, r1, r5, asl r3
	ldr	r3, [r7]
	tst	r3, #16
	beq	.L2616
	ldr	r0, .L2630+8
	mov	r2, fp
	ldr	r3, [sp, #56]
	bl	printk
.L2616:
	bl	nandc_wait_flash_ready
	mov	r0, r6
	bl	nandc_cs
	cmp	fp, #0
	mov	r0, r6
	bne	.L2617
	bl	zftl_flash_enter_slc_mode
	b	.L2618
.L2617:
	bl	zftl_flash_exit_slc_mode
.L2618:
	add	r4, r4, r6, asl #8
	mov	ip, #128
	mov	fp, #0
	uxtb	r3, r5
	str	ip, [r4, #2056]
	mov	r0, r5
	str	fp, [r4, #2052]
	str	fp, [r4, #2052]
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #16
	str	r3, [r4, #2052]
	str	ip, [sp, #4]
	bl	nandc_set_seed
	ldrb	r1, [r7, #13]	@ zero_extendqisi2
	mov	r2, r10
	mov	r3, r9
	mov	r0, #1
	bl	nandc_xfer_start
	mov	r9, #16
	bl	nandc_xfer_done
	str	r9, [r4, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	add	r0, r5, #1
	uxtb	r3, r0
	ldr	ip, [sp, #4]
	str	ip, [r4, #2056]
	str	fp, [r4, #2052]
	str	fp, [r4, #2052]
	str	r3, [r4, #2052]
	mov	r3, r0, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r0, lsr r9
	str	r3, [r4, #2052]
	bl	nandc_set_seed
	ldrb	r1, [r7, #13]	@ zero_extendqisi2
	mov	r0, #1
	ldr	r2, [sp, #48]
	ldr	r3, [sp, #52]
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	str	r9, [r4, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r0, [sp]
	bl	flash_read_status
	mov	r4, r0
	mov	r0, r6
	bl	nandc_de_cs
	ands	r4, r4, #4
	beq	.L2619
	ldr	r3, .L2630+4
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L2619
	ldr	r0, .L2630+12
	mov	r1, r8
	mov	r2, r4
	bl	printk
.L2619:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2631:
	.align	2
.L2630:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC172
	.word	.LC173
	.fnend
	.size	flash_dual_page_prog, .-flash_dual_page_prog
	.section	.text.unlikely
	.align	2
	.type	fw_flash_page_prog.constprop.30, %function
fw_flash_page_prog.constprop.30:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #16
	mov	r6, r1
	ldr	r4, .L2637
	mov	r5, r2
	ldr	r3, [r4, #1620]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldrb	r8, [r4, #584]	@ zero_extendqisi2
	mov	r7, r0
	ldrb	r0, [r4, #1624]	@ zero_extendqisi2
	bl	nandc_bch_sel
	ldr	r3, .L2637+4
	ldrb	r3, [r3, #11]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L2633
	ldr	r4, .L2637+8
	mov	r1, #255
	mov	r2, #16384
	ldr	r0, [r4, #-3288]
	bl	memset
	ldr	r3, [r4, #-3288]
	mov	r1, r7
	mov	r2, r6
	mov	r0, #0
	str	r3, [sp]
	str	r3, [sp, #4]
	mov	r3, #4
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_dual_page_prog
	b	.L2636
.L2633:
	mov	r3, #4
	mov	r0, #0
	str	r3, [sp]
	mov	r1, r7
	mov	r2, r6
	mov	r3, r5
	bl	flash_prog_page
.L2636:
	mov	r4, r0
	mov	r0, r8
	bl	nandc_bch_sel
	mov	r0, r4
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2638:
	.align	2
.L2637:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.fnend
	.size	fw_flash_page_prog.constprop.30, .-fw_flash_page_prog.constprop.30
	.text
	.align	2
	.global	flash_start_page_prog
	.type	flash_start_page_prog, %function
flash_start_page_prog:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mvn	r5, #0
	ldr	r7, .L2650
	mov	fp, r2
	mov	r9, r0
	mov	r8, r1
	mov	r10, r3
	ldrb	r6, [r7, #1700]	@ zero_extendqisi2
	ldrb	r2, [r7, #585]	@ zero_extendqisi2
	rsb	ip, r6, #24
	mvn	r6, r5, asl r6
	and	r4, r6, r1, lsr ip
	uxtb	r4, r4
	cmp	r2, r4
	bhi	.L2640
	ldr	r1, .L2650+4
	movw	r2, #821
	ldr	r0, .L2650+8
	str	ip, [sp]
	bl	printk
	bl	dump_stack
	ldr	ip, [sp]
.L2640:
	ldrb	r1, [r7, #585]	@ zero_extendqisi2
	ldr	r2, .L2650
	cmp	r1, r4
	bls	.L2639
	ldr	r3, [r2, #528]
	add	r1, r2, r4
	mvn	r5, #0
	str	r2, [sp, #4]
	bic	r5, r8, r5, asl ip
	ldrb	r6, [r1, #588]	@ zero_extendqisi2
	str	r3, [sp]
	bl	nandc_rdy_status
	cmp	r0, #0
	ldr	r2, [sp, #4]
	bne	.L2642
	ldrb	r2, [r2, #585]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L2643
	bl	nandc_wait_flash_ready
	b	.L2642
.L2643:
	mov	r0, r4
	mov	r1, r5
	mov	r2, #64
	bl	flash_wait_device_ready_raw
.L2642:
	mov	r0, r6
	bl	hynix_reconfig_rr_para
	mov	r0, r6
	bl	nandc_cs
	tst	r8, #50331648
	bne	.L2644
	mov	r0, r5
	bl	slc_phy_page_address_calc
	ldrb	r3, [r7]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r5, r0
	beq	.L2645
	mov	r0, r6
	bl	zftl_flash_enter_slc_mode
	b	.L2645
.L2644:
	mov	r0, r6
	bl	zftl_flash_exit_slc_mode
.L2645:
	ldr	r3, [sp]
	mov	r0, r5
	add	r4, r3, r6, asl #8
	mov	r3, #128
	str	r3, [r4, #2056]
	mov	r3, #0
	str	r3, [r4, #2052]
	str	r3, [r4, #2052]
	uxtb	r3, r5
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #16
	str	r3, [r4, #2052]
	bl	nandc_set_seed
	ldr	r3, .L2650+12
	mov	r2, fp
	mov	r0, #1
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	mov	r3, r10
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	ldr	r3, .L2650+16
	ldr	r0, .L2650+20
	ldr	r3, [r3, #4]
	blx	r3
	str	r9, [r4, #2056]
	mov	r0, r6
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	nandc_de_cs
.L2639:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2651:
	.align	2
.L2650:
	.word	.LANCHOR0
	.word	.LANCHOR1+1644
	.word	.LC0
	.word	.LANCHOR2
	.word	arm_delay_ops
	.word	64424400
	.fnend
	.size	flash_start_page_prog, .-flash_start_page_prog
	.align	2
	.type	queue_prog_cmd, %function
queue_prog_cmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r2, [r4, #4]
	mov	r0, #16
	ldr	r3, [r4, #12]
	ldr	r1, [r4, #24]
	bl	flash_start_page_prog
	ldr	r2, .L2661
	ldr	r0, [r4, #24]
	ldrb	r3, [r2, #3323]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2653
	ldrb	ip, [r2, #1700]	@ zero_extendqisi2
	mvn	r1, #0
	mov	r5, #48
	rsb	r6, ip, #24
	mvn	r1, r1, asl ip
	uxth	r1, r1
	and	r0, r1, r0, asr r6
.L2655:
	mla	ip, r5, r3, r2
	ldr	lr, [ip, #1808]
	and	lr, r1, lr, lsr r6
	cmp	r0, lr
	bne	.L2654
	add	lr, ip, #1824
	ldrb	ip, [ip, #1826]	@ zero_extendqisi2
	cmp	ip, #7
	moveq	r3, #3
	streqb	r3, [lr, #2]
	beq	.L2653
.L2654:
	mla	r3, r5, r3, r2
	ldrb	r3, [r3, #1784]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L2655
.L2653:
	mov	r3, #3
	mov	r1, r4
	strb	r3, [r4, #42]
	mov	r3, #1
	ldr	r0, .L2661+4
	strb	r3, [r4, #43]
	mvn	r3, #0
	strb	r3, [r4]
	ldmfd	sp!, {r4, r5, r6, lr}
	b	buf_add_tail
.L2662:
	.align	2
.L2661:
	.word	.LANCHOR0
	.word	.LANCHOR0+3323
	.fnend
	.size	queue_prog_cmd, .-queue_prog_cmd
	.align	2
	.global	sblk_prog_page
	.type	sblk_prog_page, %function
sblk_prog_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r0
	ldrh	r3, [r0, #34]
	mov	r5, r1
	cmp	r3, #0
	beq	.L2664
	ldr	r3, .L2692
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L2664
	ldr	r0, .L2692+4
	mov	r2, r5
	ldr	r1, [r4, #24]
	bl	printk
.L2664:
	ldr	r8, .L2692+8
	mov	r6, #0
	mov	r9, r8
.L2665:
	cmp	r5, #0
	beq	.L2689
	ldrb	r3, [r4]	@ zero_extendqisi2
	ldr	r7, [r4, #24]
	str	r3, [sp, #4]
.L2666:
	mov	r0, r7
	mov	r1, #1
	bl	queue_lun_state
	cmp	r0, #0
	beq	.L2691
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2666
.L2691:
	cmp	r5, #1
	beq	.L2668
	ldrb	r3, [r8, #1780]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2669
.L2668:
	mov	r0, r4
	bl	queue_prog_cmd
	b	.L2670
.L2669:
	ldrb	r3, [r8, #1700]	@ zero_extendqisi2
	mvn	fp, #0
	rsb	ip, r3, #24
	mov	ip, r7, lsr ip
	bic	fp, ip, fp, asl r3
	ldrb	r3, [r4]	@ zero_extendqisi2
	cmp	r3, #255
	uxth	fp, fp
	bne	.L2671
	ldr	r1, .L2692+12
	movw	r2, #583
	ldr	r0, .L2692+16
	bl	printk
	bl	dump_stack
.L2671:
	ldrb	r3, [r4]	@ zero_extendqisi2
	mov	r2, #48
	mov	r10, #1
	mla	r3, r2, r3, r9
	ldrb	r2, [r9, #1700]	@ zero_extendqisi2
	ldr	ip, [r3, #1808]
	rsb	r3, r2, #24
	mov	r2, r10, asl r2
	sub	r2, r2, #1
	and	r2, r2, ip, lsr r3
	uxth	r2, r2
	cmp	fp, r2
	bne	.L2672
	ldr	r2, .L2692+20
	ldrb	lr, [r8, #4030]	@ zero_extendqisi2
	ldrh	fp, [r2]
	sub	lr, lr, #1
	rsb	r3, fp, r3
	uxth	r1, lr
	mov	r3, r10, asl r3
	mov	r0, r10, asl fp
	sub	r3, r3, #1
	and	r2, r1, r7, lsr fp
	sub	r0, r0, #1
	and	r1, r1, ip, lsr fp
	uxth	r3, r3
	and	r2, r2, r3
	and	r3, r3, r1
	subs	fp, r2, r3
	uxth	r0, r0
	and	r7, r7, r0
	and	ip, ip, r0
	movne	fp, #1
	cmp	r7, ip
	movne	fp, #0
	cmp	fp, #0
	beq	.L2673
	cmp	r6, lr
	beq	.L2672
	mov	r0, #17
	ldr	r1, [r4, #24]
	ldr	r2, [r4, #4]
	add	r6, r6, r10
	ldr	r3, [r4, #12]
	bl	flash_start_page_prog
	mov	r3, #9
	strb	r10, [r4, #43]
	mov	r1, r4
	strb	r3, [r4, #42]
	mvn	r3, #0
	ldr	r0, .L2692+24
	strb	r3, [r4]
	bl	buf_add_tail
	b	.L2670
.L2673:
	mov	r0, r4
	mov	r6, fp
	bl	queue_prog_cmd
	b	.L2670
.L2672:
	mov	r0, r4
	mov	r6, #0
	bl	queue_prog_cmd
.L2670:
	subs	r5, r5, #1
	ldrne	r3, .L2692+28
	movne	r4, #48
	ldrne	r2, [sp, #4]
	mlane	r4, r4, r2, r3
	bne	.L2665
.L2689:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2693:
	.align	2
.L2692:
	.word	.LANCHOR2
	.word	.LC174
	.word	.LANCHOR0
	.word	.LANCHOR1+1668
	.word	.LC0
	.word	.LANCHOR0+4028
	.word	.LANCHOR0+3323
	.word	.LANCHOR0+1784
	.fnend
	.size	sblk_prog_page, .-sblk_prog_page
	.align	2
	.global	ftl_flush
	.type	ftl_flush, %function
ftl_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L2700
	ldr	r5, .L2700+4
	ldrb	r1, [r3, #3345]	@ zero_extendqisi2
	mov	r4, r3
	cmp	r1, #0
	beq	.L2695
	ldrb	r2, [r5, #-3255]	@ zero_extendqisi2
	mov	r0, #48
	ldr	r3, .L2700+8
	mla	r0, r0, r2, r3
	bl	sblk_prog_page
.L2695:
	mvn	r3, #0
	strb	r3, [r5, #-3255]
	mov	r3, #0
	strb	r3, [r4, #3345]
	bl	sblk_wait_write_queue_completed
	bl	ftl_write_completed
	movw	r0, #65535
	ldmfd	sp!, {r3, r4, r5, lr}
	b	ftl_vpn_decrement
.L2701:
	.align	2
.L2700:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+1784
	.fnend
	.size	ftl_flush, .-ftl_flush
	.align	2
	.global	zftl_cache_flush
	.type	zftl_cache_flush, %function
zftl_cache_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	ldr	r3, .L2707
	ldrb	r3, [r3, #3345]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, pc}
	bl	timer_get_time
	ldr	r3, .L2707+4
	ldr	r3, [r3, #-3252]
	add	r3, r3, #100
	cmp	r0, r3
	ldmlsfd	sp!, {r3, pc}
	ldmfd	sp!, {r3, lr}
	b	ftl_flush
.L2708:
	.align	2
.L2707:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	zftl_cache_flush, .-zftl_cache_flush
	.align	2
	.global	flash_start_tlc_page_prog
	.type	flash_start_tlc_page_prog, %function
flash_start_tlc_page_prog:
	.fnstart
	@ args = 12, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r3
	ldr	r3, .L2717
	mov	r7, r2
	mov	fp, r0
	mov	r6, r1
	ldr	r5, [sp, #40]
	ldrb	r2, [r3, #585]	@ zero_extendqisi2
	mov	r10, r3
	ldr	r9, [sp, #44]
	cmp	r2, r4
	ldr	r8, [sp, #48]
	bhi	.L2710
	ldr	r1, .L2717+4
	mov	r2, #740
	ldr	r0, .L2717+8
	bl	printk
	bl	dump_stack
.L2710:
	ldrb	r2, [r10, #585]	@ zero_extendqisi2
	ldr	r3, .L2717
	cmp	r2, r4
	ldmlsfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
	add	r4, r3, r4
	ldrb	r10, [r4, #588]	@ zero_extendqisi2
	ldr	r4, [r3, #528]
	mov	r0, r10
	bl	nandc_cs
	cmp	fp, #0
	mov	r3, r10, asl #8
	addne	r2, r4, r3
	add	r4, r4, r3
	mov	r3, #128
	strne	fp, [r2, #2056]
	str	r6, [r4, #2056]
	str	r3, [r4, #2056]
	mov	r3, #0
	str	r3, [r4, #2052]
	str	r3, [r4, #2052]
	uxtb	r3, r5
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #16
	add	r5, r5, r5, asl #1
	str	r3, [r4, #2052]
	sub	r0, r5, #1
	add	r0, r0, r6
	bl	nandc_set_seed
	ldr	r3, .L2717+12
	mov	r0, #1
	mov	r2, r9
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	mov	r3, r8
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	str	r7, [r4, #2056]
	mov	r0, r10
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	nandc_de_cs
.L2718:
	.align	2
.L2717:
	.word	.LANCHOR0
	.word	.LANCHOR1+1684
	.word	.LC0
	.word	.LANCHOR2
	.fnend
	.size	flash_start_tlc_page_prog, .-flash_start_tlc_page_prog
	.align	2
	.type	queue_tlc_prog_cmd, %function
queue_tlc_prog_cmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #16
	mov	r7, #1
	ldr	r8, .L2721
	mov	r6, r0
	ldr	r3, [r0]
	ldrb	r4, [r8, #1700]	@ zero_extendqisi2
	ldr	r1, [r3, #24]
	rsb	r2, r4, #24
	mov	r4, r7, asl r4
	mov	r5, r7, asl r2
	sub	r4, r4, #1
	sub	r5, r5, #1
	ldrb	r0, [r3, #44]	@ zero_extendqisi2
	and	r5, r5, r1
	and	r4, r4, r1, lsr r2
	str	r5, [sp]
	mov	r1, r7
	ldr	r2, [r3, #4]
	uxtb	r4, r4
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldmia	r6, {r2, r3}
	mov	r1, #2
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r5, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r6, #8]
	ldr	r2, [r6]
	mov	r1, #3
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r5, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #16
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	ldr	r1, [r6]
	add	r0, r8, #3312
	mov	r3, #4
	add	r0, r0, #11
	strb	r3, [r1, #42]
	mvn	r3, #0
	strb	r7, [r1, #43]
	strb	r3, [r1]
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	b	buf_add_tail
.L2722:
	.align	2
.L2721:
	.word	.LANCHOR0
	.fnend
	.size	queue_tlc_prog_cmd, .-queue_tlc_prog_cmd
	.align	2
	.global	sblk_tlc_prog_one_page
	.type	sblk_tlc_prog_one_page, %function
sblk_tlc_prog_one_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r4, r0
	ldr	r3, [r0]
	ldr	r6, [r3, #24]
.L2724:
	mov	r0, r6
	mov	r1, #1
	bl	queue_lun_state
	subs	r5, r0, #0
	beq	.L2727
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2724
.L2727:
	mov	r0, r4
	bl	queue_tlc_prog_cmd
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
	.fnend
	.size	sblk_tlc_prog_one_page, .-sblk_tlc_prog_one_page
	.align	2
	.global	sblk_xlc_prog_pages
	.type	sblk_xlc_prog_pages, %function
sblk_xlc_prog_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #16
	mov	r5, r0
	ldr	r3, [r0]
	mov	r8, r1
	mov	r9, r2
	ldr	r4, [r3, #24]
.L2729:
	mov	r0, r4
	mov	r1, #1
	bl	queue_lun_state
	cmp	r0, #0
	beq	.L2734
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2729
.L2734:
	cmp	r9, #2
	bne	.L2731
	ldr	r2, .L2735
	mov	r6, #1
	ldr	r3, [r5]
	ldr	ip, [r8]
	ldrb	r4, [r2, #1700]	@ zero_extendqisi2
	ldr	r0, [r3, #24]
	rsb	r1, r4, #24
	ldr	r7, [ip, #24]
	mov	r2, r6, asl r1
	mov	r4, r6, asl r4
	sub	r2, r2, #1
	sub	r4, r4, #1
	and	r10, r2, r0
	and	r7, r7, r2
	and	r4, r4, r0, lsr r1
	ldrb	r0, [r3, #44]	@ zero_extendqisi2
	str	r10, [sp]
	mov	r1, r6
	ldr	r2, [r3, #4]
	uxtb	r4, r4
	str	r2, [sp, #4]
	mov	r2, #17
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r8]
	ldr	r2, [r5]
	mov	r1, r6
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r7, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldmia	r5, {r2, r3}
	mov	r1, r9
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r10, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #17
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r8, #4]
	ldr	r2, [r5]
	mov	r1, r9
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r7, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r5, #8]
	ldr	r2, [r5]
	mov	r1, #3
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r10, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #17
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r8, #8]
	ldr	r2, [r5]
	mov	r1, #3
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r7, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #16
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	ldr	r1, [r5]
	mov	r3, #5
	ldr	r0, .L2735+4
	strb	r3, [r1, #42]
	mvn	r3, #0
	strb	r6, [r1, #43]
	strb	r3, [r1]
	bl	buf_add_tail
	b	.L2732
.L2731:
	mov	r0, r5
	bl	queue_tlc_prog_cmd
.L2732:
	mov	r0, #0
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2736:
	.align	2
.L2735:
	.word	.LANCHOR0
	.word	.LANCHOR0+3323
	.fnend
	.size	sblk_xlc_prog_pages, .-sblk_xlc_prog_pages
	.align	2
	.global	sblk_3d_tlc_dump_prog
	.type	sblk_3d_tlc_dump_prog, %function
sblk_3d_tlc_dump_prog:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2739
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	mov	r7, #1
	ldrb	r4, [r3, #1700]	@ zero_extendqisi2
	mov	r5, r0
	ldr	r2, [r0, #24]
	.pad #20
	sub	sp, sp, #20
	rsb	r3, r4, #24
	mov	r4, r7, asl r4
	mov	r6, r7, asl r3
	sub	r4, r4, #1
	sub	r6, r6, #1
	and	r4, r4, r2, lsr r3
	and	r6, r6, r2
	mov	r3, r2
	ldr	r0, .L2739+4
	uxtb	r4, r4
	mov	r1, r6
	bl	printk
	ldr	r3, [r5, #4]
	mov	r1, r7
	mov	r2, #26
	mov	r0, #0
	str	r3, [sp, #4]
	ldr	r3, [r5, #12]
	str	r6, [sp]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r5, #4]
	mov	r1, #2
	mov	r2, #26
	mov	r0, #0
	str	r3, [sp, #4]
	ldr	r3, [r5, #12]
	str	r6, [sp]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r5, #4]
	mov	r1, #3
	mov	r2, #16
	mov	r0, #0
	str	r3, [sp, #4]
	ldr	r3, [r5, #12]
	str	r6, [sp]
	str	r3, [sp, #8]
	mov	r3, r4
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r0, [r5, #24]
	mov	r1, #64
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, lr}
	b	flash_wait_device_ready
.L2740:
	.align	2
.L2739:
	.word	.LANCHOR0
	.word	.LC175
	.fnend
	.size	sblk_3d_tlc_dump_prog, .-sblk_3d_tlc_dump_prog
	.align	2
	.global	flash_start_3d_mlc_page_prog
	.type	flash_start_3d_mlc_page_prog, %function
flash_start_3d_mlc_page_prog:
	.fnstart
	@ args = 4, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r8, r3
	ldr	r3, .L2745
	mov	r5, r2
	mov	r7, r0
	mov	r4, r1
	ldrb	r2, [r3, #585]	@ zero_extendqisi2
	mov	r6, r3
	cmp	r2, r1
	bhi	.L2742
	ldr	r1, .L2745+4
	mov	r2, #772
	ldr	r0, .L2745+8
	bl	printk
	bl	dump_stack
.L2742:
	ldrb	r2, [r6, #585]	@ zero_extendqisi2
	ldr	r3, .L2745
	cmp	r2, r4
	ldmlsfd	sp!, {r4, r5, r6, r7, r8, pc}
	add	r4, r3, r4
	ldr	r6, [r3, #528]
	ldrb	r4, [r4, #588]	@ zero_extendqisi2
	mov	r0, r4
	add	r4, r6, r4, asl #8
	bl	nandc_cs
	mov	r3, #128
	mov	r0, r5
	str	r3, [r4, #2056]
	mov	r3, #0
	str	r3, [r4, #2052]
	str	r3, [r4, #2052]
	uxtb	r3, r5
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #16
	str	r3, [r4, #2052]
	bl	nandc_set_seed
	ldr	r3, .L2745+12
	mov	r0, #1
	mov	r2, r8
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	ldr	r3, [sp, #24]
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	str	r7, [r4, #2056]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2746:
	.align	2
.L2745:
	.word	.LANCHOR0
	.word	.LANCHOR1+1712
	.word	.LC0
	.word	.LANCHOR2
	.fnend
	.size	flash_start_3d_mlc_page_prog, .-flash_start_3d_mlc_page_prog
	.align	2
	.global	sblk_3d_mlc_prog_pages
	.type	sblk_3d_mlc_prog_pages, %function
sblk_3d_mlc_prog_pages:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	add	r5, r0, #8
	ldr	r9, .L2755
	mov	r7, r1
	mov	r8, #1
.L2748:
	cmp	r7, #0
	beq	.L2753
	ldr	r3, [r5, #-8]
	ldr	r4, [r3, #24]
.L2749:
	mov	r0, r4
	mov	r1, #1
	bl	queue_lun_state
	cmp	r0, #0
	beq	.L2754
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2749
.L2754:
	ldr	r3, [r5, #-8]
	sub	r7, r7, #1
	ldrb	r4, [r9, #1700]	@ zero_extendqisi2
	add	r5, r5, #8
	ldr	r2, [r3, #24]
	rsb	r3, r4, #24
	mov	r4, r8, asl r4
	mov	r6, r8, asl r3
	sub	r4, r4, #1
	sub	r6, r6, #1
	and	r4, r4, r2, lsr r3
	and	r6, r6, r2
	uxtb	r4, r4
	mov	r0, r4
	bl	zftl_flash_exit_slc_mode
	ldr	r3, [r5, #-16]
	mov	r1, r4
	mov	r0, #16
	ldr	r2, [r3, #12]
	str	r2, [sp]
	mov	r2, r6
	ldr	r3, [r3, #4]
	bl	flash_start_3d_mlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r5, #-12]
	mov	r1, r4
	mov	r0, #16
	ldr	r2, [r3, #12]
	str	r2, [sp]
	add	r2, r6, #1
	ldr	r3, [r3, #4]
	bl	flash_start_3d_mlc_page_prog
	mov	r0, r4
	bl	nandc_de_cs
	ldr	r1, [r5, #-16]
	mov	r3, #4
	ldr	r0, .L2755+4
	strb	r3, [r1, #42]
	mvn	r3, #0
	strb	r8, [r1, #43]
	strb	r3, [r1]
	bl	buf_add_tail
	b	.L2748
.L2753:
	mov	r0, r7
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2756:
	.align	2
.L2755:
	.word	.LANCHOR0
	.word	.LANCHOR0+3323
	.fnend
	.size	sblk_3d_mlc_prog_pages, .-sblk_3d_mlc_prog_pages
	.align	2
	.global	sblk_mlc_dump_prog
	.type	sblk_mlc_dump_prog, %function
sblk_mlc_dump_prog:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2759
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #8
	mov	r4, #1
	ldrb	r1, [r3, #1700]	@ zero_extendqisi2
	mov	r5, r0
	ldr	r2, [r0, #24]
	rsb	r3, r1, #24
	mov	r6, r4, asl r3
	mov	r4, r4, asl r1
	sub	r4, r4, #1
	sub	r6, r6, #1
	and	r4, r4, r2, lsr r3
	and	r6, r6, r2
	uxtb	r4, r4
	mov	r0, r4
	bl	zftl_flash_exit_slc_mode
	ldr	r2, [r5, #24]
	mov	r1, r6
	ldr	r0, .L2759+4
	add	r3, r2, #1
	bl	printk
	ldr	r3, [r5, #12]
	mov	r1, r4
	mov	r2, r6
	mov	r0, #16
	str	r3, [sp]
	ldr	r3, [r5, #4]
	bl	flash_start_3d_mlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r5, #12]
	add	r2, r6, #1
	mov	r1, r4
	mov	r0, #16
	str	r3, [sp]
	ldr	r3, [r5, #4]
	bl	flash_start_3d_mlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	mov	r1, #64
	ldr	r0, [r5, #24]
	bl	flash_wait_device_ready
	mov	r5, r0
	mov	r0, r4
	bl	nandc_de_cs
	mov	r0, r5
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L2760:
	.align	2
.L2759:
	.word	.LANCHOR0
	.word	.LC176
	.fnend
	.size	sblk_mlc_dump_prog, .-sblk_mlc_dump_prog
	.align	2
	.global	flash_prog_page_en
	.type	flash_prog_page_en, %function
flash_prog_page_en:
	.fnstart
	@ args = 8, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	mov	r8, r3
	ldr	r6, .L2791
	ldrb	r3, [sp, #68]	@ zero_extendqisi2
	mov	r7, r0
	mov	r5, r1
	mov	r9, r2
	ubfx	r4, r1, #24, #2
	str	r3, [sp, #16]
	ldrb	r3, [r6, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L2762
	ldr	r1, .L2791+4
	movw	r2, #554
	ldr	r0, .L2791+8
	bl	printk
	bl	dump_stack
.L2762:
	ldrb	r2, [r6, #585]	@ zero_extendqisi2
	ldr	r3, .L2791
	cmp	r2, r7
	mvnls	r0, #0
	bls	.L2763
	add	r2, r3, r7
	cmp	r4, #0
	ldrb	r10, [r2, #588]	@ zero_extendqisi2
	bne	.L2775
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2765
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2775
.L2765:
	ldrh	fp, [r6, #2]
	mov	r0, r5
	mov	r1, fp
	bl	__aeabi_uidiv
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L2791
	mul	r0, fp, r0
	rsb	r4, r0, r5
	mov	r4, r4, asl #1
	addeq	r4, r3, r4
	ldreqh	r4, [r4, #4]
	add	r4, r4, r0
	b	.L2764
.L2775:
	mov	r4, r5
.L2764:
	ldr	fp, .L2791+12
	ldr	r6, .L2791+16
.L2770:
	ldr	r3, [fp]
	tst	r3, #16
	beq	.L2767
	ldr	r0, .L2791+20
	mov	r1, r10
	mov	r2, r5
	mov	r3, r4
	bl	printk
.L2767:
	ldr	r3, [sp, #64]
	mov	r0, r10
	mov	r1, r4
	mov	r2, r9
	str	r3, [sp]
	mov	r3, r8
	bl	flash_prog_page
	ldr	r3, [sp, #16]
	mov	ip, r0
	cmp	r3, #0
	beq	.L2768
	ldr	r3, [sp, #64]
	mov	r1, r5
	str	r0, [sp, #20]
	mov	r0, r7
	str	r3, [sp]
	ldr	r3, [r6, #-3260]
	ldr	r2, [r6, #-3264]
	bl	flash_read_page_en
	sub	r3, r0, #512
	cmp	r0, #512
	cmnne	r0, #1
	clz	r3, r3
	mov	r3, r3, lsr #5
	str	r3, [sp, #12]
	beq	.L2769
	ldr	r3, [r6, #-3264]
	ldr	r2, [r9]
	ldr	ip, [sp, #20]
	ldr	r3, [r3]
	cmp	r2, r3
	bne	.L2769
	ldr	r3, [r6, #-3260]
	ldr	r2, [r8]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L2768
.L2769:
	mov	r2, #4
	ldr	r0, .L2791+24
	mov	r3, r2
	mov	r1, r9
	bl	rknand_print_hex
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2791+28
	mov	r1, r8
	bl	rknand_print_hex
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2791+32
	ldr	r1, [r6, #-3260]
	bl	rknand_print_hex
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2791+36
	ldr	r1, [r6, #-3264]
	bl	rknand_print_hex
	ldr	r3, [sp, #12]
	cmp	r3, #0
	bne	.L2770
.L2772:
	mov	r1, r5
	ldr	r0, .L2791+40
	bl	printk
	ldr	r1, .L2791+4
	movw	r2, #587
	ldr	r0, .L2791+8
	bl	printk
	bl	dump_stack
	mvn	ip, #0
	b	.L2771
.L2768:
	cmn	ip, #1
	beq	.L2772
.L2771:
	mov	r0, ip
.L2763:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2792:
	.align	2
.L2791:
	.word	.LANCHOR0
	.word	.LANCHOR1+1744
	.word	.LC0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.word	.LC177
	.word	.LC178
	.word	.LC179
	.word	.LC180
	.word	.LC181
	.word	.LC182
	.fnend
	.size	flash_prog_page_en, .-flash_prog_page_en
	.align	2
	.global	ftl_prog_page
	.type	ftl_prog_page, %function
ftl_prog_page:
	.fnstart
	@ args = 4, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r4, r3
	mov	r7, r0
	mov	r5, r1
	mov	r6, r2
	bl	sblk_wait_write_queue_completed
	mov	r0, r7
	mov	r1, r5
	mov	r2, r6
	ldr	r3, [sp, #32]
	str	r3, [sp]
	mov	r3, #1
	str	r3, [sp, #4]
	mov	r3, r4
	bl	flash_prog_page_en
	cmn	r0, #1
	mov	r4, r0
	bne	.L2794
	ldr	r1, .L2796
	movw	r2, #2324
	ldr	r0, .L2796+4
	bl	printk
	bl	dump_stack
	ldr	r0, .L2796+8
	mov	r1, r5
	bl	printk
.L2794:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2797:
	.align	2
.L2796:
	.word	.LANCHOR1+1764
	.word	.LC0
	.word	.LC182
	.fnend
	.size	ftl_prog_page, .-ftl_prog_page
	.align	2
	.global	ftl_info_flush
	.type	ftl_info_flush, %function
ftl_info_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r1, #0
	ldr	r5, .L2836
	.pad #20
	sub	sp, sp, #20
	mov	fp, r0
	ldr	r0, .L2836+4
	ldr	r7, .L2836+8
	mov	r10, #0
	ldrb	r2, [r5, #3320]	@ zero_extendqisi2
	mov	r6, r7
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r3, [r5, #3348]
	ldrh	r2, [r3, #74]
	cmp	r2, #0
	movne	r2, #0
	strneh	r2, [r3, #150]	@ movhi
.L2810:
	ldr	r3, .L2836+12
	ldr	lr, [r5, #3348]
	ldr	r8, .L2836+16
	ldrh	r3, [r3]
	ldrb	r4, [r7, #-2936]	@ zero_extendqisi2
	ldrh	ip, [r8, #2]
	str	r3, [sp, #8]
	ldr	r3, [lr, #4]
	ldr	r9, .L2836+12
	add	r3, r3, #1
	str	r3, [lr, #4]
	str	r3, [r7, #-3188]
	ldr	r3, .L2836+20
	str	fp, [r7, #-3192]
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L2800
	mov	r2, ip
	ldr	r0, .L2836+24
	mov	r1, r4
	ldr	r3, [lr, #4]
	str	ip, [sp, #12]
	bl	printk
	ldr	ip, [sp, #12]
.L2800:
	ldr	r3, .L2836+28
	ldrh	r0, [r8, #2]
	ldrh	r3, [r3]
	cmp	r3, r0
	bhi	.L2801
.L2806:
	ldrb	r3, [r6, #-2935]	@ zero_extendqisi2
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r6, #-2935]
	cmp	r3, #7
	bls	.L2802
	mov	r3, #8
.L2805:
	ldr	r2, [r5, #1696]
	sub	r4, r3, #8
	add	r2, r2, r3
	uxth	r4, r4
	ldrb	r1, [r2, #32]	@ zero_extendqisi2
	add	r2, r1, #127
	uxtb	r2, r2
	cmp	r2, #125
	bhi	.L2803
	ldr	r1, .L2836+32
	movw	r2, #751
	ldr	r0, .L2836+36
	bl	printk
	bl	dump_stack
	b	.L2804
.L2803:
	cmp	r1, #255
	bne	.L2804
	add	r3, r3, #1
	cmp	r3, #16
	bne	.L2805
	mov	r4, #8
.L2804:
	mov	r10, #1
	strb	r4, [r6, #-2935]
.L2802:
	ldr	r3, [r5, #1696]
	ldrb	r2, [r6, #-2935]	@ zero_extendqisi2
	add	r3, r3, r2
	ldrb	r3, [r3, #40]	@ zero_extendqisi2
	cmp	r3, #255
	strb	r3, [r6, #-2936]
	beq	.L2806
	ldrh	r4, [r9]
	mov	r0, #0
	mul	r4, r4, r3
	mov	r1, r4
	bl	flash_erase_block
	ldr	r3, .L2836
	ldr	r2, .L2836+40
	mov	r1, r4
	mov	r0, #0
	ldrb	r3, [r3, #3320]	@ zero_extendqisi2
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_prog_page
	ldr	r3, .L2836+16
	mov	r2, #1
	add	r4, r4, r2
	strh	r2, [r3, #2]	@ movhi
	b	.L2807
.L2801:
	ldr	r3, [sp, #8]
	cmp	r0, #0
	mla	r4, r3, r4, ip
	bne	.L2807
	mov	r1, r4
	bl	flash_erase_block
.L2807:
	ldrb	r3, [r5, #3320]	@ zero_extendqisi2
	mov	r0, #0
	ldr	r2, .L2836+40
	mov	r1, r4
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_prog_page
	ldrh	r3, [r8, #2]
	add	r3, r3, #1
	strh	r3, [r8, #2]	@ movhi
	ldr	r3, .L2836+44
	cmn	r0, #1
	beq	.L2808
	ldrb	r2, [r3, #-2856]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2809
.L2808:
	mov	r2, #0
	strb	r2, [r3, #-2856]
	b	.L2810
.L2809:
	cmp	r10, #1
	beq	.L2811
.L2819:
	ldr	r3, .L2836+8
	ldrb	r3, [r3, #-2936]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L2813
	ldr	r1, .L2836+32
	movw	r2, #787
	ldr	r0, .L2836+36
	bl	printk
	bl	dump_stack
	b	.L2813
.L2811:
	ldr	r3, .L2836+8
	ldrb	r4, [r3, #-2935]	@ zero_extendqisi2
	add	r4, r4, #1
.L2814:
	cmp	r4, #7
	bhi	.L2819
	ldr	r3, [r5, #1696]
	add	r3, r3, r4
	ldrb	r6, [r3, #40]	@ zero_extendqisi2
	add	r3, r6, #127
	uxtb	r3, r3
	cmp	r3, #125
	bhi	.L2815
	ldr	r1, .L2836+32
	mov	r2, #780
	ldr	r0, .L2836+36
	bl	printk
	bl	dump_stack
	b	.L2816
.L2815:
	cmp	r6, #255
	beq	.L2817
.L2816:
	ldrh	r1, [r9]
	mov	r0, #0
	mul	r1, r1, r6
	bl	flash_erase_block
.L2817:
	add	r4, r4, #1
	uxth	r4, r4
	b	.L2814
.L2813:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2837:
	.align	2
.L2836:
	.word	.LANCHOR0
	.word	.LANCHOR3-3192
	.word	.LANCHOR3
	.word	.LANCHOR0+4076
	.word	.LANCHOR3-2936
	.word	.LANCHOR2
	.word	.LC183
	.word	.LANCHOR0+3976
	.word	.LANCHOR1+1780
	.word	.LC0
	.word	.LANCHOR3-2872
	.word	.LANCHOR4
	.fnend
	.size	ftl_info_flush, .-ftl_info_flush
	.align	2
	.global	ftl_info_blk_init
	.type	ftl_info_blk_init, %function
ftl_info_blk_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2870
	mov	r2, #1
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r7, #0
	ldr	r4, .L2870+4
	mov	r1, #16384
	strb	r2, [r3, #-2856]
	.pad #20
	sub	sp, sp, #20
	strb	r2, [r3, #-2854]
	mov	r6, #7
	strb	r7, [r3, #-2855]
	add	r3, r4, #564
	ldr	r0, .L2870+8
	mov	r9, r4
	ldrh	r3, [r3]
	ldr	r5, .L2870+12
	str	r0, [r4, #568]
	add	r3, r0, r3, asl #2
	str	r3, [r4, #3348]
	bl	__memzero
	ldr	r0, .L2870+16
	mov	r1, #16384
	ldr	r10, .L2870+20
	bl	__memzero
	ldr	r3, [r4, #1696]
	ldr	fp, .L2870+24
	strb	r7, [r5, #-2935]
	ldrb	r3, [r3, #40]	@ zero_extendqisi2
	strb	r3, [r5, #-2936]
	ldr	r3, .L2870+28
	strh	r7, [r3, #2]	@ movhi
	str	r3, [sp, #8]
.L2843:
	ldr	r3, [r4, #1696]
	uxth	r7, r6
	add	r3, r3, r6
	ldrb	r3, [r3, #40]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2839
	ldrh	ip, [r10]
	mov	r0, #0
	ldr	r2, .L2870+8
	mul	ip, ip, r3
	ldrb	r3, [r9, #3320]	@ zero_extendqisi2
	str	r3, [sp]
	sub	r3, r2, #320
	mov	r1, ip
	str	ip, [sp, #12]
	bl	ftl_read_page
	cmn	r0, #1
	mov	r8, r0
	ldr	ip, [sp, #12]
	bne	.L2840
	ldrb	r3, [r9, #3320]	@ zero_extendqisi2
	mov	r0, #0
	ldr	r2, .L2870+8
	add	r1, ip, #1
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_read_page
	mov	r8, r0
.L2840:
	ldr	r3, .L2870+32
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L2841
	ldr	r3, [r4, #3348]
	mov	r1, r6
	ldr	r0, .L2870+36
	mov	r2, r8
	ldr	r3, [r3]
	str	r3, [sp]
	mov	r3, #672
	bl	printk
.L2841:
	cmn	r8, #1
	beq	.L2839
	ldr	r3, [r4, #3348]
	ldr	r3, [r3]
	cmp	r3, fp
	beq	.L2852
.L2839:
	subs	r6, r6, #1
	bcs	.L2843
	movw	r8, #65535
	mov	r7, #0
	b	.L2842
.L2852:
	mov	r8, r7
.L2842:
	ldr	r6, .L2870+32
	sxth	r8, r8
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2844
	ldr	r3, [r4, #3348]
	mov	r1, r8
	ldr	r0, .L2870+40
	mov	r2, #4800
	ldr	r3, [r3]
	bl	printk
.L2844:
	cmn	r8, #1
	bne	.L2845
	ldr	r0, .L2870+8
	mov	r1, #16384
	bl	__memzero
	ldr	r3, [r4, #3348]
	ldr	r2, .L2870+24
	mov	r0, r8
	str	r2, [r3]
	ldr	r2, .L2870+44
	ldr	r3, [r4, #3348]
	str	r2, [r3, #12]
	b	.L2846
.L2845:
	ldr	r3, [r4, #1696]
	mov	r0, #0
	ldr	r2, .L2870+8
	sxtah	r3, r3, r7
	strb	r7, [r5, #-2935]
	ldr	r10, .L2870+4
	ldrb	r1, [r3, #40]	@ zero_extendqisi2
	mov	r3, #4
	ldr	fp, .L2870+24
	strb	r1, [r5, #-2936]
	str	r3, [sp]
	sub	r3, r2, #320
	bl	flash_get_last_written_page
	movw	r2, #4076
	ldrb	r3, [r5, #-2936]	@ zero_extendqisi2
	mov	r5, #0
	ldrh	r7, [r4, r2]
	mul	r7, r7, r3
	uxth	r8, r0
	mov	r9, r0
	add	ip, r8, #1
	uxth	ip, ip
.L2847:
	rsb	r3, r5, r8
	tst	r3, #32768
	bne	.L2851
	ldrb	r3, [r4, #3320]	@ zero_extendqisi2
	add	r1, r9, r7
	ldr	r2, .L2870+8
	mov	r0, #0
	rsb	r1, r5, r1
	str	ip, [sp, #12]
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_read_page
	cmn	r0, #1
	ldr	ip, [sp, #12]
	beq	.L2848
	ldr	r3, [r10, #3348]
	ldr	r3, [r3]
	cmp	r3, fp
	bne	.L2848
.L2851:
	ldr	r3, [sp, #8]
	strh	ip, [r3, #2]	@ movhi
	bl	ftl_tmp_into_update
	ldr	r2, [r4, #3348]
	mov	r0, #0
	ldr	r3, [r2, #64]
	add	r3, r3, #1
	str	r3, [r2, #64]
	bl	ftl_info_flush
	mov	r0, #0
	bl	ftl_info_flush
	ldr	r0, [r6]
	ands	r0, r0, #16384
	beq	.L2846
	ldr	r3, .L2870+4
	ldr	r0, .L2870+48
	ldr	r3, [r3, #3348]
	ldr	r1, [r3, #156]
	bl	printk
	mov	r0, #0
	b	.L2846
.L2848:
	add	r5, r5, #1
	b	.L2847
.L2846:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2871:
	.align	2
.L2870:
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LANCHOR3-2872
	.word	.LANCHOR3
	.word	.LANCHOR4-2792
	.word	.LANCHOR0+4076
	.word	1229739078
	.word	.LANCHOR3-2936
	.word	.LANCHOR2
	.word	.LC184
	.word	.LC185
	.word	393240
	.word	.LC186
	.fnend
	.size	ftl_info_blk_init, .-ftl_info_blk_init
	.align	2
	.global	ftl_ext_info_flush
	.type	ftl_ext_info_flush, %function
ftl_ext_info_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #8
	bl	timer_get_time
	ldr	r3, .L2888
	ldr	r4, .L2888+4
	umull	r0, r1, r0, r3
	ldr	r3, [r4, #580]
	ldr	r0, [r3, #520]
	mov	r1, r1, lsr #5
	cmp	r1, r0
	ldrhi	r2, [r3, #12]
	rsbhi	r2, r0, r2
	addhi	r2, r2, r1
	strhi	r2, [r3, #12]
	bhi	.L2887
.L2873:
	bcs	.L2874
.L2887:
	str	r1, [r3, #520]
.L2874:
	mov	r0, #0
	ldr	r5, .L2888+8
	bl	ftl_total_vpn_update
	ldr	r7, .L2888+12
	mov	r6, r5
.L2878:
	ldr	r3, [r4, #3348]
	ldr	r2, [r3, #56]
	add	r2, r2, #1
	str	r2, [r3, #56]
	ldrh	r2, [r3, #140]
	ldrh	r3, [r7]
	cmp	r2, r3
	bcc	.L2876
	bl	ftl_ext_alloc_new_blk
.L2876:
	ldr	r3, [r4, #3348]
	ldrh	r2, [r3, #130]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2877
	ldr	r1, .L2888+16
	movw	r2, #1912
	ldr	r0, .L2888+20
	bl	printk
	bl	dump_stack
.L2877:
	ldr	r1, [r4, #3348]
	ldr	r8, .L2888+24
	ldrb	r2, [r4, #1700]	@ zero_extendqisi2
	ldrh	r3, [r1, #130]
	ldrh	r0, [r8]
	rsb	r2, r2, #24
	ldrh	r8, [r8, #48]
	rsb	r2, r0, r2
	mvn	r0, #0
	mov	r9, r3, asr r2
	bic	r3, r3, r0, asl r2
	ldrh	r2, [r1, #140]
	mov	r1, #0
	sxth	r3, r3
	ldr	r0, .L2888+28
	ldr	r10, .L2888+32
	mla	r8, r8, r3, r2
	ldrb	r2, [r4, #3320]	@ zero_extendqisi2
	mov	r2, r2, asl #1
	bl	ftl_memset
	mov	r3, #0
	str	r3, [r10, #-3192]
	uxtb	r0, r9
	ldr	r3, [r4, #3348]
	mov	r1, r8
	ldr	r2, .L2888+36
	ldr	r3, [r3, #56]
	str	r3, [r10, #-3188]
	ldrb	r3, [r4, #3320]	@ zero_extendqisi2
	str	r3, [sp]
	ldr	r3, .L2888+28
	bl	ftl_prog_page
	ldr	r2, [r4, #3348]
	ldrh	r3, [r2, #140]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r2, #140]	@ movhi
	cmp	r3, #1
	beq	.L2878
	cmn	r0, #1
	beq	.L2879
	ldrb	r0, [r5, #-2854]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L2880
.L2879:
.L2875:
	mov	r3, #0
	strb	r3, [r6, #-2854]
	b	.L2878
.L2880:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2889:
	.align	2
.L2888:
	.word	1374389535
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR0+3976
	.word	.LANCHOR1+1796
	.word	.LC0
	.word	.LANCHOR0+4028
	.word	.LANCHOR3-3192
	.word	.LANCHOR3
	.word	.LANCHOR4-2792
	.fnend
	.size	ftl_ext_info_flush, .-ftl_ext_info_flush
	.align	2
	.global	ftl_ext_info_init
	.type	ftl_ext_info_init, %function
ftl_ext_info_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r2, #0
	ldr	r4, .L2909
	movw	r0, #4028
	ldr	r3, .L2909+4
	mvn	r5, #0
	.pad #20
	sub	sp, sp, #20
	ldrh	r0, [r4, r0]
	strh	r2, [r3]	@ movhi
	add	r3, r3, #392
	ldr	r2, [r4, #3348]
	ldrh	r1, [r2, #130]
	ldrb	r2, [r4, #1700]	@ zero_extendqisi2
	rsb	r2, r2, #24
	rsb	r2, r0, r2
	mov	r6, r1, asr r2
	bic	r5, r1, r5, asl r2
	mov	r2, #4
	str	r2, [sp]
	uxtb	r9, r6
	ldr	r2, .L2909+8
	mov	r1, r5
	mov	r0, r9
	bl	flash_get_last_written_page
	ldr	r3, .L2909+12
	ldr	r2, [r3]
	tst	r2, #4096
	mov	r7, r0
	str	r3, [sp, #8]
	beq	.L2891
	ldr	r3, [r4, #3348]
	uxth	r6, r6
	str	r6, [sp]
	movw	r2, #1956
	ldr	r0, .L2909+16
	ldrh	r3, [r3, #130]
	ldr	r1, .L2909+20
	str	r3, [sp, #4]
	mov	r3, r7
	bl	printk
.L2891:
	ldr	r10, .L2909
	mov	r8, #0
	ldr	fp, .L2909+24
	ldr	ip, .L2909+28
.L2892:
	uxth	r6, r7
	rsb	r3, r8, r6
	tst	r3, #32768
	bne	.L2897
	ldrh	r3, [fp]
	rsb	r1, r8, r7
	ldrb	r2, [r4, #3320]	@ zero_extendqisi2
	mov	r0, r9
	str	ip, [sp, #12]
	mla	r1, r3, r5, r1
	str	r2, [sp]
	ldr	r3, .L2909+32
	ldr	r2, .L2909+8
	bl	flash_read_page_en
	cmp	r0, #512
	cmnne	r0, #1
	ldr	ip, [sp, #12]
	beq	.L2893
	ldr	r3, [r10, #580]
	ldr	r3, [r3]
	cmp	r3, ip
	bne	.L2893
.L2897:
	bl	zftl_sblk_list_init
	ldr	r3, [r4, #3348]
	ldrh	r2, [r3, #140]
	cmp	r2, r7
	bgt	.L2895
	add	r6, r6, #1
	strh	r6, [r3, #140]	@ movhi
	bl	ftl_ext_info_flush
	b	.L2895
.L2893:
	add	r8, r8, #1
	b	.L2892
.L2895:
	ldr	r5, [r4, #580]
	bl	timer_get_time
	ldr	r3, .L2909+36
	umull	r0, r1, r0, r3
	mov	r3, r1, lsr #5
	str	r3, [r5, #520]
	ldr	r5, [r4, #580]
	bl	timer_get_time
	ldr	r3, [r4, #580]
	mov	r2, #0
	add	r1, r3, #584
	str	r0, [r5, #604]
	str	r2, [r3, #608]
	mvn	r2, #0
	strh	r2, [r1]	@ movhi
	movw	r1, #586
	strh	r2, [r3, r1]	@ movhi
	add	r1, r3, #588
	strh	r2, [r1]	@ movhi
	movw	r1, #590
	strh	r2, [r3, r1]	@ movhi
	movw	r1, #65535
	str	r2, [r3, #564]
	ldr	r2, [sp, #8]
	str	r1, [r3, #560]
	ldr	r2, [r2]
	tst	r2, #4096
	beq	.L2898
	ldr	r6, [r3, #520]
	ldr	r5, [r3, #12]
	bl	timer_get_time
	ldr	r1, .L2909+20
	mov	r2, r6
	mov	r3, r5
	str	r0, [sp]
	ldr	r0, .L2909+40
	bl	printk
.L2898:
	movw	r3, #4010
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r4, r3]	@ movhi
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2910:
	.align	2
.L2909:
	.word	.LANCHOR0
	.word	.LANCHOR3-3584
	.word	.LANCHOR4-2792
	.word	.LANCHOR2
	.word	.LC187
	.word	.LANCHOR1+1816
	.word	.LANCHOR0+4076
	.word	1162432070
	.word	.LANCHOR3-3192
	.word	1374389535
	.word	.LC188
	.fnend
	.size	ftl_ext_info_init, .-ftl_ext_info_init
	.align	2
	.global	ftl_prog_ppa_page
	.type	ftl_prog_ppa_page, %function
ftl_prog_ppa_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, lr}
	.save {r4, lr}
	.pad #8
	mov	lr, r2
	ldr	r2, .L2913
	mov	ip, r0
	mov	r4, r1
	ldrb	r0, [r2, #1700]	@ zero_extendqisi2
	mov	r2, #1
	str	r3, [sp]
	mov	r3, lr
	rsb	r1, r0, #24
	mov	r0, r2, asl r0
	sub	r0, r0, #1
	and	r0, r0, ip, lsr r1
	mov	r1, r2, asl r1
	sub	r1, r1, #1
	mov	r2, r4
	and	r1, r1, ip
	uxtb	r0, r0
	bl	ftl_prog_page
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, pc}
.L2914:
	.align	2
.L2913:
	.word	.LANCHOR0
	.fnend
	.size	ftl_prog_ppa_page, .-ftl_prog_ppa_page
	.align	2
	.global	ftl_write_last_log_page
	.type	ftl_write_last_log_page, %function
ftl_write_last_log_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2920
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r4, r0
	ldr	r5, [r3, #-3592]
	ldrh	r3, [r0, #6]
	ldrh	r6, [r0, #12]
	cmp	r3, #1
	bne	.L2917
	bl	ftl_get_new_free_page
	cmn	r0, #1
	mov	r8, r0
	beq	.L2918
	ldrh	r0, [r4]
	add	r5, r5, r6, asl #2
	bl	ftl_vpn_decrement
	ldr	r6, .L2920+4
	ldr	r3, .L2920+8
	mov	r0, r5
	ldr	r7, .L2920+12
	mov	r4, #0
	str	r3, [r6, #-2776]
	movw	r3, #3976
	ldrh	r1, [r7, r3]
	ldrb	r3, [r7, #3984]	@ zero_extendqisi2
	mul	r1, r3, r1
	mov	r1, r1, asl #2
	bl	js_hash
	mov	r1, r5
	ldr	r2, .L2920+16
	ldrb	r3, [r7, #3320]	@ zero_extendqisi2
	str	r4, [r6, #-2768]
	str	r4, [r6, #-2764]
	str	r0, [r6, #-2772]
	mov	r0, r8
	bl	ftl_prog_ppa_page
	b	.L2918
.L2917:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2918:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2921:
	.align	2
.L2920:
	.word	.LANCHOR3
	.word	.LANCHOR5
	.word	-178307901
	.word	.LANCHOR0
	.word	.LANCHOR5-2776
	.fnend
	.size	ftl_write_last_log_page, .-ftl_write_last_log_page
	.align	2
	.global	ftl_open_sblk_recovery
	.type	ftl_open_sblk_recovery, %function
ftl_open_sblk_recovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 216
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #228
	sub	sp, sp, #228
	ldr	r6, .L3052
	mov	r4, r0
	str	r1, [sp, #20]
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2923
	ldr	r0, .L3052+4
	ldrh	r1, [r4, #2]
	bl	printk
.L2923:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2924
	ldr	r0, .L3052+8
	ldrb	r1, [r4, #5]	@ zero_extendqisi2
	bl	printk
.L2924:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2925
	ldr	r0, .L3052+12
	ldrh	r1, [r4]
	bl	printk
.L2925:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2926
	ldr	r0, .L3052+16
	ldrh	r1, [r4, #16]
	ldrh	r2, [r4, #18]
	bl	printk
.L2926:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2927
	ldr	r0, .L3052+20
	ldrb	r1, [r4, #9]	@ zero_extendqisi2
	bl	printk
.L2927:
	ldrh	r3, [r4, #10]
	ldr	r7, .L3052+24
	ldrh	r2, [r4]
	strh	r3, [r4, #14]	@ movhi
	ldrh	r3, [r7]
	cmp	r2, r3
	bcs	.L2922
	mov	r0, #1
	sub	r7, r7, #564
	bl	buf_alloc
	mov	r1, #255
	mov	r2, #64
	mov	r10, #2
	mov	r5, r0
	add	r0, sp, #32
	bl	memset
	mov	r1, #255
	mov	r2, #64
	add	r0, sp, #96
	bl	memset
	mov	r1, #255
	mov	r2, #64
	add	r0, sp, #160
	bl	memset
	ldrb	r8, [r4, #5]	@ zero_extendqisi2
	ldrh	r9, [r4, #2]
	mov	r3, #0
	str	r3, [sp, #12]
.L2929:
	ldr	r3, .L3052+28
	ldrh	r2, [r3]
	cmp	r2, r9
	bls	.L2932
	ldrb	r8, [r4, #5]	@ zero_extendqisi2
	add	r3, r3, #100
	str	r3, [sp, #24]
.L2930:
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	cmp	r3, r8
	bls	.L3048
	add	r3, r4, r8, asl #1
	movw	r2, #65535
	ldrh	r3, [r3, #16]
	cmp	r3, r2
	beq	.L2931
	ldr	r2, [sp, #24]
	ldr	r1, [r5, #4]
	ldrh	r2, [r2]
	mla	r3, r2, r3, r9
	ldr	r2, [r5, #12]
	str	r3, [sp, #16]
	mov	r0, r3
	ldrb	r3, [r7, #3320]	@ zero_extendqisi2
	bl	ftl_read_ppa_page
	cmp	r0, #512
	mov	fp, r0
	beq	.L2932
	cmn	r0, #1
	beq	.L2933
	ldr	r3, [r5, #12]
	ldr	r2, [r3]
	cmn	r2, #1
	bne	.L2933
	ldr	r3, [r3, #4]
	cmn	r3, #1
	bne	.L2933
	ldr	r3, [r5, #4]
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L2932
.L2933:
	ldr	r3, .L3052+32
	mov	r2, #1
	strb	r2, [r3, #-2855]
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	ldrh	r2, [r4, #10]
	mla	r3, r3, r9, r8
	cmp	r2, r3
	beq	.L2934
	ldr	r1, .L3052+36
	mov	r2, #1488
	ldr	r0, .L3052+40
	bl	printk
	bl	dump_stack
.L2934:
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #6]
	add	r2, r2, r3
	ldr	r3, .L3052+28
	ldrh	r1, [r3]
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	mul	r3, r3, r1
	cmp	r2, r3
	beq	.L2935
	ldr	r1, .L3052+36
	movw	r2, #1489
	ldr	r0, .L3052+40
	bl	printk
	bl	dump_stack
.L2935:
	cmn	fp, #1
	beq	.L2939
	ldr	r3, [r5, #12]
	ldr	r0, [r3, #4]
	cmn	r0, #1
	beq	.L2939
	bl	lpa_hash_get_ppa
	ldr	r2, [sp, #20]
	mov	r3, r0
	cmp	r2, #0
	beq	.L2937
	ldr	r2, [r5, #12]
	ldr	r1, [r2, #8]
	str	r2, [sp, #28]
	cmp	r0, r1
	cmnne	r0, #1
	beq	.L2937
	ldr	r2, .L3052+44
	mov	r10, #1
	ldr	fp, .L3052+48
	str	r3, [sp, #12]
	ldrb	r1, [r2, #1700]	@ zero_extendqisi2
	ldrh	r0, [fp]
	rsb	r1, r1, #24
	rsb	r1, r0, r1
	mov	r1, r10, asl r1
	sub	r1, r1, #1
	and	r0, r1, r3, lsr r0
	ldrb	r1, [r2, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r3, [sp, #20]
	uxth	r0, r0
	ldr	r2, [sp, #28]
	ldrh	r1, [r3]
	ldr	r3, [sp, #12]
	cmp	r1, r0
	bne	.L2937
	ldr	ip, [r2]
	mov	r0, r3
	ldr	r1, [r5, #4]
	ldrb	r3, [r7, #3320]	@ zero_extendqisi2
	ldr	r2, .L3052+52
	str	ip, [sp, #12]
	bl	ftl_read_ppa_page
	ldr	r3, .L3052+56
	ldr	r3, [r3, #-3192]
	ldr	ip, [sp, #12]
	cmp	r3, ip
	bcc	.L2937
	ldr	r3, [r5, #12]
	ldr	r0, [r3, #8]
	cmn	r0, #1
	beq	.L2939
	ldrb	r3, [r7, #1700]	@ zero_extendqisi2
	ldrh	r2, [fp]
	rsb	r3, r3, #24
	ldrb	r1, [r7, #4030]	@ zero_extendqisi2
	rsb	r3, r2, r3
	mov	r10, r10, asl r3
	sub	r10, r10, #1
	and	r0, r10, r0, lsr r2
	bl	__aeabi_uidiv
	uxth	r0, r0
	bl	ftl_vpn_decrement
	b	.L2939
.L2937:
	ldr	r2, [r5, #12]
	ldr	r3, [r7, #3328]
	ldr	r1, [r2, #4]
	cmp	r1, r3
	bcs	.L2939
	ldr	r3, .L3052+28
	ldrb	r0, [r4, #9]	@ zero_extendqisi2
	ldrh	r1, [r4, #10]
	ldrh	r3, [r3]
	mul	r3, r3, r0
	sub	r3, r3, #1
	cmp	r1, r3
	bge	.L2939
	ldr	r3, [r7, #3348]
	ldr	r2, [r2]
	ldr	r1, [r3, #8]
	cmp	r2, r1
	strhi	r2, [r3, #8]
	ldrh	r3, [r4, #12]
	ldrh	r2, [r4, #10]
	ldr	r1, [r5, #12]
	add	r2, r3, r2
	ldmib	r1, {r0, r1}
	uxth	r2, r2
	bl	lpa_hash_update_ppa
	ldr	r3, [sp, #36]
	str	r3, [sp, #32]
	ldr	r3, [sp, #40]
	str	r3, [sp, #36]
	ldr	r3, [sp, #44]
	str	r3, [sp, #40]
	ldr	r3, [sp, #16]
	str	r3, [sp, #44]
.L2939:
	ldrh	r3, [r4, #6]
	mov	r10, #4
	sub	r3, r3, #1
	strh	r3, [r4, #6]	@ movhi
	ldrh	r3, [r4, #10]
	add	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
	mov	r3, #1
	str	r3, [sp, #12]
.L2931:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L2930
.L3048:
	add	r9, r9, #1
	mov	r3, #0
	strb	r3, [r4, #5]
	uxth	r9, r9
	b	.L2929
.L2932:
	ldrh	r3, [r4, #6]
	ldrh	r2, [r4, #10]
	ldr	r7, .L3052+44
	add	r2, r3, r2
	movw	r3, #3976
	ldrb	r1, [r4, #9]	@ zero_extendqisi2
	ldrh	r3, [r7, r3]
	strh	r9, [r4, #2]	@ movhi
	strb	r8, [r4, #5]
	mul	r3, r3, r1
	cmp	r2, r3
	beq	.L2943
	ldr	r1, .L3052+36
	movw	r2, #1569
	ldr	r0, .L3052+40
	bl	printk
	bl	dump_stack
.L2943:
	ldr	r3, .L3052+56
	mov	ip, #0
	ldrh	r8, [r4, #10]
	mov	fp, ip
	ldr	r2, [r3, #-3592]
	str	r3, [sp, #16]
.L2944:
	cmp	fp, r8
	bcs	.L3049
	ldrh	r3, [r4, #12]
	add	r3, fp, r3
	add	fp, fp, #1
	ldr	r3, [r2, r3, asl #2]
	cmn	r3, #1
	addne	ip, ip, #1
	uxthne	ip, ip
	b	.L2944
.L3049:
	movw	r3, #3976
	ldrb	r2, [r4, #9]	@ zero_extendqisi2
	ldrh	r3, [r7, r3]
	smulbb	r3, r2, r3
	rsb	r8, r8, r3
	ldr	r3, [r6]
	add	r8, ip, r8
	tst	r3, #4096
	uxth	r8, r8
	beq	.L2947
	ldr	r3, .L3052+44
	mov	r2, r8
	ldrh	r1, [r4]
	ldr	r0, .L3052+60
	ldr	ip, [r3, #576]
	mov	r3, r1, asl #1
	ldrh	r3, [ip, r3]
	bl	printk
.L2947:
	ldrh	r3, [r4]
	ldr	r2, [r7, #576]
	mov	r3, r3, asl #1
	strh	r8, [r2, r3]	@ movhi
	ldr	r3, [r6]
	tst	r3, #16384
	beq	.L2948
	ldr	r3, [sp, #44]
	add	r1, sp, #32
	ldr	r0, .L3052+64
	str	r3, [sp]
	ldmia	r1, {r1, r2, r3}
	bl	printk
.L2948:
	ldrb	r2, [r7, #3320]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, [r5, #4]
	mov	r9, #0
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2949:
	ldrb	r3, [r7, #3984]	@ zero_extendqisi2
	mul	r3, r3, r10
	cmp	r9, r3
	bcs	.L3050
	add	ip, sp, #32
	ldr	r0, [ip, r9, asl #2]
	cmn	r0, #1
	bne	.L2950
.L2954:
	ldr	r2, [r7, #3348]
	mvn	r8, #0
	ldr	r3, [r5, #12]
	ldr	r2, [r2, #8]
	str	r2, [r3]
	ldr	r3, [r5, #12]
	str	r8, [r3, #4]
	ldr	r3, [r5, #12]
	str	r8, [r3, #8]
	mov	r3, #0
	ldr	r2, [r5, #12]
	str	r3, [r2, #12]
	ldr	r2, [r5, #4]
	str	r3, [r2]
	b	.L2951
.L2950:
	ldr	r3, .L3052+44
	ldr	r1, [r5, #4]
	ldr	r2, [r5, #12]
	ldrb	r3, [r3, #3320]	@ zero_extendqisi2
	str	ip, [sp, #20]
	bl	ftl_read_ppa_page
	ldr	r3, [r5, #12]
	mov	r8, r0
	ldr	r0, [r3, #4]
	bl	lpa_hash_get_ppa
	ldr	r3, [r6]
	tst	r3, #16384
	mov	fp, r0
	ldr	ip, [sp, #20]
	beq	.L2952
	ldr	r3, [r5, #12]
	mov	r1, fp
	ldr	r2, [ip, r9, asl #2]
	ldr	r0, .L3052+68
	ldr	r3, [r3, #4]
	bl	printk
	ldr	ip, [sp, #20]
.L2952:
	ldr	r3, [ip, r9, asl #2]
	mov	r2, #1
	cmp	fp, r3
	ldr	r3, [r5, #12]
	mvnne	r8, #0
	cmn	r8, #1
	str	fp, [r3, #8]
	ldr	r3, [r5, #12]
	str	r2, [r3, #12]
	beq	.L2954
.L2951:
	ldr	r3, [sp, #12]
	cmp	r3, #1
	bne	.L2956
	ldrh	r3, [r4, #6]
	cmp	r3, #1
	bls	.L2956
.L3021:
	mov	r0, r4
	bl	ftl_get_new_free_page
	ldr	r3, [r6]
	tst	r3, #16384
	mov	fp, r0
	beq	.L2958
	ldrh	r1, [r4, #12]
	ldr	r2, [r5, #12]
	ldrh	r3, [r4, #10]
	ldr	r0, .L3052+72
	add	r3, r3, r1
	ldr	r2, [r2, #4]
	mov	r1, fp
	sub	r3, r3, #1
	bl	printk
.L2958:
	ldrb	r3, [r7, #3984]	@ zero_extendqisi2
	ldrh	r2, [r4, #6]
	mul	r3, r3, r10
	add	r3, r3, #1
	rsb	r3, r9, r3
	cmp	r2, r3
	bls	.L2956
	ldr	r3, .L3052+44
	mov	r0, fp
	ldr	r2, [r5, #12]
	ldr	r1, [r5, #4]
	ldrb	r3, [r3, #3320]	@ zero_extendqisi2
	bl	ftl_prog_ppa_page
	str	r0, [sp, #20]
	ldrh	r0, [r4]
	bl	ftl_vpn_decrement
	adds	r3, r8, #1
	movne	r3, #1
	ldr	r2, [sp, #20]
	cmn	r2, #1
	cmnne	r8, #1
	beq	.L2959
	add	r3, sp, #96
	ldrh	r2, [r4, #12]
	str	fp, [r3, r9, asl #2]
	ldrh	r3, [r4, #10]
	add	r3, r3, r2
	add	r2, sp, #160
	sub	r3, r3, #1
	str	r3, [r2, r9, asl #2]
	b	.L2956
.L2959:
	ldrh	r2, [r4, #6]
	cmp	r2, #1
	movls	r3, #0
	andhi	r3, r3, #1
	cmp	r3, #0
	bne	.L3021
.L2956:
	add	r9, r9, #1
	b	.L2949
.L3050:
	ldr	fp, .L3052+44
	add	r9, sp, #32
	mov	r8, #0
.L2962:
	ldrb	r3, [r7, #3984]	@ zero_extendqisi2
	mul	r3, r3, r10
	cmp	r8, r3
	bcs	.L3051
	add	ip, sp, #96
	str	ip, [sp, #12]
	ldr	r3, [ip, r8, asl #2]
	cmn	r3, #1
	beq	.L2964
	ldr	r0, [r9, r8, asl #2]
	ldr	r1, [r5, #4]
	ldr	r2, [r5, #12]
	ldrb	r3, [fp, #3320]	@ zero_extendqisi2
	bl	ftl_read_ppa_page
	cmn	r0, #1
	cmpne	r0, #256
	ldr	ip, [sp, #12]
	bne	.L2964
	ldr	r0, [ip, r8, asl #2]
	ldrb	r3, [fp, #3320]	@ zero_extendqisi2
	ldr	r1, [r5, #4]
	ldr	r2, [r5, #12]
	bl	ftl_read_ppa_page
	ldr	r3, [r6]
	tst	r3, #16384
	mov	ip, r0
	beq	.L2966
	ldr	r3, [r5, #12]
	mov	r1, ip
	ldr	r0, .L3052+72
	str	ip, [sp, #12]
	ldr	r2, [r3, #8]
	ldr	r3, [r9, r8, asl #2]
	bl	printk
	ldr	ip, [sp, #12]
.L2966:
	cmn	ip, #1
	beq	.L2964
	ldr	r3, [r5, #12]
	ldr	r2, [r9, r8, asl #2]
	ldr	r1, [r3, #8]
	cmp	r2, r1
	bne	.L2964
	ldr	r1, [r6]
	add	ip, sp, #160
	tst	r1, #16384
	beq	.L2967
	ldr	r1, [r3, #4]
	ldr	r0, .L3052+76
	ldr	r3, [ip, r8, asl #2]
	str	ip, [sp, #12]
	bl	printk
	ldr	ip, [sp, #12]
.L2967:
	ldr	r3, [r5, #12]
	ldr	r2, [ip, r8, asl #2]
	ldmib	r3, {r0, r1}
	uxth	r2, r2
	bl	lpa_hash_update_ppa
.L2964:
	add	r8, r8, #1
	b	.L2962
.L3051:
	mov	r0, r5
	bl	zbuf_free
	ldr	r1, .L3052+44
	movw	r2, #3976
	ldrh	r3, [r4, #12]
	ldrh	r2, [r1, r2]
	ldrb	r1, [r4, #9]	@ zero_extendqisi2
	mla	r3, r1, r2, r3
	sub	r3, r3, #-1073741823
	ldr	r2, [sp, #16]
	ldr	r2, [r2, #-3592]
	ldr	r3, [r2, r3, asl #2]
	cmn	r3, #1
	beq	.L2969
	ldr	r1, .L3052+36
	movw	r2, #1660
	ldr	r0, .L3052+40
	bl	printk
	bl	dump_stack
.L2969:
	ldrh	r3, [r4, #6]
	cmp	r3, #1
	bne	.L2922
	mov	r0, r4
	bl	ftl_write_last_log_page
.L2922:
	add	sp, sp, #228
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3053:
	.align	2
.L3052:
	.word	.LANCHOR2
	.word	.LC189
	.word	.LC190
	.word	.LC191
	.word	.LC192
	.word	.LC193
	.word	.LANCHOR0+564
	.word	.LANCHOR0+3976
	.word	.LANCHOR4
	.word	.LANCHOR1+1836
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR0+4028
	.word	.LANCHOR3-3192
	.word	.LANCHOR3
	.word	.LC194
	.word	.LC195
	.word	.LC196
	.word	.LC197
	.word	.LC198
	.fnend
	.size	ftl_open_sblk_recovery, .-ftl_open_sblk_recovery
	.align	2
	.global	pm_write_page
	.type	pm_write_page, %function
pm_write_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #8
	mov	r4, r0
	ldr	r7, .L3068
	mov	r8, r1
	ldr	r9, .L3068+4
	mov	r6, r7
.L3055:
	ldr	r3, [r7, #3348]
	ldr	r2, [r3, #48]
	add	r2, r2, #1
	str	r2, [r3, #48]
	add	r2, r3, #696
	ldrh	r1, [r2]
	ldrh	r2, [r9]
	cmp	r1, r2
	bcs	.L3056
	add	r3, r3, #692
	ldrh	r2, [r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3057
.L3056:
	bl	pm_alloc_new_blk
	mov	r0, #0
	bl	ftl_info_flush
.L3057:
	ldr	r3, [r6, #3348]
	add	r3, r3, #692
	ldrh	r2, [r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3058
	ldr	r1, .L3068+8
	movw	r2, #263
	ldr	r0, .L3068+12
	bl	printk
	bl	dump_stack
.L3058:
	ldr	r3, [r6, #3348]
	ldr	r10, .L3068+16
	add	r2, r3, #692
	add	r3, r3, #696
	ldrh	r1, [r2]
	ldr	r2, .L3068+20
	ldr	r0, [r10, #-2520]
	ldrh	r5, [r2]
	ldrh	r2, [r3]
	mla	r5, r5, r1, r2
	mov	r1, #64
	bl	__memzero
	ldr	r3, [r10, #-2520]
	str	r4, [r3]
	ldr	r2, [r6, #3348]
	ldr	r3, [r10, #-2520]
	ldr	r1, [r2, #48]
	str	r1, [r3, #4]
	mov	r1, r5
	ldrb	r0, [r2, #694]	@ zero_extendqisi2
	ldrb	r2, [r6, #3320]	@ zero_extendqisi2
	str	r2, [sp]
	mov	r2, r8
	bl	ftl_prog_page
	ldr	r2, [r6, #3348]
	add	r1, r2, #696
	ldrh	r3, [r1]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r1]	@ movhi
	cmp	r3, #1
	beq	.L3059
	ldrb	r3, [r10, #-2516]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3060
.L3059:
	mov	r3, #0
	strb	r3, [r10, #-2516]
	b	.L3055
.L3060:
	cmn	r0, #1
	bne	.L3062
	mov	r1, r5
	ldr	r0, .L3068+24
	bl	printk
	b	.L3055
.L3062:
	movw	r3, #698
	mov	r0, #0
	ldrh	r3, [r2, r3]
	cmp	r4, r3
	addcc	r4, r4, #176
	strcc	r5, [r2, r4, asl #2]
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L3069:
	.align	2
.L3068:
	.word	.LANCHOR0
	.word	.LANCHOR0+3976
	.word	.LANCHOR1+1860
	.word	.LC0
	.word	.LANCHOR5
	.word	.LANCHOR0+4076
	.word	.LC199
	.fnend
	.size	pm_write_page, .-pm_write_page
	.align	2
	.global	ftl_dump_write_open_sblk
	.type	ftl_dump_write_open_sblk, %function
ftl_dump_write_open_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 56
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r0
	ldr	r4, .L3103
	.pad #100
	sub	sp, sp, #100
	add	r3, r4, #564
	ldrh	r3, [r3]
	cmp	r3, r0
	bls	.L3070
	ldrb	r3, [r4, #4035]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3072
	ldrb	r3, [r4, #4033]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3070
.L3072:
	ldr	r3, [r4, #568]
	mov	r6, fp, asl #2
	add	r1, sp, #80
	mov	r0, fp
	add	r3, r3, r6
	strh	fp, [sp, #64]	@ movhi
	mov	r5, #0
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	mov	r8, r5
	and	r3, r3, #224
	cmp	r3, #160
	ldreq	r3, .L3103
	movne	r9, #1
	ldreqb	r9, [r3, #4032]	@ zero_extendqisi2
	bl	ftl_get_blk_list_in_sblk
	movw	r3, #3976
	ldr	r2, [r4, #568]
	ldrh	r3, [r4, r3]
	uxtb	r0, r0
	strb	r0, [sp, #73]
	strh	r5, [sp, #66]	@ movhi
	smulbb	r0, r3, r0
	strb	r5, [sp, #69]
	strh	r5, [sp, #74]	@ movhi
	add	r3, r2, r6
	strh	r0, [sp, #70]	@ movhi
	ldrh	r1, [r2, r6]
	mov	r6, r5
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ubfx	r1, r1, #0, #11
	str	r1, [sp]
	ldr	r2, [r2, fp, asl #2]
	mov	r1, fp
	ldr	r0, .L3103+4
	ubfx	r2, r2, #11, #8
	str	r2, [sp, #4]
	mov	r2, r3, lsr #5
	ubfx	r3, r3, #3, #2
	bl	printk
	mov	r0, #1
	bl	buf_alloc
	mov	r4, r0
.L3074:
	ldr	r3, .L3103+8
	uxth	r7, r5
	ldrh	r3, [r3]
	cmp	r3, r7
	bls	.L3095
	mov	r3, r7, asl #1
	mov	r10, #0
	sub	r3, r3, #1
	str	r3, [sp, #44]
.L3084:
	ldrb	r3, [sp, #73]	@ zero_extendqisi2
	uxth	r8, r10
	cmp	r3, r8
	bls	.L3100
	add	ip, r8, #8
	mov	r6, #1
	mov	r3, ip, asl r6
	str	r3, [sp, #48]
.L3083:
	cmp	r6, r9
	bhi	.L3101
	ldr	r2, [sp, #48]
	add	r3, sp, #96
	add	r3, r3, r2
	ldrh	r2, [r3, #-32]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L3076
	ldr	r3, .L3103+12
	cmp	r9, #3
	ldrh	r3, [r3]
	mul	r3, r3, r2
	uxtaheq	r3, r3, r5
	orreq	r3, r3, r6, asl #24
	beq	.L3078
	cmp	r9, #2
	uxtahne	r3, r3, r5
	ldreq	r1, [sp, #44]
	addeq	r3, r3, r1
	ldreq	r1, .L3103
	addeq	r3, r3, r6
	ldreqb	r1, [r1, #4032]	@ zero_extendqisi2
	orreq	r3, r3, r1, asl #24
.L3078:
	str	r3, [r4, #24]
	mov	r1, #1
	mov	r0, r4
	str	r2, [sp, #60]
	str	r3, [sp, #56]
	bl	sblk_read_page
	ldr	r0, [r4, #4]
	ldr	ip, [r4, #36]
	ldr	r1, [r4, #12]
	str	ip, [sp]
	ldr	lr, [r0]
	ldr	r2, [sp, #60]
	ldr	r3, [sp, #56]
	str	lr, [sp, #4]
	ldr	lr, [r0, #4]
	str	ip, [sp, #52]
	str	lr, [sp, #8]
	ldr	lr, [r0, #8]
	str	lr, [sp, #12]
	ldr	r0, [r0, #12]
	str	r0, [sp, #16]
	ldr	r0, [r1]
	str	r0, [sp, #20]
	ldr	r0, [r1, #4]
	str	r0, [sp, #24]
	ldr	r0, [r1, #8]
	str	r0, [sp, #28]
	ldr	r1, [r1, #12]
	ldr	r0, .L3103+16
	str	r1, [sp, #32]
	mov	r1, r2
	mov	r2, r7
	bl	printk
	ldr	ip, [sp, #52]
	cmp	ip, #512
	beq	.L3095
.L3076:
	add	r6, r6, #1
	uxth	r6, r6
	b	.L3083
.L3101:
	add	r10, r10, #1
	b	.L3084
.L3100:
	add	r5, r5, #1
	b	.L3074
.L3095:
.L3080:
	str	r6, [sp]
	mov	r3, r8
	ldr	r6, .L3103
	mov	r1, fp
	mov	r2, r7
	ldr	r0, .L3103+20
	bl	printk
	mov	r1, #0
	ldrb	r2, [r6, #3320]	@ zero_extendqisi2
	mov	r5, r7
	ldr	r0, [r4, #4]
	mov	r9, r6
	mov	r2, r2, asl #9
	bl	ftl_memset
	ldrb	r2, [r6, #3320]	@ zero_extendqisi2
	ldr	r0, [r4, #12]
	mov	r1, #0
	mov	r2, r2, asl #1
	bl	ftl_memset
.L3085:
	ldr	r3, .L3103+8
	ldrh	r3, [r3]
	cmp	r3, r5
	bls	.L3090
	ldr	r7, .L3103+12
	mov	r10, r5, asl #1
	mov	r8, #0
.L3091:
	ldrb	r2, [sp, #73]	@ zero_extendqisi2
	uxth	r3, r8
	cmp	r2, r3
	bls	.L3102
	mov	r2, r3
	ldr	r0, .L3103+24
	mov	r1, r5
	str	r3, [sp, #44]
	bl	printk
	ldrb	r2, [r6, #4035]	@ zero_extendqisi2
	cmp	r2, #0
	ldr	r3, [sp, #44]
	add	r3, r3, #8
	beq	.L3086
	add	r2, sp, #96
	mov	r0, r4
	add	r3, r2, r3, asl #1
	ldrh	r2, [r3, #-32]
	ldrh	r3, [r7]
	mul	r3, r3, r2
	orr	r3, r3, r5
	str	r3, [r4, #24]
	bl	sblk_3d_tlc_dump_prog
	b	.L3087
.L3086:
	ldrb	r1, [r9, #4032]	@ zero_extendqisi2
	cmp	r1, #2
	bne	.L3088
	add	r2, sp, #96
	mov	r0, r4
	add	r3, r2, r3, asl #1
	ldrh	r2, [r3, #-32]
	ldrh	r3, [r7]
	mul	r3, r3, r2
	orr	r3, r3, r10
	orr	r3, r3, #33554432
	str	r3, [r4, #24]
	bl	sblk_mlc_dump_prog
	b	.L3087
.L3088:
	add	r1, sp, #96
	mvn	r0, #0
	add	r3, r1, r3, asl #1
	ldrb	lr, [r9, #3320]	@ zero_extendqisi2
	ldrh	r1, [r3, #-32]
	ldrh	r3, [r7]
	mul	r1, r3, r1
	ldrb	r3, [r9, #1700]	@ zero_extendqisi2
	orr	ip, r1, r5
	rsb	r1, r3, #24
	mvn	r3, r0, asl r3
	str	ip, [r4, #24]
	and	r0, r3, ip, lsr r1
	str	lr, [sp]
	str	r2, [sp, #4]
	bic	r1, ip, r3, asl r1
	uxtb	r0, r0
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #12]
	bl	flash_prog_page_en
.L3087:
	add	r8, r8, #1
	b	.L3091
.L3102:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L3085
.L3090:
	mov	r0, r4
	bl	zbuf_free
	ldr	r0, .L3103+28
	mov	r1, fp
	bl	printk
.L3070:
	add	sp, sp, #100
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3104:
	.align	2
.L3103:
	.word	.LANCHOR0
	.word	.LC200
	.word	.LANCHOR0+3976
	.word	.LANCHOR0+4076
	.word	.LC201
	.word	.LC202
	.word	.LC203
	.word	.LC204
	.fnend
	.size	ftl_dump_write_open_sblk, .-ftl_dump_write_open_sblk
	.align	2
	.global	ftl_ink_check_sblk
	.type	ftl_ink_check_sblk, %function
ftl_ink_check_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r8, r0, asl #2
	ldr	r6, .L3128
	mov	r5, r0
	.pad #52
	sub	sp, sp, #52
	mov	r1, r5
	ldr	r3, [r6, #568]
	ldrh	r2, [r3, r8]
	ldr	r3, [r3, r0, asl #2]
	ubfx	r2, r2, #0, #11
	ldr	r0, .L3128+4
	ubfx	r3, r3, #11, #8
	bl	printk
	movw	r3, #65535
	cmp	r5, r3
	beq	.L3105
	add	r3, r6, #564
	ldrh	r3, [r3]
	cmp	r3, r5
	bls	.L3105
	mov	r1, #0
	mov	r0, r5
	bl	ftl_erase_sblk
	mov	r0, r5
	mov	r9, #0
	add	r1, sp, #32
	strh	r5, [sp, #16]	@ movhi
	bl	ftl_get_blk_list_in_sblk
	strb	r0, [sp, #25]
	mov	r0, #1
	bl	buf_alloc
	ldr	r3, [r6, #568]
	ldrb	r2, [r6, #3320]	@ zero_extendqisi2
	ldrh	r3, [r3, r8]
	mov	r2, r2, asl #9
	ubfx	r3, r3, #0, #11
	cmp	r3, #1
	movle	r1, #85
	movgt	r1, #170
	mov	r4, r0
	ldr	r0, [r0, #4]
	bl	ftl_memset
	bl	sblk_wait_write_queue_completed
.L3109:
	ldr	r3, .L3128+8
	uxth	fp, r9
	ldr	r2, .L3128+8
	ldrh	r3, [r3]
	cmp	r3, fp
	bls	.L3124
	add	r7, r2, #100
	mov	r10, #0
.L3112:
	ldrb	r2, [sp, #25]	@ zero_extendqisi2
	uxth	r3, r10
	cmp	r2, r3
	bls	.L3125
	add	r2, sp, #48
	add	r3, r2, r3, asl #1
	movw	r2, #65535
	ldrh	r3, [r3, #-16]
	cmp	r3, r2
	beq	.L3110
	ldrh	r1, [r7]
	mov	ip, #0
	ldr	r2, [r4, #4]
	mvn	r0, #0
	mla	r3, r1, r3, fp
	ldr	r1, .L3128+12
	str	r3, [r2]
	ldr	r2, [r4, #4]
	str	r1, [r2, #4]
	ldr	r2, [r4, #12]
	str	ip, [r2]
	ldrb	r2, [r6, #1700]	@ zero_extendqisi2
	ldrb	lr, [r6, #3320]	@ zero_extendqisi2
	rsb	r1, r2, #24
	str	ip, [sp, #4]
	mvn	r2, r0, asl r2
	and	r0, r2, r3, lsr r1
	str	lr, [sp]
	bic	r1, r3, r2, asl r1
	uxtb	r0, r0
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #12]
	bl	flash_prog_page_en
.L3110:
	add	r10, r10, #1
	b	.L3112
.L3125:
	add	r9, r9, #1
	b	.L3109
.L3124:
	mov	r7, #0
.L3113:
	ldrh	r1, [r2]
	uxth	r10, r7
	cmp	r1, r10
	bls	.L3126
	mov	fp, #0
.L3118:
	ldrb	r1, [sp, #25]	@ zero_extendqisi2
	uxth	r6, fp
	cmp	r1, r6
	bls	.L3127
	add	r6, r6, #8
	add	r3, sp, #48
	add	r6, r3, r6, asl #1
	movw	r3, #65535
	ldrh	r1, [r6, #-32]
	cmp	r1, r3
	beq	.L3115
	ldr	r3, .L3128+16
	mov	r0, r4
	str	r2, [sp, #12]
	ldrh	r9, [r3]
	mla	r9, r9, r1, r10
	mov	r1, #1
	str	r9, [r4, #24]
	bl	sblk_read_page
	ldr	r1, [r4, #4]
	ldr	r1, [r1]
	cmp	r1, r9
	ldr	r2, [sp, #12]
	beq	.L3115
	mov	r0, r9
	bl	ftl_mask_bad_block
	mvn	r1, #0
	strh	r1, [r6, #-32]	@ movhi
	ldr	r2, [sp, #12]
.L3115:
	add	fp, fp, #1
	b	.L3118
.L3127:
	add	r7, r7, #1
	b	.L3113
.L3126:
	mov	r0, r4
	bl	zbuf_free
	ldr	r3, .L3128
	ldr	r0, .L3128+20
	mov	r1, r5
	ldr	r3, [r3, #568]
	ldrh	r2, [r3, r8]
	ldr	r3, [r3, r5, asl #2]
	ubfx	r2, r2, #0, #11
	ubfx	r3, r3, #11, #8
	bl	printk
.L3105:
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3129:
	.align	2
.L3128:
	.word	.LANCHOR0
	.word	.LC205
	.word	.LANCHOR0+3976
	.word	1437269760
	.word	.LANCHOR0+4076
	.word	.LC206
	.fnend
	.size	ftl_ink_check_sblk, .-ftl_ink_check_sblk
	.align	2
	.global	ftl_open_sblk_init
	.type	ftl_open_sblk_init, %function
ftl_open_sblk_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r4, r0
	ldr	r6, .L3141
	mov	r7, r1
	ldr	r9, .L3141+4
	mov	r8, r6
.L3131:
.L3135:
	mov	r0, r7
	bl	ftl_alloc_sblk
	ldr	r3, [r6, #3348]
	ldrh	r3, [r3, #150]
	cmp	r3, #0
	mov	r5, r0
	beq	.L3132
	ldr	r2, [r6, #568]
	mov	r3, r0, asl #2
	ldrh	r3, [r2, r3]
	ubfx	r3, r3, #0, #11
	cmp	r3, #2
	bgt	.L3132
	bl	ftl_ink_check_sblk
.L3132:
	mov	r1, #0
	mov	r0, r5
	bl	ftl_erase_sblk
	add	r1, r4, #16
	mov	r0, r5
	mov	r10, r5, asl #1
	bl	ftl_get_blk_list_in_sblk
	strh	r5, [r4]	@ movhi
	ldrh	r2, [r9]
	cmp	r7, #2
	strb	r7, [r4, #4]
	uxtb	r3, r0
	strb	r3, [r4, #9]
	mov	r0, #0
	strh	r0, [r4, #2]	@ movhi
	smulbb	r3, r3, r2
	strb	r0, [r4, #5]
	strh	r0, [r4, #10]	@ movhi
	ldrneb	r0, [r6, #3984]	@ zero_extendqisi2
	strh	r3, [r4, #6]	@ movhi
	ldr	r3, .L3141+8
	smulbbne	r0, r0, r2
	ldr	r1, [r3, #-3592]
	ldrb	r3, [r8, #3984]	@ zero_extendqisi2
	mul	r2, r3, r2
	uxthne	r0, r0
	strh	r0, [r4, #12]	@ movhi
	add	r0, r1, r0, asl #2
	mov	r1, #255
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r3, [r8, #576]
	ldrh	r2, [r4, #6]
	strh	r2, [r3, r10]	@ movhi
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	cmp	r3, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	ldr	r0, .L3141+12
	mov	r1, r5
	bl	printk
	ldr	r3, [r6, #576]
	mvn	r2, #0
	strh	r2, [r3, r10]	@ movhi
	mov	r3, #7
	strb	r3, [r4, #4]
	b	.L3135
.L3142:
	.align	2
.L3141:
	.word	.LANCHOR0
	.word	.LANCHOR0+3976
	.word	.LANCHOR3
	.word	.LC132
	.fnend
	.size	ftl_open_sblk_init, .-ftl_open_sblk_init
	.align	2
	.global	flash_info_flush
	.type	flash_info_flush, %function
flash_info_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3157
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3144
	ldr	r0, .L3157+4
	mov	r1, #324
	ldr	r2, .L3157+8
	bl	printk
.L3144:
	ldr	r6, .L3157+12
	mov	r1, #64
	ldr	r5, .L3157+16
	mov	r9, #0
	ldr	r8, .L3157+20
	ldr	r0, [r6, #-2512]
	bl	__memzero
	ldr	r0, .L3157+24
	ldr	r1, [r5, #1696]
	mov	r2, #4
	mov	r3, #16
	mov	r10, r8
	bl	rknand_print_hex
	ldr	r4, [r5, #1696]
	add	r0, r4, #16
	ldr	r1, [r4, #8]
	bl	js_hash
	str	r0, [r4, #12]
.L3145:
	ldrb	r3, [r6, #-2508]	@ zero_extendqisi2
	ldrh	r4, [r8, #2]
	ldrh	ip, [r5, #2]
	mov	r1, r3
	ldr	r0, .L3157+28
	mov	r2, r4
	str	r3, [sp, #8]
	str	ip, [sp, #12]
	bl	printk
	ldr	r2, .L3157+32
	ldrh	r0, [r8, #2]
	ldr	r7, .L3157+12
	ldrh	r2, [r2]
	ldr	fp, .L3157+16
	sub	r2, r2, #1
	cmp	r0, r2
	ldr	r3, [sp, #8]
	ldr	ip, [sp, #12]
	blt	.L3146
	ldr	r4, [fp, #1696]
	ldrb	r2, [r7, #-2507]	@ zero_extendqisi2
	ldr	r3, [r4, #4]
	mov	r0, r4
	add	r3, r3, #1
	str	r3, [r4, #4]
	ldrb	r3, [r7, #-2508]	@ zero_extendqisi2
	strb	r2, [r7, #-2508]
	ldrh	r2, [r4, #16]
	strb	r3, [r7, #-2507]
	mov	r3, #0
	add	r2, r2, #1
	strh	r2, [r0, #16]!	@ movhi
	ldr	r1, [r4, #8]
	strh	r3, [r8, #2]	@ movhi
	str	r3, [sp, #8]
	bl	js_hash
	ldrb	r2, [r7, #-2508]	@ zero_extendqisi2
	str	r0, [r4, #12]
	ldr	r3, [sp, #8]
	ldrh	r4, [fp, #2]
	mov	r0, r3
	mul	r4, r4, r2
	b	.L3155
.L3146:
	cmp	r0, #0
	mla	r4, ip, r3, r4
	bne	.L3147
.L3155:
	mov	r1, r4
	bl	flash_erase_block
.L3147:
	ldr	r2, [r5, #1696]
	mov	r7, #1
	ldr	r3, [r6, #-2512]
	mov	r0, #0
	mov	r1, r4
	ldr	r2, [r2, #4]
	str	r2, [r3]
	ldr	r2, .L3157+36
	ldr	r3, [r6, #-2512]
	str	r2, [r3, #4]
	mov	r2, #4
	stmia	sp, {r2, r7}
	ldr	r2, [r5, #1696]
	bl	flash_prog_page_en
	ldrh	r3, [r10, #2]
	add	r3, r3, r7
	strh	r3, [r10, #2]	@ movhi
	cmn	r0, #1
	bne	.L3148
	mov	r1, r4
	ldr	r0, .L3157+40
	bl	printk
	b	.L3145
.L3148:
	cmp	r9, #0
	moveq	r9, r7
	beq	.L3145
.L3156:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3158:
	.align	2
.L3157:
	.word	.LANCHOR2
	.word	.LC136
	.word	.LANCHOR1+1876
	.word	.LANCHOR5
	.word	.LANCHOR0
	.word	.LANCHOR5-2508
	.word	.LC207
	.word	.LC208
	.word	.LANCHOR0+1708
	.word	1398362953
	.word	.LC209
	.fnend
	.size	flash_info_flush, .-flash_info_flush
	.align	2
	.global	flash_info_blk_init
	.type	flash_info_blk_init, %function
flash_info_blk_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3191
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r3]
	ldr	r6, .L3191+4
	tst	r3, #4096
	beq	.L3160
	ldr	r3, [r6, #1696]
	mov	r1, #50
	ldr	r0, .L3191+8
	ldr	r2, .L3191+12
	str	r3, [sp]
	mov	r3, #2048
	bl	printk
.L3160:
	ldr	fp, .L3191+4
	mov	r5, #4
	ldr	r8, .L3191+16
.L3163:
	ldrh	r1, [r6, #2]
	mov	r9, #4
	ldr	r4, .L3191+20
	mov	r0, #0
	str	r9, [sp]
	mul	r1, r1, r5
	ldr	r2, [r6, #1696]
	ldr	r3, [r4, #-2512]
	bl	flash_read_page_en
	ldr	r7, .L3191+4
	cmn	r0, #1
	beq	.L3161
	ldr	r2, [fp, #1696]
	ldr	r10, .L3191+16
	ldr	r3, [r2]
	cmp	r3, r8
	beq	.L3162
.L3161:
	add	r5, r5, #1
	cmp	r5, #16
	bne	.L3163
.L3189:
	mvn	r0, #0
	b	.L3187
.L3162:
	ldrb	r1, [r2, #37]	@ zero_extendqisi2
	mov	r0, #0
	ldrb	r3, [r2, #36]	@ zero_extendqisi2
	strb	r1, [r4, #-2507]
	ldrh	r1, [r7, #2]
	strb	r3, [r4, #-2508]
	str	r9, [sp]
	mul	r1, r1, r3
	ldr	r3, [r4, #-2512]
	bl	flash_read_page_en
	cmn	r0, #1
	beq	.L3172
	ldr	r3, [r7, #1696]
	ldr	r2, [r3]
	cmp	r2, r8
	ldreq	r5, [r3, #4]
	beq	.L3165
.L3172:
	mov	r5, #0
.L3165:
	ldrb	r3, [r4, #-2507]	@ zero_extendqisi2
	mov	r2, #4
	ldrh	r1, [r6, #2]
	mov	r0, #0
	str	r2, [sp]
	ldr	r2, [r6, #1696]
	mul	r1, r1, r3
	ldr	r3, [r4, #-2512]
	bl	flash_read_page_en
	ldr	r7, .L3191+20
	cmn	r0, #1
	beq	.L3166
	ldr	r3, .L3191+4
	ldr	r3, [r3, #1696]
	ldr	r2, [r3]
	cmp	r2, r10
	bne	.L3166
	ldr	r2, [r3, #4]
	cmp	r5, r2
	ldrccb	r2, [r3, #37]	@ zero_extendqisi2
	ldrccb	r3, [r3, #36]	@ zero_extendqisi2
	strccb	r2, [r7, #-2508]
	strccb	r3, [r7, #-2507]
.L3166:
	ldr	r2, [r6, #1696]
	mov	r8, #4
	ldr	r3, [r4, #-2512]
	mov	r0, #0
	ldrb	r1, [r4, #-2508]	@ zero_extendqisi2
	mov	r7, #0
	str	r8, [sp]
	bl	flash_get_last_written_page
	ldr	r3, .L3191+24
	ldrh	r9, [r6, #2]
	ldr	ip, .L3191+4
	uxth	fp, r0
	add	r2, fp, #1
	strh	r2, [r3, #2]	@ movhi
	ldrb	r3, [r4, #-2508]	@ zero_extendqisi2
	mla	r9, r9, r3, r0
.L3167:
	rsb	r5, r7, fp
	sxth	r5, r5
	cmp	r5, #0
	blt	.L3190
	str	r8, [sp]
	mov	r0, #0
	rsb	r1, r7, r9
	ldr	r2, [r6, #1696]
	ldr	r3, [r4, #-2512]
	str	ip, [sp, #12]
	bl	flash_read_page_en
	cmn	r0, #1
	ldr	ip, [sp, #12]
	beq	.L3168
	ldr	r3, [ip, #1696]
	ldr	r3, [r3]
	cmp	r3, r10
	beq	.L3169
.L3168:
	add	r7, r7, #1
	b	.L3167
.L3190:
	cmn	r5, #1
	bne	.L3169
	ldr	r3, [r6, #1696]
	ldr	r0, .L3191+28
	ldr	r1, [r3]
	bl	printk
	b	.L3189
.L3169:
	cmp	r7, #1
	bls	.L3173
	bl	flash_info_flush
.L3173:
	mov	r0, #0
.L3187:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3192:
	.align	2
.L3191:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC210
	.word	.LANCHOR1+1896
	.word	1398362953
	.word	.LANCHOR5
	.word	.LANCHOR5-2508
	.word	.LC211
	.fnend
	.size	flash_info_blk_init, .-flash_info_blk_init
	.align	2
	.global	nand_flash_init
	.type	nand_flash_init, %function
nand_flash_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r0
	ldr	r6, .L3274
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3194
	ldr	r0, .L3274+4
	movw	r1, #2578
	ldr	r2, .L3274+8
	bl	printk
.L3194:
	ldr	r5, .L3274+12
	mov	r0, r4
	mov	r7, #0
	ldr	r4, .L3274+16
	str	r7, [r5, #-3272]
	bl	nandc_init
	ldr	r3, .L3274+20
	add	r0, r4, #588
	mov	r1, #8
	add	r10, r4, #1744
	mov	r9, r4
	str	r3, [r4, #1620]
	mov	r3, #1
	strb	r3, [r4, #585]
	mov	r3, #3
	strb	r3, [r4, #1700]
	bl	__memzero
	add	r0, r4, #1712
	mov	r1, #32
	bl	__memzero
	mov	fp, r10
.L3200:
	mov	r8, r7, asl #3
	uxtb	r0, r7
	add	r1, r10, r8
	bl	flash_read_id
	cmp	r7, #0
	ldr	r1, .L3274+24
	bne	.L3195
	ldrb	r3, [r9, #1744]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bls	.L3196
.L3198:
	mvn	r8, #1
	b	.L3266
.L3196:
	ldrb	r3, [r9, #1745]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L3198
.L3195:
	ldrb	r3, [fp, r8]	@ zero_extendqisi2
	add	r7, r7, #1
	cmp	r3, #181
	ldreq	r3, .L3274+24
	moveq	r2, #44
	streqb	r2, [r8, r3]
	cmp	r7, #4
	bne	.L3200
	ldr	ip, .L3274+28
	mov	r3, #0
.L3233:
	mov	r0, r3, asl #5
	ldrb	r8, [ip, r3, asl #5]	@ zero_extendqisi2
	add	r7, r0, #1
	ldr	lr, .L3274+28
	add	r7, ip, r7
	mov	r2, #0
.L3201:
	cmp	r2, r8
	bcs	.L3273
	ldrb	r10, [r7, r2]	@ zero_extendqisi2
	ldrb	r9, [r1, r2]	@ zero_extendqisi2
	cmp	r10, r9
	bne	.L3202
	add	r2, r2, #1
	b	.L3201
.L3273:
	adds	r8, lr, r0
	beq	.L3198
	ldr	r3, .L3274
	ldr	r2, .L3274+32
	add	r0, r3, r0
	mov	r3, #0
	ldrb	r1, [r0, #446]	@ zero_extendqisi2
.L3206:
	ldrb	r0, [r2, r3, asl #5]	@ zero_extendqisi2
	cmp	r0, r1
	beq	.L3205
	add	r3, r3, #1
	cmp	r3, #4
	bne	.L3206
.L3205:
	ldr	r1, .L3274+32
	mov	r2, #32
	ldr	r7, .L3274
	add	r1, r1, r3, asl #5
	ldr	r0, .L3274+36
	bl	ftl_memcpy
	add	r0, r7, #4
	mov	r1, r8
	mov	r2, #32
	bl	ftl_memcpy
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #8
	bhi	.L3207
	ldrb	r3, [r7, #24]	@ zero_extendqisi2
	cmp	r3, #60
	movhi	r3, #60
	strhib	r3, [r7, #24]
	b	.L3207
.L3202:
	add	r3, r3, #1
	cmp	r3, #44
	bne	.L3233
	b	.L3198
.L3207:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3235
	ldr	r0, .L3274+4
	movw	r1, #2600
	ldr	r2, .L3274+8
	bl	printk
.L3235:
	ldr	r3, [r4, #1620]
	mov	r0, #16384
	ldr	r7, .L3274+40
	mov	r8, #0
	ldrh	r3, [r3, #10]
	cmp	r3, #2304
	ldreq	r3, .L3274+16
	moveq	r2, #2
	streqb	r2, [r3, #1700]
	bl	ftl_malloc
	str	r0, [r5, #-3264]
	mov	r0, #16384
	bl	ftl_malloc
	str	r0, [r5, #-3288]
	mov	r0, #2048
	bl	ftl_malloc
	str	r0, [r4, #1696]
	mov	r0, #64
	bl	ftl_malloc
	str	r0, [r5, #-3260]
	mov	r0, #64
	bl	ftl_malloc
	str	r0, [r5, #-3280]
	mov	r0, #64
	bl	ftl_malloc
	strb	r8, [r7, #-2504]
	str	r0, [r7, #-2512]
	bl	flash_die_info_init
	ldrb	r0, [r6, #22]	@ zero_extendqisi2
	bl	flash_lsb_page_tbl_build
	ldrb	r0, [r6, #24]	@ zero_extendqisi2
	bl	nandc_bch_sel
	ldr	r1, [r4, #1620]
	str	r8, [r5, #-3268]
	ldrh	r3, [r1, #16]
	ubfx	r2, r3, #8, #3
	strb	r2, [r4, #1779]
	ubfx	r2, r3, #3, #1
	strb	r2, [r5, #-3256]
	ubfx	r2, r3, #4, #1
	strb	r2, [r4, #1780]
	ubfx	r2, r3, #12, #1
	strb	r2, [r4, #4035]
	ubfx	r2, r3, #13, #1
	strb	r2, [r4, #4033]
	ubfx	r2, r3, #11, #1
	strb	r2, [r4, #1778]
	ldrb	r2, [r1, #31]	@ zero_extendqisi2
	ubfx	r0, r2, #1, #1
	ubfx	r2, r2, #2, #1
	strb	r2, [r7, #-2502]
	ubfx	r2, r3, #14, #1
	mov	r3, r3, lsr #15
	strb	r3, [r4, #4034]
	mov	r3, #60
	strb	r3, [r4, #1624]
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	strb	r2, [r4, #1]
	cmp	r3, #9
	ldrb	r2, [r1, #28]	@ zero_extendqisi2
	strb	r0, [r7, #-2503]
	ldreq	r1, .L3274+16
	moveq	r0, #70
	strb	r2, [r4, #1625]
	strb	r2, [r4]
	ldrb	r2, [r6, #11]	@ zero_extendqisi2
	streqb	r0, [r1, #1624]
	cmp	r2, #9
	ldreq	r2, .L3274+16
	moveq	r1, #0
	streqb	r1, [r2, #1625]
	ldrb	r2, [r6, #35]	@ zero_extendqisi2
	tst	r2, #1
	movne	r2, #3
	strneb	r2, [r4, #1625]
	cmp	r3, #8
	bne	.L3212
	ldrb	r3, [r4, #1744]	@ zero_extendqisi2
	cmp	r3, #44
	cmpne	r3, #137
	bne	.L3212
	ldrb	r3, [r6, #32]	@ zero_extendqisi2
	cmp	r3, #3
	ldreq	r3, .L3274+16
	moveq	r2, #0
	streqb	r2, [r3, #1625]
.L3212:
	ldrh	r2, [r6, #20]
	ldrb	r3, [r6, #23]	@ zero_extendqisi2
	tst	r2, #64
	strb	r3, [r4, #1692]
	beq	.L3214
	sub	r2, r3, #17
	sub	r1, r3, #21
	clz	r1, r1
	cmp	r3, #21
	cmpne	r2, #2
	mov	r1, r1, lsr #5
	bhi	.L3215
	ldr	r2, .L3274+44
	cmp	r1, #0
	ldr	r3, .L3274+12
	str	r2, [r5, #-3268]
	moveq	r2, #15
	movne	r2, #4
	strb	r2, [r3, #-3276]
	b	.L3214
.L3215:
	sub	r2, r3, #65
	cmp	r3, #33
	cmpne	r2, #1
	bhi	.L3218
	ldr	r3, .L3274+48
	mov	r2, #4
	str	r3, [r5, #-3268]
	ldr	r3, .L3274+16
	strb	r2, [r3, #1693]
	b	.L3272
.L3218:
	sub	r2, r3, #67
	sub	ip, r3, #35
	cmp	r2, #1
	sub	lr, r3, #68
	sub	r1, r3, #34
	clz	ip, ip
	clz	lr, lr
	movhi	r0, #0
	movls	r0, #1
	cmp	r2, #1
	cmphi	r1, #1
	mov	ip, ip, lsr #5
	mov	lr, lr, lsr #5
	bhi	.L3219
	ldr	r2, .L3274+48
	ldr	r3, .L3274+12
	str	r2, [r5, #-3268]
	orrs	r2, lr, ip
	moveq	r2, #7
	movne	r2, #17
	cmp	r0, #0
	strb	r2, [r3, #-3276]
	movne	r3, #4
	moveq	r3, #5
	strb	r3, [r4, #1693]
	b	.L3214
.L3219:
	sub	r3, r3, #36
	cmp	r3, #1
	bhi	.L3214
	ldr	r3, .L3274+52
	str	r3, [r5, #-3268]
.L3272:
	mov	r3, #7
	strb	r3, [r5, #-3276]
.L3214:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3223
	ldr	r0, .L3274+4
	movw	r1, #2671
	ldr	r2, .L3274+8
	bl	printk
.L3223:
	mov	r3, #0
	strb	r3, [r4, #1658]
	ldrb	r3, [r4, #1779]	@ zero_extendqisi2
	mov	r0, #4
	tst	r3, #1
	beq	.L3224
	bl	nandc_set_if_mode
	mov	r0, #1
	bl	flash_set_interface_mode
	mov	r0, #1
.L3224:
	bl	nandc_set_if_mode
	bl	flash_info_blk_init
	cmn	r0, #1
	mov	r8, r0
	bne	.L3226
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3227
	ldr	r0, .L3274+4
	mov	r1, #2688
	ldr	r2, .L3274+8
	bl	printk
.L3227:
	ldr	r3, [r4, #1696]
	mov	r2, #17
	mov	r0, #0
	strb	r0, [r4, #1704]
	strb	r2, [r3, #32]
	bl	zftl_flash_exit_slc_mode
	b	.L3266
.L3226:
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	sub	r3, r3, #1
	cmp	r3, #7
	ldrls	r3, .L3274+56
	strls	r3, [r5, #-3268]
	ldrb	r3, [r4, #1779]	@ zero_extendqisi2
	ldr	r5, .L3274+16
	tst	r3, #4
	beq	.L3230
	ldr	r3, [r5, #1696]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3230
	ldrb	r3, [r7, #-2508]	@ zero_extendqisi2
	mov	r0, #0
	ldrh	r1, [r5, #2]
	mul	r1, r1, r3
	bl	flash_ddr_para_scan
	ldrb	r3, [r5, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3230
	ldr	r2, [r5, #1696]
	strb	r3, [r2, #19]
	bl	flash_info_flush
.L3230:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3232
	ldr	r0, .L3274+4
	movw	r1, #2740
	ldr	r2, .L3274+8
	bl	printk
.L3232:
	bl	nand_flash_print_info
	mov	r8, #0
.L3266:
	mov	r0, r8
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3275:
	.align	2
.L3274:
	.word	.LANCHOR2
	.word	.LC136
	.word	.LANCHOR1+1916
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR2+4
	.word	.LANCHOR0+1744
	.word	.LANCHOR2+424
	.word	.LANCHOR2+1832
	.word	.LANCHOR0+1626
	.word	.LANCHOR5
	.word	micron_read_retrial
	.word	toshiba_read_retrial
	.word	toshiba_3d_read_retrial
	.word	hynix_read_retrial
	.fnend
	.size	nand_flash_init, .-nand_flash_init
	.align	2
	.global	ftl_sysblk_dump
	.type	ftl_sysblk_dump, %function
ftl_sysblk_dump:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r0
	.pad #52
	sub	sp, sp, #52
	mov	r0, #1
	bl	buf_alloc
	ldr	ip, .L3288
	mov	r6, #0
	mov	r5, r6
	ldr	fp, [r0, #4]
	mov	r4, r0
.L3277:
	ldr	r3, .L3288+4
	uxth	r8, r6
	ldrh	r3, [r3]
	cmp	r3, r8
	bls	.L3287
	ldrh	r7, [ip]
	add	r6, r6, #1
	ldr	r3, .L3288+8
	ldr	r1, [r4, #4]
	mla	r7, r7, r10, r8
	ldr	r2, [r4, #12]
	ldrb	r3, [r3, #3320]	@ zero_extendqisi2
	str	ip, [sp, #44]
	mov	r0, r7
	bl	ftl_read_ppa_page
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #12]
	str	r0, [sp]
	mov	r9, r0
	ldr	r1, [r2]
	ldr	r0, .L3288+12
	str	r1, [sp, #4]
	ldr	r1, [r2, #4]
	str	r1, [sp, #8]
	ldr	r1, [r2, #8]
	str	r1, [sp, #12]
	mov	r1, r10
	ldr	r2, [r2, #12]
	str	r2, [sp, #16]
	ldr	r2, [r3]
	str	r2, [sp, #20]
	ldr	r2, [r3, #4]
	str	r2, [sp, #24]
	ldr	r2, [r3, #8]
	str	r2, [sp, #28]
	mov	r2, r8
	ldr	r3, [r3, #12]
	str	r3, [sp, #32]
	mov	r3, r7
	bl	printk
	cmn	r9, #1
	cmpne	r9, #512
	moveq	r5, #1
	ldr	ip, [sp, #44]
	b	.L3277
.L3287:
	ldr	r0, .L3288+16
	add	r1, fp, #704
	mov	r2, #4
	mov	r3, #32
	bl	rknand_print_hex
	mov	r0, r4
	bl	zbuf_free
	cmp	r5, #0
	beq	.L3280
	ldr	r1, .L3288+20
	movw	r2, #1422
	ldr	r0, .L3288+24
	bl	printk
	bl	dump_stack
.L3280:
	mov	r0, r5
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3289:
	.align	2
.L3288:
	.word	.LANCHOR0+4076
	.word	.LANCHOR0+3976
	.word	.LANCHOR0
	.word	.LC201
	.word	.LC212
	.word	.LANCHOR1+1932
	.word	.LC0
	.fnend
	.size	ftl_sysblk_dump, .-ftl_sysblk_dump
	.align	2
	.global	dump_ftl_info
	.type	dump_ftl_info, %function
dump_ftl_info:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	movw	r6, #3976
	ldr	r4, .L3292
	ldr	r5, .L3292+4
	ldr	r0, .L3292+8
	add	r7, r4, #564
	ldrb	r1, [r4, #3322]	@ zero_extendqisi2
	bl	printk
	sub	r3, r5, #2928
	sub	r3, r3, #8
	ldrb	r1, [r5, #-2936]	@ zero_extendqisi2
	ldrb	r2, [r5, #-2935]	@ zero_extendqisi2
	ldrh	r3, [r3, #2]
	ldr	r0, .L3292+12
	bl	printk
	ldr	r3, [r4, #3348]
	ldr	r0, .L3292+16
	ldrh	r1, [r3, #130]
	ldrh	r2, [r3, #140]
	bl	printk
	ldr	r0, [r4, #580]
	ldrh	ip, [r0, #22]
	ldrh	r1, [r0, #16]
	ldrh	r2, [r0, #18]
	ldrb	r3, [r0, #21]	@ zero_extendqisi2
	str	ip, [sp]
	ldrh	r0, [r0, #26]
	str	r0, [sp, #4]
	ldr	r0, .L3292+20
	bl	printk
	ldr	r0, [r4, #580]
	ldrh	ip, [r0, #54]
	ldrh	r1, [r0, #48]
	ldrh	r2, [r0, #50]
	ldrb	r3, [r0, #53]	@ zero_extendqisi2
	str	ip, [sp]
	ldrh	r0, [r0, #58]
	str	r0, [sp, #4]
	ldr	r0, .L3292+24
	bl	printk
	ldr	r0, [r4, #580]
	ldrh	ip, [r0, #86]
	ldrh	r1, [r0, #80]
	ldrh	r2, [r0, #82]
	ldrb	r3, [r0, #85]	@ zero_extendqisi2
	str	ip, [sp]
	ldrh	r0, [r0, #90]
	str	r0, [sp, #4]
	ldr	r0, .L3292+28
	bl	printk
	ldrh	r2, [r4, r6]
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	ldr	r1, [r5, #-3592]
	ldr	r0, .L3292+32
	mul	r3, r3, r2
	mov	r2, #4
	mov	r3, r3, asl #1
	bl	rknand_print_hex
	ldr	r1, [r4, #576]
	ldrh	r3, [r7]
	mov	r2, #2
	ldr	r0, .L3292+36
	bl	rknand_print_hex
	ldr	r3, [r4, #3348]
	movw	ip, #698
	ldr	r0, .L3292+40
	add	r1, r3, #704
	mov	r2, #4
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldr	r1, [r4, #568]
	ldrh	r3, [r7]
	mov	r2, #4
	ldr	r0, .L3292+44
	bl	rknand_print_hex
	add	r1, r4, #4080
	ldr	r0, .L3292+48
	mov	r2, #2
	mov	r3, #256
	bl	rknand_print_hex
	ldrh	r2, [r4, r6]
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	ldr	r0, .L3292+52
	ldr	r1, [r5, #-3588]
	mul	r3, r3, r2
	mov	r2, #2
	mov	r3, r3, asl #1
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, lr}
	b	rknand_print_hex
.L3293:
	.align	2
.L3292:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC213
	.word	.LC214
	.word	.LC215
	.word	.LC216
	.word	.LC217
	.word	.LC218
	.word	.LC219
	.word	.LC220
	.word	.LC212
	.word	.LC221
	.word	.LC222
	.word	.LC223
	.fnend
	.size	dump_ftl_info, .-dump_ftl_info
	.align	2
	.global	pm_ppa_update_check
	.type	pm_ppa_update_check, %function
pm_ppa_update_check:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r5, r2
	ldr	r4, .L3298
	movw	r3, #4028
	mov	r7, r0
	mov	r6, r1
	ldrb	r2, [r4, #1700]	@ zero_extendqisi2
	ldrh	r0, [r4, r3]
	mvn	r3, #0
	rsb	r2, r2, #24
	ldrb	r1, [r4, #4030]	@ zero_extendqisi2
	rsb	r2, r0, r2
	mov	r0, r5, lsr r0
	bic	r0, r0, r3, asl r2
	bl	__aeabi_uidiv
	ldr	r3, [r4, #568]
	uxth	r0, r0
	add	r0, r3, r0, asl #2
	ldrb	r0, [r0, #2]	@ zero_extendqisi2
	mov	r0, r0, lsr #5
	cmp	r0, #1
	cmpne	r0, #7
	moveq	r0, #1
	movne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r1, r7
	mov	r2, r6
	mov	r3, r5
	ldr	r0, .L3298+4
	bl	printk
	bl	dump_ftl_info
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3299:
	.align	2
.L3298:
	.word	.LANCHOR0
	.word	.LC224
	.fnend
	.size	pm_ppa_update_check, .-pm_ppa_update_check
	.align	2
	.type	load_l2p_region, %function
load_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #31
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	.pad #8
	mov	r5, r0
	mov	r7, r1
	bls	.L3301
	ldr	r1, .L3315
	mov	r2, #32
	ldr	r0, .L3315+4
	bl	printk
	bl	dump_stack
.L3301:
	ldr	r6, .L3315+8
	movw	r9, #698
	ldr	r3, [r6, #3348]
	mov	r8, r6
	ldrh	r2, [r3, r9]
	cmp	r5, r2
	bls	.L3302
	mov	r1, r5
	ldr	r0, .L3315+12
	bl	printk
	ldr	r3, .L3315+16
	mov	r4, #0
	mov	r1, #255
	ldr	r0, [r4, #4]
	ldrh	r2, [r3, #-4]
	bl	ftl_memset
	ldr	r3, [r6, #3348]
	ldrh	r3, [r3, r9]
	cmp	r3, r5
	bcs	.L3313
	ldr	r0, .L3315+4
	mov	r2, #36
	ldr	r1, .L3315
	b	.L3314
.L3302:
	add	r2, r5, #176
	mov	r7, r7, asl #3
	ldr	r9, .L3315+20
	ldr	r4, [r3, r2, asl #2]
	ldr	r3, .L3315+24
	add	r10, r9, r7
	add	r2, r3, r7
	strh	r5, [r3, r7]	@ movhi
	mov	r3, #0
	cmp	r4, r3
	strh	r3, [r2, #2]	@ movhi
	bne	.L3305
	mov	r1, r5
	mov	r2, r4
	ldr	r0, .L3315+28
	bl	printk
	ldr	r3, .L3315+16
	ldr	r0, [r10, #-3560]
	mov	r1, #255
	ldrh	r2, [r3, #-4]
	bl	ftl_memset
	b	.L3313
.L3305:
	ldr	r6, .L3315+32
	mov	r0, r4
	ldrb	r3, [r8, #3320]	@ zero_extendqisi2
	ldr	r1, [r10, #-3560]
	ldr	r2, [r6, #-2520]
	bl	ftl_read_ppa_page
	ldr	r2, [r6, #-2520]
	ldr	r2, [r2]
	cmp	r2, r5
	mov	r3, r0
	bne	.L3306
	cmn	r0, #1
	cmpne	r0, #512
	beq	.L3306
.L3309:
	ldr	r3, [r6, #-2520]
	ldr	r3, [r3]
	cmp	r3, r5
	beq	.L3313
	ldr	r0, .L3315+4
	mov	r2, #63
	ldr	r1, .L3315
	b	.L3314
.L3306:
	mov	r1, r5
	str	r4, [sp]
	ldr	r0, .L3315+36
	add	r7, r9, r7
	bl	printk
	ldr	r3, [r8, #3348]
	movw	ip, #698
	ldr	r0, .L3315+40
	add	r1, r3, #704
	mov	r2, #4
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldrb	r3, [r8, #3320]	@ zero_extendqisi2
	ldr	r0, .L3315+44
	mov	r2, #4
	ldr	r1, [r7, #-3560]
	mov	r3, r3, asl #7
	bl	rknand_print_hex
	ldr	r0, .L3315+48
	ldr	r1, [r6, #-2520]
	mov	r2, #4
	mov	r3, #16
	bl	rknand_print_hex
	ldrb	r3, [r8, #3320]	@ zero_extendqisi2
	mov	r0, r4
	ldr	r1, [r7, #-3560]
	ldr	r2, [r6, #-2520]
	bl	ftl_read_ppa_page
	ldr	r3, [r6, #-2520]
	ldr	r3, [r3]
	cmp	r3, r5
	bne	.L3311
	cmn	r0, #1
	cmpne	r0, #512
	bne	.L3309
	b	.L3311
.L3314:
	bl	printk
	bl	dump_stack
.L3313:
	mov	r0, #0
	b	.L3304
.L3311:
	mvn	r0, #0
.L3304:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L3316:
	.align	2
.L3315:
	.word	.LANCHOR1+1948
	.word	.LC0
	.word	.LANCHOR0
	.word	.LC225
	.word	.LANCHOR5-2496
	.word	.LANCHOR3
	.word	.LANCHOR3-3564
	.word	.LC226
	.word	.LANCHOR5
	.word	.LC227
	.word	.LC228
	.word	.LC229
	.word	.LC230
	.fnend
	.size	load_l2p_region, .-load_l2p_region
	.align	2
	.global	pm_gc
	.type	pm_gc, %function
pm_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	ldr	r4, .L3336
	ldr	r3, [r4, #3348]
	add	r3, r3, #688
	ldrh	r1, [r3]
	ldr	r3, .L3336+4
	ldrh	r2, [r3, #-2]
	mov	r5, r3
	sub	r2, r2, #1
	cmp	r1, r2
	bge	.L3318
	ldr	r3, [r4, #4068]
	cmp	r3, #0
	beq	.L3319
.L3318:
	bl	pm_free_sblk
	ldr	r2, [r4, #3348]
	add	r3, r2, #688
	ldrh	r1, [r3]
	ldrh	r3, [r5, #-2]
	sub	r3, r3, #1
	cmp	r1, r3
	bge	.L3320
	ldr	r3, .L3336
	ldr	r3, [r3, #4068]
	cmp	r3, #0
	beq	.L3319
.L3320:
	uxth	r0, r0
	movw	r5, #65535
	add	r0, r0, #208
	mov	r3, #0
	str	r3, [r4, #4068]
	mov	r0, r0, asl #1
	ldrh	r6, [r2, r0]
	cmp	r6, r5
	bne	.L3322
	ldr	r1, .L3336+8
	mov	r2, #162
	ldr	r0, .L3336+12
	bl	printk
	bl	dump_stack
	bl	pm_free_sblk
	ldr	r3, .L3336
	ldr	r3, [r3, #3348]
	uxth	r0, r0
	add	r0, r0, #208
	mov	r0, r0, asl #1
	ldrh	r6, [r3, r0]
	cmp	r6, r5
	beq	.L3319
.L3322:
	bl	pm_select_ram_region
	ldr	r3, .L3336+16
	movw	r2, #65535
	ldr	r9, .L3336+20
	mov	r5, r0, asl #3
	mov	r8, r0
	add	r7, r3, r5
	str	r3, [sp, #4]
	ldrh	r0, [r3, r5]
	cmp	r0, r2
	beq	.L3323
	add	r3, r9, r5
	ldr	r1, [r3, #-3560]
	cmp	r1, #0
	beq	.L3323
	ldrsh	r3, [r7, #2]
	cmp	r3, #0
	bge	.L3323
	bl	pm_write_page
	ldrh	r3, [r7, #2]
	ubfx	r3, r3, #0, #15
	strh	r3, [r7, #2]	@ movhi
.L3323:
	ldr	r10, .L3336
	mov	r7, #0
.L3324:
	ldr	r0, [r4, #3348]
	uxth	fp, r7
	add	r3, r0, #696
	add	r3, r3, #2
	ldrh	r3, [r3]
	cmp	r3, fp
	bls	.L3335
	ldr	r3, .L3336+24
	mov	r2, #1
	ldrh	r1, [r3]
	add	r3, fp, #176
	ldr	r0, [r0, r3, asl #2]
	ldrb	r3, [r10, #1700]	@ zero_extendqisi2
	rsb	r3, r3, #24
	rsb	r3, r1, r3
	mov	r3, r2, asl r3
	sub	r3, r3, #1
	and	r0, r3, r0, lsr r1
	ldrb	r1, [r10, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	uxth	r0, r0
	cmp	r0, r6
	bne	.L3325
	mov	r0, fp
	mov	r1, r8
	bl	load_l2p_region
	cmp	r0, #0
	bne	.L3326
	add	r3, r9, r5
	mov	r0, fp
	ldr	r1, [r3, #-3560]
	bl	pm_write_page
.L3326:
	ldr	r2, [sp, #4]
	mvn	r3, #0
	strh	r3, [r2, r5]	@ movhi
.L3325:
	add	r7, r7, #1
	b	.L3324
.L3335:
	bl	pm_free_sblk
.L3319:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3337:
	.align	2
.L3336:
	.word	.LANCHOR0
	.word	.LANCHOR3-3568
	.word	.LANCHOR1+1964
	.word	.LC0
	.word	.LANCHOR3-3564
	.word	.LANCHOR3
	.word	.LANCHOR0+4028
	.fnend
	.size	pm_gc, .-pm_gc
	.align	2
	.global	pm_flush_id
	.type	pm_flush_id, %function
pm_flush_id:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r0, r0, asl #3
	ldr	r4, .L3344
	ldr	r3, .L3344+4
	add	r2, r4, r0
	add	r5, r3, r0
	ldr	r1, [r2, #-3560]
	ldrh	r0, [r3, r0]
	bl	pm_write_page
	ldrh	r3, [r5, #2]
	ubfx	r3, r3, #0, #15
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, [r4, #-3568]
	cmp	r3, #0
	beq	.L3339
	bl	pm_gc
	mov	r3, #0
	str	r3, [r4, #-3568]
.L3339:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L3345:
	.align	2
.L3344:
	.word	.LANCHOR3
	.word	.LANCHOR3-3564
	.fnend
	.size	pm_flush_id, .-pm_flush_id
	.align	2
	.global	pm_flush
	.type	pm_flush, %function
pm_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	mov	r4, #0
	ldr	r5, .L3351
.L3348:
	add	r3, r5, r4, asl #3
	uxth	r0, r4
	ldrsh	r3, [r3, #2]
	cmp	r3, #0
	bge	.L3347
	bl	pm_flush_id
.L3347:
	add	r4, r4, #1
	cmp	r4, #32
	bne	.L3348
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L3352:
	.align	2
.L3351:
	.word	.LANCHOR3-3564
	.fnend
	.size	pm_flush, .-pm_flush
	.align	2
	.global	flt_sys_flush
	.type	flt_sys_flush, %function
flt_sys_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	ftl_flush
	bl	pm_flush
	bl	ftl_ext_info_flush
	mov	r0, #0
	ldmfd	sp!, {r3, lr}
	b	ftl_info_flush
	.fnend
	.size	flt_sys_flush, .-flt_sys_flush
	.align	2
	.global	zftl_deinit
	.type	zftl_deinit, %function
zftl_deinit:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
	bl	zftl_flash_de_init
	bl	flt_sys_flush
	ldmfd	sp!, {r3, lr}
	b	zftl_flash_de_init
	.fnend
	.size	zftl_deinit, .-zftl_deinit
	.align	2
	.global	pm_init
	.type	pm_init, %function
pm_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r0
	ldr	r7, .L3377
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L3377+4
	mov	r0, #64
	ldr	r6, .L3377+8
	mov	r5, #0
	mov	r3, #1
	strb	r3, [r7, #-2516]
	str	r5, [r4, #4068]
	mvn	fp, #0
	str	r5, [r6, #-3568]
	bl	ftl_malloc
	ldr	r10, .L3377+12
	mov	r3, r5
	str	r0, [r7, #-2520]
.L3359:
	cmp	r9, #0
	mov	r8, r10
	strh	fp, [r8, r5]!	@ movhi
	strh	r3, [r8, #2]	@ movhi
	beq	.L3358
	ldrb	r0, [r4, #3320]	@ zero_extendqisi2
	str	r3, [sp, #12]
	mov	r0, r0, asl #9
	bl	ftl_malloc
	ldr	r3, [sp, #12]
	str	r0, [r8, #4]
.L3358:
	add	r5, r5, #8
	cmp	r5, #256
	bne	.L3359
	ldr	r3, [r4, #3348]
	ldr	r9, [r7, #-2520]
	ldr	fp, [r6, #-3560]
	add	r3, r3, #692
	ldrb	r0, [r3, #2]	@ zero_extendqisi2
	ldrh	r1, [r3]
	mov	r3, #4
	mov	r2, fp
	str	r3, [sp]
	mov	r3, r9
	ldr	r10, .L3377+4
	bl	flash_get_last_written_page
	ldr	r7, [r4, #3348]
	add	r3, r7, #696
	ldrh	r2, [r3]
	cmp	r2, r0
	mov	r5, r0
	bgt	.L3360
	add	r7, r7, #692
	mov	r3, r5
	ldr	r0, .L3377+16
	add	r5, r5, #1
	ldrh	r1, [r7]
	bl	printk
	ldr	r3, [r10, #3348]
	add	r3, r3, #696
	ldrh	r8, [r3]
.L3361:
	sxth	r3, r8
	cmp	r3, r5
	bge	.L3376
	ldr	r2, [r4, #3348]
	ldr	r0, .L3377+20
	add	r1, r2, #692
	ldrh	r7, [r0]
	ldrh	r1, [r1]
	ldrb	r0, [r2, #694]	@ zero_extendqisi2
	mov	r2, fp
	mla	r7, r7, r1, r3
	ldrb	r3, [r4, #3320]	@ zero_extendqisi2
	str	r3, [sp]
	mov	r3, r9
	mov	r1, r7
	bl	flash_read_page_en
	ldr	r3, [r4, #3348]
	add	r3, r3, #692
	ldr	r2, [r3, #-644]
	add	r2, r2, #1
	str	r2, [r3, #-644]
	add	r3, r3, #2
	ldr	r1, [r9]
	mov	r2, r7
	ldrh	r3, [r3]
	str	r0, [sp, #12]
	ldr	r0, .L3377+24
	bl	printk
	ldr	ip, [sp, #12]
	cmp	ip, #512
	cmnne	ip, #1
	beq	.L3362
	ldr	r1, [r10, #3348]
	ldr	r3, [r9]
	add	r2, r1, #696
	add	r2, r2, #2
	ldrh	r2, [r2]
	cmp	r3, r2
	addcc	r3, r3, #176
	strcc	r7, [r1, r3, asl #2]
.L3362:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L3361
.L3376:
	ldr	r3, .L3377+28
	mov	r2, #1
	strb	r2, [r3, #-2855]
	ldr	r3, [r4, #3348]
	add	r3, r3, #696
	strh	r5, [r3]	@ movhi
	bl	pm_free_sblk
.L3360:
	ldr	r3, .L3377+32
	mov	r1, #255
	ldr	r0, [r6, #-3560]
	ldr	r4, .L3377+8
	ldrh	r2, [r3, #-4]
	bl	ftl_memset
	mvn	r0, #0
	ldr	r1, [r6, #-3560]
	bl	pm_write_page
	ldr	r3, .L3377+28
	ldrb	r3, [r3, #-2855]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3364
	ldr	r1, [r4, #-3560]
	mvn	r0, #0
	bl	pm_write_page
	ldr	r1, [r4, #-3560]
	mvn	r0, #0
	bl	pm_write_page
	mvn	r0, #0
	ldr	r1, [r4, #-3560]
	bl	pm_write_page
.L3364:
	bl	pm_free_sblk
	bl	pm_gc
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3378:
	.align	2
.L3377:
	.word	.LANCHOR5
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR3-3564
	.word	.LC231
	.word	.LANCHOR0+4076
	.word	.LC232
	.word	.LANCHOR4
	.word	.LANCHOR5-2496
	.fnend
	.size	pm_init, .-pm_init
	.align	2
	.global	pm_log2phys
	.type	pm_log2phys, %function
pm_log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r1
	ldr	r9, .L3398
	mov	r10, r0
	mov	r8, r2
	ldrb	r4, [r9, #3320]	@ zero_extendqisi2
	mov	r1, r4, asl #7
	mov	r5, r4, asl #7
	bl	__aeabi_uidiv
	ldr	r2, [r9, #3328]
	cmp	r10, r2
	movcc	r3, #0
	ldrcc	r2, .L3398+4
	uxth	fp, r0
	mov	r7, r0
	smulbb	r5, r5, fp
	rsb	r5, r5, r10
	uxth	r5, r5
	bcc	.L3380
	mov	r1, r10
	ldr	r0, .L3398+8
	bl	printk
	mvn	r0, #0
	cmp	r8, #0
	streq	r0, [r6]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3396:
	ldrh	r1, [r1, r2]
	cmp	r1, fp
	beq	.L3383
.L3382:
	add	r3, r3, #1
	cmp	r3, #32
	beq	.L3395
.L3380:
	mov	r1, r3, asl #3
	uxth	r4, r3
	add	r0, r2, r1
	ldr	r10, .L3398+4
	ldr	r0, [r0, #4]
	cmp	r0, #0
	bne	.L3396
	b	.L3382
.L3395:
	bl	pm_select_ram_region
	mov	r3, r0, asl #3
	mov	r4, r0
	add	r2, r10, r3
	ldrh	r1, [r10, r3]
	movw	r3, #65535
	cmp	r1, r3
	bne	.L3397
.L3386:
	ldr	r3, .L3398+12
	mov	r0, fp
	mov	r1, r4
	strb	r4, [r3, #-2498]
	bl	load_l2p_region
.L3383:
	cmp	r8, #0
	ldr	r3, .L3398+16
	mov	r4, r4, asl #3
	bne	.L3384
	add	r3, r3, r4
	mvn	r2, #0
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, r5, asl #2]
	str	r3, [r6]
	ldrb	r1, [r9, #1700]	@ zero_extendqisi2
	rsb	r0, r1, #24
	mvn	r2, r2, asl r1
	and	r3, r2, r3, lsr r0
	ldrb	r2, [r9, #4078]	@ zero_extendqisi2
	cmp	r3, r2
	mvncs	r3, #0
	strcs	r3, [r6]
	b	.L3385
.L3384:
	add	r2, r3, r4
	ldr	r1, [r6]
	ldr	r2, [r2, #-3560]
	str	r1, [r2, r5, asl #2]
	add	r1, r10, r4
	strb	r7, [r3, #-3308]
	ldrh	r2, [r1, #2]
	mvn	r2, r2, asl #17
	mvn	r2, r2, lsr #17
	strh	r2, [r1, #2]	@ movhi
.L3385:
	add	r4, r10, r4
	movw	r2, #32767
	mov	r0, #0
	ldrh	r3, [r4, #2]
	ubfx	r1, r3, #0, #15
	cmp	r1, r2
	addne	r3, r3, #1
	strneh	r3, [r4, #2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3397:
	ldrsh	r3, [r2, #2]
	cmp	r3, #0
	bge	.L3386
	bl	pm_flush_id
	b	.L3386
.L3399:
	.align	2
.L3398:
	.word	.LANCHOR0
	.word	.LANCHOR3-3564
	.word	.LC233
	.word	.LANCHOR5
	.word	.LANCHOR3
	.fnend
	.size	pm_log2phys, .-pm_log2phys
	.align	2
	.global	gc_recovery
	.type	gc_recovery, %function
gc_recovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #65535
	ldr	r8, .L3487
	mov	r4, #0
	.pad #68
	sub	sp, sp, #68
	ldr	r5, [r8, #580]
	strb	r4, [r8, #3369]
	strb	r4, [r8, #3370]
	ldrh	r2, [r5, #80]
	cmp	r2, r3
	beq	.L3401
	mvn	r3, #0
	mov	r0, #1
	strh	r3, [r5, #130]	@ movhi
	bl	buf_alloc
	ldrb	r3, [r5, #89]	@ zero_extendqisi2
	add	r3, r5, r3, asl #1
	ldrh	r1, [r3, #94]
	movw	r3, #4076
	ldrh	r3, [r8, r3]
	mul	r1, r3, r1
	ldrb	r3, [r8, #4032]	@ zero_extendqisi2
	cmp	r3, #2
	mov	r6, r0
	str	r1, [sp, #56]
	beq	.L3402
	ldrb	r2, [r8, #1778]	@ zero_extendqisi2
	cmp	r2, r4
	beq	.L3403
.L3402:
	movw	r2, #3986
	ldrh	r2, [r8, r2]
	sub	r2, r2, #1
	add	r2, r2, r1
	orr	r3, r2, r3, asl #24
	b	.L3481
.L3403:
	cmp	r3, #3
	bne	.L3404
	movw	r3, #3976
	ldrh	r3, [r8, r3]
	sub	r3, r3, #1
	add	r3, r3, r1
	orr	r3, r3, #50331648
.L3481:
	str	r3, [r6, #24]
.L3404:
	mov	r0, r6
	mov	r1, #1
	bl	sblk_read_page
	ldr	r7, [r6, #36]
	cmn	r7, #1
	cmpne	r7, #512
	moveq	r7, #1
	movne	r7, #0
	beq	.L3405
	ldr	r3, [r6, #12]
	ldr	r2, [r3]
	ldr	r3, .L3487+4
	cmp	r2, r3
	beq	.L3406
.L3405:
	mov	r0, r6
	mov	r4, #0
	bl	zbuf_free
	ldr	r3, [r6, #4]
	ldr	r2, [r6, #12]
	ldr	r0, .L3487+8
	ldr	r1, [r3, #4]
	str	r1, [sp]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2]
	str	r1, [sp, #12]
	ldr	r1, [r2, #4]
	str	r1, [sp, #16]
	ldr	r1, [r2, #8]
	str	r1, [sp, #20]
	ldr	r2, [r2, #12]
	str	r2, [sp, #24]
	ldr	r2, [r6, #36]
	ldr	r3, [r3]
	ldr	r1, [r6, #24]
	bl	printk
	ldrh	r3, [r5, #80]
	ldr	r2, [r8, #576]
	mov	r3, r3, asl #1
	strh	r4, [r2, r3]	@ movhi
	ldrh	r2, [r5, #80]
	ldr	r3, [r8, #580]
	b	.L3482
.L3406:
	movw	r10, #3986
	ldrb	r3, [r8, #3984]	@ zero_extendqisi2
	ldrh	r2, [r8, r10]
	ldr	r1, [r6, #4]
	ldr	r0, [r8, #3988]
	mul	r2, r3, r2
	ldr	r9, .L3487
	mov	r2, r2, asl #2
	bl	ftl_memcpy
	ldrh	r2, [r8, r10]
	ldrb	r3, [r8, #3984]	@ zero_extendqisi2
	ldrb	r1, [r8, #3320]	@ zero_extendqisi2
	mul	r3, r3, r2
	cmp	r1, r3, asr #6
	mov	r2, r3, asl #2
	bge	.L3408
	rsb	r3, r3, r1, asl #7
	ldr	r0, [r9, #3992]
	mov	r1, r7
	mov	r7, #1
	mov	r4, r3, asl #2
	bl	ftl_memset
	ldrh	r1, [r9, r10]
	ldrb	r3, [r9, #3984]	@ zero_extendqisi2
	mov	r2, r4
	ldr	r0, [r9, #3992]
	mul	r3, r3, r1
	ldr	r1, [r6, #4]
	add	r1, r1, r3, asl #2
	bl	ftl_memcpy
	b	.L3409
.L3408:
	ldr	r1, [r6, #4]
	mov	r4, r7
	ldr	r0, [r9, #3992]
	add	r1, r1, r2
	bl	ftl_memcpy
.L3409:
	movw	r3, #3976
	ldrh	r3, [r8, r3]
	str	r3, [sp, #36]
	ldrb	r3, [r8, #4032]	@ zero_extendqisi2
	cmp	r3, #2
	str	r3, [sp, #32]
	bne	.L3410
	ldr	r3, .L3487
	ldrb	r3, [r3, #4033]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3411
.L3410:
	ldrb	r3, [r8, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3412
.L3411:
	ldr	r3, [sp, #32]
	ldr	r2, [sp, #36]
	mul	r3, r3, r2
	str	r3, [sp, #36]
	mov	r3, #1
	str	r3, [sp, #32]
.L3412:
	mov	r8, #0
	ldr	r9, .L3487
	str	r8, [sp, #40]
	b	.L3413
.L3485:
	ldr	r0, [r9, #3992]
	ldr	ip, [r0, r10]
	ldr	r0, [r2, #8]
	cmp	ip, r0
	bne	.L3419
.L3446:
	ldr	r3, [sp, #40]
	add	fp, fp, #1
	add	r10, r10, #4
	add	r3, r3, #1
	str	r3, [sp, #40]
.L3423:
	ldr	r3, [sp, #32]
	cmp	fp, r3
	bhi	.L3483
	ldr	r3, [sp, #48]
	ldrb	r1, [r9, #4032]	@ zero_extendqisi2
	ldrh	r2, [r3]
	cmp	r1, #2
	ldr	r3, .L3487+12
	ldrh	r3, [r3]
	mul	r3, r3, r2
	str	r3, [sp, #56]
	bne	.L3484
.L3414:
	sub	r2, r8, #1
	add	r3, r2, r3
	add	r3, r3, fp
	orr	r3, r3, r1, asl #24
.L3417:
	str	r3, [r6, #24]
	mov	r0, r6
	mov	r1, #1
	bl	sblk_read_page
	cmp	r7, #0
	beq	.L3418
	ldr	r3, [r9, #3992]
	ldr	r2, [r3, r10]
	cmp	r2, #0
	ldreq	r2, [r6, #12]
	ldreq	r2, [r2, #8]
	streq	r2, [r3, r10]
.L3418:
	ldr	r3, [r9, #3988]
	ldr	r2, [r6, #12]
	ldr	r3, [r3, r10]
	ldr	r1, [r2, #4]
	cmp	r3, r1
	beq	.L3485
.L3419:
	ldr	r0, [r9, #3992]
	ldr	ip, .L3487
	ldr	r0, [r0, r10]
	str	ip, [sp, #52]
	str	r0, [sp]
	ldr	r0, [r2]
	stmib	sp, {r0, r1}
	ldr	r1, [r2, #8]
	ldr	r0, .L3487+16
	str	r1, [sp, #12]
	ldr	r2, [r2, #12]
	str	r2, [sp, #16]
	ldr	r1, [r6, #24]
	ldr	r2, [r6, #36]
	bl	printk
	ldr	r3, [r9, #3988]
	ldr	r3, [r3, r10]
	cmn	r3, #1
	ldr	ip, [sp, #52]
	beq	.L3446
	mov	r0, r6
	str	ip, [sp, #32]
	bl	zbuf_free
	ldrh	r3, [r5, #80]
	mov	r1, #0
	mov	r3, r3, asl #1
	ldr	ip, [sp, #32]
	ldr	r2, [ip, #576]
	strh	r1, [r2, r3]	@ movhi
	ldrh	r2, [r5, #80]
	ldr	r3, [ip, #580]
.L3482:
	strh	r2, [r3, #130]	@ movhi
	b	.L3407
.L3422:
	ldrb	r3, [r9, #4033]	@ zero_extendqisi2
	cmp	r3, #0
	addne	r8, r8, #1
	add	r8, r8, #1
.L3413:
	ldr	r3, [sp, #36]
	cmp	r8, r3
	bcs	.L3425
	add	r3, r5, #96
	str	r3, [sp, #48]
	mov	r3, #0
	str	r3, [sp, #44]
.L3426:
	ldrb	r3, [r5, #89]	@ zero_extendqisi2
	ldr	r2, [sp, #44]
	cmp	r2, r3
	bge	.L3422
	ldr	r3, [sp, #40]
	mov	fp, #1
	mov	r10, r3, asl #2
	b	.L3423
.L3484:
	ldrb	r2, [r9, #1778]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L3414
	cmp	r1, #3
	add	r3, r3, r8
	orreq	r3, r3, fp, asl #24
	b	.L3417
.L3483:
	ldr	r3, [sp, #44]
	add	r3, r3, #1
	str	r3, [sp, #44]
	ldr	r3, [sp, #48]
	add	r3, r3, #2
	str	r3, [sp, #48]
	b	.L3426
.L3425:
	mov	r0, r6
	ldr	r6, .L3487
	bl	zbuf_free
	movw	r2, #3986
	ldrb	r0, [r5, #89]	@ zero_extendqisi2
	mov	r4, #0
	ldrh	r2, [r6, r2]
	mov	r10, r4
	ldrh	r3, [r5, #80]
	mov	r8, r4
	ldr	r1, [r6, #576]
	mov	fp, r6
	smulbb	r2, r0, r2
	mov	r3, r3, asl #1
	strh	r2, [r1, r3]	@ movhi
.L3427:
	ldr	r3, [sp, #36]
	cmp	r10, r3
	bcs	.L3440
	add	r3, r5, #96
	str	r3, [sp, #44]
	mov	r3, #0
	str	r3, [sp, #40]
.L3441:
	ldrb	r2, [r5, #89]	@ zero_extendqisi2
	ldr	r3, [sp, #40]
	cmp	r3, r2
	bge	.L3437
	mov	r7, #1
.L3438:
	ldr	r3, [sp, #32]
	cmp	r7, r3
	bhi	.L3486
	ldr	r2, [r6, #3988]
	ldr	r9, [r2, r8, asl #2]
	cmn	r9, #1
	beq	.L3428
	ldr	r2, [r6, #3992]
	mov	r0, r9
	ldr	r3, [r2, r8, asl #2]
	str	r3, [sp, #48]
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #60]
	bne	.L3429
	mov	r0, r9
	add	r1, sp, #60
	mov	r2, #0
	bl	pm_log2phys
.L3429:
	ldr	r3, [sp, #44]
	ldrh	r1, [r3]
	ldr	r3, .L3487+12
	ldrh	r2, [r3]
	mul	r2, r2, r1
	ldrb	r1, [fp, #4032]	@ zero_extendqisi2
	cmp	r1, #2
	beq	.L3430
	ldrb	r0, [r6, #1778]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L3431
.L3430:
	sub	r0, r10, #1
	add	r2, r0, r2
	add	r2, r2, r7
	orr	r2, r2, r1, asl #24
	b	.L3433
.L3431:
	cmp	r1, #3
	add	r2, r2, r10
	orreq	r2, r2, r7, asl #24
.L3433:
	str	r2, [sp, #56]
	ldr	r2, .L3487+20
	ldrb	r1, [r6, #1700]	@ zero_extendqisi2
	ldr	r3, [sp, #48]
	ldrh	r0, [r2]
	rsb	r1, r1, #24
	mov	r2, #1
	str	r2, [sp, #52]
	rsb	r1, r0, r1
	mov	r1, r2, asl r1
	sub	r1, r1, #1
	and	r0, r1, r3, lsr r0
	ldrb	r1, [r6, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r1, [sp, #60]
	uxth	ip, r0
	ldr	r3, [sp, #48]
	ldr	r2, [sp, #52]
	cmp	r1, r3
	bne	.L3434
	add	r1, sp, #56
	mov	r0, r9
	str	ip, [sp, #48]
	bl	pm_log2phys
	add	r4, r4, #1
	ldr	ip, [sp, #48]
	mov	r0, ip
	bl	ftl_vpn_decrement
	ldr	ip, [sp, #48]
	b	.L3435
.L3434:
	ldr	r2, [sp, #56]
	cmp	r1, r2
	addeq	r4, r4, #1
.L3435:
	ldr	r2, [fp, #568]
	add	r8, r8, #1
	add	r2, r2, ip, asl #2
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	ands	r2, r2, #224
	bne	.L3428
	ldr	r1, [r6, #576]
	mov	ip, ip, asl #1
	ldrh	r0, [r1, ip]
	cmp	r0, #0
	strneh	r2, [r1, ip]	@ movhi
.L3428:
	add	r7, r7, #1
	b	.L3438
.L3486:
	ldr	r3, [sp, #40]
	add	r3, r3, #1
	str	r3, [sp, #40]
	ldr	r3, [sp, #44]
	add	r3, r3, #2
	str	r3, [sp, #44]
	b	.L3441
.L3437:
	ldrb	r2, [r6, #4033]	@ zero_extendqisi2
	cmp	r2, #0
	addne	r10, r10, #1
	add	r10, r10, #1
	b	.L3427
.L3440:
	ldr	r2, .L3487
	ldrh	r3, [r5, #80]
	ldr	r2, [r2, #576]
	mov	r3, r3, asl #1
	strh	r4, [r2, r3]	@ movhi
	ldrh	r0, [r5, #80]
	bl	zftl_insert_data_list
.L3407:
	ldr	r6, .L3487
	ldrh	r0, [r5, #80]
	ldr	r2, [r6, #576]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	bne	.L3442
	bl	ftl_dump_write_open_sblk
.L3442:
	ldrh	r1, [r5, #80]
	mov	r2, r4
	ldr	r0, .L3487+24
	bl	printk
	mvn	r3, #0
	strh	r3, [r5, #80]	@ movhi
	bl	pm_flush
	bl	ftl_ext_info_flush
	ldr	r3, [r6, #580]
	ldrh	r0, [r3, #130]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L3443
	bl	ftl_free_sblk
.L3443:
	ldr	r2, [r6, #3348]
	mvn	r3, #0
	mov	r0, #0
	strh	r3, [r2, #126]	@ movhi
	ldr	r2, [r6, #580]
	strh	r3, [r2, #130]	@ movhi
	bl	ftl_info_flush
	b	.L3400
.L3401:
	ldrh	r3, [r5, #130]
	cmp	r3, r2
	beq	.L3400
	ldr	r2, [r8, #3348]
	ldrh	r2, [r2, #126]
	cmp	r2, r3
	bne	.L3445
	bl	pm_flush
	ldr	r3, [r8, #580]
	ldrh	r0, [r3, #130]
	bl	ftl_free_sblk
	ldr	r3, [r8, #3348]
	mvn	r2, #0
	mov	r0, r4
	strh	r2, [r3, #126]	@ movhi
	bl	ftl_info_flush
.L3445:
	ldr	r3, [r8, #580]
	mvn	r2, #0
	strh	r2, [r3, #130]	@ movhi
.L3400:
	add	sp, sp, #68
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3488:
	.align	2
.L3487:
	.word	.LANCHOR0
	.word	-178307901
	.word	.LC234
	.word	.LANCHOR0+4076
	.word	.LC235
	.word	.LANCHOR0+4028
	.word	.LC236
	.fnend
	.size	gc_recovery, .-gc_recovery
	.align	2
	.global	zftl_init
	.type	zftl_init, %function
zftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L3564
	mvn	r3, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #28
	sub	sp, sp, #28
	ldr	r4, .L3564+4
	mov	ip, r2
	ldr	fp, .L3564+8
	mov	r7, #0
	ldr	r1, .L3564+12
	ldr	r0, .L3564+16
	str	r3, [ip, #-2496]!
	strb	r3, [r4, #3368]
	str	ip, [sp, #8]
	str	ip, [sp, #4]
	strb	r3, [fp, #-3573]
	strb	r3, [fp, #-3255]
	str	r2, [sp, #12]
	strb	r7, [r4, #3344]
	strb	r7, [fp, #-3572]
	strb	r7, [r4, #3345]
	bl	printk
	ldr	r6, .L3564+20
	movw	r1, #3986
	ldrb	r3, [r6, #16]	@ zero_extendqisi2
	ldrh	r8, [r6, #14]
	ldrb	r10, [r6, #13]	@ zero_extendqisi2
	strh	r8, [r4, r1]	@ movhi
	mov	r0, r8
	strb	r10, [r4, #3320]
	mov	r9, r10, asl #9
	uxth	r9, r9
	str	r3, [sp, #4]
	ldrb	r3, [sp, #4]	@ zero_extendqisi2
	ldr	r1, [sp, #4]
	strb	r3, [r4, #4032]
	bl	__aeabi_idiv
	movw	r1, #3976
	strh	r0, [r4, r1]	@ movhi
	mov	r3, r0
	ldrb	r1, [r4, #585]	@ zero_extendqisi2
	ldrb	r0, [r6, #17]	@ zero_extendqisi2
	ldrh	r6, [r6, #18]
	strb	r1, [r4, #4078]
	smulbb	r1, r0, r1
	strb	r0, [r4, #4030]
	ldrh	r0, [r4, #2]
	ldr	ip, [sp, #8]
	str	fp, [sp, #8]
	uxtb	r5, r1
	add	r1, r4, #564
	strb	r5, [r4, #3984]
	strh	r6, [r1]	@ movhi
	movw	r1, #4076
	strh	r0, [r4, r1]	@ movhi
	mov	r1, #1
	strh	r9, [ip, #-4]	@ movhi
.L3490:
	cmp	r1, r0
	uxth	r2, r7
	add	r7, r7, #1
	movls	r1, r1, asl #1
	bls	.L3490
.L3562:
	movw	r1, #4028
	sub	r2, r2, #1
	strh	r2, [r4, r1]	@ movhi
	mov	ip, #0
	mul	r2, r8, r10
	mov	r0, #1
	mul	r1, r6, r2
	mov	lr, r1, lsr #21
.L3492:
	cmp	r0, lr
	uxth	r7, ip
	add	ip, ip, #1
	movls	r0, r0, asl #1
	bls	.L3492
.L3563:
	ldr	r0, .L3564+24
	sub	r7, r7, #1
	mov	fp, #1
	str	r3, [sp, #16]
	uxth	r7, r7
	ldr	r3, [sp, #12]
	mul	r0, r0, r5
	mov	fp, fp, asl r7
	mul	r1, r5, r1
	str	r2, [sp, #20]
	ldr	r7, .L3564+4
	mul	fp, fp, r0
	str	r1, [r4, #3324]
	mov	r1, r10
	ldr	r10, .L3564+28
	add	r8, fp, #24576
	str	r8, [r3, #-2492]
	mov	r0, r8
	str	fp, [r4, #520]
	bl	__aeabi_uidiv
	sub	r1, r9, #1
	str	r0, [r4, #3328]
	add	r0, r1, r0, asl #2
	mov	r1, r9
	ldr	r9, .L3564+32
	bl	__aeabi_uidiv
	ldr	r3, [sp, #16]
	strh	r0, [r10, #-8]	@ movhi
	uxth	r0, r0
	mul	r1, r5, r3
	mov	r0, r0, asl #4
	bl	__aeabi_idiv
	add	ip, r7, #4048
	ldr	r2, [sp, #20]
	strh	r0, [r9, #-2]	@ movhi
	mov	r0, r8
	str	ip, [sp, #16]
	mul	r1, r5, r2
	sub	r1, r1, #1
	bl	__aeabi_uidiv
	add	r2, r6, r6, asl #1
	mov	r6, r6, lsr #4
	ldr	lr, .L3564+32
	cmp	r6, #79
	mov	r2, r2, asr #2
	strh	r2, [r9, #-10]	@ movhi
	movw	r2, #4008
	movls	r1, #80
	strh	r6, [r4, r2]	@ movhi
	ldr	ip, [sp, #16]
	add	r0, r0, #8
	strlsh	r1, [r7, r2]	@ movhi
	mov	r2, #2000
	ldr	r3, [sp, #4]
	movw	r1, #4072
	strh	r2, [r9, #-8]	@ movhi
	mov	r2, #50
	strh	r2, [r9, #-12]	@ movhi
	mov	r5, ip
	ldr	r2, .L3564+4
	cmp	r3, #2
	strh	r0, [ip]	@ movhi
	movw	ip, #4074
	mov	r0, #256
	strh	r0, [r4, r1]	@ movhi
	mov	r1, #48
	strh	r1, [r4, ip]	@ movhi
	add	r1, r2, #572
	mov	r0, #32
	strh	r0, [r1]	@ movhi
	beq	.L3495
	ldrb	r3, [r2, #4035]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3496
.L3495:
	movw	r2, #4074
	mov	r0, #64
	strh	r0, [r4, r2]	@ movhi
	mov	r0, #12
	strh	r0, [r1]	@ movhi
	mov	r3, #150
	ldrb	r1, [r4]	@ zero_extendqisi2
	strh	r3, [r9, #-12]	@ movhi
	cmp	r1, #0
	ldr	r3, .L3564+4
	add	r2, r3, #572
	bne	.L3498
	mov	r1, #4
	strh	r1, [r2]	@ movhi
	ldr	r2, .L3564+32
	mov	r1, #600
	strh	r1, [r2, #-8]	@ movhi
	movw	r2, #4072
	mov	r1, #128
	strh	r1, [r3, r2]	@ movhi
	b	.L3498
.L3496:
	ldrb	r3, [r2, #1778]	@ zero_extendqisi2
	cmp	r3, #0
	strneh	r0, [lr, #-12]	@ movhi
	strneh	r0, [r2, ip]	@ movhi
	movne	r3, #1200
	strneh	r3, [lr, #-8]	@ movhi
.L3498:
	ldr	r6, .L3564+20
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3500
	ldr	r0, .L3564+36
	mov	r1, fp
	bl	printk
.L3500:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3501
	ldr	r0, .L3564+40
	ldr	r1, [r4, #3324]
	bl	printk
.L3501:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3502
	ldr	r0, .L3564+44
	ldr	r1, [r4, #3328]
	bl	printk
.L3502:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3503
	ldr	r3, [sp, #12]
	ldr	r0, .L3564+48
	ldr	r1, [r3, #-2492]
	bl	printk
.L3503:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3504
	ldr	r0, .L3564+52
	ldrh	r1, [r10, #-8]
	bl	printk
.L3504:
	ldr	r3, [r6]
	ldr	r7, .L3564+56
	tst	r3, #4096
	beq	.L3505
	ldr	r0, .L3564+60
	ldrh	r1, [r7, #-4]
	bl	printk
.L3505:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3506
	ldr	r0, .L3564+64
	ldrh	r1, [r9, #-2]
	bl	printk
.L3506:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3507
	movw	r3, #4008
	ldr	r0, .L3564+68
	ldrh	r1, [r4, r3]
	bl	printk
.L3507:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3508
	ldr	r0, .L3564+72
	ldrh	r1, [r5]
	bl	printk
.L3508:
	ldr	r5, .L3564+76
	bl	zbuf_init
	mov	r0, #6
	movw	r8, #3976
	ldrh	r3, [r5]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	str	r0, [r4, #524]
	ldrh	r0, [r4, r8]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldr	r2, .L3564+80
	ldrh	r1, [r4, r8]
	ldr	r3, [sp, #8]
	str	r0, [r3, #-3588]
	str	r2, [r3, #-3592]
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	ldrh	r0, [r5]
	mul	r1, r3, r1
	mov	r0, r0, lsr #1
	add	r3, r2, r1, asl #3
	add	r1, r0, r1, asl #1
	ldr	r0, [r6]
	add	r1, r2, r1, asl #2
	str	r3, [r4, #576]
	tst	r0, #4096
	str	r1, [r4, #580]
	beq	.L3509
	ldr	r0, .L3564+84
	bl	printk
.L3509:
	movw	r3, #3976
	ldrh	r2, [r5]
	ldrh	r5, [r4, r3]
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	ldrh	r10, [r10, #-8]
	mul	r5, r3, r5
	ldr	r3, [r6]
	mov	r10, r10, asl #2
	tst	r3, #4096
	add	r10, r10, r2, asl #2
	add	r10, r10, #704
	add	r5, r2, r5, asl #2
	mov	r5, r5, asl #1
	add	r5, r5, #632
	beq	.L3510
	ldr	r0, .L3564+88
	mov	r1, r5
	mov	r2, r10
	ldrh	r3, [r7, #-4]
	bl	printk
.L3510:
	ldrh	r3, [r7, #-4]
	cmp	r10, r3
	cmpls	r5, r3
	movhi	r10, #1
	movls	r10, #0
	bls	.L3511
.L3560:
	b	.L3560
.L3511:
	bl	sblk_init
	bl	ftl_info_blk_init
	cmn	r0, #1
	beq	.L3513
	bl	ftl_ext_info_init
	bl	gc_init
	mov	r0, #1
	bl	pm_init
	bl	lpa_rebuild_hash
	ldr	r0, [r4, #580]
	mov	r1, r10
	add	r0, r0, #16
	bl	ftl_open_sblk_recovery
	ldr	r1, [r4, #580]
	add	r0, r1, #48
	add	r1, r1, #16
	bl	ftl_open_sblk_recovery
	ldr	r2, [r4, #3348]
	ldr	r0, [r4, #580]
	ldr	r3, [r2, #8]
	add	r0, r0, #16
	add	r3, r3, #16
	str	r3, [r2, #8]
	bl	ftl_info_data_recovery
	ldr	r0, [r4, #580]
	add	r0, r0, #48
	bl	ftl_info_data_recovery
	ldr	r0, [r4, #580]
	add	r0, r0, #80
	bl	ftl_info_data_recovery
	bl	gc_recovery
	bl	pm_flush
	mov	r0, #1
	bl	ftl_total_vpn_update
	ldr	r3, .L3564+92
	ldrb	r3, [r3, #-2855]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, .L3564+4
	ldrne	r2, [r3, #3348]
	ldrne	r3, [r2, #68]
	addne	r3, r3, #1
	strne	r3, [r2, #68]
	bl	ftl_ext_info_flush
	mov	r0, #0
	bl	ftl_info_flush
	bl	print_ftl_debug_info
	mov	r0, #0
.L3513:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3565:
	.align	2
.L3564:
	.word	.LANCHOR5
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC2
	.word	.LC1
	.word	.LANCHOR2
	.word	1892352
	.word	.LANCHOR5-2480
	.word	.LANCHOR3-3568
	.word	.LC237
	.word	.LC238
	.word	.LC239
	.word	.LC240
	.word	.LC241
	.word	.LANCHOR5-2496
	.word	.LC242
	.word	.LC243
	.word	.LC244
	.word	.LC245
	.word	.LANCHOR0+564
	.word	.LANCHOR4-2792
	.word	.LC246
	.word	.LC247
	.word	.LANCHOR4
	.fnend
	.size	zftl_init, .-zftl_init
	.align	2
	.global	rk_ftl_init
	.type	rk_ftl_init, %function
rk_ftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	mov	r0, #68
	ldr	r3, .L3582
	mov	r7, #0
	ldr	r4, .L3582+4
	ldr	r5, .L3582+8
	ldr	r3, [r3]
	str	r7, [r4, #-3300]
	strb	r7, [r5, #-2486]
	str	r3, [r4, #-3304]
	bl	ftl_malloc
	cmp	r0, r7
	str	r0, [r4, #-3296]
	bne	.L3567
.L3569:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3567:
	mov	r0, #2048
	bl	ftl_malloc
	mov	r1, r5
	str	r7, [r1, #-2480]!
	str	r7, [r4, #-3292]
	str	r0, [r5, #-2484]
	ldr	r0, .L3582+12
	bl	rknand_get_reg_addr
	ldr	r3, [r4, #-3292]
	cmp	r3, #0
	beq	.L3569
	bl	rk_nandc_irq_init
	mov	r1, r7
	mov	r2, r7
	mov	r3, #2048
	ldr	r0, [r5, #-2484]
	bl	flash_sram_load_store
	bl	rknand_flash_cs_init
	ldr	r3, [r4, #-3296]
	ldr	r2, .L3582+16
	ldr	r0, [r4, #-3292]
	str	r2, [r3, #40]
	ldr	r2, .L3582+20
	str	r2, [r3, #32]
	ldr	r2, .L3582+24
	str	r2, [r3, #44]
	ldr	r2, .L3582+28
	str	r2, [r3, #48]
	ldr	r2, .L3582+32
	str	r2, [r3, #36]
	ldr	r2, .L3582+36
	str	r2, [r3, #56]
	ldr	r2, .L3582+40
	str	r2, [r3, #20]
	ldr	r2, .L3582+44
	str	r2, [r3, #24]
	ldr	r2, .L3582+48
	str	r2, [r3, #12]
	ldr	r2, .L3582+52
	str	r2, [r3, #16]
	ldr	r2, .L3582+56
	str	r2, [r3, #4]
	ldr	r2, .L3582+60
	str	r2, [r3, #8]
	ldr	r2, .L3582+64
	str	r2, [r3, #60]
	ldr	r2, .L3582+68
	str	r2, [r3, #64]
	ldr	r2, .L3582+72
	str	r2, [r3, #52]
	ldr	r2, .L3582+76
	str	r2, [r3, #28]
	bl	nand_flash_init
	subs	r7, r0, #0
	bne	.L3570
	bl	zftl_init
	mov	r4, r0
	bl	zftl_proc_debug_init
	mov	r3, #1
	strb	r3, [r5, #-2486]
	b	.L3571
.L3570:
	ldr	r3, [r4, #-3296]
	mov	r6, r4
	ldr	r2, .L3582+80
	ldr	r0, [r4, #-3292]
	ldr	r5, .L3582+84
	str	r2, [r3, #40]
	ldr	r2, .L3582+88
	str	r2, [r3, #32]
	ldr	r2, .L3582+92
	str	r2, [r3, #44]
	ldr	r2, .L3582+96
	str	r2, [r3, #48]
	ldr	r2, .L3582+100
	str	r2, [r3, #36]
	ldr	r2, .L3582+104
	str	r2, [r3, #56]
	ldr	r2, .L3582+108
	str	r2, [r3, #20]
	ldr	r2, .L3582+112
	str	r2, [r3, #24]
	ldr	r2, .L3582+116
	str	r2, [r3, #12]
	ldr	r2, .L3582+120
	str	r2, [r3, #16]
	ldr	r2, .L3582+124
	str	r2, [r3, #4]
	ldr	r2, .L3582+128
	str	r2, [r3, #8]
	ldr	r2, .L3582+132
	str	r2, [r3, #60]
	ldr	r2, .L3582+136
	str	r2, [r3, #64]
	ldr	r2, .L3582+140
	str	r2, [r3, #52]
	ldr	r2, .L3582+144
	str	r2, [r3, #28]
	bl	FlashInit
	cmn	r7, #2
	mov	r4, r0
	bne	.L3572
	ldr	r1, .L3582+148
	mov	r2, #32
	add	r0, r5, #4
	bl	ftl_memcpy
	ldrb	r0, [r5, #22]	@ zero_extendqisi2
	bl	flash_lsb_page_tbl_build
	ldrh	r3, [r5, #14]
	strh	r3, [r5, #30]	@ movhi
.L3572:
	ldr	r3, .L3582+152
	ldr	r2, [r3]
	ldr	r3, .L3582+156
	cmp	r2, r3
	ldr	r3, .L3582+160
	bne	.L3573
	ldr	r2, .L3582+164
	ldrb	r2, [r2]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L3574
.L3573:
	mov	r2, #0
	strb	r2, [r3, #1625]
	strb	r2, [r3]
.L3574:
	ldr	r1, .L3582+168
	mov	r0, #1
	ldrh	ip, [r5, #14]
	mov	r2, #0
	str	r2, [r6, #-3268]
	ldrb	r1, [r1]	@ zero_extendqisi2
	strb	r1, [r3, #1624]
	ldr	r1, .L3582+172
	ldrb	r1, [r1]	@ zero_extendqisi2
	strb	r1, [r3, #584]
.L3575:
	cmp	r0, ip
	uxth	r1, r2
	add	r2, r2, #1
	movls	r0, r0, asl #1
	bls	.L3575
.L3581:
	movw	r2, #4028
	sub	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #0
	cmp	r4, r2
	strb	r2, [r3, #1658]
	bne	.L3571
	bl	FtlInit
	mov	r4, r0
.L3571:
	mov	r1, r4
	ldr	r0, .L3582+176
	bl	printk
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3583:
	.align	2
.L3582:
	.word	jiffies
	.word	.LANCHOR3
	.word	.LANCHOR5
	.word	.LANCHOR3-3292
	.word	zftl_deinit
	.word	zftl_cache_flush
	.word	zftl_flash_suspend
	.word	zftl_flash_resume
	.word	zftl_get_density
	.word	zftl_read_flash_info
	.word	zftl_read
	.word	zftl_write
	.word	zftl_sys_read
	.word	zftl_sys_write
	.word	zftl_vendor_read
	.word	zftl_vendor_write
	.word	zftl_nandc_get_irq_status
	.word	zftl_proc_ftl_read
	.word	zftl_do_gc
	.word	zftl_discard
	.word	ftl_deinit
	.word	.LANCHOR2
	.word	ftl_cache_flush
	.word	ftl_flash_suspend
	.word	ftl_flash_resume
	.word	ftl_get_density
	.word	ftl_read_flash_info
	.word	ftl_read
	.word	ftl_write
	.word	ftl_sys_read
	.word	ftl_sys_write
	.word	ftl_vendor_read
	.word	ftl_vendor_write
	.word	ftl_nandc_get_irq_status
	.word	ftl_proc_ftl_read
	.word	ftl_do_gc
	.word	ftl_discard
	.word	gNandParaInfo
	.word	g_nandc_version_data
	.word	1446522928
	.word	.LANCHOR0
	.word	gFlashSlcMode
	.word	gNandFlashIDBEccBits
	.word	gNandFlashEccBits
	.word	.LC248
	.fnend
	.size	rk_ftl_init, .-rk_ftl_init
	.align	2
	.global	gc_update_l2p_map_new
	.type	gc_update_l2p_map_new, %function
gc_update_l2p_map_new:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #3986
	ldr	r4, .L3620
	.pad #28
	sub	sp, sp, #28
	ldr	r5, [r4, #580]
	ldrh	r3, [r4, r3]
	ldrb	r2, [r5, #89]	@ zero_extendqisi2
	mul	r3, r2, r3
	str	r3, [sp]
	bl	timer_get_time
	ldr	r3, .L3620+4
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3585
	ldr	r0, .L3620+8
	ldrh	r1, [r5, #80]
	bl	printk
.L3585:
	ldrh	r3, [r5, #80]
	mov	r7, #0
	ldr	r2, [r4, #576]
	mov	r6, r7
	ldr	r9, .L3620
	ldr	fp, .L3620+4
	mov	r3, r3, asl #1
	ldr	r1, [sp]
	sub	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L3586:
	ldr	r3, [sp]
	cmp	r6, r3
	beq	.L3619
	ldr	r3, [r4, #3988]
	mov	r8, r6, asl #2
	ldr	r10, [r3, r6, asl #2]
	cmn	r10, #1
	beq	.L3587
	ldrb	r1, [r9, #3320]	@ zero_extendqisi2
	mov	r0, r10
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	uxth	r3, r0
	str	r3, [sp, #12]
	ldr	r3, [fp]
	tst	r3, #256
	beq	.L3588
	ldr	r0, .L3620+12
	mov	r2, r10
	ldr	r1, [sp, #12]
	mov	r3, r6
	bl	printk
.L3588:
	str	r6, [sp, #8]
.L3594:
	ldr	r3, [r4, #3988]
	ldr	r10, [r3, r8]
	cmn	r10, #1
	beq	.L3589
	ldrb	r1, [r9, #3320]	@ zero_extendqisi2
	mov	r0, r10
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	ldr	r3, [sp, #12]
	uxth	r0, r0
	cmp	r0, r3
	bne	.L3589
	ldr	r3, [r9, #3992]
	mov	r0, r10
	ldr	r3, [r3, r8]
	str	r3, [sp, #4]
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #20]
	bne	.L3590
	mov	r0, r10
	add	r1, sp, #20
	mov	r2, #0
	bl	pm_log2phys
.L3590:
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #4]
	cmp	r3, r2
	bne	.L3591
	ldr	r3, [r4, #4004]
	add	r1, sp, #24
	mov	r0, r10
	mov	r2, #1
	add	r7, r7, #1
	ldr	r3, [r3, r8]
	str	r3, [r1, #-8]!
	bl	pm_log2phys
	ldr	r3, .L3620+16
	ldrb	r1, [r4, #4030]	@ zero_extendqisi2
	ldrh	r0, [r3]
	ldrb	r3, [r4, #1700]	@ zero_extendqisi2
	rsb	r3, r3, #24
	rsb	r3, r0, r3
	ldr	r2, [sp, #4]
	mov	r10, r2, lsr r0
	mvn	r0, #0
	bic	r0, r10, r0, asl r3
	bl	__aeabi_uidiv
	uxth	r0, r0
	b	.L3618
.L3591:
	ldr	r2, [fp]
	tst	r2, #256
	beq	.L3593
	ldr	r0, .L3620+20
	mov	r1, r10
	ldr	r2, [sp, #4]
	bl	printk
.L3593:
	ldrh	r0, [r5, #80]
.L3618:
	bl	ftl_vpn_decrement
	ldr	r3, [r4, #3988]
	mvn	r2, #0
	str	r2, [r3, r8]
	bl	timer_get_time
.L3589:
	ldr	r3, [sp, #8]
	add	r8, r8, #4
	ldr	r2, [sp]
	add	r3, r3, #1
	str	r3, [sp, #8]
	cmp	r3, r2
	bne	.L3594
.L3587:
	add	r6, r6, #1
	b	.L3586
.L3619:
	ldr	r3, .L3620+4
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3596
	ldrh	r3, [r5, #80]
	ldr	r1, [r4, #580]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	ldr	r0, .L3620+24
	ldrh	r1, [r1, #80]
	ldrh	r2, [r2, r3]
	mov	r3, r7
	bl	printk
.L3596:
	ldrh	r3, [r5, #80]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, r7
	beq	.L3597
	ldr	r1, .L3620+28
	movw	r2, #749
	ldr	r0, .L3620+32
	bl	printk
	bl	dump_stack
.L3597:
	ldrh	r3, [r5, #80]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	strh	r7, [r2, r3]	@ movhi
	ldrh	r0, [r5, #80]
	bl	zftl_insert_data_list
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3621:
	.align	2
.L3620:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC249
	.word	.LC250
	.word	.LANCHOR0+4028
	.word	.LC251
	.word	.LC252
	.word	.LANCHOR1+1972
	.word	.LC0
	.fnend
	.size	gc_update_l2p_map_new, .-gc_update_l2p_map_new
	.align	2
	.global	gc_scan_src_blk_one_page
	.type	gc_scan_src_blk_one_page, %function
gc_scan_src_blk_one_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r1, #0
	ldr	r5, .L3652
	mov	r0, r1
	ldr	r4, .L3652+4
	movw	ip, #65535
	ldrb	r3, [r5, #3384]	@ zero_extendqisi2
	ldrb	lr, [r5, #3984]	@ zero_extendqisi2
	add	r2, r4, r3, asl #1
	ldrh	r6, [r2, #36]
	ldrh	r2, [r4, #2]
.L3623:
	cmp	r6, ip
	bne	.L3650
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, lr
	addeq	r2, r2, #1
	moveq	r3, #0
	moveq	r0, #1
	add	r1, r4, r3, asl #1
	uxtheq	r2, r2
	ldrh	r6, [r1, #36]
	mov	r1, #1
	b	.L3623
.L3650:
	cmp	r0, #0
	mov	r0, #1
	strneh	r2, [r4, #2]	@ movhi
	cmp	r1, #0
	strneb	r3, [r5, #3384]
	bl	buf_alloc
	ldr	r7, .L3652
	mov	r10, #1
	ldr	r8, .L3652+8
	mov	r9, r0
.L3628:
	ldrb	r2, [r5, #3386]	@ zero_extendqisi2
	ldr	fp, .L3652
	cmp	r10, r2
	bgt	.L3651
	ldrh	r3, [r8]
	cmp	r2, #2
	mul	r3, r3, r6
	beq	.L3629
	cmp	r2, #3
	ldrneh	r2, [r4, #2]
	addne	r3, r3, r2
	bne	.L3649
	ldrb	r2, [r7, #1778]	@ zero_extendqisi2
	cmp	r2, #0
	ldreqh	r2, [r4, #2]
	addeq	r3, r3, r2
	orreq	r3, r3, r10, asl #24
	beq	.L3649
.L3629:
	ldrh	r2, [r4, #2]
	add	r3, r3, r2, asl #1
	ldrb	r2, [r5, #4032]	@ zero_extendqisi2
	sub	r3, r3, #1
	add	r3, r3, r10
	orr	r3, r3, r2, asl #24
.L3649:
	str	r3, [r9, #24]
	mov	r0, r9
	mov	r1, #1
	bl	sblk_read_page
	ldr	r3, [r9, #36]
	cmp	r3, #512
	cmnne	r3, #1
	beq	.L3634
	ldr	r3, [r9, #12]
	ldr	fp, [r3, #4]
	mov	r0, fp
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #4]
	bne	.L3635
	ldr	r3, [r5, #3328]
	cmp	fp, r3
	bcs	.L3635
	mov	r0, fp
	add	r1, sp, #4
	mov	r2, #0
	bl	pm_log2phys
.L3635:
	ldr	r3, [r9, #24]
	ldr	r2, [sp, #4]
	cmp	r3, r2
	ldreqh	r1, [r4, #20]
	ldreq	r2, [r5, #4000]
	streq	r3, [r2, r1, asl #2]
	ldreqh	r3, [r4, #20]
	addeq	r3, r3, #1
	streqh	r3, [r4, #20]	@ movhi
.L3634:
	ldrh	r3, [r4, #22]
	add	r10, r10, #1
	add	r3, r3, #1
	strh	r3, [r4, #22]	@ movhi
	b	.L3628
.L3651:
	mov	r0, r9
	bl	zbuf_free
	ldrb	r3, [fp, #3384]	@ zero_extendqisi2
	ldrb	r2, [fp, #3984]	@ zero_extendqisi2
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [fp, #3384]
	cmp	r2, r3
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [fp, #3384]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3653:
	.align	2
.L3652:
	.word	.LANCHOR0
	.word	.LANCHOR0+3380
	.word	.LANCHOR0+4076
	.fnend
	.size	gc_scan_src_blk_one_page, .-gc_scan_src_blk_one_page
	.align	2
	.global	gc_scan_src_blk
	.type	gc_scan_src_blk, %function
gc_scan_src_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	ip, .L3689
	movw	r3, #3380
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r2, [ip, r3]
	movw	r3, #65535
	mov	fp, ip
	cmp	r2, r3
	bne	.L3655
	ldr	r1, .L3689+4
	movw	r2, #1236
	ldr	r0, .L3689+8
	bl	printk
	bl	dump_stack
.L3655:
	movw	r3, #3380
	ldrh	r1, [fp, r3]
	movw	r3, #65535
	cmp	r1, r3
	moveq	r0, #0
	beq	.L3656
	ldr	r3, .L3689+12
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3657
	ldr	r3, .L3689
	ldr	r0, .L3689+16
	ldr	r2, [r3, #576]
	mov	r3, r1, asl #1
	ldrh	r2, [r2, r3]
	bl	printk
.L3657:
	movw	r6, #3380
	bl	timer_get_time
	ldrh	r0, [fp, r6]
	ldr	r1, .L3689+20
	bl	ftl_get_blk_list_in_sblk
	ldr	r5, .L3689
	ldr	r4, .L3689+24
	uxtb	r0, r0
	strb	r0, [fp, #3385]
	cmp	r0, #0
	mvneq	r3, #0
	streqh	r3, [r5, r6]	@ movhi
	beq	.L3656
	ldrh	r3, [r5, r6]
	ldr	r2, [r5, #568]
	add	r2, r2, r3, asl #2
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	tst	r2, #192
	and	r1, r2, #224
	moveq	r2, #1
	movne	r2, #0
	cmp	r1, #224
	orreq	r2, r2, #1
	cmp	r2, #0
	bne	.L3659
	ldr	r2, [r5, #580]
	ldrh	lr, [r2, #16]
	cmp	lr, r3
	beq	.L3659
	ldrh	lr, [r2, #48]
	cmp	lr, r3
	beq	.L3659
	ldrh	r2, [r2, #80]
	cmp	r2, r3
	bne	.L3660
.L3659:
	cmp	r1, #0
	bne	.L3661
	ldr	r2, [fp, #576]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3662
	ldr	r1, .L3689+4
	movw	r2, #1256
	ldr	r0, .L3689+8
	bl	printk
	bl	dump_stack
.L3662:
	movw	r3, #3380
	ldr	r2, [fp, #576]
	ldrh	r3, [fp, r3]
	mov	r1, #0
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
.L3661:
	movw	r3, #3380
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [fp, r3]	@ movhi
	strh	r0, [r4, #20]	@ movhi
	b	.L3656
.L3660:
	add	r0, r4, r0, asl #1
	movw	r3, #65535
	ldrh	r5, [r0, #34]
	cmp	r5, r3
	bne	.L3663
	ldr	r1, .L3689+4
	mov	r2, #1264
	ldr	r0, .L3689+8
	bl	printk
	bl	dump_stack
.L3663:
	movw	r3, #3976
	ldr	r2, .L3689
	ldrh	r3, [fp, r3]
	sub	r3, r3, #1
	uxth	r6, r3
	movw	r3, #4076
	ldrh	r1, [fp, r3]
	movw	r3, #3380
	mul	r5, r1, r5
	ldrh	r1, [fp, r3]
	ldr	r3, [fp, #568]
	add	r3, r3, r1, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #160
	movne	r8, #1
	bne	.L3664
	ldrb	r3, [r2, #4032]	@ zero_extendqisi2
	cmp	r3, #2
	uxth	r8, r3
	orr	r5, r5, r3, asl #24
	movweq	r3, #3986
	ldreqh	r6, [r2, r3]
	ldrb	r3, [fp, #1778]	@ zero_extendqisi2
	subeq	r6, r6, #1
	uxtheq	r6, r6
	cmp	r3, #0
	ldrne	r2, .L3689
	movwne	r3, #3986
	ldrneh	r6, [r2, r3]
	subne	r6, r6, #1
	uxthne	r6, r6
.L3664:
	mov	r0, #1
	mov	r7, #0
	strb	r8, [fp, #3386]
	orr	r5, r6, r5
	strh	r7, [r4, #2]	@ movhi
	strb	r7, [fp, #3384]
	strh	r7, [r4, #22]	@ movhi
	strh	r7, [r4, #24]	@ movhi
	bl	buf_alloc
	mov	r1, #1
	ldr	r9, .L3689
	str	r5, [r0, #24]
	mov	r6, r0
	bl	sblk_read_page
	movw	r3, #3986
	ldrh	r2, [fp, r3]
	mov	r1, #255
	ldrb	r3, [fp, #3984]	@ zero_extendqisi2
	ldr	r0, [fp, #4000]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r5, [r6, #36]
	strh	r7, [r4, #20]	@ movhi
	cmn	r5, #1
	cmpne	r5, #512
	moveq	r5, #1
	movne	r5, #0
	beq	.L3687
	ldr	r3, [r6, #12]
	ldr	r2, [r3]
	ldr	r3, .L3689+28
	cmp	r2, r3
	beq	.L3667
	ldr	r1, .L3689+4
	movw	r2, #1299
	ldr	r0, .L3689+8
	bl	printk
	bl	dump_stack
.L3687:
	mov	r0, r6
	bl	zbuf_free
	mvn	r0, #0
	b	.L3656
.L3667:
	movw	r3, #3976
	ldrb	r7, [r9, #3984]	@ zero_extendqisi2
	ldrh	r3, [r9, r3]
	ldr	r10, .L3689+24
	smulbb	r7, r7, r3
	smulbb	r7, r7, r8
	ldr	r8, [r6, #4]
	sub	r8, r8, #4
	uxth	r7, r7
.L3668:
	cmp	r5, r7
	bge	.L3688
	ldr	r0, [r8, #4]!
	cmn	r0, #1
	beq	.L3670
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #12]
	bne	.L3671
	ldr	r0, [r8]
	add	r1, sp, #12
	mov	r2, #0
	bl	pm_log2phys
.L3671:
	ldr	r3, .L3689+32
	mvn	r1, #0
	ldr	r2, [sp, #12]
	ldrh	r0, [r3]
	ldrb	r3, [fp, #1700]	@ zero_extendqisi2
	str	r2, [sp, #4]
	rsb	r3, r3, #24
	rsb	r3, r0, r3
	mov	r0, r2, lsr r0
	bic	r0, r0, r1, asl r3
	ldrb	r1, [fp, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldrh	r3, [r4]
	cmp	r0, r3
	ldr	r2, [sp, #4]
	ldreq	r3, [r9, #4000]
	ldreqh	r1, [r10, #20]
	streq	r2, [r3, r1, asl #2]
	ldreqh	r3, [r10, #20]
	addeq	r3, r3, #1
	streqh	r3, [r10, #20]	@ movhi
.L3670:
	bl	timer_get_time
	add	r5, r5, #1
	b	.L3668
.L3688:
	mov	r0, r6
	bl	zbuf_free
	movw	r3, #3380
	ldrh	r1, [fp, r3]
	ldr	r2, [fp, #576]
	mov	r3, r1, asl #1
	ldrh	r2, [r2, r3]
	ldrh	r3, [r4, #20]
	cmp	r2, r3
	beq	.L3674
	ldr	r0, .L3689+36
	bl	printk
.L3674:
	movw	r3, #3380
	ldrh	r1, [r4, #20]
	ldrh	r3, [fp, r3]
	ldr	r2, [fp, #576]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	mov	r3, #0
	ldrh	r0, [r4, #20]
	strh	r3, [r4, #24]	@ movhi
.L3656:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3690:
	.align	2
.L3689:
	.word	.LANCHOR0
	.word	.LANCHOR1+1996
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC253
	.word	.LANCHOR0+3416
	.word	.LANCHOR0+3380
	.word	-178307901
	.word	.LANCHOR0+4028
	.word	.LC254
	.fnend
	.size	gc_scan_src_blk, .-gc_scan_src_blk
	.align	2
	.global	gc_scan_static_data
	.type	gc_scan_static_data, %function
gc_scan_static_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	ldr	r4, .L3708
	ldr	r3, [r4, #580]
	ldr	r2, [r3, #544]
	cmn	r2, #1
	beq	.L3692
	mov	r5, #11
	mov	r6, r4
.L3693:
	ldr	r3, [r4, #580]
	add	r1, sp, #4
	mov	r2, #0
	ldr	r0, [r3, #544]
	bl	pm_log2phys
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L3694
	mov	r0, #1
	bl	buf_alloc
	mov	r1, #1
	ldr	r3, [sp, #4]
	mov	r7, r0
	str	r3, [r0, #24]
	bl	sblk_read_page
	ldr	r3, [r7, #36]
	cmp	r3, #256
	bne	.L3695
	ldr	r3, .L3708+4
	mov	r8, #1
	ldr	r0, [sp, #4]
	ldrb	r1, [r6, #4030]	@ zero_extendqisi2
	ldrh	r2, [r3]
	ldrb	r3, [r6, #1700]	@ zero_extendqisi2
	rsb	r3, r3, #24
	rsb	r3, r2, r3
	mov	r3, r8, asl r3
	sub	r3, r3, #1
	and	r0, r3, r0, lsr r2
	bl	__aeabi_uidiv
	mov	r1, r8
	mov	r2, #0
	uxth	r0, r0
	bl	gc_add_sblk
.L3695:
	ldr	r2, [r7, #12]
	ldr	r3, [r4, #580]
	ldr	r2, [r2, #4]
	ldr	r3, [r3, #544]
	cmp	r2, r3
	beq	.L3696
	ldr	r1, .L3708+8
	movw	r2, #1738
	ldr	r0, .L3708+12
	bl	printk
	bl	dump_stack
.L3696:
	mov	r0, r7
	bl	zbuf_free
.L3694:
	ldr	r3, [r4, #580]
	ldr	r1, [r4, #3328]
	ldr	r2, [r3, #544]
	add	r2, r2, #1
	str	r2, [r3, #544]
	cmp	r2, r1
	bcc	.L3697
	mvn	r2, #0
	str	r2, [r3, #544]
	ldr	r2, [r3, #548]
	add	r2, r2, #1
	str	r2, [r3, #548]
	bl	ftl_flush
	bl	pm_flush
	bl	ftl_ext_info_flush
	mov	r0, #0
	bl	ftl_info_flush
	b	.L3691
.L3697:
	ldr	r3, [sp, #4]
	cmn	r3, #1
	bne	.L3691
	sub	r5, r5, #1
	uxth	r5, r5
	cmp	r5, #0
	bne	.L3693
	b	.L3691
.L3692:
	ldr	r2, [r3, #536]
	ldr	r1, [r3, #12]
	add	r2, r2, #12910592
	add	r2, r2, #49408
	cmp	r1, r2
	bhi	.L3702
	ldr	r0, [r4, #3348]
	ldr	r2, [r3, #540]
	ldr	r0, [r0, #44]
	add	r2, r2, #4992
	add	r2, r2, #8
	cmp	r0, r2
	bls	.L3691
.L3702:
	ldr	r2, [r4, #3348]
	ldr	r2, [r2, #44]
	str	r1, [r3, #536]
	str	r2, [r3, #540]
	mov	r2, #0
	str	r2, [r3, #544]
.L3691:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3709:
	.align	2
.L3708:
	.word	.LANCHOR0
	.word	.LANCHOR0+4028
	.word	.LANCHOR1+2012
	.word	.LC0
	.fnend
	.size	gc_scan_static_data, .-gc_scan_static_data
	.align	2
	.global	gc_block_vpn_scan
	.type	gc_block_vpn_scan, %function
gc_block_vpn_scan:
	.fnstart
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #44
	sub	sp, sp, #44
	ldr	r4, .L3748
	add	r5, r4, #564
	ldr	r2, [r4, #580]
	ldrh	r3, [r5]
	ldr	r2, [r2, #608]
	cmp	r2, r3
	bcs	.L3710
	bl	timer_get_time
	ldr	r6, [r4, #580]
	ldr	r3, [r6, #604]
	add	r3, r3, #29952
	add	r3, r3, #48
	cmp	r0, r3
	bls	.L3710
	bl	timer_get_time
	ldr	r3, [r4, #580]
	ldrh	r1, [r5]
	str	r0, [r6, #604]
	ldr	r2, [r3, #600]
	cmp	r2, r1
	bcs	.L3714
	ldr	r1, [r4, #3348]
	ldrh	r1, [r1, #134]
	cmp	r2, r1
	bcs	.L3715
.L3714:
	ldr	r2, [r4, #3348]
	ldrh	r2, [r2, #134]
	str	r2, [r3, #600]
.L3715:
	ldr	r5, [r3, #600]
	movw	r3, #65535
	uxth	r5, r5
	cmp	r5, r3
	bne	.L3716
	ldr	r1, .L3748+4
	movw	r2, #2035
	ldr	r0, .L3748+8
	bl	printk
	bl	dump_stack
.L3716:
	ldr	r3, [r4, #580]
	mov	r0, r5
	add	r1, sp, #24
	ldr	r6, .L3748
	ldr	r2, [r3, #600]
	add	r2, r2, #1
	str	r2, [r3, #600]
	ldr	r2, [r3, #608]
	add	r2, r2, #1
	str	r2, [r3, #608]
	bl	ftl_get_blk_list_in_sblk
	uxth	r0, r0
	cmp	r0, #0
	beq	.L3710
	ldr	r8, [r6, #568]
	add	r8, r8, r5, asl #2
	ldrb	r3, [r8, #2]	@ zero_extendqisi2
	tst	r3, #192
	and	r2, r3, #224
	moveq	r3, #1
	movne	r3, #0
	cmp	r2, #224
	orreq	r3, r3, #1
	cmp	r3, #0
	bne	.L3717
	ldr	r3, [r6, #580]
	ldrh	r1, [r3, #16]
	cmp	r1, r5
	beq	.L3717
	ldrh	r1, [r3, #48]
	cmp	r1, r5
	beq	.L3717
	ldrh	r3, [r3, #80]
	cmp	r3, r5
	bne	.L3718
.L3717:
	cmp	r2, #0
	bne	.L3710
	ldr	r3, [r4, #576]
	mov	r5, r5, asl #1
	ldrh	r3, [r3, r5]
	cmp	r3, #0
	beq	.L3719
	ldr	r1, .L3748+4
	movw	r2, #2051
	ldr	r0, .L3748+8
	bl	printk
	bl	dump_stack
.L3719:
	ldr	r3, [r4, #576]
	mov	r2, #0
	strh	r2, [r3, r5]	@ movhi
	b	.L3710
.L3718:
	add	r3, sp, #40
	add	r0, r3, r0, asl #1
	movw	r3, #65535
	ldrh	r6, [r0, #-18]
	cmp	r6, r3
	bne	.L3721
	ldr	r1, .L3748+4
	movw	r2, #2057
	ldr	r0, .L3748+8
	bl	printk
	bl	dump_stack
.L3721:
	movw	r3, #3976
	ldr	r2, .L3748
	ldrh	r7, [r4, r3]
	movw	r3, #4076
	ldrh	r3, [r4, r3]
	sub	r7, r7, #1
	mul	r6, r3, r6
	ldrb	r3, [r8, #2]	@ zero_extendqisi2
	uxth	r7, r7
	and	r3, r3, #224
	cmp	r3, #160
	movne	r10, #1
	bne	.L3722
	ldrb	r3, [r2, #4032]	@ zero_extendqisi2
	cmp	r3, #2
	uxth	r10, r3
	orr	r6, r6, r3, asl #24
	movweq	r3, #3986
	ldreqh	r7, [r2, r3]
	subeq	r7, r7, #1
	uxtheq	r7, r7
.L3722:
	mov	r0, #1
	orr	r6, r7, r6
	bl	buf_alloc
	mov	r1, #1
	ldr	r9, .L3748
	str	r6, [r0, #24]
	mov	r7, r0
	bl	sblk_read_page
	movw	r3, #3986
	ldrh	r2, [r4, r3]
	mov	r1, #255
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	ldr	r0, [r4, #4000]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r2, [r7, #36]
	cmn	r2, #1
	cmpne	r2, #512
	moveq	r2, #1
	movne	r2, #0
	moveq	r0, r5
	moveq	r1, #1
	moveq	r2, #0
	beq	.L3746
	ldr	r3, [r7, #12]
	ldr	r1, [r3]
	ldr	r3, .L3748+12
	cmp	r1, r3
	beq	.L3724
	mov	r0, r5
	mov	r1, #1
.L3746:
	bl	gc_add_sblk
	mov	r0, r7
	bl	zbuf_free
	b	.L3710
.L3724:
	movw	r3, #3976
	ldrb	r6, [r9, #3984]	@ zero_extendqisi2
	ldrh	r3, [r9, r3]
	mov	fp, #1
	ldr	r9, [r7, #4]
	smulbb	r6, r6, r3
	smulbb	r6, r6, r10
	mov	r10, r2
	uxth	r3, r6
	mov	r6, r2
	str	r3, [sp, #12]
.L3725:
	ldr	r3, [sp, #12]
	cmp	r10, r3
	bge	.L3747
	ldr	r0, [r9, r10, asl #2]
	cmn	r0, #1
	beq	.L3726
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #20]
	bne	.L3727
	ldr	r0, [r9, r10, asl #2]
	add	r1, sp, #20
	mov	r2, #0
	bl	pm_log2phys
.L3727:
	ldr	r3, .L3748+16
	ldrb	r2, [r4, #1700]	@ zero_extendqisi2
	ldr	r0, [sp, #20]
	ldrh	r1, [r3]
	rsb	r2, r2, #24
	rsb	r2, r1, r2
	mov	r2, fp, asl r2
	sub	r2, r2, #1
	and	r0, r2, r0, lsr r1
	ldrb	r1, [r4, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	cmp	r0, r5
	addeq	r6, r6, #1
	uxtheq	r6, r6
.L3726:
	add	r10, r10, #1
	b	.L3725
.L3747:
	mov	r0, r7
	mov	r7, r5, asl #1
	bl	zbuf_free
	ldr	r3, .L3748+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3729
	ldr	r3, [r4, #576]
	mov	r1, r5
	ldr	r0, .L3748+24
	ldrh	r2, [r3, r7]
	ldrb	r3, [r8, #2]	@ zero_extendqisi2
	mov	r3, r3, lsr #5
	str	r3, [sp]
	mov	r3, r6
	bl	printk
.L3729:
	ldr	r3, [r4, #576]
	cmp	r6, #31
	strh	r6, [r3, r7]	@ movhi
	bhi	.L3710
	mov	r0, r5
	mov	r1, #1
	mov	r2, #0
	bl	gc_add_sblk
.L3710:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3749:
	.align	2
.L3748:
	.word	.LANCHOR0
	.word	.LANCHOR1+2032
	.word	.LC0
	.word	-178307901
	.word	.LANCHOR0+4028
	.word	.LANCHOR2
	.word	.LC255
	.fnend
	.size	gc_block_vpn_scan, .-gc_block_vpn_scan
	.align	2
	.global	ftl_sblk_dump
	.type	ftl_sblk_dump, %function
ftl_sblk_dump:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r0, asl #2
	ldr	r4, .L3786
	.pad #132
	sub	sp, sp, #132
	mov	r7, r0
	str	r1, [sp, #64]
	ldr	r2, [r4, #568]
	add	r3, r2, r10
	ldrh	r1, [r2, r10]
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ubfx	r1, r1, #0, #11
	str	r1, [sp]
	ldr	r9, [r2, r0, asl #2]
	mov	r1, r7
	mov	r2, r3, lsr #5
	ldr	r0, .L3786+4
	ubfx	r3, r3, #3, #2
	ubfx	r9, r9, #11, #8
	str	r9, [sp, #4]
	bl	printk
	movw	r3, #65535
	cmp	r7, r3
	beq	.L3772
	add	r3, r4, #564
	ldrh	r3, [r3]
	cmp	r3, r7
	bls	.L3772
	ldr	r3, [r4, #568]
	add	r1, sp, #112
	mov	r0, r7
	strh	r7, [sp, #96]	@ movhi
	add	r3, r3, r10
	mov	r5, #0
	ldr	fp, .L3786
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #160
	ldreqb	r9, [r4, #4032]	@ zero_extendqisi2
	movne	r9, #1
	bl	ftl_get_blk_list_in_sblk
	movw	r3, #3976
	ldrh	r3, [r4, r3]
	mov	r1, r7
	uxtb	r0, r0
	strh	r5, [sp, #98]	@ movhi
	strb	r0, [sp, #105]
	smulbb	r3, r3, r0
	strb	r5, [sp, #101]
	strh	r5, [sp, #106]	@ movhi
	strh	r3, [sp, #102]	@ movhi
	ldr	r3, [r4, #568]
	add	r3, r3, r10
	ldrb	r2, [r3, #2]	@ zero_extendqisi2
	mov	r3, r9
	str	r0, [sp]
	ldr	r0, .L3786+8
	mov	r2, r2, lsr #5
	bl	printk
	mov	r0, #1
	bl	buf_alloc
	mov	r6, r0
	str	r5, [sp, #48]
	str	r5, [sp, #56]
	str	r5, [sp, #44]
.L3753:
	ldr	r3, .L3786+12
	uxth	r2, r5
	str	r2, [sp, #52]
	ldrh	r3, [r3]
	cmp	r3, r2
	bls	.L3769
	mov	r3, r2, asl #1
	add	r2, r3, r2
	sub	r3, r3, #1
	str	r3, [sp, #72]
	mov	r3, #0
	sub	r2, r2, #1
	str	r2, [sp, #68]
.L3784:
	str	r3, [sp, #60]
	ldrb	r2, [sp, #105]	@ zero_extendqisi2
	ldrh	r3, [sp, #60]
	cmp	r2, r3
	bls	.L3767
	add	r3, r3, #8
	mov	r8, #1
	mov	r3, r3, asl r8
	str	r3, [sp, #76]
.L3768:
	cmp	r8, r9
	bhi	.L3785
	ldr	r2, [sp, #76]
	add	r3, sp, #128
	add	r3, r3, r2
	movw	r2, #65535
	ldrh	r3, [r3, #-32]
	cmp	r3, r2
	beq	.L3754
	ldr	r2, .L3786+16
	cmp	r9, #3
	ldrh	r4, [r2]
	mul	r4, r4, r3
	bne	.L3755
	ldrb	r2, [fp, #1778]	@ zero_extendqisi2
	cmp	r2, #0
	uxtaheq	r4, r4, r5
	ldrne	r2, [sp, #68]
	orreq	r4, r4, r8, asl #24
	beq	.L3757
	b	.L3783
.L3755:
	cmp	r9, #2
	uxtahne	r4, r4, r5
	bne	.L3757
	ldr	r2, [sp, #72]
.L3783:
	add	r4, r4, r2
	ldrb	r2, [fp, #4032]	@ zero_extendqisi2
	add	r4, r4, r8
	orr	r4, r4, r2, asl #24
.L3757:
	mov	r1, #1
	str	r4, [r6, #24]
	mov	r0, r6
	str	r3, [sp, #84]
	bl	sblk_read_page
	ldr	r1, [r6, #4]
	ldr	ip, [r6, #36]
	ldr	r2, [r6, #12]
	str	ip, [sp]
	ldr	r0, [r1]
	ldr	r3, [sp, #84]
	str	ip, [sp, #80]
	str	r0, [sp, #4]
	ldr	r0, [r1, #4]
	str	r0, [sp, #8]
	ldr	r0, [r1, #8]
	str	r0, [sp, #12]
	ldr	r1, [r1, #12]
	ldr	r0, .L3786+20
	str	r1, [sp, #16]
	ldr	r1, [r2]
	str	r1, [sp, #20]
	ldr	r1, [r2, #4]
	str	r1, [sp, #24]
	ldr	r1, [r2, #8]
	str	r1, [sp, #28]
	mov	r1, r3
	ldr	r2, [r2, #12]
	mov	r3, r4
	str	r2, [sp, #32]
	ldr	r2, [sp, #52]
	bl	printk
	ldr	ip, [sp, #80]
	ldr	r3, [sp, #56]
	cmn	ip, #1
	cmpne	ip, #512
	moveq	r3, #1
	str	r3, [sp, #56]
	ldr	r3, [fp, #568]
	add	r3, r3, r10
	ldrb	r2, [r3, #2]	@ zero_extendqisi2
	and	r2, r2, #224
	cmp	r2, #224
	cmpne	r2, #32
	moveq	r2, #1
	movne	r2, #0
	beq	.L3754
	ldr	r3, [r6, #12]
	str	r2, [sp, #80]
	ldr	r0, [r3, #4]
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #92]
	ldr	r2, [sp, #80]
	bne	.L3760
	ldr	r3, [r6, #12]
	add	r1, sp, #92
	ldr	r0, [r3, #4]
	bl	pm_log2phys
.L3760:
	ldr	r3, [sp, #92]
	cmp	r3, r4
	bne	.L3761
	ldr	r3, [sp, #48]
	mov	r1, r4
	ldr	r0, .L3786+24
	add	r3, r3, #1
	str	r3, [sp, #48]
	ldr	r3, [r6, #12]
	ldr	r2, [r3, #4]
	ldr	r3, [sp, #48]
	bl	printk
.L3761:
	ldr	r3, [sp, #64]
	cmp	r3, #0
	beq	.L3763
	ldr	r3, [sp, #44]
	ldr	r2, [sp, #44]
	mov	r4, r3, asl #2
	ldr	r3, [sp, #64]
	ldr	r2, [r3, r2, asl #2]
	ldr	r3, [r6, #12]
	ldr	r3, [r3, #4]
	cmp	r3, r2
	beq	.L3764
	ldr	r3, .L3786+28
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3764
	ldr	r0, .L3786+32
	ldr	r1, [sp, #44]
	bl	printk
.L3764:
	ldr	r3, [sp, #64]
	ldr	r2, [r6, #12]
	ldr	r3, [r3, r4]
	ldr	r2, [r2, #4]
	cmp	r2, r3
	cmnne	r3, #1
	beq	.L3763
	ldr	r1, .L3786+36
	movw	r2, #1383
	ldr	r0, .L3786+40
	bl	printk
	bl	dump_stack
.L3763:
	ldr	r3, [sp, #44]
	add	r3, r3, #1
	str	r3, [sp, #44]
.L3754:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L3768
.L3785:
	ldr	r3, [sp, #60]
	add	r3, r3, #1
	b	.L3784
.L3767:
	add	r5, r5, #1
	b	.L3753
.L3769:
	mov	r0, r6
	bl	zbuf_free
	ldr	r3, .L3786
	ldr	r0, .L3786+44
	mov	r1, r7
	ldr	r2, [r3, #576]
	mov	r3, r7, asl #1
	ldrh	r2, [r2, r3]
	ldr	r3, [sp, #48]
	bl	printk
	ldr	r0, [sp, #56]
	b	.L3751
.L3772:
	mov	r0, #0
.L3751:
	add	sp, sp, #132
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3787:
	.align	2
.L3786:
	.word	.LANCHOR0
	.word	.LC256
	.word	.LC257
	.word	.LANCHOR0+3976
	.word	.LANCHOR0+4076
	.word	.LC201
	.word	.LC258
	.word	.LANCHOR2
	.word	.LC259
	.word	.LANCHOR1+2052
	.word	.LC0
	.word	.LC260
	.fnend
	.size	ftl_sblk_dump, .-ftl_sblk_dump
	.align	2
	.global	zftl_read
	.type	zftl_read, %function
zftl_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, r3
	ldr	r3, .L3849
	.pad #76
	sub	sp, sp, #76
	mov	r6, r0
	mov	r4, r1
	mov	r7, r2
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3789
	ldr	r0, .L3849+4
	mov	r1, r6
	mov	r2, r4
	mov	r3, r7
	bl	printk
.L3789:
	cmp	r6, #0
	ldreq	r3, .L3849+8
	ldreq	r2, [r3, #520]
	moveq	r3, #24576
	beq	.L3791
.L3790:
	cmp	r6, #3
	bhi	.L3817
	mov	r3, r6, asl #13
	mov	r2, #8192
.L3791:
	add	r1, r4, r7
	cmp	r1, r2
	bhi	.L3817
	ldr	r2, .L3849+8
	add	r3, r3, r4
	str	r3, [sp, #28]
	ldr	r0, [sp, #28]
	ldr	r1, [r2, #3348]
	ldrb	r4, [r2, #3320]	@ zero_extendqisi2
	ldr	r8, .L3849+12
	ldr	r3, [r1, #24]
	add	r3, r3, r7
	str	r3, [r1, #24]
	mov	r1, r4
	bl	__aeabi_uidiv
	mov	r1, r4
	ldr	r3, [sp, #28]
	str	r0, [sp, #36]
	add	r3, r3, r7
	str	r3, [sp, #44]
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	ldr	r3, [sp, #36]
	ldr	r4, [sp, #36]
	rsb	r3, r3, #1
	str	r0, [sp, #48]
	add	r3, r3, r0
	str	r3, [sp, #32]
	mov	r3, #0
	str	r3, [sp, #40]
.L3793:
	ldr	r3, [sp, #32]
	cmp	r3, #0
	beq	.L3846
	ldr	r3, [sp, #36]
	ldr	r6, .L3849+8
	rsb	r2, r3, r4
	ldr	r3, [sp, #48]
	clz	r2, r2
	rsb	r3, r3, r4
	ldrb	r1, [r6, #3320]	@ zero_extendqisi2
	clz	r3, r3
	mov	r2, r2, lsr #5
	mov	r9, r6
	mov	r3, r3, lsr #5
	uxth	r10, r1
	orrs	fp, r3, r2
	beq	.L3794
	cmp	r2, #0
	beq	.L3795
	ldr	r0, [sp, #28]
	bl	__aeabi_uidivmod
	uxth	fp, r1
	rsb	r10, fp, r10
	uxth	r10, r10
	cmp	r10, r7
	uxthhi	r10, r7
	b	.L3794
.L3795:
	cmp	r3, #0
	ldrne	r3, [sp, #28]
	movne	fp, r2
	moveq	fp, r3
	addne	r3, r3, r7
	mlsne	r1, r4, r1, r3
	uxtbne	r10, r1
.L3794:
	ldr	r2, .L3849+16
	mov	r3, #0
.L3798:
	ldr	r1, [r2, #20]
	cmp	r1, r4
	bne	.L3796
	ldrb	r1, [r2, #2]	@ zero_extendqisi2
	tst	r1, #8
	beq	.L3796
	ldr	r1, .L3849+8
	mov	r2, #48
	mov	r10, r10, asl #9
	mov	r0, r5
	add	r5, r5, r10
	mla	r3, r2, r3, r1
	mov	r2, r10
	ldr	r1, [r3, #1788]
	add	r1, r1, fp, asl #9
	bl	ftl_memcpy
	b	.L3797
.L3796:
	add	r3, r3, #1
	add	r2, r2, #48
	cmp	r3, #32
	bne	.L3798
	mov	r0, r4
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #68]
	bne	.L3799
	mov	r0, r4
	add	r1, sp, #68
	mov	r2, #0
	bl	pm_log2phys
.L3799:
	ldr	r3, [sp, #68]
	cmn	r3, #1
	moveq	r10, #0
	bne	.L3847
.L3801:
	ldrb	r3, [r9, #3320]	@ zero_extendqisi2
	cmp	r10, r3
	bcs	.L3797
	mla	r3, r3, r4, r10
	ldr	r2, [sp, #28]
	ldr	r1, [sp, #44]
	cmp	r3, r2
	movcs	r2, #1
	movcc	r2, #0
	cmp	r3, r1
	movcs	r2, #0
	cmp	r2, #0
	beq	.L3802
	mov	r0, r5
	mov	r1, #512
	bl	__memzero
	add	r5, r5, #512
.L3802:
	add	r10, r10, #1
	b	.L3801
.L3847:
	mov	r0, #0
	bl	buf_alloc
	subs	r3, r0, #0
	beq	.L3804
	ldr	r2, .L3849+8
	ldr	r1, [r2, #3348]
	ldr	r2, [r1, #40]
	add	r2, r2, #1
	str	r2, [r1, #40]
	ldr	r2, [sp, #68]
	str	r5, [r3, #8]
	add	r5, r5, r10, asl #9
	str	r4, [r3, #20]
	str	r2, [r3, #24]
	str	r2, [r3, #28]
	strb	r10, [r3, #40]
	strb	fp, [r3, #41]
	bl	zftl_add_read_buf
.L3797:
	ldr	r3, [sp, #32]
	add	r4, r4, #1
	sub	r3, r3, #1
	str	r3, [sp, #32]
.L3804:
	ldrb	r3, [r6, #3322]	@ zero_extendqisi2
	ldr	r2, [sp, #32]
	cmp	r2, #0
	cmpne	r3, #2
	bhi	.L3793
	ldrb	r1, [r8, #-3572]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L3793
	ldrb	r2, [r8, #-3573]	@ zero_extendqisi2
	mov	r0, #48
	ldr	r3, .L3849+16
	mla	r0, r0, r2, r3
	bl	sblk_read_page
.L3806:
	ldrb	r3, [r8, #-3572]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3848
	ldrb	r3, [r8, #-3573]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L3807
	ldr	r1, .L3849+20
	mov	r2, #1088
	ldr	r0, .L3849+24
	bl	printk
	bl	dump_stack
.L3807:
	ldr	r3, .L3849+12
	mov	r2, #48
	ldr	fp, .L3849+8
	ldr	r1, .L3849+12
	ldrb	r10, [r3, #-3573]	@ zero_extendqisi2
	ldr	r3, .L3849+16
	mul	r2, r2, r10
	add	r3, r3, r2
	add	r2, fp, r2
	str	r3, [sp, #52]
	ldr	r9, [r2, #1820]
	ldrb	r3, [r2, #1784]	@ zero_extendqisi2
	cmn	r9, #1
	strb	r3, [r1, #-3573]
	bne	.L3808
	ldr	r2, [fp, #580]
	str	r9, [sp, #40]
	ldr	r3, [r2, #552]
	add	r3, r3, #1
	str	r3, [r2, #552]
	b	.L3809
.L3808:
	cmp	r9, #256
	bne	.L3809
	ldr	r1, .L3849+28
	mov	r6, #1
	ldr	r3, [r2, #1808]
	str	r2, [sp, #60]
	ldrh	r0, [r1]
	ldrb	r1, [fp, #1700]	@ zero_extendqisi2
	str	r3, [sp, #56]
	rsb	r1, r1, #24
	rsb	r1, r0, r1
	mov	r1, r6, asl r1
	sub	r1, r1, #1
	and	r0, r1, r3, lsr r0
	ldrb	r1, [fp, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r2, [sp, #60]
	uxth	fp, r0
	ldr	r3, [sp, #56]
	mov	r1, fp
	ldr	r0, .L3849+32
	ldr	r2, [r2, #1804]
	bl	printk
	mov	r0, fp
	mov	r1, r6
	mov	r2, #0
	bl	gc_add_sblk
.L3809:
	ldr	ip, .L3849+8
	mov	r6, #48
	cmn	r9, #1
	mla	r6, r6, r10, ip
	movne	fp, #0
	moveq	fp, #1
	ldr	r3, [r6, #1796]
	ldr	r2, [r6, #1804]
	ldr	r3, [r3, #4]
	cmp	r3, r2
	moveq	r3, fp
	orrne	r3, fp, #1
	cmp	r3, #0
	beq	.L3810
	ldr	r2, [ip, #580]
	ldr	r0, .L3849+36
	str	ip, [sp, #56]
	ldr	r3, [r2, #552]
	add	r3, r3, #1
	str	r3, [r2, #552]
	ldr	r3, [r6, #1796]
	ldr	r2, [r6, #1808]
	ldrb	r1, [r6, #1785]	@ zero_extendqisi2
	str	r2, [sp]
	ldr	r2, [r3]
	str	r2, [sp, #4]
	ldr	r2, [r3, #4]
	str	r2, [sp, #8]
	ldr	r2, [r3, #8]
	str	r2, [sp, #12]
	mov	r2, r9
	ldr	r3, [r3, #12]
	str	r3, [sp, #16]
	ldr	r3, [r6, #1804]
	bl	printk
	ldr	r3, .L3849+28
	ldr	r2, [r6, #1808]
	ldrh	r0, [r3]
	mov	r2, r2, lsr r0
	ldr	ip, [sp, #56]
	ldrb	r3, [ip, #1700]	@ zero_extendqisi2
	ldrb	r1, [ip, #4030]	@ zero_extendqisi2
	rsb	r3, r3, #24
	rsb	r3, r0, r3
	mvn	r0, #0
	bic	r0, r2, r0, asl r3
	bl	__aeabi_uidiv
	mov	r1, #0
	uxth	r0, r0
	bl	ftl_sblk_dump
.L3810:
	ldr	r6, .L3849+8
	mov	r3, #48
	mla	r3, r3, r10, r6
	mov	r9, r6
	ldr	r2, [r3, #1796]
	ldr	r3, [r3, #1804]
	ldr	r2, [r2, #4]
	cmp	r2, r3
	orrne	fp, fp, #1
	cmp	fp, #0
	beq	.L3811
	ldr	r1, .L3849+20
	movw	r2, #1109
	ldr	r0, .L3849+24
	bl	printk
	bl	dump_stack
.L3811:
	mov	r3, #48
	mla	r10, r3, r10, r9
	ldrb	r3, [r6, #3320]	@ zero_extendqisi2
	ldrb	r2, [r10, #1824]	@ zero_extendqisi2
	cmp	r3, r2
	ldrlsb	r3, [r10, #1786]	@ zero_extendqisi2
	bicls	r3, r3, #8
	strlsb	r3, [r10, #1786]
	bls	.L3813
	ldrb	r1, [r10, #1825]	@ zero_extendqisi2
	mov	r2, r2, asl #9
	ldr	r3, [r10, #1788]
	ldr	r0, [r10, #1792]
	add	r1, r3, r1, asl #9
	bl	ftl_memcpy
.L3813:
	ldr	r1, [sp, #52]
	ldr	r0, .L3849+40
	bl	buf_remove_buf
	ldr	r0, [sp, #52]
	bl	zbuf_free
	ldrb	r3, [r8, #-3572]	@ zero_extendqisi2
	sub	r3, r3, #1
	strb	r3, [r8, #-3572]
	b	.L3806
.L3848:
	ldr	r1, .L3849+12
	mvn	r2, #0
	strb	r3, [r1, #-3572]
	strb	r2, [r1, #-3573]
	b	.L3793
.L3846:
	bl	timer_get_time
	ldr	r3, .L3849+44
	str	r0, [r3, #-2476]
	ldr	r0, [sp, #40]
	b	.L3792
.L3817:
	mvn	r0, #0
.L3792:
	add	sp, sp, #76
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3850:
	.align	2
.L3849:
	.word	.LANCHOR2
	.word	.LC261
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+1784
	.word	.LANCHOR1+2068
	.word	.LC0
	.word	.LANCHOR0+4028
	.word	.LC262
	.word	.LC263
	.word	.LANCHOR3-3571
	.word	.LANCHOR5
	.fnend
	.size	zftl_read, .-zftl_read
	.align	2
	.global	zftl_vendor_read
	.type	zftl_vendor_read, %function
zftl_vendor_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	ip, r1
	mov	r3, r2
	add	r1, r0, #512
	mov	r2, ip
	mov	r0, #2
	b	zftl_read
	.fnend
	.size	zftl_vendor_read, .-zftl_vendor_read
	.align	2
	.global	zftl_sys_read
	.type	zftl_sys_read, %function
zftl_sys_read:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!
	.save {lr}
	mov	ip, r1
	mov	lr, r0
	mov	r3, r2
	mov	r1, lr
	mov	r0, #2
	mov	r2, ip
	ldr	lr, [sp], #4
	b	zftl_read
	.fnend
	.size	zftl_sys_read, .-zftl_sys_read
	.align	2
	.type	zftl_debug_proc_write, %function
zftl_debug_proc_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	cmp	r2, #79
	.pad #124
	sub	sp, sp, #124
	mov	r5, r2
	add	r4, sp, #40
	mvnhi	r0, #21
	str	r4, [sp, #36]
	bhi	.L3855
	mov	r0, r4
	bl	rk_copy_from_user
	cmp	r0, #0
	mvnne	r0, #13
	bne	.L3855
	add	r3, sp, #120
	mov	r1, r4
	add	r3, r3, r5
	strb	r0, [r3, #-80]
	ldr	r0, .L3880
	bl	printk
	ldr	r0, .L3880+4
	mov	r1, r4
	mov	r2, #1
	mov	r3, #16
	bl	rknand_print_hex
	bl	rknand_device_lock
	mov	r0, r4
	ldr	r1, .L3880+8
	mov	r2, #7
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3856
	ldr	r4, .L3880+12
	movw	ip, #698
	ldr	r0, .L3880+16
	mov	r2, #4
	ldr	r7, .L3880+20
	movw	r8, #65535
	ldr	r3, [r4, #3348]
	add	r1, r3, #704
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldr	r1, [r4, #3348]
	ldr	r0, .L3880+24
	mov	r2, #2
	add	r1, r1, #416
	ldrh	r3, [r7, #-2]!
	bl	rknand_print_hex
.L3857:
	ldrh	r3, [r7]
	cmp	r6, r3
	bge	.L3860
	mov	r0, #300
	bl	msleep
	add	r3, r6, #208
	ldr	r2, [r4, #3348]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	cmp	r0, r8
	beq	.L3858
	mov	r1, #0
	bl	ftl_sblk_dump
.L3858:
	add	r6, r6, #1
	b	.L3857
.L3856:
	mov	r0, r4
	ldr	r1, .L3880+28
	mov	r2, #7
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3861
	ldr	r7, .L3880+12
	movw	ip, #698
	ldr	r0, .L3880+16
	mov	r2, #4
	ldr	r9, .L3880+20
	ldr	r3, [r7, #3348]
	add	r9, r9, #4
	add	r1, r3, #704
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldr	r1, [r7, #3348]
	ldrh	r3, [r9, #-6]
	mov	r2, #2
	ldr	r0, .L3880+24
	add	r1, r1, #416
	bl	rknand_print_hex
	ldr	r0, .L3880+32
	add	r4, sp, #120
	add	r1, sp, #47
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	mov	r4, r6
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.34
	uxth	r8, r0
	str	r0, [sp, #32]
.L3863:
	add	r6, r9, r4
	ldr	r0, .L3880+36
	ldrh	r1, [r9, r4]
	ldrh	r2, [r6, #2]
	bl	printk
	ldrh	r3, [r4, r9]
	cmp	r3, r8
	bne	.L3862
	ldrb	r3, [r7, #3320]	@ zero_extendqisi2
	mov	r2, #4
	ldr	r0, .L3880+40
	ldr	r1, [r6, #4]
	mov	r3, r3, asl #7
	bl	rknand_print_hex
.L3862:
	add	r4, r4, #8
	cmp	r4, #256
	bne	.L3863
	mov	r0, #300
	add	r8, r8, #176
	bl	msleep
	mov	r0, #1
	bl	buf_alloc
	ldr	r3, [r7, #3348]
	mov	r1, #1
	ldr	r3, [r3, r8, asl #2]
	str	r3, [r0, #24]
	mov	r4, r0
	str	r3, [sp, #32]
	bl	sblk_read_page
	ldr	ip, [r4, #4]
	ldr	r3, [r4, #12]
	ldr	r0, .L3880+44
	ldr	r2, [ip, #4]
	str	r2, [sp]
	ldr	r2, [ip, #8]
	ldr	r1, [sp, #32]
	str	r2, [sp, #4]
	ldr	r2, [ip, #12]
	str	r2, [sp, #8]
	ldr	r2, [r3]
	str	r2, [sp, #12]
	ldr	r2, [r3, #4]
	str	r2, [sp, #16]
	ldr	r2, [r3, #8]
	str	r2, [sp, #20]
	ldr	r3, [r3, #12]
	str	r3, [sp, #24]
	ldr	r2, [r4, #36]
	ldr	r3, [ip]
	bl	printk
	ldrb	r3, [r7, #3320]	@ zero_extendqisi2
	ldr	r0, .L3880+48
	mov	r2, #4
	ldr	r1, [r4, #4]
	mov	r3, r3, asl #7
	b	.L3879
.L3861:
	mov	r0, r4
	ldr	r1, .L3880+52
	mov	r2, #7
	bl	memcmp
	cmp	r0, #0
	bne	.L3864
	bl	dump_ftl_info
	b	.L3860
.L3864:
	mov	r0, r4
	ldr	r1, .L3880+56
	mov	r2, #9
	bl	memcmp
	cmp	r0, #0
	bne	.L3865
	add	r1, sp, #120
	add	r0, sp, #49
	str	r0, [r1, #-84]!
	bl	rk_simple_strtoull.constprop.34
	ldr	r3, .L3880+60
	str	r0, [sp, #32]
	strh	r0, [r3]	@ movhi
	bl	dump_all_list_info
	b	.L3860
.L3865:
	mov	r0, r4
	ldr	r1, .L3880+64
	mov	r2, #8
	bl	memcmp
	cmp	r0, #0
	beq	.L3860
	mov	r0, r4
	ldr	r1, .L3880+68
	mov	r2, #8
	bl	memcmp
	cmp	r0, #0
	bne	.L3867
	add	r4, sp, #120
	add	r1, sp, #48
	ldr	r0, .L3880+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r6, .L3880+12
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.34
	str	r0, [sp, #32]
	mov	r0, #1
	bl	buf_alloc
	mov	r1, #1
	ldr	r3, [sp, #32]
	mov	r4, r0
	str	r3, [r0, #24]
	bl	sblk_read_page
	ldr	r3, [r4, #4]
	ldr	r2, [r4, #12]
	ldr	r0, .L3880+44
	ldr	r1, [r3, #4]
	str	r1, [sp]
	ldr	r1, [r3, #8]
	str	r1, [sp, #4]
	ldr	r1, [r3, #12]
	str	r1, [sp, #8]
	ldr	r1, [r2]
	str	r1, [sp, #12]
	ldr	r1, [r2, #4]
	str	r1, [sp, #16]
	ldr	r1, [r2, #8]
	str	r1, [sp, #20]
	ldr	r2, [r2, #12]
	ldr	r1, [sp, #32]
	str	r2, [sp, #24]
	ldr	r2, [r4, #36]
	ldr	r3, [r3]
	bl	printk
	ldrb	r3, [r6, #3320]	@ zero_extendqisi2
	ldr	r0, .L3880+72
	mov	r2, #4
	ldr	r1, [r4, #4]
	mov	r3, r3, asl #7
	bl	rknand_print_hex
	ldrb	r3, [r6, #3320]	@ zero_extendqisi2
	ldr	r0, .L3880+76
	mov	r2, #4
	ldr	r1, [r4, #12]
	mov	r3, r3, asl #1
.L3879:
	bl	rknand_print_hex
	mov	r0, r4
	bl	zbuf_free
	b	.L3860
.L3867:
	mov	r0, r4
	ldr	r1, .L3880+80
	mov	r2, #8
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3868
	add	r4, sp, #120
	add	r1, sp, #48
	ldr	r0, .L3880+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.34
	mov	r1, r6
	str	r0, [sp, #32]
	uxth	r0, r0
	bl	ftl_sblk_dump
	b	.L3860
.L3868:
	mov	r0, r4
	ldr	r1, .L3880+84
	mov	r2, #10
	bl	memcmp
	cmp	r0, #0
	bne	.L3869
	add	r4, sp, #120
	add	r1, sp, #50
	ldr	r0, .L3880+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.34
	ldr	r3, .L3880+88
	str	r0, [sp, #32]
	str	r0, [r3]
	b	.L3860
.L3869:
	mov	r0, r4
	ldr	r1, .L3880+92
	mov	r2, #8
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3870
	add	r4, sp, #120
	add	r1, sp, #48
	ldr	r0, .L3880+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.34
	mov	r4, r0
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #32]
	bne	.L3871
	mov	r0, r4
	add	r1, sp, #32
	mov	r2, r6
	bl	pm_log2phys
.L3871:
	mov	r1, r4
	ldr	r2, [sp, #32]
	ldr	r0, .L3880+96
	bl	printk
	b	.L3860
.L3870:
	ldr	r0, .L3880+100
	bl	printk
	ldr	r0, .L3880+104
	bl	printk
	ldr	r0, .L3880+108
	bl	printk
	ldr	r0, .L3880+112
	bl	printk
	ldr	r0, .L3880+116
	bl	printk
	ldr	r0, .L3880+120
	bl	printk
	ldr	r0, .L3880+124
	bl	printk
	ldr	r0, .L3880+128
	bl	printk
	ldr	r0, .L3880+132
	bl	printk
.L3860:
	bl	rknand_device_unlock
	mov	r0, r5
.L3855:
	add	sp, sp, #124
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3881:
	.align	2
.L3880:
	.word	.LC264
	.word	.LC265
	.word	.LC266
	.word	.LANCHOR0
	.word	.LC267
	.word	.LANCHOR3-3568
	.word	.LC268
	.word	.LC269
	.word	.LC270
	.word	.LC271
	.word	.LC272
	.word	.LC234
	.word	.LC273
	.word	.LC274
	.word	.LC275
	.word	.LANCHOR0+4064
	.word	.LC276
	.word	.LC277
	.word	.LC229
	.word	.LC230
	.word	.LC278
	.word	.LC279
	.word	.LANCHOR2
	.word	.LC280
	.word	.LC281
	.word	.LC282
	.word	.LC283
	.word	.LC284
	.word	.LC285
	.word	.LC286
	.word	.LC287
	.word	.LC288
	.word	.LC289
	.word	.LC290
	.fnend
	.size	zftl_debug_proc_write, .-zftl_debug_proc_write
	.align	2
	.global	gc_check_data_one_wl
	.type	gc_check_data_one_wl, %function
gc_check_data_one_wl:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #36
	sub	sp, sp, #36
	ldr	r4, .L3915
	ldr	r3, [r4, #3388]
	ldr	r8, [r4, #580]
	cmp	r3, #0
	bne	.L3883
	mov	r0, #1
	bl	buf_alloc
	str	r0, [r4, #3388]
.L3883:
	ldr	r5, [r4, #3388]
	cmp	r5, #0
	bne	.L3884
	ldr	r1, .L3915+4
	movw	r2, #613
	ldr	r0, .L3915+8
	bl	printk
	bl	dump_stack
.L3884:
	ldr	r6, .L3915+12
	add	r2, r8, #96
	ldr	ip, .L3915
	mov	fp, #0
.L3885:
	ldrb	r3, [r8, #89]	@ zero_extendqisi2
	ldr	r7, .L3915+12
	cmp	fp, r3
	bge	.L3913
	add	r7, r7, #696
	mov	r9, #1
.L3895:
	ldrh	r3, [r6, #16]
	cmp	r9, r3
	bgt	.L3914
	ldrb	r0, [ip, #4032]	@ zero_extendqisi2
	ldrh	r3, [r2]
	ldrh	lr, [r7]
	cmp	r0, #2
	mul	r3, lr, r3
	beq	.L3886
	ldrb	r1, [ip, #1778]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L3887
.L3886:
	ldrh	r1, [r6, #12]
	sub	r1, r1, #1
	add	r3, r1, r3
	add	r3, r3, r9
	orr	r3, r3, r0, asl #24
	b	.L3912
.L3887:
	ldrh	r1, [r6, #12]
	cmp	r0, #3
	addne	r1, r3, r1
	strne	r1, [r5, #24]
	bne	.L3888
	add	r3, r3, r1
	orr	r3, r3, r9, asl #24
.L3912:
	str	r3, [r5, #24]
.L3888:
	mov	r0, r5
	mov	r1, #1
	str	ip, [sp, #28]
	str	r2, [sp, #24]
	bl	sblk_read_page
	ldr	r3, [r5, #36]
	cmn	r3, #1
	ldr	r2, [sp, #24]
	ldr	ip, [sp, #28]
	beq	.L3890
	ldr	r0, [r4, #3988]
	ldrh	r1, [r6, #18]
	ldr	r3, [r5, #12]
	ldr	lr, [r0, r1, asl #2]
	ldr	r0, [r3, #4]
	cmp	lr, r0
	bne	.L3890
	ldr	r0, [ip, #3992]
	ldr	r3, [r3, #8]
	ldr	r1, [r0, r1, asl #2]
	cmp	r1, r3
	beq	.L3891
.L3890:
	ldr	r1, [r4, #3988]
	ldrh	r3, [r6, #18]
	ldr	r10, .L3915
	ldr	r3, [r1, r3, asl #2]
	cmn	r3, #1
	beq	.L3891
	movw	r6, #3986
	ldrb	r3, [r8, #89]	@ zero_extendqisi2
	ldrh	ip, [r10, r6]
	mov	r2, #4
	ldr	r0, .L3915+16
	mul	r3, ip, r3
	bl	rknand_print_hex
	ldrb	r3, [r8, #89]	@ zero_extendqisi2
	ldrh	ip, [r10, r6]
	mov	r2, #4
	ldr	r0, .L3915+20
	ldr	r1, [r10, #3992]
	mul	r3, ip, r3
	bl	rknand_print_hex
	ldr	r3, .L3915+24
	ldr	r3, [r3]
	tst	r3, #1024
	beq	.L3892
	ldr	r3, .L3915+12
	ldr	r2, [r10, #3992]
	ldr	lr, [r10, #3988]
	ldrh	ip, [r3, #18]
	ldr	r3, [r5, #12]
	ldr	r0, .L3915+28
	ldr	r2, [r2, ip, asl #2]
	str	r2, [sp]
	ldr	r2, [r3]
	str	r2, [sp, #4]
	ldr	r2, [r3, #4]
	str	r2, [sp, #8]
	ldr	r2, [r3, #8]
	str	r2, [sp, #12]
	ldr	r3, [r3, #12]
	str	r3, [sp, #16]
	ldr	r1, [r5, #24]
	ldr	r2, [r5, #36]
	ldr	r3, [lr, ip, asl #2]
	bl	printk
.L3892:
	ldr	r3, [r4, #580]
	mov	r1, #0
	ldrh	r0, [r3, #80]
	bl	ftl_sblk_dump
	ldrh	r3, [r8, #80]
	ldr	r2, [r4, #576]
	mov	r1, #0
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	ldr	r2, [r4, #580]
	ldr	r3, [r2, #556]
	add	r3, r3, #1
	str	r3, [r2, #556]
	ldr	r3, [r4, #3348]
	ldr	r2, [r3, #156]
	ldr	r3, .L3915+32
	cmp	r2, r3
	bne	.L3896
	ldr	r0, [r5, #24]
	bl	ftl_mask_bad_block
	b	.L3896
.L3891:
	ldrh	r3, [r6, #18]
	add	r9, r9, #1
	add	r3, r3, #1
	strh	r3, [r6, #18]	@ movhi
	b	.L3895
.L3914:
	add	fp, fp, #1
	add	r2, r2, #2
	b	.L3885
.L3913:
	ldrb	r0, [r4, #4033]	@ zero_extendqisi2
	ldrh	r3, [r7, #12]
	cmp	r0, #0
	add	r2, r3, #1
	addne	r3, r3, #2
	strh	r2, [r7, #12]	@ movhi
	movne	r0, #0
	strneh	r3, [r7, #12]	@ movhi
	b	.L3893
.L3896:
	mvn	r0, #0
.L3893:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3916:
	.align	2
.L3915:
	.word	.LANCHOR0
	.word	.LANCHOR1+2080
	.word	.LC0
	.word	.LANCHOR0+3380
	.word	.LC291
	.word	.LC292
	.word	.LANCHOR2
	.word	.LC293
	.word	1145785929
	.fnend
	.size	gc_check_data_one_wl, .-gc_check_data_one_wl
	.align	2
	.global	ftl_update_l2p_map
	.type	ftl_update_l2p_map, %function
ftl_update_l2p_map:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #3976
	ldr	r6, .L3948
	.pad #28
	sub	sp, sp, #28
	ldrb	r8, [r0, #9]	@ zero_extendqisi2
	mov	r7, r0
	ldrh	r4, [r0, #12]
	ldrh	r3, [r6, r3]
	mul	r8, r8, r3
	ldr	r3, .L3948+4
	ldr	r5, [r3, #-3592]
	sub	r2, r8, #-1073741823
	str	r3, [sp, #8]
	add	r4, r5, r4, asl #2
	ldr	r2, [r4, r2, asl #2]
	cmn	r2, #1
	beq	.L3918
	ldr	r1, .L3948+8
	movw	r2, #1733
	ldr	r0, .L3948+12
	bl	printk
	bl	dump_stack
.L3918:
	mov	r5, #0
	sub	r4, r4, #4
	mov	r10, r5
.L3919:
	cmp	r10, r8
	beq	.L3947
	ldr	r9, [r4, #4]!
	cmn	r9, #1
	beq	.L3920
	ldrb	r1, [r6, #3320]	@ zero_extendqisi2
	mov	r0, r9
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	uxth	r3, r0
	str	r3, [sp, #4]
	ldr	r3, .L3948+16
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3921
	ldr	r0, .L3948+20
	mov	r2, r9
	ldr	r1, [sp, #4]
	mov	r3, r10
	bl	printk
.L3921:
	mov	r9, r4
	mov	fp, r10
.L3924:
	ldr	r0, [r9]
	cmn	r0, #1
	beq	.L3922
	ldrb	r1, [r6, #3320]	@ zero_extendqisi2
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	ldr	r3, [sp, #4]
	uxth	r0, r0
	cmp	r0, r3
	bne	.L3922
	ldrb	r3, [r7, #9]	@ zero_extendqisi2
	mov	r0, fp
	mov	r1, r3
	str	r3, [sp, #12]
	bl	__aeabi_idiv
	ldr	r2, .L3948+24
	ldrh	r2, [r2]
	ldr	r3, [sp, #12]
	mls	r3, r3, r0, fp
	add	r3, r7, r3, asl #1
	ldrh	r3, [r3, #16]
	mla	r2, r2, r3, r0
	ldr	r0, .L3948+28
	str	r2, [sp, #20]
	ldr	r1, [r9]
	bl	pm_ppa_update_check
	cmp	r0, #0
	beq	.L3923
	ldr	r3, [sp, #8]
	mov	r2, #4
	ldr	r0, .L3948+32
	ldr	r1, [r3, #-3592]
	mov	r3, r8
	bl	rknand_print_hex
.L3923:
	add	r5, r5, #1
	ldr	r0, [r9]
	add	r1, sp, #20
	mov	r2, #1
	uxth	r5, r5
	bl	pm_log2phys
	mvn	r3, #0
	str	r3, [r9]
.L3922:
	add	fp, fp, #1
	add	r9, r9, #4
	cmp	fp, r8
	bne	.L3924
.L3920:
	add	r10, r10, #1
	b	.L3919
.L3947:
	ldr	r3, .L3948+16
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3926
	ldrh	r1, [r7]
	mov	r2, r5
	ldr	ip, [r6, #576]
	ldr	r0, .L3948+36
	mov	r3, r1, asl #1
	ldrh	r3, [ip, r3]
	bl	printk
.L3926:
	ldrh	r3, [r7]
	ldr	r2, [r6, #576]
	mov	r3, r3, asl #1
	strh	r5, [r2, r3]	@ movhi
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3949:
	.align	2
.L3948:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR1+2104
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC294
	.word	.LANCHOR0+4076
	.word	.LC295
	.word	.LC296
	.word	.LC297
	.fnend
	.size	ftl_update_l2p_map, .-ftl_update_l2p_map
	.align	2
	.global	ftl_alloc_new_data_sblk
	.type	ftl_alloc_new_data_sblk, %function
ftl_alloc_new_data_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r6, r0
	ldrh	r5, [r0]
	bl	ftl_update_l2p_map
	bl	pm_flush
	ldrh	r0, [r6]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L3951
	bl	zftl_insert_data_list
.L3951:
	ldr	r4, .L3959
	mov	r0, r6
	ldr	r1, [r4, #580]
	add	r1, r1, #16
	cmp	r1, r6
	movw	r6, #65535
	moveq	r1, #2
	movne	r1, #3
	bl	ftl_open_sblk_init
	ldr	r3, [r4, #580]
	cmp	r5, r6
	ldr	r3, [r3, #560]
	rsb	r3, r5, r3
	clz	r3, r3
	mov	r3, r3, lsr #5
	moveq	r3, #0
	cmp	r3, #0
	beq	.L3953
	mov	r1, r5
	ldr	r0, .L3959+4
	bl	printk
	ldr	r3, [r4, #580]
	ldr	r0, [r3, #564]
	bl	gc_mark_bad_ppa
	ldr	r3, [r4, #580]
	mvn	r2, #0
	str	r6, [r3, #560]
	str	r2, [r3, #564]
.L3953:
	bl	ftl_ext_info_flush
	mov	r0, #0
	bl	ftl_info_flush
	bl	lpa_rebuild_hash
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L3960:
	.align	2
.L3959:
	.word	.LANCHOR0
	.word	.LC298
	.fnend
	.size	ftl_alloc_new_data_sblk, .-ftl_alloc_new_data_sblk
	.align	2
	.global	ftl_write_commit
	.type	ftl_write_commit, %function
ftl_write_commit:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #36
	sub	sp, sp, #36
	ldr	r6, .L4024
	mov	r8, r6
.L3962:
	ldrb	r2, [r6, #3344]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3964
	ldrb	r3, [r6, #3368]	@ zero_extendqisi2
	sub	r2, r2, #1
	strb	r2, [r6, #3344]
	str	r3, [sp, #16]
	mov	r3, #48
	ldr	r1, [sp, #16]
	mul	r3, r3, r1
	ldr	r1, .L4024+4
	add	r1, r1, r3
	add	r3, r6, r3
	str	r1, [sp, #20]
	ldr	r2, [r3, #1804]
	ldrb	r1, [r3, #1784]	@ zero_extendqisi2
	ldr	r3, [r6, #3328]
	cmp	r2, r3
	strb	r1, [r6, #3368]
	bcc	.L3965
	ldr	r1, .L4024+8
	movw	r2, #510
	ldr	r0, .L4024+12
	bl	printk
	bl	dump_stack
.L3965:
	ldr	r2, [sp, #16]
	mov	r3, #48
	ldr	r1, [r8, #3328]
	mla	r2, r3, r2, r8
	ldr	r9, [r2, #1804]
	cmp	r9, r1
	bcc	.L3966
	ldr	r0, [sp, #20]
	bl	zbuf_free
	mvn	r0, #0
	b	.L4020
.L3966:
	ldr	r1, [r2, #1796]
	ldr	fp, [r2, #1788]
	ldrb	r7, [r2, #1825]	@ zero_extendqisi2
	ldrb	r2, [r2, #1824]	@ zero_extendqisi2
	str	r1, [sp, #8]
	str	r2, [sp, #12]
	ldrb	r2, [r6, #3345]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3968
	ldr	r2, .L4024+16
	ldrb	r4, [r2, #-3255]	@ zero_extendqisi2
	ldr	r2, .L4024+4
	mla	r4, r3, r4, r2
.L3969:
	ldrb	r2, [r4]	@ zero_extendqisi2
	cmp	r2, #255
	ldrne	r1, .L4024+4
	mlane	r4, r3, r2, r1
	bne	.L3969
.L4022:
	ldr	r3, [r4, #20]
	cmp	r3, r9
	bne	.L3968
	ldr	r3, [sp, #12]
	mov	r1, r7, asl #9
	ldr	r0, [r4, #4]
	mov	r2, r3, asl #9
	add	r0, r0, r1
	add	r1, fp, r1
	bl	ftl_memcpy
	ldr	r0, [sp, #20]
	bl	zbuf_free
	b	.L3962
.L3968:
	mov	r0, r9
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #28]
	bne	.L3972
	mov	r0, r9
	add	r1, sp, #28
	mov	r2, #0
	bl	pm_log2phys
.L3972:
	ldr	r4, [r8, #580]
	mov	r3, #0
	ldr	r2, .L4024+4
	add	r4, r4, #16
.L3975:
	ldr	r1, [r2, #20]
	cmp	r1, r9
	bne	.L3973
	ldrb	r1, [r2, #2]	@ zero_extendqisi2
	tst	r1, #8
	beq	.L3973
	mov	r10, #48
	bic	r1, r1, #8
	mla	r10, r10, r3, r8
	ldr	r3, [r10, #1808]
	strb	r1, [r10, #1786]
	ldr	r5, [r10, #1788]
	str	r3, [sp, #28]
	b	.L3974
.L3973:
	add	r3, r3, #1
	add	r2, r2, #48
	cmp	r3, #32
	bne	.L3975
	mov	r5, #0
.L3974:
	ldrb	r3, [r8, #3320]	@ zero_extendqisi2
	ldr	r2, [sp, #12]
	cmp	r2, r3
	bcs	.L3999
	cmp	r5, #0
	beq	.L3977
	cmp	r7, #0
	beq	.L3978
	mov	r0, fp
	mov	r1, r5
	mov	r2, r7, asl #9
	bl	ftl_memcpy
	ldr	r4, [r6, #580]
	add	r4, r4, #48
.L3978:
	ldr	r3, [sp, #12]
	ldrb	r2, [r8, #3320]	@ zero_extendqisi2
	add	r7, r7, r3
	cmp	r7, r2
	ldrcs	r4, [r6, #580]
	addcs	r4, r4, #16
	bcs	.L3999
	rsb	r2, r7, r2
	mov	r1, r7, asl #9
	add	r0, fp, r1
	add	r1, r5, r1
	mov	r2, r2, asl #9
	bl	ftl_memcpy
	b	.L3999
.L3977:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L3980
	mov	r0, #1
	bl	buf_alloc
	mov	r1, #1
	ldr	r3, [sp, #28]
	mov	r5, r0
	str	r9, [r0, #20]
	str	r3, [r0, #24]
	bl	sblk_read_page
	ldr	r3, [r5, #12]
	ldr	r10, [r3, #12]
	ldr	r3, [r3, #4]
	add	r10, r10, #1
	cmp	r3, r9
	bne	.L3981
	ldr	r3, [r5, #36]
	cmn	r3, #1
	bne	.L3982
.L3981:
	ldr	r2, [r8, #580]
	ldr	r0, .L4024+20
	ldr	r3, [r2, #552]
	add	r3, r3, #1
	str	r3, [r2, #552]
	ldr	r3, [r5, #36]
	ldrb	r1, [r5, #1]	@ zero_extendqisi2
	ldr	r2, [sp, #28]
	str	r3, [sp]
	mov	r3, r9
	bl	printk
	mov	r2, #4
	ldr	r0, .L4024+24
	mov	r3, r2
	ldr	r1, [r5, #12]
	bl	rknand_print_hex
.L3982:
	ldr	r3, [r5, #12]
	ldr	r3, [r3, #4]
	cmp	r3, r9
	bne	.L3983
	ldr	r3, [r5, #36]
	cmn	r3, #1
	bne	.L3984
.L3983:
	ldr	r1, .L4024+8
	movw	r2, #581
	ldr	r0, .L4024+12
	bl	printk
	bl	dump_stack
.L3984:
	cmp	r7, #0
	beq	.L3985
	ldr	r3, [sp, #28]
	mov	r2, r7, asl #9
	cmn	r3, #1
	beq	.L3986
	mov	r0, fp
	ldr	r1, [r5, #4]
	bl	ftl_memcpy
	b	.L3987
.L3980:
	cmp	r7, #0
	moveq	r5, r7
	moveq	r10, r7
	beq	.L3985
	mov	r2, r7, asl #9
	mov	r10, r5
.L3986:
	mov	r0, fp
	mov	r1, #0
	bl	ftl_memset
.L3987:
	ldr	r4, [r6, #580]
	add	r4, r4, #48
.L3985:
	ldr	r3, [sp, #12]
	ldrb	r2, [r6, #3320]	@ zero_extendqisi2
	add	r7, r7, r3
	cmp	r7, r2
	bcc	.L3988
	ldrhi	r4, [r6, #580]
	addhi	r4, r4, #16
	b	.L3989
.L3988:
	ldr	r3, [sp, #28]
	rsb	r2, r7, r2
	mov	r7, r7, asl #9
	cmn	r3, #1
	mov	r2, r2, asl #9
	beq	.L3990
	ldr	r1, [r5, #4]
	add	r0, fp, r7
	add	r1, r1, r7
	bl	ftl_memcpy
	b	.L3989
.L3990:
	add	r0, fp, r7
	mov	r1, #0
	bl	ftl_memset
.L3989:
	cmp	r5, #0
	beq	.L3976
	ldrb	r3, [r5, #2]	@ zero_extendqisi2
	mov	r1, r5
	ldr	r0, .L4024+28
	bic	r3, r3, #8
	strb	r3, [r5, #2]
	bl	buf_remove_buf
	mov	r0, r5
	bl	zbuf_free
	b	.L3976
.L3999:
	mov	r10, #0
.L3976:
	ldrh	r3, [r4, #6]
	cmp	r3, #0
	bne	.L3991
	bl	ftl_flush
	mov	r0, r4
	bl	ftl_alloc_new_data_sblk
.L3991:
	mov	r0, r4
	ldr	r5, .L4024+16
	bl	ftl_get_new_free_page
	mov	r3, #48
	ldr	r2, [sp, #16]
	ldr	ip, [sp, #8]
	mul	r3, r3, r2
	add	r2, r8, r3
	ldr	r1, [r2, #1800]
	str	r9, [ip, #4]
	str	r10, [ip, #12]
	str	r1, [ip]
	ldr	r1, [sp, #28]
	str	r1, [ip, #8]
	str	r1, [r2, #1812]
	mvn	r1, #0
	strb	r1, [r2, #1784]
	ldrb	r1, [r2, #1786]	@ zero_extendqisi2
	str	r0, [r2, #1808]
	orr	r1, r1, #10
	strb	r1, [r2, #1786]
	ldr	r2, .L4024+4
	ldrh	r10, [r4, #10]
	ldrh	r9, [r4, #12]
	add	r3, r2, r3
	ldr	r0, .L4024+32
	add	r9, r10, r9
	ldr	r1, [sp, #20]
	sub	r9, r9, #1
	strh	r9, [r3, #32]	@ movhi
	bl	buf_add_tail
	ldrb	r3, [r8, #3345]	@ zero_extendqisi2
	add	r3, r3, #1
	strb	r3, [r8, #3345]
	bl	timer_get_time
	ldrb	r3, [r8, #3345]	@ zero_extendqisi2
	ldrh	r2, [r4, #6]
	cmp	r3, #2
	str	r0, [r5, #-3252]
	bhi	.L3992
	cmp	r2, #1
	bne	.L3964
.L3992:
	ldrb	r1, [r4, #5]	@ zero_extendqisi2
	mov	lr, #48
	ldrb	r0, [r5, #-3255]	@ zero_extendqisi2
	cmp	r1, #0
	mov	r1, #0
	moveq	ip, #1
	movne	ip, #2
	cmp	r2, #1
	mov	r2, r0
	moveq	ip, r3
.L3996:
	cmp	r1, ip
	mlane	r2, lr, r2, r6
	addne	r1, r1, #1
	ldrneb	r2, [r2, #1784]	@ zero_extendqisi2
	bne	.L3996
.L4023:
	strb	r2, [r5, #-3255]
	uxtb	r1, r1
	ldr	r2, .L4024+4
	rsb	r3, r1, r3
	strb	r3, [r8, #3345]
	mov	r3, #48
	mla	r0, r3, r0, r2
	bl	sblk_prog_page
	ldrh	r3, [r4, #6]
	cmp	r3, #1
	bne	.L3964
	bl	sblk_wait_write_queue_completed
	bl	ftl_write_completed
	mov	r0, r4
	bl	ftl_write_last_log_page
	mov	r0, r4
	bl	ftl_alloc_new_data_sblk
.L3964:
	ldrb	r4, [r6, #3344]	@ zero_extendqisi2
	cmp	r4, #0
	bne	.L3962
	bl	ftl_write_completed
	mov	r0, r4
.L4020:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4025:
	.align	2
.L4024:
	.word	.LANCHOR0
	.word	.LANCHOR0+1784
	.word	.LANCHOR1+2124
	.word	.LC0
	.word	.LANCHOR3
	.word	.LC299
	.word	.LC230
	.word	.LANCHOR3-3571
	.word	.LANCHOR3-3255
	.fnend
	.size	ftl_write_commit, .-ftl_write_commit
	.align	2
	.global	gc_do_copy_back
	.type	gc_do_copy_back, %function
gc_do_copy_back:
	.fnstart
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #84
	sub	sp, sp, #84
	ldr	fp, .L4175
	ldrb	r0, [fp, #3369]	@ zero_extendqisi2
	ldr	r8, [fp, #580]
	cmp	r0, #0
	bne	.L4027
	bl	buf_alloc
	subs	r4, r0, #0
	beq	.L4026
	ldr	r6, .L4175+4
	ldrh	r5, [r6, #22]
	mov	r0, r5
	add	r5, r5, #1
	bl	gc_get_src_ppa_from_index
	mov	r1, #1
	strh	r5, [r6, #22]	@ movhi
	str	r0, [r4, #24]
	mov	r8, r0
	mov	r0, r4
	bl	sblk_read_page
	ldr	r3, [r4, #36]
	cmn	r3, #1
	cmpne	r3, #512
	bne	.L4029
	ldr	r1, .L4175+8
	mov	r2, #864
	ldr	r0, .L4175+12
	bl	printk
	bl	dump_stack
.L4029:
	ldr	r3, [r4, #12]
	ldr	r5, [r3, #4]
	mov	r0, r5
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #56]
	bne	.L4030
	mov	r0, r5
	add	r1, sp, #56
	mov	r2, #0
	bl	pm_log2phys
.L4030:
	ldr	r7, [sp, #56]
	cmp	r7, r8
	bne	.L4031
	ldr	r1, .L4175+16
	mov	r3, #0
.L4034:
	add	r2, r1, r3
	ldr	r0, [r2, #20]
	cmp	r0, r5
	bne	.L4032
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	tst	r2, #2
	beq	.L4032
	mov	r0, r4
	bl	zbuf_free
	ldr	r3, .L4175+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L4026
	mov	r1, r5
	mov	r2, r7
	ldrh	r3, [r6, #22]
	ldr	r0, .L4175+24
	bl	printk
	b	.L4026
.L4032:
	add	r3, r3, #48
	cmp	r3, #1536
	bne	.L4034
	mov	r3, #0
	strb	r3, [r4, #41]
	ldrb	r3, [fp, #3320]	@ zero_extendqisi2
	str	r5, [r4, #20]
	strb	r3, [r4, #40]
	ldrb	r3, [r4, #2]	@ zero_extendqisi2
	orr	r3, r3, #10
	strb	r3, [r4, #2]
	ldr	r3, [r4, #12]
	ldr	r3, [r3]
	str	r3, [r4, #16]
	ldr	r3, .L4175+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L4035
	mov	r0, r5
	ldr	r8, [sp, #56]
	bl	lpa_hash_get_ppa
	mov	r1, r5
	str	r7, [sp]
	mov	r3, r0
	ldrh	r2, [r6, #22]
	ldr	r0, .L4175+28
	str	r2, [sp, #4]
	mov	r2, r8
	bl	printk
.L4035:
	mov	r0, r4
	bl	ftl_gc_write_buf
	bl	ftl_write_commit
	ldr	r2, [fp, #3348]
	ldr	r3, [r2, #60]
	add	r3, r3, #1
	str	r3, [r2, #60]
	ldrh	r3, [r6, #24]
	add	r3, r3, #1
	strh	r3, [r6, #24]	@ movhi
	b	.L4026
.L4031:
	ldr	r3, .L4175+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L4037
	mov	r0, r5
	bl	lpa_hash_get_ppa
	mov	r1, r5
	str	r8, [sp]
	mov	r3, r0
	ldrh	r2, [r6, #22]
	ldr	r0, .L4175+28
	str	r2, [sp, #4]
	mov	r2, r7
	bl	printk
.L4037:
	mov	r0, r4
	bl	zbuf_free
	b	.L4026
.L4027:
	ldrb	r4, [fp, #4032]	@ zero_extendqisi2
	ldr	r10, .L4175+4
	cmp	r4, #3
	ldrb	r9, [r8, #89]	@ zero_extendqisi2
	bne	.L4038
	ldrb	r2, [fp, #1778]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L4039
	movw	r2, #566
	mov	r1, r9
	ldrh	r7, [r10, r2]
	mov	r0, r7
	bl	__aeabi_idiv
	ldr	r2, .L4175+32
	ldrb	r1, [r2, #-2503]	@ zero_extendqisi2
	cmp	r1, #0
	uxth	r5, r0
	beq	.L4040
	ldr	r2, .L4175+36
	mov	r0, r0, asl #1
	ldrh	r2, [r2, r0]
	cmp	r2, #0
	moveq	r4, #1
	movne	r4, #2
	b	.L4041
.L4040:
	ldrb	r2, [r2, #-2502]	@ zero_extendqisi2
	cmp	r2, #0
	moveq	r4, #1
	beq	.L4041
	sub	r2, r5, #62
	cmp	r2, #2160
	movcs	r4, #2
	bcs	.L4041
	mov	r1, r4
	mov	r0, r5
	bl	__aeabi_uidivmod
	uxth	r1, r1
	cmp	r1, #0
	movne	r4, #1
	moveq	r4, #2
.L4041:
	smulbb	r1, r5, r9
	rsb	r1, r1, r7
	b	.L4165
.L4039:
	ldrb	r2, [fp, #4035]	@ zero_extendqisi2
	cmp	r2, #0
	movw	r2, #566
	ldrh	r7, [r10, r2]
	mov	r0, r7
	beq	.L4043
	add	r1, r9, r9, asl #1
	bl	__aeabi_idiv
	sub	r3, r9, r9, asl #2
	uxth	r5, r0
	smulbb	r1, r3, r5
	add	r1, r7, r1
.L4165:
	uxth	r3, r1
	str	r3, [sp, #36]
	b	.L4044
.L4043:
	mov	r1, r9
	bl	__aeabi_idiv
	ldr	r1, .L4175+40
	mov	r2, r0, asl #1
	ldrh	r3, [r1, r2]
	smulbb	r1, r0, r9
	mov	r5, r3, lsr #3
	and	r3, r3, #7
	str	r3, [sp, #32]
	rsb	r1, r1, r7
	uxth	r3, r1
	str	r3, [sp, #36]
	mla	r3, r9, r5, r3
	add	r3, r3, r3, asl #1
	uxth	r7, r3
	ldr	r3, [sp, #32]
	cmp	r3, #1
	bne	.L4045
	b	.L4044
.L4038:
	ldrb	r2, [fp, #4033]	@ zero_extendqisi2
	mov	r1, r9
	cmp	r2, #0
	movw	r2, #566
	ldrh	r7, [r10, r2]
	mov	r0, r7
	bne	.L4046
	bl	__aeabi_idiv
	mov	r4, #1
	uxth	r5, r0
	smulbb	r1, r5, r9
	rsb	r1, r1, r7
	uxth	r3, r1
	str	r3, [sp, #36]
	b	.L4044
.L4046:
	bl	__aeabi_idiv
	mov	r4, #2
	uxth	r5, r0
	smulbb	r1, r5, r9
	rsb	r1, r1, r7
	uxth	r3, r1
	str	r3, [sp, #36]
.L4044:
	ldr	ip, .L4175+36
	mov	r1, #0
	add	ip, ip, r5, asl #1
.L4047:
	ldrb	lr, [r8, #89]	@ zero_extendqisi2
	uxth	r0, r1
	mul	r3, lr, r4
	cmp	r0, r3
	bge	.L4098
	ldr	r3, .L4175+44
	add	r2, r7, r0
	ldrh	r3, [r3]
	mul	r3, lr, r3
	sub	r3, r3, #1
	cmp	r2, r3
	bne	.L4048
	ldr	r3, .L4175+32
	ldrb	r3, [r3, #-2503]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L4098
	ldrh	r3, [ip]
	cmp	r3, r5
	bcs	.L4098
	tst	r0, #1
	beq	.L4098
.L4048:
	ldr	r3, [fp, #3992]
	ldr	lr, [r3, r2, asl #2]
	cmn	lr, #1
	bne	.L4049
	ldrh	r9, [r10, #22]
	ldrh	lr, [r10, #20]
	cmp	lr, r9
	bls	.L4026
	ldr	lr, .L4175+32
	ldrb	lr, [lr, #-2503]	@ zero_extendqisi2
	cmp	lr, #0
	beq	.L4050
	ldrh	lr, [ip]
	cmp	lr, r5
	bcs	.L4050
	tst	r0, #1
	ldrne	r0, [r3, lr, asl #2]
	strne	r0, [r3, r2, asl #2]
	bne	.L4049
.L4050:
	mov	r0, r9
	str	ip, [sp, #48]
	str	r1, [sp, #44]
	add	r9, r9, #1
	str	r3, [sp, #40]
	str	r2, [sp, #32]
	bl	gc_get_src_ppa_from_index
	strh	r9, [r10, #22]	@ movhi
	ldr	r2, [sp, #32]
	ldr	r3, [sp, #40]
	ldr	ip, [sp, #48]
	ldr	r1, [sp, #44]
	str	r0, [r3, r2, asl #2]
.L4049:
	add	r1, r1, #1
	b	.L4047
.L4098:
	mov	r3, #1
	str	r3, [sp, #32]
.L4045:
	ldr	r3, .L4175+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L4052
	movw	r2, #566
	ldr	r3, [fp, #3992]
	ldrh	r2, [r10, r2]
	mov	r1, r5
	ldr	r0, .L4175+48
	str	r2, [sp]
	ldr	r2, [r3, r7, asl #2]
	add	r3, r3, r7, asl #2
	str	r2, [sp, #4]
	ldr	r3, [r3, #4]
	ldr	r2, .L4175+36
	str	r7, [sp, #12]
	str	r3, [sp, #8]
	mov	r3, r5, asl #1
	ldrh	r3, [r2, r3]
	ldr	r2, [sp, #32]
	str	r3, [sp, #16]
	ldr	r3, [sp, #36]
	bl	printk
.L4052:
	ldr	r1, .L4175+36
	mov	r3, #0
	str	r3, [sp, #40]
	add	r3, r1, r5, asl #1
	str	r3, [sp, #44]
.L4053:
	ldrb	r1, [r8, #89]	@ zero_extendqisi2
	ldrh	r9, [sp, #40]
	ldr	fp, .L4175
	mul	r3, r1, r4
	cmp	r9, r3
	bge	.L4171
	ldr	r3, [fp, #3996]
	add	r10, r7, r9
	ldrb	r3, [r3, r10]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L4054
	mov	r0, #0
	bl	buf_alloc
	subs	r6, r0, #0
	bne	.L4055
	bl	sblk_wait_write_queue_completed
	bl	ftl_write_completed
	bl	gc_write_completed
	bl	gc_free_temp_buf
	mov	r0, r6
	bl	buf_alloc
	subs	r6, r0, #0
	beq	.L4026
.L4055:
	ldrb	r2, [r6, #1]	@ zero_extendqisi2
	ldr	r3, [fp, #3996]
	strb	r2, [r3, r10]
	ldrb	r3, [fp, #3387]	@ zero_extendqisi2
	add	r3, r3, #1
	strb	r3, [fp, #3387]
	add	r3, r9, r7
	strh	r3, [r6, #32]	@ movhi
	ldrb	r3, [sp, #32]	@ zero_extendqisi2
	strb	r3, [r6, #45]
	ldr	r3, .L4175+44
	ldrb	r2, [r8, #89]	@ zero_extendqisi2
	ldrh	r1, [r3]
	mul	r2, r2, r1
	sub	r2, r2, #1
	cmp	r10, r2
	bne	.L4056
	ldr	r2, .L4175+32
	ldrb	r2, [r2, #-2503]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L4057
	ldr	r2, [sp, #44]
	ldrh	r2, [r2]
	cmp	r2, r5
	bcs	.L4057
	tst	r9, #1
	bne	.L4056
.L4057:
	ldr	r2, [fp, #3988]
	mvn	r1, #0
	str	r3, [sp, #48]
	str	r1, [r2, r10, asl #2]
	ldrh	r1, [r3]
	ldrb	r2, [fp, #3984]	@ zero_extendqisi2
	ldr	r0, [r6, #4]
	mul	r2, r2, r1
	ldr	r1, [fp, #3988]
	mov	r2, r2, asl #2
	bl	ftl_memcpy
	ldr	r0, [r6, #4]
	ldr	r3, [sp, #48]
	ldrh	r2, [r3]
	ldrb	r3, [fp, #3984]	@ zero_extendqisi2
	ldr	fp, .L4175
	mul	r2, r3, r2
	ldr	r3, .L4175+32
	ldrb	r3, [r3, #-2502]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r2, r2, asl #2
	ldrne	r1, .L4175
	mov	r3, r2
	add	r0, r0, r3
	ldrneb	r1, [r1, #3320]	@ zero_extendqisi2
	rsbne	r2, r2, r1, asl #9
	ldr	r1, [fp, #3992]
	bl	ftl_memcpy
	ldrb	r2, [fp, #3320]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, [r6, #12]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r3, [r6, #12]
	ldr	r2, .L4175+52
	str	r2, [r3]
	ldr	r2, .L4175+44
	ldr	r3, [r6, #12]
	ldr	r0, [r6, #4]
	ldrh	r1, [r2]
	ldrb	r2, [fp, #3984]	@ zero_extendqisi2
	mul	r1, r2, r1
	mov	r1, r1, asl #2
	str	r3, [sp, #48]
	bl	js_hash
	ldr	r2, [fp, #580]
	ldr	r3, [sp, #48]
	str	r0, [r3, #4]
	ldr	r3, [r6, #12]
	ldr	r2, [r2, #132]
	b	.L4166
.L4056:
	ldr	r2, [fp, #3992]
	mov	r0, r6
	ldr	r3, .L4175
	mov	r1, #1
	ldr	r2, [r2, r10, asl #2]
	str	r3, [sp, #48]
	str	r2, [r6, #24]
	bl	sblk_read_page
	ldr	r2, [r6, #36]
	cmn	r2, #1
	cmpne	r2, #512
	ldr	r3, [sp, #48]
	bne	.L4060
	ldr	r2, .L4175+56
	ldrb	r0, [r3, #1700]	@ zero_extendqisi2
	ldrh	r1, [r2]
	rsb	r0, r0, #24
	ldr	r2, [r6, #24]
	rsb	r0, r1, r0
	mov	r2, r2, lsr r1
	mvn	r1, #0
	bic	r0, r2, r1, asl r0
	ldrb	r1, [r3, #4030]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	mov	r1, #0
	uxth	r0, r0
	bl	ftl_sblk_dump
	ldr	r3, [r6, #36]
	cmn	r3, #1
	cmpne	r3, #512
	ldreq	r3, [r6, #12]
	mvneq	r2, #0
	streq	r2, [r3, #4]
.L4060:
	ldr	r3, [r6, #36]
	cmn	r3, #1
	cmpne	r3, #512
	bne	.L4062
	ldr	r1, .L4175+8
	movw	r2, #1003
	ldr	r0, .L4175+12
	bl	printk
	bl	dump_stack
.L4062:
	ldr	r2, [r6, #12]
	ldr	r3, .L4175
	ldr	r0, [r2, #4]
	ldr	r1, [r3, #3328]
	ldr	r3, [r3, #3988]
	cmp	r0, r1
	mvncs	r1, #0
	strcs	r1, [r2, #4]
	ldr	r2, [r6, #12]
	ldr	r2, [r2, #4]
	str	r2, [r3, r10, asl #2]
	ldr	r3, [r6, #12]
	ldr	r2, [r6, #24]
.L4166:
	str	r2, [r3, #8]
.L4054:
	ldr	r2, .L4175
	add	r1, sp, #80
	add	r1, r1, r9, asl #2
	ldr	lr, .L4175+16
	ldr	r3, [r2, #3996]
	ldrb	fp, [r3, r10]	@ zero_extendqisi2
	mov	r3, #48
	mul	r3, r3, fp
	add	lr, lr, r3
	str	lr, [r1, #-24]
	ldrb	r1, [sp, #32]	@ zero_extendqisi2
	add	r3, r2, r3
	strb	r1, [r3, #1829]
	mov	r1, #2
	strh	r1, [lr, #34]	@ movhi
	ldrb	r1, [r2, #4032]	@ zero_extendqisi2
	cmp	r1, #3
	bne	.L4064
	ldrb	r1, [r2, #1778]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L4065
	ldr	r1, .L4175+32
	and	r3, r9, #1
	add	r2, r5, r3
	ldrb	r1, [r1, #-2503]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L4066
	ldr	r1, [sp, #44]
	ldrh	r1, [r1]
	cmp	r1, r5
	movcs	r3, #0
	andcc	r3, r3, #1
	cmp	r3, #0
	movne	r2, r1
.L4066:
	ldr	r3, .L4175
	mov	r1, #48
	mov	r0, r9
	str	r2, [sp, #52]
	mla	r3, r1, fp, r3
	mov	r1, r4
	str	r3, [sp, #48]
	bl	__aeabi_uidiv
	ldr	r3, [sp, #36]
	ldr	r2, [sp, #52]
	uxtah	r0, r3, r0
	ldr	r3, [sp, #48]
	add	r0, r8, r0, asl #1
	ldrh	r1, [r0, #96]
	ldr	r0, .L4175+60
	ldrh	r0, [r0]
	mla	r2, r0, r1, r2
	orr	r2, r2, #50331648
	b	.L4167
.L4065:
	ldr	r0, .L4175+64
	ldr	r2, [r2, #4004]
	umull	r0, r1, r9, r0
	ldr	r0, [sp, #36]
	mov	r1, r1, lsr #1
	uxtah	r0, r0, r1
	add	r1, r1, r1, asl #1
	rsb	r1, r1, r9
	add	r0, r8, r0, asl #1
	mov	r1, r1, asl #24
	ldrh	fp, [r0, #96]
	add	r1, r1, #16777216
	ldr	r0, .L4175+60
	ldrh	r0, [r0]
	mul	r0, r0, fp
	orr	r0, r0, r5
	str	r0, [r3, #1808]
	ldrh	r3, [lr, #32]
	orr	r0, r1, r0
	str	r0, [r2, r3, asl #2]
	b	.L4068
.L4064:
	cmp	r1, #2
	bne	.L4067
	ldrb	r2, [r2, #4033]	@ zero_extendqisi2
	cmp	r2, #0
	ldr	r2, [sp, #36]
	bne	.L4070
	add	r2, r9, r2
	ldr	r1, .L4175+60
	add	r2, r8, r2, asl #1
	ldrh	r1, [r1]
	ldrh	r2, [r2, #96]
	mla	r2, r1, r2, r5
	b	.L4168
.L4070:
	add	r2, r2, r9, lsr #1
	ldr	r1, .L4175+60
	add	r2, r8, r2, asl #1
	ldrh	r1, [r1]
	ldrh	r2, [r2, #96]
	mla	r2, r1, r2, r5
	and	r1, r9, #1
	add	r2, r2, r1
.L4168:
	orr	r2, r2, #33554432
.L4167:
	str	r2, [r3, #1808]
.L4067:
	mov	r3, #48
	ldr	r2, .L4175
	mul	fp, r3, fp
	add	r3, r2, fp
	ldr	r1, [r3, #1808]
	ldr	r3, .L4175+16
	add	fp, r3, fp
	ldr	r3, [r2, #4004]
	ldrh	r0, [fp, #32]
	str	r1, [r3, r0, asl #2]
.L4068:
	ldr	r3, .L4175+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L4071
	ldr	r1, .L4175
	add	r3, sp, #80
	add	r3, r3, r9, asl #2
	ldr	r0, .L4175+64
	ldr	r2, [r3, #-24]
	ldrb	r3, [r1, #3386]	@ zero_extendqisi2
	ldr	r1, [r1, #3996]
	ldrb	r1, [r1, r10]	@ zero_extendqisi2
	str	r1, [sp]
	ldr	r1, [r2, #24]
	str	r1, [sp, #4]
	ldrh	r1, [r2, #32]
	str	r1, [sp, #8]
	ldrb	r1, [r2, #1]	@ zero_extendqisi2
	str	r1, [sp, #12]
	umull	r0, r1, r9, r0
	ldr	r0, [sp, #36]
	ubfx	r1, r1, #1, #16
	add	r1, r0, r1
	str	r1, [sp, #16]
	ldr	r2, [r2, #12]
	mov	r1, r9
	ldr	r0, .L4175+68
	ldr	r2, [r2, #4]
	str	r2, [sp, #20]
	ldr	r2, [r6, #12]
	ldr	r2, [r2]
	str	r2, [sp, #24]
	ldr	r2, [sp, #32]
	bl	printk
.L4071:
	ldr	r3, .L4175+32
	ldrb	r3, [r3, #-2503]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L4072
	ldr	r3, [sp, #44]
	ldrh	r3, [r3]
	cmp	r3, r5
	bcs	.L4072
	tst	r9, #1
	beq	.L4072
	ldr	fp, .L4175
	mvn	r2, #0
	ldr	r3, [fp, #3996]
	strb	r2, [r3, r10]
	add	r3, sp, #80
	add	ip, r3, r9, asl #2
	ldrb	r3, [r8, #89]	@ zero_extendqisi2
	ldr	r1, [ip, #-24]
	rsb	r3, r3, r3, asl #15
	mov	r3, r3, asl #1
	ldrb	r0, [r1, #1]	@ zero_extendqisi2
	smulbb	r3, r3, r4
	add	r3, r7, r3
	uxth	r3, r3
	strh	r3, [r1, #32]	@ movhi
	ldr	r1, [fp, #3996]
	strb	r0, [r1, r3]
	ldr	r3, [fp, #3992]
	str	r2, [r3, r10, asl #2]
.L4072:
	ldr	r3, [sp, #40]
	add	r3, r3, #1
	str	r3, [sp, #40]
	b	.L4053
.L4171:
	ldrb	r3, [fp, #4032]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L4074
	ldrb	r1, [fp, #1778]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L4172
	mov	r1, #0
.L4075:
	ldrb	r3, [r8, #89]	@ zero_extendqisi2
	uxth	r2, r1
	add	r1, r1, #1
	mul	r3, r4, r3
	sub	r3, r3, #1
	cmp	r2, r3
	bge	.L4173
	add	r3, sp, #80
	add	r0, sp, #80
	add	r3, r3, r2, asl #2
	add	r2, r0, r2, asl #2
	ldr	r2, [r2, #-20]
	ldr	r3, [r3, #-24]
	ldrb	r2, [r2, #1]	@ zero_extendqisi2
	strb	r2, [r3]
	b	.L4075
.L4173:
	add	r2, sp, #80
	add	r3, r2, r3, asl #2
	ldr	r3, [r3, #-24]
	b	.L4169
.L4172:
	ldrb	r2, [fp, #4035]	@ zero_extendqisi2
	ldr	r3, [sp, #56]
	cmp	r2, #0
	strneb	r1, [r3, #44]
	bne	.L4170
	ldr	r1, [sp, #32]
	cmp	r1, #1
	moveq	r2, #9
	beq	.L4082
	ldr	r1, [sp, #32]
	cmp	r1, #2
	moveq	r2, #13
.L4082:
	strb	r2, [r3, #44]
.L4170:
	add	r1, sp, #68
	ldrb	r2, [r8, #89]	@ zero_extendqisi2
	add	r0, sp, #56
	bl	sblk_xlc_prog_pages
	b	.L4078
.L4074:
	ldrb	r3, [fp, #4033]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L4163
	ldrb	r3, [fp, #4034]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L4163
	add	r0, sp, #56
	bl	sblk_3d_mlc_prog_pages
	b	.L4078
.L4163:
	ldrb	r2, [r8, #89]	@ zero_extendqisi2
	uxth	r1, r3
	add	r3, r3, #1
	mul	r2, r4, r2
	sub	r2, r2, #1
	cmp	r1, r2
	bge	.L4174
	add	r2, sp, #80
	add	r0, sp, #80
	add	r2, r2, r1, asl #2
	add	r1, r0, r1, asl #2
	ldr	r1, [r1, #-20]
	ldr	r2, [r2, #-24]
	ldrb	r1, [r1, #1]	@ zero_extendqisi2
	strb	r1, [r2]
	b	.L4163
.L4174:
	add	r3, sp, #80
	add	r2, r3, r2, asl #2
	ldr	r3, [r2, #-24]
.L4169:
	mvn	r2, #0
	strb	r2, [r3]
	ldrb	r1, [r8, #89]	@ zero_extendqisi2
	ldr	r0, [sp, #56]
	smulbb	r1, r4, r1
	uxtb	r1, r1
	bl	sblk_prog_page
.L4078:
	ldrb	r3, [fp, #4035]	@ zero_extendqisi2
	ldr	r2, .L4175
	cmp	r3, #0
	ldrb	r3, [r8, #89]	@ zero_extendqisi2
	addne	r3, r3, r3, asl #1
	bne	.L4086
	ldrb	r1, [r2, #4033]	@ zero_extendqisi2
	cmp	r1, #0
	movne	r3, r3, asl #1
	bne	.L4086
	ldrb	r2, [r2, #1778]	@ zero_extendqisi2
	cmp	r2, #0
	uxtheq	r3, r3
	beq	.L4086
	ldr	r1, .L4175+32
	uxth	r2, r3
	mul	r3, r4, r2
	ldrb	r1, [r1, #-2503]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L4086
	ldr	r0, .L4175+36
	mov	r1, r5, asl #1
	ldrh	r1, [r0, r1]
	cmp	r1, r5
	movcc	r3, r2
.L4086:
	ldr	r1, [fp, #3348]
	ldr	r2, [r1, #52]
	add	r2, r2, r3
	str	r2, [r1, #52]
	ldr	r1, .L4175+4
	movw	r2, #566
	ldrh	r4, [r1, r2]
	add	r4, r3, r4
	movw	r3, #3986
	uxth	r4, r4
	strh	r4, [r1, r2]	@ movhi
	ldrh	r2, [fp, r3]
	ldrb	r3, [r8, #89]	@ zero_extendqisi2
	mul	r3, r3, r2
	cmp	r4, r3
	ldrge	r3, .L4175
	movge	r2, #0
	ldrge	r3, [r3, #580]
	strgeh	r2, [r3, #86]	@ movhi
	bl	gc_write_completed
.L4026:
	add	sp, sp, #84
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4176:
	.align	2
.L4175:
	.word	.LANCHOR0
	.word	.LANCHOR0+3380
	.word	.LANCHOR1+2144
	.word	.LC0
	.word	.LANCHOR0+1784
	.word	.LANCHOR2
	.word	.LC300
	.word	.LC301
	.word	.LANCHOR5
	.word	.LANCHOR2+1960
	.word	.LANCHOR2+3496
	.word	.LANCHOR0+3986
	.word	.LC302
	.word	-178307901
	.word	.LANCHOR0+4028
	.word	.LANCHOR0+4076
	.word	-1431655765
	.word	.LC303
	.fnend
	.size	gc_do_copy_back, .-gc_do_copy_back
	.align	2
	.global	zftl_do_gc
	.type	zftl_do_gc, %function
zftl_do_gc:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #3336
	ldr	r4, .L4373
	cmp	r0, #1
	.pad #36
	sub	sp, sp, #36
	mov	r10, r0
	ldrh	r7, [r4, r3]
	movw	r3, #3332
	ldrh	r6, [r4, r3]
	movw	r3, #3334
	ldr	r5, [r4, #580]
	add	r6, r7, r6
	ldrh	r3, [r4, r3]
	uxth	r6, r6
	bne	.L4178
	ldr	r8, .L4373+4
	ldr	r9, [r8, #-2476]
	cmp	r9, #0
	bne	.L4179
	ldr	r2, [r8, #-2472]
	cmp	r2, #0
	beq	.L4178
.L4179:
	movw	r2, #4008
	ldrh	r2, [r4, r2]
	cmp	r6, r2, lsr #2
	bls	.L4178
	ldr	r1, .L4373
	movw	r2, #3352
	ldrh	r2, [r1, r2]
	cmp	r2, r6
	bcs	.L4178
	bl	timer_get_time
	add	r9, r9, #20
	ldr	r4, [r8, #-2472]
	add	r4, r4, #20
	cmp	r9, r0
	movcc	r3, #0
	strcc	r3, [r8, #-2476]
	bl	timer_get_time
	cmp	r4, r0
	ldrcc	r3, .L4373+4
	movcc	r2, #0
	strcc	r2, [r3, #-2472]
	b	.L4294
.L4178:
	ldrb	r2, [r4, #3370]	@ zero_extendqisi2
	cmp	r2, #6
	ldrls	pc, [pc, r2, asl #2]
	b	.L4294
.L4184:
	.word	.L4183
	.word	.L4185
	.word	.L4186
	.word	.L4187
	.word	.L4188
	.word	.L4295
	.word	.L4190
.L4183:
	add	r7, r3, r7
	movw	r2, #3338
	movw	r3, #3340
	ldrh	r9, [r4, r2]
	ldrh	r3, [r4, r3]
	uxth	r7, r7
	ldrh	r2, [r5, #80]
	add	r9, r3, r9
	movw	r3, #65535
	cmp	r2, r3
	ldr	r8, .L4373
	uxth	r9, r9
	beq	.L4191
	cmp	r10, #0
	bne	.L4192
	movw	r3, #3352
	ldrh	r3, [r8, r3]
	cmp	r6, r3, asl #1
	bge	.L4294
.L4192:
	ldr	r8, .L4373
	add	r10, r8, #3376
	add	r0, r10, #664
	ldrh	r1, [r10]
	add	r1, r1, #1
	uxth	r1, r1
	strh	r1, [r10]	@ movhi
	bl	_list_get_gc_head_node
	movw	r3, #65535
	cmp	r0, r3
	beq	.L4194
	ldr	r2, [r8, #576]
	mov	r3, r0, asl #1
	ldr	r1, [r8, #3372]
	ldrh	r3, [r2, r3]
	movw	r2, #3976
	ldrh	r2, [r8, r2]
	add	r1, r1, #1
	str	r1, [r8, #3372]
	cmp	r2, r3
	bcs	.L4195
	add	r2, r8, #564
	ldrh	r2, [r2]
	cmp	r1, r2, lsr #4
	bls	.L4194
	movw	r2, #3354
	ldrh	r2, [r8, r2]
	cmp	r2, r3
	bls	.L4194
.L4195:
	mov	r1, #0
	ldrb	r2, [r4, #3369]	@ zero_extendqisi2
	bl	gc_add_sblk
	ldr	r8, .L4373
	cmp	r0, #0
	beq	.L4196
	mov	r3, #1
	strb	r3, [r8, #3370]
	mov	r3, #0
	str	r3, [r8, #3372]
	b	.L4294
.L4194:
	mov	r3, #0
	strh	r3, [r10]	@ movhi
.L4196:
	cmp	r7, #15
	movls	r8, #2
	bls	.L4197
	ldr	r2, .L4373+8
	movw	r3, #3342
	ldrh	r3, [r4, r3]
	ldrh	r2, [r2]
	cmp	r3, r2
	movls	r3, #0
	movhi	r3, #1
	cmp	r9, #0
	movne	r9, r3
	orreq	r9, r3, #1
	cmp	r9, #0
	movne	r8, #2
	moveq	r8, #1
.L4197:
	ldr	r3, .L4373+12
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L4198
	ldr	r3, [r4, #580]
	ldrb	r2, [r4, #3369]	@ zero_extendqisi2
	str	r7, [sp]
	ldrh	r1, [r3, #124]
	ldr	r0, .L4373+16
	str	r1, [sp, #4]
	ldrh	r1, [r3, #120]
	str	r1, [sp, #8]
	movw	r1, #2171
	ldrh	r3, [r3, #122]
	str	r3, [sp, #12]
	ldrh	r3, [r5, #80]
	str	r3, [sp, #16]
	mov	r3, r6
	bl	printk
.L4198:
	ldrb	r0, [r4, #3369]	@ zero_extendqisi2
	mov	r1, r8
	mov	r2, #1
	bl	gc_search_src_blk
	cmp	r0, #0
	ble	.L4199
.L4200:
	mov	r3, #1
	b	.L4371
.L4199:
	ldr	r3, .L4373
	mov	r1, #3
	mov	r2, #1
	ldrb	r0, [r3, #3369]	@ zero_extendqisi2
	bl	gc_search_src_blk
	cmp	r0, #0
	bgt	.L4200
	b	.L4294
.L4191:
	ldr	fp, .L4373+12
	ldr	r3, [fp]
	tst	r3, #256
	beq	.L4201
	ldrb	r2, [r8, #3369]	@ zero_extendqisi2
	movw	r1, #2183
	str	r7, [sp]
	ldrh	r3, [r5, #124]
	ldr	r0, .L4373+16
	str	r3, [sp, #4]
	ldrh	r3, [r5, #120]
	str	r3, [sp, #8]
	ldrh	r3, [r5, #122]
	str	r10, [sp, #16]
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
.L4201:
	cmp	r10, #1
	bne	.L4202
	bl	gc_scan_static_data
	ldr	r3, [r4, #580]
	ldrh	r3, [r3, #122]
	cmp	r3, #0
	beq	.L4203
.L4204:
	mov	r3, #1
	strb	r3, [r4, #3369]
	b	.L4371
.L4203:
	bl	gc_static_wearleveling
	cmp	r0, #0
	bne	.L4204
	bl	gc_block_vpn_scan
	cmp	r7, #0
	beq	.L4299
	cmp	r6, r9
	bcs	.L4207
	ldr	r2, .L4373
	movw	r3, #4008
	ldrh	r3, [r2, r3]
	cmp	r3, r6
	bhi	.L4208
.L4207:
	movw	r1, #4008
	add	r3, r6, r9
	ldrh	r1, [r4, r1]
	ldr	r2, .L4373
	cmp	r3, r1
	blt	.L4208
	add	r3, r2, #4048
	movw	r1, #3342
	ldrh	r2, [r2, r1]
	ldrh	r3, [r3]
	cmp	r2, r3
	bcc	.L4299
.L4208:
	ldr	r5, .L4373
	mov	r8, #1
	ldr	r3, [r4, #3372]
	add	r2, r5, #564
	strb	r8, [r4, #3369]
	add	r3, r3, r8
	str	r3, [r4, #3372]
	ldrh	r2, [r2]
	cmp	r3, r2, lsr #5
	bls	.L4210
	movw	r3, #3342
	ldrh	r3, [r5, r3]
	cmp	r3, r7
	bls	.L4210
	ldr	r0, .L4373+20
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r3, #65535
	cmp	r0, r3
	beq	.L4299
	ldr	r3, [r5, #576]
	mov	r0, r0, asl r8
	ldrh	r2, [r3, r0]
	movw	r3, #3976
	ldrh	r1, [r5, r3]
	ldrb	r3, [r5, #3984]	@ zero_extendqisi2
	mul	r3, r3, r1
	cmp	r2, r3
	bgt	.L4299
	mov	r3, #0
	mov	r0, r8
	str	r3, [r5, #3372]
	mov	r1, #2
	b	.L4363
.L4210:
	ldr	r0, .L4373
	movw	r2, #3342
	ldrh	r2, [r4, r2]
	add	r3, r0, #4048
	ldrh	r3, [r3]
	cmp	r2, r3
	movcs	r0, #1
	movcs	r1, #2
	movcs	r2, r0
	bcs	.L4364
	movw	r3, #3338
	ldrh	r5, [r0, r3]
	cmp	r5, #0
	bne	.L4214
	movw	r3, #3340
	ldrh	r3, [r0, r3]
	cmp	r3, #8
	bls	.L4206
.L4214:
	mov	r0, #1
	mov	r1, r0
.L4363:
	mov	r2, #4
.L4364:
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L4200
	b	.L4299
.L4202:
	movw	r3, #3352
	ldrh	r3, [r4, r3]
	cmp	r3, r6
	bcc	.L4294
.L4299:
	mov	r5, #16
.L4206:
	ldr	r3, [r4, #3364]
	ldr	r8, .L4373
	cmp	r3, #0
	beq	.L4215
	mov	ip, #0
	ldr	r0, .L4373+20
	mov	r1, ip
	mov	r2, #1
	str	ip, [r8, #3364]
	strb	r2, [r8, #3369]
	str	r2, [sp, #28]
	str	ip, [sp, #24]
	bl	_list_get_gc_head_node
	movw	r1, #65535
	cmp	r0, r1
	beq	.L4216
	ldr	r1, [r8, #576]
	mov	r3, r0, asl #1
	ldr	ip, [sp, #24]
	ldr	r2, [sp, #28]
	ldrh	r3, [r1, r3]
	cmp	r3, #8
	bhi	.L4216
	mov	r1, ip
	str	r2, [r8, #3364]
	bl	gc_add_sblk
	cmp	r0, #0
	bne	.L4200
.L4216:
	ldr	r0, .L4373+24
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r2, #65535
	ldr	r8, .L4373
	cmp	r0, r2
	beq	.L4217
	ldr	r2, [r8, #576]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #4
	bhi	.L4217
	mov	r2, #1
	mov	r1, #0
	str	r2, [r8, #3364]
	bl	gc_add_sblk
	cmp	r0, #0
	bne	.L4200
.L4217:
	ldr	r0, .L4373+28
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r2, #65535
	ldr	r8, .L4373
	cmp	r0, r2
	beq	.L4215
	ldr	r2, [r8, #576]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #4
	bhi	.L4215
	mov	r1, #1
	mov	r2, #0
	str	r1, [r8, #3364]
	bl	gc_add_sblk
	cmp	r0, #0
	bne	.L4200
.L4215:
	ldr	ip, [r4, #580]
	mov	r0, #1
	ldr	r8, .L4373
	strb	r0, [r4, #3369]
	ldrh	r3, [ip, #124]
	cmp	r3, #0
	beq	.L4218
	ldr	r1, [fp]
	mov	r2, #0
	strb	r0, [r8, #3370]
	tst	r1, #256
	strb	r2, [r8, #3369]
	beq	.L4219
	str	r7, [sp]
	movw	r1, #2273
	str	r3, [sp, #4]
	ldrh	r3, [ip, #120]
	ldr	r0, .L4373+32
	str	r3, [sp, #8]
	ldrh	r3, [ip, #122]
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
	b	.L4219
.L4218:
	movw	r2, #3352
	ldrh	r1, [r8, r2]
	cmp	r6, r1
	bcs	.L4220
	cmp	r7, #0
	beq	.L4221
	cmp	r7, #16
	bls	.L4222
	add	r2, r8, #4048
	movw	r1, #3342
	ldrh	r1, [r8, r1]
	ldrh	r2, [r2]
	cmp	r1, r2
	bhi	.L4222
	mov	r1, r0
	mov	r2, #4
	str	r3, [sp, #24]
	bl	gc_search_src_blk
	uxth	r0, r0
	ldr	r3, [sp, #24]
	cmp	r0, #0
	ldreqb	r0, [r8, #3369]	@ zero_extendqisi2
	beq	.L4369
	mov	r1, r3
	ldr	r0, .L4373+20
	bl	_list_get_gc_head_node
	movw	r3, #65535
	cmp	r0, r3
	beq	.L4292
	ldr	r1, [r8, #576]
	mov	r2, r0, asl #1
	ldr	r3, [r8, #3372]
	ldrh	r2, [r1, r2]
	movw	r1, #3976
	ldrh	r1, [r8, r1]
	add	r3, r3, #1
	str	r3, [r8, #3372]
	cmp	r1, r2
	bcs	.L4226
	add	r1, r8, #564
	ldrh	r1, [r1]
	cmp	r3, r1, lsr #4
	bls	.L4292
	movw	r3, #3354
	ldrh	r3, [r8, r3]
	cmp	r3, r2
	bls	.L4292
.L4226:
	mov	r1, #0
	ldrb	r2, [r4, #3369]	@ zero_extendqisi2
	bl	gc_add_sblk
	mov	r3, #1
	str	r3, [r4, #3364]
	mov	r3, #0
	str	r3, [r4, #3372]
	b	.L4292
.L4222:
	mov	r0, #1
	mov	r1, #2
	mov	r2, r0
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	ldreqb	r0, [r4, #3369]	@ zero_extendqisi2
	bne	.L4292
	b	.L4367
.L4221:
	ldr	r3, [fp]
	strb	r7, [r8, #3369]
	tst	r3, #256
	beq	.L4228
	str	r7, [sp]
	movw	r1, #2303
	str	r7, [sp, #4]
	mov	r2, r7
	ldrh	r3, [ip, #120]
	ldr	r0, .L4373+32
	str	r3, [sp, #8]
	ldrh	r3, [ip, #122]
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
.L4228:
	cmp	r6, #16
	ldrb	r0, [r4, #3369]	@ zero_extendqisi2
	bls	.L4229
.L4369:
	mov	r1, #3
	mov	r2, #4
	b	.L4366
.L4229:
	mov	r1, #1
	mov	r2, r1
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L4292
	ldr	r3, .L4373
	ldrb	r0, [r3, #3369]	@ zero_extendqisi2
.L4367:
	mov	r1, #3
	mov	r2, #2
.L4366:
	bl	gc_search_src_blk
	uxth	r0, r0
	b	.L4224
.L4220:
	cmp	r10, #1
	bne	.L4219
	cmp	r6, r1, asl #1
	bge	.L4230
	cmp	r9, r7, lsr #1
	bcs	.L4231
	add	r3, r8, #4048
	movw	r1, #3342
	ldrh	r1, [r8, r1]
	ldrh	r3, [r3]
	cmp	r1, r3
	movwcc	r3, #4008
	ldrcch	r3, [r8, r3]
	movcc	r3, r3, lsr #2
	strcch	r3, [r8, r2]	@ movhi
	bcc	.L4219
.L4231:
	ldr	r0, .L4373+20
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r3, #65535
	ldr	r8, .L4373
	cmp	r0, r3
	mov	r10, r0
	bne	.L4232
.L4237:
	cmp	r7, #1
	bhi	.L4233
	b	.L4234
.L4232:
	add	r2, r8, #564
	ldr	r3, [r8, #3372]
	mov	fp, r0, asl #1
	ldrh	r2, [r2]
	add	r3, r3, #1
	str	r3, [r8, #3372]
	cmp	r3, r2, lsr #4
	bls	.L4235
	ldr	r3, [r8, #576]
	mov	r1, #0
	str	r1, [r8, #3372]
	ldrh	r2, [r3, fp]
	movw	r3, #3354
	ldrh	r3, [r8, r3]
	cmp	r2, r3
	bcs	.L4235
	mov	r2, #1
	bl	gc_add_sblk
	cmp	r0, #0
	bne	.L4372
.L4235:
	ldr	r3, [r4, #576]
	ldr	r8, .L4373
	ldrh	r2, [r3, fp]
	movw	r3, #3976
	ldrh	r3, [r4, r3]
	cmp	r2, r3, lsr #1
	bhi	.L4236
	mov	r0, r10
	mov	r1, #1
	mov	r2, #0
	bl	gc_add_sblk
.L4372:
	mov	r3, #1
	strb	r3, [r8, #3370]
	b	.L4219
.L4236:
	movw	r3, #3338
	movw	r1, #3340
	ldrh	r1, [r8, r1]
	ldrh	r3, [r8, r3]
	add	r3, r3, r1
	movw	r1, #4008
	ldrh	r1, [r8, r1]
	cmp	r3, r1, asl #1
	ble	.L4237
	movw	r3, #3354
	ldrh	r3, [r8, r3]
	cmp	r3, r2
	bcc	.L4234
	b	.L4237
.L4233:
	cmp	r7, #16
	mov	r0, #1
	ldr	r8, .L4373
	strb	r0, [r4, #3369]
	bls	.L4238
	add	r3, r8, #4048
	movw	r2, #3342
	ldrh	r2, [r8, r2]
	ldrh	r3, [r3]
	cmp	r2, r3
	bhi	.L4238
	mov	r1, r0
	mov	r2, #4
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	ldreqb	r0, [r8, #3369]	@ zero_extendqisi2
	moveq	r1, #3
	moveq	r2, #4
	bne	.L4239
	b	.L4365
.L4238:
	mov	r0, #1
	mov	r1, #2
	mov	r2, r0
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L4239
	ldrb	r0, [r4, #3369]	@ zero_extendqisi2
	mov	r1, #3
	mov	r2, #2
.L4365:
	bl	gc_search_src_blk
	uxth	r0, r0
.L4239:
	movw	r3, #4008
	cmp	r6, r9, lsr #1
	ldrh	r3, [r4, r3]
	movw	r2, #3352
	movhi	r3, r3, lsr #2
	movls	r3, r3, lsr #1
	strh	r3, [r4, r2]	@ movhi
	b	.L4224
.L4234:
	cmp	r6, r9
	bcs	.L4241
	ldr	r0, .L4373+28
	mov	r1, #4
	mov	r7, #0
	strb	r7, [r4, #3369]
	bl	_list_get_gc_head_node
	movw	r3, #65535
	ldr	r6, .L4373
	cmp	r0, r3
	beq	.L4242
	ldr	r3, [r6, #576]
	mov	r0, r0, asl #1
	ldrh	r2, [r3, r0]
	movw	r3, #3976
	ldrh	r1, [r6, r3]
	ldrb	r3, [r6, #3984]	@ zero_extendqisi2
	mul	r3, r3, r1
	cmp	r2, r3, asr #1
	ble	.L4243
.L4242:
	movw	r3, #4008
	movw	r2, #3352
	ldrh	r3, [r6, r3]
	mov	r3, r3, lsr #2
	strh	r3, [r6, r2]	@ movhi
	b	.L4219
.L4243:
	mov	r2, #4
	mov	r0, r7
	mov	r1, #3
	bl	gc_search_src_blk
	movw	r3, #4008
	ldrh	r3, [r6, r3]
	movw	r2, #3352
	mov	r3, r3, lsr #1
	strh	r3, [r6, r2]	@ movhi
	uxth	r0, r0
	b	.L4224
.L4241:
	movw	r3, #4008
	movw	r2, #3352
	ldrh	r3, [r4, r3]
	mov	r3, r3, lsr #2
	strh	r3, [r4, r2]	@ movhi
	b	.L4219
.L4230:
	movw	r1, #4008
	mov	r5, r3
	ldrh	r1, [r8, r1]
	mov	r1, r1, lsr #2
	strh	r1, [r8, r2]	@ movhi
	b	.L4219
.L4224:
	cmp	r0, #0
	beq	.L4219
.L4292:
	mov	r3, #1
	strb	r3, [r4, #3370]
	b	.L4219
.L4185:
	movw	r5, #3380
	movw	r3, #65535
	ldrh	r2, [r4, r5]
	cmp	r2, r3
	bne	.L4244
	bl	gc_get_src_blk
	ldr	r3, .L4373
	strh	r0, [r3, r5]	@ movhi
.L4244:
	movw	r3, #3380
	movw	r0, #65535
	ldrh	r1, [r4, r3]
	ldr	r2, .L4373
	cmp	r1, r0
	beq	.L4245
	add	r0, r2, #564
	ldrh	r0, [r0]
	cmp	r0, r1
	mvnls	r1, #0
	strlsh	r1, [r2, r3]	@ movhi
.L4245:
	movw	r3, #3380
	ldr	r2, .L4373
	ldrh	r3, [r4, r3]
	movw	r1, #65535
	cmp	r3, r1
	moveq	r3, #0
	streqb	r3, [r2, #3370]
	beq	.L4294
	ldr	r1, .L4373+52
	ldr	r2, [r2, #568]
	ldrh	r0, [r1, #52]
	add	r2, r2, r3, asl #2
	cmp	r0, #0
	addne	ip, r1, #52
	movne	r1, #0
	beq	.L4247
.L4248:
	uxth	lr, r1
	cmp	lr, r0
	bcs	.L4247
	ldrh	lr, [ip, #2]!
	add	r1, r1, #1
	cmp	lr, r3
	bne	.L4248
.L4252:
	movw	r3, #3380
	mvn	r2, #0
	strh	r2, [r4, r3]	@ movhi
	b	.L4294
.L4247:
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	tst	r2, #192
	and	r2, r2, #224
	moveq	r1, #1
	movne	r1, #0
	cmp	r2, #224
	movne	r2, r1
	orreq	r2, r1, #1
	cmp	r2, #0
	beq	.L4250
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L4252
	ldr	r1, .L4373+60
	movw	r2, #2538
	ldr	r0, .L4373+64
	bl	printk
	bl	dump_stack
	b	.L4252
.L4250:
	mov	r3, #2
	b	.L4371
.L4186:
	bl	gc_scan_src_blk
	cmn	r0, #1
	moveq	r3, #3
	beq	.L4371
	movw	r3, #3380
	ldr	r1, .L4373+52
	ldrh	r3, [r4, r3]
	movw	r0, #65535
	ldr	r2, .L4373
	cmp	r3, r0
	mov	r6, r1
	beq	.L4200
	ldrh	r0, [r1, #20]
	cmp	r0, #0
	movne	r3, #4
	strneb	r3, [r2, #3370]
	movne	r3, #0
	strneh	r3, [r1, #22]	@ movhi
	bne	.L4294
.L4254:
	mov	r1, #1
	strb	r1, [r2, #3370]
	ldr	r2, [r2, #576]
	mov	r3, r3, asl r1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L4255
	ldr	r1, .L4373+60
	movw	r2, #2566
	ldr	r0, .L4373+64
	bl	printk
	bl	dump_stack
.L4255:
	movw	r5, #3380
	ldr	r7, .L4373+52
	ldrh	r0, [r4, r5]
	bl	ftl_free_sblk
	ldrh	r3, [r4, r5]
	ldr	r2, [r4, #576]
	mov	r5, #0
	mov	r3, r3, asl #1
	strh	r5, [r2, r3]	@ movhi
	ldrh	r3, [r6, #26]
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #8
	strlsh	r3, [r7, #26]	@ movhi
	bls	.L4252
	strh	r5, [r7, #26]	@ movhi
	bl	ftl_flush
	bl	pm_flush
	bl	ftl_ext_info_flush
	mov	r0, r5
	bl	ftl_info_flush
	b	.L4252
.L4187:
	ldr	r7, .L4373+52
.L4340:
	bl	gc_scan_src_blk_one_page
	ldr	r3, .L4373+36
	ldrh	r2, [r7, #2]
	ldr	r0, .L4373
	ldrh	r3, [r3]
	ldr	r5, .L4373+52
	cmp	r2, r3
	bcs	.L4258
	cmp	r6, #7
	bls	.L4340
	b	.L4294
.L4258:
	ldrh	r3, [r5, #20]
	ldr	r2, .L4373+12
	cmp	r3, #0
	beq	.L4259
	ldr	r2, [r2]
	mov	r1, #4
	strb	r1, [r0, #3370]
	mov	r1, #0
	tst	r2, #256
	strh	r1, [r5, #22]	@ movhi
	beq	.L4260
	ldrh	r1, [r5]
	ldr	ip, [r0, #576]
	ldr	r0, .L4373+40
	mov	r2, r1, asl #1
	ldrh	r2, [ip, r2]
	bl	printk
.L4260:
	movw	r3, #3380
	ldr	r2, [r4, #576]
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	ldrh	r3, [r5, #20]
	cmp	r2, r3
	beq	.L4261
	ldr	r1, .L4373+60
	movw	r2, #2600
	ldr	r0, .L4373+64
	bl	printk
	bl	dump_stack
.L4261:
	movw	r3, #3380
	ldrh	r1, [r5, #20]
	ldrh	r3, [r4, r3]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	b	.L4294
.L4259:
	mov	r3, #1
	strb	r3, [r0, #3370]
	ldr	r3, [r2]
	ldrh	r1, [r5]
	ldr	r6, [r0, #568]
	tst	r3, #256
	add	r6, r6, r1, asl #2
	beq	.L4262
	ldrb	r2, [r6, #2]	@ zero_extendqisi2
	ldr	r0, .L4373+44
	mov	r2, r2, lsr #5
	bl	printk
.L4262:
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	tst	r3, #192
	and	r3, r3, #224
	moveq	r2, #1
	movne	r2, #0
	cmp	r3, #224
	movne	r3, r2
	orreq	r3, r2, #1
	cmp	r3, #0
	beq	.L4263
	ldr	r1, .L4373+60
	movw	r2, #2610
	ldr	r0, .L4373+64
	bl	printk
	bl	dump_stack
.L4263:
	movw	r7, #3380
	ldr	r6, .L4373+52
	ldrh	r0, [r4, r7]
	bl	ftl_free_sblk
	mvn	r3, #0
	strh	r3, [r4, r7]	@ movhi
	ldrh	r3, [r5, #26]
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #8
	strlsh	r3, [r6, #26]	@ movhi
	movhi	r3, #0
	strhih	r3, [r6, #26]	@ movhi
	bls	.L4294
	b	.L4370
.L4188:
	cmp	r10, #0
	bne	.L4265
	movw	r3, #3352
	ldrh	r3, [r4, r3]
	cmp	r3, r6
	bcc	.L4294
.L4265:
	ldrh	r2, [r5, #80]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L4266
	ldrb	r7, [r4, #3369]	@ zero_extendqisi2
	cmp	r7, #1
	bne	.L4266
	ldr	r8, .L4373+52
	bl	ftl_flush
	movw	r3, #570
	mov	r1, #5
	ldrh	r0, [r8, r3]
	cmp	r0, #0
	movne	r0, r7
	bl	zftl_gc_get_free_sblk
	movw	r3, #65535
	cmp	r0, r3
	mov	r6, r0
	beq	.L4269
	ldr	r7, [r4, #568]
	add	r7, r7, r0, asl #2
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	tst	r3, #224
	beq	.L4270
	ldr	r1, .L4373+60
	movw	r2, #2639
	ldr	r0, .L4373+64
	bl	printk
	bl	dump_stack
.L4270:
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	and	r3, r3, #15
	orr	r3, r3, #176
	strb	r3, [r7, #2]
.L4293:
	mov	r0, r6
	mov	r1, #1
	bl	ftl_erase_sblk
	mov	r3, #5
	add	r1, r5, #96
	strb	r3, [r5, #84]
	mov	r0, r6
	mov	r7, #0
	bl	ftl_get_blk_list_in_sblk
	movw	r3, #3976
	ldrh	r3, [r4, r3]
	mov	r1, #255
	strh	r6, [r5, #80]	@ movhi
	strh	r7, [r5, #82]	@ movhi
	strb	r7, [r5, #85]
	strh	r7, [r5, #90]	@ movhi
	uxtb	r0, r0
	strb	r0, [r5, #89]
	smulbb	r0, r3, r0
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	strh	r0, [r5, #86]	@ movhi
	movw	r5, #3986
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #3988]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r4, r5]
	ldrb	r3, [r4, #3984]	@ zero_extendqisi2
	mov	r1, #255
	ldr	r0, [r4, #3992]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r3, [r4, r5]
	ldrb	r2, [r4, #3984]	@ zero_extendqisi2
	mov	r1, #255
	ldr	r0, [r4, #3996]
	mvn	r5, #0
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r3, [r4, #580]
	strh	r5, [r3, #128]	@ movhi
	strh	r5, [r3, #130]	@ movhi
	str	r6, [r3, #132]
	bl	pm_flush
	bl	ftl_ext_info_flush
	ldr	r3, [r4, #3348]
	mov	r0, r7
	strh	r7, [r8, #52]	@ movhi
	strh	r6, [r3, #126]	@ movhi
	movw	r3, #566
	strh	r7, [r8, r3]	@ movhi
	ldr	r3, .L4373+48
	str	r5, [r4, #3956]
	strh	r7, [r3], #4	@ movhi
	strh	r7, [r3]	@ movhi
	bl	ftl_info_flush
	b	.L4294
.L4266:
	cmp	r10, #1
	ldr	r8, .L4373
	ldr	r9, .L4373+52
	movne	r7, #1
	moveq	r7, #4
	cmp	r6, #15
	addls	r7, r7, #4
.L4273:
	sub	r7, r7, #1
	uxtb	r7, r7
	cmp	r7, #255
	beq	.L4294
	bl	gc_do_copy_back
	ldrb	r3, [r4, #3369]	@ zero_extendqisi2
	ldr	r10, .L4373
	cmp	r3, #0
	ldr	r6, .L4373+52
	bne	.L4274
	ldrb	r3, [r8, #3322]	@ zero_extendqisi2
	cmp	r3, #3
	bhi	.L4275
	bl	ftl_write_commit
.L4275:
	ldrh	r2, [r9, #22]
	ldrh	r3, [r9, #20]
	ldr	r6, .L4373
	cmp	r2, r3
	bcc	.L4273
	mov	r3, #1
	strb	r3, [r6, #3370]
	bl	ftl_write_commit
	bl	ftl_flush
	ldr	r3, .L4373+52
	ldr	r2, [r6, #576]
	ldrh	r3, [r3]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L4277
	ldr	r1, .L4373+60
	movw	r2, #2714
	ldr	r0, .L4373+64
	bl	printk
	bl	dump_stack
.L4277:
	movw	r3, #3380
	ldr	r1, [r4, #576]
	ldrh	r0, [r4, r3]
	mov	r2, r0, asl #1
	ldrh	ip, [r1, r2]
	cmp	ip, #0
	bne	.L4278
	strh	ip, [r1, r2]	@ movhi
	ldr	r2, .L4373
	ldrh	r0, [r2, r3]
	bl	ftl_free_sblk
	b	.L4252
.L4278:
	mov	r1, #0
	mov	r2, #1
	bl	gc_add_sblk
	b	.L4252
.L4374:
	.align	2
.L4373:
	.word	.LANCHOR0
	.word	.LANCHOR5
	.word	.LANCHOR0+4048
	.word	.LANCHOR2
	.word	.LC304
	.word	.LANCHOR0+4040
	.word	.LANCHOR0+4036
	.word	.LANCHOR0+4044
	.word	.LC305
	.word	.LANCHOR0+3976
	.word	.LC306
	.word	.LC307
	.word	.LANCHOR0+3948
	.word	.LANCHOR0+3380
	.word	.LANCHOR0
	.word	.LANCHOR1+2160
	.word	.LC0
.L4274:
	add	r3, r10, #3952
	ldrh	r2, [r3]
	cmp	r2, #0
	beq	.L4279
	mov	r5, #0
	strh	r5, [r3]	@ movhi
	bl	sblk_wait_write_queue_completed
	bl	gc_write_completed
	ldr	r0, [r10, #3956]
	cmn	r0, #1
	beq	.L4280
	bl	ftl_mask_bad_block
.L4280:
	ldr	r3, [r4, #580]
	str	r5, [r4, #3956]
	strh	r5, [r6, #52]	@ movhi
	ldrh	r0, [r3, #80]
	bl	ftl_free_sblk
	ldr	r0, [r4, #3388]
	ldr	r2, [r4, #580]
	mvn	r3, #0
	ldr	r1, [r4, #3348]
	cmp	r0, #0
	strh	r3, [r2, #80]	@ movhi
	strh	r3, [r1, #126]	@ movhi
	strh	r3, [r2, #130]	@ movhi
	beq	.L4281
	bl	zbuf_free
.L4281:
	mov	r5, #0
	str	r5, [r4, #3388]
	bl	flt_sys_flush
	ldr	r1, .L4373+60
	mov	r2, #2752
	ldr	r0, .L4373+64
	strb	r5, [r4, #3370]
	bl	printk
	bl	dump_stack
	b	.L4294
.L4279:
	ldrh	r3, [r5, #86]
	ldrh	r2, [r9, #22]
	cmp	r3, #1
	ldrh	r3, [r9, #20]
	bls	.L4282
	cmp	r2, r3
	bcc	.L4273
	mov	r3, #1
	strb	r3, [r10, #3370]
	ldrh	r3, [r6, #52]
	add	r2, r3, #1
	strh	r2, [r6, #52]	@ movhi
	ldrh	r2, [r6]
	add	r3, r6, r3, asl #1
	strh	r2, [r3, #54]	@ movhi
	mvn	r3, #0
	strh	r3, [r6]	@ movhi
	b	.L4294
.L4282:
	cmp	r2, r3
	mov	r1, #5
	strb	r1, [r10, #3370]
	bcc	.L4283
	ldrh	r3, [r6, #52]
	add	r2, r3, #1
	strh	r2, [r6, #52]	@ movhi
	ldrh	r2, [r6]
	add	r3, r6, r3, asl #1
	strh	r2, [r3, #54]	@ movhi
	mvn	r3, #0
	strh	r3, [r6]	@ movhi
.L4283:
	bl	ftl_flush
	bl	sblk_wait_write_queue_completed
	bl	gc_write_completed
	ldrh	r2, [r5, #80]
	ldr	r3, [r4, #580]
	strh	r2, [r3, #128]	@ movhi
	bl	pm_flush
	bl	ftl_ext_info_flush
	ldrb	r1, [r4, #1778]	@ zero_extendqisi2
	mov	r3, #0
	strh	r3, [r6, #12]	@ movhi
	cmp	r1, #0
	movw	r3, #3976
	ldrh	r3, [r4, r3]
	ldrne	r0, .L4373+56
	movwne	r1, #3986
	ldr	r2, .L4373+52
	strh	r3, [r6, #14]	@ movhi
	ldrb	r3, [r4, #4032]	@ zero_extendqisi2
	strh	r3, [r6, #16]	@ movhi
	ldrneh	r1, [r0, r1]
	strneh	r1, [r2, #14]	@ movhi
	movne	r1, #1
	strneh	r1, [r2, #16]	@ movhi
	cmp	r3, #2
	bne	.L4286
	ldrh	r3, [r6, #14]
	mov	r3, r3, asl #1
	strh	r3, [r6, #14]	@ movhi
	ldrb	r3, [r4, #4033]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L4373+52
	moveq	r2, #1
	streqh	r2, [r3, #16]	@ movhi
.L4286:
	mov	r3, #0
	strh	r3, [r6, #18]	@ movhi
	b	.L4294
.L4295:
	ldr	r8, .L4373+52
	mov	r5, #0
.L4189:
	bl	gc_check_data_one_wl
	subs	r9, r0, #0
	beq	.L4289
	ldr	r3, .L4373+52
	mov	r5, #0
	strh	r5, [r3, #52]	@ movhi
	ldr	r3, [r4, #580]
	ldrh	r0, [r3, #80]
	bl	ftl_free_sblk
	ldr	r2, [r4, #580]
	ldr	r1, [r4, #3348]
	mvn	r3, #0
	ldr	r0, [r4, #3388]
	strh	r3, [r2, #80]	@ movhi
	strh	r3, [r1, #126]	@ movhi
	strh	r3, [r2, #130]	@ movhi
	bl	zbuf_free
	str	r5, [r4, #3388]
	strb	r5, [r4, #3370]
.L4370:
	bl	flt_sys_flush
	b	.L4294
.L4289:
	ldrh	r2, [r8, #12]
	ldrh	r3, [r8, #14]
	ldr	r7, .L4373+56
	cmp	r2, r3
	bcc	.L4290
	ldr	r0, [r7, #3388]
	mov	r3, #6
	strb	r3, [r7, #3370]
	bl	zbuf_free
	str	r9, [r7, #3388]
	b	.L4294
.L4290:
	cmp	r6, #15
	bls	.L4189
	cmp	r10, #1
	bne	.L4294
	add	r5, r5, #1
	uxtb	r5, r5
	cmp	r5, #4
	bls	.L4189
	b	.L4294
.L4190:
	bl	gc_update_l2p_map_new
	mvn	r6, #0
	bl	gc_free_src_blk
	bl	ftl_flush
	bl	pm_flush
	strh	r6, [r5, #80]	@ movhi
	bl	ftl_ext_info_flush
	ldr	r3, [r4, #3348]
	mov	r0, #0
	strh	r6, [r3, #126]	@ movhi
	bl	ftl_info_flush
	mov	r3, #0
.L4371:
	strb	r3, [r4, #3370]
.L4294:
	mov	r0, #16
	b	.L4361
.L4219:
	mov	r0, r5
	b	.L4361
.L4269:
	ldr	r1, .L4373+60
	movw	r2, #2645
	ldr	r0, .L4373+64
	bl	printk
	bl	dump_stack
	b	.L4293
.L4361:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
	.fnend
	.size	zftl_do_gc, .-zftl_do_gc
	.align	2
	.global	zftl_write
	.type	zftl_write, %function
zftl_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r3
	ldr	r3, .L4403
	.pad #28
	sub	sp, sp, #28
	mov	r5, r0
	mov	r4, r1
	mov	r7, r2
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L4376
	ldr	r3, [r9]
	mov	r1, r5
	ldr	r0, .L4403+4
	mov	r2, r4
	str	r3, [sp]
	mov	r3, r7
	bl	printk
.L4376:
	cmp	r5, #0
	ldreq	r3, .L4403+8
	moveq	r5, #24576
	ldreq	r3, [r3, #520]
	beq	.L4378
.L4377:
	cmp	r5, #3
	bhi	.L4392
	mov	r5, r5, asl #13
	mov	r3, #8192
.L4378:
	add	r2, r4, r7
	cmp	r2, r3
	bhi	.L4392
	ldr	r8, .L4403+8
	add	r4, r5, r4
	mov	r0, r4
	ldrb	r5, [r8, #3320]	@ zero_extendqisi2
	mov	r1, r5
	bl	__aeabi_uidiv
	mov	r1, r5
	mov	r10, r0
	sub	r0, r7, #1
	add	r0, r0, r4
	mov	r6, r10
	bl	__aeabi_uidiv
	add	r3, r4, r7
	rsb	r5, r10, r0
	str	r0, [sp, #12]
	add	r5, r5, #1
	str	r3, [sp, #16]
.L4380:
	cmp	r5, #0
	beq	.L4401
	mov	r0, #0
	bl	buf_alloc
	subs	fp, r0, #0
	bne	.L4381
	bl	ftl_write_commit
	b	.L4380
.L4381:
	mov	r3, #0
	strb	r3, [fp, #41]
	ldr	r3, [sp, #12]
	rsb	r0, r10, r6
	clz	r0, r0
	ldrb	r2, [r8, #3320]	@ zero_extendqisi2
	rsb	r1, r3, r6
	clz	r1, r1
	mov	r0, r0, lsr #5
	strb	r2, [fp, #40]
	mov	r1, r1, lsr #5
	orrs	r3, r1, r0
	beq	.L4384
	cmp	r0, #0
	beq	.L4385
	mov	r1, r2
	mov	r0, r4
	str	r2, [sp, #20]
	bl	__aeabi_uidivmod
	ldr	r2, [sp, #20]
	uxtb	r1, r1
	strb	r1, [fp, #41]
	rsb	r2, r1, r2
	uxtb	r2, r2
	cmp	r2, r7
	strhib	r7, [fp, #40]
	bhi	.L4384
	b	.L4400
.L4385:
	cmp	r1, #0
	beq	.L4384
	smulbb	r2, r6, r2
	ldr	r3, [sp, #16]
	rsb	r2, r2, r3
.L4400:
	strb	r2, [fp, #40]
.L4384:
	ldrb	r0, [fp, #41]	@ zero_extendqisi2
	sub	r5, r5, #1
	ldrb	r2, [fp, #40]	@ zero_extendqisi2
	ldr	r1, [fp, #4]
	mov	r2, r2, asl #9
	add	r0, r1, r0, asl #9
	mov	r1, r9
	bl	ftl_memcpy
	ldr	r1, [r8, #3348]
	str	r6, [fp, #20]
	add	r6, r6, #1
	ldr	r2, [r1, #8]
	add	r0, r2, #1
	str	r0, [r1, #8]
	str	r2, [fp, #16]
	mov	r0, fp
	bl	ftl_write_buf
	ldrb	r3, [fp, #40]	@ zero_extendqisi2
	add	r9, r9, r3, asl #9
	b	.L4380
.L4401:
	bl	ftl_write_commit
	mov	r0, r5
	mov	r1, #1
	ldr	r4, .L4403+12
	bl	zftl_do_gc
.L4389:
	ldr	r2, .L4403+16
	ldrh	r3, [r4]
	ldrh	r2, [r2]
	add	r3, r3, r2
	cmp	r3, #7
	bgt	.L4402
	mov	r1, #1
	mov	r0, #0
	bl	zftl_do_gc
	b	.L4389
.L4402:
	bl	timer_get_time
	ldr	r3, .L4403+20
	str	r0, [r3, #-2472]
	mov	r0, #0
	b	.L4379
.L4392:
	mvn	r0, #0
.L4379:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4404:
	.align	2
.L4403:
	.word	.LANCHOR2
	.word	.LC308
	.word	.LANCHOR0
	.word	.LANCHOR0+3332
	.word	.LANCHOR0+3336
	.word	.LANCHOR5
	.fnend
	.size	zftl_write, .-zftl_write
	.align	2
	.global	zftl_vendor_write
	.type	zftl_vendor_write, %function
zftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	mov	ip, r1
	mov	r3, r2
	add	r1, r0, #512
	mov	r2, ip
	mov	r0, #2
	b	zftl_write
	.fnend
	.size	zftl_vendor_write, .-zftl_vendor_write
	.align	2
	.global	zftl_sys_write
	.type	zftl_sys_write, %function
zftl_sys_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	str	lr, [sp, #-4]!
	.save {lr}
	mov	ip, r1
	mov	lr, r0
	mov	r3, r2
	mov	r1, lr
	mov	r0, #2
	mov	r2, ip
	ldr	lr, [sp], #4
	b	zftl_write
	.fnend
	.size	zftl_sys_write, .-zftl_sys_write
	.align	2
	.global	zftl_discard
	.type	zftl_discard, %function
zftl_discard:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	add	r2, r0, r1
	ldr	r6, .L4458
	.pad #36
	sub	sp, sp, #36
	mov	r4, r1
	ldr	r3, [r6, #520]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L4409
	ldr	r3, .L4458+4
	add	r8, r0, #24576
	ldr	r10, .L4458+8
	ldr	r2, [r3]
	ldr	r5, [r10, #-2468]
	tst	r2, #4096
	str	r10, [sp, #12]
	add	r1, r1, r5
	str	r3, [sp, #16]
	str	r1, [r10, #-2468]
	beq	.L4410
	mov	r3, #0
	ldr	r0, .L4458+12
	str	r3, [sp]
	mov	r2, r8
	mov	r3, r4
	bl	printk
.L4410:
	ldr	r3, [r6, #3348]
	ldr	r9, [r3, #8]
	add	r2, r9, #1
	str	r2, [r3, #8]
	bl	ftl_write_commit
	bl	ftl_flush
	ldrb	r7, [r6, #3320]	@ zero_extendqisi2
	mov	r0, r8
	mov	r1, r7
	bl	__aeabi_uidiv
	mls	r10, r7, r0, r8
	mov	r5, r0
	cmp	r10, #0
	beq	.L4411
	rsb	r7, r10, r7
	cmp	r7, r4
	movcs	r7, r4
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #24]
	bne	.L4412
	mov	r0, r5
	add	r1, sp, #24
	mov	r2, #0
	bl	pm_log2phys
.L4412:
	ldr	r3, [sp, #24]
	uxth	fp, r7
	cmn	r3, #1
	beq	.L4414
	mov	r0, #0
	bl	buf_alloc
	subs	r3, r0, #0
	beq	.L4414
	ldr	r0, [r3, #4]
	mov	r2, fp, asl #9
	mov	r1, #0
	str	r5, [r3, #20]
	strb	r10, [r3, #41]
	add	r0, r0, r10, asl #9
	strb	r7, [r3, #40]
	str	r9, [r3, #16]
	str	r3, [sp, #20]
	bl	ftl_memset
	ldr	r3, [sp, #20]
	mov	r0, r3
	bl	ftl_write_buf
	bl	ftl_write_commit
	ldr	r2, [r6, #3348]
	ldr	r3, [r2, #76]
	add	r3, r3, #1
	str	r3, [r2, #76]
.L4414:
	add	r5, r5, #1
	rsb	r4, fp, r4
.L4411:
	cmp	r4, #0
	beq	.L4416
	bl	ftl_flush
.L4416:
	ldr	r7, .L4458
	mvn	r3, #0
	ldr	r10, .L4458+16
	str	r3, [sp, #28]
.L4417:
	ldrb	r3, [r6, #3320]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L4457
	mov	r0, r5
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #24]
	beq	.L4418
	mov	r0, #0
	bl	buf_alloc
	subs	fp, r0, #0
	beq	.L4420
	ldrb	r2, [r7, #3320]	@ zero_extendqisi2
	mov	r1, #0
	str	r5, [fp, #20]
	strb	r1, [fp, #41]
	strb	r2, [fp, #40]
	mov	r2, r2, asl #9
	str	r9, [fp, #16]
	ldr	r0, [fp, #4]
	bl	ftl_memset
	mov	r0, fp
	bl	ftl_write_buf
	bl	ftl_write_commit
	b	.L4456
.L4418:
	mov	r0, r5
	add	r1, sp, #24
	mov	r2, #0
	bl	pm_log2phys
	ldr	r3, [sp, #24]
	cmn	r3, #1
	beq	.L4420
	add	r1, sp, #28
	mov	r2, #1
	mov	r0, r5
	bl	pm_log2phys
	ldrb	r3, [r7, #1700]	@ zero_extendqisi2
	ldrh	r0, [r10]
	rsb	r3, r3, #24
	ldrb	r1, [r7, #4030]	@ zero_extendqisi2
	rsb	r3, r0, r3
	ldr	r2, [sp, #24]
	mov	r2, r2, lsr r0
	mvn	r0, #0
	bic	r0, r2, r0, asl r3
	bl	__aeabi_uidiv
	uxth	r0, r0
	bl	ftl_vpn_decrement
.L4456:
	ldr	r2, [r7, #3348]
	ldr	r3, [r2, #76]
	add	r3, r3, #1
	str	r3, [r2, #76]
.L4420:
	ldrb	r3, [r6, #3320]	@ zero_extendqisi2
	add	r5, r5, #1
	rsb	r4, r3, r4
	b	.L4417
.L4457:
	cmp	r4, #0
	beq	.L4424
	mov	r0, r5
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #24]
	bne	.L4425
	mov	r0, r5
	add	r1, sp, #24
	mov	r2, #0
	bl	pm_log2phys
.L4425:
	ldr	r3, [sp, #24]
	cmn	r3, #1
	beq	.L4424
	mov	r0, #0
	bl	buf_alloc
	subs	r7, r0, #0
	beq	.L4424
	mov	r3, #0
	strb	r3, [r7, #41]
	ldrb	r3, [r6, #3320]	@ zero_extendqisi2
	str	r5, [r7, #20]
	cmp	r4, r3
	strb	r4, [r7, #40]
	str	r9, [r7, #16]
	bcc	.L4427
	ldr	r1, .L4458+20
	movw	r2, #1259
	ldr	r0, .L4458+24
	bl	printk
	bl	dump_stack
.L4427:
	mov	r2, r4, asl #9
	mov	r1, #0
	ldr	r0, [r7, #4]
	bl	ftl_memset
	mov	r0, r7
	bl	ftl_write_buf
	bl	ftl_write_commit
	ldr	r2, [r6, #3348]
	ldr	r3, [r2, #76]
	add	r3, r3, #1
	str	r3, [r2, #76]
.L4424:
	ldr	r3, [sp, #12]
	ldr	r1, [r3, #-2468]
	cmp	r1, #8192
	bls	.L4430
	ldr	r3, [sp, #16]
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L4428
	mov	r3, #0
	ldr	r0, .L4458+12
	str	r3, [sp]
	mov	r2, r8
	mov	r3, r4
	bl	printk
.L4428:
	ldr	r3, [sp, #12]
	mov	r4, #0
	str	r4, [r3, #-2468]
	bl	flt_sys_flush
	mov	r3, #1
	str	r3, [r6, #3364]
.L4430:
	mov	r0, #0
.L4409:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4459:
	.align	2
.L4458:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	.LC309
	.word	.LANCHOR0+4028
	.word	.LANCHOR1+2172
	.word	.LC0
	.fnend
	.size	zftl_discard, .-zftl_discard
	.align	2
	.global	dump_pm_blk
	.type	dump_pm_blk, %function
dump_pm_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
	movw	ip, #698
	ldr	r4, .L4466
	mov	r2, #4
	ldr	r0, .L4466+4
	mov	r5, #0
	ldr	r6, .L4466+8
	movw	r7, #65535
	ldr	r3, [r4, #3348]
	add	r1, r3, #704
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldr	r1, [r4, #3348]
	ldr	r0, .L4466+12
	mov	r2, #2
	add	r1, r1, #416
	ldrh	r3, [r6, #-2]!
	bl	rknand_print_hex
.L4461:
	ldrh	r2, [r6]
	uxth	r3, r5
	cmp	r2, r3
	bls	.L4465
	add	r3, r3, #208
	ldr	r2, [r4, #3348]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	cmp	r0, r7
	beq	.L4462
	mov	r1, #0
	bl	ftl_sblk_dump
.L4462:
	add	r5, r5, #1
	b	.L4461
.L4465:
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L4467:
	.align	2
.L4466:
	.word	.LANCHOR0
	.word	.LC267
	.word	.LANCHOR3-3568
	.word	.LC268
	.fnend
	.size	dump_pm_blk, .-dump_pm_blk
	.align	2
	.global	id_block_prog_msb_ff_data
	.type	id_block_prog_msb_ff_data, %function
id_block_prog_msb_ff_data:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r4, r2
	ldr	r5, .L4475
	ldrb	r2, [r5, #1625]	@ zero_extendqisi2
	ldr	r3, [r5, #1620]
	cmp	r2, #0
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	bne	.L4468
	sub	r2, r3, #5
	mov	r8, r0
	cmp	r3, #68
	cmpne	r2, #2
	mov	r9, r1
	bls	.L4470
	sub	r3, r3, #19
	tst	r3, #239
	bne	.L4468
.L4470:
	ldr	r7, .L4475
	add	r10, r7, #596
.L4472:
	ldr	r3, [r5, #1620]
	ldrh	r3, [r3, #10]
	cmp	r3, r4
	bls	.L4468
	mov	r6, r4, asl #1
	add	fp, r4, r9
	ldr	r0, .L4475+4
	mov	r1, r4
	mov	r2, fp
	ldrh	r3, [r10, r6]
	bl	printk
	ldrh	r2, [r10, r6]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L4468
	ldr	r6, .L4475+8
	mov	r1, #255
	mov	r2, #16384
	add	r4, r4, #1
	ldr	r0, [r6, #-3288]
	uxth	r4, r4
	bl	memset
	ldr	r3, [r7, #1620]
	ldr	r2, [r6, #-3288]
	mov	r0, r8
	mov	r1, fp
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	str	r3, [sp]
	mov	r3, r2
	bl	flash_prog_page
	b	.L4472
.L4468:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4476:
	.align	2
.L4475:
	.word	.LANCHOR0
	.word	.LC310
	.word	.LANCHOR3
	.fnend
	.size	id_block_prog_msb_ff_data, .-id_block_prog_msb_ff_data
	.align	2
	.global	write_idblock
	.type	write_idblock, %function
write_idblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 136
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #156
	sub	sp, sp, #156
	ldr	r9, .L4549
	mov	r6, r0
	mov	r0, #256000
	mov	r10, r1
	mov	r8, r2
	ldr	r3, [r9, #1620]
	ldrb	r4, [r3, #9]	@ zero_extendqisi2
	ldrh	r5, [r3, #10]
	ldr	r3, .L4549+4
	ldrh	r7, [r3, #30]
	ldrb	r3, [r9]	@ zero_extendqisi2
	str	r3, [sp, #60]
	ldrh	r3, [sp, #60]
	str	r3, [sp, #52]
	ldrb	r3, [r9, #1625]	@ zero_extendqisi2
	str	r3, [sp, #68]
	bl	ftl_malloc
	subs	r3, r0, #0
	str	r3, [sp, #24]
	beq	.L4514
	ldr	r3, .L4549+8
	ldrb	r3, [r3, #-2486]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L4515
	ldrb	r3, [r9]	@ zero_extendqisi2
	cmp	r3, #0
	ldrb	r3, [r9, #1625]	@ zero_extendqisi2
	clzne	r3, r3
	movne	r3, r3, lsr #5
	bne	.L4515
	cmp	r3, #3
	ldr	r3, [sp, #52]
	moveq	r3, #3
	str	r3, [sp, #52]
	movne	r3, #0
	moveq	r3, #2
.L4515:
	add	r6, r6, #508
	mov	r0, #0
	add	r6, r6, #3
	str	r3, [sp, #20]
	bl	zftl_flash_exit_slc_mode
	ldrb	r3, [r9, #1625]	@ zero_extendqisi2
	mov	r6, r6, lsr #9
	cmp	r6, #8
	strb	r3, [r9]
	bls	.L4517
	cmp	r6, #500
	bhi	.L4482
	b	.L4481
.L4517:
	mov	r6, #8
.L4481:
	ldr	r2, [r10]
	ldr	r3, .L4549+12
	cmp	r2, r3
	beq	.L4483
.L4482:
	ldr	r0, [sp, #24]
	bl	ftl_free
	b	.L4514
.L4483:
	uxth	r4, r4
	smulbb	r1, r4, r5
	uxth	r1, r1
	sub	r0, r1, #1
	add	r0, r0, r6
	bl	__aeabi_uidiv
	mov	r3, #0
	movw	r2, #63871
	str	r0, [sp, #72]
	add	r0, r10, #254976
	add	r0, r0, #512
.L4487:
	ldr	r1, [r0, #-4]!
	cmp	r1, #0
	bne	.L4484
	ldr	r1, [r10, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, #4096
	sub	r2, r2, #1
	movhi	r3, #0
	cmp	r2, #4096
	str	r1, [r0, #512]
	bne	.L4487
	b	.L4486
.L4484:
	ldr	r0, .L4549+16
	bl	printk
.L4486:
	smulbb	r3, r4, r7
	ldr	r0, .L4549+20
	mov	r1, r8
	mov	r2, #4
	ldr	r4, .L4549
	uxth	r3, r3
	str	r3, [sp, #56]
	mov	r3, #5
	bl	rknand_print_hex
	ldr	r0, .L4549+24
	mov	r1, r6
	mov	r2, r6
	bl	printk
	sub	r3, r8, #4
	str	r3, [sp, #44]
	mov	r3, #0
	str	r3, [sp, #36]
	str	r3, [sp, #28]
.L4512:
	ldr	r3, .L4549+8
	ldrb	r3, [r3, #-2486]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L4488
	ldr	r3, [sp, #44]
	ldr	r1, [sp, #72]
	ldr	r2, [r3, #4]
	ldr	r3, [sp, #56]
	mul	fp, r3, r2
	ldr	r3, [sp, #28]
	cmp	r3, #0
	cmpne	r1, #1
	bls	.L4489
	ldr	r3, [sp, #44]
	ldr	r3, [r3]
	add	r3, r3, #1
	cmp	r2, r3
	bne	.L4489
	b	.L4490
.L4488:
	ldr	r3, [r4, #1696]
	ldr	r2, [sp, #28]
	add	r3, r3, r2
	ldrb	r3, [r3, #32]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L4490
	ldr	r2, [sp, #56]
	mul	fp, r2, r3
	ldr	r3, [sp, #20]
	cmp	r3, #0
	beq	.L4489
	ldr	r3, [sp, #28]
	bic	r3, r3, #2
	cmp	r3, #1
	ldreqb	r3, [sp, #52]	@ zero_extendqisi2
	beq	.L4547
	mov	r0, #0
	bl	zftl_flash_exit_slc_mode
	mov	r3, #0
.L4547:
	strb	r3, [r4]
	strb	r3, [r4, #1625]
.L4489:
	ldr	r0, [sp, #24]
	mov	r1, #512
	bl	__memzero
	ldr	r3, [r4, #1620]
	mov	r8, #0
	mov	r0, fp
	ldrb	r2, [r3, #9]	@ zero_extendqisi2
	ldrh	r7, [r3, #10]
	ldr	r3, .L4549+4
	mov	r1, r2
	ldrh	r3, [r3, #30]
	str	r2, [sp, #40]
	ldrh	r5, [sp, #40]
	smulbb	r7, r7, r5
	smulbb	r5, r3, r5
	ldrb	r3, [r4, #1688]	@ zero_extendqisi2
	strb	r8, [r4, #1688]
	str	r3, [sp, #76]
	bl	__aeabi_uidiv
	uxth	r7, r7
	uxth	r5, r5
	mov	r1, r0
	mov	r0, r8
	bl	flash_erase_block
	cmp	r6, r7
	movls	r0, #1
	bls	.L4492
	mov	r0, r8
	add	r1, fp, r5
	bl	flash_erase_block
	mov	r0, #2
.L4492:
	ldr	r3, .L4549
	ldr	r2, [r3, #1620]
	ldrh	r3, [r2, #10]
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	mov	r3, r3, asl #2
	mul	r0, r0, r3
	bl	__aeabi_idiv
	mov	r1, r5
	str	r0, [sp, #80]
	mov	r0, fp
	bl	__aeabi_uidivmod
	rsb	r3, r1, fp
	str	r1, [sp, #64]
	cmp	fp, r3
	str	r3, [sp, #48]
	bne	.L4520
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L4520
	ldr	r3, .L4549+28
	mov	r1, #1024
	ldr	r5, [r3, #-3264]
	mov	r0, r5
	bl	__memzero
	ldr	r2, .L4549+32
	mov	r3, #12
	stmia	r5, {r2, r3}
	mov	r3, #0
	strb	r3, [r5, #16]
	str	r3, [r5, #12]
	ldrb	r3, [r4, #1625]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, [r4, #1620]
	ldrneb	r3, [r3, #29]	@ zero_extendqisi2
	strneb	r3, [r5, #16]
	mov	r3, #4
	strb	r3, [r5, #17]
	ldr	r3, [r4, #1620]
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	ldrh	r0, [r3, #10]
	bl	__aeabi_idiv
	mov	r3, #0
	mov	r1, #12
	strb	r3, [r5, #20]
	strh	r3, [r5, #22]	@ movhi
	ldr	r2, [sp, #40]
	strh	r0, [r5, #18]	@ movhi
	add	r0, r5, #12
	cmp	r2, #8
	movhi	r2, #70
	movls	r2, #16
	strb	r2, [r5, #21]
	bl	js_hash
	sub	r3, r6, #4
	str	r0, [r5, #8]
	str	r3, [sp, #32]
	b	.L4493
.L4520:
	mov	r5, #0
	str	r6, [sp, #32]
.L4493:
	mov	r8, r10
	mov	r9, #0
.L4496:
	ldr	r3, [sp, #80]
	cmp	r9, r3
	bcs	.L4548
	ldr	r3, [sp, #64]
	ldrb	r2, [r4, #1625]	@ zero_extendqisi2
	add	r7, r9, r3
	ubfx	r7, r7, #2, #16
	cmp	r2, #0
	add	r0, r7, #1
	mov	r1, r0, asl #1
	add	r3, r4, r1
	ldrh	r3, [r3, #4]
	beq	.L4498
	ldrb	ip, [r4, #1]	@ zero_extendqisi2
	cmp	ip, #0
	moveq	r3, r0
	movne	r3, r1
.L4498:
	ldrb	r1, [r4, #516]	@ zero_extendqisi2
	ldr	r0, .L4549
	cmp	r1, #9
	movw	r1, #61424
	str	r1, [sp, #92]
	mov	r1, r7, asl #1
	add	r0, r0, r1
	subne	r3, r3, #1
	ldrh	r0, [r0, #4]
	movne	r3, r3, asl #2
	cmp	r2, #0
	str	r3, [sp, #88]
	moveq	r7, r0
	beq	.L4502
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	cmp	r2, #0
	movne	r7, r1
.L4502:
	ldr	r2, [sp, #40]
	cmp	r5, #0
	mul	r0, r2, r7
	bne	.L4503
	ldr	r2, [sp, #48]
	str	r8, [sp]
	add	ip, r0, r2
	ldr	r2, [r8]
	ldr	r0, .L4549+36
	mov	r1, ip
	str	ip, [sp, #84]
	stmib	sp, {r2, r3}
	movw	r3, #61424
	mov	r2, r9
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
	mov	r1, r8
	ldr	ip, [sp, #84]
	add	r2, sp, #88
	mov	r0, ip
	bl	fw_flash_page_prog.constprop.30
	ldrb	r3, [r4, #1625]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L4504
	ldr	r1, [sp, #40]
	ldr	r0, [sp, #48]
	bl	__aeabi_uidiv
	add	r2, r7, #1
	uxth	r2, r2
	mov	r1, r0
	mov	r0, r5
	bl	id_block_prog_msb_ff_data
.L4504:
	ldr	r3, [sp, #32]
	add	r8, r8, #2048
	add	r3, r3, #16
	cmp	r9, r3
	bcc	.L4505
	ldr	r3, [sp, #32]
	add	r3, r3, #20
	cmp	r9, r3
	addcc	r8, r10, #2048
	b	.L4505
.L4503:
	ldr	r3, [sp, #48]
	mov	r1, r5
	add	r2, sp, #88
	add	r0, r0, r3
	bl	fw_flash_page_prog.constprop.30
.L4505:
	add	r9, r9, #4
	mov	r5, #0
	uxth	r9, r9
	b	.L4496
.L4548:
	ldr	r3, .L4549
	mov	r0, fp
	ldrb	r2, [sp, #76]	@ zero_extendqisi2
	ldr	r1, [sp, #32]
	strb	r2, [r3, #1688]
	ldr	r2, [sp, #24]
	bl	id_block_read_data.constprop.31
	ldr	r3, [sp, #20]
	cmp	r3, #0
	beq	.L4507
	mov	r0, #0
	bl	zftl_flash_exit_slc_mode
	mov	r3, #0
	strb	r3, [r4]
	strb	r3, [r4, #1625]
.L4507:
	ldr	r3, [sp, #32]
	mov	r5, #0
	mov	r8, r3, asl #7
.L4510:
	ldr	r3, [sp, #24]
	ldr	r2, [r3, r5, asl #2]
	ldr	r3, [r10, r5, asl #2]
	cmp	r2, r3
	beq	.L4508
	ldr	r0, [sp, #24]
	mov	r1, #512
	bl	__memzero
	mov	r0, #0
	mov	r1, fp
	bl	flash_erase_block
	cmp	r5, r8
	bcc	.L4490
	b	.L4509
.L4508:
	add	r5, r5, #1
	cmp	r5, r8
	bne	.L4510
.L4509:
	ldr	r3, [sp, #36]
	add	r3, r3, #1
	str	r3, [sp, #36]
	cmp	r3, #5
	bhi	.L4511
.L4490:
	ldr	r3, [sp, #28]
	add	r3, r3, #1
	str	r3, [sp, #28]
	ldr	r3, [sp, #44]
	add	r3, r3, #4
	str	r3, [sp, #44]
	ldr	r3, [sp, #28]
	cmp	r3, #4
	bne	.L4512
.L4511:
	mov	r0, #0
	bl	zftl_flash_exit_slc_mode
	ldr	r3, .L4549
	mov	r0, #0
	ldrb	r2, [sp, #60]	@ zero_extendqisi2
	strb	r2, [r3]
	ldr	r2, [sp, #20]
	cmp	r2, #0
	ldrneb	r2, [sp, #68]	@ zero_extendqisi2
	strneb	r2, [r3, #1625]
	mov	r2, #2
	strb	r2, [r3, #1704]
	bl	zftl_flash_enter_slc_mode
	ldr	r0, [sp, #24]
	bl	ftl_free
	ldr	r3, [sp, #36]
	clz	r0, r3
	mov	r0, r0, lsr #5
	rsb	r0, r0, #0
	b	.L4478
.L4514:
	mvn	r0, #0
.L4478:
	add	sp, sp, #156
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4550:
	.align	2
.L4549:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	-52655045
	.word	.LC311
	.word	.LC312
	.word	.LC313
	.word	.LANCHOR3
	.word	1179535694
	.word	.LC314
	.fnend
	.size	write_idblock, .-write_idblock
	.align	2
	.global	CRC_32
	.type	CRC_32, %function
CRC_32:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	ip, .L4555
	add	r1, r0, r1
	mov	r2, #0
.L4552:
	cmp	r0, r1
	beq	.L4554
	ldrb	r3, [r0], #1	@ zero_extendqisi2
	eor	r3, r3, r2, lsr #24
	add	r3, ip, r3, asl #2
	ldr	r3, [r3, #-3920]
	eor	r2, r3, r2, asl #8
	b	.L4552
.L4554:
	mov	r0, r2
	bx	lr
.L4556:
	.align	2
.L4555:
	.word	.LANCHOR6
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	mov	r4, r0
	.pad #52
	sub	sp, sp, #52
	mov	r6, r1
	mov	r8, r2
	ldr	r5, .L4580
	bne	.L4558
	ldr	r2, [r2]
	ldr	r3, .L4580+4
	cmp	r2, r3
	bne	.L4558
	mov	r0, #256000
	mov	r3, #1
	strb	r3, [r5, #-2464]
	bl	ftl_malloc
	mov	r1, #256000
	str	r0, [r5, #-2460]
	bl	__memzero
	str	r4, [r5, #-2456]
.L4558:
	str	r6, [sp]
	mov	r3, r4
	ldr	r0, .L4580+8
	ldr	r1, [r5, #-2460]
	ldr	r2, [r8]
	bl	printk
	ldrb	r3, [r5, #-2464]	@ zero_extendqisi2
	ldr	r9, .L4580
	cmp	r3, #0
	beq	.L4557
	ldr	r7, [r9, #-2460]
	cmp	r7, #0
	beq	.L4557
	sub	r0, r4, #64
	cmp	r0, #500
	bcs	.L4560
	rsb	r2, r4, #564
	add	r0, r7, r0, asl #9
	cmp	r2, r6
	mov	r1, r8
	movcs	r2, r6
	mov	r2, r2, asl #9
	bl	ftl_memcpy
	b	.L4561
.L4560:
	cmp	r4, #564
	bcs	.L4569
.L4561:
	ldr	r3, [r5, #-2456]
	ldr	r8, .L4580
	cmp	r3, r4
	beq	.L4567
	mov	r0, r7
	mov	r9, #0
	strb	r9, [r8, #-2464]
	bl	ftl_free
	str	r9, [r8, #-2460]
.L4567:
	add	r4, r4, r6
	str	r4, [r5, #-2456]
	b	.L4557
.L4569:
	ldr	r3, .L4580+12
	ldr	r0, [r9, #-2456]
	ldr	r3, [r3, #1620]
	sub	r0, r0, #64
	cmp	r0, #500
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	movcs	r0, #500
	cmp	r3, #4
	beq	.L4570
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
	b	.L4563
.L4570:
	mov	r3, #0
.L4562:
	cmp	r0, #256
	add	r1, sp, #8
	mov	r2, r3, asl #1
	movls	r2, r3
	str	r2, [r1, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, #5
	bne	.L4562
.L4563:
	mov	r3, #61952
.L4566:
	ldr	r2, [r7, r3, asl #2]
	cmp	r2, #0
	addne	r3, r3, #2048
	movne	r0, r3, asl #2
	bne	.L4565
.L4564:
	sub	r3, r3, #1
	cmp	r3, #4096
	bne	.L4566
	mov	r0, r0, asl #9
.L4565:
	mov	r1, r7
	add	r2, sp, #8
	bl	write_idblock
	ldr	r0, [r5, #-2460]
	mov	r7, #0
	strb	r7, [r5, #-2464]
	bl	ftl_free
	str	r7, [r5, #-2460]
	b	.L4567
.L4557:
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L4581:
	.align	2
.L4580:
	.word	.LANCHOR5
	.word	-52655045
	.word	.LC315
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
	mov	r6, r2
	sub	r2, r1, #64
	mov	r4, r1
	cmp	r2, #1984
	mov	r5, r3
	mov	r7, r0
	movcs	r2, #0
	movcc	r2, #1
	cmp	r0, #0
	movne	r2, #0
	cmp	r2, #0
	beq	.L4583
	mov	r0, r1
	mov	r2, r3
	mov	r1, r6
	bl	write_loader_lba
.L4583:
	ldr	r3, .L4588
	mov	r0, r7
	mov	r1, r4
	mov	r2, r6
	ldr	r3, [r3, #-3296]
	ldr	ip, [r3, #24]
	mov	r3, r5
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	bx	ip
.L4589:
	.align	2
.L4588:
	.word	.LANCHOR3
	.fnend
	.size	FtlWrite, .-FtlWrite
	.align	2
	.global	rknand_sys_storage_ioctl
	.type	rknand_sys_storage_ioctl, %function
rknand_sys_storage_ioctl:
	.fnstart
	@ args = 0, pretend = 0, frame = 528
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L4696
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	cmp	r1, r3
	.pad #532
	sub	sp, sp, #532
	mov	r4, r1
	mov	r5, r2
	beq	.L4592
	bhi	.L4593
	sub	r3, r3, #3456
	sub	r3, r3, #5
	cmp	r1, r3
	beq	.L4594
	bhi	.L4595
	sub	r3, r3, #125
	cmp	r1, r3
	beq	.L4596
	bhi	.L4597
	sub	r3, r3, #237
	cmp	r1, r3
	bne	.L4653
	bl	rknand_dev_flush
	b	.L4684
.L4597:
	ldr	r3, .L4696+4
	cmp	r1, r3
	beq	.L4599
	add	r3, r3, #1
	cmp	r1, r3
	beq	.L4600
	b	.L4653
.L4595:
	ldr	r3, .L4696+8
	cmp	r1, r3
	beq	.L4601
	bhi	.L4602
	sub	r3, r3, #1952
	sub	r3, r3, #9
	cmp	r1, r3
	beq	.L4603
	b	.L4653
.L4602:
	ldr	r3, .L4696+12
	cmp	r1, r3
	beq	.L4601
	add	r3, r3, #10
	cmp	r1, r3
	beq	.L4601
	b	.L4653
.L4593:
	ldr	r3, .L4696+16
	cmp	r1, r3
	beq	.L4604
	bhi	.L4605
	sub	r3, r3, #78
	cmp	r1, r3
	beq	.L4606
	bcc	.L4607
	add	r3, r3, #21
	cmp	r1, r3
	beq	.L4608
	add	r3, r3, #56
	cmp	r1, r3
	beq	.L4609
	b	.L4653
.L4605:
	ldr	r3, .L4696+20
	cmp	r1, r3
	mov	r6, r3
	beq	.L4610
	bhi	.L4611
	sub	r3, r3, #956
	sub	r3, r3, #1
	cmp	r1, r3
	beq	.L4612
	add	r3, r3, #956
	cmp	r1, r3
	beq	.L4613
	b	.L4653
.L4611:
	ldr	r3, .L4696+24
	cmp	r1, r3
	beq	.L4613
	add	r3, r3, #1
	cmp	r1, r3
	beq	.L4610
	b	.L4653
.L4607:
	ldr	r0, .L4696+28
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r4, r0, #0
	bne	.L4614
.L4619:
	mvn	r0, #11
	b	.L4591
.L4614:
	mov	r1, r5
	mov	r2, #512
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4690
	ldr	r0, .L4696+32
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r4, #4]
	cmp	r3, #8
	str	r3, [sp, #4]
	bhi	.L4688
	bl	rknand_device_lock
	mov	r2, r4
	ldr	r0, [r4]
	ldr	r1, [sp, #4]
	bl	id_block_read_data.constprop.31
	bl	rknand_device_unlock
	mov	r0, r5
	mov	r1, r4
	ldr	r2, [sp, #4]
	mov	r2, r2, asl #9
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L4683
	ldr	r0, .L4696+36
.L4686:
	bl	printk
.L4688:
	mov	r0, r4
.L4689:
	bl	ftl_free
.L4687:
	mvn	r0, #13
	b	.L4591
.L4606:
	ldr	r0, .L4696+40
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r4, r0, #0
	beq	.L4619
	mov	r1, r5
	mov	r2, #4096
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L4620
.L4690:
	ldr	r0, .L4696+44
	b	.L4686
.L4620:
	ldr	r5, .L4696+48
	ldr	r0, .L4696+52
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r5, #-2452]
	mov	r6, r5
	cmp	r3, #0
	bne	.L4621
	mov	r0, #260096
	bl	ftl_malloc
	cmp	r0, #0
	str	r0, [r5, #-2452]
	beq	.L4688
	mov	r1, #260096
	bl	__memzero
.L4621:
	ldr	r2, [r4, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L4688
	ldr	r3, [r4]
	cmp	r3, #251904
	bhi	.L4688
	ldr	r0, [r6, #-2452]
	add	r1, r4, #8
	add	r0, r0, r3
	bl	memcpy
.L4683:
	mov	r0, r4
	bl	ftl_free
.L4684:
	mov	r4, #0
	b	.L4618
.L4609:
	ldr	r0, .L4696+56
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4619
	mov	r1, r5
	mov	r2, #28
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L4624
	ldr	r0, .L4696+44
	bl	printk
	b	.L4625
.L4624:
	ldmia	r6, {r1, r2}
	ldr	r0, .L4696+60
	bl	printk
	ldr	r1, [r6]
	cmp	r1, #256000
	bhi	.L4625
	ldr	r5, .L4696+48
	ldr	r0, [r5, #-2452]
	cmp	r0, #0
	beq	.L4625
	bl	CRC_32
	ldr	r3, [r6, #4]
	cmp	r3, r0
	beq	.L4627
	mov	r0, r6
	bl	ftl_free
	b	.L4654
.L4627:
	bl	rknand_device_lock
	ldr	r1, [r5, #-2452]
	add	r2, r6, #8
	ldr	r0, [r6]
	bl	write_idblock
	mov	r4, #0
	bl	rknand_device_unlock
	ldr	r0, [r5, #-2452]
	bl	ftl_free
	str	r4, [r5, #-2452]
	mov	r0, r6
	b	.L4631
.L4625:
	mov	r0, r6
	b	.L4689
.L4608:
	ldr	r0, .L4696+64
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4619
	bl	ReadFlashInfo
	mov	r0, r5
	mov	r1, r6
	mov	r2, #11
	b	.L4694
.L4592:
	ldr	r0, .L4696+68
	bl	printk
	bl	nand_blk_add_whole_disk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4619
	mov	r1, #64
	bl	__memzero
	mov	r0, r5
	mov	r1, r6
	mov	r2, #64
	b	.L4694
.L4604:
	ldr	r0, .L4696+72
	bl	printk
	ldr	r3, .L4696+48
	mov	r0, r5
	mov	r2, #4
	ldr	r3, [r3, #-2448]
	ldr	r3, [r3, #20]
	add	r1, sp, #528
	str	r3, [r1, #-524]!
.L4678:
	bl	rk_copy_to_user
	cmp	r0, #0
	bne	.L4687
	b	.L4684
.L4612:
	ldr	r0, .L4696+76
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4619
	bl	rknand_device_lock
	mov	r1, #264
	mov	r2, #2
	mov	r3, r6
	mov	r0, #16
	bl	FtlRead
	bl	rknand_device_unlock
	mov	r0, r5
	mov	r1, r6
	mov	r2, #1024
.L4694:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	mov	r0, r6
	bne	.L4689
.L4631:
	bl	ftl_free
	b	.L4618
.L4596:
	ldr	r0, .L4696+80
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	subs	r4, r0, #0
	beq	.L4632
.L4637:
	ldr	r0, .L4696+44
	bl	printk
	b	.L4687
.L4632:
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+84
	cmp	r2, r3
	beq	.L4633
.L4634:
	mvn	r4, #0
	b	.L4618
.L4633:
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4634
	ldr	r6, .L4696+48
	mov	r2, #512
	add	r0, sp, #8
	ldr	r1, [r6, #-2448]
	bl	memcpy
	ldr	r2, [r6, #-2444]
	ldr	r3, .L4696+88
	cmp	r2, r3
	beq	.L4635
	add	r0, sp, #72
	mov	r1, #128
	str	r4, [sp, #16]
	str	r4, [sp, #20]
	bl	__memzero
.L4635:
	add	r0, sp, #264
	mov	r1, #256
	mov	r3, #0
	str	r3, [sp, #24]
	bl	__memzero
	b	.L4692
.L4599:
	ldr	r0, .L4696+92
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4637
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+84
	cmp	r2, r3
	bne	.L4634
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4634
	ldr	r2, .L4696+48
	ldr	r3, .L4696+88
	ldr	r1, [r2, #-2444]
	cmp	r1, r3
	bne	.L4654
	ldr	r3, [sp, #20]
	sub	r1, r3, #1
	cmp	r1, #127
	mvnhi	r0, #2
	bhi	.L4591
	ldr	r4, [r2, #-2448]
	add	r1, sp, #72
	add	r0, r4, #64
	str	r3, [r4, #12]
	ldr	r2, [sp, #20]
	bl	memcpy
	mov	r0, #1
	mov	r1, r4
	b	.L4685
.L4603:
	ldr	r0, .L4696+96
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4637
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+100
	cmp	r2, r3
	bne	.L4634
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4634
	ldr	r5, .L4696+48
	ldr	r3, [r5, #-2440]
	cmp	r3, #0
	bne	.L4638
.L4641:
	mov	r0, #0
	b	.L4591
.L4638:
	ldr	r3, [r5, #-2436]
	ldr	r2, .L4696+104
	ldr	r1, [r3]
	cmp	r1, r2
	beq	.L4639
	str	r2, [r3]
	mov	r2, #504
	ldr	r3, [r5, #-2436]
	str	r2, [r3, #4]
	mov	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L4639:
	ldr	r1, [r5, #-2436]
	mov	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #-2448]
	ldr	r2, .L4696+84
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r0, [r5, #-2448]
	mov	r1, #128
	ldrne	r3, .L4696+48
	movne	r2, #504
	add	r0, r0, #64
	ldrne	r3, [r3, #-2448]
	stmneib	r3, {r2, r4}
	mov	r4, #0
	str	r4, [r0, #-52]
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #-2448]
	bl	StorageSysDataStore
	str	r4, [r5, #-2440]
	str	r4, [r5, #-2444]
	b	.L4618
.L4594:
	ldr	r0, .L4696+108
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4637
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+112
	cmp	r2, r3
	bne	.L4634
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4634
	ldr	r5, .L4696+48
	ldr	r3, [r5, #-2440]
	cmp	r3, #1
	beq	.L4641
	ldr	r3, [r5, #-2436]
	mov	r4, #0
	ldr	r2, .L4696+104
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r1, [r5, #-2436]
	ldrne	r3, [r5, #-2436]
	movne	r2, #504
	strne	r0, [r3, #8]
	strne	r2, [r3, #4]
	strne	r0, [r3, #12]
	mov	r3, #1
	mov	r0, #0
	str	r3, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #-2448]
	ldr	r2, .L4696+84
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r0, [r5, #-2448]
	ldrne	r3, .L4696+48
	movne	r1, #504
	movne	r2, #0
	add	r0, r0, #64
	ldrne	r3, [r3, #-2448]
	stmneib	r3, {r1, r2}
	mov	r1, #128
	str	r4, [r0, #-52]
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #-2448]
	bl	StorageSysDataStore
	mov	r3, #1
	str	r3, [r5, #-2440]
	b	.L4618
.L4600:
	ldr	r0, .L4696+116
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4637
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+120
	cmp	r2, r3
	bne	.L4634
	ldr	r2, [sp, #12]
	cmp	r2, #512
	addls	r0, sp, #16
	ldrls	r1, .L4696+124
	bls	.L4691
	b	.L4634
.L4601:
	ldr	r0, .L4696+12
	cmp	r4, r0
	mov	r7, r0
	ldreq	r0, .L4696+128
	beq	.L4681
	ldr	r3, .L4696+132
	cmp	r4, r3
	ldreq	r0, .L4696+136
	ldrne	r0, .L4696+140
.L4681:
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4637
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+144
	cmp	r2, r3
	bne	.L4687
	ldr	r3, .L4696+132
	ldr	r6, .L4696+48
	cmp	r4, r3
	bne	.L4647
	ldr	r3, [r6, #-2448]
	mov	r0, r5
	add	r1, sp, #8
	mov	r2, #16
	ldr	r3, [r3, #20]
	str	r3, [sp, #12]
	strb	r3, [sp, #16]
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L4591
	b	.L4687
.L4647:
	ldr	r3, [r6, #-1920]
	cmp	r3, #10
	bhi	.L4687
	ldr	r1, [r6, #-2448]
	ldr	r2, [sp, #12]
	ldr	r3, [r1, #24]
	cmp	r3, r2
	cmpne	r3, #0
	movne	r3, #1
	moveq	r3, #0
	beq	.L4648
	ldr	r0, .L4696+148
	mov	r1, r2
	bl	printk
	ldr	r3, [r6, #-1920]
	add	r3, r3, #1
	str	r3, [r6, #-1920]
	b	.L4687
.L4648:
	cmp	r4, r7
	str	r3, [r6, #-1920]
	mov	r0, #1
	moveq	r2, r3
	movne	r3, #1
	moveq	r3, r2
	str	r2, [r1, #24]
	str	r3, [r1, #20]
	bl	StorageSysDataStore
	cmn	r0, #1
	bne	.L4684
	mvn	r4, #1
	b	.L4618
.L4613:
	ldr	r0, .L4696+152
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4637
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+156
	cmp	r2, r3
	bne	.L4634
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L4634
	ldr	r3, .L4696+160
	add	r0, sp, #16
	cmp	r4, r3
	ldr	r3, .L4696+48
	ldreq	r1, [r3, #-1916]
	ldrne	r1, [r3, #-1912]
	add	r1, r1, #8
.L4691:
	bl	memcpy
.L4692:
	mov	r0, r5
	add	r1, sp, #8
	mov	r2, #520
	b	.L4678
.L4610:
	ldr	r0, .L4696+164
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4637
	ldr	r2, [sp, #8]
	ldr	r3, .L4696+156
	cmp	r2, r3
	bne	.L4634
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L4634
	cmp	r4, r6
	add	r2, r2, #8
	ldr	r4, .L4696+48
	bne	.L4652
	add	r1, sp, #8
	ldr	r0, [r4, #-1916]
	bl	memcpy
	mov	r0, #2
	ldr	r1, [r4, #-1916]
	b	.L4685
.L4652:
	add	r1, sp, #8
	ldr	r0, [r4, #-1912]
	bl	memcpy
	ldr	r1, [r4, #-1912]
	mov	r0, #3
.L4685:
	bl	StorageSysDataStore
	mov	r4, r0
.L4618:
	ldr	r0, .L4696+168
	mov	r1, r4
	bl	printk
	mov	r0, r4
	b	.L4591
.L4653:
	mvn	r0, #21
	b	.L4591
.L4654:
	mvn	r0, #1
.L4591:
	add	sp, sp, #532
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L4697:
	.align	2
.L4696:
	.word	1074033155
	.word	1074029570
	.word	1074031656
	.word	1074031666
	.word	1074033235
	.word	1074034193
	.word	1074034194
	.word	.LC316
	.word	.LC318
	.word	.LC319
	.word	.LC320
	.word	.LC317
	.word	.LANCHOR5
	.word	.LC321
	.word	.LC322
	.word	.LC323
	.word	.LC324
	.word	.LC325
	.word	.LC326
	.word	.LC327
	.word	.LC328
	.word	1263358532
	.word	-1067903959
	.word	.LC329
	.word	.LC330
	.word	1112753220
	.word	1146313043
	.word	.LC331
	.word	1112755781
	.word	.LC332
	.word	1094995539
	.word	.LANCHOR5-2432
	.word	.LC333
	.word	1074031676
	.word	.LC334
	.word	.LC335
	.word	1280262987
	.word	.LC336
	.word	.LC337
	.word	1145980246
	.word	1074034192
	.word	.LC338
	.word	.LC339
	.fnend
	.size	rknand_sys_storage_ioctl, .-rknand_sys_storage_ioctl
	.align	2
	.global	rk_ftl_storage_sys_init
	.type	rk_ftl_storage_sys_init, %function
rk_ftl_storage_sys_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mvn	r3, #0
	ldr	r4, .L4707
	mov	r2, #512
	mov	r5, #0
	sub	r0, r4, #2432
	ldr	r1, [r4, #-2484]
	str	r3, [r4, #-2456]
	add	r3, r1, #512
	str	r3, [r4, #-2448]
	str	r1, [r4, #-2436]
	add	r3, r1, #1024
	add	r1, r1, #1536
	str	r3, [r4, #-1916]
	strb	r5, [r4, #-2464]
	str	r5, [r4, #-2460]
	str	r5, [r4, #-2452]
	str	r1, [r4, #-1912]
	bl	ftl_memcpy
	ldr	r0, [r4, #-2436]
	str	r5, [r4, #-2444]
	str	r5, [r4, #-1920]
	ldr	r6, [r0, #508]
	ldr	r3, [r0, #16]
	cmp	r6, r5
	str	r3, [r4, #-2440]
	beq	.L4700
	mov	r1, #508
	bl	js_hash
	cmp	r6, r0
	beq	.L4700
	ldr	r3, [r4, #-2436]
	ldr	r0, .L4707+4
	str	r5, [r4, #-2440]
	str	r5, [r3, #16]
	bl	printk
.L4700:
	ldr	r3, [r4, #-2440]
	mov	r0, #2
	ldr	r1, [r4, #-1916]
	cmp	r3, #0
	ldrne	r2, .L4707+8
	ldrne	r3, .L4707
	strne	r2, [r3, #-2444]
	bl	StorageSysDataLoad
	ldr	r1, [r4, #-1912]
	mov	r0, #3
	bl	StorageSysDataLoad
	ldmfd	sp!, {r4, r5, r6, lr}
	b	rknand_sys_storage_init
.L4708:
	.align	2
.L4707:
	.word	.LANCHOR5
	.word	.LC340
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
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, lr}
	mov	r0, #65536
	bl	ftl_malloc
	ldr	r5, .L4721
	cmp	r0, #0
	str	r0, [r5, #-1908]
	beq	.L4716
	ldr	r8, .L4721+4
	mov	r7, #0
	mov	r4, r7
	mov	r6, r7
	mov	r9, r5
.L4714:
	mov	r0, r6, asl #7
	mov	r1, #128
	ldr	r2, [r5, #-1908]
	bl	FlashBootVendorRead
	cmp	r0, #0
	bne	.L4712
	ldr	r3, [r9, #-1908]
	ldr	r0, .L4721+8
	add	r2, r3, #61440
	ldr	r1, [r3]
	ldr	r2, [r2, #4092]
	ldr	r3, [r3, #4]
	bl	printk
	ldr	r2, [r9, #-1908]
	ldr	r3, [r2]
	cmp	r3, r8
	bne	.L4713
	add	r1, r2, #61440
	ldr	r3, [r2, #4]
	ldr	r0, [r1, #4092]
	cmp	r4, r3
	movcs	r1, #0
	movcc	r1, #1
	cmp	r0, r3
	movne	r1, #0
	cmp	r1, #0
	movne	r7, r6
	movne	r4, r3
.L4713:
	cmp	r6, #1
	movne	r6, #1
	bne	.L4714
.L4720:
	cmp	r4, #0
	beq	.L4715
	mov	r0, r7, asl #7
	mov	r1, #128
	bl	FlashBootVendorRead
	cmp	r0, #0
	bne	.L4712
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4715:
	mov	r0, r2
	mov	r1, #65536
	bl	__memzero
	ldr	r3, [r5, #-1908]
	ldr	r2, .L4721+4
	mov	r0, r4
	str	r6, [r3, #4]
	str	r2, [r3]
	add	r2, r3, #61440
	str	r6, [r2, #4092]
	ldr	r2, .L4721+12
	strh	r4, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4712:
	ldr	r0, [r5, #-1908]
	bl	kfree
	mov	r3, #0
	mvn	r0, #0
	str	r3, [r5, #-1908]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4716:
	mvn	r0, #11
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4722:
	.align	2
.L4721:
	.word	.LANCHOR5
	.word	1380668996
	.word	.LC341
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
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, .L4730
	ldr	ip, [r3, #-1908]
	cmp	ip, #0
	ldrneh	r4, [ip, #10]
	movne	r3, #0
	beq	.L4728
.L4725:
	cmp	r3, r4
	bcs	.L4728
	add	lr, ip, r3, asl #3
	ldrh	r5, [lr, #16]
	cmp	r5, r0
	bne	.L4726
	ldrh	r3, [lr, #20]
	mov	r0, r1
	ldrh	r1, [lr, #18]
	mov	r4, r2
	cmp	r2, r3
	add	r1, r1, #1024
	movcs	r4, r3
	add	r1, ip, r1
	mov	r2, r4
	bl	memcpy
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L4726:
	add	r3, r3, #1
	b	.L4725
.L4728:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L4731:
	.align	2
.L4730:
	.word	.LANCHOR5
	.fnend
	.size	rk_ftl_vendor_read, .-rk_ftl_vendor_read
	.align	2
	.global	rk_ftl_vendor_write
	.type	rk_ftl_vendor_write, %function
rk_ftl_vendor_write:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r9, r2
	ldr	r2, .L4755
	.pad #28
	sub	sp, sp, #28
	ldr	r4, [r2, #-1908]
	cmp	r4, #0
	beq	.L4747
	mov	r3, r1
	add	r7, r9, #63
	ldrh	r1, [r4, #8]
	bic	r7, r7, #63
	ldrh	r2, [r4, #10]
	mov	ip, r0
	mov	r6, #0
	str	r1, [sp, #4]
.L4734:
	cmp	r6, r2
	bcs	.L4753
	add	r5, r4, r6, asl #3
	ldrh	r1, [r5, #16]
	cmp	r1, ip
	bne	.L4735
	ldrh	r1, [r5, #20]
	add	fp, r4, #1024
	add	r1, r1, #63
	bic	r1, r1, #63
	str	r1, [sp, #8]
	cmp	r9, r1
	bls	.L4736
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	subcs	r2, r2, #1
	strcs	r2, [sp, #12]
	ldrcsh	r8, [r5, #18]
	bcc	.L4747
.L4737:
	ldr	r2, [sp, #12]
	add	r5, r5, #8
	cmp	r6, r2
	bcs	.L4754
	ldrh	r10, [r5, #20]
	add	r0, fp, r8
	ldrh	r2, [r5, #16]
	add	r6, r6, #1
	ldrh	r1, [r5, #18]
	strh	r10, [r5, #12]	@ movhi
	add	r10, r10, #63
	bic	r10, r10, #63
	strh	r2, [r5, #8]	@ movhi
	strh	r8, [r5, #10]	@ movhi
	add	r1, fp, r1
	mov	r2, r10
	str	r3, [sp, #20]
	str	ip, [sp, #16]
	bl	memcpy
	add	r8, r8, r10
	ldr	r3, [sp, #20]
	ldr	ip, [sp, #16]
	b	.L4737
.L4754:
	add	r6, r4, r6, asl #3
	uxth	r8, r8
	add	r0, fp, r8
	mov	r1, r3
	strh	r8, [r6, #18]	@ movhi
	mov	r2, r9
	strh	ip, [r6, #16]	@ movhi
	uxth	r7, r7
	strh	r9, [r6, #20]	@ movhi
	add	r8, r8, r7
	bl	memcpy
	ldrh	r5, [r4, #14]
	strh	r8, [r4, #12]	@ movhi
	ldr	r3, [sp, #8]
	add	r5, r3, r5
	rsb	r7, r7, r5
	strh	r7, [r4, #14]	@ movhi
	b	.L4752
.L4736:
	ldrh	r0, [r5, #18]
	mov	r1, r3
	mov	r2, r9
	add	r0, fp, r0
	bl	memcpy
	strh	r9, [r5, #20]	@ movhi
	b	.L4752
.L4735:
	add	r6, r6, #1
	b	.L4734
.L4753:
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	bcc	.L4747
	add	r2, r4, r2, asl #3
	uxth	r7, r7
	rsb	r1, r7, r1
	strh	ip, [r2, #16]	@ movhi
	ldrh	r0, [r4, #12]
	strh	r9, [r2, #20]	@ movhi
	strh	r0, [r2, #18]	@ movhi
	add	r0, r7, r0
	strh	r1, [r4, #14]	@ movhi
	mov	r1, r3
	strh	r0, [r4, #12]	@ movhi
	ldrh	r0, [r2, #18]
	mov	r2, r9
	add	r0, r0, #1024
	add	r0, r4, r0
	bl	memcpy
	ldrh	r3, [r4, #10]
	add	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
.L4752:
	ldr	r3, [r4, #4]
	add	r2, r4, #61440
	mov	r1, #128
	add	r3, r3, #1
	str	r3, [r4, #4]
	str	r3, [r2, #4092]
	mov	r2, r4
	ldrh	r3, [r4, #8]
	add	r3, r3, #1
	uxth	r3, r3
	cmp	r3, #1
	movhi	r3, #0
	strh	r3, [r4, #8]	@ movhi
	ldr	r3, [sp, #4]
	mov	r0, r3, asl #7
	bl	FlashBootVendorWrite
	mov	r0, #0
	b	.L4733
.L4747:
	mvn	r0, #0
.L4733:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4756:
	.align	2
.L4755:
	.word	.LANCHOR5
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
	beq	.L4758
	ldr	r3, .L4774
	cmp	r6, r3
	beq	.L4760
	add	r3, r3, #1
	cmp	r6, r3
	beq	.L4761
	b	.L4772
.L4760:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4772
	ldr	r2, [r4]
	ldr	r3, .L4774+4
	cmp	r2, r3
	beq	.L4763
.L4764:
	mvn	r5, #0
	b	.L4759
.L4763:
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_read
	cmn	r0, #1
	beq	.L4764
	uxth	r2, r0
	strh	r0, [r4, #6]	@ movhi
	mov	r1, r4
	mov	r0, r5
	add	r2, r2, #8
	bl	rk_copy_to_user
	cmp	r0, #0
	moveq	r5, #0
	mvnne	r5, #13
	b	.L4759
.L4761:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4772
	ldr	r2, [r4]
	ldr	r3, .L4774+4
	cmp	r2, r3
	bne	.L4764
	ldrh	r2, [r4, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L4764
	mov	r0, r4
	mov	r1, r5
	add	r2, r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4772
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_write
	mov	r5, r0
	b	.L4759
.L4772:
	mvn	r5, #13
.L4759:
	mov	r0, r4
	bl	kfree
.L4758:
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L4775:
	.align	2
.L4774:
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
	.global	gp_ftl_api
	.global	rk_zftl_enable
	.global	g_pm_spare
	.global	pm_first_write
	.global	pm_force_gc
	.global	pm_gc_enable
	.global	pm_last_load_ram_id
	.global	pm_last_update_ram_id
	.global	pm_ram_info
	.global	sblk_gc_write_completed_queue_head
	.global	sblk_read_completed_queue_head
	.global	sblk_write_completed_queue_head
	.global	sblk_queue_head
	.global	slc_cache_sblk
	.global	xlc_data_sblk
	.global	slc_data_sblk
	.global	free_mix_sblk
	.global	free_xlc_sblk
	.global	free_slc_sblk
	.global	gp_data_xlc_data_head
	.global	gp_data_slc_data_head
	.global	gp_data_slc_cache_head
	.global	gp_free_mix_head
	.global	gp_free_xlc_head
	.global	gp_free_slc_head
	.global	gp_sblk_list_tbl
	.global	zftl_print_list_count
	.global	ftl_ext_info_first_write
	.global	ftl_sys_info_first_write
	.global	ftl_low_format_cur_blk
	.global	ftl_power_lost_flag
	.global	ftl_vpn_update_count
	.global	ftl_sblk_update_list_offset
	.global	ftl_sblk_update_list
	.global	ftl_sblk_vpn_update_id
	.global	ftl_sblk_lpa_tbl
	.global	ftl_sblk_vpn
	.global	gp_ftl_ext_info
	.global	gp_ftl_info
	.global	gp_blk_info
	.global	ftl_tmp_spare
	.global	ftl_info_spare
	.global	ftl_ext_info_data_buffer
	.global	ftl_info_data_buffer
	.global	ftl_tmp_buffer
	.global	g_ftl_info_blk
	.global	tlc_b05a_prog_tbl
	.global	tlc_prog_order
	.global	gc_des_ppa_tbl
	.global	gc_valid_page_ppa
	.global	gc_page_buf_id
	.global	gc_pre_ppa_tbl
	.global	gc_lpa_tbl
	.global	g_gc_info
	.global	gc_xlc_search_index
	.global	gc_xlc_data_index
	.global	gc_slc_cache_index
	.global	gc_slc_data_index
	.global	gc_free_slc_sblk_th
	.global	gc_slc_mode_vpn_th
	.global	gc_slc_mode_tlc_vpn_th
	.global	gc_tlc_mode_tlc_vpn_th
	.global	gc_tlc_mode_slc_vpn_th
	.global	gc_state
	.global	gc_mode
	.global	p_read_ahead_ext_buf
	.global	discard_sector_count
	.global	read_ahead_lpa
	.global	read_buf_count
	.global	read_buf_head
	.global	write_commit_count
	.global	write_commit_head
	.global	write_buf_count
	.global	write_buf_head
	.global	ftl_flush_jiffies
	.global	lpa_hash
	.global	lpa_hash_index
	.global	_c_slc_to_xlc_ec_ratio
	.global	_c_mix_max_xlc_ec_count
	.global	_c_mix_max_slc_ec_count
	.global	_c_swl_xlc_gc_th
	.global	_c_swl_slc_gc_th
	.global	_gc_after_discard_en
	.global	_last_write_time
	.global	_last_read_time
	.global	_min_slc_super_block
	.global	_max_slc_super_block
	.global	_max_xlc_super_block
	.global	_c_max_pm_sblk
	.global	_c_ftl_pm_page_num
	.global	_c_totle_log_page
	.global	_c_totle_data_density
	.global	_c_user_data_density
	.global	_c_totle_phy_density
	.global	_c_ftl_block_addr_log2
	.global	_c_ftl_block_align_addr
	.global	_c_ftl_byte_pre_page
	.global	_c_ftl_nand_blks_per_die
	.global	_c_ftl_page_pre_slc_blk
	.global	_c_ftl_page_pre_blk
	.global	_c_ftl_blk_pre_plane
	.global	_c_ftl_nand_planes_num
	.global	_c_ftl_planes_per_die
	.global	_c_ftl_sec_per_page
	.global	_c_ftl_nand_die_num
	.global	_c_ftl_nand_type
	.global	zftl_debug
	.global	g_flash_blk_info
	.global	gp_flash_info
	.global	p_free_buf_head
	.global	free_buf_count
	.global	g_buf
	.global	nandc_ecc_sts
	.global	g_nandc_v6_master_info
	.global	nandc_randomizer_en
	.global	nandc_hw_seed
	.global	fill_spare_size
	.global	g_nandc_ecc_bits
	.global	g_nandc_tran_timeout
	.global	g_nandc_ver
	.global	gp_nandc
	.global	hy_f26_ref_value
	.global	sd15_tlc_rr
	.global	sd15_slc_rr
	.global	g_nand_para_info
	.global	gp_nand_para_info
	.global	g_nand_opt_para
	.global	g_msb_page_tbl
	.global	g_lsb_page_tbl
	.global	g_die_addr
	.global	g_die_cs_idx
	.global	IDByte
	.global	flash_read_retry
	.global	_c_ftl_cs_bits
	.global	g_maxRetryCount
	.global	g_maxRegNum
	.global	g_retryMode
	.global	g_flash_toggle_mode_en
	.global	g_flash_micron_3d_tlc_b16a
	.global	g_flash_micron_3d_tlc_b05a
	.global	g_flash_micron_3d_tlc_flag
	.global	g_flash_3d_mlc_flag
	.global	g_flash_3d_tlc_flag
	.global	g_flash_multi_page_prog_en
	.global	g_flash_multi_page_read_en
	.global	g_flash_interface_mode
	.global	g_idb_ecc_bits
	.global	g_idb_slc_mode_enable
	.global	g_one_pass_program
	.global	g_slc_mode_addr2
	.global	g_slc_mode_enable
	.global	g_flash_cur_mode
	.global	g_flash_slc_mode
	.global	g_slc_page_num
	.global	g_totle_phy_block
	.global	g_block_align_addr
	.global	g_flash_reversd_blks
	.global	g_nand_max_die
	.global	g_flash_tmp_spare_buffer
	.global	g_flash_tmp_page_buffer
	.global	g_flash_sys_spare_buffer
	.global	g_flash_spare_buffer
	.global	g_flash_page_buffer
	.section	.rodata
	.align	2
.LANCHOR1 = . + 0
	.type	__func__.36787, %object
	.size	__func__.36787, 18
__func__.36787:
	.ascii	"_list_remove_node\000"
	.space	2
	.type	__func__.36812, %object
	.size	__func__.36812, 23
__func__.36812:
	.ascii	"_list_update_data_list\000"
	.space	1
	.type	__func__.21548, %object
	.size	__func__.21548, 22
__func__.21548:
	.ascii	"nand_flash_print_info\000"
	.space	2
	.type	__func__.21634, %object
	.size	__func__.21634, 11
__func__.21634:
	.ascii	"nandc_init\000"
	.space	1
	.type	toshiba_A19ref_value, %object
	.size	toshiba_A19ref_value, 45
toshiba_A19ref_value:
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
	.space	3
	.type	toshiba_15ref_value, %object
	.size	toshiba_15ref_value, 95
toshiba_15ref_value:
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
	.space	1
	.type	toshiba_ref_value, %object
	.size	toshiba_ref_value, 8
toshiba_ref_value:
	.byte	0
	.byte	4
	.byte	124
	.byte	120
	.byte	116
	.byte	8
	.byte	12
	.byte	112
	.type	__func__.21097, %object
	.size	__func__.21097, 28
__func__.21097:
	.ascii	"flash_wait_device_ready_raw\000"
	.type	__func__.21538, %object
	.size	__func__.21538, 20
__func__.21538:
	.ascii	"flash_die_info_init\000"
	.type	__func__.21161, %object
	.size	__func__.21161, 22
__func__.21161:
	.ascii	"flash_start_page_read\000"
	.space	2
	.type	toshiba_3D_tlc_value, %object
	.size	toshiba_3D_tlc_value, 399
toshiba_3D_tlc_value:
	.byte	-119
	.byte	-119
	.byte	-119
	.byte	-119
	.byte	-118
	.byte	-118
	.byte	-118
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	5
	.byte	-2
	.byte	-1
	.byte	0
	.byte	-3
	.byte	-2
	.byte	6
	.byte	-9
	.byte	-12
	.byte	-9
	.byte	-7
	.byte	-13
	.byte	-12
	.byte	-7
	.byte	-6
	.byte	-15
	.byte	-15
	.byte	-2
	.byte	-12
	.byte	-16
	.byte	-6
	.byte	-2
	.byte	-19
	.byte	-19
	.byte	-6
	.byte	-4
	.byte	-12
	.byte	-14
	.byte	-2
	.byte	-11
	.byte	-23
	.byte	-34
	.byte	-4
	.byte	-20
	.byte	-22
	.byte	-2
	.byte	-7
	.byte	-31
	.byte	-30
	.byte	-12
	.byte	-20
	.byte	-18
	.byte	2
	.byte	-15
	.byte	-19
	.byte	-36
	.byte	-12
	.byte	-28
	.byte	-34
	.byte	-6
	.byte	-15
	.byte	-11
	.byte	2
	.byte	-12
	.byte	-8
	.byte	-2
	.byte	2
	.byte	-3
	.byte	-7
	.byte	-10
	.byte	-4
	.byte	-8
	.byte	-6
	.byte	-6
	.byte	-11
	.byte	-27
	.byte	-38
	.byte	-16
	.byte	-12
	.byte	-2
	.byte	2
	.byte	-7
	.byte	-31
	.byte	-22
	.byte	-4
	.byte	-16
	.byte	-22
	.byte	-7
	.byte	-31
	.byte	-23
	.byte	-22
	.byte	-28
	.byte	-28
	.byte	-26
	.byte	2
	.byte	-7
	.byte	-11
	.byte	-14
	.byte	-8
	.byte	-12
	.byte	-10
	.byte	-10
	.byte	-27
	.byte	-25
	.byte	-22
	.byte	-20
	.byte	-28
	.byte	-22
	.byte	-7
	.byte	-23
	.byte	-29
	.byte	-34
	.byte	-24
	.byte	-32
	.byte	-22
	.byte	-10
	.byte	-11
	.byte	-29
	.byte	-18
	.byte	-12
	.byte	-24
	.byte	-22
	.byte	6
	.byte	1
	.byte	-3
	.byte	-6
	.byte	0
	.byte	-4
	.byte	-2
	.byte	10
	.byte	-3
	.byte	-7
	.byte	-6
	.byte	4
	.byte	-4
	.byte	-2
	.byte	-10
	.byte	-23
	.byte	-39
	.byte	-22
	.byte	-19
	.byte	-24
	.byte	-18
	.byte	-14
	.byte	-23
	.byte	-29
	.byte	-30
	.byte	-15
	.byte	-30
	.byte	-30
	.byte	-7
	.byte	-27
	.byte	-35
	.byte	-26
	.byte	-15
	.byte	-24
	.byte	-26
	.byte	6
	.byte	-11
	.byte	5
	.byte	-2
	.byte	-16
	.byte	-16
	.byte	-2
	.byte	-2
	.byte	-15
	.byte	-15
	.byte	-20
	.byte	-8
	.byte	-16
	.byte	-18
	.byte	6
	.byte	5
	.byte	-15
	.byte	-2
	.byte	-24
	.byte	-28
	.byte	-22
	.byte	10
	.byte	-15
	.byte	-3
	.byte	-30
	.byte	-8
	.byte	-24
	.byte	-30
	.byte	-10
	.byte	-27
	.byte	-19
	.byte	-30
	.byte	-12
	.byte	-16
	.byte	-10
	.byte	14
	.byte	-19
	.byte	-3
	.byte	-30
	.byte	4
	.byte	4
	.byte	6
	.byte	2
	.byte	1
	.byte	-3
	.byte	-10
	.byte	-8
	.byte	-4
	.byte	-6
	.byte	-2
	.byte	-15
	.byte	-11
	.byte	-26
	.byte	-8
	.byte	-20
	.byte	-30
	.byte	6
	.byte	-19
	.byte	-3
	.byte	-46
	.byte	0
	.byte	0
	.byte	2
	.byte	6
	.byte	9
	.byte	5
	.byte	2
	.byte	4
	.byte	8
	.byte	6
	.byte	8
	.byte	9
	.byte	9
	.byte	6
	.byte	8
	.byte	8
	.byte	6
	.byte	10
	.byte	13
	.byte	9
	.byte	6
	.byte	8
	.byte	12
	.byte	10
	.byte	2
	.byte	5
	.byte	1
	.byte	-2
	.byte	0
	.byte	0
	.byte	6
	.byte	12
	.byte	1
	.byte	13
	.byte	2
	.byte	12
	.byte	12
	.byte	14
	.byte	-12
	.byte	-14
	.byte	-20
	.byte	-18
	.byte	-16
	.byte	-16
	.byte	-14
	.byte	-12
	.byte	-10
	.byte	-21
	.byte	-14
	.byte	-12
	.byte	-12
	.byte	-10
	.byte	-12
	.byte	-18
	.byte	-22
	.byte	-24
	.byte	-18
	.byte	-18
	.byte	-18
	.byte	-12
	.byte	-14
	.byte	-23
	.byte	-20
	.byte	-20
	.byte	-20
	.byte	-20
	.byte	-12
	.byte	-24
	.byte	-24
	.byte	-30
	.byte	-24
	.byte	-28
	.byte	-28
	.byte	-12
	.byte	-26
	.byte	-25
	.byte	-34
	.byte	-24
	.byte	-24
	.byte	-24
	.byte	-12
	.byte	-13
	.byte	-26
	.byte	-20
	.byte	-14
	.byte	-18
	.byte	-18
	.byte	-12
	.byte	-15
	.byte	-27
	.byte	-22
	.byte	-20
	.byte	-24
	.byte	-22
	.byte	-12
	.byte	-21
	.byte	-28
	.byte	-28
	.byte	-24
	.byte	-26
	.byte	-24
	.byte	20
	.byte	16
	.byte	6
	.byte	10
	.byte	16
	.byte	12
	.byte	12
	.byte	16
	.byte	16
	.byte	8
	.byte	8
	.byte	12
	.byte	12
	.byte	12
	.byte	18
	.byte	18
	.byte	10
	.byte	8
	.byte	14
	.byte	14
	.byte	14
	.byte	16
	.byte	14
	.byte	6
	.byte	6
	.byte	12
	.byte	14
	.byte	8
	.byte	20
	.byte	18
	.byte	8
	.byte	6
	.byte	14
	.byte	14
	.byte	10
	.byte	20
	.byte	20
	.byte	6
	.byte	10
	.byte	10
	.byte	12
	.byte	12
	.byte	10
	.byte	13
	.byte	5
	.byte	2
	.byte	14
	.byte	8
	.byte	6
	.byte	6
	.byte	13
	.byte	9
	.byte	4
	.byte	14
	.byte	10
	.byte	10
	.byte	10
	.byte	13
	.byte	9
	.byte	6
	.byte	8
	.byte	12
	.byte	10
	.byte	2
	.byte	5
	.byte	1
	.byte	-2
	.byte	0
	.byte	0
	.byte	6
	.byte	12
	.byte	1
	.byte	13
	.byte	2
	.byte	12
	.byte	12
	.byte	14
	.space	1
	.type	toshiba_3D_slc_value, %object
	.size	toshiba_3D_slc_value, 11
toshiba_3D_slc_value:
	.byte	-117
	.byte	0
	.byte	-8
	.byte	8
	.byte	-16
	.byte	-24
	.byte	24
	.byte	-40
	.byte	40
	.byte	-56
	.byte	56
	.space	1
	.type	__func__.21186, %object
	.size	__func__.21186, 23
__func__.21186:
	.ascii	"flash_start_plane_read\000"
	.space	1
	.type	__func__.21072, %object
	.size	__func__.21072, 26
__func__.21072:
	.ascii	"flash_erase_duplane_block\000"
	.space	2
	.type	__func__.21083, %object
	.size	__func__.21083, 21
__func__.21083:
	.ascii	"flash_erase_block_en\000"
	.space	3
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
	.type	__func__.21817, %object
	.size	__func__.21817, 13
__func__.21817:
	.ascii	"buf_add_tail\000"
	.space	3
	.type	__func__.21830, %object
	.size	__func__.21830, 10
__func__.21830:
	.ascii	"buf_alloc\000"
	.space	2
	.type	__func__.21844, %object
	.size	__func__.21844, 16
__func__.21844:
	.ascii	"buf_remove_free\000"
	.type	zftl_debug_proc_fops, %object
	.size	zftl_debug_proc_fops, 108
zftl_debug_proc_fops:
	.word	0
	.word	seq_lseek
	.word	seq_read
	.word	zftl_debug_proc_write
	.space	28
	.word	zftl_debug_proc_open
	.space	4
	.word	single_release
	.space	52
	.type	__func__.36351, %object
	.size	__func__.36351, 12
__func__.36351:
	.ascii	"gc_add_sblk\000"
	.type	__func__.36445, %object
	.size	__func__.36445, 19
__func__.36445:
	.ascii	"gc_write_completed\000"
	.space	1
	.type	__func__.36994, %object
	.size	__func__.36994, 18
__func__.36994:
	.ascii	"ftl_alloc_sys_blk\000"
	.space	2
	.type	__func__.37004, %object
	.size	__func__.37004, 17
__func__.37004:
	.ascii	"ftl_free_sys_blk\000"
	.space	3
	.type	__func__.37143, %object
	.size	__func__.37143, 23
__func__.37143:
	.ascii	"ftl_get_ppa_from_index\000"
	.space	1
	.type	__func__.37183, %object
	.size	__func__.37183, 22
__func__.37183:
	.ascii	"ftl_get_new_free_page\000"
	.space	2
	.type	__func__.37194, %object
	.size	__func__.37194, 22
__func__.37194:
	.ascii	"ftl_ext_alloc_new_blk\000"
	.space	2
	.type	__func__.36501, %object
	.size	__func__.36501, 16
__func__.36501:
	.ascii	"gc_free_src_blk\000"
	.type	__func__.36102, %object
	.size	__func__.36102, 14
__func__.36102:
	.ascii	"ftl_write_buf\000"
	.space	2
	.type	__func__.36147, %object
	.size	__func__.36147, 18
__func__.36147:
	.ascii	"zftl_add_read_buf\000"
	.space	2
	.type	__func__.36952, %object
	.size	__func__.36952, 15
__func__.36952:
	.ascii	"ftl_alloc_sblk\000"
	.space	1
	.type	__func__.37540, %object
	.size	__func__.37540, 13
__func__.37540:
	.ascii	"pm_free_sblk\000"
	.space	3
	.type	__func__.37580, %object
	.size	__func__.37580, 17
__func__.37580:
	.ascii	"pm_alloc_new_blk\000"
	.space	3
	.type	__func__.37608, %object
	.size	__func__.37608, 21
__func__.37608:
	.ascii	"pm_select_ram_region\000"
	.space	3
	.type	__func__.36037, %object
	.size	__func__.36037, 17
__func__.36037:
	.ascii	"lpa_rebuild_hash\000"
	.space	3
	.type	__func__.21868, %object
	.size	__func__.21868, 21
__func__.21868:
	.ascii	"flash_info_data_init\000"
	.space	3
	.type	__func__.21014, %object
	.size	__func__.21014, 19
__func__.21014:
	.ascii	"flash_read_page_en\000"
	.space	1
	.type	__func__.21174, %object
	.size	__func__.21174, 31
__func__.21174:
	.ascii	"flash_complete_plane_page_read\000"
	.space	1
	.type	__func__.21199, %object
	.size	__func__.21199, 25
__func__.21199:
	.ascii	"flash_complete_page_read\000"
	.space	3
	.type	__func__.37379, %object
	.size	__func__.37379, 31
__func__.37379:
	.ascii	"queue_wait_first_req_completed\000"
	.space	1
	.type	__func__.37460, %object
	.size	__func__.37460, 15
__func__.37460:
	.ascii	"sblk_read_page\000"
	.space	1
	.type	__func__.21150, %object
	.size	__func__.21150, 22
__func__.21150:
	.ascii	"flash_start_page_prog\000"
	.space	2
	.type	__func__.37433, %object
	.size	__func__.37433, 15
__func__.37433:
	.ascii	"sblk_prog_page\000"
	.space	1
	.type	__func__.21121, %object
	.size	__func__.21121, 26
__func__.21121:
	.ascii	"flash_start_tlc_page_prog\000"
	.space	2
	.type	__func__.21132, %object
	.size	__func__.21132, 29
__func__.21132:
	.ascii	"flash_start_3d_mlc_page_prog\000"
	.space	3
	.type	__func__.21056, %object
	.size	__func__.21056, 19
__func__.21056:
	.ascii	"flash_prog_page_en\000"
	.space	1
	.type	__func__.37257, %object
	.size	__func__.37257, 14
__func__.37257:
	.ascii	"ftl_prog_page\000"
	.space	2
	.type	__func__.36920, %object
	.size	__func__.36920, 15
__func__.36920:
	.ascii	"ftl_info_flush\000"
	.space	1
	.type	__func__.37219, %object
	.size	__func__.37219, 19
__func__.37219:
	.ascii	"ftl_ext_info_flush\000"
	.space	1
	.type	__func__.37231, %object
	.size	__func__.37231, 18
__func__.37231:
	.ascii	"ftl_ext_info_init\000"
	.space	2
	.type	__func__.37110, %object
	.size	__func__.37110, 23
__func__.37110:
	.ascii	"ftl_open_sblk_recovery\000"
	.space	1
	.type	__func__.37590, %object
	.size	__func__.37590, 14
__func__.37590:
	.ascii	"pm_write_page\000"
	.space	2
	.type	__func__.21890, %object
	.size	__func__.21890, 17
__func__.21890:
	.ascii	"flash_info_flush\000"
	.space	3
	.type	__func__.21857, %object
	.size	__func__.21857, 20
__func__.21857:
	.ascii	"flash_info_blk_init\000"
	.type	__func__.21593, %object
	.size	__func__.21593, 16
__func__.21593:
	.ascii	"nand_flash_init\000"
	.type	__func__.37086, %object
	.size	__func__.37086, 16
__func__.37086:
	.ascii	"ftl_sysblk_dump\000"
	.type	__func__.37524, %object
	.size	__func__.37524, 16
__func__.37524:
	.ascii	"load_l2p_region\000"
	.type	__func__.37564, %object
	.size	__func__.37564, 6
__func__.37564:
	.ascii	"pm_gc\000"
	.space	2
	.type	__func__.36432, %object
	.size	__func__.36432, 22
__func__.36432:
	.ascii	"gc_update_l2p_map_new\000"
	.space	2
	.type	__func__.36545, %object
	.size	__func__.36545, 16
__func__.36545:
	.ascii	"gc_scan_src_blk\000"
	.type	__func__.36603, %object
	.size	__func__.36603, 20
__func__.36603:
	.ascii	"gc_scan_static_data\000"
	.type	__func__.36666, %object
	.size	__func__.36666, 18
__func__.36666:
	.ascii	"gc_block_vpn_scan\000"
	.space	2
	.type	__func__.37062, %object
	.size	__func__.37062, 14
__func__.37062:
	.ascii	"ftl_sblk_dump\000"
	.space	2
	.type	__func__.36180, %object
	.size	__func__.36180, 10
__func__.36180:
	.ascii	"zftl_read\000"
	.space	2
	.type	__func__.36398, %object
	.size	__func__.36398, 21
__func__.36398:
	.ascii	"gc_check_data_one_wl\000"
	.space	3
	.type	__func__.37164, %object
	.size	__func__.37164, 19
__func__.37164:
	.ascii	"ftl_update_l2p_map\000"
	.space	1
	.type	__func__.36084, %object
	.size	__func__.36084, 17
__func__.36084:
	.ascii	"ftl_write_commit\000"
	.space	3
	.type	__func__.36474, %object
	.size	__func__.36474, 16
__func__.36474:
	.ascii	"gc_do_copy_back\000"
	.type	__func__.36694, %object
	.size	__func__.36694, 11
__func__.36694:
	.ascii	"zftl_do_gc\000"
	.space	1
	.type	__func__.36211, %object
	.size	__func__.36211, 13
__func__.36211:
	.ascii	"_ftl_discard\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"\012!!!!! error @ func:%s - line:%d\012\000"
.LC1:
	.ascii	"%s\012\000"
.LC2:
	.ascii	"FTL version: 6.0.18 20190918\000"
.LC3:
	.ascii	"zftl_debug:0x%x\012\000"
.LC4:
	.ascii	"...%s enter...\012\000"
.LC5:
	.ascii	"No.0 FLASH ID: %x %x %x %x %x %x\012\000"
.LC6:
	.ascii	"DiePerChip: %x\012\000"
.LC7:
	.ascii	"SectPerPage: %x\012\000"
.LC8:
	.ascii	"PagePerBlk: %x\012\000"
.LC9:
	.ascii	"Cell: %x\012\000"
.LC10:
	.ascii	"PlanePerDie: %x\012\000"
.LC11:
	.ascii	"BlkPerPlane: %x\012\000"
.LC12:
	.ascii	"die gap: %x\012\000"
.LC13:
	.ascii	"lsbMode: %x\012\000"
.LC14:
	.ascii	"ReadRetryMode: %x\012\000"
.LC15:
	.ascii	"ecc: %x\012\000"
.LC16:
	.ascii	"idb ecc: %x\012\000"
.LC17:
	.ascii	"g_flash_slc_mode: %x\012\000"
.LC18:
	.ascii	"OptMode: %x\012\000"
.LC19:
	.ascii	"g_nand_max_die: %x\012\000"
.LC20:
	.ascii	"Cache read enable: %x\012\000"
.LC21:
	.ascii	"Cache random read enable: %x\012\000"
.LC22:
	.ascii	"Cache prog enable: %x\012\000"
.LC23:
	.ascii	"multi read enable: %x\012\000"
.LC24:
	.ascii	"multi prog enable: %x\012\000"
.LC25:
	.ascii	"interleave enable: %x\012\000"
.LC26:
	.ascii	"read retry enable: %x\012\000"
.LC27:
	.ascii	"randomizer enable: %x\012\000"
.LC28:
	.ascii	"SDR enable: %x\012\000"
.LC29:
	.ascii	"ONFI enable: %x\012\000"
.LC30:
	.ascii	"TOGGLE enable: %x\012\000"
.LC31:
	.ascii	"g_flash_slc_mode: %x %x\012\000"
.LC32:
	.ascii	"MultiPlaneProgCmd: %x %x\012\000"
.LC33:
	.ascii	"MultiPlaneReadCmd: %x %x\012\000"
.LC34:
	.ascii	"g_flash_toggle_mode_en: %x\012\000"
.LC35:
	.ascii	"...%s enter... %p\012\000"
.LC36:
	.ascii	"0:%x %x %x %x %x\012\000"
.LC37:
	.ascii	"g_nandc_ver...%d\012\000"
.LC38:
	.ascii	"nand sdr mode %x\012\000"
.LC39:
	.ascii	"nand ddr mode %x\012\000"
.LC40:
	.ascii	"No.%d FLASH ID:%x %x %x %x %x %x\012\000"
.LC41:
	.ascii	"otp:%x %x %x %x\012\000"
.LC42:
	.ascii	"bad block test:%x %x\012\000"
.LC43:
	.ascii	"flash_erase_duplane_block %x %x %x\012\000"
.LC44:
	.ascii	"flash_erase_duplane_block pageadd = %x status = %x\012"
	.ascii	"\000"
.LC45:
	.ascii	"flash_erase_block %x %x %x\012\000"
.LC46:
	.ascii	"flash_erase_block %d block = %x status = %x\012\000"
.LC47:
	.ascii	"erase done: %x\012\000"
.LC48:
	.ascii	"flash_mask_bad_block %d %d\012\000"
.LC49:
	.ascii	"zftl_debug\000"
.LC50:
	.ascii	"FLASH ID: %x %x %x %x %x %x\012\000"
.LC51:
	.ascii	"density: %d MB\012\000"
.LC52:
	.ascii	"device density: %d MB\012\000"
.LC53:
	.ascii	"FTL INFO:\012\000"
.LC54:
	.ascii	"max_lpn = 0x%x\012\000"
.LC55:
	.ascii	"density = 0x%x\012\000"
.LC56:
	.ascii	"slc vpn = 0x%x\012\000"
.LC57:
	.ascii	"xlc vpn = 0x%x\012\000"
.LC58:
	.ascii	"free slc blk = 0x%x\012\000"
.LC59:
	.ascii	"free xlc blk = 0x%x\012\000"
.LC60:
	.ascii	"free mix blk = 0x%x\012\000"
.LC61:
	.ascii	"slc data blk = 0x%x\012\000"
.LC62:
	.ascii	"slc cache blk = 0x%x\012\000"
.LC63:
	.ascii	"xlc data blk = 0x%x\012\000"
.LC64:
	.ascii	"free buf = %d, %d, %d\012\000"
.LC65:
	.ascii	"bad blk = %d %d\012\000"
.LC66:
	.ascii	"TBW = %d MB\012\000"
.LC67:
	.ascii	"TBR = %d MB\012\000"
.LC68:
	.ascii	"POC = %d\012\000"
.LC69:
	.ascii	"PLC = %d\012\000"
.LC70:
	.ascii	"sys run time = %d S\012\000"
.LC71:
	.ascii	"slc mode = %d\012\000"
.LC72:
	.ascii	"prog err = %d\012\000"
.LC73:
	.ascii	"read err = %d\012\000"
.LC74:
	.ascii	"GC XLC page = %d\012\000"
.LC75:
	.ascii	"GC SLC page = %d\012\000"
.LC76:
	.ascii	"discard page = 0x%x\012\000"
.LC77:
	.ascii	"version = %d\012\000"
.LC78:
	.ascii	"acblk = 0x%x %d %d\012\000"
.LC79:
	.ascii	"tmblk = 0x%x %d %d\012\000"
.LC80:
	.ascii	"gcblk = 0x%x %d %d\012\000"
.LC81:
	.ascii	"slc ec = %d, %d, %d, %d, %d\012\000"
.LC82:
	.ascii	"xlc ec = %d, %d, %d, %d, %d\012\000"
.LC83:
	.ascii	"gc free blk th = %d\012\000"
.LC84:
	.ascii	"gc vpn th = %d %d %d %d %d\012\000"
.LC85:
	.ascii	"swl blk = %x %x %x %x\012\000"
.LC86:
	.ascii	"rf info = %x %x %x %x %x\012\000"
.LC87:
	.ascii	"gc_add_sblk = %d, %d, %d, %d, %d, %d, %d\012\000"
.LC88:
	.ascii	"gc_add_sblk = %d, %d, %d\012\000"
.LC89:
	.ascii	"gc_add_sblk = %d, %d, %d,last update:%d, %d\012\000"
.LC90:
	.ascii	"gc_add_sblk = %d, %d, %d, %d, %d, %d\012\000"
.LC91:
	.ascii	"gc_mark_bad_ppa %d %x %x\012\000"
.LC92:
	.ascii	"status: %x, ppa: %x\012\000"
.LC93:
	.ascii	"gc_write_completed: %x  %x  %d %x\012\000"
.LC94:
	.ascii	"zftl_get_gc_node cache = %x index = %d vpn = %x\012"
	.ascii	"\000"
.LC95:
	.ascii	"gc_search_src_blk mode = %x, src mode = %x, count= "
	.ascii	"%d %d\012\000"
.LC96:
	.ascii	"%d gc_free_temp_buf buf id= %x\012\000"
.LC97:
	.ascii	"gc: b:%x,p:%x,i:%x; free buf=%d %d free slc th: %d\012"
	.ascii	"\000"
.LC98:
	.ascii	"swl_tlc_free_mini_ec_blk alloc sblk %x\012\000"
.LC99:
	.ascii	"zftl_get_free_sblk %x %d, %p %d %d\012\000"
.LC100:
	.ascii	"zftl_gc_get_free_sblk %x %x %x, %d %d %d\012\000"
.LC101:
	.ascii	"swl_slc_free_mini_ec_blk alloc sblk %x\012\000"
.LC102:
	.ascii	"list count:%p %d\012\000"
.LC103:
	.ascii	"%d: node:%x %x %x %x, %d %d %d %d %d\012\000"
.LC104:
	.ascii	"gc_static_wearleveling: min blk: %x,sec=%d,xec = %d"
	.ascii	" ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC105:
	.ascii	"gc_static_wearleveling: min slc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC106:
	.ascii	"gc_static_wearleveling: min tlc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC107:
	.ascii	"gc_static_wearleveling: max slc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC108:
	.ascii	"gc_static_wearleveling: max xlc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC109:
	.ascii	"gc_static_wearleveling: slc blk: %x, tlc blk: %d av"
	.ascii	"g slc ec: %d, avg tlc ec: %d \012\000"
.LC110:
	.ascii	"gc_static_wearleveling: min slc ec: %x, min tlc ec:"
	.ascii	" %d max slc ec: %d, max tlc ec: %d; %d %d\012\000"
.LC111:
	.ascii	"swl add tlc gc = %x, %d, %d, %d, %d, %d\012\000"
.LC112:
	.ascii	"swl add slc gc  = %x, %d, %d, %d, %d, %d\012\000"
.LC113:
	.ascii	"ftl_vpn_decrement %x = %d, %d\012\000"
.LC114:
	.ascii	"mask bad block:cs %x %x block: %x %x\012\000"
.LC115:
	.ascii	"gc_free_bad_sblk 0x%x\012\000"
.LC116:
	.ascii	"swl_slc_free_mini_ec_blk sblk %x\012\000"
.LC117:
	.ascii	"gc_free_src_blk = %x, vpn = %d\012\000"
.LC118:
	.ascii	"gc_free_src_blk %x, %d\012\000"
.LC119:
	.ascii	"bad blk = %x, %x free blk: s:%x,t:%x,m:%x, data blk"
	.ascii	":s:%x,%x,t%x vpn: s:%x t:%x, max_vpn: %x\012\000"
.LC120:
	.ascii	"totle w: %d MB,r: %d MB %d dv:0x%X,poc:%d\012\000"
.LC121:
	.ascii	"gc xlc page: %d,gc slc page: %d, tmp w: %d MB\012\000"
.LC122:
	.ascii	"slc ec: %d,%d,%d,%d,%d,tlc ec: %d,%d,%d,%d,%d\012\000"
.LC123:
	.ascii	"gc th: tlc_tlc: %d tlc_slc: %d slc_slc: %d slc_tlc:"
	.ascii	"%d free_th: %d\012\000"
.LC124:
	.ascii	"swl : %x %x %x %x %x %x\012\000"
.LC125:
	.ascii	"ftl prog error =%x, lpa = %x, ppa= %x\012\000"
.LC126:
	.ascii	"ftl re prog: lpa = %x, ppa= %x\012\000"
.LC127:
	.ascii	"alloc sblk %x %d\012\000"
.LC128:
	.ascii	"dump_sblk_queue: %d\012\000"
.LC129:
	.ascii	"buf id= %d state = %d ppa = %x\012\000"
.LC130:
	.ascii	"GC PM block %x %x %x %d\012\000"
.LC131:
	.ascii	"ftl_free_no_use_map_blk %x %x %x %d\012\000"
.LC132:
	.ascii	"blk %x is bad block\012\000"
.LC133:
	.ascii	"pm_alloc_new_blk: %x %x %x %x\012\000"
.LC134:
	.ascii	"%s %d %d\012\000"
.LC135:
	.ascii	"free blk vpn error: %x %x\012\000"
.LC136:
	.ascii	"...%d @ %s\012\000"
.LC137:
	.ascii	"rk_ftl_de_init %x\012\000"
.LC138:
	.ascii	"%s %p:0x%x:\000"
.LC139:
	.ascii	"%x \000"
.LC140:
	.ascii	"\000"
.LC141:
	.ascii	"otp error! %d\000"
.LC142:
	.ascii	"rr\000"
.LC143:
	.ascii	"flash_abort_clear = %d\012\000"
.LC144:
	.ascii	"%d mtrans_cnt = %d page_num = %d\012\000"
.LC145:
	.ascii	"nandc:\000"
.LC146:
	.ascii	"%d flReg.d32=%x %x\012\000"
.LC147:
	.ascii	"nandc_xfer_done read error %x\012\000"
.LC148:
	.ascii	"dqs data abort %x\012\000"
.LC149:
	.ascii	"dqs data timeout %x\012\000"
.LC150:
	.ascii	"xfer error %x\012\000"
.LC151:
	.ascii	"flash_read_page %x %x %x\012\000"
.LC152:
	.ascii	"MT %d row=%x,last status %d,status = %d\012\000"
.LC153:
	.ascii	"MT RR %d row=%x,count %d,status=%d\012\000"
.LC154:
	.ascii	"toshiba SRR %d row=%x, status=%d\012\000"
.LC155:
	.ascii	"toshiba TRR %d row=%x, status=%d\012\000"
.LC156:
	.ascii	"toshiba RR %d row=%x,count %d,status=%d\012\000"
.LC157:
	.ascii	"hynix RR %d row=%x, count %d, status=%d\012\000"
.LC158:
	.ascii	"%d flash_ddr_tuning_read %x ecc=%d\012\000"
.LC159:
	.ascii	"sync para %d\012\000"
.LC160:
	.ascii	"DDR mode Read error %x %x\012\000"
.LC161:
	.ascii	"SDR mode Read %x %x ecc:%x\012\000"
.LC162:
	.ascii	"flash_read_page_en %x %x %x\012\000"
.LC163:
	.ascii	"flash_read_page_en %x %x error_ecc %d %d\012\000"
.LC164:
	.ascii	"flash_get_last_written_page: %x %x %x\012\000"
.LC165:
	.ascii	"read page: %x %x %x %x\012\000"
.LC166:
	.ascii	"flash_complete_page_read %x %x error_ecc %d %d\012\000"
.LC167:
	.ascii	"read: %x %x %x %x\012\000"
.LC168:
	.ascii	"0set buf %d,status = %x, ppa = %x lun state = %d\012"
	.ascii	"\000"
.LC169:
	.ascii	"prog end %x %x error_ecc %d %d\012\000"
.LC170:
	.ascii	"1set buf %d,status = %x, ppa = %x lun state = %d\012"
	.ascii	"\000"
.LC171:
	.ascii	"dp prog end %x %x error_ecc %d %d\012\000"
.LC172:
	.ascii	"flash_prog_page %x %x %x\012\000"
.LC173:
	.ascii	"flash_prog_page page_addr = %x status = %x\012\000"
.LC174:
	.ascii	"sblk_prog_page ppa = %x, count = %d\012\000"
.LC175:
	.ascii	"sblk_3d_tlc_dump_prog wl_addr= %x ppa = %x ppa = %x"
	.ascii	"\012\000"
.LC176:
	.ascii	"sblk_mlc_dump_prog wl_addr= %x ppa = %x ppa = %x\012"
	.ascii	"\000"
.LC177:
	.ascii	"flash_prog_page_en:%x %x %x\012\000"
.LC178:
	.ascii	"w d:\000"
.LC179:
	.ascii	"w s:\000"
.LC180:
	.ascii	"spare\000"
.LC181:
	.ascii	"data\000"
.LC182:
	.ascii	"write error: %x\012\000"
.LC183:
	.ascii	"g_ftl_info_blk blk = %x, page = %x version = %d\012"
	.ascii	"\000"
.LC184:
	.ascii	"%d %x @%d %x\012\000"
.LC185:
	.ascii	"ftl_info_blk_init %d %d %x\012\000"
.LC186:
	.ascii	"ink flag: %x\012\000"
.LC187:
	.ascii	"%s %d %d %x %x\012\000"
.LC188:
	.ascii	"%s %x %x %x\012\000"
.LC189:
	.ascii	"saved_active_page  = %x\012\000"
.LC190:
	.ascii	"saved_active_plane = %x\012\000"
.LC191:
	.ascii	"sblk = %x\012\000"
.LC192:
	.ascii	"phy_blk = %x %x\012\000"
.LC193:
	.ascii	"num_planes = %x\012\000"
.LC194:
	.ascii	"sblk = %x, vpn0 = %d, vpn1 = %d\012\000"
.LC195:
	.ascii	"dump_write_lpa = %x %x %x %x\012\000"
.LC196:
	.ascii	"dump write new ppa = %x, last ppa = %x lpa = %x\012"
	.ascii	"\000"
.LC197:
	.ascii	"dump write = %x %x %x\012\000"
.LC198:
	.ascii	"dump write hash update = %x %x %x\012\000"
.LC199:
	.ascii	"pm_write_page write error: %x\012\000"
.LC200:
	.ascii	"ftl_sblk_dump_write = %x %d %d %d %d\012\000"
.LC201:
	.ascii	"blk= %x, page=%x, ppa = %x, status = %x, data:%x %x"
	.ascii	" %x %x, spare: %x %x %x %x\012\000"
.LC202:
	.ascii	"ftl_sblk_dump_write2 = %x %d %d %d\012\000"
.LC203:
	.ascii	"ftl_sblk_dump_write = %x %x\012\000"
.LC204:
	.ascii	"ftl_sblk_dump_write done = %x\012\000"
.LC205:
	.ascii	"ftl_ink_check_sblk = %x %d %d\012\000"
.LC206:
	.ascii	"ftl_ink_check_sblk = %x %d %d end\012\000"
.LC207:
	.ascii	"finfo:\000"
.LC208:
	.ascii	"flash_info_flush id = %x, page = %x\012\000"
.LC209:
	.ascii	"sys_info_flush error:%x\012\000"
.LC210:
	.ascii	"...%d @ %s %d %p\012\000"
.LC211:
	.ascii	"no sys info %x\012\000"
.LC212:
	.ascii	"l2p:\000"
.LC213:
	.ascii	"free_buf_count: %d\012\000"
.LC214:
	.ascii	"g_ftl_info_blk blk:0x%x, index:0x%x, page:0x%x\012\000"
.LC215:
	.ascii	"ftl_ext_info_blk blk:0x%x, page:0x%x\012\000"
.LC216:
	.ascii	"ac_blk:0x%x, page:0x%x, index:0x%x, free:0x%x, page"
	.ascii	"_index:0x%x\012\000"
.LC217:
	.ascii	"tmp_blk:0x%x, page:0x%x, index:0x%x, free:0x%x, pag"
	.ascii	"e_index:0x%x\012\000"
.LC218:
	.ascii	"gc_blk:0x%x, page:0x%x, index:0x%x, free:0x%x, page"
	.ascii	"_index:0x%x\012\000"
.LC219:
	.ascii	"lpa:\000"
.LC220:
	.ascii	"vpn:\000"
.LC221:
	.ascii	"sblk:\000"
.LC222:
	.ascii	"lpa_hash:\000"
.LC223:
	.ascii	"lpa_hash_index:\000"
.LC224:
	.ascii	"%s w error lpn = %x, max ppa = %d\012\000"
.LC225:
	.ascii	"region_id = %d, pm_max_region = %d\012\000"
.LC226:
	.ascii	"load_l2p_region no ppa = %x , %x, all setting 0xff."
	.ascii	"...\012\000"
.LC227:
	.ascii	"load_l2p_region = %x,%x,%x, %x\012\000"
.LC228:
	.ascii	"pm_ppa:\000"
.LC229:
	.ascii	"data:\000"
.LC230:
	.ascii	"spare:\000"
.LC231:
	.ascii	"pm_init posr %x %x %x\012\000"
.LC232:
	.ascii	"pm_init recovery %x %x %x\012\000"
.LC233:
	.ascii	"pm_log2phys  lpn = %d, max lpn = %d\012\000"
.LC234:
	.ascii	"ppa = %x, status = %x, data:%x %x %x %x, spare: %x "
	.ascii	"%x %x %x\012\000"
.LC235:
	.ascii	"ppa = %x, status = %x, %x %x spare: %x %x %x %x\012"
	.ascii	"\000"
.LC236:
	.ascii	"gc_recovery: %x vpn = %x\012\000"
.LC237:
	.ascii	"_c_user_data_density := %d\012\000"
.LC238:
	.ascii	"_c_totle_phy_density := %d\012\000"
.LC239:
	.ascii	"_c_totle_log_page := %d\012\000"
.LC240:
	.ascii	"_c_totle_data_density := %d\012\000"
.LC241:
	.ascii	"_c_ftl_pm_page_num := %d\012\000"
.LC242:
	.ascii	"_c_ftl_byte_pre_page := %d\012\000"
.LC243:
	.ascii	"_c_max_pm_sblk := %d\012\000"
.LC244:
	.ascii	"_min_slc_super_block := %d\012\000"
.LC245:
	.ascii	"_max_xlc_super_block := %d\012\000"
.LC246:
	.ascii	"gp_ftl_ext_info %p %p %p\012\000"
.LC247:
	.ascii	"flash info size: %d %d %d\012\000"
.LC248:
	.ascii	"ftl_init %x\012\000"
.LC249:
	.ascii	"gc_update_l2p_map_new sblk %x\012\000"
.LC250:
	.ascii	"gc_update_l2p_map_new: %x %x %x\012\000"
.LC251:
	.ascii	"lpa: %x %x %x\012\000"
.LC252:
	.ascii	"gc_update_l2p_map_new: %x vpn = %x vpn1 = %x done\012"
	.ascii	"\000"
.LC253:
	.ascii	"gc_scan_src_blk = %x, vpn = %d\012\000"
.LC254:
	.ascii	"gc_scan_src_blk = %x, s vpn0 = %d, c vpn1 = %d\012\000"
.LC255:
	.ascii	"gc_block_vpn_scan = %x, s vpn0 = %d, c vpn1 = %d f:"
	.ascii	"%d\012\000"
.LC256:
	.ascii	"ftl_sblk_dump = %x %d %d %d %d\012\000"
.LC257:
	.ascii	"ftl_sblk_dump = %x %x %x %x\012\000"
.LC258:
	.ascii	"page_addr = %x, lpa=%x vpn = %d\012\000"
.LC259:
	.ascii	"index= %x, lpa=%x\012\000"
.LC260:
	.ascii	"block = %x, vpn=%x check vpn = %d\012\000"
.LC261:
	.ascii	"ftl_read %x %x %x\012\000"
.LC262:
	.ascii	"ftl_read refresh =%x, lpa = %x, ppa= %x\012\000"
.LC263:
	.ascii	"id=%d, status = %x, lpa = %x, ppa = %x spare = %x %"
	.ascii	"x %x %x\012\000"
.LC264:
	.ascii	"zftl debug cmd: %s\012\000"
.LC265:
	.ascii	"cmd:\000"
.LC266:
	.ascii	"dumpl2p\000"
.LC267:
	.ascii	"pm l2p:\000"
.LC268:
	.ascii	"pm blk:\000"
.LC269:
	.ascii	"dumppm:\000"
.LC270:
	.ascii	"p_cmd: %s\012\000"
.LC271:
	.ascii	"pm ram = %x, %x\012\000"
.LC272:
	.ascii	"ram:\000"
.LC273:
	.ascii	"pm:\000"
.LC274:
	.ascii	"dumpsys\000"
.LC275:
	.ascii	"dumplist:\000"
.LC276:
	.ascii	"vpncheck\000"
.LC277:
	.ascii	"dumpppa:\000"
.LC278:
	.ascii	"dumpblk:\000"
.LC279:
	.ascii	"setzdebug:\000"
.LC280:
	.ascii	"lpa2ppa:\000"
.LC281:
	.ascii	"lpa: %x--> ppa: %x\012\000"
.LC282:
	.ascii	"help:\012\000"
.LC283:
	.ascii	"1. echo dumpl2p > /proc/zftl_debug\012\000"
.LC284:
	.ascii	"2. echo dumppm:x > /proc/zftl_debug\012\000"
.LC285:
	.ascii	"3. echo dumpsys > /proc/zftl_debug\012\000"
.LC286:
	.ascii	"4. echo dumpppa:x > /proc/zftl_debug\012\000"
.LC287:
	.ascii	"5. echo vpncheck > /proc/zftl_debug\012\000"
.LC288:
	.ascii	"6. echo setzdebug:x > /proc/zftl_debug\012\000"
.LC289:
	.ascii	"7. echo dumplist > /proc/zftl_debug\012\000"
.LC290:
	.ascii	"8. echo lpa2ppa:x> /proc/zftl_debug\012\000"
.LC291:
	.ascii	"gc_lpa:\000"
.LC292:
	.ascii	"gc_ppa:\000"
.LC293:
	.ascii	"err: ppa = %x, status = %x, %x %x spare: %x %x %x %"
	.ascii	"x\012\000"
.LC294:
	.ascii	"ftl_update_l2p_map: %x %x %x\012\000"
.LC295:
	.ascii	"ftl_update_l2p_map\000"
.LC296:
	.ascii	"lpa_tbl:\000"
.LC297:
	.ascii	"sblk %x vpn: %d %d\012\000"
.LC298:
	.ascii	"error gc_add_sblk: %x\012\000"
.LC299:
	.ascii	"%d read error: ppa:%x, lpa:%x, status:%x\012\000"
.LC300:
	.ascii	"gc page in buf: lpa %x ppa = %x pageindex= %x\012\000"
.LC301:
	.ascii	"gc_do_copy_back: lpa %x des_ppa = %x %x gc_ppa= %x "
	.ascii	"page_index= %d\012\000"
.LC302:
	.ascii	"gc_do_copy_back: %x %x %x %x gc_ppa:%x %x %x %d\012"
	.ascii	"\000"
.LC303:
	.ascii	"%d prog_step: %x %x buf id= %x ppa = %x hash=%d id "
	.ascii	"= %x plane = %x lpa=%x %x\012\000"
.LC304:
	.ascii	"gc %d: %d %d %d %d %d %d %d\012\000"
.LC305:
	.ascii	"gc %d: %d %d %d %d %d %d\012\000"
.LC306:
	.ascii	"GC_STATE_SCAN_ALL_PAGE = %x, vpn0 = %d, vpn1 = %d\012"
	.ascii	"\000"
.LC307:
	.ascii	"gc free %x, %d\012\000"
.LC308:
	.ascii	"ftlwrite %x %x %x %x\012\000"
.LC309:
	.ascii	"ftl_discard:(%x, %x, %x, %x)\012\000"
.LC310:
	.ascii	"id_block_prog_msb_ff_data slc page = %d pageadd=%x "
	.ascii	"%x\012\000"
.LC311:
	.ascii	"write_idblock fix data %x %x\012\000"
.LC312:
	.ascii	"idblk:\000"
.LC313:
	.ascii	"write_idblock totle_sec %x %x\012\000"
.LC314:
	.ascii	"prog page: %x %x %x, %p %x %x %x\012\000"
.LC315:
	.ascii	"wl_lba %p %x %x %x\012\000"
.LC316:
	.ascii	"READ_SECTOR_IO\012\000"
.LC317:
	.ascii	"rk_copy_from_user error\012\000"
.LC318:
	.ascii	"READ_SECTOR_IO %x %x\012\000"
.LC319:
	.ascii	"rk_copy_to_user error\012\000"
.LC320:
	.ascii	"WRITE_SECTOR_IO\012\000"
.LC321:
	.ascii	"WRITE_SECTOR_IO %x %x\012\000"
.LC322:
	.ascii	"END_WRITE_SECTOR_IO\012\000"
.LC323:
	.ascii	"END_WRITE_SECTOR_IO %x %x\012\000"
.LC324:
	.ascii	"GET_FLASH_INFO_IO\012\000"
.LC325:
	.ascii	"GET_BAD_BLOCK_IO\012\000"
.LC326:
	.ascii	"GET_LOCK_FLAG_IO\012\000"
.LC327:
	.ascii	"GET_PUBLIC_KEY_IO\012\000"
.LC328:
	.ascii	"RKNAND_GET_DRM_KEY\012\000"
.LC329:
	.ascii	"RKNAND_STORE_DRM_KEY\012\000"
.LC330:
	.ascii	"RKNAND_DIASBLE_SECURE_BOOT\012\000"
.LC331:
	.ascii	"RKNAND_ENASBLE_SECURE_BOOT\012\000"
.LC332:
	.ascii	"RKNAND_GET_SN_SECTOR\012\000"
.LC333:
	.ascii	"RKNAND_LOADER_UNLOCK\012\000"
.LC334:
	.ascii	"RKNAND_LOADER_STATUS\012\000"
.LC335:
	.ascii	"RKNAND_LOADER_LOCK\012\000"
.LC336:
	.ascii	"LockKey not match %d\012\000"
.LC337:
	.ascii	"RKNAND_GET_VENDOR_SECTOR\012\000"
.LC338:
	.ascii	"RKNAND_STORE_VENDOR_SECTOR\012\000"
.LC339:
	.ascii	"return ret = %lx\012\000"
.LC340:
	.ascii	"secureBootEn check error\012\000"
.LC341:
	.ascii	"\0013vendor storage %x,%x,%x\012\000"
	.data
	.align	2
.LANCHOR2 = . + 0
.LANCHOR6 = . + 8184
	.type	zftl_debug, %object
	.size	zftl_debug, 4
zftl_debug:
	.word	17476
	.type	g_nand_para_info, %object
	.size	g_nand_para_info, 32
g_nand_para_info:
	.byte	6
	.byte	-104
	.byte	58
	.byte	-104
	.byte	-77
	.byte	118
	.byte	114
	.byte	1
	.byte	1
	.byte	32
	.short	768
	.byte	3
	.byte	2
	.short	758
	.short	5593
	.byte	0
	.byte	37
	.byte	60
	.byte	32
	.byte	2
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
	.type	sd15_tlc_rr, %object
	.size	sd15_tlc_rr, 329
sd15_tlc_rr:
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	-8
	.byte	0
	.byte	16
	.byte	8
	.byte	8
	.byte	0
	.byte	-8
	.byte	-8
	.byte	-8
	.byte	-16
	.byte	-8
	.byte	-8
	.byte	-8
	.byte	-8
	.byte	-24
	.byte	0
	.byte	0
	.byte	0
	.byte	-8
	.byte	-16
	.byte	-32
	.byte	0
	.byte	8
	.byte	-8
	.byte	8
	.byte	8
	.byte	0
	.byte	0
	.byte	-16
	.byte	-8
	.byte	-8
	.byte	-8
	.byte	0
	.byte	-16
	.byte	-24
	.byte	-16
	.byte	8
	.byte	8
	.byte	-8
	.byte	-16
	.byte	-16
	.byte	0
	.byte	8
	.byte	8
	.byte	8
	.byte	8
	.byte	-8
	.byte	-8
	.byte	-24
	.byte	0
	.byte	-16
	.byte	0
	.byte	-8
	.byte	-16
	.byte	-8
	.byte	-8
	.byte	0
	.byte	8
	.byte	0
	.byte	0
	.byte	-8
	.byte	0
	.byte	-24
	.byte	-8
	.byte	0
	.byte	0
	.byte	-8
	.byte	-24
	.byte	-8
	.byte	8
	.byte	-8
	.byte	0
	.byte	-8
	.byte	8
	.byte	-16
	.byte	-8
	.byte	-8
	.byte	-8
	.byte	8
	.byte	8
	.byte	0
	.byte	0
	.byte	-8
	.byte	-8
	.byte	8
	.byte	-8
	.byte	-8
	.byte	0
	.byte	0
	.byte	-8
	.byte	-16
	.byte	-16
	.byte	-8
	.byte	0
	.byte	0
	.byte	-8
	.byte	0
	.byte	-16
	.byte	8
	.byte	0
	.byte	8
	.byte	0
	.byte	-16
	.byte	-8
	.byte	-16
	.byte	16
	.byte	0
	.byte	16
	.byte	0
	.byte	-8
	.byte	8
	.byte	0
	.byte	-24
	.byte	0
	.byte	-16
	.byte	-8
	.byte	-16
	.byte	-16
	.byte	-16
	.byte	-16
	.byte	0
	.byte	8
	.byte	-8
	.byte	-24
	.byte	0
	.byte	8
	.byte	8
	.byte	16
	.byte	16
	.byte	0
	.byte	8
	.byte	-8
	.byte	8
	.byte	16
	.byte	-8
	.byte	24
	.byte	0
	.byte	8
	.byte	-4
	.byte	0
	.byte	16
	.byte	8
	.byte	24
	.byte	8
	.byte	0
	.byte	-4
	.byte	-8
	.byte	24
	.byte	16
	.byte	16
	.byte	0
	.byte	0
	.byte	0
	.byte	-16
	.byte	0
	.byte	0
	.byte	4
	.byte	0
	.byte	-4
	.byte	-4
	.byte	-4
	.byte	8
	.byte	8
	.byte	16
	.byte	0
	.byte	16
	.byte	-4
	.byte	16
	.byte	0
	.byte	16
	.byte	8
	.byte	0
	.byte	16
	.byte	-4
	.byte	16
	.byte	-8
	.byte	0
	.byte	0
	.byte	-8
	.byte	16
	.byte	-4
	.byte	16
	.byte	-16
	.byte	-8
	.byte	-8
	.byte	-8
	.byte	8
	.byte	-4
	.byte	8
	.byte	-24
	.byte	4
	.byte	-16
	.byte	0
	.byte	8
	.byte	0
	.byte	0
	.byte	-24
	.byte	8
	.byte	-16
	.byte	8
	.byte	0
	.byte	8
	.byte	-24
	.byte	-32
	.byte	16
	.byte	-24
	.byte	8
	.byte	-8
	.byte	8
	.byte	-24
	.byte	-32
	.byte	8
	.byte	0
	.byte	16
	.byte	0
	.byte	16
	.byte	0
	.byte	-32
	.byte	4
	.byte	0
	.byte	-8
	.byte	-16
	.byte	-8
	.byte	0
	.byte	-32
	.byte	4
	.byte	0
	.byte	8
	.byte	-24
	.byte	8
	.byte	0
	.byte	-32
	.byte	4
	.byte	0
	.byte	0
	.byte	-32
	.byte	-4
	.byte	0
	.byte	-24
	.byte	4
	.byte	0
	.byte	16
	.byte	-24
	.byte	16
	.byte	0
	.byte	-24
	.byte	-4
	.byte	0
	.byte	8
	.byte	-32
	.byte	8
	.byte	0
	.byte	0
	.byte	-4
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	-4
	.byte	0
	.byte	0
	.byte	0
	.byte	-4
	.byte	0
	.byte	0
	.byte	-4
	.byte	0
	.byte	-8
	.byte	0
	.byte	-8
	.byte	0
	.byte	0
	.byte	-4
	.byte	0
	.byte	-16
	.byte	0
	.byte	-16
	.byte	0
	.byte	0
	.byte	-16
	.byte	0
	.byte	-16
	.byte	0
	.byte	-16
	.byte	0
	.byte	0
	.byte	-8
	.byte	0
	.byte	-16
	.byte	0
	.byte	-16
	.byte	0
	.byte	0
	.byte	-16
	.byte	0
	.byte	-24
	.byte	0
	.byte	-24
	.byte	0
	.byte	0
	.byte	-24
	.byte	0
	.byte	-24
	.byte	0
	.byte	-24
	.byte	0
	.byte	0
	.byte	-24
	.byte	0
	.byte	-16
	.byte	0
	.byte	-16
	.byte	0
	.byte	0
	.byte	-24
	.byte	0
	.byte	-32
	.byte	0
	.byte	-32
	.byte	0
	.space	3
	.type	sd15_slc_rr, %object
	.size	sd15_slc_rr, 25
sd15_slc_rr:
	.byte	0
	.byte	8
	.byte	-8
	.byte	16
	.byte	-16
	.byte	24
	.byte	-24
	.byte	32
	.byte	-32
	.byte	32
	.byte	-40
	.byte	48
	.byte	-48
	.byte	56
	.byte	-56
	.byte	64
	.byte	-64
	.byte	72
	.byte	-72
	.byte	80
	.byte	-80
	.byte	88
	.byte	96
	.byte	104
	.byte	112
	.space	3
	.type	hy_f26_ref_value, %object
	.size	hy_f26_ref_value, 28
hy_f26_ref_value:
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
	.type	zftl_nand_flash_para_tbl, %object
	.size	zftl_nand_flash_para_tbl, 1408
zftl_nand_flash_para_tbl:
	.byte	6
	.byte	-104
	.byte	58
	.byte	-104
	.byte	-77
	.byte	118
	.byte	114
	.byte	1
	.byte	1
	.byte	32
	.short	768
	.byte	3
	.byte	2
	.short	758
	.short	5593
	.byte	0
	.byte	37
	.byte	60
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
	.byte	6
	.byte	-104
	.byte	60
	.byte	-104
	.byte	-77
	.byte	118
	.byte	114
	.byte	1
	.byte	1
	.byte	32
	.short	768
	.byte	3
	.byte	2
	.short	1478
	.short	5593
	.byte	0
	.byte	37
	.byte	60
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
	.byte	6
	.byte	-104
	.byte	58
	.byte	-104
	.byte	-93
	.byte	118
	.byte	81
	.byte	1
	.byte	1
	.byte	32
	.short	384
	.byte	3
	.byte	2
	.short	1446
	.short	1497
	.byte	0
	.byte	36
	.byte	60
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17857
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17857
	.byte	2
	.byte	35
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17857
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	34
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17857
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	33
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	68
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	68
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17881
	.byte	2
	.byte	67
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	1
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	17857
	.byte	2
	.byte	66
	.byte	40
	.byte	32
	.byte	2
	.byte	1
	.byte	1
	.byte	0
	.short	256
	.byte	1
	.byte	-94
	.byte	0
	.byte	0
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
	.short	9671
	.byte	5
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	4
	.byte	0
	.short	512
	.byte	1
	.byte	-38
	.byte	-33
	.byte	0
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
	.short	512
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.byte	5
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
	.short	256
	.byte	3
	.byte	0
	.byte	0
	.byte	0
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
	.short	512
	.byte	0
	.byte	0
	.byte	0
	.byte	0
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
	.short	455
	.byte	4
	.byte	18
	.byte	60
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	512
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	5
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
	.byte	4
	.short	1024
	.short	449
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	256
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.byte	5
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
	.short	512
	.byte	3
	.byte	0
	.byte	0
	.byte	0
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
	.short	9671
	.byte	5
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	4
	.byte	0
	.short	512
	.byte	2
	.byte	-38
	.byte	-33
	.byte	0
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
	.short	471
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	512
	.byte	3
	.byte	0
	.byte	0
	.byte	0
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
	.short	9671
	.byte	5
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	4
	.byte	0
	.short	512
	.byte	2
	.byte	-38
	.byte	-33
	.byte	0
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
	.short	9671
	.byte	5
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	4
	.byte	0
	.short	512
	.byte	2
	.byte	-38
	.byte	-33
	.byte	0
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
	.short	9671
	.byte	5
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	4
	.byte	0
	.short	512
	.byte	2
	.byte	-38
	.byte	-33
	.byte	0
	.byte	6
	.byte	-101
	.byte	73
	.byte	1
	.byte	0
	.byte	-101
	.byte	73
	.byte	9
	.byte	1
	.byte	32
	.short	256
	.byte	2
	.byte	1
	.short	2144
	.short	-23097
	.byte	8
	.byte	21
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	8
	.byte	0
	.short	256
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.byte	5
	.byte	44
	.byte	-124
	.byte	100
	.byte	60
	.byte	-87
	.byte	4
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
	.short	512
	.byte	0
	.byte	0
	.byte	0
	.byte	1
	.byte	5
	.byte	44
	.byte	-124
	.byte	88
	.byte	50
	.byte	-95
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	768
	.byte	3
	.byte	1
	.short	1440
	.short	3527
	.byte	0
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	1024
	.byte	3
	.byte	0
	.byte	0
	.byte	2
	.byte	6
	.byte	44
	.byte	-92
	.byte	8
	.byte	50
	.byte	-95
	.byte	0
	.byte	4
	.byte	1
	.byte	32
	.short	2304
	.byte	3
	.byte	1
	.short	1008
	.short	3521
	.byte	0
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	4096
	.byte	3
	.byte	0
	.byte	0
	.byte	4
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
	.short	455
	.byte	2
	.byte	6
	.byte	40
	.byte	32
	.byte	3
	.byte	1
	.byte	4
	.byte	0
	.short	256
	.byte	2
	.byte	-65
	.byte	-66
	.byte	0
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
	.short	256
	.byte	0
	.byte	0
	.byte	0
	.byte	0
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
	.short	256
	.byte	2
	.byte	-65
	.byte	-66
	.byte	0
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
	.short	256
	.byte	0
	.byte	0
	.byte	0
	.byte	0
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
	.short	256
	.byte	0
	.byte	0
	.byte	0
	.byte	0
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
	.short	256
	.byte	0
	.byte	0
	.byte	0
	.byte	0
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
	.byte	4
	.byte	0
	.short	512
	.byte	2
	.byte	-65
	.byte	-66
	.byte	0
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
	.short	512
	.byte	3
	.byte	0
	.byte	0
	.byte	0
	.byte	5
	.byte	-119
	.byte	-124
	.byte	100
	.byte	60
	.byte	-91
	.byte	0
	.byte	7
	.byte	1
	.byte	32
	.short	512
	.byte	2
	.byte	2
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
	.short	512
	.byte	3
	.byte	0
	.byte	0
	.byte	0
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
	.byte	4
	.short	1024
	.short	449
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	256
	.byte	0
	.byte	0
	.byte	0
	.byte	0
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
	.byte	4
	.short	1024
	.short	449
	.byte	3
	.byte	17
	.byte	40
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	256
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.byte	6
	.byte	-119
	.byte	-92
	.byte	8
	.byte	50
	.byte	-95
	.byte	0
	.byte	7
	.byte	1
	.byte	32
	.short	2304
	.byte	3
	.byte	1
	.short	1008
	.short	3521
	.byte	0
	.byte	19
	.byte	70
	.byte	32
	.byte	1
	.byte	0
	.byte	1
	.byte	0
	.short	4096
	.byte	3
	.byte	0
	.byte	0
	.byte	4
	.type	nand_opt_para, %object
	.size	nand_opt_para, 128
nand_opt_para:
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
	.type	tlc_b05a_prog_tbl, %object
	.size	tlc_b05a_prog_tbl, 1536
tlc_b05a_prog_tbl:
	.short	0
	.short	0
	.short	0
	.short	0
	.short	0
	.short	0
	.short	0
	.short	0
	.short	9
	.short	0
	.short	11
	.short	0
	.short	13
	.short	0
	.short	15
	.short	0
	.short	17
	.short	0
	.short	19
	.short	0
	.short	21
	.short	0
	.short	23
	.short	0
	.short	25
	.short	0
	.short	27
	.short	0
	.short	29
	.short	0
	.short	31
	.short	0
	.short	33
	.short	0
	.short	35
	.short	0
	.short	37
	.short	0
	.short	39
	.short	0
	.short	41
	.short	0
	.short	43
	.short	0
	.short	45
	.short	0
	.short	47
	.short	0
	.short	49
	.short	0
	.short	51
	.short	0
	.short	53
	.short	0
	.short	55
	.short	0
	.short	25
	.short	58
	.short	0
	.short	27
	.short	61
	.short	0
	.short	29
	.short	64
	.short	0
	.short	31
	.short	67
	.short	0
	.short	33
	.short	70
	.short	0
	.short	35
	.short	73
	.short	0
	.short	37
	.short	76
	.short	0
	.short	39
	.short	79
	.short	0
	.short	41
	.short	82
	.short	0
	.short	43
	.short	85
	.short	0
	.short	45
	.short	88
	.short	0
	.short	47
	.short	91
	.short	0
	.short	49
	.short	94
	.short	0
	.short	51
	.short	97
	.short	0
	.short	53
	.short	100
	.short	0
	.short	55
	.short	103
	.short	0
	.short	58
	.short	106
	.short	0
	.short	61
	.short	109
	.short	0
	.short	64
	.short	112
	.short	0
	.short	67
	.short	115
	.short	0
	.short	70
	.short	118
	.short	0
	.short	73
	.short	121
	.short	0
	.short	76
	.short	124
	.short	0
	.short	79
	.short	127
	.short	0
	.short	82
	.short	130
	.short	0
	.short	85
	.short	133
	.short	0
	.short	88
	.short	136
	.short	0
	.short	91
	.short	139
	.short	0
	.short	94
	.short	142
	.short	0
	.short	97
	.short	145
	.short	0
	.short	100
	.short	148
	.short	0
	.short	103
	.short	151
	.short	0
	.short	106
	.short	154
	.short	0
	.short	109
	.short	157
	.short	0
	.short	112
	.short	160
	.short	0
	.short	115
	.short	163
	.short	0
	.short	118
	.short	166
	.short	0
	.short	121
	.short	169
	.short	0
	.short	124
	.short	172
	.short	0
	.short	127
	.short	175
	.short	0
	.short	130
	.short	178
	.short	0
	.short	133
	.short	181
	.short	0
	.short	136
	.short	184
	.short	0
	.short	139
	.short	187
	.short	0
	.short	142
	.short	190
	.short	0
	.short	145
	.short	193
	.short	0
	.short	148
	.short	196
	.short	0
	.short	151
	.short	199
	.short	0
	.short	154
	.short	202
	.short	0
	.short	157
	.short	205
	.short	0
	.short	160
	.short	208
	.short	0
	.short	163
	.short	211
	.short	0
	.short	166
	.short	214
	.short	0
	.short	169
	.short	217
	.short	0
	.short	172
	.short	220
	.short	0
	.short	175
	.short	223
	.short	0
	.short	178
	.short	226
	.short	0
	.short	181
	.short	229
	.short	0
	.short	184
	.short	232
	.short	0
	.short	187
	.short	235
	.short	0
	.short	190
	.short	238
	.short	0
	.short	193
	.short	241
	.short	0
	.short	196
	.short	244
	.short	0
	.short	199
	.short	247
	.short	0
	.short	202
	.short	250
	.short	0
	.short	205
	.short	253
	.short	0
	.short	208
	.short	256
	.short	0
	.short	211
	.short	259
	.short	0
	.short	214
	.short	262
	.short	0
	.short	217
	.short	265
	.short	0
	.short	220
	.short	268
	.short	0
	.short	223
	.short	271
	.short	0
	.short	226
	.short	274
	.short	0
	.short	229
	.short	277
	.short	0
	.short	232
	.short	280
	.short	0
	.short	235
	.short	283
	.short	0
	.short	238
	.short	286
	.short	0
	.short	241
	.short	289
	.short	0
	.short	244
	.short	292
	.short	0
	.short	247
	.short	295
	.short	0
	.short	250
	.short	298
	.short	0
	.short	253
	.short	301
	.short	0
	.short	256
	.short	304
	.short	0
	.short	259
	.short	307
	.short	0
	.short	262
	.short	310
	.short	0
	.short	265
	.short	313
	.short	0
	.short	268
	.short	316
	.short	0
	.short	271
	.short	319
	.short	0
	.short	274
	.short	322
	.short	0
	.short	277
	.short	325
	.short	0
	.short	280
	.short	328
	.short	0
	.short	283
	.short	331
	.short	0
	.short	286
	.short	334
	.short	0
	.short	289
	.short	337
	.short	0
	.short	292
	.short	340
	.short	0
	.short	295
	.short	343
	.short	0
	.short	298
	.short	346
	.short	0
	.short	301
	.short	349
	.short	0
	.short	304
	.short	352
	.short	0
	.short	307
	.short	355
	.short	0
	.short	310
	.short	358
	.short	0
	.short	313
	.short	361
	.short	0
	.short	316
	.short	364
	.short	0
	.short	319
	.short	367
	.short	0
	.short	322
	.short	370
	.short	0
	.short	325
	.short	373
	.short	0
	.short	328
	.short	376
	.short	0
	.short	331
	.short	379
	.short	0
	.short	334
	.short	382
	.short	0
	.short	337
	.short	385
	.short	0
	.short	340
	.short	388
	.short	0
	.short	343
	.short	391
	.short	0
	.short	346
	.short	394
	.short	0
	.short	349
	.short	397
	.short	0
	.short	352
	.short	400
	.short	0
	.short	355
	.short	403
	.short	0
	.short	358
	.short	406
	.short	0
	.short	361
	.short	409
	.short	0
	.short	364
	.short	412
	.short	0
	.short	367
	.short	415
	.short	0
	.short	370
	.short	418
	.short	0
	.short	373
	.short	421
	.short	0
	.short	376
	.short	424
	.short	0
	.short	379
	.short	427
	.short	0
	.short	382
	.short	430
	.short	0
	.short	385
	.short	433
	.short	0
	.short	388
	.short	436
	.short	0
	.short	391
	.short	439
	.short	0
	.short	394
	.short	442
	.short	0
	.short	397
	.short	445
	.short	0
	.short	400
	.short	448
	.short	0
	.short	403
	.short	451
	.short	0
	.short	406
	.short	454
	.short	0
	.short	409
	.short	457
	.short	0
	.short	412
	.short	460
	.short	0
	.short	415
	.short	463
	.short	0
	.short	418
	.short	466
	.short	0
	.short	421
	.short	469
	.short	0
	.short	424
	.short	472
	.short	0
	.short	427
	.short	475
	.short	0
	.short	430
	.short	478
	.short	0
	.short	433
	.short	481
	.short	0
	.short	436
	.short	484
	.short	0
	.short	439
	.short	487
	.short	0
	.short	442
	.short	490
	.short	0
	.short	445
	.short	493
	.short	0
	.short	448
	.short	496
	.short	0
	.short	451
	.short	499
	.short	0
	.short	454
	.short	502
	.short	0
	.short	457
	.short	505
	.short	0
	.short	460
	.short	508
	.short	0
	.short	463
	.short	511
	.short	0
	.short	466
	.short	514
	.short	0
	.short	469
	.short	517
	.short	0
	.short	472
	.short	520
	.short	0
	.short	475
	.short	523
	.short	0
	.short	478
	.short	526
	.short	0
	.short	481
	.short	529
	.short	0
	.short	484
	.short	532
	.short	0
	.short	487
	.short	535
	.short	0
	.short	490
	.short	538
	.short	0
	.short	493
	.short	541
	.short	0
	.short	496
	.short	544
	.short	0
	.short	499
	.short	547
	.short	0
	.short	502
	.short	550
	.short	0
	.short	505
	.short	553
	.short	0
	.short	508
	.short	556
	.short	0
	.short	511
	.short	559
	.short	0
	.short	514
	.short	562
	.short	0
	.short	517
	.short	565
	.short	0
	.short	520
	.short	568
	.short	0
	.short	523
	.short	571
	.short	0
	.short	526
	.short	574
	.short	0
	.short	529
	.short	577
	.short	0
	.short	532
	.short	580
	.short	0
	.short	535
	.short	583
	.short	0
	.short	538
	.short	586
	.short	0
	.short	541
	.short	589
	.short	0
	.short	544
	.short	592
	.short	0
	.short	547
	.short	595
	.short	0
	.short	550
	.short	598
	.short	0
	.short	553
	.short	601
	.short	0
	.short	556
	.short	604
	.short	0
	.short	559
	.short	607
	.short	0
	.short	562
	.short	610
	.short	0
	.short	565
	.short	613
	.short	0
	.short	568
	.short	616
	.short	0
	.short	571
	.short	619
	.short	0
	.short	574
	.short	622
	.short	0
	.short	577
	.short	625
	.short	0
	.short	580
	.short	628
	.short	0
	.short	583
	.short	631
	.short	0
	.short	586
	.short	634
	.short	0
	.short	589
	.short	637
	.short	0
	.short	592
	.short	640
	.short	0
	.short	595
	.short	643
	.short	0
	.short	598
	.short	646
	.short	0
	.short	601
	.short	649
	.short	0
	.short	604
	.short	652
	.short	0
	.short	607
	.short	655
	.short	0
	.short	610
	.short	658
	.short	0
	.short	613
	.short	661
	.short	0
	.short	616
	.short	664
	.short	0
	.short	619
	.short	667
	.short	0
	.short	622
	.short	670
	.short	0
	.short	625
	.short	673
	.short	0
	.short	628
	.short	676
	.short	0
	.short	631
	.short	679
	.short	0
	.short	634
	.short	682
	.short	0
	.short	637
	.short	685
	.short	0
	.short	640
	.short	688
	.short	0
	.short	643
	.short	691
	.short	0
	.short	646
	.short	694
	.short	0
	.short	649
	.short	697
	.short	0
	.short	652
	.short	700
	.short	0
	.short	655
	.short	703
	.short	0
	.short	658
	.short	706
	.short	0
	.short	661
	.short	709
	.short	0
	.short	664
	.short	712
	.short	0
	.short	667
	.short	715
	.short	0
	.short	670
	.short	718
	.short	0
	.short	673
	.short	721
	.short	0
	.short	676
	.short	724
	.short	0
	.short	679
	.short	727
	.short	0
	.short	682
	.short	730
	.short	0
	.short	685
	.short	733
	.short	0
	.short	688
	.short	736
	.short	0
	.short	691
	.short	739
	.short	0
	.short	694
	.short	742
	.short	0
	.short	697
	.short	745
	.short	0
	.short	700
	.short	748
	.short	0
	.short	703
	.short	751
	.short	0
	.short	706
	.short	0
	.short	709
	.short	0
	.short	712
	.short	0
	.short	715
	.short	0
	.short	718
	.short	0
	.short	721
	.short	0
	.short	724
	.short	0
	.short	727
	.short	0
	.type	tlc_prog_order, %object
	.size	tlc_prog_order, 768
tlc_prog_order:
	.short	1
	.short	9
	.short	2
	.short	17
	.short	10
	.short	3
	.short	25
	.short	18
	.short	11
	.short	33
	.short	26
	.short	19
	.short	41
	.short	34
	.short	27
	.short	49
	.short	42
	.short	35
	.short	57
	.short	50
	.short	43
	.short	65
	.short	58
	.short	51
	.short	73
	.short	66
	.short	59
	.short	81
	.short	74
	.short	67
	.short	89
	.short	82
	.short	75
	.short	97
	.short	90
	.short	83
	.short	105
	.short	98
	.short	91
	.short	113
	.short	106
	.short	99
	.short	121
	.short	114
	.short	107
	.short	129
	.short	122
	.short	115
	.short	137
	.short	130
	.short	123
	.short	145
	.short	138
	.short	131
	.short	153
	.short	146
	.short	139
	.short	161
	.short	154
	.short	147
	.short	169
	.short	162
	.short	155
	.short	177
	.short	170
	.short	163
	.short	185
	.short	178
	.short	171
	.short	193
	.short	186
	.short	179
	.short	201
	.short	194
	.short	187
	.short	209
	.short	202
	.short	195
	.short	217
	.short	210
	.short	203
	.short	225
	.short	218
	.short	211
	.short	233
	.short	226
	.short	219
	.short	241
	.short	234
	.short	227
	.short	249
	.short	242
	.short	235
	.short	257
	.short	250
	.short	243
	.short	265
	.short	258
	.short	251
	.short	273
	.short	266
	.short	259
	.short	281
	.short	274
	.short	267
	.short	289
	.short	282
	.short	275
	.short	297
	.short	290
	.short	283
	.short	305
	.short	298
	.short	291
	.short	313
	.short	306
	.short	299
	.short	321
	.short	314
	.short	307
	.short	329
	.short	322
	.short	315
	.short	337
	.short	330
	.short	323
	.short	345
	.short	338
	.short	331
	.short	353
	.short	346
	.short	339
	.short	361
	.short	354
	.short	347
	.short	369
	.short	362
	.short	355
	.short	377
	.short	370
	.short	363
	.short	385
	.short	378
	.short	371
	.short	393
	.short	386
	.short	379
	.short	401
	.short	394
	.short	387
	.short	409
	.short	402
	.short	395
	.short	417
	.short	410
	.short	403
	.short	425
	.short	418
	.short	411
	.short	433
	.short	426
	.short	419
	.short	441
	.short	434
	.short	427
	.short	449
	.short	442
	.short	435
	.short	457
	.short	450
	.short	443
	.short	465
	.short	458
	.short	451
	.short	473
	.short	466
	.short	459
	.short	481
	.short	474
	.short	467
	.short	489
	.short	482
	.short	475
	.short	497
	.short	490
	.short	483
	.short	505
	.short	498
	.short	491
	.short	513
	.short	506
	.short	499
	.short	521
	.short	514
	.short	507
	.short	529
	.short	522
	.short	515
	.short	537
	.short	530
	.short	523
	.short	545
	.short	538
	.short	531
	.short	553
	.short	546
	.short	539
	.short	561
	.short	554
	.short	547
	.short	569
	.short	562
	.short	555
	.short	577
	.short	570
	.short	563
	.short	585
	.short	578
	.short	571
	.short	593
	.short	586
	.short	579
	.short	601
	.short	594
	.short	587
	.short	609
	.short	602
	.short	595
	.short	617
	.short	610
	.short	603
	.short	625
	.short	618
	.short	611
	.short	633
	.short	626
	.short	619
	.short	641
	.short	634
	.short	627
	.short	649
	.short	642
	.short	635
	.short	657
	.short	650
	.short	643
	.short	665
	.short	658
	.short	651
	.short	673
	.short	666
	.short	659
	.short	681
	.short	674
	.short	667
	.short	689
	.short	682
	.short	675
	.short	697
	.short	690
	.short	683
	.short	705
	.short	698
	.short	691
	.short	713
	.short	706
	.short	699
	.short	721
	.short	714
	.short	707
	.short	729
	.short	722
	.short	715
	.short	737
	.short	730
	.short	723
	.short	745
	.short	738
	.short	731
	.short	753
	.short	746
	.short	739
	.short	761
	.short	754
	.short	747
	.short	769
	.short	762
	.short	755
	.short	777
	.short	770
	.short	763
	.short	785
	.short	778
	.short	771
	.short	793
	.short	786
	.short	779
	.short	801
	.short	794
	.short	787
	.short	809
	.short	802
	.short	795
	.short	817
	.short	810
	.short	803
	.short	825
	.short	818
	.short	811
	.short	833
	.short	826
	.short	819
	.short	841
	.short	834
	.short	827
	.short	849
	.short	842
	.short	835
	.short	857
	.short	850
	.short	843
	.short	865
	.short	858
	.short	851
	.short	873
	.short	866
	.short	859
	.short	881
	.short	874
	.short	867
	.short	889
	.short	882
	.short	875
	.short	897
	.short	890
	.short	883
	.short	905
	.short	898
	.short	891
	.short	913
	.short	906
	.short	899
	.short	921
	.short	914
	.short	907
	.short	929
	.short	922
	.short	915
	.short	937
	.short	930
	.short	923
	.short	945
	.short	938
	.short	931
	.short	953
	.short	946
	.short	939
	.short	961
	.short	954
	.short	947
	.short	969
	.short	962
	.short	955
	.short	977
	.short	970
	.short	963
	.short	985
	.short	978
	.short	971
	.short	993
	.short	986
	.short	979
	.short	1001
	.short	994
	.short	987
	.short	1009
	.short	1002
	.short	995
	.short	1017
	.short	1010
	.short	1003
	.short	1018
	.short	1011
	.short	1019
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
	.align	6
.LANCHOR0 = . + 0
.LANCHOR3 = . + 8184
.LANCHOR4 = . + 24552
.LANCHOR5 = . + 40920
	.type	g_flash_slc_mode, %object
	.size	g_flash_slc_mode, 1
g_flash_slc_mode:
	.space	1
	.type	g_slc_mode_addr2, %object
	.size	g_slc_mode_addr2, 1
g_slc_mode_addr2:
	.space	1
	.type	g_block_align_addr, %object
	.size	g_block_align_addr, 2
g_block_align_addr:
	.space	2
	.type	g_lsb_page_tbl, %object
	.size	g_lsb_page_tbl, 512
g_lsb_page_tbl:
	.space	512
	.type	g_nandc_ver, %object
	.size	g_nandc_ver, 1
g_nandc_ver:
	.space	1
	.space	3
	.type	_c_user_data_density, %object
	.size	_c_user_data_density, 4
_c_user_data_density:
	.space	4
	.type	gp_sblk_list_tbl, %object
	.size	gp_sblk_list_tbl, 4
gp_sblk_list_tbl:
	.space	4
	.type	gp_nandc, %object
	.size	gp_nandc, 4
gp_nandc:
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
	.type	_c_ftl_blk_pre_plane, %object
	.size	_c_ftl_blk_pre_plane, 2
_c_ftl_blk_pre_plane:
	.space	2
	.space	2
	.type	gp_blk_info, %object
	.size	gp_blk_info, 4
gp_blk_info:
	.space	4
	.type	_c_slc_to_xlc_ec_ratio, %object
	.size	_c_slc_to_xlc_ec_ratio, 2
_c_slc_to_xlc_ec_ratio:
	.space	2
	.space	2
	.type	ftl_sblk_vpn, %object
	.size	ftl_sblk_vpn, 4
ftl_sblk_vpn:
	.space	4
	.type	gp_ftl_ext_info, %object
	.size	gp_ftl_ext_info, 4
gp_ftl_ext_info:
	.space	4
	.type	g_nandc_ecc_bits, %object
	.size	g_nandc_ecc_bits, 1
g_nandc_ecc_bits:
	.space	1
	.type	g_nand_max_die, %object
	.size	g_nand_max_die, 1
g_nand_max_die:
	.space	1
	.space	2
	.type	g_die_cs_idx, %object
	.size	g_die_cs_idx, 8
g_die_cs_idx:
	.space	8
	.type	g_msb_page_tbl, %object
	.size	g_msb_page_tbl, 1024
g_msb_page_tbl:
	.space	1024
	.type	gp_nand_para_info, %object
	.size	gp_nand_para_info, 4
gp_nand_para_info:
	.space	4
	.type	g_idb_ecc_bits, %object
	.size	g_idb_ecc_bits, 1
g_idb_ecc_bits:
	.space	1
	.type	g_idb_slc_mode_enable, %object
	.size	g_idb_slc_mode_enable, 1
g_idb_slc_mode_enable:
	.space	1
	.type	g_nand_opt_para, %object
	.size	g_nand_opt_para, 32
g_nand_opt_para:
	.space	32
	.type	g_flash_toggle_mode_en, %object
	.size	g_flash_toggle_mode_en, 1
g_flash_toggle_mode_en:
	.space	1
	.type	nandc_hw_seed, %object
	.size	nandc_hw_seed, 1
nandc_hw_seed:
	.space	1
	.type	g_nandc_v6_master_info, %object
	.size	g_nandc_v6_master_info, 28
g_nandc_v6_master_info:
	.space	28
	.type	nandc_randomizer_en, %object
	.size	nandc_randomizer_en, 1
nandc_randomizer_en:
	.space	1
	.space	1
	.type	fill_spare_size, %object
	.size	fill_spare_size, 2
fill_spare_size:
	.space	2
	.type	g_retryMode, %object
	.size	g_retryMode, 1
g_retryMode:
	.space	1
	.type	g_maxRegNum, %object
	.size	g_maxRegNum, 1
g_maxRegNum:
	.space	1
	.space	2
	.type	gp_flash_info, %object
	.size	gp_flash_info, 4
gp_flash_info:
	.space	4
	.type	_c_ftl_cs_bits, %object
	.size	_c_ftl_cs_bits, 1
_c_ftl_cs_bits:
	.space	1
	.space	3
	.type	g_flash_cur_mode, %object
	.size	g_flash_cur_mode, 4
g_flash_cur_mode:
	.space	4
	.type	g_slc_page_num, %object
	.size	g_slc_page_num, 2
g_slc_page_num:
	.space	2
	.space	2
	.type	g_die_addr, %object
	.size	g_die_addr, 32
g_die_addr:
	.space	32
	.type	IDByte, %object
	.size	IDByte, 32
IDByte:
	.space	32
	.type	g_totle_phy_block, %object
	.size	g_totle_phy_block, 2
g_totle_phy_block:
	.space	2
	.type	g_flash_micron_3d_tlc_flag, %object
	.size	g_flash_micron_3d_tlc_flag, 1
g_flash_micron_3d_tlc_flag:
	.space	1
	.type	g_flash_interface_mode, %object
	.size	g_flash_interface_mode, 1
g_flash_interface_mode:
	.space	1
	.type	g_flash_multi_page_prog_en, %object
	.size	g_flash_multi_page_prog_en, 1
g_flash_multi_page_prog_en:
	.space	1
	.space	3
	.type	g_buf, %object
	.size	g_buf, 1536
g_buf:
	.space	1536
	.type	_c_ftl_sec_per_page, %object
	.size	_c_ftl_sec_per_page, 1
_c_ftl_sec_per_page:
	.space	1
	.type	p_free_buf_head, %object
	.size	p_free_buf_head, 1
p_free_buf_head:
	.space	1
	.type	free_buf_count, %object
	.size	free_buf_count, 1
free_buf_count:
	.space	1
	.type	sblk_queue_head, %object
	.size	sblk_queue_head, 1
sblk_queue_head:
	.space	1
	.type	_c_totle_phy_density, %object
	.size	_c_totle_phy_density, 4
_c_totle_phy_density:
	.space	4
	.type	_c_totle_log_page, %object
	.size	_c_totle_log_page, 4
_c_totle_log_page:
	.space	4
	.type	free_slc_sblk, %object
	.size	free_slc_sblk, 2
free_slc_sblk:
	.space	2
	.type	free_xlc_sblk, %object
	.size	free_xlc_sblk, 2
free_xlc_sblk:
	.space	2
	.type	free_mix_sblk, %object
	.size	free_mix_sblk, 2
free_mix_sblk:
	.space	2
	.type	slc_data_sblk, %object
	.size	slc_data_sblk, 2
slc_data_sblk:
	.space	2
	.type	slc_cache_sblk, %object
	.size	slc_cache_sblk, 2
slc_cache_sblk:
	.space	2
	.type	xlc_data_sblk, %object
	.size	xlc_data_sblk, 2
xlc_data_sblk:
	.space	2
	.type	write_buf_count, %object
	.size	write_buf_count, 1
write_buf_count:
	.space	1
	.type	write_commit_count, %object
	.size	write_commit_count, 1
write_commit_count:
	.space	1
	.space	2
	.type	gp_ftl_info, %object
	.size	gp_ftl_info, 4
gp_ftl_info:
	.space	4
	.type	gc_free_slc_sblk_th, %object
	.size	gc_free_slc_sblk_th, 2
gc_free_slc_sblk_th:
	.space	2
	.type	gc_tlc_mode_tlc_vpn_th, %object
	.size	gc_tlc_mode_tlc_vpn_th, 2
gc_tlc_mode_tlc_vpn_th:
	.space	2
	.type	gc_tlc_mode_slc_vpn_th, %object
	.size	gc_tlc_mode_slc_vpn_th, 2
gc_tlc_mode_slc_vpn_th:
	.space	2
	.type	gc_slc_mode_vpn_th, %object
	.size	gc_slc_mode_vpn_th, 2
gc_slc_mode_vpn_th:
	.space	2
	.type	gc_slc_mode_tlc_vpn_th, %object
	.size	gc_slc_mode_tlc_vpn_th, 2
gc_slc_mode_tlc_vpn_th:
	.space	2
	.space	2
	.type	_gc_after_discard_en, %object
	.size	_gc_after_discard_en, 4
_gc_after_discard_en:
	.space	4
	.type	write_buf_head, %object
	.size	write_buf_head, 1
write_buf_head:
	.space	1
	.type	gc_mode, %object
	.size	gc_mode, 1
gc_mode:
	.space	1
	.type	gc_state, %object
	.size	gc_state, 1
gc_state:
	.space	1
	.space	1
	.type	gc_search_count, %object
	.size	gc_search_count, 4
gc_search_count:
	.space	4
	.type	gc_xlc_search_index, %object
	.size	gc_xlc_search_index, 2
gc_xlc_search_index:
	.space	2
	.space	2
	.type	g_gc_info, %object
	.size	g_gc_info, 596
g_gc_info:
	.space	596
	.type	_c_ftl_page_pre_slc_blk, %object
	.size	_c_ftl_page_pre_slc_blk, 2
_c_ftl_page_pre_slc_blk:
	.space	2
	.type	gc_slc_data_index, %object
	.size	gc_slc_data_index, 2
gc_slc_data_index:
	.space	2
	.type	gc_slc_cache_index, %object
	.size	gc_slc_cache_index, 2
gc_slc_cache_index:
	.space	2
	.type	gc_xlc_data_index, %object
	.size	gc_xlc_data_index, 2
gc_xlc_data_index:
	.space	2
	.type	_c_ftl_nand_planes_num, %object
	.size	_c_ftl_nand_planes_num, 1
_c_ftl_nand_planes_num:
	.space	1
	.space	1
	.type	_c_ftl_page_pre_blk, %object
	.size	_c_ftl_page_pre_blk, 2
_c_ftl_page_pre_blk:
	.space	2
	.type	gc_lpa_tbl, %object
	.size	gc_lpa_tbl, 4
gc_lpa_tbl:
	.space	4
	.type	gc_pre_ppa_tbl, %object
	.size	gc_pre_ppa_tbl, 4
gc_pre_ppa_tbl:
	.space	4
	.type	gc_page_buf_id, %object
	.size	gc_page_buf_id, 4
gc_page_buf_id:
	.space	4
	.type	gc_valid_page_ppa, %object
	.size	gc_valid_page_ppa, 4
gc_valid_page_ppa:
	.space	4
	.type	gc_des_ppa_tbl, %object
	.size	gc_des_ppa_tbl, 4
gc_des_ppa_tbl:
	.space	4
	.type	_min_slc_super_block, %object
	.size	_min_slc_super_block, 2
_min_slc_super_block:
	.space	2
	.type	ftl_sblk_vpn_update_id, %object
	.size	ftl_sblk_vpn_update_id, 2
ftl_sblk_vpn_update_id:
	.space	2
	.type	ftl_sblk_update_list, %object
	.size	ftl_sblk_update_list, 16
ftl_sblk_update_list:
	.space	16
	.type	_c_ftl_block_addr_log2, %object
	.size	_c_ftl_block_addr_log2, 2
_c_ftl_block_addr_log2:
	.space	2
	.type	_c_ftl_planes_per_die, %object
	.size	_c_ftl_planes_per_die, 1
_c_ftl_planes_per_die:
	.space	1
	.type	sblk_gc_write_completed_queue_head, %object
	.size	sblk_gc_write_completed_queue_head, 1
sblk_gc_write_completed_queue_head:
	.space	1
	.type	_c_ftl_nand_type, %object
	.size	_c_ftl_nand_type, 1
_c_ftl_nand_type:
	.space	1
	.type	g_flash_3d_mlc_flag, %object
	.size	g_flash_3d_mlc_flag, 1
g_flash_3d_mlc_flag:
	.space	1
	.type	g_one_pass_program, %object
	.size	g_one_pass_program, 1
g_one_pass_program:
	.space	1
	.type	g_flash_3d_tlc_flag, %object
	.size	g_flash_3d_tlc_flag, 1
g_flash_3d_tlc_flag:
	.space	1
	.type	gp_data_slc_cache_head, %object
	.size	gp_data_slc_cache_head, 4
gp_data_slc_cache_head:
	.space	4
	.type	gp_data_xlc_data_head, %object
	.size	gp_data_xlc_data_head, 4
gp_data_xlc_data_head:
	.space	4
	.type	gp_data_slc_data_head, %object
	.size	gp_data_slc_data_head, 4
gp_data_slc_data_head:
	.space	4
	.type	_max_xlc_super_block, %object
	.size	_max_xlc_super_block, 2
_max_xlc_super_block:
	.space	2
	.space	2
	.type	gp_free_slc_head, %object
	.size	gp_free_slc_head, 4
gp_free_slc_head:
	.space	4
	.type	gp_free_xlc_head, %object
	.size	gp_free_xlc_head, 4
gp_free_xlc_head:
	.space	4
	.type	gp_free_mix_head, %object
	.size	gp_free_mix_head, 4
gp_free_mix_head:
	.space	4
	.type	zftl_print_list_count, %object
	.size	zftl_print_list_count, 2
zftl_print_list_count:
	.space	2
	.space	2
	.type	pm_force_gc, %object
	.size	pm_force_gc, 4
pm_force_gc:
	.space	4
	.type	_c_swl_slc_gc_th, %object
	.size	_c_swl_slc_gc_th, 2
_c_swl_slc_gc_th:
	.space	2
	.type	_c_swl_xlc_gc_th, %object
	.size	_c_swl_xlc_gc_th, 2
_c_swl_xlc_gc_th:
	.space	2
	.type	_c_ftl_block_align_addr, %object
	.size	_c_ftl_block_align_addr, 2
_c_ftl_block_align_addr:
	.space	2
	.type	_c_ftl_nand_die_num, %object
	.size	_c_ftl_nand_die_num, 1
_c_ftl_nand_die_num:
	.space	1
	.space	1
	.type	lpa_hash, %object
	.size	lpa_hash, 512
lpa_hash:
	.space	512
	.type	ftl_sblk_lpa_tbl, %object
	.size	ftl_sblk_lpa_tbl, 4
ftl_sblk_lpa_tbl:
	.space	4
	.type	lpa_hash_index, %object
	.size	lpa_hash_index, 4
lpa_hash_index:
	.space	4
	.type	ftl_vpn_update_count, %object
	.size	ftl_vpn_update_count, 2
ftl_vpn_update_count:
	.space	2
	.type	ftl_sblk_update_list_offset, %object
	.size	ftl_sblk_update_list_offset, 2
ftl_sblk_update_list_offset:
	.space	2
	.type	_c_mix_max_xlc_ec_count, %object
	.size	_c_mix_max_xlc_ec_count, 2
_c_mix_max_xlc_ec_count:
	.space	2
	.type	_max_slc_super_block, %object
	.size	_max_slc_super_block, 2
_max_slc_super_block:
	.space	2
	.type	_c_mix_max_slc_ec_count, %object
	.size	_c_mix_max_slc_ec_count, 2
_c_mix_max_slc_ec_count:
	.space	2
	.type	sblk_write_completed_queue_head, %object
	.size	sblk_write_completed_queue_head, 1
sblk_write_completed_queue_head:
	.space	1
	.type	read_buf_head, %object
	.size	read_buf_head, 1
read_buf_head:
	.space	1
	.type	read_buf_count, %object
	.size	read_buf_count, 1
read_buf_count:
	.space	1
	.type	sblk_read_completed_queue_head, %object
	.size	sblk_read_completed_queue_head, 1
sblk_read_completed_queue_head:
	.space	1
	.type	_c_max_pm_sblk, %object
	.size	_c_max_pm_sblk, 2
_c_max_pm_sblk:
	.space	2
	.type	pm_gc_enable, %object
	.size	pm_gc_enable, 4
pm_gc_enable:
	.space	4
	.type	pm_ram_info, %object
	.size	pm_ram_info, 256
pm_ram_info:
	.space	256
	.type	pm_last_update_ram_id, %object
	.size	pm_last_update_ram_id, 1
pm_last_update_ram_id:
	.space	1
	.space	3
	.type	power_on_init_jiffies, %object
	.size	power_on_init_jiffies, 8
power_on_init_jiffies:
	.space	8
	.type	gp_ftl_api, %object
	.size	gp_ftl_api, 4
gp_ftl_api:
	.space	4
	.type	RK29_NANDC_REG_BASE, %object
	.size	RK29_NANDC_REG_BASE, 4
RK29_NANDC_REG_BASE:
	.space	4
	.type	g_flash_tmp_page_buffer, %object
	.size	g_flash_tmp_page_buffer, 4
g_flash_tmp_page_buffer:
	.space	4
	.type	g_nandc_tran_timeout, %object
	.size	g_nandc_tran_timeout, 1
g_nandc_tran_timeout:
	.space	1
	.space	3
	.type	g_flash_tmp_spare_buffer, %object
	.size	g_flash_tmp_spare_buffer, 4
g_flash_tmp_spare_buffer:
	.space	4
	.type	g_maxRetryCount, %object
	.size	g_maxRetryCount, 1
g_maxRetryCount:
	.space	1
	.space	3
	.type	flash_ddr_tuning_sdr_read_count, %object
	.size	flash_ddr_tuning_sdr_read_count, 4
flash_ddr_tuning_sdr_read_count:
	.space	4
	.type	flash_read_retry, %object
	.size	flash_read_retry, 4
flash_read_retry:
	.space	4
	.type	g_flash_page_buffer, %object
	.size	g_flash_page_buffer, 4
g_flash_page_buffer:
	.space	4
	.type	g_flash_spare_buffer, %object
	.size	g_flash_spare_buffer, 4
g_flash_spare_buffer:
	.space	4
	.type	g_flash_multi_page_read_en, %object
	.size	g_flash_multi_page_read_en, 1
g_flash_multi_page_read_en:
	.space	1
	.type	write_commit_head, %object
	.size	write_commit_head, 1
write_commit_head:
	.space	1
	.space	2
	.type	ftl_flush_jiffies, %object
	.size	ftl_flush_jiffies, 4
ftl_flush_jiffies:
	.space	4
	.space	56
	.type	ftl_info_spare, %object
	.size	ftl_info_spare, 256
ftl_info_spare:
	.space	256
	.type	g_ftl_info_blk, %object
	.size	g_ftl_info_blk, 4
g_ftl_info_blk:
	.space	4
	.space	60
	.type	ftl_info_data_buffer, %object
	.size	ftl_info_data_buffer, 16384
ftl_info_data_buffer:
	.space	16384
	.type	ftl_sys_info_first_write, %object
	.size	ftl_sys_info_first_write, 1
ftl_sys_info_first_write:
	.space	1
	.type	ftl_power_lost_flag, %object
	.size	ftl_power_lost_flag, 1
ftl_power_lost_flag:
	.space	1
	.type	ftl_ext_info_first_write, %object
	.size	ftl_ext_info_first_write, 1
ftl_ext_info_first_write:
	.space	1
	.space	61
	.type	ftl_ext_info_data_buffer, %object
	.size	ftl_ext_info_data_buffer, 16384
ftl_ext_info_data_buffer:
	.space	16384
	.type	ftl_tmp_spare, %object
	.size	ftl_tmp_spare, 256
ftl_tmp_spare:
	.space	256
	.type	g_pm_spare, %object
	.size	g_pm_spare, 4
g_pm_spare:
	.space	4
	.type	pm_first_write, %object
	.size	pm_first_write, 1
pm_first_write:
	.space	1
	.space	3
	.type	g_flash_sys_spare_buffer, %object
	.size	g_flash_sys_spare_buffer, 4
g_flash_sys_spare_buffer:
	.space	4
	.type	g_flash_blk_info, %object
	.size	g_flash_blk_info, 4
g_flash_blk_info:
	.space	4
	.type	g_flash_reversd_blks, %object
	.size	g_flash_reversd_blks, 1
g_flash_reversd_blks:
	.space	1
	.type	g_flash_micron_3d_tlc_b05a, %object
	.size	g_flash_micron_3d_tlc_b05a, 1
g_flash_micron_3d_tlc_b05a:
	.space	1
	.type	g_flash_micron_3d_tlc_b16a, %object
	.size	g_flash_micron_3d_tlc_b16a, 1
g_flash_micron_3d_tlc_b16a:
	.space	1
	.space	1
	.type	_c_ftl_byte_pre_page, %object
	.size	_c_ftl_byte_pre_page, 2
_c_ftl_byte_pre_page:
	.space	2
	.type	pm_last_load_ram_id, %object
	.size	pm_last_load_ram_id, 1
pm_last_load_ram_id:
	.space	1
	.space	1
	.type	read_ahead_lpa, %object
	.size	read_ahead_lpa, 4
read_ahead_lpa:
	.space	4
	.type	_c_totle_data_density, %object
	.size	_c_totle_data_density, 4
_c_totle_data_density:
	.space	4
	.type	_c_ftl_pm_page_num, %object
	.size	_c_ftl_pm_page_num, 2
_c_ftl_pm_page_num:
	.space	2
	.type	rk_zftl_enable, %object
	.size	rk_zftl_enable, 1
rk_zftl_enable:
	.space	1
	.space	1
	.type	gLoaderBootInfo, %object
	.size	gLoaderBootInfo, 4
gLoaderBootInfo:
	.space	4
	.type	RK29_NANDC1_REG_BASE, %object
	.size	RK29_NANDC1_REG_BASE, 4
RK29_NANDC1_REG_BASE:
	.space	4
	.type	_last_read_time, %object
	.size	_last_read_time, 4
_last_read_time:
	.space	4
	.type	_last_write_time, %object
	.size	_last_write_time, 4
_last_write_time:
	.space	4
	.type	discard_sector_count, %object
	.size	discard_sector_count, 4
discard_sector_count:
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
	.type	ftl_low_format_cur_blk, %object
	.size	ftl_low_format_cur_blk, 2
ftl_low_format_cur_blk:
	.space	2
	.space	22
	.type	ftl_tmp_buffer, %object
	.size	ftl_tmp_buffer, 16384
ftl_tmp_buffer:
	.space	16384
	.type	p_read_ahead_ext_buf, %object
	.size	p_read_ahead_ext_buf, 4
p_read_ahead_ext_buf:
	.space	4
	.type	_c_ftl_nand_blks_per_die, %object
	.size	_c_ftl_nand_blks_per_die, 2
_c_ftl_nand_blks_per_die:
	.space	2
	.space	2
	.type	nandc_ecc_sts, %object
	.size	nandc_ecc_sts, 16
nandc_ecc_sts:
	.space	16
	.type	g_slc_mode_enable, %object
	.size	g_slc_mode_enable, 1
g_slc_mode_enable:
	.space	1
