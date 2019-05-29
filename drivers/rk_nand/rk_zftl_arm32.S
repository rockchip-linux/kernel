/*
 * Copyright (c) 2016-2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * date: 2019-06-03
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
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L106
	add	ip, r3, #564
	ldrh	ip, [ip]
	cmp	ip, r1
	bxls	lr
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, #6
	ldrh	ip, [r2]
	mov	r5, r3
	mul	r6, r6, r1
	add	ip, ip, #1
	strh	ip, [r2]	@ movhi
	.pad #20
	sub	sp, sp, #20
	ldr	fp, [r3, #524]
	mvn	r3, #0
	add	r4, fp, r6
	strh	r3, [r4, #2]	@ movhi
	strh	r3, [fp, r6]	@ movhi
	ldr	r3, [r0]
	cmp	r3, #0
	beq	.L105
	ldr	r2, [r5, #576]
	add	r9, r5, #572
	ldr	r8, [r5, #568]
	ldrh	r9, [r9]
	str	r2, [sp]
	mov	r2, r1, asl #1
	ldr	ip, [sp]
	ldrh	r7, [r4, #4]
	str	r9, [sp, #4]
	ldrh	lr, [ip, r2]
	muls	lr, r7, lr
	ldr	r2, [r8, r1, asl #2]
	mov	ip, r1, asl #2
	ldrneh	r9, [sp, #4]
	ubfx	r2, r2, #11, #8
	ldrh	ip, [r8, ip]
	smulbbne	r2, r2, r9
	ubfx	ip, ip, #0, #11
	addne	r2, ip, r2
	uxtahne	lr, lr, r2
	ldr	r2, [r5, #524]
	cmp	r7, #0
	mov	r5, #0
	rsb	ip, r2, r3
	str	r2, [sp, #8]
	mvneq	lr, #0
	ldr	r2, .L106+4
	mov	ip, ip, asr #1
	mul	r2, r2, ip
	ldr	ip, .L106+8
	ldrh	ip, [ip]
	uxth	r2, r2
	str	ip, [sp, #12]
.L95:
	add	r5, r5, #1
	ldr	ip, [sp, #12]
	uxth	r5, r5
	cmp	r5, ip
	movls	ip, #0
	movhi	ip, #1
	cmp	r1, r2
	orreq	ip, ip, #1
	cmp	ip, #0
	bne	.L84
	ldr	r7, [sp]
	mov	ip, r2, asl #1
	ldrh	r9, [r3, #4]
	ldrh	r10, [r7, ip]
	mov	ip, r2, asl #2
	ldr	r7, [r8, r2, asl #2]
	muls	r10, r9, r10
	ldrh	ip, [r8, ip]
	ubfx	r7, r7, #11, #8
	ubfx	ip, ip, #0, #11
	beq	.L91
	cmp	r9, #0
	beq	.L99
	ldrh	r9, [sp, #4]
	smulbb	r7, r7, r9
	add	r7, ip, r7
	uxtah	r10, r10, r7
	b	.L92
.L91:
	cmp	r9, #0
	bne	.L97
.L99:
	mvn	r10, #0
.L92:
	cmp	r10, lr
	bhi	.L93
.L97:
	ldrh	ip, [r3]
	movw	r7, #65535
	cmp	ip, r7
	streqh	r2, [r4, #2]	@ movhi
	streqh	r1, [r3]	@ movhi
	beq	.L84
.L94:
	ldr	r2, [sp, #8]
	mov	r3, #6
	mla	r3, r3, ip, r2
	mov	r2, ip
	b	.L95
.L93:
	ldrh	ip, [r3, #2]
	strh	ip, [r4, #2]	@ movhi
	strh	r2, [fp, r6]	@ movhi
	ldr	r2, [r0]
	cmp	r3, r2
	bne	.L96
	strh	r1, [r3, #2]	@ movhi
.L105:
	str	r4, [r0]
	b	.L84
.L96:
	ldr	r2, .L106
	ldrh	ip, [r3, #2]
	ldr	r0, [r2, #524]
	mov	r2, #6
	mul	r2, r2, ip
	strh	r1, [r0, r2]	@ movhi
	strh	r1, [r3, #2]	@ movhi
.L84:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L107:
	.align	2
.L106:
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
	ldr	r1, .L119
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
	bne	.L112
	ldrh	r2, [r10, r8]
	cmp	r2, r3
	bne	.L112
	ldr	r1, .L119+4
	mov	r2, #255
	ldr	r0, .L119+8
	bl	printk
	bl	dump_stack
.L112:
	ldrh	r2, [r9, #2]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L113
	ldrh	r3, [r10, r8]
	cmp	r3, r2
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L113:
	ldr	r0, [r5, #576]
	mov	r3, r4, asl #1
	ldrh	r1, [r0, r3]
	ldrh	r3, [r9, #4]
	cmp	r3, #0
	mulne	r1, r3, r1
	mov	r3, #6
	mul	r2, r3, r2
	ldr	r3, .L119+12
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
.L120:
	.align	2
.L119:
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
	ldr	r2, .L126
	mov	ip, #1
	ldr	r3, .L126+4
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
.L122:
	uxtb	r1, r3
	cmp	r1, r0
	bcs	.L125
	ldrb	lr, [r3, r2]	@ zero_extendqisi2
	add	r3, r3, #1
	ldrb	r1, [r4, #10]	@ zero_extendqisi2
	orr	r1, r1, ip, asl lr
	strb	r1, [r4, #10]
	b	.L122
.L125:
	ldmfd	sp!, {r4, pc}
.L127:
	.align	2
.L126:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.fnend
	.size	zftl_read_flash_info, .-zftl_read_flash_info
	.section	.text.unlikely,"ax",%progbits
	.align	2
	.type	rk_simple_strtoull.constprop.33, %function
rk_simple_strtoull.constprop.33:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r3, #48
	movne	r2, r0
	movne	lr, #10
	bne	.L129
	ldrb	r3, [r0, #1]	@ zero_extendqisi2
	add	r2, r0, #1
	orr	r3, r3, #32
	cmp	r3, #120
	bne	.L140
	ldrb	r3, [r0, #2]	@ zero_extendqisi2
	bic	ip, r3, #32
	sub	ip, ip, #65
	cmp	ip, #25
	bls	.L130
	sub	r3, r3, #48
	cmp	r3, #9
	bls	.L130
.L140:
	mov	lr, #8
.L129:
	mov	r0, #0
.L131:
	mov	r4, r2
	ldrb	ip, [r2], #1	@ zero_extendqisi2
	bic	r3, ip, #32
	sub	r3, r3, #65
	cmp	r3, #25
	sub	r3, ip, #48
	bls	.L132
	cmp	r3, #9
	bls	.L132
.L136:
	cmp	r1, #0
	bne	.L133
	ldmfd	sp!, {r4, pc}
.L133:
	str	r4, [r1]
	ldmfd	sp!, {r4, pc}
.L130:
	add	r2, r0, #2
	mov	lr, #16
	b	.L129
.L132:
	cmp	r3, #9
	orrhi	ip, ip, #32
	subhi	r3, ip, #87
	cmp	r3, lr
	bcs	.L136
	mla	r0, lr, r0, r3
	b	.L131
	.fnend
	.size	rk_simple_strtoull.constprop.33, .-rk_simple_strtoull.constprop.33
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
	bne	.L144
	ldr	r3, .L173
.L145:
	mov	r2, r0, asl #1
	strh	r0, [r2, r3]	@ movhi
	add	r0, r0, #1
	cmp	r0, #256
	bne	.L145
.L149:
	mov	r1, #255
	ldr	r0, .L173+4
	mov	r2, #1024
	bl	memset
	ldr	r1, .L173
	mov	r3, #0
	add	ip, r1, #592
	b	.L146
.L144:
	cmp	r0, #1
	bne	.L147
	ldr	ip, .L173
	mov	r3, #0
.L148:
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
	bne	.L148
	b	.L149
.L147:
	cmp	r0, #2
	bne	.L150
	ldr	r1, .L173
	mov	r3, #0
.L151:
	uxth	r2, r3
	mov	r0, r3, asl #1
	cmp	r2, #1
	add	r3, r3, #1
	mov	ip, r2, asl #1
	subhi	r2, ip, #1
	cmp	r3, #256
	strh	r2, [r0, r1]	@ movhi
	bne	.L151
	b	.L149
.L150:
	cmp	r0, #3
	bne	.L152
	ldr	ip, .L173
	mov	r3, #0
.L153:
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
	bne	.L153
	b	.L149
.L152:
	cmp	r0, #4
	mov	r3, #0
	bne	.L154
	ldr	r2, .L173+8
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
.L155:
	tst	r3, #1
	movne	r1, #7
	moveq	r1, #6
	rsb	r1, r1, r3, asl #1
	add	r3, r3, #1
	strh	r1, [r2, #2]!	@ movhi
	uxth	r3, r3
	cmp	r3, #256
	bne	.L155
	b	.L149
.L154:
	cmp	r0, #5
	bne	.L156
	ldr	r2, .L173
.L157:
	mov	r1, r3, asl #1
	strh	r3, [r1, r2]	@ movhi
	add	r3, r3, #1
	cmp	r3, #16
	bne	.L157
	ldr	r2, .L173+12
.L158:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, #496
	bne	.L158
	b	.L149
.L156:
	cmp	r0, #8
	bne	.L159
	ldr	r2, .L173
.L160:
	strh	r3, [r3, r2]	@ movhi
	add	r3, r3, #2
	cmp	r3, #512
	bne	.L160
	b	.L149
.L159:
	cmp	r0, #9
	bne	.L149
	ldr	r2, .L173+8
	movw	r1, #509
	strh	r3, [r2, #4]	@ movhi
	mov	r3, #1
	strh	r3, [r2, #6]	@ movhi
	mov	r3, #2
	strh	r3, [r2, #8]!	@ movhi
	mov	r3, #3
.L161:
	strh	r3, [r2, #2]!	@ movhi
	add	r3, r3, #2
	uxth	r3, r3
	cmp	r3, r1
	bne	.L161
	b	.L149
.L146:
	ldrh	r2, [r3, r1]
	add	r3, r3, #2
	cmp	r3, #512
	mov	r0, r2, asl #1
	strh	r2, [ip, r0]	@ movhi
	bne	.L146
	ldmfd	sp!, {r3, pc}
.L174:
	.align	2
.L173:
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
	ldr	r4, .L298
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L176
	ldr	r0, .L298+4
	ldr	r1, .L298+8
	bl	printk
.L176:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L177
	ldr	r3, .L298+12
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
	ldr	r0, .L298+16
	bl	printk
.L177:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L178
	ldr	r3, .L298+12
	ldr	r0, .L298+20
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #8]	@ zero_extendqisi2
	bl	printk
.L178:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L179
	ldr	r3, .L298+12
	ldr	r0, .L298+24
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	bl	printk
.L179:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L180
	ldr	r3, .L298+12
	ldr	r0, .L298+28
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #10]
	bl	printk
.L180:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L181
	ldr	r3, .L298+12
	ldr	r0, .L298+32
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #12]	@ zero_extendqisi2
	bl	printk
.L181:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L182
	ldr	r3, .L298+12
	ldr	r0, .L298+36
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	bl	printk
.L182:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L183
	ldr	r3, .L298+12
	ldr	r0, .L298+40
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #14]
	bl	printk
.L183:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L184
	ldr	r3, .L298+12
	ldr	r0, .L298+44
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #23]	@ zero_extendqisi2
	bl	printk
.L184:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L185
	ldr	r3, .L298+12
	ldr	r0, .L298+48
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #18]	@ zero_extendqisi2
	bl	printk
.L185:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L186
	ldr	r3, .L298+12
	ldr	r0, .L298+52
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #19]	@ zero_extendqisi2
	bl	printk
.L186:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L187
	ldr	r3, .L298+12
	ldr	r0, .L298+56
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #20]	@ zero_extendqisi2
	bl	printk
.L187:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L188
	ldr	r3, .L298+12
	ldr	r0, .L298+60
	ldrb	r1, [r3, #1624]	@ zero_extendqisi2
	bl	printk
.L188:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L189
	ldr	r3, .L298+12
	ldr	r0, .L298+64
	ldrb	r1, [r3]	@ zero_extendqisi2
	bl	printk
.L189:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L190
	ldr	r3, .L298+12
	ldr	r0, .L298+68
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #22]	@ zero_extendqisi2
	bl	printk
.L190:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L191
	ldr	r3, .L298+12
	ldr	r0, .L298+72
	ldrb	r1, [r3, #585]	@ zero_extendqisi2
	bl	printk
.L191:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L192
	ldr	r3, .L298+12
	ldr	r0, .L298+76
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	and	r1, r1, #1
	bl	printk
.L192:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L193
	ldr	r3, .L298+12
	ldr	r0, .L298+80
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #1, #1
	bl	printk
.L193:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L194
	ldr	r3, .L298+12
	ldr	r0, .L298+84
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #2, #1
	bl	printk
.L194:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L195
	ldr	r3, .L298+12
	ldr	r0, .L298+88
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #3, #1
	bl	printk
.L195:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L196
	ldr	r3, .L298+12
	ldr	r0, .L298+92
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #4, #1
	bl	printk
.L196:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L197
	ldr	r3, .L298+12
	ldr	r0, .L298+96
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #5, #1
	bl	printk
.L197:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L198
	ldr	r3, .L298+12
	ldr	r0, .L298+100
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #6, #1
	bl	printk
.L198:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L199
	ldr	r3, .L298+12
	ldr	r0, .L298+104
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #7, #1
	bl	printk
.L199:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L200
	ldr	r3, .L298+12
	ldr	r0, .L298+108
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #8, #1
	bl	printk
.L200:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L201
	ldr	r3, .L298+12
	ldr	r0, .L298+112
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #9, #1
	bl	printk
.L201:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L202
	ldr	r3, .L298+12
	ldr	r0, .L298+116
	ldr	r3, [r3, #1620]
	ldrh	r1, [r3, #16]
	ubfx	r1, r1, #10, #1
	bl	printk
.L202:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L203
	ldr	r3, .L298+12
	ldr	r0, .L298+120
	ldrb	r1, [r3]	@ zero_extendqisi2
	ldrb	r2, [r3, #1625]	@ zero_extendqisi2
	bl	printk
.L203:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L204
	ldr	r3, .L298+12
	ldr	r0, .L298+124
	ldrb	r1, [r3, #1636]	@ zero_extendqisi2
	ldrb	r2, [r3, #1637]	@ zero_extendqisi2
	bl	printk
.L204:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L205
	ldr	r3, .L298+12
	ldr	r0, .L298+128
	ldrb	r1, [r3, #1634]	@ zero_extendqisi2
	ldrb	r2, [r3, #1635]	@ zero_extendqisi2
	bl	printk
.L205:
	ldr	r3, [r4]
	tst	r3, #4096
	beq	.L175
	ldr	r3, .L298+12
	ldr	r0, .L298+132
	ldrb	r1, [r3, #1658]	@ zero_extendqisi2
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, lr}
	b	printk
.L175:
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, pc}
.L299:
	.align	2
.L298:
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
	ldr	r5, .L320
	mov	r3, #0
	str	r3, [sp, #12]
	mov	r7, r0
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L301
	ldr	r0, .L320+4
	mov	r2, r7
	ldr	r1, .L320+8
	bl	printk
.L301:
	ldr	r4, .L320+12
	mov	r3, #6
	ldr	r2, [r7, #352]
	ldr	r6, .L320+12
	strb	r3, [r4, #516]
	ldr	r3, .L320+16
	str	r7, [r4, #528]
	cmp	r2, r3
	ldr	r2, [r7, #128]
	moveq	r3, #8
	streqb	r3, [r4, #516]
	ldr	r3, .L320+20
	cmp	r2, r3
	ldr	r2, .L320+24
	moveq	r3, #9
	streqb	r3, [r4, #516]
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L304
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
	beq	.L306
	ldr	r1, [r0]
	ldr	r2, [r0, #8]
	ldr	r3, [r0, #80]
	ldr	ip, [r0, #84]
	ldr	r0, [r0, #88]
	b	.L319
.L304:
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
	beq	.L306
	ldr	r0, [r6, #528]
	ldr	r1, [r0]
	ldr	r2, [r0, #344]
	ldr	r3, [r0, #304]
	ldr	ip, [r0, #308]
	ldr	r0, [r0, #312]
.L319:
	str	r0, [sp, #4]
	str	ip, [sp]
	ldr	r0, .L320+28
	bl	printk
.L306:
	movw	r2, #1690
	mov	r3, #1
	strb	r3, [r4, #1688]
	mov	r3, #0
	strh	r3, [r4, r2]	@ movhi
	strb	r3, [r4, #584]
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L300
	ldr	r3, .L320+12
	ldr	r0, .L320+32
	ldrb	r1, [r3, #516]	@ zero_extendqisi2
	bl	printk
.L300:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L321:
	.align	2
.L320:
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
.L323:
	ldr	r3, [sp, #4]
	cmp	r3, #0
	sub	r2, r3, #1
	str	r2, [sp, #4]
	bne	.L323
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
	ldr	r9, .L338
	mov	r5, r0
	ldr	r7, .L338+4
	mov	r6, r1
	add	r10, r9, #80
	mov	r4, #0
.L329:
	ldrb	r3, [r7, #1693]	@ zero_extendqisi2
	cmp	r4, r3
	bcs	.L337
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
	beq	.L336
	cmp	r3, #35
	addne	r3, r9, r6
	ldrnesb	r3, [r3, #224]
	bne	.L335
	ldr	r3, .L338+8
	add	r2, r4, r8
	add	r3, r3, r2
.L336:
	ldrsb	r3, [r3, #5]
.L335:
	str	r3, [r5]
	add	r4, r4, #1
	b	.L329
.L337:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L339:
	.align	2
.L338:
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
	ldr	r3, .L347
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
.L343:
	cmp	r9, r8
	beq	.L346
	ldrb	r3, [r9, #1]!	@ zero_extendqisi2
	mov	r0, #120
	str	r3, [r6, #2052]
	bl	timer_delay_ns
	ldrsb	r3, [r4, #1]!
	str	r3, [r6, #2048]
	b	.L343
.L346:
	add	r5, r7, r5
	mov	r3, #22
	str	r3, [r5, #2056]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L348:
	.align	2
.L347:
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
	ldr	r5, .L355
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
.L356:
	.align	2
.L355:
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
	ldr	r3, .L360
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
.L361:
	.align	2
.L360:
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
	ldr	r3, .L366
	ldrb	r2, [r3, #516]	@ zero_extendqisi2
	ldr	r3, [r3, #528]
	cmp	r2, #9
	ldreq	r0, [r3, #80]
	ldrne	r0, [r3, #304]
	ubfx	r0, r0, #8, #8
	bx	lr
.L367:
	.align	2
.L366:
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
	ldr	r1, .L376
	ands	ip, r0, #6
	ldr	r3, [r1, #528]
	ldr	r2, [r3]
	bfieq	r2, ip, #13, #1
	beq	.L372
	orr	r2, r2, #24576
	ldrb	r1, [r1, #516]	@ zero_extendqisi2
	bfc	r2, #15, #1
	tst	r0, #4
	orr	r2, r2, #196608
	movw	r0, #8321
	orrne	r2, r2, #32768
	cmp	r1, #9
	ldr	r1, .L376+4
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
.L372:
	str	r2, [r3]
	bx	lr
.L377:
	.align	2
.L376:
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
	ldr	r3, .L379
	mov	r2, #1
	mov	r0, r2, asl r0
	ldr	r1, [r3, #528]
	ldr	r3, [r1]
	bfi	r3, r0, #0, #8
	str	r3, [r1]
	bx	lr
.L380:
	.align	2
.L379:
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
	ldr	r3, .L382
	ldr	r2, [r3, #528]
	ldr	r3, [r2]
	bfc	r3, #0, #8
	bfc	r3, #17, #1
	str	r3, [r2]
	bx	lr
.L383:
	.align	2
.L382:
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
	ldr	r4, .L389
	mov	r6, r1
	mov	r7, r2
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L385
	ldr	r1, .L389+4
	movw	r2, #678
	ldr	r0, .L389+8
	bl	printk
	bl	dump_stack
.L385:
	add	r5, r4, r5
	ldr	r4, [r4, #528]
	mov	r10, r6, lsr #8
	mov	r9, r6, lsr #16
	ldrb	r8, [r5, #588]	@ zero_extendqisi2
	mov	fp, #120
	mov	r0, r8
	add	r4, r4, r8, asl #8
	bl	nandc_cs
.L386:
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
	bne	.L386
	mov	r0, r8
	bl	nandc_de_cs
	mov	r0, r5
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L390:
	.align	2
.L389:
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
	tst	r0, #50331648
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r7, r1
	ubfx	r4, r0, #0, #21
	ubfx	r8, r0, #21, #3
	bne	.L392
	ldr	r6, .L403
	ldrb	r3, [r6]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L393
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L392
.L393:
	ldrh	r5, [r6, #2]
	mov	r0, r4
	mov	r1, r5
	bl	__aeabi_uidiv
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L403
	mul	r0, r5, r0
	rsb	r4, r0, r4
	mov	r4, r4, asl #1
	addeq	r4, r3, r4
	ldreqh	r4, [r4, #4]
	add	r4, r4, r0
.L392:
	mov	r0, r8
	mov	r1, r4
	mov	r2, r7
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	b	flash_wait_device_ready_raw
.L404:
	.align	2
.L403:
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
	ldr	r4, .L411
	ldr	r5, .L411+4
.L407:
	ldr	r3, [r5, #528]
	ldr	r3, [r3]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #512
	bne	.L408
	mov	r0, #10
	bl	timer_delay_ns
	subs	r4, r4, #1
	bne	.L407
	mvn	r0, #0
	b	.L406
.L408:
	mov	r0, #0
.L406:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, pc}
.L412:
	.align	2
.L411:
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
	ldr	r2, .L434
	ldrb	r1, [r2]	@ zero_extendqisi2
	cmp	r1, #0
	bxeq	lr
	cmp	r1, #1
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
	ldr	r3, [r2, #528]
	bne	.L416
	ldr	r2, .L434+4
	ldrb	r2, [r2, #33]	@ zero_extendqisi2
	cmp	r2, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	b	.L433
.L416:
	cmp	r1, #2
	bne	.L418
	add	r2, r2, r0
	ldrb	r1, [r2, #1700]	@ zero_extendqisi2
	cmp	r1, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	mov	r1, #0
	strb	r1, [r2, #1700]
	ldr	r2, .L434+4
	ldrb	r2, [r2, #33]	@ zero_extendqisi2
	cmp	r2, r1
	ldmeqfd	sp!, {r3, r4, r5, pc}
.L433:
	add	r0, r3, r0, asl #8
	str	r2, [r0, #2056]
	ldmfd	sp!, {r3, r4, r5, pc}
.L418:
	cmp	r1, #3
	ldmnefd	sp!, {r3, r4, r5, pc}
	add	r2, r2, r0
	ldrb	r1, [r2, #1700]	@ zero_extendqisi2
	cmp	r1, #0
	ldmeqfd	sp!, {r3, r4, r5, pc}
	add	r4, r3, r0, asl #8
	mov	r3, #239
	mov	r0, #50
	mov	r5, #0
	str	r3, [r4, #2056]
	mov	r3, #145
	str	r3, [r4, #2052]
	strb	r5, [r2, #1700]
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
.L435:
	.align	2
.L434:
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
	ldr	r5, .L467
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L437
	ldr	r0, .L467+4
	ldr	r1, .L467+8
	bl	printk
.L437:
	ldr	r4, .L467+12
	mov	r6, #0
	ldrh	r3, [r5, #30]
	ldrb	r1, [r5, #16]	@ zero_extendqisi2
	ldrh	r0, [r5, #14]
	strh	r3, [r4, #2]	@ movhi
	strb	r6, [r4, #585]
	bl	__aeabi_idiv
	movw	r3, #1704
	mov	r1, #8
	ldr	r7, .L467+16
	add	r9, r7, #40
	strh	r0, [r4, r3]	@ movhi
	add	r0, r4, #588
	bl	__memzero
	ldr	r0, .L467+20
	mov	r1, #32
	bl	__memzero
.L442:
	ldrb	r2, [r5, #4]	@ zero_extendqisi2
	add	r1, r9, r6, asl #3
	ldr	r8, .L467+24
	mov	r3, #2
	ldr	r0, .L467+28
	strb	r3, [r6, r7]
	mov	r3, #0
.L438:
	cmp	r3, r2
	bcs	.L465
	add	ip, r0, r3
	ldrb	lr, [ip, #1]	@ zero_extendqisi2
	ldrb	ip, [r1, r3]	@ zero_extendqisi2
	cmp	lr, ip
	bne	.L439
	add	r3, r3, #1
	b	.L438
.L465:
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	mov	r1, #0
	uxtb	r0, r6
	add	r2, r4, r3, asl #2
	str	r1, [r2, #1708]
	add	r2, r3, #1
	add	r3, r4, r3
	strb	r2, [r4, #585]
	strb	r0, [r3, #588]
	bl	zftl_flash_enter_slc_mode
.L439:
	add	r6, r6, #1
	cmp	r6, #4
	bne	.L442
	ldrb	r2, [r5, #12]	@ zero_extendqisi2
	ldr	r3, .L467
	cmp	r2, #2
	beq	.L443
.L450:
	ldrb	r3, [r5, #17]	@ zero_extendqisi2
	ldrb	r2, [r4, #585]	@ zero_extendqisi2
	smulbb	r2, r2, r3
	ldrh	r3, [r5, #18]
	smulbb	r3, r2, r3
	movw	r2, #1772
	strh	r3, [r4, r2]	@ movhi
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L443:
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
.L448:
	add	r9, r8, r2, asl #3
	mov	r0, #0
.L444:
	cmp	r0, r7
	bcs	.L466
	add	r10, r3, r0
	ldrb	fp, [r10, #1]	@ zero_extendqisi2
	ldrb	r10, [r9, r0]	@ zero_extendqisi2
	cmp	fp, r10
	bne	.L445
	add	r0, r0, #1
	b	.L444
.L466:
	ldr	lr, [sp, #4]
	mov	r0, r1
	cmp	r6, #0
	add	r1, r4, r1, asl #2
	moveq	lr, ip
	str	lr, [r1, #1708]
	add	r1, r0, #1
	add	r0, r4, r0
	mov	lr, #1
	uxtb	r1, r1
	strb	r2, [r0, #588]
.L445:
	add	r2, r2, #1
	cmp	r2, #4
	bne	.L448
	cmp	lr, #0
	strneb	r1, [r4, #585]
	b	.L450
.L468:
	.align	2
.L467:
	.word	.LANCHOR2
	.word	.LC4
	.word	.LANCHOR1+260
	.word	.LANCHOR0
	.word	.LANCHOR0+1700
	.word	.LANCHOR0+1708
	.word	.LANCHOR0+1740
	.word	.LANCHOR2+4
	.fnend
	.size	flash_die_info_init, .-flash_die_info_init
	.align	2
	.global	zftl_flash_exit_slc_mode
	.type	zftl_flash_exit_slc_mode, %function
zftl_flash_exit_slc_mode:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L494
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #0
	bxeq	lr
	cmp	r2, #1
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	ldr	r7, [r3, #528]
	ldreq	r3, .L494+4
	ldreqb	r3, [r3, #34]	@ zero_extendqisi2
	beq	.L493
	cmp	r2, #2
	bne	.L474
	add	r3, r3, r0
	ldrb	r2, [r3, #1700]	@ zero_extendqisi2
	cmp	r2, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	ldr	r1, .L494+4
	ldrb	r2, [r1, #16]	@ zero_extendqisi2
	cmp	r2, #2
	movne	r2, #4
	strb	r2, [r3, #1700]
	ldrb	r3, [r1, #34]	@ zero_extendqisi2
.L493:
	cmp	r3, #0
	addne	r0, r7, r0, asl #8
	strne	r3, [r0, #2056]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L474:
	cmp	r2, #3
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	add	r6, r3, r0
	ldrb	r3, [r6, #1700]	@ zero_extendqisi2
	cmp	r3, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, pc}
	ldr	r8, .L494+4
	mov	r4, r0, asl #8
	add	r5, r7, r4
	mov	r0, #50
	add	r4, r7, r4
	ldrb	r3, [r8, #16]	@ zero_extendqisi2
	cmp	r3, #2
	movne	r3, #4
	strb	r3, [r6, #1700]
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
	ldrneb	r2, [r6, #1700]	@ zero_extendqisi2
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
.L495:
	.align	2
.L494:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	ubfx	r8, r1, #21, #3
	ldr	r4, .L505
	mov	r7, r0
	ubfx	r5, r1, #0, #21
	ubfx	r6, r1, #24, #2
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	cmp	r3, r8
	bhi	.L497
	ldr	r1, .L505+4
	movw	r2, #846
	ldr	r0, .L505+8
	bl	printk
	bl	dump_stack
.L497:
	add	r8, r4, r8
	ldr	r4, [r4, #528]
	ldr	r9, .L505
	ldrb	r8, [r8, #588]	@ zero_extendqisi2
	mov	r0, r8
	mov	r10, r8, asl #8
	bl	nandc_cs
	cmp	r6, #0
	bne	.L498
	mov	r0, r5
	bl	slc_phy_page_address_calc
	ldrb	r3, [r9]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r5, r0
	beq	.L499
	mov	r0, r8
	bl	zftl_flash_enter_slc_mode
	b	.L499
.L498:
	ldr	r3, [r9, #1620]
	ldrb	r3, [r3, #12]	@ zero_extendqisi2
	cmp	r3, #3
	addeq	r3, r4, r10
	streq	r6, [r3, #2056]
	beq	.L499
	mov	r0, r8
	bl	zftl_flash_exit_slc_mode
.L499:
	add	r3, r4, r10
	mov	r0, #0
	uxtb	r2, r5
	str	r0, [r3, #2056]
	str	r0, [r3, #2052]
	str	r0, [r3, #2052]
	str	r2, [r3, #2052]
	mov	r2, r5, lsr #8
	mov	r5, r5, lsr #16
	str	r2, [r3, #2052]
	str	r5, [r3, #2052]
	str	r7, [r3, #2056]
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
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	ubfx	r7, r0, #21, #3
	ldr	r10, .L557
	ubfx	r4, r0, #0, #21
	ubfx	r5, r1, #0, #21
	ubfx	r6, r0, #24, #2
	ldrb	r3, [r10, #585]	@ zero_extendqisi2
	cmp	r3, r7
	bhi	.L539
	ldr	r1, .L557+4
	movw	r2, #941
	ldr	r0, .L557+8
	bl	printk
	bl	dump_stack
.L539:
	add	r7, r10, r7
	ldr	fp, .L557
	ldrb	r8, [r7, #588]	@ zero_extendqisi2
	ldr	r7, [r10, #528]
	mov	r0, r8
	mov	r9, r8, asl #8
	bl	nandc_cs
	cmp	r6, #0
	bne	.L540
	mov	r0, r4
	bl	slc_phy_page_address_calc
	mov	r4, r0
	mov	r0, r5
	bl	slc_phy_page_address_calc
	ldrb	r3, [fp]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r5, r0
	beq	.L541
	mov	r0, r8
	bl	zftl_flash_enter_slc_mode
	b	.L541
.L540:
	ldr	r3, [fp, #1620]
	ldrb	r3, [r3, #12]	@ zero_extendqisi2
	cmp	r3, #3
	addeq	r3, r7, r9
	streq	r6, [r3, #2056]
	beq	.L541
	mov	r0, r8
	bl	zftl_flash_exit_slc_mode
.L541:
	ldrb	r3, [r10, #1642]	@ zero_extendqisi2
	uxtb	r1, r4
	ldrb	r0, [r10, #1634]	@ zero_extendqisi2
	mov	r2, r4, lsr #8
	cmp	r3, #1
	mov	r3, r4, lsr #16
	add	r4, r7, r9
	ldr	fp, .L557
	str	r0, [r4, #2056]
	bne	.L543
	mov	r10, #0
	str	r10, [r4, #2052]
	str	r10, [r4, #2052]
	add	r9, r7, r9
	str	r1, [r4, #2052]
	str	r2, [r4, #2052]
	str	r3, [r4, #2052]
	ldrb	r3, [fp, #1635]	@ zero_extendqisi2
	str	r3, [r4, #2056]
	bl	nandc_wait_flash_ready
	ldr	r3, [fp, #1620]
	ldrb	r2, [r3, #12]	@ zero_extendqisi2
	subs	r3, r6, r10
	movne	r3, #1
	cmp	r2, #3
	movne	r3, #0
	cmp	r3, r10
	uxtb	r3, r5
	strne	r6, [r4, #2056]
	str	r10, [r9, #2056]
	str	r10, [r4, #2052]
	str	r10, [r4, #2052]
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #8
	mov	r5, r5, lsr #16
	str	r3, [r4, #2052]
	str	r5, [r4, #2052]
	mov	r3, #48
	str	r3, [r9, #2056]
	b	.L545
.L543:
	str	r1, [r4, #2052]
	add	r7, r7, r9
	str	r2, [r4, #2052]
	str	r3, [r4, #2052]
	ldr	r3, [fp, #1620]
	ldrb	r2, [r3, #12]	@ zero_extendqisi2
	adds	r3, r6, #0
	movne	r3, #1
	cmp	r2, #3
	movne	r3, #0
	cmp	r3, #0
	ldrb	r3, [r10, #1635]	@ zero_extendqisi2
	strne	r6, [r4, #2056]
	str	r3, [r7, #2056]
	uxtb	r3, r5
	str	r3, [r4, #2052]
	mov	r3, r5, lsr #8
	mov	r5, r5, lsr #16
	str	r3, [r4, #2052]
	str	r5, [r4, #2052]
	mov	r3, #48
	str	r3, [r7, #2056]
.L545:
	mov	r0, r8
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
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
	mov	r10, #0
	ldr	r9, .L592+4
	mov	r7, r6
.L571:
	ldrb	r4, [r9, r10, asl #3]	@ zero_extendqisi2
	ldr	r5, [r6, #528]
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
	ldrb	r1, [r7, #1774]	@ zero_extendqisi2
	bne	.L562
	tst	r1, #1
	beq	.L561
	ldr	r3, .L592+8
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L563
	ldr	r0, .L592+12
	bl	printk
.L563:
	mov	r2, r10, asl #8
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
	ldr	r3, .L592+8
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L567
	ldr	r0, .L592+16
	bl	printk
.L567:
	mov	r2, r10, asl #8
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
	add	r10, r10, #1
	cmp	r10, #4
	bne	.L571
	bl	nandc_wait_flash_ready
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L593:
	.align	2
.L592:
	.word	.LANCHOR0
	.word	.LANCHOR0+1740
	.word	.LANCHOR2
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
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	ldr	r3, .L643
	mov	r1, #1
	ldrb	r2, [r3, #516]	@ zero_extendqisi2
	strb	r0, [r3, #584]
	cmp	r2, #9
	ldr	r2, [r3, #528]
	bne	.L630
	cmp	r0, #70
	str	r1, [r2, #16]
	moveq	r3, #0
	beq	.L631
	cmp	r0, #60
	moveq	r3, #3
	beq	.L631
	cmp	r0, #40
	moveq	r3, #2
	movne	r3, r1
.L631:
	mov	r3, r3, asl #25
	orr	r3, r3, #1
	str	r3, [r2, #32]
	bx	lr
.L630:
	str	r1, [r2, #8]
	mov	r1, #16
	cmp	r0, r1
	mov	r3, #0
	bfi	r3, r1, #8, #8
	bfc	r3, #18, #1
	bne	.L633
.L636:
	bfc	r3, #4, #1
	b	.L634
.L633:
	cmp	r0, #24
	orreq	r3, r3, #16
	beq	.L634
	cmp	r0, #40
	orr	r3, r3, #262144
	orr	r3, r3, #16
	beq	.L636
.L634:
	orr	r3, r3, #1
	str	r3, [r2, #12]
	bx	lr
.L644:
	.align	2
.L643:
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
	ldr	r4, .L656
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	ldr	r2, [r4, #532]
	cmp	r3, #9
	ldr	r3, [r4, #528]
	str	r2, [r3]
	ldr	r2, [r4, #536]
	ldr	r3, [r4, #528]
	str	r2, [r3, #4]
	ldr	r2, [r4, #540]
	bne	.L646
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
	b	.L647
.L646:
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
.L647:
	ldr	r6, .L656+4
	mov	r5, #0
	mov	r8, #2
	sub	r7, r6, #40
.L649:
	ldrb	r3, [r6, r5, asl #3]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bhi	.L648
	uxtb	r9, r5
	mov	r0, r9
	bl	flash_reset
	strb	r8, [r5, r7]
	mov	r0, r9
	bl	zftl_flash_enter_slc_mode
.L648:
	add	r5, r5, #1
	cmp	r5, #4
	bne	.L649
	ldrb	r3, [r4, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L650
	mov	r0, #1
	bl	nandc_set_if_mode
	mov	r0, r5
	bl	flash_set_interface_mode
	mov	r0, r5
	bl	nandc_set_if_mode
	ldr	r3, .L656
	ldrb	r0, [r3, #549]	@ zero_extendqisi2
	bl	nandc_set_ddr_para
.L650:
	ldr	r3, .L656+8
	ldrb	r0, [r3, #24]	@ zero_extendqisi2
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	b	nandc_bch_sel
.L657:
	.align	2
.L656:
	.word	.LANCHOR0
	.word	.LANCHOR0+1740
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
	ldr	r3, .L661
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
.L662:
	.align	2
.L661:
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
	ldr	r4, .L670
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
	bne	.L666
	ldmfd	sp!, {r4, lr}
	b	wait_for_nand_flash_ready
.L666:
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
.L671:
	.align	2
.L670:
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
	mov	r7, r3
	ldr	r10, .L687
	mov	r4, r0
	mov	fp, r1
	mov	r8, r2
	ldrb	r3, [r10, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L673
	ldr	r1, .L687+4
	movw	r2, #589
	ldr	r0, .L687+8
	bl	printk
	bl	dump_stack
.L673:
	ldr	r3, .L687+12
	add	r4, r10, r4
	ldr	r6, [r10, #528]
	ldrb	r5, [r4, #588]	@ zero_extendqisi2
	ldr	r3, [r3]
	add	r9, r5, #8
	tst	r3, #16
	add	r9, r6, r9, asl #8
	beq	.L674
	ldr	r0, .L687+16
	mov	r1, r5
	mov	r2, r8
	mov	r3, r7
	bl	printk
.L674:
	bl	nandc_wait_flash_ready
	mov	r0, r5
	bl	nandc_cs
	cmp	fp, #0
	mov	r0, r5
	bne	.L675
	bl	zftl_flash_enter_slc_mode
	b	.L676
.L675:
	bl	zftl_flash_exit_slc_mode
.L676:
	mov	fp, r5, asl #8
	mov	r3, #96
	add	r4, r6, fp
	str	r3, [r4, #2056]
	uxtb	r3, r8
	str	r3, [r4, #2052]
	mov	r3, r8, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r8, lsr #16
	str	r3, [r4, #2052]
	ldrb	r3, [r10, #1775]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r10, #0
	bne	.L677
	mov	r3, #208
	str	r3, [r4, #2056]
	bl	nandc_wait_flash_ready
	mov	r0, r9
	bl	flash_read_status
	and	r10, r0, #5
.L677:
	add	r6, r6, fp
	mov	r3, #96
	str	r3, [r6, #2056]
	uxtb	r3, r7
	str	r3, [r4, #2052]
	mov	r3, r7, lsr #8
	mov	r7, r7, lsr #16
	str	r3, [r4, #2052]
	mov	r3, #208
	str	r7, [r4, #2052]
	str	r3, [r6, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	mov	r0, r9
	bl	flash_read_status
	mov	r4, r0
	mov	r0, r5
	bl	nandc_de_cs
	and	r3, r4, #5
	orrs	r10, r3, r10
	beq	.L678
	ldr	r0, .L687+20
	mov	r1, r8
	mov	r2, r4
	bl	printk
.L678:
	mov	r0, r10
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L688:
	.align	2
.L687:
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
	mov	r8, r0
	ldr	r6, .L702
	mov	fp, r1
	mov	r7, r2
	ubfx	r9, r2, #0, #21
	ldrb	r3, [r6, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L690
	ldr	r1, .L702+4
	movw	r2, #634
	ldr	r0, .L702+8
	bl	printk
	bl	dump_stack
.L690:
	add	r3, r6, r8
	ldr	r4, [r6, #528]
	ldrb	r5, [r3, #588]	@ zero_extendqisi2
	ldr	r3, .L702+12
	add	r10, r5, #8
	ldr	r3, [r3]
	add	r10, r4, r10, asl #8
	tst	r3, #16
	beq	.L691
	ldr	r0, .L702+16
	mov	r1, r5
	mov	r2, r7
	mov	r3, fp
	bl	printk
.L691:
	bl	nandc_wait_flash_ready
	mov	r0, r5
	bl	nandc_cs
	cmp	fp, #0
	mov	r0, r5
	bne	.L692
	bl	zftl_flash_enter_slc_mode
	b	.L693
.L692:
	bl	zftl_flash_exit_slc_mode
.L693:
	add	r3, r4, r5, asl #8
	mov	r2, #96
	str	r2, [r3, #2056]
	uxtb	r2, r7
	str	r2, [r3, #2052]
	mov	r2, r9, lsr #8
	mov	r9, r9, lsr #16
	str	r2, [r3, #2052]
	str	r9, [r3, #2052]
	mov	r2, #208
	str	r2, [r3, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	mov	r0, r10
	bl	flash_read_status
	mov	r4, r0
	mov	r0, r5
	bl	nandc_de_cs
	ands	r4, r4, #5
	beq	.L694
	ldrh	r1, [r6, #2]
	mov	r0, r7
	bl	__aeabi_uidiv
	mov	r1, r8
	mov	r3, r4
	mov	r2, r0
	ldr	r0, .L702+20
	bl	printk
.L694:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L703:
	.align	2
.L702:
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
	ldr	r3, .L713
	ldr	r7, .L713+4
	ldrb	r4, [r3, #17]	@ zero_extendqisi2
	ldrh	r3, [r3, #18]
	mov	r8, r7
	smulbb	r4, r4, r3
	uxth	r4, r4
.L706:
	ldrb	r2, [r7, #585]	@ zero_extendqisi2
	uxth	r3, r6
	cmp	r2, r3
	bls	.L711
	add	r3, r7, r3
	mov	r5, #0
	ldrb	r9, [r3, #588]	@ zero_extendqisi2
.L707:
	uxth	r3, r5
	cmp	r3, r4
	bcs	.L712
	ldrh	r1, [r8, #2]
	mov	r0, r9
	mul	r1, r1, r5
	bl	flash_erase_block
	add	r5, r5, #1
	b	.L707
.L712:
	add	r6, r6, #1
	b	.L706
.L711:
	ldr	r0, .L713+8
	mov	r1, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L714:
	.align	2
.L713:
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
	ldr	r3, .L718
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
.L719:
	.align	2
.L718:
	.word	.LANCHOR0
	.fnend
	.size	rk_nandc_flash_xfer_completed, .-rk_nandc_flash_xfer_completed
	.align	2
	.global	nandc_xfer_start
	.type	nandc_xfer_start, %function
nandc_xfer_start:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	mov	r6, r0
	ldr	r5, .L738
	mov	fp, r1
	mov	r10, r3
	mov	r8, r2
	add	r9, fp, #1
	ldr	r0, [r5, #528]
	bl	rk_nandc_xfer_irq_flag_init
	ldrb	r3, [r5, #516]	@ zero_extendqisi2
	ldr	r4, [r5, #528]
	cmp	r3, #9
	bne	.L721
	ldr	r3, [r4, #292]
	orr	r3, r3, #1
	str	r3, [r4, #292]
	ldr	r3, [r4, #288]
	orr	r3, r3, #1
	str	r3, [r4, #288]
	mov	r4, #0
	bfi	r4, r6, #1, #1
	orr	r4, r4, #8
	mov	r3, #1
	bfi	r4, r3, #5, #2
	mov	r9, r9, asr r3
	orr	r4, r4, #536870912
	ldrb	r3, [r5, #1659]	@ zero_extendqisi2
	orr	r4, r4, #1024
	bfc	r4, #4, #1
	cmp	r3, #0
	bfi	r4, r9, #22, #6
	orr	r4, r4, #128
	beq	.L723
	ldrb	r3, [r5, #1688]	@ zero_extendqisi2
	cmp	r3, #0
	orrne	r4, r4, #512
	b	.L723
.L721:
	ldr	r3, [r4, #368]
	cmp	r6, #0
	orr	r3, r3, #1
	str	r3, [r4, #368]
	ldr	r3, [r4, #364]
	orr	r3, r3, #1
	str	r3, [r4, #364]
	ldr	r7, [r4, #12]
	mov	r4, #0
	bfi	r4, r6, #1, #1
	mov	r3, #16
	orr	r4, r4, #8
	bfi	r7, r3, #8, #8
	mov	r3, #1
	bfi	r4, r3, #5, #2
	orr	r4, r4, #536870912
	bfc	r7, #3, #1
	orr	r4, r4, #1024
	mov	r0, r9, asr r3
	bfc	r4, #4, #1
	bfc	r7, #5, #3
	bfi	r4, r0, #22, #6
	beq	.L737
	ldrb	r3, [r5, #584]	@ zero_extendqisi2
	mov	r1, #0
	mov	r2, r1
	cmp	r3, #25
	mov	r3, fp, lsr #1
	str	r3, [sp, #4]
	mov	r3, r10
	movcc	r9, #64
	movcs	r9, #128
	mov	r10, r5
	b	.L728
.L723:
	and	r9, r9, #63
	mov	r2, r6
	mov	r0, r8
	str	r8, [r5, #1664]
	mov	r1, r9, asl #10
	str	r10, [r5, #1668]
	bl	rknand_dma_map_single
	mov	r2, r6
	mov	r1, r9, asl #2
	clz	r6, r6
	mov	r6, r6, lsr #5
	str	r0, [r5, #1672]
	ldr	r0, [r5, #1668]
	bl	rknand_dma_map_single
	ldr	r7, [r5, #528]
	mov	r3, #1
	str	r3, [r5, #1680]
	ldr	r3, [r5, #1672]
	mov	r2, #16
	str	r3, [r7, #52]
	str	r0, [r7, #56]
	ldr	r3, [r7, #48]
	str	r0, [r5, #1676]
	bfi	r3, r2, #9, #5
	mov	r2, #2
	orr	r3, r3, #448
	bfi	r3, r2, #3, #3
	movw	r2, #1690
	orr	r3, r3, #4
	ldrh	r2, [r5, r2]
	bfi	r3, r6, #1, #1
	orr	r3, r3, #1
	bfi	r3, r2, #16, #11
	str	r3, [r7, #48]
	str	r4, [r7, #16]
	orr	r4, r4, #4
	str	r4, [r7, #16]
	b	.L720
.L728:
	ldr	ip, [sp, #4]
	add	r3, r3, #4
	add	lr, r1, r9
	cmp	r2, ip
	bge	.L730
	ldrh	fp, [r3, #-2]
	mov	r1, r1, lsr #2
	ldrh	ip, [r3, #-4]
	add	r2, r2, #1
	orr	ip, ip, fp, asl #16
	ldr	fp, [r10, #1660]
	str	ip, [fp, r1, asl #2]
	mov	r1, lr
	b	.L728
.L737:
	ldr	r2, [r5, #1660]
	str	r3, [r2]
.L730:
	ldr	r3, [r5, #1660]
	and	r9, r0, #63
	mov	r2, r6
	mov	r0, r8
	mov	r1, r9, asl #10
	str	r8, [r5, #1664]
	str	r3, [r5, #1668]
	bl	rknand_dma_map_single
	mov	r2, r6
	mov	r1, r9, asl #7
	clz	r6, r6
	mov	r6, r6, lsr #5
	str	r0, [r5, #1672]
	ldr	r0, [r5, #1668]
	bl	rknand_dma_map_single
	ldr	r2, [r5, #528]
	mov	r3, #1
	str	r3, [r5, #1680]
	ldr	r3, [r5, #1672]
	mov	r1, #16
	str	r3, [r2, #20]
	mov	r3, #0
	bfi	r3, r1, #9, #5
	mov	r1, #2
	orr	r3, r3, #448
	bfi	r3, r1, #3, #3
	orr	r3, r3, #4
	bfi	r3, r6, #1, #1
	orr	r3, r3, #1
	str	r0, [r5, #1676]
	str	r0, [r2, #24]
	str	r3, [r2, #16]
	str	r7, [r2, #12]
	str	r4, [r2, #8]
	orr	r4, r4, #4
	str	r4, [r2, #8]
.L720:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L739:
	.align	2
.L738:
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
	ldr	r2, .L747
	and	r0, r0, #127
	ldr	r3, .L747+4
	mov	r0, r0, asl #1
	ldrb	r1, [r2, #1688]	@ zero_extendqisi2
	ldrh	r3, [r3, r0]
	cmp	r1, #0
	ldr	r1, .L747
	ldrb	r2, [r2, #516]	@ zero_extendqisi2
	orrne	r3, r3, #-1073741824
	cmp	r2, #9
	ldr	r2, [r1, #528]
	streq	r3, [r2, #520]
	strne	r3, [r2, #336]
	bx	lr
.L748:
	.align	2
.L747:
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
	ldr	r4, .L766
	ldrb	r3, [r4]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L750
	ldrb	r0, [r4, #1625]	@ zero_extendqisi2
	cmp	r0, #0
	bne	.L750
	bl	zftl_flash_exit_slc_mode
.L750:
	mov	r0, #0
	ldr	r5, .L766
	bl	hynix_reconfig_rr_para
	ldrb	r3, [r4, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L751
	ldrb	r3, [r5, #1774]	@ zero_extendqisi2
	tst	r3, #1
	beq	.L751
	mov	r0, #1
	bl	flash_set_interface_mode
	mov	r0, #1
	bl	nandc_set_if_mode
	mov	r3, #0
	strb	r3, [r5, #1658]
.L751:
	ldrb	r3, [r4, #1688]	@ zero_extendqisi2
	ldr	r5, .L766
	cmp	r3, #0
	beq	.L752
	mov	r0, #0
	strb	r0, [r5, #1688]
	bl	nandc_set_seed
	mov	r3, #1
	strb	r3, [r5, #1688]
.L752:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L767:
	.align	2
.L766:
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
	ldr	r3, .L769
	strb	r0, [r3, #1688]
	bx	lr
.L770:
	.align	2
.L769:
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
	ldr	r3, .L772
	add	r0, r0, #8
	ldr	r3, [r3, #528]
	add	r0, r3, r0, asl #8
	bx	lr
.L773:
	.align	2
.L772:
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
	ldr	r4, .L778
	mov	r7, r5
	sub	r8, r4, #1776
.L775:
	ldrb	r0, [r8, #3312]	@ zero_extendqisi2
	uxtb	r3, r5
	add	r2, r3, #1
	strb	r3, [r4, #1]
	strb	r2, [r4]
	add	r5, r5, #1
	mov	r0, r0, asl #9
	strb	r7, [r4, #2]
	str	r7, [r4, #8]
	bl	ftl_malloc
	ldr	r6, .L778+4
	add	r4, r4, #48
	str	r0, [r4, #-44]
	mov	r0, #64
	bl	ftl_malloc
	cmp	r5, #32
	str	r0, [r4, #-36]
	bne	.L775
	mvn	r3, #0
	strb	r5, [r6, #3314]
	strb	r3, [r6, #3264]
	mov	r3, #0
	strb	r3, [r6, #3313]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L779:
	.align	2
.L778:
	.word	.LANCHOR0+1776
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
	ldr	r2, .L783
	mov	r3, #0
	mov	r1, r3
.L781:
	uxtb	r0, r3
	add	r3, r3, #1
	cmp	r3, #32
	add	ip, r0, #1
	strb	r0, [r2, #1]
	add	r2, r2, #48
	strb	ip, [r2, #-48]
	strb	r1, [r2, #-46]
	str	r1, [r2, #-40]
	bne	.L781
	ldr	r2, .L783+4
	mvn	r1, #0
	strb	r3, [r2, #3314]
	strb	r1, [r2, #3264]
	mov	r1, #0
	strb	r1, [r2, #3313]
	bx	lr
.L784:
	.align	2
.L783:
	.word	.LANCHOR0+1776
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
	ldrne	r2, .L793
	movne	r1, #48
	bne	.L791
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L787
	ldr	r1, .L793+4
	mov	r2, #74
	ldr	r0, .L793+8
	bl	printk
	bl	dump_stack
.L787:
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	strb	r3, [r5]
	ldmfd	sp!, {r3, r4, r5, pc}
.L791:
	mov	r5, r3
	mla	r3, r1, r3, r2
	ldrb	r3, [r3, #1776]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L791
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L789
	ldr	r1, .L793+4
	mov	r2, #81
	ldr	r0, .L793+8
	bl	printk
	bl	dump_stack
.L789:
	ldr	r2, .L793
	mov	r1, #48
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	mla	r5, r1, r5, r2
	strb	r3, [r5, #1776]
	ldmfd	sp!, {r3, r4, r5, pc}
.L794:
	.align	2
.L793:
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
	ldr	r0, .L797
	mov	r3, #0
	strb	r3, [r4, #43]
	mvn	r3, #0
	strb	r3, [r4]
	ldmfd	sp!, {r4, lr}
	b	buf_add_tail
.L798:
	.align	2
.L797:
	.word	.LANCHOR0+3315
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
	ldr	r4, .L809
	and	r3, r3, #8
	uxtb	r3, r3
	strb	r3, [r0, #2]
	cmp	r3, #0
	beq	.L800
	ldr	r3, [r0, #20]
	cmn	r3, #1
	beq	.L800
	mov	r1, r0
	ldr	r0, .L809+4
	bl	buf_add_tail
	b	.L801
.L800:
	ldrb	r3, [r4, #3313]	@ zero_extendqisi2
	strb	r3, [r0]
	ldrb	r3, [r0, #1]	@ zero_extendqisi2
	strb	r3, [r4, #3313]
.L801:
	ldrb	r3, [r4, #3314]	@ zero_extendqisi2
	add	r3, r3, #1
	strb	r3, [r4, #3314]
	ldmfd	sp!, {r4, pc}
.L810:
	.align	2
.L809:
	.word	.LANCHOR0
	.word	.LANCHOR0+3313
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
	ldr	r4, .L822
	ldrb	r3, [r4, #3314]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L812
.L815:
	ldr	r2, .L822
	cmp	r5, #0
	ldrb	r1, [r4, #3313]	@ zero_extendqisi2
	mov	r0, #48
	add	r3, r2, #1776
	mla	r0, r0, r1, r3
	beq	.L813
	b	.L814
.L812:
	ldr	r1, .L822+4
	mov	r2, #121
	ldr	r0, .L822+8
	bl	printk
	bl	dump_stack
	ldrb	r3, [r4, #3314]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L815
.L817:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L813:
	ldrb	r2, [r2, #3314]	@ zero_extendqisi2
	cmp	r2, #1
	beq	.L817
.L814:
	mov	r2, #48
	mul	r1, r2, r1
	add	r2, r4, r1
	add	r3, r3, r1
	mov	r1, #0
	ldrb	ip, [r2, #1776]	@ zero_extendqisi2
	strh	r1, [r3, #34]	@ movhi
	str	r1, [r2, #1784]
	strb	ip, [r4, #3313]
	ldrb	ip, [r4, #3314]	@ zero_extendqisi2
	strb	r1, [r2, #1816]
	sub	ip, ip, #1
	strb	r1, [r2, #1817]
	strb	ip, [r4, #3314]
	mov	ip, #1
	strb	ip, [r2, #1778]
	mvn	ip, #0
	strb	ip, [r2, #1776]
	str	ip, [r2, #1796]
	ldmfd	sp!, {r3, r4, r5, pc}
.L823:
	.align	2
.L822:
	.word	.LANCHOR0
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
	bne	.L825
	ldrb	r3, [r1]	@ zero_extendqisi2
	strb	r3, [r0]
	mov	r0, #1
	bx	lr
.L825:
	ldr	r0, .L832
	str	lr, [sp, #-4]!
	.save {lr}
	mov	lr, #48
.L827:
	cmp	r3, #255
	beq	.L831
	mov	r2, r3
	mla	r3, lr, r3, r0
	ldrb	r3, [r3, #1776]	@ zero_extendqisi2
	cmp	r3, ip
	bne	.L827
	ldr	r0, .L832
	mov	ip, #48
	ldrb	r3, [r1]	@ zero_extendqisi2
	mla	r2, ip, r2, r0
	mov	r0, #1
	strb	r3, [r2, #1776]
	mvn	r3, #0
	strb	r3, [r1]
	ldr	pc, [sp], #4
.L831:
	mov	r0, #0
	ldr	pc, [sp], #4
.L833:
	.align	2
.L832:
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
	ldr	r3, .L840
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r0
	ldrb	r2, [r3, #3314]	@ zero_extendqisi2
	mov	r4, r3
	cmp	r2, #0
	bne	.L835
	ldr	r1, .L840+4
	mov	r2, #172
	ldr	r0, .L840+8
	bl	printk
	bl	dump_stack
.L835:
	ldrb	r3, [r4, #3314]	@ zero_extendqisi2
	ldr	r6, .L840
	cmp	r3, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldr	r0, .L840+12
	mov	r1, r5
	bl	buf_remove_buf
	cmp	r0, #1
	ldreqb	r3, [r6, #3314]	@ zero_extendqisi2
	subeq	r3, r3, #1
	streqb	r3, [r6, #3314]
	ldreqb	r3, [r5, #2]	@ zero_extendqisi2
	orreq	r3, r3, #1
	streqb	r3, [r5, #2]
	ldmfd	sp!, {r4, r5, r6, pc}
.L841:
	.align	2
.L840:
	.word	.LANCHOR0
	.word	.LANCHOR1+1076
	.word	.LC0
	.word	.LANCHOR0+3313
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
	ldr	r3, .L843
	ldrb	r2, [r3, #17]	@ zero_extendqisi2
	ldrh	r3, [r3, #18]
	smulbb	r3, r2, r3
	ldr	r2, .L843+4
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
.L844:
	.align	2
.L843:
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
	ldr	r3, .L847
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	mov	r5, r1
	ldrb	r2, [r3, #17]	@ zero_extendqisi2
	mov	r6, r0
	ldrh	r4, [r3, #18]
	mov	r1, r6
	ldr	r0, .L847+4
	smulbb	r4, r2, r4
	mov	r2, r5
	bl	printk
	ldr	r3, .L847+8
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
.L848:
	.align	2
.L847:
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
	bne	.L850
	ldrb	r3, [r0, #1]	@ zero_extendqisi2
	and	r3, r3, #223
	cmp	r3, #88
	addeq	r0, r0, #2
.L850:
	ldrb	r3, [r0]	@ zero_extendqisi2
	and	r3, r3, #223
	cmp	r3, #88
	addeq	r0, r0, #1
	sub	r3, r0, #1
	mov	r0, #0
.L852:
	ldrb	r2, [r3, #1]!	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L861
	sub	r1, r2, #48
	uxtb	ip, r1
	cmp	ip, #9
	addls	r0, r1, r0, asl #4
	bls	.L852
	sub	r1, r2, #97
	cmp	r1, #5
	subls	r2, r2, #87
	bls	.L860
	sub	r1, r2, #65
	cmp	r1, #5
	bxhi	lr
	sub	r2, r2, #55
.L860:
	add	r0, r2, r0, asl #4
	b	.L852
.L861:
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
	ldr	r3, .L864
	ldr	r0, .L864+4
	bl	proc_create_data
	add	sp, sp, #12
	@ sp needed
	ldr	pc, [sp], #4
.L865:
	.align	2
.L864:
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
	ldr	r1, .L868
	ldr	r2, .L868+4
	bl	sprintf
	ldr	r1, .L868+8
	ldr	r5, .L868+12
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
	ldr	r1, .L868+16
	bl	sprintf
	ldr	r2, [r5, #520]
	ldr	r1, .L868+20
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r2, [r5, #3316]
	ldr	r1, .L868+24
	mov	r2, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r1, .L868+28
	add	r4, r4, r0
	mov	r0, r4
	add	r4, r4, #10
	bl	strcpy
	ldr	r2, [r5, #3320]
	mov	r0, r4
	ldr	r1, .L868+32
	bl	sprintf
	ldr	r2, [r5, #520]
	ldr	r1, .L868+36
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L868+40
	ldr	r2, [r3, #524]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L868+44
	ldr	r2, [r3, #528]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3324
	ldrh	r2, [r5, r3]
	ldr	r1, .L868+48
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3326
	ldrh	r2, [r5, r3]
	ldr	r1, .L868+52
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	add	r3, r5, #3328
	ldr	r1, .L868+56
	ldrh	r2, [r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3330
	ldrh	r2, [r5, r3]
	ldr	r1, .L868+60
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3332
	ldrh	r2, [r5, r3]
	ldr	r1, .L868+64
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r3, #3334
	ldrh	r2, [r5, r3]
	ldr	r1, .L868+68
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrb	r1, [r5, #3337]	@ zero_extendqisi2
	ldrb	r2, [r5, #3314]	@ zero_extendqisi2
	ldrb	r3, [r5, #3336]	@ zero_extendqisi2
	add	r4, r4, r0
	str	r1, [sp]
	ldr	r1, .L868+72
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+76
	ldrh	r2, [r3, #148]
	ldrh	r3, [r3, #146]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+80
	ldr	r2, [r3, #16]
	ldr	r3, [r3, #20]
	add	r2, r3, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+84
	ldr	r2, [r3, #24]
	ldr	r3, [r3, #28]
	add	r2, r3, r2, lsr #11
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+88
	ldr	r2, [r3, #64]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+92
	ldr	r2, [r3, #68]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r2, .L868+96
	ldr	r1, .L868+100
	ldr	r3, [r3, #12]
	umull	r2, r3, r3, r2
	mov	r2, r3, lsr #3
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldrb	r2, [r5]	@ zero_extendqisi2
	ldr	r1, .L868+104
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L868+108
	ldr	r2, [r3, #556]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L868+112
	ldr	r2, [r3, #552]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+116
	ldr	r2, [r3, #52]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+120
	ldr	r2, [r3, #60]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+124
	ldr	r2, [r3, #76]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldr	r1, .L868+128
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
	ldr	r1, .L868+132
	mov	r0, r4
	bl	sprintf
	ldr	r1, [r5, #580]
	ldrh	r2, [r1, #48]
	ldrh	r3, [r1, #54]
	ldrb	r1, [r1, #57]	@ zero_extendqisi2
	add	r4, r4, r0
	str	r1, [sp]
	ldr	r1, .L868+136
	mov	r0, r4
	bl	sprintf
	ldr	r1, [r5, #580]
	ldrh	r2, [r1, #80]
	ldrh	r3, [r1, #86]
	ldrb	r1, [r1, #89]	@ zero_extendqisi2
	add	r4, r4, r0
	str	r1, [sp]
	ldr	r1, .L868+140
	mov	r0, r4
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldrh	r1, [r3, #88]
	ldrh	r2, [r3, #74]
	str	r1, [sp]
	add	r4, r4, r0
	ldrh	r1, [r3, #92]
	mov	r0, r4
	str	r1, [sp, #4]
	ldrh	r1, [r3, #96]
	str	r1, [sp, #8]
	ldr	r1, .L868+144
	ldr	r3, [r3, #84]
	bl	sprintf
	ldr	r3, [r5, #3340]
	ldrh	r1, [r3, #90]
	ldrh	r2, [r3, #72]
	str	r1, [sp]
	add	r4, r4, r0
	ldrh	r1, [r3, #94]
	mov	r0, r4
	str	r1, [sp, #4]
	ldrh	r1, [r3, #98]
	str	r1, [sp, #8]
	ldr	r1, .L868+148
	ldr	r3, [r3, #80]
	bl	sprintf
	add	r3, r5, #3344
	ldr	r1, .L868+152
	ldrh	r2, [r3]
	add	r4, r4, r0
	mov	r0, r4
	bl	sprintf
	movw	r1, #3350
	ldrh	r1, [r5, r1]
	movw	r3, #3346
	ldrh	r2, [r5, r3]
	movw	r3, #3348
	ldrh	r3, [r5, r3]
	str	r1, [sp]
	movw	r1, #3352
	ldrh	r1, [r5, r1]
	add	r4, r4, r0
	mov	r0, r4
	str	r1, [sp, #4]
	ldr	r1, [r5, #3356]
	str	r1, [sp, #8]
	ldr	r1, .L868+156
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
	ldr	r1, .L868+160
	bl	sprintf
	ldr	r3, [r5, #580]
	ldr	r1, .L868+164
	ldr	r2, [r3, #544]
	str	r2, [sp]
	add	r4, r4, r0
	ldr	r2, [r5, #3340]
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
.L869:
	.align	2
.L868:
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
	ldr	r4, .L873
	orr	r3, r3, #2
	strb	r3, [r0, #2]
	add	r0, r4, #3360
	bl	buf_add_tail
	ldrb	r0, [r4, #3336]	@ zero_extendqisi2
	add	r0, r0, #1
	uxtb	r0, r0
	strb	r0, [r4, #3336]
	ldmfd	sp!, {r4, pc}
.L874:
	.align	2
.L873:
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
	movw	r7, #3372
	ldr	r4, .L879
	movw	r3, #3368
	mov	r1, #340
	mov	r5, #0
	add	r6, r4, r7
	strb	r5, [r4, #3361]
	mov	r0, r6
	strb	r5, [r4, #3362]
	str	r5, [r4, #3364]
	strh	r5, [r4, r3]	@ movhi
	bl	__memzero
	mvn	r3, #0
	strh	r3, [r4, r7]	@ movhi
	add	r3, r4, #3712
	movw	ip, #3348
	str	r5, [r4, #3380]
	ldrh	r2, [r3]
	mov	r3, r2, lsr #1
	strh	r3, [r6, #34]	@ movhi
	movw	r3, #3714
	mov	r0, r2, lsr #2
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3716
	strh	r5, [r4, r3]	@ movhi
	movw	r3, #3718
	strh	r5, [r4, r3]	@ movhi
	movw	r5, #3722
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	strh	r0, [r6, #32]	@ movhi
	smulbb	r1, r3, r2
	uxth	r1, r1
	sub	lr, r1, #32
	strh	lr, [r4, ip]	@ movhi
	movw	ip, #3346
	strh	r1, [r4, ip]	@ movhi
	movw	r1, #3352
	strh	r2, [r4, r1]	@ movhi
	movw	r2, #3350
	strh	r0, [r4, r2]	@ movhi
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	str	r0, [r4, #3724]
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrh	r3, [r4, r5]
	str	r0, [r4, #3728]
	ldrb	r0, [r4, #3720]	@ zero_extendqisi2
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	str	r0, [r4, #3732]
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	str	r0, [r4, #3736]
	ldrh	r0, [r4, r5]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	add	r2, r4, #3344
	add	r4, r4, #3744
	str	r0, [r4, #-4]
	ldrh	r3, [r4]
	mov	r3, r3, lsr #2
	strh	r3, [r2]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L880:
	.align	2
.L879:
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
	ldr	r3, .L916
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r0
	ldrh	r3, [r3]
	.pad #20
	sub	sp, sp, #20
	mov	r4, r1
	mov	r9, r2
	cmp	r3, r0
	bhi	.L882
	ldr	r1, .L916+4
	movw	r2, #338
	ldr	r0, .L916+8
	bl	printk
	bl	dump_stack
.L882:
	ldr	r7, .L916+12
	mov	r8, r6, asl #1
	mov	fp, r6, asl #2
	ldr	r3, [r7, #576]
	ldrh	r5, [r3, r8]
	ldr	r3, .L916+16
	ldr	r2, [r3]
	mov	r10, r3
	tst	r2, #256
	beq	.L883
	ldr	r3, [r7, #568]
	mov	r1, r6
	ldr	r2, .L916+20
	add	r3, r3, fp
	ldr	r0, .L916+24
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	str	r5, [sp]
	ldrh	r2, [r2, #52]
	mov	r3, r3, lsr #5
	str	r2, [sp, #4]
	movw	r2, #3350
	ldrh	r2, [r7, r2]
	str	r2, [sp, #8]
	mov	r2, r4
	bl	printk
.L883:
	ldr	r2, [r7, #568]
	ldr	r3, .L916+12
	add	r2, r2, fp
	ldrb	ip, [r2, #2]	@ zero_extendqisi2
	ands	fp, ip, #224
	bne	.L884
	cmp	r5, #0
	beq	.L902
	ldr	r1, .L916+4
	movw	r2, #343
	ldr	r0, .L916+8
	bl	printk
	bl	dump_stack
	b	.L902
.L884:
	movw	r2, #3372
	ldrh	r2, [r3, r2]
	cmp	r2, r6
	beq	.L902
	ldr	r2, [r3, #580]
	ldrh	r1, [r2, #48]
	cmp	r1, r6
	beq	.L902
	ldrh	r1, [r2, #16]
	cmp	r1, r6
	beq	.L902
	ldrh	r1, [r2, #80]
	cmp	r1, r6
	beq	.L902
	ldr	r1, .L916+20
	add	r3, r3, #3424
	ldrh	r0, [r1, #52]
	mov	r1, #0
.L886:
	cmp	r1, r0
	bcs	.L915
	ldrh	lr, [r3, #2]!
	cmp	lr, r6
	beq	.L902
	add	r1, r1, #1
	b	.L886
.L915:
	cmp	r4, #0
	bne	.L891
	movw	r3, #3746
	ldrh	r1, [r7, r3]
	cmp	r1, r6
	moveq	r0, r4
	beq	.L885
	ldr	lr, .L916+28
	mov	r3, r4
.L890:
	ldrh	r5, [r3, lr]
	cmp	r6, r5
	bne	.L889
	ldr	r0, [r10]
	ands	r0, r0, #256
	beq	.L885
	ldr	r3, .L916+12
	mov	r2, ip, lsr #5
	ldr	r0, .L916+32
	ldr	r3, [r3, #576]
	ldrh	r3, [r3, r8]
	str	r6, [sp]
	str	r1, [sp, #4]
	mov	r1, r6
	bl	printk
	b	.L902
.L889:
	add	r3, r3, #2
	cmp	r3, #16
	bne	.L890
	add	r5, r2, r9, asl #7
	add	r5, r5, #136
	b	.L892
.L891:
	add	r5, r2, #392
.L892:
	ldr	r3, [r10]
	tst	r3, #256
	beq	.L893
	ldr	r3, [r7, #576]
	mov	r1, r6
	mov	r2, r4
	ldrh	r3, [r3, r8]
	str	r0, [sp, #4]
	ldr	r0, .L916+24
	str	r3, [sp]
	movw	r3, #3350
	ldrh	r3, [r7, r3]
	str	r3, [sp, #8]
	mov	r3, ip, lsr #5
	bl	printk
.L893:
	add	r1, r5, #128
	mov	r3, r5
	movw	r0, #65535
.L896:
	mov	r2, r3
	add	r3, r3, #2
	ldrh	ip, [r2]
	cmp	ip, r0
	bne	.L894
	cmp	r4, #0
	strh	r6, [r2]	@ movhi
	ldreq	r3, [r7, #580]
	ldrne	r2, [r7, #580]
	addeq	r9, r3, r9, asl #1
	ldrneh	r3, [r2, #124]
	ldreqh	r3, [r9, #120]
	addne	r3, r3, #1
	addeq	r3, r3, #1
	strneh	r3, [r2, #124]	@ movhi
	streqh	r3, [r9, #120]	@ movhi
	b	.L914
.L894:
	cmp	r3, r1
	bne	.L896
.L914:
	mov	r0, #1
	b	.L885
.L902:
	mov	r0, #0
.L885:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L917:
	.align	2
.L916:
	.word	.LANCHOR0+564
	.word	.LANCHOR1+1200
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR0+3372
	.word	.LC87
	.word	.LANCHOR0+3748
	.word	.LC88
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
	movw	r3, #3764
	ldr	r4, .L924
	mov	r7, r0
	mov	r6, #1
	ldrh	r2, [r4, r3]
	ldrb	r1, [r4, #3766]	@ zero_extendqisi2
	mov	r0, r0, lsr r2
	rsb	r2, r2, #21
	mov	r2, r6, asl r2
	sub	r2, r2, #1
	uxth	r5, r0
	and	r0, r0, r2
	bl	__aeabi_uidiv
	mov	r3, r7
	ldr	r1, [r4, #3696]
	mov	r2, r5
	uxth	r8, r0
	ldr	r0, .L924+4
	bl	printk
	mov	r1, r6
	mov	r0, r8
	mov	r2, #0
	bl	gc_add_sblk
	ldr	r3, [r4, #3696]
	ldr	r0, .L924+8
	mov	r1, #0
.L919:
	uxth	r2, r1
	cmp	r2, r3
	bcs	.L923
	add	r2, r2, #164
	add	r1, r1, #1
	mov	r2, r2, asl #1
	ldrh	r2, [r0, r2]
	cmp	r2, r5
	bne	.L919
	b	.L920
.L923:
	cmp	r3, #5
	addls	r2, r3, #1
	addls	r3, r3, #164
	strls	r2, [r4, #3696]
	ldrls	r2, .L924+8
	movls	r3, r3, asl #1
	strlsh	r5, [r2, r3]	@ movhi
.L920:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L925:
	.align	2
.L924:
	.word	.LANCHOR0
	.word	.LC89
	.word	.LANCHOR0+3372
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
	ldr	r3, .L927
	ldr	r3, [r3, #3736]
	ldr	r0, [r3, r0, asl #2]
	bx	lr
.L928:
	.align	2
.L927:
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
	ldr	r4, .L960
	add	r8, r4, #1776
	mov	r5, r4
.L930:
	ldrb	r9, [r4, #3767]	@ zero_extendqisi2
	cmp	r9, #255
	beq	.L959
	mov	r3, #48
	ldr	fp, .L960+4
	mul	r3, r3, r9
	add	r2, r4, r3
	add	r3, r8, r3
	ldrb	r1, [r2, #1776]	@ zero_extendqisi2
	ldrh	r7, [r3, #32]
	strb	r1, [r4, #3767]
	ldr	r1, [r2, #1812]
	cmp	r1, #0
	beq	.L931
	ldr	r3, .L960+8
	mov	r0, #1
	ldr	r2, [r2, #1800]
	strh	r0, [r3]	@ movhi
	ldr	r0, .L960+12
	str	r2, [r4, #3692]
	bl	printk
	ldr	r1, .L960+16
	mov	r2, #672
	ldr	r0, .L960+20
	bl	printk
	bl	dump_stack
.L931:
	ldrb	r3, [r5, #3768]	@ zero_extendqisi2
	cmp	r3, #3
	ldreq	r3, [r4, #580]
	ldreqb	r6, [r3, #89]	@ zero_extendqisi2
	addeq	r6, r6, r6, asl #1
	beq	.L933
	ldrb	r3, [r4, #3769]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L945
	ldrb	r3, [r4, #3770]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, [r4, #580]
	ldrneb	r6, [r3, #89]	@ zero_extendqisi2
	movne	r6, r6, asl #1
	bne	.L933
.L945:
	mov	r6, #1
.L933:
	ldr	r3, .L960+24
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L934
	mov	r3, #48
	ldr	r0, .L960+28
	mla	r3, r3, r9, r5
	mov	r1, r7
	ldrb	r2, [r3, #1777]	@ zero_extendqisi2
	ldr	r3, [r5, #3732]
	ldrb	r3, [r3, r7]	@ zero_extendqisi2
	str	r3, [sp]
	mov	r3, r7
	bl	printk
.L934:
	mov	r3, #48
	ldr	r2, [r5, #3732]
	mla	r3, r3, r9, r5
	mov	r10, r7
	ldrb	r2, [r2, r7]	@ zero_extendqisi2
	ldrb	r3, [r3, #1777]	@ zero_extendqisi2
	cmp	r2, r3
	beq	.L935
	ldr	r1, .L960+16
	movw	r2, #681
	ldr	r0, .L960+20
	bl	printk
	bl	dump_stack
.L935:
	mov	r3, #48
	mla	r9, r3, r9, r5
	ldrb	r3, [r9, #1821]	@ zero_extendqisi2
	cmp	r3, #3
	beq	.L936
	ldrb	r3, [r4, #3768]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L936
	ldrb	r3, [r4, #3771]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L936
	ldrb	r3, [r4, #3769]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L937
	ldrb	r3, [r4, #3770]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L936
.L937:
	ldr	r2, [r5, #3732]
	mov	r3, #0
	mov	r1, #48
	mov	r0, r3
	add	r7, r2, r7
	b	.L938
.L936:
	ldr	r3, .L960+32
	mvn	r9, #0
	mov	r2, #0
	strh	r7, [r3]	@ movhi
	mov	r3, #48
.L939:
	rsb	r1, r7, r10
	uxth	r1, r1
	cmp	r1, r6
	bcs	.L930
	ldr	r1, [r5, #3732]
	str	r3, [sp, #12]
	str	r2, [sp, #8]
	ldrb	r0, [r1, r10]	@ zero_extendqisi2
	strb	r9, [r1, r10]
	add	r10, r10, #1
	mul	r0, r3, r0
	add	r1, r5, r0
	add	r0, fp, r0
	strb	r2, [r1, #1821]
	bl	zbuf_free
	ldrb	r1, [r5, #3379]	@ zero_extendqisi2
	sub	r1, r1, #1
	strb	r1, [r5, #3379]
	ldr	r3, [sp, #12]
	ldr	r2, [sp, #8]
	b	.L939
.L938:
	uxth	r2, r3
	cmp	r2, r6
	bcs	.L930
	ldrb	r2, [r7, r3]	@ zero_extendqisi2
	add	r3, r3, #1
	mla	r2, r1, r2, r5
	strb	r0, [r2, #1821]
	b	.L938
.L959:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L961:
	.align	2
.L960:
	.word	.LANCHOR0
	.word	.LANCHOR0+1776
	.word	.LANCHOR0+3688
	.word	.LC90
	.word	.LANCHOR1+1212
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC91
	.word	.LANCHOR0+3684
	.fnend
	.size	gc_write_completed, .-gc_write_completed
	.align	2
	.global	gc_get_src_blk
	.type	gc_get_src_blk, %function
gc_get_src_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L972
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r2, #580]
	ldrb	r0, [r2, #3361]	@ zero_extendqisi2
	ldrh	r1, [r3, #124]
	cmp	r1, #0
	addne	r3, r3, #392
	movne	r1, #1
	bne	.L964
	add	ip, r3, r0, asl #1
	ldrh	ip, [ip, #120]
	cmp	ip, #0
	beq	.L969
	add	r3, r3, r0, asl #7
	add	r3, r3, #136
.L964:
	add	ip, r3, #128
	movw	lr, #65535
.L968:
	mov	r4, r3
	ldrh	r0, [r3], #2
	cmp	r0, lr
	beq	.L966
	cmp	r1, #0
	mvn	r3, #0
	strh	r3, [r4]	@ movhi
	ldreqb	r1, [r2, #3361]	@ zero_extendqisi2
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
.L966:
	cmp	r3, ip
	bne	.L968
	ldmfd	sp!, {r4, pc}
.L969:
	movw	r0, #65535
	ldmfd	sp!, {r4, pc}
.L973:
	.align	2
.L972:
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
	ldr	r4, .L1076
	mov	r5, r0
	str	r1, [sp, #12]
	ldr	r3, [r4, #580]
	add	r3, r3, r0, asl #1
	ldrh	ip, [r3, #120]
	cmp	ip, #0
	movne	r0, ip
	bne	.L1071
	cmp	r5, #0
	mov	r6, r2
	bne	.L977
	movw	r3, #3716
	mov	r10, r5
	mov	r7, r5
	strh	r5, [r4, r3]	@ movhi
.L978:
	uxth	r9, r6
	uxth	r3, r10
	cmp	r3, r9
	bcs	.L983
	ldr	r3, .L1076+4
	add	r0, r3, #56
	ldrh	fp, [r3]
	str	r3, [sp, #16]
	mov	r1, fp
	bl	_list_get_gc_head_node
	ldr	r3, .L1076+4
	add	r2, fp, #1
	movw	r1, #65535
	uxth	r2, r2
	strh	r2, [r3]	@ movhi
	cmp	r0, r1
	mov	r8, r0
	ldr	r3, [sp, #16]
	beq	.L979
	ldr	r3, .L1076+8
	mov	fp, r0, asl #1
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L980
	ldr	r3, .L1076
	mov	r1, r8
	ldr	r0, .L1076+12
	ldr	r3, [r3, #576]
	ldrh	r3, [r3, fp]
	bl	printk
.L980:
	ldr	r3, [r4, #576]
	ldrh	r2, [r3, fp]
	ldr	r3, .L1076+16
	ldrh	r3, [r3]
	cmp	r2, r3
	bcs	.L981
	mov	r1, #0
	mov	r0, r8
	mov	r2, r1
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L982
	add	ip, r7, #1
	uxth	r7, ip
	cmp	r7, r9
	bls	.L982
	b	.L983
.L981:
	ldr	r2, .L1076
	movw	r3, #3716
	mov	r1, #0
	strh	r1, [r2, r3]	@ movhi
	b	.L983
.L979:
	mov	r2, #0
	strh	r2, [r3]	@ movhi
	b	.L983
.L982:
	add	r10, r10, #1
	b	.L978
.L983:
	ldr	r3, [sp, #12]
	tst	r3, #2
	beq	.L985
	ldr	r10, .L1076
	mov	fp, #0
	ldr	r8, .L1076+20
.L986:
	uxth	r3, fp
	cmp	r3, r9
	bcs	.L990
	ldrh	r3, [r8]
	ldr	r2, .L1076+20
	mov	r1, r3
	add	r0, r2, #58
	str	r2, [sp, #20]
	str	r3, [sp, #16]
	bl	_list_get_gc_head_node
	ldr	r3, [sp, #16]
	mov	lr, r0
	ldr	r2, [sp, #20]
	add	r1, r3, #1
	movw	r3, #65535
	cmp	r0, r3
	strh	r1, [r8]	@ movhi
	beq	.L987
	ldr	r1, [r10, #576]
	mov	r3, r0, asl #1
	ldrh	r1, [r1, r3]
	ldr	r3, .L1076+24
	ldrh	r3, [r3]
	cmp	r1, r3
	bcs	.L987
	mov	r1, #0
	mov	r2, r1
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L989
	add	ip, r7, #1
	uxth	r7, ip
	cmp	r7, r9
	bls	.L989
	b	.L992
.L987:
	mov	r3, #0
	strh	r3, [r2]	@ movhi
	b	.L990
.L989:
	add	fp, fp, #1
	b	.L986
.L990:
	cmp	r7, r9
	bcs	.L992
	ldr	r0, .L1076
	movw	r1, #3352
	ldrh	r2, [r4, r1]
	add	r3, r0, #3712
	ldrh	lr, [r3]
	movw	r3, #3722
	ldrh	r8, [r4, r3]
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	mul	r3, r3, r8
	sub	r3, r3, lr, lsr #2
	cmp	r2, r3
	addlt	r3, r2, lr, lsr #3
	strlth	r3, [r0, r1]	@ movhi
	b	.L985
.L992:
	ldr	r1, .L1076
	movw	r2, #3352
	ldrh	r3, [r4, r2]
	add	r0, r1, #3712
	ldrh	r0, [r0]
	cmp	r3, r0
	subhi	r3, r3, r0, lsr #3
	strhih	r3, [r1, r2]	@ movhi
.L985:
	ldr	r3, [sp, #12]
	tst	r3, #1
	beq	.L993
	cmp	r7, r9
	movcc	r10, #0
	ldrcc	r8, .L1076+28
	bcs	.L993
.L994:
	uxth	r2, r10
	cmp	r2, r9
	bcs	.L997
	ldrh	fp, [r8]
	ldr	r2, .L1076+28
	mov	r1, fp
	add	r0, r2, #66
	str	r2, [sp, #16]
	bl	_list_get_gc_head_node
	movw	r3, #65535
	add	fp, fp, #1
	strh	fp, [r8]	@ movhi
	cmp	r0, r3
	ldr	r2, [sp, #16]
	beq	.L995
	mov	r1, #0
	mov	r2, r1
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L996
	add	ip, r7, #1
	uxth	r7, ip
	cmp	r7, r9
	bcc	.L996
	b	.L999
.L995:
	mov	r3, #0
	strh	r3, [r2]	@ movhi
	b	.L997
.L996:
	add	r10, r10, #1
	b	.L994
.L997:
	cmp	r7, r9
	bcs	.L999
	ldr	r1, .L1076
	movw	r2, #3350
	ldrh	r3, [r4, r2]
	add	r0, r1, #3712
	ldrh	r0, [r0]
	cmp	r3, r0
	addcc	r3, r3, r0, lsr #3
	bcc	.L1075
	b	.L993
.L999:
	ldr	r1, .L1076
	movw	r2, #3350
	ldrh	r3, [r4, r2]
	add	r0, r1, #3712
	ldrh	r0, [r0]
	cmp	r3, r0, lsr #1
	bls	.L993
	sub	r3, r3, r0, lsr #3
.L1075:
	strh	r3, [r1, r2]	@ movhi
	b	.L993
.L977:
	ldr	r2, .L1076+32
	movw	r3, #314
	strh	ip, [r2, r3]	@ movhi
	movw	r3, #3368
	strh	ip, [r4, r3]	@ movhi
	ldr	r3, [sp, #12]
	ands	r9, r3, #1
	moveq	r7, r9
	beq	.L1000
	add	r8, r4, #3744
	movw	r3, #3332
	ldrh	r3, [r4, r3]
	ldrh	r2, [r8]
	str	r8, [sp, #16]
	cmp	r3, r2, lsr #2
	bcc	.L1001
	movw	r2, #3330
	ldrh	r2, [r4, r2]
	cmp	r2, r3
	movls	r7, ip
	bls	.L1002
.L1001:
	movw	r3, #3330
	movw	fp, #3714
	ldrh	r3, [r4, r3]
	mov	r1, #0
	ldr	r0, .L1076+36
	ldr	r10, .L1076
	mov	r3, r3, lsr #2
	strh	r3, [r4, fp]	@ movhi
	bl	_list_get_gc_head_node
	movw	r2, #65535
	cmp	r0, r2
	moveq	r7, #0
	beq	.L1003
	ldr	r1, [r10, #576]
	add	r2, r10, #3712
	mov	r3, r0, asl #1
	ldrh	r2, [r2]
	ldrh	r3, [r1, r3]
	mov	r1, #0
	cmp	r3, r2, lsr #2
	movcs	r7, r1
	bcs	.L1003
	mov	r2, r5
	strh	r1, [r10, fp]	@ movhi
	bl	gc_add_sblk
	adds	r7, r0, #0
	movne	r7, #1
.L1003:
	ldr	r10, .L1076+28
	mov	r8, #64
	movw	fp, #65535
.L1005:
	ldrh	r2, [r10]
	ldr	r0, .L1076+36
	mov	r1, r2
	str	r2, [sp, #20]
	bl	_list_get_gc_head_node
	ldr	r2, [sp, #20]
	cmp	r0, fp
	add	r2, r2, #1
	strh	r2, [r10]	@ movhi
	beq	.L1002
	mov	r1, #0
	mov	r2, r5
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1004
	add	ip, r7, #1
	uxth	r7, ip
	cmp	r7, r6
	bcs	.L1002
.L1004:
	sub	r3, r8, #1
	uxth	r8, r3
	cmp	r8, #0
	bne	.L1005
.L1002:
	movw	r3, #3332
	ldrh	r2, [r4, r3]
	ldr	r3, [sp, #16]
	ldrh	r3, [r3]
	cmp	r2, r3, lsr #3
	bhi	.L1022
	ldr	r1, .L1076
	movw	r3, #3330
	ldrh	r3, [r1, r3]
	add	r3, r3, #8
	cmp	r2, r3
	bgt	.L1022
.L1000:
	ldr	r3, [sp, #12]
	tst	r3, #2
	beq	.L1008
	ldr	r8, .L1076+20
	movw	r3, #3718
	mov	r10, #64
	mov	fp, #0
	strh	fp, [r4, r3]	@ movhi
.L1013:
	ldrh	r2, [r8]
	ldr	r0, .L1076+40
	mov	r1, r2
	str	r2, [sp, #16]
	bl	_list_get_gc_head_node
	ldr	r2, [sp, #16]
	add	r2, r2, #1
	strh	r2, [r8]	@ movhi
	movw	r2, #65535
	cmp	r0, r2
	beq	.L1009
	cmp	r6, #1
	ldr	r1, .L1076+44
	bne	.L1010
	ldr	r3, .L1076
	add	r2, r1, #376
	ldrh	r2, [r2]
	ldrb	lr, [r3, #3720]	@ zero_extendqisi2
	smulbb	r2, lr, r2
	add	lr, r3, #3712
	ldrh	lr, [lr]
	sub	r2, r2, lr, lsr #3
	strh	r2, [r1]	@ movhi
.L1010:
	ldr	lr, [r4, #576]
	mov	r2, r0, asl #1
	ldrh	lr, [lr, r2]
	ldrh	r2, [r1]
	mov	r1, #0
	cmp	lr, r2
	bcs	.L1011
	mov	r2, r5
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1012
	add	ip, r7, #1
	add	fp, fp, #1
	uxth	r7, ip
	uxth	fp, fp
	cmp	r7, r6
	bcc	.L1012
	b	.L1009
.L1011:
	ldr	r2, .L1076
	movw	r3, #3718
	strh	r1, [r2, r3]	@ movhi
	b	.L1009
.L1012:
	sub	r3, r10, #1
	uxth	r10, r3
	cmp	r10, #0
	bne	.L1013
.L1009:
	cmp	r7, r6
	ldr	r2, .L1076+48
	bcc	.L1014
	cmp	fp, #0
	bne	.L1015
	movw	r3, #3334
	ldrh	r1, [r4, r3]
	movw	r3, #3784
	ldrh	r3, [r4, r3]
	cmp	r1, r3
	bls	.L1015
.L1014:
	ldrh	r3, [r2]
	movw	r2, #3722
	ldrh	lr, [r4, r2]
	movw	r1, #3346
	ldrb	r2, [r4, #3720]	@ zero_extendqisi2
	ldrh	r0, [r4, r1]
	mov	r3, r3, lsr #3
	mul	r2, r2, lr
	rsb	r2, r3, r2
	cmp	r0, r2
	addlt	r3, r3, r0
	blt	.L1074
	b	.L1008
.L1015:
	ldrh	r0, [r2]
	movw	r1, #3346
	ldrb	r2, [r4, #3720]	@ zero_extendqisi2
	ldrh	r3, [r4, r1]
	mul	r2, r2, r0
	cmp	r3, r2
	ble	.L1008
	sub	r3, r3, r0, lsr #3
.L1074:
	ldr	r2, .L1076
	strh	r3, [r2, r1]	@ movhi
.L1008:
	cmp	r9, #0
	beq	.L993
	ldr	r10, .L1076
	movw	r3, #3716
	ldr	r9, .L1076+4
	mov	fp, #64
	mov	r2, #0
	strh	r2, [r4, r3]	@ movhi
.L1019:
	ldrh	r8, [r9]
	ldr	r0, .L1076+52
	mov	r1, r8
	add	r8, r8, #1
	bl	_list_get_gc_head_node
	movw	r3, #65535
	strh	r8, [r9]	@ movhi
	cmp	r0, r3
	beq	.L1016
	ldr	r2, [r10, #576]
	mov	r3, r0, asl #1
	ldrh	r2, [r2, r3]
	ldr	r3, .L1076+56
	ldrh	r1, [r3]
	cmp	r1, r2
	cmphi	r2, #2
	bls	.L1017
	add	r2, r3, #396
	ldrh	r3, [r3, #-16]
	ldrh	r2, [r2]
	cmp	r3, r2, lsr #1
	bls	.L1016
.L1017:
	mov	r1, #0
	mov	r2, r5
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1018
	add	ip, r7, #1
	uxth	r7, ip
	cmp	r7, r6
	bcs	.L1016
.L1018:
	sub	fp, fp, #1
	uxth	fp, fp
	cmp	fp, #0
	bne	.L1019
.L1016:
	cmp	r7, r6
	ldr	r3, .L1076+48
	movw	r1, #3348
	bcs	.L1020
	sub	r0, r3, #3712
	ldrh	r3, [r3]
	ldrh	r2, [r4, r1]
	cmp	r2, r3, lsr #1
	subhi	r2, r2, r3, lsr #3
	strhih	r2, [r0, r1]	@ movhi
	b	.L993
.L1020:
	ldrh	r0, [r3]
	sub	lr, r3, #3712
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	ldrh	r2, [r4, r1]
	mul	r3, r3, r0
	sub	r3, r3, #32
	cmp	r2, r3
	addlt	r2, r2, r0, lsr #3
	strlth	r2, [lr, r1]	@ movhi
.L993:
	ldr	r3, .L1076+8
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1021
	str	r6, [sp]
	mov	r1, r5
	ldr	r0, .L1076+60
	mov	r3, r7
	ldr	r2, [sp, #12]
	bl	printk
.L1021:
	mov	r0, r7
	b	.L1071
.L1022:
	ldr	r8, .L1076+4
	movw	r3, #3716
	mov	fp, #64
	movw	r10, #65535
	mov	r2, #0
	strh	r2, [r4, r3]	@ movhi
.L1007:
	ldrh	r3, [r8]
	ldr	r0, .L1076+52
	mov	r1, r3
	str	r3, [sp, #16]
	bl	_list_get_gc_head_node
	ldr	r3, [sp, #16]
	cmp	r0, r10
	add	r3, r3, #1
	strh	r3, [r8]	@ movhi
	beq	.L1000
	mov	r1, #0
	mov	r2, r5
	bl	gc_add_sblk
	cmp	r0, #0
	beq	.L1006
	add	ip, r7, #1
	uxth	r7, ip
	cmp	r7, r6
	bcs	.L1000
.L1006:
	sub	fp, fp, #1
	uxth	fp, fp
	cmp	fp, #0
	bne	.L1007
	b	.L1000
.L1071:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1077:
	.align	2
.L1076:
	.word	.LANCHOR0
	.word	.LANCHOR0+3716
	.word	.LANCHOR2
	.word	.LC92
	.word	.LANCHOR0+3350
	.word	.LANCHOR0+3718
	.word	.LANCHOR0+3352
	.word	.LANCHOR0+3714
	.word	.LANCHOR0+3372
	.word	.LANCHOR0+3780
	.word	.LANCHOR0+3776
	.word	.LANCHOR0+3346
	.word	.LANCHOR0+3712
	.word	.LANCHOR0+3772
	.word	.LANCHOR0+3348
	.word	.LC93
	.fnend
	.size	gc_search_src_blk, .-gc_search_src_blk
	.align	2
	.global	gc_free_temp_buf
	.type	gc_free_temp_buf, %function
gc_free_temp_buf:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r4, .L1093
	ldrb	r0, [r4, #3379]	@ zero_extendqisi2
	cmp	r0, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	ldrb	r3, [r4, #3314]	@ zero_extendqisi2
	cmp	r3, #1
	bhi	.L1085
	ldr	r3, .L1093+4
	mov	r0, #48
	ldrh	r5, [r3]
	movw	r3, #3722
	ldrh	r1, [r4, r3]
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	add	r2, r5, #24
	mul	r3, r3, r1
	ldr	r1, [r4, #3732]
	cmp	r3, r2
	movcs	r3, r2
.L1080:
	cmp	r5, r3
	bcs	.L1085
	ldrb	r2, [r1, r5]	@ zero_extendqisi2
	cmp	r2, #255
	beq	.L1081
	mul	r2, r0, r2
	ldr	r6, .L1093
	add	ip, r4, r2
	ldrb	ip, [ip, #1821]	@ zero_extendqisi2
	cmp	ip, #0
	bne	.L1081
	add	r0, r6, #1776
	add	r0, r0, r2
	bl	zbuf_free
	ldr	r3, .L1093+8
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1082
	ldr	r3, [r6, #3732]
	mov	r1, r5
	ldr	r0, .L1093+12
	ldrb	r2, [r3, r5]	@ zero_extendqisi2
	bl	printk
.L1082:
	ldr	r3, [r4, #3732]
	mvn	r2, #0
	mov	r0, #1
	strb	r2, [r3, r5]
	ldrb	r3, [r4, #3379]	@ zero_extendqisi2
	add	r3, r3, r2
	strb	r3, [r4, #3379]
	ldmfd	sp!, {r4, r5, r6, pc}
.L1081:
	add	r5, r5, #1
	b	.L1080
.L1085:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1094:
	.align	2
.L1093:
	.word	.LANCHOR0
	.word	.LANCHOR0+3684
	.word	.LANCHOR2
	.word	.LC94
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
	movw	r2, #3372
	ldr	r0, .L1097
	movw	ip, #310
	add	r3, r0, r2
	ldrh	r1, [r0, r2]
	add	r0, r0, #3344
	ldrh	r2, [r3, #2]
	ldrh	r3, [r3, ip]
	ldrb	ip, [r0, #-30]	@ zero_extendqisi2
	str	ip, [sp]
	ldrb	ip, [r0, #35]	@ zero_extendqisi2
	str	ip, [sp, #4]
	ldrh	r0, [r0]
	str	r0, [sp, #8]
	ldr	r0, .L1097+4
	bl	printk
	add	sp, sp, #20
	@ sp needed
	ldr	pc, [sp], #4
.L1098:
	.align	2
.L1097:
	.word	.LANCHOR0
	.word	.LC95
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
	ldreq	r0, .L1103
	beq	.L1102
	cmp	r1, #2
	ldreq	r0, .L1103+4
	ldrne	r0, .L1103+8
.L1102:
	mov	r1, r3
	b	_list_get_gc_head_node
.L1104:
	.align	2
.L1103:
	.word	.LANCHOR0+3776
	.word	.LANCHOR0+3780
	.word	.LANCHOR0+3772
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
	ldr	r2, .L1109
	mov	r1, r0
	ldr	r3, [r2, #568]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ands	r3, r3, #24
	ldreq	r0, .L1109+4
	subeq	r2, r0, #464
	beq	.L1108
	cmp	r3, #16
	addeq	r0, r2, #3792
	addne	r2, r2, #3328
	ldreq	r2, .L1109+8
	ldrne	r0, .L1109+12
.L1108:
	b	_insert_free_list
.L1110:
	.align	2
.L1109:
	.word	.LANCHOR0
	.word	.LANCHOR0+3788
	.word	.LANCHOR0+3326
	.word	.LANCHOR0+3796
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
	mov	r1, r0
	ldr	r0, .L1116
	ldr	r3, [r0, #568]
	add	r3, r3, r1, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #64
	ldreq	r0, .L1116+4
	ldreq	r2, .L1116+8
	beq	.L1115
.L1112:
	cmp	r3, #96
	ldreq	r0, .L1116+12
	subeq	r2, r0, #440
	beq	.L1115
	cmp	r3, #160
	bxne	lr
	ldr	r2, .L1116+16
	add	r0, r0, #3776
.L1115:
	b	_insert_data_list
.L1117:
	.align	2
.L1116:
	.word	.LANCHOR0
	.word	.LANCHOR0+3780
	.word	.LANCHOR0+3330
	.word	.LANCHOR0+3772
	.word	.LANCHOR0+3334
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
	ldr	r8, .L1132
	mov	r6, r0
	ldr	r3, [r8, #580]
	add	r3, r3, #588
	ldrh	r4, [r3]
	clz	r3, r0
	cmp	r4, r2
	mov	r3, r3, lsr #5
	moveq	r3, #0
	cmp	r3, #0
	beq	.L1119
	ldr	r0, .L1132+4
	mov	r1, r4
	bl	printk
	ldr	r3, [r8, #580]
	mvn	r2, #0
	add	r3, r3, #588
	strh	r2, [r3]	@ movhi
	b	.L1120
.L1119:
	movw	r3, #3326
	mov	r5, r8
	ldrh	r4, [r8, r3]
	add	r8, r8, #3328
	mov	r7, r1
	ldrh	r1, [r8]
	cmp	r4, r1
	bls	.L1121
	cmp	r0, #0
	ldr	r2, .L1132+8
	movne	r1, r4, lsr #3
	moveq	r1, r0
	ldr	r0, .L1132+12
	b	.L1131
.L1121:
	cmp	r0, #0
	moveq	r1, r1, lsr #2
	beq	.L1125
	rsb	r4, r4, r4, asl #3
	add	r3, r4, #7
	cmp	r4, #0
	movlt	r4, r3
	ubfx	r1, r4, #3, #16
.L1125:
	ldr	r0, .L1132+16
	sub	r2, r0, #468
.L1131:
	bl	_list_pop_index_node
	movw	r3, #65535
	uxth	r4, r0
	cmp	r4, r3
	bne	.L1126
	movw	r3, #3324
	ldr	r0, .L1132+20
	ldrh	r3, [r5, r3]
	mov	r1, r4
	mov	r2, r7
	str	r3, [sp]
	ldrh	r3, [r8]
	str	r3, [sp, #4]
	ldr	r3, [r5, #3788]
	bl	printk
.L1126:
	cmp	r6, #0
	beq	.L1120
	ldr	r3, .L1132+24
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1120
	ldr	r2, [r5, #568]
	mov	r0, r4, asl #2
	add	r3, r2, r0
	ldr	r1, [r2, r4, asl #2]
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ubfx	r1, r1, #11, #8
	str	r1, [sp]
	ldrh	r2, [r2, r0]
	ldr	r0, .L1132+28
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
.L1120:
	mov	r0, r4
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1133:
	.align	2
.L1132:
	.word	.LANCHOR0
	.word	.LC96
	.word	.LANCHOR0+3326
	.word	.LANCHOR0+3792
	.word	.LANCHOR0+3796
	.word	.LC97
	.word	.LANCHOR2
	.word	.LC98
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
	ldr	r5, .L1149
	bne	.L1135
	movw	r3, #3326
	ldrh	r1, [r5, r3]
	add	r3, r5, #3328
	ldrh	r2, [r3]
	cmp	r1, r2
	bcc	.L1136
	movw	r3, #3324
	ldrh	r3, [r5, r3]
	cmp	r2, #0
	cmpne	r3, r1
	addls	r0, r5, #3792
	movhi	r1, #1
	movls	r1, #0
	ldrls	r2, .L1149+4
	bls	.L1147
.L1136:
	ldr	r0, .L1149+8
	mov	r1, r2, lsr #1
	b	.L1148
.L1135:
	ldr	r2, [r5, #580]
	movw	r7, #590
	ldrh	r4, [r2, r7]
	movw	r2, #65535
	cmp	r1, #1
	cmpne	r4, r2
	beq	.L1139
	ldr	r0, .L1149+12
	mov	r1, r4
	bl	printk
	ldr	r3, [r5, #580]
	mvn	r2, #0
	strh	r2, [r3, r7]	@ movhi
	b	.L1140
.L1139:
	add	r1, r5, #3328
	movw	r2, #3324
	ldrh	r2, [r5, r2]
	mov	r3, r0
	ldrh	r1, [r1]
	cmp	r2, r1
	bcc	.L1141
	movw	r0, #3326
	ldrh	r0, [r5, r0]
	cmp	r1, #0
	cmpne	r0, r2
	bls	.L1142
.L1141:
	cmp	r6, #1
	ldr	r0, .L1149+8
	moveq	r3, r1, lsr #1
	mov	r1, r3
.L1148:
	sub	r2, r0, #468
	b	.L1147
.L1142:
	cmp	r6, #1
	ldr	r0, .L1149+16
	moveq	r3, r2, lsr #1
	sub	r2, r0, #464
	mov	r1, r3
.L1147:
	bl	_list_pop_index_node
	movw	r3, #65535
	uxth	r4, r0
	cmp	r4, r3
	bne	.L1140
	movw	r3, #3324
	ldr	r0, .L1149+20
	ldrh	r3, [r5, r3]
	mov	r1, r4
	mov	r2, r6
	str	r3, [sp]
	ldr	r3, .L1149+24
	ldrh	r3, [r3]
	str	r3, [sp, #4]
	ldr	r3, [r5, #3788]
	bl	printk
.L1140:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L1150:
	.align	2
.L1149:
	.word	.LANCHOR0
	.word	.LANCHOR0+3326
	.word	.LANCHOR0+3796
	.word	.LC99
	.word	.LANCHOR0+3788
	.word	.LC97
	.word	.LANCHOR0+3328
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
	mov	r1, r0
	ldr	r0, .L1156
	ldr	r3, [r0, #568]
	add	r3, r3, r1, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #64
	ldreq	r0, .L1156+4
	ldreq	r2, .L1156+8
	beq	.L1155
.L1152:
	cmp	r3, #96
	ldreq	r0, .L1156+12
	subeq	r2, r0, #440
	beq	.L1155
	cmp	r3, #160
	bxne	lr
	ldr	r2, .L1156+16
	add	r0, r0, #3776
.L1155:
	b	_list_remove_node
.L1157:
	.align	2
.L1156:
	.word	.LANCHOR0
	.word	.LANCHOR0+3780
	.word	.LANCHOR0+3330
	.word	.LANCHOR0+3772
	.word	.LANCHOR0+3334
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
	ldr	r2, .L1162
	mov	r1, r0
	ldr	r3, [r2, #568]
	add	r3, r3, r0, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ands	r3, r3, #24
	ldreq	r0, .L1162+4
	subeq	r2, r0, #464
	beq	.L1161
	cmp	r3, #16
	addeq	r0, r2, #3792
	addne	r2, r2, #3328
	ldreq	r2, .L1162+8
	ldrne	r0, .L1162+12
.L1161:
	b	_list_remove_node
.L1163:
	.align	2
.L1162:
	.word	.LANCHOR0
	.word	.LANCHOR0+3788
	.word	.LANCHOR0+3326
	.word	.LANCHOR0+3796
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
	mov	r1, r0
	ldr	r0, .L1169
	ldr	r3, [r0, #568]
	add	r3, r3, r1, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #64
	ldreq	r0, .L1169+4
	ldreq	r2, .L1169+8
	beq	.L1168
.L1165:
	cmp	r3, #96
	ldreq	r0, .L1169+12
	subeq	r2, r0, #440
	beq	.L1168
	cmp	r3, #160
	bxne	lr
	ldr	r2, .L1169+16
	add	r0, r0, #3776
.L1168:
	b	_list_update_data_list
.L1170:
	.align	2
.L1169:
	.word	.LANCHOR0
	.word	.LANCHOR0+3780
	.word	.LANCHOR0+3330
	.word	.LANCHOR0+3772
	.word	.LANCHOR0+3334
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
	ldr	r0, .L1176
	bl	printk
	ldr	r4, [r4]
	cmp	r4, #0
	beq	.L1171
	ldr	r6, .L1176+4
	mov	r5, #0
	ldr	r7, .L1176+8
.L1174:
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
	ldr	r0, .L1176+12
	str	r1, [sp, #24]
	mov	r1, r5
	bl	printk
	ldrh	r2, [r4]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L1171
	ldr	r3, [r6, #524]
	mov	r4, #6
	add	r5, r5, #1
	mla	r4, r4, r2, r3
	ldr	r3, .L1176+16
	uxth	r5, r5
	ldrh	r3, [r3]
	cmp	r3, r5
	bcs	.L1174
.L1171:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L1177:
	.align	2
.L1176:
	.word	.LC100
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC101
	.word	.LANCHOR0+3800
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
	ldr	r4, .L1180
	add	r0, r4, #3776
	add	r1, r4, #3312
	add	r0, r0, #12
	add	r1, r1, #12
	bl	print_list_info
	add	r1, r4, #3312
	add	r0, r4, #3792
	add	r1, r1, #14
	bl	print_list_info
	add	r0, r4, #3792
	add	r1, r4, #3328
	add	r0, r0, #4
	bl	print_list_info
	add	r0, r4, #3776
	add	r1, r4, #3328
	add	r0, r0, #4
	add	r1, r1, #2
	bl	print_list_info
	add	r0, r4, #3760
	add	r1, r4, #3328
	add	r0, r0, #12
	add	r1, r1, #4
	bl	print_list_info
	add	r1, r4, #3328
	add	r0, r4, #3776
	add	r1, r1, #6
	ldmfd	sp!, {r4, lr}
	b	print_list_info
.L1181:
	.align	2
.L1180:
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
	ldr	r3, .L1187
	ldr	r3, [r3, #3340]
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
.L1188:
	.align	2
.L1187:
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
	ldr	r4, .L1304
	ldr	r3, [r4, #3340]
	ldr	r3, [r3, #32]
	cmp	r3, #10240
	bls	.L1190
	bl	ftl_tmp_into_update
.L1190:
	ldr	r3, [r4, #580]
	ldr	r2, [r3, #568]
	ldr	r1, [r3, #12]
	add	r2, r2, #860160
	add	r2, r2, #3840
	cmp	r1, r2
	bhi	.L1191
	ldr	r2, .L1304
	ldr	r0, [r2, #3340]
	ldr	r2, [r3, #572]
	ldr	r0, [r0, #36]
	add	r2, r2, #32
	cmp	r0, r2
	movls	r7, #0
	movls	r10, r7
	bls	.L1192
.L1191:
	ldr	r2, [r4, #3340]
	mov	r8, #0
	ldr	ip, .L1304
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
.L1193:
	ldr	r3, .L1304
	add	r2, r3, #564
	ldrh	r2, [r2]
	cmp	r2, r7
	bls	.L1301
	ldr	r0, [ip, #568]
	mov	r1, r7, asl #2
	add	lr, r0, r1
	ldrb	r3, [lr, #2]	@ zero_extendqisi2
	and	r2, r3, #224
	cmp	r2, #224
	beq	.L1194
	tst	r3, #8
	ldrneh	r3, [r0, r1]
	ldrne	r4, [r0, r1]
	ubfxne	r3, r3, #0, #11
	ubfxne	r4, r4, #11, #8
	bne	.L1196
.L1195:
	tst	r3, #24
	ldreqh	r3, [r0, r1]
	movweq	r4, #65535
	ldrne	r4, [r0, r1]
	movwne	r3, #65535
	ubfxeq	r3, r3, #0, #11
	ubfxne	r4, r4, #11, #8
	bne	.L1198
.L1196:
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
.L1239:
	ldr	r2, [sp, #20]
	cmp	r2, r3
	movw	r2, #65535
	strcc	r3, [sp, #20]
	movcc	r8, r7
	cmp	r4, r2
	beq	.L1200
.L1198:
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
.L1200:
	cmp	r4, #9
	cmphi	r3, #9
	bhi	.L1194
	ldr	r3, .L1304+4
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L1194
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
	ldr	r0, .L1304+8
	str	r1, [sp, #12]
	mov	r1, r7
	bl	printk
	ldr	ip, [sp, #36]
.L1194:
	add	r7, r7, #1
	uxth	r7, r7
	b	.L1193
.L1301:
	ldr	r4, [r3, #580]
	mov	r2, #0
	mov	r1, #255
	str	r3, [sp, #36]
	add	r0, r4, #264
	strh	r2, [r4, #122]	@ movhi
	mov	r2, #128
	bl	memset
	ldr	ip, .L1304+4
	movw	r2, #586
	ldrh	r1, [r4, r2]
	ldr	ip, [ip]
	tst	ip, #1024
	mov	r2, r1, asl #2
	ldr	r3, [sp, #36]
	ldr	r0, [r3, #568]
	add	r4, r0, r2
	beq	.L1203
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
	ldr	r0, .L1304+12
	str	r3, [sp, #12]
	ubfx	r3, ip, #11, #8
	bl	printk
.L1203:
	ldrb	r3, [r4, #2]	@ zero_extendqisi2
	tst	r3, #8
	beq	.L1204
	and	r2, r3, #192
	ldr	r7, .L1304
	cmp	r2, #64
	bne	.L1205
	ldr	r2, [r7, #580]
	movw	r3, #586
	mov	r1, #0
	ldrh	r0, [r2, r3]
	mov	r2, #1
	bl	gc_add_sblk
	ldr	r2, .L1304+16
	movw	r3, #314
	mov	r1, #1
	strh	r1, [r2, r3]	@ movhi
	b	.L1204
.L1205:
	tst	r3, #224
	bne	.L1204
	ldr	r3, [r7, #580]
	movw	r2, #590
	movw	r1, #65535
	ldrh	r0, [r3, r2]
	cmp	r0, r1
	bne	.L1204
	movw	r4, #586
	str	r2, [sp, #36]
	ldrh	r0, [r3, r4]
	add	r3, r3, #588
	ldrh	r3, [r3]
	cmp	r3, r0
	beq	.L1204
	bl	zftl_remove_free_node
	ldr	r3, [r7, #580]
	ldrh	r1, [r3, r4]
	ldr	r2, [sp, #36]
	strh	r1, [r3, r2]	@ movhi
	mvn	r2, #0
	strh	r2, [r3, r4]	@ movhi
.L1204:
	ldr	r4, .L1304
	ldr	r0, .L1304+4
	ldr	r3, [r4, #580]
	ldr	r0, [r0]
	add	r3, r3, #584
	tst	r0, #1024
	ldrh	r1, [r3]
	ldr	r3, [r4, #568]
	mov	r2, r1, asl #2
	add	r7, r3, r2
	beq	.L1206
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
	ldr	r0, .L1304+20
	bl	printk
.L1206:
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	tst	r3, #8
	beq	.L1207
	and	r2, r3, #192
	ldr	r7, .L1304
	cmp	r2, #64
	bne	.L1208
	ldr	r3, [r7, #580]
	mov	r1, #0
	mov	r2, #1
	add	r3, r3, #584
	ldrh	r0, [r3]
	bl	gc_add_sblk
	ldr	r2, .L1304+16
	movw	r3, #314
	mov	r1, #1
	strh	r1, [r2, r3]	@ movhi
	b	.L1207
.L1208:
	tst	r3, #224
	bne	.L1207
	ldr	r3, [r7, #580]
	add	r2, r3, #588
	ldrh	r1, [r2]
	movw	r2, #65535
	cmp	r1, r2
	bne	.L1207
	add	r2, r3, #584
	ldrh	r0, [r2]
	movw	r2, #590
	ldrh	r3, [r3, r2]
	cmp	r3, r0
	beq	.L1207
	bl	zftl_remove_free_node
	ldr	r3, [r7, #580]
	add	r2, r3, #588
	add	r3, r3, #584
	ldrh	r1, [r3]
	strh	r1, [r2]	@ movhi
	mvn	r2, #0
	strh	r2, [r3]	@ movhi
.L1207:
	ldr	r2, .L1304+4
	ldr	ip, .L1304
	ldr	r2, [r2]
	ldr	r3, [ip, #568]
	tst	r2, #1024
	beq	.L1209
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
	ldr	r0, .L1304+24
	str	r1, [sp, #12]
	mov	r1, r8
	bl	printk
.L1209:
	ldr	r2, .L1304+4
	ldr	ip, .L1304
	ldr	r2, [r2]
	ldr	r3, [ip, #568]
	tst	r2, #1024
	beq	.L1210
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
	ldr	r0, .L1304+28
	str	r1, [sp, #12]
	mov	r1, fp
	bl	printk
.L1210:
	ldr	r4, [r4, #3340]
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
	ldr	r4, .L1304+4
	ldr	r3, [r4]
	tst	r3, #1024
	beq	.L1211
	uxth	r0, r0
	mov	r1, r10
	str	r0, [sp]
	uxth	r3, r7
	ldr	r0, .L1304+32
	ldr	r2, [sp, #24]
	bl	printk
.L1211:
	ldr	r3, [r4]
	ldr	r7, .L1304
	tst	r3, #1024
	beq	.L1212
	movw	r3, #3802
	str	r9, [sp]
	ldrh	r3, [r7, r3]
	mov	r1, r5
	ldr	r0, .L1304+36
	mov	r2, r6
	str	r3, [sp, #4]
	movw	r3, #3804
	ldrh	r3, [r7, r3]
	str	r3, [sp, #8]
	ldr	r3, [sp, #20]
	bl	printk
.L1212:
	rsb	r3, r6, r9
	str	r3, [sp, #24]
	movw	r3, #3804
	ldr	r2, [sp, #24]
	ldrh	r3, [r7, r3]
	cmp	r2, r3
	bgt	.L1213
	ldr	r1, .L1304
	movw	r2, #3802
	ldr	r3, [sp, #20]
	ldrh	r2, [r1, r2]
	rsb	r3, r5, r3
	cmp	r3, r2
	movle	r7, #0
	movle	r10, r7
	ble	.L1214
.L1213:
	ldr	r3, [r7, #580]
	ldr	r4, [r3, #580]
	ldr	r3, [r7, #3340]
	mov	r7, #0
	mov	r10, r7
	uxth	r4, r4
	ldrh	fp, [r3, #134]
	ldr	r3, .L1304+40
	sub	r9, r3, #564
.L1215:
	ldr	r3, .L1304+40
	ldrh	r3, [r3]
	cmp	fp, r3
	bcs	.L1224
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
	bne	.L1217
	ubfx	r2, r2, #3, #2
	ands	r1, r2, #1
	beq	.L1218
	cmp	r0, #160
	b	.L1299
.L1218:
	cmp	r2, #2
.L1299:
	bne	.L1220
	ldr	r2, .L1304+44
	ldr	r0, [sp, #24]
	ldrh	r2, [r2]
	cmp	r0, r2
	ble	.L1221
	ldr	r2, [r3, r8]
	ubfx	r2, r2, #11, #8
	cmp	r2, r6
	bls	.L1222
	cmp	r1, #0
	beq	.L1221
	ldrh	r2, [r3, r8]
	ubfx	r2, r2, #0, #11
	cmp	r2, r5
	bgt	.L1221
.L1222:
	mov	r1, #0
	mov	r2, #1
	mov	r0, r4
	str	r3, [sp, #28]
	bl	gc_add_sblk
	ldr	r2, .L1304+48
	mov	r1, #1
	add	r10, r10, r1
	strh	r1, [r2]	@ movhi
	ldr	r2, .L1304+4
	ldr	r2, [r2]
	tst	r2, #1024
	ldr	r3, [sp, #28]
	beq	.L1221
	ldr	r0, [r9, #576]
	mov	r1, r4, asl r1
	ldr	r2, [r9, #568]
	ldrh	lr, [r0, r1]
	add	r2, r2, r8
	ldr	r1, .L1304+16
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	ldr	r0, .L1304+52
	ldrh	r1, [r1, #52]
	str	r1, [sp]
	ldrh	r1, [r3, r8]
	ubfx	r1, r1, #0, #11
	str	r1, [sp, #4]
	ldr	r3, [r3, r8]
	ubfx	r3, r3, #11, #8
	str	r3, [sp, #8]
	b	.L1300
.L1220:
	ldr	r0, .L1304+56
	ldr	r2, [sp, #20]
	ldrh	r0, [r0]
	rsb	r2, r5, r2
	cmp	r2, r0
	ble	.L1221
	ldrh	r2, [r3, r8]
	add	r0, r5, #8
	ubfx	r2, r2, #0, #11
	cmp	r2, r0
	ble	.L1223
	cmp	r1, #0
	beq	.L1221
	ldr	r2, [r3, r8]
	add	r1, r6, #4
	ubfx	r2, r2, #11, #8
	cmp	r2, r1
	bgt	.L1221
.L1223:
	mov	r1, #0
	mov	r2, #1
	mov	r0, r4
	str	r3, [sp, #28]
	bl	gc_add_sblk
	ldr	r2, .L1304+48
	mov	r1, #1
	add	r7, r7, r1
	strh	r1, [r2]	@ movhi
	ldr	r2, .L1304+4
	ldr	r2, [r2]
	tst	r2, #1024
	ldr	r3, [sp, #28]
	beq	.L1221
	ldr	r0, [r9, #576]
	mov	r1, r4, asl r1
	ldr	r2, [r9, #568]
	ldrh	lr, [r0, r1]
	add	r2, r2, r8
	ldr	r1, .L1304+16
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	ldr	r0, .L1304+60
	ldrh	r1, [r1, #52]
	str	r1, [sp]
	ldrh	r1, [r3, r8]
	ubfx	r1, r1, #0, #11
	str	r1, [sp, #4]
	ldr	r3, [r3, r8]
	ubfx	r3, r3, #11, #8
	str	r3, [sp, #8]
.L1300:
	mov	r1, r4
	mov	r2, r2, lsr #5
	mov	r3, lr
	bl	printk
.L1221:
	cmp	r10, #4
	cmpls	r7, #4
	bhi	.L1224
.L1217:
	add	fp, fp, #1
	uxth	fp, fp
	b	.L1215
.L1224:
	ldr	r3, .L1304
	ldr	r3, [r3, #580]
	str	r4, [r3, #580]
.L1214:
	ldr	r0, .L1304
	cmp	r6, #0
	ldrne	r3, [r0, #3340]
	movne	r4, r0
	ldrneh	r3, [r3, #134]
	beq	.L1227
.L1228:
	ldr	r2, .L1304
	add	r1, r2, #564
	ldrh	r1, [r1]
	cmp	r1, r3
	bls	.L1302
	ldr	ip, [r4, #568]
	mov	lr, r3, asl #2
	add	r8, ip, lr
	ldr	r2, [ip, r3, asl #2]
	ubfx	r1, r2, #11, #8
	cmp	r1, r6
	bcc	.L1229
	ldrb	r8, [r8, #2]	@ zero_extendqisi2
	tst	r8, #24
	rsbne	r1, r6, r1
	bfine	r2, r1, #11, #8
	strne	r2, [ip, lr]
.L1229:
	add	r3, r3, #1
	uxth	r3, r3
	b	.L1228
.L1302:
	ldr	r3, [r2, #3340]
	ldrh	r2, [r3, #72]
	add	r2, r6, r2
	strh	r2, [r3, #72]	@ movhi
	ldrh	r2, [r3, #98]
	cmp	r2, r6
	rsbhi	r2, r6, r2
	strhih	r2, [r3, #98]	@ movhi
.L1227:
	cmp	r5, #0
	beq	.L1233
	ldr	r3, [r0, #3340]
	ldr	r4, .L1304
	ldrh	r3, [r3, #134]
.L1234:
	ldr	r2, .L1304
	add	r1, r2, #564
	ldrh	r1, [r1]
	cmp	r1, r3
	bls	.L1303
	ldr	ip, [r4, #568]
	mov	r0, r3, asl #2
	add	lr, ip, r0
	ldrh	r2, [ip, r0]
	ubfx	r1, r2, #0, #11
	cmp	r1, r5
	blt	.L1235
	ldrb	lr, [lr, #2]	@ zero_extendqisi2
	and	lr, lr, #24
	cmp	lr, #16
	rsbne	r1, r5, r1
	bfine	r2, r1, #0, #11
	strneh	r2, [ip, r0]	@ movhi
.L1235:
	add	r3, r3, #1
	uxth	r3, r3
	b	.L1234
.L1303:
	ldr	r3, [r2, #3340]
	ldrh	r2, [r3, #74]
	add	r2, r5, r2
	strh	r2, [r3, #74]	@ movhi
	ldrh	r2, [r3, #96]
	cmp	r2, r5
	rsbhi	r2, r5, r2
	strhih	r2, [r3, #96]	@ movhi
.L1233:
	ldr	r4, .L1304
	mov	r1, #0
	add	r0, r4, #3776
	bl	_list_get_gc_head_node
	movw	r2, #65535
	cmp	r0, r2
	beq	.L1192
	ldr	r2, [r4, #576]
	mov	r3, r0, asl #1
	add	r4, r4, #3712
	ldrh	r2, [r2, r3]
	ldrh	r3, [r4]
	cmp	r2, r3
	bhi	.L1192
	mov	r1, #0
	mov	r2, #1
	bl	gc_add_sblk
	add	r10, r10, #1
.L1192:
	add	r0, r7, r10
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1305:
	.align	2
.L1304:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC102
	.word	.LC103
	.word	.LANCHOR0+3372
	.word	.LC104
	.word	.LC105
	.word	.LC106
	.word	.LC107
	.word	.LC108
	.word	.LANCHOR0+564
	.word	.LANCHOR0+3804
	.word	.LANCHOR0+3686
	.word	.LC109
	.word	.LANCHOR0+3802
	.word	.LC110
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
	ldr	r6, .L1316
	mov	r9, r0
	ldr	r2, .L1316+4
	mov	r7, r1
	mov	r8, r5
	ldr	r4, [r6, #568]
	mov	r10, r6
	add	r4, r4, r0, asl #2
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
.L1307:
	ldrb	r1, [r6, #3720]	@ zero_extendqisi2
	cmp	r8, r1
	bge	.L1314
	mov	r1, r3, asr r8
	tst	r1, #1
	bne	.L1308
	ldrb	r4, [r10, #3766]	@ zero_extendqisi2
	mov	r0, r8
	str	r2, [sp, #4]
	mov	fp, r5, asl #1
	str	r3, [sp]
	add	r5, r5, #1
	mov	r1, r4
	bl	__aeabi_idiv
	smulbb	r4, r9, r4
	ldr	r2, [sp, #4]
	ldr	r3, [sp]
	ldrh	r1, [r2]
	rsb	r1, r1, #21
	add	r0, r4, r0, asl r1
	uxth	r4, r0
	strh	r4, [r7, fp]	@ movhi
	ldrb	r1, [r10, #3766]	@ zero_extendqisi2
	cmp	r1, #1
	subhi	r1, r1, #1
	andhi	r1, r1, r8
	addhi	r4, r4, r1
	strhih	r4, [r7, fp]	@ movhi
.L1308:
	add	r8, r8, #1
	b	.L1307
.L1314:
	add	r7, r7, r5, asl #1
	mov	r0, r5
	mvn	r2, #0
.L1311:
	ldrb	r3, [r6, #3720]	@ zero_extendqisi2
	cmp	r5, r3
	strlth	r2, [r7], #2	@ movhi
	addlt	r5, r5, #1
	blt	.L1311
.L1315:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1317:
	.align	2
.L1316:
	.word	.LANCHOR0
	.word	.LANCHOR0+3764
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
	movw	r3, #3764
	ldr	r5, .L1324
	mov	r7, r1
	ldrh	r4, [r5, r3]
	mvn	r3, #0
	rsb	r4, r4, #21
	mov	r6, r0, asr r4
	bic	r4, r0, r3, asl r4
	ldrb	r3, [r5, #3769]	@ zero_extendqisi2
	uxtb	r6, r6
	sxth	r4, r4
	cmp	r3, #0
	beq	.L1319
	ldrb	r3, [r5, #3770]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1319
	movw	r3, #3806
	clz	r1, r1
	ldrh	r2, [r5, r3]
	mov	r0, r6
	mov	r1, r1, lsr #5
	mul	r2, r2, r4
	bl	flash_erase_block_en
.L1319:
	movw	r3, #3806
	mov	r0, r6
	ldrh	r2, [r5, r3]
	uxtb	r1, r7
	mul	r2, r2, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, lr}
	b	flash_erase_block_en
.L1325:
	.align	2
.L1324:
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
	ldr	r6, .L1354
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
.L1327:
	ldrb	r2, [r6, #3808]	@ zero_extendqisi2
	ldr	r3, .L1354
	cmp	r5, r2
	bge	.L1338
	ldrb	r2, [r10, #3766]	@ zero_extendqisi2
	mov	r4, #0
	sub	r3, r2, #1
	str	r3, [sp, #4]
	ldr	r3, .L1354+4
	mul	lr, r2, r5
	mul	ip, r2, r8
	ldrh	r3, [r3]
	mov	r0, r3
	mov	r3, r4
.L1339:
	cmp	r3, r2
	bge	.L1353
	ldr	fp, [sp, #8]
	add	r1, r3, lr
	mov	r1, fp, asr r1
	tst	r1, #1
	bne	.L1328
	add	r1, sp, #80
	add	fp, r1, r4, asl #2
	ldr	r1, [sp, #4]
	add	r4, r4, #1
	and	r1, r3, r1
	add	r1, ip, r1
	mul	r1, r0, r1
	str	r1, [fp, #-64]
.L1328:
	add	r3, r3, #1
	b	.L1339
.L1353:
	cmp	r2, #4
	bne	.L1351
	uxtb	r3, r5
	uxtb	ip, r7
	mov	fp, #0
.L1330:
	cmp	fp, r4
	beq	.L1333
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
	b	.L1330
.L1351:
	cmp	r4, #2
	bne	.L1334
	ldrb	r3, [r6, #3769]	@ zero_extendqisi2
	uxtb	r4, r5
	cmp	r3, #0
	beq	.L1335
	ldrb	r3, [r10, #3770]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1335
	clz	r1, r7
	mov	r0, r4
	ldr	r2, [sp, #16]
	mov	r1, r1, lsr #5
	ldr	r3, [sp, #20]
	bl	flash_erase_duplane_block
.L1335:
	mov	r0, r4
	uxtb	r1, r7
	ldr	r2, [sp, #16]
	ldr	r3, [sp, #20]
	bl	flash_erase_duplane_block
	b	.L1333
.L1334:
	cmp	r4, #1
	bne	.L1333
	ldrb	r3, [r6, #3769]	@ zero_extendqisi2
	uxtb	r4, r5
	cmp	r3, #0
	beq	.L1337
	ldrb	r3, [r10, #3770]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L1337
	clz	r1, r7
	mov	r0, r4
	ldr	r2, [sp, #16]
	mov	r1, r1, lsr #5
	bl	flash_erase_block_en
.L1337:
	mov	r0, r4
	uxtb	r1, r7
	ldr	r2, [sp, #16]
	bl	flash_erase_block_en
.L1333:
	add	r5, r5, #1
	b	.L1327
.L1338:
	cmp	r7, #0
	ldr	r0, [r3, #568]
	bne	.L1340
	ldrh	r2, [r0, r9]
	ubfx	r1, r2, #0, #11
	add	r1, r1, #1
	bfi	r2, r1, #0, #11
	strh	r2, [r0, r9]	@ movhi
	ldr	r2, [r3, #3340]
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
	b	.L1342
.L1340:
	ldr	r2, [r0, r8, asl #2]
	ubfx	r1, r2, #11, #8
	add	r1, r1, #1
	bfi	r2, r1, #11, #8
	str	r2, [r0, r8, asl #2]
	ldr	r2, [r3, #3340]
	ldr	r3, [r3, #568]
	ldr	r1, [r2, #80]
	add	r1, r1, #1
	str	r1, [r2, #80]
	ldr	r3, [r3, r8, asl #2]
	ldrh	r1, [r2, #98]
	ubfx	r3, r3, #11, #8
	cmp	r1, r3
	strcch	r3, [r2, #98]	@ movhi
.L1342:
	mov	r0, #0
	add	sp, sp, #84
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1355:
	.align	2
.L1354:
	.word	.LANCHOR0
	.word	.LANCHOR0+3806
	.fnend
	.size	ftl_erase_sblk, .-ftl_erase_sblk
	.align	2
	.global	ftl_alloc_sys_blk
	.type	ftl_alloc_sys_blk, %function
ftl_alloc_sys_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1367
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r2, [r3, #3340]
	mov	r4, r3
	ldrh	r1, [r2, #136]
	cmp	r1, #63
	movhi	r3, #0
	strhih	r3, [r2, #136]	@ movhi
	ldrh	r3, [r2, #112]
	cmp	r3, #0
	bne	.L1358
	ldr	r1, .L1367+4
	movw	r2, #1130
	ldr	r0, .L1367+8
	bl	printk
	bl	dump_stack
.L1358:
	ldr	r3, [r4, #3340]
	movw	lr, #65535
	mov	ip, #0
.L1362:
	ldrh	r2, [r3, #136]
	add	r1, r2, #79
	add	r1, r3, r1, asl #1
.L1359:
	cmp	r2, #63
	bgt	.L1366
	ldrh	r0, [r1, #2]!
	cmp	r0, lr
	bne	.L1364
	add	r2, r2, #1
	b	.L1359
.L1366:
	strh	ip, [r3, #136]	@ movhi
	b	.L1362
.L1364:
	add	r1, r3, r2, asl #1
	mvn	ip, #0
	strh	ip, [r1, #160]	@ movhi
	strh	r2, [r3, #136]	@ movhi
	ldrh	r2, [r3, #112]
	add	r2, r2, ip
	strh	r2, [r3, #112]	@ movhi
	ldmfd	sp!, {r4, pc}
.L1368:
	.align	2
.L1367:
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
	ldr	r3, .L1379
	ldr	r2, [r3, #3340]
	mov	r4, r3
	ldrh	r1, [r2, #138]
	cmp	r1, #63
	movhi	r3, #0
	strhih	r3, [r2, #138]	@ movhi
	ldrh	r3, [r2, #112]
	cmp	r3, #63
	bls	.L1371
	ldr	r1, .L1379+4
	movw	r2, #1154
	ldr	r0, .L1379+8
	bl	printk
	bl	dump_stack
.L1371:
	ldr	r3, [r4, #3340]
	movw	ip, #65535
	mov	r0, #0
.L1375:
	ldrh	r2, [r3, #138]
	add	r1, r2, #79
	add	r1, r3, r1, asl #1
.L1372:
	cmp	r2, #63
	bgt	.L1378
	ldrh	lr, [r1, #2]!
	cmp	lr, ip
	bne	.L1373
	add	r1, r3, r2, asl #1
	strh	r5, [r1, #160]	@ movhi
	strh	r2, [r3, #138]	@ movhi
	ldrh	r2, [r3, #112]
	add	r2, r2, #1
	strh	r2, [r3, #112]	@ movhi
	ldmfd	sp!, {r3, r4, r5, pc}
.L1373:
	add	r2, r2, #1
	b	.L1372
.L1378:
	strh	r0, [r3, #138]	@ movhi
	b	.L1375
.L1380:
	.align	2
.L1379:
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
	ldr	r4, .L1391
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
	ldr	r2, [r4, #3340]
	tst	r3, #8
	ldrneh	r3, [r2, #116]
	subne	r3, r3, #1
	strneh	r3, [r2, #116]	@ movhi
	bne	.L1386
	tst	r3, #24
	ldreqh	r3, [r2, #114]
	ldrneh	r3, [r2, #118]
	subeq	r3, r3, #1
	subne	r3, r3, #1
	streqh	r3, [r2, #114]	@ movhi
	strneh	r3, [r2, #118]	@ movhi
.L1386:
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	and	r3, r3, #224
	cmp	r3, #160
	bne	.L1388
	ldr	r3, [r5, r6, asl #2]
	ubfx	r2, r3, #11, #8
	add	r2, r2, #1
	bfi	r3, r2, #11, #8
	str	r3, [r5, r6, asl #2]
	ldr	r2, [r4, #3340]
	ldrh	r3, [r2, #120]
	sub	r3, r3, #1
	strh	r3, [r2, #120]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1388:
	ldrh	r2, [r5, r8]
	cmp	r3, #64
	ubfx	r1, r2, #0, #11
	add	r1, r1, #1
	bfi	r2, r1, #0, #11
	strh	r2, [r5, r8]	@ movhi
	bne	.L1389
	ldr	r2, [r4, #3340]
	ldrh	r3, [r2, #122]
	sub	r3, r3, #1
	strh	r3, [r2, #122]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1389:
	cmp	r3, #96
	ldreq	r2, [r4, #3340]
	ldreqh	r3, [r2, #124]
	subeq	r3, r3, #1
	streqh	r3, [r2, #124]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1392:
	.align	2
.L1391:
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
	mov	r4, r0
	ldr	r6, .L1398
	add	r3, r6, #3712
	ldrb	r2, [r6, #3720]	@ zero_extendqisi2
	ldrh	r3, [r3]
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
	bne	.L1396
	ldr	r1, .L1398+4
	movw	r2, #1593
	ldr	r0, .L1398+8
	bl	printk
	bl	dump_stack
.L1396:
	movw	r3, #3806
	ldrh	r0, [r6, r3]
	mla	r0, r0, r4, r8
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1399:
	.align	2
.L1398:
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
	ldr	r2, .L1406
	str	lr, [sp, #-4]!
	.save {lr}
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	ldr	r2, .L1406+4
	ldr	r1, [r2, #-3860]
	ldr	ip, [r2, #-3856]
	movw	r2, #65535
.L1401:
	cmp	r3, r2
	beq	.L1405
	ldr	lr, [r1, r3, asl #2]
	cmp	lr, r0
	bne	.L1402
	mov	r0, r3
	ldr	lr, [sp], #4
	b	ftl_get_ppa_from_index
.L1402:
	mov	r3, r3, asl #1
	ldrh	r3, [ip, r3]
	b	.L1401
.L1405:
	mvn	r0, #0
	ldr	pc, [sp], #4
.L1407:
	.align	2
.L1406:
	.word	.LANCHOR0+3812
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
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	movw	r3, #65535
	ldrh	r2, [r0]
	mov	r4, r0
	cmp	r2, r3
	bne	.L1409
	ldr	r1, .L1418
	movw	r2, #1716
	ldr	r0, .L1418+4
	bl	printk
	bl	dump_stack
.L1409:
	ldr	r3, .L1418+8
	ldrh	r2, [r4, #2]
	ldrh	r3, [r3]
	cmp	r2, r3
	bne	.L1410
	ldr	r1, .L1418
	movw	r2, #1717
	ldr	r0, .L1418+4
	bl	printk
	bl	dump_stack
.L1410:
	ldrh	r3, [r4, #6]
	cmp	r3, #0
	bne	.L1411
	ldr	r1, .L1418
	movw	r2, #1718
	ldr	r0, .L1418+4
	bl	printk
	bl	dump_stack
.L1411:
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	movw	r2, #65535
	ldr	r1, .L1418+12
	mov	lr, #0
	add	r3, r4, r3, asl #1
	ldrb	r0, [r1, #3720]	@ zero_extendqisi2
	ldrh	ip, [r3, #16]
.L1412:
	cmp	ip, r2
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	bne	.L1417
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r4, #5]
	cmp	r3, r0
	streqb	lr, [r4, #5]
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	add	r3, r4, r3, asl #1
	ldrh	ip, [r3, #16]
	b	.L1412
.L1417:
	movw	r0, #3806
	add	r3, r3, #1
	ldrh	r0, [r1, r0]
	ldrb	r1, [r1, #3720]	@ zero_extendqisi2
	uxtb	r3, r3
	ldrh	r2, [r4, #2]
	mul	r0, r0, ip
	ldrh	ip, [r4, #6]
	cmp	r1, r3
	strb	r3, [r4, #5]
	sub	ip, ip, #1
	strh	ip, [r4, #6]	@ movhi
	ldrh	ip, [r4, #10]
	orr	r0, r0, r2
	moveq	r3, #0
	addeq	r2, r2, #1
	add	ip, ip, #1
	strh	ip, [r4, #10]	@ movhi
	streqh	r2, [r4, #2]	@ movhi
	streqb	r3, [r4, #5]
	ldmfd	sp!, {r4, pc}
.L1419:
	.align	2
.L1418:
	.word	.LANCHOR1+1296
	.word	.LC0
	.word	.LANCHOR0+3712
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
	bls	.L1421
	ldr	r1, .L1423
	movw	r2, #1749
	ldr	r0, .L1423+4
	bl	printk
	bl	dump_stack
.L1421:
	ldr	r5, .L1423+8
	mov	r1, #0
	mov	r0, r4
	bl	ftl_erase_phy_blk
	ldr	r3, [r5, #3340]
	ldrh	r0, [r3, #130]
	bl	ftl_free_sys_blk
	ldr	r3, [r5, #3340]
	mov	r0, #0
	strh	r4, [r3, #130]	@ movhi
	strh	r0, [r3, #140]	@ movhi
	ldmfd	sp!, {r3, r4, r5, pc}
.L1424:
	.align	2
.L1423:
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
	ldr	r2, .L1438
	ldrh	r3, [r2, #-12]
	cmp	r3, #4
	cmpls	r0, #0
	addeq	r3, r3, #1
	streqh	r3, [r2, #-12]	@ movhi
	bxeq	lr
	ldr	r3, .L1438+4
	mov	r0, #0
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	add	ip, r3, #564
	strh	r0, [r2, #-12]	@ movhi
	movw	r6, #65535
	ldrh	lr, [ip]
	mov	ip, r0
	ldr	r1, [r3, #576]
	ldr	r2, [r3, #568]
	sub	r1, r1, #2
	add	lr, r2, lr, asl #2
.L1428:
	cmp	r2, lr
	beq	.L1437
	ldrh	r4, [r1, #2]!
	cmp	r4, r6
	beq	.L1429
	ldrb	r5, [r2, #2]	@ zero_extendqisi2
	and	r5, r5, #224
	cmp	r5, #160
	addeq	r0, r0, r4
	addne	ip, ip, r4
.L1429:
	add	r2, r2, #4
	b	.L1428
.L1437:
	ldr	r4, [r3, #580]
	ldr	r3, [r3, #3340]
	str	ip, [r4, #524]
	str	r0, [r4, #528]
	ldrh	r1, [r3, #120]
	cmp	r1, #0
	ldmeqfd	sp!, {r4, r5, r6, pc}
	bl	__aeabi_uidiv
	str	r0, [r4, #532]
	ldmfd	sp!, {r4, r5, r6, pc}
.L1439:
	.align	2
.L1438:
	.word	.LANCHOR3-3840
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
	ldr	r3, .L1445
	mov	r4, r4, asl #1
	ldr	r2, [r3, #576]
	ldrh	r2, [r2, r4]
	cmp	r2, #0
	moveq	r0, #1
	movne	r0, #0
	streq	r0, [r3, #3356]
	ldmfd	sp!, {r4, pc}
.L1446:
	.align	2
.L1445:
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
	ldr	r6, .L1457
	beq	.L1448
	ldr	r2, [r6, #576]
	mov	r3, r0, asl #1
	ldrh	r4, [r2, r3]
	cmp	r4, #0
	subne	r4, r4, #1
	strneh	r4, [r2, r3]	@ movhi
	bne	.L1448
	ldr	r3, [r6, #568]
	mov	r1, r5
	mov	r2, r4
	add	r3, r3, r0, asl #2
	ldr	r0, .L1457+4
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	mov	r3, r3, lsr #5
	bl	printk
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1448:
	movw	r3, #3746
	ldrh	r0, [r6, r3]
	cmp	r0, r5
	beq	.L1454
	movw	r2, #65535
	cmp	r0, r2
	ldreq	r2, .L1457
	streqh	r5, [r2, r3]	@ movhi
	beq	.L1454
	bl	ftl_vpn_update
	ldr	r3, .L1457+8
	ldr	r1, .L1457+12
	ldrh	r2, [r3, #-10]
	add	r2, r2, #1
	uxth	r2, r2
	adds	r0, r0, #0
	movne	r0, #1
	cmp	r2, #7
	movhi	r2, #0
	strh	r2, [r3, #-10]	@ movhi
	ldrh	r3, [r3, #-10]
	movw	r2, #3746
	ldrh	ip, [r6, r2]
	strh	r5, [r6, r2]	@ movhi
	mov	r3, r3, asl #1
	strh	ip, [r1, r3]	@ movhi
	ldmfd	sp!, {r4, r5, r6, pc}
.L1454:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1458:
	.align	2
.L1457:
	.word	.LANCHOR0
	.word	.LC111
	.word	.LANCHOR3-3840
	.word	.LANCHOR0+3748
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
	ldr	r3, .L1471
	movw	lr, #65535
	ldr	r6, .L1471+4
	mov	r5, r5, asl #1
	mov	r8, lr
	ldr	r7, [r3, #-3860]
	ldrh	ip, [r6, r5]
	ldr	r9, [r3, #-3856]
.L1460:
	cmp	ip, r8
	beq	.L1464
	ldr	r4, [r7, ip, asl #2]
	add	r10, r7, ip, asl #2
	cmp	r4, r0
	mov	r4, ip, asl #1
	bne	.L1461
	mvn	ip, #0
	str	ip, [r10]
	movw	ip, #65535
	cmp	lr, ip
	ldr	ip, [r3, #-3856]
	movne	lr, lr, asl #1
	ldreqh	ip, [ip, r4]
	ldrneh	r7, [ip, r4]
	streqh	ip, [r6, r5]	@ movhi
	strneh	r7, [ip, lr]	@ movhi
	mvn	lr, #0
	ldr	ip, [r3, #-3856]
	strh	lr, [ip, r4]	@ movhi
	b	.L1464
.L1461:
	mov	lr, ip
	ldrh	ip, [r9, r4]
	b	.L1460
.L1464:
	ldr	ip, [r3, #-3860]
	cmn	r1, #1
	str	r0, [ip, r2, asl #2]
	ldrh	r0, [r6, r5]
	ldr	r3, [r3, #-3856]
	strh	r2, [r6, r5]	@ movhi
	mov	r2, r2, asl #1
	ldr	ip, .L1471+8
	strh	r0, [r3, r2]	@ movhi
	beq	.L1466
	movw	r3, #3764
	mvn	r0, #0
	ldrh	r3, [ip, r3]
	mov	r1, r1, lsr r3
	rsb	r2, r3, #21
	bic	r0, r1, r0, asl r2
	ldrb	r1, [ip, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	uxth	r0, r0
	bl	ftl_vpn_decrement
.L1466:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L1472:
	.align	2
.L1471:
	.word	.LANCHOR3
	.word	.LANCHOR0+3812
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
	ldr	r3, .L1481
	movw	r2, #3764
	stmfd	sp!, {r0, r1, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #8
	ubfx	r7, r0, #21, #3
	ldrh	r2, [r3, r2]
	ldrb	r4, [r3, #3766]	@ zero_extendqisi2
	mvn	r3, #0
	rsb	r1, r2, #21
	mvn	r3, r3, asl r1
	and	r0, r3, r0, lsr r2
	mov	r1, r4
	uxth	r8, r0
	bl	__aeabi_uidiv
	smulbb	r5, r7, r4
	cmp	r4, #1
	subhi	r3, r4, #1
	uxtb	r5, r5
	andhi	r3, r3, r8
	uxth	r6, r0
	addhi	r3, r5, r3
	uxtbhi	r5, r3
	ldr	r3, .L1481+4
	ldr	r3, [r3]
	tst	r3, #16384
	beq	.L1475
	str	r8, [sp]
	mov	r1, r7
	ldr	r0, .L1481+8
	mov	r2, r5
	mov	r3, r6
	bl	printk
.L1475:
	ldr	r3, .L1481
	add	r2, r3, #564
	ldrh	r2, [r2]
	cmp	r2, r6
	ldrhi	r4, [r3, #568]
	movhi	r2, #1
	addhi	r4, r4, r6, asl #2
	ldrhib	r3, [r4, #3]	@ zero_extendqisi2
	orrhi	r5, r3, r2, asl r5
	strhib	r5, [r4, #3]
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L1482:
	.align	2
.L1481:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC112
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
	ldr	r10, .L1498
	ldr	r3, [r10, #3696]
	cmp	r3, #0
	beq	.L1494
	ldr	r7, .L1498+4
	mov	r9, r0
	mov	fp, #0
	mov	r8, r7
.L1484:
	ldrb	r3, [r10, #3720]	@ zero_extendqisi2
	uxth	r0, fp
	cmp	r3, r0
	bls	.L1494
	ldr	r3, .L1498
	ldrb	r6, [r3, #3766]	@ zero_extendqisi2
	mov	r1, r6
	uxth	r4, r6
	bl	__aeabi_idiv
	ldr	r1, .L1498+8
	smulbb	r3, r9, r4
	cmp	r6, #1
	ldrh	r2, [r1]
	subhi	r4, r4, #1
	andhi	r4, r4, fp
	ldr	ip, .L1498
	rsb	r2, r2, #21
	mov	r6, r1
	add	r0, r3, r0, asl r2
	mov	r2, #0
	uxth	r5, r0
	addhi	r5, r5, r4
	uxthhi	r5, r5
.L1486:
	ldr	r3, [r10, #3696]
	uxth	r4, r2
	cmp	r4, r3
	bcs	.L1496
	add	r3, r4, #164
	mov	r3, r3, asl #1
	ldrh	r3, [r7, r3]
	cmp	r3, r5
	bne	.L1487
	mov	r1, r5
	ldr	r0, .L1498+12
	stmia	sp, {r2, ip}
	bl	printk
	ldrh	r0, [r6]
	mov	r0, r5, asl r0
	bl	ftl_mask_bad_block
	ldr	ip, [sp, #4]
	ldr	r2, [sp]
	ldr	r3, [ip, #3696]
.L1488:
	cmp	r4, r3
	bcs	.L1497
	add	r1, r4, #165
	mov	r1, r1, asl #1
	ldrh	r0, [r8, r1]
	add	r1, r4, #164
	add	r4, r4, #1
	mov	r1, r1, asl #1
	uxth	r4, r4
	strh	r0, [r8, r1]	@ movhi
	b	.L1488
.L1497:
	sub	r3, r3, #1
	str	r3, [r10, #3696]
.L1487:
	add	r2, r2, #1
	b	.L1486
.L1496:
	add	fp, fp, #1
	b	.L1484
.L1494:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1499:
	.align	2
.L1498:
	.word	.LANCHOR0
	.word	.LANCHOR0+3372
	.word	.LANCHOR0+3764
	.word	.LC113
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
	mov	r3, r0, asl #2
	ldr	r6, .L1521
	.pad #20
	sub	sp, sp, #20
	mov	r7, r0
	ldr	r10, [r6, #568]
	add	r8, r10, r3
	ldrb	r4, [r8, #2]	@ zero_extendqisi2
	tst	r4, #8
	mov	r2, r4, lsr #5
	str	r2, [sp]
	beq	.L1502
	ldr	r1, [r6, #3340]
	ldrh	r2, [r10, r3]
	ldr	r5, [r10, r0, asl #2]
	ldrh	fp, [r1, #74]
	ubfx	r2, r2, #0, #11
	ldrh	r9, [r1, #72]
	add	r1, r6, #572
	add	fp, r2, fp
	str	r3, [sp, #12]
	ldrh	ip, [r1]
	ubfx	r5, r5, #11, #8
	uxth	fp, fp
	str	r2, [sp, #8]
	add	r9, r5, r9
	mov	r1, ip
	mov	r0, fp
	str	ip, [sp, #4]
	bl	__aeabi_uidiv
	ldr	lr, .L1521+4
	uxth	r9, r9
	ldrh	r1, [lr, #-8]
	uxtah	r0, r9, r0
	ldr	ip, [sp, #4]
	cmp	r0, r1
	ldr	r2, [sp, #8]
	ldr	r3, [sp, #12]
	ble	.L1503
	movw	r1, #3334
	ldrh	r0, [r6, r1]
	movw	r1, #3326
	ldrh	r1, [r6, r1]
	add	r0, r0, r1
	movw	r1, #3784
	ldrh	r1, [r6, r1]
	add	r1, r1, #8
	cmp	r0, r1
	blt	.L1520
	movw	r1, #3330
	movw	r0, #3324
	ldrh	r0, [r6, r0]
	ldrh	r1, [r6, r1]
	add	r1, r1, r0
	movw	r0, #3332
	ldrh	r0, [r6, r0]
	add	r1, r1, r0
	ldrh	r0, [lr, #-6]
	add	r0, r0, #8
	cmp	r1, r0
	bge	.L1520
	b	.L1508
.L1503:
	mla	r9, ip, r9, fp
	ldrh	r1, [lr, #-4]
	cmp	r9, r1
	ble	.L1505
	movw	r1, #3330
	movw	r0, #3324
	ldrh	r0, [r6, r0]
	ldrh	r1, [r6, r1]
	add	r1, r1, r0
	movw	r0, #3332
	ldrh	r0, [r6, r0]
	add	r1, r1, r0
	ldrh	r0, [lr, #-6]
	add	r0, r0, #8
	cmp	r1, r0
	blt	.L1508
	movw	r1, #3334
	movw	r0, #3326
	ldrh	r1, [r6, r1]
	ldrh	r0, [r6, r0]
	add	r0, r1, r0
	movw	r1, #3784
	ldrh	r1, [r6, r1]
	add	r1, r1, #8
	cmp	r0, r1
	bge	.L1508
.L1520:
	mov	r1, #2
	bfi	r4, r1, #3, #2
	b	.L1518
.L1508:
	bfc	r4, #3, #2
.L1518:
	strb	r4, [r8, #2]
.L1505:
	ldrb	r1, [r8, #2]	@ zero_extendqisi2
	ands	r1, r1, #24
	bne	.L1509
	mul	ip, ip, r5
	ldrh	r1, [r10, r3]
	add	ip, ip, ip, asl #1
	add	r2, r2, ip, asr #2
	ubfx	r2, r2, #2, #9
	bfi	r1, r2, #0, #11
	strh	r1, [r10, r3]	@ movhi
	b	.L1502
.L1509:
	cmp	r1, #16
	bne	.L1502
	mov	r0, r2
	mov	r1, ip
	str	r3, [sp, #4]
	bl	__aeabi_idiv
	ldr	r2, [r10, r7, asl #2]
	add	r0, r0, r0, asl #1
	add	r5, r5, r0, asr #2
	ubfx	r5, r5, #2, #6
	bfi	r2, r5, #11, #8
	str	r2, [r10, r7, asl #2]
	ldr	r3, [sp, #4]
.L1502:
	mov	r0, r7
	str	r3, [sp, #4]
	bl	zftl_remove_data_node
	ldr	r2, .L1521
	mov	r0, #0
	ldr	r1, [r2, #568]
	mov	r4, r2
	ldr	r3, [sp, #4]
	add	r3, r1, r3
	ldrb	r1, [r3, #2]	@ zero_extendqisi2
	bfc	r1, #5, #3
	strb	r1, [r3, #2]
	ldr	r1, [r2, #576]
	mov	r3, r7, asl #1
	strh	r0, [r1, r3]	@ movhi
	ldr	r3, [sp]
	add	r1, r3, #6
	and	r1, r1, #7
	cmp	r1, #4
	bhi	.L1511
	mov	r0, r7
	bl	gc_free_bad_sblk
.L1511:
	ldrb	r3, [r8, #2]	@ zero_extendqisi2
	tst	r3, #8
	beq	.L1512
	ldr	r3, [r4, #580]
	add	r2, r3, #584
	ldrh	r1, [r2]
	cmp	r1, r7
	bne	.L1513
	add	r3, r3, #588
	mvn	r1, #0
	strh	r1, [r2]	@ movhi
	movw	r2, #65535
	ldrh	r1, [r3]
	cmp	r1, r2
	streqh	r7, [r3]	@ movhi
	ldreq	r0, .L1521+8
	bne	.L1512
	b	.L1519
.L1513:
	movw	r2, #586
	ldrh	r1, [r3, r2]
	cmp	r1, r7
	bne	.L1512
	mvn	r1, #0
	strh	r1, [r3, r2]	@ movhi
	movw	r2, #590
	movw	r1, #65535
	ldrh	r0, [r3, r2]
	cmp	r0, r1
	bne	.L1512
	ldr	r0, .L1521+12
	strh	r7, [r3, r2]	@ movhi
.L1519:
	mov	r1, r7
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	printk
.L1512:
	mov	r0, r7
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	zftl_insert_free_list
.L1522:
	.align	2
.L1521:
	.word	.LANCHOR0
	.word	.LANCHOR3-3840
	.word	.LC114
	.word	.LC115
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
	ldr	r6, .L1562
	ldr	r8, .L1562+4
	mov	r7, r6
.L1524:
	ldrh	r2, [r8, #52]
	uxth	r3, r5
	cmp	r2, r3
	bls	.L1561
	add	r3, r8, r3, asl #1
	ldrh	r4, [r3, #54]
	ldr	r3, [r6, #576]
	mov	r9, r4, asl #1
	ldrh	r2, [r3, r9]
	cmp	r2, #0
	beq	.L1525
	ldr	r0, .L1562+8
	mov	r1, r4
	bl	printk
.L1525:
	ldr	r3, [r7, #576]
	mov	r2, #0
	strh	r2, [r3, r9]	@ movhi
	ldr	r3, [r7, #576]
	ldrh	r3, [r3, r9]
	cmp	r3, r2
	bne	.L1526
	ldr	r3, .L1562+12
	ldr	r9, [r6, #568]
	ldr	r3, [r3]
	add	r9, r9, r4, asl #2
	tst	r3, #256
	beq	.L1527
	ldrb	r2, [r9, #2]	@ zero_extendqisi2
	mov	r1, r4
	ldr	r0, .L1562+16
	mov	r2, r2, lsr #5
	bl	printk
.L1527:
	ldrb	r3, [r9, #2]	@ zero_extendqisi2
	tst	r3, #192
	and	r3, r3, #224
	moveq	r2, #1
	movne	r2, #0
	cmp	r3, #224
	movne	r3, r2
	orreq	r3, r2, #1
	cmp	r3, #0
	beq	.L1528
	ldr	r1, .L1562+20
	movw	r2, #931
	ldr	r0, .L1562+24
	bl	printk
	bl	dump_stack
.L1528:
	mov	r0, r4
	bl	ftl_free_sblk
	ldr	r3, [r7, #580]
	ldrh	r1, [r3, #124]
	cmp	r1, #0
	beq	.L1529
	add	r0, r3, #388
	mov	r2, #0
	add	r0, r0, #2
.L1531:
	ldrh	ip, [r0, #2]!
	cmp	ip, r4
	bne	.L1530
	add	r2, r2, #196
	mvn	r0, #0
	add	r1, r1, r0
	mov	r2, r2, asl #1
	strh	r0, [r3, r2]	@ movhi
	strh	r1, [r3, #124]	@ movhi
	b	.L1529
.L1530:
	add	r2, r2, #1
	cmp	r2, #64
	bne	.L1531
.L1529:
	ldrh	r1, [r3, #120]
	cmp	r1, #0
	beq	.L1532
	add	r0, r3, #134
	mov	r2, #0
.L1534:
	ldrh	ip, [r0, #2]!
	cmp	ip, r4
	bne	.L1533
	add	r2, r3, r2, asl #1
	mvn	r0, #0
	add	r1, r1, r0
	strh	r0, [r2, #136]	@ movhi
	strh	r1, [r3, #120]	@ movhi
	b	.L1532
.L1533:
	add	r2, r2, #1
	cmp	r2, #64
	bne	.L1534
.L1532:
	ldrh	r0, [r3, #122]
	cmp	r0, #0
	beq	.L1535
	add	r1, r3, #260
	mov	r2, #0
	add	r1, r1, #2
.L1537:
	ldrh	ip, [r1, #2]!
	cmp	ip, r4
	bne	.L1536
	add	r2, r2, #132
	mvn	r1, #0
	add	r0, r0, r1
	mov	r2, r2, asl #1
	strh	r1, [r3, r2]	@ movhi
	strh	r0, [r3, #122]	@ movhi
	b	.L1535
.L1536:
	add	r2, r2, #1
	cmp	r2, #64
	bne	.L1537
	b	.L1535
.L1526:
	mov	r0, r4
	mov	r1, #1
	bl	gc_add_sblk
.L1535:
	add	r5, r5, #1
	b	.L1524
.L1561:
	ldr	r3, .L1562+4
	mov	r2, #0
	strh	r2, [r3, #52]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L1563:
	.align	2
.L1562:
	.word	.LANCHOR0
	.word	.LANCHOR0+3372
	.word	.LC116
	.word	.LANCHOR2
	.word	.LC117
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
	movw	ip, #3326
	ldr	r4, .L1566
	.pad #32
	sub	sp, sp, #32
	ldr	r3, [r4, #3340]
	ldrh	ip, [r4, ip]
	ldr	r0, [r4, #580]
	ldrh	r1, [r3, #148]
	ldrh	r2, [r3, #146]
	movw	r3, #3324
	str	ip, [sp]
	add	ip, r4, #3328
	ldrh	r3, [r4, r3]
	ldrh	ip, [ip]
	str	ip, [sp, #4]
	movw	ip, #3330
	ldrh	ip, [r4, ip]
	str	ip, [sp, #8]
	movw	ip, #3332
	ldrh	ip, [r4, ip]
	str	ip, [sp, #12]
	movw	ip, #3334
	ldrh	ip, [r4, ip]
	str	ip, [sp, #16]
	ldr	ip, [r0, #524]
	str	ip, [sp, #20]
	ldr	r0, [r0, #528]
	str	r0, [sp, #24]
	ldr	r0, [r4, #3320]
	str	r0, [sp, #28]
	ldr	r0, .L1566+4
	bl	printk
	ldr	r2, [r4, #3340]
	ldrb	r3, [r4, #3337]	@ zero_extendqisi2
	ldr	r0, .L1566+8
	ldr	r1, [r2, #8]
	str	r1, [sp]
	ldr	r1, [r2, #64]
	str	r1, [sp, #4]
	ldr	r1, [r2, #20]
	ldr	r2, [r2, #28]
	bl	printk
	ldr	r2, [r4, #3340]
	ldr	r0, .L1566+12
	ldr	r3, [r2, #16]
	ldr	r1, [r2, #52]
	ldr	r2, [r2, #60]
	mov	r3, r3, lsr #11
	bl	printk
	ldr	r2, [r4, #3340]
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
	ldr	r0, .L1566+16
	ldr	r2, [r2, #84]
	bl	printk
	movw	r0, #3352
	ldrh	r0, [r4, r0]
	movw	r3, #3346
	ldrh	r1, [r4, r3]
	movw	r3, #3348
	ldrh	r2, [r4, r3]
	movw	r3, #3350
	ldrh	r3, [r4, r3]
	str	r0, [sp]
	add	r0, r4, #3344
	ldrh	r0, [r0]
	str	r0, [sp, #4]
	ldr	r0, .L1566+20
	bl	printk
	ldr	r0, [r4, #580]
	movw	ip, #590
	add	r3, r0, #584
	ldrh	r1, [r3]
	movw	r3, #586
	ldrh	r2, [r0, r3]
	add	r3, r0, #588
	ldrh	r0, [r0, ip]
	ldrh	r3, [r3]
	str	r0, [sp]
	ldr	r0, .L1566+24
	bl	printk
	add	sp, sp, #32
	@ sp needed
	ldmfd	sp!, {r4, pc}
.L1567:
	.align	2
.L1566:
	.word	.LANCHOR0
	.word	.LC118
	.word	.LC119
	.word	.LC120
	.word	.LC121
	.word	.LC122
	.word	.LC123
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
	bne	.L1569
	ldr	r1, .L1580
	mov	r2, #668
	ldr	r0, .L1580+4
	bl	printk
	bl	dump_stack
	bl	print_ftl_debug_info
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L1569:
	ldr	r5, .L1580+8
	ldrb	r2, [r4, #40]	@ zero_extendqisi2
	ldrb	r3, [r5, #3312]	@ zero_extendqisi2
	cmp	r2, r3
	bls	.L1574
	ldr	r1, .L1580
	movw	r2, #673
	ldr	r0, .L1580+4
	bl	printk
	bl	dump_stack
.L1574:
	ldrb	r3, [r4, #40]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L1571
	ldrb	r2, [r5, #3312]	@ zero_extendqisi2
	ldr	r6, .L1580+8
	cmp	r2, r3
	bcs	.L1572
.L1571:
	mov	r0, r4
	bl	zbuf_free
	ldrb	r0, [r5, #3336]	@ zero_extendqisi2
	ldmfd	sp!, {r4, r5, r6, pc}
.L1572:
	add	r0, r6, #3360
	mov	r1, r4
	bl	buf_add_tail
	ldr	r3, [r6, #3340]
	ldrb	r1, [r4, #40]	@ zero_extendqisi2
	ldrb	r0, [r6, #3336]	@ zero_extendqisi2
	ldr	r2, [r3, #16]
	add	r0, r0, #1
	add	r2, r2, r1
	str	r2, [r3, #16]
	ldr	r2, [r3, #32]
	uxtb	r0, r0
	strb	r0, [r6, #3336]
	add	r2, r2, #1
	str	r2, [r3, #32]
	ldmfd	sp!, {r4, r5, r6, pc}
.L1581:
	.align	2
.L1580:
	.word	.LANCHOR1+1360
	.word	.LC0
	.word	.LANCHOR0
	.fnend
	.size	ftl_write_buf, .-ftl_write_buf
	.align	2
	.global	ftl_write_completed
	.type	ftl_write_completed, %function
ftl_write_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r5, #0
	ldr	r6, .L1594
	ldr	r8, .L1594+4
	add	r9, r6, #1776
.L1583:
	ldrb	fp, [r8, #-3842]	@ zero_extendqisi2
	cmp	fp, #255
	beq	.L1593
	mov	r4, #48
	ldr	r10, .L1594
	mul	r4, r4, fp
	add	r7, r9, r4
	add	r4, r6, r4
	ldrb	r3, [r4, #1776]	@ zero_extendqisi2
	strb	r3, [r8, #-3842]
	ldr	r3, [r4, #1812]
	cmn	r3, #1
	bne	.L1584
	ldr	r3, .L1594+8
	mvn	r0, #0
	ldrh	r2, [r3]
	ldr	r3, [r4, #1800]
	rsb	r1, r2, #21
	mov	r3, r3, lsr r2
	bic	r0, r3, r0, asl r1
	ldrb	r1, [r6, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r3, [r6, #580]
	movw	r2, #65535
	ldr	r1, [r3, #560]
	cmp	r1, r2
	uxth	r5, r0
	streq	r5, [r3, #560]
	ldreq	r2, [r4, #1800]
	ldr	r0, .L1594+12
	mov	r1, r5
	streq	r2, [r3, #564]
	mov	r3, #48
	mla	r3, r3, fp, r10
	ldr	r2, [r3, #1796]
	ldr	r3, [r3, #1800]
	bl	printk
	ldr	r3, [r10, #580]
	ldr	r2, [r3, #556]
	add	r2, r2, #1
	str	r2, [r3, #556]
	ldrh	r2, [r3, #16]
	cmp	r2, r5
	moveq	r2, #0
	streqh	r2, [r3, #22]	@ movhi
	beq	.L1587
	ldrh	r2, [r3, #48]
	cmp	r2, r5
	moveq	r2, #0
	streqh	r2, [r3, #54]	@ movhi
.L1587:
	mov	r0, r7
	mov	r5, #1
	bl	ftl_write_buf
	b	.L1583
.L1584:
	cmp	r5, #1
	bne	.L1589
	ldr	r1, [r4, #1796]
	ldr	r2, [r4, #1800]
	ldr	r0, .L1594+16
	bl	printk
	mov	r0, r7
	bl	ftl_write_buf
	b	.L1583
.L1589:
	ldr	r0, [r4, #1796]
	ldr	r1, [r4, #1804]
	ldrh	r2, [r7, #32]
	bl	lpa_hash_update_ppa
	ldrb	r3, [r4, #1778]	@ zero_extendqisi2
	and	r5, r3, #4
	ands	r5, r5, #255
	bicne	r3, r3, #2
	strneb	r3, [r4, #1778]
	movne	r5, #0
	bne	.L1583
	mov	r0, r7
	bl	zbuf_free
	b	.L1583
.L1593:
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1595:
	.align	2
.L1594:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+3764
	.word	.LC124
	.word	.LC125
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
	bne	.L1597
	ldr	r1, .L1602
	mov	r2, #964
	ldr	r0, .L1602+4
	bl	printk
	bl	dump_stack
	ldmfd	sp!, {r3, r4, r5, lr}
	b	print_ftl_debug_info
.L1597:
	ldr	r3, .L1602+8
	ldrb	r2, [r4, #40]	@ zero_extendqisi2
	ldrb	r3, [r3, #3312]	@ zero_extendqisi2
	cmp	r2, r3
	bls	.L1599
	ldr	r1, .L1602
	movw	r2, #969
	ldr	r0, .L1602+4
	bl	printk
	bl	dump_stack
.L1599:
	ldr	r5, .L1602+12
	mov	r1, r4
	ldr	r0, .L1602+16
	bl	buf_add_tail
	ldrb	r3, [r5, #-3840]	@ zero_extendqisi2
	add	r3, r3, #1
	strb	r3, [r5, #-3840]
	ldmfd	sp!, {r3, r4, r5, pc}
.L1603:
	.align	2
.L1602:
	.word	.LANCHOR1+1376
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR3-3841
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
	beq	.L1606
	ldr	r3, .L1615
	ldr	r6, [r3, #568]
	add	r6, r6, r0, asl #2
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	tst	r3, #224
	beq	.L1607
	ldr	r1, .L1615+4
	movw	r2, #833
	ldr	r0, .L1615+8
	bl	printk
	bl	dump_stack
.L1607:
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	bfi	r3, r5, #5, #3
	uxtb	r3, r3
	ubfx	r2, r3, #3, #2
	orr	r2, r7, r2
	bfi	r3, r2, #3, #2
	strb	r3, [r6, #2]
	b	.L1614
.L1606:
	bl	print_ftl_debug_info
	mov	r1, r4
	mov	r2, r5
	ldr	r0, .L1615+12
	bl	printk
	mov	r1, r6
	mov	r0, #0
	bl	zftl_get_free_sblk
	mov	r2, r5
	mov	r4, r0
	ldr	r0, .L1615+12
	mov	r1, r4
	bl	printk
	bl	dump_all_list_info
	ldr	r1, .L1615+4
	movw	r2, #843
	ldr	r0, .L1615+8
	bl	printk
	bl	dump_stack
.L1614:
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L1616:
	.align	2
.L1615:
	.word	.LANCHOR0
	.word	.LANCHOR1+1396
	.word	.LC0
	.word	.LC126
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
	ldr	r2, .L1618
	mvn	r3, #0
	ldr	r1, .L1618+4
	mov	r0, #0
	strb	r3, [r2, #3315]
	strb	r3, [r1, #-3842]
	strb	r3, [r1, #-3839]
	strb	r3, [r2, #3767]
	bx	lr
.L1619:
	.align	2
.L1618:
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
	ldr	r5, .L1630
	ldr	r0, .L1630+4
	ldrb	r1, [r5, #3315]	@ zero_extendqisi2
	bl	printk
	ldrb	r4, [r5, #3315]	@ zero_extendqisi2
	cmp	r4, #255
	ldmeqfd	sp!, {r4, r5, r6, pc}
	add	r5, r5, #1776
	mov	r3, #48
	mov	r6, r3
	mla	r4, r3, r4, r5
.L1622:
	ldrb	r1, [r4, #1]	@ zero_extendqisi2
	ldrb	r2, [r4, #42]	@ zero_extendqisi2
	ldr	r3, [r4, #24]
	ldr	r0, .L1630+8
	bl	printk
	ldrb	r4, [r4]	@ zero_extendqisi2
	cmp	r4, #255
	ldmeqfd	sp!, {r4, r5, r6, pc}
	mla	r4, r6, r4, r5
	b	.L1622
.L1631:
	.align	2
.L1630:
	.word	.LANCHOR0
	.word	.LC127
	.word	.LC128
	.fnend
	.size	dump_sblk_queue, .-dump_sblk_queue
	.align	2
	.global	queue_lun_state
	.type	queue_lun_state, %function
queue_lun_state:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1649
	ldrb	r3, [r2, #3315]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L1640
	movw	ip, #3764
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	ubfx	r7, r0, #21, #3
	ldrh	r6, [r2, ip]
	mvn	ip, #0
	mov	r8, #48
	rsb	lr, r6, #21
	mvn	ip, ip, asl lr
	ldrb	lr, [r2, #3766]	@ zero_extendqisi2
	uxth	ip, ip
	sub	lr, lr, #1
	uxth	lr, lr
	and	r5, lr, r0, asr r6
	and	r5, r5, ip
.L1639:
	mla	r0, r8, r3, r2
	ldr	r4, [r0, #1800]
	ubfx	r9, r4, #21, #3
	cmp	r9, r7
	bne	.L1634
	and	r4, lr, r4, lsr r6
	ldrb	r0, [r0, #1818]	@ zero_extendqisi2
	and	r4, r4, ip
	cmp	r4, r5
	bne	.L1635
	cmp	r1, #1
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, pc}
	sub	r4, r0, #7
	tst	r4, #253
	beq	.L1634
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L1635:
	cmp	r1, #3
	ldrls	pc, [pc, r1, asl #2]
	b	.L1634
.L1636:
	.word	.L1633
	.word	.L1637
	.word	.L1638
	.word	.L1633
.L1637:
	sub	r4, r0, #7
	tst	r4, #253
	beq	.L1634
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L1638:
	cmp	r0, #11
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L1634:
	mla	r3, r8, r3, r2
	ldrb	r3, [r3, #1776]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L1639
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L1640:
	mov	r0, #0
	bx	lr
.L1633:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L1650:
	.align	2
.L1649:
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
	ldr	r3, .L1678
	mov	ip, #0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #12
	add	r10, r3, #1776
	ldr	r1, .L1678+4
	mov	r8, r3
	ldrb	r0, [r3, #3767]	@ zero_extendqisi2
	mvn	r9, #0
	ldrb	r2, [r3, #3315]	@ zero_extendqisi2
	mov	r4, r3
	ldrb	r6, [r1, #-3839]	@ zero_extendqisi2
	ldrb	r7, [r1, #-3842]	@ zero_extendqisi2
	str	r0, [sp, #4]
	mov	r0, #48
	mov	r5, r0
.L1652:
	cmp	r2, #255
	beq	.L1653
	mla	fp, r0, r2, r3
	mov	r1, r2
	ldrb	lr, [fp, #1818]	@ zero_extendqisi2
	sub	lr, lr, #12
	cmp	lr, #1
	bls	.L1654
	cmp	ip, #0
	ldrne	r3, .L1678
	beq	.L1651
	b	.L1676
.L1654:
	mul	ip, r0, r1
	ldrb	r2, [fp, #1776]	@ zero_extendqisi2
	add	lr, r8, ip
	ldrb	fp, [lr, #1819]	@ zero_extendqisi2
	strb	r9, [lr, #1776]
	cmp	fp, #1
	bne	.L1657
	add	ip, r10, ip
	ldrh	ip, [ip, #34]
	cmp	ip, #0
	moveq	ip, r7
	ldreq	lr, .L1678+8
	ldrne	ip, [sp, #4]
	ldrne	lr, .L1678+12
	b	.L1658
.L1657:
	cmp	fp, #0
	beq	.L1677
.L1659:
	mov	ip, #1
	b	.L1652
.L1677:
	ldr	ip, [lr, #1796]
	cmn	ip, #1
	movne	ip, r6
	ldrne	lr, .L1678+16
	beq	.L1659
.L1658:
	cmp	ip, #255
	bne	.L1663
	strb	r2, [r3, #3315]
	mov	r2, #48
	mla	r1, r2, r1, r3
	ldrb	r3, [r1, #1777]	@ zero_extendqisi2
	strb	r3, [lr]
	b	.L1651
.L1663:
	mov	lr, ip
	mla	ip, r0, ip, r3
	ldrb	ip, [ip, #1776]	@ zero_extendqisi2
	cmp	ip, #255
	bne	.L1663
	mla	r1, r5, r1, r4
	mla	lr, r5, lr, r4
	ldrb	r1, [r1, #1777]	@ zero_extendqisi2
	strb	r1, [lr, #1776]
	b	.L1659
.L1653:
	cmp	ip, #0
	beq	.L1651
	mvn	r2, #0
.L1676:
	strb	r2, [r3, #3315]
.L1651:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1679:
	.align	2
.L1678:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR3-3842
	.word	.LANCHOR0+3767
	.word	.LANCHOR3-3839
	.fnend
	.size	queue_remove_completed_req, .-queue_remove_completed_req
	.align	2
	.global	pm_free_sblk
	.type	pm_free_sblk, %function
pm_free_sblk:
	.fnstart
	@ args = 0, pretend = 0, frame = 272
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L1704
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #284
	sub	sp, sp, #284
	ldrh	r2, [r3, #-14]
	mov	r6, r3
	cmp	r2, #128
	bls	.L1681
	ldr	r1, .L1704+4
	mov	r2, #78
	ldr	r0, .L1704+8
	bl	printk
	bl	dump_stack
.L1681:
	add	r5, sp, #24
	mov	r1, #256
	ldr	r4, .L1704+12
	mvn	r8, #0
	mov	r0, r5
	bl	__memzero
	movw	r3, #698
	ldr	r7, [r4, #3340]
	ldrb	fp, [r4, #3766]	@ zero_extendqisi2
	add	r10, r7, #700
	ldrh	ip, [r6, #-14]
	ldrh	r3, [r7, r3]
	mov	r6, #0
	str	r3, [sp, #12]
	movw	r3, #3764
	ldrh	r9, [r4, r3]
	rsb	r3, r9, #21
	mvn	r8, r8, asl r3
.L1682:
	ldr	r2, [sp, #12]
	uxth	r3, r6
	cmp	r3, r2
	bcs	.L1701
	ldr	r0, [r10, #4]!
	mov	r1, fp
	str	ip, [sp, #16]
	and	r0, r8, r0, lsr r9
	bl	__aeabi_uidiv
	add	r2, r7, #412
	add	r2, r2, #2
	mov	r3, r5
	mov	r1, #0
	ldr	ip, [sp, #16]
	uxth	r0, r0
.L1683:
	uxth	lr, r1
	cmp	lr, ip
	bcs	.L1702
	ldrh	lr, [r2, #2]!
	add	r1, r1, #1
	add	r3, r3, #2
	cmp	lr, r0
	ldreqh	lr, [r3, #-2]
	addeq	lr, lr, #1
	streqh	lr, [r3, #-2]	@ movhi
	b	.L1683
.L1702:
	add	r6, r6, #1
	b	.L1682
.L1701:
	ldr	r9, .L1704+16
	mov	r10, #0
	ldrb	r8, [r4, #3720]	@ zero_extendqisi2
	mov	ip, r10
	ldr	fp, .L1704+20
	ldrh	r6, [r9]
	smulbb	r8, r8, r6
	uxth	r7, r8
.L1687:
	ldr	r3, .L1704+24
	uxth	r6, r10
	ldrh	r3, [r3]
	cmp	r3, r6
	bls	.L1703
	add	r3, r6, #208
	ldr	r2, [r4, #3340]
	str	ip, [sp, #20]
	mov	r8, r6
	mov	r3, r3, asl #1
	add	r1, r2, #692
	str	r2, [sp, #12]
	ldrh	r3, [r2, r3]
	ldrh	r0, [r1]
	ldrb	r1, [r4, #3766]	@ zero_extendqisi2
	str	r3, [sp, #16]
	bl	__aeabi_idiv
	ldr	r3, [sp, #16]
	ldr	ip, [sp, #20]
	cmp	r0, r3
	mov	r0, r6, asl #1
	ldr	r2, [sp, #12]
	ldreq	r1, .L1704+12
	ldreqb	lr, [r1, #3720]	@ zero_extendqisi2
	ldreqh	r1, [r9]
	smulbbeq	r1, lr, r1
	streqh	r1, [r5, r0]	@ movhi
	ldrh	r1, [r5, r0]
	cmp	r1, #0
	cmpne	r7, r1
	movhi	r0, #1
	movls	r0, #0
	cmp	r0, #0
	movw	r0, #65535
	movne	r7, r1
	clz	r1, r1
	moveq	r6, ip
	cmp	r3, r0
	mov	r1, r1, lsr #5
	moveq	r1, #0
	cmp	r1, #0
	beq	.L1690
	ldr	r1, [fp]
	tst	r1, #4096
	beq	.L1691
	add	r2, r2, #688
	ldr	r0, .L1704+28
	mov	r1, r8
	ldrh	r2, [r2]
	str	r2, [sp]
	mov	r2, #0
	bl	printk
.L1691:
	add	r8, r8, #208
	ldr	r3, [r4, #3340]
	mov	r8, r8, asl #1
	ldrh	r0, [r3, r8]
	bl	ftl_free_sblk
	ldr	r3, [r4, #3340]
	mvn	r2, #0
	strh	r2, [r3, r8]	@ movhi
	add	r3, r3, #688
	ldrh	r2, [r3]
	sub	r2, r2, #1
	strh	r2, [r3]	@ movhi
.L1690:
	add	r10, r10, #1
	mov	ip, r6
	b	.L1687
.L1703:
	mov	r0, ip
	add	sp, sp, #284
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1705:
	.align	2
.L1704:
	.word	.LANCHOR3-3824
	.word	.LANCHOR1+1412
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR0+3712
	.word	.LANCHOR2
	.word	.LANCHOR3-3838
	.word	.LC129
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
	ldr	r4, .L1723
	ldr	r2, [r4, #3340]
	ldrh	r3, [r2, r1]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r2, r1]	@ movhi
	ldrb	r1, [r4, #3720]	@ zero_extendqisi2
	cmp	r1, r3
	bls	.L1707
	add	r3, r3, #336
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L1708
.L1707:
	ldr	r5, .L1723
.L1720:
	mov	r0, #1
	bl	ftl_alloc_sblk
	mov	r1, #0
	mov	r6, r0
	bl	ftl_erase_sblk
	ldr	r1, [r4, #3340]
	mov	r0, r6
	add	r1, r1, #672
	bl	ftl_get_blk_list_in_sblk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L1709
	mov	r1, r6
	ldr	r0, .L1723+4
	bl	printk
	ldr	r3, [r5, #568]
	add	r6, r3, r6, asl #2
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	orr	r3, r3, #224
	strb	r3, [r6, #2]
	b	.L1720
.L1709:
	ldr	r3, .L1723
	movw	r2, #690
	mov	r5, #0
	mov	r1, #1
	ldr	r3, [r3, #3340]
	strh	r5, [r3, r2]	@ movhi
	add	r3, r3, #412
	ldr	r2, .L1723+8
	add	r3, r3, #2
	str	r1, [r2, #-3836]
	movw	r1, #65535
.L1711:
	ldrh	r0, [r3, #2]!
	uxth	r2, r5
	cmp	r0, r1
	beq	.L1710
	add	r5, r5, #1
	cmp	r5, #128
	bne	.L1711
	mov	r2, #194
	ldr	r1, .L1723+12
	ldr	r0, .L1723+16
	bl	printk
	bl	dump_stack
	mov	r2, r5
.L1710:
	add	r2, r2, #208
	ldr	r3, [r4, #3340]
	mov	r2, r2, asl #1
	strh	r6, [r3, r2]	@ movhi
	add	r3, r3, #688
	ldrh	r2, [r3]
	add	r2, r2, #1
	strh	r2, [r3]	@ movhi
.L1708:
	ldr	r2, [r4, #3340]
	movw	r3, #690
	ldrh	r3, [r2, r3]
	add	r3, r3, #336
	mov	r3, r3, asl #1
	ldrh	r5, [r2, r3]
	movw	r2, #65533
	sub	r3, r5, #1
	uxth	r3, r3
	cmp	r3, r2
	bls	.L1713
	ldr	r1, .L1723+12
	mov	r2, #199
	ldr	r0, .L1723+16
	bl	printk
	bl	dump_stack
.L1713:
	ldr	r3, [r4, #3340]
	mov	r1, #0
	movw	r0, #694
	add	r2, r3, #696
	strh	r1, [r2]	@ movhi
	movw	r2, #3764
	ldrh	r2, [r4, r2]
	rsb	r2, r2, #21
	mov	r1, r5, asr r2
	strh	r1, [r3, r0]	@ movhi
	add	r3, r3, #692
	strh	r5, [r3]	@ movhi
	ldr	r3, .L1723+20
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L1717
	mvn	r3, #0
	uxth	r1, r1
	mvn	r2, r3, asl r2
	str	r1, [sp]
	ldr	r0, .L1723+24
	mov	r1, r5
	mov	r3, r5
	bl	printk
.L1717:
	mov	r0, #0
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, pc}
.L1724:
	.align	2
.L1723:
	.word	.LANCHOR0
	.word	.LC130
	.word	.LANCHOR3
	.word	.LANCHOR1+1428
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC131
	.fnend
	.size	pm_alloc_new_blk, .-pm_alloc_new_blk
	.align	2
	.global	pm_select_ram_region
	.type	pm_select_ram_region, %function
pm_select_ram_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L1740
	mov	r3, #0
	movw	ip, #65535
.L1727:
	mov	r1, r3, asl #3
	uxth	r0, r3
	ldrh	r1, [r1, r2]
	cmp	r1, ip
	ldr	r1, .L1740
	bxeq	lr
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L1727
	mov	r2, #0
	mov	r0, r3
	mov	ip, #32768
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
.L1729:
	add	r3, r1, r2, asl #3
	uxth	lr, r2
	ldrh	r3, [r3, #2]
	tst	r3, #32768
	bne	.L1728
	cmp	r3, ip
	movcc	ip, r3
	movcc	r0, lr
.L1728:
	add	r2, r2, #1
	cmp	r2, #32
	bne	.L1729
	cmp	r0, #32
	mov	r4, r0
	ldmnefd	sp!, {r4, r5, r6, pc}
	ldr	r3, .L1740+4
	mvn	r2, #0
	ldr	r5, .L1740
	ldrb	lr, [r3, #-3576]	@ zero_extendqisi2
	mov	r3, #0
.L1731:
	mov	ip, r3, asl #3
	uxth	r6, r3
	add	r0, r1, ip
	ldrh	r0, [r0, #2]
	cmp	r0, r2
	bcs	.L1730
	ldrh	ip, [ip, r5]
	cmp	ip, lr
	movne	r2, r0
	movne	r4, r6
.L1730:
	add	r3, r3, #1
	cmp	r3, #32
	bne	.L1731
	cmp	r4, #32
	bne	.L1732
	ldr	r1, .L1740+8
	movw	r2, #297
	ldr	r0, .L1740+12
	bl	printk
	bl	dump_stack
.L1732:
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1741:
	.align	2
.L1740:
	.word	.LANCHOR3-3832
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
	beq	.L1743
	bl	memset
.L1743:
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
	ldr	r4, .L1750
	mov	r2, #512
	add	r0, r4, #3808
	add	r0, r0, #4
	bl	memset
	add	r3, r4, #3712
	mov	r1, #255
	ldrh	r2, [r3]
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	mul	r2, r3, r2
	ldr	r3, .L1750+4
	ldr	r0, [r3, #-3856]
	mov	r2, r2, asl #2
	ldmfd	sp!, {r4, lr}
	b	ftl_memset
.L1751:
	.align	2
.L1750:
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
	ldr	r3, .L1765
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L1753
	ldr	r0, .L1765+4
	mov	r2, #224
	ldr	r1, .L1765+8
	mov	r3, #0
	bl	printk
.L1753:
	ldr	r4, .L1765+12
	mov	r1, #255
	mov	r2, #512
	ldr	r0, .L1765+16
	add	r5, r4, #3712
	bl	memset
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	mov	r1, #255
	ldrh	r2, [r5]
	ldr	r6, .L1765+20
	mul	r2, r3, r2
	ldr	r0, [r6, #-3856]
	mov	r2, r2, asl #2
	bl	ftl_memset
	add	ip, r5, #100
	mov	r0, #0
	mov	r1, r4
	mov	lr, r6
.L1754:
	ldrh	r4, [r5]
	uxth	r2, r0
	ldrb	r3, [r1, #3720]	@ zero_extendqisi2
	mul	r3, r3, r4
	cmp	r2, r3, asl #1
	bge	.L1764
	ldr	r3, [r6, #-3860]
	ldr	r3, [r3, r2, asl #2]
	cmn	r3, #1
	beq	.L1755
	uxtb	r3, r3
	mov	r3, r3, asl #1
	ldrh	r4, [ip, r3]
	strh	r2, [ip, r3]	@ movhi
	mov	r2, r2, asl #1
	ldr	r3, [lr, #-3856]
	strh	r4, [r3, r2]	@ movhi
.L1755:
	add	r0, r0, #1
	b	.L1754
.L1764:
	ldmfd	sp!, {r4, r5, r6, pc}
.L1766:
	.align	2
.L1765:
	.word	.LANCHOR2
	.word	.LC132
	.word	.LANCHOR1+1472
	.word	.LANCHOR0
	.word	.LANCHOR0+3812
	.word	.LANCHOR3
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
	ldr	r6, .L1798
	mov	r1, #0
	.pad #20
	sub	sp, sp, #20
	mov	r9, #0
	add	r8, r6, #564
	add	r7, r6, #3328
	ldr	r0, [r6, #524]
	mov	r4, r6
	ldrh	r3, [r8]
	ldr	fp, .L1798+4
	mul	r2, r2, r3
	bl	ftl_memset
	movw	r3, #3800
	mov	r2, #16
	strh	r2, [r6, r3]	@ movhi
	movw	r3, #3324
	movw	r2, #3722
	strh	r9, [r6, r3]	@ movhi
	movw	r3, #3326
	strh	r9, [r6, r3]	@ movhi
	movw	r3, #3330
	strh	r9, [r6, r3]	@ movhi
	movw	r3, #3334
	strh	r9, [r6, r3]	@ movhi
	movw	r3, #3332
	strh	r9, [r6, r3]	@ movhi
	mov	r0, #32768
	ldrb	r3, [r6, #3720]	@ zero_extendqisi2
	ldrh	r1, [r6, r2]
	strh	r9, [r7]	@ movhi
	str	r9, [r6, #3788]
	mul	r1, r1, r3
	str	r9, [r6, #3792]
	str	r9, [r6, #3796]
	str	r9, [r6, #3772]
	str	r9, [r6, #3780]
	str	r9, [r6, #3776]
	bl	__aeabi_idiv
	uxth	r3, r0
	str	r3, [sp]
	ldr	r3, [r6, #3340]
	str	r7, [sp, #8]
	mov	r7, r6
	str	r8, [sp, #4]
	ldrh	r5, [r3, #134]
	strh	r9, [r3, #146]	@ movhi
.L1768:
	ldr	r3, [sp, #4]
	sxth	r2, r5
	ldr	r1, .L1798
	ldrh	r3, [r3]
	cmp	r2, r3
	bge	.L1796
	ldr	r3, [r7, #568]
	add	r8, r3, r2, asl #2
	ldrb	r1, [r8, #3]	@ zero_extendqisi2
	cmp	r1, #0
	ldreq	r1, [sp]
	beq	.L1769
	ldrb	r6, [r7, #3720]	@ zero_extendqisi2
	mov	ip, #0
	ldrh	r9, [fp]
	mov	r1, ip
	ldr	lr, [r7, #3340]
.L1770:
	cmp	ip, r6
	bge	.L1797
	ldrb	r0, [r8, #3]	@ zero_extendqisi2
	mov	r0, r0, asr ip
	add	ip, ip, #1
	tst	r0, #1
	addeq	r1, r9, r1
	ldrneh	r0, [lr, #146]
	uxtheq	r1, r1
	addne	r0, r0, #1
	strneh	r0, [lr, #146]	@ movhi
	b	.L1770
.L1797:
	cmp	r1, #0
	beq	.L1774
	sxth	r1, r1
	mov	r0, #32768
	str	r2, [sp, #12]
	bl	__aeabi_idiv
	add	r0, r0, #1
	ldr	r2, [sp, #12]
	uxth	r1, r0
	b	.L1769
.L1774:
	ldrb	r0, [r8, #2]	@ zero_extendqisi2
	mvn	lr, #0
	orr	r0, r0, #224
	strb	r0, [r8, #2]
	ldr	ip, [r4, #576]
	mov	r0, r2, asl #1
	strh	lr, [ip, r0]	@ movhi
.L1769:
	mov	r10, r2, asl #1
	ldr	lr, [r4, #524]
	add	r0, r10, r2
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
	moveq	r9, #1
	movne	r9, #0
	beq	.L1776
	ldr	r0, [r7, #580]
	ldrh	lr, [r0, #16]
	cmp	r2, lr
	beq	.L1776
	ldrh	lr, [r0, #48]
	cmp	r2, lr
	beq	.L1776
	ldrh	r0, [r0, #80]
	cmp	r2, r0
	beq	.L1776
	cmp	r6, #64
	uxtheq	r6, r5
	ldreq	r0, .L1798+8
	ldreq	r2, .L1798+12
	moveq	r1, r6
	beq	.L1794
.L1778:
	cmp	r6, #96
	uxtheq	r6, r5
	ldreq	r0, .L1798+16
	moveq	r1, r6
	subeq	r2, r0, #440
	beq	.L1794
.L1779:
	cmp	r6, #160
	bne	.L1780
	uxth	r6, r5
	ldr	r0, .L1798+20
	ldr	r2, .L1798+24
	mov	r1, r6
.L1794:
	bl	_insert_data_list
	ldr	r3, [r7, #576]
	ldrh	r3, [r3, r10]
	cmp	r3, #7
	movls	r0, r6
	movls	r1, #1
	movls	r2, r9
	bhi	.L1776
	b	.L1792
.L1780:
	cmp	r6, #0
	bne	.L1776
	ldr	r0, [r7, #576]
	uxth	r9, r5
	ldrh	ip, [r0, r10]
	cmp	ip, #0
	beq	.L1781
	mov	r1, r2
	ldr	r0, .L1798+28
	mov	r2, ip
	bl	printk
	ldrb	r2, [r8, #2]	@ zero_extendqisi2
	mov	r0, r9
	tst	r2, #16
	movne	r1, #5
	moveq	r1, #2
	bfi	r2, r1, #5, #3
	mov	r1, #1
	strb	r2, [r8, #2]
	mov	r2, r6
.L1792:
	bl	gc_add_sblk
	b	.L1776
.L1781:
	ands	r1, r1, #24
	ldreq	r0, .L1798+32
	moveq	r1, r9
	subeq	r2, r0, #464
	beq	.L1790
.L1783:
	cmp	r1, #16
	ldrne	r0, .L1798+36
	moveq	r1, r9
	ldreq	r0, .L1798+40
	movne	r1, r9
	ldreq	r2, .L1798+44
	subne	r2, r0, #468
.L1790:
	bl	_insert_free_list
.L1776:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L1768
.L1796:
	movw	r2, #3324
	ldr	r3, [r1, #3340]
	ldrh	r2, [r1, r2]
	strh	r2, [r3, #114]	@ movhi
	movw	r2, #3326
	ldrh	r2, [r1, r2]
	strh	r2, [r3, #118]	@ movhi
	ldr	r2, [sp, #8]
	ldrh	r2, [r2]
	strh	r2, [r3, #116]	@ movhi
	movw	r2, #3330
	ldrh	r2, [r1, r2]
	strh	r2, [r3, #122]	@ movhi
	movw	r2, #3334
	ldrh	r2, [r1, r2]
	strh	r2, [r3, #120]	@ movhi
	movw	r2, #3332
	ldrh	r2, [r1, r2]
	strh	r2, [r3, #124]	@ movhi
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1799:
	.align	2
.L1798:
	.word	.LANCHOR0
	.word	.LANCHOR0+3722
	.word	.LANCHOR0+3780
	.word	.LANCHOR0+3330
	.word	.LANCHOR0+3772
	.word	.LANCHOR0+3776
	.word	.LANCHOR0+3334
	.word	.LC133
	.word	.LANCHOR0+3788
	.word	.LANCHOR0+3796
	.word	.LANCHOR0+3792
	.word	.LANCHOR0+3326
	.fnend
	.size	zftl_sblk_list_init, .-zftl_sblk_list_init
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
	ldr	r6, .L1807
	mov	r7, r1
	ldr	r9, .L1807+4
	mov	r8, r6
.L1801:
.L1804:
	mov	r0, r7
	bl	ftl_alloc_sblk
	mov	r1, #0
	mov	r5, r0
	bl	ftl_erase_sblk
	add	r1, r4, #16
	mov	r0, r5
	bl	ftl_get_blk_list_in_sblk
	ldr	r2, .L1807+8
	strh	r5, [r4]	@ movhi
	cmp	r7, #2
	strb	r7, [r4, #4]
	mov	r1, #255
	ldrh	r2, [r2]
	mov	r10, r5, asl #1
	uxtb	r3, r0
	strb	r3, [r4, #9]
	mov	r0, #0
	strh	r0, [r4, #2]	@ movhi
	smulbb	r3, r3, r2
	strb	r0, [r4, #5]
	strh	r0, [r4, #10]	@ movhi
	ldrneb	r0, [r6, #3720]	@ zero_extendqisi2
	strh	r3, [r4, #6]	@ movhi
	ldrb	r3, [r8, #3720]	@ zero_extendqisi2
	smulbbne	r0, r0, r2
	mul	r2, r3, r2
	ldr	r3, [r9, #-3860]
	uxthne	r0, r0
	strh	r0, [r4, #12]	@ movhi
	mov	r2, r2, asl #2
	add	r0, r3, r0, asl #2
	bl	ftl_memset
	ldr	r3, [r8, #576]
	ldrh	r2, [r4, #6]
	strh	r2, [r3, r10]	@ movhi
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	cmp	r3, #0
	ldmnefd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
	ldr	r0, .L1807+12
	mov	r1, r5
	bl	printk
	ldr	r3, [r6, #576]
	mvn	r2, #0
	strh	r2, [r3, r10]	@ movhi
	mov	r3, #7
	strb	r3, [r4, #4]
	b	.L1804
.L1808:
	.align	2
.L1807:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+3712
	.word	.LC130
	.fnend
	.size	ftl_open_sblk_init, .-ftl_open_sblk_init
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
	ldr	r3, .L1816
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L1811
	ldr	r0, .L1816+4
	mov	r1, #105
	ldr	r2, .L1816+8
	bl	printk
.L1811:
	ldr	r4, .L1816+12
	mov	r1, #2048
	ldr	r0, [r4, #1696]
	bl	__memzero
	ldr	r3, [r4, #1696]
	ldr	r2, .L1816+16
	ldr	r1, .L1816+20
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
	ldr	r1, .L1816+24
	mov	r2, #32
	add	r0, r0, #48
	ldmfd	sp!, {r4, lr}
	b	ftl_memcpy
.L1817:
	.align	2
.L1816:
	.word	.LANCHOR2
	.word	.LC134
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
.L1819:
	cmp	r3, r2
	ldrne	ip, [r1, r3, asl #2]
	strne	ip, [r0, r3, asl #2]
	addne	r3, r3, #1
	bne	.L1819
.L1821:
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
	ldr	r3, .L1824
	ldr	r0, [r3]
	ldr	r3, .L1824+4
	ldr	r3, [r3, #-3568]
	rsb	r0, r3, r0
	b	jiffies_to_msecs
.L1825:
	.align	2
.L1824:
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
	ldr	r3, .L1828
	mov	r2, r4
	mov	r1, #1
	mov	r0, r5
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #12]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1829:
	.align	2
.L1828:
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
	ldr	r3, .L1832
	mov	r2, r4
	mov	r1, #1
	mov	r0, r5
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #16]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, pc}
.L1833:
	.align	2
.L1832:
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
	ldr	r3, .L1836
	mov	r2, r4
	mov	r1, r5
	mov	r0, r6
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #4]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1837:
	.align	2
.L1836:
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
	ldr	r3, .L1840
	mov	r2, r4
	mov	r1, r5
	mov	r0, r6
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #8]
	blx	r3
	mov	r4, r0
	bl	rknand_device_unlock
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, pc}
.L1841:
	.align	2
.L1840:
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
	ldr	ip, .L1846
	cmp	r2, #0
	moveq	r2, r3
	ldr	ip, [ip, #-3556]
	add	ip, ip, #4096
	add	r1, ip, r1
	beq	.L1845
	str	lr, [sp, #-4]!
	.save {lr}
	mov	lr, r0
	mov	r0, r1
	mov	r1, lr
	ldr	lr, [sp], #4
	mov	r2, r3
.L1845:
	b	ftl_memcpy
.L1847:
	.align	2
.L1846:
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
	ldr	r0, .L1851
	bl	printk
	ldr	r3, .L1851+4
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #40]
	ldmfd	sp!, {r4, lr}
	bx	r3	@ indirect register sibling call
.L1852:
	.align	2
.L1851:
	.word	.LC135
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
	ldr	r3, .L1854
	mov	r0, #0
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #32]
	bx	r3	@ indirect register sibling call
.L1855:
	.align	2
.L1854:
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
	ldr	r3, .L1857
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #44]
	bx	r3	@ indirect register sibling call
.L1858:
	.align	2
.L1857:
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
	ldr	r3, .L1860
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #48]
	bx	r3	@ indirect register sibling call
.L1861:
	.align	2
.L1860:
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
	ldr	r3, .L1863
	mov	r0, #0
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #36]
	bx	r3
.L1864:
	.align	2
.L1863:
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
	ldr	r3, .L1866
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #60]
	bx	r3
.L1867:
	.align	2
.L1866:
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
	ldr	r3, .L1869
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #64]
	bx	r3
.L1870:
	.align	2
.L1869:
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
	ldr	ip, .L1872
	ldr	ip, [ip, #-3560]
	ldr	ip, [ip, #20]
	bx	ip
.L1873:
	.align	2
.L1872:
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
	ldr	r3, .L1875
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #28]
	bx	r3
.L1876:
	.align	2
.L1875:
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
	ldr	r3, .L1878
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #52]
	bx	r3
.L1879:
	.align	2
.L1878:
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
	ldr	r3, .L1881
	ldr	r3, [r3, #-3560]
	ldr	r3, [r3, #56]
	bx	r3	@ indirect register sibling call
.L1882:
	.align	2
.L1881:
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
.L1884:
	cmp	r4, r8
	beq	.L1893
	cmp	r5, #0
	bne	.L1885
	ldr	r0, .L1894
	mov	r1, r9
	mov	r2, r6
	mov	r3, r4
	bl	printk
.L1885:
	cmp	r7, #4
	ldreq	r0, .L1894+4
	ldreq	r1, [r6, r4, asl #2]
	beq	.L1892
	cmp	r7, #2
	moveq	r3, r4, asl #1
	ldreq	r0, .L1894+4
	ldreqsh	r1, [r6, r3]
	ldrne	r0, .L1894+4
	ldrneb	r1, [r6, r4]	@ zero_extendqisi2
.L1892:
	add	r5, r5, #1
	bl	printk
	cmp	r5, #15
	bls	.L1889
	ldr	r0, .L1894+8
	mov	r5, #0
	ldr	r1, .L1894+12
	bl	printk
.L1889:
	add	r4, r4, #1
	b	.L1884
.L1893:
	ldr	r0, .L1894+8
	ldr	r1, .L1894+12
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, lr}
	b	printk
.L1895:
	.align	2
.L1894:
	.word	.LC136
	.word	.LC137
	.word	.LC1
	.word	.LC138
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
	ldr	r8, .L2007
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
	bne	.L1897
	mvn	r3, #88
	strb	r3, [r4, #128]
	ldr	r3, .L2007+4
	mvn	r1, #8
	mov	r6, #7
	strb	r1, [r3, #413]
	b	.L1958
.L1897:
	cmp	r0, #3
	bne	.L1899
	add	r1, r4, #127
	mov	r3, #176
.L1900:
	strb	r3, [r1, #1]!
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #184
	bne	.L1900
	b	.L2001
.L1899:
	cmp	r0, #4
	bne	.L1901
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
.L2001:
	mov	r6, #8
	mov	fp, r6
	b	.L1898
.L1901:
	cmp	r0, #5
	bne	.L1902
	mov	r3, #56
	mov	r6, #8
	strb	r3, [r4, #128]
	mov	r3, #57
	strb	r3, [r4, #129]
	mov	r3, #58
	strb	r3, [r4, #130]
	mov	r3, #59
	strb	r3, [r4, #131]
	b	.L1958
.L1902:
	cmp	r0, #6
	bne	.L1903
	mov	r3, #14
	mov	r6, #12
	strb	r3, [r4, #128]
	mov	r3, #15
	strb	r3, [r4, #129]
	mov	r3, #16
	strb	r3, [r4, #130]
	mov	r3, #17
	strb	r3, [r4, #131]
	b	.L1958
.L1903:
	cmp	r0, #7
	bne	.L1904
	add	r1, r4, #127
	mov	r3, #176
.L1905:
	strb	r3, [r1, #1]!
	add	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #184
	bne	.L1905
	mvn	r3, #43
	mov	r6, #12
	strb	r3, [r4, #136]
	mov	fp, #10
	mvn	r3, #42
	strb	r3, [r4, #137]
	b	.L1898
.L1904:
	cmp	r0, #8
	mov	r6, #7
	bne	.L1958
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
	b	.L1898
.L1958:
	mov	fp, #4
.L1898:
	sub	r3, r5, #1
	cmp	r3, #1
	bhi	.L1997
	ldr	r1, .L2007+8
	mov	r9, #0
.L1906:
	ldrb	ip, [r8, #585]	@ zero_extendqisi2
	uxtb	r3, r9
	ldr	r0, .L2007
	cmp	ip, r3
	bls	.L1913
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
.L1908:
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
	bne	.L1908
	mov	ip, r5
	mov	r0, #0
.L1909:
	add	r7, r1, r0
	mov	r3, #1
.L1910:
	ldrb	lr, [r7, r3, asl #2]	@ zero_extendqisi2
	ldrb	r10, [ip]	@ zero_extendqisi2
	add	lr, lr, r10
	strb	lr, [ip, r3, asl #3]
	add	r3, r3, #1
	cmp	r3, #7
	bne	.L1910
	add	r0, r0, #1
	add	ip, ip, #1
	cmp	r0, #4
	bne	.L1909
	mov	r3, #0
	add	r9, r9, #1
	strb	r3, [r5, #16]
	strb	r3, [r5, #24]
	strb	r3, [r5, #32]
	strb	r3, [r5, #40]
	strb	r3, [r5, #48]
	strb	r3, [r5, #41]
	strb	r3, [r5, #49]
	b	.L1906
.L1997:
	sub	r3, r5, #3
	cmp	r3, #5
	bhi	.L1913
	smulbb	r2, fp, r6
	mov	r3, r2, asl #4
	mov	r2, r2, asr #1
	str	r3, [sp, #28]
	mov	r3, r2, asl #1
	str	r3, [sp, #8]
	mov	r3, #0
.L2005:
	str	r3, [sp, #12]
	ldr	r3, .L2007
	ldrb	r2, [r3, #585]	@ zero_extendqisi2
	ldrb	r3, [sp, #12]	@ zero_extendqisi2
	cmp	r2, r3
	bhi	.L1957
.L1913:
	strb	fp, [r4, #113]
	strb	r6, [r4, #114]
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L1957:
	ldr	r2, .L2007
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
	ldr	r3, .L2007
	ldr	r3, [r3, #528]
	str	r3, [sp, #4]
	ldr	r3, [sp]
	mov	r8, r3, asl #8
	ldr	r3, [sp, #4]
	add	r7, r3, r8
	str	r9, [r7, #2056]
	bl	nandc_wait_flash_ready
	cmp	r5, #8
	bne	.L1915
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
	b	.L1916
.L1915:
	cmp	r5, #4
	mov	r3, #54
	str	r3, [r7, #2056]
	moveq	r3, #64
	streq	r9, [r7, #2052]
	streq	r3, [r7, #2048]
	moveq	r3, #204
	beq	.L2002
.L1917:
	sub	r3, r5, #5
	cmp	r3, #1
	ldrlsb	r3, [r4, #128]	@ zero_extendqisi2
	strls	r3, [r7, #2052]
	movls	r3, #82
	bls	.L2003
	cmp	r5, #7
	bne	.L1918
	mov	r3, #174
	str	r3, [r7, #2052]
	mov	r3, #0
	str	r3, [r7, #2048]
	mov	r3, #176
.L2002:
	str	r3, [r7, #2052]
	mov	r3, #77
.L2003:
	str	r3, [r7, #2048]
.L1918:
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
.L1916:
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
	bne	.L1922
	cmp	r5, #7
	movne	ip, #2
	moveq	ip, #32
.L1922:
	ldr	r3, .L2007+12
	ldmia	sp, {r1, r2}
	ldr	r3, [r3, #-3552]
	add	r1, r2, r1, asl #8
	mov	r0, r3
.L1923:
	ldr	r2, [r1, #2048]
	strb	r2, [r0], #1
	rsb	r2, r3, r0
	uxtb	r2, r2
	cmp	r2, ip
	bcc	.L1923
	cmp	r10, #0
	beq	.L1924
	mov	r2, #0
.L1926:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r0, r2
	cmp	ip, #50
	beq	.L1925
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #5
	beq	.L1925
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L1926
	b	.L1927
.L1925:
	cmp	r0, #7
	bne	.L1928
.L1927:
	ldr	r0, .L2007+16
	mov	r1, #0
	bl	printk
.L1929:
	b	.L1929
.L1924:
	cmp	r5, #7
	bne	.L1930
	mov	r2, r10
.L1932:
	ldrb	ip, [r3, r2, asl #2]	@ zero_extendqisi2
	uxtb	r0, r2
	cmp	ip, #12
	beq	.L1931
	add	ip, r3, r2, asl #2
	ldrb	ip, [ip, #1]	@ zero_extendqisi2
	cmp	ip, #10
	beq	.L1931
	add	r2, r2, #1
	cmp	r2, #8
	bne	.L1932
	b	.L1933
.L1931:
	cmp	r0, #7
	bne	.L1928
.L1933:
	ldr	r0, .L2007+16
	mov	r1, #0
	bl	printk
.L1934:
	b	.L1934
.L1930:
	cmp	r5, #6
	bne	.L1928
	sub	r2, r3, #1
	add	r3, r3, #7
.L1935:
	ldrb	r0, [r2, #1]!	@ zero_extendqisi2
	cmp	r0, #12
	beq	.L1928
	ldrb	r0, [r2, #8]	@ zero_extendqisi2
	cmp	r0, #4
	beq	.L1928
	cmp	r2, r3
	bne	.L1935
	ldr	r0, .L2007+16
	mov	r1, #0
	bl	printk
.L1937:
	b	.L1937
.L1928:
	ldr	r3, .L2007+12
	ldr	r0, [sp, #28]
	ldr	r2, [r3, #-3552]
	add	r0, r2, r0
	mov	r3, r2
.L1938:
	cmp	r3, r0
	ldrne	ip, [r1, #2048]
	strneb	ip, [r3], #1
	bne	.L1938
.L2006:
	ldr	r3, .L2007+12
	mov	r0, #8
	ldr	r1, [sp, #8]
	ldr	r3, [r3, #-3552]
	add	r1, r3, r1
	str	r1, [sp, #20]
.L1941:
	ldr	ip, [sp, #8]
	add	lr, r1, ip
.L1940:
	ldrh	ip, [r1]
	mvn	ip, ip
	strh	ip, [r1], #2	@ movhi
	cmp	r1, lr
	bne	.L1940
	ldr	ip, [sp, #8]
	subs	r0, r0, #1
	add	r1, r1, ip
	bne	.L1941
.L1942:
	mov	ip, #0
	mov	r0, ip
.L1945:
	mov	r1, #1
	mov	lr, #0
	mov	r1, r1, asl r0
	mov	r8, #16
	mov	r7, lr
.L1943:
	ldrh	r9, [r3, lr]
	and	r9, r9, r1
	cmp	r9, r1
	addeq	r7, r7, #1
	ldr	r9, [sp, #8]
	subs	r8, r8, #1
	add	lr, lr, r9
	bne	.L1943
	cmp	r7, #8
	add	r0, r0, #1
	orrhi	ip, ip, r1
	uxthhi	ip, ip
	cmp	r0, #16
	bne	.L1945
	ldr	r1, [sp, #20]
	strh	ip, [r3], #2	@ movhi
	cmp	r3, r1
	bne	.L1942
	ldr	r3, .L2007+12
	ldr	r1, [r3, #-3552]
	mov	r3, #0
	sub	r0, r1, #4
	add	ip, r1, #28
.L1947:
	ldr	lr, [r0, #4]!
	cmp	lr, #0
	addeq	r3, r3, #1
	cmp	r0, ip
	bne	.L1947
	cmp	r3, #7
	ble	.L1948
	ldr	r0, .L2007+20
	mov	r2, #1
	mov	r3, #1024
	bl	rknand_print_hex
	ldr	r0, .L2007+16
	mov	r1, #0
	bl	printk
.L1949:
	b	.L1949
.L1948:
	cmp	r5, #6
	moveq	r0, #4
	beq	.L1950
	cmp	r5, #7
	moveq	r0, #10
	beq	.L1950
	cmp	r10, #0
	moveq	r0, #8
	movne	r0, #5
.L1950:
	sub	r9, fp, #1
	ldr	r1, [sp, #24]
	mov	ip, #0
	uxtb	r9, r9
	add	r9, r9, #1
.L1951:
	mov	r7, r1
	mov	r3, r2
.L1952:
	ldrb	lr, [r3], #1	@ zero_extendqisi2
	strb	lr, [r7], #1
	rsb	lr, r2, r3
	uxtb	lr, lr
	cmp	lr, fp
	bcc	.L1952
	add	ip, ip, #1
	add	r2, r2, r9
	cmp	ip, r6
	add	r1, r1, r0
	blt	.L1951
	ldmia	sp, {r2, r3}
	mov	r8, #255
	add	r7, r3, r2, asl #8
	str	r8, [r7, #2056]
	bl	nandc_wait_flash_ready
	ldr	r3, [sp, #16]
	cmp	r3, #0
	beq	.L1954
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
	b	.L2004
.L1954:
	cmp	r10, #0
	movne	r3, #190
	moveq	r3, #56
.L2004:
	str	r3, [r7, #2056]
	bl	nandc_wait_flash_ready
	ldr	r3, [sp, #12]
	add	r3, r3, #1
	b	.L2005
.L2008:
	.align	2
.L2007:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR2+396
	.word	.LANCHOR3
	.word	.LC139
	.word	.LC140
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
	ldr	r3, .L2011
	ldrb	r0, [r3, #23]	@ zero_extendqisi2
	sub	r3, r0, #1
	cmp	r3, #7
	bxhi	lr
	b	hynix_get_read_retry_default
.L2012:
	.align	2
.L2011:
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
	mov	r5, #0
	ldr	r4, .L2064
	ldr	r7, .L2064+4
	ldr	r0, [r4, #528]
	bl	wait_for_nandc_xfer_completed
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	strb	r5, [r7, #-3548]
	cmp	r3, #9
	bne	.L2014
	ldr	r6, [r4, #528]
	ldr	r3, [r6, #16]
	str	r3, [sp]
	ldr	r3, [r6, #48]
	ubfx	r3, r3, #1, #1
	cmp	r3, r5
	moveq	r5, r3
	moveq	r7, r4
	beq	.L2016
.L2015:
	ldr	r2, [r6, #64]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #6
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	bge	.L2018
	ldr	r3, [r4, #528]
	ldr	r3, [r3]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L2017
	ldr	r3, [sp, #4]
	tst	r3, #131072
	beq	.L2017
	ldr	r1, [sp, #4]
	ldr	r0, .L2064+8
	ubfx	r1, r1, #17, #1
	bl	printk
	b	.L2018
.L2017:
	ldr	r3, [sp]
	add	r5, r5, #1
	ubfx	r3, r3, #22, #6
	cmp	r5, r3, asl #12
	bne	.L2015
	ldr	r2, [r6, #64]
	mov	r1, r5
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L2064+12
	ubfx	r3, r3, #22, #6
	bl	printk
	mov	r3, #64
	ldr	r0, .L2064+16
	mov	r2, #4
	ldr	r1, [r4, #528]
	bl	rknand_print_hex
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L2015
	mov	r3, #1
	strb	r3, [r7, #-3548]
.L2018:
	ldr	r3, [r4, #1680]
	ldr	r5, .L2064
	cmp	r3, #0
	beq	.L2022
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
	b	.L2063
.L2024:
	ldr	r3, [r4, #528]
	add	r5, r5, #1
	ldr	r3, [r3, #16]
	str	r3, [sp]
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L2016
	ldr	r2, [sp]
	mov	r1, r5
	ldr	r3, [r6, #64]
	ldr	r0, .L2064+20
	ubfx	r3, r3, #16, #6
	bl	printk
	ldr	r0, .L2064+16
	ldr	r1, [r7, #528]
	mov	r2, #4
	mov	r3, #64
	bl	rknand_print_hex
.L2016:
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L2024
	ldr	r3, [r4, #1680]
	ldr	r5, .L2064
	cmp	r3, #0
	beq	.L2022
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
	b	.L2062
.L2014:
	ldr	r7, [r4, #528]
	ldr	r3, [r7, #8]
	str	r3, [sp]
	ldr	r6, [r7, #16]
	ubfx	r6, r6, #1, #1
	cmp	r6, #0
	moveq	r5, r4
	beq	.L2027
.L2026:
	ldr	r2, [r7, #28]
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ubfx	r3, r3, #22, #6
	cmp	r2, r3
	bge	.L2029
	ldr	r3, [r4, #528]
	ldr	r3, [r3]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #8192
	beq	.L2028
	ldr	r3, [sp, #4]
	tst	r3, #131072
	beq	.L2028
	ldr	r1, [sp, #4]
	ldr	r0, .L2064+24
	bl	printk
	b	.L2029
.L2028:
	add	r5, r5, #1
	bic	r3, r5, #-16777216
	cmp	r3, #0
	bne	.L2026
	ldr	r2, [r7, #28]
	mov	r1, r5
	ldr	r3, [sp]
	ubfx	r2, r2, #16, #5
	ldr	r0, .L2064+12
	ubfx	r3, r3, #22, #6
	bl	printk
	ldr	r0, .L2064+16
	ldr	r1, [r4, #528]
	mov	r2, #4
	mov	r3, #64
	bl	rknand_print_hex
	b	.L2026
.L2029:
	ldr	r3, [r4, #1680]
	ldr	r5, .L2064
	cmp	r3, #0
	beq	.L2022
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
.L2063:
	mov	r2, #0
	b	.L2061
.L2033:
	ldr	r3, [r4, #528]
	add	r6, r6, #1
	ldr	r3, [r3, #8]
	str	r3, [sp]
	bic	r3, r6, #-16777216
	cmp	r3, #0
	bne	.L2027
	ldr	r2, [sp]
	mov	r1, r6
	ldr	r3, [r7, #28]
	ldr	r0, .L2064+20
	ubfx	r3, r3, #16, #5
	bl	printk
	ldr	r0, .L2064+16
	ldr	r1, [r5, #528]
	mov	r2, #4
	mov	r3, #64
	bl	rknand_print_hex
.L2027:
	ldr	r3, [sp]
	tst	r3, #1048576
	beq	.L2033
	ldr	r3, [r4, #1680]
	ldr	r5, .L2064
	cmp	r3, #0
	beq	.L2022
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
.L2062:
	mov	r2, #1
.L2061:
	bl	rknand_dma_unmap_single
.L2022:
	mov	r3, #0
	str	r3, [r4, #1680]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2065:
	.align	2
.L2064:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC141
	.word	.LC142
	.word	.LC143
	.word	.LC144
	.word	.LC145
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
	bne	.L2096
	ldr	r5, .L2109
	ldrb	r3, [r5, #516]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L2068
	ldr	lr, [r5, #528]
	mov	r1, r8, lsr #2
	mov	ip, #1
	mov	r3, r4
	mov	r0, r4
.L2069:
	cmp	r3, r1
	bcs	.L2106
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
	bne	.L2088
	ldr	r2, [sp, #4]
	tst	r2, #262144
	bne	.L2088
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
	b	.L2070
.L2088:
	mvn	r0, #0
.L2070:
	add	r3, r3, #1
	b	.L2069
.L2106:
	ldr	r3, [r5, #528]
	cmp	ip, #0
	movne	r0, #512
	ldr	r4, [r3]
	and	r3, r4, #139264
	cmp	r3, #139264
	bne	.L2075
	mov	r1, r4
	ldr	r0, .L2109+4
	bl	printk
	ldr	r3, [r5, #528]
	mvn	r0, #0
	orr	r4, r4, #131072
	str	r4, [r3]
.L2075:
	tst	r4, #8192
	beq	.L2076
	ldr	r3, .L2109+8
	ldrb	r3, [r3, #-3548]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2076
	ldr	r0, .L2109+12
	mov	r1, r4
	bl	printk
	ldr	r3, [r5, #528]
	mov	r2, #1
	str	r2, [r3, #16]
	b	.L2077
.L2068:
	ldrb	r3, [r5, #584]	@ zero_extendqisi2
	mov	r0, r8, lsr #1
	mov	r2, r7
	mov	r1, r4
	cmp	r3, #25
	mov	r3, r4
	movcc	lr, #64
	movcs	lr, #128
.L2079:
	cmp	r1, r0
	add	ip, r3, lr
	add	r2, r2, #4
	bcs	.L2107
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
	b	.L2079
.L2107:
	ldr	lr, [r5, #528]
	mov	ip, #0
	mov	r1, r8, lsr #2
	mov	r0, ip
.L2081:
	cmp	ip, r1
	bcs	.L2108
	add	r3, ip, #8
	ldr	r3, [lr, r3, asl #2]
	str	r3, [sp, #4]
	ldr	r3, [sp, #4]
	tst	r3, #4
	bne	.L2091
	ldr	r3, [sp, #4]
	tst	r3, #32768
	bne	.L2091
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
	b	.L2082
.L2091:
	mvn	r0, #0
.L2082:
	add	ip, ip, #1
	b	.L2081
.L2108:
	ldr	r3, [r5, #528]
	mov	r2, #0
	str	r2, [r3, #16]
	ldr	r4, [r3]
	and	r3, r4, #139264
	cmp	r3, #139264
	bne	.L2076
	mov	r1, r4
	ldr	r0, .L2109+16
	bl	printk
	ldr	r3, .L2109
	orr	r4, r4, #131072
	ldr	r3, [r3, #528]
	str	r4, [r3]
	b	.L2077
.L2076:
	cmn	r0, #1
	beq	.L2096
	ldr	r3, [r7]
	cmn	r3, #1
	bne	.L2096
	ldr	r3, [r7, #4]
	cmn	r3, #1
	bne	.L2096
	ldr	r3, [r6]
	cmn	r3, #1
	moveq	r0, #512
	b	.L2096
.L2077:
	mvn	r0, #0
.L2096:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2110:
	.align	2
.L2109:
	.word	.LANCHOR0
	.word	.LC146
	.word	.LANCHOR3
	.word	.LC147
	.word	.LC148
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
	.pad #20
	sub	sp, sp, #20
	ldr	r9, .L2123
	mov	r5, r0
	str	r3, [sp, #12]
	mov	r7, r1
	ldr	r3, .L2123+4
	ubfx	r4, r1, #0, #21
	str	r2, [sp, #8]
	ubfx	r8, r1, #24, #2
	ldr	r6, [r9, #528]
	ldr	r3, [r3]
	tst	r3, #16
	beq	.L2112
	ldr	r0, .L2123+8
	mov	r1, r5
	mov	r2, r8
	mov	r3, r7
	bl	printk
.L2112:
	bl	nandc_wait_flash_ready
	mov	r0, r5
	bl	nandc_cs
	cmp	r8, #0
	mov	r10, r5, asl #8
	bne	.L2113
	mov	r0, r5
	bl	zftl_flash_enter_slc_mode
	b	.L2114
.L2113:
	ldr	r3, [r9, #1620]
	ldrb	r3, [r3, #12]	@ zero_extendqisi2
	cmp	r3, #3
	addeq	r3, r6, r10
	streq	r8, [r3, #2056]
	beq	.L2114
	mov	r0, r5
	bl	zftl_flash_exit_slc_mode
.L2114:
	add	fp, r6, r10
	mov	r2, #0
	uxtb	r7, r7
	add	r6, r6, r10
	str	r2, [fp, #2056]
	str	r2, [fp, #2052]
	str	r2, [fp, #2052]
	mov	r2, r4, lsr #8
	str	r7, [fp, #2052]
	str	r2, [fp, #2052]
	mov	r2, r4, lsr #16
	str	r2, [fp, #2052]
	mov	r2, #48
	str	r2, [fp, #2056]
	ldr	r2, [r9, #1620]
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	adds	r2, r8, #0
	movne	r2, #1
	cmp	r1, #3
	movne	r2, #0
	cmp	r2, #0
	addne	r4, r4, r4, asl #1
	subne	r0, r4, #1
	moveq	r0, r4
	addne	r0, r0, r8
	mov	r4, #0
	bl	nandc_set_seed
	bl	nandc_wait_flash_ready
	mov	r2, #5
	mov	r3, #224
	str	r2, [r6, #2056]
	str	r4, [fp, #2052]
	mov	r1, r4
	str	r4, [fp, #2052]
	mov	r0, r5
	str	r3, [r6, #2056]
	ldr	r3, [sp, #12]
	ldrb	r2, [sp, #56]	@ zero_extendqisi2
	str	r3, [sp]
	ldr	r3, [sp, #8]
	bl	nandc_xfer
	mov	r5, r0
	mov	r0, r4
	bl	nandc_de_cs
	mov	r0, r5
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2124:
	.align	2
.L2123:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC149
	.fnend
	.size	flash_read_page, .-flash_read_page
	.align	2
	.global	micron_read_retrial
	.type	micron_read_retrial, %function
micron_read_retrial:
	.fnstart
	@ args = 4, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r3
	ldr	r4, .L2147
	.pad #28
	sub	sp, sp, #28
	mov	r9, r0
	mov	r8, r1
	str	r2, [sp, #20]
	mov	r6, #0
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	ldr	r10, .L2147+4
	add	r3, r3, r3, asl #1
	mov	r3, r3, asr #2
	str	r3, [sp, #8]
	bl	nandc_wait_flash_ready
	ldr	r3, [r4, #528]
	mvn	r4, #0
	add	r5, r3, r9, asl #8
	str	r3, [sp, #16]
.L2126:
	ldrb	r3, [r10, #-3540]	@ zero_extendqisi2
	cmp	r6, r3
	bcs	.L2130
	mov	r3, #239
	mov	r0, #200
	str	r3, [r5, #2056]
	mov	r3, #137
	str	r3, [r5, #2052]
	bl	timer_delay_ns
	add	r3, r6, #1
	str	r3, [r5, #2048]
	mov	r0, r9
	mov	r1, r8
	str	r3, [sp, #12]
	mov	r3, #0
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
	str	r3, [r5, #2048]
	ldr	r3, [sp, #64]
	ldr	r2, [sp, #20]
	str	r3, [sp]
	mov	r3, fp
	bl	flash_read_page
	ldr	r3, .L2147+8
	ldr	r3, [r3]
	tst	r3, #4096
	mov	r7, r0
	beq	.L2127
	str	r0, [sp]
	mov	r1, r6
	ldr	r0, .L2147+12
	mov	r2, r8
	mov	r3, r4
	bl	printk
.L2127:
	cmn	r7, #1
	beq	.L2128
	ldr	r3, [r10, #-3552]
	cmn	r4, #1
	ldr	fp, [r10, #-3544]
	moveq	r4, r7
	str	r3, [sp, #20]
	ldr	r3, [sp, #8]
	cmp	r7, r3
	bcc	.L2134
.L2128:
	ldr	r6, [sp, #12]
	b	.L2126
.L2134:
	mov	r4, r7
.L2130:
	ldr	r3, [sp, #16]
	mov	r0, #200
	add	r9, r3, r9, asl #8
	mov	r3, #239
	str	r3, [r9, #2056]
	mov	r3, #137
	str	r3, [r9, #2052]
	bl	timer_delay_ns
	mov	r3, #0
	str	r3, [r9, #2048]
	str	r3, [r9, #2048]
	str	r3, [r9, #2048]
	str	r3, [r9, #2048]
	ldr	r3, [sp, #8]
	cmp	r4, r3
	bcc	.L2132
	cmn	r4, #1
	movne	r4, #256
.L2132:
	cmn	r4, #1
	cmpne	r4, #256
	bne	.L2133
	str	r4, [sp]
	mov	r1, r6
	ldr	r0, .L2147+16
	mov	r2, r8
	mov	r3, r6
	bl	printk
.L2133:
	bl	nandc_wait_flash_ready
	mov	r0, r4
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2148:
	.align	2
.L2147:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR2
	.word	.LC150
	.word	.LC151
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
	ldr	r7, .L2197
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
	bne	.L2151
	ldr	r3, [sp, #8]
	mov	r8, #1
	add	r3, r3, r10, asl #8
	str	r3, [sp, #16]
.L2158:
	ldr	r3, .L2197
	mov	r0, r6
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2152
	mov	r2, #0
	mov	r1, r8
	bl	toshiba_tlc_set_rr_para
	mov	r3, #93
	ldr	r2, [sp, #16]
	str	r3, [r2, #2056]
	b	.L2153
.L2152:
	uxtb	r1, r8
	bl	toshiba_3d_set_slc_rr_para
.L2153:
	ldr	r3, [sp, #64]
	mov	r0, r10
	mov	r1, r9
	ldr	r2, [sp, #12]
	str	r3, [sp]
	mov	r3, fp
	bl	flash_read_page
	ldr	r3, .L2197+4
	ldr	r3, [r3]
	tst	r3, #16
	mov	ip, r0
	beq	.L2154
	mov	r3, ip
	ldr	r0, .L2197+8
	mov	r1, r8
	mov	r2, r9
	str	ip, [sp, #20]
	bl	printk
	ldr	ip, [sp, #20]
.L2154:
	cmn	ip, #1
	beq	.L2155
	ldr	r3, .L2197+12
	cmn	r4, #1
	moveq	r4, ip
	ldr	r2, [r3, #-3552]
	ldr	fp, [r3, #-3544]
	ldrb	r3, [r7, #584]	@ zero_extendqisi2
	str	r2, [sp, #12]
	add	r3, r3, r3, asl #1
	cmp	ip, r3, asr #2
	bcc	.L2173
.L2155:
	add	r8, r8, #1
	cmp	r8, r5
	bne	.L2158
	b	.L2157
.L2173:
	mov	r5, r8
	mov	r4, ip
.L2157:
	ldr	r3, .L2197
	mov	r1, #0
	mov	r0, r6
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	moveq	r2, r1
	beq	.L2196
	bl	toshiba_3d_set_slc_rr_para
	b	.L2160
.L2151:
	ldr	r3, [sp, #8]
	mov	r5, #1
	add	r8, r3, r10, asl #8
.L2167:
	ldr	r3, .L2197
	mov	r0, r6
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2161
	mov	r1, r5
	mov	r2, #1
	bl	toshiba_tlc_set_rr_para
	mov	r3, #93
	b	.L2195
.L2161:
	uxtb	r1, r5
	bl	toshiba_3d_set_tlc_rr_para
	mov	r3, #38
.L2195:
	str	r3, [r8, #2056]
	mov	r0, r10
	ldr	r3, [sp, #64]
	mov	r1, r9
	ldr	r2, [sp, #12]
	str	r3, [sp]
	mov	r3, fp
	bl	flash_read_page
	ldr	r3, .L2197+4
	ldr	r3, [r3]
	tst	r3, #16
	mov	ip, r0
	beq	.L2163
	mov	r3, ip
	ldr	r0, .L2197+16
	mov	r1, r5
	mov	r2, r9
	str	ip, [sp, #20]
	bl	printk
	ldr	ip, [sp, #20]
.L2163:
	cmn	ip, #1
	beq	.L2164
	ldr	r3, .L2197+12
	cmn	r4, #1
	moveq	r4, ip
	ldr	r2, [r3, #-3552]
	ldr	fp, [r3, #-3544]
	ldrb	r3, [r7, #584]	@ zero_extendqisi2
	str	r2, [sp, #12]
	add	r3, r3, r3, asl #1
	cmp	ip, r3, asr #2
	bcc	.L2174
.L2164:
	ldr	r3, [sp, #16]
	add	r5, r5, #1
	cmp	r5, r3
	bne	.L2167
	b	.L2166
.L2174:
	mov	r4, ip
.L2166:
	ldr	r3, .L2197
	mov	r0, r6
	mov	r1, #0
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2168
	mov	r2, #1
.L2196:
	bl	toshiba_tlc_set_rr_para
	b	.L2160
.L2168:
	bl	toshiba_3d_set_tlc_rr_para
.L2160:
	ldr	r3, .L2197
	ldrb	r3, [r3, #1692]	@ zero_extendqisi2
	cmp	r3, #36
	bne	.L2169
	ldr	r3, [sp, #8]
	add	r10, r3, r10, asl #8
	mov	r3, #85
	str	r3, [r10, #2056]
	mov	r3, #0
	str	r3, [r10, #2052]
	str	r3, [r10, #2048]
	mov	r3, #255
	str	r3, [r10, #2056]
.L2169:
	ldrb	r3, [r7, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r4, r3, asr #2
	bcc	.L2170
	cmn	r4, #1
	movne	r4, #256
.L2170:
	cmn	r4, #1
	cmpne	r4, #256
	bne	.L2171
	str	r4, [sp]
	mov	r1, r5
	ldr	r0, .L2197+20
	mov	r2, r9
	mov	r3, r5
	bl	printk
.L2171:
	bl	nandc_wait_flash_ready
	mov	r0, r4
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2198:
	.align	2
.L2197:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC152
	.word	.LANCHOR3
	.word	.LC153
	.word	.LC154
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
	ldr	r4, .L2232
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
	bls	.L2200
	ldrb	r5, [r4, #1658]	@ zero_extendqisi2
	cmp	r5, #0
	streq	r5, [sp, #8]
	beq	.L2201
	mov	r0, #1
	bl	nandc_set_if_mode
	mov	r3, #1
	str	r3, [sp, #8]
.L2201:
	ldr	r3, [sp, #24]
	mov	r2, #92
	add	r3, r6, r3
	str	r2, [r3, #2056]
	mov	r2, #197
	str	r2, [r3, #2056]
.L2200:
	ldr	fp, .L2232+4
	mov	r7, #1
	mvn	r10, #0
	mov	r3, r8, asl #8
	str	r3, [sp, #28]
.L2202:
	ldrb	r3, [fp, #-3540]	@ zero_extendqisi2
	add	r3, r3, #1
	cmp	r7, r3
	bcs	.L2231
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	mov	r0, r9
	uxtb	r1, r7
	sub	r3, r3, #67
	cmp	r3, #1
	bhi	.L2203
	bl	sandisk_set_rr_para
	b	.L2204
.L2203:
	bl	toshiba_set_rr_para
.L2204:
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	cmp	r3, #34
	bne	.L2205
	ldr	r3, .L2232+4
	ldrb	r3, [r3, #-3540]	@ zero_extendqisi2
	sub	r3, r3, #3
	cmp	r7, r3
	ldreq	r3, [sp, #28]
	moveq	r2, #179
	addeq	r3, r6, r3
	streq	r2, [r3, #2056]
.L2205:
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
	beq	.L2208
	ldr	r3, [fp, #-3552]
	cmn	r10, #1
	moveq	r10, r0
	str	r3, [sp, #16]
	ldr	r3, [fp, #-3544]
	str	r3, [sp, #20]
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L2210
.L2208:
	add	r7, r7, #1
	b	.L2202
.L2231:
	mov	r5, r10
.L2210:
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	mov	r0, r9
	mov	r1, #0
	sub	r3, r3, #67
	cmp	r3, #1
	bhi	.L2212
	bl	sandisk_set_rr_para
	b	.L2213
.L2212:
	bl	toshiba_set_rr_para
.L2213:
	add	r6, r6, r8, asl #8
	mov	r3, #255
	str	r3, [r6, #2056]
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r5, r3, asr #2
	bcc	.L2214
	cmn	r5, #1
	movne	r5, #256
.L2214:
	cmn	r5, #1
	cmpne	r5, #256
	bne	.L2215
	str	r5, [sp]
	mov	r1, r7
	ldr	r0, .L2232+8
	mov	r3, r7
	ldr	r2, [sp, #12]
	bl	printk
.L2215:
	bl	nandc_wait_flash_ready
	ldr	r3, [sp, #8]
	cmp	r3, #0
	beq	.L2216
	mov	r0, #4
	bl	nandc_set_if_mode
.L2216:
	mov	r0, r5
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2233:
	.align	2
.L2232:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC154
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
	ldr	r3, .L2253
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
	ldr	ip, .L2253+4
.L2235:
	cmp	r6, r9
	bcs	.L2239
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
	beq	.L2237
	ldr	r3, [sp, #16]
	cmn	r4, #1
	ldr	r10, [ip, #-3552]
	moveq	r4, r0
	ldr	fp, [ip, #-3544]
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	bcc	.L2243
.L2237:
	add	r6, r6, #1
	b	.L2235
.L2243:
	mov	r4, r0
.L2239:
	ldr	r3, [sp, #12]
	strb	r5, [r3, #120]
	ldr	r3, [sp, #16]
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r4, r3, asr #2
	bcc	.L2241
	cmn	r4, #1
	movne	r4, #256
.L2241:
	cmn	r4, #1
	cmpne	r4, #256
	bne	.L2242
	str	r4, [sp]
	mov	r1, r6
	ldr	r0, .L2253+8
	mov	r2, r8
	mov	r3, r6
	bl	printk
.L2242:
	bl	nandc_wait_flash_ready
	mov	r0, r4
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2254:
	.align	2
.L2253:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC155
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
.L2261:
	uxtb	r0, r7
	bl	nandc_set_ddr_para
	mov	r0, r8
	mov	r1, r10
	mov	r2, fp
	ldr	r3, [sp, #72]
	str	r3, [sp]
	ldr	r3, [sp, #16]
	bl	flash_read_page
	ldr	r3, .L2279
	ldr	r3, [r3]
	tst	r3, #16
	mov	ip, r0
	beq	.L2256
	mov	r3, ip
	ldr	r0, .L2279+4
	mov	r1, r7
	mov	r2, r10
	str	ip, [sp, #28]
	bl	printk
	ldr	ip, [sp, #28]
.L2256:
	add	r3, r6, #1
	cmp	ip, r3
	bhi	.L2257
	ldr	r3, .L2279+8
	ldr	fp, [r3, #-3552]
	ldr	r3, [r3, #-3544]
	str	r3, [sp, #16]
	ldr	r3, .L2279+12
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	cmp	ip, r3, lsr #2
	bcs	.L2267
	add	r5, r5, #1
	cmp	r5, #7
	bls	.L2267
	rsb	r4, r5, r7
	mov	r6, ip
	mov	r9, #0
	b	.L2259
.L2257:
	ldr	r3, [sp, #12]
	cmp	r3, r5
	bcs	.L2268
	cmp	r5, #7
	rsb	r3, r5, r4
	str	r3, [sp, #20]
	bhi	.L2260
	str	r5, [sp, #12]
	b	.L2268
.L2267:
	mov	r4, r7
	mov	r6, ip
	mov	r9, #0
	b	.L2258
.L2268:
	mov	r5, #0
.L2258:
	add	r7, r7, #2
	cmp	r7, #50
	bne	.L2261
.L2259:
	ldr	r2, [sp, #12]
	ldr	r3, [sp, #20]
	cmp	r2, r5
	movhi	r4, r3
.L2260:
	cmp	r4, #0
	beq	.L2262
	ldr	r3, .L2279+12
	ldr	r2, .L2279+16
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	umull	r2, r3, r3, r2
	ubfx	r3, r3, #1, #8
	cmp	r6, r3
	bcs	.L2262
	ldr	r0, .L2279+20
	mov	r1, r4
	bl	printk
	uxtb	r0, r4
	b	.L2278
.L2262:
	ldrb	r0, [sp, #24]	@ zero_extendqisi2
.L2278:
	bl	nandc_set_ddr_para
	cmn	r9, #1
	bne	.L2264
	mov	r1, r8
	mov	r2, r10
	ldr	r0, .L2279+24
	bl	printk
	mov	r0, r8
	bl	flash_reset
	ldr	r4, .L2279+12
	mov	r0, #1
	bl	flash_set_interface_mode
	mov	r0, #1
	bl	nandc_set_if_mode
	add	r3, r4, r8
	mov	r0, r8
	mov	r2, #2
	strb	r2, [r3, #1700]
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
	ldr	r0, .L2279+28
	mov	r3, r6
	bl	printk
	ldrb	r3, [r4, #584]	@ zero_extendqisi2
	cmp	r6, r3
	bhi	.L2269
	ldr	r2, .L2279+8
	ldr	r3, [r2, #-3536]
	add	r3, r3, #1
	str	r3, [r2, #-3536]
	cmp	r3, #100
	movhi	r3, #0
	strhib	r3, [r4, #1658]
	bls	.L2265
	b	.L2264
.L2269:
	mov	r6, r9
.L2265:
	ldrb	r0, [r5, #1774]	@ zero_extendqisi2
	bl	flash_set_interface_mode
	ldrb	r0, [r5, #1774]	@ zero_extendqisi2
	bl	nandc_set_if_mode
.L2264:
	mov	r0, r6
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2280:
	.align	2
.L2279:
	.word	.LANCHOR2
	.word	.LC156
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	-1431655765
	.word	.LC157
	.word	.LC158
	.word	.LC159
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
	ldr	r5, .L2312
	mov	r10, r0
	mov	r4, r1
	mov	r8, r2
	ldr	r9, [sp, #48]
	ldrb	r3, [r5, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L2282
	ldr	r1, .L2312+4
	movw	r2, #399
	ldr	r0, .L2312+8
	bl	printk
	bl	dump_stack
.L2282:
	add	r2, r5, r10
	ldr	r3, .L2312
	ldrb	r6, [r2, #588]	@ zero_extendqisi2
	ldrb	r2, [r5, #585]	@ zero_extendqisi2
	cmp	r2, r10
	bcs	.L2283
	ldr	r3, .L2312+12
	ldr	r3, [r3]
	tst	r3, #64
	beq	.L2311
	ldr	r0, .L2312+16
	mov	r1, r6
	mov	r2, r10
	mov	r3, r4
	bl	printk
.L2311:
	mvn	r0, #0
	b	.L2308
.L2283:
	tst	r4, #50331648
	bne	.L2286
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2287
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2286
.L2287:
	ldrh	r10, [r5, #2]
	mov	r0, r4
	mov	r1, r10
	bl	__aeabi_uidiv
	ldrb	r3, [r5, #1]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L2312
	mul	r0, r10, r0
	rsb	r4, r0, r4
	mov	r4, r4, asl #1
	addeq	r4, r3, r4
	ldreqh	r4, [r4, #4]
	add	r4, r4, r0
.L2286:
	str	r9, [sp]
	mov	r0, r6
	mov	r1, r4
	mov	r2, r8
	mov	r3, r7
	bl	flash_read_page
	cmn	r0, #1
	bne	.L2308
	ldrb	fp, [r5, #1688]	@ zero_extendqisi2
	ldr	r10, .L2312
	cmp	fp, #0
	bne	.L2290
.L2293:
	ldr	r3, .L2312+20
	ldr	ip, [r3, #-3532]
	cmp	ip, #0
	bne	.L2291
	b	.L2292
.L2290:
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
	beq	.L2293
	b	.L2308
.L2291:
	str	r9, [sp]
	mov	r0, r6
	mov	r1, r4
	mov	r2, r8
	mov	r3, r7
	blx	ip
	cmn	r0, #1
	bne	.L2308
.L2292:
	ldrb	r3, [r5, #1688]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, .L2312+24
	mov	r2, r4
	str	r3, [sp]
	mvn	r3, #0
	bl	printk
	ldrb	r3, [r5, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2311
	str	r9, [sp, #48]
	mov	r0, r6
	mov	r1, r4
	mov	r2, r8
	mov	r3, r7
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	flash_ddr_tuning_read
.L2308:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2313:
	.align	2
.L2312:
	.word	.LANCHOR0
	.word	.LANCHOR1+1516
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC160
	.word	.LANCHOR3
	.word	.LC161
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
	ldr	r6, .L2325
	movw	r1, #1704
	ldr	ip, .L2325+4
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
	bne	.L2315
.L2316:
	sxth	r3, ip
	sxth	r2, r4
	cmp	r3, r2
	bgt	.L2315
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
	b	.L2316
.L2315:
	ldr	r3, [r6]
	sxth	r4, r4
	tst	r3, #4096
	beq	.L2320
	ldr	r0, .L2325+8
	mov	r1, r9
	mov	r2, r4
	ldr	r3, [r8]
	bl	printk
.L2320:
	mov	r0, r4
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2326:
	.align	2
.L2325:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC162
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
	ldr	r1, .L2329
	mov	lr, r2
	movw	r2, #3764
	mov	ip, r0
	ldrh	r2, [r1, r2]
	mvn	r1, #0
	str	r3, [sp]
	mov	r3, lr
	rsb	r2, r2, #21
	mov	r0, r0, asr r2
	bic	r1, ip, r1, asl r2
	mov	r2, r4
	uxtb	r0, r0
	bl	flash_get_last_written_page
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, pc}
.L2330:
	.align	2
.L2329:
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
	ldr	r4, .L2338
	mov	r7, r1
	mov	r6, #1
	ldr	r5, .L2338+4
	mov	r9, #4
	ldrb	r0, [r4, #1774]	@ zero_extendqisi2
	strb	r6, [r4, #1658]
	bl	flash_set_interface_mode
	ldrb	r0, [r4, #1774]	@ zero_extendqisi2
	bl	nandc_set_if_mode
	mov	r1, r7
	ldr	r2, [r5, #-3528]
	mov	r0, r8
	ldr	r3, [r5, #-3524]
	str	r9, [sp]
	bl	flash_ddr_tuning_read
	ldr	r3, [r5, #-3524]
	mov	r0, r8
	mov	r1, r7
	ldr	r2, [r5, #-3528]
	str	r9, [sp]
	bl	flash_read_page
	mov	r3, r4
	cmn	r0, #1
	bne	.L2332
	ldrb	r2, [r4, #1774]	@ zero_extendqisi2
	tst	r2, #1
	beq	.L2332
	mov	r0, r6
	bl	flash_set_interface_mode
	mov	r0, r6
	bl	nandc_set_if_mode
	mov	r3, #0
	strb	r3, [r4, #1658]
	b	.L2333
.L2332:
	mov	r2, #1
	strb	r2, [r3, #1658]
.L2333:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2339:
	.align	2
.L2338:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	flash_ddr_para_scan, .-flash_ddr_para_scan
	.global	__aeabi_uidivmod
	.align	2
	.type	id_block_read_data.constprop.30, %function
id_block_read_data.constprop.30:
	.fnstart
	@ args = 0, pretend = 0, frame = 112
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2371
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r6, r2
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	.pad #124
	sub	sp, sp, #124
	ldr	r4, .L2371+4
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
.L2341:
	ldr	r3, [sp, #24]
	cmp	r5, r3
	bcs	.L2370
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
	beq	.L2343
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	cmp	r2, #0
	movne	r0, r3
.L2343:
	ldr	r2, [sp, #32]
	ldr	r3, [sp, #28]
	ldr	r1, [sp, #16]
	mla	r3, r3, r0, r2
	str	r3, [sp, #36]
	ldr	r3, .L2371+4
	ldr	r2, [sp, #36]
	ldrb	r9, [r3, #1624]	@ zero_extendqisi2
	add	r0, r2, r1
	ldr	r3, [r3, #1620]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r3, .L2371+4
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	mov	fp, r0
	mov	r0, r9
	str	r3, [sp, #48]
	bl	nandc_bch_sel
.L2344:
	str	r7, [sp]
	mov	r0, #0
	mov	r1, fp
	mov	r2, r6
	add	r3, sp, #56
	bl	flash_read_page
	cmn	r0, #1
	mov	r8, r0
	bne	.L2351
	ldrb	ip, [r4, #1688]	@ zero_extendqisi2
	cmp	ip, #0
	bne	.L2346
.L2349:
	ldr	r3, .L2371+8
	ldr	ip, [r3, #-3532]
	cmp	ip, #0
	bne	.L2347
	b	.L2348
.L2346:
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
	beq	.L2349
	b	.L2369
.L2347:
	str	r7, [sp]
	mov	r0, #0
	mov	r1, fp
	mov	r2, r6
	add	r3, sp, #56
	blx	ip
	cmn	r0, #1
	bne	.L2369
.L2348:
	ldrb	r3, [r4, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2351
	str	r7, [sp]
	mov	r0, #0
	mov	r1, fp
	mov	r2, r6
	add	r3, sp, #56
	bl	flash_ddr_tuning_read
.L2369:
	mov	r8, r0
.L2351:
	subs	r9, r9, #16
	movne	r9, #1
	cmn	r8, #1
	movne	r9, #0
	cmp	r9, #0
	beq	.L2352
	mov	r0, #16
	mov	r9, #16
	bl	nandc_bch_sel
	b	.L2344
.L2352:
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
	beq	.L2354
	ldr	r3, [r6]
	ldr	r2, .L2371+12
	cmp	r3, r2
	bne	.L2354
	ldr	r3, [sp, #24]
	ldr	r2, [sp, #20]
	ldrb	r7, [r6, #17]	@ zero_extendqisi2
	add	r3, r3, r2
	str	r3, [sp, #24]
	b	.L2355
.L2354:
	ldr	r3, [sp, #20]
	mov	r2, r5
	ldr	r0, .L2371+16
	ldr	r1, [sp, #36]
	add	r6, r6, r3, asl #9
	ldr	r3, [sp, #60]
	str	r3, [sp]
	ldr	r3, [sp, #56]
	bl	printk
	mov	r3, #0
	str	r3, [sp, #16]
.L2355:
	ldr	r3, [sp, #20]
	add	r5, r3, r5
	uxth	r5, r5
	b	.L2341
.L2370:
	ldr	r3, .L2371+4
	mov	r0, r10
	ldrb	r2, [sp, #44]	@ zero_extendqisi2
	strb	r2, [r3, #1688]
	add	sp, sp, #124
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2372:
	.align	2
.L2371:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	1179535694
	.word	.LC163
	.fnend
	.size	id_block_read_data.constprop.30, .-id_block_read_data.constprop.30
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
	ubfx	r6, r0, #21, #3
	ldr	r5, .L2385
	mov	r7, r1
	mov	r8, r2
	ubfx	r9, r0, #0, #21
	ubfx	r4, r0, #24, #2
	ldrb	r3, [r5, #585]	@ zero_extendqisi2
	cmp	r3, r6
	bhi	.L2374
	ldr	r1, .L2385+4
	movw	r2, #887
	ldr	r0, .L2385+8
	bl	printk
	bl	dump_stack
.L2374:
	add	r6, r5, r6
	ldr	r10, [r5, #528]
	ldrb	r6, [r6, #588]	@ zero_extendqisi2
	mov	r0, r6
	bl	nandc_cs
	cmp	r4, #0
	bne	.L2375
	mov	r0, r9
	bl	slc_phy_page_address_calc
	mov	r9, r0
.L2375:
	ldrb	r3, [r5, #1642]	@ zero_extendqisi2
	uxtb	r0, r9
	mov	r1, r9, lsr #8
	mov	r2, r9, lsr #16
	cmp	r3, #1
	mov	r3, r6, asl #8
	addeq	r3, r10, r3
	moveq	ip, #6
	beq	.L2384
	ldr	ip, .L2385
	add	r3, r10, r3
	ldr	ip, [ip, #1620]
	ldrb	ip, [ip, #12]	@ zero_extendqisi2
	cmp	ip, #3
	bne	.L2378
	mov	ip, #5
.L2384:
	str	ip, [r3, #2056]
	mov	ip, #0
	str	ip, [r3, #2052]
	str	ip, [r3, #2052]
	str	r0, [r3, #2052]
	str	r1, [r3, #2052]
	str	r2, [r3, #2052]
	b	.L2382
.L2378:
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
.L2382:
	mov	r2, #224
	str	r2, [r3, #2056]
	ldr	r3, [r5, #1620]
	ldrb	r2, [r3, #12]	@ zero_extendqisi2
	adds	r3, r4, #0
	movne	r3, #1
	cmp	r2, #3
	movne	r3, #0
	cmp	r3, #0
	subne	r3, r4, #1
	addne	r0, r9, r9, asl #1
	addne	r0, r3, r0
	moveq	r0, r9
	bl	nandc_set_seed
	ldr	r3, .L2385+12
	mov	r1, #0
	mov	r0, r6
	ldrb	r2, [r3, #13]	@ zero_extendqisi2
	mov	r3, r7
	str	r8, [sp]
	bl	nandc_xfer
	mov	r4, r0
	mov	r0, r6
	bl	nandc_de_cs
	mov	r0, r4
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2386:
	.align	2
.L2385:
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
	ubfx	r7, r0, #21, #3
	ldr	r5, .L2418
	.pad #20
	sub	sp, sp, #20
	mov	r9, r0
	mov	r10, r1
	mov	fp, r2
	ubfx	r4, r0, #0, #21
	ldrb	r3, [r5, #585]	@ zero_extendqisi2
	ubfx	r6, r0, #24, #2
	cmp	r3, r7
	bhi	.L2388
	ldr	r1, .L2418+4
	movw	r2, #1009
	ldr	r0, .L2418+8
	bl	printk
	bl	dump_stack
.L2388:
	add	r7, r5, r7
	ldrb	r8, [r7, #588]	@ zero_extendqisi2
	ldr	r7, [r5, #528]
	mov	r0, r8
	bl	nandc_cs
	cmp	r6, #0
	bne	.L2389
	mov	r0, r4
	bl	slc_phy_page_address_calc
	mov	r4, r0
.L2389:
	add	r3, r7, r8, asl #8
	mov	r2, #5
	ldr	r7, .L2418+12
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
	subne	r0, r6, #1
	addne	r3, r4, r4, asl #1
	addne	r0, r0, r3
	moveq	r0, r4
	bl	nandc_set_seed
	ldrb	r2, [r7, #13]	@ zero_extendqisi2
	mov	r0, r8
	mov	r1, #0
	mov	r3, r10
	str	fp, [sp]
	bl	nandc_xfer
	cmn	r0, #1
	bne	.L2392
	ldrb	ip, [r5, #1688]	@ zero_extendqisi2
	cmp	ip, #0
	beq	.L2393
	ldr	r2, .L2418
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
	ldr	r3, .L2418
	cmp	r6, #0
	ldr	ip, [sp, #12]
	strb	ip, [r3, #1688]
	bne	.L2394
.L2401:
	ldrb	r3, [r5]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2394
	ldr	r3, .L2418
	ldrb	r3, [r3, #584]	@ zero_extendqisi2
	add	r3, r3, r3, asl #1
	cmp	r0, r3, asr #2
	blt	.L2394
	ldrb	r3, [r7, #23]	@ zero_extendqisi2
	sub	r3, r3, #4
	cmp	r3, #4
	movls	r0, #256
	b	.L2407
.L2394:
	cmn	r0, #1
	bne	.L2407
.L2402:
	ldr	r3, .L2418+16
	ldr	ip, [r3, #-3532]
	cmp	ip, #0
	bne	.L2397
.L2400:
	ldrb	r3, [r5, #1688]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, .L2418+20
	mov	r2, r9
	str	r3, [sp]
	mvn	r3, #0
	bl	printk
	ldrb	r3, [r5, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	mvneq	r0, #0
	beq	.L2407
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	mov	r0, r8
	orr	r1, r4, r6, asl #24
	mov	r2, r10
	str	r3, [sp]
	mov	r3, fp
	bl	flash_ddr_tuning_read
	b	.L2407
.L2397:
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	mov	r0, r8
	orr	r1, r4, r6, asl #24
	mov	r2, r10
	str	r3, [sp]
	mov	r3, fp
	blx	ip
	cmn	r0, #1
	bne	.L2407
	b	.L2400
.L2392:
	cmp	r6, #0
	beq	.L2401
	b	.L2407
.L2393:
	cmp	r6, #0
	beq	.L2401
	b	.L2402
.L2407:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2419:
	.align	2
.L2418:
	.word	.LANCHOR0
	.word	.LANCHOR1+1568
	.word	.LC0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.word	.LC164
	.fnend
	.size	flash_complete_page_read, .-flash_complete_page_read
	.align	2
	.type	queue_wait_first_req_completed, %function
queue_wait_first_req_completed:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L2516
	ldrb	r5, [r4, #3315]	@ zero_extendqisi2
	cmp	r5, #255
	beq	.L2454
	mov	r3, #48
	mla	r3, r3, r5, r4
	ldrb	r2, [r3, #1818]	@ zero_extendqisi2
	ldr	r6, [r3, #1800]
	sub	r3, r2, #1
	cmp	r3, #10
	ldrls	pc, [pc, r3, asl #2]
	b	.L2454
.L2424:
	.word	.L2423
	.word	.L2425
	.word	.L2426
	.word	.L2426
	.word	.L2426
	.word	.L2426
	.word	.L2427
	.word	.L2428
	.word	.L2429
	.word	.L2426
	.word	.L2429
.L2423:
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	beq	.L2454
	mov	r3, #48
	mla	r3, r3, r5, r4
	ldr	r2, [r3, #1784]
	ldr	r1, [r3, #1780]
	cmp	r2, #0
	beq	.L2431
	ldrb	r0, [r3, #1816]	@ zero_extendqisi2
	ldr	r3, .L2516
	ldrb	r3, [r3, #3312]	@ zero_extendqisi2
	cmp	r0, r3
	moveq	r1, r2
.L2431:
	mov	r3, #48
	mov	r0, r6
	mla	r5, r3, r5, r4
	ldr	r2, [r5, #1788]
	bl	flash_complete_page_read
	mov	r3, #13
	strb	r3, [r5, #1818]
	ldrb	r3, [r5, #1778]	@ zero_extendqisi2
	orr	r3, r3, #8
	strb	r3, [r5, #1778]
	str	r0, [r5, #1812]
	b	.L2454
.L2425:
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	mov	r2, #48
	mla	r3, r2, r5, r4
	ldr	r1, .L2516
	ldrb	r6, [r3, #1776]	@ zero_extendqisi2
	tst	r0, #64
	beq	.L2454
	mla	r2, r2, r6, r1
	ldr	fp, [r3, #1780]
	ldr	r9, [r2, #1780]
	ldr	r2, [r3, #1784]
	cmp	r2, #0
	beq	.L2432
	ldrb	r0, [r3, #1816]	@ zero_extendqisi2
	ldrb	r3, [r1, #3312]	@ zero_extendqisi2
	cmp	r0, r3
	moveq	fp, r2
.L2432:
	mov	r3, #48
	mla	r3, r3, r6, r4
	ldr	r2, [r3, #1784]
	cmp	r2, #0
	beq	.L2433
	ldrb	r1, [r3, #1816]	@ zero_extendqisi2
	ldr	r3, .L2516
	ldrb	r3, [r3, #3312]	@ zero_extendqisi2
	cmp	r1, r3
	moveq	r9, r2
.L2433:
	mov	r7, #48
	mov	r1, fp
	mla	ip, r7, r5, r4
	mla	r7, r7, r6, r4
	add	r3, ip, #1776
	add	r3, r3, #8
	str	r3, [sp, #8]
	ldr	r2, [ip, #1788]
	ldr	r0, [ip, #1800]
	str	ip, [sp, #12]
	bl	flash_complete_plane_page_read
	mov	r1, r9
	ldr	r2, [r7, #1788]
	mov	r8, r0
	ldr	r0, [r7, #1800]
	bl	flash_complete_plane_page_read
	cmn	r8, #1
	mov	r10, r0
	beq	.L2434
	ldr	ip, [sp, #12]
	ldr	r2, [ip, #1796]
	cmn	r2, #1
	beq	.L2435
	ldr	r3, [sp, #8]
	ldr	r3, [r3, #4]
	ldr	r3, [r3, #4]
	cmp	r2, r3
	beq	.L2435
.L2434:
	mov	r7, #48
	ldrb	r3, [r4, #3312]	@ zero_extendqisi2
	mla	r7, r7, r5, r4
	mov	r2, fp
	add	ip, r7, #1776
	add	ip, ip, #8
	str	ip, [sp, #8]
	ldr	r1, [r7, #1800]
	str	r3, [sp]
	ubfx	r0, r1, #21, #3
	ldr	r3, [r7, #1788]
	bic	r1, r1, #14680064
	bl	flash_read_page_en
	ldr	r2, [r7, #1796]
	cmn	r2, #1
	mov	r8, r0
	beq	.L2436
	ldr	ip, [sp, #8]
	ldr	r3, [ip, #4]
	ldr	r1, [r3, #4]
	cmp	r2, r1
	beq	.L2436
	ldr	r0, .L2516+4
	ldr	r0, [r0]
	tst	r0, #64
	beq	.L2436
	str	r1, [sp]
	ldr	r0, .L2516+8
	ldr	r1, [r7, #1800]
	ldr	r3, [r3]
	bl	printk
.L2436:
	mov	r3, #48
	mla	r3, r3, r5, r4
	ldr	r2, [r3, #1796]
	cmn	r2, #1
	beq	.L2435
	ldr	r3, [r3, #1788]
	ldr	r3, [r3, #4]
	cmp	r2, r3
	beq	.L2435
	ldr	r1, .L2516+12
	mov	r2, #360
	ldr	r0, .L2516+16
	bl	printk
	bl	dump_stack
.L2435:
	mov	r3, #48
	mov	r2, #13
	mla	r5, r3, r5, r4
	cmn	r10, #1
	strb	r2, [r5, #1818]
	ldrb	r2, [r5, #1778]	@ zero_extendqisi2
	str	r8, [r5, #1812]
	orr	r2, r2, #8
	strb	r2, [r5, #1778]
	beq	.L2437
	ldr	r2, .L2516
	mla	r3, r3, r6, r2
	ldr	r2, [r3, #1796]
	cmn	r2, #1
	beq	.L2439
	ldr	r3, [r3, #1788]
	ldr	r3, [r3, #4]
	cmp	r2, r3
	beq	.L2439
.L2437:
	mov	r5, #48
	ldrb	r3, [r4, #3312]	@ zero_extendqisi2
	mla	r5, r5, r6, r4
	mov	r2, r9
	add	r7, r5, #1776
	add	r7, r7, #8
	ldr	r1, [r5, #1800]
	str	r3, [sp]
	ubfx	r0, r1, #21, #3
	ldr	r3, [r5, #1788]
	bic	r1, r1, #14680064
	bl	flash_read_page_en
	ldr	r2, [r5, #1796]
	cmn	r2, #1
	beq	.L2441
	ldr	r3, [r7, #4]
	ldr	r1, [r3, #4]
	cmp	r2, r1
	beq	.L2441
	ldr	r0, .L2516+4
	ldr	r0, [r0]
	tst	r0, #64
	beq	.L2441
	str	r1, [sp]
	ldr	r0, .L2516+8
	ldr	r1, [r5, #1800]
	ldr	r3, [r3]
	bl	printk
.L2441:
	mov	r3, #48
	mla	r3, r3, r6, r4
	ldr	r2, [r3, #1796]
	cmn	r2, #1
	beq	.L2439
	ldr	r3, [r3, #1788]
	ldr	r3, [r3, #4]
	cmp	r2, r3
	beq	.L2439
	ldr	r1, .L2516+12
	movw	r2, #375
	ldr	r0, .L2516+16
	bl	printk
	bl	dump_stack
.L2439:
	mov	r3, #48
	mla	r4, r3, r6, r4
	mov	r3, #13
	strb	r3, [r4, #1818]
	ldrb	r3, [r4, #1778]	@ zero_extendqisi2
	str	r8, [r4, #1812]
	orr	r3, r3, #8
	strb	r3, [r4, #1778]
	b	.L2454
.L2426:
	bl	nandc_iqr_wait_flash_ready
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	mov	r7, r0
	beq	.L2454
	ands	r6, r0, #5
	mov	r3, #48
	beq	.L2442
	mla	r5, r3, r5, r4
	ldr	r0, .L2516+20
	mov	r3, #12
	mov	r2, r7
	mov	r6, r7
	strb	r3, [r5, #1818]
	ldrb	r1, [r5, #1777]	@ zero_extendqisi2
	str	r3, [sp]
	ldr	r3, [r5, #1800]
	bl	printk
	mvn	r3, #0
	str	r3, [r5, #1812]
	b	.L2422
.L2442:
	mul	r3, r3, r5
	mov	r1, #13
	ldr	r2, .L2516
	add	r7, r4, r3
	strb	r1, [r7, #1818]
	ldr	r1, [r4, #3340]
	str	r6, [r7, #1812]
	ldr	r0, [r1, #156]
	ldr	r1, .L2516+24
	cmp	r0, r1
	bne	.L2454
	add	r3, r2, r3
	add	r3, r3, #1808
	ldrh	r3, [r3, #2]
	cmp	r3, #0
	bne	.L2454
	ldrb	r3, [r2, #3312]	@ zero_extendqisi2
	ldr	r8, .L2516+28
	ldr	r1, [r7, #1800]
	str	r3, [sp]
	ubfx	r0, r1, #21, #3
	ldr	r3, [r8, #-3524]
	bic	r1, r1, #14680064
	ldr	r2, [r8, #-3528]
	bl	flash_read_page_en
	cmn	r0, #1
	mov	r3, r0
	beq	.L2443
	ldr	r1, [r7, #1788]
	ldr	r2, [r8, #-3524]
	ldr	r1, [r1]
	ldr	r2, [r2]
	cmp	r1, r2
	beq	.L2454
.L2443:
	mov	r2, #48
	ldr	r0, .L2516+32
	mla	r5, r2, r5, r4
	ldrb	r2, [r4, #1688]	@ zero_extendqisi2
	ldrb	r1, [r5, #1777]	@ zero_extendqisi2
	str	r2, [sp]
	ldr	r2, [r5, #1800]
	bl	printk
	mvn	r3, #0
	str	r3, [r5, #1812]
	b	.L2422
.L2429:
	ldr	r3, .L2516+36
	mov	r1, #48
	cmp	r2, #11
	ubfx	lr, r6, #21, #3
	mla	r0, r1, r5, r3
	movne	ip, #3
	moveq	ip, #10
.L2445:
	ldrb	r8, [r0]	@ zero_extendqisi2
	cmp	r8, #255
	beq	.L2515
	mul	r2, r1, r8
	ldr	r7, .L2516
	add	r9, r7, #1776
	add	r0, r3, r2
	add	r2, r4, r2
	ldrb	r10, [r2, #1818]	@ zero_extendqisi2
	cmp	r10, ip
	bne	.L2445
	ldr	r2, [r2, #1800]
	ubfx	r2, r2, #21, #3
	cmp	lr, r2
	bne	.L2445
	bl	nandc_iqr_wait_flash_ready
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	mov	r6, r0
	beq	.L2514
	ands	r2, r0, #15
	mov	r3, #48
	mul	r3, r3, r5
	beq	.L2449
	add	r7, r7, r3
	mov	r9, #12
	ldr	r0, .L2516+40
	mov	r2, r6
	ldrb	r1, [r7, #1777]	@ zero_extendqisi2
	str	r9, [sp]
	ldr	r3, [r7, #1800]
	bl	printk
	mvn	r3, #0
	strb	r9, [r7, #1818]
	str	r3, [r7, #1812]
	b	.L2448
.L2449:
	add	r6, r7, r3
	mov	r1, #13
	str	r2, [r6, #1812]
	ldr	r2, [r7, #3340]
	strb	r1, [r6, #1818]
	ldr	r1, [r2, #156]
	ldr	r2, .L2516+24
	cmp	r1, r2
	bne	.L2514
	add	r3, r9, r3
	ldrh	r3, [r3, #34]
	cmp	r3, #0
	bne	.L2514
	ldrb	r3, [r7, #3312]	@ zero_extendqisi2
	ldr	r9, .L2516+28
	ldr	r1, [r6, #1800]
	str	r3, [sp]
	ubfx	r0, r1, #21, #3
	ldr	r3, [r9, #-3524]
	bic	r1, r1, #14680064
	ldr	r2, [r9, #-3528]
	bl	flash_read_page_en
	cmn	r0, #1
	mov	r3, r0
	beq	.L2451
	ldr	r1, [r6, #1788]
	ldr	r2, [r9, #-3524]
	ldr	r1, [r1]
	ldr	r2, [r2]
	cmp	r1, r2
	beq	.L2514
.L2451:
	mov	r6, #48
	ldrb	r2, [r4, #1688]	@ zero_extendqisi2
	mla	r6, r6, r5, r4
	ldr	r0, .L2516+44
	ldrb	r1, [r6, #1777]	@ zero_extendqisi2
	str	r2, [sp]
	ldr	r2, [r6, #1800]
	bl	printk
	mvn	r3, #0
	str	r3, [r6, #1812]
.L2514:
	mov	r6, #0
.L2448:
	mov	r3, #48
	mla	r5, r3, r5, r4
	mla	r4, r3, r8, r4
	ldrb	r2, [r5, #1818]	@ zero_extendqisi2
	ldr	r3, [r5, #1812]
	strb	r2, [r4, #1818]
	str	r3, [r4, #1812]
	b	.L2422
.L2515:
	mvn	r6, #0
	b	.L2422
.L2427:
	mov	r0, r6
	mov	r1, #32
	bl	flash_wait_device_ready
	tst	r0, #32
	beq	.L2454
	ands	r6, r0, #15
	mov	r3, #48
	mlaeq	r5, r3, r5, r4
	movne	r2, #12
	mlane	r3, r3, r5, r4
	moveq	r3, #13
	movne	r6, r0
	streqb	r3, [r5, #1818]
	streq	r6, [r5, #1812]
	strneb	r2, [r3, #1818]
	mvnne	r2, #0
	strne	r2, [r3, #1812]
	b	.L2422
.L2428:
	mov	r0, r6
	mov	r1, #64
	bl	flash_wait_device_ready
	tst	r0, #64
	movne	r3, #48
	mlane	r5, r3, r5, r4
	movne	r3, #7
	strne	r0, [r5, #1812]
	strneb	r3, [r5, #1818]
.L2454:
	mov	r6, #0
.L2422:
	mov	r0, r6
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2517:
	.align	2
.L2516:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC165
	.word	.LANCHOR1+1596
	.word	.LC0
	.word	.LC166
	.word	1145785929
	.word	.LANCHOR3
	.word	.LC167
	.word	.LANCHOR0+1776
	.word	.LC168
	.word	.LC169
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
	ldr	r4, .L2523
.L2519:
	ldrb	r3, [r4, #3315]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2522
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2519
.L2522:
	ldmfd	sp!, {r4, pc}
.L2524:
	.align	2
.L2523:
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
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r4, r0
	mov	r6, r1
	mov	r5, r2
	mov	r7, r3
	bl	sblk_wait_write_queue_completed
	ubfx	r0, r4, #21, #3
	mov	r1, r4
	mov	r2, r6
	mov	r3, r5
	str	r7, [sp]
	bl	flash_read_page_en
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
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
	ldr	fp, .L2557
	mov	r8, r0
	mov	r6, r1
	mov	r4, r0
	mov	r5, r1
.L2530:
	cmp	r5, #0
	beq	.L2553
	ldrb	r10, [r4]	@ zero_extendqisi2
	ldr	r9, [r4, #24]
.L2531:
	mov	r0, r9
	mov	r1, #0
	bl	queue_lun_state
	cmp	r0, #0
	beq	.L2555
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2531
.L2555:
	cmp	r5, #1
	beq	.L2537
	ldr	r3, .L2557+4
	ldrb	r3, [r3, #-3520]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2537
	ldrb	r3, [r4]	@ zero_extendqisi2
	ubfx	ip, r9, #21, #3
	cmp	r3, #255
	bne	.L2536
	ldr	r1, .L2557+8
	movw	r2, #661
	ldr	r0, .L2557+12
	str	ip, [sp, #4]
	bl	printk
	bl	dump_stack
	ldr	ip, [sp, #4]
.L2536:
	ldrb	r3, [r4]	@ zero_extendqisi2
	mov	r2, #48
	mul	r3, r2, r3
	ldr	r2, .L2557+16
	add	r7, r2, r3
	ldr	r1, [r7, #1800]
	ubfx	r2, r1, #21, #3
	cmp	ip, r2
	bne	.L2537
	ldrh	r2, [fp]
	add	r9, r2, r9
	cmp	r1, r9
	bne	.L2537
	ldr	r0, [r4, #24]
	mvn	r9, #0
	str	r3, [sp, #12]
	add	r5, r5, r9
	ldrb	r10, [r7, #1776]	@ zero_extendqisi2
	bl	flash_start_plane_read
	mov	ip, #2
	mov	r2, #0
	strb	ip, [r4, #42]
	mov	r1, r4
	strb	r2, [r4, #43]
	ldr	r0, .L2557+20
	strb	r9, [r4]
	str	ip, [sp, #8]
	str	r2, [sp, #4]
	bl	buf_add_tail
	ldr	r1, .L2557+24
	strb	r9, [r7, #1776]
	ldr	r0, .L2557+20
	ldr	ip, [sp, #8]
	ldr	r2, [sp, #4]
	ldr	r3, [sp, #12]
	strb	ip, [r7, #1818]
	strb	r2, [r7, #1819]
	add	r1, r1, r3
	bl	buf_add_tail
	b	.L2535
.L2537:
	mov	r0, r4
	bl	queue_read_cmd
.L2535:
	subs	r5, r5, #1
	ldrne	r3, .L2557+24
	movne	r4, #48
	mlane	r4, r4, r10, r3
	bne	.L2530
.L2553:
	ldr	r4, .L2557+24
	mov	r5, #48
.L2541:
	cmp	r6, #0
	beq	.L2556
	ldrb	r3, [r8, #42]	@ zero_extendqisi2
	cmp	r3, #13
	bne	.L2542
	ldrb	r3, [r8]	@ zero_extendqisi2
	sub	r6, r6, #1
	cmp	r3, #255
	mlane	r8, r5, r3, r4
.L2542:
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2541
.L2556:
	mov	r0, r6
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2558:
	.align	2
.L2557:
	.word	.LANCHOR0+3806
	.word	.LANCHOR3
	.word	.LANCHOR1+1628
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR0+3315
	.word	.LANCHOR0+1776
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
	ldr	r3, .L2571
	mov	fp, r2
	add	r7, r0, #8
	mov	r5, r0
	mov	r6, r1
	ubfx	r8, r1, #0, #21
	ldr	r4, [r3, #528]
	ubfx	r9, r1, #24, #2
	ldr	r3, .L2571+4
	add	r7, r4, r7, asl #8
	ldr	r2, [r3]
	str	r3, [sp, #4]
	tst	r2, #16
	beq	.L2560
	ldr	r0, .L2571+8
	mov	r2, r9
	ldr	r3, [sp, #48]
	bl	printk
.L2560:
	bl	nandc_wait_flash_ready
	mov	r0, r5
	bl	hynix_reconfig_rr_para
	mov	r0, r5
	bl	nandc_cs
	cmp	r9, #0
	mov	r0, r5
	bne	.L2561
	bl	zftl_flash_enter_slc_mode
	b	.L2562
.L2561:
	bl	zftl_flash_exit_slc_mode
.L2562:
	add	r4, r4, r5, asl #8
	mov	r3, #128
	mov	r0, r8
	str	r3, [r4, #2056]
	mov	r3, #0
	str	r3, [r4, #2052]
	str	r3, [r4, #2052]
	uxtb	r3, r6
	str	r3, [r4, #2052]
	mov	r3, r8, lsr #8
	str	r3, [r4, #2052]
	mov	r3, r8, lsr #16
	str	r3, [r4, #2052]
	bl	nandc_set_seed
	mov	r2, fp
	mov	r0, #1
	ldr	r3, [sp, #4]
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	mov	r3, r10
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	mov	r3, #16
	str	r3, [r4, #2056]
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	mov	r0, r7
	bl	flash_read_status
	mov	r4, r0
	mov	r0, r5
	bl	nandc_de_cs
	ands	r4, r4, #4
	beq	.L2563
	ldr	r0, .L2571+12
	mov	r1, r6
	mov	r2, r4
	bl	printk
.L2563:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2572:
	.align	2
.L2571:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC170
	.word	.LC171
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
	ldr	r4, .L2584
	mov	r5, r1
	mov	r2, #32
	mov	r1, #165
	ldr	r0, [r4, #-3552]
	bl	memset
	mov	r1, #90
	mov	r2, #8
	ldr	r0, [r4, #-3544]
	bl	memset
	ldr	r3, .L2584+4
	mov	r0, r6
	ldrh	r3, [r3, #2]
	mul	r5, r3, r5
	mov	r1, r5
	bl	flash_erase_block
	cmn	r0, #1
	bne	.L2574
.L2576:
	mvn	r4, #0
	b	.L2575
.L2574:
	ldr	r7, .L2584+8
	mov	r0, r6
	mov	r1, r5
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	str	r3, [sp]
	ldr	r2, [r4, #-3552]
	ldr	r3, [r4, #-3544]
	bl	flash_prog_page
	cmn	r0, #1
	beq	.L2576
	ldrb	r3, [r7, #13]	@ zero_extendqisi2
	mov	r0, r6
	mov	r1, r5
	str	r3, [sp]
	ldr	r2, [r4, #-3552]
	ldr	r3, [r4, #-3544]
	bl	flash_read_page
	cmn	r0, #1
	beq	.L2576
	ldr	r3, [r4, #-3552]
	ldr	r2, [r3]
	ldr	r3, .L2584+12
	cmp	r2, r3
	bne	.L2576
	ldr	r3, [r4, #-3544]
	ldr	r4, .L2584+16
	ldr	r3, [r3]
	subs	r4, r3, r4
	mvnne	r4, #0
.L2575:
	mov	r0, r6
	mov	r1, r5
	bl	flash_erase_block
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2585:
	.align	2
.L2584:
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	-1515870811
	.word	1515870810
	.fnend
	.size	flash_test_blk, .-flash_test_blk
	.align	2
	.global	flash_start_tlc_page_prog
	.type	flash_start_tlc_page_prog, %function
flash_start_tlc_page_prog:
	.fnstart
	@ args = 12, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r10, r3
	ldr	r4, .L2593
	mov	fp, r0
	mov	r6, r1
	mov	r7, r2
	ldr	r5, [sp, #40]
	ldrb	r3, [r4, #585]	@ zero_extendqisi2
	ldr	r9, [sp, #44]
	cmp	r3, r10
	ldr	r8, [sp, #48]
	bhi	.L2587
	ldr	r1, .L2593+4
	movw	r2, #727
	ldr	r0, .L2593+8
	bl	printk
	bl	dump_stack
.L2587:
	add	r10, r4, r10
	ldr	r4, [r4, #528]
	ldrb	r10, [r10, #588]	@ zero_extendqisi2
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
	ldr	r3, .L2593+12
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
.L2594:
	.align	2
.L2593:
	.word	.LANCHOR0
	.word	.LANCHOR1+1644
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
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #16
	mov	r4, r0
	ldr	r3, [r0]
	mov	r1, #1
	ldr	r5, [r3, #24]
	ldrb	r0, [r3, #44]	@ zero_extendqisi2
	ubfx	r6, r5, #0, #21
	str	r6, [sp]
	ldr	r2, [r3, #4]
	ubfx	r5, r5, #21, #3
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldmia	r4, {r2, r3}
	mov	r1, #2
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r6, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r4, #8]
	ldr	r2, [r4]
	mov	r1, #3
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r6, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #16
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	ldr	r1, [r4]
	mov	r3, #4
	ldr	r0, .L2597
	strb	r3, [r1, #42]
	mov	r3, #1
	strb	r3, [r1, #43]
	mvn	r3, #0
	strb	r3, [r1]
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, lr}
	b	buf_add_tail
.L2598:
	.align	2
.L2597:
	.word	.LANCHOR0+3315
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
.L2600:
	mov	r0, r6
	mov	r1, #1
	bl	queue_lun_state
	subs	r5, r0, #0
	beq	.L2603
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2600
.L2603:
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
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r0]
	mov	r4, r0
	mov	r7, r1
	mov	r9, r2
	ldr	r5, [r3, #24]
.L2605:
	mov	r0, r5
	mov	r1, #1
	bl	queue_lun_state
	cmp	r0, #0
	beq	.L2610
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2605
.L2610:
	cmp	r9, #2
	bne	.L2607
	ldr	r3, [r4]
	mov	r1, #1
	ldr	r2, [r7]
	ldr	r5, [r3, #24]
	ldr	r6, [r2, #24]
	ldrb	r0, [r3, #44]	@ zero_extendqisi2
	ubfx	r8, r5, #0, #21
	str	r8, [sp]
	ubfx	r5, r5, #21, #3
	ldr	r2, [r3, #4]
	ubfx	r6, r6, #0, #21
	str	r2, [sp, #4]
	mov	r2, #17
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r7]
	ldr	r2, [r4]
	mov	r1, #1
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r6, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldmia	r4, {r2, r3}
	mov	r1, r9
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r8, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #17
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r7, #4]
	ldr	r2, [r4]
	mov	r1, r9
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r6, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #26
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r4, #8]
	ldr	r2, [r4]
	mov	r1, #3
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r8, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #17
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r7, #8]
	ldr	r2, [r4]
	mov	r1, #3
	ldrb	r0, [r2, #44]	@ zero_extendqisi2
	str	r6, [sp]
	ldr	r2, [r3, #4]
	str	r2, [sp, #4]
	mov	r2, #16
	ldr	r3, [r3, #12]
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_start_tlc_page_prog
	ldr	r1, [r4]
	mov	r3, #5
	ldr	r0, .L2611
	strb	r3, [r1, #42]
	mov	r3, #1
	strb	r3, [r1, #43]
	mvn	r3, #0
	strb	r3, [r1]
	bl	buf_add_tail
	b	.L2608
.L2607:
	mov	r0, r4
	bl	queue_tlc_prog_cmd
.L2608:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L2612:
	.align	2
.L2611:
	.word	.LANCHOR0+3315
	.fnend
	.size	sblk_xlc_prog_pages, .-sblk_xlc_prog_pages
	.align	2
	.global	sblk_3d_tlc_dump_prog
	.type	sblk_3d_tlc_dump_prog, %function
sblk_3d_tlc_dump_prog:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #16
	mov	r4, r0
	ldr	r2, [r0, #24]
	ldr	r0, .L2615
	ubfx	r5, r2, #0, #21
	ubfx	r6, r2, #21, #3
	mov	r3, r2
	mov	r1, r5
	bl	printk
	ldr	r3, [r4, #4]
	mov	r1, #1
	mov	r2, #26
	mov	r0, #0
	str	r3, [sp, #4]
	ldr	r3, [r4, #12]
	str	r5, [sp]
	str	r3, [sp, #8]
	mov	r3, r6
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r4, #4]
	mov	r1, #2
	mov	r2, #26
	mov	r0, #0
	str	r3, [sp, #4]
	ldr	r3, [r4, #12]
	str	r5, [sp]
	str	r3, [sp, #8]
	mov	r3, r6
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r3, [r4, #4]
	mov	r1, #3
	mov	r2, #16
	mov	r0, #0
	str	r3, [sp, #4]
	ldr	r3, [r4, #12]
	str	r5, [sp]
	str	r3, [sp, #8]
	mov	r3, r6
	bl	flash_start_tlc_page_prog
	bl	nandc_iqr_wait_flash_ready
	bl	nandc_wait_flash_ready
	ldr	r0, [r4, #24]
	mov	r1, #64
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, lr}
	b	flash_wait_device_ready
.L2616:
	.align	2
.L2615:
	.word	.LC172
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
	ldr	r5, .L2620
	mov	r7, r0
	mov	r4, r1
	mov	r6, r2
	ldrb	r3, [r5, #585]	@ zero_extendqisi2
	cmp	r3, r1
	bhi	.L2618
	ldr	r1, .L2620+4
	movw	r2, #757
	ldr	r0, .L2620+8
	bl	printk
	bl	dump_stack
.L2618:
	add	r4, r5, r4
	ldr	r5, [r5, #528]
	ldrb	r4, [r4, #588]	@ zero_extendqisi2
	mov	r0, r4
	add	r4, r5, r4, asl #8
	bl	nandc_cs
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
	ldr	r3, .L2620+12
	mov	r0, #1
	mov	r2, r8
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	ldr	r3, [sp, #24]
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	str	r7, [r4, #2056]
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2621:
	.align	2
.L2620:
	.word	.LANCHOR0
	.word	.LANCHOR1+1672
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
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	.pad #12
	mov	r5, r0
	ldr	r3, [r0]
	ldr	r4, [r3, #24]
.L2623:
	mov	r0, r4
	mov	r1, #1
	bl	queue_lun_state
	subs	r7, r0, #0
	beq	.L2626
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2623
.L2626:
	ldr	r3, [r5]
	ldr	r4, [r3, #24]
	ubfx	r6, r4, #0, #21
	ubfx	r4, r4, #21, #3
	mov	r0, r4
	bl	zftl_flash_exit_slc_mode
	ldr	r3, [r5]
	mov	r1, r4
	mov	r0, #16
	ldr	r2, [r3, #12]
	str	r2, [sp]
	mov	r2, r6
	ldr	r3, [r3, #4]
	bl	flash_start_3d_mlc_page_prog
	bl	nandc_wait_flash_ready
	ldr	r3, [r5, #4]
	mov	r1, r4
	mov	r0, #16
	ldr	r2, [r3, #12]
	str	r2, [sp]
	add	r2, r6, #1
	ldr	r3, [r3, #4]
	bl	flash_start_3d_mlc_page_prog
	mov	r0, r4
	bl	nandc_de_cs
	ldr	r1, [r5]
	mov	r3, #4
	ldr	r0, .L2627
	strb	r3, [r1, #42]
	mov	r3, #1
	strb	r3, [r1, #43]
	mvn	r3, #0
	strb	r3, [r1]
	bl	buf_add_tail
	mov	r0, r7
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2628:
	.align	2
.L2627:
	.word	.LANCHOR0+3315
	.fnend
	.size	sblk_3d_mlc_prog_pages, .-sblk_3d_mlc_prog_pages
	.align	2
	.global	sblk_mlc_dump_prog
	.type	sblk_mlc_dump_prog, %function
sblk_mlc_dump_prog:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
	.pad #8
	mov	r5, r0
	ldr	r4, [r0, #24]
	ubfx	r6, r4, #0, #21
	ubfx	r4, r4, #21, #3
	mov	r0, r4
	bl	zftl_flash_exit_slc_mode
	ldr	r2, [r5, #24]
	mov	r1, r6
	ldr	r0, .L2631
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
.L2632:
	.align	2
.L2631:
	.word	.LC173
	.fnend
	.size	sblk_mlc_dump_prog, .-sblk_mlc_dump_prog
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
	mov	r10, r2
	ldr	r7, .L2643
	ubfx	r4, r1, #21, #3
	mov	r8, r0
	mov	r9, r3
	ubfx	r5, r1, #0, #21
	ubfx	fp, r1, #24, #2
	ldrb	r2, [r7, #585]	@ zero_extendqisi2
	cmp	r2, r4
	bhi	.L2634
	ldr	r1, .L2643+4
	mov	r2, #804
	ldr	r0, .L2643+8
	bl	printk
	bl	dump_stack
.L2634:
	ldr	r3, [r7, #528]
	add	r2, r7, r4
	ldrb	r6, [r2, #588]	@ zero_extendqisi2
	str	r3, [sp, #4]
	bl	nandc_rdy_status
	cmp	r0, #0
	bne	.L2635
	ldr	r2, .L2643
	ldrb	r2, [r2, #585]	@ zero_extendqisi2
	cmp	r2, #1
	bne	.L2636
	bl	nandc_wait_flash_ready
	b	.L2635
.L2636:
	mov	r0, r4
	mov	r1, r5
	mov	r2, #64
	bl	flash_wait_device_ready_raw
.L2635:
	mov	r0, r6
	bl	hynix_reconfig_rr_para
	mov	r0, r6
	bl	nandc_cs
	cmp	fp, #0
	bne	.L2637
	mov	r0, r5
	bl	slc_phy_page_address_calc
	ldrb	r3, [r7]	@ zero_extendqisi2
	cmp	r3, #0
	mov	r5, r0
	beq	.L2638
	mov	r0, r6
	bl	zftl_flash_enter_slc_mode
	b	.L2638
.L2637:
	mov	r0, r6
	bl	zftl_flash_exit_slc_mode
.L2638:
	ldr	r3, [sp, #4]
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
	ldr	r3, .L2643+12
	mov	r0, #1
	mov	r2, r10
	ldrb	r1, [r3, #13]	@ zero_extendqisi2
	mov	r3, r9
	bl	nandc_xfer_start
	bl	nandc_xfer_done
	str	r8, [r4, #2056]
	mov	r0, r6
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	b	nandc_de_cs
.L2644:
	.align	2
.L2643:
	.word	.LANCHOR0
	.word	.LANCHOR1+1704
	.word	.LC0
	.word	.LANCHOR2
	.fnend
	.size	flash_start_page_prog, .-flash_start_page_prog
	.align	2
	.type	queue_prog_cmd, %function
queue_prog_cmd:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
	mov	r4, r0
	ldr	r1, [r4, #24]
	mov	r0, #16
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #12]
	bl	flash_start_page_prog
	ldr	r2, .L2654
	ldr	r1, [r4, #24]
	ldrb	r3, [r2, #3315]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2646
	ubfx	r1, r1, #21, #3
	mov	lr, #48
.L2648:
	mla	ip, lr, r3, r2
	ldr	r0, [ip, #1800]
	ubfx	r0, r0, #21, #3
	cmp	r1, r0
	bne	.L2647
	add	r0, ip, #1808
	ldrb	ip, [ip, #1818]	@ zero_extendqisi2
	add	r0, r0, #8
	cmp	ip, #7
	moveq	r3, #3
	streqb	r3, [r0, #2]
	beq	.L2646
.L2647:
	mla	r3, lr, r3, r2
	ldrb	r3, [r3, #1776]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L2648
.L2646:
	mov	r3, #3
	mov	r1, r4
	strb	r3, [r4, #42]
	mov	r3, #1
	ldr	r0, .L2654+4
	strb	r3, [r4, #43]
	mvn	r3, #0
	strb	r3, [r4]
	ldmfd	sp!, {r4, lr}
	b	buf_add_tail
.L2655:
	.align	2
.L2654:
	.word	.LANCHOR0
	.word	.LANCHOR0+3315
	.fnend
	.size	queue_prog_cmd, .-queue_prog_cmd
	.align	2
	.global	sblk_prog_page
	.type	sblk_prog_page, %function
sblk_prog_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r4, r0
	ldrh	r3, [r0, #34]
	mov	r5, r1
	cmp	r3, #0
	beq	.L2657
	ldr	r3, .L2685
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L2657
	ldr	r0, .L2685+4
	mov	r2, r5
	ldr	r1, [r4, #24]
	bl	printk
.L2657:
	ldr	r8, .L2685+8
	mov	r6, #0
.L2658:
	cmp	r5, #0
	beq	.L2682
	ldrb	r9, [r4]	@ zero_extendqisi2
	ldr	r7, [r4, #24]
.L2659:
	mov	r0, r7
	mov	r1, #1
	bl	queue_lun_state
	cmp	r0, #0
	beq	.L2684
	bl	queue_wait_first_req_completed
	bl	queue_remove_completed_req
	b	.L2659
.L2684:
	cmp	r5, #1
	beq	.L2661
	ldrb	r3, [r8, #1775]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L2662
.L2661:
	mov	r0, r4
	bl	queue_prog_cmd
	b	.L2663
.L2662:
	ldrb	r3, [r4]	@ zero_extendqisi2
	ubfx	r10, r7, #21, #3
	cmp	r3, #255
	bne	.L2664
	ldr	r1, .L2685+12
	movw	r2, #583
	ldr	r0, .L2685+16
	bl	printk
	bl	dump_stack
.L2664:
	ldrb	r3, [r4]	@ zero_extendqisi2
	mov	r2, #48
	ldr	r1, .L2685+8
	mla	r3, r2, r3, r1
	ldr	ip, [r3, #1800]
	ubfx	r3, ip, #21, #3
	cmp	r10, r3
	bne	.L2665
	ldr	r3, .L2685+20
	mov	r10, #1
	ldrb	lr, [r8, #3766]	@ zero_extendqisi2
	ldrh	fp, [r3]
	sub	lr, lr, #1
	rsb	r3, fp, #21
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
	beq	.L2666
	cmp	r6, lr
	beq	.L2665
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
	ldr	r0, .L2685+24
	strb	r3, [r4]
	bl	buf_add_tail
	b	.L2663
.L2666:
	mov	r0, r4
	mov	r6, fp
	bl	queue_prog_cmd
	b	.L2663
.L2665:
	mov	r0, r4
	mov	r6, #0
	bl	queue_prog_cmd
.L2663:
	subs	r5, r5, #1
	ldrne	r3, .L2685+28
	movne	r4, #48
	mlane	r4, r4, r9, r3
	bne	.L2658
.L2682:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2686:
	.align	2
.L2685:
	.word	.LANCHOR2
	.word	.LC174
	.word	.LANCHOR0
	.word	.LANCHOR1+1728
	.word	.LC0
	.word	.LANCHOR0+3764
	.word	.LANCHOR0+3315
	.word	.LANCHOR0+1776
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
	ldr	r4, .L2693
	ldr	r5, .L2693+4
	ldrb	r1, [r4, #3337]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L2688
	ldrb	r2, [r5, #-3519]	@ zero_extendqisi2
	add	r3, r4, #1776
	mov	r0, #48
	mla	r0, r0, r2, r3
	bl	sblk_prog_page
.L2688:
	mvn	r3, #0
	strb	r3, [r5, #-3519]
	mov	r3, #0
	strb	r3, [r4, #3337]
	bl	sblk_wait_write_queue_completed
	bl	ftl_write_completed
	movw	r0, #65535
	ldmfd	sp!, {r3, r4, r5, lr}
	b	ftl_vpn_decrement
.L2694:
	.align	2
.L2693:
	.word	.LANCHOR0
	.word	.LANCHOR3
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
	ldr	r3, .L2700
	ldrb	r3, [r3, #3337]	@ zero_extendqisi2
	cmp	r3, #0
	ldmeqfd	sp!, {r3, pc}
	bl	timer_get_time
	ldr	r3, .L2700+4
	ldr	r3, [r3, #-3516]
	add	r3, r3, #100
	cmp	r0, r3
	ldmlsfd	sp!, {r3, pc}
	ldmfd	sp!, {r3, lr}
	b	ftl_flush
.L2701:
	.align	2
.L2700:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.fnend
	.size	zftl_cache_flush, .-zftl_cache_flush
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
	ldr	r3, .L2717
	mov	r6, r0
	ldr	r7, .L2717+4
	mov	r8, r1
	mov	r10, r2
	ubfx	r5, r1, #0, #21
	ldr	r4, [r3, #528]
	add	r3, r0, #8
	ubfx	fp, r1, #24, #2
	add	r3, r4, r3, asl #8
	str	r3, [sp]
	ldr	r3, [r7]
	tst	r3, #16
	beq	.L2703
	ldr	r0, .L2717+8
	mov	r2, fp
	ldr	r3, [sp, #56]
	bl	printk
.L2703:
	bl	nandc_wait_flash_ready
	mov	r0, r6
	bl	nandc_cs
	cmp	fp, #0
	mov	r0, r6
	bne	.L2704
	bl	zftl_flash_enter_slc_mode
	b	.L2705
.L2704:
	bl	zftl_flash_exit_slc_mode
.L2705:
	add	r4, r4, r6, asl #8
	mov	ip, #128
	mov	fp, #0
	uxtb	r3, r8
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
	beq	.L2706
	ldr	r3, .L2717+4
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L2706
	ldr	r0, .L2717+12
	mov	r1, r8
	mov	r2, r4
	bl	printk
.L2706:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2718:
	.align	2
.L2717:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC170
	.word	.LC171
	.fnend
	.size	flash_dual_page_prog, .-flash_dual_page_prog
	.section	.text.unlikely
	.align	2
	.type	fw_flash_page_prog.constprop.29, %function
fw_flash_page_prog.constprop.29:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r3, r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	.pad #16
	mov	r6, r1
	ldr	r4, .L2724
	mov	r5, r2
	ldr	r3, [r4, #1620]
	ldrb	r1, [r3, #9]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldrb	r8, [r4, #584]	@ zero_extendqisi2
	mov	r7, r0
	ldrb	r0, [r4, #1624]	@ zero_extendqisi2
	bl	nandc_bch_sel
	ldr	r3, .L2724+4
	ldrb	r3, [r3, #11]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L2720
	ldr	r4, .L2724+8
	mov	r1, #255
	mov	r2, #16384
	ldr	r0, [r4, #-3552]
	bl	memset
	ldr	r3, [r4, #-3552]
	mov	r1, r7
	mov	r2, r6
	mov	r0, #0
	str	r3, [sp]
	str	r3, [sp, #4]
	mov	r3, #4
	str	r3, [sp, #8]
	mov	r3, r5
	bl	flash_dual_page_prog
	b	.L2723
.L2720:
	mov	r3, #4
	mov	r0, #0
	str	r3, [sp]
	mov	r1, r7
	mov	r2, r6
	mov	r3, r5
	bl	flash_prog_page
.L2723:
	mov	r4, r0
	mov	r0, r8
	bl	nandc_bch_sel
	mov	r0, r4
	add	sp, sp, #16
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2725:
	.align	2
.L2724:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.fnend
	.size	fw_flash_page_prog.constprop.29, .-fw_flash_page_prog.constprop.29
	.text
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
	mov	r7, r3
	ldr	r6, .L2754
	ldrb	r3, [sp, #68]	@ zero_extendqisi2
	mov	r9, r0
	mov	r5, r1
	mov	r8, r2
	ubfx	r4, r1, #24, #2
	str	r3, [sp, #16]
	ldrb	r3, [r6, #585]	@ zero_extendqisi2
	cmp	r3, r0
	bhi	.L2727
	ldr	r1, .L2754+4
	movw	r2, #547
	ldr	r0, .L2754+8
	bl	printk
	bl	dump_stack
.L2727:
	add	r2, r6, r9
	cmp	r4, #0
	ldr	r3, .L2754
	ldrb	r10, [r2, #588]	@ zero_extendqisi2
	bne	.L2738
	ldrb	r2, [r3]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2729
	ldrb	r3, [r3, #1]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2738
.L2729:
	ldrh	fp, [r6, #2]
	mov	r0, r5
	mov	r1, fp
	bl	__aeabi_uidiv
	ldrb	r3, [r6, #1]	@ zero_extendqisi2
	cmp	r3, #0
	ldreq	r3, .L2754
	mul	r0, fp, r0
	rsb	r4, r0, r5
	mov	r4, r4, asl #1
	addeq	r4, r3, r4
	ldreqh	r4, [r4, #4]
	add	r4, r4, r0
	b	.L2728
.L2738:
	mov	r4, r5
.L2728:
	ldr	fp, .L2754+12
	ldr	r6, .L2754+16
.L2734:
	ldr	r3, [fp]
	tst	r3, #16
	beq	.L2731
	ldr	r0, .L2754+20
	mov	r1, r10
	mov	r2, r5
	mov	r3, r4
	bl	printk
.L2731:
	ldr	r3, [sp, #64]
	mov	r0, r10
	mov	r1, r4
	mov	r2, r8
	str	r3, [sp]
	mov	r3, r7
	bl	flash_prog_page
	ldr	r3, [sp, #16]
	mov	ip, r0
	cmp	r3, #0
	beq	.L2732
	ldr	r3, [sp, #64]
	mov	r1, r5
	str	r0, [sp, #20]
	mov	r0, r9
	str	r3, [sp]
	ldr	r3, [r6, #-3524]
	ldr	r2, [r6, #-3528]
	bl	flash_read_page_en
	sub	r3, r0, #512
	cmp	r0, #512
	cmnne	r0, #1
	clz	r3, r3
	mov	r3, r3, lsr #5
	str	r3, [sp, #12]
	beq	.L2733
	ldr	r3, [r6, #-3528]
	ldr	r2, [r8]
	ldr	ip, [sp, #20]
	ldr	r3, [r3]
	cmp	r2, r3
	bne	.L2733
	ldr	r3, [r6, #-3524]
	ldr	r2, [r7]
	ldr	r3, [r3]
	cmp	r2, r3
	beq	.L2732
.L2733:
	mov	r2, #4
	ldr	r0, .L2754+24
	mov	r3, r2
	mov	r1, r8
	bl	rknand_print_hex
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2754+28
	mov	r1, r7
	bl	rknand_print_hex
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2754+32
	ldr	r1, [r6, #-3524]
	bl	rknand_print_hex
	mov	r2, #4
	mov	r3, r2
	ldr	r0, .L2754+36
	ldr	r1, [r6, #-3528]
	bl	rknand_print_hex
	ldr	r3, [sp, #12]
	cmp	r3, #0
	bne	.L2734
.L2736:
	mov	r1, r5
	ldr	r0, .L2754+40
	bl	printk
	ldr	r1, .L2754+4
	movw	r2, #578
	ldr	r0, .L2754+8
	bl	printk
	bl	dump_stack
	mvn	r0, #0
	b	.L2735
.L2732:
	cmn	ip, #1
	mov	r0, ip
	beq	.L2736
.L2735:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2755:
	.align	2
.L2754:
	.word	.LANCHOR0
	.word	.LANCHOR1+1744
	.word	.LC0
	.word	.LANCHOR2
	.word	.LANCHOR3
	.word	.LC175
	.word	.LC176
	.word	.LC177
	.word	.LC178
	.word	.LC179
	.word	.LC180
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
	bne	.L2757
	ldr	r1, .L2759
	movw	r2, #2231
	ldr	r0, .L2759+4
	bl	printk
	bl	dump_stack
	ldr	r0, .L2759+8
	mov	r1, r5
	bl	printk
.L2757:
	mov	r0, r4
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L2760:
	.align	2
.L2759:
	.word	.LANCHOR1+1764
	.word	.LC0
	.word	.LC180
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
	ldr	r5, .L2795
	.pad #20
	sub	sp, sp, #20
	mov	fp, r0
	ldr	r0, .L2795+4
	ldr	r7, .L2795+8
	mov	r10, #0
	ldrb	r2, [r5, #3312]	@ zero_extendqisi2
	mov	r6, r7
	mov	r2, r2, asl #1
	bl	ftl_memset
.L2762:
	ldr	r3, .L2795+12
	ldr	lr, [r5, #3340]
	ldr	r8, .L2795+16
	ldrh	r3, [r3]
	ldrb	r4, [r7, #-3256]	@ zero_extendqisi2
	ldrh	ip, [r8, #2]
	str	r3, [sp, #8]
	ldr	r3, [lr, #4]
	ldr	r9, .L2795+12
	add	r3, r3, #1
	str	r3, [lr, #4]
	str	r3, [r7, #-3508]
	ldr	r3, .L2795+20
	str	fp, [r7, #-3512]
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L2763
	mov	r2, ip
	ldr	r0, .L2795+24
	mov	r1, r4
	ldr	r3, [lr, #4]
	str	ip, [sp, #12]
	bl	printk
	ldr	ip, [sp, #12]
.L2763:
	ldr	r3, .L2795+28
	ldrh	r0, [r8, #2]
	ldrh	r3, [r3]
	cmp	r3, r0
	bhi	.L2764
.L2769:
	ldrb	r3, [r6, #-3255]	@ zero_extendqisi2
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [r6, #-3255]
	cmp	r3, #7
	bls	.L2765
	mov	r3, #8
.L2768:
	ldr	r2, [r5, #1696]
	sub	r4, r3, #8
	add	r2, r2, r3
	uxth	r4, r4
	ldrb	r1, [r2, #32]	@ zero_extendqisi2
	add	r2, r1, #127
	uxtb	r2, r2
	cmp	r2, #125
	bhi	.L2766
	ldr	r1, .L2795+32
	mov	r2, #748
	ldr	r0, .L2795+36
	bl	printk
	bl	dump_stack
	b	.L2767
.L2766:
	cmp	r1, #255
	bne	.L2767
	add	r3, r3, #1
	cmp	r3, #16
	bne	.L2768
	mov	r4, #8
.L2767:
	mov	r10, #1
	strb	r4, [r6, #-3255]
.L2765:
	ldr	r3, [r5, #1696]
	ldrb	r2, [r6, #-3255]	@ zero_extendqisi2
	add	r3, r3, r2
	ldrb	r3, [r3, #40]	@ zero_extendqisi2
	cmp	r3, #255
	strb	r3, [r6, #-3256]
	beq	.L2769
	ldrh	r4, [r9]
	mov	r0, #0
	mul	r4, r4, r3
	mov	r1, r4
	bl	flash_erase_block
	ldr	r3, .L2795
	ldr	r2, .L2795+40
	mov	r1, r4
	mov	r0, #0
	ldrb	r3, [r3, #3312]	@ zero_extendqisi2
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_prog_page
	ldr	r3, .L2795+16
	mov	r2, #1
	add	r4, r4, r2
	strh	r2, [r3, #2]	@ movhi
	b	.L2770
.L2764:
	ldr	r3, [sp, #8]
	cmp	r0, #0
	mla	r4, r3, r4, ip
	bne	.L2770
	mov	r1, r4
	bl	flash_erase_block
.L2770:
	ldrb	r3, [r5, #3312]	@ zero_extendqisi2
	mov	r0, #0
	ldr	r2, .L2795+40
	mov	r1, r4
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_prog_page
	ldrh	r3, [r8, #2]
	add	r3, r3, #1
	strh	r3, [r8, #2]	@ movhi
	ldr	r3, .L2795+44
	cmn	r0, #1
	beq	.L2771
	ldrb	r2, [r3, #-3176]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L2772
.L2771:
	mov	r2, #0
	strb	r2, [r3, #-3176]
	b	.L2762
.L2772:
	cmp	r10, #1
	beq	.L2773
.L2781:
	ldr	r3, .L2795+8
	ldrb	r3, [r3, #-3256]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L2775
	ldr	r1, .L2795+32
	mov	r2, #784
	ldr	r0, .L2795+36
	bl	printk
	bl	dump_stack
	b	.L2775
.L2773:
	ldr	r3, .L2795+8
	ldrb	r4, [r3, #-3255]	@ zero_extendqisi2
	add	r4, r4, #1
.L2776:
	cmp	r4, #7
	bhi	.L2781
	ldr	r3, [r5, #1696]
	add	r3, r3, r4
	ldrb	r6, [r3, #40]	@ zero_extendqisi2
	add	r3, r6, #127
	uxtb	r3, r3
	cmp	r3, #125
	bhi	.L2777
	ldr	r1, .L2795+32
	movw	r2, #777
	ldr	r0, .L2795+36
	bl	printk
	bl	dump_stack
	b	.L2778
.L2777:
	cmp	r6, #255
	beq	.L2779
.L2778:
	ldrh	r1, [r9]
	mov	r0, #0
	mul	r1, r1, r6
	bl	flash_erase_block
.L2779:
	add	r4, r4, #1
	uxth	r4, r4
	b	.L2776
.L2775:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2796:
	.align	2
.L2795:
	.word	.LANCHOR0
	.word	.LANCHOR3-3512
	.word	.LANCHOR3
	.word	.LANCHOR0+3806
	.word	.LANCHOR3-3256
	.word	.LANCHOR2
	.word	.LC181
	.word	.LANCHOR0+3712
	.word	.LANCHOR1+1780
	.word	.LC0
	.word	.LANCHOR3-3192
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
	ldr	r3, .L2829
	mov	r2, #1
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	r7, #0
	ldr	r4, .L2829+4
	mov	r1, #16384
	strb	r2, [r3, #-3176]
	.pad #20
	sub	sp, sp, #20
	strb	r2, [r3, #-3174]
	mov	r6, #7
	strb	r7, [r3, #-3175]
	add	r3, r4, #564
	ldr	r0, .L2829+8
	mov	r9, r4
	ldrh	r3, [r3]
	ldr	r5, .L2829+12
	str	r0, [r4, #568]
	add	r3, r0, r3, asl #2
	str	r3, [r4, #3340]
	bl	__memzero
	ldr	r0, .L2829+16
	mov	r1, #16384
	ldr	r10, .L2829+20
	bl	__memzero
	ldr	r3, [r4, #1696]
	ldr	fp, .L2829+24
	strb	r7, [r5, #-3255]
	ldrb	r3, [r3, #40]	@ zero_extendqisi2
	strb	r3, [r5, #-3256]
	ldr	r3, .L2829+28
	strh	r7, [r3, #2]	@ movhi
	str	r3, [sp, #8]
.L2802:
	ldr	r3, [r4, #1696]
	uxth	r7, r6
	add	r3, r3, r6
	ldrb	r3, [r3, #40]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L2798
	ldrh	ip, [r10]
	mov	r0, #0
	ldr	r2, .L2829+8
	mul	ip, ip, r3
	ldrb	r3, [r9, #3312]	@ zero_extendqisi2
	str	r3, [sp]
	sub	r3, r2, #320
	mov	r1, ip
	str	ip, [sp, #12]
	bl	ftl_read_page
	cmn	r0, #1
	mov	r8, r0
	ldr	ip, [sp, #12]
	bne	.L2799
	ldrb	r3, [r9, #3312]	@ zero_extendqisi2
	mov	r0, #0
	ldr	r2, .L2829+8
	add	r1, ip, #1
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_read_page
	mov	r8, r0
.L2799:
	ldr	r3, .L2829+32
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L2800
	ldr	r3, [r4, #3340]
	mov	r1, r6
	ldr	r0, .L2829+36
	mov	r2, r8
	ldr	r3, [r3]
	str	r3, [sp]
	mov	r3, #672
	bl	printk
.L2800:
	cmn	r8, #1
	beq	.L2798
	ldr	r3, [r4, #3340]
	ldr	r3, [r3]
	cmp	r3, fp
	beq	.L2811
.L2798:
	subs	r6, r6, #1
	bcs	.L2802
	movw	r8, #65535
	mov	r7, #0
	b	.L2801
.L2811:
	mov	r8, r7
.L2801:
	ldr	r6, .L2829+32
	sxth	r8, r8
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2803
	ldr	r3, [r4, #3340]
	mov	r1, r8
	ldr	r0, .L2829+40
	mov	r2, #4800
	ldr	r3, [r3]
	bl	printk
.L2803:
	cmn	r8, #1
	bne	.L2804
	ldr	r0, .L2829+8
	mov	r1, #16384
	bl	__memzero
	ldr	r3, [r4, #3340]
	ldr	r2, .L2829+24
	mov	r0, r8
	str	r2, [r3]
	ldr	r2, .L2829+44
	ldr	r3, [r4, #3340]
	str	r2, [r3, #12]
	b	.L2805
.L2804:
	ldr	r3, [r4, #1696]
	mov	r0, #0
	ldr	r2, .L2829+8
	sxtah	r3, r3, r7
	strb	r7, [r5, #-3255]
	ldr	r10, .L2829+4
	ldrb	r1, [r3, #40]	@ zero_extendqisi2
	mov	r3, #4
	ldr	fp, .L2829+24
	strb	r1, [r5, #-3256]
	str	r3, [sp]
	sub	r3, r2, #320
	bl	flash_get_last_written_page
	movw	r2, #3806
	ldrb	r3, [r5, #-3256]	@ zero_extendqisi2
	mov	r5, #0
	ldrh	r7, [r4, r2]
	mul	r7, r7, r3
	uxth	r8, r0
	mov	r9, r0
	add	ip, r8, #1
	uxth	ip, ip
.L2806:
	rsb	r3, r5, r8
	tst	r3, #32768
	bne	.L2810
	ldrb	r3, [r4, #3312]	@ zero_extendqisi2
	add	r1, r9, r7
	ldr	r2, .L2829+8
	mov	r0, #0
	rsb	r1, r5, r1
	str	ip, [sp, #12]
	str	r3, [sp]
	sub	r3, r2, #320
	bl	ftl_read_page
	cmn	r0, #1
	ldr	ip, [sp, #12]
	beq	.L2807
	ldr	r3, [r10, #3340]
	ldr	r3, [r3]
	cmp	r3, fp
	bne	.L2807
.L2810:
	ldr	r3, [sp, #8]
	strh	ip, [r3, #2]	@ movhi
	bl	ftl_tmp_into_update
	ldr	r2, [r4, #3340]
	mov	r0, #0
	ldr	r3, [r2, #64]
	add	r3, r3, #1
	str	r3, [r2, #64]
	bl	ftl_info_flush
	mov	r0, #0
	bl	ftl_info_flush
	ldr	r0, [r6]
	ands	r0, r0, #16384
	beq	.L2805
	ldr	r3, .L2829+4
	ldr	r0, .L2829+48
	ldr	r3, [r3, #3340]
	ldr	r1, [r3, #156]
	bl	printk
	mov	r0, #0
	b	.L2805
.L2807:
	add	r5, r5, #1
	b	.L2806
.L2805:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2830:
	.align	2
.L2829:
	.word	.LANCHOR4
	.word	.LANCHOR0
	.word	.LANCHOR3-3192
	.word	.LANCHOR3
	.word	.LANCHOR4-3112
	.word	.LANCHOR0+3806
	.word	1229739078
	.word	.LANCHOR3-3256
	.word	.LANCHOR2
	.word	.LC182
	.word	.LC183
	.word	393238
	.word	.LC184
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
	ldr	r3, .L2847
	ldr	r4, .L2847+4
	umull	r0, r1, r0, r3
	ldr	r3, [r4, #580]
	ldr	r0, [r3, #520]
	mov	r1, r1, lsr #5
	cmp	r1, r0
	ldrhi	r2, [r3, #12]
	rsbhi	r2, r0, r2
	addhi	r2, r2, r1
	strhi	r2, [r3, #12]
	bhi	.L2846
.L2832:
	bcs	.L2833
.L2846:
	str	r1, [r3, #520]
.L2833:
	mov	r0, #0
	ldr	r6, .L2847+8
	bl	ftl_total_vpn_update
	ldr	r8, .L2847+12
	mov	r7, r6
.L2837:
	ldr	r3, [r4, #3340]
	ldr	r2, [r3, #56]
	add	r2, r2, #1
	str	r2, [r3, #56]
	ldrh	r2, [r3, #140]
	ldrh	r3, [r8]
	cmp	r2, r3
	bcc	.L2835
	bl	ftl_ext_alloc_new_blk
.L2835:
	ldr	r3, [r4, #3340]
	ldrh	r2, [r3, #130]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2836
	ldr	r1, .L2847+16
	movw	r2, #1819
	ldr	r0, .L2847+20
	bl	printk
	bl	dump_stack
.L2836:
	ldr	r1, [r4, #3340]
	mvn	ip, #0
	ldr	r0, .L2847+24
	ldr	r10, .L2847+28
	ldrh	r3, [r1, #130]
	ldrh	r2, [r0]
	ldrh	r1, [r1, #140]
	rsb	r2, r2, #21
	mov	r9, r3, asr r2
	bic	r3, r3, ip, asl r2
	ldrh	r2, [r0, #42]
	ldr	r0, .L2847+32
	smlabb	r3, r2, r3, r1
	ldrb	r2, [r4, #3312]	@ zero_extendqisi2
	mov	r1, #0
	mov	r2, r2, asl #1
	uxth	r5, r3
	bl	ftl_memset
	mov	r3, #0
	str	r3, [r10, #-3512]
	ldr	r3, [r4, #3340]
	uxtb	r0, r9
	ldr	r2, .L2847+36
	mov	r1, r5
	ldr	r3, [r3, #56]
	str	r3, [r10, #-3508]
	ldrb	r3, [r4, #3312]	@ zero_extendqisi2
	str	r3, [sp]
	ldr	r3, .L2847+32
	bl	ftl_prog_page
	ldr	r2, [r4, #3340]
	ldrh	r3, [r2, #140]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r2, #140]	@ movhi
	cmp	r3, #1
	beq	.L2837
	cmn	r0, #1
	beq	.L2838
	ldrb	r0, [r6, #-3174]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L2839
.L2838:
.L2834:
	mov	r3, #0
	strb	r3, [r7, #-3174]
	b	.L2837
.L2839:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L2848:
	.align	2
.L2847:
	.word	1374389535
	.word	.LANCHOR0
	.word	.LANCHOR4
	.word	.LANCHOR0+3712
	.word	.LANCHOR1+1796
	.word	.LC0
	.word	.LANCHOR0+3764
	.word	.LANCHOR3
	.word	.LANCHOR3-3512
	.word	.LANCHOR4-3112
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
	ldr	r3, .L2864
	mvn	r5, #0
	ldr	r4, .L2864+4
	.pad #20
	sub	sp, sp, #20
	strh	r2, [r3, #-12]	@ movhi
	add	r3, r3, #328
	ldr	r2, [r4, #3340]
	ldrh	r1, [r2, #130]
	movw	r2, #3764
	ldrh	r2, [r4, r2]
	rsb	r2, r2, #21
	mov	r6, r1, asr r2
	bic	r5, r1, r5, asl r2
	mov	r2, #4
	str	r2, [sp]
	uxtb	r9, r6
	mov	r1, r5
	ldr	r2, .L2864+8
	mov	r0, r9
	bl	flash_get_last_written_page
	ldr	r3, .L2864+12
	ldr	r3, [r3]
	tst	r3, #4096
	mov	r7, r0
	beq	.L2850
	ldr	r3, [r4, #3340]
	uxth	r6, r6
	str	r6, [sp]
	movw	r2, #1863
	ldr	r0, .L2864+16
	ldrh	r3, [r3, #130]
	ldr	r1, .L2864+20
	str	r3, [sp, #4]
	mov	r3, r7
	bl	printk
.L2850:
	ldr	r10, .L2864+4
	mov	r8, #0
	ldr	fp, .L2864+24
	ldr	ip, .L2864+28
.L2851:
	uxth	r6, r7
	rsb	r3, r8, r6
	tst	r3, #32768
	bne	.L2856
	ldrh	r3, [fp]
	rsb	r1, r8, r7
	ldrb	r2, [r4, #3312]	@ zero_extendqisi2
	mov	r0, r9
	str	ip, [sp, #12]
	mla	r1, r3, r5, r1
	str	r2, [sp]
	ldr	r3, .L2864+32
	ldr	r2, .L2864+8
	bl	flash_read_page_en
	cmp	r0, #512
	cmnne	r0, #1
	ldr	ip, [sp, #12]
	beq	.L2852
	ldr	r3, [r10, #580]
	ldr	r3, [r3]
	cmp	r3, ip
	bne	.L2852
.L2856:
	bl	zftl_sblk_list_init
	ldr	r3, [r4, #3340]
	ldrh	r2, [r3, #140]
	cmp	r2, r7
	bgt	.L2854
	add	r6, r6, #1
	strh	r6, [r3, #140]	@ movhi
	bl	ftl_ext_info_flush
	b	.L2854
.L2852:
	add	r8, r8, #1
	b	.L2851
.L2854:
	ldr	r5, [r4, #580]
	bl	timer_get_time
	ldr	r3, .L2864+36
	umull	r0, r1, r0, r3
	mov	r3, r1, lsr #5
	str	r3, [r5, #520]
	ldr	r5, [r4, #580]
	bl	timer_get_time
	ldr	r3, [r4, #580]
	mvn	r2, #0
	add	r1, r3, #584
	str	r0, [r5, #604]
	mov	r0, #0
	str	r0, [r3, #608]
	strh	r2, [r1]	@ movhi
	movw	r1, #586
	strh	r2, [r3, r1]	@ movhi
	add	r1, r3, #588
	strh	r2, [r1]	@ movhi
	movw	r1, #590
	strh	r2, [r3, r1]	@ movhi
	movw	r1, #65535
	str	r2, [r3, #564]
	str	r1, [r3, #560]
	movw	r3, #3746
	strh	r2, [r4, r3]	@ movhi
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2865:
	.align	2
.L2864:
	.word	.LANCHOR3-3840
	.word	.LANCHOR0
	.word	.LANCHOR4-3112
	.word	.LANCHOR2
	.word	.LC185
	.word	.LANCHOR1+1816
	.word	.LANCHOR0+3806
	.word	1162432070
	.word	.LANCHOR3-3512
	.word	1374389535
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
	mov	r4, r1
	mov	ip, r0
	str	r3, [sp]
	ubfx	r1, ip, #0, #21
	mov	r2, r4
	mov	r3, lr
	ubfx	r0, r0, #21, #3
	bl	ftl_prog_page
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, pc}
	.fnend
	.size	ftl_prog_ppa_page, .-ftl_prog_ppa_page
	.align	2
	.global	ftl_write_last_log_page
	.type	ftl_write_last_log_page, %function
ftl_write_last_log_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L2873
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
	mov	r4, r0
	ldr	r5, [r3, #-3860]
	ldrh	r3, [r0, #6]
	ldrh	r6, [r0, #12]
	cmp	r3, #1
	bne	.L2870
	bl	ftl_get_new_free_page
	cmn	r0, #1
	mov	r8, r0
	beq	.L2871
	ldrh	r0, [r4]
	add	r5, r5, r6, asl #2
	bl	ftl_vpn_decrement
	ldr	r6, .L2873+4
	ldr	r3, .L2873+8
	mov	r0, r5
	ldr	r7, .L2873+12
	mov	r4, #0
	str	r3, [r6, #-3096]
	add	r3, r7, #3712
	ldrh	r1, [r3]
	ldrb	r3, [r7, #3720]	@ zero_extendqisi2
	mul	r1, r3, r1
	mov	r1, r1, asl #2
	bl	js_hash
	mov	r1, r5
	ldr	r2, .L2873+16
	ldrb	r3, [r7, #3312]	@ zero_extendqisi2
	str	r4, [r6, #-3088]
	str	r4, [r6, #-3084]
	str	r0, [r6, #-3092]
	mov	r0, r8
	bl	ftl_prog_ppa_page
	b	.L2871
.L2870:
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2871:
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L2874:
	.align	2
.L2873:
	.word	.LANCHOR3
	.word	.LANCHOR5
	.word	-178307901
	.word	.LANCHOR0
	.word	.LANCHOR5-3096
	.fnend
	.size	ftl_write_last_log_page, .-ftl_write_last_log_page
	.align	2
	.global	ftl_open_sblk_recovery
	.type	ftl_open_sblk_recovery, %function
ftl_open_sblk_recovery:
	.fnstart
	@ args = 0, pretend = 0, frame = 88
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #100
	sub	sp, sp, #100
	ldr	r6, .L2985
	mov	r4, r0
	str	r1, [sp, #24]
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2876
	ldr	r0, .L2985+4
	ldrh	r1, [r4, #2]
	bl	printk
.L2876:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2877
	ldr	r0, .L2985+8
	ldrb	r1, [r4, #5]	@ zero_extendqisi2
	bl	printk
.L2877:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2878
	ldr	r0, .L2985+12
	ldrh	r1, [r4]
	bl	printk
.L2878:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L2879
	ldr	r0, .L2985+16
	ldrh	r1, [r4, #16]
	ldrh	r2, [r4, #18]
	bl	printk
.L2879:
	ldrh	r3, [r4, #10]
	ldr	r8, .L2985+20
	ldrh	r2, [r4]
	strh	r3, [r4, #14]	@ movhi
	ldrh	r3, [r8]
	cmp	r2, r3
	bcs	.L2875
	mov	r0, #1
	sub	r8, r8, #564
	bl	buf_alloc
	mov	r1, #255
	mov	r2, #64
	mov	r5, r0
	add	r0, sp, #32
	bl	memset
	ldrb	r7, [r4, #5]	@ zero_extendqisi2
	ldrh	r9, [r4, #2]
	mov	r3, #2
	str	r3, [sp, #12]
	mov	r3, #0
	str	r3, [sp, #16]
.L2881:
	ldr	r3, .L2985+24
	ldrh	r2, [r3]
	cmp	r2, r9
	bls	.L2884
	ldrb	r7, [r4, #5]	@ zero_extendqisi2
	add	r3, r3, #94
	str	r3, [sp, #28]
.L2882:
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	cmp	r3, r7
	bls	.L2982
	add	r3, r4, r7, asl #1
	movw	r2, #65535
	ldrh	r3, [r3, #16]
	cmp	r3, r2
	beq	.L2883
	ldr	r2, [sp, #28]
	ldr	r1, [r5, #4]
	ldrh	r2, [r2]
	mla	r3, r2, r3, r9
	ldr	r2, [r5, #12]
	str	r3, [sp, #20]
	mov	r0, r3
	ldrb	r3, [r8, #3312]	@ zero_extendqisi2
	bl	ftl_read_ppa_page
	cmp	r0, #512
	mov	r10, r0
	beq	.L2884
	cmn	r0, #1
	beq	.L2885
	ldr	r3, [r5, #12]
	ldr	r2, [r3]
	cmn	r2, #1
	bne	.L2885
	ldr	r3, [r3, #4]
	cmn	r3, #1
	bne	.L2885
	ldr	r3, [r5, #4]
	ldr	r3, [r3]
	cmn	r3, #1
	beq	.L2884
.L2885:
	ldr	r3, .L2985+28
	mov	r2, #1
	strb	r2, [r3, #-3175]
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	ldrh	r2, [r4, #10]
	mla	r3, r3, r9, r7
	cmp	r2, r3
	beq	.L2886
	ldr	r1, .L2985+32
	movw	r2, #1412
	ldr	r0, .L2985+36
	bl	printk
	bl	dump_stack
.L2886:
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #6]
	add	r2, r2, r3
	ldr	r3, .L2985+24
	ldrh	r1, [r3]
	ldrb	r3, [r4, #9]	@ zero_extendqisi2
	mul	r3, r3, r1
	cmp	r2, r3
	beq	.L2887
	ldr	r1, .L2985+32
	movw	r2, #1413
	ldr	r0, .L2985+36
	bl	printk
	bl	dump_stack
.L2887:
	cmn	r10, #1
	beq	.L2891
	ldr	r3, [r5, #12]
	ldr	r0, [r3, #4]
	cmn	r0, #1
	beq	.L2891
	bl	lpa_hash_get_ppa
	ldr	r2, [sp, #24]
	mov	r3, r0
	cmp	r2, #0
	beq	.L2889
	ldr	r2, [r5, #12]
	ldr	r1, [r2, #8]
	str	r2, [sp, #16]
	cmp	r0, r1
	cmnne	r0, #1
	beq	.L2889
	ldr	fp, .L2985+40
	mov	r10, #1
	str	r3, [sp, #12]
	ldrh	r0, [fp]
	rsb	r1, r0, #21
	mov	r1, r10, asl r1
	sub	r1, r1, #1
	and	r0, r1, r3, lsr r0
	ldr	r3, .L2985+44
	ldrb	r1, [r3, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r3, [sp, #24]
	uxth	r0, r0
	ldr	r2, [sp, #16]
	ldrh	r1, [r3]
	ldr	r3, [sp, #12]
	cmp	r1, r0
	bne	.L2889
	ldr	ip, [r2]
	mov	r0, r3
	ldr	r1, [r5, #4]
	ldrb	r3, [r8, #3312]	@ zero_extendqisi2
	ldr	r2, .L2985+48
	str	ip, [sp, #12]
	bl	ftl_read_ppa_page
	ldr	r3, .L2985+52
	ldr	r3, [r3, #-3512]
	ldr	ip, [sp, #12]
	cmp	r3, ip
	bcc	.L2889
	ldr	r3, [r5, #12]
	ldr	r0, [r3, #8]
	cmn	r0, #1
	beq	.L2891
	ldrh	r3, [fp]
	ldrb	r1, [r8, #3766]	@ zero_extendqisi2
	rsb	r2, r3, #21
	mov	r10, r10, asl r2
	sub	r10, r10, #1
	and	r0, r10, r0, lsr r3
	bl	__aeabi_uidiv
	uxth	r0, r0
	bl	ftl_vpn_decrement
	b	.L2891
.L2889:
	ldr	r2, [r5, #12]
	ldr	r3, [r8, #3320]
	ldr	r1, [r2, #4]
	cmp	r1, r3
	bcs	.L2891
	ldr	r3, .L2985+24
	ldrb	r0, [r4, #9]	@ zero_extendqisi2
	ldrh	r1, [r4, #10]
	ldrh	r3, [r3]
	mul	r3, r3, r0
	sub	r3, r3, #1
	cmp	r1, r3
	bge	.L2891
	ldr	r3, [r8, #3340]
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
	ldr	r3, [sp, #20]
	str	r3, [sp, #44]
.L2891:
	ldrh	r3, [r4, #6]
	sub	r3, r3, #1
	strh	r3, [r4, #6]	@ movhi
	ldrh	r3, [r4, #10]
	add	r3, r3, #1
	strh	r3, [r4, #10]	@ movhi
	mov	r3, #4
	str	r3, [sp, #12]
	mov	r3, #1
	str	r3, [sp, #16]
.L2883:
	add	r7, r7, #1
	uxth	r7, r7
	b	.L2882
.L2982:
	add	r9, r9, #1
	mov	r3, #0
	strb	r3, [r4, #5]
	uxth	r9, r9
	b	.L2881
.L2884:
	ldrh	r3, [r4, #6]
	ldrh	r8, [r4, #10]
	strb	r7, [r4, #5]
	add	r8, r3, r8
	ldr	r3, .L2985+24
	ldrb	ip, [r4, #9]	@ zero_extendqisi2
	strh	r9, [r4, #2]	@ movhi
	ldrh	r7, [r3]
	str	r3, [sp, #20]
	mul	ip, r7, ip
	cmp	r8, ip
	beq	.L2895
	ldr	r1, .L2985+32
	movw	r2, #1493
	ldr	r0, .L2985+36
	bl	printk
	bl	dump_stack
.L2895:
	ldr	r3, .L2985+52
	mov	r10, #0
	ldrh	r7, [r4, #10]
	mov	ip, r10
	ldr	r2, [r3, #-3860]
	str	r3, [sp, #24]
.L2896:
	cmp	ip, r7
	bcs	.L2983
	ldrh	r3, [r4, #12]
	add	r3, ip, r3
	add	ip, ip, #1
	ldr	r3, [r2, r3, asl #2]
	cmn	r3, #1
	addne	r10, r10, #1
	uxthne	r10, r10
	b	.L2896
.L2983:
	ldr	r3, [sp, #20]
	ldrb	r2, [r4, #9]	@ zero_extendqisi2
	ldrh	r3, [r3]
	smulbb	r3, r2, r3
	rsb	r7, r7, r3
	ldr	r3, [r6]
	add	r7, r10, r7
	ldr	r10, .L2985+44
	tst	r3, #4096
	uxth	r7, r7
	beq	.L2899
	ldrh	r1, [r4]
	mov	r2, r7
	ldr	ip, [r10, #576]
	ldr	r0, .L2985+56
	mov	r3, r1, asl #1
	ldrh	r3, [ip, r3]
	bl	printk
.L2899:
	ldrh	r3, [r4]
	ldr	r2, [r10, #576]
	mov	r3, r3, asl #1
	strh	r7, [r2, r3]	@ movhi
	ldr	r3, [r6]
	tst	r3, #16384
	beq	.L2900
	ldr	r3, [sp, #44]
	add	r1, sp, #32
	ldr	r0, .L2985+60
	str	r3, [sp]
	ldmia	r1, {r1, r2, r3}
	bl	printk
.L2900:
	ldrb	r2, [r10, #3312]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, [r5, #4]
	mov	r8, #0
	ldr	r9, .L2985+44
	mov	r2, r2, asl #9
	bl	ftl_memset
.L2901:
	ldrb	r3, [r9, #3720]	@ zero_extendqisi2
	ldr	r2, [sp, #12]
	mul	r3, r3, r2
	cmp	r8, r3
	bcs	.L2984
	add	ip, sp, #32
	ldr	r0, [ip, r8, asl #2]
	cmn	r0, #1
	bne	.L2902
.L2906:
	ldr	r2, [r10, #3340]
	mvn	r7, #0
	ldr	r3, [r5, #12]
	ldr	r2, [r2, #8]
	str	r2, [r3]
	ldr	r3, [r5, #12]
	str	r7, [r3, #4]
	ldr	r3, [r5, #12]
	str	r7, [r3, #8]
	mov	r3, #0
	ldr	r2, [r5, #12]
	str	r3, [r2, #12]
	ldr	r2, [r5, #4]
	str	r3, [r2]
	b	.L2903
.L2902:
	ldr	r1, [r5, #4]
	ldr	r2, [r5, #12]
	ldrb	r3, [r9, #3312]	@ zero_extendqisi2
	str	ip, [sp, #28]
	bl	ftl_read_ppa_page
	ldr	r3, [r5, #12]
	mov	r7, r0
	ldr	r0, [r3, #4]
	bl	lpa_hash_get_ppa
	ldr	r3, [r6]
	tst	r3, #16384
	mov	fp, r0
	ldr	ip, [sp, #28]
	beq	.L2904
	ldr	r3, [r5, #12]
	mov	r1, fp
	ldr	r0, .L2985+64
	ldr	r2, [ip, r8, asl #2]
	ldr	r3, [r3, #4]
	bl	printk
.L2904:
	add	r3, sp, #32
	ldr	r2, [r9, #3340]
	ldr	r1, [r5, #12]
	ldr	r3, [r3, r8, asl #2]
	cmp	fp, r3
	ldr	r3, [r2, #8]
	add	r0, r3, #1
	str	r0, [r2, #8]
	str	r3, [r1]
	mvnne	r7, #0
	ldr	r3, [r5, #12]
	cmn	r7, #1
	mov	r2, #1
	str	fp, [r3, #8]
	ldr	r3, [r5, #12]
	str	r2, [r3, #12]
	beq	.L2906
.L2903:
	ldr	r3, [sp, #16]
	cmp	r3, #1
	bne	.L2908
	ldrh	r3, [r4, #6]
	cmp	r3, #1
	bls	.L2908
.L2959:
	mov	r0, r4
	bl	ftl_get_new_free_page
	ldr	r3, [r6]
	tst	r3, #16384
	mov	fp, r0
	beq	.L2910
	ldr	r3, [r5, #12]
	mov	r1, fp
	ldr	r0, .L2985+68
	ldr	r2, [r3, #4]
	bl	printk
.L2910:
	ldrb	r3, [r9, #3720]	@ zero_extendqisi2
	ldr	r1, [sp, #12]
	ldrh	r2, [r4, #6]
	mul	r3, r3, r1
	add	r3, r3, #1
	rsb	r3, r8, r3
	cmp	r2, r3
	bls	.L2908
	ldrb	r3, [r9, #3312]	@ zero_extendqisi2
	mov	r0, fp
	ldr	r1, [r5, #4]
	ldr	r2, [r5, #12]
	bl	ftl_prog_ppa_page
	mov	fp, r0
	ldrh	r0, [r4]
	bl	ftl_vpn_decrement
	adds	r3, r7, #1
	movne	r3, #1
	cmn	fp, #1
	cmnne	r7, #1
	beq	.L2911
	ldr	r3, [r6]
	tst	r3, #16384
	beq	.L2912
	ldrh	r1, [r4, #12]
	ldr	r2, [r5, #12]
	ldrh	r3, [r4, #10]
	ldr	r0, .L2985+72
	add	r3, r3, r1
	ldmib	r2, {r1, r2}
	sub	r3, r3, #1
	bl	printk
.L2912:
	ldrh	r3, [r4, #10]
	ldrh	r2, [r4, #12]
	ldr	r1, [r5, #12]
	add	r2, r3, r2
	sub	r2, r2, #1
	ldmib	r1, {r0, r1}
	uxth	r2, r2
	bl	lpa_hash_update_ppa
	b	.L2908
.L2911:
	ldrh	r2, [r4, #6]
	cmp	r2, #1
	movls	r3, #0
	andhi	r3, r3, #1
	cmp	r3, #0
	bne	.L2959
.L2908:
	add	r8, r8, #1
	b	.L2901
.L2984:
	mov	r0, r5
	bl	zbuf_free
	ldrh	r3, [r4, #12]
	ldrb	r1, [r4, #9]	@ zero_extendqisi2
	ldr	r2, [sp, #20]
	ldrh	r2, [r2]
	mla	r3, r1, r2, r3
	ldr	r2, [sp, #24]
	ldr	r2, [r2, #-3860]
	sub	r3, r3, #-1073741823
	ldr	r3, [r2, r3, asl #2]
	cmn	r3, #1
	beq	.L2915
	ldr	r1, .L2985+32
	movw	r2, #1567
	ldr	r0, .L2985+36
	bl	printk
	bl	dump_stack
.L2915:
	ldrh	r3, [r4, #6]
	cmp	r3, #1
	bne	.L2875
	mov	r0, r4
	bl	ftl_write_last_log_page
.L2875:
	add	sp, sp, #100
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L2986:
	.align	2
.L2985:
	.word	.LANCHOR2
	.word	.LC186
	.word	.LC187
	.word	.LC188
	.word	.LC189
	.word	.LANCHOR0+564
	.word	.LANCHOR0+3712
	.word	.LANCHOR4
	.word	.LANCHOR1+1836
	.word	.LC0
	.word	.LANCHOR0+3764
	.word	.LANCHOR0
	.word	.LANCHOR3-3512
	.word	.LANCHOR3
	.word	.LC190
	.word	.LC191
	.word	.LC192
	.word	.LC193
	.word	.LC194
	.fnend
	.size	ftl_open_sblk_recovery, .-ftl_open_sblk_recovery
	.align	2
	.global	pm_write_page
	.type	pm_write_page, %function
pm_write_page:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	mov	r4, r0
	ldr	r6, .L3001
	mov	r7, r1
	mov	r5, r6
.L2988:
	ldr	r3, [r6, #3340]
	ldr	r2, [r3, #48]
	add	r2, r2, #1
	str	r2, [r3, #48]
	add	r2, r3, #696
	ldrh	r1, [r2]
	ldr	r2, .L3001+4
	ldrh	r2, [r2]
	cmp	r1, r2
	bcs	.L2989
	add	r3, r3, #692
	ldrh	r2, [r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2990
.L2989:
	bl	pm_alloc_new_blk
	mov	r0, #0
	bl	ftl_info_flush
.L2990:
	ldr	r3, [r5, #3340]
	add	r3, r3, #692
	ldrh	r2, [r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L2991
	ldr	r1, .L3001+8
	mov	r2, #230
	ldr	r0, .L3001+12
	bl	printk
	bl	dump_stack
.L2991:
	ldr	r3, [r5, #3340]
	ldr	r9, .L3001+16
	add	r2, r3, #692
	add	r3, r3, #696
	ldrh	r1, [r2]
	ldr	r2, .L3001+20
	ldr	r0, [r9, #-2840]
	ldrh	r8, [r2]
	ldrh	r2, [r3]
	mla	r8, r8, r1, r2
	mov	r1, #64
	bl	__memzero
	ldr	r3, [r9, #-2840]
	str	r4, [r3]
	ldr	r2, [r5, #3340]
	ldr	r3, [r9, #-2840]
	ldr	r1, [r2, #48]
	str	r1, [r3, #4]
	mov	r1, r8
	ldrb	r0, [r2, #694]	@ zero_extendqisi2
	ldrb	r2, [r5, #3312]	@ zero_extendqisi2
	str	r2, [sp]
	mov	r2, r7
	bl	ftl_prog_page
	ldr	r2, [r5, #3340]
	add	r1, r2, #696
	ldrh	r3, [r1]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r1]	@ movhi
	cmp	r3, #1
	beq	.L2992
	ldrb	r3, [r9, #-2836]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L2993
.L2992:
	mov	r3, #0
	strb	r3, [r9, #-2836]
	b	.L2988
.L2993:
	cmn	r0, #1
	bne	.L2995
	mov	r1, r8
	ldr	r0, .L3001+24
	bl	printk
	b	.L2988
.L2995:
	movw	r3, #698
	mov	r0, #0
	ldrh	r3, [r2, r3]
	cmp	r4, r3
	addcc	r4, r4, #176
	strcc	r8, [r2, r4, asl #2]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3002:
	.align	2
.L3001:
	.word	.LANCHOR0
	.word	.LANCHOR0+3712
	.word	.LANCHOR1+1860
	.word	.LC0
	.word	.LANCHOR5
	.word	.LANCHOR0+3806
	.word	.LC195
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
	ldr	r7, .L3036
	.pad #100
	sub	sp, sp, #100
	add	r3, r7, #564
	ldrh	r3, [r3]
	cmp	r3, r0
	bls	.L3003
	ldrb	r3, [r7, #3771]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3005
	ldrb	r3, [r7, #3769]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3003
.L3005:
	ldr	r3, [r7, #568]
	mov	r4, fp, asl #2
	add	r1, sp, #80
	mov	r0, fp
	add	r3, r3, r4
	strh	fp, [sp, #64]	@ movhi
	ldr	r10, .L3036+4
	mov	r5, #0
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	mov	r6, r5
	mov	r8, r5
	and	r3, r3, #224
	cmp	r3, #160
	ldreq	r3, .L3036
	movne	r9, #1
	ldreqb	r9, [r3, #3768]	@ zero_extendqisi2
	bl	ftl_get_blk_list_in_sblk
	ldrh	r3, [r10]
	ldr	r2, [r7, #568]
	uxtb	r0, r0
	strb	r0, [sp, #73]
	strh	r5, [sp, #66]	@ movhi
	smulbb	r0, r3, r0
	strb	r5, [sp, #69]
	strh	r5, [sp, #74]	@ movhi
	add	r3, r2, r4
	strh	r0, [sp, #70]	@ movhi
	ldrh	r1, [r2, r4]
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ubfx	r1, r1, #0, #11
	str	r1, [sp]
	ldr	r2, [r2, fp, asl #2]
	mov	r1, fp
	ldr	r0, .L3036+8
	ubfx	r2, r2, #11, #8
	str	r2, [sp, #4]
	mov	r2, r3, lsr #5
	ubfx	r3, r3, #3, #2
	bl	printk
	mov	r0, #1
	bl	buf_alloc
	mov	ip, r10
	mov	r4, r0
.L3007:
	ldrh	r3, [ip]
	uxth	r7, r5
	cmp	r3, r7
	bls	.L3028
	mov	r3, r7, asl #1
	mov	r10, #0
	sub	r3, r3, #1
	str	r3, [sp, #44]
.L3017:
	ldrb	r3, [sp, #73]	@ zero_extendqisi2
	uxth	r8, r10
	cmp	r3, r8
	bls	.L3033
	add	r3, r8, #8
	mov	r6, #1
	mov	r3, r3, asl r6
	str	r3, [sp, #48]
.L3016:
	cmp	r6, r9
	bhi	.L3034
	ldr	r2, [sp, #48]
	add	r3, sp, #96
	add	r3, r3, r2
	ldrh	r2, [r3, #-32]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L3009
	ldr	r3, .L3036+12
	cmp	r9, #3
	ldrh	r3, [r3]
	mul	r3, r3, r2
	uxtaheq	r3, r3, r5
	orreq	r3, r3, r6, asl #24
	beq	.L3011
	cmp	r9, #2
	uxtahne	r3, r3, r5
	ldreq	r1, [sp, #44]
	addeq	r3, r3, r1
	ldreq	r1, .L3036
	addeq	r3, r3, r6
	ldreqb	r1, [r1, #3768]	@ zero_extendqisi2
	orreq	r3, r3, r1, asl #24
.L3011:
	str	r3, [r4, #24]
	mov	r1, #1
	mov	r0, r4
	str	ip, [sp, #60]
	str	r2, [sp, #56]
	str	r3, [sp, #52]
	bl	sblk_read_page
	ldr	r0, [r4, #4]
	ldr	r3, [r4, #36]
	ldr	r1, [r4, #12]
	str	r3, [sp]
	ldr	lr, [r0]
	ldr	r2, [sp, #56]
	str	r3, [sp, #40]
	str	lr, [sp, #4]
	ldr	lr, [r0, #4]
	ldr	r3, [sp, #52]
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
	ldr	r0, .L3036+16
	str	r1, [sp, #32]
	mov	r1, r2
	mov	r2, r7
	bl	printk
	ldr	r3, [sp, #40]
	ldr	ip, [sp, #60]
	cmp	r3, #512
	beq	.L3028
.L3009:
	add	r6, r6, #1
	uxth	r6, r6
	b	.L3016
.L3034:
	add	r10, r10, #1
	b	.L3017
.L3033:
	add	r5, r5, #1
	b	.L3007
.L3028:
.L3013:
	str	r6, [sp]
	mov	r3, r8
	ldr	r6, .L3036
	mov	r1, fp
	mov	r2, r7
	ldr	r0, .L3036+20
	bl	printk
	mov	r1, #0
	ldrb	r2, [r6, #3312]	@ zero_extendqisi2
	mov	r5, r7
	ldr	r0, [r4, #4]
	mov	r9, r6
	mov	r2, r2, asl #9
	bl	ftl_memset
	ldrb	r2, [r6, #3312]	@ zero_extendqisi2
	ldr	r0, [r4, #12]
	mov	r1, #0
	mov	r2, r2, asl #1
	bl	ftl_memset
.L3018:
	ldr	r3, .L3036+4
	ldrh	r3, [r3]
	cmp	r3, r5
	bls	.L3023
	ldr	r7, .L3036+12
	mov	r10, r5, asl #1
	mov	r8, #0
.L3024:
	ldrb	r2, [sp, #73]	@ zero_extendqisi2
	uxth	r3, r8
	cmp	r2, r3
	bls	.L3035
	mov	r2, r3
	ldr	r0, .L3036+24
	mov	r1, r5
	str	r3, [sp, #40]
	bl	printk
	ldrb	r2, [r6, #3771]	@ zero_extendqisi2
	cmp	r2, #0
	ldr	r3, [sp, #40]
	add	r3, r3, #8
	beq	.L3019
	add	r2, sp, #96
	mov	r0, r4
	add	r3, r2, r3, asl #1
	ldrh	r2, [r3, #-32]
	ldrh	r3, [r7]
	mul	r3, r3, r2
	orr	r3, r3, r5
	str	r3, [r4, #24]
	bl	sblk_3d_tlc_dump_prog
	b	.L3020
.L3019:
	ldrb	r1, [r9, #3768]	@ zero_extendqisi2
	cmp	r1, #2
	bne	.L3021
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
	b	.L3020
.L3021:
	add	r1, sp, #96
	add	r3, r1, r3, asl #1
	ldrh	r1, [r3, #-32]
	ldrh	r3, [r7]
	mul	r1, r3, r1
	ldrb	r3, [r9, #3312]	@ zero_extendqisi2
	orr	r1, r1, r5
	str	r1, [r4, #24]
	ubfx	r0, r1, #21, #3
	str	r3, [sp]
	str	r2, [sp, #4]
	bic	r1, r1, #14680064
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #12]
	bl	flash_prog_page_en
.L3020:
	add	r8, r8, #1
	b	.L3024
.L3035:
	add	r5, r5, #1
	uxth	r5, r5
	b	.L3018
.L3023:
	mov	r0, r4
	bl	zbuf_free
	ldr	r0, .L3036+28
	mov	r1, fp
	bl	printk
.L3003:
	add	sp, sp, #100
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3037:
	.align	2
.L3036:
	.word	.LANCHOR0
	.word	.LANCHOR0+3712
	.word	.LC196
	.word	.LANCHOR0+3806
	.word	.LC197
	.word	.LC198
	.word	.LC199
	.word	.LC200
	.fnend
	.size	ftl_dump_write_open_sblk, .-ftl_dump_write_open_sblk
	.align	2
	.global	flash_info_flush
	.type	flash_info_flush, %function
flash_info_flush:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3052
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3039
	ldr	r0, .L3052+4
	mov	r1, #324
	ldr	r2, .L3052+8
	bl	printk
.L3039:
	ldr	r6, .L3052+12
	mov	r1, #64
	ldr	r5, .L3052+16
	mov	r9, #0
	ldr	r8, .L3052+20
	ldr	r0, [r6, #-2832]
	bl	__memzero
	ldr	r0, .L3052+24
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
.L3040:
	ldrb	r3, [r6, #-2828]	@ zero_extendqisi2
	ldrh	r4, [r8, #2]
	ldrh	ip, [r5, #2]
	mov	r1, r3
	ldr	r0, .L3052+28
	mov	r2, r4
	str	r3, [sp, #8]
	str	ip, [sp, #12]
	bl	printk
	ldr	r2, .L3052+32
	ldrh	r0, [r8, #2]
	ldr	r7, .L3052+12
	ldrh	r2, [r2]
	ldr	fp, .L3052+16
	sub	r2, r2, #1
	cmp	r0, r2
	ldr	r3, [sp, #8]
	ldr	ip, [sp, #12]
	blt	.L3041
	ldr	r4, [fp, #1696]
	ldrb	r2, [r7, #-2827]	@ zero_extendqisi2
	ldr	r3, [r4, #4]
	mov	r0, r4
	add	r3, r3, #1
	str	r3, [r4, #4]
	ldrb	r3, [r7, #-2828]	@ zero_extendqisi2
	strb	r2, [r7, #-2828]
	ldrh	r2, [r4, #16]
	strb	r3, [r7, #-2827]
	mov	r3, #0
	add	r2, r2, #1
	strh	r2, [r0, #16]!	@ movhi
	ldr	r1, [r4, #8]
	strh	r3, [r8, #2]	@ movhi
	str	r3, [sp, #8]
	bl	js_hash
	ldrb	r2, [r7, #-2828]	@ zero_extendqisi2
	str	r0, [r4, #12]
	ldr	r3, [sp, #8]
	ldrh	r4, [fp, #2]
	mov	r0, r3
	mul	r4, r4, r2
	b	.L3050
.L3041:
	cmp	r0, #0
	mla	r4, ip, r3, r4
	bne	.L3042
.L3050:
	mov	r1, r4
	bl	flash_erase_block
.L3042:
	ldr	r2, [r5, #1696]
	mov	r7, #1
	ldr	r3, [r6, #-2832]
	mov	r0, #0
	mov	r1, r4
	ldr	r2, [r2, #4]
	str	r2, [r3]
	ldr	r2, .L3052+36
	ldr	r3, [r6, #-2832]
	str	r2, [r3, #4]
	mov	r2, #4
	stmia	sp, {r2, r7}
	ldr	r2, [r5, #1696]
	bl	flash_prog_page_en
	ldrh	r3, [r10, #2]
	add	r3, r3, r7
	strh	r3, [r10, #2]	@ movhi
	cmn	r0, #1
	bne	.L3043
	mov	r1, r4
	ldr	r0, .L3052+40
	bl	printk
	b	.L3040
.L3043:
	cmp	r9, #0
	moveq	r9, r7
	beq	.L3040
.L3051:
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3053:
	.align	2
.L3052:
	.word	.LANCHOR2
	.word	.LC134
	.word	.LANCHOR1+1876
	.word	.LANCHOR5
	.word	.LANCHOR0
	.word	.LANCHOR5-2828
	.word	.LC201
	.word	.LC202
	.word	.LANCHOR0+1704
	.word	1398362953
	.word	.LC203
	.fnend
	.size	flash_info_flush, .-flash_info_flush
	.align	2
	.global	flash_info_blk_init
	.type	flash_info_blk_init, %function
flash_info_blk_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3086
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r3, [r3]
	ldr	r6, .L3086+4
	tst	r3, #4096
	beq	.L3055
	ldr	r3, [r6, #1696]
	mov	r1, #50
	ldr	r0, .L3086+8
	ldr	r2, .L3086+12
	str	r3, [sp]
	mov	r3, #2048
	bl	printk
.L3055:
	ldr	fp, .L3086+4
	mov	r5, #4
	ldr	r8, .L3086+16
.L3058:
	ldrh	r1, [r6, #2]
	mov	r9, #4
	ldr	r4, .L3086+20
	mov	r0, #0
	str	r9, [sp]
	mul	r1, r1, r5
	ldr	r2, [r6, #1696]
	ldr	r3, [r4, #-2832]
	bl	flash_read_page_en
	ldr	r7, .L3086+4
	cmn	r0, #1
	beq	.L3056
	ldr	r2, [fp, #1696]
	ldr	r10, .L3086+16
	ldr	r3, [r2]
	cmp	r3, r8
	beq	.L3057
.L3056:
	add	r5, r5, #1
	cmp	r5, #16
	bne	.L3058
.L3084:
	mvn	r0, #0
	b	.L3082
.L3057:
	ldrb	r1, [r2, #37]	@ zero_extendqisi2
	mov	r0, #0
	ldrb	r3, [r2, #36]	@ zero_extendqisi2
	strb	r1, [r4, #-2827]
	ldrh	r1, [r7, #2]
	strb	r3, [r4, #-2828]
	str	r9, [sp]
	mul	r1, r1, r3
	ldr	r3, [r4, #-2832]
	bl	flash_read_page_en
	cmn	r0, #1
	beq	.L3067
	ldr	r3, [r7, #1696]
	ldr	r2, [r3]
	cmp	r2, r8
	ldreq	r5, [r3, #4]
	beq	.L3060
.L3067:
	mov	r5, #0
.L3060:
	ldrb	r3, [r4, #-2827]	@ zero_extendqisi2
	mov	r2, #4
	ldrh	r1, [r6, #2]
	mov	r0, #0
	str	r2, [sp]
	ldr	r2, [r6, #1696]
	mul	r1, r1, r3
	ldr	r3, [r4, #-2832]
	bl	flash_read_page_en
	ldr	r7, .L3086+20
	cmn	r0, #1
	beq	.L3061
	ldr	r3, .L3086+4
	ldr	r3, [r3, #1696]
	ldr	r2, [r3]
	cmp	r2, r10
	bne	.L3061
	ldr	r2, [r3, #4]
	cmp	r5, r2
	ldrccb	r2, [r3, #37]	@ zero_extendqisi2
	ldrccb	r3, [r3, #36]	@ zero_extendqisi2
	strccb	r2, [r7, #-2828]
	strccb	r3, [r7, #-2827]
.L3061:
	ldr	r2, [r6, #1696]
	mov	r8, #4
	ldr	r3, [r4, #-2832]
	mov	r0, #0
	ldrb	r1, [r4, #-2828]	@ zero_extendqisi2
	mov	r7, #0
	str	r8, [sp]
	bl	flash_get_last_written_page
	ldr	r3, .L3086+24
	ldrh	r9, [r6, #2]
	ldr	ip, .L3086+4
	uxth	fp, r0
	add	r2, fp, #1
	strh	r2, [r3, #2]	@ movhi
	ldrb	r3, [r4, #-2828]	@ zero_extendqisi2
	mla	r9, r9, r3, r0
.L3062:
	rsb	r5, r7, fp
	sxth	r5, r5
	cmp	r5, #0
	blt	.L3085
	str	r8, [sp]
	mov	r0, #0
	rsb	r1, r7, r9
	ldr	r2, [r6, #1696]
	ldr	r3, [r4, #-2832]
	str	ip, [sp, #12]
	bl	flash_read_page_en
	cmn	r0, #1
	ldr	ip, [sp, #12]
	beq	.L3063
	ldr	r3, [ip, #1696]
	ldr	r3, [r3]
	cmp	r3, r10
	beq	.L3064
.L3063:
	add	r7, r7, #1
	b	.L3062
.L3085:
	cmn	r5, #1
	bne	.L3064
	ldr	r3, [r6, #1696]
	ldr	r0, .L3086+28
	ldr	r1, [r3]
	bl	printk
	b	.L3084
.L3064:
	cmp	r7, #1
	bls	.L3068
	bl	flash_info_flush
.L3068:
	mov	r0, #0
.L3082:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3087:
	.align	2
.L3086:
	.word	.LANCHOR2
	.word	.LANCHOR0
	.word	.LC204
	.word	.LANCHOR1+1896
	.word	1398362953
	.word	.LANCHOR5
	.word	.LANCHOR5-2828
	.word	.LC205
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
	ldr	r6, .L3167
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3089
	ldr	r0, .L3167+4
	movw	r1, #2491
	ldr	r2, .L3167+8
	bl	printk
.L3089:
	ldr	r5, .L3167+12
	mov	r0, r4
	mov	r7, #0
	ldr	r4, .L3167+16
	ldr	r8, .L3167+20
	str	r7, [r5, #-3536]
	bl	nandc_init
	ldr	r3, .L3167+24
	add	r0, r4, #588
	mov	r1, #8
	mov	r10, r4
	mov	fp, r8
	str	r3, [r4, #1620]
	mov	r3, #1
	strb	r3, [r4, #585]
	bl	__memzero
	ldr	r0, .L3167+28
	mov	r1, #32
	bl	__memzero
.L3095:
	mov	r9, r7, asl #3
	uxtb	r0, r7
	add	r1, r8, r9
	bl	flash_read_id
	cmp	r7, #0
	ldr	r1, .L3167+20
	bne	.L3090
	ldrb	r3, [r10, #1740]	@ zero_extendqisi2
	sub	r3, r3, #1
	uxtb	r3, r3
	cmp	r3, #253
	bls	.L3091
.L3093:
	mvn	r8, #1
	b	.L3159
.L3091:
	ldrb	r3, [r10, #1741]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L3093
.L3090:
	ldrb	r3, [r9, fp]	@ zero_extendqisi2
	add	r7, r7, #1
	cmp	r3, #181
	moveq	r3, #44
	streqb	r3, [r9, r8]
	cmp	r7, #4
	bne	.L3095
	ldr	ip, .L3167+32
	mov	r3, #0
.L3127:
	mov	r0, r3, asl #5
	ldrb	r8, [ip, r3, asl #5]	@ zero_extendqisi2
	add	r7, r0, #1
	ldr	lr, .L3167+32
	add	r7, ip, r7
	mov	r2, #0
.L3096:
	cmp	r2, r8
	bcs	.L3166
	ldrb	r10, [r7, r2]	@ zero_extendqisi2
	ldrb	r9, [r2, r1]	@ zero_extendqisi2
	cmp	r10, r9
	bne	.L3097
	add	r2, r2, #1
	b	.L3096
.L3166:
	adds	r8, lr, r0
	beq	.L3093
	ldr	r3, .L3167
	ldr	r2, .L3167+36
	add	r0, r3, r0
	mov	r3, #0
	ldrb	r1, [r0, #446]	@ zero_extendqisi2
.L3101:
	ldrb	r0, [r2, r3, asl #5]	@ zero_extendqisi2
	cmp	r0, r1
	beq	.L3100
	add	r3, r3, #1
	cmp	r3, #4
	bne	.L3101
.L3100:
	ldr	r1, .L3167+36
	mov	r2, #32
	ldr	r7, .L3167
	add	r1, r1, r3, asl #5
	ldr	r0, .L3167+40
	bl	ftl_memcpy
	add	r0, r7, #4
	mov	r1, r8
	mov	r2, #32
	bl	ftl_memcpy
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #8
	bhi	.L3102
	ldrb	r2, [r7, #24]	@ zero_extendqisi2
	cmp	r2, #60
	movhi	r2, #60
	strhib	r2, [r7, #24]
	cmp	r3, #8
	bne	.L3102
	ldrb	r3, [r4, #1740]	@ zero_extendqisi2
	cmp	r3, #44
	cmpne	r3, #137
	bne	.L3102
	ldrb	r3, [r6, #32]	@ zero_extendqisi2
	cmp	r3, #3
	ldreq	r3, .L3167
	moveq	r2, #0
	streqb	r2, [r3, #32]
.L3102:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3104
	ldr	r0, .L3167+4
	movw	r1, #2516
	ldr	r2, .L3167+8
	bl	printk
.L3104:
	mov	r0, #16384
	ldr	r7, .L3167+44
	bl	ftl_malloc
	mov	r8, #0
	str	r0, [r5, #-3528]
	mov	r0, #16384
	bl	ftl_malloc
	str	r0, [r5, #-3552]
	mov	r0, #2048
	bl	ftl_malloc
	str	r0, [r4, #1696]
	mov	r0, #64
	bl	ftl_malloc
	str	r0, [r5, #-3524]
	mov	r0, #64
	bl	ftl_malloc
	str	r0, [r5, #-3544]
	mov	r0, #64
	bl	ftl_malloc
	strb	r8, [r7, #-2824]
	str	r0, [r7, #-2832]
	bl	flash_die_info_init
	ldrb	r0, [r6, #22]	@ zero_extendqisi2
	bl	flash_lsb_page_tbl_build
	ldrb	r0, [r6, #24]	@ zero_extendqisi2
	bl	nandc_bch_sel
	ldr	r2, [r4, #1620]
	str	r8, [r5, #-3532]
	ldrh	r3, [r2, #16]
	ubfx	r1, r3, #8, #3
	strb	r1, [r4, #1774]
	ubfx	r1, r3, #3, #1
	strb	r1, [r5, #-3520]
	ubfx	r1, r3, #4, #1
	strb	r1, [r4, #1775]
	ubfx	r1, r3, #12, #1
	strb	r1, [r4, #3771]
	ubfx	r1, r3, #13, #1
	strb	r1, [r4, #3769]
	ubfx	r1, r3, #14, #1
	mov	r3, r3, lsr #15
	strb	r3, [r4, #3770]
	ldrb	r3, [r2, #28]	@ zero_extendqisi2
	mov	r2, #60
	strb	r2, [r4, #1624]
	ldrb	r2, [r4, #516]	@ zero_extendqisi2
	strb	r3, [r4, #1625]
	cmp	r2, #9
	strb	r3, [r4]
	ldrb	r3, [r6, #11]	@ zero_extendqisi2
	ldreq	r2, .L3167+16
	strb	r1, [r4, #1]
	moveq	r1, #70
	streqb	r1, [r2, #1624]
	cmp	r3, #9
	ldreq	r3, .L3167+16
	moveq	r2, #0
	streqb	r2, [r3, #1625]
	ldrh	r2, [r6, #20]
	ldrb	r3, [r6, #23]	@ zero_extendqisi2
	tst	r2, #64
	strb	r3, [r4, #1692]
	beq	.L3108
	sub	r2, r3, #17
	sub	r1, r3, #21
	clz	r1, r1
	cmp	r3, #21
	cmpne	r2, #2
	mov	r1, r1, lsr #5
	bhi	.L3109
	ldr	r2, .L3167+48
	cmp	r1, #0
	ldr	r3, .L3167+12
	str	r2, [r5, #-3532]
	moveq	r2, #15
	movne	r2, #4
	strb	r2, [r3, #-3540]
	b	.L3108
.L3109:
	sub	r2, r3, #65
	cmp	r3, #33
	cmpne	r2, #1
	bhi	.L3112
	ldr	r3, .L3167+52
	mov	r2, #4
	str	r3, [r5, #-3532]
	ldr	r3, .L3167+16
	strb	r2, [r3, #1693]
	b	.L3165
.L3112:
	sub	r2, r3, #67
	sub	r1, r3, #34
	cmp	r2, #1
	movhi	r0, #0
	movls	r0, #1
	cmp	r2, #1
	cmphi	r1, #1
	bhi	.L3113
	cmp	r3, #68
	cmpne	r3, #35
	ldr	r2, .L3167+12
	ldr	r1, .L3167+52
	movne	r3, #7
	moveq	r3, #17
	cmp	r0, #0
	strb	r3, [r2, #-3540]
	str	r1, [r5, #-3532]
	movne	r3, #4
	moveq	r3, #5
	strb	r3, [r4, #1693]
	b	.L3108
.L3113:
	sub	r3, r3, #36
	cmp	r3, #1
	bhi	.L3108
	ldr	r3, .L3167+56
	str	r3, [r5, #-3532]
.L3165:
	mov	r3, #7
	strb	r3, [r5, #-3540]
.L3108:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3117
	ldr	r0, .L3167+4
	movw	r1, #2574
	ldr	r2, .L3167+8
	bl	printk
.L3117:
	mov	r3, #0
	strb	r3, [r4, #1658]
	ldrb	r3, [r4, #1774]	@ zero_extendqisi2
	mov	r0, #4
	tst	r3, #1
	beq	.L3118
	bl	nandc_set_if_mode
	mov	r0, #1
	bl	flash_set_interface_mode
	mov	r0, #1
.L3118:
	bl	nandc_set_if_mode
	bl	flash_info_blk_init
	cmn	r0, #1
	mov	r8, r0
	bne	.L3120
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3121
	ldr	r0, .L3167+4
	movw	r1, #2606
	ldr	r2, .L3167+8
	bl	printk
.L3121:
	ldr	r3, [r4, #1696]
	mov	r2, #17
	mov	r0, #0
	strb	r2, [r3, #32]
	bl	zftl_flash_exit_slc_mode
	b	.L3159
.L3120:
	ldrb	r3, [r4, #1692]	@ zero_extendqisi2
	sub	r3, r3, #1
	cmp	r3, #7
	ldrls	r3, .L3167+60
	strls	r3, [r5, #-3532]
	ldrb	r3, [r4, #1774]	@ zero_extendqisi2
	ldr	r5, .L3167+16
	tst	r3, #4
	beq	.L3124
	ldr	r3, [r5, #1696]
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3124
	ldrb	r3, [r7, #-2828]	@ zero_extendqisi2
	mov	r0, #0
	ldrh	r1, [r5, #2]
	mul	r1, r1, r3
	bl	flash_ddr_para_scan
	ldrb	r3, [r5, #1658]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L3124
	ldr	r2, [r5, #1696]
	strb	r3, [r2, #19]
	bl	flash_info_flush
.L3124:
	ldr	r3, [r6]
	tst	r3, #4096
	beq	.L3126
	ldr	r0, .L3167+4
	mov	r1, #2672
	ldr	r2, .L3167+8
	bl	printk
.L3126:
	bl	nand_flash_print_info
	mov	r8, #0
	b	.L3159
.L3097:
	add	r3, r3, #1
	cmp	r3, #39
	bne	.L3127
	b	.L3093
.L3159:
	mov	r0, r8
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3168:
	.align	2
.L3167:
	.word	.LANCHOR2
	.word	.LC134
	.word	.LANCHOR1+1916
	.word	.LANCHOR3
	.word	.LANCHOR0
	.word	.LANCHOR0+1740
	.word	.LANCHOR2+4
	.word	.LANCHOR0+1708
	.word	.LANCHOR2+424
	.word	.LANCHOR2+1672
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
	ldr	ip, .L3181
	mov	r6, #0
	mov	r5, r6
	ldr	fp, [r0, #4]
	mov	r4, r0
.L3170:
	ldr	r3, .L3181+4
	uxth	r8, r6
	ldrh	r3, [r3]
	cmp	r3, r8
	bls	.L3180
	ldrh	r7, [ip]
	add	r6, r6, #1
	ldr	r3, .L3181+8
	ldr	r1, [r4, #4]
	mla	r7, r7, r10, r8
	ldr	r2, [r4, #12]
	ldrb	r3, [r3, #3312]	@ zero_extendqisi2
	str	ip, [sp, #44]
	mov	r0, r7
	bl	ftl_read_ppa_page
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #12]
	str	r0, [sp]
	mov	r9, r0
	ldr	r1, [r2]
	ldr	r0, .L3181+12
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
	b	.L3170
.L3180:
	ldr	r0, .L3181+16
	add	r1, fp, #704
	mov	r2, #4
	mov	r3, #32
	bl	rknand_print_hex
	mov	r0, r4
	bl	zbuf_free
	cmp	r5, #0
	beq	.L3173
	ldr	r1, .L3181+20
	movw	r2, #1351
	ldr	r0, .L3181+24
	bl	printk
	bl	dump_stack
.L3173:
	mov	r0, r5
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3182:
	.align	2
.L3181:
	.word	.LANCHOR0+3806
	.word	.LANCHOR0+3712
	.word	.LANCHOR0
	.word	.LC197
	.word	.LC206
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
	ldr	r4, .L3185
	ldr	r5, .L3185+4
	ldr	r0, .L3185+8
	add	r6, r4, #3712
	ldrb	r1, [r4, #3314]	@ zero_extendqisi2
	add	r7, r4, #564
	bl	printk
	sub	r3, r5, #3248
	sub	r3, r3, #8
	ldrb	r1, [r5, #-3256]	@ zero_extendqisi2
	ldrb	r2, [r5, #-3255]	@ zero_extendqisi2
	ldrh	r3, [r3, #2]
	ldr	r0, .L3185+12
	bl	printk
	ldr	r3, [r4, #3340]
	ldr	r0, .L3185+16
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
	ldr	r0, .L3185+20
	bl	printk
	ldr	r0, [r4, #580]
	ldrh	ip, [r0, #54]
	ldrh	r1, [r0, #48]
	ldrh	r2, [r0, #50]
	ldrb	r3, [r0, #53]	@ zero_extendqisi2
	str	ip, [sp]
	ldrh	r0, [r0, #58]
	str	r0, [sp, #4]
	ldr	r0, .L3185+24
	bl	printk
	ldr	r0, [r4, #580]
	ldrh	ip, [r0, #86]
	ldrh	r1, [r0, #80]
	ldrh	r2, [r0, #82]
	ldrb	r3, [r0, #85]	@ zero_extendqisi2
	str	ip, [sp]
	ldrh	r0, [r0, #90]
	str	r0, [sp, #4]
	ldr	r0, .L3185+28
	bl	printk
	ldrh	r2, [r6]
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	ldr	r1, [r5, #-3860]
	ldr	r0, .L3185+32
	mul	r3, r3, r2
	mov	r2, #4
	mov	r3, r3, asl #1
	bl	rknand_print_hex
	ldr	r1, [r4, #576]
	ldrh	r3, [r7]
	mov	r2, #2
	ldr	r0, .L3185+36
	bl	rknand_print_hex
	ldr	r3, [r4, #3340]
	movw	ip, #698
	ldr	r0, .L3185+40
	add	r1, r3, #704
	mov	r2, #4
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldr	r1, [r4, #568]
	ldrh	r3, [r7]
	mov	r2, #4
	ldr	r0, .L3185+44
	bl	rknand_print_hex
	add	r1, r4, #3808
	ldr	r0, .L3185+48
	add	r1, r1, #4
	mov	r2, #2
	mov	r3, #256
	bl	rknand_print_hex
	ldrh	r2, [r6]
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	ldr	r0, .L3185+52
	ldr	r1, [r5, #-3856]
	mul	r3, r3, r2
	mov	r2, #2
	mov	r3, r3, asl #1
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, lr}
	b	rknand_print_hex
.L3186:
	.align	2
.L3185:
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC207
	.word	.LC208
	.word	.LC209
	.word	.LC210
	.word	.LC211
	.word	.LC212
	.word	.LC213
	.word	.LC214
	.word	.LC206
	.word	.LC215
	.word	.LC216
	.word	.LC217
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
	movw	r3, #3764
	ldr	r7, .L3191
	mov	r4, r2
	mov	r6, r0
	mvn	r0, #0
	mov	r5, r1
	ldrh	r2, [r7, r3]
	ldrb	r1, [r7, #3766]	@ zero_extendqisi2
	rsb	r3, r2, #21
	mov	r2, r4, lsr r2
	bic	r0, r2, r0, asl r3
	bl	__aeabi_uidiv
	ldr	r2, [r7, #568]
	uxth	r0, r0
	add	r0, r2, r0, asl #2
	ldrb	r0, [r0, #2]	@ zero_extendqisi2
	mov	r0, r0, lsr #5
	cmp	r0, #1
	cmpne	r0, #7
	moveq	r0, #1
	movne	r0, #0
	ldmnefd	sp!, {r3, r4, r5, r6, r7, pc}
	mov	r1, r6
	mov	r2, r5
	mov	r3, r4
	ldr	r0, .L3191+4
	bl	printk
	bl	dump_ftl_info
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3192:
	.align	2
.L3191:
	.word	.LANCHOR0
	.word	.LC218
	.fnend
	.size	pm_ppa_update_check, .-pm_ppa_update_check
	.align	2
	.type	load_l2p_region, %function
load_l2p_region:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	cmp	r1, #31
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, lr}
	.save {r4, r5, r6, r7, r8, r9, lr}
	.pad #12
	mov	r4, r0
	mov	r5, r1
	bls	.L3194
	ldr	r1, .L3203
	mov	r2, #31
	ldr	r0, .L3203+4
	bl	printk
	bl	dump_stack
.L3194:
	ldr	r7, .L3203+8
	movw	r8, #698
	ldr	r3, [r7, #3340]
	ldrh	r2, [r3, r8]
	cmp	r4, r2
	bls	.L3195
	mov	r1, r4
	ldr	r0, .L3203+12
	bl	printk
	mov	r3, #0
	ldr	r0, [r3, #4]
	mov	r1, #255
	ldr	r3, .L3203+16
	ldrh	r2, [r3, #-6]
	bl	ftl_memset
	ldr	r3, [r7, #3340]
	ldrh	r3, [r3, r8]
	cmp	r3, r4
	bcs	.L3197
	ldr	r0, .L3203+4
	mov	r2, #35
	ldr	r1, .L3203
	b	.L3202
.L3195:
	add	r2, r4, #176
	ldr	r6, .L3203+20
	ldr	r9, [r3, r2, asl #2]
	mov	r3, r5, asl #3
	ldr	r2, .L3203+24
	add	r6, r6, r3
	add	r1, r2, r3
	strh	r4, [r2, r3]	@ movhi
	mov	r2, #0
	cmp	r9, r2
	strh	r2, [r1, #2]	@ movhi
	bne	.L3198
	mov	r1, r4
	mov	r2, r9
	ldr	r0, .L3203+28
	bl	printk
	ldr	r3, .L3203+16
	ldr	r0, [r6, #-3828]
	mov	r1, #255
	ldrh	r2, [r3, #-6]
	bl	ftl_memset
	b	.L3197
.L3198:
	ldr	r5, .L3203+32
	mov	r0, r9
	ldrb	r3, [r7, #3312]	@ zero_extendqisi2
	ldr	r1, [r6, #-3828]
	ldr	r2, [r5, #-2840]
	bl	ftl_read_ppa_page
	ldr	r2, [r5, #-2840]
	ldr	r2, [r2]
	cmp	r2, r4
	mov	r3, r0
	beq	.L3199
	mov	r1, r4
	str	r9, [sp]
	ldr	r0, .L3203+36
	bl	printk
	ldr	r3, [r7, #3340]
	ldr	r0, .L3203+40
	mov	r2, #4
	add	r1, r3, #704
	ldrh	r3, [r3, r8]
	bl	rknand_print_hex
	ldrb	r3, [r7, #3312]	@ zero_extendqisi2
	ldr	r0, .L3203+44
	mov	r2, #4
	ldr	r1, [r6, #-3828]
	mov	r3, r3, asl #7
	bl	rknand_print_hex
	ldr	r0, .L3203+48
	ldr	r1, [r5, #-2840]
	mov	r2, #4
	mov	r3, #16
	bl	rknand_print_hex
.L3199:
	ldr	r3, [r5, #-2840]
	ldr	r3, [r3]
	cmp	r3, r4
	beq	.L3197
	ldr	r0, .L3203+4
	mov	r2, #59
	ldr	r1, .L3203
.L3202:
	bl	printk
	bl	dump_stack
.L3197:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3204:
	.align	2
.L3203:
	.word	.LANCHOR1+1948
	.word	.LC0
	.word	.LANCHOR0
	.word	.LC219
	.word	.LANCHOR5-2816
	.word	.LANCHOR3
	.word	.LANCHOR3-3832
	.word	.LC220
	.word	.LANCHOR5
	.word	.LC221
	.word	.LC222
	.word	.LC223
	.word	.LC224
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
	ldr	r4, .L3219
	ldr	r5, .L3219+4
	ldr	r3, [r4, #3340]
	add	r3, r3, #688
	ldrh	r2, [r3]
	ldrh	r3, [r5, #-14]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L3207
	bl	pm_free_sblk
	ldr	r2, [r4, #3340]
	add	r3, r2, #688
	ldrh	r1, [r3]
	ldrh	r3, [r5, #-14]
	sub	r3, r3, #1
	cmp	r1, r3
	blt	.L3207
	uxth	r0, r0
	movw	r3, #65535
	add	r0, r0, #208
	mov	r0, r0, asl #1
	ldrh	r6, [r2, r0]
	cmp	r6, r3
	bne	.L3209
	ldr	r1, .L3219+8
	mov	r2, #131
	ldr	r0, .L3219+12
	bl	printk
	bl	dump_stack
	bl	pm_free_sblk
	ldr	r3, [r4, #3340]
	uxth	r0, r0
	add	r0, r0, #208
	mov	r0, r0, asl #1
	ldrh	r6, [r3, r0]
.L3209:
	bl	pm_select_ram_region
	ldr	r3, .L3219+16
	movw	r2, #65535
	ldr	r8, .L3219+20
	mov	r5, r0, asl #3
	mov	r7, r0
	add	r9, r3, r5
	str	r3, [sp, #4]
	ldrh	r0, [r3, r5]
	cmp	r0, r2
	beq	.L3210
	add	r3, r8, r5
	ldr	r1, [r3, #-3828]
	cmp	r1, #0
	beq	.L3210
	ldrsh	r3, [r9, #2]
	cmp	r3, #0
	bge	.L3210
	bl	pm_write_page
	ldrh	r3, [r9, #2]
	ubfx	r3, r3, #0, #15
	strh	r3, [r9, #2]	@ movhi
.L3210:
	mov	r10, #0
	mov	fp, #1
.L3211:
	ldr	r0, [r4, #3340]
	uxth	r9, r10
	add	r3, r0, #696
	add	r3, r3, #2
	ldrh	r3, [r3]
	cmp	r3, r9
	bls	.L3218
	ldr	r3, .L3219+24
	ldrh	r1, [r3]
	add	r3, r9, #176
	ldr	r0, [r0, r3, asl #2]
	rsb	r3, r1, #21
	mov	r3, fp, asl r3
	sub	r3, r3, #1
	and	r0, r3, r0, lsr r1
	ldr	r3, .L3219
	ldrb	r1, [r3, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	uxth	r0, r0
	cmp	r0, r6
	bne	.L3212
	mov	r1, r7
	mov	r0, r9
	bl	load_l2p_region
	add	r3, r8, r5
	mov	r0, r9
	ldr	r1, [r3, #-3828]
	bl	pm_write_page
	mvn	r3, #0
	ldr	r2, [sp, #4]
	strh	r3, [r2, r5]	@ movhi
.L3212:
	add	r10, r10, #1
	b	.L3211
.L3218:
	bl	pm_free_sblk
.L3207:
	mov	r0, #0
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3220:
	.align	2
.L3219:
	.word	.LANCHOR0
	.word	.LANCHOR3-3824
	.word	.LANCHOR1+1964
	.word	.LC0
	.word	.LANCHOR3-3832
	.word	.LANCHOR3
	.word	.LANCHOR0+3764
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
	ldr	r4, .L3227
	ldr	r3, .L3227+4
	add	r2, r4, r0
	add	r5, r3, r0
	ldr	r1, [r2, #-3828]
	ldrh	r0, [r3, r0]
	bl	pm_write_page
	ldrh	r3, [r5, #2]
	ubfx	r3, r3, #0, #15
	strh	r3, [r5, #2]	@ movhi
	ldr	r3, [r4, #-3836]
	cmp	r3, #0
	beq	.L3222
	bl	pm_gc
	mov	r3, #0
	str	r3, [r4, #-3836]
.L3222:
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L3228:
	.align	2
.L3227:
	.word	.LANCHOR3
	.word	.LANCHOR3-3832
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
	ldr	r5, .L3234
.L3231:
	add	r3, r5, r4, asl #3
	uxth	r0, r4
	ldrsh	r3, [r3, #2]
	cmp	r3, #0
	bge	.L3230
	bl	pm_flush_id
.L3230:
	add	r4, r4, #1
	cmp	r4, #32
	bne	.L3231
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L3235:
	.align	2
.L3234:
	.word	.LANCHOR3-3832
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
	mov	r7, r0
	ldr	r6, .L3256
	.pad #20
	sub	sp, sp, #20
	ldr	r5, .L3256+4
	mov	r0, #64
	mov	r4, #0
	mov	r3, #1
	strb	r3, [r6, #-2836]
	mov	r10, r4
	str	r4, [r5, #-3836]
	bl	ftl_malloc
	ldr	r8, .L3256+8
	mvn	r9, #0
	ldr	fp, .L3256+12
	str	r0, [r6, #-2840]
.L3242:
	mov	r3, #0
	cmp	r7, r3
	mov	r4, r8
	strh	r9, [r4, r10]!	@ movhi
	strh	r3, [r4, #2]	@ movhi
	beq	.L3241
	ldrb	r0, [fp, #3312]	@ zero_extendqisi2
	mov	r0, r0, asl #9
	bl	ftl_malloc
	str	r0, [r4, #4]
.L3241:
	add	r10, r10, #8
	cmp	r10, #256
	bne	.L3242
	ldr	r4, .L3256+12
	ldr	r9, [r6, #-2840]
	ldr	r10, [r5, #-3828]
	ldr	r3, [r4, #3340]
	add	r3, r3, #692
	mov	r2, r10
	ldrb	r0, [r3, #2]	@ zero_extendqisi2
	ldrh	r1, [r3]
	mov	r3, #4
	str	r3, [sp]
	mov	r3, r9
	bl	flash_get_last_written_page
	ldr	r3, [r4, #3340]
	add	r2, r3, #696
	ldrh	r2, [r2]
	cmp	r2, r0
	mov	r6, r0
	bgt	.L3243
	add	r3, r3, #692
	ldr	r0, .L3256+16
	ldrh	r1, [r3]
	mov	r3, r6
	bl	printk
	ldr	r3, [r4, #3340]
	add	r6, r6, #1
	mov	ip, r4
	add	r3, r3, #696
	ldrh	r8, [r3]
.L3244:
	sxth	r3, r8
	cmp	r3, r6
	bge	.L3255
	ldr	r2, [r4, #3340]
	ldr	r0, .L3256+20
	add	r1, r2, #692
	str	ip, [sp, #12]
	ldrh	r1, [r1]
	ldrh	r7, [r0]
	ldrb	r0, [r2, #694]	@ zero_extendqisi2
	mov	r2, r10
	mla	r7, r7, r1, r3
	ldrb	r3, [r4, #3312]	@ zero_extendqisi2
	str	r3, [sp]
	mov	r3, r9
	mov	r1, r7
	bl	flash_read_page_en
	ldr	r3, [r4, #3340]
	add	r3, r3, #692
	ldr	r2, [r3, #-644]
	add	r2, r2, #1
	str	r2, [r3, #-644]
	add	r3, r3, #2
	ldr	r1, [r9]
	mov	r2, r7
	ldrh	r3, [r3]
	mov	fp, r0
	ldr	r0, .L3256+24
	bl	printk
	cmp	fp, #512
	cmnne	fp, #1
	ldr	ip, [sp, #12]
	beq	.L3245
	ldr	r1, [ip, #3340]
	ldr	r3, [r9]
	add	r2, r1, #696
	add	r2, r2, #2
	ldrh	r2, [r2]
	cmp	r3, r2
	addcc	r3, r3, #176
	strcc	r7, [r1, r3, asl #2]
.L3245:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L3244
.L3255:
	ldr	r3, .L3256+28
	mov	r2, #1
	strb	r2, [r3, #-3175]
	ldr	r3, [r4, #3340]
	add	r3, r3, #696
	strh	r6, [r3]	@ movhi
	bl	pm_free_sblk
.L3243:
	ldr	r1, [r5, #-3828]
	mvn	r0, #0
	bl	pm_write_page
	ldr	r1, [r5, #-3828]
	mvn	r0, #0
	bl	pm_write_page
	ldr	r1, [r5, #-3828]
	mvn	r0, #0
	bl	pm_write_page
	ldr	r1, [r5, #-3828]
	mvn	r0, #0
	bl	pm_write_page
	bl	pm_free_sblk
	bl	pm_gc
	mov	r0, #0
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3257:
	.align	2
.L3256:
	.word	.LANCHOR5
	.word	.LANCHOR3
	.word	.LANCHOR3-3832
	.word	.LANCHOR0
	.word	.LC225
	.word	.LANCHOR0+3806
	.word	.LC226
	.word	.LANCHOR4
	.fnend
	.size	pm_init, .-pm_init
	.align	2
	.global	pm_log2phys
	.type	pm_log2phys, %function
pm_log2phys:
	.fnstart
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, lr}
	mov	r6, r1
	ldr	r9, .L3277
	mov	r4, r0
	mov	r8, r2
	ldrb	r5, [r9, #3312]	@ zero_extendqisi2
	mov	r1, r5, asl #7
	mov	r5, r5, asl #7
	bl	__aeabi_uidiv
	ldr	r2, [r9, #3320]
	cmp	r4, r2
	movcc	r3, #0
	ldrcc	r2, .L3277+4
	uxth	r10, r0
	mov	r7, r0
	smulbb	r5, r5, r10
	rsb	r5, r5, r4
	uxth	r5, r5
	bcc	.L3259
	mov	r1, r4
	ldr	r0, .L3277+8
	bl	printk
	mvn	r0, #0
	cmp	r8, #0
	streq	r0, [r6]
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L3275:
	ldrh	r1, [r1, r2]
	cmp	r1, r10
	beq	.L3262
.L3261:
	add	r3, r3, #1
	cmp	r3, #32
	beq	.L3274
.L3259:
	mov	r1, r3, asl #3
	uxth	r4, r3
	add	r0, r2, r1
	ldr	r9, .L3277+4
	ldr	r0, [r0, #4]
	cmp	r0, #0
	bne	.L3275
	b	.L3261
.L3274:
	bl	pm_select_ram_region
	mov	r3, r0, asl #3
	mov	r4, r0
	add	r2, r9, r3
	ldrh	r1, [r9, r3]
	movw	r3, #65535
	cmp	r1, r3
	bne	.L3276
.L3265:
	ldr	r3, .L3277+12
	mov	r0, r10
	mov	r1, r4
	strb	r4, [r3, #-2820]
	bl	load_l2p_region
.L3262:
	cmp	r8, #0
	ldr	r3, .L3277+16
	mov	r4, r4, asl #3
	addeq	r3, r3, r4
	ldreq	r3, [r3, #-3828]
	ldreq	r3, [r3, r5, asl #2]
	streq	r3, [r6]
	beq	.L3264
.L3263:
	add	r2, r3, r4
	ldr	r1, [r6]
	ldr	r2, [r2, #-3828]
	str	r1, [r2, r5, asl #2]
	add	r1, r9, r4
	strb	r7, [r3, #-3576]
	ldrh	r2, [r1, #2]
	mvn	r2, r2, asl #17
	mvn	r2, r2, lsr #17
	strh	r2, [r1, #2]	@ movhi
.L3264:
	add	r4, r9, r4
	movw	r2, #32767
	mov	r0, #0
	ldrh	r3, [r4, #2]
	ubfx	r1, r3, #0, #15
	cmp	r1, r2
	addne	r3, r3, #1
	strneh	r3, [r4, #2]	@ movhi
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, pc}
.L3276:
	ldrsh	r3, [r2, #2]
	cmp	r3, #0
	bge	.L3265
	bl	pm_flush_id
	b	.L3265
.L3278:
	.align	2
.L3277:
	.word	.LANCHOR0
	.word	.LANCHOR3-3832
	.word	.LC227
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
	ldr	r6, .L3332
	mov	r4, #0
	.pad #68
	sub	sp, sp, #68
	ldr	r5, [r6, #580]
	strb	r4, [r6, #3361]
	strb	r4, [r6, #3362]
	ldrh	r2, [r5, #80]
	cmp	r2, r3
	beq	.L3280
	mvn	r3, #0
	mov	r0, #1
	strh	r3, [r5, #130]	@ movhi
	bl	buf_alloc
	ldrb	r3, [r5, #89]	@ zero_extendqisi2
	add	r3, r5, r3, asl #1
	ldrh	r2, [r3, #94]
	movw	r3, #3806
	ldrh	r3, [r6, r3]
	mul	r3, r3, r2
	ldrb	r2, [r6, #3768]	@ zero_extendqisi2
	cmp	r2, #3
	mov	r4, r0
	str	r3, [sp, #56]
	bne	.L3281
	add	r2, r6, #3712
	ldrh	r2, [r2]
	sub	r2, r2, #1
	add	r3, r2, r3
	orr	r3, r3, #50331648
	b	.L3325
.L3281:
	cmp	r2, #2
	bne	.L3282
	add	r2, r6, #3712
	ldrh	r2, [r2]
	add	r3, r3, r2, asl #1
	sub	r3, r3, #1
	orr	r3, r3, #33554432
.L3325:
	str	r3, [r4, #24]
.L3282:
	mov	r0, r4
	mov	r1, #1
	bl	sblk_read_page
	ldr	r3, [r4, #36]
	cmn	r3, #1
	cmpne	r3, #512
	beq	.L3283
	ldr	r3, [r4, #12]
	ldr	r2, [r3]
	ldr	r3, .L3332+4
	cmp	r2, r3
	beq	.L3284
.L3283:
	mov	r0, r4
	bl	zbuf_free
	ldr	r3, [r4, #4]
	ldr	r2, [r4, #12]
	ldr	r0, .L3332+8
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
	ldr	r2, [r4, #36]
	ldr	r1, [r4, #24]
	mov	r4, #0
	ldr	r3, [r3]
	bl	printk
	ldrh	ip, [r5, #80]
	ldr	r3, [r6, #576]
	mov	ip, ip, asl #1
	strh	r4, [r3, ip]	@ movhi
	ldrh	r2, [r5, #80]
	ldr	r3, [r6, #580]
	b	.L3328
.L3284:
	movw	r7, #3722
	ldrb	r3, [r6, #3720]	@ zero_extendqisi2
	ldrh	r2, [r6, r7]
	ldr	r0, [r6, #3724]
	ldr	r1, [r4, #4]
	mul	r2, r3, r2
	ldr	r8, .L3332
	mov	r2, r2, asl #2
	bl	ftl_memcpy
	ldrb	r3, [r6, #3720]	@ zero_extendqisi2
	ldrh	r2, [r6, r7]
	ldr	r1, [r4, #4]
	ldr	r0, [r6, #3728]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	add	r1, r1, r2
	bl	ftl_memcpy
	ldr	r3, .L3332+12
	ldrh	r3, [r3]
	str	r3, [sp, #36]
	ldrb	r3, [r6, #3768]	@ zero_extendqisi2
	mov	r6, #0
	mov	fp, r6
	cmp	r3, #2
	str	r3, [sp, #40]
	ldreq	r3, [sp, #36]
	moveq	r3, r3, asl #1
	streq	r3, [sp, #36]
	moveq	r3, #1
	streq	r3, [sp, #40]
	b	.L3287
.L3330:
	ldr	r0, [r8, #3728]
	ldr	ip, [r0, r7]
	ldr	r0, [r2, #8]
	cmp	ip, r0
	bne	.L3291
.L3314:
	add	fp, fp, #1
	add	r9, r9, #1
	add	r7, r7, #4
.L3295:
	ldr	r3, [sp, #40]
	cmp	r9, r3
	bhi	.L3329
	ldr	r3, .L3332+16
	ldrh	r2, [r10]
	ldrh	r3, [r3]
	mul	r3, r3, r2
	ldrb	r2, [r8, #3768]	@ zero_extendqisi2
	cmp	r2, #3
	str	r3, [sp, #56]
	addeq	r3, r3, r6
	orreq	r3, r3, r9, asl #24
	beq	.L3326
	cmp	r2, #2
	subeq	r2, r6, #1
	addne	r3, r3, r6
	addeq	r3, r3, r2
	addeq	r3, r3, r9
	orreq	r3, r3, #33554432
.L3326:
	str	r3, [r4, #24]
	mov	r1, #1
	mov	r0, r4
	bl	sblk_read_page
	ldr	r3, .L3332
	ldr	r2, [r4, #12]
	ldr	r3, [r3, #3724]
	ldr	r1, [r2, #4]
	ldr	r3, [r3, r7]
	cmp	r3, r1
	beq	.L3330
.L3291:
	ldr	r0, [r8, #3728]
	ldr	ip, .L3332
	ldr	r0, [r0, r7]
	str	ip, [sp, #48]
	str	r0, [sp]
	ldr	r0, [r2]
	stmib	sp, {r0, r1}
	ldr	r1, [r2, #8]
	ldr	r0, .L3332+20
	str	r1, [sp, #12]
	ldr	r2, [r2, #12]
	str	r2, [sp, #16]
	ldr	r1, [r4, #24]
	ldr	r2, [r4, #36]
	bl	printk
	ldr	r3, [r8, #3724]
	ldr	r3, [r3, r7]
	cmn	r3, #1
	ldr	ip, [sp, #48]
	beq	.L3314
	mov	r0, r4
	str	ip, [sp, #36]
	bl	zbuf_free
	ldrh	r3, [r5, #80]
	mov	r4, #0
	mov	r3, r3, asl #1
	ldr	ip, [sp, #36]
	ldr	r2, [ip, #576]
	strh	r4, [r2, r3]	@ movhi
	ldrh	r2, [r5, #80]
	ldr	r3, [ip, #580]
.L3328:
	strh	r2, [r3, #130]	@ movhi
	b	.L3285
.L3294:
	add	r6, r6, #1
.L3287:
	ldr	r3, [sp, #36]
	cmp	r6, r3
	beq	.L3296
	add	r10, r5, #96
	mov	r3, #0
	str	r3, [sp, #44]
.L3297:
	ldrb	r3, [r5, #89]	@ zero_extendqisi2
	ldr	r2, [sp, #44]
	cmp	r2, r3
	bge	.L3294
	mov	r7, fp, asl #2
	mov	r9, #1
	b	.L3295
.L3329:
	ldr	r3, [sp, #44]
	add	r10, r10, #2
	add	r3, r3, #1
	str	r3, [sp, #44]
	b	.L3297
.L3296:
	mov	r0, r4
	mov	r4, #0
	bl	zbuf_free
	ldr	ip, .L3332
	movw	r2, #3722
	ldrb	r0, [r5, #89]	@ zero_extendqisi2
	ldrh	r3, [r5, #80]
	mov	r6, r4
	ldrh	r2, [ip, r2]
	mov	r8, r4
	ldr	r1, [ip, #576]
	mov	r10, ip
	mov	r3, r3, asl #1
	smulbb	r2, r0, r2
	strh	r2, [r1, r3]	@ movhi
.L3298:
	ldr	r3, [sp, #36]
	cmp	r6, r3
	beq	.L3309
	add	r3, r5, #96
	str	r3, [sp, #48]
	mov	r3, #0
	str	r3, [sp, #44]
.L3310:
	ldrb	r3, [r5, #89]	@ zero_extendqisi2
	ldr	r2, [sp, #44]
	cmp	r2, r3
	bge	.L3307
	mov	r7, #1
.L3308:
	ldr	r3, [sp, #40]
	cmp	r7, r3
	bhi	.L3331
	ldr	r3, [r10, #3724]
	ldr	r9, [r3, r8, asl #2]
	cmn	r9, #1
	beq	.L3299
	ldr	r3, [r10, #3728]
	mov	r0, r9
	ldr	fp, [r3, r8, asl #2]
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #60]
	bne	.L3300
	mov	r0, r9
	add	r1, sp, #60
	mov	r2, #0
	bl	pm_log2phys
.L3300:
	ldr	r3, [sp, #48]
	ldrh	r2, [r3]
	ldr	r3, .L3332+16
	ldrh	r3, [r3]
	mul	r3, r3, r2
	ldr	r2, .L3332
	ldrb	r2, [r2, #3768]	@ zero_extendqisi2
	cmp	r2, #3
	addeq	r3, r3, r6
	orreq	r3, r3, r7, asl #24
	beq	.L3327
	cmp	r2, #2
	subeq	r2, r6, #1
	addne	r3, r3, r6
	addeq	r3, r3, r2
	addeq	r3, r3, r7
	orreq	r3, r3, #33554432
.L3327:
	str	r3, [sp, #56]
	mov	r2, #1
	ldr	r3, .L3332+24
	ldrb	r1, [r10, #3766]	@ zero_extendqisi2
	str	r2, [sp, #52]
	ldrh	r0, [r3]
	rsb	r3, r0, #21
	mov	r3, r2, asl r3
	sub	r3, r3, #1
	and	r0, r3, fp, lsr r0
	bl	__aeabi_uidiv
	ldr	r3, [sp, #60]
	ldr	r2, [sp, #52]
	cmp	r3, fp
	uxth	fp, r0
	bne	.L3304
	mov	r0, r9
	add	r1, sp, #56
	bl	pm_log2phys
	mov	r0, fp
	add	r4, r4, #1
	bl	ftl_vpn_decrement
	b	.L3305
.L3304:
	ldr	r2, [sp, #56]
	cmp	r3, r2
	addeq	r4, r4, #1
.L3305:
	ldr	r3, .L3332
	add	r8, r8, #1
	ldr	r3, [r3, #568]
	add	r3, r3, fp, asl #2
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ands	r3, r3, #224
	bne	.L3299
	ldr	r2, [r10, #576]
	mov	fp, fp, asl #1
	ldrh	r1, [r2, fp]
	cmp	r1, #0
	strneh	r3, [r2, fp]	@ movhi
.L3299:
	add	r7, r7, #1
	b	.L3308
.L3331:
	ldr	r3, [sp, #44]
	add	r3, r3, #1
	str	r3, [sp, #44]
	ldr	r3, [sp, #48]
	add	r3, r3, #2
	str	r3, [sp, #48]
	b	.L3310
.L3307:
	add	r6, r6, #1
	b	.L3298
.L3309:
	ldr	r2, .L3332
	ldrh	r3, [r5, #80]
	ldr	r2, [r2, #576]
	mov	r3, r3, asl #1
	strh	r4, [r2, r3]	@ movhi
	ldrh	r0, [r5, #80]
	bl	zftl_insert_data_list
.L3285:
	ldrh	r0, [r5, #80]
	bl	ftl_dump_write_open_sblk
	mov	r2, r4
	ldrh	r1, [r5, #80]
	ldr	r0, .L3332+28
	ldr	r4, .L3332
	bl	printk
	mvn	r3, #0
	strh	r3, [r5, #80]	@ movhi
	bl	pm_flush
	bl	ftl_ext_info_flush
	ldr	r3, [r4, #580]
	ldrh	r0, [r3, #130]
	movw	r3, #65535
	cmp	r0, r3
	beq	.L3311
	bl	ftl_free_sblk
.L3311:
	ldr	r2, [r4, #3340]
	mvn	r3, #0
	mov	r0, #0
	strh	r3, [r2, #126]	@ movhi
	ldr	r2, [r4, #580]
	strh	r3, [r2, #130]	@ movhi
	bl	ftl_info_flush
	b	.L3279
.L3280:
	ldrh	r3, [r5, #130]
	cmp	r3, r2
	beq	.L3279
	ldr	r2, [r6, #3340]
	ldrh	r2, [r2, #126]
	cmp	r2, r3
	bne	.L3313
	bl	pm_flush
	ldr	r3, [r6, #580]
	ldrh	r0, [r3, #130]
	bl	ftl_free_sblk
	ldr	r3, [r6, #3340]
	mvn	r2, #0
	mov	r0, r4
	strh	r2, [r3, #126]	@ movhi
	bl	ftl_info_flush
.L3313:
	ldr	r3, [r6, #580]
	mvn	r2, #0
	strh	r2, [r3, #130]	@ movhi
.L3279:
	add	sp, sp, #68
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3333:
	.align	2
.L3332:
	.word	.LANCHOR0
	.word	-178307901
	.word	.LC228
	.word	.LANCHOR0+3712
	.word	.LANCHOR0+3806
	.word	.LC229
	.word	.LANCHOR0+3764
	.word	.LC230
	.fnend
	.size	gc_recovery, .-gc_recovery
	.align	2
	.global	zftl_init
	.type	zftl_init, %function
zftl_init:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r2, .L3404
	mvn	r3, #0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldr	r4, .L3404+4
	mov	ip, r2
	ldr	fp, .L3404+8
	mov	r7, #0
	ldr	r1, .L3404+12
	ldr	r0, .L3404+16
	str	r3, [ip, #-2816]!
	strb	r3, [fp, #-3841]
	strb	r3, [fp, #-3519]
	str	r2, [sp, #8]
	str	ip, [sp, #4]
	str	ip, [sp]
	strb	r3, [r4, #3360]
	strb	r7, [fp, #-3840]
	strb	r7, [r4, #3336]
	strb	r7, [r4, #3337]
	bl	printk
	ldr	r6, .L3404+20
	movw	r1, #3722
	ldrb	r3, [r6, #16]	@ zero_extendqisi2
	ldrh	r8, [r6, #14]
	ldrb	r10, [r6, #13]	@ zero_extendqisi2
	strh	r8, [r4, r1]	@ movhi
	mov	r0, r8
	strb	r10, [r4, #3312]
	mov	r9, r10, asl #9
	uxth	r9, r9
	str	r3, [sp]
	ldrb	r3, [sp]	@ zero_extendqisi2
	ldr	r1, [sp]
	strb	r3, [r4, #3768]
	bl	__aeabi_idiv
	add	r1, r4, #3712
	mov	r3, r0
	strh	r0, [r1]	@ movhi
	ldrb	r0, [r6, #17]	@ zero_extendqisi2
	ldrb	r1, [r4, #585]	@ zero_extendqisi2
	ldrh	r6, [r6, #18]
	strb	r0, [r4, #3766]
	strb	r1, [r4, #3808]
	smulbb	r1, r0, r1
	ldr	r2, [sp, #8]
	ldrh	r0, [r4, #2]
	ldr	ip, [sp, #4]
	str	fp, [sp, #4]
	mov	fp, r2
	uxtb	r5, r1
	add	r1, r4, #564
	strb	r5, [r4, #3720]
	strh	r6, [r1]	@ movhi
	movw	r1, #3806
	strh	r0, [r4, r1]	@ movhi
	mov	r1, #1
	strh	r9, [ip, #-6]	@ movhi
.L3335:
	cmp	r1, r0
	uxth	r2, r7
	add	r7, r7, #1
	movls	r1, r1, asl #1
	bls	.L3335
.L3402:
	movw	r1, #3764
	sub	r2, r2, #1
	strh	r2, [r4, r1]	@ movhi
	mov	ip, #0
	mul	r2, r8, r10
	mov	r0, #1
	mul	r1, r6, r2
	mov	lr, r1, lsr #21
.L3337:
	cmp	r0, lr
	uxth	r8, ip
	add	ip, ip, #1
	movls	r0, r0, asl #1
	bls	.L3337
.L3403:
	ldr	r0, .L3404+24
	sub	r8, r8, #1
	mov	ip, #1
	str	r2, [sp, #12]
	uxth	r8, r8
	str	r3, [sp, #8]
	mul	r0, r0, r5
	mov	r8, ip, asl r8
	mul	r1, r5, r1
	mul	r7, r8, r0
	str	r1, [r4, #3316]
	mov	r1, r10
	ldr	r10, .L3404+28
	add	r8, r7, #24576
	str	r8, [fp, #-2812]
	mov	r0, r8
	str	r7, [r4, #520]
	bl	__aeabi_uidiv
	sub	r1, r9, #1
	str	r0, [r4, #3320]
	add	r0, r1, r0, asl #2
	mov	r1, r9
	ldr	r9, .L3404+32
	bl	__aeabi_uidiv
	ldr	r3, [sp, #8]
	strh	r0, [r9, #-8]	@ movhi
	uxth	r0, r0
	mul	r1, r5, r3
	mov	r0, r0, asl #4
	bl	__aeabi_idiv
	ldr	r2, [sp, #12]
	strh	r0, [r10, #-14]	@ movhi
	mov	r0, r8
	mul	r1, r5, r2
	sub	r1, r1, #1
	bl	__aeabi_uidiv
	movw	r2, #3784
	sub	r1, r10, #16
	mov	ip, #256
	add	r0, r0, #8
	strh	r0, [r4, r2]	@ movhi
	add	r2, r6, r6, asl #1
	movw	r0, #3802
	ldr	r3, [sp]
	mov	r2, r2, asr #2
	strh	r2, [r1, #-6]	@ movhi
	mov	r2, r6, lsr #4
	ldr	r6, .L3404+36
	cmp	r2, #79
	strh	ip, [r4, r0]	@ movhi
	movw	r0, #3804
	mov	ip, #48
	strh	r2, [r6]	@ movhi
	movls	r2, #80
	strlsh	r2, [r6]	@ movhi
	mov	r2, #2000
	strh	r2, [r1, #-4]	@ movhi
	mov	r2, #50
	strh	r2, [r1, #-8]	@ movhi
	cmp	r3, #2
	ldr	r2, .L3404+4
	strh	ip, [r4, r0]	@ movhi
	mov	ip, #32
	add	r0, r2, #572
	strh	ip, [r0]	@ movhi
	beq	.L3340
	ldrb	r3, [r2, #3771]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3341
.L3340:
	movw	r2, #3804
	mov	r3, #150
	strh	r3, [r1, #-8]	@ movhi
	mov	r1, #64
	strh	r1, [r4, r2]	@ movhi
	mov	r1, #12
	strh	r1, [r0]	@ movhi
	ldrb	r1, [r4]	@ zero_extendqisi2
	ldr	r3, .L3404+4
	cmp	r1, #0
	add	r2, r3, #572
	bne	.L3341
	mov	r1, #4
	strh	r1, [r2]	@ movhi
	ldr	r2, .L3404+40
	mov	r1, #600
	strh	r1, [r2, #-4]	@ movhi
	movw	r2, #3802
	mov	r1, #128
	strh	r1, [r3, r2]	@ movhi
.L3341:
	ldr	r5, .L3404+20
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3343
	ldr	r0, .L3404+44
	mov	r1, r7
	bl	printk
.L3343:
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3344
	ldr	r0, .L3404+48
	ldr	r1, [r4, #3316]
	bl	printk
.L3344:
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3345
	ldr	r0, .L3404+52
	ldr	r1, [r4, #3320]
	bl	printk
.L3345:
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3346
	ldr	r0, .L3404+56
	ldr	r1, [fp, #-2812]
	bl	printk
.L3346:
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3347
	ldr	r0, .L3404+60
	ldrh	r1, [r9, #-8]
	bl	printk
.L3347:
	ldr	r3, [r5]
	ldr	r7, .L3404+64
	tst	r3, #4096
	beq	.L3348
	ldr	r0, .L3404+68
	ldrh	r1, [r7, #-6]
	bl	printk
.L3348:
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3349
	ldr	r0, .L3404+72
	ldrh	r1, [r10, #-14]
	bl	printk
.L3349:
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3350
	ldr	r0, .L3404+76
	ldrh	r1, [r6]
	bl	printk
.L3350:
	ldr	r3, [r5]
	tst	r3, #4096
	beq	.L3351
	movw	r3, #3784
	ldr	r0, .L3404+80
	ldrh	r1, [r4, r3]
	bl	printk
.L3351:
	ldr	r8, .L3404+84
	bl	zbuf_init
	mov	r0, #6
	ldr	r6, .L3404+88
	ldrh	r3, [r8]
	mul	r0, r0, r3
	bl	ftl_malloc
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	str	r0, [r4, #524]
	ldrh	r0, [r6]
	mul	r0, r3, r0
	mov	r0, r0, asl #2
	bl	ftl_malloc
	ldr	r2, .L3404+92
	ldrh	r1, [r6]
	ldr	r3, [sp, #4]
	str	r0, [r3, #-3856]
	str	r2, [r3, #-3860]
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	ldrh	r0, [r8]
	mul	r1, r3, r1
	mov	r0, r0, lsr #1
	add	r3, r2, r1, asl #3
	add	r1, r0, r1, asl #1
	ldr	r0, [r5]
	add	r1, r2, r1, asl #2
	str	r3, [r4, #576]
	tst	r0, #4096
	str	r1, [r4, #580]
	beq	.L3352
	ldr	r0, .L3404+96
	bl	printk
.L3352:
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	ldrh	r6, [r6]
	ldrh	r2, [r8]
	ldrh	r9, [r9, #-8]
	mul	r6, r3, r6
	ldr	r3, [r5]
	mov	r9, r9, asl #2
	tst	r3, #4096
	add	r9, r9, r2, asl #2
	add	r9, r9, #704
	add	r6, r2, r6, asl #2
	mov	r6, r6, asl #1
	add	r6, r6, #632
	beq	.L3353
	ldr	r0, .L3404+100
	mov	r1, r6
	mov	r2, r9
	ldrh	r3, [r7, #-6]
	bl	printk
.L3353:
	ldrh	r3, [r7, #-6]
	cmp	r9, r3
	cmpls	r6, r3
	movhi	r9, #1
	movls	r9, #0
	bls	.L3354
.L3400:
	b	.L3400
.L3354:
	bl	sblk_init
	bl	ftl_info_blk_init
	cmn	r0, #1
	beq	.L3356
	bl	ftl_ext_info_init
	bl	gc_init
	mov	r0, #1
	bl	pm_init
	bl	lpa_rebuild_hash
	ldr	r0, [r4, #580]
	mov	r1, r9
	add	r0, r0, #16
	bl	ftl_open_sblk_recovery
	ldr	r1, [r4, #580]
	add	r0, r1, #48
	add	r1, r1, #16
	bl	ftl_open_sblk_recovery
	ldr	r2, [r4, #3340]
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
	ldr	r3, .L3404+104
	ldrb	r3, [r3, #-3175]	@ zero_extendqisi2
	cmp	r3, #0
	ldrne	r3, .L3404+4
	ldrne	r2, [r3, #3340]
	ldrne	r3, [r2, #68]
	addne	r3, r3, #1
	strne	r3, [r2, #68]
	bl	ftl_ext_info_flush
	mov	r0, #0
	bl	ftl_info_flush
	bl	print_ftl_debug_info
	mov	r0, #0
.L3356:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3405:
	.align	2
.L3404:
	.word	.LANCHOR5
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LC2
	.word	.LC1
	.word	.LANCHOR2
	.word	1892352
	.word	.LANCHOR3-3824
	.word	.LANCHOR5-2800
	.word	.LANCHOR0+3744
	.word	.LANCHOR3-3840
	.word	.LC231
	.word	.LC232
	.word	.LC233
	.word	.LC234
	.word	.LC235
	.word	.LANCHOR5-2816
	.word	.LC236
	.word	.LC237
	.word	.LC238
	.word	.LC239
	.word	.LANCHOR0+564
	.word	.LANCHOR0+3712
	.word	.LANCHOR4-3112
	.word	.LC240
	.word	.LC241
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
	ldr	r3, .L3422
	mov	r7, #0
	ldr	r4, .L3422+4
	ldr	r5, .L3422+8
	ldr	r3, [r3]
	str	r7, [r4, #-3564]
	strb	r7, [r5, #-2806]
	str	r3, [r4, #-3568]
	bl	ftl_malloc
	cmp	r0, r7
	str	r0, [r4, #-3560]
	bne	.L3407
.L3409:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3407:
	mov	r0, #2048
	bl	ftl_malloc
	mov	r1, r5
	str	r7, [r1, #-2800]!
	str	r7, [r4, #-3556]
	str	r0, [r5, #-2804]
	ldr	r0, .L3422+12
	bl	rknand_get_reg_addr
	ldr	r3, [r4, #-3556]
	cmp	r3, #0
	beq	.L3409
	bl	rk_nandc_irq_init
	mov	r1, r7
	mov	r2, r7
	mov	r3, #2048
	ldr	r0, [r5, #-2804]
	bl	flash_sram_load_store
	bl	rknand_flash_cs_init
	ldr	r3, [r4, #-3560]
	ldr	r2, .L3422+16
	ldr	r0, [r4, #-3556]
	str	r2, [r3, #40]
	ldr	r2, .L3422+20
	str	r2, [r3, #32]
	ldr	r2, .L3422+24
	str	r2, [r3, #44]
	ldr	r2, .L3422+28
	str	r2, [r3, #48]
	ldr	r2, .L3422+32
	str	r2, [r3, #36]
	ldr	r2, .L3422+36
	str	r2, [r3, #56]
	ldr	r2, .L3422+40
	str	r2, [r3, #20]
	ldr	r2, .L3422+44
	str	r2, [r3, #24]
	ldr	r2, .L3422+48
	str	r2, [r3, #12]
	ldr	r2, .L3422+52
	str	r2, [r3, #16]
	ldr	r2, .L3422+56
	str	r2, [r3, #4]
	ldr	r2, .L3422+60
	str	r2, [r3, #8]
	ldr	r2, .L3422+64
	str	r2, [r3, #60]
	ldr	r2, .L3422+68
	str	r2, [r3, #64]
	ldr	r2, .L3422+72
	str	r2, [r3, #52]
	ldr	r2, .L3422+76
	str	r2, [r3, #28]
	bl	nand_flash_init
	subs	r7, r0, #0
	bne	.L3410
	bl	zftl_init
	mov	r4, r0
	bl	zftl_proc_debug_init
	mov	r3, #1
	strb	r3, [r5, #-2806]
	b	.L3411
.L3410:
	ldr	r3, [r4, #-3560]
	mov	r6, r4
	ldr	r2, .L3422+80
	ldr	r0, [r4, #-3556]
	ldr	r5, .L3422+84
	str	r2, [r3, #40]
	ldr	r2, .L3422+88
	str	r2, [r3, #32]
	ldr	r2, .L3422+92
	str	r2, [r3, #44]
	ldr	r2, .L3422+96
	str	r2, [r3, #48]
	ldr	r2, .L3422+100
	str	r2, [r3, #36]
	ldr	r2, .L3422+104
	str	r2, [r3, #56]
	ldr	r2, .L3422+108
	str	r2, [r3, #20]
	ldr	r2, .L3422+112
	str	r2, [r3, #24]
	ldr	r2, .L3422+116
	str	r2, [r3, #12]
	ldr	r2, .L3422+120
	str	r2, [r3, #16]
	ldr	r2, .L3422+124
	str	r2, [r3, #4]
	ldr	r2, .L3422+128
	str	r2, [r3, #8]
	ldr	r2, .L3422+132
	str	r2, [r3, #60]
	ldr	r2, .L3422+136
	str	r2, [r3, #64]
	ldr	r2, .L3422+140
	str	r2, [r3, #52]
	ldr	r2, .L3422+144
	str	r2, [r3, #28]
	bl	FlashInit
	cmn	r7, #2
	mov	r4, r0
	bne	.L3412
	ldr	r1, .L3422+148
	mov	r2, #32
	add	r0, r5, #4
	bl	ftl_memcpy
	ldrb	r0, [r5, #22]	@ zero_extendqisi2
	bl	flash_lsb_page_tbl_build
	ldrh	r3, [r5, #14]
	strh	r3, [r5, #30]	@ movhi
.L3412:
	ldr	r3, .L3422+152
	ldr	r2, [r3]
	ldr	r3, .L3422+156
	cmp	r2, r3
	ldr	r3, .L3422+160
	bne	.L3413
	ldr	r2, .L3422+164
	ldrb	r2, [r2]	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L3414
.L3413:
	mov	r2, #0
	strb	r2, [r3, #1625]
	strb	r2, [r3]
.L3414:
	ldr	r1, .L3422+168
	mov	r0, #1
	ldrh	ip, [r5, #14]
	mov	r2, #0
	str	r2, [r6, #-3532]
	ldrb	r1, [r1]	@ zero_extendqisi2
	strb	r1, [r3, #1624]
	ldr	r1, .L3422+172
	ldrb	r1, [r1]	@ zero_extendqisi2
	strb	r1, [r3, #584]
.L3415:
	cmp	r0, ip
	uxth	r1, r2
	add	r2, r2, #1
	movls	r0, r0, asl #1
	bls	.L3415
.L3421:
	movw	r2, #3764
	sub	r1, r1, #1
	strh	r1, [r3, r2]	@ movhi
	mov	r2, #0
	cmp	r4, r2
	strb	r2, [r3, #1658]
	bne	.L3411
	bl	FtlInit
	mov	r4, r0
.L3411:
	mov	r1, r4
	ldr	r0, .L3422+176
	bl	printk
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L3423:
	.align	2
.L3422:
	.word	jiffies
	.word	.LANCHOR3
	.word	.LANCHOR5
	.word	.LANCHOR3-3556
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
	.word	.LC242
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
	movw	r3, #3722
	ldr	r4, .L3460
	.pad #28
	sub	sp, sp, #28
	ldr	r5, [r4, #580]
	ldrh	r3, [r4, r3]
	ldrb	r2, [r5, #89]	@ zero_extendqisi2
	mul	r3, r2, r3
	str	r3, [sp]
	bl	timer_get_time
	ldr	r3, .L3460+4
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3425
	ldr	r0, .L3460+8
	ldrh	r1, [r5, #80]
	bl	printk
.L3425:
	ldrh	r3, [r5, #80]
	mov	r7, #0
	ldr	r2, [r4, #576]
	mov	r6, r7
	ldr	r9, .L3460
	ldr	fp, .L3460+4
	mov	r3, r3, asl #1
	ldr	r1, [sp]
	sub	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
.L3426:
	ldr	r3, [sp]
	cmp	r6, r3
	beq	.L3459
	ldr	r3, [r4, #3724]
	mov	r8, r6, asl #2
	ldr	r10, [r3, r6, asl #2]
	cmn	r10, #1
	beq	.L3427
	ldrb	r1, [r9, #3312]	@ zero_extendqisi2
	mov	r0, r10
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	uxth	r3, r0
	str	r3, [sp, #12]
	ldr	r3, [fp]
	tst	r3, #256
	beq	.L3428
	ldr	r0, .L3460+12
	mov	r2, r10
	ldr	r1, [sp, #12]
	mov	r3, r6
	bl	printk
.L3428:
	str	r6, [sp, #8]
.L3434:
	ldr	r3, [r4, #3724]
	ldr	r10, [r3, r8]
	cmn	r10, #1
	beq	.L3429
	ldrb	r1, [r9, #3312]	@ zero_extendqisi2
	mov	r0, r10
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	ldr	r3, [sp, #12]
	uxth	r0, r0
	cmp	r0, r3
	bne	.L3429
	ldr	r3, [r9, #3728]
	mov	r0, r10
	ldr	r3, [r3, r8]
	str	r3, [sp, #4]
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #20]
	bne	.L3430
	mov	r0, r10
	add	r1, sp, #20
	mov	r2, #0
	bl	pm_log2phys
.L3430:
	ldr	r3, [sp, #20]
	ldr	r2, [sp, #4]
	cmp	r3, r2
	bne	.L3431
	ldr	r3, [r4, #3740]
	add	r1, sp, #24
	mov	r2, #1
	mov	r0, r10
	add	r7, r7, #1
	ldr	r3, [r3, r8]
	str	r3, [r1, #-8]!
	bl	pm_log2phys
	ldr	r3, .L3460+16
	mvn	r0, #0
	ldrh	r3, [r3]
	rsb	r2, r3, #21
	ldr	r1, [sp, #4]
	mov	r3, r1, lsr r3
	ldrb	r1, [r4, #3766]	@ zero_extendqisi2
	bic	r0, r3, r0, asl r2
	bl	__aeabi_uidiv
	uxth	r0, r0
	b	.L3458
.L3431:
	ldr	r2, [fp]
	tst	r2, #256
	beq	.L3433
	ldr	r0, .L3460+20
	mov	r1, r10
	ldr	r2, [sp, #4]
	bl	printk
.L3433:
	ldrh	r0, [r5, #80]
.L3458:
	bl	ftl_vpn_decrement
	ldr	r3, [r4, #3724]
	mvn	r2, #0
	str	r2, [r3, r8]
	bl	timer_get_time
.L3429:
	ldr	r3, [sp, #8]
	add	r8, r8, #4
	ldr	r2, [sp]
	add	r3, r3, #1
	str	r3, [sp, #8]
	cmp	r3, r2
	bne	.L3434
.L3427:
	add	r6, r6, #1
	b	.L3426
.L3459:
	ldr	r3, .L3460+4
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3436
	ldrh	r3, [r5, #80]
	ldr	r1, [r4, #580]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	ldr	r0, .L3460+24
	ldrh	r1, [r1, #80]
	ldrh	r2, [r2, r3]
	mov	r3, r7
	bl	printk
.L3436:
	ldrh	r3, [r5, #80]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, r7
	beq	.L3437
	ldr	r1, .L3460+28
	mov	r2, #616
	ldr	r0, .L3460+32
	bl	printk
	bl	dump_stack
.L3437:
	ldrh	r3, [r5, #80]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	strh	r7, [r2, r3]	@ movhi
	ldrh	r0, [r5, #80]
	bl	zftl_insert_data_list
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3461:
	.align	2
.L3460:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC243
	.word	.LC244
	.word	.LANCHOR0+3764
	.word	.LC245
	.word	.LC246
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
	ldr	r5, .L3488
	mov	r0, r1
	ldr	r4, .L3488+4
	movw	ip, #65535
	ldrb	r3, [r5, #3376]	@ zero_extendqisi2
	ldrb	lr, [r5, #3720]	@ zero_extendqisi2
	add	r2, r4, r3, asl #1
	ldrh	r6, [r2, #36]
	ldrh	r2, [r4, #2]
.L3463:
	cmp	r6, ip
	bne	.L3486
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
	b	.L3463
.L3486:
	cmp	r0, #0
	mov	r0, #1
	strneh	r2, [r4, #2]	@ movhi
	cmp	r1, #0
	strneb	r3, [r5, #3376]
	bl	buf_alloc
	ldr	r7, .L3488
	mov	r10, #1
	ldr	r8, .L3488+8
	mov	r9, r0
.L3468:
	ldrb	r2, [r5, #3378]	@ zero_extendqisi2
	ldr	fp, .L3488
	cmp	r10, r2
	bgt	.L3487
	ldrh	r3, [r8]
	cmp	r2, #3
	ldreqh	r2, [r4, #2]
	mul	r3, r3, r6
	addeq	r3, r3, r2
	orreq	r3, r3, r10, asl #24
	beq	.L3485
	cmp	r2, #2
	ldrh	r2, [r4, #2]
	addne	r3, r3, r2
	addeq	r3, r3, r2, asl #1
	ldreqb	r2, [r7, #3768]	@ zero_extendqisi2
	subeq	r3, r3, #1
	addeq	r3, r3, r10
	orreq	r3, r3, r2, asl #24
.L3485:
	str	r3, [r9, #24]
	mov	r0, r9
	mov	r1, #1
	bl	sblk_read_page
	ldr	r3, [r9, #36]
	cmp	r3, #512
	cmnne	r3, #1
	beq	.L3473
	ldr	r3, [r9, #12]
	ldr	fp, [r3, #4]
	mov	r0, fp
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #4]
	bne	.L3474
	ldr	r3, [r5, #3320]
	cmp	fp, r3
	bcs	.L3474
	mov	r0, fp
	add	r1, sp, #4
	mov	r2, #0
	bl	pm_log2phys
.L3474:
	ldr	r3, [r9, #24]
	ldr	r2, [sp, #4]
	cmp	r3, r2
	ldreqh	r1, [r4, #20]
	ldreq	r2, [r5, #3736]
	streq	r3, [r2, r1, asl #2]
	ldreqh	r3, [r4, #20]
	addeq	r3, r3, #1
	streqh	r3, [r4, #20]	@ movhi
.L3473:
	ldrh	r3, [r4, #22]
	add	r10, r10, #1
	add	r3, r3, #1
	strh	r3, [r4, #22]	@ movhi
	b	.L3468
.L3487:
	mov	r0, r9
	bl	zbuf_free
	ldrb	r3, [fp, #3376]	@ zero_extendqisi2
	ldrb	r2, [fp, #3720]	@ zero_extendqisi2
	add	r3, r3, #1
	uxtb	r3, r3
	strb	r3, [fp, #3376]
	cmp	r2, r3
	ldreqh	r3, [r4, #2]
	addeq	r3, r3, #1
	streqh	r3, [r4, #2]	@ movhi
	moveq	r3, #0
	streqb	r3, [fp, #3376]
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3489:
	.align	2
.L3488:
	.word	.LANCHOR0
	.word	.LANCHOR0+3372
	.word	.LANCHOR0+3806
	.fnend
	.size	gc_scan_src_blk_one_page, .-gc_scan_src_blk_one_page
	.align	2
	.global	gc_scan_src_blk
	.type	gc_scan_src_blk, %function
gc_scan_src_blk:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	movw	r3, #3372
	ldr	r4, .L3521
	.pad #20
	sub	sp, sp, #20
	ldrh	r2, [r4, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L3491
	ldr	r1, .L3521+4
	movw	r2, #1051
	ldr	r0, .L3521+8
	bl	printk
	bl	dump_stack
.L3491:
	movw	r3, #3372
	ldrh	r1, [r4, r3]
	movw	r3, #65535
	cmp	r1, r3
	moveq	r0, #0
	beq	.L3492
	ldr	r3, .L3521+12
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3493
	ldr	r3, .L3521
	ldr	r0, .L3521+16
	ldr	r2, [r3, #576]
	mov	r3, r1, asl #1
	ldrh	r2, [r2, r3]
	bl	printk
.L3493:
	ldr	r6, .L3521
	movw	r7, #3372
	bl	timer_get_time
	ldrh	r0, [r4, r7]
	add	r1, r6, #3408
	ldr	r5, .L3521+20
	bl	ftl_get_blk_list_in_sblk
	uxtb	r0, r0
	strb	r0, [r4, #3377]
	cmp	r0, #0
	mvneq	r3, #0
	streqh	r3, [r6, r7]	@ movhi
	beq	.L3492
	ldrh	r3, [r6, r7]
	ldr	r2, [r6, #568]
	add	r2, r2, r3, asl #2
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	tst	r2, #192
	and	r1, r2, #224
	moveq	r2, #1
	movne	r2, #0
	cmp	r1, #224
	orreq	r2, r2, #1
	cmp	r2, #0
	bne	.L3495
	ldr	r2, [r6, #580]
	ldrh	ip, [r2, #16]
	cmp	ip, r3
	beq	.L3495
	ldrh	ip, [r2, #48]
	cmp	ip, r3
	beq	.L3495
	ldrh	r2, [r2, #80]
	cmp	r2, r3
	bne	.L3496
.L3495:
	cmp	r1, #0
	bne	.L3497
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L3498
	ldr	r1, .L3521+4
	movw	r2, #1071
	ldr	r0, .L3521+8
	bl	printk
	bl	dump_stack
.L3498:
	movw	r3, #3372
	ldr	r2, [r4, #576]
	ldrh	r3, [r4, r3]
	mov	r1, #0
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
.L3497:
	movw	r3, #3372
	mvn	r2, #0
	mov	r0, #0
	strh	r2, [r4, r3]	@ movhi
	strh	r0, [r5, #20]	@ movhi
	b	.L3492
.L3496:
	add	r0, r5, r0, asl #1
	movw	r3, #65535
	ldrh	r6, [r0, #34]
	cmp	r6, r3
	bne	.L3499
	ldr	r1, .L3521+4
	movw	r2, #1079
	ldr	r0, .L3521+8
	bl	printk
	bl	dump_stack
.L3499:
	ldr	r1, .L3521
	add	r8, r1, #3712
	ldrh	r3, [r8]
	sub	fp, r3, #1
	movw	r3, #3806
	ldrh	r2, [r4, r3]
	movw	r3, #3372
	ldrh	r9, [r4, r3]
	uxth	r7, fp
	ldr	fp, [r4, #568]
	mul	r2, r2, r6
	add	r9, fp, r9, asl #2
	ldrb	fp, [r9, #2]	@ zero_extendqisi2
	and	r6, fp, #224
	cmp	r6, #160
	movne	fp, #1
	bne	.L3500
	ldrb	r3, [r1, #3768]	@ zero_extendqisi2
	cmp	r3, #2
	uxth	fp, r3
	orr	r2, r2, r3, asl #24
	movweq	r3, #3722
	ldreqh	r7, [r1, r3]
	subeq	r7, r7, #1
	uxtheq	r7, r7
.L3500:
	mov	r0, #1
	orr	r6, r7, r2
	mov	r10, #0
	strb	fp, [r4, #3378]
	strh	r10, [r5, #2]	@ movhi
	strb	r10, [r4, #3376]
	strh	r10, [r5, #22]	@ movhi
	strh	r10, [r5, #24]	@ movhi
	bl	buf_alloc
	mov	r1, #1
	ldr	r9, .L3521
	str	r6, [r0, #24]
	mov	r7, r0
	bl	sblk_read_page
	movw	r3, #3722
	ldrh	r2, [r4, r3]
	mov	r1, #255
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	ldr	r0, [r4, #3736]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r6, [r7, #36]
	strh	r10, [r5, #20]	@ movhi
	cmn	r6, #1
	cmpne	r6, #512
	moveq	r6, #1
	movne	r6, #0
	beq	.L3519
	ldr	r3, [r7, #12]
	ldr	r2, [r3]
	ldr	r3, .L3521+24
	cmp	r2, r3
	beq	.L3502
	ldr	r1, .L3521+4
	movw	r2, #1112
	ldr	r0, .L3521+8
	bl	printk
	bl	dump_stack
.L3519:
	mov	r0, r7
	bl	zbuf_free
	mvn	r0, #0
	b	.L3492
.L3502:
	ldrh	r3, [r8]
	ldrb	r2, [r9, #3720]	@ zero_extendqisi2
	ldr	r8, [r7, #4]
	smulbb	r3, r2, r3
	sub	r8, r8, #4
	smulbb	r3, r3, fp
	ldr	fp, .L3521+28
	sub	r10, fp, #392
	uxth	r3, r3
	str	r3, [sp]
.L3503:
	ldr	r3, [sp]
	cmp	r6, r3
	bge	.L3520
	ldr	r0, [r8, #4]!
	cmn	r0, #1
	beq	.L3505
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #12]
	bne	.L3506
	ldr	r0, [r8]
	add	r1, sp, #12
	mov	r2, #0
	bl	pm_log2phys
.L3506:
	ldr	r2, [sp, #12]
	mvn	r0, #0
	ldrh	r3, [fp]
	rsb	r1, r3, #21
	mov	r3, r2, lsr r3
	bic	r0, r3, r0, asl r1
	ldrb	r1, [r4, #3766]	@ zero_extendqisi2
	str	r2, [sp, #4]
	bl	__aeabi_uidiv
	ldrh	r3, [r5]
	cmp	r0, r3
	ldr	r2, [sp, #4]
	ldreq	r3, [r9, #3736]
	ldreqh	r1, [r10, #20]
	streq	r2, [r3, r1, asl #2]
	ldreqh	r3, [r10, #20]
	addeq	r3, r3, #1
	streqh	r3, [r10, #20]	@ movhi
.L3505:
	bl	timer_get_time
	add	r6, r6, #1
	b	.L3503
.L3520:
	mov	r0, r7
	bl	zbuf_free
	movw	r3, #3372
	ldrh	r1, [r4, r3]
	ldr	r2, [r4, #576]
	mov	r3, r1, asl #1
	ldrh	r2, [r2, r3]
	ldrh	r3, [r5, #20]
	cmp	r2, r3
	beq	.L3509
	ldr	r0, .L3521+32
	bl	printk
.L3509:
	movw	r3, #3372
	ldrh	r1, [r5, #20]
	ldrh	r3, [r4, r3]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	mov	r3, #0
	ldrh	r0, [r5, #20]
	strh	r3, [r5, #24]	@ movhi
.L3492:
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3522:
	.align	2
.L3521:
	.word	.LANCHOR0
	.word	.LANCHOR1+1996
	.word	.LC0
	.word	.LANCHOR2
	.word	.LC247
	.word	.LANCHOR0+3372
	.word	-178307901
	.word	.LANCHOR0+3764
	.word	.LC248
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
	ldr	r4, .L3540
	ldr	r3, [r4, #580]
	ldr	r2, [r3, #544]
	cmn	r2, #1
	beq	.L3524
	mov	r5, #11
	mov	r6, r4
.L3525:
	ldr	r3, [r4, #580]
	add	r1, sp, #4
	mov	r2, #0
	ldr	r0, [r3, #544]
	bl	pm_log2phys
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L3526
	mov	r0, #1
	bl	buf_alloc
	mov	r1, #1
	ldr	r3, [sp, #4]
	mov	r7, r0
	str	r3, [r0, #24]
	bl	sblk_read_page
	ldr	r3, [r7, #36]
	cmp	r3, #256
	bne	.L3527
	ldr	r3, .L3540+4
	mov	r8, #1
	ldr	r0, [sp, #4]
	ldrb	r1, [r6, #3766]	@ zero_extendqisi2
	ldrh	r2, [r3]
	rsb	r3, r2, #21
	mov	r3, r8, asl r3
	sub	r3, r3, #1
	and	r0, r3, r0, lsr r2
	bl	__aeabi_uidiv
	mov	r1, r8
	mov	r2, #0
	uxth	r0, r0
	bl	gc_add_sblk
.L3527:
	ldr	r2, [r7, #12]
	ldr	r3, [r4, #580]
	ldr	r2, [r2, #4]
	ldr	r3, [r3, #544]
	cmp	r2, r3
	beq	.L3528
	ldr	r1, .L3540+8
	movw	r2, #1551
	ldr	r0, .L3540+12
	bl	printk
	bl	dump_stack
.L3528:
	mov	r0, r7
	bl	zbuf_free
.L3526:
	ldr	r3, [r4, #580]
	ldr	r1, [r4, #3320]
	ldr	r2, [r3, #544]
	add	r2, r2, #1
	str	r2, [r3, #544]
	cmp	r2, r1
	bcc	.L3529
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
	b	.L3523
.L3529:
	ldr	r3, [sp, #4]
	cmn	r3, #1
	bne	.L3523
	sub	r5, r5, #1
	uxth	r5, r5
	cmp	r5, #0
	bne	.L3525
	b	.L3523
.L3524:
	ldr	r2, [r3, #536]
	ldr	r1, [r3, #12]
	add	r2, r2, #12910592
	add	r2, r2, #49408
	cmp	r1, r2
	bhi	.L3534
	ldr	r0, [r4, #3340]
	ldr	r2, [r3, #540]
	ldr	r0, [r0, #44]
	add	r2, r2, #4992
	add	r2, r2, #8
	cmp	r0, r2
	bls	.L3523
.L3534:
	ldr	r2, [r4, #3340]
	ldr	r2, [r2, #44]
	str	r1, [r3, #536]
	str	r2, [r3, #540]
	mov	r2, #0
	str	r2, [r3, #544]
.L3523:
	add	sp, sp, #8
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L3541:
	.align	2
.L3540:
	.word	.LANCHOR0
	.word	.LANCHOR0+3764
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
	ldr	r5, .L3580
	add	r4, r5, #564
	ldr	r2, [r5, #580]
	ldrh	r3, [r4]
	ldr	r2, [r2, #608]
	cmp	r2, r3
	bcs	.L3542
	bl	timer_get_time
	ldr	r6, [r5, #580]
	ldr	r3, [r6, #604]
	add	r3, r3, #29952
	add	r3, r3, #48
	cmp	r0, r3
	bls	.L3542
	bl	timer_get_time
	ldr	r3, [r5, #580]
	ldrh	r1, [r4]
	str	r0, [r6, #604]
	ldr	r2, [r3, #600]
	cmp	r2, r1
	bcs	.L3546
	ldr	r1, [r5, #3340]
	ldrh	r1, [r1, #134]
	cmp	r2, r1
	bcs	.L3547
.L3546:
	ldr	r2, [r5, #3340]
	ldrh	r2, [r2, #134]
	str	r2, [r3, #600]
.L3547:
	ldr	r4, [r3, #600]
	movw	r3, #65535
	uxth	r4, r4
	cmp	r4, r3
	bne	.L3548
	ldr	r1, .L3580+4
	movw	r2, #1846
	ldr	r0, .L3580+8
	bl	printk
	bl	dump_stack
.L3548:
	ldr	r3, [r5, #580]
	mov	r0, r4
	add	r1, sp, #24
	ldr	r6, .L3580
	ldr	r2, [r3, #600]
	add	r2, r2, #1
	str	r2, [r3, #600]
	ldr	r2, [r3, #608]
	add	r2, r2, #1
	str	r2, [r3, #608]
	bl	ftl_get_blk_list_in_sblk
	uxth	r0, r0
	cmp	r0, #0
	beq	.L3542
	ldr	r8, [r6, #568]
	add	r8, r8, r4, asl #2
	ldrb	r3, [r8, #2]	@ zero_extendqisi2
	tst	r3, #192
	and	r2, r3, #224
	moveq	r3, #1
	movne	r3, #0
	cmp	r2, #224
	orreq	r3, r3, #1
	cmp	r3, #0
	bne	.L3549
	ldr	r3, [r6, #580]
	ldrh	r1, [r3, #16]
	cmp	r1, r4
	beq	.L3549
	ldrh	r1, [r3, #48]
	cmp	r1, r4
	beq	.L3549
	ldrh	r3, [r3, #80]
	cmp	r3, r4
	bne	.L3550
.L3549:
	cmp	r2, #0
	bne	.L3542
	ldr	r3, [r5, #576]
	mov	r4, r4, asl #1
	ldrh	r3, [r3, r4]
	cmp	r3, #0
	beq	.L3551
	ldr	r1, .L3580+4
	movw	r2, #1862
	ldr	r0, .L3580+8
	bl	printk
	bl	dump_stack
.L3551:
	ldr	r3, [r5, #576]
	mov	r2, #0
	strh	r2, [r3, r4]	@ movhi
	b	.L3542
.L3550:
	add	r3, sp, #40
	add	r0, r3, r0, asl #1
	movw	r3, #65535
	ldrh	r10, [r0, #-18]
	cmp	r10, r3
	bne	.L3553
	ldr	r1, .L3580+4
	movw	r2, #1868
	ldr	r0, .L3580+8
	bl	printk
	bl	dump_stack
.L3553:
	ldr	r1, .L3580
	movw	r3, #3806
	ldrb	r2, [r8, #2]	@ zero_extendqisi2
	add	r9, r1, #3712
	ldrh	fp, [r9]
	sub	r6, fp, #1
	and	fp, r2, #224
	cmp	fp, #160
	uxth	r7, r6
	ldrh	r6, [r5, r3]
	movne	fp, #1
	mul	r6, r6, r10
	bne	.L3554
	ldrb	r3, [r1, #3768]	@ zero_extendqisi2
	cmp	r3, #2
	uxth	fp, r3
	orr	r6, r6, r3, asl #24
	movweq	r3, #3722
	ldreqh	r7, [r1, r3]
	subeq	r7, r7, #1
	uxtheq	r7, r7
.L3554:
	mov	r0, #1
	orr	r6, r7, r6
	bl	buf_alloc
	mov	r1, #1
	str	r6, [r0, #24]
	mov	r7, r0
	bl	sblk_read_page
	movw	r3, #3722
	ldrh	r2, [r5, r3]
	mov	r1, #255
	ldrb	r3, [r5, #3720]	@ zero_extendqisi2
	ldr	r0, [r5, #3736]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldr	r2, [r7, #36]
	cmn	r2, #1
	cmpne	r2, #512
	moveq	r2, #1
	movne	r2, #0
	moveq	r0, r4
	moveq	r1, #1
	moveq	r2, #0
	beq	.L3578
	ldr	r3, [r7, #12]
	ldr	r1, [r3]
	ldr	r3, .L3580+12
	cmp	r1, r3
	beq	.L3556
	mov	r0, r4
	mov	r1, #1
.L3578:
	bl	gc_add_sblk
	mov	r0, r7
	bl	zbuf_free
	b	.L3542
.L3556:
	ldr	r3, .L3580
	ldrh	r1, [r9]
	ldr	r9, [r7, #4]
	ldrb	r6, [r3, #3720]	@ zero_extendqisi2
	smulbb	r6, r6, r1
	smulbb	r10, r6, fp
	mov	r6, r2
	mov	fp, #1
	uxth	r3, r10
	mov	r10, r2
	str	r3, [sp, #12]
.L3557:
	ldr	r3, [sp, #12]
	cmp	r10, r3
	bge	.L3579
	ldr	r0, [r9, r10, asl #2]
	cmn	r0, #1
	beq	.L3558
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #20]
	bne	.L3559
	ldr	r0, [r9, r10, asl #2]
	add	r1, sp, #20
	mov	r2, #0
	bl	pm_log2phys
.L3559:
	ldr	r3, .L3580+16
	ldr	r0, [sp, #20]
	ldrh	r1, [r3]
	rsb	r2, r1, #21
	mov	r2, fp, asl r2
	sub	r2, r2, #1
	and	r0, r2, r0, lsr r1
	ldrb	r1, [r5, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	cmp	r0, r4
	addeq	r6, r6, #1
	uxtheq	r6, r6
.L3558:
	add	r10, r10, #1
	b	.L3557
.L3579:
	mov	r0, r7
	mov	r7, r4, asl #1
	bl	zbuf_free
	ldr	r3, .L3580+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3561
	ldr	r3, [r5, #576]
	mov	r1, r4
	ldr	r0, .L3580+24
	ldrh	r2, [r3, r7]
	ldrb	r3, [r8, #2]	@ zero_extendqisi2
	mov	r3, r3, lsr #5
	str	r3, [sp]
	mov	r3, r6
	bl	printk
.L3561:
	ldr	r3, [r5, #576]
	cmp	r6, #31
	strh	r6, [r3, r7]	@ movhi
	bhi	.L3542
	mov	r0, r4
	mov	r1, #1
	mov	r2, #0
	bl	gc_add_sblk
.L3542:
	add	sp, sp, #44
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3581:
	.align	2
.L3580:
	.word	.LANCHOR0
	.word	.LANCHOR1+2032
	.word	.LC0
	.word	-178307901
	.word	.LANCHOR0+3764
	.word	.LANCHOR2
	.word	.LC249
	.fnend
	.size	gc_block_vpn_scan, .-gc_block_vpn_scan
	.align	2
	.global	ftl_sblk_dump
	.type	ftl_sblk_dump, %function
ftl_sblk_dump:
	.fnstart
	@ args = 0, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	mov	fp, r0, asl #2
	ldr	r6, .L3616
	.pad #124
	sub	sp, sp, #124
	mov	r7, r0
	str	r1, [sp, #52]
	ldr	r2, [r6, #568]
	add	r3, r2, fp
	ldrh	r1, [r2, fp]
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	ubfx	r1, r1, #0, #11
	str	r1, [sp]
	ldr	r10, [r2, r0, asl #2]
	mov	r1, r7
	mov	r2, r3, lsr #5
	ldr	r0, .L3616+4
	ubfx	r3, r3, #3, #2
	ubfx	r4, r10, #11, #8
	str	r4, [sp, #4]
	bl	printk
	movw	r3, #65535
	cmp	r7, r3
	beq	.L3603
	add	r3, r6, #564
	ldrh	r3, [r3]
	cmp	r3, r7
	bls	.L3603
	ldr	r3, [r6, #568]
	add	r1, sp, #104
	mov	r0, r7
	strh	r7, [sp, #88]	@ movhi
	add	r3, r3, fp
	ldr	r4, .L3616+8
	mov	r5, #0
	ldrb	r3, [r3, #2]	@ zero_extendqisi2
	mov	r9, r5
	and	r3, r3, #224
	cmp	r3, #160
	ldreqb	r10, [r6, #3768]	@ zero_extendqisi2
	movne	r10, #1
	bl	ftl_get_blk_list_in_sblk
	ldrh	r3, [r4]
	mov	r1, r7
	uxtb	r0, r0
	strh	r5, [sp, #90]	@ movhi
	strb	r0, [sp, #97]
	smulbb	r3, r3, r0
	strb	r5, [sp, #93]
	strh	r5, [sp, #98]	@ movhi
	strh	r3, [sp, #94]	@ movhi
	ldr	r3, [r6, #568]
	add	r3, r3, fp
	ldrb	r2, [r3, #2]	@ zero_extendqisi2
	mov	r3, r10
	str	r0, [sp]
	ldr	r0, .L3616+12
	mov	r2, r2, lsr #5
	bl	printk
	mov	r0, #1
	bl	buf_alloc
	sub	r3, r4, #3712
	mov	r6, r0
	str	r5, [sp, #40]
	str	r5, [sp, #44]
	str	r3, [sp, #60]
.L3585:
	ldr	r4, .L3616
	uxth	r2, r5
	str	r2, [sp, #56]
	add	r3, r4, #3712
	ldrh	r3, [r3]
	cmp	r3, r2
	bls	.L3600
	mov	r3, r2, asl #1
	sub	r3, r3, #1
	str	r3, [sp, #64]
	mov	r3, #0
.L3614:
	str	r3, [sp, #48]
	ldrb	r2, [sp, #97]	@ zero_extendqisi2
	ldrh	r3, [sp, #48]
	cmp	r2, r3
	bls	.L3598
	add	r3, r3, #8
	mov	r8, #1
	mov	r3, r3, asl r8
	str	r3, [sp, #68]
.L3599:
	cmp	r8, r10
	bhi	.L3615
	ldr	r2, [sp, #68]
	add	r3, sp, #120
	add	r3, r3, r2
	movw	r2, #65535
	ldrh	r3, [r3, #-32]
	cmp	r3, r2
	beq	.L3586
	ldr	r2, .L3616+16
	cmp	r10, #3
	ldrh	r4, [r2]
	mul	r4, r4, r3
	uxtaheq	r4, r4, r5
	orreq	r4, r4, r8, asl #24
	beq	.L3588
	cmp	r10, #2
	uxtahne	r4, r4, r5
	ldreq	r2, [sp, #64]
	addeq	r4, r4, r2
	ldreq	r2, [sp, #60]
	addeq	r4, r4, r8
	ldreqb	r2, [r2, #3768]	@ zero_extendqisi2
	orreq	r4, r4, r2, asl #24
.L3588:
	mov	r1, #1
	str	r4, [r6, #24]
	mov	r0, r6
	str	r3, [sp, #76]
	bl	sblk_read_page
	ldr	r1, [r6, #4]
	ldr	ip, [r6, #36]
	ldr	r2, [r6, #12]
	str	ip, [sp]
	ldr	r0, [r1]
	ldr	r3, [sp, #76]
	str	ip, [sp, #72]
	str	r0, [sp, #4]
	ldr	r0, [r1, #4]
	str	r0, [sp, #8]
	ldr	r0, [r1, #8]
	str	r0, [sp, #12]
	ldr	r1, [r1, #12]
	ldr	r0, .L3616+20
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
	ldr	r2, [sp, #56]
	bl	printk
	ldr	ip, [sp, #72]
	ldr	r3, [sp, #44]
	cmn	ip, #1
	cmpne	ip, #512
	moveq	r3, #1
	str	r3, [sp, #44]
	ldr	r3, [sp, #60]
	ldr	r3, [r3, #568]
	add	r3, r3, fp
	ldrb	r2, [r3, #2]	@ zero_extendqisi2
	and	r2, r2, #224
	cmp	r2, #224
	cmpne	r2, #32
	moveq	r2, #1
	movne	r2, #0
	beq	.L3586
	ldr	r3, [r6, #12]
	str	r2, [sp, #72]
	ldr	r0, [r3, #4]
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #84]
	ldr	r2, [sp, #72]
	bne	.L3591
	ldr	r3, [r6, #12]
	add	r1, sp, #84
	ldr	r0, [r3, #4]
	bl	pm_log2phys
.L3591:
	ldr	r3, [sp, #84]
	cmp	r3, r4
	bne	.L3592
	ldr	r3, [sp, #40]
	mov	r1, r4
	ldr	r0, .L3616+24
	add	r3, r3, #1
	str	r3, [sp, #40]
	ldr	r3, [r6, #12]
	ldr	r2, [r3, #4]
	ldr	r3, [sp, #40]
	bl	printk
.L3592:
	ldr	r3, [sp, #52]
	cmp	r3, #0
	beq	.L3594
	ldr	r2, [r3, r9, asl #2]
	mov	r4, r9, asl #2
	ldr	r3, [r6, #12]
	ldr	r3, [r3, #4]
	cmp	r3, r2
	beq	.L3595
	ldr	r3, .L3616+28
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3595
	ldr	r0, .L3616+32
	mov	r1, r9
	bl	printk
.L3595:
	ldr	r3, [sp, #52]
	ldr	r2, [r6, #12]
	ldr	r3, [r3, r4]
	ldr	r2, [r2, #4]
	cmp	r2, r3
	cmnne	r3, #1
	beq	.L3594
	ldr	r1, .L3616+36
	mov	r2, #1312
	ldr	r0, .L3616+40
	bl	printk
	bl	dump_stack
.L3594:
	add	r9, r9, #1
.L3586:
	add	r8, r8, #1
	uxth	r8, r8
	b	.L3599
.L3615:
	ldr	r3, [sp, #48]
	add	r3, r3, #1
	b	.L3614
.L3598:
	add	r5, r5, #1
	b	.L3585
.L3600:
	mov	r0, r6
	bl	zbuf_free
	ldr	r2, [r4, #576]
	mov	r3, r7, asl #1
	ldr	r0, .L3616+44
	mov	r1, r7
	ldrh	r2, [r2, r3]
	ldr	r3, [sp, #40]
	bl	printk
	ldr	r0, [sp, #44]
	b	.L3583
.L3603:
	mov	r0, #0
.L3583:
	add	sp, sp, #124
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3617:
	.align	2
.L3616:
	.word	.LANCHOR0
	.word	.LC250
	.word	.LANCHOR0+3712
	.word	.LC251
	.word	.LANCHOR0+3806
	.word	.LC197
	.word	.LC252
	.word	.LANCHOR2
	.word	.LC253
	.word	.LANCHOR1+2052
	.word	.LC0
	.word	.LC254
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
	ldr	r3, .L3679
	.pad #76
	sub	sp, sp, #76
	mov	r6, r0
	mov	r4, r1
	mov	r7, r2
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3619
	ldr	r0, .L3679+4
	mov	r1, r6
	mov	r2, r4
	mov	r3, r7
	bl	printk
.L3619:
	cmp	r6, #0
	ldreq	r3, .L3679+8
	ldreq	r2, [r3, #520]
	moveq	r3, #24576
	beq	.L3621
.L3620:
	cmp	r6, #3
	bhi	.L3647
	mov	r3, r6, asl #13
	mov	r2, #8192
.L3621:
	add	r1, r4, r7
	cmp	r1, r2
	bhi	.L3647
	ldr	r2, .L3679+8
	add	r3, r3, r4
	str	r3, [sp, #28]
	ldr	r0, [sp, #28]
	ldr	r1, [r2, #3340]
	ldrb	r4, [r2, #3312]	@ zero_extendqisi2
	ldr	r8, .L3679+12
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
.L3623:
	ldr	r3, [sp, #32]
	cmp	r3, #0
	beq	.L3676
	ldr	r3, [sp, #36]
	ldr	r6, .L3679+8
	rsb	r2, r3, r4
	ldr	r3, [sp, #48]
	clz	r2, r2
	rsb	r3, r3, r4
	ldrb	r1, [r6, #3312]	@ zero_extendqisi2
	clz	r3, r3
	mov	r2, r2, lsr #5
	mov	r9, r6
	mov	r3, r3, lsr #5
	uxth	r10, r1
	orrs	fp, r3, r2
	beq	.L3624
	cmp	r2, #0
	beq	.L3625
	ldr	r0, [sp, #28]
	bl	__aeabi_uidivmod
	uxth	fp, r1
	rsb	r10, fp, r10
	uxth	r10, r10
	cmp	r10, r7
	uxthhi	r10, r7
	b	.L3624
.L3625:
	cmp	r3, #0
	ldrne	r3, [sp, #28]
	movne	fp, r2
	moveq	fp, r3
	addne	r3, r3, r7
	mlsne	r1, r4, r1, r3
	uxtbne	r10, r1
.L3624:
	ldr	r2, .L3679+16
	mov	r3, #0
.L3628:
	ldr	r1, [r2, #20]
	cmp	r1, r4
	bne	.L3626
	ldrb	r1, [r2, #2]	@ zero_extendqisi2
	tst	r1, #8
	beq	.L3626
	ldr	r1, .L3679+8
	mov	r2, #48
	mov	r10, r10, asl #9
	mov	r0, r5
	add	r5, r5, r10
	mla	r3, r2, r3, r1
	mov	r2, r10
	ldr	r1, [r3, #1780]
	add	r1, r1, fp, asl #9
	bl	ftl_memcpy
	b	.L3627
.L3626:
	add	r3, r3, #1
	add	r2, r2, #48
	cmp	r3, #32
	bne	.L3628
	mov	r0, r4
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #68]
	bne	.L3629
	mov	r0, r4
	add	r1, sp, #68
	mov	r2, #0
	bl	pm_log2phys
.L3629:
	ldr	r3, [sp, #68]
	cmn	r3, #1
	moveq	r10, #0
	bne	.L3677
.L3631:
	ldrb	r3, [r9, #3312]	@ zero_extendqisi2
	cmp	r10, r3
	bcs	.L3627
	mla	r3, r3, r4, r10
	ldr	r2, [sp, #28]
	ldr	r1, [sp, #44]
	cmp	r3, r2
	movcs	r2, #1
	movcc	r2, #0
	cmp	r3, r1
	movcs	r2, #0
	cmp	r2, #0
	beq	.L3632
	mov	r0, r5
	mov	r1, #512
	bl	__memzero
	add	r5, r5, #512
.L3632:
	add	r10, r10, #1
	b	.L3631
.L3677:
	mov	r0, #0
	bl	buf_alloc
	subs	r3, r0, #0
	beq	.L3634
	ldr	r2, .L3679+8
	ldr	r1, [r2, #3340]
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
.L3627:
	ldr	r3, [sp, #32]
	add	r4, r4, #1
	sub	r3, r3, #1
	str	r3, [sp, #32]
.L3634:
	ldrb	r3, [r6, #3314]	@ zero_extendqisi2
	ldr	r2, [sp, #32]
	cmp	r2, #0
	cmpne	r3, #2
	bhi	.L3623
	ldrb	r1, [r8, #-3840]	@ zero_extendqisi2
	cmp	r1, #0
	beq	.L3623
	ldrb	r2, [r8, #-3841]	@ zero_extendqisi2
	mov	r0, #48
	ldr	r3, .L3679+16
	mla	r0, r0, r2, r3
	bl	sblk_read_page
.L3636:
	ldrb	r3, [r8, #-3840]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3678
	ldrb	r3, [r8, #-3841]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L3637
	ldr	r1, .L3679+20
	movw	r2, #1080
	ldr	r0, .L3679+24
	bl	printk
	bl	dump_stack
.L3637:
	ldr	r3, .L3679+12
	mov	r2, #48
	ldr	fp, .L3679+8
	ldr	r1, .L3679+12
	ldrb	r10, [r3, #-3841]	@ zero_extendqisi2
	add	r3, fp, #1776
	mul	r2, r2, r10
	add	r3, r3, r2
	add	r2, fp, r2
	str	r3, [sp, #52]
	ldr	r9, [r2, #1812]
	ldrb	r3, [r2, #1776]	@ zero_extendqisi2
	cmn	r9, #1
	strb	r3, [r1, #-3841]
	bne	.L3638
	ldr	r2, [fp, #580]
	str	r9, [sp, #40]
	ldr	r3, [r2, #552]
	add	r3, r3, #1
	str	r3, [r2, #552]
	b	.L3639
.L3638:
	cmp	r9, #256
	bne	.L3639
	ldr	r1, .L3679+28
	mov	r6, #1
	ldr	r3, [r2, #1800]
	str	r2, [sp, #60]
	ldrh	r0, [r1]
	str	r3, [sp, #56]
	rsb	r1, r0, #21
	mov	r1, r6, asl r1
	sub	r1, r1, #1
	and	r0, r1, r3, lsr r0
	ldrb	r1, [fp, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	ldr	r2, [sp, #60]
	uxth	fp, r0
	ldr	r3, [sp, #56]
	mov	r1, fp
	ldr	r0, .L3679+32
	ldr	r2, [r2, #1796]
	bl	printk
	mov	r0, fp
	mov	r1, r6
	mov	r2, #0
	bl	gc_add_sblk
.L3639:
	ldr	ip, .L3679+8
	mov	r6, #48
	cmn	r9, #1
	mla	r6, r6, r10, ip
	movne	fp, #0
	moveq	fp, #1
	ldr	r3, [r6, #1788]
	ldr	r2, [r6, #1796]
	ldr	r3, [r3, #4]
	cmp	r3, r2
	moveq	r3, fp
	orrne	r3, fp, #1
	cmp	r3, #0
	beq	.L3640
	ldr	r2, [ip, #580]
	ldr	r0, .L3679+36
	str	ip, [sp, #56]
	ldr	r3, [r2, #552]
	add	r3, r3, #1
	str	r3, [r2, #552]
	ldr	r3, [r6, #1788]
	ldr	r2, [r6, #1800]
	ldrb	r1, [r6, #1777]	@ zero_extendqisi2
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
	ldr	r3, [r6, #1796]
	bl	printk
	ldr	r3, .L3679+28
	mvn	r0, #0
	ldrh	r2, [r3]
	ldr	r3, [r6, #1800]
	rsb	r1, r2, #21
	mov	r3, r3, lsr r2
	bic	r0, r3, r0, asl r1
	ldr	ip, [sp, #56]
	ldrb	r1, [ip, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	mov	r1, #0
	uxth	r0, r0
	bl	ftl_sblk_dump
.L3640:
	ldr	r6, .L3679+8
	mov	r3, #48
	mla	r3, r3, r10, r6
	mov	r9, r6
	ldr	r2, [r3, #1788]
	ldr	r3, [r3, #1796]
	ldr	r2, [r2, #4]
	cmp	r2, r3
	orrne	fp, fp, #1
	cmp	fp, #0
	beq	.L3641
	ldr	r1, .L3679+20
	movw	r2, #1101
	ldr	r0, .L3679+24
	bl	printk
	bl	dump_stack
.L3641:
	mov	r3, #48
	mla	r10, r3, r10, r9
	ldrb	r3, [r6, #3312]	@ zero_extendqisi2
	ldrb	r2, [r10, #1816]	@ zero_extendqisi2
	cmp	r3, r2
	ldrlsb	r3, [r10, #1778]	@ zero_extendqisi2
	bicls	r3, r3, #8
	strlsb	r3, [r10, #1778]
	bls	.L3643
	ldrb	r1, [r10, #1817]	@ zero_extendqisi2
	mov	r2, r2, asl #9
	ldr	r3, [r10, #1780]
	ldr	r0, [r10, #1784]
	add	r1, r3, r1, asl #9
	bl	ftl_memcpy
.L3643:
	ldr	r1, [sp, #52]
	ldr	r0, .L3679+40
	bl	buf_remove_buf
	ldr	r0, [sp, #52]
	bl	zbuf_free
	ldrb	r3, [r8, #-3840]	@ zero_extendqisi2
	sub	r3, r3, #1
	strb	r3, [r8, #-3840]
	b	.L3636
.L3678:
	ldr	r1, .L3679+12
	mvn	r2, #0
	strb	r3, [r1, #-3840]
	strb	r2, [r1, #-3841]
	b	.L3623
.L3676:
	bl	timer_get_time
	ldr	r3, .L3679+44
	str	r0, [r3, #-2796]
	ldr	r0, [sp, #40]
	b	.L3622
.L3647:
	mvn	r0, #0
.L3622:
	add	sp, sp, #76
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3680:
	.align	2
.L3679:
	.word	.LANCHOR2
	.word	.LC255
	.word	.LANCHOR0
	.word	.LANCHOR3
	.word	.LANCHOR0+1776
	.word	.LANCHOR1+2068
	.word	.LC0
	.word	.LANCHOR0+3764
	.word	.LC256
	.word	.LC257
	.word	.LANCHOR3-3839
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
	bhi	.L3685
	mov	r0, r4
	bl	rk_copy_from_user
	cmp	r0, #0
	mvnne	r0, #13
	bne	.L3685
	add	r3, sp, #120
	mov	r1, r4
	add	r3, r3, r5
	strb	r0, [r3, #-80]
	ldr	r0, .L3710
	bl	printk
	ldr	r0, .L3710+4
	mov	r1, r4
	mov	r2, #1
	mov	r3, #16
	bl	rknand_print_hex
	bl	rknand_device_lock
	mov	r0, r4
	ldr	r1, .L3710+8
	mov	r2, #7
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3686
	ldr	r4, .L3710+12
	movw	ip, #698
	ldr	r0, .L3710+16
	mov	r2, #4
	ldr	r7, .L3710+20
	movw	r8, #65535
	ldr	r3, [r4, #3340]
	add	r1, r3, #704
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldr	r1, [r4, #3340]
	ldr	r0, .L3710+24
	mov	r2, #2
	add	r1, r1, #416
	ldrh	r3, [r7, #-14]!
	bl	rknand_print_hex
.L3687:
	ldrh	r3, [r7]
	cmp	r6, r3
	bge	.L3690
	mov	r0, #300
	bl	msleep
	add	r3, r6, #208
	ldr	r2, [r4, #3340]
	mov	r3, r3, asl #1
	ldrh	r0, [r2, r3]
	cmp	r0, r8
	beq	.L3688
	mov	r1, #0
	bl	ftl_sblk_dump
.L3688:
	add	r6, r6, #1
	b	.L3687
.L3686:
	mov	r0, r4
	ldr	r1, .L3710+28
	mov	r2, #7
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3691
	ldr	r7, .L3710+12
	movw	ip, #698
	ldr	r0, .L3710+16
	mov	r2, #4
	ldr	r9, .L3710+20
	ldr	r3, [r7, #3340]
	sub	r9, r9, #8
	add	r1, r3, #704
	ldrh	r3, [r3, ip]
	bl	rknand_print_hex
	ldr	r1, [r7, #3340]
	ldrh	r3, [r9, #-6]
	mov	r2, #2
	ldr	r0, .L3710+24
	add	r1, r1, #416
	bl	rknand_print_hex
	ldr	r0, .L3710+32
	add	r4, sp, #120
	add	r1, sp, #47
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	mov	r4, r6
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.33
	uxth	r8, r0
	str	r0, [sp, #32]
.L3693:
	add	r6, r9, r4
	ldr	r0, .L3710+36
	ldrh	r1, [r9, r4]
	ldrh	r2, [r6, #2]
	bl	printk
	ldrh	r3, [r4, r9]
	cmp	r3, r8
	bne	.L3692
	ldrb	r3, [r7, #3312]	@ zero_extendqisi2
	mov	r2, #4
	ldr	r0, .L3710+40
	ldr	r1, [r6, #4]
	mov	r3, r3, asl #7
	bl	rknand_print_hex
.L3692:
	add	r4, r4, #8
	cmp	r4, #256
	bne	.L3693
	mov	r0, #300
	add	r8, r8, #176
	bl	msleep
	mov	r0, #1
	bl	buf_alloc
	ldr	r3, [r7, #3340]
	mov	r1, #1
	ldr	r3, [r3, r8, asl #2]
	str	r3, [r0, #24]
	mov	r4, r0
	str	r3, [sp, #32]
	bl	sblk_read_page
	ldr	ip, [r4, #4]
	ldr	r3, [r4, #12]
	ldr	r0, .L3710+44
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
	ldrb	r3, [r7, #3312]	@ zero_extendqisi2
	ldr	r0, .L3710+48
	mov	r2, #4
	ldr	r1, [r4, #4]
	mov	r3, r3, asl #7
	b	.L3709
.L3691:
	mov	r0, r4
	ldr	r1, .L3710+52
	mov	r2, #7
	bl	memcmp
	cmp	r0, #0
	bne	.L3694
	bl	dump_ftl_info
	b	.L3690
.L3694:
	mov	r0, r4
	ldr	r1, .L3710+56
	mov	r2, #9
	bl	memcmp
	cmp	r0, #0
	bne	.L3695
	add	r1, sp, #120
	add	r0, sp, #49
	str	r0, [r1, #-84]!
	bl	rk_simple_strtoull.constprop.33
	ldr	r2, .L3710+12
	movw	r3, #3800
	str	r0, [sp, #32]
	strh	r0, [r2, r3]	@ movhi
	bl	dump_all_list_info
	b	.L3690
.L3695:
	mov	r0, r4
	ldr	r1, .L3710+60
	mov	r2, #8
	bl	memcmp
	cmp	r0, #0
	beq	.L3690
	mov	r0, r4
	ldr	r1, .L3710+64
	mov	r2, #8
	bl	memcmp
	cmp	r0, #0
	bne	.L3697
	add	r4, sp, #120
	add	r1, sp, #48
	ldr	r0, .L3710+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r6, .L3710+12
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.33
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
	ldr	r0, .L3710+44
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
	ldrb	r3, [r6, #3312]	@ zero_extendqisi2
	ldr	r0, .L3710+68
	mov	r2, #4
	ldr	r1, [r4, #4]
	mov	r3, r3, asl #7
	bl	rknand_print_hex
	ldrb	r3, [r6, #3312]	@ zero_extendqisi2
	ldr	r0, .L3710+72
	mov	r2, #4
	ldr	r1, [r4, #12]
	mov	r3, r3, asl #1
.L3709:
	bl	rknand_print_hex
	mov	r0, r4
	bl	zbuf_free
	b	.L3690
.L3697:
	mov	r0, r4
	ldr	r1, .L3710+76
	mov	r2, #8
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3698
	add	r4, sp, #120
	add	r1, sp, #48
	ldr	r0, .L3710+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.33
	mov	r1, r6
	str	r0, [sp, #32]
	uxth	r0, r0
	bl	ftl_sblk_dump
	b	.L3690
.L3698:
	mov	r0, r4
	ldr	r1, .L3710+80
	mov	r2, #10
	bl	memcmp
	cmp	r0, #0
	bne	.L3699
	add	r4, sp, #120
	add	r1, sp, #50
	ldr	r0, .L3710+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.33
	ldr	r3, .L3710+84
	str	r0, [sp, #32]
	str	r0, [r3]
	b	.L3690
.L3699:
	mov	r0, r4
	ldr	r1, .L3710+88
	mov	r2, #8
	bl	memcmp
	subs	r6, r0, #0
	bne	.L3700
	add	r4, sp, #120
	add	r1, sp, #48
	ldr	r0, .L3710+32
	str	r1, [r4, #-84]!
	bl	printk
	mov	r1, r4
	ldr	r0, [sp, #36]
	bl	rk_simple_strtoull.constprop.33
	mov	r4, r0
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #32]
	bne	.L3701
	mov	r0, r4
	add	r1, sp, #32
	mov	r2, r6
	bl	pm_log2phys
.L3701:
	mov	r1, r4
	ldr	r2, [sp, #32]
	ldr	r0, .L3710+92
	bl	printk
	b	.L3690
.L3700:
	ldr	r0, .L3710+96
	bl	printk
	ldr	r0, .L3710+100
	bl	printk
	ldr	r0, .L3710+104
	bl	printk
	ldr	r0, .L3710+108
	bl	printk
	ldr	r0, .L3710+112
	bl	printk
	ldr	r0, .L3710+116
	bl	printk
	ldr	r0, .L3710+120
	bl	printk
	ldr	r0, .L3710+124
	bl	printk
	ldr	r0, .L3710+128
	bl	printk
.L3690:
	bl	rknand_device_unlock
	mov	r0, r5
.L3685:
	add	sp, sp, #124
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L3711:
	.align	2
.L3710:
	.word	.LC258
	.word	.LC259
	.word	.LC260
	.word	.LANCHOR0
	.word	.LC261
	.word	.LANCHOR3-3824
	.word	.LC262
	.word	.LC263
	.word	.LC264
	.word	.LC265
	.word	.LC266
	.word	.LC228
	.word	.LC267
	.word	.LC268
	.word	.LC269
	.word	.LC270
	.word	.LC271
	.word	.LC223
	.word	.LC224
	.word	.LC272
	.word	.LC273
	.word	.LANCHOR2
	.word	.LC274
	.word	.LC275
	.word	.LC276
	.word	.LC277
	.word	.LC278
	.word	.LC279
	.word	.LC280
	.word	.LC281
	.word	.LC282
	.word	.LC283
	.word	.LC284
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
	ldr	r4, .L3740
	ldr	r3, [r4, #3380]
	ldr	r7, [r4, #580]
	cmp	r3, #0
	bne	.L3713
	mov	r0, #1
	bl	buf_alloc
	str	r0, [r4, #3380]
.L3713:
	ldr	r5, [r4, #3380]
	cmp	r5, #0
	bne	.L3714
	ldr	r1, .L3740+4
	movw	r2, #494
	ldr	r0, .L3740+8
	bl	printk
	bl	dump_stack
.L3714:
	ldr	r6, .L3740+12
	add	fp, r7, #96
	ldr	r2, .L3740
	mov	r10, #0
.L3715:
	ldrb	r3, [r7, #89]	@ zero_extendqisi2
	ldr	r1, .L3740+12
	cmp	r10, r3
	bge	.L3738
	ldr	ip, .L3740+16
	mov	r8, #1
.L3724:
	ldrh	r3, [r6, #16]
	cmp	r8, r3
	bgt	.L3739
	ldrh	r3, [ip]
	ldrh	r1, [fp]
	ldrb	r0, [r2, #3768]	@ zero_extendqisi2
	mul	r1, r3, r1
	ldrh	r3, [r6, #12]
	cmp	r0, #3
	addeq	r3, r1, r3
	orreq	r3, r3, r8, asl #24
	beq	.L3737
	cmp	r0, #2
	addne	r1, r1, r3
	strne	r1, [r5, #24]
	bne	.L3717
	sub	r3, r3, #1
	add	r3, r3, r1
	add	r3, r3, r8
	orr	r3, r3, #33554432
.L3737:
	str	r3, [r5, #24]
.L3717:
	mov	r0, r5
	mov	r1, #1
	str	r2, [sp, #28]
	str	ip, [sp, #24]
	bl	sblk_read_page
	ldr	r3, [r5, #36]
	cmn	r3, #1
	ldr	ip, [sp, #24]
	ldr	r2, [sp, #28]
	beq	.L3719
	ldr	r0, [r4, #3724]
	ldrh	r1, [r6, #18]
	ldr	r3, [r5, #12]
	ldr	lr, [r0, r1, asl #2]
	ldr	r0, [r3, #4]
	cmp	lr, r0
	bne	.L3719
	ldr	r0, [r2, #3728]
	ldr	r3, [r3, #8]
	ldr	r1, [r0, r1, asl #2]
	cmp	r1, r3
	beq	.L3720
.L3719:
	ldr	r1, [r4, #3724]
	ldrh	r3, [r6, #18]
	ldr	r9, .L3740
	ldr	r3, [r1, r3, asl #2]
	cmn	r3, #1
	beq	.L3720
	movw	r6, #3722
	ldrb	r3, [r7, #89]	@ zero_extendqisi2
	ldrh	ip, [r9, r6]
	mov	r2, #4
	ldr	r0, .L3740+20
	mul	r3, ip, r3
	bl	rknand_print_hex
	ldrb	r3, [r7, #89]	@ zero_extendqisi2
	ldrh	ip, [r9, r6]
	mov	r2, #4
	ldr	r0, .L3740+24
	ldr	r1, [r9, #3728]
	mul	r3, ip, r3
	bl	rknand_print_hex
	ldr	r3, .L3740+28
	ldr	r3, [r3]
	tst	r3, #1024
	beq	.L3721
	ldr	r3, .L3740+12
	ldr	r2, [r9, #3728]
	ldr	lr, [r9, #3724]
	ldrh	ip, [r3, #18]
	ldr	r3, [r5, #12]
	ldr	r0, .L3740+32
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
.L3721:
	ldr	r3, [r4, #580]
	mov	r1, #0
	ldrh	r0, [r3, #80]
	bl	ftl_sblk_dump
	ldrh	r3, [r7, #80]
	ldr	r2, [r4, #576]
	mov	r1, #0
	mvn	r0, #0
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	b	.L3722
.L3720:
	ldrh	r3, [r6, #18]
	add	r8, r8, #1
	add	r3, r3, #1
	strh	r3, [r6, #18]	@ movhi
	b	.L3724
.L3739:
	add	r10, r10, #1
	add	fp, fp, #2
	b	.L3715
.L3738:
	ldrb	r0, [r4, #3769]	@ zero_extendqisi2
	ldrh	r3, [r1, #12]
	cmp	r0, #0
	add	r2, r3, #1
	addne	r3, r3, #2
	movne	r0, #0
	strh	r2, [r1, #12]	@ movhi
	strneh	r3, [r1, #12]	@ movhi
.L3722:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3741:
	.align	2
.L3740:
	.word	.LANCHOR0
	.word	.LANCHOR1+2080
	.word	.LC0
	.word	.LANCHOR0+3372
	.word	.LANCHOR0+3806
	.word	.LC285
	.word	.LC286
	.word	.LANCHOR2
	.word	.LC287
	.fnend
	.size	gc_check_data_one_wl, .-gc_check_data_one_wl
	.align	2
	.global	ftl_update_l2p_map
	.type	ftl_update_l2p_map, %function
ftl_update_l2p_map:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, .L3773
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #20
	sub	sp, sp, #20
	ldrh	r3, [r3]
	mov	r6, r0
	ldrb	r7, [r0, #9]	@ zero_extendqisi2
	ldrh	r4, [r0, #12]
	mul	r7, r7, r3
	ldr	r3, .L3773+4
	ldr	r5, [r3, #-3860]
	sub	r2, r7, #-1073741823
	str	r3, [sp]
	add	r4, r5, r4, asl #2
	ldr	r2, [r4, r2, asl #2]
	cmn	r2, #1
	beq	.L3743
	ldr	r1, .L3773+8
	movw	r2, #1640
	ldr	r0, .L3773+12
	bl	printk
	bl	dump_stack
.L3743:
	mov	r5, #0
	sub	r4, r4, #4
	mov	r9, r5
.L3744:
	cmp	r9, r7
	beq	.L3772
	ldr	r8, [r4, #4]!
	cmn	r8, #1
	beq	.L3745
	ldr	r3, .L3773+16
	mov	r0, r8
	ldrb	r1, [r3, #3312]	@ zero_extendqisi2
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	ldr	r3, .L3773+20
	ldr	r3, [r3]
	tst	r3, #4096
	uxth	fp, r0
	beq	.L3746
	ldr	r0, .L3773+24
	mov	r1, fp
	mov	r2, r8
	mov	r3, r9
	bl	printk
.L3746:
	mov	r8, r4
	mov	r10, r9
.L3749:
	ldr	r0, [r8]
	cmn	r0, #1
	beq	.L3747
	ldr	r3, .L3773+16
	ldrb	r1, [r3, #3312]	@ zero_extendqisi2
	mov	r1, r1, asl #7
	bl	__aeabi_uidiv
	uxth	r0, r0
	cmp	r0, fp
	bne	.L3747
	ldrb	r3, [r6, #9]	@ zero_extendqisi2
	mov	r0, r10
	mov	r1, r3
	str	r3, [sp, #4]
	bl	__aeabi_idiv
	ldr	r2, .L3773+28
	ldrh	r2, [r2]
	ldr	r3, [sp, #4]
	mls	r3, r3, r0, r10
	add	r3, r6, r3, asl #1
	ldrh	r3, [r3, #16]
	mla	r2, r2, r3, r0
	ldr	r0, .L3773+32
	str	r2, [sp, #12]
	ldr	r1, [r8]
	bl	pm_ppa_update_check
	cmp	r0, #0
	beq	.L3748
	ldr	r3, [sp]
	mov	r2, #4
	ldr	r0, .L3773+36
	ldr	r1, [r3, #-3860]
	mov	r3, r7
	bl	rknand_print_hex
.L3748:
	add	r5, r5, #1
	ldr	r0, [r8]
	add	r1, sp, #12
	mov	r2, #1
	uxth	r5, r5
	bl	pm_log2phys
	mvn	r3, #0
	str	r3, [r8]
.L3747:
	add	r10, r10, #1
	add	r8, r8, #4
	cmp	r10, r7
	bne	.L3749
.L3745:
	add	r9, r9, #1
	b	.L3744
.L3772:
	ldr	r3, .L3773+20
	ldr	r4, .L3773+16
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L3751
	ldrh	r1, [r6]
	mov	r2, r5
	ldr	ip, [r4, #576]
	ldr	r0, .L3773+40
	mov	r3, r1, asl #1
	ldrh	r3, [ip, r3]
	bl	printk
.L3751:
	ldrh	r3, [r6]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	strh	r5, [r2, r3]	@ movhi
	add	sp, sp, #20
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3774:
	.align	2
.L3773:
	.word	.LANCHOR0+3712
	.word	.LANCHOR3
	.word	.LANCHOR1+2104
	.word	.LC0
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LC288
	.word	.LANCHOR0+3806
	.word	.LC289
	.word	.LC290
	.word	.LC291
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
	beq	.L3776
	bl	zftl_insert_data_list
.L3776:
	ldr	r4, .L3784
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
	beq	.L3778
	mov	r1, r5
	ldr	r0, .L3784+4
	bl	printk
	ldr	r3, [r4, #580]
	ldr	r0, [r3, #564]
	bl	gc_mark_bad_ppa
	ldr	r3, [r4, #580]
	mvn	r2, #0
	str	r6, [r3, #560]
	str	r2, [r3, #564]
.L3778:
	bl	ftl_ext_info_flush
	mov	r0, #0
	bl	ftl_info_flush
	bl	lpa_rebuild_hash
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, pc}
.L3785:
	.align	2
.L3784:
	.word	.LANCHOR0
	.word	.LC292
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
	ldr	r6, .L3849
	mov	r8, r6
.L3787:
	ldrb	r2, [r6, #3336]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3789
	ldrb	r3, [r6, #3360]	@ zero_extendqisi2
	sub	r2, r2, #1
	strb	r2, [r6, #3336]
	str	r3, [sp, #16]
	mov	r3, #48
	ldr	r1, [sp, #16]
	mul	r3, r3, r1
	ldr	r1, .L3849+4
	add	r1, r1, r3
	add	r3, r6, r3
	str	r1, [sp, #20]
	ldr	r2, [r3, #1796]
	ldrb	r1, [r3, #1776]	@ zero_extendqisi2
	ldr	r3, [r6, #3320]
	cmp	r2, r3
	strb	r1, [r6, #3360]
	bcc	.L3790
	ldr	r1, .L3849+8
	movw	r2, #502
	ldr	r0, .L3849+12
	bl	printk
	bl	dump_stack
.L3790:
	ldr	r2, [sp, #16]
	mov	r3, #48
	ldr	r1, [r8, #3320]
	mla	r2, r3, r2, r8
	ldr	r9, [r2, #1796]
	cmp	r9, r1
	bcc	.L3791
	ldr	r0, [sp, #20]
	bl	zbuf_free
	mvn	r0, #0
	b	.L3845
.L3791:
	ldr	r1, [r2, #1788]
	ldr	fp, [r2, #1780]
	ldrb	r7, [r2, #1817]	@ zero_extendqisi2
	ldrb	r2, [r2, #1816]	@ zero_extendqisi2
	str	r1, [sp, #8]
	str	r2, [sp, #12]
	ldrb	r2, [r6, #3337]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L3793
	ldr	r2, .L3849+16
	ldrb	r4, [r2, #-3519]	@ zero_extendqisi2
	ldr	r2, .L3849+4
	mla	r4, r3, r4, r2
.L3794:
	ldrb	r2, [r4]	@ zero_extendqisi2
	cmp	r2, #255
	ldrne	r1, .L3849+4
	mlane	r4, r3, r2, r1
	bne	.L3794
.L3847:
	ldr	r3, [r4, #20]
	cmp	r3, r9
	bne	.L3793
	ldr	r3, [sp, #12]
	mov	r1, r7, asl #9
	ldr	r0, [r4, #4]
	mov	r2, r3, asl #9
	add	r0, r0, r1
	add	r1, fp, r1
	bl	ftl_memcpy
	ldr	r0, [sp, #20]
	bl	zbuf_free
	b	.L3787
.L3793:
	mov	r0, r9
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #28]
	bne	.L3797
	mov	r0, r9
	add	r1, sp, #28
	mov	r2, #0
	bl	pm_log2phys
.L3797:
	ldr	r4, [r8, #580]
	mov	r3, #0
	ldr	r2, .L3849+4
	add	r4, r4, #16
.L3800:
	ldr	r1, [r2, #20]
	cmp	r1, r9
	bne	.L3798
	ldrb	r1, [r2, #2]	@ zero_extendqisi2
	tst	r1, #8
	beq	.L3798
	mov	r10, #48
	bic	r1, r1, #8
	mla	r10, r10, r3, r8
	ldr	r3, [r10, #1800]
	strb	r1, [r10, #1778]
	ldr	r5, [r10, #1780]
	str	r3, [sp, #28]
	b	.L3799
.L3798:
	add	r3, r3, #1
	add	r2, r2, #48
	cmp	r3, #32
	bne	.L3800
	mov	r5, #0
.L3799:
	ldrb	r3, [r8, #3312]	@ zero_extendqisi2
	ldr	r2, [sp, #12]
	cmp	r2, r3
	bcs	.L3824
	cmp	r5, #0
	beq	.L3802
	cmp	r7, #0
	beq	.L3803
	mov	r0, fp
	mov	r1, r5
	mov	r2, r7, asl #9
	bl	ftl_memcpy
	ldr	r4, [r6, #580]
	add	r4, r4, #48
.L3803:
	ldr	r3, [sp, #12]
	ldrb	r2, [r8, #3312]	@ zero_extendqisi2
	add	r7, r7, r3
	cmp	r7, r2
	ldrcs	r4, [r6, #580]
	addcs	r4, r4, #16
	bcs	.L3824
	rsb	r2, r7, r2
	mov	r1, r7, asl #9
	add	r0, fp, r1
	add	r1, r5, r1
	mov	r2, r2, asl #9
	bl	ftl_memcpy
	b	.L3824
.L3802:
	ldr	r3, [sp, #28]
	cmn	r3, #1
	beq	.L3805
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
	bne	.L3806
	ldr	r3, [r5, #36]
	cmn	r3, #1
	bne	.L3807
.L3806:
	ldr	r2, [r8, #580]
	ldr	r0, .L3849+20
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
	ldr	r0, .L3849+24
	mov	r3, r2
	ldr	r1, [r5, #12]
	bl	rknand_print_hex
.L3807:
	ldr	r3, [r5, #12]
	ldr	r3, [r3, #4]
	cmp	r3, r9
	bne	.L3808
	ldr	r3, [r5, #36]
	cmn	r3, #1
	bne	.L3809
.L3808:
	ldr	r1, .L3849+8
	movw	r2, #573
	ldr	r0, .L3849+12
	bl	printk
	bl	dump_stack
.L3809:
	cmp	r7, #0
	beq	.L3810
	ldr	r3, [sp, #28]
	mov	r2, r7, asl #9
	cmn	r3, #1
	beq	.L3811
	mov	r0, fp
	ldr	r1, [r5, #4]
	bl	ftl_memcpy
	b	.L3812
.L3805:
	cmp	r7, #0
	moveq	r5, r7
	moveq	r10, r7
	beq	.L3810
	mov	r2, r7, asl #9
	mov	r10, r5
.L3811:
	mov	r0, fp
	mov	r1, #0
	bl	ftl_memset
.L3812:
	ldr	r4, [r6, #580]
	add	r4, r4, #48
.L3810:
	ldr	r3, [sp, #12]
	ldrb	r2, [r6, #3312]	@ zero_extendqisi2
	add	r7, r7, r3
	cmp	r7, r2
	bcc	.L3813
	ldrhi	r4, [r6, #580]
	addhi	r4, r4, #16
	b	.L3814
.L3813:
	ldr	r3, [sp, #28]
	rsb	r2, r7, r2
	mov	r7, r7, asl #9
	cmn	r3, #1
	mov	r2, r2, asl #9
	beq	.L3815
	ldr	r1, [r5, #4]
	add	r0, fp, r7
	add	r1, r1, r7
	bl	ftl_memcpy
	b	.L3814
.L3815:
	add	r0, fp, r7
	mov	r1, #0
	bl	ftl_memset
.L3814:
	cmp	r5, #0
	beq	.L3801
	ldrb	r3, [r5, #2]	@ zero_extendqisi2
	mov	r1, r5
	ldr	r0, .L3849+28
	bic	r3, r3, #8
	strb	r3, [r5, #2]
	bl	buf_remove_buf
	mov	r0, r5
	bl	zbuf_free
	b	.L3801
.L3824:
	mov	r10, #0
.L3801:
	ldrh	r3, [r4, #6]
	cmp	r3, #0
	bne	.L3816
	bl	ftl_flush
	mov	r0, r4
	bl	ftl_alloc_new_data_sblk
.L3816:
	mov	r0, r4
	ldr	r5, .L3849+16
	bl	ftl_get_new_free_page
	mov	r3, #48
	ldr	r2, [sp, #16]
	ldr	ip, [sp, #8]
	mul	r3, r3, r2
	add	r2, r8, r3
	ldr	r1, [r2, #1792]
	str	r9, [ip, #4]
	str	r10, [ip, #12]
	str	r1, [ip]
	ldr	r1, [sp, #28]
	str	r1, [ip, #8]
	str	r1, [r2, #1804]
	mvn	r1, #0
	strb	r1, [r2, #1776]
	ldrb	r1, [r2, #1778]	@ zero_extendqisi2
	str	r0, [r2, #1800]
	orr	r1, r1, #10
	strb	r1, [r2, #1778]
	ldr	r2, .L3849+4
	ldrh	r10, [r4, #10]
	ldrh	r9, [r4, #12]
	add	r3, r2, r3
	ldr	r0, .L3849+32
	add	r9, r10, r9
	ldr	r1, [sp, #20]
	sub	r9, r9, #1
	strh	r9, [r3, #32]	@ movhi
	bl	buf_add_tail
	ldrb	r3, [r8, #3337]	@ zero_extendqisi2
	add	r3, r3, #1
	strb	r3, [r8, #3337]
	bl	timer_get_time
	ldrb	r3, [r8, #3337]	@ zero_extendqisi2
	ldrh	r2, [r4, #6]
	cmp	r3, #2
	str	r0, [r5, #-3516]
	bhi	.L3817
	cmp	r2, #1
	bne	.L3789
.L3817:
	ldrb	r1, [r4, #5]	@ zero_extendqisi2
	mov	lr, #48
	ldrb	r0, [r5, #-3519]	@ zero_extendqisi2
	cmp	r1, #0
	mov	r1, #0
	moveq	ip, #1
	movne	ip, #2
	cmp	r2, #1
	mov	r2, r0
	moveq	ip, r3
.L3821:
	cmp	r1, ip
	mlane	r2, lr, r2, r6
	addne	r1, r1, #1
	ldrneb	r2, [r2, #1776]	@ zero_extendqisi2
	bne	.L3821
.L3848:
	strb	r2, [r5, #-3519]
	uxtb	r1, r1
	ldr	r2, .L3849+4
	rsb	r3, r1, r3
	strb	r3, [r8, #3337]
	mov	r3, #48
	mla	r0, r3, r0, r2
	bl	sblk_prog_page
	ldrh	r3, [r4, #6]
	cmp	r3, #1
	bne	.L3789
	bl	sblk_wait_write_queue_completed
	bl	ftl_write_completed
	mov	r0, r4
	bl	ftl_write_last_log_page
	mov	r0, r4
	bl	ftl_alloc_new_data_sblk
.L3789:
	ldrb	r4, [r6, #3336]	@ zero_extendqisi2
	cmp	r4, #0
	bne	.L3787
	bl	ftl_write_completed
	mov	r0, r4
.L3845:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3850:
	.align	2
.L3849:
	.word	.LANCHOR0
	.word	.LANCHOR0+1776
	.word	.LANCHOR1+2124
	.word	.LC0
	.word	.LANCHOR3
	.word	.LC293
	.word	.LC224
	.word	.LANCHOR3-3839
	.word	.LANCHOR3-3519
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
	.pad #76
	sub	sp, sp, #76
	ldr	r7, .L3948
	ldrb	r0, [r7, #3361]	@ zero_extendqisi2
	ldr	r5, [r7, #580]
	cmp	r0, #0
	bne	.L3852
	bl	buf_alloc
	subs	r4, r0, #0
	beq	.L3851
	ldr	r6, .L3948+4
	ldrh	r5, [r6, #22]
	mov	r0, r5
	add	r5, r5, #1
	bl	gc_get_src_ppa_from_index
	mov	r1, #1
	strh	r5, [r6, #22]	@ movhi
	str	r0, [r4, #24]
	mov	r9, r0
	mov	r0, r4
	bl	sblk_read_page
	ldr	r3, [r4, #36]
	cmn	r3, #1
	cmpne	r3, #512
	bne	.L3854
	ldr	r1, .L3948+8
	movw	r2, #731
	ldr	r0, .L3948+12
	bl	printk
	bl	dump_stack
.L3854:
	ldr	r3, [r4, #12]
	ldr	r5, [r3, #4]
	mov	r0, r5
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #48]
	bne	.L3855
	mov	r0, r5
	add	r1, sp, #48
	mov	r2, #0
	bl	pm_log2phys
.L3855:
	ldr	r8, [sp, #48]
	cmp	r8, r9
	bne	.L3856
	ldr	r1, .L3948+16
	mov	r3, #0
.L3859:
	add	r2, r1, r3
	ldr	r0, [r2, #20]
	cmp	r0, r5
	bne	.L3857
	ldrb	r2, [r2, #2]	@ zero_extendqisi2
	tst	r2, #2
	beq	.L3857
	mov	r0, r4
	bl	zbuf_free
	ldr	r3, .L3948+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3851
	mov	r1, r5
	mov	r2, r8
	ldrh	r3, [r6, #22]
	ldr	r0, .L3948+24
	bl	printk
	b	.L3851
.L3857:
	add	r3, r3, #48
	cmp	r3, #1536
	bne	.L3859
	mov	r3, #0
	strb	r3, [r4, #41]
	ldrb	r3, [r7, #3312]	@ zero_extendqisi2
	str	r5, [r4, #20]
	strb	r3, [r4, #40]
	ldrb	r3, [r4, #2]	@ zero_extendqisi2
	orr	r3, r3, #10
	strb	r3, [r4, #2]
	ldr	r3, [r4, #12]
	ldr	r3, [r3]
	str	r3, [r4, #16]
	ldr	r3, .L3948+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3860
	mov	r0, r5
	ldr	r9, [sp, #48]
	bl	lpa_hash_get_ppa
	mov	r1, r5
	str	r8, [sp]
	mov	r3, r0
	ldrh	r2, [r6, #22]
	ldr	r0, .L3948+28
	str	r2, [sp, #4]
	mov	r2, r9
	bl	printk
.L3860:
	mov	r0, r4
	bl	ftl_gc_write_buf
	bl	ftl_write_commit
	ldr	r2, [r7, #3340]
	ldr	r3, [r2, #60]
	add	r3, r3, #1
	str	r3, [r2, #60]
	ldrh	r3, [r6, #24]
	add	r3, r3, #1
	strh	r3, [r6, #24]	@ movhi
	b	.L3851
.L3856:
	ldr	r3, .L3948+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3862
	mov	r0, r5
	bl	lpa_hash_get_ppa
	mov	r1, r5
	str	r9, [sp]
	mov	r3, r0
	ldrh	r2, [r6, #22]
	ldr	r0, .L3948+28
	str	r2, [sp, #4]
	mov	r2, r8
	bl	printk
.L3862:
	mov	r0, r4
	bl	zbuf_free
	b	.L3851
.L3852:
	ldrb	fp, [r7, #3768]	@ zero_extendqisi2
	ldr	r8, .L3948+4
	cmp	fp, #3
	ldrb	r10, [r5, #89]	@ zero_extendqisi2
	bne	.L3863
	ldrb	r3, [r7, #3771]	@ zero_extendqisi2
	cmp	r3, #0
	movw	r3, #310
	ldrh	r4, [r8, r3]
	mov	r0, r4
	beq	.L3864
	add	r1, r10, r10, asl #1
	sub	r10, r10, r10, asl #2
	bl	__aeabi_idiv
	uxth	r6, r0
	str	fp, [sp, #28]
	smulbb	r2, r10, r6
	add	r3, r4, r2
	uxth	r3, r3
	str	r3, [sp, #24]
	b	.L3866
.L3864:
	mov	r1, r10
	bl	__aeabi_idiv
	ldr	r2, .L3948+32
	mov	r3, r0, asl #1
	str	fp, [sp, #28]
	smulbb	r0, r0, r10
	ldrh	r9, [r2, r3]
	rsb	r0, r0, r4
	mov	r6, r9, lsr #3
	and	r9, r9, #7
	uxth	r3, r0
	cmp	r9, #1
	str	r3, [sp, #24]
	mla	r10, r10, r6, r3
	add	r10, r10, r10, asl #1
	uxth	r4, r10
	bne	.L3867
	b	.L3866
.L3863:
	ldrb	r3, [r7, #3769]	@ zero_extendqisi2
	mov	r1, r10
	cmp	r3, #0
	movw	r3, #310
	ldrh	r4, [r8, r3]
	mov	r0, r4
	bne	.L3868
	bl	__aeabi_idiv
	uxth	r6, r0
	smulbb	r0, r6, r10
	rsb	r0, r0, r4
	uxth	r3, r0
	str	r3, [sp, #24]
	mov	r3, #1
	b	.L3943
.L3868:
	bl	__aeabi_idiv
	uxth	r6, r0
	smulbb	r0, r6, r10
	rsb	r0, r0, r4
	uxth	r3, r0
	str	r3, [sp, #24]
	mov	r3, #2
.L3943:
	str	r3, [sp, #28]
.L3866:
	ldr	r2, .L3948
	mov	fp, #0
	ldr	r1, .L3948+36
	ldr	ip, .L3948+4
.L3869:
	ldrb	r3, [r5, #89]	@ zero_extendqisi2
	uxth	r9, fp
	ldr	r0, [sp, #28]
	mul	r0, r3, r0
	cmp	r9, r0
	bge	.L3901
	ldrh	r0, [r1]
	add	r9, r4, r9
	mul	r3, r3, r0
	sub	r3, r3, #1
	cmp	r9, r3
	beq	.L3901
	ldr	r3, [r2, #3728]
	ldr	r0, [r3, r9, asl #2]
	cmn	r0, #1
	bne	.L3870
	ldrh	r10, [r8, #22]
	ldrh	r0, [r8, #20]
	str	ip, [sp, #44]
	cmp	r0, r10
	str	r1, [sp, #40]
	str	r2, [sp, #36]
	str	r3, [sp, #32]
	bls	.L3851
	mov	r0, r10
	add	r10, r10, #1
	bl	gc_get_src_ppa_from_index
	ldr	ip, [sp, #44]
	ldr	r3, [sp, #32]
	ldr	r1, [sp, #40]
	ldr	r2, [sp, #36]
	strh	r10, [ip, #22]	@ movhi
	str	r0, [r3, r9, asl #2]
.L3870:
	add	fp, fp, #1
	b	.L3869
.L3901:
	mov	r9, #1
.L3867:
	ldr	r3, .L3948+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3872
	movw	r3, #310
	ldr	r0, .L3948+40
	ldrh	r3, [r8, r3]
	mov	r1, r6
	mov	r2, r9
	str	r3, [sp]
	ldr	r3, [r7, #3728]
	ldr	r3, [r3, r4, asl #2]
	stmib	sp, {r3, r4}
	ldr	r3, [sp, #24]
	bl	printk
.L3872:
	ldr	r8, .L3948
	mov	r3, #0
	str	r3, [sp, #32]
.L3873:
	ldrb	r1, [r5, #89]	@ zero_extendqisi2
	ldr	r3, [sp, #28]
	ldrh	fp, [sp, #32]
	ldr	r7, .L3948
	mul	r3, r1, r3
	cmp	fp, r3
	bge	.L3946
	ldr	r3, [r8, #3732]
	add	r10, r4, fp
	ldrb	r3, [r3, r10]	@ zero_extendqisi2
	cmp	r3, #255
	bne	.L3874
	mov	r0, #0
	bl	buf_alloc
	subs	r7, r0, #0
	bne	.L3875
	bl	sblk_wait_write_queue_completed
	bl	ftl_write_completed
	bl	gc_write_completed
	bl	gc_free_temp_buf
	mov	r0, r7
	bl	buf_alloc
	subs	r7, r0, #0
	beq	.L3851
.L3875:
	ldrb	r2, [r7, #1]	@ zero_extendqisi2
	ldr	r3, [r8, #3732]
	strb	r2, [r3, r10]
	ldrb	r3, [r8, #3379]	@ zero_extendqisi2
	strb	r9, [r7, #45]
	add	r3, r3, #1
	strb	r3, [r8, #3379]
	add	r3, fp, r4
	strh	r3, [r7, #32]	@ movhi
	ldr	r3, .L3948+36
	ldrb	r2, [r5, #89]	@ zero_extendqisi2
	ldrh	r1, [r3]
	mul	r2, r2, r1
	sub	r2, r2, #1
	cmp	r10, r2
	bne	.L3876
	ldr	r2, [r8, #3724]
	mvn	r1, #0
	str	r3, [sp, #36]
	str	r1, [r2, r10, asl #2]
	ldrh	r1, [r3]
	ldrb	r2, [r8, #3720]	@ zero_extendqisi2
	ldr	r0, [r7, #4]
	mul	r2, r2, r1
	ldr	r1, [r8, #3724]
	mov	r2, r2, asl #2
	bl	ftl_memcpy
	ldrb	r2, [r8, #3720]	@ zero_extendqisi2
	ldr	r0, [r7, #4]
	ldr	r3, [sp, #36]
	ldrh	r1, [r3]
	str	r3, [sp, #40]
	mul	r2, r2, r1
	ldr	r1, [r8, #3728]
	mov	r2, r2, asl #2
	add	r0, r0, r2
	bl	ftl_memcpy
	ldrb	r2, [r8, #3312]	@ zero_extendqisi2
	mov	r1, #0
	ldr	r0, [r7, #12]
	mov	r2, r2, asl #1
	bl	ftl_memset
	ldr	r2, [r7, #12]
	ldr	r1, .L3948+44
	str	r1, [r2]
	ldr	r2, [r7, #12]
	ldr	r0, [r7, #4]
	ldr	r3, [sp, #40]
	str	r2, [sp, #36]
	ldrh	r1, [r3]
	ldrb	r3, [r8, #3720]	@ zero_extendqisi2
	mul	r1, r3, r1
	mov	r1, r1, asl #2
	bl	js_hash
	ldr	r2, [sp, #36]
	str	r0, [r2, #4]
	ldr	r2, [r8, #580]
	ldr	r3, [r7, #12]
	ldr	r2, [r2, #132]
	b	.L3944
.L3876:
	ldr	r2, [r8, #3728]
	mov	r0, r7
	mov	r1, #1
	str	r3, [sp, #36]
	ldr	r2, [r2, r10, asl #2]
	str	r2, [r7, #24]
	bl	sblk_read_page
	ldr	r2, [r7, #36]
	cmn	r2, #1
	cmpne	r2, #512
	ldr	r3, [sp, #36]
	bne	.L3878
	ldrh	r2, [r3, #42]
	mvn	r0, #0
	ldr	r3, [r7, #24]
	rsb	r1, r2, #21
	mov	r3, r3, lsr r2
	bic	r0, r3, r0, asl r1
	ldrb	r1, [r8, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	mov	r1, #0
	uxth	r0, r0
	bl	ftl_sblk_dump
	ldr	r3, [r7, #36]
	cmn	r3, #1
	cmpne	r3, #512
	ldreq	r3, [r7, #12]
	mvneq	r2, #0
	streq	r2, [r3, #4]
.L3878:
	ldr	r3, [r7, #36]
	cmn	r3, #1
	cmpne	r3, #512
	bne	.L3880
	ldr	r1, .L3948+8
	movw	r2, #846
	ldr	r0, .L3948+12
	bl	printk
	bl	dump_stack
.L3880:
	ldr	r3, [r7, #12]
	ldr	r2, [r8, #3320]
	ldr	r1, [r3, #4]
	cmp	r1, r2
	mvncs	r2, #0
	strcs	r2, [r3, #4]
	ldr	r3, [r7, #12]
	ldr	r2, [r3, #4]
	ldr	r3, .L3948
	ldr	r3, [r3, #3724]
	str	r2, [r3, r10, asl #2]
	ldr	r3, [r7, #12]
	ldr	r2, [r7, #24]
.L3944:
	str	r2, [r3, #8]
.L3874:
	ldr	r3, [r8, #3732]
	mov	r2, #48
	ldr	ip, .L3948+16
	ldrb	r0, [r3, r10]	@ zero_extendqisi2
	add	r3, sp, #72
	add	r3, r3, fp, asl #2
	mul	r2, r2, r0
	add	ip, ip, r2
	str	ip, [r3, #-24]
	mov	r3, #2
	add	r2, r8, r2
	strh	r3, [ip, #34]	@ movhi
	ldrb	r3, [r8, #3768]	@ zero_extendqisi2
	strb	r9, [r2, #1821]
	cmp	r3, #3
	bne	.L3882
	ldr	r3, .L3948+48
	ldrh	ip, [ip, #32]
	umull	r0, r1, fp, r3
	mov	r3, r1, lsr #1
	ldr	r1, [sp, #24]
	uxtah	r1, r1, r3
	add	r3, r3, r3, asl #1
	rsb	r3, r3, fp
	add	r1, r5, r1, asl #1
	mov	r3, r3, asl #24
	ldrh	r0, [r1, #96]
	add	r3, r3, #16777216
	ldr	r1, .L3948+52
	ldrh	r1, [r1]
	mul	r1, r1, r0
	orr	r1, r1, r6
	str	r1, [r2, #1800]
	ldr	r2, [r8, #3740]
	orr	r1, r1, r3
	str	r1, [r2, ip, asl #2]
	b	.L3883
.L3882:
	cmp	r3, #2
	bne	.L3884
	ldrb	r3, [r8, #3769]	@ zero_extendqisi2
	cmp	r3, #0
	ldr	r3, [sp, #24]
	bne	.L3885
	add	r3, fp, r3
	ldr	r1, .L3948+52
	add	r3, r5, r3, asl #1
	ldrh	r1, [r1]
	ldrh	r3, [r3, #96]
	mla	r3, r1, r3, r6
	b	.L3945
.L3885:
	add	r3, r3, fp, lsr #1
	ldr	r1, .L3948+52
	add	r3, r5, r3, asl #1
	ldrh	r1, [r1]
	ldrh	r3, [r3, #96]
	mla	r3, r1, r3, r6
	and	r1, fp, #1
	add	r3, r3, r1
.L3945:
	orr	r3, r3, #33554432
	str	r3, [r2, #1800]
.L3884:
	mov	r3, #48
	mul	r0, r3, r0
	ldr	r3, .L3948
	add	r3, r3, r0
	ldr	r2, [r3, #1800]
	ldr	r3, .L3948+16
	add	r0, r3, r0
	sub	r3, r3, #1776
	ldrh	r1, [r0, #32]
	ldr	r3, [r3, #3740]
	str	r2, [r3, r1, asl #2]
.L3883:
	ldr	r3, .L3948+20
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3886
	ldr	r1, .L3948
	add	r3, sp, #72
	add	r3, r3, fp, asl #2
	ldr	r1, [r1, #3732]
	ldr	r2, [r3, #-24]
	ldr	r3, .L3948
	ldrb	r1, [r1, r10]	@ zero_extendqisi2
	ldrb	r3, [r3, #3378]	@ zero_extendqisi2
	str	r1, [sp]
	ldr	r1, [r2, #24]
	str	r1, [sp, #4]
	ldrh	r1, [r2, #32]
	str	r1, [sp, #8]
	ldrb	r1, [r2, #1]	@ zero_extendqisi2
	str	r1, [sp, #12]
	ldr	r1, .L3948+48
	umull	r0, r1, fp, r1
	ldr	r0, [sp, #24]
	ubfx	r1, r1, #1, #16
	add	r1, r0, r1
	str	r1, [sp, #16]
	ldr	r2, [r2, #12]
	mov	r1, fp
	ldr	r0, .L3948+56
	ldr	r2, [r2, #4]
	str	r2, [sp, #20]
	mov	r2, r9
	bl	printk
.L3886:
	ldr	r3, [sp, #32]
	add	r3, r3, #1
	str	r3, [sp, #32]
	b	.L3873
.L3946:
	ldrb	r3, [r7, #3768]	@ zero_extendqisi2
	cmp	r3, #3
	bne	.L3888
	ldrb	r2, [r7, #3771]	@ zero_extendqisi2
	ldr	r3, [sp, #48]
	cmp	r2, #0
	movne	r2, #0
	bne	.L3892
	cmp	r9, #1
	moveq	r2, #9
	beq	.L3892
	cmp	r9, #2
	moveq	r2, #13
.L3892:
	strb	r2, [r3, #44]
	add	r1, sp, #60
	ldrb	r2, [r5, #89]	@ zero_extendqisi2
	add	r0, sp, #48
	bl	sblk_xlc_prog_pages
	b	.L3893
.L3888:
	ldrb	r3, [r7, #3769]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3941
	ldrb	r3, [r7, #3770]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L3941
	add	r0, sp, #48
	bl	sblk_3d_mlc_prog_pages
	b	.L3893
.L3941:
	ldrb	r2, [r5, #89]	@ zero_extendqisi2
	uxth	r1, r3
	ldr	r0, [sp, #28]
	add	r3, r3, #1
	mul	r2, r0, r2
	sub	r2, r2, #1
	cmp	r1, r2
	bge	.L3947
	add	r2, sp, #72
	add	r0, sp, #72
	add	r2, r2, r1, asl #2
	add	r1, r0, r1, asl #2
	ldr	r1, [r1, #-20]
	ldr	r2, [r2, #-24]
	ldrb	r1, [r1, #1]	@ zero_extendqisi2
	strb	r1, [r2]
	b	.L3941
.L3947:
	add	r3, sp, #72
	add	r2, r3, r2, asl #2
	ldr	r3, [r2, #-24]
	mvn	r2, #0
	strb	r2, [r3]
	ldrh	r3, [sp, #28]
	ldrb	r1, [r5, #89]	@ zero_extendqisi2
	ldr	r0, [sp, #48]
	smulbb	r1, r3, r1
	uxtb	r1, r1
	bl	sblk_prog_page
.L3893:
	ldrb	r2, [r7, #3771]	@ zero_extendqisi2
	ldrb	r3, [r5, #89]	@ zero_extendqisi2
	cmp	r2, #0
	addne	r3, r3, r3, asl #1
	bne	.L3897
	ldr	r2, .L3948
	ldrb	r2, [r2, #3769]	@ zero_extendqisi2
	cmp	r2, #0
	movne	r3, r3, asl #1
	uxthne	r3, r3
.L3897:
	ldr	r1, [r7, #3340]
	ldr	r2, [r1, #52]
	add	r2, r2, r3
	str	r2, [r1, #52]
	ldr	r1, .L3948+4
	movw	r2, #310
	ldrh	r0, [r1, r2]
	add	r3, r3, r0
	uxth	r3, r3
	strh	r3, [r1, r2]	@ movhi
	movw	r2, #3722
	ldrh	r1, [r7, r2]
	ldrb	r2, [r5, #89]	@ zero_extendqisi2
	mul	r2, r2, r1
	cmp	r3, r2
	ldrge	r3, .L3948
	movge	r2, #0
	ldrge	r3, [r3, #580]
	strgeh	r2, [r3, #86]	@ movhi
	bl	gc_write_completed
.L3851:
	add	sp, sp, #76
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L3949:
	.align	2
.L3948:
	.word	.LANCHOR0
	.word	.LANCHOR0+3372
	.word	.LANCHOR1+2144
	.word	.LC0
	.word	.LANCHOR0+1776
	.word	.LANCHOR2
	.word	.LC294
	.word	.LC295
	.word	.LANCHOR2+1800
	.word	.LANCHOR0+3722
	.word	.LC296
	.word	-178307901
	.word	-1431655765
	.word	.LANCHOR0+3806
	.word	.LC297
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
	cmp	r0, #1
	ldr	r4, .L4142+52
	.pad #36
	sub	sp, sp, #36
	mov	r9, r0
	add	r3, r4, #3328
	ldr	r5, [r4, #580]
	ldrh	r7, [r3]
	movw	r3, #3324
	ldrh	r6, [r4, r3]
	movw	r3, #3326
	ldrh	r2, [r4, r3]
	add	r6, r7, r6
	uxth	r6, r6
	bne	.L3951
	ldr	r8, .L4142
	ldr	r10, [r8, #-2796]
	cmp	r10, #0
	bne	.L3952
	ldr	r3, [r8, #-2792]
	cmp	r3, #0
	beq	.L3951
.L3952:
	ldr	r3, .L4142+4
	ldrh	r1, [r3]
	cmp	r6, r1, lsr #2
	bls	.L3951
	sub	r3, r3, #400
	ldrh	r3, [r3]
	cmp	r3, r6
	bcs	.L3951
	bl	timer_get_time
	add	r10, r10, #20
	ldr	r4, [r8, #-2792]
	add	r4, r4, #20
	cmp	r10, r0
	movcc	r3, #0
	strcc	r3, [r8, #-2796]
	bl	timer_get_time
	cmp	r4, r0
	ldrcc	r3, .L4142
	movcc	r2, #0
	strcc	r2, [r3, #-2792]
	b	.L4065
.L3951:
	ldrb	r3, [r4, #3362]	@ zero_extendqisi2
	cmp	r3, #6
	ldrls	pc, [pc, r3, asl #2]
	b	.L4065
.L3957:
	.word	.L3956
	.word	.L3958
	.word	.L3959
	.word	.L3960
	.word	.L3961
	.word	.L4066
	.word	.L3963
.L3956:
	add	r7, r2, r7
	movw	r2, #3332
	ldrh	fp, [r4, r2]
	movw	r2, #3330
	ldrh	r8, [r4, r2]
	movw	r2, #65535
	ldrh	r1, [r5, #80]
	uxth	r7, r7
	add	r8, fp, r8
	ldr	r3, .L4142+52
	cmp	r1, r2
	uxth	r8, r8
	beq	.L3964
	cmp	r9, #0
	bne	.L3965
	add	r3, r3, #3344
	ldrh	r3, [r3]
	cmp	r6, r3, asl #1
	bge	.L4065
.L3965:
	movw	r3, #3368
	ldr	r9, .L4142+52
	ldrh	r1, [r4, r3]
	add	r0, r9, #3776
	add	r1, r1, #1
	uxth	r1, r1
	strh	r1, [r4, r3]	@ movhi
	bl	_list_get_gc_head_node
	movw	r3, #65535
	cmp	r0, r3
	beq	.L3967
	ldr	r1, [r9, #576]
	mov	r2, r0, asl #1
	ldr	r3, [r9, #3364]
	ldrh	r2, [r1, r2]
	add	r1, r9, #3712
	add	r3, r3, #1
	str	r3, [r9, #3364]
	ldrh	r1, [r1]
	cmp	r1, r2
	bcs	.L3968
	add	r1, r9, #564
	ldrh	r1, [r1]
	cmp	r3, r1, lsr #4
	bls	.L3967
	movw	r3, #3346
	ldrh	r3, [r9, r3]
	cmp	r3, r2
	bls	.L3967
.L3968:
	mov	r1, #0
	ldrb	r2, [r4, #3361]	@ zero_extendqisi2
	bl	gc_add_sblk
	ldr	r9, .L4142+52
	cmp	r0, #0
	beq	.L3969
	mov	r3, #1
	strb	r3, [r9, #3362]
	mov	r3, #0
	str	r3, [r9, #3364]
	b	.L4065
.L3967:
	movw	r3, #3368
	mov	r2, #0
	strh	r2, [r4, r3]	@ movhi
.L3969:
	cmp	r7, #15
	movls	r8, #2
	bls	.L3970
	movw	r3, #3334
	movw	r2, #3784
	ldrh	r3, [r4, r3]
	ldrh	r2, [r4, r2]
	cmp	r3, r2
	movls	r3, #0
	movhi	r3, #1
	cmp	r8, #0
	movne	r8, r3
	orreq	r8, r3, #1
	cmp	r8, #0
	movne	r8, #2
	moveq	r8, #1
.L3970:
	ldr	r3, .L4142+8
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3971
	ldr	r3, [r4, #580]
	ldrb	r2, [r4, #3361]	@ zero_extendqisi2
	str	r7, [sp]
	ldrh	r1, [r3, #124]
	ldr	r0, .L4142+12
	str	r1, [sp, #4]
	ldrh	r1, [r3, #120]
	str	r1, [sp, #8]
	movw	r1, #1982
	ldrh	r3, [r3, #122]
	str	r3, [sp, #12]
	ldrh	r3, [r5, #80]
	str	r3, [sp, #16]
	mov	r3, r6
	bl	printk
.L3971:
	ldrb	r0, [r4, #3361]	@ zero_extendqisi2
	mov	r1, r8
	mov	r2, #1
	bl	gc_search_src_blk
	cmp	r0, #0
	ble	.L3972
.L3973:
	mov	r3, #1
	b	.L4140
.L3972:
	ldr	r3, .L4142+52
	mov	r1, #3
	mov	r2, #1
	ldrb	r0, [r3, #3361]	@ zero_extendqisi2
	bl	gc_search_src_blk
	cmp	r0, #0
	bgt	.L3973
	b	.L4065
.L3964:
	ldr	fp, .L4142+8
	ldr	r2, [fp]
	str	fp, [sp, #24]
	tst	r2, #256
	beq	.L3974
	ldrb	r2, [r3, #3361]	@ zero_extendqisi2
	movw	r1, #1994
	str	r7, [sp]
	ldrh	r3, [r5, #124]
	ldr	r0, .L4142+12
	str	r3, [sp, #4]
	ldrh	r3, [r5, #120]
	str	r3, [sp, #8]
	ldrh	r3, [r5, #122]
	str	r9, [sp, #16]
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
.L3974:
	cmp	r9, #1
	bne	.L3975
	bl	gc_scan_static_data
	ldr	r3, [r4, #580]
	ldrh	r3, [r3, #122]
	cmp	r3, #0
	beq	.L3976
.L3977:
	mov	r3, #1
	strb	r3, [r4, #3361]
	b	.L4140
.L3976:
	bl	gc_static_wearleveling
	cmp	r0, #0
	bne	.L3977
	bl	gc_block_vpn_scan
	cmp	r7, #0
	beq	.L4070
	cmp	r6, r8
	ldr	r1, .L4142+4
	bcs	.L3980
	ldrh	r3, [r1]
	cmp	r3, r6
	bhi	.L3981
.L3980:
	ldrh	r1, [r1]
	add	r3, r6, r8
	ldr	r2, .L4142+52
	cmp	r3, r1
	blt	.L3981
	movw	r3, #3334
	ldrh	r1, [r2, r3]
	movw	r3, #3784
	ldrh	r3, [r2, r3]
	cmp	r1, r3
	bcc	.L4070
.L3981:
	ldr	r5, .L4142+52
	mov	r10, #1
	ldr	r3, [r4, #3364]
	add	r2, r5, #564
	strb	r10, [r4, #3361]
	add	r3, r3, r10
	str	r3, [r4, #3364]
	ldrh	r2, [r2]
	cmp	r3, r2, lsr #5
	bls	.L3983
	movw	r3, #3334
	ldrh	r3, [r5, r3]
	cmp	r3, r7
	bls	.L3983
	add	r0, r5, #3776
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r3, #65535
	cmp	r0, r3
	beq	.L4070
	ldr	r3, [r5, #576]
	mov	r0, r0, asl r10
	ldrh	r2, [r3, r0]
	add	r3, r5, #3712
	ldrh	r1, [r3]
	ldrb	r3, [r5, #3720]	@ zero_extendqisi2
	mul	r3, r3, r1
	cmp	r2, r3
	bgt	.L4070
	mov	r3, #0
	mov	r0, r10
	str	r3, [r5, #3364]
	mov	r1, #2
	b	.L4131
.L3983:
	movw	r2, #3334
	ldr	r3, .L4142+52
	ldrh	r1, [r4, r2]
	movw	r2, #3784
	ldrh	r2, [r4, r2]
	cmp	r1, r2
	movcs	r0, #1
	movcs	r1, #2
	movcs	r2, r0
	bcs	.L4132
	movw	r2, #3330
	ldrh	r5, [r3, r2]
	cmp	r5, #0
	bne	.L3987
	movw	r2, #3332
	ldrh	r3, [r3, r2]
	cmp	r3, #8
	bls	.L3979
.L3987:
	mov	r0, #1
	mov	r1, r0
.L4131:
	mov	r2, #4
.L4132:
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L3973
	b	.L4070
.L3975:
	ldr	r3, .L4142+16
	ldrh	r3, [r3]
	cmp	r3, r6
	bcc	.L4065
.L4070:
	mov	r5, #16
.L3979:
	ldr	r3, [r4, #3356]
	ldr	r10, .L4142+52
	cmp	r3, #0
	beq	.L3988
	mov	ip, #0
	add	r0, r10, #3776
	mov	r1, ip
	str	ip, [r10, #3356]
	mov	fp, #1
	str	ip, [sp, #28]
	strb	fp, [r10, #3361]
	bl	_list_get_gc_head_node
	movw	r1, #65535
	cmp	r0, r1
	beq	.L3989
	ldr	r1, [r10, #576]
	mov	r3, r0, asl fp
	ldr	ip, [sp, #28]
	ldrh	r3, [r1, r3]
	cmp	r3, #8
	bhi	.L3989
	mov	r1, ip
	mov	r2, fp
	str	fp, [r10, #3356]
	bl	gc_add_sblk
	cmp	r0, #0
	bne	.L3973
.L3989:
	ldr	r0, .L4142+20
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r2, #65535
	ldr	r10, .L4142+52
	cmp	r0, r2
	beq	.L3990
	ldr	r2, [r10, #576]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #4
	bhi	.L3990
	mov	r2, #1
	mov	r1, #0
	str	r2, [r10, #3356]
	bl	gc_add_sblk
	cmp	r0, #0
	bne	.L3973
.L3990:
	ldr	r0, .L4142+24
	mov	r1, #0
	bl	_list_get_gc_head_node
	movw	r2, #65535
	ldr	r10, .L4142+52
	cmp	r0, r2
	beq	.L3988
	ldr	r2, [r10, #576]
	mov	r3, r0, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #4
	bhi	.L3988
	mov	r1, #1
	mov	r2, #0
	str	r1, [r10, #3356]
	bl	gc_add_sblk
	cmp	r0, #0
	bne	.L3973
.L3988:
	ldr	r1, [r4, #580]
	mov	r0, #1
	ldr	fp, .L4142+52
	strb	r0, [r4, #3361]
	ldrh	ip, [r1, #124]
	cmp	ip, #0
	beq	.L3991
	ldr	r3, [sp, #24]
	mov	r2, #0
	strb	r0, [fp, #3362]
	strb	r2, [fp, #3361]
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L3992
	stmia	sp, {r7, ip}
	ldrh	r3, [r1, #120]
	ldr	r0, .L4142+28
	str	r3, [sp, #8]
	ldrh	r3, [r1, #122]
	movw	r1, #2084
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
	b	.L3992
.L3991:
	add	r2, fp, #3344
	ldrh	lr, [r2]
	mov	r10, r2
	cmp	r6, lr
	bcs	.L3993
	cmp	r7, #0
	beq	.L3994
	cmp	r7, #16
	bls	.L3995
	movw	r2, #3334
	ldrh	r1, [fp, r2]
	movw	r2, #3784
	ldrh	r2, [fp, r2]
	cmp	r1, r2
	bhi	.L3995
	mov	r1, r0
	mov	r2, #4
	str	ip, [sp, #24]
	bl	gc_search_src_blk
	uxth	r0, r0
	ldr	ip, [sp, #24]
	cmp	r0, #0
	ldreqb	r0, [fp, #3361]	@ zero_extendqisi2
	beq	.L4138
	add	r0, fp, #3776
	mov	r1, ip
	bl	_list_get_gc_head_node
	movw	r2, #65535
	cmp	r0, r2
	beq	.L4063
	ldr	ip, [fp, #576]
	mov	r1, r0, asl #1
	ldr	r2, [fp, #3364]
	ldrh	r1, [ip, r1]
	add	ip, fp, #3712
	add	r2, r2, #1
	str	r2, [fp, #3364]
	ldrh	ip, [ip]
	cmp	ip, r1
	bcs	.L3999
	add	ip, fp, #564
	ldrh	ip, [ip]
	cmp	r2, ip, lsr #4
	bls	.L4063
	movw	r2, #3346
	ldrh	r3, [fp, r2]
	cmp	r3, r1
	bls	.L4063
.L3999:
	mov	r1, #0
	ldrb	r2, [r4, #3361]	@ zero_extendqisi2
	bl	gc_add_sblk
	mov	r3, #1
	str	r3, [r4, #3356]
	mov	r3, #0
	str	r3, [r4, #3364]
	b	.L4063
.L3995:
	mov	r0, #1
	mov	r1, #2
	mov	r2, r0
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L4063
	ldrb	r0, [r4, #3361]	@ zero_extendqisi2
.L4137:
	mov	r1, #3
	mov	r2, #2
.L4134:
	bl	gc_search_src_blk
	uxth	r0, r0
	b	.L3997
.L3994:
	ldr	r3, [sp, #24]
	strb	r7, [fp, #3361]
	ldr	r3, [r3]
	tst	r3, #256
	beq	.L4001
	str	r7, [sp]
	mov	r2, r7
	str	r7, [sp, #4]
	ldrh	r3, [r1, #120]
	ldr	r0, .L4142+28
	str	r3, [sp, #8]
	ldrh	r3, [r1, #122]
	movw	r1, #2114
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
.L4001:
	cmp	r6, #16
	ldrb	r0, [r4, #3361]	@ zero_extendqisi2
	bls	.L4002
.L4138:
	mov	r1, #3
	mov	r2, #4
	b	.L4134
.L4002:
	mov	r1, #1
	mov	r2, r1
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	ldreq	r3, .L4142+52
	ldreqb	r0, [r3, #3361]	@ zero_extendqisi2
	bne	.L4063
	b	.L4137
.L3993:
	cmp	r9, #1
	bne	.L3992
	cmp	r6, lr, asl #1
	bge	.L4003
	cmp	r8, r7, lsr #1
	bcs	.L4004
	movw	r1, #3334
	ldrh	r0, [fp, r1]
	movw	r1, #3784
	ldrh	r1, [fp, r1]
	cmp	r0, r1
	addcc	r3, fp, #3744
	ldrcch	r3, [r3]
	movcc	r3, r3, lsr #2
	strcch	r3, [r2]	@ movhi
	bcc	.L3992
.L4004:
	ldr	r9, .L4142+52
	mov	r1, #0
	add	r0, r9, #3776
	bl	_list_get_gc_head_node
	movw	r3, #65535
	cmp	r0, r3
	mov	fp, r0
	bne	.L4005
.L4010:
	cmp	r7, #1
	bhi	.L4006
	b	.L4007
.L4005:
	add	r2, r9, #564
	ldr	r3, [r9, #3364]
	ldrh	r2, [r2]
	add	r3, r3, #1
	str	r3, [r9, #3364]
	cmp	r3, r2, lsr #4
	mov	r3, r0, asl #1
	bls	.L4008
	ldr	r2, [r9, #576]
	mov	r1, #0
	str	r1, [r9, #3364]
	ldrh	ip, [r2, r3]
	movw	r2, #3346
	ldrh	r2, [r9, r2]
	cmp	ip, r2
	bcs	.L4008
	mov	r2, #1
	str	r3, [sp, #24]
	bl	gc_add_sblk
	cmp	r0, #0
	ldr	r3, [sp, #24]
	bne	.L4141
.L4008:
	ldr	r2, [r4, #576]
	ldr	r9, .L4142+52
	ldrh	r2, [r2, r3]
	add	r3, r9, #3712
	ldrh	r3, [r3]
	cmp	r2, r3, lsr #1
	bhi	.L4009
	mov	r0, fp
	mov	r1, #1
	mov	r2, #0
	bl	gc_add_sblk
.L4141:
	mov	r3, #1
	strb	r3, [r9, #3362]
	b	.L3992
.L4009:
	movw	r3, #3330
	movw	r1, #3332
	ldrh	r1, [r9, r1]
	ldrh	r3, [r9, r3]
	add	r3, r3, r1
	add	r1, r9, #3744
	ldrh	r1, [r1]
	cmp	r3, r1, asl #1
	ble	.L4010
	movw	r3, #3346
	ldrh	r3, [r9, r3]
	cmp	r3, r2
	bcc	.L4007
	b	.L4010
.L4006:
	cmp	r7, #16
	mov	r0, #1
	ldr	r9, .L4142+52
	strb	r0, [r4, #3361]
	bls	.L4011
	movw	r3, #3334
	ldrh	r2, [r9, r3]
	movw	r3, #3784
	ldrh	r3, [r9, r3]
	cmp	r2, r3
	bhi	.L4011
	mov	r1, r0
	mov	r2, #4
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	ldreqb	r0, [r9, #3361]	@ zero_extendqisi2
	moveq	r1, #3
	moveq	r2, #4
	bne	.L4012
	b	.L4133
.L4011:
	mov	r0, #1
	mov	r1, #2
	mov	r2, r0
	bl	gc_search_src_blk
	uxth	r0, r0
	cmp	r0, #0
	bne	.L4012
	ldrb	r0, [r4, #3361]	@ zero_extendqisi2
	mov	r1, #3
	mov	r2, #2
.L4133:
	bl	gc_search_src_blk
	uxth	r0, r0
.L4012:
	ldr	r3, .L4142+4
	cmp	r6, r8, lsr #1
	ldrh	r3, [r3]
	movhi	r3, r3, lsr #2
	bhi	.L4136
	b	.L4135
.L4007:
	cmp	r6, r8
	ldr	r6, .L4142+4
	bcs	.L4014
	add	r0, r6, #36
	mov	r1, #4
	mov	r7, #0
	strb	r7, [r4, #3361]
	bl	_list_get_gc_head_node
	movw	r3, #65535
	cmp	r0, r3
	beq	.L4014
	ldr	r3, [r6, #-3168]
	mov	r0, r0, asl #1
	ldrh	r1, [r6, #-32]
	ldrh	r2, [r3, r0]
	ldrb	r3, [r6, #-24]	@ zero_extendqisi2
	mul	r3, r3, r1
	cmp	r2, r3, asr #1
	bgt	.L4014
	mov	r0, r7
	mov	r1, #3
	mov	r2, #4
	bl	gc_search_src_blk
	ldrh	r3, [r6]
	uxth	r0, r0
.L4135:
	mov	r3, r3, lsr #1
.L4136:
	strh	r3, [r10]	@ movhi
	b	.L3997
.L4014:
	ldrh	r3, [r6]
	mov	r3, r3, lsr #2
	strh	r3, [r10]	@ movhi
	b	.L3992
.L4003:
	add	r3, fp, #3744
	mov	r5, ip
	ldrh	r3, [r3]
	mov	r3, r3, lsr #2
	strh	r3, [r2]	@ movhi
	b	.L3992
.L3997:
	cmp	r0, #0
	beq	.L3992
.L4063:
	mov	r3, #1
	strb	r3, [r4, #3362]
	b	.L3992
.L3958:
	movw	r5, #3372
	movw	r3, #65535
	ldrh	r2, [r4, r5]
	cmp	r2, r3
	bne	.L4017
	bl	gc_get_src_blk
	ldr	r3, .L4142+52
	strh	r0, [r3, r5]	@ movhi
.L4017:
	movw	r2, #3372
	ldr	r3, .L4142+52
	ldrh	r2, [r4, r2]
	movw	r1, #65535
	cmp	r2, r1
	moveq	r2, #0
	streqb	r2, [r3, #3362]
	beq	.L4065
	ldr	r0, .L4142+48
	ldr	r1, [r3, #568]
	ldrh	ip, [r0, #52]
	add	r1, r1, r2, asl #2
	cmp	ip, #0
	addne	r3, r3, #3424
	movne	r0, #0
	beq	.L4019
.L4020:
	uxth	lr, r0
	cmp	lr, ip
	bcs	.L4019
	ldrh	lr, [r3, #2]!
	add	r0, r0, #1
	cmp	lr, r2
	bne	.L4020
.L4024:
	movw	r3, #3372
	mvn	r2, #0
	strh	r2, [r4, r3]	@ movhi
	b	.L4065
.L4019:
	ldrb	r3, [r1, #2]	@ zero_extendqisi2
	tst	r3, #192
	and	r3, r3, #224
	moveq	r1, #1
	movne	r1, #0
	cmp	r3, #224
	movne	r3, r1
	orreq	r3, r1, #1
	cmp	r3, #0
	beq	.L4022
	ldr	r3, [r4, #576]
	mov	r2, r2, asl #1
	ldrh	r3, [r3, r2]
	cmp	r3, #0
	beq	.L4024
	ldr	r1, .L4142+56
	movw	r2, #2347
	ldr	r0, .L4142+60
	bl	printk
	bl	dump_stack
	b	.L4024
.L4022:
	mov	r3, #2
	b	.L4140
.L3959:
	bl	gc_scan_src_blk
	cmn	r0, #1
	moveq	r3, #3
	beq	.L4140
	movw	r3, #3372
	ldr	r1, .L4142+48
	ldrh	r3, [r4, r3]
	movw	r0, #65535
	ldr	r2, .L4142+52
	cmp	r3, r0
	mov	r6, r1
	beq	.L3973
	ldrh	r0, [r1, #20]
	cmp	r0, #0
	movne	r3, #4
	strneb	r3, [r2, #3362]
	movne	r3, #0
	strneh	r3, [r1, #22]	@ movhi
	bne	.L4065
.L4026:
	mov	r1, #1
	strb	r1, [r2, #3362]
	ldr	r2, [r2, #576]
	mov	r3, r3, asl r1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L4027
	ldr	r1, .L4142+56
	movw	r2, #2375
	ldr	r0, .L4142+60
	bl	printk
	bl	dump_stack
.L4027:
	movw	r5, #3372
	ldr	r7, .L4142+48
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
	bls	.L4024
	strh	r5, [r7, #26]	@ movhi
	bl	ftl_flush
	bl	pm_flush
	bl	ftl_ext_info_flush
	mov	r0, r5
	bl	ftl_info_flush
	b	.L4024
.L3960:
	ldr	r7, .L4142+48
.L4109:
	bl	gc_scan_src_blk_one_page
	ldr	r0, .L4142+52
	ldrh	r2, [r7, #2]
	add	r3, r0, #3712
	ldr	r5, .L4142+48
	ldrh	r3, [r3]
	cmp	r2, r3
	bcs	.L4030
	cmp	r6, #7
	bls	.L4109
	b	.L4065
.L4030:
	ldrh	r3, [r5, #20]
	ldr	r2, .L4142+8
	cmp	r3, #0
	beq	.L4031
	ldr	r2, [r2]
	mov	r1, #4
	strb	r1, [r0, #3362]
	mov	r1, #0
	tst	r2, #256
	strh	r1, [r5, #22]	@ movhi
	beq	.L4032
	ldrh	r1, [r5]
	ldr	ip, [r0, #576]
	ldr	r0, .L4142+32
	mov	r2, r1, asl #1
	ldrh	r2, [ip, r2]
	bl	printk
.L4032:
	movw	r3, #3372
	ldr	r2, [r4, #576]
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #1
	ldrh	r2, [r2, r3]
	ldrh	r3, [r5, #20]
	cmp	r2, r3
	beq	.L4033
	ldr	r1, .L4142+56
	movw	r2, #2409
	ldr	r0, .L4142+60
	bl	printk
	bl	dump_stack
.L4033:
	movw	r3, #3372
	ldrh	r1, [r5, #20]
	ldrh	r3, [r4, r3]
	ldr	r2, [r4, #576]
	mov	r3, r3, asl #1
	strh	r1, [r2, r3]	@ movhi
	b	.L4065
.L4031:
	mov	r3, #1
	strb	r3, [r0, #3362]
	ldr	r3, [r2]
	ldrh	r1, [r5]
	ldr	r6, [r0, #568]
	tst	r3, #256
	add	r6, r6, r1, asl #2
	beq	.L4034
	ldrb	r2, [r6, #2]	@ zero_extendqisi2
	ldr	r0, .L4142+36
	mov	r2, r2, lsr #5
	bl	printk
.L4034:
	ldrb	r3, [r6, #2]	@ zero_extendqisi2
	tst	r3, #192
	and	r3, r3, #224
	moveq	r2, #1
	movne	r2, #0
	cmp	r3, #224
	movne	r3, r2
	orreq	r3, r2, #1
	cmp	r3, #0
	beq	.L4035
	ldr	r1, .L4142+56
	movw	r2, #2419
	ldr	r0, .L4142+60
	bl	printk
	bl	dump_stack
.L4035:
	movw	r7, #3372
	ldr	r6, .L4142+48
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
	bls	.L4065
	b	.L4139
.L3961:
	cmp	r9, #0
	bne	.L4037
	ldr	r3, .L4142+16
	ldrh	r3, [r3]
	cmp	r3, r6
	bcc	.L4065
.L4037:
	ldrh	r2, [r5, #80]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L4038
	ldrb	r7, [r4, #3361]	@ zero_extendqisi2
	cmp	r7, #1
	bne	.L4038
	ldr	r8, .L4142+48
	bl	ftl_flush
	movw	r3, #314
	mov	r1, #5
	ldrh	r0, [r8, r3]
	cmp	r0, #0
	movne	r0, r7
	bl	zftl_gc_get_free_sblk
	movw	r3, #65535
	cmp	r0, r3
	mov	r6, r0
	beq	.L4041
	ldr	r7, [r4, #568]
	add	r7, r7, r0, asl #2
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	tst	r3, #224
	beq	.L4042
	ldr	r1, .L4142+56
	mov	r2, #2448
	ldr	r0, .L4142+60
	bl	printk
	bl	dump_stack
.L4042:
	ldrb	r3, [r7, #2]	@ zero_extendqisi2
	and	r3, r3, #15
	orr	r3, r3, #176
	strb	r3, [r7, #2]
.L4064:
	mov	r0, r6
	mov	r1, #1
	bl	ftl_erase_sblk
	mov	r3, #5
	add	r1, r5, #96
	strb	r3, [r5, #84]
	mov	r0, r6
	ldr	r9, .L4142+40
	bl	ftl_get_blk_list_in_sblk
	mov	r7, #0
	strh	r6, [r5, #80]	@ movhi
	mov	r1, #255
	mov	r10, r9
	strh	r7, [r5, #82]	@ movhi
	ldrh	r3, [r10], #-28
	strb	r7, [r5, #85]
	strh	r7, [r5, #90]	@ movhi
	uxtb	r0, r0
	strb	r0, [r5, #89]
	smulbb	r0, r3, r0
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	strh	r0, [r5, #86]	@ movhi
	movw	r5, #3722
	ldrh	r2, [r4, r5]
	ldr	r0, [r4, #3724]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r2, [r4, r5]
	ldrb	r3, [r4, #3720]	@ zero_extendqisi2
	mov	r1, #255
	ldr	r0, [r4, #3728]
	mul	r2, r3, r2
	mov	r2, r2, asl #2
	bl	ftl_memset
	ldrh	r3, [r4, r5]
	ldrb	r2, [r4, #3720]	@ zero_extendqisi2
	mov	r1, #255
	ldr	r0, [r4, #3732]
	mvn	r5, #0
	mul	r2, r2, r3
	bl	ftl_memset
	ldr	r3, [r4, #580]
	strh	r5, [r3, #128]	@ movhi
	strh	r5, [r3, #130]	@ movhi
	str	r6, [r3, #132]
	bl	pm_flush
	bl	ftl_ext_info_flush
	ldr	r3, [r4, #3340]
	mov	r0, r7
	strh	r7, [r8, #52]	@ movhi
	strh	r6, [r3, #126]	@ movhi
	movw	r3, #310
	strh	r7, [r8, r3]	@ movhi
	strh	r7, [r10]	@ movhi
	strh	r7, [r9, #-24]	@ movhi
	str	r5, [r4, #3692]
	bl	ftl_info_flush
	b	.L4065
.L4038:
	cmp	r9, #1
	ldr	r8, .L4142+52
	ldr	r9, .L4142+48
	movne	r7, #1
	moveq	r7, #4
	cmp	r6, #15
	addls	r7, r7, #4
.L4045:
	sub	r7, r7, #1
	uxtb	r7, r7
	cmp	r7, #255
	beq	.L4065
	bl	gc_do_copy_back
	ldrb	r3, [r4, #3361]	@ zero_extendqisi2
	ldr	r10, .L4142+52
	cmp	r3, #0
	ldr	r6, .L4142+48
	bne	.L4046
	ldrb	r3, [r8, #3314]	@ zero_extendqisi2
	cmp	r3, #3
	bhi	.L4047
	bl	ftl_write_commit
.L4047:
	ldrh	r2, [r9, #22]
	ldrh	r3, [r9, #20]
	ldr	r6, .L4142+52
	cmp	r2, r3
	bcc	.L4045
	mov	r3, #1
	strb	r3, [r6, #3362]
	bl	ftl_write_commit
	bl	ftl_flush
	ldr	r3, .L4142+48
	ldr	r2, [r6, #576]
	ldrh	r3, [r3]
	mov	r3, r3, asl #1
	ldrh	r3, [r2, r3]
	cmp	r3, #0
	beq	.L4049
	ldr	r1, .L4142+56
	movw	r2, #2523
	ldr	r0, .L4142+60
	bl	printk
	bl	dump_stack
.L4049:
	movw	r3, #3372
	ldr	r1, [r4, #576]
	ldrh	r0, [r4, r3]
	mov	r2, r0, asl #1
	ldrh	ip, [r1, r2]
	cmp	ip, #0
	bne	.L4050
	strh	ip, [r1, r2]	@ movhi
	ldr	r2, .L4142+52
	ldrh	r0, [r2, r3]
	bl	ftl_free_sblk
	b	.L4024
.L4050:
	mov	r1, #0
	mov	r2, #1
	bl	gc_add_sblk
	b	.L4024
.L4143:
	.align	2
.L4142:
	.word	.LANCHOR5
	.word	.LANCHOR0+3744
	.word	.LANCHOR2
	.word	.LC298
	.word	.LANCHOR0+3344
	.word	.LANCHOR0+3772
	.word	.LANCHOR0+3780
	.word	.LC299
	.word	.LC300
	.word	.LC301
	.word	.LANCHOR0+3712
	.word	.LANCHOR0+3688
	.word	.LANCHOR0+3372
	.word	.LANCHOR0
	.word	.LANCHOR1+2160
	.word	.LC0
.L4046:
	ldr	r3, .L4142+44
	ldrh	r2, [r3]
	cmp	r2, #0
	beq	.L4051
	mov	r5, #0
	strh	r5, [r3]	@ movhi
	bl	sblk_wait_write_queue_completed
	bl	gc_write_completed
	ldr	r0, [r10, #3692]
	cmn	r0, #1
	beq	.L4052
	bl	ftl_mask_bad_block
.L4052:
	ldr	r3, [r4, #580]
	str	r5, [r4, #3692]
	strh	r5, [r6, #52]	@ movhi
	ldrh	r0, [r3, #80]
	bl	ftl_free_sblk
	ldr	r0, [r4, #3380]
	ldr	r2, [r4, #580]
	mvn	r3, #0
	ldr	r1, [r4, #3340]
	cmp	r0, #0
	strh	r3, [r2, #80]	@ movhi
	strh	r3, [r1, #126]	@ movhi
	strh	r3, [r2, #130]	@ movhi
	beq	.L4053
	bl	zbuf_free
.L4053:
	mov	r5, #0
	str	r5, [r4, #3380]
	bl	flt_sys_flush
	ldr	r1, .L4142+56
	movw	r2, #2561
	ldr	r0, .L4142+60
	strb	r5, [r4, #3362]
	bl	printk
	bl	dump_stack
	b	.L4065
.L4051:
	ldrh	r3, [r5, #86]
	ldrh	r2, [r9, #22]
	cmp	r3, #1
	ldrh	r3, [r9, #20]
	bls	.L4054
	cmp	r2, r3
	bcc	.L4045
	mov	r3, #1
	strb	r3, [r10, #3362]
	ldrh	r3, [r6, #52]
	add	r2, r3, #1
	strh	r2, [r6, #52]	@ movhi
	ldrh	r2, [r6]
	add	r3, r6, r3, asl #1
	strh	r2, [r3, #54]	@ movhi
	mvn	r3, #0
	strh	r3, [r6]	@ movhi
	b	.L4065
.L4054:
	cmp	r2, r3
	mov	r1, #5
	strb	r1, [r10, #3362]
	bcc	.L4055
	ldrh	r3, [r6, #52]
	add	r2, r3, #1
	strh	r2, [r6, #52]	@ movhi
	ldrh	r2, [r6]
	add	r3, r6, r3, asl #1
	strh	r2, [r3, #54]	@ movhi
	mvn	r3, #0
	strh	r3, [r6]	@ movhi
.L4055:
	bl	ftl_flush
	bl	sblk_wait_write_queue_completed
	bl	gc_write_completed
	ldrh	r2, [r5, #80]
	ldr	r3, [r4, #580]
	ldr	r5, .L4142+52
	strh	r2, [r3, #128]	@ movhi
	bl	pm_flush
	bl	ftl_ext_info_flush
	mov	r3, #0
	strh	r3, [r6, #12]	@ movhi
	add	r3, r5, #3712
	ldrb	r1, [r4, #3768]	@ zero_extendqisi2
	ldrh	r3, [r3]
	cmp	r1, #2
	ldr	r2, .L4142+48
	strh	r1, [r6, #16]	@ movhi
	strh	r3, [r6, #14]	@ movhi
	bne	.L4057
	mov	r3, r3, asl #1
	strh	r3, [r2, #14]	@ movhi
	ldrb	r3, [r5, #3769]	@ zero_extendqisi2
	cmp	r3, #0
	moveq	r3, #1
	streqh	r3, [r2, #16]	@ movhi
.L4057:
	mov	r3, #0
	strh	r3, [r6, #18]	@ movhi
	b	.L4065
.L4066:
	ldr	r8, .L4142+48
	mov	r5, #0
.L3962:
	bl	gc_check_data_one_wl
	subs	r10, r0, #0
	beq	.L4060
	ldr	r3, .L4142+48
	mov	r5, #0
	strh	r5, [r3, #52]	@ movhi
	ldr	r3, [r4, #580]
	ldrh	r0, [r3, #80]
	bl	ftl_free_sblk
	ldr	r2, [r4, #580]
	ldr	r1, [r4, #3340]
	mvn	r3, #0
	ldr	r0, [r4, #3380]
	strh	r3, [r2, #80]	@ movhi
	strh	r3, [r1, #126]	@ movhi
	strh	r3, [r2, #130]	@ movhi
	bl	zbuf_free
	str	r5, [r4, #3380]
	strb	r5, [r4, #3362]
.L4139:
	bl	flt_sys_flush
	b	.L4065
.L4060:
	ldrh	r2, [r8, #12]
	ldrh	r3, [r8, #14]
	ldr	r7, .L4142+52
	cmp	r2, r3
	bcc	.L4061
	ldr	r0, [r7, #3380]
	mov	r3, #6
	strb	r3, [r7, #3362]
	bl	zbuf_free
	str	r10, [r7, #3380]
	b	.L4065
.L4061:
	cmp	r6, #15
	bls	.L3962
	cmp	r9, #1
	bne	.L4065
	add	r5, r5, #1
	uxtb	r5, r5
	cmp	r5, #4
	bls	.L3962
	b	.L4065
.L3963:
	bl	gc_update_l2p_map_new
	mvn	r6, #0
	bl	gc_free_src_blk
	bl	ftl_flush
	bl	pm_flush
	strh	r6, [r5, #80]	@ movhi
	bl	ftl_ext_info_flush
	ldr	r3, [r4, #3340]
	mov	r0, #0
	strh	r6, [r3, #126]	@ movhi
	bl	ftl_info_flush
	mov	r3, #0
.L4140:
	strb	r3, [r4, #3362]
.L4065:
	mov	r0, #16
	b	.L4129
.L3992:
	mov	r0, r5
	b	.L4129
.L4041:
	ldr	r1, .L4142+56
	movw	r2, #2454
	ldr	r0, .L4142+60
	bl	printk
	bl	dump_stack
	b	.L4064
.L4129:
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
	ldr	r3, .L4172
	.pad #28
	sub	sp, sp, #28
	mov	r5, r0
	mov	r4, r1
	mov	r7, r2
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L4145
	ldr	r3, [r9]
	mov	r1, r5
	ldr	r0, .L4172+4
	mov	r2, r4
	str	r3, [sp]
	mov	r3, r7
	bl	printk
.L4145:
	cmp	r5, #0
	ldreq	r3, .L4172+8
	moveq	r5, #24576
	ldreq	r3, [r3, #520]
	beq	.L4147
.L4146:
	cmp	r5, #3
	bhi	.L4161
	mov	r5, r5, asl #13
	mov	r3, #8192
.L4147:
	add	r2, r4, r7
	cmp	r2, r3
	bhi	.L4161
	ldr	r8, .L4172+8
	add	r4, r5, r4
	mov	r0, r4
	ldrb	r5, [r8, #3312]	@ zero_extendqisi2
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
.L4149:
	cmp	r5, #0
	beq	.L4170
	mov	r0, #0
	bl	buf_alloc
	subs	fp, r0, #0
	bne	.L4150
	bl	ftl_write_commit
	b	.L4149
.L4150:
	mov	r3, #0
	strb	r3, [fp, #41]
	ldr	r3, [sp, #12]
	rsb	r0, r10, r6
	clz	r0, r0
	ldrb	r2, [r8, #3312]	@ zero_extendqisi2
	rsb	r1, r3, r6
	clz	r1, r1
	mov	r0, r0, lsr #5
	strb	r2, [fp, #40]
	mov	r1, r1, lsr #5
	orrs	r3, r1, r0
	beq	.L4153
	cmp	r0, #0
	beq	.L4154
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
	bhi	.L4153
	b	.L4169
.L4154:
	cmp	r1, #0
	beq	.L4153
	smulbb	r2, r6, r2
	ldr	r3, [sp, #16]
	rsb	r2, r2, r3
.L4169:
	strb	r2, [fp, #40]
.L4153:
	ldrb	r0, [fp, #41]	@ zero_extendqisi2
	sub	r5, r5, #1
	ldrb	r2, [fp, #40]	@ zero_extendqisi2
	ldr	r1, [fp, #4]
	mov	r2, r2, asl #9
	add	r0, r1, r0, asl #9
	mov	r1, r9
	bl	ftl_memcpy
	ldr	r1, [r8, #3340]
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
	b	.L4149
.L4170:
	bl	ftl_write_commit
	mov	r0, r5
	mov	r1, #1
	ldr	r4, .L4172+12
	bl	zftl_do_gc
.L4158:
	ldr	r2, .L4172+16
	ldrh	r3, [r4]
	ldrh	r2, [r2]
	add	r3, r3, r2
	cmp	r3, #7
	bgt	.L4171
	mov	r1, #1
	mov	r0, #0
	bl	zftl_do_gc
	b	.L4158
.L4171:
	bl	timer_get_time
	ldr	r3, .L4172+20
	str	r0, [r3, #-2792]
	mov	r0, #0
	b	.L4148
.L4161:
	mvn	r0, #0
.L4148:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4173:
	.align	2
.L4172:
	.word	.LANCHOR2
	.word	.LC302
	.word	.LANCHOR0
	.word	.LANCHOR0+3324
	.word	.LANCHOR0+3328
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
	ldr	r6, .L4227
	.pad #36
	sub	sp, sp, #36
	mov	r4, r1
	ldr	r3, [r6, #520]
	cmp	r2, r3
	mvnhi	r0, #0
	bhi	.L4178
	ldr	r3, .L4227+4
	add	r8, r0, #24576
	ldr	r10, .L4227+8
	ldr	r2, [r3]
	ldr	r5, [r10, #-2788]
	tst	r2, #4096
	str	r10, [sp, #12]
	add	r1, r1, r5
	str	r3, [sp, #16]
	str	r1, [r10, #-2788]
	beq	.L4179
	mov	r3, #0
	ldr	r0, .L4227+12
	str	r3, [sp]
	mov	r2, r8
	mov	r3, r4
	bl	printk
.L4179:
	ldr	r3, [r6, #3340]
	ldr	r9, [r3, #8]
	add	r2, r9, #1
	str	r2, [r3, #8]
	bl	ftl_write_commit
	bl	ftl_flush
	ldrb	r7, [r6, #3312]	@ zero_extendqisi2
	mov	r0, r8
	mov	r1, r7
	bl	__aeabi_uidiv
	mls	r10, r7, r0, r8
	mov	r5, r0
	cmp	r10, #0
	beq	.L4180
	rsb	r7, r10, r7
	cmp	r7, r4
	movcs	r7, r4
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #24]
	bne	.L4181
	mov	r0, r5
	add	r1, sp, #24
	mov	r2, #0
	bl	pm_log2phys
.L4181:
	ldr	r3, [sp, #24]
	uxth	fp, r7
	cmn	r3, #1
	beq	.L4183
	mov	r0, #0
	bl	buf_alloc
	subs	r3, r0, #0
	beq	.L4183
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
	ldr	r2, [r6, #3340]
	ldr	r3, [r2, #76]
	add	r3, r3, #1
	str	r3, [r2, #76]
.L4183:
	add	r5, r5, #1
	rsb	r4, fp, r4
.L4180:
	cmp	r4, #0
	beq	.L4185
	bl	ftl_flush
.L4185:
	ldr	r7, .L4227
	mvn	r3, #0
	ldr	r10, .L4227+16
	str	r3, [sp, #28]
.L4186:
	ldrb	r3, [r6, #3312]	@ zero_extendqisi2
	cmp	r4, r3
	bcc	.L4226
	mov	r0, r5
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #24]
	beq	.L4187
	mov	r0, #0
	bl	buf_alloc
	subs	fp, r0, #0
	beq	.L4189
	ldrb	r2, [r7, #3312]	@ zero_extendqisi2
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
	b	.L4225
.L4187:
	mov	r0, r5
	add	r1, sp, #24
	mov	r2, #0
	bl	pm_log2phys
	ldr	r3, [sp, #24]
	cmn	r3, #1
	beq	.L4189
	add	r1, sp, #28
	mov	r2, #1
	mov	r0, r5
	bl	pm_log2phys
	ldrh	r2, [r10]
	mvn	r0, #0
	rsb	r1, r2, #21
	ldr	r3, [sp, #24]
	mov	r3, r3, lsr r2
	bic	r0, r3, r0, asl r1
	ldrb	r1, [r7, #3766]	@ zero_extendqisi2
	bl	__aeabi_uidiv
	uxth	r0, r0
	bl	ftl_vpn_decrement
.L4225:
	ldr	r2, [r7, #3340]
	ldr	r3, [r2, #76]
	add	r3, r3, #1
	str	r3, [r2, #76]
.L4189:
	ldrb	r3, [r6, #3312]	@ zero_extendqisi2
	add	r5, r5, #1
	rsb	r4, r3, r4
	b	.L4186
.L4226:
	cmp	r4, #0
	beq	.L4193
	mov	r0, r5
	bl	lpa_hash_get_ppa
	cmn	r0, #1
	str	r0, [sp, #24]
	bne	.L4194
	mov	r0, r5
	add	r1, sp, #24
	mov	r2, #0
	bl	pm_log2phys
.L4194:
	ldr	r3, [sp, #24]
	cmn	r3, #1
	beq	.L4193
	mov	r0, #0
	bl	buf_alloc
	subs	r7, r0, #0
	beq	.L4193
	mov	r3, #0
	strb	r3, [r7, #41]
	ldrb	r3, [r6, #3312]	@ zero_extendqisi2
	str	r5, [r7, #20]
	cmp	r4, r3
	strb	r4, [r7, #40]
	str	r9, [r7, #16]
	bcc	.L4196
	ldr	r1, .L4227+20
	movw	r2, #1251
	ldr	r0, .L4227+24
	bl	printk
	bl	dump_stack
.L4196:
	mov	r2, r4, asl #9
	mov	r1, #0
	ldr	r0, [r7, #4]
	bl	ftl_memset
	mov	r0, r7
	bl	ftl_write_buf
	bl	ftl_write_commit
	ldr	r2, [r6, #3340]
	ldr	r3, [r2, #76]
	add	r3, r3, #1
	str	r3, [r2, #76]
.L4193:
	ldr	r3, [sp, #12]
	ldr	r1, [r3, #-2788]
	cmp	r1, #8192
	bls	.L4199
	ldr	r3, [sp, #16]
	ldr	r3, [r3]
	tst	r3, #4096
	beq	.L4197
	mov	r3, #0
	ldr	r0, .L4227+12
	str	r3, [sp]
	mov	r2, r8
	mov	r3, r4
	bl	printk
.L4197:
	ldr	r3, [sp, #12]
	mov	r4, #0
	str	r4, [r3, #-2788]
	bl	flt_sys_flush
	mov	r3, #1
	str	r3, [r6, #3356]
.L4199:
	mov	r0, #0
.L4178:
	add	sp, sp, #36
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4228:
	.align	2
.L4227:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	.LC303
	.word	.LANCHOR0+3764
	.word	.LANCHOR1+2172
	.word	.LC0
	.fnend
	.size	zftl_discard, .-zftl_discard
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
	ldr	r5, .L4236
	ldrb	r2, [r5, #1625]	@ zero_extendqisi2
	ldr	r3, [r5, #1620]
	cmp	r2, #0
	ldrb	r3, [r3, #19]	@ zero_extendqisi2
	bne	.L4229
	sub	r2, r3, #5
	mov	r8, r0
	cmp	r3, #68
	cmpne	r2, #2
	mov	r9, r1
	bls	.L4231
	sub	r3, r3, #19
	tst	r3, #239
	bne	.L4229
.L4231:
	ldr	r7, .L4236
	add	r10, r7, #596
.L4233:
	ldr	r3, [r5, #1620]
	ldrh	r3, [r3, #10]
	cmp	r3, r4
	bls	.L4229
	mov	r6, r4, asl #1
	add	fp, r4, r9
	ldr	r0, .L4236+4
	mov	r1, r4
	mov	r2, fp
	ldrh	r3, [r10, r6]
	bl	printk
	ldrh	r2, [r10, r6]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L4229
	ldr	r6, .L4236+8
	mov	r1, #255
	mov	r2, #16384
	add	r4, r4, #1
	ldr	r0, [r6, #-3552]
	uxth	r4, r4
	bl	memset
	ldr	r3, [r7, #1620]
	ldr	r2, [r6, #-3552]
	mov	r0, r8
	mov	r1, fp
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	str	r3, [sp]
	mov	r3, r2
	bl	flash_prog_page
	b	.L4233
.L4229:
	add	sp, sp, #12
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4237:
	.align	2
.L4236:
	.word	.LANCHOR0
	.word	.LC304
	.word	.LANCHOR3
	.fnend
	.size	id_block_prog_msb_ff_data, .-id_block_prog_msb_ff_data
	.align	2
	.global	write_idblock
	.type	write_idblock, %function
write_idblock:
	.fnstart
	@ args = 0, pretend = 0, frame = 128
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	.pad #148
	sub	sp, sp, #148
	ldr	r9, .L4305
	mov	r6, r0
	mov	r0, #256000
	mov	r10, r1
	mov	r8, r2
	ldr	r3, [r9, #1620]
	ldrb	r4, [r3, #9]	@ zero_extendqisi2
	ldrh	r5, [r3, #10]
	ldr	r3, .L4305+4
	ldrh	r7, [r3, #30]
	ldrb	r3, [r9]	@ zero_extendqisi2
	str	r3, [sp, #52]
	bl	ftl_malloc
	subs	r3, r0, #0
	str	r3, [sp, #32]
	bne	.L4239
.L4243:
	mvn	r0, #0
	b	.L4240
.L4239:
	ldrb	r3, [r9]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L4302
	ldrb	r3, [r9, #1625]	@ zero_extendqisi2
	cmp	r3, #0
	movne	r3, #0
	bne	.L4302
	ldr	r3, .L4305+8
	ldrb	r3, [r3, #-2806]	@ zero_extendqisi2
	adds	r3, r3, #0
	movne	r3, #1
.L4302:
	add	r6, r6, #508
	mov	r0, #0
	add	r6, r6, #3
	str	r3, [sp, #40]
	bl	zftl_flash_exit_slc_mode
	ldrb	r3, [r9, #1625]	@ zero_extendqisi2
	mov	r6, r6, lsr #9
	cmp	r6, #8
	strb	r3, [r9]
	bls	.L4275
	cmp	r6, #500
	bhi	.L4243
	b	.L4242
.L4275:
	mov	r6, #8
.L4242:
	ldr	r2, [r10]
	ldr	r3, .L4305+12
	cmp	r2, r3
	bne	.L4243
	uxth	r4, r4
	smulbb	r1, r4, r5
	uxth	r1, r1
	sub	r0, r1, #1
	add	r0, r0, r6
	bl	__aeabi_uidiv
	mov	r3, #0
	movw	r2, #63871
	str	r0, [sp, #64]
	add	r0, r10, #254976
	add	r0, r0, #512
.L4247:
	ldr	r1, [r0, #-4]!
	cmp	r1, #0
	bne	.L4244
	ldr	r1, [r10, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, #4096
	sub	r2, r2, #1
	movhi	r3, #0
	cmp	r2, #4096
	str	r1, [r0, #512]
	bne	.L4247
	b	.L4246
.L4244:
	ldr	r0, .L4305+16
	bl	printk
.L4246:
	smulbb	r3, r4, r7
	ldr	r0, .L4305+20
	mov	r1, r8
	mov	r2, #4
	ldr	r4, .L4305
	uxth	r3, r3
	str	r3, [sp, #56]
	mov	r3, #5
	bl	rknand_print_hex
	ldr	r0, .L4305+24
	mov	r1, r6
	mov	r2, r6
	bl	printk
	sub	r3, r8, #4
	str	r3, [sp, #44]
	mov	r3, #0
	str	r3, [sp, #28]
	str	r3, [sp, #20]
.L4272:
	ldr	r3, .L4305+8
	ldrb	r3, [r3, #-2806]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L4248
	ldr	r3, [sp, #44]
	ldr	r1, [sp, #64]
	ldr	r2, [r3, #4]
	ldr	r3, [sp, #56]
	mul	fp, r3, r2
	ldr	r3, [sp, #20]
	cmp	r3, #0
	cmpne	r1, #1
	bls	.L4249
	ldr	r3, [sp, #44]
	ldr	r3, [r3]
	add	r3, r3, #1
	cmp	r2, r3
	bne	.L4249
	b	.L4250
.L4248:
	ldr	r3, [r4, #1696]
	ldr	r2, [sp, #20]
	add	r3, r3, r2
	ldrb	r3, [r3, #32]	@ zero_extendqisi2
	cmp	r3, #255
	beq	.L4250
	ldr	r2, [sp, #56]
	mul	fp, r2, r3
	ldr	r3, [sp, #40]
	cmp	r3, #0
	beq	.L4249
	ldr	r3, [sp, #20]
	cmp	r3, #1
	ldreqb	r3, [sp, #52]	@ zero_extendqisi2
	beq	.L4303
	mov	r0, #0
	bl	zftl_flash_exit_slc_mode
	mov	r3, #0
.L4303:
	strb	r3, [r4]
	strb	r3, [r4, #1625]
.L4249:
	ldr	r0, [sp, #32]
	mov	r1, #512
	bl	__memzero
	ldr	r3, [r4, #1620]
	mov	r8, #0
	mov	r0, fp
	ldrb	r2, [r3, #9]	@ zero_extendqisi2
	ldrh	r3, [r3, #10]
	str	r2, [sp, #36]
	ldrh	r2, [sp, #36]
	ldr	r1, [sp, #36]
	smulbb	r3, r3, r2
	uxth	r7, r3
	ldr	r3, .L4305+4
	ldrh	r5, [r3, #30]
	ldrb	r3, [r4, #1688]	@ zero_extendqisi2
	strb	r8, [r4, #1688]
	smulbb	r5, r5, r2
	str	r3, [sp, #68]
	bl	__aeabi_uidiv
	uxth	r5, r5
	mov	r1, r0
	mov	r0, r8
	bl	flash_erase_block
	cmp	r6, r7
	movls	r0, #1
	bls	.L4252
	mov	r0, r8
	add	r1, fp, r5
	bl	flash_erase_block
	mov	r0, #2
.L4252:
	ldr	r3, .L4305
	ldr	r2, [r3, #1620]
	ldrh	r3, [r2, #10]
	ldrb	r1, [r2, #12]	@ zero_extendqisi2
	mov	r3, r3, asl #2
	mul	r0, r0, r3
	bl	__aeabi_idiv
	mov	r1, r5
	str	r0, [sp, #72]
	mov	r0, fp
	bl	__aeabi_uidivmod
	rsb	r3, r1, fp
	str	r1, [sp, #60]
	cmp	fp, r3
	str	r3, [sp, #48]
	bne	.L4278
	ldrb	r3, [r4, #516]	@ zero_extendqisi2
	cmp	r3, #9
	bne	.L4278
	ldr	r3, .L4305+28
	mov	r1, #1024
	ldr	r5, [r3, #-3528]
	mov	r0, r5
	bl	__memzero
	ldr	r2, .L4305+32
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
	ldr	r2, [sp, #36]
	strh	r0, [r5, #18]	@ movhi
	add	r0, r5, #12
	cmp	r2, #8
	movhi	r2, #70
	movls	r2, #16
	strb	r2, [r5, #21]
	bl	js_hash
	sub	r3, r6, #4
	str	r0, [r5, #8]
	str	r3, [sp, #24]
	b	.L4253
.L4278:
	mov	r5, #0
	str	r6, [sp, #24]
.L4253:
	mov	r8, r10
	mov	r9, #0
.L4256:
	ldr	r3, [sp, #72]
	cmp	r9, r3
	bcs	.L4304
	ldr	r3, [sp, #60]
	ldrb	r2, [r4, #1625]	@ zero_extendqisi2
	add	r7, r9, r3
	ubfx	r7, r7, #2, #16
	cmp	r2, #0
	add	r0, r7, #1
	mov	r1, r0, asl #1
	add	r3, r4, r1
	ldrh	r3, [r3, #4]
	beq	.L4258
	ldrb	ip, [r4, #1]	@ zero_extendqisi2
	cmp	ip, #0
	moveq	r3, r0
	movne	r3, r1
.L4258:
	ldrb	r1, [r4, #516]	@ zero_extendqisi2
	ldr	r0, .L4305
	cmp	r1, #9
	movw	r1, #61424
	str	r1, [sp, #84]
	mov	r1, r7, asl #1
	add	r0, r0, r1
	subne	r3, r3, #1
	ldrh	r0, [r0, #4]
	movne	r3, r3, asl #2
	cmp	r2, #0
	str	r3, [sp, #80]
	moveq	r7, r0
	beq	.L4262
	ldrb	r2, [r4, #1]	@ zero_extendqisi2
	cmp	r2, #0
	movne	r7, r1
.L4262:
	ldr	r2, [sp, #36]
	cmp	r5, #0
	mul	r0, r2, r7
	bne	.L4263
	ldr	r2, [sp, #48]
	str	r8, [sp]
	add	ip, r0, r2
	ldr	r2, [r8]
	ldr	r0, .L4305+36
	mov	r1, ip
	str	ip, [sp, #76]
	stmib	sp, {r2, r3}
	movw	r3, #61424
	mov	r2, r9
	str	r3, [sp, #12]
	mov	r3, r6
	bl	printk
	mov	r1, r8
	ldr	ip, [sp, #76]
	add	r2, sp, #80
	mov	r0, ip
	bl	fw_flash_page_prog.constprop.29
	ldrb	r3, [r4, #1625]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L4264
	ldr	r1, [sp, #36]
	ldr	r0, [sp, #48]
	bl	__aeabi_uidiv
	add	r2, r7, #1
	uxth	r2, r2
	mov	r1, r0
	mov	r0, r5
	bl	id_block_prog_msb_ff_data
.L4264:
	ldr	r3, [sp, #24]
	add	r8, r8, #2048
	add	r3, r3, #16
	cmp	r9, r3
	bcc	.L4265
	ldr	r3, [sp, #24]
	add	r3, r3, #20
	cmp	r9, r3
	addcc	r8, r10, #2048
	b	.L4265
.L4263:
	ldr	r3, [sp, #48]
	mov	r1, r5
	add	r2, sp, #80
	add	r0, r0, r3
	bl	fw_flash_page_prog.constprop.29
.L4265:
	add	r9, r9, #4
	mov	r5, #0
	uxth	r9, r9
	b	.L4256
.L4304:
	ldr	r3, .L4305
	mov	r0, fp
	ldrb	r2, [sp, #68]	@ zero_extendqisi2
	ldr	r1, [sp, #24]
	strb	r2, [r3, #1688]
	ldr	r2, [sp, #32]
	bl	id_block_read_data.constprop.30
	ldr	r3, [sp, #40]
	cmp	r3, #0
	beq	.L4267
	mov	r0, #0
	bl	zftl_flash_exit_slc_mode
	mov	r3, #0
	strb	r3, [r4]
	strb	r3, [r4, #1625]
.L4267:
	ldr	r3, [sp, #24]
	mov	r5, #0
	mov	r8, r3, asl #7
.L4270:
	ldr	r3, [sp, #32]
	ldr	r2, [r3, r5, asl #2]
	ldr	r3, [r10, r5, asl #2]
	cmp	r2, r3
	beq	.L4268
	ldr	r0, [sp, #32]
	mov	r1, #512
	bl	__memzero
	mov	r0, #0
	mov	r1, fp
	bl	flash_erase_block
	cmp	r5, r8
	bcs	.L4269
	b	.L4250
.L4268:
	add	r5, r5, #1
	cmp	r5, r8
	bne	.L4270
.L4269:
	ldr	r3, [sp, #28]
	add	r3, r3, #1
	str	r3, [sp, #28]
	cmp	r3, #5
	bhi	.L4271
.L4250:
	ldr	r3, [sp, #20]
	add	r3, r3, #1
	str	r3, [sp, #20]
	ldr	r3, [sp, #44]
	add	r3, r3, #4
	str	r3, [sp, #44]
	ldr	r3, [sp, #20]
	cmp	r3, #4
	bne	.L4272
.L4271:
	ldr	r3, .L4305
	mov	r0, #0
	ldrb	r2, [sp, #52]	@ zero_extendqisi2
	strb	r2, [r3]
	mov	r2, #2
	strb	r2, [r3, #1700]
	bl	zftl_flash_enter_slc_mode
	ldr	r0, [sp, #32]
	bl	ftl_free
	ldr	r3, [sp, #28]
	clz	r0, r3
	mov	r0, r0, lsr #5
	rsb	r0, r0, #0
.L4240:
	add	sp, sp, #148
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4306:
	.align	2
.L4305:
	.word	.LANCHOR0
	.word	.LANCHOR2
	.word	.LANCHOR5
	.word	-52655045
	.word	.LC305
	.word	.LC306
	.word	.LC307
	.word	.LANCHOR3
	.word	1179535694
	.word	.LC308
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
	ldr	ip, .L4311
	add	r1, r0, r1
	mov	r2, #0
.L4308:
	cmp	r0, r1
	beq	.L4310
	ldrb	r3, [r0], #1	@ zero_extendqisi2
	eor	r3, r3, r2, lsr #24
	add	r3, ip, r3, asl #2
	ldr	r3, [r3, #2568]
	eor	r2, r3, r2, asl #8
	b	.L4308
.L4310:
	mov	r0, r2
	bx	lr
.L4312:
	.align	2
.L4311:
	.word	.LANCHOR2
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
	ldr	r5, .L4336
	bne	.L4314
	ldr	r2, [r2]
	ldr	r3, .L4336+4
	cmp	r2, r3
	bne	.L4314
	mov	r0, #256000
	mov	r3, #1
	strb	r3, [r5, #-2784]
	bl	ftl_malloc
	mov	r1, #256000
	str	r0, [r5, #-2780]
	bl	__memzero
	str	r4, [r5, #-2776]
.L4314:
	str	r6, [sp]
	mov	r3, r4
	ldr	r0, .L4336+8
	ldr	r1, [r5, #-2780]
	ldr	r2, [r8]
	bl	printk
	ldrb	r3, [r5, #-2784]	@ zero_extendqisi2
	ldr	r9, .L4336
	cmp	r3, #0
	beq	.L4313
	ldr	r7, [r9, #-2780]
	cmp	r7, #0
	beq	.L4313
	sub	r0, r4, #64
	cmp	r0, #500
	bcs	.L4316
	rsb	r2, r4, #564
	add	r0, r7, r0, asl #9
	cmp	r2, r6
	mov	r1, r8
	movcs	r2, r6
	mov	r2, r2, asl #9
	bl	ftl_memcpy
	b	.L4317
.L4316:
	cmp	r4, #564
	bcs	.L4325
.L4317:
	ldr	r3, [r5, #-2776]
	ldr	r8, .L4336
	cmp	r3, r4
	beq	.L4323
	mov	r0, r7
	mov	r9, #0
	strb	r9, [r8, #-2784]
	bl	ftl_free
	str	r9, [r8, #-2780]
.L4323:
	add	r4, r4, r6
	str	r4, [r5, #-2776]
	b	.L4313
.L4325:
	ldr	r3, .L4336+12
	ldr	r0, [r9, #-2776]
	ldr	r3, [r3, #1620]
	sub	r0, r0, #64
	cmp	r0, #500
	ldrb	r3, [r3, #9]	@ zero_extendqisi2
	movcs	r0, #500
	cmp	r3, #4
	beq	.L4326
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
	b	.L4319
.L4326:
	mov	r3, #0
.L4318:
	cmp	r0, #256
	add	r1, sp, #8
	mov	r2, r3, asl #1
	movls	r2, r3
	str	r2, [r1, r3, asl #2]
	add	r3, r3, #1
	cmp	r3, #5
	bne	.L4318
.L4319:
	mov	r3, #61952
.L4322:
	ldr	r2, [r7, r3, asl #2]
	cmp	r2, #0
	addne	r3, r3, #2048
	movne	r0, r3, asl #2
	bne	.L4321
.L4320:
	sub	r3, r3, #1
	cmp	r3, #4096
	bne	.L4322
	mov	r0, r0, asl #9
.L4321:
	mov	r1, r7
	add	r2, sp, #8
	bl	write_idblock
	ldr	r0, [r5, #-2780]
	mov	r7, #0
	strb	r7, [r5, #-2784]
	bl	ftl_free
	str	r7, [r5, #-2780]
	b	.L4323
.L4313:
	add	sp, sp, #52
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, pc}
.L4337:
	.align	2
.L4336:
	.word	.LANCHOR5
	.word	-52655045
	.word	.LC309
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
	beq	.L4339
	mov	r0, r1
	mov	r2, r3
	mov	r1, r6
	bl	write_loader_lba
.L4339:
	ldr	r3, .L4344
	mov	r0, r7
	mov	r1, r4
	mov	r2, r6
	ldr	r3, [r3, #-3560]
	ldr	ip, [r3, #24]
	mov	r3, r5
	ldmfd	sp!, {r4, r5, r6, r7, r8, lr}
	bx	ip
.L4345:
	.align	2
.L4344:
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
	ldr	r3, .L4452
	stmfd	sp!, {r4, r5, r6, r7, lr}
	.save {r4, r5, r6, r7, lr}
	cmp	r1, r3
	.pad #532
	sub	sp, sp, #532
	mov	r4, r1
	mov	r5, r2
	beq	.L4348
	bhi	.L4349
	sub	r3, r3, #3456
	sub	r3, r3, #5
	cmp	r1, r3
	beq	.L4350
	bhi	.L4351
	sub	r3, r3, #125
	cmp	r1, r3
	beq	.L4352
	bhi	.L4353
	sub	r3, r3, #237
	cmp	r1, r3
	bne	.L4409
	bl	rknand_dev_flush
	b	.L4440
.L4353:
	ldr	r3, .L4452+4
	cmp	r1, r3
	beq	.L4355
	add	r3, r3, #1
	cmp	r1, r3
	beq	.L4356
	b	.L4409
.L4351:
	ldr	r3, .L4452+8
	cmp	r1, r3
	beq	.L4357
	bhi	.L4358
	sub	r3, r3, #1952
	sub	r3, r3, #9
	cmp	r1, r3
	beq	.L4359
	b	.L4409
.L4358:
	ldr	r3, .L4452+12
	cmp	r1, r3
	beq	.L4357
	add	r3, r3, #10
	cmp	r1, r3
	beq	.L4357
	b	.L4409
.L4349:
	ldr	r3, .L4452+16
	cmp	r1, r3
	beq	.L4360
	bhi	.L4361
	sub	r3, r3, #78
	cmp	r1, r3
	beq	.L4362
	bcc	.L4363
	add	r3, r3, #21
	cmp	r1, r3
	beq	.L4364
	add	r3, r3, #56
	cmp	r1, r3
	beq	.L4365
	b	.L4409
.L4361:
	ldr	r3, .L4452+20
	cmp	r1, r3
	mov	r6, r3
	beq	.L4366
	bhi	.L4367
	sub	r3, r3, #956
	sub	r3, r3, #1
	cmp	r1, r3
	beq	.L4368
	add	r3, r3, #956
	cmp	r1, r3
	beq	.L4369
	b	.L4409
.L4367:
	ldr	r3, .L4452+24
	cmp	r1, r3
	beq	.L4369
	add	r3, r3, #1
	cmp	r1, r3
	beq	.L4366
	b	.L4409
.L4363:
	ldr	r0, .L4452+28
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r4, r0, #0
	bne	.L4370
.L4375:
	mvn	r0, #11
	b	.L4347
.L4370:
	mov	r1, r5
	mov	r2, #512
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4446
	ldr	r0, .L4452+32
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r4, #4]
	cmp	r3, #8
	str	r3, [sp, #4]
	bhi	.L4444
	bl	rknand_device_lock
	mov	r2, r4
	ldr	r0, [r4]
	ldr	r1, [sp, #4]
	bl	id_block_read_data.constprop.30
	bl	rknand_device_unlock
	mov	r0, r5
	mov	r1, r4
	ldr	r2, [sp, #4]
	mov	r2, r2, asl #9
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L4439
	ldr	r0, .L4452+36
.L4442:
	bl	printk
.L4444:
	mov	r0, r4
.L4445:
	bl	ftl_free
.L4443:
	mvn	r0, #13
	b	.L4347
.L4362:
	ldr	r0, .L4452+40
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r4, r0, #0
	beq	.L4375
	mov	r1, r5
	mov	r2, #4096
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L4376
.L4446:
	ldr	r0, .L4452+44
	b	.L4442
.L4376:
	ldr	r5, .L4452+48
	ldr	r0, .L4452+52
	ldmia	r4, {r1, r2}
	bl	printk
	ldr	r3, [r5, #-2772]
	mov	r6, r5
	cmp	r3, #0
	bne	.L4377
	mov	r0, #260096
	bl	ftl_malloc
	cmp	r0, #0
	str	r0, [r5, #-2772]
	beq	.L4444
	mov	r1, #260096
	bl	__memzero
.L4377:
	ldr	r2, [r4, #4]
	movw	r3, #4088
	cmp	r2, r3
	bhi	.L4444
	ldr	r3, [r4]
	cmp	r3, #251904
	bhi	.L4444
	ldr	r0, [r6, #-2772]
	add	r1, r4, #8
	add	r0, r0, r3
	bl	memcpy
.L4439:
	mov	r0, r4
	bl	ftl_free
.L4440:
	mov	r4, #0
	b	.L4374
.L4365:
	ldr	r0, .L4452+56
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4375
	mov	r1, r5
	mov	r2, #28
	bl	rk_copy_from_user
	cmp	r0, #0
	beq	.L4380
	ldr	r0, .L4452+44
	bl	printk
	b	.L4381
.L4380:
	ldmia	r6, {r1, r2}
	ldr	r0, .L4452+60
	bl	printk
	ldr	r1, [r6]
	cmp	r1, #256000
	bhi	.L4381
	ldr	r5, .L4452+48
	ldr	r0, [r5, #-2772]
	cmp	r0, #0
	beq	.L4381
	bl	CRC_32
	ldr	r3, [r6, #4]
	cmp	r3, r0
	beq	.L4383
	mov	r0, r6
	bl	ftl_free
	b	.L4410
.L4383:
	bl	rknand_device_lock
	ldr	r1, [r5, #-2772]
	add	r2, r6, #8
	ldr	r0, [r6]
	bl	write_idblock
	mov	r4, #0
	bl	rknand_device_unlock
	ldr	r0, [r5, #-2772]
	bl	ftl_free
	str	r4, [r5, #-2772]
	mov	r0, r6
	b	.L4387
.L4381:
	mov	r0, r6
	b	.L4445
.L4364:
	ldr	r0, .L4452+64
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4375
	bl	ReadFlashInfo
	mov	r0, r5
	mov	r1, r6
	mov	r2, #11
	b	.L4450
.L4348:
	ldr	r0, .L4452+68
	bl	printk
	bl	nand_blk_add_whole_disk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4375
	mov	r1, #64
	bl	__memzero
	mov	r0, r5
	mov	r1, r6
	mov	r2, #64
	b	.L4450
.L4360:
	ldr	r0, .L4452+72
	bl	printk
	ldr	r3, .L4452+48
	mov	r0, r5
	mov	r2, #4
	ldr	r3, [r3, #-2768]
	ldr	r3, [r3, #20]
	add	r1, sp, #528
	str	r3, [r1, #-524]!
.L4434:
	bl	rk_copy_to_user
	cmp	r0, #0
	bne	.L4443
	b	.L4440
.L4368:
	ldr	r0, .L4452+76
	bl	printk
	mov	r0, #4096
	bl	ftl_malloc
	subs	r6, r0, #0
	beq	.L4375
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
.L4450:
	bl	rk_copy_to_user
	subs	r4, r0, #0
	mov	r0, r6
	bne	.L4445
.L4387:
	bl	ftl_free
	b	.L4374
.L4352:
	ldr	r0, .L4452+80
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	subs	r4, r0, #0
	beq	.L4388
.L4393:
	ldr	r0, .L4452+44
	bl	printk
	b	.L4443
.L4388:
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+84
	cmp	r2, r3
	beq	.L4389
.L4390:
	mvn	r4, #0
	b	.L4374
.L4389:
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4390
	ldr	r6, .L4452+48
	mov	r2, #512
	add	r0, sp, #8
	ldr	r1, [r6, #-2768]
	bl	memcpy
	ldr	r2, [r6, #-2764]
	ldr	r3, .L4452+88
	cmp	r2, r3
	beq	.L4391
	add	r0, sp, #72
	mov	r1, #128
	str	r4, [sp, #16]
	str	r4, [sp, #20]
	bl	__memzero
.L4391:
	add	r0, sp, #264
	mov	r1, #256
	mov	r3, #0
	str	r3, [sp, #24]
	bl	__memzero
	b	.L4448
.L4355:
	ldr	r0, .L4452+92
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4393
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+84
	cmp	r2, r3
	bne	.L4390
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4390
	ldr	r2, .L4452+48
	ldr	r3, .L4452+88
	ldr	r1, [r2, #-2764]
	cmp	r1, r3
	bne	.L4410
	ldr	r3, [sp, #20]
	sub	r1, r3, #1
	cmp	r1, #127
	mvnhi	r0, #2
	bhi	.L4347
	ldr	r4, [r2, #-2768]
	add	r1, sp, #72
	add	r0, r4, #64
	str	r3, [r4, #12]
	ldr	r2, [sp, #20]
	bl	memcpy
	mov	r0, #1
	mov	r1, r4
	b	.L4441
.L4359:
	ldr	r0, .L4452+96
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4393
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+100
	cmp	r2, r3
	bne	.L4390
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4390
	ldr	r5, .L4452+48
	ldr	r3, [r5, #-2760]
	cmp	r3, #0
	bne	.L4394
.L4397:
	mov	r0, #0
	b	.L4347
.L4394:
	ldr	r3, [r5, #-2756]
	ldr	r2, .L4452+104
	ldr	r1, [r3]
	cmp	r1, r2
	beq	.L4395
	str	r2, [r3]
	mov	r2, #504
	ldr	r3, [r5, #-2756]
	str	r2, [r3, #4]
	mov	r2, #0
	str	r2, [r3, #8]
	str	r2, [r3, #12]
.L4395:
	ldr	r1, [r5, #-2756]
	mov	r4, #0
	mov	r0, r4
	str	r4, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #-2768]
	ldr	r2, .L4452+84
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r0, [r5, #-2768]
	mov	r1, #128
	ldrne	r3, .L4452+48
	movne	r2, #504
	add	r0, r0, #64
	ldrne	r3, [r3, #-2768]
	stmneib	r3, {r2, r4}
	mov	r4, #0
	str	r4, [r0, #-52]
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #-2768]
	bl	StorageSysDataStore
	str	r4, [r5, #-2760]
	str	r4, [r5, #-2764]
	b	.L4374
.L4350:
	ldr	r0, .L4452+108
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4393
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+112
	cmp	r2, r3
	bne	.L4390
	ldr	r3, [sp, #12]
	cmp	r3, #512
	bhi	.L4390
	ldr	r5, .L4452+48
	ldr	r3, [r5, #-2760]
	cmp	r3, #1
	beq	.L4397
	ldr	r3, [r5, #-2756]
	mov	r4, #0
	ldr	r2, .L4452+104
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r1, [r5, #-2756]
	ldrne	r3, [r5, #-2756]
	movne	r2, #504
	strne	r0, [r3, #8]
	strne	r2, [r3, #4]
	strne	r0, [r3, #12]
	mov	r3, #1
	mov	r0, #0
	str	r3, [r1, #16]
	bl	StorageSysDataStore
	ldr	r3, [r5, #-2768]
	ldr	r2, .L4452+84
	ldr	r1, [r3]
	cmp	r1, r2
	strne	r2, [r3]
	ldr	r0, [r5, #-2768]
	ldrne	r3, .L4452+48
	movne	r1, #504
	movne	r2, #0
	add	r0, r0, #64
	ldrne	r3, [r3, #-2768]
	stmneib	r3, {r1, r2}
	mov	r1, #128
	str	r4, [r0, #-52]
	bl	__memzero
	mov	r0, #1
	ldr	r1, [r5, #-2768]
	bl	StorageSysDataStore
	mov	r3, #1
	str	r3, [r5, #-2760]
	b	.L4374
.L4356:
	ldr	r0, .L4452+116
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4393
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+120
	cmp	r2, r3
	bne	.L4390
	ldr	r2, [sp, #12]
	cmp	r2, #512
	addls	r0, sp, #16
	ldrls	r1, .L4452+124
	bls	.L4447
	b	.L4390
.L4357:
	ldr	r0, .L4452+12
	cmp	r4, r0
	mov	r7, r0
	ldreq	r0, .L4452+128
	beq	.L4437
	ldr	r3, .L4452+132
	cmp	r4, r3
	ldreq	r0, .L4452+136
	ldrne	r0, .L4452+140
.L4437:
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4393
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+144
	cmp	r2, r3
	bne	.L4443
	ldr	r3, .L4452+132
	ldr	r6, .L4452+48
	cmp	r4, r3
	bne	.L4403
	ldr	r3, [r6, #-2768]
	mov	r0, r5
	add	r1, sp, #8
	mov	r2, #16
	ldr	r3, [r3, #20]
	str	r3, [sp, #12]
	strb	r3, [sp, #16]
	bl	rk_copy_to_user
	cmp	r0, #0
	beq	.L4347
	b	.L4443
.L4403:
	ldr	r3, [r6, #-2240]
	cmp	r3, #10
	bhi	.L4443
	ldr	r1, [r6, #-2768]
	ldr	r2, [sp, #12]
	ldr	r3, [r1, #24]
	cmp	r3, r2
	cmpne	r3, #0
	movne	r3, #1
	moveq	r3, #0
	beq	.L4404
	ldr	r0, .L4452+148
	mov	r1, r2
	bl	printk
	ldr	r3, [r6, #-2240]
	add	r3, r3, #1
	str	r3, [r6, #-2240]
	b	.L4443
.L4404:
	cmp	r4, r7
	str	r3, [r6, #-2240]
	mov	r0, #1
	moveq	r2, r3
	movne	r3, #1
	moveq	r3, r2
	str	r2, [r1, #24]
	str	r3, [r1, #20]
	bl	StorageSysDataStore
	cmn	r0, #1
	bne	.L4440
	mvn	r4, #1
	b	.L4374
.L4369:
	ldr	r0, .L4452+152
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4393
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+156
	cmp	r2, r3
	bne	.L4390
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L4390
	ldr	r3, .L4452+160
	add	r0, sp, #16
	cmp	r4, r3
	ldr	r3, .L4452+48
	ldreq	r1, [r3, #-2236]
	ldrne	r1, [r3, #-2232]
	add	r1, r1, #8
.L4447:
	bl	memcpy
.L4448:
	mov	r0, r5
	add	r1, sp, #8
	mov	r2, #520
	b	.L4434
.L4366:
	ldr	r0, .L4452+164
	bl	printk
	mov	r1, r5
	mov	r2, #520
	add	r0, sp, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4393
	ldr	r2, [sp, #8]
	ldr	r3, .L4452+156
	cmp	r2, r3
	bne	.L4390
	ldr	r2, [sp, #12]
	cmp	r2, #504
	bhi	.L4390
	cmp	r4, r6
	add	r2, r2, #8
	ldr	r4, .L4452+48
	bne	.L4408
	add	r1, sp, #8
	ldr	r0, [r4, #-2236]
	bl	memcpy
	mov	r0, #2
	ldr	r1, [r4, #-2236]
	b	.L4441
.L4408:
	add	r1, sp, #8
	ldr	r0, [r4, #-2232]
	bl	memcpy
	ldr	r1, [r4, #-2232]
	mov	r0, #3
.L4441:
	bl	StorageSysDataStore
	mov	r4, r0
.L4374:
	ldr	r0, .L4452+168
	mov	r1, r4
	bl	printk
	mov	r0, r4
	b	.L4347
.L4409:
	mvn	r0, #21
	b	.L4347
.L4410:
	mvn	r0, #1
.L4347:
	add	sp, sp, #532
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, pc}
.L4453:
	.align	2
.L4452:
	.word	1074033155
	.word	1074029570
	.word	1074031656
	.word	1074031666
	.word	1074033235
	.word	1074034193
	.word	1074034194
	.word	.LC310
	.word	.LC312
	.word	.LC313
	.word	.LC314
	.word	.LC311
	.word	.LANCHOR5
	.word	.LC315
	.word	.LC316
	.word	.LC317
	.word	.LC318
	.word	.LC319
	.word	.LC320
	.word	.LC321
	.word	.LC322
	.word	1263358532
	.word	-1067903959
	.word	.LC323
	.word	.LC324
	.word	1112753220
	.word	1146313043
	.word	.LC325
	.word	1112755781
	.word	.LC326
	.word	1094995539
	.word	.LANCHOR5-2752
	.word	.LC327
	.word	1074031676
	.word	.LC328
	.word	.LC329
	.word	1280262987
	.word	.LC330
	.word	.LC331
	.word	1145980246
	.word	1074034192
	.word	.LC332
	.word	.LC333
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
	ldr	r4, .L4463
	mov	r2, #512
	mov	r5, #0
	sub	r0, r4, #2752
	ldr	r1, [r4, #-2804]
	str	r3, [r4, #-2776]
	add	r3, r1, #512
	str	r3, [r4, #-2768]
	str	r1, [r4, #-2756]
	add	r3, r1, #1024
	add	r1, r1, #1536
	str	r3, [r4, #-2236]
	strb	r5, [r4, #-2784]
	str	r5, [r4, #-2780]
	str	r5, [r4, #-2772]
	str	r1, [r4, #-2232]
	bl	ftl_memcpy
	ldr	r0, [r4, #-2756]
	str	r5, [r4, #-2764]
	str	r5, [r4, #-2240]
	ldr	r6, [r0, #508]
	ldr	r3, [r0, #16]
	cmp	r6, r5
	str	r3, [r4, #-2760]
	beq	.L4456
	mov	r1, #508
	bl	js_hash
	cmp	r6, r0
	beq	.L4456
	ldr	r3, [r4, #-2756]
	ldr	r0, .L4463+4
	str	r5, [r4, #-2760]
	str	r5, [r3, #16]
	bl	printk
.L4456:
	ldr	r3, [r4, #-2760]
	mov	r0, #2
	ldr	r1, [r4, #-2236]
	cmp	r3, #0
	ldrne	r2, .L4463+8
	ldrne	r3, .L4463
	strne	r2, [r3, #-2764]
	bl	StorageSysDataLoad
	ldr	r1, [r4, #-2232]
	mov	r0, #3
	bl	StorageSysDataLoad
	ldmfd	sp!, {r4, r5, r6, lr}
	b	rknand_sys_storage_init
.L4464:
	.align	2
.L4463:
	.word	.LANCHOR5
	.word	.LC334
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
	ldr	r5, .L4477
	cmp	r0, #0
	str	r0, [r5, #-2228]
	beq	.L4472
	ldr	r8, .L4477+4
	mov	r7, #0
	mov	r4, r7
	mov	r6, r7
	mov	r9, r5
.L4470:
	mov	r0, r6, asl #7
	mov	r1, #128
	ldr	r2, [r5, #-2228]
	bl	FlashBootVendorRead
	cmp	r0, #0
	bne	.L4468
	ldr	r3, [r9, #-2228]
	ldr	r0, .L4477+8
	add	r2, r3, #61440
	ldr	r1, [r3]
	ldr	r2, [r2, #4092]
	ldr	r3, [r3, #4]
	bl	printk
	ldr	r2, [r9, #-2228]
	ldr	r3, [r2]
	cmp	r3, r8
	bne	.L4469
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
.L4469:
	cmp	r6, #1
	movne	r6, #1
	bne	.L4470
.L4476:
	cmp	r4, #0
	beq	.L4471
	mov	r0, r7, asl #7
	mov	r1, #128
	bl	FlashBootVendorRead
	cmp	r0, #0
	bne	.L4468
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4471:
	mov	r0, r2
	mov	r1, #65536
	bl	__memzero
	ldr	r3, [r5, #-2228]
	ldr	r2, .L4477+4
	mov	r0, r4
	str	r6, [r3, #4]
	str	r2, [r3]
	add	r2, r3, #61440
	str	r6, [r2, #4092]
	ldr	r2, .L4477+12
	strh	r4, [r3, #12]	@ movhi
	strh	r2, [r3, #14]	@ movhi
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4468:
	ldr	r0, [r5, #-2228]
	bl	kfree
	mov	r3, #0
	mvn	r0, #0
	str	r3, [r5, #-2228]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4472:
	mvn	r0, #11
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, pc}
.L4478:
	.align	2
.L4477:
	.word	.LANCHOR5
	.word	1380668996
	.word	.LC335
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
	ldr	r3, .L4486
	ldr	ip, [r3, #-2228]
	cmp	ip, #0
	ldrneh	r4, [ip, #10]
	movne	r3, #0
	beq	.L4484
.L4481:
	cmp	r3, r4
	bcs	.L4484
	add	lr, ip, r3, asl #3
	ldrh	r5, [lr, #16]
	cmp	r5, r0
	bne	.L4482
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
.L4482:
	add	r3, r3, #1
	b	.L4481
.L4484:
	mvn	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L4487:
	.align	2
.L4486:
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
	ldr	r2, .L4511
	.pad #28
	sub	sp, sp, #28
	ldr	r4, [r2, #-2228]
	cmp	r4, #0
	beq	.L4503
	mov	r3, r1
	add	r7, r9, #63
	ldrh	r1, [r4, #8]
	bic	r7, r7, #63
	ldrh	r2, [r4, #10]
	mov	ip, r0
	mov	r6, #0
	str	r1, [sp, #4]
.L4490:
	cmp	r6, r2
	bcs	.L4509
	add	r5, r4, r6, asl #3
	ldrh	r1, [r5, #16]
	cmp	r1, ip
	bne	.L4491
	ldrh	r1, [r5, #20]
	add	fp, r4, #1024
	add	r1, r1, #63
	bic	r1, r1, #63
	str	r1, [sp, #8]
	cmp	r9, r1
	bls	.L4492
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	subcs	r2, r2, #1
	strcs	r2, [sp, #12]
	ldrcsh	r8, [r5, #18]
	bcc	.L4503
.L4493:
	ldr	r2, [sp, #12]
	add	r5, r5, #8
	cmp	r6, r2
	bcs	.L4510
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
	b	.L4493
.L4510:
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
	b	.L4508
.L4492:
	ldrh	r0, [r5, #18]
	mov	r1, r3
	mov	r2, r9
	add	r0, fp, r0
	bl	memcpy
	strh	r9, [r5, #20]	@ movhi
	b	.L4508
.L4491:
	add	r6, r6, #1
	b	.L4490
.L4509:
	ldrh	r1, [r4, #14]
	cmp	r1, r7
	bcc	.L4503
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
.L4508:
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
	b	.L4489
.L4503:
	mvn	r0, #0
.L4489:
	add	sp, sp, #28
	@ sp needed
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L4512:
	.align	2
.L4511:
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
	beq	.L4514
	ldr	r3, .L4530
	cmp	r6, r3
	beq	.L4516
	add	r3, r3, #1
	cmp	r6, r3
	beq	.L4517
	b	.L4528
.L4516:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4528
	ldr	r2, [r4]
	ldr	r3, .L4530+4
	cmp	r2, r3
	beq	.L4519
.L4520:
	mvn	r5, #0
	b	.L4515
.L4519:
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_read
	cmn	r0, #1
	beq	.L4520
	uxth	r2, r0
	strh	r0, [r4, #6]	@ movhi
	mov	r1, r4
	mov	r0, r5
	add	r2, r2, #8
	bl	rk_copy_to_user
	cmp	r0, #0
	moveq	r5, #0
	mvnne	r5, #13
	b	.L4515
.L4517:
	mov	r1, r5
	mov	r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4528
	ldr	r2, [r4]
	ldr	r3, .L4530+4
	cmp	r2, r3
	bne	.L4520
	ldrh	r2, [r4, #6]
	movw	r3, #4087
	cmp	r2, r3
	bhi	.L4520
	mov	r0, r4
	mov	r1, r5
	add	r2, r2, #8
	bl	rk_copy_from_user
	cmp	r0, #0
	bne	.L4528
	ldrh	r0, [r4, #4]
	add	r1, r4, #8
	ldrh	r2, [r4, #6]
	bl	rk_ftl_vendor_write
	mov	r5, r0
	b	.L4515
.L4528:
	mvn	r5, #13
.L4515:
	mov	r0, r4
	bl	kfree
.L4514:
	mov	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L4531:
	.align	2
.L4530:
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
	.global	g_maxRetryCount
	.global	g_maxRegNum
	.global	g_retryMode
	.global	g_flash_toggle_mode_en
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
	.type	__func__.36768, %object
	.size	__func__.36768, 18
__func__.36768:
	.ascii	"_list_remove_node\000"
	.space	2
	.type	__func__.36793, %object
	.size	__func__.36793, 23
__func__.36793:
	.ascii	"_list_update_data_list\000"
	.space	1
	.type	__func__.21434, %object
	.size	__func__.21434, 22
__func__.21434:
	.ascii	"nand_flash_print_info\000"
	.space	2
	.type	__func__.21520, %object
	.size	__func__.21520, 11
__func__.21520:
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
	.type	__func__.20990, %object
	.size	__func__.20990, 28
__func__.20990:
	.ascii	"flash_wait_device_ready_raw\000"
	.type	__func__.21424, %object
	.size	__func__.21424, 20
__func__.21424:
	.ascii	"flash_die_info_init\000"
	.type	__func__.21054, %object
	.size	__func__.21054, 22
__func__.21054:
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
	.type	__func__.21079, %object
	.size	__func__.21079, 23
__func__.21079:
	.ascii	"flash_start_plane_read\000"
	.space	1
	.type	__func__.20965, %object
	.size	__func__.20965, 26
__func__.20965:
	.ascii	"flash_erase_duplane_block\000"
	.space	2
	.type	__func__.20976, %object
	.size	__func__.20976, 21
__func__.20976:
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
	.type	__func__.21699, %object
	.size	__func__.21699, 13
__func__.21699:
	.ascii	"buf_add_tail\000"
	.space	3
	.type	__func__.21712, %object
	.size	__func__.21712, 10
__func__.21712:
	.ascii	"buf_alloc\000"
	.space	2
	.type	__func__.21726, %object
	.size	__func__.21726, 16
__func__.21726:
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
	.type	__func__.36337, %object
	.size	__func__.36337, 12
__func__.36337:
	.ascii	"gc_add_sblk\000"
	.type	__func__.36431, %object
	.size	__func__.36431, 19
__func__.36431:
	.ascii	"gc_write_completed\000"
	.space	1
	.type	__func__.36975, %object
	.size	__func__.36975, 18
__func__.36975:
	.ascii	"ftl_alloc_sys_blk\000"
	.space	2
	.type	__func__.36985, %object
	.size	__func__.36985, 17
__func__.36985:
	.ascii	"ftl_free_sys_blk\000"
	.space	3
	.type	__func__.37096, %object
	.size	__func__.37096, 23
__func__.37096:
	.ascii	"ftl_get_ppa_from_index\000"
	.space	1
	.type	__func__.37136, %object
	.size	__func__.37136, 22
__func__.37136:
	.ascii	"ftl_get_new_free_page\000"
	.space	2
	.type	__func__.37147, %object
	.size	__func__.37147, 22
__func__.37147:
	.ascii	"ftl_ext_alloc_new_blk\000"
	.space	2
	.type	__func__.36482, %object
	.size	__func__.36482, 16
__func__.36482:
	.ascii	"gc_free_src_blk\000"
	.type	__func__.36090, %object
	.size	__func__.36090, 14
__func__.36090:
	.ascii	"ftl_write_buf\000"
	.space	2
	.type	__func__.36135, %object
	.size	__func__.36135, 18
__func__.36135:
	.ascii	"zftl_add_read_buf\000"
	.space	2
	.type	__func__.36933, %object
	.size	__func__.36933, 15
__func__.36933:
	.ascii	"ftl_alloc_sblk\000"
	.space	1
	.type	__func__.37487, %object
	.size	__func__.37487, 13
__func__.37487:
	.ascii	"pm_free_sblk\000"
	.space	3
	.type	__func__.37523, %object
	.size	__func__.37523, 17
__func__.37523:
	.ascii	"pm_alloc_new_blk\000"
	.space	3
	.type	__func__.37551, %object
	.size	__func__.37551, 21
__func__.37551:
	.ascii	"pm_select_ram_region\000"
	.space	3
	.type	__func__.36025, %object
	.size	__func__.36025, 17
__func__.36025:
	.ascii	"lpa_rebuild_hash\000"
	.space	3
	.type	__func__.21750, %object
	.size	__func__.21750, 21
__func__.21750:
	.ascii	"flash_info_data_init\000"
	.space	3
	.type	__func__.20907, %object
	.size	__func__.20907, 19
__func__.20907:
	.ascii	"flash_read_page_en\000"
	.space	1
	.type	__func__.21067, %object
	.size	__func__.21067, 31
__func__.21067:
	.ascii	"flash_complete_plane_page_read\000"
	.space	1
	.type	__func__.21092, %object
	.size	__func__.21092, 25
__func__.21092:
	.ascii	"flash_complete_page_read\000"
	.space	3
	.type	__func__.37332, %object
	.size	__func__.37332, 31
__func__.37332:
	.ascii	"queue_wait_first_req_completed\000"
	.space	1
	.type	__func__.37413, %object
	.size	__func__.37413, 15
__func__.37413:
	.ascii	"sblk_read_page\000"
	.space	1
	.type	__func__.21014, %object
	.size	__func__.21014, 26
__func__.21014:
	.ascii	"flash_start_tlc_page_prog\000"
	.space	2
	.type	__func__.21025, %object
	.size	__func__.21025, 29
__func__.21025:
	.ascii	"flash_start_3d_mlc_page_prog\000"
	.space	3
	.type	__func__.21043, %object
	.size	__func__.21043, 22
__func__.21043:
	.ascii	"flash_start_page_prog\000"
	.space	2
	.type	__func__.37386, %object
	.size	__func__.37386, 15
__func__.37386:
	.ascii	"sblk_prog_page\000"
	.space	1
	.type	__func__.20949, %object
	.size	__func__.20949, 19
__func__.20949:
	.ascii	"flash_prog_page_en\000"
	.space	1
	.type	__func__.37210, %object
	.size	__func__.37210, 14
__func__.37210:
	.ascii	"ftl_prog_page\000"
	.space	2
	.type	__func__.36901, %object
	.size	__func__.36901, 15
__func__.36901:
	.ascii	"ftl_info_flush\000"
	.space	1
	.type	__func__.37172, %object
	.size	__func__.37172, 19
__func__.37172:
	.ascii	"ftl_ext_info_flush\000"
	.space	1
	.type	__func__.37184, %object
	.size	__func__.37184, 18
__func__.37184:
	.ascii	"ftl_ext_info_init\000"
	.space	2
	.type	__func__.37066, %object
	.size	__func__.37066, 23
__func__.37066:
	.ascii	"ftl_open_sblk_recovery\000"
	.space	1
	.type	__func__.37533, %object
	.size	__func__.37533, 14
__func__.37533:
	.ascii	"pm_write_page\000"
	.space	2
	.type	__func__.21772, %object
	.size	__func__.21772, 17
__func__.21772:
	.ascii	"flash_info_flush\000"
	.space	3
	.type	__func__.21739, %object
	.size	__func__.21739, 20
__func__.21739:
	.ascii	"flash_info_blk_init\000"
	.type	__func__.21479, %object
	.size	__func__.21479, 16
__func__.21479:
	.ascii	"nand_flash_init\000"
	.type	__func__.37044, %object
	.size	__func__.37044, 16
__func__.37044:
	.ascii	"ftl_sysblk_dump\000"
	.type	__func__.37473, %object
	.size	__func__.37473, 16
__func__.37473:
	.ascii	"load_l2p_region\000"
	.type	__func__.37507, %object
	.size	__func__.37507, 6
__func__.37507:
	.ascii	"pm_gc\000"
	.space	2
	.type	__func__.36418, %object
	.size	__func__.36418, 22
__func__.36418:
	.ascii	"gc_update_l2p_map_new\000"
	.space	2
	.type	__func__.36526, %object
	.size	__func__.36526, 16
__func__.36526:
	.ascii	"gc_scan_src_blk\000"
	.type	__func__.36584, %object
	.size	__func__.36584, 20
__func__.36584:
	.ascii	"gc_scan_static_data\000"
	.type	__func__.36647, %object
	.size	__func__.36647, 18
__func__.36647:
	.ascii	"gc_block_vpn_scan\000"
	.space	2
	.type	__func__.37020, %object
	.size	__func__.37020, 14
__func__.37020:
	.ascii	"ftl_sblk_dump\000"
	.space	2
	.type	__func__.36168, %object
	.size	__func__.36168, 10
__func__.36168:
	.ascii	"zftl_read\000"
	.space	2
	.type	__func__.36384, %object
	.size	__func__.36384, 21
__func__.36384:
	.ascii	"gc_check_data_one_wl\000"
	.space	3
	.type	__func__.37117, %object
	.size	__func__.37117, 19
__func__.37117:
	.ascii	"ftl_update_l2p_map\000"
	.space	1
	.type	__func__.36072, %object
	.size	__func__.36072, 17
__func__.36072:
	.ascii	"ftl_write_commit\000"
	.space	3
	.type	__func__.36460, %object
	.size	__func__.36460, 16
__func__.36460:
	.ascii	"gc_do_copy_back\000"
	.type	__func__.36675, %object
	.size	__func__.36675, 11
__func__.36675:
	.ascii	"zftl_do_gc\000"
	.space	1
	.type	__func__.36199, %object
	.size	__func__.36199, 13
__func__.36199:
	.ascii	"_ftl_discard\000"
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"\012!!!!! error @ func:%s - line:%d\012\000"
.LC1:
	.ascii	"%s\012\000"
.LC2:
	.ascii	"FTL version: 6.0.16 20190603\000"
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
	.ascii	"gc_add_sblk = %d, %d, %d, %d, %d, %d\012\000"
.LC88:
	.ascii	"gc_add_sblk = %d, %d, %d,last update:%d, %d\012\000"
.LC89:
	.ascii	"gc_mark_bad_ppa %d %x %x\012\000"
.LC90:
	.ascii	"status: %x, ppa: %x\012\000"
.LC91:
	.ascii	"gc_write_completed: %x  %x  %x %x\012\000"
.LC92:
	.ascii	"zftl_get_gc_node cache = %x index = %d vpn = %x\012"
	.ascii	"\000"
.LC93:
	.ascii	"gc_search_src_blk mode = %x, src mode = %x, count= "
	.ascii	"%d %d\012\000"
.LC94:
	.ascii	"%d gc_free_temp_buf buf id= %x\012\000"
.LC95:
	.ascii	"gc: b:%x,p:%x,i:%x; free buf=%d %d free slc th: %d\012"
	.ascii	"\000"
.LC96:
	.ascii	"swl_tlc_free_mini_ec_blk alloc sblk %x\012\000"
.LC97:
	.ascii	"zftl_get_free_sblk %x %d, %p %d %d\012\000"
.LC98:
	.ascii	"zftl_gc_get_free_sblk %x %x %x, %d %d %d\012\000"
.LC99:
	.ascii	"swl_slc_free_mini_ec_blk alloc sblk %x\012\000"
.LC100:
	.ascii	"list count:%p %d\012\000"
.LC101:
	.ascii	"%d: node:%x %x %x %x, %d %d %d %d %d\012\000"
.LC102:
	.ascii	"gc_static_wearleveling: min blk: %x,sec=%d,xec = %d"
	.ascii	" ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC103:
	.ascii	"gc_static_wearleveling: min slc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC104:
	.ascii	"gc_static_wearleveling: min tlc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC105:
	.ascii	"gc_static_wearleveling: max slc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC106:
	.ascii	"gc_static_wearleveling: max xlc blk: %x,sec=%d,xec "
	.ascii	"= %d ,mode=%d, func=%x, bbt=%x vpn = %d\012\000"
.LC107:
	.ascii	"gc_static_wearleveling: slc blk: %x, tlc blk: %d av"
	.ascii	"g slc ec: %d, avg tlc ec: %d \012\000"
.LC108:
	.ascii	"gc_static_wearleveling: min slc ec: %x, min tlc ec:"
	.ascii	" %d max slc ec: %d, max tlc ec: %d; %d %d\012\000"
.LC109:
	.ascii	"swl add tlc gc = %x, %d, %d, %d, %d, %d\012\000"
.LC110:
	.ascii	"swl add slc gc  = %x, %d, %d, %d, %d, %d\012\000"
.LC111:
	.ascii	"ftl_vpn_decrement %x = %d, %d\012\000"
.LC112:
	.ascii	"mask bad block:cs %x %x block: %x %x\012\000"
.LC113:
	.ascii	"gc_free_bad_sblk 0x%x\012\000"
.LC114:
	.ascii	"swl_tlc_free_mini_ec_blk sblk %x\012\000"
.LC115:
	.ascii	"swl_slc_free_mini_ec_blk sblk %x\012\000"
.LC116:
	.ascii	"gc_free_src_blk = %x, vpn = %d\012\000"
.LC117:
	.ascii	"gc_free_src_blk %x, %d\012\000"
.LC118:
	.ascii	"bad blk = %x, %x free blk: s:%x,t:%x,m:%x, data blk"
	.ascii	":s:%x,%x,t%x vpn: s:%x t:%x, max_vpn: %x\012\000"
.LC119:
	.ascii	"totle w: %d MB,r: %d MB %d dv:0x%X,poc:%d\012\000"
.LC120:
	.ascii	"gc xlc page: %d,gc slc page: %d, tmp w: %d MB\012\000"
.LC121:
	.ascii	"slc ec: %d,%d,%d,%d,%d,tlc ec: %d,%d,%d,%d,%d\012\000"
.LC122:
	.ascii	"gc th: tlc_tlc: %d tlc_slc: %d slc_slc: %d slc_tlc:"
	.ascii	"%d free_th: %d\012\000"
.LC123:
	.ascii	"swl : %x %x %x %x\012\000"
.LC124:
	.ascii	"ftl prog error =%x, lpa = %x, ppa= %x\012\000"
.LC125:
	.ascii	"ftl re prog: lpa = %x, ppa= %x\012\000"
.LC126:
	.ascii	"alloc sblk %x %d\012\000"
.LC127:
	.ascii	"dump_sblk_queue: %d\012\000"
.LC128:
	.ascii	"buf id= %d state = %d ppa = %x\012\000"
.LC129:
	.ascii	"ftl_free_no_use_map_blk %x %x %x %d\012\000"
.LC130:
	.ascii	"blk %x is bad block\012\000"
.LC131:
	.ascii	"pm_alloc_new_blk: %x %x %x %x\012\000"
.LC132:
	.ascii	"%s %d %d\012\000"
.LC133:
	.ascii	"free blk vpn error: %x %x\012\000"
.LC134:
	.ascii	"...%d @ %s\012\000"
.LC135:
	.ascii	"rk_ftl_de_init %x\012\000"
.LC136:
	.ascii	"%s %p:0x%x:\000"
.LC137:
	.ascii	"%x \000"
.LC138:
	.ascii	"\000"
.LC139:
	.ascii	"otp error! %d\000"
.LC140:
	.ascii	"rr\000"
.LC141:
	.ascii	"flash_abort_clear = %d\012\000"
.LC142:
	.ascii	"%d mtrans_cnt = %d page_num = %d\012\000"
.LC143:
	.ascii	"nandc:\000"
.LC144:
	.ascii	"%d flReg.d32=%x %x\012\000"
.LC145:
	.ascii	"nandc_xfer_done read error %x\012\000"
.LC146:
	.ascii	"dqs data abort %x\012\000"
.LC147:
	.ascii	"dqs data timeout %x\012\000"
.LC148:
	.ascii	"xfer error %x\012\000"
.LC149:
	.ascii	"flash_read_page %x %x %x\012\000"
.LC150:
	.ascii	"micron %d row=%x,last status %d,status = %d\012\000"
.LC151:
	.ascii	"micron RR %d row=%x,count %d,status=%d\012\000"
.LC152:
	.ascii	"toshiba SRR %d row=%x, status=%d\012\000"
.LC153:
	.ascii	"toshiba TRR %d row=%x, status=%d\012\000"
.LC154:
	.ascii	"toshiba RR %d row=%x,count %d,status=%d\012\000"
.LC155:
	.ascii	"hynix RR %d row=%x, count %d, status=%d\012\000"
.LC156:
	.ascii	"%d flash_ddr_tuning_read %x ecc=%d\012\000"
.LC157:
	.ascii	"sync para %d\012\000"
.LC158:
	.ascii	"DDR mode Read error %x %x\012\000"
.LC159:
	.ascii	"SDR mode Read %x %x ecc:%x\012\000"
.LC160:
	.ascii	"flash_read_page_en %x %x %x\012\000"
.LC161:
	.ascii	"flash_read_page_en %x %x error_ecc %d %d\012\000"
.LC162:
	.ascii	"flash_get_last_written_page: %x %x %x\012\000"
.LC163:
	.ascii	"read page: %x %x %x %x\012\000"
.LC164:
	.ascii	"flash_complete_page_read %x %x error_ecc %d %d\012\000"
.LC165:
	.ascii	"read: %x %x %x %x\012\000"
.LC166:
	.ascii	"0set buf %d,status = %x, ppa = %x lun state = %d\012"
	.ascii	"\000"
.LC167:
	.ascii	"prog end %x %x error_ecc %d %d\012\000"
.LC168:
	.ascii	"1set buf %d,status = %x, ppa = %x lun state = %d\012"
	.ascii	"\000"
.LC169:
	.ascii	"dp prog end %x %x error_ecc %d %d\012\000"
.LC170:
	.ascii	"flash_prog_page %x %x %x\012\000"
.LC171:
	.ascii	"flash_prog_page page_addr = %x status = %x\012\000"
.LC172:
	.ascii	"sblk_3d_tlc_dump_prog wl_addr= %x ppa = %x ppa = %x"
	.ascii	"\012\000"
.LC173:
	.ascii	"sblk_mlc_dump_prog wl_addr= %x ppa = %x ppa = %x\012"
	.ascii	"\000"
.LC174:
	.ascii	"sblk_prog_page ppa = %x, count = %d\012\000"
.LC175:
	.ascii	"flash_prog_page_en:%x %x %x\012\000"
.LC176:
	.ascii	"w d:\000"
.LC177:
	.ascii	"w s:\000"
.LC178:
	.ascii	"spare\000"
.LC179:
	.ascii	"data\000"
.LC180:
	.ascii	"write error: %x\012\000"
.LC181:
	.ascii	"g_ftl_info_blk blk = %x, page = %x version = %d\012"
	.ascii	"\000"
.LC182:
	.ascii	"%d %x @%d %x\012\000"
.LC183:
	.ascii	"ftl_info_blk_init %d %d %x\012\000"
.LC184:
	.ascii	"ink flag: %x\012\000"
.LC185:
	.ascii	"%s %d %d %x %x\012\000"
.LC186:
	.ascii	"saved_active_page  = %x\012\000"
.LC187:
	.ascii	"saved_active_plane = %x\012\000"
.LC188:
	.ascii	"sblk = %x\012\000"
.LC189:
	.ascii	"phy_blk = %x %x\012\000"
.LC190:
	.ascii	"sblk = %x, vpn0 = %d, vpn1 = %d\012\000"
.LC191:
	.ascii	"dump_write_lpa = %x %x %x %x\012\000"
.LC192:
	.ascii	"dump write new ppa = %x, last ppa = %x lpa = %x\012"
	.ascii	"\000"
.LC193:
	.ascii	"dump write = %x %x\012\000"
.LC194:
	.ascii	"dump write hash update = %x %x %x\012\000"
.LC195:
	.ascii	"pm_write_page write error: %x\012\000"
.LC196:
	.ascii	"ftl_sblk_dump_write = %x %d %d %d %d\012\000"
.LC197:
	.ascii	"blk= %x, page=%x, ppa = %x, status = %x, data:%x %x"
	.ascii	" %x %x, spare: %x %x %x %x\012\000"
.LC198:
	.ascii	"ftl_sblk_dump_write2 = %x %d %d %d\012\000"
.LC199:
	.ascii	"ftl_sblk_dump_write = %x %x\012\000"
.LC200:
	.ascii	"ftl_sblk_dump_write done = %x\012\000"
.LC201:
	.ascii	"finfo:\000"
.LC202:
	.ascii	"flash_info_flush id = %x, page = %x\012\000"
.LC203:
	.ascii	"sys_info_flush error:%x\012\000"
.LC204:
	.ascii	"...%d @ %s %d %p\012\000"
.LC205:
	.ascii	"no sys info %x\012\000"
.LC206:
	.ascii	"l2p:\000"
.LC207:
	.ascii	"free_buf_count: %d\012\000"
.LC208:
	.ascii	"g_ftl_info_blk blk:0x%x, index:0x%x, page:0x%x\012\000"
.LC209:
	.ascii	"ftl_ext_info_blk blk:0x%x, page:0x%x\012\000"
.LC210:
	.ascii	"ac_blk:0x%x, page:0x%x, index:0x%x, free:0x%x, page"
	.ascii	"_index:0x%x\012\000"
.LC211:
	.ascii	"tmp_blk:0x%x, page:0x%x, index:0x%x, free:0x%x, pag"
	.ascii	"e_index:0x%x\012\000"
.LC212:
	.ascii	"gc_blk:0x%x, page:0x%x, index:0x%x, free:0x%x, page"
	.ascii	"_index:0x%x\012\000"
.LC213:
	.ascii	"lpa:\000"
.LC214:
	.ascii	"vpn:\000"
.LC215:
	.ascii	"sblk:\000"
.LC216:
	.ascii	"lpa_hash:\000"
.LC217:
	.ascii	"lpa_hash_index:\000"
.LC218:
	.ascii	"%s w error lpn = %x, max ppa = %d\012\000"
.LC219:
	.ascii	"region_id = %d, pm_max_region = %d\012\000"
.LC220:
	.ascii	"load_l2p_region no ppa = %x , %x, all setting 0xff."
	.ascii	"...\012\000"
.LC221:
	.ascii	"load_l2p_region = %x,%x,%x, %x\012\000"
.LC222:
	.ascii	"pm_ppa:\000"
.LC223:
	.ascii	"data:\000"
.LC224:
	.ascii	"spare:\000"
.LC225:
	.ascii	"pm_init posr %x %x %x\012\000"
.LC226:
	.ascii	"pm_init recovery %x %x %x\012\000"
.LC227:
	.ascii	"pm_log2phys  lpn = %d, max lpn = %d\012\000"
.LC228:
	.ascii	"ppa = %x, status = %x, data:%x %x %x %x, spare: %x "
	.ascii	"%x %x %x\012\000"
.LC229:
	.ascii	"ppa = %x, status = %x, %x %x spare: %x %x %x %x\012"
	.ascii	"\000"
.LC230:
	.ascii	"gc_recovery: %x vpn = %x\012\000"
.LC231:
	.ascii	"_c_user_data_density := %d\012\000"
.LC232:
	.ascii	"_c_totle_phy_density := %d\012\000"
.LC233:
	.ascii	"_c_totle_log_page := %d\012\000"
.LC234:
	.ascii	"_c_totle_data_density := %d\012\000"
.LC235:
	.ascii	"_c_ftl_pm_page_num := %d\012\000"
.LC236:
	.ascii	"_c_ftl_byte_pre_page := %d\012\000"
.LC237:
	.ascii	"_c_max_pm_sblk := %d\012\000"
.LC238:
	.ascii	"_min_slc_super_block := %d\012\000"
.LC239:
	.ascii	"_max_xlc_super_block := %d\012\000"
.LC240:
	.ascii	"gp_ftl_ext_info %p %p %p\012\000"
.LC241:
	.ascii	"flash info size: %d %d %d\012\000"
.LC242:
	.ascii	"ftl_init %x\012\000"
.LC243:
	.ascii	"gc_update_l2p_map_new sblk %x\012\000"
.LC244:
	.ascii	"gc_update_l2p_map_new: %x %x %x\012\000"
.LC245:
	.ascii	"lpa: %x %x %x\012\000"
.LC246:
	.ascii	"gc_update_l2p_map_new: %x vpn = %x vpn1 = %x done\012"
	.ascii	"\000"
.LC247:
	.ascii	"gc_scan_src_blk = %x, vpn = %d\012\000"
.LC248:
	.ascii	"gc_scan_src_blk = %x, s vpn0 = %d, c vpn1 = %d\012\000"
.LC249:
	.ascii	"gc_block_vpn_scan = %x, s vpn0 = %d, c vpn1 = %d f:"
	.ascii	"%d\012\000"
.LC250:
	.ascii	"ftl_sblk_dump = %x %d %d %d %d\012\000"
.LC251:
	.ascii	"ftl_sblk_dump = %x %x %x %x\012\000"
.LC252:
	.ascii	"page_addr = %x, lpa=%x vpn = %d\012\000"
.LC253:
	.ascii	"index= %x, lpa=%x\012\000"
.LC254:
	.ascii	"block = %x, vpn=%x check vpn = %d\012\000"
.LC255:
	.ascii	"ftl_read %x %x %x\012\000"
.LC256:
	.ascii	"ftl_read refresh =%x, lpa = %x, ppa= %x\012\000"
.LC257:
	.ascii	"id=%d, status = %x, lpa = %x, ppa = %x spare = %x %"
	.ascii	"x %x %x\012\000"
.LC258:
	.ascii	"zftl debug cmd: %s\012\000"
.LC259:
	.ascii	"cmd:\000"
.LC260:
	.ascii	"dumpl2p\000"
.LC261:
	.ascii	"pm l2p:\000"
.LC262:
	.ascii	"pm blk:\000"
.LC263:
	.ascii	"dumppm:\000"
.LC264:
	.ascii	"p_cmd: %s\012\000"
.LC265:
	.ascii	"pm ram = %x, %x\012\000"
.LC266:
	.ascii	"ram:\000"
.LC267:
	.ascii	"pm:\000"
.LC268:
	.ascii	"dumpsys\000"
.LC269:
	.ascii	"dumplist:\000"
.LC270:
	.ascii	"vpncheck\000"
.LC271:
	.ascii	"dumpppa:\000"
.LC272:
	.ascii	"dumpblk:\000"
.LC273:
	.ascii	"setzdebug:\000"
.LC274:
	.ascii	"lpa2ppa:\000"
.LC275:
	.ascii	"lpa: %x--> ppa: %x\012\000"
.LC276:
	.ascii	"help:\012\000"
.LC277:
	.ascii	"1. echo dumpl2p > /proc/zftl_debug\012\000"
.LC278:
	.ascii	"2. echo dumppm:x > /proc/zftl_debug\012\000"
.LC279:
	.ascii	"3. echo dumpsys > /proc/zftl_debug\012\000"
.LC280:
	.ascii	"4. echo dumpppa:x > /proc/zftl_debug\012\000"
.LC281:
	.ascii	"5. echo vpncheck > /proc/zftl_debug\012\000"
.LC282:
	.ascii	"6. echo setzdebug:x > /proc/zftl_debug\012\000"
.LC283:
	.ascii	"7. echo dumplist > /proc/zftl_debug\012\000"
.LC284:
	.ascii	"8. echo lpa2ppa:x> /proc/zftl_debug\012\000"
.LC285:
	.ascii	"gc_lpa:\000"
.LC286:
	.ascii	"gc_ppa:\000"
.LC287:
	.ascii	"err: ppa = %x, status = %x, %x %x spare: %x %x %x %"
	.ascii	"x\012\000"
.LC288:
	.ascii	"ftl_update_l2p_map: %x %x %x\012\000"
.LC289:
	.ascii	"ftl_update_l2p_map\000"
.LC290:
	.ascii	"lpa_tbl:\000"
.LC291:
	.ascii	"sblk %x vpn: %d %d\012\000"
.LC292:
	.ascii	"error gc_add_sblk: %x\012\000"
.LC293:
	.ascii	"%d read error: ppa:%x, lpa:%x, status:%x\012\000"
.LC294:
	.ascii	"gc page in buf: lpa %x ppa = %x pageindex= %x\012\000"
.LC295:
	.ascii	"gc_do_copy_back: lpa %x des_ppa = %x %x gc_ppa= %x "
	.ascii	"page_index= %d\012\000"
.LC296:
	.ascii	"gc_do_copy_back: %x %x %x %x gc_ppa:%x %x\012\000"
.LC297:
	.ascii	"%d prog_step: %x %x buf id= %x ppa = %x hash=%x id "
	.ascii	"= %x plane = %x lpa=%x\012\000"
.LC298:
	.ascii	"gc %d: %d %d %d %d %d %d %d\012\000"
.LC299:
	.ascii	"gc %d: %d %d %d %d %d %d\012\000"
.LC300:
	.ascii	"GC_STATE_SCAN_ALL_PAGE = %x, vpn0 = %d, vpn1 = %d\012"
	.ascii	"\000"
.LC301:
	.ascii	"gc free %x, %d\012\000"
.LC302:
	.ascii	"ftlwrite %x %x %x %x\012\000"
.LC303:
	.ascii	"ftl_discard:(%x, %x, %x, %x)\012\000"
.LC304:
	.ascii	"id_block_prog_msb_ff_data slc page = %d pageadd=%x "
	.ascii	"%x\012\000"
.LC305:
	.ascii	"write_idblock fix data %x %x\012\000"
.LC306:
	.ascii	"idblk:\000"
.LC307:
	.ascii	"write_idblock totle_sec %x %x\012\000"
.LC308:
	.ascii	"prog page: %x %x %x, %p %x %x %x\012\000"
.LC309:
	.ascii	"wl_lba %p %x %x %x\012\000"
.LC310:
	.ascii	"READ_SECTOR_IO\012\000"
.LC311:
	.ascii	"rk_copy_from_user error\012\000"
.LC312:
	.ascii	"READ_SECTOR_IO %x %x\012\000"
.LC313:
	.ascii	"rk_copy_to_user error\012\000"
.LC314:
	.ascii	"WRITE_SECTOR_IO\012\000"
.LC315:
	.ascii	"WRITE_SECTOR_IO %x %x\012\000"
.LC316:
	.ascii	"END_WRITE_SECTOR_IO\012\000"
.LC317:
	.ascii	"END_WRITE_SECTOR_IO %x %x\012\000"
.LC318:
	.ascii	"GET_FLASH_INFO_IO\012\000"
.LC319:
	.ascii	"GET_BAD_BLOCK_IO\012\000"
.LC320:
	.ascii	"GET_LOCK_FLAG_IO\012\000"
.LC321:
	.ascii	"GET_PUBLIC_KEY_IO\012\000"
.LC322:
	.ascii	"RKNAND_GET_DRM_KEY\012\000"
.LC323:
	.ascii	"RKNAND_STORE_DRM_KEY\012\000"
.LC324:
	.ascii	"RKNAND_DIASBLE_SECURE_BOOT\012\000"
.LC325:
	.ascii	"RKNAND_ENASBLE_SECURE_BOOT\012\000"
.LC326:
	.ascii	"RKNAND_GET_SN_SECTOR\012\000"
.LC327:
	.ascii	"RKNAND_LOADER_UNLOCK\012\000"
.LC328:
	.ascii	"RKNAND_LOADER_STATUS\012\000"
.LC329:
	.ascii	"RKNAND_LOADER_LOCK\012\000"
.LC330:
	.ascii	"LockKey not match %d\012\000"
.LC331:
	.ascii	"RKNAND_GET_VENDOR_SECTOR\012\000"
.LC332:
	.ascii	"RKNAND_STORE_VENDOR_SECTOR\012\000"
.LC333:
	.ascii	"return ret = %lx\012\000"
.LC334:
	.ascii	"secureBootEn check error\012\000"
.LC335:
	.ascii	"\0013vendor storage %x,%x,%x\012\000"
	.data
	.align	2
.LANCHOR2 = . + 0
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
	.size	zftl_nand_flash_para_tbl, 1248
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
	.byte	0
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
	.byte	2
	.byte	-65
	.byte	-66
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
	.byte	4
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
	.byte	4
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
	.type	g_flash_interface_mode, %object
	.size	g_flash_interface_mode, 1
g_flash_interface_mode:
	.space	1
	.type	g_flash_multi_page_prog_en, %object
	.size	g_flash_multi_page_prog_en, 1
g_flash_multi_page_prog_en:
	.space	1
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
	.size	g_gc_info, 340
g_gc_info:
	.space	340
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
	.space	3
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
	.space	7
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
	.space	1
	.type	_c_ftl_byte_pre_page, %object
	.size	_c_ftl_byte_pre_page, 2
_c_ftl_byte_pre_page:
	.space	2
	.type	pm_last_load_ram_id, %object
	.size	pm_last_load_ram_id, 1
pm_last_load_ram_id:
	.space	1
	.space	3
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
