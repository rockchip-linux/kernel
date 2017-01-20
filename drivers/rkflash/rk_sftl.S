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
	.file	"rk_sftl.c"
	.text
.Ltext0:
	.global	__aeabi_uidiv
	.global	__aeabi_uidivmod
	.align	2
	.global	sftl_var_init
	.type	sftl_var_init, %function
sftl_var_init:
	.fnstart
.LFB124:
	.file 1 "drivers/rksftl/sftl.c"
	.loc 1 6 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	.loc 1 9 0
	ldr	r3, .L5
	.loc 1 6 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
.LCFI0:
	.loc 1 9 0
	ldrh	r2, [r3, #20]
	ldr	r4, .L5+4
	.loc 1 12 0
	ldrh	r6, [r3, #14]
	.loc 1 9 0
	add	r1, r2, #2
	.loc 1 10 0
	add	r2, r2, #14
	.loc 1 14 0
	ldrh	r7, [r3, #2]
	.loc 1 16 0
	ldrh	r5, [r3, #6]
	.loc 1 10 0
	uxth	r2, r2
	strh	r2, [r4, #2]	@ movhi
	.loc 1 11 0
	strh	r2, [r4, #4]	@ movhi
	.loc 1 12 0
	mov	r2, r6, asl #9
	str	r2, [r4, #8]
	.loc 1 14 0
	ldrh	r2, [r3, #4]
	.loc 1 16 0
	strh	r5, [r4, #18]	@ movhi
	.loc 1 9 0
	strh	r1, [r4, #0]	@ movhi
	.loc 1 14 0
	mul	r7, r7, r2
	.loc 1 17 0
	ldrh	r1, [r3, #12]
	.loc 1 15 0
	strh	r2, [r4, #16]	@ movhi
	.loc 1 13 0
	strh	r6, [r4, #12]	@ movhi
	.loc 1 17 0
	strh	r1, [r4, #20]	@ movhi
	.loc 1 14 0
	uxth	r7, r7
	strh	r7, [r4, #14]	@ movhi
	.loc 1 20 0
	mul	r5, r5, r7
	.loc 1 18 0
	mul	r3, r1, r7
	.loc 1 24 0
	mov	r7, r7, asl #4
	uxth	r7, r7
	strh	r7, [r4, #36]	@ movhi
	.loc 1 20 0
	uxth	r5, r5
	strh	r5, [r4, #24]	@ movhi
	.loc 1 21 0
	mul	r5, r5, r1
	.loc 1 18 0
	strh	r3, [r4, #22]	@ movhi
	.loc 1 22 0
	ubfx	r0, r5, #9, #16
	strh	r0, [r4, #32]	@ movhi
	.loc 1 23 0
	strh	r0, [r4, #34]	@ movhi
	.loc 1 21 0
	str	r5, [r4, #28]
	.loc 1 25 0
	bl	__aeabi_uidiv
	.loc 1 27 0
	mul	r6, r6, r5
.LBB79:
	.loc 1 30 0 discriminator 3
	mov	r1, #2064
.LBE79:
	.loc 1 28 0
	sub	r5, r5, #1
	str	r5, [r4, #44]
	.loc 1 37 0
	ldr	r5, .L5+8
	.loc 1 27 0
	sub	r6, r6, #1
	str	r6, [r4, #40]
	.loc 1 25 0
	rsb	r0, r0, r7
	strh	r0, [r4, #38]	@ movhi
.LBB80:
	.loc 1 30 0 discriminator 3
	add	r0, r4, #48
	bl	__memzero
.LBE80:
.LBB81:
	.loc 1 31 0 discriminator 3
	add	r0, r4, #2112
	mov	r1, #2064
	bl	__memzero
.LBE81:
.LBB82:
	.loc 1 32 0 discriminator 3
	ldr	r0, .L5+12
	mov	r1, #20
	bl	__memzero
.LBE82:
.LBB83:
	.loc 1 33 0 discriminator 3
	mov	r1, #12
	ldr	r0, .L5+16
	bl	__memzero
.LBE83:
	.loc 1 35 0
	ldr	r3, .L5+20
	ldr	r2, .L5+24
	mov	r1, #0
	.loc 1 36 0 discriminator 1
	ldrh	r7, [r4, #14]
	.loc 1 37 0
	ldrh	r6, [r4, #16]
	.loc 1 36 0
	mov	r4, #1
.LVL0:
	.loc 1 35 0
	strh	r1, [r2, r3]	@ movhi
	.loc 1 36 0
	b	.L2
.L3:
	.loc 1 37 0
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidiv
	.loc 1 38 0
	mov	r1, r6
	.loc 1 37 0
	mov	r8, r0, asl #14
	.loc 1 38 0
	mov	r0, r4
	bl	__aeabi_uidivmod
	.loc 1 36 0 discriminator 2
	add	r4, r4, #1
	.loc 1 37 0
	add	r1, r8, r1
	strh	r1, [r5, #2]!	@ movhi
.L2:
	.loc 1 36 0 discriminator 1
	cmp	r4, r7
	bcc	.L3
	.loc 1 39 0
	ldr	r4, .L5+24
.LVL1:
	mov	r6, #0
	.loc 1 48 0
	ldr	r5, .L5+4
.LBB84:
	.loc 1 47 0 discriminator 3
	mov	r1, #24
	ldr	r0, .L5+28
.LBE84:
	.loc 1 39 0
	str	r6, [r4, #-3968]
	.loc 1 40 0
	str	r6, [r4, #-3964]
	.loc 1 42 0
	strb	r6, [r4, #-3960]
	.loc 1 43 0
	strb	r6, [r4, #-3959]
	.loc 1 44 0
	strb	r6, [r4, #-3958]
	.loc 1 45 0
	strb	r6, [r4, #-3957]
.LBB85:
	.loc 1 47 0 discriminator 3
	bl	__memzero
.LBE85:
	.loc 1 48 0
	ldrh	r2, [r5, #18]
	ldrh	r3, [r5, #4]
	.loc 1 55 0
	mov	r0, #128
	.loc 1 48 0
	rsb	r3, r3, r2
	ldrh	r2, [r5, #36]
	rsb	r3, r2, r3
	ldr	r2, .L5+32
	uxth	r3, r3
	sub	r1, r3, #40
	strh	r1, [r4, r2]	@ movhi
	.loc 1 49 0
	add	r2, r2, #2
	sub	r1, r3, #30
	.loc 1 50 0
	sub	r3, r3, #20
	.loc 1 49 0
	strh	r1, [r4, r2]	@ movhi
	.loc 1 50 0
	add	r2, r2, #2
	strh	r3, [r4, r2]	@ movhi
	.loc 1 51 0
	add	r2, r2, #8
	mvn	r3, #0
	strh	r3, [r4, r2]	@ movhi
	.loc 1 52 0
	add	r2, r2, #2
	strh	r6, [r4, r2]	@ movhi
	.loc 1 53 0
	add	r2, r2, #2
	strh	r3, [r4, r2]	@ movhi
	.loc 1 55 0
	bl	ftl_malloc
.LBB86:
	.loc 1 56 0 discriminator 4
	mov	r2, #128
	mov	r1, #255
.LBE86:
	.loc 1 55 0
	str	r0, [r4, #-3948]
.LBB87:
	.loc 1 56 0 discriminator 4
	bl	memset
.LBE87:
	.loc 1 57 0
	ldrh	r0, [r5, #22]
	mov	r0, r0, asl #2
	bl	ftl_malloc
.LBB88:
	.loc 1 58 0
	ldrh	r2, [r5, #22]
	movs	r2, r2, asl #2
.LVL2:
.LBE88:
	.loc 1 57 0
	str	r0, [r4, #-3936]
.LBB89:
	.loc 1 58 0
	beq	.L4
	.loc 1 58 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL3:
.L4:
.LBE89:
	.loc 1 60 0 is_stmt 1
	mov	r0, #0
	str	r0, [r4, #-3932]
	.loc 1 61 0
	str	r0, [r4, #-3928]
	.loc 1 64 0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L6:
	.align	2
.L5:
	.word	g_nand_phy_info
	.word	.LANCHOR0
	.word	.LANCHOR1-3976
	.word	.LANCHOR1-4008
	.word	.LANCHOR1-3988
	.word	-3976
	.word	.LANCHOR1
	.word	.LANCHOR1-3956
	.word	-3956
.LFE124:
	.fnend
	.size	sftl_var_init, .-sftl_var_init
	.align	2
	.global	sftl_mem_init
	.type	sftl_mem_init, %function
sftl_mem_init:
	.fnstart
.LFB125:
	.loc 1 67 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
.LCFI1:
	.loc 1 70 0
	ldr	r4, .L15
	ldr	r5, .L15+4
	ldr	r0, [r4, #8]
	bl	ftl_malloc
	str	r0, [r5, #-3924]
	.loc 1 71 0
	ldr	r0, [r4, #8]
	bl	ftl_malloc
	str	r0, [r5, #-3920]
	.loc 1 72 0
	mov	r0, #8
	bl	ftl_malloc
	str	r0, [r5, #-3916]
	.loc 1 73 0
	mov	r0, #8
	bl	ftl_malloc
	str	r0, [r5, #-3912]
	.loc 1 74 0
	mov	r0, #6144
	bl	ftl_malloc
	.loc 1 75 0
	ldrh	r3, [r4, #14]
	.loc 1 74 0
	str	r0, [r5, #-3908]
	.loc 1 75 0
	mov	r0, #20
	mul	r0, r0, r3
	bl	ftl_malloc
	str	r0, [r5, #-3904]
	.loc 1 77 0
	ldrh	r0, [r4, #32]
	mov	r0, r0, asl #2
	bl	ftl_malloc
.LBB90:
	.loc 1 78 0
	ldrh	r2, [r4, #32]
	movs	r2, r2, asl #2
.LVL4:
.LBE90:
	.loc 1 77 0
	str	r0, [r5, #-3900]
.LBB91:
	.loc 1 78 0
	beq	.L8
	.loc 1 78 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL5:
.L8:
.LBE91:
	.loc 1 79 0 is_stmt 1
	ldrh	r3, [r4, #34]
	ldr	r0, [r4, #8]
	ldr	r5, .L15+4
	mul	r0, r0, r3
	bl	ftl_malloc
.LVL6:
	.loc 1 80 0
	ldrh	r3, [r4, #34]
	.loc 1 79 0
	str	r0, [r5, #-3896]
	.loc 1 80 0
	mov	r0, #12
	mul	r0, r0, r3
	bl	ftl_malloc
	.loc 1 81 0
	mov	r3, #0
	mov	r2, r3
.LVL7:
	mov	r1, r5
	.loc 1 82 0
	mvn	ip, #0
	.loc 1 80 0
	str	r0, [r5, #-3892]
	.loc 1 81 0 discriminator 1
	ldr	r0, .L15
	.loc 1 81 0 is_stmt 0
	b	.L9
.L10:
	.loc 1 82 0 is_stmt 1
	ldr	lr, [r1, #-3892]
	strh	ip, [lr, r3]	@ movhi
	.loc 1 83 0
	ldr	r4, [r0, #8]
	ldr	lr, [r1, #-3892]
	ldr	r5, [r1, #-3896]
	mul	r4, r4, r2
	add	lr, lr, r3
	.loc 1 81 0 discriminator 2
	add	r2, r2, #1
	add	r3, r3, #12
	.loc 1 83 0
	bic	r4, r4, #3
	add	r4, r5, r4
	str	r4, [lr, #8]
.L9:
	.loc 1 81 0 discriminator 1
	ldrh	lr, [r0, #34]
	ldr	r4, .L15
	cmp	r2, lr
	ldr	r5, .L15+4
	bcc	.L10
	.loc 1 85 0
	ldr	r0, [r4, #8]
	bl	ftl_malloc
.LVL8:
	str	r0, [r5, #-3888]
	.loc 1 86 0
	mov	r0, #8
	bl	ftl_malloc
	str	r0, [r5, #-3884]
	.loc 1 87 0
	ldrh	r0, [r4, #22]
	mov	r0, r0, asl #1
	bl	ftl_malloc
.LBB92:
	.loc 1 88 0
	ldrh	r2, [r4, #22]
	movs	r2, r2, asl #1
.LVL9:
.LBE92:
	.loc 1 87 0
	str	r0, [r5, #-3880]
.LBB93:
	.loc 1 88 0
	beq	.L11
	.loc 1 88 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL10:
.L11:
.LBE93:
	.loc 1 89 0 is_stmt 1
	ldr	r4, .L15
	ldrh	r0, [r4, #22]
	mov	r0, r0, asl #1
	bl	ftl_malloc
.LVL11:
.LBB94:
	.loc 1 90 0
	ldrh	r2, [r4, #22]
.LBE94:
	.loc 1 89 0
	ldr	r3, .L15+4
.LBB95:
	.loc 1 90 0
	movs	r2, r2, asl #1
.LVL12:
.LBE95:
	.loc 1 89 0
	str	r0, [r3, #-3876]
.LBB96:
	.loc 1 90 0
	beq	.L12
	.loc 1 90 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL13:
.L12:
.LBE96:
	.loc 1 92 0 is_stmt 1
	ldrh	r0, [r4, #22]
	mov	r0, r0, asl #2
	bl	ftl_malloc
.LVL14:
.LBB97:
	.loc 1 93 0
	ldrh	r2, [r4, #22]
.LBE97:
	.loc 1 92 0
	ldr	r3, .L15+4
.LBB98:
	.loc 1 93 0
	movs	r2, r2, asl #2
.LVL15:
.LBE98:
	.loc 1 92 0
	str	r0, [r3, #-3872]
.LBB99:
	.loc 1 93 0
	beq	.L13
	.loc 1 93 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL16:
.L13:
.LBE99:
	.loc 1 94 0 is_stmt 1
	ldr	r4, .L15
	ldrh	r0, [r4, #22]
	mov	r0, r0, asl #2
	bl	ftl_malloc
.LVL17:
.LBB100:
	.loc 1 95 0
	ldrh	r2, [r4, #22]
.LBE100:
	.loc 1 94 0
	ldr	r3, .L15+4
.LBB101:
	.loc 1 95 0
	movs	r2, r2, asl #2
.LVL18:
.LBE101:
	.loc 1 94 0
	str	r0, [r3, #-3868]
.LBB102:
	.loc 1 95 0
	beq	.L14
	.loc 1 95 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL19:
.L14:
.LBE102:
	.loc 1 98 0 is_stmt 1
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L16:
	.align	2
.L15:
	.word	.LANCHOR0
	.word	.LANCHOR1
.LFE125:
	.fnend
	.size	sftl_mem_init, .-sftl_mem_init
	.align	2
	.global	sftl_mem_deinit
	.type	sftl_mem_deinit, %function
sftl_mem_deinit:
	.fnstart
.LFB126:
	.loc 1 101 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
.LCFI2:
	.loc 1 102 0
	ldr	r4, .L18
	ldr	r5, .L18+4
	ldr	r0, [r4, #-3924]
	ldr	r1, [r5, #8]
	bl	ftl_free
	.loc 1 103 0
	ldr	r0, [r4, #-3920]
	ldr	r1, [r5, #8]
	bl	ftl_free
	.loc 1 104 0
	ldr	r0, [r4, #-3916]
	mov	r1, #8
	bl	ftl_free
	.loc 1 105 0
	ldr	r0, [r4, #-3912]
	mov	r1, #8
	bl	ftl_free
	.loc 1 106 0
	ldr	r0, [r4, #-3908]
	mov	r1, #6144
	bl	ftl_free
	.loc 1 107 0
	ldrh	r3, [r5, #14]
	mov	r1, #20
	ldr	r0, [r4, #-3904]
	mul	r1, r1, r3
	bl	ftl_free
	.loc 1 109 0
	ldrh	r1, [r5, #32]
	ldr	r0, [r4, #-3900]
	mov	r1, r1, asl #2
	bl	ftl_free
	.loc 1 110 0
	ldrh	r3, [r5, #34]
	ldr	r1, [r5, #8]
	ldr	r0, [r4, #-3896]
	mul	r1, r1, r3
	bl	ftl_free
	.loc 1 111 0
	ldrh	r3, [r5, #34]
	mov	r1, #12
	ldr	r0, [r4, #-3892]
	mul	r1, r1, r3
	bl	ftl_free
	.loc 1 112 0
	ldr	r0, [r4, #-3888]
	ldr	r1, [r5, #8]
	bl	ftl_free
	.loc 1 113 0
	ldr	r0, [r4, #-3884]
	mov	r1, #8
	bl	ftl_free
	.loc 1 114 0
	ldrh	r1, [r5, #22]
	ldr	r0, [r4, #-3880]
	mov	r1, r1, asl #1
	bl	ftl_free
	.loc 1 115 0
	ldrh	r1, [r5, #22]
	ldr	r0, [r4, #-3872]
	mov	r1, r1, asl #2
	bl	ftl_free
	.loc 1 118 0
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, pc}
.L19:
	.align	2
.L18:
	.word	.LANCHOR1
	.word	.LANCHOR0
.LFE126:
	.fnend
	.size	sftl_mem_deinit, .-sftl_mem_deinit
	.align	2
	.global	sftl_free_blk_Q_init
	.type	sftl_free_blk_Q_init, %function
sftl_free_blk_Q_init:
	.fnstart
.LFB127:
	.loc 1 121 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
.LVL20:
	.loc 1 123 0
	mov	r0, #0
.LVL21:
	bx	lr
.LFE127:
	.fnend
	.size	sftl_free_blk_Q_init, .-sftl_free_blk_Q_init
	.align	2
	.global	sftl_get_density
	.type	sftl_get_density, %function
sftl_get_density:
	.fnstart
.LFB134:
	.loc 1 385 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.loc 1 386 0
	ldr	r3, .L22
	ldrh	r2, [r3, #18]
	ldrh	r1, [r3, #4]
	ldrh	r0, [r3, #36]
	rsb	r1, r1, r2
	rsb	r1, r0, r1
	.loc 1 387 0
	ldrh	r0, [r3, #12]
	.loc 1 386 0
	sub	r2, r1, r2, lsr #4
	.loc 1 387 0
	ldrh	r1, [r3, #22]
	mul	r2, r1, r2
	.loc 1 390 0
	mul	r0, r0, r2
	bx	lr
.L23:
	.align	2
.L22:
	.word	.LANCHOR0
.LFE134:
	.fnend
	.size	sftl_get_density, .-sftl_get_density
	.align	2
	.global	dump_sftl_bbt_info
	.type	dump_sftl_bbt_info, %function
dump_sftl_bbt_info:
	.fnstart
.LFB136:
	.loc 1 403 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.loc 1 405 0
	ldr	r1, .L25
	mov	r2, #4
	ldr	r0, .L25+4
	ldr	r3, [r1, #8]
	add	r1, r1, #2112
	mov	r3, r3, lsr #2
	add	r3, r3, #2
	.loc 1 406 0
	.loc 1 405 0
	b	rknand_print_hex
.L26:
	.align	2
.L25:
	.word	.LANCHOR0
	.word	.LC0
.LFE136:
	.fnend
	.size	dump_sftl_bbt_info, .-dump_sftl_bbt_info
	.align	2
	.global	dump_sftl_sys_info
	.type	dump_sftl_sys_info, %function
dump_sftl_sys_info:
	.fnstart
.LFB137:
	.loc 1 408 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
.LCFI3:
	.loc 1 410 0
	mov	r2, #2
	.loc 1 419 0
	ldr	r4, .L28
	.loc 1 410 0
	mov	r3, #12
	ldr	r0, .L28+4
	ldr	r1, .L28+8
	bl	rknand_print_hex
	.loc 1 419 0
	add	r1, r4, #1584
	ldr	r0, .L28+12
	mov	r2, #4
	mov	r3, #130
	bl	rknand_print_hex
	.loc 1 420 0
	ldr	r3, .L28+16
	ldr	r0, .L28+20
	mov	r2, #4
	ldr	r1, [r3, #-3908]
	ldrh	r3, [r4, #18]
	.loc 1 421 0
	ldmfd	sp!, {r4, lr}
	.loc 1 420 0
	b	rknand_print_hex
.L29:
	.align	2
.L28:
	.word	.LANCHOR0
	.word	.LC1
	.word	.LANCHOR1-3864
	.word	.LC2
	.word	.LANCHOR1
	.word	.LC3
.LFE137:
	.fnend
	.size	dump_sftl_sys_info, .-dump_sftl_sys_info
	.align	2
	.global	dump_sftl_l2p_info
	.type	dump_sftl_l2p_info, %function
dump_sftl_l2p_info:
	.fnstart
.LFB138:
	.loc 1 423 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.loc 1 425 0
	ldr	r3, .L31
	mov	r2, #2
	ldr	r0, .L31+4
	add	r1, r3, #1600
	ldrh	r3, [r3, #36]
	.loc 1 436 0
	.loc 1 425 0
	b	rknand_print_hex
.L32:
	.align	2
.L31:
	.word	.LANCHOR0
	.word	.LC4
.LFE138:
	.fnend
	.size	dump_sftl_l2p_info, .-dump_sftl_l2p_info
	.align	2
	.global	dump_sftl_user_info
	.type	dump_sftl_user_info, %function
dump_sftl_user_info:
	.fnstart
.LFB139:
	.loc 1 439 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.loc 1 459 0
	bx	lr
.LFE139:
	.fnend
	.size	dump_sftl_user_info, .-dump_sftl_user_info
	.align	2
	.global	dump_ftl_info
	.type	dump_ftl_info, %function
dump_ftl_info:
	.fnstart
.LFB140:
	.loc 1 461 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
.LCFI4:
	.loc 1 463 0
	bl	dump_sftl_bbt_info
	.loc 1 464 0
	bl	dump_sftl_sys_info
	.loc 1 467 0
	ldmfd	sp!, {r3, lr}
	.loc 1 465 0
	b	dump_sftl_l2p_info
.LFE140:
	.fnend
	.size	dump_ftl_info, .-dump_ftl_info
	.align	2
	.global	sftl_bbt_mark_bad_blk
	.type	sftl_bbt_mark_bad_blk, %function
sftl_bbt_mark_bad_blk:
	.fnstart
.LFB142:
	.file 2 "drivers/rksftl/sftl_bbm.c"
	.loc 2 4 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
.LVL22:
	.loc 2 5 0
	ldr	r3, .L36
	mov	r2, r0, lsr #14
	.loc 2 6 0
	mov	r0, r0, asl #18
.LVL23:
	.loc 2 8 0
	mov	ip, #1
	.loc 2 5 0
	ldrh	r1, [r3, #8]
	.loc 2 6 0
	mov	r3, r0, lsr #18
	.loc 2 5 0
	mla	r0, r1, r2, r3
.LVL24:
	.loc 2 8 0
	ldr	r3, .L36+4
	mov	r2, r0, lsr #5
	and	r0, r0, #31
.LVL25:
	add	r2, r2, #128
	add	r2, r3, r2, asl #2
	ldr	r1, [r2, #2112]
	orr	r0, r1, ip, asl r0
	str	r0, [r2, #2112]
	.loc 2 12 0
	movw	r2, #4148
	.loc 2 14 0
	mov	r0, #0
	.loc 2 12 0
	ldrh	r1, [r3, r2]
	add	r1, r1, ip
	strh	r1, [r3, r2]	@ movhi
	.loc 2 14 0
	bx	lr
.L37:
	.align	2
.L36:
	.word	g_nand_phy_info
	.word	.LANCHOR0
.LFE142:
	.fnend
	.size	sftl_bbt_mark_bad_blk, .-sftl_bbt_mark_bad_blk
	.align	2
	.global	sftl_bbt_bad_blk_check
	.type	sftl_bbt_bad_blk_check, %function
sftl_bbt_bad_blk_check:
	.fnstart
.LFB143:
	.loc 2 21 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
.LVL26:
	.loc 2 24 0
	ldr	r3, .L39
	mov	r2, r0, lsr #14
	.loc 2 25 0
	mov	r0, r0, asl #18
.LVL27:
	.loc 2 24 0
	ldrh	r1, [r3, #8]
	.loc 2 25 0
	mov	r3, r0, lsr #18
	.loc 2 24 0
	mla	r0, r1, r2, r3
.LVL28:
	.loc 2 27 0
	ldr	r3, .L39+4
	mov	r2, r0, lsr #5
	and	r0, r0, #31
.LVL29:
	add	r3, r3, r2, asl #2
	ldr	r3, [r3, #2624]
	mov	r0, r3, lsr r0
	.loc 2 29 0
	and	r0, r0, #1
	bx	lr
.L40:
	.align	2
.L39:
	.word	g_nand_phy_info
	.word	.LANCHOR0
.LFE143:
	.fnend
	.size	sftl_bbt_bad_blk_check, .-sftl_bbt_bad_blk_check
	.align	2
	.global	sftl_bbt_blk_to_bitmap
	.type	sftl_bbt_blk_to_bitmap, %function
sftl_bbt_blk_to_bitmap:
	.fnstart
.LFB145:
	.loc 2 99 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL30:
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
.LCFI5:
	.loc 2 99 0
	mov	r4, r1
.LBB103:
	.loc 2 102 0
	ldr	r3, .L48
.LBE103:
	.loc 2 99 0
	mov	r5, r0
.LBB104:
	.loc 2 102 0
	ldrh	r1, [r3, #8]
.LVL31:
	movs	r1, r1, lsr #3
.LVL32:
	beq	.L42
	.loc 2 102 0 is_stmt 0 discriminator 3
	mov	r0, r4
.LVL33:
	bl	__memzero
.LVL34:
.L42:
.LBE104:
	.loc 2 106 0 is_stmt 1
	ldr	ip, .L48
	.loc 2 99 0
	mov	r3, #0
	.loc 2 104 0
	movw	r6, #65535
	.loc 2 107 0
	mov	r0, #1
.L46:
	.loc 2 104 0
	ldrh	r2, [r5, r3]
	cmp	r2, r6
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, pc}
	.loc 2 106 0
	ldrh	r1, [ip, #8]
.LVL35:
	cmp	r1, r2
	bhi	.L44
	.loc 2 106 0 is_stmt 0 discriminator 2
	ldr	r1, .L48+4
	mov	r2, #106
	ldr	r0, .L48+8
	bl	printk
	.loc 2 106 0 discriminator 3
	bl	dump_ftl_info
.L45:
	b	.L45
.L44:
	.loc 2 107 0 is_stmt 1
	mov	r1, r2, lsr #5
	add	r3, r3, #2
	.loc 2 103 0 discriminator 1
	cmp	r3, #1024
	.loc 2 107 0
	and	r2, r2, #31
	ldr	r7, [r4, r1, asl #2]
	orr	r2, r7, r0, asl r2
	str	r2, [r4, r1, asl #2]
	.loc 2 103 0 discriminator 1
	bne	.L46
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L49:
	.align	2
.L48:
	.word	g_nand_phy_info
	.word	.LC6
	.word	.LC5
.LFE145:
	.fnend
	.size	sftl_bbt_blk_to_bitmap, .-sftl_bbt_blk_to_bitmap
	.align	2
	.global	sftl_bbt_flush
	.type	sftl_bbt_flush, %function
sftl_bbt_flush:
	.fnstart
.LFB147:
	.loc 2 166 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI6:
	.loc 2 168 0
	mov	r5, #0
.LVL36:
	.loc 2 171 0
	ldr	r6, .L58
	.loc 2 174 0
	mov	r9, r5
	.loc 2 171 0
	ldr	r3, .L58+4
	.loc 2 174 0
	mov	fp, r5
	.loc 2 171 0
	str	r3, [r6, #-3832]
	.loc 2 172 0
	add	r3, r3, #2048
	str	r3, [r6, #-3828]
	b	.L57
.L54:
	.loc 2 198 0
	mov	r5, #1
.L57:
	.loc 2 175 0
	ldr	r4, .L58+8
	movw	r7, #4154
	movw	r8, #4150
	.loc 2 179 0
	ldr	sl, .L58+12
	mov	r1, #1
	.loc 2 174 0
	str	fp, [r6, #-3840]
	.loc 2 175 0
	ldrh	r2, [r4, r7]
	ldrh	r3, [r4, r8]
	.loc 2 179 0
	mov	r0, sl
	.loc 2 175 0
	orr	r3, r3, r2, asl #11
	.loc 2 179 0
	mov	r2, #0
	.loc 2 175 0
	str	r3, [r6, #-3836]
	.loc 2 179 0
	mov	r3, r1
	bl	ftl_flash_prog_pages
	.loc 2 180 0
	ldr	r3, .L58+16
	ldrh	r2, [r4, r8]
	ldrh	r3, [r3, #12]
	sub	r3, r3, #1
	cmp	r2, r3
	blt	.L52
	.loc 2 181 0
	mov	r3, #4160
	.loc 2 182 0
	ldrh	r1, [r4, r7]
	.loc 2 181 0
	ldr	r2, [r4, r3]
	.loc 2 187 0
	mov	r0, sl
	.loc 2 184 0
	strh	r9, [r4, r8]	@ movhi
	.loc 2 181 0
	add	r2, r2, #1
	str	r2, [r4, r3]
	.loc 2 183 0
	movw	r2, #4152
	ldrh	r3, [r4, r2]
	.loc 2 185 0
	strh	r1, [r4, r2]	@ movhi
	.loc 2 187 0
	mov	r1, #1
	.loc 2 183 0
	strh	r3, [r4, r7]	@ movhi
	.loc 2 186 0
	mov	r3, r3, asl #11
	str	r3, [r6, #-3836]
	.loc 2 187 0
	bl	ftl_flash_erase_blocks
	.loc 2 188 0
	mov	r1, #1
	mov	r0, sl
	mov	r2, #0
	mov	r3, r1
	bl	ftl_flash_prog_pages
.L52:
	.loc 2 190 0
	ldr	r2, .L58+8
	movw	r3, #4150
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	strh	r1, [r2, r3]	@ movhi
	.loc 2 191 0
	ldr	r3, [r6, #-3840]
	cmn	r3, #1
	beq	.L57
	.loc 2 198 0
	cmp	r5, #0
	beq	.L54
	.loc 2 202 0
	mov	r0, #0
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L59:
	.align	2
.L58:
	.word	.LANCHOR1
	.word	.LANCHOR0+2112
	.word	.LANCHOR0
	.word	.LANCHOR1-3840
	.word	g_nand_phy_info
.LFE147:
	.fnend
	.size	sftl_bbt_flush, .-sftl_bbt_flush
	.align	2
	.global	sftl_bbt_make
	.type	sftl_bbt_make, %function
sftl_bbt_make:
	.fnstart
.LFB146:
	.loc 2 112 0
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
	.loc 2 118 0
	ldr	r2, .L76
	movw	r3, #4156
	.loc 2 112 0
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
.LCFI7:
	.pad #44
	sub	sp, sp, #44
.LCFI8:
	.loc 2 118 0
	ldr	r2, [r2, r3]
	ldr	r3, .L76+4
	cmp	r2, r3
	beq	.L61
.LBB105:
	.loc 2 122 0
	ldr	r7, .L76+8
.LBE105:
	.loc 2 118 0
	mov	r4, #0
.LVL37:
	.loc 2 121 0 discriminator 1
	ldr	r8, .L76+12
	mov	r6, r7
	b	.L62
.L64:
.LBB106:
	.loc 2 122 0
	ldr	r3, .L76
	ldr	r0, [r7, #-3924]
.LVL38:
	ldr	r2, [r3, #8]
.LVL39:
	cmp	r2, #0
	beq	.L63
	.loc 2 122 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL40:
.L63:
.LBE106:
	.loc 2 123 0 is_stmt 1
	mov	r1, r4
	ldr	r0, [r6, #-3924]
.LVL41:
	bl	ftl_flash_get_bad_blk_list
.LVL42:
	.loc 2 125 0
	ldrh	r1, [r8, #8]
	.loc 2 124 0
	ldr	r3, .L76+16
	ldr	r0, [r6, #-3924]
	.loc 2 125 0
	mul	r1, r1, r4
	.loc 2 121 0
	add	r4, r4, #1
	.loc 2 125 0
	mov	r1, r1, lsr #5
	.loc 2 124 0
	add	r1, r3, r1, asl #2
	bl	sftl_bbt_blk_to_bitmap
.L62:
	.loc 2 121 0 discriminator 1
	ldrh	r3, [r8, #2]
	ldr	r5, .L76+12
	cmp	r4, r3
	bcc	.L64
	.loc 2 127 0
	ldr	r0, .L76+20
	mov	r2, #512
	sub	r1, r0, #512
	bl	memcpy
	.loc 2 130 0
	ldrh	r6, [r5, #20]
	b	.L65
.LVL43:
.L67:
	.loc 2 133 0
	mov	r0, r6
	bl	ftl_flash_test_blk
	cmp	r0, #0
	beq	.L66
	.loc 2 134 0
	mov	r0, r6
	bl	sftl_bbt_mark_bad_blk
.L66:
	.loc 2 132 0
	add	r6, r6, #1
	uxth	r6, r6
.LVL44:
.L65:
	.loc 2 131 0
	ldrh	r4, [r5, #20]
.LVL45:
	add	r7, r4, #16
	.loc 2 130 0 discriminator 1
	cmp	r6, r7
	blt	.L67
	b	.L75
.L72:
	.loc 2 139 0
	mov	r0, r4
	bl	sftl_bbt_bad_blk_check
	cmp	r0, #1
	beq	.L69
	.loc 2 141 0
	ldrh	r3, [r6, r5]
	cmp	r3, #0
	.loc 2 142 0
	streqh	r4, [r6, r5]	@ movhi
.L70:
	.loc 2 144 0
	ldrne	r2, .L76
	movwne	r3, #4152
	strneh	r4, [r2, r3]	@ movhi
	.loc 2 145 0
	bne	.L71
.L69:
	.loc 2 138 0
	add	r4, r4, #1
	uxth	r4, r4
	b	.L74
.L75:
	.loc 2 141 0
	ldr	r6, .L76
	movw	r5, #4154
.L74:
	.loc 2 136 0 discriminator 1
	cmp	r4, r7
	blt	.L72
.L71:
	.loc 2 148 0
	ldr	r4, .L76
	.loc 2 151 0
	movw	r7, #4154
	.loc 2 148 0
	ldr	r2, .L76+4
	movw	r3, #4156
	.loc 2 152 0
	movw	r5, #4152
	.loc 2 150 0
	mov	r8, #4160
	.loc 2 149 0
	movw	sl, #4150
	mov	r6, #0
	.loc 2 148 0
	str	r2, [r4, r3]
	.loc 2 153 0
	mov	r1, #2
	.loc 2 151 0
	ldrh	r3, [r4, r7]
	.loc 2 153 0
	mov	r0, sp
	.loc 2 149 0
	strh	r6, [r4, sl]	@ movhi
	.loc 2 150 0
	str	r6, [r4, r8]
	.loc 2 151 0
	mov	r3, r3, asl #11
	str	r3, [sp, #4]
	.loc 2 152 0
	ldrh	r3, [r4, r5]
	mov	r3, r3, asl #11
	str	r3, [sp, #24]
	.loc 2 153 0
	bl	ftl_flash_erase_blocks
	.loc 2 154 0
	bl	sftl_bbt_flush
	.loc 2 156 0
	ldr	r3, [r4, r8]
	.loc 2 158 0
	ldrh	r2, [r4, r5]
	.loc 2 156 0
	add	r3, r3, #1
	str	r3, [r4, r8]
	.loc 2 157 0
	ldrh	r3, [r4, r7]
	.loc 2 159 0
	strh	r6, [r4, sl]	@ movhi
	.loc 2 158 0
	strh	r2, [r4, r7]	@ movhi
	.loc 2 160 0
	strh	r3, [r4, r5]	@ movhi
	.loc 2 161 0
	bl	sftl_bbt_flush
.L61:
	.loc 2 163 0
	mov	r0, #0
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L77:
	.align	2
.L76:
	.word	.LANCHOR0
	.word	1111643168
	.word	.LANCHOR1
	.word	g_nand_phy_info
	.word	.LANCHOR0+2112
	.word	.LANCHOR0+2624
.LFE146:
	.fnend
	.size	sftl_bbt_make, .-sftl_bbt_make
	.align	2
	.global	sftl_sblk_to_pblk
	.type	sftl_sblk_to_pblk, %function
sftl_sblk_to_pblk:
	.fnstart
.LFB148:
	.file 3 "drivers/rksftl/sftl_sys.c"
	.loc 3 9 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
.LVL46:
	.loc 3 10 0
	ldr	r3, .L79
	ldr	r2, .L79+4
	add	r3, r3, r0, asl #1
	ldrh	r3, [r3, r2]
	ldr	r2, .L79+8
	ldrh	r0, [r2, #16]
.LVL47:
	.loc 3 11 0
	mla	r0, r0, r1, r3
	bx	lr
.L80:
	.align	2
.L79:
	.word	.LANCHOR1
	.word	-3976
	.word	.LANCHOR0
.LFE148:
	.fnend
	.size	sftl_sblk_to_pblk, .-sftl_sblk_to_pblk
	.align	2
	.global	sftl_pblk_to_plane
	.type	sftl_pblk_to_plane, %function
sftl_pblk_to_plane:
	.fnstart
.LFB149:
	.loc 3 18 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL48:
	.loc 3 19 0
	ldr	r3, .L82
	.loc 3 18 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
.LCFI9:
	.loc 3 19 0
	mov	r4, r0, lsr #14
	ldrh	r1, [r3, #16]
	mul	r4, r4, r1
	.loc 3 20 0
	bl	__aeabi_uidivmod
.LVL49:
	.loc 3 19 0
	add	r1, r4, r1
	.loc 3 21 0
	uxth	r0, r1
	ldmfd	sp!, {r4, pc}
.L83:
	.align	2
.L82:
	.word	.LANCHOR0
.LFE149:
	.fnend
	.size	sftl_pblk_to_plane, .-sftl_pblk_to_plane
	.align	2
	.global	sftl_ppn_to_sblk
	.type	sftl_ppn_to_sblk, %function
sftl_ppn_to_sblk:
	.fnstart
.LFB150:
	.loc 3 27 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL50:
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
.LCFI10:
	.loc 3 28 0
	ubfx	r0, r0, #11, #14
.LVL51:
	ldr	r3, .L85
	ldrh	r1, [r3, #16]
	bl	__aeabi_uidiv
	.loc 3 29 0
	uxth	r0, r0
	ldmfd	sp!, {r3, pc}
.L86:
	.align	2
.L85:
	.word	.LANCHOR0
.LFE150:
	.fnend
	.size	sftl_ppn_to_sblk, .-sftl_ppn_to_sblk
	.align	2
	.global	sftl_offs_to_ppn
	.type	sftl_offs_to_ppn, %function
sftl_offs_to_ppn:
	.fnstart
.LFB151:
	.loc 3 35 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL52:
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
.LCFI11:
	.loc 3 35 0
	mov	r7, r0
	.loc 3 39 0
	ldr	r6, .L88
	.loc 3 35 0
	mov	r5, r1
	.loc 3 40 0
	mov	r0, r1
.LVL53:
	.loc 3 39 0
	ldrh	r4, [r6, #14]
	.loc 3 40 0
	mov	r1, r4
.LVL54:
	bl	__aeabi_uidivmod
.LBB107:
.LBB108:
	.loc 3 10 0
	ldr	r3, .L88+4
	ldrh	r6, [r6, #16]
.LBE108:
.LBE107:
	.loc 3 39 0
	mov	r0, r5
.LBB111:
.LBB109:
	.loc 3 10 0
	uxth	r1, r1
	add	r1, r3, r1, asl #1
	ldr	r3, .L88+8
	ldrh	r3, [r1, r3]
.LBE109:
.LBE111:
	.loc 3 39 0
	mov	r1, r4
.LBB112:
.LBB110:
	.loc 3 10 0
	mla	r6, r6, r7, r3
.LBE110:
.LBE112:
	.loc 3 39 0
	bl	__aeabi_uidiv
	.loc 3 42 0
	mov	r6, r6, asl #11
	uxth	r0, r0
	.loc 3 44 0
	orr	r0, r6, r0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L89:
	.align	2
.L88:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	-3976
.LFE151:
	.fnend
	.size	sftl_offs_to_ppn, .-sftl_offs_to_ppn
	.align	2
	.global	sftl_ppn_to_offs
	.type	sftl_ppn_to_offs, %function
sftl_ppn_to_offs:
	.fnstart
.LFB152:
	.loc 3 50 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL55:
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
.LCFI12:
	.loc 3 50 0
	mov	r4, r0
.LBB113:
.LBB114:
	.loc 3 19 0
	ldr	r6, .L91
	.loc 3 20 0
	mov	r0, r0, lsr #11
.LVL56:
	.loc 3 19 0
	ldrh	r5, [r6, #16]
	.loc 3 20 0
	mov	r1, r5
	bl	__aeabi_uidivmod
.LBE114:
.LBE113:
	.loc 3 54 0
	mov	r3, r4, asl #21
	.loc 3 57 0
	ldrh	r2, [r6, #14]
.LBB117:
.LBB115:
	.loc 3 19 0
	mov	r4, r4, lsr #25
.LVL57:
.LBE115:
.LBE117:
	.loc 3 54 0
	mov	r3, r3, lsr #21
.LBB118:
.LBB116:
	.loc 3 19 0
	mla	r3, r2, r3, r1
.LBE116:
.LBE118:
	.loc 3 57 0
	mla	r5, r4, r5, r3
	.loc 3 58 0
	uxth	r0, r5
	ldmfd	sp!, {r4, r5, r6, pc}
.L92:
	.align	2
.L91:
	.word	.LANCHOR0
.LFE152:
	.fnend
	.size	sftl_ppn_to_offs, .-sftl_ppn_to_offs
	.align	2
	.global	sftl_select_trace_offs
	.type	sftl_select_trace_offs, %function
sftl_select_trace_offs:
	.fnstart
.LFB153:
	.loc 3 90 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL58:
	.loc 3 92 0
	ldr	r3, .L100
	.loc 3 90 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
.LCFI13:
	.loc 3 94 0
	mov	r8, #1
	ldrh	r6, [r3, #14]
	.loc 3 90 0
	mov	r7, r0
	.loc 3 92 0
	ldrh	r4, [r3, #22]
	.loc 3 94 0
	mov	r3, r8, asl r6
	sub	r3, r3, #1
	cmp	r3, r0
	bne	.L94
.LBB121:
.LBB122:
	.loc 3 94 0 is_stmt 0 discriminator 2
	ldr	r1, .L100+4
	mov	r2, #94
	ldr	r0, .L100+8
.LVL59:
	bl	printk
	.loc 3 94 0 discriminator 3
	bl	dump_ftl_info
.L95:
	b	.L95
.L94:
.LBE122:
.LBE121:
	.loc 3 92 0 is_stmt 1
	sub	r4, r4, #1
	mov	r5, #0
.LVL60:
	uxth	r4, r4
.LVL61:
.L97:
	.loc 3 96 0
	mov	r0, r4
	mov	r1, r6
	bl	__aeabi_uidivmod
	uxth	r1, r1
	ands	r1, r7, r8, asl r1
	bne	.L96
	.loc 3 101 0
	mov	r0, r4
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L96:
	.loc 3 99 0
	add	r5, r5, #1
.LVL62:
	.loc 3 98 0
	sub	r4, r4, #1
.LVL63:
	.loc 3 99 0
	uxth	r5, r5
.LVL64:
	.loc 3 98 0
	uxth	r4, r4
.LVL65:
	.loc 3 99 0
	cmp	r5, r6
	bcc	.L97
	.loc 3 99 0 is_stmt 0 discriminator 2
	ldr	r1, .L100+4
	mov	r2, #99
	ldr	r0, .L100+8
	bl	printk
	.loc 3 99 0 discriminator 3
	bl	dump_ftl_info
.L98:
	b	.L98
.L101:
	.align	2
.L100:
	.word	.LANCHOR0
	.word	.LC7
	.word	.LC5
.LFE153:
	.fnend
	.size	sftl_select_trace_offs, .-sftl_select_trace_offs
	.align	2
	.global	sftl_get_last_written_offset
	.type	sftl_get_last_written_offset, %function
sftl_get_last_written_offset:
	.fnstart
.LFB154:
	.loc 3 115 0 is_stmt 1
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL66:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI14:
	.pad #36
	sub	sp, sp, #36
.LCFI15:
	.loc 3 119 0
	ldr	r6, .L111
	.loc 3 115 0
	mov	r8, r0
	mov	r5, r1
.LBB123:
.LBB124:
	.loc 3 39 0
	ldr	sl, .L111+4
.LBE124:
.LBE123:
	.loc 3 119 0
	ldr	r3, [r6, #-3924]
	str	r3, [sp, #20]
	.loc 3 120 0
	ldr	r3, [r6, #-3916]
	str	r3, [sp, #24]
	.loc 3 124 0
	ldr	r3, [r6, #-3908]
	ldr	r0, [r3, r0, asl #2]
.LVL67:
	mov	r0, r0, lsr #28
	bl	sftl_select_trace_offs
.LVL68:
.LBB134:
.LBB131:
	.loc 3 39 0
	ldrh	r7, [sl, #14]
	.loc 3 40 0
	mov	r1, r7
.LBE131:
.LBE134:
	.loc 3 124 0
	sub	r4, r0, #1
.LVL69:
	.loc 3 125 0
	uxth	r9, r4
.LBB135:
.LBB132:
	.loc 3 40 0
	mov	r0, r9
	bl	__aeabi_uidivmod
.LBB125:
.LBB126:
	.loc 3 10 0
	ldr	r3, .L111+8
	ldrh	fp, [sl, #16]
.LBE126:
.LBE125:
	.loc 3 39 0
	mov	r0, r9
.LBB129:
.LBB127:
	.loc 3 10 0
	uxth	r1, r1
	add	r1, r6, r1, asl #1
	ldrh	r3, [r1, r3]
.LBE127:
.LBE129:
	.loc 3 39 0
	mov	r1, r7
.LBB130:
.LBB128:
	.loc 3 10 0
	mla	fp, fp, r8, r3
.LBE128:
.LBE130:
	.loc 3 39 0
	bl	__aeabi_uidiv
.LBE132:
.LBE135:
	.loc 3 127 0
	mov	r2, #0
	mov	r1, #1
.LBB136:
.LBB133:
	.loc 3 42 0
	mov	fp, fp, asl #11
	uxth	r0, r0
	orr	fp, fp, r0
.LBE133:
.LBE136:
	.loc 3 127 0
	add	r0, sp, #12
	.loc 3 125 0
	str	fp, [sp, #16]
	.loc 3 127 0
	bl	ftl_flash_read_pages
	.loc 3 128 0
	ldr	r3, [r6, #-3916]
	ldr	r3, [r3, #0]
	str	r3, [r5, #0]
	.loc 3 129 0
	ldr	r3, [r6, #-3916]
	ldr	r2, [r3, #4]
	str	r2, [r5, #4]
	.loc 3 130 0
	ldr	r2, [r3, #0]
	cmn	r2, #1
	bne	.L103
	.loc 3 130 0 is_stmt 0 discriminator 1
	ldr	r3, [r3, #4]
	cmn	r3, #1
	bne	.L103
	b	.L110
.LVL70:
.L107:
.LBB137:
.LBB138:
	.loc 3 39 0 is_stmt 1
	ldr	r0, .L111+4
.LBE138:
.LBE137:
	.loc 3 133 0
	add	sl, r7, r4
.LVL71:
	mov	sl, sl, asr #1
.LVL72:
.LBB146:
.LBB143:
	.loc 3 39 0
	ldrh	r9, [r0, #14]
.LBE143:
.LBE146:
	.loc 3 134 0
	uxth	fp, sl
.LBB147:
.LBB144:
	.loc 3 40 0
	str	r3, [sp, #0]
	mov	r0, fp
	mov	r1, r9
	bl	__aeabi_uidivmod
.LBB139:
.LBB140:
	.loc 3 10 0
	ldr	r0, .L111+4
	uxth	r2, r1
	ldrh	r1, [r0, #16]
.LBE140:
.LBE139:
	.loc 3 39 0
	mov	r0, fp
.LBB142:
.LBB141:
	.loc 3 10 0
	add	r2, r6, r2, asl #1
	sub	r2, r2, #3968
	sub	r2, r2, #8
	ldrh	r2, [r2, #0]
	mla	r2, r1, r8, r2
.LBE141:
.LBE142:
	.loc 3 39 0
	mov	r1, r9
	.loc 3 42 0
	mov	r2, r2, asl #11
	.loc 3 39 0
	str	r2, [sp, #4]
	bl	__aeabi_uidiv
	.loc 3 42 0
	ldr	r2, [sp, #4]
.LBE144:
.LBE147:
	.loc 3 135 0
	mov	r1, #1
.LBB148:
.LBB145:
	.loc 3 42 0
	uxth	r0, r0
	orr	r2, r2, r0
.LBE145:
.LBE148:
	.loc 3 135 0
	add	r0, sp, #12
	.loc 3 134 0
	str	r2, [sp, #16]
	.loc 3 135 0
	mov	r2, #0
	bl	ftl_flash_read_pages
	.loc 3 136 0
	ldr	r1, [r6, #-3916]
	ldr	r3, [sp, #0]
	ldr	r2, [r1, #0]
	cmn	r2, #1
	bne	.L105
	.loc 3 136 0 is_stmt 0 discriminator 1
	ldr	r1, [r1, #4]
	cmn	r1, #1
	.loc 3 138 0 is_stmt 1
	subeq	r4, sl, #1
	.loc 3 136 0 discriminator 1
	beq	.L109
.L105:
	.loc 3 141 0
	str	r2, [r5, #0]
	.loc 3 140 0
	add	r7, sl, #1
	.loc 3 142 0
	ldr	r2, [r3, #-3916]
	ldr	r2, [r2, #4]
	str	r2, [r5, #4]
	b	.L109
.LVL73:
.L110:
	.loc 3 130 0 discriminator 1
	mov	r7, #0
.LVL74:
	mov	r3, r6
.LVL75:
.L109:
	.loc 3 132 0 discriminator 1
	cmp	r7, r4
	ble	.L107
.L103:
	.loc 3 147 0
	mov	r0, r4
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L112:
	.align	2
.L111:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	-3976
.LFE154:
	.fnend
	.size	sftl_get_last_written_offset, .-sftl_get_last_written_offset
	.align	2
	.global	sftl_get_last_written_page
	.type	sftl_get_last_written_page, %function
sftl_get_last_written_page:
	.fnstart
.LFB155:
	.loc 3 155 0
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL76:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
.LCFI16:
	.pad #24
	sub	sp, sp, #24
.LCFI17:
	.loc 3 160 0
	ldr	r9, .L122
	.loc 3 165 0
	add	r8, sp, #4
	.loc 3 159 0
	str	r0, [sp, #12]
	.loc 3 164 0
	mov	r7, r1, asl #11
	.loc 3 165 0
	mov	r0, r8
.LVL77:
	mov	r1, #1
.LVL78:
	.loc 3 160 0
	ldr	r3, [r9, #-3916]
	.loc 3 155 0
	mov	sl, r2
	.loc 3 160 0
	str	r3, [sp, #16]
	.loc 3 163 0
	ldr	r3, .L122+4
	ldrh	r4, [r3, #10]
	sub	r4, r4, #1
	uxth	r4, r4
.LVL79:
	.loc 3 164 0
	sxth	r3, r4
.LVL80:
	orr	r3, r7, r3
.LVL81:
	str	r3, [sp, #8]
	.loc 3 165 0
	bl	ftl_flash_read_pages
.LVL82:
	.loc 3 166 0
	ldr	r3, [r9, #-3916]
	ldr	r3, [r3, #0]
	cmn	r3, #1
	bne	.L115
	b	.L121
.LVL83:
.L118:
	.loc 3 168 0
	add	r5, r5, r3
.LVL84:
	.loc 3 170 0
	mov	r2, sl
	mov	r0, r8
	mov	r1, #1
	.loc 3 168 0
	add	r5, r5, r5, lsr #31
	ubfx	r5, r5, #1, #16
.LVL85:
	.loc 3 169 0
	sxth	r3, r5
.LVL86:
	orr	r3, r3, r7
.LVL87:
	str	r3, [sp, #8]
	.loc 3 170 0
	bl	ftl_flash_read_pages
	.loc 3 171 0
	ldr	r3, [r9, #-3916]
	ldr	r2, [r3, #0]
	cmn	r2, #1
	bne	.L116
	.loc 3 171 0 is_stmt 0 discriminator 1
	ldr	r3, [r3, #4]
	cmn	r3, #1
	.loc 3 173 0 is_stmt 1
	subeq	r4, r5, #1
.LVL88:
	uxtheq	r4, r4
.LVL89:
	.loc 3 171 0 discriminator 1
	beq	.L120
.L116:
	.loc 3 175 0
	add	r5, r5, #1
.LVL90:
	uxth	r6, r5
.LVL91:
	b	.L120
.LVL92:
.L121:
	.loc 3 166 0
	mov	r6, #0
.LVL93:
.L120:
	.loc 3 167 0 discriminator 1
	sxth	r5, r6
.LVL94:
	sxth	r3, r4
.LVL95:
	cmp	r5, r3
	ble	.L118
.L115:
	.loc 3 179 0
	sxth	r0, r4
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L123:
	.align	2
.L122:
	.word	.LANCHOR1
	.word	g_nand_phy_info
.LFE155:
	.fnend
	.size	sftl_get_last_written_page, .-sftl_get_last_written_page
	.align	2
	.global	sftl_bbt_load
	.type	sftl_bbt_load, %function
sftl_bbt_load:
	.fnstart
.LFB144:
	.loc 2 32 0
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI18:
.LBB149:
	.loc 2 42 0 discriminator 3
	mov	r1, #2064
.LBE149:
	.loc 2 43 0
	ldr	r6, .L142
.LBB150:
	.loc 2 42 0 discriminator 3
	ldr	r0, .L142+4
	bl	__memzero
.LBE150:
	.loc 2 46 0
	ldr	r9, .L142+8
	.loc 2 43 0
	ldr	r4, [r6, #-3924]
	mov	sl, r6
	.loc 2 44 0
	ldr	r3, [r6, #-3916]
	.loc 2 46 0
	ldrh	r7, [r9, #20]
.LVL96:
	.loc 2 43 0
	str	r4, [r6, #-3832]
	.loc 2 44 0
	str	r3, [r6, #-3828]
	.loc 2 46 0
	b	.L125
.L130:
	.loc 2 49 0
	mov	r3, r7, asl #11
	.loc 2 50 0
	ldr	r0, .L142+12
	.loc 2 49 0
	str	r3, [r6, #-3836]
	.loc 2 50 0
	mov	r1, #1
	mov	r2, #0
	bl	ftl_flash_read_pages
	.loc 2 51 0
	ldr	r3, [r6, #-3840]
	cmn	r3, #1
	bne	.L126
	.loc 2 52 0
	ldr	r3, [r6, #-3836]
	.loc 2 53 0
	mov	r1, #1
	ldr	r0, .L142+12
	mov	r2, #0
	.loc 2 52 0
	add	r3, r3, #1
	str	r3, [r6, #-3836]
	.loc 2 53 0
	bl	ftl_flash_read_pages
.L126:
	.loc 2 55 0
	ldr	r3, [sl, #-3840]
	ldr	r5, .L142
	cmn	r3, #1
	beq	.L127
	.loc 2 57 0
	ldr	r8, [r4, #2044]
	ldr	r3, .L142+16
	cmp	r8, r3
	bne	.L127
	.loc 2 59 0
	movw	r7, #2042
.LVL97:
	.loc 2 58 0
	ldr	r3, [r5, #-3916]
	.loc 2 59 0
	ldrh	r6, [r4, r7]
.LVL98:
	.loc 2 65 0
	cmp	r6, #0
	.loc 2 58 0
	ldr	sl, [r3, #0]
.LVL99:
	.loc 2 60 0
	movw	r3, #2040
	ldrh	r3, [r4, r3]
.LVL100:
	.loc 2 65 0
	bne	.L141
	b	.L139
.LVL101:
.L127:
	.loc 2 48 0
	add	r7, r7, #1
.LVL102:
	uxth	r7, r7
.LVL103:
.L125:
	.loc 2 47 0
	ldrh	r3, [r9, #20]
	add	r3, r3, #16
	.loc 2 46 0 discriminator 1
	cmp	r7, r3
	blt	.L130
	b	.L139
.LVL104:
.L141:
	.loc 2 68 0
	cmp	r3, #0
	beq	.L131
	.loc 2 69 0
	mov	r3, r3, asl #11
.LVL105:
	.loc 2 70 0
	sub	r0, r5, #3840
	.loc 2 69 0
	str	r3, [r5, #-3836]
	.loc 2 70 0
	mov	r1, #1
	mov	r2, #0
	bl	ftl_flash_read_pages
	.loc 2 71 0
	ldr	r3, [r5, #-3840]
	cmn	r3, #1
	beq	.L131
	.loc 2 72 0
	ldr	r3, [r4, #2044]
	cmp	r3, r8
	bne	.L131
	.loc 2 72 0 is_stmt 0 discriminator 1
	ldr	r3, [r5, #-3916]
	ldr	r3, [r3, #0]
	cmp	r3, sl
	.loc 2 74 0 is_stmt 1
	ldrhih	r6, [r4, r7]
.LVL106:
.L131:
	.loc 2 79 0
	ldr	r4, .L142
	mov	r1, r6
	mov	r2, #0
	.loc 2 83 0
	ldr	r8, .L142+20
	.loc 2 82 0
	mov	r9, r6, asl #11
	.loc 2 81 0
	mov	r5, #0
	.loc 2 79 0
	ldr	r0, [r4, #-3924]
	.loc 2 83 0
	add	fp, r8, #2112
	.loc 2 79 0
	bl	sftl_get_last_written_page
.LVL107:
	.loc 2 80 0
	add	r3, r0, #1
	.loc 2 79 0
	mov	sl, r0
.LVL108:
	.loc 2 80 0
	uxth	r3, r3
	.loc 2 81 0
	b	.L132
.L135:
	.loc 2 82 0
	orr	r2, r2, r9
	str	r2, [r4, #-3836]
	.loc 2 84 0
	ldr	r2, .L142+24
	.loc 2 85 0
	mov	r1, #1
	.loc 2 83 0
	str	fp, [r4, #-3832]
	.loc 2 85 0
	ldr	r0, .L142+12
	.loc 2 84 0
	str	r2, [r4, #-3828]
	.loc 2 85 0
	mov	r2, #0
	str	r3, [sp, #4]
	bl	ftl_flash_read_pages
	.loc 2 86 0
	ldr	r2, [r4, #-3840]
	ldr	r3, [sp, #4]
	cmn	r2, #1
	beq	.L133
	.loc 2 86 0 is_stmt 0 discriminator 1
	movw	r2, #4156
	ldr	r1, [r8, r2]
	ldr	r2, .L142+16
	cmp	r1, r2
	beq	.L134
.L133:
	sub	r5, r5, #1
	uxth	r5, r5
.L132:
	.loc 2 31 0 is_stmt 1
	add	r7, r5, sl
	uxth	r7, r7
	.loc 2 81 0 discriminator 1
	sxth	r2, r7
	cmp	r2, #0
	bge	.L135
.L134:
	.loc 2 90 0
	ldr	r2, .L142+20
	.loc 2 91 0
	tst	r7, #32768
	.loc 2 90 0
	movw	r1, #4150
	strh	r3, [r2, r1]	@ movhi
	.loc 2 91 0
	beq	.L136
	.loc 2 91 0 is_stmt 0 discriminator 2
	ldr	r1, .L142+28
	mov	r2, #91
	ldr	r0, .L142+32
	bl	printk
	.loc 2 91 0 discriminator 3
	bl	dump_ftl_info
.L137:
	b	.L137
.L136:
	.loc 2 92 0 is_stmt 1
	movw	r3, #4154
	ldrh	r3, [r2, r3]
	cmp	r3, r6
	.loc 2 95 0
	moveq	r0, #0
	.loc 2 92 0
	beq	.L128
	.loc 2 92 0 is_stmt 0 discriminator 2
	ldr	r1, .L142+28
	mov	r2, #92
	ldr	r0, .L142+32
	bl	printk
	.loc 2 92 0 discriminator 3
	bl	dump_ftl_info
.L138:
	b	.L138
.LVL109:
.L139:
	.loc 2 66 0 is_stmt 1
	mvn	r0, #0
.L128:
	.loc 2 96 0
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L143:
	.align	2
.L142:
	.word	.LANCHOR1
	.word	.LANCHOR0+2112
	.word	g_nand_phy_info
	.word	.LANCHOR1-3840
	.word	1111643168
	.word	.LANCHOR0
	.word	.LANCHOR0+4160
	.word	.LC6
	.word	.LC5
.LFE144:
	.fnend
	.size	sftl_bbt_load, .-sftl_bbt_load
	.align	2
	.global	sftl_trace
	.type	sftl_trace, %function
sftl_trace:
	.fnstart
.LFB157:
	.loc 3 262 0
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL110:
	.loc 3 269 0
	cmp	r1, #1
	.loc 3 262 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI19:
	mov	sl, r0
	.pad #36
	sub	sp, sp, #36
.LCFI20:
	.loc 3 262 0
	mov	r5, r1
	.loc 3 269 0
	bne	.L145
.LBB151:
	.loc 3 270 0
	ldr	r3, .L167
	ldr	r0, [r3, #-3880]
.LVL111:
	ldr	r3, .L167+4
	ldrh	r2, [r3, #22]
	movs	r2, r2, asl #1
.LVL112:
	b	.L166
.LVL113:
.L145:
.LBE151:
	.loc 3 271 0
	cmp	r1, #2
	bne	.L146
.LBB152:
	.loc 3 272 0
	ldr	r3, .L167
	ldr	r0, [r3, #-3872]
.LVL114:
	ldr	r3, .L167+4
	ldrh	r2, [r3, #22]
	movs	r2, r2, asl #2
.LVL115:
.L166:
	beq	.L146
	.loc 3 272 0 is_stmt 0 discriminator 4
	mov	r1, #255
.LVL116:
	bl	memset
.LVL117:
.L146:
.LBE152:
	.loc 3 274 0 is_stmt 1
	mov	r0, sl
.LVL118:
	add	r1, sp, #24
	bl	sftl_get_last_written_offset
.LVL119:
	.loc 3 275 0
	cmn	r0, #1
.LVL120:
	beq	.L147
	.loc 3 278 0
	add	r7, r0, #1
.LVL121:
	.loc 3 279 0
	mov	r6, #0
.LVL122:
	ldr	r4, .L167
	.loc 3 280 0
	b	.L148
.LVL123:
.L155:
.LBB153:
.LBB154:
.LBB155:
.LBB156:
	.loc 3 10 0
	ldr	r1, .L167+4
.LBE156:
.LBE155:
.LBE154:
.LBE153:
	.loc 3 283 0
	mov	ip, r6, asl #5
.LBB171:
.LBB165:
	.loc 3 39 0
	ldr	r2, .L167+4
.LBE165:
.LBE171:
	.loc 3 281 0
	cmp	r7, #32
	movlt	r8, r7
	movge	r8, #32
	.loc 3 283 0
	uxth	ip, ip
	.loc 3 282 0
	mov	r9, #0
.LBB172:
.LBB166:
.LBB161:
.LBB157:
	.loc 3 10 0
	ldrh	r3, [r1, #16]
.LBE157:
.LBE161:
	.loc 3 39 0
	ldrh	r2, [r2, #14]
	mov	fp, ip
	str	r7, [sp, #20]
.LBB162:
.LBB158:
	.loc 3 10 0
	mul	r3, r3, sl
.LBE158:
.LBE162:
	.loc 3 39 0
	str	r2, [sp, #8]
.LBE166:
.LBE172:
	.loc 3 284 0
	ldr	r2, [r4, #-3924]
	str	r2, [sp, #12]
.LBB173:
.LBB167:
.LBB163:
.LBB159:
	.loc 3 10 0
	str	r3, [sp, #16]
.LBE159:
.LBE163:
.LBE167:
.LBE173:
	.loc 3 285 0
	ldr	r3, .L167+8
	.loc 3 282 0
	b	.L149
.L150:
	.loc 3 283 0
	add	r2, r9, fp
	mov	r1, #20
	mla	r7, r1, r9, r4
.LVL124:
	uxth	r2, r2
.LBB174:
.LBB168:
	.loc 3 40 0
	ldr	r1, [sp, #8]
	mov	r0, r2
.LVL125:
	str	r3, [sp, #0]
	str	r2, [sp, #4]
	bl	__aeabi_uidivmod
	.loc 3 39 0
	ldr	r2, [sp, #4]
	mov	r0, r2
.LBB164:
.LBB160:
	.loc 3 10 0
	uxth	r1, r1
	add	r1, r4, r1, asl #1
	sub	r1, r1, #3968
	ldrh	ip, [r1, #-8]
	ldr	r1, [sp, #16]
	add	ip, ip, r1
.LBE160:
.LBE164:
	.loc 3 39 0
	ldr	r1, [sp, #8]
	.loc 3 42 0
	mov	ip, ip, asl #11
	.loc 3 39 0
	str	ip, [sp, #4]
	bl	__aeabi_uidiv
.LBE168:
.LBE174:
	.loc 3 284 0
	ldr	r2, [sp, #12]
.LBB175:
.LBB169:
	.loc 3 42 0
	ldr	ip, [sp, #4]
.LBE169:
.LBE175:
	.loc 3 284 0
	str	r2, [r7, #-3812]
.LBB176:
.LBB170:
	.loc 3 42 0
	uxth	r0, r0
	orr	ip, ip, r0
.LBE170:
.LBE176:
	.loc 3 283 0
	str	ip, [r7, #-3816]
	.loc 3 285 0
	ldr	r3, [sp, #0]
	add	r2, r3, r9, asl #3
	.loc 3 282 0 discriminator 2
	add	r9, r9, #1
	.loc 3 285 0
	str	r2, [r7, #-3808]
	.loc 3 282 0 discriminator 2
	uxth	r9, r9
.LVL126:
.L149:
	.loc 3 282 0 is_stmt 0 discriminator 1
	cmp	r9, r8
	blt	.L150
	.loc 3 287 0 is_stmt 1
	mov	r1, r8
	mov	r2, #0
	ldr	r0, .L167+12
.LVL127:
	ldr	r7, [sp, #20]
	bl	ftl_flash_read_pages
	.loc 3 291 0
	mov	r2, r6, asl #5
	.loc 3 288 0
	mov	r3, #0
	.loc 3 289 0
	mov	r1, #20
	.loc 3 288 0
	b	.L151
.L154:
	.loc 3 289 0
	mla	r0, r1, r3, r4
	ldr	r0, [r0, #-3820]
	cmn	r0, #1
	beq	.L152
	.loc 3 290 0
	cmp	r5, #1
	bne	.L153
	.loc 3 291 0
	add	r9, r4, r3, asl #3
	add	r0, r3, r2
	ldr	ip, [r4, #-3880]
	ldr	r9, [r9, #-3176]
	mov	r0, r0, asl #1
	strh	r9, [ip, r0]	@ movhi
	b	.L152
.L153:
	.loc 3 292 0
	cmp	r5, #2
	.loc 3 293 0
	addeq	r0, r4, r3, asl #3
	addeq	ip, r3, r2
	ldreq	r9, [r0, #-3176]
	ldreq	r0, [r4, #-3872]
	streq	r9, [r0, ip, asl #2]
.L152:
	.loc 3 288 0
	add	r3, r3, #1
	uxth	r3, r3
.L151:
	.loc 3 288 0 is_stmt 0 discriminator 1
	cmp	r3, r8
	blt	.L154
	.loc 3 297 0 is_stmt 1
	add	r6, r6, #1
.LVL128:
	.loc 3 296 0
	rsb	r7, r8, r7
	.loc 3 297 0
	uxth	r6, r6
.LVL129:
.L148:
	.loc 3 280 0 discriminator 1
	cmp	r7, #0
	ldr	ip, .L167
	bgt	.L155
	.loc 3 300 0
	ldr	r3, [ip, #-3908]
	.loc 3 302 0
	mov	r4, #1
	.loc 3 301 0 discriminator 1
	ldr	r1, .L167+4
	.loc 3 307 0
	mvn	r0, #0
.LVL130:
	.loc 3 300 0
	ldr	r6, [r3, sl, asl #2]
.LVL131:
	.loc 3 301 0
	mov	r3, #0
.LVL132:
	.loc 3 300 0
	mov	r6, r6, lsr #28
.LVL133:
	.loc 3 301 0
	b	.L156
.L162:
	.loc 3 302 0
	ands	r2, r6, r4, asl r3
	movne	r2, #0
	bne	.L157
	b	.L158
.LVL134:
.L161:
	.loc 3 304 0
	cmp	r5, #1
	bne	.L159
	.loc 3 305 0
	ldrh	r7, [r1, #14]
	ldr	r8, [ip, #-3880]
.LVL135:
	mla	r7, r7, r2, r3
	mov	r7, r7, asl #1
	strh	r0, [r8, r7]	@ movhi
	b	.L160
.LVL136:
.L159:
	.loc 3 306 0
	cmp	r5, #2
	.loc 3 307 0
	ldreqh	r8, [r1, #14]
.LVL137:
	ldreq	r7, [ip, #-3872]
	mlaeq	r8, r8, r2, r3
	streq	r0, [r7, r8, asl #2]
.L160:
	.loc 3 303 0
	add	r2, r2, #1
	uxth	r2, r2
.LVL138:
.L157:
	.loc 3 303 0 is_stmt 0 discriminator 1
	ldrh	r7, [r1, #20]
.LVL139:
	cmp	r7, r2
	bhi	.L161
.LVL140:
.L158:
	.loc 3 301 0 is_stmt 1
	add	r3, r3, #1
.LVL141:
	uxth	r3, r3
.LVL142:
.L156:
	.loc 3 301 0 is_stmt 0 discriminator 1
	ldrh	r2, [r1, #14]
	cmp	r2, r3
	bhi	.L162
.LVL143:
.L147:
	.loc 3 313 0 is_stmt 1
	mov	r0, #0
.LVL144:
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L168:
	.align	2
.L167:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR1-3180
	.word	.LANCHOR1-3820
.LFE157:
	.fnend
	.size	sftl_trace, .-sftl_trace
	.align	2
	.global	sftl_write_trace_page
	.type	sftl_write_trace_page, %function
sftl_write_trace_page:
	.fnstart
.LFB158:
	.loc 3 320 0
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL145:
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
.LCFI21:
	mov	r4, r1
.LBB177:
.LBB178:
	.loc 3 39 0
	ldr	r7, .L174
.LBE178:
.LBE177:
	.loc 3 320 0
	.pad #28
	sub	sp, sp, #28
.LCFI22:
	.loc 3 320 0
	mov	r5, r0
.LBB190:
.LBB187:
	.loc 3 40 0
	mov	r0, r1
.LVL146:
.LBE187:
.LBE190:
	.loc 3 320 0
	mov	r8, r2
.LBB191:
.LBB188:
.LBB179:
.LBB180:
	.loc 3 10 0
	ldr	r6, .L174+4
.LBE180:
.LBE179:
	.loc 3 39 0
	ldrh	sl, [r7, #14]
	.loc 3 40 0
	mov	r1, sl
.LVL147:
	bl	__aeabi_uidivmod
.LVL148:
.LBB184:
.LBB181:
	.loc 3 10 0
	ldr	r3, .L174+8
	ldrh	r2, [r7, #16]
.LBE181:
.LBE184:
	.loc 3 39 0
	mov	r0, r4
	mov	r4, r6
.LVL149:
.LBB185:
.LBB182:
	.loc 3 10 0
	uxth	r1, r1
	add	r1, r6, r1, asl #1
	ldrh	r3, [r1, r3]
.LBE182:
.LBE185:
	.loc 3 39 0
	mov	r1, sl
.LBB186:
.LBB183:
	.loc 3 10 0
	mla	r5, r2, r5, r3
.LVL150:
.LBE183:
.LBE186:
	.loc 3 39 0
	bl	__aeabi_uidiv
.LBE188:
.LBE191:
.LBB192:
	.loc 3 329 0
	ldr	r1, [r7, #8]
.LVL151:
.LBE192:
	.loc 3 328 0
	ldr	r3, [r6, #-3916]
.LBB193:
	.loc 3 329 0
	cmp	r1, #0
.LBE193:
	.loc 3 328 0
	str	r3, [sp, #16]
.LBB194:
.LBB189:
	.loc 3 42 0
	mov	r5, r5, asl #11
	uxth	r0, r0
	orr	r5, r5, r0
.LBE189:
.LBE194:
	.loc 3 327 0
	ldr	r0, [r6, #-3924]
	.loc 3 326 0
	str	r5, [sp, #8]
	.loc 3 327 0
	str	r0, [sp, #12]
.LBB195:
	.loc 3 329 0
	beq	.L170
	.loc 3 329 0 is_stmt 0 discriminator 3
	bl	__memzero
.LVL152:
.L170:
.LBE195:
	.loc 3 330 0 is_stmt 1
	cmp	r8, #1
	.loc 3 331 0
	ldreqh	r2, [r7, #22]
	ldreq	r0, [r4, #-3924]
	ldreq	r1, [r4, #-3876]
.LVL153:
	moveq	r2, r2, asl #1
	beq	.L173
.LVL154:
.L171:
	.loc 3 332 0
	cmp	r8, #2
	bne	.L172
	.loc 3 333 0
	ldr	r3, .L174
	ldr	r0, [r4, #-3924]
	ldr	r1, [r4, #-3868]
.LVL155:
	ldrh	r2, [r3, #22]
	mov	r2, r2, asl #2
.L173:
	bl	memcpy
.LVL156:
.L172:
.LBB196:
	.loc 3 334 0 discriminator 3
	ldr	r4, .L174+4
	mov	r1, #8
.LVL157:
	ldr	r0, [r4, #-3916]
	bl	__memzero
.LBE196:
	.loc 3 335 0
	ldr	r3, [r4, #-3916]
	ldr	r2, .L174+12
	.loc 3 336 0
	mov	r1, #1
	add	r0, sp, #4
	.loc 3 335 0
	str	r2, [r3, #0]
	.loc 3 336 0
	mov	r2, #0
	mov	r3, r1
	bl	ftl_flash_prog_pages
	.loc 3 338 0
	mov	r0, #0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L175:
	.align	2
.L174:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	-3976
	.word	1414676803
.LFE158:
	.fnend
	.size	sftl_write_trace_page, .-sftl_write_trace_page
	.align	2
	.global	sftl_get_trace_list
	.type	sftl_get_trace_list, %function
sftl_get_trace_list:
	.fnstart
.LFB159:
	.loc 3 345 0
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL158:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
.LCFI23:
	mov	r7, r0
	.loc 3 351 0
	ldr	r4, .L181
	.loc 3 345 0
	.pad #24
	sub	sp, sp, #24
.LCFI24:
	.loc 3 345 0
	mov	r6, r1
.LBB197:
.LBB198:
	.loc 3 39 0
	ldr	r5, .L181+4
.LBE198:
.LBE197:
	.loc 3 351 0
	ldr	r3, [r4, #-3908]
	ldr	r0, [r3, r0, asl #2]
.LVL159:
	.loc 3 352 0
	mov	r0, r0, lsr #28
	bl	sftl_select_trace_offs
.LVL160:
.LBB209:
.LBB205:
	.loc 3 39 0
	ldrh	r8, [r5, #14]
	.loc 3 40 0
	mov	r1, r8
.LBE205:
.LBE209:
	.loc 3 352 0
	mov	r9, r0
.LBB210:
.LBB206:
	.loc 3 40 0
	bl	__aeabi_uidivmod
.LBB199:
.LBB200:
	.loc 3 10 0
	ldr	r3, .L181+8
.LBE200:
.LBE199:
	.loc 3 39 0
	mov	r0, r9
.LBB203:
.LBB201:
	.loc 3 10 0
	ldrh	sl, [r5, #16]
	uxth	r1, r1
	add	r1, r4, r1, asl #1
	ldrh	r3, [r1, r3]
.LBE201:
.LBE203:
	.loc 3 39 0
	mov	r1, r8
.LBB204:
.LBB202:
	.loc 3 10 0
	mla	sl, sl, r7, r3
.LBE202:
.LBE204:
	.loc 3 39 0
	bl	__aeabi_uidiv
.LBE206:
.LBE210:
	.loc 3 354 0
	ldr	r3, [r4, #-3924]
	.loc 3 356 0
	mov	r1, #1
	mov	r2, #0
	.loc 3 354 0
	str	r3, [sp, #12]
	.loc 3 355 0
	ldr	r3, [r4, #-3916]
.LBB211:
.LBB207:
	.loc 3 42 0
	mov	sl, sl, asl #11
.LBE207:
.LBE211:
	.loc 3 355 0
	str	r3, [sp, #16]
.LBB212:
.LBB208:
	.loc 3 42 0
	uxth	r0, r0
	orr	sl, sl, r0
.LBE208:
.LBE212:
	.loc 3 356 0
	add	r0, sp, #4
	.loc 3 353 0
	str	sl, [sp, #8]
	.loc 3 356 0
	bl	ftl_flash_read_pages
	.loc 3 357 0
	ldr	r3, [sp, #4]
	cmn	r3, #1
	beq	.L177
	.loc 3 357 0 is_stmt 0 discriminator 1
	ldr	r3, [r4, #-3916]
	ldr	r2, [r3, #0]
	ldr	r3, .L181+12
	cmp	r2, r3
	bne	.L177
	.loc 3 358 0 is_stmt 1
	cmp	r6, #1
	.loc 3 359 0
	ldreqh	r2, [r5, #22]
	ldreq	r0, [r4, #-3880]
	ldreq	r1, [r4, #-3924]
	moveq	r2, r2, asl #1
	beq	.L180
.L178:
	.loc 3 360 0
	cmp	r6, #2
	bne	.L179
	.loc 3 361 0
	ldrh	r2, [r5, #22]
	ldr	r0, [r4, #-3872]
	ldr	r1, [r4, #-3924]
	mov	r2, r2, asl #2
.L180:
	bl	memcpy
	b	.L179
.L177:
	.loc 3 365 0
	mov	r0, r7
	mov	r1, r6
	bl	sftl_trace
.L179:
	.loc 3 367 0
	mov	r0, #0
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L182:
	.align	2
.L181:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	-3976
	.word	1414676803
.LFE159:
	.fnend
	.size	sftl_get_trace_list, .-sftl_get_trace_list
	.align	2
	.global	sftl_l2p_check_real_vpn
	.type	sftl_l2p_check_real_vpn, %function
sftl_l2p_check_real_vpn:
	.fnstart
.LFB162:
	.loc 3 533 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL161:
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
.LCFI25:
	.loc 3 537 0
	mov	r4, #0
.LVL162:
	.loc 3 533 0
	mov	r8, r0
	.loc 3 538 0
	mov	r5, r4
.LVL163:
	.loc 3 538 0 is_stmt 0 discriminator 1
	ldr	r7, .L187
	.loc 3 540 0 is_stmt 1
	ldr	r6, .L187+4
	.loc 3 538 0
	b	.L184
.LVL164:
.L186:
	.loc 3 540 0
	ldr	r3, [r6, #-3900]
	ldr	r0, [r3, r5, asl #2]
.LVL165:
	.loc 3 541 0
	cmn	r0, #1
	beq	.L185
	.loc 3 542 0
	bl	sftl_ppn_to_sblk
.LVL166:
	.loc 3 541 0 discriminator 1
	cmp	r0, r8
	.loc 3 543 0
	addeq	r4, r4, #1
.LVL167:
	uxtheq	r4, r4
.LVL168:
.L185:
	.loc 3 538 0
	add	r5, r5, #1
.LVL169:
	uxth	r5, r5
.LVL170:
.L184:
	.loc 3 538 0 is_stmt 0 discriminator 1
	ldrh	r3, [r7, #32]
	cmp	r3, r5
	bhi	.L186
	.loc 3 547 0 is_stmt 1
	mov	r0, r4
.LVL171:
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L188:
	.align	2
.L187:
	.word	.LANCHOR0
	.word	.LANCHOR1
.LFE162:
	.fnend
	.size	sftl_l2p_check_real_vpn, .-sftl_l2p_check_real_vpn
	.align	2
	.global	sftl_add_sblk_to_wait_gc_pool
	.type	sftl_add_sblk_to_wait_gc_pool, %function
sftl_add_sblk_to_wait_gc_pool:
	.fnstart
.LFB169:
	.loc 3 957 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL172:
	.loc 3 960 0
	ldr	r3, .L195
	movw	ip, #65535
	ldr	r2, .L195+4
	.loc 3 957 0
	stmfd	sp!, {r4, r5, lr}
	.save {r4, r5, lr}
.LCFI26:
	.loc 3 960 0
	ldrh	r1, [r3, r2]
	cmp	r1, ip
	.loc 3 965 0
	ldrne	r2, [r3, #-3948]
	movne	r3, r1
	.loc 3 960 0
	bne	.L191
	.loc 3 961 0
	mov	r1, #0
	strh	r1, [r3, r2]	@ movhi
	.loc 3 962 0
	ldr	r3, [r3, #-3948]
	strh	r0, [r3, #0]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L194:
	.loc 3 965 0
	mov	r4, r3, asl #1
	add	r5, r2, r4
	ldrh	r4, [r2, r4]
	cmp	r4, ip
	bne	.L193
	.loc 3 966 0
	strh	r0, [r5, #0]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L193:
	.loc 3 964 0
	add	r3, r3, #1
	uxth	r3, r3
.L191:
	.loc 3 964 0 is_stmt 0 discriminator 1
	cmp	r3, #63
	bls	.L194
	.loc 3 970 0 is_stmt 1
	cmp	r3, #64
	.loc 3 971 0
	addeq	r2, r2, r1, asl #1
	streqh	r0, [r2, #2]	@ movhi
	ldmfd	sp!, {r4, r5, pc}
.L196:
	.align	2
.L195:
	.word	.LANCHOR1
	.word	-3944
.LFE169:
	.fnend
	.size	sftl_add_sblk_to_wait_gc_pool, .-sftl_add_sblk_to_wait_gc_pool
	.align	2
	.global	user_gc_get_wait_offs
	.type	user_gc_get_wait_offs, %function
user_gc_get_wait_offs:
	.fnstart
.LFB170:
	.loc 3 980 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL173:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
.LCFI27:
	.loc 3 988 0
	mov	r3, #0
	.loc 3 990 0
	ldr	ip, .L213
	movw	r2, #65535
	.loc 3 988 0
	strb	r3, [r1, #0]
	.loc 3 990 0
	ldr	r3, .L213+4
	ldrh	r3, [ip, r3]
	cmp	r3, r2
	beq	.L198
	.loc 3 991 0
	cmp	r3, #63
	beq	.L199
	.loc 3 992 0
	ldr	r0, [ip, #-3948]
.LVL174:
	add	r3, r0, r3, asl #1
	.loc 3 991 0 discriminator 1
	ldrh	r3, [r3, #2]
	cmp	r3, r2
	bne	.L200
.LVL175:
.L199:
	.loc 3 993 0
	mov	r3, #1
	strb	r3, [r1, #0]
.L200:
	.loc 3 994 0
	ldr	r3, .L213+4
	ldr	r2, .L213
	ldrh	r0, [r2, r3]
.LVL176:
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.LVL177:
.L198:
	ldr	r3, .L213+8
	.loc 3 998 0
	cmp	r0, #0
	.loc 3 1004 0
	movw	r1, #65535
.LVL178:
	.loc 3 1005 0
	mov	r2, #0
.LVL179:
	.loc 3 1003 0
	mov	r7, r1
.LVL180:
	.loc 3 1006 0 discriminator 1
	ldr	ip, .L213+8
	.loc 3 999 0
	ldrh	r8, [r3, #22]
	.loc 3 1007 0
	movw	r9, #1732
	.loc 3 1006 0
	ldrh	r3, [r3, #4]
.LVL181:
	.loc 3 1008 0
	ldr	r6, .L213
	.loc 3 999 0
	movne	r8, r8, lsr #1
.LVL182:
	.loc 3 1001 0
	moveq	r8, r8, lsr #2
.LVL183:
	.loc 3 1006 0
	b	.L204
.L209:
	.loc 3 1007 0
	ldrh	r4, [ip, r9]
	cmp	r4, r3
	beq	.L205
	.loc 3 1008 0
	ldr	r4, [r6, #-3908]
	mov	sl, r3, asl #2
	add	r5, r4, sl
	ldrb	r5, [r5, #3]	@ zero_extendqisi2
	.loc 3 1007 0 discriminator 1
	and	r5, r5, #14
	cmp	r5, #4
	bne	.L205
	.loc 3 1009 0
	ldr	r5, [r4, r3, asl #2]
	ubfx	r5, r5, #16, #9
	cmp	r5, r7
	.loc 3 1010 0
	movlt	r7, r5
	movlt	r1, r3
	.loc 3 1013 0
	cmp	r5, r8
	blt	.L207
	.loc 3 1014 0
	ldrh	r5, [r4, sl]
	.loc 3 1013 0 discriminator 1
	ldr	r4, [ip, #2064]
	.loc 3 1014 0
	add	r5, r5, #256
	.loc 3 1013 0 discriminator 1
	cmp	r5, r4
	bcs	.L205
.L207:
	.loc 3 1018 0
	mov	r4, r2, asl #1
	add	r2, r2, #1
.LVL184:
	ldr	r5, [r6, #-3948]
	uxth	r2, r2
.LVL185:
	.loc 3 1019 0
	cmp	r2, #63
	.loc 3 1018 0
	strh	r3, [r5, r4]	@ movhi
	.loc 3 1019 0
	bhi	.L208
.L205:
	.loc 3 1006 0
	add	r3, r3, #1
.LVL186:
	uxth	r3, r3
.LVL187:
.L204:
	.loc 3 1006 0 is_stmt 0 discriminator 1
	ldrh	r4, [ip, #18]
	cmp	r4, r3
	bhi	.L209
	.loc 3 1025 0 is_stmt 1
	cmp	r2, #0
	beq	.L210
.L208:
	.loc 3 1026 0
	ldr	r3, .L213
.LVL188:
	mov	r2, #0
.LVL189:
	b	.L212
.LVL190:
.L210:
	.loc 3 1028 0
	cmp	r0, #0
	beq	.L211
	.loc 3 1029 0
	ldr	r3, .L213
.LVL191:
	ldr	r0, [r3, #-3948]
.LVL192:
	strh	r1, [r0, #0]	@ movhi
.LVL193:
.L212:
	.loc 3 1033 0
	ldr	r1, .L213+12
.LVL194:
	strh	r2, [r3, r1]	@ movhi
	.loc 3 1034 0
	sub	r1, r1, #2
	strh	r2, [r3, r1]	@ movhi
.LVL195:
.L211:
	.loc 3 1037 0
	ldr	r3, .L213+4
.LVL196:
	ldr	r2, .L213
.LVL197:
	ldrh	r0, [r2, r3]
.LVL198:
	.loc 3 1038 0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L214:
	.align	2
.L213:
	.word	.LANCHOR1
	.word	-3944
	.word	.LANCHOR0
	.word	-3942
.LFE170:
	.fnend
	.size	user_gc_get_wait_offs, .-user_gc_get_wait_offs
	.align	2
	.global	sftl_sys_mark_bad_blk_flag
	.type	sftl_sys_mark_bad_blk_flag, %function
sftl_sys_mark_bad_blk_flag:
	.fnstart
.LFB176:
	.loc 3 1338 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL199:
	.loc 3 1339 0
	ldr	r2, .L219
	.loc 3 1338 0
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
.LCFI28:
	.loc 3 1338 0
	mov	r3, r0
	.loc 3 1339 0
	ldrh	r0, [r2, #18]
.LVL200:
	cmp	r0, r1
	bls	.L216
	.loc 3 1339 0 is_stmt 0 discriminator 2
	ldrh	r2, [r2, #14]
	cmp	r2, r3
	bhi	.L217
.L216:
.LBB215:
.LBB216:
	.loc 3 1339 0 discriminator 3
	ldr	r1, .L219+4
.LVL201:
	movw	r2, #1339
	ldr	r0, .L219+8
	bl	printk
.LVL202:
	.loc 3 1339 0 discriminator 4
	bl	dump_ftl_info
.L218:
	b	.L218
.LVL203:
.L217:
.LBE216:
.LBE215:
	.loc 3 1340 0 is_stmt 1
	ldr	r4, .L219+12
	mov	r5, #1
	ldr	r2, [r4, #-3908]
	ldr	r0, [r2, r1, asl #2]
	ubfx	ip, r0, #28, #4
	orr	r3, ip, r5, asl r3
.LVL204:
	bfi	r0, r3, #28, #4
	str	r0, [r2, r1, asl #2]
	.loc 3 1341 0
	mov	r0, r1
	mov	r1, #384
.LVL205:
	bl	__aeabi_uidiv
	ldr	r3, [r4, #-3932]
	uxth	r0, r0
	str	r0, [r4, #-3928]
	orr	r5, r3, r5, asl r0
	str	r5, [r4, #-3932]
	.loc 3 1342 0
	ldmfd	sp!, {r3, r4, r5, pc}
.L220:
	.align	2
.L219:
	.word	.LANCHOR0
	.word	.LC7
	.word	.LC5
	.word	.LANCHOR1
.LFE176:
	.fnend
	.size	sftl_sys_mark_bad_blk_flag, .-sftl_sys_mark_bad_blk_flag
	.align	2
	.global	sftl_free_sblk
	.type	sftl_free_sblk, %function
sftl_free_sblk:
	.fnstart
.LFB177:
	.loc 3 1345 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL206:
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
.LCFI29:
	.loc 3 1346 0
	ldr	r4, .L222
	ldr	r2, [r4, #-3908]
	ldr	r1, [r2, r0, asl #2]
	bfc	r1, #25, #3
	str	r1, [r2, r0, asl #2]
	.loc 3 1347 0
	ldr	r2, [r4, #-3908]
	ldr	r1, [r2, r0, asl #2]
	bfc	r1, #16, #9
	str	r1, [r2, r0, asl #2]
	.loc 3 1349 0
	mov	r1, #384
	bl	__aeabi_uidiv
.LVL207:
	ldr	r3, [r4, #-3932]
	mov	r2, #1
	uxth	r0, r0
	str	r0, [r4, #-3928]
	orr	r3, r3, r2, asl r0
	str	r3, [r4, #-3932]
	.loc 3 1350 0
	ldmfd	sp!, {r4, pc}
.L223:
	.align	2
.L222:
	.word	.LANCHOR1
.LFE177:
	.fnend
	.size	sftl_free_sblk, .-sftl_free_sblk
	.align	2
	.global	sftl_format_erase_sblk
	.type	sftl_format_erase_sblk, %function
sftl_format_erase_sblk:
	.fnstart
.LFB178:
	.loc 3 1353 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL208:
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r3, r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI30:
	.loc 3 1353 0
	mov	r4, r0
	.loc 3 1362 0
	ldr	fp, .L240
	ldrh	r3, [fp, #18]
	cmp	r3, r0
	bls	.L225
	.loc 3 1362 0 is_stmt 0 discriminator 2
	ldrh	r3, [fp, #4]
	cmp	r3, r0
	bls	.L238
.L225:
	.loc 3 1362 0 discriminator 3
	ldr	r1, .L240+4
	movw	r2, #1362
	ldr	r0, .L240+8
.LVL209:
	bl	printk
	.loc 3 1362 0 discriminator 4
	bl	dump_ftl_info
.L227:
	b	.L227
.LVL210:
.L230:
	.loc 3 1365 0 is_stmt 1
	mul	r3, r9, r5
	ldr	r2, [r8, #-3904]
	mov	r1, #0
	str	r1, [r2, r3]
.LBB217:
.LBB218:
	.loc 3 10 0
	add	r3, r8, r5, asl #1
	sub	r3, r3, #3968
	ldrh	sl, [fp, #16]
	sub	r3, r3, #8
	ldrh	r3, [r3, #0]
	mla	sl, sl, r4, r3
.LVL211:
.LBE218:
.LBE217:
	.loc 3 1367 0
	mov	r0, sl
	bl	sftl_bbt_bad_blk_check
	cmp	r0, #0
	bne	.L228
	.loc 3 1368 0
	ldr	r3, [r8, #-3904]
	mov	sl, sl, asl #11
.LVL212:
	.loc 3 1369 0
	ldr	r2, [r8, #-3924]
	.loc 3 1368 0
	mla	r3, r9, r7, r3
	.loc 3 1371 0
	add	r7, r7, #1
.LVL213:
	uxth	r7, r7
.LVL214:
	.loc 3 1369 0
	str	r2, [r3, #8]
	.loc 3 1370 0
	ldr	r2, [r8, #-3916]
	.loc 3 1368 0
	str	sl, [r3, #4]
	.loc 3 1370 0
	str	r2, [r3, #12]
	b	.L229
.LVL215:
.L228:
	.loc 3 1373 0
	add	r6, r6, #1
	.loc 3 1374 0
	mov	r0, r5
	mov	r1, r4
	.loc 3 1373 0
	uxth	r6, r6
	.loc 3 1374 0
	bl	sftl_sys_mark_bad_blk_flag
.L229:
	.loc 3 1364 0
	add	r5, r5, #1
.LVL216:
	uxth	r5, r5
.LVL217:
	b	.L226
.LVL218:
.L238:
	ldr	r8, .L240+12
	.loc 3 1362 0 discriminator 2
	mov	r5, #0
.LVL219:
	mov	r7, r5
.LVL220:
	mov	r6, r5
	.loc 3 1365 0
	mov	r9, #20
.LVL221:
.L226:
	.loc 3 1364 0 discriminator 1
	ldrh	r3, [fp, #14]
	ldr	sl, .L240+12
.LVL222:
	cmp	r3, r5
	bhi	.L230
	.loc 3 1379 0
	cmp	r7, #0
	.loc 3 1380 0
	ldreq	r3, [sl, #-3908]
	.loc 3 1379 0
	beq	.L239
	.loc 3 1384 0
	mov	r1, r7
	ldr	r0, [sl, #-3904]
	bl	ftl_flash_erase_blocks
	.loc 3 1352 0
	mov	r3, #20
	mul	r7, r3, r7
.LVL223:
	.loc 3 1385 0
	mov	r5, #0
.LVL224:
.LBB219:
.LBB220:
	.loc 3 19 0
	ldr	r8, .L240
.LBE220:
.LBE219:
	.loc 3 1385 0
	b	.L233
.L235:
	.loc 3 1386 0
	ldr	r3, [sl, #-3904]
	add	r2, r3, r5
	ldr	r3, [r3, r5]
	cmn	r3, #1
	bne	.L234
	.loc 3 1388 0
	ldr	r0, [r2, #4]
	.loc 3 1387 0
	add	r6, r6, #1
	uxth	r6, r6
	.loc 3 1388 0
	mov	r0, r0, lsr #11
	bl	sftl_bbt_mark_bad_blk
	.loc 3 1389 0
	ldr	r3, [sl, #-3904]
.LBB223:
.LBB221:
	.loc 3 19 0
	ldrh	r1, [r8, #16]
.LBE221:
.LBE223:
	.loc 3 1389 0
	add	r3, r3, r5
	ldr	r0, [r3, #4]
.LBB224:
.LBB222:
	.loc 3 19 0
	mov	r9, r0, lsr #25
	.loc 3 20 0
	mov	r0, r0, lsr #11
	.loc 3 19 0
	mul	r9, r9, r1
	.loc 3 20 0
	bl	__aeabi_uidivmod
	.loc 3 19 0
	add	r1, r9, r1
.LBE222:
.LBE224:
	.loc 3 1390 0
	uxth	r0, r1
	mov	r1, r4
	bl	sftl_sys_mark_bad_blk_flag
.L234:
	add	r5, r5, #20
.L233:
	.loc 3 1385 0 discriminator 1
	cmp	r5, r7
	ldr	r3, .L240+12
	bne	.L235
	.loc 3 1395 0
	ldr	r2, .L240
	ldrh	r1, [r2, #14]
	cmp	r1, r6
	bne	.L236
	.loc 3 1396 0
	ldr	r3, [r3, #-3908]
.LVL225:
.L239:
	ldr	r2, [r3, r4, asl #2]
	mov	r1, #3
	bfi	r2, r1, #25, #3
	str	r2, [r3, r4, asl #2]
	.loc 3 1397 0
	b	.L232
.LVL226:
.L236:
	.loc 3 1400 0
	ldr	r0, [r3, #-3908]
	mov	r1, r4, asl #2
	ldr	ip, [r0, r4, asl #2]
	bfc	ip, #25, #3
	str	ip, [r0, r4, asl #2]
	.loc 3 1401 0
	ldr	r0, [r3, #-3908]
	ldr	ip, [r0, r4, asl #2]
	bfc	ip, #16, #9
	str	ip, [r0, r4, asl #2]
	.loc 3 1403 0
	ldr	ip, [r3, #-3908]
	ldrh	r0, [ip, r1]
	add	r0, r0, #1
	uxth	r0, r0
	strh	r0, [ip, r1]	@ movhi
	ldr	ip, [r2, #2064]
	cmp	r0, ip
	.loc 3 1405 0
	mov	r0, r4
	.loc 3 1404 0
	ldrcc	r3, [r3, #-3908]
	ldrcch	r3, [r3, r1]
	.loc 3 1405 0
	mov	r1, #384
	.loc 3 1404 0
	strcc	r3, [r2, #2064]
	.loc 3 1405 0
	bl	__aeabi_uidiv
	ldr	r3, .L240+12
	mov	r1, #1
	ldr	r2, [r3, #-3932]
	uxth	r0, r0
	str	r0, [r3, #-3928]
	orr	r2, r2, r1, asl r0
	str	r2, [r3, #-3932]
.LVL227:
.L232:
	.loc 3 1408 0
	mov	r0, r6
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L241:
	.align	2
.L240:
	.word	.LANCHOR0
	.word	.LC7
	.word	.LC5
	.word	.LANCHOR1
.LFE178:
	.fnend
	.size	sftl_format_erase_sblk, .-sftl_format_erase_sblk
	.align	2
	.global	sys_check_sblk_cnt
	.type	sys_check_sblk_cnt, %function
sys_check_sblk_cnt:
	.fnstart
.LFB179:
	.loc 3 1425 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
.LCFI31:
	.loc 3 1430 0
	mov	r5, #0
.LVL228:
	.loc 3 1436 0
	ldr	r6, .L250
	.loc 3 1437 0
	ldr	r7, .L250+4
	.loc 3 1436 0
	ldrh	r4, [r6, #4]
.LVL229:
	b	.L243
.L248:
	.loc 3 1437 0
	ldr	r3, [r7, #-3908]
	add	r3, r3, r4, asl #2
	ldrb	r3, [r3, #3]	@ zero_extendqisi2
	and	r3, r3, #14
	cmp	r3, #2
	bne	.L244
	.loc 3 1440 0 discriminator 1
	ldrh	r1, [r6, #36]
	mov	r3, #0
.LVL230:
	ldr	r2, .L250+8
	b	.L245
.L247:
	.loc 3 1441 0
	ldrh	r0, [r2, #2]!
	cmp	r0, r4
	beq	.L246
	.loc 3 1440 0
	add	r3, r3, #1
.LVL231:
	uxth	r3, r3
.LVL232:
.L245:
	.loc 3 1440 0 is_stmt 0 discriminator 1
	cmp	r3, r1
	bne	.L247
	.loc 3 1446 0 is_stmt 1
	mov	r0, r4
	bl	sftl_free_sblk
.LVL233:
	b	.L246
.L244:
	.loc 3 1449 0
	cmp	r3, #4
	.loc 3 1450 0
	addeq	r5, r5, #1
.LVL234:
	uxtheq	r5, r5
.LVL235:
.L246:
	.loc 3 1436 0
	add	r4, r4, #1
.LVL236:
	uxth	r4, r4
.LVL237:
.L243:
	.loc 3 1436 0 is_stmt 0 discriminator 1
	ldrh	r3, [r6, #18]
.LVL238:
	ldr	r2, .L250
	cmp	r3, r4
	bhi	.L248
	.loc 3 1459 0 is_stmt 1
	mov	r3, #1728
	.loc 3 1468 0
	mov	r0, #0
	.loc 3 1459 0
	ldrh	r1, [r2, r3]
	add	r1, r1, #1
	cmp	r5, r1
	.loc 3 1464 0
	subne	r5, r5, #1
.LVL239:
	strneh	r5, [r2, r3]	@ movhi
	.loc 3 1468 0
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L251:
	.align	2
.L250:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+1598
.LFE179:
	.fnend
	.size	sys_check_sblk_cnt, .-sys_check_sblk_cnt
	.align	2
	.global	sftl_sys_blk_init
	.type	sftl_sys_blk_init, %function
sftl_sys_blk_init:
	.fnstart
.LFB180:
	.loc 3 1471 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, r4, r5, r6, r7, lr}
	.save {r3, r4, r5, r6, r7, lr}
.LCFI32:
	.loc 3 1475 0
	movw	r2, #4152
	ldr	r3, .L258
	.loc 3 1480 0
	ldr	r5, .L258+4
	ldr	r6, .L258+8
	.loc 3 1475 0
	ldrh	r2, [r3, r2]
	ldrh	r1, [r3, #0]
	cmp	r1, r2
	.loc 3 1476 0
	addls	r2, r2, #1
	strlsh	r2, [r3, #0]	@ movhi
	.loc 3 1477 0
	movw	r2, #4154
	ldrh	r2, [r3, r2]
	ldrh	r3, [r3, #0]
	cmp	r3, r2
	.loc 3 1478 0
	addls	r2, r2, #1
	ldrls	r3, .L258
	strlsh	r2, [r3, #0]	@ movhi
	.loc 3 1480 0
	mov	r3, #0
	strh	r3, [r5, r6]	@ movhi
	.loc 3 1481 0
	ldr	r3, .L258
	ldrh	r4, [r3, #0]
	.loc 3 1481 0 is_stmt 0 discriminator 1
	add	r7, r4, #12
	.loc 3 1481 0
	b	.L255
.L257:
	.loc 3 1482 0 is_stmt 1
	mov	r0, r4
	bl	sftl_bbt_bad_blk_check
	cmp	r0, #0
	bne	.L256
	.loc 3 1483 0
	ldrh	r2, [r5, r6]
	add	r3, r5, r2, asl #1
	add	r2, r2, #1
	sub	r3, r3, #3856
	strh	r2, [r5, r6]	@ movhi
	strh	r4, [r3, #-8]	@ movhi
.L256:
	.loc 3 1481 0
	add	r4, r4, #1
	uxth	r4, r4
.L255:
	.loc 3 1481 0 is_stmt 0 discriminator 1
	cmp	r4, r7
	blt	.L257
	.loc 3 1484 0 is_stmt 1
	ldmfd	sp!, {r3, r4, r5, r6, r7, pc}
.L259:
	.align	2
.L258:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	-2924
.LFE180:
	.fnend
	.size	sftl_sys_blk_init, .-sftl_sys_blk_init
	.align	2
	.global	sftl_sys_info_load
	.type	sftl_sys_info_load, %function
sftl_sys_info_load:
	.fnstart
.LFB181:
	.loc 3 1487 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
.LCFI33:
	.loc 3 1494 0
	mov	r3, #0
	ldr	r4, .L281
.LBB225:
	.loc 3 1495 0 discriminator 3
	mov	r1, #2064
	ldr	r0, .L281+4
.LBE225:
	.loc 3 1513 0
	ldr	r9, .L281+8
	.loc 3 1494 0
	str	r3, [r4, #-3932]
	mov	sl, r4
.LBB226:
	.loc 3 1495 0 discriminator 3
	bl	__memzero
.LBE226:
	.loc 3 1496 0
	bl	sftl_sys_blk_init
	.loc 3 1498 0
	ldr	r3, [r4, #-3916]
	.loc 3 1497 0
	ldr	r8, [r4, #-3924]
	.loc 3 1498 0
	str	r3, [r4, #-3828]
	.loc 3 1502 0
	ldr	r3, .L281+12
	.loc 3 1497 0
	str	r8, [r4, #-3832]
	.loc 3 1502 0
	ldrh	r6, [r4, r3]
	sub	r6, r6, #1
	uxth	r6, r6
.LVL240:
	b	.L261
.LVL241:
.L265:
	.loc 3 1503 0
	add	r3, r4, r7, asl #1
	.loc 3 1504 0
	ldr	r0, .L281+16
	.loc 3 1503 0
	sub	r3, r3, #3856
	.loc 3 1504 0
	mov	r1, #1
	.loc 3 1503 0
	sub	r3, r3, #8
	.loc 3 1504 0
	mov	r2, #0
	.loc 3 1503 0
	ldrh	r3, [r3, #0]
	mov	r3, r3, asl #11
	str	r3, [r4, #-3836]
	.loc 3 1504 0
	bl	ftl_flash_read_pages
	.loc 3 1505 0
	ldr	r3, [r4, #-3840]
	cmn	r3, #1
	bne	.L262
	.loc 3 1507 0
	ldr	r3, [r4, #-3836]
	.loc 3 1508 0
	mov	r1, #1
	ldr	r0, .L281+16
	mov	r2, #0
	.loc 3 1507 0
	add	r3, r3, #1
	str	r3, [r4, #-3836]
	.loc 3 1508 0
	bl	ftl_flash_read_pages
.L262:
	.loc 3 1510 0
	ldr	r3, [sl, #-3840]
	ldr	r5, .L281
	cmn	r3, #1
	beq	.L263
	.loc 3 1513 0
	ldr	r3, [r8, #2044]
	cmp	r3, r9
	beq	.L264
.L263:
	.loc 3 1502 0
	sub	r6, r6, #1
	uxth	r6, r6
.LVL242:
.L261:
	.loc 3 1502 0 is_stmt 0 discriminator 1
	sxth	r7, r6
.LVL243:
	cmp	r7, #0
	bge	.L265
	.loc 3 1520 0 is_stmt 1
	mvn	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L264:
	.loc 3 1522 0
	ldr	r3, .L281+20
	add	r7, r5, r7, asl #1
.LVL244:
	.loc 3 1524 0
	ldr	r0, [r5, #-3924]
	mov	r2, #0
	.loc 3 1526 0
	mov	r4, #0
	.loc 3 1522 0
	ldrh	r1, [r7, r3]
	ldr	r3, .L281+24
	strh	r1, [r5, r3]	@ movhi
	.loc 3 1523 0
	add	r3, r3, #2
	strh	r6, [r5, r3]	@ movhi
.LVL245:
	.loc 3 1524 0
	bl	sftl_get_last_written_page
	.loc 3 1528 0
	ldr	r6, .L281+28
.LVL246:
	add	sl, r6, #48
	.loc 3 1525 0
	add	r8, r0, #1
	.loc 3 1524 0
	mov	r7, r0
	.loc 3 1525 0
	uxth	r8, r8
	.loc 3 1526 0
	b	.L267
.L270:
	.loc 3 1527 0
	ldr	r2, .L281+24
	.loc 3 1530 0
	mov	r1, #1
	.loc 3 1528 0
	str	sl, [r5, #-3832]
	.loc 3 1530 0
	ldr	r0, .L281+16
	.loc 3 1527 0
	ldrh	r2, [r5, r2]
	orr	r3, r3, r2, asl #11
	str	r3, [r5, #-3836]
	.loc 3 1529 0
	ldr	r3, .L281+32
	.loc 3 1530 0
	mov	r2, #0
	.loc 3 1529 0
	str	r3, [r5, #-3828]
	.loc 3 1530 0
	bl	ftl_flash_read_pages
	.loc 3 1531 0
	ldr	r3, [r5, #-3840]
	cmn	r3, #1
	beq	.L268
	.loc 3 1531 0 is_stmt 0 discriminator 1
	ldr	r2, [r6, #2092]
	ldr	r3, .L281+8
	cmp	r2, r3
	beq	.L269
.L268:
	sub	r4, r4, #1
	uxth	r4, r4
.L267:
	.loc 3 1486 0 is_stmt 1
	add	r3, r4, r7
	.loc 3 1526 0 discriminator 1
	sxth	r3, r3
	cmp	r3, #0
	bge	.L270
.L269:
	.loc 3 1535 0
	ldr	r4, .L281
	mov	r6, #0
	ldr	r3, .L281+36
	mov	r5, r6
	ldr	r7, .L281+40
	strh	r8, [r4, r3]	@ movhi
	.loc 3 1544 0 discriminator 2
	ldr	r8, .L281+28
.LVL247:
.L277:
	.loc 3 1538 0
	ldr	r3, [r7, #4]!
	.loc 3 1539 0
	cmp	r3, #0
	beq	.L271
	.loc 3 1540 0
	str	r3, [r4, #-3836]
	.loc 3 1543 0
	mov	r1, #1
	.loc 3 1541 0
	ldr	r3, [r4, #-3924]
.LVL248:
	.loc 3 1543 0
	mov	r2, #0
	ldr	r0, .L281+16
	.loc 3 1541 0
	str	r3, [r4, #-3832]
	.loc 3 1542 0
	ldr	r3, [r4, #-3916]
	str	r3, [r4, #-3828]
	.loc 3 1543 0
	bl	ftl_flash_read_pages
	.loc 3 1544 0
	ldr	r3, [r4, #-3840]
	cmn	r3, #1
	beq	.L272
	.loc 3 1544 0 is_stmt 0 discriminator 2
	ldr	r2, [r8, #2092]
	ldr	r3, .L281+8
	cmp	r2, r3
	beq	.L273
.L272:
	.loc 3 1544 0 discriminator 3
	ldr	r1, .L281+44
	movw	r2, #1544
	ldr	r0, .L281+48
	bl	printk
	.loc 3 1544 0 discriminator 4
	bl	dump_ftl_info
.L274:
	b	.L274
.L273:
	.loc 3 1545 0 is_stmt 1
	ldr	r3, [r4, #-3916]
	ldr	r3, [r3, #4]
	cmp	r3, r5
	beq	.L275
	.loc 3 1545 0 is_stmt 0 discriminator 2
	ldr	r1, .L281+44
	movw	r2, #1545
	ldr	r0, .L281+48
	bl	printk
	.loc 3 1545 0 discriminator 3
	bl	dump_ftl_info
.L276:
	b	.L276
.L275:
	.loc 3 1546 0 is_stmt 1
	ldr	r0, [r4, #-3908]
	mov	r2, #1536
	ldr	r1, [r4, #-3924]
	add	r0, r0, r6
	bl	memcpy
.LVL249:
.L271:
	add	r5, r5, #1
	add	r6, r6, #1536
	.loc 3 1537 0 discriminator 1
	cmp	r5, #4
	bne	.L277
	.loc 3 1551 0
	ldr	r3, .L281+28
.LVL250:
	.loc 3 1558 0
	mov	r4, #0
	.loc 3 1551 0
	ldr	r2, .L281
	ldr	r1, [r3, #2068]
	str	r1, [r2, #-3968]
	.loc 3 1552 0
	ldr	r1, [r3, #2072]
	str	r1, [r2, #-3964]
	.loc 3 1556 0
	ldr	r1, [r3, #2084]
	.loc 3 1558 0
	str	r4, [r3, #2084]
	.loc 3 1556 0
	tst	r1, #1
	.loc 3 1560 0
	ldr	r3, .L281
	.loc 3 1557 0
	moveq	r1, #1
	streqb	r1, [r2, #-3960]
	.loc 3 1560 0
	ldrb	r0, [r2, #-3960]	@ zero_extendqisi2
	cmp	r0, r4
	ldmeqfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
	.loc 3 1561 0
	mov	r2, #1
	strb	r2, [r3, #-3959]
	.loc 3 1562 0
	strb	r2, [r3, #-3958]
	.loc 3 1563 0
	strb	r2, [r3, #-3957]
	.loc 3 1564 0
	bl	sys_check_sblk_cnt
	.loc 3 1567 0
	mov	r0, r4
	.loc 3 1568 0
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L282:
	.align	2
.L281:
	.word	.LANCHOR1
	.word	.LANCHOR0+48
	.word	1398362953
	.word	-2924
	.word	.LANCHOR1-3840
	.word	-3864
	.word	-2922
	.word	.LANCHOR0
	.word	.LANCHOR0+2096
	.word	-2918
	.word	.LANCHOR0+1580
	.word	.LC7
	.word	.LC5
.LFE181:
	.fnend
	.size	sftl_sys_info_load, .-sftl_sys_info_load
	.align	2
	.global	sftl_sys_info_write
	.type	sftl_sys_info_write, %function
sftl_sys_info_write:
	.fnstart
.LFB182:
	.loc 3 1571 0
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL251:
	stmfd	sp!, {r4, r5, r6, r7, r8, sl, lr}
	.save {r4, r5, r6, r7, r8, sl, lr}
.LCFI34:
	.loc 3 1574 0
	add	r5, r0, #384
	ldr	r8, .L289
	.loc 3 1577 0
	cmp	r0, #3
	.loc 3 1571 0
	.pad #28
	sub	sp, sp, #28
.LCFI35:
	.loc 3 1571 0
	mov	sl, r0
	.loc 3 1574 0
	add	r5, r8, r5, asl #2
	ldr	r7, [r5, #48]
.LVL252:
	.loc 3 1577 0
	bls	.L284
	.loc 3 1577 0 is_stmt 0 discriminator 2
	ldr	r1, .L289+4
	movw	r2, #1577
	ldr	r0, .L289+8
.LVL253:
	bl	printk
	.loc 3 1577 0 discriminator 3
	bl	dump_ftl_info
.L285:
	b	.L285
.L284:
	.loc 3 1579 0 is_stmt 1
	ldr	r4, .L289+12
	.loc 3 1582 0
	ldr	r6, .L289+16
	.loc 3 1579 0
	ldr	r3, [r4, #-3968]
	.loc 3 1587 0
	ldr	r1, [r4, #-3908]
	.loc 3 1585 0
	ldr	r0, [r4, #-3924]
	.loc 3 1579 0
	str	r3, [r8, #2068]
	.loc 3 1580 0
	ldr	r3, [r4, #-3964]
	.loc 3 1585 0
	str	r0, [sp, #12]
	.loc 3 1580 0
	str	r3, [r8, #2072]
	.loc 3 1582 0
	ldr	r3, .L289+20
	ldrh	r2, [r4, r3]
	ldrh	r3, [r4, r6]
	orr	r3, r3, r2, asl #11
.LVL254:
	.loc 3 1583 0
	str	r3, [r5, #48]
	.loc 3 1584 0
	str	r3, [sp, #8]
	.loc 3 1587 0
	mov	r2, #1536
	.loc 3 1586 0
	ldr	r3, [r4, #-3916]
.LVL255:
	.loc 3 1587 0
	mla	r1, r2, sl, r1
	.loc 3 1586 0
	str	r3, [sp, #16]
	.loc 3 1587 0
	bl	memcpy
	.loc 3 1588 0
	ldr	r0, [r4, #-3924]
	add	r1, r8, #1584
	ldr	r2, [r8, #8]
	add	r0, r0, #1536
	sub	r2, r2, #1536
	bl	memcpy
	.loc 3 1589 0
	ldr	r3, [r8, #2096]
	.loc 3 1591 0
	add	r1, r8, #2096
	mov	r2, #8
	ldr	r0, [r4, #-3916]
	.loc 3 1589 0
	add	r3, r3, #1
	.loc 3 1590 0
	str	sl, [r8, #2100]
	.loc 3 1589 0
	str	r3, [r8, #2096]
	.loc 3 1591 0
	bl	memcpy
	.loc 3 1592 0
	mov	r1, #1
	add	r0, sp, #4
	mov	r3, r1
	mov	r2, #0
	bl	ftl_flash_prog_pages
	.loc 3 1593 0
	ldrh	r3, [r4, r6]
	add	r3, r3, #1
	strh	r3, [r4, r6]	@ movhi
	.loc 3 1598 0
	ldrb	r3, [r4, #-3959]	@ zero_extendqisi2
	cmp	r3, #0
	.loc 3 1599 0
	movne	r3, #0
	.loc 3 1600 0
	mvnne	r0, #0
	.loc 3 1599 0
	strneb	r3, [r4, #-3959]
	.loc 3 1598 0
	bne	.L287
	.loc 3 1602 0
	ldr	r0, [sp, #4]
	cmp	r0, #0
	.loc 3 1603 0
	strne	r7, [r5, #48]
.L287:
	.loc 3 1606 0
	add	sp, sp, #28
	ldmfd	sp!, {r4, r5, r6, r7, r8, sl, pc}
.L290:
	.align	2
.L289:
	.word	.LANCHOR0
	.word	.LC7
	.word	.LC5
	.word	.LANCHOR1
	.word	-2918
	.word	-2922
.LFE182:
	.fnend
	.size	sftl_sys_info_write, .-sftl_sys_info_write
	.align	2
	.global	sftl_sys_info_flush
	.type	sftl_sys_info_flush, %function
sftl_sys_info_flush:
	.fnstart
.LFB183:
	.loc 3 1609 0
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
.LCFI36:
	.pad #24
	sub	sp, sp, #24
.LCFI37:
	.loc 3 1615 0
	ldr	r4, .L320
	.loc 3 1614 0
	mov	r5, #0
.LVL256:
	mov	r7, r4
.L312:
	.loc 3 1615 0
	ldr	r3, [r4, #-3932]
	mov	r2, #1
	ands	r3, r3, r2, asl r5
	beq	.L292
.L313:
	.loc 3 1617 0
	ldr	r3, .L320+4
	ldrh	r2, [r4, r3]
	ldr	r3, .L320+8
	ldrh	r3, [r3, #20]
	cmp	r2, r3
	bne	.L293
	.loc 3 1618 0
	ldr	r3, .L320+12
	.loc 3 1624 0
	add	r0, sp, #4
	.loc 3 1619 0
	ldr	r1, .L320+16
	.loc 3 1618 0
	ldrh	r2, [r4, r3]
	.loc 3 1619 0
	ldrh	r1, [r4, r1]
	.loc 3 1618 0
	add	r2, r2, #1
	uxth	r2, r2
	strh	r2, [r4, r3]	@ movhi
	.loc 3 1619 0
	cmp	r1, r2
	.loc 3 1621 0
	mov	r1, #0
	.loc 3 1620 0
	movls	r2, #0
	strlsh	r2, [r4, r3]	@ movhi
	.loc 3 1622 0
	ldrh	r3, [r4, r3]
	.loc 3 1621 0
	ldr	r2, .L320+4
	.loc 3 1622 0
	add	r3, r4, r3, asl #1
	sub	r3, r3, #3856
	.loc 3 1621 0
	strh	r1, [r4, r2]	@ movhi
	.loc 3 1622 0
	sub	r3, r3, #8
	sub	r2, r2, #4
	.loc 3 1624 0
	mov	r1, #1
	.loc 3 1622 0
	ldrh	r3, [r3, #0]
	strh	r3, [r4, r2]	@ movhi
	.loc 3 1623 0
	mov	r3, r3, asl #11
	str	r3, [sp, #8]
	.loc 3 1624 0
	bl	ftl_flash_erase_blocks
.L293:
	.loc 3 1626 0
	ldr	r3, .L320+12
	ldrh	r2, [r4, r3]
	sub	r3, r3, #4
	ldrh	r3, [r4, r3]
	sub	r3, r3, #1
	cmp	r2, r3
	bne	.L295
	.loc 3 1626 0 is_stmt 0 discriminator 1
	ldr	r3, .L320+4
	ldrh	r6, [r4, r3]
	cmp	r6, #0
	bne	.L295
	ldr	r8, .L320+20
.L301:
	.loc 3 1629 0 is_stmt 1
	ldr	r3, [r8, #4]!
	cmp	r3, #0
	.loc 3 1630 0 discriminator 1
	ldrne	r9, .L320+4
	.loc 3 1629 0
	bne	.L318
	b	.L297
.L299:
	.loc 3 1631 0
	mov	r0, r6
	bl	sftl_sys_info_write
	cmn	r0, #1
	bne	.L298
.L318:
	.loc 3 1630 0 discriminator 1
	ldr	sl, .L320+8
	ldrh	r2, [r4, r9]
	ldrh	r3, [sl, #20]
	sub	r3, r3, #1
	cmp	r2, r3
	ble	.L299
.L298:
	.loc 3 1634 0
	ldr	r3, .L320+4
	ldrh	r2, [r7, r3]
	ldrh	r3, [sl, #20]
	cmp	r2, r3
	bcc	.L297
	.loc 3 1634 0 is_stmt 0 discriminator 2
	ldr	r1, .L320+24
	movw	r2, #1634
	ldr	r0, .L320+28
	bl	printk
	.loc 3 1634 0 discriminator 3
	bl	dump_ftl_info
.L300:
	b	.L300
.L297:
	.loc 3 1628 0 is_stmt 1
	add	r6, r6, #1
	.loc 3 1628 0 is_stmt 0 discriminator 1
	cmp	r6, #4
	bne	.L301
.L295:
	.loc 3 1638 0 is_stmt 1
	ldr	r3, .L320+12
	ldrh	r3, [r4, r3]
	cmp	r3, #0
	bne	.L302
	.loc 3 1638 0 is_stmt 0 discriminator 1
	ldr	r3, .L320+4
	ldrh	r6, [r4, r3]
	cmp	r6, #0
	bne	.L302
	.loc 3 1640 0 is_stmt 1
	ldr	r3, .L320+32
	.loc 3 1641 0
	add	r0, sp, #4
	mov	r1, #1
	ldr	r8, .L320+20
	.loc 3 1640 0
	ldrh	r3, [r4, r3]
	mov	r3, r3, asl #11
	str	r3, [sp, #8]
	.loc 3 1641 0
	bl	ftl_flash_erase_blocks
.L308:
	.loc 3 1643 0
	ldr	r3, [r8, #4]!
	cmp	r3, #0
	.loc 3 1644 0 discriminator 1
	ldrne	r9, .L320+4
	.loc 3 1643 0
	bne	.L319
	b	.L304
.L306:
	.loc 3 1645 0
	mov	r0, r6
	bl	sftl_sys_info_write
	cmn	r0, #1
	bne	.L305
.L319:
	.loc 3 1644 0 discriminator 1
	ldr	sl, .L320+8
	ldrh	r2, [r4, r9]
	ldrh	r3, [sl, #20]
	sub	r3, r3, #1
	cmp	r2, r3
	ble	.L306
.L305:
	.loc 3 1648 0
	ldr	r3, .L320+4
	ldrh	r2, [r7, r3]
	ldrh	r3, [sl, #20]
	cmp	r2, r3
	bcc	.L304
	.loc 3 1648 0 is_stmt 0 discriminator 2
	ldr	r1, .L320+24
	mov	r2, #1648
	ldr	r0, .L320+28
	bl	printk
	.loc 3 1648 0 discriminator 3
	bl	dump_ftl_info
.L307:
	b	.L307
.L304:
	.loc 3 1642 0 is_stmt 1
	add	r6, r6, #1
	.loc 3 1642 0 is_stmt 0 discriminator 1
	cmp	r6, #4
	bne	.L308
	ldr	r8, .L320+36
	mov	r6, #1
	.loc 3 1651 0 is_stmt 1 discriminator 1
	ldr	sl, .L320+16
	b	.L309
.L310:
	.loc 3 1652 0
	ldrh	r3, [r8, #2]!
	.loc 3 1653 0
	add	r0, sp, #4
	mov	r1, #1
	.loc 3 1651 0 discriminator 2
	add	r6, r6, #1
	.loc 3 1652 0
	mov	r3, r3, asl #11
	str	r3, [sp, #8]
	.loc 3 1653 0
	bl	ftl_flash_erase_blocks
.L309:
	.loc 3 1651 0 discriminator 1
	ldrh	r3, [r4, sl]
	cmp	r6, r3
	blt	.L310
.L302:
	.loc 3 1656 0
	mov	r0, r5
	bl	sftl_sys_info_write
	cmn	r0, #1
	beq	.L313
	.loc 3 1658 0
	ldr	r3, [r4, #-3932]
	mov	r2, #1
	bic	r3, r3, r2, asl r5
	str	r3, [r4, #-3932]
.L292:
	.loc 3 1614 0
	add	r5, r5, #1
	.loc 3 1614 0 is_stmt 0 discriminator 1
	cmp	r5, #4
	bne	.L312
	.loc 3 1662 0 is_stmt 1
	mov	r0, #0
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L321:
	.align	2
.L320:
	.word	.LANCHOR1
	.word	-2918
	.word	.LANCHOR0
	.word	-2920
	.word	-2924
	.word	.LANCHOR0+1580
	.word	.LC7
	.word	.LC5
	.word	-3864
	.word	.LANCHOR1-3864
.LFE183:
	.fnend
	.size	sftl_sys_info_flush, .-sftl_sys_info_flush
	.align	2
	.global	sftl_open_sblk
	.type	sftl_open_sblk, %function
sftl_open_sblk:
	.fnstart
.LFB156:
	.loc 3 185 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL257:
	.loc 3 194 0
	ldr	r2, .L339
	.loc 3 185 0
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
.LCFI38:
	.loc 3 194 0
	movw	r3, #2062
	ldrh	r3, [r2, r3]
	.loc 3 185 0
	mov	r7, r0
	.loc 3 200 0
	ldr	r1, .L339+4
	.loc 3 195 0
	ldrh	r6, [r2, #18]
	.loc 3 194 0
	add	r3, r3, #1
	.loc 3 195 0
	ldrh	r5, [r2, #4]
	.loc 3 201 0
	ldr	r8, [r2, #2064]
	.loc 3 192 0
	movw	r2, #65535
	.loc 3 200 0
	ldr	lr, [r1, #-3908]
	.loc 3 194 0
	uxth	r3, r3
	.loc 3 195 0
	rsb	ip, r5, r6
	.loc 3 191 0
	mov	r4, r2
.LVL258:
	.loc 3 193 0
	mov	r1, #0
.LVL259:
	.loc 3 194 0
	b	.L323
.L327:
	.loc 3 199 0
	cmp	r3, r6
	movcs	r3, r5
	.loc 3 200 0
	mov	r0, r3, asl #2
.LVL260:
	add	sl, lr, r0
	ldrb	sl, [sl, #3]	@ zero_extendqisi2
	tst	sl, #14
	bne	.L325
	.loc 3 201 0
	ldrh	r0, [lr, r0]
	cmp	r0, r8
	beq	.L337
	.loc 3 204 0
	cmp	r0, r2
	movcc	r2, r0
	movcc	r4, r3
.L325:
	.loc 3 196 0
	add	r3, r3, #1
	add	r1, r1, #1
.LVL261:
	uxth	r3, r3
	uxth	r1, r1
.LVL262:
.L323:
	.loc 3 194 0 discriminator 1
	cmp	r1, ip
	blt	.L327
	b	.L326
.L337:
	.loc 3 201 0
	mov	r4, r3
.L326:
	.loc 3 211 0
	movw	r3, #65535
	cmp	r4, r3
	bne	.L328
	.loc 3 211 0 is_stmt 0 discriminator 2
	ldr	r1, .L339+8
.LVL263:
	mov	r2, #211
	ldr	r0, .L339+12
	bl	printk
	.loc 3 211 0 discriminator 3
	bl	dump_ftl_info
.L329:
	b	.L329
.LVL264:
.L328:
	ldr	r5, .L339
	.loc 3 213 0 is_stmt 1
	cmp	r1, ip
	.loc 3 216 0
	mov	r0, r4
	.loc 3 217 0
	ldr	r6, .L339
	.loc 3 214 0
	streq	r2, [r5, #2064]
	.loc 3 216 0
	bl	sftl_format_erase_sblk
.LVL265:
	.loc 3 217 0
	movw	r3, #2062
	strh	r4, [r5, r3]	@ movhi
	.loc 3 221 0
	cmp	r7, #1
	.loc 3 219 0
	ldr	r5, .L339+4
	ldr	r3, [r5, #-3908]
	ldr	r2, [r3, r4, asl #2]
	bfi	r2, r7, #25, #3
	str	r2, [r3, r4, asl #2]
	.loc 3 220 0
	ldr	r3, [r5, #-3908]
	ldr	r0, [r3, r4, asl #2]
	mov	r0, r0, lsr #28
.LVL266:
	.loc 3 221 0
	beq	.L332
	cmp	r7, #2
	bne	.L331
	b	.L338
.L332:
	.loc 3 223 0
	ldr	r3, .L339+16
	.loc 3 225 0
	mov	r2, #0
	.loc 3 226 0
	ldr	r7, .L339+20
.LVL267:
	.loc 3 224 0
	str	r0, [r5, #-4004]
	.loc 3 223 0
	strh	r4, [r5, r3]	@ movhi
	.loc 3 225 0
	add	r3, r3, #6
	strh	r2, [r5, r3]	@ movhi
	.loc 3 226 0
	ldrh	r3, [r5, r7]
	.loc 3 227 0
	ldrh	r2, [r6, #36]
	.loc 3 226 0
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r5, r7]	@ movhi
	.loc 3 227 0
	cmp	r2, r3
	bhi	.L334
	.loc 3 227 0 is_stmt 0 discriminator 2
	ldr	r1, .L339+8
	mov	r2, #227
	ldr	r0, .L339+12
.LVL268:
	bl	printk
	.loc 3 227 0 discriminator 3
	bl	dump_ftl_info
.L335:
	b	.L335
.LVL269:
.L334:
	.loc 3 228 0 is_stmt 1
	bl	sftl_select_trace_offs
.LVL270:
	ldr	r3, .L339+24
.LBB227:
	.loc 3 233 0
	ldrh	r2, [r6, #22]
	movs	r2, r2, asl #1
.LVL271:
.LBE227:
	.loc 3 228 0
	strh	r0, [r5, r3]	@ movhi
	.loc 3 229 0
	ldrh	r3, [r5, r7]
.LBB228:
	.loc 3 233 0
	ldr	r0, [r5, #-3876]
.LVL272:
.LBE228:
	.loc 3 229 0
	add	r3, r3, #776
	add	r3, r6, r3, asl #1
	strh	r4, [r3, #48]	@ movhi
.LBB229:
	.loc 3 233 0
	beq	.L331
	.loc 3 233 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL273:
	b	.L331
.LVL274:
.L338:
.LBE229:
	.loc 3 236 0 is_stmt 1
	mov	r3, #1728
	.loc 3 241 0
	str	r0, [r5, #-3988]
	.loc 3 236 0
	ldrh	r2, [r6, r3]
	add	r2, r2, #1
	strh	r2, [r6, r3]	@ movhi
	.loc 3 237 0
	add	r3, r3, #4
	movw	r2, #1730
	ldrh	r1, [r6, r3]
	.loc 3 238 0
	strh	r4, [r6, r3]	@ movhi
	.loc 3 239 0
	mov	r3, #0
	.loc 3 237 0
	strh	r1, [r6, r2]	@ movhi
	.loc 3 239 0
	add	r2, r2, #4
	strh	r3, [r6, r2]	@ movhi
	.loc 3 240 0
	ldr	r2, .L339+28
	strh	r3, [r5, r2]	@ movhi
	.loc 3 242 0
	bl	sftl_select_trace_offs
.LVL275:
.LBB230:
	.loc 3 243 0
	ldrh	r2, [r6, #22]
.LBE230:
	.loc 3 242 0
	ldr	r3, .L339+32
.LBB231:
	.loc 3 243 0
	movs	r2, r2, asl #2
.LVL276:
.LBE231:
	.loc 3 242 0
	strh	r0, [r5, r3]	@ movhi
.LBB232:
	.loc 3 243 0
	ldr	r0, [r5, #-3868]
.LVL277:
	beq	.L336
	.loc 3 243 0 is_stmt 0 discriminator 4
	mov	r1, #255
	bl	memset
.LVL278:
.L336:
.LBE232:
	.loc 3 245 0 is_stmt 1
	bl	sftl_l2p_flush
.LVL279:
.L331:
	.loc 3 250 0
	mov	r0, r4
.LVL280:
	mov	r1, #384
	bl	__aeabi_uidiv
.LVL281:
	ldr	r3, .L339+4
	mov	r1, #1
	ldr	r2, [r3, #-3932]
	uxth	r0, r0
	str	r0, [r3, #-3928]
	orr	r2, r2, r1, asl r0
	str	r2, [r3, #-3932]
	.loc 3 252 0
	bl	sftl_sys_info_flush
	.loc 3 254 0
	mov	r0, r4
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L340:
	.align	2
.L339:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC7
	.word	.LC5
	.word	-4006
	.word	-4008
	.word	-3998
	.word	-3984
	.word	-3982
.LFE156:
	.fnend
	.size	sftl_open_sblk, .-sftl_open_sblk
	.align	2
	.global	sftl_write_page
	.type	sftl_write_page, %function
sftl_write_page:
	.fnstart
.LFB161:
	.loc 3 423 0
	@ args = 0, pretend = 0, frame = 64
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL282:
	.loc 3 430 0
	cmp	r1, #1
	.loc 3 423 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI39:
	mov	r4, r0
	.pad #68
	sub	sp, sp, #68
.LCFI40:
	.loc 3 423 0
	mov	r5, r1
	.loc 3 430 0
	beq	.L343
	cmp	r1, #2
	.loc 3 459 0
	mvnne	r0, #0
.LVL283:
	.loc 3 430 0
	bne	.L342
	b	.L365
.L343:
	.loc 3 440 0
	ldr	r6, .L366
	.loc 3 432 0
	ldr	r3, .L366+4
	.loc 3 442 0
	ldr	r8, .L366+8
	.loc 3 440 0
	ldr	r2, [r6, #-3888]
	.loc 3 432 0
	ldrh	r3, [r3, #38]
	.loc 3 437 0
	sub	lr, r8, #40
	.loc 3 441 0
	ldr	r9, [r6, #-3884]
	.loc 3 439 0
	sub	sl, r8, #34
.LVL284:
	.loc 3 440 0
	str	r2, [sp, #8]
	.loc 3 436 0
	sub	r6, r6, #4000
.LVL285:
	.loc 3 432 0
	str	r3, [sp, #20]
	.loc 3 435 0
	sub	r2, r8, #46
	.loc 3 438 0
	sub	r3, r8, #38
	.loc 3 437 0
	str	lr, [sp, #4]
	.loc 3 438 0
	str	r3, [sp, #12]
	.loc 3 434 0
	sub	fp, r8, #48
.LVL286:
	.loc 3 435 0
	str	r2, [sp, #16]
	.loc 3 433 0
	sub	r3, r8, #50
	b	.L364
.LVL287:
.L365:
	.loc 3 445 0
	ldr	r3, .L366+4
	ldr	r2, [r3, #44]
	cmp	r4, r2
	bls	.L346
	.loc 3 445 0 is_stmt 0 discriminator 2
	ldr	r1, .L366+12
.LVL288:
	movw	r2, #445
	ldr	r0, .L366+16
	bl	printk
	.loc 3 445 0 discriminator 3
	bl	dump_ftl_info
.L347:
	b	.L347
.L346:
	.loc 3 446 0 is_stmt 1
	ldr	r6, .L366
	.loc 3 447 0
	add	r3, r3, #1728
	.loc 3 446 0
	ldr	r2, .L366+20
	.loc 3 453 0
	ldr	sl, .L366+24
.LVL289:
	.loc 3 454 0
	ldr	lr, [r6, #-3920]
	.loc 3 446 0
	ldrh	r2, [r6, r2]
	.loc 3 448 0
	sub	fp, sl, #356
.LVL290:
	.loc 3 456 0
	ldr	r8, .L366+28
	.loc 3 455 0
	ldr	r9, [r6, #-3912]
	.loc 3 450 0
	sub	r6, r6, #3984
.LVL291:
	.loc 3 446 0
	str	r2, [sp, #20]
	.loc 3 452 0
	sub	r2, r8, #23
	.loc 3 454 0
	str	lr, [sp, #8]
	.loc 3 451 0
	sub	lr, r8, #25
	.loc 3 452 0
	str	r2, [sp, #12]
	.loc 3 449 0
	sub	r2, r8, #31
	.loc 3 451 0
	str	lr, [sp, #4]
	.loc 3 449 0
	str	r2, [sp, #16]
.LVL292:
.L364:
	.loc 3 447 0
	str	r3, [sp, #24]
.LVL293:
.L361:
	.loc 3 463 0
	ldr	lr, [sp, #4]
	ldrh	r1, [r6, #0]
	ldrh	r3, [lr, #0]
	cmp	r1, r3
	bne	.L349
	.loc 3 464 0
	ldrb	r3, [r8, #0]	@ zero_extendqisi2
	cmp	r3, #0
	.loc 3 465 0
	movne	r3, #0
	strneb	r3, [r8, #0]
	.loc 3 464 0
	bne	.L351
	.loc 3 468 0
	ldrh	r0, [fp, #0]
	mov	r2, r5
	bl	sftl_write_trace_page
.L351:
	.loc 3 470 0
	mov	r0, r5
	bl	sftl_open_sblk
	.loc 3 471 0
	ldr	r2, [sp, #24]
	ldr	lr, [sp, #20]
	ldrh	r3, [r2, #0]
	cmp	r3, lr
	.loc 3 472 0
	ldreq	r3, [sp, #12]
	moveq	r2, #1
	streqb	r2, [r3, #0]
	.loc 3 471 0
	beq	.L361
.L349:
	.loc 3 479 0
	ldr	lr, .L366+4
	mov	ip, r8
	ldrh	r2, [lr, #14]
	mov	r8, r2
.L354:
	ldrh	r7, [r6, #0]
	mov	r1, r8
	str	ip, [sp, #0]
	mov	r0, r7
	bl	__aeabi_uidivmod
	.loc 3 480 0
	ldr	r3, [sp, #16]
	mov	lr, #1
	ldr	ip, [sp, #0]
	ldr	r2, [r3, #0]
	uxth	r1, r1
	ands	r2, r2, lr, asl r1
	str	r2, [sp, #28]
	beq	.L353
	.loc 3 483 0
	ldr	r3, [sp, #4]
	.loc 3 482 0
	add	r7, r7, lr
	uxth	r7, r7
	strh	r7, [r6, #0]	@ movhi
	.loc 3 483 0
	ldrh	r2, [r3, #0]
	cmp	r2, r7
	bne	.L354
	mov	r8, ip
	b	.L361
.L353:
	mov	r3, r7
.LBB233:
.LBB234:
.LBB235:
.LBB236:
	.loc 3 10 0
	ldr	r7, .L366
	ldr	lr, .L366+4
	mov	r2, r8
	add	r1, r7, r1, asl #1
	ldrh	r0, [fp, #0]
	sub	r1, r1, #3968
	mov	r8, ip
	sub	r1, r1, #8
	ldrh	ip, [lr, #16]
	ldrh	r1, [r1, #0]
	mla	ip, ip, r0, r1
.LBE236:
.LBE235:
	.loc 3 39 0
	mov	r1, r2
	mov	r0, r3
	.loc 3 42 0
	mov	ip, ip, asl #11
	.loc 3 39 0
	str	ip, [sp, #0]
	bl	__aeabi_uidiv
.LBE234:
.LBE233:
	.loc 3 490 0
	ldr	r2, [sp, #8]
.LBB239:
.LBB237:
	.loc 3 42 0
	ldr	ip, [sp, #0]
.LBE237:
.LBE239:
	.loc 3 494 0
	mov	r1, #1
	.loc 3 491 0
	ldr	r3, [sl, #0]
	.loc 3 490 0
	str	r2, [sp, #52]
	.loc 3 491 0
	add	r3, r3, #1
	.loc 3 494 0
	ldr	r2, [sp, #28]
	.loc 3 491 0
	str	r3, [sl, #0]
.LBB240:
.LBB238:
	.loc 3 42 0
	uxth	r0, r0
	orr	ip, ip, r0
.LVL294:
.LBE238:
.LBE240:
	.loc 3 494 0
	add	r0, sp, #44
	.loc 3 487 0
	str	ip, [sp, #40]
	.loc 3 488 0
	str	ip, [sp, #48]
	.loc 3 492 0
	stmia	r9, {r3, r4}
.LVL295:
	.loc 3 494 0
	mov	r3, r1
	.loc 3 493 0
	str	r9, [sp, #56]
	.loc 3 494 0
	bl	ftl_flash_prog_pages
.LVL296:
	.loc 3 495 0
	ldrh	r3, [r6, #0]
	.loc 3 501 0
	ldrb	r2, [r8, #0]	@ zero_extendqisi2
	.loc 3 495 0
	add	r3, r3, #1
	.loc 3 501 0
	cmp	r2, #0
	.loc 3 495 0
	uxth	r3, r3
	strh	r3, [r6, #0]	@ movhi
	.loc 3 502 0
	ldrne	r3, [sp, #28]
	strneb	r3, [r8, #0]
	.loc 3 501 0
	bne	.L361
	.loc 3 505 0
	ldr	r2, [sp, #44]
	cmp	r2, #0
	bne	.L361
	.loc 3 510 0
	cmp	r5, #2
	bne	.L356
	.loc 3 511 0
	ldr	r1, [r7, #-3868]
	sub	r3, r3, #1
	str	r4, [r1, r3, asl #2]
.LVL297:
	.loc 3 512 0
	ldr	r3, .L366+32
	ldr	r1, .L366+4
	ldrh	r0, [r7, r3]
	movw	r3, #1734
	strh	r0, [r1, r3]	@ movhi
	.loc 3 513 0
	add	r1, sp, #36
	mov	r0, r4
	bl	sftl_log_to_phy
	.loc 3 514 0
	mov	r0, r4
	add	r1, sp, #40
	mov	r2, #1
	bl	sftl_log_to_phy
	b	.L357
.LVL298:
.L356:
	.loc 3 515 0
	cmp	r5, #1
	bne	.L357
	.loc 3 516 0
	ldr	r2, [r7, #-3876]
	add	r3, r2, r3, asl #1
	strh	r4, [r3, #-2]	@ movhi
	.loc 3 517 0
	ldr	r3, [r7, #-3900]
	ldr	r2, [r3, r4, asl #2]
.LVL299:
	str	r2, [sp, #36]
	.loc 3 518 0
	ldr	r2, [sp, #40]
.LVL300:
	str	r2, [r3, r4, asl #2]
.L357:
	.loc 3 521 0
	ldrh	r0, [fp, #0]
	mov	r2, r5
	ldr	r1, [sp, #36]
	bl	sftl_update_vpn
	.loc 3 523 0
	ldr	r0, [sp, #44]
.L342:
	.loc 3 524 0
	add	sp, sp, #68
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L367:
	.align	2
.L366:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LANCHOR1-3958
	.word	.LC7
	.word	.LC5
	.word	-3952
	.word	.LANCHOR0+2088
	.word	.LANCHOR1-3957
	.word	-3984
.LFE161:
	.fnend
	.size	sftl_write_page, .-sftl_write_page
	.align	2
	.global	sftl_l2p_gc
	.type	sftl_l2p_gc, %function
sftl_l2p_gc:
	.fnstart
.LFB163:
	.loc 3 554 0
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	.loc 3 569 0
	ldr	r3, .L387
	.loc 3 566 0
	mvn	r2, #0
.LVL301:
	.loc 3 554 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
.LCFI41:
	.loc 3 570 0
	mov	r7, #0
	.loc 3 569 0
	ldrh	ip, [r3, #38]
.LVL302:
	.loc 3 554 0
	.pad #24
	sub	sp, sp, #24
.LCFI42:
	.loc 3 572 0
	ldr	r3, .L387+4
	.loc 3 567 0
	mov	r6, r7
.LVL303:
	.loc 3 572 0
	ldr	r1, .L387+8
	ldr	r0, [r3, #-3908]
	.loc 3 570 0
	b	.L369
.LVL304:
.L375:
	.loc 3 571 0
	ldrh	r4, [r1, #2]!
	.loc 3 572 0
	ldr	r3, [r0, r4, asl #2]
.LVL305:
	ubfx	r3, r3, #16, #9
.LVL306:
	.loc 3 573 0
	cmp	r3, #0
	bne	.L370
	.loc 3 575 0
	mov	r0, r4
	bl	sftl_l2p_check_real_vpn
.LVL307:
	.loc 3 576 0
	subs	r2, r0, #0
	beq	.L371
	.loc 3 577 0 discriminator 1
	mov	r1, r4
	ldr	r0, .L387+12
	bl	printk
	.loc 3 578 0 discriminator 2
	ldr	r1, .L387+16
	movw	r2, #578
	ldr	r0, .L387+20
	bl	printk
	.loc 3 578 0 is_stmt 0 discriminator 3
	bl	dump_ftl_info
.L372:
	b	.L372
.L371:
	.loc 3 580 0 is_stmt 1
	mov	r0, r4
	.loc 3 584 0
	ldr	r4, .L387
	.loc 3 580 0
	bl	sftl_free_sblk
	.loc 3 582 0
	add	r0, r8, #776
	add	r8, r8, #1
	add	r3, r4, #48
	.loc 3 584 0
	ldrh	r5, [r4, #36]
	.loc 3 582 0
	add	r1, r8, #776
	add	r0, r3, r0, asl #1
	.loc 3 584 0
	sub	r5, r5, #1
	.loc 3 582 0
	add	r1, r3, r1, asl #1
	.loc 3 584 0
	rsb	r2, r7, r5
	.loc 3 585 0
	add	r5, r5, #776
	add	r4, r4, r5, asl #1
	.loc 3 582 0
	mov	r2, r2, asl #1
	bl	memcpy
	.loc 3 585 0
	mvn	r3, #0
	strh	r3, [r4, #48]	@ movhi
	b	.L373
.LVL308:
.L370:
	.loc 3 588 0
	cmp	r3, r2
	add	r7, r7, #1
	movcc	r2, r3
	movcc	r6, r8
.L369:
	uxth	r8, r7
	.loc 3 570 0 discriminator 1
	cmp	r8, ip
	bcc	.L375
	b	.L386
.LVL309:
.L381:
	.loc 3 602 0
	ldr	r3, [r7, #-3900]
	ldr	r8, [r3, r5, asl #2]
.LVL310:
	.loc 3 603 0
	cmn	r8, #1
	beq	.L377
	.loc 3 603 0 is_stmt 0 discriminator 1
	mov	r0, r8
	bl	sftl_ppn_to_sblk
.LVL311:
	cmp	r0, r4
	bne	.L377
	.loc 3 607 0 is_stmt 1
	ldr	r3, [r7, #-3888]
	.loc 3 609 0
	mov	r1, #1
	add	r0, sp, #4
	mov	r2, #0
	.loc 3 606 0
	str	r8, [sp, #8]
	.loc 3 607 0
	str	r3, [sp, #12]
	.loc 3 608 0
	ldr	r3, [r7, #-3884]
	str	r3, [sp, #16]
	.loc 3 609 0
	bl	ftl_flash_read_pages
	.loc 3 614 0
	ldr	r1, [sp, #4]
	cmn	r1, #1
	beq	.L378
	.loc 3 614 0 is_stmt 0 discriminator 1
	ldr	r3, [r7, #-3884]
	ldr	r3, [r3, #4]
	cmp	r3, r5
	beq	.L379
.L378:
	.loc 3 615 0 is_stmt 1 discriminator 1
	ldr	r4, .L387+4
	mov	r2, r5
	ldr	r0, .L387+24
	ldr	r3, [r4, #-3884]
	ldr	r3, [r3, #4]
	bl	printk
	.loc 3 620 0
	ldr	r3, .L387
	ldr	r0, .L387+28
	mov	r2, #4
	ldr	r1, [r4, #-3888]
	ldr	r3, [r3, #8]
	mov	r3, r3, lsr #2
	bl	rknand_print_hex
	.loc 3 621 0
	ldr	r0, .L387+32
	ldr	r1, [r4, #-3884]
	mov	r2, #4
	mov	r3, #2
	bl	rknand_print_hex
	.loc 3 622 0 discriminator 2
	ldr	r1, .L387+16
	movw	r2, #622
	ldr	r0, .L387+20
	bl	printk
	.loc 3 622 0 is_stmt 0 discriminator 3
	bl	dump_ftl_info
.L380:
	b	.L380
.L379:
	.loc 3 625 0 is_stmt 1
	mov	r0, r5
	mov	r1, #1
	bl	sftl_write_page
.LVL312:
.L377:
	.loc 3 600 0
	add	r5, r5, #1
	uxth	r5, r5
.L383:
	.loc 3 600 0 is_stmt 0 discriminator 1
	ldrh	r3, [sl, #32]
	ldr	r8, .L387
.LVL313:
	cmp	r3, r5
	ldr	r9, .L387+4
	bhi	.L381
	.loc 3 632 0 is_stmt 1
	ldr	r3, [r9, #-3908]
	.loc 3 633 0
	mov	r1, #384
	mov	r0, r4
	.loc 3 632 0
	ldr	r2, [r3, r4, asl #2]
.LVL314:
	bfc	r2, #16, #9
	str	r2, [r3, r4, asl #2]
	.loc 3 633 0
	bl	__aeabi_uidiv
.LVL315:
	ldr	r3, [r9, #-3932]
	mov	r2, #1
	uxth	r0, r0
	str	r0, [r9, #-3928]
	orr	r3, r3, r2, asl r0
	.loc 3 635 0
	mov	r0, r4
	.loc 3 633 0
	str	r3, [r9, #-3932]
	.loc 3 635 0
	bl	sftl_free_sblk
	.loc 3 639 0
	ldrh	r4, [r8, #36]
	.loc 3 637 0
	add	r0, r6, #776
	add	r3, r8, #48
	.loc 3 639 0
	sub	r4, r4, #1
	.loc 3 637 0
	add	r0, r3, r0, asl #1
	.loc 3 639 0
	rsb	r2, r6, r4
	.loc 3 637 0
	add	r6, r6, #1
.LVL316:
	.loc 3 640 0
	add	r4, r4, #776
	.loc 3 637 0
	add	r1, r6, #776
	mov	r2, r2, asl #1
	.loc 3 640 0
	add	r8, r8, r4, asl #1
	.loc 3 637 0
	add	r1, r3, r1, asl #1
	bl	memcpy
	.loc 3 640 0
	mvn	r3, #0
	strh	r3, [r8, #48]	@ movhi
.LVL317:
.L373:
	.loc 3 643 0
	ldr	r4, .L387+4
	ldr	r5, .L387+36
	ldrh	r3, [r4, r5]
	sub	r3, r3, #1
	strh	r3, [r4, r5]	@ movhi
	.loc 3 645 0
	bl	sftl_sys_info_flush
	.loc 3 650 0
	ldr	r3, .L387
	ldrh	r2, [r4, r5]
	ldrh	r3, [r3, #38]
	cmp	r2, r3
	bcc	.L382
	.loc 3 651 0
	bl	sftl_l2p_gc
.L382:
	.loc 3 652 0
	mov	r0, #0
	strb	r0, [r4, #-3996]
	.loc 3 655 0
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.LVL318:
.L386:
	.loc 3 599 0
	ldr	sl, .L387
	add	r3, r6, #776
.LVL319:
	.loc 3 600 0
	mov	r5, #0
	ldr	r7, .L387+4
	.loc 3 599 0
	add	r3, sl, r3, asl #1
	ldrh	r4, [r3, #48]
	.loc 3 600 0
	b	.L383
.L388:
	.align	2
.L387:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LANCHOR0+1598
	.word	.LC8
	.word	.LC7
	.word	.LC5
	.word	.LC9
	.word	.LC10
	.word	.LC11
	.word	-4008
.LFE163:
	.fnend
	.size	sftl_l2p_gc, .-sftl_l2p_gc
	.align	2
	.global	sftl_l2p_flush_cache
	.type	sftl_l2p_flush_cache, %function
sftl_l2p_flush_cache:
	.fnstart
.LFB164:
	.loc 3 665 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL320:
	stmfd	sp!, {r4, r5, r6, lr}
	.save {r4, r5, r6, lr}
.LCFI43:
	.loc 3 666 0
	mov	r4, #12
	mul	r4, r4, r0
	ldr	r5, .L392
	ldr	r3, [r5, #-3892]
	add	r2, r3, r4
	ldrh	r6, [r3, r4]
.LVL321:
	.loc 3 669 0
	ldr	r3, .L392+4
	ldrh	r1, [r3, #32]
	cmp	r1, r6
	bls	.L391
	.loc 3 671 0
	ldr	r1, [r2, #8]
	ldr	r0, [r5, #-3888]
.LVL322:
	ldr	r2, [r3, #8]
	bl	memcpy
	.loc 3 672 0
	mov	r0, r6
	mov	r1, #1
	bl	sftl_write_page
	.loc 3 673 0
	ldr	r3, [r5, #-3892]
	mov	r0, #0
	add	r3, r3, r4
	strb	r0, [r3, #2]
	.loc 3 674 0
	ldr	r3, [r5, #-3892]
	add	r4, r3, r4
	str	r0, [r4, #4]
	.loc 3 676 0
	ldmfd	sp!, {r4, r5, r6, pc}
.LVL323:
.L391:
	.loc 3 670 0
	mvn	r0, #0
.LVL324:
	.loc 3 677 0
	ldmfd	sp!, {r4, r5, r6, pc}
.L393:
	.align	2
.L392:
	.word	.LANCHOR1
	.word	.LANCHOR0
.LFE164:
	.fnend
	.size	sftl_l2p_flush_cache, .-sftl_l2p_flush_cache
	.align	2
	.global	sftl_l2p_flush
	.type	sftl_l2p_flush, %function
sftl_l2p_flush:
	.fnstart
.LFB165:
	.loc 3 683 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
.LCFI44:
	.loc 3 687 0
	mov	r4, #0
	mov	r5, r4
.LVL325:
	.loc 3 687 0 is_stmt 0 discriminator 1
	ldr	r8, .L399
	ldr	r7, .L399+4
	.loc 3 688 0 is_stmt 1
	movw	r6, #65535
	.loc 3 687 0
	b	.L395
.L397:
	.loc 3 688 0
	ldr	r3, [r7, #-3892]
	add	r2, r3, r4
	ldrh	r3, [r3, r4]
	cmp	r3, r6
	beq	.L396
	.loc 3 688 0 is_stmt 0 discriminator 1
	ldrb	r3, [r2, #2]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L396
	.loc 3 691 0 is_stmt 1
	mov	r0, r5
	bl	sftl_l2p_flush_cache
.L396:
	.loc 3 687 0
	add	r5, r5, #1
	add	r4, r4, #12
.L395:
	.loc 3 687 0 is_stmt 0 discriminator 1
	ldrh	r3, [r8, #34]
	cmp	r5, r3
	bcc	.L397
	.loc 3 693 0 is_stmt 1
	ldr	r3, .L399+4
	ldrb	r3, [r3, #-3996]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L398
	.loc 3 694 0
	bl	sftl_l2p_gc
.L398:
	.loc 3 696 0
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L400:
	.align	2
.L399:
	.word	.LANCHOR0
	.word	.LANCHOR1
.LFE165:
	.fnend
	.size	sftl_l2p_flush, .-sftl_l2p_flush
	.align	2
	.global	sftl_log_to_phy
	.type	sftl_log_to_phy, %function
sftl_log_to_phy:
	.fnstart
.LFB166:
	.loc 3 704 0
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL326:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, lr}
.LCFI45:
	.loc 3 710 0
	subs	r7, r2, #0
	.loc 3 704 0
	.pad #24
	sub	sp, sp, #24
.LCFI46:
	.loc 3 704 0
	mov	sl, r0
	mov	r5, r1
	ldr	r4, .L431
	.loc 3 710 0
	beq	.L402
	.loc 3 714 0
	ldr	r0, [r1, #0]
.LVL327:
	bl	sftl_ppn_to_sblk
.LVL328:
	ldrh	r3, [r4, #18]
	cmp	r3, r0
	bhi	.L402
	.loc 3 714 0 is_stmt 0 discriminator 2
	ldr	r1, .L431+4
	movw	r2, #714
	ldr	r0, .L431+8
	bl	printk
	.loc 3 714 0 discriminator 3
	bl	dump_ftl_info
.L403:
	b	.L403
.L402:
	.loc 3 716 0 is_stmt 1
	ldr	r3, [r4, #44]
	cmp	sl, r3
	bls	.L404
	.loc 3 716 0 is_stmt 0 discriminator 2
	ldr	r1, .L431+4
	mov	r2, #716
	ldr	r0, .L431+8
	bl	printk
	.loc 3 716 0 discriminator 3
	bl	dump_ftl_info
.L405:
	b	.L405
.L404:
	.loc 3 720 0 is_stmt 1 discriminator 1
	ldr	r3, .L431
	.loc 3 717 0
	ubfx	r8, sl, #9, #16
.LVL329:
	.loc 3 718 0
	mov	sl, sl, asl #23
.LVL330:
	.loc 3 720 0 discriminator 1
	ldrh	ip, [r3, #34]
	.loc 3 718 0
	mov	sl, sl, lsr #23
.LVL331:
	.loc 3 721 0
	ldr	r3, .L431+12
	ldr	r0, [r3, #-3892]
.LVL332:
	.loc 3 720 0
	mov	r3, #0
	mov	r2, r3
	b	.L406
.L411:
	.loc 3 721 0
	ldrh	r6, [r0, r3]
	.loc 3 703 0
	add	r1, r0, r3
	add	r4, r3, #12
	.loc 3 721 0
	cmp	r6, r8
	bne	.L407
	.loc 3 723 0
	cmp	r7, #0
	ldr	r2, [r1, #8]
	.loc 3 726 0
	ldrne	r1, [r5, #0]
	.loc 3 724 0
	ldreq	r2, [r2, sl, asl #2]
	.loc 3 726 0
	strne	r1, [r2, sl, asl #2]
	.loc 3 727 0
	movne	r1, #1
	ldrne	r2, .L431+12
	.loc 3 724 0
	streq	r2, [r5, #0]
	.loc 3 727 0
	ldrne	r2, [r2, #-3892]
	addne	r2, r2, r3
	strneb	r1, [r2, #2]
	.loc 3 729 0
	ldr	r2, .L431+12
	ldr	r2, [r2, #-3892]
	add	r3, r2, r3
	ldr	r2, [r3, #4]
	add	r2, r2, #1
	str	r2, [r3, #4]
	.loc 3 731 0
	b	.L410
.L407:
	.loc 3 720 0
	add	r2, r2, #1
	mov	r3, r4
	uxth	r2, r2
.L406:
	.loc 3 720 0 is_stmt 0 discriminator 1
	cmp	r2, ip
	bne	.L411
	mov	r1, #0
	mvn	r2, #0
	mov	r6, r1
	mov	r3, r1
	b	.L412
.L415:
	.loc 3 703 0 is_stmt 1
	add	r4, r0, r1
	.loc 3 738 0
	ldrb	r9, [r4, #2]	@ zero_extendqisi2
	cmp	r9, #0
	bne	.L413
	.loc 3 739 0
	ldr	r4, [r4, #4]
	.loc 3 738 0 discriminator 1
	cmp	r4, r2
	bcs	.L413
	.loc 3 742 0
	cmp	r4, #0
	mov	r6, r3
	beq	.L414
	mov	r2, r4
.L413:
	.loc 3 737 0
	add	r3, r3, #1
	add	r1, r1, #12
	uxth	r3, r3
.L412:
	.loc 3 737 0 is_stmt 0 discriminator 1
	cmp	r3, ip
	bne	.L415
	.loc 3 746 0 is_stmt 1
	cmn	r2, #1
	bne	.L414
	b	.L430
.L418:
	.loc 3 703 0
	add	lr, r0, r1
	.loc 3 748 0
	add	r1, r1, #12
	.loc 3 749 0
	ldr	lr, [lr, #4]
	cmp	lr, r2
	movcc	r6, r3
	.loc 3 748 0
	add	r3, r3, #1
	.loc 3 749 0
	movcc	r2, lr
	.loc 3 748 0
	uxth	r3, r3
	b	.L416
.L430:
	.loc 3 746 0
	mov	r1, #0
	mov	r3, r1
.L416:
	.loc 3 748 0 discriminator 1
	cmp	r3, ip
	bne	.L418
	.loc 3 754 0
	mov	r0, r6
	bl	sftl_l2p_flush_cache
	.loc 3 755 0
	ldr	r3, .L431+12
	ldrb	r3, [r3, #-3996]	@ zero_extendqisi2
	cmp	r3, #0
	beq	.L414
	.loc 3 756 0
	bl	sftl_l2p_flush
.L414:
	.loc 3 759 0
	ldr	r4, .L431+12
	ldr	r3, [r4, #-3900]
	ldr	r3, [r3, r8, asl #2]
	cmn	r3, #1
	bne	.L419
	.loc 3 761 0
	cmp	r7, #0
	.loc 3 762 0
	streq	r7, [r5, #0]
	.loc 3 761 0
	beq	.L410
.LBB241:
	.loc 3 766 0
	mov	r3, #12
	mul	r6, r3, r6
	ldr	r3, [r4, #-3892]
	add	r3, r3, r6
	ldr	r0, [r3, #8]
.LVL333:
	ldr	r3, .L431
	ldr	r1, [r3, #8]
.LVL334:
	cmp	r1, #0
	beq	.L421
	.loc 3 766 0 is_stmt 0 discriminator 3
	bl	__memzero
.LVL335:
.L421:
.LBE241:
	.loc 3 767 0 is_stmt 1
	ldr	r3, .L431+12
	ldr	r1, [r5, #0]
.LVL336:
	ldr	r2, [r3, #-3892]
	add	r2, r2, r6
	ldr	r2, [r2, #8]
	str	r1, [r2, sl, asl #2]
	.loc 3 768 0
	ldr	r2, [r3, #-3892]
	strh	r8, [r2, r6]	@ movhi
	.loc 3 769 0
	mov	r2, #1
	ldr	r1, [r3, #-3892]
	add	r1, r1, r6
	strb	r2, [r1, #2]
	.loc 3 770 0
	ldr	r3, [r3, #-3892]
	add	r6, r3, r6
	str	r2, [r6, #4]
	.loc 3 772 0
	b	.L410
.LVL337:
.L419:
	.loc 3 775 0
	str	r3, [sp, #8]
	.loc 3 778 0
	mov	r1, #1
	.loc 3 776 0
	ldr	r3, [r4, #-3924]
	.loc 3 778 0
	add	r0, sp, #4
	mov	r2, #0
	.loc 3 776 0
	str	r3, [sp, #12]
	.loc 3 777 0
	ldr	r3, [r4, #-3916]
	str	r3, [sp, #16]
	.loc 3 778 0
	bl	ftl_flash_read_pages
	.loc 3 779 0
	ldr	r1, [sp, #4]
	cmn	r1, #1
	beq	.L422
	.loc 3 779 0 is_stmt 0 discriminator 1
	ldr	r3, [r4, #-3916]
	ldr	r3, [r3, #4]
	cmp	r3, r8
	beq	.L423
.L422:
	.loc 3 783 0 is_stmt 1 discriminator 1
	ldr	r4, .L431+12
	.loc 3 780 0
	cmp	r7, #0
	.loc 3 781 0
	streq	r7, [r5, #0]
	.loc 3 783 0 discriminator 1
	mov	r2, r8
	ldr	r0, .L431+16
	ldr	r3, [r4, #-3916]
	ldr	r3, [r3, #4]
	bl	printk
	.loc 3 788 0
	ldr	r3, .L431
	ldr	r0, .L431+20
	mov	r2, #4
	ldr	r1, [r4, #-3924]
	ldr	r3, [r3, #8]
	mov	r3, r3, lsr #2
	bl	rknand_print_hex
	.loc 3 789 0
	ldr	r0, .L431+24
	ldr	r1, [r4, #-3916]
	mov	r2, #4
	mov	r3, #2
	bl	rknand_print_hex
	.loc 3 790 0 discriminator 2
	ldr	r1, .L431+4
	movw	r2, #790
	ldr	r0, .L431+8
	bl	printk
	.loc 3 790 0 is_stmt 0 discriminator 3
	bl	dump_ftl_info
.L425:
	b	.L425
.L423:
	.loc 3 796 0 is_stmt 1
	mov	r3, #12
	ldr	r1, [r4, #-3924]
	mul	r6, r3, r6
	ldr	r3, [r4, #-3892]
	add	r3, r3, r6
	ldr	r0, [r3, #8]
	ldr	r3, .L431
	ldr	r2, [r3, #8]
	bl	memcpy
	.loc 3 797 0
	cmp	r7, #0
	bne	.L426
	.loc 3 798 0
	ldr	r3, [r4, #-3892]
	add	r3, r3, r6
	ldr	r3, [r3, #8]
	ldr	r3, [r3, sl, asl #2]
	str	r3, [r5, #0]
	.loc 3 799 0
	ldr	r3, [r4, #-3892]
	add	r3, r3, r6
	strb	r7, [r3, #2]
	b	.L427
.L426:
	.loc 3 801 0
	ldr	r3, [r4, #-3892]
	ldr	r2, [r5, #0]
	add	r3, r3, r6
	ldr	r3, [r3, #8]
	str	r2, [r3, sl, asl #2]
	.loc 3 802 0
	mov	r2, #1
	ldr	r3, [r4, #-3892]
	add	r3, r3, r6
	strb	r2, [r3, #2]
.L427:
	.loc 3 804 0
	ldr	r3, .L431+12
	ldr	r2, [r3, #-3892]
	strh	r8, [r2, r6]	@ movhi
	.loc 3 805 0
	ldr	r3, [r3, #-3892]
	add	r6, r3, r6
	mov	r3, #1
	str	r3, [r6, #4]
.LVL338:
.L410:
	.loc 3 812 0
	mov	r0, #0
.LVL339:
	add	sp, sp, #24
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, pc}
.L432:
	.align	2
.L431:
	.word	.LANCHOR0
	.word	.LC7
	.word	.LC5
	.word	.LANCHOR1
	.word	.LC12
	.word	.LC10
	.word	.LC11
.LFE166:
	.fnend
	.size	sftl_log_to_phy, .-sftl_log_to_phy
	.align	2
	.global	sftl_user_check_real_vpn
	.type	sftl_user_check_real_vpn, %function
sftl_user_check_real_vpn:
	.fnstart
.LFB168:
	.loc 3 933 0
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL340:
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI47:
.LVL341:
	.loc 3 938 0
	mov	r1, #2
	.loc 3 937 0
	mov	r5, #0
.LVL342:
	.loc 3 933 0
	mov	r8, r0
.LVL343:
	.loc 3 939 0
	mov	r4, r5
.LVL344:
	.loc 3 938 0
	bl	sftl_get_trace_list
.LVL345:
	.loc 3 939 0 discriminator 1
	ldr	r6, .L439
	.loc 3 940 0
	ldr	r7, .L439+4
	.loc 3 939 0
	b	.L434
.LVL346:
.L438:
	.loc 3 940 0
	ldr	r3, [r7, #-3872]
	ldr	r0, [r3, r4, asl #2]
	.loc 3 941 0
	cmn	r0, #1
	beq	.L435
	.loc 3 943 0
	add	r1, sp, #4
	mov	r2, #0
	bl	sftl_log_to_phy
.LVL347:
	.loc 3 944 0
	ldr	sl, [sp, #4]
	cmp	sl, #0
	bne	.L436
	.loc 3 944 0 is_stmt 0 discriminator 2
	ldr	r1, .L439+8
	mov	r2, #944
	ldr	r0, .L439+12
	bl	printk
	.loc 3 944 0 discriminator 3
	bl	dump_ftl_info
.L437:
	b	.L437
.L436:
.LBB242:
.LBB243:
	.loc 3 39 0 is_stmt 1
	ldrh	r9, [r6, #14]
	.loc 3 40 0
	mov	r0, r4
	mov	r1, r9
	bl	__aeabi_uidivmod
.LBB244:
.LBB245:
	.loc 3 10 0
	ldrh	fp, [r6, #16]
.LBE245:
.LBE244:
	.loc 3 39 0
	mov	r0, r4
.LBB248:
.LBB246:
	.loc 3 10 0
	uxth	r3, r1
.LBE246:
.LBE248:
	.loc 3 39 0
	mov	r1, r9
.LBB249:
.LBB247:
	.loc 3 10 0
	add	r3, r7, r3, asl #1
	sub	r3, r3, #3968
	sub	r3, r3, #8
	ldrh	r3, [r3, #0]
	mla	fp, fp, r8, r3
.LBE247:
.LBE249:
	.loc 3 39 0
	bl	__aeabi_uidiv
	.loc 3 42 0
	mov	fp, fp, asl #11
	uxth	r0, r0
	orr	fp, fp, r0
.LBE243:
.LBE242:
	.loc 3 945 0
	cmp	sl, fp
	.loc 3 946 0
	addeq	r5, r5, #1
.LVL348:
	uxtheq	r5, r5
.LVL349:
.L435:
	.loc 3 939 0
	add	r4, r4, #1
.LVL350:
	uxth	r4, r4
.LVL351:
.L434:
	.loc 3 939 0 is_stmt 0 discriminator 1
	ldrh	r3, [r6, #22]
	sub	r3, r3, #1
	cmp	r4, r3
	blt	.L438
	.loc 3 950 0 is_stmt 1
	mov	r0, r5
.LVL352:
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L440:
	.align	2
.L439:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC7
	.word	.LC5
.LFE168:
	.fnend
	.size	sftl_user_check_real_vpn, .-sftl_user_check_real_vpn
	.align	2
	.global	sftl_update_vpn
	.type	sftl_update_vpn, %function
sftl_update_vpn:
	.fnstart
.LFB160:
	.loc 3 373 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL353:
	stmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, lr}
	.save {r3, r4, r5, r6, r7, r8, sl, lr}
.LCFI48:
	.loc 3 373 0
	mov	r7, r2
	.loc 3 376 0
	ldr	sl, .L448
	.loc 3 373 0
	mov	r4, r0
	.loc 3 376 0
	ldr	r3, .L448+4
	.loc 3 373 0
	mov	r5, r1
	.loc 3 376 0
	ldr	r6, [sl, #-3908]
	ldrh	r3, [r3, #22]
	ldr	r8, [r6, r0, asl #2]
	sub	r3, r3, #1
	ubfx	r2, r8, #16, #9
.LVL354:
	cmp	r2, r3
	blt	.L442
	.loc 3 377 0
	cmp	r7, #1
	bne	.L443
	.loc 3 378 0
	bl	sftl_l2p_check_real_vpn
.LVL355:
	bfi	r8, r0, #16, #9
	str	r8, [r6, r4, asl #2]
	b	.L442
.L443:
	.loc 3 380 0
	bl	sftl_user_check_real_vpn
	ldr	r3, [r6, r4, asl #2]
	bfi	r3, r0, #16, #9
	str	r3, [r6, r4, asl #2]
.L442:
	.loc 3 382 0
	ldr	r3, [sl, #-3908]
	ldr	r8, .L448
	ldr	r2, [r3, r4, asl #2]
	ubfx	r1, r2, #16, #9
	add	r1, r1, #1
	bfi	r2, r1, #16, #9
	str	r2, [r3, r4, asl #2]
	.loc 3 383 0
	ldr	r3, .L448+4
	ldr	r6, [sl, #-3908]
	ldrh	r3, [r3, #22]
	ldr	r2, [r6, r4, asl #2]
	sub	r3, r3, #1
	ubfx	r2, r2, #16, #9
	cmp	r2, r3
	ble	.L444
	.loc 3 383 0 is_stmt 0 discriminator 2
	ldr	r1, .L448+8
	movw	r2, #383
	ldr	r0, .L448+12
	bl	printk
	.loc 3 383 0 discriminator 3
	bl	dump_ftl_info
.L445:
	b	.L445
.L444:
	.loc 3 384 0 is_stmt 1
	mov	r1, #384
	mov	r0, r4
	bl	__aeabi_uidiv
	ldr	r3, [r8, #-3932]
	.loc 3 385 0
	sub	r1, r7, #2
	.loc 3 384 0
	mov	r2, #1
	uxth	r0, r0
	str	r0, [r8, #-3928]
	orr	r3, r3, r2, asl r0
	str	r3, [r8, #-3932]
	.loc 3 385 0
	rsbs	r3, r1, #0
	adc	r3, r3, r1
	cmp	r5, #0
	moveq	r3, #0
	cmp	r3, #0
	bne	.L446
	.loc 3 385 0 is_stmt 0 discriminator 1
	rsb	r3, r2, r7
	rsbs	r7, r3, #0
.LVL356:
	adc	r7, r7, r3
	cmn	r5, #1
	moveq	r7, #0
	cmp	r7, #0
	ldmeqfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.LVL357:
.L446:
	.loc 3 387 0 is_stmt 1
	mov	r0, r5
	bl	sftl_ppn_to_sblk
	.loc 3 393 0
	ldr	r2, [r6, r0, asl #2]
	ubfx	r1, r2, #16, #9
	sub	r1, r1, #1
	bfi	r2, r1, #16, #9
	.loc 3 394 0
	mov	r1, #384
	.loc 3 393 0
	str	r2, [r6, r0, asl #2]
	.loc 3 394 0
	bl	__aeabi_uidiv
	ldr	r3, .L448
	mov	r1, #1
	ldr	r2, [r3, #-3932]
	uxth	r0, r0
	str	r0, [r3, #-3928]
	orr	r2, r2, r1, asl r0
	str	r2, [r3, #-3932]
	ldmfd	sp!, {r3, r4, r5, r6, r7, r8, sl, pc}
.L449:
	.align	2
.L448:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	.LC7
	.word	.LC5
.LFE160:
	.fnend
	.size	sftl_update_vpn, .-sftl_update_vpn
	.align	2
	.global	sftl_recover_prev_sblk
	.type	sftl_recover_prev_sblk, %function
sftl_recover_prev_sblk:
	.fnstart
.LFB174:
	.loc 3 1256 0
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r0, r1, r2, r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI49:
	.loc 3 1260 0
	movw	r6, #1730
	ldr	r4, .L455
	mov	r1, #2
	.loc 3 1261 0
	mov	r5, #0
.LVL358:
.LBB250:
.LBB251:
.LBB252:
.LBB253:
	.loc 3 10 0
	ldr	r7, .L455+4
.LBE253:
.LBE252:
.LBE251:
.LBE250:
	.loc 3 1260 0
	ldrh	r0, [r4, r6]
	bl	sftl_get_trace_list
	.loc 3 1261 0
	b	.L451
.LVL359:
.L454:
.LBB263:
.LBB260:
	.loc 3 39 0
	ldrh	r6, [r4, #14]
.LVL360:
.LBE260:
.LBE263:
	.loc 3 1262 0
	uxth	r8, r5
.LBB264:
.LBB261:
	.loc 3 40 0
	mov	r0, r8
	mov	r1, r6
	bl	__aeabi_uidivmod
.LBB257:
.LBB254:
	.loc 3 10 0
	ldrh	sl, [r4, #16]
.LBE254:
.LBE257:
	.loc 3 39 0
	mov	r0, r8
.LBB258:
.LBB255:
	.loc 3 10 0
	uxth	r3, r1
	movw	r1, #1730
	ldrh	r2, [r4, r1]
.LBE255:
.LBE258:
	.loc 3 39 0
	mov	r1, r6
.LBB259:
.LBB256:
	.loc 3 10 0
	add	r3, r7, r3, asl #1
	sub	r3, r3, #3968
	sub	r3, r3, #8
	ldrh	r3, [r3, #0]
	mla	sl, sl, r2, r3
.LBE256:
.LBE259:
	.loc 3 39 0
	bl	__aeabi_uidiv
.LBE261:
.LBE264:
	.loc 3 1263 0
	ldr	r3, [r7, #-3872]
.LBB265:
.LBB262:
	.loc 3 42 0
	mov	sl, sl, asl #11
	uxth	r0, r0
	orr	sl, sl, r0
.LBE262:
.LBE265:
	.loc 3 1262 0
	str	sl, [sp, #4]
	.loc 3 1263 0
	ldr	r6, [r3, r5, asl #2]
.LVL361:
	.loc 3 1264 0
	cmn	r6, #1
	beq	.L452
	.loc 3 1266 0
	mov	r1, sp
	mov	r2, #0
	mov	r0, r6
	bl	sftl_log_to_phy
	.loc 3 1271 0
	ldr	r8, [sp, #0]
	mov	r0, r8
	bl	sftl_ppn_to_sblk
	movw	r3, #1730
	ldrh	sl, [r4, r3]
	cmp	r0, sl
	bne	.L453
.LBB266:
.LBB267:
.LBB268:
.LBB269:
	.loc 3 19 0
	ldrh	r9, [r4, #16]
.LBE269:
.LBE268:
	.loc 3 54 0
	mov	fp, r8, asl #21
	.loc 3 57 0
	ldrh	r3, [r4, #14]
.LBB273:
.LBB270:
	.loc 3 20 0
	mov	r0, r8, lsr #11
.LBE270:
.LBE273:
	.loc 3 54 0
	mov	fp, fp, lsr #21
.LBB274:
.LBB271:
	.loc 3 20 0
	mov	r1, r9
.LBE271:
.LBE274:
	.loc 3 57 0
	mul	fp, r3, fp
.LBB275:
.LBB272:
	.loc 3 20 0
	bl	__aeabi_uidivmod
	.loc 3 19 0
	add	r1, fp, r1
	mov	fp, r8, lsr #25
.LBE272:
.LBE275:
	.loc 3 57 0
	mla	r9, fp, r9, r1
.LBE267:
.LBE266:
	.loc 3 1272 0
	uxth	r9, r9
	.loc 3 1271 0 discriminator 1
	cmp	r9, r5
	bcs	.L452
.L453:
	.loc 3 1274 0
	mov	r0, sl
	mov	r1, r8
	mov	r2, #2
	bl	sftl_update_vpn
	.loc 3 1275 0
	mov	r0, r6
	add	r1, sp, #4
	mov	r2, #1
	bl	sftl_log_to_phy
.L452:
	.loc 3 1261 0
	add	r5, r5, #1
.L451:
	.loc 3 1261 0 is_stmt 0 discriminator 1
	ldrh	r3, [r4, #22]
	sub	r3, r3, #1
	cmp	r5, r3
	bcc	.L454
	.loc 3 1277 0 is_stmt 1
	ldmfd	sp!, {r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L456:
	.align	2
.L455:
	.word	.LANCHOR0
	.word	.LANCHOR1
.LFE174:
	.fnend
	.size	sftl_recover_prev_sblk, .-sftl_recover_prev_sblk
	.align	2
	.global	sftl_recover_active_sblk
	.type	sftl_recover_active_sblk, %function
sftl_recover_active_sblk:
	.fnstart
.LFB173:
	.loc 3 1208 0
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI50:
	.loc 3 1219 0
	mov	r5, #0
.LVL362:
	.loc 3 1208 0
	.pad #36
	sub	sp, sp, #36
.LCFI51:
	.loc 3 1219 0 discriminator 1
	ldr	r6, .L462
	ldr	r4, .L462+4
	.loc 3 1219 0 is_stmt 0
	b	.L458
.LVL363:
.L461:
	.loc 3 1220 0 is_stmt 1
	ldr	r3, [r6, #-3924]
	.loc 3 1222 0
	uxth	r8, r5
.LBB276:
.LBB277:
	.loc 3 39 0
	ldrh	r7, [r4, #14]
	.loc 3 40 0
	mov	r0, r8
.LBE277:
.LBE276:
	.loc 3 1220 0
	str	r3, [sp, #20]
	.loc 3 1221 0
	ldr	r3, [r6, #-3916]
.LBB287:
.LBB284:
	.loc 3 40 0
	mov	r1, r7
.LBE284:
.LBE287:
	.loc 3 1221 0
	str	r3, [sp, #24]
.LBB288:
.LBB285:
	.loc 3 40 0
	bl	__aeabi_uidivmod
.LBB278:
.LBB279:
	.loc 3 10 0
	ldrh	sl, [r4, #16]
.LVL364:
.LBE279:
.LBE278:
	.loc 3 39 0
	mov	r0, r8
.LBB282:
.LBB280:
	.loc 3 10 0
	uxth	r3, r1
	movw	r1, #1732
	ldrh	r2, [r4, r1]
.LBE280:
.LBE282:
	.loc 3 39 0
	mov	r1, r7
.LBB283:
.LBB281:
	.loc 3 10 0
	add	r3, r6, r3, asl #1
	sub	r3, r3, #3968
	sub	r3, r3, #8
	ldrh	r3, [r3, #0]
	mla	sl, sl, r2, r3
.LBE281:
.LBE283:
	.loc 3 39 0
	bl	__aeabi_uidiv
.LBE285:
.LBE288:
	.loc 3 1224 0
	mov	r1, #1
	mov	r2, #0
.LBB289:
.LBB286:
	.loc 3 42 0
	mov	sl, sl, asl #11
	uxth	r0, r0
	orr	sl, sl, r0
.LVL365:
.LBE286:
.LBE289:
	.loc 3 1224 0
	add	r0, sp, #12
	.loc 3 1222 0
	str	sl, [sp, #8]
	.loc 3 1223 0
	str	sl, [sp, #16]
	.loc 3 1224 0
	bl	ftl_flash_read_pages
	.loc 3 1225 0
	ldr	r3, [sp, #12]
	cmn	r3, #1
	beq	.L459
	.loc 3 1226 0
	ldr	r3, [r6, #-3916]
	ldr	r7, [r3, #4]
	.loc 3 1225 0 discriminator 1
	cmn	r7, #1
	beq	.L459
	.loc 3 1229 0
	add	r1, sp, #4
	mov	r2, #0
	mov	r0, r7
	bl	sftl_log_to_phy
	.loc 3 1234 0
	ldr	r8, [sp, #4]
	mov	r0, r8
	bl	sftl_ppn_to_sblk
	movw	r3, #1732
	ldrh	sl, [r4, r3]
.LVL366:
	cmp	r0, sl
	bne	.L460
.LBB290:
.LBB291:
.LBB292:
.LBB293:
	.loc 3 19 0
	ldrh	r9, [r4, #16]
.LBE293:
.LBE292:
	.loc 3 54 0
	mov	fp, r8, asl #21
	.loc 3 57 0
	ldrh	r3, [r4, #14]
.LBB297:
.LBB294:
	.loc 3 20 0
	mov	r0, r8, lsr #11
.LBE294:
.LBE297:
	.loc 3 54 0
	mov	fp, fp, lsr #21
.LBB298:
.LBB295:
	.loc 3 20 0
	mov	r1, r9
.LBE295:
.LBE298:
	.loc 3 57 0
	mul	fp, r3, fp
.LBB299:
.LBB296:
	.loc 3 20 0
	bl	__aeabi_uidivmod
	.loc 3 19 0
	add	r1, fp, r1
	mov	fp, r8, lsr #25
.LBE296:
.LBE299:
	.loc 3 57 0
	mla	r9, fp, r9, r1
.LBE291:
.LBE290:
	.loc 3 1235 0
	uxth	r9, r9
	cmp	r9, r5
	bhi	.L459
	.loc 3 1237 0
	bne	.L460
	.loc 3 1238 0
	movw	r3, #1734
	ldrh	r3, [r4, r3]
	.loc 3 1237 0 discriminator 1
	cmp	r5, r3
	bcc	.L459
.L460:
	.loc 3 1242 0
	mov	r0, sl
	mov	r1, r8
	mov	r2, #2
	bl	sftl_update_vpn
	.loc 3 1243 0
	mov	r0, r7
	add	r1, sp, #8
	mov	r2, #1
	bl	sftl_log_to_phy
.LVL367:
.L459:
	.loc 3 1219 0
	add	r5, r5, #1
.L458:
	.loc 3 1219 0 is_stmt 0 discriminator 1
	ldr	r3, .L462+8
	ldrh	r3, [r6, r3]
	cmp	r5, r3
	bcc	.L461
	.loc 3 1245 0 is_stmt 1
	ldr	r1, .L462+4
	movw	r2, #1734
	strh	r3, [r1, r2]	@ movhi
	.loc 3 1246 0
	add	sp, sp, #36
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L463:
	.align	2
.L462:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	-3984
.LFE173:
	.fnend
	.size	sftl_recover_active_sblk, .-sftl_recover_active_sblk
	.align	2
	.global	sftl_l2p_load
	.type	sftl_l2p_load, %function
sftl_l2p_load:
	.fnstart
.LFB167:
	.loc 3 818 0
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
	.loc 3 830 0 discriminator 1
	ldr	r3, .L487
	.loc 3 831 0
	movw	r2, #65535
	.loc 3 818 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI52:
	.loc 3 830 0
	mov	r5, #0
.LVL368:
	.loc 3 830 0 is_stmt 0 discriminator 1
	ldrh	r0, [r3, #36]
	.loc 3 818 0 is_stmt 1
	.pad #52
	sub	sp, sp, #52
.LCFI53:
	.loc 3 830 0 discriminator 1
	ldr	r3, .L487+4
	.loc 3 830 0 is_stmt 0
	b	.L465
.L467:
	.loc 3 831 0 is_stmt 1
	ldrh	r1, [r3, #2]!
	cmp	r1, r2
	beq	.L466
	.loc 3 830 0
	add	r5, r5, #1
.LVL369:
	uxth	r5, r5
.LVL370:
.L465:
	.loc 3 830 0 is_stmt 0 discriminator 1
	cmp	r5, r0
	bne	.L467
.L466:
	.loc 3 834 0 is_stmt 1
	cmp	r5, #0
	bne	.L468
	.loc 3 835 0 discriminator 2
	ldr	r1, .L487+8
	movw	r2, #835
	ldr	r0, .L487+12
	bl	printk
	.loc 3 835 0 is_stmt 0 discriminator 3
	bl	dump_ftl_info
.L469:
	b	.L469
.L468:
	.loc 3 839 0 is_stmt 1
	ldr	r4, .L487
	.loc 3 844 0
	mov	r0, r0, asl #2
	.loc 3 841 0
	sub	r6, r5, #1
	.loc 3 839 0
	ldrh	r3, [r4, #38]
	.loc 3 841 0
	uxth	r6, r6
.LVL371:
	.loc 3 839 0
	str	r3, [sp, #24]
	.loc 3 844 0
	bl	ftl_malloc
.LBB300:
	.loc 3 845 0
	ldrh	r1, [r4, #36]
	movs	r1, r1, asl #2
.LVL372:
.LBE300:
	.loc 3 844 0
	str	r0, [sp, #12]
.LBB301:
	.loc 3 845 0
	beq	.L470
	.loc 3 845 0 is_stmt 0 discriminator 3
	bl	__memzero
.LVL373:
.L470:
.LBE301:
	.loc 3 848 0 is_stmt 1
	ldrh	r0, [r4, #32]
	mov	r0, r0, lsr #3
	bl	ftl_malloc
.LVL374:
.LBB302:
	.loc 3 849 0
	ldrh	r1, [r4, #32]
	movs	r1, r1, lsr #3
.LVL375:
.LBE302:
	.loc 3 848 0
	str	r0, [sp, #16]
.LBB303:
	.loc 3 849 0
	beq	.L471
	.loc 3 849 0 is_stmt 0 discriminator 3
	bl	__memzero
.LVL376:
.L471:
.LBE303:
	.loc 3 855 0 is_stmt 1
	ldr	r4, .L487
	.loc 3 830 0
	mov	sl, #0
.LVL377:
	.loc 3 858 0
	ldr	r7, .L487+16
	.loc 3 830 0
	mov	r8, sl
.LVL378:
	.loc 3 858 0
	str	r5, [sp, #28]
.LVL379:
.L479:
.LBB304:
	.loc 3 817 0
	rsb	r3, r8, r6
.LBE304:
	.loc 3 856 0
	mov	r1, #1
.LVL380:
.LBB305:
	.loc 3 817 0
	uxth	r3, r3
	str	r3, [sp, #20]
.LBE305:
	.loc 3 855 0
	add	r3, r3, #776
	add	r3, r4, r3, asl #1
	ldrh	r5, [r3, #48]
.LVL381:
	.loc 3 856 0
	mov	r0, r5
	bl	sftl_get_trace_list
	.loc 3 857 0
	cmp	r8, #0
	bne	.L472
	.loc 3 858 0
	ldrh	r2, [r4, #22]
	ldr	r0, [r7, #-3876]
	ldr	r1, [r7, #-3880]
	mov	r2, r2, asl #1
	bl	memcpy
.L472:
	.loc 3 872 0
	ldr	r0, [sp, #20]
	mov	r9, #0
	ldr	r3, [sp, #12]
	str	r6, [sp, #32]
	add	r2, r3, r0, asl #2
	str	r8, [sp, #36]
	mov	fp, r2
	b	.L473
.LVL382:
.L476:
	.loc 3 862 0
	sub	r8, r8, #2
	.loc 3 863 0
	ldr	r0, [r7, #-3880]
	.loc 3 862 0
	rsb	r8, r9, r8
	uxth	r8, r8
.LVL383:
	.loc 3 863 0
	mov	r1, r8, asl #1
	ldrh	r6, [r0, r1]
.LVL384:
	.loc 3 864 0
	movw	r1, #65535
	cmp	r6, r1
	beq	.L474
	.loc 3 866 0
	ldr	r3, [sp, #16]
	mov	r1, r6, lsr #5
	and	r0, r6, #31
	mov	r2, #1
	mov	r0, r2, asl r0
	ldr	ip, [r3, r1, asl #2]
	tst	ip, r0
	bne	.L474
	.loc 3 868 0
	orr	r0, ip, r0
	str	r0, [r3, r1, asl #2]
.LBB306:
.LBB307:
	.loc 3 39 0
	ldrh	ip, [r4, #14]
	.loc 3 40 0
	mov	r0, r8
.LBE307:
.LBE306:
	.loc 3 870 0
	ldr	r2, [r7, #-3900]
	.loc 3 871 0
	add	sl, sl, #1
.LVL385:
.LBB315:
.LBB312:
	.loc 3 40 0
	mov	r1, ip
.LBE312:
.LBE315:
	.loc 3 871 0
	uxth	sl, sl
.LVL386:
.LBB316:
.LBB313:
	.loc 3 40 0
	stmib	sp, {r2, ip}
	bl	__aeabi_uidivmod
.LBB308:
.LBB309:
	.loc 3 10 0
	ldrh	r0, [r4, #16]
.LBE309:
.LBE308:
	.loc 3 39 0
	ldr	ip, [sp, #8]
.LBB311:
.LBB310:
	.loc 3 10 0
	uxth	r1, r1
	add	r1, r7, r1, asl #1
	sub	r1, r1, #3968
	sub	r1, r1, #8
	ldrh	r1, [r1, #0]
	mla	r1, r0, r5, r1
.LBE310:
.LBE311:
	.loc 3 39 0
	mov	r0, r8
	.loc 3 42 0
	mov	r3, r1, asl #11
	.loc 3 39 0
	mov	r1, ip
	str	r3, [sp, #8]
	bl	__aeabi_uidiv
.LBE313:
.LBE316:
	.loc 3 870 0
	ldr	r2, [sp, #4]
.LBB317:
.LBB314:
	.loc 3 42 0
	ldr	r3, [sp, #8]
	uxth	r0, r0
	orr	r0, r3, r0
.LBE314:
.LBE317:
	.loc 3 870 0
	str	r0, [r2, r6, asl #2]
	.loc 3 872 0
	ldr	r1, [fp, #0]
	add	r1, r1, #1
	str	r1, [fp, #0]
	.loc 3 873 0
	ldrh	r1, [r4, #32]
	cmp	r1, sl
	beq	.L475
.L474:
	.loc 3 861 0
	add	r9, r9, #1
.LVL387:
	uxth	r9, r9
.LVL388:
.L473:
	.loc 3 861 0 is_stmt 0 discriminator 1
	ldrh	r8, [r4, #22]
.LVL389:
	sub	r1, r8, #1
	cmp	r9, r1
	blt	.L476
.LVL390:
.L475:
	.loc 3 877 0 is_stmt 1
	ldr	r0, [sp, #20]
	ldr	r2, [sp, #12]
	ldr	r3, [r7, #-3908]
	ldr	r6, [sp, #32]
.LVL391:
	ldr	r1, [r2, r0, asl #2]
	ldr	r2, [r3, r5, asl #2]
	ldr	r8, [sp, #36]
.LVL392:
	ubfx	r0, r2, #16, #9
	cmp	r0, r1
	.loc 3 883 0
	bfine	r2, r1, #16, #9
	strne	r2, [r3, r5, asl #2]
	.loc 3 885 0
	ldrh	r1, [r4, #32]
	cmp	sl, r1
	beq	.L478
	.loc 3 853 0
	add	r8, r8, #1
.LVL393:
	uxth	r8, r8
.LVL394:
	.loc 3 853 0 is_stmt 0 discriminator 1
	cmp	r8, r6
	bls	.L479
.L478:
	.loc 3 889 0 is_stmt 1
	ldr	r7, .L487
	.loc 3 888 0
	mov	r1, r1, lsr #3
	ldr	r0, [sp, #16]
	ldr	r5, [sp, #28]
.LVL395:
	bl	ftl_free
	.loc 3 889 0
	ldrh	r1, [r7, #36]
	ldr	r0, [sp, #12]
.LBB318:
	.loc 3 892 0
	ldr	r4, .L487+16
.LBE318:
	.loc 3 889 0
	mov	r1, r1, asl #2
	bl	ftl_free
.LBB319:
	.loc 3 892 0 discriminator 3
	ldr	r0, .L487+20
	mov	r1, #20
	bl	__memzero
.LBE319:
	.loc 3 893 0
	ldr	r3, .L487+24
	.loc 3 899 0
	ldr	r2, [r4, #-3908]
	.loc 3 894 0
	ldr	r1, .L487+28
	.loc 3 893 0
	strh	r6, [r4, r3]	@ movhi
	.loc 3 894 0
	add	r3, r6, #776
	add	r7, r7, r3, asl #1
	ldrh	r3, [r7, #48]
	.loc 3 899 0
	add	r0, r2, r3, asl #2
	.loc 3 894 0
	strh	r3, [r4, r1]	@ movhi
	.loc 3 899 0
	ldrb	r0, [r0, #3]	@ zero_extendqisi2
	and	r0, r0, #14
	cmp	r0, #2
	beq	.L480
	.loc 3 901 0
	ldr	r0, [r2, r3, asl #2]
	mov	r7, #1
	bfi	r0, r7, #25, #3
	str	r0, [r2, r3, asl #2]
	.loc 3 902 0
	ldrh	r0, [r4, r1]
	mov	r1, #384
	bl	__aeabi_uidiv
	ldr	r3, [r4, #-3932]
	uxth	r0, r0
	str	r0, [r4, #-3928]
	orr	r7, r3, r7, asl r0
	str	r7, [r4, #-3932]
.L480:
	.loc 3 904 0
	ldr	r4, .L487+16
	ldr	r7, .L487+28
	ldr	r3, [r4, #-3908]
	ldrh	r2, [r4, r7]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr #28
	str	r0, [r4, #-4004]
	.loc 3 905 0
	bl	sftl_select_trace_offs
	ldr	r3, .L487+32
	.loc 3 906 0
	add	r1, sp, #40
	.loc 3 905 0
	strh	r0, [r4, r3]	@ movhi
	.loc 3 906 0
	ldrh	r0, [r4, r7]
	bl	sftl_get_last_written_offset
	.loc 3 907 0
	ldr	r3, .L487+36
	.loc 3 908 0
	cmn	r0, #1
	.loc 3 907 0
	add	r2, r0, #1
	strh	r2, [r4, r3]	@ movhi
	.loc 3 908 0
	bne	.L483
.L481:
	.loc 3 910 0
	cmp	r6, #0
	beq	.L482
	.loc 3 912 0
	ldr	r3, .L487
	add	r6, r6, #772
.LVL396:
	.loc 3 913 0
	add	r1, sp, #40
	.loc 3 912 0
	add	r6, r3, r6, asl #1
	.loc 3 913 0
	ldrh	r0, [r6, #54]
	bl	sftl_get_last_written_offset
	.loc 3 914 0
	cmn	r0, #1
	bne	.L483
	.loc 3 914 0 is_stmt 0 discriminator 2
	ldr	r1, .L487+8
	movw	r2, #914
	ldr	r0, .L487+12
	bl	printk
	.loc 3 914 0 discriminator 3
	bl	dump_ftl_info
.L484:
	b	.L484
.LVL397:
.L483:
	.loc 3 915 0 is_stmt 1
	ldr	r3, [sp, #40]
	str	r3, [r4, #-3992]
.L482:
	.loc 3 918 0
	ldr	r3, [sp, #24]
	cmp	r5, r3
	bls	.L485
	.loc 3 919 0
	bl	sftl_sys_info_flush
	.loc 3 920 0
	bl	sftl_l2p_gc
.L485:
	.loc 3 924 0
	mov	r0, #0
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L488:
	.align	2
.L487:
	.word	.LANCHOR0
	.word	.LANCHOR0+1598
	.word	.LC7
	.word	.LC5
	.word	.LANCHOR1
	.word	.LANCHOR1-4008
	.word	-4008
	.word	-4006
	.word	-3998
	.word	-4000
.LFE167:
	.fnend
	.size	sftl_l2p_load, .-sftl_l2p_load
	.align	2
	.global	sftl_user_gc_by_level
	.type	sftl_user_gc_by_level, %function
sftl_user_gc_by_level:
	.fnstart
.LFB171:
	.loc 3 1049 0
	@ args = 0, pretend = 0, frame = 48
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL398:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI54:
	.pad #52
	sub	sp, sp, #52
.LCFI55:
	.loc 3 1053 0
	mov	r3, #0
	.loc 3 1049 0
	mov	sl, r0
	.loc 3 1053 0
	str	r3, [sp, #8]
	.loc 3 1060 0
	ldr	r5, .L514
	b	.L513
.LVL399:
.L510:
	.loc 3 1062 0
	mov	r3, #1
	str	r3, [sp, #8]
.LVL400:
.L513:
	.loc 3 1057 0
	ldr	r0, [sp, #8]
	add	r1, sp, #23
	bl	user_gc_get_wait_offs
.LVL401:
	.loc 3 1058 0
	movw	r3, #65535
	ldr	r6, .L514+4
	cmp	r0, r3
	bne	.L491
	.loc 3 1059 0
	mov	r3, #0
	strb	r3, [r6, #-3980]
	.loc 3 1060 0
	mov	r3, #1728
	ldrh	r2, [r5, r3]
	ldr	r3, .L514+8
	ldrh	r3, [r6, r3]
	cmp	r2, r3
	bcs	.L510
	b	.L493
.L491:
	.loc 3 1068 0
	ldr	r3, [r6, #-3948]
	mov	r0, r0, asl #1
	.loc 3 1077 0
	ldr	r7, .L514+12
	.loc 3 1068 0
	str	r0, [sp, #12]
	ldrh	r4, [r3, r0]
	.loc 3 1077 0
	ldrh	r3, [r6, r7]
	cmp	r3, r4
	beq	.L494
	.loc 3 1078 0
	mov	r1, #2
	mov	r0, r4
	bl	sftl_get_trace_list
	.loc 3 1079 0
	ldrh	r2, [r5, #22]
	ldr	r0, [r6, #-3936]
	ldr	r1, [r6, #-3872]
	mov	r2, r2, asl #2
	bl	memcpy
	.loc 3 1080 0
	strh	r4, [r6, r7]	@ movhi
.L494:
	.loc 3 1082 0
	ldr	r7, .L514+4
	ldr	r3, .L514+16
	ldrh	r6, [r7, r3]
.LVL402:
	b	.L495
.L503:
	.loc 3 1083 0
	ldr	r3, [r7, #-3936]
	ldr	r8, [r3, r6, asl #2]
	.loc 3 1084 0
	cmn	r8, #1
	beq	.L496
	.loc 3 1086 0
	mov	r0, r8
	add	r1, sp, #24
	mov	r2, #0
	bl	sftl_log_to_phy
	.loc 3 1087 0
	ldr	fp, [sp, #24]
	cmp	fp, #0
	bne	.L497
	.loc 3 1087 0 is_stmt 0 discriminator 2
	ldr	r1, .L514+20
	movw	r2, #1087
	ldr	r0, .L514+24
	bl	printk
	.loc 3 1087 0 discriminator 3
	bl	dump_ftl_info
.L498:
	b	.L498
.L497:
.LBB320:
.LBB321:
	.loc 3 39 0 is_stmt 1
	ldrh	r3, [r5, #14]
	.loc 3 40 0
	mov	r0, r6
	mov	r1, r3
	str	r3, [sp, #4]
	bl	__aeabi_uidivmod
	.loc 3 39 0
	ldr	r3, [sp, #4]
	mov	r0, r6
.LBB322:
.LBB323:
	.loc 3 10 0
	uxth	r2, r1
	ldrh	r1, [r5, #16]
	add	r2, r7, r2, asl #1
	sub	r2, r2, #3968
	sub	r2, r2, #8
	ldrh	r2, [r2, #0]
	mla	r2, r1, r4, r2
.LBE323:
.LBE322:
	.loc 3 39 0
	mov	r1, r3
	.loc 3 42 0
	mov	r2, r2, asl #11
	.loc 3 39 0
	str	r2, [sp, #4]
	bl	__aeabi_uidiv
	.loc 3 42 0
	ldr	r2, [sp, #4]
	uxth	r0, r0
	orr	r2, r2, r0
.LBE321:
.LBE320:
	.loc 3 1088 0
	cmp	fp, r2
	bne	.L496
	.loc 3 1093 0
	ldr	r3, [r7, #-3920]
	.loc 3 1095 0
	mov	r1, #1
	add	r0, sp, #28
	mov	r2, #0
	.loc 3 1092 0
	str	fp, [sp, #32]
	.loc 3 1093 0
	str	r3, [sp, #36]
	.loc 3 1094 0
	ldr	r3, [r7, #-3912]
	str	r3, [sp, #40]
	.loc 3 1095 0
	bl	ftl_flash_read_pages
	.loc 3 1100 0
	ldr	r1, [sp, #28]
	cmn	r1, #1
	beq	.L499
	.loc 3 1100 0 is_stmt 0 discriminator 1
	ldr	r3, [r7, #-3912]
	ldr	r0, [r3, #4]
	cmp	r0, r8
	beq	.L500
.L499:
	.loc 3 1101 0 is_stmt 1 discriminator 1
	ldr	r4, .L514+4
.LVL403:
	mov	r2, r8
	ldr	r0, .L514+28
	ldr	r3, [r4, #-3912]
	ldr	r3, [r3, #4]
	bl	printk
	.loc 3 1106 0
	ldr	r3, .L514
	ldr	r0, .L514+32
	mov	r2, #4
	ldr	r1, [r4, #-3920]
	ldr	r3, [r3, #8]
	mov	r3, r3, lsr #2
	bl	rknand_print_hex
	.loc 3 1107 0
	ldr	r0, .L514+36
	ldr	r1, [r4, #-3912]
	mov	r2, #4
	mov	r3, #2
	bl	rknand_print_hex
	.loc 3 1108 0 discriminator 2
	ldr	r1, .L514+20
	movw	r2, #1108
	ldr	r0, .L514+24
	bl	printk
	.loc 3 1108 0 is_stmt 0 discriminator 3
	bl	dump_ftl_info
.L501:
	b	.L501
.LVL404:
.L500:
	.loc 3 1111 0 is_stmt 1
	mov	r1, #2
	bl	sftl_write_page
	.loc 3 1113 0
	cmp	sl, #0
	.loc 3 1114 0
	ldreq	r3, .L514+16
	addeq	r6, r6, #1
.LVL405:
	streqh	r6, [r9, r3]	@ movhi
	.loc 3 1115 0
	beq	.L502
.LVL406:
.L496:
	.loc 3 1082 0
	add	r6, r6, #1
.LVL407:
	uxth	r6, r6
.LVL408:
.L495:
	.loc 3 1082 0 is_stmt 0 discriminator 1
	ldrh	r3, [r5, #22]
	ldr	r9, .L514+4
	sub	r3, r3, #1
	cmp	r6, r3
	blt	.L503
	.loc 3 1118 0 is_stmt 1
	ldr	r3, .L514+16
	mov	r1, #0
	strh	r1, [r9, r3]	@ movhi
	.loc 3 1131 0
	ldr	r3, [r9, #-3908]
	ldr	r2, [r3, r4, asl #2]
	bfi	r2, r1, #16, #9
	str	r2, [r3, r4, asl #2]
.L502:
	.loc 3 1134 0
	ldr	r6, .L514+4
.LVL409:
	mov	r7, r4, asl #2
	ldr	r3, [r6, #-3908]
	add	r3, r3, r7
	ldrh	r3, [r3, #2]
	movs	r3, r3, asl #23
	bne	.L504
	.loc 3 1135 0
	cmp	sl, #0
	bne	.L505
	.loc 3 1136 0
	mov	r0, r4
	bl	sftl_user_check_real_vpn
	.loc 3 1137 0
	subs	r8, r0, #0
.LVL410:
	beq	.L506
	.loc 3 1138 0 discriminator 1
	mov	r1, r4
	mov	r2, r8
	ldr	r0, .L514+40
	bl	printk
	.loc 3 1139 0
	ldr	r3, [r6, #-3908]
	.loc 3 1140 0
	mov	r0, r4
	mov	r1, #384
	.loc 3 1139 0
	ldr	r2, [r3, r7]
	bfi	r2, r8, #16, #9
	str	r2, [r3, r7]
	.loc 3 1140 0
	bl	__aeabi_uidiv
	ldr	r3, [r6, #-3932]
	mov	r2, #1
	uxth	r0, r0
	str	r0, [r6, #-3928]
	orr	r3, r3, r2, asl r0
	str	r3, [r6, #-3932]
	.loc 3 1141 0
	b	.L493
.L506:
	.loc 3 1143 0
	ldr	r3, .L514+16
	strh	sl, [r6, r3]	@ movhi
.LVL411:
.L505:
	.loc 3 1145 0
	ldr	r3, [r6, #-3948]
	mvn	r2, #0
	ldr	r1, [sp, #12]
	ldr	r7, .L514+4
	strh	r2, [r3, r1]	@ movhi
	.loc 3 1146 0
	ldr	r2, .L514+44
	ldrh	r3, [r6, r2]
	add	r3, r3, #1
	uxth	r3, r3
	strh	r3, [r6, r2]	@ movhi
	.loc 3 1147 0
	cmp	r3, #64
	beq	.L507
	.loc 3 1148 0
	ldr	r2, [r7, #-3948]
	mov	r3, r3, asl #1
	.loc 3 1147 0 discriminator 1
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	bne	.L508
.L507:
	.loc 3 1149 0
	ldr	r3, .L514+44
	mvn	r2, #0
	strh	r2, [r7, r3]	@ movhi
.L508:
	.loc 3 1150 0
	ldr	r3, .L514+12
	mvn	r2, #0
	.loc 3 1152 0
	mov	r0, r4
	.loc 3 1150 0
	strh	r2, [r7, r3]	@ movhi
	.loc 3 1152 0
	bl	sftl_free_sblk
	.loc 3 1153 0
	mov	r3, #1728
	ldrh	r2, [r5, r3]
	.loc 3 1154 0
	mov	r1, #1
	.loc 3 1153 0
	sub	r2, r2, #1
	strh	r2, [r5, r3]	@ movhi
	.loc 3 1154 0
	ldr	r2, [r7, #-3932]
	ldr	r3, [r7, #-3928]
	orr	r3, r2, r1, asl r3
	str	r3, [r7, #-3932]
	.loc 3 1155 0
	bl	sftl_l2p_flush
	.loc 3 1156 0
	bl	sftl_sys_info_flush
.L504:
	.loc 3 1158 0
	cmp	sl, #2
	bne	.L509
	.loc 3 1158 0 is_stmt 0 discriminator 1
	ldrb	r3, [sp, #23]	@ zero_extendqisi2
.LVL412:
	cmp	r3, #0
	beq	.L513
.L509:
	.loc 3 1164 0 is_stmt 1
	mov	r2, #1728
	ldr	r3, .L514+4
.LVL413:
	ldrh	r1, [r5, r2]
	ldr	r2, .L514+8
	ldrh	r2, [r3, r2]
	cmp	r1, r2
	bcs	.L513
	.loc 3 1167 0
	mov	r2, #0
	strb	r2, [r3, #-3980]
.L493:
	.loc 3 1170 0
	mov	r0, #0
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L515:
	.align	2
.L514:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	-3952
	.word	-3940
	.word	-3942
	.word	.LC7
	.word	.LC5
	.word	.LC13
	.word	.LC10
	.word	.LC11
	.word	.LC14
	.word	-3944
.LFE171:
	.fnend
	.size	sftl_user_gc_by_level, .-sftl_user_gc_by_level
	.align	2
	.global	sftl_user_gc
	.type	sftl_user_gc, %function
sftl_user_gc:
	.fnstart
.LFB172:
	.loc 3 1173 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL414:
	stmfd	sp!, {r3, r4, r5, lr}
	.save {r3, r4, r5, lr}
.LCFI56:
	.loc 3 1176 0
	cmp	r0, #3
	ldrls	pc, [pc, r0, asl #2]
	b	.L517
.L522:
	.word	.L518
	.word	.L519
	.word	.L520
	.word	.L521
.L518:
	.loc 3 1178 0
	ldr	r3, .L529
	mov	r2, #1728
	ldrh	r1, [r3, #4]
	ldrh	r0, [r3, #18]
.LVL415:
	ldrh	r2, [r3, r2]
	rsb	r0, r1, r0
	ldrh	r1, [r3, #36]
	rsb	r3, r1, r0
	sub	r3, r3, #40
	cmp	r2, r3
	blt	.L517
	b	.L528
.LVL416:
.L519:
	.loc 3 1182 0
	ldr	r3, .L529
	mov	r2, #1728
	ldrh	r1, [r3, #4]
	ldrh	r0, [r3, #18]
.LVL417:
	ldrh	r2, [r3, r2]
	rsb	r0, r1, r0
	ldrh	r1, [r3, #36]
	rsb	r3, r1, r0
	sub	r1, r3, #40
	cmp	r2, r1
	blt	.L517
	.loc 3 1183 0
	sub	r3, r3, #30
	cmp	r2, r3
	.loc 3 1184 0
	movge	r0, #1
	.loc 3 1183 0
	bge	.L527
	b	.L528
.LVL418:
.L520:
	.loc 3 1190 0
	ldr	r4, .L529
	mov	r5, #1728
	ldrh	r1, [r4, #18]
	ldrh	r3, [r4, #4]
	ldrh	r2, [r4, r5]
	rsb	r3, r3, r1
	ldrh	r1, [r4, #36]
	rsb	r3, r1, r3
	sub	r3, r3, #40
	cmp	r2, r3
	bge	.L526
.L528:
	.loc 3 1191 0
	mov	r0, #0
.LVL419:
	b	.L527
.LVL420:
.L525:
	.loc 3 1194 0
	mov	r0, #1
.LVL421:
	bl	sftl_user_gc_by_level
.LVL422:
.L526:
	.loc 3 1193 0 discriminator 1
	ldrh	r1, [r4, #18]
	ldrh	r3, [r4, #4]
	ldrh	r2, [r4, r5]
	rsb	r3, r3, r1
	ldrh	r1, [r4, #36]
	rsb	r3, r1, r3
	sub	r3, r3, #40
	cmp	r2, r3
	bge	.L525
	b	.L517
.L521:
	.loc 3 1198 0
	mov	r0, #2
.LVL423:
.L527:
	bl	sftl_user_gc_by_level
.LVL424:
.L517:
	.loc 3 1202 0
	mov	r0, #0
.LVL425:
	ldmfd	sp!, {r3, r4, r5, pc}
.L530:
	.align	2
.L529:
	.word	.LANCHOR0
.LFE172:
	.fnend
	.size	sftl_user_gc, .-sftl_user_gc
	.align	2
	.global	sftl_gc
	.type	sftl_gc, %function
sftl_gc:
	.fnstart
.LFB141:
	.loc 1 470 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.loc 1 471 0
	mov	r0, #2
	.loc 1 472 0
	.loc 1 471 0
	b	sftl_user_gc
.LFE141:
	.fnend
	.size	sftl_gc, .-sftl_gc
	.align	2
	.global	sftl_read
	.type	sftl_read, %function
sftl_read:
	.fnstart
.LFB132:
	.loc 1 225 0
	@ args = 0, pretend = 0, frame = 32
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL426:
	.loc 1 235 0
	ldr	r3, .L548
	.loc 1 225 0
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI57:
	mov	sl, r2
	.loc 1 235 0
	ldr	r2, [r3, #40]
.LVL427:
	add	r9, r1, r0
	.loc 1 225 0
	.pad #44
	sub	sp, sp, #44
.LCFI58:
	.loc 1 225 0
	mov	r8, r0
	.loc 1 235 0
	cmp	r9, r2
	.loc 1 225 0
	mov	r7, r1
	.loc 1 235 0
	bhi	.L544
	.loc 1 237 0
	cmp	r1, #0
	beq	.L545
	.loc 1 240 0
	ldrh	r4, [r3, #12]
	ldr	r5, .L548+4
	mov	r1, r4
.LVL428:
	bl	__aeabi_uidiv
.LVL429:
	.loc 1 241 0
	mov	r1, r4
	.loc 1 240 0
	mov	fp, r0
.LVL430:
	.loc 1 241 0
	sub	r0, r9, #1
	bl	__aeabi_uidiv
	.loc 1 242 0
	rsb	r6, fp, #1
	.loc 1 243 0
	mov	r4, fp
.LVL431:
	.loc 1 241 0
	str	r0, [sp, #12]
	.loc 1 242 0
	add	r6, r6, r0
.LVL432:
	.loc 1 245 0
	b	.L534
.LVL433:
.L543:
	.loc 1 246 0
	mov	r0, r4
	add	r1, sp, #16
	mov	r2, #0
	bl	sftl_log_to_phy
	.loc 1 247 0
	ldr	r3, [sp, #16]
	cmp	r3, #0
	bne	.L535
.LBB324:
	.loc 1 248 0
	ldr	r3, .L548
	ldr	r0, [r5, #-3920]
.LVL434:
	ldr	r1, [r3, #8]
.LVL435:
	cmp	r1, #0
	beq	.L536
	.loc 1 248 0 is_stmt 0 discriminator 3
	bl	__memzero
.LVL436:
	b	.L536
.L535:
.LBE324:
	.loc 1 250 0 is_stmt 1
	str	r3, [sp, #24]
	.loc 1 253 0
	add	r0, sp, #20
	.loc 1 251 0
	ldr	r3, [r5, #-3920]
	.loc 1 253 0
	mov	r1, #1
	mov	r2, #0
	.loc 1 251 0
	str	r3, [sp, #28]
	.loc 1 252 0
	ldr	r3, [r5, #-3912]
	str	r3, [sp, #32]
	.loc 1 253 0
	bl	ftl_flash_read_pages
	.loc 1 254 0
	ldr	r3, [sp, #20]
	cmp	r3, #256
	bne	.L537
	.loc 1 255 0
	ldr	r0, [sp, #16]
	bl	sftl_ppn_to_sblk
	.loc 1 256 0
	bl	sftl_add_sblk_to_wait_gc_pool
.L537:
	.loc 1 262 0
	ldr	r1, [sp, #20]
	cmn	r1, #1
	beq	.L538
	.loc 1 262 0 is_stmt 0 discriminator 1
	ldr	r3, [r5, #-3912]
	ldr	r3, [r3, #4]
	cmp	r3, r4
	beq	.L536
.L538:
	.loc 1 263 0 is_stmt 1 discriminator 1
	ldr	r6, [sp, #16]
.LVL437:
	str	r1, [sp, #8]
	ldr	r5, .L548+4
	mov	r0, r6
	bl	sftl_ppn_to_sblk
	str	r6, [sp, #0]
	ldr	r3, [r5, #-3912]
	mov	r2, r4
	ldr	r1, [sp, #8]
	str	r0, [sp, #4]
	ldr	r0, .L548+8
	ldr	r3, [r3, #4]
	bl	printk
	.loc 1 270 0
	ldr	r3, .L548
	ldr	r0, .L548+12
	mov	r2, #4
	ldr	r1, [r5, #-3920]
	ldr	r3, [r3, #8]
	mov	r3, r3, lsr #2
	bl	rknand_print_hex
	.loc 1 271 0
	ldr	r0, .L548+16
	ldr	r1, [r5, #-3912]
	mov	r2, #4
	mov	r3, #2
	bl	rknand_print_hex
	.loc 1 272 0 discriminator 2
	ldr	r1, .L548+20
	mov	r2, #272
	ldr	r0, .L548+24
	bl	printk
	.loc 1 272 0 is_stmt 0 discriminator 3
	bl	dump_ftl_info
.L539:
	b	.L539
.LVL438:
.L536:
	.loc 1 276 0 is_stmt 1
	cmp	r4, fp
	ldr	r3, .L548
	bne	.L540
	.loc 1 277 0
	ldrh	r2, [r3, #12]
	.loc 1 281 0
	mov	r0, sl
.LVL439:
	ldr	r1, [r5, #-3920]
.LVL440:
	.loc 1 277 0
	sub	r3, r2, #1
	and	r3, r3, r8
.LVL441:
	.loc 1 278 0
	rsb	r2, r3, r2
	.loc 1 281 0
	cmp	r2, r7
	movcs	r2, r7
	add	r1, r1, r3, asl #9
	b	.L547
.LVL442:
.L540:
	.loc 1 284 0
	ldr	r2, [sp, #12]
	cmp	r4, r2
	bne	.L542
	.loc 1 286 0
	ldrh	r2, [r3, #12]
	.loc 1 287 0
	ldr	r1, [r5, #-3920]
.LVL443:
	.loc 1 286 0
	mul	r2, r2, r4
	.loc 1 287 0
	rsb	r0, r8, r2
.LVL444:
	.loc 1 286 0
	rsb	r2, r2, r9
	.loc 1 287 0
	add	r0, sl, r0, asl #9
.LVL445:
.L547:
	mov	r2, r2, asl #9
	b	.L546
.LVL446:
.L542:
	.loc 1 291 0
	ldrh	r0, [r3, #12]
.LVL447:
	ldr	r1, [r5, #-3920]
.LVL448:
	ldr	r2, [r3, #8]
	mul	r0, r0, r4
	rsb	r0, r8, r0
	add	r0, sl, r0, asl #9
.LVL449:
.L546:
	bl	memcpy
.LVL450:
	.loc 1 295 0
	add	r4, r4, #1
	.loc 1 296 0
	sub	r6, r6, #1
.L534:
	.loc 1 245 0 discriminator 1
	cmp	r6, #0
	bne	.L543
	.loc 1 298 0
	ldr	r3, .L548+4
	.loc 1 299 0
	mov	r0, r6
	.loc 1 298 0
	ldr	r2, [r3, #-3968]
	add	r2, r2, #1
	str	r2, [r3, #-3968]
	.loc 1 299 0
	bl	sftl_user_gc
	.loc 1 300 0
	mov	r0, r6
.LVL451:
	b	.L533
.LVL452:
.L544:
	.loc 1 236 0
	mvn	r0, #0
.LVL453:
	b	.L533
.LVL454:
.L545:
	.loc 1 238 0
	mov	r0, r1
.LVL455:
.L533:
.LVL456:
	.loc 1 301 0
	add	sp, sp, #44
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L549:
	.align	2
.L548:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC15
	.word	.LC10
	.word	.LC11
	.word	.LC16
	.word	.LC5
.LFE132:
	.fnend
	.size	sftl_read, .-sftl_read
	.align	2
	.global	sftl_write
	.type	sftl_write, %function
sftl_write:
	.fnstart
.LFB133:
	.loc 1 304 0
	@ args = 0, pretend = 0, frame = 40
	@ frame_needed = 0, uses_anonymous_args = 0
.LVL457:
	stmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, lr}
	.save {r4, r5, r6, r7, r8, r9, sl, fp, lr}
.LCFI59:
	.pad #52
	sub	sp, sp, #52
.LCFI60:
	.loc 1 313 0
	ldr	r6, .L566
	add	r3, r1, r0
	str	r3, [sp, #12]
	.loc 1 304 0
	mov	fp, r2
	.loc 1 313 0
	ldr	r2, [sp, #12]
.LVL458:
	.loc 1 304 0
	mov	sl, r0
	.loc 1 313 0
	ldr	r3, [r6, #40]
	.loc 1 304 0
	mov	r9, r1
	.loc 1 313 0
	cmp	r2, r3
	bhi	.L562
	.loc 1 315 0
	cmp	r1, #0
	beq	.L563
	.loc 1 318 0
	ldrh	r4, [r6, #12]
	.loc 1 369 0
	ldr	r5, .L566+4
	.loc 1 318 0
	mov	r1, r4
.LVL459:
	bl	__aeabi_uidiv
.LVL460:
	.loc 1 319 0
	ldr	r3, [sp, #12]
	mov	r1, r4
	.loc 1 318 0
	mov	r8, r0
.LVL461:
	.loc 1 319 0
	sub	r0, r3, #1
	bl	__aeabi_uidiv
	.loc 1 320 0
	rsb	r7, r8, #1
	.loc 1 321 0
	mov	r4, r8
.LVL462:
	.loc 1 319 0
	str	r0, [sp, #16]
	.loc 1 320 0
	add	r7, r7, r0
.LVL463:
	.loc 1 323 0
	b	.L552
.LVL464:
.L561:
	.loc 1 324 0
	rsb	r0, r8, r4
	rsbs	r3, r0, #0
	adc	r3, r3, r0
	str	r3, [sp, #20]
	ldr	r3, [sp, #16]
	cmp	r4, r8
	cmpne	r4, r3
	bne	.L553
	.loc 1 326 0
	mov	r0, r4
	add	r1, sp, #24
	mov	r2, #0
	bl	sftl_log_to_phy
	.loc 1 327 0
	ldr	r3, [sp, #24]
	cmp	r3, #0
	bne	.L554
.LBB325:
	.loc 1 328 0
	ldr	r1, [r6, #8]
.LVL465:
	ldr	r0, [r5, #-3920]
.LVL466:
	cmp	r1, #0
	beq	.L555
	.loc 1 328 0 is_stmt 0 discriminator 3
	bl	__memzero
.LVL467:
	b	.L555
.L554:
.LBE325:
	.loc 1 330 0 is_stmt 1
	str	r3, [sp, #32]
	.loc 1 333 0
	mov	r1, #1
	.loc 1 331 0
	ldr	r3, [r5, #-3920]
	.loc 1 333 0
	add	r0, sp, #28
	mov	r2, #0
	.loc 1 331 0
	str	r3, [sp, #36]
	.loc 1 332 0
	ldr	r3, [r5, #-3912]
	str	r3, [sp, #40]
	.loc 1 333 0
	bl	ftl_flash_read_pages
	.loc 1 338 0
	ldr	r1, [sp, #28]
	cmn	r1, #1
	beq	.L556
	.loc 1 338 0 is_stmt 0 discriminator 1
	ldr	r3, [r5, #-3912]
	ldr	r3, [r3, #4]
	cmp	r3, r4
	beq	.L555
.L556:
	.loc 1 339 0 is_stmt 1 discriminator 1
	ldr	r6, [sp, #24]
	str	r1, [sp, #8]
	ldr	r5, .L566+4
	mov	r0, r6
	bl	sftl_ppn_to_sblk
	str	r6, [sp, #0]
	ldr	r3, [r5, #-3912]
	mov	r2, r4
	ldr	r1, [sp, #8]
	str	r0, [sp, #4]
	ldr	r0, .L566+8
	ldr	r3, [r3, #4]
	bl	printk
	.loc 1 346 0
	ldr	r3, .L566
	ldr	r0, .L566+12
	mov	r2, #4
	ldr	r1, [r5, #-3920]
	ldr	r3, [r3, #8]
	mov	r3, r3, lsr #2
	bl	rknand_print_hex
	.loc 1 347 0
	ldr	r0, .L566+16
	ldr	r1, [r5, #-3912]
	mov	r2, #4
	mov	r3, #2
	bl	rknand_print_hex
	.loc 1 348 0 discriminator 2
	ldr	r1, .L566+20
	mov	r2, #348
	ldr	r0, .L566+24
	bl	printk
	.loc 1 348 0 is_stmt 0 discriminator 3
	bl	dump_ftl_info
.L557:
	b	.L557
.LVL468:
.L555:
	.loc 1 353 0 is_stmt 1
	ldr	r3, [sp, #20]
	cmp	r3, #0
	beq	.L558
	.loc 1 354 0
	ldrh	r2, [r6, #12]
	.loc 1 358 0
	mov	r1, fp
.LVL469:
	ldr	r0, [r5, #-3920]
.LVL470:
	.loc 1 354 0
	sub	r3, r2, #1
	and	r3, r3, sl
.LVL471:
	.loc 1 355 0
	rsb	r2, r3, r2
	.loc 1 358 0
	cmp	r2, r9
	movcs	r2, r9
	add	r0, r0, r3, asl #9
	b	.L565
.LVL472:
.L558:
	.loc 1 363 0
	ldrh	r1, [r6, #12]
.LVL473:
	ldr	r3, [sp, #12]
	.loc 1 364 0
	ldr	r0, [r5, #-3920]
.LVL474:
	.loc 1 363 0
	mul	r1, r1, r4
	rsb	r2, r1, r3
	.loc 1 365 0
	rsb	r1, sl, r1
	.loc 1 364 0
	add	r1, fp, r1, asl #9
.LVL475:
.L565:
	mov	r2, r2, asl #9
	b	.L564
.LVL476:
.L553:
	.loc 1 370 0
	ldrh	r1, [r6, #12]
	.loc 1 369 0
	ldr	r0, [r5, #-3920]
	ldr	r2, [r6, #8]
	.loc 1 370 0
	mul	r1, r1, r4
	rsb	r1, sl, r1
	.loc 1 369 0
	add	r1, fp, r1, asl #9
.LVL477:
.L564:
	bl	memcpy
.LVL478:
	.loc 1 373 0
	mov	r0, r4
	mov	r1, #2
	bl	sftl_write_page
	.loc 1 374 0
	ldrb	r3, [r5, #-3980]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L560
	.loc 1 375 0
	mov	r0, #3
	bl	sftl_user_gc
.L560:
	.loc 1 376 0
	add	r4, r4, #1
	.loc 1 377 0
	sub	r7, r7, #1
.L552:
	.loc 1 323 0 discriminator 1
	cmp	r7, #0
	bne	.L561
	.loc 1 379 0
	ldr	r3, .L566+4
	.loc 1 380 0
	mov	r0, #1
	.loc 1 379 0
	ldr	r2, [r3, #-3964]
	add	r2, r2, #1
	str	r2, [r3, #-3964]
	.loc 1 380 0
	bl	sftl_user_gc
	.loc 1 381 0
	mov	r0, r7
.LVL479:
	b	.L551
.LVL480:
.L562:
	.loc 1 314 0
	mvn	r0, #0
.LVL481:
	b	.L551
.LVL482:
.L563:
	.loc 1 316 0
	mov	r0, r1
.LVL483:
.L551:
.LVL484:
	.loc 1 382 0
	add	sp, sp, #52
	ldmfd	sp!, {r4, r5, r6, r7, r8, r9, sl, fp, pc}
.L567:
	.align	2
.L566:
	.word	.LANCHOR0
	.word	.LANCHOR1
	.word	.LC17
	.word	.LC10
	.word	.LC11
	.word	.LC16
	.word	.LC5
.LFE133:
	.fnend
	.size	sftl_write, .-sftl_write
	.align	2
	.global	sftl_recovery
	.type	sftl_recovery, %function
sftl_recovery:
	.fnstart
.LFB175:
	.loc 3 1283 0
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r0, r1, r4, r5, r6, lr}
	.save {r0, r1, r4, r5, r6, lr}
.LCFI61:
	.loc 3 1289 0
	movw	r5, #1732
	ldr	r4, .L579
	ldr	r6, .L579+4
	ldr	r3, [r4, #-3908]
	ldrh	r2, [r6, r5]
	ldr	r0, [r3, r2, asl #2]
	mov	r0, r0, lsr #28
	str	r0, [r4, #-3988]
	.loc 3 1290 0
	bl	sftl_select_trace_offs
	ldr	r3, .L579+8
	.loc 3 1293 0
	mov	r1, sp
	.loc 3 1290 0
	strh	r0, [r4, r3]	@ movhi
	.loc 3 1291 0
	mov	r3, #1728
	ldrh	r2, [r6, r3]
	ldr	r3, .L579+12
	ldrh	r3, [r4, r3]
	cmp	r2, r3
	mov	r3, r5
	.loc 3 1293 0
	ldrh	r0, [r6, r3]
	.loc 3 1292 0
	movcs	r2, #1
	strcsb	r2, [r4, #-3980]
	.loc 3 1293 0
	bl	sftl_get_last_written_offset
	.loc 3 1294 0
	ldr	r4, .L579
	ldr	r3, .L579+16
	.loc 3 1293 0
	ldr	r5, .L579+4
	.loc 3 1294 0
	add	r0, r0, #1
	uxth	r0, r0
	strh	r0, [r4, r3]	@ movhi
	.loc 3 1296 0
	movw	r3, #1734
	ldrh	r3, [r6, r3]
	cmp	r3, r0
	bls	.L570
	.loc 3 1296 0 is_stmt 0 discriminator 2
	ldr	r1, .L579+20
	mov	r2, #1296
	ldr	r0, .L579+24
	bl	printk
	.loc 3 1296 0 discriminator 3
	bl	dump_ftl_info
.L571:
	b	.L571
.L570:
	.loc 3 1297 0 is_stmt 1
	cmp	r0, #0
	beq	.L572
	.loc 3 1298 0
	movw	r3, #1732
	mov	r1, #2
	ldrh	r0, [r5, r3]
	bl	sftl_get_trace_list
	.loc 3 1299 0
	ldrh	r2, [r5, #22]
	ldr	r0, [r4, #-3868]
	ldr	r1, [r4, #-3872]
	mov	r2, r2, asl #2
	bl	memcpy
.L572:
	.loc 3 1301 0
	ldr	r4, .L579
	ldrb	r0, [r4, #-3960]	@ zero_extendqisi2
	cmp	r0, #0
	beq	.L573
	.loc 3 1305 0
	ldr	r5, .L579+4
	.loc 3 1312 0
	movw	r1, #1732
	.loc 3 1305 0
	ldr	r3, [sp, #0]
	.loc 3 1312 0
	ldrh	r2, [r5, r1]
	.loc 3 1305 0
	str	r3, [r5, #2088]
	.loc 3 1312 0
	ldr	r3, [r4, #-3908]
	add	r0, r3, r2, asl #2
	ldrb	r0, [r0, #3]	@ zero_extendqisi2
	and	r0, r0, #14
	cmp	r0, #4
	beq	.L574
	.loc 3 1314 0
	ldr	r0, [r3, r2, asl #2]
	mov	ip, #2
	bfi	r0, ip, #25, #3
	str	r0, [r3, r2, asl #2]
	.loc 3 1315 0
	ldrh	r0, [r5, r1]
	mov	r1, #384
	bl	__aeabi_uidiv
	ldr	r2, [r4, #-3932]
	mov	r3, #1
	.loc 3 1316 0
	mov	r1, #1728
	.loc 3 1315 0
	uxth	r0, r0
	str	r0, [r4, #-3928]
	orr	r2, r2, r3, asl r0
	str	r2, [r4, #-3932]
	.loc 3 1316 0
	ldrh	r2, [r5, r1]
	add	r2, r2, r3
	uxth	r2, r2
	strh	r2, [r5, r1]	@ movhi
	.loc 3 1317 0
	ldr	r1, .L579+12
	ldrh	r1, [r4, r1]
	cmp	r1, r2
	.loc 3 1318 0
	strlsb	r3, [r4, #-3980]
.L574:
	.loc 3 1321 0
	ldr	r3, .L579
	ldr	r2, .L579+16
	mov	r4, r3
	ldrh	r2, [r3, r2]
	cmp	r2, #0
	beq	.L575
	.loc 3 1322 0
	bl	sftl_recover_active_sblk
	b	.L576
.L575:
	.loc 3 1323 0
	ldr	r2, .L579+4
	movw	r3, #1730
	ldrh	r2, [r2, r3]
	movw	r3, #65535
	cmp	r2, r3
	beq	.L576
	.loc 3 1324 0
	bl	sftl_recover_prev_sblk
.L576:
	.loc 3 1326 0
	ldrb	r3, [r4, #-3980]	@ zero_extendqisi2
	cmp	r3, #1
	bne	.L577
	.loc 3 1327 0
	bl	sftl_l2p_flush
	.loc 3 1328 0
	bl	sftl_sys_info_flush
	.loc 3 1329 0
	mov	r0, #3
	bl	sftl_user_gc
.L577:
	.loc 3 1333 0
	ldr	r3, .L579
	ldrb	r0, [r3, #-3960]	@ zero_extendqisi2
.L573:
	.loc 3 1334 0
	ldmfd	sp!, {r2, r3, r4, r5, r6, pc}
.L580:
	.align	2
.L579:
	.word	.LANCHOR1
	.word	.LANCHOR0
	.word	-3982
	.word	-3952
	.word	-3984
	.word	.LC7
	.word	.LC5
.LFE175:
	.fnend
	.size	sftl_recovery, .-sftl_recovery
	.align	2
	.global	sftl_deinit
	.type	sftl_deinit, %function
sftl_deinit:
	.fnstart
.LFB131:
	.loc 1 215 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
.LCFI62:
	.loc 1 218 0
	mov	r0, #1
	.loc 1 217 0
	ldr	r3, .L582
	ldr	r2, [r3, #2084]
	orr	r2, r2, #1
	str	r2, [r3, #2084]
	.loc 1 218 0
	ldr	r3, .L582+4
	ldr	r1, [r3, #-3932]
	ldr	r2, [r3, #-3928]
	orr	r2, r1, r0, asl r2
	str	r2, [r3, #-3932]
	.loc 1 219 0
	bl	sftl_l2p_flush
	.loc 1 220 0
	bl	sftl_sys_info_flush
	.loc 1 222 0
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L583:
	.align	2
.L582:
	.word	.LANCHOR0
	.word	.LANCHOR1
.LFE131:
	.fnend
	.size	sftl_deinit, .-sftl_deinit
	.align	2
	.global	sftl_sys_init
	.type	sftl_sys_init, %function
sftl_sys_init:
	.fnstart
.LFB129:
	.loc 1 172 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, lr}
	.save {r4, lr}
.LCFI63:
	.loc 1 174 0
	bl	sftl_sys_info_load
	cmp	r0, #0
	.loc 1 175 0
	mvnne	r0, #0
	.loc 1 174 0
	ldmnefd	sp!, {r4, pc}
	.loc 1 176 0
	bl	sftl_l2p_load
	subs	r4, r0, #0
	bne	.L587
	.loc 1 178 0
	bl	sftl_recovery
	cmp	r0, #0
	ldmeqfd	sp!, {r4, pc}
	.loc 1 179 0
	bl	sftl_l2p_flush
	.loc 1 181 0
	bl	sftl_sys_info_flush
	.loc 1 184 0
	mov	r0, r4
	ldmfd	sp!, {r4, pc}
.L587:
	.loc 1 177 0
	mvn	r0, #0
	.loc 1 185 0
	ldmfd	sp!, {r4, pc}
.LFE129:
	.fnend
	.size	sftl_sys_init, .-sftl_sys_init
	.align	2
	.global	sftl_low_format
	.type	sftl_low_format, %function
sftl_low_format:
	.fnstart
.LFB128:
	.loc 1 126 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r4, r5, r6, r7, r8, lr}
	.save {r4, r5, r6, r7, r8, lr}
.LCFI64:
	.loc 1 130 0
	bl	sftl_bbt_load
	cmp	r0, #0
	beq	.L590
	.loc 1 131 0
	bl	sftl_bbt_make
.L590:
	.loc 1 132 0
	bl	sftl_sys_blk_init
	.loc 1 133 0
	ldr	r3, .L599
	mov	r2, #0
	.loc 1 135 0
	ldr	r1, .L599+4
	.loc 1 137 0
	ldr	r5, .L599+8
	.loc 1 133 0
	str	r2, [r3, #-3932]
	.loc 1 134 0
	str	r2, [r3, #-3928]
	.loc 1 135 0
	ldr	r2, [r3, #-3924]
	str	r1, [r2, #0]
	.loc 1 136 0
	ldr	r2, .L599+12
	ldr	r3, [r3, #-3916]
	str	r2, [r3, #0]
	.loc 1 137 0
	ldrh	r4, [r5, #4]
	b	.L591
.L592:
	.loc 1 138 0
	uxth	r0, r4
	.loc 1 137 0 discriminator 2
	add	r4, r4, #1
	.loc 1 138 0
	bl	sftl_format_erase_sblk
.L591:
	.loc 1 137 0 discriminator 1
	ldrh	r3, [r5, #18]
	ldr	r1, .L599+8
	cmp	r4, r3
	blt	.L592
	.loc 1 140 0
	ldr	r2, .L599
	ldr	r3, .L599+16
	.loc 1 148 0
	ldrh	r7, [r1, #14]
	.loc 1 143 0 discriminator 1
	ldrh	r6, [r1, #4]
	.loc 1 140 0
	ldrh	r0, [r2, r3]
	ldr	r3, .L599+20
	.loc 1 144 0
	ldrh	r4, [r1, #16]
	.loc 1 140 0
	strh	r0, [r2, r3]	@ movhi
	.loc 1 141 0
	mov	r3, #0
	ldr	r0, .L599+24
	.loc 1 143 0
	mov	r1, r3
	.loc 1 141 0
	strh	r3, [r2, r0]	@ movhi
	.loc 1 142 0
	add	r0, r0, #2
	strh	r3, [r2, r0]	@ movhi
	.loc 1 148 0
	mov	r0, #1
	mov	r7, r0, asl r7
	.loc 1 147 0
	mov	r8, r0
	.loc 1 148 0
	sub	r7, r7, #1
	.loc 1 145 0
	mov	r0, #3
	.loc 1 143 0
	b	.L593
.L597:
	.loc 1 145 0
	ldr	ip, [r2, #-3908]
	.loc 1 144 0
	cmp	r4, #2
	.loc 1 143 0
	add	r1, r1, #1
	.loc 1 145 0
	ldr	lr, [ip, r3]
	bfieq	lr, r0, #28, #4
	.loc 1 147 0
	bfine	lr, r8, #28, #4
	str	lr, [ip, r3]
	.loc 1 148 0
	ldr	ip, [r2, #-3908]
	ldr	lr, [ip, r3]
	cmp	r7, lr, lsr #28
	.loc 1 149 0
	bfieq	lr, r0, #25, #3
	streq	lr, [ip, r3]
	.loc 1 143 0
	add	r3, r3, #4
.L593:
	.loc 1 143 0 is_stmt 0 discriminator 1
	cmp	r1, r6
	ldr	r5, .L599
	blt	.L597
.LBB326:
	.loc 1 153 0 is_stmt 1
	ldr	r4, .L599+8
.LBE326:
	.loc 1 152 0
	mov	r6, #1
	ldr	r2, [r5, #-3928]
.LBB327:
	.loc 1 153 0 discriminator 3
	mov	r1, #2064
.LBE327:
	.loc 1 152 0
	ldr	r3, [r5, #-3932]
.LBB328:
	.loc 1 153 0 discriminator 3
	add	r0, r4, #48
.LBE328:
	.loc 1 159 0
	mvn	r7, #0
	.loc 1 152 0
	orr	r3, r3, r6, asl r2
	str	r3, [r5, #-3932]
.LBB329:
	.loc 1 153 0 discriminator 3
	bl	__memzero
.LBE329:
	.loc 1 154 0
	ldrh	r2, [r4, #4]
	movw	r3, #2062
.LBB330:
	.loc 1 158 0 discriminator 4
	mov	r1, #255
.LBE330:
	.loc 1 154 0
	sub	r2, r2, #1
	strh	r2, [r4, r3]	@ movhi
	.loc 1 156 0
	ldr	r3, .L599+28
.LBB331:
	.loc 1 158 0 discriminator 4
	mov	r2, #128
.LBE331:
	.loc 1 155 0
	str	r6, [r4, #2064]
.LBB332:
	.loc 1 158 0 discriminator 4
	add	r0, r4, #1600
.LBE332:
	.loc 1 156 0
	str	r3, [r4, #2092]
.LBB333:
	.loc 1 158 0 discriminator 4
	bl	memset
.LBE333:
	.loc 1 159 0
	ldr	r3, .L599+32
	.loc 1 160 0
	mov	r0, r6
	.loc 1 159 0
	strh	r7, [r5, r3]	@ movhi
	.loc 1 160 0
	bl	sftl_open_sblk
	.loc 1 161 0
	mov	r3, #1728
	strh	r7, [r4, r3]	@ movhi
	.loc 1 162 0
	add	r3, r3, #2
	.loc 1 164 0
	mov	r0, #2
	.loc 1 162 0
	strh	r7, [r4, r3]	@ movhi
	.loc 1 163 0
	add	r3, r3, #2
	strh	r7, [r4, r3]	@ movhi
	.loc 1 164 0
	bl	sftl_open_sblk
	.loc 1 166 0
	bl	sftl_sys_info_flush
	.loc 1 169 0
	mov	r0, #0
	ldmfd	sp!, {r4, r5, r6, r7, r8, pc}
.L600:
	.align	2
.L599:
	.word	.LANCHOR1
	.word	1437269760
	.word	.LANCHOR0
	.word	1520828415
	.word	-3864
	.word	-2922
	.word	-2920
	.word	1398362953
	.word	-4008
.LFE128:
	.fnend
	.size	sftl_low_format, .-sftl_low_format
	.align	2
	.global	ftl_low_format
	.type	ftl_low_format, %function
ftl_low_format:
	.fnstart
.LFB135:
	.loc 1 393 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
.LCFI65:
	.loc 1 394 0 discriminator 1
	ldr	r0, .L602
	bl	printk
	.loc 1 395 0
	bl	sftl_var_init
	.loc 1 396 0
	bl	sftl_mem_init
	.loc 1 398 0
	bl	sftl_low_format
	.loc 1 400 0
	mov	r0, #0
	ldmfd	sp!, {r3, pc}
.L603:
	.align	2
.L602:
	.word	.LC18
.LFE135:
	.fnend
	.size	ftl_low_format, .-ftl_low_format
	.align	2
	.global	sftl_init
	.type	sftl_init, %function
sftl_init:
	.fnstart
.LFB130:
	.loc 1 188 0
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	stmfd	sp!, {r3, lr}
	.save {r3, lr}
.LCFI66:
	.loc 1 190 0
	bl	sftl_var_init
	.loc 1 191 0
	bl	sftl_mem_init
	.loc 1 193 0 discriminator 1
	ldr	r0, .L611
	ldr	r1, .L611+4
	bl	printk
.L609:
	.loc 1 195 0
	bl	sftl_bbt_load
	cmp	r0, #0
	bne	.L610
.L606:
	.loc 1 203 0
	bl	sftl_sys_init
	cmp	r0, #0
	ldmeqfd	sp!, {r3, pc}
.L610:
	.loc 1 205 0
	bl	sftl_low_format
	.loc 1 206 0
	b	.L609
.L612:
	.align	2
.L611:
	.word	.LC19
	.word	.LC20
.LFE130:
	.fnend
	.size	sftl_init, .-sftl_init
	.global	user_gc
	.global	g_user_power_lost_rewrite
	.global	g_l2p_power_lost_rewrite
	.global	g_sys_power_lost_rewrite
	.global	g_suddenly_power_lost
	.global	ftl_prog_cnt
	.global	ftl_read_cnt
	.global	trace_spare_buf
	.global	trace_req
	.global	p_user_cur_trace_list
	.global	p_user_trace_list
	.global	user_info
	.global	g_max_lpn
	.global	g_max_lba_sector
	.global	g_sys_page_last_dirty_index
	.global	g_sys_page_write_bitmap
	.global	g_sys_cur_page
	.global	g_sys_cur_blk_index
	.global	g_sys_cur_blk
	.global	g_sys_blk_num
	.global	g_sys_blk_list
	.global	c_plane_base_pblk
	.global	c_ftl_sblk_start
	.global	c_page_per_sblk
	.global	c_nand_page_per_blk
	.global	c_nand_page_num
	.global	c_nand_blk_num
	.global	c_nand_blk_per_plane
	.global	c_nand_plane_per_die
	.global	c_nand_plane_num
	.global	c_nand_sec_per_page
	.global	c_nand_page_size
	.global	g_sys_blk_start
	.global	g_ftl_rev_blk_num
	.global	p_l2p_spare_buf
	.global	p_l2p_data_buf
	.global	p_l2p_cur_trace_list
	.global	p_l2p_trace_list
	.global	p_l2p_cache_buf
	.global	p_l2p_cache_info
	.global	p_l2p_index_ppn
	.global	c_l2p_cache_num
	.global	c_l2p_index_num
	.global	c_l2p_trigger_offs
	.global	c_l2p_sblk_num
	.global	l2p_info
	.global	req_erase
	.global	req_sys
	.global	p_io_spare_buf
	.global	p_io_data_buf
	.global	p_sys_spare_buf
	.global	p_sys_data_buf
	.global	p_sblk_info_buf
	.global	bbt_info
	.global	sys_info
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"bbt\000"
.LC1:
	.ascii	"sys_blk_list\000"
.LC2:
	.ascii	"sys_page\000"
.LC3:
	.ascii	"sblk_info\000"
.LC4:
	.ascii	"l2p_sblk_list\000"
.LC5:
	.ascii	"\0016FILE: %s:%d:\012\000"
.LC6:
	.ascii	"drivers/rksftl/sftl_bbm.c\000"
.LC7:
	.ascii	"drivers/rksftl/sftl_sys.c\000"
.LC8:
	.ascii	"\0016l2p_sblk[%d].real_vpn=%d\012\000"
.LC9:
	.ascii	"\0016GC: read l2p_page err: status=0x%x, expect_lpa"
	.ascii	"=0x%x, readout_lpa=0x%x\012\000"
.LC10:
	.ascii	"data\000"
.LC11:
	.ascii	"spare\000"
.LC12:
	.ascii	"\0016read l2p_page err: status=0x%x, expect_index=0"
	.ascii	"x%x, readout_index=0x%x\012\000"
.LC13:
	.ascii	"\0016GC: read user_page err: status=0x%x, expect_lp"
	.ascii	"a=0x%x, readout_lpa=0x%x\012\000"
.LC14:
	.ascii	"\0016wait_gc_sblk[%d].real_vpn=%d\012\000"
.LC15:
	.ascii	"\0016read pg err: status=0x%x, lpa:expect=0x%x,read"
	.ascii	"out=0x%x, ppa=0x%x, sblk=%d\012\000"
.LC16:
	.ascii	"drivers/rksftl/sftl.c\000"
.LC17:
	.ascii	"\0016read pg err: status=0x%x, lpa:expect=0x%x,read"
	.ascii	"out=0x%x, ppa=0x%x, sblk=0x%x\012\000"
.LC18:
	.ascii	"\0016ftl_low_format\012\000"
.LC19:
	.ascii	"\0016%s\012\000"
.LC20:
	.ascii	"ftl_ver 1.2.2\000"
	.bss
	.align	2
	.set	.LANCHOR0,. + 0
	.set	.LANCHOR1,. + 8184
	.type	g_sys_blk_start, %object
	.size	g_sys_blk_start, 2
g_sys_blk_start:
	.space	2
	.type	g_ftl_rev_blk_num, %object
	.size	g_ftl_rev_blk_num, 2
g_ftl_rev_blk_num:
	.space	2
	.type	c_ftl_sblk_start, %object
	.size	c_ftl_sblk_start, 2
c_ftl_sblk_start:
	.space	2
	.space	2
	.type	c_nand_page_size, %object
	.size	c_nand_page_size, 4
c_nand_page_size:
	.space	4
	.type	c_nand_sec_per_page, %object
	.size	c_nand_sec_per_page, 2
c_nand_sec_per_page:
	.space	2
	.type	c_nand_plane_num, %object
	.size	c_nand_plane_num, 2
c_nand_plane_num:
	.space	2
	.type	c_nand_plane_per_die, %object
	.size	c_nand_plane_per_die, 2
c_nand_plane_per_die:
	.space	2
	.type	c_nand_blk_per_plane, %object
	.size	c_nand_blk_per_plane, 2
c_nand_blk_per_plane:
	.space	2
	.type	c_nand_page_per_blk, %object
	.size	c_nand_page_per_blk, 2
c_nand_page_per_blk:
	.space	2
	.type	c_page_per_sblk, %object
	.size	c_page_per_sblk, 2
c_page_per_sblk:
	.space	2
	.type	c_nand_blk_num, %object
	.size	c_nand_blk_num, 2
c_nand_blk_num:
	.space	2
	.space	2
	.type	c_nand_page_num, %object
	.size	c_nand_page_num, 4
c_nand_page_num:
	.space	4
	.type	c_l2p_index_num, %object
	.size	c_l2p_index_num, 2
c_l2p_index_num:
	.space	2
	.type	c_l2p_cache_num, %object
	.size	c_l2p_cache_num, 2
c_l2p_cache_num:
	.space	2
	.type	c_l2p_sblk_num, %object
	.size	c_l2p_sblk_num, 2
c_l2p_sblk_num:
	.space	2
	.type	c_l2p_trigger_offs, %object
	.size	c_l2p_trigger_offs, 2
c_l2p_trigger_offs:
	.space	2
	.type	g_max_lba_sector, %object
	.size	g_max_lba_sector, 4
g_max_lba_sector:
	.space	4
	.type	g_max_lpn, %object
	.size	g_max_lpn, 4
g_max_lpn:
	.space	4
	.type	sys_info, %object
	.size	sys_info, 2064
sys_info:
	.space	2064
	.type	bbt_info, %object
	.size	bbt_info, 2064
bbt_info:
	.space	2064
	.type	l2p_info, %object
	.size	l2p_info, 20
l2p_info:
	.space	20
	.type	user_info, %object
	.size	user_info, 12
user_info:
	.space	12
	.type	c_plane_base_pblk, %object
	.size	c_plane_base_pblk, 8
c_plane_base_pblk:
	.space	8
	.type	ftl_read_cnt, %object
	.size	ftl_read_cnt, 4
ftl_read_cnt:
	.space	4
	.type	ftl_prog_cnt, %object
	.size	ftl_prog_cnt, 4
ftl_prog_cnt:
	.space	4
	.type	g_suddenly_power_lost, %object
	.size	g_suddenly_power_lost, 1
g_suddenly_power_lost:
	.space	1
	.type	g_sys_power_lost_rewrite, %object
	.size	g_sys_power_lost_rewrite, 1
g_sys_power_lost_rewrite:
	.space	1
	.type	g_l2p_power_lost_rewrite, %object
	.size	g_l2p_power_lost_rewrite, 1
g_l2p_power_lost_rewrite:
	.space	1
	.type	g_user_power_lost_rewrite, %object
	.size	g_user_power_lost_rewrite, 1
g_user_power_lost_rewrite:
	.space	1
	.type	user_gc, %object
	.size	user_gc, 24
user_gc:
	.space	24
	.type	g_sys_page_write_bitmap, %object
	.size	g_sys_page_write_bitmap, 4
g_sys_page_write_bitmap:
	.space	4
	.type	g_sys_page_last_dirty_index, %object
	.size	g_sys_page_last_dirty_index, 4
g_sys_page_last_dirty_index:
	.space	4
	.type	p_sys_data_buf, %object
	.size	p_sys_data_buf, 4
p_sys_data_buf:
	.space	4
	.type	p_io_data_buf, %object
	.size	p_io_data_buf, 4
p_io_data_buf:
	.space	4
	.type	p_sys_spare_buf, %object
	.size	p_sys_spare_buf, 4
p_sys_spare_buf:
	.space	4
	.type	p_io_spare_buf, %object
	.size	p_io_spare_buf, 4
p_io_spare_buf:
	.space	4
	.type	p_sblk_info_buf, %object
	.size	p_sblk_info_buf, 4
p_sblk_info_buf:
	.space	4
	.type	req_erase, %object
	.size	req_erase, 4
req_erase:
	.space	4
	.type	p_l2p_index_ppn, %object
	.size	p_l2p_index_ppn, 4
p_l2p_index_ppn:
	.space	4
	.type	p_l2p_cache_buf, %object
	.size	p_l2p_cache_buf, 4
p_l2p_cache_buf:
	.space	4
	.type	p_l2p_cache_info, %object
	.size	p_l2p_cache_info, 4
p_l2p_cache_info:
	.space	4
	.type	p_l2p_data_buf, %object
	.size	p_l2p_data_buf, 4
p_l2p_data_buf:
	.space	4
	.type	p_l2p_spare_buf, %object
	.size	p_l2p_spare_buf, 4
p_l2p_spare_buf:
	.space	4
	.type	p_l2p_trace_list, %object
	.size	p_l2p_trace_list, 4
p_l2p_trace_list:
	.space	4
	.type	p_l2p_cur_trace_list, %object
	.size	p_l2p_cur_trace_list, 4
p_l2p_cur_trace_list:
	.space	4
	.type	p_user_trace_list, %object
	.size	p_user_trace_list, 4
p_user_trace_list:
	.space	4
	.type	p_user_cur_trace_list, %object
	.size	p_user_cur_trace_list, 4
p_user_cur_trace_list:
	.space	4
	.type	g_sys_blk_list, %object
	.size	g_sys_blk_list, 24
g_sys_blk_list:
	.space	24
	.type	req_sys, %object
	.size	req_sys, 20
req_sys:
	.space	20
	.type	trace_req, %object
	.size	trace_req, 640
trace_req:
	.space	640
	.type	trace_spare_buf, %object
	.size	trace_spare_buf, 256
trace_spare_buf:
	.space	256
	.type	g_sys_blk_num, %object
	.size	g_sys_blk_num, 2
g_sys_blk_num:
	.space	2
	.type	g_sys_cur_blk, %object
	.size	g_sys_cur_blk, 2
g_sys_cur_blk:
	.space	2
	.type	g_sys_cur_blk_index, %object
	.size	g_sys_cur_blk_index, 2
g_sys_cur_blk_index:
	.space	2
	.type	g_sys_cur_page, %object
	.size	g_sys_cur_page, 2
g_sys_cur_page:
	.space	2
	.section	.debug_frame,"",%progbits
.Lframe0:
	.4byte	.LECIE0-.LSCIE0
.LSCIE0:
	.4byte	0xffffffff
	.byte	0x1
	.ascii	"\000"
	.uleb128 0x1
	.sleb128 -4
	.byte	0xe
	.byte	0xc
	.uleb128 0xd
	.uleb128 0
	.align	2
.LECIE0:
.LSFDE0:
	.4byte	.LEFDE0-.LASFDE0
.LASFDE0:
	.4byte	.Lframe0
	.4byte	.LFB124
	.4byte	.LFE124-.LFB124
	.byte	0x4
	.4byte	.LCFI0-.LFB124
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x88
	.uleb128 0x2
	.byte	0x87
	.uleb128 0x3
	.byte	0x86
	.uleb128 0x4
	.byte	0x85
	.uleb128 0x5
	.byte	0x84
	.uleb128 0x6
	.align	2
.LEFDE0:
.LSFDE2:
	.4byte	.LEFDE2-.LASFDE2
.LASFDE2:
	.4byte	.Lframe0
	.4byte	.LFB125
	.4byte	.LFE125-.LFB125
	.byte	0x4
	.4byte	.LCFI1-.LFB125
	.byte	0xe
	.uleb128 0x10
	.byte	0x8e
	.uleb128 0x1
	.byte	0x85
	.uleb128 0x2
	.byte	0x84
	.uleb128 0x3
	.byte	0x83
	.uleb128 0x4
	.align	2
.LEFDE2:
.LSFDE4:
	.4byte	.LEFDE4-.LASFDE4
.LASFDE4:
	.4byte	.Lframe0
	.4byte	.LFB126
	.4byte	.LFE126-.LFB126
	.byte	0x4
	.4byte	.LCFI2-.LFB126
	.byte	0xe
	.uleb128 0x10
	.byte	0x8e
	.uleb128 0x1
	.byte	0x85
	.uleb128 0x2
	.byte	0x84
	.uleb128 0x3
	.byte	0x83
	.uleb128 0x4
	.align	2
.LEFDE4:
.LSFDE6:
	.4byte	.LEFDE6-.LASFDE6
.LASFDE6:
	.4byte	.Lframe0
	.4byte	.LFB127
	.4byte	.LFE127-.LFB127
	.align	2
.LEFDE6:
.LSFDE8:
	.4byte	.LEFDE8-.LASFDE8
.LASFDE8:
	.4byte	.Lframe0
	.4byte	.LFB134
	.4byte	.LFE134-.LFB134
	.align	2
.LEFDE8:
.LSFDE10:
	.4byte	.LEFDE10-.LASFDE10
.LASFDE10:
	.4byte	.Lframe0
	.4byte	.LFB136
	.4byte	.LFE136-.LFB136
	.align	2
.LEFDE10:
.LSFDE12:
	.4byte	.LEFDE12-.LASFDE12
.LASFDE12:
	.4byte	.Lframe0
	.4byte	.LFB137
	.4byte	.LFE137-.LFB137
	.byte	0x4
	.4byte	.LCFI3-.LFB137
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x84
	.uleb128 0x2
	.align	2
.LEFDE12:
.LSFDE14:
	.4byte	.LEFDE14-.LASFDE14
.LASFDE14:
	.4byte	.Lframe0
	.4byte	.LFB138
	.4byte	.LFE138-.LFB138
	.align	2
.LEFDE14:
.LSFDE16:
	.4byte	.LEFDE16-.LASFDE16
.LASFDE16:
	.4byte	.Lframe0
	.4byte	.LFB139
	.4byte	.LFE139-.LFB139
	.align	2
.LEFDE16:
.LSFDE18:
	.4byte	.LEFDE18-.LASFDE18
.LASFDE18:
	.4byte	.Lframe0
	.4byte	.LFB140
	.4byte	.LFE140-.LFB140
	.byte	0x4
	.4byte	.LCFI4-.LFB140
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x83
	.uleb128 0x2
	.align	2
.LEFDE18:
.LSFDE20:
	.4byte	.LEFDE20-.LASFDE20
.LASFDE20:
	.4byte	.Lframe0
	.4byte	.LFB142
	.4byte	.LFE142-.LFB142
	.align	2
.LEFDE20:
.LSFDE22:
	.4byte	.LEFDE22-.LASFDE22
.LASFDE22:
	.4byte	.Lframe0
	.4byte	.LFB143
	.4byte	.LFE143-.LFB143
	.align	2
.LEFDE22:
.LSFDE24:
	.4byte	.LEFDE24-.LASFDE24
.LASFDE24:
	.4byte	.Lframe0
	.4byte	.LFB145
	.4byte	.LFE145-.LFB145
	.byte	0x4
	.4byte	.LCFI5-.LFB145
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x87
	.uleb128 0x2
	.byte	0x86
	.uleb128 0x3
	.byte	0x85
	.uleb128 0x4
	.byte	0x84
	.uleb128 0x5
	.byte	0x83
	.uleb128 0x6
	.align	2
.LEFDE24:
.LSFDE26:
	.4byte	.LEFDE26-.LASFDE26
.LASFDE26:
	.4byte	.Lframe0
	.4byte	.LFB147
	.4byte	.LFE147-.LFB147
	.byte	0x4
	.4byte	.LCFI6-.LFB147
	.byte	0xe
	.uleb128 0x28
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x83
	.uleb128 0xa
	.align	2
.LEFDE26:
.LSFDE28:
	.4byte	.LEFDE28-.LASFDE28
.LASFDE28:
	.4byte	.Lframe0
	.4byte	.LFB146
	.4byte	.LFE146-.LFB146
	.byte	0x4
	.4byte	.LCFI7-.LFB146
	.byte	0xe
	.uleb128 0x1c
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x88
	.uleb128 0x3
	.byte	0x87
	.uleb128 0x4
	.byte	0x86
	.uleb128 0x5
	.byte	0x85
	.uleb128 0x6
	.byte	0x84
	.uleb128 0x7
	.byte	0x4
	.4byte	.LCFI8-.LCFI7
	.byte	0xe
	.uleb128 0x48
	.align	2
.LEFDE28:
.LSFDE30:
	.4byte	.LEFDE30-.LASFDE30
.LASFDE30:
	.4byte	.Lframe0
	.4byte	.LFB148
	.4byte	.LFE148-.LFB148
	.align	2
.LEFDE30:
.LSFDE32:
	.4byte	.LEFDE32-.LASFDE32
.LASFDE32:
	.4byte	.Lframe0
	.4byte	.LFB149
	.4byte	.LFE149-.LFB149
	.byte	0x4
	.4byte	.LCFI9-.LFB149
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x84
	.uleb128 0x2
	.align	2
.LEFDE32:
.LSFDE34:
	.4byte	.LEFDE34-.LASFDE34
.LASFDE34:
	.4byte	.Lframe0
	.4byte	.LFB150
	.4byte	.LFE150-.LFB150
	.byte	0x4
	.4byte	.LCFI10-.LFB150
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x83
	.uleb128 0x2
	.align	2
.LEFDE34:
.LSFDE36:
	.4byte	.LEFDE36-.LASFDE36
.LASFDE36:
	.4byte	.Lframe0
	.4byte	.LFB151
	.4byte	.LFE151-.LFB151
	.byte	0x4
	.4byte	.LCFI11-.LFB151
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x87
	.uleb128 0x2
	.byte	0x86
	.uleb128 0x3
	.byte	0x85
	.uleb128 0x4
	.byte	0x84
	.uleb128 0x5
	.byte	0x83
	.uleb128 0x6
	.align	2
.LEFDE36:
.LSFDE38:
	.4byte	.LEFDE38-.LASFDE38
.LASFDE38:
	.4byte	.Lframe0
	.4byte	.LFB152
	.4byte	.LFE152-.LFB152
	.byte	0x4
	.4byte	.LCFI12-.LFB152
	.byte	0xe
	.uleb128 0x10
	.byte	0x8e
	.uleb128 0x1
	.byte	0x86
	.uleb128 0x2
	.byte	0x85
	.uleb128 0x3
	.byte	0x84
	.uleb128 0x4
	.align	2
.LEFDE38:
.LSFDE40:
	.4byte	.LEFDE40-.LASFDE40
.LASFDE40:
	.4byte	.Lframe0
	.4byte	.LFB153
	.4byte	.LFE153-.LFB153
	.byte	0x4
	.4byte	.LCFI13-.LFB153
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x88
	.uleb128 0x2
	.byte	0x87
	.uleb128 0x3
	.byte	0x86
	.uleb128 0x4
	.byte	0x85
	.uleb128 0x5
	.byte	0x84
	.uleb128 0x6
	.align	2
.LEFDE40:
.LSFDE42:
	.4byte	.LEFDE42-.LASFDE42
.LASFDE42:
	.4byte	.Lframe0
	.4byte	.LFB154
	.4byte	.LFE154-.LFB154
	.byte	0x4
	.4byte	.LCFI14-.LFB154
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI15-.LCFI14
	.byte	0xe
	.uleb128 0x48
	.align	2
.LEFDE42:
.LSFDE44:
	.4byte	.LEFDE44-.LASFDE44
.LASFDE44:
	.4byte	.Lframe0
	.4byte	.LFB155
	.4byte	.LFE155-.LFB155
	.byte	0x4
	.4byte	.LCFI16-.LFB155
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x89
	.uleb128 0x3
	.byte	0x88
	.uleb128 0x4
	.byte	0x87
	.uleb128 0x5
	.byte	0x86
	.uleb128 0x6
	.byte	0x85
	.uleb128 0x7
	.byte	0x84
	.uleb128 0x8
	.byte	0x4
	.4byte	.LCFI17-.LCFI16
	.byte	0xe
	.uleb128 0x38
	.align	2
.LEFDE44:
.LSFDE46:
	.4byte	.LEFDE46-.LASFDE46
.LASFDE46:
	.4byte	.Lframe0
	.4byte	.LFB144
	.4byte	.LFE144-.LFB144
	.byte	0x4
	.4byte	.LCFI18-.LFB144
	.byte	0xe
	.uleb128 0x30
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x82
	.uleb128 0xa
	.byte	0x81
	.uleb128 0xb
	.byte	0x80
	.uleb128 0xc
	.align	2
.LEFDE46:
.LSFDE48:
	.4byte	.LEFDE48-.LASFDE48
.LASFDE48:
	.4byte	.Lframe0
	.4byte	.LFB157
	.4byte	.LFE157-.LFB157
	.byte	0x4
	.4byte	.LCFI19-.LFB157
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI20-.LCFI19
	.byte	0xe
	.uleb128 0x48
	.align	2
.LEFDE48:
.LSFDE50:
	.4byte	.LEFDE50-.LASFDE50
.LASFDE50:
	.4byte	.Lframe0
	.4byte	.LFB158
	.4byte	.LFE158-.LFB158
	.byte	0x4
	.4byte	.LCFI21-.LFB158
	.byte	0xe
	.uleb128 0x1c
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x88
	.uleb128 0x3
	.byte	0x87
	.uleb128 0x4
	.byte	0x86
	.uleb128 0x5
	.byte	0x85
	.uleb128 0x6
	.byte	0x84
	.uleb128 0x7
	.byte	0x4
	.4byte	.LCFI22-.LCFI21
	.byte	0xe
	.uleb128 0x38
	.align	2
.LEFDE50:
.LSFDE52:
	.4byte	.LEFDE52-.LASFDE52
.LASFDE52:
	.4byte	.Lframe0
	.4byte	.LFB159
	.4byte	.LFE159-.LFB159
	.byte	0x4
	.4byte	.LCFI23-.LFB159
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x89
	.uleb128 0x3
	.byte	0x88
	.uleb128 0x4
	.byte	0x87
	.uleb128 0x5
	.byte	0x86
	.uleb128 0x6
	.byte	0x85
	.uleb128 0x7
	.byte	0x84
	.uleb128 0x8
	.byte	0x4
	.4byte	.LCFI24-.LCFI23
	.byte	0xe
	.uleb128 0x38
	.align	2
.LEFDE52:
.LSFDE54:
	.4byte	.LEFDE54-.LASFDE54
.LASFDE54:
	.4byte	.Lframe0
	.4byte	.LFB162
	.4byte	.LFE162-.LFB162
	.byte	0x4
	.4byte	.LCFI25-.LFB162
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x88
	.uleb128 0x2
	.byte	0x87
	.uleb128 0x3
	.byte	0x86
	.uleb128 0x4
	.byte	0x85
	.uleb128 0x5
	.byte	0x84
	.uleb128 0x6
	.align	2
.LEFDE54:
.LSFDE56:
	.4byte	.LEFDE56-.LASFDE56
.LASFDE56:
	.4byte	.Lframe0
	.4byte	.LFB169
	.4byte	.LFE169-.LFB169
	.byte	0x4
	.4byte	.LCFI26-.LFB169
	.byte	0xe
	.uleb128 0xc
	.byte	0x8e
	.uleb128 0x1
	.byte	0x85
	.uleb128 0x2
	.byte	0x84
	.uleb128 0x3
	.align	2
.LEFDE56:
.LSFDE58:
	.4byte	.LEFDE58-.LASFDE58
.LASFDE58:
	.4byte	.Lframe0
	.4byte	.LFB170
	.4byte	.LFE170-.LFB170
	.byte	0x4
	.4byte	.LCFI27-.LFB170
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x89
	.uleb128 0x3
	.byte	0x88
	.uleb128 0x4
	.byte	0x87
	.uleb128 0x5
	.byte	0x86
	.uleb128 0x6
	.byte	0x85
	.uleb128 0x7
	.byte	0x84
	.uleb128 0x8
	.align	2
.LEFDE58:
.LSFDE60:
	.4byte	.LEFDE60-.LASFDE60
.LASFDE60:
	.4byte	.Lframe0
	.4byte	.LFB176
	.4byte	.LFE176-.LFB176
	.byte	0x4
	.4byte	.LCFI28-.LFB176
	.byte	0xe
	.uleb128 0x10
	.byte	0x8e
	.uleb128 0x1
	.byte	0x85
	.uleb128 0x2
	.byte	0x84
	.uleb128 0x3
	.byte	0x83
	.uleb128 0x4
	.align	2
.LEFDE60:
.LSFDE62:
	.4byte	.LEFDE62-.LASFDE62
.LASFDE62:
	.4byte	.Lframe0
	.4byte	.LFB177
	.4byte	.LFE177-.LFB177
	.byte	0x4
	.4byte	.LCFI29-.LFB177
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x84
	.uleb128 0x2
	.align	2
.LEFDE62:
.LSFDE64:
	.4byte	.LEFDE64-.LASFDE64
.LASFDE64:
	.4byte	.Lframe0
	.4byte	.LFB178
	.4byte	.LFE178-.LFB178
	.byte	0x4
	.4byte	.LCFI30-.LFB178
	.byte	0xe
	.uleb128 0x28
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x83
	.uleb128 0xa
	.align	2
.LEFDE64:
.LSFDE66:
	.4byte	.LEFDE66-.LASFDE66
.LASFDE66:
	.4byte	.Lframe0
	.4byte	.LFB179
	.4byte	.LFE179-.LFB179
	.byte	0x4
	.4byte	.LCFI31-.LFB179
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x87
	.uleb128 0x2
	.byte	0x86
	.uleb128 0x3
	.byte	0x85
	.uleb128 0x4
	.byte	0x84
	.uleb128 0x5
	.byte	0x83
	.uleb128 0x6
	.align	2
.LEFDE66:
.LSFDE68:
	.4byte	.LEFDE68-.LASFDE68
.LASFDE68:
	.4byte	.Lframe0
	.4byte	.LFB180
	.4byte	.LFE180-.LFB180
	.byte	0x4
	.4byte	.LCFI32-.LFB180
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x87
	.uleb128 0x2
	.byte	0x86
	.uleb128 0x3
	.byte	0x85
	.uleb128 0x4
	.byte	0x84
	.uleb128 0x5
	.byte	0x83
	.uleb128 0x6
	.align	2
.LEFDE68:
.LSFDE70:
	.4byte	.LEFDE70-.LASFDE70
.LASFDE70:
	.4byte	.Lframe0
	.4byte	.LFB181
	.4byte	.LFE181-.LFB181
	.byte	0x4
	.4byte	.LCFI33-.LFB181
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x89
	.uleb128 0x3
	.byte	0x88
	.uleb128 0x4
	.byte	0x87
	.uleb128 0x5
	.byte	0x86
	.uleb128 0x6
	.byte	0x85
	.uleb128 0x7
	.byte	0x84
	.uleb128 0x8
	.align	2
.LEFDE70:
.LSFDE72:
	.4byte	.LEFDE72-.LASFDE72
.LASFDE72:
	.4byte	.Lframe0
	.4byte	.LFB182
	.4byte	.LFE182-.LFB182
	.byte	0x4
	.4byte	.LCFI34-.LFB182
	.byte	0xe
	.uleb128 0x1c
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x88
	.uleb128 0x3
	.byte	0x87
	.uleb128 0x4
	.byte	0x86
	.uleb128 0x5
	.byte	0x85
	.uleb128 0x6
	.byte	0x84
	.uleb128 0x7
	.byte	0x4
	.4byte	.LCFI35-.LCFI34
	.byte	0xe
	.uleb128 0x38
	.align	2
.LEFDE72:
.LSFDE74:
	.4byte	.LEFDE74-.LASFDE74
.LASFDE74:
	.4byte	.Lframe0
	.4byte	.LFB183
	.4byte	.LFE183-.LFB183
	.byte	0x4
	.4byte	.LCFI36-.LFB183
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x89
	.uleb128 0x3
	.byte	0x88
	.uleb128 0x4
	.byte	0x87
	.uleb128 0x5
	.byte	0x86
	.uleb128 0x6
	.byte	0x85
	.uleb128 0x7
	.byte	0x84
	.uleb128 0x8
	.byte	0x4
	.4byte	.LCFI37-.LCFI36
	.byte	0xe
	.uleb128 0x38
	.align	2
.LEFDE74:
.LSFDE76:
	.4byte	.LEFDE76-.LASFDE76
.LASFDE76:
	.4byte	.Lframe0
	.4byte	.LFB156
	.4byte	.LFE156-.LFB156
	.byte	0x4
	.4byte	.LCFI38-.LFB156
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x88
	.uleb128 0x3
	.byte	0x87
	.uleb128 0x4
	.byte	0x86
	.uleb128 0x5
	.byte	0x85
	.uleb128 0x6
	.byte	0x84
	.uleb128 0x7
	.byte	0x83
	.uleb128 0x8
	.align	2
.LEFDE76:
.LSFDE78:
	.4byte	.LEFDE78-.LASFDE78
.LASFDE78:
	.4byte	.Lframe0
	.4byte	.LFB161
	.4byte	.LFE161-.LFB161
	.byte	0x4
	.4byte	.LCFI39-.LFB161
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI40-.LCFI39
	.byte	0xe
	.uleb128 0x68
	.align	2
.LEFDE78:
.LSFDE80:
	.4byte	.LEFDE80-.LASFDE80
.LASFDE80:
	.4byte	.Lframe0
	.4byte	.LFB163
	.4byte	.LFE163-.LFB163
	.byte	0x4
	.4byte	.LCFI41-.LFB163
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x89
	.uleb128 0x3
	.byte	0x88
	.uleb128 0x4
	.byte	0x87
	.uleb128 0x5
	.byte	0x86
	.uleb128 0x6
	.byte	0x85
	.uleb128 0x7
	.byte	0x84
	.uleb128 0x8
	.byte	0x4
	.4byte	.LCFI42-.LCFI41
	.byte	0xe
	.uleb128 0x38
	.align	2
.LEFDE80:
.LSFDE82:
	.4byte	.LEFDE82-.LASFDE82
.LASFDE82:
	.4byte	.Lframe0
	.4byte	.LFB164
	.4byte	.LFE164-.LFB164
	.byte	0x4
	.4byte	.LCFI43-.LFB164
	.byte	0xe
	.uleb128 0x10
	.byte	0x8e
	.uleb128 0x1
	.byte	0x86
	.uleb128 0x2
	.byte	0x85
	.uleb128 0x3
	.byte	0x84
	.uleb128 0x4
	.align	2
.LEFDE82:
.LSFDE84:
	.4byte	.LEFDE84-.LASFDE84
.LASFDE84:
	.4byte	.Lframe0
	.4byte	.LFB165
	.4byte	.LFE165-.LFB165
	.byte	0x4
	.4byte	.LCFI44-.LFB165
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x88
	.uleb128 0x2
	.byte	0x87
	.uleb128 0x3
	.byte	0x86
	.uleb128 0x4
	.byte	0x85
	.uleb128 0x5
	.byte	0x84
	.uleb128 0x6
	.align	2
.LEFDE84:
.LSFDE86:
	.4byte	.LEFDE86-.LASFDE86
.LASFDE86:
	.4byte	.Lframe0
	.4byte	.LFB166
	.4byte	.LFE166-.LFB166
	.byte	0x4
	.4byte	.LCFI45-.LFB166
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x89
	.uleb128 0x3
	.byte	0x88
	.uleb128 0x4
	.byte	0x87
	.uleb128 0x5
	.byte	0x86
	.uleb128 0x6
	.byte	0x85
	.uleb128 0x7
	.byte	0x84
	.uleb128 0x8
	.byte	0x4
	.4byte	.LCFI46-.LCFI45
	.byte	0xe
	.uleb128 0x38
	.align	2
.LEFDE86:
.LSFDE88:
	.4byte	.LEFDE88-.LASFDE88
.LASFDE88:
	.4byte	.Lframe0
	.4byte	.LFB168
	.4byte	.LFE168-.LFB168
	.byte	0x4
	.4byte	.LCFI47-.LFB168
	.byte	0xe
	.uleb128 0x30
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x82
	.uleb128 0xa
	.byte	0x81
	.uleb128 0xb
	.byte	0x80
	.uleb128 0xc
	.align	2
.LEFDE88:
.LSFDE90:
	.4byte	.LEFDE90-.LASFDE90
.LASFDE90:
	.4byte	.Lframe0
	.4byte	.LFB160
	.4byte	.LFE160-.LFB160
	.byte	0x4
	.4byte	.LCFI48-.LFB160
	.byte	0xe
	.uleb128 0x20
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8a
	.uleb128 0x2
	.byte	0x88
	.uleb128 0x3
	.byte	0x87
	.uleb128 0x4
	.byte	0x86
	.uleb128 0x5
	.byte	0x85
	.uleb128 0x6
	.byte	0x84
	.uleb128 0x7
	.byte	0x83
	.uleb128 0x8
	.align	2
.LEFDE90:
.LSFDE92:
	.4byte	.LEFDE92-.LASFDE92
.LASFDE92:
	.4byte	.Lframe0
	.4byte	.LFB174
	.4byte	.LFE174-.LFB174
	.byte	0x4
	.4byte	.LCFI49-.LFB174
	.byte	0xe
	.uleb128 0x30
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x82
	.uleb128 0xa
	.byte	0x81
	.uleb128 0xb
	.byte	0x80
	.uleb128 0xc
	.align	2
.LEFDE92:
.LSFDE94:
	.4byte	.LEFDE94-.LASFDE94
.LASFDE94:
	.4byte	.Lframe0
	.4byte	.LFB173
	.4byte	.LFE173-.LFB173
	.byte	0x4
	.4byte	.LCFI50-.LFB173
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI51-.LCFI50
	.byte	0xe
	.uleb128 0x48
	.align	2
.LEFDE94:
.LSFDE96:
	.4byte	.LEFDE96-.LASFDE96
.LASFDE96:
	.4byte	.Lframe0
	.4byte	.LFB167
	.4byte	.LFE167-.LFB167
	.byte	0x4
	.4byte	.LCFI52-.LFB167
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI53-.LCFI52
	.byte	0xe
	.uleb128 0x58
	.align	2
.LEFDE96:
.LSFDE98:
	.4byte	.LEFDE98-.LASFDE98
.LASFDE98:
	.4byte	.Lframe0
	.4byte	.LFB171
	.4byte	.LFE171-.LFB171
	.byte	0x4
	.4byte	.LCFI54-.LFB171
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI55-.LCFI54
	.byte	0xe
	.uleb128 0x58
	.align	2
.LEFDE98:
.LSFDE100:
	.4byte	.LEFDE100-.LASFDE100
.LASFDE100:
	.4byte	.Lframe0
	.4byte	.LFB172
	.4byte	.LFE172-.LFB172
	.byte	0x4
	.4byte	.LCFI56-.LFB172
	.byte	0xe
	.uleb128 0x10
	.byte	0x8e
	.uleb128 0x1
	.byte	0x85
	.uleb128 0x2
	.byte	0x84
	.uleb128 0x3
	.byte	0x83
	.uleb128 0x4
	.align	2
.LEFDE100:
.LSFDE102:
	.4byte	.LEFDE102-.LASFDE102
.LASFDE102:
	.4byte	.Lframe0
	.4byte	.LFB141
	.4byte	.LFE141-.LFB141
	.align	2
.LEFDE102:
.LSFDE104:
	.4byte	.LEFDE104-.LASFDE104
.LASFDE104:
	.4byte	.Lframe0
	.4byte	.LFB132
	.4byte	.LFE132-.LFB132
	.byte	0x4
	.4byte	.LCFI57-.LFB132
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI58-.LCFI57
	.byte	0xe
	.uleb128 0x50
	.align	2
.LEFDE104:
.LSFDE106:
	.4byte	.LEFDE106-.LASFDE106
.LASFDE106:
	.4byte	.Lframe0
	.4byte	.LFB133
	.4byte	.LFE133-.LFB133
	.byte	0x4
	.4byte	.LCFI59-.LFB133
	.byte	0xe
	.uleb128 0x24
	.byte	0x8e
	.uleb128 0x1
	.byte	0x8b
	.uleb128 0x2
	.byte	0x8a
	.uleb128 0x3
	.byte	0x89
	.uleb128 0x4
	.byte	0x88
	.uleb128 0x5
	.byte	0x87
	.uleb128 0x6
	.byte	0x86
	.uleb128 0x7
	.byte	0x85
	.uleb128 0x8
	.byte	0x84
	.uleb128 0x9
	.byte	0x4
	.4byte	.LCFI60-.LCFI59
	.byte	0xe
	.uleb128 0x58
	.align	2
.LEFDE106:
.LSFDE108:
	.4byte	.LEFDE108-.LASFDE108
.LASFDE108:
	.4byte	.Lframe0
	.4byte	.LFB175
	.4byte	.LFE175-.LFB175
	.byte	0x4
	.4byte	.LCFI61-.LFB175
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x86
	.uleb128 0x2
	.byte	0x85
	.uleb128 0x3
	.byte	0x84
	.uleb128 0x4
	.byte	0x81
	.uleb128 0x5
	.byte	0x80
	.uleb128 0x6
	.align	2
.LEFDE108:
.LSFDE110:
	.4byte	.LEFDE110-.LASFDE110
.LASFDE110:
	.4byte	.Lframe0
	.4byte	.LFB131
	.4byte	.LFE131-.LFB131
	.byte	0x4
	.4byte	.LCFI62-.LFB131
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x83
	.uleb128 0x2
	.align	2
.LEFDE110:
.LSFDE112:
	.4byte	.LEFDE112-.LASFDE112
.LASFDE112:
	.4byte	.Lframe0
	.4byte	.LFB129
	.4byte	.LFE129-.LFB129
	.byte	0x4
	.4byte	.LCFI63-.LFB129
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x84
	.uleb128 0x2
	.align	2
.LEFDE112:
.LSFDE114:
	.4byte	.LEFDE114-.LASFDE114
.LASFDE114:
	.4byte	.Lframe0
	.4byte	.LFB128
	.4byte	.LFE128-.LFB128
	.byte	0x4
	.4byte	.LCFI64-.LFB128
	.byte	0xe
	.uleb128 0x18
	.byte	0x8e
	.uleb128 0x1
	.byte	0x88
	.uleb128 0x2
	.byte	0x87
	.uleb128 0x3
	.byte	0x86
	.uleb128 0x4
	.byte	0x85
	.uleb128 0x5
	.byte	0x84
	.uleb128 0x6
	.align	2
.LEFDE114:
.LSFDE116:
	.4byte	.LEFDE116-.LASFDE116
.LASFDE116:
	.4byte	.Lframe0
	.4byte	.LFB135
	.4byte	.LFE135-.LFB135
	.byte	0x4
	.4byte	.LCFI65-.LFB135
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x83
	.uleb128 0x2
	.align	2
.LEFDE116:
.LSFDE118:
	.4byte	.LEFDE118-.LASFDE118
.LASFDE118:
	.4byte	.Lframe0
	.4byte	.LFB130
	.4byte	.LFE130-.LFB130
	.byte	0x4
	.4byte	.LCFI66-.LFB130
	.byte	0xe
	.uleb128 0x8
	.byte	0x8e
	.uleb128 0x1
	.byte	0x83
	.uleb128 0x2
	.align	2
.LEFDE118:
	.text
.Letext0:
	.file 4 "include/asm-generic/int-ll64.h"
	.file 5 "/home/zjw/kernel/include/uapi/asm-generic/posix_types.h"
	.file 6 "include/linux/types.h"
	.file 7 "drivers/rksftl/flash_com.h"
	.file 8 "drivers/rksftl/sftl.h"
	.file 9 "/home/zjw/kernel/arch/arm/include/asm/hwcap.h"
	.file 10 "include/linux/printk.h"
	.file 11 "include/linux/kernel.h"
	.section	.debug_info,"",%progbits
.Ldebug_info0:
	.4byte	0x288d
	.2byte	0x2
	.4byte	.Ldebug_abbrev0
	.byte	0x4
	.uleb128 0x1
	.4byte	.LASF304
	.byte	0x1
	.4byte	.LASF305
	.4byte	.LASF306
	.4byte	.Ltext0
	.4byte	.Letext0
	.4byte	.Ldebug_line0
	.4byte	.Ldebug_pubnames0
	.4byte	.Ldebug_pubtypes0
	.uleb128 0x2
	.byte	0x4
	.uleb128 0x3
	.byte	0x4
	.byte	0x7
	.4byte	.LASF0
	.uleb128 0x3
	.byte	0x4
	.byte	0x7
	.4byte	.LASF1
	.uleb128 0x3
	.byte	0x1
	.byte	0x8
	.4byte	.LASF2
	.uleb128 0x3
	.byte	0x1
	.byte	0x6
	.4byte	.LASF3
	.uleb128 0x3
	.byte	0x1
	.byte	0x8
	.4byte	.LASF4
	.uleb128 0x3
	.byte	0x2
	.byte	0x5
	.4byte	.LASF5
	.uleb128 0x3
	.byte	0x2
	.byte	0x7
	.4byte	.LASF6
	.uleb128 0x4
	.byte	0x4
	.byte	0x5
	.ascii	"int\000"
	.uleb128 0x3
	.byte	0x8
	.byte	0x5
	.4byte	.LASF7
	.uleb128 0x3
	.byte	0x8
	.byte	0x7
	.4byte	.LASF8
	.uleb128 0x5
	.ascii	"u8\000"
	.byte	0x4
	.byte	0x10
	.4byte	0x4b
	.uleb128 0x5
	.ascii	"s16\000"
	.byte	0x4
	.byte	0x12
	.4byte	0x52
	.uleb128 0x5
	.ascii	"u16\000"
	.byte	0x4
	.byte	0x13
	.4byte	0x59
	.uleb128 0x5
	.ascii	"s32\000"
	.byte	0x4
	.byte	0x15
	.4byte	0x60
	.uleb128 0x5
	.ascii	"u32\000"
	.byte	0x4
	.byte	0x16
	.4byte	0x36
	.uleb128 0x3
	.byte	0x4
	.byte	0x5
	.4byte	.LASF9
	.uleb128 0x6
	.4byte	.LASF10
	.byte	0x5
	.byte	0x43
	.4byte	0x36
	.uleb128 0x3
	.byte	0x1
	.byte	0x2
	.4byte	.LASF11
	.uleb128 0x6
	.4byte	.LASF12
	.byte	0x6
	.byte	0x36
	.4byte	0xb2
	.uleb128 0x7
	.4byte	.LASF17
	.byte	0x14
	.byte	0x7
	.byte	0x27
	.4byte	0x122
	.uleb128 0x8
	.4byte	.LASF13
	.byte	0x7
	.byte	0x28
	.4byte	0xa0
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF14
	.byte	0x7
	.byte	0x29
	.4byte	0xa0
	.byte	0x2
	.byte	0x23
	.uleb128 0x4
	.uleb128 0x8
	.4byte	.LASF15
	.byte	0x7
	.byte	0x2a
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0x8
	.uleb128 0x8
	.4byte	.LASF16
	.byte	0x7
	.byte	0x2b
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0xc
	.uleb128 0x9
	.ascii	"lpa\000"
	.byte	0x7
	.byte	0x2c
	.4byte	0xa0
	.byte	0x2
	.byte	0x23
	.uleb128 0x10
	.byte	0
	.uleb128 0xa
	.byte	0x4
	.4byte	0xa0
	.uleb128 0x7
	.4byte	.LASF18
	.byte	0x18
	.byte	0x7
	.byte	0x2f
	.4byte	0x1dd
	.uleb128 0x8
	.4byte	.LASF19
	.byte	0x7
	.byte	0x30
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF20
	.byte	0x7
	.byte	0x31
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x2
	.uleb128 0x8
	.4byte	.LASF21
	.byte	0x7
	.byte	0x32
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x4
	.uleb128 0x8
	.4byte	.LASF22
	.byte	0x7
	.byte	0x33
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x6
	.uleb128 0x8
	.4byte	.LASF23
	.byte	0x7
	.byte	0x34
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x8
	.uleb128 0x8
	.4byte	.LASF24
	.byte	0x7
	.byte	0x35
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0xa
	.uleb128 0x8
	.4byte	.LASF25
	.byte	0x7
	.byte	0x36
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0xc
	.uleb128 0x8
	.4byte	.LASF26
	.byte	0x7
	.byte	0x37
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0xe
	.uleb128 0x8
	.4byte	.LASF27
	.byte	0x7
	.byte	0x38
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x10
	.uleb128 0x8
	.4byte	.LASF28
	.byte	0x7
	.byte	0x39
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x12
	.uleb128 0x8
	.4byte	.LASF29
	.byte	0x7
	.byte	0x3a
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x14
	.uleb128 0x8
	.4byte	.LASF30
	.byte	0x7
	.byte	0x3b
	.4byte	0x75
	.byte	0x2
	.byte	0x23
	.uleb128 0x16
	.byte	0
	.uleb128 0xa
	.byte	0x4
	.4byte	0x8a
	.uleb128 0xb
	.4byte	.LASF75
	.byte	0x4
	.byte	0x8
	.byte	0x35
	.4byte	0x208
	.uleb128 0xc
	.4byte	.LASF31
	.sleb128 0
	.uleb128 0xc
	.4byte	.LASF32
	.sleb128 1
	.uleb128 0xc
	.4byte	.LASF33
	.sleb128 2
	.uleb128 0xc
	.4byte	.LASF34
	.sleb128 3
	.byte	0
	.uleb128 0x7
	.4byte	.LASF35
	.byte	0x4
	.byte	0x8
	.byte	0x3c
	.4byte	0x259
	.uleb128 0xd
	.4byte	.LASF36
	.byte	0x8
	.byte	0x3d
	.4byte	0xa0
	.byte	0x4
	.byte	0x10
	.byte	0x10
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0xe
	.ascii	"vpn\000"
	.byte	0x8
	.byte	0x3e
	.4byte	0xa0
	.byte	0x4
	.byte	0x9
	.byte	0x7
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0xd
	.4byte	.LASF37
	.byte	0x8
	.byte	0x3f
	.4byte	0xa0
	.byte	0x4
	.byte	0x3
	.byte	0x4
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0xd
	.4byte	.LASF38
	.byte	0x8
	.byte	0x40
	.4byte	0xa0
	.byte	0x4
	.byte	0x4
	.byte	0
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.byte	0
	.uleb128 0xf
	.4byte	.LASF39
	.2byte	0x810
	.byte	0x8
	.byte	0x43
	.4byte	0x392
	.uleb128 0x8
	.4byte	.LASF40
	.byte	0x8
	.byte	0x45
	.4byte	0x392
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF41
	.byte	0x8
	.byte	0x46
	.4byte	0x3a3
	.byte	0x3
	.byte	0x23
	.uleb128 0x600
	.uleb128 0x8
	.4byte	.LASF42
	.byte	0x8
	.byte	0x47
	.4byte	0x3b3
	.byte	0x3
	.byte	0x23
	.uleb128 0x610
	.uleb128 0x8
	.4byte	.LASF43
	.byte	0x8
	.byte	0x48
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x690
	.uleb128 0x8
	.4byte	.LASF44
	.byte	0x8
	.byte	0x49
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x692
	.uleb128 0x8
	.4byte	.LASF45
	.byte	0x8
	.byte	0x4a
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x694
	.uleb128 0x8
	.4byte	.LASF46
	.byte	0x8
	.byte	0x4b
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x696
	.uleb128 0x8
	.4byte	.LASF47
	.byte	0x8
	.byte	0x4c
	.4byte	0x3c3
	.byte	0x3
	.byte	0x23
	.uleb128 0x698
	.uleb128 0x8
	.4byte	.LASF48
	.byte	0x8
	.byte	0x4d
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x7de
	.uleb128 0x8
	.4byte	.LASF49
	.byte	0x8
	.byte	0x4e
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7e0
	.uleb128 0x8
	.4byte	.LASF50
	.byte	0x8
	.byte	0x4f
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7e4
	.uleb128 0x8
	.4byte	.LASF51
	.byte	0x8
	.byte	0x50
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7e8
	.uleb128 0x8
	.4byte	.LASF52
	.byte	0x8
	.byte	0x51
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7ec
	.uleb128 0x8
	.4byte	.LASF53
	.byte	0x8
	.byte	0x52
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7f0
	.uleb128 0x8
	.4byte	.LASF54
	.byte	0x8
	.byte	0x53
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7f4
	.uleb128 0x8
	.4byte	.LASF55
	.byte	0x8
	.byte	0x54
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7f8
	.uleb128 0x9
	.ascii	"tag\000"
	.byte	0x8
	.byte	0x55
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7fc
	.uleb128 0x8
	.4byte	.LASF56
	.byte	0x8
	.byte	0x57
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x800
	.uleb128 0x8
	.4byte	.LASF57
	.byte	0x8
	.byte	0x58
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x804
	.uleb128 0x8
	.4byte	.LASF58
	.byte	0x8
	.byte	0x59
	.4byte	0x3d3
	.byte	0x3
	.byte	0x23
	.uleb128 0x808
	.byte	0
	.uleb128 0x10
	.4byte	0xa0
	.4byte	0x3a3
	.uleb128 0x11
	.4byte	0x36
	.2byte	0x17f
	.byte	0
	.uleb128 0x10
	.4byte	0xa0
	.4byte	0x3b3
	.uleb128 0x12
	.4byte	0x36
	.byte	0x3
	.byte	0
	.uleb128 0x10
	.4byte	0x8a
	.4byte	0x3c3
	.uleb128 0x12
	.4byte	0x36
	.byte	0x3f
	.byte	0
	.uleb128 0x10
	.4byte	0x8a
	.4byte	0x3d3
	.uleb128 0x12
	.4byte	0x36
	.byte	0xa2
	.byte	0
	.uleb128 0x10
	.4byte	0xa0
	.4byte	0x3e3
	.uleb128 0x12
	.4byte	0x36
	.byte	0x1
	.byte	0
	.uleb128 0xf
	.4byte	.LASF59
	.2byte	0x810
	.byte	0x8
	.byte	0x5c
	.4byte	0x486
	.uleb128 0x8
	.4byte	.LASF60
	.byte	0x8
	.byte	0x5e
	.4byte	0x486
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF61
	.byte	0x8
	.byte	0x5f
	.4byte	0x486
	.byte	0x3
	.byte	0x23
	.uleb128 0x200
	.uleb128 0x8
	.4byte	.LASF47
	.byte	0x8
	.byte	0x60
	.4byte	0x496
	.byte	0x3
	.byte	0x23
	.uleb128 0x400
	.uleb128 0x9
	.ascii	"cnt\000"
	.byte	0x8
	.byte	0x61
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x7f4
	.uleb128 0x8
	.4byte	.LASF62
	.byte	0x8
	.byte	0x62
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x7f6
	.uleb128 0x8
	.4byte	.LASF63
	.byte	0x8
	.byte	0x63
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x7f8
	.uleb128 0x9
	.ascii	"blk\000"
	.byte	0x8
	.byte	0x64
	.4byte	0x8a
	.byte	0x3
	.byte	0x23
	.uleb128 0x7fa
	.uleb128 0x9
	.ascii	"tag\000"
	.byte	0x8
	.byte	0x65
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x7fc
	.uleb128 0x8
	.4byte	.LASF56
	.byte	0x8
	.byte	0x67
	.4byte	0xa0
	.byte	0x3
	.byte	0x23
	.uleb128 0x800
	.uleb128 0x8
	.4byte	.LASF58
	.byte	0x8
	.byte	0x68
	.4byte	0x4a6
	.byte	0x3
	.byte	0x23
	.uleb128 0x804
	.byte	0
	.uleb128 0x10
	.4byte	0xa0
	.4byte	0x496
	.uleb128 0x12
	.4byte	0x36
	.byte	0x7f
	.byte	0
	.uleb128 0x10
	.4byte	0xa0
	.4byte	0x4a6
	.uleb128 0x12
	.4byte	0x36
	.byte	0xfc
	.byte	0
	.uleb128 0x10
	.4byte	0xa0
	.4byte	0x4b6
	.uleb128 0x12
	.4byte	0x36
	.byte	0x2
	.byte	0
	.uleb128 0x7
	.4byte	.LASF64
	.byte	0xc
	.byte	0x8
	.byte	0x6b
	.4byte	0x4fb
	.uleb128 0x8
	.4byte	.LASF57
	.byte	0x8
	.byte	0x6c
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF65
	.byte	0x8
	.byte	0x6d
	.4byte	0x75
	.byte	0x2
	.byte	0x23
	.uleb128 0x2
	.uleb128 0x8
	.4byte	.LASF66
	.byte	0x8
	.byte	0x6e
	.4byte	0xa0
	.byte	0x2
	.byte	0x23
	.uleb128 0x4
	.uleb128 0x8
	.4byte	.LASF15
	.byte	0x8
	.byte	0x6f
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0x8
	.byte	0
	.uleb128 0x7
	.4byte	.LASF67
	.byte	0x14
	.byte	0x8
	.byte	0x71
	.4byte	0x56a
	.uleb128 0x8
	.4byte	.LASF68
	.byte	0x8
	.byte	0x72
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF69
	.byte	0x8
	.byte	0x73
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x2
	.uleb128 0x8
	.4byte	.LASF70
	.byte	0x8
	.byte	0x74
	.4byte	0xa0
	.byte	0x2
	.byte	0x23
	.uleb128 0x4
	.uleb128 0x8
	.4byte	.LASF71
	.byte	0x8
	.byte	0x75
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x8
	.uleb128 0x8
	.4byte	.LASF72
	.byte	0x8
	.byte	0x76
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0xa
	.uleb128 0x8
	.4byte	.LASF73
	.byte	0x8
	.byte	0x77
	.4byte	0x75
	.byte	0x2
	.byte	0x23
	.uleb128 0xc
	.uleb128 0x8
	.4byte	.LASF56
	.byte	0x8
	.byte	0x78
	.4byte	0xa0
	.byte	0x2
	.byte	0x23
	.uleb128 0x10
	.byte	0
	.uleb128 0x7
	.4byte	.LASF74
	.byte	0xc
	.byte	0x8
	.byte	0x7b
	.4byte	0x5af
	.uleb128 0x8
	.4byte	.LASF38
	.byte	0x8
	.byte	0x7c
	.4byte	0xa0
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF71
	.byte	0x8
	.byte	0x7d
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x4
	.uleb128 0x8
	.4byte	.LASF72
	.byte	0x8
	.byte	0x7e
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x6
	.uleb128 0x8
	.4byte	.LASF73
	.byte	0x8
	.byte	0x7f
	.4byte	0x75
	.byte	0x2
	.byte	0x23
	.uleb128 0x8
	.byte	0
	.uleb128 0xb
	.4byte	.LASF76
	.byte	0x4
	.byte	0x8
	.byte	0x82
	.4byte	0x5d4
	.uleb128 0xc
	.4byte	.LASF77
	.sleb128 0
	.uleb128 0xc
	.4byte	.LASF78
	.sleb128 1
	.uleb128 0xc
	.4byte	.LASF79
	.sleb128 2
	.uleb128 0xc
	.4byte	.LASF80
	.sleb128 3
	.byte	0
	.uleb128 0xb
	.4byte	.LASF81
	.byte	0x4
	.byte	0x8
	.byte	0x89
	.4byte	0x5f3
	.uleb128 0xc
	.4byte	.LASF82
	.sleb128 0
	.uleb128 0xc
	.4byte	.LASF83
	.sleb128 1
	.uleb128 0xc
	.4byte	.LASF84
	.sleb128 2
	.byte	0
	.uleb128 0x7
	.4byte	.LASF85
	.byte	0x18
	.byte	0x8
	.byte	0x8f
	.4byte	0x670
	.uleb128 0x8
	.4byte	.LASF86
	.byte	0x8
	.byte	0x90
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x8
	.4byte	.LASF87
	.byte	0x8
	.byte	0x91
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x2
	.uleb128 0x8
	.4byte	.LASF88
	.byte	0x8
	.byte	0x92
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x4
	.uleb128 0x8
	.4byte	.LASF89
	.byte	0x8
	.byte	0x93
	.4byte	0x1dd
	.byte	0x2
	.byte	0x23
	.uleb128 0x8
	.uleb128 0x8
	.4byte	.LASF90
	.byte	0x8
	.byte	0x94
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0xc
	.uleb128 0x8
	.4byte	.LASF91
	.byte	0x8
	.byte	0x95
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0xe
	.uleb128 0x8
	.4byte	.LASF92
	.byte	0x8
	.byte	0x96
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0x10
	.uleb128 0x8
	.4byte	.LASF93
	.byte	0x8
	.byte	0x97
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0x14
	.byte	0
	.uleb128 0x13
	.4byte	.LASF94
	.byte	0x2c
	.byte	0x3
	.2byte	0x18e
	.4byte	0x723
	.uleb128 0x14
	.4byte	.LASF95
	.byte	0x3
	.2byte	0x18f
	.4byte	0x8a
	.byte	0x2
	.byte	0x23
	.uleb128 0
	.uleb128 0x14
	.4byte	.LASF68
	.byte	0x3
	.2byte	0x190
	.4byte	0x1dd
	.byte	0x2
	.byte	0x23
	.uleb128 0x4
	.uleb128 0x14
	.4byte	.LASF69
	.byte	0x3
	.2byte	0x191
	.4byte	0x1dd
	.byte	0x2
	.byte	0x23
	.uleb128 0x8
	.uleb128 0x14
	.4byte	.LASF38
	.byte	0x3
	.2byte	0x192
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0xc
	.uleb128 0x14
	.4byte	.LASF71
	.byte	0x3
	.2byte	0x193
	.4byte	0x1dd
	.byte	0x2
	.byte	0x23
	.uleb128 0x10
	.uleb128 0x14
	.4byte	.LASF72
	.byte	0x3
	.2byte	0x194
	.4byte	0x1dd
	.byte	0x2
	.byte	0x23
	.uleb128 0x14
	.uleb128 0x14
	.4byte	.LASF73
	.byte	0x3
	.2byte	0x195
	.4byte	0x723
	.byte	0x2
	.byte	0x23
	.uleb128 0x18
	.uleb128 0x14
	.4byte	.LASF56
	.byte	0x3
	.2byte	0x196
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0x1c
	.uleb128 0x14
	.4byte	.LASF96
	.byte	0x3
	.2byte	0x197
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0x20
	.uleb128 0x14
	.4byte	.LASF97
	.byte	0x3
	.2byte	0x198
	.4byte	0x122
	.byte	0x2
	.byte	0x23
	.uleb128 0x24
	.uleb128 0x14
	.4byte	.LASF98
	.byte	0x3
	.2byte	0x199
	.4byte	0x723
	.byte	0x2
	.byte	0x23
	.uleb128 0x28
	.byte	0
	.uleb128 0xa
	.byte	0x4
	.4byte	0x75
	.uleb128 0x15
	.byte	0x1
	.4byte	.LASF307
	.byte	0x1
	.2byte	0x1b6
	.byte	0x1
	.byte	0x1
	.uleb128 0x16
	.byte	0x1
	.4byte	.LASF101
	.byte	0x3
	.byte	0x8
	.byte	0x1
	.4byte	0xa0
	.byte	0x3
	.4byte	0x75d
	.uleb128 0x17
	.4byte	.LASF99
	.byte	0x3
	.byte	0x8
	.4byte	0x8a
	.uleb128 0x17
	.4byte	.LASF100
	.byte	0x3
	.byte	0x8
	.4byte	0x8a
	.byte	0
	.uleb128 0x16
	.byte	0x1
	.4byte	.LASF102
	.byte	0x3
	.byte	0x11
	.byte	0x1
	.4byte	0x8a
	.byte	0x3
	.4byte	0x77b
	.uleb128 0x17
	.4byte	.LASF103
	.byte	0x3
	.byte	0x11
	.4byte	0xa0
	.byte	0
	.uleb128 0x16
	.byte	0x1
	.4byte	.LASF104
	.byte	0x3
	.byte	0x59
	.byte	0x1
	.4byte	0x8a
	.byte	0x1
	.4byte	0x7af
	.uleb128 0x17
	.4byte	.LASF38
	.byte	0x3
	.byte	0x59
	.4byte	0xa0
	.uleb128 0x18
	.ascii	"cnt\000"
	.byte	0x3
	.byte	0x5b
	.4byte	0x8a
	.uleb128 0x19
	.4byte	.LASF105
	.byte	0x3
	.byte	0x5c
	.4byte	0x8a
	.byte	0
	.uleb128 0x16
	.byte	0x1
	.4byte	.LASF106
	.byte	0x3
	.byte	0x22
	.byte	0x1
	.4byte	0xa0
	.byte	0x3
	.4byte	0x804
	.uleb128 0x17
	.4byte	.LASF100
	.byte	0x3
	.byte	0x22
	.4byte	0x8a
	.uleb128 0x17
	.4byte	.LASF105
	.byte	0x3
	.byte	0x22
	.4byte	0x8a
	.uleb128 0x19
	.4byte	.LASF99
	.byte	0x3
	.byte	0x24
	.4byte	0x8a
	.uleb128 0x19
	.4byte	.LASF107
	.byte	0x3
	.byte	0x24
	.4byte	0x8a
	.uleb128 0x19
	.4byte	.LASF103
	.byte	0x3
	.byte	0x25
	.4byte	0xa0
	.uleb128 0x18
	.ascii	"ppn\000"
	.byte	0x3
	.byte	0x25
	.4byte	0xa0
	.byte	0
	.uleb128 0x1a
	.byte	0x1
	.4byte	.LASF308
	.byte	0x3
	.2byte	0x539
	.byte	0x1
	.byte	0x1
	.4byte	0x82c
	.uleb128 0x1b
	.4byte	.LASF99
	.byte	0x3
	.2byte	0x539
	.4byte	0x8a
	.uleb128 0x1b
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x539
	.4byte	0x8a
	.byte	0
	.uleb128 0x16
	.byte	0x1
	.4byte	.LASF108
	.byte	0x3
	.byte	0x31
	.byte	0x1
	.4byte	0x8a
	.byte	0x3
	.4byte	0x86b
	.uleb128 0x1c
	.ascii	"ppn\000"
	.byte	0x3
	.byte	0x31
	.4byte	0xa0
	.uleb128 0x19
	.4byte	.LASF109
	.byte	0x3
	.byte	0x33
	.4byte	0x8a
	.uleb128 0x19
	.4byte	.LASF110
	.byte	0x3
	.byte	0x33
	.4byte	0x8a
	.uleb128 0x19
	.4byte	.LASF103
	.byte	0x3
	.byte	0x34
	.4byte	0xa0
	.byte	0
	.uleb128 0x16
	.byte	0x1
	.4byte	.LASF111
	.byte	0x1
	.byte	0x78
	.byte	0x1
	.4byte	0x60
	.byte	0x1
	.4byte	0x889
	.uleb128 0x17
	.4byte	.LASF112
	.byte	0x1
	.byte	0x78
	.4byte	0x60
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF113
	.byte	0x1
	.byte	0x5
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB124
	.4byte	.LFE124
	.4byte	.LLST0
	.4byte	0x9a0
	.uleb128 0x1e
	.ascii	"i\000"
	.byte	0x1
	.byte	0x7
	.4byte	0xa0
	.4byte	.LLST1
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0
	.4byte	0x8d3
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x1e
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x1e
	.4byte	0xc4
	.byte	0
	.uleb128 0x20
	.4byte	.LBB81
	.4byte	.LBE81
	.4byte	0x8f7
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x1f
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x1f
	.4byte	0xc4
	.byte	0
	.uleb128 0x20
	.4byte	.LBB82
	.4byte	.LBE82
	.4byte	0x91b
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x20
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x20
	.4byte	0xc4
	.byte	0
	.uleb128 0x20
	.4byte	.LBB83
	.4byte	.LBE83
	.4byte	0x93f
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x21
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x21
	.4byte	0xc4
	.byte	0
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x18
	.4byte	0x95f
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x2f
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x2f
	.4byte	0xc4
	.byte	0
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x30
	.4byte	0x97f
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x38
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x38
	.4byte	0xc4
	.byte	0
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x48
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x3a
	.4byte	0x2d
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x3a
	.4byte	0xc4
	.4byte	.LLST2
	.byte	0
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF114
	.byte	0x1
	.byte	0x42
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB125
	.4byte	.LFE125
	.4byte	.LLST3
	.4byte	0xa7b
	.uleb128 0x1e
	.ascii	"i\000"
	.byte	0x1
	.byte	0x44
	.4byte	0xa0
	.4byte	.LLST4
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x60
	.4byte	0x9ee
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x4e
	.4byte	0x2d
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x4e
	.4byte	0xc4
	.4byte	.LLST5
	.byte	0
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x78
	.4byte	0xa12
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x58
	.4byte	0x2d
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x58
	.4byte	0xc4
	.4byte	.LLST6
	.byte	0
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x90
	.4byte	0xa36
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x5a
	.4byte	0x2d
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x5a
	.4byte	0xc4
	.4byte	.LLST7
	.byte	0
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0xb0
	.4byte	0xa5a
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x5d
	.4byte	0x2d
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x5d
	.4byte	0xc4
	.4byte	.LLST8
	.byte	0
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0xd0
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x5f
	.4byte	0x2d
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x5f
	.4byte	0xc4
	.4byte	.LLST9
	.byte	0
	.byte	0
	.uleb128 0x22
	.byte	0x1
	.4byte	.LASF228
	.byte	0x1
	.byte	0x64
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB126
	.4byte	.LFE126
	.4byte	.LLST10
	.uleb128 0x23
	.4byte	0x86b
	.4byte	.LFB127
	.4byte	.LFE127
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	0xab2
	.uleb128 0x24
	.4byte	0x87d
	.4byte	.LLST11
	.byte	0
	.uleb128 0x25
	.byte	0x1
	.4byte	.LASF115
	.byte	0x1
	.2byte	0x180
	.byte	0x1
	.4byte	0xa0
	.4byte	.LFB134
	.4byte	.LFE134
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	0xadc
	.uleb128 0x26
	.4byte	.LASF116
	.byte	0x1
	.2byte	0x182
	.4byte	0xa0
	.byte	0
	.uleb128 0x27
	.byte	0x1
	.4byte	.LASF117
	.byte	0x1
	.2byte	0x192
	.byte	0x1
	.4byte	.LFB136
	.4byte	.LFE136
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.uleb128 0x28
	.byte	0x1
	.4byte	.LASF118
	.byte	0x1
	.2byte	0x197
	.byte	0x1
	.4byte	.LFB137
	.4byte	.LFE137
	.4byte	.LLST12
	.uleb128 0x27
	.byte	0x1
	.4byte	.LASF119
	.byte	0x1
	.2byte	0x1a6
	.byte	0x1
	.4byte	.LFB138
	.4byte	.LFE138
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.uleb128 0x29
	.4byte	0x729
	.4byte	.LFB139
	.4byte	.LFE139
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.uleb128 0x28
	.byte	0x1
	.4byte	.LASF120
	.byte	0x1
	.2byte	0x1cc
	.byte	0x1
	.4byte	.LFB140
	.4byte	.LFE140
	.4byte	.LLST13
	.uleb128 0x2a
	.byte	0x1
	.4byte	.LASF121
	.byte	0x2
	.byte	0x3
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB142
	.4byte	.LFE142
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	0xb6e
	.uleb128 0x2b
	.4byte	.LASF123
	.byte	0x2
	.byte	0x3
	.4byte	0xa0
	.4byte	.LLST14
	.byte	0
	.uleb128 0x2a
	.byte	0x1
	.4byte	.LASF122
	.byte	0x2
	.byte	0x14
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB143
	.4byte	.LFE143
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	0xba3
	.uleb128 0x2b
	.4byte	.LASF123
	.byte	0x2
	.byte	0x14
	.4byte	0xa0
	.4byte	.LLST15
	.uleb128 0x18
	.ascii	"i\000"
	.byte	0x2
	.byte	0x16
	.4byte	0xa0
	.byte	0
	.uleb128 0x2c
	.byte	0x1
	.4byte	.LASF153
	.byte	0x2
	.byte	0x62
	.byte	0x1
	.4byte	.LFB145
	.4byte	.LFE145
	.4byte	.LLST16
	.4byte	0xc04
	.uleb128 0x2b
	.4byte	.LASF124
	.byte	0x2
	.byte	0x62
	.4byte	0x1dd
	.4byte	.LLST17
	.uleb128 0x2b
	.4byte	.LASF125
	.byte	0x2
	.byte	0x62
	.4byte	0x122
	.4byte	.LLST18
	.uleb128 0x18
	.ascii	"i\000"
	.byte	0x2
	.byte	0x64
	.4byte	0xa0
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0xf0
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x2
	.byte	0x66
	.4byte	0x2d
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x2
	.byte	0x66
	.4byte	0xc4
	.4byte	.LLST19
	.byte	0
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF126
	.byte	0x2
	.byte	0xa5
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB147
	.4byte	.LFE147
	.4byte	.LLST20
	.4byte	0xc3f
	.uleb128 0x18
	.ascii	"i\000"
	.byte	0x2
	.byte	0xa7
	.4byte	0x60
	.uleb128 0x2d
	.4byte	.LASF127
	.byte	0x2
	.byte	0xa8
	.4byte	0x60
	.byte	0x1
	.byte	0x55
	.uleb128 0x2e
	.4byte	.LASF178
	.byte	0x2
	.byte	0xad
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF128
	.byte	0x2
	.byte	0x6f
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB146
	.4byte	.LFE146
	.4byte	.LLST21
	.4byte	0xcaa
	.uleb128 0x1e
	.ascii	"die\000"
	.byte	0x2
	.byte	0x71
	.4byte	0xa0
	.4byte	.LLST22
	.uleb128 0x18
	.ascii	"blk\000"
	.byte	0x2
	.byte	0x72
	.4byte	0x8a
	.uleb128 0x2d
	.4byte	.LASF129
	.byte	0x2
	.byte	0x73
	.4byte	0xcaa
	.byte	0x3
	.byte	0x91
	.sleb128 -72
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x108
	.uleb128 0x1e
	.ascii	"__p\000"
	.byte	0x2
	.byte	0x7a
	.4byte	0x2d
	.4byte	.LLST23
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x2
	.byte	0x7a
	.4byte	0xc4
	.4byte	.LLST24
	.byte	0
	.byte	0
	.uleb128 0x10
	.4byte	0xcf
	.4byte	0xcba
	.uleb128 0x12
	.4byte	0x36
	.byte	0x1
	.byte	0
	.uleb128 0x23
	.4byte	0x734
	.4byte	.LFB148
	.4byte	.LFE148
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	0xcdf
	.uleb128 0x24
	.4byte	0x746
	.4byte	.LLST25
	.uleb128 0x2f
	.4byte	0x751
	.byte	0x1
	.byte	0x51
	.byte	0
	.uleb128 0x30
	.4byte	0x75d
	.4byte	.LFB149
	.4byte	.LFE149
	.4byte	.LLST26
	.4byte	0xcfe
	.uleb128 0x24
	.4byte	0x76f
	.4byte	.LLST27
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF130
	.byte	0x3
	.byte	0x1a
	.byte	0x1
	.4byte	0x8a
	.4byte	.LFB150
	.4byte	.LFE150
	.4byte	.LLST28
	.4byte	0xd2b
	.uleb128 0x31
	.ascii	"ppn\000"
	.byte	0x3
	.byte	0x1a
	.4byte	0xa0
	.4byte	.LLST29
	.byte	0
	.uleb128 0x30
	.4byte	0x7af
	.4byte	.LFB151
	.4byte	.LFE151
	.4byte	.LLST30
	.4byte	0xd81
	.uleb128 0x24
	.4byte	0x7c1
	.4byte	.LLST31
	.uleb128 0x24
	.4byte	0x7cc
	.4byte	.LLST32
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB107
	.4byte	.Ldebug_ranges0+0x120
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.uleb128 0x30
	.4byte	0x82c
	.4byte	.LFB152
	.4byte	.LFE152
	.4byte	.LLST33
	.4byte	0xdc4
	.uleb128 0x24
	.4byte	0x83e
	.4byte	.LLST34
	.uleb128 0x32
	.4byte	0x849
	.uleb128 0x32
	.4byte	0x854
	.uleb128 0x32
	.4byte	0x85f
	.uleb128 0x33
	.4byte	0x75d
	.4byte	.LBB113
	.4byte	.Ldebug_ranges0+0x140
	.byte	0x3
	.byte	0x38
	.uleb128 0x34
	.4byte	0x76f
	.byte	0
	.byte	0
	.uleb128 0x30
	.4byte	0x77b
	.4byte	.LFB153
	.4byte	.LFE153
	.4byte	.LLST35
	.4byte	0xe1e
	.uleb128 0x24
	.4byte	0x78d
	.4byte	.LLST36
	.uleb128 0x35
	.4byte	0x798
	.4byte	.LLST37
	.uleb128 0x35
	.4byte	0x7a3
	.4byte	.LLST38
	.uleb128 0x36
	.4byte	0x77b
	.4byte	.LBB121
	.4byte	.LBE121
	.byte	0x3
	.byte	0x59
	.uleb128 0x37
	.4byte	.LBB122
	.4byte	.LBE122
	.uleb128 0x32
	.4byte	0x798
	.uleb128 0x34
	.4byte	0x78d
	.uleb128 0x32
	.4byte	0x7a3
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF131
	.byte	0x3
	.byte	0x72
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB154
	.4byte	.LFE154
	.4byte	.LLST39
	.4byte	0xf31
	.uleb128 0x38
	.4byte	.LASF100
	.byte	0x3
	.byte	0x72
	.4byte	0x8a
	.byte	0x1
	.byte	0x50
	.uleb128 0x2b
	.4byte	.LASF132
	.byte	0x3
	.byte	0x72
	.4byte	0x122
	.4byte	.LLST40
	.uleb128 0x39
	.ascii	"req\000"
	.byte	0x3
	.byte	0x74
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -60
	.uleb128 0x1e
	.ascii	"low\000"
	.byte	0x3
	.byte	0x75
	.4byte	0x95
	.4byte	.LLST41
	.uleb128 0x2d
	.4byte	.LASF133
	.byte	0x3
	.byte	0x75
	.4byte	0x95
	.byte	0x1
	.byte	0x54
	.uleb128 0x1e
	.ascii	"mid\000"
	.byte	0x3
	.byte	0x75
	.4byte	0x95
	.4byte	.LLST42
	.uleb128 0x3a
	.4byte	0x7af
	.4byte	.LBB123
	.4byte	.Ldebug_ranges0+0x160
	.byte	0x3
	.byte	0x7d
	.4byte	0xee2
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x188
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB125
	.4byte	.Ldebug_ranges0+0x1b0
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x33
	.4byte	0x7af
	.4byte	.LBB137
	.4byte	.Ldebug_ranges0+0x1d0
	.byte	0x3
	.byte	0x86
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x1f8
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB139
	.4byte	.Ldebug_ranges0+0x220
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF134
	.byte	0x3
	.byte	0x9a
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB155
	.4byte	.LFE155
	.4byte	.LLST43
	.4byte	0xfb7
	.uleb128 0x31
	.ascii	"buf\000"
	.byte	0x3
	.byte	0x9a
	.4byte	0x122
	.4byte	.LLST44
	.uleb128 0x2b
	.4byte	.LASF123
	.byte	0x3
	.byte	0x9a
	.4byte	0xa0
	.4byte	.LLST45
	.uleb128 0x2b
	.4byte	.LASF135
	.byte	0x3
	.byte	0x9a
	.4byte	0x60
	.4byte	.LLST46
	.uleb128 0x39
	.ascii	"req\000"
	.byte	0x3
	.byte	0x9c
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x1e
	.ascii	"low\000"
	.byte	0x3
	.byte	0x9d
	.4byte	0x7f
	.4byte	.LLST47
	.uleb128 0x3b
	.4byte	.LASF133
	.byte	0x3
	.byte	0x9d
	.4byte	0x7f
	.4byte	.LLST48
	.uleb128 0x1e
	.ascii	"mid\000"
	.byte	0x3
	.byte	0x9d
	.4byte	0x7f
	.4byte	.LLST49
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF136
	.byte	0x2
	.byte	0x1f
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB144
	.4byte	.LFE144
	.4byte	.LLST50
	.4byte	0x104e
	.uleb128 0x1e
	.ascii	"blk\000"
	.byte	0x2
	.byte	0x21
	.4byte	0x8a
	.4byte	.LLST51
	.uleb128 0x3b
	.4byte	.LASF137
	.byte	0x2
	.byte	0x22
	.4byte	0x8a
	.4byte	.LLST52
	.uleb128 0x3b
	.4byte	.LASF138
	.byte	0x2
	.byte	0x23
	.4byte	0x8a
	.4byte	.LLST53
	.uleb128 0x3b
	.4byte	.LASF56
	.byte	0x2
	.byte	0x24
	.4byte	0xa0
	.4byte	.LLST54
	.uleb128 0x19
	.4byte	.LASF139
	.byte	0x2
	.byte	0x25
	.4byte	0x104e
	.uleb128 0x19
	.4byte	.LASF140
	.byte	0x2
	.byte	0x26
	.4byte	0x7f
	.uleb128 0x19
	.4byte	.LASF141
	.byte	0x2
	.byte	0x27
	.4byte	0x7f
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x238
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x2
	.byte	0x2a
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x2
	.byte	0x2a
	.4byte	0xc4
	.byte	0
	.byte	0
	.uleb128 0xa
	.byte	0x4
	.4byte	0x3e3
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF142
	.byte	0x3
	.2byte	0x105
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB157
	.4byte	.LFE157
	.4byte	.LLST55
	.4byte	0x11b2
	.uleb128 0x3d
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x105
	.4byte	0x8a
	.4byte	.LLST56
	.uleb128 0x3e
	.ascii	"id\000"
	.byte	0x3
	.2byte	0x105
	.4byte	0x1e3
	.4byte	.LLST57
	.uleb128 0x3f
	.4byte	.LASF143
	.byte	0x3
	.2byte	0x107
	.4byte	0x95
	.4byte	.LLST58
	.uleb128 0x3f
	.4byte	.LASF144
	.byte	0x3
	.2byte	0x107
	.4byte	0x95
	.4byte	.LLST59
	.uleb128 0x3f
	.4byte	.LASF145
	.byte	0x3
	.2byte	0x107
	.4byte	0x95
	.4byte	.LLST60
	.uleb128 0x40
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x108
	.4byte	0x8a
	.uleb128 0x41
	.ascii	"j\000"
	.byte	0x3
	.2byte	0x108
	.4byte	0x8a
	.4byte	.LLST61
	.uleb128 0x3f
	.4byte	.LASF146
	.byte	0x3
	.2byte	0x108
	.4byte	0x8a
	.4byte	.LLST62
	.uleb128 0x42
	.4byte	.LASF38
	.byte	0x3
	.2byte	0x109
	.4byte	0xa0
	.byte	0x1
	.byte	0x56
	.uleb128 0x42
	.4byte	.LASF132
	.byte	0x3
	.2byte	0x10a
	.4byte	0x3d3
	.byte	0x2
	.byte	0x91
	.sleb128 -48
	.uleb128 0x20
	.4byte	.LBB151
	.4byte	.LBE151
	.4byte	0x1134
	.uleb128 0x41
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x10e
	.4byte	0x2d
	.4byte	.LLST63
	.uleb128 0x41
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x10e
	.4byte	0xc4
	.4byte	.LLST64
	.byte	0
	.uleb128 0x20
	.4byte	.LBB152
	.4byte	.LBE152
	.4byte	0x1162
	.uleb128 0x41
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x110
	.4byte	0x2d
	.4byte	.LLST65
	.uleb128 0x41
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x110
	.4byte	0xc4
	.4byte	.LLST66
	.byte	0
	.uleb128 0x43
	.4byte	0x7af
	.4byte	.LBB153
	.4byte	.Ldebug_ranges0+0x250
	.byte	0x3
	.2byte	0x11b
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x290
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB155
	.4byte	.Ldebug_ranges0+0x2d0
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF147
	.byte	0x3
	.2byte	0x13f
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB158
	.4byte	.LFE158
	.4byte	.LLST67
	.4byte	0x12aa
	.uleb128 0x3d
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x13f
	.4byte	0x8a
	.4byte	.LLST68
	.uleb128 0x3d
	.4byte	.LASF72
	.byte	0x3
	.2byte	0x13f
	.4byte	0x8a
	.4byte	.LLST69
	.uleb128 0x3e
	.ascii	"id\000"
	.byte	0x3
	.2byte	0x13f
	.4byte	0x1e3
	.4byte	.LLST70
	.uleb128 0x42
	.4byte	.LASF148
	.byte	0x3
	.2byte	0x141
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x44
	.4byte	0x7af
	.4byte	.LBB177
	.4byte	.Ldebug_ranges0+0x300
	.byte	0x3
	.2byte	0x146
	.4byte	0x1261
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x328
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB179
	.4byte	.Ldebug_ranges0+0x350
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x378
	.4byte	0x1287
	.uleb128 0x40
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x149
	.4byte	0x2d
	.uleb128 0x41
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x149
	.4byte	0xc4
	.4byte	.LLST71
	.byte	0
	.uleb128 0x37
	.4byte	.LBB196
	.4byte	.LBE196
	.uleb128 0x40
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x14e
	.4byte	0x2d
	.uleb128 0x40
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x14e
	.4byte	0xc4
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF149
	.byte	0x3
	.2byte	0x158
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB159
	.4byte	.LFE159
	.4byte	.LLST72
	.4byte	0x135c
	.uleb128 0x45
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x158
	.4byte	0x8a
	.byte	0x1
	.byte	0x50
	.uleb128 0x3e
	.ascii	"id\000"
	.byte	0x3
	.2byte	0x158
	.4byte	0x1e3
	.4byte	.LLST73
	.uleb128 0x26
	.4byte	.LASF72
	.byte	0x3
	.2byte	0x15a
	.4byte	0x8a
	.uleb128 0x26
	.4byte	.LASF38
	.byte	0x3
	.2byte	0x15b
	.4byte	0xa0
	.uleb128 0x42
	.4byte	.LASF148
	.byte	0x3
	.2byte	0x15c
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x43
	.4byte	0x7af
	.4byte	.LBB197
	.4byte	.Ldebug_ranges0+0x398
	.byte	0x3
	.2byte	0x161
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x3c8
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB199
	.4byte	.Ldebug_ranges0+0x3f8
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF150
	.byte	0x3
	.2byte	0x214
	.byte	0x1
	.4byte	0x8a
	.4byte	.LFB162
	.4byte	.LFE162
	.4byte	.LLST74
	.4byte	0x13b9
	.uleb128 0x3d
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x214
	.4byte	0x8a
	.4byte	.LLST75
	.uleb128 0x41
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x216
	.4byte	0x8a
	.4byte	.LLST76
	.uleb128 0x3f
	.4byte	.LASF151
	.byte	0x3
	.2byte	0x216
	.4byte	0x8a
	.4byte	.LLST77
	.uleb128 0x3f
	.4byte	.LASF152
	.byte	0x3
	.2byte	0x217
	.4byte	0xa0
	.4byte	.LLST78
	.byte	0
	.uleb128 0x46
	.byte	0x1
	.4byte	.LASF154
	.byte	0x3
	.2byte	0x3bc
	.byte	0x1
	.4byte	.LFB169
	.4byte	.LFE169
	.4byte	.LLST79
	.4byte	0x13ec
	.uleb128 0x45
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x3bc
	.4byte	0x8a
	.byte	0x1
	.byte	0x50
	.uleb128 0x40
	.ascii	"j\000"
	.byte	0x3
	.2byte	0x3be
	.4byte	0x8a
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF155
	.byte	0x3
	.2byte	0x3d3
	.byte	0x1
	.4byte	0x8a
	.4byte	.LFB170
	.4byte	.LFE170
	.4byte	.LLST80
	.4byte	0x1475
	.uleb128 0x3d
	.4byte	.LASF156
	.byte	0x3
	.2byte	0x3d3
	.4byte	0x75
	.4byte	.LLST81
	.uleb128 0x3d
	.4byte	.LASF157
	.byte	0x3
	.2byte	0x3d3
	.4byte	0x723
	.4byte	.LLST82
	.uleb128 0x41
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x3d5
	.4byte	0x8a
	.4byte	.LLST83
	.uleb128 0x41
	.ascii	"j\000"
	.byte	0x3
	.2byte	0x3d5
	.4byte	0x8a
	.4byte	.LLST84
	.uleb128 0x3f
	.4byte	.LASF158
	.byte	0x3
	.2byte	0x3d6
	.4byte	0x8a
	.4byte	.LLST85
	.uleb128 0x42
	.4byte	.LASF159
	.byte	0x3
	.2byte	0x3d7
	.4byte	0x8a
	.byte	0x1
	.byte	0x57
	.uleb128 0x3f
	.4byte	.LASF160
	.byte	0x3
	.2byte	0x3d8
	.4byte	0x8a
	.4byte	.LLST86
	.byte	0
	.uleb128 0x30
	.4byte	0x804
	.4byte	.LFB176
	.4byte	.LFE176
	.4byte	.LLST87
	.4byte	0x14c2
	.uleb128 0x24
	.4byte	0x813
	.4byte	.LLST88
	.uleb128 0x24
	.4byte	0x81f
	.4byte	.LLST89
	.uleb128 0x47
	.4byte	0x804
	.4byte	.LBB215
	.4byte	.LBE215
	.byte	0x3
	.2byte	0x539
	.uleb128 0x37
	.4byte	.LBB216
	.4byte	.LBE216
	.uleb128 0x34
	.4byte	0x81f
	.uleb128 0x34
	.4byte	0x813
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x46
	.byte	0x1
	.4byte	.LASF161
	.byte	0x3
	.2byte	0x540
	.byte	0x1
	.4byte	.LFB177
	.4byte	.LFE177
	.4byte	.LLST90
	.4byte	0x14ed
	.uleb128 0x3d
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x540
	.4byte	0x8a
	.4byte	.LLST91
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF162
	.byte	0x3
	.2byte	0x548
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB178
	.4byte	.LFE178
	.4byte	.LLST92
	.4byte	0x1599
	.uleb128 0x3d
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x548
	.4byte	0x8a
	.4byte	.LLST93
	.uleb128 0x26
	.4byte	.LASF163
	.byte	0x3
	.2byte	0x54a
	.4byte	0x8a
	.uleb128 0x3f
	.4byte	.LASF164
	.byte	0x3
	.2byte	0x54a
	.4byte	0x8a
	.4byte	.LLST94
	.uleb128 0x3f
	.4byte	.LASF99
	.byte	0x3
	.2byte	0x54b
	.4byte	0x8a
	.4byte	.LLST95
	.uleb128 0x26
	.4byte	.LASF165
	.byte	0x3
	.2byte	0x54b
	.4byte	0x8a
	.uleb128 0x3f
	.4byte	.LASF123
	.byte	0x3
	.2byte	0x54c
	.4byte	0xa0
	.4byte	.LLST96
	.uleb128 0x48
	.4byte	0x734
	.4byte	.LBB217
	.4byte	.LBE217
	.byte	0x3
	.2byte	0x556
	.4byte	0x1582
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.uleb128 0x43
	.4byte	0x75d
	.4byte	.LBB219
	.4byte	.Ldebug_ranges0+0x418
	.byte	0x3
	.2byte	0x56d
	.uleb128 0x34
	.4byte	0x76f
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF166
	.byte	0x3
	.2byte	0x590
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB179
	.4byte	.LFE179
	.4byte	.LLST97
	.4byte	0x15fc
	.uleb128 0x41
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x592
	.4byte	0x8a
	.4byte	.LLST98
	.uleb128 0x41
	.ascii	"j\000"
	.byte	0x3
	.2byte	0x592
	.4byte	0x8a
	.4byte	.LLST99
	.uleb128 0x26
	.4byte	.LASF167
	.byte	0x3
	.2byte	0x592
	.4byte	0x8a
	.uleb128 0x26
	.4byte	.LASF168
	.byte	0x3
	.2byte	0x592
	.4byte	0x8a
	.uleb128 0x3f
	.4byte	.LASF169
	.byte	0x3
	.2byte	0x592
	.4byte	0x8a
	.4byte	.LLST100
	.byte	0
	.uleb128 0x46
	.byte	0x1
	.4byte	.LASF170
	.byte	0x3
	.2byte	0x5be
	.byte	0x1
	.4byte	.LFB180
	.4byte	.LFE180
	.4byte	.LLST101
	.4byte	0x1621
	.uleb128 0x40
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x5c0
	.4byte	0x8a
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF171
	.byte	0x3
	.2byte	0x5ce
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB181
	.4byte	.LFE181
	.4byte	.LLST102
	.4byte	0x16ac
	.uleb128 0x26
	.4byte	.LASF172
	.byte	0x3
	.2byte	0x5d0
	.4byte	0x7f
	.uleb128 0x41
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x5d0
	.4byte	0x7f
	.4byte	.LLST103
	.uleb128 0x26
	.4byte	.LASF140
	.byte	0x3
	.2byte	0x5d0
	.4byte	0x7f
	.uleb128 0x26
	.4byte	.LASF141
	.byte	0x3
	.2byte	0x5d0
	.4byte	0x7f
	.uleb128 0x3f
	.4byte	.LASF14
	.byte	0x3
	.2byte	0x5d1
	.4byte	0xa0
	.4byte	.LLST104
	.uleb128 0x26
	.4byte	.LASF173
	.byte	0x3
	.2byte	0x5d3
	.4byte	0x16ac
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x438
	.uleb128 0x40
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x5d7
	.4byte	0x2d
	.uleb128 0x40
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x5d7
	.4byte	0xc4
	.byte	0
	.byte	0
	.uleb128 0xa
	.byte	0x4
	.4byte	0x259
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF174
	.byte	0x3
	.2byte	0x622
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB182
	.4byte	.LFE182
	.4byte	.LLST105
	.4byte	0x170e
	.uleb128 0x3d
	.4byte	.LASF57
	.byte	0x3
	.2byte	0x622
	.4byte	0xa0
	.4byte	.LLST106
	.uleb128 0x42
	.4byte	.LASF129
	.byte	0x3
	.2byte	0x624
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x3f
	.4byte	.LASF175
	.byte	0x3
	.2byte	0x625
	.4byte	0xa0
	.4byte	.LLST107
	.uleb128 0x42
	.4byte	.LASF176
	.byte	0x3
	.2byte	0x626
	.4byte	0xa0
	.byte	0x1
	.byte	0x57
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF177
	.byte	0x3
	.2byte	0x648
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB183
	.4byte	.LFE183
	.4byte	.LLST108
	.4byte	0x175a
	.uleb128 0x49
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x64a
	.4byte	0x95
	.byte	0x1
	.byte	0x55
	.uleb128 0x40
	.ascii	"j\000"
	.byte	0x3
	.2byte	0x64a
	.4byte	0x95
	.uleb128 0x42
	.4byte	.LASF129
	.byte	0x3
	.2byte	0x64b
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x4a
	.4byte	.LASF179
	.byte	0x3
	.2byte	0x650
	.byte	0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF180
	.byte	0x3
	.byte	0xb8
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB156
	.4byte	.LFE156
	.4byte	.LLST109
	.4byte	0x1811
	.uleb128 0x31
	.ascii	"id\000"
	.byte	0x3
	.byte	0xb8
	.4byte	0x1e3
	.4byte	.LLST110
	.uleb128 0x18
	.ascii	"i\000"
	.byte	0x3
	.byte	0xba
	.4byte	0x8a
	.uleb128 0x1e
	.ascii	"cnt\000"
	.byte	0x3
	.byte	0xba
	.4byte	0x8a
	.4byte	.LLST111
	.uleb128 0x2d
	.4byte	.LASF181
	.byte	0x3
	.byte	0xba
	.4byte	0x8a
	.byte	0x1
	.byte	0x54
	.uleb128 0x19
	.4byte	.LASF182
	.byte	0x3
	.byte	0xbb
	.4byte	0xa0
	.uleb128 0x3b
	.4byte	.LASF38
	.byte	0x3
	.byte	0xbb
	.4byte	0xa0
	.4byte	.LLST112
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x450
	.4byte	0x17ec
	.uleb128 0x1e
	.ascii	"__p\000"
	.byte	0x3
	.byte	0xe9
	.4byte	0x2d
	.4byte	.LLST113
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x3
	.byte	0xe9
	.4byte	0xc4
	.4byte	.LLST114
	.byte	0
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x470
	.uleb128 0x1e
	.ascii	"__p\000"
	.byte	0x3
	.byte	0xf3
	.4byte	0x2d
	.4byte	.LLST115
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x3
	.byte	0xf3
	.4byte	0xc4
	.4byte	.LLST116
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF183
	.byte	0x3
	.2byte	0x1a6
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB161
	.4byte	.LFE161
	.4byte	.LLST117
	.4byte	0x18f5
	.uleb128 0x3d
	.4byte	.LASF57
	.byte	0x3
	.2byte	0x1a6
	.4byte	0xa0
	.4byte	.LLST118
	.uleb128 0x3e
	.ascii	"id\000"
	.byte	0x3
	.2byte	0x1a6
	.4byte	0x1e3
	.4byte	.LLST119
	.uleb128 0x42
	.4byte	.LASF129
	.byte	0x3
	.2byte	0x1a8
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -60
	.uleb128 0x3f
	.4byte	.LASF184
	.byte	0x3
	.2byte	0x1a9
	.4byte	0xa0
	.4byte	.LLST120
	.uleb128 0x42
	.4byte	.LASF185
	.byte	0x3
	.2byte	0x1a9
	.4byte	0xa0
	.byte	0x3
	.byte	0x91
	.sleb128 -72
	.uleb128 0x26
	.4byte	.LASF99
	.byte	0x3
	.2byte	0x1aa
	.4byte	0x8a
	.uleb128 0x3f
	.4byte	.LASF186
	.byte	0x3
	.2byte	0x1ab
	.4byte	0x670
	.4byte	.LLST121
	.uleb128 0x4a
	.4byte	.LASF187
	.byte	0x3
	.2byte	0x1ce
	.uleb128 0x43
	.4byte	0x7af
	.4byte	.LBB233
	.4byte	.Ldebug_ranges0+0x490
	.byte	0x3
	.2byte	0x1e7
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x4b0
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x35
	.4byte	0x7f8
	.4byte	.LLST122
	.uleb128 0x36
	.4byte	0x734
	.4byte	.LBB235
	.4byte	.LBE235
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF188
	.byte	0x3
	.2byte	0x229
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB163
	.4byte	.LFE163
	.4byte	.LLST123
	.4byte	0x19ad
	.uleb128 0x40
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x22b
	.4byte	0x8a
	.uleb128 0x26
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x22b
	.4byte	0x8a
	.uleb128 0x26
	.4byte	.LASF189
	.byte	0x3
	.2byte	0x22b
	.4byte	0x8a
	.uleb128 0x3f
	.4byte	.LASF190
	.byte	0x3
	.2byte	0x22b
	.4byte	0x8a
	.4byte	.LLST124
	.uleb128 0x3f
	.4byte	.LASF191
	.byte	0x3
	.2byte	0x22b
	.4byte	0x8a
	.4byte	.LLST125
	.uleb128 0x3f
	.4byte	.LASF192
	.byte	0x3
	.2byte	0x22c
	.4byte	0xa0
	.4byte	.LLST126
	.uleb128 0x3f
	.4byte	.LASF193
	.byte	0x3
	.2byte	0x22c
	.4byte	0xa0
	.4byte	.LLST127
	.uleb128 0x26
	.4byte	.LASF151
	.byte	0x3
	.2byte	0x22c
	.4byte	0xa0
	.uleb128 0x3f
	.4byte	.LASF184
	.byte	0x3
	.2byte	0x22d
	.4byte	0xa0
	.4byte	.LLST128
	.uleb128 0x42
	.4byte	.LASF148
	.byte	0x3
	.2byte	0x22e
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x4b
	.4byte	.LASF203
	.byte	0x3
	.2byte	0x282
	.4byte	.L373
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF194
	.byte	0x3
	.2byte	0x298
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB164
	.4byte	.LFE164
	.4byte	.LLST129
	.4byte	0x19ea
	.uleb128 0x3d
	.4byte	.LASF195
	.byte	0x3
	.2byte	0x298
	.4byte	0xa0
	.4byte	.LLST130
	.uleb128 0x42
	.4byte	.LASF196
	.byte	0x3
	.2byte	0x29a
	.4byte	0x8a
	.byte	0x1
	.byte	0x56
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF197
	.byte	0x3
	.2byte	0x2aa
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB165
	.4byte	.LFE165
	.4byte	.LLST131
	.4byte	0x1a17
	.uleb128 0x42
	.4byte	.LASF105
	.byte	0x3
	.2byte	0x2ac
	.4byte	0xa0
	.byte	0x1
	.byte	0x55
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF198
	.byte	0x3
	.2byte	0x2bf
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB166
	.4byte	.LFE166
	.4byte	.LLST132
	.4byte	0x1af5
	.uleb128 0x3e
	.ascii	"lpn\000"
	.byte	0x3
	.2byte	0x2bf
	.4byte	0xa0
	.4byte	.LLST133
	.uleb128 0x3e
	.ascii	"ppn\000"
	.byte	0x3
	.2byte	0x2bf
	.4byte	0x122
	.4byte	.LLST134
	.uleb128 0x3d
	.4byte	.LASF199
	.byte	0x3
	.2byte	0x2bf
	.4byte	0x60
	.4byte	.LLST135
	.uleb128 0x40
	.ascii	"ret\000"
	.byte	0x3
	.2byte	0x2c1
	.4byte	0x95
	.uleb128 0x40
	.ascii	"lru\000"
	.byte	0x3
	.2byte	0x2c2
	.4byte	0xa0
	.uleb128 0x40
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x2c3
	.4byte	0x8a
	.uleb128 0x26
	.4byte	.LASF200
	.byte	0x3
	.2byte	0x2c3
	.4byte	0x8a
	.uleb128 0x42
	.4byte	.LASF201
	.byte	0x3
	.2byte	0x2c3
	.4byte	0x8a
	.byte	0x1
	.byte	0x58
	.uleb128 0x42
	.4byte	.LASF202
	.byte	0x3
	.2byte	0x2c3
	.4byte	0x8a
	.byte	0x1
	.byte	0x5a
	.uleb128 0x42
	.4byte	.LASF129
	.byte	0x3
	.2byte	0x2c4
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x4c
	.ascii	"end\000"
	.byte	0x3
	.2byte	0x327
	.4byte	.L410
	.uleb128 0x37
	.4byte	.LBB241
	.4byte	.LBE241
	.uleb128 0x41
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x2fe
	.4byte	0x2d
	.4byte	.LLST136
	.uleb128 0x41
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x2fe
	.4byte	0xc4
	.4byte	.LLST137
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF204
	.byte	0x3
	.2byte	0x3a4
	.byte	0x1
	.4byte	0x8a
	.4byte	.LFB168
	.4byte	.LFE168
	.4byte	.LLST138
	.4byte	0x1bb4
	.uleb128 0x3d
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x3a4
	.4byte	0x8a
	.4byte	.LLST139
	.uleb128 0x41
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x3a6
	.4byte	0x8a
	.4byte	.LLST140
	.uleb128 0x3f
	.4byte	.LASF151
	.byte	0x3
	.2byte	0x3a6
	.4byte	0x8a
	.4byte	.LLST141
	.uleb128 0x41
	.ascii	"lpn\000"
	.byte	0x3
	.2byte	0x3a7
	.4byte	0xa0
	.4byte	.LLST142
	.uleb128 0x49
	.ascii	"ppn\000"
	.byte	0x3
	.2byte	0x3a7
	.4byte	0xa0
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x47
	.4byte	0x7af
	.4byte	.LBB242
	.4byte	.LBE242
	.byte	0x3
	.2byte	0x3b1
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x37
	.4byte	.LBB243
	.4byte	.LBE243
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB244
	.4byte	.Ldebug_ranges0+0x4d0
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x46
	.byte	0x1
	.4byte	.LASF205
	.byte	0x3
	.2byte	0x174
	.byte	0x1
	.4byte	.LFB160
	.4byte	.LFE160
	.4byte	.LLST143
	.4byte	0x1c0a
	.uleb128 0x3d
	.4byte	.LASF206
	.byte	0x3
	.2byte	0x174
	.4byte	0x8a
	.4byte	.LLST144
	.uleb128 0x3d
	.4byte	.LASF184
	.byte	0x3
	.2byte	0x174
	.4byte	0xa0
	.4byte	.LLST145
	.uleb128 0x3e
	.ascii	"id\000"
	.byte	0x3
	.2byte	0x174
	.4byte	0x1e3
	.4byte	.LLST146
	.uleb128 0x26
	.4byte	.LASF207
	.byte	0x3
	.2byte	0x176
	.4byte	0x8a
	.byte	0
	.uleb128 0x46
	.byte	0x1
	.4byte	.LASF208
	.byte	0x3
	.2byte	0x4e7
	.byte	0x1
	.4byte	.LFB174
	.4byte	.LFE174
	.4byte	.LLST147
	.4byte	0x1cf6
	.uleb128 0x49
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x4e9
	.4byte	0xa0
	.byte	0x1
	.byte	0x55
	.uleb128 0x41
	.ascii	"lpn\000"
	.byte	0x3
	.2byte	0x4e9
	.4byte	0xa0
	.4byte	.LLST148
	.uleb128 0x42
	.4byte	.LASF209
	.byte	0x3
	.2byte	0x4e9
	.4byte	0xa0
	.byte	0x2
	.byte	0x91
	.sleb128 -56
	.uleb128 0x42
	.4byte	.LASF210
	.byte	0x3
	.2byte	0x4e9
	.4byte	0xa0
	.byte	0x2
	.byte	0x91
	.sleb128 -52
	.uleb128 0x44
	.4byte	0x7af
	.4byte	.LBB250
	.4byte	.Ldebug_ranges0+0x4f0
	.byte	0x3
	.2byte	0x4ee
	.4byte	0x1cb1
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x518
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB252
	.4byte	.Ldebug_ranges0+0x540
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x47
	.4byte	0x82c
	.4byte	.LBB266
	.4byte	.LBE266
	.byte	0x3
	.2byte	0x4f8
	.uleb128 0x34
	.4byte	0x83e
	.uleb128 0x37
	.4byte	.LBB267
	.4byte	.LBE267
	.uleb128 0x32
	.4byte	0x849
	.uleb128 0x32
	.4byte	0x854
	.uleb128 0x32
	.4byte	0x85f
	.uleb128 0x33
	.4byte	0x75d
	.4byte	.LBB268
	.4byte	.Ldebug_ranges0+0x568
	.byte	0x3
	.byte	0x38
	.uleb128 0x34
	.4byte	0x76f
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x46
	.byte	0x1
	.4byte	.LASF211
	.byte	0x3
	.2byte	0x4b7
	.byte	0x1
	.4byte	.LFB173
	.4byte	.LFE173
	.4byte	.LLST149
	.4byte	0x1df3
	.uleb128 0x49
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x4b9
	.4byte	0xa0
	.byte	0x1
	.byte	0x55
	.uleb128 0x40
	.ascii	"lpn\000"
	.byte	0x3
	.2byte	0x4b9
	.4byte	0xa0
	.uleb128 0x42
	.4byte	.LASF209
	.byte	0x3
	.2byte	0x4b9
	.4byte	0xa0
	.byte	0x3
	.byte	0x91
	.sleb128 -76
	.uleb128 0x42
	.4byte	.LASF210
	.byte	0x3
	.2byte	0x4b9
	.4byte	0xa0
	.byte	0x3
	.byte	0x91
	.sleb128 -72
	.uleb128 0x42
	.4byte	.LASF148
	.byte	0x3
	.2byte	0x4ba
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -60
	.uleb128 0x44
	.4byte	0x7af
	.4byte	.LBB276
	.4byte	.Ldebug_ranges0+0x590
	.byte	0x3
	.2byte	0x4c6
	.4byte	0x1dae
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x5b8
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x35
	.4byte	0x7f8
	.4byte	.LLST150
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB278
	.4byte	.Ldebug_ranges0+0x5e0
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x47
	.4byte	0x82c
	.4byte	.LBB290
	.4byte	.LBE290
	.byte	0x3
	.2byte	0x4d3
	.uleb128 0x34
	.4byte	0x83e
	.uleb128 0x37
	.4byte	.LBB291
	.4byte	.LBE291
	.uleb128 0x32
	.4byte	0x849
	.uleb128 0x32
	.4byte	0x854
	.uleb128 0x32
	.4byte	0x85f
	.uleb128 0x33
	.4byte	0x75d
	.4byte	.LBB292
	.4byte	.Ldebug_ranges0+0x600
	.byte	0x3
	.byte	0x38
	.uleb128 0x34
	.4byte	0x76f
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF212
	.byte	0x3
	.2byte	0x331
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB167
	.4byte	.LFE167
	.4byte	.LLST151
	.4byte	0x1f8c
	.uleb128 0x41
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x333
	.4byte	0x8a
	.4byte	.LLST152
	.uleb128 0x41
	.ascii	"j\000"
	.byte	0x3
	.2byte	0x333
	.4byte	0x8a
	.4byte	.LLST153
	.uleb128 0x3f
	.4byte	.LASF213
	.byte	0x3
	.2byte	0x333
	.4byte	0x8a
	.4byte	.LLST154
	.uleb128 0x3f
	.4byte	.LASF214
	.byte	0x3
	.2byte	0x333
	.4byte	0x8a
	.4byte	.LLST155
	.uleb128 0x3f
	.4byte	.LASF215
	.byte	0x3
	.2byte	0x334
	.4byte	0x8a
	.4byte	.LLST156
	.uleb128 0x3f
	.4byte	.LASF68
	.byte	0x3
	.2byte	0x334
	.4byte	0x8a
	.4byte	.LLST157
	.uleb128 0x3f
	.4byte	.LASF57
	.byte	0x3
	.2byte	0x334
	.4byte	0x8a
	.4byte	.LLST158
	.uleb128 0x26
	.4byte	.LASF100
	.byte	0x3
	.2byte	0x334
	.4byte	0x8a
	.uleb128 0x26
	.4byte	.LASF73
	.byte	0x3
	.2byte	0x335
	.4byte	0x75
	.uleb128 0x42
	.4byte	.LASF216
	.byte	0x3
	.2byte	0x336
	.4byte	0x122
	.byte	0x2
	.byte	0x7d
	.sleb128 16
	.uleb128 0x42
	.4byte	.LASF217
	.byte	0x3
	.2byte	0x337
	.4byte	0x122
	.byte	0x2
	.byte	0x7d
	.sleb128 12
	.uleb128 0x26
	.4byte	.LASF144
	.byte	0x3
	.2byte	0x338
	.4byte	0x95
	.uleb128 0x42
	.4byte	.LASF132
	.byte	0x3
	.2byte	0x339
	.4byte	0x3d3
	.byte	0x2
	.byte	0x91
	.sleb128 -48
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x628
	.4byte	0x1ef4
	.uleb128 0x40
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x34d
	.4byte	0x2d
	.uleb128 0x41
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x34d
	.4byte	0xc4
	.4byte	.LLST159
	.byte	0
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x640
	.4byte	0x1f1a
	.uleb128 0x40
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x351
	.4byte	0x2d
	.uleb128 0x41
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x351
	.4byte	0xc4
	.4byte	.LLST160
	.byte	0
	.uleb128 0x44
	.4byte	0x7af
	.4byte	.LBB306
	.4byte	.Ldebug_ranges0+0x668
	.byte	0x3
	.2byte	0x366
	.4byte	0x1f6d
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x690
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x33
	.4byte	0x734
	.4byte	.LBB308
	.4byte	.Ldebug_ranges0+0x6b8
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x6d0
	.uleb128 0x40
	.ascii	"__p\000"
	.byte	0x3
	.2byte	0x37c
	.4byte	0x2d
	.uleb128 0x40
	.ascii	"__n\000"
	.byte	0x3
	.2byte	0x37c
	.4byte	0xc4
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF218
	.byte	0x3
	.2byte	0x418
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB171
	.4byte	.LFE171
	.4byte	.LLST161
	.4byte	0x20b2
	.uleb128 0x3d
	.4byte	.LASF219
	.byte	0x3
	.2byte	0x418
	.4byte	0x5d4
	.4byte	.LLST162
	.uleb128 0x41
	.ascii	"i\000"
	.byte	0x3
	.2byte	0x41a
	.4byte	0x8a
	.4byte	.LLST163
	.uleb128 0x3f
	.4byte	.LASF220
	.byte	0x3
	.2byte	0x41a
	.4byte	0x8a
	.4byte	.LLST164
	.uleb128 0x26
	.4byte	.LASF221
	.byte	0x3
	.2byte	0x41a
	.4byte	0x8a
	.uleb128 0x41
	.ascii	"lpn\000"
	.byte	0x3
	.2byte	0x41b
	.4byte	0xa0
	.4byte	.LLST165
	.uleb128 0x49
	.ascii	"ppn\000"
	.byte	0x3
	.2byte	0x41b
	.4byte	0xa0
	.byte	0x3
	.byte	0x91
	.sleb128 -72
	.uleb128 0x26
	.4byte	.LASF222
	.byte	0x3
	.2byte	0x41b
	.4byte	0xa0
	.uleb128 0x26
	.4byte	.LASF151
	.byte	0x3
	.2byte	0x41b
	.4byte	0xa0
	.uleb128 0x42
	.4byte	.LASF148
	.byte	0x3
	.2byte	0x41c
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -60
	.uleb128 0x42
	.4byte	.LASF156
	.byte	0x3
	.2byte	0x41d
	.4byte	0x75
	.byte	0x2
	.byte	0x7d
	.sleb128 8
	.uleb128 0x3f
	.4byte	.LASF223
	.byte	0x3
	.2byte	0x41e
	.4byte	0x75
	.4byte	.LLST166
	.uleb128 0x4a
	.4byte	.LASF224
	.byte	0x3
	.2byte	0x420
	.uleb128 0x4b
	.4byte	.LASF225
	.byte	0x3
	.2byte	0x46d
	.4byte	.L502
	.uleb128 0x47
	.4byte	0x7af
	.4byte	.LBB320
	.4byte	.LBE320
	.byte	0x3
	.2byte	0x440
	.uleb128 0x34
	.4byte	0x7cc
	.uleb128 0x34
	.4byte	0x7c1
	.uleb128 0x37
	.4byte	.LBB321
	.4byte	.LBE321
	.uleb128 0x32
	.4byte	0x7d7
	.uleb128 0x32
	.4byte	0x7e2
	.uleb128 0x32
	.4byte	0x7ed
	.uleb128 0x32
	.4byte	0x7f8
	.uleb128 0x36
	.4byte	0x734
	.4byte	.LBB322
	.4byte	.LBE322
	.byte	0x3
	.byte	0x29
	.uleb128 0x34
	.4byte	0x751
	.uleb128 0x34
	.4byte	0x746
	.byte	0
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF226
	.byte	0x3
	.2byte	0x494
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB172
	.4byte	.LFE172
	.4byte	.LLST167
	.4byte	0x20e1
	.uleb128 0x3d
	.4byte	.LASF227
	.byte	0x3
	.2byte	0x494
	.4byte	0x5af
	.4byte	.LLST168
	.byte	0
	.uleb128 0x4d
	.byte	0x1
	.4byte	.LASF229
	.byte	0x1
	.2byte	0x1d5
	.byte	0x1
	.4byte	0x95
	.4byte	.LFB141
	.4byte	.LFE141
	.byte	0x2
	.byte	0x7d
	.sleb128 0
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF230
	.byte	0x1
	.byte	0xe0
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB132
	.4byte	.LFE132
	.4byte	.LLST169
	.4byte	0x21f9
	.uleb128 0x2b
	.4byte	.LASF57
	.byte	0x1
	.byte	0xe0
	.4byte	0xa0
	.4byte	.LLST170
	.uleb128 0x2b
	.4byte	.LASF231
	.byte	0x1
	.byte	0xe0
	.4byte	0xa0
	.4byte	.LLST171
	.uleb128 0x31
	.ascii	"buf\000"
	.byte	0x1
	.byte	0xe0
	.4byte	0x723
	.4byte	.LLST172
	.uleb128 0x1e
	.ascii	"ret\000"
	.byte	0x1
	.byte	0xe2
	.4byte	0x60
	.4byte	.LLST173
	.uleb128 0x3b
	.4byte	.LASF232
	.byte	0x1
	.byte	0xe3
	.4byte	0xa0
	.4byte	.LLST174
	.uleb128 0x2d
	.4byte	.LASF233
	.byte	0x1
	.byte	0xe3
	.4byte	0xa0
	.byte	0x2
	.byte	0x7d
	.sleb128 12
	.uleb128 0x1e
	.ascii	"lpa\000"
	.byte	0x1
	.byte	0xe4
	.4byte	0xa0
	.4byte	.LLST175
	.uleb128 0x1e
	.ascii	"ppa\000"
	.byte	0x1
	.byte	0xe4
	.4byte	0xa0
	.4byte	.LLST176
	.uleb128 0x3b
	.4byte	.LASF234
	.byte	0x1
	.byte	0xe5
	.4byte	0xa0
	.4byte	.LLST177
	.uleb128 0x3b
	.4byte	.LASF235
	.byte	0x1
	.byte	0xe6
	.4byte	0xa0
	.4byte	.LLST178
	.uleb128 0x19
	.4byte	.LASF236
	.byte	0x1
	.byte	0xe6
	.4byte	0xa0
	.uleb128 0x39
	.ascii	"req\000"
	.byte	0x1
	.byte	0xe7
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -60
	.uleb128 0x19
	.4byte	.LASF100
	.byte	0x1
	.byte	0xe8
	.4byte	0x8a
	.uleb128 0x37
	.4byte	.LBB324
	.4byte	.LBE324
	.uleb128 0x1e
	.ascii	"__p\000"
	.byte	0x1
	.byte	0xf8
	.4byte	0x2d
	.4byte	.LLST179
	.uleb128 0x1e
	.ascii	"__n\000"
	.byte	0x1
	.byte	0xf8
	.4byte	0xc4
	.4byte	.LLST180
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF237
	.byte	0x1
	.2byte	0x12f
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB133
	.4byte	.LFE133
	.4byte	.LLST181
	.4byte	0x22fc
	.uleb128 0x3d
	.4byte	.LASF57
	.byte	0x1
	.2byte	0x12f
	.4byte	0xa0
	.4byte	.LLST182
	.uleb128 0x3d
	.4byte	.LASF231
	.byte	0x1
	.2byte	0x12f
	.4byte	0xa0
	.4byte	.LLST183
	.uleb128 0x3e
	.ascii	"buf\000"
	.byte	0x1
	.2byte	0x12f
	.4byte	0x723
	.4byte	.LLST184
	.uleb128 0x41
	.ascii	"ret\000"
	.byte	0x1
	.2byte	0x131
	.4byte	0x60
	.4byte	.LLST185
	.uleb128 0x3f
	.4byte	.LASF232
	.byte	0x1
	.2byte	0x132
	.4byte	0xa0
	.4byte	.LLST186
	.uleb128 0x42
	.4byte	.LASF233
	.byte	0x1
	.2byte	0x132
	.4byte	0xa0
	.byte	0x2
	.byte	0x7d
	.sleb128 16
	.uleb128 0x41
	.ascii	"lpa\000"
	.byte	0x1
	.2byte	0x133
	.4byte	0xa0
	.4byte	.LLST187
	.uleb128 0x41
	.ascii	"ppa\000"
	.byte	0x1
	.2byte	0x133
	.4byte	0xa0
	.4byte	.LLST188
	.uleb128 0x3f
	.4byte	.LASF234
	.byte	0x1
	.2byte	0x134
	.4byte	0xa0
	.4byte	.LLST189
	.uleb128 0x3f
	.4byte	.LASF235
	.byte	0x1
	.2byte	0x135
	.4byte	0xa0
	.4byte	.LLST190
	.uleb128 0x26
	.4byte	.LASF236
	.byte	0x1
	.2byte	0x135
	.4byte	0xa0
	.uleb128 0x49
	.ascii	"req\000"
	.byte	0x1
	.2byte	0x136
	.4byte	0xcf
	.byte	0x2
	.byte	0x91
	.sleb128 -60
	.uleb128 0x37
	.4byte	.LBB325
	.4byte	.LBE325
	.uleb128 0x41
	.ascii	"__p\000"
	.byte	0x1
	.2byte	0x148
	.4byte	0x2d
	.4byte	.LLST191
	.uleb128 0x41
	.ascii	"__n\000"
	.byte	0x1
	.2byte	0x148
	.4byte	0xc4
	.4byte	.LLST192
	.byte	0
	.byte	0
	.uleb128 0x3c
	.byte	0x1
	.4byte	.LASF238
	.byte	0x3
	.2byte	0x502
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB175
	.4byte	.LFE175
	.4byte	.LLST193
	.4byte	0x2336
	.uleb128 0x26
	.4byte	.LASF144
	.byte	0x3
	.2byte	0x504
	.4byte	0x95
	.uleb128 0x42
	.4byte	.LASF132
	.byte	0x3
	.2byte	0x505
	.4byte	0x3d3
	.byte	0x2
	.byte	0x91
	.sleb128 -24
	.byte	0
	.uleb128 0x22
	.byte	0x1
	.4byte	.LASF239
	.byte	0x1
	.byte	0xd6
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB131
	.4byte	.LFE131
	.4byte	.LLST194
	.uleb128 0x22
	.byte	0x1
	.4byte	.LASF240
	.byte	0x1
	.byte	0xab
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB129
	.4byte	.LFE129
	.4byte	.LLST195
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF241
	.byte	0x1
	.byte	0x7d
	.byte	0x1
	.4byte	0xa0
	.4byte	.LFB128
	.4byte	.LFE128
	.4byte	.LLST196
	.4byte	0x23cb
	.uleb128 0x18
	.ascii	"i\000"
	.byte	0x1
	.byte	0x7f
	.4byte	0x60
	.uleb128 0x1f
	.4byte	.Ldebug_ranges0+0x6e8
	.4byte	0x23ae
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x99
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x99
	.4byte	0xc4
	.byte	0
	.uleb128 0x21
	.4byte	.Ldebug_ranges0+0x710
	.uleb128 0x18
	.ascii	"__p\000"
	.byte	0x1
	.byte	0x9e
	.4byte	0x2d
	.uleb128 0x18
	.ascii	"__n\000"
	.byte	0x1
	.byte	0x9e
	.4byte	0xc4
	.byte	0
	.byte	0
	.uleb128 0x4e
	.byte	0x1
	.4byte	.LASF242
	.byte	0x1
	.2byte	0x188
	.byte	0x1
	.4byte	0xa0
	.4byte	.LFB135
	.4byte	.LFE135
	.4byte	.LLST197
	.uleb128 0x1d
	.byte	0x1
	.4byte	.LASF243
	.byte	0x1
	.byte	0xbb
	.byte	0x1
	.4byte	0x60
	.4byte	.LFB130
	.4byte	.LFE130
	.4byte	.LLST198
	.4byte	0x240a
	.uleb128 0x2e
	.4byte	.LASF244
	.byte	0x1
	.byte	0xc2
	.byte	0
	.uleb128 0x4f
	.4byte	.LASF245
	.byte	0x9
	.byte	0xc
	.4byte	0x36
	.byte	0x1
	.byte	0x1
	.uleb128 0x10
	.4byte	0x60
	.4byte	0x2422
	.uleb128 0x50
	.byte	0
	.uleb128 0x4f
	.4byte	.LASF246
	.byte	0xa
	.byte	0x24
	.4byte	0x2417
	.byte	0x1
	.byte	0x1
	.uleb128 0x10
	.4byte	0x3d
	.4byte	0x243a
	.uleb128 0x50
	.byte	0
	.uleb128 0x51
	.4byte	.LASF247
	.byte	0xb
	.2byte	0x1bf
	.4byte	0x2448
	.byte	0x1
	.byte	0x1
	.uleb128 0x52
	.4byte	0x242f
	.uleb128 0x4f
	.4byte	.LASF248
	.byte	0x8
	.byte	0x9a
	.4byte	0x128
	.byte	0x1
	.byte	0x1
	.uleb128 0x53
	.4byte	.LASF249
	.byte	0x8
	.byte	0xa3
	.4byte	0x259
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	sys_info
	.uleb128 0x53
	.4byte	.LASF250
	.byte	0x8
	.byte	0xa4
	.4byte	0x3e3
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	bbt_info
	.uleb128 0x53
	.4byte	.LASF251
	.byte	0x8
	.byte	0xa5
	.4byte	0x2490
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_sblk_info_buf
	.uleb128 0xa
	.byte	0x4
	.4byte	0x208
	.uleb128 0x53
	.4byte	.LASF252
	.byte	0x8
	.byte	0xa6
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_sys_data_buf
	.uleb128 0x53
	.4byte	.LASF253
	.byte	0x8
	.byte	0xa7
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_sys_spare_buf
	.uleb128 0x53
	.4byte	.LASF254
	.byte	0x8
	.byte	0xa8
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_io_data_buf
	.uleb128 0x53
	.4byte	.LASF255
	.byte	0x8
	.byte	0xa9
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_io_spare_buf
	.uleb128 0x53
	.4byte	.LASF256
	.byte	0x8
	.byte	0xab
	.4byte	0xcf
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	req_sys
	.uleb128 0x53
	.4byte	.LASF257
	.byte	0x8
	.byte	0xac
	.4byte	0x2502
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	req_erase
	.uleb128 0xa
	.byte	0x4
	.4byte	0xcf
	.uleb128 0x53
	.4byte	.LASF258
	.byte	0x8
	.byte	0xae
	.4byte	0x4fb
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	l2p_info
	.uleb128 0x53
	.4byte	.LASF259
	.byte	0x8
	.byte	0xaf
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_l2p_sblk_num
	.uleb128 0x53
	.4byte	.LASF260
	.byte	0x8
	.byte	0xb0
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_l2p_trigger_offs
	.uleb128 0x53
	.4byte	.LASF261
	.byte	0x8
	.byte	0xb1
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_l2p_index_num
	.uleb128 0x53
	.4byte	.LASF262
	.byte	0x8
	.byte	0xb2
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_l2p_cache_num
	.uleb128 0x53
	.4byte	.LASF263
	.byte	0x8
	.byte	0xb4
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_l2p_index_ppn
	.uleb128 0x53
	.4byte	.LASF264
	.byte	0x8
	.byte	0xb5
	.4byte	0x2586
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_l2p_cache_info
	.uleb128 0xa
	.byte	0x4
	.4byte	0x4b6
	.uleb128 0x53
	.4byte	.LASF265
	.byte	0x8
	.byte	0xb6
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_l2p_cache_buf
	.uleb128 0x53
	.4byte	.LASF266
	.byte	0x8
	.byte	0xb7
	.4byte	0x1dd
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_l2p_trace_list
	.uleb128 0x53
	.4byte	.LASF267
	.byte	0x8
	.byte	0xb8
	.4byte	0x1dd
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_l2p_cur_trace_list
	.uleb128 0x53
	.4byte	.LASF268
	.byte	0x8
	.byte	0xb9
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_l2p_data_buf
	.uleb128 0x53
	.4byte	.LASF269
	.byte	0x8
	.byte	0xba
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_l2p_spare_buf
	.uleb128 0x53
	.4byte	.LASF270
	.byte	0x8
	.byte	0xbc
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_ftl_rev_blk_num
	.uleb128 0x53
	.4byte	.LASF271
	.byte	0x8
	.byte	0xbd
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_blk_start
	.uleb128 0x53
	.4byte	.LASF272
	.byte	0x8
	.byte	0xbe
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_page_size
	.uleb128 0x53
	.4byte	.LASF273
	.byte	0x8
	.byte	0xbf
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_sec_per_page
	.uleb128 0x53
	.4byte	.LASF274
	.byte	0x8
	.byte	0xc0
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_plane_num
	.uleb128 0x53
	.4byte	.LASF275
	.byte	0x8
	.byte	0xc1
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_plane_per_die
	.uleb128 0x53
	.4byte	.LASF276
	.byte	0x8
	.byte	0xc2
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_blk_per_plane
	.uleb128 0x53
	.4byte	.LASF277
	.byte	0x8
	.byte	0xc3
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_blk_num
	.uleb128 0x53
	.4byte	.LASF278
	.byte	0x8
	.byte	0xc4
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_page_num
	.uleb128 0x53
	.4byte	.LASF279
	.byte	0x8
	.byte	0xc5
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_nand_page_per_blk
	.uleb128 0x53
	.4byte	.LASF280
	.byte	0x8
	.byte	0xc6
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_page_per_sblk
	.uleb128 0x53
	.4byte	.LASF281
	.byte	0x8
	.byte	0xc7
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_ftl_sblk_start
	.uleb128 0x10
	.4byte	0x8a
	.4byte	0x26ce
	.uleb128 0x12
	.4byte	0x36
	.byte	0x3
	.byte	0
	.uleb128 0x53
	.4byte	.LASF282
	.byte	0x8
	.byte	0xc8
	.4byte	0x26be
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	c_plane_base_pblk
	.uleb128 0x10
	.4byte	0x8a
	.4byte	0x26f0
	.uleb128 0x12
	.4byte	0x36
	.byte	0xb
	.byte	0
	.uleb128 0x53
	.4byte	.LASF283
	.byte	0x8
	.byte	0xca
	.4byte	0x26e0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_blk_list
	.uleb128 0x53
	.4byte	.LASF284
	.byte	0x8
	.byte	0xcb
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_blk_num
	.uleb128 0x53
	.4byte	.LASF285
	.byte	0x8
	.byte	0xcc
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_cur_blk
	.uleb128 0x53
	.4byte	.LASF286
	.byte	0x8
	.byte	0xcd
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_cur_blk_index
	.uleb128 0x53
	.4byte	.LASF287
	.byte	0x8
	.byte	0xce
	.4byte	0x8a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_cur_page
	.uleb128 0x53
	.4byte	.LASF288
	.byte	0x8
	.byte	0xcf
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_page_write_bitmap
	.uleb128 0x53
	.4byte	.LASF289
	.byte	0x8
	.byte	0xd0
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_page_last_dirty_index
	.uleb128 0x53
	.4byte	.LASF290
	.byte	0x8
	.byte	0xd2
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_max_lba_sector
	.uleb128 0x53
	.4byte	.LASF291
	.byte	0x8
	.byte	0xd3
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_max_lpn
	.uleb128 0x53
	.4byte	.LASF292
	.byte	0x8
	.byte	0xd5
	.4byte	0x56a
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	user_info
	.uleb128 0x53
	.4byte	.LASF293
	.byte	0x8
	.byte	0xd6
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_user_trace_list
	.uleb128 0x53
	.4byte	.LASF294
	.byte	0x8
	.byte	0xd7
	.4byte	0x122
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	p_user_cur_trace_list
	.uleb128 0x10
	.4byte	0xcf
	.4byte	0x27d8
	.uleb128 0x12
	.4byte	0x36
	.byte	0x1f
	.byte	0
	.uleb128 0x53
	.4byte	.LASF295
	.byte	0x8
	.byte	0xda
	.4byte	0x27c8
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	trace_req
	.uleb128 0x10
	.4byte	0xa0
	.4byte	0x2800
	.uleb128 0x12
	.4byte	0x36
	.byte	0x1f
	.uleb128 0x12
	.4byte	0x36
	.byte	0x1
	.byte	0
	.uleb128 0x53
	.4byte	.LASF296
	.byte	0x8
	.byte	0xdb
	.4byte	0x27ea
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	trace_spare_buf
	.uleb128 0x53
	.4byte	.LASF297
	.byte	0x8
	.byte	0xdc
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	ftl_read_cnt
	.uleb128 0x53
	.4byte	.LASF298
	.byte	0x8
	.byte	0xdd
	.4byte	0xa0
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	ftl_prog_cnt
	.uleb128 0x53
	.4byte	.LASF299
	.byte	0x8
	.byte	0xe0
	.4byte	0x75
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_suddenly_power_lost
	.uleb128 0x53
	.4byte	.LASF300
	.byte	0x8
	.byte	0xe1
	.4byte	0x75
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_sys_power_lost_rewrite
	.uleb128 0x53
	.4byte	.LASF301
	.byte	0x8
	.byte	0xe2
	.4byte	0x75
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_l2p_power_lost_rewrite
	.uleb128 0x53
	.4byte	.LASF302
	.byte	0x8
	.byte	0xe3
	.4byte	0x75
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	g_user_power_lost_rewrite
	.uleb128 0x53
	.4byte	.LASF303
	.byte	0x8
	.byte	0xe5
	.4byte	0x5f3
	.byte	0x1
	.byte	0x5
	.byte	0x3
	.4byte	user_gc
	.byte	0
	.section	.debug_abbrev,"",%progbits
.Ldebug_abbrev0:
	.uleb128 0x1
	.uleb128 0x11
	.byte	0x1
	.uleb128 0x25
	.uleb128 0xe
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1b
	.uleb128 0xe
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x10
	.uleb128 0x6
	.uleb128 0x2134
	.uleb128 0x6
	.uleb128 0x2135
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x2
	.uleb128 0xf
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x3
	.uleb128 0x24
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0xe
	.byte	0
	.byte	0
	.uleb128 0x4
	.uleb128 0x24
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3e
	.uleb128 0xb
	.uleb128 0x3
	.uleb128 0x8
	.byte	0
	.byte	0
	.uleb128 0x5
	.uleb128 0x16
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x6
	.uleb128 0x16
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x7
	.uleb128 0x13
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x8
	.uleb128 0xd
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x38
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x9
	.uleb128 0xd
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x38
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0xa
	.uleb128 0xf
	.byte	0
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xb
	.uleb128 0x4
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0xc
	.uleb128 0x28
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x1c
	.uleb128 0xd
	.byte	0
	.byte	0
	.uleb128 0xd
	.uleb128 0xd
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0xd
	.uleb128 0xb
	.uleb128 0xc
	.uleb128 0xb
	.uleb128 0x38
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0xe
	.uleb128 0xd
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0xd
	.uleb128 0xb
	.uleb128 0xc
	.uleb128 0xb
	.uleb128 0x38
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0xf
	.uleb128 0x13
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0x5
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x10
	.uleb128 0x1
	.byte	0x1
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x11
	.uleb128 0x21
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2f
	.uleb128 0x5
	.byte	0
	.byte	0
	.uleb128 0x12
	.uleb128 0x21
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2f
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x13
	.uleb128 0x13
	.byte	0x1
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0xb
	.uleb128 0xb
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x14
	.uleb128 0xd
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x38
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x15
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x20
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x16
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x20
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x17
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x18
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x19
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x1a
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x20
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x1b
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x1c
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x1d
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x1e
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x1f
	.uleb128 0xb
	.byte	0x1
	.uleb128 0x55
	.uleb128 0x6
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x20
	.uleb128 0xb
	.byte	0x1
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x21
	.uleb128 0xb
	.byte	0x1
	.uleb128 0x55
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x22
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x23
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x24
	.uleb128 0x5
	.byte	0
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x25
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x26
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x27
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x28
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x29
	.uleb128 0x2e
	.byte	0
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x2a
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x2b
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x2c
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x2d
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x2e
	.uleb128 0xa
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x2f
	.uleb128 0x5
	.byte	0
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x30
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x31
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x32
	.uleb128 0x34
	.byte	0
	.uleb128 0x31
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x33
	.uleb128 0x1d
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x52
	.uleb128 0x1
	.uleb128 0x55
	.uleb128 0x6
	.uleb128 0x58
	.uleb128 0xb
	.uleb128 0x59
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x34
	.uleb128 0x5
	.byte	0
	.uleb128 0x31
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x35
	.uleb128 0x34
	.byte	0
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x36
	.uleb128 0x1d
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x58
	.uleb128 0xb
	.uleb128 0x59
	.uleb128 0xb
	.byte	0
	.byte	0
	.uleb128 0x37
	.uleb128 0xb
	.byte	0x1
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.byte	0
	.byte	0
	.uleb128 0x38
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x39
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x3a
	.uleb128 0x1d
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x52
	.uleb128 0x1
	.uleb128 0x55
	.uleb128 0x6
	.uleb128 0x58
	.uleb128 0xb
	.uleb128 0x59
	.uleb128 0xb
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x3b
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x3c
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x3d
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x3e
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x3f
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x40
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x41
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x42
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x43
	.uleb128 0x1d
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x52
	.uleb128 0x1
	.uleb128 0x55
	.uleb128 0x6
	.uleb128 0x58
	.uleb128 0xb
	.uleb128 0x59
	.uleb128 0x5
	.byte	0
	.byte	0
	.uleb128 0x44
	.uleb128 0x1d
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x52
	.uleb128 0x1
	.uleb128 0x55
	.uleb128 0x6
	.uleb128 0x58
	.uleb128 0xb
	.uleb128 0x59
	.uleb128 0x5
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x45
	.uleb128 0x5
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x46
	.uleb128 0x2e
	.byte	0x1
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x47
	.uleb128 0x1d
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x58
	.uleb128 0xb
	.uleb128 0x59
	.uleb128 0x5
	.byte	0
	.byte	0
	.uleb128 0x48
	.uleb128 0x1d
	.byte	0x1
	.uleb128 0x31
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x58
	.uleb128 0xb
	.uleb128 0x59
	.uleb128 0x5
	.uleb128 0x1
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x49
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x4a
	.uleb128 0xa
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.byte	0
	.byte	0
	.uleb128 0x4b
	.uleb128 0xa
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x11
	.uleb128 0x1
	.byte	0
	.byte	0
	.uleb128 0x4c
	.uleb128 0xa
	.byte	0
	.uleb128 0x3
	.uleb128 0x8
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x11
	.uleb128 0x1
	.byte	0
	.byte	0
	.uleb128 0x4d
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0xa
	.byte	0
	.byte	0
	.uleb128 0x4e
	.uleb128 0x2e
	.byte	0
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x27
	.uleb128 0xc
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x11
	.uleb128 0x1
	.uleb128 0x12
	.uleb128 0x1
	.uleb128 0x40
	.uleb128 0x6
	.byte	0
	.byte	0
	.uleb128 0x4f
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3c
	.uleb128 0xc
	.byte	0
	.byte	0
	.uleb128 0x50
	.uleb128 0x21
	.byte	0
	.byte	0
	.byte	0
	.uleb128 0x51
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0x5
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x3c
	.uleb128 0xc
	.byte	0
	.byte	0
	.uleb128 0x52
	.uleb128 0x26
	.byte	0
	.uleb128 0x49
	.uleb128 0x13
	.byte	0
	.byte	0
	.uleb128 0x53
	.uleb128 0x34
	.byte	0
	.uleb128 0x3
	.uleb128 0xe
	.uleb128 0x3a
	.uleb128 0xb
	.uleb128 0x3b
	.uleb128 0xb
	.uleb128 0x49
	.uleb128 0x13
	.uleb128 0x3f
	.uleb128 0xc
	.uleb128 0x2
	.uleb128 0xa
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_loc,"",%progbits
.Ldebug_loc0:
.LLST0:
	.4byte	.LFB124-.Ltext0
	.4byte	.LCFI0-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI0-.Ltext0
	.4byte	.LFE124-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST1:
	.4byte	.LVL0-.Ltext0
	.4byte	.LVL1-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST2:
	.4byte	.LVL2-.Ltext0
	.4byte	.LVL3-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL3-.Ltext0
	.4byte	.LFE124-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST3:
	.4byte	.LFB125-.Ltext0
	.4byte	.LCFI1-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI1-.Ltext0
	.4byte	.LFE125-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 16
	.4byte	0
	.4byte	0
.LLST4:
	.4byte	.LVL7-.Ltext0
	.4byte	.LVL8-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST5:
	.4byte	.LVL4-.Ltext0
	.4byte	.LVL5-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL5-.Ltext0
	.4byte	.LVL6-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST6:
	.4byte	.LVL9-.Ltext0
	.4byte	.LVL10-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL10-.Ltext0
	.4byte	.LVL11-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST7:
	.4byte	.LVL12-.Ltext0
	.4byte	.LVL13-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL13-.Ltext0
	.4byte	.LVL14-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST8:
	.4byte	.LVL15-.Ltext0
	.4byte	.LVL16-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL16-.Ltext0
	.4byte	.LVL17-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST9:
	.4byte	.LVL18-.Ltext0
	.4byte	.LVL19-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL19-.Ltext0
	.4byte	.LFE125-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST10:
	.4byte	.LFB126-.Ltext0
	.4byte	.LCFI2-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI2-.Ltext0
	.4byte	.LFE126-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 16
	.4byte	0
	.4byte	0
.LLST11:
	.4byte	.LVL20-.Ltext0
	.4byte	.LVL21-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST12:
	.4byte	.LFB137-.Ltext0
	.4byte	.LCFI3-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI3-.Ltext0
	.4byte	.LFE137-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST13:
	.4byte	.LFB140-.Ltext0
	.4byte	.LCFI4-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI4-.Ltext0
	.4byte	.LFE140-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST14:
	.4byte	.LVL22-.Ltext0
	.4byte	.LVL23-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL24-.Ltext0
	.4byte	.LVL25-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST15:
	.4byte	.LVL26-.Ltext0
	.4byte	.LVL27-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL28-.Ltext0
	.4byte	.LVL29-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST16:
	.4byte	.LFB145-.Ltext0
	.4byte	.LCFI5-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI5-.Ltext0
	.4byte	.LFE145-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST17:
	.4byte	.LVL30-.Ltext0
	.4byte	.LVL33-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL33-.Ltext0
	.4byte	.LFE145-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST18:
	.4byte	.LVL30-.Ltext0
	.4byte	.LVL31-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL31-.Ltext0
	.4byte	.LFE145-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST19:
	.4byte	.LVL32-.Ltext0
	.4byte	.LVL34-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL34-.Ltext0
	.4byte	.LVL35-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST20:
	.4byte	.LFB147-.Ltext0
	.4byte	.LCFI6-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI6-.Ltext0
	.4byte	.LFE147-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 40
	.4byte	0
	.4byte	0
.LLST21:
	.4byte	.LFB146-.Ltext0
	.4byte	.LCFI7-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI7-.Ltext0
	.4byte	.LCFI8-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 28
	.4byte	.LCFI8-.Ltext0
	.4byte	.LFE146-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 72
	.4byte	0
	.4byte	0
.LLST22:
	.4byte	.LVL37-.Ltext0
	.4byte	.LVL43-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL44-.Ltext0
	.4byte	.LVL45-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST23:
	.4byte	.LVL38-.Ltext0
	.4byte	.LVL40-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL40-.Ltext0
	.4byte	.LVL41-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST24:
	.4byte	.LVL39-.Ltext0
	.4byte	.LVL40-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL40-.Ltext0
	.4byte	.LVL42-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST25:
	.4byte	.LVL46-.Ltext0
	.4byte	.LVL47-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST26:
	.4byte	.LFB149-.Ltext0
	.4byte	.LCFI9-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI9-.Ltext0
	.4byte	.LFE149-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST27:
	.4byte	.LVL48-.Ltext0
	.4byte	.LVL49-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST28:
	.4byte	.LFB150-.Ltext0
	.4byte	.LCFI10-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI10-.Ltext0
	.4byte	.LFE150-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST29:
	.4byte	.LVL50-.Ltext0
	.4byte	.LVL51-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST30:
	.4byte	.LFB151-.Ltext0
	.4byte	.LCFI11-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI11-.Ltext0
	.4byte	.LFE151-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST31:
	.4byte	.LVL52-.Ltext0
	.4byte	.LVL53-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL53-.Ltext0
	.4byte	.LFE151-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST32:
	.4byte	.LVL52-.Ltext0
	.4byte	.LVL54-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL54-.Ltext0
	.4byte	.LFE151-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST33:
	.4byte	.LFB152-.Ltext0
	.4byte	.LCFI12-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI12-.Ltext0
	.4byte	.LFE152-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 16
	.4byte	0
	.4byte	0
.LLST34:
	.4byte	.LVL55-.Ltext0
	.4byte	.LVL57-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST35:
	.4byte	.LFB153-.Ltext0
	.4byte	.LCFI13-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI13-.Ltext0
	.4byte	.LFE153-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST36:
	.4byte	.LVL58-.Ltext0
	.4byte	.LVL59-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL59-.Ltext0
	.4byte	.LFE153-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST37:
	.4byte	.LVL60-.Ltext0
	.4byte	.LVL62-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL64-.Ltext0
	.4byte	.LFE153-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST38:
	.4byte	.LVL61-.Ltext0
	.4byte	.LVL63-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL65-.Ltext0
	.4byte	.LFE153-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST39:
	.4byte	.LFB154-.Ltext0
	.4byte	.LCFI14-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI14-.Ltext0
	.4byte	.LCFI15-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI15-.Ltext0
	.4byte	.LFE154-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 72
	.4byte	0
	.4byte	0
.LLST40:
	.4byte	.LVL66-.Ltext0
	.4byte	.LVL68-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL68-1-.Ltext0
	.4byte	.LFE154-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST41:
	.4byte	.LVL70-.Ltext0
	.4byte	.LVL73-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL74-.Ltext0
	.4byte	.LFE154-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST42:
	.4byte	.LVL70-.Ltext0
	.4byte	.LVL71-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL72-.Ltext0
	.4byte	.LVL73-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL75-.Ltext0
	.4byte	.LFE154-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST43:
	.4byte	.LFB155-.Ltext0
	.4byte	.LCFI16-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI16-.Ltext0
	.4byte	.LCFI17-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	.LCFI17-.Ltext0
	.4byte	.LFE155-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 56
	.4byte	0
	.4byte	0
.LLST44:
	.4byte	.LVL76-.Ltext0
	.4byte	.LVL77-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST45:
	.4byte	.LVL76-.Ltext0
	.4byte	.LVL78-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST46:
	.4byte	.LVL76-.Ltext0
	.4byte	.LVL82-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL82-1-.Ltext0
	.4byte	.LFE155-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST47:
	.4byte	.LVL83-.Ltext0
	.4byte	.LVL84-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL91-.Ltext0
	.4byte	.LVL92-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL93-.Ltext0
	.4byte	.LVL94-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL94-.Ltext0
	.4byte	.LFE155-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST48:
	.4byte	.LVL79-.Ltext0
	.4byte	.LVL80-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL80-.Ltext0
	.4byte	.LVL81-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL83-.Ltext0
	.4byte	.LVL86-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL89-.Ltext0
	.4byte	.LVL92-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL93-.Ltext0
	.4byte	.LVL95-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL95-.Ltext0
	.4byte	.LFE155-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST49:
	.4byte	.LVL85-.Ltext0
	.4byte	.LVL86-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL86-.Ltext0
	.4byte	.LVL87-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL88-.Ltext0
	.4byte	.LVL90-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL93-.Ltext0
	.4byte	.LVL94-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST50:
	.4byte	.LFB144-.Ltext0
	.4byte	.LCFI18-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI18-.Ltext0
	.4byte	.LFE144-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 48
	.4byte	0
	.4byte	0
.LLST51:
	.4byte	.LVL96-.Ltext0
	.4byte	.LVL97-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL101-.Ltext0
	.4byte	.LVL102-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL103-.Ltext0
	.4byte	.LVL104-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL109-.Ltext0
	.4byte	.LFE144-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST52:
	.4byte	.LVL98-.Ltext0
	.4byte	.LVL101-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL104-.Ltext0
	.4byte	.LFE144-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST53:
	.4byte	.LVL100-.Ltext0
	.4byte	.LVL101-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL104-.Ltext0
	.4byte	.LVL105-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL106-.Ltext0
	.4byte	.LVL107-1-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL109-.Ltext0
	.4byte	.LFE144-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST54:
	.4byte	.LVL99-.Ltext0
	.4byte	.LVL101-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL104-.Ltext0
	.4byte	.LVL108-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL109-.Ltext0
	.4byte	.LFE144-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST55:
	.4byte	.LFB157-.Ltext0
	.4byte	.LCFI19-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI19-.Ltext0
	.4byte	.LCFI20-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI20-.Ltext0
	.4byte	.LFE157-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 72
	.4byte	0
	.4byte	0
.LLST56:
	.4byte	.LVL110-.Ltext0
	.4byte	.LVL111-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL111-.Ltext0
	.4byte	.LFE157-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST57:
	.4byte	.LVL110-.Ltext0
	.4byte	.LVL116-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL116-.Ltext0
	.4byte	.LFE157-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST58:
	.4byte	.LVL121-.Ltext0
	.4byte	.LVL124-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL126-.Ltext0
	.4byte	.LVL134-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL138-.Ltext0
	.4byte	.LVL139-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL140-.Ltext0
	.4byte	.LFE157-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST59:
	.4byte	.LVL120-.Ltext0
	.4byte	.LVL125-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL126-.Ltext0
	.4byte	.LVL127-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL129-.Ltext0
	.4byte	.LVL130-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL143-.Ltext0
	.4byte	.LVL144-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST60:
	.4byte	.LVL123-.Ltext0
	.4byte	.LVL135-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL136-.Ltext0
	.4byte	.LVL137-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL138-.Ltext0
	.4byte	.LFE157-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST61:
	.4byte	.LVL132-.Ltext0
	.4byte	.LVL141-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL142-.Ltext0
	.4byte	.LFE157-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST62:
	.4byte	.LVL122-.Ltext0
	.4byte	.LVL128-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL129-.Ltext0
	.4byte	.LVL131-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST63:
	.4byte	.LVL111-.Ltext0
	.4byte	.LVL113-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL115-.Ltext0
	.4byte	.LVL117-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL117-.Ltext0
	.4byte	.LVL118-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST64:
	.4byte	.LVL112-.Ltext0
	.4byte	.LVL113-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL115-.Ltext0
	.4byte	.LVL117-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL117-.Ltext0
	.4byte	.LVL119-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST65:
	.4byte	.LVL114-.Ltext0
	.4byte	.LVL117-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL117-.Ltext0
	.4byte	.LVL118-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST66:
	.4byte	.LVL115-.Ltext0
	.4byte	.LVL117-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL117-.Ltext0
	.4byte	.LVL119-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST67:
	.4byte	.LFB158-.Ltext0
	.4byte	.LCFI21-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI21-.Ltext0
	.4byte	.LCFI22-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 28
	.4byte	.LCFI22-.Ltext0
	.4byte	.LFE158-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 56
	.4byte	0
	.4byte	0
.LLST68:
	.4byte	.LVL145-.Ltext0
	.4byte	.LVL146-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL146-.Ltext0
	.4byte	.LVL150-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST69:
	.4byte	.LVL145-.Ltext0
	.4byte	.LVL147-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL147-.Ltext0
	.4byte	.LVL149-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST70:
	.4byte	.LVL145-.Ltext0
	.4byte	.LVL148-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL148-1-.Ltext0
	.4byte	.LFE158-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST71:
	.4byte	.LVL151-.Ltext0
	.4byte	.LVL152-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL152-.Ltext0
	.4byte	.LVL153-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL154-.Ltext0
	.4byte	.LVL155-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL156-.Ltext0
	.4byte	.LVL157-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST72:
	.4byte	.LFB159-.Ltext0
	.4byte	.LCFI23-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI23-.Ltext0
	.4byte	.LCFI24-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	.LCFI24-.Ltext0
	.4byte	.LFE159-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 56
	.4byte	0
	.4byte	0
.LLST73:
	.4byte	.LVL158-.Ltext0
	.4byte	.LVL160-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL160-1-.Ltext0
	.4byte	.LFE159-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST74:
	.4byte	.LFB162-.Ltext0
	.4byte	.LCFI25-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI25-.Ltext0
	.4byte	.LFE162-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST75:
	.4byte	.LVL161-.Ltext0
	.4byte	.LVL165-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL165-.Ltext0
	.4byte	.LFE162-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST76:
	.4byte	.LVL163-.Ltext0
	.4byte	.LVL169-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL170-.Ltext0
	.4byte	.LFE162-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST77:
	.4byte	.LVL162-.Ltext0
	.4byte	.LVL167-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL168-.Ltext0
	.4byte	.LFE162-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST78:
	.4byte	.LVL164-.Ltext0
	.4byte	.LVL166-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL168-.Ltext0
	.4byte	.LVL171-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST79:
	.4byte	.LFB169-.Ltext0
	.4byte	.LCFI26-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI26-.Ltext0
	.4byte	.LFE169-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 12
	.4byte	0
	.4byte	0
.LLST80:
	.4byte	.LFB170-.Ltext0
	.4byte	.LCFI27-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI27-.Ltext0
	.4byte	.LFE170-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	0
	.4byte	0
.LLST81:
	.4byte	.LVL173-.Ltext0
	.4byte	.LVL174-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL175-.Ltext0
	.4byte	.LVL176-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL177-.Ltext0
	.4byte	.LVL192-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL193-.Ltext0
	.4byte	.LVL198-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST82:
	.4byte	.LVL173-.Ltext0
	.4byte	.LVL178-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST83:
	.4byte	.LVL181-.Ltext0
	.4byte	.LVL186-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL187-.Ltext0
	.4byte	.LVL188-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL190-.Ltext0
	.4byte	.LVL191-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL195-.Ltext0
	.4byte	.LVL196-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST84:
	.4byte	.LVL179-.Ltext0
	.4byte	.LVL184-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL185-.Ltext0
	.4byte	.LVL189-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL190-.Ltext0
	.4byte	.LVL197-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST85:
	.4byte	.LVL182-.Ltext0
	.4byte	.LVL183-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST86:
	.4byte	.LVL178-.Ltext0
	.4byte	.LVL194-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL195-.Ltext0
	.4byte	.LFE170-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST87:
	.4byte	.LFB176-.Ltext0
	.4byte	.LCFI28-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI28-.Ltext0
	.4byte	.LFE176-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 16
	.4byte	0
	.4byte	0
.LLST88:
	.4byte	.LVL199-.Ltext0
	.4byte	.LVL200-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL200-.Ltext0
	.4byte	.LVL202-1-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL203-.Ltext0
	.4byte	.LVL204-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST89:
	.4byte	.LVL199-.Ltext0
	.4byte	.LVL201-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL203-.Ltext0
	.4byte	.LVL205-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST90:
	.4byte	.LFB177-.Ltext0
	.4byte	.LCFI29-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI29-.Ltext0
	.4byte	.LFE177-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST91:
	.4byte	.LVL206-.Ltext0
	.4byte	.LVL207-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST92:
	.4byte	.LFB178-.Ltext0
	.4byte	.LCFI30-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI30-.Ltext0
	.4byte	.LFE178-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 40
	.4byte	0
	.4byte	0
.LLST93:
	.4byte	.LVL208-.Ltext0
	.4byte	.LVL209-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL209-.Ltext0
	.4byte	.LFE178-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST94:
	.4byte	.LVL210-.Ltext0
	.4byte	.LVL213-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL214-.Ltext0
	.4byte	.LVL218-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL220-.Ltext0
	.4byte	.LVL223-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL225-.Ltext0
	.4byte	.LVL226-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL227-.Ltext0
	.4byte	.LFE178-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST95:
	.4byte	.LVL210-.Ltext0
	.4byte	.LVL216-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL217-.Ltext0
	.4byte	.LVL218-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL219-.Ltext0
	.4byte	.LVL224-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL225-.Ltext0
	.4byte	.LVL226-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL227-.Ltext0
	.4byte	.LFE178-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST96:
	.4byte	.LVL211-.Ltext0
	.4byte	.LVL212-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL215-.Ltext0
	.4byte	.LVL218-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL221-.Ltext0
	.4byte	.LVL222-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST97:
	.4byte	.LFB179-.Ltext0
	.4byte	.LCFI31-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI31-.Ltext0
	.4byte	.LFE179-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST98:
	.4byte	.LVL229-.Ltext0
	.4byte	.LVL236-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL237-.Ltext0
	.4byte	.LFE179-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST99:
	.4byte	.LVL230-.Ltext0
	.4byte	.LVL231-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL232-.Ltext0
	.4byte	.LVL233-1-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL235-.Ltext0
	.4byte	.LVL238-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST100:
	.4byte	.LVL228-.Ltext0
	.4byte	.LVL234-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL235-.Ltext0
	.4byte	.LVL239-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST101:
	.4byte	.LFB180-.Ltext0
	.4byte	.LCFI32-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI32-.Ltext0
	.4byte	.LFE180-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST102:
	.4byte	.LFB181-.Ltext0
	.4byte	.LCFI33-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI33-.Ltext0
	.4byte	.LFE181-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	0
	.4byte	0
.LLST103:
	.4byte	.LVL240-.Ltext0
	.4byte	.LVL241-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL241-.Ltext0
	.4byte	.LVL242-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL242-.Ltext0
	.4byte	.LVL243-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL243-.Ltext0
	.4byte	.LVL244-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL245-.Ltext0
	.4byte	.LVL246-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST104:
	.4byte	.LVL247-.Ltext0
	.4byte	.LVL248-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL249-.Ltext0
	.4byte	.LVL250-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST105:
	.4byte	.LFB182-.Ltext0
	.4byte	.LCFI34-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI34-.Ltext0
	.4byte	.LCFI35-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 28
	.4byte	.LCFI35-.Ltext0
	.4byte	.LFE182-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 56
	.4byte	0
	.4byte	0
.LLST106:
	.4byte	.LVL251-.Ltext0
	.4byte	.LVL253-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL253-.Ltext0
	.4byte	.LFE182-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST107:
	.4byte	.LVL254-.Ltext0
	.4byte	.LVL255-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST108:
	.4byte	.LFB183-.Ltext0
	.4byte	.LCFI36-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI36-.Ltext0
	.4byte	.LCFI37-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	.LCFI37-.Ltext0
	.4byte	.LFE183-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 56
	.4byte	0
	.4byte	0
.LLST109:
	.4byte	.LFB156-.Ltext0
	.4byte	.LCFI38-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI38-.Ltext0
	.4byte	.LFE156-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	0
	.4byte	0
.LLST110:
	.4byte	.LVL257-.Ltext0
	.4byte	.LVL260-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL260-.Ltext0
	.4byte	.LVL267-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL274-.Ltext0
	.4byte	.LFE156-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST111:
	.4byte	.LVL259-.Ltext0
	.4byte	.LVL261-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL262-.Ltext0
	.4byte	.LVL263-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL264-.Ltext0
	.4byte	.LVL265-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST112:
	.4byte	.LVL266-.Ltext0
	.4byte	.LVL268-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL269-.Ltext0
	.4byte	.LVL270-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL274-.Ltext0
	.4byte	.LVL275-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL279-.Ltext0
	.4byte	.LVL280-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST113:
	.4byte	.LVL272-.Ltext0
	.4byte	.LVL273-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL279-.Ltext0
	.4byte	.LVL280-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST114:
	.4byte	.LVL271-.Ltext0
	.4byte	.LVL273-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL279-.Ltext0
	.4byte	.LVL281-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST115:
	.4byte	.LVL277-.Ltext0
	.4byte	.LVL278-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL278-.Ltext0
	.4byte	.LVL279-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST116:
	.4byte	.LVL276-.Ltext0
	.4byte	.LVL278-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL278-.Ltext0
	.4byte	.LVL279-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST117:
	.4byte	.LFB161-.Ltext0
	.4byte	.LCFI39-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI39-.Ltext0
	.4byte	.LCFI40-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI40-.Ltext0
	.4byte	.LFE161-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 104
	.4byte	0
	.4byte	0
.LLST118:
	.4byte	.LVL282-.Ltext0
	.4byte	.LVL283-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL283-.Ltext0
	.4byte	.LVL295-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL297-.Ltext0
	.4byte	.LVL298-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST119:
	.4byte	.LVL282-.Ltext0
	.4byte	.LVL288-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL288-.Ltext0
	.4byte	.LFE161-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST120:
	.4byte	.LVL299-.Ltext0
	.4byte	.LVL300-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL300-.Ltext0
	.4byte	.LFE161-.Ltext0
	.2byte	0x3
	.byte	0x91
	.sleb128 -76
	.4byte	0
	.4byte	0
.LLST121:
	.4byte	.LVL284-.Ltext0
	.4byte	.LVL285-.Ltext0
	.2byte	0x7
	.byte	0x93
	.uleb128 0x1c
	.byte	0x5a
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0xc
	.4byte	.LVL285-.Ltext0
	.4byte	.LVL286-.Ltext0
	.2byte	0xc
	.byte	0x93
	.uleb128 0x10
	.byte	0x56
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x8
	.byte	0x5a
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0xc
	.4byte	.LVL286-.Ltext0
	.4byte	.LVL287-.Ltext0
	.2byte	0x11
	.byte	0x93
	.uleb128 0x8
	.byte	0x5b
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x4
	.byte	0x56
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x8
	.byte	0x5a
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0xc
	.4byte	.LVL289-.Ltext0
	.4byte	.LVL290-.Ltext0
	.2byte	0x7
	.byte	0x93
	.uleb128 0x1c
	.byte	0x5a
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0xc
	.4byte	.LVL290-.Ltext0
	.4byte	.LVL291-.Ltext0
	.2byte	0x7
	.byte	0x93
	.uleb128 0x8
	.byte	0x5b
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x20
	.4byte	.LVL291-.Ltext0
	.4byte	.LVL292-.Ltext0
	.2byte	0xc
	.byte	0x93
	.uleb128 0x8
	.byte	0x5b
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x4
	.byte	0x56
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x18
	.4byte	.LVL292-.Ltext0
	.4byte	.LFE161-.Ltext0
	.2byte	0x11
	.byte	0x93
	.uleb128 0x8
	.byte	0x5b
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x4
	.byte	0x56
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0x8
	.byte	0x5a
	.byte	0x93
	.uleb128 0x4
	.byte	0x93
	.uleb128 0xc
	.4byte	0
	.4byte	0
.LLST122:
	.4byte	.LVL294-.Ltext0
	.4byte	.LVL296-1-.Ltext0
	.2byte	0x1
	.byte	0x5c
	.4byte	0
	.4byte	0
.LLST123:
	.4byte	.LFB163-.Ltext0
	.4byte	.LCFI41-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI41-.Ltext0
	.4byte	.LCFI42-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	.LCFI42-.Ltext0
	.4byte	.LFE163-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 56
	.4byte	0
	.4byte	0
.LLST124:
	.4byte	.LVL303-.Ltext0
	.4byte	.LVL316-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL317-.Ltext0
	.4byte	.LFE163-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST125:
	.4byte	.LVL302-.Ltext0
	.4byte	.LVL307-1-.Ltext0
	.2byte	0x1
	.byte	0x5c
	.4byte	.LVL308-.Ltext0
	.4byte	.LVL311-1-.Ltext0
	.2byte	0x1
	.byte	0x5c
	.4byte	.LVL312-.Ltext0
	.4byte	.LVL315-1-.Ltext0
	.2byte	0x1
	.byte	0x5c
	.4byte	.LVL318-.Ltext0
	.4byte	.LFE163-.Ltext0
	.2byte	0x1
	.byte	0x5c
	.4byte	0
	.4byte	0
.LLST126:
	.4byte	.LVL304-.Ltext0
	.4byte	.LVL305-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL306-.Ltext0
	.4byte	.LVL307-1-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL308-.Ltext0
	.4byte	.LVL309-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL318-.Ltext0
	.4byte	.LVL319-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST127:
	.4byte	.LVL301-.Ltext0
	.4byte	.LVL307-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL308-.Ltext0
	.4byte	.LVL311-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL312-.Ltext0
	.4byte	.LVL314-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL318-.Ltext0
	.4byte	.LFE163-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	0
	.4byte	0
.LLST128:
	.4byte	.LVL310-.Ltext0
	.4byte	.LVL313-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST129:
	.4byte	.LFB164-.Ltext0
	.4byte	.LCFI43-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI43-.Ltext0
	.4byte	.LFE164-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 16
	.4byte	0
	.4byte	0
.LLST130:
	.4byte	.LVL320-.Ltext0
	.4byte	.LVL322-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL323-.Ltext0
	.4byte	.LVL324-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST131:
	.4byte	.LFB165-.Ltext0
	.4byte	.LCFI44-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI44-.Ltext0
	.4byte	.LFE165-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST132:
	.4byte	.LFB166-.Ltext0
	.4byte	.LCFI45-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI45-.Ltext0
	.4byte	.LCFI46-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	.LCFI46-.Ltext0
	.4byte	.LFE166-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 56
	.4byte	0
	.4byte	0
.LLST133:
	.4byte	.LVL326-.Ltext0
	.4byte	.LVL327-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL327-.Ltext0
	.4byte	.LVL330-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL330-.Ltext0
	.4byte	.LVL332-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST134:
	.4byte	.LVL326-.Ltext0
	.4byte	.LVL328-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL328-1-.Ltext0
	.4byte	.LFE166-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST135:
	.4byte	.LVL326-.Ltext0
	.4byte	.LVL328-1-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL328-1-.Ltext0
	.4byte	.LFE166-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST136:
	.4byte	.LVL333-.Ltext0
	.4byte	.LVL335-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL335-.Ltext0
	.4byte	.LVL337-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL338-.Ltext0
	.4byte	.LVL339-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST137:
	.4byte	.LVL334-.Ltext0
	.4byte	.LVL335-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL335-.Ltext0
	.4byte	.LVL336-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST138:
	.4byte	.LFB168-.Ltext0
	.4byte	.LCFI47-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI47-.Ltext0
	.4byte	.LFE168-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 48
	.4byte	0
	.4byte	0
.LLST139:
	.4byte	.LVL340-.Ltext0
	.4byte	.LVL341-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL343-.Ltext0
	.4byte	.LVL345-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL345-1-.Ltext0
	.4byte	.LFE168-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST140:
	.4byte	.LVL344-.Ltext0
	.4byte	.LVL350-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL351-.Ltext0
	.4byte	.LFE168-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST141:
	.4byte	.LVL342-.Ltext0
	.4byte	.LVL348-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL349-.Ltext0
	.4byte	.LFE168-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST142:
	.4byte	.LVL346-.Ltext0
	.4byte	.LVL347-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL349-.Ltext0
	.4byte	.LVL352-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST143:
	.4byte	.LFB160-.Ltext0
	.4byte	.LCFI48-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI48-.Ltext0
	.4byte	.LFE160-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 32
	.4byte	0
	.4byte	0
.LLST144:
	.4byte	.LVL353-.Ltext0
	.4byte	.LVL355-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL355-1-.Ltext0
	.4byte	.LFE160-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST145:
	.4byte	.LVL353-.Ltext0
	.4byte	.LVL355-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL355-1-.Ltext0
	.4byte	.LFE160-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST146:
	.4byte	.LVL353-.Ltext0
	.4byte	.LVL354-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL354-.Ltext0
	.4byte	.LVL356-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL357-.Ltext0
	.4byte	.LFE160-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST147:
	.4byte	.LFB174-.Ltext0
	.4byte	.LCFI49-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI49-.Ltext0
	.4byte	.LFE174-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 48
	.4byte	0
	.4byte	0
.LLST148:
	.4byte	.LVL359-.Ltext0
	.4byte	.LVL360-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL361-.Ltext0
	.4byte	.LFE174-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST149:
	.4byte	.LFB173-.Ltext0
	.4byte	.LCFI50-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI50-.Ltext0
	.4byte	.LCFI51-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI51-.Ltext0
	.4byte	.LFE173-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 72
	.4byte	0
	.4byte	0
.LLST150:
	.4byte	.LVL363-.Ltext0
	.4byte	.LVL364-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL365-.Ltext0
	.4byte	.LVL366-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL367-.Ltext0
	.4byte	.LFE173-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST151:
	.4byte	.LFB167-.Ltext0
	.4byte	.LCFI52-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI52-.Ltext0
	.4byte	.LCFI53-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI53-.Ltext0
	.4byte	.LFE167-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 88
	.4byte	0
	.4byte	0
.LLST152:
	.4byte	.LVL378-.Ltext0
	.4byte	.LVL382-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL388-.Ltext0
	.4byte	.LVL389-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL392-.Ltext0
	.4byte	.LVL393-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL394-.Ltext0
	.4byte	.LFE167-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST153:
	.4byte	.LVL379-.Ltext0
	.4byte	.LVL387-.Ltext0
	.2byte	0x1
	.byte	0x59
	.4byte	.LVL388-.Ltext0
	.4byte	.LFE167-.Ltext0
	.2byte	0x1
	.byte	0x59
	.4byte	0
	.4byte	0
.LLST154:
	.4byte	.LVL368-.Ltext0
	.4byte	.LVL369-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL370-.Ltext0
	.4byte	.LVL381-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	.LVL395-.Ltext0
	.4byte	.LFE167-.Ltext0
	.2byte	0x1
	.byte	0x55
	.4byte	0
	.4byte	0
.LLST155:
	.4byte	.LVL383-.Ltext0
	.4byte	.LVL389-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL390-.Ltext0
	.4byte	.LVL392-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST156:
	.4byte	.LVL377-.Ltext0
	.4byte	.LVL385-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	.LVL386-.Ltext0
	.4byte	.LFE167-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST157:
	.4byte	.LVL371-.Ltext0
	.4byte	.LVL384-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL388-.Ltext0
	.4byte	.LVL396-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL397-.Ltext0
	.4byte	.LFE167-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST158:
	.4byte	.LVL382-.Ltext0
	.4byte	.LVL391-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST159:
	.4byte	.LVL372-.Ltext0
	.4byte	.LVL373-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL373-.Ltext0
	.4byte	.LVL374-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST160:
	.4byte	.LVL375-.Ltext0
	.4byte	.LVL376-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL376-.Ltext0
	.4byte	.LVL380-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST161:
	.4byte	.LFB171-.Ltext0
	.4byte	.LCFI54-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI54-.Ltext0
	.4byte	.LCFI55-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI55-.Ltext0
	.4byte	.LFE171-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 88
	.4byte	0
	.4byte	0
.LLST162:
	.4byte	.LVL398-.Ltext0
	.4byte	.LVL399-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL399-.Ltext0
	.4byte	.LFE171-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST163:
	.4byte	.LVL402-.Ltext0
	.4byte	.LVL405-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL406-.Ltext0
	.4byte	.LVL407-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL408-.Ltext0
	.4byte	.LVL409-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST164:
	.4byte	.LVL399-.Ltext0
	.4byte	.LVL403-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL404-.Ltext0
	.4byte	.LFE171-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST165:
	.4byte	.LVL399-.Ltext0
	.4byte	.LVL410-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL411-.Ltext0
	.4byte	.LFE171-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST166:
	.4byte	.LVL400-.Ltext0
	.4byte	.LVL401-1-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL412-.Ltext0
	.4byte	.LVL413-.Ltext0
	.2byte	0x3
	.byte	0x91
	.sleb128 -73
	.4byte	0
	.4byte	0
.LLST167:
	.4byte	.LFB172-.Ltext0
	.4byte	.LCFI56-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI56-.Ltext0
	.4byte	.LFE172-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 16
	.4byte	0
	.4byte	0
.LLST168:
	.4byte	.LVL414-.Ltext0
	.4byte	.LVL415-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL416-.Ltext0
	.4byte	.LVL417-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL418-.Ltext0
	.4byte	.LVL419-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL420-.Ltext0
	.4byte	.LVL421-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL422-.Ltext0
	.4byte	.LVL423-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL424-.Ltext0
	.4byte	.LVL425-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST169:
	.4byte	.LFB132-.Ltext0
	.4byte	.LCFI57-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI57-.Ltext0
	.4byte	.LCFI58-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI58-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 80
	.4byte	0
	.4byte	0
.LLST170:
	.4byte	.LVL426-.Ltext0
	.4byte	.LVL429-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL429-1-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST171:
	.4byte	.LVL426-.Ltext0
	.4byte	.LVL428-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL428-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST172:
	.4byte	.LVL426-.Ltext0
	.4byte	.LVL427-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL427-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST173:
	.4byte	.LVL451-.Ltext0
	.4byte	.LVL452-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL453-.Ltext0
	.4byte	.LVL454-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL455-.Ltext0
	.4byte	.LVL456-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST174:
	.4byte	.LVL430-.Ltext0
	.4byte	.LVL452-.Ltext0
	.2byte	0x1
	.byte	0x5b
	.4byte	.LVL455-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x1
	.byte	0x5b
	.4byte	0
	.4byte	0
.LLST175:
	.4byte	.LVL431-.Ltext0
	.4byte	.LVL452-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL455-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST176:
	.4byte	.LVL433-.Ltext0
	.4byte	.LVL452-.Ltext0
	.2byte	0x3
	.byte	0x91
	.sleb128 -72
	.4byte	.LVL455-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x3
	.byte	0x91
	.sleb128 -72
	.4byte	0
	.4byte	0
.LLST177:
	.4byte	.LVL432-.Ltext0
	.4byte	.LVL437-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL438-.Ltext0
	.4byte	.LVL452-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	.LVL455-.Ltext0
	.4byte	.LFE132-.Ltext0
	.2byte	0x1
	.byte	0x56
	.4byte	0
	.4byte	0
.LLST178:
	.4byte	.LVL441-.Ltext0
	.4byte	.LVL442-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL445-.Ltext0
	.4byte	.LVL446-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL449-.Ltext0
	.4byte	.LVL450-1-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST179:
	.4byte	.LVL434-.Ltext0
	.4byte	.LVL436-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL438-.Ltext0
	.4byte	.LVL439-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL442-.Ltext0
	.4byte	.LVL444-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL446-.Ltext0
	.4byte	.LVL447-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST180:
	.4byte	.LVL435-.Ltext0
	.4byte	.LVL436-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL438-.Ltext0
	.4byte	.LVL440-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL442-.Ltext0
	.4byte	.LVL443-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL446-.Ltext0
	.4byte	.LVL448-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST181:
	.4byte	.LFB133-.Ltext0
	.4byte	.LCFI59-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI59-.Ltext0
	.4byte	.LCFI60-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 36
	.4byte	.LCFI60-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x3
	.byte	0x7d
	.sleb128 88
	.4byte	0
	.4byte	0
.LLST182:
	.4byte	.LVL457-.Ltext0
	.4byte	.LVL460-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL460-1-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x1
	.byte	0x5a
	.4byte	0
	.4byte	0
.LLST183:
	.4byte	.LVL457-.Ltext0
	.4byte	.LVL459-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL459-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x1
	.byte	0x59
	.4byte	0
	.4byte	0
.LLST184:
	.4byte	.LVL457-.Ltext0
	.4byte	.LVL458-.Ltext0
	.2byte	0x1
	.byte	0x52
	.4byte	.LVL458-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x1
	.byte	0x5b
	.4byte	0
	.4byte	0
.LLST185:
	.4byte	.LVL479-.Ltext0
	.4byte	.LVL480-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL481-.Ltext0
	.4byte	.LVL482-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL483-.Ltext0
	.4byte	.LVL484-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST186:
	.4byte	.LVL461-.Ltext0
	.4byte	.LVL480-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	.LVL483-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x1
	.byte	0x58
	.4byte	0
	.4byte	0
.LLST187:
	.4byte	.LVL462-.Ltext0
	.4byte	.LVL480-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	.LVL483-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x1
	.byte	0x54
	.4byte	0
	.4byte	0
.LLST188:
	.4byte	.LVL464-.Ltext0
	.4byte	.LVL480-.Ltext0
	.2byte	0x3
	.byte	0x91
	.sleb128 -72
	.4byte	.LVL483-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x3
	.byte	0x91
	.sleb128 -72
	.4byte	0
	.4byte	0
.LLST189:
	.4byte	.LVL463-.Ltext0
	.4byte	.LVL480-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	.LVL483-.Ltext0
	.4byte	.LFE133-.Ltext0
	.2byte	0x1
	.byte	0x57
	.4byte	0
	.4byte	0
.LLST190:
	.4byte	.LVL471-.Ltext0
	.4byte	.LVL472-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL475-.Ltext0
	.4byte	.LVL476-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	.LVL477-.Ltext0
	.4byte	.LVL478-1-.Ltext0
	.2byte	0x1
	.byte	0x53
	.4byte	0
	.4byte	0
.LLST191:
	.4byte	.LVL466-.Ltext0
	.4byte	.LVL467-1-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL468-.Ltext0
	.4byte	.LVL470-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	.LVL472-.Ltext0
	.4byte	.LVL474-.Ltext0
	.2byte	0x1
	.byte	0x50
	.4byte	0
	.4byte	0
.LLST192:
	.4byte	.LVL465-.Ltext0
	.4byte	.LVL467-1-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL468-.Ltext0
	.4byte	.LVL469-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	.LVL472-.Ltext0
	.4byte	.LVL473-.Ltext0
	.2byte	0x1
	.byte	0x51
	.4byte	0
	.4byte	0
.LLST193:
	.4byte	.LFB175-.Ltext0
	.4byte	.LCFI61-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI61-.Ltext0
	.4byte	.LFE175-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST194:
	.4byte	.LFB131-.Ltext0
	.4byte	.LCFI62-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI62-.Ltext0
	.4byte	.LFE131-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST195:
	.4byte	.LFB129-.Ltext0
	.4byte	.LCFI63-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI63-.Ltext0
	.4byte	.LFE129-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST196:
	.4byte	.LFB128-.Ltext0
	.4byte	.LCFI64-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI64-.Ltext0
	.4byte	.LFE128-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 24
	.4byte	0
	.4byte	0
.LLST197:
	.4byte	.LFB135-.Ltext0
	.4byte	.LCFI65-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI65-.Ltext0
	.4byte	.LFE135-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
.LLST198:
	.4byte	.LFB130-.Ltext0
	.4byte	.LCFI66-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 0
	.4byte	.LCFI66-.Ltext0
	.4byte	.LFE130-.Ltext0
	.2byte	0x2
	.byte	0x7d
	.sleb128 8
	.4byte	0
	.4byte	0
	.section	.debug_pubnames,"",%progbits
.Ldebug_pubnames0:
	.4byte	0x9b4
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x2891
	.4byte	0x889
	.ascii	"sftl_var_init\000"
	.4byte	0x9a0
	.ascii	"sftl_mem_init\000"
	.4byte	0xa7b
	.ascii	"sftl_mem_deinit\000"
	.4byte	0xa94
	.ascii	"sftl_free_blk_Q_init\000"
	.4byte	0xab2
	.ascii	"sftl_get_density\000"
	.4byte	0xadc
	.ascii	"dump_sftl_bbt_info\000"
	.4byte	0xaf1
	.ascii	"dump_sftl_sys_info\000"
	.4byte	0xb07
	.ascii	"dump_sftl_l2p_info\000"
	.4byte	0xb1c
	.ascii	"dump_sftl_user_info\000"
	.4byte	0xb2c
	.ascii	"dump_ftl_info\000"
	.4byte	0xb42
	.ascii	"sftl_bbt_mark_bad_blk\000"
	.4byte	0xb6e
	.ascii	"sftl_bbt_bad_blk_check\000"
	.4byte	0xba3
	.ascii	"sftl_bbt_blk_to_bitmap\000"
	.4byte	0xc04
	.ascii	"sftl_bbt_flush\000"
	.4byte	0xc3f
	.ascii	"sftl_bbt_make\000"
	.4byte	0xcba
	.ascii	"sftl_sblk_to_pblk\000"
	.4byte	0xcdf
	.ascii	"sftl_pblk_to_plane\000"
	.4byte	0xcfe
	.ascii	"sftl_ppn_to_sblk\000"
	.4byte	0xd2b
	.ascii	"sftl_offs_to_ppn\000"
	.4byte	0xd81
	.ascii	"sftl_ppn_to_offs\000"
	.4byte	0xdc4
	.ascii	"sftl_select_trace_offs\000"
	.4byte	0xe1e
	.ascii	"sftl_get_last_written_offset\000"
	.4byte	0xf31
	.ascii	"sftl_get_last_written_page\000"
	.4byte	0xfb7
	.ascii	"sftl_bbt_load\000"
	.4byte	0x1054
	.ascii	"sftl_trace\000"
	.4byte	0x11b2
	.ascii	"sftl_write_trace_page\000"
	.4byte	0x12aa
	.ascii	"sftl_get_trace_list\000"
	.4byte	0x135c
	.ascii	"sftl_l2p_check_real_vpn\000"
	.4byte	0x13b9
	.ascii	"sftl_add_sblk_to_wait_gc_pool\000"
	.4byte	0x13ec
	.ascii	"user_gc_get_wait_offs\000"
	.4byte	0x1475
	.ascii	"sftl_sys_mark_bad_blk_flag\000"
	.4byte	0x14c2
	.ascii	"sftl_free_sblk\000"
	.4byte	0x14ed
	.ascii	"sftl_format_erase_sblk\000"
	.4byte	0x1599
	.ascii	"sys_check_sblk_cnt\000"
	.4byte	0x15fc
	.ascii	"sftl_sys_blk_init\000"
	.4byte	0x1621
	.ascii	"sftl_sys_info_load\000"
	.4byte	0x16b2
	.ascii	"sftl_sys_info_write\000"
	.4byte	0x170e
	.ascii	"sftl_sys_info_flush\000"
	.4byte	0x175a
	.ascii	"sftl_open_sblk\000"
	.4byte	0x1811
	.ascii	"sftl_write_page\000"
	.4byte	0x18f5
	.ascii	"sftl_l2p_gc\000"
	.4byte	0x19ad
	.ascii	"sftl_l2p_flush_cache\000"
	.4byte	0x19ea
	.ascii	"sftl_l2p_flush\000"
	.4byte	0x1a17
	.ascii	"sftl_log_to_phy\000"
	.4byte	0x1af5
	.ascii	"sftl_user_check_real_vpn\000"
	.4byte	0x1bb4
	.ascii	"sftl_update_vpn\000"
	.4byte	0x1c0a
	.ascii	"sftl_recover_prev_sblk\000"
	.4byte	0x1cf6
	.ascii	"sftl_recover_active_sblk\000"
	.4byte	0x1df3
	.ascii	"sftl_l2p_load\000"
	.4byte	0x1f8c
	.ascii	"sftl_user_gc_by_level\000"
	.4byte	0x20b2
	.ascii	"sftl_user_gc\000"
	.4byte	0x20e1
	.ascii	"sftl_gc\000"
	.4byte	0x20fa
	.ascii	"sftl_read\000"
	.4byte	0x21f9
	.ascii	"sftl_write\000"
	.4byte	0x22fc
	.ascii	"sftl_recovery\000"
	.4byte	0x2336
	.ascii	"sftl_deinit\000"
	.4byte	0x234f
	.ascii	"sftl_sys_init\000"
	.4byte	0x2368
	.ascii	"sftl_low_format\000"
	.4byte	0x23cb
	.ascii	"ftl_low_format\000"
	.4byte	0x23e5
	.ascii	"sftl_init\000"
	.4byte	0x245a
	.ascii	"sys_info\000"
	.4byte	0x246c
	.ascii	"bbt_info\000"
	.4byte	0x247e
	.ascii	"p_sblk_info_buf\000"
	.4byte	0x2496
	.ascii	"p_sys_data_buf\000"
	.4byte	0x24a8
	.ascii	"p_sys_spare_buf\000"
	.4byte	0x24ba
	.ascii	"p_io_data_buf\000"
	.4byte	0x24cc
	.ascii	"p_io_spare_buf\000"
	.4byte	0x24de
	.ascii	"req_sys\000"
	.4byte	0x24f0
	.ascii	"req_erase\000"
	.4byte	0x2508
	.ascii	"l2p_info\000"
	.4byte	0x251a
	.ascii	"c_l2p_sblk_num\000"
	.4byte	0x252c
	.ascii	"c_l2p_trigger_offs\000"
	.4byte	0x253e
	.ascii	"c_l2p_index_num\000"
	.4byte	0x2550
	.ascii	"c_l2p_cache_num\000"
	.4byte	0x2562
	.ascii	"p_l2p_index_ppn\000"
	.4byte	0x2574
	.ascii	"p_l2p_cache_info\000"
	.4byte	0x258c
	.ascii	"p_l2p_cache_buf\000"
	.4byte	0x259e
	.ascii	"p_l2p_trace_list\000"
	.4byte	0x25b0
	.ascii	"p_l2p_cur_trace_list\000"
	.4byte	0x25c2
	.ascii	"p_l2p_data_buf\000"
	.4byte	0x25d4
	.ascii	"p_l2p_spare_buf\000"
	.4byte	0x25e6
	.ascii	"g_ftl_rev_blk_num\000"
	.4byte	0x25f8
	.ascii	"g_sys_blk_start\000"
	.4byte	0x260a
	.ascii	"c_nand_page_size\000"
	.4byte	0x261c
	.ascii	"c_nand_sec_per_page\000"
	.4byte	0x262e
	.ascii	"c_nand_plane_num\000"
	.4byte	0x2640
	.ascii	"c_nand_plane_per_die\000"
	.4byte	0x2652
	.ascii	"c_nand_blk_per_plane\000"
	.4byte	0x2664
	.ascii	"c_nand_blk_num\000"
	.4byte	0x2676
	.ascii	"c_nand_page_num\000"
	.4byte	0x2688
	.ascii	"c_nand_page_per_blk\000"
	.4byte	0x269a
	.ascii	"c_page_per_sblk\000"
	.4byte	0x26ac
	.ascii	"c_ftl_sblk_start\000"
	.4byte	0x26ce
	.ascii	"c_plane_base_pblk\000"
	.4byte	0x26f0
	.ascii	"g_sys_blk_list\000"
	.4byte	0x2702
	.ascii	"g_sys_blk_num\000"
	.4byte	0x2714
	.ascii	"g_sys_cur_blk\000"
	.4byte	0x2726
	.ascii	"g_sys_cur_blk_index\000"
	.4byte	0x2738
	.ascii	"g_sys_cur_page\000"
	.4byte	0x274a
	.ascii	"g_sys_page_write_bitmap\000"
	.4byte	0x275c
	.ascii	"g_sys_page_last_dirty_index\000"
	.4byte	0x276e
	.ascii	"g_max_lba_sector\000"
	.4byte	0x2780
	.ascii	"g_max_lpn\000"
	.4byte	0x2792
	.ascii	"user_info\000"
	.4byte	0x27a4
	.ascii	"p_user_trace_list\000"
	.4byte	0x27b6
	.ascii	"p_user_cur_trace_list\000"
	.4byte	0x27d8
	.ascii	"trace_req\000"
	.4byte	0x2800
	.ascii	"trace_spare_buf\000"
	.4byte	0x2812
	.ascii	"ftl_read_cnt\000"
	.4byte	0x2824
	.ascii	"ftl_prog_cnt\000"
	.4byte	0x2836
	.ascii	"g_suddenly_power_lost\000"
	.4byte	0x2848
	.ascii	"g_sys_power_lost_rewrite\000"
	.4byte	0x285a
	.ascii	"g_l2p_power_lost_rewrite\000"
	.4byte	0x286c
	.ascii	"g_user_power_lost_rewrite\000"
	.4byte	0x287e
	.ascii	"user_gc\000"
	.4byte	0
	.section	.debug_pubtypes,"",%progbits
.Ldebug_pubtypes0:
	.4byte	0x2e1
	.2byte	0x2
	.4byte	.Ldebug_info0
	.4byte	0x2891
	.4byte	0x2f
	.ascii	"long unsigned int\000"
	.4byte	0x36
	.ascii	"unsigned int\000"
	.4byte	0x3d
	.ascii	"char\000"
	.4byte	0x44
	.ascii	"signed char\000"
	.4byte	0x4b
	.ascii	"unsigned char\000"
	.4byte	0x52
	.ascii	"short int\000"
	.4byte	0x59
	.ascii	"short unsigned int\000"
	.4byte	0x60
	.ascii	"int\000"
	.4byte	0x67
	.ascii	"long long int\000"
	.4byte	0x6e
	.ascii	"long long unsigned int\000"
	.4byte	0x75
	.ascii	"u8\000"
	.4byte	0x7f
	.ascii	"s16\000"
	.4byte	0x8a
	.ascii	"u16\000"
	.4byte	0x95
	.ascii	"s32\000"
	.4byte	0xa0
	.ascii	"u32\000"
	.4byte	0xab
	.ascii	"long int\000"
	.4byte	0xb2
	.ascii	"__kernel_size_t\000"
	.4byte	0xbd
	.ascii	"_Bool\000"
	.4byte	0xc4
	.ascii	"size_t\000"
	.4byte	0xcf
	.ascii	"nand_req\000"
	.4byte	0x128
	.ascii	"nand_phy_info\000"
	.4byte	0x1e3
	.ascii	"sblk_use_id_t\000"
	.4byte	0x1ef
	.ascii	"SBLK_USE_ID_FREE\000"
	.4byte	0x1f5
	.ascii	"SBLK_USE_ID_L2P_MAP\000"
	.4byte	0x1fb
	.ascii	"SBLK_USE_ID_USER_DATA\000"
	.4byte	0x201
	.ascii	"SBLK_USE_ID_MAX\000"
	.4byte	0x208
	.ascii	"sblk_info_t\000"
	.4byte	0x259
	.ascii	"sys_info_t\000"
	.4byte	0x3e3
	.ascii	"bbt_info_t\000"
	.4byte	0x4b6
	.ascii	"l2p_cache_info_t\000"
	.4byte	0x4fb
	.ascii	"l2p_info_t\000"
	.4byte	0x56a
	.ascii	"user_info_t\000"
	.4byte	0x5af
	.ascii	"e_user_gc_case\000"
	.4byte	0x5bb
	.ascii	"USER_GC_IN_READ\000"
	.4byte	0x5c1
	.ascii	"USER_GC_IN_WRITE\000"
	.4byte	0x5c7
	.ascii	"USER_GC_IN_LEISURE\000"
	.4byte	0x5cd
	.ascii	"USER_GC_HEEL_TRIGGER\000"
	.4byte	0x5d4
	.ascii	"e_user_gc_level\000"
	.4byte	0x5e0
	.ascii	"USER_GC_ONE_VPN\000"
	.4byte	0x5e6
	.ascii	"USER_GC_ONE_SBLK\000"
	.4byte	0x5ec
	.ascii	"USER_GC_DEEPLY\000"
	.4byte	0x5f3
	.ascii	"user_gc_t\000"
	.4byte	0x670
	.ascii	"w_page_para_t\000"
	.4byte	0
	.section	.debug_aranges,"",%progbits
	.4byte	0x1c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0
	.2byte	0
	.2byte	0
	.4byte	.Ltext0
	.4byte	.Letext0-.Ltext0
	.4byte	0
	.4byte	0
	.section	.debug_ranges,"",%progbits
.Ldebug_ranges0:
	.4byte	.LBB79-.Ltext0
	.4byte	.LBE79-.Ltext0
	.4byte	.LBB80-.Ltext0
	.4byte	.LBE80-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB84-.Ltext0
	.4byte	.LBE84-.Ltext0
	.4byte	.LBB85-.Ltext0
	.4byte	.LBE85-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB86-.Ltext0
	.4byte	.LBE86-.Ltext0
	.4byte	.LBB87-.Ltext0
	.4byte	.LBE87-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB88-.Ltext0
	.4byte	.LBE88-.Ltext0
	.4byte	.LBB89-.Ltext0
	.4byte	.LBE89-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB90-.Ltext0
	.4byte	.LBE90-.Ltext0
	.4byte	.LBB91-.Ltext0
	.4byte	.LBE91-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB92-.Ltext0
	.4byte	.LBE92-.Ltext0
	.4byte	.LBB93-.Ltext0
	.4byte	.LBE93-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB94-.Ltext0
	.4byte	.LBE94-.Ltext0
	.4byte	.LBB95-.Ltext0
	.4byte	.LBE95-.Ltext0
	.4byte	.LBB96-.Ltext0
	.4byte	.LBE96-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB97-.Ltext0
	.4byte	.LBE97-.Ltext0
	.4byte	.LBB98-.Ltext0
	.4byte	.LBE98-.Ltext0
	.4byte	.LBB99-.Ltext0
	.4byte	.LBE99-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB100-.Ltext0
	.4byte	.LBE100-.Ltext0
	.4byte	.LBB101-.Ltext0
	.4byte	.LBE101-.Ltext0
	.4byte	.LBB102-.Ltext0
	.4byte	.LBE102-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB103-.Ltext0
	.4byte	.LBE103-.Ltext0
	.4byte	.LBB104-.Ltext0
	.4byte	.LBE104-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB105-.Ltext0
	.4byte	.LBE105-.Ltext0
	.4byte	.LBB106-.Ltext0
	.4byte	.LBE106-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB107-.Ltext0
	.4byte	.LBE107-.Ltext0
	.4byte	.LBB111-.Ltext0
	.4byte	.LBE111-.Ltext0
	.4byte	.LBB112-.Ltext0
	.4byte	.LBE112-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB113-.Ltext0
	.4byte	.LBE113-.Ltext0
	.4byte	.LBB117-.Ltext0
	.4byte	.LBE117-.Ltext0
	.4byte	.LBB118-.Ltext0
	.4byte	.LBE118-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB123-.Ltext0
	.4byte	.LBE123-.Ltext0
	.4byte	.LBB134-.Ltext0
	.4byte	.LBE134-.Ltext0
	.4byte	.LBB135-.Ltext0
	.4byte	.LBE135-.Ltext0
	.4byte	.LBB136-.Ltext0
	.4byte	.LBE136-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB124-.Ltext0
	.4byte	.LBE124-.Ltext0
	.4byte	.LBB131-.Ltext0
	.4byte	.LBE131-.Ltext0
	.4byte	.LBB132-.Ltext0
	.4byte	.LBE132-.Ltext0
	.4byte	.LBB133-.Ltext0
	.4byte	.LBE133-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB125-.Ltext0
	.4byte	.LBE125-.Ltext0
	.4byte	.LBB129-.Ltext0
	.4byte	.LBE129-.Ltext0
	.4byte	.LBB130-.Ltext0
	.4byte	.LBE130-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB137-.Ltext0
	.4byte	.LBE137-.Ltext0
	.4byte	.LBB146-.Ltext0
	.4byte	.LBE146-.Ltext0
	.4byte	.LBB147-.Ltext0
	.4byte	.LBE147-.Ltext0
	.4byte	.LBB148-.Ltext0
	.4byte	.LBE148-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB138-.Ltext0
	.4byte	.LBE138-.Ltext0
	.4byte	.LBB143-.Ltext0
	.4byte	.LBE143-.Ltext0
	.4byte	.LBB144-.Ltext0
	.4byte	.LBE144-.Ltext0
	.4byte	.LBB145-.Ltext0
	.4byte	.LBE145-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB139-.Ltext0
	.4byte	.LBE139-.Ltext0
	.4byte	.LBB142-.Ltext0
	.4byte	.LBE142-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB149-.Ltext0
	.4byte	.LBE149-.Ltext0
	.4byte	.LBB150-.Ltext0
	.4byte	.LBE150-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB153-.Ltext0
	.4byte	.LBE153-.Ltext0
	.4byte	.LBB171-.Ltext0
	.4byte	.LBE171-.Ltext0
	.4byte	.LBB172-.Ltext0
	.4byte	.LBE172-.Ltext0
	.4byte	.LBB173-.Ltext0
	.4byte	.LBE173-.Ltext0
	.4byte	.LBB174-.Ltext0
	.4byte	.LBE174-.Ltext0
	.4byte	.LBB175-.Ltext0
	.4byte	.LBE175-.Ltext0
	.4byte	.LBB176-.Ltext0
	.4byte	.LBE176-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB154-.Ltext0
	.4byte	.LBE154-.Ltext0
	.4byte	.LBB165-.Ltext0
	.4byte	.LBE165-.Ltext0
	.4byte	.LBB166-.Ltext0
	.4byte	.LBE166-.Ltext0
	.4byte	.LBB167-.Ltext0
	.4byte	.LBE167-.Ltext0
	.4byte	.LBB168-.Ltext0
	.4byte	.LBE168-.Ltext0
	.4byte	.LBB169-.Ltext0
	.4byte	.LBE169-.Ltext0
	.4byte	.LBB170-.Ltext0
	.4byte	.LBE170-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB155-.Ltext0
	.4byte	.LBE155-.Ltext0
	.4byte	.LBB161-.Ltext0
	.4byte	.LBE161-.Ltext0
	.4byte	.LBB162-.Ltext0
	.4byte	.LBE162-.Ltext0
	.4byte	.LBB163-.Ltext0
	.4byte	.LBE163-.Ltext0
	.4byte	.LBB164-.Ltext0
	.4byte	.LBE164-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB177-.Ltext0
	.4byte	.LBE177-.Ltext0
	.4byte	.LBB190-.Ltext0
	.4byte	.LBE190-.Ltext0
	.4byte	.LBB191-.Ltext0
	.4byte	.LBE191-.Ltext0
	.4byte	.LBB194-.Ltext0
	.4byte	.LBE194-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB178-.Ltext0
	.4byte	.LBE178-.Ltext0
	.4byte	.LBB187-.Ltext0
	.4byte	.LBE187-.Ltext0
	.4byte	.LBB188-.Ltext0
	.4byte	.LBE188-.Ltext0
	.4byte	.LBB189-.Ltext0
	.4byte	.LBE189-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB179-.Ltext0
	.4byte	.LBE179-.Ltext0
	.4byte	.LBB184-.Ltext0
	.4byte	.LBE184-.Ltext0
	.4byte	.LBB185-.Ltext0
	.4byte	.LBE185-.Ltext0
	.4byte	.LBB186-.Ltext0
	.4byte	.LBE186-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB192-.Ltext0
	.4byte	.LBE192-.Ltext0
	.4byte	.LBB193-.Ltext0
	.4byte	.LBE193-.Ltext0
	.4byte	.LBB195-.Ltext0
	.4byte	.LBE195-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB197-.Ltext0
	.4byte	.LBE197-.Ltext0
	.4byte	.LBB209-.Ltext0
	.4byte	.LBE209-.Ltext0
	.4byte	.LBB210-.Ltext0
	.4byte	.LBE210-.Ltext0
	.4byte	.LBB211-.Ltext0
	.4byte	.LBE211-.Ltext0
	.4byte	.LBB212-.Ltext0
	.4byte	.LBE212-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB198-.Ltext0
	.4byte	.LBE198-.Ltext0
	.4byte	.LBB205-.Ltext0
	.4byte	.LBE205-.Ltext0
	.4byte	.LBB206-.Ltext0
	.4byte	.LBE206-.Ltext0
	.4byte	.LBB207-.Ltext0
	.4byte	.LBE207-.Ltext0
	.4byte	.LBB208-.Ltext0
	.4byte	.LBE208-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB199-.Ltext0
	.4byte	.LBE199-.Ltext0
	.4byte	.LBB203-.Ltext0
	.4byte	.LBE203-.Ltext0
	.4byte	.LBB204-.Ltext0
	.4byte	.LBE204-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB219-.Ltext0
	.4byte	.LBE219-.Ltext0
	.4byte	.LBB223-.Ltext0
	.4byte	.LBE223-.Ltext0
	.4byte	.LBB224-.Ltext0
	.4byte	.LBE224-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB225-.Ltext0
	.4byte	.LBE225-.Ltext0
	.4byte	.LBB226-.Ltext0
	.4byte	.LBE226-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB227-.Ltext0
	.4byte	.LBE227-.Ltext0
	.4byte	.LBB228-.Ltext0
	.4byte	.LBE228-.Ltext0
	.4byte	.LBB229-.Ltext0
	.4byte	.LBE229-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB230-.Ltext0
	.4byte	.LBE230-.Ltext0
	.4byte	.LBB231-.Ltext0
	.4byte	.LBE231-.Ltext0
	.4byte	.LBB232-.Ltext0
	.4byte	.LBE232-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB233-.Ltext0
	.4byte	.LBE233-.Ltext0
	.4byte	.LBB239-.Ltext0
	.4byte	.LBE239-.Ltext0
	.4byte	.LBB240-.Ltext0
	.4byte	.LBE240-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB234-.Ltext0
	.4byte	.LBE234-.Ltext0
	.4byte	.LBB237-.Ltext0
	.4byte	.LBE237-.Ltext0
	.4byte	.LBB238-.Ltext0
	.4byte	.LBE238-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB244-.Ltext0
	.4byte	.LBE244-.Ltext0
	.4byte	.LBB248-.Ltext0
	.4byte	.LBE248-.Ltext0
	.4byte	.LBB249-.Ltext0
	.4byte	.LBE249-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB250-.Ltext0
	.4byte	.LBE250-.Ltext0
	.4byte	.LBB263-.Ltext0
	.4byte	.LBE263-.Ltext0
	.4byte	.LBB264-.Ltext0
	.4byte	.LBE264-.Ltext0
	.4byte	.LBB265-.Ltext0
	.4byte	.LBE265-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB251-.Ltext0
	.4byte	.LBE251-.Ltext0
	.4byte	.LBB260-.Ltext0
	.4byte	.LBE260-.Ltext0
	.4byte	.LBB261-.Ltext0
	.4byte	.LBE261-.Ltext0
	.4byte	.LBB262-.Ltext0
	.4byte	.LBE262-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB252-.Ltext0
	.4byte	.LBE252-.Ltext0
	.4byte	.LBB257-.Ltext0
	.4byte	.LBE257-.Ltext0
	.4byte	.LBB258-.Ltext0
	.4byte	.LBE258-.Ltext0
	.4byte	.LBB259-.Ltext0
	.4byte	.LBE259-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB268-.Ltext0
	.4byte	.LBE268-.Ltext0
	.4byte	.LBB273-.Ltext0
	.4byte	.LBE273-.Ltext0
	.4byte	.LBB274-.Ltext0
	.4byte	.LBE274-.Ltext0
	.4byte	.LBB275-.Ltext0
	.4byte	.LBE275-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB276-.Ltext0
	.4byte	.LBE276-.Ltext0
	.4byte	.LBB287-.Ltext0
	.4byte	.LBE287-.Ltext0
	.4byte	.LBB288-.Ltext0
	.4byte	.LBE288-.Ltext0
	.4byte	.LBB289-.Ltext0
	.4byte	.LBE289-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB277-.Ltext0
	.4byte	.LBE277-.Ltext0
	.4byte	.LBB284-.Ltext0
	.4byte	.LBE284-.Ltext0
	.4byte	.LBB285-.Ltext0
	.4byte	.LBE285-.Ltext0
	.4byte	.LBB286-.Ltext0
	.4byte	.LBE286-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB278-.Ltext0
	.4byte	.LBE278-.Ltext0
	.4byte	.LBB282-.Ltext0
	.4byte	.LBE282-.Ltext0
	.4byte	.LBB283-.Ltext0
	.4byte	.LBE283-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB292-.Ltext0
	.4byte	.LBE292-.Ltext0
	.4byte	.LBB297-.Ltext0
	.4byte	.LBE297-.Ltext0
	.4byte	.LBB298-.Ltext0
	.4byte	.LBE298-.Ltext0
	.4byte	.LBB299-.Ltext0
	.4byte	.LBE299-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB300-.Ltext0
	.4byte	.LBE300-.Ltext0
	.4byte	.LBB301-.Ltext0
	.4byte	.LBE301-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB302-.Ltext0
	.4byte	.LBE302-.Ltext0
	.4byte	.LBB303-.Ltext0
	.4byte	.LBE303-.Ltext0
	.4byte	.LBB304-.Ltext0
	.4byte	.LBE304-.Ltext0
	.4byte	.LBB305-.Ltext0
	.4byte	.LBE305-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB306-.Ltext0
	.4byte	.LBE306-.Ltext0
	.4byte	.LBB315-.Ltext0
	.4byte	.LBE315-.Ltext0
	.4byte	.LBB316-.Ltext0
	.4byte	.LBE316-.Ltext0
	.4byte	.LBB317-.Ltext0
	.4byte	.LBE317-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB307-.Ltext0
	.4byte	.LBE307-.Ltext0
	.4byte	.LBB312-.Ltext0
	.4byte	.LBE312-.Ltext0
	.4byte	.LBB313-.Ltext0
	.4byte	.LBE313-.Ltext0
	.4byte	.LBB314-.Ltext0
	.4byte	.LBE314-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB308-.Ltext0
	.4byte	.LBE308-.Ltext0
	.4byte	.LBB311-.Ltext0
	.4byte	.LBE311-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB318-.Ltext0
	.4byte	.LBE318-.Ltext0
	.4byte	.LBB319-.Ltext0
	.4byte	.LBE319-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB326-.Ltext0
	.4byte	.LBE326-.Ltext0
	.4byte	.LBB327-.Ltext0
	.4byte	.LBE327-.Ltext0
	.4byte	.LBB328-.Ltext0
	.4byte	.LBE328-.Ltext0
	.4byte	.LBB329-.Ltext0
	.4byte	.LBE329-.Ltext0
	.4byte	0
	.4byte	0
	.4byte	.LBB330-.Ltext0
	.4byte	.LBE330-.Ltext0
	.4byte	.LBB331-.Ltext0
	.4byte	.LBE331-.Ltext0
	.4byte	.LBB332-.Ltext0
	.4byte	.LBE332-.Ltext0
	.4byte	.LBB333-.Ltext0
	.4byte	.LBE333-.Ltext0
	.4byte	0
	.4byte	0
	.section	.debug_line,"",%progbits
.Ldebug_line0:
	.section	.debug_str,"MS",%progbits,1
.LASF69:
	.ascii	"cur_sblk\000"
.LASF154:
	.ascii	"sftl_add_sblk_to_wait_gc_pool\000"
.LASF209:
	.ascii	"mapped_ppn\000"
.LASF82:
	.ascii	"USER_GC_ONE_VPN\000"
.LASF86:
	.ascii	"rw_trigger_num\000"
.LASF17:
	.ascii	"nand_req\000"
.LASF16:
	.ascii	"p_spare\000"
.LASF91:
	.ascii	"candi_p_offs\000"
.LASF117:
	.ascii	"dump_sftl_bbt_info\000"
.LASF189:
	.ascii	"invalid_num\000"
.LASF99:
	.ascii	"plane\000"
.LASF23:
	.ascii	"blk_per_die\000"
.LASF97:
	.ascii	"spare_buf\000"
.LASF50:
	.ascii	"read_count\000"
.LASF121:
	.ascii	"sftl_bbt_mark_bad_blk\000"
.LASF137:
	.ascii	"bbm_blk\000"
.LASF64:
	.ascii	"l2p_cache_info_t\000"
.LASF51:
	.ascii	"prog_count\000"
.LASF169:
	.ascii	"user_sblk_num\000"
.LASF246:
	.ascii	"console_printk\000"
.LASF40:
	.ascii	"sys_data\000"
.LASF30:
	.ascii	"ecc_bits\000"
.LASF112:
	.ascii	"max_blk\000"
.LASF25:
	.ascii	"page_per_slc_blk\000"
.LASF1:
	.ascii	"unsigned int\000"
.LASF15:
	.ascii	"p_data\000"
.LASF206:
	.ascii	"new_sblk\000"
.LASF273:
	.ascii	"c_nand_sec_per_page\000"
.LASF59:
	.ascii	"bbt_info_t\000"
.LASF177:
	.ascii	"sftl_sys_info_flush\000"
.LASF219:
	.ascii	"gc_level\000"
.LASF275:
	.ascii	"c_nand_plane_per_die\000"
.LASF185:
	.ascii	"new_ppn\000"
.LASF111:
	.ascii	"sftl_free_blk_Q_init\000"
.LASF239:
	.ascii	"sftl_deinit\000"
.LASF218:
	.ascii	"sftl_user_gc_by_level\000"
.LASF139:
	.ascii	"p_bbt\000"
.LASF70:
	.ascii	"cur_bad_flag\000"
.LASF271:
	.ascii	"g_sys_blk_start\000"
.LASF47:
	.ascii	"padding\000"
.LASF53:
	.ascii	"read_error_count\000"
.LASF145:
	.ascii	"trace_once\000"
.LASF37:
	.ascii	"use_id\000"
.LASF266:
	.ascii	"p_l2p_trace_list\000"
.LASF276:
	.ascii	"c_nand_blk_per_plane\000"
.LASF58:
	.ascii	"spare_padding\000"
.LASF49:
	.ascii	"free_ec_min\000"
.LASF264:
	.ascii	"p_l2p_cache_info\000"
.LASF92:
	.ascii	"trace_list_owner\000"
.LASF268:
	.ascii	"p_l2p_data_buf\000"
.LASF260:
	.ascii	"c_l2p_trigger_offs\000"
.LASF175:
	.ascii	"new_page_addr\000"
.LASF293:
	.ascii	"p_user_trace_list\000"
.LASF89:
	.ascii	"list\000"
.LASF14:
	.ascii	"page_addr\000"
.LASF72:
	.ascii	"trace_offs\000"
.LASF45:
	.ascii	"active_sblk\000"
.LASF308:
	.ascii	"sftl_sys_mark_bad_blk_flag\000"
.LASF147:
	.ascii	"sftl_write_trace_page\000"
.LASF257:
	.ascii	"req_erase\000"
.LASF88:
	.ascii	"deep_trigger_num\000"
.LASF243:
	.ascii	"sftl_init\000"
.LASF10:
	.ascii	"__kernel_size_t\000"
.LASF171:
	.ascii	"sftl_sys_info_load\000"
.LASF258:
	.ascii	"l2p_info\000"
.LASF157:
	.ascii	"p_is_last\000"
.LASF43:
	.ascii	"user_sblk_cnt\000"
.LASF231:
	.ascii	"count\000"
.LASF57:
	.ascii	"index\000"
.LASF192:
	.ascii	"tmp_vpn\000"
.LASF186:
	.ascii	"para\000"
.LASF106:
	.ascii	"sftl_offs_to_ppn\000"
.LASF237:
	.ascii	"sftl_write\000"
.LASF197:
	.ascii	"sftl_l2p_flush\000"
.LASF8:
	.ascii	"long long unsigned int\000"
.LASF103:
	.ascii	"pblk\000"
.LASF129:
	.ascii	"req_temp\000"
.LASF52:
	.ascii	"prog_error_count\000"
.LASF236:
	.ascii	"lpa_nscts\000"
.LASF249:
	.ascii	"sys_info\000"
.LASF256:
	.ascii	"req_sys\000"
.LASF133:
	.ascii	"high\000"
.LASF159:
	.ascii	"tmp_min_vpn\000"
.LASF24:
	.ascii	"page_per_blk\000"
.LASF29:
	.ascii	"reserved_blk\000"
.LASF80:
	.ascii	"USER_GC_HEEL_TRIGGER\000"
.LASF46:
	.ascii	"active_candidate\000"
.LASF241:
	.ascii	"sftl_low_format\000"
.LASF141:
	.ascii	"next_page\000"
.LASF230:
	.ascii	"sftl_read\000"
.LASF146:
	.ascii	"loop\000"
.LASF151:
	.ascii	"real_vpn\000"
.LASF67:
	.ascii	"l2p_info_t\000"
.LASF163:
	.ascii	"bad_block_cnt\000"
.LASF212:
	.ascii	"sftl_l2p_load\000"
.LASF216:
	.ascii	"load_stat\000"
.LASF76:
	.ascii	"e_user_gc_case\000"
.LASF284:
	.ascii	"g_sys_blk_num\000"
.LASF295:
	.ascii	"trace_req\000"
.LASF213:
	.ascii	"s_offs\000"
.LASF77:
	.ascii	"USER_GC_IN_READ\000"
.LASF12:
	.ascii	"size_t\000"
.LASF200:
	.ascii	"offset\000"
.LASF248:
	.ascii	"g_nand_phy_info\000"
.LASF115:
	.ascii	"sftl_get_density\000"
.LASF289:
	.ascii	"g_sys_page_last_dirty_index\000"
.LASF31:
	.ascii	"SBLK_USE_ID_FREE\000"
.LASF11:
	.ascii	"_Bool\000"
.LASF36:
	.ascii	"erase_cnt\000"
.LASF178:
	.ascii	"re_save_bbmdata\000"
.LASF259:
	.ascii	"c_l2p_sblk_num\000"
.LASF228:
	.ascii	"sftl_mem_deinit\000"
.LASF302:
	.ascii	"g_user_power_lost_rewrite\000"
.LASF221:
	.ascii	"wait_gc_offs\000"
.LASF87:
	.ascii	"rw_stop_num\000"
.LASF270:
	.ascii	"g_ftl_rev_blk_num\000"
.LASF164:
	.ascii	"num_req\000"
.LASF143:
	.ascii	"remain\000"
.LASF238:
	.ascii	"sftl_recovery\000"
.LASF267:
	.ascii	"p_l2p_cur_trace_list\000"
.LASF39:
	.ascii	"sys_info_t\000"
.LASF173:
	.ascii	"p_sys\000"
.LASF41:
	.ascii	"sys_page_addr\000"
.LASF181:
	.ascii	"ret_sblk\000"
.LASF287:
	.ascii	"g_sys_cur_page\000"
.LASF35:
	.ascii	"sblk_info_t\000"
.LASF118:
	.ascii	"dump_sftl_sys_info\000"
.LASF2:
	.ascii	"char\000"
.LASF282:
	.ascii	"c_plane_base_pblk\000"
.LASF280:
	.ascii	"c_page_per_sblk\000"
.LASF242:
	.ascii	"ftl_low_format\000"
.LASF300:
	.ascii	"g_sys_power_lost_rewrite\000"
.LASF108:
	.ascii	"sftl_ppn_to_offs\000"
.LASF18:
	.ascii	"nand_phy_info\000"
.LASF290:
	.ascii	"g_max_lba_sector\000"
.LASF274:
	.ascii	"c_nand_plane_num\000"
.LASF132:
	.ascii	"spare\000"
.LASF176:
	.ascii	"old_page_addr\000"
.LASF149:
	.ascii	"sftl_get_trace_list\000"
.LASF296:
	.ascii	"trace_spare_buf\000"
.LASF119:
	.ascii	"dump_sftl_l2p_info\000"
.LASF116:
	.ascii	"sec_num\000"
.LASF13:
	.ascii	"status\000"
.LASF153:
	.ascii	"sftl_bbt_blk_to_bitmap\000"
.LASF220:
	.ascii	"wait_gc_sblk\000"
.LASF252:
	.ascii	"p_sys_data_buf\000"
.LASF265:
	.ascii	"p_l2p_cache_buf\000"
.LASF294:
	.ascii	"p_user_cur_trace_list\000"
.LASF224:
	.ascii	"user_gc_loop\000"
.LASF306:
	.ascii	"/home/zjw/kernel\000"
.LASF190:
	.ascii	"vpn_min_offs\000"
.LASF214:
	.ascii	"p_offs\000"
.LASF211:
	.ascii	"sftl_recover_active_sblk\000"
.LASF94:
	.ascii	"w_page_para_t\000"
.LASF303:
	.ascii	"user_gc\000"
.LASF81:
	.ascii	"e_user_gc_level\000"
.LASF7:
	.ascii	"long long int\000"
.LASF32:
	.ascii	"SBLK_USE_ID_L2P_MAP\000"
.LASF93:
	.ascii	"trace_list\000"
.LASF255:
	.ascii	"p_io_spare_buf\000"
.LASF203:
	.ascii	"l2p_sblk_released\000"
.LASF54:
	.ascii	"normal_power_off_flag\000"
.LASF27:
	.ascii	"sec_per_blk\000"
.LASF62:
	.ascii	"page\000"
.LASF34:
	.ascii	"SBLK_USE_ID_MAX\000"
.LASF179:
	.ascii	"re_write_sys_data\000"
.LASF281:
	.ascii	"c_ftl_sblk_start\000"
.LASF95:
	.ascii	"trigger_offs\000"
.LASF193:
	.ascii	"min_vpn\000"
.LASF130:
	.ascii	"sftl_ppn_to_sblk\000"
.LASF63:
	.ascii	"prev_blk\000"
.LASF128:
	.ascii	"sftl_bbt_make\000"
.LASF123:
	.ascii	"phy_blk\000"
.LASF156:
	.ascii	"adjust_filter_vpn_flag\000"
.LASF222:
	.ascii	"recorded_vpn\000"
.LASF202:
	.ascii	"target_offs\000"
.LASF28:
	.ascii	"byte_per_sec\000"
.LASF124:
	.ascii	"blk_tbl\000"
.LASF184:
	.ascii	"old_ppn\000"
.LASF301:
	.ascii	"g_l2p_power_lost_rewrite\000"
.LASF277:
	.ascii	"c_nand_blk_num\000"
.LASF263:
	.ascii	"p_l2p_index_ppn\000"
.LASF66:
	.ascii	"use_cnt\000"
.LASF127:
	.ascii	"write_count\000"
.LASF253:
	.ascii	"p_sys_spare_buf\000"
.LASF74:
	.ascii	"user_info_t\000"
.LASF26:
	.ascii	"sec_per_page\000"
.LASF235:
	.ascii	"lpa_sctidx\000"
.LASF233:
	.ascii	"end_lpa\000"
.LASF279:
	.ascii	"c_nand_page_per_blk\000"
.LASF122:
	.ascii	"sftl_bbt_bad_blk_check\000"
.LASF42:
	.ascii	"l2p_sblk\000"
.LASF292:
	.ascii	"user_info\000"
.LASF22:
	.ascii	"blk_per_plane\000"
.LASF65:
	.ascii	"dirty\000"
.LASF244:
	.ascii	"load_bbt\000"
.LASF210:
	.ascii	"recover_ppn\000"
.LASF85:
	.ascii	"user_gc_t\000"
.LASF44:
	.ascii	"prev_sblk\000"
.LASF61:
	.ascii	"new_bbt\000"
.LASF140:
	.ascii	"last_write_page\000"
.LASF110:
	.ascii	"plane_id\000"
.LASF254:
	.ascii	"p_io_data_buf\000"
.LASF307:
	.ascii	"dump_sftl_user_info\000"
.LASF5:
	.ascii	"short int\000"
.LASF285:
	.ascii	"g_sys_cur_blk\000"
.LASF240:
	.ascii	"sftl_sys_init\000"
.LASF71:
	.ascii	"candidate_offs\000"
.LASF9:
	.ascii	"long int\000"
.LASF161:
	.ascii	"sftl_free_sblk\000"
.LASF262:
	.ascii	"c_l2p_cache_num\000"
.LASF33:
	.ascii	"SBLK_USE_ID_USER_DATA\000"
.LASF56:
	.ascii	"version\000"
.LASF170:
	.ascii	"sftl_sys_blk_init\000"
.LASF75:
	.ascii	"sblk_use_id_t\000"
.LASF83:
	.ascii	"USER_GC_ONE_SBLK\000"
.LASF234:
	.ascii	"num_lpa\000"
.LASF102:
	.ascii	"sftl_pblk_to_plane\000"
.LASF208:
	.ascii	"sftl_recover_prev_sblk\000"
.LASF196:
	.ascii	"updating_index\000"
.LASF298:
	.ascii	"ftl_prog_cnt\000"
.LASF198:
	.ascii	"sftl_log_to_phy\000"
.LASF21:
	.ascii	"plane_per_die\000"
.LASF245:
	.ascii	"elf_hwcap\000"
.LASF104:
	.ascii	"sftl_select_trace_offs\000"
.LASF160:
	.ascii	"min_vpn_sblk\000"
.LASF155:
	.ascii	"user_gc_get_wait_offs\000"
.LASF96:
	.ascii	"data_buf\000"
.LASF199:
	.ascii	"action\000"
.LASF113:
	.ascii	"sftl_var_init\000"
.LASF101:
	.ascii	"sftl_sblk_to_pblk\000"
.LASF114:
	.ascii	"sftl_mem_init\000"
.LASF247:
	.ascii	"hex_asc\000"
.LASF152:
	.ascii	"newest_ppn\000"
.LASF165:
	.ascii	"real_plane\000"
.LASF272:
	.ascii	"c_nand_page_size\000"
.LASF126:
	.ascii	"sftl_bbt_flush\000"
.LASF144:
	.ascii	"last_offs\000"
.LASF188:
	.ascii	"sftl_l2p_gc\000"
.LASF194:
	.ascii	"sftl_l2p_flush_cache\000"
.LASF297:
	.ascii	"ftl_read_cnt\000"
.LASF304:
	.ascii	"GNU C 4.6.x-google 20120106 (prerelease)\000"
.LASF174:
	.ascii	"sftl_sys_info_write\000"
.LASF225:
	.ascii	"out_of_user_gc_loop\000"
.LASF98:
	.ascii	"power_lost_rewrite\000"
.LASF217:
	.ascii	"real_vpn_tbl\000"
.LASF251:
	.ascii	"p_sblk_info_buf\000"
.LASF148:
	.ascii	"tmp_req\000"
.LASF120:
	.ascii	"dump_ftl_info\000"
.LASF250:
	.ascii	"bbt_info\000"
.LASF229:
	.ascii	"sftl_gc\000"
.LASF0:
	.ascii	"long unsigned int\000"
.LASF201:
	.ascii	"target_index\000"
.LASF207:
	.ascii	"old_sblk\000"
.LASF195:
	.ascii	"cache_offs\000"
.LASF191:
	.ascii	"tmp_trig_offs\000"
.LASF283:
	.ascii	"g_sys_blk_list\000"
.LASF261:
	.ascii	"c_l2p_index_num\000"
.LASF215:
	.ascii	"load_cnt\000"
.LASF78:
	.ascii	"USER_GC_IN_WRITE\000"
.LASF158:
	.ascii	"filter_vpn_num\000"
.LASF226:
	.ascii	"sftl_user_gc\000"
.LASF205:
	.ascii	"sftl_update_vpn\000"
.LASF4:
	.ascii	"unsigned char\000"
.LASF125:
	.ascii	"bitmap\000"
.LASF150:
	.ascii	"sftl_l2p_check_real_vpn\000"
.LASF162:
	.ascii	"sftl_format_erase_sblk\000"
.LASF109:
	.ascii	"offs_in_pblk\000"
.LASF288:
	.ascii	"g_sys_page_write_bitmap\000"
.LASF20:
	.ascii	"die_num\000"
.LASF60:
	.ascii	"fat_bbt\000"
.LASF166:
	.ascii	"sys_check_sblk_cnt\000"
.LASF183:
	.ascii	"sftl_write_page\000"
.LASF136:
	.ascii	"sftl_bbt_load\000"
.LASF105:
	.ascii	"offs\000"
.LASF100:
	.ascii	"sblk\000"
.LASF291:
	.ascii	"g_max_lpn\000"
.LASF286:
	.ascii	"g_sys_cur_blk_index\000"
.LASF79:
	.ascii	"USER_GC_IN_LEISURE\000"
.LASF38:
	.ascii	"bad_flag\000"
.LASF48:
	.ascii	"last_open_sblk\000"
.LASF90:
	.ascii	"cur_offs\000"
.LASF172:
	.ascii	"last_blk\000"
.LASF278:
	.ascii	"c_nand_page_num\000"
.LASF3:
	.ascii	"signed char\000"
.LASF138:
	.ascii	"prev_bbm_blk\000"
.LASF168:
	.ascii	"recorded_l2p_sblk_num\000"
.LASF107:
	.ascii	"offs_in_blk\000"
.LASF84:
	.ascii	"USER_GC_DEEPLY\000"
.LASF6:
	.ascii	"short unsigned int\000"
.LASF73:
	.ascii	"gc_wait\000"
.LASF232:
	.ascii	"start_lpa\000"
.LASF55:
	.ascii	"data_version\000"
.LASF305:
	.ascii	"drivers/rksftl/rk_sftl.c\000"
.LASF182:
	.ascii	"tmp_ec_min\000"
.LASF134:
	.ascii	"sftl_get_last_written_page\000"
.LASF299:
	.ascii	"g_suddenly_power_lost\000"
.LASF269:
	.ascii	"p_l2p_spare_buf\000"
.LASF167:
	.ascii	"l2p_sblk_num\000"
.LASF223:
	.ascii	"is_last\000"
.LASF131:
	.ascii	"sftl_get_last_written_offset\000"
.LASF180:
	.ascii	"sftl_open_sblk\000"
.LASF68:
	.ascii	"cur_sblk_offs\000"
.LASF187:
	.ascii	"rewrite\000"
.LASF135:
	.ascii	"flash_type\000"
.LASF227:
	.ascii	"gc_case\000"
.LASF19:
	.ascii	"nand_type\000"
.LASF204:
	.ascii	"sftl_user_check_real_vpn\000"
.LASF142:
	.ascii	"sftl_trace\000"
	.ident	"GCC: (GNU) 4.6.x-google 20120106 (prerelease)"
	.section	.note.GNU-stack,"",%progbits
