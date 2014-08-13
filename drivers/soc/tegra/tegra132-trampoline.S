/*
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

	.align 5
	.globl __aarch64_tramp

__aarch64_tramp:
	.inst   0xe3a00003	//	mov r0, #3
	.inst   0xee0c0f50	//	mcr p15, 0, r0, c12, c0, {2}
	.inst   0xeafffffe	//	b   .
