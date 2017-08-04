/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_POWERPC_SPINLOCK_TYPES_H
#define _ASM_POWERPC_SPINLOCK_TYPES_H

#ifdef CONFIG_PPC_QUEUED_SPINLOCKS
#include <asm-generic/qspinlock_types.h>
#include <asm-generic/qrwlock_types.h>
#else
#include <asm/simple_spinlock_types.h>
#endif

#endif
