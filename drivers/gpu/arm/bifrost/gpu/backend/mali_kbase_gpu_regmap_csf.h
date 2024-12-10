/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2019-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#ifndef _KBASE_GPU_REGMAP_CSF_H_
#define _KBASE_GPU_REGMAP_CSF_H_

#include <linux/types.h>

#if !MALI_USE_CSF && defined(__KERNEL__)
#error "Cannot be compiled with JM"
#endif

/* GPU_CONTROL_MCU base address */
#define GPU_CONTROL_MCU_BASE 0x3000

/* MCU_SUBSYSTEM base address */
#define MCU_SUBSYSTEM_BASE 0x20000

/* IPA control registers */
#define COMMAND                0x000 /* (WO) Command register */
#define TIMER                  0x008 /* (RW) Timer control register */

#define SELECT_CSHW_LO         0x010 /* (RW) Counter select for CS hardware, low word */
#define SELECT_CSHW_HI         0x014 /* (RW) Counter select for CS hardware, high word */
#define SELECT_MEMSYS_LO       0x018 /* (RW) Counter select for Memory system, low word */
#define SELECT_MEMSYS_HI       0x01C /* (RW) Counter select for Memory system, high word */
#define SELECT_TILER_LO        0x020 /* (RW) Counter select for Tiler cores, low word */
#define SELECT_TILER_HI        0x024 /* (RW) Counter select for Tiler cores, high word */
#define SELECT_SHADER_LO       0x028 /* (RW) Counter select for Shader cores, low word */
#define SELECT_SHADER_HI       0x02C /* (RW) Counter select for Shader cores, high word */

/* Accumulated counter values for CS hardware */
#define VALUE_CSHW_BASE        0x100
#define VALUE_CSHW_REG_LO(n)   (VALUE_CSHW_BASE + ((n) << 3))       /* (RO) Counter value #n, low word */
#define VALUE_CSHW_REG_HI(n)   (VALUE_CSHW_BASE + ((n) << 3) + 4)   /* (RO) Counter value #n, high word */

/* Accumulated counter values for memory system */
#define VALUE_MEMSYS_BASE      0x140
#define VALUE_MEMSYS_REG_LO(n) (VALUE_MEMSYS_BASE + ((n) << 3))     /* (RO) Counter value #n, low word */
#define VALUE_MEMSYS_REG_HI(n) (VALUE_MEMSYS_BASE + ((n) << 3) + 4) /* (RO) Counter value #n, high word */

#define VALUE_TILER_BASE       0x180
#define VALUE_TILER_REG_LO(n)  (VALUE_TILER_BASE + ((n) << 3))      /* (RO) Counter value #n, low word */
#define VALUE_TILER_REG_HI(n)  (VALUE_TILER_BASE + ((n) << 3) + 4)  /* (RO) Counter value #n, high word */

#define VALUE_SHADER_BASE      0x1C0
#define VALUE_SHADER_REG_LO(n) (VALUE_SHADER_BASE + ((n) << 3))     /* (RO) Counter value #n, low word */
#define VALUE_SHADER_REG_HI(n) (VALUE_SHADER_BASE + ((n) << 3) + 4) /* (RO) Counter value #n, high word */

#define AS_STATUS_AS_ACTIVE_INT 0x2

/* Set to implementation defined, outer caching */
#define AS_MEMATTR_AARCH64_OUTER_IMPL_DEF 0x88ull
/* Set to write back memory, outer caching */
#define AS_MEMATTR_AARCH64_OUTER_WA       0x8Dull
/* Set to inner non-cacheable, outer-non-cacheable
 * Setting defined by the alloc bits is ignored, but set to a valid encoding:
 * - no-alloc on read
 * - no alloc on write
 */
#define AS_MEMATTR_AARCH64_NON_CACHEABLE  0x4Cull
/* Set to shared memory, that is inner cacheable on ACE and inner or outer
 * shared, otherwise inner non-cacheable.
 * Outer cacheable if inner or outer shared, otherwise outer non-cacheable.
 */
#define AS_MEMATTR_AARCH64_SHARED         0x8ull

/* Symbols for default MEMATTR to use
 * Default is - HW implementation defined caching
 */
#define AS_MEMATTR_INDEX_DEFAULT               0
#define AS_MEMATTR_INDEX_DEFAULT_ACE           3

/* HW implementation defined caching */
#define AS_MEMATTR_INDEX_IMPL_DEF_CACHE_POLICY 0
/* Force cache on */
#define AS_MEMATTR_INDEX_FORCE_TO_CACHE_ALL    1
/* Write-alloc */
#define AS_MEMATTR_INDEX_WRITE_ALLOC           2
/* Outer coherent, inner implementation defined policy */
#define AS_MEMATTR_INDEX_OUTER_IMPL_DEF        3
/* Outer coherent, write alloc inner */
#define AS_MEMATTR_INDEX_OUTER_WA              4
/* Normal memory, inner non-cacheable, outer non-cacheable (ARMv8 mode only) */
#define AS_MEMATTR_INDEX_NON_CACHEABLE         5
/* Normal memory, shared between MCU and Host */
#define AS_MEMATTR_INDEX_SHARED                6

/* Configuration bits for the CSF. */
#define CSF_CONFIG 0xF00

/* CSF_CONFIG register */
#define CSF_CONFIG_FORCE_COHERENCY_FEATURES_SHIFT 2

/* GPU control registers */
#define CORE_FEATURES           0x008   /* () Shader Core Features */
#define MCU_CONTROL             0x700
#define MCU_STATUS              0x704

#define MCU_CNTRL_ENABLE        (1 << 0)
#define MCU_CNTRL_AUTO          (1 << 1)
#define MCU_CNTRL_DISABLE       (0)

#define MCU_CNTRL_DOORBELL_DISABLE_SHIFT (31)
#define MCU_CNTRL_DOORBELL_DISABLE_MASK (1 << MCU_CNTRL_DOORBELL_DISABLE_SHIFT)

#define MCU_STATUS_HALTED        (1 << 1)

#define L2_CONFIG_PBHA_HWU_SHIFT GPU_U(12)
#define L2_CONFIG_PBHA_HWU_MASK (GPU_U(0xF) << L2_CONFIG_PBHA_HWU_SHIFT)
#define L2_CONFIG_PBHA_HWU_GET(reg_val)                                                            \
	(((reg_val)&L2_CONFIG_PBHA_HWU_MASK) >> L2_CONFIG_PBHA_HWU_SHIFT)
#define L2_CONFIG_PBHA_HWU_SET(reg_val, value)                                                     \
	(((reg_val) & ~L2_CONFIG_PBHA_HWU_MASK) |                                                  \
	 (((value) << L2_CONFIG_PBHA_HWU_SHIFT) & L2_CONFIG_PBHA_HWU_MASK))

/* JOB IRQ flags */
#define JOB_IRQ_GLOBAL_IF (1u << 31) /* Global interface interrupt received */

/* GPU_COMMAND codes */
#define GPU_COMMAND_CODE_NOP                0x00 /* No operation, nothing happens */
#define GPU_COMMAND_CODE_RESET              0x01 /* Reset the GPU */
#define GPU_COMMAND_CODE_TIME               0x03 /* Configure time sources */
#define GPU_COMMAND_CODE_FLUSH_CACHES       0x04 /* Flush caches */
#define GPU_COMMAND_CODE_SET_PROTECTED_MODE 0x05 /* Places the GPU in protected mode */
#define GPU_COMMAND_CODE_FINISH_HALT        0x06 /* Halt CSF */
#define GPU_COMMAND_CODE_CLEAR_FAULT        0x07 /* Clear GPU_FAULTSTATUS and GPU_FAULTADDRESS, TODX */
#define GPU_COMMAND_CODE_FLUSH_PA_RANGE 0x08 /* Flush the GPU caches for a physical range, TITX */

/* GPU_COMMAND_RESET payloads */

/* This will leave the state of active jobs UNDEFINED, but will leave the external bus in a defined and idle state.
 * Power domains will remain powered on.
 */
#define GPU_COMMAND_RESET_PAYLOAD_FAST_RESET 0x00

/* This will leave the state of active CSs UNDEFINED, but will leave the external bus in a defined and
 * idle state.
 */
#define GPU_COMMAND_RESET_PAYLOAD_SOFT_RESET 0x01

/* This reset will leave the state of currently active streams UNDEFINED, will likely lose data, and may leave
 * the system bus in an inconsistent state. Use only as a last resort when nothing else works.
 */
#define GPU_COMMAND_RESET_PAYLOAD_HARD_RESET 0x02

/* GPU_COMMAND_TIME payloads */
#define GPU_COMMAND_TIME_DISABLE 0x00 /* Disable cycle counter */
#define GPU_COMMAND_TIME_ENABLE  0x01 /* Enable cycle counter */

/* GPU_COMMAND_FLUSH_CACHES payloads bits for L2 caches */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_L2_NONE 0x000 /* No flush */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_L2_CLEAN 0x001 /* CLN only */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_L2_CLEAN_INVALIDATE 0x003 /* CLN + INV */

/* GPU_COMMAND_FLUSH_CACHES payloads bits for Load-store caches */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_LSC_NONE 0x000 /* No flush */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_LSC_CLEAN 0x010 /* CLN only */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_LSC_CLEAN_INVALIDATE 0x030 /* CLN + INV */

/* GPU_COMMAND_FLUSH_CACHES payloads bits for Other caches */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_OTHER_NONE 0x000 /* No flush */
#define GPU_COMMAND_FLUSH_CACHES_PAYLOAD_OTHER_INVALIDATE 0x200 /* INV only */

/* GPU_COMMAND_FLUSH_PA_RANGE payload bits for flush modes */
#define GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_MODE_NONE 0x00 /* No flush */
#define GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_MODE_CLEAN 0x01 /* CLN only */
#define GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_MODE_INVALIDATE 0x02 /* INV only */
#define GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_MODE_CLEAN_INVALIDATE 0x03 /* CLN + INV */

/* GPU_COMMAND_FLUSH_PA_RANGE payload bits for which caches should be the target of the command */
#define GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_OTHER_CACHE 0x10 /* Other caches */
#define GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_LSC_CACHE 0x20 /* Load-store caches */
#define GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_L2_CACHE 0x40 /* L2 caches */

/* GPU_COMMAND command + payload */
#define GPU_COMMAND_CODE_PAYLOAD(opcode, payload) \
	((__u32)opcode | ((__u32)payload << 8))

/* Final GPU_COMMAND form */
/* No operation, nothing happens */
#define GPU_COMMAND_NOP \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_NOP, 0)

/* Stop all external bus interfaces, and then reset the entire GPU. */
#define GPU_COMMAND_SOFT_RESET \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_RESET, GPU_COMMAND_RESET_PAYLOAD_SOFT_RESET)

/* Immediately reset the entire GPU. */
#define GPU_COMMAND_HARD_RESET \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_RESET, GPU_COMMAND_RESET_PAYLOAD_HARD_RESET)

/* Starts the cycle counter, and system timestamp propagation */
#define GPU_COMMAND_CYCLE_COUNT_START \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_TIME, GPU_COMMAND_TIME_ENABLE)

/* Stops the cycle counter, and system timestamp propagation */
#define GPU_COMMAND_CYCLE_COUNT_STOP \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_TIME, GPU_COMMAND_TIME_DISABLE)

/* Clean and invalidate L2 cache (Equivalent to FLUSH_PT) */
#define GPU_COMMAND_CACHE_CLN_INV_L2                                                               \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FLUSH_CACHES,                                    \
				 (GPU_COMMAND_FLUSH_CACHES_PAYLOAD_L2_CLEAN_INVALIDATE |           \
				  GPU_COMMAND_FLUSH_CACHES_PAYLOAD_LSC_NONE |                      \
				  GPU_COMMAND_FLUSH_CACHES_PAYLOAD_OTHER_NONE))

/* Clean and invalidate L2 and LSC caches (Equivalent to FLUSH_MEM) */
#define GPU_COMMAND_CACHE_CLN_INV_L2_LSC                                                           \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FLUSH_CACHES,                                    \
				 (GPU_COMMAND_FLUSH_CACHES_PAYLOAD_L2_CLEAN_INVALIDATE |           \
				  GPU_COMMAND_FLUSH_CACHES_PAYLOAD_LSC_CLEAN_INVALIDATE |          \
				  GPU_COMMAND_FLUSH_CACHES_PAYLOAD_OTHER_NONE))

/* Clean and invalidate L2, LSC, and Other caches */
#define GPU_COMMAND_CACHE_CLN_INV_FULL                                                             \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FLUSH_CACHES,                                    \
				 (GPU_COMMAND_FLUSH_CACHES_PAYLOAD_L2_CLEAN_INVALIDATE |           \
				  GPU_COMMAND_FLUSH_CACHES_PAYLOAD_LSC_CLEAN_INVALIDATE |          \
				  GPU_COMMAND_FLUSH_CACHES_PAYLOAD_OTHER_INVALIDATE))

/* Clean and invalidate only LSC cache */
#define GPU_COMMAND_CACHE_CLN_INV_LSC                                                              \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FLUSH_CACHES,                                    \
				  (GPU_COMMAND_FLUSH_CACHES_PAYLOAD_L2_NONE |                      \
				   GPU_COMMAND_FLUSH_CACHES_PAYLOAD_LSC_CLEAN_INVALIDATE |         \
				   GPU_COMMAND_FLUSH_CACHES_PAYLOAD_OTHER_NONE))

/* Clean and invalidate physical range L2 cache (equivalent to FLUSH_PT) */
#define GPU_COMMAND_FLUSH_PA_RANGE_CLN_INV_L2                                                      \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FLUSH_PA_RANGE,                                  \
				 (GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_MODE_CLEAN_INVALIDATE |       \
				  GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_L2_CACHE))

/* Clean and invalidate physical range L2 and LSC cache (equivalent to FLUSH_MEM) */
#define GPU_COMMAND_FLUSH_PA_RANGE_CLN_INV_L2_LSC                                                  \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FLUSH_PA_RANGE,                                  \
				 (GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_MODE_CLEAN_INVALIDATE |       \
				  GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_LSC_CACHE |                   \
				  GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_L2_CACHE))

/* Clean and invalidate physical range L2, LSC and Other caches */
#define GPU_COMMAND_FLUSH_PA_RANGE_CLN_INV_FULL                                                    \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FLUSH_PA_RANGE,                                  \
				 (GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_MODE_CLEAN_INVALIDATE |       \
				  GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_OTHER_CACHE |                 \
				  GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_LSC_CACHE |                   \
				  GPU_COMMAND_FLUSH_PA_RANGE_PAYLOAD_L2_CACHE))

/* Merge cache flush commands */
#define GPU_COMMAND_FLUSH_CACHE_MERGE(cmd1, cmd2) ((cmd1) | (cmd2))

/* Places the GPU in protected mode */
#define GPU_COMMAND_SET_PROTECTED_MODE \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_SET_PROTECTED_MODE, 0)

/* Halt CSF */
#define GPU_COMMAND_FINISH_HALT \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_FINISH_HALT, 0)

/* Clear GPU faults */
#define GPU_COMMAND_CLEAR_FAULT \
	GPU_COMMAND_CODE_PAYLOAD(GPU_COMMAND_CODE_CLEAR_FAULT, 0)

/* End Command Values */

/* GPU_FAULTSTATUS register */
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_SHIFT 0
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_MASK (0xFFul)
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_GET(reg_val) \
	(((reg_val)&GPU_FAULTSTATUS_EXCEPTION_TYPE_MASK) \
	 >> GPU_FAULTSTATUS_EXCEPTION_TYPE_SHIFT)
#define GPU_FAULTSTATUS_ACCESS_TYPE_SHIFT 8
#define GPU_FAULTSTATUS_ACCESS_TYPE_MASK \
	(0x3ul << GPU_FAULTSTATUS_ACCESS_TYPE_SHIFT)

#define GPU_FAULTSTATUS_ADDR_VALID_SHIFT 10
#define GPU_FAULTSTATUS_ADDR_VALID_FLAG \
	(1ul << GPU_FAULTSTATUS_ADDR_VALID_SHIFT)

#define GPU_FAULTSTATUS_JASID_VALID_SHIFT 11
#define GPU_FAULTSTATUS_JASID_VALID_FLAG \
	(1ul << GPU_FAULTSTATUS_JASID_VALID_SHIFT)

#define GPU_FAULTSTATUS_JASID_SHIFT 12
#define GPU_FAULTSTATUS_JASID_MASK (0xF << GPU_FAULTSTATUS_JASID_SHIFT)
#define GPU_FAULTSTATUS_JASID_GET(reg_val) \
	(((reg_val)&GPU_FAULTSTATUS_JASID_MASK) >> GPU_FAULTSTATUS_JASID_SHIFT)
#define GPU_FAULTSTATUS_JASID_SET(reg_val, value) \
	(((reg_val) & ~GPU_FAULTSTATUS_JASID_MASK) |  \
	(((value) << GPU_FAULTSTATUS_JASID_SHIFT) & GPU_FAULTSTATUS_JASID_MASK))

#define GPU_FAULTSTATUS_SOURCE_ID_SHIFT 16
#define GPU_FAULTSTATUS_SOURCE_ID_MASK \
	(0xFFFFul << GPU_FAULTSTATUS_SOURCE_ID_SHIFT)
/* End GPU_FAULTSTATUS register */

/* GPU_FAULTSTATUS_ACCESS_TYPE values */
#define GPU_FAULTSTATUS_ACCESS_TYPE_ATOMIC 0x0
#define GPU_FAULTSTATUS_ACCESS_TYPE_EXECUTE 0x1
#define GPU_FAULTSTATUS_ACCESS_TYPE_READ 0x2
#define GPU_FAULTSTATUS_ACCESS_TYPE_WRITE 0x3
/* End of GPU_FAULTSTATUS_ACCESS_TYPE values */

/* Implementation-dependent exception codes used to indicate CSG
 * and CS errors that are not specified in the specs.
 */
#define GPU_EXCEPTION_TYPE_SW_FAULT_0 ((__u8)0x70)
#define GPU_EXCEPTION_TYPE_SW_FAULT_1 ((__u8)0x71)
#define GPU_EXCEPTION_TYPE_SW_FAULT_2 ((__u8)0x72)

/* GPU_FAULTSTATUS_EXCEPTION_TYPE values */
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_OK 0x00
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_GPU_BUS_FAULT 0x80
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_GPU_SHAREABILITY_FAULT 0x88
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_SYSTEM_SHAREABILITY_FAULT 0x89
#define GPU_FAULTSTATUS_EXCEPTION_TYPE_GPU_CACHEABILITY_FAULT 0x8A
/* End of GPU_FAULTSTATUS_EXCEPTION_TYPE values */

#define GPU_FAULTSTATUS_ADDRESS_VALID_SHIFT GPU_U(10)
#define GPU_FAULTSTATUS_ADDRESS_VALID_MASK (GPU_U(0x1) << GPU_FAULTSTATUS_ADDRESS_VALID_SHIFT)
#define GPU_FAULTSTATUS_ADDRESS_VALID_GET(reg_val) \
	(((reg_val)&GPU_FAULTSTATUS_ADDRESS_VALID_MASK) >> GPU_FAULTSTATUS_ADDRESS_VALID_SHIFT)
#define GPU_FAULTSTATUS_ADDRESS_VALID_SET(reg_val, value) \
	(((reg_val) & ~GPU_FAULTSTATUS_ADDRESS_VALID_MASK) |  \
	(((value) << GPU_FAULTSTATUS_ADDRESS_VALID_SHIFT) & GPU_FAULTSTATUS_ADDRESS_VALID_MASK))

/* IRQ flags */
#define GPU_FAULT (1 << 0) /* A GPU Fault has occurred */
#define GPU_PROTECTED_FAULT (1 << 1) /* A GPU fault has occurred in protected mode */
#define RESET_COMPLETED (1 << 8) /* Set when a reset has completed.  */
#define POWER_CHANGED_SINGLE (1 << 9) /* Set when a single core has finished powering up or down. */
#define POWER_CHANGED_ALL (1 << 10) /* Set when all cores have finished powering up or down. */
#define CLEAN_CACHES_COMPLETED (1 << 17) /* Set when a cache clean operation has completed. */
#define DOORBELL_MIRROR (1 << 18) /* Mirrors the doorbell interrupt line to the CPU */
#define MCU_STATUS_GPU_IRQ (1 << 19) /* MCU requires attention */
#define FLUSH_PA_RANGE_COMPLETED                                                                   \
	(1 << 20) /* Set when a physical range cache clean operation has completed. */

/*
 * In Debug build,
 * GPU_IRQ_REG_COMMON | POWER_CHANGED_SINGLE is used to clear and unmask interupts sources of GPU_IRQ
 * by writing it onto GPU_IRQ_CLEAR/MASK registers.
 *
 * In Release build,
 * GPU_IRQ_REG_COMMON is used.
 *
 * Note:
 * CLEAN_CACHES_COMPLETED - Used separately for cache operation.
 * DOORBELL_MIRROR - Do not have it included for GPU_IRQ_REG_COMMON
 *                   as it can't be cleared by GPU_IRQ_CLEAR, thus interrupt storm might happen
 */
#define GPU_IRQ_REG_COMMON (GPU_FAULT | GPU_PROTECTED_FAULT | RESET_COMPLETED \
			| POWER_CHANGED_ALL | MCU_STATUS_GPU_IRQ)

/* GPU_FEATURES register */
#define GPU_FEATURES_RAY_TRACING_SHIFT GPU_U(2)
#define GPU_FEATURES_RAY_TRACING_MASK (GPU_U(0x1) << GPU_FEATURES_RAY_TRACING_SHIFT)
#define GPU_FEATURES_RAY_TRACING_GET(reg_val) \
	(((reg_val)&GPU_FEATURES_RAY_TRACING_MASK) >> GPU_FEATURES_RAY_TRACING_SHIFT)
/* End of GPU_FEATURES register */

#endif /* _KBASE_GPU_REGMAP_CSF_H_ */
