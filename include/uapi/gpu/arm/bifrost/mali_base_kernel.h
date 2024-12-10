/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2010-2022 ARM Limited. All rights reserved.
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

/*
 * Base structures shared with the kernel.
 */

#ifndef _UAPI_BASE_KERNEL_H_
#define _UAPI_BASE_KERNEL_H_

#include <linux/types.h>
#include "mali_base_mem_priv.h"
#include "gpu/mali_kbase_gpu_id.h"
#include "gpu/mali_kbase_gpu_coherency.h"

#define BASE_MAX_COHERENT_GROUPS 16

#if defined(PAGE_MASK) && defined(PAGE_SHIFT)
#define LOCAL_PAGE_SHIFT PAGE_SHIFT
#define LOCAL_PAGE_LSB ~PAGE_MASK
#else
#ifndef OSU_CONFIG_CPU_PAGE_SIZE_LOG2
#define OSU_CONFIG_CPU_PAGE_SIZE_LOG2 12
#endif

#if defined(OSU_CONFIG_CPU_PAGE_SIZE_LOG2)
#define LOCAL_PAGE_SHIFT OSU_CONFIG_CPU_PAGE_SIZE_LOG2
#define LOCAL_PAGE_LSB ((1ul << OSU_CONFIG_CPU_PAGE_SIZE_LOG2) - 1)
#else
#error Failed to find page size
#endif
#endif

/* Physical memory group ID for normal usage.
 */
#define BASE_MEM_GROUP_DEFAULT (0)

/* Number of physical memory groups.
 */
#define BASE_MEM_GROUP_COUNT (16)

/**
 * typedef base_mem_alloc_flags - Memory allocation, access/hint flags.
 *
 * A combination of MEM_PROT/MEM_HINT flags must be passed to each allocator
 * in order to determine the best cache policy. Some combinations are
 * of course invalid (e.g. MEM_PROT_CPU_WR | MEM_HINT_CPU_RD),
 * which defines a write-only region on the CPU side, which is
 * heavily read by the CPU...
 * Other flags are only meaningful to a particular allocator.
 * More flags can be added to this list, as long as they don't clash
 * (see BASE_MEM_FLAGS_NR_BITS for the number of the first free bit).
 */
typedef __u32 base_mem_alloc_flags;

/* A mask for all the flags which are modifiable via the base_mem_set_flags
 * interface.
 */
#define BASE_MEM_FLAGS_MODIFIABLE \
	(BASE_MEM_DONT_NEED | BASE_MEM_COHERENT_SYSTEM | \
	 BASE_MEM_COHERENT_LOCAL)

/* A mask of all the flags that can be returned via the base_mem_get_flags()
 * interface.
 */
#define BASE_MEM_FLAGS_QUERYABLE \
	(BASE_MEM_FLAGS_INPUT_MASK & ~(BASE_MEM_SAME_VA | \
		BASE_MEM_COHERENT_SYSTEM_REQUIRED | BASE_MEM_DONT_NEED | \
		BASE_MEM_IMPORT_SHARED | BASE_MEM_FLAGS_RESERVED | \
		BASEP_MEM_FLAGS_KERNEL_ONLY))

/**
 * enum base_mem_import_type - Memory types supported by @a base_mem_import
 *
 * @BASE_MEM_IMPORT_TYPE_INVALID: Invalid type
 * @BASE_MEM_IMPORT_TYPE_UMM: UMM import. Handle type is a file descriptor (int)
 * @BASE_MEM_IMPORT_TYPE_USER_BUFFER: User buffer import. Handle is a
 * base_mem_import_user_buffer
 *
 * Each type defines what the supported handle type is.
 *
 * If any new type is added here ARM must be contacted
 * to allocate a numeric value for it.
 * Do not just add a new type without synchronizing with ARM
 * as future releases from ARM might include other new types
 * which could clash with your custom types.
 */
enum base_mem_import_type {
	BASE_MEM_IMPORT_TYPE_INVALID = 0,
	/*
	 * Import type with value 1 is deprecated.
	 */
	BASE_MEM_IMPORT_TYPE_UMM = 2,
	BASE_MEM_IMPORT_TYPE_USER_BUFFER = 3
};

/**
 * struct base_mem_import_user_buffer - Handle of an imported user buffer
 *
 * @ptr:	address of imported user buffer
 * @length:	length of imported user buffer in bytes
 *
 * This structure is used to represent a handle of an imported user buffer.
 */

struct base_mem_import_user_buffer {
	__u64 ptr;
	__u64 length;
};

/* Mask to detect 4GB boundary alignment */
#define BASE_MEM_MASK_4GB  0xfffff000UL
/* Mask to detect 4GB boundary (in page units) alignment */
#define BASE_MEM_PFN_MASK_4GB  (BASE_MEM_MASK_4GB >> LOCAL_PAGE_SHIFT)

/* Limit on the 'extension' parameter for an allocation with the
 * BASE_MEM_TILER_ALIGN_TOP flag set
 *
 * This is the same as the maximum limit for a Buffer Descriptor's chunk size
 */
#define BASE_MEM_TILER_ALIGN_TOP_EXTENSION_MAX_PAGES_LOG2                      \
	(21u - (LOCAL_PAGE_SHIFT))
#define BASE_MEM_TILER_ALIGN_TOP_EXTENSION_MAX_PAGES                           \
	(1ull << (BASE_MEM_TILER_ALIGN_TOP_EXTENSION_MAX_PAGES_LOG2))

/* Bit mask of cookies used for memory allocation setup */
#define KBASE_COOKIE_MASK  ~1UL /* bit 0 is reserved */

/* Maximum size allowed in a single KBASE_IOCTL_MEM_ALLOC call */
#define KBASE_MEM_ALLOC_MAX_SIZE ((8ull << 30) >> PAGE_SHIFT) /* 8 GB */

/*
 * struct base_fence - Cross-device synchronisation fence.
 *
 * A fence is used to signal when the GPU has finished accessing a resource that
 * may be shared with other devices, and also to delay work done asynchronously
 * by the GPU until other devices have finished accessing a shared resource.
 */
struct base_fence {
	struct {
		int fd;
		int stream_fd;
	} basep;
};

/**
 * struct base_mem_aliasing_info - Memory aliasing info
 *
 * @handle: Handle to alias, can be BASE_MEM_WRITE_ALLOC_PAGES_HANDLE
 * @offset: Offset within the handle to start aliasing from, in pages.
 *          Not used with BASE_MEM_WRITE_ALLOC_PAGES_HANDLE.
 * @length: Length to alias, in pages. For BASE_MEM_WRITE_ALLOC_PAGES_HANDLE
 *          specifies the number of times the special page is needed.
 *
 * Describes a memory handle to be aliased.
 * A subset of the handle can be chosen for aliasing, given an offset and a
 * length.
 * A special handle BASE_MEM_WRITE_ALLOC_PAGES_HANDLE is used to represent a
 * region where a special page is mapped with a write-alloc cache setup,
 * typically used when the write result of the GPU isn't needed, but the GPU
 * must write anyway.
 *
 * Offset and length are specified in pages.
 * Offset must be within the size of the handle.
 * Offset+length must not overrun the size of the handle.
 */
struct base_mem_aliasing_info {
	struct base_mem_handle handle;
	__u64 offset;
	__u64 length;
};

/* Maximum percentage of just-in-time memory allocation trimming to perform
 * on free.
 */
#define BASE_JIT_MAX_TRIM_LEVEL (100)

/* Maximum number of concurrent just-in-time memory allocations.
 */
#define BASE_JIT_ALLOC_COUNT (255)

/**
 * struct base_jit_alloc_info - Structure which describes a JIT allocation
 *                              request.
 * @gpu_alloc_addr:             The GPU virtual address to write the JIT
 *                              allocated GPU virtual address to.
 * @va_pages:                   The minimum number of virtual pages required.
 * @commit_pages:               The minimum number of physical pages which
 *                              should back the allocation.
 * @extension:                     Granularity of physical pages to grow the
 *                              allocation by during a fault.
 * @id:                         Unique ID provided by the caller, this is used
 *                              to pair allocation and free requests.
 *                              Zero is not a valid value.
 * @bin_id:                     The JIT allocation bin, used in conjunction with
 *                              @max_allocations to limit the number of each
 *                              type of JIT allocation.
 * @max_allocations:            The maximum number of allocations allowed within
 *                              the bin specified by @bin_id. Should be the same
 *                              for all allocations within the same bin.
 * @flags:                      flags specifying the special requirements for
 *                              the JIT allocation, see
 *                              %BASE_JIT_ALLOC_VALID_FLAGS
 * @padding:                    Expansion space - should be initialised to zero
 * @usage_id:                   A hint about which allocation should be reused.
 *                              The kernel should attempt to use a previous
 *                              allocation with the same usage_id
 * @heap_info_gpu_addr:         Pointer to an object in GPU memory describing
 *                              the actual usage of the region.
 *
 * Kbase version history:
 * 11.20: added @heap_info_gpu_addr
 */
struct base_jit_alloc_info {
	__u64 gpu_alloc_addr;
	__u64 va_pages;
	__u64 commit_pages;
	__u64 extension;
	__u8 id;
	__u8 bin_id;
	__u8 max_allocations;
	__u8 flags;
	__u8 padding[2];
	__u16 usage_id;
	__u64 heap_info_gpu_addr;
};

enum base_external_resource_access {
	BASE_EXT_RES_ACCESS_SHARED,
	BASE_EXT_RES_ACCESS_EXCLUSIVE
};

struct base_external_resource {
	__u64 ext_resource;
};

/**
 * BASE_EXT_RES_COUNT_MAX - The maximum number of external resources
 * which can be mapped/unmapped in a single request.
 */
#define BASE_EXT_RES_COUNT_MAX 10

/**
 * struct base_external_resource_list - Structure which describes a list of
 *                                      external resources.
 * @count:                              The number of resources.
 * @ext_res:                            Array of external resources which is
 *                                      sized at allocation time.
 */
struct base_external_resource_list {
	__u64 count;
	struct base_external_resource ext_res[1];
};

struct base_jd_debug_copy_buffer {
	__u64 address;
	__u64 size;
	struct base_external_resource extres;
};

#define GPU_MAX_JOB_SLOTS 16

/**
 * DOC: User-side Base GPU Property Queries
 *
 * The User-side Base GPU Property Query interface encapsulates two
 * sub-modules:
 *
 * - "Dynamic GPU Properties"
 * - "Base Platform Config GPU Properties"
 *
 * Base only deals with properties that vary between different GPU
 * implementations - the Dynamic GPU properties and the Platform Config
 * properties.
 *
 * For properties that are constant for the GPU Architecture, refer to the
 * GPU module. However, we will discuss their relevance here just to
 * provide background information.
 *
 * About the GPU Properties in Base and GPU modules
 *
 * The compile-time properties (Platform Config, GPU Compile-time
 * properties) are exposed as pre-processor macros.
 *
 * Complementing the compile-time properties are the Dynamic GPU
 * Properties, which act as a conduit for the GPU Configuration
 * Discovery.
 *
 * In general, the dynamic properties are present to verify that the platform
 * has been configured correctly with the right set of Platform Config
 * Compile-time Properties.
 *
 * As a consistent guide across the entire DDK, the choice for dynamic or
 * compile-time should consider the following, in order:
 * 1. Can the code be written so that it doesn't need to know the
 * implementation limits at all?
 * 2. If you need the limits, get the information from the Dynamic Property
 * lookup. This should be done once as you fetch the context, and then cached
 * as part of the context data structure, so it's cheap to access.
 * 3. If there's a clear and arguable inefficiency in using Dynamic Properties,
 * then use a Compile-Time Property (Platform Config, or GPU Compile-time
 * property). Examples of where this might be sensible follow:
 *  - Part of a critical inner-loop
 *  - Frequent re-use throughout the driver, causing significant extra load
 * instructions or control flow that would be worthwhile optimizing out.
 *
 * We cannot provide an exhaustive set of examples, neither can we provide a
 * rule for every possible situation. Use common sense, and think about: what
 * the rest of the driver will be doing; how the compiler might represent the
 * value if it is a compile-time constant; whether an OEM shipping multiple
 * devices would benefit much more from a single DDK binary, instead of
 * insignificant micro-optimizations.
 *
 * Dynamic GPU Properties
 *
 * Dynamic GPU properties are presented in two sets:
 * 1. the commonly used properties in @ref base_gpu_props, which have been
 * unpacked from GPU register bitfields.
 * 2. The full set of raw, unprocessed properties in gpu_raw_gpu_props
 * (also a member of base_gpu_props). All of these are presented in
 * the packed form, as presented by the GPU  registers themselves.
 *
 * The raw properties in gpu_raw_gpu_props are necessary to
 * allow a user of the Mali Tools (e.g. PAT) to determine "Why is this device
 * behaving differently?". In this case, all information about the
 * configuration is potentially useful, but it does not need to be processed
 * by the driver. Instead, the raw registers can be processed by the Mali
 * Tools software on the host PC.
 *
 * The properties returned extend the GPU Configuration Discovery
 * registers. For example, GPU clock speed is not specified in the GPU
 * Architecture, but is necessary for OpenCL's clGetDeviceInfo() function.
 *
 * The GPU properties are obtained by a call to
 * base_get_gpu_props(). This simply returns a pointer to a const
 * base_gpu_props structure. It is constant for the life of a base
 * context. Multiple calls to base_get_gpu_props() to a base context
 * return the same pointer to a constant structure. This avoids cache pollution
 * of the common data.
 *
 * This pointer must not be freed, because it does not point to the start of a
 * region allocated by the memory allocator; instead, just close the @ref
 * base_context.
 *
 *
 * Kernel Operation
 *
 * During Base Context Create time, user-side makes a single kernel call:
 * - A call to fill user memory with GPU information structures
 *
 * The kernel-side will fill the provided the entire processed base_gpu_props
 * structure, because this information is required in both
 * user and kernel side; it does not make sense to decode it twice.
 *
 * Coherency groups must be derived from the bitmasks, but this can be done
 * kernel side, and just once at kernel startup: Coherency groups must already
 * be known kernel-side, to support chains that specify a 'Only Coherent Group'
 * SW requirement, or 'Only Coherent Group with Tiler' SW requirement.
 *
 * Coherency Group calculation
 *
 * Creation of the coherent group data is done at device-driver startup, and so
 * is one-time. This will most likely involve a loop with CLZ, shifting, and
 * bit clearing on the L2_PRESENT mask, depending on whether the
 * system is L2 Coherent. The number of shader cores is done by a
 * population count, since faulty cores may be disabled during production,
 * producing a non-contiguous mask.
 *
 * The memory requirements for this algorithm can be determined either by a __u64
 * population count on the L2_PRESENT mask (a LUT helper already is
 * required for the above), or simple assumption that there can be no more than
 * 16 coherent groups, since core groups are typically 4 cores.
 */

/*
 * More information is possible - but associativity and bus width are not
 * required by upper-level apis.
 */
struct mali_base_gpu_l2_cache_props {
	__u8 log2_line_size;
	__u8 log2_cache_size;
	__u8 num_l2_slices; /* Number of L2C slices. 1 or higher */
	__u8 padding[5];
};

struct mali_base_gpu_tiler_props {
	__u32 bin_size_bytes;	/* Max is 4*2^15 */
	__u32 max_active_levels;	/* Max is 2^15 */
};

/**
 * struct mali_base_gpu_thread_props - GPU threading system details.
 * @max_threads: Max. number of threads per core
 * @max_workgroup_size:     Max. number of threads per workgroup
 * @max_barrier_size:       Max. number of threads that can synchronize on a
 *                          simple barrier
 * @max_registers:          Total size [1..65535] of the register file available
 *                          per core.
 * @max_task_queue:         Max. tasks [1..255] which may be sent to a core
 *                          before it becomes blocked.
 * @max_thread_group_split: Max. allowed value [1..15] of the Thread Group Split
 *                          field.
 * @impl_tech:              0 = Not specified, 1 = Silicon, 2 = FPGA,
 *                          3 = SW Model/Emulation
 * @padding:                padding to align to 8-byte
 * @tls_alloc:              Number of threads per core that TLS must be
 *                          allocated for
 */
struct mali_base_gpu_thread_props {
	__u32 max_threads;
	__u32 max_workgroup_size;
	__u32 max_barrier_size;
	__u16 max_registers;
	__u8 max_task_queue;
	__u8 max_thread_group_split;
	__u8 impl_tech;
	__u8  padding[3];
	__u32 tls_alloc;
};

/**
 * struct mali_base_gpu_coherent_group - descriptor for a coherent group
 * @core_mask: Core restriction mask required for the group
 * @num_cores: Number of cores in the group
 * @padding:   padding to align to 8-byte
 *
 * \c core_mask exposes all cores in that coherent group, and \c num_cores
 * provides a cached population-count for that mask.
 *
 * @note Whilst all cores are exposed in the mask, not all may be available to
 *       the application, depending on the Kernel Power policy.
 *
 * @note if u64s must be 8-byte aligned, then this structure has 32-bits of
 *       wastage.
 */
struct mali_base_gpu_coherent_group {
	__u64 core_mask;
	__u16 num_cores;
	__u16 padding[3];
};

/**
 * struct mali_base_gpu_coherent_group_info - Coherency group information
 * @num_groups: Number of coherent groups in the GPU.
 * @num_core_groups: Number of core groups (coherent or not) in the GPU.
 *                   Equivalent to the number of L2 Caches.
 *                   The GPU Counter dumping writes 2048 bytes per core group,
 *                   regardless of whether the core groups are coherent or not.
 *                   Hence this member is needed to calculate how much memory
 *                   is required for dumping.
 *                   @note Do not use it to work out how many valid elements
 *                         are in the group[] member. Use num_groups instead.
 * @coherency: Coherency features of the memory, accessed by gpu_mem_features
 *             methods
 * @padding: padding to align to 8-byte
 * @group: Descriptors of coherent groups
 *
 * Note that the sizes of the members could be reduced. However, the \c group
 * member might be 8-byte aligned to ensure the __u64 core_mask is 8-byte
 * aligned, thus leading to wastage if the other members sizes were reduced.
 *
 * The groups are sorted by core mask. The core masks are non-repeating and do
 * not intersect.
 */
struct mali_base_gpu_coherent_group_info {
	__u32 num_groups;
	__u32 num_core_groups;
	__u32 coherency;
	__u32 padding;
	struct mali_base_gpu_coherent_group group[BASE_MAX_COHERENT_GROUPS];
};

#if MALI_USE_CSF
#include "csf/mali_base_csf_kernel.h"
#else
#include "jm/mali_base_jm_kernel.h"
#endif

/**
 * struct gpu_raw_gpu_props - A complete description of the GPU's Hardware
 *                            Configuration Discovery registers.
 * @shader_present: Shader core present bitmap
 * @tiler_present: Tiler core present bitmap
 * @l2_present: Level 2 cache present bitmap
 * @stack_present: Core stack present bitmap
 * @l2_features: L2 features
 * @core_features: Core features
 * @mem_features: Mem features
 * @mmu_features: Mmu features
 * @as_present: Bitmap of address spaces present
 * @js_present: Job slots present
 * @js_features: Array of job slot features.
 * @tiler_features: Tiler features
 * @texture_features: TEXTURE_FEATURES_x registers, as exposed by the GPU
 * @gpu_id: GPU and revision identifier
 * @thread_max_threads: Maximum number of threads per core
 * @thread_max_workgroup_size: Maximum number of threads per workgroup
 * @thread_max_barrier_size: Maximum number of threads per barrier
 * @thread_features: Thread features
 * @coherency_mode: Note: This is the _selected_ coherency mode rather than the
 *                  available modes as exposed in the coherency_features register
 * @thread_tls_alloc: Number of threads per core that TLS must be allocated for
 * @gpu_features: GPU features
 *
 * The information is presented inefficiently for access. For frequent access,
 * the values should be better expressed in an unpacked form in the
 * base_gpu_props structure.
 *
 * The raw properties in gpu_raw_gpu_props are necessary to
 * allow a user of the Mali Tools (e.g. PAT) to determine "Why is this device
 * behaving differently?". In this case, all information about the
 * configuration is potentially useful, but it does not need to be processed
 * by the driver. Instead, the raw registers can be processed by the Mali
 * Tools software on the host PC.
 *
 */
struct gpu_raw_gpu_props {
	__u64 shader_present;
	__u64 tiler_present;
	__u64 l2_present;
	__u64 stack_present;
	__u32 l2_features;
	__u32 core_features;
	__u32 mem_features;
	__u32 mmu_features;

	__u32 as_present;

	__u32 js_present;
	__u32 js_features[GPU_MAX_JOB_SLOTS];
	__u32 tiler_features;
	__u32 texture_features[BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS];

	__u32 gpu_id;

	__u32 thread_max_threads;
	__u32 thread_max_workgroup_size;
	__u32 thread_max_barrier_size;
	__u32 thread_features;

	/*
	 * Note: This is the _selected_ coherency mode rather than the
	 * available modes as exposed in the coherency_features register.
	 */
	__u32 coherency_mode;

	__u32 thread_tls_alloc;
	__u64 gpu_features;
};

/**
 * struct base_gpu_props - Return structure for base_get_gpu_props().
 * @core_props:     Core props.
 * @l2_props:       L2 props.
 * @unused_1:       Keep for backwards compatibility.
 * @tiler_props:    Tiler props.
 * @thread_props:   Thread props.
 * @raw_props:      This member is large, likely to be 128 bytes.
 * @coherency_info: This must be last member of the structure.
 *
 * NOTE: the raw_props member in this data structure contains the register
 * values from which the value of the other members are derived. The derived
 * members exist to allow for efficient access and/or shielding the details
 * of the layout of the registers.
 */
struct base_gpu_props {
	struct mali_base_gpu_core_props core_props;
	struct mali_base_gpu_l2_cache_props l2_props;
	__u64 unused_1;
	struct mali_base_gpu_tiler_props tiler_props;
	struct mali_base_gpu_thread_props thread_props;
	struct gpu_raw_gpu_props raw_props;
	struct mali_base_gpu_coherent_group_info coherency_info;
};

#define BASE_MEM_GROUP_ID_GET(flags)                                           \
	((flags & BASE_MEM_GROUP_ID_MASK) >> BASEP_MEM_GROUP_ID_SHIFT)

#define BASE_MEM_GROUP_ID_SET(id)                                              \
	(((base_mem_alloc_flags)((id < 0 || id >= BASE_MEM_GROUP_COUNT) ?      \
					 BASE_MEM_GROUP_DEFAULT :              \
					 id)                                   \
	  << BASEP_MEM_GROUP_ID_SHIFT) &                                       \
	 BASE_MEM_GROUP_ID_MASK)

#define BASE_CONTEXT_MMU_GROUP_ID_SET(group_id)                                \
	(BASEP_CONTEXT_MMU_GROUP_ID_MASK &                                     \
	 ((base_context_create_flags)(group_id)                                \
	  << BASEP_CONTEXT_MMU_GROUP_ID_SHIFT))

#define BASE_CONTEXT_MMU_GROUP_ID_GET(flags)                                   \
	((flags & BASEP_CONTEXT_MMU_GROUP_ID_MASK) >>                          \
	 BASEP_CONTEXT_MMU_GROUP_ID_SHIFT)

/*
 * A number of bit flags are defined for requesting cpu_gpu_timeinfo. These
 * flags are also used, where applicable, for specifying which fields
 * are valid following the request operation.
 */

/* For monotonic (counter) timefield */
#define BASE_TIMEINFO_MONOTONIC_FLAG (1UL << 0)
/* For system wide timestamp */
#define BASE_TIMEINFO_TIMESTAMP_FLAG (1UL << 1)
/* For GPU cycle counter */
#define BASE_TIMEINFO_CYCLE_COUNTER_FLAG (1UL << 2)
/* Specify kernel GPU register timestamp */
#define BASE_TIMEINFO_KERNEL_SOURCE_FLAG (1UL << 30)
/* Specify userspace cntvct_el0 timestamp source */
#define BASE_TIMEINFO_USER_SOURCE_FLAG (1UL << 31)

#define BASE_TIMEREQUEST_ALLOWED_FLAGS (\
		BASE_TIMEINFO_MONOTONIC_FLAG | \
		BASE_TIMEINFO_TIMESTAMP_FLAG | \
		BASE_TIMEINFO_CYCLE_COUNTER_FLAG | \
		BASE_TIMEINFO_KERNEL_SOURCE_FLAG | \
		BASE_TIMEINFO_USER_SOURCE_FLAG)

/* Maximum number of source allocations allowed to create an alias allocation.
 * This needs to be 4096 * 6 to allow cube map arrays with up to 4096 array
 * layers, since each cube map in the array will have 6 faces.
 */
#define BASE_MEM_ALIAS_MAX_ENTS ((size_t)24576)

#endif /* _UAPI_BASE_KERNEL_H_ */
