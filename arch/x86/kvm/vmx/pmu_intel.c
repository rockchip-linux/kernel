// SPDX-License-Identifier: GPL-2.0-only
/*
 * KVM PMU support for Intel CPUs
 *
 * Copyright 2011 Red Hat, Inc. and/or its affiliates.
 *
 * Authors:
 *   Avi Kivity   <avi@redhat.com>
 *   Gleb Natapov <gleb@redhat.com>
 */
#include <linux/types.h>
#include <linux/kvm_host.h>
#include <linux/perf_event.h>
#include <asm/perf_event.h>
#include "x86.h"
#include "cpuid.h"
#include "lapic.h"
#include "nested.h"
#include "pmu.h"

#define MSR_PMC_FULL_WIDTH_BIT      (MSR_IA32_PMC0 - MSR_IA32_PERFCTR0)

static struct kvm_event_hw_type_mapping intel_arch_events[] = {
	/* Index must match CPUID 0x0A.EBX bit vector */
	[0] = { 0x3c, 0x00, PERF_COUNT_HW_CPU_CYCLES },
	[1] = { 0xc0, 0x00, PERF_COUNT_HW_INSTRUCTIONS },
	[2] = { 0x3c, 0x01, PERF_COUNT_HW_BUS_CYCLES  },
	[3] = { 0x2e, 0x4f, PERF_COUNT_HW_CACHE_REFERENCES },
	[4] = { 0x2e, 0x41, PERF_COUNT_HW_CACHE_MISSES },
	[5] = { 0xc4, 0x00, PERF_COUNT_HW_BRANCH_INSTRUCTIONS },
	[6] = { 0xc5, 0x00, PERF_COUNT_HW_BRANCH_MISSES },
	[7] = { 0x00, 0x03, PERF_COUNT_HW_REF_CPU_CYCLES },
};

/* mapping between fixed pmc index and intel_arch_events array */
static int fixed_pmc_events[] = {1, 0, 7};

static void reprogram_fixed_counters(struct kvm_pmu *pmu, u64 data)
{
	int i;

	for (i = 0; i < pmu->nr_arch_fixed_counters; i++) {
		u8 new_ctrl = fixed_ctrl_field(data, i);
		u8 old_ctrl = fixed_ctrl_field(pmu->fixed_ctr_ctrl, i);
		struct kvm_pmc *pmc;

		pmc = get_fixed_pmc(pmu, MSR_CORE_PERF_FIXED_CTR0 + i);

		if (old_ctrl == new_ctrl)
			continue;

		__set_bit(INTEL_PMC_IDX_FIXED + i, pmu->pmc_in_use);
		reprogram_fixed_counter(pmc, new_ctrl, i);
	}

	pmu->fixed_ctr_ctrl = data;
}

/* function is called when global control register has been updated. */
static void global_ctrl_changed(struct kvm_pmu *pmu, u64 data)
{
	int bit;
	u64 diff = pmu->global_ctrl ^ data;

	pmu->global_ctrl = data;

	for_each_set_bit(bit, (unsigned long *)&diff, X86_PMC_IDX_MAX)
		reprogram_counter(pmu, bit);
}

static unsigned int intel_pmc_perf_hw_id(struct kvm_pmc *pmc)
{
	struct kvm_pmu *pmu = pmc_to_pmu(pmc);
	u8 event_select = pmc->eventsel & ARCH_PERFMON_EVENTSEL_EVENT;
	u8 unit_mask = (pmc->eventsel & ARCH_PERFMON_EVENTSEL_UMASK) >> 8;
	int i;

	for (i = 0; i < ARRAY_SIZE(intel_arch_events); i++)
		if (intel_arch_events[i].eventsel == event_select
		    && intel_arch_events[i].unit_mask == unit_mask
		    && (pmu->available_event_types & (1 << i)))
			break;

	if (i == ARRAY_SIZE(intel_arch_events))
		return PERF_COUNT_HW_MAX;

	return intel_arch_events[i].event_type;
}

static unsigned intel_find_fixed_event(int idx)
{
	u32 event;
	size_t size = ARRAY_SIZE(fixed_pmc_events);

	if (idx >= size)
		return PERF_COUNT_HW_MAX;

	event = fixed_pmc_events[array_index_nospec(idx, size)];
	return intel_arch_events[event].event_type;
}

/* check if a PMC is enabled by comparing it with globl_ctrl bits. */
static bool intel_pmc_is_enabled(struct kvm_pmc *pmc)
{
	struct kvm_pmu *pmu = pmc_to_pmu(pmc);

	if (pmu->version < 2)
		return true;

	return test_bit(pmc->idx, (unsigned long *)&pmu->global_ctrl);
}

static struct kvm_pmc *intel_pmc_idx_to_pmc(struct kvm_pmu *pmu, int pmc_idx)
{
	if (pmc_idx < INTEL_PMC_IDX_FIXED)
		return get_gp_pmc(pmu, MSR_P6_EVNTSEL0 + pmc_idx,
				  MSR_P6_EVNTSEL0);
	else {
		u32 idx = pmc_idx - INTEL_PMC_IDX_FIXED;

		return get_fixed_pmc(pmu, idx + MSR_CORE_PERF_FIXED_CTR0);
	}
}

/* returns 0 if idx's corresponding MSR exists; otherwise returns 1. */
static int intel_is_valid_rdpmc_ecx(struct kvm_vcpu *vcpu, unsigned int idx)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	bool fixed = idx & (1u << 30);

	idx &= ~(3u << 30);

	return (!fixed && idx >= pmu->nr_arch_gp_counters) ||
		(fixed && idx >= pmu->nr_arch_fixed_counters);
}

static struct kvm_pmc *intel_rdpmc_ecx_to_pmc(struct kvm_vcpu *vcpu,
					    unsigned int idx, u64 *mask)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	bool fixed = idx & (1u << 30);
	struct kvm_pmc *counters;
	unsigned int num_counters;

	idx &= ~(3u << 30);
	if (fixed) {
		counters = pmu->fixed_counters;
		num_counters = pmu->nr_arch_fixed_counters;
	} else {
		counters = pmu->gp_counters;
		num_counters = pmu->nr_arch_gp_counters;
	}
	if (idx >= num_counters)
		return NULL;
	*mask &= pmu->counter_bitmask[fixed ? KVM_PMC_FIXED : KVM_PMC_GP];
	return &counters[array_index_nospec(idx, num_counters)];
}

static inline u64 vcpu_get_perf_capabilities(struct kvm_vcpu *vcpu)
{
	if (!guest_cpuid_has(vcpu, X86_FEATURE_PDCM))
		return 0;

	return vcpu->arch.perf_capabilities;
}

static inline bool fw_writes_is_enabled(struct kvm_vcpu *vcpu)
{
	return (vcpu_get_perf_capabilities(vcpu) & PMU_CAP_FW_WRITES) != 0;
}

static inline struct kvm_pmc *get_fw_gp_pmc(struct kvm_pmu *pmu, u32 msr)
{
	if (!fw_writes_is_enabled(pmu_to_vcpu(pmu)))
		return NULL;

	return get_gp_pmc(pmu, msr, MSR_IA32_PMC0);
}

static bool intel_is_valid_msr(struct kvm_vcpu *vcpu, u32 msr)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	int ret;

	switch (msr) {
	case MSR_CORE_PERF_FIXED_CTR_CTRL:
	case MSR_CORE_PERF_GLOBAL_STATUS:
	case MSR_CORE_PERF_GLOBAL_CTRL:
	case MSR_CORE_PERF_GLOBAL_OVF_CTRL:
		ret = pmu->version > 1;
		break;
	default:
		ret = get_gp_pmc(pmu, msr, MSR_IA32_PERFCTR0) ||
			get_gp_pmc(pmu, msr, MSR_P6_EVNTSEL0) ||
			get_fixed_pmc(pmu, msr) || get_fw_gp_pmc(pmu, msr);
		break;
	}

	return ret;
}

static struct kvm_pmc *intel_msr_idx_to_pmc(struct kvm_vcpu *vcpu, u32 msr)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct kvm_pmc *pmc;

	pmc = get_fixed_pmc(pmu, msr);
	pmc = pmc ? pmc : get_gp_pmc(pmu, msr, MSR_P6_EVNTSEL0);
	pmc = pmc ? pmc : get_gp_pmc(pmu, msr, MSR_IA32_PERFCTR0);

	return pmc;
}

static int intel_pmu_get_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct kvm_pmc *pmc;
	u32 msr = msr_info->index;

	switch (msr) {
	case MSR_CORE_PERF_FIXED_CTR_CTRL:
		msr_info->data = pmu->fixed_ctr_ctrl;
		return 0;
	case MSR_CORE_PERF_GLOBAL_STATUS:
		msr_info->data = pmu->global_status;
		return 0;
	case MSR_CORE_PERF_GLOBAL_CTRL:
		msr_info->data = pmu->global_ctrl;
		return 0;
	case MSR_CORE_PERF_GLOBAL_OVF_CTRL:
		msr_info->data = pmu->global_ovf_ctrl;
		return 0;
	default:
		if ((pmc = get_gp_pmc(pmu, msr, MSR_IA32_PERFCTR0)) ||
		    (pmc = get_gp_pmc(pmu, msr, MSR_IA32_PMC0))) {
			u64 val = pmc_read_counter(pmc);
			msr_info->data =
				val & pmu->counter_bitmask[KVM_PMC_GP];
			return 0;
		} else if ((pmc = get_fixed_pmc(pmu, msr))) {
			u64 val = pmc_read_counter(pmc);
			msr_info->data =
				val & pmu->counter_bitmask[KVM_PMC_FIXED];
			return 0;
		} else if ((pmc = get_gp_pmc(pmu, msr, MSR_P6_EVNTSEL0))) {
			msr_info->data = pmc->eventsel;
			return 0;
		}
	}

	return 1;
}

static int intel_pmu_set_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct kvm_pmc *pmc;
	u32 msr = msr_info->index;
	u64 data = msr_info->data;

	switch (msr) {
	case MSR_CORE_PERF_FIXED_CTR_CTRL:
		if (pmu->fixed_ctr_ctrl == data)
			return 0;
		if (!(data & pmu->fixed_ctr_ctrl_mask)) {
			reprogram_fixed_counters(pmu, data);
			return 0;
		}
		break;
	case MSR_CORE_PERF_GLOBAL_STATUS:
		if (msr_info->host_initiated) {
			pmu->global_status = data;
			return 0;
		}
		break; /* RO MSR */
	case MSR_CORE_PERF_GLOBAL_CTRL:
		if (pmu->global_ctrl == data)
			return 0;
		if (kvm_valid_perf_global_ctrl(pmu, data)) {
			global_ctrl_changed(pmu, data);
			return 0;
		}
		break;
	case MSR_CORE_PERF_GLOBAL_OVF_CTRL:
		if (!(data & pmu->global_ovf_ctrl_mask)) {
			if (!msr_info->host_initiated)
				pmu->global_status &= ~data;
			pmu->global_ovf_ctrl = data;
			return 0;
		}
		break;
	default:
		if ((pmc = get_gp_pmc(pmu, msr, MSR_IA32_PERFCTR0)) ||
		    (pmc = get_gp_pmc(pmu, msr, MSR_IA32_PMC0))) {
			if ((msr & MSR_PMC_FULL_WIDTH_BIT) &&
			    (data & ~pmu->counter_bitmask[KVM_PMC_GP]))
				return 1;
			if (!msr_info->host_initiated &&
			    !(msr & MSR_PMC_FULL_WIDTH_BIT))
				data = (s64)(s32)data;
			pmc->counter += data - pmc_read_counter(pmc);
			if (pmc->perf_event)
				perf_event_period(pmc->perf_event,
						  get_sample_period(pmc, data));
			return 0;
		} else if ((pmc = get_fixed_pmc(pmu, msr))) {
			pmc->counter += data - pmc_read_counter(pmc);
			if (pmc->perf_event)
				perf_event_period(pmc->perf_event,
						  get_sample_period(pmc, data));
			return 0;
		} else if ((pmc = get_gp_pmc(pmu, msr, MSR_P6_EVNTSEL0))) {
			if (data == pmc->eventsel)
				return 0;
			if (!(data & pmu->reserved_bits)) {
				reprogram_gp_counter(pmc, data);
				return 0;
			}
		}
	}

	return 1;
}

static void intel_pmu_refresh(struct kvm_vcpu *vcpu)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct x86_pmu_capability x86_pmu;
	struct kvm_cpuid_entry2 *entry;
	union cpuid10_eax eax;
	union cpuid10_edx edx;
	int i;

	pmu->nr_arch_gp_counters = 0;
	pmu->nr_arch_fixed_counters = 0;
	pmu->counter_bitmask[KVM_PMC_GP] = 0;
	pmu->counter_bitmask[KVM_PMC_FIXED] = 0;
	pmu->version = 0;
	pmu->reserved_bits = 0xffffffff00200000ull;
	pmu->raw_event_mask = X86_RAW_EVENT_MASK;
	pmu->global_ctrl_mask = ~0ull;
	pmu->global_ovf_ctrl_mask = ~0ull;
	pmu->fixed_ctr_ctrl_mask = ~0ull;

	entry = kvm_find_cpuid_entry(vcpu, 0xa, 0);
	if (!entry)
		return;
	eax.full = entry->eax;
	edx.full = entry->edx;

	pmu->version = eax.split.version_id;
	if (!pmu->version)
		return;

	perf_get_x86_pmu_capability(&x86_pmu);

	pmu->nr_arch_gp_counters = min_t(int, eax.split.num_counters,
					 x86_pmu.num_counters_gp);
	eax.split.bit_width = min_t(int, eax.split.bit_width, x86_pmu.bit_width_gp);
	pmu->counter_bitmask[KVM_PMC_GP] = ((u64)1 << eax.split.bit_width) - 1;
	eax.split.mask_length = min_t(int, eax.split.mask_length, x86_pmu.events_mask_len);
	pmu->available_event_types = ~entry->ebx &
					((1ull << eax.split.mask_length) - 1);

	if (pmu->version == 1) {
		pmu->nr_arch_fixed_counters = 0;
	} else {
		pmu->nr_arch_fixed_counters =
			min_t(int, edx.split.num_counters_fixed,
			      x86_pmu.num_counters_fixed);
		edx.split.bit_width_fixed = min_t(int,
			edx.split.bit_width_fixed, x86_pmu.bit_width_fixed);
		pmu->counter_bitmask[KVM_PMC_FIXED] =
			((u64)1 << edx.split.bit_width_fixed) - 1;
	}

	for (i = 0; i < pmu->nr_arch_fixed_counters; i++)
		pmu->fixed_ctr_ctrl_mask &= ~(0xbull << (i * 4));
	pmu->global_ctrl = ((1ull << pmu->nr_arch_gp_counters) - 1) |
		(((1ull << pmu->nr_arch_fixed_counters) - 1) << INTEL_PMC_IDX_FIXED);
	pmu->global_ctrl_mask = ~pmu->global_ctrl;
	pmu->global_ovf_ctrl_mask = pmu->global_ctrl_mask
			& ~(MSR_CORE_PERF_GLOBAL_OVF_CTRL_OVF_BUF |
			    MSR_CORE_PERF_GLOBAL_OVF_CTRL_COND_CHGD);
	if (vmx_pt_mode_is_host_guest())
		pmu->global_ovf_ctrl_mask &=
				~MSR_CORE_PERF_GLOBAL_OVF_CTRL_TRACE_TOPA_PMI;

	entry = kvm_find_cpuid_entry(vcpu, 7, 0);
	if (entry &&
	    (boot_cpu_has(X86_FEATURE_HLE) || boot_cpu_has(X86_FEATURE_RTM)) &&
	    (entry->ebx & (X86_FEATURE_HLE|X86_FEATURE_RTM)))
		pmu->reserved_bits ^= HSW_IN_TX|HSW_IN_TX_CHECKPOINTED;

	bitmap_set(pmu->all_valid_pmc_idx,
		0, pmu->nr_arch_gp_counters);
	bitmap_set(pmu->all_valid_pmc_idx,
		INTEL_PMC_MAX_GENERIC, pmu->nr_arch_fixed_counters);

	nested_vmx_pmu_entry_exit_ctls_update(vcpu);
}

static void intel_pmu_init(struct kvm_vcpu *vcpu)
{
	int i;
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);

	for (i = 0; i < INTEL_PMC_MAX_GENERIC; i++) {
		pmu->gp_counters[i].type = KVM_PMC_GP;
		pmu->gp_counters[i].vcpu = vcpu;
		pmu->gp_counters[i].idx = i;
		pmu->gp_counters[i].current_config = 0;
	}

	for (i = 0; i < INTEL_PMC_MAX_FIXED; i++) {
		pmu->fixed_counters[i].type = KVM_PMC_FIXED;
		pmu->fixed_counters[i].vcpu = vcpu;
		pmu->fixed_counters[i].idx = i + INTEL_PMC_IDX_FIXED;
		pmu->fixed_counters[i].current_config = 0;
	}

	vcpu->arch.perf_capabilities = vmx_get_perf_capabilities();
}

static void intel_pmu_reset(struct kvm_vcpu *vcpu)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct kvm_pmc *pmc = NULL;
	int i;

	for (i = 0; i < INTEL_PMC_MAX_GENERIC; i++) {
		pmc = &pmu->gp_counters[i];

		pmc_stop_counter(pmc);
		pmc->counter = pmc->eventsel = 0;
	}

	for (i = 0; i < INTEL_PMC_MAX_FIXED; i++) {
		pmc = &pmu->fixed_counters[i];

		pmc_stop_counter(pmc);
		pmc->counter = 0;
	}

	pmu->fixed_ctr_ctrl = pmu->global_ctrl = pmu->global_status =
		pmu->global_ovf_ctrl = 0;
}

struct kvm_pmu_ops intel_pmu_ops = {
	.pmc_perf_hw_id = intel_pmc_perf_hw_id,
	.find_fixed_event = intel_find_fixed_event,
	.pmc_is_enabled = intel_pmc_is_enabled,
	.pmc_idx_to_pmc = intel_pmc_idx_to_pmc,
	.rdpmc_ecx_to_pmc = intel_rdpmc_ecx_to_pmc,
	.msr_idx_to_pmc = intel_msr_idx_to_pmc,
	.is_valid_rdpmc_ecx = intel_is_valid_rdpmc_ecx,
	.is_valid_msr = intel_is_valid_msr,
	.get_msr = intel_pmu_get_msr,
	.set_msr = intel_pmu_set_msr,
	.refresh = intel_pmu_refresh,
	.init = intel_pmu_init,
	.reset = intel_pmu_reset,
};
