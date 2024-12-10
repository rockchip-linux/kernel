// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 SiFive
 */

#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/memory.h>
#include <linux/uaccess.h>
#include <linux/stop_machine.h>
#include <asm/kprobes.h>
#include <asm/cacheflush.h>
#include <asm/fixmap.h>
#include <asm/ftrace.h>
#include <asm/patch.h>

struct patch_insn {
	void *addr;
	u32 insn;
	atomic_t cpu_count;
};

int riscv_patch_in_stop_machine = false;

#ifdef CONFIG_MMU
static void *patch_map(void *addr, int fixmap)
{
	uintptr_t uintaddr = (uintptr_t) addr;
	struct page *page;

	if (core_kernel_text(uintaddr))
		page = phys_to_page(__pa_symbol(addr));
	else if (IS_ENABLED(CONFIG_STRICT_MODULE_RWX))
		page = vmalloc_to_page(addr);
	else
		return addr;

	BUG_ON(!page);

	return (void *)set_fixmap_offset(fixmap, page_to_phys(page) +
					 (uintaddr & ~PAGE_MASK));
}
NOKPROBE_SYMBOL(patch_map);

static void patch_unmap(int fixmap)
{
	clear_fixmap(fixmap);
}
NOKPROBE_SYMBOL(patch_unmap);

static int patch_insn_write(void *addr, const void *insn, size_t len)
{
	void *waddr = addr;
	bool across_pages = (((uintptr_t) addr & ~PAGE_MASK) + len) > PAGE_SIZE;
	int ret;

	/*
	 * Before reaching here, it was expected to lock the text_mutex
	 * already, so we don't need to give another lock here and could
	 * ensure that it was safe between each cores.
	 *
	 * We're currently using stop_machine() for ftrace & kprobes, and while
	 * that ensures text_mutex is held before installing the mappings it
	 * does not ensure text_mutex is held by the calling thread.  That's
	 * safe but triggers a lockdep failure, so just elide it for that
	 * specific case.
	 */
	if (!riscv_patch_in_stop_machine)
		lockdep_assert_held(&text_mutex);

	if (across_pages)
		patch_map(addr + len, FIX_TEXT_POKE1);

	waddr = patch_map(addr, FIX_TEXT_POKE0);

	ret = copy_to_kernel_nofault(waddr, insn, len);

	patch_unmap(FIX_TEXT_POKE0);

	if (across_pages)
		patch_unmap(FIX_TEXT_POKE1);

	return ret;
}
NOKPROBE_SYMBOL(patch_insn_write);
#else
static int patch_insn_write(void *addr, const void *insn, size_t len)
{
	return copy_to_kernel_nofault(addr, insn, len);
}
NOKPROBE_SYMBOL(patch_insn_write);
#endif /* CONFIG_MMU */

int patch_text_nosync(void *addr, const void *insns, size_t len)
{
	u32 *tp = addr;
	int ret;

	ret = patch_insn_write(tp, insns, len);

	if (!ret)
		flush_icache_range((uintptr_t) tp, (uintptr_t) tp + len);

	return ret;
}
NOKPROBE_SYMBOL(patch_text_nosync);

static int patch_text_cb(void *data)
{
	struct patch_insn *patch = data;
	int ret = 0;

	if (atomic_inc_return(&patch->cpu_count) == num_online_cpus()) {
		ret =
		    patch_text_nosync(patch->addr, &patch->insn,
					    GET_INSN_LENGTH(patch->insn));
		atomic_inc(&patch->cpu_count);
	} else {
		while (atomic_read(&patch->cpu_count) <= num_online_cpus())
			cpu_relax();
		smp_mb();
	}

	return ret;
}
NOKPROBE_SYMBOL(patch_text_cb);

int patch_text(void *addr, u32 insn)
{
	int ret;
	struct patch_insn patch = {
		.addr = addr,
		.insn = insn,
		.cpu_count = ATOMIC_INIT(0),
	};

	/*
	 * kprobes takes text_mutex, before calling patch_text(), but as we call
	 * calls stop_machine(), the lockdep assertion in patch_insn_write()
	 * gets confused by the context in which the lock is taken.
	 * Instead, ensure the lock is held before calling stop_machine(), and
	 * set riscv_patch_in_stop_machine to skip the check in
	 * patch_insn_write().
	 */
	lockdep_assert_held(&text_mutex);
	riscv_patch_in_stop_machine = true;
	ret = stop_machine_cpuslocked(patch_text_cb, &patch, cpu_online_mask);
	riscv_patch_in_stop_machine = false;
	return ret;
}
NOKPROBE_SYMBOL(patch_text);
