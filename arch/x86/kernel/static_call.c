// SPDX-License-Identifier: GPL-2.0
#include <linux/static_call.h>
#include <linux/memory.h>
#include <linux/bug.h>
#include <asm/text-patching.h>

enum insn_type {
	CALL = 0, /* site call */
	NOP = 1,  /* site cond-call */
	JMP = 2,  /* tramp / site tail-call */
	RET = 3,  /* tramp / site cond-tail-call */
};

/*
 * ud1 %esp, %ecx - a 3 byte #UD that is unique to trampolines, chosen such
 * that there is no false-positive trampoline identification while also being a
 * speculation stop.
 */
static const u8 tramp_ud[] = { 0x0f, 0xb9, 0xcc };

static const u8 retinsn[] = { RET_INSN_OPCODE, 0xcc, 0xcc, 0xcc, 0xcc };

static void __ref __static_call_transform(void *insn, enum insn_type type,
					  void *func, bool modinit)
{
	int size = CALL_INSN_SIZE;
	const void *code;

	switch (type) {
	case CALL:
		code = text_gen_insn(CALL_INSN_OPCODE, insn, func);
		break;

	case NOP:
		code = ideal_nops[NOP_ATOMIC5];
		break;

	case JMP:
		code = text_gen_insn(JMP32_INSN_OPCODE, insn, func);
		break;

	case RET:
		if (cpu_feature_enabled(X86_FEATURE_RETHUNK))
			code = text_gen_insn(JMP32_INSN_OPCODE, insn, &__x86_return_thunk);
		else
			code = &retinsn;
		break;
	}

	if (memcmp(insn, code, size) == 0)
		return;

	if (system_state == SYSTEM_BOOTING || modinit)
		return text_poke_early(insn, code, size);

	text_poke_bp(insn, code, size, NULL);
}

static void __static_call_validate(void *insn, bool tail)
{
	u8 opcode = *(u8 *)insn;

	if (tail) {
		if (opcode == JMP32_INSN_OPCODE ||
		    opcode == RET_INSN_OPCODE)
			return;
	} else {
		if (opcode == CALL_INSN_OPCODE ||
		    !memcmp(insn, ideal_nops[NOP_ATOMIC5], 5))
			return;
	}

	/*
	 * If we ever trigger this, our text is corrupt, we'll probably not live long.
	 */
	WARN_ONCE(1, "unexpected static_call insn opcode 0x%x at %pS\n", opcode, insn);
}

static inline enum insn_type __sc_insn(bool null, bool tail)
{
	/*
	 * Encode the following table without branches:
	 *
	 *	tail	null	insn
	 *	-----+-------+------
	 *	  0  |   0   |  CALL
	 *	  0  |   1   |  NOP
	 *	  1  |   0   |  JMP
	 *	  1  |   1   |  RET
	 */
	return 2*tail + null;
}

void arch_static_call_transform(void *site, void *tramp, void *func, bool tail)
{
	mutex_lock(&text_mutex);

	if (tramp) {
		__static_call_validate(tramp, true);
		__static_call_transform(tramp, __sc_insn(!func, true), func, false);
	}

	if (IS_ENABLED(CONFIG_HAVE_STATIC_CALL_INLINE) && site) {
		__static_call_validate(site, tail);
		__static_call_transform(site, __sc_insn(!func, tail), func, false);
	}

	mutex_unlock(&text_mutex);
}
EXPORT_SYMBOL_GPL(arch_static_call_transform);

#ifdef CONFIG_RETHUNK
/*
 * This is called by apply_returns() to fix up static call trampolines,
 * specifically ARCH_DEFINE_STATIC_CALL_NULL_TRAMP which is recorded as
 * having a return trampoline.
 *
 * The problem is that static_call() is available before determining
 * X86_FEATURE_RETHUNK and, by implication, running alternatives.
 *
 * This means that __static_call_transform() above can have overwritten the
 * return trampoline and we now need to fix things up to be consistent.
 */
bool __static_call_fixup(void *tramp, u8 op, void *dest)
{
	unsigned long addr = (unsigned long)tramp;
	/*
	 * Not all .return_sites are a static_call trampoline (most are not).
	 * Check if the 3 bytes after the return are still kernel text, if not,
	 * then this definitely is not a trampoline and we need not worry
	 * further.
	 *
	 * This avoids the memcmp() below tripping over pagefaults etc..
	 */
	if (((addr >> PAGE_SHIFT) != ((addr + 7) >> PAGE_SHIFT)) &&
	    !kernel_text_address(addr + 7))
		return false;

	if (memcmp(tramp+5, tramp_ud, 3)) {
		/* Not a trampoline site, not our problem. */
		return false;
	}

	mutex_lock(&text_mutex);
	if (op == RET_INSN_OPCODE || dest == &__x86_return_thunk)
		__static_call_transform(tramp, RET, NULL, true);
	mutex_unlock(&text_mutex);

	return true;
}
#endif
