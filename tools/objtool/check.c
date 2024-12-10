// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2015-2017 Josh Poimboeuf <jpoimboe@redhat.com>
 */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <sys/mman.h>

#include "builtin.h"
#include "cfi.h"
#include "arch.h"
#include "check.h"
#include "special.h"
#include "warn.h"
#include "arch_elf.h"

#include <linux/objtool.h>
#include <linux/hashtable.h>
#include <linux/kernel.h>
#include <linux/static_call_types.h>

struct alternative {
	struct list_head list;
	struct instruction *insn;
	bool skip_orig;
};

static unsigned long nr_cfi, nr_cfi_reused, nr_cfi_cache;

static struct cfi_init_state initial_func_cfi;
static struct cfi_state init_cfi;
static struct cfi_state func_cfi;

struct instruction *find_insn(struct objtool_file *file,
			      struct section *sec, unsigned long offset)
{
	struct instruction *insn;

	hash_for_each_possible(file->insn_hash, insn, hash, sec_offset_hash(sec, offset)) {
		if (insn->sec == sec && insn->offset == offset)
			return insn;
	}

	return NULL;
}

static struct instruction *next_insn_same_sec(struct objtool_file *file,
					      struct instruction *insn)
{
	struct instruction *next = list_next_entry(insn, list);

	if (!next || &next->list == &file->insn_list || next->sec != insn->sec)
		return NULL;

	return next;
}

static struct instruction *next_insn_same_func(struct objtool_file *file,
					       struct instruction *insn)
{
	struct instruction *next = list_next_entry(insn, list);
	struct symbol *func = insn->func;

	if (!func)
		return NULL;

	if (&next->list != &file->insn_list && next->func == func)
		return next;

	/* Check if we're already in the subfunction: */
	if (func == func->cfunc)
		return NULL;

	/* Move to the subfunction: */
	return find_insn(file, func->cfunc->sec, func->cfunc->offset);
}

static struct instruction *prev_insn_same_sym(struct objtool_file *file,
					       struct instruction *insn)
{
	struct instruction *prev = list_prev_entry(insn, list);

	if (&prev->list != &file->insn_list && prev->func == insn->func)
		return prev;

	return NULL;
}

#define func_for_each_insn(file, func, insn)				\
	for (insn = find_insn(file, func->sec, func->offset);		\
	     insn;							\
	     insn = next_insn_same_func(file, insn))

#define sym_for_each_insn(file, sym, insn)				\
	for (insn = find_insn(file, sym->sec, sym->offset);		\
	     insn && &insn->list != &file->insn_list &&			\
		insn->sec == sym->sec &&				\
		insn->offset < sym->offset + sym->len;			\
	     insn = list_next_entry(insn, list))

#define sym_for_each_insn_continue_reverse(file, sym, insn)		\
	for (insn = list_prev_entry(insn, list);			\
	     &insn->list != &file->insn_list &&				\
		insn->sec == sym->sec && insn->offset >= sym->offset;	\
	     insn = list_prev_entry(insn, list))

#define sec_for_each_insn_from(file, insn)				\
	for (; insn; insn = next_insn_same_sec(file, insn))

#define sec_for_each_insn_continue(file, insn)				\
	for (insn = next_insn_same_sec(file, insn); insn;		\
	     insn = next_insn_same_sec(file, insn))

static bool is_jump_table_jump(struct instruction *insn)
{
	struct alt_group *alt_group = insn->alt_group;

	if (insn->jump_table)
		return true;

	/* Retpoline alternative for a jump table? */
	return alt_group && alt_group->orig_group &&
	       alt_group->orig_group->first_insn->jump_table;
}

static bool is_sibling_call(struct instruction *insn)
{
	/*
	 * Assume only ELF functions can make sibling calls.  This ensures
	 * sibling call detection consistency between vmlinux.o and individual
	 * objects.
	 */
	if (!insn->func)
		return false;

	/* An indirect jump is either a sibling call or a jump to a table. */
	if (insn->type == INSN_JUMP_DYNAMIC)
		return !is_jump_table_jump(insn);

	/* add_jump_destinations() sets insn->call_dest for sibling calls. */
	return (is_static_jump(insn) && insn->call_dest);
}

/*
 * This checks to see if the given function is a "noreturn" function.
 *
 * For global functions which are outside the scope of this object file, we
 * have to keep a manual list of them.
 *
 * For local functions, we have to detect them manually by simply looking for
 * the lack of a return instruction.
 */
static bool __dead_end_function(struct objtool_file *file, struct symbol *func,
				int recursion)
{
	int i;
	struct instruction *insn;
	bool empty = true;

	/*
	 * Unfortunately these have to be hard coded because the noreturn
	 * attribute isn't provided in ELF data.
	 */
	static const char * const global_noreturns[] = {
		"__stack_chk_fail",
		"panic",
		"do_exit",
		"do_task_dead",
		"make_task_dead",
		"__module_put_and_exit",
		"complete_and_exit",
		"__reiserfs_panic",
		"lbug_with_loc",
		"fortify_panic",
		"usercopy_abort",
		"machine_real_restart",
		"rewind_stack_and_make_dead",
		"kunit_try_catch_throw",
		"xen_start_kernel",
		"cpu_bringup_and_idle",
		"stop_this_cpu",
	};

	if (!func)
		return false;

	if (func->bind == STB_WEAK)
		return false;

	if (func->bind == STB_GLOBAL)
		for (i = 0; i < ARRAY_SIZE(global_noreturns); i++)
			if (!strcmp(func->name, global_noreturns[i]))
				return true;

	if (!func->len)
		return false;

	insn = find_insn(file, func->sec, func->offset);
	if (!insn || !insn->func)
		return false;

	func_for_each_insn(file, func, insn) {
		empty = false;

		if (insn->type == INSN_RETURN)
			return false;
	}

	if (empty)
		return false;

	/*
	 * A function can have a sibling call instead of a return.  In that
	 * case, the function's dead-end status depends on whether the target
	 * of the sibling call returns.
	 */
	func_for_each_insn(file, func, insn) {
		if (is_sibling_call(insn)) {
			struct instruction *dest = insn->jump_dest;

			if (!dest)
				/* sibling call to another file */
				return false;

			/* local sibling call */
			if (recursion == 5) {
				/*
				 * Infinite recursion: two functions have
				 * sibling calls to each other.  This is a very
				 * rare case.  It means they aren't dead ends.
				 */
				return false;
			}

			return __dead_end_function(file, dest->func, recursion+1);
		}
	}

	return true;
}

static bool dead_end_function(struct objtool_file *file, struct symbol *func)
{
	return __dead_end_function(file, func, 0);
}

static void init_cfi_state(struct cfi_state *cfi)
{
	int i;

	for (i = 0; i < CFI_NUM_REGS; i++) {
		cfi->regs[i].base = CFI_UNDEFINED;
		cfi->vals[i].base = CFI_UNDEFINED;
	}
	cfi->cfa.base = CFI_UNDEFINED;
	cfi->drap_reg = CFI_UNDEFINED;
	cfi->drap_offset = -1;
}

static void init_insn_state(struct insn_state *state, struct section *sec)
{
	memset(state, 0, sizeof(*state));
	init_cfi_state(&state->cfi);

	/*
	 * We need the full vmlinux for noinstr validation, otherwise we can
	 * not correctly determine insn->call_dest->sec (external symbols do
	 * not have a section).
	 */
	if (vmlinux && noinstr && sec)
		state->noinstr = sec->noinstr;
}

static struct cfi_state *cfi_alloc(void)
{
	struct cfi_state *cfi = calloc(sizeof(struct cfi_state), 1);
	if (!cfi) {
		WARN("calloc failed");
		exit(1);
	}
	nr_cfi++;
	return cfi;
}

static int cfi_bits;
static struct hlist_head *cfi_hash;

static inline bool cficmp(struct cfi_state *cfi1, struct cfi_state *cfi2)
{
	return memcmp((void *)cfi1 + sizeof(cfi1->hash),
		      (void *)cfi2 + sizeof(cfi2->hash),
		      sizeof(struct cfi_state) - sizeof(struct hlist_node));
}

static inline u32 cfi_key(struct cfi_state *cfi)
{
	return jhash((void *)cfi + sizeof(cfi->hash),
		     sizeof(*cfi) - sizeof(cfi->hash), 0);
}

static struct cfi_state *cfi_hash_find_or_add(struct cfi_state *cfi)
{
	struct hlist_head *head = &cfi_hash[hash_min(cfi_key(cfi), cfi_bits)];
	struct cfi_state *obj;

	hlist_for_each_entry(obj, head, hash) {
		if (!cficmp(cfi, obj)) {
			nr_cfi_cache++;
			return obj;
		}
	}

	obj = cfi_alloc();
	*obj = *cfi;
	hlist_add_head(&obj->hash, head);

	return obj;
}

static void cfi_hash_add(struct cfi_state *cfi)
{
	struct hlist_head *head = &cfi_hash[hash_min(cfi_key(cfi), cfi_bits)];

	hlist_add_head(&cfi->hash, head);
}

static void *cfi_hash_alloc(void)
{
	cfi_bits = vmlinux ? ELF_HASH_BITS - 3 : 13;
	cfi_hash = mmap(NULL, sizeof(struct hlist_head) << cfi_bits,
			PROT_READ|PROT_WRITE,
			MAP_PRIVATE|MAP_ANON, -1, 0);
	if (cfi_hash == (void *)-1L) {
		WARN("mmap fail cfi_hash");
		cfi_hash = NULL;
	}  else if (stats) {
		printf("cfi_bits: %d\n", cfi_bits);
	}

	return cfi_hash;
}

static unsigned long nr_insns;
static unsigned long nr_insns_visited;

/*
 * Call the arch-specific instruction decoder for all the instructions and add
 * them to the global instruction list.
 */
static int decode_instructions(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *func;
	unsigned long offset;
	struct instruction *insn;
	int ret;

	for_each_sec(file, sec) {

		if (!(sec->sh.sh_flags & SHF_EXECINSTR))
			continue;

		if (strcmp(sec->name, ".altinstr_replacement") &&
		    strcmp(sec->name, ".altinstr_aux") &&
		    strncmp(sec->name, ".discard.", 9))
			sec->text = true;

		if (!strcmp(sec->name, ".noinstr.text") ||
		    !strcmp(sec->name, ".entry.text") ||
		    !strncmp(sec->name, ".text..__x86.", 13))
			sec->noinstr = true;

		for (offset = 0; offset < sec->len; offset += insn->len) {
			insn = malloc(sizeof(*insn));
			if (!insn) {
				WARN("malloc failed");
				return -1;
			}
			memset(insn, 0, sizeof(*insn));
			INIT_LIST_HEAD(&insn->alts);
			INIT_LIST_HEAD(&insn->stack_ops);

			insn->sec = sec;
			insn->offset = offset;

			ret = arch_decode_instruction(file->elf, sec, offset,
						      sec->len - offset,
						      &insn->len, &insn->type,
						      &insn->immediate,
						      &insn->stack_ops);
			if (ret)
				goto err;

			hash_add(file->insn_hash, &insn->hash, sec_offset_hash(sec, insn->offset));
			list_add_tail(&insn->list, &file->insn_list);
			nr_insns++;
		}

		list_for_each_entry(func, &sec->symbol_list, list) {
			if (func->type != STT_FUNC || func->alias != func)
				continue;

			if (!find_insn(file, sec, func->offset)) {
				WARN("%s(): can't find starting instruction",
				     func->name);
				return -1;
			}

			sym_for_each_insn(file, func, insn)
				insn->func = func;
		}
	}

	if (stats)
		printf("nr_insns: %lu\n", nr_insns);

	return 0;

err:
	free(insn);
	return ret;
}

static struct instruction *find_last_insn(struct objtool_file *file,
					  struct section *sec)
{
	struct instruction *insn = NULL;
	unsigned int offset;
	unsigned int end = (sec->len > 10) ? sec->len - 10 : 0;

	for (offset = sec->len - 1; offset >= end && !insn; offset--)
		insn = find_insn(file, sec, offset);

	return insn;
}

/*
 * Mark "ud2" instructions and manually annotated dead ends.
 */
static int add_dead_ends(struct objtool_file *file)
{
	struct section *sec;
	struct reloc *reloc;
	struct instruction *insn;

	/*
	 * By default, "ud2" is a dead end unless otherwise annotated, because
	 * GCC 7 inserts it for certain divide-by-zero cases.
	 */
	for_each_insn(file, insn)
		if (insn->type == INSN_BUG)
			insn->dead_end = true;

	/*
	 * Check for manually annotated dead ends.
	 */
	sec = find_section_by_name(file->elf, ".rela.discard.unreachable");
	if (!sec)
		goto reachable;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		if (reloc->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}
		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (insn)
			insn = list_prev_entry(insn, list);
		else if (reloc->addend == reloc->sym->sec->len) {
			insn = find_last_insn(file, reloc->sym->sec);
			if (!insn) {
				WARN("can't find unreachable insn at %s+0x%" PRIx64,
				     reloc->sym->sec->name, reloc->addend);
				return -1;
			}
		} else {
			WARN("can't find unreachable insn at %s+0x%" PRIx64,
			     reloc->sym->sec->name, reloc->addend);
			return -1;
		}

		insn->dead_end = true;
	}

reachable:
	/*
	 * These manually annotated reachable checks are needed for GCC 4.4,
	 * where the Linux unreachable() macro isn't supported.  In that case
	 * GCC doesn't know the "ud2" is fatal, so it generates code as if it's
	 * not a dead end.
	 */
	sec = find_section_by_name(file->elf, ".rela.discard.reachable");
	if (!sec)
		return 0;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		if (reloc->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}
		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (insn)
			insn = list_prev_entry(insn, list);
		else if (reloc->addend == reloc->sym->sec->len) {
			insn = find_last_insn(file, reloc->sym->sec);
			if (!insn) {
				WARN("can't find reachable insn at %s+0x%" PRIx64,
				     reloc->sym->sec->name, reloc->addend);
				return -1;
			}
		} else {
			WARN("can't find reachable insn at %s+0x%" PRIx64,
			     reloc->sym->sec->name, reloc->addend);
			return -1;
		}

		insn->dead_end = false;
	}

	return 0;
}

static int create_static_call_sections(struct objtool_file *file)
{
	struct section *sec;
	struct static_call_site *site;
	struct instruction *insn;
	struct symbol *key_sym;
	char *key_name, *tmp;
	int idx;

	sec = find_section_by_name(file->elf, ".static_call_sites");
	if (sec) {
		INIT_LIST_HEAD(&file->static_call_list);
		WARN("file already has .static_call_sites section, skipping");
		return 0;
	}

	if (list_empty(&file->static_call_list))
		return 0;

	idx = 0;
	list_for_each_entry(insn, &file->static_call_list, call_node)
		idx++;

	sec = elf_create_section(file->elf, ".static_call_sites", SHF_WRITE,
				 sizeof(struct static_call_site), idx);
	if (!sec)
		return -1;

	idx = 0;
	list_for_each_entry(insn, &file->static_call_list, call_node) {

		site = (struct static_call_site *)sec->data->d_buf + idx;
		memset(site, 0, sizeof(struct static_call_site));

		/* populate reloc for 'addr' */
		if (elf_add_reloc_to_insn(file->elf, sec,
					  idx * sizeof(struct static_call_site),
					  R_X86_64_PC32,
					  insn->sec, insn->offset))
			return -1;

		/* find key symbol */
		key_name = strdup(insn->call_dest->name);
		if (!key_name) {
			perror("strdup");
			return -1;
		}
		if (strncmp(key_name, STATIC_CALL_TRAMP_PREFIX_STR,
			    STATIC_CALL_TRAMP_PREFIX_LEN)) {
			WARN("static_call: trampoline name malformed: %s", key_name);
			free(key_name);
			return -1;
		}
		tmp = key_name + STATIC_CALL_TRAMP_PREFIX_LEN - STATIC_CALL_KEY_PREFIX_LEN;
		memcpy(tmp, STATIC_CALL_KEY_PREFIX_STR, STATIC_CALL_KEY_PREFIX_LEN);

		key_sym = find_symbol_by_name(file->elf, tmp);
		if (!key_sym) {
			if (!module) {
				WARN("static_call: can't find static_call_key symbol: %s", tmp);
				free(key_name);
				return -1;
			}

			/*
			 * For modules(), the key might not be exported, which
			 * means the module can make static calls but isn't
			 * allowed to change them.
			 *
			 * In that case we temporarily set the key to be the
			 * trampoline address.  This is fixed up in
			 * static_call_add_module().
			 */
			key_sym = insn->call_dest;
		}
		free(key_name);

		/* populate reloc for 'key' */
		if (elf_add_reloc(file->elf, sec,
				  idx * sizeof(struct static_call_site) + 4,
				  R_X86_64_PC32, key_sym,
				  is_sibling_call(insn) * STATIC_CALL_SITE_TAIL))
			return -1;

		idx++;
	}

	return 0;
}

static int create_retpoline_sites_sections(struct objtool_file *file)
{
	struct instruction *insn;
	struct section *sec;
	int idx;

	sec = find_section_by_name(file->elf, ".retpoline_sites");
	if (sec) {
		WARN("file already has .retpoline_sites, skipping");
		return 0;
	}

	idx = 0;
	list_for_each_entry(insn, &file->retpoline_call_list, call_node)
		idx++;

	if (!idx)
		return 0;

	sec = elf_create_section(file->elf, ".retpoline_sites", 0,
				 sizeof(int), idx);
	if (!sec) {
		WARN("elf_create_section: .retpoline_sites");
		return -1;
	}

	idx = 0;
	list_for_each_entry(insn, &file->retpoline_call_list, call_node) {

		int *site = (int *)sec->data->d_buf + idx;
		*site = 0;

		if (elf_add_reloc_to_insn(file->elf, sec,
					  idx * sizeof(int),
					  R_X86_64_PC32,
					  insn->sec, insn->offset)) {
			WARN("elf_add_reloc_to_insn: .retpoline_sites");
			return -1;
		}

		idx++;
	}

	return 0;
}

static int create_return_sites_sections(struct objtool_file *file)
{
	struct instruction *insn;
	struct section *sec;
	int idx;

	sec = find_section_by_name(file->elf, ".return_sites");
	if (sec) {
		WARN("file already has .return_sites, skipping");
		return 0;
	}

	idx = 0;
	list_for_each_entry(insn, &file->return_thunk_list, call_node)
		idx++;

	if (!idx)
		return 0;

	sec = elf_create_section(file->elf, ".return_sites", 0,
				 sizeof(int), idx);
	if (!sec) {
		WARN("elf_create_section: .return_sites");
		return -1;
	}

	idx = 0;
	list_for_each_entry(insn, &file->return_thunk_list, call_node) {

		int *site = (int *)sec->data->d_buf + idx;
		*site = 0;

		if (elf_add_reloc_to_insn(file->elf, sec,
					  idx * sizeof(int),
					  R_X86_64_PC32,
					  insn->sec, insn->offset)) {
			WARN("elf_add_reloc_to_insn: .return_sites");
			return -1;
		}

		idx++;
	}

	return 0;
}

static int create_mcount_loc_sections(struct objtool_file *file)
{
	struct section *sec;
	unsigned long *loc;
	struct instruction *insn;
	int idx;

	sec = find_section_by_name(file->elf, "__mcount_loc");
	if (sec) {
		INIT_LIST_HEAD(&file->mcount_loc_list);
		WARN("file already has __mcount_loc section, skipping");
		return 0;
	}

	if (list_empty(&file->mcount_loc_list))
		return 0;

	idx = 0;
	list_for_each_entry(insn, &file->mcount_loc_list, mcount_loc_node)
		idx++;

	sec = elf_create_section(file->elf, "__mcount_loc", 0, sizeof(unsigned long), idx);
	if (!sec)
		return -1;

	idx = 0;
	list_for_each_entry(insn, &file->mcount_loc_list, mcount_loc_node) {

		loc = (unsigned long *)sec->data->d_buf + idx;
		memset(loc, 0, sizeof(unsigned long));

		if (elf_add_reloc_to_insn(file->elf, sec,
					  idx * sizeof(unsigned long),
					  R_X86_64_64,
					  insn->sec, insn->offset))
			return -1;

		idx++;
	}

	return 0;
}

/*
 * Warnings shouldn't be reported for ignored functions.
 */
static void add_ignores(struct objtool_file *file)
{
	struct instruction *insn;
	struct section *sec;
	struct symbol *func;
	struct reloc *reloc;

	sec = find_section_by_name(file->elf, ".rela.discard.func_stack_frame_non_standard");
	if (!sec)
		return;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		switch (reloc->sym->type) {
		case STT_FUNC:
			func = reloc->sym;
			break;

		case STT_SECTION:
			func = find_func_by_offset(reloc->sym->sec, reloc->addend);
			if (!func)
				continue;
			break;

		default:
			WARN("unexpected relocation symbol type in %s: %d", sec->name, reloc->sym->type);
			continue;
		}

		func_for_each_insn(file, func, insn)
			insn->ignore = true;
	}
}

/*
 * This is a whitelist of functions that is allowed to be called with AC set.
 * The list is meant to be minimal and only contains compiler instrumentation
 * ABI and a few functions used to implement *_{to,from}_user() functions.
 *
 * These functions must not directly change AC, but may PUSHF/POPF.
 */
static const char *uaccess_safe_builtin[] = {
	/* KASAN */
	"kasan_report",
	"kasan_check_range",
	/* KASAN out-of-line */
	"__asan_loadN_noabort",
	"__asan_load1_noabort",
	"__asan_load2_noabort",
	"__asan_load4_noabort",
	"__asan_load8_noabort",
	"__asan_load16_noabort",
	"__asan_storeN_noabort",
	"__asan_store1_noabort",
	"__asan_store2_noabort",
	"__asan_store4_noabort",
	"__asan_store8_noabort",
	"__asan_store16_noabort",
	"__kasan_check_read",
	"__kasan_check_write",
	/* KASAN in-line */
	"__asan_report_load_n_noabort",
	"__asan_report_load1_noabort",
	"__asan_report_load2_noabort",
	"__asan_report_load4_noabort",
	"__asan_report_load8_noabort",
	"__asan_report_load16_noabort",
	"__asan_report_store_n_noabort",
	"__asan_report_store1_noabort",
	"__asan_report_store2_noabort",
	"__asan_report_store4_noabort",
	"__asan_report_store8_noabort",
	"__asan_report_store16_noabort",
	/* KCSAN */
	"__kcsan_check_access",
	"kcsan_found_watchpoint",
	"kcsan_setup_watchpoint",
	"kcsan_check_scoped_accesses",
	"kcsan_disable_current",
	"kcsan_enable_current_nowarn",
	/* KCSAN/TSAN */
	"__tsan_func_entry",
	"__tsan_func_exit",
	"__tsan_read_range",
	"__tsan_write_range",
	"__tsan_read1",
	"__tsan_read2",
	"__tsan_read4",
	"__tsan_read8",
	"__tsan_read16",
	"__tsan_write1",
	"__tsan_write2",
	"__tsan_write4",
	"__tsan_write8",
	"__tsan_write16",
	"__tsan_read_write1",
	"__tsan_read_write2",
	"__tsan_read_write4",
	"__tsan_read_write8",
	"__tsan_read_write16",
	"__tsan_volatile_read1",
	"__tsan_volatile_read2",
	"__tsan_volatile_read4",
	"__tsan_volatile_read8",
	"__tsan_volatile_read16",
	"__tsan_volatile_write1",
	"__tsan_volatile_write2",
	"__tsan_volatile_write4",
	"__tsan_volatile_write8",
	"__tsan_volatile_write16",
	"__tsan_atomic8_load",
	"__tsan_atomic16_load",
	"__tsan_atomic32_load",
	"__tsan_atomic64_load",
	"__tsan_atomic8_store",
	"__tsan_atomic16_store",
	"__tsan_atomic32_store",
	"__tsan_atomic64_store",
	"__tsan_atomic8_exchange",
	"__tsan_atomic16_exchange",
	"__tsan_atomic32_exchange",
	"__tsan_atomic64_exchange",
	"__tsan_atomic8_fetch_add",
	"__tsan_atomic16_fetch_add",
	"__tsan_atomic32_fetch_add",
	"__tsan_atomic64_fetch_add",
	"__tsan_atomic8_fetch_sub",
	"__tsan_atomic16_fetch_sub",
	"__tsan_atomic32_fetch_sub",
	"__tsan_atomic64_fetch_sub",
	"__tsan_atomic8_fetch_and",
	"__tsan_atomic16_fetch_and",
	"__tsan_atomic32_fetch_and",
	"__tsan_atomic64_fetch_and",
	"__tsan_atomic8_fetch_or",
	"__tsan_atomic16_fetch_or",
	"__tsan_atomic32_fetch_or",
	"__tsan_atomic64_fetch_or",
	"__tsan_atomic8_fetch_xor",
	"__tsan_atomic16_fetch_xor",
	"__tsan_atomic32_fetch_xor",
	"__tsan_atomic64_fetch_xor",
	"__tsan_atomic8_fetch_nand",
	"__tsan_atomic16_fetch_nand",
	"__tsan_atomic32_fetch_nand",
	"__tsan_atomic64_fetch_nand",
	"__tsan_atomic8_compare_exchange_strong",
	"__tsan_atomic16_compare_exchange_strong",
	"__tsan_atomic32_compare_exchange_strong",
	"__tsan_atomic64_compare_exchange_strong",
	"__tsan_atomic8_compare_exchange_weak",
	"__tsan_atomic16_compare_exchange_weak",
	"__tsan_atomic32_compare_exchange_weak",
	"__tsan_atomic64_compare_exchange_weak",
	"__tsan_atomic8_compare_exchange_val",
	"__tsan_atomic16_compare_exchange_val",
	"__tsan_atomic32_compare_exchange_val",
	"__tsan_atomic64_compare_exchange_val",
	"__tsan_atomic_thread_fence",
	"__tsan_atomic_signal_fence",
	"__tsan_unaligned_read16",
	"__tsan_unaligned_write16",
	/* KCOV */
	"write_comp_data",
	"check_kcov_mode",
	"__sanitizer_cov_trace_pc",
	"__sanitizer_cov_trace_const_cmp1",
	"__sanitizer_cov_trace_const_cmp2",
	"__sanitizer_cov_trace_const_cmp4",
	"__sanitizer_cov_trace_const_cmp8",
	"__sanitizer_cov_trace_cmp1",
	"__sanitizer_cov_trace_cmp2",
	"__sanitizer_cov_trace_cmp4",
	"__sanitizer_cov_trace_cmp8",
	"__sanitizer_cov_trace_switch",
	/* UBSAN */
	"ubsan_type_mismatch_common",
	"__ubsan_handle_type_mismatch",
	"__ubsan_handle_type_mismatch_v1",
	"__ubsan_handle_shift_out_of_bounds",
	/* misc */
	"csum_partial_copy_generic",
	"copy_mc_fragile",
	"copy_mc_fragile_handle_tail",
	"copy_mc_enhanced_fast_string",
	"ftrace_likely_update", /* CONFIG_TRACE_BRANCH_PROFILING */
	NULL
};

static void add_uaccess_safe(struct objtool_file *file)
{
	struct symbol *func;
	const char **name;

	if (!uaccess)
		return;

	for (name = uaccess_safe_builtin; *name; name++) {
		func = find_symbol_by_name(file->elf, *name);
		if (!func)
			continue;

		func->uaccess_safe = true;
	}
}

/*
 * FIXME: For now, just ignore any alternatives which add retpolines.  This is
 * a temporary hack, as it doesn't allow ORC to unwind from inside a retpoline.
 * But it at least allows objtool to understand the control flow *around* the
 * retpoline.
 */
static int add_ignore_alternatives(struct objtool_file *file)
{
	struct section *sec;
	struct reloc *reloc;
	struct instruction *insn;

	sec = find_section_by_name(file->elf, ".rela.discard.ignore_alts");
	if (!sec)
		return 0;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		if (reloc->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}

		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (!insn) {
			WARN("bad .discard.ignore_alts entry");
			return -1;
		}

		insn->ignore_alts = true;
	}

	return 0;
}

/*
 * Symbols that replace INSN_CALL_DYNAMIC, every (tail) call to such a symbol
 * will be added to the .retpoline_sites section.
 */
__weak bool arch_is_retpoline(struct symbol *sym)
{
	return false;
}

/*
 * Symbols that replace INSN_RETURN, every (tail) call to such a symbol
 * will be added to the .return_sites section.
 */
__weak bool arch_is_rethunk(struct symbol *sym)
{
	return false;
}

/*
 * Symbols that are embedded inside other instructions, because sometimes crazy
 * code exists. These are mostly ignored for validation purposes.
 */
__weak bool arch_is_embedded_insn(struct symbol *sym)
{
	return false;
}

#define NEGATIVE_RELOC	((void *)-1L)

static struct reloc *insn_reloc(struct objtool_file *file, struct instruction *insn)
{
	if (insn->reloc == NEGATIVE_RELOC)
		return NULL;

	if (!insn->reloc) {
		insn->reloc = find_reloc_by_dest_range(file->elf, insn->sec,
						       insn->offset, insn->len);
		if (!insn->reloc) {
			insn->reloc = NEGATIVE_RELOC;
			return NULL;
		}
	}

	return insn->reloc;
}

static void remove_insn_ops(struct instruction *insn)
{
	struct stack_op *op, *tmp;

	list_for_each_entry_safe(op, tmp, &insn->stack_ops, list) {
		list_del(&op->list);
		free(op);
	}
}

static void annotate_call_site(struct objtool_file *file,
			       struct instruction *insn, bool sibling)
{
	struct reloc *reloc = insn_reloc(file, insn);
	struct symbol *sym = insn->call_dest;

	if (!sym)
		sym = reloc->sym;

	/*
	 * Alternative replacement code is just template code which is
	 * sometimes copied to the original instruction. For now, don't
	 * annotate it. (In the future we might consider annotating the
	 * original instruction if/when it ever makes sense to do so.)
	 */
	if (!strcmp(insn->sec->name, ".altinstr_replacement"))
		return;

	if (sym->static_call_tramp) {
		list_add_tail(&insn->call_node, &file->static_call_list);
		return;
	}

	if (sym->retpoline_thunk) {
		list_add_tail(&insn->call_node, &file->retpoline_call_list);
		return;
	}

	/*
	 * Many compilers cannot disable KCOV with a function attribute
	 * so they need a little help, NOP out any KCOV calls from noinstr
	 * text.
	 */
	if (insn->sec->noinstr && sym->kcov) {
		if (reloc) {
			reloc->type = R_NONE;
			elf_write_reloc(file->elf, reloc);
		}

		elf_write_insn(file->elf, insn->sec,
			       insn->offset, insn->len,
			       sibling ? arch_ret_insn(insn->len)
			               : arch_nop_insn(insn->len));

		insn->type = sibling ? INSN_RETURN : INSN_NOP;

		if (sibling) {
			/*
			 * We've replaced the tail-call JMP insn by two new
			 * insn: RET; INT3, except we only have a single struct
			 * insn here. Mark it retpoline_safe to avoid the SLS
			 * warning, instead of adding another insn.
			 */
			insn->retpoline_safe = true;
		}

		return;
	}
}

static void add_call_dest(struct objtool_file *file, struct instruction *insn,
			  struct symbol *dest, bool sibling)
{
	insn->call_dest = dest;
	if (!dest)
		return;

	/*
	 * Whatever stack impact regular CALLs have, should be undone
	 * by the RETURN of the called function.
	 *
	 * Annotated intra-function calls retain the stack_ops but
	 * are converted to JUMP, see read_intra_function_calls().
	 */
	remove_insn_ops(insn);

	annotate_call_site(file, insn, sibling);
}

static void add_retpoline_call(struct objtool_file *file, struct instruction *insn)
{
	/*
	 * Retpoline calls/jumps are really dynamic calls/jumps in disguise,
	 * so convert them accordingly.
	 */
	switch (insn->type) {
	case INSN_CALL:
		insn->type = INSN_CALL_DYNAMIC;
		break;
	case INSN_JUMP_UNCONDITIONAL:
		insn->type = INSN_JUMP_DYNAMIC;
		break;
	case INSN_JUMP_CONDITIONAL:
		insn->type = INSN_JUMP_DYNAMIC_CONDITIONAL;
		break;
	default:
		return;
	}

	insn->retpoline_safe = true;

	/*
	 * Whatever stack impact regular CALLs have, should be undone
	 * by the RETURN of the called function.
	 *
	 * Annotated intra-function calls retain the stack_ops but
	 * are converted to JUMP, see read_intra_function_calls().
	 */
	remove_insn_ops(insn);

	annotate_call_site(file, insn, false);
}

static void add_return_call(struct objtool_file *file, struct instruction *insn, bool add)
{
	/*
	 * Return thunk tail calls are really just returns in disguise,
	 * so convert them accordingly.
	 */
	insn->type = INSN_RETURN;
	insn->retpoline_safe = true;

	/* Skip the non-text sections, specially .discard ones */
	if (add && insn->sec->text)
		list_add_tail(&insn->call_node, &file->return_thunk_list);
}

/*
 * CONFIG_CFI_CLANG: Check if the section is a CFI jump table or a
 * compiler-generated CFI handler.
 */
static bool is_cfi_section(struct section *sec)
{
	return (sec->name &&
		(!strncmp(sec->name, ".text..L.cfi.jumptable", 22) ||
		 !strcmp(sec->name, ".text.__cfi_check")));
}

/*
 * CONFIG_CFI_CLANG: Ignore CFI jump tables.
 */
static void add_cfi_jumptables(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *func;
	struct instruction *insn;

	for_each_sec(file, sec) {
		if (!is_cfi_section(sec))
			continue;

		list_for_each_entry(func, &sec->symbol_list, list) {
			sym_for_each_insn(file, func, insn)
				insn->ignore = true;
		}
	}
}

/*
 * Find the destination instructions for all jumps.
 */
static int add_jump_destinations(struct objtool_file *file)
{
	struct instruction *insn;
	struct reloc *reloc;
	struct section *dest_sec;
	unsigned long dest_off;

	for_each_insn(file, insn) {
		if (!is_static_jump(insn))
			continue;

		reloc = insn_reloc(file, insn);
		if (!reloc) {
			dest_sec = insn->sec;
			dest_off = arch_jump_destination(insn);
		} else if (reloc->sym->type == STT_SECTION) {
			dest_sec = reloc->sym->sec;
			dest_off = arch_dest_reloc_offset(reloc->addend);
		} else if (reloc->sym->retpoline_thunk) {
			add_retpoline_call(file, insn);
			continue;
		} else if (reloc->sym->return_thunk) {
			add_return_call(file, insn, true);
			continue;
		} else if (insn->func) {
			/* internal or external sibling call (with reloc) */
			add_call_dest(file, insn, reloc->sym, true);
			continue;
		} else if (reloc->sym->sec->idx) {
			dest_sec = reloc->sym->sec;
			dest_off = reloc->sym->sym.st_value +
				   arch_dest_reloc_offset(reloc->addend);
		} else {
			/* non-func asm code jumping to another file */
			continue;
		}

		insn->jump_dest = find_insn(file, dest_sec, dest_off);

		if (!insn->jump_dest && dest_sec->len == dest_off)
			insn->jump_dest = find_last_insn(file, dest_sec);

		if (!insn->jump_dest) {
			struct symbol *sym = find_symbol_by_offset(dest_sec, dest_off);

			/*
			 * This is a special case where an alt instruction
			 * jumps past the end of the section.  These are
			 * handled later in handle_group_alt().
			 */
			if (!strcmp(insn->sec->name, ".altinstr_replacement"))
				continue;

			if (is_cfi_section(insn->sec))
				continue;

			/*
			 * This is a special case for retbleed_untrain_ret().
			 * It jumps to __x86_return_thunk(), but objtool
			 * can't find the thunk's starting RET
			 * instruction, because the RET is also in the
			 * middle of another instruction.  Objtool only
			 * knows about the outer instruction.
			 */
			if (sym && sym->embedded_insn) {
				add_return_call(file, insn, false);
				continue;
			}

			WARN_FUNC("can't find jump dest instruction at %s+0x%lx",
				  insn->sec, insn->offset, dest_sec->name,
				  dest_off);
			return -1;
		}

		/*
		 * Cross-function jump.
		 */
		if (insn->func && insn->jump_dest->func &&
		    insn->func != insn->jump_dest->func) {

			/*
			 * For GCC 8+, create parent/child links for any cold
			 * subfunctions.  This is _mostly_ redundant with a
			 * similar initialization in read_symbols().
			 *
			 * If a function has aliases, we want the *first* such
			 * function in the symbol table to be the subfunction's
			 * parent.  In that case we overwrite the
			 * initialization done in read_symbols().
			 *
			 * However this code can't completely replace the
			 * read_symbols() code because this doesn't detect the
			 * case where the parent function's only reference to a
			 * subfunction is through a jump table.
			 */
			if (!strstr(insn->func->name, ".cold") &&
			    strstr(insn->jump_dest->func->name, ".cold")) {
				insn->func->cfunc = insn->jump_dest->func;
				insn->jump_dest->func->pfunc = insn->func;

			} else if (insn->jump_dest->func->pfunc != insn->func->pfunc &&
				   insn->jump_dest->offset == insn->jump_dest->func->offset) {
				/* internal sibling call (without reloc) */
				add_call_dest(file, insn, insn->jump_dest->func, true);
			}
		}
	}

	return 0;
}

static struct symbol *find_call_destination(struct section *sec, unsigned long offset)
{
	struct symbol *call_dest;

	call_dest = find_func_by_offset(sec, offset);
	if (!call_dest)
		call_dest = find_symbol_by_offset(sec, offset);

	return call_dest;
}

/*
 * Find the destination instructions for all calls.
 */
static int add_call_destinations(struct objtool_file *file)
{
	struct instruction *insn;
	unsigned long dest_off;
	struct symbol *dest;
	struct reloc *reloc;

	for_each_insn(file, insn) {
		if (insn->type != INSN_CALL)
			continue;

		reloc = insn_reloc(file, insn);
		if (!reloc) {
			dest_off = arch_jump_destination(insn);
			dest = find_call_destination(insn->sec, dest_off);

			add_call_dest(file, insn, dest, false);

			if (insn->ignore)
				continue;

			if (!insn->call_dest) {
				WARN_FUNC("unannotated intra-function call", insn->sec, insn->offset);
				return -1;
			}

			if (insn->func && insn->call_dest->type != STT_FUNC) {
				WARN_FUNC("unsupported call to non-function",
					  insn->sec, insn->offset);
				return -1;
			}

		} else if (reloc->sym->type == STT_SECTION) {
			dest_off = arch_dest_reloc_offset(reloc->addend);
			dest = find_call_destination(reloc->sym->sec, dest_off);
			if (!dest) {
				if (is_cfi_section(reloc->sym->sec))
					continue;

				WARN_FUNC("can't find call dest symbol at %s+0x%lx",
					  insn->sec, insn->offset,
					  reloc->sym->sec->name,
					  dest_off);
				return -1;
			}

			add_call_dest(file, insn, dest, false);

		} else if (reloc->sym->retpoline_thunk) {
			add_retpoline_call(file, insn);

		} else
			add_call_dest(file, insn, reloc->sym, false);
	}

	return 0;
}

/*
 * The .alternatives section requires some extra special care over and above
 * other special sections because alternatives are patched in place.
 */
static int handle_group_alt(struct objtool_file *file,
			    struct special_alt *special_alt,
			    struct instruction *orig_insn,
			    struct instruction **new_insn)
{
	struct instruction *last_orig_insn, *last_new_insn = NULL, *insn, *nop = NULL;
	struct alt_group *orig_alt_group, *new_alt_group;
	unsigned long dest_off;


	orig_alt_group = malloc(sizeof(*orig_alt_group));
	if (!orig_alt_group) {
		WARN("malloc failed");
		return -1;
	}
	orig_alt_group->cfi = calloc(special_alt->orig_len,
				     sizeof(struct cfi_state *));
	if (!orig_alt_group->cfi) {
		WARN("calloc failed");
		return -1;
	}

	last_orig_insn = NULL;
	insn = orig_insn;
	sec_for_each_insn_from(file, insn) {
		if (insn->offset >= special_alt->orig_off + special_alt->orig_len)
			break;

		insn->alt_group = orig_alt_group;
		last_orig_insn = insn;
	}
	orig_alt_group->orig_group = NULL;
	orig_alt_group->first_insn = orig_insn;
	orig_alt_group->last_insn = last_orig_insn;


	new_alt_group = malloc(sizeof(*new_alt_group));
	if (!new_alt_group) {
		WARN("malloc failed");
		return -1;
	}

	if (special_alt->new_len < special_alt->orig_len) {
		/*
		 * Insert a fake nop at the end to make the replacement
		 * alt_group the same size as the original.  This is needed to
		 * allow propagate_alt_cfi() to do its magic.  When the last
		 * instruction affects the stack, the instruction after it (the
		 * nop) will propagate the new state to the shared CFI array.
		 */
		nop = malloc(sizeof(*nop));
		if (!nop) {
			WARN("malloc failed");
			return -1;
		}
		memset(nop, 0, sizeof(*nop));
		INIT_LIST_HEAD(&nop->alts);
		INIT_LIST_HEAD(&nop->stack_ops);

		nop->sec = special_alt->new_sec;
		nop->offset = special_alt->new_off + special_alt->new_len;
		nop->len = special_alt->orig_len - special_alt->new_len;
		nop->type = INSN_NOP;
		nop->func = orig_insn->func;
		nop->alt_group = new_alt_group;
		nop->ignore = orig_insn->ignore_alts;
	}

	if (!special_alt->new_len) {
		*new_insn = nop;
		goto end;
	}

	insn = *new_insn;
	sec_for_each_insn_from(file, insn) {
		struct reloc *alt_reloc;

		if (insn->offset >= special_alt->new_off + special_alt->new_len)
			break;

		last_new_insn = insn;

		insn->ignore = orig_insn->ignore_alts;
		insn->func = orig_insn->func;
		insn->alt_group = new_alt_group;

		/*
		 * Since alternative replacement code is copy/pasted by the
		 * kernel after applying relocations, generally such code can't
		 * have relative-address relocation references to outside the
		 * .altinstr_replacement section, unless the arch's
		 * alternatives code can adjust the relative offsets
		 * accordingly.
		 */
		alt_reloc = insn_reloc(file, insn);
		if (alt_reloc &&
		    !arch_support_alt_relocation(special_alt, insn, alt_reloc)) {

			WARN_FUNC("unsupported relocation in alternatives section",
				  insn->sec, insn->offset);
			return -1;
		}

		if (!is_static_jump(insn))
			continue;

		if (!insn->immediate)
			continue;

		dest_off = arch_jump_destination(insn);
		if (dest_off == special_alt->new_off + special_alt->new_len)
			insn->jump_dest = next_insn_same_sec(file, last_orig_insn);

		if (!insn->jump_dest) {
			WARN_FUNC("can't find alternative jump destination",
				  insn->sec, insn->offset);
			return -1;
		}
	}

	if (!last_new_insn) {
		WARN_FUNC("can't find last new alternative instruction",
			  special_alt->new_sec, special_alt->new_off);
		return -1;
	}

	if (nop)
		list_add(&nop->list, &last_new_insn->list);
end:
	new_alt_group->orig_group = orig_alt_group;
	new_alt_group->first_insn = *new_insn;
	new_alt_group->last_insn = nop ? : last_new_insn;
	new_alt_group->cfi = orig_alt_group->cfi;
	return 0;
}

/*
 * A jump table entry can either convert a nop to a jump or a jump to a nop.
 * If the original instruction is a jump, make the alt entry an effective nop
 * by just skipping the original instruction.
 */
static int handle_jump_alt(struct objtool_file *file,
			   struct special_alt *special_alt,
			   struct instruction *orig_insn,
			   struct instruction **new_insn)
{
	if (orig_insn->type == INSN_NOP)
		return 0;

	if (orig_insn->type != INSN_JUMP_UNCONDITIONAL) {
		WARN_FUNC("unsupported instruction at jump label",
			  orig_insn->sec, orig_insn->offset);
		return -1;
	}

	*new_insn = list_next_entry(orig_insn, list);
	return 0;
}

/*
 * Read all the special sections which have alternate instructions which can be
 * patched in or redirected to at runtime.  Each instruction having alternate
 * instruction(s) has them added to its insn->alts list, which will be
 * traversed in validate_branch().
 */
static int add_special_section_alts(struct objtool_file *file)
{
	struct list_head special_alts;
	struct instruction *orig_insn, *new_insn;
	struct special_alt *special_alt, *tmp;
	struct alternative *alt;
	int ret;

	ret = special_get_alts(file->elf, &special_alts);
	if (ret)
		return ret;

	list_for_each_entry_safe(special_alt, tmp, &special_alts, list) {

		orig_insn = find_insn(file, special_alt->orig_sec,
				      special_alt->orig_off);
		if (!orig_insn) {
			WARN_FUNC("special: can't find orig instruction",
				  special_alt->orig_sec, special_alt->orig_off);
			ret = -1;
			goto out;
		}

		new_insn = NULL;
		if (!special_alt->group || special_alt->new_len) {
			new_insn = find_insn(file, special_alt->new_sec,
					     special_alt->new_off);
			if (!new_insn) {
				WARN_FUNC("special: can't find new instruction",
					  special_alt->new_sec,
					  special_alt->new_off);
				ret = -1;
				goto out;
			}
		}

		if (special_alt->group) {
			if (!special_alt->orig_len) {
				WARN_FUNC("empty alternative entry",
					  orig_insn->sec, orig_insn->offset);
				continue;
			}

			ret = handle_group_alt(file, special_alt, orig_insn,
					       &new_insn);
			if (ret)
				goto out;
		} else if (special_alt->jump_or_nop) {
			ret = handle_jump_alt(file, special_alt, orig_insn,
					      &new_insn);
			if (ret)
				goto out;
		}

		alt = malloc(sizeof(*alt));
		if (!alt) {
			WARN("malloc failed");
			ret = -1;
			goto out;
		}

		alt->insn = new_insn;
		alt->skip_orig = special_alt->skip_orig;
		orig_insn->ignore_alts |= special_alt->skip_alt;
		list_add_tail(&alt->list, &orig_insn->alts);

		list_del(&special_alt->list);
		free(special_alt);
	}

out:
	return ret;
}

static int add_jump_table(struct objtool_file *file, struct instruction *insn,
			    struct reloc *table)
{
	struct reloc *reloc = table;
	struct instruction *dest_insn;
	struct alternative *alt;
	struct symbol *pfunc = insn->func->pfunc;
	unsigned int prev_offset = 0;

	/*
	 * Each @reloc is a switch table relocation which points to the target
	 * instruction.
	 */
	list_for_each_entry_from(reloc, &table->sec->reloc_list, list) {

		/* Check for the end of the table: */
		if (reloc != table && reloc->jump_table_start)
			break;

		/* Make sure the table entries are consecutive: */
		if (prev_offset && reloc->offset != prev_offset + 8)
			break;

		/* Detect function pointers from contiguous objects: */
		if (reloc->sym->sec == pfunc->sec &&
		    reloc->addend == pfunc->offset)
			break;

		dest_insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (!dest_insn)
			break;

		/* Make sure the destination is in the same function: */
		if (!dest_insn->func || dest_insn->func->pfunc != pfunc)
			break;

		alt = malloc(sizeof(*alt));
		if (!alt) {
			WARN("malloc failed");
			return -1;
		}

		alt->insn = dest_insn;
		list_add_tail(&alt->list, &insn->alts);
		prev_offset = reloc->offset;
	}

	if (!prev_offset) {
		WARN_FUNC("can't find switch jump table",
			  insn->sec, insn->offset);
		return -1;
	}

	return 0;
}

/*
 * find_jump_table() - Given a dynamic jump, find the switch jump table
 * associated with it.
 */
static struct reloc *find_jump_table(struct objtool_file *file,
				      struct symbol *func,
				      struct instruction *insn)
{
	struct reloc *table_reloc;
	struct instruction *dest_insn, *orig_insn = insn;

	/*
	 * Backward search using the @first_jump_src links, these help avoid
	 * much of the 'in between' code. Which avoids us getting confused by
	 * it.
	 */
	for (;
	     insn && insn->func && insn->func->pfunc == func;
	     insn = insn->first_jump_src ?: prev_insn_same_sym(file, insn)) {

		if (insn != orig_insn && insn->type == INSN_JUMP_DYNAMIC)
			break;

		/* allow small jumps within the range */
		if (insn->type == INSN_JUMP_UNCONDITIONAL &&
		    insn->jump_dest &&
		    (insn->jump_dest->offset <= insn->offset ||
		     insn->jump_dest->offset > orig_insn->offset))
		    break;

		table_reloc = arch_find_switch_table(file, insn);
		if (!table_reloc)
			continue;
		dest_insn = find_insn(file, table_reloc->sym->sec, table_reloc->addend);
		if (!dest_insn || !dest_insn->func || dest_insn->func->pfunc != func)
			continue;

		return table_reloc;
	}

	return NULL;
}

/*
 * First pass: Mark the head of each jump table so that in the next pass,
 * we know when a given jump table ends and the next one starts.
 */
static void mark_func_jump_tables(struct objtool_file *file,
				    struct symbol *func)
{
	struct instruction *insn, *last = NULL;
	struct reloc *reloc;

	func_for_each_insn(file, func, insn) {
		if (!last)
			last = insn;

		/*
		 * Store back-pointers for unconditional forward jumps such
		 * that find_jump_table() can back-track using those and
		 * avoid some potentially confusing code.
		 */
		if (insn->type == INSN_JUMP_UNCONDITIONAL && insn->jump_dest &&
		    insn->offset > last->offset &&
		    insn->jump_dest->offset > insn->offset &&
		    !insn->jump_dest->first_jump_src) {

			insn->jump_dest->first_jump_src = insn;
			last = insn->jump_dest;
		}

		if (insn->type != INSN_JUMP_DYNAMIC)
			continue;

		reloc = find_jump_table(file, func, insn);
		if (reloc) {
			reloc->jump_table_start = true;
			insn->jump_table = reloc;
		}
	}
}

static int add_func_jump_tables(struct objtool_file *file,
				  struct symbol *func)
{
	struct instruction *insn;
	int ret;

	func_for_each_insn(file, func, insn) {
		if (!insn->jump_table)
			continue;

		ret = add_jump_table(file, insn, insn->jump_table);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * For some switch statements, gcc generates a jump table in the .rodata
 * section which contains a list of addresses within the function to jump to.
 * This finds these jump tables and adds them to the insn->alts lists.
 */
static int add_jump_table_alts(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *func;
	int ret;

	if (!file->rodata)
		return 0;

	for_each_sec(file, sec) {
		list_for_each_entry(func, &sec->symbol_list, list) {
			if (func->type != STT_FUNC)
				continue;

			mark_func_jump_tables(file, func);
			ret = add_func_jump_tables(file, func);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static void set_func_state(struct cfi_state *state)
{
	state->cfa = initial_func_cfi.cfa;
	memcpy(&state->regs, &initial_func_cfi.regs,
	       CFI_NUM_REGS * sizeof(struct cfi_reg));
	state->stack_size = initial_func_cfi.cfa.offset;
}

static int read_unwind_hints(struct objtool_file *file)
{
	struct cfi_state cfi = init_cfi;
	struct section *sec, *relocsec;
	struct unwind_hint *hint;
	struct instruction *insn;
	struct reloc *reloc;
	int i;

	sec = find_section_by_name(file->elf, ".discard.unwind_hints");
	if (!sec)
		return 0;

	relocsec = sec->reloc;
	if (!relocsec) {
		WARN("missing .rela.discard.unwind_hints section");
		return -1;
	}

	if (sec->len % sizeof(struct unwind_hint)) {
		WARN("struct unwind_hint size mismatch");
		return -1;
	}

	file->hints = true;

	for (i = 0; i < sec->len / sizeof(struct unwind_hint); i++) {
		hint = (struct unwind_hint *)sec->data->d_buf + i;

		reloc = find_reloc_by_dest(file->elf, sec, i * sizeof(*hint));
		if (!reloc) {
			WARN("can't find reloc for unwind_hints[%d]", i);
			return -1;
		}

		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (!insn) {
			WARN("can't find insn for unwind_hints[%d]", i);
			return -1;
		}

		insn->hint = true;

		if (hint->type == UNWIND_HINT_TYPE_SAVE) {
			insn->hint = false;
			insn->save = true;
			continue;
		}

		if (hint->type == UNWIND_HINT_TYPE_RESTORE) {
			insn->restore = true;
			continue;
		}

		if (hint->type == UNWIND_HINT_TYPE_REGS_PARTIAL) {
			struct symbol *sym = find_symbol_by_offset(insn->sec, insn->offset);

			if (sym && sym->bind == STB_GLOBAL) {
				insn->entry = 1;
			}
		}

		if (hint->type == UNWIND_HINT_TYPE_ENTRY) {
			hint->type = UNWIND_HINT_TYPE_CALL;
			insn->entry = 1;
		}

		if (hint->type == UNWIND_HINT_TYPE_FUNC) {
			insn->cfi = &func_cfi;
			continue;
		}

		if (insn->cfi)
			cfi = *(insn->cfi);

		if (arch_decode_hint_reg(hint->sp_reg, &cfi.cfa.base)) {
			WARN_FUNC("unsupported unwind_hint sp base reg %d",
				  insn->sec, insn->offset, hint->sp_reg);
			return -1;
		}

		cfi.cfa.offset = hint->sp_offset;
		cfi.type = hint->type;
		cfi.end = hint->end;

		insn->cfi = cfi_hash_find_or_add(&cfi);
	}

	return 0;
}

static int read_retpoline_hints(struct objtool_file *file)
{
	struct section *sec;
	struct instruction *insn;
	struct reloc *reloc;

	sec = find_section_by_name(file->elf, ".rela.discard.retpoline_safe");
	if (!sec)
		return 0;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		if (reloc->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}

		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (!insn) {
			WARN("bad .discard.retpoline_safe entry");
			return -1;
		}

		if (insn->type != INSN_JUMP_DYNAMIC &&
		    insn->type != INSN_CALL_DYNAMIC &&
		    insn->type != INSN_RETURN &&
		    insn->type != INSN_NOP) {
			WARN_FUNC("retpoline_safe hint not an indirect jump/call/ret/nop",
				  insn->sec, insn->offset);
			return -1;
		}

		insn->retpoline_safe = true;
	}

	return 0;
}

static int read_instr_hints(struct objtool_file *file)
{
	struct section *sec;
	struct instruction *insn;
	struct reloc *reloc;

	sec = find_section_by_name(file->elf, ".rela.discard.instr_end");
	if (!sec)
		return 0;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		if (reloc->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}

		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (!insn) {
			WARN("bad .discard.instr_end entry");
			return -1;
		}

		insn->instr--;
	}

	sec = find_section_by_name(file->elf, ".rela.discard.instr_begin");
	if (!sec)
		return 0;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		if (reloc->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s", sec->name);
			return -1;
		}

		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (!insn) {
			WARN("bad .discard.instr_begin entry");
			return -1;
		}

		insn->instr++;
	}

	return 0;
}

static int read_intra_function_calls(struct objtool_file *file)
{
	struct instruction *insn;
	struct section *sec;
	struct reloc *reloc;

	sec = find_section_by_name(file->elf, ".rela.discard.intra_function_calls");
	if (!sec)
		return 0;

	list_for_each_entry(reloc, &sec->reloc_list, list) {
		unsigned long dest_off;

		if (reloc->sym->type != STT_SECTION) {
			WARN("unexpected relocation symbol type in %s",
			     sec->name);
			return -1;
		}

		insn = find_insn(file, reloc->sym->sec, reloc->addend);
		if (!insn) {
			WARN("bad .discard.intra_function_call entry");
			return -1;
		}

		if (insn->type != INSN_CALL) {
			WARN_FUNC("intra_function_call not a direct call",
				  insn->sec, insn->offset);
			return -1;
		}

		/*
		 * Treat intra-function CALLs as JMPs, but with a stack_op.
		 * See add_call_destinations(), which strips stack_ops from
		 * normal CALLs.
		 */
		insn->type = INSN_JUMP_UNCONDITIONAL;

		dest_off = insn->offset + insn->len + insn->immediate;
		insn->jump_dest = find_insn(file, insn->sec, dest_off);
		if (!insn->jump_dest) {
			WARN_FUNC("can't find call dest at %s+0x%lx",
				  insn->sec, insn->offset,
				  insn->sec->name, dest_off);
			return -1;
		}
	}

	return 0;
}

static int classify_symbols(struct objtool_file *file)
{
	struct section *sec;
	struct symbol *func;

	for_each_sec(file, sec) {
		list_for_each_entry(func, &sec->symbol_list, list) {
			if (func->bind != STB_GLOBAL)
				continue;

			if (!strncmp(func->name, STATIC_CALL_TRAMP_PREFIX_STR,
				     strlen(STATIC_CALL_TRAMP_PREFIX_STR)))
				func->static_call_tramp = true;

			if (arch_is_retpoline(func))
				func->retpoline_thunk = true;

			if (arch_is_rethunk(func))
				func->return_thunk = true;

			if (arch_is_embedded_insn(func))
				func->embedded_insn = true;

			if (!strcmp(func->name, "__fentry__"))
				func->fentry = true;

			if (!strncmp(func->name, "__sanitizer_cov_", 16))
				func->kcov = true;
		}
	}

	return 0;
}

static void mark_rodata(struct objtool_file *file)
{
	struct section *sec;
	bool found = false;

	/*
	 * Search for the following rodata sections, each of which can
	 * potentially contain jump tables:
	 *
	 * - .rodata: can contain GCC switch tables
	 * - .rodata.<func>: same, if -fdata-sections is being used
	 * - .rodata..c_jump_table: contains C annotated jump tables
	 *
	 * .rodata.str1.* sections are ignored; they don't contain jump tables.
	 */
	for_each_sec(file, sec) {
		if (!strncmp(sec->name, ".rodata", 7) &&
		    !strstr(sec->name, ".str1.")) {
			sec->rodata = true;
			found = true;
		}
	}

	file->rodata = found;
}

static int decode_sections(struct objtool_file *file)
{
	int ret;

	mark_rodata(file);

	ret = decode_instructions(file);
	if (ret)
		return ret;

	ret = add_dead_ends(file);
	if (ret)
		return ret;

	add_ignores(file);
	add_uaccess_safe(file);
	add_cfi_jumptables(file);

	ret = add_ignore_alternatives(file);
	if (ret)
		return ret;

	/*
	 * Must be before add_{jump_call}_destination.
	 */
	ret = classify_symbols(file);
	if (ret)
		return ret;

	/*
	 * Must be before add_special_section_alts() as that depends on
	 * jump_dest being set.
	 */
	ret = add_jump_destinations(file);
	if (ret)
		return ret;

	ret = add_special_section_alts(file);
	if (ret)
		return ret;

	/*
	 * Must be before add_call_destination(); it changes INSN_CALL to
	 * INSN_JUMP.
	 */
	ret = read_intra_function_calls(file);
	if (ret)
		return ret;

	ret = add_call_destinations(file);
	if (ret)
		return ret;

	ret = add_jump_table_alts(file);
	if (ret)
		return ret;

	ret = read_unwind_hints(file);
	if (ret)
		return ret;

	ret = read_retpoline_hints(file);
	if (ret)
		return ret;

	ret = read_instr_hints(file);
	if (ret)
		return ret;

	return 0;
}

static bool is_special_call(struct instruction *insn)
{
	if (insn->type == INSN_CALL) {
		struct symbol *dest = insn->call_dest;

		if (!dest)
			return false;

		if (dest->fentry)
			return true;
	}

	return false;
}

static bool has_modified_stack_frame(struct instruction *insn, struct insn_state *state)
{
	struct cfi_state *cfi = &state->cfi;
	int i;

	if (cfi->cfa.base != initial_func_cfi.cfa.base || cfi->drap)
		return true;

	if (cfi->cfa.offset != initial_func_cfi.cfa.offset)
		return true;

	if (cfi->stack_size != initial_func_cfi.cfa.offset)
		return true;

	for (i = 0; i < CFI_NUM_REGS; i++) {
		if (cfi->regs[i].base != initial_func_cfi.regs[i].base ||
		    cfi->regs[i].offset != initial_func_cfi.regs[i].offset)
			return true;
	}

	return false;
}

static bool has_valid_stack_frame(struct insn_state *state)
{
	struct cfi_state *cfi = &state->cfi;

	if (cfi->cfa.base == CFI_BP && cfi->regs[CFI_BP].base == CFI_CFA &&
	    cfi->regs[CFI_BP].offset == -16)
		return true;

	if (cfi->drap && cfi->regs[CFI_BP].base == CFI_BP)
		return true;

	return false;
}

static int update_cfi_state_regs(struct instruction *insn,
				  struct cfi_state *cfi,
				  struct stack_op *op)
{
	struct cfi_reg *cfa = &cfi->cfa;

	if (cfa->base != CFI_SP && cfa->base != CFI_SP_INDIRECT)
		return 0;

	/* push */
	if (op->dest.type == OP_DEST_PUSH || op->dest.type == OP_DEST_PUSHF)
		cfa->offset += 8;

	/* pop */
	if (op->src.type == OP_SRC_POP || op->src.type == OP_SRC_POPF)
		cfa->offset -= 8;

	/* add immediate to sp */
	if (op->dest.type == OP_DEST_REG && op->src.type == OP_SRC_ADD &&
	    op->dest.reg == CFI_SP && op->src.reg == CFI_SP)
		cfa->offset -= op->src.offset;

	return 0;
}

static void save_reg(struct cfi_state *cfi, unsigned char reg, int base, int offset)
{
	if (arch_callee_saved_reg(reg) &&
	    cfi->regs[reg].base == CFI_UNDEFINED) {
		cfi->regs[reg].base = base;
		cfi->regs[reg].offset = offset;
	}
}

static void restore_reg(struct cfi_state *cfi, unsigned char reg)
{
	cfi->regs[reg].base = initial_func_cfi.regs[reg].base;
	cfi->regs[reg].offset = initial_func_cfi.regs[reg].offset;
}

/*
 * A note about DRAP stack alignment:
 *
 * GCC has the concept of a DRAP register, which is used to help keep track of
 * the stack pointer when aligning the stack.  r10 or r13 is used as the DRAP
 * register.  The typical DRAP pattern is:
 *
 *   4c 8d 54 24 08		lea    0x8(%rsp),%r10
 *   48 83 e4 c0		and    $0xffffffffffffffc0,%rsp
 *   41 ff 72 f8		pushq  -0x8(%r10)
 *   55				push   %rbp
 *   48 89 e5			mov    %rsp,%rbp
 *				(more pushes)
 *   41 52			push   %r10
 *				...
 *   41 5a			pop    %r10
 *				(more pops)
 *   5d				pop    %rbp
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * There are some variations in the epilogues, like:
 *
 *   5b				pop    %rbx
 *   41 5a			pop    %r10
 *   41 5c			pop    %r12
 *   41 5d			pop    %r13
 *   41 5e			pop    %r14
 *   c9				leaveq
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * and:
 *
 *   4c 8b 55 e8		mov    -0x18(%rbp),%r10
 *   48 8b 5d e0		mov    -0x20(%rbp),%rbx
 *   4c 8b 65 f0		mov    -0x10(%rbp),%r12
 *   4c 8b 6d f8		mov    -0x8(%rbp),%r13
 *   c9				leaveq
 *   49 8d 62 f8		lea    -0x8(%r10),%rsp
 *   c3				retq
 *
 * Sometimes r13 is used as the DRAP register, in which case it's saved and
 * restored beforehand:
 *
 *   41 55			push   %r13
 *   4c 8d 6c 24 10		lea    0x10(%rsp),%r13
 *   48 83 e4 f0		and    $0xfffffffffffffff0,%rsp
 *				...
 *   49 8d 65 f0		lea    -0x10(%r13),%rsp
 *   41 5d			pop    %r13
 *   c3				retq
 */
static int update_cfi_state(struct instruction *insn, struct cfi_state *cfi,
			     struct stack_op *op)
{
	struct cfi_reg *cfa = &cfi->cfa;
	struct cfi_reg *regs = cfi->regs;

	/* stack operations don't make sense with an undefined CFA */
	if (cfa->base == CFI_UNDEFINED) {
		if (insn->func) {
			WARN_FUNC("undefined stack state", insn->sec, insn->offset);
			return -1;
		}
		return 0;
	}

	if (cfi->type == UNWIND_HINT_TYPE_REGS ||
	    cfi->type == UNWIND_HINT_TYPE_REGS_PARTIAL)
		return update_cfi_state_regs(insn, cfi, op);

	switch (op->dest.type) {

	case OP_DEST_REG:
		switch (op->src.type) {

		case OP_SRC_REG:
			if (op->src.reg == CFI_SP && op->dest.reg == CFI_BP &&
			    cfa->base == CFI_SP &&
			    regs[CFI_BP].base == CFI_CFA &&
			    regs[CFI_BP].offset == -cfa->offset) {

				/* mov %rsp, %rbp */
				cfa->base = op->dest.reg;
				cfi->bp_scratch = false;
			}

			else if (op->src.reg == CFI_SP &&
				 op->dest.reg == CFI_BP && cfi->drap) {

				/* drap: mov %rsp, %rbp */
				regs[CFI_BP].base = CFI_BP;
				regs[CFI_BP].offset = -cfi->stack_size;
				cfi->bp_scratch = false;
			}

			else if (op->src.reg == CFI_SP && cfa->base == CFI_SP) {

				/*
				 * mov %rsp, %reg
				 *
				 * This is needed for the rare case where GCC
				 * does:
				 *
				 *   mov    %rsp, %rax
				 *   ...
				 *   mov    %rax, %rsp
				 */
				cfi->vals[op->dest.reg].base = CFI_CFA;
				cfi->vals[op->dest.reg].offset = -cfi->stack_size;
			}

			else if (op->src.reg == CFI_BP && op->dest.reg == CFI_SP &&
				 cfa->base == CFI_BP) {

				/*
				 * mov %rbp, %rsp
				 *
				 * Restore the original stack pointer (Clang).
				 */
				cfi->stack_size = -cfi->regs[CFI_BP].offset;
			}

			else if (op->dest.reg == cfa->base) {

				/* mov %reg, %rsp */
				if (cfa->base == CFI_SP &&
				    cfi->vals[op->src.reg].base == CFI_CFA) {

					/*
					 * This is needed for the rare case
					 * where GCC does something dumb like:
					 *
					 *   lea    0x8(%rsp), %rcx
					 *   ...
					 *   mov    %rcx, %rsp
					 */
					cfa->offset = -cfi->vals[op->src.reg].offset;
					cfi->stack_size = cfa->offset;

				} else {
					cfa->base = CFI_UNDEFINED;
					cfa->offset = 0;
				}
			}

			break;

		case OP_SRC_ADD:
			if (op->dest.reg == CFI_SP && op->src.reg == CFI_SP) {

				/* add imm, %rsp */
				cfi->stack_size -= op->src.offset;
				if (cfa->base == CFI_SP)
					cfa->offset -= op->src.offset;
				break;
			}

			if (op->dest.reg == CFI_SP && op->src.reg == CFI_BP) {

				/* lea disp(%rbp), %rsp */
				cfi->stack_size = -(op->src.offset + regs[CFI_BP].offset);
				break;
			}

			if (op->src.reg == CFI_SP && cfa->base == CFI_SP) {

				/* drap: lea disp(%rsp), %drap */
				cfi->drap_reg = op->dest.reg;

				/*
				 * lea disp(%rsp), %reg
				 *
				 * This is needed for the rare case where GCC
				 * does something dumb like:
				 *
				 *   lea    0x8(%rsp), %rcx
				 *   ...
				 *   mov    %rcx, %rsp
				 */
				cfi->vals[op->dest.reg].base = CFI_CFA;
				cfi->vals[op->dest.reg].offset = \
					-cfi->stack_size + op->src.offset;

				break;
			}

			if (cfi->drap && op->dest.reg == CFI_SP &&
			    op->src.reg == cfi->drap_reg) {

				 /* drap: lea disp(%drap), %rsp */
				cfa->base = CFI_SP;
				cfa->offset = cfi->stack_size = -op->src.offset;
				cfi->drap_reg = CFI_UNDEFINED;
				cfi->drap = false;
				break;
			}

			if (op->dest.reg == cfi->cfa.base) {
				WARN_FUNC("unsupported stack register modification",
					  insn->sec, insn->offset);
				return -1;
			}

			break;

		case OP_SRC_AND:
			if (op->dest.reg != CFI_SP ||
			    (cfi->drap_reg != CFI_UNDEFINED && cfa->base != CFI_SP) ||
			    (cfi->drap_reg == CFI_UNDEFINED && cfa->base != CFI_BP)) {
				WARN_FUNC("unsupported stack pointer realignment",
					  insn->sec, insn->offset);
				return -1;
			}

			if (cfi->drap_reg != CFI_UNDEFINED) {
				/* drap: and imm, %rsp */
				cfa->base = cfi->drap_reg;
				cfa->offset = cfi->stack_size = 0;
				cfi->drap = true;
			}

			/*
			 * Older versions of GCC (4.8ish) realign the stack
			 * without DRAP, with a frame pointer.
			 */

			break;

		case OP_SRC_POP:
		case OP_SRC_POPF:
			if (!cfi->drap && op->dest.reg == cfa->base) {

				/* pop %rbp */
				cfa->base = CFI_SP;
			}

			if (cfi->drap && cfa->base == CFI_BP_INDIRECT &&
			    op->dest.reg == cfi->drap_reg &&
			    cfi->drap_offset == -cfi->stack_size) {

				/* drap: pop %drap */
				cfa->base = cfi->drap_reg;
				cfa->offset = 0;
				cfi->drap_offset = -1;

			} else if (regs[op->dest.reg].offset == -cfi->stack_size) {

				/* pop %reg */
				restore_reg(cfi, op->dest.reg);
			}

			cfi->stack_size -= 8;
			if (cfa->base == CFI_SP)
				cfa->offset -= 8;

			break;

		case OP_SRC_REG_INDIRECT:
			if (cfi->drap && op->src.reg == CFI_BP &&
			    op->src.offset == cfi->drap_offset) {

				/* drap: mov disp(%rbp), %drap */
				cfa->base = cfi->drap_reg;
				cfa->offset = 0;
				cfi->drap_offset = -1;
			}

			if (cfi->drap && op->src.reg == CFI_BP &&
			    op->src.offset == regs[op->dest.reg].offset) {

				/* drap: mov disp(%rbp), %reg */
				restore_reg(cfi, op->dest.reg);

			} else if (op->src.reg == cfa->base &&
			    op->src.offset == regs[op->dest.reg].offset + cfa->offset) {

				/* mov disp(%rbp), %reg */
				/* mov disp(%rsp), %reg */
				restore_reg(cfi, op->dest.reg);
			}

			break;

		default:
			WARN_FUNC("unknown stack-related instruction",
				  insn->sec, insn->offset);
			return -1;
		}

		break;

	case OP_DEST_PUSH:
	case OP_DEST_PUSHF:
		cfi->stack_size += 8;
		if (cfa->base == CFI_SP)
			cfa->offset += 8;

		if (op->src.type != OP_SRC_REG)
			break;

		if (cfi->drap) {
			if (op->src.reg == cfa->base && op->src.reg == cfi->drap_reg) {

				/* drap: push %drap */
				cfa->base = CFI_BP_INDIRECT;
				cfa->offset = -cfi->stack_size;

				/* save drap so we know when to restore it */
				cfi->drap_offset = -cfi->stack_size;

			} else if (op->src.reg == CFI_BP && cfa->base == cfi->drap_reg) {

				/* drap: push %rbp */
				cfi->stack_size = 0;

			} else {

				/* drap: push %reg */
				save_reg(cfi, op->src.reg, CFI_BP, -cfi->stack_size);
			}

		} else {

			/* push %reg */
			save_reg(cfi, op->src.reg, CFI_CFA, -cfi->stack_size);
		}

		/* detect when asm code uses rbp as a scratch register */
		if (!no_fp && insn->func && op->src.reg == CFI_BP &&
		    cfa->base != CFI_BP)
			cfi->bp_scratch = true;
		break;

	case OP_DEST_REG_INDIRECT:

		if (cfi->drap) {
			if (op->src.reg == cfa->base && op->src.reg == cfi->drap_reg) {

				/* drap: mov %drap, disp(%rbp) */
				cfa->base = CFI_BP_INDIRECT;
				cfa->offset = op->dest.offset;

				/* save drap offset so we know when to restore it */
				cfi->drap_offset = op->dest.offset;
			} else {

				/* drap: mov reg, disp(%rbp) */
				save_reg(cfi, op->src.reg, CFI_BP, op->dest.offset);
			}

		} else if (op->dest.reg == cfa->base) {

			/* mov reg, disp(%rbp) */
			/* mov reg, disp(%rsp) */
			save_reg(cfi, op->src.reg, CFI_CFA,
				 op->dest.offset - cfi->cfa.offset);
		}

		break;

	case OP_DEST_LEAVE:
		if ((!cfi->drap && cfa->base != CFI_BP) ||
		    (cfi->drap && cfa->base != cfi->drap_reg)) {
			WARN_FUNC("leave instruction with modified stack frame",
				  insn->sec, insn->offset);
			return -1;
		}

		/* leave (mov %rbp, %rsp; pop %rbp) */

		cfi->stack_size = -cfi->regs[CFI_BP].offset - 8;
		restore_reg(cfi, CFI_BP);

		if (!cfi->drap) {
			cfa->base = CFI_SP;
			cfa->offset -= 8;
		}

		break;

	case OP_DEST_MEM:
		if (op->src.type != OP_SRC_POP && op->src.type != OP_SRC_POPF) {
			WARN_FUNC("unknown stack-related memory operation",
				  insn->sec, insn->offset);
			return -1;
		}

		/* pop mem */
		cfi->stack_size -= 8;
		if (cfa->base == CFI_SP)
			cfa->offset -= 8;

		break;

	default:
		WARN_FUNC("unknown stack-related instruction",
			  insn->sec, insn->offset);
		return -1;
	}

	return 0;
}

/*
 * The stack layouts of alternatives instructions can sometimes diverge when
 * they have stack modifications.  That's fine as long as the potential stack
 * layouts don't conflict at any given potential instruction boundary.
 *
 * Flatten the CFIs of the different alternative code streams (both original
 * and replacement) into a single shared CFI array which can be used to detect
 * conflicts and nicely feed a linear array of ORC entries to the unwinder.
 */
static int propagate_alt_cfi(struct objtool_file *file, struct instruction *insn)
{
	struct cfi_state **alt_cfi;
	int group_off;

	if (!insn->alt_group)
		return 0;

	if (!insn->cfi) {
		WARN("CFI missing");
		return -1;
	}

	alt_cfi = insn->alt_group->cfi;
	group_off = insn->offset - insn->alt_group->first_insn->offset;

	if (!alt_cfi[group_off]) {
		alt_cfi[group_off] = insn->cfi;
	} else {
		if (cficmp(alt_cfi[group_off], insn->cfi)) {
			WARN_FUNC("stack layout conflict in alternatives",
				  insn->sec, insn->offset);
			return -1;
		}
	}

	return 0;
}

static int handle_insn_ops(struct instruction *insn, struct insn_state *state)
{
	struct stack_op *op;

	list_for_each_entry(op, &insn->stack_ops, list) {

		if (update_cfi_state(insn, &state->cfi, op))
			return 1;

		if (op->dest.type == OP_DEST_PUSHF) {
			if (!state->uaccess_stack) {
				state->uaccess_stack = 1;
			} else if (state->uaccess_stack >> 31) {
				WARN_FUNC("PUSHF stack exhausted",
					  insn->sec, insn->offset);
				return 1;
			}
			state->uaccess_stack <<= 1;
			state->uaccess_stack  |= state->uaccess;
		}

		if (op->src.type == OP_SRC_POPF) {
			if (state->uaccess_stack) {
				state->uaccess = state->uaccess_stack & 1;
				state->uaccess_stack >>= 1;
				if (state->uaccess_stack == 1)
					state->uaccess_stack = 0;
			}
		}
	}

	return 0;
}

static bool insn_cfi_match(struct instruction *insn, struct cfi_state *cfi2)
{
	struct cfi_state *cfi1 = insn->cfi;
	int i;

	if (!cfi1) {
		WARN("CFI missing");
		return false;
	}

	if (memcmp(&cfi1->cfa, &cfi2->cfa, sizeof(cfi1->cfa))) {

		WARN_FUNC("stack state mismatch: cfa1=%d%+d cfa2=%d%+d",
			  insn->sec, insn->offset,
			  cfi1->cfa.base, cfi1->cfa.offset,
			  cfi2->cfa.base, cfi2->cfa.offset);

	} else if (memcmp(&cfi1->regs, &cfi2->regs, sizeof(cfi1->regs))) {
		for (i = 0; i < CFI_NUM_REGS; i++) {
			if (!memcmp(&cfi1->regs[i], &cfi2->regs[i],
				    sizeof(struct cfi_reg)))
				continue;

			WARN_FUNC("stack state mismatch: reg1[%d]=%d%+d reg2[%d]=%d%+d",
				  insn->sec, insn->offset,
				  i, cfi1->regs[i].base, cfi1->regs[i].offset,
				  i, cfi2->regs[i].base, cfi2->regs[i].offset);
			break;
		}

	} else if (cfi1->type != cfi2->type) {

		WARN_FUNC("stack state mismatch: type1=%d type2=%d",
			  insn->sec, insn->offset, cfi1->type, cfi2->type);

	} else if (cfi1->drap != cfi2->drap ||
		   (cfi1->drap && cfi1->drap_reg != cfi2->drap_reg) ||
		   (cfi1->drap && cfi1->drap_offset != cfi2->drap_offset)) {

		WARN_FUNC("stack state mismatch: drap1=%d(%d,%d) drap2=%d(%d,%d)",
			  insn->sec, insn->offset,
			  cfi1->drap, cfi1->drap_reg, cfi1->drap_offset,
			  cfi2->drap, cfi2->drap_reg, cfi2->drap_offset);

	} else
		return true;

	return false;
}

static inline bool func_uaccess_safe(struct symbol *func)
{
	if (func)
		return func->uaccess_safe;

	return false;
}

static inline const char *call_dest_name(struct instruction *insn)
{
	if (insn->call_dest)
		return insn->call_dest->name;

	return "{dynamic}";
}

static inline bool noinstr_call_dest(struct symbol *func)
{
	/*
	 * We can't deal with indirect function calls at present;
	 * assume they're instrumented.
	 */
	if (!func)
		return false;

	/*
	 * If the symbol is from a noinstr section; we good.
	 */
	if (func->sec->noinstr)
		return true;

	/*
	 * The __ubsan_handle_*() calls are like WARN(), they only happen when
	 * something 'BAD' happened. At the risk of taking the machine down,
	 * let them proceed to get the message out.
	 */
	if (!strncmp(func->name, "__ubsan_handle_", 15))
		return true;

	return false;
}

static int validate_call(struct instruction *insn, struct insn_state *state)
{
	if (state->noinstr && state->instr <= 0 &&
	    !noinstr_call_dest(insn->call_dest)) {
		WARN_FUNC("call to %s() leaves .noinstr.text section",
				insn->sec, insn->offset, call_dest_name(insn));
		return 1;
	}

	if (state->uaccess && !func_uaccess_safe(insn->call_dest)) {
		WARN_FUNC("call to %s() with UACCESS enabled",
				insn->sec, insn->offset, call_dest_name(insn));
		return 1;
	}

	if (state->df) {
		WARN_FUNC("call to %s() with DF set",
				insn->sec, insn->offset, call_dest_name(insn));
		return 1;
	}

	return 0;
}

static int validate_sibling_call(struct instruction *insn, struct insn_state *state)
{
	if (has_modified_stack_frame(insn, state)) {
		WARN_FUNC("sibling call from callable instruction with modified stack frame",
				insn->sec, insn->offset);
		return 1;
	}

	return validate_call(insn, state);
}

static int validate_return(struct symbol *func, struct instruction *insn, struct insn_state *state)
{
	if (state->noinstr && state->instr > 0) {
		WARN_FUNC("return with instrumentation enabled",
			  insn->sec, insn->offset);
		return 1;
	}

	if (state->uaccess && !func_uaccess_safe(func)) {
		WARN_FUNC("return with UACCESS enabled",
			  insn->sec, insn->offset);
		return 1;
	}

	if (!state->uaccess && func_uaccess_safe(func)) {
		WARN_FUNC("return with UACCESS disabled from a UACCESS-safe function",
			  insn->sec, insn->offset);
		return 1;
	}

	if (state->df) {
		WARN_FUNC("return with DF set",
			  insn->sec, insn->offset);
		return 1;
	}

	if (func && has_modified_stack_frame(insn, state)) {
		WARN_FUNC("return with modified stack frame",
			  insn->sec, insn->offset);
		return 1;
	}

	if (state->cfi.bp_scratch) {
		WARN_FUNC("BP used as a scratch register",
			  insn->sec, insn->offset);
		return 1;
	}

	return 0;
}

static struct instruction *next_insn_to_validate(struct objtool_file *file,
						 struct instruction *insn)
{
	struct alt_group *alt_group = insn->alt_group;

	/*
	 * Simulate the fact that alternatives are patched in-place.  When the
	 * end of a replacement alt_group is reached, redirect objtool flow to
	 * the end of the original alt_group.
	 */
	if (alt_group && insn == alt_group->last_insn && alt_group->orig_group)
		return next_insn_same_sec(file, alt_group->orig_group->last_insn);

	return next_insn_same_sec(file, insn);
}

/*
 * Follow the branch starting at the given instruction, and recursively follow
 * any other branches (jumps).  Meanwhile, track the frame pointer state at
 * each instruction and validate all the rules described in
 * tools/objtool/Documentation/stack-validation.txt.
 */
static int validate_branch(struct objtool_file *file, struct symbol *func,
			   struct instruction *insn, struct insn_state state)
{
	struct alternative *alt;
	struct instruction *next_insn, *prev_insn = NULL;
	struct section *sec;
	u8 visited;
	int ret;

	sec = insn->sec;

	while (1) {
		next_insn = next_insn_to_validate(file, insn);

		if (file->c_file && func && insn->func && func != insn->func->pfunc) {
			WARN("%s() falls through to next function %s()",
			     func->name, insn->func->name);
			return 1;
		}

		if (func && insn->ignore) {
			WARN_FUNC("BUG: why am I validating an ignored function?",
				  sec, insn->offset);
			return 1;
		}

		visited = VISITED_BRANCH << state.uaccess;
		if (insn->visited & VISITED_BRANCH_MASK) {
			if (!insn->hint && !insn_cfi_match(insn, &state.cfi))
				return 1;

			if (insn->visited & visited)
				return 0;
		} else {
			nr_insns_visited++;
		}

		if (state.noinstr)
			state.instr += insn->instr;

		if (insn->hint) {
			if (insn->restore) {
				struct instruction *save_insn, *i;

				i = insn;
				save_insn = NULL;

				sym_for_each_insn_continue_reverse(file, func, i) {
					if (i->save) {
						save_insn = i;
						break;
					}
				}

				if (!save_insn) {
					WARN_FUNC("no corresponding CFI save for CFI restore",
						  sec, insn->offset);
					return 1;
				}

				if (!save_insn->visited) {
					WARN_FUNC("objtool isn't smart enough to handle this CFI save/restore combo",
						  sec, insn->offset);
					return 1;
				}

				insn->cfi = save_insn->cfi;
				nr_cfi_reused++;
			}

			state.cfi = *insn->cfi;
		} else {
			/* XXX track if we actually changed state.cfi */

			if (prev_insn && !cficmp(prev_insn->cfi, &state.cfi)) {
				insn->cfi = prev_insn->cfi;
				nr_cfi_reused++;
			} else {
				insn->cfi = cfi_hash_find_or_add(&state.cfi);
			}
		}

		insn->visited |= visited;

		if (propagate_alt_cfi(file, insn))
			return 1;

		if (!insn->ignore_alts && !list_empty(&insn->alts)) {
			bool skip_orig = false;

			list_for_each_entry(alt, &insn->alts, list) {
				if (alt->skip_orig)
					skip_orig = true;

				ret = validate_branch(file, func, alt->insn, state);
				if (ret) {
					if (backtrace)
						BT_FUNC("(alt)", insn);
					return ret;
				}
			}

			if (skip_orig)
				return 0;
		}

		if (handle_insn_ops(insn, &state))
			return 1;

		switch (insn->type) {

		case INSN_RETURN:
			if (sls && !insn->retpoline_safe &&
			    next_insn && next_insn->type != INSN_TRAP) {
				WARN_FUNC("missing int3 after ret",
					  insn->sec, insn->offset);
			}
			return validate_return(func, insn, &state);

		case INSN_CALL:
		case INSN_CALL_DYNAMIC:
			ret = validate_call(insn, &state);
			if (ret)
				return ret;

			if (!no_fp && func && !is_special_call(insn) &&
			    !has_valid_stack_frame(&state)) {
				WARN_FUNC("call without frame pointer save/setup",
					  sec, insn->offset);
				return 1;
			}

			if (dead_end_function(file, insn->call_dest))
				return 0;

			break;

		case INSN_JUMP_CONDITIONAL:
		case INSN_JUMP_UNCONDITIONAL:
			if (is_sibling_call(insn)) {
				ret = validate_sibling_call(insn, &state);
				if (ret)
					return ret;

			} else if (insn->jump_dest) {
				ret = validate_branch(file, func,
						      insn->jump_dest, state);
				if (ret) {
					if (backtrace)
						BT_FUNC("(branch)", insn);
					return ret;
				}
			}

			if (insn->type == INSN_JUMP_UNCONDITIONAL)
				return 0;

			break;

		case INSN_JUMP_DYNAMIC:
			if (sls && !insn->retpoline_safe &&
			    next_insn && next_insn->type != INSN_TRAP) {
				WARN_FUNC("missing int3 after indirect jump",
					  insn->sec, insn->offset);
			}

			/* fallthrough */
		case INSN_JUMP_DYNAMIC_CONDITIONAL:
			if (is_sibling_call(insn)) {
				ret = validate_sibling_call(insn, &state);
				if (ret)
					return ret;
			}

			if (insn->type == INSN_JUMP_DYNAMIC)
				return 0;

			break;

		case INSN_CONTEXT_SWITCH:
			if (func && (!next_insn || !next_insn->hint)) {
				WARN_FUNC("unsupported instruction in callable function",
					  sec, insn->offset);
				return 1;
			}
			return 0;

		case INSN_STAC:
			if (state.uaccess) {
				WARN_FUNC("recursive UACCESS enable", sec, insn->offset);
				return 1;
			}

			state.uaccess = true;
			break;

		case INSN_CLAC:
			if (!state.uaccess && func) {
				WARN_FUNC("redundant UACCESS disable", sec, insn->offset);
				return 1;
			}

			if (func_uaccess_safe(func) && !state.uaccess_stack) {
				WARN_FUNC("UACCESS-safe disables UACCESS", sec, insn->offset);
				return 1;
			}

			state.uaccess = false;
			break;

		case INSN_STD:
			if (state.df) {
				WARN_FUNC("recursive STD", sec, insn->offset);
				return 1;
			}

			state.df = true;
			break;

		case INSN_CLD:
			if (!state.df && func) {
				WARN_FUNC("redundant CLD", sec, insn->offset);
				return 1;
			}

			state.df = false;
			break;

		default:
			break;
		}

		if (insn->dead_end)
			return 0;

		if (!next_insn) {
			if (state.cfi.cfa.base == CFI_UNDEFINED)
				return 0;
			WARN("%s: unexpected end of section", sec->name);
			return 1;
		}

		prev_insn = insn;
		insn = next_insn;
	}

	return 0;
}

static int validate_unwind_hints(struct objtool_file *file, struct section *sec)
{
	struct instruction *insn;
	struct insn_state state;
	int ret, warnings = 0;

	if (!file->hints)
		return 0;

	init_insn_state(&state, sec);

	if (sec) {
		insn = find_insn(file, sec, 0);
		if (!insn)
			return 0;
	} else {
		insn = list_first_entry(&file->insn_list, typeof(*insn), list);
	}

	while (&insn->list != &file->insn_list && (!sec || insn->sec == sec)) {
		if (insn->hint && !insn->visited) {
			ret = validate_branch(file, insn->func, insn, state);
			if (ret && backtrace)
				BT_FUNC("<=== (hint)", insn);
			warnings += ret;
		}

		insn = list_next_entry(insn, list);
	}

	return warnings;
}

/*
 * Validate rethunk entry constraint: must untrain RET before the first RET.
 *
 * Follow every branch (intra-function) and ensure ANNOTATE_UNRET_END comes
 * before an actual RET instruction.
 */
static int validate_entry(struct objtool_file *file, struct instruction *insn)
{
	struct instruction *next, *dest;
	int ret, warnings = 0;

	for (;;) {
		next = next_insn_to_validate(file, insn);

		if (insn->visited & VISITED_ENTRY)
			return 0;

		insn->visited |= VISITED_ENTRY;

		if (!insn->ignore_alts && !list_empty(&insn->alts)) {
			struct alternative *alt;
			bool skip_orig = false;

			list_for_each_entry(alt, &insn->alts, list) {
				if (alt->skip_orig)
					skip_orig = true;

				ret = validate_entry(file, alt->insn);
				if (ret) {
				        if (backtrace)
						BT_FUNC("(alt)", insn);
					return ret;
				}
			}

			if (skip_orig)
				return 0;
		}

		switch (insn->type) {

		case INSN_CALL_DYNAMIC:
		case INSN_JUMP_DYNAMIC:
		case INSN_JUMP_DYNAMIC_CONDITIONAL:
			WARN_FUNC("early indirect call", insn->sec, insn->offset);
			return 1;

		case INSN_JUMP_UNCONDITIONAL:
		case INSN_JUMP_CONDITIONAL:
			if (!is_sibling_call(insn)) {
				if (!insn->jump_dest) {
					WARN_FUNC("unresolved jump target after linking?!?",
						  insn->sec, insn->offset);
					return -1;
				}
				ret = validate_entry(file, insn->jump_dest);
				if (ret) {
					if (backtrace) {
						BT_FUNC("(branch%s)", insn,
							insn->type == INSN_JUMP_CONDITIONAL ? "-cond" : "");
					}
					return ret;
				}

				if (insn->type == INSN_JUMP_UNCONDITIONAL)
					return 0;

				break;
			}

			/* fallthrough */
		case INSN_CALL:
			dest = find_insn(file, insn->call_dest->sec,
					 insn->call_dest->offset);
			if (!dest) {
				WARN("Unresolved function after linking!?: %s",
				     insn->call_dest->name);
				return -1;
			}

			ret = validate_entry(file, dest);
			if (ret) {
				if (backtrace)
					BT_FUNC("(call)", insn);
				return ret;
			}
			/*
			 * If a call returns without error, it must have seen UNTRAIN_RET.
			 * Therefore any non-error return is a success.
			 */
			return 0;

		case INSN_RETURN:
			WARN_FUNC("RET before UNTRAIN", insn->sec, insn->offset);
			return 1;

		case INSN_NOP:
			if (insn->retpoline_safe)
				return 0;
			break;

		default:
			break;
		}

		if (!next) {
			WARN_FUNC("teh end!", insn->sec, insn->offset);
			return -1;
		}
		insn = next;
	}

	return warnings;
}

/*
 * Validate that all branches starting at 'insn->entry' encounter UNRET_END
 * before RET.
 */
static int validate_unret(struct objtool_file *file)
{
	struct instruction *insn;
	int ret, warnings = 0;

	for_each_insn(file, insn) {
		if (!insn->entry)
			continue;

		ret = validate_entry(file, insn);
		if (ret < 0) {
			WARN_FUNC("Failed UNRET validation", insn->sec, insn->offset);
			return ret;
		}
		warnings += ret;
	}

	return warnings;
}

static int validate_retpoline(struct objtool_file *file)
{
	struct instruction *insn;
	int warnings = 0;

	for_each_insn(file, insn) {
		if (insn->type != INSN_JUMP_DYNAMIC &&
		    insn->type != INSN_CALL_DYNAMIC &&
		    insn->type != INSN_RETURN)
			continue;

		if (insn->retpoline_safe)
			continue;

		/*
		 * .init.text code is ran before userspace and thus doesn't
		 * strictly need retpolines, except for modules which are
		 * loaded late, they very much do need retpoline in their
		 * .init.text
		 */
		if (!strcmp(insn->sec->name, ".init.text") && !module)
			continue;

		if (insn->type == INSN_RETURN) {
			if (rethunk) {
				WARN_FUNC("'naked' return found in RETHUNK build",
					  insn->sec, insn->offset);
			} else
				continue;
		} else {
			WARN_FUNC("indirect %s found in RETPOLINE build",
				  insn->sec, insn->offset,
				  insn->type == INSN_JUMP_DYNAMIC ? "jump" : "call");
		}

		warnings++;
	}

	return warnings;
}

static bool is_kasan_insn(struct instruction *insn)
{
	return (insn->type == INSN_CALL &&
		!strcmp(insn->call_dest->name, "__asan_handle_no_return"));
}

static bool is_ubsan_insn(struct instruction *insn)
{
	return (insn->type == INSN_CALL &&
		!strcmp(insn->call_dest->name,
			"__ubsan_handle_builtin_unreachable"));
}

static bool ignore_unreachable_insn(struct objtool_file *file, struct instruction *insn)
{
	int i;
	struct instruction *prev_insn;

	if (insn->ignore || insn->type == INSN_NOP || insn->type == INSN_TRAP)
		return true;

	/*
	 * Ignore any unused exceptions.  This can happen when a whitelisted
	 * function has an exception table entry.
	 *
	 * Also ignore alternative replacement instructions.  This can happen
	 * when a whitelisted function uses one of the ALTERNATIVE macros.
	 */
	if (!strcmp(insn->sec->name, ".fixup") ||
	    !strcmp(insn->sec->name, ".altinstr_replacement") ||
	    !strcmp(insn->sec->name, ".altinstr_aux"))
		return true;

	if (!insn->func)
		return false;

	/*
	 * CONFIG_UBSAN_TRAP inserts a UD2 when it sees
	 * __builtin_unreachable().  The BUG() macro has an unreachable() after
	 * the UD2, which causes GCC's undefined trap logic to emit another UD2
	 * (or occasionally a JMP to UD2).
	 *
	 * It may also insert a UD2 after calling a __noreturn function.
	 */
	prev_insn = list_prev_entry(insn, list);
	if ((prev_insn->dead_end || dead_end_function(file, prev_insn->call_dest)) &&
	    (insn->type == INSN_BUG ||
	     (insn->type == INSN_JUMP_UNCONDITIONAL &&
	      insn->jump_dest && insn->jump_dest->type == INSN_BUG)))
		return true;

	/*
	 * Check if this (or a subsequent) instruction is related to
	 * CONFIG_UBSAN or CONFIG_KASAN.
	 *
	 * End the search at 5 instructions to avoid going into the weeds.
	 */
	for (i = 0; i < 5; i++) {

		if (is_kasan_insn(insn) || is_ubsan_insn(insn))
			return true;

		if (insn->type == INSN_JUMP_UNCONDITIONAL) {
			if (insn->jump_dest &&
			    insn->jump_dest->func == insn->func) {
				insn = insn->jump_dest;
				continue;
			}

			break;
		}

		if (insn->offset + insn->len >= insn->func->offset + insn->func->len)
			break;

		insn = list_next_entry(insn, list);
	}

	return false;
}

static int validate_symbol(struct objtool_file *file, struct section *sec,
			   struct symbol *sym, struct insn_state *state)
{
	struct instruction *insn;
	int ret;

	if (!sym->len) {
		WARN("%s() is missing an ELF size annotation", sym->name);
		return 1;
	}

	if (sym->pfunc != sym || sym->alias != sym)
		return 0;

	insn = find_insn(file, sec, sym->offset);
	if (!insn || insn->ignore || insn->visited)
		return 0;

	state->uaccess = sym->uaccess_safe;

	ret = validate_branch(file, insn->func, insn, *state);
	if (ret && backtrace)
		BT_FUNC("<=== (sym)", insn);
	return ret;
}

static int validate_section(struct objtool_file *file, struct section *sec)
{
	struct insn_state state;
	struct symbol *func;
	int warnings = 0;

	list_for_each_entry(func, &sec->symbol_list, list) {
		if (func->type != STT_FUNC)
			continue;

		init_insn_state(&state, sec);
		set_func_state(&state.cfi);

		warnings += validate_symbol(file, sec, func, &state);
	}

	return warnings;
}

static int validate_vmlinux_functions(struct objtool_file *file)
{
	struct section *sec;
	int warnings = 0;

	sec = find_section_by_name(file->elf, ".noinstr.text");
	if (sec) {
		warnings += validate_section(file, sec);
		warnings += validate_unwind_hints(file, sec);
	}

	sec = find_section_by_name(file->elf, ".entry.text");
	if (sec) {
		warnings += validate_section(file, sec);
		warnings += validate_unwind_hints(file, sec);
	}

	return warnings;
}

static int validate_functions(struct objtool_file *file)
{
	struct section *sec;
	int warnings = 0;

	for_each_sec(file, sec) {
		if (!(sec->sh.sh_flags & SHF_EXECINSTR))
			continue;

		warnings += validate_section(file, sec);
	}

	return warnings;
}

static int validate_reachable_instructions(struct objtool_file *file)
{
	struct instruction *insn;

	if (file->ignore_unreachables)
		return 0;

	for_each_insn(file, insn) {
		if (insn->visited || ignore_unreachable_insn(file, insn))
			continue;

		WARN_FUNC("unreachable instruction", insn->sec, insn->offset);
		return 1;
	}

	return 0;
}

int check(struct objtool_file *file)
{
	int ret, warnings = 0;

	arch_initial_func_cfi_state(&initial_func_cfi);
	init_cfi_state(&init_cfi);
	init_cfi_state(&func_cfi);
	set_func_state(&func_cfi);

	if (!cfi_hash_alloc())
		goto out;

	cfi_hash_add(&init_cfi);
	cfi_hash_add(&func_cfi);

	ret = decode_sections(file);
	if (ret < 0)
		goto out;

	warnings += ret;

	if (list_empty(&file->insn_list))
		goto out;

	if (vmlinux && !validate_dup) {
		ret = validate_vmlinux_functions(file);
		if (ret < 0)
			goto out;

		warnings += ret;
		goto out;
	}

	if (retpoline) {
		ret = validate_retpoline(file);
		if (ret < 0)
			return ret;
		warnings += ret;
	}

	ret = validate_functions(file);
	if (ret < 0)
		goto out;
	warnings += ret;

	ret = validate_unwind_hints(file, NULL);
	if (ret < 0)
		goto out;
	warnings += ret;

	if (unret) {
		/*
		 * Must be after validate_branch() and friends, it plays
		 * further games with insn->visited.
		 */
		ret = validate_unret(file);
		if (ret < 0)
			return ret;
		warnings += ret;
	}

	if (!warnings) {
		ret = validate_reachable_instructions(file);
		if (ret < 0)
			goto out;
		warnings += ret;
	}

	ret = create_static_call_sections(file);
	if (ret < 0)
		goto out;
	warnings += ret;

	if (mcount) {
		ret = create_mcount_loc_sections(file);
		if (ret < 0)
			goto out;
		warnings += ret;
	}

	if (retpoline) {
		ret = create_retpoline_sites_sections(file);
		if (ret < 0)
			goto out;
		warnings += ret;
	}

	if (rethunk) {
		ret = create_return_sites_sections(file);
		if (ret < 0)
			goto out;
		warnings += ret;
	}

	if (stats) {
		printf("nr_insns_visited: %ld\n", nr_insns_visited);
		printf("nr_cfi: %ld\n", nr_cfi);
		printf("nr_cfi_reused: %ld\n", nr_cfi_reused);
		printf("nr_cfi_cache: %ld\n", nr_cfi_cache);
	}

out:
	/*
	 *  For now, don't fail the kernel build on fatal warnings.  These
	 *  errors are still fairly common due to the growing matrix of
	 *  supported toolchains and their recent pace of change.
	 */
	return 0;
}
