// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * elf.c - ELF access library
 *
 * Adapted from kpatch (https://github.com/dynup/kpatch):
 * Copyright (C) 2013-2015 Josh Poimboeuf <jpoimboe@redhat.com>
 * Copyright (C) 2014 Seth Jennings <sjenning@redhat.com>
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "builtin.h"

#include "elf.h"
#include "warn.h"

#define MAX_NAME_LEN 128

static inline u32 str_hash(const char *str)
{
	return jhash(str, strlen(str), 0);
}

static inline int elf_hash_bits(void)
{
	return vmlinux ? ELF_HASH_BITS : 16;
}

#define elf_hash_add(hashtable, node, key) \
	hlist_add_head(node, &hashtable[hash_min(key, elf_hash_bits())])

static void elf_hash_init(struct hlist_head *table)
{
	__hash_init(table, 1U << elf_hash_bits());
}

#define elf_hash_for_each_possible(name, obj, member, key)			\
	hlist_for_each_entry(obj, &name[hash_min(key, elf_hash_bits())], member)

static void rb_add(struct rb_root *tree, struct rb_node *node,
		   int (*cmp)(struct rb_node *, const struct rb_node *))
{
	struct rb_node **link = &tree->rb_node;
	struct rb_node *parent = NULL;

	while (*link) {
		parent = *link;
		if (cmp(node, parent) < 0)
			link = &parent->rb_left;
		else
			link = &parent->rb_right;
	}

	rb_link_node(node, parent, link);
	rb_insert_color(node, tree);
}

static struct rb_node *rb_find_first(const struct rb_root *tree, const void *key,
			       int (*cmp)(const void *key, const struct rb_node *))
{
	struct rb_node *node = tree->rb_node;
	struct rb_node *match = NULL;

	while (node) {
		int c = cmp(key, node);
		if (c <= 0) {
			if (!c)
				match = node;
			node = node->rb_left;
		} else if (c > 0) {
			node = node->rb_right;
		}
	}

	return match;
}

static struct rb_node *rb_next_match(struct rb_node *node, const void *key,
				    int (*cmp)(const void *key, const struct rb_node *))
{
	node = rb_next(node);
	if (node && cmp(key, node))
		node = NULL;
	return node;
}

#define rb_for_each(tree, node, key, cmp) \
	for ((node) = rb_find_first((tree), (key), (cmp)); \
	     (node); (node) = rb_next_match((node), (key), (cmp)))

static int symbol_to_offset(struct rb_node *a, const struct rb_node *b)
{
	struct symbol *sa = rb_entry(a, struct symbol, node);
	struct symbol *sb = rb_entry(b, struct symbol, node);

	if (sa->offset < sb->offset)
		return -1;
	if (sa->offset > sb->offset)
		return 1;

	if (sa->len < sb->len)
		return -1;
	if (sa->len > sb->len)
		return 1;

	sa->alias = sb;

	return 0;
}

static int symbol_by_offset(const void *key, const struct rb_node *node)
{
	const struct symbol *s = rb_entry(node, struct symbol, node);
	const unsigned long *o = key;

	if (*o < s->offset)
		return -1;
	if (*o >= s->offset + s->len)
		return 1;

	return 0;
}

struct section *find_section_by_name(const struct elf *elf, const char *name)
{
	struct section *sec;

	elf_hash_for_each_possible(elf->section_name_hash, sec, name_hash, str_hash(name))
		if (!strcmp(sec->name, name))
			return sec;

	return NULL;
}

static struct section *find_section_by_index(struct elf *elf,
					     unsigned int idx)
{
	struct section *sec;

	elf_hash_for_each_possible(elf->section_hash, sec, hash, idx)
		if (sec->idx == idx)
			return sec;

	return NULL;
}

static struct symbol *find_symbol_by_index(struct elf *elf, unsigned int idx)
{
	struct symbol *sym;

	elf_hash_for_each_possible(elf->symbol_hash, sym, hash, idx)
		if (sym->idx == idx)
			return sym;

	return NULL;
}

struct symbol *find_symbol_by_offset(struct section *sec, unsigned long offset)
{
	struct rb_node *node;

	rb_for_each(&sec->symbol_tree, node, &offset, symbol_by_offset) {
		struct symbol *s = rb_entry(node, struct symbol, node);

		if (s->offset == offset && s->type != STT_SECTION)
			return s;
	}

	return NULL;
}

struct symbol *find_func_by_offset(struct section *sec, unsigned long offset)
{
	struct rb_node *node;

	rb_for_each(&sec->symbol_tree, node, &offset, symbol_by_offset) {
		struct symbol *s = rb_entry(node, struct symbol, node);

		if (s->offset == offset && s->type == STT_FUNC)
			return s;
	}

	return NULL;
}

struct symbol *find_symbol_containing(const struct section *sec, unsigned long offset)
{
	struct rb_node *node;

	rb_for_each(&sec->symbol_tree, node, &offset, symbol_by_offset) {
		struct symbol *s = rb_entry(node, struct symbol, node);

		if (s->type != STT_SECTION)
			return s;
	}

	return NULL;
}

struct symbol *find_func_containing(struct section *sec, unsigned long offset)
{
	struct rb_node *node;

	rb_for_each(&sec->symbol_tree, node, &offset, symbol_by_offset) {
		struct symbol *s = rb_entry(node, struct symbol, node);

		if (s->type == STT_FUNC)
			return s;
	}

	return NULL;
}

struct symbol *find_symbol_by_name(const struct elf *elf, const char *name)
{
	struct symbol *sym;

	elf_hash_for_each_possible(elf->symbol_name_hash, sym, name_hash, str_hash(name))
		if (!strcmp(sym->name, name))
			return sym;

	return NULL;
}

struct reloc *find_reloc_by_dest_range(const struct elf *elf, struct section *sec,
				     unsigned long offset, unsigned int len)
{
	struct reloc *reloc, *r = NULL;
	unsigned long o;

	if (!sec->reloc)
		return NULL;

	sec = sec->reloc;

	for_offset_range(o, offset, offset + len) {
		elf_hash_for_each_possible(elf->reloc_hash, reloc, hash,
				       sec_offset_hash(sec, o)) {
			if (reloc->sec != sec)
				continue;

			if (reloc->offset >= offset && reloc->offset < offset + len) {
				if (!r || reloc->offset < r->offset)
					r = reloc;
			}
		}
		if (r)
			return r;
	}

	return NULL;
}

struct reloc *find_reloc_by_dest(const struct elf *elf, struct section *sec, unsigned long offset)
{
	return find_reloc_by_dest_range(elf, sec, offset, 1);
}

static int read_sections(struct elf *elf)
{
	Elf_Scn *s = NULL;
	struct section *sec;
	size_t shstrndx, sections_nr;
	int i;

	if (elf_getshdrnum(elf->elf, &sections_nr)) {
		WARN_ELF("elf_getshdrnum");
		return -1;
	}

	if (elf_getshdrstrndx(elf->elf, &shstrndx)) {
		WARN_ELF("elf_getshdrstrndx");
		return -1;
	}

	for (i = 0; i < sections_nr; i++) {
		sec = malloc(sizeof(*sec));
		if (!sec) {
			perror("malloc");
			return -1;
		}
		memset(sec, 0, sizeof(*sec));

		INIT_LIST_HEAD(&sec->symbol_list);
		INIT_LIST_HEAD(&sec->reloc_list);

		s = elf_getscn(elf->elf, i);
		if (!s) {
			WARN_ELF("elf_getscn");
			return -1;
		}

		sec->idx = elf_ndxscn(s);

		if (!gelf_getshdr(s, &sec->sh)) {
			WARN_ELF("gelf_getshdr");
			return -1;
		}

		sec->name = elf_strptr(elf->elf, shstrndx, sec->sh.sh_name);
		if (!sec->name) {
			WARN_ELF("elf_strptr");
			return -1;
		}

		if (sec->sh.sh_size != 0) {
			sec->data = elf_getdata(s, NULL);
			if (!sec->data) {
				WARN_ELF("elf_getdata");
				return -1;
			}
			if (sec->data->d_off != 0 ||
			    sec->data->d_size != sec->sh.sh_size) {
				WARN("unexpected data attributes for %s",
				     sec->name);
				return -1;
			}
		}
		sec->len = sec->sh.sh_size;

		list_add_tail(&sec->list, &elf->sections);
		elf_hash_add(elf->section_hash, &sec->hash, sec->idx);
		elf_hash_add(elf->section_name_hash, &sec->name_hash, str_hash(sec->name));
	}

	if (stats)
		printf("nr_sections: %lu\n", (unsigned long)sections_nr);

	/* sanity check, one more call to elf_nextscn() should return NULL */
	if (elf_nextscn(elf->elf, s)) {
		WARN("section entry mismatch");
		return -1;
	}

	return 0;
}

static void elf_add_symbol(struct elf *elf, struct symbol *sym)
{
	struct list_head *entry;
	struct rb_node *pnode;

	sym->alias = sym;

	sym->type = GELF_ST_TYPE(sym->sym.st_info);
	sym->bind = GELF_ST_BIND(sym->sym.st_info);

	sym->offset = sym->sym.st_value;
	sym->len = sym->sym.st_size;

	rb_add(&sym->sec->symbol_tree, &sym->node, symbol_to_offset);
	pnode = rb_prev(&sym->node);
	if (pnode)
		entry = &rb_entry(pnode, struct symbol, node)->list;
	else
		entry = &sym->sec->symbol_list;
	list_add(&sym->list, entry);
	elf_hash_add(elf->symbol_hash, &sym->hash, sym->idx);
	elf_hash_add(elf->symbol_name_hash, &sym->name_hash, str_hash(sym->name));

	/*
	 * Don't store empty STT_NOTYPE symbols in the rbtree.  They
	 * can exist within a function, confusing the sorting.
	 */
	if (!sym->len)
		rb_erase(&sym->node, &sym->sec->symbol_tree);
}

static int read_symbols(struct elf *elf)
{
	struct section *symtab, *symtab_shndx, *sec;
	struct symbol *sym, *pfunc;
	int symbols_nr, i;
	char *coldstr;
	Elf_Data *shndx_data = NULL;
	Elf32_Word shndx;

	symtab = find_section_by_name(elf, ".symtab");
	if (!symtab) {
		/*
		 * A missing symbol table is actually possible if it's an empty
		 * .o file.  This can happen for thunk_64.o.
		 */
		return 0;
	}

	symtab_shndx = find_section_by_name(elf, ".symtab_shndx");
	if (symtab_shndx)
		shndx_data = symtab_shndx->data;

	symbols_nr = symtab->sh.sh_size / symtab->sh.sh_entsize;

	for (i = 0; i < symbols_nr; i++) {
		sym = malloc(sizeof(*sym));
		if (!sym) {
			perror("malloc");
			return -1;
		}
		memset(sym, 0, sizeof(*sym));

		sym->idx = i;

		if (!gelf_getsymshndx(symtab->data, shndx_data, i, &sym->sym,
				      &shndx)) {
			WARN_ELF("gelf_getsymshndx");
			goto err;
		}

		sym->name = elf_strptr(elf->elf, symtab->sh.sh_link,
				       sym->sym.st_name);
		if (!sym->name) {
			WARN_ELF("elf_strptr");
			goto err;
		}

		if ((sym->sym.st_shndx > SHN_UNDEF &&
		     sym->sym.st_shndx < SHN_LORESERVE) ||
		    (shndx_data && sym->sym.st_shndx == SHN_XINDEX)) {
			if (sym->sym.st_shndx != SHN_XINDEX)
				shndx = sym->sym.st_shndx;

			sym->sec = find_section_by_index(elf, shndx);
			if (!sym->sec) {
				WARN("couldn't find section for symbol %s",
				     sym->name);
				goto err;
			}
			if (GELF_ST_TYPE(sym->sym.st_info) == STT_SECTION) {
				sym->name = sym->sec->name;
				sym->sec->sym = sym;
			}
		} else
			sym->sec = find_section_by_index(elf, 0);

		elf_add_symbol(elf, sym);
	}

	if (stats)
		printf("nr_symbols: %lu\n", (unsigned long)symbols_nr);

	/* Create parent/child links for any cold subfunctions */
	list_for_each_entry(sec, &elf->sections, list) {
		list_for_each_entry(sym, &sec->symbol_list, list) {
			char pname[MAX_NAME_LEN + 1];
			size_t pnamelen;
			if (sym->type != STT_FUNC)
				continue;

			if (sym->pfunc == NULL)
				sym->pfunc = sym;

			if (sym->cfunc == NULL)
				sym->cfunc = sym;

			coldstr = strstr(sym->name, ".cold");
			if (!coldstr)
				continue;

			pnamelen = coldstr - sym->name;
			if (pnamelen > MAX_NAME_LEN) {
				WARN("%s(): parent function name exceeds maximum length of %d characters",
				     sym->name, MAX_NAME_LEN);
				return -1;
			}

			strncpy(pname, sym->name, pnamelen);
			pname[pnamelen] = '\0';
			pfunc = find_symbol_by_name(elf, pname);

			if (!pfunc) {
				WARN("%s(): can't find parent function",
				     sym->name);
				return -1;
			}

			sym->pfunc = pfunc;
			pfunc->cfunc = sym;

			/*
			 * Unfortunately, -fnoreorder-functions puts the child
			 * inside the parent.  Remove the overlap so we can
			 * have sane assumptions.
			 *
			 * Note that pfunc->len now no longer matches
			 * pfunc->sym.st_size.
			 */
			if (sym->sec == pfunc->sec &&
			    sym->offset >= pfunc->offset &&
			    sym->offset + sym->len == pfunc->offset + pfunc->len) {
				pfunc->len -= sym->len;
			}
		}
	}

	return 0;

err:
	free(sym);
	return -1;
}

static struct section *elf_create_reloc_section(struct elf *elf,
						struct section *base,
						int reltype);

int elf_add_reloc(struct elf *elf, struct section *sec, unsigned long offset,
		  unsigned int type, struct symbol *sym, s64 addend)
{
	struct reloc *reloc;

	if (!sec->reloc && !elf_create_reloc_section(elf, sec, SHT_RELA))
		return -1;

	reloc = malloc(sizeof(*reloc));
	if (!reloc) {
		perror("malloc");
		return -1;
	}
	memset(reloc, 0, sizeof(*reloc));

	reloc->sec = sec->reloc;
	reloc->offset = offset;
	reloc->type = type;
	reloc->sym = sym;
	reloc->addend = addend;

	list_add_tail(&reloc->list, &sec->reloc->reloc_list);
	elf_hash_add(elf->reloc_hash, &reloc->hash, reloc_hash(reloc));

	sec->reloc->changed = true;

	return 0;
}

/*
 * Ensure that any reloc section containing references to @sym is marked
 * changed such that it will get re-generated in elf_rebuild_reloc_sections()
 * with the new symbol index.
 */
static void elf_dirty_reloc_sym(struct elf *elf, struct symbol *sym)
{
	struct section *sec;

	list_for_each_entry(sec, &elf->sections, list) {
		struct reloc *reloc;

		if (sec->changed)
			continue;

		list_for_each_entry(reloc, &sec->reloc_list, list) {
			if (reloc->sym == sym) {
				sec->changed = true;
				break;
			}
		}
	}
}

/*
 * The libelf API is terrible; gelf_update_sym*() takes a data block relative
 * index value, *NOT* the symbol index. As such, iterate the data blocks and
 * adjust index until it fits.
 *
 * If no data block is found, allow adding a new data block provided the index
 * is only one past the end.
 */
static int elf_update_symbol(struct elf *elf, struct section *symtab,
			     struct section *symtab_shndx, struct symbol *sym)
{
	Elf32_Word shndx = sym->sec ? sym->sec->idx : SHN_UNDEF;
	Elf_Data *symtab_data = NULL, *shndx_data = NULL;
	Elf64_Xword entsize = symtab->sh.sh_entsize;
	int max_idx, idx = sym->idx;
	Elf_Scn *s, *t = NULL;
	bool is_special_shndx = sym->sym.st_shndx >= SHN_LORESERVE &&
				sym->sym.st_shndx != SHN_XINDEX;

	if (is_special_shndx)
		shndx = sym->sym.st_shndx;

	s = elf_getscn(elf->elf, symtab->idx);
	if (!s) {
		WARN_ELF("elf_getscn");
		return -1;
	}

	if (symtab_shndx) {
		t = elf_getscn(elf->elf, symtab_shndx->idx);
		if (!t) {
			WARN_ELF("elf_getscn");
			return -1;
		}
	}

	for (;;) {
		/* get next data descriptor for the relevant sections */
		symtab_data = elf_getdata(s, symtab_data);
		if (t)
			shndx_data = elf_getdata(t, shndx_data);

		/* end-of-list */
		if (!symtab_data) {
			void *buf;

			if (idx) {
				/* we don't do holes in symbol tables */
				WARN("index out of range");
				return -1;
			}

			/* if @idx == 0, it's the next contiguous entry, create it */
			symtab_data = elf_newdata(s);
			if (t)
				shndx_data = elf_newdata(t);

			buf = calloc(1, entsize);
			if (!buf) {
				WARN("malloc");
				return -1;
			}

			symtab_data->d_buf = buf;
			symtab_data->d_size = entsize;
			symtab_data->d_align = 1;
			symtab_data->d_type = ELF_T_SYM;

			symtab->sh.sh_size += entsize;
			symtab->changed = true;

			if (t) {
				shndx_data->d_buf = &sym->sec->idx;
				shndx_data->d_size = sizeof(Elf32_Word);
				shndx_data->d_align = sizeof(Elf32_Word);
				shndx_data->d_type = ELF_T_WORD;

				symtab_shndx->sh.sh_size += sizeof(Elf32_Word);
				symtab_shndx->changed = true;
			}

			break;
		}

		/* empty blocks should not happen */
		if (!symtab_data->d_size) {
			WARN("zero size data");
			return -1;
		}

		/* is this the right block? */
		max_idx = symtab_data->d_size / entsize;
		if (idx < max_idx)
			break;

		/* adjust index and try again */
		idx -= max_idx;
	}

	/* something went side-ways */
	if (idx < 0) {
		WARN("negative index");
		return -1;
	}

	/* setup extended section index magic and write the symbol */
	if ((shndx >= SHN_UNDEF && shndx < SHN_LORESERVE) || is_special_shndx) {
		sym->sym.st_shndx = shndx;
		if (!shndx_data)
			shndx = 0;
	} else {
		sym->sym.st_shndx = SHN_XINDEX;
		if (!shndx_data) {
			WARN("no .symtab_shndx");
			return -1;
		}
	}

	if (!gelf_update_symshndx(symtab_data, shndx_data, idx, &sym->sym, shndx)) {
		WARN_ELF("gelf_update_symshndx");
		return -1;
	}

	return 0;
}

static struct symbol *
elf_create_section_symbol(struct elf *elf, struct section *sec)
{
	struct section *symtab, *symtab_shndx;
	Elf32_Word first_non_local, new_idx;
	struct symbol *sym, *old;

	symtab = find_section_by_name(elf, ".symtab");
	if (symtab) {
		symtab_shndx = find_section_by_name(elf, ".symtab_shndx");
	} else {
		WARN("no .symtab");
		return NULL;
	}

	sym = calloc(1, sizeof(*sym));
	if (!sym) {
		perror("malloc");
		return NULL;
	}

	sym->name = sec->name;
	sym->sec = sec;

	// st_name 0
	sym->sym.st_info = GELF_ST_INFO(STB_LOCAL, STT_SECTION);
	// st_other 0
	// st_value 0
	// st_size 0

	/*
	 * Move the first global symbol, as per sh_info, into a new, higher
	 * symbol index. This fees up a spot for a new local symbol.
	 */
	first_non_local = symtab->sh.sh_info;
	new_idx = symtab->sh.sh_size / symtab->sh.sh_entsize;
	old = find_symbol_by_index(elf, first_non_local);
	if (old) {
		old->idx = new_idx;

		hlist_del(&old->hash);
		elf_hash_add(elf->symbol_hash, &old->hash, old->idx);

		elf_dirty_reloc_sym(elf, old);

		if (elf_update_symbol(elf, symtab, symtab_shndx, old)) {
			WARN("elf_update_symbol move");
			return NULL;
		}

		new_idx = first_non_local;
	}

	sym->idx = new_idx;
	if (elf_update_symbol(elf, symtab, symtab_shndx, sym)) {
		WARN("elf_update_symbol");
		return NULL;
	}

	/*
	 * Either way, we added a LOCAL symbol.
	 */
	symtab->sh.sh_info += 1;

	elf_add_symbol(elf, sym);

	return sym;
}

int elf_add_reloc_to_insn(struct elf *elf, struct section *sec,
			  unsigned long offset, unsigned int type,
			  struct section *insn_sec, unsigned long insn_off)
{
	struct symbol *sym = insn_sec->sym;
	int addend = insn_off;

	if (!sym) {
		/*
		 * Due to how weak functions work, we must use section based
		 * relocations. Symbol based relocations would result in the
		 * weak and non-weak function annotations being overlaid on the
		 * non-weak function after linking.
		 */
		sym = elf_create_section_symbol(elf, insn_sec);
		if (!sym)
			return -1;

		insn_sec->sym = sym;
	}

	return elf_add_reloc(elf, sec, offset, type, sym, addend);
}

static int read_rel_reloc(struct section *sec, int i, struct reloc *reloc, unsigned int *symndx)
{
	if (!gelf_getrel(sec->data, i, &reloc->rel)) {
		WARN_ELF("gelf_getrel");
		return -1;
	}
	reloc->type = GELF_R_TYPE(reloc->rel.r_info);
	reloc->addend = 0;
	reloc->offset = reloc->rel.r_offset;
	*symndx = GELF_R_SYM(reloc->rel.r_info);
	return 0;
}

static int read_rela_reloc(struct section *sec, int i, struct reloc *reloc, unsigned int *symndx)
{
	if (!gelf_getrela(sec->data, i, &reloc->rela)) {
		WARN_ELF("gelf_getrela");
		return -1;
	}
	reloc->type = GELF_R_TYPE(reloc->rela.r_info);
	reloc->addend = reloc->rela.r_addend;
	reloc->offset = reloc->rela.r_offset;
	*symndx = GELF_R_SYM(reloc->rela.r_info);
	return 0;
}

static int read_relocs(struct elf *elf)
{
	struct section *sec;
	struct reloc *reloc;
	int i;
	unsigned int symndx;
	unsigned long nr_reloc, max_reloc = 0, tot_reloc = 0;

	list_for_each_entry(sec, &elf->sections, list) {
		if ((sec->sh.sh_type != SHT_RELA) &&
		    (sec->sh.sh_type != SHT_REL))
			continue;

		sec->base = find_section_by_index(elf, sec->sh.sh_info);
		if (!sec->base) {
			WARN("can't find base section for reloc section %s",
			     sec->name);
			return -1;
		}

		sec->base->reloc = sec;

		nr_reloc = 0;
		for (i = 0; i < sec->sh.sh_size / sec->sh.sh_entsize; i++) {
			reloc = malloc(sizeof(*reloc));
			if (!reloc) {
				perror("malloc");
				return -1;
			}
			memset(reloc, 0, sizeof(*reloc));
			switch (sec->sh.sh_type) {
			case SHT_REL:
				if (read_rel_reloc(sec, i, reloc, &symndx))
					return -1;
				break;
			case SHT_RELA:
				if (read_rela_reloc(sec, i, reloc, &symndx))
					return -1;
				break;
			default: return -1;
			}

			reloc->sec = sec;
			reloc->idx = i;
			reloc->sym = find_symbol_by_index(elf, symndx);
			if (!reloc->sym) {
				WARN("can't find reloc entry symbol %d for %s",
				     symndx, sec->name);
				return -1;
			}

			list_add_tail(&reloc->list, &sec->reloc_list);
			elf_hash_add(elf->reloc_hash, &reloc->hash, reloc_hash(reloc));

			nr_reloc++;
		}
		max_reloc = max(max_reloc, nr_reloc);
		tot_reloc += nr_reloc;
	}

	if (stats) {
		printf("max_reloc: %lu\n", max_reloc);
		printf("tot_reloc: %lu\n", tot_reloc);
	}

	return 0;
}

struct elf *elf_open_read(const char *name, int flags)
{
	struct elf *elf;
	Elf_Cmd cmd;

	elf_version(EV_CURRENT);

	elf = malloc(sizeof(*elf));
	if (!elf) {
		perror("malloc");
		return NULL;
	}
	memset(elf, 0, offsetof(struct elf, sections));

	INIT_LIST_HEAD(&elf->sections);

	elf_hash_init(elf->symbol_hash);
	elf_hash_init(elf->symbol_name_hash);
	elf_hash_init(elf->section_hash);
	elf_hash_init(elf->section_name_hash);
	elf_hash_init(elf->reloc_hash);

	elf->fd = open(name, flags);
	if (elf->fd == -1) {
		fprintf(stderr, "objtool: Can't open '%s': %s\n",
			name, strerror(errno));
		goto err;
	}

	if ((flags & O_ACCMODE) == O_RDONLY)
		cmd = ELF_C_READ_MMAP;
	else if ((flags & O_ACCMODE) == O_RDWR)
		cmd = ELF_C_RDWR;
	else /* O_WRONLY */
		cmd = ELF_C_WRITE;

	elf->elf = elf_begin(elf->fd, cmd, NULL);
	if (!elf->elf) {
		WARN_ELF("elf_begin");
		goto err;
	}

	if (!gelf_getehdr(elf->elf, &elf->ehdr)) {
		WARN_ELF("gelf_getehdr");
		goto err;
	}

	if (read_sections(elf))
		goto err;

	if (read_symbols(elf))
		goto err;

	if (read_relocs(elf))
		goto err;

	return elf;

err:
	elf_close(elf);
	return NULL;
}

static int elf_add_string(struct elf *elf, struct section *strtab, char *str)
{
	Elf_Data *data;
	Elf_Scn *s;
	int len;

	if (!strtab)
		strtab = find_section_by_name(elf, ".strtab");
	if (!strtab) {
		WARN("can't find .strtab section");
		return -1;
	}

	s = elf_getscn(elf->elf, strtab->idx);
	if (!s) {
		WARN_ELF("elf_getscn");
		return -1;
	}

	data = elf_newdata(s);
	if (!data) {
		WARN_ELF("elf_newdata");
		return -1;
	}

	data->d_buf = str;
	data->d_size = strlen(str) + 1;
	data->d_align = 1;
	data->d_type = ELF_T_SYM;

	len = strtab->len;
	strtab->len += data->d_size;
	strtab->changed = true;

	return len;
}

struct section *elf_create_section(struct elf *elf, const char *name,
				   unsigned int sh_flags, size_t entsize, int nr)
{
	struct section *sec, *shstrtab;
	size_t size = entsize * nr;
	Elf_Scn *s;

	sec = malloc(sizeof(*sec));
	if (!sec) {
		perror("malloc");
		return NULL;
	}
	memset(sec, 0, sizeof(*sec));

	INIT_LIST_HEAD(&sec->symbol_list);
	INIT_LIST_HEAD(&sec->reloc_list);

	s = elf_newscn(elf->elf);
	if (!s) {
		WARN_ELF("elf_newscn");
		return NULL;
	}

	sec->name = strdup(name);
	if (!sec->name) {
		perror("strdup");
		return NULL;
	}

	sec->idx = elf_ndxscn(s);
	sec->len = size;
	sec->changed = true;

	sec->data = elf_newdata(s);
	if (!sec->data) {
		WARN_ELF("elf_newdata");
		return NULL;
	}

	sec->data->d_size = size;
	sec->data->d_align = 1;

	if (size) {
		sec->data->d_buf = malloc(size);
		if (!sec->data->d_buf) {
			perror("malloc");
			return NULL;
		}
		memset(sec->data->d_buf, 0, size);
	}

	if (!gelf_getshdr(s, &sec->sh)) {
		WARN_ELF("gelf_getshdr");
		return NULL;
	}

	sec->sh.sh_size = size;
	sec->sh.sh_entsize = entsize;
	sec->sh.sh_type = SHT_PROGBITS;
	sec->sh.sh_addralign = 1;
	sec->sh.sh_flags = SHF_ALLOC | sh_flags;

	/* Add section name to .shstrtab (or .strtab for Clang) */
	shstrtab = find_section_by_name(elf, ".shstrtab");
	if (!shstrtab)
		shstrtab = find_section_by_name(elf, ".strtab");
	if (!shstrtab) {
		WARN("can't find .shstrtab or .strtab section");
		return NULL;
	}
	sec->sh.sh_name = elf_add_string(elf, shstrtab, sec->name);
	if (sec->sh.sh_name == -1)
		return NULL;

	list_add_tail(&sec->list, &elf->sections);
	elf_hash_add(elf->section_hash, &sec->hash, sec->idx);
	elf_hash_add(elf->section_name_hash, &sec->name_hash, str_hash(sec->name));

	elf->changed = true;

	return sec;
}

static struct section *elf_create_rel_reloc_section(struct elf *elf, struct section *base)
{
	char *relocname;
	struct section *sec;

	relocname = malloc(strlen(base->name) + strlen(".rel") + 1);
	if (!relocname) {
		perror("malloc");
		return NULL;
	}
	strcpy(relocname, ".rel");
	strcat(relocname, base->name);

	sec = elf_create_section(elf, relocname, 0, sizeof(GElf_Rel), 0);
	free(relocname);
	if (!sec)
		return NULL;

	base->reloc = sec;
	sec->base = base;

	sec->sh.sh_type = SHT_REL;
	sec->sh.sh_addralign = 8;
	sec->sh.sh_link = find_section_by_name(elf, ".symtab")->idx;
	sec->sh.sh_info = base->idx;
	sec->sh.sh_flags = SHF_INFO_LINK;

	return sec;
}

static struct section *elf_create_rela_reloc_section(struct elf *elf, struct section *base)
{
	char *relocname;
	struct section *sec;

	relocname = malloc(strlen(base->name) + strlen(".rela") + 1);
	if (!relocname) {
		perror("malloc");
		return NULL;
	}
	strcpy(relocname, ".rela");
	strcat(relocname, base->name);

	sec = elf_create_section(elf, relocname, 0, sizeof(GElf_Rela), 0);
	free(relocname);
	if (!sec)
		return NULL;

	base->reloc = sec;
	sec->base = base;

	sec->sh.sh_type = SHT_RELA;
	sec->sh.sh_addralign = 8;
	sec->sh.sh_link = find_section_by_name(elf, ".symtab")->idx;
	sec->sh.sh_info = base->idx;
	sec->sh.sh_flags = SHF_INFO_LINK;

	return sec;
}

static struct section *elf_create_reloc_section(struct elf *elf,
					 struct section *base,
					 int reltype)
{
	switch (reltype) {
	case SHT_REL:  return elf_create_rel_reloc_section(elf, base);
	case SHT_RELA: return elf_create_rela_reloc_section(elf, base);
	default:       return NULL;
	}
}

static int elf_rebuild_rel_reloc_section(struct section *sec, int nr)
{
	struct reloc *reloc;
	int idx = 0, size;
	GElf_Rel *relocs;

	/* Allocate a buffer for relocations */
	size = nr * sizeof(*relocs);
	relocs = malloc(size);
	if (!relocs) {
		perror("malloc");
		return -1;
	}

	sec->data->d_buf = relocs;
	sec->data->d_size = size;

	sec->sh.sh_size = size;

	idx = 0;
	list_for_each_entry(reloc, &sec->reloc_list, list) {
		relocs[idx].r_offset = reloc->offset;
		relocs[idx].r_info = GELF_R_INFO(reloc->sym->idx, reloc->type);
		idx++;
	}

	return 0;
}

static int elf_rebuild_rela_reloc_section(struct section *sec, int nr)
{
	struct reloc *reloc;
	int idx = 0, size;
	GElf_Rela *relocs;

	/* Allocate a buffer for relocations with addends */
	size = nr * sizeof(*relocs);
	relocs = malloc(size);
	if (!relocs) {
		perror("malloc");
		return -1;
	}

	sec->data->d_buf = relocs;
	sec->data->d_size = size;

	sec->sh.sh_size = size;

	idx = 0;
	list_for_each_entry(reloc, &sec->reloc_list, list) {
		relocs[idx].r_offset = reloc->offset;
		relocs[idx].r_addend = reloc->addend;
		relocs[idx].r_info = GELF_R_INFO(reloc->sym->idx, reloc->type);
		idx++;
	}

	return 0;
}

static int elf_rebuild_reloc_section(struct elf *elf, struct section *sec)
{
	struct reloc *reloc;
	int nr;

	nr = 0;
	list_for_each_entry(reloc, &sec->reloc_list, list)
		nr++;

	switch (sec->sh.sh_type) {
	case SHT_REL:  return elf_rebuild_rel_reloc_section(sec, nr);
	case SHT_RELA: return elf_rebuild_rela_reloc_section(sec, nr);
	default:       return -1;
	}
}

int elf_write_insn(struct elf *elf, struct section *sec,
		   unsigned long offset, unsigned int len,
		   const char *insn)
{
	Elf_Data *data = sec->data;

	if (data->d_type != ELF_T_BYTE || data->d_off) {
		WARN("write to unexpected data for section: %s", sec->name);
		return -1;
	}

	memcpy(data->d_buf + offset, insn, len);
	elf_flagdata(data, ELF_C_SET, ELF_F_DIRTY);

	elf->changed = true;

	return 0;
}

int elf_write_reloc(struct elf *elf, struct reloc *reloc)
{
	struct section *sec = reloc->sec;

	if (sec->sh.sh_type == SHT_REL) {
		reloc->rel.r_info = GELF_R_INFO(reloc->sym->idx, reloc->type);
		reloc->rel.r_offset = reloc->offset;

		if (!gelf_update_rel(sec->data, reloc->idx, &reloc->rel)) {
			WARN_ELF("gelf_update_rel");
			return -1;
		}
	} else {
		reloc->rela.r_info = GELF_R_INFO(reloc->sym->idx, reloc->type);
		reloc->rela.r_addend = reloc->addend;
		reloc->rela.r_offset = reloc->offset;

		if (!gelf_update_rela(sec->data, reloc->idx, &reloc->rela)) {
			WARN_ELF("gelf_update_rela");
			return -1;
		}
	}

	elf->changed = true;

	return 0;
}

int elf_write(struct elf *elf)
{
	struct section *sec;
	Elf_Scn *s;

	/* Update changed relocation sections and section headers: */
	list_for_each_entry(sec, &elf->sections, list) {
		if (sec->changed) {
			if (sec->base &&
			    elf_rebuild_reloc_section(elf, sec)) {
				WARN("elf_rebuild_reloc_section");
				return -1;
			}

			s = elf_getscn(elf->elf, sec->idx);
			if (!s) {
				WARN_ELF("elf_getscn");
				return -1;
			}
			if (!gelf_update_shdr(s, &sec->sh)) {
				WARN_ELF("gelf_update_shdr");
				return -1;
			}

			sec->changed = false;
			elf->changed = true;
		}
	}

	/* Make sure the new section header entries get updated properly. */
	elf_flagelf(elf->elf, ELF_C_SET, ELF_F_DIRTY);

	/* Write all changes to the file. */
	if (elf_update(elf->elf, ELF_C_WRITE) < 0) {
		WARN_ELF("elf_update");
		return -1;
	}

	elf->changed = false;

	return 0;
}

void elf_close(struct elf *elf)
{
	struct section *sec, *tmpsec;
	struct symbol *sym, *tmpsym;
	struct reloc *reloc, *tmpreloc;

	if (elf->elf)
		elf_end(elf->elf);

	if (elf->fd > 0)
		close(elf->fd);

	list_for_each_entry_safe(sec, tmpsec, &elf->sections, list) {
		list_for_each_entry_safe(sym, tmpsym, &sec->symbol_list, list) {
			list_del(&sym->list);
			hash_del(&sym->hash);
			free(sym);
		}
		list_for_each_entry_safe(reloc, tmpreloc, &sec->reloc_list, list) {
			list_del(&reloc->list);
			hash_del(&reloc->hash);
			free(reloc);
		}
		list_del(&sec->list);
		free(sec);
	}

	free(elf);
}
