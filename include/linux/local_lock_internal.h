/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_LOCAL_LOCK_H
# error "Do not include directly, include linux/local_lock.h"
#endif

#include <linux/percpu-defs.h>
#include <linux/lockdep.h>

typedef struct {
#ifdef CONFIG_PREEMPT_RT
	spinlock_t              lock;
	struct task_struct      *owner;
	int                     nestcnt;

#elif defined(CONFIG_DEBUG_LOCK_ALLOC)
	struct lockdep_map	dep_map;
	struct task_struct	*owner;
#endif
} local_lock_t;

#ifdef CONFIG_PREEMPT_RT

#define INIT_LOCAL_LOCK(lockname)	{	\
	__SPIN_LOCK_UNLOCKED((lockname).lock),	\
	.owner		= NULL,			\
	.nestcnt	= 0,			\
	}
#else

# ifdef CONFIG_DEBUG_LOCK_ALLOC
#  define LL_DEP_MAP_INIT(lockname)			\
	.dep_map = {					\
		.name = #lockname,			\
		.wait_type_inner = LD_WAIT_CONFIG,	\
	}
# else
#  define LL_DEP_MAP_INIT(lockname)
# endif

#define INIT_LOCAL_LOCK(lockname)	{ LL_DEP_MAP_INIT(lockname) }

#endif

#ifdef CONFIG_PREEMPT_RT

static inline void ___local_lock_init(local_lock_t *l)
{
	l->owner = NULL;
	l->nestcnt = 0;
}

#define __local_lock_init(l)					\
do {								\
	spin_lock_init(&(l)->lock);				\
	___local_lock_init(l);					\
} while (0)

#else

#define __local_lock_init(l)					\
do {								\
	static struct lock_class_key __key;			\
								\
	debug_check_no_locks_freed((void *)l, sizeof(*l));	\
	lockdep_init_map_wait(&(l)->dep_map, #l, &__key, 0, LD_WAIT_CONFIG);\
} while (0)
#endif

#ifdef CONFIG_PREEMPT_RT

static inline void local_lock_acquire(local_lock_t *l)
{
	if (l->owner != current) {
		spin_lock(&l->lock);
		DEBUG_LOCKS_WARN_ON(l->owner);
		DEBUG_LOCKS_WARN_ON(l->nestcnt);
		l->owner = current;
	}
	l->nestcnt++;
}

static inline void local_lock_release(local_lock_t *l)
{
	DEBUG_LOCKS_WARN_ON(l->nestcnt == 0);
	DEBUG_LOCKS_WARN_ON(l->owner != current);
	if (--l->nestcnt)
		return;

	l->owner = NULL;
	spin_unlock(&l->lock);
}

#elif defined(CONFIG_DEBUG_LOCK_ALLOC)
static inline void local_lock_acquire(local_lock_t *l)
{
	lock_map_acquire(&l->dep_map);
	DEBUG_LOCKS_WARN_ON(l->owner);
	l->owner = current;
}

static inline void local_lock_release(local_lock_t *l)
{
	DEBUG_LOCKS_WARN_ON(l->owner != current);
	l->owner = NULL;
	lock_map_release(&l->dep_map);
}

#else /* CONFIG_DEBUG_LOCK_ALLOC */
static inline void local_lock_acquire(local_lock_t *l) { }
static inline void local_lock_release(local_lock_t *l) { }
#endif /* !CONFIG_DEBUG_LOCK_ALLOC */

#ifdef CONFIG_PREEMPT_RT

#define __local_lock(lock)					\
	do {							\
		migrate_disable();				\
		local_lock_acquire(this_cpu_ptr(lock));		\
	} while (0)

#define __local_unlock(lock)					\
	do {							\
		local_lock_release(this_cpu_ptr(lock));		\
		migrate_enable();				\
	} while (0)

#define __local_lock_irq(lock)					\
	do {							\
		migrate_disable();				\
		local_lock_acquire(this_cpu_ptr(lock));		\
	} while (0)

#define __local_lock_irqsave(lock, flags)			\
	do {							\
		migrate_disable();				\
		flags = 0;					\
		local_lock_acquire(this_cpu_ptr(lock));		\
	} while (0)

#define __local_unlock_irq(lock)				\
	do {							\
		local_lock_release(this_cpu_ptr(lock));		\
		migrate_enable();				\
	} while (0)

#define __local_unlock_irqrestore(lock, flags)			\
	do {							\
		local_lock_release(this_cpu_ptr(lock));		\
		migrate_enable();				\
	} while (0)

#else

#define __local_lock(lock)					\
	do {							\
		preempt_disable();				\
		local_lock_acquire(this_cpu_ptr(lock));		\
	} while (0)

#define __local_unlock(lock)					\
	do {							\
		local_lock_release(this_cpu_ptr(lock));		\
		preempt_enable();				\
	} while (0)

#define __local_lock_irq(lock)					\
	do {							\
		local_irq_disable();				\
		local_lock_acquire(this_cpu_ptr(lock));		\
	} while (0)

#define __local_lock_irqsave(lock, flags)			\
	do {							\
		local_irq_save(flags);				\
		local_lock_acquire(this_cpu_ptr(lock));		\
	} while (0)

#define __local_unlock_irq(lock)				\
	do {							\
		local_lock_release(this_cpu_ptr(lock));		\
		local_irq_enable();				\
	} while (0)

#define __local_unlock_irqrestore(lock, flags)			\
	do {							\
		local_lock_release(this_cpu_ptr(lock));		\
		local_irq_restore(flags);			\
	} while (0)

#endif
