// SPDX-License-Identifier: GPL-2.0-only
#ifndef __LINUX_SPINLOCK_TYPES_RT_H
#define __LINUX_SPINLOCK_TYPES_RT_H

#ifndef __LINUX_SPINLOCK_TYPES_H
#error "Do not include directly. Include spinlock_types.h instead"
#endif

#include <linux/cache.h>

/*
 * PREEMPT_RT: spinlocks - an RT mutex plus lock-break field:
 */
typedef struct spinlock {
	struct rt_mutex		lock;
	unsigned int		break_lock;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
} spinlock_t;

#define __RT_SPIN_INITIALIZER(name) \
	{ \
	.wait_lock = __RAW_SPIN_LOCK_UNLOCKED(name.wait_lock), \
	.save_state = 1, \
	}
/*
.wait_list = PLIST_HEAD_INIT_RAW((name).lock.wait_list, (name).lock.wait_lock)
*/

#define __SPIN_LOCK_UNLOCKED(name)			\
	{ .lock = __RT_SPIN_INITIALIZER(name.lock),		\
	  SPIN_DEP_MAP_INIT(name) }

#define DEFINE_SPINLOCK(name) \
	spinlock_t name = __SPIN_LOCK_UNLOCKED(name)

#endif
