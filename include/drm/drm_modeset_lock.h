/*
 * Copyright (C) 2014 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef DRM_MODESET_LOCK_H_
#define DRM_MODESET_LOCK_H_

#include <linux/ww_mutex.h>

struct drm_modeset_lock;

/**
 * drm_modeset_acquire_ctx - locking context (see ww_acquire_ctx)
 * @ww_ctx: base acquire ctx
 * @contended: used internally for -EDEADLK handling
 * @nolock: bypass locking (for emergencies)
 * @nonblock: don't block
 * @frozen: for debugging
 * @locked: list of held locks
 *
 * Each thread competing for a set of locks must use one acquire
 * ctx.  And if any lock fxn returns -EDEADLK, it must backoff and
 * retry.
 */
struct drm_modeset_acquire_ctx {

	struct ww_acquire_ctx ww_ctx;

	bool nolock : 1;
	bool nonblock : 1;

	/**
	 * Just for debugging, the context is 'frozen' in drm_atomic_check()
	 * to catch anyone who might be trying to acquire a lock after it is
	 * too late.
	 */
	bool frozen : 1;

	/**
	 * Contended lock: if a lock is contended you should only call
	 * drm_modeset_backoff() which drops locks and slow-locks the
	 * contended lock.
	 */
	struct drm_modeset_lock *contended;

	/**
	 * list of held locks (drm_modeset_lock)
	 */
	struct list_head locked;

	/* currently simply for protecting against 'locked' list manipulation
	 * between original thread calling atomic->end() and driver thread
	 * calling back drm_atomic_commit_unlocked().
	 *
	 * Other spots are sufficiently synchronized by virtue of holding
	 * the lock's ww_mutex.  But during the lock/resource hand-over to the
	 * driver thread (drop_locks()/grab_locks()), we cannot rely on this.
	 */
	struct mutex mutex;
};

/**
 * drm_modeset_lock - used for locking modeset resources.
 * @mutex: resource locking
 * @atomic_pending: is this resource part of a still-pending
 *    atomic update
 * @head: used to hold it's place on state->locked list when
 *    part of an atomic update
 *
 * Used for locking CRTCs and other modeset resources.
 */
struct drm_modeset_lock {
	/**
	 * modeset lock
	 */
	struct ww_mutex mutex;

	/**
	 * Are we busy (pending asynchronous/NONBLOCK update)?  Any further
	 * asynchronous update will return -EBUSY if it also needs to acquire
	 * this lock.  While a synchronous update will block until the pending
	 * async update completes.
	 *
	 * Drivers must ensure the update is completed before sending vblank
	 * event to userspace.  Typically this just means don't send event
	 * before drm_atomic_commit_unlocked() returns.
	 */
	bool atomic_pending;

	/**
	 * Resources that are locked as part of an atomic update are added
	 * to a list (so we know what to unlock at the end).
	 */
	struct list_head head;

	/**
	 * For waiting on atomic_pending locks, if not a NONBLOCK operation.
	 */
	wait_queue_head_t event;
};

extern struct ww_class crtc_ww_class;

#define DRM_MODESET_ACQUIRE_NOLOCK     0x0001
#define DRM_MODESET_ACQUIRE_NONBLOCK   0x0002

void drm_modeset_acquire_init(struct drm_modeset_acquire_ctx *ctx,
		uint32_t flags);
void __drm_modeset_acquire_fini(struct drm_modeset_acquire_ctx *ctx);
void drm_modeset_acquire_fini(struct drm_modeset_acquire_ctx *ctx);
void drm_modeset_drop_locks(struct drm_modeset_acquire_ctx *ctx);
void drm_modeset_backoff(struct drm_modeset_acquire_ctx *ctx);
int drm_modeset_backoff_interruptible(struct drm_modeset_acquire_ctx *ctx);

/**
 * drm_modeset_lock_init - initialize lock
 * @lock: lock to init
 */
static inline void drm_modeset_lock_init(struct drm_modeset_lock *lock)
{
	ww_mutex_init(&lock->mutex, &crtc_ww_class);
	INIT_LIST_HEAD(&lock->head);
	init_waitqueue_head(&lock->event);
}

/**
 * drm_modeset_lock_fini - cleanup lock
 * @lock: lock to cleanup
 */
static inline void drm_modeset_lock_fini(struct drm_modeset_lock *lock)
{
	WARN_ON(!list_empty(&lock->head));
}

/**
 * drm_modeset_is_locked - equivalent to mutex_is_locked()
 * @lock: lock to check
 */
static inline bool drm_modeset_is_locked(struct drm_modeset_lock *lock)
{
	return ww_mutex_is_locked(&lock->mutex);
}

int drm_modeset_lock(struct drm_modeset_lock *lock,
		struct drm_modeset_acquire_ctx *ctx);
int drm_modeset_lock_interruptible(struct drm_modeset_lock *lock,
		struct drm_modeset_acquire_ctx *ctx);
void drm_modeset_unlock(struct drm_modeset_lock *lock);

struct drm_device;
int drm_modeset_lock_all_crtcs(struct drm_device *dev,
		struct drm_modeset_acquire_ctx *ctx);

#endif /* DRM_MODESET_LOCK_H_ */
