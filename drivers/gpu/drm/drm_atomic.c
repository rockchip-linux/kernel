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


#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_plane_helper.h>

/**
 * drm_atomic_begin - start a sequence of atomic updates
 * @dev: DRM device
 * @flags: the modifier flags that userspace has requested
 *
 * Begin a sequence of atomic property sets.  Returns a driver
 * private state object that is passed back into the various
 * object's set_property() fxns, and into the remainder of the
 * atomic funcs.  The state object should accumulate the changes
 * from one o more set_property()'s.  At the end, the state can
 * be checked, and optionally committed.
 *
 * RETURNS
 *   a driver state object, which is passed back in to the
 *   various other atomic fxns, or error (such as -EBUSY if
 *   there is still a pending async update)
 */
struct drm_atomic_state *drm_atomic_begin(struct drm_device *dev,
		uint32_t flags)
{
	struct drm_atomic_state *state;
	uint32_t acquire_flags = 0;
	int nplanes = dev->mode_config.num_total_plane;
	int ncrtcs  = dev->mode_config.num_crtc;
	int sz;
	void *ptr;

	sz = sizeof(*state);
	sz += (sizeof(state->planes) + sizeof(state->pstates)) * nplanes;
	sz += (sizeof(state->crtcs) + sizeof(state->cstates)) * ncrtcs;

	ptr = kzalloc(sz, GFP_KERNEL);

	state = ptr;
	ptr = &state[1];

	kref_init(&state->refcount);

	if (flags & DRM_MODE_ATOMIC_NOLOCK)
		acquire_flags |= DRM_MODESET_ACQUIRE_NOLOCK;
	if (flags & DRM_MODE_ATOMIC_NONBLOCK)
		acquire_flags |= DRM_MODESET_ACQUIRE_NONBLOCK;

	drm_modeset_acquire_init(&state->acquire_ctx, acquire_flags);

	state->dev = dev;
	state->flags = flags;

	state->planes = ptr;
	ptr = &state->planes[nplanes];

	state->pstates = ptr;
	ptr = &state->pstates[nplanes];

	state->crtcs = ptr;
	ptr = &state->crtcs[ncrtcs];

	state->cstates = ptr;
	ptr = &state->cstates[ncrtcs];

	return state;
}
EXPORT_SYMBOL(drm_atomic_begin);

/**
 * drm_atomic_set_event - set a pending event on mode object
 * @dev: DRM device
 * @state: the driver state object
 * @obj: the object to set the event on
 * @event: the event to send back
 *
 * Set pending event for an update on the specified object.  The
 * event is to be sent back to userspace after the update completes.
 */
int drm_atomic_set_event(struct drm_device *dev,
		struct drm_atomic_state *state, struct drm_mode_object *obj,
		struct drm_pending_vblank_event *event)
{
	switch (obj->type) {
	case DRM_MODE_OBJECT_CRTC: {
		struct drm_crtc_state *cstate =
			drm_atomic_get_crtc_state(obj_to_crtc(obj), state);
		if (IS_ERR(cstate))
			return PTR_ERR(cstate);
		cstate->event = event;
		return 0;
	}
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(drm_atomic_set_event);

/**
 * drm_atomic_check - validate state object
 * @dev: DRM device
 * @state: the driver state object
 *
 * Check the state object to see if the requested state is
 * physically possible.
 *
 * RETURNS
 * Zero for success or -errno
 */
int drm_atomic_check(struct drm_device *dev, struct drm_atomic_state *state)
{
	struct drm_atomic_state *a = state;
	int nplanes = dev->mode_config.num_total_plane;
	int ncrtcs = dev->mode_config.num_crtc;
	int i, ret = 0;

	for (i = 0; i < nplanes; i++) {
		if (a->planes[i]) {
			ret = drm_atomic_check_plane_state(a->planes[i], a->pstates[i]);
			if (ret)
				break;
		}
	}
	for (i = 0; i < ncrtcs; i++) {
		if (a->crtcs[i]) {
			ret = drm_atomic_check_crtc_state(a->crtcs[i], a->cstates[i]);
			if (ret)
				break;
		}
	}

	a->acquire_ctx.frozen = true;

	return ret;
}
EXPORT_SYMBOL(drm_atomic_check);

/* Note that we drop and re-acquire the locks w/ ww_mutex directly,
 * since we keep the crtc in our list with in_atomic == true.
 */

static void drop_locks(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	struct drm_modeset_acquire_ctx *ctx = &a->acquire_ctx;
	struct drm_modeset_lock *lock;

	mutex_lock(&ctx->mutex);
	list_for_each_entry(lock, &ctx->locked, head)
		ww_mutex_unlock(&lock->mutex);
	mutex_unlock(&ctx->mutex);

	ww_acquire_fini(ww_ctx);
}

static void grab_locks(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	struct drm_modeset_acquire_ctx *ctx = &a->acquire_ctx;
	struct drm_modeset_lock *lock, *slow_locked, *contended;
	int ret;

	lock = slow_locked = contended = NULL;


	ww_acquire_init(ww_ctx, &crtc_ww_class);

	/*
	 * We need to do proper rain^Hww dance.. another context
	 * could sneak in a grab the lock in order to check
	 * crtc->in_atomic, and we get -EDEADLK.  But the winner
	 * will realize the mistake when it sees crtc->in_atomic
	 * already set, and then drop lock and return -EBUSY.
	 * So we just need to keep dancing until we win.
	 */
retry:
	ret = 0;
	list_for_each_entry(lock, &ctx->locked, head) {
		if (lock == slow_locked) {
			slow_locked = NULL;
			continue;
		}
		contended = lock;
		ret = ww_mutex_lock(&lock->mutex, ww_ctx);
		if (ret)
			goto fail;
	}

fail:
	if (ret == -EDEADLK) {
		/* we lost out in a seqno race, backoff, lock and retry.. */

		list_for_each_entry(lock, &ctx->locked, head) {
			if (lock == contended)
				break;
			ww_mutex_unlock(&lock->mutex);
		}

		if (slow_locked)
			ww_mutex_unlock(&slow_locked->mutex);

		ww_mutex_lock_slow(&contended->mutex, ww_ctx);
		slow_locked = contended;
		goto retry;
	}
	WARN_ON(ret);   /* if we get EALREADY then something is fubar */
}

static void commit_locks(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	struct drm_device *dev = a->dev;
	int nplanes = dev->mode_config.num_total_plane;
	int ncrtcs = dev->mode_config.num_crtc;
	int i;

	for (i = 0; i < nplanes; i++) {
		struct drm_plane *plane = a->planes[i];
		if (plane) {
			plane->state->state = NULL;
			drm_plane_destroy_state(plane, a->pstates[i]);
		}
	}

	for (i = 0; i < ncrtcs; i++) {
		struct drm_crtc *crtc = a->crtcs[i];
		if (crtc) {
			crtc->state->state = NULL;
			drm_crtc_destroy_state(crtc, a->cstates[i]);
		}
	}

	/* and properly release them (clear in_atomic, remove from list): */
	drm_modeset_drop_locks(&a->acquire_ctx);
	ww_acquire_fini(ww_ctx);
	a->committed = true;
}

static int atomic_commit(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	int nplanes = a->dev->mode_config.num_total_plane;
	int ncrtcs = a->dev->mode_config.num_crtc;
	int i, ret = 0;

	for (i = 0; i < ncrtcs; i++) {
		struct drm_crtc *crtc = a->crtcs[i];
		if (crtc) {
			ret = drm_atomic_commit_crtc_state(crtc, a->cstates[i]);
			if (ret)
				break;
		}
	}

	for (i = 0; i < nplanes; i++) {
		struct drm_plane *plane = a->planes[i];
		if (plane) {
			ret = drm_atomic_commit_plane_state(plane, a->pstates[i]);
			if (ret)
				break;
		}
	}

	commit_locks(a, ww_ctx);

	return ret;
}

/**
 * drm_atomic_commit - commit state
 * @dev: DRM device
 * @state: the driver state object
 *
 * Commit the state.  This will only be called if atomic_check()
 * succeeds.
 *
 * RETURNS
 * Zero for success or -errno
 */
int drm_atomic_commit(struct drm_device *dev, struct drm_atomic_state *a)
{
	return atomic_commit(a, &a->acquire_ctx.ww_ctx);
}
EXPORT_SYMBOL(drm_atomic_commit);

/**
 * drm_atomic_commit_unlocked - like drm_atomic_commit
 * but can be called back by driver in other thread.  Manages the lock
 * transfer from initiating thread.
 */
int drm_atomic_commit_unlocked(struct drm_device *dev,
		struct drm_atomic_state *a)
{
	struct ww_acquire_ctx ww_ctx;
	grab_locks(a, &ww_ctx);
	return atomic_commit(a, &ww_ctx);
}
EXPORT_SYMBOL(drm_atomic_commit_unlocked);

/**
 * drm_atomic_end - conclude the atomic update
 * @dev: DRM device
 * @state: the driver state object
 *
 * Release resources associated with the state object.
 */
void drm_atomic_end(struct drm_device *dev, struct drm_atomic_state *a)
{
	/* if commit is happening from another thread, it will
	 * block grabbing locks until we drop (and not set
	 * a->committed until after), so this is not a race:
	 */
	if (!a->committed)
		drop_locks(a, &a->acquire_ctx.ww_ctx);

	drm_atomic_state_unreference(a);
}
EXPORT_SYMBOL(drm_atomic_end);

void _drm_atomic_state_free(struct kref *kref)
{
	struct drm_atomic_state *a =
		container_of(kref, struct drm_atomic_state, refcount);

	/* in case we haven't already: */
	if (!a->committed) {
		grab_locks(a, &a->acquire_ctx.ww_ctx);
		commit_locks(a, &a->acquire_ctx.ww_ctx);
	}

	__drm_modeset_acquire_fini(&a->acquire_ctx);

	kfree(a);
}
EXPORT_SYMBOL(_drm_atomic_state_free);

int drm_atomic_plane_set_property(struct drm_plane *plane,
		struct drm_atomic_state *state, struct drm_property *property,
		uint64_t val, void *blob_data)
{
	struct drm_plane_state *pstate = drm_atomic_get_plane_state(plane, state);
	if (IS_ERR(pstate))
		return PTR_ERR(pstate);
	return drm_plane_set_property(plane, pstate, property, val, blob_data);
}
EXPORT_SYMBOL(drm_atomic_plane_set_property);

static void init_plane_state(struct drm_plane *plane,
		struct drm_plane_state *pstate, struct drm_atomic_state *state)
{
	/* snapshot current state: */
	*pstate = *plane->state;
	pstate->state = state;
	if (pstate->fb)
		drm_framebuffer_reference(pstate->fb);
}

struct drm_plane_state *
drm_atomic_get_plane_state(struct drm_plane *plane,
		struct drm_atomic_state *state)
{
	struct drm_atomic_state *a = state;
	struct drm_plane_state *pstate;
	int ret;

	pstate = a->pstates[plane->index];

	if (!pstate) {
		struct drm_modeset_acquire_ctx *ctx = &a->acquire_ctx;

		/* grab lock of current crtc.. if crtc is NULL then grab all: */
		if (plane->state->crtc)
			ret = drm_modeset_lock(&plane->state->crtc->mutex, ctx);
		else
			ret = drm_modeset_lock_all_crtcs(plane->dev, ctx);
		if (ret)
			return ERR_PTR(ret);

		pstate = drm_plane_create_state(plane);
		if (!pstate)
			return ERR_PTR(-ENOMEM);
		init_plane_state(plane, pstate, state);
		a->planes[plane->index] = plane;
		a->pstates[plane->index] = pstate;
	}

	return pstate;
}
EXPORT_SYMBOL(drm_atomic_get_plane_state);

static void
swap_plane_state(struct drm_plane *plane, struct drm_atomic_state *a)
{
	struct drm_plane_state *pstate = a->pstates[plane->index];

	/* clear transient state (only valid during atomic update): */
	pstate->update_plane = false;
	pstate->new_fb = false;

	swap(plane->state, a->pstates[plane->index]);
	plane->base.propvals = &plane->state->propvals;
}

/* For primary plane, if the driver implements ->page_flip(), then
 * we can use that.  But drivers can now choose not to bother with
 * implementing page_flip().
 */
static bool can_flip(struct drm_plane *plane, struct drm_plane_state *pstate)
{
	struct drm_crtc *crtc = pstate->crtc;
	return (plane == crtc->primary) && crtc->funcs->page_flip &&
			!pstate->update_plane;
}

/* clear crtc/fb, ie. after disable_plane().  But takes care to keep
 * the property state in sync.  Once we get rid of plane->crtc/fb ptrs
 * and just use state, we can get rid of this fxn:
 */
static void
reset_plane(struct drm_plane *plane, struct drm_plane_state *pstate)
{
	struct drm_mode_config *config = &plane->dev->mode_config;
	drm_plane_set_property(plane, pstate, config->prop_fb_id, 0, NULL);
	drm_plane_set_property(plane, pstate, config->prop_crtc_id, 0, NULL);
	plane->crtc = NULL;
	plane->fb = NULL;
}

static bool
is_primary_helper(struct drm_plane *plane)
{
#ifdef CONFIG_DRM_KMS_HELPER
	if ((plane->type == DRM_PLANE_TYPE_PRIMARY) &&
			(plane->funcs == &drm_primary_helper_funcs))
		return true;
#endif
	return false;
}

static int
commit_plane_state(struct drm_plane *plane, struct drm_plane_state *pstate)
{
	struct drm_atomic_state *a = pstate->state;
	struct drm_crtc_state *cstate = NULL;
	struct drm_framebuffer *old_fb = plane->fb;
	struct drm_framebuffer *fb = pstate->fb;
	bool enabled = pstate->crtc && fb;
	int ret = 0;

	if (fb)
		drm_framebuffer_reference(fb);

	if (!enabled) {
		if (is_primary_helper(plane)) {
			/* primary plane helpers don't like ->disable_plane()..
			 * so this hack for now until someone comes up with
			 * something better:
			 */
			old_fb = NULL;
			ret = 0;
		} else {
			ret = plane->funcs->disable_plane(plane);
			reset_plane(plane, pstate);
		}
	} else {
		struct drm_crtc *crtc = pstate->crtc;
		cstate = drm_atomic_get_crtc_state(crtc, pstate->state);
		if (pstate->update_plane ||
				(pstate->new_fb && !can_flip(plane, pstate))) {
			WARN_ON(cstate->event);
			ret = plane->funcs->update_plane(plane, crtc, pstate->fb,
					pstate->crtc_x, pstate->crtc_y,
					pstate->crtc_w, pstate->crtc_h,
					pstate->src_x,  pstate->src_y,
					pstate->src_w,  pstate->src_h);
			if (ret == 0) {
				/*
				 * For page_flip(), the driver does this, but for
				 * update_plane() it doesn't.. hurray \o/
				 */
				plane->crtc = crtc;
				plane->fb = fb;
				fb = NULL;  /* don't unref */
			}

		} else if (pstate->new_fb) {
			ret = crtc->funcs->page_flip(crtc, fb, cstate->event, a->flags);
			if (ret == 0) {
				/*
				 * Warn if the driver hasn't properly updated the plane->fb
				 * field to reflect that the new framebuffer is now used.
				 * Failing to do so will screw with the reference counting
				 * on framebuffers.
				 */
				WARN_ON(plane->fb != fb);
				fb = NULL;  /* don't unref */
			}
		} else {
			old_fb = NULL;
			ret = 0;
		}
	}

	if (ret) {
		/* Keep the old fb, don't unref it. */
		old_fb = NULL;
	} else {
		/* on success, update state and fb refcnting: */
		/* NOTE: if we ensure no driver sets plane->state->fb = NULL
		 * on disable, we can move this up a level and not duplicate
		 * nearly the same thing for both update_plane and disable_plane
		 * cases..  I leave it like this for now to be paranoid due to
		 * the slightly different ordering in the two cases in the
		 * original code.
		 */
		swap_plane_state(plane, pstate->state);
		if (cstate)
			cstate->event = NULL;
	}

	if (fb)
		drm_framebuffer_unreference(fb);
	if (old_fb)
		drm_framebuffer_unreference(old_fb);

	return ret;
}

int drm_atomic_crtc_set_property(struct drm_crtc *crtc,
		struct drm_atomic_state *state, struct drm_property *property,
		uint64_t val, void *blob_data)
{
	struct drm_crtc_state *cstate = drm_atomic_get_crtc_state(crtc, state);
	if (IS_ERR(cstate))
		return PTR_ERR(cstate);
	return drm_crtc_set_property(crtc, cstate, property, val, blob_data);
}
EXPORT_SYMBOL(drm_atomic_crtc_set_property);

static void init_crtc_state(struct drm_crtc *crtc,
		struct drm_crtc_state *cstate, struct drm_atomic_state *state)
{
	/* snapshot current state: */
	*cstate = *crtc->state;
	cstate->state = state;

	if (cstate->connector_ids) {
		int sz = cstate->num_connector_ids * sizeof(cstate->connector_ids[0]);
		cstate->connector_ids = kmemdup(cstate->connector_ids, sz, GFP_KERNEL);
	}

	/* this should never happen.. but make sure! */
	WARN_ON(cstate->event);
	cstate->event = NULL;
}

struct drm_crtc_state *
drm_atomic_get_crtc_state(struct drm_crtc *crtc, struct drm_atomic_state *a)
{
	struct drm_crtc_state *cstate;
	int ret;

	cstate = a->cstates[crtc->index];

	if (!cstate) {
		ret = drm_modeset_lock(&crtc->mutex, &a->acquire_ctx);
		if (ret)
			return ERR_PTR(ret);

		cstate = drm_crtc_create_state(crtc);
		if (!cstate)
			return ERR_PTR(-ENOMEM);
		init_crtc_state(crtc, cstate, a);
		a->crtcs[crtc->index] = crtc;
		a->cstates[crtc->index] = cstate;

		/* we'll need it later, so make sure we have state
		 * for primary plane too:
		 */
		drm_atomic_get_plane_state(crtc->primary, a);
	}
	return cstate;
}
EXPORT_SYMBOL(drm_atomic_get_crtc_state);

static void
swap_crtc_state(struct drm_crtc *crtc, struct drm_atomic_state *a)
{
	struct drm_crtc_state *cstate = a->cstates[crtc->index];
	struct drm_device *dev = crtc->dev;
	struct drm_pending_vblank_event *event = cstate->event;

	if (event) {
		/* hrm, need to sort out a better way to send events for
		 * other-than-pageflip.. but modeset is not async, so:
		 */
		unsigned long flags;
		spin_lock_irqsave(&dev->event_lock, flags);
		drm_send_vblank_event(dev, crtc->index, event);
		cstate->event = NULL;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	/* clear transient state (only valid during atomic update): */
	cstate->set_config = false;
	cstate->connectors_change = false;
	cstate->commit_state = false;

	swap(crtc->state, a->cstates[crtc->index]);
	crtc->base.propvals = &crtc->state->propvals;
}

static struct drm_connector **get_connector_set(struct drm_device *dev,
		uint32_t *connector_ids, uint32_t num_connector_ids)
{
	struct drm_connector **connector_set = NULL;
	int i;

	connector_set = kmalloc(num_connector_ids *
			sizeof(struct drm_connector *),
			GFP_KERNEL);
	if (!connector_set)
		return NULL;

	for (i = 0; i < num_connector_ids; i++)
		connector_set[i] = drm_connector_find(dev, connector_ids[i]);

	return connector_set;
}

static int set_config(struct drm_crtc *crtc, struct drm_crtc_state *cstate)
{
	struct drm_device *dev = crtc->dev;
	struct drm_plane_state *pstate =
			drm_atomic_get_plane_state(crtc->primary, cstate->state);
	struct drm_framebuffer *fb = pstate->fb;
	struct drm_connector **connector_set = get_connector_set(crtc->dev,
			cstate->connector_ids, cstate->num_connector_ids);
	struct drm_display_mode *mode = drm_crtc_get_mode(crtc, cstate);
	struct drm_mode_set set = {
			.crtc = crtc,
			.x = pstate->src_x >> 16,
			.y = pstate->src_y >> 16,
			.mode = mode,
			.num_connectors = cstate->num_connector_ids,
			.connectors = connector_set,
			.fb = fb,
	};
	int ret;

	if (IS_ERR(mode)) {
		ret = PTR_ERR(mode);
		return ret;
	}

	if (fb)
		drm_framebuffer_reference(fb);

	ret = drm_mode_set_config_internal(&set);
	if (!ret) {
		swap_crtc_state(crtc, cstate->state);
		pstate->new_fb = pstate->update_plane = false;
	}

	if (fb)
		drm_framebuffer_unreference(fb);

	kfree(connector_set);
	if (mode)
		drm_mode_destroy(dev, mode);
	return ret;
}

static int
commit_crtc_state(struct drm_crtc *crtc,
		struct drm_crtc_state *cstate)
{
	struct drm_plane_state *pstate =
			drm_atomic_get_plane_state(crtc->primary, cstate->state);
	int ret = -EINVAL;

	if (!cstate->commit_state)
		return 0;

	if (cstate->set_config)
		return set_config(crtc, cstate);

	if (!pstate->fb) {
		/* disable */
		struct drm_mode_set set = {
				.crtc = crtc,
				.fb = NULL,
		};

		ret = drm_mode_set_config_internal(&set);
		if (!ret) {
			swap_crtc_state(crtc, cstate->state);
		}
	}

	return ret;
}

const struct drm_atomic_funcs drm_atomic_funcs = {
		.check_plane_state  = drm_plane_check_state,
		.commit_plane_state = commit_plane_state,

		.check_crtc_state   = drm_crtc_check_state,
		.commit_crtc_state  = commit_crtc_state,
};
EXPORT_SYMBOL(drm_atomic_funcs);
