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

#ifndef DRM_ATOMIC_HELPER_H_
#define DRM_ATOMIC_HELPER_H_

/**
 * DOC: atomic state helpers
 *
 * Base helper atomic state and functions.  Drivers are free to either
 * use these as-is, extend them, or completely replace them, in order
 * to implement the atomic KMS API.
 *
 * A naive driver, with no special constraints or hw support for atomic
 * updates may simply add the following to their driver struct:
 *
 *     .atomic_begin     = drm_atomic_begin,
 *     .atomic_set_event = drm_atomic_set_event,
 *     .atomic_check     = drm_atomic_check,
 *     .atomic_commit    = drm_atomic_commit,
 *     .atomic_end       = drm_atomic_end,
 *     .atomics   = &drm_atomic_funcs,
 *
 * In addition, if you're plane/crtc doesn't already have it's own custom
 * properties, then add to your plane/crtc_funcs:
 *
 *     .set_property     = drm_atomic_{plane,crtc}_set_property,
 *
 * Unlike the crtc helpers, it is intended that the atomic helpers can be
 * used piecemeal by the drivers, either using all or overriding parts as
 * needed.
 *
 * A driver which can have (for example) conflicting modes across multiple
 * crtcs (for example, bandwidth limitations or clock/pll configuration
 * restrictions), can simply wrap drm_atomic_check() with their own
 * driver specific .atomic_check() function.
 *
 * A driver which can support true atomic updates can wrap
 * drm_atomic_commit().
 *
 * A driver with custom properties should override the appropriate get_state(),
 * check_state(), and commit_state() functions in .atomics if it uses
 * the drm-atomic-helpers.  Otherwise it is free to use &drm_atomic_funcs
 * as-is.
 */

/**
 * struct drm_atomic_funcs - helper funcs used by the atomic helpers
 */
struct drm_atomic_funcs {
	int (*check_plane_state)(struct drm_plane *plane, struct drm_plane_state *pstate);
	int (*commit_plane_state)(struct drm_plane *plane, struct drm_plane_state *pstate);

	int (*check_crtc_state)(struct drm_crtc *crtc, struct drm_crtc_state *cstate);
	int (*commit_crtc_state)(struct drm_crtc *crtc, struct drm_crtc_state *cstate);
};

const extern struct drm_atomic_funcs drm_atomic_funcs;

struct drm_atomic_state *drm_atomic_begin(struct drm_device *dev,
		uint32_t flags);
int drm_atomic_set_event(struct drm_device *dev,
		struct drm_atomic_state *state, struct drm_mode_object *obj,
		struct drm_pending_vblank_event *event);
int drm_atomic_check(struct drm_device *dev, struct drm_atomic_state *state);
int drm_atomic_commit(struct drm_device *dev, struct drm_atomic_state *state);
int drm_atomic_commit_unlocked(struct drm_device *dev,
		struct drm_atomic_state *state);
void drm_atomic_end(struct drm_device *dev, struct drm_atomic_state *state);

int drm_atomic_plane_set_property(struct drm_plane *plane,
		struct drm_atomic_state *state, struct drm_property *property,
		uint64_t val, void *blob_data);
struct drm_plane_state *drm_atomic_get_plane_state(struct drm_plane *plane,
		struct drm_atomic_state *state);

static inline int
drm_atomic_check_plane_state(struct drm_plane *plane,
		struct drm_plane_state *pstate)
{
	const struct drm_atomic_funcs *funcs =
			plane->dev->driver->atomic_funcs;
	return funcs->check_plane_state(plane, pstate);
}

static inline int
drm_atomic_commit_plane_state(struct drm_plane *plane,
		struct drm_plane_state *pstate)
{
	const struct drm_atomic_funcs *funcs =
			plane->dev->driver->atomic_funcs;
	return funcs->commit_plane_state(plane, pstate);
}

int drm_atomic_crtc_set_property(struct drm_crtc *crtc,
		struct drm_atomic_state *state, struct drm_property *property,
		uint64_t val, void *blob_data);
struct drm_crtc_state *drm_atomic_get_crtc_state(struct drm_crtc *crtc,
		struct drm_atomic_state *state);

static inline int
drm_atomic_check_crtc_state(struct drm_crtc *crtc,
		struct drm_crtc_state *cstate)
{
	const struct drm_atomic_funcs *funcs =
			crtc->dev->driver->atomic_funcs;
	return funcs->check_crtc_state(crtc, cstate);
}

static inline int
drm_atomic_commit_crtc_state(struct drm_crtc *crtc,
		struct drm_crtc_state *cstate)
{
	const struct drm_atomic_funcs *funcs =
			crtc->dev->driver->atomic_funcs;
	return funcs->commit_crtc_state(crtc, cstate);
}

/**
 * struct drm_atomic_state - the state object used by atomic helpers
 */
struct drm_atomic_state {
	struct kref refcount;
	struct drm_device *dev;
	uint32_t flags;
	struct drm_plane **planes;
	struct drm_plane_state **pstates;
	struct drm_crtc **crtcs;
	struct drm_crtc_state **cstates;

	bool committed;
	bool checked;       /* just for debugging */

	struct drm_modeset_acquire_ctx acquire_ctx;
};

static inline void
drm_atomic_state_reference(struct drm_atomic_state *state)
{
	kref_get(&state->refcount);
}

static inline void
drm_atomic_state_unreference(struct drm_atomic_state *state)
{
	void _drm_atomic_state_free(struct kref *kref);
	kref_put(&state->refcount, _drm_atomic_state_free);
}

#endif /* DRM_ATOMIC_HELPER_H_ */
