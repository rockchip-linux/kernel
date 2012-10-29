#ifndef _DRM_NOTIFIER_H_
#define _DRM_NOTIFIER_H_

#include <linux/notifier.h>

extern int drm_vblank_register_notifier(struct notifier_block *);
extern int drm_vblank_unregister_notifier(struct notifier_block *);

#endif
