#ifndef _LINUX_CHROMEOS_PLATFORM_H
#define _LINUX_CHROMEOS_PLATFORM_H

#include <linux/errno.h>
#include <linux/types.h>

#ifdef CONFIG_CHROMEOS
/*
 * ChromeOS platform support code. Glue layer between higher level functions
 * and per-platform firmware interfaces.
 */

/* Checks to see if the current device is in devmode */
extern bool chromeos_is_devmode(void);

/*
 * Set the taint bit telling firmware that the currently running side needs
 * recovery (or reinstall).
 */
extern int chromeos_set_need_recovery(void);

#else

/* Stubbed-out versions so we can keep code common */
static inline bool chromeos_is_devmode(void)
{
	return true;
}

static inline int chromeos_set_need_recovery(void)
{
	return -ENODEV;
}
#endif /* CONFIG_CHROMEOS */

#endif /* _LINUX_CHROMEOS_PLATFORM_H */
