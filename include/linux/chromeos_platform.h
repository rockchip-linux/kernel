#ifndef _LINUX_CHROMEOS_PLATFORM_H
#define _LINUX_CHROMEOS_PLATFORM_H

#include <linux/errno.h>
#include <linux/types.h>

extern int chromeos_platform_read_nvram(u8 *nvram_buffer, int buf_size);
extern int chromeos_platform_write_nvram(u8 *nvram_buffer, int buf_size);

#ifdef CONFIG_CHROMEOS
/*
 * ChromeOS platform support code. Glue layer between higher level functions
 * and per-platform firmware interfaces.
 */

/*
 * Set the taint bit telling firmware that the currently running side needs
 * recovery (or reinstall).
 */
extern int chromeos_set_need_recovery(void);

#else

static inline int chromeos_set_need_recovery(void)
{
	return -ENODEV;
}
#endif /* CONFIG_CHROMEOS */

#endif /* _LINUX_CHROMEOS_PLATFORM_H */
