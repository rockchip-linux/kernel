#ifndef _DRIVERS_PLATFORM_X86_CHROMEOS_ACPI_H
#define _DRIVERS_PLATFORM_X86_CHROMEOS_ACPI_H

#include <linux/types.h>

#define NVRAM_BYTES (128 - NVRAM_FIRST_BYTE) /* from drivers/char/nvram.c */

#define CHNV_DEBUG_RESET_FLAG	0x40	     /* flag for S3 reboot */
#define CHNV_RECOVERY_FLAG	0x80	     /* flag for recovery reboot */

#define CHSW_RECOVERY_FW	0x00000002   /* recovery button depressed */
#define CHSW_RECOVERY_EC	0x00000004   /* recovery button depressed */
#define CHSW_DEVELOPER_MODE	0x00000020   /* developer switch set */
#define CHSW_WP			0x00000200   /* write-protect (optional) */

extern int chromeos_acpi_chnv;
extern int chromeos_acpi_chsw;
extern bool chromeos_acpi_available;

#endif /* _DRIVERS_PLATFORM_X86_CHROMEOS_ACPI_H */
