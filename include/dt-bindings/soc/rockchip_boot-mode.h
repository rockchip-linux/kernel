#ifndef __ROCKCHIP_BOOT_MODE_H
#define __ROCKCHIP_BOOT_MODE_H

/* high 24 bits is tag, low 8 bits is type. */
#define	REBOOT_FLAG		0x5242C300
/* normal boot. */
#define	BOOT_NORMAL		(REBOOT_FLAG + 0)
/* enter loader rockusb mode. */
#define	BOOT_LOADER		(REBOOT_FLAG + 1)
/* enter maskrom rockusb mode. */
#define	BOOT_MASKROM		(0xEF08A53C)
/* enter recovery. */
#define	BOOT_RECOVERY		(REBOOT_FLAG + 3)
/* do not enter recover. */
#define	BOOT_NORECOVER		(REBOOT_FLAG + 4)
/* boot second OS. */
#define	BOOT_SECONDOS		(REBOOT_FLAG + 5)
/* enter recover and wipe data. */
#define	BOOT_WIPEDATA		(REBOOT_FLAG + 6)
/* enter recover and wipe all data. */
#define	BOOT_WIPEALL		(REBOOT_FLAG + 7)
/* check firmware img with backup part. */
#define	BOOT_CHECKIMG		(REBOOT_FLAG + 8)
 /* enter fast boot mode. */
#define	BOOT_FASTBOOT		(REBOOT_FLAG + 9)
#define	BOOT_SECUREBOOT_DISABLE	(REBOOT_FLAG + 10)
/* enter charge mode. */
#define	BOOT_CHARGING		(REBOOT_FLAG + 11)

#endif
