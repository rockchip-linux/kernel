/******************************************************************************
 *
 * Copyright(c) 2013 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#ifdef CONFIG_PLATFORM_AML_S905
#include <linux/version.h>	/* Linux vresion */
#include <linux/printk.h>       /* printk() */
#include <linux/delay.h>	/* msleep() */

extern void sdio_reinit(void);
extern void extern_wifi_set_enable(int is_on);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
extern void wifi_teardown_dt(void);
extern int wifi_setup_dt(void);
#endif /* kernel < 3.14.0 */
#endif /* CONFIG_PLATFORM_AML_S905 */


/*
 * Return:
 *	0:	power on successfully
 *	others: power on failed
 */
int platform_wifi_power_on(void)
{
	int ret = 0;

#ifdef CONFIG_PLATFORM_AML_S905
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
	ret = wifi_setup_dt();
	if (ret) {
		printk("%s: setup dt failed!!(%d)\n", __func__, ret);
		return -1;
	}
#endif /* kernel < 3.14.0 */

#if 0 /* Seems redundancy? Already done before insert driver */
	printk("######%s: \n",__func__);
	extern_wifi_set_enable(0);
	msleep(500);
	extern_wifi_set_enable(1);
	msleep(500);
	sdio_reinit();
#endif
#endif /* CONFIG_PLATFORM_AML_S905 */

	return ret;
}

void platform_wifi_power_off(void)
{
#ifdef CONFIG_PLATFORM_AML_S905
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0))
	wifi_teardown_dt();
#endif /* kernel < 3.14.0 */
#endif /* CONFIG_PLATFORM_AML_S905 */
}
