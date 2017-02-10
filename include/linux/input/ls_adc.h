/* drivers/input/sensors/lsensor/ls_adc.c
 *
 * light sensor driver for the rv1108
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ROCKCHIP_LS_ADC_H__
#define __ROCKCHIP_LS_ADC_H__

#ifdef CONFIG_LS_ADC
int lightsensor_vol_r(struct device *dev,
	int *light_vol);
#else
static inline int lightsensor_vol_r(struct device *dev,
	int *light_vol)
{
	return -ENODEV;
};
#endif

#endif
