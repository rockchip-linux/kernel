#ifndef _ARCH_ROCKCHIP_EFUSE_H_
#define _ARCH_ROCKCHIP_EFUSE_H_

struct platform_device;

int rockchip_efuse_get_cpuleakage(struct platform_device *efuse,
				unsigned int *value);

int rockchip_efuse_get_chip_version(struct platform_device *efuse,
				    unsigned int *version);

#endif /* _ARCH_ROCKCHIP_EFUSE_H_ */
