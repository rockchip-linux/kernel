#ifndef __MACH_ROCKCHIP_CPU_H
#define __MACH_ROCKCHIP_CPU_H

#include <linux/types.h>

extern unsigned long rockchip_soc_id;

static inline bool cpu_is_rockchip(void)
{
	return rockchip_soc_id;
}

#define ROCKCHIP_CPU_VERION_MASK	0x0000f000
#define ROCKCHIP_CPU_VERION_SHIFT	12

static inline unsigned long rockchip_get_cpu_version(void)
{
	return (rockchip_soc_id & ROCKCHIP_CPU_VERION_MASK)
		>> ROCKCHIP_CPU_VERION_SHIFT;
}

static inline void rockchip_set_cpu_version(unsigned long ver)
{
	rockchip_soc_id &= ~ROCKCHIP_CPU_VERION_MASK;
	rockchip_soc_id |=
		(ver << ROCKCHIP_CPU_VERION_SHIFT) & ROCKCHIP_CPU_VERION_MASK;
}

#define ROCKCHIP_CPU_MASK       0xffff0000
#define ROCKCHIP_CPU_RK2928     0x29280000
#define ROCKCHIP_CPU_RK3026     0x30260000
#define ROCKCHIP_CPU_RK312X     0x31260000
#define ROCKCHIP_CPU_RK3036     0x30360000
#define ROCKCHIP_CPU_RK30XX     0x30660000
#define ROCKCHIP_CPU_RK3066B    0x31680000
#define ROCKCHIP_CPU_RK3188     0x31880000
#define ROCKCHIP_CPU_RK319X     0x31900000
#define ROCKCHIP_CPU_RK3288     0x32880000
#define ROCKCHIP_CPU_RK322X     0x32280000
#define ROCKCHIP_CPU_RV110X     0x11080000

#ifdef CONFIG_ARM
#define ROCKCHIP_CPU(id, ID) \
static inline bool cpu_is_##id(void) \
{ \
	return (rockchip_soc_id & ROCKCHIP_CPU_MASK) == ROCKCHIP_CPU_ ##ID; \
}
#else
#define ROCKCHIP_CPU(id, ID) \
static inline bool cpu_is_##id(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK2928
ROCKCHIP_CPU(rk2928, RK2928)
#else
static inline bool cpu_is_rk2928(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK3026
ROCKCHIP_CPU(rk3026, RK3026)
#else
static inline bool cpu_is_rk3026(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK3036
ROCKCHIP_CPU(rk3036, RK3036)
#else
static inline bool cpu_is_rk3036(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK30XX
ROCKCHIP_CPU(rk30xx, RK30XX)
#else
static inline bool cpu_is_rk30xx(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK3066B
ROCKCHIP_CPU(rk3066b, RK3066B)
#else
static inline bool cpu_is_rk3066b(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK312X
ROCKCHIP_CPU(rk312x, RK312X)
#else
static inline bool cpu_is_rk312x(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK3188
ROCKCHIP_CPU(rk3188, RK3188)
#else
static inline bool cpu_is_rk3188(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK319X
ROCKCHIP_CPU(rk319x, RK319X)
#else
static inline bool cpu_is_rk319x(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK3288
ROCKCHIP_CPU(rk3288, RK3288)
#else
static inline bool cpu_is_rk3288(void) { return false; }
#endif

#ifdef CONFIG_CPU_RK322X
ROCKCHIP_CPU(rk322x, RK322X)
#else
static inline bool cpu_is_rk322x(void) { return false; }
#endif

#ifdef CONFIG_CPU_RV110X
ROCKCHIP_CPU(rv110x, RV110X)
#else
static inline bool cpu_is_rv110x(void) { return false; }
#endif

#define ROCKCHIP_SOC_MASK	(ROCKCHIP_CPU_MASK | 0xff)
#define ROCKCHIP_SOC_RK2926     (ROCKCHIP_CPU_RK2928 | 0x00)
#define ROCKCHIP_SOC_RK2928G    (ROCKCHIP_CPU_RK2928 | 0x01)
#define ROCKCHIP_SOC_RK2928L    (ROCKCHIP_CPU_RK2928 | 0x02)
#define ROCKCHIP_SOC_RK3028A    (ROCKCHIP_CPU_RK3026 | 0x03)
#define ROCKCHIP_SOC_RK3026     (ROCKCHIP_CPU_RK3026 | 0x04)
#define ROCKCHIP_SOC_RK3126     (ROCKCHIP_CPU_RK312X | 0x00)
#define ROCKCHIP_SOC_RK3126B    (ROCKCHIP_CPU_RK312X | 0x10)
#define ROCKCHIP_SOC_RK3126C    (ROCKCHIP_CPU_RK312X | 0x20)
#define ROCKCHIP_SOC_RK3128     (ROCKCHIP_CPU_RK312X | 0x01)
#define ROCKCHIP_SOC_RK3036     (ROCKCHIP_CPU_RK3036 | 0x00)
#define ROCKCHIP_SOC_RK3000     (ROCKCHIP_CPU_RK30XX | 0x00)
#define ROCKCHIP_SOC_RK3066     (ROCKCHIP_CPU_RK30XX | 0x01)
#define ROCKCHIP_SOC_RK3068     (ROCKCHIP_CPU_RK30XX | 0x02)
#define ROCKCHIP_SOC_RK3066B    (ROCKCHIP_CPU_RK3066B| 0x00)
#define ROCKCHIP_SOC_RK3168     (ROCKCHIP_CPU_RK3066B| 0x01)
#define ROCKCHIP_SOC_RK3028     (ROCKCHIP_CPU_RK3066B| 0x03)
#define ROCKCHIP_SOC_RK3188     (ROCKCHIP_CPU_RK3188 | 0x00)
#define ROCKCHIP_SOC_RK3188PLUS (ROCKCHIP_CPU_RK3188 | 0x10)
#define ROCKCHIP_SOC_RK3190     (ROCKCHIP_CPU_RK319X | 0x00)
#define ROCKCHIP_SOC_RK3288     (ROCKCHIP_CPU_RK3288 | 0x00)
#define ROCKCHIP_SOC_RK3288W    (ROCKCHIP_CPU_RK3288 | 0x01)
#define ROCKCHIP_SOC_RK3228A    (ROCKCHIP_CPU_RK322X | 0x00)
#define ROCKCHIP_SOC_RK3228B    (ROCKCHIP_CPU_RK322X | 0x01)
#define ROCKCHIP_SOC_RK3229     (ROCKCHIP_CPU_RK322X | 0x02)
#define ROCKCHIP_SOC_RV1107     (ROCKCHIP_CPU_RV110X | 0x01)
#define ROCKCHIP_SOC_RV1108     (ROCKCHIP_CPU_RV110X | 0x00)

#ifdef CONFIG_ARM
#define ROCKCHIP_SOC(id, ID) \
static inline bool soc_is_##id(void) \
{ \
	return (rockchip_soc_id & ROCKCHIP_SOC_MASK) == ROCKCHIP_SOC_ ##ID; \
}
#else
#define ROCKCHIP_SOC(id, ID) \
static inline bool soc_is_##id(void) { return false; }
#endif

ROCKCHIP_SOC(rk2926, RK2926)
ROCKCHIP_SOC(rk2928g, RK2928G)
ROCKCHIP_SOC(rk2928l, RK2928L)
ROCKCHIP_SOC(rk3028a, RK3028A)
ROCKCHIP_SOC(rk3026, RK3026)
ROCKCHIP_SOC(rk3126, RK3126)
ROCKCHIP_SOC(rk3126b, RK3126B)
ROCKCHIP_SOC(rk3126c, RK3126C)
ROCKCHIP_SOC(rk3128, RK3128)
ROCKCHIP_SOC(rk3036, RK3036)
ROCKCHIP_SOC(rk3000, RK3000)
ROCKCHIP_SOC(rk3066, RK3066)
ROCKCHIP_SOC(rk3068, RK3068)
ROCKCHIP_SOC(rk3066b, RK3066B)
ROCKCHIP_SOC(rk3168, RK3168)
ROCKCHIP_SOC(rk3028, RK3028)
ROCKCHIP_SOC(rk3188, RK3188)
ROCKCHIP_SOC(rk3188plus, RK3188PLUS)
ROCKCHIP_SOC(rk3190, RK3190)
ROCKCHIP_SOC(rk3288, RK3288)
ROCKCHIP_SOC(rk3288w, RK3288W)
ROCKCHIP_SOC(rk3228a, RK3228A)
ROCKCHIP_SOC(rk3228b, RK3228B)
ROCKCHIP_SOC(rk3229, RK3229)
ROCKCHIP_SOC(rv1107, RV1107)
ROCKCHIP_SOC(rv1108, RV1108)

#endif
