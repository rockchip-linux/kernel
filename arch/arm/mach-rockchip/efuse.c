/*
 * Copyright (C) 2013-2015 ROCKCHIP, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/iomap.h>
#ifdef CONFIG_ARM
#include <linux/rockchip/psci.h>
#endif
#include <asm/compiler.h>
#include <asm/psci.h>
#include <asm/system_info.h>
#include "efuse.h"

#ifdef CONFIG_ARM
#ifdef CONFIG_ARM_TRUSTZONE
#define efuse_readl(offset) \
	rockchip_secure_reg_read(RK3288_EFUSE_PHYS + offset)
#define efuse_writel(val, offset) \
	rockchip_secure_reg_write(RK3288_EFUSE_PHYS + offset, val)
#else
#define efuse_readl(offset) readl_relaxed(RK_EFUSE_VIRT + offset)
#define efuse_writel(val, offset) writel_relaxed(val, RK_EFUSE_VIRT + offset)
#endif
#endif

#define FRAC_BITS 10
#define int_to_frac(x) ((x) << FRAC_BITS)
#define frac_to_int(x) ((x) >> FRAC_BITS)

static u8 efuse_buf[32] = {};

struct rockchip_efuse {
	int (*get_leakage)(int ch);
	int (*get_temp)(int ch);
	int efuse_version;
	int process_version;
};

static struct rockchip_efuse efuse;

struct lkg_conversion_table {
	int temp;
	int conv;
};

/*
 * temp = temp*10
 * conv = exp(-ln(1.2) / 5 * (T - 23)) *100
 */
static const struct lkg_conversion_table conv_table[] = {
	{200, 111},
	{205, 109},
	{210, 107},
	{215, 105},
	{220, 103},
	{225, 101},
	{230, 100},
	{235, 98},
	{240, 96},
	{245, 94},
	{250, 92},
	{255, 91},
	{260, 89},
	{265, 88},
	{270, 86},
	{275, 84},
	{280, 83},
	{285, 81},
	{290, 80},
	{295, 78},
	{300, 77},
	{305, 76},
	{310, 74},
	{315, 73},
	{320, 72},
	{325, 70},
	{330, 69},
	{335, 68},
	{340, 66},
	{345, 65},
	{350, 64},
	{355, 63},
	{360, 62},
	{365, 61},
	{370, 60},
	{375, 58},
	{380, 57},
	{385, 56},
	{390, 55},
	{395, 54},
	{400, 53},
};

#ifdef CONFIG_ARM64
/****************************secure reg access****************************/

#define SEC_REG_RW_SHT (0x0)
#define SEC_REG_RD (0x0)
#define SEC_REG_WR (0x1)

#define SEC_REG_BITS_SHT (0x1)
#define SEC_REG_32 (0x0)
#define SEC_REG_64 (0x2)

#define SEC_REG_RD_32 (SEC_REG_RD | SEC_REG_32)
#define SEC_REG_RD_64 (SEC_REG_RD | SEC_REG_64)
#define SEC_REG_WR_32 (SEC_REG_WR | SEC_REG_32)
#define SEC_REG_WR_64 (SEC_REG_WR | SEC_REG_64)

#define PSCI_SIP_ACCESS_REG		(0x82000002)
#define PSCI_SIP_RKTF_VER		(0x82000001)

static phys_addr_t efuse_phys;

/*
 * arg2: rd/wr control, bit[0] 0-rd 1-rt, bit[1] 0-32bit, 1-64bit
 * arg1: base addr
 * arg0: read or write val
 * function_id: return fail/succes
 */
static u32 reg_wr_fn_smc(u64 function_id, u64 arg0, u64 arg1, u64 arg2)
{
	asm volatile(
			__asmeq("%0", "x0")
			__asmeq("%1", "x1")
			__asmeq("%2", "x2")
			__asmeq("%3", "x3")
			"smc	#0\n"
		: "+r" (function_id), "+r" (arg0)
		: "r" (arg1), "r" (arg2));

	return function_id;
}

static u32 reg_rd_fn_smc(u64 function_id, u64 arg0, u64 arg1, u64 arg2,
			 u64 *val)
{
	asm volatile(
			__asmeq("%0", "x0")
			__asmeq("%1", "x1")
			__asmeq("%2", "x2")
			__asmeq("%3", "x3")
			"smc	#0\n"
		: "+r" (function_id), "+r" (arg0)
		: "r" (arg1), "r" (arg2));

		*val = arg0;

	return function_id;
}

static u32 (*reg_wr_fn)(u64, u64, u64, u64) = reg_wr_fn_smc;
static u32 (*reg_rd_fn)(u64, u64, u64, u64, u64 *) = reg_rd_fn_smc;

static u32 secure_regs_rd_32(u64 addr_phy)
{
	u64 val = 0;

	reg_rd_fn(PSCI_SIP_ACCESS_REG, 0, addr_phy, SEC_REG_RD_32, &val);
	return val;
}

static u32 secure_regs_wr_32(u64 addr_phy, u32 val)
{
	u64 val_64 = val;

	return reg_wr_fn(PSCI_SIP_ACCESS_REG, val_64, addr_phy, SEC_REG_WR_32);
}

static u32 efuse_readl(u32 offset)
{
	return secure_regs_rd_32(efuse_phys + offset);
}

static void efuse_writel(u32 val, u32 offset)
{
	secure_regs_wr_32(efuse_phys + offset, val);
}

#define RKTF_VER_MAJOR(ver) (((ver) >> 16) & 0xffff)
#define RKTF_VER_MINOR(ver) ((ver) & 0xffff)
/* valid ver */
#define RKTF_VLDVER_MAJOR (1)
#define RKTF_VLDVER_MINOR (3)


static int __init rockchip_tf_ver_check(void)
{
	u64 val;
	u32 ver_val;

	ver_val = reg_rd_fn(PSCI_SIP_RKTF_VER, 0, 0, 0, &val);
	if (ver_val == 0xffffffff)
		goto ver_error;

	if ((RKTF_VER_MAJOR(ver_val) >= RKTF_VLDVER_MAJOR) &&
		(RKTF_VER_MINOR(ver_val) >= RKTF_VLDVER_MINOR))
		return 0;

ver_error:

	pr_err("read tf version 0x%x!\n", ver_val);

	do {
		mdelay(1000);
		pr_err("trusted firmware need to update to(%d.%d) or is invaild!\n",
			RKTF_VLDVER_MAJOR, RKTF_VLDVER_MINOR);
	} while(1);

	return 0;
}
device_initcall_sync(rockchip_tf_ver_check);
#endif

static int rk3288_efuse_readregs(u32 addr, u32 length, u8 *buf)
{
	int ret = length;

	if (!length)
		return 0;
	if (!buf)
		return 0;

	efuse_writel(EFUSE_CSB, REG_EFUSE_CTRL);
	efuse_writel(EFUSE_LOAD | EFUSE_PGENB, REG_EFUSE_CTRL);
	udelay(2);
	do {
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) &
			(~(EFUSE_A_MASK << EFUSE_A_SHIFT)), REG_EFUSE_CTRL);
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) |
			((addr & EFUSE_A_MASK) << EFUSE_A_SHIFT),
			REG_EFUSE_CTRL);
		udelay(2);
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) |
				EFUSE_STROBE, REG_EFUSE_CTRL);
		udelay(2);
		*buf = efuse_readl(REG_EFUSE_DOUT);
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) &
				(~EFUSE_STROBE), REG_EFUSE_CTRL);
		udelay(2);
		buf++;
		addr++;
	} while (--length);
	udelay(2);
	efuse_writel(efuse_readl(REG_EFUSE_CTRL) | EFUSE_CSB, REG_EFUSE_CTRL);
	udelay(1);

	return ret;
}

static int __init rk3288_get_efuse_version(void)
{
	int ret = efuse_buf[4] & (~(0x1 << 3));
	return ret;
}

static int __init rk3288_get_process_version(void)
{
	int ret = efuse_buf[6]&0x0f;

	return ret;
}

static int rk3288_get_leakage(int ch)
{
	if ((ch < 0) || (ch > 2))
		return 0;

	return efuse_buf[23+ch];
}

static void __init rk3288_set_system_serial(void)
{
	int i;
	u8 buf[16];

	for (i = 0; i < 8; i++) {
		buf[i] = efuse_buf[8 + (i << 1)];
		buf[i + 8] = efuse_buf[7 + (i << 1)];
	}

	system_serial_low = crc32(0, buf, 8);
	system_serial_high = crc32(system_serial_low, buf + 8, 8);
}

int rk312x_efuse_readregs(u32 addr, u32 length, u8 *buf)
{
	int ret = length;

	if (!length)
		return 0;

	efuse_writel(EFUSE_LOAD, REG_EFUSE_CTRL);
	udelay(2);
	do {
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) &
				(~(EFUSE_A_MASK << RK312X_EFUSE_A_SHIFT)),
				REG_EFUSE_CTRL);
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) |
				((addr & EFUSE_A_MASK) << RK312X_EFUSE_A_SHIFT),
				REG_EFUSE_CTRL);
		udelay(2);
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) |
				EFUSE_STROBE, REG_EFUSE_CTRL);
		udelay(2);
		*buf = efuse_readl(REG_EFUSE_DOUT);
		efuse_writel(efuse_readl(REG_EFUSE_CTRL) &
				(~EFUSE_STROBE), REG_EFUSE_CTRL);
		udelay(2);
		buf++;
		addr++;
	} while (--length);
	udelay(2);
	efuse_writel(efuse_readl(REG_EFUSE_CTRL) &
			(~EFUSE_LOAD) , REG_EFUSE_CTRL);
	udelay(1);

	return ret;
}

int rockchip_efuse_version(void)
{
	return efuse.efuse_version;
}

int rockchip_process_version(void)
{
	return efuse.process_version;
}

int rockchip_get_leakage(int ch)
{
	int ret = 0;

	if (efuse.get_leakage) {
		return efuse.get_leakage(ch);
	} else {
		ret = rk3288_efuse_readregs(0, 32, efuse_buf);
		if (ret == 32)
			return efuse_buf[23+ch];
	}
	return 0;
}

int rockchip_get_cvbs_adjust(void)
{
	return efuse_buf[29] >> 3;
}

int rockchip_efuse_get_temp_adjust(int ch)
{
	int temp;

	if (efuse_buf[31] & 0x80)
		temp = -(efuse_buf[31] & 0x7f);
	else
		temp = efuse_buf[31];

	return temp;
}

/**
 * mul_frac() - multiply two fixed-point numbers
 * @x:	first multiplicand
 * @y:	second multiplicand
 *
 * Return: the result of multiplying two fixed-point numbers.  The
 * result is also a fixed-point number.
 */
static inline s64 mul_frac(s64 x, s64 y)
{
	return (x * y) >> FRAC_BITS;
}

static int get_ft_environment_temp(void)
{
	int temp_efuse;
	int temp_ft;

	/*
	 * the temp is change from 20c to 40c in ft,
	 * in order to improve the precision, we do a conversion.
	 * Ft write temp : temp_efuse = (temp_ft - 20) / (40 - 20) * 63
	 * We conversion temp : temp_ft = (temp_efuse / 63) * (40 - 20) + 20
	 * Reserves a decimal point : temp_ft = temp_ft * 10
	 */
	temp_efuse = (int)((efuse_buf[30] & 0x7e) >> 1);
	temp_ft =
		mul_frac((int_to_frac(temp_efuse) / 63 * 20 + int_to_frac(20)),
			 int_to_frac(10));

	return frac_to_int(temp_ft);
}

static int temp_to_conversion_rate(int temp)
{
	int high, low, mid;

	low = 0;
	high = ARRAY_SIZE(conv_table) - 1;
	mid = (high + low) / 2;

	/* No temp available, return max conversion_rate */
	if (temp < conv_table[low].temp || temp > conv_table[high].temp)
		return 100;

	while (low <= high) {
		if (temp <= conv_table[mid].temp &&
		    temp > conv_table[mid - 1].temp) {
			return conv_table[mid - 1].conv +
			    (conv_table[mid].conv - conv_table[mid - 1].conv) *
				(temp - conv_table[mid - 1].temp) /
			    (conv_table[mid].temp - conv_table[mid - 1].temp);
		} else if (temp > conv_table[mid].temp) {
			low = mid + 1;
		} else {
			high = mid - 1;
		}
		mid = (low + high) / 2;
	}

	return 100;
}

static int rk3368_get_leakage(int ch)
{
	int temp_ft;
	int leakage_efuse;
	int leakage;
	int conversion;

	if ((ch < 0) || (ch > 2))
		return 0;

	leakage_efuse = rk3288_get_leakage(ch);
	if (!leakage_efuse || (leakage_efuse == 0xff))
		return 0;

	/*
	 * because the temp is variational in tf,
	 * and the leakage we need is under 23c, so we convert it.
	 * leakage(23) = leakage_efuse *  exp(-ln(1.2) / 5 * (temp_ft - 23))
	 * leakage(23) = leakage_efuse * lkg_table->conversion / 100
	 */
	temp_ft = get_ft_environment_temp();
	conversion = temp_to_conversion_rate(temp_ft);
	leakage = leakage_efuse * conversion / 100;

	/*
	 * if ft write leakage use 1.35v, need convert to 1v.
	 * leakage(1v) = leakage(1.35v) / 4
	 */
	if ((efuse_buf[26] & 0x80) == 0x80)
		leakage = leakage / 4;

	return leakage;
}

static void __init rk3288_efuse_init(void)
{
	int ret;

	ret = rk3288_efuse_readregs(0, 32, efuse_buf);
	if (ret == 32) {
		if (efuse_buf[2] == 0x33 && efuse_buf[3] == 0x86)
			efuse.get_leakage = rk3368_get_leakage;
		else
			efuse.get_leakage = rk3288_get_leakage;
		efuse.efuse_version = rk3288_get_efuse_version();
		efuse.process_version = rk3288_get_process_version();
		rockchip_set_cpu_version((efuse_buf[6] >> 4) & 3);
		rk3288_set_system_serial();
	} else {
		pr_err("failed to read eFuse, return %d\n", ret);
	}
}

void __init rockchip_efuse_init(void)
{
	int ret;

	if (cpu_is_rk3288() || cpu_is_rk322x()) {
		rk3288_efuse_init();
	} else if (cpu_is_rk312x()) {
		ret = rk312x_efuse_readregs(0, 32, efuse_buf);
		if (ret == 32)
			efuse.get_leakage = rk3288_get_leakage;
		else
			pr_err("failed to read eFuse, return %d\n", ret);
	}
}

#ifdef CONFIG_ARM64
static int __init rockchip_efuse_probe(struct platform_device *pdev)
{
	struct resource *regs;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		return -ENODEV;
	}
	efuse_phys = regs->start;

	rk3288_efuse_init();
	return 0;
}

static const struct of_device_id rockchip_efuse_of_match[] = {
	{ .compatible = "rockchip,rk3368-efuse-256", .data = NULL, },
	{},
};

static struct platform_driver rockchip_efuse_driver = {
	.driver		= {
		.name		= "efuse",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(rockchip_efuse_of_match),
	},
};

static int __init rockchip_efuse_module_init(void)
{
	return platform_driver_probe(&rockchip_efuse_driver,
				     rockchip_efuse_probe);
}
arch_initcall_sync(rockchip_efuse_module_init);
#endif
