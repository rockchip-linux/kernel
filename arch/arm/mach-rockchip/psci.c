/*
 * arch/arm/mach-rockchip/psci.c
 *
 * PSCI call interface for rockchip
 *
 * Copyright (C) 2015 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/of.h>
#include <linux/types.h>
#include <linux/rockchip/psci.h>
#include <asm/compiler.h>
#include <asm/smp_plat.h>
#ifdef CONFIG_ARM_PSCI
#include <asm/psci.h>
#endif
#ifdef CONFIG_ARM
#include <linux/mm.h>
#include <asm/opcodes-sec.h>
#include <linux/dma-mapping.h>
#endif

static int sip_version;

static void __invoke_sip_fn_smc32(u32 function_id, u32 arg0,
				  u32 arg1, u32 arg2, struct arm_smccc_res *res)
{
	asm volatile(
#ifdef CONFIG_ARM
			__asmeq("%0", "r0")
			__asmeq("%1", "r1")
			__asmeq("%2", "r2")
			__asmeq("%3", "r3")
			__SMC(0)
#else
			__asmeq("%w0", "w0")
			__asmeq("%w1", "w1")
			__asmeq("%w2", "w2")
			__asmeq("%w3", "w3")
			"smc	#0\n"
#endif
		: "+r" (function_id), "+r" (arg0)
		: "r" (arg1), "r" (arg2));

	if (res) {
		res->a0 = function_id;
		res->a1 = arg0;
		res->a2 = arg1;
		res->a3 = arg2;
	}
}

static void (*sip_fn_smc32)(u32, u32, u32, u32, struct arm_smccc_res *) =
							__invoke_sip_fn_smc32;

#ifdef CONFIG_ARM64
static void __invoke_sip_fn_smc64(u64 function_id, u64 arg0,
				  u64 arg1, u64 arg2, struct arm_smccc_res *res)
{
	asm volatile(
			__asmeq("%0", "x0")
			__asmeq("%1", "x1")
			__asmeq("%2", "x2")
			__asmeq("%3", "x3")
			"smc	#0\n"
		: "+r" (function_id), "+r" (arg0)
		: "r" (arg1), "r" (arg2));

	if (res) {
		res->a0 = function_id;
		res->a1 = arg0;
		res->a2 = arg1;
		res->a3 = arg2;
	}
}

static void (*sip_fn_smc64)(u64, u64, u64, u64, struct arm_smccc_res *) =
							__invoke_sip_fn_smc64;

struct arm_smccc_res rockchip_psci_smc_read64(u64 function_id,
					      u64 arg0,
					      u64 arg1,
					      u64 arg2)
{
	struct arm_smccc_res res;

	sip_fn_smc64(function_id, arg0, arg1, arg2, &res);
	return res;
}

int rockchip_psci_smc_write64(u64 function_id, u64 arg0, u64 arg1, u64 arg2)
{
	struct arm_smccc_res res;

	sip_fn_smc64(function_id, arg0, arg1, arg2, &res);
	return res.a0;
}

struct arm_smccc_res rockchip_secure_reg_read64(u64 addr_phy)
{
	struct arm_smccc_res res;

	sip_fn_smc64(PSCI_SIP_ACCESS_REG64, 0, addr_phy, SEC_REG_RD, &res);
	return res;
}

int rockchip_secure_reg_write64(u64 addr_phy, u64 val)
{
	struct arm_smccc_res res;

	sip_fn_smc64(PSCI_SIP_ACCESS_REG64, val, addr_phy, SEC_REG_WR, &res);
	return res.a0;
}
#endif /*CONFIG_ARM64*/

struct arm_smccc_res rockchip_psci_smc_read(u32 function_id, u32 arg0,
					    u32 arg1, u32 arg2)
{
	struct arm_smccc_res res;

	sip_fn_smc32(function_id, arg0, arg1, arg2, &res);
	return res;
}

u32 rockchip_psci_smc_write(u32 function_id, u32 arg0, u32 arg1, u32 arg2)
{
	struct arm_smccc_res res;

	sip_fn_smc32(function_id, arg0, arg1, arg2, &res);
	return res.a0;
}

u32 rockchip_secure_reg_read(u32 addr_phy)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_ACCESS_REG, 0, addr_phy, SEC_REG_RD, &res);
	return res.a1;
}

int rockchip_secure_reg_write(u32 addr_phy, u32 val)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_ACCESS_REG, val, addr_phy, SEC_REG_WR, &res);
	return res.a0;
}

u32 rockchip_psci_smc_get_tf_ver(void)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_RKTF_VER, 0, 0, 0, &res);
	if (sip_version == SIP_IMPLEMENT_V2)
		return res.a1;
	else
		return res.a0;
}

int rockchip_psci_smc_set_suspend_mode(u32 mode)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_SUSPEND_WR_CTRBITS, mode, 0, 0, &res);
	return res.a0;
}

int psci_set_memory_secure(bool val)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_SMEM_CONFIG, val, 0, 0, &res);
	return res.a0;
}

struct arm_smccc_res
rockchip_request_share_memory(enum share_page_type_t page_type,
			      u32 page_nums)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_SHARE_MEM, page_nums, page_type, 0, &res);
	return res;
}

int rockchip_psci_remotectl_config(u32 func, u32 data)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_REMOTECTL_CFG, func, data, 0, &res);
	return res.a0;
}

/*************************** fiq debug *****************************/
#ifdef CONFIG_ARM64
static u32 fiq_target_cpu;
static u64 ft_fiq_mem_phy;
static void __iomem *ft_fiq_mem_base;
static void (*psci_fiq_debugger_uart_irq_tf)(void *reg_base, u64 sp_el1);

void psci_fiq_debugger_uart_irq_tf_cb(u64 sp_el1, u64 offset)
{
	if (ft_fiq_mem_base)
		psci_fiq_debugger_uart_irq_tf((char *)ft_fiq_mem_base + offset,
					      sp_el1);
	sip_fn_smc64(PSCI_SIP_UARTDBG_CFG64, 0, 0,
		     UARTDBG_CFG_OSHDL_TO_OS, NULL);
}

int psci_fiq_debugger_request_share_memory(void)
{
	struct arm_smccc_res res;

	/* default success */
	if (sip_version == SIP_IMPLEMENT_V1)
		return 0;

	/* request page share memory */
	res = rockchip_request_share_memory(SHARE_PAGE_TYPE_UARTDBG,
					    FIQ_UARTDBG_PAGE_NUMS);
	if (IS_SIP_ERROR(res.a0))
		return res.a0;

	return 0;
}

int psci_fiq_debugger_uart_irq_tf_init(u32 irq_id, void *callback)
{
	struct arm_smccc_res res;

	fiq_target_cpu = 0;
	psci_fiq_debugger_uart_irq_tf = callback;
	sip_fn_smc64(PSCI_SIP_UARTDBG_CFG64, irq_id,
		     (u64)psci_fiq_debugger_uart_irq_tf_cb,
		     UARTDBG_CFG_INIT, &res);
	if (sip_version == SIP_IMPLEMENT_V2) {
		if (IS_SIP_ERROR(res.a0)) {
			pr_err("%s: uartdbg init tf cb fail\n", __func__);
			return res.a0;
		}
		ft_fiq_mem_phy = res.a1;
	} else {
		ft_fiq_mem_phy = res.a0;
	}

	if (!ft_fiq_mem_base) {
		ft_fiq_mem_base = ioremap(ft_fiq_mem_phy,
					  FIQ_UARTDBG_SHARE_MEM_SIZE);
		if (!ft_fiq_mem_base) {
			pr_err("%s: share memory ioremap fail\n", __func__);
			return -ENOMEM;
		}
	}

	return PSCI_SMC_SUCCESS;
}

int psci_fiq_debugger_switch_cpu(u32 cpu)
{
	struct arm_smccc_res res;

	fiq_target_cpu = cpu;
	sip_fn_smc64(PSCI_SIP_UARTDBG_CFG64, cpu_logical_map(cpu),
		     0, UARTDBG_CFG_OSHDL_CPUSW, &res);
	return res.a0;
}

void psci_fiq_debugger_enable_debug(bool val)
{
	u32 enable;

	enable = val ? UARTDBG_CFG_OSHDL_DEBUG_ENABLE :
		       UARTDBG_CFG_OSHDL_DEBUG_DISABLE;
	sip_fn_smc64(PSCI_SIP_UARTDBG_CFG64, 0, 0, enable, NULL);
}

int psci_fiq_debugger_set_print_port(u32 port, u32 baudrate)
{
	struct arm_smccc_res res;

	sip_fn_smc64(PSCI_SIP_UARTDBG_CFG64, port, baudrate,
		     UARTDBG_CFG_SET_PRINT_PORT, &res);
	return res.a0;
}

int psci_fiq_debugger_get_target_cpu(void)
{
	return fiq_target_cpu;
}

void psci_fiq_debugger_enable_fiq(bool enable, uint32_t tgt_cpu)
{
	u32 en;

	en = enable ? UARTDBG_CFG_FIQ_ENABEL : UARTDBG_CFG_FIQ_DISABEL;
	fiq_target_cpu = tgt_cpu;

	sip_fn_smc64(PSCI_SIP_UARTDBG_CFG64, tgt_cpu, 0, en, NULL);
}
#endif

#ifdef CONFIG_ARM
static u32 ft_fiq_mem_phy;
static void __iomem *ft_fiq_mem_base;
static void (*psci_fiq_debugger_uart_irq_tf)(void *reg_base);
static int psci_enable;

int is_psci_enable(void)
{
	return psci_enable;
}

void psci_fiq_debugger_uart_irq_tf_cb(u32 offset)
{
	if (!IS_ERR_OR_NULL(ft_fiq_mem_base))
		psci_fiq_debugger_uart_irq_tf((char *)ft_fiq_mem_base + offset);
	sip_fn_smc32(PSCI_SIP_UARTDBG_CFG, 0, 0, UARTDBG_CFG_OSHDL_TO_OS, NULL);
}

int psci_fiq_debugger_uart_irq_tf_init(u32 irq_id, void *callback)
{
	struct arm_smccc_res res;

	psci_fiq_debugger_uart_irq_tf = callback;
	if (!ft_fiq_mem_base) {
		dma_addr_t ft_fiq_mem_dma;

		ft_fiq_mem_base = dma_alloc_coherent(NULL, PAGE_SIZE,
						     &ft_fiq_mem_dma,
						     GFP_KERNEL);
		if (IS_ERR_OR_NULL(ft_fiq_mem_base)) {
			ft_fiq_mem_base = NULL;
			pr_err("%s: alloc mem failed\n", __func__);
			return PSCI_SMC_INVALID_PARAMS;
		}
		ft_fiq_mem_phy = ft_fiq_mem_dma;
		WARN_ON(ft_fiq_mem_phy != ft_fiq_mem_dma);
	}

	sip_fn_smc32(PSCI_SIP_UARTDBG_CFG, irq_id,
		     (u32)psci_fiq_debugger_uart_irq_tf_cb,
		     UARTDBG_CFG_INIT, &res);

	if (sip_version == SIP_IMPLEMENT_V2 && IS_SIP_ERROR(res.a0)) {
		pr_err("%s: uartdbg init tf cb fail\n", __func__);
		return res.a0;
	} else if (sip_version == SIP_IMPLEMENT_V1 &&
		   (int)res.a0 == PSCI_SMC_FUNC_UNK) {
		pr_err("%s: uartdbg init tf cb fail(smc_unk)\n", __func__);
		return PSCI_SMC_FUNC_UNK;
	}

	sip_fn_smc32(PSCI_SIP_UARTDBG_CFG, ft_fiq_mem_phy, 0,
		     UARTDBG_CFG_SET_SHARE_MEM, &res);
	if (sip_version == SIP_IMPLEMENT_V2 && IS_SIP_ERROR(res.a0)) {
		pr_err("%s: uartdbg set share mem fail\n", __func__);
		return res.a0;
	}

	psci_enable = 1;

	return PSCI_SMC_SUCCESS;
}

int psci_fiq_debugger_switch_cpu(u32 cpu)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_UARTDBG_CFG, cpu, 0,
		     UARTDBG_CFG_OSHDL_CPUSW, &res);
	return res.a0;
}

void psci_fiq_debugger_enable_debug(bool val)
{
	u32 enable;

	enable = val ? UARTDBG_CFG_OSHDL_DEBUG_ENABLE :
		       UARTDBG_CFG_OSHDL_DEBUG_DISABLE;
	sip_fn_smc32(PSCI_SIP_UARTDBG_CFG, 0, 0, enable, NULL);
}

int psci_fiq_debugger_set_print_port(u32 port, u32 baudrate)
{
	struct arm_smccc_res res;

	sip_fn_smc32(PSCI_SIP_UARTDBG_CFG, port, baudrate,
		     UARTDBG_CFG_SET_PRINT_PORT, &res);
	return res.a0;
}
#endif

static __init int sip_implement_version_init(void)
{
#if (defined(CONFIG_ARM_PSCI) && defined(CONFIG_SMP)) || defined(CONFIG_ARM64)
	struct arm_smccc_res res;

#if (defined(CONFIG_ARM_PSCI) && defined(CONFIG_SMP))
	if (!psci_smp_available())
		return 0;
#endif
	sip_fn_smc32(PSCI_SIP_IMPLEMENT_CALL_VER, 0, 0, 0, &res);
	if (res.a0)
		sip_version = SIP_IMPLEMENT_V1;
	else
		sip_version = res.a1;

	pr_info("rockchip sip version v%d\n", sip_version);
#endif

	return 0;
}
arch_initcall(sip_implement_version_init);

