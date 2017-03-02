#ifndef __ROCKCHIP_PSCI_H
#define __ROCKCHIP_PSCI_H

struct arm_smccc_res {
	unsigned long a0;
	unsigned long a1;
	unsigned long a2;
	unsigned long a3;
};

#define SEC_REG_RD (0x0)
#define SEC_REG_WR (0x1)

/*
 * trust firmware verison
 */
#define RKTF_VER_MAJOR(ver)		(((ver) >> 16) & 0xffff)
#define RKTF_VER_MINOR(ver)		((ver) & 0xffff)

/*
 * pcsi smc funciton id
 */
#define PSCI_SIP_RKTF_VER		(0x82000001)
#define PSCI_SIP_ACCESS_REG		(0x82000002)
#define PSCI_SIP_ACCESS_REG64		(0xc2000002)
#define PSCI_SIP_SUSPEND_WR_CTRBITS	(0x82000003)
#define PSCI_SIP_PENDING_CPUS		(0x82000004)
#define PSCI_SIP_UARTDBG_CFG		(0x82000005)
#define PSCI_SIP_UARTDBG_CFG64		(0xc2000005)
#define PSCI_SIP_EL3FIQ_CFG		(0x82000006)
#define PSCI_SIP_ACCESS_CHIP_STATE64	(0xc2000006)
#define PSCI_SIP_SMEM_CONFIG		(0x82000007)
#define PSCI_SIP_ACCESS_CHIP_EXTRA_STATE64 (0xc2000007)
#define PSCI_SIP_DRAM_FREQ_CONFIG	(0x82000008)
#define PSCI_SIP_SHARE_MEM		(0x82000009)
#define PSCI_SIP_IMPLEMENT_CALL_VER	(0x8200000a)
#define PSCI_SIP_REMOTECTL_CFG		(0x8200000b)

/*
 * pcsi smc funciton err code
 */
#define PSCI_SMC_SUCCESS		0
#define PSCI_SMC_FUNC_UNK		-1
#define PSCI_SMC_NOT_SUPPORTED		-2
#define PSCI_SMC_INVALID_PARAMS		-3
#define PSCI_SMC_INVALID_ADDRESS	-4
#define PSCI_SMC_DENIED			-5

#define SIP_IMPLEMENT_V1		(1)
#define SIP_IMPLEMENT_V2		(2)

/*
 * define PSCI_SIP_UARTDBG_CFG call type
 */
#define UARTDBG_CFG_INIT		0xf0
#define UARTDBG_CFG_OSHDL_TO_OS		0xf1
#define UARTDBG_CFG_OSHDL_CPUSW		0xf3
#define UARTDBG_CFG_OSHDL_DEBUG_ENABLE	0xf4
#define UARTDBG_CFG_OSHDL_DEBUG_DISABLE	0xf5
#define UARTDBG_CFG_SET_SHARE_MEM	0xf6
#define UARTDBG_CFG_SET_PRINT_PORT	0xf7
#define UARTDBG_CFG_FIQ_ENABEL		0xf8
#define UARTDBG_CFG_FIQ_DISABEL		0xf9

/*
 * define PSCI_SIP_REMOTECTL_CFG call type
 */
#define	REMOTECTL_SET_IRQ		0xf0
#define REMOTECTL_SET_PWM_CH		0xf1
#define REMOTECTL_SET_PWRKEY		0xf2
#define REMOTECTL_GET_WAKEUP_STATE	0xf3
#define REMOTECTL_ENABLE		0xf4
/* wakeup state */
#define REMOTECTL_PWRKEY_WAKEUP		0xdeadbeaf

/* Share mem page types */
enum share_page_type_t {
	SHARE_PAGE_TYPE_INVALID = 0,
	SHARE_PAGE_TYPE_UARTDBG,
	SHARE_PAGE_TYPE_MAX,
};

#define FIQ_UARTDBG_PAGE_NUMS		2
#define FIQ_UARTDBG_SHARE_MEM_SIZE	((FIQ_UARTDBG_PAGE_NUMS) * 4096)

#define IS_SIP_ERROR(x)			(!!(x))

/*
 * rockchip psci function call interface
 */
#if defined(CONFIG_ARM_PSCI) || defined(CONFIG_ARM64)
struct arm_smccc_res rockchip_psci_smc_read(u32 function_id, u32 arg0,
					    u32 arg1, u32 arg2);
u32 rockchip_psci_smc_write(u32 function_id, u32 arg0, u32 arg1, u32 arg2);

u32 rockchip_psci_smc_get_tf_ver(void);
int rockchip_psci_smc_set_suspend_mode(u32 mode);
u32 rockchip_secure_reg_read(u32 addr_phy);
int rockchip_secure_reg_write(u32 addr_phy, u32 val);
struct arm_smccc_res
rockchip_request_share_memory(enum share_page_type_t page_type,
			      u32 page_nums);
int rockchip_psci_remotectl_config(u32 func, u32 data);

#ifdef CONFIG_ARM64
int rockchip_psci_smc_write64(u64 function_id, u64 arg0, u64 arg1, u64 arg2);
struct arm_smccc_res rockchip_psci_smc_read64(u64 function_id, u64 arg0,
					      u64 arg1, u64 arg2);

struct arm_smccc_res rockchip_secure_reg_read64(u64 addr_phy);
int rockchip_secure_reg_write64(u64 addr_phy, u64 val);

void psci_fiq_debugger_uart_irq_tf_cb(u64 sp_el1, u64 offset);
int psci_fiq_debugger_request_share_memory(void);
int psci_fiq_debugger_get_target_cpu(void);
void psci_fiq_debugger_enable_fiq(bool enable, uint32_t tgt_cpu);
#endif

int psci_fiq_debugger_switch_cpu(u32 cpu);
int psci_fiq_debugger_uart_irq_tf_init(u32 irq_id, void *callback);
void psci_fiq_debugger_enable_debug(bool val);
int psci_fiq_debugger_set_print_port(u32 port, u32 baudrate);
int psci_set_memory_secure(bool val);
#else
static inline struct arm_smccc_res
	rockchip_psci_smc_read(u32 function_id, u32 arg0, u32 arg1, u32 arg2)
{
	struct arm_smccc_res res;

	memset(&res, 0, sizeof(struct arm_smccc_res));
	return res;
}

static inline u32 rockchip_psci_smc_write(u32 function_id, u32 arg0,
					  u32 arg1, u32 arg2)
{
	return 0;
}

static inline u32 rockchip_psci_smc_get_tf_ver(void) { return 0; }
static inline int
rockchip_psci_remotectl_config(u32 func, u32 data) { return 0; }
static inline int rockchip_psci_smc_set_suspend_mode(u32 mode) { return 0; }
static inline u32 rockchip_secure_reg_read(u32 addr_phy) { return 0; }
static inline int rockchip_secure_reg_write(u32 addr_phy, u32 val) { return 0; }

static inline int psci_fiq_debugger_switch_cpu(u32 cpu) { return 0; }
static inline int
psci_fiq_debugger_uart_irq_tf_init(u32 irq_id, void *callback) { return 0; }
static inline void psci_fiq_debugger_enable_debug(bool val) { }
static inline u32
psci_fiq_debugger_set_print_port(u32 port, u32 baudrate) { return 0; }

static inline u32 psci_set_memory_secure(bool val)
{
	return 0;
}
#endif

#ifdef CONFIG_ARM_PSCI
struct sm_nsec_ctx {
	u32 usr_sp;
	u32 usr_lr;
	u32 irq_spsr;
	u32 irq_sp;
	u32 irq_lr;
	u32 svc_spsr;
	u32 svc_sp;
	u32 svc_lr;
	u32 abt_spsr;
	u32 abt_sp;
	u32 abt_lr;
	u32 und_spsr;
	u32 und_sp;
	u32 und_lr;
	u32 mon_lr;
	u32 mon_spsr;
	u32 r4;
	u32 r5;
	u32 r6;
	u32 r7;
	u32 r8;
	u32 r9;
	u32 r10;
	u32 r11;
	u32 r12;
	u32 r0;
	u32 r1;
	u32 r2;
	u32 r3;
};

int is_psci_enable(void);
#endif

#endif /* __ROCKCHIP_PSCI_H */
