/*
 **************************************************************************
 * Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all copies.
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 **************************************************************************
 */

/*
 * nss_init.c
 *	NSS init APIs
 *
 */
#include "nss_core.h"
#if (NSS_PM_SUPPORT == 1)
#include "nss_pm.h"
#endif
#include "nss_tx_rx_common.h"

#include <nss_hal.h>
#include <nss_clocks.h>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reset.h>

#if (NSS_DT_SUPPORT != 1)
#include <mach/msm_nss.h>
#endif

#include <linux/sysctl.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/firmware.h>

/*
 * Macros
 */
#define MIN_IMG_SIZE 64*1024
#define NETAP0_IMAGE "qca-nss0.bin"
#define NETAP1_IMAGE "qca-nss1.bin"

/*
 * Global declarations
 */
int nss_ctl_redirect __read_mostly = 0;
int nss_ctl_debug __read_mostly = 0;

/*
 * PM client handle
 */
#if (NSS_PM_SUPPORT == 1)
static void *pm_client;
#endif

/*
 * Handler to send NSS messages
 */
void *nss_freq_change_context;
struct clk *nss_core0_clk;

/*
 * Top level nss context structure
 */
struct nss_top_instance nss_top_main;
struct nss_cmd_buffer nss_cmd_buf;
struct nss_runtime_sampling nss_runtime_samples;
struct workqueue_struct *nss_wq;

/*
 * Work Queue to handle messages to Kernel
 */
nss_work_t *nss_work;

/*
 * File local/Static variables/functions
 */

static const struct net_device_ops nss_netdev_ops;
static const struct ethtool_ops nss_ethtool_ops;

/*
 * nss_dummy_netdev_setup()
 *	Dummy setup for net_device handler
 */
static void nss_dummy_netdev_setup(struct net_device *ndev)
{

}

/*
 * nss_handle_irq()
 *	HLOS interrupt handler for nss interrupts
 */
static irqreturn_t nss_handle_irq (int irq, void *ctx)
{
	struct int_ctx_instance *int_ctx = (struct int_ctx_instance *) ctx;
	struct nss_ctx_instance *nss_ctx = int_ctx->nss_ctx;

	/*
	 * Mask interrupt until our bottom half re-enables it
	 */
	nss_hal_disable_interrupt(nss_ctx->nmap, int_ctx->irq,
			int_ctx->shift_factor, NSS_HAL_SUPPORTED_INTERRUPTS);

	/*
	 * Schedule tasklet to process interrupt cause
	 */
	napi_schedule(&int_ctx->napi);
	return IRQ_HANDLED;
}

/*
 * nss_drv_of_get_pdata()
 *	Retrieve platform data from device node.
 */
static struct nss_platform_data *nss_drv_of_get_pdata(struct device_node *np,
						      struct platform_device *pdev)
{
	struct nss_platform_data *npd = NULL;
	struct nss_ctx_instance *nss_ctx = NULL;
	struct nss_top_instance *nss_top = &nss_top_main;
	uint32_t val;
	struct resource res_nphys, res_vphys;
	int32_t i;

	npd = devm_kzalloc(&pdev->dev, sizeof(struct nss_platform_data), GFP_KERNEL);
	if (!npd) {
		return NULL;
	}

	if (of_property_read_u32(np, "qcom,id", &npd->id)
	    || of_property_read_u32(np, "qcom,rst_addr", &npd->rst_addr)
	    || of_property_read_u32(np, "qcom,load_addr", &npd->load_addr)
	    || of_property_read_u32(np, "qcom,turbo_frequency", &npd->turbo_frequency)
	    || of_property_read_u32(np, "qcom,gmac0_enabled", &npd->gmac_enabled[0])
	    || of_property_read_u32(np, "qcom,gmac1_enabled", &npd->gmac_enabled[1])
	    || of_property_read_u32(np, "qcom,gmac2_enabled", &npd->gmac_enabled[2])
	    || of_property_read_u32(np, "qcom,gmac3_enabled", &npd->gmac_enabled[3])
	    || of_property_read_u32(np, "qcom,num_irq", &npd->num_irq)) {
		pr_err("%s: error reading critical device node properties\n", np->name);
		goto out;
	}

	nss_ctx = &nss_top->nss[npd->id];
	nss_ctx->id = npd->id;

	if (of_address_to_resource(np, 0, &res_nphys) != 0) {
		nss_info("%p: nss%d: of_address_to_resource() fail for nphys \n", nss_ctx, nss_ctx->id);
		goto out;
	}

	if (of_address_to_resource(np, 1, &res_vphys) != 0) {
		nss_info("%p: nss%d: of_address_to_resource() fail for vphys \n", nss_ctx, nss_ctx->id);
		goto out;
	}

	/*
	 * Save physical addresses
	 */
	npd->nphys = res_nphys.start;
	npd->vphys = res_vphys.start;

	npd->nmap = (uint32_t)ioremap_nocache(npd->nphys, resource_size(&res_nphys));
	if (!npd->nmap) {
		nss_info("%p: nss%d: ioremap() fail for nphys \n", nss_ctx, nss_ctx->id);
		goto out;
	}

	npd->vmap = (uint32_t)ioremap_nocache(npd->vphys, resource_size(&res_vphys));
	if (!npd->vmap) {
		nss_info("%p: nss%d: ioremap() fail for vphys \n", nss_ctx, nss_ctx->id);
		goto out;
	}

	/*
	 * Clear TCM memory used by this core
	 */
	for (i = 0; i < resource_size(&res_vphys) ; i += 4) {
		nss_write_32((uint32_t)npd->vmap, i, 0);
	}

	/*
	 * Get IRQ numbers
	 */
	for (val = 0 ; val < npd->num_irq ; val++) {
		npd->irq[val] = irq_of_parse_and_map(np, val);
		if (!npd->irq[val]) {
			nss_info("%p: nss%d: irq_of_parse_and_map() fail for irq %d\n",
				 nss_ctx, nss_ctx->id, val);
			goto out;
		}
	}

	if (of_property_read_u32(np, "qcom,ipv4_enabled", &npd->ipv4_enabled)
	    || of_property_read_u32(np, "qcom,ipv6_enabled", &npd->ipv6_enabled)
	    || of_property_read_u32(np, "qcom,l2switch_enabled", &npd->l2switch_enabled)
	    || of_property_read_u32(np, "qcom,crypto_enabled", &npd->crypto_enabled)
	    || of_property_read_u32(np, "qcom,ipsec_enabled", &npd->ipsec_enabled)
	    || of_property_read_u32(np, "qcom,wlan_enabled", &npd->wlan_enabled)
	    || of_property_read_u32(np, "qcom,tun6rd_enabled", &npd->tun6rd_enabled)
	    || of_property_read_u32(np, "qcom,tunipip6_enabled", &npd->tunipip6_enabled)
	    || of_property_read_u32(np, "qcom,shaping_enabled", &npd->shaping_enabled)) {
		pr_warn("%s: error reading non-critical device node properties\n", np->name);
	}

	return npd;

out:
	if (npd->nmap) {
		iounmap((void *)npd->nmap);
	}

	if (npd->vmap) {
		iounmap((void *)npd->vmap);
	}

	devm_kfree(&pdev->dev, npd);

	return NULL;
}

/*
 * nss_probe()
 *	HLOS device probe callback
 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3,7,0))
static int __devinit nss_probe(struct platform_device *nss_dev)
#else
static int nss_probe(struct platform_device *nss_dev)
#endif
{
	struct nss_top_instance *nss_top = &nss_top_main;
	struct nss_ctx_instance *nss_ctx = NULL;
	struct nss_platform_data *npd = NULL;
	struct netdev_priv_instance *ndev_priv;
	struct reset_control *rstctl = NULL;
	int i, err = 0;

	const struct firmware *nss_fw = NULL;
	int rc = -ENODEV;
	void __iomem *load_mem;

	struct device_node *np = NULL;

	if (nss_top_main.nss_hal_common_init_done == false) {
		/*
		 * Perform clock init common to all NSS cores
		 */
		struct clk *nss_tcm_src = NULL;
		struct clk *nss_tcm_clk = NULL;

		/*
		 * Attach debug interface to TLMM
		 */
		nss_write_32((uint32_t)nss_top_main.nss_fpb_base, NSS_REGS_FPB_CSR_CFG_OFFSET, 0x360);

		/*
		 * NSS TCM CLOCK
		 */
		nss_tcm_src = clk_get(&nss_dev->dev, NSS_TCM_SRC_CLK);
		if (IS_ERR(nss_tcm_src)) {
			pr_err("nss-driver: cannot get clock: " NSS_TCM_SRC_CLK);
			return -EFAULT;
		}

		clk_set_rate(nss_tcm_src, NSSTCM_FREQ);
		clk_prepare(nss_tcm_src);
		clk_enable(nss_tcm_src);

		nss_tcm_clk = clk_get(&nss_dev->dev, NSS_TCM_CLK);
		if (IS_ERR(nss_tcm_clk)) {
			pr_err("nss-driver: cannot get clock: " NSS_TCM_CLK);
			return -EFAULT;
		}

		clk_prepare(nss_tcm_clk);
		clk_enable(nss_tcm_clk);

		nss_top_main.nss_hal_common_init_done = true;
		nss_info("nss_hal_common_reset Done.\n");
	}

	if (nss_dev->dev.of_node) {
		/*
		 * Device Tree based init
		 */

		np = of_node_get(nss_dev->dev.of_node);
		npd = nss_drv_of_get_pdata(np, nss_dev);

		of_node_put(np);

		if (!npd) {
			return -EFAULT;
		}

		nss_ctx = &nss_top->nss[npd->id];
		nss_ctx->id = npd->id;
		nss_dev->id = nss_ctx->id;

	} else {
		/*
		 * Platform Device based init
		 */

		npd = (struct nss_platform_data *) nss_dev->dev.platform_data;
		nss_ctx = &nss_top->nss[nss_dev->id];
		nss_ctx->id = nss_dev->id;
	}

	nss_ctx->nss_top = nss_top;

	nss_info("%p: NSS_DEV_ID %s \n", nss_ctx, dev_name(&nss_dev->dev));

	/*
	 * F/W load from NSS Driver
	 */
	if (nss_ctx->id == 0) {
		rc = request_firmware(&nss_fw, NETAP0_IMAGE, &(nss_dev->dev));
	} else if (nss_ctx->id == 1) {
		rc = request_firmware(&nss_fw, NETAP1_IMAGE, &(nss_dev->dev));
	} else {
		nss_warning("%p: Invalid nss dev: %d \n", nss_ctx, nss_ctx->id);
	}

	/*
	 *  Check if the file read is successful
	 */
	if (rc) {
		nss_warning("%p: request_firmware failed with err code: %d", nss_ctx, rc);
		err = rc;
		goto err_init_0;
	}

	if (nss_fw->size < MIN_IMG_SIZE) {
		nss_warning("%p: nss firmware is truncated, size:%d", nss_ctx, nss_fw->size);
	}

	load_mem = ioremap_nocache(npd->load_addr, nss_fw->size);
	if (load_mem == NULL) {
		nss_warning("%p: ioremap_nocache failed: %x", nss_ctx, npd->load_addr);
		release_firmware(nss_fw);
		goto err_init_0;
	}

	printk("nss_driver - fw of size %u  bytes copied to load addr: %x\n", nss_fw->size, npd->load_addr);
	memcpy_toio(load_mem, nss_fw->data, nss_fw->size);
	release_firmware(nss_fw);
	iounmap(load_mem);

	/*
	 * Both NSS cores controlled by same regulator, Hook only Once
	 */
	if (!nss_ctx->id) {
		nss_core0_clk = clk_get(&nss_dev->dev, "nss_core_clk");
		if (IS_ERR(nss_core0_clk)) {

			err = PTR_ERR(nss_core0_clk);
			nss_info("%p: Regulator %s get failed, err=%d\n", nss_ctx, dev_name(&nss_dev->dev), err);
			return err;

		}
		clk_set_rate(nss_core0_clk, NSS_FREQ_550);
		clk_prepare(nss_core0_clk);
		clk_enable(nss_core0_clk);

#if (NSS_PM_SUPPORT == 1)
		/*
		 * Check if turbo is supported
		 */
		if (npd->turbo_frequency) {
			/*
			 * Turbo is supported
			 */
			printk("nss_driver - Turbo Support %d\n", npd->turbo_frequency);
			nss_runtime_samples.freq_scale_sup_max = NSS_MAX_CPU_SCALES;
			nss_pm_set_turbo();
		} else {
			printk("nss_driver - Turbo No Support %d\n", npd->turbo_frequency);
			nss_runtime_samples.freq_scale_sup_max = NSS_MAX_CPU_SCALES - 1;
		}
#else
		printk("nss_driver - Turbo Not Supported\n");
#endif
	}

	/*
	 * Get load address of NSS firmware
	 */
	nss_info("%p: Setting NSS%d Firmware load address to %x\n", nss_ctx, nss_ctx->id, npd->load_addr);
	nss_top->nss[nss_ctx->id].load = npd->load_addr;

	/*
	 * Get virtual and physical memory addresses for nss logical/hardware address maps
	 */

	/*
	 * Virtual address of CSM space
	 */
	nss_ctx->nmap = npd->nmap;
	nss_assert(nss_ctx->nmap);

	/*
	 * Physical address of CSM space
	 */
	nss_ctx->nphys = npd->nphys;
	nss_assert(nss_ctx->nphys);

	/*
	 * Virtual address of logical registers space
	 */
	nss_ctx->vmap = npd->vmap;
	nss_assert(nss_ctx->vmap);

	/*
	 * Physical address of logical registers space
	 */
	nss_ctx->vphys = npd->vphys;
	nss_assert(nss_ctx->vphys);
	nss_info("%d:ctx=%p, vphys=%x, vmap=%x, nphys=%x, nmap=%x",
			nss_ctx->id, nss_ctx, nss_ctx->vphys, nss_ctx->vmap, nss_ctx->nphys, nss_ctx->nmap);

	/*
	 * Register netdevice handlers
	 */
	nss_ctx->int_ctx[0].ndev = alloc_netdev(sizeof(struct netdev_priv_instance),
					"qca-nss-dev%d", nss_dummy_netdev_setup);
	if (nss_ctx->int_ctx[0].ndev == NULL) {
		nss_warning("%p: Could not allocate net_device #0", nss_ctx);
		err = -ENOMEM;
		goto err_init_0;
	}

	nss_ctx->int_ctx[0].ndev->netdev_ops = &nss_netdev_ops;
	nss_ctx->int_ctx[0].ndev->ethtool_ops = &nss_ethtool_ops;
	err = register_netdev(nss_ctx->int_ctx[0].ndev);
	if (err) {
		nss_warning("%p: Could not register net_device #0", nss_ctx);
		goto err_init_1;
	}

	/*
	 * request for IRQs
	 *
	 * WARNING: CPU affinities should be set using OS supported methods
	 */
	nss_ctx->int_ctx[0].nss_ctx = nss_ctx;
	nss_ctx->int_ctx[0].shift_factor = 0;
	nss_ctx->int_ctx[0].irq = npd->irq[0];
	err = request_irq(npd->irq[0], nss_handle_irq, IRQF_DISABLED, "nss", &nss_ctx->int_ctx[0]);
	if (err) {
		nss_warning("%d: IRQ0 request failed", nss_dev->id);
		goto err_init_2;
	}

	/*
	 * Register NAPI for NSS core interrupt #0
	 */
	ndev_priv = netdev_priv(nss_ctx->int_ctx[0].ndev);
	ndev_priv->int_ctx = &nss_ctx->int_ctx[0];
	netif_napi_add(nss_ctx->int_ctx[0].ndev, &nss_ctx->int_ctx[0].napi, nss_core_handle_napi, 64);
	napi_enable(&nss_ctx->int_ctx[0].napi);
	nss_ctx->int_ctx[0].napi_active = true;

	/*
	 * Check if second interrupt is supported on this nss core
	 */
	if (npd->num_irq > 1) {
		nss_info("%d: This NSS core supports two interrupts", nss_dev->id);

		/*
		 * Register netdevice handlers
		 */
		nss_ctx->int_ctx[1].ndev = alloc_netdev(sizeof(struct netdev_priv_instance),
						"qca-nss-dev%d", nss_dummy_netdev_setup);
		if (nss_ctx->int_ctx[1].ndev == NULL) {
			nss_warning("%p: Could not allocate net_device #1", nss_ctx);
			err = -ENOMEM;
			goto err_init_3;
		}

		nss_ctx->int_ctx[1].ndev->netdev_ops = &nss_netdev_ops;
		nss_ctx->int_ctx[1].ndev->ethtool_ops = &nss_ethtool_ops;
		err = register_netdev(nss_ctx->int_ctx[1].ndev);
		if (err) {
			nss_warning("%p: Could not register net_device #1", nss_ctx);
			goto err_init_4;
		}

		nss_ctx->int_ctx[1].nss_ctx = nss_ctx;
		nss_ctx->int_ctx[1].shift_factor = 15;
		nss_ctx->int_ctx[1].irq = npd->irq[1];
		err = request_irq(npd->irq[1], nss_handle_irq, IRQF_DISABLED, "nss", &nss_ctx->int_ctx[1]);
		if (err) {
			nss_warning("%d: IRQ1 request failed for nss", nss_dev->id);
			goto err_init_5;
		}

		/*
		 * Register NAPI for NSS core interrupt #1
		 */
		ndev_priv = netdev_priv(nss_ctx->int_ctx[1].ndev);
		ndev_priv->int_ctx = &nss_ctx->int_ctx[1];
		netif_napi_add(nss_ctx->int_ctx[1].ndev, &nss_ctx->int_ctx[1].napi, nss_core_handle_napi, 64);
		napi_enable(&nss_ctx->int_ctx[1].napi);
		nss_ctx->int_ctx[1].napi_active = true;
	}

	spin_lock_bh(&(nss_top->lock));

	/*
	 * Check functionalities are supported by this NSS core
	 */
	if (npd->shaping_enabled == NSS_FEATURE_ENABLED) {
		nss_top->shaping_handler_id = nss_dev->id;
		printk(KERN_INFO "%p: NSS Shaping is enabled, handler id: %u\n", __func__, nss_top->shaping_handler_id);
	}

	if (npd->ipv4_enabled == NSS_FEATURE_ENABLED) {
		nss_top->ipv4_handler_id = nss_dev->id;
		nss_ipv4_register_handler();
		nss_pppoe_register_handler();
		nss_eth_rx_register_handler();
		nss_n2h_register_handler();
		nss_virt_if_register_handler();
		nss_lag_register_handler();
		for (i = 0; i < NSS_MAX_VIRTUAL_INTERFACES; i++) {
			nss_top->virt_if_handler_id[i] = nss_dev->id;
		}
	}

	if (npd->ipv6_enabled == NSS_FEATURE_ENABLED) {
		nss_top->ipv6_handler_id = nss_dev->id;
		nss_ipv6_register_handler();
	}

	if (npd->crypto_enabled == NSS_FEATURE_ENABLED) {
		nss_top->crypto_handler_id = nss_dev->id;
		nss_crypto_register_handler();
	}

	if (npd->ipsec_enabled == NSS_FEATURE_ENABLED) {
		nss_top->ipsec_handler_id = nss_dev->id;
		nss_ipsec_register_handler();
	}

	if (npd->wlan_enabled == NSS_FEATURE_ENABLED) {
		nss_top->wlan_handler_id = nss_dev->id;
	}

	if (npd->tun6rd_enabled == NSS_FEATURE_ENABLED) {
		nss_top->tun6rd_handler_id = nss_dev->id;
		nss_tun6rd_register_handler();
	}

	if (npd->tunipip6_enabled == NSS_FEATURE_ENABLED) {
		nss_top->tunipip6_handler_id = nss_dev->id;
		nss_tunipip6_register_handler();
	}

	if (npd->gmac_enabled[0] == NSS_FEATURE_ENABLED) {
		nss_top->phys_if_handler_id[0] = nss_dev->id;
		nss_phys_if_register_handler(0);
	}

	if (npd->gmac_enabled[1] == NSS_FEATURE_ENABLED) {
		nss_top->phys_if_handler_id[1] = nss_dev->id;
		nss_phys_if_register_handler(1);
	}

	if (npd->gmac_enabled[2] == NSS_FEATURE_ENABLED) {
		nss_top->phys_if_handler_id[2] = nss_dev->id;
		nss_phys_if_register_handler(2);
	}

	if (npd->gmac_enabled[3] == NSS_FEATURE_ENABLED) {
		nss_top->phys_if_handler_id[3] = nss_dev->id;
		nss_phys_if_register_handler(3);
	}

#if (NSS_PM_SUPPORT == 1)
	nss_core_freq_register_handler();
#endif

	nss_top->frequency_handler_id = nss_dev->id;

	spin_unlock_bh(&(nss_top->lock));

	/*
	 * Initialize decongestion callbacks to NULL
	 */
	for (i = 0; i< NSS_MAX_CLIENTS; i++) {
		nss_ctx->queue_decongestion_callback[i] = 0;
		nss_ctx->queue_decongestion_ctx[i] = 0;
	}

	spin_lock_init(&(nss_ctx->decongest_cb_lock));
	nss_ctx->magic = NSS_CTX_MAGIC;

	nss_info("%p: Reseting NSS core %d now", nss_ctx, nss_ctx->id);

	/*
	 * Enable clocks and bring NSS core out of reset
	 */

	/*
	 * Remove UBI32 reset clamp
	 */
	rstctl = devm_reset_control_get(&nss_dev->dev, NSS_CORE_CLK_RST_CLAMP);
	if (IS_ERR(rstctl)) {
		nss_info("%p: Deassert UBI32 reset clamp failed", nss_ctx, nss_ctx->id);
		err = -EFAULT;
		goto err_init_5;
	}
	reset_control_deassert(rstctl);
	mdelay(1);
	reset_control_put(rstctl);

	/*
	 * Remove UBI32 core clamp
	 */
	rstctl = devm_reset_control_get(&nss_dev->dev, NSS_CORE_CLAMP);
	if (IS_ERR(rstctl)) {
		nss_info("%p: Deassert UBI32 core clamp failed", nss_ctx, nss_ctx->id);
		err = -EFAULT;
		goto err_init_5;
	}
	reset_control_deassert(rstctl);
	mdelay(1);
	reset_control_put(rstctl);

	/*
	 * Remove UBI32 AHB reset
	 */
	rstctl = devm_reset_control_get(&nss_dev->dev, NSS_CORE_AHB_RESET);
	if (IS_ERR(rstctl)) {
		nss_info("%p: Deassert AHB reset failed", nss_ctx, nss_ctx->id);
		err = -EFAULT;
		goto err_init_5;
	}
	reset_control_deassert(rstctl);
	mdelay(1);
	reset_control_put(rstctl);

	/*
	 * Remove UBI32 AXI reset
	 */
	rstctl = devm_reset_control_get(&nss_dev->dev, NSS_CORE_AXI_RESET);
	if (IS_ERR(rstctl)) {
		nss_info("%p: Deassert AXI reset failed", nss_ctx, nss_ctx->id);
		err = -EFAULT;
		goto err_init_5;
	}
	reset_control_deassert(rstctl);
	mdelay(1);
	reset_control_put(rstctl);

	nss_hal_core_reset(nss_ctx->nmap, nss_ctx->load);

	/*
	 * Enable interrupts for NSS core
	 */
	nss_hal_enable_interrupt(nss_ctx->nmap, nss_ctx->int_ctx[0].irq,
					nss_ctx->int_ctx[0].shift_factor, NSS_HAL_SUPPORTED_INTERRUPTS);

	if (npd->num_irq > 1) {
		nss_hal_enable_interrupt(nss_ctx->nmap, nss_ctx->int_ctx[1].irq,
					nss_ctx->int_ctx[1].shift_factor, NSS_HAL_SUPPORTED_INTERRUPTS);
	}

	/*
	 * Initialize max buffer size for NSS core
	 */
	nss_ctx->max_buf_size = NSS_NBUF_PAYLOAD_SIZE;
	nss_info("%p: All resources initialized and nss core%d has been brought out of reset", nss_ctx, nss_dev->id);
	goto err_init_0;

err_init_5:
	unregister_netdev(nss_ctx->int_ctx[1].ndev);
err_init_4:
	free_netdev(nss_ctx->int_ctx[1].ndev);
err_init_3:
	free_irq(npd->irq[0], &nss_ctx->int_ctx[0]);
err_init_2:
	unregister_netdev(nss_ctx->int_ctx[0].ndev);
err_init_1:
	free_netdev(nss_ctx->int_ctx[0].ndev);

	if (nss_dev->dev.of_node) {
		if (npd->nmap) {
			iounmap((void *)npd->nmap);
		}

		if (npd->vmap) {
			iounmap((void *)npd->vmap);
		}
	}

err_init_0:

	if (nss_dev->dev.of_node) {
		devm_kfree(&nss_dev->dev, npd);
	}

	return err;
}

/*
 * nss_remove()
 *	HLOS device remove callback
 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3,7,0))
static int __devexit nss_remove(struct platform_device *nss_dev)
#else
static int nss_remove(struct platform_device *nss_dev)
#endif
{
	struct nss_top_instance *nss_top = &nss_top_main;
	struct nss_ctx_instance *nss_ctx = &nss_top->nss[nss_dev->id];

	/*
	 * Clean-up debugfs
	 */
	nss_stats_clean();

	/*
	 * Disable interrupts and bottom halves in HLOS
	 * Disable interrupts from NSS to HLOS
	 */
	nss_hal_disable_interrupt(nss_ctx->nmap, nss_ctx->int_ctx[0].irq,
					nss_ctx->int_ctx[0].shift_factor, NSS_HAL_SUPPORTED_INTERRUPTS);

	free_irq(nss_ctx->int_ctx[0].irq, &nss_ctx->int_ctx[0]);
	unregister_netdev(nss_ctx->int_ctx[0].ndev);
	free_netdev(nss_ctx->int_ctx[0].ndev);

	/*
	 * Check if second interrupt is supported
	 * If so then clear resources for second interrupt as well
	 */
	if (nss_ctx->int_ctx[1].irq) {
		nss_hal_disable_interrupt(nss_ctx->nmap, nss_ctx->int_ctx[1].irq,
					nss_ctx->int_ctx[1].shift_factor, NSS_HAL_SUPPORTED_INTERRUPTS);
		free_irq(nss_ctx->int_ctx[1].irq, &nss_ctx->int_ctx[1]);
		unregister_netdev(nss_ctx->int_ctx[1].ndev);
		free_netdev(nss_ctx->int_ctx[1].ndev);
	}

	if (nss_dev->dev.of_node) {
		if (nss_ctx->nmap) {
			iounmap((void *)nss_ctx->nmap);
			nss_ctx->nmap = 0;
		}

		if (nss_ctx->vmap) {
			iounmap((void *)nss_ctx->vmap);
			nss_ctx->vmap = 0;
		}
	}

	nss_info("%p: All resources freed for nss core%d", nss_ctx, nss_dev->id);
	return 0;
}


static struct of_device_id nss_dt_ids[] = {
	{ .compatible =  "qcom,nss0" },
	{ .compatible =  "qcom,nss1" },
	{},
};
MODULE_DEVICE_TABLE(of, nss_dt_ids);

/*
 * nss_driver
 *	Platform driver structure for NSS
 */
struct platform_driver nss_driver = {
	.probe	= nss_probe,
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(3,7,0))
	.remove	= __devexit_p(nss_remove),
#else
	.remove	= nss_remove,
#endif
	.driver	= {
		.name	= "qca-nss",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(nss_dt_ids),
	},
};

#if (NSS_PM_SUPPORT == 1)
/*
 * nss_reset_frequency_stats_samples()
 *	Reset all frequency sampling state when auto scaling is turned off.
 */
static void nss_reset_frequency_stats_samples (void)
{
	nss_runtime_samples.buffer_index = 0;
	nss_runtime_samples.sum = 0;
	nss_runtime_samples.average = 0;
	nss_runtime_samples.sample_count = 0;
	nss_runtime_samples.message_rate_limit = 0;
	nss_runtime_samples.freq_scale_rate_limit_up = 0;
	nss_runtime_samples.freq_scale_rate_limit_down = 0;
}

/*
 ***************************************************************************************************
 * nss_wq_function() is used to queue up requests to change NSS frequencies.
 * The function will take care of NSS notices and also control clock.
 * The auto rate algorithmn will queue up requests or the procfs may also queue up these requests.
 ***************************************************************************************************
 */

/*
 * nss_wq_function()
 *	Added to Handle BH requests to kernel
 */
void nss_wq_function (struct work_struct *work)
{
	nss_work_t *my_work = (nss_work_t *)work;

	nss_freq_change(nss_freq_change_context, my_work->frequency, my_work->stats_enable, 0);
	clk_set_rate(nss_core0_clk, my_work->frequency);
	nss_freq_change(nss_freq_change_context, my_work->frequency, my_work->stats_enable, 1);

	if(!pm_client) {
		goto out;
	}

	if (my_work->frequency == NSS_FREQ_733) {
		nss_pm_set_perf_level(pm_client, NSS_PM_PERF_LEVEL_TURBO);
	} else if ((my_work->frequency == NSS_FREQ_275) || (my_work->frequency == NSS_FREQ_550)) {
		nss_pm_set_perf_level(pm_client, NSS_PM_PERF_LEVEL_NOMINAL);
	} else {
		nss_pm_set_perf_level(pm_client, NSS_PM_PERF_LEVEL_IDLE);
	}
out:
	kfree((void *)work);
}

/*
 * nss_current_freq_handler()
 *	Handle Userspace Frequency Change Requests
 */
static int nss_current_freq_handler (ctl_table *ctl, int write, void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	BUG_ON(!nss_wq);

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);

	if (!write) {
		printk("Frequency Set to %d\n", nss_cmd_buf.current_freq);
		return ret;
	}

	/* Turn off Auto Scale */
	nss_cmd_buf.auto_scale = 0;
	nss_runtime_samples.freq_scale_ready = 0;

	/* If support NSS freq is in the table send the new frequency request to NSS or If No Turbo and ask for turbo freq */
	if (((nss_cmd_buf.current_freq != NSS_FREQ_110) && (nss_cmd_buf.current_freq != NSS_FREQ_275) && (nss_cmd_buf.current_freq != NSS_FREQ_550) && (nss_cmd_buf.current_freq != NSS_FREQ_733)) || ((nss_runtime_samples.freq_scale_sup_max != NSS_MAX_CPU_SCALES) && (nss_cmd_buf.current_freq == NSS_FREQ_733))) {
		printk("Frequency not found. Please check Frequency Table\n");
		return ret;
	}

	nss_work = (nss_work_t *)kmalloc(sizeof(nss_work_t), GFP_KERNEL);
	if (!nss_work) {
		nss_info("NSS Freq WQ kmalloc fail");
		return ret;
	}
	INIT_WORK((struct work_struct *)nss_work, nss_wq_function);
	nss_work->frequency = nss_cmd_buf.current_freq;
	nss_work->stats_enable = 0;

	/* Ensure we start with a fresh set of samples later */
	nss_reset_frequency_stats_samples();

	queue_work(nss_wq, (struct work_struct *)nss_work);

	return ret;
}

/*
 * nss_auto_scale_handler()
 *	Enables or Disable Auto Scaling
 */
static int nss_auto_scale_handler (ctl_table *ctl, int write, void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);

	if (!write) {
		return ret;
	}

	if (nss_cmd_buf.auto_scale != 1) {
		/*
		 * Is auto scaling currently enabled? If so, send the command to
		 * disable stats reporting to NSS
		 */
		if (nss_runtime_samples.freq_scale_ready != 0) {
			nss_cmd_buf.current_freq = nss_runtime_samples.freq_scale[nss_runtime_samples.freq_scale_index].frequency;
			nss_work = (nss_work_t *)kmalloc(sizeof(nss_work_t), GFP_KERNEL);
			if (!nss_work) {
				nss_info("NSS Freq WQ kmalloc fail");
				return ret;
			}
			INIT_WORK((struct work_struct *)nss_work, nss_wq_function);
			nss_work->frequency = nss_cmd_buf.current_freq;
			nss_work->stats_enable = 0;
			queue_work(nss_wq, (struct work_struct *)nss_work);
			nss_runtime_samples.freq_scale_ready = 0;

			/*
			 * The current samples would be stale later when scaling is
			 * enabled again, hence reset them
			 */
			nss_reset_frequency_stats_samples();
		}
		return ret;
	}

	/*
	 * Auto Scaling is already being done
	 */
	if (nss_runtime_samples.freq_scale_ready == 1) {
		return ret;
	}

	/*
	 * Setup default values - Middle of Freq Scale Band
	 */
	nss_runtime_samples.freq_scale_index = 1;
	nss_cmd_buf.current_freq = nss_runtime_samples.freq_scale[nss_runtime_samples.freq_scale_index].frequency;

	nss_work = (nss_work_t *)kmalloc(sizeof(nss_work_t), GFP_KERNEL);
	if (!nss_work) {
		nss_info("NSS Freq WQ kmalloc fail");
		return ret;
	}
	INIT_WORK((struct work_struct *)nss_work, nss_wq_function);
	nss_work->frequency = nss_cmd_buf.current_freq;
	nss_work->stats_enable = 1;
	queue_work(nss_wq, (struct work_struct *)nss_work);

	nss_runtime_samples.freq_scale_ready = 1;

	return ret;
}

/*
 * nss_get_freq_table_handler()
 *	Display Support Freq and Ex how to Change.
 */
static int nss_get_freq_table_handler(ctl_table *ctl, int write, void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);

	if (nss_runtime_samples.freq_scale_sup_max != NSS_MAX_CPU_SCALES) {
		printk("Frequency Supported - 110Mhz 275Mhz 550Mhz\n");
		printk("Ex. To Change Frequency - echo 110000000 > current_freq \n");

		return ret;
	}

	printk("Frequency Supported - 110Mhz 275Mhz 550Mhz 733Mhz \n");
	printk("Ex. To Change Frequency - echo 110000000 > current_freq \n");

	return ret;
}

/*
 * nss_get_average_inst_handler()
 *	Display AVG Inst Per Ms.
 */
static int nss_get_average_inst_handler(ctl_table *ctl, int write, void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);

	if (!ret && !write) {
		printk("Current Inst Per Ms %x\n", nss_runtime_samples.average);
	}

	return ret;
}

/*
 * nss_debug_handler()
 *	Enable NSS debug output
 */
static int nss_debug_handler(ctl_table *ctl, int write, void __user *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	if (!ret) {
		if ((write) && (nss_ctl_debug != 0)) {
			printk("Enabling NSS SPI Debug\n");
			nss_hal_debug_enable();
		}
	}

	return ret;
}

/*
 * nss_coredump_handler()
 *	Send Signal To Coredump NSS Cores
 */
static int nss_coredump_handler(ctl_table *ctl, int write, void __user *buffer, size_t *lenp, loff_t *ppos)
{
	struct nss_ctx_instance *nss_ctx = (struct nss_ctx_instance *) nss_freq_change_context;
	int ret;

	ret = proc_dointvec(ctl, write, buffer, lenp, ppos);
	if (!ret) {
		if ((write) && (nss_ctl_debug != 0)) {
			printk("Coredumping to DDR\n");
			nss_hal_send_interrupt(nss_ctx->nmap, nss_ctx->h2n_desc_rings[NSS_IF_CMD_QUEUE].desc_ring.int_bit, NSS_REGS_H2N_INTR_STATUS_COREDUMP_START);
		}
	}

	return ret;
}

/*
 * sysctl-tuning infrastructure.
 */
static ctl_table nss_freq_table[] = {
	{
		.procname		= "current_freq",
		.data			= &nss_cmd_buf.current_freq,
		.maxlen			= sizeof(int),
		.mode			= 0644,
		.proc_handler	= &nss_current_freq_handler,
	},
	{
		.procname		= "freq_table",
		.data			= &nss_cmd_buf.max_freq,
		.maxlen			= sizeof(int),
		.mode			= 0644,
		.proc_handler	= &nss_get_freq_table_handler,
	},
	{
		.procname		= "auto_scale",
		.data			= &nss_cmd_buf.auto_scale,
		.maxlen			= sizeof(int),
		.mode			= 0644,
		.proc_handler	= &nss_auto_scale_handler,
	},
	{
		.procname		= "inst_per_sec",
		.data			= &nss_cmd_buf.average_inst,
		.maxlen			= sizeof(int),
		.mode			= 0644,
		.proc_handler	= &nss_get_average_inst_handler,
	},
	{ }
};

static ctl_table nss_general_table[] = {
	{
		.procname               = "redirect",
		.data                   = &nss_ctl_redirect,
		.maxlen                 = sizeof(int),
		.mode                   = 0644,
		.proc_handler   = proc_dointvec,
	},
	{
		.procname               = "debug",
		.data                   = &nss_ctl_debug,
		.maxlen                 = sizeof(int),
		.mode                   = 0644,
		.proc_handler   = &nss_debug_handler,
	},
	{
		.procname               = "coredump",
		.data                   = &nss_cmd_buf.coredump,
		.maxlen                 = sizeof(int),
		.mode                   = 0644,
		.proc_handler   = &nss_coredump_handler,
	},
	{ }
};

static ctl_table nss_clock_dir[] = {
	{
		.procname               = "clock",
		.mode                   = 0555,
		.child                  = nss_freq_table,
	},
	{
		.procname               = "general",
		.mode                   = 0555,
		.child                  = nss_general_table,
	},
	{ }
};

static ctl_table nss_root_dir[] = {
	{
		.procname		= "nss",
		.mode			= 0555,
		.child			= nss_clock_dir,
	},
	{ }
};

static ctl_table nss_root[] = {
	{
		.procname		= "dev",
		.mode			= 0555,
		.child			= nss_root_dir,
	},
	{ }
};
#endif /** NSS_PM_SUPPORT */

static struct ctl_table_header *nss_dev_header;

/*
 * nss_init()
 *	Registers nss driver
 */
static int __init nss_init(void)
{
	struct device_node *cmn = NULL;
	struct resource res_nss_fpb_base;

	nss_info("Init NSS driver");

	/*
	 * Get reference to NSS common device node
	 */
	cmn = of_find_node_by_name(NULL, "nss-common");
	if (!cmn) {
		nss_info("cannot find nss-common node\n");
		return -EFAULT;
	}

	if (of_address_to_resource(cmn, 0, &res_nss_fpb_base) != 0) {
		nss_info("of_address_to_resource() return error for nss_fpb_base\n");
		of_node_put(cmn);
		return -EFAULT;
	}

	nss_top_main.nss_fpb_base = ioremap_nocache(res_nss_fpb_base.start,
						    resource_size(&res_nss_fpb_base));
	if (!nss_top_main.nss_fpb_base) {
		nss_info("ioremap fail for nss_fpb_base\n");
		of_node_put(cmn);
		return -EFAULT;
	}

#if (NSS_PM_SUPPORT == 1)
	nss_freq_change_context = nss_get_frequency_mgr();
#else
	nss_freq_change_context = NULL;
#endif

	nss_top_main.nss_hal_common_init_done = false;

	/*
	 * Release reference to NSS common device node
	 */
	of_node_put(cmn);
	cmn = NULL;

	/*
	 * Enable spin locks
	 */
	spin_lock_init(&(nss_top_main.lock));
	spin_lock_init(&(nss_top_main.stats_lock));

	/*
	 * Enable NSS statistics
	 */
	nss_stats_init();

#if (NSS_PM_SUPPORT == 1)
	/*
	 * Register sysctl table.
	 */
	nss_dev_header = register_sysctl_table(nss_root);

	/*
	 * Setup Runtime Sample values
	 */
	nss_runtime_samples.freq_scale[0].frequency = 	NSS_FREQ_110;
	nss_runtime_samples.freq_scale[0].minimum =	NSS_FREQ_110_MIN;
	nss_runtime_samples.freq_scale[0].maximum = 	NSS_FREQ_110_MAX;
	nss_runtime_samples.freq_scale[1].frequency = 	NSS_FREQ_550;
	nss_runtime_samples.freq_scale[1].minimum = 	NSS_FREQ_550_MIN;
	nss_runtime_samples.freq_scale[1].maximum = 	NSS_FREQ_550_MAX;
	nss_runtime_samples.freq_scale[2].frequency = 	NSS_FREQ_733;
	nss_runtime_samples.freq_scale[2].minimum = 	NSS_FREQ_733_MIN;
	nss_runtime_samples.freq_scale[2].maximum = 	NSS_FREQ_733_MAX;
	nss_runtime_samples.freq_scale_index = 1;
	nss_runtime_samples.freq_scale_ready = 0;
	nss_runtime_samples.freq_scale_rate_limit_up = 0;
	nss_runtime_samples.freq_scale_rate_limit_down = 0;
	nss_runtime_samples.buffer_index = 0;
	nss_runtime_samples.sum = 0;
	nss_runtime_samples.sample_count = 0;
	nss_runtime_samples.average = 0;
	nss_runtime_samples.message_rate_limit = 0;
	nss_runtime_samples.initialized = 0;

	nss_cmd_buf.current_freq = nss_runtime_samples.freq_scale[nss_runtime_samples.freq_scale_index].frequency;

	/*
	 * Initial Workqueue
	 */
	nss_wq = create_workqueue("nss_freq_queue");

	/*
	 * Initialize NSS Bus PM module
	 */
	nss_pm_init();

	/*
	 * Register with Bus driver
	 */
	pm_client = nss_pm_client_register(NSS_PM_CLIENT_NETAP);
	if (!pm_client) {
		nss_warning("Error registering with PM driver");
	}
#endif

	/*
	 * Register platform_driver
	 */
	return platform_driver_register(&nss_driver);
}

/*
 * nss_cleanup()
 *	Unregisters nss driver
 */
static void __exit nss_cleanup(void)
{
	nss_info("Exit NSS driver");

	if (nss_dev_header)
		unregister_sysctl_table(nss_dev_header);

	if(nss_top_main.nss_fpb_base) {
		iounmap(nss_top_main.nss_fpb_base);
		nss_top_main.nss_fpb_base = 0;
	}

	platform_driver_unregister(&nss_driver);
}

module_init(nss_init);
module_exit(nss_cleanup);

MODULE_DESCRIPTION("QCA NSS Driver");
MODULE_AUTHOR("Qualcomm Atheros Inc");
MODULE_LICENSE("Dual BSD/GPL");
