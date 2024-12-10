/*
 * Header file describing the internal (inter-module) DHD interfaces.
 *
 * Provides type definitions and function prototypes used to link the
 * DHD OS, bus, and protocol modules.
 *
 * Portions of this code are copyright (c) 2022 Cypress Semiconductor Corporation
 *
 * Copyright (C) 1999-2017, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: dhd_bus.h 701741 2017-05-26 08:18:08Z $
 */

#ifndef _dhd_bus_h_
#define _dhd_bus_h_

/*
 * Exported from dhd bus module (dhd_usb, dhd_sdio)
 */

/* global variable for the bus */
extern struct dhd_bus *g_dhd_bus;

/* Indicate (dis)interest in finding dongles. */
extern int dhd_bus_register(void);
extern void dhd_bus_unregister(void);

/* Download firmware image and nvram image */
extern int dhd_bus_download_firmware(struct dhd_bus *bus, osl_t *osh, char *fw_path, char *nv_path);
#if defined(BT_OVER_SDIO)
extern int dhd_bus_download_btfw(struct dhd_bus *bus, osl_t *osh, char *btfw_path);
#endif /* defined (BT_OVER_SDIO) */

/* Stop bus module: clear pending frames, disable data flow */
extern void dhd_bus_stop(struct dhd_bus *bus, bool enforce_mutex);

/* Initialize bus module: prepare for communication w/dongle */
extern int dhd_bus_init(dhd_pub_t *dhdp, bool enforce_mutex);

/* Get the Bus Idle Time */
extern void dhd_bus_getidletime(dhd_pub_t *dhdp, int *idletime);

/* Set the Bus Idle Time */
extern void dhd_bus_setidletime(dhd_pub_t *dhdp, int idle_time);

/* Send a data frame to the dongle.  Callee disposes of txp. */
#ifdef BCMPCIE
extern int dhd_bus_txdata(struct dhd_bus *bus, void *txp, uint8 ifidx);
#else
extern int dhd_bus_txdata(struct dhd_bus *bus, void *txp);
#endif // endif

#ifdef BCMPCIE
extern void dhdpcie_cto_recovery_handler(dhd_pub_t *dhd);
#endif /* BCMPCIE */

/* Send/receive a control message to/from the dongle.
 * Expects caller to enforce a single outstanding transaction.
 */
extern int dhd_bus_txctl(struct dhd_bus *bus, uchar *msg, uint msglen);
extern int dhd_bus_rxctl(struct dhd_bus *bus, uchar *msg, uint msglen);

/* Watchdog timer function */
extern bool dhd_bus_watchdog(dhd_pub_t *dhd);

extern int dhd_bus_oob_intr_register(dhd_pub_t *dhdp);
extern void dhd_bus_oob_intr_unregister(dhd_pub_t *dhdp);
extern void dhd_bus_oob_intr_set(dhd_pub_t *dhdp, bool enable);
extern void dhd_bus_dev_pm_stay_awake(dhd_pub_t *dhdpub);
extern void dhd_bus_dev_pm_relax(dhd_pub_t *dhdpub);
extern bool dhd_bus_dev_pm_enabled(dhd_pub_t *dhdpub);

/* Device console input function */
extern int dhd_bus_console_in(dhd_pub_t *dhd, uchar *msg, uint msglen);

/* Deferred processing for the bus, return TRUE requests reschedule */
extern bool dhd_bus_dpc(struct dhd_bus *bus);
extern void dhd_bus_isr(bool * InterruptRecognized, bool * QueueMiniportHandleInterrupt, void *arg);

/* Check for and handle local prot-specific iovar commands */
extern int dhd_bus_iovar_op(dhd_pub_t *dhdp, const char *name,
                            void *params, int plen, void *arg, int len, bool set);

/* Add bus dump output to a buffer */
extern void dhd_bus_dump(dhd_pub_t *dhdp, struct bcmstrbuf *strbuf);

/* Clear any bus counters */
extern void dhd_bus_clearcounts(dhd_pub_t *dhdp);

/* return the dongle chipid */
extern uint dhd_bus_chip(struct dhd_bus *bus);

/* return the dongle chiprev */
extern uint dhd_bus_chiprev(struct dhd_bus *bus);

/* Set user-specified nvram parameters. */
extern void dhd_bus_set_nvram_params(struct dhd_bus * bus, const char *nvram_params);

extern void *dhd_bus_pub(struct dhd_bus *bus);
extern void *dhd_bus_txq(struct dhd_bus *bus);
extern void *dhd_bus_sih(struct dhd_bus *bus);
extern uint dhd_bus_hdrlen(struct dhd_bus *bus);
#ifdef BCMSDIO
extern void dhd_bus_set_dotxinrx(struct dhd_bus *bus, bool val);
/* return sdio io status */
extern uint8 dhd_bus_is_ioready(struct dhd_bus *bus);
#else
#define dhd_bus_set_dotxinrx(a, b) do {} while (0)
#endif // endif

#define DHD_SET_BUS_STATE_DOWN(_bus)  do { \
	(_bus)->dhd->busstate = DHD_BUS_DOWN; \
} while (0)

/* Register a dummy SDIO client driver in order to be notified of new SDIO device */
extern int dhd_bus_reg_sdio_notify(void* semaphore);
extern void dhd_bus_unreg_sdio_notify(void);
extern void dhd_txglom_enable(dhd_pub_t *dhdp, bool enable);
extern int dhd_bus_get_ids(struct dhd_bus *bus, uint32 *bus_type, uint32 *bus_num,
	uint32 *slot_num);

#if defined(DHD_FW_COREDUMP) && (defined(BCMPCIE) || defined(BCMSDIO))
extern int dhd_bus_mem_dump(dhd_pub_t *dhd);
extern int dhd_bus_get_mem_dump(dhd_pub_t *dhdp);
#else
#define dhd_bus_mem_dump(x)
#define dhd_bus_get_mem_dump(x)
#endif /* DHD_FW_COREDUMP && (BCMPCIE || BCMSDIO) */

#ifdef BCMPCIE
enum {
	/* Scratch buffer confiuguration update */
	D2H_DMA_SCRATCH_BUF,
	D2H_DMA_SCRATCH_BUF_LEN,

	/* DMA Indices array buffers for: H2D WR and RD, and D2H WR and RD */
	H2D_DMA_INDX_WR_BUF, /* update H2D WR dma indices buf base addr to dongle */
	H2D_DMA_INDX_RD_BUF, /* update H2D RD dma indices buf base addr to dongle */
	D2H_DMA_INDX_WR_BUF, /* update D2H WR dma indices buf base addr to dongle */
	D2H_DMA_INDX_RD_BUF, /* update D2H RD dma indices buf base addr to dongle */

	/* DHD sets/gets WR or RD index, in host's H2D and D2H DMA indices buffer */
	H2D_DMA_INDX_WR_UPD, /* update H2D WR index in H2D WR dma indices buf */
	H2D_DMA_INDX_RD_UPD, /* update H2D RD index in H2D RD dma indices buf */
	D2H_DMA_INDX_WR_UPD, /* update D2H WR index in D2H WR dma indices buf */
	D2H_DMA_INDX_RD_UPD, /* update D2H RD index in D2H RD dma indices buf */

	/* DHD Indices array buffers and update for: H2D flow ring WR */
	H2D_IFRM_INDX_WR_BUF, /* update H2D WR dma indices buf base addr to dongle */
	H2D_IFRM_INDX_WR_UPD, /* update H2D WR dma indices buf base addr to dongle */

	/* H2D and D2H Mailbox data update */
	H2D_MB_DATA,
	D2H_MB_DATA,

	/* (Common) MsgBuf Ring configuration update */
	RING_BUF_ADDR,       /* update ring base address to dongle */
	RING_ITEM_LEN,       /* update ring item size to dongle */
	RING_MAX_ITEMS,      /* update ring max items to dongle */

	/* Update of WR or RD index, for a MsgBuf Ring */
	RING_RD_UPD,         /* update ring read index from/to dongle */
	RING_WR_UPD,         /* update ring write index from/to dongle */

	TOTAL_LFRAG_PACKET_CNT,
	MAX_HOST_RXBUFS,
	HOST_API_VERSION,
	DNGL_TO_HOST_TRAP_ADDR,
	HOST_SCB_ADDR,		/* update host scb base address to dongle */
};

typedef void (*dhd_mb_ring_t) (struct dhd_bus *, uint32);
typedef void (*dhd_mb_ring_2_t) (struct dhd_bus *, uint32, bool);
extern void dhd_bus_cmn_writeshared(struct dhd_bus *bus, void * data, uint32 len, uint8 type,
	uint16 ringid);
extern void dhd_bus_ringbell(struct dhd_bus *bus, uint32 value);
extern void dhd_bus_ringbell_2(struct dhd_bus *bus, uint32 value, bool devwake);
extern void dhd_bus_cmn_readshared(struct dhd_bus *bus, void* data, uint8 type, uint16 ringid);
extern uint32 dhd_bus_get_sharedflags(struct dhd_bus *bus);
extern void dhd_bus_rx_frame(struct dhd_bus *bus, void* pkt, int ifidx, uint pkt_count);
extern void dhd_bus_start_queue(struct dhd_bus *bus);
extern void dhd_bus_stop_queue(struct dhd_bus *bus);
extern dhd_mb_ring_t dhd_bus_get_mbintr_fn(struct dhd_bus *bus);
extern dhd_mb_ring_2_t dhd_bus_get_mbintr_2_fn(struct dhd_bus *bus);
extern void dhd_bus_write_flow_ring_states(struct dhd_bus *bus,
	void * data, uint16 flowid);
extern void dhd_bus_read_flow_ring_states(struct dhd_bus *bus,
	void * data, uint8 flowid);
extern int dhd_bus_flow_ring_create_request(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_clean_flow_ring(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_flow_ring_create_response(struct dhd_bus *bus, uint16 flow_id, int32 status);
extern int dhd_bus_flow_ring_delete_request(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_flow_ring_delete_response(struct dhd_bus *bus, uint16 flowid, uint32 status);
extern int dhd_bus_flow_ring_flush_request(struct dhd_bus *bus, void *flow_ring_node);
extern void dhd_bus_flow_ring_flush_response(struct dhd_bus *bus, uint16 flowid, uint32 status);
extern uint32 dhd_bus_max_h2d_queues(struct dhd_bus *bus);
extern int dhd_bus_schedule_queue(struct dhd_bus *bus, uint16 flow_id, bool txs);

#ifdef IDLE_TX_FLOW_MGMT
extern void dhd_bus_flow_ring_resume_response(struct dhd_bus *bus, uint16 flowid, int32 status);
#endif /* IDLE_TX_FLOW_MGMT */

extern int dhdpcie_bus_clock_start(struct dhd_bus *bus);
extern int dhdpcie_bus_clock_stop(struct dhd_bus *bus);
extern int dhdpcie_bus_enable_device(struct dhd_bus *bus);
extern int dhdpcie_bus_disable_device(struct dhd_bus *bus);
extern int dhdpcie_bus_alloc_resource(struct dhd_bus *bus);
extern void dhdpcie_bus_free_resource(struct dhd_bus *bus);
extern bool dhdpcie_bus_dongle_attach(struct dhd_bus *bus);
extern int dhd_bus_release_dongle(struct dhd_bus *bus);
extern int dhd_bus_request_irq(struct dhd_bus *bus);
extern int dhdpcie_get_pcieirq(struct dhd_bus *bus, unsigned int *irq);
extern void dhd_bus_aer_config(struct dhd_bus *bus);

extern struct device * dhd_bus_to_dev(struct dhd_bus *bus);

extern int dhdpcie_cto_init(struct dhd_bus *bus, bool enable);
extern int dhdpcie_cto_cfg_init(struct dhd_bus *bus, bool enable);

extern void dhdpcie_ssreset_dis_enum_rst(struct dhd_bus *bus);

#ifdef DHD_FW_COREDUMP
#ifdef BCMDHDX
extern int dhdx_dongle_mem_dump(void);
#else
extern int dhd_dongle_mem_dump(void);
#endif /* BCMDHDX */
#endif /* DHD_FW_COREDUMP */

#ifdef IDLE_TX_FLOW_MGMT
extern void dhd_bus_idle_tx_ring_suspend(dhd_pub_t *dhd, uint16 flow_ring_id);
#endif /* IDLE_TX_FLOW_MGMT */
extern void dhd_bus_handle_mb_data(struct dhd_bus *bus, uint32 d2h_mb_data);
#endif /* BCMPCIE */

/* dump the device trap informtation  */
extern void dhd_bus_dump_trap_info(struct dhd_bus *bus, struct bcmstrbuf *b);
extern void dhd_bus_copy_trap_sig(struct dhd_bus *bus,  trap_t *tr);
#ifdef WL_CFGVENDOR_SEND_HANG_EVENT
void copy_ext_trap_sig(dhd_pub_t *dhd, trap_t *tr);
void copy_hang_info_trap(dhd_pub_t *dhd);
#endif /* WL_CFGVENDOR_SEND_HANG_EVENT */
/* Function to set default min res mask */
extern bool dhd_bus_set_default_min_res_mask(struct dhd_bus *bus);

/* Function to reset PMU registers */
extern void dhd_bus_pmu_reg_reset(dhd_pub_t *dhdp);

extern void dhd_bus_ucode_download(struct dhd_bus *bus);

#ifdef DHD_ULP
extern void dhd_bus_ulp_disable_console(dhd_pub_t *dhdp);
#endif /* DHD_ULP */
extern int dhd_bus_readwrite_bp_addr(dhd_pub_t *dhdp, uint addr, uint size, uint* data, bool read);
extern int dhd_get_idletime(dhd_pub_t *dhd);
#ifdef BCMPCIE
extern void dhd_bus_dump_console_buffer(struct dhd_bus *bus);
extern void dhd_bus_intr_count_dump(dhd_pub_t *dhdp);
extern void dhd_bus_set_dpc_sched_time(dhd_pub_t *dhdp);
extern bool dhd_bus_query_dpc_sched_errors(dhd_pub_t *dhdp);
extern int dhd_bus_dmaxfer_lpbk(dhd_pub_t *dhdp, uint32 type);
#ifndef BCMDHDX
extern bool dhd_bus_check_driver_up(void);
#else
extern bool dhdx_bus_check_driver_up(void);
#endif /* BCMDHDX */
extern int dhd_bus_get_cto(dhd_pub_t *dhdp);
extern void dhd_bus_set_linkdown(dhd_pub_t *dhdp, bool val);
extern int dhd_bus_get_linkdown(dhd_pub_t *dhdp);
#else
#define dhd_bus_dump_console_buffer(x)
static INLINE void dhd_bus_intr_count_dump(dhd_pub_t *dhdp) { UNUSED_PARAMETER(dhdp); }
static INLINE void dhd_bus_set_dpc_sched_time(dhd_pub_t *dhdp) { }
static INLINE bool dhd_bus_query_dpc_sched_errors(dhd_pub_t *dhdp) { return 0; }
static INLINE int dhd_bus_dmaxfer_lpbk(dhd_pub_t *dhdp, uint32 type) { return 0; }
static INLINE bool dhd_bus_check_driver_up(void) { return FALSE; }
extern INLINE void dhd_bus_set_linkdown(dhd_pub_t *dhdp, bool val) { }
extern INLINE int dhd_bus_get_linkdown(dhd_pub_t *dhdp) { return 0; }
static INLINE int dhd_bus_get_cto(dhd_pub_t *dhdp) { return 0; }
#endif /* BCMPCIE */

#if defined(BCMPCIE) && defined(EWP_ETD_PRSRV_LOGS)
void dhdpcie_get_etd_preserve_logs(dhd_pub_t *dhd, uint8 *ext_trap_data,
		void *event_decode_data);
#endif // endif

extern uint16 dhd_get_chipid(dhd_pub_t *dhd);

#ifdef DHD_WAKE_STATUS
extern wake_counts_t* dhd_bus_get_wakecount(dhd_pub_t *dhd);
extern int dhd_bus_get_bus_wake(dhd_pub_t * dhd);
#endif /* DHD_WAKE_STATUS */

#ifdef BT_OVER_SDIO
/*
 * SDIO layer clock control functions exposed to be called from other layers.
 * This is required especially in the case where the BUS is shared between
 * BT and SDIO and we have to control the clock. The callers of this function
 * are expected to hold the sdlock
 */
int __dhdsdio_clk_enable(struct dhd_bus *bus, bus_owner_t owner, int can_wait);
int __dhdsdio_clk_disable(struct dhd_bus *bus, bus_owner_t owner, int can_wait);
void dhdsdio_reset_bt_use_count(struct dhd_bus *bus);
#endif /* BT_OVER_SDIO */

int dhd_bus_perform_flr(struct dhd_bus *bus, bool force_fail);
extern bool dhd_bus_get_flr_force_fail(struct dhd_bus *bus);

extern bool dhd_bus_aspm_enable_rc_ep(struct dhd_bus *bus, bool enable);
extern void dhd_bus_l1ss_enable_rc_ep(struct dhd_bus *bus, bool enable);

bool dhd_bus_is_multibp_capable(struct dhd_bus *bus);

#ifdef BCMPCIE
extern void dhdpcie_advertise_bus_cleanup(dhd_pub_t  *dhdp);
extern void dhd_msgbuf_iovar_timeout_dump(dhd_pub_t *dhd);
#endif /* BCMPCIE */

extern bool dhd_bus_force_bt_quiesce_enabled(struct dhd_bus *bus);

#ifdef DHD_SSSR_DUMP
extern int dhd_bus_fis_trigger(dhd_pub_t *dhd);
extern int dhd_bus_fis_dump(dhd_pub_t *dhd);
#endif /* DHD_SSSR_DUMP */

#ifdef PCIE_FULL_DONGLE
extern int dhdpcie_set_dma_ring_indices(dhd_pub_t *dhd, int32 int_val);
#endif /* PCIE_FULL_DONGLE */

#ifdef DHD_USE_BP_RESET
extern int dhd_bus_perform_bp_reset(struct dhd_bus *bus);
#endif /* DHD_USE_BP_RESET */

extern void dhd_bwm_bt_quiesce(struct dhd_bus *bus);
extern void dhd_bwm_bt_resume(struct dhd_bus *bus);
#endif /* _dhd_bus_h_ */
