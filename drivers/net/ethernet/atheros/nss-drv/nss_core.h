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
 * na_core.h
 *	NSS driver core header file.
 */

#ifndef __NSS_CORE_H
#define __NSS_CORE_H

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>

#include <nss_api_if.h>
#include "nss_hlos_if.h"

/*
 * XXX:can't add this to api_if.h till the deprecated
 * API(s) are present. Once, thats removed we will move it
 * to this file
 */
#include "nss_ipsec.h"
#include "nss_crypto.h"

/*
 * NSS debug macros
 */
#if (NSS_DEBUG_LEVEL < 1)
#define nss_assert(fmt, args...)
#else
#define nss_assert(c) if (!(c)) { BUG_ON(!(c)); }
#endif

#if (NSS_DEBUG_LEVEL < 2)
#define nss_warning(fmt, args...)
#else
#define nss_warning(fmt, args...) printk(KERN_WARNING "nss:"fmt, ##args)
#endif

#if (NSS_DEBUG_LEVEL < 3)
#define nss_info(fmt, args...)
#else
#define nss_info(fmt, args...) printk(KERN_INFO "nss:"fmt, ##args)
#endif

#if (NSS_DEBUG_LEVEL < 4)
#define nss_trace(fmt, args...)
#else
#define nss_trace(fmt, args...) printk(KERN_DEBUG "nss:"fmt, ##args)
#endif

#if (NSS_PKT_STATS_ENABLED == 1)
#define NSS_PKT_STATS_INCREMENT(nss_ctx, x) nss_pkt_stats_increment((nss_ctx), (x))
#else
#define NSS_PKT_STATS_INCREMENT(nss_ctx, x)
#endif

/*
 * NSS max values supported
 */
#define NSS_MAX_CORES 2
#define NSS_MAX_DEVICE_INTERFACES (NSS_MAX_PHYSICAL_INTERFACES + NSS_MAX_VIRTUAL_INTERFACES + NSS_MAX_TUNNEL_INTERFACES)
#define NSS_MAX_NET_INTERFACES (NSS_MAX_DEVICE_INTERFACES + NSS_MAX_SPECIAL_INTERFACES)

#define NSS_DEVICE_IF_START NSS_PHYSICAL_IF_START

#define NSS_IS_IF_TYPE(type, if_num) ((if_num >= NSS_##type##_IF_START) && (if_num < (NSS_##type##_IF_START + NSS_MAX_##type##_INTERFACES)))

/*
 * Default payload size for NSS buffers
 */
#define NSS_NBUF_PAYLOAD_SIZE NSS_EMPTY_BUFFER_SIZE
#define NSS_NBUF_PAD_EXTRA 256
#define NSS_NBUF_ETH_EXTRA 192

/*
 * N2H/H2N Queue IDs
 */
#define NSS_IF_EMPTY_BUFFER_QUEUE 0
#define NSS_IF_DATA_QUEUE 1
#define NSS_IF_CMD_QUEUE 1


/*
 * NSS Interrupt Causes
 */
#define NSS_INTR_CAUSE_INVALID 0
#define NSS_INTR_CAUSE_QUEUE 1
#define NSS_INTR_CAUSE_NON_QUEUE 2

/*
 * NSS Core Status
 */
#define NSS_CORE_STATUS_SUCCESS 0
#define NSS_CORE_STATUS_FAILURE 1
#define NSS_CORE_STATUS_FAILURE_QUEUE 2

/*
 * NSS context magic
 */
#define NSS_CTX_MAGIC 0xDEDEDEDE

/*
 * NSS maximum clients
 */
#define NSS_MAX_CLIENTS 12

/*
 * Interrupt cause processing weights
 */
#define NSS_EMPTY_BUFFER_SOS_PROCESSING_WEIGHT 64
#define NSS_DATA_COMMAND_BUFFER_PROCESSING_WEIGHT 64
#define NSS_EMPTY_BUFFER_RETURN_PROCESSING_WEIGHT 64
#define NSS_TX_UNBLOCKED_PROCESSING_WEIGHT 1

/*
 * Statistics struct
 *
 * INFO: These numbers are based on previous generation chip
 *	These may change in future
 */
#define NSS_PPPOE_NUM_SESSION_PER_INTERFACE 8
					/* Number of maximum simultaneous PPPoE sessions per physical interface */
#define NSS_PPPOE_NUM_SESSION_TOTAL (NSS_MAX_PHYSICAL_INTERFACES * NSS_PPPOE_NUM_SESSION_PER_INTERFACE)
					/* Number of total PPPoE sessions */


/*
 * NSS Frequency Defines and Values
 *
 * INFO: The LOW and MAX value together describe the "performance" band that we should operate the frequency at.
 *
 */
#define NSS_FREQ_110		110000000	/* Frequency in hz */
#define NSS_FREQ_110_MIN	0x03000		/* Instructions Per ms Min */
#define NSS_FREQ_110_MAX	0x08000		/* Instructions Per ms Max */

#define NSS_FREQ_275		275000000	/* Frequency in hz */

#define NSS_FREQ_550		550000000	/* Frequency in hz */
#define NSS_FREQ_550_MIN	0x07000		/* Instructions Per ms Min */
#define NSS_FREQ_550_MAX	0x16000		/* Instructions Per ms Max */

#define NSS_FREQ_733		733000000	/* Frequency in hz */
#define NSS_FREQ_733_MIN	0x10000		/* Instructions Per ms Min */
#define NSS_FREQ_733_MAX	0x50000		/* Instructions Per ms Max */

#define NSSTCM_FREQ		400000000	/* NSS TCM Frequency in Hz */


/* NSS Clock names */
#define NSS_TCM_SRC_CLK		"nss_tcm_src"
#define NSS_TCM_CLK		"nss_tcm_clk"

/* NSS core reset/clamp names */
#define NSS_CORE_CLK_RST_CLAMP	"clkrst_clamp"
#define NSS_CORE_CLAMP		"clamp"
#define NSS_CORE_AHB_RESET	"ahb"
#define NSS_CORE_AXI_RESET	"axi"

/*
 * IPV4 node statistics
 *
 * WARNING: There is a 1:1 mapping between values below and corresponding
 *	stats string array in nss_stats.c
 */
enum nss_stats_ipv4 {
	NSS_STATS_IPV4_ACCELERATED_RX_PKTS = 0,
					/* Accelerated IPv4 RX packets */
	NSS_STATS_IPV4_ACCELERATED_RX_BYTES,
					/* Accelerated IPv4 RX bytes */
	NSS_STATS_IPV4_ACCELERATED_TX_PKTS,
					/* Accelerated IPv4 TX packets */
	NSS_STATS_IPV4_ACCELERATED_TX_BYTES,
					/* Accelerated IPv4 TX bytes */
	NSS_STATS_IPV4_CONNECTION_CREATE_REQUESTS,
					/* Number of IPv4 connection create requests */
	NSS_STATS_IPV4_CONNECTION_CREATE_COLLISIONS,
					/* Number of IPv4 connection create requests that collided with existing entries */
	NSS_STATS_IPV4_CONNECTION_CREATE_INVALID_INTERFACE,
					/* Number of IPv4 connection create requests that had invalid interface */
	NSS_STATS_IPV4_CONNECTION_DESTROY_REQUESTS,
					/* Number of IPv4 connection destroy requests */
	NSS_STATS_IPV4_CONNECTION_DESTROY_MISSES,
					/* Number of IPv4 connection destroy requests that missed the cache */
	NSS_STATS_IPV4_CONNECTION_HASH_HITS,
					/* Number of IPv4 connection hash hits */
	NSS_STATS_IPV4_CONNECTION_HASH_REORDERS,
					/* Number of IPv4 connection hash reorders */
	NSS_STATS_IPV4_CONNECTION_FLUSHES,
					/* Number of IPv4 connection flushes */
	NSS_STATS_IPV4_CONNECTION_EVICTIONS,
					/* Number of IPv4 connection evictions */
	NSS_STATS_IPV4_MAX,
};

/*
 * IPV6 node statistics
 *
 * WARNING: There is a 1:1 mapping between values below and corresponding
 *	stats string array in nss_stats.c
 */
enum nss_stats_ipv6 {
	NSS_STATS_IPV6_ACCELERATED_RX_PKTS = 0,
					/* Accelerated IPv6 RX packets */
	NSS_STATS_IPV6_ACCELERATED_RX_BYTES,
					/* Accelerated IPv6 RX bytes */
	NSS_STATS_IPV6_ACCELERATED_TX_PKTS,
					/* Accelerated IPv6 TX packets */
	NSS_STATS_IPV6_ACCELERATED_TX_BYTES,
					/* Accelerated IPv6 TX bytes */
	NSS_STATS_IPV6_CONNECTION_CREATE_REQUESTS,
					/* Number of IPv6 connection create requests */
	NSS_STATS_IPV6_CONNECTION_CREATE_COLLISIONS,
					/* Number of IPv6 connection create requests that collided with existing entries */
	NSS_STATS_IPV6_CONNECTION_CREATE_INVALID_INTERFACE,
					/* Number of IPv6 connection create requests that had invalid interface */
	NSS_STATS_IPV6_CONNECTION_DESTROY_REQUESTS,
					/* Number of IPv6 connection destroy requests */
	NSS_STATS_IPV6_CONNECTION_DESTROY_MISSES,
					/* Number of IPv6 connection destroy requests that missed the cache */
	NSS_STATS_IPV6_CONNECTION_HASH_HITS,
					/* Number of IPv6 connection hash hits */
	NSS_STATS_IPV6_CONNECTION_HASH_REORDERS,
					/* Number of IPv6 connection hash reorders */
	NSS_STATS_IPV6_CONNECTION_FLUSHES,
					/* Number of IPv6 connection flushes */
	NSS_STATS_IPV6_CONNECTION_EVICTIONS,
					/* Number of IPv6 connection evictions */
	NSS_STATS_IPV6_MAX,
};

/*
 * HLOS driver statistics
 *
 * WARNING: There is a 1:1 mapping between values below and corresponding
 *	stats string array in nss_stats.c
 */
enum nss_stats_drv {
	NSS_STATS_DRV_NBUF_ALLOC_FAILS = 0,	/* NBUF allocation errors */
	NSS_STATS_DRV_TX_QUEUE_FULL_0,	/* Tx queue full for Core 0*/
	NSS_STATS_DRV_TX_QUEUE_FULL_1,	/* Tx queue full for Core 1*/
	NSS_STATS_DRV_TX_EMPTY,		/* H2N Empty buffers */
	NSS_STATS_DRV_TX_PACKET,	/* H2N Data packets */
	NSS_STATS_DRV_TX_CMD_REQ,	/* H2N Control packets */
	NSS_STATS_DRV_TX_CRYPTO_REQ,	/* H2N Crypto requests */
	NSS_STATS_DRV_RX_EMPTY,		/* N2H Empty buffers */
	NSS_STATS_DRV_RX_PACKET,	/* N2H Data packets */
	NSS_STATS_DRV_RX_CMD_RESP,	/* N2H Command responses */
	NSS_STATS_DRV_RX_STATUS,	/* N2H Status packets */
	NSS_STATS_DRV_RX_CRYPTO_RESP,	/* N2H Crypto responses */
	NSS_STATS_DRV_RX_VIRTUAL,	/* N2H Virtual packets */
	NSS_STATS_DRV_MAX,
};

/*
 * PPPoE statistics
 *
 * WARNING: There is a 1:1 mapping between values below and corresponding
 *	stats string array in nss_stats.c
 */
enum nss_stats_pppoe {
	NSS_STATS_PPPOE_SESSION_CREATE_REQUESTS = 0,
					/* Number of PPPoE session create requests */
	NSS_STATS_PPPOE_SESSION_CREATE_FAILURES,
					/* Number of PPPoE session create failures */
	NSS_STATS_PPPOE_SESSION_DESTROY_REQUESTS,
					/* Number of PPPoE session destroy requests */
	NSS_STATS_PPPOE_SESSION_DESTROY_MISSES,
					/* Number of PPPoE session destroy requests that missed the cache */
	NSS_STATS_PPPOE_MAX,
};

/*
 * GMAC node statistics
 *
 * WARNING: There is a 1:1 mapping between values below and corresponding
 *	stats string array in nss_stats.c
 */
enum nss_stats_gmac {
	NSS_STATS_GMAC_TOTAL_TICKS = 0,
					/* Total clock ticks spend inside the GMAC */
	NSS_STATS_GMAC_WORST_CASE_TICKS,
					/* Worst case iteration of the GMAC in ticks */
	NSS_STATS_GMAC_ITERATIONS,	/* Number of iterations around the GMAC */
	NSS_STATS_GMAC_MAX,
};

/*
 * Node statistics
 *
 * WARNING: There is a 1:1 mapping between values below and corresponding
 *	stats string array in nss_stats.c
 */
enum nss_stats_node {
	NSS_STATS_NODE_RX_PKTS,
					/* Accelerated node RX packets */
	NSS_STATS_NODE_RX_BYTES,
					/* Accelerated node RX bytes */
	NSS_STATS_NODE_RX_DROPPED,
					/* Accelerated node RX dropped */
	NSS_STATS_NODE_TX_PKTS,
					/* Accelerated node TX packets */
	NSS_STATS_NODE_TX_BYTES,
					/* Accelerated node TX bytes */
	NSS_STATS_NODE_MAX,
};

/*
 * N2H node statistics
 *
 * WARNING: There is a 1:1 mapping between values below and corresponding
 *	stats string array in nss_stats.c
 */
enum nss_stats_n2h {
	NSS_STATS_N2H_QUEUE_DROPPED = NSS_STATS_NODE_MAX,
					/* Number of packets dropped because the exception queue is too full */
	NSS_STATS_N2H_TOTAL_TICKS,	/* Total clock ticks spend inside the N2H */
	NSS_STATS_N2H_WORST_CASE_TICKS,	/* Worst case iteration of the exception path in ticks */
	NSS_STATS_N2H_ITERATIONS,	/* Number of iterations around the N2H */
	NSS_STATS_N2H_PBUF_ALLOC_FAILS,	/* Number of pbuf allocations that have failed */
	NSS_STATS_N2H_PAYLOAD_ALLOC_FAILS,
					/* Number of pbuf allocations that have failed because there were no free payloads */
	NSS_STATS_N2H_MAX,
};

/*
 * NSS core state
 */
enum nss_core_state {
	NSS_CORE_STATE_UNINITIALIZED = 0,
	NSS_CORE_STATE_INITIALIZED
};

/*
 * Forward declarations
 */
struct nss_top_instance;
struct nss_ctx_instance;
struct int_ctx_instance;
struct net_dev_priv_instance;

/*
 * Network device private data instance
 */
struct netdev_priv_instance {
	struct int_ctx_instance *int_ctx;	/* Back pointer to interrupt context */
};

/*
 * Interrupt context instance (one per IRQ per NSS core)
 */
struct int_ctx_instance {
	struct nss_ctx_instance *nss_ctx;
					/* Back pointer to NSS context of core that
					owns this interrupt */
	uint32_t irq;			/* HLOS IRQ number */
	uint32_t shift_factor;		/* Shift factor for this IRQ number */
	uint32_t cause;			/* Interrupt cause carried forward to BH */
	struct net_device *ndev;	/* Network device associated with this interrupt
					   context */
	struct napi_struct napi;	/* NAPI handler */
	bool napi_active;		/* NAPI is active */
};

/*
 * N2H descriptor ring information
 */
struct hlos_n2h_desc_ring {
	struct n2h_desc_if_instance desc_if;
					/* Descriptor ring */
	uint32_t hlos_index;		/* Current HLOS index for this ring */
};

/*
 * H2N descriptor ring information
 */
struct hlos_h2n_desc_rings {
	struct h2n_desc_if_instance desc_ring;	/* Descriptor ring */
	uint32_t hlos_index;
	spinlock_t lock;			/* Lock to save from simultaneous access */
	uint32_t flags;				/* Flags */
	uint64_t tx_q_full_cnt;			/* Descriptor queue full count */
};

#define NSS_H2N_DESC_RING_FLAGS_TX_STOPPED 0x1	/* Tx has been stopped for this queue */

/*
 * struct nss_shaper_bounce_registrant
 *	Registrant detail for shaper bounce operations
 */
struct nss_shaper_bounce_registrant {
	nss_shaper_bounced_callback_t bounced_callback;		/* Invoked for each shaper bounced packet returned from the NSS */
	void *app_data;						/* Argument given to the callback */
	struct module *owner;					/* Owning module of the callback + arg */
	bool registered;
	volatile bool callback_active;				/* true when the bounce callback is being called */
};

/*
 * NSS context instance (one per NSS core)
 */
struct nss_ctx_instance {
	struct nss_top_instance *nss_top;
					/* Back pointer to NSS Top */
	uint32_t id;			/* Core ID for this instance */
	uint32_t nmap;			/* Pointer to NSS CSM registers */
	uint32_t vmap;			/* Virt mem pointer to virtual register map */
	uint32_t nphys;			/* Phys mem pointer to CSM register map */
	uint32_t vphys;			/* Phys mem pointer to virtual register map */
	uint32_t load;			/* Load address for this core */
	enum nss_core_state state;	/* State of NSS core */
	uint32_t c2c_start;		/* C2C start address */
	struct int_ctx_instance int_ctx[2];
					/* Interrupt context instances */
	struct hlos_h2n_desc_rings h2n_desc_rings[16];
					/* Host to NSS descriptor rings */
	struct hlos_n2h_desc_ring n2h_desc_ring[15];
					/* NSS to Host descriptor rings */
	uint32_t max_buf_size;		/* Maximum buffer size */
	nss_cmn_queue_decongestion_callback_t queue_decongestion_callback[NSS_MAX_CLIENTS];
					/* Queue decongestion callbacks */
	void *queue_decongestion_ctx[NSS_MAX_CLIENTS];
					/* Queue decongestion callback contexts */
	spinlock_t decongest_cb_lock;	/* Lock to protect queue decongestion cb table */
	uint16_t phys_if_mtu[NSS_MAX_PHYSICAL_INTERFACES];
					/* Current MTU value of physical interface */
	uint64_t stats_n2h[NSS_STATS_N2H_MAX];
					/* N2H node stats: includes node, n2h, pbuf in this order */
	uint32_t magic;
					/* Magic protection */
};

/*
 * Main NSS context structure (singleton)
 */
struct nss_top_instance {
	uint8_t num_nss;		/* Number of NSS cores supported */
	uint8_t num_phys_ports;		/* Number of physical ports supported */
	uint32_t clk_src;		/* Clock source: default/alternate */
	spinlock_t lock;		/* Big lock for NSS driver */
	spinlock_t stats_lock;		/* Statistics lock */
	struct dentry *top_dentry;	/* Top dentry for nss */
	struct dentry *stats_dentry;	/* Top dentry for nss stats */
	struct dentry *ipv4_dentry;	/* IPv4 stats dentry */
	struct dentry *ipv6_dentry;	/* IPv6 stats dentry */
	struct dentry *eth_rx_dentry;	/* ETH_RX stats dentry */
	struct dentry *n2h_dentry;	/* N2H stats dentry */
	struct dentry *drv_dentry;	/* HLOS driver stats dentry */
	struct dentry *pppoe_dentry;	/* PPPOE stats dentry */
	struct dentry *gmac_dentry;	/* GMAC ethnode stats dentry */
	struct nss_ctx_instance nss[NSS_MAX_CORES];
					/* NSS contexts */
	/*
	 * Network processing handler core ids (CORE0/CORE1) for various interfaces
	 */
	uint8_t phys_if_handler_id[NSS_MAX_PHYSICAL_INTERFACES];
	uint8_t virt_if_handler_id[NSS_MAX_VIRTUAL_INTERFACES];
	uint8_t shaping_handler_id;
	uint8_t ipv4_handler_id;
	uint8_t ipv6_handler_id;
	uint8_t crypto_handler_id;
	uint8_t ipsec_handler_id;
	uint8_t wlan_handler_id;
	uint8_t tun6rd_handler_id;
	uint8_t tunipip6_handler_id;
	uint8_t frequency_handler_id;

	/*
	 * Data/Message callbacks for various interfaces
	 */
	nss_phys_if_rx_callback_t if_rx_callback[NSS_MAX_NET_INTERFACES];
					/* Physical interface packet callback functions */
	nss_phys_if_msg_callback_t phys_if_msg_callback[NSS_MAX_PHYSICAL_INTERFACES];
					/* Physical interface event callback functions */
	nss_virt_if_msg_callback_t virt_if_msg_callback[NSS_MAX_VIRTUAL_INTERFACES];
					/* Virtual interface messsage callback functions */
	nss_ipv4_msg_callback_t ipv4_callback;
					/* IPv4 sync/establish callback function */
	nss_ipv6_msg_callback_t ipv6_callback;
					/* IPv6 sync/establish callback function */
	nss_ipsec_msg_callback_t ipsec_encap_callback;
	nss_ipsec_msg_callback_t ipsec_decap_callback;
					/* IPsec event callback function */
	nss_crypto_msg_callback_t crypto_msg_callback;
	nss_crypto_buf_callback_t crypto_buf_callback;
					/* crypto interface callback functions */
	nss_profiler_callback_t profiler_callback[NSS_MAX_CORES];
					/* Profiler interface callback function */
	nss_tun6rd_msg_callback_t tun6rd_msg_callback;
					/* 6rd tunnel interface event callback function */
	nss_tunipip6_msg_callback_t tunipip6_msg_callback;
					/* ipip6 tunnel interface event callback function */
	struct nss_shaper_bounce_registrant bounce_interface_registrants[NSS_MAX_NET_INTERFACES];
					/* Registrants for interface shaper bounce operations */
	struct nss_shaper_bounce_registrant bounce_bridge_registrants[NSS_MAX_NET_INTERFACES];
					/* Registrants for bridge shaper bounce operations */
	nss_lag_event_callback_t lag_event_callback;

	/*
	 * Interface contexts (non network device)
	 */
	void *ipv4_ctx;			/* IPv4 connection manager context */
	void *ipv6_ctx;			/* IPv6 connection manager context */
	void *crypto_ctx;		/* Crypto interface context */
	void *profiler_ctx[NSS_MAX_CORES];
					/* Profiler interface context */

	void *ipsec_encap_ctx;		/* IPsec encap context */
	void *ipsec_decap_ctx;		/* IPsec decap context */

	/*
	 * Interface contexts (network device)
	 */
	struct net_device *if_ctx[NSS_MAX_NET_INTERFACES];
					/* Phys/Virt interface context */

	/*
	 * Statistics for various interfaces
	 */
	uint64_t stats_ipv4[NSS_STATS_IPV4_MAX];
					/* IPv4 statistics */
	uint64_t stats_ipv6[NSS_STATS_IPV6_MAX];
					/* IPv6 statistics */
	uint64_t stats_drv[NSS_STATS_DRV_MAX];
					/* Hlos driver statistics */
	uint64_t stats_pppoe[NSS_STATS_PPPOE_MAX];
					/* PPPoE statistics */
	uint64_t stats_gmac[NSS_MAX_PHYSICAL_INTERFACES][NSS_STATS_GMAC_MAX];
					/* GMAC statistics */
	uint64_t stats_node[NSS_MAX_NET_INTERFACES][NSS_STATS_NODE_MAX];
					/* IPv4 statistics per interface */
	uint64_t stats_if_exception_eth_rx[NSS_EXCEPTION_EVENT_ETH_RX_MAX];
					/* Unknown protocol exception events per interface */
	uint64_t stats_if_exception_ipv4[NSS_EXCEPTION_EVENT_IPV4_MAX];
					/* IPv4 protocol exception events per interface */
	uint64_t stats_if_exception_ipv6[NSS_EXCEPTION_EVENT_IPV6_MAX];
					/* IPv6 protocol exception events per interface */
	uint64_t stats_if_exception_pppoe[NSS_MAX_PHYSICAL_INTERFACES][NSS_PPPOE_NUM_SESSION_PER_INTERFACE][NSS_EXCEPTION_EVENT_PPPOE_MAX];
					/* PPPoE exception events for per session on per interface */
	void *nss_fpb_base;			/* Virtual address of FPB base */
	bool nss_hal_common_init_done;

	/*
	 * TODO: Review and update following fields
	 */
	uint64_t last_rx_jiffies;	/* Time of the last RX message from the NA in jiffies */
};

#if (NSS_PKT_STATS_ENABLED == 1)
/*
 * nss_pkt_stats_increment()
 */
static inline void nss_pkt_stats_increment(struct nss_ctx_instance *nss_ctx, uint64_t *stat)
{
	spin_lock_bh(&nss_ctx->nss_top->stats_lock);
	*stat = *stat + 1;
	spin_unlock_bh(&nss_ctx->nss_top->stats_lock);
}
#endif

/*
 * NSS Statistics and Data for User Space
 */
struct nss_cmd_buffer {
	uint32_t current_freq;	/* Current Running Freq of NSS */
	int32_t auto_scale;	/* Enable or Disable auto_scale */
	int32_t max_freq;	/* Maximum supported frequency index value */
	uint32_t register_addr;	/* register addr buffer */
	uint32_t register_data;	/* register data buffer */
	uint32_t average_inst;	/* average of inst for nss core */
	uint32_t coredump;	/* cmd coredump buffer */
};

/*
 * NSS Core Statistics and Frequencies
 */
#define NSS_SAMPLE_BUFFER_SIZE 32	/* Ring Buffer should be a Size of two */
#define NSS_SAMPLE_BUFFER_MASK (NSS_SAMPLE_BUFFER_SIZE - 1)
#define NSS_MAX_CPU_SCALES 3			/* Max Number of Frequencies */
#define NSS_FREQUENCY_SCALE_RATE_LIMIT_UP 2	/* Adjust the Rate of Frequency Switching Up */
#define NSS_FREQUENCY_SCALE_RATE_LIMIT_DOWN 60000	/* Adjust the Rate of Frequency Switching Down */
#define NSS_MESSAGE_RATE_LIMIT 15000		/* Adjust the Rate of Displaying Statistic Messages */

/*
 * NSS Frequency Scale Info
 *
 * INFO: Contains the Scale information per Frequency
 *	Per Scale information needed to Program PLL and make switching decisions
 */
struct nss_scale_info {
	uint32_t frequency;	/* Frequency in Mhz */
	uint32_t minimum;	/* Minimum INST_CNT per Sec */
	uint32_t maximum;	/* Maximum INST_CNT per Sec */
};

/*
 * NSS Runtime Sample Structure
 *
 * INFO: Contains the runtime statistic of the NSS core
 *	Also contains the per frequency scale array
 */
struct nss_runtime_sampling {
	struct nss_scale_info freq_scale[NSS_MAX_CPU_SCALES];	/* NSS Max Scale Per Freq */
	uint32_t freq_scale_index;				/* Current Freq Index */
	uint32_t freq_scale_ready;				/* Allow Freq Scaling */
	uint32_t freq_scale_sup_max;				/* NSS Max Supported Scale - Limit Turbo */
	uint32_t freq_scale_rate_limit_up;			/* Scaling Change Rate Limit */
	uint32_t freq_scale_rate_limit_down;			/* Scaling Change Rate Limit */
	uint32_t buffer[NSS_SAMPLE_BUFFER_SIZE];		/* Sample Ring Buffer */
	uint32_t buffer_index;					/* Running Buffer Index */
	uint32_t sum;						/* Total INST_CNT SUM */
	uint32_t sample_count;					/* Number of Samples stored in Ring Buffer */
	uint32_t average;					/* Average of INST_CNT */
	uint32_t message_rate_limit;				/* Debug Message Rate Limit */
	uint32_t initialized;					/* Flag to check for adequate initial samples */
};


#if (NSS_DT_SUPPORT == 1)
/*
 * nss_feature_enabled
 */
enum nss_feature_enabled {
	NSS_FEATURE_NOT_ENABLED = 0,    /* Feature is not enabled on this core */
	NSS_FEATURE_ENABLED,            /* Feature is enabled on this core */
};

/*
 * nss_platform_data
 *      Platform data per core
 */
struct nss_platform_data {
	uint32_t id;		/* NSS core ID */
	uint32_t num_irq;       /* No. of interrupts supported per core */
	uint32_t irq[2];        /* IRQ numbers per interrupt */
	uint32_t nmap;          /* Virtual address of NSS CSM space */
	uint32_t vmap;          /* Virtual address of NSS virtual register map */
	uint32_t nphys;         /* Physical address of NSS CSM space */
	uint32_t vphys;         /* Physical address of NSS virtual register map */
	uint32_t rst_addr;      /* Reset address of NSS core */
	uint32_t load_addr;     /* Load address of NSS firmware */
	enum nss_feature_enabled turbo_frequency;	/* Does this core support turbo frequencies */
	enum nss_feature_enabled ipv4_enabled;		/* Does this core handle IPv4? */
	enum nss_feature_enabled ipv6_enabled;		/* Does this core handle IPv6? */
	enum nss_feature_enabled l2switch_enabled;	/* Does this core handle L2 switch? */
	enum nss_feature_enabled crypto_enabled;	/* Does this core handle crypto? */
	enum nss_feature_enabled ipsec_enabled;		/* Does this core handle IPsec? */
	enum nss_feature_enabled wlan_enabled;		/* Does this core handle WLAN 11ac? */
	enum nss_feature_enabled tun6rd_enabled;	/* Does this core handle 6rd Tunnel ? */
	enum nss_feature_enabled tunipip6_enabled;	/* Does this core handle ipip6 Tunnel ? */
	enum nss_feature_enabled shaping_enabled;	/* Does this core handle shaping ? */
	enum nss_feature_enabled gmac_enabled[4];	/* Does this core handle GMACs? */
};
#endif

/*
 * nss_core_log_msg_failures()
 *	Driver function for logging failed messages.
 */
static inline void nss_core_log_msg_failures(struct nss_ctx_instance *nss_ctx, struct nss_cmn_msg *ncm)
{
	if ((ncm->response == NSS_CMN_RESPONSE_ACK) || (ncm->response == NSS_CMM_RESPONSE_NOTIFY)) {
		return;
	}

	/*
	 * TODO: Is it worth doing value to name on these values?
	 */
	nss_warning("%p: msg failure - interface: %d, type: %d, response: %d, error: %d",
		nss_ctx, ncm->interface, ncm->type, ncm->response, ncm->error);
}

/*
 * NSS workqueue to change frequencies
 */
typedef struct {
	struct work_struct my_work;	/* Work Structure */
	uint32_t frequency;		/* Frequency To Change */
	uint32_t stats_enable;		/* Auto scale on/off */
} nss_work_t;

/*
 * CB function declarations
 */
typedef void (*nss_core_rx_callback_t)(struct nss_ctx_instance *, struct nss_cmn_msg *, void *);

/*
 * APIs provided by nss_core.c
 */
extern int nss_core_handle_napi(struct napi_struct *napi, int budget);
extern int32_t nss_core_send_buffer(struct nss_ctx_instance *nss_ctx, uint32_t if_num,
					struct sk_buff *nbuf, uint16_t qid,
					uint8_t buffer_type, uint16_t flags);
extern int32_t nss_core_send_crypto(struct nss_ctx_instance *nss_ctx, void *buf, uint32_t buf_paddr, uint16_t len);
extern void nss_wq_function( struct work_struct *work);
extern uint32_t nss_core_register_handler(uint32_t interface, nss_core_rx_callback_t cb, void *app_data);

/*
 * APIs provided by nss_tx_rx.c
 */
extern void nss_rx_handle_status_pkt(struct nss_ctx_instance *nss_ctx, struct sk_buff *nbuf);
extern void nss_crypto_buf_handler(struct nss_ctx_instance *nss_ctx, void *buf, uint32_t paddr, uint16_t len);
/*
 * APIs provided by nss_stats.c
 */
extern void nss_stats_init(void);
extern void nss_stats_clean(void);
#endif /* __NSS_CORE_H */
