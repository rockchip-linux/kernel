/*
 * HND generic packet pool operation primitives
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
 * $Id: hnd_pktpool.h 633941 2016-04-26 07:04:26Z $
 */

#ifndef _hnd_pktpool_h_
#define _hnd_pktpool_h_

#include <osl_ext.h>

#ifdef __cplusplus
extern "C" {
#endif // endif

/* mutex macros for thread safe */
#ifdef HND_PKTPOOL_THREAD_SAFE
#define HND_PKTPOOL_MUTEX_DECL(mutex)		OSL_EXT_MUTEX_DECL(mutex)
#else
#define HND_PKTPOOL_MUTEX_DECL(mutex)
#endif // endif

#ifdef BCMPKTPOOL
#define POOL_ENAB(pool)		((pool) && (pool)->inited)
#else /* BCMPKTPOOL */
#define POOL_ENAB(bus)		0
#endif /* BCMPKTPOOL */

#ifndef PKTPOOL_LEN_MAX
#define PKTPOOL_LEN_MAX		40
#endif /* PKTPOOL_LEN_MAX */
#define PKTPOOL_CB_MAX		3
#define PKTPOOL_CB_MAX_AVL	4

/* REMOVE_RXCPLID is an arg for pktpool callback function for removing rxcplID
 * and host addr associated with the rxfrag or shared pool buffer during pktpool_reclaim().
 */
#define REMOVE_RXCPLID            2

/* forward declaration */
struct pktpool;

typedef void (*pktpool_cb_t)(struct pktpool *pool, void *arg);
typedef struct {
	pktpool_cb_t cb;
	void *arg;
	uint8 refcnt;
} pktpool_cbinfo_t;

/** PCIe SPLITRX related: call back fn extension to populate host address in pool pkt */
typedef int (*pktpool_cb_extn_t)(struct pktpool *pool, void *arg1, void* pkt, int arg2);
typedef struct {
	pktpool_cb_extn_t cb;
	void *arg;
} pktpool_cbextn_info_t;

#ifdef BCMDBG_POOL
/* pkt pool debug states */
#define POOL_IDLE	0
#define POOL_RXFILL	1
#define POOL_RXDH	2
#define POOL_RXD11	3
#define POOL_TXDH	4
#define POOL_TXD11	5
#define POOL_AMPDU	6
#define POOL_TXENQ	7

typedef struct {
	void *p;
	uint32 cycles;
	uint32 dur;
} pktpool_dbg_t;

typedef struct {
	uint8 txdh;	/* tx to host */
	uint8 txd11;	/* tx to d11 */
	uint8 enq;	/* waiting in q */
	uint8 rxdh;	/* rx from host */
	uint8 rxd11;	/* rx from d11 */
	uint8 rxfill;	/* dma_rxfill */
	uint8 idle;	/* avail in pool */
} pktpool_stats_t;
#endif /* BCMDBG_POOL */

typedef struct pktpool {
	bool inited;            /**< pktpool_init was successful */
	uint8 type;             /**< type of lbuf: basic, frag, etc */
	uint8 id;               /**< pktpool ID:  index in registry */
	bool istx;              /**< direction: transmit or receive data path */
	HND_PKTPOOL_MUTEX_DECL(mutex)	/**< thread-safe mutex */

	void * freelist;        /**< free list: see PKTNEXTFREE(), PKTSETNEXTFREE() */
	uint16 avail;           /**< number of packets in pool's free list */
	uint16 n_pkts;             /**< number of packets managed by pool */
	uint16 maxlen;          /**< maximum size of pool <= PKTPOOL_LEN_MAX */
	uint16 max_pkt_bytes;   /**< size of pkt buffer in [bytes], excluding lbuf|lbuf_frag */

	bool empty;
	uint8 cbtoggle;
	uint8 cbcnt;
	uint8 ecbcnt;
	uint8 emptycb_disable;	/**< Value of type enum pktpool_empty_cb_state */
	pktpool_cbinfo_t *availcb_excl;
	pktpool_cbinfo_t cbs[PKTPOOL_CB_MAX_AVL];
	pktpool_cbinfo_t ecbs[PKTPOOL_CB_MAX];
	pktpool_cbextn_info_t cbext;	/**< PCIe SPLITRX related */
	pktpool_cbextn_info_t rxcplidfn;
#ifdef BCMDBG_POOL
	uint8 dbg_cbcnt;
	pktpool_cbinfo_t dbg_cbs[PKTPOOL_CB_MAX];
	uint16 dbg_qlen;
	pktpool_dbg_t dbg_q[PKTPOOL_LEN_MAX + 1];
#endif // endif
	pktpool_cbinfo_t dmarxfill;
} pktpool_t;

pktpool_t *get_pktpools_registry(int id);

/* Incarnate a pktpool registry. On success returns total_pools. */
extern int pktpool_attach(osl_t *osh, uint32 total_pools);
extern int pktpool_dettach(osl_t *osh); /* Relinquish registry */

extern int pktpool_init(osl_t *osh, pktpool_t *pktp, int *n_pkts, int max_pkt_bytes, bool istx,
	uint8 type);
extern int pktpool_deinit(osl_t *osh, pktpool_t *pktp);
extern int pktpool_fill(osl_t *osh, pktpool_t *pktp, bool minimal);
extern int pktpool_empty(osl_t *osh, pktpool_t *pktp);
extern uint16 pktpool_reclaim(osl_t *osh, pktpool_t *pktp, uint16 free_cnt);
extern void* pktpool_get(pktpool_t *pktp);
extern void pktpool_free(pktpool_t *pktp, void *p);
extern int pktpool_add(pktpool_t *pktp, void *p);
extern int pktpool_avail_notify_normal(osl_t *osh, pktpool_t *pktp);
extern int pktpool_avail_notify_exclusive(osl_t *osh, pktpool_t *pktp, pktpool_cb_t cb);
extern int pktpool_avail_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg);
extern int pktpool_avail_deregister(pktpool_t *pktp, pktpool_cb_t cb, void *arg);
extern int pktpool_empty_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg);
extern int pktpool_setmaxlen(pktpool_t *pktp, uint16 max_pkts);
extern int pktpool_setmaxlen_strict(osl_t *osh, pktpool_t *pktp, uint16 max_pkts);
extern void pktpool_emptycb_disable(pktpool_t *pktp, bool disable);
extern bool pktpool_emptycb_disabled(pktpool_t *pktp);
extern int pktpool_hostaddr_fill_register(pktpool_t *pktp, pktpool_cb_extn_t cb, void *arg1);
extern int pktpool_rxcplid_fill_register(pktpool_t *pktp, pktpool_cb_extn_t cb, void *arg);
extern void pktpool_invoke_dmarxfill(pktpool_t *pktp);
extern int pkpool_haddr_avail_register_cb(pktpool_t *pktp, pktpool_cb_t cb, void *arg);

#define POOLPTR(pp)         ((pktpool_t *)(pp))
#define POOLID(pp)          (POOLPTR(pp)->id)

#define POOLSETID(pp, ppid) (POOLPTR(pp)->id = (ppid))

#define pktpool_tot_pkts(pp)  (POOLPTR(pp)->n_pkts)   /**< n_pkts = avail + in_use <= max_pkts */
#define pktpool_avail(pp)   (POOLPTR(pp)->avail)
#define pktpool_max_pkt_bytes(pp)    (POOLPTR(pp)->max_pkt_bytes)
#define pktpool_max_pkts(pp)  (POOLPTR(pp)->maxlen)

/*
 * ----------------------------------------------------------------------------
 * A pool ID is assigned with a pkt pool during pool initialization. This is
 * done by maintaining a registry of all initialized pools, and the registry
 * index at which the pool is registered is used as the pool's unique ID.
 * ID 0 is reserved and is used to signify an invalid pool ID.
 * All packets henceforth allocated from a pool will be tagged with the pool's
 * unique ID. Packets allocated from the heap will use the reserved ID = 0.
 * Packets with non-zero pool id signify that they were allocated from a pool.
 * A maximum of 15 pools are supported, allowing a 4bit pool ID to be used
 * in place of a 32bit pool pointer in each packet.
 * ----------------------------------------------------------------------------
 */
#define PKTPOOL_INVALID_ID          (0)
#define PKTPOOL_MAXIMUM_ID          (15)

/* Registry of pktpool(s) */
/* Pool ID to/from Pool Pointer converters */
#define PKTPOOL_ID2PTR(id)          (get_pktpools_registry(id))
#define PKTPOOL_PTR2ID(pp)          (POOLID(pp))

#ifdef BCMDBG_POOL
extern int pktpool_dbg_register(pktpool_t *pktp, pktpool_cb_t cb, void *arg);
extern int pktpool_start_trigger(pktpool_t *pktp, void *p);
extern int pktpool_dbg_dump(pktpool_t *pktp);
extern int pktpool_dbg_notify(pktpool_t *pktp);
extern int pktpool_stats_dump(pktpool_t *pktp, pktpool_stats_t *stats);
#endif /* BCMDBG_POOL */

#ifdef BCMPKTPOOL
#define SHARED_POOL		(pktpool_shared)
extern pktpool_t *pktpool_shared;
#ifdef BCMFRAGPOOL
#define SHARED_FRAG_POOL	(pktpool_shared_lfrag)
extern pktpool_t *pktpool_shared_lfrag;
#endif // endif

#ifdef BCMRESVFRAGPOOL
#define RESV_FRAG_POOL		(pktpool_resv_lfrag)
#define RESV_POOL_INFO		(resv_pool_info)
#else
#define RESV_FRAG_POOL		((struct pktpool *)NULL)
#define RESV_POOL_INFO		(NULL)
#endif /* BCMRESVFRAGPOOL */

/** PCIe SPLITRX related */
#define SHARED_RXFRAG_POOL	(pktpool_shared_rxlfrag)
extern pktpool_t *pktpool_shared_rxlfrag;

int hnd_pktpool_init(osl_t *osh);
int hnd_pktpool_fill(pktpool_t *pktpool, bool minimal);
void hnd_pktpool_refill(bool minimal);
#ifdef BCMRESVFRAGPOOL
extern pktpool_t *pktpool_resv_lfrag;
extern struct resv_info *resv_pool_info;
#endif /* BCMRESVFRAGPOOL */
#else /* BCMPKTPOOL */
#define SHARED_POOL		((struct pktpool *)NULL)
#endif /* BCMPKTPOOL */

#ifdef __cplusplus
	}
#endif // endif

#endif /* _hnd_pktpool_h_ */
