// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Copyright (C) 2017-2022 Jason A. Donenfeld <Jason@zx2c4.com>. All Rights Reserved.
 * Copyright Matt Mackall <mpm@selenic.com>, 2003, 2004, 2005
 * Copyright Theodore Ts'o, 1994, 1995, 1996, 1997, 1998, 1999. All rights reserved.
 *
 * This driver produces cryptographically secure pseudorandom data. It is divided
 * into roughly six sections, each with a section header:
 *
 *   - Initialization and readiness waiting.
 *   - Fast key erasure RNG, the "crng".
 *   - Entropy accumulation and extraction routines.
 *   - Entropy collection routines.
 *   - Userspace reader/writer interfaces.
 *   - Sysctl interface.
 *
 * The high level overview is that there is one input pool, into which
 * various pieces of data are hashed. Prior to initialization, some of that
 * data is then "credited" as having a certain number of bits of entropy.
 * When enough bits of entropy are available, the hash is finalized and
 * handed as a key to a stream cipher that expands it indefinitely for
 * various consumers. This key is periodically refreshed as the various
 * entropy collectors, described below, add data to the input pool.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/utsname.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/nodemask.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/ptrace.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/ratelimit.h>
#include <linux/syscalls.h>
#include <linux/completion.h>
#include <linux/uuid.h>
#include <linux/uaccess.h>
#include <linux/siphash.h>
#include <linux/uio.h>
#include <crypto/chacha.h>
#include <crypto/blake2s.h>
#include <asm/processor.h>
#include <asm/irq.h>
#include <asm/irq_regs.h>
#include <asm/io.h>

// GKI: Keep this header to retain the original CRC that previously used the
// random.h tracepoints.
#include <linux/writeback.h>

/*********************************************************************
 *
 * Initialization and readiness waiting.
 *
 * Much of the RNG infrastructure is devoted to various dependencies
 * being able to wait until the RNG has collected enough entropy and
 * is ready for safe consumption.
 *
 *********************************************************************/

/*
 * crng_init is protected by base_crng->lock, and only increases
 * its value (from empty->early->ready).
 */
static enum {
	CRNG_EMPTY = 0, /* Little to no entropy collected */
	CRNG_EARLY = 1, /* At least POOL_EARLY_BITS collected */
	CRNG_READY = 2  /* Fully initialized with POOL_READY_BITS collected */
} crng_init __read_mostly = CRNG_EMPTY;
#define crng_ready() (likely(crng_init >= CRNG_READY))
/* Various types of waiters for crng_init->CRNG_READY transition. */
static DECLARE_WAIT_QUEUE_HEAD(crng_init_wait);
static struct fasync_struct *fasync;
static DEFINE_SPINLOCK(random_ready_chain_lock);
static RAW_NOTIFIER_HEAD(random_ready_chain);

/* Control how we warn userspace. */
static struct ratelimit_state urandom_warning =
	RATELIMIT_STATE_INIT_FLAGS("urandom_warning", HZ, 3, RATELIMIT_MSG_ON_RELEASE);
static int ratelimit_disable __read_mostly =
	IS_ENABLED(CONFIG_WARN_ALL_UNSEEDED_RANDOM);
module_param_named(ratelimit_disable, ratelimit_disable, int, 0644);
MODULE_PARM_DESC(ratelimit_disable, "Disable random ratelimit suppression");

/*
 * Returns whether or not the input pool has been seeded and thus guaranteed
 * to supply cryptographically secure random numbers. This applies to: the
 * /dev/urandom device, the get_random_bytes function, and the get_random_{u32,
 * ,u64,int,long} family of functions.
 *
 * Returns: true if the input pool has been seeded.
 *          false if the input pool has not been seeded.
 */
bool rng_is_initialized(void)
{
	return crng_ready();
}
EXPORT_SYMBOL(rng_is_initialized);

/* Used by wait_for_random_bytes(), and considered an entropy collector, below. */
static void try_to_generate_entropy(void);

/*
 * Wait for the input pool to be seeded and thus guaranteed to supply
 * cryptographically secure random numbers. This applies to: the /dev/urandom
 * device, the get_random_bytes function, and the get_random_{u32,u64,int,long}
 * family of functions. Using any of these functions without first calling
 * this function forfeits the guarantee of security.
 *
 * Returns: 0 if the input pool has been seeded.
 *          -ERESTARTSYS if the function was interrupted by a signal.
 */
int wait_for_random_bytes(void)
{
	while (!crng_ready()) {
		int ret;

		try_to_generate_entropy();
		ret = wait_event_interruptible_timeout(crng_init_wait, crng_ready(), HZ);
		if (ret)
			return ret > 0 ? 0 : ret;
	}
	return 0;
}
EXPORT_SYMBOL(wait_for_random_bytes);

/*
 * Add a callback function that will be invoked when the input
 * pool is initialised.
 *
 * returns: 0 if callback is successfully added
 *	    -EALREADY if pool is already initialised (callback not called)
 */
int __cold register_random_ready_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret = -EALREADY;

	if (crng_ready())
		return ret;

	spin_lock_irqsave(&random_ready_chain_lock, flags);
	if (!crng_ready())
		ret = raw_notifier_chain_register(&random_ready_chain, nb);
	spin_unlock_irqrestore(&random_ready_chain_lock, flags);
	return ret;
}

/*
 * Delete a previously registered readiness callback function.
 */
int __cold unregister_random_ready_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&random_ready_chain_lock, flags);
	ret = raw_notifier_chain_unregister(&random_ready_chain, nb);
	spin_unlock_irqrestore(&random_ready_chain_lock, flags);
	return ret;
}

static void process_oldschool_random_ready_list(void);
static void __cold process_random_ready_list(void)
{
	unsigned long flags;

	spin_lock_irqsave(&random_ready_chain_lock, flags);
	raw_notifier_call_chain(&random_ready_chain, 0, NULL);
	spin_unlock_irqrestore(&random_ready_chain_lock, flags);

	process_oldschool_random_ready_list();
}

#define warn_unseeded_randomness() \
	if (IS_ENABLED(CONFIG_WARN_ALL_UNSEEDED_RANDOM) && !crng_ready()) \
		printk_deferred(KERN_NOTICE "random: %s called from %pS with crng_init=%d\n", \
				__func__, (void *)_RET_IP_, crng_init)


/*********************************************************************
 *
 * Fast key erasure RNG, the "crng".
 *
 * These functions expand entropy from the entropy extractor into
 * long streams for external consumption using the "fast key erasure"
 * RNG described at <https://blog.cr.yp.to/20170723-random.html>.
 *
 * There are a few exported interfaces for use by other drivers:
 *
 *	void get_random_bytes(void *buf, size_t len)
 *	u32 get_random_u32()
 *	u64 get_random_u64()
 *	unsigned int get_random_int()
 *	unsigned long get_random_long()
 *
 * These interfaces will return the requested number of random bytes
 * into the given buffer or as a return value. This is equivalent to
 * a read from /dev/urandom. The u32, u64, int, and long family of
 * functions may be higher performance for one-off random integers,
 * because they do a bit of buffering and do not invoke reseeding
 * until the buffer is emptied.
 *
 *********************************************************************/

enum {
	CRNG_RESEED_START_INTERVAL = HZ,
	CRNG_RESEED_INTERVAL = 60 * HZ
};

static struct {
	u8 key[CHACHA_KEY_SIZE] __aligned(__alignof__(long));
	unsigned long birth;
	unsigned long generation;
	spinlock_t lock;
} base_crng = {
	.lock = __SPIN_LOCK_UNLOCKED(base_crng.lock)
};

struct crng {
	u8 key[CHACHA_KEY_SIZE];
	unsigned long generation;
	local_lock_t lock;
};

static DEFINE_PER_CPU(struct crng, crngs) = {
	.generation = ULONG_MAX,
	.lock = INIT_LOCAL_LOCK(crngs.lock),
};

/* Used by crng_reseed() and crng_make_state() to extract a new seed from the input pool. */
static void extract_entropy(void *buf, size_t len);

/* This extracts a new crng key from the input pool. */
static void crng_reseed(void)
{
	unsigned long flags;
	unsigned long next_gen;
	u8 key[CHACHA_KEY_SIZE];

	extract_entropy(key, sizeof(key));

	/*
	 * We copy the new key into the base_crng, overwriting the old one,
	 * and update the generation counter. We avoid hitting ULONG_MAX,
	 * because the per-cpu crngs are initialized to ULONG_MAX, so this
	 * forces new CPUs that come online to always initialize.
	 */
	spin_lock_irqsave(&base_crng.lock, flags);
	memcpy(base_crng.key, key, sizeof(base_crng.key));
	next_gen = base_crng.generation + 1;
	if (next_gen == ULONG_MAX)
		++next_gen;
	WRITE_ONCE(base_crng.generation, next_gen);
	WRITE_ONCE(base_crng.birth, jiffies);
	if (!crng_ready())
		crng_init = CRNG_READY;
	spin_unlock_irqrestore(&base_crng.lock, flags);
	memzero_explicit(key, sizeof(key));
}

/*
 * This generates a ChaCha block using the provided key, and then
 * immediately overwites that key with half the block. It returns
 * the resultant ChaCha state to the user, along with the second
 * half of the block containing 32 bytes of random data that may
 * be used; random_data_len may not be greater than 32.
 *
 * The returned ChaCha state contains within it a copy of the old
 * key value, at index 4, so the state should always be zeroed out
 * immediately after using in order to maintain forward secrecy.
 * If the state cannot be erased in a timely manner, then it is
 * safer to set the random_data parameter to &chacha_state[4] so
 * that this function overwrites it before returning.
 */
static void crng_fast_key_erasure(u8 key[CHACHA_KEY_SIZE],
				  u32 chacha_state[CHACHA_STATE_WORDS],
				  u8 *random_data, size_t random_data_len)
{
	u8 first_block[CHACHA_BLOCK_SIZE];

	BUG_ON(random_data_len > 32);

	chacha_init_consts(chacha_state);
	memcpy(&chacha_state[4], key, CHACHA_KEY_SIZE);
	memset(&chacha_state[12], 0, sizeof(u32) * 4);
	chacha20_block(chacha_state, first_block);

	memcpy(key, first_block, CHACHA_KEY_SIZE);
	memcpy(random_data, first_block + CHACHA_KEY_SIZE, random_data_len);
	memzero_explicit(first_block, sizeof(first_block));
}

/*
 * Return whether the crng seed is considered to be sufficiently old
 * that a reseeding is needed. This happens if the last reseeding
 * was CRNG_RESEED_INTERVAL ago, or during early boot, at an interval
 * proportional to the uptime.
 */
static bool crng_has_old_seed(void)
{
	static bool early_boot = true;
	unsigned long interval = CRNG_RESEED_INTERVAL;

	if (unlikely(READ_ONCE(early_boot))) {
		time64_t uptime = ktime_get_seconds();
		if (uptime >= CRNG_RESEED_INTERVAL / HZ * 2)
			WRITE_ONCE(early_boot, false);
		else
			interval = max_t(unsigned int, CRNG_RESEED_START_INTERVAL,
					 (unsigned int)uptime / 2 * HZ);
	}
	return time_is_before_jiffies(READ_ONCE(base_crng.birth) + interval);
}

/*
 * This function returns a ChaCha state that you may use for generating
 * random data. It also returns up to 32 bytes on its own of random data
 * that may be used; random_data_len may not be greater than 32.
 */
static void crng_make_state(u32 chacha_state[CHACHA_STATE_WORDS],
			    u8 *random_data, size_t random_data_len)
{
	unsigned long flags;
	struct crng *crng;

	BUG_ON(random_data_len > 32);

	/*
	 * For the fast path, we check whether we're ready, unlocked first, and
	 * then re-check once locked later. In the case where we're really not
	 * ready, we do fast key erasure with the base_crng directly, extracting
	 * when crng_init is CRNG_EMPTY.
	 */
	if (!crng_ready()) {
		bool ready;

		spin_lock_irqsave(&base_crng.lock, flags);
		ready = crng_ready();
		if (!ready) {
			if (crng_init == CRNG_EMPTY)
				extract_entropy(base_crng.key, sizeof(base_crng.key));
			crng_fast_key_erasure(base_crng.key, chacha_state,
					      random_data, random_data_len);
		}
		spin_unlock_irqrestore(&base_crng.lock, flags);
		if (!ready)
			return;
	}

	/*
	 * If the base_crng is old enough, we reseed, which in turn bumps the
	 * generation counter that we check below.
	 */
	if (unlikely(crng_has_old_seed()))
		crng_reseed();

	local_lock_irqsave(&crngs.lock, flags);
	crng = raw_cpu_ptr(&crngs);

	/*
	 * If our per-cpu crng is older than the base_crng, then it means
	 * somebody reseeded the base_crng. In that case, we do fast key
	 * erasure on the base_crng, and use its output as the new key
	 * for our per-cpu crng. This brings us up to date with base_crng.
	 */
	if (unlikely(crng->generation != READ_ONCE(base_crng.generation))) {
		spin_lock(&base_crng.lock);
		crng_fast_key_erasure(base_crng.key, chacha_state,
				      crng->key, sizeof(crng->key));
		crng->generation = base_crng.generation;
		spin_unlock(&base_crng.lock);
	}

	/*
	 * Finally, when we've made it this far, our per-cpu crng has an up
	 * to date key, and we can do fast key erasure with it to produce
	 * some random data and a ChaCha state for the caller. All other
	 * branches of this function are "unlikely", so most of the time we
	 * should wind up here immediately.
	 */
	crng_fast_key_erasure(crng->key, chacha_state, random_data, random_data_len);
	local_unlock_irqrestore(&crngs.lock, flags);
}

static void _get_random_bytes(void *buf, size_t len)
{
	u32 chacha_state[CHACHA_STATE_WORDS];
	u8 tmp[CHACHA_BLOCK_SIZE];
	size_t first_block_len;

	if (!len)
		return;

	first_block_len = min_t(size_t, 32, len);
	crng_make_state(chacha_state, buf, first_block_len);
	len -= first_block_len;
	buf += first_block_len;

	while (len) {
		if (len < CHACHA_BLOCK_SIZE) {
			chacha20_block(chacha_state, tmp);
			memcpy(buf, tmp, len);
			memzero_explicit(tmp, sizeof(tmp));
			break;
		}

		chacha20_block(chacha_state, buf);
		if (unlikely(chacha_state[12] == 0))
			++chacha_state[13];
		len -= CHACHA_BLOCK_SIZE;
		buf += CHACHA_BLOCK_SIZE;
	}

	memzero_explicit(chacha_state, sizeof(chacha_state));
}

/*
 * This function is the exported kernel interface.  It returns some
 * number of good random numbers, suitable for key generation, seeding
 * TCP sequence numbers, etc.  It does not rely on the hardware random
 * number generator.  For random bytes direct from the hardware RNG
 * (when available), use get_random_bytes_arch(). In order to ensure
 * that the randomness provided by this function is okay, the function
 * wait_for_random_bytes() should be called and return 0 at least once
 * at any point prior.
 */
void get_random_bytes(void *buf, int len)
{
	warn_unseeded_randomness();
	_get_random_bytes(buf, len);
}
EXPORT_SYMBOL(get_random_bytes);

static ssize_t get_random_bytes_user(struct iov_iter *iter)
{
	u32 chacha_state[CHACHA_STATE_WORDS];
	u8 block[CHACHA_BLOCK_SIZE];
	size_t ret = 0, copied;

	if (unlikely(!iov_iter_count(iter)))
		return 0;

	/*
	 * Immediately overwrite the ChaCha key at index 4 with random
	 * bytes, in case userspace causes copy_to_iter() below to sleep
	 * forever, so that we still retain forward secrecy in that case.
	 */
	crng_make_state(chacha_state, (u8 *)&chacha_state[4], CHACHA_KEY_SIZE);
	/*
	 * However, if we're doing a read of len <= 32, we don't need to
	 * use chacha_state after, so we can simply return those bytes to
	 * the user directly.
	 */
	if (iov_iter_count(iter) <= CHACHA_KEY_SIZE) {
		ret = copy_to_iter(&chacha_state[4], CHACHA_KEY_SIZE, iter);
		goto out_zero_chacha;
	}

	for (;;) {
		chacha20_block(chacha_state, block);
		if (unlikely(chacha_state[12] == 0))
			++chacha_state[13];

		copied = copy_to_iter(block, sizeof(block), iter);
		ret += copied;
		if (!iov_iter_count(iter) || copied != sizeof(block))
			break;

		BUILD_BUG_ON(PAGE_SIZE % sizeof(block) != 0);
		if (ret % PAGE_SIZE == 0) {
			if (signal_pending(current))
				break;
			cond_resched();
		}
	}

	memzero_explicit(block, sizeof(block));
out_zero_chacha:
	memzero_explicit(chacha_state, sizeof(chacha_state));
	return ret ? ret : -EFAULT;
}

/*
 * Batched entropy returns random integers. The quality of the random
 * number is good as /dev/urandom. In order to ensure that the randomness
 * provided by this function is okay, the function wait_for_random_bytes()
 * should be called and return 0 at least once at any point prior.
 */

#define DEFINE_BATCHED_ENTROPY(type)						\
struct batch_ ##type {								\
	/*									\
	 * We make this 1.5x a ChaCha block, so that we get the			\
	 * remaining 32 bytes from fast key erasure, plus one full		\
	 * block from the detached ChaCha state. We can increase		\
	 * the size of this later if needed so long as we keep the		\
	 * formula of (integer_blocks + 0.5) * CHACHA_BLOCK_SIZE.		\
	 */									\
	type entropy[CHACHA_BLOCK_SIZE * 3 / (2 * sizeof(type))];		\
	local_lock_t lock;							\
	unsigned long generation;						\
	unsigned int position;							\
};										\
										\
static DEFINE_PER_CPU(struct batch_ ##type, batched_entropy_ ##type) = {	\
	.lock = INIT_LOCAL_LOCK(batched_entropy_ ##type.lock),			\
	.position = UINT_MAX							\
};										\
										\
type get_random_ ##type(void)							\
{										\
	type ret;								\
	unsigned long flags;							\
	struct batch_ ##type *batch;						\
	unsigned long next_gen;							\
										\
	warn_unseeded_randomness();						\
										\
	if  (!crng_ready()) {							\
		_get_random_bytes(&ret, sizeof(ret));				\
		return ret;							\
	}									\
										\
	local_lock_irqsave(&batched_entropy_ ##type.lock, flags);		\
	batch = raw_cpu_ptr(&batched_entropy_##type);				\
										\
	next_gen = READ_ONCE(base_crng.generation);				\
	if (batch->position >= ARRAY_SIZE(batch->entropy) ||			\
	    next_gen != batch->generation) {					\
		_get_random_bytes(batch->entropy, sizeof(batch->entropy));	\
		batch->position = 0;						\
		batch->generation = next_gen;					\
	}									\
										\
	ret = batch->entropy[batch->position];					\
	batch->entropy[batch->position] = 0;					\
	++batch->position;							\
	local_unlock_irqrestore(&batched_entropy_ ##type.lock, flags);		\
	return ret;								\
}										\
EXPORT_SYMBOL(get_random_ ##type);

DEFINE_BATCHED_ENTROPY(u64)
DEFINE_BATCHED_ENTROPY(u32)

#ifdef CONFIG_SMP
/*
 * This function is called when the CPU is coming up, with entry
 * CPUHP_RANDOM_PREPARE, which comes before CPUHP_WORKQUEUE_PREP.
 */
int __cold random_prepare_cpu(unsigned int cpu)
{
	/*
	 * When the cpu comes back online, immediately invalidate both
	 * the per-cpu crng and all batches, so that we serve fresh
	 * randomness.
	 */
	per_cpu_ptr(&crngs, cpu)->generation = ULONG_MAX;
	per_cpu_ptr(&batched_entropy_u32, cpu)->position = UINT_MAX;
	per_cpu_ptr(&batched_entropy_u64, cpu)->position = UINT_MAX;
	return 0;
}
#endif

/*
 * This function will use the architecture-specific hardware random
 * number generator if it is available. It is not recommended for
 * use. Use get_random_bytes() instead. It returns the number of
 * bytes filled in.
 */
int __must_check get_random_bytes_arch(void *buf, int len)
{
	size_t left = len;
	u8 *p = buf;

	while (left) {
		unsigned long v;
		size_t block_len = min_t(size_t, left, sizeof(unsigned long));

		if (!arch_get_random_long(&v))
			break;

		memcpy(p, &v, block_len);
		p += block_len;
		left -= block_len;
	}

	return len - left;
}
EXPORT_SYMBOL(get_random_bytes_arch);


/**********************************************************************
 *
 * Entropy accumulation and extraction routines.
 *
 * Callers may add entropy via:
 *
 *     static void mix_pool_bytes(const void *buf, size_t len)
 *
 * After which, if added entropy should be credited:
 *
 *     static void credit_init_bits(size_t bits)
 *
 * Finally, extract entropy via:
 *
 *     static void extract_entropy(void *buf, size_t len)
 *
 **********************************************************************/

enum {
	POOL_BITS = BLAKE2S_HASH_SIZE * 8,
	POOL_READY_BITS = POOL_BITS, /* When crng_init->CRNG_READY */
	POOL_EARLY_BITS = POOL_READY_BITS / 2 /* When crng_init->CRNG_EARLY */
};

static struct {
	struct blake2s_state hash;
	spinlock_t lock;
	unsigned int init_bits;
} input_pool = {
	.hash.h = { BLAKE2S_IV0 ^ (0x01010000 | BLAKE2S_HASH_SIZE),
		    BLAKE2S_IV1, BLAKE2S_IV2, BLAKE2S_IV3, BLAKE2S_IV4,
		    BLAKE2S_IV5, BLAKE2S_IV6, BLAKE2S_IV7 },
	.hash.outlen = BLAKE2S_HASH_SIZE,
	.lock = __SPIN_LOCK_UNLOCKED(input_pool.lock),
};

static void _mix_pool_bytes(const void *buf, size_t len)
{
	blake2s_update(&input_pool.hash, buf, len);
}

/*
 * This function adds bytes into the input pool. It does not
 * update the initialization bit counter; the caller should call
 * credit_init_bits if this is appropriate.
 */
static void mix_pool_bytes(const void *buf, size_t len)
{
	unsigned long flags;

	spin_lock_irqsave(&input_pool.lock, flags);
	_mix_pool_bytes(buf, len);
	spin_unlock_irqrestore(&input_pool.lock, flags);
}

/*
 * This is an HKDF-like construction for using the hashed collected entropy
 * as a PRF key, that's then expanded block-by-block.
 */
static void extract_entropy(void *buf, size_t len)
{
	unsigned long flags;
	u8 seed[BLAKE2S_HASH_SIZE], next_key[BLAKE2S_HASH_SIZE];
	struct {
		unsigned long rdseed[32 / sizeof(long)];
		size_t counter;
	} block;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(block.rdseed); ++i) {
		if (!arch_get_random_seed_long(&block.rdseed[i]) &&
		    !arch_get_random_long(&block.rdseed[i]))
			block.rdseed[i] = random_get_entropy();
	}

	spin_lock_irqsave(&input_pool.lock, flags);

	/* seed = HASHPRF(last_key, entropy_input) */
	blake2s_final(&input_pool.hash, seed);

	/* next_key = HASHPRF(seed, RDSEED || 0) */
	block.counter = 0;
	blake2s(next_key, (u8 *)&block, seed, sizeof(next_key), sizeof(block), sizeof(seed));
	blake2s_init_key(&input_pool.hash, BLAKE2S_HASH_SIZE, next_key, sizeof(next_key));

	spin_unlock_irqrestore(&input_pool.lock, flags);
	memzero_explicit(next_key, sizeof(next_key));

	while (len) {
		i = min_t(size_t, len, BLAKE2S_HASH_SIZE);
		/* output = HASHPRF(seed, RDSEED || ++counter) */
		++block.counter;
		blake2s(buf, (u8 *)&block, seed, i, sizeof(block), sizeof(seed));
		len -= i;
		buf += i;
	}

	memzero_explicit(seed, sizeof(seed));
	memzero_explicit(&block, sizeof(block));
}

#define credit_init_bits(bits) if (!crng_ready()) _credit_init_bits(bits)

static void __cold _credit_init_bits(size_t bits)
{
	unsigned int new, orig, add;
	unsigned long flags;

	if (!bits)
		return;

	add = min_t(size_t, bits, POOL_BITS);

	do {
		orig = READ_ONCE(input_pool.init_bits);
		new = min_t(unsigned int, POOL_BITS, orig + add);
	} while (cmpxchg(&input_pool.init_bits, orig, new) != orig);

	if (orig < POOL_READY_BITS && new >= POOL_READY_BITS) {
		crng_reseed(); /* Sets crng_init to CRNG_READY under base_crng.lock. */
		process_random_ready_list();
		wake_up_interruptible(&crng_init_wait);
		kill_fasync(&fasync, SIGIO, POLL_IN);
		pr_notice("crng init done\n");
		if (urandom_warning.missed)
			pr_notice("%d urandom warning(s) missed due to ratelimiting\n",
				  urandom_warning.missed);
	} else if (orig < POOL_EARLY_BITS && new >= POOL_EARLY_BITS) {
		spin_lock_irqsave(&base_crng.lock, flags);
		/* Check if crng_init is CRNG_EMPTY, to avoid race with crng_reseed(). */
		if (crng_init == CRNG_EMPTY) {
			extract_entropy(base_crng.key, sizeof(base_crng.key));
			crng_init = CRNG_EARLY;
		}
		spin_unlock_irqrestore(&base_crng.lock, flags);
	}
}


/**********************************************************************
 *
 * Entropy collection routines.
 *
 * The following exported functions are used for pushing entropy into
 * the above entropy accumulation routines:
 *
 *	void add_device_randomness(const void *buf, size_t len);
 *	void add_hwgenerator_randomness(const void *buf, size_t len, size_t entropy);
 *	void add_bootloader_randomness(const void *buf, size_t len);
 *	void add_interrupt_randomness(int irq);
 *	void add_input_randomness(unsigned int type, unsigned int code, unsigned int value);
 *	void add_disk_randomness(struct gendisk *disk);
 *
 * add_device_randomness() adds data to the input pool that
 * is likely to differ between two devices (or possibly even per boot).
 * This would be things like MAC addresses or serial numbers, or the
 * read-out of the RTC. This does *not* credit any actual entropy to
 * the pool, but it initializes the pool to different values for devices
 * that might otherwise be identical and have very little entropy
 * available to them (particularly common in the embedded world).
 *
 * add_hwgenerator_randomness() is for true hardware RNGs, and will credit
 * entropy as specified by the caller. If the entropy pool is full it will
 * block until more entropy is needed.
 *
 * add_bootloader_randomness() is called by bootloader drivers, such as EFI
 * and device tree, and credits its input depending on whether or not the
 * configuration option CONFIG_RANDOM_TRUST_BOOTLOADER is set.
 *
 * add_interrupt_randomness() uses the interrupt timing as random
 * inputs to the entropy pool. Using the cycle counters and the irq source
 * as inputs, it feeds the input pool roughly once a second or after 64
 * interrupts, crediting 1 bit of entropy for whichever comes first.
 *
 * add_input_randomness() uses the input layer interrupt timing, as well
 * as the event type information from the hardware.
 *
 * add_disk_randomness() uses what amounts to the seek time of block
 * layer request events, on a per-disk_devt basis, as input to the
 * entropy pool. Note that high-speed solid state drives with very low
 * seek times do not make for good sources of entropy, as their seek
 * times are usually fairly consistent.
 *
 * The last two routines try to estimate how many bits of entropy
 * to credit. They do this by keeping track of the first and second
 * order deltas of the event timings.
 *
 **********************************************************************/

static bool trust_cpu __ro_after_init = IS_ENABLED(CONFIG_RANDOM_TRUST_CPU);
static bool trust_bootloader __ro_after_init = IS_ENABLED(CONFIG_RANDOM_TRUST_BOOTLOADER);
static int __init parse_trust_cpu(char *arg)
{
	return kstrtobool(arg, &trust_cpu);
}
static int __init parse_trust_bootloader(char *arg)
{
	return kstrtobool(arg, &trust_bootloader);
}
early_param("random.trust_cpu", parse_trust_cpu);
early_param("random.trust_bootloader", parse_trust_bootloader);

/*
 * The first collection of entropy occurs at system boot while interrupts
 * are still turned off. Here we push in latent entropy, RDSEED, a timestamp,
 * utsname(), and the command line. Depending on the above configuration knob,
 * RDSEED may be considered sufficient for initialization. Note that much
 * earlier setup may already have pushed entropy into the input pool by the
 * time we get here.
 */
int __init random_init(const char *command_line)
{
	ktime_t now = ktime_get_real();
	unsigned int i, arch_bytes;
	unsigned long entropy;

#if defined(LATENT_ENTROPY_PLUGIN)
	static const u8 compiletime_seed[BLAKE2S_BLOCK_SIZE] __initconst __latent_entropy;
	_mix_pool_bytes(compiletime_seed, sizeof(compiletime_seed));
#endif

	for (i = 0, arch_bytes = BLAKE2S_BLOCK_SIZE;
	     i < BLAKE2S_BLOCK_SIZE; i += sizeof(entropy)) {
		if (!arch_get_random_seed_long_early(&entropy) &&
		    !arch_get_random_long_early(&entropy)) {
			entropy = random_get_entropy();
			arch_bytes -= sizeof(entropy);
		}
		_mix_pool_bytes(&entropy, sizeof(entropy));
	}
	_mix_pool_bytes(&now, sizeof(now));
	_mix_pool_bytes(utsname(), sizeof(*(utsname())));
	_mix_pool_bytes(command_line, strlen(command_line));
	add_latent_entropy();

	if (crng_ready())
		crng_reseed();
	else if (trust_cpu)
		credit_init_bits(arch_bytes * 8);

	return 0;
}

/*
 * Add device- or boot-specific data to the input pool to help
 * initialize it.
 *
 * None of this adds any entropy; it is meant to avoid the problem of
 * the entropy pool having similar initial state across largely
 * identical devices.
 */
void add_device_randomness(const void *buf, unsigned int len)
{
	unsigned long entropy = random_get_entropy();
	unsigned long flags;

	spin_lock_irqsave(&input_pool.lock, flags);
	_mix_pool_bytes(&entropy, sizeof(entropy));
	_mix_pool_bytes(buf, len);
	spin_unlock_irqrestore(&input_pool.lock, flags);
}
EXPORT_SYMBOL(add_device_randomness);

/*
 * Interface for in-kernel drivers of true hardware RNGs.
 * Those devices may produce endless random bits and will be throttled
 * when our pool is full.
 */
void add_hwgenerator_randomness(const void *buf, size_t len, size_t entropy)
{
	mix_pool_bytes(buf, len);
	credit_init_bits(entropy);

	/*
	 * Throttle writing to once every CRNG_RESEED_INTERVAL, unless
	 * we're not yet initialized.
	 */
	if (!kthread_should_stop() && crng_ready())
		schedule_timeout_interruptible(CRNG_RESEED_INTERVAL);
}
EXPORT_SYMBOL_GPL(add_hwgenerator_randomness);

/*
 * Handle random seed passed by bootloader, and credit it if
 * CONFIG_RANDOM_TRUST_BOOTLOADER is set.
 */
void __cold add_bootloader_randomness(const void *buf, size_t len)
{
	mix_pool_bytes(buf, len);
	if (trust_bootloader)
		credit_init_bits(len * 8);
}
EXPORT_SYMBOL_GPL(add_bootloader_randomness);

struct fast_pool {
	unsigned long pool[4];
	unsigned long last;
	unsigned int count;
	struct timer_list mix;
};

static void mix_interrupt_randomness(struct timer_list *work);

static DEFINE_PER_CPU(struct fast_pool, irq_randomness) = {
#ifdef CONFIG_64BIT
#define FASTMIX_PERM SIPHASH_PERMUTATION
	.pool = { SIPHASH_CONST_0, SIPHASH_CONST_1, SIPHASH_CONST_2, SIPHASH_CONST_3 },
#else
#define FASTMIX_PERM HSIPHASH_PERMUTATION
	.pool = { HSIPHASH_CONST_0, HSIPHASH_CONST_1, HSIPHASH_CONST_2, HSIPHASH_CONST_3 },
#endif
	.mix = __TIMER_INITIALIZER(mix_interrupt_randomness, 0)
};

/*
 * This is [Half]SipHash-1-x, starting from an empty key. Because
 * the key is fixed, it assumes that its inputs are non-malicious,
 * and therefore this has no security on its own. s represents the
 * four-word SipHash state, while v represents a two-word input.
 */
static void fast_mix(unsigned long s[4], unsigned long v1, unsigned long v2)
{
	s[3] ^= v1;
	FASTMIX_PERM(s[0], s[1], s[2], s[3]);
	s[0] ^= v1;
	s[3] ^= v2;
	FASTMIX_PERM(s[0], s[1], s[2], s[3]);
	s[0] ^= v2;
}

#ifdef CONFIG_SMP
/*
 * This function is called when the CPU has just come online, with
 * entry CPUHP_AP_RANDOM_ONLINE, just after CPUHP_AP_WORKQUEUE_ONLINE.
 */
int __cold random_online_cpu(unsigned int cpu)
{
	/*
	 * During CPU shutdown and before CPU onlining, add_interrupt_
	 * randomness() may schedule mix_interrupt_randomness(), and
	 * set the MIX_INFLIGHT flag. However, because the worker can
	 * be scheduled on a different CPU during this period, that
	 * flag will never be cleared. For that reason, we zero out
	 * the flag here, which runs just after workqueues are onlined
	 * for the CPU again. This also has the effect of setting the
	 * irq randomness count to zero so that new accumulated irqs
	 * are fresh.
	 */
	per_cpu_ptr(&irq_randomness, cpu)->count = 0;
	return 0;
}
#endif

static void mix_interrupt_randomness(struct timer_list *work)
{
	struct fast_pool *fast_pool = container_of(work, struct fast_pool, mix);
	/*
	 * The size of the copied stack pool is explicitly 2 longs so that we
	 * only ever ingest half of the siphash output each time, retaining
	 * the other half as the next "key" that carries over. The entropy is
	 * supposed to be sufficiently dispersed between bits so on average
	 * we don't wind up "losing" some.
	 */
	unsigned long pool[2];
	unsigned int count;

	/* Check to see if we're running on the wrong CPU due to hotplug. */
	local_irq_disable();
	if (fast_pool != this_cpu_ptr(&irq_randomness)) {
		local_irq_enable();
		return;
	}

	/*
	 * Copy the pool to the stack so that the mixer always has a
	 * consistent view, before we reenable irqs again.
	 */
	memcpy(pool, fast_pool->pool, sizeof(pool));
	count = fast_pool->count;
	fast_pool->count = 0;
	fast_pool->last = jiffies;
	local_irq_enable();

	mix_pool_bytes(pool, sizeof(pool));
	credit_init_bits(clamp_t(unsigned int, (count & U16_MAX) / 64, 1, sizeof(pool) * 8));

	memzero_explicit(pool, sizeof(pool));
}

void add_interrupt_randomness(int irq)
{
	enum { MIX_INFLIGHT = 1U << 31 };
	unsigned long entropy = random_get_entropy();
	struct fast_pool *fast_pool = this_cpu_ptr(&irq_randomness);
	struct pt_regs *regs = get_irq_regs();
	unsigned int new_count;

	fast_mix(fast_pool->pool, entropy,
		 (regs ? instruction_pointer(regs) : _RET_IP_) ^ swab(irq));
	new_count = ++fast_pool->count;

	if (new_count & MIX_INFLIGHT)
		return;

	if (new_count < 1024 && !time_is_before_jiffies(fast_pool->last + HZ))
		return;

	fast_pool->count |= MIX_INFLIGHT;
	if (!timer_pending(&fast_pool->mix)) {
		fast_pool->mix.expires = jiffies;
		add_timer_on(&fast_pool->mix, raw_smp_processor_id());
	}
}
EXPORT_SYMBOL_GPL(add_interrupt_randomness);

/* There is one of these per entropy source */
struct timer_rand_state {
	unsigned long last_time;
	long last_delta, last_delta2;
};

/*
 * This function adds entropy to the entropy "pool" by using timing
 * delays. It uses the timer_rand_state structure to make an estimate
 * of how many bits of entropy this call has added to the pool. The
 * value "num" is also added to the pool; it should somehow describe
 * the type of event that just happened.
 */
static void add_timer_randomness(struct timer_rand_state *state, unsigned int num)
{
	unsigned long entropy = random_get_entropy(), now = jiffies, flags;
	long delta, delta2, delta3;
	unsigned int bits;

	/*
	 * If we're in a hard IRQ, add_interrupt_randomness() will be called
	 * sometime after, so mix into the fast pool.
	 */
	if (in_irq()) {
		fast_mix(this_cpu_ptr(&irq_randomness)->pool, entropy, num);
	} else {
		spin_lock_irqsave(&input_pool.lock, flags);
		_mix_pool_bytes(&entropy, sizeof(entropy));
		_mix_pool_bytes(&num, sizeof(num));
		spin_unlock_irqrestore(&input_pool.lock, flags);
	}

	if (crng_ready())
		return;

	/*
	 * Calculate number of bits of randomness we probably added.
	 * We take into account the first, second and third-order deltas
	 * in order to make our estimate.
	 */
	delta = now - READ_ONCE(state->last_time);
	WRITE_ONCE(state->last_time, now);

	delta2 = delta - READ_ONCE(state->last_delta);
	WRITE_ONCE(state->last_delta, delta);

	delta3 = delta2 - READ_ONCE(state->last_delta2);
	WRITE_ONCE(state->last_delta2, delta2);

	if (delta < 0)
		delta = -delta;
	if (delta2 < 0)
		delta2 = -delta2;
	if (delta3 < 0)
		delta3 = -delta3;
	if (delta > delta2)
		delta = delta2;
	if (delta > delta3)
		delta = delta3;

	/*
	 * delta is now minimum absolute delta. Round down by 1 bit
	 * on general principles, and limit entropy estimate to 11 bits.
	 */
	bits = min(fls(delta >> 1), 11);

	/*
	 * As mentioned above, if we're in a hard IRQ, add_interrupt_randomness()
	 * will run after this, which uses a different crediting scheme of 1 bit
	 * per every 64 interrupts. In order to let that function do accounting
	 * close to the one in this function, we credit a full 64/64 bit per bit,
	 * and then subtract one to account for the extra one added.
	 */
	if (in_irq())
		this_cpu_ptr(&irq_randomness)->count += max(1u, bits * 64) - 1;
	else
		_credit_init_bits(bits);
}

void add_input_randomness(unsigned int type, unsigned int code, unsigned int value)
{
	static unsigned char last_value;
	static struct timer_rand_state input_timer_state = { INITIAL_JIFFIES };

	/* Ignore autorepeat and the like. */
	if (value == last_value)
		return;

	last_value = value;
	add_timer_randomness(&input_timer_state,
			     (type << 4) ^ code ^ (code >> 4) ^ value);
}
EXPORT_SYMBOL_GPL(add_input_randomness);

#ifdef CONFIG_BLOCK
void add_disk_randomness(struct gendisk *disk)
{
	if (!disk || !disk->random)
		return;
	/* First major is 1, so we get >= 0x200 here. */
	add_timer_randomness(disk->random, 0x100 + disk_devt(disk));
}
EXPORT_SYMBOL_GPL(add_disk_randomness);

void __cold rand_initialize_disk(struct gendisk *disk)
{
	struct timer_rand_state *state;

	/*
	 * If kzalloc returns null, we just won't use that entropy
	 * source.
	 */
	state = kzalloc(sizeof(struct timer_rand_state), GFP_KERNEL);
	if (state) {
		state->last_time = INITIAL_JIFFIES;
		disk->random = state;
	}
}
#endif

/*
 * Each time the timer fires, we expect that we got an unpredictable
 * jump in the cycle counter. Even if the timer is running on another
 * CPU, the timer activity will be touching the stack of the CPU that is
 * generating entropy..
 *
 * Note that we don't re-arm the timer in the timer itself - we are
 * happy to be scheduled away, since that just makes the load more
 * complex, but we do not want the timer to keep ticking unless the
 * entropy loop is running.
 *
 * So the re-arming always happens in the entropy loop itself.
 */
static void __cold entropy_timer(struct timer_list *t)
{
	credit_init_bits(1);
}

/*
 * If we have an actual cycle counter, see if we can
 * generate enough entropy with timing noise
 */
static void __cold try_to_generate_entropy(void)
{
	struct {
		unsigned long entropy;
		struct timer_list timer;
	} stack;

	stack.entropy = random_get_entropy();

	/* Slow counter - or none. Don't even bother */
	if (stack.entropy == random_get_entropy())
		return;

	timer_setup_on_stack(&stack.timer, entropy_timer, 0);
	while (!crng_ready() && !signal_pending(current)) {
		if (!timer_pending(&stack.timer))
			mod_timer(&stack.timer, jiffies + 1);
		mix_pool_bytes(&stack.entropy, sizeof(stack.entropy));
		schedule();
		stack.entropy = random_get_entropy();
	}

	del_timer_sync(&stack.timer);
	destroy_timer_on_stack(&stack.timer);
	mix_pool_bytes(&stack.entropy, sizeof(stack.entropy));
}


/**********************************************************************
 *
 * Userspace reader/writer interfaces.
 *
 * getrandom(2) is the primary modern interface into the RNG and should
 * be used in preference to anything else.
 *
 * Reading from /dev/random has the same functionality as calling
 * getrandom(2) with flags=0. In earlier versions, however, it had
 * vastly different semantics and should therefore be avoided, to
 * prevent backwards compatibility issues.
 *
 * Reading from /dev/urandom has the same functionality as calling
 * getrandom(2) with flags=GRND_INSECURE. Because it does not block
 * waiting for the RNG to be ready, it should not be used.
 *
 * Writing to either /dev/random or /dev/urandom adds entropy to
 * the input pool but does not credit it.
 *
 * Polling on /dev/random indicates when the RNG is initialized, on
 * the read side, and when it wants new entropy, on the write side.
 *
 * Both /dev/random and /dev/urandom have the same set of ioctls for
 * adding entropy, getting the entropy count, zeroing the count, and
 * reseeding the crng.
 *
 **********************************************************************/

SYSCALL_DEFINE3(getrandom, char __user *, ubuf, size_t, len, unsigned int, flags)
{
	struct iov_iter iter;
	struct iovec iov;
	int ret;

	if (flags & ~(GRND_NONBLOCK | GRND_RANDOM | GRND_INSECURE))
		return -EINVAL;

	/*
	 * Requesting insecure and blocking randomness at the same time makes
	 * no sense.
	 */
	if ((flags & (GRND_INSECURE | GRND_RANDOM)) == (GRND_INSECURE | GRND_RANDOM))
		return -EINVAL;

	if (!crng_ready() && !(flags & GRND_INSECURE)) {
		if (flags & GRND_NONBLOCK)
			return -EAGAIN;
		ret = wait_for_random_bytes();
		if (unlikely(ret))
			return ret;
	}

	ret = import_single_range(READ, ubuf, len, &iov, &iter);
	if (unlikely(ret))
		return ret;
	return get_random_bytes_user(&iter);
}

static __poll_t random_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &crng_init_wait, wait);
	return crng_ready() ? EPOLLIN | EPOLLRDNORM : EPOLLOUT | EPOLLWRNORM;
}

static ssize_t write_pool_user(struct iov_iter *iter)
{
	u8 block[BLAKE2S_BLOCK_SIZE];
	ssize_t ret = 0;
	size_t copied;

	if (unlikely(!iov_iter_count(iter)))
		return 0;

	for (;;) {
		copied = copy_from_iter(block, sizeof(block), iter);
		ret += copied;
		mix_pool_bytes(block, copied);
		if (!iov_iter_count(iter) || copied != sizeof(block))
			break;

		BUILD_BUG_ON(PAGE_SIZE % sizeof(block) != 0);
		if (ret % PAGE_SIZE == 0) {
			if (signal_pending(current))
				break;
			cond_resched();
		}
	}

	memzero_explicit(block, sizeof(block));
	return ret ? ret : -EFAULT;
}

static ssize_t random_write_iter(struct kiocb *kiocb, struct iov_iter *iter)
{
	return write_pool_user(iter);
}

static ssize_t urandom_read_iter(struct kiocb *kiocb, struct iov_iter *iter)
{
	static int maxwarn = 10;

	if (!crng_ready()) {
		if (!ratelimit_disable && maxwarn <= 0)
			++urandom_warning.missed;
		else if (ratelimit_disable || __ratelimit(&urandom_warning)) {
			--maxwarn;
			pr_notice("%s: uninitialized urandom read (%zu bytes read)\n",
				  current->comm, iov_iter_count(iter));
		}
	}

	return get_random_bytes_user(iter);
}

static ssize_t random_read_iter(struct kiocb *kiocb, struct iov_iter *iter)
{
	int ret;

	if (!crng_ready() &&
	    ((kiocb->ki_flags & (IOCB_NOWAIT | IOCB_NOIO)) ||
	     (kiocb->ki_filp->f_flags & O_NONBLOCK)))
		return -EAGAIN;

	ret = wait_for_random_bytes();
	if (ret != 0)
		return ret;
	return get_random_bytes_user(iter);
}

static long random_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	int __user *p = (int __user *)arg;
	int ent_count;

	switch (cmd) {
	case RNDGETENTCNT:
		/* Inherently racy, no point locking. */
		if (put_user(input_pool.init_bits, p))
			return -EFAULT;
		return 0;
	case RNDADDTOENTCNT:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		if (get_user(ent_count, p))
			return -EFAULT;
		if (ent_count < 0)
			return -EINVAL;
		credit_init_bits(ent_count);
		return 0;
	case RNDADDENTROPY: {
		struct iov_iter iter;
		struct iovec iov;
		ssize_t ret;
		int len;

		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		if (get_user(ent_count, p++))
			return -EFAULT;
		if (ent_count < 0)
			return -EINVAL;
		if (get_user(len, p++))
			return -EFAULT;
		ret = import_single_range(WRITE, p, len, &iov, &iter);
		if (unlikely(ret))
			return ret;
		ret = write_pool_user(&iter);
		if (unlikely(ret < 0))
			return ret;
		/* Since we're crediting, enforce that it was all written into the pool. */
		if (unlikely(ret != len))
			return -EFAULT;
		credit_init_bits(ent_count);
		return 0;
	}
	case RNDZAPENTCNT:
	case RNDCLEARPOOL:
		/* No longer has any effect. */
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		return 0;
	case RNDRESEEDCRNG:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		if (!crng_ready())
			return -ENODATA;
		crng_reseed();
		return 0;
	default:
		return -EINVAL;
	}
}

static int random_fasync(int fd, struct file *filp, int on)
{
	return fasync_helper(fd, filp, on, &fasync);
}

const struct file_operations random_fops = {
	.read_iter = random_read_iter,
	.write_iter = random_write_iter,
	.poll = random_poll,
	.unlocked_ioctl = random_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.fasync = random_fasync,
	.llseek = noop_llseek,
	.splice_read = generic_file_splice_read,
	.splice_write = iter_file_splice_write,
};

const struct file_operations urandom_fops = {
	.read_iter = urandom_read_iter,
	.write_iter = random_write_iter,
	.unlocked_ioctl = random_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.fasync = random_fasync,
	.llseek = noop_llseek,
	.splice_read = generic_file_splice_read,
	.splice_write = iter_file_splice_write,
};


/********************************************************************
 *
 * Sysctl interface.
 *
 * These are partly unused legacy knobs with dummy values to not break
 * userspace and partly still useful things. They are usually accessible
 * in /proc/sys/kernel/random/ and are as follows:
 *
 * - boot_id - a UUID representing the current boot.
 *
 * - uuid - a random UUID, different each time the file is read.
 *
 * - poolsize - the number of bits of entropy that the input pool can
 *   hold, tied to the POOL_BITS constant.
 *
 * - entropy_avail - the number of bits of entropy currently in the
 *   input pool. Always <= poolsize.
 *
 * - write_wakeup_threshold - the amount of entropy in the input pool
 *   below which write polls to /dev/random will unblock, requesting
 *   more entropy, tied to the POOL_READY_BITS constant. It is writable
 *   to avoid breaking old userspaces, but writing to it does not
 *   change any behavior of the RNG.
 *
 * - urandom_min_reseed_secs - fixed to the value CRNG_RESEED_INTERVAL.
 *   It is writable to avoid breaking old userspaces, but writing
 *   to it does not change any behavior of the RNG.
 *
 ********************************************************************/

#ifdef CONFIG_SYSCTL

#include <linux/sysctl.h>

static int sysctl_random_min_urandom_seed = CRNG_RESEED_INTERVAL / HZ;
static int sysctl_random_write_wakeup_bits = POOL_READY_BITS;
static int sysctl_poolsize = POOL_BITS;
static u8 sysctl_bootid[UUID_SIZE];

/*
 * This function is used to return both the bootid UUID, and random
 * UUID. The difference is in whether table->data is NULL; if it is,
 * then a new UUID is generated and returned to the user.
 */
static int proc_do_uuid(struct ctl_table *table, int write, void *buf,
			size_t *lenp, loff_t *ppos)
{
	u8 tmp_uuid[UUID_SIZE], *uuid;
	char uuid_string[UUID_STRING_LEN + 1];
	struct ctl_table fake_table = {
		.data = uuid_string,
		.maxlen = UUID_STRING_LEN
	};

	if (write)
		return -EPERM;

	uuid = table->data;
	if (!uuid) {
		uuid = tmp_uuid;
		generate_random_uuid(uuid);
	} else {
		static DEFINE_SPINLOCK(bootid_spinlock);

		spin_lock(&bootid_spinlock);
		if (!uuid[8])
			generate_random_uuid(uuid);
		spin_unlock(&bootid_spinlock);
	}

	snprintf(uuid_string, sizeof(uuid_string), "%pU", uuid);
	return proc_dostring(&fake_table, 0, buf, lenp, ppos);
}

/* The same as proc_dointvec, but writes don't change anything. */
static int proc_do_rointvec(struct ctl_table *table, int write, void *buf,
			    size_t *lenp, loff_t *ppos)
{
	return write ? 0 : proc_dointvec(table, 0, buf, lenp, ppos);
}

extern struct ctl_table random_table[];
struct ctl_table random_table[] = {
	{
		.procname	= "poolsize",
		.data		= &sysctl_poolsize,
		.maxlen		= sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "entropy_avail",
		.data		= &input_pool.init_bits,
		.maxlen		= sizeof(int),
		.mode		= 0444,
		.proc_handler	= proc_dointvec,
	},
	{
		.procname	= "write_wakeup_threshold",
		.data		= &sysctl_random_write_wakeup_bits,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_do_rointvec,
	},
	{
		.procname	= "urandom_min_reseed_secs",
		.data		= &sysctl_random_min_urandom_seed,
		.maxlen		= sizeof(int),
		.mode		= 0644,
		.proc_handler	= proc_do_rointvec,
	},
	{
		.procname	= "boot_id",
		.data		= &sysctl_bootid,
		.mode		= 0444,
		.proc_handler	= proc_do_uuid,
	},
	{
		.procname	= "uuid",
		.mode		= 0444,
		.proc_handler	= proc_do_uuid,
	},
	{ }
};
#endif	/* CONFIG_SYSCTL */

/*
 * Android KABI fixups
 *
 * Add back two functions that were being used by out-of-tree drivers.
 *
 * Yes, horrible hack, the things we do for FIPS "compliance"...
 */
static DEFINE_SPINLOCK(random_ready_list_lock);
static LIST_HEAD(random_ready_list);

int add_random_ready_callback(struct random_ready_callback *rdy)
{
	struct module *owner;
	unsigned long flags;
	int err = -EALREADY;

	if (crng_ready())
		return err;

	owner = rdy->owner;
	if (!try_module_get(owner))
		return -ENOENT;

	spin_lock_irqsave(&random_ready_list_lock, flags);
	if (crng_ready())
		goto out;

	owner = NULL;

	list_add(&rdy->list, &random_ready_list);
	err = 0;

out:
	spin_unlock_irqrestore(&random_ready_list_lock, flags);

	module_put(owner);

	return err;
}
EXPORT_SYMBOL(add_random_ready_callback);

void del_random_ready_callback(struct random_ready_callback *rdy)
{
	unsigned long flags;
	struct module *owner = NULL;

	spin_lock_irqsave(&random_ready_list_lock, flags);
	if (!list_empty(&rdy->list)) {
		list_del_init(&rdy->list);
		owner = rdy->owner;
	}
	spin_unlock_irqrestore(&random_ready_list_lock, flags);

	module_put(owner);
}
EXPORT_SYMBOL(del_random_ready_callback);

static void process_oldschool_random_ready_list(void)
{
	unsigned long flags;
	struct random_ready_callback *rdy, *tmp;

	spin_lock_irqsave(&random_ready_list_lock, flags);
	list_for_each_entry_safe(rdy, tmp, &random_ready_list, list) {
		struct module *owner = rdy->owner;

		list_del_init(&rdy->list);
		rdy->func(rdy);
		module_put(owner);
	}
	spin_unlock_irqrestore(&random_ready_list_lock, flags);
}
