/*
 * Crypto API.
 *
 * Support for Rockchip Crypto HW acceleration.
 *
 * Copyright (C) 2013 Rockchip Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 */

#ifndef __ROCKCHIP_CRYPTO__
#define __ROCKCHIP_CRYPTO__

#include <linux/rockchip/cpu.h>
#include <linux/rockchip/iomap.h>
#include <linux/rockchip/grf.h>
#include <linux/reset.h>
#include <linux/miscdevice.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/ctr.h>

/* Crypto control registers*/
#define CRYPTO_INTSTS		0x0000
#define PKA_DONE_INT		BIT(5)
#define HASH_DONE_INT		BIT(4)
#define HRDMA_ERR_INT		BIT(3)
#define HRDMA_DONE_INT		BIT(2)
#define BCDMA_ERR_INT		BIT(1)
#define BCDMA_DONE_INT		BIT(0)

#define CRYPTO_INTENA		0x0004
#define PKA_DONE_ENA		BIT(5)
#define HASH_DONE_ENA		BIT(4)
#define HRDMA_ERR_ENA		BIT(3)
#define HRDMA_DONE_ENA		BIT(2)
#define BCDMA_ERR_ENA		BIT(1)
#define BCDMA_DONE_ENA		BIT(0)

#define CRYPTO_CTRL		0x0008
#define TRNG_FLUSH		BIT(9)
#define TRNG_START		BIT(8)
#define PKA_FLUSH		BIT(7)
#define HASH_FLUSH		BIT(6)
#define BLOCK_FLUSH		BIT(5)
#define PKA_START		BIT(4)
#define HASH_START		BIT(3)
#define BLOCK_START		BIT(2)
#define TDES_START		BIT(1)
#define AES_START		BIT(0)

#define CRYPTO_CONF		0x000c
/* HASH Receive DMA Address Mode:   fix | increment */
#define HR_ADDR_MODE		BIT(8)
/* Block Transmit DMA Address Mode: fix | increment */
#define BT_ADDR_MODE		BIT(7)
/* Block Receive DMA Address Mode:  fix | increment */
#define BR_ADDR_MODE		BIT(6)
#define BYTESWAP_HRFIFO		BIT(5)
#define BYTESWAP_BTFIFO		BIT(4)
#define BYTESWAP_BRFIFO		BIT(3)
/* AES = 0 OR DES = 1 */
#define DESSEL			BIT(2)
#define INDEPENDENT_SOURCE	0x00
#define BLOCK_CIPHER_INPUT	0x01
#define BLOCK_CIPHER_OUTPUT	0x02

/* Block Receiving DMA Start Address Register */
#define CRYPTO_BRDMAS		0x0010
/* Block Transmitting DMA Start Address Register */
#define CRYPTO_BTDMAS		0x0014
/* Block Receiving DMA Length Register */
#define CRYPTO_BRDMAL		0x0018
/* Hash Receiving DMA Start Address Register */
#define CRYPTO_HRDMAS		0x001c
/* Hash Receiving DMA Length Register */
#define CRYPTO_HRDMAL		0x0020

/* AES registers */
#define AES_CTRL		0x0080
#define AES_BYTESWAP_CNT	BIT(11)
#define AES_BYTESWAP_KEY	BIT(10)
#define AES_BYTESWAP_IV		BIT(9)
#define AES_BYTESWAP_DO		BIT(8)
#define AES_BYTESWAP_DI		BIT(7)
#define AES_KEY_CHANGE		BIT(6)
#define AES_ECB_MODE		(0x0 << 4)
#define AES_CBC_MODE		(0x1 << 4)
#define AES_CTR_MODE		(0x2 << 4)
#define AES_128_BIT_KEY		(0x0 << 2)
#define AES_192_BIT_KEY		(0x1 << 2)
#define AES_256_BIT_KEY		(0x2 << 2)
/* Slave = 0 / fifo = 1 */
#define AES_FIFO_MODE		BIT(1)
/* Encryption = 0 , Decryption = 1 */
#define AES_ENC			BIT(0)

#define AES_STS			0x0084
#define AES_DONE		BIT(0)

/* AES Input Data 0-3 Register */
#define AES_DIN_0		0x0088
#define AES_DIN_1		0x008c
#define AES_DIN_2		0x0090
#define AES_DIN_3		0x0094

/* AES output Data 0-3 Register */
#define AES_DOUT_0		0x0098
#define AES_DOUT_1		0x009c
#define AES_DOUT_2		0x00a0
#define AES_DOUT_3		0x00a4

/* AES IV Data 0-3 Register */
#define AES_IV_0		0x00a8
#define AES_IV_1		0x00ac
#define AES_IV_2		0x00b0
#define AES_IV_3		0x00b4

/* AES Key Data 0-3 Register */
#define AES_KEY_0		0x00b8
#define AES_KEY_1		0x00bc
#define AES_KEY_2		0x00c0
#define AES_KEY_3		0x00c4
#define AES_KEY_4		0x00c8
#define AES_KEY_5		0x00cc
#define AES_KEY_6		0x00d0
#define AES_KEY_7		0x00d4

/* AES Input Counter 0-3 Register */
#define AES_CNT_0		0x00d8
#define AES_CNT_1		0x00dc
#define AES_CNT_2		0x00e0
#define AES_CNT_3		0x00e4

/* des/tdes */
#define TDES_CTRL		0x0100
#define TDES_BYTESWAP_KEY	BIT(8)
#define TDES_BYTESWAP_IV	BIT(7)
#define TDES_BYTESWAP_DO	BIT(6)
#define TDES_BYTESWAP_DI	BIT(5)
/* 0: ECB, 1: CBC */
#define TDES_CHAINMODE		BIT(4)
/* TDES Key Mode, 0 : EDE, 1 : EEE */
#define TDES_EEE		BIT(3)
/* 0: DES, 1:TDES */
#define TDES_SELECT		BIT(2)
/* 0: Slave, 1:Fifo */
#define TDES_FIFO_MODE		BIT(1)
/* Encryption = 0 , Decryption = 1 */
#define TDES_ENC		BIT(0)

#define TDES_STS		0x0104
#define TDES_DONE		BIT(0)

#define TDES_DIN_0		0x0108
#define TDES_DIN_1		0x010c
#define TDES_DOUT_0		0x0110
#define TDES_DOUT_1		0x0114
#define TDES_IV_0		0x0118
#define TDES_IV_1		0x011c
#define TDES_KEY1_0		0x0120
#define TDES_KEY1_1		0x0124
#define TDES_KEY2_0		0x0128
#define TDES_KEY2_1		0x012c
#define TDES_KEY3_0		0x0130
#define TDES_KEY3_1		0x0134

/* HASH */
#define HASH_CTRL		0x0180
#define HASH_SWAP_DO		BIT(3)
#define HASH_SWAP_DI		BIT(2)
#define HASH_SHA1		(0x0 << 0)
#define HASH_MD5		(0x1 << 0)
#define HASH_SHA256		(0x2 << 0)
#define HASH_PRNG		(0x3 << 0)

#define HASH_STS		0x0184
#define HASH_DONE		BIT(0)

#define HASH_MSG_LEN		0x0188
#define HASH_DOUT_0		0x018c
#define HASH_DOUT_1		0x0190
#define HASH_DOUT_2		0x0194
#define HASH_DOUT_3		0x0198
#define HASH_DOUT_4		0x019c
#define HASH_DOUT_5		0x01a0
#define HASH_DOUT_6		0x01a4
#define HASH_DOUT_7		0x01a8
#define HASH_SEED_0		0x01ac
#define HASH_SEED_1		0x01b0
#define HASH_SEED_2		0x01b4
#define HASH_SEED_3		0x01b8
#define HASH_SEED_4		0x01bc

/* TRNG */
#define TRNG_CTRL		0x0200
#define OSC_ENABLE		BIT(16)

#define TRNG_DOUT_0		0x0204
#define TRNG_DOUT_1		0x0208
#define TRNG_DOUT_2		0x020c
#define TRNG_DOUT_3		0x0210
#define TRNG_DOUT_4		0x0214
#define TRNG_DOUT_5		0x0218
#define TRNG_DOUT_6		0x021c
#define TRNG_DOUT_7		0x0220

/* PAK OR RSA */
#define PKA_CTRL		0x0280
#define PKA_BLOCK_SIZE_512BIT	(0x0 << 0)
#define PKA_BLOCK_SIZE_1024BIT	(0x1 << 0)
#define PKA_BLOCK_SIZE_2048BIT	(0x2 << 0)

/* result = (M ^ E) mod N */
#define PKA_M			0x0400
/* C = 2 ^ (2n+2) mod N */
#define PKA_C			0x0500
#define PKA_N			0x0600
#define PKA_E			0x0700

#define CRYPRO_READ(dev, reg)          \
		__raw_readl(((dev)->ioaddr + (reg)))
#define CRYPRO_WRITE(dev, reg, val)      \
		__raw_writel((val), ((dev)->ioaddr + (reg)))

/* get register virt address */
#define CRYPTO_GET_REG_VIRT(dev, reg)   ((dev)->ioaddr + (reg))

#define RSA_DATA_MAX_LEN	512

enum alg_list {
	RK_AES_ECB = 3,
	RK_AES_CBC,
	RK_AES_CTR,
	RK_DES_ECB,
	RK_DES_CBC,
	RK_DES3_EDE_ECB,
	RK_DES3_EDE_CBC,
	RK_DES3_EEE_ECB,
	RK_DES3_EEE_CBC,
	RK_SHA1,
	RK_SHA256,
	RK_MD5,
	RK_RSA_512_BIT,
	RK_RSA_1024_BIT,
	RK_RSA_2048_BIT
};

struct rk_crypto_reqctx {
	u32 mode;
};

struct rk_aes_ctx {
	struct crypto_dev *dev;
	u8 aes_key[AES_MAX_KEY_SIZE];
	u8 iv[AES_BLOCK_SIZE];
	u32 keylen;
};

struct rk_des_ctx {
	struct crypto_dev *dev;
	u8 des_key[DES3_EDE_KEY_SIZE];
	u8 iv[DES_BLOCK_SIZE];
	u32 keylen;
};

struct crypto_dev {
	struct device *dev;
	struct clk *hclk;
	struct clk *aclk;
	struct clk *clk;
	void __iomem *ioaddr;
	int irq_crypto;

	struct reset_control *crypto_rst;
	struct reset_control *dma1_rst;
	struct miscdevice miscdev;
	struct crypto_ablkcipher *tfm;
	struct ablkcipher_request *ablk_req;
	struct ahash_request *ahash_req;
	struct rk_aes_ctx *aes_ctx;
	struct rk_des_ctx *des_ctx;
	struct scatterlist *sg_src;
	struct scatterlist *sg_dst;
	u32 flags;

	struct tasklet_struct crypto_tasklet;
	struct crypto_queue queue;
	bool busy;
	spinlock_t lock; /* spinlock for crypto handling */
};

extern struct crypto_dev *cdev;
#define MODE_NAME  "rockchip-crypto"

struct rk_ahash_reqctx {
	u32 mode;

	/* total request */
	u32 total;
};

struct rk_ahash_ctx {
	struct crypto_dev *dev;
};

#ifdef RK_CRYPTO_DEBUG
#define CRYPTO_DEBUG()  pr_warn("%s:%d\n", __func__, __LINE__)
#define CRYPTO_INFO(fmt, args...)    \
	pr_warn("\033[33m[%s:%d]\033[0m " #fmt "\r\n", \
	__func__, __LINE__, ##args)
#define hexdump(buf, len)	print_hex_dump(KERN_CONT, "", \
			DUMP_PREFIX_OFFSET, 16, 1, (buf), (len), false)
#else
#define CRYPTO_DEBUG()  do {} while (0)
#define CRYPTO_INFO(fmt, args...)
#define hexdump(buf, len)
#endif

int rockchip_aes_test(void);
int rockchip_des_test(void);
int rockchip_hash_test(void);
int rockchip_crypto_misc_register(struct crypto_dev *dev);

#endif

