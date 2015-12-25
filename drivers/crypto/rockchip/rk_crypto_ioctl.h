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

#ifndef __ROCKCHIP_CRYPTO_IO__
#define __ROCKCHIP_CRYPTO_IO__

#include <linux/rockchip/cpu.h>
#include <linux/rockchip/iomap.h>

#define DMA_BUF_LEN     0x1000
#define RSA_DATA_MAX 512
#define RSA_REG_LEN 256
/* get register virt address */
#define CRYPTO_GET_REG_VIRT(dev, reg)   ((dev)->ioaddr + (reg))

enum io_cmd {
	SET_ALG = 3,
	SET_PARAM,
	SET_INPUT,
	GET_OUTPUT
};

enum crypto_mode {
	RK_ENCRYPT,
	RK_DECRYPT
};

struct aes_ctx {
	u32 cmode;		/*ecrypt or decrypt*/
	u32 keylen;
	u8 key[32];
	u8 iv[16];
	u8 count[16];
};

struct des_ctx {
	u32 cmode;		/*ecrypt or decrypt*/
	u32 keylen;
	u8 key[24];
	u8 iv[8];
};

struct rsa_ctx {
	u8 *rsa_m;
	u8 *rsa_c;
	u8 *rsa_n;
	u8 *rsa_e;
	u32 mlen;
	u32 clen;
	u32 nlen;
	u32 elen;
};

struct alg_name {
	u32 name;
};

struct data_in {
	u32 len;
	u8  *in;
};

struct data_out {
	u32 len;
	u8 *out;
};

struct ioctl_ctx {
	struct crypto_dev *dev;
	struct alg_name *rk_algn;
	struct aes_ctx *rk_aes;
	struct des_ctx *rk_des;
	struct hash_ctx *rk_hash;
	struct rsa_ctx *rk_rsa;
	struct data_in *rk_in;
	dma_addr_t dma_in;
	dma_addr_t dma_out;
	u8 *out;
};

#endif

