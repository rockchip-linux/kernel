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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <crypto/algapi.h>
#include <linux/slab.h>
#include <crypto/hash.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/highmem.h>
#include <linux/rockchip/cru.h>
#include "rk_crypto.h"
#include "rk_crypto_ioctl.h"

#define AES_BLOCK_LEN 16
#define DES_BLOCK_LEN 8
#define DES_KEY_LEN 8
#define TDES_KEY_LEN (DES_KEY_LEN * 3)
#define HASH_OUT_LEN 32

static int rockchip_io_clk_enable(struct crypto_dev *dev)
{
	if (clk_prepare_enable(dev->hclk)) {
		pr_err("Couldn't enable clock 'hclk_crypto'\n");
		return -ENOENT;
	}

	if (clk_prepare_enable(dev->aclk)) {
		pr_err("Couldn't enable clock 'aclk_crypto'\n");
		clk_disable_unprepare(dev->hclk);
		return -ENOENT;
	}

	if (clk_prepare_enable(dev->clk)) {
		pr_err("Couldn't enable clock 'clk_crypto'\n");
		clk_disable_unprepare(dev->hclk);
		clk_disable_unprepare(dev->aclk);
		return -ENOENT;
	}

	return 0;
}

static void rockchip_io_clk_disable(struct crypto_dev *dev)
{
	clk_disable_unprepare(dev->hclk);
	clk_disable_unprepare(dev->aclk);
	clk_disable_unprepare(dev->clk);
}

static int rockchip_io_set_alg(struct ioctl_ctx *ctx, void __user *arg)
{
	int cbytes = 0;

	cbytes = sizeof(struct alg_name);
	ctx->rk_algn = kzalloc(cbytes, GFP_KERNEL);
	if (!ctx->rk_algn)
		return -ENOMEM;

	if (unlikely(copy_from_user(ctx->rk_algn, arg, cbytes))) {
		pr_err("%s:%d, copy_from_user failed\n", __func__, __LINE__);
		kfree(ctx->rk_algn);
		return -EFAULT;
	}

	return 0;
}

static void rockchip_io_set_aes_count(
			struct crypto_dev *dev, u8 *count)
{
	if (count)
		memcpy(dev->ioaddr + AES_CNT_0, count, AES_BLOCK_LEN);
	else
		memset(dev->ioaddr + AES_CNT_0, 0, AES_BLOCK_LEN);
}

static void rockchip_io_set_aes_iv(struct crypto_dev *dev, u8 *iv)
{
	if (iv)
		memcpy(dev->ioaddr + AES_IV_0, iv, AES_BLOCK_LEN);
	else
		memset(dev->ioaddr + AES_IV_0, 0, AES_BLOCK_LEN);
}

static int rockchip_io_set_aes_key(struct crypto_dev *dev,
				   u8 *key, u32 len)
{
	if (len != AES_KEYSIZE_128 &&
	    len != AES_KEYSIZE_192 &&
	    len != AES_KEYSIZE_256) {
		pr_err("%s:%d, key len is error!!\n", __func__, __LINE__);
		return -EINVAL;
	}
#ifdef RK_CRYPTO_DEBUG
	hexdump(key, len);
#endif
	memcpy(dev->ioaddr + AES_KEY_0, key, len);
	return 0;
}

static void rockchip_io_set_aes_param(struct crypto_dev *dev,
				      struct aes_ctx *ctx, int alg)
{
	switch (alg) {
	case RK_AES_CTR:
		rockchip_io_set_aes_count(dev, ctx->count);
	case RK_AES_CBC:
		rockchip_io_set_aes_iv(dev, ctx->iv);
	case RK_AES_ECB:
		rockchip_io_set_aes_key(dev, ctx->key, ctx->keylen);
		break;
	}
}

static void rockchip_io_set_aes_data(struct ioctl_ctx *ctx)
{
	dma_addr_t dmain = ctx->dma_in;
	dma_addr_t dmaout = ctx->dma_out;
	int len = ctx->rk_in->len;
	struct crypto_dev *dev = ctx->dev;

	if (len % 4 != 0)
		len = (len / 4 + 1) * 4;

	/* set input data address and data length*/
	CRYPRO_WRITE(dev, CRYPTO_BRDMAS, dmain);
	CRYPRO_WRITE(dev, CRYPTO_BRDMAL, len);

	/* set output data address*/
	CRYPRO_WRITE(dev, CRYPTO_BTDMAS, dmaout);
}

static void rockchip_io_aes_start(struct ioctl_ctx *ctx)
{
	struct crypto_dev *dev = ctx->dev;
	u32 cmd = ctx->rk_algn->name;
	u32 cmode = ctx->rk_aes->cmode;
	u32 keylen = ctx->rk_aes->keylen;
	u32 aes_ctl = 0;
	u32 crypto_ctl = 0;
	u32 conf_ctl = 0;

	/* aes control config */
	if (cmode == RK_DECRYPT)
		aes_ctl |= AES_ENC;

	if (keylen == AES_KEYSIZE_128)
		aes_ctl |= AES_128_BIT_KEY;
	else if (keylen == AES_KEYSIZE_192)
		aes_ctl |= AES_192_BIT_KEY;
	else if (keylen == AES_KEYSIZE_256)
		aes_ctl |= AES_256_BIT_KEY;

	if (cmd == RK_AES_ECB)
		aes_ctl |= AES_ECB_MODE;
	else if (cmd == RK_AES_CBC)
		aes_ctl |= AES_CBC_MODE;
	else if (cmd == RK_AES_CTR)
		aes_ctl |= AES_CTR_MODE;

	/* fifo mode | key change */
	aes_ctl |= AES_FIFO_MODE | AES_KEY_CHANGE;
	/* key byte swap | iv byte swap */
	aes_ctl |= AES_BYTESWAP_KEY | AES_BYTESWAP_IV;

	spin_lock(&dev->lock);
	CRYPRO_WRITE(dev, AES_CTRL, aes_ctl);
	CRYPRO_WRITE(dev, CRYPTO_INTENA, 0);

	/*input swap*/
	conf_ctl |= BYTESWAP_BRFIFO;
	/*output swap*/
	conf_ctl |= BYTESWAP_BTFIFO;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	crypto_ctl |= (BLOCK_START << 16) | BLOCK_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, crypto_ctl);
	spin_unlock(&dev->lock);
}

static void rockchip_io_set_des_iv(struct crypto_dev *dev, u8 *iv)
{
	if (iv)
		memcpy(dev->ioaddr + TDES_IV_0, iv, DES_BLOCK_LEN);
	else
		memset(dev->ioaddr + TDES_IV_0, 0, DES_BLOCK_LEN);
}

static int rockchip_io_set_des_key(struct crypto_dev *dev,
				   u8 *key, u32 len)
{
	if (len != DES_KEY_LEN &&
	    len != TDES_KEY_LEN) {
		pr_err("%s:%d, key len is error!!\n", __func__, __LINE__);
		return -EINVAL;
	}

	memcpy(dev->ioaddr + TDES_KEY1_0, key, len);
	return 0;
}

static void rockchip_io_set_des_data(struct ioctl_ctx *ctx)
{
	dma_addr_t dmain = ctx->dma_in;
	dma_addr_t dmaout = ctx->dma_out;
	int len = ctx->rk_in->len;
	struct crypto_dev *dev = ctx->dev;

	if (len % 4 != 0)
		len = (len / 4 + 1) * 4;

	/* set input data address and data length*/
	CRYPRO_WRITE(dev, CRYPTO_BRDMAS, dmain);
	CRYPRO_WRITE(dev, CRYPTO_BRDMAL, len);

	/* set output data address*/
	CRYPRO_WRITE(dev, CRYPTO_BTDMAS, dmaout);
}

static void rockchip_io_des_start(struct ioctl_ctx *ctx)
{
	struct crypto_dev *dev = ctx->dev;
	u32 cmd = ctx->rk_algn->name;
	u32 cmode = ctx->rk_des->cmode;
	u32 des_ctl = 0;
	u32 crypto_ctl = 0;
	u32 conf_ctl = 0;

	/* des control config */
	if (cmode == RK_DECRYPT)
		des_ctl |= TDES_ENC;

	switch (cmd) {
	case RK_DES_ECB:
		break;
	case RK_DES_CBC:
		des_ctl |= TDES_CHAINMODE;
		break;
	case RK_DES3_EDE_ECB:
		des_ctl |= TDES_SELECT;
		break;
	case RK_DES3_EDE_CBC:
		des_ctl |= TDES_SELECT | TDES_CHAINMODE;
		break;
	case RK_DES3_EEE_ECB:
		des_ctl |= TDES_SELECT | TDES_EEE;
		break;
	case RK_DES3_EEE_CBC:
		des_ctl |= TDES_SELECT | TDES_EEE | TDES_CHAINMODE;
		break;
	}

	/* fifo mode */
	des_ctl |= TDES_FIFO_MODE;
	/* key byte swap | iv byte swap */
	des_ctl |= TDES_BYTESWAP_KEY | TDES_BYTESWAP_IV;

	spin_lock(&dev->lock);
	CRYPRO_WRITE(dev, TDES_CTRL, des_ctl);

	CRYPRO_WRITE(dev, CRYPTO_INTENA, 0);

	/*input swap*/
	conf_ctl |= BYTESWAP_BRFIFO;
	/*output swap*/
	conf_ctl |= BYTESWAP_BTFIFO;
	conf_ctl |= DESSEL;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	crypto_ctl |= (BLOCK_START << 16) | BLOCK_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, crypto_ctl);
	spin_unlock(&dev->lock);
}

static void rockchip_io_set_des_param(struct crypto_dev *dev,
				      struct des_ctx *ctx, int alg)
{
	switch (alg) {
	case RK_DES_CBC:
	case RK_DES3_EDE_CBC:
	case RK_DES3_EEE_CBC:
		rockchip_io_set_des_iv(dev, ctx->iv);
	case RK_DES_ECB:
	case RK_DES3_EDE_ECB:
	case RK_DES3_EEE_ECB:
		rockchip_io_set_des_key(dev, ctx->key, ctx->keylen);
		break;
	}
}

static void rockchip_io_hash_init(struct ioctl_ctx *ctx)
{
	int i;
	u32 reg_status = 0;
	struct crypto_dev *dev = ctx->dev;

	/* write 1 to start flush process,
	 * The process will clear BRFIFO, BTFIFO, and state machine.
	 */
	reg_status = CRYPRO_READ(dev, CRYPTO_CTRL);
	reg_status = reg_status | (HASH_FLUSH << 16) | HASH_FLUSH;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, reg_status);

	/* write 0 to end flush process */
	reg_status = CRYPRO_READ(dev, CRYPTO_CTRL);
	reg_status &= (~HASH_FLUSH);
	reg_status |= (HASH_FLUSH << 16);
	CRYPRO_WRITE(dev, CRYPTO_CTRL, reg_status);

	/* clear out register */
	for (i = 0; i < 8; i++)
		CRYPRO_WRITE(dev, HASH_DOUT_0 + 4 * i, 0);
}

static void rockchip_io_set_hash_data(struct ioctl_ctx *ctx)
{
	dma_addr_t dmain = ctx->dma_in;
	u32 len = ctx->rk_in->len;
	struct crypto_dev *dev = ctx->dev;

	CRYPRO_WRITE(dev, CRYPTO_HRDMAS, dmain);
	/* the length uint is word */
	CRYPRO_WRITE(dev, CRYPTO_HRDMAL, (len + 3) / 4);
	/* the length uint is bytes */
	CRYPRO_WRITE(dev, HASH_MSG_LEN, len);
}

static int rockchip_io_hash_start(struct ioctl_ctx *ctx)
{
	struct crypto_dev *dev = ctx->dev;
	u32 cmd = ctx->rk_algn->name;
	u32 hash_ctl = 0;
	u32 crypto_ctl = 0;
	u32 conf_ctl = 0;

	switch (cmd) {
	case RK_SHA1:
		hash_ctl |= HASH_SHA1;
		break;
	case RK_SHA256:
		hash_ctl |= HASH_SHA256;
		break;
	case RK_MD5:
		hash_ctl |= HASH_MD5;
		break;
	default:
		pr_err("%s:%d, alg is error!\n", __func__, __LINE__);
		return -1;
	}

	spin_lock(&dev->lock);
	hash_ctl |= HASH_SWAP_DO;
	CRYPRO_WRITE(dev, HASH_CTRL, hash_ctl);
	CRYPRO_WRITE(dev, CRYPTO_INTENA, 0);

	conf_ctl |= BYTESWAP_HRFIFO;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	/* crypto ctrl status */
	crypto_ctl |= (HASH_START << 16) | HASH_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, crypto_ctl);
	spin_unlock(&dev->lock);
	return 0;
}

static int rockchip_io_hash_get_out(struct ioctl_ctx *ctx, u8 *out)
{
	int i;
	int len;
	u32 reg_val;
	u32 cmd = ctx->rk_algn->name;
	struct crypto_dev *dev = ctx->dev;

	switch (cmd) {
	case RK_SHA1:
		len = 20;
		break;
	case RK_SHA256:
		len = 32;
		break;
	case RK_MD5:
		len = 16;
		break;
	}

	for (i = 0; i < len / 4; i++) {
		reg_val = CRYPRO_READ(dev, HASH_DOUT_0 + 4 * i);
		memcpy(out + 4 * i, &reg_val, 4);
	}
	return len;
}

static u8 *rockchip_rsa_data_reverse(u8 *indata, int len)
{
	char *start = indata;
	char *end = indata + len - 1;
	char temp;

	if (indata) {
		while (start < end) {
			temp = *start;
			*start++ = *end;
			*end-- = temp;
		}
	}
	return indata;
}

static int rockchip_rsa_write_M(struct crypto_dev *dev,
				u8 *indata, int len, int arg_len)
{
	u8 *m_buf;
	u8 *rev_out;

	if (!dev || (len <= 0) || (len > RSA_REG_LEN)) {
		pr_err("%s:%d, write RSA m error!!\n", __func__, __LINE__);
		return -1;
	}

	rev_out = rockchip_rsa_data_reverse(indata, len);
	m_buf = CRYPTO_GET_REG_VIRT(dev, PKA_M);
	memset(m_buf, 0, RSA_REG_LEN);
	memcpy(m_buf, rev_out, arg_len);

	return 0;
}

static int rockchip_rsa_write_C(struct crypto_dev *dev,
				u8 *indata, int len, int arg_len)
{
	u8 *c_buf;
	u8 *rev_out;

	if (!dev || (len <= 0) || (len > RSA_REG_LEN)) {
		pr_err("%s:%d, write RSA c error!!\n", __func__, __LINE__);
		return -1;
	}

	rev_out = rockchip_rsa_data_reverse(indata, len);
	c_buf = CRYPTO_GET_REG_VIRT(dev, PKA_C);
	memset(c_buf, 0, RSA_REG_LEN);
	memcpy(c_buf, rev_out, arg_len);

	return 0;
}

int rockchip_rsa_write_N(struct crypto_dev *dev,
			 u8 *indata, int len, int arg_len)
{
	u8 *n_buf;
	u8 *rev_out;

	if (!dev || (len <= 0) || (len > RSA_REG_LEN)) {
		pr_err("%s:%d, write RSA n error!!\n", __func__, __LINE__);
		return -1;
	}

	rev_out = rockchip_rsa_data_reverse(indata, len);
	n_buf = CRYPTO_GET_REG_VIRT(dev, PKA_N);
	memset(n_buf, 0, RSA_REG_LEN);
	memcpy(n_buf, rev_out, arg_len);

	return 0;
}

int rockchip_rsa_write_E(struct crypto_dev *dev,
			 u8 *indata, int len, int arg_len)
{
	u8 *e_buf;
	u8 *rev_out;

	if (!dev || (len <= 0) || (len > RSA_REG_LEN)) {
		pr_err("%s:%d, write RSA e error!!\n", __func__, __LINE__);
		return -1;
	}

	rev_out = rockchip_rsa_data_reverse(indata, len);
	e_buf = CRYPTO_GET_REG_VIRT(dev, PKA_E);
	memset(e_buf, 0, RSA_REG_LEN);
	memcpy(e_buf, rev_out, arg_len);

	return 0;
}

static void rockchip_rsa_init(struct ioctl_ctx *ctx)
{
	struct crypto_dev *dev = ctx->dev;
	u32 cmd = ctx->rk_algn->name;
	unsigned long reg_val = 0;
	unsigned long pka_val = 0;

	reg_val |= (PKA_FLUSH << 16) | PKA_FLUSH;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, reg_val);

	reg_val &= (~PKA_FLUSH);
	reg_val |= (PKA_FLUSH << 16);
	CRYPRO_WRITE(dev, CRYPTO_CTRL, reg_val);

	if (cmd == RK_RSA_512_BIT)
		pka_val |= PKA_BLOCK_SIZE_512BIT;
	else if (cmd == RK_RSA_1024_BIT)
		pka_val |= PKA_BLOCK_SIZE_1024BIT;
	else if (cmd == RK_RSA_2048_BIT)
		pka_val |= PKA_BLOCK_SIZE_2048BIT;
	else
		pr_err("%s:%d, crypto is error!!\n", __func__, __LINE__);

	CRYPRO_WRITE(dev, PKA_CTRL, pka_val);
}

static int rockchip_rsa_config(struct ioctl_ctx *ctx, struct rsa_ctx *arg)
{
	u32 ret = 0;
	u32 len = 0;
	u32 cbytes = 0;
	u32 cmd = ctx->rk_algn->name;
	struct crypto_dev *dev = ctx->dev;
	struct rsa_ctx *prsa = NULL;

	if (cmd == RK_RSA_512_BIT)
		len = 64;
	else if (cmd == RK_RSA_1024_BIT)
		len = 128;
	else if (cmd == RK_RSA_2048_BIT)
		len = 256;

	cbytes = sizeof(struct rsa_ctx);
	ctx->rk_rsa = kzalloc(cbytes, GFP_KERNEL);
	if (!ctx->rk_rsa)
		return -ENOMEM;

	if (unlikely(copy_from_user(ctx->rk_rsa, arg, cbytes))) {
		pr_err("%s:%d, copy from user fail\n", __func__, __LINE__);
		ret = -EFAULT;
		goto rsa_err;
	} else {
		prsa = ctx->rk_rsa;
	}

	/* config M*/
	prsa->rsa_m = kzalloc(RSA_DATA_MAX, GFP_KERNEL);
	if (!prsa->rsa_m) {
		ret = -ENOMEM;
		goto rsa_err;
	}
	if (unlikely(copy_from_user(prsa->rsa_m, arg->rsa_m, prsa->mlen))) {
		pr_err("%s:%d, copy from user fail\n", __func__, __LINE__);
		ret = -EFAULT;
		goto m_err;
	}
	ret = rockchip_rsa_write_M(dev, prsa->rsa_m, prsa->mlen, len);
	if (ret) {
		pr_err("%s:%d, write rsa m error!!\n", __func__, __LINE__);
		goto m_err;
	}

	/* config C*/
	prsa->rsa_c = kzalloc(RSA_DATA_MAX, GFP_KERNEL);
	if (!prsa->rsa_c) {
		ret = -ENOMEM;
		goto m_err;
	}
	if (unlikely(copy_from_user(prsa->rsa_c, arg->rsa_c, prsa->clen))) {
		pr_err("%s:%d, copy from user fail\n", __func__, __LINE__);
		ret = -EFAULT;
		goto c_err;
	}
	ret = rockchip_rsa_write_C(dev, prsa->rsa_c, prsa->clen, len);
	if (ret) {
		pr_err("%s:%d, write rsa c error!!\n", __func__, __LINE__);
		goto c_err;
	}

	/* config N*/
	prsa->rsa_n = kzalloc(RSA_DATA_MAX, GFP_KERNEL);
	if (!prsa->rsa_n) {
		ret = -ENOMEM;
		goto c_err;
	}
	if (unlikely(copy_from_user(prsa->rsa_n, arg->rsa_n, prsa->nlen))) {
		pr_err("%s:%d, copy from user fail\n", __func__, __LINE__);
		ret = -EFAULT;
		goto n_err;
	}
	ret = rockchip_rsa_write_N(dev, prsa->rsa_n, prsa->nlen, len);
	if (ret) {
		pr_err("%s:%d, write rsa n error!!\n", __func__, __LINE__);
		goto n_err;
	}

	/* config E*/
	prsa->rsa_e = kzalloc(RSA_DATA_MAX, GFP_KERNEL);
	if (!prsa->rsa_e) {
		ret = -ENOMEM;
		goto n_err;
	}
	if (unlikely(copy_from_user(prsa->rsa_e, arg->rsa_e, prsa->elen))) {
		pr_err("%s:%d, copy from user fail\n", __func__, __LINE__);
		ret = -EFAULT;
		goto e_err;
	}
	ret = rockchip_rsa_write_E(dev, prsa->rsa_e, prsa->elen, len);
	if (ret) {
		pr_err("%s:%d, write rsa e error!!\n", __func__, __LINE__);
		goto e_err;
	}

e_err:
	kfree(prsa->rsa_e);
	prsa->rsa_e = NULL;

n_err:
	kfree(prsa->rsa_n);
	prsa->rsa_n = NULL;

c_err:
	kfree(prsa->rsa_c);
	prsa->rsa_c = NULL;

m_err:
	kfree(prsa->rsa_m);
	prsa->rsa_m = NULL;

	return ret;
rsa_err:
	kfree(ctx->rk_rsa);
	ctx->rk_rsa = NULL;

	return ret;
}

static void rockchip_rsa_start(struct ioctl_ctx *ctx)
{
	struct crypto_dev *dev = ctx->dev;
	unsigned long reg_val = 0;

	CRYPRO_WRITE(dev, CRYPTO_INTENA, 0);

	/* start pka crypto */
	reg_val |= (PKA_START << 16) | PKA_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, reg_val);
}

static int rockchip_io_set_param(struct ioctl_ctx *ctx, void __user *arg)
{
	int ret = 0;
	int cbytes = 0;
	u32 cmd = ctx->rk_algn->name;

	switch (cmd) {
	case RK_AES_ECB:
	case RK_AES_CBC:
	case RK_AES_CTR:
		cbytes = sizeof(struct aes_ctx);
		ctx->rk_aes = kzalloc(cbytes, GFP_KERNEL);
		if (!ctx->rk_aes) {
			ret = -ENOMEM;
			break;
		}

		if (unlikely(copy_from_user(ctx->rk_aes, arg, cbytes))) {
			pr_err("copy aes from user failed\n");
			ret = -EFAULT;
			kfree(ctx->rk_aes);
			break;
		}

		rockchip_io_set_aes_param(ctx->dev, ctx->rk_aes, cmd);
		break;
	case RK_DES_ECB:
	case RK_DES_CBC:
	case RK_DES3_EDE_ECB:
	case RK_DES3_EDE_CBC:
	case RK_DES3_EEE_ECB:
	case RK_DES3_EEE_CBC:
		cbytes = sizeof(struct des_ctx);
		ctx->rk_des = kzalloc(cbytes, GFP_KERNEL);
		if (!ctx->rk_des) {
			ret = -ENOMEM;
			break;
		}

		if (unlikely(copy_from_user(ctx->rk_des, arg, cbytes))) {
			pr_err("copy des from user failed\n");
			ret = -EFAULT;
			kfree(ctx->rk_des);
			break;
		}

		rockchip_io_set_des_param(ctx->dev, ctx->rk_des, cmd);
		break;
	case RK_SHA1:
	case RK_SHA256:
	case RK_MD5:
		break;
	case RK_RSA_512_BIT:
	case RK_RSA_1024_BIT:
	case RK_RSA_2048_BIT:
		break;
	default:
		pr_err("%s:%d, alg is error!\n", __func__, __LINE__);
		break;
	}

	return ret;
}

static int rockchip_io_rsa_input(struct ioctl_ctx *ctx, struct rsa_ctx *arg)
{
	int ret = 0;

	rockchip_rsa_init(ctx);
	ret = rockchip_rsa_config(ctx, arg);
	rockchip_rsa_start(ctx);

	return ret;
}

static int rockchip_rsa_get_out(struct ioctl_ctx *ctx, u8 *outdata)
{
	struct crypto_dev *dev = ctx->dev;
	u32 cmd = ctx->rk_algn->name;
	u32 mlen = ctx->rk_rsa->mlen;
	u8 *m_buf;
	int outlen;

	if (!dev || !outdata) {
		pr_err("%s:%d, point is NULL!!\n", __func__, __LINE__);
		return -1;
	}

	m_buf = CRYPTO_GET_REG_VIRT(dev, PKA_M);

	if (cmd == RK_RSA_512_BIT)
		outlen = 64;
	else if (cmd == RK_RSA_1024_BIT)
		outlen = 128;
	else if (cmd == RK_RSA_2048_BIT)
		outlen = 256;
	else
		pr_err("%s:%d, crypto is error!!\n", __func__, __LINE__);

	memcpy(outdata, m_buf, outlen);
	outdata = rockchip_rsa_data_reverse(outdata, mlen);
	return mlen;
}

static int rockchip_io_copy_input(struct ioctl_ctx *ctx, struct data_in *arg)
{
	int cbytes = 0;
	dma_addr_t input_dma;
	struct crypto_dev *dev = ctx->dev;
	struct data_in *din;

	cbytes = sizeof(struct data_in);
	ctx->rk_in = kzalloc(cbytes, GFP_KERNEL);
	if (!ctx->rk_in)
		return -ENOMEM;

	din = ctx->rk_in;

	if (unlikely(copy_from_user(din, arg, cbytes))) {
		pr_err("%s:%d, copy_from_user failed\n", __func__, __LINE__);
		kfree(din);
		return -EFAULT;
	}

	din->in = dma_alloc_coherent(dev->dev, DMA_BUF_LEN,
			&input_dma, GFP_KERNEL);
	if (!din->in) {
		pr_err("%s:%d, malloc dma buf fail\n", __func__, __LINE__);
		kfree(din);
		return -ENOMEM;
	}

	ctx->dma_in = input_dma;
	memset(din->in, 0, DMA_BUF_LEN);

	if (unlikely(copy_from_user(din->in, arg->in, din->len))) {
		pr_err("copy from user failed, din->len = 0x%x\n", din->len);
		dma_free_coherent(dev->dev, DMA_BUF_LEN, din->in, input_dma);
		din->in = NULL;
		kfree(din);
		return -EFAULT;
	}

#ifdef RK_CRYPTO_DEBUG
	hexdump(din->in, din->len);
#endif
	return 0;
}

static int rockchip_io_malloc_outdma(struct ioctl_ctx *ctx)
{
	u8 *in = ctx->rk_in->in;
	struct crypto_dev *dev = ctx->dev;

	ctx->out = dma_alloc_coherent(dev->dev, DMA_BUF_LEN,
			&ctx->dma_out, GFP_KERNEL);
	if (!ctx->out) {
		pr_err("%s:%d, malloc dma buf fail!!\n", __func__, __LINE__);
		dma_free_coherent(dev->dev, DMA_BUF_LEN, in, ctx->dma_in);
		in = NULL;
		kfree(ctx->rk_in);
		return -ENOMEM;
	}
	memset(ctx->out, 0, DMA_BUF_LEN);

	return 0;
}

static int rockchip_io_set_input(struct ioctl_ctx *ctx, void __user *arg)
{
	int ret = 0;
	u32 cmd = ctx->rk_algn->name;

	switch (cmd) {
	case RK_AES_ECB:
	case RK_AES_CBC:
	case RK_AES_CTR:
		ret = rockchip_io_copy_input(ctx, (struct data_in *)arg);
		if (ret)
			break;

		ret = rockchip_io_malloc_outdma(ctx);
		if (ret)
			break;

		rockchip_io_set_aes_data(ctx);
		rockchip_io_aes_start(ctx);
		break;
	case RK_DES_ECB:
	case RK_DES_CBC:
	case RK_DES3_EDE_ECB:
	case RK_DES3_EDE_CBC:
	case RK_DES3_EEE_ECB:
	case RK_DES3_EEE_CBC:
		ret = rockchip_io_copy_input(ctx, (struct data_in *)arg);
		if (ret)
			break;

		ret = rockchip_io_malloc_outdma(ctx);
		if (ret)
			break;

		rockchip_io_set_des_data(ctx);
		rockchip_io_des_start(ctx);
		break;
	case RK_SHA1:
	case RK_SHA256:
	case RK_MD5:
		ret = rockchip_io_copy_input(ctx, (struct data_in *)arg);
		if (ret)
			break;
		rockchip_io_hash_init(ctx);
		rockchip_io_set_hash_data(ctx);
		rockchip_io_hash_start(ctx);
		break;
	case RK_RSA_512_BIT:
	case RK_RSA_1024_BIT:
	case RK_RSA_2048_BIT:
		ret = rockchip_io_rsa_input(ctx, (struct rsa_ctx *)arg);
		if (ret)
			break;
		break;
	default:
		pr_err("%s:%d, alg is error!\n", __func__, __LINE__);
		break;
	}

	return ret;
}

static int rockchip_io_get_output(struct ioctl_ctx *ctx, void __user *arg)
{
	int ret = 0;
	int hash_len;
	int count = 3;
	u32 reg_val = 0;
	u32 cmd = ctx->rk_algn->name;
	struct crypto_dev *dev = ctx->dev;
	struct data_in *din = ctx->rk_in;
	u8 *hash_out = NULL;
	u8 *rsa_out = NULL;

	switch (cmd) {
	case RK_AES_ECB:
	case RK_AES_CBC:
	case RK_AES_CTR:
	case RK_DES_ECB:
	case RK_DES_CBC:
	case RK_DES3_EDE_ECB:
	case RK_DES3_EDE_CBC:
	case RK_DES3_EEE_ECB:
	case RK_DES3_EEE_CBC:
		while (count--) {
			reg_val = CRYPRO_READ(dev, CRYPTO_CTRL);
			if (reg_val & BLOCK_START)
				usleep_range(10, 20);
			else
				break;
		}

		if (din->in) {
			dma_free_coherent(dev->dev, DMA_BUF_LEN,
					  din->in, ctx->dma_in);
			din->in = NULL;
		}

		if (ctx->out) {
			if (unlikely(copy_to_user(arg, ctx->out, din->len))) {
				pr_err("copy ablk to user fail!\n");
				ret = -EFAULT;
			}

			dma_free_coherent(dev->dev, DMA_BUF_LEN,
					  ctx->out,  ctx->dma_out);
			ctx->out = NULL;
		}
		break;
	case RK_SHA1:
	case RK_SHA256:
	case RK_MD5:
		while (count--) {
			reg_val = CRYPRO_READ(dev, CRYPTO_CTRL);
			if (reg_val & HASH_START)
				usleep_range(10, 20);
			else
				break;
		}

		if (din->in) {
			dma_free_coherent(dev->dev, DMA_BUF_LEN,
					  din->in, ctx->dma_in);
			din->in = NULL;
		}

		hash_out = kzalloc(HASH_OUT_LEN, GFP_KERNEL);
		if (!hash_out)
			return -ENOMEM;

		hash_len = rockchip_io_hash_get_out(ctx, hash_out);
		if (unlikely(copy_to_user(arg, hash_out, hash_len))) {
			pr_err("copy hash to user fail!\n");
			ret = -EFAULT;
		}
		kfree(hash_out);
		break;
	case RK_RSA_512_BIT:
	case RK_RSA_1024_BIT:
	case RK_RSA_2048_BIT:
		while (1) {
			reg_val = CRYPRO_READ(dev, CRYPTO_CTRL);
			if (reg_val & PKA_START)
				usleep_range(10, 20);
			else
				break;
		}

		rsa_out = kzalloc(RSA_DATA_MAX_LEN, GFP_KERNEL);
		if (!rsa_out)
			return  -ENOMEM;

		ret = rockchip_rsa_get_out(ctx, rsa_out);
		if (ret == -1) {
			kfree(rsa_out);
			return -EFAULT;
		}
		if (unlikely(copy_to_user(arg, rsa_out, ret))) {
			pr_err("copy rsa to user fail!!\n");
			return -EFAULT;
		}
		kfree(ctx->rk_rsa);
		ctx->rk_rsa = NULL;
		kfree(rsa_out);
		break;
	default:
		pr_err("%s:%d, alg is error!\n", __func__, __LINE__);
		break;
	}

	return ret;
}

static int rockchip_io_free_buf(struct ioctl_ctx *ctx)
{
	struct crypto_dev *dev = ctx->dev;
	struct data_in *din = ctx->rk_in;

	if (din) {
		if (din->in) {
			dma_free_coherent(dev->dev, DMA_BUF_LEN,
					  din->in, ctx->dma_in);
			din->in = NULL;
		}
		kfree(din);
		din = NULL;
	}

	if (ctx->out) {
		dma_free_coherent(dev->dev, DMA_BUF_LEN,
				  ctx->out,  ctx->dma_out);
		ctx->out = NULL;
	}

	kfree(ctx);
	return 0;
}

/**
 * rk_crypto_ioctl - ioctl handler for sep device
 * @filp: pointer to struct file
 * @cmd: command
 * @arg: pointer to argument structure
 *
 * Implement the ioctl methods available on the rk crypto device.
 */
static long rockchip_crypto_ioctl(struct file *filp, u32 cmd,
				  unsigned long arg)
{
	int ret = 0;
	struct ioctl_ctx *ctx = (struct ioctl_ctx *)(filp->private_data);

#ifdef RK_CRYPTO_DEBUG
	CRYPTO_INFO("cmd : %d", cmd);
#endif

	switch (cmd) {
	case SET_ALG:
		ret = rockchip_io_set_alg(ctx, (void __user *)arg);
		break;
	case SET_PARAM:
		ret = rockchip_io_set_param(ctx, (void __user *)arg);
		break;
	case SET_INPUT:
		ret = rockchip_io_set_input(ctx, (void __user *)arg);
		break;
	case GET_OUTPUT:
		ret = rockchip_io_get_output(ctx, (void __user *)arg);
		break;
	default:
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
/**
 * rockchip_crypto_compat_ioctl - ioctl handler for 64 bit sep device
 * @filp: pointer to struct file
 * @cmd: command
 * @arg: pointer to argument structure
 *
 * Implement the ioctl methods available on the rk crypto device.
 */
static long rockchip_crypto_compat_ioctl(
		struct file *filp, u32 cmd, unsigned long arg)
{
	int ret = 0;
	struct ioctl_ctx *ctx = (struct ioctl_ctx *)(filp->private_data);

#ifdef RK_CRYPTO_DEBUG
	CRYPTO_INFO("cmd : %d", cmd);
#endif

	switch (cmd) {
	case SET_ALG:
		ret = rockchip_io_set_alg(ctx, (void __user *)arg);
		break;
	case SET_PARAM:
		ret = rockchip_io_set_param(ctx, compat_ptr(arg));
		break;
	case SET_INPUT:
		ret = rockchip_io_set_input(ctx, compat_ptr(arg));
		break;
	case GET_OUTPUT:
		ret = rockchip_io_get_output(ctx, compat_ptr(arg));
		break;
	default:
		break;
	}

	return ret;
}
#endif

/**
 * rk_crypto_open - device open method
 * @inode: inode of rk crypto device
 * @filp: file handle to rk crypto device
 *
 * Open method for the rk crypto device. Called when userspace opens
 * the rk crypto device node.
 *
 * Returns zero on success otherwise an error code.
 */
static int rockchip_crypto_open(struct inode *inode, struct file *filp)
{
	struct ioctl_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = cdev;
	filp->private_data = ctx;

	if (rockchip_io_clk_enable(ctx->dev))
		return -ENOENT;

	return 0;
}

/**
 * rk_crypto_release - close a rk crypto device
 * @inode: inode of rk crypto device
 * @filp: file handle being closed
 *
 * Called on the final close of a rk crypto device.
 */
static int rockchip_crypto_release(struct inode *inode, struct file *filp)
{
	struct ioctl_ctx *ctx = (struct ioctl_ctx *)filp->private_data;

	rockchip_io_clk_disable(ctx->dev);
	if (ctx)
		rockchip_io_free_buf(ctx);

	return 0;
}

/**
 * rockchip_crypto_operations - file operation on rk crypto device
 * @rockchip_crypto_open:	handles rk crypto device open request
 * @rockchip_crypto_release: handles rk crypto device release request
 * @rockchip_crypto_ioctl:	ioctl handler from user space call
 * @rockchip_crypto_compat_ioctl:
 *	   ioctl handler from user space call for 64bit system
 */
static const struct file_operations rockchip_crypto_operations = {
	.owner = THIS_MODULE,
	.open = rockchip_crypto_open,
	.release = rockchip_crypto_release,
	.unlocked_ioctl = rockchip_crypto_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = rockchip_crypto_compat_ioctl,
#endif
};

int rockchip_crypto_misc_register(struct crypto_dev *dev)
{
	int ret;

	dev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dev->miscdev.name = MODE_NAME;
	dev->miscdev.mode = 0644;
	dev->miscdev.fops = &rockchip_crypto_operations;

	ret = misc_register(&dev->miscdev);
	if (ret) {
		pr_err("Crypto: Could not add character driver\n");
		return ret;
	}
	return 0;
}
