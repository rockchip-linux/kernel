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
#include <crypto/internal/hash.h>
#include <crypto/md5.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/highmem.h>
#include <linux/rockchip/cru.h>

#include "rk_crypto.h"

#define COMPATIBLE_3368	"rockchip,rk3368-crypto"
#define COMPATIBLE_rockchip	MODE_NAME

#define CRYPTO_QUEUE_LEN	(20)
#define AES_MODE_MASK		(0x3 << 4)
#define DES_MODE_MASK		(0x7 << 2)
#define DECRYPT_MASK		BIT(0)
struct crypto_dev *cdev;

static int rockchip_crypto_clk_enable(struct crypto_dev *dev)
{
	if (clk_prepare_enable(dev->clk)) {
		pr_err("Cann't enable clk_crypto\n");
		return -ENOENT;
	}

	if (clk_prepare_enable(dev->hclk)) {
		pr_err("Cann't enable hclk_crypto\n");
		clk_disable_unprepare(dev->clk);
		return -ENOENT;
	}

	if (!cpu_is_rk312x()) {
		if (clk_prepare_enable(dev->aclk)) {
			pr_err("Cann't enable aclk_crypto\n");
			clk_disable_unprepare(dev->hclk);
			clk_disable_unprepare(dev->clk);
			return -ENOENT;
		}
	}

	return 0;
}

static void rockchip_crypto_clk_disable(struct crypto_dev *dev)
{
	if (!cpu_is_rk312x())
		clk_disable_unprepare(dev->aclk);
	clk_disable_unprepare(dev->hclk);
	clk_disable_unprepare(dev->clk);
}

static void rockchip_unset_outdata(struct crypto_dev *dev)
{
	dma_unmap_sg(dev->dev, dev->sg_dst, 1, DMA_FROM_DEVICE);
}

static void rockchip_unset_indata(struct crypto_dev *dev)
{
	dma_unmap_sg(dev->dev, dev->sg_src, 1, DMA_TO_DEVICE);
}

static void rockchip_ahash_crypto_complete(
				struct crypto_dev *dev, int err)
{
	/* holding a lock outside */
	if (!dev->ahash_req->base.complete)
		dev->ahash_req->base.complete(&dev->ahash_req->base, err);

	dev->busy = false;
}

static void rockchip_ahash_set_dma_indata(
			struct crypto_dev *dev, struct scatterlist *sg)
{
	int len = sg_dma_len(sg);

	CRYPRO_WRITE(dev, CRYPTO_HRDMAS, sg_dma_address(sg));
	/* hash dma length, the length uint is word */
	CRYPRO_WRITE(dev, CRYPTO_HRDMAL, (len + 3) / 4);
	/* the length uint is bytes */
	CRYPRO_WRITE(dev, HASH_MSG_LEN, len);
}

static int rockchip_ahash_set_indata(struct crypto_dev *dev,
				     struct scatterlist *sg)
{
	int err;

	/* size is not SHA1_BLOCK_SIZE aligned */
	if (!IS_ALIGNED(sg_dma_len(sg), 4)) {
		err = -EINVAL;
		goto exit;
	}

	if (!sg_dma_len(sg)) {
		err = -EINVAL;
		goto exit;
	}

	err = dma_map_sg(dev->dev, sg, 1, DMA_TO_DEVICE);
	if (!err) {
		err = -ENOMEM;
		goto exit;
	}

	dev->sg_src = sg;
	err = 0;
exit:
	return err;
}

static void rockchip_ahash_rx(struct crypto_dev *dev)
{
	int err = 0;

	rockchip_unset_indata(dev);

	if (!sg_is_last(dev->sg_src)) {
		err = rockchip_ahash_set_indata(dev, sg_next(dev->sg_src));
		if (err) {
			rockchip_ahash_crypto_complete(dev, err);
			return;
		}

		rockchip_ahash_set_dma_indata(dev, dev->sg_src);
	} else {
		pr_info("%s:%d, sg is last!!\n", __func__, __LINE__);
		rockchip_ahash_crypto_complete(dev, err);
	}
}

static void rockchip_ahash_restart(struct crypto_dev *dev)
{
	u32 irq_sts = 0;
	u32 ctrl_status = 0;
	u32 conf_ctl = 0;

	irq_sts = HASH_DONE_ENA | HRDMA_ERR_ENA | HRDMA_DONE_ENA;
	CRYPRO_WRITE(dev, CRYPTO_INTENA, irq_sts);

	conf_ctl |= BYTESWAP_HRFIFO;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	/* crypto ctrl status */
	ctrl_status |= (HASH_START << 16) | HASH_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, ctrl_status);
}

static int rockchip_ahash_start(struct crypto_dev *dev,
				unsigned long mode)
{
	struct ahash_request *hash_req = dev->ahash_req;
	u32 hash_status = 0;
	u32 ctrl_status = 0;
	u32 conf_ctl = 0;
	int err = 0;

	/* dma */
	spin_lock(&dev->lock);
	err = rockchip_ahash_set_indata(dev, hash_req->src);
	if (err) {
		pr_err("[%s:%d]set indata error\n", __func__, __LINE__);
		goto indata_error;
	}

	rockchip_ahash_set_dma_indata(dev, hash_req->src);

	CRYPRO_WRITE(dev, CRYPTO_INTENA, HASH_DONE_ENA |
			HRDMA_ERR_ENA | HRDMA_DONE_ENA);

	hash_status |= mode;
	hash_status |= HASH_SWAP_DO;
	CRYPRO_WRITE(dev, HASH_CTRL, hash_status);

	conf_ctl |= BYTESWAP_HRFIFO;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	/* crypto ctrl status */
	ctrl_status |= (HASH_START << 16) | HASH_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, ctrl_status);
	spin_unlock(&dev->lock);
	return err;

indata_error:
	rockchip_ahash_crypto_complete(dev, err);
	spin_unlock(&dev->lock);
	return err;
}

static int rockchip_ahash_handle_req(struct crypto_dev *dev,
				     struct ahash_request *req)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->busy) {
		ret = -EAGAIN;
		spin_unlock_irqrestore(&dev->lock, flags);
		goto exit;
	}
	dev->busy = true;

	ret = ahash_enqueue_request(&dev->queue, req);
	spin_unlock_irqrestore(&dev->lock, flags);

	tasklet_schedule(&dev->crypto_tasklet);

exit:
	return ret;
}

static void rockchip_ahash_hw_init(struct crypto_dev *dev)
{
	int i;
	u32 reg_status = 0;

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

static int rockchip_ahash_final(struct ahash_request *req)
{
	struct rk_ahash_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_dev *dev = tctx->dev;
	u32 reg_val = 0;
	int len;
	int i;

	if (dev->flags == RK_SHA1)
		len = 20;
	else if (dev->flags == RK_SHA256)
		len = 32;
	else if (dev->flags == RK_MD5)
		len = 16;

	for (i = 0; i < len / 4; i++) {
		reg_val = CRYPRO_READ(dev, HASH_DOUT_0 + 4 * i);
		memcpy(req->result + 4 * i, &reg_val, 4);
	}

	return 0;
}

static int rockchip_ahash_update(struct ahash_request *req)
{
	struct rk_ahash_reqctx *reqctx = ahash_request_ctx(req);
	struct rk_ahash_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct crypto_dev *dev = tctx->dev;

	if (!req->nbytes)
		return 0;

	reqctx->total = req->nbytes;

	return rockchip_ahash_handle_req(dev, req);
}

static int rockchip_ahash_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct rk_ahash_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct rk_ahash_reqctx *reqctx = ahash_request_ctx(req);
	struct crypto_dev *dev = tctx->dev;
	int rk_digest_size;

	reqctx->mode = 0;

	rk_digest_size = crypto_ahash_digestsize(tfm);
	CRYPTO_INFO("init: digest size: %d", rk_digest_size);

	if (rk_digest_size == SHA1_DIGEST_SIZE) {
		reqctx->mode |= HASH_SHA1;
		dev->flags = RK_SHA1;
	} else if (rk_digest_size == SHA256_DIGEST_SIZE) {
		reqctx->mode |= HASH_SHA256;
		dev->flags = RK_SHA256;
	} else if (rk_digest_size == MD5_DIGEST_SIZE) {
		reqctx->mode |= HASH_MD5;
		dev->flags = RK_MD5;
	}

	rockchip_ahash_hw_init(dev);
	return 0;
}

static int rockchip_ahash_cra_init(struct crypto_tfm *tfm)
{
	struct rk_ahash_ctx *tctx = crypto_tfm_ctx(tfm);

	tctx->dev = cdev;
	if (rockchip_crypto_clk_enable(tctx->dev))
		return -ENOENT;

	return 0;
}

static void rockchip_ahash_cra_exit(struct crypto_tfm *tfm)
{
	struct rk_ahash_ctx *tctx = crypto_tfm_ctx(tfm);

	rockchip_crypto_clk_disable(tctx->dev);
}

static struct ahash_alg hash_algs[] = {
	{
		.init = rockchip_ahash_init,
		.update = rockchip_ahash_update,
		.final = rockchip_ahash_final,
		.halg.digestsize = SHA1_DIGEST_SIZE,
		.halg.statesize = sizeof(struct rk_ahash_ctx),
		.halg.base = {
			.cra_name = "sha1",
			.cra_driver_name = "rk-sha1",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH |
						CRYPTO_ALG_ASYNC,
			.cra_blocksize = SHA1_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct rk_ahash_ctx),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = rockchip_ahash_cra_init,
			.cra_exit = rockchip_ahash_cra_exit,
		}
	},
	{
		.init = rockchip_ahash_init,
		.update = rockchip_ahash_update,
		.final = rockchip_ahash_final,
		.halg.statesize = sizeof(struct rk_ahash_ctx),
		.halg.digestsize = MD5_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "md5",
			.cra_driver_name = "rk-md5",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize = SHA1_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct rk_ahash_ctx),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = rockchip_ahash_cra_init,
			.cra_exit = rockchip_ahash_cra_exit,
		}
	},
	{
		.init = rockchip_ahash_init,
		.update = rockchip_ahash_update,
		.final = rockchip_ahash_final,
		.halg.statesize = sizeof(struct rk_ahash_ctx),
		.halg.digestsize = SHA256_DIGEST_SIZE,
		.halg.base = {
			.cra_name = "sha256",
			.cra_driver_name = "rk-sha256",
			.cra_priority = 100,
			.cra_flags = CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize = SHA256_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct rk_ahash_ctx),
			.cra_alignmask = 0,
			.cra_module = THIS_MODULE,
			.cra_init = rockchip_ahash_cra_init,
			.cra_exit = rockchip_ahash_cra_exit,
		}
	},
};

static int rockchip_register_hash(void)
{
	int i, j;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(hash_algs); i++) {
		err = crypto_register_ahash(&hash_algs[i]);
		if (err)
			goto err_hash_algs;
	}

	return err;
err_hash_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_ahash(&hash_algs[j]);
	return err;
}

static void rockchip_unregister_hash(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hash_algs); i++)
		crypto_unregister_ahash(&hash_algs[i]);
}

static void rockchip_crypto_complete(struct crypto_dev *dev, int err)
{
	/* holding a lock outside */
	if (!dev->ablk_req->base.complete)
		dev->ablk_req->base.complete(&dev->ablk_req->base, err);

	dev->busy = false;
}

static void rockchip_ablk_set_dma_indata(struct crypto_dev *dev,
					 struct scatterlist *sg)
{
	uint word_num = (sg_dma_len(sg) + 3) / 4;

	CRYPRO_WRITE(dev, CRYPTO_BRDMAS, sg_dma_address(sg));
	/* block dma length, the length uint is word! */
	CRYPRO_WRITE(dev, CRYPTO_BRDMAL, word_num);
}

static void rockchip_ablk_set_dma_outdata(struct crypto_dev *dev,
					  struct scatterlist *sg)
{
	CRYPRO_WRITE(dev, CRYPTO_BTDMAS, sg_dma_address(sg));
}

static int rockchip_handle_req(struct crypto_dev *dev,
			       struct ablkcipher_request *req)
{
	unsigned long flags;
	int err;

	spin_lock_irqsave(&dev->lock, flags);
	if (dev->busy) {
		err = -EAGAIN;
		spin_unlock_irqrestore(&dev->lock, flags);
		goto exit;
	}
	dev->busy = true;

	err = ablkcipher_enqueue_request(&dev->queue, req);
	spin_unlock_irqrestore(&dev->lock, flags);

	tasklet_schedule(&dev->crypto_tasklet);
 exit:
	return err;
}

static int rockchip_des_set_outdata(struct crypto_dev *dev,
				    struct scatterlist *sg)
{
	int err;

	if (!IS_ALIGNED(sg_dma_len(sg), DES_BLOCK_SIZE)) {
		err = -EINVAL;
		goto exit;
	}

	if (!sg_dma_len(sg)) {
		err = -EINVAL;
		goto exit;
	}

	err = dma_map_sg(dev->dev, sg, 1, DMA_FROM_DEVICE);
	if (!err) {
		err = -ENOMEM;
		goto exit;
	}

	dev->sg_dst = sg;
	err = 0;

 exit:
	return err;
}

static int rockchip_des_set_indata(struct crypto_dev *dev,
				   struct scatterlist *sg)
{
	int err;

	if (!IS_ALIGNED(sg_dma_len(sg), DES_BLOCK_SIZE)) {
		err = -EINVAL;
		goto exit;
	}

	if (!sg_dma_len(sg)) {
		err = -EINVAL;
		goto exit;
	}

	err = dma_map_sg(dev->dev, sg, 1, DMA_TO_DEVICE);
	if (!err) {
		err = -ENOMEM;
		goto exit;
	}

	dev->sg_src = sg;
	err = 0;
 exit:
	return err;
}

static void rockchip_des_get_iv(struct crypto_dev *dev)
{
	struct scatterlist *sg;
	u8 *buf;
	u8 *iv = dev->des_ctx->iv;
	u32 slen;

	sg = dev->sg_dst;
	buf = kmap((struct page *)(sg[0].page_link)) + sg[0].offset;
	slen = dev->sg_src[0].length;
	memcpy(iv, &buf[slen - DES_BLOCK_SIZE], DES_BLOCK_SIZE);
}

static void rockchip_des_tx(struct crypto_dev *dev)
{
	int err = 0;

	rockchip_unset_outdata(dev);

	if (!sg_is_last(dev->sg_dst)) {
		err = rockchip_des_set_outdata(dev, sg_next(dev->sg_dst));
		if (err) {
			rockchip_crypto_complete(dev, err);
			return;
		}

		rockchip_ablk_set_dma_outdata(dev, dev->sg_dst);
	} else {
		rockchip_crypto_complete(dev, err);
	}
}

static void rockchip_des_rx(struct crypto_dev *dev)
{
	int err;

	if ((dev->flags == RK_DES_CBC) ||
	    (dev->flags == RK_DES3_EDE_CBC) ||
	    (dev->flags == RK_DES3_EEE_CBC))
		rockchip_des_get_iv(dev);

	rockchip_unset_indata(dev);

	if (!sg_is_last(dev->sg_src)) {
		err = rockchip_des_set_indata(dev, sg_next(dev->sg_src));
		if (err) {
			rockchip_crypto_complete(dev, err);
			return;
		}

		rockchip_ablk_set_dma_indata(dev, dev->sg_src);
	} else {
		pr_info("%s:%d, sg is last!!\n", __func__, __LINE__);
	}
}

static void rockchip_des_set_param(struct crypto_dev *dev,
				   u8 *key, u8 *iv, u32 keylen)
{
	if (iv)
		memcpy(dev->ioaddr + TDES_IV_0, iv, DES_BLOCK_SIZE);
	if (key)
		memcpy(dev->ioaddr + TDES_KEY1_0, key, keylen);
}

static void rockchip_des_restart(struct crypto_dev *dev)
{
	u32 conf_ctl = 0;
	u32 crypto_ctl = 0;
	u8 *iv = dev->des_ctx->iv;

	CRYPRO_WRITE(dev, CRYPTO_INTENA, BCDMA_ERR_ENA | BCDMA_DONE_ENA);

	if ((dev->flags == RK_DES_CBC) ||
	    (dev->flags == RK_DES3_EDE_CBC) ||
	    (dev->flags == RK_DES3_EEE_CBC))
		memcpy(dev->ioaddr + TDES_IV_0, iv, DES_BLOCK_SIZE);

	conf_ctl |= BYTESWAP_BRFIFO;
	conf_ctl |= BYTESWAP_BTFIFO;
	conf_ctl |= DESSEL;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	crypto_ctl |= (BLOCK_START << 16) | BLOCK_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, crypto_ctl);
}

static int rockchip_des_start(struct crypto_dev *dev, unsigned long mode)
{
	struct ablkcipher_request *req = dev->ablk_req;
	u8 *des_key = dev->des_ctx->des_key;
	u32 keylen = dev->des_ctx->keylen;
	u32 des_ctl = 0;
	u32 crypto_ctl = 0;
	u32 conf_ctl = 0;
	int ret = 0;

	/* des control config */
	if (mode & DECRYPT_MASK)
		des_ctl |= TDES_ENC;

	if (dev->flags == RK_DES_CBC)
		des_ctl |= TDES_CHAINMODE;
	else if (dev->flags == RK_DES3_EDE_ECB)
		des_ctl |= TDES_SELECT;
	else if (dev->flags == RK_DES3_EDE_CBC)
		des_ctl |= TDES_SELECT | TDES_CHAINMODE;
	else if (dev->flags == RK_DES3_EEE_ECB)
		des_ctl |= TDES_SELECT | TDES_EEE;
	else if (dev->flags == RK_DES3_EEE_CBC)
		des_ctl |= TDES_SELECT | TDES_EEE | TDES_CHAINMODE;

	/* fifo mode */
	des_ctl |= TDES_FIFO_MODE;
	/* key byte swap | iv byte swap */
	des_ctl |= TDES_BYTESWAP_KEY | TDES_BYTESWAP_IV;

	spin_lock(&dev->lock);
	CRYPRO_WRITE(dev, TDES_CTRL, des_ctl);

	ret = rockchip_des_set_indata(dev, req->src);
	if (ret)
		goto indata_error;
	ret = rockchip_des_set_outdata(dev, req->dst);
	if (ret)
		goto outdata_error;
	rockchip_des_set_param(dev, des_key, req->info, keylen);
	rockchip_ablk_set_dma_indata(dev, req->src);
	rockchip_ablk_set_dma_outdata(dev, req->dst);

	/* set block cipher interrupt */
	CRYPRO_WRITE(dev, CRYPTO_INTENA, BCDMA_ERR_ENA | BCDMA_DONE_ENA);

	/*input swap*/
	conf_ctl |= BYTESWAP_BRFIFO;
	/*output swap*/
	conf_ctl |= BYTESWAP_BTFIFO;
	conf_ctl |= DESSEL;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	crypto_ctl |= (BLOCK_START << 16) | BLOCK_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, crypto_ctl);
	spin_unlock(&dev->lock);

	return ret;
outdata_error:
	rockchip_unset_indata(dev);

indata_error:
	rockchip_crypto_complete(dev, ret);
	spin_unlock(&dev->lock);
	return ret;
}

static int rockchip_des_crypt(struct ablkcipher_request *req,
			      unsigned long mode)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(req);
	struct rk_des_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	struct rk_crypto_reqctx *reqctx = ablkcipher_request_ctx(req);
	struct crypto_dev *dev = ctx->dev;
	unsigned long mask = mode >> 2;

	if (!IS_ALIGNED(req->nbytes, DES_BLOCK_SIZE)) {
		pr_err("request size is not exact amount of TDES blocks\n");
		return -EINVAL;
	}

	CRYPTO_INFO("mode = 0x%lx", mode);

	if ((mask & DES_MODE_MASK) == 0x0)
		dev->flags = RK_DES_ECB;
	else if ((mask & DES_MODE_MASK) == 0x01)
		dev->flags = RK_DES3_EDE_ECB;
	else if ((mask & DES_MODE_MASK) == 0x03)
		dev->flags = RK_DES3_EEE_ECB;
	else if ((mask & DES_MODE_MASK) == 0x04)
		dev->flags = RK_DES_CBC;
	else if ((mask & DES_MODE_MASK) == 0x05)
		dev->flags = RK_DES3_EDE_CBC;
	else if ((mask & DES_MODE_MASK) == 0x07)
		dev->flags = RK_DES3_EEE_CBC;

	reqctx->mode = mode;

	return rockchip_handle_req(dev, req);
}

static int rockchip_des_ecb_encrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, 0);
}

static int rockchip_des_ecb_decrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_ENC);
}

static int rockchip_des_cbc_encrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_CHAINMODE);
}

static int rockchip_des_cbc_decrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_ENC | TDES_CHAINMODE);
}

static int rockchip_des_ede_ecb_encrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_SELECT);
}

static int rockchip_des_ede_ecb_decrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_ENC | TDES_SELECT);
}

static int rockchip_des_ede_cbc_encrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_SELECT | TDES_CHAINMODE);
}

static int rockchip_des_ede_cbc_decrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req,
		TDES_ENC | TDES_SELECT | TDES_CHAINMODE);
}

static int rockchip_des_eee_ecb_encrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_SELECT | TDES_EEE);
}

static int rockchip_des_eee_ecb_decrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req,
		TDES_ENC | TDES_SELECT | TDES_EEE);
}

static int rockchip_des_eee_cbc_encrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req,
			TDES_SELECT | TDES_EEE | TDES_CHAINMODE);
}

static int rockchip_des_eee_cbc_decrypt(struct ablkcipher_request *req)
{
	return rockchip_des_crypt(req, TDES_ENC |
		TDES_SELECT | TDES_EEE | TDES_CHAINMODE);
}

static int rockchip_des_setkey(struct crypto_ablkcipher *cipher,
			       const u8 *key, u32 keylen)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct rk_des_ctx *ctx = crypto_tfm_ctx(tfm);

	if (!key)
		return -EINVAL;

	if (keylen != DES_KEY_SIZE &&
	    keylen != DES3_EDE_KEY_SIZE)
		return -EINVAL;

	memcpy(ctx->des_key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}

static int rockchip_des_cra_init(struct crypto_tfm *tfm)
{
	struct rk_des_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->dev = cdev;

	if (rockchip_crypto_clk_enable(ctx->dev))
		return -ENOENT;

	tfm->crt_ablkcipher.reqsize = sizeof(struct rk_crypto_reqctx);

	return 0;
}

static void rockchip_des_cra_exit(struct crypto_tfm *tfm)
{
	struct rk_des_ctx *ctx = crypto_tfm_ctx(tfm);

	rockchip_crypto_clk_disable(ctx->dev);
}

static struct crypto_alg des_algs[] = {
	{
		.cra_name = "ecb(des)",
		.cra_driver_name = "ecb-des-rk",
		.cra_priority = 100,
		/* algorithm types : ASYNC or NONAYNC */
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		/* block size : byte */
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_des_ctx),
		/* 16 bytes align */
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_des_cra_init,
		.cra_exit = rockchip_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.setkey = rockchip_des_setkey,
			.encrypt = rockchip_des_ecb_encrypt,
			.decrypt = rockchip_des_ecb_decrypt,
		}
	},
	{
		.cra_name = "cbc(des)",
		.cra_driver_name = "cbc-des-rk",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_des_ctx),
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_des_cra_init,
		.cra_exit = rockchip_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = DES_KEY_SIZE,
			.max_keysize = DES_KEY_SIZE,
			.ivsize = DES_BLOCK_SIZE,
			.setkey = rockchip_des_setkey,
			.encrypt = rockchip_des_cbc_encrypt,
			.decrypt = rockchip_des_cbc_decrypt,
		}
	},
	{
		.cra_name = "ecb(tdes_ede)",
		.cra_driver_name = "ecb-tdes-ede-rk",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_des_ctx),
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_des_cra_init,
		.cra_exit = rockchip_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rockchip_des_setkey,
			.encrypt = rockchip_des_ede_ecb_encrypt,
			.decrypt = rockchip_des_ede_ecb_decrypt,
		}
	},
	{
		.cra_name = "cbc(tdes_ede)",
		.cra_driver_name = "cbc-tdes_ede-rk",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_des_ctx),
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_des_cra_init,
		.cra_exit = rockchip_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.ivsize = DES_BLOCK_SIZE,
			.setkey = rockchip_des_setkey,
			.encrypt = rockchip_des_ede_cbc_encrypt,
			.decrypt = rockchip_des_ede_cbc_decrypt,
		}
	},
	{
		.cra_name = "ecb(tdes_eee)",
		.cra_driver_name = "ecb-tdes-eee-rk",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_des_ctx),
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_des_cra_init,
		.cra_exit = rockchip_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.setkey = rockchip_des_setkey,
			.encrypt = rockchip_des_eee_ecb_encrypt,
			.decrypt = rockchip_des_eee_ecb_decrypt,
		}
	},
	{
		.cra_name = "cbc(tdes_eee)",
		.cra_driver_name = "cbc-tdes_eee-rk",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize = DES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_des_ctx),
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_des_cra_init,
		.cra_exit = rockchip_des_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = DES3_EDE_KEY_SIZE,
			.max_keysize = DES3_EDE_KEY_SIZE,
			.ivsize = DES_BLOCK_SIZE,
			.setkey = rockchip_des_setkey,
			.encrypt = rockchip_des_eee_cbc_encrypt,
			.decrypt = rockchip_des_eee_cbc_decrypt,
		}
	},
};

static int rockchip_des_register(void)
{
	int i, j;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(des_algs); i++) {
		err = crypto_register_alg(&des_algs[i]);
		if (err)
			goto err_des_algs;
	}

	return err;
err_des_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_alg(&des_algs[j]);
	return err;
}

static void rockchip_des_unregister(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(des_algs); i++)
		crypto_unregister_alg(&des_algs[i]);
}

static int rockchip_aes_set_outdata(struct crypto_dev *dev,
				    struct scatterlist *sg)
{
	int err;

	if (!IS_ALIGNED(sg_dma_len(sg), AES_BLOCK_SIZE)) {
		err = -EINVAL;
		goto exit;
	}

	if (!sg_dma_len(sg)) {
		err = -EINVAL;
		goto exit;
	}

	err = dma_map_sg(dev->dev, sg, 1, DMA_FROM_DEVICE);
	if (!err) {
		err = -ENOMEM;
		goto exit;
	}

	dev->sg_dst = sg;
	err = 0;

 exit:
	return err;
}

static int rockchip_aes_set_indata(struct crypto_dev *dev,
				   struct scatterlist *sg)
{
	int err;

	if (!IS_ALIGNED(sg_dma_len(sg), AES_BLOCK_SIZE)) {
		err = -EINVAL;
		goto exit;
	}

	if (!sg_dma_len(sg)) {
		err = -EINVAL;
		goto exit;
	}

	err = dma_map_sg(dev->dev, sg, 1, DMA_TO_DEVICE);
	if (!err) {
		err = -ENOMEM;
		goto exit;
	}

	dev->sg_src = sg;
	err = 0;
 exit:
	return err;
}

static void rockchip_aes_get_iv(struct crypto_dev *dev)
{
	struct scatterlist *sg;
	u8 *buf;
	u8 *iv = dev->aes_ctx->iv;
	u32 slen;

	sg = dev->sg_dst;
	buf = kmap((struct page *)(sg[0].page_link)) + sg[0].offset;
	slen = dev->sg_src[0].length;
	memcpy(iv, &buf[slen - AES_BLOCK_SIZE], AES_BLOCK_SIZE);
}

static void rockchip_aes_tx(struct crypto_dev *dev)
{
	int err = 0;

	rockchip_unset_outdata(dev);

	if (!sg_is_last(dev->sg_dst)) {
		err = rockchip_aes_set_outdata(dev, sg_next(dev->sg_dst));
		if (err) {
			rockchip_crypto_complete(dev, err);
			return;
		}

		rockchip_ablk_set_dma_outdata(dev, dev->sg_dst);
	} else {
		rockchip_crypto_complete(dev, err);
	}
}

static void rockchip_aes_rx(struct crypto_dev *dev)
{
	int err;

	if (dev->flags == RK_AES_CBC)
		rockchip_aes_get_iv(dev);

	rockchip_unset_indata(dev);

	if (!sg_is_last(dev->sg_src)) {
		err = rockchip_aes_set_indata(dev, sg_next(dev->sg_src));
		if (err) {
			rockchip_crypto_complete(dev, err);
			return;
		}

		rockchip_ablk_set_dma_indata(dev, dev->sg_src);
	} else {
		pr_info("%s:%d, sg is last!!\n", __func__, __LINE__);
	}
}

static void rockchip_aes_set_param(struct crypto_dev *dev,
				   u8 *key, u8 *iv, u32 keylen)
{
	if (iv)
		memcpy(dev->ioaddr + AES_IV_0, iv, AES_BLOCK_SIZE);
	if (key)
		memcpy(dev->ioaddr + AES_KEY_0, key, keylen);
}

static int rockchip_aes_start(struct crypto_dev *dev, unsigned long mode)
{
	struct ablkcipher_request *req = dev->ablk_req;
	u8 *aes_key = dev->aes_ctx->aes_key;
	u32 keylen = dev->aes_ctx->keylen;
	u32 aes_ctl = 0;
	u32 crypto_ctl = 0;
	u32 conf_ctl = 0;
	u32 irq_sts = 0;
	int ret = 0;

	/* aes control config */
	if (mode & DECRYPT_MASK)
		aes_ctl |= AES_ENC;

	if (keylen == AES_KEYSIZE_192)
		aes_ctl |= AES_192_BIT_KEY;
	else if (keylen == AES_KEYSIZE_256)
		aes_ctl |= AES_256_BIT_KEY;

	if (dev->flags == RK_AES_CBC)
		aes_ctl |= AES_CBC_MODE;
	else if (dev->flags == RK_AES_CTR)
		aes_ctl |= AES_CTR_MODE;

	/* fifo mode | key change */
	aes_ctl |= AES_FIFO_MODE | AES_KEY_CHANGE;
	/* key byte swap | iv byte swap */
	aes_ctl |= AES_BYTESWAP_KEY | AES_BYTESWAP_IV;

	spin_lock(&dev->lock);
	CRYPRO_WRITE(dev, AES_CTRL, aes_ctl);

	ret = rockchip_aes_set_indata(dev, req->src);
	if (ret)
		goto indata_error;
	ret = rockchip_aes_set_outdata(dev, req->dst);
	if (ret)
		goto outdata_error;
	rockchip_aes_set_param(dev, aes_key, req->info, keylen);
	rockchip_ablk_set_dma_indata(dev, req->src);
	rockchip_ablk_set_dma_outdata(dev, req->dst);

	/* set block cipher interrupt */
	irq_sts = BCDMA_ERR_ENA | BCDMA_DONE_ENA;
	CRYPRO_WRITE(dev, CRYPTO_INTENA, irq_sts);

	/*input swap*/
	conf_ctl |= BYTESWAP_BRFIFO;
	/*output swap*/
	conf_ctl |= BYTESWAP_BTFIFO;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	crypto_ctl |= (BLOCK_START << 16) | BLOCK_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, crypto_ctl);
	spin_unlock(&dev->lock);

	return ret;
outdata_error:
	rockchip_unset_indata(dev);

indata_error:
	rockchip_crypto_complete(dev, ret);
	spin_unlock(&dev->lock);
	return ret;
}

static void rockchip_aes_restart(struct crypto_dev *dev)
{
	u32 conf_ctl = 0;
	u32 crypto_ctl = 0;
	u8 *iv = dev->aes_ctx->iv;

	CRYPRO_WRITE(dev, CRYPTO_INTENA, BCDMA_ERR_ENA | BCDMA_DONE_ENA);

	if (dev->flags == RK_AES_CBC)
		memcpy(dev->ioaddr + AES_IV_0, iv, AES_BLOCK_SIZE);

	conf_ctl |= BYTESWAP_BRFIFO;
	conf_ctl |= BYTESWAP_BTFIFO;
	CRYPRO_WRITE(dev, CRYPTO_CONF, conf_ctl);

	crypto_ctl |= (BLOCK_START << 16) | BLOCK_START;
	CRYPRO_WRITE(dev, CRYPTO_CTRL, crypto_ctl);
}

static int rockchip_aes_crypt(struct ablkcipher_request *req,
			      unsigned long mode)
{
	struct crypto_ablkcipher *tfm = crypto_ablkcipher_reqtfm(req);
	struct rk_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	struct rk_crypto_reqctx *reqctx = ablkcipher_request_ctx(req);
	struct crypto_dev *dev = ctx->dev;

	if (!IS_ALIGNED(req->nbytes, AES_BLOCK_SIZE)) {
		pr_err("request size is not exact amount of TDES blocks\n");
		return -EINVAL;
	}

	if ((mode & AES_MODE_MASK) == AES_ECB_MODE)
		dev->flags = RK_AES_ECB;
	else if ((mode & AES_MODE_MASK) == AES_CBC_MODE)
		dev->flags = RK_AES_CBC;
	else if ((mode & AES_MODE_MASK) == AES_CTR_MODE)
		dev->flags = RK_AES_CTR;

	reqctx->mode = mode;

	return rockchip_handle_req(dev, req);
}

static int rockchip_aes_cra_init(struct crypto_tfm *tfm)
{
	struct rk_aes_ctx *ctx = crypto_tfm_ctx(tfm);

	ctx->dev = cdev;

	if (rockchip_crypto_clk_enable(ctx->dev))
		return -ENOENT;

	tfm->crt_ablkcipher.reqsize = sizeof(struct rk_crypto_reqctx);

	return 0;
}

static void rockchip_aes_cra_exit(struct crypto_tfm *tfm)
{
	struct rk_aes_ctx *ctx = crypto_tfm_ctx(tfm);

	rockchip_crypto_clk_disable(ctx->dev);
}

static int rockchip_aes_setkey(struct crypto_ablkcipher *cipher,
			       const u8 *key, u32 keylen)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct rk_aes_ctx *ctx = crypto_tfm_ctx(tfm);

	if (!key)
		return -EINVAL;

	if (keylen != AES_KEYSIZE_128 &&
	    keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256){
		return -EINVAL;
	}

	memcpy(ctx->aes_key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}

static int rockchip_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	return rockchip_aes_crypt(req, AES_ECB_MODE);
}

static int rockchip_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	return rockchip_aes_crypt(req, AES_ENC | AES_ECB_MODE);
}

static int rockchip_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	return rockchip_aes_crypt(req, AES_CBC_MODE);
}

static int rockchip_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	return rockchip_aes_crypt(req, AES_ENC | AES_CBC_MODE);
}

static int rockchip_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	return rockchip_aes_crypt(req, AES_CTR_MODE);
}

static int rockchip_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	return rockchip_aes_crypt(req, AES_ENC | AES_CTR_MODE);
}

static struct crypto_alg aes_algs[] = {
	{
		.cra_name = "ecb(aes)",
		.cra_driver_name = "ecb-aes-rk",
		.cra_priority = 100,
		 /* algorithm types : ASYNC or NONAYNC */
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		/* block size : byte */
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_aes_ctx),
		/* 16 bytes align */
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_aes_cra_init,
		.cra_exit = rockchip_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.setkey = rockchip_aes_setkey,
			.encrypt = rockchip_aes_ecb_encrypt,
			.decrypt = rockchip_aes_ecb_decrypt,
		}
	},
	{
		.cra_name = "cbc(aes)",
		.cra_driver_name = "cbc-aes-rk",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_aes_ctx),
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_aes_cra_init,
		.cra_exit = rockchip_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
			.setkey = rockchip_aes_setkey,
			.encrypt = rockchip_aes_cbc_encrypt,
			.decrypt = rockchip_aes_cbc_decrypt,
		}
	},
	{
		.cra_name = "ctr(aes)",
		.cra_driver_name = "ctr-aes-rk",
		.cra_priority = 100,
		.cra_flags = CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
		.cra_blocksize = AES_BLOCK_SIZE,
		.cra_ctxsize = sizeof(struct rk_aes_ctx),
		.cra_alignmask = 0x0f,
		.cra_type = &crypto_ablkcipher_type,
		.cra_module = THIS_MODULE,
		.cra_init = rockchip_aes_cra_init,
		.cra_exit = rockchip_aes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize = AES_MIN_KEY_SIZE,
			.max_keysize = AES_MAX_KEY_SIZE,
			.ivsize = AES_BLOCK_SIZE,
			.setkey = rockchip_aes_setkey,
			.encrypt = rockchip_aes_ctr_encrypt,
			.decrypt = rockchip_aes_ctr_decrypt,
		}
	},
};

static int rockchip_aes_register(void)
{
	int i, j;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(aes_algs); i++) {
		err = crypto_register_alg(&aes_algs[i]);
		if (err)
			goto err_aes_algs;
	}

	return err;
err_aes_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_alg(&aes_algs[j]);
	return err;
}

static void rockchip_aes_unregister(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(aes_algs); i++)
		crypto_unregister_alg(&aes_algs[i]);
}

static irqreturn_t rockchip_crypto_interrupt(int irq, void *dev_id)
{
	struct crypto_dev *dev  = platform_get_drvdata(dev_id);
	u32 irq_sts;
	u32 conf_sts;

	spin_lock(&dev->lock);

	if (irq == dev->irq_crypto) {
		irq_sts = CRYPRO_READ(dev, CRYPTO_INTSTS);
		CRYPRO_WRITE(dev, CRYPTO_INTSTS, irq_sts);
#ifdef RK_CRYPTO_DEBUG
		CRYPTO_INFO("irq_sts = 0x%x\n", irq_sts);
#endif

		if ((irq_sts & BCDMA_DONE_INT)) {
			conf_sts = CRYPRO_READ(dev, CRYPTO_CONF);
			if (conf_sts & DESSEL) {
				rockchip_des_rx(dev);
				rockchip_des_tx(dev);
			} else {
				rockchip_aes_rx(dev);
				rockchip_aes_tx(dev);
			}

			if (dev->busy) {
				if (conf_sts & DESSEL)
					rockchip_aes_restart(dev);
				else
					rockchip_des_restart(dev);
			}
		} else if (irq_sts & HRDMA_DONE_ENA) {
			if (!(irq_sts & HASH_DONE_ENA)) {
				rockchip_ahash_rx(dev);
				if (dev->busy)
					rockchip_ahash_restart(dev);
			} else {
				rockchip_unset_indata(dev);
				rockchip_ahash_crypto_complete(dev, 0);
			}
		}
	}

	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

static void rockchip_crypto_tasklet_cb(unsigned long data)
{
	struct crypto_dev *dev = (struct crypto_dev *)data;
	struct crypto_async_request *async_req, *backlog;
	struct rk_crypto_reqctx *ablk_reqctx;
	struct rk_ahash_reqctx *hash_reqctx;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dev->lock, flags);
	backlog   = crypto_get_backlog(&dev->queue);
	async_req = crypto_dequeue_request(&dev->queue);
	spin_unlock_irqrestore(&dev->lock, flags);

	if (!async_req) {
		pr_err("async_req is NULL !!\n");
		return;
	}

	if (backlog) {
		pr_err("backlog is null\n");
		backlog->complete(backlog, -EINPROGRESS);
	}

	switch (dev->flags) {
	case RK_AES_ECB:
	case RK_AES_CBC:
	case RK_AES_CTR:
		dev->ablk_req = ablkcipher_request_cast(async_req);
		dev->aes_ctx = crypto_tfm_ctx(dev->ablk_req->base.tfm);
		ablk_reqctx   = ablkcipher_request_ctx(dev->ablk_req);
		ret = rockchip_aes_start(dev, ablk_reqctx->mode);
		if (ret)
			pr_err("failed to start aes!\n");
		break;
	case RK_DES_ECB:
	case RK_DES_CBC:
	case RK_DES3_EDE_ECB:
	case RK_DES3_EDE_CBC:
	case RK_DES3_EEE_ECB:
	case RK_DES3_EEE_CBC:
		dev->ablk_req = ablkcipher_request_cast(async_req);
		dev->des_ctx = crypto_tfm_ctx(dev->ablk_req->base.tfm);
		ablk_reqctx   = ablkcipher_request_ctx(dev->ablk_req);
		ret = rockchip_des_start(dev, ablk_reqctx->mode);
		if (ret)
			pr_err("failed to start des!\n");
		break;
	case RK_SHA1:
	case RK_SHA256:
	case RK_MD5:
		dev->ahash_req = ahash_request_cast(async_req);
		hash_reqctx = ahash_request_ctx(dev->ahash_req);
		ret = rockchip_ahash_start(dev, hash_reqctx->mode);
		if (ret)
			pr_err("failed to start hash!\n");
		break;
	default:
		break;
	}
}

static int rockchip_crypto_probe(struct platform_device *pdev)
{
	struct crypto_dev *pdata;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;
	int err = -ENODEV;

	if (cdev)
		return -EEXIST;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* get I/O memory resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "failed to get platform memory resource\n");
		return -ENODEV;
	}
	pdata->ioaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->ioaddr))
		return PTR_ERR(pdata->ioaddr);

	pdata->clk = devm_clk_get(dev, "clk_crypto");
	if (IS_ERR(pdata->clk)) {
		dev_err(dev, "failed to find crypto clock source\n");
		return -ENOENT;
	}

	pdata->hclk = devm_clk_get(dev, "hclk_crypto");
	if (IS_ERR(pdata->hclk)) {
		dev_err(dev, "failed to find crypto clock source\n");
		return -ENOENT;
	}

	if (!cpu_is_rk312x()) {
		pdata->aclk = devm_clk_get(dev, "aclk_crypto");
		if (IS_ERR(pdata->aclk)) {
			dev_err(dev, "failed to find crypto clock source\n");
			return -ENOENT;
		}
	}

	pdata->irq_crypto = platform_get_irq_byname(pdev, "irq_crypto");
	if (pdata->irq_crypto < 0) {
		err = pdata->irq_crypto;
		dev_warn(dev, "crypto interrupt is not available.\n");
		goto err_algs;
	}

	err = devm_request_irq(dev, pdata->irq_crypto,
			       rockchip_crypto_interrupt, IRQF_SHARED,
			       pdev->name, pdev);
	if (err < 0) {
		dev_warn(dev, "crypto interrupt is not available.\n");
		goto err_algs;
	}

	tasklet_init(&pdata->crypto_tasklet,
		     rockchip_crypto_tasklet_cb, (unsigned long)pdata);
	crypto_init_queue(&pdata->queue, CRYPTO_QUEUE_LEN);

	spin_lock_init(&pdata->lock);

	pdata->dev = dev;
	platform_set_drvdata(pdev, pdata);

	rockchip_crypto_misc_register(pdata);

	ret = rockchip_aes_register();
	if (ret)
		pr_err("register aes fail!\n");

	ret = rockchip_des_register();
	if (ret)
		pr_err("register des fail!\n");

	ret = rockchip_register_hash();
	if (ret)
		pr_err("register hash fail!\n");

	cdev = pdata;

	pr_info("rk crypto driver registered\n");
	return 0;
err_algs:
	tasklet_kill(&pdata->crypto_tasklet);
	cdev = NULL;
	platform_set_drvdata(pdev, NULL);

	return err;
}

static int rockchip_crypto_remove(struct platform_device *pdev)
{
	struct crypto_dev *pdata = platform_get_drvdata(pdev);

	if (!pdata)
		return -ENODEV;

	tasklet_kill(&pdata->crypto_tasklet);
	rockchip_aes_unregister();
	rockchip_des_unregister();
	rockchip_unregister_hash();
	rockchip_crypto_clk_disable(pdata);

	cdev = NULL;
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id rockchip_crypto_dt_match[] = {
	{ .compatible = COMPATIBLE_rockchip,},
	{ .compatible = COMPATIBLE_3368,},
	{ }
};
MODULE_DEVICE_TABLE(of, rockchip_crypto_dt_match);
#endif /* CONFIG_OF */

static struct platform_driver rockchip_crypto = {
	.probe	= rockchip_crypto_probe,
	.remove	= rockchip_crypto_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= MODE_NAME,
		.of_match_table = of_match_ptr(rockchip_crypto_dt_match),
	},
};

module_platform_driver(rockchip_crypto);

MODULE_DESCRIPTION("Rockchip Crypto hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rockchip");

