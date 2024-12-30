// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 Nuclei
 */

#include <compiler.h>
#include <common.h>
#include <cpu_func.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <hw_sha.h>
#include <asm/cache.h>
#include <linux/errno.h>
#include "mailbox.h"

#define CRYPTO_MAX_ALG_NAME     80
#define SHA1_DIGEST_SIZE        20
#define SHA256_DIGEST_SIZE      32

#define MAX_SG_32 32

struct nuclei_hash_template {
	char name[CRYPTO_MAX_ALG_NAME];
	unsigned int digestsize;
	u32 alg_type;
};

struct sg_entry {
	uint32_t addr_lo;
	uint32_t addr_hi;
	uint32_t len_flag;
#define SG_ENTRY_LENGTH_MASK    0x7FFFFFFF
#define SG_ENTRY_FINAL_BIT      0x80000000	
};

enum nuclei_hash_algos {
	SHA1 = 0,
	SHA256
};

struct nuclei_sha_ctx {
	enum nuclei_hash_algos algo;
	uint32_t sg_num;
	uint32_t len;
	struct sg_entry sg_tbl[MAX_SG_32];
};

static struct nuclei_hash_template driver_hash[] = {
	{
		.name = "sha1",
		.digestsize = SHA1_DIGEST_SIZE,
		.alg_type = SECURE_SERVICE_HASH_SHA1,
	},
	{
		.name = "sha256",
		.digestsize = SHA256_DIGEST_SIZE,
		.alg_type = SECURE_SERVICE_HASH_SHA256,
	},
};

static enum nuclei_hash_algos get_hash_type(struct hash_algo *algo)
{
	if (!strcmp(algo->name, driver_hash[SHA1].name))
		return SHA1;
	else
		return SHA256;
}

/* Create the context for progressive hashing using h/w acceleration.
 *
 * @ctxp: Pointer to the pointer of the context for hashing
 * @caam_algo: Enum for SHA1 or SHA256
 * Return: 0 if ok, -ENOMEM on error
 */
static int nuclei_hash_init(void **ctxp, enum nuclei_hash_algos hash_algo)
{
	struct nuclei_sha_ctx *p;

	*ctxp = calloc(1, sizeof(struct nuclei_sha_ctx));

	if (*ctxp == NULL) {
		debug("Cannot allocate memory for context\n");
		return -ENOMEM;
	}
	p = *ctxp;
	p->algo = hash_algo;
	p->sg_num = 0;

	return 0;
}

/*
 * Update sg table for progressive hashing using h/w acceleration
 *
 * The context is freed by this function if an error occurs.
 * We support at most 32 Scatter/Gather Entries.
 *
 * @hash_ctx: Pointer to the context for hashing
 * @buf: Pointer to the buffer being hashed
 * @size: Size of the buffer being hashed
 * @is_last: 1 if this is the last update; 0 otherwise
 * @caam_algo: Enum for SHA1 or SHA256
 * Return: 0 if ok, -EINVAL on error
 */
static int nuclei_hash_update(void *hash_ctx, const void *buf,
			    unsigned int size, int is_last,
			    enum nuclei_hash_algos hash_algo)
{
	dma_addr_t addr = (dma_addr_t)buf;
	struct nuclei_sha_ctx *ctx = hash_ctx;
	(void)hash_algo;
	
	if (ctx->sg_num >= MAX_SG_32) {
		free(ctx);
		return -EINVAL;
	}

	ctx->sg_tbl[ctx->sg_num].addr_hi = (uint32_t)(addr >> 32);
	ctx->sg_tbl[ctx->sg_num].addr_lo = (uint32_t)addr;
	ctx->sg_tbl[ctx->sg_num].len_flag = (size & SG_ENTRY_LENGTH_MASK)
								| (is_last ? SG_ENTRY_FINAL_BIT : 0);
	ctx->sg_num++;

	return 0;
}

/*
 * Perform progressive hashing on the given buffer and copy hash at
 * destination buffer
 *
 * The context is freed after successful completion of hash operation.
 * In case of failure, context is not freed.
 * @hash_ctx: Pointer to the context for hashing
 * @dest_buf: Pointer to the destination buffer where hash is to be copied
 * @size: Size of the buffer being hashed
 * @caam_algo: Enum for SHA1 or SHA256
 * Return: 0 if ok, -EINVAL on error
 */
static int nuclei_hash_finish(void *hash_ctx, void *dest_buf,
				int size, enum nuclei_hash_algos hash_algo)
{
	struct nuclei_sha_ctx *ctx = hash_ctx;
	int i = 0, ret = 0;
	dma_addr_t addr;
	mailbox_hash_cmd_in_token hash_in_token = {0};
	struct sg_entry *sg_cur;
	uint8_t mailbox_num;
	uint32_t rBuf[32]={0};
	
    mailbox_num = mailbox_avaliable_linked_num();
    if (mailbox_num == -1) {
        return -EAGAIN;
    }
	
	if (size < driver_hash[hash_algo].digestsize ||
		ctx->sg_num < 1) {
		return -EINVAL;
	}

	sg_cur = &ctx->sg_tbl[0];
	if (ctx->sg_num >= 2) {
		mailbox_hash_in_token_set(&hash_in_token, SECURE_SERVICE_HASH_MODE,
			(hash_algo == SHA256) ? SECURE_SERVICE_HASH_SHA256 : SECURE_SERVICE_HASH_SHA1,
			SECURE_SERVICE_IN_INIT, sg_cur->addr_lo, sg_cur->addr_hi,
			sg_cur->len_flag & SG_ENTRY_LENGTH_MASK);
		addr = sg_cur->addr_hi;
		addr = addr << 32 | sg_cur->addr_lo;
		flush_dcache_range(addr, addr + (sg_cur->len_flag & SG_ENTRY_LENGTH_MASK));
		mailbox_secure_service_host_send((uint32_t *)(&hash_in_token), SECURE_SERVICE_OPCODE_HASH, mailbox_num);
		mailbox_secure_service_host_receive(rBuf, mailbox_num);
		ret = rBuf[0] & BIT(31);
		if (ret)
			return ret;

		sg_cur++;
		for (i = 0; i < ctx->sg_num -2; i++){
			mailbox_hash_in_token_set(&hash_in_token, SECURE_SERVICE_HASH_MODE,
				(hash_algo == SHA256) ? SECURE_SERVICE_HASH_SHA256 : SECURE_SERVICE_HASH_SHA1,
				SECURE_SERVICE_IN_UPDATE, sg_cur->addr_lo, sg_cur->addr_hi,
				sg_cur->len_flag & SG_ENTRY_LENGTH_MASK);
			addr = sg_cur->addr_hi;
			addr = addr << 32 | sg_cur->addr_lo;
			flush_dcache_range(addr, addr + (sg_cur->len_flag & SG_ENTRY_LENGTH_MASK));
			mailbox_secure_service_host_send((uint32_t *)(&hash_in_token), SECURE_SERVICE_OPCODE_HASH, mailbox_num);
			mailbox_secure_service_host_receive(rBuf, mailbox_num);
			ret = rBuf[0] & BIT(31);
			if (ret)
				return ret;
			sg_cur++;
		}
		mailbox_hash_in_token_set(&hash_in_token, SECURE_SERVICE_HASH_MODE,
			(hash_algo == SHA256) ? SECURE_SERVICE_HASH_SHA256 : SECURE_SERVICE_HASH_SHA1,
			SECURE_SERVICE_IN_END, sg_cur->addr_lo, sg_cur->addr_hi,
			sg_cur->len_flag & SG_ENTRY_LENGTH_MASK);
		addr = sg_cur->addr_hi;
		addr = addr << 32 | sg_cur->addr_lo;		
		flush_dcache_range(addr, addr + (sg_cur->len_flag & SG_ENTRY_LENGTH_MASK));
		mailbox_secure_service_host_send((uint32_t *)(&hash_in_token), SECURE_SERVICE_OPCODE_HASH, mailbox_num);
		mailbox_secure_service_host_receive(rBuf, mailbox_num);
		ret = rBuf[0] & BIT(31);
		if (ret)
			return ret;
		u32 *dst = (u32 *)dest_buf;
		u32 *src = &rBuf[2];

		for(i = 0; i < driver_hash[hash_algo].digestsize/4; i++)
		 	dst[i] = be32_to_cpu(src[i]);
	} else if (ctx->sg_num == 1) {
		mailbox_hash_in_token_set(&hash_in_token, SECURE_SERVICE_HASH_MODE,
			(hash_algo == SHA256) ? SECURE_SERVICE_HASH_SHA256 : SECURE_SERVICE_HASH_SHA1,
			SECURE_SERVICE_IN_ALL, sg_cur->addr_lo, sg_cur->addr_hi,
			sg_cur->len_flag & SG_ENTRY_LENGTH_MASK);
		addr = sg_cur->addr_hi;
		addr = addr << 32 | sg_cur->addr_lo;		
		flush_dcache_range(addr, addr + (sg_cur->len_flag & SG_ENTRY_LENGTH_MASK));
		mailbox_secure_service_host_send((uint32_t *)(&hash_in_token), SECURE_SERVICE_OPCODE_HASH, mailbox_num);
		mailbox_secure_service_host_receive(rBuf, mailbox_num);
		ret = rBuf[0] & BIT(31);
		if (ret)
			return ret;
		u32 *dst = (u32 *)dest_buf;
		u32 *src = &rBuf[2];

		for(i = 0; i < driver_hash[hash_algo].digestsize/4; i++)
		 	dst[i] = be32_to_cpu(src[i]);
	} 

	free(ctx);
	return ret;
}

int nuclei_hash(const unsigned char *pbuf, unsigned int buf_len,
	      unsigned char *pout, enum nuclei_hash_algos algo)
{
	int ret = 0;
	uint8_t mailbox_num;
	uint32_t rBuf[32]={0};
	mailbox_hash_cmd_in_token hash_in_token = {0};
	
    mailbox_num = mailbox_avaliable_linked_num();
    if (mailbox_num == -1) {
        return -EAGAIN;
    }

	mailbox_hash_in_token_set(&hash_in_token, SECURE_SERVICE_HASH_MODE,
		(algo == SHA256) ? SECURE_SERVICE_HASH_SHA256 : SECURE_SERVICE_HASH_SHA1,
		SECURE_SERVICE_IN_ALL, (u32)(size_t)pbuf, (u32)((u64)pbuf >> 32),
		buf_len);
	flush_dcache_range((unsigned long)pbuf, (unsigned long)pbuf + buf_len);		
	mailbox_secure_service_host_send((uint32_t *)(&hash_in_token), SECURE_SERVICE_OPCODE_HASH, mailbox_num);
	mailbox_secure_service_host_receive(rBuf, mailbox_num);
	ret = rBuf[0] & BIT(31);
	if (ret)
		return ret;
	u32 *dst = (u32 *)pout;
	u32 *src = &rBuf[2];

	for(int i = 0; i < driver_hash[algo].digestsize/4; i++)
	 	dst[i] = be32_to_cpu(src[i]);
		
	return ret;
}

void hw_sha256(const unsigned char *pbuf, unsigned int buf_len,
			unsigned char *pout, unsigned int chunk_size)
{
	if (nuclei_hash(pbuf, buf_len, pout, SHA256))
		printf("Nuclei Crypto was not setup properly or it is faulty\n");
}

void hw_sha1(const unsigned char *pbuf, unsigned int buf_len,
			unsigned char *pout, unsigned int chunk_size)
{
	if (nuclei_hash(pbuf, buf_len, pout, SHA1))
		printf("Nuclei Crypto was not setup properly or it is faulty\n");
}

int hw_sha_init(struct hash_algo *algo, void **ctxp)
{
	return nuclei_hash_init(ctxp, get_hash_type(algo));
}

int hw_sha_update(struct hash_algo *algo, void *ctx, const void *buf,
			    unsigned int size, int is_last)
{
	return nuclei_hash_update(ctx, buf, size, is_last, get_hash_type(algo));
}

int hw_sha_finish(struct hash_algo *algo, void *ctx, void *dest_buf,
		     int size)
{
	return nuclei_hash_finish(ctx, dest_buf, size, get_hash_type(algo));
}
