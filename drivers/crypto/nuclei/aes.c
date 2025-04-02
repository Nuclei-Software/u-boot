// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2019, softathome
 */

#ifndef USE_HOSTCC
#include <common.h>
#include <malloc.h>
#endif
#include <image.h>
#include <uboot_aes.h>
#include "mailbox.h"
#include "asm/barrier.h"
#include "cpu_func.h"

#define NUCLEI_CRYPTO_DMA_MAXLEN	(0x100000 - 0x10)

int image_aes_encrypt_hw(struct image_cipher_info *info,
              unsigned char *data, int size,
              unsigned char **cipher, int *cipher_len)
{
	return 0;
}

int image_aes_decrypt_hw(struct image_cipher_info *info,
		      const void *cipher, size_t cipher_len,
		      void **data, size_t *size)
{
	uint8_t key_len;
	mailbox_cryp_cmd_in_token cryp_in_token = {0};
	uint32_t rbuf[32];
	uint8_t mailbox_num;
	int ret = 0;

	uint32_t input_addr_h;
	uint32_t input_addr_l;
	uint32_t output_addr_h;
	uint32_t output_addr_l;

    mailbox_num = mailbox_avaliable_linked_num();
    if (mailbox_num == -1) {
        ret = -EAGAIN;
		goto err_ret;
    }

	if (info->cipher->key_len == 32)
		key_len = SECURE_SERVICE_CRYP_KEY_256BITS;
	else if (info->cipher->key_len == 24)
		key_len = SECURE_SERVICE_CRYP_KEY_192BITS;
	else if (info->cipher->key_len == 16)
		key_len = SECURE_SERVICE_CRYP_KEY_128BITS;

	input_addr_h = (uint32_t)((size_t)cipher >> 32);
	input_addr_l = (uint32_t)(size_t)cipher;
	output_addr_h = input_addr_h;
	output_addr_l = input_addr_l;
	flush_dcache_range((size_t)cipher, (size_t)cipher + cipher_len);
	/*
	 * cipher_len is aligned to 16bytes, info->size_unciphered is real data length,
	 * we should pass aligned len to crypto hardware.
	 * */
	mailbox_cryp_in_token_set(&cryp_in_token, (uint8_t *)info->iv, (uint8_t *)info->key, 0,
		SECURE_SERVICE_CRYP_DECRYPT, 0, key_len,
		SECURE_SERVICE_CRYP_CBC, SECURE_SERVICE_CRYP_AES,
		SECURE_SERVICE_IN_ALL, input_addr_l, input_addr_h, (uint32_t)(cipher_len),
		output_addr_l, output_addr_h);
	mailbox_secure_service_host_send((uint32_t *)(&cryp_in_token), SECURE_SERVICE_OPCODE_CRYP, mailbox_num);
	mailbox_secure_service_host_receive(rbuf, mailbox_num);

	if (rbuf[0] & BIT(31)) {
		debug("aes final decrypt fail, %x\n", rbuf[0]);
		*data = NULL;
		*size = 0;
		ret = -EIO;
		goto err_ret;
	} else {
		size_t addr;

		addr = (size_t)output_addr_h << 32 | output_addr_l;
		invalidate_dcache_range(addr, addr + cipher_len);
		*size = info->size_unciphered;
		*data = (void*)addr; 
	}
err_ret:
	return ret;
}
