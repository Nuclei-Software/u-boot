// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2024 Nucleisys, Inc.
 */

#include <config.h>
#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <log.h>
#include <asm/types.h>
#include <malloc.h>
#include <u-boot/rsa-mod-exp.h>
#include <u-boot/rsa.h>
#include "mailbox.h"

void mailbox_acryp_in_token_setup(acryp_in_token_t *cmd_t,
                                uint8_t mode, uint8_t algo,
                                uint32_t dataLen, 
                                uint32_t inAddrLow, uint32_t inAddrHi, uint32_t inLen, 
                                uint32_t signAddrLow, uint32_t signAddrHi, 
                                uint32_t pubKeyAddrLow, uint32_t pubKeyAddrHi)
{
    cmd_t->header.opcode = SECURE_SERVICE_OPCODE_ACRYP;
    cmd_t->identity = (SECURE_SERVICE_OPCODE_ACRYP << 28) | (algo << 16) | (mode);
    cmd_t->length = dataLen;
    cmd_t->input_data_addr_low = inAddrLow;
    cmd_t->input_data_addr_hig = inAddrHi;
    cmd_t->input_data_length = inLen;
    cmd_t->input_signdata_addr_low = signAddrLow;
    cmd_t->input_signdata_addr_hig = signAddrHi;
    cmd_t->input_publickey_addr_low = pubKeyAddrLow;
    cmd_t->Input_PublicKey_addr_hig = pubKeyAddrHi;

    cmd_t->cmd_cfg.algo = algo;
    cmd_t->cmd_cfg.mode = mode;
}

int nuclei_mod_exp(struct udevice *dev, const uint8_t *sig, uint32_t sig_len,
		struct key_prop *prop, uint8_t *out)
{
	uint8_t mailbox_num;
	acryp_in_token_t acryp_in_token = {0};
	uint32_t rBuf[32]={0};
	int ret = 0;

	(void)dev;
	mailbox_num = mailbox_avaliable_linked_num();
	if (mailbox_num == -1) {
		debug("no available mailbox!\n");
		return -EAGAIN;
	}

	mailbox_acryp_in_token_setup(&acryp_in_token, 
		SECURE_SERVICE_ACRYP_MOD_EXP_RSA2048, SECURE_SERVICE_ACRYP_MOD_EXP,
		sig_len/4,
		0/*exponent default 65537*/, 0, sig_len/4,
		(uint32_t)((size_t)sig & 0xFFFFFFFF), (uint32_t)((size_t)sig >> 32),
		(uint32_t)((size_t)prop->modulus & 0xFFFFFFFF), (uint32_t)((size_t)prop->modulus >> 32));
	
	flush_dcache_range((size_t)prop->modulus, (size_t)prop->modulus + prop->num_bits/8);
	flush_dcache_range((size_t)sig, (size_t)sig + sig_len);
	mailbox_secure_service_host_send((uint32_t *)(&acryp_in_token), SECURE_SERVICE_OPCODE_ACRYP, mailbox_num);
	mailbox_secure_service_host_receive(rBuf, mailbox_num);
	if (rBuf[0] & BIT(31)) {
		debug("rsa verify fail, %x\n", rBuf[0]);
		ret = -EIO;
	} else {
		invalidate_dcache_range((size_t)sig, (size_t)sig + sig_len);
		memcpy(out, sig, sig_len);
		debug("rsa verify sucess!\n");
	}

	return ret;
}

static const struct mod_exp_ops nuclei_mod_exp_ops = {
	.mod_exp	= nuclei_mod_exp,
};

U_BOOT_DRIVER(nuclei_rsa_mod_exp) = {
	.name	= "nuclei_rsa_mod_exp",
	.id		= UCLASS_MOD_EXP,
	.ops	= &nuclei_mod_exp_ops,
	.flags	= DM_FLAG_PRE_RELOC,
};

U_BOOT_DRVINFO(nuclei_rsa) = {
	.name = "nuclei_rsa_mod_exp",
};
