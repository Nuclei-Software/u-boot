// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Nucleisys Technology Corporation
 */

#ifndef _ASM_NUCLEI_CSR_H
#define _ASM_NUCLEI_CSR_H

#include <asm/asm.h>
#include <linux/const.h>

#define CSR_CCM_FPIPE					0x4CF
#define CSR_CCM_SBEGINADDR				0x5CB
#define CSR_CCM_SCOMMAND				0x5CC
#define CSR_MCACHE_CTL					0x7CA
#define CSR_CCM_MBEGINADDR				0x7CB
#define CSR_CCM_MCOMMAND				0x7CC
#define CSR_MMISC_CTL					0x7D0

#define CSR_MCACHE_ICACHE_EN			BIT(0)
#define CSR_MCACHE_ICACHE_PF_EN			BIT(6)
#define CSR_MCACHE_ICACHE_CANCLE_EN		BIT(7)
#define CSR_MCACHE_DCACHE_EN			BIT(16)


#endif /* _ASM_NUCLEI_CSR_H */
