// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Nucleisys Technology Corporation
 */

#ifndef _ASM_NUCLEI_CSR_H
#define _ASM_NUCLEI_CSR_H

#include <asm/asm.h>
#include <linux/const.h>

#define CSR_MCACHE_CTL					0x7ca
#define CSR_MMISC_CTL					0x7d0


#define CSR_MCACHE_ICACHE_EN			BIT(0)
#define CSR_MCACHE_ICACHE_PF_EN			BIT(6)
#define CSR_MCACHE_ICACHE_CANCLE_EN		BIT(7)
#define CSR_MCACHE_DCACHE_EN			BIT(16)


#endif /* _ASM_NUCLEI_CSR_H */
