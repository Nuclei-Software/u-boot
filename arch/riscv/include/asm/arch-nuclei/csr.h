// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2024 Nucleisys Technology Corporation
 */

#ifndef _ASM_NUCLEI_CSR_H
#define _ASM_NUCLEI_CSR_H

#include <asm/asm.h>
#include <linux/const.h>

#define CSR_MCACHE_CTL 0x7ca
#define CSR_MMISC_CTL 0x7d0


#define CSR_MCACHE_ICACHE_EN			BIT(0)
#define CSR_MCACHE_ICACHE_PF_EN			BIT(6)
#define CSR_MCACHE_ICACHE_CANCLE_EN		BIT(7)
#define CSR_MCACHE_DCACHE_EN			BIT(16)

#define CSR_CCM_MBEGINADDR      0x7CB
#define CSR_CCM_MCOMMAND        0x7CC
#define CSR_CCM_SBEGINADDR      0x5CB
#define CSR_CCM_SCOMMAND        0x5CC

typedef enum CCM_CMD {
    CCM_DC_INVAL = 0x0,                 /*!< Unlock and invalidate D-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_DC_WB = 0x1,                    /*!< Flush the specific D-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_DC_WBINVAL = 0x2,               /*!< Unlock, flush and invalidate the specific D-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_DC_LOCK = 0x3,                  /*!< Lock the specific D-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_DC_UNLOCK = 0x4,                /*!< Unlock the specific D-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_DC_WBINVAL_ALL = 0x6,           /*!< Unlock and flush and invalidate all the valid and dirty D-Cache lines */
    CCM_DC_WB_ALL = 0x7,                /*!< Flush all the valid and dirty D-Cache lines */
    CCM_DC_INVAL_ALL = 0x17,            /*!< Unlock and invalidate all the D-Cache lines */
    CCM_IC_INVAL = 0x8,                 /*!< Unlock and invalidate I-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_IC_LOCK = 0xb,                  /*!< Lock the specific I-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_IC_UNLOCK = 0xc,                /*!< Unlock the specific I-Cache line specified by CSR CCM_XBEGINADDR */
    CCM_IC_INVAL_ALL = 0xd              /*!< Unlock and invalidate all the I-Cache lines */
} CCM_CMD_Type;

#define CACHE_LINE_SHIFT 	6
#define CACHE_LINE_SIZE 	(1 << CACHE_LINE_SHIFT)
#define CACHE_LINE_MASK 	(~((1 << CACHE_LINE_SHIFT) - 1))

#endif /* _ASM_NUCLEI_CSR_H */
