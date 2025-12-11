// SPDX-License-Identifier: GPL-2.0+
#include <common.h>
#include <irq_func.h>
#include <asm/cache.h>
#include <asm/arch-nuclei/csr.h>
#include <asm/barrier.h>

#define CACHE_LINE_SHIFT		6
#define CACHE_LINE_SIZE			(1 << CACHE_LINE_SHIFT)
#define CACHE_LINE_MASK			(~((1 << CACHE_LINE_SHIFT) - 1))

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

static inline void ccm_flush_pipe(void)
{
	csr_write(CSR_CCM_FPIPE, 0x1);
}

void icache_enable(void)
{
#if CONFIG_IS_ENABLED(RISCV_MMODE)
	csr_set(CSR_MCACHE_CTL, CSR_MCACHE_ICACHE_EN 
			| CSR_MCACHE_ICACHE_PF_EN | CSR_MCACHE_ICACHE_CANCLE_EN);
#endif
}

void dcache_enable(void)
{
#if CONFIG_IS_ENABLED(RISCV_MMODE)
	csr_set(CSR_MCACHE_CTL, CSR_MCACHE_DCACHE_EN);
#endif
}

static void ccm_cache_ops(size_t start_addr, size_t end_addr, CCM_CMD_Type type)
{
	size_t cache_aligned;

#if CONFIG_IS_ENABLED(RISCV_MMODE)
	cache_aligned = start_addr & CACHE_LINE_MASK;
	csr_write(CSR_CCM_MBEGINADDR, cache_aligned);
	for (; cache_aligned < end_addr; cache_aligned += CACHE_LINE_SIZE) {
		csr_write(CSR_CCM_MCOMMAND, type);
	}
#elif CONFIG_IS_ENABLED(RISCV_SMODE)
	cache_aligned = start_addr & CACHE_LINE_MASK;
	csr_write(CSR_CCM_SBEGINADDR, cache_aligned);
	for (; cache_aligned < end_addr; cache_aligned += CACHE_LINE_SIZE) {
		csr_write(CSR_CCM_SCOMMAND, type);
	}
#endif
}

/* invalidate cache */
void invalidate_dcache_range(size_t start, size_t stop)
{
	ccm_cache_ops(start, stop, CCM_DC_INVAL);
	ccm_flush_pipe();
	mb();
}

/* writeback and invalidate cache */
void flush_dcache_range(size_t start, size_t stop)
{
	ccm_cache_ops(start, stop, CCM_DC_WBINVAL);
	ccm_flush_pipe();
	mb();
}
