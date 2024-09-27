// SPDX-License-Identifier: GPL-2.0+
#include <common.h>
#include <irq_func.h>
#include <asm/cache.h>
#include <asm/arch-nuclei/csr.h>

/*
 * cleanup_before_linux() is called just before we call linux
 * it prepares the processor for linux
 *
 * we disable interrupt and caches.
 */
int cleanup_before_linux(void)
{
	disable_interrupts();

	cache_flush();

	return 0;
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
void invalidate_dcache_range(size_t start, size_t stop)
{
	ccm_cache_ops(start, stop, CCM_DC_INVAL);
}
void flush_dcache_range(size_t start, size_t stop)
{
	ccm_cache_ops(start, stop, CCM_DC_WB);
}

void invalidate_dcache_all(void)
{
	csr_write(CSR_CCM_MCOMMAND, CCM_DC_INVAL_ALL);
}
