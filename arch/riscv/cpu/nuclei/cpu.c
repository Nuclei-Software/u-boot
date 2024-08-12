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
