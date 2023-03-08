// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2019 Nuclei System Technology
 *
 * Authors:
 *   Huaqi Fang <hqfang@nucleisys.com>
 */

#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <asm/sections.h>

void *board_fdt_blob_setup(int *err)
{
	*err = 0;
	if (IS_ENABLED(CONFIG_OF_SEPARATE) || IS_ENABLED(CONFIG_OF_BOARD)) {
		if (gd->arch.firmware_fdt_addr)
			return (ulong *)(uintptr_t)gd->arch.firmware_fdt_addr;
	}

	return (ulong *)&_end;
}

int board_init(void)
{
	/*
	 * pinmux are already initialized done in opensbi stage source code
	 * opensbi/platform/generic/nuclei/<soc>.c
	 * no need to re-initialize it, unless you have new configurations.
	 */
	
	__asm__ __volatile__ ("fence w,o" : : : "memory");

	printf ("Board: Initialized\n");
	return 0;
}

phys_size_t get_effective_memsize(void)
{
	/* Here assume at least 128MB effective memory */
	return 128*1024*1024;
}
