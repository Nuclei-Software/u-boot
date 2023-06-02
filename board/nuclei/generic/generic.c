// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2019 Nuclei System Technology
 *
 * Authors:
 *   Ruigang Wan <rgwan@nucleisys.com>
 */

#include <common.h>
#include <dm.h>
#include <env.h>
#include <fdtdec.h>
#include <image.h>
#include <log.h>
#include <init.h>


int board_init(void)
{
	/*
	 * pinmux are already initialized done in opensbi stage source code
	 * opensbi/platform/nuclei/<SOC>/platform.c
	 * no need to re-initialize it, unless you have new configurations.
	 */
	
	__asm__ __volatile__ ("fence w,o" : : : "memory");

	printf ("Board: Initialized\n");
	return 0;
}

/*
 * Use the default weak implementation in common/memsize.c
 * If will get max memory size defined in dts memory section.
 * 128M RAM may overlap the kernel/initrd/fdt section.
 * Here we use most of the ram available and defined in dts
 * to make sure when uboot reallocated, kernel/initrd/fdt section
 * won't be touched
 */
#if 0
phys_size_t get_effective_memsize(void)
{
	/* Here assume at least 128MB effective memory */
	return 128*1024*1024;
}
#endif
