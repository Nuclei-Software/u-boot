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
	 * opensbi/platform/nuclei/demosoc/platform.c
	 * no need to re-initialize it, unless you have new configurations.
	 */
	
	__asm__ __volatile__ ("fence w,o" : : : "memory");

	printf ("Board: Initialized\n");
	return 0;
}

phys_size_t get_effective_memsize(void)
{
	return 128*1024*1024;
}