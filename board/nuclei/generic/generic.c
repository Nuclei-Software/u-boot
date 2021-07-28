// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2021 Nuclei System Technology
 *
 * Authors:
 *   Huaqi Fang <hqfang@nucleisys.com>
 */

#include <common.h>
#include <dm.h>
#include <linux/delay.h>
#include <linux/io.h>

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
