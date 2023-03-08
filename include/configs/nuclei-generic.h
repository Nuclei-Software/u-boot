/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (c) 2021 Nuclei System Technology
 *
 * Authors:
 *   Huaqi Fang <hqfang@nucleisys.com>
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <linux/sizes.h>

/* Environment options */

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0)

#include <config_distro_bootcmd.h>

/* kernel_addr_r, fdt_addr_r, scriptaddr and ramdisk_addr_r are all set to CONFIG_SYS_TEXT_BASE  */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0"  \
	"fdt_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0"  \
	"scriptaddr=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0"  \
	"ramdisk_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0"  \
	BOOTENV

#define CONFIG_PREBOOT \
	"setenv fdt_addr ${fdtcontroladdr};" \
	"fdt addr ${fdtcontroladdr};"

#endif /* __CONFIG_H */
