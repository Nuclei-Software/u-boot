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

#ifdef CONFIG_ARCH_RV64I
#define CONFIG_SYS_SDRAM_BASE		(CONFIG_SYS_TEXT_BASE - SZ_2M)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + SZ_2M)
#elif defined(CONFIG_ARCH_RV32I)
#define CONFIG_SYS_SDRAM_BASE		(CONFIG_SYS_TEXT_BASE - SZ_4M)
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + SZ_4M)
#endif

#define CONFIG_SYS_INIT_SP_ADDR		(CONFIG_SYS_SDRAM_BASE + SZ_8M)

#define CONFIG_SYS_MALLOC_LEN		SZ_8M

#define CONFIG_SYS_BOOTM_LEN		SZ_64M

#define CONFIG_STANDALONE_LOAD_ADDR	CONFIG_SYS_LOAD_ADDR

/* Environment options */

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0)

#include <config_distro_bootcmd.h>

/* kernel_addr_r, fdt_addr_r, scriptaddr and ramdisk_addr_r are all set to CONFIG_SYS_TEXT_BASE  */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_TEXT_BASE) "\0"  \
	"fdt_addr_r=" __stringify(CONFIG_SYS_TEXT_BASE) "\0"  \
	"scriptaddr=" __stringify(CONFIG_SYS_TEXT_BASE) "\0"  \
	"ramdisk_addr_r=" __stringify(CONFIG_SYS_TEXT_BASE) "\0"  \
	BOOTENV

#define CONFIG_PREBOOT \
	"setenv fdt_addr ${fdtcontroladdr};" \
	"fdt addr ${fdtcontroladdr};"

#endif /* __CONFIG_H */
