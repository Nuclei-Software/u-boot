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
#define CFG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffffffffffff\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"mmcdev=0x0\0" \
	"fit_addr= 0xc3000000\0"  \
	"mmcloadfit=fatload mmc ${mmcdev} ${fit_addr} kernel.itb\0" \
	"mmcboot_fit= echo Trying load from mmc..; run mmcloadfit; bootm ${fit_addr}:kernel ${fit_addr}:ramdisk 0xc8000000 \0" \
	"flash_kernel_offset=0x20100000\0" \
	"flash_kernel_size=0xC00000\0" \
	"flashboot_fit= echo Trying load from xipflash..; cp.b ${flash_kernel_offset} ${fit_addr} ${flash_kernel_size}; bootm ${fit_addr}:kernel ${fit_addr}:ramdisk 0xc8000000 \0" \
	BOOTENV

#define CONFIG_PREBOOT \
	"setenv fdt_addr ${fdtcontroladdr};" \
	"fdt addr ${fdtcontroladdr};"

#endif /* __CONFIG_H */
