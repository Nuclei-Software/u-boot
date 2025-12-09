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
	"fit_load_addr=" __stringify(CONFIG_KERNEL_FIT_LOAD_ADDR) "\0" \
	"dtb_load_addr=" __stringify(CONFIG_DTB_LOAD_ADDR) "\0" \
	"mmcloadfit=fatload mmc ${mmcdev} ${fit_load_addr} kernel_rootfs.itb \0" \
	"mmcboot_fit= echo Trying load from mmc..; run mmcloadfit; bootm ${fit_load_addr}:kernel ${fit_load_addr}:ramdisk  ${dtb_load_addr} \0" \
	"kernel_flash_addr=" __stringify(CONFIG_KERNEL_FLASH_ADDR) "\0" \
	"kernel_flash_size=" __stringify(CONFIG_KERNEL_FLASH_SIZE) "\0" \
	"flashboot_fit= echo Trying load from xipflash..; cp.b ${kernel_flash_addr} ${fit_load_addr} ${kernel_flash_size}; bootm ${fit_load_addr}:kernel ${fit_load_addr}:ramdisk ${dtb_load_addr} \0" \
	BOOTENV

#define CONFIG_PREBOOT \
	"setenv fdt_addr ${fdtcontroladdr};" \
	"fdt addr ${fdtcontroladdr};"

#endif /* __CONFIG_H */
