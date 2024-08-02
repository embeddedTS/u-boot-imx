// SPDX-License-Identifier:	GPL-2.0+
/*
 * Copyright (C) 2020-2022 Technologic Systems, Inc. dba embeddedTS
 */
#ifndef __TS7180_CONFIG_H
#define __TS7180_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

/* SPL options in include/configs */
#include "imx6_spl.h"

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR
#define CONFIG_SYS_FSL_USDHC_NUM	2
#endif

#define CONFIG_IMX_THERMAL

#define BOOT_TARGET_DEVICES(func) \
	func(USB, usb, 0) \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 1) \
	func(DHCP, dhcp, na)

#define CONFIG_BOOTCOMMAND \
	"run silochargeon; run silowaitcharge; run distro_bootcmd;"

#define FDT_ADDR_R			0x83000000
#define RAMDISK_ADDR_R			0x90000000

#define CONFIG_EXTRA_ENV_SETTINGS \
	"autoload=no\0" \
	"netretry=once\0" \
	"clearenv=env default -f -a; env save;\0" \
	"loadaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtaddr=" __stringify(FDT_ADDR_R) "\0" \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdt_addr_r=" __stringify(FDT_ADDR_R) "\0" \
	"ramdisk_addr_r=" __stringify(RAMDISK_ADDR_R) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtfile=imx6ul-ts7180.dtb\0" \
	"console=ttymxc0,115200\0" \
	"ethact=ethernet@2188000\0" \
	"chrg_pct=60\0" \
	"silochargeon=silabs scaps disable;" \
		"if test $silopresent = '1';" \
			"then if test $jpnochrg = 'off';" \
				"then silabs scaps enable;"\
			"fi;"\
		"fi;\0" \
	"silowaitcharge=if test $silopresent = '1';" \
		"then if test $jpnochrg = 'on';" \
			"then echo 'NO CHRG jumper is set, not waiting';" \
			"else silabs scaps wait pct ${chrg_pct};" \
		"fi;" \
	"fi;\0" \
	"nfsroot=192.168.0.36:/mnt/storage/imx6ul\0" \
	"nfsboot=echo Booting from NFS ...;" \
		"dhcp;" \
		"nfs ${fdt_addr_r} ${nfsroot}/boot/${fdtfile};" \
		"nfs ${kernel_addr_r} ${nfsroot}/boot/zImage;" \
		"setenv bootargs root=/dev/nfs rw ip=dhcp nfsroot=${nfsroot}${nfsroot_options} " \
			"loglevel=4 console=${console};" \
		"bootz ${kernel_addr_r} - ${fdt_addr_r};\0" \
	BOOTENV

#include <config_distro_bootcmd.h>

/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* environment organization */
#define CONFIG_SYS_MMC_ENV_DEV		1
#define CONFIG_SYS_MMC_ENV_PART		1 /* boot0 */

/* I2C Configs */
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1
#define CONFIG_SYS_I2C_MXC_I2C3
#define CONFIG_SYS_I2C_SPEED		100000

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif

#endif
