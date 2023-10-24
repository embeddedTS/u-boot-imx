/*
 * Copyright (C) 2019-2022 Technologic Systems dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Auto-included in files that include common.h
 */
#ifndef __TS7100_CONFIG_H
#define __TS7100_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

/* SPL options in include/configs */
#include "imx6_spl.h"

/* Env is at the 1MB boundary in emmc boot partition 0 */
#define CONFIG_SYS_MMC_ENV_DEV  	0 /* mmcblk0 */
#define CONFIG_SYS_MMC_ENV_PART 	1 /* boot0 */

#define STATUS_LED_RED                  0
#define STATUS_LED_GREEN                1
#define STATUS_LED_BLUE                 2

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

#define BOOT_TARGET_DEVICES(func) \
	func(USB, usb, 0) \
	func(MMC, mmc, 0) \
	func(DHCP, dhcp, na)
#define CONFIG_BOOTCOMMAND \
	"run distro_bootcmd;"

#define CONFIG_EXTRA_ENV_SETTINGS \
	"fdt_high=0xffffffff\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"autoload=no\0" \
	"clearenv=env default -f -a && env save;\0" \
	"loadaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtaddr=" __stringify(FDT_ADDR_R) "\0" \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdt_addr_r=" __stringify(FDT_ADDR_R) "\0" \
	"ramdisk_addr_r=" __stringify(RAMDISK_ADDR_R) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"console=ttymxc0,115200\0" \
	"chrg_pct=60\0" \
	"chrg_verb=0\0" \
	"silochargeon=silabs scaps disable;" \
		"if test $silopresent = '1';" \
			"then if test $jpnochrg = 'off';" \
				"then silab scaps enable;"\
			"fi;"\
		"fi;\0" \
	"silowaitcharge=if test $silopresent = '1';" \
		"then if test $jpnochrg = 'on';" \
			"then echo 'NO CHRG jumper is set, not waiting';" \
			"else silabs scaps wait pct ${chrg_pct};" \
		"fi;" \
	"fi;\0" \
	"nfsboot-kernel=if nfs ${loadaddr} ${nfsip}:${nfsroot}/boot/zImage;" \
			"setenv bootargs root=/dev/nfs ip=dhcp " \
			  "nfsroot=${nfsip}:${nfsroot}${nfsroot_options} rootwait rw " \
			  "cpu_opts=0x${opts} io_opts=0x${io_opts} " \
			  "io_model=0x${io_model} ${cmdline_append};" \
			"bootz ${loadaddr} - ${fdtaddr};" \
		"else echo Failed to load kernel from NFS;" \
		"fi\0"                                             \
	"nfsboot=echo Booting from NFS ...;"				\
		"dhcp &&"						\
		"if nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/boot.scr.uimg;"\
		"then "							\
			"echo Booting from custom /boot/boot.scr.uimg;"                 \
			"source ${loadaddr};"				\
		"elif nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/boot.scr;"\
		"then "							\
			"echo Booting from custom /boot/boot.scr;"	\
			"source ${loadaddr};"				\
		"fi;"							\
		"if nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/imx6ul-ts${model}-${io_model}.dtb;" \
		"then "								\
			"run nfsboot-kernel ;"                          \
		"fi\0"	                                		\
	BOOTENV

#define FDT_ADDR_R			0x83000000
#define RAMDISK_ADDR_R			0x90000000

#include <config_distro_bootcmd.h>

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

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

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* I2C configs */
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_SPEED		100000

/* USB Configs */
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#endif /* __TS7100_CONFIG_H */
