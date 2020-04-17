/*
 * Copyright (C) 2019, 2020 Technologic Systems
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
	"emmc_dev=0\0" \
	"loadaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtaddr=" __stringify(FDT_ADDR_R) "\0" \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdt_addr_r=" __stringify(FDT_ADDR_R) "\0" \
	"ramdisk_addr_r=" __stringify(RAMDISK_ADDR_R) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtfile=imx6ul-ts7100-0.dtb\0" \
	"clearbootcnt=mw.b 50004018 0;\0" \
	"cmdline_append=console=ttymxc0,115200 init=/sbin/init\0" \
	"altbootcmd=echo taking some recovery action\0" \
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
	"usbprod=usb start;" \
		"if usb storage;" \
			"then echo Checking USB storage for updates;" \
			"if load usb 0:1 ${loadaddr} /tsinit.scr;" \
				"then led green on;" \
				"source ${loadaddr};" \
				"led red off;" \
				"exit;" \
			"fi;" \
			"if load usb 0:1 ${loadaddr} /tsinit.scr.uimg;" \
				"then led green on;" \
				"source ${loadaddr};" \
				"led red off;" \
				"exit;" \
			"fi;" \
		"fi;\0" \
	"emmcboot=echo Booting from the eMMC ...;" \
		"if load mmc ${emmc_dev}:1 ${loadaddr} /boot/boot.ub;" \
			"then echo Booting from custom /boot/boot.ub;" \
			"source ${loadaddr};" \
		"fi;" \
		"load mmc ${emmc_dev}:1 ${fdtaddr} " \
		  "/boot/imx6ul-ts${model}-${io_model}.dtb;" \
		"if load mmc ${emmc_dev}:1 ${loadaddr} /boot/zImage;" \
			"then run silowaitcharge;" \
			"setenv bootargs root=/dev/mmcblk${emmc_dev}p1 rootwait rw " \
			  "cpu_opts=0x${opts} io_opts=0x${io_opts} " \
			  "io_model=0x${io_model} ${cmdline_append};" \
			"bootz ${loadaddr} - ${fdtaddr};" \
		"else echo Failed to load kernel from eMMC;" \
		"fi;\0" \
	"nfsboot-kernel=if nfs ${loadaddr} ${nfsip}:${nfsroot}/boot/zImage;" \
			"then run silowaitcharge;" \
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
	"bootcmd_mfg=echo Booted over USB, running test/prime;" \
		"if post;" \
			"then ums mmc 0.1;" \
			"mmc bootbus 0 1 0 2;" \
			"mmc partconf 0 1 1 1;" \
			"fuse prog -y 0 5 A070;" \
			"fuse prog -y 0 6 10;" \
			"fuse prog -y 0 3 300000;" \
			"i2c mw 38 0.0 83;" \
			"i2c mw 38 0.0 3;" \
			"while true;" \
				"do led green on;" \
				"i2c mw 38 0.0 23;" \
				"sleep 1;" \
				"led green off;" \
				"i2c mw 38 0.0 3;" \
				"sleep 1;" \
			"done;" \
		"else echo Test Failed;" \
			"i2c mw 38 0.0 83;" \
			"i2c mw 38 0.0 3;" \
			"while true;" \
				"do led red on;" \
				"i2c mw 38 0.0 13;" \
				"sleep 1;" \
				"led red off;" \
				"i2c mw 38 0.0 3;" \
				"sleep 1;" \
			"done;" \
		"fi;\0" \
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

#ifdef CONFIG_CMD_NET
#define CONFIG_NFS_TIMEOUT 100UL
#define CONFIG_FEC_ENET_DEV 1

#if (CONFIG_FEC_ENET_DEV == 0)
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x2
#define CONFIG_FEC_XCV_TYPE             RMII
#elif (CONFIG_FEC_ENET_DEV == 1)
#define IMX_FEC_BASE			ENET2_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x1
#define CONFIG_FEC_XCV_TYPE             RMII
#endif
#define CONFIG_ETHPRIME                 "FEC0"
#endif

#endif /* __TS7100_CONFIG_H */
