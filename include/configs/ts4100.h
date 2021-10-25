/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6UL 14x14 DDR3 ARM2.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __TS4100_CONFIG_H
#define __TS4100_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include <asm/imx-common/gpio.h>
#include "mx6_common.h"

/* Env is at the 1MB boundary in emmc boot partition 0 */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV  	1 /* mmcblk0 */
#define CONFIG_SYS_MMC_ENV_PART 	1 /* boot0 */

#define CONFIG_ENV_OFFSET		0x400000 /* 4MiB */
#define CONFIG_ENV_SIZE			SZ_128K
#define CONFIG_ENV_OFFSET_REDUND 	0x600000 /* 6MiB */

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
#define CONFIG_MODULE_FUSE

#define CONFIG_FSL_USDHC
#define CONFIG_MX6
#define CONFIG_SYS_CONSOLE_INFO_QUIET

#define CONFIG_BOARD_SPECIFIC_LED
#define CONFIG_STATUS_LED
#define CONFIG_CMD_LED

#define STATUS_LED_RED                  0
#define STATUS_LED_GREEN                1

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT1                 STATUS_LED_GREEN
#define STATUS_LED_STATE1               STATUS_LED_ON
#define STATUS_LED_PERIOD1              (CONFIG_SYS_HZ / 2)

/* #define CONFIG_USE_PLUGIN */

/* No PMIC */
#undef CONFIG_LDO_BYPASS_CHECK

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

#define CONFIG_FPGA
#define CONFIG_FPGA_LATTICE

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#define CONFIG_CMD_SPI
#define CONFIG_MXC_SPI
#define CONFIG_ATMEL_WIFI_BUS		3 /* ECSPI4 */
#define CONFIG_ATMEL_WIFI_CS		0 /* ECSPI4 */

/* I2C configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2 	/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_AUTOBOOT_KEYED_CTRLC

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY		0
#define CONFIG_AUTOBOOT_KEYED 		1
#define CONFIG_AUTOBOOT_PROMPT "Press Ctrl+C to abort autoboot in %d second(s)\n"
#define CTRL(c) ((c)&0x1F)     
#define CONFIG_AUTOBOOT_STOP_STR  (char []){CTRL('C'), 0}

#define ENV_IMX_TYPE "imx_type="CONFIG_IMX_TYPE"\0"

#define CONFIG_PREBOOT \
	"run silochargeon;"

#define CONFIG_EXTRA_ENV_SETTINGS \
	ENV_IMX_TYPE \
	"chrg_pct=0\0" \
	"chrg_verb=0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdtaddr=0x83000000\0" \
	"rstuboot=1\0" \
	"usbboot=1\0" \
	"model=4100\0" \
	"autoload=no\0" \
	"nfsip=192.168.0.36\0" \
	"nfsroot=/nfsroot/imx6ul/\0" \
	"clearenv=mmc dev 1 1; mmc erase 2000 400; mmc erase 3000 400;\0" \
	"cmdline_append=rootwait rw console=ttymxc0,115200 " \
	  "init=/sbin/init loglevel=3\0" \
	"silochargeon=tsmicroctl d;" \
		"if test $silopresent = '1';" \
			"then if test $jpnochrg = 'off';" \
				"then tsmicroctl e;"\
			"fi;"\
		"fi;\0" \
	"silowaitcharge=if test $silopresent = '1';" \
			"then if test $jpnochrg = 'on';" \
			  "then echo 'NO CHRG jumper is set, not waiting';" \
			  "else tsmicroctl w ${chrg_pct} ${chrg_verb};" \
			"fi;" \
		"fi;\0" \
	"usbprod=powercheck;" \
		"usb start;" \
		"if usb storage;" \
			"then echo Checking USB storage for updates;" \
			"if load usb 0:1 ${loadaddr} /tsinit.scr;" \
				"then led green on;" \
				"source ${loadaddr};" \
				"led red off;" \
				"exit;" \
			"fi;" \
		"fi;\0" \
	"sdboot=echo Booting from the SD card ...;" \
		"powercheck;" \
		"if load mmc 0:1 ${loadaddr} /boot/boot.scr;" \
			"then echo Booting from custom /boot/boot.scr;" \
			"source ${loadaddr};" \
		"fi;" \
                "if load mmc 0:1 ${loadaddr} /boot/ts${model}-fpga.vme; " \
			"then fpga load 0 ${loadaddr} ${filesize}; " \
                "fi; " \
		"if load mmc 0:1 ${fdtaddr} "\
		  "/boot/imx6ul-ts${model}-${baseboardid}.dtb;" \
			"then echo $baseboardid detected;" \
		"else " \
			"echo Booting default device tree;" \
			"load mmc 0:1 ${fdtaddr} /boot/imx6ul-ts${model}.dtb;" \
		"fi;" \
		"if load mmc 0:1 ${loadaddr} /boot/zImage;" \
			"then setenv bootargs root=/dev/mmcblk0p1 " \
			  "${cmdline_append} bbid=0x${baseboardid} " \
			  "bbrev=0x${baseboardrev};" \
			"run silowaitcharge;" \
			"bootz ${loadaddr} - ${fdtaddr};" \
		"else echo Failed to load kernel from SD;" \
		"fi;\0" \
	"emmcboot=echo Booting from the eMMC ...;" \
		"powercheck;" \
		"if load mmc 1:1 ${loadaddr} /boot/boot.scr;" \
			"then echo Booting from custom /boot/boot.scr;" \
			"source ${loadaddr};" \
		"fi;" \
                "if load mmc 1:1 ${loadaddr} /boot/ts${model}-fpga.vme; " \
			"then fpga load 0 ${loadaddr} ${filesize}; " \
                "fi; " \
		"if load mmc 1:1 ${fdtaddr} " \
		  "/boot/imx6ul-ts${model}-${baseboardid}.dtb;" \
			"then echo $baseboardid detected;" \
		"else " \
			"echo Booting default device tree;" \
			"load mmc 1:1 ${fdtaddr} /boot/imx6ul-ts${model}.dtb;" \
		"fi;" \
		"if load mmc 1:1 ${loadaddr} /boot/zImage;" \
			"then setenv bootargs root=/dev/mmcblk1p1 "\
			  "${cmdline_append} bbid=0x${baseboardid} " \
			  "bbrev=0x${baseboardrev};" \
			"run silowaitcharge;" \
			"bootz ${loadaddr} - ${fdtaddr};" \
		"else echo Failed to load kernel from eMMC;" \
		"fi;\0" \
	"nfsboot=echo Booting from NFS ...;" \
		"powercheck;" \
		"dhcp;" \
		"if nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/boot.scr;" \
			"then echo Booting from custom /boot/boot.scr;" \
			"source ${loadaddr};" \
		"fi;" \
		"if nfs ${fdtaddr} " \
		  "${nfsip}:" \
		  "${nfsroot}/boot/imx6ul-ts${model}-${baseboardid}.dtb;" \
			"then echo Baseboard $baseboardid detected;" \
		"else " \
			"echo Booting default device tree;" \
			"nfs ${fdtaddr} " \
			  "${nfsip}:${nfsroot}/boot/imx6ul-ts${model}.dtb;" \
		"fi;" \
		"if nfs ${loadaddr} ${nfsip}:${nfsroot}/boot/zImage;" \
			"then setenv bootargs root=/dev/nfs ip=dhcp " \
			  "nfsroot=${nfsip}:${nfsroot} ${cmdline_append} " \
			  "bbid=0x${baseboardid} bbrev=0x${baseboardrev};" \
			"run silowaitcharge;" \
			"bootz ${loadaddr} - ${fdtaddr};" \
		"else echo Failed to load kernel from NFS;" \
		"fi;\0" \
	"bootcmd_mfg=echo Booted over USB, running test/prime;" \
		"if post;" \
			"then ums mmc 1.1;" \
			"mmc bootbus 1 1 0 2;" \
			"mmc partconf 1 1 1 1;" \
			"fuse prog -y 0 5 A870;" \
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
	"update-uboot=env set filesize 0;" \
		"if test ${jpsdboot} = 'on';" \
			"then echo Updating U-Boot image from SD;" \
			"load mmc 0:1 ${loadaddr} /boot/u-boot-${imx_type}.imx;"\
		"else echo Updating U-Boot image from eMMC;" \
			"load mmc 1:1 ${loadaddr} /boot/u-boot-${imx_type}.imx;"\
		"fi;" \
		"if test ${filesize} != 0;" \
			"then setexpr filesize ${filesize} / 200;" \
			"setexpr filesize ${filesize} + 1;" \
			"mmc dev 1 1;" \
			"mmc write ${loadaddr} 2 ${filesize};" \
		"fi;\0"\
	"update-uboot-nfs=dhcp;"\
		"if nfs ${loadaddr} " \
		  "${nfsip}:${nfsroot}/boot/u-boot-${imx_type}.imx; " \
			"then setexpr filesize ${filesize} / 200;" \
			"setexpr filesize ${filesize} + 1;" \
			"mmc dev 1 1;" \
			"mmc write ${loadaddr} 2 ${filesize};"\
		"fi;\0" \



#define CONFIG_BOOTCOMMAND \
	"if test ${jpuboot} = 'on'; then " \
		"run usbprod;" \
	"else ;" \
		"if test ${usbboot} = '1'; then " \
			"run usbprod;" \
		"fi;" \
		"if test ${jpsdboot} = 'on';" \
			"then run sdboot;" \
			"else run emmcboot;" \
		"fi;" \
	"fi; "

#define CONFIG_CMD_BMODE

#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START	0x80800000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x0f800000)
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_CMD_TIME

#define CONFIG_SYS_HZ			1000

#define CONFIG_STACKSIZE		SZ_128K

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_SYS_NO_FLASH

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	2

#define CONFIG_CMD_EXT2_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_FAT_WRITE

/* USB Configs */
#define CONFIG_CI_UDC
#define CONFIG_CMD_USB
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_G_DNL_MANUFACTURER	"FSL"
#define CONFIG_G_DNL_PRODUCT_NUM	0x4100
#define CONFIG_G_DNL_VENDOR_NUM		0x15a2
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_STORAGE
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_FUNCTION_MASS_STORAGE
#define CONFIG_USB_GADGET
#define CONFIG_USB_GADGET_DOWNLOAD
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#ifdef CONFIG_CMD_NET
#define CONFIG_CMD_MII
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define CONFIG_NFS_TIMEOUT 100UL
#define CONFIG_FEC_ENET_DEV 0

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

#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#endif

#endif /* __TS4100_CONFIG_H */
