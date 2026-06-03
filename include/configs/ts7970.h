/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the embeddedTS TS-4900
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __TS7990_CONFIG_H
#define __TS7990_CONFIG_H

#include "mx6_common.h"
#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <linux/sizes.h>

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SYS_GENERIC_BOARD

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(10 * SZ_1M)

#define CONFIG_BOARD_LATE_INIT
#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MXC_GPIO
#define CONFIG_CMD_GPIO

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	       UART1_BASE

#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_ISSI
#define CONFIG_SPI_FLASH_SST
#define CONFIG_MXC_SPI

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_FPGA
#define CONFIG_MAX_FPGA_DEVICES			1
#define CONFIG_FPGA_LATTICE
#define CONFIG_FPGA_TDI                 IMX_GPIO_NR(5, 16)
#define CONFIG_FPGA_TMS                 IMX_GPIO_NR(5, 8)
#define CONFIG_FPGA_TCK                 IMX_GPIO_NR(5, 11)
#define CONFIG_FPGA_TDO                 IMX_GPIO_NR(5, 12)

/* OCOTP Configs */
#define CONFIG_CMD_IMXOTP
#define CONFIG_MXC_OCOTP
#define CONFIG_IMX_OTP
#define IMX_OTP_BASE				OCOTP_BASE_ADDR
#define IMX_OTP_ADDR_MAX			0x7F
#define IMX_OTP_DATA_ERROR_VAL		0xBADABADA
#define IMX_OTPWRITE_ENABLED

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       2

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT2_WRITE
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_FAT
#define CONFIG_FAT_WRITE
#define CONFIG_DOS_PARTITION

#define CONFIG_BOARD_SPECIFIC_LED
#define CONFIG_STATUS_LED
#define CONFIG_CMD_LED

#define CONFIG_RED_LED                  IMX_GPIO_NR(1, 2)
#define CONFIG_GREEN_LED                IMX_GPIO_NR(3, 27)
#define CONFIG_YEL_LED                  IMX_GPIO_NR(1, 9)
#define CONFIG_BLUE_LED                 IMX_GPIO_NR(4, 25)
#define STATUS_LED_RED                  0
#define STATUS_LED_GREEN                1
#define STATUS_LED_YELLOW               2
#define STATUS_LED_BLUE                 3

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT1                 STATUS_LED_GREEN
#define STATUS_LED_STATE1               STATUS_LED_ON
#define STATUS_LED_PERIOD1              (CONFIG_SYS_HZ / 2)

#ifdef CONFIG_MX6Q
#define CONFIG_CMD_SATA
#endif

#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

#define CONFIG_RANDOM_MACADDR
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_LIB_RAND
#define CONFIG_FEC_MXC
#define CONFIG_NET_RETRY_COUNT        5
#define CONFIG_MII
#define IMX_FEC_BASE			      ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		      RGMII
#define CONFIG_ETHPRIME			      "FEC"
#define CONFIG_FEC_MXC_PHYADDR		  7
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MARVELL
#define PHY_ANEG_TIMEOUT              50000

/* USB Configs */ 
#define CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_USB_STORAGE
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USBD_HS
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_MXC_USB_PORT	1
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0 
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP
#define CONFIG_USB_GADGET
#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_USB_FASTBOOT_BUF_ADDR   CONFIG_SYS_LOAD_ADDR
#define CONFIG_USB_FASTBOOT_BUF_SIZE   0x07000000
#define CONFIG_G_DNL_VENDOR_NUM 0x0451
#define CONFIG_G_DNL_PRODUCT_NUM 0x5678
#define CONFIG_G_DNL_MANUFACTURER "Technologic"
#undef is_boot_from_usb

#define CONFIG_CMD_FASTBOOT
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_SERIAL_TAG

/* Miscellaneous commands */
#define CONFIG_CMD_SETEXPR

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	       1
#define CONFIG_BAUDRATE			       115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	       1
#define CONFIG_AUTOBOOT_KEYED 1
#define CONFIG_AUTOBOOT_PROMPT "Press Ctrl+C to abort autoboot in %d second(s)\n", bootdelay
#define CTRL(c) ((c)&0x1F)     
#define CONFIG_AUTOBOOT_STOP_STR  (char []){CTRL('C'), 0}
#define CONFIG_LOADADDR			       0x12000000
#define CONFIG_SYS_TEXT_BASE	       0x17800000
#define CONFIG_MISC_INIT_R

#define CONFIG_NFS_TIMEOUT 100UL

/* Create build specific env vars */
#define ENV_IMX_TYPE "imx_type="CONFIG_IMX_TYPE"\0"
#ifdef CONFIG_MX6Q
#define ENV_CPU_TYPE "cpu=q\0"
#else
#define ENV_CPU_TYPE "cpu=dl\0"
#endif

#define CONFIG_PREBOOT \
	"if test ${pushsw} = 'on'; then " \
		"env set bootdelay -1;" \
		"run usbprod;" \
	"else " \
		"env set bootdelay 0;" \
	"fi" 

#define CONFIG_EXTRA_ENV_SETTINGS \
	"uimage=/boot/uImage\0" \
	"initrd_high=0xffffffff\0" \
	"fdtaddr=0x18000000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_addr=0x10800000\0" \
	ENV_IMX_TYPE \
	ENV_CPU_TYPE \
	"model=7970\0" \
	"autoload=no\0" \
	"disable_giga=1\0" \
	"cmdline_append=console=ttymxc0,115200 rootwait ro init=/sbin/init\0" \
	"clearenv=if sf probe; then " \
		"sf erase 0x100000 0x2000;" \
		"sf erase 0x180000 0x2000;" \
		"echo restored environment to factory default; fi\0" \
	"findfdt=" \
		"if test $rev > 'E'; then " \
			"if load ${bootdev} ${bootpart} ${fdtaddr} /boot/imx6${cpu}-ts7970-revf.dtb; then " \
				"echo Loaded TS-7970 REV F dtb;" \
			"elif load ${bootdev} ${bootpart} ${fdtaddr} /boot/imx6${cpu}-ts7970.dtb; then " \
				"echo Booting TS-7970 dtb;" \
			"fi;" \
		"else " \
			"if load ${bootdev} ${bootpart} ${fdtaddr} /boot/imx6${cpu}-ts7970.dtb; then " \
				"echo Booting TS-7970 dtb;" \
			"fi;" \
		"fi;\0" \
	"bootlinux=if load ${bootdev} ${bootpart} ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"run findfdt; " \
		"if load ${bootdev} ${bootpart} ${loadaddr} /boot/ts7970-fpga.vme; " \
			"then fpga load 0 ${loadaddr} ${filesize};" \
		"else " \
			"echo Using default built in FPGA;" \
		"fi;" \
		"load ${bootdev} ${bootpart} ${loadaddr} ${uimage}; " \
		"setenv bootargs root=${rootdev} rootwait rw ${cmdline_append}; " \
		"bootm ${loadaddr} - ${fdtaddr};\0" \
	"sdboot=echo Booting from the SD card ...; " \
		"env set bootdev mmc; " \
		"env set bootpart 0:1; " \
		"env set rootdev '/dev/mmcblk1p1'; " \
		"run bootlinux;\0" \
	"emmcboot=echo Booting from the eMMC ...; " \
		"env set bootdev mmc; " \
		"env set bootpart 1:1; " \
		"env set rootdev '/dev/mmcblk2p1'; " \
		"run bootlinux;\0" \
	"sataboot=echo Booting from SATA ...; " \
		"env set bootdev sata; " \
		"env set bootpart 0:1; " \
		"env set rootdev '/dev/sda1'; " \
		"sata init; " \
		"run bootlinux;\0" \
	"usbboot=echo Booting from USB ...; " \
		"env set bootdev usb; " \
		"env set bootpart 0:1; " \
		"env set rootdev '/dev/sda1'; " \
		"usb start; " \
		"run bootlinux;\0" \
	"usbprod=usb start;" \
		"if usb storage;" \
			"then echo Checking USB storage for updates;" \
			"if load usb 0:1 ${loadaddr} /tsinit.ub;" \
				"then led green on;" \
				"source ${loadaddr};" \
				"led red off;" \
				"exit;" \
			"fi;" \
		"fi;\0" \
	"nfsboot=echo Booting from NFS ...;" \
		"dhcp;" \
		"nfs ${fdtaddr} ${nfsroot}/boot/imx6${cpu}-ts7970-revf.dtb;" \
		"nfs ${loadaddr} ${nfsroot}/boot/uImage;" \
		"setenv bootargs root=/dev/nfs ip=dhcp nfsroot=${nfsroot} ${cmdline_append};" \
		"bootm ${loadaddr} - ${fdtaddr};\0"

#define CONFIG_BOOTCOMMAND \
	"if test ${jpsdboot} = 'on';" \
		"then run sdboot;" \
		"else run emmcboot;" \
	"fi;"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_VERSION_VARIABLE
#define CONFIG_SYS_CBSIZE	       1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       32
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END	       0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x10800000
#define CONFIG_CMD_MEMTEST

#define CONFIG_SYS_LOAD_ADDR	       CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_OFFSET		0x100000
#define CONFIG_ENV_OFFSET_REDUND 0x180000
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_SECT_SIZE	(4 * 1024)
#define CONFIG_SF_DEFAULT_BUS  0
#define CONFIG_SF_DEFAULT_CS   0
#define CONFIG_SF_DEFAULT_SPEED 15000000
#define CONFIG_SF_DEFAULT_MODE (SPI_MODE_0)

#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ	CONFIG_SF_DEFAULT_SPEED
#define CONFIG_CMD_SPI

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#define CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#define CONFIG_CMD_BMODE

#define CONFIG_CMD_TIME
#define CONFIG_SYS_ALT_MEMTEST

#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD

#endif /*__TS7990_CONFIG_H */
