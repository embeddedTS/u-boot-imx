/*
 * Copyright (C) 2019-2021 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Auto-included in files that include common.h
 */
#ifndef __TS7180_CONFIG_H
#define __TS7180_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

/* SPL options in include/configs */
#include "imx6_spl.h"

// invalid length for memory region SRAM?
//#define CONFIG_IMAGE_MAX_SIZE 0x10000

#if 1
/* moved to ../../configs/ts7180_defconfig */
/* Env is at the 1MB boundary in emmc boot partition 0 */
#define CONFIG_SYS_MMC_ENV_DEV  	1 /* mmcblk0 */
#define CONFIG_SYS_MMC_ENV_PART 	1 /* boot0 */
#endif

/* can these go in led.c too, like the color #define's */
#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

/*#define CONFIG_USE_PLUGIN*/

/* No PMIC */
#undef CONFIG_LDO_BYPASS_CHECK

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

//#define CONFIG_BOARD_EARLY_INIT_F
//#define CONFIG_MISC_INIT_R

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

#undef CONFIG_BOOTM_NETBSD
#undef CONFIG_BOOTM_PLAN9
#undef CONFIG_BOOTM_RTEMS

#define CONFIG_MXC_SPI

#define CONFIG_FPGA
#define CONFIG_FPGA_LATTICE

/* Video */
/*#define CONFIG_VIDEO*/

#ifdef CONFIG_VIDEO
#define	CONFIG_CFB_CONSOLE
#define	CONFIG_VIDEO_MXS
#define	CONFIG_VIDEO_LOGO
#define	CONFIG_VIDEO_SW_CURSOR
#define	CONFIG_VGA_AS_SINGLE_DEVICE
#define	CONFIG_SYS_CONSOLE_IS_IN_ENV
#define	CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define	CONFIG_CMD_BMP
#define	CONFIG_BMP_16BPP
#define	CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_IMX_VIDEO_SKIP
#endif

/* I2C configs */
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_MXC_I2C3		/* enable I2C bus 3 */
#define CONFIG_SYS_I2C_SPEED		100000

#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTDELAY		-1
#define CONFIG_AUTOBOOT_KEYED 		1
//#define CTRL(c) ((c)&0x1F)

/*#define CONFIG_PREBOOT \
	"run silochargeon;"*/

#define FACTORY_DEFAULT 0
#define FACTORY_REMOTE 1
#define FACTORY FACTORY_REMOTE
#if defined(FACTORY)
#  if (FACTORY == FACTORY_REMOTE)
#    if true
#      define FACTORY_NFS \
  	    "nfsip=192.168.71.2\0" \
  	    "serverip=192.168.71.20\0" \
	    "nfsroot=/mnt/sdb1/production/product/TS-7180\0" \
	    "sandbox=remote\0"
#    elif false
#      define FACTORY_NFS \
  	    "nfsip=192.168.71.2\0" \
  	    "serverip=192.168.71.20\0" \
	    "nfsroot=/mnt/sdb1/\0"
#    else
#      define FACTORY_NFS \
  	    "nfsip=192.168.71.2\0" \
  	    "serverip=192.168.71.20\0" \
            "nfsroot=/mnt/sdb1/ts${model}#${serial}_rootfs\0"
#    endif
#  else
#      define FACTORY_NFS \
  	    "nfsip=192.168.0.36\0" \
  	    "serverip=192.168.0.36\0" \
	    "nfsroot=/nfsroot/imx6ul/\0"
#  endif
#  define FACTORY_PRODUCTION \
	"production=blast\0"
#  define FACTORY_DEVELOPMENT \
        "bootfile=u-boot.bin\0" \
        "serverpath=/~lionel/vimx/\0" \
        "wgetaddr=87800000\0" \
        "uboot-go=env run uboot-wget ; go 87800000\0" \
        "uboot-go-safe=silabs wdog 30000 ; echo remember to disable the watchdog if good ; run uboot-go ;\0" \
        "uboot-init=env set bootfile u-boot.bin ; env set serverpath /~lionel/u-boot-imx/ ;\0" \
        "uboot-wget=run uboot-init ; wget ${wgetaddr} ${serverip}:${serverpath}${bootfile} ;\0" \
        "uboot-wget-img=run uboot-init ; wget ${loadaddr} ${serverip}:${serverpath}${u-boot-bin} ;\0" \
        "wdog-status=if tsmicroctl wdog ; then echo unarmed ; else echo armed ; fi ;\0" \
        "wdog0=tsmicroctl wdog set 0\0" \
        "wdog30=tsmicroctl wdog set 30000\0" \
	"production=blast\0"
#endif /* FACTORY */

/* If a board already has an environment, where do we force this one? */
#define FACTORY_MFG_ENV \
	"bootcmd_mfg=echo Booted over USB, running test/prime;" \
		"if post;" \
			"then ums mmc 1.1;" \
			"mmc bootbus 1 1 0 2;" \
			"mmc partconf 1 1 1 1;" \
			"fuse prog -y 0 5 A070;" \
			"fuse prog -y 0 6 10;" \
			"fuse prog -y 0 3 300000;" \
			"i2c dev 0;" \
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
		"fi;\0"

#define FDT_ADDR_R			0x83000000
#define RAMDISK_ADDR_R			0x90000000

#define CONFIG_EXTRA_ENV_SETTINGS \
	"board_name='TS-7180'\0" \
	"chrg_pct=60\0" \
	"chrg_verb=0\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_addr=0x83800000\0" \
	"initrd_high=0xffffffff\0" \
	"autoload=no\0" \
	"model=7180\0" \
        FACTORY_NFS \
        FACTORY_PRODUCTION \
        FACTORY_DEVELOPMENT \
	"clearenv=env default -f -a; env save;\0" \
	"loadaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtaddr=" __stringify(FDT_ADDR_R) "\0" \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdt_addr_r=" __stringify(FDT_ADDR_R) "\0" \
	"ramdisk_addr_r=" __stringify(RAMDISK_ADDR_R) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtfile=imx6ul-ts7250v3.dtb\0" \
	"clearbootcnt=mw.b 50004018 0;\0" \
	"cmdline_append=console=ttymxc0,115200 init=/sbin/init\0" \
	"altbootcmd=echo taking some recovery action\0" \
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
		"if load mmc 1:1 ${loadaddr} /boot/boot.ub;" \
			"then echo Booting from custom /boot/boot.ub;" \
			"source ${loadaddr};" \
		"fi;" \
		"load mmc 1:1 ${fdtaddr} " \
		  "/boot/imx6ul-ts${model}-${io_model}.dtb;" \
		"if load mmc 1:1 ${loadaddr} /boot/zImage;" \
			"then run silowaitcharge;" \
			"setenv bootargs root=/dev/mmcblk1p1 rootwait rw ${cmdline_append};" \
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
	"nfsboot=echo Booting from NFS ...;" \
		"dhcp;" \
		"if nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/boot.scr.uimg;" \
			"then echo Booting from custom /boot/boot.scr.uimg;" \
			"source ${loadaddr};" \
		"else if nfs ${fdtaddr} ${nfsip}:${nfsroot}/boot/boot.scr;" \
			"then echo Booting from custom /boot/boot.scr;" \
			"source ${loadaddr};" \
		"fi;" \
		"nfs ${fdtaddr} " \
		  "${nfsip}:${nfsroot}/boot/imx6ul-ts${model}-${io_model}.dtb;"\
                "run nfsboot-kernel ;\0" \
        FACTORY_MFG_ENV \
	"update-spl=dhcp;"\
		"if nfs ${loadaddr} ${nfsip}:${nfsroot}/boot/SPL; then " \
                        "setexpr filesize ${filesize} / 200 ; " \
                        "setexpr filesize ${filesize} + 1 ; " \
                        "mmc dev 0 1 ; " \
                        "mmc write ${loadaddr} 2 ${filesize} ; "\
                "fi;\0" \
        "u-boot-bin=u-boot-dtb.img\0" \
        "u-boot-dir=boot/\0" \
	"u-boot-flash=setexpr filesize ${filesize} / 200 ; " \
                "setexpr filesize ${filesize} + 1 ; " \
                "mmc dev 0 1 ; " \
                "mmc write ${loadaddr} 8a ${filesize} ;\0" \
	"update-uboot=dhcp;"\
		"if nfs ${loadaddr} ${nfsip}:${nfsroot}/${u-boot-dir}${u-boot-bin}; then " \
                        "setexpr filesize ${filesize} / 200 ; " \
                        "setexpr filesize ${filesize} + 1 ; " \
                        "mmc dev 0 1;" \
                        "mmc write ${loadaddr} 8a ${filesize};"\
                "fi;\0" \
	"update-fpga=dhcp; " \
		"if nfs ${loadaddr} ${nfsip}:${nfsroot}/boot/ts7180.vme; " \
			"then fpga load 0 ${loadaddr} ${filesize};" \
		"fi;\0"

#define CONFIG_BOOTCOMMAND \
	"run usbprod; " \
	"run emmcboot;"

#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x20000000)

#define CONFIG_SYS_HZ			1000

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

/* CPU GPIOs */
#define FPGA_RESETN		IMX_GPIO_NR(4, 11)

/* LED Configs */
#define STATUS_LED_OFF		0
#define STATUS_LED_BLINKING	1
#define STATUS_LED_ON		2

#define STATUS_LED_RED                  0
#define STATUS_LED_YELLOW               1
#define STATUS_LED_GREEN                2
#define STATUS_LED_BLUE                 3

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* USB Configs */
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_USB_GADGET_VBUS_DRAW	2
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2

#ifdef CONFIG_CMD_NET
#define CONFIG_NFS_TIMEOUT 100UL
#endif

#endif /* __TS7180_CONFIG_H */
