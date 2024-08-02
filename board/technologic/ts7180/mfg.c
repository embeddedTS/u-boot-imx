// SPDX-License-Identifier:	GPL-2.0+
/*
 * Copyright (C) 2020, 2021-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <common.h>

#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <cli.h>
#include <i2c.h>
#include <mmc.h>
#include <fuse.h>
#include <command.h>
#include <console.h>
#include <status_led.h>
#include <g_dnl.h>
#include <part.h>
#include <usb.h>
#include <usb_mass_storage.h>

#include "tsfpga.h"

extern int is_mfg(void);

#define FPGA_FLASH_SELECT	IMX_GPIO_NR(4, 0)

void mfgbd_state(struct udevice *dev, uint8_t state)
{
	uint8_t buf = state;
	dm_i2c_write(dev, 0, &buf, 1);
}

void mfg_chirp(struct udevice *dev)
{
	mfgbd_state(dev, 0x83);
	mdelay(75);
	mfgbd_state(dev, 0x3);
}

void mfg_green_on(struct udevice *dev)
{
	mfgbd_state(dev, 0x23);
}

void mfg_red_on(struct udevice *dev)
{
	mfgbd_state(dev, 0x13);
}

void mfg_default(struct udevice *dev)
{
	mfgbd_state(dev, 0x3);
}

void mfg_result(int pass)
{
	int ret;
	struct udevice *dev;

	ret = i2c_get_chip_for_busnum(0, 0x38, 0, &dev);
	if(ret) {
		printf("Can't talk to mfg board.\n");
		return;
	}

	/* This recover the bus in the case i2c is out of sync */
	for (int i = 0; i < 10; i++)	
		status_led_set(0, 0);

	/* Chirp */
	mfg_chirp(dev);
	if (!pass)
		mfg_chirp(dev);

	while (!ctrlc()) {
		status_led_set(0, 0);
		status_led_set(1, 0);
		mfg_default(dev);
		mdelay(1000);

		if (pass) {
			mfg_green_on(dev);
			status_led_set(1, 1);
		} else {
			mfg_red_on(dev);
			status_led_set(0, 1);
		}
		mdelay(1000);
	}
}

int setup_emmc(void)
{
	struct mmc *mmc;
	int ret;

	mmc = find_mmc_device(1);
	ret = mmc_init(mmc);
	if (ret) {
		printf("emmc init failed\n");
		return 1;
	}

	ret = mmc_set_boot_bus_width(mmc, 1, 0, 2);
	if (ret) {
		printf("emmc bootbus failed, ret=%d\n", ret);
		return 1;
	}

	ret = mmc_set_part_conf(mmc, 1, 1, 1);
	if (ret) {
		printf("emmc partconf failed\n");
		return 1;
	}
	return 0;
}

int cpu_write_fuses(void)
{
	int ret = 0;

	ret |= fuse_prog(0, 5, 0xA870);
	ret |= fuse_prog(0, 6, 0x10);
	if(ret) {
		printf("Fuses failed to write!\n");
		return 1;
	}

	fuse_prog(0, 3, 0x300000);

	return 0;
}

void start_ums(void)
{
	char *ums_argv[3] = { "ums", "mmc", "1.1" };

	cmd_process(0, 3, ums_argv, 0, 0);
}

static int do_mfg(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;

	if(!is_mfg()) {
		printf("MFG board not detected.\n");
		return 1;
	}

	printf("Starting MFG, do not interrupt.\n");

	printf("Setting up eMMC\n");
	ret = setup_emmc();
	if(ret){
		mfg_result(0);
		return ret;
	}

	printf("Writing Fuses\n");
	ret = cpu_write_fuses();
	if(ret){
		mfg_result(0);
		return ret;
	}

	printf("Starting UMS\n");
	start_ums();

	printf("MFG Done\n");
	mfg_result(1);

	return 0;
}

U_BOOT_CMD(mfg, 1, 1,	do_mfg,
	"Technologic MFG",
	"Used for early board bringup.  MFG use only."
);
