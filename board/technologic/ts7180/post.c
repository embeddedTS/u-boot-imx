/*
 * Copyright (C) 2017 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <cli.h>
#include <command.h>
#include <i2c.h>
#include <spi.h>
#include <status_led.h>
#include <usb.h>

#include <miiphy.h>

void leds_test(void)
{
	int i;
	red_led_on();
	green_led_on();

	for(i = 0; i < 24; i++){
		if(i % 4 == 0) red_led_on();
		else red_led_off();
		if(i % 4 == 1) yellow_led_on();
		else yellow_led_off();
		if(i % 4 == 2) green_led_on();
		else green_led_off();
		if(i % 4 == 3) blue_led_on();
		else blue_led_off();
		mdelay(100);
	}

	red_led_on();
	green_led_on();
	blue_led_off();
	yellow_led_off();
}

/* Placeholder for now */
static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	leds_test();

	if (ret == 0) printf("All POST tests passed\n");
	else printf("One or more POST tests failed\n");
	return ret;
}

U_BOOT_CMD(post, 2, 1,	do_post_test,
	"Runs a POST test",
	"[-d]\n"
	"If -d is supplied, the test is destructive to data on eMMC\n"
);
