/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * Author: Mark Featherston <mark@embeddedTS.com>
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
#include <status_led.h>
#include <usb.h>

#include <miiphy.h>

#include "post.h"

#define LOOP_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const posttest_pads[] = {
	MX6_PAD_SD4_DAT6__GPIO2_IO14 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // UART2_CTS
	MX6_PAD_SD4_DAT5__GPIO2_IO13 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // UART2_RTS
};

int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
int do_mem_mtest(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[]);
void do_usb_start(void);

extern char board_rev(void);

int fpga_test(void)
{
	int ret = 0;
	int i;
	uint8_t val;

	/* FPGA should be loaded before running this test.  
	 * The TS-7990 has no model register, so we can only verify
	 * some sane values in existing registers
	 */

	ret |= i2c_read(0x28, 51, 2, &val, 1);
	if(val == 0) {
		printf("Read back 0 for addr 51.  FPGA not sane?\n");
		ret = 1;
	}

	/* DIO header should always default to 0x7c for GPIO 
	 * on the crossbar
	 */
	for(i = 19; i < 26; i++) {
		ret |= i2c_read(0x28, i, 2, &val, 1);
		if((val & 0x7c) != 0x7c) {
			printf("Read back 0x%X from addr %d.  Should be 0x7c\n", (val & 0x7c), i);
			ret = 1;
		}
	}

	if (ret == 0) printf("FPGA test passed\n");
	else printf("FPGA test failed\n");
	return ret;
}

int marvell_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC", 0x1, &oui, &model, &rev) != 0) {
		printf("Failed to find PHY\n");
		return 1;
	}

	if(oui != 0x5043) {
		printf("Wrong PHY?  Bad OUI 0x%X 0x5043\n", oui);
		ret |= 1;
	}

	if(model != 0x1d) {
		printf("Wrong PHY?  Bad model 0x%X not 0x1d\n", oui);
		ret |= 1;
	}

	if (ret == 0) printf("PHY test passed\n");
	else printf("PHY test failed\n");
	return ret;
}


int broadcom_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC", 0x1, &oui, &model, &rev) != 0) {
		printf("Failed to find PHY\n");
		return 1;
	}

	if(oui != 0x180361) {
		printf("Wrong PHY?  Bad OUI 0x%X 0x180361\n", oui);
		ret |= 1;
	}

	if(model != 0x0A) {
		printf("Wrong PHY?  Bad model 0x%X not 0x0A\n", oui);
		ret |= 1;
	}

	if (ret == 0) printf("PHY test passed\n");
	else printf("PHY test failed\n");
	return ret;
}

int emmc_test(void)
{
	int ret = 0, i;
	uint32_t *loadaddr = (uint32_t *)0x12000000;

	char *query_argv[3] = { "mmc", "dev", "1" };
	char *write_argv[5] = { "mmc", "write", "0x12000000", "0x0", "0x800" };
	char *read_argv[5] = { "mmc", "read", "0x12000000", "0x0", "0x800" };

	/* This tests simple enumeration */
	ret |= do_mmcops(0, 0, 3, query_argv);

	if(!getenv("post_nowrite")) {
		memset(loadaddr, 0xAAAAAAAA, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++)
		{
			if (loadaddr[i] != 0xAAAAAAAA)
			{
				printf("\tWrong value at %d\n", i);
				printf("got 0x%X, expected 0xAAAAAAAA\n", loadaddr[i]);
				ret = 1;
			}
		}

		memset(loadaddr, 0x55555555, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= do_mmcops(0, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++)
		{
			if (loadaddr[i] != 0x55555555)
			{
				printf("\tWrong value at %d\n", i);
				printf("got 0x%X, expected 0x55555555\n", loadaddr[i]);
				ret = 1;
			}
		}
	}

	if (ret == 0) printf("eMMC test passed\n");
	else printf("eMMC test failed\n");
	return ret;
}

/* Check for m41t000 rtc */
int rtc_test(void)
{
	int ret;

	ret = i2c_probe(0x68);

	if (ret == 0) printf("RTC test passed\n");
	else printf("RTC test failed\n");
	return ret;
}

int mem_test(void)
{
	int ret = 0;
	int argc = 5;
	/* Arguments are the memory addresses, pattern (not relevant in alt test) and
	 * 20 which is the number of iterations (in hex) which takes about 1 second
	 * on the i.MX6 quad */
	char *argv[5] = { "mtest", "0x10000000", "0x10010000", "1", "20" };

	ret |= do_mem_mtest(0, 0, argc, argv);

	if (ret == 0) printf("RAM test passed\n");
	else printf("RAM test failed\n");
	return ret;
}

int usbhub_test(void)
{
	int i;
	struct usb_device *dev = NULL;

	do_usb_start();

	for (i = 0; i < USB_MAX_DEVICE; i++) {
		dev = usb_get_dev_index(i);
		if (dev == NULL)
			break;

		if(dev->descriptor.idVendor == 0x424 &&
		   dev->descriptor.idProduct == 0x9514) {
		   	printf("USB test passed\n");
			return 0;
		}
	}

	printf("Did not find SMSC USB hub!\n");
	return 1;
}

int is_quad(void)
{
#ifdef CONFIG_MX6Q
	return 1;
#else
	return 0;
#endif
}

// Scale voltage to silabs 0-2.5V
uint16_t sscale(uint16_t data){
	return data * (2.5/1023) * 1000;
}

// Scale voltage for resistor dividers
uint16_t rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

int silabs_test(void)
{
	uint16_t data[16];
	uint8_t tmp[32];
	int ret;
	int i;

	ret = i2c_read(0x4a, 0, 0, tmp, 32);
	for (i = 0; i <= 15; i++)
		data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];

	/* Should never see a rev earlier than 6 with POST tests,
	 */
	if(data[15] < 6) {
		printf("Bad Silabs Revision %d - failed\n", data[15]);
		ret = 1;
	}

	/* 5V_A is between 4.5 and 5.5 VDC */
	if(rscale(data[1], 147, 107) > 5500 ||
	   rscale(data[1], 147, 107) < 4500) {
		printf("Bad 5V_A rail voltage, %dmV\n", rscale(data[1], 147, 107));
		ret = 1;
	}

	/* DDR_1.5V is between 1.3 and 1.7 VDC */
	if(sscale(data[3]) < 1300 ||
	   sscale(data[3]) > 1700)	{
		printf("Bad DDR_1.5VDC, %dmV\n", sscale(data[3]));
		ret = 1;
	}

	/* 3.3V is between 3.0 and 3.6 VDC */
	if(rscale(data[7], 499, 499) < 3000 ||
	   rscale(data[7], 499, 499) > 3600) {
		printf("Bad 3.3 VDC, %dmV\n", rscale(data[7], 499, 499));
		ret = 1;
	}

	if (ret == 0) printf("Silabs test passed\n");
	else printf("Silabs test failed\n");
	return ret;
}

void leds_test(void)
{
	int i;
	red_led_on();
	green_led_on();
	blue_led_on();
	yellow_led_on();

	for(i = 0; i < 24; i++){
		if(i % 4 == 0) red_led_on();
		else red_led_off();
		if(i % 4 == 1) green_led_on();
		else green_led_off();
	}

	red_led_on();
	green_led_on();
	blue_led_on();
	yellow_led_on();
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	uint8_t val;

	leds_test();

	imx_iomux_v3_setup_multiple_pads(posttest_pads, ARRAY_SIZE(posttest_pads));

	/* Test FPGA first so we know we can trust the build variant */
	ret |= fpga_test();
	ret |= i2c_read(0x28, 51, 2, &val, 1);
	switch (board_rev()) {
		case 'A':
		default:
			break;
		case 'B':
		case 'C':
		case 'D':
			ret |= marvell_phy_test();
			break;
		case 'E':
			ret |= broadcom_phy_test();
			break;
	}

	ret |= emmc_test();
	ret |= mem_test();
	ret |= usbhub_test();
	ret |= rtc_test();
	ret |= silabs_test();

	if (ret == 0) printf("All POST tests passed\n");
	else printf("One or more POST tests failed\n");
	return ret;
}

U_BOOT_CMD(post, 1, 1,	do_post_test,
	"Runs a POST test",
	""
);
