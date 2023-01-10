/*
 * Copyright (C) 2017-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * Author: Kris Bahnsen <kris@embeddedTS.com>
 * Based on work by Mark Featherston <mark@embeddedTS.com>
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

#include "parse_strap.h"

int micrel_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC0", 0x2, &oui, &model, &rev) != 0) {
		printf("Failed to find PHY\n");
		return 1;
	}

	if(oui != 0x0885) {
		printf("Wrong PHY? Bad OUI 0x%X 0x0885\n", oui);
		ret |= 1;
	}

	if(model != 0x16) {
		printf("Wrong PHY? Bad model 0x%X not 0x16\n", model);
		ret |= 1;
	}

	if (ret == 0) printf("PHY test passed\n");
	else printf("PHY test failed\n");
	return ret;
}

int emmc_test(int destructive)
{
	int ret = 0, i;
	uint32_t *loadaddr = (uint32_t *)0x80800000;
	cmd_tbl_t *cmd;

	char *query_argv[3] = { "mmc", "dev", "1" };
	char *write_argv[5] = { "mmc", "write", "0x80800000", "0x0", "0x800" };
	char *read_argv[5] = { "mmc", "read", "0x80800000", "0x0", "0x800" };

	cmd = find_cmd("mmc");

	/* This tests simple enumeration */
	ret |= cmd->cmd(cmd, 0, 3, query_argv);

	if(destructive) {
		memset(loadaddr, 0xAAAAAAAA, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++)
		{
			if (loadaddr[i] != 0xAAAAAAAA)
			{
				ret = 1;
			}
		}

		memset(loadaddr, 0x55555555, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++)
		{
			if (loadaddr[i] != 0x55555555)
			{
				ret = 1;
			}
		}

		if (ret == 0) printf("eMMC test passed\n");
		else printf("eMMC test failed\n");
	} else {
		printf("Not running eMMC test!\n");
	}

	return ret;
}

int atmel_wifi_test(void)
{
	/* Magic number SPI string.
	 * This is sent by the kernel driver first thing after out of reset.
	 * It is doing an internal read of some address, contents don't really
	 *   matter.  We only care about first response byte, which should
	 *   match the first byte sent (which is the command that is sent).
	 */
	static const char dout[17] = {0xc4, 0x0, 0x24, 0x0, 0x20, 0x0, 0x0, 0x0,
	  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	char din[17] = {0};
	static int ret = 0;
	struct spi_slave *slave;


	/* Unreset and enable device */
	gpio_direction_output(IMX_GPIO_NR(4, 26), 1); // chip enable
	mdelay(5);
	gpio_direction_output(IMX_GPIO_NR(4, 10), 1); // Reset
	mdelay(10);/* not documented in wifi datasheet, but needs at least 1ms */

	slave = spi_setup_slave(CONFIG_ATMEL_WIFI_BUS, CONFIG_ATMEL_WIFI_CS,
	  24000000, SPI_MODE_0);
	if(spi_claim_bus(slave)){
		printf("Failed to claim the SPI bus\n");
		ret = 1;
	}

	gpio_direction_output(IMX_GPIO_NR(4, 9), 0); // CS#
	if(!ret) ret = spi_xfer(slave, 136, (void *)dout, (void *)din, 0);
	gpio_direction_output(IMX_GPIO_NR(4, 9), 1); // CS#

	/* Verify first response byte is the command byte we sent */
	ret |= !(din[5] == 0xc4);

	/* Reset and disable device device */
	gpio_direction_output(IMX_GPIO_NR(4, 10), 0); // Reset
	gpio_direction_output(IMX_GPIO_NR(4, 26), 0); // chip enable

	if (ret == 0) printf("WIFI test passed\n");
	else printf("WIFI test failed\n");
	return ret;
}

int fram_test(int destructive)
{
	static const char wren_cmd = 0x6;
	static const char rdsr_cmd[2] = {0x5, 0x0};
	char wr_pattern[19] = {0x2, 0x0, 0x0, 0xAA, 0x55, 0xAA,
	  0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
	  0xAA, 0x55};
	char rd_cmd[19] = {0};
	char din[19] = {0};
	struct spi_slave *slave;
	int ret = 0, i;

	rd_cmd[0] = 0x3;

	slave = spi_setup_slave(CONFIG_FRAM_BUS, CONFIG_FRAM_CS,
	  1000000, SPI_MODE_0);
	if(spi_claim_bus(slave)){
		printf("Failed to claim the SPI bus\n");
		ret = 1;
	}

	/* Read status reg */
	gpio_direction_output(IMX_GPIO_NR(5, 6), 0); // CS#
	if(!ret) ret = spi_xfer(slave, 16, &rdsr_cmd, (void *)din, 0);
	gpio_direction_output(IMX_GPIO_NR(5, 6), 1); // CS#
	if(din[1] != 0x0) ret = 1;

	if (destructive) {
		/* Enable writes */
		gpio_direction_output(IMX_GPIO_NR(5, 6), 0); // CS#
		if(!ret) ret = spi_xfer(slave, 8, &wren_cmd, (void *)din, 0);
		gpio_direction_output(IMX_GPIO_NR(5, 6), 1); // CS#
		/* Write bit pattern */
		gpio_direction_output(IMX_GPIO_NR(5, 6), 0); // CS#
		if(!ret) ret = spi_xfer(slave, 152, (void *)wr_pattern, (void *)din, 0);
		gpio_direction_output(IMX_GPIO_NR(5, 6), 1); // CS#
		/* Read back */
		gpio_direction_output(IMX_GPIO_NR(5, 6), 0); // CS#
		if(!ret) ret = spi_xfer(slave, 152, (void *)rd_cmd, (void *)din, 0);
		gpio_direction_output(IMX_GPIO_NR(5, 6), 1); // CS#

		for (i = 3; i < 19; i++) {
			if (din[i] != wr_pattern[i]) {
				ret = 1;
				printf("Pattern mismatch at byte %d\n", i-3);
				break;
			}
		}
		memset(&din, 0, sizeof(din));
		memset(&wr_pattern, 0, sizeof(wr_pattern));
		wr_pattern[0] = 0x2;

		/* Enable writes */
		gpio_direction_output(IMX_GPIO_NR(5, 6), 0); // CS#
		if(!ret) ret = spi_xfer(slave, 8, &wren_cmd, (void *)din, 0);
		gpio_direction_output(IMX_GPIO_NR(5, 6), 1); // CS#
		/* Write zeros */
		gpio_direction_output(IMX_GPIO_NR(5, 6), 0); // CS#
		if(!ret) ret = spi_xfer(slave, 152, (void *)wr_pattern, (void *)din, 0);
		gpio_direction_output(IMX_GPIO_NR(5, 6), 1); // CS#
		/* Read back */
		gpio_direction_output(IMX_GPIO_NR(5, 6), 0); // CS#
		if(!ret) ret = spi_xfer(slave, 152, (void *)rd_cmd, (void *)din, 0);
		gpio_direction_output(IMX_GPIO_NR(5, 6), 1); // CS#

		for (i = 3; i < 19; i++) {
			if (din[i] != 0x0) {
				ret = 1;
				printf("Zero mismatch at byte %d\n", i-3);
				break;
			}
		}
	}

	if (ret == 0) printf("FRAM test passed\n");
	else printf("FRAM test failed\n");
	return ret;
}

/* Check for M41T00S */
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
	cmd_tbl_t *cmd;

	/* Arguments to mtest are start, end, pattern, and iterations */
	char *argv[5] = {"mtest", "0x80800000", "0x80810000", "1", "20" };

	cmd = find_cmd("mtest");

	ret |= cmd->cmd(cmd, 0, 5, argv);

	if (ret == 0) printf("RAM test passed\n");
	else printf("RAM test failed\n");
	return ret;
}

int usbhub_test(void)
{
	int i;
	struct usb_device *dev = NULL;
	cmd_tbl_t *cmd;
	char *argv[2] = {"usb", "start"};

	cmd = find_cmd("usb");
	cmd->cmd(cmd, 0, 2, argv);

	for (i = 0; i < USB_MAX_DEVICE; i++) {
		dev = usb_get_dev_index(i);
		if (dev == NULL)
			break;

		if(dev->descriptor.idVendor == 0x424 &&
		   dev->descriptor.idProduct == 0x2514) {
		   	printf("USB test passed\n");
			return 0;
		}
	}

	printf("Did not find SMSC USB hub!\n");
	return 1;
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

	ret = i2c_read(0x2a, 0, 0, tmp, 32);
	for (i = 0; i <= 15; i++)
		data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];

	/* 5V_A is between 4.5 and 5.5 VDC */
	if(rscale(data[4], 536, 422) > 5500 ||
	   rscale(data[4], 536, 422) < 4500) {
		printf("Bad 5V_A rail voltage, %dmV\n", rscale(data[8], 147, 107));
		ret = 1;
	}

	/* 3.3V is between 3.0 and 3.6 VDC */
	if(rscale(data[5], 422, 422) < 3000 ||
	   rscale(data[5], 422, 422) > 3600) {
		printf("Bad 3.1V rail voltage, %dmV\n", rscale(data[9], 499, 499));
		ret = 1;
	}

	/* RAM_1.35V is between 1.55 and 1.15 VDC */
	if(sscale(data[6]) < 1150 ||
	   sscale(data[6]) > 1550)	{
		printf("Bad DDR_1.5VDC, %dmV\n", sscale(data[10]));
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

	for(i = 0; i < 24; i++){
		if(i % 4 == 0) red_led_on();
		else red_led_off();
		if(i % 4 == 1) green_led_on();
		else green_led_off();
		mdelay(100);
	}

	red_led_on();
	green_led_on();
}

static int silab_rev(void)
{
	uint8_t val[32];
	i2c_read(0x2a, 0, 0, val, 32);
	return val[31];
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	char *p;
	int destructive = 0;
	uint8_t opts;

	if (argv[1][0] == '-') p = &argv[1][1];
	else p = &argv[1][0];

	if (*p == 'd') destructive = 1;

	opts = parse_strap();

	leds_test();

	if(silab_rev() < 8) {
		ret = 1;
		printf("Silab rev is old or invalid\n");
	}
	printf("Silab rev is 0x%x\n", silab_rev());

	switch(opts) {
	  case 0x6: // Option 6
	  case 0x2: // Option 1
		break;
	  case 0x3: // Option 2
	  case 0x5: // Option 3
	  case 0x7: // Option 4
		ret |= atmel_wifi_test();
		ret |= fram_test(destructive);
		break;
	  default:
		printf("Warning, unknown strapping %d!\n", opts);
		break;
	}


	ret |= usbhub_test();
	if (opts != 0x6)
		ret |= rtc_test();
	ret |= micrel_phy_test();

	ret |= emmc_test(destructive);
	ret |= mem_test();
	ret |= silabs_test();

	if (ret == 0) printf("All POST tests passed\n");
	else printf("One or more POST tests failed\n");
	return ret;
}

U_BOOT_CMD(post, 2, 1,	do_post_test,
	"Runs a POST test",
	"[-d]\n"
	"If -d is supplied, the test is destructive to data on eMMC\n"
);
