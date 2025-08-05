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
#include <spi_flash.h>
#include <usb.h>

#include <miiphy.h>

#include "post.h"

#define LOOP_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const custom1_led_pads[] = {
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), // YEL_LED#
	MX6_PAD_GPIO_9__GPIO1_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED2#
	MX6_PAD_EIM_D27__GPIO3_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED3#
	MX6_PAD_DISP0_DAT7__GPIO4_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED4#
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED5#
	MX6_PAD_DISP0_DAT10__GPIO4_IO31 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_LED6#
};

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
	 * The TS-7970 has no model register, so we can only verify
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
	for(i = 19; i < 25; i++) {
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

int micrel_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC", 0x7, &oui, &model, &rev) != 0) {
		printf("Failed to find PHY\n");
		return 1;
	}

	if(oui != 0x0885) {
		printf("Wrong PHY?  Bad OUI 0x%X 0x0885\n", oui);
		ret |= 1;
	}

	if(model != 0x22) {
		printf("Wrong PHY?  Bad model 0x%X not 0x22\n", oui);
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
		   dev->descriptor.idProduct == 0x2514) {
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

	ret = i2c_read(0x10, 0, 0, tmp, 32);
	for (i = 0; i <= 15; i++)
		data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];

	/* 5V_A is between 4.5 and 5.5 VDC */
	if(rscale(data[8], 147, 107) > 5500 ||
	   rscale(data[8], 147, 107) < 4500) {
		printf("Bad 5V_A rail voltage, %dmV\n", rscale(data[8], 147, 107));
		ret = 1;
	}

	/* 3.1V is between 2.8 and 3.3 VDC */
	if(rscale(data[9], 499, 499) < 2800 ||
	   rscale(data[9], 499, 499) > 3300) {
		printf("Bad 3.1V rail voltage, %dmV\n", rscale(data[9], 499, 499));
		ret = 1;
	}

	/* DDR_1.5V is between 1.3 and 1.7 VDC */
	if(sscale(data[10]) < 1300 ||
	   sscale(data[10]) > 1700)	{
		printf("Bad DDR_1.5VDC, %dmV\n", sscale(data[10]));
		ret = 1;
	}

	/* 3.3V is between 3.0 and 3.6 VDC */
	if(rscale(data[14], 499, 499) < 3000 ||
	   rscale(data[14], 499, 499) > 3600) {
		printf("Bad 3.3 VDC, %dmV\n", rscale(data[14], 499, 499));
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
	blue_led_on();
	yellow_led_on();
}

static int silab_rev(void)
{
	uint8_t val[32];
	i2c_read(0x10, 0, 0, val, 32);
	return val[31];
}

static int fpga_rev(void)
{
	uint8_t val;
	i2c_read(0x28, 51, 2, &val, 1);
	return (val & 0xf0) >> 4;
}

static int spi_mux_test(void)
{
	struct spi_flash *flash;
	uint32_t buf;
	int ret = 0;
	uint8_t val;

	if(getenv("post_nomux")) {
		return 0;
	}

	val = 0;
	i2c_write(0x28, 57, 2, &val, 1); /* Select offboard */
	flash = spi_flash_probe(0, 0, 15000000, 0);
	spi_flash_read(flash, 0x90000, 4, &buf);
	if(buf != 0xdeadbeef) {
		printf("Expected offboard flash 0xDEADBEEF, got 0x%X\n", buf);
		ret = 1;
	}

	val = 1;
	i2c_write(0x28, 57, 2, &val, 1); /* Select onboard */
	flash = spi_flash_probe(0, 0, 15000000, 0);
	spi_flash_read(flash, 0x90000, 4, &buf);
	if(buf == 0xdeadbeef) {
		printf("Onboard flash returned sentry value, mux failure\n");
		ret = 1;
	}

	/* set back to default offboard */
	i2c_write(0x28, 57, 2, &val, 1);

	if (ret == 0) printf("SPI MUX test passed\n");
	else printf("SPI MUX test failed\n");

	return ret;
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	int build_variant = 0;
	int opt_r37;
	int opt_r36;
	int opt_r34;
	int opt_r39;
	uint8_t val;

	leds_test();

	imx_iomux_v3_setup_multiple_pads(posttest_pads, ARRAY_SIZE(posttest_pads));

	/* Test FPGA first so we know we can trust the build variant */
	ret |= fpga_test();
	ret |= i2c_read(0x28, 51, 2, &val, 1);

	opt_r37 = (~val) & 0x1;
	opt_r36 = ((~val) & 0x2) >> 1;
	opt_r34 = ((~val) & 0x4) >> 2;
	opt_r39 = ((~val) & 0x8) >> 3;
	build_variant = (opt_r39 << 3) | (opt_r37 << 2) | (opt_r36 << 1) | (opt_r34);

	switch (board_rev()) {
		case 'A':
		case 'B':
			ret |= micrel_phy_test();
			break;
		case 'D':
		case 'F':
		case 'G':
		case 'H':
			ret |= marvell_phy_test();
			break;
		case 'J':
			ret |= broadcom_phy_test();
			break;
	}

	switch (build_variant) {
		case 1:
			printf("Build variant %d, TS-7970-1G-4GF-S8S-RTC-I\n", build_variant);
			if(is_quad()) {
				ret = 1;
				printf("Build variant should not be a quad.\n");
			}

			if(fpga_rev() <= 6) {
				ret = 1;
				printf("FPGA REV is old or invalid\n");
			}
			if(silab_rev() <= 1) {
				ret = 1;
				printf("Silab rev is old or invalid\n");
			}
			ret |= usbhub_test();

			break;
		case 2: 
			printf("Build variant %d, TS-7970-2G-4GF-Q10S-RTC-E\n", build_variant);
			if(!is_quad()) {
				ret = 1;
				printf("Build variant should not be a solo.\n");
			}

			if(fpga_rev() <= 6) {
				ret = 1;
				printf("FPGA REV is old or invalid\n");
			}
			if(silab_rev() <= 1) {
				ret = 1;
				printf("Silab rev is old or invalid\n");
			}
			ret |= usbhub_test();

			break;
		case 3: /* */
			printf("Build variant %d, TS-7970-1G-4GF-S8S-RTC-CP-WIFI-I \n", build_variant);
			if(is_quad()) {
				ret = 1;
				printf("Build variant should not be a quad.\n");
			}

			if(fpga_rev() <= 6) {
				ret = 1;
				printf("FPGA REV is old or invalid\n");
			}
			if(silab_rev() <= 1) {
				ret = 1;
				printf("Silab rev is old or invalid\n");
			}

			ret |= usbhub_test();

			break;
		case 4:
			printf("Build variant %d, TS-7970-2G-4GF-Q10S-RTC-CP-WIFI-E \n", build_variant);
			if(!is_quad()) {
				ret = 1;
				printf("Build variant should not be a solo.\n");
			}

			if(fpga_rev() <= 6) {
				ret = 1;
				printf("FPGA REV is old or invalid\n");
			}
			if(silab_rev() <= 1) {
				ret = 1;
				printf("Silab rev is old or invalid\n");
			}

			ret |= usbhub_test();
			break;
		case 13:
			printf("Build variant %d, CUSTOM3\n", build_variant);
			if(!is_quad()) {
				ret = 1;
				printf("Build variant should not be a solo.\n");
			}

			if(fpga_rev() <= 6) {
				ret = 1;
				printf("FPGA REV is old or invalid\n");
			}
			if(silab_rev() <= 1) {
				ret = 1;
				printf("Silab rev is old or invalid\n");
			}

			break;
		case 14:
			printf("Build variant %d, CUSTOM2\n", build_variant);
			if(!is_quad()) {
				ret = 1;
				printf("Build variant should not be a solo.\n");
			}

			if(fpga_rev() <= 6) {
				ret = 1;
				printf("FPGA REV is old or invalid\n");
			}
			if(silab_rev() <= 1) {
				ret = 1;
				printf("Silab rev is old or invalid\n");
			}

			break;
		case 15: /* Custom board or failure */
			/* Some custom board was made before straps worked.  This checks
			 * for that FPGA + silabs rev combination which will not happen in the field */
			if(silab_rev() == 1 && fpga_rev() == 6)
			{
				printf("Build variant %d, CUSTOM1 \n", build_variant);

				/* Turn on unique LEDs */
				imx_iomux_v3_setup_multiple_pads(custom1_led_pads, ARRAY_SIZE(custom1_led_pads));
				gpio_direction_output(IMX_GPIO_NR(1, 2), 0);
				gpio_direction_output(IMX_GPIO_NR(1, 9), 0);
				gpio_direction_output(IMX_GPIO_NR(3, 27), 0);
				gpio_direction_output(IMX_GPIO_NR(4, 28), 0);
				gpio_direction_output(IMX_GPIO_NR(3, 23), 0);
				gpio_direction_output(IMX_GPIO_NR(4, 31), 0);
				
				if(is_quad()) {
					ret = 1;
					printf("Build variant should not be a quad.\n");
				}
				break;
			}
		default:
			printf("unknown build variant %d.  Using basic test\n", build_variant);
			printf("R37: %d\n", opt_r37);
			printf("R36: %d\n", opt_r36);
			printf("R34: %d\n", opt_r34);
			printf("R39: %d\n", opt_r39);
			printf("Build variant: %d\n", build_variant);
	}

	ret |= emmc_test();
	ret |= mem_test();
	ret |= silabs_test();
	ret |= spi_mux_test();

	if (ret == 0) printf("All POST tests passed\n");
	else printf("One or more POST tests failed\n");
	return ret;
}

U_BOOT_CMD(post, 1, 1,	do_post_test,
	"Runs a POST test",
	""
);
