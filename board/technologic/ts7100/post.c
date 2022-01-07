/*
 * Copyright (C) 2019-2022 Technologic Systems, Inc. dba embeddedTS
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
#include <console.h>
#include <i2c.h>
#include <spi.h>
#include <status_led.h>
#include <usb.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>

#include "tsfpga.h"
#include "fram.h"

#include <miiphy.h>

/* Some pads are used for strapping and are IOMUXed differently by the ts7100.c
 * main file. Need to change the IOMUX settings for pad use here. */
#define MISC_PAD_PU_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PKE | PAD_CTL_PUE | \
        PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)
static iomux_v3_cfg_t const wifi_spi[] = {
	/* WIFI SPI CLK */
	MX6_PAD_NAND_CE0_B__ECSPI3_SCLK | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* WIFI SPI MOSI */
	MX6_PAD_NAND_CE1_B__ECSPI3_MOSI | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* WIFI SPI MISO */
	MX6_PAD_NAND_CLE__ECSPI3_MISO | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* WIFI SPI CS# */
	MX6_PAD_NAND_READY_B__ECSPI3_SS0 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* WIFI CHIP EN */
	MX6_PAD_JTAG_TCK__GPIO1_IO14 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* WIFI RESETN is provided by FPGA */
};

static void leds_test(void)
{
	int i;
	red_led_on();
	green_led_on();
	fpga_dio2_oe_set(BANK2_RED_LEDN);
	fpga_dio2_oe_set(BANK2_GREEN_LEDN);

	for(i = 0; i < 24; i++){
		if(i % 4 == 0) green_led_on();
		else green_led_off();
		if(i % 4 == 1) red_led_on();
		else red_led_off();
		if(i % 4 == 2) fpga_dio2_dat_clr(BANK2_RED_LEDN);
		else fpga_dio2_dat_set(BANK2_RED_LEDN);
		if(i % 4 == 3) fpga_dio2_dat_clr(BANK2_GREEN_LEDN);
		else fpga_dio2_dat_set(BANK2_GREEN_LEDN);
		mdelay(100);
	}

	red_led_on();
	green_led_on();
	fpga_dio2_dat_clr(BANK2_RED_LEDN);
	fpga_dio2_dat_clr(BANK2_GREEN_LEDN);
}

#define UART_TEST_SIZE (32768)

/* Test scratch register & 16550 loopback */
static int uart_16550_test(uint32_t addr)
{
	unsigned long long start, end;
	int ret = 0;
	int i;
	int rd = 0;
	int sz = 0;
	uint8_t reg;
	uint8_t rbuf[UART_TEST_SIZE], wbuf[UART_TEST_SIZE];

	/* Test scratch register */
	for (i = 0; i < 0xff; i++) {
		writeb(i, addr + TS16550_SR);
		if(i != readb(addr + TS16550_SR)) {
			printf("16550 0x%X scratch reg did not retain value\n",
			  addr);
			return 1;
		}
	}

	/* Enable loopback */
	writeb(0x10, addr + TS16550_MCR);

	/* Enable DLAB, select 8n1 */
	writeb(0x83, addr + TS16550_LCR);

	/* Set divisor to 1 */
	writeb(0x1, addr + TS16550_DLL);
	writeb(0x0, addr + TS16550_DLH);

	/* Disable DLAB */
	writeb(0x3, addr + TS16550_LCR);

	/* Enable FIFO */
	writeb(0x1, addr + TS16550_FCR);

	/* clear tx/rx fifos, enable 8-byte trigger level */
	writeb(0x83, addr + TS16550_FCR);

	/* Enable RX/TX IRQs */
	writeb(0x3, addr + TS16550_IER);

	/* Write back 0-UART_TEST_SIZE and read it back */
	for (i = 0; i < UART_TEST_SIZE; i++)
		wbuf[i] = i;

	start = get_ticks();

	while(rd != UART_TEST_SIZE) {
		reg = readb(addr + TS16550_IIR);

		if(ctrlc()) return 1;

		switch((reg >> 1) & 0x7){
		case 0x0:
			readb(addr + TS16550_MSR);
			break;
		case 0x1:
			if(sz < UART_TEST_SIZE) {
				for (i = 0; i < 16; i++) {
					if(sz != UART_TEST_SIZE) {
						writeb(wbuf[sz],
						  addr + TS16550_THR);
						sz++;
						/* Dummy rd following a wr */
						readb(addr + TS16550_SR);
					}
				}
			}
			break;
		case 0x6:
		case 0x2:
			rbuf[rd] = readb(addr + TS16550_RBR);
			rd++;
			break;
		case 0x3:
			readb(addr + TS16550_LSR);
			break;
		default:
			printf("Read back an invalid 16550 IIR value!\n");
			return 1;
		}

		end = get_ticks();

		/* Wait for 10 s max (averages 2.8 seconds). In terms of 8MHz
		 * ticks, this is 80000000 */
		if((end - start) > 80000000) {
			printf("16550 timeout\n");
			return 1;
		}
	}

	/* Turn off IRQs */
	writeb(0x0, addr + TS16550_IER);

	/* Turn off loopback */
	writeb(0x10, addr + TS16550_MCR);

	for (i = 0; i < UART_TEST_SIZE; i++) {
		if(wbuf[i] != rbuf[i]) {
			printf("IDX %d: Expected 0x%X, got 0x%X\n",
			  i, wbuf[i], rbuf[i]);
			ret++;
		}
	}
	if(ret) {
		printf("%d errors\n", ret);
	} else {
		printf("Took %llu 8MHz ticks (%dms)\n",
		  end - start, (uint32_t)(end - start)/8000);
	}

	return ret;
}

/* Tx only without loopback on */
/* XXX: This is very closely matching uart_16550_test, this could be eliminated
 * or just integrated in to uart_16550_test as a function argument */
static int uart_16550_tx_test(uint32_t addr)
{
	unsigned long long start, end;
	int ret = 0;
	int i;
	int sz = 0;
	uint8_t reg;
	uint8_t wbuf[UART_TEST_SIZE];

	/* Test scratch register */
	for (i = 0; i < 0xff; i++) {
		writeb(i, addr + TS16550_SR);
		if(i != readb(addr + TS16550_SR)) {
			printf("16550 0x%X scratch reg did not retain value\n",
			  addr);
			return 1;
		}
	}

	/* Enable DLAB, select 8n1 */
	writeb(0x83, addr + TS16550_LCR);

	/* Set divisor to 1 */
	writeb(0x1, addr + TS16550_DLL);
	writeb(0x0, addr + TS16550_DLH);

	/* Disable DLAB */
	writeb(0x3, addr + TS16550_LCR);

	/* Enable FIFO */
	writeb(0x1, addr + TS16550_FCR);

	/* clear tx/rx fifos, enable 8-byte trigger level */
	writeb(0x83, addr + TS16550_FCR);

	/* Enable TX IRQs */
	writeb(0x2, addr + TS16550_IER);

	/* Write back 0-UART_TEST_SIZE and read it back */
	for (i = 0; i < UART_TEST_SIZE; i++)
		wbuf[i] = i;

	start = get_ticks();

	while(sz != UART_TEST_SIZE) {
		reg = readb(addr + TS16550_IIR);

		if(ctrlc()) return 1;

		switch((reg >> 1) & 0x7){
		case 0x0:
			readb(addr + TS16550_MSR);
			break;
		case 0x1:
			if(sz < UART_TEST_SIZE) {
				for (i = 0; i < 16; i++){
					if(sz != UART_TEST_SIZE){
						writeb(wbuf[sz],
						  addr + TS16550_THR);
						sz++;
						/* Dummy rd following a wr */
						readb(addr + TS16550_SR);
					}
				}
			}
			break;
		case 0x6:
		case 0x2:
			readb(addr + TS16550_RBR);
			break;
		case 0x3:
			readb(addr + TS16550_LSR);
			break;
		default:
			printf("Read back an invalid 16550 IIR value!\n");
			return 1;
		}

		end = get_ticks();
	}

	/* Turn off IRQs */
	writeb(0x0, addr + TS16550_IER);

	printf("TX Took %llu 8MHz ticks (%dms)\n",
	  end - start, (uint32_t)(end - start)/8000);

	return ret;
}

static int micrel_phy_test(void)
{
	int ret = 0;
	unsigned int oui;
	unsigned char model;
	unsigned char rev;

	if (miiphy_info ("FEC1", 0x1, &oui, &model, &rev) != 0) {
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

static int atmel_wifi_test(void)
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

	imx_iomux_v3_setup_multiple_pads(wifi_spi, ARRAY_SIZE(wifi_spi));

	/* Unreset and enable device */
	gpio_direction_output(IMX_GPIO_NR(1, 14), 1); // chip enable
	mdelay(5);
	fpga_dio2_dat_set(BANK2_WIFI_RESETN);
	fpga_dio2_oe_set(BANK2_WIFI_RESETN);
	mdelay(10);/* not documented in wifi datasheet, but needs at least 1ms */

	slave = spi_setup_slave(CONFIG_ATMEL_WIFI_BUS, CONFIG_ATMEL_WIFI_CS,
	  24000000, SPI_MODE_0);
	if(spi_claim_bus(slave)){
		printf("Failed to claim the SPI bus\n");
		ret = 1;
	}

	/* XXX IOMUX in u-boot may need to set this as GPIO? */
	if(!ret) ret = spi_xfer(slave, 136, (void *)dout, (void *)din, 0);

	/* Verify first response byte is the command byte we sent */
	ret |= !(din[5] == 0xc4);

	/* Reset and disable device device */
	fpga_dio2_dat_clr(BANK2_WIFI_RESETN);
	gpio_direction_output(IMX_GPIO_NR(1, 14), 0); // chip enable

	if (ret == 0) printf("WIFI test passed\n");
	else printf("WIFI test failed\n");

	return ret;
}

/* On the TS-7100, the FRAM is initialized by the main ts7100.c setup. This func
 * should not have to do any setup, and a failure caused by that is worth
 * leaving in place.
 * Additionally, the FRAM API is byte read/write at a time.
 * The destructive test writes and reads back a string from FRAM.
 * The non-destructive test simply checks the status register.
 */
static int fram_test(int destructive)
{
	char wr_pattern[19] = {0x2, 0x0, 0x0, 0xAA, 0x55, 0xAA,
	  0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
	  0xAA, 0x55};
	uint8_t din[19] = {0};
	int ret = 0, i;


	din[1] = fram_rdsr();
	if(din[1] != 0x0) ret |= 1;

	if (destructive) {
		/* Write bit pattern */
		for (i = 0; i < sizeof(wr_pattern); i++) {
			fram_write(i, wr_pattern[i]);
		}

		/* Read back */
		for (i = 0; i < sizeof(wr_pattern); i++) {
			din[i] = fram_read(i);
		}
		for (i = 0; i < sizeof(wr_pattern); i++) {
			if (din[i] != wr_pattern[i]) {
				ret |= 1;
				printf("Pattern mismatch at byte %d\n", i);
				break;
			}
		}
		memset(&din, 0, sizeof(din));
		memset(&wr_pattern, 0, sizeof(wr_pattern));

		/* Write zeros */
		for (i = 0; i < sizeof(wr_pattern); i++) {
			fram_write(i, wr_pattern[i]);
		}

		/* Read back */
		for (i = 0; i < sizeof(wr_pattern); i++) {
			din[i] = fram_read(i);
		}
		for (i = 0; i < sizeof(wr_pattern); i++) {
			if (din[i] != 0x0) {
				ret |= 1;
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
static int rtc_test(void)
{
	int ret;

	i2c_set_bus_num(0);
	ret = i2c_probe(0x68);

	if (ret == 0) printf("RTC test passed\n");
	else printf("RTC test failed\n");
	return ret;
}

/* Short mem test compatible with both 512 and 1 G RAM */
static int mem_test(void)
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

/* Verify the functionality of the eMMC
 * This functions sets the current mmc dev to eMMC bus and runs tests.
 * The destructive test will write a pattern of 0xAA that is 4 MiB long starting
 * at eMMC address 0x8080000, or roughtly 2.1 GiB, and then reads it back and
 * verifies. Followed by a pattern of 0x55, also 4 MiB long, also at the same
 * start, this is also read back and verified.
 * If either of the writes or read-backs fails, the test fails.
 * The non-destructive test simple does an enumeration with the mmc command.
 */
static int emmc_test(int destructive)
{
	int ret = 0, i;
	uint32_t *loadaddr = (uint32_t *)0x80800000;
	cmd_tbl_t *cmd;

	/* NOTE: When porting, the last element of query_argv[] needs to be
	 * updated to be the correct mmc bus which the eMMC is on */
	char *query_argv[3] = { "mmc", "dev", "0" };
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

		for (i = 0; i < (1024*1024)/4; i++) {
			if (loadaddr[i] != 0xAAAAAAAA) {
				ret = 1;
			}
		}

		memset(loadaddr, 0x55555555, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, write_argv);
		memset(loadaddr, 0x00000000, 1024*1024*4);
		ret |= cmd->cmd(cmd, 0, 5, read_argv);

		for (i = 0; i < (1024*1024)/4; i++) {
			if (loadaddr[i] != 0x55555555) {
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

/* Only gets the uC revision.
 * NOTE: When porting, the i2c_read command args will likely need to change. */
static int uc_rev(void)
{
	uint8_t val;
	i2c_set_bus_num(0);
	i2c_read(0x54, 0x800, 2, &val, 1);
	return val;
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	char *p;
	int destructive = 0;
	unsigned long cpu_opts, io_model, io_opts;
	uint8_t rev;
	uint64_t start, end;

	if (argv[1][0] == '-') p = &argv[1][1];
	else p = &argv[1][0];

	if (*p == 'd') destructive = 1;

	/* XXX: Currently there is no real meaning to any of these. The P2 7100
	 * builds have all of these straps unpopulated, so for now just ensure
	 * the values are sane and run all of the tests.
	 */
	io_model = getenv_ulong("io_model", 10, 0xFFFFFFFF);
	io_opts = getenv_ulong("io_opts", 10, 0xFFFFFFFF);
	cpu_opts = getenv_ulong("opts", 10, 0xFFFFFFFF);
	if (io_model == 0xFFFFFFFF || io_opts == 0xFFFFFFFF ||
	  cpu_opts == 0xFFFFFFFF) {
		//ret |= 1;
		printf("Strapping values read by U-Boot are invalid!\n");
		printf("io_model: 0x%X\n", io_model);
		printf("io_opts: 0x%X\n", io_opts);
		printf("cpu_opts: 0x%X\n", cpu_opts);
	}

	/* TODO: IO board may or may not have LEDs */
	leds_test();

	rev = uc_rev();
	printf("Microcontroller rev is 0x%x\n", rev);

	//ret |= atmel_wifi_test();
	ret |= fram_test(destructive);
	ret |= rtc_test();
	ret |= mem_test();
	ret |= emmc_test(destructive);
	ret |= micrel_phy_test();
	/* XXX: Add Splash Flash test? */
	/* XXX: Add uC voltage rail check */
	ret |= uart_16550_tx_test(0x50000000);
	ret |= uart_16550_test(0x50000000);

#if 0
	start = get_ticks();
	udelay(1000*1000);
	end = get_ticks();
	printf("Took %llu 8MHz ticks (%dus)\n",
	  end - start, (uint32_t)(end - start)/8);
#endif

	return ret;
}

U_BOOT_CMD(post, 2, 1,	do_post_test,
	"Runs a POST test",
	"[-d]\n"
	"If -d is supplied, the test is destructive to data on eMMC\n"
);
