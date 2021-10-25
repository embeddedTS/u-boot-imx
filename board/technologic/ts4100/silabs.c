/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <console.h>
#include <stdlib.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <status_led.h>
#include <i2c.h>

#include "silabs.h"
#include "fpga.h"

int silab_rev(void)
{
	uint8_t val[17];
	i2c_set_bus_num(0);
	i2c_read(0x4a, 0, 0, val, 17);
	return (val[16] & 0xF);
}

void board_sleep(int seconds)
{
	uint8_t dat[4] = {0};

	dat[0] = 0x0;
	dat[1] = ((seconds >> 16) & 0xff);
	dat[2] = ((seconds >> 8) & 0xff);
	dat[3] = (seconds & 0xff);
	
	i2c_write(0x4a, 0, 0, dat, 4);
}

void enable_tssilo(void)
{
        uint8_t dat = 0x1;
        i2c_write(0x4a, 0x0, 0, &dat, 1);
}

void disable_tssilo(void)
{
        uint8_t dat = 0x0;
        i2c_write(0x4a, 0x0, 0, &dat, 1);
}

// Scale voltage to silabs 0-2.5V
static uint16_t inline sscale(uint16_t data){
	return data * (2.5/1023) * 1000;
}

// Scale voltage for resistor dividers
static uint16_t inline rscale(uint16_t data, uint16_t r1, uint16_t r2)
{
	uint16_t ret = (data * (r1 + r2)/r2);
	return sscale(ret);
}

void read_adcs(void)
{
	uint16_t data[14];
	uint8_t tmp[28];
	int i, ret;

	ret = i2c_read(0x4a, 0, 0, tmp, 28);

	if(ret){
		printf("I2C Read failed with %d\n", ret);
		return;
	}
	for (i = 0; i < 14; i++){
		data[i] = (tmp[i*2] << 8) | tmp[(i*2)+1];
	}

	/* Byte order is P1.2-P1.4, P2.0-P2.7, temp sensor */
	printf("REVISION=%d\n", ((data[8] >> 8) & 0xF));
	printf("AN_SUP_CAP_1=%d\n", sscale(data[0]));
	printf("AN_SUP_CAP_2=%d\n", rscale(data[1], 20, 20));
	printf("AN_MAIN_4P7V=%d\n", rscale(data[2], 20, 20));
	printf("MAIN_5V=%d\n", rscale(data[3], 536, 422));
	printf("USB_OTG_5V=%d\n", rscale(data[4], 536, 422));
	printf("V3P3=%d\n", rscale(data[5], 422, 422));
	printf("RAM_1P35V=%d\n", sscale(data[6]));
	printf("VDD_6UL_CORE=%d\n", sscale(data[9]));
	printf("AN_CHRG=%d\n", rscale(data[10], 422, 422));
	printf("VDD_SOC_CAP=%d\n", sscale(data[11]));
	printf("VDD_ARM_CAP=%d\n", sscale(data[12]));
}

int wait_for_supercaps(int pct, int verbose)
{
	unsigned char buf[4] = {0};
	unsigned int check;

	enable_tssilo();

	if(pct == 0) {
		printf("Not waiting for SuperCaps to charge\n");
		return 0;
	} else {
		printf("Waiting until SuperCaps are charged to %d%%\n", pct);
	}
	if(pct > 100) pct = 100;

	while(1) {
		i2c_set_bus_num(0);
		i2c_read(0x4a, 0x0, 0, &buf[0], 4);
		check = (((buf[2]<<8|buf[3])*100/237));
		if(check > 311) {
			check = check-311;
			if(check > 100) check = 100;
			if(verbose) printf("%d%%\n", check);
			if(check >= pct) return 0;
		} else {
			if(verbose) printf("0%%\n");
		}
		if(ctrlc()) return 1;

		udelay(125000);
		red_led_off();
		udelay(125000);
		red_led_on();
		udelay(125000);
		red_led_off();
		udelay(125000);
		green_led_on();
		udelay(250000);
		green_led_off();
		udelay(250000);
		red_led_on();
	}
}

static int do_microctl(cmd_tbl_t *cmdtp, int flag, 
	int argc, char * const argv[])
{
	int i;
	unsigned int pct = 0;
	unsigned int verbose = 0;
	i2c_set_bus_num(0);

	for (i = 1; i < argc; i++)
	{
		int micros;
		char *p;
		if(argv[i][0] == '-')
			p = &argv[i][1];
		else
			p = &argv[i][0];

		switch(p[0]) {
			case 'i':
				read_adcs();
				break;
			case 's':
				if(i+1 == argc) {
					printf("Missing option for microseconds to sleep\n");
					return 1;
				}
				micros = simple_strtoul(argv[++i], NULL, 10);
				printf("Sleep for %d seconds\n", micros);
				board_sleep(micros);
				break;
			case '0':
				break;
			case '1':
				verbose = 1;
				break;
			case 'w':
				if (i+1 == argc) {
					printf("Missing argument for percent to charge\n");
					return 1;
				}
				pct = simple_strtoul(argv[++i], NULL, 10);
				break;
			case 'e':
				enable_tssilo();
				break;
			case 'd':
				disable_tssilo();
				break;
			default:
				printf("Unknown option '%s'\n", argv[i]);
				return 1;
		}
	}

	if (pct) wait_for_supercaps(pct, verbose);

	return 0;
}

U_BOOT_CMD(tsmicroctl, 4, 0, do_microctl,
	"TS supervisory microcontroller access",
	"  Usage: tsmicroctl <options>\n"
	"    -i           Print information\n"
	"    -s <seconds> Sleep for <seconds>\n"
	"    -w <pct>     Wait until TS-SILO supercaps are at <pct>% charge\n"
	"    -1           Verbose output when -w is supplied\n"
	"    -e           Enable charging of TS-SILO supercaps\n"
	"    -d           Disable charging of TS-SILO supercaps\n"
);

/* The following function is overly complex because I2C is very slow.
 * In order to prevent a RMW every time we touch the data pin we need to
 * maintain a copy of the current state.
 * And, in order to reduce unecessary transactions, we check that we are
 * actually changing a bit before issuing a command to the FPGA.
 * In all, this greatly speeds up how fast we can program the uC.
 */
uint8_t data_state = 0;
void c2d_set(unsigned char state) /* 1, 0, or ‘z’ */
{
	if (state == 'z') {
		data_state &= ~(0x1);
		i2c_write(0x28, RED_LED_PADN, 2, &data_state, 1);
	} else if (state == 1) {
		if(!(data_state & 0x2)) {
			data_state |= 0x2;
			i2c_write(0x28, RED_LED_PADN, 2, &data_state, 1);
		}
		if(!(data_state & 0x1)) {
			data_state |= 0x1;
			i2c_write(0x28, RED_LED_PADN, 2, &data_state, 1);
		}
	} else {
		if(data_state & 0x2) {
			data_state &= ~(0x2);
			i2c_write(0x28, RED_LED_PADN, 2, &data_state, 1);
		}
		if(!(data_state & 0x1)) {
			data_state |= 0x1;
			i2c_write(0x28, RED_LED_PADN, 2, &data_state, 1);
		}
	}
}

int c2d_get(void)
{
	uint8_t val;

	i2c_read(0x28, RED_LED_PADN, 2, &val, 1);
	return (val & 0x4) ? 1 : 0;
}

void c2ck_set(unsigned char state)
{
	uint8_t val;

	/* TS-4100 SILAB_CLK is inverted */
	/* INFO: c2.c likes to set z state, seems to cause issues
	 * Its not really necessary, not sure why its doing it that often
	 *
	 * Since tristate is never set, we can assume we never have to touch OE
	 */
	if (state == 'z') {
		;
	} else if (state == 1) {
		val = 0x1;
		i2c_write(0x28, SILAB_CLK, 2, &val, 1);
	} else {
		val = 0x3;
		i2c_write(0x28, SILAB_CLK, 2, &val, 1);
	}

}
void c2ck_strobe(void) {
	uint8_t val;

	val = 0x3;
	i2c_write(0x28, SILAB_CLK, 2, &val, 1);
	val = 0x1;
	i2c_write(0x28, SILAB_CLK, 2, &val, 1);
}

unsigned int len;
unsigned char *data;

unsigned int c2_fopen(void) {
	return len;
}

unsigned char c2_getc(void) {
	unsigned char ret;
	ret = (*data++);
	return ret;
}

void c2_reset(void) {
	/* SILAB_RST follows SILAB_CLK behavior */
	fpga_gpio_output(SILAB_RST, 1);
	udelay(25);
	fpga_gpio_output(SILAB_RST, 0);
	udelay(1);
}

int blast_silabs(void);

static int do_silabs(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	//Initialize IO pins
	i2c_set_bus_num(2);
	fpga_gpio_output(FORCE_5V, 1);
	fpga_gpio_output(SILAB_RST, 0);
	fpga_gpio_output(RED_LED_PADN, 1);
	data_state = 3;
	fpga_gpio_output(SILAB_CLK, 0);

	data = (unsigned char *)simple_strtoul(argv[1], NULL, 16);
	len = simple_strtoul(argv[2], NULL, 16);

	blast_silabs();

	fpga_gpio_input(FORCE_5V);
	fpga_gpio_input(SILAB_RST);
	fpga_gpio_output(RED_LED_PADN, 0);
	fpga_gpio_input(SILAB_CLK);

	return 0;

}

U_BOOT_CMD(silabs, 3, 0, do_silabs,
	"TS supervisory microcontroller programming",
	"  Usage: silabs <image address> <length>\n"
);
