/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <console.h>
#include <stdlib.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-pins.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <status_led.h>
#include <i2c.h>

#define GPIO_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
        PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define DAT             IMX_GPIO_NR(3, 10)
#define CLK             IMX_GPIO_NR(3, 9)
#define F5V             IMX_GPIO_NR(3, 8)
#define RST             IMX_GPIO_NR(2, 12)



void board_sleep(int seconds)
{
	uint8_t dat[4] = {0};

	dat[0] = 0x0;
	dat[1] = ((seconds >> 16) & 0xff);
	dat[2] = ((seconds >> 8) & 0xff);
	dat[3] = (seconds & 0xff);
	
	i2c_write(0x2a, 0, 0, dat, 4);
}

void enable_tssilo(void)
{
	uint8_t dat = 0x1;
	i2c_write(0x2a, 0x0, 0, &dat, 1);
}

void disable_tssilo(void)
{
	uint8_t dat = 0x0;
	i2c_write(0x2a, 0x0, 0, &dat, 1);
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
		i2c_read(0x2a, 0x0, 0, &buf[0], 4);
		check = (((buf[2]<<8|buf[3])*100/237));
		if(check > 311 ) {
			check = check-311;
			if(check > 100) check = 100;
			if(verbose) printf("%d%%\n", check);
			if(check >= pct) return 0;
		} else {
			if(verbose) printf("0%%\n");
		}
		if(ctrlc()) return 1;
		udelay(1000000);
	}
}

static int do_microctl(cmd_tbl_t *cmdtp, int flag, 
	int argc, char * const argv[])
{
	int i;
	unsigned int verbose = 0;
	unsigned int pct = 0;
	unsigned int micros;
	char *p;

	i2c_set_bus_num(0);
	for (i = 1; i < argc; i++)
	{
		if(argv[i][0] == '-')
			p = &argv[i][1];
		else
			p = &argv[i][0];

		switch(p[0]) {
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
				if(i+1 == argc) {
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
	"    -s <seconds> Sleep for <seconds>\n"
	"    -w <pct>     Wait until Supercaps are charged to <pct>%\n"
	"    -1           Verbose output when -w is supplied\n"
	"    -e           Enable charging of TS-SILO supercaps\n"
	"    -d           Disable charging of TS-SILO supercaps\n"
);

static iomux_v3_cfg_t const c2_pads[] = {
	MX6_PAD_LCD_DATA05__GPIO3_IO10 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_LCD_DATA04__GPIO3_IO09 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_LCD_DATA03__GPIO3_IO08 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__GPIO2_IO12 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

void c2d_set(unsigned char state) /* 1, 0, or ‘z’ */
{
	if (state == 'z') {
		gpio_direction_input(DAT);
	} else if (state == 1) {
		gpio_direction_output(DAT, 1);
	} else {
		gpio_direction_output(DAT, 0);
	}
}

int c2d_get(void) { return gpio_get_value(DAT); }

void c2ck_set(unsigned char state)
{
	//7553-V2 c2ck is inverted
	/* INFO: c2.c likes to set z state, seems to cause issues
	 * Its not really necessary, not sure why its doing it that often
	 */
	if (state == 'z') {
		//gpio_direction_input(CLK);
	} else if (state == 1) {
		gpio_direction_output(CLK, 0);
	} else {
		gpio_direction_output(CLK, 1);
	}
}
void c2ck_strobe(void) {
	//7553-V2 c2ck is inverted
	gpio_direction_output(CLK, 1);
	gpio_direction_output(CLK, 0);
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
	gpio_direction_output(RST, 1);
	udelay(25);
	gpio_direction_output(RST, 0);
	udelay(1);
}

int blast_silabs(void);

static int do_silabs(cmd_tbl_t *cmdtp, int flag, 
	int argc, char * const argv[])
{
	//Initialize IO pins
	gpio_direction_output(F5V, 1);
	gpio_direction_output(RST, 0);
	gpio_direction_output(DAT, 1);
	gpio_direction_output(CLK, 0);
	imx_iomux_v3_setup_multiple_pads(c2_pads,
	  ARRAY_SIZE(c2_pads));

	data = (unsigned char *)simple_strtoul(argv[1], NULL, 16);
	len = simple_strtoul(argv[2], NULL, 16);

	blast_silabs();

	gpio_direction_input(F5V);
	gpio_direction_input(RST);
	gpio_direction_input(CLK);
	gpio_direction_input(DAT);

	return 0;
	
}

U_BOOT_CMD(silabs, 3, 0, do_silabs,
	"TS supervisory microcontroller programming",
	"  Usage: silabs <image address> <length>\n"
);
