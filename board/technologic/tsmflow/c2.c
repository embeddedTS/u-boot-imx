/*
 * Copyright 2014 Dirk Eibach <eibach@gdsys.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <common.h>
#include <linux/sizes.h>

#include "c2.h"

#ifndef EINVAL
#define EINVAL 22
#endif

#ifndef EIO
#define EIO 5
#endif

extern void c2d_set(unsigned char state);
extern int c2d_get(void);
extern void c2ck_set(unsigned char state);
extern void c2ck_strobe(void);
extern unsigned char c2_getc(void);
extern unsigned int c2_fopen(void);

struct c2tool_state {
        struct c2family *family;
};

struct c2_device_info {
	unsigned char device_id;
	unsigned char revision_id;
};

struct c2_pi_info {
	unsigned char version;
	unsigned char derivative;
};

void c2_reset(void);
int c2_halt(void);
int c2_get_device_info(struct c2_device_info *info);

int c2_get_pi_info(struct c2tool_state *state, struct c2_pi_info *info);

int c2_write_sfr(unsigned char reg, unsigned char value);
int c2_write_direct(struct c2tool_state *state, unsigned char reg, unsigned char value);
int c2_read_sfr(unsigned char reg);
int c2_read_direct(struct c2tool_state *state, unsigned char reg);

int c2_flash_read(struct c2tool_state *state, unsigned int addr, unsigned int length, unsigned char *dest);
int c2_flash_erase_device(struct c2tool_state *state);

enum {
	C2_MEM_TYPE_FLASH,
	C2_MEM_TYPE_OTP,
};

enum {
	C2_SECURITY_JTAG,
	C2_SECURITY_C2_1,
	C2_SECURITY_C2_2,
	C2_SECURITY_C2_3,
};

struct c2_setupcmd {
	unsigned char token;
	unsigned char reg;
	unsigned char value;
	unsigned int time;
};

struct c2family {
	unsigned int device_id; /* c2/jtag family id */
	const char *name; /* name of device family */
	unsigned char mem_type; /* flash or otp */
	unsigned int page_size; /* number of bytes per code page */
	char has_sfle; /* true if device has sfle bit */
	unsigned char security_type; /* flash security type */
	unsigned char fpdat; /* flash programming data register address */
	struct c2_setupcmd *setup;	/* list of initialization commands */
};

struct c2tool_state;

int c2family_find(unsigned int device_id, struct c2family **family);
int c2family_setup(struct c2tool_state *state);

enum {
	C2_DONE,
	C2_WRITE_SFR,
	C2_WRITE_DIRECT, /* for devices with SFR paging */
	C2_READ_SFR,
	C2_READ_DIRECT, /* for devices with SFR paging */
	C2_WAIT_US,
};

static struct c2_setupcmd c2init_f30x[] = {
	{ C2_WRITE_SFR, 0xb2, 0x07 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f31x[] = {
	{ C2_WRITE_DIRECT, 0xef, 0x00 }, /* set SFRPAGE to 0x00 */
	{ C2_WRITE_DIRECT, 0xb2, 0x83 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};
static struct c2_setupcmd c2init_f32x[] = {
	{ C2_WRITE_SFR, 0xb2, 0x83 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f33x[] = {
	{ C2_WRITE_DIRECT, 0xb2, 0x83 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f35x[] = {
	{ C2_WRITE_SFR, 0xb6, 0x10 }, /* Set Flash read timing for 50 MHz */
	{ C2_WRITE_SFR, 0xab, 0x00 }, /* Reset Clk multiplier to intosc/2 */
	{ C2_WRITE_SFR, 0xbe, 0x80 }, /* Enable Clk multiplier */
	{ C2_WAIT_US, 0, 0, 5 }, /* wait for 5 us */
	{ C2_WRITE_SFR, 0xbe, 0xc0 }, /* Initialize the multiplier */
	{ C2_WAIT_US, 0, 0, 10000 }, /* wait for 10 ms */
	{ C2_READ_SFR, 0xbe }, /* read clk multiplier value */
	{ C2_WRITE_SFR, 0xa9, 0x02 }, /* Set clk mul as system clock */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f41x[] = {
	{ C2_WRITE_SFR, 0xb6, 0x10 }, /* Set Flash read timing for 50 MHz */
	{ C2_WRITE_SFR, 0xc9, 0x10 }, /* Set Voltage regulator for 2.5 V */
	{ C2_WRITE_SFR, 0xff, 0xa0 }, /* Enable VDD monitor at high threshold */
	{ C2_WRITE_SFR, 0xef, 0x02 }, /* Enable VDDmon as a reset source */
	{ C2_WRITE_SFR, 0xb2, 0x87 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_si8250[] = {
	{ C2_WRITE_SFR, 0xb2, 0x87 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f34x[] = {
	{ C2_WRITE_SFR, 0xb6, 0x90 }, /* Set Flash read timing for 50 MHz, one shot enabled */
	{ C2_WRITE_SFR, 0xff, 0x80 }, /* Enable VDD monitor */
	{ C2_WRITE_SFR, 0xef, 0x02 }, /* Enable VDDmon as a reset source */
	{ C2_WRITE_SFR, 0xb9, 0x00 }, /* Reset Clk multiplier to intosc */
	{ C2_WRITE_SFR, 0xb9, 0x80 }, /* Enable Clk multiplier */
	{ C2_WAIT_US, 0, 0, 5 }, /* wait for 5 us */
	{ C2_WRITE_SFR, 0xb9, 0xc0 }, /* Initialize the multiplier */
	{ C2_WAIT_US, 0, 0, 0x10000 }, /* wait for 10 ms */
	{ C2_READ_SFR, 0xb9 }, /* read clk multiplier value */
	{ C2_WRITE_SFR, 0xa9, 0x03 }, /* Set clk mul as system clock */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_t60x[] = {
	{ C2_WRITE_SFR, 0xb2, 0x07 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f52x[] = {
//	{ C2_WRITE_DIRECT, 0xb2, 0x87 }, /* set intosc to 24.5 MHz */
//	{ C2_WRITE_DIRECT, 0xff, 0xa0 }, /*enable VDD monitor to high setting */
	{ C2_WRITE_SFR, 0xb2, 0x87 }, /* set intosc to 24.5 MHz */
	{ C2_WRITE_SFR, 0xff, 0xa0 }, /* enable VDD monitor to high setting */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f36x[] = {
	{ C2_WRITE_DIRECT, 0xa7, 0x0f }, /* SFRPAGE = 0x0f;  Set SFRPAGE to 0xf */
	{ C2_WRITE_DIRECT, 0xb7, 0x83 }, /* OSCICN = 0x83;   Set internal osc to 24.5 MHZ */
	{ C2_WRITE_DIRECT, 0x8f, 0x00 }, /* CLKSEL = 0x00;   Set sysclk to intosc */
	{ C2_WRITE_DIRECT, 0xa7, 0x00 }, /* SFRPAGE = 0x00;  Set SFRPAGE to 0x0 */
	{ C2_DONE },
#if 0
	/*
	 * The following is the sequence to operate from a 98 MHz system clock.  Flash read time testing
	 * indicates that this provides no speed benefit over operating from a 24.5 MHz clock,
	 * so we leave this out and just use the above.
	 */
	{ C2_WRITE_DIRECT, 0xa7, 0x0f }, /* SFRPAGE = 0x0f;  Set SFRPAGE to 0xf */
	{ C2_WRITE_DIRECT, 0x84, 0x00 }, /* CCH0CN = 0x00;   Disable prefetch engine while writing to FLRD bits */
	{ C2_WRITE_DIRECT, 0xa7, 0x00 }, /* SFRPAGE = 0x00;  Set SFRPAGE to 0x0 */
	{ C2_WRITE_DIRECT, 0xb6, 0x30 }, /* FLSCL = 0x30;    Set Flash read timing for 100 MHz */
	{ C2_WRITE_DIRECT, 0xa7, 0x0f }, /* SFRPAGE = 0x0f;  Set SFRPAGE to 0xf */
	{ C2_WRITE_DIRECT, 0xb7, 0x83 }, /* OSCICN = 0x83;   Set internal osc to 24.5 MHZ */
	{ C2_WRITE_DIRECT, 0xb3, 0x00 }, /* PLL0CN = 0x00;   Select intosc as pll clock source */
	{ C2_WRITE_DIRECT, 0xb3, 0x01 }, /* PLL0CN = 0x01;   Enable PLL power */
	{ C2_WRITE_DIRECT, 0xa9, 0x01 }, /* PLL0DIV = 0x01;  Divide by 1 */
	{ C2_WRITE_DIRECT, 0xa9, 0x01 }, /* PLL0DIV = 0x01;  Divide by 1 */
	{ C2_WRITE_DIRECT, 0xb1, 0x04 }, /* PLL0MUL = 0x04;  Multiply by 4 */
	{ C2_WRITE_DIRECT, 0xb2, 0x01 }, /* PLL0FLT = 0x01;  ICO = 100 MHz, PD clock 25 MHz */
	{ C2_WAIT_US, .time = 5 }, /* wait for 5 us */
	{ C2_WRITE_DIRECT, 0xb3, 0x03 }, /* PLL0CN = 0x03;   Enable the PLL */
	{ C2_WAIT_US, .time = 10000 }, /* wait for 10 ms for PLL to start */
	{ C2_READ_DIRECT, 0xb3 }, /* PLL0CN = ??;     read lock value */
	{ C2_WRITE_DIRECT, 0x8f, 0x04 }, /* CLKSEL = 0x04;   Set PLL as system clock */
	{ C2_WRITE_DIRECT, 0x84, 0xe6 }, /* CCH0CN = 0xe6;   0xe7 for block write enable; re-enable prefetch */
	{ C2_WRITE_DIRECT, 0xa7, 0x00 }, /* SFRPAGE = 0x00;  Set SFRPAGE to 0x0 */
#endif
};

static struct c2_setupcmd c2init_t61x[] = {
	{ C2_WRITE_DIRECT, 0xcf, 0x00 }, /* set SFRPAGE to 0x00 */
	{ C2_WRITE_DIRECT, 0xb2, 0x83 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f336[] = {
	{ C2_WRITE_DIRECT, 0xb2, 0x83 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f50x[] = {
//	{ C2_WRITE_SFR, 0xff, 0xa0 }, /* set VDD monitor to high threshold */
	{ C2_WRITE_SFR, 0xef, 0x00 }, /* disable VDD monitor as a reset source */
	{ C2_WRITE_SFR, 0xff, 0xa0 }, /* set VDD monitor to high threshold */
	{ C2_WAIT_US, 0, 0, 100 }, /* wait 100 us for VDD monitor to stabilize */
	{ C2_WRITE_SFR, 0xef, 0x02 }, /* enable VDD monitor as a reset source */
	{ C2_WRITE_SFR, 0xa7, 0x0f }, /* set SFRPAGE to 0x0f */
	{ C2_WRITE_SFR, 0xa1, 0xc7 }, /* set OSCICN to 24 MHz */
	{ C2_WRITE_SFR, 0x8f, 0x00 }, /* set CLKSEL to intosc */
	{ C2_WRITE_SFR, 0xa7, 0x00 }, /* set SFRPAGE to 0x00 */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f92x[] = {
	{ C2_WRITE_DIRECT, 0xa7, 0x00 }, /* set SFRPAGE to 0x00 */
	{ C2_WRITE_DIRECT, 0xb2, 0x8F }, /* enable precision oscillator */
	{ C2_WRITE_DIRECT, 0xa9, 0x00 }, /* select precision oscillator divide by 1 as system clock source */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_t63x[] = {
	{ C2_WRITE_DIRECT, 0xcf, 0x00 }, /* set SFRPAGE to 0x00 */
	{ C2_WRITE_DIRECT, 0xb2, 0x83 }, /* set intosc to 24.5 MHz */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f90x[] = {
	{ C2_WRITE_DIRECT, 0xa7, 0x00 }, /* set SFRPAGE to 0x00 */
	{ C2_WRITE_DIRECT, 0xb2, 0x8F }, /* enable precision oscillator */
	{ C2_WRITE_DIRECT, 0xa9, 0x00 }, /* select precision oscillator divide by 1 as system clock source */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f58x[] = { // VDD monitor high; VREG high; INTOSC high; clksel INTOSC
//	{ C2_WRITE_DIRECT, 0xc9, 0x10 }, /* set VREG to high value */
//	{ C2_WRITE_DIRECT, 0xff, 0xa0 }, /* set VDD monitor to high threshold */
	{ C2_WRITE_SFR, 0xff, 0xa0 }, /* set VDD monitor to high threshold */
//	{ C2_WRITE_DIRECT, 0xa7, 0x0f }, /* set SFRPAGE to 0x0F */
//	{ C2_WRITE_DIRECT, 0xa1, 0xc7 }, /* set OSCICN to 0xC7 (24 MHz) */
//	{ C2_WRITE_DIRECT, 0xa7, 0x00 }, /* set SFRPAGE to 0x00 */
//	{ C2_WRITE_DIRECT, 0xb6, 0x10 }, /* set FLRT to >25 MHz operation */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f80x[] = {
//	{ C2_WRITE_SFR, 0xb2, 0x83 }, /* set OSCICN to 0x83 (24.5 MHz) */
	{ C2_DONE },
};

static struct c2_setupcmd c2init_f38x[] = {
	{ C2_WRITE_SFR, 0xb6, 0x90 }, /* Set Flash read timing for 50 MHz, one shot enabled */
	{ C2_WRITE_SFR, 0xff, 0x80 }, /* Enable VDD monitor */
	{ C2_WRITE_SFR, 0xef, 0x02 }, /* Enable VDDmon as a reset source */
	{ C2_WRITE_SFR, 0xb2, 0x83 }, /* set OSCICN to 0x83 (24.5 MHz) */
	{ C2_DONE },
};

static struct c2family families[] = {
	{ 0x04, "C8051F30x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_1, 0xb4, c2init_f30x },
	{ 0x08, "C8051F31x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f31x },
	{ 0x09, "C8051F32x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f32x },
	{ 0x0a, "C8051F33x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f33x },
	{ 0x0b, "C8051F35x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f35x },
	{ 0x0c, "C8051F41x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f41x },
	{ 0x0d, "C8051F326/7",           C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x0e, "Si825x",                C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_si8250 },
	{ 0x0f, "C8051F34x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xad, c2init_f34x },
	{ 0x10, "C8051T60x",               C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_t60x },
	{ 0x11, "C8051F52x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f52x },
	{ 0x12, "C8051F36x",             C2_MEM_TYPE_FLASH, 1024, 0, C2_SECURITY_C2_2, 0xb4, c2init_f36x },
	{ 0x13, "C8051T61x",               C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_t61x },
	{ 0x14, "C8051F336/7",           C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f336 },
	{ 0x15, "Si8100",                C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x16, "C8051F92x/93x",         C2_MEM_TYPE_FLASH, 1024, 0, C2_SECURITY_C2_2, 0xb4, c2init_f92x },
	{ 0x17, "C8051T63x",               C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_t63x },
	{ 0x18, "C8051T62x",               C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xad, NULL },
	{ 0x19, "C8051T622/3",             C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xad, NULL },
	{ 0x1a, "C8051T624/5",             C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xad, NULL },
	{ 0x1b, "C8051T606",               C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x1c, "C8051F50x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f50x },
	{ 0x1d, "C8051F338POE",          C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x1e, "C8051F70x/71x",         C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x1f, "C8051F90x/91x",         C2_MEM_TYPE_FLASH,  512, 1, C2_SECURITY_C2_2, 0xb4, c2init_f90x },
	{ 0x20, "C8051F58x/59x",         C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f58x },
	{ 0x21, "C8051F54x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x22, "C8051F54x/55x",         C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x23, "C8051F80x/81x/82x/83x", C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, c2init_f80x },
	{ 0x24, "Si4010",                  C2_MEM_TYPE_OTP,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x25, "C8051F99x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x26, "C8051F80x/81x/82x/83x", C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xb4, NULL },
	{ 0x28, "C8051F38x",             C2_MEM_TYPE_FLASH,  512, 0, C2_SECURITY_C2_2, 0xad, c2init_f38x },
};

static int process_setupcmd(struct c2tool_state *state,
			    struct c2_setupcmd *setupcmd)
{
	int res;

	switch (setupcmd->token) {
	case C2_DONE:
		return 1;
	case C2_WRITE_SFR:
		res = c2_write_sfr(setupcmd->reg, setupcmd->value);
		break;
	case C2_WRITE_DIRECT:
		res = c2_write_direct(state, setupcmd->reg, setupcmd->value);
		break;
	case C2_READ_SFR:
		res = c2_read_sfr(setupcmd->reg);
		break;
	case C2_READ_DIRECT:
		res = c2_read_direct(state, setupcmd->reg);
		break;
	case C2_WAIT_US:
		udelay(setupcmd->time);
		res = 0;
		break;
	default:
		res = -1;
		break;
	}

	return res;
}

int c2family_find(unsigned int device_id, struct c2family **family)
{
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(families); ++k) {
		if (families[k].device_id == device_id) {
			*family = &families[k];
			return 0;
		}
	}

	return -EINVAL;
}

int c2family_setup(struct c2tool_state *state)
{
	struct c2family *family = state->family;
	int res;
	struct c2_setupcmd *setupcmd = family->setup;

	if (!setupcmd)
		return 0;

	while ((res = process_setupcmd(state, setupcmd++)) == 0) {}

	if (res == 1)
		return 0;
	else
		return -EIO;
}


/*
 * C2 registers & commands defines
 */

/* C2 registers */
#define C2_DEVICEID		0x00
#define C2_REVID		0x01
#define C2_FPCTL		0x02

/* C2 interface commands */
#define C2_GET_VERSION	0x01
#define C2_DEVICE_ERASE	0x03
#define C2_BLOCK_READ	0x06
#define C2_BLOCK_WRITE	0x07
#define C2_PAGE_ERASE	0x08

#define C2_FPDAT_GET_VERSION	0x01
#define C2_FPDAT_GET_DERIVATIVE	0x02
#define C2_FPDAT_DEVICE_ERASE	0x03
#define C2_FPDAT_BLOCK_READ	0x06
#define C2_FPDAT_BLOCK_WRITE	0x07
#define C2_FPDAT_PAGE_ERASE	0x08
#define C2_FPDAT_DIRECT_READ	0x09
#define C2_FPDAT_DIRECT_WRITE	0x0a
#define C2_FPDAT_INDIRECT_READ	0x0b
#define C2_FPDAT_INDIRECT_WRITE	0x0c

#define C2_FPDAT_RETURN_INVALID_COMMAND	0x00
#define C2_FPDAT_RETURN_COMMAND_FAILED	0x02
#define C2_FPDAT_RETURN_COMMAND_OK	0x0D

#define C2_FPCTL_HALT		0x01
#define C2_FPCTL_RESET		0x02
#define C2_FPCTL_CORE_RESET	0x04

static void c2_write_ar(unsigned char addr)
{
	int i;

	/* START field */
	c2d_set('z');
	c2ck_set(1);
	c2ck_strobe();

	/* INS field (11b, LSB first) */
	c2d_set(1);
	c2ck_strobe();
	c2d_set(1);
	c2ck_strobe();

	/* ADDRESS field */
	for (i = 0; i < 8; i++) {
		c2d_set(addr & 0x01);
		c2ck_strobe();

		addr >>= 1;
	}

	/* STOP field */
	c2d_set('z');
	c2ck_strobe();
	c2ck_set('z');
}

static int c2_read_ar(unsigned char *addr)
{
	int i;

	/* START field */
	c2d_set('z');
	c2ck_set(1);
	c2ck_strobe();

	/* INS field (10b, LSB first) */
	c2d_set(0);
	c2ck_strobe();
	c2d_set(1);
	c2ck_strobe();

	/* ADDRESS field */
	c2d_set('z');
	*addr = 0;
	for (i = 0; i < 8; i++) {
		*addr >>= 1;	/* shift in 8-bit ADDRESS field LSB first */

		c2ck_strobe();
		if (c2d_get())
			*addr |= 0x80;
	}

	/* STOP field */
	c2ck_strobe();
	c2ck_set('z');

	return 0;
}

static int c2_write_dr(unsigned char dta)
{
	int timeout, i;

	/* START field */
	c2d_set('z');
	c2ck_set(1);
	c2ck_strobe();

	/* INS field (01b, LSB first) */
	c2d_set(1);
	c2ck_strobe();
	c2d_set(0);
	c2ck_strobe();

	/* LENGTH field (00b, LSB first -> 1 byte) */
	c2d_set(0);
	c2ck_strobe();
	c2d_set(0);
	c2ck_strobe();

	/* DATA field */
	for (i = 0; i < 8; i++) {
		c2d_set(dta & 0x01);
		c2ck_strobe();

		dta >>= 1;
	}

	/* WAIT field */
	c2d_set('z');
	timeout = 20;
	do {
		c2ck_strobe();
		if (c2d_get())
			break;

		udelay(1);
	} while (--timeout > 0);
	if (timeout == 0) {
		c2ck_set('z');
		return -EIO;
	}

	/* STOP field */
	c2ck_strobe();
	c2ck_set('z');

	return 0;
}

static int c2_read_dr(unsigned char *dta)
{
	int timeout, i;

	/* START field */
	c2d_set('z');
	c2ck_set(1);
	c2ck_strobe();

	/* INS field (00b, LSB first) */
	c2d_set(0);
	c2ck_strobe();
	c2d_set(0);
	c2ck_strobe();

	/* LENGTH field (00b, LSB first -> 1 byte) */
	c2d_set(0);
	c2ck_strobe();
	c2d_set(0);
	c2ck_strobe();

	/* WAIT field */
	c2d_set('z');
	timeout = 20;
	do {
		c2ck_strobe();
		if (c2d_get())
			break;

		udelay(1);
	} while (--timeout > 0);
	if (timeout == 0) {
		c2ck_set('z');
		return -EIO;
	}

	/* DATA field */
	*dta = 0;
	for (i = 0; i < 8; i++) {
		*dta >>= 1;	/* shift in 8-bit DATA field LSB first */

		c2ck_strobe();
		if (c2d_get())
			*dta |= 0x80;
	}

	/* STOP field */
	c2ck_strobe();
	c2ck_set('z');

	return 0;
}

static int c2_poll_in_busy(void)
{
	unsigned char addr;
	int ret, timeout = 2000;

	do {
		ret = (c2_read_ar(&addr));
		if (ret < 0)
			return -EIO;

		if (!(addr & 0x02))
			break;

		udelay(1);
	} while (--timeout > 0);
	if (timeout == 0) {
		return -EIO;
	}

	return 0;
}

static int c2_poll_out_ready(void)
{
	unsigned char addr;
	int ret;
	uint32_t timeout = 400000;	/* erase flash needs long time... */

	do {
		ret = (c2_read_ar(&addr));
		if (ret < 0)
			return -EIO;

		if (addr & 0x01)
			break;

		udelay(1);
	} while (--timeout > 0);
	if (timeout == 0) {
		return -EIO;
	}

	return 0;
}

int c2_read_sfr(unsigned char sfra)
{
	unsigned char dta;

	c2_write_ar(sfra);

	if (c2_read_dr(&dta) < 0)
		return -EIO;

	return dta;
}

int c2_write_sfr(unsigned char sfra, unsigned char dta)
{
	c2_write_ar(sfra);

	if (c2_write_dr(dta) < 0)
		return -EIO;

	return 0;
}

/*
 * Programming interface (PI)
 * Each command is executed using a sequence of reads and writes of the FPDAT register.
 */

static int c2_pi_write_command(unsigned char command)
{
	if (c2_write_dr(command) < 0)
		return -EIO;

	if (c2_poll_in_busy() < 0)
		return -EIO;

	return 0;
}

static int c2_pi_get_data(unsigned char *dta)
{
	if (c2_poll_out_ready() < 0)
		return -EIO;

	if (c2_read_dr(dta) < 0)
		return -EIO;

	return 0;
}

static int c2_pi_check_command(void)
{
	unsigned char response;

	if (c2_pi_get_data(&response) < 0)
		return -EIO;

	if (response != C2_FPDAT_RETURN_COMMAND_OK)
		return -EIO;

	return 0;
}

static int c2_pi_command(unsigned char command, int verify, unsigned char *result)
{
	if (c2_pi_write_command(command) < 0) 
		return -EIO;

	if (!verify)
		return 0;

	if (c2_pi_check_command() < 0) 
		return -EIO;

	if (!result)
		return 0;

	if (c2_pi_get_data(result) < 0) 
		return -EIO;

	return 0;
}

int c2_read_direct(struct c2tool_state *state, unsigned char reg)
{
	unsigned char dta;
	struct c2family *family = state->family;

	c2_write_ar(family->fpdat);

	if (c2_pi_command(C2_FPDAT_DIRECT_READ, 1, NULL))
		return -EIO;

	if (c2_pi_write_command(reg))
		return -EIO;

	if (c2_pi_write_command(0x01))
		return -EIO;
	if (c2_poll_out_ready() < 0)
		return -EIO;
	if (c2_read_dr(&dta) < 0)
		return -EIO;

	return dta;
}

int c2_write_direct(struct c2tool_state *state, unsigned char reg, unsigned char value)
{
	struct c2family *family = state->family;

	c2_write_ar(family->fpdat);

	if (c2_pi_command(C2_FPDAT_DIRECT_WRITE, 1, NULL))
		return -EIO;

	if (c2_pi_write_command(reg))
		return -EIO;

	if (c2_pi_write_command(0x01))
		return -EIO;

	if (c2_pi_write_command(value))
		return -EIO;

	return 0;
}

int c2_flash_read(struct c2tool_state *state, unsigned int addr, unsigned int length,
			 unsigned char *dest)
{
	struct c2family *family = state->family;

	c2_write_ar(family->fpdat);

	while (length) {
		unsigned int blocksize;
		unsigned int k;

		if (c2_pi_command(C2_FPDAT_BLOCK_READ, 1, NULL) < 0) 
			break;

		if (c2_pi_command(addr >> 8, 0, NULL) < 0) 
			break;

		if (c2_pi_command(addr & 0xff, 0, NULL) < 0) 
			break;

		if (length > 255) {
			if (c2_pi_command(0, 1, NULL) < 0) 
				break;
			blocksize = 256;
		} else {
			if (c2_pi_command(length, 1, NULL) < 0) 
				break;
			blocksize = length;
		}

		for (k = 0; k < blocksize; ++k) {
			unsigned char dta;

			if (c2_pi_get_data(&dta) < 0)
				return -EIO;
			if (dest)
				*dest++ = dta;
		}

		length -= blocksize;
		addr += blocksize;
	}

	if (length) return -EIO;
	else return 0;
}


static int c2_flash_write(struct c2tool_state *state, unsigned int addr, unsigned int length,
			 unsigned char *src)
{
	struct c2family *family = state->family;

	c2_write_ar(family->fpdat);

	while (length) {
		int i, k, psz;

		if (src) {
			for (i = length > 255 ? 255 : length - 1; i >= 0; i--) {
				  if (src[i] != 0xff) break;
			}
			psz = i + 1;
		} else psz = length > 256 ? 256 : length;

		if (psz > 0) {
			if (c2_pi_command(C2_FPDAT_BLOCK_WRITE, 1, NULL) < 0)
			  break;

			if (c2_pi_command(addr >> 8, 0, NULL) < 0)
			  break;
			if (c2_pi_command(addr & 0xff, 0, NULL) < 0)
			  break;

			if (c2_pi_command(psz == 256 ? 0 : psz, 1, NULL) < 0)
			  break;

			if (src) {
				for (k = 0; k < psz; k++) {
					if (c2_pi_command(src[k], 0, NULL) < 0) return -EIO;
				}
			} else {
				for (k = 0; k < psz; k++) {
					  if (c2_pi_command(c2_getc(), 0, NULL) < 0) return -EIO;
				}
			}
		} 

		if (length < 256) length = 0; else length -= 256;
		addr += 256;
		if (src) src += 256;
	}

	if (length) return -EIO;
	else return 0;
}

int c2_flash_erase(struct c2tool_state *state, unsigned char page)
{
	struct c2family *family = state->family;

	c2_write_ar(family->fpdat);

	if (c2_pi_command(C2_FPDAT_PAGE_ERASE, 1, NULL) < 0)
		return -EIO;

	if (c2_pi_command(page, 1, NULL) < 0)
		return -EIO;

	if (c2_pi_command(0, 1, NULL) < 0)
		return -EIO;

	return 0;
}

int c2_halt(void)
{
	c2_reset();

	udelay(2);

	c2_write_ar(C2_FPCTL);

	if (c2_write_dr(C2_FPCTL_RESET) < 0)
		return -EIO;

	if (c2_write_dr(C2_FPCTL_CORE_RESET) < 0)
		return -EIO;

	if (c2_write_dr(C2_FPCTL_HALT) < 0)
		return -EIO;

	udelay(20000);

	return 0;
}

int c2_get_device_info(struct c2_device_info *info)
{
	unsigned char dta;
	int ret;

	/* Select DEVID register for C2 data register accesses */
	c2_write_ar(C2_DEVICEID);

	/* Read and return the device ID register */
	ret = c2_read_dr(&dta);
	if (ret < 0)
		return -EIO;

	info->device_id = dta;

	/* Select REVID register for C2 data register accesses */
	c2_write_ar(C2_REVID);

	/* Read and return the revision ID register */
	ret = c2_read_dr(&dta);
	if (ret < 0)
		return -EIO;

	info->revision_id = dta;

	return 0;
}

int c2_get_pi_info(struct c2tool_state *state, struct c2_pi_info *info)
{
	unsigned char dta;
	int ret;
	struct c2family *family = state->family;

	/* Select FPDAT register for C2 data register accesses */
	c2_write_ar(family->fpdat);

	ret = c2_pi_command(C2_FPDAT_GET_VERSION, 1, &dta);
	if (ret < 0)
		return -EIO;

	info->version = dta;

	ret = c2_pi_command(C2_FPDAT_GET_DERIVATIVE, 1, &dta);
	if (ret < 0)
		return -EIO;

	info->derivative = dta;

	return 0;
}

int c2_flash_erase_device(struct c2tool_state *state)
{
	struct c2family *family = state->family;

	c2_write_ar(family->fpdat);

	if (c2_pi_command(C2_FPDAT_DEVICE_ERASE, 1, NULL) < 0)
		return -EIO;

	if (c2_pi_command(0xde, 0, NULL) < 0)
		return -EIO;

	if (c2_pi_command(0xad, 0, NULL) < 0)
		return -EIO;

	if (c2_pi_command(0xa5, 1, NULL) < 0)
		return -EIO;

	return 0;
}


int blast_silabs(void)
{
	struct c2_device_info info;
	struct c2_pi_info pi_info;
	struct c2tool_state state;
	unsigned int len;
	static unsigned char buf[1] = { 0 };

	len = c2_fopen();
	if (len == 0) return 0;

	printf("Initializing C2 bus to Silabs...");
	if (c2_halt() < 0 ||
	    c2_get_device_info(&info) < 0 ||
	    c2family_find(info.device_id, &state.family) < 0 ||
	    c2_get_pi_info(&state, &pi_info) < 0 ||
	    c2family_setup(&state) < 0) {
		printf("Failed\n");
		return 1;
	}
	printf("ok, %s detected\n", state.family->name);


	printf("Erasing Silabs ...");
	if (c2_flash_erase_device(&state) < 0) {
		return 1;
	}

	printf("Writing Silabs flash...");
	if (c2_flash_write(&state, 0, len, NULL) < 0) {
		return 1;
	}

	printf("Setting security byte on Silabs...");
	if (c2_flash_write(&state, (63 * 1024) - 1, 1, buf) < 0) {
		printf("Failed setting 64k size security bit\n");
		if (c2_flash_write(&state, (32 * 1024) - 1, 1, buf) < 0) {
			printf("Failed setting 32k size security bit\n");
			return 1;
		}
	} 
	printf("ok\n");

	c2_reset();

	return 0;
}
