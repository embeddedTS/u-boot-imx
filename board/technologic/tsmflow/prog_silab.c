#include <common.h>
#include <command.h>
#include <console.h>
#include <i2c.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <status_led.h>
#include "tsfpga.h"
#include "c2.h"

unsigned int c2_len;
unsigned char *c2_data;

/*************************************
 *  C8051 C2 reprogramming interface
 *************************************/

int c2d_get(void)
{
	return fpga_dio1_data_get(BANK1_SILAB_DATA);
}

 /* 1, 0, or ‘z’ */
void c2d_set(unsigned char state)
{
	switch (state) {
	  default:
	  case 'z':
		fpga_dio1_oe_clr(BANK1_SILAB_DATA);
		fpga_dio1_dat_clr(BANK1_SILAB_DATA);
		break;
	  case 1:
		fpga_dio1_dat_set(BANK1_SILAB_DATA);
		fpga_dio1_oe_set(BANK1_SILAB_DATA);
		break;
	  case 0:
		fpga_dio1_dat_clr(BANK1_SILAB_DATA);
		fpga_dio1_oe_set(BANK1_SILAB_DATA);
		break;
	}
}

unsigned int c2_fopen(void) {
	return c2_len;
}

unsigned char c2_getc(void) {
	unsigned char ret;
	ret = (*c2_data++);
	return ret;
}

void c2_reset(void) {
	fpga_dio1_dat_set(BANK1_SILAB_CLK);
	udelay(25*100000);
	fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	udelay(1);
}

void c2ck_set(unsigned char state)
{
	switch (state) {
	  case 1:
		/* TS-7100 SILAB_CLK from the FPGA is inverted. This bit drives
		 * a MOSFET gate which pulls the clock pin low. This is to
		 * prevent conflict when TS-9471 is installed which does direct
		 * drive of the SILAB_CLK to the uC itself.
		 */
		fpga_dio1_dat_clr(BANK1_SILAB_CLK);
		break;
	  case 0:
		/* This FPGA has a bit specifically for strobing SILAB_CLK.
		 * Technically, this case should never be reached. All of the
		 * code that calls c2ck_set() as a part of c2.c only ever calls
		 * it to set as a 1. It will then use the c2ck_strobe() function
		 * to issue a clock. For sake of completeness, we will set the
		 * clock here.
		 */
		fpga_dio1_dat_set(BANK1_SILAB_CLK);
		break;
	  case 'z':
	  default:
		break;
	}
}

void c2ck_strobe(void)
{
	/* SILAB_CLK_STB is auto-clearing after one wb_clk of being asserted. */
	fpga_dio3_dat_set(BANK3_SILAB_CLK_STB);
}

static int do_silabs_program(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc == 1) {
		printf("%s - %s\n\nUsage:\n%s %s\n",
		  cmdtp->name, cmdtp->usage, cmdtp->name, cmdtp->help);
		return 1;
	}

        c2_data = (unsigned char *)simple_strtoul(argv[1], NULL, 16);
        c2_len = simple_strtoul(argv[2], NULL, 16);

	/* Force 5V to stay on during programming */
	fpga_dio2_dat_set(BANK2_EN_PROG_SILAB);
	fpga_dio2_oe_set(BANK2_EN_PROG_SILAB);
	udelay(1000*1000);

	/* CLK is inverted, drive low to keep actual signal high
	 * This FPGA is set up to have a separate GPIO be a clock one-shot
	 * with auto clear. This reduces the possibility of some CPU delay
	 * during programming causing the uC to reset. Holding the C2 clk low
	 * in the uC for a long enough period will hardware reset the uC.
	 */
	fpga_dio1_dat_clr(BANK1_SILAB_CLK);
	fpga_dio1_oe_set(BANK1_SILAB_CLK);

	fpga_dio1_dat_set(BANK1_SILAB_DATA);
	fpga_dio1_oe_set(BANK1_SILAB_DATA);

	blast_silabs();

	fpga_dio2_oe_clr(BANK2_EN_PROG_SILAB);

	return 0;
}

U_BOOT_CMD(prog_silab, 3, 0, do_silabs_program,
        "TS supervisory microcontroller re-flashing tool",
        "<image address> <length>\n"
	"\nMISUSE MAY DAMAGE DEVICE!\n"
);
