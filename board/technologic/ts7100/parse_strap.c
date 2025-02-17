/*
 * Copyright (C) 2019-2023 Technologic Systems dba embeddedTS
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>

#include "tsfpga.h"
#include "parse_strap.h"

#define	NAND_CE0_B	IMX_GPIO_NR(4, 13)	/* Bit 0 / IO opt bit 0 */
#define	UART2_TX_DATA	IMX_GPIO_NR(1, 20)	/* Bit 1 / IO opt bit 1 */
#define	UART5_TX_DATA	IMX_GPIO_NR(1, 30)	/* Bit 2 / IO opt bit 2 */
#define	UART4_TX_DATA	IMX_GPIO_NR(1, 28)	/* Bit 3 / IO opt bit 3 */
#define	UART3_TX_DATA	IMX_GPIO_NR(1, 24)	/* Bit 4 / IO model bit 0 */
#define	LCD_DATA08	IMX_GPIO_NR(3, 13)	/* Bit 5 / IO model bit 1 */
#define	NAND_CE1_B	IMX_GPIO_NR(4, 14)	/* Bit 6 / IO model bit 2 */
#define	UART3_CTS_B	IMX_GPIO_NR(1, 26)	/* Bit 7 / IO model bit 3 */
#define	JTAG_TDO_N15	IMX_GPIO_NR(1, 12)	/* Rev B on pad N15 */

#define STRAP_PAD_PU_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const strap_pads[] = {
	MX6_PAD_NAND_CE0_B__GPIO4_IO13 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	MX6_PAD_UART2_TX_DATA__GPIO1_IO20 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	MX6_PAD_UART5_TX_DATA__GPIO1_IO30 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	MX6_PAD_UART3_TX_DATA__GPIO1_IO24 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	MX6_PAD_NAND_CE1_B__GPIO4_IO14 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	MX6_PAD_LCD_DATA08__GPIO3_IO13 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	MX6_PAD_UART3_CTS_B__GPIO1_IO26 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
};

static iomux_v3_cfg_t const rev_pads[] = {
	MX6_PAD_JTAG_TDO__GPIO1_IO12 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
};

const char *get_board_name(void)
{
	uint32_t cpu_opts;
	uint32_t io_model;
	uint32_t io_opts;
	board_read_straps(&cpu_opts, &io_opts, &io_model);

	if (io_model == 1) {
		return "TS-7100-Z";
	} else {
		return "TS-7100";
	}
}

const char get_cpu_board_version_char(void)
{
	uint16_t fpga_straps = readw(0x50004050);
	char rev = '0';

	imx_iomux_v3_setup_multiple_pads(rev_pads, ARRAY_SIZE(rev_pads));
	gpio_request(JTAG_TDO_N15, "JTAG_TDO_N15");
	gpio_direction_input(JTAG_TDO_N15);

	/*
	 * Any board rev newer than the newest here in this version of
	 * U-Boot *should* appear to be the newest board rev mentioned
	 * below (barring the unexpected):
	 */
	if (gpio_get_value(JTAG_TDO_N15) == 0) {
		rev = 'B';
	} else if ((fpga_straps & (1 << 12)) == 0) {
		rev = 'A';
	}

	/*
	 * If we reach this point, there is a problem with reading the
	 * hardware.  Return an invalid revision character to flag
	 * attention on the boot screen and that compares lower than 'A'.
	 */
	return rev;
}

/*
 * This board has resistor straps to detect different pcb/assembly options
 * cpu_straps - Which SOM is in use
 * 	cpu_straps[2:0] = {R34, R28, R29}
 * io_model - Which carrier board is in use (last 2 bits do not have a defined resistor)
 * 	io_model[3:0] = {UART3_CTS_B, NAND_CE1_B, R156, R151}
 * io_opts - Can be use to detect different population options
 * 	io_opts[3:0] = {R152, R153, R154, R155}
 * Each of these are set to 1 when the resistor is populated
 * See the Schematic for the full table of variants
 */
int board_read_straps(uint32_t *cpu_straps, uint32_t *io_opts, uint32_t *io_model)
{
	static uint32_t saved_cpu_straps;
	static uint32_t saved_io_opts;
	static uint32_t saved_io_model;
	static bool read = 0;
	uint32_t fpga_straps;

	if (!read) {
		*io_opts = 0;
		*io_model = 0;
		*cpu_straps = 0;
		imx_iomux_v3_setup_multiple_pads(strap_pads, ARRAY_SIZE(strap_pads));

		// CPU straps are on FPGA GPIO bank 2:
		fpga_straps = readw(0x50004050);
		if (fpga_straps & (1 << 3)) // B8 pad
			*cpu_straps |= (1 << 0); // R29
		if (fpga_straps & (1 << 4)) // C9 Pad
			*cpu_straps |= (1 << 1); // R28
		if (fpga_straps & (1 << 5)) // C8 Pad
			*cpu_straps |= (1 << 2); // R34
		*cpu_straps ^= 0x7; /* 1 = populated */

		gpio_request(NAND_CE0_B, "NAND_CE0_B");
		gpio_request(UART2_TX_DATA, "UART2_TX_DATA");
		gpio_request(UART5_TX_DATA, "UART5_TX_DATA");
		gpio_request(UART4_TX_DATA, "UART4_TX_DATA");
		gpio_request(UART3_TX_DATA, "UART3_TX_DATA");
		gpio_request(NAND_CE1_B, "NAND_CE1_B");
		gpio_request(LCD_DATA08, "LCD_DATA08");
		gpio_request(UART3_CTS_B, "UART3_CTS_B");

		gpio_direction_input(NAND_CE0_B);
		gpio_direction_input(UART2_TX_DATA);
		gpio_direction_input(UART5_TX_DATA);
		gpio_direction_input(UART4_TX_DATA);
		gpio_direction_input(UART3_TX_DATA);
		gpio_direction_input(NAND_CE1_B);
		gpio_direction_input(LCD_DATA08);
		gpio_direction_input(UART3_CTS_B);

		*io_opts |= (gpio_get_value(NAND_CE0_B) << 0); // R155
		*io_opts |= (gpio_get_value(UART2_TX_DATA) << 1); // R154
		*io_opts |= (gpio_get_value(UART5_TX_DATA) << 2); // R153
		*io_opts |= (gpio_get_value(UART4_TX_DATA) << 3); // R152
		*io_opts ^= 0xf; /* 1 = populated */

		*io_model |= (gpio_get_value(UART3_TX_DATA) << 0); // R151
		*io_model |= (gpio_get_value(LCD_DATA08) << 1); // R156
		*io_model |= (gpio_get_value(NAND_CE1_B) << 2); // Reserved
		*io_model |= (gpio_get_value(UART3_CTS_B) << 3); // Reserved
		*io_model ^= 0xf; /* 1 = populated */

		read = 1;
		saved_cpu_straps = *cpu_straps;
		saved_io_opts = *io_opts;
		saved_io_model = *io_model;
	} else {
		*cpu_straps = saved_cpu_straps;
		*io_opts = saved_io_opts;
		*io_model = saved_io_model;
	}

	return 0;
}
