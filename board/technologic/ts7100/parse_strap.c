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

/* Starting from Rev C schematic, a table was defined of all current and future
 * PCB revisions.
 *
 * These rely on reading specific GPIO pins that are tied to ground or V+ in
 * a look-up table to determine revision.
 *
 * The following was pulled from the Rev C schematic on 20260908:
 *
 * CPU Ball    FPGA Balls  0 = GND, 1 = NC/V+/Floating
 * N15         G3      C4      G12     Rev
 * 1           1       1       0       Rev A
 * 0           1       1       0       Rev B
 * 0           1       0       1       Rev C
 * 0           1       0       0       Rev D
 * 0           0       1       1       Rev E
 * 0           0       1       0       Rev F
 * 0           0       0       1       Rev G
 * 0           0       0       0       Rev H
 *
 * Table Bit:  2       1       0
 *
 * If additional revisions are needed, N15 can be extended as an upper bit
 *
 * Note that, on Rev A PCBs, reading the FPGA balls are invalid. While G12
 * is connected in the FPGA register, G3 and C4 are not connected until
 * Rev C. Rev B never was produced.
 */
struct fpga_strap_regs {
	u32 addr;
	u32 bit;
};

/* In bit order of the final table, from 0 to highest bit */
/* 0x4050 is bank 2 (0 indexed)
 *   bit 15: ball G12
 * 0x4040 is bank 1 (0 indexed)
 *   bit 15: ball G3
 *   bit 14: ball C4
 */
static struct fpga_strap_regs fpga_strap_regs[] = {
	{ 0x50004050, 15 },
	{ 0x50004040, 14 },
	{ 0x50004040, 15 },
};

const char get_cpu_board_version_char(void)
{
	uint8_t table = 0;
	char rev;
	int i;

	imx_iomux_v3_setup_multiple_pads(rev_pads, ARRAY_SIZE(rev_pads));
	gpio_request(JTAG_TDO_N15, "JTAG_TDO_N15");
	gpio_direction_input(JTAG_TDO_N15);

	/* We can quickly check for Rev A, if N15 is high, its Rev A */
	if (gpio_get_value(JTAG_TDO_N15))
		return 'A';

	/* Otherwise, we look up the rev from the table of straps.
	 * First, build the table.
	 */
	for (i = 0; i < ARRAY_SIZE(fpga_strap_regs); i ++) {
		if (readw(fpga_strap_regs[i].addr) & BIT(fpga_strap_regs[i].bit))
			table |= BIT(i);
	}
	/* Since straps are nonpop = 1, invert them */
	table ^= GENMASK(ARRAY_SIZE(fpga_strap_regs)-1, 0);

	switch (table) {
		case 0x01: rev = 'B'; break;
		case 0x02: rev = 'C'; break;
		case 0x03: rev = 'D'; break;
		case 0x04: rev = 'E'; break;
		case 0x05: rev = 'F'; break;
		case 0x06: rev = 'G'; break;
		case 0x07: rev = 'H'; break;
		/*
		 * If we reach this point, there is a problem with reading the
		 * hardware. Return an invalid revision character to flag
		 * attention on the boot screen and that compares lower than 'A'.
		 */
		default:   rev = '0'; break;
	}

	return rev;
}

/* These are ordered from bit 0, to highest bit */
static unsigned io_opts_gpio[] = {
	NAND_CE0_B,
	UART2_TX_DATA,
	UART5_TX_DATA,
	UART4_TX_DATA,
};

static unsigned io_model_gpio[] = {
	UART3_TX_DATA,
	LCD_DATA08,
	NAND_CE1_B,
	UART3_CTS_B,
};

static unsigned cpu_opts_fpga_bank2[] = {
	3, // B8 pad / R29
	4, // C9 pad / R28
	5, // C8 pad / R34
	1, // M12 pad / R27
};

/*
 * This board has resistor straps to detect different pcb/assembly options
 * cpu_straps - Which SOM is in use
 * 	cpu_straps[3:0] = {R27, R34, R28, R29}
 * io_model - Which carrier board is in use
 * 	io_model[3:0] = {R160, R157, R156, R151}
 * io_opts - Can be use to detect different population options
 * 	io_opts[3:0] = {R152, R153, R154, R155}
 *
 * Each of these are set to 1 when the resistor is populated
 * See the Schematic for the full table of variants
 *
 *
 * IMPORTANT NOTE!
 * cpu_strap[3] is not hooked up in older FPGA revisions. Ultimately, the value
 * of that bit read from the FPGA may change between builds. It is only after
 * FPGA Rev ~38 that this bit is valid. This U-Boot code will always return the
 * value of that bit and doesn't care about the FPGA revision. It is up to higher
 * level processes to determine, based on the board model, PCB rev, FPGA rev, etc.
 * if that bit is valid or not. That is not the job of U-Boot at this point.
 */
int board_read_straps(uint32_t *cpu_straps, uint32_t *io_opts, uint32_t *io_model)
{
	static uint32_t saved_cpu_straps;
	static uint32_t saved_io_opts;
	static uint32_t saved_io_model;
	static bool read = 0;
	uint32_t fpga_straps;
	int i;

	if (!read) {
		*io_opts = 0;
		*io_model = 0;
		*cpu_straps = 0;
		imx_iomux_v3_setup_multiple_pads(strap_pads, ARRAY_SIZE(strap_pads));

		// CPU straps are on FPGA GPIO bank 2:
		fpga_straps = readw(0x50004050);
		for (i = 0; i < ARRAY_SIZE(cpu_opts_fpga_bank2); i ++) {
			if (fpga_straps & BIT(cpu_opts_fpga_bank2[i]))
				*cpu_straps |= BIT(i);
		}
		/* 1 == populated */
		*cpu_straps ^= GENMASK(ARRAY_SIZE(cpu_opts_fpga_bank2)-1, 0);

		/* Requst all needed CPU pins */
		for (i = 0; i < ARRAY_SIZE(io_opts_gpio); i++) {
			gpio_request(io_opts_gpio[i], "strap");
			gpio_direction_input(io_opts_gpio[i]);
		}

		for (i = 0; i < ARRAY_SIZE(io_model_gpio); i++) {
			gpio_request(io_model_gpio[i], "strap");
			gpio_direction_input(io_model_gpio[i]);
		}

		/* Read pins */
		for (i = 0; i < ARRAY_SIZE(io_opts_gpio); i++) {
			*io_opts |= gpio_get_value(io_opts_gpio[i]) << i;
		}
		/* 1 == populated */
		*io_opts ^= GENMASK(ARRAY_SIZE(io_opts_gpio)-1, 0);

		for (i = 0; i < ARRAY_SIZE(io_model_gpio); i++) {
			*io_model |= gpio_get_value(io_model_gpio[i]) << i;
		}
		/* 1 == populated */
		*io_model ^= GENMASK(ARRAY_SIZE(io_model_gpio)-1, 0);

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
