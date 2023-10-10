/*
 * Copyright (C) 2019-2023 Technologic Systems dba embeddedTS
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

/*
 * Knowledge of TS-7100 strapping is encapsulated in this file.
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

/*
 * For Rev A: IO_MODEL is non-zero
 * For Rev B: MX6UL_PAD_JTAG_TDO__GPIO1_IO12 (pad N15) reads as 1 instead of 0
 *
 * Five straps on the CPU board go into the FPGA:
 *   RAM size:
 *     IO_B0 (C6)
 *   CPU opts:
 *     DONE_IO_B0 (A13)
 *     IO_B0_SCL? (C8)
 *     IO_B0 (C9)
 *     IO_B0_SDA? (B8),
 */

#define STRAP_PAD_PU_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const strap_pads[] = {
	/* IO Strap 0, WIFI_SPI_CLK */
	MX6_PAD_NAND_CE0_B__GPIO4_IO13 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	/* IO Strap 1, UART2_TXD */
	MX6_PAD_UART2_TX_DATA__GPIO1_IO20 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	/* IO Strap 2, UART5_TXD */
	MX6_PAD_UART5_TX_DATA__GPIO1_IO30 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	/* IO Strap 3, UART4_TXD */
	MX6_PAD_UART4_TX_DATA__GPIO1_IO28 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	/* IO Strap 4, UART3_TXD */
	MX6_PAD_UART3_TX_DATA__GPIO1_IO24 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	/* IO Strap 5, WIFI_SPI_MOSI */
	MX6_PAD_NAND_CE1_B__GPIO4_IO14 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	/* IO Strap 6, CAN_1_TXD */
	MX6_PAD_LCD_DATA08__GPIO3_IO13 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
	/* IO Strap 7, UART3_CTS */
	MX6_PAD_UART3_CTS_B__GPIO1_IO26 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),

	/* JTAG_TDO (Pad N15) */
	MX6_PAD_JTAG_TDO__GPIO1_IO12 | MUX_PAD_CTRL(STRAP_PAD_PU_CTRL),
};

const char *get_board_model(void)
{
	return "7100";
}

const char *get_board_name(void)
{
	uint8_t io_model;
	static char name_str[12] = {0};

	io_model = read_io_board_model();
	if (io_model == 1) {
		snprintf(name_str, sizeof(name_str),
			 "TS-7100-Z");
	} else if (io_model == 0) {
		snprintf(name_str, sizeof(name_str),
			 "TS-%4s", get_board_model());
	} else {
		snprintf(name_str, sizeof(name_str),
			 "TS-%4s-C%02d", get_board_model(), io_model);
	}
	return name_str;
}

const char get_cpu_board_version_char(void)
{
	uint16_t raw_cpu_straps = 0;
	uint16_t raw_fpga_straps = 0;

	raw_cpu_straps =  read_raw_cpu_straps();
	raw_fpga_straps =  read_raw_fpga_straps();

	/*
	 * Any board rev newer than the newest here in this version of
	 * U-Boot *should* appear to be the newest board rev mentioned
	 * below (barring the unexpected):
	 */
	if (raw_cpu_straps & (1 << 8)) {     // Rev B strap on the 6UL
		return 'B';
	}
	if (raw_fpga_straps & (1 << 12)) {   // Rev A strap on the FPGA
		return 'A';
	}

	/*
	 * If we reach this point, there is a problem with reading the
	 * hardware.  Return an invalid revision character to flag
	 * attention on the boot screen and that compares lower than 'A'.
	 */
	return '0';
}

const char *get_straps_str(void)
{
	uint16_t raw_cpu_straps = 0;
	uint16_t raw_fpga_straps = 0;
	static char straps_str[10] = {0};

	raw_cpu_straps =  read_raw_cpu_straps();
	raw_fpga_straps =  read_raw_fpga_straps();

	snprintf(straps_str, sizeof(straps_str),
		 "%04x-%04x", raw_cpu_straps, raw_fpga_straps);
	return straps_str;
}

const char *get_cpu_board_version_str(void)
{
	char rev_char = 0;
	uint8_t io_model = 0;
	static char model_str[24] = {0};

	rev_char = get_cpu_board_version_char();

	if (rev_char != '0') {
		snprintf(model_str, sizeof(model_str),
			 "%c/%s", rev_char, get_straps_str());
		return model_str;
	}

	/*
	 * Now handle some strange cases that appear on
	 * broken/unsupported hardware that we want to quickly
	 * recognize and debug if/when they happen.
	 */
	io_model = read_io_board_model();
	if (io_model == 0) {
		snprintf(model_str, sizeof(model_str),
			 "P2/%s", get_straps_str());
	} else if (io_model == 1) {
		snprintf(model_str, sizeof(model_str),
			 "A/%s", get_straps_str());
	} else if (io_model == 3) {
		snprintf(model_str, sizeof(model_str),
			 "B/%s", get_straps_str());
	} else {
		snprintf(model_str, sizeof(model_str),
			 "UNKNOWN_IO_MODEL_%02d/%s", io_model, get_straps_str());
	}
	return model_str;
}

uint8_t read_cpu_board_opts(void)
{
	uint16_t fpga_straps;
	uint8_t cpu_opts;

	/*
	 * bits 3:0 are FPGA GPIO bank 3, 5:2 and are purely straps
	 * bits 5:4 are FPGA GPIO bank 3, 12:11 and are DIO_18:DIO_17
	 * bank 3 12:11 are latched values of DIO_18:DIO_17 after unreset
	 */
	fpga_straps = read_raw_fpga_straps();
	cpu_opts = (((fpga_straps & 0x1800) >> 7) | ((fpga_straps & 0x3C) >> 2));

	return cpu_opts;
}

uint8_t read_io_board_model(void)
{
	uint8_t io_model;
	io_model = (read_raw_cpu_straps() & 0xF0) >> 4;
	return io_model;
}

uint8_t read_io_board_opts(void)
{
	uint8_t io_opts;
	io_opts = (uint32_t)(read_raw_cpu_straps() & 0x0F);
	return io_opts;
}

uint16_t read_raw_cpu_straps(void)
{
	static uint16_t cpu_straps = 0;
	static uint8_t read;

	if (!read) {
		imx_iomux_v3_setup_multiple_pads(strap_pads, ARRAY_SIZE(strap_pads));

		gpio_request(NAND_CE0_B, "NAND_CE0_B");
		gpio_request(UART2_TX_DATA, "UART2_TX_DATA");
		gpio_request(UART5_TX_DATA, "UART5_TX_DATA");
		gpio_request(UART4_TX_DATA, "UART4_TX_DATA");
		gpio_request(UART3_TX_DATA, "UART3_TX_DATA");
		gpio_request(NAND_CE1_B, "NAND_CE1_B");
		gpio_request(LCD_DATA08, "LCD_DATA08");
		gpio_request(UART3_CTS_B, "UART3_CTS_B");

		gpio_request(JTAG_TDO_N15, "JTAG_TDO_N15");

		gpio_direction_input(NAND_CE0_B);
		gpio_direction_input(UART2_TX_DATA);
		gpio_direction_input(UART5_TX_DATA);
		gpio_direction_input(UART4_TX_DATA);
		gpio_direction_input(UART3_TX_DATA);
		gpio_direction_input(NAND_CE1_B);
		gpio_direction_input(LCD_DATA08);
		gpio_direction_input(UART3_CTS_B);

		gpio_direction_input(JTAG_TDO_N15);

		mdelay(1);

		cpu_straps |= (gpio_get_value(NAND_CE0_B) << 0);
		cpu_straps |= (gpio_get_value(UART2_TX_DATA) << 1);
		cpu_straps |= (gpio_get_value(UART5_TX_DATA) << 2);
		cpu_straps |= (gpio_get_value(UART4_TX_DATA) << 3);
		cpu_straps |= (gpio_get_value(UART3_TX_DATA) << 4);
		cpu_straps |= (gpio_get_value(LCD_DATA08) << 5);
		cpu_straps |= (gpio_get_value(NAND_CE1_B) << 6);
		cpu_straps |= (gpio_get_value(UART3_CTS_B) << 7);

		cpu_straps |= (gpio_get_value(JTAG_TDO_N15) << 8);

		/*
		 * All straps are 1 = resistor populated. This is inverted from
		 * the logic level read from the IO pins.
		 */
		cpu_straps ^= 0x1FF;

		read = 1;
	}
	return cpu_straps;
}

uint16_t read_raw_fpga_straps(void)
{
	static uint16_t fpga_straps;
	static uint8_t read;
	uint32_t fpga_rev = readl(FPGA_REV);
	uint16_t saved_straps;

	/* 
	 * Here and now we need to read latched FPGA values.
	 *
	 *    | Pull   | Pad | Bank | Bit | FPGA Label | Net Label   |
	 *    |--------+-----+------+-----+------------+-------------+
	 *    | RN17-D | C13 |    3 |  12 | IO_B1      | SEL_NIM_USB |
	 *    | RN17-C | A13 |    3 |  11 | DONE_IO_B0 | NIM_PWR_ON  |
	 *    | R34    | C8  |    3 |   3 | IO_B0_SCL  | Strap       |
	 *    | R28    | C9  |    3 |   2 | IO_B0      | Strap       |
	 *    | R29    | B8  |    3 |   1 | IO_B0_SDA  | Strap       |
	 *    | R36    | C6  |    3 |   0 | IO_B0      | RAM Strap   |
	 *    | GND    | G12 |      |     | IO_B1      | Rev. A ID   |
	 *    |--------+-----+------+-----+------------+-------------+
	 */
	if (!read) {
		fpga_straps = readw(0x50004050); /* DIO bank 3 */
		fpga_straps ^= 0xFFFF;
		fpga_straps &= 0x180F;
		if (fpga_rev >= 8) {
			saved_straps = readl(FPGA_STRAPS);
			saved_straps |= (fpga_straps << 16);
			writel(saved_straps, FPGA_STRAPS);
		} else if (fpga_rev == 7) {
			/* Intermediate version with this enhancement but only 16 bits of STRAPS save space */
			saved_straps = readw(FPGA_STRAPS);
			saved_straps |= (fpga_straps << 8);
			writew(saved_straps, FPGA_STRAPS);
		}
		read = 1;
	}

	return fpga_straps;
}
