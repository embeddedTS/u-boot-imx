/*
 * Copyright (C) 2019-2022 Technologic Systems dba embeddedTS
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

#include "parse_strap.h"

#define	NAND_CE0_B	IMX_GPIO_NR(4, 13)	/* Bit 0 / IO opt bit 0 */
#define	UART2_TX_DATA	IMX_GPIO_NR(1, 20)	/* Bit 1 / IO opt bit 1 */
#define	UART5_TX_DATA	IMX_GPIO_NR(1, 30)	/* Bit 2 / IO opt bit 2 */
#define	UART4_TX_DATA	IMX_GPIO_NR(1, 28)	/* Bit 3 / IO opt bit 3 */
#define	NAND_CE1_B	IMX_GPIO_NR(4, 14)	/* Bit 4 / IO model bit 1 */
#define	UART3_TX_DATA	IMX_GPIO_NR(1, 24)	/* Bit 5 / IO model bit 0 */
#define	LCD_DATA08	IMX_GPIO_NR(3, 13)	/* Bit 6 / IO model bit 2 */
#define	UART3_CTS_B	IMX_GPIO_NR(1, 26)	/* Bit 7 / IO model bit 3 */

/*
 * For Rev A:
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
};

const char *get_board_model(void)
{
	return "7100";
}

const char *get_board_name(void)
{
	uint8_t io_model;

	io_model = read_io_board_model();
	if (io_model == 0) {
		return "TS-7100";
	} else if (io_model == 1) {
		return "TS-7100-Z";
	}
	return "TS-7100";
}

const char *get_cpu_board_version(void)
{
	uint8_t io_model = 0;
	uint8_t cpu_board_rev = 0;
	uint16_t raw_fpga_straps = 0;
	static char model_str[24] = {0};

	io_model = read_io_board_model();
	raw_fpga_straps =  read_raw_fpga_straps();

	cpu_board_rev |= ((raw_fpga_straps >> 12) & 0x01); // Rev A 0x01 strap on the FPGA

	if (cpu_board_rev & 0x1) {
		snprintf(model_str, sizeof(model_str), "A");
		return model_str;
	}

	if (io_model == 0) {
		snprintf(model_str, sizeof(model_str),
			 "P2-%02x/%04x-%04x", cpu_board_rev, read_raw_cpu_straps(), raw_fpga_straps);
	} else if (io_model == 1) {
		snprintf(model_str, sizeof(model_str),
			 "Am-%02x/%04x-%04x", cpu_board_rev, read_raw_cpu_straps(), raw_fpga_straps);
	} else {
		snprintf(model_str, sizeof(model_str),
			 "UNKNOWN-%02x/%04x-%04x", cpu_board_rev, read_raw_cpu_straps(), raw_fpga_straps);
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

		gpio_direction_input(NAND_CE0_B);
		gpio_direction_input(UART2_TX_DATA);
		gpio_direction_input(UART5_TX_DATA);
		gpio_direction_input(UART4_TX_DATA);
		gpio_direction_input(UART3_TX_DATA);
		gpio_direction_input(NAND_CE1_B);
		gpio_direction_input(LCD_DATA08);
		gpio_direction_input(UART3_CTS_B);

		mdelay(1);

		cpu_straps |= (gpio_get_value(NAND_CE0_B) << 0);
		cpu_straps |= (gpio_get_value(UART2_TX_DATA) << 1);
		cpu_straps |= (gpio_get_value(UART5_TX_DATA) << 2);
		cpu_straps |= (gpio_get_value(UART4_TX_DATA) << 3);
		cpu_straps |= (gpio_get_value(UART3_TX_DATA) << 4);
		cpu_straps |= (gpio_get_value(LCD_DATA08) << 5);
		cpu_straps |= (gpio_get_value(NAND_CE1_B) << 6);
		cpu_straps |= (gpio_get_value(UART3_CTS_B) << 7);

		/* All straps are 1 = resistor populated. This is inverted from
		 * the logic level read from the IO pins.
		 */
		cpu_straps ^= 0xFF;

		read = 1;
	}
	return cpu_straps;
}
