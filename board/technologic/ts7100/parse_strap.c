/*
 * Copyright (C) 2019-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>

#define	NAND_CE0_B	IMX_GPIO_NR(4, 13)	/* Bit 0 */
#define	UART2_TX_DATA	IMX_GPIO_NR(1, 20)	/* Bit 1 */
#define	UART5_TX_DATA	IMX_GPIO_NR(1, 30)	/* Bit 2 */
#define	UART4_TX_DATA	IMX_GPIO_NR(1, 28)	/* Bit 3 */
#define	UART3_TX_DATA	IMX_GPIO_NR(1, 24)	/* Bit 4 */
#define	NAND_CE1_B	IMX_GPIO_NR(4, 14)	/* Bit 5 */
#define	LCD_DATA08	IMX_GPIO_NR(3, 13)	/* Bit 6 */
#define	UART3_CTS_B	IMX_GPIO_NR(1, 26)	/* Bit 7 */


/* NOTE: Before this function is called, it expects that the IOMUX be set up
 * ahead of time. Meaning, pin function (GPIO) and pullup need to be set up
 * before the parse_strap() function is called.
 */
uint8_t parse_strap(const char *env_name)
{
	static uint8_t opts;
	static uint8_t read;

	if(!read) {
		gpio_direction_input(NAND_CE0_B);
		gpio_direction_input(UART2_TX_DATA);
		gpio_direction_input(UART5_TX_DATA);
		gpio_direction_input(UART4_TX_DATA);
		gpio_direction_input(UART3_TX_DATA);
		gpio_direction_input(NAND_CE1_B);
		gpio_direction_input(LCD_DATA08);
		gpio_direction_input(UART3_CTS_B);

		mdelay(1);

		opts |= (gpio_get_value(NAND_CE0_B) << 0);
		opts |= (gpio_get_value(UART2_TX_DATA) << 1);
		opts |= (gpio_get_value(UART5_TX_DATA) << 2);
		opts |= (gpio_get_value(UART4_TX_DATA) << 3);
		opts |= (gpio_get_value(UART3_TX_DATA) << 4);
		opts |= (gpio_get_value(NAND_CE1_B) << 5);
		opts |= (gpio_get_value(LCD_DATA08) << 6);
		opts |= (gpio_get_value(UART3_CTS_B) << 7);

		/* All straps are 1 = resistor populated. This is inverted from
		 * the logic level read from the IO pins.
		 */
		opts ^= 0xFF;

		read = 1;
	}
	if (env_name) setenv_hex(env_name, opts);
	return opts;
}
