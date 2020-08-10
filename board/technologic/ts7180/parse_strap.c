/*
 * Copyright (C) 2017 Technologic Systems
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>

uint8_t parse_strap(void)
{
	static uint8_t opts;
	static uint8_t read;

	if(!read) {
		gpio_direction_input(IMX_GPIO_NR(3, 23));
		gpio_direction_input(IMX_GPIO_NR(3, 24));
		gpio_direction_input(IMX_GPIO_NR(3, 27));
		gpio_direction_input(IMX_GPIO_NR(3, 28));

		mdelay(1);

		opts |= (gpio_get_value(IMX_GPIO_NR(3, 23)) << 0);
		opts |= (gpio_get_value(IMX_GPIO_NR(3, 24)) << 1);
		opts |= (gpio_get_value(IMX_GPIO_NR(3, 27)) << 2);
		opts |= (gpio_get_value(IMX_GPIO_NR(3, 28)) << 3);

		read = 1;
	}
	setenv_hex("opts", opts);
	return opts;
}
