/*
 * Copyright (C) 2018-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include "fpga.h"

int bbdetect(void)
{
	static int id;
	int i;
	static uint8_t read;

	if (!read) {
		for(i = 0; i < 8; i++) {
			if(i & 1)fpga_gpio_output(RED_LED_PADN, 1);
			else fpga_gpio_output(RED_LED_PADN, 0);

			if(i & 2)fpga_gpio_output(GREEN_LED_PADN, 1);
			else fpga_gpio_output(GREEN_LED_PADN, 0);
		
			if(i & 4)fpga_gpio_output(DIO_20, 1);
			else fpga_gpio_output(DIO_20, 0);

			id = (id >> 1);
			if(fpga_gpio_input(DIO_05)) id |= 0x80;
		}
		printf("Baseboard ID: 0x%X\n", id & ~0xc0);
		printf("Baseboard Rev: %d\n", ((id & 0xc0) >> 6));
	}
	setenv_hex("baseboardid", id & ~0xc0);
	setenv_hex("baseboardrev", ((id & 0xc0) >> 6));

	return id;
}

