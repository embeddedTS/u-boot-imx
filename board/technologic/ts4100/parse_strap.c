/*
 * Copyright (C) 2018-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <i2c.h>

uint8_t parse_strap(void)
{
	static uint8_t opts;
	int ret;

	i2c_set_bus_num(2);
	ret = i2c_read(0x28, 308, 2, &opts, 1);
	if (ret != 0) opts = 0;
	else opts = (opts & 0x1F);
		
	setenv_hex("opts", opts);
	return opts;
}
