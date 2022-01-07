/*
 * Copyright (C) 2018-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>

int fpga_get_rev(void)
{
	uint8_t val = 0;
	int ret;
	i2c_set_bus_num(2);
	ret = i2c_read(0x28, 306, 2, &val, 1);
	if(ret != 0) return ret;
	return val;
}

/* We need to to a RMW here. Read the current OE state, write that and the new
 * value to the reg, and then if OE was not set, set it on a separate write.
 * If this is not done, it is possible that the output state can change
 * non-deterministically in relation to OE.
 */
void fpga_gpio_output(int io, int value)
{
	uint8_t val;

	i2c_set_bus_num(2);

	// DATA
	i2c_read(0x28, io, 2, &val, 1);
	if(value) val |= 0x2;
	else val &= ~(0x2);
	i2c_write(0x28, io, 2, &val, 1);

	// OE
	if(!(val & 0x1)) { //Only update OE if currently not an output
		val |= 0x1;
		i2c_write(0x28, io, 2, &val, 1);
	}
}

/* We need to do a RMW, and then read the data on the input.
 * We want to keep the output state while changing the pin to an input.
 * If we don't, it is possible to change the state of the output before OE is
 * set. This is a bad thing to happen.
 */
int fpga_gpio_input(int io)
{
	uint8_t val;

	i2c_set_bus_num(2);

	i2c_read(0x28, io, 2, &val, 1);
	if(val & 0x1) { //Only do the RMW if the pin is an output
		val &= ~(0x1); //Clear the OE bit to set as input
		i2c_write(0x28, io, 2, &val, 1);
		i2c_read(0x28, io, 2, &val, 1);
	}
	return (val & 0x4) ? 1 : 0;
}
