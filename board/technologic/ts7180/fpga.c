#undef DEBUG
/*
 * Copyright (C) 2016, 2021 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <i2c.h>

#include <configs/ts7180.h>
#include "tsfpga.h"

/*
 * ts7100.c on v2020 is now using setup_i2c to set up the NXP's i2c bus.
 * This is still being ported from v2016.
 */
#ifndef CONFIG_SPL_BUILD
int fpga_get_rev(void)
{
	struct udevice *dev;
	uint8_t val = 0;
	int ret;

        ret = i2c_get_chip_for_busnum(2, 0x28, 2, &dev);
        if (ret) {
          pr_err("fpga_get_rev: ERROR: couldn't access i2c bus index 2 for FPGA\n");
          return ret;
        }

        ret = dm_i2c_read(dev, 306, &val, 1);
	if (ret != 0) return ret;
	return val;
}

/*
 * See: https://docs.embeddedarm.com/TS-7180#FPGA
 */
void fpga_gpio_output(int io, int value)
{
	uint8_t val;
        struct udevice *dev;
        uint8_t ret;

	// OE 
	val = 0x1;
	// DATA
	if(value) val |= 0x2;

        debug("fpga_gpio_output: Write: Adr 0x28, subadr 0x%X\n", io);
        ret = i2c_get_chip_for_busnum(2, 0x28, 2, &dev);
        if (ret) {
          pr_err("fpga_gpio_output: couldn't get i2c bus index 2\n");
          return;
        }

        dm_i2c_write(dev, io, &val, 1);
        debug("Write GPIO: val => 0x%X for 0x%X\n", val, value);
}

int fpga_gpio_input(int io)
{
	uint8_t val = 0;
	// Set input with 0
        struct udevice *dev;
        uint8_t ret;

        debug("Write/READ GPIO: Adr 0x28, io 0x%X\n", io);
        ret = i2c_get_chip_for_busnum(2, 0x28, 2, &dev);
        if (ret) {
          pr_err("fpga_gpio_input: couldn't get i2c bus index 2, ret=%d\n", ret);
          //printf("couldn't get i2c bus 2: ret = %02x", ret)
          return ret;
        }

        /* Why would we write before read? To ensure direction is set. */
        ret = dm_i2c_write(dev, io, &val, 1);
        if (ret) {
          pr_err("fpga_gpio_input: write value to bus 2 failed\n");
          //printf("write value %02X to bus 2 io %s failed - %d\n", val, io, ret);
          return ret;
        }
        dm_i2c_read(dev, io, &val, 1);

        debug("Write/Read GPIO: val <= 0x%X\n", val);

	return (val & 0x4) ? 1 : 0;
}
#endif /* !SPL */

void fpga_reset(void)
{
	/* reset the FGPA */
	gpio_direction_output(FPGA_RESETN, 0);
	mdelay(1);
	gpio_direction_output(FPGA_RESETN, 1);

}
