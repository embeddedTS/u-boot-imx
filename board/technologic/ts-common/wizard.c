// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Technologic Systems, Inc. (dba embeddedTS)
 *
 * This provides an abstraction to the wizard microcontroller
 * on embeddedTS platforms. This device uses 16-bit address/data
 */

#include <dm/uclass.h>
#include <i2c.h>

#include "wizard.h"

/*
 * wizard_get_i2c_chip - Locate the Wizard
 *
 * On early prototypes the wizard is on bus 0, on new designs
 * it is on bus 3. When the early prototypes are dropped this
 * can be simplified to just use bus 3, or locate the compatible node
 * on the device tree.
 *
 * This has to work using the provisional/default device tree; before
 * the correct device tree for this board model has been selected.
 */
static struct udevice *wizard_get_i2c_chip(void)
{
	struct udevice *chip;
	struct udevice *bus;
	u16 value;

	if (uclass_get_device_by_seq(UCLASS_I2C, 3, &bus))
		return NULL;

	if (i2c_get_chip(bus, WIZARD_I2C_ADDR, 2, &chip))
		return NULL;

	if (i2c_set_chip_offset_len(chip, 2))
		return NULL;

	if (dm_i2c_read(chip, 0, (uint8_t *)&value, sizeof(value)))
		return NULL;

	return chip;
}

int wizard_write(u16 addr, u16 value)
{
	struct udevice *chip;

	chip = wizard_get_i2c_chip();
	if (!chip)
		return -ENODEV;

	return dm_i2c_write(chip, cpu_to_be16(addr), (uint8_t *)&value, 2);
}

int wizard_read(u16 addr, u16 *value)
{
	struct udevice *chip;
	int ret;

	chip = wizard_get_i2c_chip();
	if (!chip) {
		printf("Error: No I2C chip found\n");
		return -ENODEV;
	}

	ret = dm_i2c_read(chip, cpu_to_be16(addr), (uint8_t *)value, 2);
	if (ret) {
		printf("Error: dm_i2c_read failed, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

int wizard_read_mac(uint8_t *mac_buffer)
{
	u16 reg_addr = WIZARD_SERIAL;
	int n_words = 3;
	u16 word;
	int ret;

	while (n_words--) {
		ret = wizard_read(reg_addr, &word);
		if (ret) {
			printf("i2c read failed at addr %04x, rc=%d (-ve)\n",
			       reg_addr, ret);
			break;
		}
		mac_buffer[2 * (2 - n_words)] = word & 0xff;
		mac_buffer[2 * (2 - n_words) + 1] = (word >> 8) & 0xff;
		reg_addr += 1;
	}

	return ret;
}
