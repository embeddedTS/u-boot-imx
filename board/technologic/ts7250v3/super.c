#include <common.h>
#include <dm/uclass.h>
#include <i2c.h>

#include "super.h"

static struct udevice *super_get_i2c_chip(void)
{
	struct udevice *chip;
	struct udevice *bus;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	if (ret)
		return NULL;

	ret = i2c_get_chip(bus, SUPER_I2C_ADDR, 2, &chip);
	if (ret)
		return NULL;

	return chip;
}

int super_write(uint16_t addr, uint16_t value)
{
	struct udevice *chip;
	uint16_t swap_addr;

	chip = super_get_i2c_chip();
	if (!chip)
		return -ENODEV;

	swap_addr = addr >> 8;
	swap_addr |= (addr & 0xff) << 8;

	return dm_i2c_write(chip, swap_addr, (uint8_t *)&value, 2);
}

int super_read(uint16_t addr, uint16_t *value)
{
	struct udevice *chip;
	uint16_t swap_addr;

	chip = super_get_i2c_chip();
	if (!chip) {
		return -ENODEV;
	}

	swap_addr = addr >> 8;
	swap_addr |= (addr & 0xff) << 8;
	return dm_i2c_read(chip, swap_addr, (uint8_t *)value, 2);
}
