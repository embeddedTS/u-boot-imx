#include <stdint.h>
#include <common.h>
#include <asm/gpio.h>
#include <linux/sizes.h>
#include <asm/io.h>
#include <common.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/iomux-v3.h>
#include <i2c.h>
#include <console.h>

#include "micro.h"

#define MIN_CHARGE_MV 3680
#define MAX_CHARGE_MV 4800
#define POWER_FAIL IMX_GPIO_NR(5, 0)
#define MICRO_ADDR 0x54

static bool read_power_fail_status(void);

int micro_read(uint16_t addr, void *data, size_t size)
{
	struct udevice *dev;
	int ret;

	ret = i2c_get_chip_for_busnum(0, MICRO_ADDR, 2, &dev);
	if (ret) {
		pr_err("couldn't get i2c bus - %d\n", ret);
		return ret;
	}
	return dm_i2c_read(dev, addr, data, size);
}

int micro_write(uint16_t addr, const void *data, size_t size)
{
	struct udevice *dev;
	int ret = i2c_get_chip_for_busnum(0, MICRO_ADDR, 2, &dev);
	if (ret) {
		pr_err("couldn't get i2c bus - %d\n", ret);
		return ret;
	}

	return dm_i2c_write(dev, addr, data, size);
}

static int micro_read16_swap(int addr, uint16_t *data)
{
	int result = micro_read(addr, (uint16_t *)data, sizeof(uint16_t));
	if (result >= 0)
		*data = swab16(*data);
	return result;
}

uint8_t micro_scaps_remaining_pct(void)
{
	uint16_t current_voltage;
	uint32_t voltage_range, normalized_voltage;
	uint8_t remaining_percentage;

	// Read the current supercap voltage
	if (micro_read16_swap(MICRO_ADC_8, &current_voltage) < 0) {
		printf("Failed to read current supercap voltage");
		return 0;
	}

	// Calculate remaining percentage
	if (current_voltage <= MIN_CHARGE_MV) {
		remaining_percentage = 0;
	} else {
		normalized_voltage = current_voltage - MIN_CHARGE_MV;
		voltage_range = MAX_CHARGE_MV - MIN_CHARGE_MV;

		if (normalized_voltage >= voltage_range) {
			remaining_percentage = 100;
		} else {
			remaining_percentage = (normalized_voltage * 100 / voltage_range);
		}
	}

	// Ensure the remaining percentage does not exceed 100%
	if (remaining_percentage > 100) {
		remaining_percentage = 100;
	}

	return remaining_percentage;
}

void micro_scaps_en(int en)
{
	uint8_t value;

	micro_read8(MICRO_STATUS_FLAGS, &value);
	if (en)
		value |= MICRO_STATUS_FLAGS_SCAPS_EN;
	micro_write8(MICRO_STATUS_FLAGS, &value);
}

static bool read_power_fail_status(void)
{
	return gpio_get_value(POWER_FAIL);
}

// Blocks until charge is above `block_pct` and power fail is cleared
uint8_t micro_scaps_block_pct(int block_pct)
{
	uint8_t cur_pct = 0;
	bool charge_ok, power_fail_clear;
	int counter = 0;

	assert(block_pct <= 100);
	
	gpio_request(POWER_FAIL, "POWER_FAIL");
	gpio_direction_input(POWER_FAIL);

	micro_scaps_en(1);

	printf("Charging supercaps, press ctrl+c to boot immediately\n");

	while (!ctrlc()) {
		cur_pct = micro_scaps_remaining_pct();
		power_fail_clear = !read_power_fail_status();
		charge_ok = (cur_pct >= block_pct);

		// Print status once per second
		if (counter % 10 == 0) {
			if (cur_pct == 0)
				printf("Supercaps below minimum operating charge, (Target: %d%%) | Power Fail: %s\n",
					block_pct, power_fail_clear ? "no" : "yes");
			else
			printf("Supercap Charge: %d%% (Target: %d%%) | Power Fail: %s\n",
			       cur_pct, block_pct, power_fail_clear ? "no" : "yes");
		}

		if (charge_ok && power_fail_clear) {
			break;
		}

		udelay(1000 * 100);
		counter++;
	}

	gpio_free(POWER_FAIL);

	return cur_pct;
}

int board_prep_linux(bootm_headers_t *images)
{
	const char *block_pct_str = env_get("silo_chrg_pct");
	long block_pct = 0;
	uint8_t charged_pct = 0;

	if (block_pct_str)
		block_pct = simple_strtol(block_pct_str, NULL, 10);

	if (!block_pct_str || block_pct < 0) {
		printf("Supercaps disabled\n");
		return 0;
	}

	if (block_pct == 0) {
		printf("Supercaps enabled\n");
		micro_scaps_en(1);
	} else {
		if (block_pct > 100)
			block_pct = 100;

		charged_pct = micro_scaps_block_pct(block_pct);
		printf("Supercaps charged to %d%%\n", charged_pct);
	}

	return 0;
}
