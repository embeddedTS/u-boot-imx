#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <extension_board.h>
#include <dm/uclass.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm-generic/gpio.h>
#include <i2c.h>
#include <env.h>

/*
 * construct_baseboard_overlay_str
 *
 * This function allocates and initializes a struct extension.
 * It returns a pointer to the allocated struct if successful,
 * or NULL if the environment does not have the required variables.
 *
 * The caller is responsible for freeing the returned struct.
 */
struct extension *create_extension(char *suffix)
{
	uint8_t baseboard_id;
	struct extension *ext;

	baseboard_id = env_get_hex("baseboard_id", 0);
	if (!baseboard_id)
		return NULL;

	ext = calloc(1, sizeof(struct extension));
	if (!ext)
		return NULL;

	snprintf(ext->owner, sizeof(ext->owner), "embeddedTS");

	if (!suffix)
		snprintf(ext->overlay, sizeof(ext->overlay),
			 "imx93-ts4300-%02x.dtbo", baseboard_id);
	else
		snprintf(ext->overlay, sizeof(ext->overlay),
			 "imx93-ts4300-%02x-%s.dtbo", baseboard_id, suffix);

	return ext;
}

/* Returns 1 if the mipi2dp is present */
int ts8551_mipi2dp_present(void)
{
	static struct udevice *chip;
	struct udevice *bus;
	struct gpio_desc reset = {0}; /* CN1_096 / DP_RESET -> GPIO6_17 */
	uint32_t value;
	int ret = 0;
	int err;

	/* Drive DP_RESET with DM GPIO: pulse high then low */
	err = dm_gpio_lookup_name("FPGA_GPIO2_17", &reset);
	if (err) {
		printf("%s: GPIO lookup failed\n", __func__);
		return 0;
	}
	err = dm_gpio_request(&reset, "ts8551");
	if (err) {
		printf("%s: GPIO request failed\n", __func__);
		return 0;
	}
	err = dm_gpio_set_dir_flags(&reset, GPIOD_IS_OUT);
	if (err) {
		printf("%s: GPIO dir set failed\n", __func__);
		goto out;
	}

	dm_gpio_set_value(&reset, 1);
	udelay(2);   /* tRSTON = 2us */
	dm_gpio_set_value(&reset, 0);
	mdelay(1);   /* tCORERDY = 1ms */

	err = uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	if (err) {
		printf("%s: Failed to get i2c device\n", __FUNCTION__);
		goto out;
	}

	err = i2c_get_chip(bus, 0x0f, 0, &chip);
	if (err) {
		printf("%s: Failed to get i2c chip\n", __FUNCTION__);
		goto out;
	}

	i2c_set_chip_offset_len(chip, 2);
	if (err) {
		printf("%s: Failed to get set offset length\n", __FUNCTION__);
		goto out;
	}

	/* I'd expect this to fail if the i2c device does not ack, but currently
	 * it returns successfully with all 0s. If this is fixed in the future
	 * we can pass/fail just off i2c responding, otherwise we rely on the
	 * expected id value from this chip. */
	err = dm_i2c_read(chip, 0x500, (uint8_t *)&value, sizeof(value));
	if (err)
		goto out;

	if (value == 0x6603)
		ret = 1;

out:
	dm_gpio_free(NULL, &reset);

	/* Any errors mean the card is not present, return 0 not errs */
	return ret;
}

/*
 * extension_board_scan
 *
 * This function returns "the number of extension boards found", i.e.,
 * 1 if a recognized/valid extension board was added to the list of
 * extensions, 0 if not.
 *
 * Call do_bbdetect() before calling this function. Doing so ensures
 * the needed environment variables are populated.
 */
int extension_board_scan(struct list_head *extension_list)
{
	struct extension *baseboard;
	uint8_t baseboard_rev;
	uint8_t baseboard_id;
	int extensions = 1; /* will always be at least 1 baseboard extension */

	baseboard_id = env_get_hex("baseboard_id", 0);
	if (!baseboard_id)
		return 0;
	baseboard_rev = env_get_hex("baseboard_rev", 0);

	/* Add baseboard extension */
	baseboard = create_extension(0);
	if (!baseboard)
		return 0;
	list_add_tail(&baseboard->list, extension_list);

	snprintf(baseboard->version, sizeof(baseboard->version),
		 "%d", baseboard_rev);

	switch (baseboard_id) {
	case 0x16:
		snprintf(baseboard->name, sizeof(baseboard->name), "TS-8551");

		/* Check for TS-RD-MIPI2DP card by toggling CN1_096 to take it out of
		 * reset, and looking for 0x0f to ack.
		 */
		if (ts8551_mipi2dp_present()) {
			struct extension *dc = create_extension("mipi2dp");

			snprintf(dc->name, sizeof(dc->name), "TS-RD-MIPI2DP");
			list_add_tail(&dc->list, extension_list);
			extensions++;
		}

		break;
	default:
		snprintf(baseboard->name, sizeof(baseboard->name),
			 "Custom Baseboard ID=0x%02x", baseboard_id);
		break;
	}

	return extensions;
}