#include <env.h>
#include <stdio.h>
// bbdetect_dm_simplified.c
#include <dm.h>
#include <asm/gpio.h>
#include <env.h>
#include <stdio.h>
#include <stdbool.h>

static int setup_bbdetect_io(const char *name, bool is_out, struct gpio_desc *d)
{
    int ret;

    ret = dm_gpio_lookup_name(name, d);
    if (ret) {
        printf("GPIO not found: %s\n", name);
        return ret;
    }

    ret = dm_gpio_request(d, "bbdetect");
    if (ret) {
        printf("GPIO request failed: %s\n", name);
        return ret;
    }

    ret = dm_gpio_set_dir_flags(d, is_out ? GPIOD_IS_OUT : GPIOD_IS_IN);
    if (ret) {
        printf("GPIO dir set failed (%s): %s\n", is_out ? "OUT" : "IN", name);
        dm_gpio_free(NULL, d);
        return ret;
    }

    return 0;
}

int do_bbdetect(void)
{
    struct gpio_desc s0;  // EN_RED_LED#   -> GPIO4_01 (OUT)
    struct gpio_desc s1;  // EN_GREEN_LED# -> GPIO4_00 (OUT)
    struct gpio_desc s2;  // DIO_52        -> GPIO6_19 (OUT)
    struct gpio_desc din; // DIO_84        -> GPIO7_19 (IN)
    uint8_t id = 0;
    int ret, i;

    ret = setup_bbdetect_io("FPGA_GPIO0_01", true,  &s0);
	if (ret)
		return ret;
    ret = setup_bbdetect_io("FPGA_GPIO0_00", true,  &s1);
	if (ret)
		goto out;
    ret = setup_bbdetect_io("FPGA_GPIO2_19", true,  &s2);
	if (ret)
		goto out;
    ret = setup_bbdetect_io("FPGA_GPIO3_19", false, &din);
	if (ret)
		goto out;

    for (i = 0; i < 8; i++) {
        dm_gpio_set_value(&s0, (i & 1) ? 1 : 0);
        dm_gpio_set_value(&s1, (i & 2) ? 1 : 0);
        dm_gpio_set_value(&s2, (i & 4) ? 1 : 0);

        id >>= 1;
        if (dm_gpio_get_value(&din))
            id |= 0x80;
    }

    printf("Baseboard ID: 0x%X\n", id & ~0xC0);
    printf("Baseboard Rev: %d\n", (id & 0xC0) >> 6);

    env_set_hex("baseboard_id",  (unsigned long)(id & ~0xC0));
    env_set_hex("baseboard_rev", (unsigned long)((id & 0xC0) >> 6));

out:
    dm_gpio_free(NULL, &din);
    dm_gpio_free(NULL, &s2);
    dm_gpio_free(NULL, &s1);
    dm_gpio_free(NULL, &s0);
    return ret;
}
