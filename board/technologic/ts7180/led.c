// SPDX-License-Identifier:	GPL-2.0+
/*
 * Copyright (C) 2020-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/iomux-v3.h>
#include <status_led.h>

#define LED_PAD_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_CSI_DATA02__GPIO4_IO23 | MUX_PAD_CTRL(LED_PAD_CTRL),
	MX6_PAD_CSI_DATA03__GPIO4_IO24 | MUX_PAD_CTRL(LED_PAD_CTRL),
	MX6_PAD_CSI_DATA04__GPIO4_IO25 | MUX_PAD_CTRL(LED_PAD_CTRL),
	MX6_PAD_CSI_DATA05__GPIO4_IO26 | MUX_PAD_CTRL(LED_PAD_CTRL),   
};

#define EN_YEL_LEDN	IMX_GPIO_NR(4, 23)
#define EN_RED_LEDN	IMX_GPIO_NR(4, 24)
#define EN_GREEN_LEDN	IMX_GPIO_NR(4, 25)
#define EN_BLUE_LED	IMX_GPIO_NR(4, 26)

void __led_init (led_id_t mask, int state)
{
	imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));

	if (CONFIG_LED_STATUS_BIT == mask) {
		gpio_request(EN_RED_LEDN, "EN_RED_LEDN");
		gpio_direction_output(EN_RED_LEDN, !state);
	}
	if (CONFIG_LED_STATUS_BIT1) {
		gpio_request(EN_GREEN_LEDN, "EN_GREEN_LEDN");
		gpio_direction_output(EN_GREEN_LEDN, !state);
	}
	if (CONFIG_LED_STATUS_BIT2) {
		gpio_request(EN_YEL_LEDN, "EN_YEL_LEDN");
		gpio_direction_output(EN_YEL_LEDN, !state);
	}
	if (CONFIG_LED_STATUS_BIT3) {
		gpio_request(EN_BLUE_LED, "EN_BLUE_LED");
		gpio_direction_output(EN_BLUE_LED, !state);
	}
}

void __led_set (led_id_t mask, int state)
{
	if (CONFIG_LED_STATUS_BIT == mask) {
		if (state)
			gpio_set_value(EN_RED_LEDN, 0);
		else
			gpio_set_value(EN_RED_LEDN, 1);
	}

	if (CONFIG_LED_STATUS_BIT1 == mask) {
		if (state)
			gpio_set_value(EN_GREEN_LEDN, 0);
		else
			gpio_set_value(EN_GREEN_LEDN, 1);
	}

	if (CONFIG_LED_STATUS_BIT2 == mask) {
		if (state)
			gpio_set_value(EN_YEL_LEDN, 0);
		else
			gpio_set_value(EN_YEL_LEDN, 1);
	}

	if (CONFIG_LED_STATUS_BIT3 == mask) {
		if (state)
			gpio_set_value(EN_BLUE_LED, 1);
		else
			gpio_set_value(EN_BLUE_LED, 0);
	}
}

/* Not implemented */
void __led_toggle (led_id_t mask) {}
void __led_blink(led_id_t mask, int freq) {}
