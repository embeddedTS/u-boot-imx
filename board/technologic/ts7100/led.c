// SPDX-License-Identifier:	GPL-2.0+
/*
 * Copyright (C) 2020-2022 Technologic Systems dba embeddedTS
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-pins.h>
#include <asm/mach-imx/iomux-v3.h>
#include <status_led.h>

#define LED_PAD_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_UART1_CTS_B__GPIO1_IO18 | MUX_PAD_CTRL(LED_PAD_CTRL),
	MX6_PAD_UART1_RTS_B__GPIO1_IO19 | MUX_PAD_CTRL(LED_PAD_CTRL),
};

#define RED_LED 		IMX_GPIO_NR(1, 18)
#define GRN_LED 		IMX_GPIO_NR(1, 19)

void __led_init (led_id_t mask, int state)
{
	imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));

	if (CONFIG_LED_STATUS_BIT == mask) {
		gpio_request(RED_LED, "RED_LED");
		gpio_direction_output(RED_LED, !state);

	}
	if (CONFIG_LED_STATUS_BIT1) {
		gpio_request(GRN_LED, "GRN_LED");
		gpio_direction_output(GRN_LED, !state);
	}
}

void __led_set (led_id_t mask, int state)
{
	if (CONFIG_LED_STATUS_BIT == mask) {
		if(state)
			gpio_set_value(RED_LED, 0);
		else
			gpio_set_value(RED_LED, 1);
	}

	if (CONFIG_LED_STATUS_BIT1 == mask) {
		if(state)
			gpio_set_value(GRN_LED, 0);
		else
			gpio_set_value(GRN_LED, 1);
	}
}

/* Not implemented */
void __led_toggle (led_id_t mask) {}
void __led_blink(led_id_t mask, int freq) {}
