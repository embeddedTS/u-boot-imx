/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-pins.h>
#include <status_led.h>
#include <asm/imx-common/iomux-v3.h>

#include "tsfpga.h"

#define MISC_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_UART1_CTS_B__GPIO1_IO18 | MUX_PAD_CTRL(MISC_PAD_CTRL), /* EN_RED_LED# */
	MX6_PAD_UART1_RTS_B__GPIO1_IO19 | MUX_PAD_CTRL(MISC_PAD_CTRL), /* EN_GRN_LED# */
};

#define EN_RED_LEDN		IMX_GPIO_NR(1, 18)
#define EN_GREEN_LEDN		IMX_GPIO_NR(1, 19)

static unsigned int saved_state[3] = {
	STATUS_LED_OFF,
	STATUS_LED_OFF,
	STATUS_LED_OFF
};
									  
void red_led_on(void)
{
	gpio_set_value(EN_RED_LEDN, 0);
	saved_state[STATUS_LED_RED] = STATUS_LED_ON;
}

void red_led_off(void)
{
	gpio_set_value(EN_RED_LEDN, 1);
	saved_state[STATUS_LED_RED] = STATUS_LED_OFF;
}

void blue_led_on(void)
{
	fpga_dio_dat_set(EN_BLUE_LED_PAD);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_ON;
}

void blue_led_off(void)
{
	fpga_dio_dat_clr(EN_BLUE_LED_PAD);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_OFF;
}

void green_led_on(void)
{
	gpio_set_value(EN_GREEN_LEDN, 0);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_ON;
}

void green_led_off(void)
{
	gpio_set_value(EN_GREEN_LEDN, 1);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_OFF;
}

void __led_init(led_id_t mask, int state)
{
	imx_iomux_v3_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));

	gpio_direction_output(EN_GREEN_LEDN, 1);
	gpio_direction_output(EN_RED_LEDN, 1);
	fpga_dio_dat_clr(EN_BLUE_LED_PAD);
	fpga_dio_oe_set(EN_BLUE_LED_PAD);

	__led_set(mask, state);
}

void __led_toggle(led_id_t mask)
{
	if (STATUS_LED_RED == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_RED])
			red_led_off();
		else
			red_led_on();

	} else if (STATUS_LED_GREEN == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_GREEN])
			green_led_off();
		else
			green_led_on();

	} else if (STATUS_LED_BLUE == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_BLUE])
			blue_led_off();
		else
			blue_led_on();

	}
}

void __led_set(led_id_t mask, int state)
{
	if (STATUS_LED_RED == mask) {
		if (STATUS_LED_ON == state)
			red_led_on();
		else
			red_led_off();

	} else if (STATUS_LED_GREEN == mask) {
		if (STATUS_LED_ON == state)
			green_led_on();
		else
			green_led_off();
	} else if (STATUS_LED_BLUE == mask) {
		if (STATUS_LED_ON == state)
			blue_led_on();
		else
			blue_led_off();
	}
}
