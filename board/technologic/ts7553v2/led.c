/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/gpio.h>
#include <common.h>
#include <status_led.h>

static unsigned int saved_state[4] = {STATUS_LED_OFF,
	STATUS_LED_OFF, STATUS_LED_OFF, STATUS_LED_OFF};

u32 const led_pads[] = {
	CONFIG_RED_LED, // Red
	CONFIG_GREEN_LED, // Green
};

void red_led_on(void)
{
	gpio_direction_output(led_pads[0], 0);
	saved_state[STATUS_LED_RED] = STATUS_LED_ON;
}

void red_led_off(void)
{
	gpio_direction_output(led_pads[0], 1);
	saved_state[STATUS_LED_RED] = STATUS_LED_OFF;
}

void green_led_on(void)
{
	gpio_direction_output(led_pads[1], 0);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_ON;
}

void green_led_off(void)
{
	gpio_direction_output(led_pads[1], 1);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_OFF;
}

void __led_init(led_id_t mask, int state)
{
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
	}
}

void __led_set(led_id_t mask, int state)
{
	switch(mask) {
	  case STATUS_LED_RED:
		if(STATUS_LED_ON == state) {
			red_led_on();
		} else {
			red_led_off();
		}
		break;
	  case STATUS_LED_GREEN:
		if(STATUS_LED_ON == state) {
			green_led_on();
		} else {
			green_led_off();
		}
		break;
	}
}
