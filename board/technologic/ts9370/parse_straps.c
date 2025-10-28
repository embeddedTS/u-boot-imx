// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2019-2025 Technologic Systems dba embeddedTS
 */

#include <vsprintf.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch-imx9/imx93_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm-generic/gpio.h>
#include <linux/delay.h>

#include "parse_straps.h"

#define STRAP_PAD_PD_CTRL (PAD_CTL_PDE)

static const iomux_v3_cfg_t strap_pads[] = {
	MX93_PAD_GPIO_IO04__GPIO2_IO04 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
	MX93_PAD_GPIO_IO10__GPIO2_IO10 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
	MX93_PAD_GPIO_IO14__GPIO2_IO14 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
	MX93_PAD_GPIO_IO00__GPIO2_IO00 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
	MX93_PAD_GPIO_IO12__GPIO2_IO12 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
	MX93_PAD_GPIO_IO08__GPIO2_IO08 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
	MX93_PAD_GPIO_IO18__GPIO2_IO18 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
	MX93_PAD_GPIO_IO20__GPIO2_IO20 | MUX_PAD_CTRL(STRAP_PAD_PD_CTRL),
};

const char *get_board_version_str(void)
{
	static char model_str[24] = {0};

	snprintf(model_str, sizeof(model_str), "PROTO");
	return model_str;
}

int alloc_strap_gpio_desc(char *label, struct gpio_desc *desc)
{
	int ret;

	ret = dm_gpio_lookup_name(label, desc);
	if (ret) {
		printf("Could not find GPIO: \"%s\"\n", label);
		return ret;
	}

	ret = dm_gpio_request(desc, "strap");
	if (ret) {
		printf("Could not request GPIO: \"%s\"\n", label);
		return ret;
	}

	ret = dm_gpio_set_dir_flags(desc, GPIOD_IS_IN);
	if (ret) {
		printf("Could not set GPIO to input: \"%s\"\n", label);
		return ret;
	}

	return 0;
}

u16 read_bom_straps(void)
{
	static u16 cpu_straps;
	struct gpio_desc desc[8];
	static bool read;

	if (!read) {
		imx_iomux_v3_setup_multiple_pads(strap_pads, ARRAY_SIZE(strap_pads));

		alloc_strap_gpio_desc("GPIO1_04", &desc[7]); /* R131 */
		alloc_strap_gpio_desc("GPIO1_10", &desc[6]); /* R132 */
		alloc_strap_gpio_desc("GPIO1_14", &desc[5]); /* R133 */
		alloc_strap_gpio_desc("GPIO1_00", &desc[4]); /* R134 */
		alloc_strap_gpio_desc("GPIO1_12", &desc[3]); /* R127 */
		alloc_strap_gpio_desc("GPIO1_08", &desc[2]); /* R128 */
		alloc_strap_gpio_desc("GPIO1_18", &desc[1]); /* R129 */
		alloc_strap_gpio_desc("GPIO1_20", &desc[0]); /* R130 */

		cpu_straps = dm_gpio_get_values_as_int(desc, 8);

		for (int i = 0; i < 8; i++)
			dm_gpio_free(NULL, &desc[i]);

		read = 1;
	}
	return cpu_straps;
}
