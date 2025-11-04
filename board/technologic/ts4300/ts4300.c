// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 * Copyright 2023-2024 Technologic Systems, Inc. (dba embeddedTS)
 */

#include <env.h>
#include <efi_loader.h>
#include <init.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/global_data.h>
#include <asm/arch-imx9/ccm_regs.h>
#include <asm/arch-imx9/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch-imx9/imx93_pins.h>
#include <asm/arch/clock.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <usb.h>
#include <dwc3-uboot.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <led.h>

#include "parse_straps.h"

#include <dm/root.h>
#include "../ts-common/fpga_bootloader.h"
#include "../ts-common/ts-macs.h"
#include "../ts-common/wizard.h"

extern void do_bbdetect(void);

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_FSEL2)
#define FPGA_PAD_CTRL	(PAD_CTL_DSE(6) | PAD_CTL_ODE | PAD_CTL_PUE)

static const iomux_v3_cfg_t uart_pads[] = {
	MX93_PAD_UART1_RXD__LPUART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX93_PAD_UART1_TXD__LPUART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static int setup_eqos(void);
static const iomux_v3_cfg_t fpga_pads[] = {
	MX93_PAD_GPIO_IO02__GPIO2_IO02 | MUX_PAD_CTRL(FPGA_PAD_CTRL), // DEV_CLRN / GPIO_02
	MX93_PAD_GPIO_IO11__GPIO2_IO11 | MUX_PAD_CTRL(FPGA_PAD_CTRL), // NSTATUS / GPIO_11
	MX93_PAD_GPIO_IO10__GPIO2_IO10 | MUX_PAD_CTRL(FPGA_PAD_CTRL), // CONF_DONE / GPIO_10
};

#if CONFIG_IS_ENABLED(EFI_HAVE_CAPSULE_SUPPORT)
#define IMX_BOOT_IMAGE_GUID \
	EFI_GUID(0xbc550d86, 0xda26, 0x4b70, 0xac, 0x05, \
		 0x2a, 0x44, 0x8e, 0xda, 0x6f, 0x21)

struct efi_fw_image fw_images[] = {
	{
		.image_type_id = IMX_BOOT_IMAGE_GUID,
		.fw_name = u"IMX93-TS-4300",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=flash-bin raw 0 0x2000 mmcpart 1",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

#endif /* EFI_HAVE_CAPSULE_SUPPORT */

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
	init_uart_clk(LPUART1_CLK_ROOT);

	print_fpga_version();

	return 0;
}

static int setup_fec(void)
{
	return set_clk_enet(ENET_125MHZ);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

static int setup_eqos(void)
{
	struct blk_ctrl_wakeupmix_regs *bctrl =
		(struct blk_ctrl_wakeupmix_regs *)BLK_CTRL_WAKEUPMIX_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(fpga_pads, ARRAY_SIZE(fpga_pads));
	/* set INTF as RGMII, enable RGMII TXC clock */
	clrsetbits_le32(&bctrl->eqos_gpr,
			BCTRL_GPR_ENET_QOS_INTF_MODE_MASK,
			BCTRL_GPR_ENET_QOS_INTF_SEL_RGMII | BCTRL_GPR_ENET_QOS_CLK_GEN_EN);

	return 0;
}

int board_init(void)
{
	if (CONFIG_IS_ENABLED(FEC_MXC))
		setup_fec();

	if (CONFIG_IS_ENABLED(DWC_ETH_QOS))
		setup_eqos();

	return 0;
}

int board_late_init(void)
{
	struct udevice *led;
	u32 bom_straps;

	/* TS-4300 has 2 onboard ethernet, and reserves 1 mac for some carrier
	 * boards that have a USB ethernet */
	setup_mac_addresses(3);

	if (IS_ENABLED(CONFIG_ENV_IS_IN_MMC))
		board_late_mmc_env_init();

	if (IS_ENABLED(CONFIG_AHAB_BOOT))
		env_set("sec_boot", "yes");
	else
		env_set("sec_boot", "no");

	bom_straps = read_bom_straps();
	env_set_hex("bom_straps", bom_straps);

	if (IS_ENABLED(CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG)) {
		env_set("board_name", "TS-9370");
		env_set("board_rev", get_board_version_str());
	}

	do_bbdetect();

	if (!led_get_by_label("red:status", &led))
		led_set_state(led, LEDST_ON);

	if (!led_get_by_label("green:power", &led))
		led_set_state(led, LEDST_OFF);

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT
#ifdef CONFIG_ANDROID_RECOVERY
int is_recovery_key_pressing(void)
{
	return 0;
}
#endif /*CONFIG_ANDROID_RECOVERY*/
#endif /*CONFIG_FSL_FASTBOOT*/

int fdt_update_straps(void *fdt)
{
	u32 bom_options;
	int chosen_node;
	int ret;

	bom_options = (u32)read_bom_straps();

	chosen_node = fdt_path_offset(fdt, "/chosen");
	if (chosen_node < 0) {
		printf("Failed to find /chosen node: %d\n", chosen_node);
		return -1;
	}
	ret = fdt_setprop(fdt, chosen_node, "bom-options", &bom_options, sizeof(bom_options));
	if (ret < 0) {
		printf("Failed to set property bom-straps: %d\n", ret);
		return -1;
	}
	return 0;
}

int ft_board_setup(void *fdt, struct bd_info *bd)
{
	int ret;

	ret = fdt_update_straps(fdt);
	if (ret)
		return ret;

	return 0;
}
