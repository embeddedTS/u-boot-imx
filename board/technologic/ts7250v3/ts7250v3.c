// SPDX-License-Identifier:	GPL-2.0+
/*
 * Copyright (C) 2020-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/video.h>
#include <asm/io.h>
#include <common.h>
#include <env.h>
#include <i2c.h>
#include <miiphy.h>
#include <lattice.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include <asm/spl.h>
#include "tsfpga.h"

DECLARE_GLOBAL_DATA_PTR;

#define PHY1_DUPLEX 		IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX 		IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2 		IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2		IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2 		IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2		IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE		IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE		IMX_GPIO_NR(2, 15)
#define DETECT_94120		IMX_GPIO_NR(4, 1)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                  \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

int is_mfg(void)
{
	return is_boot_from_usb();
}

extern int64_t silab_cmd(int argc, char *const argv[]);

int wdog_en = 0;
void hw_watchdog_init(void)
{
#ifndef CONFIG_SPL_BUILD
	char * const checkflag[] = {"silabs", "wdog"};
	wdog_en = 1;
	wdog_en = (u8)silab_cmd(2, checkflag);
#endif
}

void hw_watchdog_reset(void)
{
#ifndef CONFIG_SPL_BUILD
	char * const feed[] = {"silabs", "wdog", "feed"};
	static ulong lastfeed;

	if(wdog_en != 1) return;

	if(get_timer(lastfeed) > 1000) {
		silab_cmd(3, feed);
		lastfeed = get_timer(0);
	}
#endif
}

void reset_cpu(ulong addr)
{
#ifndef CONFIG_SPL_BUILD
	char * const rebootcmd[] = {"silabs", "reboot"};
	silab_cmd(2, rebootcmd);
#endif
}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else {
		if(is_mfg())
			return usb_phy_mode(port);
		return USB_INIT_HOST;
	}
}
#endif

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const fec_enet_pads[] = {
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),

	MX6_PAD_ENET2_RX_DATA0__ENET2_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__ENET2_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__ENET2_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__ENET2_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),

	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const fec_strap_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* DUPLEX */
	MX6_PAD_ENET1_RX_DATA0__GPIO2_IO00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* DUPLEX */
	MX6_PAD_ENET2_RX_DATA0__GPIO2_IO08 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* PHYADDR2 */
	MX6_PAD_ENET1_RX_DATA1__GPIO2_IO01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* PHYADDR2 */
	MX6_PAD_ENET2_RX_DATA1__GPIO2_IO09 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* CONFIG_2 */
	MX6_PAD_ENET1_RX_EN__GPIO2_IO02 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* CONFIG_2 */
	MX6_PAD_ENET2_RX_EN__GPIO2_IO10 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* Isolate */
	MX6_PAD_ENET1_RX_ER__GPIO2_IO07 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* Isolate */
	MX6_PAD_ENET2_RX_ER__GPIO2_IO15 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

void reset_phy(void)
{
	/* Set pins to strapping GPIO modes */
	imx_iomux_v3_setup_multiple_pads(fec_strap_pads,
					 ARRAY_SIZE(fec_strap_pads));

	gpio_request(PHY1_DUPLEX, "PHY1_DUPLEX");
	gpio_request(PHY2_DUPLEX, "PHY2_DUPLEX");
	gpio_request(PHY1_PHYADDR2, "PHY1_PHYADDR2");
	gpio_request(PHY2_PHYADDR2, "PHY2_PHYADDR2");
	gpio_request(PHY1_CONFIG_2, "PHY1_CONFIG_2");
	gpio_request(PHY2_CONFIG_2, "PHY2_CONFIG_2");
	gpio_request(PHY1_ISOLATE, "PHY1_ISOLATE");
	gpio_request(PHY2_ISOLATE, "PHY2_ISOLATE");

	fpga_dio1_dat_clr(BANK1_PHY_RESETN);
	fpga_dio1_oe_set(BANK1_PHY_RESETN);
	gpio_direction_output(PHY1_DUPLEX, 0);
	gpio_direction_output(PHY2_DUPLEX, 0);
	gpio_direction_output(PHY1_PHYADDR2, 0);
	gpio_direction_output(PHY2_PHYADDR2, 0);
	gpio_direction_output(PHY1_CONFIG_2, 0);
	gpio_direction_output(PHY2_CONFIG_2, 0);
	gpio_direction_output(PHY1_ISOLATE, 0);
	gpio_direction_output(PHY2_ISOLATE, 0);

	udelay(1);
	fpga_dio1_dat_set(BANK1_PHY_RESETN);
	mdelay(10);

	gpio_free(PHY1_DUPLEX);
	gpio_free(PHY2_DUPLEX);
	gpio_free(PHY1_PHYADDR2);
	gpio_free(PHY2_PHYADDR2);
	gpio_free(PHY1_CONFIG_2);
	gpio_free(PHY2_CONFIG_2);
	gpio_free(PHY1_ISOLATE);
	gpio_free(PHY2_ISOLATE);

	/* Set pins to enet modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_pads,
					 ARRAY_SIZE(fec_enet_pads));
}

static int setup_fec(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17].
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);

	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
			IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);

	ret = enable_fec_anatop_clock(0, ENET_50MHZ);
	ret |= enable_fec_anatop_clock(1, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* Soft reset */
	phydev->bus->write(phydev->bus, phydev->addr, MDIO_DEVAD_NONE, 0x0, 0x8000);
	/* Disable BCAST, select RMII */
	phydev->bus->write(phydev->bus, phydev->addr, MDIO_DEVAD_NONE, 0x16, 0x202);
	/* Enable 50MHZ Clock */
	phydev->bus->write(phydev->bus, phydev->addr, MDIO_DEVAD_NONE, 0x1f, 0x8180);
	return 0;
}
#endif

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	reset_phy();
	setup_fec();
#endif

	//hw_watchdog_init(0);

	return 0;
}

int board_late_init(void)
{
	uint32_t fpga_rev = readl(FPGA_REV);
	uint32_t opts = readl(FPGA_STRAPS) & 0xF;

	env_set_hex("opts", opts);
	env_set_hex("fpga_rev", fpga_rev & 0x7fffffff);

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "TS-7250-V3");
	env_set("board_rev", "A");
#endif

	if(is_mfg()) {
		env_set("bootcmd", "mfg");
		env_set("bootdelay", "1");
	}

	return 0;
}

int checkboard(void)
{
	uint32_t fpga_rev = readl(FPGA_REV);
	uint32_t fpga_hash = readl(FPGA_HASH);

	printf("Board: TS-7250-V3 REV A\n");
	printf("FPGA:  Rev %d ", fpga_rev & 0x7fffffff);

	if(fpga_rev & (1 << 31))
		printf("(%x-dirty)\n", fpga_hash);
	else
		printf("(%x)\n", fpga_hash);

	return 0;
}
