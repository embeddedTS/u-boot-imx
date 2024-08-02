// SPDX-License-Identifier:	GPL-2.0+
/*
 * Copyright (C) 2020-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <env.h>
#include <fpga.h>
#include <fsl_esdhc_imx.h>
#include <fuse.h>
#include <i2c.h>
#include <lattice.h>
#include <linux/sizes.h>
#include <linux/fb.h>
#include <miiphy.h>
#include <mmc.h>
#include <netdev.h>
#include <status_led.h>
#include <usb.h>

#include <configs/ts7180.h>
#include "tsfpga.h"

DECLARE_GLOBAL_DATA_PTR;

void ts7180_fpga_init(void);
int16_t silab_inb(uint16_t subadr);

#define U_BOOT_JMPN		IMX_GPIO_NR(3, 16)
#define PUSH_SW_CPUN		IMX_GPIO_NR(3, 18)
#define NO_CHRG_JMPN		IMX_GPIO_NR(3, 11)
#define OPT_ID_1		IMX_GPIO_NR(3, 27)
#define OPT_ID_4		IMX_GPIO_NR(3, 23)
#define OPT_ID_5		IMX_GPIO_NR(4, 19)
#define JTAG_FPGA_TDO		IMX_GPIO_NR(3, 24)
#define JTAG_FPGA_TDI		IMX_GPIO_NR(3, 3)
#define JTAG_FPGA_TMS		IMX_GPIO_NR(3, 2)
#define JTAG_FPGA_TCK		IMX_GPIO_NR(3, 1)
#define ETH_PHY_RESETN		IMX_GPIO_NR(3, 28)
#define PHY1_DUPLEX		IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX		IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2		IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2		IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2		IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2		IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE		IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE		IMX_GPIO_NR(2, 15)


#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define MDIO_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST | PAD_CTL_ODE)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

iomux_v3_cfg_t const fpga_jtag_pads[] = {
	/* JTAG_FPGA_TDO */
	MX6_PAD_LCD_DATA19__GPIO3_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* JTAG_FPGA_TDI */
	MX6_PAD_LCD_VSYNC__GPIO3_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* JTAG_FPGA_TMS */
	MX6_PAD_LCD_HSYNC__GPIO3_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* JTAG_FPGA_TCK */
	MX6_PAD_LCD_ENABLE__GPIO3_IO01 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* FPGA_RESET# */
	MX6_PAD_NAND_WP_B__GPIO4_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int is_mfg(void)
{
	return is_boot_from_usb();
}

extern int64_t silab_cmd(int argc, char *const argv[]);
extern int64_t silab_rev(void);

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

//void reset_cpu(ulong addr)
//{
//#ifndef CONFIG_SPL_BUILD
//	char * const rebootcmd[] = {"silabs", "wdog", "set", "1"};
//	silab_cmd(4, rebootcmd);
//#endif
//	while (1) {}
//}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

void fpga_late_init(void)
{
#if defined(HAVE_TSFPGA)
	int sdboot;
	int uboot;

	/* Onboard jumpers to boot to SD or break in u-boot */
	gpio_request(SD_BOOT_JMPN, "SD_BOOT_JMP#");
	gpio_request(U_BOOT_JMPN, "U_BOOT_JMP#");

	sdboot = fpga_gpio_input(SD_BOOT_JMPN);
	gpio_direction_input(U_BOOT_JMPN);
	uboot = gpio_get_value(U_BOOT_JMPN);
	if(sdboot)
		env_set("jpsdboot", "off");
	else
		env_set("jpsdboot", "on");

	if(uboot)
		env_set("jpuboot", "off");
	else
		env_set("jpuboot", "on");

	fpga_gpio_output(EN_USB_HOST_5V, 1);
	gpio_free(SD_BOOT_JMPN);
	gpio_free(U_BOOT_JMPN);

#endif
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

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
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_PAD_CTRL),

	MX6_PAD_GPIO1_IO06__ENET2_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),

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
	/* ETH_PHY_RESET */
	MX6_PAD_LCD_DATA23__GPIO3_IO28 | MUX_PAD_CTRL(ENET_PAD_CTRL)
};

int board_phy_config(struct phy_device *phydev)
{
	debug("** board_phy_config\n");
	/* Reset phy 1, phy 2 is reset by default */
	phydev->bus->write(phydev->bus, 0x1, MDIO_DEVAD_NONE, 0x0, 0x8000);
	/* Disable BCAST, select RMII */
	phydev->bus->write(phydev->bus, 0x1, MDIO_DEVAD_NONE, 0x16, 0x202);
	phydev->bus->write(phydev->bus, 0x2, MDIO_DEVAD_NONE, 0x16, 0x202);
	/* Enable 50MHZ Clock */
	phydev->bus->write(phydev->bus, 0x1, MDIO_DEVAD_NONE, 0x1f, 0x8180);
	phydev->bus->write(phydev->bus, 0x2, MDIO_DEVAD_NONE, 0x1f, 0x8180);

	return 0;
}
#endif /* CONFIG_FEC_MXC */

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

	/* Reset */
	gpio_request(ETH_PHY_RESETN, "ETH_PHY_RESET#");
	gpio_direction_output(ETH_PHY_RESETN, 0);
	gpio_direction_output(PHY1_DUPLEX, 0);
	gpio_direction_output(PHY2_DUPLEX, 0);
	gpio_direction_output(PHY1_PHYADDR2, 0);
	gpio_direction_output(PHY2_PHYADDR2, 0);
	gpio_direction_output(PHY1_CONFIG_2, 0);
	gpio_direction_output(PHY2_CONFIG_2, 0);
	gpio_direction_output(PHY1_ISOLATE, 0);
	gpio_direction_output(PHY2_ISOLATE, 0);
        
	/* PHYs need 10 ms first reset, USB hub only need 1 us */
	mdelay(15);
	gpio_direction_output(ETH_PHY_RESETN, 1);
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

/*
 * board_early_init_f (called iff CONFIG_BOARD_EARLY_INIT_F)
 */
int board_early_init_f(void)
{
	setup_iomux_uart();

	imx_iomux_v3_setup_multiple_pads(fpga_jtag_pads,
					 ARRAY_SIZE(fpga_jtag_pads));

	/* Keep as inputs to allow offboard programming */
	gpio_direction_input(JTAG_FPGA_TDI);
	gpio_direction_input(JTAG_FPGA_TCK);
	gpio_direction_input(JTAG_FPGA_TMS);
	gpio_direction_input(JTAG_FPGA_TDO);

	/* Enable LVDS clock output.
	 * Writing CCM_ANALOG_MISC1 to use output from 24M OSC */
	setbits_le32(0x020C8160, 0x412);

	fpga_reset();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_FEC_MXC
	reset_phy();
	setup_fec();
#endif

#if defined(CONFIG_FPGA)
	/* Set up FPGA subsystem for JTAGing, i.e., soft loading */
	ts7180_fpga_init();
#endif	

	return 0;
}

int board_late_init(void)
{
	int jpr;
	int jpsw_n;
	uint8_t opts = 0;

	gpio_request(SD_BOOT_JMPN, "SD_BOOT_JMP#");
	gpio_request(PUSH_SW_CPUN, "PUSH_SW_CPU#");
	gpio_request(U_BOOT_JMPN, "U_BOOT_JMP#");
	gpio_request(NO_CHRG_JMPN, "NO_CHRG_JMP#");
	gpio_request(OPT_ID_1, "ID1");
	gpio_request(OPT_ID_4, "ID4");
	gpio_request(OPT_ID_5, "ID5");

	gpio_direction_input(SD_BOOT_JMPN);
	gpio_direction_input(PUSH_SW_CPUN);
	gpio_direction_input(U_BOOT_JMPN);
	gpio_direction_input(NO_CHRG_JMPN);
	gpio_direction_input(OPT_ID_1);
	gpio_direction_input(OPT_ID_4);
	gpio_direction_input(OPT_ID_5);

	jpsw_n = gpio_get_value(PUSH_SW_CPUN);
	if (!jpsw_n) env_set("jpsw", "on");
	else env_set("jpsw", "off");

	/* Onboard jumpers to boot to SD or break in u-boot */
	jpr = gpio_get_value(NO_CHRG_JMPN);
	if (jpr) env_set("jpnochrg", "off");
	else env_set("jpnochrg", "on");

	jpr = gpio_get_value(SD_BOOT_JMPN);
	if (jpr) env_set("jpsdboot", "off");
	else env_set("jpsdboot", "on");

	jpr = gpio_get_value(U_BOOT_JMPN);
	if (!jpr) env_set("jpuboot", "on");
	else env_set("jpuboot", "off");

	opts |= (gpio_get_value(OPT_ID_5) << 4); // R30 (extra CPU strap)
	opts |= (gpio_get_value(OPT_ID_4) << 3); // R38

	if (fpga_gpio_input(48)) /* FPGA pad P7, R37 on TS-7180 schematic) */
		opts |= (1 << 2);

	if (fpga_gpio_input(47)) /* FPGA pad N7, R36 on TS-7180 schematic) */
		opts |= (1 << 1);

	opts |= (gpio_get_value(OPT_ID_1) << 0); // R31

	env_set_hex("opts", (~opts & 0x1F));

	if (is_mfg()) {
		env_set("bootcmd", "mfg");
		env_set("bootdelay", "1");
	} else if (!gpio_get_value(U_BOOT_JMPN)) {
		/* U-Boot jumper -> no autoboot */
		env_set("bootdelay", "-1");
	}

	gpio_free(SD_BOOT_JMPN);
	gpio_free(PUSH_SW_CPUN);
	gpio_free(U_BOOT_JMPN);
	gpio_free(NO_CHRG_JMPN);
	gpio_free(OPT_ID_1);
	gpio_free(OPT_ID_4);
	gpio_free(OPT_ID_5);

	fpga_late_init();
	red_led_on();
	green_led_off();
	blue_led_off();
	yellow_led_off();

	fpga_gpio_output(EN_SD_POWER, 0);
	udelay(500);
	fpga_gpio_output(EN_SD_POWER, 1);

	return 0;
}

#if defined(CONFIG_FPGA)

static void ts7180_fpga_jtag_init(void)
{
	gpio_request(JTAG_FPGA_TDI, "FPGA_TDI");
	gpio_request(JTAG_FPGA_TCK, "FPGA_TCK");
	gpio_request(JTAG_FPGA_TMS, "FPGA_TMS");
	gpio_request(JTAG_FPGA_TDO, "FPGA_TDO");

	gpio_direction_output(JTAG_FPGA_TDI, 1);
	gpio_direction_output(JTAG_FPGA_TCK, 1);
	gpio_direction_output(JTAG_FPGA_TMS, 1);
	gpio_direction_input(JTAG_FPGA_TDO);
}

static void ts7180_fpga_done(void)
{
	gpio_direction_input(JTAG_FPGA_TDI);
	gpio_direction_input(JTAG_FPGA_TCK);
	gpio_direction_input(JTAG_FPGA_TMS);
	gpio_direction_input(JTAG_FPGA_TDO);

	gpio_free(JTAG_FPGA_TDI);
	gpio_free(JTAG_FPGA_TCK);
	gpio_free(JTAG_FPGA_TMS);
	gpio_free(JTAG_FPGA_TDO);

	fpga_reset();

	/* During FPGA programming several important pins will
	 * have been tristated.  Put it back to normal */
	fpga_gpio_output(EN_SD_POWER, 1);

	fpga_late_init();
	red_led_on();
	green_led_off();
}

static void ts7180_fpga_tdi(int value)
{
	printf("Here: %s, %s; %d\n", __FILE__, __FUNCTION__, __LINE__);
	gpio_set_value(JTAG_FPGA_TDI, value);
}

static void ts7180_fpga_tms(int value)
{
	gpio_set_value(JTAG_FPGA_TMS, value);
}

static void ts7180_fpga_tck(int value)
{
	gpio_set_value(JTAG_FPGA_TCK, value);
}

static int ts7180_fpga_tdo(void)
{
	return gpio_get_value(JTAG_FPGA_TDO);
}

lattice_board_specific_func ts7180_fpga_fns = {
	ts7180_fpga_jtag_init,
	ts7180_fpga_tdi,
	ts7180_fpga_tms,
	ts7180_fpga_tck,
	ts7180_fpga_tdo,
	ts7180_fpga_done
};

Lattice_desc ts7180_fpga = {
	Lattice_XP2,
	lattice_jtag_mode,
	589012,
	(void *) &ts7180_fpga_fns,
	NULL,
	0,
	"machxo_2_cb132"
};

void ts7180_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts7180_fpga);
}

#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

iomux_v3_cfg_t const usb_otg1_pads[] = {
	MX6_PAD_GPIO1_IO04__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO00__ANATOP_OTG1_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
};

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return USB_INIT_DEVICE;
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
					 ARRAY_SIZE(usb_otg1_pads));

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);
	return 0;
}
#endif

/* U-Boot calls checkboard() to display information about the board it is running on */
int checkboard(void)
{
	int fpgarev;

	fpgarev = fpga_get_rev();
	if(fpgarev < 0)
		printf("FPGA I2C communication failed: %d\n", fpgarev);
	else
		printf("FPGA: Rev %d\n", fpgarev);

	printf("Wizard: Rev %d\n", silab_inb(2048));
	return 0;
}
