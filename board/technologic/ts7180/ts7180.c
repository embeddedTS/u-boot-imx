/*
 * Copyright (C) 2016 Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fpga.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <lattice.h>
#include <linux/sizes.h>
#include <linux/fb.h>
#include <miiphy.h>
#include <mmc.h>
#include <mxsfb.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-fsl.h>
#include "fpga.h"
#include "parse_strap.h"


DECLARE_GLOBAL_DATA_PTR;

#define FPGA_RESETN		IMX_GPIO_NR(4, 11)
#define U_BOOT_JMPN		IMX_GPIO_NR(3, 16)
#define PUSH_SW_CPUN		IMX_GPIO_NR(3, 18)
#define NO_CHRG_JMPN		IMX_GPIO_NR(3, 11)
#define OPT_ID_1                IMX_GPIO_NR(3, 27)
#define OPT_ID_4                IMX_GPIO_NR(3, 23)   // Randy added this one
#define OPT_ID_5                IMX_GPIO_NR(4, 19)   // Randy added this one
#define JTAG_FPGA_TDO		IMX_GPIO_NR(3, 24)
#define JTAG_FPGA_TDI		IMX_GPIO_NR(3, 3)
#define JTAG_FPGA_TMS		IMX_GPIO_NR(3, 2)
#define JTAG_FPGA_TCK		IMX_GPIO_NR(3, 1)
#define EN_ETH_PHY_PWR 		IMX_GPIO_NR(1, 10)
#define PHY1_DUPLEX 		IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX 		IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2 		IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2		IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2 		IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2		IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE		IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE		IMX_GPIO_NR(2, 15)


#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL_WP (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

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

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define MISC_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP | PAD_CTL_SPEED_MED | PAD_CTL_HYS)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/*TODO: Verify that this implemtation of SPI_PAD is needed
#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST | PAD_CTL_PUE |\
*/

/* I2C1 for Silabs */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO02__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO02__GPIO1_IO02 | PC,
		.gp = IMX_GPIO_NR(1, 2),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO03__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO03__GPIO1_IO03 | PC,
		.gp = IMX_GPIO_NR(1, 3),
	},
};

/* I2C3 for FPGA/offbd */
struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode = MX6_PAD_LCD_DATA01__I2C3_SCL | PC,
		.gpio_mode = MX6_PAD_LCD_DATA01__GPIO3_IO06 | PC,
		.gp = IMX_GPIO_NR(3, 6),
	},
	.sda = {
		.i2c_mode = MX6_PAD_LCD_DATA00__I2C3_SDA | PC,
		.gpio_mode = MX6_PAD_LCD_DATA00__GPIO3_IO05 | PC,
		.gp = IMX_GPIO_NR(3, 5),
	},
};

int dram_init(void)
{
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;

	return 0;
}

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

void fpga_mmc_init(void)
{
	fpga_gpio_output(EN_SD_POWER, 1);
}

void fpga_late_init(void)
{
	int sdboot;
	int uboot;

	/* Onboard jumpers to boot to SD or break in u-boot */
	sdboot = fpga_gpio_input(SD_BOOT_JMPN);
	gpio_direction_input(U_BOOT_JMPN);
	uboot = gpio_get_value(U_BOOT_JMPN);
	if(sdboot)
		setenv("jpsdboot", "off");
	else
		setenv("jpsdboot", "on");

	if(uboot)
		setenv("jpuboot", "off");
	else
		setenv("jpuboot", "on");

	fpga_gpio_output(EN_USB_HOST_5V, 1);
}

#if defined(CONFIG_FPGA)

static void ts7180_fpga_jtag_init(void)
{
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

	/* During FPGA programming several important pins will
	 * have been tristated.  Put it back to normal */
	fpga_mmc_init();
	fpga_late_init();
	red_led_on();
	green_led_off();
}

static void ts7180_fpga_tdi(int value)
{
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

int ts7180_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts7180_fpga);

	return 0;
}

#endif

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const misc_pads[] = {
	MX6_PAD_LCD_DATA11__GPIO3_IO16 | MUX_PAD_CTRL(MISC_PAD_CTRL), /* U_BOOT_JMP# */
	MX6_PAD_LCD_DATA13__GPIO3_IO18 | MUX_PAD_CTRL(MISC_PAD_CTRL), /* PUSH_SW_CPU# */
	MX6_PAD_LCD_DATA06__GPIO3_IO11 | MUX_PAD_CTRL(MISC_PAD_CTRL), /* NO_CHRG_JMP# */

	/* note: options 2 and 3 are connected to the fpga */
        MX6_PAD_CSI_VSYNC__GPIO4_IO19 | MUX_PAD_CTRL(MISC_PAD_CTRL),  /* Option ID5 */
	MX6_PAD_LCD_DATA18__GPIO3_IO23 | MUX_PAD_CTRL(MISC_PAD_CTRL), /* Option ID4 */
	MX6_PAD_LCD_DATA22__GPIO3_IO27 | MUX_PAD_CTRL(MISC_PAD_CTRL), /* Option ID1 */
};

static iomux_v3_cfg_t const usdhc1_sd_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_CSI_DATA07__USDHC1_VSELECT | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_emmc_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC2_BASE_ADDR, 0, 4},
};


int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1 (SD/WIFI)
	 * mmc1                    USDHC2 (eMMC)
	 */
	fpga_mmc_init();

	imx_iomux_v3_setup_multiple_pads(
		usdhc1_sd_pads, ARRAY_SIZE(usdhc1_sd_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);

	imx_iomux_v3_setup_multiple_pads(
		usdhc2_emmc_pads, ARRAY_SIZE(usdhc2_emmc_pads));
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[0]))
		printf("Warning: failed to initialize sd dev\n");

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[1]))
		printf("Warning: failed to initialize emmc dev\n");

	return 0;
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

#if (CONFIG_FEC_ENET_DEV == 0)
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
#else
	MX6_PAD_GPIO1_IO06__ENET2_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
#endif

};

static iomux_v3_cfg_t const fec_enet_pads1[] = {
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
	/* EN_ETH_PHY_PWR */
	MX6_PAD_JTAG_MOD__GPIO1_IO10 | MUX_PAD_CTRL(ENET_PAD_CTRL)
};

int board_eth_init(bd_t *bis)
{

	/* Set pins to strapping GPIO modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_pads1,
					 ARRAY_SIZE(fec_enet_pads1));

	/* Reset */
	gpio_direction_output(EN_ETH_PHY_PWR, 0);
	mdelay(5); // falls in ~2ms
	gpio_direction_output(EN_ETH_PHY_PWR, 1);

	gpio_direction_output(PHY1_DUPLEX, 0);
	gpio_direction_output(PHY2_DUPLEX, 0);
	gpio_direction_output(PHY1_PHYADDR2, 0);
	gpio_direction_output(PHY2_PHYADDR2, 0);
	gpio_direction_output(PHY1_CONFIG_2, 0);
	gpio_direction_output(PHY2_CONFIG_2, 0);
	gpio_direction_output(PHY1_ISOLATE, 0);
	gpio_direction_output(PHY2_ISOLATE, 0);

	/* PHY_RESET automatically deasserts 140-280ms after we turn on power.
	 * where it will strap in the startup values from GPIO. */
	mdelay(320);

	/* Set pins to enet modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_pads,
					 ARRAY_SIZE(fec_enet_pads));

	return fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
						 CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
}

static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;
	if (check_module_fused(MX6_MODULE_ENET1))
		return -1;
	if (check_module_fused(MX6_MODULE_ENET2))
		return -1;

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
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	imx_iomux_v3_setup_multiple_pads(fpga_jtag_pads,
					 ARRAY_SIZE(fpga_jtag_pads));

	imx_iomux_v3_setup_multiple_pads(
		misc_pads, ARRAY_SIZE(misc_pads));

	/* Keep as inputs to allow offboard programming */
	gpio_direction_input(JTAG_FPGA_TDI);
	gpio_direction_input(JTAG_FPGA_TCK);
	gpio_direction_input(JTAG_FPGA_TMS);
	gpio_direction_input(JTAG_FPGA_TDO);

	/* Enable LVDS clock output.
	 * Writing CCM_ANALOG_MISC1 to use output from 24M OSC */
	setbits_le32(0x020C8160, 0x412);

	/* reset the FGPA */
	gpio_direction_output(FPGA_RESETN, 0);
	mdelay(1);
	gpio_direction_output(FPGA_RESETN, 1);

	/* Set up I2C bus for uC */
	setup_i2c(

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif

#ifdef CONFIG_FPGA
	/* Set up FPGA subsystem for JTAGing, i.e., soft loading */
	ts7180_fpga_init();
#endif

	return 0;
}

int board_late_init(void)
{
	int jpr;
	uint8_t opts = 0;

	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	/* Onboard jumpers to boot to SD or break in u-boot */
	gpio_direction_input(SD_BOOT_JMPN);
	gpio_direction_input(PUSH_SW_CPUN);
	gpio_direction_input(U_BOOT_JMPN);
	gpio_direction_input(NO_CHRG_JMPN);

	jpr = gpio_get_value(NO_CHRG_JMPN);
	if(jpr) setenv("jpnochrg", "off");
	else setenv("jpnochrg", "on");

	jpr = gpio_get_value(SD_BOOT_JMPN);
	if(jpr) setenv("jpsdboot", "off");
	else setenv("jpsdboot", "on");

	jpr = gpio_get_value(U_BOOT_JMPN);
	setenv("jpuboot", "off");
	if(!jpr) setenv("jpuboot", "on");
	else {
		if(getenv_ulong("rstuboot", 10, 1)) {
			jpr = gpio_get_value(PUSH_SW_CPUN);
			if(!jpr) setenv("jpuboot", "on");
		}
	}

	opts |= (gpio_get_value(OPT_ID_1) << 0);
	opts |= (gpio_get_value(OPT_ID_4) << 3);
	opts |= (gpio_get_value(OPT_ID_5) << 4); //Added line to support gpio read for ID5
        
	/* note: options 2 and 3 are connected to the FPGA */
	if (fpga_gpio_input(47))   /* FPGA pad N7, R36 on TS-7180 schematic) */
		opts |= (1 << 1);

	if (fpga_gpio_input(48))   /* FPGA pad P7, R37 on TS-7180 schematic) */
		opts |= (1 << 2);

	setenv_hex("opts", (opts & 0xF));
	fpga_late_init();
	red_led_on();
	green_led_off();

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	int fpgarev;

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info3);
	fpgarev = fpga_get_rev();
	puts("Board: Technologic Systems TS-7180\n");
	if(fpgarev < 0)
		printf("FPGA I2C communication failed: %d\n", fpgarev);
	else
		printf("FPGA:  Rev %d\n", fpgarev);
	return 0;
}

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
