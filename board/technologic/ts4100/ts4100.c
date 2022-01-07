/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
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
#include <fuse.h>
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
#include "bb.h"
#include "fpga.h"
#include "silabs.h"
#include "parse_strap.h"

DECLARE_GLOBAL_DATA_PTR;

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

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |         \
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST | PAD_CTL_PUE |\
	PAD_CTL_PUS_100K_UP)

#define EN_FPGA_PWR             IMX_GPIO_NR(5, 2)
#define FPGA_RESETN             IMX_GPIO_NR(4, 11)
#define JTAG_FPGA_TDO           IMX_GPIO_NR(5, 4)
#define JTAG_FPGA_TDI           IMX_GPIO_NR(5, 5)
#define JTAG_FPGA_TMS           IMX_GPIO_NR(5, 6)
#define JTAG_FPGA_TCK           IMX_GPIO_NR(5, 7)
#define EN_ETH_PHY_PWR 		IMX_GPIO_NR(1, 10)
#define PHY1_DUPLEX 		IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX 		IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2 		IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2		IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2 		IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2		IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE		IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE		IMX_GPIO_NR(2, 15)

#define PRESENT 1
#define NOT_PRESENT 0
#define FORCE_SET 0
#define FORCE_UNSET 1

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

/* SPI - WiFi */
iomux_v3_cfg_t const ecspi4_pads[] = {
	MX6_PAD_NAND_DATA07__GPIO4_IO09  | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_NAND_DATA04__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_NAND_DATA05__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_NAND_DATA06__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

int dram_init(void)
{
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;

	return 0;
}

iomux_v3_cfg_t const fpga_jtag_pads[] = {
	/* JTAG_FPGA_TDO */
	MX6_PAD_SNVS_TAMPER4__GPIO5_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), 
	/* JTAG_FPGA_TDI */
	MX6_PAD_SNVS_TAMPER5__GPIO5_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* JTAG_FPGA_TMS */
	MX6_PAD_SNVS_TAMPER6__GPIO5_IO06 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* JTAG_FPGA_TCK */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* EN_FPGA_PWR */
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* FPGA_RESET# */
	MX6_PAD_NAND_WP_B__GPIO4_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), 
};

void fpga_mmc_init(void)
{
	/* Rev B has SD power connected to TAMPER9, already biased high */
}

/* This function is responsible for setting up strapping resistor options
 * as well as jumpers/env var set up for baseboard, if any.
 * Not all baseboards that the TS-4100 can be on support the proper jumpers
 * and the TS-4100 can be run without a baseboard.
 * We create a blacklist of baseboards we know of that don't support
 * the correct jumper configuration and just assume all other BBs have the
 * proper support.
 */
void config_opts(int bbid)
{
	unsigned int sdboot, uboot, nochrg, pswitch, bbsilo;
	uint8_t opts;

	/* SD Boot jumper on some BBs use OFF_BD_RESET as the ground conn. */
	fpga_gpio_output(OFF_BD_RESET_PADN, 0);

	/* We have access to the full ID of the BB which includes PCB rev.
	 * This can be used here if needed for changes from rev to rev.
	 *
	 * FORCE_SET and FORCE_UNSET are used to force a jumper state. This is
	 * used in situations where a specific jumper is not present.
	 *
	 * In applications without a U-Boot jumper, the following paradigm is
	 * recommended. See case 0x3F below for an example.
	 * Force the jumper to off
	 * Set the bootdelay env var to force_bootdelay, fallback to "1"
	 *
	 * PSwitch is intended for use only on production/development baseboards.
	 * Because of this, we assume no PSwitch is present unless its in the
	 * exception list below.
	 */
	pswitch = FORCE_UNSET;
	sdboot = fpga_gpio_input(DIO_20);
	uboot = fpga_gpio_input(DIO_43);
	nochrg = fpga_gpio_input(SPARE1);
	bbsilo = PRESENT;

	switch (bbid & ~0xC0) {
	  case 0x3F: /* No BB/no ID means no valid jumpers present */
		setenv("baseboard", "None/Unknown");
		sdboot = !(getenv_ulong("force_jpsdboot", 10, 0) & 0x1);
		uboot = FORCE_UNSET;
		setenv_ulong("bootdelay",
		  getenv_ulong("force_bootdelay", 10, 1));
		nochrg = FORCE_UNSET;
		bbsilo = NOT_PRESENT;
		break;
	  case 0x2F: /* Reserved */
	  case 0x2E: /* Reserved */
		setenv("baseboard", "Custom");
		nochrg = FORCE_UNSET;
		bbsilo = NOT_PRESENT;
		break;
	  case 0x2A: /* Custom baseboard, assumed std jumpers and TS-SILO */
		setenv("baseboard", "Custom");
		break;
	  case 0x16: /* TS-8551 adds PSwitch */
		setenv("baseboard", "TS-8551");
		pswitch = fpga_gpio_input(DIO_09);
		break;
	  case 0x07: /* TS-8100 */
	        setenv("baseboard", "TS-8100");
	        pswitch = fpga_gpio_input(DIO_09);
                uboot = FORCE_UNSET;
	        setenv_ulong("bootdelay",
                  getenv_ulong("force_bootdelay", 10, 1));
                nochrg = FORCE_UNSET;
	        bbsilo = NOT_PRESENT;
	        break;
	  case 0x08: /* TS-8820 */
		setenv("baseboard", "TS-8820");
		pswitch = fpga_gpio_input(DIO_09);
		/* Starting from BB_REV 1, Rev D PCB, the TS-8820 has TS-SILO, and U-Boot
		 * jumper present.
		 * BB_REV 0, PCB Rev C and earlier, only have pswitch and SD boot jumpers
		 */
		if (((bbid & 0xC0) >> 6) == 0) {
			uboot = FORCE_UNSET;
			setenv_ulong("bootdelay",
			  getenv_ulong("force_bootdelay", 10, 1));
			nochrg = FORCE_UNSET;
			bbsilo = NOT_PRESENT;
		}
		break;
	  default: /* All other boards presumed to have std. jumper locations w/
		    * TS-SILO
		    */
		setenv("baseboard", "Custom/Unknown");
		break;
	}


	/* While OFF_BD_RESET_PADN is low read CN1_98 which 
	 * will have a pulldown to OFF_BD_RESET_PADN if the sd
	 * boot jumper is on */
	if(sdboot) {
		setenv("jpsdboot", "off");
	} else {
		setenv("jpsdboot", "on");
	}

	setenv("jpuboot", "off");
	if(!uboot) setenv("jpuboot", "on");
	else {
		if(getenv_ulong("rstuboot", 10, 1)) {
			if(!pswitch) setenv("jpuboot", "on");
		}
	}

	if (nochrg) {
		setenv("jpnochrg", "off");
	} else {
		setenv("jpnochrg", "on");
	}

	opts = parse_strap();
	if ((opts & 0x10) && bbsilo) setenv("silopresent", "1");
	else setenv("silopresent", "0");

	mdelay(10);
	fpga_gpio_output(OFF_BD_RESET_PADN, 1);
	fpga_gpio_output(EN_USB_HOST_5V_PAD, 1);
}

#if defined(CONFIG_FPGA)

static void ts4100_fpga_jtag_init(void)
{
	gpio_direction_output(JTAG_FPGA_TDI, 1);
	gpio_direction_output(JTAG_FPGA_TCK, 1);
	gpio_direction_output(JTAG_FPGA_TMS, 1);
	gpio_direction_input(JTAG_FPGA_TDO);
}

static void ts4100_fpga_done(void)
{
	gpio_direction_input(JTAG_FPGA_TDI);
	gpio_direction_input(JTAG_FPGA_TCK);
	gpio_direction_input(JTAG_FPGA_TMS);
	gpio_direction_input(JTAG_FPGA_TDO);

	/* During FPGA programming several important pins will
	 * have been tristated.  Put it back to normal */
	fpga_mmc_init();
	red_led_on();
	green_led_off();
	fpga_gpio_output(OFF_BD_RESET_PADN, 1);
	fpga_gpio_output(EN_USB_HOST_5V_PAD, 1);
}

static void ts4100_fpga_tdi(int value)
{
	gpio_set_value(JTAG_FPGA_TDI, value);
}

static void ts4100_fpga_tms(int value)
{
	gpio_set_value(JTAG_FPGA_TMS, value);
}

static void ts4100_fpga_tck(int value)
{
	gpio_set_value(JTAG_FPGA_TCK, value);
}

static int ts4100_fpga_tdo(void)
{
	return gpio_get_value(JTAG_FPGA_TDO);
}

lattice_board_specific_func ts4100_fpga_fns = {
	ts4100_fpga_jtag_init,
	ts4100_fpga_tdi,
	ts4100_fpga_tms,
	ts4100_fpga_tck,
	ts4100_fpga_tdo,
	ts4100_fpga_done
};

Lattice_desc ts4100_fpga = {
	Lattice_XP2,
	lattice_jtag_mode,
	589012,
	(void *) &ts4100_fpga_fns,
	NULL,
	0,
	"machxo_2_cb132"
};

int ts4100_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts4100_fpga);

	return 0;
}

#endif

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_JTAG_TMS__CCM_CLKO1 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_sd_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__USDHC1_VSELECT | MUX_PAD_CTRL(NO_PAD_CTRL),
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

#define USDHC1_VSELECT IMX_GPIO_NR(1, 5)

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

	/* For the SD Select 3.3V instead of 1.8V */
	gpio_direction_output(USDHC1_VSELECT, 1);

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
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA0__ENET2_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__ENET2_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__ENET2_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__ENET2_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
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
	int ret, i;
	uint32_t uniq1, uniq2;
	uchar enetaddr[6];

	/* Set pins to strapping GPIO modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_pads1,
					 ARRAY_SIZE(fec_enet_pads1));

	/* Reset */
	gpio_direction_output(EN_ETH_PHY_PWR, 0);
	fpga_gpio_output(ETH_PHY_RESET, 1);
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

	/* PHYs require minimum 10 ms from power valid, to unreset
	 * Strap values are read ~at unreset time.
	 * We wait 15 ms just to provide some additional margin
	 */
	mdelay(15);
	fpga_gpio_output(ETH_PHY_RESET, 0);

	/* Set pins to enet modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_pads,
					 ARRAY_SIZE(fec_enet_pads));

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
				       CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);

	/* If env var ethaddr is not set, then the fuse'ed MAC is not a
	 * proper MAC (generally just not set).  In this case, a random one
	 * needs to be generated.  Stock generation uses time as the srand()
	 * seed, and this is a bad idea when multiple units are done on the
	 * rack at the same time.  This pulls two unique IDs from fuses;
	 * correspond to lot number and part in lot number, and sets srand()
	 * from the XOR of those two values.
	 */
	if (!eth_getenv_enetaddr("ethaddr", enetaddr)) {
		printf("No MAC address set in fuses. Using random MAC\n");

		/* Read two unique IDs stored in OTP fuses */
		fuse_read(0, 1, &uniq1);
		fuse_read(0, 2, &uniq2);

		srand(uniq1 ^ uniq2);
		for (i = 0; i < 6; i++) {
			enetaddr[i] = rand();
		}
		enetaddr[0] &= 0xfe;	/* Clear multicast bit */
		enetaddr[0] |= 0x02;	/* Set local assignment bit (IEEE802) */

		if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
			printf("Failed to set a random MAC address\n");
		}
	}

	/* Linux FEC driver needs enetaddr set in FTD. Custom Freescale/NXP
	 * patches will read this from OTP and self-modify the booted FDT early
	 * boot. To support mainline, export both eth0 and eth1 MAC to U-Boot
	 * env. The TS-4100 is assigned two MAC addresses, the second is just
	 * +1 from the first. Ensure that all the lower 3 btyes are properly
	 * adjusted for rollover.
	 */
	if (enetaddr[5] == 0xff) {
		if (enetaddr[4] == 0xff) {
			enetaddr[3]++;
		}
		enetaddr[4]++;
	}
	enetaddr[5]++;
	if (eth_setenv_enetaddr("eth1addr", enetaddr)) {
		printf("Failed to set eth1addr\n");
	}

	return ret;
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

void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi4_pads,
	  ARRAY_SIZE(ecspi4_pads));
}

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
	ts4100_fpga_init();
	#endif

	setup_spi();

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"sd", MAKE_CFGVAL(0x42, 0x20, 0x00, 0x00)},
	{"emmc1", MAKE_CFGVAL(0x74, 0xa8, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	/* Detect the carrier board ID and 4100 opts */
	config_opts(bbdetect());

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

	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x4a, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x28, &i2c_pad_info3);

	if (is_boot_from_usb()) {
		int tout = 600; /* Expect ~42 seconds.  Padding to 60 */
		uint8_t val[2];
		i2c_set_bus_num(2);
		printf("Waiting for FPGA... ");
		mdelay(1000);
		val[0] = 8; /* Request internal state # */
		while (tout-- > 0) {
			i2c_read(0x38, 8, 1, val, 2);
			if(val[1] == 5) /* State 5 == finished programming */
				break;
			udelay(100000);
		}
		if(tout <= 0)
			printf("FPGA Timed out!\n");
		else
			printf("FPGA programmed!\n");
	}

	/* reset the FGPA */
	gpio_direction_output(FPGA_RESETN, 0);
	gpio_direction_output(EN_FPGA_PWR, 0);
	// off is 70us max
	mdelay(1);
	gpio_direction_output(EN_FPGA_PWR, 1);
	// on is typical ~5ms
	mdelay(10);
	gpio_direction_output(FPGA_RESETN, 1);

	fpgarev = fpga_get_rev();
	puts("Board: embeddedTS TS-4100\n");
	if(fpgarev < 0)
		printf("FPGA I2C communication failed: %d\n", fpgarev);
	else
		printf("FPGA:  Rev %d\n", fpgarev);
	printf("Silab: Rev %d\n", silab_rev());

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
		return usb_phy_mode(port);
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


static int do_bbdetect(cmd_tbl_t *cmdtp, int flag,
	int argc, char * const argv[])
{
	return 0;
}

U_BOOT_CMD(bbdetect, 4, 0, do_bbdetect,
	"TS Baseboard detect compat",
	"  This command does nothing on this platform. It is provided for\n"
	"  script compatibility. U-Boot already exports the environment\n"
	"  names that this command would normally set.\n"
);
