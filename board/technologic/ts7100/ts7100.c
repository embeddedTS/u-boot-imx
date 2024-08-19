/*
 * Copyright (C) 2016-2022 Technologic Systems dba embeddedTS
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
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <fdt_support.h>
#include <fpga.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <lattice.h>
#include <linux/sizes.h>
#include <linux/fb.h>
#include <miiphy.h>
#include <mmc.h>
#include <netdev.h>
#include <usb.h>
#include "parse_strap.h"
#include "tsfpga.h"

DECLARE_GLOBAL_DATA_PTR;

#define EN_ETH_PHY_PWR 		IMX_GPIO_NR(1, 10)
#define PHY1_DUPLEX 		IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX 		IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2 		IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2		IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2 		IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2		IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE		IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE		IMX_GPIO_NR(2, 15)
#define JTAG_FPGA_TDO		IMX_GPIO_NR(4, 0)
#define JTAG_FPGA_TDI		IMX_GPIO_NR(4, 17)
#define JTAG_FPGA_TMS		IMX_GPIO_NR(3, 6)
#define JTAG_FPGA_TCK		IMX_GPIO_NR(3, 5)

#define ENET_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |      \
		       PAD_CTL_SPEED_HIGH |                     \
		       PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_120ohm |     \
			   PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_SPEED_HIGH | \
			  PAD_CTL_SRE_FAST)

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
			PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_22K_UP | \
			PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define USDHC_PAD_CTRL_WP (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN | \
			   PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define MISC_PAD_PU_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PKE | PAD_CTL_PUE | \
			  PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define MISC_PAD_PD_CTRL (PAD_CTL_PUS_100K_DOWN | PAD_CTL_PKE | PAD_CTL_PUE | \
			  PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define MISC_PAD_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
		      PAD_CTL_ODE)

#define LCD_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
		      PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_34ohm)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

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

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |                    \
			 PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |      \
			 PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

int is_mfg(void)
{
	return is_boot_from_usb();
}

extern int64_t silab_cmd(int argc, char *const argv[]);
#if defined(HAVE_SILAB_REV)
extern int64_t silab_rev(void);
#endif

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
	char * const rebootcmd[] = {"silabs", "wdog", "set", "1"};
	silab_cmd(4, rebootcmd);
#endif
}

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
static iomux_v3_cfg_t const eim_pins[] = {
	MX6_PAD_CSI_DATA00__EIM_AD00 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_DATA01__EIM_AD01 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_DATA02__EIM_AD02 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_DATA03__EIM_AD03 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_DATA04__EIM_AD04 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_DATA05__EIM_AD05 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_DATA06__EIM_AD06 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_DATA07__EIM_AD07 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__EIM_AD08 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__EIM_AD09 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__EIM_AD10 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__EIM_AD11 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA04__EIM_AD12 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA05__EIM_AD13 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA06__EIM_AD14 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DATA07__EIM_AD15 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_VSYNC__EIM_RW | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_HSYNC__EIM_LBA_B | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_CSI_PIXCLK__EIM_OE | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_DQS__EIM_WAIT | MUX_PAD_CTRL(MISC_PAD_CTRL),
	MX6_PAD_NAND_WP_B__EIM_BCLK | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),

	/* FPGA_IRQ */
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* FPGA_IRQ2 */
	MX6_PAD_NAND_ALE__GPIO4_IO10 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
};
#endif

static iomux_v3_cfg_t const misc_pads[] = {
	/* POWER_FAIL_3V */
	MX6_PAD_SNVS_TAMPER0__GPIO5_IO00 | MUX_PAD_CTRL(MISC_PAD_CTRL),

	/* FPGA_1 */
	MX6_PAD_SNVS_TAMPER6__GPIO5_IO06 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	/* FPGA_2 */
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	/* FPGA_3 */
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	/* FPGA_4 */
	MX6_PAD_SNVS_TAMPER9__GPIO5_IO09 | MUX_PAD_CTRL(MISC_PAD_CTRL),

	/* JTAG_FPGA_TDI */
	MX6_PAD_CSI_MCLK__GPIO4_IO17 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* JTAG_FPGA_TDO */
	MX6_PAD_NAND_RE_B__GPIO4_IO00 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* JTAG_FPGA_TMS */
	MX6_PAD_LCD_DATA01__GPIO3_IO06 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* JTAT_FPGA_TCK */
	MX6_PAD_LCD_DATA00__GPIO3_IO05 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
};

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return USB_INIT_DEVICE;
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
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_PAD_CTRL),

#if (CONFIG_FEC_ENET_DEV == 0)
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
#else
	MX6_PAD_GPIO1_IO06__ENET2_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
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
};

int board_phy_config(struct phy_device *phydev)
{
	debug(">> board_phy_config\n");
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

void reset_phy(void)
{
	/* This was pulled from board_eth_init */
	/* Set pins to strapping GPIO modes */
	debug(">> reset_phy: START\n");

	imx_iomux_v3_setup_multiple_pads(fec_enet_pads1,
					 ARRAY_SIZE(fec_enet_pads1));

	gpio_request(PHY1_DUPLEX, "PHY1_DUPLEX");
	gpio_request(PHY2_DUPLEX, "PHY2_DUPLEX");
	gpio_request(PHY1_PHYADDR2, "PHY1_PHYADDR2");
	gpio_request(PHY2_PHYADDR2, "PHY2_PHYADDR2");
	gpio_request(PHY1_CONFIG_2, "PHY1_CONFIG_2");
	gpio_request(PHY2_CONFIG_2, "PHY2_CONFIG_2");
	gpio_request(PHY1_ISOLATE, "PHY1_ISOLATE");
	gpio_request(PHY2_ISOLATE, "PHY2_ISOLATE");

	/* Assert Reset and Enable Output */
	fpga_dio2_dat_clr(BANK2_PHY_RESETN);
	fpga_dio2_oe_set(BANK2_PHY_RESETN);

	gpio_direction_output(PHY1_DUPLEX, 0);
	gpio_direction_output(PHY2_DUPLEX, 0);
	gpio_direction_output(PHY1_PHYADDR2, 0);
	gpio_direction_output(PHY2_PHYADDR2, 0);
	gpio_direction_output(PHY1_CONFIG_2, 0);
	gpio_direction_output(PHY2_CONFIG_2, 0);
	gpio_direction_output(PHY1_ISOLATE, 0);
	gpio_direction_output(PHY2_ISOLATE, 0);

	/* Hold reset asserted */
	udelay(1);
	fpga_dio2_dat_set(BANK2_PHY_RESETN);
	mdelay(10);
	/* PHYs need 10 ms after first reset */

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
	debug(">> reset_phy: DONE\n");
}

static int setup_fec(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret0, ret1;
	debug(">> ts7100.c: setup_fec\n");

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17].
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);

	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
			IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);

	ret0 = enable_fec_anatop_clock(0, ENET_50MHZ);
	if (ret0) {
		debug("ERROR: enable_fec_anatop_clock failed for fec_id 0, ret=%d\n", ret0);
	}
	ret1 = enable_fec_anatop_clock(1, ENET_50MHZ);
	if (ret1) {
		debug("ERROR: enable_fec_anatop_clock failed for rec_id 1, ret=%d\n", ret1);
	}
	if (ret0 || ret1) {
		return (ret1 << 16) | ret0;
	}

	enable_enet_clk(1);

	debug(">> setup_fec DONE\n");
	return 0;
}
#endif /* CONFIG_FEC_MXC */

#ifdef CONFIG_BOARD_EARLY_INIT_F
int board_early_init_f(void)
{
	struct weim *weim_regs = (struct weim *)WEIM_BASE_ADDR;
	struct mxc_ccm_reg *const imx_ccm =
		(struct mxc_ccm_reg *)CCM_BASE_ADDR;

	/* Set up FPGA for communication */
	/* 396mhz PLL2_PDF2 div by 8 = 49.5MHz EIM clk */
	clrsetbits_le32(&imx_ccm->cscmr1,
			MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_MASK |
			MXC_CCM_CSCMR1_ACLK_EMI_SLOW_MASK,
			7 << MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_OFFSET |
			2 << MXC_CCM_CSCMR1_ACLK_EMI_SLOW_OFFSET);

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
	// Look in spl:board_setup_eim(), called by spl:board_init_f
	/* Set up EIM bus for FPGA */
	imx_iomux_v3_setup_multiple_pads(
		eim_pins, ARRAY_SIZE(eim_pins));
#endif

	writel(0x0161030F, &weim_regs->cs0gcr1);
	writel(0x00000000, &weim_regs->cs0gcr2);
	writel(0x03000000, &weim_regs->cs0rcr1);
	writel(0x00000000, &weim_regs->cs0rcr2);
	writel(0x01000000, &weim_regs->cs0wcr1);
	writel(0x00000E09, &weim_regs->wcr);

	set_chipselect_size(CS0_128);

	/* Set up I2C bus for uC */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
	setup_iomux_uart();
#endif

	return 0;
}
#endif  /* CONFIG_BOARD_EARLY_INIT_F */

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
#ifdef CONFIG_FEC_MXC
	reset_phy();
	setup_fec();
#endif

	return 0;
}

int board_late_init(void)
{
	char fdtfile[64] = {0};
	char rev_as_str[2] = {0};
	uint32_t cpu_opts;
	uint32_t fpga_rev = readl(FPGA_REV);
	uint32_t io_model;
	uint32_t io_opts;
	uint32_t cpu_straps;
	uint32_t fpga_straps;

	hw_watchdog_reset();

	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	/*
	 * WARNING: All of these are wiped out of the environment
	 * after an "env default -a", until the board is reset.
	 */
	env_set("model", get_board_model());

	fpga_straps = read_raw_fpga_straps();
	env_set_hex("raw_fpga_straps", fpga_straps);

	cpu_straps = read_raw_cpu_straps();
	env_set_hex("raw_cpu_straps", cpu_straps);

	cpu_opts = read_cpu_board_opts();
	env_set_hex("opts", cpu_opts);

	io_model = read_io_board_model();
	env_set_hex("io_model", io_model);

	snprintf(fdtfile, sizeof(fdtfile),
		 "imx6ul-ts%s-%d.dtb", get_board_model(), io_model);
	env_set("fdtfile", fdtfile);

	io_opts = read_io_board_opts();
	env_set_hex("io_opts", io_opts);

	env_set("board_name", get_board_name());
	rev_as_str[0] = get_cpu_board_version_char();
	env_set("board_rev", rev_as_str);
	env_set("board_rev_straps", get_cpu_board_version_str());

	env_set_hex("fpga_rev", fpga_rev & 0x7fffffff);

	if(is_mfg()) {
		env_set("bootcmd", "mfg");
		env_set("bootdelay", "1");
	}

	return 0;
}

static int fixup_ism330(void *blob, bd_t *bd)
{
	const char fdtpath[] = "/soc/bus@2100000/i2c@21a0000/gyro@6a";
	const char *compatible = NULL;
	struct udevice *chip;
	struct udevice *bus;
	uint8_t value;
	int ret;

	ret = uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	if (ret)
		return 1;

	ret = i2c_get_chip(bus, 0x6a, 1, &chip);
	if (ret)
		return 1;

	/* WHOAMI register */
	ret = dm_i2c_read(chip, 0x0f, &value, 1);
	if (ret) {
		printf("fixup_ism330: No IMU found\n");
		return 1;
	}
	switch (value) {
		case 0x6b:
			compatible = "st,ism330dhcx";
			break;
		case 0x6a:
			compatible = "st,ism330dlc";
			break;
		default:
			printf("fixup_ism330: Unknown whoami (0x%X)\n", value);
			return 1;
	}
	return fdt_find_and_setprop(blob, fdtpath, "compatible", compatible, strlen(compatible), 0);
}

int ft_board_setup(void *blob, bd_t *bd)
{
	fixup_ism330(blob, bd);
	return 0;
}

int checkboard(void)
{
	uint32_t fpga_rev = readl(FPGA_REV);
	uint32_t fpga_hash = readl(FPGA_HASH);

	/* TS-7100 variant info goes here */
	printf("FPGA: Rev %d ", fpga_rev & 0x7fffffff);
	if (fpga_rev & (1 << 31))
		printf("(%x-dirty)\n", fpga_hash);
	else
		printf("(%x)\n", fpga_hash);

	return 0;
}
