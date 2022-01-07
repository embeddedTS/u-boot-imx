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
#include <asm/imx-common/video.h>
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

DECLARE_GLOBAL_DATA_PTR;

#define ETH_PHY_RSTN		IMX_GPIO_NR(3, 11)
/* Pads used to strap the PHY at unreset */
#define PHY1_DUPLEX 		IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX 		IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2 		IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2		IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2 		IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2		IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE		IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE		IMX_GPIO_NR(2, 15)

/* Option straps */
#define STRAP_0			IMX_GPIO_NR(3, 26) /* LCD_DATA21 */
#define STRAP_1			IMX_GPIO_NR(3, 27) /* LCD_DATA22 */
#define STRAP_2			IMX_GPIO_NR(3, 28) /* LCD_DATA23 */
#define STRAP_3			IMX_GPIO_NR(3, 0)  /* LCD_CLK */
#define STRAP_4			IMX_GPIO_NR(3, 1)  /* LCD_ENABLE */

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_22K_UP | \
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define USDHC_PAD_CTRL_WP (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN | \
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define MISC_PAD_PU_CTRL (PAD_CTL_PUS_100K_UP | PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define MISC_PAD_PD_CTRL (PAD_CTL_PUS_100K_DOWN | PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define MISC_PAD_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_120ohm | \
	PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_SPEED_HIGH | \
	PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
	PAD_CTL_ODE)

#define LCD_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_34ohm)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP  | \
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | \
	PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |         \
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST | PAD_CTL_PUE |\
	PAD_CTL_PUS_100K_UP)


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

int dram_init(void)
{
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const misc_pads[] = {
	/* POWER_FAIL_3V */
	MX6_PAD_SNVS_TAMPER0__GPIO5_IO00 | MUX_PAD_CTRL(MISC_PAD_CTRL),

	/* Board strap 0 */
	MX6_PAD_LCD_DATA21__GPIO3_IO26 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* Board strap 1 */
	MX6_PAD_LCD_DATA22__GPIO3_IO27 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* Board strap 2 */
	MX6_PAD_LCD_DATA23__GPIO3_IO28 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* Board strap 3 */
	MX6_PAD_LCD_CLK__GPIO3_IO00 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* Board strap 4 */
	MX6_PAD_LCD_ENABLE__GPIO3_IO01 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),

	/* Status LED */
	MX6_PAD_LCD_DATA12__GPIO3_IO17 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* Fault LED */
	MX6_PAD_LCD_DATA13__GPIO3_IO18 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* RS-485 LED */
	MX6_PAD_LCD_DATA14__GPIO3_IO19 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),

	/* En. OLED 3.3 V */
	MX6_PAD_LCD_DATA03__GPIO3_IO08 | MUX_PAD_CTRL(MISC_PAD_PD_CTRL),
	/* En. RF 24 V */
	MX6_PAD_LCD_DATA05__GPIO3_IO10 | MUX_PAD_CTRL(MISC_PAD_PD_CTRL),
};

/* SD */
static iomux_v3_cfg_t const usdhc2_sd_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	
};

/* eMMC */
static iomux_v3_cfg_t const usdhc1_emmc_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

/* OLED SPI */
static iomux_v3_cfg_t const ecspi4_pads[] = {
        MX6_PAD_NAND_DATA07__GPIO4_IO09  | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_NAND_DATA04__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_NAND_DATA05__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
        MX6_PAD_NAND_DATA06__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
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
	 * mmc0                    USDHC1 (eMMC)
	 * mmc1                    USDHC2 (SD)
	 */

	imx_iomux_v3_setup_multiple_pads(
		usdhc1_emmc_pads, ARRAY_SIZE(usdhc1_emmc_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);

	imx_iomux_v3_setup_multiple_pads(
		usdhc2_sd_pads, ARRAY_SIZE(usdhc2_sd_pads));
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[0]))
		printf("Warning: failed to initialize eMMC dev\n");

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[1]))
		printf("Warning: failed to initialize eMMC dev\n");

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
	MX6_PAD_GPIO1_IO06__ENET2_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
#endif

	/* PHY Reset */
	MX6_PAD_LCD_DATA06__GPIO3_IO11 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const fec_enet_strap_pads[] = {
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
	/* PHY Reset */
	MX6_PAD_LCD_DATA06__GPIO3_IO11 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

int board_eth_init(bd_t *bis)
{
	int ret, i;
	uint32_t uniq1, uniq2;
	uchar enetaddr[6];

	/* Set pins to strapping GPIO modes */
	imx_iomux_v3_setup_multiple_pads(fec_enet_strap_pads,
					 ARRAY_SIZE(fec_enet_strap_pads));

	gpio_direction_output(ETH_PHY_RSTN, 0);
	gpio_direction_output(PHY1_DUPLEX, 0);
	gpio_direction_output(PHY2_DUPLEX, 0);
	gpio_direction_output(PHY1_PHYADDR2, 0);
	gpio_direction_output(PHY2_PHYADDR2, 0);
	gpio_direction_output(PHY1_CONFIG_2, 0);
	gpio_direction_output(PHY2_CONFIG_2, 0);
	gpio_direction_output(PHY1_ISOLATE, 0);
	gpio_direction_output(PHY2_ISOLATE, 0);

	udelay(1); // 5ns
	gpio_direction_output(ETH_PHY_RSTN, 1);
	udelay(0); // 5ns

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

	/* Linux FEC driver needs enetaddr set in FDT. Custom Freescale/NXP
	 * patches used to read this from OTP and self-modify the booted FDT early
	 * boot. To support mainline, export both eth0 and eth1 MAC to U-Boot
	 * env. This device is assigned two MAC addresses, the second is just
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
	/* Set up I2C bus for uC */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	setup_iomux_uart();

	return 0;
}

int misc_init_r(void)
{
	uint8_t opts = 0;

	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	setenv("model", "tsmflow");

	/* Read option straps on CPU pins */
	gpio_direction_input(STRAP_0);
	gpio_direction_input(STRAP_1);
	gpio_direction_input(STRAP_2);
	gpio_direction_input(STRAP_3);
	gpio_direction_input(STRAP_4);
	opts |= (gpio_get_value(STRAP_0) << 0);
	opts |= (gpio_get_value(STRAP_1) << 1);
	opts |= (gpio_get_value(STRAP_2) << 2);
	opts |= (gpio_get_value(STRAP_3) << 3);
	opts |= (gpio_get_value(STRAP_4) << 4);
	
	/* All straps are 1 = resistor populated. This is inverted from
	 * the logic level read from the IO pins.  */
	opts = opts ^ 0x1F;
	setenv_hex("opts", opts);

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_spi();

#ifdef CONFIG_FEC_MXC
	setup_fec(CONFIG_FEC_ENET_DEV);
#endif


	return 0;
}

int board_late_init(void)
{
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return USB_INIT_DEVICE;
}

#endif
