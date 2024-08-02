// SPDX-License-Identifier:	GPL-2.0+
/*
 * Copyright (C) 2020-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <common.h>
#include <spl.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <usb.h>
#include <asm/spl.h>
#include <linux/libfdt.h>
#include <fsl_esdhc_imx.h>

#include "tsfpga.h"

#define OPT_ID_4                IMX_GPIO_NR(3, 23)
#define EN_EMMC_3V_N            IMX_GPIO_NR(4, 22)

#define MISC_PAD_PU_CTRL (PAD_CTL_PUS_22K_UP | PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_DSE_48ohm | PAD_CTL_SPEED_MED | PAD_CTL_HYS)

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define EMMC_PAD_CTRL (PAD_CTL_PUS_100K_DOWN | PAD_CTL_PKE | \
	 PAD_CTL_DSE_65ohm | PAD_CTL_SPEED_LOW)

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const misc_pads[] = {
	MX6_PAD_CSI_DATA01__GPIO4_IO22 | MUX_PAD_CTRL(EMMC_PAD_CTRL), /* EN_EMMC_3.3V# */
	MX6_PAD_LCD_DATA11__GPIO3_IO16 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL), /* U_BOOT_JMP# */
	MX6_PAD_LCD_DATA13__GPIO3_IO18 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL), /* PUSH_SW_CPU# */
	MX6_PAD_LCD_DATA06__GPIO3_IO11 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL), /* NO_CHRG_JMP# */
	MX6_PAD_CSI_VSYNC__GPIO4_IO19 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),  /* R30 = Option ID5 */
	MX6_PAD_LCD_DATA18__GPIO3_IO23 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL), /* R38 = Option ID4 */
	MX6_PAD_LCD_DATA22__GPIO3_IO27 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL), /* R31 = Option ID1 */
};


/* 
 * USB is handled in ts7180.c:
 * 
 *   const usb_otg1_pads
 *   board_usb_phy_mode()
 *   board_ehci_hcd_init()
 *
 * I *presume* these are set up in SPL boot.
 */

static void early_init(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));

	/* v2016 is happy with the POR value (i.e., for the EN_MMC_3V pad) */
	/* Or maybe its DCD just takes care of things? */
	SETUP_IOMUX_PADS(misc_pads);
}

static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x00080000,
};

static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000030,
	.dram_odt1 = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000030,
	.dram_sdqs0 = 0x00000030,
	.dram_sdqs1 = 0x00000030,
	.dram_reset = 0x00000030,
};

static struct mx6_mmdc_calibration ts7180_512m_calibration = {
	.p0_mpwldectrl0 = 0x00050000,
	.p0_mpdgctrl0 = 0x01480144,
	.p0_mprddlctl = 0x40404E52,
	.p0_mpwrdlctl = 0x4040504E,
};

struct mx6_ddr_sysinfo ts7180_sysinfo = {
	.dsize = 0, /* 16-bit bus */
	.cs_density = 32, /* actually 512MB/1G */
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 0,
	.rtt_nom = 1,
	.walat = 1,
	.ralat = 5,
	.mif3_mode = 3,
	.bi_on = 1,
	.sde_to_rst = 0x10,
	.rst_to_cke = 0x23,
	.ddr_type = DDR_TYPE_DDR3,
	.refsel = 1,
	.refr = 3,
};

static struct mx6_ddr3_cfg ts7180_512m_ddr = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static struct mx6_ddr3_cfg ts7180_1g_ddr = {
	.mem_speed = 800,
	.density = 8,
	.width = 16,
	.banks = 8,
	.rowaddr = 16,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
	uint32_t reg;

	gpio_direction_input(OPT_ID_4);

	mx6ul_dram_iocfg(16, &mx6_ddr_ioregs, &mx6_grp_ioregs);

	reg = gpio_get_value(OPT_ID_4);
	if (reg == 0) {
		/* 4Gb Alliance AS4C256M16D3LB */
		mx6_dram_cfg(&ts7180_sysinfo,
			     &ts7180_512m_calibration,
			     &ts7180_512m_ddr);
	} else {
		/* 8Gb Alliance AS4C512M16D3L-12BCN */
		mx6_dram_cfg(&ts7180_sysinfo,
			     &ts7180_512m_calibration,
			     &ts7180_1g_ddr);
	}
}

void board_init_f(ulong dummy)
{
	ccgr_init();
	arch_cpu_init();
	timer_init();
	early_init();
	preloader_console_init();
	spl_dram_init();
	memset(__bss_start, 0, __bss_end - __bss_start);

	fpga_reset();
	board_init_r(NULL, 0);
}

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

#define USDHC_CLK_PAD_CTRL (PAD_CTL_HYS |	\
			    PAD_CTL_SPEED_MED |	\
			    PAD_CTL_DSE_52ohm |	\
			    PAD_CTL_SRE_FAST)

#define USDHC_DATA_PAD_CTRL (PAD_CTL_HYS |			\
			     PAD_CTL_PUS_47K_UP | PAD_CTL_PKE |	\
			     PAD_CTL_SPEED_HIGH |		\
			     PAD_CTL_DSE_88ohm |		\
			     PAD_CTL_SRE_FAST)

static iomux_v3_cfg_t const usdhc1_sd_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_CLK_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_CSI_DATA07__USDHC1_VSELECT | MUX_PAD_CTRL(NO_PAD_CTRL),  /* SD_VSEL_1.8V */
};

static iomux_v3_cfg_t const usdhc2_emmc_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_CLK_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_DATA_PAD_CTRL),
};

void board_boot_order(u32 *spl_boot_list) {
	if (spl_boot_device() == BOOT_DEVICE_BOARD) {
		spl_boot_list[0] = BOOT_DEVICE_BOARD;
		return;
	}
	spl_boot_list[0] = BOOT_DEVICE_MMC2;
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC2_BASE_ADDR, 0, 4},
};

int board_mmc_init(bd_t *bis)
{
	gpio_direction_output(EN_EMMC_3V_N, 0);

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
