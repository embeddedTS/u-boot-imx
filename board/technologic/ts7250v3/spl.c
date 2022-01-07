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
#include <fsl_esdhc_imx.h>
#include <linux/libfdt.h>

#include "tsfpga.h"

#define EIM_PAD_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST | PAD_CTL_SPEED_MED)

#define MISC_PAD_CTRL (PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)

#define MISC_PAD_PU_CTRL (MISC_PAD_CTRL | PAD_CTL_PUS_100K_UP | PAD_CTL_PKE | \
	PAD_CTL_PUE)

#define MISC_PAD_PD_CTRL (MISC_PAD_CTRL | PAD_CTL_PUS_100K_DOWN | PAD_CTL_PKE | \
	PAD_CTL_PUE)

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const eim_pins[] = {
	MX6_PAD_CSI_DATA00__EIM_AD00 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_DATA01__EIM_AD01 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_DATA02__EIM_AD02 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_DATA03__EIM_AD03 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_DATA04__EIM_AD04 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_DATA05__EIM_AD05 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_DATA06__EIM_AD06 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_DATA07__EIM_AD07 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA00__EIM_AD08 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA01__EIM_AD09 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA02__EIM_AD10 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA03__EIM_AD11 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA04__EIM_AD12 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA05__EIM_AD13 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA06__EIM_AD14 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DATA07__EIM_AD15 | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_VSYNC__EIM_RW | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_HSYNC__EIM_LBA_B | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_CSI_PIXCLK__EIM_OE | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_DQS__EIM_WAIT | MUX_PAD_CTRL(EIM_PAD_CTRL),
	MX6_PAD_NAND_WP_B__EIM_BCLK | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),

	/* FPGA_IRQ */
	MX6_PAD_SNVS_TAMPER1__GPIO5_IO01 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
	/* FPGA_IRQ2 */
	MX6_PAD_NAND_ALE__GPIO4_IO10 | MUX_PAD_CTRL(MISC_PAD_PU_CTRL),
};

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const misc_pads[] = {
	/* FPGA_FLASH_SELECT */
	MX6_PAD_NAND_RE_B__GPIO4_IO00 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	/* DETECT_94-120 */
	MX6_PAD_NAND_WE_B__GPIO4_IO01 | MUX_PAD_CTRL(MISC_PAD_CTRL),
	/* SYS_RESET */
	MX6_PAD_LCD_DATA17__GPIO3_IO22 | MUX_PAD_CTRL(MISC_PAD_PD_CTRL),
	/* FPGA_RESET */
	MX6_PAD_JTAG_TDI__GPIO1_IO13 | MUX_PAD_CTRL(MISC_PAD_CTRL),
};

#define SYS_RESET_GPIO	IMX_GPIO_NR(3, 22)

static void early_init(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	gpio_request(SYS_RESET_GPIO, "sys-reset");
	gpio_direction_output(SYS_RESET_GPIO, 0);
	mdelay(10);
	gpio_set_value(SYS_RESET_GPIO, 1);
}

#define FPGA_RESET_GPIO	IMX_GPIO_NR(1, 13)
void board_setup_eim(void)
{
	struct weim *weim_regs = (struct weim *)WEIM_BASE_ADDR;
	struct mxc_ccm_reg *const imx_ccm =
		(struct mxc_ccm_reg *)CCM_BASE_ADDR;
	int i;

	/* select PLL2 PFD2 for EIM, and set divider to divide-by-4, to yield
	 * a 99MHz clock */
	clrsetbits_le32(&imx_ccm->cscmr1,
			MXC_CCM_CSCMR1_ACLK_EMI_SLOW_MASK |
			MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_MASK,
			2 << MXC_CCM_CSCMR1_ACLK_EMI_SLOW_OFFSET |
			4 << MXC_CCM_CSCMR1_ACLK_EMI_SLOW_PODF_OFFSET);

	/* Set up EIM bus for FPGA */
	imx_iomux_v3_setup_multiple_pads(
		eim_pins, ARRAY_SIZE(eim_pins));

	writel(0x0161030F, &weim_regs->cs0gcr1);
	writel(0x00000000, &weim_regs->cs0gcr2);
	writel(0x03000000, &weim_regs->cs0rcr1);
	writel(0x00000000, &weim_regs->cs0rcr2);
	writel(0x01000000, &weim_regs->cs0wcr1);
	writel(0x00000E09, &weim_regs->wcr);

	set_chipselect_size(CS0_128);

	gpio_request(FPGA_RESET_GPIO, "FPGA_RESET");
	gpio_direction_input(FPGA_RESET_GPIO);

	/* Wait for sane fpga, should take 62ms */
	for (i = 0; i < 1000; i++) {
		if(!gpio_get_value(FPGA_RESET_GPIO))
			break;
		mdelay(1);
	}
	/* FPGA needs 10-20ns for FPGA state machines to release themselves. */
	udelay(1);
}

static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000028,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000028,
	.grp_ctlds = 0x00000028,
	.grp_b1ds = 0x00000028,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x000C0000,
};

static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000028,
	.dram_dqm1 = 0x00000028,
	.dram_ras = 0x00000028,
	.dram_cas = 0x00000028,
	.dram_odt0 = 0x00000028,
	.dram_odt1 = 0x00000028,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000028,
	.dram_sdqs0 = 0x00000028,
	.dram_sdqs1 = 0x00000028,
	.dram_reset = 0x00000028,
};

static struct mx6_mmdc_calibration ts7250v3_512m_calibration = {
	.p0_mpwldectrl0 = 0x00000000,
	.p0_mpdgctrl0 = 0x01480148,
	.p0_mprddlctl = 0x40405050,
	.p0_mpwrdlctl = 0x4040504C,
};

static struct mx6_mmdc_calibration ts7250v3_1g_calibration = {
	.p0_mpwldectrl0 = 0x000F0005,
	.p0_mpdgctrl0 = 0x01580154,
	.p0_mprddlctl = 0x40404E52,
	.p0_mpwrdlctl = 0x40404E48,
};

struct mx6_ddr_sysinfo ts7250v3_sysinfo = {
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

static struct mx6_ddr3_cfg ts7250v3_512m_ddr = {
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

static struct mx6_ddr3_cfg ts7250v3_1g_ddr = {
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
	uint32_t reg = readl(FPGA_STRAPS);

	mx6ul_dram_iocfg(16, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	if(reg != 0x4) {
		/* 4Gb Alliance AS4C256M16D3LB */
		mx6_dram_cfg(&ts7250v3_sysinfo,
			     &ts7250v3_512m_calibration,
			     &ts7250v3_512m_ddr);
	} else {
		/* 8Gb Alliance AS4C512M16D3L-12BCN */
		mx6_dram_cfg(&ts7250v3_sysinfo,
			     &ts7250v3_1g_calibration,
			     &ts7250v3_1g_ddr);
	}
}

void board_init_f(ulong dummy)
{
	ccgr_init();
	arch_cpu_init();
	timer_init();
	early_init();
	preloader_console_init();
	board_setup_eim();
	spl_dram_init();
	memset(__bss_start, 0, __bss_end - __bss_start);
	board_init_r(NULL, 0);
}

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC2_BASE_ADDR, 0, 4},
};

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_LCD_DATA18__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_LCD_DATA19__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_LCD_DATA20__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_LCD_DATA21__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_LCD_DATA22__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_LCD_DATA23__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_GPIO1_IO04__GPIO1_IO04 | MUX_PAD_CTRL(MISC_PAD_PD_CTRL),
};

#define USDHC2_PWR_GPIO	IMX_GPIO_NR(1, 4)

int board_mmc_init(bd_t *bis)
{
	int ret;

	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	gpio_request(USDHC2_PWR_GPIO, "sd pwr");
	gpio_direction_output(USDHC2_PWR_GPIO, 0);
	udelay(500);
	gpio_set_value(USDHC2_PWR_GPIO, 1);

	ret = fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
	ret |= fsl_esdhc_initialize(bis, &usdhc_cfg[1]);

	return ret;
}
