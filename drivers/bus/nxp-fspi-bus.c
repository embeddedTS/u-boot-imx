// SPDX-License-Identifier: GPL-2.0+
/*
 * NXP FlexSPI(FSPI) controller driver.
 *
 * Copyright (c) 2019 Michael Walle <michael@walle.cc>
 * Copyright (c) 2019 NXP
 *
 * This driver was originally ported from the linux kernel v5.4-rc3, which had
 * the following notes:
 *
 * FlexSPI is a flexsible SPI host controller which supports two SPI
 * channels and up to 4 external devices. Each channel supports
 * Single/Dual/Quad/Octal mode data transfer (1/2/4/8 bidirectional
 * data lines).
 *
 * FlexSPI controller is driven by the LUT(Look-up Table) registers
 * LUT registers are a look-up-table for sequences of instructions.
 * A valid sequence consists of four LUT registers.
 * Maximum 32 LUT sequences can be programmed simultaneously.
 *
 * LUTs are being created at run-time based on the commands passed
 * from the spi-mem framework, thus using single LUT index.
 *
 * Software triggered Flash read/write access by IP Bus.
 *
 * Memory mapped read access by AHB Bus.
 *
 * Based on SPI MEM interface and spi-fsl-qspi.c driver.
 *
 * Author:
 *     Yogesh Narayan Gaur <yogeshnarayan.gaur@nxp.com>
 *     Boris Brezillon <bbrezillon@kernel.org>
 *     Frieder Schrempf <frieder.schrempf@kontron.de>
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <malloc.h>
#include <spi.h>
#include <spi-mem.h>
#include <asm/io.h>

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/iopoll.h>
#include <linux/bug.h>
#include <linux/err.h>
#include <linux/delay.h>

/*
 * The driver only uses one single LUT entry, that is updated on
 * each call of exec_op(). Index 0 is preset at boot with a basic
 * read operation, so let's use the middle entry (15).
 * for some platforms, this is the last entry
 */
#define	SEQID_LUT			15
#define	SEQID_AHB_LUT			14

/* Registers used by the driver */
#define FSPI_MCR0			0x00
#define FSPI_MCR0_AHB_TIMEOUT(x)	((x) << 24)
#define FSPI_MCR0_IP_TIMEOUT(x)		((x) << 16)
#define FSPI_MCR0_LEARN_EN		BIT(15)
#define FSPI_MCR0_SCRFRUN_EN		BIT(14)
#define FSPI_MCR0_OCTCOMB_EN		BIT(13)
#define FSPI_MCR0_DOZE_EN		BIT(12)
#define FSPI_MCR0_HSEN			BIT(11)
#define FSPI_MCR0_SERCLKDIV		BIT(8)
#define FSPI_MCR0_ATDF_EN		BIT(7)
#define FSPI_MCR0_ARDF_EN		BIT(6)
#define FSPI_MCR0_RXCLKSRC(x)		((x) << 4)
#define FSPI_MCR0_END_CFG(x)		((x) << 2)
#define FSPI_MCR0_MDIS			BIT(1)
#define FSPI_MCR0_SWRST			BIT(0)

#define FSPI_MCR1			0x04
#define FSPI_MCR1_SEQ_TIMEOUT(x)	((x) << 16)
#define FSPI_MCR1_AHB_TIMEOUT(x)	(x)

#define FSPI_MCR2			0x08
#define FSPI_MCR2_IDLE_WAIT(x)		((x) << 24)
#define FSPI_MCR2_SAMEDEVICEEN		BIT(15)
#define FSPI_MCR2_CLRLRPHS		BIT(14)
#define FSPI_MCR2_CLRAHBBUFOPT		BIT(11)
#define FSPI_MCR2_ABRDATSZ		BIT(8)
#define FSPI_MCR2_ABRLEARN		BIT(7)
#define FSPI_MCR2_ABR_READ		BIT(6)
#define FSPI_MCR2_ABRWRITE		BIT(5)
#define FSPI_MCR2_ABRDUMMY		BIT(4)
#define FSPI_MCR2_ABR_MODE		BIT(3)
#define FSPI_MCR2_ABRCADDR		BIT(2)
#define FSPI_MCR2_ABRRADDR		BIT(1)
#define FSPI_MCR2_ABR_CMD		BIT(0)

#define FSPI_AHBCR			0x0c
#define FSPI_AHBCR_RDADDROPT		BIT(6)
#define FSPI_AHBCR_PREF_EN		BIT(5)
#define FSPI_AHBCR_BUFF_EN		BIT(4)
#define FSPI_AHBCR_CACH_EN		BIT(3)
#define FSPI_AHBCR_CLRTXBUF		BIT(2)
#define FSPI_AHBCR_CLRRXBUF		BIT(1)
#define FSPI_AHBCR_PAR_EN		BIT(0)

#define FSPI_INTEN			0x10
#define FSPI_INTEN_SCLKSBWR		BIT(9)
#define FSPI_INTEN_SCLKSBRD		BIT(8)
#define FSPI_INTEN_DATALRNFL		BIT(7)
#define FSPI_INTEN_IPTXWE		BIT(6)
#define FSPI_INTEN_IPRXWA		BIT(5)
#define FSPI_INTEN_AHBCMDERR		BIT(4)
#define FSPI_INTEN_IPCMDERR		BIT(3)
#define FSPI_INTEN_AHBCMDGE		BIT(2)
#define FSPI_INTEN_IPCMDGE		BIT(1)
#define FSPI_INTEN_IPCMDDONE		BIT(0)

#define FSPI_INTR			0x14
#define FSPI_INTR_SCLKSBWR		BIT(9)
#define FSPI_INTR_SCLKSBRD		BIT(8)
#define FSPI_INTR_DATALRNFL		BIT(7)
#define FSPI_INTR_IPTXWE		BIT(6)
#define FSPI_INTR_IPRXWA		BIT(5)
#define FSPI_INTR_AHBCMDERR		BIT(4)
#define FSPI_INTR_IPCMDERR		BIT(3)
#define FSPI_INTR_AHBCMDGE		BIT(2)
#define FSPI_INTR_IPCMDGE		BIT(1)
#define FSPI_INTR_IPCMDDONE		BIT(0)

#define FSPI_LUTKEY			0x18
#define FSPI_LUTKEY_VALUE		0x5AF05AF0

#define FSPI_LCKCR			0x1C

#define FSPI_LCKER_LOCK			0x1
#define FSPI_LCKER_UNLOCK		0x2

#define FSPI_BUFXCR_INVALID_MSTRID	0xE
#define FSPI_AHBRX_BUF0CR0		0x20
#define FSPI_AHBRX_BUF1CR0		0x24
#define FSPI_AHBRX_BUF2CR0		0x28
#define FSPI_AHBRX_BUF3CR0		0x2C
#define FSPI_AHBRX_BUF4CR0		0x30
#define FSPI_AHBRX_BUF5CR0		0x34
#define FSPI_AHBRX_BUF6CR0		0x38
#define FSPI_AHBRX_BUF7CR0		0x3C
#define FSPI_AHBRXBUF0CR7_MSTRID(x)	(((u32)(x) << 16))
#define FSPI_AHBRXBUF0CR7_BUFSZ(x)	(((u32)(x) << 0))
#define FSPI_AHBRXBUF0CR7_PREF		BIT(31)

#define FSPI_AHBRX_BUF0CR1		0x40
#define FSPI_AHBRX_BUF1CR1		0x44
#define FSPI_AHBRX_BUF2CR1		0x48
#define FSPI_AHBRX_BUF3CR1		0x4C
#define FSPI_AHBRX_BUF4CR1		0x50
#define FSPI_AHBRX_BUF5CR1		0x54
#define FSPI_AHBRX_BUF6CR1		0x58
#define FSPI_AHBRX_BUF7CR1		0x5C

#define FSPI_FLSHA1CR0			0x60
#define FSPI_FLSHA2CR0			0x64
#define FSPI_FLSHB1CR0			0x68
#define FSPI_FLSHB2CR0			0x6C
#define FSPI_FLSHXCR0_SZ_KB		10
#define FSPI_FLSHXCR0_SZ(x)		((x) >> FSPI_FLSHXCR0_SZ_KB)

#define FSPI_FLSHA1CR1			0x70
#define FSPI_FLSHA2CR1			0x74
#define FSPI_FLSHB1CR1			0x78
#define FSPI_FLSHB2CR1			0x7C
#define FSPI_FLSHXCR1_CSINTR(x)		((x) << 16)
#define FSPI_FLSHXCR1_CAS(x)		((x) << 11)
#define FSPI_FLSHXCR1_WA		BIT(10)
#define FSPI_FLSHXCR1_TCSH(x)		((x) << 5)
#define FSPI_FLSHXCR1_TCSS(x)		(x)

#define FSPI_FLSHA1CR2			0x80
#define FSPI_FLSHA2CR2			0x84
#define FSPI_FLSHB1CR2			0x88
#define FSPI_FLSHB2CR2			0x8C
#define FSPI_FLSHXCR2_CLRINSP		BIT(24)
#define FSPI_FLSHXCR2_AWRWAIT		BIT(16)
#define FSPI_FLSHXCR2_AWRSEQN_SHIFT	13
#define FSPI_FLSHXCR2_AWRSEQI_SHIFT	8
#define FSPI_FLSHXCR2_ARDSEQN_SHIFT	5
#define FSPI_FLSHXCR2_ARDSEQI_SHIFT	0

#define FSPI_FLSHACR4			0x94

#define FSPI_IPCR0			0xA0

#define FSPI_IPCR1			0xA4
#define FSPI_IPCR1_IPAREN		BIT(31)
#define FSPI_IPCR1_SEQNUM_SHIFT		24
#define FSPI_IPCR1_SEQID_SHIFT		16
#define FSPI_IPCR1_IDATSZ(x)		(x)

#define FSPI_IPCMD			0xB0
#define FSPI_IPCMD_TRG			BIT(0)

#define FSPI_DLPR			0xB4

#define FSPI_IPRXFCR			0xB8
#define FSPI_IPRXFCR_CLR		BIT(0)
#define FSPI_IPRXFCR_DMA_EN		BIT(1)
#define FSPI_IPRXFCR_WMRK(x)		((x) << 2)

#define FSPI_IPTXFCR			0xBC
#define FSPI_IPTXFCR_CLR		BIT(0)
#define FSPI_IPTXFCR_DMA_EN		BIT(1)
#define FSPI_IPTXFCR_WMRK(x)		((x) << 2)

#define FSPI_DLLACR			0xC0
#define FSPI_DLLACR_OVRDEN		BIT(8)
#define FSPI_DLLACR_SLVDLY(x)          ((x) << 3)
#define FSPI_DLLACR_DLLRESET		BIT(1)
#define FSPI_DLLACR_DLLEN              BIT(0)

#define FSPI_DLLBCR			0xC4
#define FSPI_DLLBCR_OVRDEN		BIT(8)
#define FSPI_DLLBCR_SLVDLY(x)          ((x) << 3)
#define FSPI_DLLBCR_DLLRESET		BIT(1)
#define FSPI_DLLBCR_DLLEN              BIT(0)

#define FSPI_STS0			0xE0
#define FSPI_STS0_DLPHB(x)		((x) << 8)
#define FSPI_STS0_DLPHA(x)		((x) << 4)
#define FSPI_STS0_CMD_SRC(x)		((x) << 2)
#define FSPI_STS0_ARB_IDLE		BIT(1)
#define FSPI_STS0_SEQ_IDLE		BIT(0)

#define FSPI_STS1			0xE4
#define FSPI_STS1_IP_ERRCD(x)		((x) << 24)
#define FSPI_STS1_IP_ERRID(x)		((x) << 16)
#define FSPI_STS1_AHB_ERRCD(x)		((x) << 8)
#define FSPI_STS1_AHB_ERRID(x)		(x)

#define FSPI_STS2			0xE8
#define FSPI_STS2_BREFLOCK		BIT(17)
#define FSPI_STS2_BSLVLOCK		BIT(16)
#define FSPI_STS2_AREFLOCK		BIT(1)
#define FSPI_STS2_ASLVLOCK		BIT(0)
#define FSPI_STS2_AB_LOCK		(FSPI_STS2_BREFLOCK | \
					 FSPI_STS2_BSLVLOCK | \
					 FSPI_STS2_AREFLOCK | \
					 FSPI_STS2_ASLVLOCK)

#define FSPI_AHBSPNST			0xEC
#define FSPI_AHBSPNST_DATLFT(x)		((x) << 16)
#define FSPI_AHBSPNST_BUFID(x)		((x) << 1)
#define FSPI_AHBSPNST_ACTIVE		BIT(0)

#define FSPI_IPRXFSTS			0xF0
#define FSPI_IPRXFSTS_RDCNTR(x)		((x) << 16)
#define FSPI_IPRXFSTS_FILL(x)		(x)

#define FSPI_IPTXFSTS			0xF4
#define FSPI_IPTXFSTS_WRCNTR(x)		((x) << 16)
#define FSPI_IPTXFSTS_FILL(x)		(x)

#define FSPI_RFDR			0x100
#define FSPI_TFDR			0x180

#define FSPI_LUT_BASE			0x200
#define FSPI_LUT_OFFSET			(SEQID_LUT * 4 * 4)
#define FSPI_LUT_REG(idx) \
	(FSPI_LUT_BASE + FSPI_LUT_OFFSET + (idx) * 4)

#define FSPI_AHB_LUT_OFFSET			(SEQID_AHB_LUT * 4 * 4)
#define FSPI_AHB_LUT_REG(idx) \
	(FSPI_LUT_BASE + FSPI_AHB_LUT_OFFSET + (idx) * 4)

/* register map end */

/* Instruction set for the LUT register. */
#define LUT_STOP			0x00
#define LUT_CMD				0x01
#define LUT_ADDR			0x02
#define LUT_CADDR_SDR			0x03
#define LUT_MODE			0x04
#define LUT_MODE2			0x05
#define LUT_MODE4			0x06
#define LUT_MODE8			0x07
#define LUT_NXP_WRITE			0x08
#define LUT_NXP_READ			0x09
#define LUT_LEARN_SDR			0x0A
#define LUT_DATSZ_SDR			0x0B
#define LUT_DUMMY			0x0C
#define LUT_DUMMY_RWDS_SDR		0x0D
#define LUT_JMP_ON_CS			0x1F
#define LUT_CMD_DDR			0x21
#define LUT_ADDR_DDR			0x22
#define LUT_CADDR_DDR			0x23
#define LUT_MODE_DDR			0x24
#define LUT_MODE2_DDR			0x25
#define LUT_MODE4_DDR			0x26
#define LUT_MODE8_DDR			0x27
#define LUT_WRITE_DDR			0x28
#define LUT_READ_DDR			0x29
#define LUT_LEARN_DDR			0x2A
#define LUT_DATSZ_DDR			0x2B
#define LUT_DUMMY_DDR			0x2C
#define LUT_DUMMY_RWDS_DDR		0x2D

/*
 * Calculate number of required PAD bits for LUT register.
 *
 * The pad stands for the number of IO lines [0:7].
 * For example, the octal read needs eight IO lines,
 * so you should use LUT_PAD(8). This macro
 * returns 3 i.e. use eight (2^3) IP lines for read.
 */
#define LUT_PAD(x) (fls(x) - 1)

/*
 * Macro for constructing the LUT entries with the following
 * register layout:
 *
 *  ---------------------------------------------------
 *  | INSTR1 | PAD1 | OPRND1 | INSTR0 | PAD0 | OPRND0 |
 *  ---------------------------------------------------
 */
#define PAD_SHIFT		8
#define INSTR_SHIFT		10
#define OPRND_SHIFT		16

/* Macros for constructing the LUT register. */
#define LUT_DEF(idx, ins, pad, opr)			  \
	((((ins) << INSTR_SHIFT) | ((pad) << PAD_SHIFT) | \
	(opr)) << (((idx) % 2) * OPRND_SHIFT))

#define POLL_TOUT		5000
#define NXP_FSPI_MAX_CHIPSELECT		4
#define MHZ(x)	((x) * 1000000UL)

/* Access flash memory using IP bus only */
#define FSPI_QUIRK_USE_IP_ONLY		BIT(0)

struct nxp_fspi {
	struct udevice *dev;
	void __iomem *iobase;
	void __iomem *ahb_addr;
	u32 memmap_phy;
	u32 memmap_phy_size;
	bool ahb_bus_only;
	int ahb_clock_rate;
	int selected;
	struct clk clk, clk_en;
#define FSPI_DTR_ODD_ADDR       (1 << 0)
#define FSPI_RXCLKSRC_3		(1 << 1)
	int flags;
};

/*
 * R/W functions for big- or little-endian registers:
 * The FSPI controller's endianness is independent of
 * the CPU core's endianness. So far, although the CPU
 * core is little-endian the FSPI controller can use
 * big-endian or little-endian.
 */
static void fspi_writel(struct nxp_fspi *f, u32 val, void __iomem *addr)
{
	out_le32(addr, val);
}

static u32 fspi_readl(struct nxp_fspi *f, void __iomem *addr)
{
	return in_le32(addr);
}

/* Instead of busy looping invoke readl_poll_sleep_timeout functionality. */
static int fspi_readl_poll_tout(struct nxp_fspi *f, void __iomem *base,
				u32 mask, u32 delay_us,
				u32 timeout_us, bool c)
{
	u32 reg;

	if (c)
		return readl_poll_sleep_timeout(base, reg, (reg & mask),
						delay_us, timeout_us);
	else
		return readl_poll_sleep_timeout(base, reg, !(reg & mask),
						delay_us, timeout_us);
}

#if CONFIG_IS_ENABLED(CLK)
static int nxp_fspi_clk_prep_enable(struct nxp_fspi *f)
{
	int ret;

	ret = clk_enable(&f->clk_en);
	if (ret)
		return ret;

	ret = clk_enable(&f->clk);
	if (ret) {
		clk_disable(&f->clk_en);
		return ret;
	}

	return 0;
}

static void nxp_fspi_clk_disable_unprep(struct nxp_fspi *f)
{
	clk_disable(&f->clk);
	clk_disable(&f->clk_en);
}
#endif

static int nxp_fspi_default_setup(struct nxp_fspi *f)
{
	void __iomem *base = f->iobase;
	int ret, i;
	u32 reg;

#if CONFIG_IS_ENABLED(CLK)
	/* the default frequency, we will change it later if necessary. */
	ret = clk_set_rate(&f->clk, 20000000);
	if (ret < 0)
		return ret;

	ret = nxp_fspi_clk_prep_enable(f);
	if (ret)
		return ret;
#endif

#ifdef CONFIG_FSL_LAYERSCAPE
	/*
	 * ERR050568: Flash access by FlexSPI AHB command may not work with
	 * platform frequency equal to 300 MHz on LS1028A.
	 * LS1028A reuses LX2160A compatible entry. Make errata applicable for
	 * Layerscape LS1028A platform family.
	 */
	if (device_is_compatible(f->dev, "nxp,lx2160a-fspi"))
		erratum_err050568(f);
#endif

	/* Reset the module */
	/* w1c register, wait unit clear */
	ret = fspi_readl_poll_tout(f, f->iobase + FSPI_MCR0,
				   FSPI_MCR0_SWRST, 0, POLL_TOUT, false);
	WARN_ON(ret);

	/* Disable the module */
	fspi_writel(f, FSPI_MCR0_MDIS, base + FSPI_MCR0);

	/* Reset the DLL register to default value */
	fspi_writel(f, FSPI_DLLACR_OVRDEN, base + FSPI_DLLACR);
	fspi_writel(f, FSPI_DLLBCR_OVRDEN, base + FSPI_DLLBCR);

	/* enable module */
	fspi_writel(f, (u32)FSPI_MCR0_AHB_TIMEOUT(0xFF) |
		    FSPI_MCR0_IP_TIMEOUT(0xFF) | FSPI_MCR0_OCTCOMB_EN,
		    base + FSPI_MCR0);

	/*
	 * Disable same device enable bit and configure all slave devices
	 * independently.
	 */
	reg = fspi_readl(f, f->iobase + FSPI_MCR2);
	reg = reg & ~(FSPI_MCR2_SAMEDEVICEEN);
	fspi_writel(f, reg, base + FSPI_MCR2);

	/* AHB configuration for access buffer 0~7. */
	for (i = 0; i < 7; i++)
		fspi_writel(f, 0, base + FSPI_AHBRX_BUF0CR0 + 4 * i);

	/*
	 * Set ADATSZ with the maximum AHB buffer size to improve the read
	 * performance.
	 */
	fspi_writel(f, (SZ_2K / 8 | FSPI_AHBRXBUF0CR7_PREF),
		    base + FSPI_AHBRX_BUF7CR0);

	/* prefetch and no start address alignment limitation */
	fspi_writel(f, FSPI_AHBCR_PREF_EN | FSPI_AHBCR_RDADDROPT,
		    base + FSPI_AHBCR);

	/* Reset the flashx control1 registers */
	reg = FSPI_FLSHXCR1_TCSH(0x3) | FSPI_FLSHXCR1_TCSS(0x3);
	fspi_writel(f, reg, base + FSPI_FLSHA1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHA2CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB2CR1);

	/* AHB Read - Set lut sequence ID for all CS. */
	fspi_writel(f, SEQID_AHB_LUT, base + FSPI_FLSHA1CR2);
	fspi_writel(f, SEQID_AHB_LUT, base + FSPI_FLSHA2CR2);
	fspi_writel(f, SEQID_AHB_LUT, base + FSPI_FLSHB1CR2);
	fspi_writel(f, SEQID_AHB_LUT, base + FSPI_FLSHB2CR2);

	f->selected = -1;

	return 0;
}

static int nxp_fspi_ahb_bus_setup(struct nxp_fspi *f)
{
	const int cs = 0;
	const int pads = 4;

	void __iomem *base = f->iobase;
	uint64_t size_kb;
	u32 seq0[4];
	u32 seq1[4];
	u32 reg, mcr0;
	int ret;
	int i;

	memset(&seq0, 0, sizeof(seq0));
	memset(&seq1, 0, sizeof(seq1));

	/* Reset peripheral */
	ret = fspi_readl_poll_tout(f, f->iobase + FSPI_MCR0,
				   FSPI_MCR0_SWRST, 0, POLL_TOUT, false);
	if (ret)
		return ret;

	nxp_fspi_clk_disable_unprep(f);

	ret = clk_set_rate(&f->clk, 66666666);
	if (ret < 0)
		return ret;

	ret = nxp_fspi_clk_prep_enable(f);
	if (ret)
		return ret;

	/* Reset the module */
	/* w1c register, wait unit clear */
	ret = fspi_readl_poll_tout(f, f->iobase + FSPI_MCR0,
				   FSPI_MCR0_SWRST, 0, POLL_TOUT, false);
	WARN_ON(ret);

	/* Disable the module */
	fspi_writel(f, FSPI_MCR0_MDIS, base + FSPI_MCR0);

	/* Setup MCR0 config */
	mcr0 = (u32) (FSPI_MCR0_RXCLKSRC(0x0) |
		      FSPI_MCR0_DOZE_EN |
		      FSPI_MCR0_SCRFRUN_EN |
		      FSPI_MCR0_IP_TIMEOUT(0xFF) |
		      FSPI_MCR0_AHB_TIMEOUT(0xFF));

	fspi_writel(f, mcr0, base + FSPI_MCR0);

	/* Setup MCR1 config */
	reg = FSPI_MCR1_SEQ_TIMEOUT(0xFFFF) |
	      FSPI_MCR1_AHB_TIMEOUT(0xFFFF);
	fspi_writel(f, reg, base + FSPI_MCR1);

	/* Setup MCR2 config */
	reg = FSPI_MCR2_IDLE_WAIT(20) |
	      FSPI_MCR2_CLRAHBBUFOPT;
	fspi_writel(f, reg, base + FSPI_MCR2);

	/* Setup AHB config */
	reg = FSPI_AHBCR_BUFF_EN;
	fspi_writel(f, reg, base + FSPI_AHBCR);

	/* Set up ahb buffers */
	reg = FSPI_AHBRXBUF0CR7_MSTRID(FSPI_BUFXCR_INVALID_MSTRID);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF0CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF1CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF2CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF3CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF4CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF5CR0);
	fspi_writel(f, reg, base + FSPI_AHBRX_BUF6CR0);

	reg = FSPI_AHBRXBUF0CR7_BUFSZ(1);// in units of 64-bits // Max burst size is 32-bytes
	fspi_writel(f, 0, base + FSPI_AHBRX_BUF7CR0);

	/* Reset FLSHxxCR0 registers */
	fspi_writel(f, 0, f->iobase + FSPI_FLSHA1CR0);
	fspi_writel(f, 0, f->iobase + FSPI_FLSHA2CR0);
	fspi_writel(f, 0, f->iobase + FSPI_FLSHB1CR0);
	fspi_writel(f, 0, f->iobase + FSPI_FLSHB2CR0);

	/* Assign controller memory mapped space as size, KBytes, of flash. */
	size_kb = FSPI_FLSHXCR0_SZ(f->memmap_phy_size);

	fspi_writel(f, size_kb, f->iobase + FSPI_FLSHA1CR0 + 4 * cs);

	/* Set bus parameters */
	reg = FSPI_FLSHXCR1_CSINTR(2) | /* 2 is minimum */
	      FSPI_FLSHXCR1_TCSH(0) |
	      FSPI_FLSHXCR1_TCSS(0);
	fspi_writel(f, reg, base + FSPI_FLSHA1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHA2CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB1CR1);
	fspi_writel(f, reg, base + FSPI_FLSHB2CR1);

	reg = fspi_readl(f, base + FSPI_FLSHA1CR2) & 0xffff0000;
	reg |= (0 << FSPI_FLSHXCR2_AWRSEQN_SHIFT) |
	      (1 << FSPI_FLSHXCR2_AWRSEQI_SHIFT) |
	      (0 << FSPI_FLSHXCR2_ARDSEQI_SHIFT) |
	      (0 << FSPI_FLSHXCR2_ARDSEQN_SHIFT);
	fspi_writel(f, reg, base + FSPI_FLSHA1CR2);

	reg = 0x1;
	fspi_writel(f, reg, base + FSPI_FLSHACR4);

	/* Magic number, finish testing and see what we need here*/
	/*fspi_writel(f, FSPI_DLLACR_DLLEN | FSPI_DLLACR_SLVDLY(0x37),
		    f->iobase + FSPI_DLLACR);*/
	fspi_writel(f, FSPI_DLLACR_DLLEN | FSPI_DLLACR_SLVDLY(0xf),
		    f->iobase + FSPI_DLLACR);

	/*
	 * LUT Sequence 0 is a DDR Read.
	 * This issues on the bus:
	 * [7:0] "0x9" (identifies ddr read)
	 * [15:0] address
	 * 8 dummy clocks
	 * [(8*n)-1:0] read data
	 */
	seq0[0] = LUT_DEF(0, LUT_CMD, LUT_PAD(pads), LUT_NXP_READ) |
	          LUT_DEF(1, LUT_ADDR, LUT_PAD(pads), 16);
	seq0[1] = LUT_DEF(2, LUT_DUMMY, LUT_PAD(pads), 8) |
		  LUT_DEF(3, LUT_NXP_READ, LUT_PAD(pads), 0);
	seq1[2] = LUT_DEF(4, LUT_STOP, 0, 0);

	/*
	 * LUT Sequence 1 is a DDR Write. This issues on the bus:
	 * [7:0] "0x8" (identifies write)
	 * [15:0] address
	 * [(8*n)-1:0] write data
	 */
	seq1[0] = LUT_DEF(0, LUT_CMD, LUT_PAD(pads), LUT_NXP_WRITE) |
	          LUT_DEF(1, LUT_ADDR, LUT_PAD(pads), 16);
	seq1[1] = LUT_DEF(2, LUT_NXP_WRITE, LUT_PAD(pads), 0) | 
		  LUT_DEF(3, LUT_STOP, 0, 0);

	/* unlock LUT */
	fspi_writel(f, FSPI_LUTKEY_VALUE, f->iobase + FSPI_LUTKEY);
	fspi_writel(f, FSPI_LCKER_UNLOCK, f->iobase + FSPI_LCKCR);

	/* configure read lut */
	for (i = 0; i < ARRAY_SIZE(seq0); i++)
		fspi_writel(f, seq0[i], base + FSPI_LUT_BASE + (i*4));

	/* configure write lut */
	for (i = 0; i < ARRAY_SIZE(seq1); i++)
		fspi_writel(f, seq1[i], base + FSPI_LUT_BASE + 0x10 + (i*4));

	/* lock LUT */
	fspi_writel(f, FSPI_LUTKEY_VALUE, f->iobase + FSPI_LUTKEY);
	fspi_writel(f, FSPI_LCKER_LOCK, f->iobase + FSPI_LCKCR);

	/* Deassert disable */
	fspi_writel(f, mcr0, base + FSPI_MCR0);

	/* Wait for controller being ready. */
	ret = fspi_readl_poll_tout(f, f->iobase + FSPI_STS0,
				   FSPI_STS0_ARB_IDLE, 1, POLL_TOUT, true);
	if (ret)
		return ret;

	/* Enable additional interrupts */
	fspi_writel(f,
		    FSPI_INTEN_IPCMDDONE |
		    FSPI_INTR_SCLKSBWR |
		    FSPI_INTR_SCLKSBRD |
		    FSPI_INTR_DATALRNFL |
		    FSPI_INTR_IPRXWA |
		    FSPI_INTR_AHBCMDERR |
		    FSPI_INTR_IPCMDERR |
		    FSPI_INTR_AHBCMDGE |
		    FSPI_INTR_IPCMDGE,
		    base + FSPI_INTEN);

	return 0;
}

void __weak board_flexspi_start(void)
{
}

static int nxp_fspi_probe(struct udevice *bus)
{
	struct nxp_fspi *f = dev_get_priv(bus);

	nxp_fspi_default_setup(f);
	nxp_fspi_ahb_bus_setup(f);
	board_flexspi_start();

	return 0;
}

static int nxp_fspi_of_to_plat(struct udevice *bus)
{
	struct nxp_fspi *f = dev_get_priv(bus);
#if CONFIG_IS_ENABLED(CLK)
	int ret;
#endif

	fdt_addr_t iobase;
	fdt_addr_t iobase_size;
	fdt_addr_t ahb_addr;
	fdt_addr_t ahb_size;

	f->dev = bus;

	iobase = devfdt_get_addr_size_name(bus, "fspi_base", &iobase_size);
	if (iobase == FDT_ADDR_T_NONE) {
		dev_err(bus, "fspi_base regs missing\n");
		return -ENODEV;
	}
	f->iobase = map_physmem(iobase, iobase_size, MAP_NOCACHE);

	ahb_addr = devfdt_get_addr_size_name(bus, "fspi_mmap", &ahb_size);
	if (ahb_addr == FDT_ADDR_T_NONE) {
		dev_err(bus, "fspi_mmap regs missing\n");
		return -ENODEV;
	}
	f->ahb_addr = map_physmem(ahb_addr, ahb_size, MAP_NOCACHE);
	f->memmap_phy_size = ahb_size;

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_get_by_name(bus, "fspi_en", &f->clk_en);
	if (ret) {
		dev_err(bus, "failed to get fspi_en clock\n");
		return ret;
	}

	ret = clk_get_by_name(bus, "fspi", &f->clk);
	if (ret) {
		dev_err(bus, "failed to get fspi clock\n");
		return ret;
	}
#endif

	dev_dbg(bus, "iobase=<0x%llx>, ahb_addr=<0x%llx>\n", iobase, ahb_addr);

	return 0;
}


static const struct udevice_id nxp_fspi_ids[] = {
	{ .compatible = "nxp,imx93-fspi-bus", },
	{ }
};

U_BOOT_DRIVER(nxp_fspi_bus) = {
	.name	= "nxp_fspi_bus",
	.id	= UCLASS_SIMPLE_BUS,
	.of_match = nxp_fspi_ids,
	.of_to_plat = nxp_fspi_of_to_plat,
	.priv_auto	= sizeof(struct nxp_fspi),
	.probe	= nxp_fspi_probe,
};
