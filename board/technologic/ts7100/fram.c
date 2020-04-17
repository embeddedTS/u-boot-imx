#include <common.h>
#include <spi.h>
#include <command.h>

#include "tsfpga.h"

/* OCSPI Controller */
#define OCSPI_TXRX0		0x0
#define OCSPI_TXRX1		0x4
#define OCSPI_TXRX2		0x8
#define OCSPI_TXRX3		0xc
#define OCSPI_CTRL		0x10
#define OCSPI_DIVIDER		0x14
#define OCSPI_SS		0x18

#define OCSPI_CTRL_ASS		(1 << 13)
#define OCSPI_CTRL_IE		(1 << 12)
#define OCSPI_CTRL_LSB		(1 << 11)
#define OCSPI_CTRL_TX_NEG	(1 << 10)
#define OCSPI_CTRL_RX_NEG	(1 << 9)
#define OCSPI_CTRL_GO_BSY	(1 << 8)
#define OCSPI_CTRL_CHARLEN_MASK	(0x7F)

#define FRAM_WREN	0x6
#define FRAM_WRDI	0x4
#define FRAM_RDSR	0x5
#define FRAM_WRSR	0x1
#define FRAM_READ	0x3
#define FRAM_WRITE	0x2

static void ocspi_setup(int divider, int cpol, int cpha)
{
	uint32_t reg = readl(OCSPI_BASE + OCSPI_CTRL);
	writel(divider, OCSPI_BASE + OCSPI_DIVIDER);
	/* In u-boot this SPI is only used for FRAM on CS0 */
	writel(1, OCSPI_BASE + OCSPI_SS);

	if (cpol) {
		if (cpha) {
			setbits_le32(&reg, OCSPI_CTRL_TX_NEG);
			clrbits_le32(&reg, OCSPI_CTRL_RX_NEG);
		} else {
			clrbits_le32(&reg, OCSPI_CTRL_TX_NEG);
			setbits_le32(&reg, OCSPI_CTRL_RX_NEG);
		}
	} else {
		if (cpha) {
			clrbits_le32(&reg, OCSPI_CTRL_TX_NEG);
			setbits_le32(&reg, OCSPI_CTRL_RX_NEG);
		} else {
			setbits_le32(&reg, OCSPI_CTRL_TX_NEG);
			clrbits_le32(&reg, OCSPI_CTRL_RX_NEG);
		}
	}
	writel(reg, OCSPI_BASE + OCSPI_CTRL);
}

static int ocspi_busy(void)
{
	int timeout = 1000;
	while(readl(OCSPI_BASE + OCSPI_CTRL) & OCSPI_CTRL_GO_BSY) {
		mdelay(1);
		timeout--;
		if(timeout == 0){
			puts("SPI transfer timed out\n");
			return 1;
		}
	}
	return 0;
}

static void fram_wren(void)
{
	uint32_t reg;

	reg = readl(OCSPI_BASE + OCSPI_CTRL);
	reg &= ~OCSPI_CTRL_CHARLEN_MASK;
	reg |= OCSPI_CTRL_ASS | 8;
	writel(reg, OCSPI_BASE + OCSPI_CTRL);
	writel(FRAM_WREN, OCSPI_BASE + OCSPI_TXRX0);
	writel(reg | OCSPI_CTRL_GO_BSY, OCSPI_BASE + OCSPI_CTRL);
	ocspi_busy();
}

uint8_t fram_rdsr(void)
{
	uint32_t reg;
	uint32_t din;

	writel(1, OCSPI_BASE + OCSPI_SS);
	reg = readl(OCSPI_BASE + OCSPI_CTRL);
	reg &= ~OCSPI_CTRL_CHARLEN_MASK;
	reg |= OCSPI_CTRL_ASS | 16;
	writel(reg, OCSPI_BASE + OCSPI_CTRL);
	writel(FRAM_RDSR << 8, OCSPI_BASE + OCSPI_TXRX0);
	writel(reg | OCSPI_CTRL_GO_BSY, OCSPI_BASE + OCSPI_CTRL);
	ocspi_busy();

	din = readl(OCSPI_BASE + OCSPI_TXRX0);

	return (uint8_t)din;
}

void fram_init(void)
{
	/* Set clock speed of 12.375MHz, FRAM supports MODE 0 */
	ocspi_setup(1, 0, 0);
}

void fram_write(uint16_t addr, uint8_t data)
{
	uint32_t reg;
	uint32_t dout;

	fram_wren();

	reg = readl(OCSPI_BASE + OCSPI_CTRL);
	reg &= ~OCSPI_CTRL_CHARLEN_MASK;
	reg |= OCSPI_CTRL_ASS | 32;
	writel(reg, OCSPI_BASE + OCSPI_CTRL);
	dout = FRAM_WRITE << 24;
	dout |= addr << 8;
	dout |= data;
	writel(dout, OCSPI_BASE + OCSPI_TXRX0);
	writel(reg | OCSPI_CTRL_GO_BSY, OCSPI_BASE + OCSPI_CTRL);
	ocspi_busy();
}

uint8_t fram_read(uint16_t addr)
{
	uint32_t reg;
	uint32_t dout;
	uint32_t din;

	reg = readl(OCSPI_BASE + OCSPI_CTRL);
	reg &= ~OCSPI_CTRL_CHARLEN_MASK;
	reg |= OCSPI_CTRL_ASS | 32;
	writel(reg, OCSPI_BASE + OCSPI_CTRL);
	dout = FRAM_READ << 24;
	dout |= addr << 8;
	writel(dout, OCSPI_BASE + OCSPI_TXRX0);
	writel(reg | OCSPI_CTRL_GO_BSY, OCSPI_BASE + OCSPI_CTRL);
	ocspi_busy();

	din = readl(OCSPI_BASE + OCSPI_TXRX0);

	return (uint8_t)din;
}

static int do_fram(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;

	if(argc == 1)
		printf("bootcount=%d\n", fram_read(2047));
	if(argc == 2)
	{
		fram_write(2047, simple_strtoul(argv[1], NULL, 16));
	}
	
	return ret;
}

U_BOOT_CMD(fram, 2, 1,	do_fram,
	"FRAM access",
	"<num> - set boot count\n"
	"No arguments retreives boot count"
);
