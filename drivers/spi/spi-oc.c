// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * Opencore SPI controller
 */

#define LOG_CATEGORY UCLASS_SPI

#include <common.h>
#include <dm.h>
#include <malloc.h>
#include <spi.h>
#include <asm/io.h>
#include <os.h>
#include <clk.h>

#include <linux/errno.h>
#include <dm/device-internal.h>

/* register definitions */
#define SPIOC_RX(i)	(i * 4)
#define SPIOC_TX(i)	(i * 4)
#define SPIOC_CTRL	0x10
#define SPIOC_DIV	0x14
#define SPIOC_SS	0x18

/* SPIOC_CTRL register */
#define CTRL_LEN(x)	((x < 128) ? x : 0)
#define CTRL_BUSY	(1 <<  8)
#define CTRL_RXNEG	(1 <<  9)
#define CTRL_TXNEG	(1 << 10)
#define CTRL_LSB	(1 << 11)
#define CTRL_IE		(1 << 12)
#define CTRL_ASS	(1 << 13)
#define CTRL_CPOL	(1 << 14)

struct spioc {
	void		*regs;		/* base address of the registers */
	u32		freq;
	u32		num_cs;
};

static inline u32 spioc_read(struct spioc *spioc, unsigned long offset)
{
	return readl(spioc->regs + offset);
}

static inline void spioc_write(struct spioc *spioc, unsigned offset,
		u32 value)
{
	writel(value, spioc->regs + offset);
}

static void spioc_chipselect(struct spioc *master, struct dm_spi_slave_platdata *slave)
{
	if (slave) {
		spioc_write(master, SPIOC_SS, 1 << slave->cs);
	} else {
		spioc_write(master, SPIOC_SS, 0);
	}
}

static void spioc_copy_tx(struct spioc *spioc, const void *src, size_t count)
{
	u32 val = 0;
	int i;

	for (i = 0; i < count; i++) {
		int rem = count - i;
		int reg = (rem - 1) / 4;
		int ofs = (rem - 1) % 4;
		//printf("TX: 0x%X=0x%X\n", i, ((u8 *)src)[i]);

		val |= (((u8 *)src)[i] & 0xff) << (ofs * 8);
		if (!ofs) {
			spioc_write(spioc, SPIOC_TX(reg), val);
			val = 0;
		}
	}
}

static void spioc_copy_rx(struct spioc *spioc, void *dest, size_t count)
{
	u32 val = 0;
	int i;

	for (i = 0; i < count; i++) {
		int rem = count - i;
		int reg = (rem - 1) / 4;
		int ofs = (rem - 1) % 4;

		if ((i == 0) || (rem % 4 == 0))
			val = spioc_read(spioc, SPIOC_RX(reg));

		((u8 *)dest)[i] = (val >> (ofs * 8)) & 0xff;
		//printf("RX: 0x%X=0x%X\n", i, ((u8 *)dest)[i]);
	}
}

static int ocspi_busy(struct spioc *spioc)
{
	int timeout = 100000;
	while(spioc_read(spioc, SPIOC_CTRL) & CTRL_BUSY) {
		udelay(10);
		timeout--;
		if(timeout == 0){
			puts("SPI transfer timed out\n");
			return 1;
		}
	}
	return 0;
}

static int ocspi_spi_xfer(struct udevice *dev, unsigned int bitlen,
			    const void *dout, void *din, unsigned long flags)
{
	struct dm_spi_slave_platdata *slave = dev_get_parent_platdata(dev);
	struct udevice *bus = dev->parent;
	struct spioc *spioc = dev_get_priv(bus);
	u32 remaining_len = bitlen / 8;
	u32 pos = 0;
	u32 ctrl;
	const u8 *tx_ptr = dout;
	u8 *rx_ptr = din;

	if (bitlen == 0)
		return 0;

	if (flags & SPI_XFER_BEGIN) {
		spioc_chipselect(spioc, slave);
	}

	while (remaining_len) {
		int xfer_size = min((u32)remaining_len, (u32)16);

		if (tx_ptr)
			spioc_copy_tx(spioc, tx_ptr + pos, xfer_size);

		ctrl = spioc_read(spioc, SPIOC_CTRL);
		ctrl &= ~CTRL_LEN(127);
		ctrl |= CTRL_LEN(xfer_size * 8);
		spioc_write(spioc, SPIOC_CTRL, ctrl);

		/* Start transfer */
		ctrl |= CTRL_BUSY;
		spioc_write(spioc, SPIOC_CTRL, ctrl);
		ocspi_busy(spioc);

		if (rx_ptr)
			spioc_copy_rx(spioc, rx_ptr + pos, xfer_size);

		remaining_len -= xfer_size;
		pos += xfer_size;
	}

	if (flags & SPI_XFER_END)
		spioc_chipselect(spioc, NULL);

	return 0;
}

static int ocspi_spi_set_speed(struct udevice *bus, uint speed)
{
	struct spioc *spioc = dev_get_priv(bus);
	unsigned long clkdiv = 0x0000ffff;

//	/* set the clock divider */
//	clkdiv = DIV_ROUND_UP(spioc->freq,
//			2 * speed) - 1;
//
//	if (clkdiv > 0x0000ffff)
//		clkdiv = 0x0000ffff;

	clkdiv = 3;

	spioc_write(spioc, SPIOC_DIV, clkdiv);

	return 0;
}

static int ocspi_spi_set_mode(struct udevice *bus, uint mode)
{
	struct spioc *spioc = dev_get_priv(bus);
	u32 ctrl = spioc_read(spioc, SPIOC_CTRL);

	/* make sure we're not busy */
	BUG_ON(ctrl & CTRL_BUSY);

	ctrl &= ~(CTRL_RXNEG | CTRL_TXNEG | CTRL_CPOL | CTRL_ASS);

	if (mode == 0) {
		ctrl |=  CTRL_TXNEG;
	} else if (mode == 1) {
		ctrl |=  CTRL_RXNEG;
	} else if (mode == 2) {
		ctrl |=  CTRL_TXNEG | CTRL_CPOL; /* Mode 2 */
	} else if (mode == 3) {
		ctrl |=  CTRL_RXNEG | CTRL_CPOL; /* Mode 3 */
	}

	spioc_write(spioc, SPIOC_CTRL, ctrl);
	spioc_chipselect(spioc, NULL);

	return 0;
}

static int ocspi_spi_cs_info(struct udevice *bus, uint cs,
			      struct spi_cs_info *info)
{
	struct spioc *spioc = dev_get_priv(bus);

	if (cs >= spioc->num_cs)
		return -EINVAL;

	return 0;
}

static int ocspi_spi_probe(struct udevice *bus)
{
	struct spioc *spioc = dev_get_priv(bus);

	spioc->regs = (void *)(ulong)dev_remap_addr(bus);
	if (!spioc->regs)
		return -ENODEV;

	spioc->num_cs = dev_read_u32_default(bus,
					       "opencores-spi,num-chipselects",
					       8);

	spioc->freq = 99000000;

	return 0;
}

static const struct dm_spi_ops ocspi_spi_ops = {
	.xfer		= ocspi_spi_xfer,
	.set_speed	= ocspi_spi_set_speed,
	.set_mode	= ocspi_spi_set_mode,
	.cs_info        = ocspi_spi_cs_info,
};

static const struct udevice_id ocspi_spi_ids[] = {
	{ .compatible = "opencores,spi-oc" },
	{ }
};

U_BOOT_DRIVER(ocspi) = {
	.name     = "ocspi",
	.id       = UCLASS_SPI,
	.of_match = ocspi_spi_ids,
	.ops      = &ocspi_spi_ops,
	.priv_auto_alloc_size = sizeof(struct spioc),
	.probe    = ocspi_spi_probe,
};
