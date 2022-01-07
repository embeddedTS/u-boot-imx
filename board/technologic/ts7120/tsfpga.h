/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __FPGA_H__
#define __FPGA_H__

#include <asm/io.h>

#define FPGA_BASE 		(WEIM_ARB_BASE_ADDR)
#define FPGA_BLOCKRAM		(FPGA_BASE + 0x0)
#define FPGA_SYSCON		(FPGA_BASE + 0x4000)

#define FPGA_MODEL		(FPGA_SYSCON + 0x0) /* Currently 0xdeadbeef */
#define FPGA_REV_CRC32		(FPGA_SYSCON + 0x4) /* 32-bit crc32 of 905KB of 0xf0000 offset*/
#define FPGA_ASMI		(FPGA_SYSCON + 0x8) /* Core to access FPGA flash */
#define FPGA_SCRATCH_REG        (FPGA_SYSCON + 0xc) /* scratch register used for bus testing */
#define FPGA_DIO0_DAT_SET	(FPGA_SYSCON + 0x10)
#define FPGA_DIO0_OE_SET	(FPGA_SYSCON + 0x12)
#define FPGA_DIO0_DAT_CLR	(FPGA_SYSCON + 0x14)
#define FPGA_DIO0_OE_CLR	(FPGA_SYSCON + 0x16)
#define FPGA_FRAM_BOOTCOUNT	(FPGA_SYSCON + 0x18)
#define FPGA_RELOAD		(FPGA_SYSCON + 0x1a)
#define FPGA_USEC_CTR		(FPGA_SYSCON + 0x38)

#define FPGA_SPI_0		(FPGA_BASE + 0x180)

#define OCSPI_RX0		0x0
#define OCSPI_RX1		0x4
#define OCSPI_RX2		0x8
#define OCSPI_RX3		0xc
#define OCSPI_TX0		0x0
#define OCSPI_TX1		0x4
#define OCSPI_TX2		0x8
#define OCSPI_TX3		0xc
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

/* Only present in app load */
#define FPGA_UART0		(FPGA_BASE + 0x0)

#define TS16550_THR		0x0
#define TS16550_RBR		0x0
#define TS16550_DLL		0x0
#define TS16550_IER		0x2
#define TS16550_DLH		0x2
#define TS16550_IIR		0x4
#define TS16550_FCR		0x4
#define TS16550_LCR		0x6
#define TS16550_MCR		0x8
#define TS16550_LSR		0xa
#define TS16550_MSR		0xc
#define TS16550_SR		0xe

#define DIO_2_PAD		(1 << 0)
#define DIO_4_PAD		(1 << 1)
#define DIO_5_PAD		(1 << 2)
#define DIO_6_PAD		(1 << 3)
#define DIO_7_PAD		(1 << 4)
#define DIO_8_PAD		(1 << 5)
#define DIO_9_PAD		(1 << 6)
#define DIO_10_PAD		(1 << 7)
#define DIO_11_PAD		(1 << 8)
#define DIO_14_PAD		(1 << 11)
#define FPGA_SILABS_CLK_PADN	(1 << 12)
#define FPGA_SILABS_DAT_PAD	(1 << 13)
#define EN_BLUE_LED_PAD		(1 << 14)
#define SYS_RESET_PADN		(1 << 15)

/* Output Enables */
#define fpga_dio_oe_set(value) writew(value, FPGA_DIO0_OE_SET)
#define fpga_dio_oe_clr(value) writew(value, FPGA_DIO0_OE_CLR)

/* Output Data */
#define fpga_dio_dat_set(value) writew(value, FPGA_DIO0_DAT_SET)
#define fpga_dio_dat_clr(value) writew(value, FPGA_DIO0_DAT_CLR)

/* Input Data */
#define fpga_dio_data_get(pin) !!(readl(FPGA_DIO0_DAT_SET) & pin)

#endif // __FPGA_H__
