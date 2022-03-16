/*
 * Copyright (C) 2016-2022 Technologic Systems dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __FPGA_H__
#define __FPGA_H__

#include <asm/io.h>

#define FPGA_BASE 		(WEIM_ARB_BASE_ADDR)
#define FPGA_BLOCKRAM		(FPGA_BASE + 0x0)
#define FPGA_SYSCON		(FPGA_BASE + 0x4000)

#define FPGA_SCRATCH_REG	(FPGA_BLOCKRAM)
#define FPGA_MODEL		(FPGA_SYSCON + 0x0)
#define FPGA_DIO1_DAT_SET	(FPGA_SYSCON + 0x10)
#define FPGA_DIO1_OE_SET	(FPGA_SYSCON + 0x12)
#define FPGA_DIO1_DAT_CLR	(FPGA_SYSCON + 0x14)
#define FPGA_DIO1_OE_CLR	(FPGA_SYSCON + 0x16)
#define FPGA_DIO2_DAT_SET	(FPGA_SYSCON + 0x40)
#define FPGA_DIO2_OE_SET	(FPGA_SYSCON + 0x42)
#define FPGA_DIO2_DAT_CLR	(FPGA_SYSCON + 0x44)
#define FPGA_DIO2_OE_CLR	(FPGA_SYSCON + 0x46)
#define FPGA_DIO3_DAT_SET	(FPGA_SYSCON + 0x50)
#define FPGA_DIO3_OE_SET	(FPGA_SYSCON + 0x52)
#define FPGA_DIO3_DAT_CLR	(FPGA_SYSCON + 0x54)
#define FPGA_DIO3_OE_CLR	(FPGA_SYSCON + 0x56)

#define FPGA_SPI_0		(FPGA_BASE + 0x180)

#define OCSPI_BASE			0x50000100

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

#define BANK1_DIO_1		(1 << 0)
#define BANK1_DIO_2		(1 << 1)
#define BANK1_DIO_3		(1 << 2)
#define BANK1_DIO_4		(1 << 3)
#define BANK1_DIO_5		(1 << 4)
#define BANK1_DIO_6		(1 << 5)
#define BANK1_DIO_7		(1 << 6)
#define BANK1_DIO_8		(1 << 7)
#define BANK1_DIO_9		(1 << 8)
#define BANK1_DIO_10		(1 << 9)
#define BANK1_DIO_11		(1 << 10)
#define BANK1_DIO_12		(1 << 11)
#define BANK1_SILAB_CLK		(1 << 12)
#define BANK1_SILAB_DATA	(1 << 13)
#define BANK1_DIO_14		(1 << 14)
#define BANK1_DIO_15		(1 << 15)

#define BANK2_EN_EMMC_3V3N	(1 << 0)
#define BANK2_EN_ADC1_12V	(1 << 1)
#define BANK2_EN_ADC2_12V	(1 << 2)
#define BANK2_EN_ADC3_12V	(1 << 3)
#define BANK2_EN_ADC4_12V	(1 << 4)
#define BANK2_EN_USB_HOST_5V	(1 << 4)
#define BANK2_PHY_RESETN	(1 << 6)
#define BANK2_WIFI_RESETN	(1 << 7)
#define BANK2_RED_LEDN		(1 << 8)
#define BANK2_GREEN_LEDN	(1 << 9)
#define BANK2_RESERVED10	(1 << 10)
#define BANK2_RESERVED11	(1 << 11)
#define BANK2_EN_PROG_SILAB	(1 << 12)
#define BANK2_IO_BD_PRESENT	(1 << 13)

#define BANK3_CPU_STRAP0	(1 << 2)
#define BANK3_CPU_STRAP1	(1 << 3)
#define BANK3_CPU_STRAP2	(1 << 4)
#define BANK3_CPU_STRAP3	(1 << 5)
#define BANK3_DIO_15		(1 << 6)
#define BANK3_DIO_16		(1 << 7)
#define BANK3_DIO_17		(1 << 8)
#define BANK3_DIO_18		(1 << 9)
#define BANK3_BREAKER_TIPPED	(1 << 10)
#define BANK3_CPU_STRAP4	(1 << 11)
#define BANK3_CPU_STRAP5	(1 << 12)
#define BANK3_SILAB_CLK_STB	(1 << 13)

/* Output Enables */
#define fpga_dio1_oe_set(value) writew(value, FPGA_DIO1_OE_SET)
#define fpga_dio1_oe_clr(value) writew(value, FPGA_DIO1_OE_CLR)
#define fpga_dio2_oe_set(value) writew(value, FPGA_DIO2_OE_SET)
#define fpga_dio2_oe_clr(value) writew(value, FPGA_DIO2_OE_CLR)
#define fpga_dio3_oe_set(value) writew(value, FPGA_DIO3_OE_SET)
#define fpga_dio3_oe_clr(value) writew(value, FPGA_DIO3_OE_CLR)

/* Output Data */
#define fpga_dio1_dat_set(value) writew(value, FPGA_DIO1_DAT_SET)
#define fpga_dio1_dat_clr(value) writew(value, FPGA_DIO1_DAT_CLR)
#define fpga_dio2_dat_set(value) writew(value, FPGA_DIO2_DAT_SET)
#define fpga_dio2_dat_clr(value) writew(value, FPGA_DIO2_DAT_CLR)
#define fpga_dio3_dat_set(value) writew(value, FPGA_DIO3_DAT_SET)
#define fpga_dio3_dat_clr(value) writew(value, FPGA_DIO3_DAT_CLR)

/* Input Data */
#define fpga_dio1_data_get(pin) !!(readw(FPGA_DIO1_DAT_SET) & pin)
#define fpga_dio2_data_get(pin) !!(readw(FPGA_DIO2_DAT_SET) & pin)
#define fpga_dio3_data_get(pin) !!(readw(FPGA_DIO3_DAT_SET) & pin)

#endif // __FPGA_H__
