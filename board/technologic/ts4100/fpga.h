/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __FPGA_H__
#define __FPGA_H__

/* FPGA IO */
#define EN_USB_HOST_5V_PAD	22
#define OFF_BD_RESET_PADN	25
#define GREEN_LED_PADN		27
#define RED_LED_PADN		28
#define ETH_PHY_RESET		24
#define DIO_20			57
#define DIO_05			42
#define DIO_43			80
#define SPARE1			01
#define DIO_09			46
#define SILAB_CLK		84
#define SILAB_RST		85
#define FORCE_5V		86

void fpga_gpio_output(int io, int value);
int fpga_gpio_input(int io);
int fpga_get_rev(void);

#endif // __FPGA_H__
