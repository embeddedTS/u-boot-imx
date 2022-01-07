/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __FPGA_H__
#define __FPGA_H__

/* FPGA IO */
#define EN_SD_POWER 		14
#define EN_USB_HOST_5V		15
#define EN_OFF_BD_5V		16
#define SD_BOOT_JMPN		36

void fpga_gpio_output(int io, int value);
int fpga_gpio_input(int io);
int fpga_get_rev(void);

#endif // __FPGA_H__