/*
 * Copyright (C) 2016, 2021-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __TSFPGA_H__
#define __TSFPGA_H__

/* FPGA IO on the TS-7180 */

#define EN_SD_POWER 		14
#define EN_USB_HOST_5V		15
#define EN_OFF_BD_5V		16
#define SD_BOOT_JMPN		36

void fpga_gpio_output(int io, int value);
int fpga_gpio_input(int io);
int fpga_get_rev(void);
void fpga_reset(void);

#endif // __TSFPGA_H__
