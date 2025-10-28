/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __WIZARD_H__
#define __WIZARD_H__

#define WIZARD_I2C_ADDR 0x54
#define WIZARD_SERIAL 34

int wizard_write(u16 addr, u16 value);
int wizard_read(u16 addr, u16 *value);
int wizard_read_mac(uint8_t *mac_buffer);

#endif // __WIZARD_H__
