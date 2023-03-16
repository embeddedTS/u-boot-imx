#ifndef __SUPER_H__
#define __SUPER_H__

#include <common.h>

#define SUPER_I2C_ADDR 0x10

#define SUPER_MODEL 0
#define SUPER_REV_INFO 1
#define SUPER_ADC_CHAN_ADV 2
#define SUPER_FEATURES0 3
#define SUPER_CMDS 8
#define SUPER_GEN_FLAGS 16
#define SUPER_GEN_INPUTS 24
#define SUPER_REBOOT_REASON 32
#define SUPER_ADC_BASE 128
#define SUPER_TEMPERATURE 160

enum i2c_cmds_t {
	I2C_NOCMD  = ((uint16_t)0 << 0),
	I2C_REBOOT = ((uint16_t)1 << 0),
	I2C_HALT   = ((uint16_t)1 << 1),
};

enum reboot_reasons_t {
    REBOOT_REASON_POR = 0,
    REBOOT_REASON_CPU_WDT = 1,
    REBOOT_REASON_SOFTWARE_REBOOT = 2,
    REBOOT_REASON_BROWNOUT = 3,
    REBOOT_REASON_RTC_ALARM_REBOOT = 4,
    REBOOT_REASON_WAKE_FROM_PWR_CYCLE = 5,
    REBOOT_REASON_WAKE_FROM_WAKE_SIGNAL = 6,
    REBOOT_REASON_WAKE_FROM_RTC_ALARM = 7,
    REBOOT_REASON_WAKE_FROM_USB_VBUS = 8,
};

int super_write(uint16_t addr, uint16_t value);
int super_read(uint16_t addr, uint16_t *value);

#endif // __SUPER_H__
