#pragma once

#define MICRO_ADC_0 0
#define MICRO_ADC_1 2
#define MICRO_ADC_2 4
#define MICRO_ADC_3 6
#define MICRO_ADC_4 8
#define MICRO_ADC_5 10
#define MICRO_ADC_6 12
#define MICRO_ADC_7 14
#define MICRO_ADC_8 16
#define MICRO_ADC_9 18
#define MICRO_ADC_10 20
#define MICRO_STATUS_FLAGS 22
#define MICRO_STATUS_FLAGS_POWER_FAIL (1 << 0)
#define MICRO_STATUS_FLAGS_SCAPS_EN (1 << 1)
#define MICRO_STATUS_FLAGS_SCAPS_MET_MIN (1 << 2)
#define MICRO_STATUS_FLAGS_SCAPS_CHARGING (1 << 3)
#define MICRO_STATUS_FLAGS_USB_PRESENT (1 << 4)
#define MICRO_CHARGE_CURRENT_DEFAULT 24
#define MICRO_CHARGE_CURRENT 26
#define MICRO_CMD 1024
#define MICRO_CMD_SLEEP (1 << 1)
#define MICRO_REVISION 2048
#define MICRO_BUILD_STRING 4096

int micro_read(uint16_t addr, void *data, size_t size);
int micro_write(uint16_t addr, const void *data, size_t size);

#define micro_read8(addr, data) micro_read(addr, (uint8_t *)data, sizeof(uint8_t))
#define micro_write8(addr, data) micro_write(addr, (uint8_t *)data, sizeof(uint8_t))

uint8_t micro_scaps_remaining_pct(void);
void micro_scaps_en(int en);
uint8_t micro_scaps_block_pct(int block_pct);
