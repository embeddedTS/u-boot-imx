#ifndef __FRAM_H
#define __FRAM_H

uint8_t fram_read(uint16_t addr);
void fram_write(uint16_t addr, uint8_t data);
void fram_init(void);
uint8_t fram_rdsr(void);

#endif 
