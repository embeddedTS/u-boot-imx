#ifndef __PARSE_STRAP_H__
#define __PARSE_STRAP_H__

const char *get_board_model(void);
const char *get_board_name(void);
const char *get_cpu_board_version(void);

u32 get_board_rev(void);

uint8_t parse_strap(const char *);

uint8_t read_cpu_board_opts(void);
uint8_t read_io_board_model(void);
uint8_t read_io_board_opts(void);

uint16_t read_raw_cpu_straps(void);
uint16_t read_raw_fpga_straps(void);
#endif
