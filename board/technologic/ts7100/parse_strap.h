#ifndef __PARSE_STRAP_H__
#define __PARSE_STRAP_H__

const char *get_board_name(void);
const char get_cpu_board_version_char(void);
int board_read_straps(uint32_t *cpu_straps, uint32_t *io_opts, uint32_t *io_model);

#endif
