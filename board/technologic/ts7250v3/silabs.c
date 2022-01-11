// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020-2022 Technologic Systems, Inc. dba embeddedTS
 */

/* This file only valid on silabs built after Sep 25, 9am, 2019 */

#define MIN_CHARGE_MV 3680
#define MAX_CHARGE_MV 4800
#define DEFAULT_WDOG_MS 60000

int silab_cmd(int argc, char *const argv[]); /* One public func API */

/* clang-format off */
#define SILAB_HELP                                                                 \
"  help                       Print this help\n"                                   \
"  status                     Print human-readable status\n"                       \
"  sleep <N>                  Enter uC sleep mode (low-power) for N millisec\n"    \
"  reboot                     Power cycle and reboot both board and uC\n"          \
"  wdog                       Exits errorlevel 1 if watchdog armed\n"              \
"  wdog expired               Exits errorlevel 1 if last reboot was from wdog\n"   \
"  wdog set <N>               Arms watchdog for N milliseconds\n"                  \
"  wdog feed                  Feeds watchdog\n"                                    \
"  wdog disable               Disables watchdog\n"                                 \
"  scaps                      Exits errorlevel 1 if supercaps enabled\n"           \
"  scaps enable               Turns on supercaps for this boot\n"                  \
"  scaps disable              Turns off supercaps for this boot\n"                 \
"  scaps default enable       Sets supercaps to be default enabled on bootup\n"    \
"  scaps default disable      Sets supercaps to be default disabled on bootup\n"   \
"  scaps current <N>          Sets supercaps charging current in N milliamps\n"    \
"  scaps current default <N>  Sets default charging current to N milliamps\n"      \
"  scaps wait                 Blocks until supercaps reach minimum charge\n"       \
"  scaps wait full            Blocks until supercaps reach max charge\n"           \
"  scaps wait pct <N>         Blocks until scaps reach (max-min)*N/100+min\n"      \
"  scaps pct <N>              Exits errorlevel 1 if scaps at N %\n"                \
"  usb                        Exits errorlevel 1 if USB console connected\n"       \
"  flags <N>                  Exits errorlevel 1 if uC flash flag N set\n"         \
"  flags set <N>              Sets uC flash flag N\n"                              \
"  flags clear <N>            Clears uC flash flag N\n"                            \
"  fan enable                 Turns on fan\n"                                      \
"  fan disable                Turns off fan\n"
/* clang-format on */

static const char *silab_help = SILAB_HELP;

/* Helper function for wait_hook() porting layer impl */
static void inform_human_of_progress(const char *msg, int pct);

#if defined(__linux__) && !defined(__UBOOT__)
#include <assert.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char *const argv[]) {
  // static const char *wdog_feed[] = {"silab", "wdog", "feed"};
  // return silab_cmd(3, wdog_feed);

  setvbuf(stdout, NULL, _IONBF, 0);
  return silab_cmd(argc, argv);
}

static int i2c_fd = -1;
static int8_t i2c_eeprom_read(uint8_t adr, uint16_t subadr, uint8_t *buf,
                              int len) {
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg msgs[2];
  uint8_t busaddr[2];

  if (i2c_fd == -1)
    i2c_fd = open("/dev/i2c-0", O_RDWR);

  if (i2c_fd == -1) {
    perror("/dev/i2c-0");
    return 1;
  }

  busaddr[0] = ((subadr >> 8) & 0xff);
  busaddr[1] = (subadr & 0xff);

  msgs[0].addr = adr;
  msgs[0].flags = 0;
  msgs[0].len = 2;
  msgs[0].buf = busaddr;

  msgs[1].addr = adr;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = len;
  msgs[1].buf = buf;

  packets.msgs = msgs;
  packets.nmsgs = 2;

  return ioctl(i2c_fd, I2C_RDWR, &packets) < 0;
}

static int8_t i2c_eeprom_write(uint8_t adr, uint16_t subadr, uint8_t *buf,
                               int len) {
  struct i2c_rdwr_ioctl_data packets;
  struct i2c_msg msg;
  uint8_t *buf2 = (uint8_t *)alloca(len + 2);

  if (i2c_fd == -1)
    i2c_fd = open("/dev/i2c-0", O_RDWR);
  if (i2c_fd == -1) {
    perror("/dev/i2c-0");
    return 1;
  }

  buf2[0] = subadr >> 8;
  buf2[1] = subadr & 0xff;
  memcpy(&buf2[2], buf, len);
  msg.addr = adr;
  msg.flags = 0;
  msg.len = 2 + len;
  msg.buf = buf2;
  packets.msgs = &msg;
  packets.nmsgs = 1;
  return ioctl(i2c_fd, I2C_RDWR, &packets) < 0;
}

/* Return -1 to abort */
static int wait_hook(int pct) {
  if (isatty(0))
    inform_human_of_progress("Waiting on supercaps charging...", pct);
  if (pct != -1)
    usleep(100000);
  return 0;
}

#elif defined(__UBOOT__)
#include <common.h>

#include <cli.h>
#include <command.h>
#include <console.h>
#include <i2c.h>
#include <stdlib.h>

static int do_silabs(cmd_tbl_t *cmdtp, int flags, int argc,
                     char *const argv[]) {
  return silab_cmd(argc, argv);
}

U_BOOT_CMD(silabs, 5, 0, do_silabs, "Silabs management utility", SILAB_HELP);

static int8_t i2c_eeprom_read(uint8_t adr, uint16_t subadr, uint8_t *buf,
                              int len) {
  struct udevice *dev;
  int ret;

  printf("Read: Adr 0x%X, subadr 0x%X\n", adr, subadr);

  ret = i2c_get_chip_for_busnum(0, adr, 2, &dev);
  if (ret) {
    pr_err("couldn't get i2c bus - %d\n", ret);
    return ret;
  }
  return dm_i2c_read(dev, subadr, buf, len);
}

static int8_t i2c_eeprom_write(uint8_t adr, uint16_t subadr, uint8_t *buf,
                               int len) {
  struct udevice *dev;
  int ret;

  printf("Write: Adr 0x%X, subadr 0x%X\n", adr, subadr);
  ret = i2c_get_chip_for_busnum(0, adr, 2, &dev);
  if (ret) {
    pr_err("couldn't get i2c bus - %d\n", ret);
    return ret;
  }

  return dm_i2c_write(dev, subadr, buf, len);
}

static int wait_hook(int pct) {
  inform_human_of_progress("Waiting on supercaps charging...", pct);
  if (pct != -1)
    udelay(100000);
  if (ctrlc()) {
    puts("\n");
    return 1;
  } else
    return 0;
}

#elif defined(__WATCOMC__)
#include <assert.h>
#include <bios.h>
#include <conio.h>
#include <i86.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pci.h"

/* 1AB102
 * INT 1A - PCI BIOS v2.0c+ - FIND PCI DEVICE
 * http://www.delorie.com/djgpp/doc/rbinter/id/82/23.html
 *
 * Find the index'th PCI device with the given vendorID and deviceID.
 * Returns 0 if OK, -1 if error invoking the BIOS through DPMI, or a
 * PCI BIOS error code.  If OK, sets bus, device, and func to the
 * address of the device found.
 */

static int pci_find(uint16_t vendorID, uint16_t deviceID, uint8_t index,
                    uint8_t *bus, uint8_t *device, uint8_t *func) {
  union REGS r;

  memset(&r, 0, sizeof(r));

  r.x.ax = 0xb102;
  r.x.cx = deviceID;
  r.x.dx = vendorID;
  r.x.si = index;

  int86(0x1a, &r, &r);

  if (r.h.ah == 0) {
    *bus = r.h.bh;
    *device = (r.h.bl >> 3) & 0x1f;
    *func = r.h.bl & 0x03;
  }

  return r.h.ah;
}

static uint16_t i2c_ba;
static int8_t i2c_read(uint8_t adr, uint8_t *buf, int len) {
  int i;
  uint8_t r;

  outp(i2c_ba + 3, adr | 1);
  while (!((r = inp(i2c_ba + 1)) & (1 << 5)))
    ;
  if (r & (1 << 4))
    return -1; /* Not detected */

  outp(i2c_ba + 1, r); /* Clear status */
  if (len == 1)
    outp(i2c_ba, 2);
  inp(i2c_ba + 4); /* Start read */

  for (i = 0; i < len; i++) {
    while (!((r = inp(i2c_ba + 1)) & (1 << 6)))
      ;                  /* Wait for RX_Rdy */
    outp(i2c_ba + 1, r); /* Clear status */
    if (i == (len - 2))
      outp(i2c_ba, 2); /* Send STOP on last byte */
    buf[i] = inp(i2c_ba + 4);
  }
  return 0;
}

static int8_t i2c_write(uint8_t adr, uint8_t *buf, int len) {
  int i;
  uint8_t r;

  outp(i2c_ba + 3, adr & 0x2fe);
  while (!((r = inp(i2c_ba + 1)) & (1 << 5)))
    ;
  if (r & (1 << 4))
    return -1; /* Not detected */

  outp(i2c_ba + 1, r); /* Clear status */

  for (i = 0; i < len; i++) {
    outp(i2c_ba + 4, buf[i]);
    while (!((r = inp(i2c_ba + 1)) & (1 << 5)))
      ;                  /* Wait for TX_Done */
    outp(i2c_ba + 1, r); /* Clear status */
  }
  outp(i2c_ba, 2); /* Send STOP on last byte */
  return 0;
}

static int8_t i2c_eeprom_read(uint8_t adr, uint16_t subadr, uint8_t *buf,
                              int len) {
  uint8_t sa[2];

  sa[0] = subadr >> 8;
  sa[1] = subadr & 0xff;
  if (i2c_write(adr, sa, 2) == -1)
    return -1;

  return i2c_read(adr, buf, len);
}

static int8_t i2c_eeprom_write(uint8_t adr, uint16_t subadr, uint8_t *buf,
                               int len) {
  int i;
  uint8_t r;
  uint8_t sa[2];

  sa[0] = subadr >> 8;
  sa[1] = subadr & 0xff;

  outp(i2c_ba + 3, adr & 0x2fe);
  while (!((r = inp(i2c_ba + 1)) & (1 << 5)))
    ;
  if (r & (1 << 4))
    return -1; /* Not detected */

  outp(i2c_ba + 1, r); /* Clear status */

  for (i = 0; i < 2; i++) {
    outp(i2c_ba + 4, sa[i]);
    while (!((r = inp(i2c_ba + 1)) & (1 << 5)))
      ;                  /* Wait for TX_Done */
    outp(i2c_ba + 1, r); /* Clear status */
  }
  for (i = 0; i < len; i++) {
    outp(i2c_ba + 4, buf[i]);
    while (!((r = inp(i2c_ba + 1)) & (1 << 5)))
      ;                  /* Wait for TX_Done */
    outp(i2c_ba + 1, r); /* Clear status */
  }
  outp(i2c_ba, 2); /* Send STOP on last byte */
  return 0;
}

static int wait_hook(int pct) {
  unsigned short c;

  inform_human_of_progress("Waiting on supercaps charging...", pct);
  if (pct != -1)
    delay(100);
  if (c = _bios_keybrd(_KEYBRD_READY)) {
    _bios_keybrd(_KEYBRD_READ);
    if (c == 3)
      return 1; /* ctrl-C */
  }
  return 0;
}

int main(int argc, char **argv) {
  uint32_t v;
  uint8_t bus, device, func;
  int32_t r;

  r = pci_find(0x17f3, 0x6011, 0, &bus, &device, &func);
  if (r != 0) {
    printf("Vortex not found!\n"); /* South bridge */
    return 2;
  }

  v = pci_read_cfgl(bus, device, func, 0xd4);
  i2c_ba = v & 0xffff;

  outp(i2c_ba + 7, 0x80); /* Reset */
  while (inp(i2c_ba + 7) & 0x80)
    ;

  return silab_cmd(argc, argv);
}

#else /* Use external porting layer hooks in silabs-port.c */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
static int wait_hook(int pct); /* pct == -1 for end */
static int8_t i2c_eeprom_read(uint8_t adr, uint16_t subadr, uint8_t *buf,
                              int len);
static int8_t i2c_eeprom_write(uint8_t adr, uint16_t subadr, uint8_t *buf,
                               int len);

/* silabs-port.c should contain implementations of the above 3 functions */
#include "silabs-port.c"
#endif

/* Assumes being called 10 times per second for purpose of computing ETAs */
static void inform_human_of_progress(const char *msg, int pct) {
  static int projected_end = 0, n = 0, pending = 0, last_pct = 0;
  static char *seq = "-\\|/";

  if (!pending && pct == -1)
    return;

  if (pending && pct == -1)
    printf("\r%s... done         \n", msg);
  else if (0 == (n & 0x7)) {
    int n_to_go;
    if (pct > last_pct) {
      last_pct = pct;
      projected_end = n + ((100 - pct) * n) / pct;
    }
    if (projected_end < n)
      n_to_go = 0;
    else
      n_to_go = projected_end - n;
    printf("\r%s... %3d%% [%d:%02d] %c \x08", msg, pct, n_to_go / 600,
           n_to_go % 600 / 10, seq[n & 3]);
  } else
    printf("\x08%c", seq[n & 3]);

  pending = 1;
  n++;

  if (pct == -1)
    n = last_pct = pending = projected_end = 0;
}

static volatile uint8_t busy = 0;
static volatile uint8_t wdog_feed_pending = 0;
static int8_t silab_outw(uint16_t subadr, uint16_t s) {
  int8_t r;
  uint8_t buf[2];

  buf[0] = s >> 8;
  buf[1] = s & 0xff;
  /* Detects recursive call.  API is not async-safe except for wdog feed! */
  assert(!busy);
  busy = 1;
  r = i2c_eeprom_write(0x54, subadr, buf, 2);
  busy = 0;
  return r;
}

static int8_t silab_outb(uint16_t subadr, uint8_t b) {
  int8_t r;
  uint8_t buf[1];

  buf[0] = b;
  /* Detects recursive call.  API is not async-safe except for wdog feed! */
  assert(!busy);
  busy = 1;
  r = i2c_eeprom_write(0x54, subadr, buf, 1);
  busy = 0;
  return r;
}

static int16_t silab_inb(uint16_t subadr) {
  int r;
  uint8_t buf[1];

  /* Detects recursive call.  API is not async-safe except for wdog feed! */
  assert(!busy);
  busy = 1;
  r = i2c_eeprom_read(0x54, subadr, buf, 1);
  busy = 0;
  if (r == -1)
    return -1;
  else
    return buf[0];
}

static int32_t silab_inw(uint16_t subadr) {
  int r;
  uint8_t buf[2];

  /* Detects recursive call.  API is not async-safe except for wdog feed! */
  assert(!busy);
  busy = 1;
  r = i2c_eeprom_read(0x54, subadr, buf, 2);
  busy = 0;
  if (r == -1)
    return -1;
  else
    return ((buf[0] << 8) | buf[1]);
}

static int silab_read(uint16_t subadr, uint8_t *buf, int len) {
  int r;
  /* Detects recursive call.  API is not async-safe except for wdog feed! */
  assert(!busy);
  busy = 1;
  r = i2c_eeprom_read(0x54, subadr, buf, len);
  busy = 0;
  return r;
}

static int silab_write(uint16_t subadr, uint8_t *buf, int len) {
  int r;
  /* Detects recursive call.  API is not async-safe except for wdog feed! */
  assert(!busy);
  busy = 1;
  r = i2c_eeprom_write(0x54, subadr, buf, len);
  busy = 0;
  return r;
}

static uint8_t silab_board_is(const char *board) {
  static uint8_t buf[8];
  static uint8_t d = 0;
  int r;

  if (!d) {
    d = 1;
    r = silab_read(4096, buf, sizeof(buf));
    assert(r == 0);
    buf[sizeof(buf) - 1] = 0;
  }

  return strstr((char *)buf, board) == NULL ? 0 : 1;
}

/* Sets the current runtime scaps_en */
static void silab_scaps_en(uint8_t val) {
  uint8_t ctl;

  ctl = silab_inb(22);
  ctl &= ~6;
  if (val)
    ctl |= 1 << 1;
  silab_outb(22, ctl);
}

static void silab_fan_en(uint8_t val) { silab_outb(1024 + 8, val ? 0 : 1); }

/* Sets powerup/reboot default for scaps_en (does not effect current boot) */
static void silab_scaps_default_en(uint8_t val) {
  uint8_t flags;

  flags = silab_inb(23);
  if (val)
    flags &= ~1;
  else
    flags |= 1;
  silab_outb(23, flags);
}

/* returns int 0-100 (could be >100 too) */
static uint8_t silab_scaps_charge_pct(void) {
  uint32_t a;

  a = silab_inw(16);
  return (a * 100 / MAX_CHARGE_MV);
}

/* Stores the value in flash */
static void silab_scaps_default_current(uint16_t ma) { silab_outw(24, ma); }

/* Temporary, for this current boot only. */
static void silab_scaps_current(uint16_t ma) { silab_outw(26, ma); }

/* Returns 100% at MIN_CHARGE_MV and 0% for full charge */
static uint8_t silab_scaps_discharge_pct(void) {
  int32_t a, b;

  a = silab_inw(16);
  if (a <= MIN_CHARGE_MV)
    return 100;
  else
    a -= MIN_CHARGE_MV;
  b = (MAX_CHARGE_MV - MIN_CHARGE_MV);
  if (a >= b)
    return 0;
  else
    return 100 - (a * 100 / b);
}

/* Sleeps CPU, FPGA, and uC.  About 13ma on 2/23/2018. */
static void silab_sleep(uint32_t ms) {
  uint8_t buf[5];
  if (ms % 10)
    ms = (ms / 10) + 1;
  else
    ms = ms / 10;

  buf[0] = ms & 0xff;
  buf[1] = (ms >> 8) & 0xff;
  buf[2] = (ms >> 16) & 0xff;
  buf[3] = (ms >> 24) & 0xff;
  buf[4] = 2;

  silab_write(1024, buf, 5);
}

static uint8_t wdog_init = 0;
/* 0 ms disables */
static void silab_wdog_set(uint32_t ms) {
  uint8_t buf[5];
  if (ms % 10)
    ms = (ms / 10) + 1;
  else
    ms = ms / 10;

  buf[0] = ms & 0xff;
  buf[1] = (ms >> 8) & 0xff;
  buf[2] = (ms >> 16) & 0xff;
  buf[3] = (ms >> 24) & 0xff;
  buf[4] = 1;

  silab_write(1024, buf, 5);
  wdog_init = 1;
}

/* Feeds for another interval (interval set via silab_wdog_set()) */
static void silab_wdog_feed(void) {
  if (!busy) {
    if (!wdog_init) { /* If the wdog is being fed but has never been
                         initialized, set it once to a reasonable
                         default. */
      uint8_t buf[4];

      wdog_init = 1;
      silab_read(1024, buf, 4);
      if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
        silab_wdog_set(DEFAULT_WDOG_MS);
    }

    silab_outb(1028, 1);
    wdog_feed_pending = 0;
  } else
    wdog_feed_pending = 1;
}

static uint8_t lockn = 0;
void silab_i2c_lock(void) {
  if (lockn++ == 0) {
    assert(!busy);
    assert(lockn < 128); /* Over 128 deep I don't think so... */
    busy = 1;
  }
}

void silab_i2c_unlock(void) {
  assert(busy); /* Unlock with no lock ? */
  if (--lockn == 0) {
    busy = 0;
    if (wdog_feed_pending)
      silab_wdog_feed();
  }
}

static void silab_scaps_wait_pct(int pct) {
  int ctl = silab_inb(22);
  int init, cur, tar;
  if (!(ctl & 2)) {
    ctl |= 2;
    ctl &= ~4;
    silab_outb(22, ctl); /* Turn on scaps if not already */
  }

  if (pct > 100)
    pct = 100;
  tar = MIN_CHARGE_MV + (MAX_CHARGE_MV - MIN_CHARGE_MV) * pct / 100;
  init = cur = silab_inw(16);
  while (cur < tar) {
    int r = wait_hook((cur - init) * 100 / (tar - init));
    if (r)
      return;
    cur = silab_inw(16);
    if (cur < init)
      init = cur;
    if (wdog_feed_pending)
      silab_wdog_feed();
  }
  wait_hook(-1);
}

/* full_charge means wait for 100%.  not full_charge means wait for min */
static void silab_scaps_wait(uint8_t full_charge) {
  silab_scaps_wait_pct(full_charge ? 100 : 1);
}

/* Returns true if last reboot was caused by silab wdog */
static int silab_wdog_expired(void) {
  return ((silab_inb(1028) & (1 << 7)) ? 1 : 0);
}

/* Returns true if USB console is connected */
static int silab_usb_connected(void) {
  return ((silab_inb(22) & 0x10) ? 1 : 0);
}

static void silab_flags_set(uint8_t n) {
  n &= 0x7;
  silab_outb(23, silab_inb(23) | (1 << n));
}

static void silab_flags_clr(uint8_t n) {
  n &= 0x7;
  silab_outb(23, silab_inb(23) & ~(1 << n));
}

static uint8_t silab_flags(uint8_t n) { return ((silab_inb(23) >> n) & 1); }

static void silab_status_ts4400(void) {
  int i;
  uint8_t buf[27];
  uint16_t *sbuf = (uint16_t *)buf;
  uint8_t wdog[4];
  uint32_t w;
  uint8_t ver;
  const static char *an[10] = {
      "5V",   "Charge V",  "3.3V",       "1.5V",       "1.2V",
      "1.8V", "Core cur.", "Supercap 1", "Supercap 2", "4.7V"};

  silab_read(0, buf, 27);
  ver = silab_inb(0xffff);
  silab_read(1024, wdog, 4);
  w = wdog[0];
  w |= (uint32_t)wdog[1] << 8;
  w |= (uint32_t)wdog[2] << 16;
  w |= (uint32_t)wdog[3] << 24;

  for (i = 0; i < 11; i++)
    sbuf[i] = ((uint16_t)buf[i << 1] << 8) | buf[(i << 1) | 1];

  for (i = 0; i < 10; i++)
    printf("%s:\t%1d.%03d\n", an[i], sbuf[i] / 1000, sbuf[i] % 1000);

  printf("Temperature:\t%dC\n", sbuf[10]);
  printf("uC ver:\t%d\n", ver);
  if (sbuf[8] > (sbuf[9] + 250))
    buf[22] |= 1;
  printf("Supercaps:\t");
  if ((buf[22] & 2) == 0)
    printf("disabled");
  else if ((buf[22] & 1))
    printf("discharging");
  else if ((buf[22] & 4))
    printf("charged, %d%%", silab_scaps_charge_pct());
  else
    printf("charging, %d%%", silab_scaps_charge_pct());
  printf(" (default: %s)", (buf[23] & 1) ? "disabled" : "enabled");
  printf("\nSupercaps charge cur.:\t%d mA (default: %d mA)\n", silab_inw(26),
         silab_inw(24));
  printf("Watchdog:\t%d ms", w);
  printf(" (%s)", (buf[22] & (1 << 6)) ? "ARMED" : "disabled");
  printf(" (last reboot was %sfrom watchdog)",
         (silab_inb(1028) & (1 << 7)) ? "" : "NOT ");
  printf("\nUSB console:\t%s\n",
         (buf[22] & 0x10) ? "connected" : "disconnected");
}

static void silab_status_ts7100(void) {
  int i, r;
  uint8_t buf[27], build[80];
  uint16_t *sbuf = (uint16_t *)buf;
  uint8_t wdog[4];
  uint32_t w;
  const static char *an[10] = {"5V",         "Charge V", "3.3V", "8-48V",
                               "",           "",         "",     "Supercap 1",
                               "Supercap 2", ""};

  r = silab_read(4096, build, sizeof(build));
  assert(r == 0);

  r = silab_read(0, buf, 27);
  assert(r == 0);
  silab_read(1024, wdog, 4);
  w = wdog[0];
  w |= (uint32_t)wdog[1] << 8;
  w |= (uint32_t)wdog[2] << 16;
  w |= (uint32_t)wdog[3] << 24;
  w *= 10;

  for (i = 0; i < 11; i++)
    sbuf[i] = ((uint16_t)buf[i << 1] << 8) | buf[(i << 1) | 1];

  for (i = 0; i < 10; i++)
    if (*an[i])
      printf("%s:\t%1d.%03d\n", an[i], sbuf[i] / 1000, sbuf[i] % 1000);

  printf("Temperature:\t%dC (%dC initial)\n", sbuf[10], sbuf[4]);
  printf("uC build:\t%s\n", build);
  printf("uC ver:\t%d\n", silab_inb(2048));
  if (sbuf[8] > (sbuf[9] + 250))
    buf[22] |= 1;
  printf("Supercaps:\t");
  if ((buf[22] & 2) == 0)
    printf("disabled");
  else if ((buf[22] & 1))
    printf("discharging, %d%%", silab_scaps_discharge_pct());
  else if ((buf[22] & 4))
    printf("charged, %d%%", silab_scaps_charge_pct());
  else
    printf("charging, %d%%", silab_scaps_charge_pct());
  printf(" (default: %s)", (buf[23] & 1) ? "disabled" : "enabled");
  printf("\nSupercaps charge cur.:\t%d mA (default: %d mA)\n", silab_inw(26),
         silab_inw(24));
  printf("Watchdog:\t%d ms", w);
  printf(" (%s)", (buf[22] & (1 << 6)) ? "ARMED" : "disabled");
  printf(" (last reboot was %sfrom watchdog)",
         (silab_inb(1028) & (1 << 7)) ? "" : "NOT ");
  printf("\nUSB console:\t%s\n",
         (buf[22] & 0x10) ? "connected" : "disconnected");
}

static void silab_status_ts7840(void) {
  int i, r;
  uint8_t buf[27];
  uint8_t build[80];
  uint16_t *sbuf = (uint16_t *)buf;
  uint8_t wdog[4];
  uint32_t w;
  const static char *an[10] = {"5V",
                               "Charge V",
                               "",
                               "8-48V",
                               "",
                               "Supercap 2 (initial)",
                               "Fan current",
                               "Supercap 1",
                               "Supercap 2",
                               ""};

  r = silab_read(4096, build, sizeof(build));
  assert(r == 0);

  r = silab_read(0, buf, 27);
  assert(r == 0);
  silab_read(1024, wdog, 4);
  w = wdog[0];
  w |= (uint32_t)wdog[1] << 8;
  w |= (uint32_t)wdog[2] << 16;
  w |= (uint32_t)wdog[3] << 24;
  w *= 10;

  for (i = 0; i < 11; i++)
    sbuf[i] = ((uint16_t)buf[i << 1] << 8) | buf[(i << 1) | 1];

  for (i = 0; i < 10; i++)
    if (*an[i])
      printf("%s:\t%1d.%03d\n", an[i], sbuf[i] / 1000, sbuf[i] % 1000);

  printf("Temperature:\t%dC (%dC initial)\n", sbuf[10], sbuf[4]);
  printf("uC build:\t%s\n", build);
  printf("uC ver:\t%d\n", silab_inb(2048));
  if (sbuf[8] > (sbuf[9] + 250))
    buf[22] |= 1;
  printf("Supercaps:\t");
  if ((buf[22] & 2) == 0)
    printf("disabled");
  else if ((buf[22] & 1))
    printf("discharging, %d%%", silab_scaps_discharge_pct());
  else if ((buf[22] & 4))
    printf("charged, %d%%", silab_scaps_charge_pct());
  else
    printf("charging, %d%%", silab_scaps_charge_pct());
  printf(" (default: %s)", (buf[23] & 1) ? "disabled" : "enabled");
  printf("\nSupercaps charge cur.:\t%d mA (default: %d mA)\n", silab_inw(26),
         silab_inw(24));
  printf("Watchdog:\t%d ms", w);
  printf(" (%s)", (buf[22] & (1 << 6)) ? "ARMED" : "disabled");
  printf(" (last reboot was %sfrom watchdog)",
         (silab_inb(1028) & (1 << 7)) ? "" : "NOT ");
  printf("\nUSB console:\t%s\n",
         (buf[22] & 0x10) ? "connected" : "disconnected");
}

static void silab_status_ts7250v3(void) {
  int i, r;
  uint8_t buf[27];
  uint8_t build[80];
  uint16_t *sbuf = (uint16_t *)buf;
  uint8_t wdog[4];
  uint32_t w;
  const static char *an[10] = {"5V",      "VDD_SOC", "3.3V", "10-48V", "",
                               "VDD_ARM", "",        "",     "",       ""};

  r = silab_read(4096, build, sizeof(build));
  assert(r == 0);

  r = silab_read(0, buf, 27);
  assert(r == 0);
  silab_read(1024, wdog, 4);
  w = wdog[0];
  w |= (uint32_t)wdog[1] << 8;
  w |= (uint32_t)wdog[2] << 16;
  w |= (uint32_t)wdog[3] << 24;
  w *= 10;

  for (i = 0; i < 11; i++)
    sbuf[i] = ((uint16_t)buf[i << 1] << 8) | buf[(i << 1) | 1];

  for (i = 0; i < 10; i++)
    if (*an[i])
      printf("%s:\t%1d.%03d\n", an[i], sbuf[i] / 1000, sbuf[i] % 1000);

  printf("Temperature:\t%dC (%dC initial)\n", sbuf[10], sbuf[4]);
  printf("uC build:\t%s\n", build);
  printf("uC ver:\t%d\n", silab_inb(2048));
  printf("Watchdog:\t%d ms", w);
  printf(" (%s)", (buf[22] & (1 << 6)) ? "ARMED" : "disabled");
  printf(" (last reboot was %sfrom watchdog)",
         (silab_inb(1028) & (1 << 7)) ? "" : "NOT ");
  printf("\nUSB console:\t%s\n",
         (buf[22] & 0x10) ? "connected" : "disconnected");
}

static void silab_status(void) {
  if (silab_board_is("7840"))
    silab_status_ts7840();
  else if (silab_board_is("7100"))
    silab_status_ts7100();
  else if (silab_board_is("7250"))
    silab_status_ts7250v3();
  else if (silab_board_is("4400"))
    silab_status_ts4400();
  else
    assert(0); /* Invalid silabs */
}

static int my_atoi(char *s) { /* Because uboot doesnt have atoi() */
  int ret;
  for (ret = 0; *s != '\0'; ++s)
    ret = ret * 10 + *s - '0';

  return ret;
}

int silab_cmd(int argc, char *const argv[]) {

  if (argc == 1) {
    printf("Usage: %s [CMD] ...\n", argv[0]);
    puts(silab_help);
  } else if (strcmp("fan", argv[1]) == 0) {
    if (argc == 3 && strcmp("disable", argv[2]) == 0)
      silab_fan_en(0);
    else if (argc == 3 && strcmp("enable", argv[2]) == 0)
      silab_fan_en(1);
  } else if (strcmp("status", argv[1]) == 0) {
    silab_status();
  } else if (strcmp("reboot", argv[1]) == 0)
    silab_sleep(400);
  else if (strcmp("sleep", argv[1]) == 0) {
    if (argc >= 2)
      silab_sleep(my_atoi(argv[2]));
    else
      silab_sleep(0);
  } else if (strcmp("wdog", argv[1]) == 0) {
    if (argc == 2)
      return ((silab_inb(22) & (1 << 6)) ? 1 : 0);
    else if (argc == 3 && strcmp("expired", argv[2]) == 0)
      return silab_wdog_expired();
    else if (argc >= 3 && strcmp("set", argv[2]) == 0)
      silab_wdog_set(my_atoi(argv[3]));
    else if (argc >= 2 && strcmp("feed", argv[2]) == 0)
      silab_wdog_feed();
    else if (argc >= 2 && strcmp("disable", argv[2]) == 0)
      silab_wdog_set(0);
  } else if (strcmp("scaps", argv[1]) == 0) {
    /* 7250 has no supercaps */
    if (silab_board_is("7250"))
      return (argc == 2) ? 0 : 1;
    else if (argc == 2)
      return ((silab_inb(22) & 2) ? 1 : 0);
    else if (argc == 4 && strcmp("pct", argv[2]) == 0) {
      if (100 - my_atoi(argv[3]) >= silab_scaps_discharge_pct())
        return 1;
      else
        return 0;
    } else if (argc >= 3 && strcmp("current", argv[2]) == 0) {
      if (argc >= 4 && strcmp("default", argv[3]) == 0)
        silab_scaps_default_current(my_atoi(argv[4]));
      else
        silab_scaps_current(my_atoi(argv[3]));
    } else if (argc == 3 && strcmp("enable", argv[2]) == 0)
      silab_scaps_en(1);
    else if (argc == 3 && strcmp("disable", argv[2]) == 0)
      silab_scaps_en(0);
    else if (argc == 4 && strcmp("default", argv[2]) == 0) {
      if (strcmp("enable", argv[3]) == 0)
        silab_scaps_default_en(1);
      else if (strcmp("disable", argv[3]) == 0)
        silab_scaps_default_en(0);
    } else if (argc >= 3 && strcmp("wait", argv[2]) == 0) {
      if (argc == 5 && strcmp("pct", argv[3]) == 0)
        silab_scaps_wait_pct(my_atoi(argv[4]));
      else if (argc == 4 && strcmp("full", argv[3]))
        silab_scaps_wait(1);
      else if (argc == 3)
        silab_scaps_wait(0);
    }
  } else if (strcmp("usb", argv[1]) == 0)
    return silab_usb_connected();
  else if (argc >= 3 && strcmp("flags", argv[1]) == 0) {
    if (argc == 4 && strcmp("set", argv[2]) == 0)
      silab_flags_set(my_atoi(argv[3]));
    else if (argc == 4 && strcmp("clear", argv[2]) == 0)
      silab_flags_clr(my_atoi(argv[3]));
    else if (argc == 3)
      return (silab_flags(my_atoi(argv[2])));
  } else {
    printf("Usage: %s [CMD] ...\n", argv[0]);
    puts(silab_help);
  }

  return 0;
}
