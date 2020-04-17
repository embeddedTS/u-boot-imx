# SPDX-License-Identifier: GPL-2.0+
# (C) Copyright 2020 Technologic Systems

obj-y  := ts7100.o

ifdef CONFIG_SPL_BUILD
	obj-y += spl.o
endif

ifndef CONFIG_SPL_BUILD
	obj-y += silabs.o mfg.o led.o
	obj-y += fram.o
	obj-y += parse_strap.o
endif
