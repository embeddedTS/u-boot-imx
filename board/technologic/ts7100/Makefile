# SPDX-License-Identifier: GPL-2.0+
# Copyright (C) 2020-2022 Technologic Systems dba embeddedTS

obj-y  := ts7100.o

ifdef CONFIG_SPL_BUILD
	obj-y += spl.o
endif

ifndef CONFIG_SPL_BUILD
	obj-y += mfg.o led.o parse_strap.o
	obj-y += ../common/micro/micro.o
endif
