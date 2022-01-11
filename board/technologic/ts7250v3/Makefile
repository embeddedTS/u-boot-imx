# SPDX-License-Identifier: GPL-2.0+
# (C) Copyright 2020-2022 Technologic Systems, Inc. dba embeddedTS

obj-y  := ts7250v3.o

ifdef CONFIG_SPL_BUILD
	obj-y += spl.o
endif

ifndef CONFIG_SPL_BUILD
	obj-y += silabs.o mfg.o led.o
endif
