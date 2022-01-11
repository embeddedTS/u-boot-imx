# SPDX-License-Identifier: GPL-2.0+

# (C) Copyright 2021-2022 Technologic Systems, Inc. dba embeddedTS

obj-y  := ts7180.o

ifdef CONFIG_SPL_BUILD
	obj-y += spl.o
endif

ifndef CONFIG_SPL_BUILD
	obj-y += silabs.o mfg.o
	obj-y += fram.o
endif

obj-y += led.o
obj-y += fpga.o
