# (C) Copyright 2015-2022 Technologic Systems, Inc. dba embeddedTS
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := ts7180.o led.o fpga.o silabs.o post.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
