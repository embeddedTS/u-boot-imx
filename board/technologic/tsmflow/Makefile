# (C) Copyright 2015 Technologic Systems
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := tsmflow.o led.o silabs.o post.o

# prog_silab.o c2.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
