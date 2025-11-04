#
# Copyright 2023-2024 embeddedTS
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += ts4300.o
obj-y += ../../../board/freescale/common/

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-y += lpddr4x-8gb.o
obj-y += lpddr4x-16gb.o
endif

obj-y += ../ts-common/
obj-y += parse_straps.o
obj-y += baseboards.o
obj-y += bbdetect.o
