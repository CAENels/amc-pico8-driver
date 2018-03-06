ifneq ($(KERNELRELEASE),)
# kbuild

-include $(M)/.config

obj-m	:= amc_pico.o

amc_pico-objs := amc_pico_main.o
amc_pico-objs += amc_pico_bist.o
amc_pico-objs += amc_pico_char.o
amc_pico-objs += amc_pico_dma.o

# This is a no-op when dynamic debugging is enabled.  See README
ccflags-$(CONFIG_AMC_PICO_DEBUG) += -DDEBUG -DDEBUG_SYS=1 -DDEBUG_CHAR=1 -DDEBUG_DMA=1 -DDEBUG_IRQ=1 -DDEBUG_FULL=1

ccflags-$(CONFIG_AMC_PICO_FRIB) += -DCONFIG_AMC_PICO_FRIB 

ifneq ($(CONFIG_AMC_PICO_SITE_DEFAULT),)
ccflags-y += -DCONFIG_AMC_PICO_SITE_DEFAULT=\"$(CONFIG_AMC_PICO_SITE_DEFAULT)\"
endif

else
# user makefile

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
PERL := perl

all: modules gen_py test/picodefs.py

modules_install modules: amc_pico_version.h

modules_install modules help:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $@

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $@
	rm -f amc_pico_version.h
	rm -f gen_py

gen_py: gen_py.c amc_pico.h amc_pico_version.h
	$(CC) -o $@ -g -Wall $<

amc_pico_version.h::
	$(PERL) genVersionHeader.pl -t . -N AMC_PICO_VERSION $(PWD)/amc_pico_version.h

test/picodefs.py: gen_py
	./$< $@

.PHONY: all modules_install modules clean

endif
