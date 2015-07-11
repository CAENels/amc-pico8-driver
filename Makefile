obj-m	:= amc_pico.o

amc_pico-objs := 		\
	amc_pico_main.o 	\
	amc_pico_bist.o 	\
	amc_pico_char.o 	\
	amc_pico_dma.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all: default

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

debug:
	KCPPFLAGS="-DDEBUG_SYS=1 -DDEBUG_CHAR=1 -DDEBUG_DMA=1 -DDEBUG_IRQ=1 -DDEBUG_FULL=1" make all

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
