obj-m	:= amc_pico.o

amc_pico-objs := 		\
	amc_pico_main.o 	\
	amc_pico_bist.o 	\
	amc_pico_char.o 	\
	amc_pico_dma.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
ccflags-y += $(shell \
	if [ $(cc-version) -ge 0409 ] ; then \
	echo "-Wno-error=date-time"; fi ;)

all: default

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

debug:
	KCPPFLAGS="-DDEBUG_SYS=1 -DDEBUG_CHAR=1 -DDEBUG_DMA=1 -DDEBUG_IRQ=1 -DDEBUG_FULL=1" make all

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

install:
	echo "KERNEL==\"amc_pico*\", MODE=\"0666\"\n" > /etc/udev/rules.d/10-CAENels-AMC-Pico.rules
	udevadm control --reload-rules
	cp amc_pico.ko /lib/modules/$(shell uname -r)/kernel/drivers/misc/
	echo amc_pico >> /etc/modules
	depmod -a
	modprobe amc_pico
	echo "Installation done"

uninstall:
	modprobe -r amc_pico
	rm /etc/udev/rules.d/10-CAENels-AMC-Pico.rules
	udevadm control --reload-rules
	rm /lib/modules/$(shell uname -r)/kernel/drivers/misc/amc_pico.ko
	sed -i '/amc_pico/d' /etc/modules
	depmod -a
	echo "Driver uninstalled"
