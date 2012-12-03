ccflags-y := -std=gnu99 -Wno-declaration-after-statement
DRIVER = dnt900

ifneq ($(KERNELRELEASE),)

obj-m := $(DRIVER).o

else

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
	
default: $(DRIVER).ko

$(DRIVER).ko: $(DRIVER).c
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

load: $(DRIVER).ko
	sudo insmod $(DRIVER).ko

unload:
	sudo rmmod dnt900

endif
