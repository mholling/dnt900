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

tty: $(DRIVER).ko
	sudo insmod $(DRIVER).ko n_dnt900=29 gpio_cfg=25 gpio_cts=27
	ldattach --debug --eightbits --noparity --onestopbit --speed 9600 29 /dev/ttyAMA0 &

usb: $(DRIVER).ko
	sudo insmod $(DRIVER).ko n_dnt900=29
	ldattach --debug --eightbits --noparity --onestopbit --speed 9600 29 /dev/ttyUSB0 &

unload:
	killall ldattach || true
	sudo rmmod dnt900

endif
