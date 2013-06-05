# External Makefile for dnt900 line dicipline driver

ifneq ($(KERNELRELEASE),)
include Kbuild
else

PACKAGE := dnt900
VERSION := 1.0
export PACKAGE VERSION

# Directories

KSRC ?= /lib/modules/$(shell uname -r)/build
#DESTDIR=
PWD := $(shell pwd)

all: $(PACKAGE).ko

# Makefile rules for modules.

INCLUDEDIR = $(KSRC)/include

CFLAGS = -D__KERNEL__ -DMODULE -DEXPORT_SYMTAB -O3 -Wall -I$(INCLUDEDIR) -I./include

OBJS = $(PACKAGE).o

$(PACKAGE).ko: $(PACKAGE).c
	$(MAKE) -C $(KSRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KSRC) M=$(PWD) INSTALL_MOD_PATH=$(DESTDIR) modules_install

clean:
	$(MAKE) -C $(KSRC) M=$(PWD) clean
	rm -f *.o core *.ko

distclean:
	$(MAKE) -C $(KSRC) M=$(PWD) distclean
	rm -f *.o core *.ko

endif # KERNELRELEASE != ""

