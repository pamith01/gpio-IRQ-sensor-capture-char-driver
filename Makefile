
# Makefile for am335x_sensor_cap driver
#
# Cross-compile on host:
#   make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
#        KDIR=/path/to/linux-am335x
#
# Native compile on BBB:
#   make KDIR=/lib/modules/$(uname -r)/build

obj-m := am335x_sensor_cap.o

KDIR  ?= /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

# ---- User-space test app (native ARM build on BBB) -----------------
test: sensor_cap_test.c sensor_cap_uapi.h
	$(CC) -Wall -Wextra -o sensor_cap_test sensor_cap_test.c -lpthread

.PHONY: all clean install test
