
PWD := $(shell pwd)

INSTALL := install

KERNEL_MOD_DIR := polygator

ifeq ($(TARGET_DEVICE), k5)

obj-m := vinetic.o k5.o
vinetic-objs := vinetic-base.o
k5-objs := k5-base.o

#export ARCH=arm
#export CROSS_COMPILE=arm-polygator-linux-gnueabi-

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else

obj-m := vinetic.o gsm8ch.o
vinetic-objs := vinetic-base.o
gsm8ch-objs := gsm8ch-base.o

KERNEL_VERSION := `uname -r`
KERNEL_SRC_DIR := /lib/modules/$(KERNEL_VERSION)/build
KERNEL_STG_DIR := /

endif

all: modules libvinetic

modules:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) modules

.PHONY: libvinetic
libvinetic:
	make -C libvinetic

install: modules_install headers_install libvinetic_install

modules_install:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) INSTALL_MOD_PATH=$(KERNEL_STG_DIR) INSTALL_MOD_DIR=$(KERNEL_MOD_DIR) modules_install

headers_install:
	$(INSTALL) -m 755 -d "$(DESTDIR)/usr/include/polygator"
	for header in polygator/*.h ; do \
		$(INSTALL) -m 644 $$header "$(DESTDIR)/usr/include/polygator" ; \
	done

libvinetic_install:
	make -C libvinetic libvinetic_install

clean:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) clean
	make -C libvinetic clean
	@rm -f *~ polygator/*~
