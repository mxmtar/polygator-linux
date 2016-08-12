
PWD := $(shell pwd)

INSTALL := install

KERNEL_MOD_DIR := polygator

ifeq ($(TARGET_DEVICE), k5)

obj-m := polygator.o vinetic.o simcard.o k5.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
simcard-objs := simcard-base.o
k5-objs := k5-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else ifeq ($(TARGET_DEVICE), fxo4)

obj-m := polygator.o vinetic.o fxo4.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
fxo4-objs := fxo4-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else ifeq ($(TARGET_DEVICE), fxs)

obj-m := polygator.o vinetic.o fxs.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
fxs-objs := fxs-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else ifeq ($(TARGET_DEVICE), gx)

obj-m := polygator.o vinetic.o simcard.o gx.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
simcard-objs := simcard-base.o
gx-objs := gx-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else ifeq ($(TARGET_DEVICE), g8)

obj-m := polygator.o vinetic.o simcard.o g8.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
simcard-objs := simcard-base.o
g8-objs := g8-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else ifeq ($(TARGET_DEVICE), g20)

obj-m := polygator.o vinetic.o simcard.o g20.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
simcard-objs := simcard-base.o
g20-objs := g20-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else ifeq ($(TARGET_DEVICE), pc)

obj-m := polygator.o vinetic.o simcard.o k32pci.o k32isa.o k32pcie.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
simcard-objs := simcard-base.o
k32pci-objs := k32pci-base.o
k32isa-objs := k32isa-base.o
k32pcie-objs := k32pcie-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else

obj-m := polygator.o vinetic.o simcard.o k32pci.o k32isa.o k32pcie.o k32pci2.o
polygator-objs := polygator-base.o
vinetic-objs := vinetic-base.o
simcard-objs := simcard-base.o
k32pci-objs := k32pci-base.o
k32isa-objs := k32isa-base.o
k32pcie-objs := k32pcie-base.o
k32pci2-objs := k32pci2-base.o

KERNEL_VERSION := `uname -r`
KERNEL_SRC_DIR := /lib/modules/$(KERNEL_VERSION)/build
KERNEL_STG_DIR := /

endif

CHKCONFIG	:= $(wildcard /sbin/chkconfig)
UPDATE_RCD	:= $(wildcard /usr/sbin/update-rc.d)
ifeq (,$(DESTDIR))
	ifneq (,$(CHKCONFIG))
		SYSVINIT_ADD := $(CHKCONFIG) --add polygator
	else
		ifneq (,$(UPDATE_RCD))
			SYSVINIT_ADD := $(UPDATE_RCD) polygator defaults 30 95
		endif
	endif
endif

all: modules

modules:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) modules

modules_install: install_modules

install: install_modules install_headers

install_modules:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) INSTALL_MOD_PATH=$(KERNEL_STG_DIR) INSTALL_MOD_DIR=$(KERNEL_MOD_DIR) modules_install

install_headers:
	$(INSTALL) -m 755 -d "$(DESTDIR)/usr/include/polygator"
	for header in polygator/*.h ; do \
		$(INSTALL) -m 644 $$header "$(DESTDIR)/usr/include/polygator" ; \
	done

install_sysvinit:
	$(INSTALL) -m 755 polygator.sysvinit $(DESTDIR)/etc/init.d/polygator
ifneq (,$(SYSVINIT_ADD))
	$(SYSVINIT_ADD)
endif

install_pgctl:
	$(INSTALL) -m 755 pgctl $(DESTDIR)/usr/bin/pgctl
	$(INSTALL) -m 755 pgsncfg $(DESTDIR)/usr/bin/pgsncfg
	$(INSTALL) -m 755 pgtty $(DESTDIR)/usr/bin/pgtty

install_asterisk_owner_udev_rules:
	$(INSTALL) -m 644 polygator-asterisk-udev.rules "$(DESTDIR)/etc/udev/rules.d/polygator-asterisk.rules"

uninstall: uninstall_modules uninstall_headers uninstall_sysvinit uninstall_pgctl uninstall_asterisk_owner_udev_rules

uninstall_modules:
	rm -rvf "$(DESTDIR)/lib/modules/$(KERNEL_VERSION)/$(KERNEL_MOD_DIR)"
	depmod

uninstall_headers:
	rm -rvf "$(DESTDIR)/usr/include/polygator"

uninstall_sysvinit:
	rm -fv $(DESTDIR)/etc/rc0.d/*polygator
	rm -fv $(DESTDIR)/etc/rc1.d/*polygator
	rm -fv $(DESTDIR)/etc/rc2.d/*polygator
	rm -fv $(DESTDIR)/etc/rc3.d/*polygator
	rm -fv $(DESTDIR)/etc/rc4.d/*polygator
	rm -fv $(DESTDIR)/etc/rc5.d/*polygator
	rm -fv $(DESTDIR)/etc/rc6.d/*polygator
	rm -fv $(DESTDIR)/etc/init.d/polygator

uninstall_pgctl:
	rm -fv $(DESTDIR)/usr/bin/pgctl
	rm -fv $(DESTDIR)/usr/bin/pgsncfg
	rm -fv $(DESTDIR)/usr/bin/pgtty

uninstall_asterisk_owner_udev_rules:
	rm -fv $(DESTDIR)/etc/udev/rules.d/polygator-asterisk.rules

clean:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) clean
	@rm -f *~ polygator/*~
