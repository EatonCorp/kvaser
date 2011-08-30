MODULE_NAME:=leaf

PWD:=$(shell pwd)

KBASE:=/lib/modules/$(shell uname -r)
KDIR:=$(KBASE)/build
KSOURCE:=$(KBASE)/source

headers_common:= \
  canIfData.h \
  debug.h \
  hwnames.h \
  module_versioning.h \
  osif_common.h \
  osif_functions_kernel.h \
  osif_kernel.h \
  poppack.h \
  pshpack1.h \
  queue.h \
  vcanevt.h \
  vcan_ioctl.h \
  VCanOsIf.h \

headers:= \
  filo_cmds.h \
  leafHWIf.h \
  $(headers_common) \

srcs_common:= \
  osif_functions_kernel.c \
  queue.c \
  VCanOsIf.c \

srcs:= \
  leafHWIf.c \
  $(srcs_common) \

ifneq ($(KERNELRELEASE),)
objs:= $(patsubst %.c,%.o,$(srcs))


obj-m:=$(MODULE_NAME).o
$(MODULE_NAME)-y:=$(objs)
ccflags-y:=-DLINUX=1 -DDEBUG=0 -DLINUX_2_6 -DWIN32=0

else
.PHONY: all
all: leaf.ko

$(headers_common):
	for file in $(headers_common); do ln -sf ../include/$$file; done

$(srcs_common):
	for file in $(srcs_common); do ln -sf ../common/$$file; done

leaf.ko: $(srcs) $(headers)
	$(MAKE) -C $(KDIR) M=$(PWD)

.PHONY: install
install: leaf.ko
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	install -m 700 leaf.sh /usr/sbin/
	install -m 644 10-kvaser_leaf.rules /etc/udev/rules.d 
	if [ `which udevcontrol` ] ; then udevcontrol reload_rules; else udevadm control --reload-rules; fi
	depmod

.PHONY: clean
clean:
	-rm $(headers_common) $(srcs_common)
	$(MAKE) -C $(KDIR) M=$(PWD) clean

endif
