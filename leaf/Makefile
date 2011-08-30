MODULE_NAME:=leaf

PWD:=$(shell pwd)

KBASE:=/lib/modules/$(shell uname -r)
KDIR:=$(KBASE)/build
KSOURCE:=$(KBASE)/source

ifneq ($(KERNELRELEASE),)
srcs:= \
  leafHWIf.c \
  ../common/objbuf.c \
  ../common/osif_functions_kernel.c \
  ../common/queue.c \
  ../common/VCanOsIf.c \

objs:= $(patsubst %.c,%.o,$(srcs))


obj-m:=$(MODULE_NAME).o
$(MODULE_NAME)-y:=$(objs)
ccflags-y:= \
  -DLINUX=1 \
  -D_LINUX=1 \
  -DDEBUG=0 \
  -D_DEBUG=0 \
  -DLINUX_2_6 \
  -DWIN32=0 \
  -I$(src)/../include/ \

else
.PHONY: all
all: leaf.ko

leaf.ko:
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
	$(MAKE) -C $(KDIR) M=$(PWD) clean

endif
