# Kvaser Linux Canlib library Makefile 
# Copyright (C) 2005 Kvaser AB - support@kvaser.com - www.kvaser.com
# Modified 2011-08-26: jacobrfoell@eaton.com


LIBNAME = libcanlib.so
LIBRARY = $(LIBNAME).1.0.1
SONAME  = $(LIBNAME).1

CFLAGS  += -Wall -DCANLIB_NAME_STRING=\"$(LIBRARY)\" -O2 -fomit-frame-pointer $(XTRA_CFLAGS) #-DPCMCIA_DEBUG=0
CFLAGS  +=-DLINUX=1 -DDEBUG=0 -DLINUX_2_6 -DWIN32=0
CFLAGS  += -I. -I../include

LIBCFLAGS = $(CFLAGS) -D_REENTRANT -fPIC
LDFLAGS = -lc -lpthread

SRCS := canlib.c 
SRCS += linkedlist.c 
SRCS += VCanFunctions.c
SRCS += ../common/osif_functions_user.c

OBJS := $(patsubst %.c, %.o, $(SRCS))
OTHERDEPS := ../include/canlib.h
DEPS := $(patsubst %.c, %.d, $(SRCS))

.PHONY:	all
all: $(LIBRARY) 

$(LIBRARY): $(DEPS) $(OTHERDEPS) $(OBJS)
	$(CC) $(CFLAGS) -shared -Wl,-soname,$(SONAME) -o $@ $(OBJS) $(LDFLAGS)
	ln -sf $@ $(LIBNAME)
	ln -sf $@ $(SONAME)

%.o: %.c
	$(CC) $(CFLAGS) $(LIBCFLAGS)  -c -o $@ $< 

%.d: %.c
	$(CC) -M $(CFLAGS) $< > $@ 
	$(CC) -M $(CFLAGS) $< | sed s/\\.o/.d/> $@ 

.PHONY: install
install: all
	-rm -f /usr/lib/$(LIBNAME)
	-rm -f /usr/lib/$(SONAME)
	install  -m 755 $(LIBRARY) /usr/lib/
	ln -s $(LIBRARY) /usr/lib/$(LIBNAME)
	ln -s $(LIBRARY) /usr/lib/$(SONAME)
	/sbin/ldconfig
	install -m 644 ../include/canlib.h /usr/include
	install -m 644 ../include/canstat.h /usr/include
	
	mkdir -p /usr/doc/canlib
	cp -r ../doc/HTMLhelp /usr/doc/canlib

.PHONY: clean
clean:	
	rm -f $(OBJS) $(LIBRARY) $(DEPS) $(LIBNAME) $(LIBRARY) $(SONAME)

depend: $(DEPS)

-include $(DEPS) 
