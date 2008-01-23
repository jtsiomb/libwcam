# change this if you want to install elsewhere
PREFIX = /usr/local

obj = wcam.o
lib_a = libwcam.a
lib_so = libwcam.so

CC = gcc
CFLAGS = -pedantic -Wall -g -fpic
AR = ar

.PHONY: all
all: $(lib_a) $(lib_so)

$(lib_a): $(obj)
	$(AR) rcs $@ $(obj)

$(lib_so): $(obj)
	$(CC) $(CFLAGS) -shared -o $@ $(obj) $(LDFLAGS)

.PHONY: clean
clean:
	rm -f $(obj) $(lib_a) $(lib_so)

.PHONY: install
install: $(lib_a) $(lib_so)
	install -d $(PREFIX)/lib
	install -m 644 $(lib_a) $(lib_so) $(PREFIX)/lib/
	install -d $(PREFIX)/include
	install -m 644 wcam.h $(PREFIX)/include/wcam.h

.PHONY: uninstall
uninstall:
	rm -f $(PREFIX)/lib/$(lib_a)
	rm -f $(PREFIX)/lib/$(lib_so)
	rm -f $(PREFIX)/include/wcam.h
