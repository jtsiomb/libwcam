# change this if you want to install elsewhere
PREFIX = /usr/local

obj = wcam.o
name = wcam
so_abi = 0
so_rev = 1

lib_a = lib$(name).a
ldname = lib$(name).so
soname = lib$(name).so.$(so_abi)
lib_so = lib$(name).so.$(so_abi).$(so_rev)
shared = -shared -Wl,-soname=$(soname)

CFLAGS = -pedantic -Wall -g -fPIC

.PHONY: all
all: $(lib_a) $(lib_so)

$(lib_a): $(obj)
	$(AR) rcs $@ $(obj)

$(lib_so): $(obj)
	$(CC) -o $@ $(shared) $(obj) $(LDFLAGS)

$(soname): $(lib_so)
	ln -s $< $@

$(ldname): $(soname)
	ln -s $< $@

.PHONY: clean
clean:
	rm -f $(obj) $(lib_a) $(lib_so)

.PHONY: install
install: $(lib_a) $(lib_so)
	mkdir $(DESTDIR)$(PREFIX)/include $(DESTDIR)$(PREFIX)/lib
	cp wcam.h $(DESTDIR)$(PREFIX)/include/wcam.h
	cp $(lib_a) $(DESTDIR)$(PREFIX)/lib/$(lib_a)
	cp $(lib_so) $(DESTDIR)$(PREFIX)/lib/$(lib_so)
	cd $(DESTDIR)$(PREFIX)/lib && \
		ln -s $(lib_so) $(soname) && \
		ln -s $(soname) $(ldname)
	

.PHONY: uninstall
uninstall:
	rm -f $(DESTDIR)$(PREFIX)/lib/$(lib_a)
	rm -f $(DESTDIR)$(PREFIX)/lib/$(lib_so)
	rm -f $(DESTDIR)$(PREFIX)/lib/$(soname)
	rm -f $(DESTDIR)$(PREFIX)/lib/$(ldname)
	rm -f $(DESTDIR)$(PREFIX)/include/wcam.h
