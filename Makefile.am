## Process this file with automake to generate Makefile.in
AUTOMAKE_OPTIONS = foreign

udevrulesdir = @UDEV_RULES_DIR@
firmwaredir = $(datadir)/usb/maudio

sbin_PROGRAMS = madfuload
madfuload_SOURCES = madfuload.c

udevrules_DATA = 42-madfuload.rules
dist_firmware_DATA = ma003101.bin ma004103.bin ma005101.bin ma006100.bin ma008100.bin

MOSTLYCLEANFILES = 42-madfuload.rules files.list
EXTRA_DIST = 42-madfuload.rules.in LICENSE Changelog $(PACKAGE).spec.in $(PACKAGE).spec

all-local: files.list

42-madfuload.rules: Makefile $(srcdir)/42-madfuload.rules.in
	rm -f $@ $@.tmp
	sed -e 's,@firmwaredir\@,$(firmwaredir),g' \
		-e 's,@sbindir\@,$(sbindir),g' \
		$(srcdir)/42-madfuload.rules.in >$@.tmp
	mv $@.tmp $@

$(PACKAGE).spec: Makefile $(srcdir)/$(PACKAGE).spec.in
	rm -f $@ $@.tmp
	sed -e 's,@version@,$(VERSION),g' \
		$(srcdir)/$(PACKAGE).spec.in > $@.tmp
	mv $@.tmp $@

files.list: Makefile
	rm -f $@
	for i in $(sbin_PROGRAMS); do \
		echo "$(sbindir)/$$i"; \
	done >> $@
	for i in $(udevrules_DATA); do \
		echo "$(udevrulesdir)/$$i"; \
	done >> $@
	for i in $(dist_firmware_DATA); do \
		echo "$(firmwaredir)/$$i"; \
	done >> $@
