What's this?
------------

This package allows you to use the USB Audio interfaces from M-Audio/
Midiman with Linux.  It sets up udev rules to load the firmware into the
device.

Supported devices:
- MobilePre
- Ozone
- Sonica
- Transit

The project page is <http://usb-midi-fw.sourceforge.net/>.

Updated Firmware
------------------
I have added the updated transit firmware, the ma006101.bin, and created a branch (ma006101-mod) to test with.

I'm not sure if there are more updated firmwares for the supported hardware.

Patches
------------
The following patches are already applied to this source tree (from the debian package source):
    42-madfuload-rules.patch
    configure.ac.patch
    fix-64-bit-implicit-declarations.patch

Prerequisites
-------------

- Linux 2.6.8 or later.  Earlier kernels have a bug in the USB reset
  code which will prevent the loader from working correctly.
- udev version 057 or later


Installing
----------

1) Run './configure'.

2) Run 'make'.

3) As root, run 'make install'.


Uninstalling
------------

As root, run 'make uninstall'.

Running
-------

The udev rules cause the firmware to be automatically loaded when the
device is connected to the computer.


Contact
-------

For help, discussion or feedback, send a mail to the mailing list at
<usb-midi-fw-user@lists.sourceforge.net>.

written by Clemens Ladisch <clemens@ladisch.de>
