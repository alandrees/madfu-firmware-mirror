/*
 * madfuload.c - firmware loader for M-Audio DFU devices
 *
 * For legal details, see the LICENSE file.
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <string.h>

/* USB stuff -- we can't rely on all Linux headers being there */

#define USB_DIR_OUT	0
#define USB_DIR_IN	0x80

#define USB_TYPE_CLASS		(0x01 << 5)
#define USB_RECIP_INTERFACE	0x01
#define USB_DT_INTERFACE	0x04
#define USB_CLASS_APP_SPEC	0xfe

struct usb_descriptor_header {
	u_int8_t bLength;
	u_int8_t bDescriptorType;
} __attribute__ ((packed));

#define USBDEVFS_CONTROL	_IOWR('U', 0, struct usbdevfs_ctrltransfer)
#define USBDEVFS_CLAIMINTERFACE	_IOR('U', 15, unsigned int)
#define USBDEVFS_RESET		_IO('U', 20)

struct usbdevfs_ctrltransfer {
	u_int8_t bRequestType;
	u_int8_t bRequest;
	u_int16_t wValue;
	u_int16_t wIndex;
	u_int16_t wLength;
	u_int32_t timeout;
	void *data;
};

/* symbols and types from the DFU standard */

#define USB_SUBCLASS_DFU	0x01
#define USB_DT_DFU_FUNCTIONAL	0x21

#define DFU_CAN_DNLOAD			0x01
#define DFU_CAN_UPLOAD			0x02
#define DFU_MANIFESTATION_TOLERANT	0x04

#define DFU_DETACH	0
#define DFU_DNLOAD	1
#define DFU_UPLOAD	2
#define DFU_GETSTATUS	3
#define DFU_CLRSTATUS	4
#define DFU_GETSTATE	5
#define DFU_ABORT	6

struct dfu_status {
	u_int8_t bStatus;
	u_int8_t bwPollTimeout[3];
	u_int8_t bState;
	u_int8_t iString;
} __attribute__((packed));


enum {
	WAIT_STANDARD,
	WAIT_IGNORE,
	WAIT_BYTE3
};

static char *command;
static int to_logger;
static int wait_type;
static char *firmware;
static int firmware_length;
static int device;
static int interface;
static int transfer_size = 64;
static int logging;

static void print(int level, const char *format, va_list ap)
{
	if (to_logger) {
		vsyslog(LOG_DAEMON | level, format, ap);
	} else {
		vfprintf(stderr, format, ap);
		putc('\n', stderr);
	}
}

static void fatal(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);
	print(LOG_ERR, format, ap);
	va_end(ap);
	exit(1);
}

static void warning(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);
	print(LOG_WARNING, format, ap);
	va_end(ap);
}

static void logmsg(const char *format, ...)
{
	va_list ap;

	if (!logging)
		return;
	va_start(ap, format);
	print(LOG_DEBUG, format, ap);
	va_end(ap);
}

static int usb_control(unsigned char request_type, unsigned char request,
		       unsigned short value, unsigned short index,
		       unsigned short length, void *data)
{
	struct usbdevfs_ctrltransfer ct;
	int retries, err;

	ct.bRequestType = request_type;
	ct.bRequest = request;
	ct.wValue = value;
	ct.wIndex = index;
	ct.wLength = length;
	ct.timeout = 500;
	ct.data = data;
	for (retries = 0; retries < 3; ++retries) {
		err = ioctl(device, USBDEVFS_CONTROL, &ct);
		if (err >= 0)
			break;
		warning("control transfer failed: (%d) %s", errno, strerror(errno));
	}
	return err;
}

static int usb_claim_interface(unsigned int interface)
{
	return ioctl(device, USBDEVFS_CLAIMINTERFACE, &interface);
}

static int usb_reset(void)
{
	return ioctl(device, USBDEVFS_RESET, NULL);
}

static void load_firmware(char *filename)
{
	int fd = open(filename, O_RDONLY);
	if (fd == -1)
		fatal("cannot open %s: %s", filename, strerror(errno));
	firmware_length = lseek(fd, 0, SEEK_END);
	if (firmware_length < 0)
		fatal("cannot determine length of %s: %s", filename, strerror(errno));
	if (firmware_length == 0)
		fatal("firmware file %s is empty", filename);
	lseek(fd, 0, SEEK_SET);
	firmware = malloc(firmware_length);
	if (!firmware)
		fatal("out of memory");
	if (read(fd, firmware, firmware_length) != firmware_length)
		fatal("cannot read from %s: %s", filename, strerror(errno));
	close(fd);
	logmsg("%s: %d bytes read successfully", filename, firmware_length);
}

static void parse_descriptors(void)
{
	struct usb_descriptor_header desc;
	int interface_found = 0;

	logmsg("reading device descriptor ...");
	for (;;) {
		if (read(device, &desc, 2) != 2)
			return;
		if (desc.bLength < 2) {
			logmsg("invalid descriptor length %d", desc.bLength);
			return;
		}
		if (desc.bDescriptorType == USB_DT_INTERFACE &&
		    desc.bLength >= 9) {
			unsigned char d[7];
			if (read(device, d, 7) != 7)
				return;
			logmsg("interface descriptor %d:%d", d[0], d[1]);
			if (d[3] == USB_CLASS_APP_SPEC &&
			    d[4] == USB_SUBCLASS_DFU) {
				interface_found = 1;
				interface = d[0];
				logmsg("DFU interface is %d", interface);
			} else {
				interface_found = 0;
			}
			desc.bLength -= 7;
		} else if (desc.bDescriptorType == USB_DT_DFU_FUNCTIONAL &&
			   desc.bLength >= 7 &&
			   interface_found) {
			unsigned char d[5];
			logmsg("DFU descriptor found");
			if (read(device, d, 5) != 5)
				return;
			transfer_size = d[3] | (d[4] << 8);
			if (transfer_size < 1)
				fatal("invalid transfer size %d", transfer_size);
			logmsg("transfer size is %d", transfer_size);
			desc.bLength -= 5;
		}
		/* skip (rest of) descriptor */
		if (desc.bLength > 2)
			if (lseek(device, desc.bLength - 2, SEEK_CUR) == (off_t)-1)
				return;
	}
}

static void dfu_dnload(void *buf, int length, int block)
{
	int err;
	
	err = usb_control(USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_OUT,
			  DFU_DNLOAD, block, interface,
			  length, buf);
	if (err < 0)
		fatal("downloading block %d failed", block);
}

static void dfu_status_wait()
{
	struct dfu_status status;
	int wait, err;

	err = usb_control(USB_TYPE_CLASS | USB_RECIP_INTERFACE | USB_DIR_IN,
			  DFU_GETSTATUS, 0, interface,
			  6, &status);
	if (err < 0)
		fatal("cannot get device status");
	switch (wait_type) {
#if 0
	case WAIT_STANDARD:
	/*
	 * This will be implemented when^H^H^H^Hif M-Audio builds a device
	 * that at least partially conforms to the DFU specification.
	 */
#endif
	case WAIT_IGNORE:
	default:
		wait = 0;
		break;
	case WAIT_BYTE3:
		wait = status.bwPollTimeout[2];
		break;
	}
	if (wait > 0) {
		struct timespec tv;
		logmsg("waiting %d ms", wait);
		tv.tv_sec = wait / 1000;
		tv.tv_nsec = (wait % 1000) * 1000000;
		if (nanosleep(&tv, NULL) == -1)
			fatal("sleep failed: %s", strerror(errno));
	}
}

static void download_firmware(char *device_file)
{
	unsigned short block = 0;
	int pos, length, err;

	device = open(device_file, O_RDWR);
	if (device == -1)
		fatal("cannot open %s: %s", device_file, strerror(errno));
	parse_descriptors();
	if (usb_claim_interface(interface) == -1)
		fatal("cannot claim interface %d: (%d) %s", interface, errno, strerror(errno));

	for (pos = 0; pos < firmware_length; pos += transfer_size) {
		if (firmware_length - pos > transfer_size)
			length = transfer_size;
		else
			length = firmware_length - pos;
		dfu_dnload(firmware + pos, length, block);
		++block;

		dfu_status_wait();
	}
	dfu_dnload(firmware, 0, block);
	dfu_status_wait();

	if (usb_reset() == -1)
		/* might fail because the device has already reset itself ... */
		warning("cannot reset device: (%d) %s", errno, strerror(errno));
	close(device);
}

static void help()
{
	fprintf(stderr,
		"Usage: %s options...\n"
		"\n"
		"-h, --help           help\n"
		"-V, --version        print version\n"
		"-f, --firmware=path  file containing the firmware to download\n"
		"-D, --device=path    usbfs file of the device (default: $DEVICE)\n"
		"-v, --verbose        output debugging messages\n"
		"-l, --logger         output to system logger instead of standard output\n"
		"-n, --nowait         don't wait between download requests\n"
		"-3, --waitbyte3      use only the third byte of the bwPollTimeout field\n",
		command);
}

static void version()
{
	fprintf(stderr, "%s version " VERSION "\n", command);
}

int main(int argc, char *argv[])
{
	static char short_options[] = "hVf:D:vln3";
	static struct option long_options[] = {
		{"help",      0, NULL, 'h'},
		{"version",   0, NULL, 'V'},
		{"firmware",  1, NULL, 'f'},
		{"device",    1, NULL, 'D'},
		{"verbose",   0, NULL, 'v'},
		{"logger",    0, NULL, 'l'},
		{"nowait",    0, NULL, 'n'},
		{"waitbyte3", 0, NULL, '3'},
		{ }
	};
	char *firmware = NULL;
	char *device = getenv("DEVICE");
	int c;

	command = argv[0];
	while ((c = getopt_long(argc, argv, short_options,
				long_options, NULL)) != -1) {
		switch (c) {
		case 'h':
			help();
			return 0;
		case 'V':
			version();
			return 0;
		case 'f':
			firmware = optarg;
			break;
		case 'D':
			device = optarg;
			break;
		case 'v':
			logging = 1;
			break;
		case 'l':
			to_logger = 1;
			break;
		case 'n':
			wait_type = WAIT_IGNORE;
			break;
		case '3':
			wait_type = WAIT_BYTE3;
			break;
		default:
			fprintf(stderr, "Try %s --help for more information.\n", command);
			return 1;
		}
	}
	if (!firmware) {
		fputs("Please specify a firmware file.\n", stderr);
		return 1;
	}
	if (wait_type == WAIT_STANDARD) {
		fputs("Please specify --nowait or --waitbyte3.\n", stderr);
		return 1;
	}
	if (to_logger)
		openlog(PACKAGE, 0, LOG_DAEMON);
	if (!device)
		fatal("--device not given, and DEVICE not set");

	load_firmware(firmware);
	download_firmware(device);

	return 0;
}
