/*
 * godfu.cc
 *
 * Author: David Guillen Fandos (2020) <david@davidgf.net>
 *
 * Forces the device into DFU mode in order to re-flash it
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

#define CMD_GO_DFU          0xff
#define VENDOR_ID         0x0483
#define PRODUCT_ID        0x5740
#define IFACE_NUMBER         0x0
#define TIMEOUT_MS          5000

static const int CTRL_REQ_TYPE_IN  = LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE;   // 0x21

void fatal_error(const char * errmsg, int code) {
	fprintf(stderr, "ERROR! %d %s\n", code, errmsg);
	exit(1);
}

int main(int argc, char ** argv) {
	int result = libusb_init(NULL);
	if (result < 0)
		fatal_error("libusb_init failed!", result);

	struct libusb_device_handle *devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);

	if (!devh)
		fatal_error("libusb_open_device_with_vid_pid failed to find a matching device!", 0);

	result = libusb_detach_kernel_driver(devh, IFACE_NUMBER);
	result = libusb_claim_interface(devh, IFACE_NUMBER);
	if (result < 0)
		fatal_error("libusb_claim_interface failed!", result);

	if (libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_GO_DFU, 0, IFACE_NUMBER, 0, 0, TIMEOUT_MS) < 0)
		fatal_error("Reboot into DFU command failed!", 0);

	libusb_release_interface(devh, 0);
	libusb_close(devh);
	libusb_exit(NULL);
	return 0;
}

