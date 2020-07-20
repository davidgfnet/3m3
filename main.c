/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2020 David Guillen Fandos <david@davidgf.net>
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/stm32/iwdg.h>

#include "stm32-bootloader/reboot.h"

static usbd_device *usbd_dev;

// We have 3 UARTS
#define UART_DEV_COUNT 3
static const uint32_t uarts[] = { USART1, USART2, USART3 };

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static unsigned min(unsigned a, unsigned b, unsigned c) {
	if (a < b)
		return MIN(a, c);
	return MIN(b, c);
}

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
#define SERIAL_INTERFACE(CNTRL_NUM) \
	static const struct usb_endpoint_descriptor comm_endp##CNTRL_NUM[] = {{ \
		.bLength = USB_DT_ENDPOINT_SIZE, \
		.bDescriptorType = USB_DT_ENDPOINT, \
		.bEndpointAddress =  0x85 + CNTRL_NUM /*0x80 | (CNTRL_NUM * 2 + 2)*/ , \
		.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT, \
		.wMaxPacketSize = 16, \
		.bInterval = 255, \
	} }; \
	 \
	static const struct usb_endpoint_descriptor data_endp##CNTRL_NUM[] = {{ \
		.bLength = USB_DT_ENDPOINT_SIZE, \
		.bDescriptorType = USB_DT_ENDPOINT, \
		.bEndpointAddress = (CNTRL_NUM + 1), \
		.bmAttributes = USB_ENDPOINT_ATTR_BULK, \
		.wMaxPacketSize = 32, \
		.bInterval = 1, \
	}, { \
		.bLength = USB_DT_ENDPOINT_SIZE, \
		.bDescriptorType = USB_DT_ENDPOINT, \
		.bEndpointAddress = 0x80 | (CNTRL_NUM + 1), \
		.bmAttributes = USB_ENDPOINT_ATTR_BULK, \
		.wMaxPacketSize = 32, \
		.bInterval = 1, \
	} }; \
	 \
	static const struct { \
		struct usb_cdc_header_descriptor header; \
		struct usb_cdc_call_management_descriptor call_mgmt; \
		struct usb_cdc_acm_descriptor acm; \
		struct usb_cdc_union_descriptor cdc_union; \
	} __attribute__((packed)) cdcacm_functional_descriptors##CNTRL_NUM = { \
		.header = { \
			.bFunctionLength = sizeof(struct usb_cdc_header_descriptor), \
			.bDescriptorType = CS_INTERFACE, \
			.bDescriptorSubtype = USB_CDC_TYPE_HEADER, \
			.bcdCDC = 0x0110, \
		}, \
		.call_mgmt = { \
			.bFunctionLength = \
				sizeof(struct usb_cdc_call_management_descriptor), \
			.bDescriptorType = CS_INTERFACE, \
			.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT, \
			.bmCapabilities = 0, \
			.bDataInterface = 1 + CNTRL_NUM * 2,   /* Data Iface */  \
		}, \
		.acm = { \
			.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor), \
			.bDescriptorType = CS_INTERFACE, \
			.bDescriptorSubtype = USB_CDC_TYPE_ACM, \
			.bmCapabilities = 0, \
		}, \
		.cdc_union = { \
			.bFunctionLength = sizeof(struct usb_cdc_union_descriptor), \
			.bDescriptorType = CS_INTERFACE, \
			.bDescriptorSubtype = USB_CDC_TYPE_UNION, \
			.bControlInterface = CNTRL_NUM * 2,   /* Control Iface */ \
			.bSubordinateInterface0 = 1 + CNTRL_NUM * 2,  /* Data Iface */ \
		 } \
	}; \
 \
	static const struct usb_interface_descriptor comm_iface##CNTRL_NUM[] = {{ \
		.bLength = USB_DT_INTERFACE_SIZE, \
		.bDescriptorType = USB_DT_INTERFACE, \
		.bInterfaceNumber = CNTRL_NUM * 2, \
		.bAlternateSetting = 0, \
		.bNumEndpoints = 1, \
		.bInterfaceClass = USB_CLASS_CDC, \
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM, \
		.bInterfaceProtocol = USB_CDC_PROTOCOL_AT, \
		.iInterface = 0, \
		.endpoint = comm_endp##CNTRL_NUM, \
		.extra = &cdcacm_functional_descriptors##CNTRL_NUM, \
		.extralen = sizeof(cdcacm_functional_descriptors##CNTRL_NUM) \
	} }; \
	 \
	static const struct usb_interface_descriptor data_iface##CNTRL_NUM[] = {{ \
		.bLength = USB_DT_INTERFACE_SIZE, \
		.bDescriptorType = USB_DT_INTERFACE, \
		.bInterfaceNumber = CNTRL_NUM * 2 + 1, \
		.bAlternateSetting = 0, \
		.bNumEndpoints = 2, \
		.bInterfaceClass = USB_CLASS_DATA, \
		.bInterfaceSubClass = 0, \
		.bInterfaceProtocol = 0, \
		.iInterface = 0, \
		.endpoint = data_endp##CNTRL_NUM, \
	} };

#define IFACE(NUM) \
	{ \
		.num_altsetting = 1, \
		.altsetting = comm_iface##NUM, \
	}, { \
		.num_altsetting = 1, \
		.altsetting = data_iface##NUM, \
	}


// Define three copies of the required descriptors, with the right EPs and Iface numbers
SERIAL_INTERFACE(0)
SERIAL_INTERFACE(1)
SERIAL_INTERFACE(2)

static const struct usb_interface ifaces[] = {
	IFACE(0),
	IFACE(1),
	IFACE(2),
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2 * UART_DEV_COUNT,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static char serial_no[25] = {0};

static void get_dev_unique_id(char *s) {
	volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
	int i;

	/* Fetch serial number from chip's unique ID */
	for(i = 0; i < 24; i+=2) {
		s[i] = ((*unique_id >> 4) & 0xF) + '0';
		s[i+1] = (*unique_id++ & 0xF) + '0';
	}
	for(i = 0; i < 24; i++)
		if(s[i] > '9')
			s[i] += 'A' - '9' - 1;
}

static const char * usb_strings[] = {
	"davidgf.net",
	"Tri-GSM Modem (STM32 + SIM800L)",
	serial_no,
};

// Some custom sutff int the EP0 for rebooting and poking
void reboot_info_dfu() {
	// Reboot into DFU!
	reboot_into_bootloader();
	scb_reset_system();
}

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[1024];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;

		unsigned idx = (req->wIndex >> 1) & 3;
		const struct usb_cdc_line_coding *req = (struct usb_cdc_line_coding*)(*buf);
		usart_disable(uarts[idx]);
		usart_set_baudrate(uarts[idx], req->dwDTERate);
		usart_set_databits(uarts[idx], req->bDataBits);
		usart_set_stopbits(uarts[idx], req->bCharFormat == USB_CDC_1_STOP_BITS   ? USART_STOPBITS_1 :
		                               req->bCharFormat == USB_CDC_1_5_STOP_BITS ? USART_STOPBITS_1_5 : USART_STOPBITS_2);
		usart_set_mode    (uarts[idx], USART_MODE_TX_RX);
		usart_set_parity  (uarts[idx], req->bParityType == USB_CDC_NO_PARITY  ? USART_PARITY_NONE :
		                               req->bParityType == USB_CDC_ODD_PARITY ? USART_PARITY_EVEN : USART_PARITY_ODD);
		usart_enable(uarts[idx]);

		return USBD_REQ_HANDLED;
	case 0xFF:
		*len = 0;
		*complete = reboot_info_dfu;
		break;
	}
	return USBD_REQ_NOTSUPP;
}

#define USART_CAN_SEND(x) \
	((USART_SR(x) & USART_SR_TXE) != 0)


// Generic data-received bulk endpoint

#define BUF_SIZE       1024
#define BUF_SIZE_NAK    512   // Threshold after we indicate we can't take any more data
#define BUF_SIZE_THR    896   // Do not store more than this to avoid overflows
#define BUFCNT(b) (((b).writeptr - (b).readptr) & (BUF_SIZE - 1))

typedef struct {
	uint32_t readptr, writeptr;
	uint8_t buffer[BUF_SIZE];
} t_buffer;

t_buffer host2dev_buffer[UART_DEV_COUNT] = {0};
t_buffer dev2host_buffer[UART_DEV_COUNT] = {0};

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
	unsigned devn = ep - 1;
	t_buffer *buf = &host2dev_buffer[devn];

	uint8_t tmpb[64];
	int len = usbd_ep_read_packet(usbd_dev, ep, tmpb, sizeof(tmpb));

	// Stop the endpoint before we get to overflow the buffer
	if (BUFCNT(*buf) > BUF_SIZE_NAK)
		usbd_ep_nak_set(usbd_dev, ep, 1);

	// Unfortunately drop data, this should not happen due to previous NAK
	if (BUFCNT(*buf) > BUF_SIZE_THR)
		return;

	unsigned p = buf->writeptr;
	for (unsigned i = 0; i < len; i++) {
		buf->buffer[p] = tmpb[i];
		p = (p + 1) & (BUF_SIZE - 1);
	}
	buf->writeptr = p;

	gpio_toggle(GPIOC, GPIO13);
}

void usart_isr(int devn, uint8_t data) {
	t_buffer *buf = &dev2host_buffer[devn];

	// Unfortunately drop data, this should not happen often (USB is faster)
	if (BUFCNT(*buf) > BUF_SIZE_THR)
		return;

	buf->buffer[buf->writeptr] = data;
	buf->writeptr = (buf->writeptr + 1) & (BUF_SIZE - 1);

	gpio_toggle(GPIOC, GPIO13);
}

void usart1_isr(void) {
	while (usart_get_flag(USART1, USART_SR_RXNE))
		usart_isr(0, usart_recv(USART1));
}
void usart2_isr(void) {
	while (usart_get_flag(USART2, USART_SR_RXNE))
		usart_isr(1, usart_recv(USART2));
}
void usart3_isr(void) {
	while (usart_get_flag(USART3, USART_SR_RXNE))
		usart_isr(2, usart_recv(USART3));
}


static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	for (unsigned i = 0; i < UART_DEV_COUNT; i++) {
		usbd_ep_setup(usbd_dev, 0x01 + i, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
		usbd_ep_setup(usbd_dev, 0x81 + i, USB_ENDPOINT_ATTR_BULK, 64, NULL);
		//usbd_ep_setup(usbd_dev, 0x82 + i*2, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
	}

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

// Setup the four UART/USARTs
static void usart_setup(void) {
	// Enable interrupts for these devices
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_enable_irq(NVIC_USART3_IRQ);

	// Manually setup the GPIOs
	// USART1: PA9 and PA10
	gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIO_BANK_USART1_RX, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	// USART2: PA2 and PA3
	gpio_set_mode(GPIO_BANK_USART2_TX, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
	gpio_set_mode(GPIO_BANK_USART2_RX, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	// USART3: PB10 and PB11
	gpio_set_mode(GPIO_BANK_USART3_TX, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
	gpio_set_mode(GPIO_BANK_USART3_RX, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

	// Enable interrupts for RX buffer
	for (unsigned i = 0; i < UART_DEV_COUNT; i++)
		usart_enable_rx_interrupt(uarts[i]);

	for (unsigned i = 0; i < UART_DEV_COUNT; i++) {
		usart_set_baudrate(uarts[i], 115200);
		usart_set_databits(uarts[i], 8);
		usart_set_stopbits(uarts[i], USART_STOPBITS_1);
		usart_set_mode    (uarts[i], USART_MODE_TX_RX);
		usart_set_parity  (uarts[i], USART_PARITY_NONE);
		usart_set_flow_control(uarts[i], USART_FLOWCONTROL_NONE);
		usart_enable(uarts[i]);
	}
}

void init_low_power_modes() {
	// Enable clock gating the Cortex on WFI/WFE
	// Make sure we continue running after returning from ISR
	SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
	SCB_SCR &= ~SCB_SCR_SLEEPONEXIT;
	pwr_voltage_regulator_on_in_stop();
	pwr_set_stop_mode();
}

void usb_suspend_callback() {
	*USB_CNTR_REG |= USB_CNTR_FSUSP;
	*USB_CNTR_REG |= USB_CNTR_LP_MODE;
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
	__WFI();
}

void usb_wakeup_isr() {
	exti_reset_request(EXTI18);
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	*USB_CNTR_REG &= ~USB_CNTR_FSUSP;
}

void reenumerate_usb() {
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned int i = 0; i < 800000; i++)
		__asm__("nop");
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO12);
}

void usb_lp_can_rx0_isr(void) {
	usbd_poll(usbd_dev);
}

void start_usb() {
	get_dev_unique_id(serial_no);

	// Force USB connection
	reenumerate_usb();

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
			     usb_strings, 3,
			     usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, ~0);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

int main(void) {
	clear_reboot_flags();  // Make sure accidental reboot doesn't result into rebooting into DFU
	iwdg_reset();  // Watchdog ping

	// Interrupt priorities
	scb_set_priority_grouping(SCB_AIRCR_PRIGROUP_GROUP4_SUB4);

	// Start USB machinery
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	start_usb();

	init_low_power_modes();

	rcc_periph_clock_enable(RCC_GPIOA);   // For USARTs
	rcc_periph_clock_enable(RCC_GPIOB);   // For USARTs
	rcc_periph_clock_enable(RCC_USART1);  // USART clocks!
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_USART3);

	// LED
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	usart_setup();

	while (1) {
		for (unsigned i = 0; i < UART_DEV_COUNT; i++) {
			// Check whether we have data to send via UART
			if (BUFCNT(host2dev_buffer[i]) > 0 && USART_CAN_SEND(uarts[i])) {
				usart_send(uarts[i], host2dev_buffer[i].buffer[host2dev_buffer[i].readptr]);
				host2dev_buffer[i].readptr = (host2dev_buffer[i].readptr + 1) & (BUF_SIZE - 1);

				// Unblock USB once buffer has some space left
				if (BUFCNT(host2dev_buffer[i]) < BUF_SIZE_NAK)
					usbd_ep_nak_set(usbd_dev, 0x01 + i, 0);
			}

			// Check for the other direction now
			if (BUFCNT(dev2host_buffer[i]) > 0) {
				unsigned avail = BUFCNT(dev2host_buffer[i]);
				unsigned maxeb = BUF_SIZE - dev2host_buffer[i].readptr;
				unsigned tosend = min(avail, maxeb, 64);

				unsigned sent = 0;
				do {
					sent = usbd_ep_write_packet(usbd_dev, 0x81 + i, &dev2host_buffer[i].buffer[dev2host_buffer[i].readptr], tosend);
				} while (!sent);

				dev2host_buffer[i].readptr = (dev2host_buffer[i].readptr + sent) & (BUF_SIZE - 1);
			}
		}
	}
}


