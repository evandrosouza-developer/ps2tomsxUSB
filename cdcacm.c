/*
 * This file is part of the PS/2 Keyboard Adapter for MSX, using libopencm3 project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>, based on:
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
 * But it is possible to update the table sending a Intel Hex File through serial or USB, by this effort
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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>

#include "system.h"
#include "cdcacm.h"
#include "serial.h"
#include "webusb.h"
#include "winusb.h"
#include "usb21_standard.h"
#include "logger.h"

#define USB_CLASS_MISCELLANEOUS 					0xEF  //  Copy from Blue Pill Bootloader
//#include "basic_usb_desc.c"

#if MCU == STM32F401CC
//https://github.com/libopencm3/libopencm3/pull/1256#	//Disable VBUS sensing #1256
#include <libopencm3/usb/dwc/otg_fs.h>
#endif

#define USB_CDC_REQ_GET_LINE_CODING				0x21

usbd_device *global_usbd_dev;

#if USE_USB == true
static int configured;
static int cdcacm_con_dtr = 1;
#endif		//#if USE_USB == true

extern struct ring console_tx_ring;
extern struct ring console_rx_ring;
extern struct ring serial_tx_ring;
extern struct ring serial_rx_ring;


//Prototypes
//void cdc_setup(usbd_device*);
void dump_usb_request(const char*, struct usb_setup_data*);
void usbuart_set_line_coding(struct usb_cdc_line_coding*);

//To place strings entirely into Flash/read-only memory, use
//static const * const strings[] = { ... };

uint8_t serial_no[LEN_SERIAL_No + 1];

#if USE_USB == true

static const char *usb_strings[] = {
	"Evandro de Souza Tecnology",								//  USB Manufacturer
	"PS/2 keyboard Interface for MSX",					//  USB Product
	(char*)serial_no,														//  Serial number
	"PS2MSX Console",														//  Console Port
	"PS2MSX Console ACM Port",									//  Console ACM Port
	"PS2MSX Console DataPort",									//  Console DATA Port
	/*"PS2MSX USB <-> Serial",									//  Serial Port
	"PS2MSX USB-Serial ACM Port",								//  Serial ACM Port
	"PS2MSX USB-Serial DataPort",								//  Serial DATA Port*/
};

enum usb_strings_index {  //  Index of USB strings. Must sync with above, starts from 1.
	USB_STRING_MANUFACTURER = 1,
	USB_STRING_PRODUCT,
	USB_STRING_SERIAL_NUMBER,
	USB_STRING_CONSOLE,
	USB_STRING_CON_COMM,
	USB_STRING_CON_DATA,
	/*USB_STRING_UART,
	USB_STRING_UART_COMM,
	USB_STRING_UART_DATA,*/
};


/*
 * The notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
// Console ACM interface
//  Console Endpoint Descriptors
static const struct usb_endpoint_descriptor con_comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,					//5 (as defined in usbstd.h)
	.bEndpointAddress = CON_COMM | USB_REQ_TYPE_IN,//0x82 Console USB IN Control Endpoint
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,//0x03 (as defined in usbstd.h)
	.wMaxPacketSize = COMM_PACKET_SIZE,  				//  Smaller than others: 16
	.bInterval = 255,
}	};

static const struct usb_endpoint_descriptor con_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,					//5 (as defined in usbstd.h)
	.bEndpointAddress = CON_DATA_OUT,						//0x01 Console USB OUT Data Endpoint
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,			//0x02 (as defined in usbstd.h)
	.wMaxPacketSize = MAX_USB_PACKET_SIZE,			//32
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,					//5 (as defined in usbstd.h)
	.bEndpointAddress = CON_DATA_IN,						//0x81 Console USB IN Control Endpoint
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,			//0x02 (as defined in usbstd.h)
	.wMaxPacketSize = MAX_USB_PACKET_SIZE,			//32
	.bInterval = 1,
} };

//  Console Functional Descriptor
static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors_console = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,					//0x24 (as defined in cdc.h)
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,//0x00 (as defined in cdc.h)
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,					//0x24 (as defined in cdc.h)
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,//0x01 (as defined in cdc.h)
		.bmCapabilities = 0,
		.bDataInterface = INTF_CON_DATA,  				//  DATA Interface
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,					//0x24 (as defined in cdc.h)
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,		//0x02 (as defined in cdc.h)
		.bmCapabilities = 2,											// SET_LINE_CODING supported
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,					//0x24 (as defined in cdc.h)
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,	//0x06 (as defined in cdc.h)
		.bControlInterface = INTF_CON_COMM,       //  COMM Interface
		.bSubordinateInterface0 = INTF_CON_DATA,  //  DATA Interface
	 }
};

//  Console Interface Descriptor
static const struct usb_interface_descriptor con_comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,						//9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE,				//4 (as defined in usbstd.h)
	.bInterfaceNumber = INTF_CON_COMM,  				//  CDC ACM interface ID is 0
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,						//0x02 (as defined in cdc.h)
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,	//0x02 (as defined in cdc.h)
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,//0x00 (as defined in cdc.h)
	//.iInterface = USB_STRING_CON_COMM,       	//  Name of CDC ACM interface (index of string descriptor)
	.iInterface = 0,									        	//  Name of CDC ACM interface (index of string descriptor)

	.endpoint = con_comm_endp,  								//  CDC ACM Endpoint (1 common endpoint)
	.extra = &cdcacm_functional_descriptors_console,
	.extralen = sizeof(cdcacm_functional_descriptors_console),
} };

static const struct usb_interface_descriptor con_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,						//9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE,				//4 (as defined in usbstd.h)
	.bInterfaceNumber = INTF_CON_DATA,  				//  CDC ACM interface ID is 1
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,					//0x0A (as defined in cdc.h)
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = USB_STRING_CON_DATA,	      	//  Name of CDC ACM interface (index of string descriptor)

	.endpoint = con_data_endp,  								//  CDC ACM Endpoint
}	};

//  Console Interface Association Descriptor - CDC Interfaces
static const struct usb_iface_assoc_descriptor con_iface_assoc = {  //  Copied from BMP.  Mandatory for composite device.
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,//8 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,//11 (as defined in usbstd.h)
	.bFirstInterface = INTF_CON_COMM, //  First associated interface (INTF_CON_COMM and INTF_CON_DATA)
	.bInterfaceCount = 2,          		//  Total number of associated interfaces (INTF_CON_COMM and INTF_CON_DATA), ID must be consecutive.
	.bFunctionClass = USB_CLASS_CDC,						//0x02 This is a USB CDC (Comms Device Class) interface
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,	//0x02 That implements ACM (Abstract Control Model)
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,		//0x01 Using the AT protocol
	.iFunction = USB_STRING_CONSOLE, 						//  Name of Serial Port
};


/*// UART ACM interface
//  UART Endpoint Descriptors
static const struct usb_endpoint_descriptor uart_comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,					//5 (as defined in usbstd.h)
	.bEndpointAddress = UART_COMM | USB_REQ_TYPE_IN,//0x84
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,//0x03 (as defined in usbstd.h)
	.wMaxPacketSize = COMM_PACKET_SIZE,
	.bInterval = 255,
} };

static const struct usb_endpoint_descriptor uart_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,					//5 (as defined in usbstd.h)
	.bEndpointAddress = UART_DATA_OUT,					//3
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,			//0x02 (as defined in usbstd.h)
	.wMaxPacketSize = MAX_USB_PACKET_SIZE,			//BMP uses MAX_USB_PACKET_SIZE / 2 here
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,					//5 (as defined in usbstd.h)
	.bEndpointAddress = UART_DATA_IN,						//0x83
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,			//0x02 (as defined in usbstd.h)
	.wMaxPacketSize = MAX_USB_PACKET_SIZE,
	.bInterval = 1,
} };


//  UART Functional Descriptor
static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors_uart = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = INTF_UART_DATA,  				//  DATA Interface
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 2,											// SET_LINE_CODING supported
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = INTF_UART_COMM,      //  COMM Interface
		.bSubordinateInterface0 = INTF_UART_DATA, //  DATA Interface
	 }
};


//  UART Interface Descriptor
static const struct usb_interface_descriptor uart_comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,						//9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = INTF_UART_COMM,  				//  CDC ACM interface ID is 2
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
	.iInterface = USB_STRING_UART_COMM,       	//  Name of CDC ACM interface (index of string descriptor)

	.endpoint = uart_comm_endp,  								//  CDC ACM Endpoint
	.extra = &cdcacm_functional_descriptors_uart,
	.extralen = sizeof(cdcacm_functional_descriptors_uart),
} };

static const struct usb_interface_descriptor uart_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,						//9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = INTF_UART_DATA,  				//  CDC ACM interface ID is 3
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = USB_STRING_UART_DATA,        	//  Name of CDC ACM interface (index of string descriptor)

	.endpoint = uart_data_endp,  								//  CDC ACM Endpoint
} };

//  UART Interface Association Descriptor - CDC Interfaces
static const struct usb_iface_assoc_descriptor uart_iface_assoc = {  //  Copied from BMP.  Mandatory for composite device.
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,//8 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = INTF_UART_COMM,//  First associated interface (INTF_UART_COMM and INTF_UART_DATA)
	.bInterfaceCount = 2,          		//  Total number of associated interfaces (INTF_UART_COMM and INTF_UART_DATA), ID must be consecutive.
	.bFunctionClass = USB_CLASS_CDC,						//  This is a USB CDC (Comms Device Class) interface
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,	//  That implements ACM (Abstract Control Model)
	.bFunctionProtocol = USB_CDC_PROTOCOL_NONE,	//  Using no protocol
	.iFunction = USB_STRING_UART,			  				//  Name of Serial Port
};*/



// Both Console and UART ACM interfaces
//  USB Configuration Descriptor (Both interfaces)
static const struct usb_interface interfaces[] = {
{
	.num_altsetting = 1,
	.iface_assoc = &con_iface_assoc,						//Mandatory for composite device with multiple interfaces. static const struct usb_iface_assoc_descriptor con_iface_assoc
	.altsetting = con_comm_iface,								//Points to static const struct usb_interface_descriptor con_comm_iface[]
},
{
	.num_altsetting = 1,
	.altsetting = con_data_iface,  							//Index must sync with INTF_CON_COMM.
}/*,
{
	.num_altsetting = 1,
	.iface_assoc = &uart_iface_assoc,						//Mandatory for composite device with multiple interfaces. static const struct usb_iface_assoc_descriptor uart_iface_assoc
	.altsetting = uart_comm_iface,							//Points to static const struct usb_interface_descriptor uart_comm_iface[]
},
{
	.num_altsetting = 1,
	.altsetting = uart_data_iface,  						//Index must sync with INTF_UART_COMM.
},*/ };


static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,				//  Length of this descriptor. 9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_CONFIGURATION,		//  This is a Configuration Descriptor (0x02)
	.wTotalLength =
		sizeof(struct usb_config_descriptor) + //Configuration Descriptor (9 bytes)
		// For Console Only: Total bytes = 75 (9 + 66)
		sizeof(struct usb_iface_assoc_descriptor) + //Interface Association Descriptor (8 bytes)
		sizeof(struct usb_interface_descriptor) + //Interface Descriptor (9 bytes)
		sizeof(struct usb_cdc_header_descriptor)+	//CDC Header Functional Descriptor (5 bytes)
		sizeof(struct usb_cdc_call_management_descriptor) + //CDC Call Management Functional Descriptor (5 bytes)
		sizeof(struct usb_cdc_acm_descriptor) 	+	//CDC Abstract Control Management Functional Descriptor (4 bytes)
		sizeof(struct usb_cdc_union_descriptor) + //CDC Union Functional Descriptor (5 bytes)
		sizeof(struct usb_endpoint_descriptor)	+	//Endpoint Descriptor (7 bytes)
		sizeof(struct usb_interface_descriptor) + //Interface Descriptor (9 bytes)
		sizeof(struct usb_endpoint_descriptor)	+	//Endpoint Descriptor (7 bytes)
		sizeof(struct usb_endpoint_descriptor),		//Endpoint Descriptor (7 bytes)

		// Now including UART: Total bytes = 141 (9 + 2*66)
		/*sizeof(struct usb_iface_assoc_descriptor) + \
		sizeof(struct usb_interface_descriptor) + \
		sizeof(struct usb_cdc_header_descriptor)+ \
		sizeof(struct usb_cdc_call_management_descriptor) + \
		sizeof(struct usb_cdc_acm_descriptor) 	+ \
		sizeof(struct usb_cdc_union_descriptor) + \
		sizeof(struct usb_endpoint_descriptor)	+ \
		sizeof(struct usb_interface_descriptor) + \
		sizeof(struct usb_endpoint_descriptor)	+ \
		sizeof(struct usb_endpoint_descriptor),*/
	.bNumInterfaces =	sizeof(interfaces) /	\
										sizeof(interfaces[0]),		//  We will have 4 interfaces
	.bConfigurationValue = 1,										//  This is the configuration ID 1
	.iConfiguration = 0,												//  Configuration string (0 means none)
	.bmAttributes = \
		USB_CONFIG_ATTR_DEFAULT | \
		USB_CONFIG_ATTR_SELF_POWERED,					 		//  Self-powered: it doesn't draw power from USB bus.
	.bMaxPower = 5,															//  Specifies how much bus current a device requires: 10 mA. (2 x 5)

	.interface = interfaces,										//  List of all interfaces
};


//Device Descriptor - The highest priority and first descriptor sent
static const struct usb_device_descriptor dev_desc = {
	.bLength = USB_DT_DEVICE_SIZE,							//  18 Length of this descriptor.
	.bDescriptorType = USB_DT_DEVICE,						//  This is a Device Descriptor
#if USB21_INTERFACE == true
	.bcdUSB = 0x0210,  //  USB Version 2.1.  Need to handle special requests e.g. BOS.
#else
	.bcdUSB = 0x0200,  //  USB Version 2.0.  No need to handle special requests e.g. BOS.
#endif  //  #if USB21_INTERFACE == true
#if CDC_ONLY_ON_USB == true  //  If we are providing serial interface only...
	.bDeviceClass = USB_CLASS_CDC,  						//  Set the class to CDC if it's only serial.  Serial interface will not start on Windows when class = 0.
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
#else  //  If we are providing multiple interfaces...
	.bDeviceClass = USB_CLASS_MISCELLANEOUS,  	//  Miscellaneous Class. For composite device, let host probe the interfaces. Uses an interface association descriptor
	.bDeviceSubClass = 2,  											//  Miscellaneous Common Class
	.bDeviceProtocol = 1,										  	//  Use Interface Association Descriptor
#endif  //  CDC_ONLY_ON_USB == true
	.bMaxPacketSize0 = MAX_USB_PACKET_SIZE,			//  USB packet size (64)
	.idVendor = USB_VID,												//  Official USB Vendor ID
	.idProduct = USB_PID,												//  Official USB Product ID
	.bcdDevice = 0x0100,												//  Device Release number 1.0
	.iManufacturer = USB_STRING_MANUFACTURER,	//  Name of manufacturer (index of string descriptor)
	.iProduct = USB_STRING_PRODUCT,						//  Name of product (index of string descriptor)
	.iSerialNumber = USB_STRING_SERIAL_NUMBER,	//  Serial number (index of string descriptor)
	.bNumConfigurations = 1,										//  How many configurations we support
};
#endif	//#if USE_USB == true



void serialno_read(uint8_t *s)
{
	uint16_t i, ii;

	// Fetch serial number from chip's unique ID
	// Use the same serial number as the ST DFU Bootloader.s
	uint16_t *uid = (uint16_t *)DESIG_UNIQUE_ID_BASE;
# if (MCU == STM32F401CC)
	int offset = 3;

	uint64_t unique_id =  ((uint64_t)(uid[1] + uid[5]) << 32) +
												((uint64_t)(uid[0] + uid[4]) << 16) +
												uid[offset];
# endif
	
# if (MCU == STM32F103C8)
	uint32_t unique_id = *unique_id_p +
			*(unique_id_p + 1) +
			*(unique_id_p + 2);
# endif

	//Convert each nibble to ASCII HEX
	for(i = 0; i < LEN_SERIAL_No; i++)
	{
		ii = LEN_SERIAL_No-1-i;
		s[ii] = ((unique_id >> (4*i)) & 0xF) + '0';
		if(s[ii] > '9')
			s[ii] += 'A' - '9' - 1;
	}
	s[LEN_SERIAL_No] = 0;

	return; //s
}

#if USE_USB == true
//  Line config to be returned.
static const struct usb_cdc_line_coding line_coding = {
	.dwDTERate = 115200,
	.bCharFormat = USB_CDC_1_STOP_BITS,
	.bParityType = USB_CDC_NO_PARITY,
	.bDataBits = 0x08
};

//Ideas taked from BMP
int cdcacm_get_config(void)
{
	return configured;
}

static int cdcacm_get_dtr(void)
{
	return cdcacm_con_dtr;
}

static void cdcacm_set_modem_state(usbd_device *usbd_dev, int iface, bool dsr, bool dcd)
{
	char buf[10];
	struct usb_cdc_notification *notif = (void*)buf;
	/* We echo signals back to host as notification */
	notif->bmRequestType = 0xA1;
	notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notif->wValue = 0;
	notif->wIndex = iface;
	notif->wLength = 2;
	buf[8] = (dsr ? 2 : 0) | (dcd ? 1 : 0);
	buf[9] = 0;
	usbd_ep_write_packet(usbd_dev, iface, buf, 10);
}


void usbuart_set_line_coding(struct usb_cdc_line_coding *coding)
{
	usart_set_baudrate(USART_PORT, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(USART_PORT, (coding->bDataBits + 1 <= 8 ? 8 : 9));
	else
		usart_set_databits(USART_PORT, (coding->bDataBits <= 8 ? 8 : 9));

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USART_PORT, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USART_PORT, USART_STOPBITS_1_5);
		break;
	case 2:
	default:
		usart_set_stopbits(USART_PORT, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USART_PORT, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USART_PORT, USART_PARITY_ODD);
		break;
	case 2:
	default:
		usart_set_parity(USART_PORT, USART_PARITY_EVEN);
		break;
	}
}


/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[USBD_CONTROL_BUFFER_SIZE];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device* usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device* usbd_dev, struct usb_setup_data *req))
{
	// This callback is called whenever a USB request is received.
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	if (req->bmRequestType == DEV_IN_bmReqTyp && req->bRequest == GET_DESCRIPTOR_bReq)
	{
		//  Dump the packet if GET_DESCRIPTOR.
		dump_usb_request(">> ", req); debug_flush(); ////
	} 

	if (req->bmRequestType == DEV_OUT_bmReqTyp && req->bRequest == SET_DESCRIPTOR_bReq)
	{
		//  Dump the packet if SET_DESCRIPTOR.
		dump_usb_request(">> ", req); debug_flush(); ////
	} 

	switch (req->bRequest)
	{
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:			// Defined in cdc.h (0x22)
		{
			/*
			* This Linux cdc_acm driver requires this to be implemented
			* even though it's optional in the CDC spec, and we don't
			* advertise it in the ACM functional descriptor.
			*/
			cdcacm_set_modem_state(usbd_dev, req->wIndex, true, true);
			/* Ignore if not for CON interface */
			if(req->wIndex != 0)
				return USBD_REQ_HANDLED;

			cdcacm_con_dtr = req->wValue & 1;

			return USBD_REQ_HANDLED;
		}
		case USB_CDC_REQ_GET_LINE_CODING:
		{
			//  Windows requires this request, not Mac nor Linux.
			//  From https://github.com/PX4/Bootloader/blob/master/stm32/cdcacm.c
			if ( *len < sizeof(struct usb_cdc_line_coding) ) {
				debug_print("*** cdcacm_control notsupp line_coding "); debug_print_unsigned(sizeof(struct usb_cdc_line_coding)); 
				debug_print(", len "); debug_print_unsigned(*len);
				debug_println(""); debug_flush(); ////
				return USBD_REQ_NOTSUPP;
			}
			*buf = (uint8_t *) &line_coding;
			*len = sizeof(struct usb_cdc_line_coding);
			return USBD_REQ_HANDLED;
		}
		case USB_CDC_REQ_SET_LINE_CODING:
		{
			if ( *len < sizeof(struct usb_cdc_line_coding) )
			{
				debug_print("*** cdcacm_control notsupp line_coding "); debug_print_unsigned(sizeof(struct usb_cdc_line_coding)); 
				debug_print(", len "); debug_print_unsigned(*len);
				debug_println(""); debug_flush(); ////
				return USBD_REQ_NOTSUPP;
			}
			switch(req->wIndex)
			{
				case 2:
					usbuart_set_line_coding((struct usb_cdc_line_coding*)*buf);
					return USBD_REQ_HANDLED;
				case 0:
					return USBD_REQ_HANDLED; // Ignore on COMM Port
				default:
					return USBD_REQ_NOTSUPP;
			}
		}
		default:
			return USBD_REQ_NOTSUPP;
	} //switch (req->bRequest)
	return USBD_REQ_NEXT_CALLBACK;  //  Previously USBD_REQ_NOTSUPP
}


static void cdcacm_con_usb_out_cb(usbd_device* usbd_dev, uint8_t ep)
{
	/*		This example is an echo code (all OUT chars are routed to IN: send back:)
	(void)ep;

	char buf[USBD_CONTROL_BUFFER_SIZE];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, USBD_CONTROL_BUFFER_SIZE);

	if (len) {
		while (usbd_ep_write_packet(usbd_dev, 0x82, buf, len) == 0);
	}*/
	uint16_t len, i;
	uint8_t buf[USBD_CONTROL_BUFFER_SIZE];
	(void)ep;
	len = usbd_ep_read_packet(usbd_dev, CON_DATA_OUT, buf, USBD_CONTROL_BUFFER_SIZE);

	if (len)
	{
		for(i = 0; i == len - 1; i++)
		{
			if(ring_put_ch(&console_rx_ring, buf[i]) == 0)
				//console_rx_ring is full
				break;	//quit transfer loop
		}
	}
}


static void cdcacm_con_usb_in_cb(usbd_device* usbd_dev, uint8_t ep)
{
	(void)ep;
	uint16_t i, qty_in_buffer, len;

	uint8_t buf[USBD_CONTROL_BUFFER_SIZE];
	len = (ring_avail_get_ch(&console_tx_ring));
	if (len > USBD_CONTROL_BUFFER_SIZE)
		len = USBD_CONTROL_BUFFER_SIZE;

	if (len)
	{
		for(i = 0; i <= (len - 1); i++)
		{
			buf[i] = (uint8_t)ring_get_ch(&console_tx_ring, &qty_in_buffer);
			if(qty_in_buffer == 0)
			{
				//console_tx_ring is empty
				break;	//quit transfer loop
			}
		}
		usbd_ep_write_packet(usbd_dev, CON_DATA_IN, buf, len);
	}
}


/*static void cdcacm_uart_data_rx_cb(usbd_device* usbd_dev, uint8_t ep)
{
	(void)ep;
	uint16_t len, i;
	uint8_t buf[USBD_CONTROL_BUFFER_SIZE];
	len = usbd_ep_read_packet(usbd_dev, UART_DATA_OUT, buf, USBD_CONTROL_BUFFER_SIZE);

	if (len)
	{
		for(i = 0; i == len - 1; i++)
		{
			if(ring_put_ch(&serial_tx_ring, buf[i]) == 0)
				//console_tx_ring is full
				break;
		}
	}
}


static void cdcacm_uart_data_tx_cb(usbd_device* usbd_dev, uint8_t ep)
{
	uint16_t i, qty_in_buffer, len;

	(void)ep;
	uint8_t buf[USBD_CONTROL_BUFFER_SIZE];
	len = (ring_avail_get_ch(&serial_rx_ring));
	if (len > USBD_CONTROL_BUFFER_SIZE)
		len = USBD_CONTROL_BUFFER_SIZE;

	if (len) {
		for(i = 0; i <= (len - 1); i++)
		{
				buf[i] = (uint8_t)ring_get_ch(&serial_rx_ring, &qty_in_buffer);
				if(qty_in_buffer == 0)
					//console_rx_ring is full
					break;
		}
		usbd_ep_write_packet(usbd_dev, UART_DATA_IN, buf, len);
	}
}*/


#if USB21_INTERFACE == true
//  BOS Capabilities for WebUSB and Microsoft Platform
static const struct usb_device_capability_descriptor* capabilities[] =
{
	(const struct usb_device_capability_descriptor*) &webusb_platform_capability_descriptor,
	(const struct usb_device_capability_descriptor*) &microsoft_platform_capability_descriptor,
};

//  BOS Descriptor for WebUSB and Microsoft Platform
static const struct usb_bos_descriptor bos_descriptor =
{
	.bLength = USB_DT_BOS_SIZE,
	.bDescriptorType = USB_DT_BOS,
	.bNumDeviceCaps = sizeof(capabilities) / sizeof(capabilities[0]),
	.capabilities = capabilities
};
#endif  //  #if USB21_INTERFACE == true


static void cdcacm_set_config(usbd_device* usbd_dev, uint16_t wValue)
{
	configured = wValue;

	int status;

	// Console interface
	usbd_ep_setup(usbd_dev, CON_COMM | USB_REQ_TYPE_IN, USB_ENDPOINT_ATTR_INTERRUPT, COMM_PACKET_SIZE, NULL);
	usbd_ep_setup(usbd_dev, CON_DATA_OUT, USB_ENDPOINT_ATTR_BULK, USBD_CONTROL_BUFFER_SIZE,
								cdcacm_con_usb_out_cb);
	usbd_ep_setup(usbd_dev, CON_DATA_IN, USB_ENDPOINT_ATTR_BULK, USBD_CONTROL_BUFFER_SIZE,
								cdcacm_con_usb_in_cb);

	// Serial interface
	/*usbd_ep_setup(usbd_dev, UART_COMM | USB_REQ_TYPE_IN, USB_ENDPOINT_ATTR_INTERRUPT, COMM_PACKET_SIZE, NULL);
	usbd_ep_setup(usbd_dev, UART_DATA_OUT, USB_ENDPOINT_ATTR_BULK, USBD_CONTROL_BUFFER_SIZE,
								cdcacm_uart_data_rx_cb);
	usbd_ep_setup(usbd_dev, UART_DATA_IN, USB_ENDPOINT_ATTR_BULK, USBD_CONTROL_BUFFER_SIZE,
								cdcacm_uart_data_tx_cb);*/

	//status = aggregate_register_callback(	
	status = usbd_register_control_callback(
						usbd_dev,
						USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
						USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
						cdcacm_control_request);
	if (status < 0) 
		debug_println("*** ERROR usbd_register_control_callback FAILED");
	else
		debug_println("usbd_register_control_callback Ok");
	debug_flush();

	/* Notify the host that DCD is asserted.
	 * Allows the use of /dev/tty* devices on *BSD/MacOS
	 * Taked from BMP
	 */
	cdcacm_set_modem_state(usbd_dev, INTF_CON_COMM , true, true);
	cdcacm_set_modem_state(usbd_dev, INTF_UART_COMM, true, true);
}


void usb_soft_connect(bool turn_action)
{
	//0: Normal operation. 1: The core generates a device disconnect event to the USB host.
	if (!turn_action)
		OTG_FS_DCTL &= ~OTG_FS_DCTL_SDIS;
	else
		OTG_FS_DCTL |= OTG_FS_DCTL_SDIS;
}


bool isUSBconnected(void)
{
	return OTG_FS_DSTS & OTG_FS_DSTS_SUSPSTS ? false : true;
}


struct control_callback_struct {
    uint8_t type;
    uint8_t type_mask;
    usbd_control_callback cb;
};


/*#define MAX_CONTROL_CALLBACK 10
static struct control_callback_struct control_callback[MAX_CONTROL_CALLBACK];
static usbd_set_config_callback config_callback[MAX_CONTROL_CALLBACK];


int aggregate_register_config_callback(usbd_device *usbd_dev, usbd_set_config_callback callback)
{
	//To avoid compiler warning: "unused parameter 'usbd_dev' [-Wunused-parameter]"
	void*local_usbd_dev = usbd_dev;
	local_usbd_dev += 0;

	//  Register the USB config callback.  We do this to overcome the 4 callback limit.
	int i;
	for (i = 0; i < MAX_CONTROL_CALLBACK; i++) {
		if (config_callback[i]) {
      if (config_callback[i] == callback) { return 0; }  //  Skip duplicate.
			continue;
		}
		config_callback[i] = callback;
		return 0;
	}
    debug_println("*** ERROR: Too many config callbacks"); debug_flush();
	return -1;
}


int aggregate_register_callback(usbd_device *usbd_dev, uint8_t type, uint8_t type_mask, usbd_control_callback callback)
{
	//To avoid compiler warning: unused parameter 'usbd_dev' [-Wunused-parameter]
	void*local_usbd_dev = usbd_dev;
	local_usbd_dev += 0;

	// Register application callback function for handling USB control requests.  We aggregate here so we can handle more than 4 callbacks.
	// debug_println("aggregate_register_callback"); ////
	int i;
	for (i = 0; i < MAX_CONTROL_CALLBACK; i++)
	{
		if (control_callback[i].cb)
		{
			//  If already exists, skip.
			if (control_callback[i].type == type &&
			control_callback[i].type_mask == type_mask &&
			control_callback[i].cb == callback)
			{
				//  debug_println("callback exists"); ////
				return 0;
			}
			continue;  //  Continue checking.
		}
		control_callback[i].type = type;
		control_callback[i].type_mask = type_mask;
		control_callback[i].cb = callback;
		return 0;
	}
    debug_println("*** ERROR: Too many control callbacks"); debug_flush();
	return -1;
}*/


static uint8_t usb_descriptor_type(uint16_t wValue) {
	return wValue >> 8;
}


static uint8_t usb_descriptor_index(uint16_t wValue) {
	return wValue & 0xFF;
}


void dump_usb_request(const char *msg, struct usb_setup_data *req)
{
	uint8_t desc_type = usb_descriptor_type(req->wValue);
	uint8_t desc_index = usb_descriptor_index(req->wValue);

	debug_print(msg);
	debug_print( " typ "); debug_printhex(req->bmRequestType);
	debug_print(", req "); debug_printhex(req->bRequest);
	debug_print(", val "); debug_printhex(req->wValue >> 8); debug_printhex(req->wValue & 0xff);
	debug_print(", idx "); debug_printhex(req->wIndex >> 8); debug_printhex(req->wIndex & 0xff);
	debug_print(", len "); debug_printhex(req->wLength >> 8); debug_printhex(req->wLength & 0xff);

	if (req->bmRequestType == DEV_OUT_bmReqTyp || req->bmRequestType == DEV_IN_bmReqTyp) {		//Only Device
		//  Dump USB standard requests.
		if (req->bmRequestType == DEV_IN_bmReqTyp && req->bRequest == GET_DESCRIPTOR_bReq) {
			debug_print(", GET_DES");
			switch(desc_type) {
				case 1: debug_print("_DEV"); break;
				case 2:	debug_print("_CFG"); break;
				case 3:	debug_print("_STR"); break;
				case 4:	debug_print("_INF"); break;
				case 5:	debug_print("_ENP"); break;
				case 15:debug_print("_BOS"); break;
				}
		} else if (req->bmRequestType == DEV_OUT_bmReqTyp && req->bRequest == SET_ADDRESS_bReq) {
			//  Note: We should see SET_ADDRESS only once per session.  If we see this again, it means
			//  we have previously returned invalid data to the host and the host is attempting to reset our connection.
			debug_print(", SET_ADR    ");
		} else if (req->bmRequestType == DEV_OUT_bmReqTyp && req->bRequest == SET_CONFIGURATION_bReq) {
			debug_print(", SET_CFG    ");
		} else if (req->bmRequestType == DEV_IN_bmReqTyp && req->bRequest == GET_CONFIGURATION_bReq) {
			debug_print(", GET_CFG    ");
		} else {
			debug_print(",");
		}
		debug_print(" t "); debug_printhex(desc_type); 	
		debug_print(" i "); debug_printhex(desc_index); 	
	}
	debug_println("");
}


/*static enum usbd_request_return_codes aggregate_callback(usbd_device *usbd_dev, 
			struct usb_setup_data *req, unsigned char **buf, short unsigned int *len,
			usbd_control_complete_callback *complete)
{
	//  This callback is called whenever a USB request is received.  We route to the right driver callbacks.
	int i, result = 0;
	//  Call the callbacks registered by the drivers.
	for (i = 0; i < MAX_CONTROL_CALLBACK; i++)
	{
		if (control_callback[i].cb == NULL) { break; }
		if ((req->bmRequestType & control_callback[i].type_mask) == control_callback[i].type) 
		{
			result = control_callback[i].cb(usbd_dev, req, buf, len, complete);
			if (result == USBD_REQ_HANDLED || result == USBD_REQ_NOTSUPP)
				return result;
		}
	}
	if (!(req->bmRequestType == 0x80 && req->bRequest == 0x06))
	{
		//  Dump the packet if not GET_DESCRIPTOR.
		dump_usb_request(">> ", req); debug_flush(); ////
	} 
	return USBD_REQ_NEXT_CALLBACK;
}*/


/*static void set_aggregate_control_callback(usbd_device *usbd_dev, uint16_t wValue)
{
	//  This callback is called when the device is updated.  We set our control callback.
	if (wValue != (uint16_t) -1)   //  If this is an actual callback, not a call by cdcacm_init()...
	{
		//  Call the config functions before setting our callback.
		for (int i = 0; i < MAX_CONTROL_CALLBACK; i++)
		{
			if (!config_callback[i]) { break; }
			(config_callback[i])(usbd_dev, wValue);
		}
	}
	//  Set our callback.
	//This registers a callback function that is called when the USB device receives a request on
	//the control channel.
	int status = usbd_register_control_callback(usbd_dev, 0, 0, aggregate_callback); //  Register for all notifications.
	if (status < 0)
	{ 
		debug_println("*** ERROR: set_aggregate_control_callback FAILED");
	}
	else
		debug_println("set_aggregate_control_callback Ok");
	debug_flush();
}*/
#endif		//#if USE_USB == true


void cdcacm_init(void)
{
#if USE_USB == true
	int status, num_strings;

	usbd_device *usbd_dev = global_usbd_dev;
	usbd_dev = NULL;

#if MCU == STM32F401CC
	gpio_mode_setup(USB_FS_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USB_FS_DM_PIN_ID | USB_FS_DP_PIN_ID);
	gpio_set_output_options(USB_FS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, USB_FS_DM_PIN_ID | USB_FS_DP_PIN_ID);
	gpio_set_af(USB_FS_PORT, GPIO_AF10, USB_FS_DM_PIN_ID | USB_FS_DP_PIN_ID);
#endif

	rcc_periph_clock_enable(RCC_CRC);

	num_strings = sizeof(usb_strings) / sizeof(usb_strings[0]);

	usbd_dev = usbd_init(&USB_DRIVER, &dev_desc, &config,
			usb_strings, num_strings,
			usbd_control_buffer, sizeof(usbd_control_buffer));

	global_usbd_dev = usbd_dev;

#if MCU == STM32F401CC
	//Disable VBUS sensing => https://github.com/libopencm3/libopencm3/pull/1256#
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
#if USART_PORT == USART1
	//Now recover A9 and A10 back to serial 1:
#define GPIO_USART_PIN_PORT				GPIOA
#define GPIO_USART_TX							GPIO9
#define GPIO_USART_RX							GPIO10
	gpio_set_af(GPIO_USART_PIN_PORT, GPIO_AF7, GPIO_USART_TX | GPIO_USART_RX);
#endif	//#if USART_PORT == USART1
#endif	//#if MCU == STM32F401CC

  //  Set the config callback.
	//status = aggregate_register_config_callback(usbd_dev, cdcacm_set_config);
	status = usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
	if (status < 0)
		//debug_println("*** aggregate_register_config_callback FAILED");
		debug_println("\r\n*** ERROR: usbd_register_set_config_callback FAILED");
	else
		//debug_println("aggregate_register_config_callback Ok");
		debug_println("\r\nusbd_register_set_config_callback Ok");
	debug_flush();

	//  For WinUSB: Windows probes the compatible ID before setting the configuration,
	//  so also register the callback now.
	/*set_aggregate_control_callback(usbd_dev, (uint16_t)0xFFFF);*/

#if USB21_INTERFACE == true
  //  Define USB 2.1 BOS interface used by WebUSB.
	usb21_setup(usbd_dev, &bos_descriptor);
	//webusb_setup(usbd_dev, origin_url);
	//winusb_setup(usbd_dev, INTF_DFU);  //  Previously INTF_DFU
#endif  //#if USB21_INTERFACE == true

	cdcacm_get_dtr();	//Kill this => Only to avoid warning

	nvic_set_priority(NVIC_OTG_FS_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(NVIC_OTG_FS_IRQ);
#endif			//#if USE_USB == true
}


void otg_fs_isr(void)
{
	usbd_poll(global_usbd_dev);
}
/*
OTG_FS device status register (OTG_FS_DSTS)
Address offset: 0x808
Reset value: 0x0000 0010

Bit 0 SUSPSTS: Suspend status
In device mode, this bit is set as long as a Suspend condition is detected on the USB. The
core enters the Suspended state when there is no activity on the USB data lines for a period
of 3 ms. The core comes out of the suspend:
– When there is an activity on the USB data lines
– When the application writes to the Remote


OTG_FS control and status register (OTG_FS_GOTGCTL)
Address offset: 0x000
Reset value: 0x0001 0000

Bit 19 BSVLD: B-session valid
Indicates the device mode transceiver status.
0: B-session is not valid.
1: B-session is valid.
In OTG mode, you can use this bit to determine if the device is connected or disconnected.
Note: Only accessible in device mode.


OTG_FS core interrupt register (OTG_FS_GINTSTS)
Address offset: 0x014
Reset value: 0x0400 0020

Bit 13 ENUMDNE: Enumeration done
The core sets this bit to indicate that speed enumeration is complete. The application must
read the OTG_FS_DSTS register to obtain the enumerated speed.
Note: Only accessible in device mode.

OTG_FS device control OUT endpoint 0 control register
(OTG_FS_DOEPCTL0)
Address offset: 0xB00
Reset value: 0x0000 8000
Bit 15 USBAEP: USB active endpoint
This bit is always set to 1, indicating that a control endpoint 0 is always active in all
configurations and interfaces.

//Example code http://forum.chibios.org/viewtopic.php?t=781
//
// Handles the USB driver global events.
//
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromIsr();

    // Enables the endpoints specified into the configuration.
    //   Note, this callback is invoked from an ISR so I-Class functions
    //   must be used.
    usbInitEndpointI(usbp, USB_CDC_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USB_CDC_INTERRUPT_REQUEST_EP, &ep2config);

    // Resetting the state of the CDC subsystem.
    sduConfigureHookI(usbp);

    chSysUnlockFromIsr();
    return;
  case USB_EVENT_SUSPEND:
    if (usbp->state == USB_ACTIVE)
    {
      // USB cable unplugged
      chSysLockFromIsr();
      _usb_reset(usbp);
      // Reset queues and unlock waiting threads
      chIQResetI(&SDU1.iqueue);
      chOQResetI(&SDU1.oqueue);
      chSysUnlockFromIsr();
    }
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

*/
