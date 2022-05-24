/*
 * This file is part of the PS/2 Keyboard Adapter for MSX, using libopencm3 project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
 * But it is possible to update the table sending a Intel Hex File through serial or USB
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

 //Use Tab width=2

#include "system.h"														//Local file with definitions of the design

uint8_t serial_no[LEN_SERIAL_No + 1];

#if USE_USB == true

static const char *usb_strings[] = {
	"Evandro de Souza Tecnology",								//  USB Manufacturer
	"PS/2 keyboard Interface for MSX",					//  USB Product
	(char*)serial_no,														//  Serial number
	"PS2MSX Console",														//  Console Port
	"PS2MSX Console ACM Port",									//  Console ACM Port
	"PS2MSX Console DataPort",									//  Console DATA Port
	"PS2MSX USB <-> Serial",										//  Serial Port
	"PS2MSX USB-Serial ACM Port",								//  Serial ACM Port
	"PS2MSX USB-Serial DataPort",								//  Serial DATA Port
};

enum usb_strings_index {  //  Index of USB strings. Must sync with above, starts from 1.
	USB_STRING_MANUFACTURER = 1,
	USB_STRING_PRODUCT,
	USB_STRING_SERIAL_NUMBER,
	USB_STRING_CONSOLE,
	USB_STRING_CON_COMM,
	USB_STRING_CON_DATA,
	USB_STRING_UART,
	USB_STRING_UART_COMM,
	USB_STRING_UART_DATA,
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
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CON_COMM | 0x80,				//Console USB IN Control Endpoint
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = COMM_PACKET_SIZE,  				//  Smaller than others: 16
	.bInterval = 255,
} };

static const struct usb_endpoint_descriptor con_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CON_DATA_OUT,						//Console USB OUT Data Endpoint
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = MAX_USB_PACKET_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CON_DATA_IN,						//Console USB IN Control Endpoint
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = MAX_USB_PACKET_SIZE,
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
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = INTF_CON_DATA,  				//  DATA Interface
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
		.bControlInterface = INTF_CON_COMM,       //  COMM Interface
		.bSubordinateInterface0 = INTF_CON_DATA,  //  DATA Interface
	 }
};

//  Console Interface Descriptor
static const struct usb_interface_descriptor con_comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,						//9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = INTF_CON_COMM,  				//  CDC ACM interface ID is 0
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
	//.iInterface = USB_STRING_CON_COMM,       	//  Name of CDC ACM interface (index of string descriptor)
	.iInterface = 0,									        	//  Name of CDC ACM interface (index of string descriptor)

	.endpoint = con_comm_endp,  								//  CDC ACM Endpoint (1 common endpoint)
	.extra = &cdcacm_functional_descriptors_console,
	.extralen = sizeof(cdcacm_functional_descriptors_console),
} };

static const struct usb_interface_descriptor con_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,						//9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = INTF_CON_DATA,  				//  CDC ACM interface ID is 1
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = USB_STRING_CON_DATA,	      	//  Name of CDC ACM interface (index of string descriptor)
	//.iInterface = 0,								        	//  Name of CDC ACM interface (index of string descriptor)

	.endpoint = con_data_endp,  								//  CDC ACM Endpoint
} };

//  Console Interface Association Descriptor - CDC Interfaces
static const struct usb_iface_assoc_descriptor con_iface_assoc = {  //  Copied from BMP.  Mandatory for composite device.
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,//8 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = INTF_CON_COMM, //  First associated interface (INTF_CON_COMM and INTF_CON_DATA)
	.bInterfaceCount = 2,          		//  Total number of associated interfaces (INTF_CON_COMM and INTF_CON_DATA), ID must be consecutive.
	.bFunctionClass = USB_CLASS_CDC,						//  This is a USB CDC (Comms Device Class) interface
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,	//  That implements ACM (Abstract Control Model)
	.bFunctionProtocol = USB_CDC_PROTOCOL_AT,		//  Using the AT protocol
	.iFunction = USB_STRING_CONSOLE, 						//  Name of Serial Port
};


// UART ACM interface
//  UART Endpoint Descriptors
static const struct usb_endpoint_descriptor uart_comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = UART_COMM | 0x80,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = COMM_PACKET_SIZE,
	.bInterval = 255,
} };

static const struct usb_endpoint_descriptor uart_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = UART_DATA_OUT,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = MAX_USB_PACKET_SIZE,			//BMP uses MAX_USB_PACKET_SIZE / 2 here
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,						//7 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = UART_DATA_IN,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
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
};



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
},
{
	.num_altsetting = 1,
	.iface_assoc = &uart_iface_assoc,						//Mandatory for composite device with multiple interfaces. static const struct usb_iface_assoc_descriptor uart_iface_assoc
	.altsetting = uart_comm_iface,							//Points to static const struct usb_interface_descriptor uart_comm_iface[]
},
{
	.num_altsetting = 1,
	.altsetting = uart_data_iface,  						//Index must sync with INTF_UART_COMM.
}, };


static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,				//  Length of this descriptor. 9 (as defined in usbstd.h)
	.bDescriptorType = USB_DT_CONFIGURATION,		//  This is a Configuration Descriptor (0x02)
	.wTotalLength =
		sizeof(struct usb_config_descriptor) + //Configuration Descriptor (9 bytes)
		// For Console
		sizeof(struct usb_iface_assoc_descriptor) + //Interface Association Descriptor (8 bytes)
		sizeof(struct usb_interface_descriptor) + //Interface Descriptor (9 bytes)
		sizeof(struct usb_cdc_header_descriptor)+	//CDC Header Functional Descriptor (5 bytes)
		sizeof(struct usb_cdc_call_management_descriptor) + //CDC Call Management Functional Descriptor (5 bytes)
		sizeof(struct usb_cdc_acm_descriptor) 	+	//CDC Abstract Control Management Functional Descriptor (4 bytes)
		sizeof(struct usb_cdc_union_descriptor) + //CDC Union Functional Descriptor (5 bytes)
		sizeof(struct usb_endpoint_descriptor)	+	//Endpoint Descriptor (7 bytes)
		sizeof(struct usb_interface_descriptor) + //Interface Descriptor (9 bytes)
		sizeof(struct usb_endpoint_descriptor)	+	//Endpoint Descriptor (7 bytes)
		sizeof(struct usb_endpoint_descriptor)	+	//Endpoint Descriptor (7 bytes)
		// Now for UART
		sizeof(struct usb_iface_assoc_descriptor) + \
		sizeof(struct usb_interface_descriptor) + \
		sizeof(struct usb_cdc_header_descriptor)+ \
		sizeof(struct usb_cdc_call_management_descriptor) + \
		sizeof(struct usb_cdc_acm_descriptor) 	+ \
		sizeof(struct usb_cdc_union_descriptor) + \
		sizeof(struct usb_endpoint_descriptor)	+ \
		sizeof(struct usb_interface_descriptor) + \
		sizeof(struct usb_endpoint_descriptor)	+ \
		sizeof(struct usb_endpoint_descriptor),// Total # of bytes (9 + 2*66) = 141 bytes
	.bNumInterfaces =	sizeof(interfaces) /	\
										sizeof(interfaces[0]),		//  We will have 4 interfaces
	.bConfigurationValue = 1,										//  This is the configuration ID 1
	.iConfiguration = 0,												//  Configuration string (0 means none)
	.bmAttributes = \
		USB_CONFIG_ATTR_DEFAULT | \
		USB_CONFIG_ATTR_SELF_POWERED,					 		//  Self-powered: it doesn't draw power from USB bus.
	//.bmAttributes = 0x80, 										//  Bus powered: it draws power from USB bus.
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
