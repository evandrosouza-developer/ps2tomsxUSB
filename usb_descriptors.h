/** @addtogroup 06 USB USB_Group
 *
 * @ingroup infrastructure_apis
 *
 * @file usb_descriptors.h Defines the USB descriptors. Header file of cdcacm.c.
 *
 * @brief <b>Defines the USB descriptors: Header file of cdcacm.c</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 01 September 2022
 *
 * This library supports the USART with DMA in the STM32F4 and STM32F1
 * series of ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 * This file is part of the MSX Keyboard Subsystem Emulator project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * Based on CDCACM example of:
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

#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dfu.h>


#include "system.h"
#include "serial_no.h"
#include "version.h"


/**  Defines the Board identification + Firmware version
 *
@{*/
//#define BOARD_IDENT "MSX keyboard subsystem emulator " FIRMWARE_VERSION
#define BOARD_IDENT DESIGN_DEF HARDWARE_BASE FIRMWARE_VERSION
/**@}*/

/**  Get locale of serial_no
 *
@{*/
extern char serial_no[LEN_SERIAL_No + 1];     //Declared as uint8_t on serial_no.c
/**@}*/

#if USE_USB == true 
static const char *usb_strings[] = {
  "Evandro Rodrigues de Souza Technologies",
  BOARD_IDENT,
  serial_no,
  DESIGN_DEF "Console",                       //  Console Port
  DESIGN_DEF "Console ACM Port",              //  Console ACM Port
  DESIGN_DEF "Console DataPort",              //  Console DATA Port
  DESIGN_DEF "Converter USB <-> Serial",      //  Serial Port
  DESIGN_DEF "USB-Serial ACM Port",           //  Serial ACM Port
  DESIGN_DEF "USB-Serial DataPort",           //  Serial DATA Port
};

#define NUM_STRINGS (sizeof(usb_strings) / sizeof(usb_strings[0]))

enum usb_strings_index {  //  Index of USB strings.  Must sync with *usb_strings[], starts from 1.
  USB_STRINGS_MANUFACTURER = 1,
  USB_STRINGS_PRODUCT,
  USB_STRINGS_SERIAL_NUMBER,
  USB_STRINGS_CONSOLE,
  USB_STRINGS_CON_COMM,
  USB_STRINGS_CON_DATA,
  USB_STRINGS_UART,
  USB_STRINGS_UART_COMM,
  USB_STRINGS_UART_DATA,
};

static const struct usb_device_descriptor dev_desc = {
  .bLength = USB_DT_DEVICE_SIZE,              //  18: Length of this descriptor.
  .bDescriptorType = USB_DT_DEVICE,           //  This is a Device Descriptor
#if USB21_INTERFACE == true
  .bcdUSB = 0x0210,  //  USB Version 2.1.  Need to handle special requests e.g. BOS.
#else
  .bcdUSB = 0x0200,  //  USB Version 2.0.  No need to handle special requests e.g. BOS.
#endif  //  #if USB21_INTERFACE == true
#if (CDC_ONLY_ON_USB == true)
  .bDeviceClass = USB_CLASS_CDC,              //0x02 Set the class to CDC if it's only serial.  Serial interface will not start on Windows when class = 0.
  .bDeviceSubClass = 0,                       //Communications Device Subclass code, unused at this time.
  .bDeviceProtocol = 0,                       //Communications Device Subclass code, unused at this time.
#else //#if (CDC_ONLY_ON_USB == true)
  .bDeviceClass = USB_CLASS_MISCELLANEOUS,    //0xEF Set the class to CDC to miscellaneous.
  .bDeviceSubClass = 2,                       //0xEF, 0x02, 0x01 is Interface Association Descriptor
  .bDeviceProtocol = 1,                       //0xEF, 0x02, 0x01 is Interface Association Descriptor
#endif  //#if (CDC_ONLY_ON_USB == true)
  .bMaxPacketSize0 = USBD_DATA_BUFFER_SIZE,   //  USB packet size (64)
  .idVendor = USB_VID,                        //  Official USB Vendor ID
  .idProduct = USB_PID,                       //  Official USB Product ID
  .bcdDevice = 0x0100,                        //  Device Release number 1.0
  .iManufacturer = USB_STRINGS_MANUFACTURER,  //  Name of manufacturer (index of string descriptor)
  .iProduct = USB_STRINGS_PRODUCT,            //  Name of product (index of string descriptor)
  .iSerialNumber = USB_STRINGS_SERIAL_NUMBER, //  Serial number (index of string descriptor)
  .bNumConfigurations = 1,                    //  How many configurations we support
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
// Console ACM interface
//  CON Endpoint Descriptors
static const struct usb_endpoint_descriptor con_comm_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,            //7 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_ENDPOINT,         //5 (as defined in usbstd.h)
  .bEndpointAddress = EP_CON_COMM_IN,         //0x82 Console USB IN Control Endpoint//EP_CON_COMM_IN=0x83 originally
  .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,//0x03 (as defined in usbstd.h)
  .wMaxPacketSize = COMM_PACKET_SIZE,         //16 - Smaller than others
  .bInterval = 255,
}};

static const struct usb_endpoint_descriptor con_data_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,            //7 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_ENDPOINT,         //5 (as defined in usbstd.h)
  .bEndpointAddress = EP_CON_DATA_OUT,        //0x01 Console USB OUT Data Endpoint//EP_CON_DATA_OUT=0x01 originally
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,     //0x02 (as defined in usbstd.h)
  .wMaxPacketSize = USBD_DATA_BUFFER_SIZE,    //64
  .bInterval = 1,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,            //7 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_ENDPOINT,         //5 (as defined in usbstd.h)
  .bEndpointAddress = EP_CON_DATA_IN,         //0x01 Console USB OUT Data Endpoint//EP_CON_DATA_IN=0x82 originally
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,     //0x02 (as defined in usbstd.h)
  .wMaxPacketSize = USBD_DATA_BUFFER_SIZE,    //64
  .bInterval = 1,
}};

//  CON Functional Descriptor
static const struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) con_cdcacm_functional_descriptors = {
  .header = {
    .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,//0x00 (as defined in cdc.h)
    .bcdCDC = 0x0110,
  },
  .call_mgmt = {
    .bFunctionLength =
      sizeof(struct usb_cdc_call_management_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,//0x01 (as defined in cdc.h)
    .bmCapabilities = 0,
    .bDataInterface = INTF_CON_DATA,          //  DATA Interface
  },
  .acm = {
    .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_ACM,   //0x02 (as defined in cdc.h)
    .bmCapabilities = 2,                      // SET_LINE_CODING supported (BMP uses for both GDB and uart)
  },
  .cdc_union = {
    .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_UNION, //0x06 (as defined in cdc.h)
    .bControlInterface = INTF_CON_COMM,       //  COMM Interface
    .bSubordinateInterface0 = INTF_CON_DATA,  //  DATA Interface
   },
};

//  CON Interface Descriptor (Comm)
static const struct usb_interface_descriptor con_comm_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,           //9 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_INTERFACE,        //4 (as defined in usbstd.h)
  .bInterfaceNumber = INTF_CON_COMM,          //  CDC ACM interface ID is 0
  .bAlternateSetting = 0,                     //No alternate setting
  .bNumEndpoints = 1,                         //1 EP: CDC ACM Endpoint
  .bInterfaceClass = USB_CLASS_CDC,           //0x02 Communications Interface Class (as defined in cdc.h)
  .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM, //0x02 Abstract Control Model (as defined in cdc.h)
  .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,  //0x01 AT Commands: V.250 etc (as defined in cdc.h)
  .iInterface = USB_STRINGS_CON_COMM,         //  Name of CDC ACM interface (index of string descriptor)

  .endpoint = con_comm_endp,

  .extra = &con_cdcacm_functional_descriptors,
  .extralen = sizeof(con_cdcacm_functional_descriptors),
}};

//  CON Interface Descriptor (Data)
static const struct usb_interface_descriptor con_data_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,           //9 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_INTERFACE,        //4 (as defined in usbstd.h)
  .bInterfaceNumber = INTF_CON_DATA,          //  CDC ACM interface ID is 1
  .bAlternateSetting = 0,                     //No alternate setting
  .bNumEndpoints = 2,                         //2 EP's: Data OUT (for received data) and Data IN (for sent data)
  .bInterfaceClass = USB_CLASS_DATA,          //0x0A (as defined in cdc.h)
  .bInterfaceSubClass = 0,
  .bInterfaceProtocol = 0,
  .iInterface = USB_STRINGS_CON_DATA,         //  Name of CDC ACM interface (index of string descriptor)

  .endpoint = con_data_endp,                  //  CDC ACM Endpoint
}};

#if !(CDC_ONLY_ON_USB == true)
//  CON Interface Association Descriptor - CDC Interfaces
static const struct usb_iface_assoc_descriptor con_iface_assoc = {  //  Copied from BMP.  Mandatory for composite device.
  .bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,//8 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,//11 (as defined in usbstd.h)
  .bFirstInterface = INTF_CON_COMM, //  First associated interface (INTF_CON_COMM and INTF_CON_DATA)
  .bInterfaceCount = 2,             //  Total number of associated interfaces (INTF_CON_COMM and INTF_CON_DATA), ID must be consecutive.
  .bFunctionClass = USB_CLASS_CDC,            //0x02 This is a USB CDC (Comms Device Class) interface
  .bFunctionSubClass = USB_CDC_SUBCLASS_ACM,  //0x02 That implements ACM (Abstract Control Model)
  .bFunctionProtocol = USB_CDC_PROTOCOL_AT,   //0x01 Using the AT protocol
  .iFunction = USB_STRINGS_CONSOLE,           //  Name of Console Port
};
#endif  //#if (CDC_ONLY_ON_USB == true)


// UART ACM interface
//  UART Endpoint Descriptors
static const struct usb_endpoint_descriptor uart_comm_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,            //7 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_ENDPOINT,         //5 (as defined in usbstd.h)
  .bEndpointAddress = EP_UART_COMM_IN,            //0x84 UART USB Comm Endpoint
  .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,//0x03 (as defined in usbstd.h)
  .wMaxPacketSize = COMM_PACKET_SIZE,         // Smaller than others: 16
  .bInterval = 255,
} };

static const struct usb_endpoint_descriptor uart_data_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,            //7 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_ENDPOINT,         //5 (as defined in usbstd.h)
  .bEndpointAddress = EP_UART_DATA_OUT,       //3 UART USB OUT Data Endpoint
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,     //0x02 (as defined in usbstd.h)
  .wMaxPacketSize = USBD_DATA_BUFFER_SIZE,    //64
  .bInterval = 1,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,            //7 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_ENDPOINT,         //5 (as defined in usbstd.h)
  .bEndpointAddress = EP_UART_DATA_IN,        //0x83 UART USB IN Data Endpoint
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,     //0x02 (as defined in usbstd.h)
  .wMaxPacketSize = USBD_DATA_BUFFER_SIZE,    //64
  .bInterval = 1,
} };

//  UART Functional Descriptor
static const struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) uart_cdcacm_functional_descriptors = {
  .header = {
    .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,//0x00 (as defined in cdc.h)
    .bcdCDC = 0x0110,
  },
  .call_mgmt = {
    .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,//0x01 (as defined in cdc.h)
    .bmCapabilities = 0,
    .bDataInterface = INTF_UART_DATA,         //  DATA Interface
  },
  .acm = {
    .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_ACM,   //0x02 (as defined in cdc.h)
    .bmCapabilities = 2,                      // SET_LINE_CODING supported (BMP uses for both GDB and uart)
  },
  .cdc_union = {
    .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
    .bDescriptorType = CS_INTERFACE,          //0x24 (as defined in cdc.h)
    .bDescriptorSubtype = USB_CDC_TYPE_UNION, //0x06 (as defined in cdc.h)
    .bControlInterface = INTF_UART_COMM,      //  COMM Interface
    .bSubordinateInterface0 = INTF_UART_DATA, //  DATA Interface
   }
};


//  UART Interface Descriptor (comm)
static const struct usb_interface_descriptor uart_comm_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,           //9 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_INTERFACE,        //4 (as defined in usbstd.h)
  .bInterfaceNumber = INTF_UART_COMM,         //  CDC ACM interface ID is 2
  .bAlternateSetting = 0,
  .bNumEndpoints = 1,
  .bInterfaceClass = USB_CLASS_CDC,           //0x02 (as defined in cdc.h)
  .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM, //0x02 (as defined in cdc.h)
  .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,  //0x01 (as defined in cdc.h)
  .iInterface = USB_STRINGS_UART_COMM,        //  Name of CDC ACM interface (index of string descriptor)

  .endpoint = uart_comm_endp,
  .extra = &uart_cdcacm_functional_descriptors,
  .extralen = sizeof(uart_cdcacm_functional_descriptors),
} };

//  UART Interface Descriptor (data)
static const struct usb_interface_descriptor uart_data_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,           //9 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_INTERFACE,        //4 (as defined in usbstd.h)
  .bInterfaceNumber = INTF_UART_DATA,         //  CDC ACM interface ID is 3
  .bAlternateSetting = 0,
  .bNumEndpoints = 2,
  .bInterfaceClass = USB_CLASS_DATA,          //0x0A (as defined in cdc.h)
  .bInterfaceSubClass = 0,
  .bInterfaceProtocol = 0,
  .iInterface = USB_STRINGS_UART_DATA,        //  Name of CDC ACM interface (index of string descriptor)

  .endpoint = uart_data_endp,                 //  CDC ACM Endpoint
} };

#if !(CDC_ONLY_ON_USB == true)
//  UART Interface Association Descriptor - CDC Interfaces
static const struct usb_iface_assoc_descriptor uart_iface_assoc = //  Copied from BMP.  Mandatory for composite device.
{
  .bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,//8 (as defined in usbstd.h)
  .bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,//11 (as defined in usbstd.h)
  .bFirstInterface = INTF_UART_COMM,//  First associated interface (INTF_UART_COMM and INTF_UART_DATA)
  .bInterfaceCount = 2,             //  Total number of associated interfaces (INTF_UART_COMM and INTF_UART_DATA), ID must be consecutive.
  .bFunctionClass = USB_CLASS_CDC,            //0x02 This is a USB CDC (Comms Device Class) interface
  .bFunctionSubClass = USB_CDC_SUBCLASS_ACM,  //0x02 That implements ACM (Abstract Control Model)
  .bFunctionProtocol = USB_CDC_PROTOCOL_AT,   //0x01 Using the AT protocol
  .iFunction = USB_STRINGS_UART,              //  Name of Serial Port
};
#endif  //#if (CDC_ONLY_ON_USB == true)


// Both Console and UART ACM interfaces
//  USB Configuration Descriptor (Both interfaces)
static const struct usb_interface interfaces[] = {
{
  .num_altsetting = 1,
#if !(CDC_ONLY_ON_USB == true)
  .iface_assoc = &con_iface_assoc,            //Mandatory for composite device with multiple interfaces. static const struct usb_iface_assoc_descriptor con_iface_assoc
#endif  //#if !(CDC_ONLY_ON_USB == true)
  .altsetting = con_comm_iface,               //Index must sync with INTF_CON_COMM.
}, {
  .num_altsetting = 1,
  .altsetting = con_data_iface,               //Index must sync with INTF_CON_DATA.
},
{
  .num_altsetting = 1,
#if !(CDC_ONLY_ON_USB == true)
  .iface_assoc = &uart_iface_assoc,           //Mandatory for composite device with multiple interfaces. static const struct usb_iface_assoc_descriptor uart_iface_assoc
#endif  //#if !(CDC_ONLY_ON_USB == true)
  .altsetting = uart_comm_iface,              //Index must sync with INTF_UART_COMM.
}, {
  .num_altsetting = 1,
  .altsetting = uart_data_iface,              //Index must sync with INTF_UART_DATA.
},};


static const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,                          //Libopencm3 will fill this automatically at sending time
  .bNumInterfaces = sizeof(interfaces)    / \
                    sizeof(interfaces[0]),    //  We will have 4 interfaces
  .bConfigurationValue = 1,                   //  This is the configuration ID 1
  .iConfiguration = 0,                        //  Configuration string (0 means none)
  .bmAttributes = USB_CONFIG_ATTR_DEFAULT | \
                  USB_CONFIG_ATTR_SELF_POWERED,//  Self-powered: it doesn't draw power from USB bus.
  .bMaxPower = 5,                             //  Specifies how much bus current a device requires: 10 mA. (2 x .bMaxPower)

  .interface = interfaces,                    //  List of all interfaces
};

#endif  //#if USE_USB == true 


#endif  //#ifndef USB_DESCRIPTORS_H
