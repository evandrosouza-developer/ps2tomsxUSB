/*
 * Copyright (c) 2017, Niklas Gürtler
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * In this file the concrete descriptors for the device are defined using the helper functions
 * in usb_desc_helper.hh. The descriptors are defined as single global constant variables and land
 * thus as constants in Flash. In the "descriptors" table, pointers to the descriptors together with length,
 * Type and index listed so they can be found at runtime via the getUsbDescriptor function.
 */

#include "usb_desc.hh"
#include "main.hh"

// The device class specified here in the device descriptor marks the device as a composite device.
static constexpr auto deviceDescriptor = EncodeDescriptors::USB20::device (
			0x200,		// bcdUSB
			0xFF,		// bDeviceClass
			0xFF,		// bDeviceSubClass
			0xFF,		// bDeviceProtocol
			64,			// bMaxPacketSize0
			0xDEAD,		// idVendor		TODO - anpassen
			0xBEEF,		// idProduct	TODO - anpassen
			0x0100,		// bcdDevice
			1,			// iManufacturer, corresponds to the index of the strManufacturer descriptor
			2,			// iProduct, corresponds to the index of the strProduct descriptor
			3,			// iSerialNumber, corresponds to the index of the strSerial descriptor
			1			// bNumConfigurations
		);
// static constexpr std::array<unsigned char, 18> deviceDescriptor {{ 0x12, 0x01, 0x00, 0x02, 0xff, 0xff, 0xff, 0x40, 0xad, 0xde, 0xef, 0xbe, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01 }};

/*
 * For endpoints, wMaxPacketSize must not exceed the size of the respective buffer defined in usb.cc.
 * bEndpointAddress is the address of the endpoint on the bus; an endpoint can be any EPnR
 * Registers are assigned.
 */


static constexpr auto confDescriptor = EncodeDescriptors::USB20::configuration (
			1,			// bNumInterfaces
			1,			// bConfigurationValue
			0,			// iConfiguration
			0x80,		// bmAttributes
			250,		// bMaxPower (500mA)

			EncodeDescriptors::USB20::interface (
				0,		// bInterfaceNumber
				0,		// bAlternateSetting
				2,		// bNumEndpoints
				0xFF,	// bInterfaceClass
				0xFF,	// bInterfaceSubClass
				0xFF,	// bInterfaceProtocol
				0		// iInterface
			),
			/*
			 * wMaxPacketSize must not exceed the size of the respective buffer defined in usb.cc.
			 * bEndpointAddress is the address of the endpoint on the bus; an endpoint can be any EPnR
			 * Registers are assigned. @TODO - configure desired endpoints here.
			 */
			EncodeDescriptors::USB20::endpoint (
				1,		// bEndpointAddress
				2,		// bmAttributes
				dataEpMaxPacketSize,		// wMaxPacketSize
				10		// bInterval
			),
			EncodeDescriptors::USB20::endpoint (
				0x81,	// bEndpointAddress
				2,		// bmAttributes
				dataEpMaxPacketSize,		// wMaxPacketSize
				10		// bInterval
		)
);
// static constexpr std::array<unsigned char, 32> confDescriptor {{ 0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0xfa, 0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0xff, 0xff, 0x00, 0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x0a, 0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x0a }};

/**
 * This Microsoft Compat Id Descriptor identifies the device as "WinUsb Device", s.d. automatically the
 * WinUsb driver is loaded, which allows applications direct access to the device, e.g. via libusb.
 * This has no impact on other operating systems.
 */
static constexpr auto compatIdDescriptor = EncodeDescriptors::MS_OS_Desc::compatId (
			0x100,					// bcdVersion
			4,						// wIndex
			EncodeDescriptors::MS_OS_Desc::compatIdFunction (
				0,									// bFirstInterfaceNumber
				Util::encodeString ("WINUSB\0\0"),	// compatibleID
				std::array<Util::EncChar, 8> {}				// subCompatibleID
			)
);
// static constexpr std::array<unsigned char, 40> compatIdDescriptor {{ 0x28, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x57, 0x49, 0x4e, 0x55, 0x53, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }};

// These strings identify the device to the user. They must be encoded as UTF-16 (small u)

static constexpr auto strManufacturer = EncodeDescriptors::USB20::string (u"ACME Corp.");
//static  constexpr std::array<unsigned char, 22> strManufacturer {{ 0x16, 0x03, 0x41, 0x00, 0x43, 0x00, 0x4d, 0x00, 0x45, 0x00, 0x20, 0x00, 0x43, 0x00, 0x6f, 0x00, 0x72, 0x00, 0x70, 0x00, 0x2e, 0x00 }};

static constexpr auto strProduct = EncodeDescriptors::USB20::string (u"Fluxkompensator");
//static constexpr std::array<unsigned char, 32> strProduct {{ 0x20, 0x03, 0x46, 0x00, 0x6c, 0x00, 0x75, 0x00, 0x78, 0x00, 0x6b, 0x00, 0x6f, 0x00, 0x6d, 0x00, 0x70, 0x00, 0x65, 0x00, 0x6e, 0x00, 0x73, 0x00, 0x61, 0x00, 0x74, 0x00, 0x6f, 0x00, 0x72, 0x00 }};

static constexpr auto strSerial = EncodeDescriptors::USB20::string (u"42-1337-47/11");
//static constexpr std::array<unsigned char, 28> strSerial {{ 0x1c, 0x03, 0x34, 0x00, 0x32, 0x00, 0x2d, 0x00, 0x31, 0x00, 0x33, 0x00, 0x33, 0x00, 0x37, 0x00, 0x2d, 0x00, 0x34, 0x00, 0x37, 0x00, 0x2f, 0x00, 0x31, 0x00, 0x31, 0x00 }};

static constexpr auto strFunction = EncodeDescriptors::USB20::string (u"STM32 Serial Port");
//static constexpr std::array<unsigned char, 36> strFunction {{ 0x24, 0x03, 0x53, 0x00, 0x54, 0x00, 0x4d, 0x00, 0x33, 0x00, 0x32, 0x00, 0x20, 0x00, 0x53, 0x00, 0x65, 0x00, 0x72, 0x00, 0x69, 0x00, 0x61, 0x00, 0x6c, 0x00, 0x20, 0x00, 0x50, 0x00, 0x6f, 0x00, 0x72, 0x00, 0x74, 0x00 }};
/**
 * The Microsoft OS String Descriptor is required for the Compat Id Descriptor to be queried at all.
 * It must have the index 0xEE. The byte with the value 3 is arbitrary and is used by the PC for bRequest to
 * Query the compatIdDescriptor. Since 1 and 2 are already used for LED control, 3 is used here.
 */
static constexpr auto strOsStringDesc = EncodeDescriptors::USB20::string(u"MSFT100\u0003");
//static constexpr std::array<unsigned char, 18> strOsStringDesc {{ 0x12, 0x03, 0x4d, 0x00, 0x53, 0x00, 0x46, 0x00, 0x54, 0x00, 0x31, 0x00, 0x30, 0x00, 0x30, 0x00, 0x03, 0x00 }};

/// Since the strings are German, only German is specified as the language here.
static constexpr auto langTable = EncodeDescriptors::USB20::languageTable (0x0407 /* German (Standard) */);
//static constexpr std::array<unsigned char, 4> langTable {{ 0x04, 0x03, 0x07, 0x04 }};
/**
 * All descriptors given above must be entered in this table in order to be used at runtime
 * can be found. The length of the descriptor is inferred from "Descriptor" in the constructor.
 */
static constexpr Descriptor descriptors [] = { { deviceDescriptor, D_TYPE::DEVICE, 0 },
									{ confDescriptor, D_TYPE::CONFIGURATION, 0 },
									{ langTable, D_TYPE::STRING, 0 },
									{ strManufacturer, D_TYPE::STRING, 1 },
									{ strProduct, D_TYPE::STRING, 2 },
									{ strSerial, D_TYPE::STRING, 3 },
	//								{ strFunction, D_TYPE::STRING, 4 },
									{ strOsStringDesc, D_TYPE::STRING, 0xEE },
									{ compatIdDescriptor, D_TYPE::OS_DESCRIPTOR, 0 }
};

/**
 * This function searches the table for a descriptor of the specified type and index. none will
 * found, nullptr is returned, otherwise a pointer to the corresponding "Descriptor" instance.
 */
const Descriptor* getUsbDescriptor (D_TYPE type, uint8_t index) {
	// Search descriptor table
	for (auto& d : descriptors) {
		if (d.type == type && d.index == index)
			return &d;
	}
	return nullptr;
}
