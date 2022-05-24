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
 * This file defines utility functions that allow easy creation of USB descriptors. This one
 * Defined functions can be called with the values you want, and display the result
 * Assign global "static constexpr" variables so it ends up in flash as read-only data. The functions
 * automatically add length and type, so you don't need to specify them when calling. In the descriptors for
 * Configuration and Microsoft Compatibility ID immediately followed in memory by a variable number of others
 * Descriptors - these are variable as arguments to the respective functions "configuration" or "compatId".
 * Pass number after the simple number arguments after they have been previously generated via the respective function.
 * The total length is automatically calculated in each case.
 *
 * For the precise meaning of the individual values, see the USB specification (usb_20.pdf, p. 260ff.).
 */

#ifndef USB_DESC_HELPER_HH_
#define USB_DESC_HELPER_HH_

#include "encode.hh"

/**
 * This enum defines each type of descriptor. The numerical value is sent via USB request to certain query descriptors.
 * The OS_DESCRIPTOR type is not specified by the specification, nor is it sent via USB transmitted,
 * but only used here to identify and retrieve the Microsoft OS string descriptor.
 */
enum class D_TYPE : uint8_t {	DEVICE = 1, CONFIGURATION = 2, STRING = 3, INTERFACE = 4, ENDPOINT = 5, DEVICE_QUALIFIER = 6,
								OTHER_SPEED_CONFIGURATION = 7, INTERFACE_POWER = 8, OS_DESCRIPTOR = 100 };

namespace EncodeDescriptors {

	namespace USB20 {

	/// The device descriptor describes the general properties of the device.
		usb_always_inline constexpr std::array<Util::EncChar, 18> device (uint16_t bcdUSB, uint8_t bDeviceClass, uint8_t bDeviceSubClass, uint8_t bDeviceProtocol, uint8_t bMaxPacketSize0,
				uint16_t idVendor, uint16_t idProduct, uint16_t bcdDevice, uint8_t iManufacturer, uint8_t iProduct,	uint8_t iSerialNumber, uint8_t bNumConfigurations) {
			return Util::encodeMulti (
					uint8_t { 18 },
					D_TYPE::DEVICE,
					bcdUSB,
					bDeviceClass,
					bDeviceSubClass,
					bDeviceProtocol,
					bMaxPacketSize0,
					idVendor,
					idProduct,
					bcdDevice,
					iManufacturer,
					iProduct,
					iSerialNumber,
					bNumConfigurations);

		}

		/**
		 * String descriptors are partially displayed on host devices and are also used for the Microsoft OS string.
		 * The parameter should be a UTF-16 string literal, which is achieved by specifying a small(!) "u" in front of the literal
		 * becomes, e.g. u"MyDevice". Any special characters are therefore allowed. The terminating 0 character is truncated.
		*/
		template <typename T, size_t N>
		usb_always_inline constexpr std::array<Util::EncChar, 2 + sizeof(T)*(N-1)> string (const T (&str) [N]) {
			return Util::encodeMulti (
					static_cast<uint8_t> (2 + sizeof(T)*(N-1)),
					D_TYPE::STRING,
					Util::encodeString (str)
			);
		}

		/**
		 * The language table gives a list of the languages supported by the device. For strings, the host can choose the desired
		 * Request language. The parameters should be a sequence of 16bit numbers corresponding to the language IDs
		 * (see http://www.usb.org/developers/docs/USB_LANGIDs.pdf ).
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, 2 + 2*sizeof...(T)> languageTable (T... langs) {
			return Util::encodeMulti (
					static_cast<uint8_t> (2 + 2*sizeof...(T)),
					D_TYPE::STRING,
					static_cast<uint16_t> (langs)...
			);
		}

		/**
		 * Information about an operating mode of the device is collected in the configuration descriptor.
		 * Especially interesting here is the specification of the maximum power consumption (in 2mA steps).
		 * After the integer parameters, a sequence of interface and endpoint descriptors of any length and
		 * those that are class/device-specific follow.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, Util::staticSum<size_t> (9, Util::arrSize<T> ()...)> configuration (uint8_t bNumInterfaces, uint8_t bConfigurationValue, uint8_t iConfiguration, uint8_t bmAttributes, uint8_t bMaxPower, T&&... sub) {
			return Util::concatArrays<Util::EncChar> (
					Util::encodeMulti (
						uint8_t { 9 },
						D_TYPE::CONFIGURATION,
						static_cast<uint16_t> (Util::staticSum<size_t> (9, Util::arrSize<T> ()...)),
						bNumInterfaces,
						bConfigurationValue,
						iConfiguration,
						bmAttributes,
						bMaxPower
					),
					std::forward<T> (sub)...
				);
		}

		/// Information about a supported interface
		usb_always_inline constexpr std::array<Util::EncChar, 9> interface (uint8_t bInterfaceNumber, uint8_t bAlternateSetting, uint8_t bNumEndpoints, uint8_t bInterfaceClass, uint8_t bInterfaceSubClass,
				uint8_t bInterfaceProtocol, uint8_t iInterface) {
			return Util::encodeMulti (
					uint8_t { 9 },
					D_TYPE::INTERFACE,
					bInterfaceNumber,
					bAlternateSetting,
					bNumEndpoints,
					bInterfaceClass,
					bInterfaceSubClass,
					bInterfaceProtocol,
					iInterface
			);
		}

		/**
		 * Information about a supported endpoint. All endpoints used except endpoint 0 must be defined here,
		 * whereby the maximum packet size and the endpoint type are particularly important.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 7> endpoint (uint8_t bEndpointAddress, uint8_t bmAttributes, uint16_t wMaxPacketSize, uint8_t bInterval) {
			return Util::encodeMulti (
				uint8_t { 7 },
				D_TYPE::ENDPOINT,
				bEndpointAddress,
				bmAttributes,
				wMaxPacketSize,
				bInterval);
		}
	}

	namespace IAD {
		/**
		 * Returns an interface association descriptor that tells the OS which interfaces belong together in a function. This is for
		 * Composite Devices required under Windows.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 8> interfaceAssociation (uint8_t bFirstInterface, uint8_t bInterfaceCount, uint8_t bFunctionClass, uint8_t bFunctionSubClass, uint8_t bFunctionProtocol, uint8_t iFunction) {
			return Util::encodeMulti (
				uint8_t { 0x08 },			// bLength
				uint8_t { 0x0B },			// bDescriptorType
				bFirstInterface,
				bInterfaceCount,
				bFunctionClass,
				bFunctionSubClass,
				bFunctionProtocol,
				iFunction
			);
		}
	}

	namespace MS_OS_Desc {
		/**
		 * The Microsoft Compat ID Descriptor defines a set of functions. After the integer parameters, any
		 * long sequence of Compat ID Function descriptors follow, i.e. from return values of compatIdFunction.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, Util::staticSum<size_t> (16, Util::arrSize<T> ()...)> compatId (uint16_t bcdVersion, uint16_t wIndex, T&&... functions) {
			return Util::concatArrays<Util::EncChar> (
					Util::encodeMulti (
						static_cast<uint32_t> (Util::staticSum<size_t> (16, Util::arrSize<T> ()...)),
						bcdVersion,
						wIndex,
						static_cast<uint8_t> (sizeof...(functions))
					),
					std::array<Util::EncChar, 7> {{}},
					std::forward<T> (functions)...
				);
		}

		/**
		 * Defines a Microsoft Compat ID function, e.g. to identify a device as a WinUSB device. The parameters compatibleID
		 * and subcompatibleID are simple ASCII strings that may need to be padded with zeros. You can use Util::encodeString
		 * are generated, e.g. Util::encodeString ("WINUSB\0\0").
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 24> compatIdFunction (uint8_t bFirstInterfaceNumber, std::array<Util::EncChar, 8> compatibleID, std::array<Util::EncChar, 8> subCompatibleID) {
			return Util::concatArrays<Util::EncChar> (
					Util::encode (bFirstInterfaceNumber),
					std::array<Util::EncChar, 1> {{1}},
					compatibleID,
					subCompatibleID,
					std::array<Util::EncChar, 6> {{}}
					);
		}
	}

	namespace CDC {
		/**
		 * Assembles CDC specific descriptors into one large descriptor attached to a configuration descriptor
		 * can be passed. An arbitrarily long sequence of CDC descriptors can follow the "bcdCDC" parameter.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, Util::staticSum<size_t> (5, Util::arrSize<T> ()...)> classSpecific (uint16_t bcdCDC, T&&... sub) {
			return Util::concatArrays<Util::EncChar> (
					Util::encodeMulti (
						uint8_t { 5 },
						uint8_t { 0x24 }, // CS_INTERFACE
						uint8_t { 0 }, // Header
						bcdCDC
					),
					std::forward<T> (sub)...
				);
		}
		/**
		 * Defines a "Union Interface" descriptor for CDC. bControlInterface specifies the master interface on which are allowed
		 * any number of indices of sub-interfaces follow.
		 */
		template <typename... T>
		usb_always_inline constexpr std::array<Util::EncChar, 4+sizeof...(T)> unionInterface (uint8_t bControlInterface, T&&... bSubordinateInterfaces) {
			return Util::encodeMulti (
					static_cast<uint8_t> (4+sizeof...(T)),
					uint8_t { 0x24 }, // CS_INTERFACE
					uint8_t { 6 }, // Union Functional Descriptor
					bControlInterface,
					static_cast<uint8_t> (bSubordinateInterfaces)...
					);
		}
		/**
		 * Defines a descriptor for CDC Call Management.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 5> callManagement (uint8_t bmCapabilities, uint8_t bDataInterface) {
			return Util::encodeMulti (
					uint8_t { 5 },
					uint8_t { 0x24 }, // CS_INTERFACE
					uint8_t { 1 }, // Call Management Subtype
					bmCapabilities,
					bDataInterface
					);
		}
		/**
		 * Defines a descriptor for the CDC Abstract Control Model.
		 */
		usb_always_inline constexpr std::array<Util::EncChar, 4> ACM (uint8_t bmCapabilities) {
			return Util::encodeMulti (
					uint8_t { 4 },
					uint8_t { 0x24 }, // CS_INTERFACE
					uint8_t { 2 }, // ACM Subtype
					bmCapabilities);
		}

	}
}

#endif /* USB_DESC_HELPER_HH_ */
