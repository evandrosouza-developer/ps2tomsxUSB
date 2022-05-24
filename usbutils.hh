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
 * This file contains the declarations of some auxiliary functions and data types for accessing the USB peripherals.
 */

#ifndef USBUTILS_HH_
#define USBUTILS_HH_

#include <cstddef>
#include <cstdint>

#include "util.hh"

/// The controller only has 8 EPnR registers
static constexpr uint8_t numEP = 8;

/// Must be used for variables located in the USB buffer memory (buffer descriptor table and endpoint buffer).
#define USB_MEM __attribute__((section(".usbbuf")))

/// The instances of this are stored in the USB RAM and indicate the start and size of the receive/transmit buffers.
struct EP_BufDesc {
	// start of send buffer
	uint16_t txBufferAddr;
	uint16_t Padding1;
	// Size of the send buffer in bytes
	uint16_t txBufferCount;
	uint16_t Padding2;
	// Start of receive buffer
	uint16_t rxBufferAddr;
	uint16_t Padding3;
	// Size and fill level of the receive buffer; See Reference Manual for special format
	uint16_t rxBufferCount;
	uint16_t Padding4;
};
// The individual instances of EP_BufDesc must be 16 bytes in size so that they are in the array at addresses that are multiples of 16.
static_assert(sizeof(EP_BufDesc) == 16, "");

/**
 * Since the USB-RAM is organized in the form of 16-bit words, between which there is a 16-bit gap,
 * UsbMem defines a 16-bit word with a gap, i.e. an array of these fits exactly on the memory.
 */
class UsbMem {
	public:
		uint16_t data;
	private:
		char padding [2];
};
// Each instance of UsbMem must be 4 bytes in size so they are in the array at addresses that are multiples of 4.
static_assert (sizeof(UsbMem) == 4, "");

/**
 * Contains an array named "data" from UsbMem, assuming size N/2. Can be used
 * to define buffer memory in the USB RAM for endpoints by specifying bytes instead of 16bit words as the size.
 * It is automatically checked whether the requested size is supported by the hardware.
 */
template <size_t N>
struct UsbAlloc {
	static_assert (((N <= 62) && (N%2 == 0)) || ((N <= 512) && (N % 32 == 0)), "Invalid reception buffer size requested");
	static constexpr size_t size = N;

	/// The actual data array
	UsbMem data [N/2];
	/// Provides access to 16-bit word "i".
	usb_always_inline uint16_t& operator [] (size_t i) {
		return data [i].data;
	}
};


/**
 * Since the register definitions EP0R, EP1R, ... EP8R from stm32f1xx.h cannot be accessed via an index known at runtime,
 * an array of 16-bit words is defined here, the entries of which correspond to the EPnR registers. The UsbMem type takes care of that
 * the required 4-byte increments of the address. Thus, e.g. EPnR[2] corresponds to register USB_EP2R
 */
static __IO UsbMem (&EPnR) [numEP] = *reinterpret_cast<__IO UsbMem (*)[numEP]> (USB_BASE);
extern EP_BufDesc BufDescTable [numEP];

/**
 * From the processor's point of view, the data in the USB buffer memory is located at address 0x40006000 and following, each with a 16-bit gap
 * between two 16-bit words. From the peripheral perspective, memory starts at 0, with no gaps. Since in the buffer descriptor table as well as in
 * USB_BTABLE the address must be specified from the perspective of the peripherals, a conversion from "processor" address to "peripheral" address is required here.
 * Address made: If the "addr" parameter is a normal pointer to a variable defined by USB_MEM, it will be a 16bit number
 * returned which can be used as address in USB_BTABLE or EP_BufDesc.
 */
template <typename T>
usb_always_inline uint16_t mapAddr (T* addr) {
	// Ziehe Adresse von Anfangsadresse ab und teile durch 2 um Lücken herauszurechnen.
	return static_cast<uint16_t> ((reinterpret_cast<uintptr_t> (addr) - 0x40006000) / 2);
}
/**
 * Clears the interrupts specified in "mask" (as an OR of the USB_ISTR_xxx constants) in the USB_ISTR register.
 */
usb_always_inline void clearInterrupt (uint16_t mask) {
	USB->ISTR = (USB_ISTR_PMAOVR | USB_ISTR_ERR | USB_ISTR_WKUP | USB_ISTR_SUSP | USB_ISTR_RESET | USB_ISTR_SOF | USB_ISTR_ESOF) & ~mask;
}

void setEPnR (uint8_t EP, uint16_t mask, uint16_t data, uint16_t old);
void setEPnR (uint8_t EP, uint16_t mask, uint16_t data);

#endif /* USBUTILS_HH_ */
