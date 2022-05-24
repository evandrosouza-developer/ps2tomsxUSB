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

//#include <stm32f1xx.h>
#include "util.hh"
#include "clockconf.h"
#include "usb.hh"
#include "main.hh"

extern USBPhys usbPhys;

/* Allocation of buffers for sending/receiving on endpoints in the USB buffer memory, 512 bytes in total
  * The variables are stored in the memory of the USB controller via "USB_MEM" via linker script.
  * The USB peripherals read/write the transferred data directly to these variables.
  */

/**
  * Buffer for endpoint 0. Control endpoints may only transmit 8,16,32 or 64 bytes per packet.
  * Used for sending & receiving ("half-duplex")
  */

alignas(4) static UsbAlloc<64> EP0_BUF	USB_MEM;

alignas(4) static UsbAlloc<dataEpMaxPacketSize> EP1_BUF	USB_MEM;

/// The default control endpoint 0 is mandatory for all USB devices.
static DefaultControlEP controlEP (usbPhys, 0, EP0_BUF.data, EP0_BUF.size);

/// Create an endpoint to flip the data
MirrorEP mirrorEP (EP1_BUF.data, EP1_BUF.size);

/// Central instance for USB access. Pass 2 endpoints.
USBPhys usbPhys (std::array<EPBuffer*, 8> {{ &controlEP, &mirrorEP, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr }});

void initializePeriphalClocks () {
	// Enable GPIO modules for the pins used
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
}

void MirrorEP::onReset () {
	EPBuffer::onReset ();
	// Prepare to receive data
	receivePacket (std::min<size_t> (getRxBufLength(), sizeof (m_buffer)));
}

void MirrorEP::onReceive (bool, size_t rxBytes) {
	// query received data
	size_t count = std::min<size_t> (sizeof (m_buffer), rxBytes);
	getReceivedData (m_buffer, count);

	// Flip each byte
	for (size_t i = 0; i < count; ++i) {
		m_buffer [i] = static_cast<uint8_t> (__RBIT(m_buffer [i]) >> 24);
	}

	// Send back result
	transmitPacket (m_buffer, count);
}

void MirrorEP::onTransmit () {
	// After successful sending, ready to receive again
	receivePacket (std::min<size_t> (getRxBufLength(), sizeof (m_buffer)));
}

/**
* Global interrupt for USB peripherals. Since CAN and USB cannot be used at the same time anyway,
* this ISR also applies to the CAN peripherals.
*/
extern "C" void USB_LP_CAN1_RX0_IRQHandler () {
	usbPhys.irq ();
}

int main () {
	// Don't handle interrupts yet
	__disable_irq ();
	// Turn on system clock to 72MHz
	configureSysClock ();

	// enable peripheral clocks
	initializePeriphalClocks ();

	LED1.configureOutput ();
	LED2.configureOutput ();

	// Enable USB peripherals
	usbPhys.init ();

	// Establish connection to host
	usbPhys.connect ();

	// Enable interrupt handling
	__enable_irq ();

	// Wait for interrupts in an infinite loop
	while (1) {
		// The WFI instruction lets the processor core sleep until an interrupt occurs to conserve power.
		// Some JTAG programming adapters (ST-Link) may not be able to establish a connection as long as the controller
		// "hangs" here. The "Connect under Reset" option can help, or simply comment out this line
		// at the expense of higher power consumption.
		__WFI ();
	}
}
