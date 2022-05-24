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
 * This file contains the main part of the USB peripheral control and the handling of standard requests
 * and data transfers.
 */

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <stm32f1xx.h>

#include "usb.hh"
#include "usb_desc.hh"
#include "main.hh"

/// Global interrupt for USB peripherals. Should be called directly by the ISR.
void USBPhys::irq () {
	uint16_t ISTR;

	// Only these interrupts will be processed
	const uint16_t interrupts = USB_ISTR_RESET | USB_ISTR_CTR;

	// Process events that have occurred in a loop until the USB peripherals do not signal any more.
	while (((ISTR = USB->ISTR) & interrupts) != 0) {
		// A "RESET" occurs when there are missing packets from the host. This is the case at the beginning of every connection,
		// until the host recognizes the device.
		if (ISTR & USB_ISTR_RESET) {
			// Clear interrupt
			clearInterrupt (USB_ISTR_RESET);
			// Bring hardware and software to a defined initial state
			initializeDevice ();
		}
		// CTR signals the completion of a correct transfer
		if (ISTR & USB_ISTR_CTR) {
			// Direction of last transfer. false means "IN" transfer (Device->Host), true means "IN" or "OUT"
			bool dir = ISTR & USB_ISTR_DIR; // 0=TX, 1=RX/TX
			// The number of the EPnR register associated with this transfer."
			uint8_t EP = (ISTR & USB_ISTR_EP_ID) >> USB_ISTR_EP_ID_Pos;
			// Make sure the EPnR register exists
			myassert (EP < numEP);
			// Query the state of this endpoint
			uint16_t s = EPnR [EP].data;

			// Clear the RX/TX flags in the EPnR register if they are set. If the hardware between polling and clearing
			// if one of the bits is set, this is not cleared and is dealt with in the next iteration of the loop.
			setEPnR (EP, s & (USB_EP0R_CTR_RX_Msk | USB_EP0R_CTR_TX_Msk), 0, s);

			// Check if it was a receiving transfer (OUT/SETUP).
			if (dir && (s & USB_EP0R_CTR_RX_Msk) != 0) {
				// Check for SETUP transfer (OUT and SETUP behave identically except for this bit)
				bool setup = s & USB_EP0R_SETUP_Msk;

				// Query the number of bytes received in the buffer.
				uint16_t rxBytes = BufDescTable[EP].rxBufferCount & USB_COUNT0_RX_0_COUNT0_RX_0;

				if (m_epBuffers [EP])
					// Forward to EPBuffer instance.
					m_epBuffers [EP]->onReceive (setup, rxBytes);
				else
					// If the endpoint is not in use, resolve errors in the debugger; in normal operation nothing should happen
					// instead of a crash avoided by the explicit query.
					myassert (false);
			}
			// Check if it was a sending transfer (IN).
			if ((s & USB_EP0R_CTR_TX_Msk) != 0) {
				if (m_epBuffers [EP])
					// Forward to EPBuffer instance.
					m_epBuffers [EP]->onTransmit ();
				else
					// If the endpoint is not in use, resolve errors in the debugger; in normal operation nothing should happen
					// instead of a crash avoided by the explicit query.
					myassert (false);
			}
		}
	}
}

/**
 * This function waits at least 1µs (tSTARTUP), implemented with a simple "NOP" loop. This will ever
 * actually take longer after the compiler, but that is usually not a problem since it only occurs once when
 * boot up is called. If necessary, the function can be implemented using inline assembler or using the
 * ARM cycle counters are rebuilt so that they do not need longer than 1µs.
 */
static void delay () {
	// Count at least 72 clocks (1µs at 72MHz)
	for (int i = 0; i < 72; ++i) __NOP ();
}

/// This function activates the peripherals required for USB transmission, but does not yet establish a connection.
void USBPhys::init () {
	// The USB pins are in GPIOA, but the GPIOA peripheral clock does not need to be activated, USB works directly
	// Enable USB clock
	RCC->APB1ENR |= RCC_APB1ENR_USBEN_Msk;

	// Configure pin for 1.5kOhm resistor, and switch pin to high, i.e. resistance is off
	// TODO: If necessary, select another port/pin
#if MCU == STM32F103C8
	GPIOC->CRH = 0x44474444;
#endif
	GPIOC->BSRR = GPIO_BSRR_BS12;

	// Turn on USB interrupt
	NVIC_EnableIRQ (USB_LP_CAN1_RX0_IRQn);

	// Leave USB peripherals in power-saving mode
	USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
}

/// This function starts the connection to the host. init() must be called before hand.
void USBPhys::connect () {
	// Turn on transceivers, turn off logic
	USB->CNTR = USB_CNTR_FRES;

	// Wait for analog circuit to start up (tSTARTUP)
	delay ();

	// Turn on USB, enable interrupts
	USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM;

	// Clear all interrupts except reset interrupt
	USB->ISTR = USB_ISTR_RESET_Msk;
	NVIC_ClearPendingIRQ (USB_LP_CAN1_RX0_IRQn);

	// TODO: If necessary, select a different port/pin
	// turn on 1.5kOhm resistor, s.d. Host recognizes the device.
	GPIOC->BSRR = GPIO_BSRR_BR12;
}

/// Puts the USB peripherals back into power-saving mode (like after init()), disconnects from the host.
void USBPhys::disconnect () {
	// Put peripherals in power-saving mode
	USB->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;

	// Switch pin to high, i.e. resistance is off. Host detects disconnect.
	// TODO: If necessary, select another port/pin
	GPIOC->BSRR = GPIO_BSRR_BS12;

	// Clear all interrupts
	USB->ISTR = 0;
	NVIC_ClearPendingIRQ (USB_LP_CAN1_RX0_IRQn);
}

/**
 * This function is called when the host resets the device ("RESET"), possibly also when booting.
 * It should set the hardware and software to a defined initial state so that the driver on the host side can
 * Clean condition may emanate.
 */
void USBPhys::initializeDevice () {
	myassert ((mapAddr (BufDescTable) & 7) == 0);
	// Make the buffer descriptor table known to the peripherals
	USB->BTABLE = mapAddr (BufDescTable);
	// Newly connected devices have address 0. Store this in the USB register.
	USB->DADDR = USB_DADDR_EF | (0 << USB_DADDR_ADD_Pos);

	// Initialize all endpoints.
	for (EPBuffer* ep : m_epBuffers)
		if (ep)
			ep->onReset ();
}

/**
 * Uses DTOG_RX/TX in the EPnR register to reset all bulk and interrupt endpoints to "DATA0".
 * This is called when a SET_CONFIGURATION command has been received. Control endpoints always start
 * with DATA0.
 */
void USBPhys::resetDataToggle () {
	for (uint_fast8_t iEP = 0; iEP < numEP; ++iEP) {
		uint16_t s = EPnR [iEP].data;

		// Check type of endpoint (bulk/interrupt)
		if ((s & USB_EP_T_FIELD_Msk) == USB_EP_BULK || (s & USB_EP_T_FIELD_Msk) == USB_EP_INTERRUPT) {
			// Check if send/receive is enabled. This information could also be obtained via m_epBuffers
			// win, but it's easier this way.
			bool rx = (s & USB_EPRX_STAT) != USB_EP_RX_DIS;
			bool tx = (s & USB_EPTX_STAT) != USB_EP_TX_DIS;
			if (rx && tx)
				// Reset both directions
				setEPnR (static_cast<uint8_t> (iEP), USB_EP_DTOG_RX_Msk | USB_EP_DTOG_TX_Msk, 0, s);
			else if (rx)
				// Reset received only
				setEPnR (static_cast<uint8_t> (iEP), USB_EP_DTOG_RX_Msk, 0, s);
			else if (tx)
				// Reset send only
				setEPnR (static_cast<uint8_t> (iEP), USB_EP_DTOG_TX_Msk, 0, s);
		}
	}
}

/**
 * Should be called upon receipt of SET_ADDERSS. Makes the new address known to the periphery.
 */
void USBPhys::setAddress (uint8_t addr) {
	USB->DADDR = static_cast<uint16_t> (USB_DADDR_EF | addr);
}

/**
 * Prepares to send a single data packet. Copies the data to the USB buffer and configures
 * the peripherals for sending. After the transfer is complete, onTransmit is called. "length" must be <=
 * be the buffer size specified in the constructor.
 */
void EPBuffer::transmitPacket (const uint8_t* data, size_t length) {
	if (length) {
		myassert (length <= m_txBufLength && length <= 0xFFFF);
		// Assemble two bytes to a uint16_t and write this to the USB buffer memory.
		for (uint_fast16_t i = 0; i < length / 2; ++i) {
			uint_fast16_t a = static_cast<uint8_t> (data [i*2]);
			uint_fast16_t b = static_cast<uint8_t> (data [i*2+1]);
			m_txBuffer [i].data = static_cast<uint16_t> (a | (b << 8));
		}
		// If there is one byte left, write it one by one.
		if (length % 2) {
			m_txBuffer [length/2].data = static_cast<uint8_t> (data [length-1]);
		}
		// Give buffer size start to peripheral
		BufDescTable [m_iBuffer].txBufferAddr = mapAddr (m_txBuffer);
	} else {
		// Empty package
		BufDescTable [m_iBuffer].txBufferAddr = 0;
	}
	BufDescTable [m_iBuffer].txBufferCount = static_cast<uint16_t> (length);
	// Allow data to be sent.
	setEPnR (m_iBuffer, USB_EPTX_STAT_Msk, USB_EP_TX_VALID);
}

/**
 * For use in the event of an error, configure the endpoint so that when data is received ("OUT") always
 * STALL returns. Used by control endpoints to reject a request.
 */
void EPBuffer::transmitStall () {
	setEPnR (m_iBuffer, USB_EPTX_STAT_Msk, USB_EP_TX_STALL);
}

/**
 * Prepares to receive a single data packet. Configures the peripherals to receive.
 * After the transfer is complete, onReceive is called. "length" must be either <= 62 and multiples
 * be of 2, or <= 512 and multiple of 32. If length is 0, just an empty data packet
 * accepted (set via EP_KIND).
 */
void EPBuffer::receivePacket (size_t length) {
	if (length) {
		myassert (( ((length <= 62) && (length % 2) == 0)
				||	((length <= 512) && (length % 32 == 0)))
				&&	length <= m_rxBufLength);

		// The bytes to be sent are specified in the rxBufferCount register either in 2 bytes or 32 bytes
		// steps. Automatically determine the necessary size.
		const uint16_t BL_SIZE = !(length <= 62);
		const uint16_t NUM_BLOCK = BL_SIZE ? static_cast<uint16_t> ((length/32)-1) : static_cast<uint16_t> (length/2);

		// Configure buffers
		BufDescTable [m_iBuffer].rxBufferAddr = mapAddr (m_rxBuffer);
		BufDescTable [m_iBuffer].rxBufferCount = static_cast<uint16_t> ((BL_SIZE << 15) | (NUM_BLOCK << 10));

		// Reset EP_KIND since packet is greater than 0.
		setEPnR (m_iBuffer, USB_EPRX_STAT_Msk | USB_EP_KIND_Msk, USB_EP_RX_VALID);
	} else {
		BufDescTable [m_iBuffer].rxBufferCount = 0;
		// Set EP_KIND to only accept 0 packets
		setEPnR (m_iBuffer, USB_EPRX_STAT_Msk | USB_EP_KIND_Msk, USB_EP_RX_VALID | USB_EP_KIND);
	}
}

/**
 * To be called in the onReceive callback; copies the received data from the USB buffer to normal
 * Storage. length should be <= the rxBytes parameter of on Receive.
 */
void EPBuffer::getReceivedData (uint8_t* buffer, size_t length) {
	myassert (length <= 0xFFFF);
	// Iterate 16bit words in USB buffer memory, split into 2 bytes, and store in destination.
	for (uint_fast16_t i = 0; i < length/2; ++i) {
		uint16_t x = m_rxBuffer [i].data;
		buffer [i*2] = static_cast<uint8_t> (x & 0xFF);
		buffer [i*2+1] = static_cast<uint8_t> ((x>>8) & 0xFF);
	}
	// If size odd, transfer remaining byte
	if (length % 2) {
		buffer [length-1] = static_cast<uint8_t> (m_rxBuffer [length/2].data & 0xFF);
	}
}

/**
 * Called by USBPhys when USB peripherals are reset. Reconfigures the endpoint,
 * sends/receives nothing yet. Can be overwritten, but always has to be included
 * be called.
 */
void EPBuffer::onReset () {
	setEPnR (m_iBuffer, USB_EPRX_STAT_Msk | USB_EP_T_FIELD_Msk | USB_EPADDR_FIELD | USB_EP_KIND | USB_EPTX_STAT_Msk,
						static_cast<uint16_t> (USB_EP_RX_NAK | USB_EP_TX_NAK
						| (static_cast<uint16_t> (m_type) << USB_EP_T_FIELD_Pos)
						| (uint16_t { m_address } << USB_EPADDR_FIELD_Pos)));
}

/// Resets the protocol state machine.
void ControlEP::onReset () {
	EPBuffer::onReset ();

	m_data = nullptr; m_count = 0; m_remaining = 0;
	m_state = CTRL_STATE::SETUP;

	receiveControlPacket ();
}

/// Contains the receiving part of the state machine for control transfers.
void ControlEP::onReceive (bool setup, size_t rxBytes) {
	if (setup) {
		// SETUP packets should only appear when we are in the "SETUP" state (check
		// via assert), but to increase robustness we simply reset the state machine.
		myassert (m_state == CTRL_STATE::SETUP);
		m_data = nullptr; m_count = 0; m_remaining = 0;
		m_state = CTRL_STATE::SETUP;
		onSetupStage ();
	} else {
		switch (m_state) {
			case CTRL_STATE::STATUS_IN:
				// We received a 0 byte packet as confirmation of sent data

				// Go back to the beginning
				m_state = CTRL_STATE::SETUP;
				// Prepare the receive buffer again
				receiveControlPacket ();
				// Notify higher log level
				onStatusStage (true);
				break;
			case CTRL_STATE::DATA_OUT:
				// Part of a data block was received
				{
					// How many bytes can actually still be stored
					// (normally both should always be the same)
					size_t usableBytes = std::min (rxBytes, m_remaining);
					if (usableBytes) {
						// Save received data
						getReceivedData (m_data, usableBytes);
						m_remaining -= usableBytes;
						m_data += usableBytes;
					}
					// Empty or incomplete data packet terminates sequence
					if (rxBytes <= getRxBufLength ()) {
						onDataOut (m_count - m_remaining);
					} else {
						// Not done yet - calculate next buffer, prepare for reception
						receiveControlPacket ();
					}
				}
				break;
			default:
				myassert (false);
		}
	}
}

/// Contains the sending part of the state machine for control transfers.
void ControlEP::onTransmit () {
	switch (m_state) {
		case CTRL_STATE::DATA_IN:
			// Data packet sent
			size_t length;
			if (m_remaining == 0) {
				// Previous transfer sent the last data which exactly long enough
				// were for a package. Therefore, send a 0 packet to signal end
				length = 0;
				m_state = CTRL_STATE::DATA_IN_LAST;
			} else if (m_remaining < getRxBufLength ()) {
				// Data is shorter than packet size. send rest of data as last packet;
				// Host uses this to recognize the end of the block
				length = m_remaining;
				m_state = CTRL_STATE::DATA_IN_LAST;
			} else if (m_remaining == getRxBufLength ()) {
				// Remaining data fits exactly into one packet. Send off, and then a 0 packet
				length = m_remaining;
				m_remaining = 0;
			} else {
				// There is more data to be sent than can fit in one packet.
				// Send and run through this query again
				length = getRxBufLength ();
				m_remaining -= length;
			}
			transmitPacket (m_data, length);
			// Mark position in buffer
			m_data += length;
			break;
		case CTRL_STATE::DATA_IN_LAST:
			// Last package sent. Waiting for confirmation.
			m_state = CTRL_STATE::STATUS_IN;
			// Receive a 0 packet
			receivePacket (0);
			// Notify next level of log of successful transmission
			onDataIn ();
			break;
		case CTRL_STATE::STATUS_OUT:
			// A 0 packet was sent.
			// Start state machines from the beginning
			m_state = CTRL_STATE::SETUP;
			// Prepare the receive buffer again
			receiveControlPacket ();

			// Notify next level of protocol on transmission of status
			onStatusStage (false);
			break;
		default:
			myassert (false);
	}
}

/// Prepares to receive a packet on Control Endpoint.
void ControlEP::receiveControlPacket () {
	// Packets from control endpoints may have a maximum size of 64 bytes; receive as much as possible.
	receivePacket (std::min<size_t> (getRxBufLength (), 64));
}

/// To be called from the onSetupStage() callback. Prepares to send a block of data.
void ControlEP::dataInStage (const uint8_t* data, size_t length) {
	size_t pkLength;
	size_t bufSize = getTxBufLength ();
	if (length < bufSize) {
		// Only need to send 1 package
		m_state = CTRL_STATE::DATA_IN_LAST;
		m_remaining = 0;
		m_data = nullptr;
		pkLength = length;
	} else if (length == bufSize) {
		// Must send 1 packet and 0 packet to mark end
		m_state = CTRL_STATE::DATA_IN;
		m_remaining = 0;
		m_data = nullptr;
		pkLength = length;
	} else {
		// Must send multiple packages
		m_state = CTRL_STATE::DATA_IN;
		m_remaining = length - bufSize;
		// const_cast is not clean but not wrong, because when sending m_data
		// never written, so m_data can be used for both sending and receiving.
		m_data = const_cast<uint8_t*> (data) + bufSize;
		pkLength = bufSize;
	}
	// Send data
	transmitPacket (data, pkLength);
}

/// To be called from the onSetupStage() callback. Prepares to receive a block of data.
void ControlEP::dataOutStage (uint8_t* data, size_t length) {
	// Mark data
	m_state = CTRL_STATE::DATA_OUT;
	m_count = length;
	m_remaining = length;
	m_data = data;
	// Prepare reception
	receiveControlPacket ();
}

/**
 * To be called from one of the callbacks. Signals success to the host via empty packet or STALL
  * or failure of an operation.
 */
void ControlEP::statusStage (bool success) {
	if (success) {
		// Send 0 packet, wait for sending
		m_state = CTRL_STATE::STATUS_OUT;
		transmitPacket (nullptr, 0);
	} else {
		// Go straight to the beginning of the state machine
		m_state = CTRL_STATE::SETUP;
		// Send STALL
		transmitStall ();
		// Prepare the receive buffer again
		receiveControlPacket ();
	}
}

/// Resets the log.
void DefaultControlEP::onReset () {
	ControlEP::onReset ();
	// More than this variable cannot be reset here
	m_setAddress = 0;
}

/// Process standard requests on the default control endpoint 0.
void DefaultControlEP::onSetupStage () {
	// Query received data
	uint8_t pkBuffer [8];
	getReceivedData (pkBuffer, 8);
	// Assemble the individual numbers from raw data and store them in member variables to also
	// to be able to access them in the data stage callbacks.
	m_bmRequestType = pkBuffer [0];
	m_bRequest = pkBuffer [1];
	m_wValue = static_cast<uint16_t> (pkBuffer [2] | (uint16_t {pkBuffer [3]} << 8));
	m_wIndex = static_cast<uint16_t> (pkBuffer [4] | (uint16_t {pkBuffer [5]} << 8));
	m_wLength = static_cast<uint16_t> (pkBuffer [6] | (uint16_t {pkBuffer [7]} << 8));

	if (	// A standard USB request for a descriptor
			(m_bmRequestType == 0x80 && m_bRequest == ST_REQ::GET_DESCRIPTOR)
			// Or a Microsoft specific query of an OS string descriptor
	||		(m_bmRequestType == 0xC0 && m_bRequest == ST_REQ::GET_OS_STRING_DESC)
	) {
		// The type of descriptor is specified for standard queries, otherwise always use the OS string descriptor
		D_TYPE descType = m_bmRequestType == 0xC0 ? D_TYPE::OS_DESCRIPTOR : static_cast<D_TYPE> (m_wValue >> 8);
		// There is only 1 OS String Descriptor; for others use the desired index
		uint8_t descIndex = m_bmRequestType == 0xC0 ? 0 : static_cast<uint8_t> (m_wValue & 0xFF);

		// Search descriptor table
		const Descriptor* d = getUsbDescriptor (descType, descIndex);
		if (!d) {
			// No matching descriptor - sending error
			statusStage (false);
		} else {
			// Only send as much as requested. If descriptor is longer, the host will re-request the place the entire descriptor,
			// the total length of which is always at the very beginning and thus after the 1st request is known.
			uint16_t length = std::min<uint16_t> (m_wLength, d->length);

			// Send descriptor, possibly in several packets
			dataInStage (d->data, length);
		}
	} else if (m_bmRequestType == 0 && m_bRequest == ST_REQ::SET_ADDRESS) {
		// Assignment of a USB address.

		// Mark address; this is only set after the confirmation has been sent
		m_setAddress = static_cast<uint8_t> (m_wValue & 0x7F);

		// Send confirmation
		statusStage (true);
	} else if (m_bmRequestType == 0 && m_bRequest == ST_REQ::SET_CONFIGURATION) {
		// This device only supports 1 configuration. Simulate betting by accepting only one
		// and then nothing is done

		uint8_t conf = static_cast<uint8_t> (m_wValue & 0xFF);
		if (conf == 0) {
			// If configuration 0 is set, the device should reinitialize as if it had just received an initial address.
			// Nothing to do in this simple example.

			// Send confirmation.
			statusStage (true);
		} else if (conf == 1) {
			// Since there is only 1 configuration, no actual switching is necessary

			// With IN/OUT transfers, commands are transmitted alternately with DATA0/DATA1 in order to detect errors.
			// After setting a configuration, you should always continue with DATA0.

			// Therefore, set all non-control endpoints to "DATA0" here.
			m_phys.resetDataToggle ();

			// Send confirmation.
			statusStage (true);
		} else {
			// Unsupported configuration
			statusStage (false);
		}
	} else if (m_bmRequestType <= 2 && m_bRequest == ST_REQ::CLEAR_FEATURE) {
		// Features are not supported
		statusStage (false);
	} else if (m_bmRequestType <= 2 && m_bRequest == ST_REQ::SET_FEATURE) {
		// Features are not supported
		statusStage (false);
	} else if (m_bmRequestType >= 0x80 && m_bmRequestType <= 0x82 && m_bRequest == ST_REQ::GET_STATUS) {
		// Always answer 0 - device is not self-powered, does not support remote wakeup, and no halt feature.
		uint8_t buffer [2] = { 0, 0 };
		// 2 bytes always fit in 1 packet, so the buffer can be local
		dataInStage (buffer, 2);
	} else if (m_bmRequestType == 0x81 && m_bRequest == ST_REQ::GET_INTERFACE) {
		if (m_wValue == 0 && m_wIndex == 0 && m_wLength == 1) {
			// Device has only 1 interface. Always return 0.
			uint8_t buffer [1] = { 0 };
			// 1 byte always fits in 1 packet, so the buffer can be local
			dataInStage (buffer, 1);
		} else {
			// Unsupported operation
			statusStage (false);
		}
	} else if (m_bmRequestType == 1 && m_bRequest == ST_REQ::SET_INTERFACE) {
		// This simple device only supports 1 interface. Therefore, simulate the switching
		if (m_wValue == 0 && m_wIndex == 0 && m_wLength == 0) {
			// The only interface has been activated. Confirm it
			statusStage (true);
		} else {
			// Unsupported operation
			statusStage (false);
		}

	// From here, device/class-specific requests follow
	} else if (m_bmRequestType == 0xC0 && m_bRequest == 2) {
		// Query LED status.
		uint8_t data = static_cast<uint8_t> (LED1.getOutput () | (uint8_t { LED2.getOutput () } << 1));
		dataInStage (&data, 1);
	} else if (m_bmRequestType == 0x40 && m_bRequest == 1) {
		LED1.set (m_wValue & 1);
		LED2.set (m_wValue & 2);
		statusStage (true);
	} else {
		// Reject unknown requests
		statusStage (false);
	}
}

void DefaultControlEP::onDataOut (size_t) {
}

void DefaultControlEP::onDataIn () {
	// As for the implemented operations, there is nothing to do here
}

void DefaultControlEP::onStatusStage (bool in) {
	// Did we just send the confirmation for SET_ADDRESS?
	if (m_setAddress && !in) {
		// Only now accept the address (given by the specification)
		m_phys.setAddress (m_setAddress);
		// But only this time
		m_setAddress = 0;
	}
}
