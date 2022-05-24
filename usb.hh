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
 * This file contains definitions and classes for the USB protocol levels, especially for control transfers.
 */

#ifndef USB_HH_
#define USB_HH_

#include <cstddef>
#include <cstdint>
#include <array>
#include "util.hh"
#include "usbutils.hh"

/// Listing of the standard requests on the default control endpoint 0.
enum class ST_REQ : uint8_t { GET_STATUS = 0, CLEAR_FEATURE = 1, SET_FEATURE = 3, SET_ADDRESS = 5, GET_DESCRIPTOR = 6,	SET_DESCRIPTOR = 7,
	GET_CONFIGURATION = 8, SET_CONFIGURATION = 9, GET_INTERFACE = 10, SET_INTERFACE = 11, SYNCH_FRAME = 12, GET_OS_STRING_DESC = 3 };

/// List some USB CDC requests
enum class CDC_REQ : uint8_t { SEND_ENCAPSULATED_COMMAND = 0, GET_ENCAPSULATED_RESPONSE = 1, SET_LINE_CODING = 0x20, GET_LINE_CODING = 0x21, SET_CONTROL_LINE_STATE = 0x22 };

/// List of standard features.
enum class FEATURE : uint8_t { DEVICE_REMOTE_WAKEUP = 1, ENDPOINT_HALT = 0, TEST_MODE = 2 };

/// States of the state machine for control endpoints (class ControlEP)
enum class CTRL_STATE : uint8_t { SETUP, DATA_OUT, DATA_IN, DATA_IN_LAST, STATUS_IN, STATUS_OUT };

/// Collection of endpoint types, for the EPBuffer constructor. The values correspond to those of the EPnR register
enum class EP_TYPE : uint8_t { BULK = 0, CONTROL = 1, ISOCHRONOUS = 2, INTERRUPT = 3 };

// Allow direct comparison of integer type commands

usb_always_inline constexpr bool operator == (uint8_t a, ST_REQ b) { return a == static_cast<uint8_t> (b); }
usb_always_inline constexpr bool operator == (uint8_t a, CDC_REQ b) { return a == static_cast<uint8_t> (b); }

class EPBuffer;
class DefaultControlEP;

/**
 * Represents the USB peripherals together with EPBuffer. Provides some global operations on USB peripherals.
 * An array of pointers to individual EPBuffer instances is passed to the constructor; USBPhys calls in it the
 * corresponding callbacks (onReset, onReceive, onTransmit).
 */
class USBPhys {
	friend class DefaultControlEP;
	public:
		constexpr USBPhys (std::array<EPBuffer*, numEP> epBuffers) : m_epBuffers (epBuffers) {}

		// Disallow copy/assign

		USBPhys (const USBPhys&) = delete;
		USBPhys (USBPhys&&) = delete;
		USBPhys& operator = (const USBPhys&) = delete;
		USBPhys& operator = (USBPhys&&) = delete;

		void init ();
		void connect ();
		void disconnect ();

		void irq ();
	private:
		void resetDataToggle ();
		void setAddress (uint8_t addr);
		void initializeDevice ();

		/// Memorizes the pointers to each EP buffer.
		const std::array<EPBuffer*, numEP> m_epBuffers;
};

/**
 * Represents a peripheral endpoint buffer. One such corresponds to 0-2 endpoints
 * the bus (IN/OUT with the respective address). Both endpoints are of the same type. buffer
 * No. "n" consists of the EPnR register and BufDescTable[n] - this class provides the bottom line
 * an API to access these two.
 *
 * The order of the buffers does not necessarily correspond to the order of the endpoints
 * on the bus (from the host perspective); For example, buffer 0 can process endpoint 0x7 (OUT) and 0x87 (IN).
 * while buffer 4 handles endpoint 0x0 (OUT) and 0x80 (IN). In the constructor, in iBuffer
 * the index of the buffer (0-7) and the address of the endpoint (0-15) are specified. The IN address
 * is automatically formed from this by the peripherals using "addr|0x80".
 *
 * Pointers to the buffers in the USB memory should be specified in rxBuffer and txBuffer, i.e.
 * on global variables marked USB_MEM. Will never be data in both directions
 * transmitted simultaneously (i.e. no call to transmitPacket between receivePacket & onReceive,
 * and no call to receivePacket between transmitPacket & onTransmit) can use both pointers
 * be the same ("half duplex"). */
class EPBuffer {
	friend class USBPhys;
	public:
		constexpr EPBuffer (uint8_t iBuffer, uint8_t addr, EP_TYPE type, UsbMem* rxBuffer, size_t rxBufLength, UsbMem* txBuffer, size_t txBufLength)
			: m_rxBuffer (rxBuffer), m_txBuffer (txBuffer), m_rxBufLength (rxBufLength), m_txBufLength (txBufLength),
			  m_iBuffer (iBuffer), m_address (addr), m_type (type) {}

		// Disallow copy/assign

		EPBuffer (const EPBuffer&) = delete;
		EPBuffer (EPBuffer&&) = delete;
		EPBuffer& operator = (const EPBuffer&) = delete;
		EPBuffer& operator = (EPBuffer&&) = delete;

		void transmitPacket (const uint8_t* data, size_t length);
		void transmitStall ();
		void receivePacket (size_t length);
		void getReceivedData (uint8_t* buffer, size_t length);

		/// Get the size of the included receive buffer, in bytes.
		size_t getRxBufLength () const { return m_rxBufLength; }
		/// Get the size of the included send buffer, in bytes.
		size_t getTxBufLength () const { return m_txBufLength; }
	protected:
		virtual void onReset ();
		/**
		 * Called by USBPhys when data has been received on this buffer.
		 * "setup" indicates whether it was a SETUP transfer and rxBytes the count
		 * bytes received.
		 */
		virtual void onReceive (bool setup, size_t rxBytes) = 0;
		/// Called by USBPhys when data has been sent from this buffer.
		virtual void onTransmit () = 0;
	private:
		/// Saves receive resp. Send buffer.
		UsbMem* const m_rxBuffer, *const m_txBuffer;
		/// Stores length of both buffers.
		const size_t m_rxBufLength, m_txBufLength;
		/// Stores index of buffer, i.e. number of EPnR register and BufDescTable entry.
		const uint8_t m_iBuffer;
		/// Stores bus addresses of endpoints assigned to this buffer.
		const uint8_t m_address;
		/// Stores the type of endpoints.
		const EP_TYPE m_type;
};

/**
 * Implements the protocol for control transfers, e.g. for the default control endpoint 0. Control transfers
 * consist of the steps "Setup", "Data" (optional), and "Status". With "Setup" an initial data packet
 * sent by the host, and the onSetupStage() callback called. This can then be sent via dataInStage/dataOutStage
 * initiate the "Data" stage, or directly the status stage via "statusStage". The "Data" stage is defined by the
 * Callbacks onDataIn or onDataOut acknowledged where statusStage has to be called. The "Status" stage will
 * acknowledged by onStatusStage, and it is automatically switched to the next "Setup" stage.
 * The implementation enables the transmission of larger data blocks that do not fit into one data packet.
 * Since control endpoints work "half-duplex", only one buffer is required.
 */
class ControlEP : public EPBuffer {
	public:
		constexpr ControlEP (uint8_t iBuffer, uint8_t addr, UsbMem* buffer, size_t bufLength)
			: EPBuffer (iBuffer, addr, EP_TYPE::CONTROL, buffer, bufLength, buffer, bufLength), m_data (nullptr), m_count (0), m_remaining (0), m_state (CTRL_STATE::SETUP) {}

		// Disallow copy/assign

		ControlEP (const ControlEP&) = delete;
		ControlEP (ControlEP&&) = delete;
		ControlEP& operator = (const ControlEP&) = delete;
		ControlEP& operator = (ControlEP&&) = delete;

		void dataInStage (const uint8_t* data, size_t length);
		void dataOutStage (uint8_t* data, size_t length);
		void statusStage (bool success);

	protected:
		/**
		 * Called after receiving a "setup" packet from the host. Should dataInStage,
		 * Call dataOutStage or statusStage.
		 */
		virtual void onSetupStage () = 0;
		/**
		 * Called when all data has been received from the host in the data stage.
		 * "rxbytes" indicates the number of bytes received. Should call statusStage.
		 */
		virtual void onDataOut (size_t rxBytes) = 0;
		/**
		 * Called when all data has been sent to the host in the data stage.
		 * Since "In" transfers do not signal success, here NOT call statusStage.
		 * Can therefore be left blank.
		 */
		virtual void onDataIn () = 0;
		/**
		 * Called when an empty data packet to signal success
		 * was sent (for out-transfers or those without data stage - in=false),
		 * or an empty packet was received (for in-transfers - in=true).
		 */
		virtual void onStatusStage (bool in) = 0;

		virtual void onReset () override;
		virtual void onReceive (bool setup, size_t rxBytes) override final;
		virtual void onTransmit () override final;
	private:
		void receiveControlPacket ();
		/// Data buffer for data to be sent/received.
		uint8_t* m_data;
		/// Total number of bytes to be transferred
		size_t m_count;
		/// Remaining number of bytes to transfer
		size_t m_remaining;
		/// Current state of the state machine
		CTRL_STATE m_state;
};

/**
 * Uses the protocol for Control Endpoints to respond to Default Requests on the Default Control
 * Endpoint to respond. Always uses endpoint address 0. Has no external API; the processing
 * own or class-specific requests can be added here.
 */
class DefaultControlEP : public ControlEP {
	public:
		constexpr DefaultControlEP (USBPhys& phys, uint8_t iBuffer, UsbMem* buffer, size_t bufLength)
			: ControlEP (iBuffer, 0, buffer, bufLength), m_phys (phys), m_wValue (0), m_wIndex (0), m_wLength (0),
			  m_setAddress (0), m_bmRequestType (0), m_bRequest (0) {}

		// Disallow copy/assign

		DefaultControlEP (const DefaultControlEP&) = delete;
		DefaultControlEP (DefaultControlEP&&) = delete;
		DefaultControlEP& operator = (const DefaultControlEP&) = delete;
		DefaultControlEP& operator = (DefaultControlEP&&) = delete;
	protected:
		virtual void onReset () override final;
		virtual void onSetupStage () override final;
		virtual void onDataOut (size_t rxBytes) override final;
		virtual void onDataIn () override final;
		virtual void onStatusStage (bool in) override final;
	private:
		/// Associated USBPhys instance.
		USBPhys& m_phys;
		/// Is filled with the data from the request in onSetupStage, saves the data for further callbacks.
		uint16_t m_wValue, m_wIndex, m_wLength;

		/**
		 * When the host assigns an address, it is first stored in this variable. In the
		 * Status stage, the address is written to the peripherals and the variable is reset to 0.
		 * According to the specification, the address must not be accepted immediately, but only after confirmation.
		 */
		uint8_t m_setAddress;

		/// Is filled with the data from the request in onSetupStage, saves the data for further callbacks.
		uint8_t m_bmRequestType, m_bRequest;
};

#endif /* USB_HH_ */
