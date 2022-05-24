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
 * This file contains the implementations of helper functions for accessing the USB peripherals.
 */

#include <cstdint>
#include <cstddef>
#include <stm32f1xx.h>
#include "usbutils.hh"

/**
 * The Buffer Descriptor Table is stored together with the buffer memory for the endpoints in the USB-RAM discarded.
 * The exact position is arbitrary and is specified in the USB_BTABLE register.
 */
alignas(8) EP_BufDesc BufDescTable [8] USB_MEM;

/**
 * Because the individual bits of the EPnR registers have to be set in different ways and when writing
 * care must be taken that the wrong bits are not accidentally written,
 * This function encapsulates write access. The EP parameter specifies the index of the register. All
 * Bits to be written (regardless of value) must be set to 1 in "mask";
 * if mask=0, nothing is written at all. The actual values to be written are stored in "data"
 * specified. Bits which are 0 in "mask" are ignored in "data". In "old" becomes the previous one
 * Pass the status of the register if it has already been queried beforehand. Isn't it
 * is the case, the overloaded function can be used without this parameter.
 */
void setEPnR (uint8_t EP, uint16_t mask, uint16_t data, uint16_t old) {
	myassert (EP < numEP);

	// These bits are cleared when writing 0 and remain unchanged when 1.
	constexpr uint16_t rc_w0 = USB_EP_CTR_RX_Msk | USB_EP_CTR_TX_Msk;
	// These bits are toggled on writing from 1, and remain unchanged on 0.
	constexpr uint16_t toggle = USB_EP_DTOG_RX_Msk | USB_EPRX_STAT_Msk | USB_EP_DTOG_TX_Msk | USB_EPTX_STAT_Msk;
	// These bits behave "normally", i.e. the written value is accepted directly.
	constexpr uint16_t rw = USB_EP_T_FIELD_Msk | USB_EP_KIND_Msk | USB_EPADDR_FIELD;

	// Check bits to clear
	uint16_t wr0 = static_cast<uint16_t> (rc_w0 & (~mask | data));
	// For bits with toggle behavior, the old state must be taken into account and processed using XOR
	uint16_t wr1 = (mask & toggle) & (old ^ data);
	// With "normal" bits, the old state is retained or overwritten if desired.
	uint16_t wr2 = rw & ((old & ~mask) | data);

	// Combine all three writing methods.
	EPnR[EP].data = static_cast<uint16_t> (wr0 | wr1 | wr2);
}

/**
 * This overload can be used if the current state of EPnR has not yet been queried;
 * in this case, the function queries the state and passes it to the "old" parameter.
 */
void setEPnR (uint8_t EP, uint16_t mask, uint16_t data) {
	setEPnR (EP, mask, data, EPnR [EP].data);
}
