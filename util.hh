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
 * Various auxiliary functions are defined here.
 */

#ifndef UTIL_HH_
#define UTIL_HH_

#include <stm32f1xx.h>

/// Functions marked with usb always_inline are always inlined
#define usb_always_inline __attribute__((always_inline)) inline

// Define your own myassert() analogous to standard assert(), which does not call printf but only causes a breakpoint.

#ifdef NDEBUG
#	define myassert(cond) static_cast<void>(cond)
#else
#	define myassert(cond) ((cond) ? static_cast<void>(0) : assert_failed ())
#endif

void assert_failed ();

/// Calculates the distance between two integers correctly, even if they are unsigned.
template <typename T>
usb_always_inline constexpr T distance (T a, T b) {
	return (a < b) ? (b - a) : (a - b);
}

/// Represents a GPIO pin&port. Can be used to pass desired pins to classes/functions.
class Pin {
	public:
		/**
		 * Creates a pin instance; iPort & iPin start at 0 and indicate the number of Port & Pin.
		 * Pin { 1, 7 } corresponds to e.g. PB7.
		 */
		constexpr Pin (uint8_t iPort, uint8_t iPin) : m_iPort (iPort), m_iPin (iPin) {}

		void initClock () const;
		void configureOutput () const;
		void configureAFOut () const;
		void setMode (uint8_t mode) const;
		void set (bool value) const;
		bool getOutput () const;
	private:
		GPIO_TypeDef* port () const;
		__IO uint32_t& CRx () const;

		uint8_t m_iPort, m_iPin;
};

#endif /* UTIL_HH_ */
