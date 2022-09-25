/** @defgroup 03 msxmap MSX Interface Translator
 *
 * @ingroup MSX_specific_definitions
 *
 * @file msxmap.h Get a PS/2 key, translates it, put in keyboard interface buffer, dispatch smooth type buffer and process MSX interrupt of reading colunms.
 *
 * @brief <b>Get a PS/2 key, translates it, put in keyboard interface buffer, dispatch smooth type buffer and process MSX interrupt of reading colunms. Header file of msxmap.cpp.</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 25 September 2022
 *
 * This library executes functions to get a PS/2 key, translates it, put
 * in keyboard interface buffer, dispatch smooth type buffer and process
 * MSX interrupt of reading colunms on the STM32F4 and STM32F1 series of
 * ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 * This file is part of the PS/2 to MSX Keyboard converter enviroment:
 * PS/2 to MSX keyboard Converter and MSX Keyboard Subsystem Emulator
 * designs, based on libopencm3 project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
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

#ifndef msxmap_h
#define msxmap_h

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

#include "system.h"
#include "ps2handl.h"
#include "serial.h"
#include "dbasemgt.h"

//Use Tab width=2


class msxmap
{
private:


public:
	/**
   * Setup the interface to connect MSX
  */
  void msx_interface_setup(void);
  
  /**
   * Implement a smooth typing.
  */
	void msxqueuekeys(void);
  
  /**
   * Get the PS/2 event and do a pre-filter
  */
	void convert2msx(void);
  
  /**
   * Convert from PS/2 event to MSX keyboard matrix
  */
	void msx_dispatch(void);
  
  /**
   * Check the availability MSX key in buffer, to implement a smooth typing.
   *
   * The usage is to put a byte key (bit 7:4 represents Y, bit 3 is the release=1/press=0, bits 2:0 are the X )
   * F = 15Hz. This routine is called from Ticks interrupt routine
  */
	bool available_msx_disp_keys_queue_buffer(void);

  /**
   * Put a MSX key in buffer, to implement a smooth typing.
  */
	uint8_t put_msx_disp_keys_queue_buffer(uint8_t);

  /**
   * Get a MSX key in buffer, to implement a smooth typing.
  */
	uint8_t get_msx_disp_keys_queue_buffer(void);

  /**
   * Update memory and X port, according PS/2 keyboard event.
   *
   * y_local Which colunm you put the x_local_setb
   * x_local Which bit you put x_local_setb
   * x_local_setb 0 if keypress or 1 if key release
  */	void compute_x_bits_and_check_interrupt_stuck ( 
					volatile uint8_t y_local, volatile uint8_t x_local, volatile bool x_local_setb);
};


#endif
