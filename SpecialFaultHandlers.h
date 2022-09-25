/** @defgroup 08 special_function_handlers Special Function Handlers
 * 
 * @ingroup Main_design_definitions
 *
 * @file SpecialFaultHandlers.h High Resolution Timer 1us delay and PS/2 clocks measurements routines.
 *
 * @brief <b>High Resolution Timer 1us delay and PS/2 clocks measurements routines. Header file of SpecialFaultHandlers.c.</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 25 September 2022
 *
 * This library executes functions to interface and control a PS/2 keyboard, like:
 * power control of a PS/2 key, general interface to read events and write commands to PS/2
 * keyboard, including interrupt service routines on the STM32F4 and STM32F1 series of ARM
 * Cortex Microcontrollers by ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 *
 * Copyright (C) 2021 Evandro Souza <evandro.r.souza@gmail.com>
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

#ifndef special_function_handlers_h
#define special_function_handlers_h

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/vector.h>
#include <stdio.h>
#include <stdint.h>

#include "serial.h"


#ifdef __cplusplus
}
#endif

#endif	//#ifndef special_function_handlers_h