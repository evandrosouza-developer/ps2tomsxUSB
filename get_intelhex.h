/** @addtogroup 05 dbasemgt Database Management
 *
 * @file get_intelhex.h Database maintenance routines - Get IntelHex file from console.
 *
 * @brief <b>Database maintenance routines - Get IntelHex file from console. Header file of get_intelhex.c.</b>
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
 * This file was part of the libopencm3 project, but highly modifyed.
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


#if !defined GET_INTELHEX_TO_RAM_H
#define GET_INTELHEX_TO_RAM_H_H

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "system.h"
#include "ps2handl.h"
#include "serial.h"



/*entry point*/
/**
 * @ brief Gets from user the Intel Hex file to update the Database+
 * 
 * From USB cdcacm interface or UART (115200, 8, n, 1)
 * @param ram_buffer Pointer to where the ram_buffer_max_size bytes must be placed on.
 * @param ram_buffer_max_size 2560 bytes in this design.
 */
void get_intelhex_to_RAM(uint8_t *ram_buffer, uint16_t ram_buffer_max_size);

#ifdef __cplusplus
}
#endif

#endif  //#if !defined GET_INTELHEX_TO_RAM_H
