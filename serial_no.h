/** @defgroup 04 serial_no Read_Serial_Number
 *
 * @ingroup infrastructure_apis
 *
 * @file serial_no.h Generates ST style and algoritms a serial number based on ST factory mask.
 *
 * @brief <b>Generates ST style and algoritms a serial number based on ST factory mask. Header file of serial_no.c.</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 01 September 2022
 *
 * This library supports to compute a serial number based on the Unique_ID
 * of the STM32F4 and STM32F1 series of ARM Cortex Microcontrollers by
 * ST Microelectronics.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 * This file is part of the MSX Keyboard Subsystem Emulator project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The device's unique id is used as the USB serial number string.
 */
//Use Tab width=2

#ifdef __cplusplus
extern "C" {
#endif

#if !defined SERIAL_NO_H
#define SERIAL_NO_H

#include "system.h"
#include "serial.h"

void serialno_read(uint8_t*);


#endif  //#if !defined SERIAL_NO_H

#ifdef __cplusplus
}
#endif
