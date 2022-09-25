/** @addtogroup 05 dbasemgt Database Management
 *
 * @file dbasemgt.h Database check and maintenance routines.
 *
 * @brief <b>Database check and maintenance routines. Header file of dbasemgt.c.</b>
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
 * This file is part of the PS/2 to MSX keyboard Converter and 
 * MSX Keyboard Subsystem Emulator projects, using libopencm3 project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * This original SW is compiled to a Sharp/Epcom MSX HB-8000 and a brazilian ABNT2 PS/2 keyboard (ID=275)
 * But it is possible to update the table sending a Intel Hex File through serial or USB
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


#if !defined DBASEMGT_H
#define DBASEMGT_H

#ifdef __cplusplus
extern "C" {
#endif

/*entry point*/
/** 
 * @brief Inits Database update process
 *
 * @return RESULT_OK: everything ok or FLASH_WRONG_DATA_WRITTEN: data read from Flash is different than written data
 */
int flash_rw(void);

/** 
 * @brief Checks Database consistensy
 *
 */
void database_setup(void);

#ifdef __cplusplus
}
#endif

#endif	//#if !defined DBASEMGT_H
