/** @addtogroup 01 Manager_group
 *
 * @ingroup infrastructure_apis
 *
 * @file newlib_warning_fix.c USB Support routines group on STM32F4 and STM32F1
 *
 * @brief <b>Main code. Support routines group on STM32F4 and STM32F1</b>
 *
 * @version 1.0.0
 *
 * @author @htmlonly &copy; @endhtmlonly 2022
 * Evandro Souza <evandro.r.souza@gmail.com>
 *
 * @date 02 December 2022
 *
 * This library supports the USB on the STM32F4 and STM32F1
 * series of ARM Cortex Microcontrollers by ST Microelectronics.
 *
 * This file fixes the warnings of reentrant calls, due to modifications
 * since August 8th, 2022, on version 11.3 Rel1.
 * In this code, I do NOT use any functions of the standard library (NEWLIB).
 * Although these reentrant functions are not used, they started to be
 * requested by gcc, in NEWLIB requests, and, in their absence, generate
 * warnings of "not implemented and will always fail" on reentrant calls
 * close, lseek, read, kill and getpid.
 *
 * LGPL License Terms ref lgpl_license
 */

/*
 * This file is part of the PS/2 to MSX keyboard Converter and 
 * MSX Keyboard Subsystem Emulator projects, based on libopencm3 project.
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

#ifdef __cplusplus
extern "C" {
#endif

#include "newlib_warning_fix.h"

void _close() {};
void _lseek() {};
void _read()  {};
void _getpid(){};
void _kill()  {};

#ifdef __cplusplus
}
#endif  //#ifdef __cplusplus
