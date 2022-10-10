/** @addtogroup 04 serial_no Read_Serial_Number
 *
 * @ingroup infrastructure_apis
 *
 * @file serial_no.c Generates ST style and algoritms a serial number based on ST factory mask.
 *
 * @brief <b>Generates ST style and algoritms a serial number based on ST factory mask.</b>
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

/*#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>*/

#include "serial_no.h"


//Global variables
uint8_t serial_no[LEN_SERIAL_No + 1];


void serialno_read(uint8_t *s)
{
  // Fetch serial number from chip's unique ID
  // Use the same serial number as the ST DFU Bootloader.
# if (MCU == STM32F401)
  uint16_t *uid_p = (uint16_t *)DESIG_UNIQUE_ID_BASE;
#define OFFSET 3

  uint64_t unique_id =  ((uint64_t)(uid_p[1] + uid_p[5]) << 32) +
                        ((uint64_t)(uid_p[0] + uid_p[4]) << 16) +
                        uid_p[OFFSET];
# endif
  
# if (MCU == STM32F103)
  uint32_t *uid_p = (uint32_t *)DESIG_UNIQUE_ID_BASE;
  uint32_t unique_id = uid_p[0] + uid_p[1] + uid_p[2];
# endif

  //Convert each nibble to ASCII HEX
  uint16_t i, ii;
  for(i = 0; i < LEN_SERIAL_No; i++)
  {
    ii = LEN_SERIAL_No-1-i;
    #if defined CHECK_INDEX
    check_idx_u16(ii, (uintptr_t)s, LEN_SERIAL_No);
    #endif
    s[ii] = ((unique_id >> (4*i)) & 0xF) + '0';
    if(s[ii] > '9')
      s[ii] += 'A' - '9' - 1;
  }
  s[LEN_SERIAL_No] = 0;

  return; //s
}
