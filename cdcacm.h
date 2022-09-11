/*
 * This file is part of the PS/2 to MSX keyboard Converter and 
 * MSX Keyboard Subsystem Emulator projects, using libopencm3 project.
 *
 * Copyright (C) 2022 Evandro Souza <evandro.r.souza@gmail.com>
 *
 * Based on CDCACM example of:
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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

//Use Tab width=2


#if !defined CDCACM_H
#define CDCACM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "serial.h"


void cdcacm_init(void);
void set_nak_endpoint(uint8_t);
void clear_nak_endpoint(uint8_t);
void first_put_ring_content_onto_ep(struct sring*, uint8_t);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
void disable_usb(void);

#ifdef __cplusplus
}
#endif

#endif  //#if !defined CDCACM_H
