/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

/* This file implements a the USB Communications Device Class - Abstract
 * Control Model (CDC-ACM) as defined in CDC PSTN subclass 1.2.
 *
 * The device's unique id is used as the USB serial number string.
 */
//Use Tab width=2

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/usb/usbd.h>


//extern usbd_device *usbdev;

void cdcacm_init(void);
void serialno_read(uint8_t*);
int aggregate_register_config_callback(usbd_device*, usbd_set_config_callback);
int aggregate_register_callback(usbd_device*, uint8_t, uint8_t, usbd_control_callback);

/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);

void usb_soft_connect(bool);
bool isUSBconnected(void);

#ifdef __cplusplus
}
#endif
