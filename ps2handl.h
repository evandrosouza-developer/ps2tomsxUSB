/** @defgroup 04 ps2handl PS/2 Keyboard Interface Handler
 *
 * @file ps2handl.h Power Control of a PS/2 key and general interface to and from PS/2 keyboard, inlcuding interrupt routines.
 *
 * @brief <b>Power Control of a PS/2 key and general interface to and from PS/2 keyboard, inlcuding interrupt routines. Header file of ps2handl.c.</b>
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

#ifndef ps2handl_h
#define ps2handl_h

#ifdef __cplusplus
extern "C" {
#endif


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/scb.h>

#include "system.h"
#include "serial.h"
#include "hr_timer.h"


#define PS2_RECV_BUFFER_SIZE_POWER 6	//64 uint8_t positions
#define PS2_RECV_BUFFER_SIZE 64				//(2 << PS2_RECV_BUFFER_SIZE_POWER)

//State definitions of PS/2 clock interrupt machine
enum ps2int{
	PS2INT_RECEIVE =												0x400,
	PS2INT_SEND_COMMAND,
	PS2INT_WAIT_FOR_COMMAND_ACK,
	PS2INT_SEND_ARGUMENT,
	PS2INT_WAIT_FOR_ARGUMENT_ACK,
	PS2INT_WAIT_FOR_ECHO
};

/**
 * @brief Turn on the 5V to power the PS/2 keyboard on
 */
void power_on_ps2_keyboard(void);

/**
 * @brief Turn off the 5V to power the PS/2 keyboard off
 */
void power_off_ps2_keyboard(void);

/**
 * @brief Perform a software reset
 */
void reset_requested(void);

/**
 * @brief Detect a connected PS/2 Keyboard
 *
 * @return True if keyboard is connected and responds to POST and ID.
 */
bool ps2_keyb_detect(void);

/**
 * @brief Enter point of PS/2 clock line, called from interrupt handled by msxhid
 * 
 * @param ps2datapin_logicstate what has been read from ps2_data pin
 */
void ps2_clock_update(bool ps2datapin_logicstate);

/**
 * @brief Enter point of PS/2 clock line, called from interrupt handled by msxhid
 * 
 * @param num Desired state of PS/2 Number Lock led: True for turn on)
 * @param caps Desired state of PS/2 Caps Lock led: True for turn on)
 * @param scroll Desired state of PS/2 Scroll Lock led: True for turn on)
 */
void ps2_update_leds(bool num, bool caps, bool scroll);

/**
 * @brief Checks if PS/2 Keyboard is responding to echo command
 * 
 */
bool keyboard_check_alive(void);

/**
 * @brief Checks if PS/2 Keyboard has sent an event. responding to echo command
 * 
 * @return true if decoded event and the decoded event is stored at scancode
 */
bool mount_scancode(void);

/**
 * @brief Setup debug signaling pins
 * 
 */
void general_debug_setup(void);

/**
 * @brief Setup unused pins to avoid floating, so minimizing current consumption.
 * 
 */
void put_pullups_on_non_used_pins(void);

#ifdef __cplusplus
}
#endif

#endif	//#ifndef ps2handl_h