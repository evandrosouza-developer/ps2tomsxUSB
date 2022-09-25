/** @defgroup 07 hr_timer High Resolution Timer
 *
 * @file hr_timer.h High Resolution Timer 1us delay and PS/2 clocks measurements routines.
 *
 * @brief <b>High Resolution Timer 1us delay and PS/2 clocks measurements routines. Header file of hr_timer.c.</b>
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
 * This file is part of PS/2 to MSX keyboard adapter
 * This file was part of the libopencm3 project.
 *
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
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


#ifndef hr_timer_h
#define hr_timer_h

#ifdef __cplusplus
extern "C" {
#endif

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>

#include "system.h"
#if (USE_USB == true)
#include "cdcacm.h"
#endif	//#if (USE_USB == true)
#include "ps2handl.h"


/**
 * @brief Sets up the High Resolution timer.
 * 
 * @param timer_peripheral Timer to be used.
 */
void tim_hr_setup(uint32_t timer_peripheral);

/**
 * @brief Inserts a delay with a resolution of a microsecond and call the desired function.
 *
 * @param timer_peripheral Timer to be used.
 * @param qusec delay (in microseconds).
 * @param next_step pointer of the desired function to be called after time is up.
 */
void delay_usec(uint32_t timer_peripheral, uint16_t qusec, void next_step (void));

/**
 * @brief Starts the Timer to be used to make a time capture.
 * 
 * @param timer_peripheral Timer to be used.
 */
void prepares_capture(uint32_t timer_peripheral);


#ifdef __cplusplus
}
#endif	//#ifndef hr_timer_h


#endif	//#ifndef hr_timer_h
