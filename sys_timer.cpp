/** @defgroup 06 sys_timer System Timer and smooth typing
 *
 * @ingroup infrastructure_apis
 *
 * @file sys_timer.cpp System Timer and smooth typing routines.
 *
 * @brief <b>System Timer and smooth typing routines.</b>
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

#include "sys_timer.h"
//Use Tab width=2


//Global vars
volatile uint32_t systicks, ticks;
volatile uint16_t ticks_keys;
volatile uint16_t last_ps2_fails=0;
volatile uint16_t fail_count;
extern bool ps2_keyb_detected;										//Declared on ps2handl.c
extern bool update_ps2_leds, ps2numlockstate;			//Declared on msxmap.cpp


void systick_setup(void)
{
	//Make sure systick doesn't interrupt PS/2 protocol bitbang action
	nvic_set_priority(NVIC_SYSTICK_IRQ, IRQ_PRI_SYSTICK);

	/* 84MHz / 8 => 10500000 counts per second */
	/* 10500000 / 30(ints per second) = 350000 systick reload - every 33,3ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(((rcc_apb2_frequency * 2) / FREQ_INT_SYSTICK) - 1);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	
	systicks = 0; //systick_clear
	ticks = 0;
	ticks_keys = 0;

	/* Start counting. */
	systick_counter_enable();

	systick_interrupt_enable();

	// GPIO C13 is the onboard LED
#if MCU == STM32F103
	gpio_set_mode(EMBEDDED_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, EMBEDDED_LED_PIN);
	// Enable the led. It is active LOW, but the instruction was omitted, since 0 is the default.
#endif//#if MCU == STM32F103
#if MCU == STM32F401
	gpio_mode_setup(EMBEDDED_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EMBEDDED_LED_PIN);
	gpio_set_output_options(EMBEDDED_LED_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, EMBEDDED_LED_PIN);
	// Enable the led. It is active LOW, but the instruction was omitted, since 0 is the default.
#endif//#if MCU == STM32F401
}

/*************************************************************************************************/
/*************************************************************************************************/
/******************************************* ISR's ***********************************************/
/*************************************************************************************************/
/*************************************************************************************************/
void sys_tick_handler(void) // f=30Hz (Each 33,33ms)
{
	iwdg_reset();	//Prevent from reset

	systicks++;

	ticks++;
	if( (ps2_keyb_detected == true) && (ticks >= (10*3)) )
	{ /*C13 LED blinks each 2s (2*1 s) on keyboard detected */
		gpio_toggle(EMBEDDED_LED_PORT, EMBEDDED_LED_PIN);
		ticks=0;
	} else if( (ps2_keyb_detected == false) && (ticks >= 7) )
	{ /*C13 LED blinks each 467 ms (2*233.3 ms) on keyboard absence*/
		gpio_toggle(EMBEDDED_LED_PORT, EMBEDDED_LED_PIN);
		ticks=0;
	}

	if(ps2_keyb_detected == true)
	{
		//Queue keys processing:
		ticks_keys = (ticks_keys++ & (MAX_TICKS_KEYS - 1));
		if (!ticks_keys)
		{
			msxmap objeto;
			objeto.msxqueuekeys();
		}
	}

	if(fail_count!=last_ps2_fails)
	{
		// printf("PS/2 failure count: %03d\r\n", fail_count);
		/*uint8_t mountstring[3];
		usart_send_string((uint8_t*)"PS/2 failure count: ");
		conv_uint8_to_2a_hex(fail_count, mountstring);
		usart_send_string(mountstring);
		usart_send_string((uint8_t*)"\r\n");*/

		last_ps2_fails=fail_count;
	}
}
