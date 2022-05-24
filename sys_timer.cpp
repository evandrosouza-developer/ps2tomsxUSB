#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/iwdg.h>

#include "sys_timer.h"
#include "serial.h"
#include "hr_timer.h"
#include "ps2handl.h"
#include "msxmap.h"
#include "system.h"

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
	gpio_mode_setup(EMBEDDED_BLUE_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EMBEDDED_BLUE_LED_PIN);
	gpio_set_output_options(EMBEDDED_BLUE_LED_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, EMBEDDED_BLUE_LED_PIN);
	
	// Enable the led. It is active LOW, but the instruction was omitted, since 0 is the default.
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
	if(ps2_keyb_detected==true && (ticks>=(10*3)))
	{ /*C13 LED blinks each 2s (2*1 s) on keyboard detected */
		gpio_toggle(EMBEDDED_BLUE_LED_PORT, EMBEDDED_BLUE_LED_PIN);
		ticks=0;
	} else if (ps2_keyb_detected==false && ticks >= 7)
	{ /*C13 LED blinks each 467 ms (2*233.3 ms) on keyboard absence*/
		gpio_toggle(EMBEDDED_BLUE_LED_PORT, EMBEDDED_BLUE_LED_PIN);
		ticks=0;
	}

	if (ps2_keyb_detected == true)
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
		conv_uint8_to_2a_hex(fail_count, &mountstring[0]);
		usart_send_string(&mountstring[0]);
		usart_send_string((uint8_t*)"\r\n");*/

		last_ps2_fails=fail_count;
	}
}
